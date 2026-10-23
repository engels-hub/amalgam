//! Sketch transform operations (SK-18): mirror, rectangular and circular pattern, and
//! move / copy / rotate / scale.
//!
//! All operations take a selection of entities (points, lines, circles, arcs, ellipses, splines
//! and conics). A selected
//! curve brings its points along, so `items` may list only the curves. They report what they
//! changed in an [`EditResult`] and are atomic: on an error the entities and constraints
//! created so far are removed again, so a failed call leaves the sketch's geometry unchanged
//! (only the revision counters and retired parameter slots differ).
//!
//! # Constraints
//!
//! *Copies* ([`transform`] with `copy = true`, patterns and mirror with `constrain = false`)
//! are independent of their source: they get copies of the constraints and dimensions that
//! lie entirely inside the selection (dimension values are scaled by the transform's scale),
//! and nothing else. A constraint is only copied if its residual is zero on the copy, so
//! translation-variant ones (`Fix`) and ones the transform breaks are dropped. Orientation
//! flags (`Side`, `TangentKind`, `AngleBranch`) are captured again from the copy's geometry.
//!
//! *Constrained* results (`constrain = true`) are tied to the source, so editing the source
//! moves them, and they add no degrees of freedom:
//!
//! - **Mirror** adds one `Symmetric` constraint per point pair and an `Equal` per circle. A
//!   source point on the axis is shared with its image; it gets a `PointOnCurve` on the axis
//!   (unless it is an axis end point or already has one). Nothing else is copied.
//! - **Rectangular pattern** adds, per source point and instance, a horizontal and a vertical
//!   distance (a `Vertical`/`Horizontal` constraint where the offset is zero) between the
//!   point and its image, and an `Equal` per circle.
//! - **Circular pattern** adds, per source point and instance, construction lines from the
//!   centre to the point and to its image, an `Equal` between them and an `Angle` dimension
//!   (the instance's rotation), and an `Equal` per circle.
//!
//! The pattern links are ordinary constraints: no new constraint kind is needed.

use std::collections::{BTreeMap, BTreeSet};

use amg_core::{Point2, Tolerance, Transform2, Vec2};

use crate::error::SketchError;
use crate::ids::{ConstraintId, EntityId};
use crate::model::{
    AngleBranch, ConstraintKind, Dimension, DimensionKind, End, EntityKind, EntityOrigin, HvTarget,
    ParamFlags, Quantity, Side, Sketch, SplineDef,
};
use crate::solve::orient_constraint;
use crate::solve::{candidate_max_residual, characteristic_length, fixed_params};

const TOL: Tolerance = Tolerance::DEFAULT;

/// What an edit operation changed (shared with trim/extend/break).
pub use crate::edit::EditResult;

// ---- Transform constructors ---------------------------------------------------------------

/// Translation by `v`.
pub fn translation(v: Vec2) -> Transform2 {
    Transform2::from_translation(v)
}

/// Counter-clockwise rotation by `angle` (rad) about `center`.
pub fn rotation_about(center: Point2, angle: f64) -> Transform2 {
    Transform2::from_translation(center)
        * Transform2::from_angle(angle)
        * Transform2::from_translation(-center)
}

/// Uniform scaling by `factor` about `center`.
pub fn scale_about(center: Point2, factor: f64) -> Transform2 {
    Transform2::from_translation(center)
        * Transform2::from_scale(Vec2::splat(factor))
        * Transform2::from_translation(-center)
}

// ---- Plumbing -----------------------------------------------------------------------------

/// The selected entities.
struct Selection {
    /// Selected points and the points of selected curves.
    points: BTreeSet<EntityId>,
    /// Selected lines, circles and arcs.
    curves: Vec<EntityId>,
    /// Both of the above.
    all: BTreeSet<EntityId>,
    /// The selected arcs.
    arcs: BTreeSet<EntityId>,
}

fn select(s: &Sketch, items: &[EntityId]) -> Result<Selection, SketchError> {
    if items.is_empty() {
        return Err(SketchError::NotApplicable("nothing is selected"));
    }
    let ids: BTreeSet<EntityId> = items.iter().copied().collect();
    let mut sel = Selection {
        points: BTreeSet::new(),
        curves: Vec::new(),
        all: BTreeSet::new(),
        arcs: BTreeSet::new(),
    };
    for id in ids {
        let e = s.entity(id).ok_or(SketchError::UnknownEntity(id))?;
        match &e.kind {
            EntityKind::Point { .. } => {
                sel.points.insert(id);
            }
            k => {
                sel.curves.push(id);
                if matches!(k, EntityKind::Arc { .. }) {
                    sel.arcs.insert(id);
                }
                sel.points.extend(k.children());
            }
        }
    }
    sel.all.extend(sel.points.iter().copied());
    sel.all.extend(sel.curves.iter().copied());
    Ok(sel)
}

/// Constraints that only reference selected entities.
fn internal_constraints(s: &Sketch, sel: &Selection) -> Vec<ConstraintId> {
    s.constraints()
        .filter(|(_, c)| c.kind.entities().iter().all(|e| sel.all.contains(e)))
        .map(|(id, _)| id)
        .collect()
}

/// Largest residual of `kind` at the sketch's current values; `None` if it cannot be built.
fn residual_of(s: &Sketch, kind: &ConstraintKind, lc: f64) -> Option<f64> {
    candidate_max_residual(s, kind, lc).ok()
}

/// Residual tolerance for "this constraint holds" (mm).
fn holds_tol(lc: f64) -> f64 {
    TOL.linear * lc.max(1.0)
}

fn holds(s: &Sketch, kind: &ConstraintKind, lc: f64) -> bool {
    residual_of(s, kind, lc).is_some_and(|r| r <= holds_tol(lc))
}

fn flip(e: End) -> End {
    match e {
        End::Start => End::End,
        End::End => End::Start,
    }
}

/// `kind` with every entity replaced through `f`; `None` if `f` returns `None` for one.
/// `flip_end(e)` says whether the ends of curve `e` swap (a mirrored arc).
fn remap(
    kind: &ConstraintKind,
    f: &dyn Fn(EntityId) -> Option<EntityId>,
    flip_end: &dyn Fn(EntityId) -> bool,
) -> Option<ConstraintKind> {
    use ConstraintKind as K;
    let hv = |t: &HvTarget| -> Option<HvTarget> {
        Some(match *t {
            HvTarget::Line(l) => HvTarget::Line(f(l)?),
            HvTarget::Points(a, b) => HvTarget::Points(f(a)?, f(b)?),
        })
    };
    Some(match kind {
        K::Coincident { a, b } => K::Coincident {
            a: f(*a)?,
            b: f(*b)?,
        },
        K::PointOnCurve { point, curve } => K::PointOnCurve {
            point: f(*point)?,
            curve: f(*curve)?,
        },
        K::Midpoint { point, line } => K::Midpoint {
            point: f(*point)?,
            line: f(*line)?,
        },
        K::Horizontal(t) => K::Horizontal(hv(t)?),
        K::Vertical(t) => K::Vertical(hv(t)?),
        K::Parallel { a, b } => K::Parallel {
            a: f(*a)?,
            b: f(*b)?,
        },
        K::Perpendicular { a, b } => K::Perpendicular {
            a: f(*a)?,
            b: f(*b)?,
        },
        K::Collinear { a, b } => K::Collinear {
            a: f(*a)?,
            b: f(*b)?,
        },
        K::Tangent { a, b, kind } => {
            let mut kind = *kind;
            if let crate::model::TangentKind::AtEndpoint { a_end, b_end, .. } = &mut kind {
                if flip_end(*a) {
                    *a_end = flip(*a_end);
                }
                if flip_end(*b) {
                    *b_end = flip(*b_end);
                }
            }
            K::Tangent {
                a: f(*a)?,
                b: f(*b)?,
                kind,
            }
        }
        K::Curvature {
            a,
            b,
            a_end,
            b_end,
            reversed,
        } => K::Curvature {
            a: f(*a)?,
            b: f(*b)?,
            a_end: if flip_end(*a) { flip(*a_end) } else { *a_end },
            b_end: if flip_end(*b) { flip(*b_end) } else { *b_end },
            reversed: *reversed,
        },
        K::Concentric { a, b } => K::Concentric {
            a: f(*a)?,
            b: f(*b)?,
        },
        K::Equal { a, b } => K::Equal {
            a: f(*a)?,
            b: f(*b)?,
        },
        K::Symmetric { a, b, axis } => K::Symmetric {
            a: f(*a)?,
            b: f(*b)?,
            axis: f(*axis)?,
        },
        K::Fix { .. } => return None,
        K::Dimension(d) => {
            use DimensionKind as D;
            let kind = match &d.kind {
                D::Distance { a, b, side } => D::Distance {
                    a: f(*a)?,
                    b: f(*b)?,
                    side: *side,
                },
                D::HorizontalDistance { a, b, side } => D::HorizontalDistance {
                    a: f(*a)?,
                    b: f(*b)?,
                    side: *side,
                },
                D::VerticalDistance { a, b, side } => D::VerticalDistance {
                    a: f(*a)?,
                    b: f(*b)?,
                    side: *side,
                },
                D::Length { line } => D::Length { line: f(*line)? },
                D::Angle { a, b, branch } => D::Angle {
                    a: f(*a)?,
                    b: f(*b)?,
                    branch: *branch,
                },
                D::Radius { curve } => D::Radius { curve: f(*curve)? },
                D::Diameter { curve } => D::Diameter { curve: f(*curve)? },
                D::ArcLength { arc } => D::ArcLength { arc: f(*arc)? },
                D::EllipseMajor { ellipse } => D::EllipseMajor {
                    ellipse: f(*ellipse)?,
                },
                D::EllipseMinor { ellipse } => D::EllipseMinor {
                    ellipse: f(*ellipse)?,
                },
            };
            K::Dimension(Dimension { kind, ..d.clone() })
        }
    })
}

/// Entities and constraints created so far, to undo them if the operation fails.
#[derive(Default)]
struct Tx {
    created: Vec<EntityId>,
    added: Vec<ConstraintId>,
}

impl Tx {
    fn finish(self) -> EditResult {
        EditResult {
            created: self.created,
            constraints_added: self.added,
            ..EditResult::default()
        }
    }
}

/// Runs `f`; on error removes everything it created.
fn atomic(
    s: &mut Sketch,
    f: impl FnOnce(&mut Sketch, &mut Tx) -> Result<(), SketchError>,
) -> Result<EditResult, SketchError> {
    let mut tx = Tx::default();
    match f(s, &mut tx) {
        Ok(()) => Ok(tx.finish()),
        Err(e) => {
            for c in &tx.added {
                let _ = s.remove_constraint(*c);
            }
            let _ = s.remove_entities(&tx.created);
            Err(e)
        }
    }
}

impl Tx {
    fn constraint(&mut self, s: &mut Sketch, kind: ConstraintKind) -> Result<(), SketchError> {
        let id = s.add_constraint_oriented(kind)?;
        self.added.push(id);
        Ok(())
    }

    fn construction_line(
        &mut self,
        s: &mut Sketch,
        a: EntityId,
        b: EntityId,
    ) -> Result<EntityId, SketchError> {
        let id = s.add_entity(
            EntityKind::Line { start: a, end: b },
            true,
            EntityOrigin::User,
        )?;
        self.created.push(id);
        Ok(id)
    }
}

/// Copies the selection through `xf`. Returns the map from source to image entity; a source
/// entity whose image would be itself (a point at the fixed point of `xf`, a line on the
/// mirror axis) maps to itself and is not created again. Circle radii are multiplied by
/// `rscale`; `flip_arcs` swaps the end points of every arc (a reflection).
fn duplicate(
    s: &mut Sketch,
    tx: &mut Tx,
    sel: &Selection,
    xf: &dyn Fn(Point2) -> Point2,
    rscale: f64,
    flip_arcs: bool,
) -> Result<BTreeMap<EntityId, EntityId>, SketchError> {
    let mut map = BTreeMap::new();
    for &pid in &sel.points {
        let p = s.point(pid).ok_or(SketchError::UnknownEntity(pid))?;
        let q = xf(p);
        if p.distance(q) <= TOL.linear {
            map.insert(pid, pid);
            continue;
        }
        let new = s.add_point(q)?;
        tx.created.push(new);
        if s.entity(pid).is_some_and(|e| e.construction) {
            s.set_construction(new, true)?;
        }
        map.insert(pid, new);
    }
    for &cid in &sel.curves {
        let e = s.entity(cid).ok_or(SketchError::UnknownEntity(cid))?;
        let construction = e.construction;
        let image = match e.kind.clone() {
            EntityKind::Line { start, end } => {
                let (a, b) = (map[&start], map[&end]);
                if (a, b) == (start, end) || (b, a) == (start, end) {
                    None
                } else {
                    Some(EntityKind::Line { start: a, end: b })
                }
            }
            EntityKind::Circle { center, radius } => {
                let c = map[&center];
                let r = s.params().get(radius);
                if c == center && (r * rscale - r).abs() <= TOL.linear {
                    None
                } else {
                    let radius = s.alloc_param(r * rscale, ParamFlags::default())?;
                    Some(EntityKind::Circle { center: c, radius })
                }
            }
            EntityKind::Arc { center, start, end } => {
                let (st, en) = if flip_arcs {
                    (end, start)
                } else {
                    (start, end)
                };
                let (c, a, b) = (map[&center], map[&st], map[&en]);
                if (c, a, b) == (center, start, end) {
                    None
                } else {
                    Some(EntityKind::Arc {
                        center: c,
                        start: a,
                        end: b,
                    })
                }
            }
            EntityKind::Ellipse {
                center,
                major,
                minor_radius,
            } => {
                let (c, m) = (map[&center], map[&major]);
                let b = s.params().get(minor_radius);
                if (c, m) == (center, major) && (b * rscale - b).abs() <= TOL.linear {
                    None
                } else {
                    let minor_radius = s.alloc_param(b * rscale, ParamFlags::default())?;
                    Some(EntityKind::Ellipse {
                        center: c,
                        major: m,
                        minor_radius,
                    })
                }
            }
            EntityKind::EllipticArc {
                center,
                major,
                minor_radius,
                start,
                end,
                t0,
                t1,
            } => {
                let (st, en) = if flip_arcs {
                    (end, start)
                } else {
                    (start, end)
                };
                let (c, m, a, e) = (map[&center], map[&major], map[&st], map[&en]);
                let b = s.params().get(minor_radius);
                if (c, m, a, e) == (center, major, start, end)
                    && (b * rscale - b).abs() <= TOL.linear
                {
                    None
                } else {
                    let (v0, v1) = (s.params().get(t0), s.params().get(t1));
                    // A reflection reverses the sense of the eccentric angle: the image of
                    // the point at `t` is the image ellipse's point at `-t`.
                    let (n0, n1) = if flip_arcs { (-v1, -v0) } else { (v0, v1) };
                    let minor_radius = s.alloc_param(b * rscale, ParamFlags::default())?;
                    let t0 = s.alloc_param(n0, ParamFlags::default())?;
                    let t1 = s.alloc_param(n1, ParamFlags::default())?;
                    Some(EntityKind::EllipticArc {
                        center: c,
                        major: m,
                        minor_radius,
                        start: a,
                        end: e,
                        t0,
                        t1,
                    })
                }
            }
            EntityKind::Spline(def) => {
                let ids = def.points();
                let mapped: Vec<EntityId> = ids.iter().map(|p| map[p]).collect();
                if mapped.iter().zip(&ids).all(|(m, p)| m == p) {
                    None
                } else {
                    let m = |p: &EntityId| map[p];
                    Some(EntityKind::Spline(match def {
                        SplineDef::Control {
                            degree,
                            poles,
                            weights,
                            knots,
                            periodic,
                        } => {
                            let weights = match weights {
                                Some(ws) => Some(
                                    ws.iter()
                                        .map(|&w| {
                                            s.alloc_param(s.params().get(w), ParamFlags::default())
                                        })
                                        .collect::<Result<Vec<_>, _>>()?,
                                ),
                                None => None,
                            };
                            SplineDef::Control {
                                degree,
                                poles: poles.iter().map(m).collect(),
                                weights,
                                knots,
                                periodic,
                            }
                        }
                        SplineDef::Fit {
                            points,
                            start_tangent,
                            end_tangent,
                            params,
                            periodic,
                        } => SplineDef::Fit {
                            points: points.iter().map(m).collect(),
                            start_tangent: start_tangent.as_ref().map(m),
                            end_tangent: end_tangent.as_ref().map(m),
                            params,
                            periodic,
                        },
                    }))
                }
            }
            EntityKind::Conic {
                start,
                end,
                apex,
                rho,
            } => {
                let (a, e, c) = (map[&start], map[&end], map[&apex]);
                if (a, e, c) == (start, end, apex) {
                    None
                } else {
                    let rho = s.alloc_param(s.params().get(rho), ParamFlags::default())?;
                    Some(EntityKind::Conic {
                        start: a,
                        end: e,
                        apex: c,
                        rho,
                    })
                }
            }
            EntityKind::Point { .. } => None,
        };
        match image {
            None => {
                map.insert(cid, cid);
            }
            Some(kind) => {
                let new = s.add_entity(kind, construction, EntityOrigin::User)?;
                tx.created.push(new);
                map.insert(cid, new);
            }
        }
    }
    Ok(map)
}

/// Copies the selection's internal constraints onto the image entities `map`. Dimension
/// values of lengths are multiplied by `dim_scale`. A constraint that does not hold on the
/// image (`Fix`, a horizontal line that was rotated) is skipped.
#[allow(
    clippy::too_many_arguments,
    reason = "private plumbing, one call site per op"
)]
fn copy_constraints(
    s: &mut Sketch,
    tx: &mut Tx,
    sel: &Selection,
    map: &BTreeMap<EntityId, EntityId>,
    xf: &dyn Fn(Point2) -> Point2,
    dim_scale: f64,
    flip_ends: bool,
    lc: f64,
) {
    for cid in internal_constraints(s, sel) {
        let Some(kind) = s.constraint(cid).map(|c| c.kind.clone()) else {
            continue;
        };
        // Everything maps to itself: the constraint already covers the "copy".
        if kind.entities().iter().all(|e| map.get(e) == Some(e)) {
            continue;
        }
        let Some(mut new) = remap(&kind, &|e| map.get(&e).copied(), &|e| {
            flip_ends && sel.arcs.contains(&e)
        }) else {
            continue;
        };
        if let ConstraintKind::Dimension(d) = &mut new {
            if d.kind.quantity() == Quantity::Length {
                d.value *= dim_scale;
            }
            d.label_pos = xf(d.label_pos);
        }
        let new = orient_constraint(s, new);
        if !holds(s, &new, lc) {
            continue;
        }
        if let Ok(id) = s.add_constraint(new) {
            tx.added.push(id);
        }
    }
}

/// Points of `map` that have a distinct image, and circles/arcs with one.
fn moved_points(sel: &Selection, map: &BTreeMap<EntityId, EntityId>) -> Vec<(EntityId, EntityId)> {
    sel.points
        .iter()
        .filter_map(|p| map.get(p).filter(|q| *q != p).map(|q| (*p, *q)))
        .collect()
}

/// `Equal` between every circle and its image.
fn equal_circles(
    s: &mut Sketch,
    tx: &mut Tx,
    sel: &Selection,
    map: &BTreeMap<EntityId, EntityId>,
) -> Result<(), SketchError> {
    for &c in &sel.curves {
        let img = map[&c];
        if img != c
            && matches!(
                s.entity(c).map(|e| &e.kind),
                Some(
                    EntityKind::Circle { .. }
                        | EntityKind::Ellipse { .. }
                        | EntityKind::EllipticArc { .. }
                        | EntityKind::Conic { .. }
                        | EntityKind::Spline(SplineDef::Control {
                            weights: Some(_),
                            ..
                        })
                )
            )
        {
            tx.constraint(s, ConstraintKind::Equal { a: c, b: img })?;
        }
    }
    Ok(())
}

fn finite_vec(what: &'static str, v: Vec2) -> Result<(), SketchError> {
    for c in [v.x, v.y] {
        if !c.is_finite() {
            return Err(SketchError::InvalidValue { what, value: c });
        }
    }
    Ok(())
}

fn unit_dir(what: &'static str, v: Vec2) -> Result<Vec2, SketchError> {
    finite_vec(what, v)?;
    if TOL.is_zero_len(v.length()) {
        return Err(SketchError::Degenerate("pattern direction has zero length"));
    }
    Ok(v.normalize())
}

fn finite_value(what: &'static str, v: f64) -> Result<(), SketchError> {
    if v.is_finite() {
        Ok(())
    } else {
        Err(SketchError::InvalidValue { what, value: v })
    }
}

// ---- Mirror -------------------------------------------------------------------------------

/// Mirrors `items` about the line `axis`.
///
/// Creates the image entities. A mirrored arc swaps its start and end (the image of a CCW arc
/// is CW). Points on the axis are shared with their images; lines on the axis and circles
/// centred on it are not duplicated. With `constrain = false` the image is independent and
/// gets copies of the selection's internal constraints that still hold. With `constrain =
/// true` the image is tied to the source (see the [module docs](self)).
///
/// Errors: `UnknownEntity`, `WrongEntityKind` (`axis` is not a line), `Degenerate` (zero
/// length axis), `NotApplicable` (empty selection).
pub fn mirror(
    s: &mut Sketch,
    items: &[EntityId],
    axis: EntityId,
    constrain: bool,
) -> Result<EditResult, SketchError> {
    let (a, b) = match s.entity(axis).ok_or(SketchError::UnknownEntity(axis))?.kind {
        EntityKind::Line { start, end } => (start, end),
        ref k => {
            return Err(SketchError::WrongEntityKind {
                entity: axis,
                expected: "line",
                found: k.name(),
            });
        }
    };
    let (Some(p0), Some(p1)) = (s.point(a), s.point(b)) else {
        return Err(SketchError::UnknownEntity(axis));
    };
    let dir = p1 - p0;
    if TOL.is_zero_len(dir.length()) {
        return Err(SketchError::Degenerate("mirror axis has zero length"));
    }
    let u = dir.normalize();
    let sel = select(s, items)?;
    let lc = characteristic_length(s);
    let xf = move |p: Point2| {
        let r = p - p0;
        p0 + 2.0 * r.dot(u) * u - r
    };
    atomic(s, |s, tx| {
        let map = duplicate(s, tx, &sel, &xf, 1.0, true)?;
        if constrain {
            for &p in &sel.points {
                let q = map[&p];
                if q != p {
                    tx.constraint(s, ConstraintKind::Symmetric { a: p, b: q, axis })?;
                } else if p != a && p != b && !on_axis_constrained(s, p, axis) {
                    tx.constraint(
                        s,
                        ConstraintKind::PointOnCurve {
                            point: p,
                            curve: axis,
                        },
                    )?;
                }
            }
            equal_circles(s, tx, &sel, &map)?;
        } else {
            copy_constraints(s, tx, &sel, &map, &xf, 1.0, true, lc);
        }
        Ok(())
    })
}

fn on_axis_constrained(s: &Sketch, p: EntityId, axis: EntityId) -> bool {
    s.constraints().any(|(_, c)| {
        matches!(&c.kind, ConstraintKind::PointOnCurve { point, curve }
            if *point == p && *curve == axis)
    })
}

// ---- Rectangular pattern ------------------------------------------------------------------

const MAX_INSTANCES: u64 = 100_000;

/// Repeats `items` on a grid: `n_a` instances (the source included) spaced `d_a` mm along
/// `dir_a`, and optionally `n_b` rows spaced `d_b` along `dir_b` (`dir_b` is
/// `Some((direction, n_b, d_b))`). Creates `n_a * n_b - 1` copies.
///
/// With `constrain = false` the copies are independent (they keep copies of the selection's
/// internal constraints); with `constrain = true` they follow the source (see the
/// [module docs](self)).
///
/// Errors: `Degenerate` (zero direction or spacing), `InvalidValue` (non-finite input or a
/// count of 0), `NotApplicable` (no copies, or more than 100 000), plus the selection errors
/// of [`mirror`].
pub fn pattern_rect(
    s: &mut Sketch,
    items: &[EntityId],
    dir_a: Vec2,
    n_a: u32,
    d_a: f64,
    dir_b: Option<(Vec2, u32, f64)>,
    constrain: bool,
) -> Result<EditResult, SketchError> {
    let ua = unit_dir("pattern direction", dir_a)?;
    finite_value("pattern spacing", d_a)?;
    if n_a == 0 {
        return Err(SketchError::InvalidValue {
            what: "pattern count",
            value: 0.0,
        });
    }
    let (ub, n_b, d_b) = match dir_b {
        Some((d, n, sp)) => {
            finite_value("pattern spacing", sp)?;
            if n == 0 {
                return Err(SketchError::InvalidValue {
                    what: "pattern count",
                    value: 0.0,
                });
            }
            (unit_dir("pattern direction", d)?, n, sp)
        }
        None => (Vec2::ZERO, 1, 0.0),
    };
    let total = u64::from(n_a) * u64::from(n_b);
    if total < 2 {
        return Err(SketchError::NotApplicable("the pattern has no copies"));
    }
    if total > MAX_INSTANCES {
        return Err(SketchError::NotApplicable("too many pattern instances"));
    }
    if (n_a > 1 && TOL.is_zero_len(d_a)) || (n_b > 1 && TOL.is_zero_len(d_b)) {
        return Err(SketchError::Degenerate("pattern spacing is zero"));
    }
    let sel = select(s, items)?;
    let lc = characteristic_length(s);
    atomic(s, |s, tx| {
        for j in 0..n_b {
            for i in 0..n_a {
                if i == 0 && j == 0 {
                    continue;
                }
                let off = ua * (d_a * f64::from(i)) + ub * (d_b * f64::from(j));
                let xf = move |p: Point2| p + off;
                let map = duplicate(s, tx, &sel, &xf, 1.0, false)?;
                if constrain {
                    link_offsets(s, tx, &moved_points(&sel, &map))?;
                    equal_circles(s, tx, &sel, &map)?;
                } else {
                    copy_constraints(s, tx, &sel, &map, &xf, 1.0, false, lc);
                }
            }
        }
        Ok(())
    })
}

/// Fixes the offset of every `(source, image)` pair to its current value.
fn link_offsets(
    s: &mut Sketch,
    tx: &mut Tx,
    pairs: &[(EntityId, EntityId)],
) -> Result<(), SketchError> {
    for &(p, q) in pairs {
        let (Some(a), Some(b)) = (s.point(p), s.point(q)) else {
            continue;
        };
        let d = b - a;
        let mid = a.midpoint(b);
        let horizontal = |value: f64, side: Side| {
            ConstraintKind::Dimension(Dimension {
                kind: DimensionKind::HorizontalDistance { a: p, b: q, side },
                value,
                driving: true,
                label_pos: mid,
            })
        };
        let vertical = |value: f64, side: Side| {
            ConstraintKind::Dimension(Dimension {
                kind: DimensionKind::VerticalDistance { a: p, b: q, side },
                value,
                driving: true,
                label_pos: mid,
            })
        };
        let side = |v: f64| {
            if v < 0.0 {
                Side::Negative
            } else {
                Side::Positive
            }
        };
        if TOL.is_zero_len(d.x) {
            tx.constraint(s, ConstraintKind::Vertical(HvTarget::Points(p, q)))?;
        } else {
            tx.constraint(s, horizontal(d.x.abs(), side(d.x)))?;
        }
        if TOL.is_zero_len(d.y) {
            tx.constraint(s, ConstraintKind::Horizontal(HvTarget::Points(p, q)))?;
        } else {
            tx.constraint(s, vertical(d.y.abs(), side(d.y)))?;
        }
    }
    Ok(())
}

// ---- Circular pattern ---------------------------------------------------------------------

/// Repeats `items` around `center` (a point, or a circle or arc, whose centre is used): `n`
/// instances including the source over `total_angle` radians (CCW positive). A full turn
/// (`|total_angle| = 2π`) spaces the instances by `total_angle / n`; otherwise the last
/// instance sits at `total_angle`, so the spacing is `total_angle / (n - 1)`.
///
/// With `constrain = false` the copies are independent; with `constrain = true` they follow
/// the source, including a change of its distance from `center` (see the [module docs](self)).
///
/// Errors: `WrongEntityKind` (`center` is not a point, circle or arc), `InvalidValue`
/// (non-finite angle), `Degenerate` (zero angle), `NotApplicable` (`n < 2` or too many
/// instances), plus the selection errors of [`mirror`].
pub fn pattern_circ(
    s: &mut Sketch,
    items: &[EntityId],
    center: EntityId,
    n: u32,
    total_angle: f64,
    constrain: bool,
) -> Result<EditResult, SketchError> {
    let ce = s.entity(center).ok_or(SketchError::UnknownEntity(center))?;
    let center_pt = match ce.kind {
        EntityKind::Point { .. } => center,
        EntityKind::Circle { center: c, .. }
        | EntityKind::Arc { center: c, .. }
        | EntityKind::Ellipse { center: c, .. }
        | EntityKind::EllipticArc { center: c, .. } => c,
        ref k => {
            return Err(SketchError::WrongEntityKind {
                entity: center,
                expected: "point, circle, arc or ellipse",
                found: k.name(),
            });
        }
    };
    let c = s
        .point(center_pt)
        .ok_or(SketchError::UnknownEntity(center_pt))?;
    finite_value("pattern angle", total_angle)?;
    if total_angle.abs() <= TOL.angular {
        return Err(SketchError::Degenerate("pattern angle is zero"));
    }
    if n < 2 {
        return Err(SketchError::NotApplicable("the pattern has no copies"));
    }
    if u64::from(n) > MAX_INSTANCES {
        return Err(SketchError::NotApplicable("too many pattern instances"));
    }
    let tau = std::f64::consts::TAU;
    let step = if (total_angle.abs() - tau).abs() <= 1e-9 {
        total_angle / f64::from(n)
    } else {
        total_angle / f64::from(n - 1)
    };
    let sel = select(s, items)?;
    let lc = characteristic_length(s);
    atomic(s, |s, tx| {
        // Construction line centre → source point, shared by all instances.
        let mut src_links: BTreeMap<EntityId, EntityId> = BTreeMap::new();
        for k in 1..n {
            let theta = step * f64::from(k);
            let rot = rotation_about(c, theta);
            let xf = move |p: Point2| rot.transform_point2(p);
            let map = duplicate(s, tx, &sel, &xf, 1.0, false)?;
            if constrain {
                // Angle dimension values live in (-pi, pi].
                let value = {
                    let w = theta.rem_euclid(tau);
                    if w > std::f64::consts::PI { w - tau } else { w }
                };
                for (p, q) in moved_points(&sel, &map) {
                    let src = match src_links.get(&p) {
                        Some(&l) => l,
                        None => {
                            let l = tx.construction_line(s, center_pt, p)?;
                            src_links.insert(p, l);
                            l
                        }
                    };
                    let img = tx.construction_line(s, center_pt, q)?;
                    tx.constraint(s, ConstraintKind::Equal { a: src, b: img })?;
                    let mid = c.midpoint(s.point(q).unwrap_or(c));
                    tx.constraint(
                        s,
                        ConstraintKind::Dimension(Dimension {
                            kind: DimensionKind::Angle {
                                a: src,
                                b: img,
                                branch: AngleBranch {
                                    reverse_a: false,
                                    reverse_b: false,
                                },
                            },
                            value,
                            driving: true,
                            label_pos: mid,
                        }),
                    )?;
                }
                equal_circles(s, tx, &sel, &map)?;
            } else {
                copy_constraints(s, tx, &sel, &map, &xf, 1.0, false, lc);
            }
        }
        Ok(())
    })
}

// ---- Move / copy / rotate / scale ---------------------------------------------------------

/// Splits a similarity into its uniform scale; `Err` if it is not a rotation + translation
/// + positive uniform scale.
fn similarity_scale(t: &Transform2) -> Result<f64, SketchError> {
    let m = t.matrix2;
    let (x, y) = (m.x_axis, m.y_axis);
    let det = x.perp_dot(y);
    if !t.translation.is_finite() || !m.to_cols_array().iter().all(|v| v.is_finite()) {
        return Err(SketchError::InvalidValue {
            what: "transform",
            value: f64::NAN,
        });
    }
    let (lx, ly) = (x.length(), y.length());
    if lx <= 0.0 || det.abs() <= 1e-12 * lx * ly || TOL.is_zero_len(lx) {
        return Err(SketchError::Degenerate("transform scale is zero"));
    }
    if det < 0.0 {
        return Err(SketchError::NotApplicable(
            "reflections are not supported by transform; use mirror",
        ));
    }
    if (lx - ly).abs() > 1e-9 * lx || x.dot(y).abs() > 1e-9 * lx * ly {
        return Err(SketchError::NotApplicable(
            "only rotation, translation and uniform scale are supported",
        ));
    }
    Ok(lx)
}

/// Moves, rotates and/or scales `items` by the in-plane similarity `t` (build it with
/// [`translation`], [`rotation_about`], [`scale_about`] and `*`).
///
/// `t` must be a rotation, translation and positive uniform scale (`NotApplicable`
/// otherwise; use [`mirror`] for reflections, `Degenerate` for a zero scale).
///
/// - `copy = false` moves the selection in place. Circle radii and the values of length
///   dimensions whose entities are all selected are multiplied by the scale. Constraints
///   that held before and no longer hold (`Horizontal` after a rotation) are removed and
///   reported in `constraints_removed`. Constraints that reach outside the selection are kept
///   (a later solve pulls the selection back or reports them). `Locked` if a selected point or
///   radius is projected or fixed.
/// - `copy = true` leaves the source alone and creates an independent transformed copy with
///   copies of the internal constraints (see the [module docs](self)).
pub fn transform(
    s: &mut Sketch,
    items: &[EntityId],
    t: Transform2,
    copy: bool,
) -> Result<EditResult, SketchError> {
    let scale = similarity_scale(&t)?;
    let sel = select(s, items)?;
    let lc = characteristic_length(s);
    let xf = move |p: Point2| t.transform_point2(p);
    if copy {
        return atomic(s, |s, tx| {
            let map = duplicate(s, tx, &sel, &xf, scale, false)?;
            copy_constraints(s, tx, &sel, &map, &xf, scale, false, lc);
            Ok(())
        });
    }

    // In place. Check for locks first.
    let fixed = fixed_params(s);
    let mut coords: Vec<(crate::ids::ParamId, crate::ids::ParamId, Point2)> = Vec::new();
    for &pid in &sel.points {
        if let Some(EntityKind::Point { x, y }) = s.entity(pid).map(|e| &e.kind) {
            if fixed.contains(x) || fixed.contains(y) {
                return Err(SketchError::Locked(pid));
            }
            let p = s.point(pid).ok_or(SketchError::UnknownEntity(pid))?;
            coords.push((*x, *y, xf(p)));
        }
    }
    let mut radii: Vec<(crate::ids::ParamId, f64)> = Vec::new();
    for &cid in &sel.curves {
        if let Some(
            EntityKind::Circle { radius, .. }
            | EntityKind::Ellipse {
                minor_radius: radius,
                ..
            }
            | EntityKind::EllipticArc {
                minor_radius: radius,
                ..
            },
        ) = s.entity(cid).map(|e| &e.kind)
        {
            if fixed.contains(radius) {
                return Err(SketchError::Locked(cid));
            }
            radii.push((*radius, s.params().get(*radius) * scale));
        }
    }
    let internal = internal_constraints(s, &sel);
    let held_before: Vec<bool> = internal
        .iter()
        .map(|id| s.constraint(*id).is_some_and(|c| holds(s, &c.kind, lc)))
        .collect();

    {
        let params = s.params_mut();
        for (x, y, q) in coords {
            params.set(x, q.x);
            params.set(y, q.y);
        }
        for (r, v) in radii {
            params.set(r, v);
        }
    }
    let mut result = EditResult::default();
    if (scale - 1.0).abs() > f64::EPSILON {
        for id in &internal {
            if let Some(d) = s.constraint(*id).and_then(|c| c.kind.as_dimension())
                && d.kind.quantity() == Quantity::Length
            {
                let v = d.value * scale;
                s.set_dimension_value(*id, v)?;
            }
        }
    }
    // The scaled geometry's characteristic length differs; judge with the new one.
    let lc = characteristic_length(s);
    for (id, was) in internal.iter().zip(held_before) {
        let broken = was && s.constraint(*id).is_some_and(|c| !holds(s, &c.kind, lc));
        if broken && s.remove_constraint(*id).is_ok() {
            result.constraints_removed.push(*id);
        }
    }
    Ok(result)
}
