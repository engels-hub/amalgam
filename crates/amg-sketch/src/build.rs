//! Composite builders (SK-21): rectangles, polygons, slots, three-point and tangent circles
//! and arcs.
//!
//! Each builder adds its entities and constraints to a [`Sketch`] in one step and returns
//! them as a [`Built`]. The geometry is exact, so every residual is zero right after the call
//! (up to rounding), and the constraint set is the minimal one that keeps the shape's
//! generic degrees of freedom (see each function). Dimensions are not added, except the
//! radius that [`circle_2_tangent`] is defined by.
//!
//! All builders validate before they mutate. If an error still occurs half way, the entities
//! created so far are removed again, so a failed call leaves the sketch's geometry unchanged
//! (only the revision counters and retired parameter slots differ).

use amg_core::{Point2, Tolerance, Vec2};

use crate::error::SketchError;
use crate::ids::{ConstraintId, EntityId};
use crate::model::{
    ConstraintKind, Dimension, DimensionKind, End, EntityKind, EntityOrigin, HvTarget, ParamFlags,
    Sketch, TangentKind,
};

/// What a builder created.
#[derive(Clone, Debug, Default)]
pub struct Built {
    /// Every entity the builder created (points included), in creation order. Entities the
    /// builder merely reused (the `from` curve of [`arc_tangent`]) are not listed.
    pub entities: Vec<EntityId>,
    /// Every constraint the builder created.
    pub constraints: Vec<ConstraintId>,
}

const TOL: Tolerance = Tolerance::DEFAULT;

// ---- Plumbing -----------------------------------------------------------------------------

/// Accumulates created ids and offers short helpers.
struct B<'a> {
    s: &'a mut Sketch,
    built: Built,
}

impl B<'_> {
    fn point(&mut self, p: Point2) -> Result<EntityId, SketchError> {
        let id = self.s.add_point(p)?;
        self.built.entities.push(id);
        Ok(id)
    }

    fn entity(&mut self, kind: EntityKind, construction: bool) -> Result<EntityId, SketchError> {
        let id = self.s.add_entity(kind, construction, EntityOrigin::User)?;
        self.built.entities.push(id);
        Ok(id)
    }

    fn line(
        &mut self,
        a: EntityId,
        b: EntityId,
        construction: bool,
    ) -> Result<EntityId, SketchError> {
        self.entity(EntityKind::Line { start: a, end: b }, construction)
    }

    fn arc(
        &mut self,
        center: EntityId,
        start: EntityId,
        end: EntityId,
        construction: bool,
    ) -> Result<EntityId, SketchError> {
        self.entity(EntityKind::Arc { center, start, end }, construction)
    }

    fn circle(
        &mut self,
        center: EntityId,
        r: f64,
        construction: bool,
    ) -> Result<EntityId, SketchError> {
        let radius = self.s.alloc_param(r, ParamFlags::default())?;
        self.entity(EntityKind::Circle { center, radius }, construction)
    }

    fn con(&mut self, kind: ConstraintKind) -> Result<ConstraintId, SketchError> {
        let id = self.s.add_constraint(kind)?;
        self.built.constraints.push(id);
        Ok(id)
    }

    /// Like [`con`](B::con), but picks the tangent / side flags from the current geometry.
    fn con_oriented(&mut self, kind: ConstraintKind) -> Result<ConstraintId, SketchError> {
        let id = self.s.add_constraint_oriented(kind)?;
        self.built.constraints.push(id);
        Ok(id)
    }

    fn horizontal(&mut self, line: EntityId) -> Result<ConstraintId, SketchError> {
        self.con(ConstraintKind::Horizontal(HvTarget::Line(line)))
    }

    fn vertical(&mut self, line: EntityId) -> Result<ConstraintId, SketchError> {
        self.con(ConstraintKind::Vertical(HvTarget::Line(line)))
    }

    fn tangent_at(
        &mut self,
        a: EntityId,
        a_end: End,
        b: EntityId,
        b_end: End,
    ) -> Result<ConstraintId, SketchError> {
        self.con_oriented(ConstraintKind::Tangent {
            a,
            b,
            kind: TangentKind::AtEndpoint {
                a_end,
                b_end,
                reversed: false,
            },
        })
    }
}

/// Runs `f` on a fresh [`B`]; on error removes everything it created.
fn run(
    s: &mut Sketch,
    f: impl FnOnce(&mut B<'_>) -> Result<(), SketchError>,
) -> Result<Built, SketchError> {
    let mut b = B {
        s,
        built: Built::default(),
    };
    match f(&mut b) {
        Ok(()) => Ok(b.built),
        Err(e) => {
            // Removing the entities also removes their constraints and orphaned points.
            let _ = b.s.remove_entities(&b.built.entities);
            Err(e)
        }
    }
}

fn finite(points: &[Point2]) -> Result<(), SketchError> {
    for p in points {
        for v in [p.x, p.y] {
            if !v.is_finite() {
                return Err(SketchError::InvalidValue {
                    what: "point coordinate",
                    value: v,
                });
            }
        }
    }
    Ok(())
}

/// `w` must be finite and positive beyond tolerance.
fn width_ok(w: f64) -> Result<(), SketchError> {
    if !w.is_finite() || w < 0.0 {
        return Err(SketchError::InvalidValue {
            what: "slot width",
            value: w,
        });
    }
    if TOL.is_zero_len(w) {
        return Err(SketchError::Degenerate("slot width is zero"));
    }
    Ok(())
}

fn unit(v: Vec2) -> Option<Vec2> {
    let l = v.length();
    (!TOL.is_zero_len(l)).then(|| v / l)
}

// ---- Rectangles ---------------------------------------------------------------------------

/// Axis-aligned rectangle from two opposite corners.
///
/// Creates 4 points and 4 lines (`a → (b.x, a.y) → b → (a.x, b.y) → a`), with two
/// `Horizontal` and two `Vertical` constraints. DOF 4 (position of two corners).
/// `Err(Degenerate)` if the width or height is zero.
pub fn rectangle_2pt(s: &mut Sketch, a: Point2, b: Point2) -> Result<Built, SketchError> {
    finite(&[a, b])?;
    if TOL.is_zero_len((b.x - a.x).abs()) || TOL.is_zero_len((b.y - a.y).abs()) {
        return Err(SketchError::Degenerate(
            "rectangle has zero width or height",
        ));
    }
    run(s, |k| {
        let corners = [a, Point2::new(b.x, a.y), b, Point2::new(a.x, b.y)];
        axis_aligned(k, corners)?;
        Ok(())
    })
}

/// Four points, four lines and the H/V constraints; returns the lines.
fn axis_aligned(k: &mut B<'_>, c: [Point2; 4]) -> Result<[EntityId; 4], SketchError> {
    let p = [
        k.point(c[0])?,
        k.point(c[1])?,
        k.point(c[2])?,
        k.point(c[3])?,
    ];
    let l = [
        k.line(p[0], p[1], false)?,
        k.line(p[1], p[2], false)?,
        k.line(p[2], p[3], false)?,
        k.line(p[3], p[0], false)?,
    ];
    k.horizontal(l[0])?;
    k.vertical(l[1])?;
    k.horizontal(l[2])?;
    k.vertical(l[3])?;
    Ok(l)
}

/// Rectangle with one side `a → b` (any angle) and the opposite side through `c`.
///
/// Creates 4 points (`a, b`, then the corners above `b` and `a` at the signed distance of `c`
/// from line `ab`) and 4 lines, with three `Perpendicular` constraints (the fourth corner
/// follows). DOF 5 (position, angle, width, height). `Err(Degenerate)` if `a == b` or `c`
/// lies on line `ab`.
pub fn rectangle_3pt(
    s: &mut Sketch,
    a: Point2,
    b: Point2,
    c: Point2,
) -> Result<Built, SketchError> {
    finite(&[a, b, c])?;
    let u = unit(b - a).ok_or(SketchError::Degenerate("rectangle base has zero length"))?;
    let n = u.perp();
    let d = (c - a).dot(n);
    if TOL.is_zero_len(d) {
        return Err(SketchError::Degenerate("rectangle has zero height"));
    }
    run(s, |k| {
        let p = [
            k.point(a)?,
            k.point(b)?,
            k.point(b + n * d)?,
            k.point(a + n * d)?,
        ];
        let l = [
            k.line(p[0], p[1], false)?,
            k.line(p[1], p[2], false)?,
            k.line(p[2], p[3], false)?,
            k.line(p[3], p[0], false)?,
        ];
        for w in l.windows(2) {
            k.con(ConstraintKind::Perpendicular { a: w[0], b: w[1] })?;
        }
        Ok(())
    })
}

/// Axis-aligned rectangle from its centre and one corner.
///
/// Creates the 4 corner points and lines with H/V constraints, a construction diagonal from
/// the first to the third corner, and the centre point constrained to the diagonal's
/// midpoint. DOF 4 (centre, width, height). `Err(Degenerate)` if the width or height is zero.
pub fn rectangle_center(
    s: &mut Sketch,
    center: Point2,
    corner: Point2,
) -> Result<Built, SketchError> {
    finite(&[center, corner])?;
    let h = corner - center;
    if TOL.is_zero_len(h.x.abs()) || TOL.is_zero_len(h.y.abs()) {
        return Err(SketchError::Degenerate(
            "rectangle has zero width or height",
        ));
    }
    run(s, |k| {
        let c = [
            center - h,
            Point2::new(center.x + h.x, center.y - h.y),
            center + h,
            Point2::new(center.x - h.x, center.y + h.y),
        ];
        let l = axis_aligned(k, c)?;
        let (first, _) = line_ends(k.s, l[0])?;
        let (third, _) = line_ends(k.s, l[2])?;
        let diag = k.line(first, third, true)?;
        let m = k.point(center)?;
        k.con(ConstraintKind::Midpoint {
            point: m,
            line: diag,
        })?;
        Ok(())
    })
}

fn line_ends(s: &Sketch, line: EntityId) -> Result<(EntityId, EntityId), SketchError> {
    match s.entity(line).map(|e| &e.kind) {
        Some(EntityKind::Line { start, end }) => Ok((*start, *end)),
        Some(k) => Err(SketchError::WrongEntityKind {
            entity: line,
            expected: "line",
            found: k.name(),
        }),
        None => Err(SketchError::UnknownEntity(line)),
    }
}

// ---- Polygon ------------------------------------------------------------------------------

/// Regular polygon with `sides` sides about `center`, CCW.
///
/// Creates a construction circle, `sides` points and `sides` lines, and:
/// - `inscribed == true`: `vertex` is the first vertex; every vertex lies on the circle
///   (`PointOnCurve`), whose radius is `|vertex - center|`; consecutive sides are `Equal`
///   (`sides - 1`);
/// - `inscribed == false`: `vertex` is the *midpoint of the first side*; the circle is the
///   incircle of radius `|vertex - center|` and every side is tangent to it. `sides`
///   construction spokes from the centre to the vertices are made `Equal` (`sides - 1`).
///
/// DOF 4 (centre, size, rotation). `Err(Degenerate)` if `sides < 3` or `vertex == center`.
pub fn polygon(
    s: &mut Sketch,
    center: Point2,
    vertex: Point2,
    sides: u32,
    inscribed: bool,
) -> Result<Built, SketchError> {
    finite(&[center, vertex])?;
    if sides < 3 {
        return Err(SketchError::Degenerate("polygon needs at least 3 sides"));
    }
    let v = vertex - center;
    let r = v.length();
    if TOL.is_zero_len(r) {
        return Err(SketchError::Degenerate(
            "polygon vertex coincides with its centre",
        ));
    }
    let n = sides as usize;
    let step = std::f64::consts::TAU / f64::from(sides);
    let phi0 = v.y.atan2(v.x);
    // Circumradius and angle of the first vertex.
    let (big_r, first) = if inscribed {
        (r, phi0)
    } else {
        (r / (step / 2.0).cos(), phi0 - step / 2.0)
    };
    run(s, |k| {
        let c = k.point(center)?;
        let circle = k.circle(c, r, true)?;
        let mut pts = Vec::with_capacity(n);
        for i in 0..n {
            let a = first + step * i as f64;
            pts.push(k.point(center + Vec2::new(a.cos(), a.sin()) * big_r)?);
        }
        let mut lines = Vec::with_capacity(n);
        for i in 0..n {
            lines.push(k.line(pts[i], pts[(i + 1) % n], false)?);
        }
        if inscribed {
            for &p in &pts {
                k.con(ConstraintKind::PointOnCurve {
                    point: p,
                    curve: circle,
                })?;
            }
            for w in lines.windows(2) {
                k.con(ConstraintKind::Equal { a: w[0], b: w[1] })?;
            }
        } else {
            // Tangent sides alone leave even-sided polygons free to become rhombi, so
            // construction spokes of equal length pin the vertices to a common circle.
            let mut spokes = Vec::with_capacity(n);
            for &p in &pts {
                spokes.push(k.line(c, p, true)?);
            }
            for &l in &lines {
                k.con_oriented(ConstraintKind::Tangent {
                    a: l,
                    b: circle,
                    kind: TangentKind::External,
                })?;
            }
            for w in spokes.windows(2) {
                k.con(ConstraintKind::Equal { a: w[0], b: w[1] })?;
            }
        }
        Ok(())
    })
}

// ---- Slots --------------------------------------------------------------------------------

/// Straight slot (stadium) with end-arc centres `c1`, `c2` and radius `r`.
///
/// Entities: both arc centres (the first two entities created), 4 arc/line end points, 2
/// lines, 2 arcs. Constraints: four `AtEndpoint` tangencies and `Equal` radii. DOF 5 (two
/// centres and the radius).
fn slot_core(k: &mut B<'_>, c1: Point2, c2: Point2, r: f64) -> Result<(), SketchError> {
    let u = unit(c2 - c1).ok_or(SketchError::Degenerate("slot has zero length"))?;
    let n = u.perp();
    let pc1 = k.point(c1)?;
    let pc2 = k.point(c2)?;
    let t1 = k.point(c1 + n * r)?;
    let t2 = k.point(c2 + n * r)?;
    let b2 = k.point(c2 - n * r)?;
    let b1 = k.point(c1 - n * r)?;
    let top = k.line(t1, t2, false)?;
    let bottom = k.line(b2, b1, false)?;
    let right = k.arc(pc2, b2, t2, false)?;
    let left = k.arc(pc1, t1, b1, false)?;
    k.tangent_at(top, End::End, right, End::End)?;
    k.tangent_at(right, End::Start, bottom, End::Start)?;
    k.tangent_at(left, End::End, bottom, End::End)?;
    k.tangent_at(top, End::Start, left, End::Start)?;
    k.con(ConstraintKind::Equal { a: right, b: left })?;
    Ok(())
}

/// Slot from the centres `a`, `b` of its two end arcs and the slot `width`.
///
/// Creates 6 points, 2 lines and 2 arcs, four `AtEndpoint` tangencies and an `Equal` on the
/// two arc radii. DOF 5. `Err(Degenerate)` if `a == b` or `width == 0`.
pub fn slot_center_to_center(
    s: &mut Sketch,
    a: Point2,
    b: Point2,
    width: f64,
) -> Result<Built, SketchError> {
    finite(&[a, b])?;
    width_ok(width)?;
    unit(b - a).ok_or(SketchError::Degenerate("slot has zero length"))?;
    run(s, |k| slot_core(k, a, b, width / 2.0))
}

/// Slot from the two extreme points `a`, `b` of its overall length and the `width`.
///
/// The end-arc centres lie `width / 2` inside `a` and `b`. Same entities and constraints as
/// [`slot_center_to_center`]; DOF 5. `Err(Degenerate)` if `|ab| <= width` or `width == 0`.
pub fn slot_overall(
    s: &mut Sketch,
    a: Point2,
    b: Point2,
    width: f64,
) -> Result<Built, SketchError> {
    finite(&[a, b])?;
    width_ok(width)?;
    let u = unit(b - a).ok_or(SketchError::Degenerate("slot has zero length"))?;
    if (b - a).length() - width <= TOL.linear {
        return Err(SketchError::Degenerate("slot is not longer than its width"));
    }
    let r = width / 2.0;
    run(s, |k| slot_core(k, a + u * r, b - u * r, r))
}

/// Slot from its centre point `center`, the centre `end` of one end arc, and the `width`.
///
/// The other arc centre is the mirror image of `end`. Adds a construction centreline between
/// the two arc centres and a point at `center` constrained to its midpoint, on top of the
/// [`slot_center_to_center`] set. DOF 5. `Err(Degenerate)` if `center == end` or `width == 0`.
pub fn slot_center_point(
    s: &mut Sketch,
    center: Point2,
    end: Point2,
    width: f64,
) -> Result<Built, SketchError> {
    finite(&[center, end])?;
    width_ok(width)?;
    unit(end - center).ok_or(SketchError::Degenerate("slot has zero length"))?;
    let other = center * 2.0 - end;
    run(s, |k| {
        slot_core(k, other, end, width / 2.0)?;
        // The arc centres are the first two entities created.
        let (c1, c2) = (k.built.entities[0], k.built.entities[1]);
        let centerline = k.line(c1, c2, true)?;
        let m = k.point(center)?;
        k.con(ConstraintKind::Midpoint {
            point: m,
            line: centerline,
        })?;
        Ok(())
    })
}

/// Arc slot: the centreline is the CCW arc about `center` from `start` to `end`.
///
/// Entities: a construction centreline arc, the outer and inner arcs (radii `R ± width / 2`,
/// sharing the centre point) and two semicircular end caps. Constraints: four `AtEndpoint`
/// tangencies. DOF 6 (centre, two radii, two angles). The centreline arc's own radius
/// equation is implied by the caps, so the Jacobian has one dependent row.
///
/// `Err(Degenerate)` if `start` or `end` is at `center`, the width is zero, the inner radius
/// `R - width / 2` is not positive, or the end caps would overlap (chord `|start - end|` not
/// longer than the width).
pub fn slot_arc(
    s: &mut Sketch,
    center: Point2,
    start: Point2,
    end: Point2,
    width: f64,
) -> Result<Built, SketchError> {
    finite(&[center, start, end])?;
    width_ok(width)?;
    let us = unit(start - center).ok_or(SketchError::Degenerate("slot arc start at centre"))?;
    let ue = unit(end - center).ok_or(SketchError::Degenerate("slot arc end at centre"))?;
    let r = start.distance(center);
    let h = width / 2.0;
    if r - h <= TOL.linear {
        return Err(SketchError::Degenerate(
            "slot arc radius is not larger than half the width",
        ));
    }
    let (ps, pe) = (center + us * r, center + ue * r);
    if ps.distance(pe) <= width + TOL.linear {
        return Err(SketchError::Degenerate(
            "slot arc is not longer than its width",
        ));
    }
    run(s, |k| {
        let c = k.point(center)?;
        let sp = k.point(ps)?;
        let ep = k.point(pe)?;
        let so = k.point(center + us * (r + h))?;
        let si = k.point(center + us * (r - h))?;
        let eo = k.point(center + ue * (r + h))?;
        let ei = k.point(center + ue * (r - h))?;
        k.arc(c, sp, ep, true)?;
        let outer = k.arc(c, so, eo, false)?;
        let inner = k.arc(c, si, ei, false)?;
        let cap_s = k.arc(sp, si, so, false)?;
        let cap_e = k.arc(ep, eo, ei, false)?;
        k.tangent_at(outer, End::End, cap_e, End::Start)?;
        k.tangent_at(cap_e, End::End, inner, End::End)?;
        k.tangent_at(inner, End::Start, cap_s, End::Start)?;
        k.tangent_at(cap_s, End::End, outer, End::Start)?;
        Ok(())
    })
}

// ---- Circles and arcs ---------------------------------------------------------------------

/// Centre of the circle through three points; `None` if they are (nearly) collinear.
fn circumcenter(a: Point2, b: Point2, c: Point2) -> Option<Point2> {
    let (bb, cc) = (b - a, c - a);
    let cross = bb.perp_dot(cc);
    let scale = bb.length() * cc.length();
    if TOL.is_zero_len(scale) || cross.abs() <= TOL.angular * scale {
        return None;
    }
    let d = 2.0 * cross;
    let (b2, c2) = (bb.length_squared(), cc.length_squared());
    let ux = (cc.y * b2 - bb.y * c2) / d;
    let uy = (bb.x * c2 - cc.x * b2) / d;
    Some(a + Vec2::new(ux, uy))
}

/// Circle through three points. Creates the circle and its centre point only (the clicked
/// points are not kept); no constraints, DOF 3. `Err(Degenerate)` if the points are
/// collinear or coincide.
pub fn circle_3pt(s: &mut Sketch, a: Point2, b: Point2, c: Point2) -> Result<Built, SketchError> {
    finite(&[a, b, c])?;
    let o =
        circumcenter(a, b, c).ok_or(SketchError::Degenerate("the three points are collinear"))?;
    let r = o.distance(a);
    run(s, |k| {
        let p = k.point(o)?;
        k.circle(p, r, false)?;
        Ok(())
    })
}

/// Arc from `a` through `through` to `b`. Creates the arc (counter-clockwise from `a` to `b`
/// when `a → through → b` turns left, otherwise from `b` to `a`) with its three points; no
/// constraints, DOF 5. `Err(Degenerate)` if the points are collinear or coincide.
pub fn arc_3pt(
    s: &mut Sketch,
    a: Point2,
    through: Point2,
    b: Point2,
) -> Result<Built, SketchError> {
    finite(&[a, through, b])?;
    let o = circumcenter(a, through, b)
        .ok_or(SketchError::Degenerate("the three points are collinear"))?;
    let ccw = (through - a).perp_dot(b - through) > 0.0;
    let (start, end) = if ccw { (a, b) } else { (b, a) };
    run(s, |k| {
        let c = k.point(o)?;
        let sp = k.point(start)?;
        let ep = k.point(end)?;
        k.arc(c, sp, ep, false)?;
        Ok(())
    })
}

/// A locus for the centre of a circle tangent to an entity.
#[derive(Copy, Clone)]
enum Locus {
    Line { p: Point2, dir: Vec2 },
    Circle { c: Point2, r: f64 },
}

fn loci(s: &Sketch, e: EntityId, r: f64) -> Result<Vec<Locus>, SketchError> {
    let kind = s
        .entity(e)
        .ok_or(SketchError::UnknownEntity(e))?
        .kind
        .clone();
    let pt = |id: EntityId| s.point(id).ok_or(SketchError::UnknownEntity(id));
    match kind {
        EntityKind::Line { start, end } => {
            let (p, q) = (pt(start)?, pt(end)?);
            let dir = unit(q - p).ok_or(SketchError::Degenerate("tangent line has zero length"))?;
            let n = dir.perp() * r;
            Ok(vec![
                Locus::Line { p: p + n, dir },
                Locus::Line { p: p - n, dir },
            ])
        }
        EntityKind::Circle { center, radius } => {
            circle_loci(pt(center)?, s.params().get(radius), r)
        }
        EntityKind::Arc { center, start, .. } => {
            let c = pt(center)?;
            circle_loci(c, pt(start)?.distance(c), r)
        }
        k @ EntityKind::Point { .. } => Err(SketchError::WrongEntityKind {
            entity: e,
            expected: "line, circle or arc",
            found: k.name(),
        }),
    }
}

fn circle_loci(c: Point2, big: f64, r: f64) -> Result<Vec<Locus>, SketchError> {
    let mut v = vec![Locus::Circle { c, r: big + r }];
    if (big - r).abs() > TOL.linear {
        v.push(Locus::Circle {
            c,
            r: (big - r).abs(),
        });
    }
    Ok(v)
}

fn intersect_loci(a: Locus, b: Locus) -> Vec<Point2> {
    match (a, b) {
        (Locus::Line { p: p1, dir: d1 }, Locus::Line { p: p2, dir: d2 }) => {
            let den = d1.perp_dot(d2);
            if den.abs() <= TOL.angular {
                return Vec::new();
            }
            let t = (p2 - p1).perp_dot(d2) / den;
            vec![p1 + d1 * t]
        }
        (Locus::Line { p, dir }, Locus::Circle { c, r })
        | (Locus::Circle { c, r }, Locus::Line { p, dir }) => {
            let foot = p + dir * (c - p).dot(dir);
            let dist = (c - foot).length();
            let h2 = r * r - dist * dist;
            if h2 < -TOL.linear * r.max(1.0) {
                return Vec::new();
            }
            let h = h2.max(0.0).sqrt();
            if h <= TOL.linear {
                vec![foot]
            } else {
                vec![foot + dir * h, foot - dir * h]
            }
        }
        (Locus::Circle { c: c1, r: r1 }, Locus::Circle { c: c2, r: r2 }) => {
            let d = c1.distance(c2);
            if TOL.is_zero_len(d) {
                return Vec::new();
            }
            let a = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
            let h2 = r1 * r1 - a * a;
            if h2 < -TOL.linear * r1.max(1.0) {
                return Vec::new();
            }
            let h = h2.max(0.0).sqrt();
            let u = (c2 - c1) / d;
            let m = c1 + u * a;
            if h <= TOL.linear {
                vec![m]
            } else {
                vec![m + u.perp() * h, m - u.perp() * h]
            }
        }
    }
}

/// Circle of radius `r` tangent to the curves `a` and `b` (lines, circles or arcs; arcs and
/// lines count as their full circle / infinite line), choosing the solution whose centre is
/// closest to `near`.
///
/// Creates the circle and its centre, two `Tangent` constraints (the `External`/`Internal`
/// kind is chosen from the geometry) and a driving `Radius` dimension. DOF 0 for the new
/// circle. `Err(InvalidValue)` unless `r > 0`; `Err(Degenerate)` if no tangent circle of
/// that radius exists (or there are infinitely many, as for two parallel lines `2r` apart);
/// `Err(NotApplicable)` if `a == b`; `Err(WrongEntityKind)` for a point.
pub fn circle_2_tangent(
    s: &mut Sketch,
    a: EntityId,
    b: EntityId,
    r: f64,
    near: Point2,
) -> Result<Built, SketchError> {
    finite(&[near])?;
    if !r.is_finite() || r <= 0.0 {
        return Err(SketchError::InvalidValue {
            what: "circle radius",
            value: r,
        });
    }
    if a == b {
        return Err(SketchError::NotApplicable(
            "a circle cannot be tangent to one curve twice",
        ));
    }
    let (la, lb) = (loci(s, a, r)?, loci(s, b, r)?);
    let mut best: Option<(f64, Point2)> = None;
    for &x in &la {
        for &y in &lb {
            for p in intersect_loci(x, y) {
                let d = p.distance(near);
                if best.is_none_or(|(bd, _)| d < bd) {
                    best = Some((d, p));
                }
            }
        }
    }
    let (_, center) = best.ok_or(SketchError::Degenerate("no tangent circle of that radius"))?;
    run(s, |k| {
        let c = k.point(center)?;
        let circle = k.circle(c, r, false)?;
        for e in [a, b] {
            k.con_oriented(ConstraintKind::Tangent {
                a: e,
                b: circle,
                kind: TangentKind::External,
            })?;
        }
        k.con(ConstraintKind::Dimension(Dimension {
            kind: DimensionKind::Radius { curve: circle },
            value: r,
            driving: true,
            label_pos: center,
        }))?;
        Ok(())
    })
}

/// Arc tangent to the line or arc `from` at its end `at`, ending at `end`.
///
/// The new arc starts (or, if it turns clockwise, ends) at the existing endpoint entity of
/// `from`, so the two share that point; an `AtEndpoint` tangent constraint keeps them smooth.
/// DOF of the new arc: 2 (centre position along the normal, end point along its circle).
/// `Err(Degenerate)` if `end` is at the endpoint or on the tangent line through it;
/// `Err(WrongEntityKind)` unless `from` is a line or an arc.
pub fn arc_tangent(
    s: &mut Sketch,
    from: EntityId,
    at: End,
    end: Point2,
) -> Result<Built, SketchError> {
    finite(&[end])?;
    let kind = s
        .entity(from)
        .ok_or(SketchError::UnknownEntity(from))?
        .kind
        .clone();
    let pt = |id: EntityId| s.point(id).ok_or(SketchError::UnknownEntity(id));
    // Direction of travel at the chosen end.
    let (anchor, travel) = match kind {
        EntityKind::Line { start, end: e } => {
            let (p, q) = (pt(start)?, pt(e)?);
            let d = unit(q - p).ok_or(SketchError::Degenerate("line has zero length"))?;
            (if at == End::Start { start } else { e }, d)
        }
        EntityKind::Arc {
            center,
            start,
            end: e,
        } => {
            let c = pt(center)?;
            let id = if at == End::Start { start } else { e };
            let r = unit(pt(id)? - c).ok_or(SketchError::Degenerate("arc has zero radius"))?;
            (id, r.perp())
        }
        k => {
            return Err(SketchError::WrongEntityKind {
                entity: from,
                expected: "line or arc",
                found: k.name(),
            });
        }
    };
    let outward = if at == End::Start { -travel } else { travel };
    let p = pt(anchor)?;
    let w = end - p;
    let len = w.length();
    let normal = outward.perp();
    let k_n = w.dot(normal);
    if TOL.is_zero_len(len) || k_n.abs() <= TOL.angular * len {
        return Err(SketchError::Degenerate("arc end is on the tangent line"));
    }
    let rho = len * len / (2.0 * k_n);
    let centre = p + normal * rho;
    let ccw = rho > 0.0;
    run(s, |k| {
        let c = k.point(centre)?;
        let q = k.point(end)?;
        let (arc, b_end) = if ccw {
            (k.arc(c, anchor, q, false)?, End::Start)
        } else {
            (k.arc(c, q, anchor, false)?, End::End)
        };
        k.tangent_at(from, at, arc, b_end)?;
        Ok(())
    })
}
