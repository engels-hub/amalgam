//! Sketch fillet and chamfer (SK-16).
//!
//! Both operations replace the corner between two lines/arcs. Each curve is cut at a new
//! point (SK-07's `rebuild::apply`, so its constraints migrate by the table there); the
//! corner is closed by a new arc (fillet) or a new line (chamfer) that **shares** those points
//! with the trimmed pieces.
//!
//! # Rules
//!
//! - **Kept side.** Each curve keeps the part on the far side of its cut point, away from the
//!   corner (the fillet arc, or the chamfer chord, departs towards the corner). The rest of the
//!   curve is removed, so filleting a T or an X junction also drops the part of the through
//!   curve that lies beyond the junction on the corner's side.
//! - **Which corner.** The curves are treated as their full carrier lines/circles to find
//!   candidate corners; a candidate is valid only when every cut point lies on the actual
//!   curve and each curve keeps a part of positive length. Among valid candidates the one whose
//!   arc (fillet) or chord (chamfer) is nearest to `near` wins. No valid candidate is
//!   `NotApplicable` (radius too large, parallel curves, curves that do not reach).
//! - **Fillet constraints.** `Tangent` (`AtEndpoint`, chirality captured) between each piece
//!   and the fillet arc, plus a driving `Radius` dimension. Net DOF of the sketch is unchanged
//!   when the two curves met at a shared corner point.
//! - **Chamfer constraints.** A construction point `X` at the old corner, `PointOnCurve` on
//!   both pieces, and construction legs `X -> Pa`, `X -> Pb`. `Equal(d)`: `Equal` legs plus a
//!   `Length` on one; `TwoDistances`: a `Length` on each leg; `DistanceAngle(d, a)`: a `Length`
//!   on the first leg plus an `Angle` between the chamfer line and that leg (the interior angle
//!   at `Pa`, in `(0, pi)`). Net DOF is unchanged for a joined corner.
//! - Circles and projected curves are rejected (`NotApplicable`, `Locked`).

use amg_core::{Point2, Vec2};

use super::cuts::MERGE;
use super::rebuild::{Bnd, PieceSpec, Plan, apply};
use super::{DroppedConstraint, EditOutcome, EditResult, MigrationReport, checked};
use crate::error::SketchError;
use crate::geom::{Arc2, Curve2};
use crate::ids::EntityId;
use crate::model::{
    AngleBranch, ConstraintKind, Dimension, DimensionKind, End, Entity, EntityKind, Sketch,
    TangentKind,
};

/// How a chamfer is sized.
#[derive(Copy, Clone, Debug)]
pub enum ChamferSpec {
    /// Both legs, measured from the corner along each curve, are `d` long.
    Equal(f64),
    /// `d1` on the first curve, `d2` on the second.
    TwoDistances(f64, f64),
    /// `d` on the first curve and the angle (rad, in `(0, pi)`) between the chamfer line and
    /// that curve's leg at the cut point.
    DistanceAngle(f64, f64),
}

/// A point is on a curve when it is this close.
const ON: f64 = 1e-7;

struct Operand {
    ent: Entity,
    curve: Curve2,
}

fn operand(s: &Sketch, id: EntityId) -> Result<Operand, SketchError> {
    let (ent, curve) = checked(s, id)?;
    match curve {
        Curve2::Circle(_) => Err(SketchError::NotApplicable(
            "a full circle has no ends to fillet or chamfer",
        )),
        Curve2::Line(seg) if seg.a.distance(seg.b) <= MERGE => {
            Err(SketchError::Degenerate("zero-length line"))
        }
        _ => Ok(Operand { ent, curve }),
    }
}

fn scale_of(c: &Curve2) -> f64 {
    match c {
        Curve2::Line(seg) => seg.a.distance(seg.b),
        Curve2::Arc(a) => a.radius,
        Curve2::Circle(c) => c.radius,
    }
}

/// `p` lies on the curve; returns its parameter.
fn param_on(c: &Curve2, p: Point2) -> Option<f64> {
    let (t, q) = c.closest(p);
    (q.distance(p) <= ON).then_some(t)
}

/// Whether cutting at parameter `t` leaves a piece of positive length on the kept side.
fn kept_ok(c: &Curve2, t: f64, keep_start: bool) -> bool {
    let tend = c.param_range().1;
    let eps = MERGE / scale_of(c).max(1e-9);
    (if keep_start { t } else { tend - t }) > eps
}

// ---- carriers and intersections -------------------------------------------------------------

#[derive(Copy, Clone, Debug)]
enum Carrier {
    Line { p: Point2, u: Vec2 },
    Circle { o: Point2, r: f64 },
}

fn carrier(c: &Curve2) -> Carrier {
    match c {
        Curve2::Line(seg) => Carrier::Line {
            p: seg.a,
            u: (seg.b - seg.a).normalize(),
        },
        Curve2::Arc(a) => Carrier::Circle {
            o: a.center,
            r: a.radius,
        },
        Curve2::Circle(c) => Carrier::Circle {
            o: c.center,
            r: c.radius,
        },
    }
}

fn line_line(p1: Point2, u1: Vec2, p2: Point2, u2: Vec2) -> Vec<Point2> {
    let den = u1.perp_dot(u2);
    if den.abs() < 1e-12 {
        return vec![];
    }
    let t = (p2 - p1).perp_dot(u2) / den;
    vec![p1 + u1 * t]
}

fn line_circle(p: Point2, u: Vec2, o: Point2, r: f64) -> Vec<Point2> {
    let f = p - o;
    let b = f.dot(u);
    let disc = b * b - (f.dot(f) - r * r);
    if disc < -1e-12 * r.max(1.0) * r.max(1.0) {
        return vec![];
    }
    let h = disc.max(0.0).sqrt();
    if h < 1e-9 {
        vec![p + u * -b]
    } else {
        vec![p + u * (-b - h), p + u * (-b + h)]
    }
}

fn circle_circle(o1: Point2, r1: f64, o2: Point2, r2: f64) -> Vec<Point2> {
    let d = o1.distance(o2);
    if d < 1e-12 {
        return vec![];
    }
    let e = (o2 - o1) / d;
    let a = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
    let h2 = r1 * r1 - a * a;
    if h2 < -1e-12 * r1.max(1.0) * r1.max(1.0) {
        return vec![];
    }
    let h = h2.max(0.0).sqrt();
    let base = o1 + e * a;
    if h < 1e-9 {
        vec![base]
    } else {
        vec![base + e.perp() * h, base - e.perp() * h]
    }
}

fn intersect_carriers(a: Carrier, b: Carrier) -> Vec<Point2> {
    match (a, b) {
        (Carrier::Line { p: p1, u: u1 }, Carrier::Line { p: p2, u: u2 }) => {
            line_line(p1, u1, p2, u2)
        }
        (Carrier::Line { p, u }, Carrier::Circle { o, r })
        | (Carrier::Circle { o, r }, Carrier::Line { p, u }) => line_circle(p, u, o, r),
        (Carrier::Circle { o: o1, r: r1 }, Carrier::Circle { o: o2, r: r2 }) => {
            circle_circle(o1, r1, o2, r2)
        }
    }
}

/// Points where the ray `p + s w` (unit `w`, `s > 0`) meets the carrier.
fn ray_hits(p: Point2, w: Vec2, c: Carrier) -> Vec<Point2> {
    let hits = match c {
        Carrier::Line { p: q, u } => {
            let den = w.perp_dot(u);
            if den.abs() < 1e-12 {
                vec![]
            } else {
                vec![p + w * ((q - p).perp_dot(u) / den)]
            }
        }
        Carrier::Circle { o, r } => line_circle(p, w, o, r),
    };
    hits.into_iter()
        .filter(|h| (*h - p).dot(w) > MERGE)
        .collect()
}

/// Points of `curve` at straight-line distance `d` from `x` (a point of the curve's carrier).
fn at_distance(curve: &Curve2, x: Point2, d: f64) -> Vec<Point2> {
    let pts = match carrier(curve) {
        Carrier::Line { u, .. } => vec![x + u * d, x - u * d],
        Carrier::Circle { o, r } => circle_circle(o, r, x, d),
    };
    pts.into_iter()
        .filter_map(|q| param_on(curve, q).map(|t| curve.eval(t)))
        .collect()
}

// ---- results ----------------------------------------------------------------------------------

/// Merges the migration reports of the two curve edits: chains old -> mid -> new are
/// collapsed, and only constraints that existed before the whole operation are reported.
fn merge_migration(before: &Sketch, a: MigrationReport, b: MigrationReport) -> MigrationReport {
    let mut migrated = a.migrated;
    for (o, n) in b.migrated {
        if let Some(slot) = migrated.iter_mut().find(|(_, m)| *m == o) {
            slot.1 = n;
        } else {
            migrated.push((o, n));
        }
    }
    let mut dropped: Vec<DroppedConstraint> = a.dropped;
    for d in b.dropped {
        if before.constraint(d.id).is_some() {
            dropped.push(d);
        } else if let Some(pos) = migrated.iter().position(|(_, m)| *m == d.id) {
            let (o, _) = migrated.remove(pos);
            dropped.push(DroppedConstraint { id: o, ..d });
        }
    }
    MigrationReport { migrated, dropped }
}

/// Created/removed ids by diffing (ids are never reused).
fn diff(before: &Sketch, after: &Sketch) -> EditResult {
    EditResult {
        created: after
            .entities()
            .map(|(id, _)| id)
            .filter(|&id| before.entity(id).is_none())
            .collect(),
        removed: before
            .entities()
            .map(|(id, _)| id)
            .filter(|&id| after.entity(id).is_none())
            .collect(),
        constraints_added: after
            .constraints()
            .map(|(id, _)| id)
            .filter(|&id| before.constraint(id).is_none())
            .collect(),
        constraints_removed: before
            .constraints()
            .map(|(id, _)| id)
            .filter(|&id| after.constraint(id).is_none())
            .collect(),
    }
}

/// Cuts `op` (entity `id`) at `p`, whose point entity `pid` becomes the boundary of the kept
/// piece. Returns the new piece, which of its ends is `pid`, and the migration report.
fn trim_to(
    s: &mut Sketch,
    id: EntityId,
    op: &Operand,
    p: Point2,
    pid: EntityId,
    keep_start: bool,
) -> Result<(EntityId, End, MigrationReport), SketchError> {
    let (t, _) = op.curve.closest(p);
    let tend = op.curve.param_range().1;
    let (st, en) = match op.ent.kind {
        EntityKind::Line { start, end } | EntityKind::Arc { start, end, .. } => (start, end),
        _ => return Err(SketchError::NotApplicable("not a line or arc")),
    };
    let spec = if keep_start {
        PieceSpec {
            start: Bnd::Keep(st),
            end: Bnd::Keep(pid),
            t0: 0.0,
            len: t,
        }
    } else {
        PieceSpec {
            start: Bnd::Keep(pid),
            end: Bnd::Keep(en),
            t0: t,
            len: tend - t,
        }
    };
    let plan = Plan {
        points: vec![],
        pieces: vec![spec],
        link: None,
    };
    let out = apply(s, id, &plan)?;
    let piece = out
        .result
        .created
        .iter()
        .copied()
        .find(|&c| {
            s.entity(c)
                .is_some_and(|e| matches!(e.kind, EntityKind::Line { .. } | EntityKind::Arc { .. }))
        })
        .ok_or(SketchError::NotApplicable("no piece was created"))?;
    let at = if keep_start { End::End } else { End::Start };
    Ok((piece, at, out.migration))
}

fn dim(kind: DimensionKind, value: f64, at: Point2) -> ConstraintKind {
    ConstraintKind::Dimension(Dimension {
        kind,
        value,
        driving: true,
        label_pos: at,
    })
}

// ---- fillet -------------------------------------------------------------------------------------

#[derive(Copy, Clone)]
struct FilletGeom {
    center: Point2,
    ta: Point2,
    tb: Point2,
    /// `ta` is the fillet arc's start (the arc runs CCW from start to end).
    a_is_start: bool,
    keep_start_a: bool,
    keep_start_b: bool,
}

#[derive(Copy, Clone)]
enum Off {
    Line { p: Point2, u: Vec2 },
    Circle { o: Point2, r: f64 },
}

fn offsets(c: &Curve2, r: f64) -> Vec<Off> {
    match carrier(c) {
        Carrier::Line { p, u } => {
            let n = u.perp() * r;
            vec![Off::Line { p: p + n, u }, Off::Line { p: p - n, u }]
        }
        Carrier::Circle { o, r: rad } => {
            let mut v = vec![Off::Circle { o, r: rad + r }];
            if (rad - r).abs() > 1e-9 {
                v.push(Off::Circle {
                    o,
                    r: (rad - r).abs(),
                });
            }
            v
        }
    }
}

fn off_pair(a: Off, b: Off) -> Vec<Point2> {
    let as_c = |o: Off| match o {
        Off::Line { p, u } => Carrier::Line { p, u },
        Off::Circle { o, r } => Carrier::Circle { o, r },
    };
    intersect_carriers(as_c(a), as_c(b))
}

/// Tangent point of the fillet circle (centre `c`, radius `r`) with `curve`.
fn tangent_point(curve: &Curve2, c: Point2, r: f64) -> Option<Point2> {
    match carrier(curve) {
        Carrier::Line { p, u } => Some(p + u * (c - p).dot(u)),
        Carrier::Circle { o, r: rad } => {
            let d = c - o;
            let dist = d.length();
            if dist < 1e-12 {
                return None;
            }
            let external = (dist - (rad + r)).abs() <= (dist - (rad - r).abs()).abs();
            let dir = d / dist;
            Some(if external || rad > r {
                o + dir * rad
            } else {
                o - dir * rad
            })
        }
    }
}

fn solve_fillet(a: &Operand, b: &Operand, near: Point2, r: f64) -> Option<FilletGeom> {
    let mut best: Option<(f64, FilletGeom)> = None;
    for oa in offsets(&a.curve, r) {
        for ob in offsets(&b.curve, r) {
            for c in off_pair(oa, ob) {
                let (Some(ta), Some(tb)) =
                    (tangent_point(&a.curve, c, r), tangent_point(&b.curve, c, r))
                else {
                    continue;
                };
                let (Some(t_a), Some(t_b)) = (param_on(&a.curve, ta), param_on(&b.curve, tb))
                else {
                    continue;
                };
                let cross = (ta - c).perp_dot(tb - c);
                if cross.abs() < 1e-9 * r * r {
                    continue;
                }
                let a_is_start = cross > 0.0;
                let perp = |q: Point2| (q - c).perp() / r;
                let (dep_a, dep_b) = if a_is_start {
                    (perp(ta), -perp(tb))
                } else {
                    (-perp(ta), perp(tb))
                };
                let keep_start_a = a.curve.derivative(t_a).dot(dep_a) > 0.0;
                let keep_start_b = b.curve.derivative(t_b).dot(dep_b) > 0.0;
                if !kept_ok(&a.curve, t_a, keep_start_a) || !kept_ok(&b.curve, t_b, keep_start_b) {
                    continue;
                }
                let (st, en) = if a_is_start { (ta, tb) } else { (tb, ta) };
                let arc = Curve2::Arc(Arc2::from_center_start_end(c, st, en));
                let score = arc.closest(near).1.distance(near);
                if best.as_ref().is_none_or(|(bs, _)| score < *bs - 1e-12) {
                    best = Some((
                        score,
                        FilletGeom {
                            center: c,
                            ta,
                            tb,
                            a_is_start,
                            keep_start_a,
                            keep_start_b,
                        },
                    ));
                }
            }
        }
    }
    best.map(|(_, g)| g)
}

/// Rounds the corner of two lines/arcs with an arc of radius `r`.
///
/// `near` picks the corner when the curves allow several (an X or T junction, or an arc
/// cutting a line twice): the candidate fillet arc nearest to it wins. Both curves are
/// replaced by trimmed pieces (new ids) that are tangent to the new arc, which carries a
/// driving `Radius` dimension.
///
/// Errors: `UnknownEntity`, `WrongEntityKind`, `Locked`, `InvalidValue` (`r` not finite and
/// positive), `Degenerate` (zero-length line), `NotApplicable` (same curve twice, circles, or
/// no fillet of this radius fits: parallel curves, curves that do not reach the tangent points,
/// radius too large).
pub fn fillet(
    s: &mut Sketch,
    a: EntityId,
    b: EntityId,
    near: Point2,
    r: f64,
) -> Result<EditResult, SketchError> {
    fillet_detailed(s, a, b, near, r).map(|o| o.result)
}

/// [`fillet`] with the constraint migration report.
pub fn fillet_detailed(
    s: &mut Sketch,
    a: EntityId,
    b: EntityId,
    near: Point2,
    r: f64,
) -> Result<EditOutcome, SketchError> {
    if !(r.is_finite() && r > 0.0) {
        return Err(SketchError::InvalidValue {
            what: "fillet radius",
            value: r,
        });
    }
    if a == b {
        return Err(SketchError::NotApplicable(
            "a fillet needs two different curves",
        ));
    }
    let (oa, ob) = (operand(s, a)?, operand(s, b)?);
    let g = solve_fillet(&oa, &ob, near, r)
        .ok_or(SketchError::NotApplicable("no fillet of this radius fits"))?;
    let before = s.clone();
    match build_fillet(s, a, b, &oa, &ob, g, r) {
        Ok((ma, mb)) => Ok(EditOutcome {
            result: diff(&before, s),
            migration: merge_migration(&before, ma, mb),
        }),
        Err(e) => {
            *s = before;
            Err(e)
        }
    }
}

fn build_fillet(
    s: &mut Sketch,
    a: EntityId,
    b: EntityId,
    oa: &Operand,
    ob: &Operand,
    g: FilletGeom,
    r: f64,
) -> Result<(MigrationReport, MigrationReport), SketchError> {
    let (st, en) = if g.a_is_start {
        (g.ta, g.tb)
    } else {
        (g.tb, g.ta)
    };
    let arc = s.add_arc(g.center, st, en)?;
    let EntityKind::Arc { start, end, .. } = s
        .entity(arc)
        .ok_or(SketchError::UnknownEntity(arc))?
        .kind
        .clone()
    else {
        return Err(SketchError::NotApplicable("not an arc"));
    };
    let (pa, pb) = if g.a_is_start {
        (start, end)
    } else {
        (end, start)
    };
    let (piece_a, end_a, ma) = trim_to(s, a, oa, g.ta, pa, g.keep_start_a)?;
    let (piece_b, end_b, mb) = trim_to(s, b, ob, g.tb, pb, g.keep_start_b)?;
    for (piece, at, pid) in [(piece_a, end_a, pa), (piece_b, end_b, pb)] {
        let arc_end = if pid == start { End::Start } else { End::End };
        s.add_constraint_oriented(ConstraintKind::Tangent {
            a: piece,
            b: arc,
            kind: TangentKind::AtEndpoint {
                a_end: at,
                b_end: arc_end,
                reversed: false,
            },
        })?;
    }
    s.add_constraint_oriented(dim(DimensionKind::Radius { curve: arc }, r, g.center))?;
    Ok((ma, mb))
}

// ---- chamfer ------------------------------------------------------------------------------------

#[derive(Copy, Clone)]
struct ChamferGeom {
    x: Point2,
    pa: Point2,
    pb: Point2,
    keep_start_a: bool,
    keep_start_b: bool,
}

fn seg_distance(p: Point2, a: Point2, b: Point2) -> f64 {
    let d = b - a;
    let t = ((p - a).dot(d) / d.length_squared().max(1e-30)).clamp(0.0, 1.0);
    p.distance(a + d * t)
}

fn rotate(v: Vec2, ang: f64) -> Vec2 {
    let (sn, cs) = ang.sin_cos();
    Vec2::new(v.x * cs - v.y * sn, v.x * sn + v.y * cs)
}

fn solve_chamfer(a: &Operand, b: &Operand, near: Point2, spec: ChamferSpec) -> Option<ChamferGeom> {
    let mut best: Option<(f64, ChamferGeom)> = None;
    let (d1, d2) = match spec {
        ChamferSpec::Equal(d) => (d, d),
        ChamferSpec::TwoDistances(d1, d2) => (d1, d2),
        ChamferSpec::DistanceAngle(d, _) => (d, 0.0),
    };
    for x in intersect_carriers(carrier(&a.curve), carrier(&b.curve)) {
        for pa in at_distance(&a.curve, x, d1) {
            let pbs: Vec<Point2> = match spec {
                ChamferSpec::DistanceAngle(_, ang) => {
                    let to_x = (x - pa) / d1;
                    [ang, -ang]
                        .into_iter()
                        .flat_map(|sg| ray_hits(pa, rotate(to_x, sg), carrier(&b.curve)))
                        .collect()
                }
                _ => at_distance(&b.curve, x, d2),
            };
            for pb in pbs {
                let (Some(t_a), Some(t_b)) = (param_on(&a.curve, pa), param_on(&b.curve, pb))
                else {
                    continue;
                };
                if pa.distance(pb) <= MERGE {
                    continue;
                }
                let keep_start_a = a.curve.derivative(t_a).dot(x - pa) > 0.0;
                let keep_start_b = b.curve.derivative(t_b).dot(x - pb) > 0.0;
                if !kept_ok(&a.curve, t_a, keep_start_a) || !kept_ok(&b.curve, t_b, keep_start_b) {
                    continue;
                }
                let score = seg_distance(near, pa, pb);
                if best.as_ref().is_none_or(|(bs, _)| score < *bs - 1e-12) {
                    best = Some((
                        score,
                        ChamferGeom {
                            x,
                            pa,
                            pb,
                            keep_start_a,
                            keep_start_b,
                        },
                    ));
                }
            }
        }
    }
    best.map(|(_, g)| g)
}

/// Cuts the corner of two lines/arcs with a straight chamfer.
///
/// Distances are straight-line distances from the corner (the intersection of the curves'
/// carriers) to the cut points; see [`ChamferSpec`]. `near` picks among several corners or
/// cut points (the chamfer line nearest to it wins). Both curves are replaced by trimmed
/// pieces (new ids) joined by the chamfer line; a construction point at the old corner, on
/// both pieces, and construction legs to the cut points carry the dimensions.
///
/// Errors: as [`fillet`]; `InvalidValue` for a distance that is not finite and positive or an
/// angle outside `(0, pi)`; `NotApplicable` when a distance exceeds the curve, the curves are
/// parallel, or the angle ray misses the second curve.
pub fn chamfer(
    s: &mut Sketch,
    a: EntityId,
    b: EntityId,
    near: Point2,
    spec: ChamferSpec,
) -> Result<EditResult, SketchError> {
    chamfer_detailed(s, a, b, near, spec).map(|o| o.result)
}

/// [`chamfer`] with the constraint migration report.
pub fn chamfer_detailed(
    s: &mut Sketch,
    a: EntityId,
    b: EntityId,
    near: Point2,
    spec: ChamferSpec,
) -> Result<EditOutcome, SketchError> {
    let pos = |what: &'static str, v: f64| {
        if v.is_finite() && v > 0.0 {
            Ok(())
        } else {
            Err(SketchError::InvalidValue { what, value: v })
        }
    };
    match spec {
        ChamferSpec::Equal(d) => pos("chamfer distance", d)?,
        ChamferSpec::TwoDistances(d1, d2) => {
            pos("chamfer distance", d1)?;
            pos("chamfer distance", d2)?;
        }
        ChamferSpec::DistanceAngle(d, ang) => {
            pos("chamfer distance", d)?;
            if !(ang.is_finite() && ang > 0.0 && ang < std::f64::consts::PI) {
                return Err(SketchError::InvalidValue {
                    what: "chamfer angle",
                    value: ang,
                });
            }
        }
    }
    if a == b {
        return Err(SketchError::NotApplicable(
            "a chamfer needs two different curves",
        ));
    }
    let (oa, ob) = (operand(s, a)?, operand(s, b)?);
    let g = solve_chamfer(&oa, &ob, near, spec)
        .ok_or(SketchError::NotApplicable("no chamfer of this size fits"))?;
    let before = s.clone();
    match build_chamfer(s, a, b, &oa, &ob, g, spec) {
        Ok((ma, mb)) => Ok(EditOutcome {
            result: diff(&before, s),
            migration: merge_migration(&before, ma, mb),
        }),
        Err(e) => {
            *s = before;
            Err(e)
        }
    }
}

fn build_chamfer(
    s: &mut Sketch,
    a: EntityId,
    b: EntityId,
    oa: &Operand,
    ob: &Operand,
    g: ChamferGeom,
    spec: ChamferSpec,
) -> Result<(MigrationReport, MigrationReport), SketchError> {
    let x = s.add_point(g.x)?;
    s.set_construction(x, true)?;
    let pa = s.add_point(g.pa)?;
    let pb = s.add_point(g.pb)?;
    let (piece_a, _, ma) = trim_to(s, a, oa, g.pa, pa, g.keep_start_a)?;
    let (piece_b, _, mb) = trim_to(s, b, ob, g.pb, pb, g.keep_start_b)?;
    let line = s.add_line_between(pa, pb)?;
    let leg_a = s.add_line_between(x, pa)?;
    let leg_b = s.add_line_between(x, pb)?;
    s.set_construction(leg_a, true)?;
    s.set_construction(leg_b, true)?;
    s.add_constraint(ConstraintKind::PointOnCurve {
        point: x,
        curve: piece_a,
    })?;
    s.add_constraint(ConstraintKind::PointOnCurve {
        point: x,
        curve: piece_b,
    })?;
    let len = |l: EntityId, v: f64, at: Point2| dim(DimensionKind::Length { line: l }, v, at);
    let mid = |p: Point2| (g.x + p) * 0.5;
    match spec {
        ChamferSpec::Equal(d) => {
            s.add_constraint(len(leg_a, d, mid(g.pa)))?;
            s.add_constraint(ConstraintKind::Equal { a: leg_a, b: leg_b })?;
        }
        ChamferSpec::TwoDistances(d1, d2) => {
            s.add_constraint(len(leg_a, d1, mid(g.pa)))?;
            s.add_constraint(len(leg_b, d2, mid(g.pb)))?;
        }
        ChamferSpec::DistanceAngle(d, ang) => {
            s.add_constraint(len(leg_a, d, mid(g.pa)))?;
            s.add_constraint_oriented(dim(
                DimensionKind::Angle {
                    a: line,
                    b: leg_a,
                    branch: AngleBranch {
                        reverse_a: false,
                        reverse_b: false,
                    },
                },
                ang,
                g.pa,
            ))?;
        }
    }
    Ok((ma, mb))
}
