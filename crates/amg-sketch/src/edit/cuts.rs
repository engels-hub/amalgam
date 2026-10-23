//! Curve–curve intersection helpers for trim, extend and break (SK-07).
//!
//! Everything here is read-only: it finds *where* an entity can be cut or extended, and the
//! point entity (if any) the new end should be glued to. `rebuild` does the mutation.

use std::f64::consts::TAU;

use amg_core::{Point2, Segment2, Tolerance};

use crate::geom::{Circle2, Curve2, intersect_detailed};
use crate::ids::EntityId;
use crate::model::{End, EntityKind, Sketch};

pub(crate) const TOL: Tolerance = Tolerance::DEFAULT;

/// A hit this close to an existing end point of the cutter glues to that point
/// (`Coincident`); farther away the new point is only put `PointOnCurve` the cutter.
const SNAP: f64 = 1e-9;

/// Hits closer than this to an end of the cut curve, or to each other, do not count.
pub(crate) const MERGE: f64 = 10.0 * 1e-6;

/// One place where a curve can be cut (or, for extend, reached).
#[derive(Copy, Clone, Debug)]
pub(crate) struct Cut {
    /// Parameter on the cut curve (`Curve2::eval` domain; beyond the range for extend).
    pub t: f64,
    /// Where the new point goes.
    pub point: Point2,
    /// The curve that causes the cut.
    pub cutter: EntityId,
    /// An end point of the cutter at the hit; the new point is made coincident with it.
    pub snap: Option<EntityId>,
}

/// The end point entities of a line or arc.
fn end_points(s: &Sketch, id: EntityId) -> Vec<EntityId> {
    s.entity(id)
        .map(|e| {
            [End::Start, End::End]
                .into_iter()
                .filter_map(|k| e.kind.endpoint(k))
                .collect()
        })
        .unwrap_or_default()
}

fn snap_point(s: &Sketch, cutter: EntityId, p: Point2) -> Option<EntityId> {
    end_points(s, cutter)
        .into_iter()
        .find(|&q| s.point(q).is_some_and(|qp| qp.distance(p) <= SNAP))
}

/// Interior cut points of `curve` (the evaluated geometry of `e`) against every other curve,
/// sorted by parameter.
///
/// Curves that overlap `curve` (collinear lines, co-circular arcs) are skipped. Hits at an end
/// of `curve` are skipped too: they are not interior cuts. Tangential contacts count.
pub(crate) fn cuts(s: &Sketch, e: EntityId, curve: &Curve2) -> Vec<Cut> {
    let (t0, t1) = curve.param_range();
    let ends: Vec<Point2> = if curve.is_closed() {
        Vec::new()
    } else {
        vec![curve.eval(t0), curve.eval(t1)]
    };
    let mut out: Vec<Cut> = Vec::new();
    for (id, ent) in s.entities() {
        if id == e || !ent.kind.is_curve() {
            continue;
        }
        let Some(other) = s.curve(id) else { continue };
        let det = intersect_detailed(curve, &other, &TOL);
        if det.overlap {
            continue;
        }
        for h in det.hits {
            if ends.iter().any(|p| p.distance(h.point) <= MERGE)
                || out.iter().any(|c| c.point.distance(h.point) <= MERGE)
            {
                continue;
            }
            out.push(Cut {
                t: h.ta,
                point: h.point,
                cutter: id,
                snap: snap_point(s, id, h.point),
            });
        }
    }
    out.sort_by(|a, b| a.t.total_cmp(&b.t));
    out
}

/// Where the new point of `cut` goes: on a snapped end point of the cutter, else on `curve`
/// exactly (so the piece's own constraints, e.g. `Horizontal`, hold to rounding).
pub(crate) fn place(s: &Sketch, curve: &Curve2, cut: &Cut) -> Point2 {
    cut.snap
        .and_then(|q| s.point(q))
        .unwrap_or_else(|| curve.eval(cut.t))
}

/// The nearest cutter along the extension of `e` past its `end`.
///
/// Lines extend along their direction, arcs along their circle (never past their own other
/// end). Hits within linear tolerance of the current end are already touching and are skipped.
/// `Cut::t` is the parameter of the new end on `e` (`> 1` / `< 0` for lines, `> sweep` / `< 0`
/// for arcs).
pub(crate) fn extension_target(s: &Sketch, e: EntityId, curve: &Curve2, end: End) -> Option<Cut> {
    let reach = 2.0 * crate::solve::characteristic_length(s) + curve.length();
    let mut best: Option<(f64, Cut)> = None;
    let mut consider = |dist: f64, cut: Cut| {
        if best.as_ref().is_none_or(|(d, _)| dist < *d) {
            best = Some((dist, cut));
        }
    };
    match curve {
        Curve2::Line(seg) => {
            let d = seg.b - seg.a;
            let len = d.length();
            if len <= 0.0 {
                return None;
            }
            let (origin, dir) = match end {
                End::End => (seg.b, d / len),
                End::Start => (seg.a, -(d / len)),
            };
            let ray = Curve2::Line(Segment2 {
                a: origin,
                b: origin + dir * reach,
            });
            for (id, ent) in s.entities() {
                if id == e || !ent.kind.is_curve() {
                    continue;
                }
                let Some(other) = s.curve(id) else { continue };
                let det = intersect_detailed(&ray, &other, &TOL);
                if det.overlap {
                    continue;
                }
                for h in det.hits {
                    let dist = h.ta * reach;
                    if dist <= MERGE {
                        continue;
                    }
                    let t = match end {
                        End::End => 1.0 + dist / len,
                        End::Start => -dist / len,
                    };
                    consider(
                        dist,
                        Cut {
                            t,
                            point: h.point,
                            cutter: id,
                            snap: snap_point(s, id, h.point),
                        },
                    );
                }
            }
        }
        Curve2::Arc(arc) => {
            let circle = Curve2::Circle(Circle2 {
                center: arc.center,
                radius: arc.radius,
            });
            let span = arc.sweep.abs();
            let eps = MERGE / arc.radius;
            let theta_start = arc.start_angle;
            let theta_end = arc.start_angle + arc.sweep;
            for (id, ent) in s.entities() {
                if id == e || !ent.kind.is_curve() {
                    continue;
                }
                let Some(other) = s.curve(id) else { continue };
                let det = intersect_detailed(&circle, &other, &TOL);
                if det.overlap {
                    continue;
                }
                for h in det.hits {
                    let theta = h.ta;
                    let delta = match end {
                        End::End => (theta - theta_end).rem_euclid(TAU),
                        End::Start => (theta_start - theta).rem_euclid(TAU),
                    };
                    if delta <= eps || delta >= TAU - span - eps {
                        continue;
                    }
                    let t = match end {
                        End::End => span + delta,
                        End::Start => -delta,
                    };
                    consider(
                        delta,
                        Cut {
                            t,
                            point: h.point,
                            cutter: id,
                            snap: snap_point(s, id, h.point),
                        },
                    );
                }
            }
        }
        // Circles have no end; ellipses are not editable (`is_cuttable`), so extension of
        // one is never requested.
        Curve2::Circle(_) | Curve2::Ellipse(_) => return None,
    }
    best.map(|(_, c)| c)
}

/// Parameter of `p` on `curve` without clamping: the line projection parameter, or the polar
/// angle measured from the arc's start (`[0, 2π)`; circles: from `+x`).
pub(crate) fn raw_param(curve: &Curve2, p: Point2) -> f64 {
    match curve {
        Curve2::Line(seg) => {
            let d = seg.b - seg.a;
            (p - seg.a).dot(d) / d.length_squared()
        }
        Curve2::Circle(c) => {
            let v = p - c.center;
            v.y.atan2(v.x).rem_euclid(TAU)
        }
        Curve2::Arc(a) => {
            let v = p - a.center;
            (v.y.atan2(v.x) - a.start_angle).rem_euclid(TAU)
        }
        Curve2::Ellipse(e) => (e.sign() * (e.angle_of(p) - e.t0)).rem_euclid(TAU),
    }
}

/// Entity kinds that trim, extend and break accept. Ellipses and elliptical arcs are cut *by*
/// these operations (they are valid cutters) but cannot be edited themselves yet.
pub(crate) fn is_cuttable(kind: &EntityKind) -> bool {
    matches!(
        kind,
        EntityKind::Line { .. } | EntityKind::Circle { .. } | EntityKind::Arc { .. }
    )
}
