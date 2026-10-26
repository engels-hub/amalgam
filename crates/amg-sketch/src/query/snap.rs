//! Snap candidates around a cursor position (see [`QueryIndex::snap_candidates`]).

use std::f64::consts::{FRAC_PI_2, TAU};

use amg_core::{Point2, Tolerance};
use smallvec::SmallVec;

use crate::geom::{Curve2, intersect};
use crate::ids::EntityId;
use crate::model::{ConstraintKind as K, HvTarget, TangentKind};

use super::index::{PointRole, Shape};
use super::{NEW_ENTITY, QueryIndex, SnapCandidate, SnapKind, SnapOptions};

/// Coincidence tolerance between candidate points, mm.
const SAME: f64 = 1e-9;
/// Samples of the root search on a general curve.
const SAMPLES: usize = 64;
/// At most this many curves are intersected pairwise (the nearest ones).
const MAX_PAIR_CURVES: usize = 12;
/// At most this many alignment guides per axis.
const MAX_ALIGN: usize = 3;
/// Output cap.
const MAX_CANDIDATES: usize = 32;

fn rank(k: SnapKind) -> u8 {
    match k {
        SnapKind::Endpoint => 0,
        SnapKind::Center => 1,
        SnapKind::Midpoint => 2,
        SnapKind::Intersection => 3,
        SnapKind::Quadrant => 4,
        SnapKind::Tangent => 5,
        SnapKind::Perpendicular => 6,
        SnapKind::OnCurve => 7,
        SnapKind::AlignHorizontal | SnapKind::AlignVertical => 8,
    }
}

fn cand(
    kind: SnapKind,
    point: Point2,
    entities: &[EntityId],
    suggested: Option<K>,
    secondary: Option<K>,
) -> SnapCandidate {
    SnapCandidate {
        kind,
        point,
        entities: entities.iter().copied().collect(),
        suggested,
        secondary,
    }
}

impl QueryIndex {
    /// Snap candidates within `radius` of `p`, best first (kind priority, then distance).
    /// See [`snap_candidates`](super::snap_candidates).
    pub fn snap_candidates(
        &self,
        p: Point2,
        radius: f64,
        opts: &SnapOptions<'_>,
    ) -> Vec<SnapCandidate> {
        let np = opts.moving.unwrap_or(NEW_ENTITY);
        let nl = opts.rubber_band.unwrap_or(NEW_ENTITY);
        let skip = |id: EntityId| {
            Some(id) == opts.moving || Some(id) == opts.rubber_band || opts.exclude.contains(&id)
        };
        let mut out: Vec<SnapCandidate> = Vec::new();
        let mut close: SmallVec<[(usize, f64); 8]> = SmallVec::new();

        for i in self.near(p, radius) {
            let it = &self.items[i];
            if skip(it.id) {
                continue;
            }
            match &it.shape {
                Shape::Point(q, role) => {
                    if q.distance(p) <= radius {
                        let kind = if *role == PointRole::Center {
                            SnapKind::Center
                        } else {
                            SnapKind::Endpoint
                        };
                        let k = K::Coincident { a: np, b: it.id };
                        out.push(cand(kind, *q, &[it.id], Some(k), None));
                    }
                }
                Shape::Curve(c) => {
                    let (t, q) = c.closest(p);
                    let d = q.distance(p);
                    if d <= radius {
                        close.push((i, d));
                        let on = K::PointOnCurve {
                            point: np,
                            curve: it.id,
                        };
                        out.push(cand(SnapKind::OnCurve, q, &[it.id], Some(on), None));
                        on_curve_snaps(&mut out, it.id, c, t, p, radius, np);
                        if let Some(f) = opts.from {
                            from_snaps(&mut out, it.id, c, f, p, radius, nl);
                        }
                    }
                }
            }
        }

        intersections(self, &mut close, &mut out, p, radius, np);
        alignments(self, &mut out, p, radius, np, opts, &skip);
        finish(&mut out, p);
        out
    }
}

/// Midpoint and quadrant snaps of one curve near `p`.
fn on_curve_snaps(
    out: &mut Vec<SnapCandidate>,
    id: EntityId,
    c: &Curve2,
    _t: f64,
    p: Point2,
    r: f64,
    np: EntityId,
) {
    let on = || {
        Some(K::PointOnCurve {
            point: np,
            curve: id,
        })
    };
    match c {
        Curve2::Line(s) => {
            let m = (s.a + s.b) * 0.5;
            if m.distance(p) <= r {
                let k = K::Midpoint {
                    point: np,
                    line: id,
                };
                out.push(cand(SnapKind::Midpoint, m, &[id], Some(k), None));
            }
        }
        Curve2::Circle(_) | Curve2::Arc(_) => {
            if let Some(ro) = c.round() {
                if !ro.circle {
                    let m = ro.eval(ro.span() * 0.5);
                    if m.distance(p) <= r {
                        out.push(cand(SnapKind::Midpoint, m, &[id], None, None));
                    }
                }
                for k in 0..4 {
                    let phi = f64::from(k) * FRAC_PI_2;
                    if ro.circle || ro.param_of_angle(phi, 0.0).is_some() {
                        let q = ro.c + amg_core::Vec2::new(phi.cos(), phi.sin()) * ro.r;
                        if q.distance(p) <= r {
                            out.push(cand(SnapKind::Quadrant, q, &[id], on(), None));
                        }
                    }
                }
            }
        }
        Curve2::Ellipse(e) => {
            if !e.is_full() {
                let m = e.eval(e.span() * 0.5);
                if m.distance(p) <= r {
                    out.push(cand(SnapKind::Midpoint, m, &[id], None, None));
                }
            }
            for k in 0..4 {
                let theta = f64::from(k) * FRAC_PI_2;
                let along = (e.sign() * (theta - e.t0)).rem_euclid(TAU);
                if e.is_full() || along <= e.span() {
                    let q = e.point_at(theta);
                    if q.distance(p) <= r {
                        out.push(cand(SnapKind::Quadrant, q, &[id], on(), None));
                    }
                }
            }
        }
        Curve2::BSpline(b) => {
            let m = b.eval(b.param_at_length(b.length() * 0.5));
            if m.distance(p) <= r {
                out.push(cand(SnapKind::Midpoint, m, &[id], None, None));
            }
        }
    }
}

/// Perpendicular and tangent snaps of a rubber-band line from `f` to a point on curve `c`.
fn from_snaps(
    out: &mut Vec<SnapCandidate>,
    id: EntityId,
    c: &Curve2,
    f: Point2,
    p: Point2,
    r: f64,
    nl: EntityId,
) {
    let tangent_kind = |tp: Point2, center: Point2| {
        let d = tp - f;
        let len = d.length();
        if len > 0.0 && d.perp_dot(center - f) / len < 0.0 {
            TangentKind::Internal
        } else {
            TangentKind::External
        }
    };
    match c {
        Curve2::Line(s) => {
            let d = s.b - s.a;
            let ll = d.length_squared();
            if ll > 0.0 {
                let t = (f - s.a).dot(d) / ll;
                let foot = s.a + d * t;
                if (0.0..=1.0).contains(&t) && foot.distance(p) <= r && foot.distance(f) > SAME {
                    let k = K::Perpendicular { a: nl, b: id };
                    out.push(cand(SnapKind::Perpendicular, foot, &[id], Some(k), None));
                }
            }
        }
        Curve2::Circle(_) | Curve2::Arc(_) => {
            let Some(ro) = c.round() else { return };
            let u = f - ro.c;
            let d = u.length();
            if d <= SAME {
                return;
            }
            for sgn in [1.0, -1.0] {
                let q = ro.c + u * (sgn * ro.r / d);
                let ang = (q.y - ro.c.y).atan2(q.x - ro.c.x);
                if (ro.circle || ro.param_of_angle(ang, 0.0).is_some()) && q.distance(p) <= r {
                    out.push(cand(SnapKind::Perpendicular, q, &[id], None, None));
                }
            }
            if d > ro.r * (1.0 + 1e-9) {
                let base = u.y.atan2(u.x);
                let del = (ro.r / d).acos();
                for sgn in [-1.0, 1.0] {
                    let ang = base + sgn * del;
                    if !(ro.circle || ro.param_of_angle(ang, 0.0).is_some()) {
                        continue;
                    }
                    let q = ro.c + amg_core::Vec2::new(ang.cos(), ang.sin()) * ro.r;
                    if q.distance(p) <= r {
                        let k = K::Tangent {
                            a: nl,
                            b: id,
                            kind: tangent_kind(q, ro.c),
                        };
                        out.push(cand(SnapKind::Tangent, q, &[id], Some(k), None));
                    }
                }
            }
        }
        Curve2::Ellipse(_) | Curve2::BSpline(_) => {
            let tan = |t: f64| (c.eval(t) - f).perp_dot(c.derivative(t));
            let perp = |t: f64| (c.eval(t) - f).dot(c.derivative(t));
            for t in roots(c, &tan) {
                let q = c.eval(t);
                if q.distance(p) <= r && q.distance(f) > SAME {
                    let k = match c {
                        Curve2::Ellipse(e) => Some(K::Tangent {
                            a: nl,
                            b: id,
                            kind: tangent_kind(q, e.center),
                        }),
                        _ => None,
                    };
                    out.push(cand(SnapKind::Tangent, q, &[id], k, None));
                }
            }
            for t in roots(c, &perp) {
                let q = c.eval(t);
                if q.distance(p) <= r && q.distance(f) > SAME {
                    out.push(cand(SnapKind::Perpendicular, q, &[id], None, None));
                }
            }
        }
    }
}

/// Parameters in the curve's range where `g` changes sign (bisection on a uniform sampling).
fn roots(c: &Curve2, g: &dyn Fn(f64) -> f64) -> SmallVec<[f64; 4]> {
    let (t0, t1) = c.param_range();
    let mut found = SmallVec::new();
    let step = (t1 - t0) / SAMPLES as f64;
    let (mut ta, mut ga) = (t0, g(t0));
    for k in 1..=SAMPLES {
        let tb = t0 + step * k as f64;
        let gb = g(tb);
        if (ga < 0.0) != (gb < 0.0) {
            let (mut lo, mut hi, mut glo) = (ta, tb, ga);
            for _ in 0..48 {
                let mid = 0.5 * (lo + hi);
                let gm = g(mid);
                if (glo < 0.0) == (gm < 0.0) {
                    lo = mid;
                    glo = gm;
                } else {
                    hi = mid;
                }
            }
            found.push(0.5 * (lo + hi));
        }
        (ta, ga) = (tb, gb);
    }
    found
}

/// Intersections of the curves near `p`.
fn intersections(
    ix: &QueryIndex,
    close: &mut SmallVec<[(usize, f64); 8]>,
    out: &mut Vec<SnapCandidate>,
    p: Point2,
    r: f64,
    np: EntityId,
) {
    if close.len() < 2 {
        return;
    }
    close.sort_by(|a, b| a.1.total_cmp(&b.1));
    close.truncate(MAX_PAIR_CURVES);
    close.sort_by_key(|c| c.0);
    let tol = Tolerance::DEFAULT;
    for (n, &(i, _)) in close.iter().enumerate() {
        for &(j, _) in &close[n + 1..] {
            let (a, b) = (&ix.items[i], &ix.items[j]);
            let (Shape::Curve(ca), Shape::Curve(cb)) = (&a.shape, &b.shape) else {
                continue;
            };
            if !ca.bbox().intersects(&cb.bbox(), &tol) {
                continue;
            }
            for h in intersect(ca, cb, &tol) {
                if h.point.distance(p) <= r {
                    let on = |curve| Some(K::PointOnCurve { point: np, curve });
                    out.push(cand(
                        SnapKind::Intersection,
                        h.point,
                        &[a.id, b.id],
                        on(a.id),
                        on(b.id),
                    ));
                }
            }
        }
    }
}

/// Horizontal and vertical alignment with the rubber-band anchor and other points.
fn alignments(
    ix: &QueryIndex,
    out: &mut Vec<SnapCandidate>,
    p: Point2,
    r: f64,
    np: EntityId,
    opts: &SnapOptions<'_>,
    skip: &dyn Fn(EntityId) -> bool,
) {
    let anchor = opts
        .from
        .and_then(|f| opts.from_entity.or_else(|| ix.point_at(f)));
    if let Some(f) = opts.from {
        let ents: SmallVec<[EntityId; 1]> = anchor.into_iter().collect();
        let hv = |h: bool| {
            anchor.map(|a| {
                let t = HvTarget::Points(a, np);
                if h { K::Horizontal(t) } else { K::Vertical(t) }
            })
        };
        if (f.y - p.y).abs() <= r {
            out.push(cand(
                SnapKind::AlignHorizontal,
                Point2::new(p.x, f.y),
                &ents,
                hv(true),
                None,
            ));
        }
        if (f.x - p.x).abs() <= r {
            out.push(cand(
                SnapKind::AlignVertical,
                Point2::new(f.x, p.y),
                &ents,
                hv(false),
                None,
            ));
        }
    }
    let mut hs: SmallVec<[(f64, EntityId); 4]> = SmallVec::new();
    let mut vs: SmallVec<[(f64, EntityId); 4]> = SmallVec::new();
    let mut pts: SmallVec<[(EntityId, Point2); 8]> = SmallVec::new();
    for &i in &ix.points {
        let it = &ix.items[i as usize];
        let Shape::Point(q, _) = it.shape else {
            continue;
        };
        if skip(it.id) || Some(it.id) == anchor {
            continue;
        }
        let (dx, dy) = ((q.x - p.x).abs(), (q.y - p.y).abs());
        if dy <= r && dx > r {
            hs.push((dy, it.id));
            pts.push((it.id, q));
        }
        if dx <= r && dy > r {
            vs.push((dx, it.id));
            pts.push((it.id, q));
        }
    }
    let pos = |id: EntityId| pts.iter().find(|(e, _)| *e == id).map(|(_, q)| *q);
    hs.sort_by(|a, b| a.0.total_cmp(&b.0));
    vs.sort_by(|a, b| a.0.total_cmp(&b.0));
    for &(_, id) in hs.iter().take(MAX_ALIGN) {
        if let Some(q) = pos(id) {
            let k = K::Horizontal(HvTarget::Points(id, np));
            out.push(cand(
                SnapKind::AlignHorizontal,
                Point2::new(p.x, q.y),
                &[id],
                Some(k),
                None,
            ));
        }
    }
    for &(_, id) in vs.iter().take(MAX_ALIGN) {
        if let Some(q) = pos(id) {
            let k = K::Vertical(HvTarget::Points(id, np));
            out.push(cand(
                SnapKind::AlignVertical,
                Point2::new(q.x, p.y),
                &[id],
                Some(k),
                None,
            ));
        }
    }
}

/// True if candidate `c` adds nothing next to `d` at the same point.
fn covered(c: &SnapCandidate, d: &SnapCandidate) -> bool {
    use SnapKind::{Center, Endpoint, Intersection, Midpoint, OnCurve, Quadrant};
    if c.point.distance(d.point) > SAME {
        return false;
    }
    match c.kind {
        OnCurve => !matches!(
            d.kind,
            OnCurve | SnapKind::AlignHorizontal | SnapKind::AlignVertical
        ),
        Quadrant => matches!(d.kind, Endpoint | Center | Midpoint | Intersection),
        Intersection => matches!(d.kind, Endpoint | Center),
        _ => false,
    }
}

/// Removes covered and duplicate candidates, sorts best first and caps the list.
fn finish(out: &mut Vec<SnapCandidate>, p: Point2) {
    let keep: Vec<bool> = (0..out.len())
        .map(|i| {
            let c = &out[i];
            let dup = out[..i].iter().any(|d| {
                d.kind == c.kind && d.point.distance(c.point) <= SAME && d.entities == c.entities
            });
            !dup && !out.iter().enumerate().any(|(j, d)| j != i && covered(c, d))
        })
        .collect();
    let mut k = keep.iter();
    out.retain(|_| k.next().copied().unwrap_or(false));
    out.sort_by_key(|c| {
        (
            rank(c.kind),
            (c.point.distance(p) * 1e9).round() as i64,
            c.entities.first().copied(),
        )
    });
    out.truncate(MAX_CANDIDATES);
}
