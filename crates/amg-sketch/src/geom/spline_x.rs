//! Intersections of B-splines (SK-08) with the other curves and with themselves.
//!
//! - **Spline vs line / circle / arc / ellipse.** The other curve is an implicit function `F`
//!   with a gradient. On every knot span `f(t) = F(C(t))` is split at the roots of
//!   `f'(t) = ∇F · C'(t)` (found as sign changes on a dense sample grid, then bisected) into
//!   monotone pieces; each piece holds at most one simple root, found by bisection. An extremum
//!   of `f` within the tolerance of the curve is a tangential contact.
//! - **Spline vs spline.** Bézier pieces of both curves are subdivided with their control-polygon
//!   boxes (convex hull property of positive-weight rational Béziers) until both are tiny; each
//!   surviving pair is polished with a damped Gauss–Newton (Levenberg–Marquardt) iteration on the
//!   exact curves. Contacts within the tolerance that are not crossings (grazing curves) converge
//!   to the minimum-distance pair and are accepted when that distance is within the tolerance.
//! - **Overlap.** If a run of consecutive samples of the spline lies within the tolerance of the
//!   other curve, the pair overlaps: the ends of every such run are reported as tangential hits
//!   and no transversal hits are searched (the same convention as for the other curve types).
//! - **Self-intersection** ([`self_intersections`]): the spline is cut into short pieces and the
//!   pieces are intersected with each other. A loop shorter than a twelfth of a knot span is not
//!   detected.

use amg_core::{Aabb2, Point2, Tolerance};
use smallvec::SmallVec;

use super::ellipse::{Implicit, param_on};
use super::query::Intersection;
use super::spline::{BSpline2, Hom, bisect};
use super::{Curve2, CurveHit};

fn push_unique(hits: &mut SmallVec<[CurveHit; 4]>, h: CurveHit, tol: f64) {
    if !(h.ta.is_finite() && h.tb.is_finite() && h.point.is_finite()) {
        return;
    }
    if hits.iter().all(|o| o.point.distance(h.point) > tol) {
        hits.push(h);
    }
}

/// Parameters of the roots of `F ∘ C` on the spline, with a flag for tangential contacts.
fn roots(s: &BSpline2, imp: &Implicit, lin: f64) -> SmallVec<[(f64, bool); 8]> {
    let n = s.samples();
    let f = |t: f64| imp.value(s.eval(t)).0;
    let g = |t: f64| {
        let (c, d, _) = s.derivs(t);
        imp.value(c).1.dot(d)
    };
    let mut out: SmallVec<[(f64, bool); 8]> = SmallVec::new();
    for (lo, hi) in s.spans() {
        let mut crit: SmallVec<[f64; 8]> = SmallVec::new();
        let (mut t_prev, mut g_prev) = (lo, g(lo));
        for i in 1..=n {
            let t = lo + (hi - lo) * i as f64 / n as f64;
            let gi = g(t);
            if (g_prev < 0.0 && gi > 0.0) || (g_prev > 0.0 && gi < 0.0) {
                crit.push(bisect(g, t_prev, t));
            } else if gi.abs() <= 0.0 && i < n {
                crit.push(t);
            }
            t_prev = t;
            g_prev = gi;
        }
        let mut cuts: SmallVec<[f64; 10]> = SmallVec::new();
        cuts.push(lo);
        cuts.extend(crit.iter().copied());
        cuts.push(hi);
        cuts.sort_by(f64::total_cmp);
        for w in cuts.windows(2) {
            let (flo, fhi) = (f(w[0]), f(w[1]));
            if flo.abs() <= 0.0 {
                out.push((w[0], false));
            } else if flo * fhi < 0.0 {
                out.push((bisect(f, w[0], w[1]), false));
            }
        }
        for &c in &crit {
            let (fv, gr) = imp.value(s.eval(c));
            let gl = gr.length();
            if gl > 0.0 && fv.abs() / gl <= lin {
                out.push((c, true));
            }
        }
    }
    let end = s.span();
    if f(end).abs() <= 0.0 {
        out.push((end, false));
    }
    out
}

/// Intersection of the spline `s` (the curve `scurve`) with `other`. With `swap`, `scurve` is
/// the second curve of the pair (hits get `ta` on `other`, `tb` on the spline).
pub(crate) fn intersect(
    s: &BSpline2,
    scurve: &Curve2,
    other: &Curve2,
    tol: &Tolerance,
    swap: bool,
) -> Intersection {
    let lin = tol.linear;
    let mut out = Intersection::default();
    let mk = |ts: f64, to: f64, point: Point2, tangential: bool| {
        if swap {
            CurveHit {
                ta: to,
                tb: ts,
                point,
                tangential,
            }
        } else {
            CurveHit {
                ta: ts,
                tb: to,
                point,
                tangential,
            }
        }
    };
    let span = s.span();

    // Overlap: runs of samples of the spline lying on the other curve.
    let on = |t: f64| {
        let p = s.eval(t);
        other.closest(p).1.distance(p) <= lin
    };
    let m = 40 * s.spans().len();
    let at = |i: usize| span * i as f64 / m as f64;
    let flags: Vec<bool> = (0..=m).map(|i| on(at(i))).collect();
    let mut run_ends: SmallVec<[f64; 4]> = SmallVec::new();
    for i in 0..m {
        if flags[i] && flags[i + 1] && on(0.5 * (at(i) + at(i + 1))) {
            out.overlap = true;
        }
    }
    if out.overlap {
        // Boundaries of the maximal runs, refined by bisection on the predicate.
        for i in 0..m {
            if flags[i] != flags[i + 1] {
                let (mut lo, mut hi) = (at(i), at(i + 1));
                let lo_on = flags[i];
                for _ in 0..60 {
                    let mid = 0.5 * (lo + hi);
                    if on(mid) == lo_on {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                run_ends.push(if lo_on { lo } else { hi });
            }
        }
        if flags[0] {
            run_ends.push(0.0);
        }
        if flags[m] {
            run_ends.push(span);
        }
        for t in run_ends {
            let p = s.eval(t);
            let (to, _) = other.closest(p);
            push_unique(&mut out.hits, mk(t, to, p, true), lin);
        }
        return out;
    }

    if let Some(imp) = Implicit::of(other) {
        for (t, tangential) in roots(s, &imp, lin) {
            let p = s.eval(t);
            if let Some(to) = param_on(other, p, lin) {
                push_unique(&mut out.hits, mk(t, to, p, tangential), lin);
            }
        }
    } else if let Curve2::BSpline(o) = other {
        for (ta, tb, p) in spline_spline(s, o, lin) {
            push_unique(&mut out.hits, mk(ta, tb, p, false), lin);
        }
    }

    // End points touching the other curve (T-junctions and tangential approaches).
    let mut ends: SmallVec<[CurveHit; 4]> = SmallVec::new();
    for ts in [0.0, span] {
        let p = s.eval(ts);
        let (to, q) = other.closest(p);
        if q.distance(p) <= lin {
            push_unique(&mut ends, mk(ts, to, p, false), lin);
        }
    }
    if !other.is_closed() {
        let t_end = other.param_range().1;
        for to in [0.0, t_end] {
            let p = other.eval(to);
            let (ts, q) = scurve.closest(p);
            if q.distance(p) <= lin {
                push_unique(&mut ends, mk(ts, to, p, false), lin);
            }
        }
    }
    for h in ends {
        push_unique(&mut out.hits, h, lin);
    }
    out
}

// ---- Spline vs spline ------------------------------------------------------------------------

/// A Bézier piece of a spline: parameter range and homogeneous poles.
struct Bz {
    lo: f64,
    hi: f64,
    pw: Vec<Hom>,
}

impl Bz {
    fn bbox(&self) -> Aabb2 {
        Aabb2::from_points(
            self.pw
                .iter()
                .map(|h| Point2::new(h[0] / h[2], h[1] / h[2])),
        )
    }

    fn split(&self) -> (Bz, Bz) {
        let (l, r) = BSpline2::bezier_split(&self.pw, 0.5);
        let mid = 0.5 * (self.lo + self.hi);
        (
            Bz {
                lo: self.lo,
                hi: mid,
                pw: l,
            },
            Bz {
                lo: mid,
                hi: self.hi,
                pw: r,
            },
        )
    }
}

fn pieces(s: &BSpline2) -> Vec<Bz> {
    s.bezier_pieces()
        .into_iter()
        .map(|(lo, hi, pw)| Bz { lo, hi, pw })
        .collect()
}

/// Levenberg–Marquardt polish of `A(ta) − B(tb)`. Returns `(ta, tb, distance)`.
fn refine(a: &BSpline2, b: &BSpline2, mut ta: f64, mut tb: f64) -> (f64, f64, f64) {
    let (la, lb) = (a.span(), b.span());
    let mut mu = 1e-6;
    let mut g = a.eval(ta) - b.eval(tb);
    for _ in 0..80 {
        let dist = g.length();
        if dist < 1e-14 {
            break;
        }
        let (ja, jb) = (a.derivative(ta), -b.derivative(tb));
        let h00 = ja.dot(ja) + mu;
        let h01 = ja.dot(jb);
        let h11 = jb.dot(jb) + mu;
        let (g0, g1) = (ja.dot(g), jb.dot(g));
        let det = h00 * h11 - h01 * h01;
        if det.abs() < 1e-300 {
            break;
        }
        let da = -(h11 * g0 - h01 * g1) / det;
        let db = -(-h01 * g0 + h00 * g1) / det;
        let (na, nb) = ((ta + da).clamp(0.0, la), (tb + db).clamp(0.0, lb));
        let ng = a.eval(na) - b.eval(nb);
        if ng.length_squared() < g.length_squared() {
            let moved = (na - ta).abs().max((nb - tb).abs());
            ta = na;
            tb = nb;
            g = ng;
            mu = (mu * 0.2).max(1e-12);
            if moved < 1e-15 {
                break;
            }
        } else {
            mu *= 10.0;
            if mu > 1e12 {
                break;
            }
        }
    }
    (ta, tb, g.length())
}

/// Intersections `(ta, tb, point)` of two splines within the tolerance `lin`.
pub(crate) fn spline_spline(a: &BSpline2, b: &BSpline2, lin: f64) -> Vec<(f64, f64, Point2)> {
    let diag = a.bbox().diagonal().max(b.bbox().diagonal());
    let leaf = (1e-3 * diag).clamp(1e-4, 1e-2);
    let mut stack: Vec<(Bz, Bz)> = Vec::new();
    let pb = pieces(b);
    for x in pieces(a) {
        for y in &pb {
            stack.push((
                Bz {
                    lo: x.lo,
                    hi: x.hi,
                    pw: x.pw.clone(),
                },
                Bz {
                    lo: y.lo,
                    hi: y.hi,
                    pw: y.pw.clone(),
                },
            ));
        }
    }
    let mut out: Vec<(f64, f64, Point2)> = Vec::new();
    let mut budget = 40_000usize;
    while let Some((x, y)) = stack.pop() {
        if budget == 0 {
            break;
        }
        budget -= 1;
        let (bx, by) = (x.bbox(), y.bbox());
        if bx.expanded(lin).min.x > by.max.x
            || by.expanded(lin).min.x > bx.max.x
            || bx.expanded(lin).min.y > by.max.y
            || by.expanded(lin).min.y > bx.max.y
        {
            continue;
        }
        let (dx, dy) = (bx.diagonal(), by.diagonal());
        if dx <= leaf && dy <= leaf {
            let (ta, tb, d) = refine(a, b, 0.5 * (x.lo + x.hi), 0.5 * (y.lo + y.hi));
            if d <= lin {
                let p = a.eval(ta);
                if out.iter().all(|o| o.2.distance(p) > lin) {
                    out.push((ta, tb, p));
                }
            }
        } else if dx >= dy {
            let (l, r) = x.split();
            stack.push((
                r,
                Bz {
                    lo: y.lo,
                    hi: y.hi,
                    pw: y.pw.clone(),
                },
            ));
            stack.push((l, y));
        } else {
            let (l, r) = y.split();
            stack.push((
                Bz {
                    lo: x.lo,
                    hi: x.hi,
                    pw: x.pw.clone(),
                },
                r,
            ));
            stack.push((x, l));
        }
    }
    out.sort_by(|p, q| p.0.total_cmp(&q.0));
    out
}

/// Self-intersections `(t1, t2, point)`, `t1 < t2`, of a spline.
pub(crate) fn self_intersections(s: &BSpline2, lin: f64) -> Vec<(f64, f64, Point2)> {
    const PER_SPAN: usize = 12;
    let mut cuts: Vec<f64> = Vec::new();
    for (lo, hi) in s.spans() {
        for k in 0..PER_SPAN {
            cuts.push(lo + (hi - lo) * k as f64 / PER_SPAN as f64);
        }
    }
    cuts.push(s.span());
    let parts: Vec<(f64, BSpline2)> = cuts
        .windows(2)
        .map(|w| (w[0], s.trimmed(w[0], w[1])))
        .collect();
    let mut out = Vec::new();
    for i in 0..parts.len() {
        for j in i + 1..parts.len() {
            let (oi, pi) = &parts[i];
            let (oj, pj) = &parts[j];
            let shared = if j == i + 1 {
                Some(pi.eval(pi.span()))
            } else {
                None
            };
            for (ta, tb, p) in spline_spline(pi, pj, lin) {
                if shared.is_some_and(|q| q.distance(p) <= 10.0 * lin.max(1e-9)) {
                    continue;
                }
                out.push((oi + ta, oj + tb, p));
            }
        }
    }
    // The closing point of a closed curve (end on start) is not a self-intersection.
    let span = s.span();
    let eps = 1e-7 * span.max(1.0);
    out.retain(|&(t1, t2, _)| !(t1 <= eps && t2 >= span - eps));
    out.sort_by(|p, q| p.0.total_cmp(&q.0));
    out.dedup_by(|p, q| p.2.distance(q.2) <= lin);
    out
}
