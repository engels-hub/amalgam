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
//! - **Self-intersection** ([`self_intersections`]): one Bézier subdivision of the spline
//!   against itself; pieces and consecutive pairs whose tangents fit in a half-plane are
//!   monotone and dropped at once, so the shared ends of consecutive pieces are joins and are
//!   not subdivided. Loops below the subdivision floor are not detected.

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

    // Overlap: runs of samples of the spline lying on the other curve. A sample outside every
    // box of the other curve (its Bézier pieces for a spline) is not on it, so the exact
    // closest-point query only runs for the few samples inside (SK-23: unfiltered, this
    // sampling is quadratic in the knot spans and dominated spline–spline pairs).
    let near: Vec<Aabb2> = match other {
        Curve2::BSpline(o) => pieces(o).iter().map(|b| b.bbox().expanded(lin)).collect(),
        _ => vec![other.bbox().expanded(lin)],
    };
    let inside = |p: Point2| {
        near.iter()
            .any(|b| p.x >= b.min.x && p.x <= b.max.x && p.y >= b.min.y && p.y <= b.max.y)
    };
    let on = |t: f64| {
        let p = s.eval(t);
        inside(p) && other.closest(p).1.distance(p) <= lin
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
#[derive(Clone)]
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
fn refine(a: &BSpline2, b: &BSpline2, ta: f64, tb: f64) -> (f64, f64, f64) {
    refine_in(a, b, ta, tb, (0.0, a.span()), (0.0, b.span()))
}

/// [`refine`] with `ta` kept in `ra` and `tb` in `rb`.
fn refine_in(
    a: &BSpline2,
    b: &BSpline2,
    mut ta: f64,
    mut tb: f64,
    ra: (f64, f64),
    rb: (f64, f64),
) -> (f64, f64, f64) {
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
        let (na, nb) = ((ta + da).clamp(ra.0, ra.1), (tb + db).clamp(rb.0, rb.1));
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
    // Only piece pairs whose boxes meet start the subdivision (the other pairs would be
    // pruned at once, but pushing them clones their poles).
    let pb = pieces(b);
    let bb: Vec<Aabb2> = pb.iter().map(|y| y.bbox().expanded(lin)).collect();
    for x in pieces(a) {
        let bx = x.bbox();
        for (y, by) in pb.iter().zip(&bb) {
            if meet(&bx, by) {
                stack.push((x.clone(), y.clone()));
            }
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

/// True if the boxes overlap (closed boxes, no tolerance).
fn meet(a: &Aabb2, b: &Aabb2) -> bool {
    a.min.x <= b.max.x && b.min.x <= a.max.x && a.min.y <= b.max.y && b.min.y <= a.max.y
}

/// True if every tangent of the Bézier pieces `pws` points into one open half-plane, so the
/// chain of pieces is strictly monotone along some direction and cannot meet itself (except
/// where consecutive pieces share an end point). The derivative of a positive-weight rational
/// Bézier is a positive combination of the pole differences `P_j − P_i`, `j > i` (Floater), so
/// those differences bound the tangent cone. Pole differences shorter than `1e-12` of the
/// piece size are ignored (repeated poles add nothing to the derivative).
fn monotone(pws: &[&[Hom]]) -> bool {
    const MARGIN: f64 = 1e-6;
    let mut angles: SmallVec<[f64; 32]> = SmallVec::new();
    for pw in pws {
        let pts: SmallVec<[Point2; 12]> = pw
            .iter()
            .map(|h| Point2::new(h[0] / h[2], h[1] / h[2]))
            .collect();
        let size = Aabb2::from_points(pts.iter().copied()).diagonal();
        for i in 0..pts.len() {
            for j in i + 1..pts.len() {
                let d = pts[j] - pts[i];
                if d.length() > 1e-12 * size {
                    angles.push(d.y.atan2(d.x));
                }
            }
        }
    }
    if !angles.iter().all(|a| a.is_finite()) {
        return false;
    }
    if angles.len() < 2 {
        return true;
    }
    angles.sort_by(f64::total_cmp);
    // The vectors fit in an arc narrower than π iff some angular gap between neighbours
    // (cyclically) is wider than π.
    let wrap = angles[0] + std::f64::consts::TAU - angles[angles.len() - 1];
    let widest = angles.windows(2).map(|w| w[1] - w[0]).fold(wrap, f64::max);
    widest > std::f64::consts::PI + MARGIN
}

/// Which ends of a pair of parameter ranges `(x, y)` are the same curve point: consecutive
/// pieces, or the seam of a closed curve. A shared point is a join, not a crossing.
#[derive(Copy, Clone, Default)]
struct Shared {
    /// The end of `x` is the start of `y`.
    end_start: bool,
    /// The start of `x` is the end of `y`.
    start_end: bool,
}

/// One unit of work of [`self_intersections`].
enum SelfJob {
    /// The piece may cross itself; with the flag, its two ends are the same point (a closed
    /// curve of one piece).
    One(Bz, bool),
    /// Two disjoint parameter ranges and their shared ends.
    Two(Bz, Bz, Shared),
}

/// The Cartesian point of a homogeneous pole.
fn cart(h: Hom) -> Point2 {
    Point2::new(h[0] / h[2], h[1] / h[2])
}

/// Self-intersections `(t1, t2, point)`, `t1 < t2`, of a spline.
///
/// One Bézier subdivision of the curve against itself (SK-23; the previous version
/// intersected every pair of twelfth-of-a-span parts, quadratic in the span count, with the
/// shared end of every consecutive pair subdivided to the floor). A piece, or a pair of
/// pieces sharing one end, whose tangents all lie in an open half-plane is monotone along
/// some direction and is dropped at once; that is the common case for consecutive pieces of a
/// smooth curve, so their shared end is never subdivided. Other pairs are pruned by
/// control-polygon boxes and subdivided until both are tiny, then polished on the exact curve
/// within the pair's parameter ranges; hits at a shared end are dropped. Loops smaller than
/// the subdivision floor (`1e-3` of the curve's box diagonal, clamped to `1e-4..=1e-2` mm)
/// are not detected.
pub(crate) fn self_intersections(s: &BSpline2, lin: f64) -> Vec<(f64, f64, Point2)> {
    let span = s.span();
    let leaf = (1e-3 * s.bbox().diagonal()).clamp(1e-4, 1e-2);
    let ps = pieces(s);
    let n = ps.len();
    let closed = s.eval(0.0).distance(s.eval(span)) <= lin;
    let boxes: Vec<Aabb2> = ps.iter().map(|b| b.bbox().expanded(lin)).collect();
    let mut stack: Vec<SelfJob> = Vec::new();
    for i in 0..n {
        for j in i + 1..n {
            if meet(&boxes[i], &boxes[j]) {
                let shared = Shared {
                    end_start: j == i + 1,
                    start_end: closed && i == 0 && j == n - 1,
                };
                stack.push(SelfJob::Two(ps[i].clone(), ps[j].clone(), shared));
            }
        }
    }
    stack.extend(ps.into_iter().map(|x| SelfJob::One(x, closed && n == 1)));
    let shared_tol = 10.0 * lin.max(1e-9);
    let mut out: Vec<(f64, f64, Point2)> = Vec::new();
    let mut budget = 40_000 + 2_000 * n;
    while let Some(job) = stack.pop() {
        if budget == 0 {
            break;
        }
        budget -= 1;
        match job {
            SelfJob::One(x, closes) => {
                if (!closes && monotone(&[&x.pw])) || x.bbox().diagonal() <= leaf {
                    continue;
                }
                let (l, r) = x.split();
                let shared = Shared {
                    end_start: true,
                    start_end: closes,
                };
                stack.push(SelfJob::Two(l.clone(), r.clone(), shared));
                stack.push(SelfJob::One(l, false));
                stack.push(SelfJob::One(r, false));
            }
            SelfJob::Two(x, y, sh) => {
                let (bx, by) = (x.bbox(), y.bbox());
                if !meet(&bx.expanded(lin), &by) {
                    continue;
                }
                // Sharing exactly one end and monotone together: they meet only there.
                if sh.end_start != sh.start_end && monotone(&[&x.pw, &y.pw]) {
                    continue;
                }
                let (dx, dy) = (bx.diagonal(), by.diagonal());
                if dx <= leaf && dy <= leaf {
                    let (ta, tb, d) = refine_in(
                        s,
                        s,
                        0.5 * (x.lo + x.hi),
                        0.5 * (y.lo + y.hi),
                        (x.lo, x.hi),
                        (y.lo, y.hi),
                    );
                    if d > lin {
                        continue;
                    }
                    let p = s.eval(ta);
                    let at_end =
                        sh.end_start && cart(x.pw[x.pw.len() - 1]).distance(p) <= shared_tol;
                    let at_start = sh.start_end && cart(x.pw[0]).distance(p) <= shared_tol;
                    if !(at_end || at_start) {
                        out.push((ta.min(tb), ta.max(tb), p));
                    }
                } else if dx >= dy {
                    let (l, r) = x.split();
                    let to_l = Shared {
                        end_start: false,
                        start_end: sh.start_end,
                    };
                    let to_r = Shared {
                        end_start: sh.end_start,
                        start_end: false,
                    };
                    stack.push(SelfJob::Two(l, y.clone(), to_l));
                    stack.push(SelfJob::Two(r, y, to_r));
                } else {
                    let (l, r) = y.split();
                    let to_l = Shared {
                        end_start: sh.end_start,
                        start_end: false,
                    };
                    let to_r = Shared {
                        end_start: false,
                        start_end: sh.start_end,
                    };
                    stack.push(SelfJob::Two(x.clone(), r, to_r));
                    stack.push(SelfJob::Two(x, l, to_l));
                }
            }
        }
    }
    // The closing point of a closed curve (end on start) is not a self-intersection.
    let eps = 1e-7 * span.max(1.0);
    out.retain(|&(t1, t2, _)| !(t1 <= eps && t2 >= span - eps));
    out.sort_by(|p, q| p.0.total_cmp(&q.0));
    let mut uniq: Vec<(f64, f64, Point2)> = Vec::new();
    for h in out {
        if uniq.iter().all(|o| o.2.distance(h.2) > lin) {
            uniq.push(h);
        }
    }
    uniq
}
