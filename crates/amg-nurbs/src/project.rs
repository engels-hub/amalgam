//! Point projection (KH-13): curve and surface `closest_point` (P&T §6.1 Newton, eq. 6.3–6.8)
//! inside a branch-and-bound over the Bézier pieces, so the result is the global minimum.
//!
//! The curve or surface is split into Bézier pieces (A5.6 / A5.7). Each piece lies in the
//! convex hull of its (Euclidean) control points, which gives a lower bound on its distance:
//! for curves the distance to the chord between the end points minus the largest pole offset
//! from that chord, for surfaces the distance to a box aligned with the corner frame. Both
//! bounds converge quadratically as the pieces shrink. Pieces are processed in order of their
//! bound; Newton runs from every piece whose midpoint improves the best distance, and pieces
//! are halved (de Casteljau) until every remaining bound is within `EPS` of the best
//! distance. The returned distance is then within `EPS` (1e-10 mm, or 4 ulp of the model
//! scale for very large models) of the true minimum.
//!
//! That guarantee does not depend on Newton converging: a Newton run that stops short (its
//! iteration cap, a stalled line search) is repaired by subdivision, which continues until a
//! piece midpoint beats the stalled distance and restarts Newton next to the foot point.
//! It does depend on the pieces being allowed to shrink far enough, so the depth limits must
//! not bind before the parameters run out of `f64` resolution.

#![allow(
    clippy::float_cmp,
    reason = "parameters compared exactly for convergence"
)]

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use amg_core::{Point3, Tolerance, Vec3};
use smallvec::SmallVec;

use crate::knot_ops::{self, H, to_h};
use crate::{CtrlPt, CurvePoint, NurbsCurve, NurbsSurface, SurfacePoint, UV};

/// Absolute certification gap of the branch-and-bound, mm.
const EPS_ABS: f64 = 1e-10;
/// Node budget per query (keeps the work bounded; never reached in the KH-13 tests).
const MAX_NODES: usize = 20_000;
/// Subdivision depth limit per curve piece: more halvings than a parameter has bits.
const CURVE_MAX_DEPTH: u32 = 64;
/// Depth limit per surface patch, shared by both directions. With 64 a patch only reached a
/// width of 2⁻³² per direction (about 1e-7 mm on a 100 mm patch), too coarse to certify
/// [`EPS_ABS`] (KH-13 fix).
const SURF_MAX_DEPTH: u32 = 2 * CURVE_MAX_DEPTH;

/// Heap entry: smallest lower bound first, then insertion order (deterministic).
struct Entry {
    lb: f64,
    seq: u64,
    node: usize,
}

impl PartialEq for Entry {
    fn eq(&self, o: &Self) -> bool {
        self.cmp(o) == Ordering::Equal
    }
}
impl Eq for Entry {}
impl PartialOrd for Entry {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Entry {
    fn cmp(&self, o: &Self) -> Ordering {
        // Reversed: BinaryHeap is a max-heap.
        o.lb.total_cmp(&self.lb).then(o.seq.cmp(&self.seq))
    }
}

/// Distance from `q` to the segment `a b`.
#[inline]
fn seg_dist<P: CtrlPt>(q: P, a: P, b: P) -> f64 {
    let ab = b - a;
    let l2 = ab.dot(ab);
    let t = if l2 > 0.0 {
        ((q - a).dot(ab) / l2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (q - (a + ab * t)).length()
}

/// Euclidean point of a homogeneous pole.
#[inline]
fn euc<P: CtrlPt>(h: H<P>) -> P {
    h.p * (1.0 / h.w)
}

/// de Casteljau split of a Bézier line at 0.5.
fn split_half<P: CtrlPt>(pts: &[H<P>]) -> (Vec<H<P>>, Vec<H<P>>) {
    let n = pts.len();
    let mut work = pts.to_vec();
    let mut left = Vec::with_capacity(n);
    let mut right = vec![H::zero(); n];
    left.push(work[0]);
    right[n - 1] = work[n - 1];
    for r in 1..n {
        for i in 0..n - r {
            work[i] = (work[i] + work[i + 1]) * 0.5;
        }
        left.push(work[0]);
        right[n - 1 - r] = work[n - 1 - r];
    }
    (left, right)
}

/// Lower bound of the distance from `q` to the Bézier piece with homogeneous poles `pts`.
fn curve_lb<P: CtrlPt>(q: P, pts: &[H<P>]) -> f64 {
    let a = euc(pts[0]);
    let b = euc(pts[pts.len() - 1]);
    let delta = pts[1..pts.len() - 1]
        .iter()
        .fold(0.0f64, |m, &h| m.max(seg_dist(euc(h), a, b)));
    (seg_dist(q, a, b) - delta).max(0.0)
}

/// Certification gap for a query at `q` with model scale `scale`.
fn eps_for(scale: f64) -> f64 {
    EPS_ABS.max(4.0 * f64::EPSILON * scale)
}

impl<P: CtrlPt> NurbsCurve<P> {
    /// Safeguarded Newton (P&T eq. 6.3) for the foot point of `q` from `t0`, clamped to the
    /// domain; every accepted step decreases the distance. Returns `(t, distance)`.
    pub(crate) fn newton_foot(&self, q: P, t0: f64, angular: f64, max_iter: usize) -> (f64, f64) {
        let (a, b) = self.domain();
        let mut t = t0.clamp(a, b);
        let mut d = self.derivs(t, 2);
        let mut r = d[0] - q;
        let mut dist = r.length();
        for _ in 0..max_iter {
            let (c1, c2) = (d[1], d[2]);
            let f = c1.dot(r);
            let l1 = c1.length();
            if dist == 0.0 || l1 == 0.0 || f.abs() <= angular * l1 * dist {
                break;
            }
            let fp = c2.dot(r) + c1.dot(c1);
            let mut step = if fp > 0.0 { -f / fp } else { -f / c1.dot(c1) };
            let mut improved = false;
            for _ in 0..30 {
                let tn = (t + step).clamp(a, b);
                if tn == t {
                    break;
                }
                let dn = self.derivs(tn, 2);
                let rn = dn[0] - q;
                let distn = rn.length();
                if distn < dist {
                    t = tn;
                    d = dn;
                    r = rn;
                    dist = distn;
                    improved = true;
                    break;
                }
                step *= 0.5;
            }
            if !improved {
                break;
            }
        }
        (t, dist)
    }

    /// Closest point on the curve to `p` (global minimum; P&T §6.1 Newton inside a
    /// branch-and-bound over the Bézier segments, see the module docs).
    ///
    /// `tol.angular` is the orthogonality criterion of the Newton iteration (cosine between
    /// `C'` and `C − p`); the distance returned is within 1e-10 mm of the true minimum for
    /// models below about 1e5 mm.
    pub fn closest_point(&self, p: P, tol: &Tolerance) -> CurvePoint<P> {
        let angular = tol.angular.clamp(1e-15, 1e-6);
        let (a, b) = self.domain();
        let scale = self
            .poles()
            .iter()
            .fold(p.max_abs(), |m, q| m.max(q.max_abs()));
        let eps = eps_for(scale);
        let mut best_t = a;
        let mut best_d = (self.start() - p).length();
        let de = (self.end() - p).length();
        if de < best_d {
            best_t = b;
            best_d = de;
        }

        let pw = to_h(self.poles(), self.weights());
        let segs = knot_ops::decompose(self.degree(), self.knots().as_slice(), &pw);
        struct Node<P> {
            root: usize,
            range: (f64, f64),
            pts: Vec<H<P>>,
            depth: u32,
        }
        let rational = self.weights().is_some();
        let roots: Vec<NurbsCurve<P>> = segs
            .iter()
            .map(|s| {
                let (poles, w) = knot_ops::from_h(&s.pts, rational);
                let k = crate::KnotVector::bezier(self.degree()).reparametrized(s.range);
                NurbsCurve::new_unchecked(self.degree(), k, poles, w)
            })
            .collect();
        let mut nodes: Vec<Node<P>> = Vec::with_capacity(segs.len() * 4);
        let mut heap = BinaryHeap::new();
        let mut seq = 0u64;
        for (root, s) in segs.into_iter().enumerate() {
            let lb = curve_lb(p, &s.pts);
            nodes.push(Node {
                root,
                range: s.range,
                pts: s.pts,
                depth: 0,
            });
            heap.push(Entry {
                lb,
                seq,
                node: nodes.len() - 1,
            });
            seq += 1;
        }
        let mut popped = 0;
        while let Some(e) = heap.pop() {
            if e.lb >= best_d - eps || popped >= MAX_NODES {
                break;
            }
            popped += 1;
            let (t0, t1) = nodes[e.node].range;
            let tm = 0.5 * (t0 + t1);
            // Newton on the node's own Bézier segment (a kink at a knot is its domain end).
            let root = &roots[nodes[e.node].root];
            let dm = (root.eval(tm) - p).length();
            if dm < best_d {
                let (t, d) = root.newton_foot(p, tm, angular, 50);
                let (t, d) = if d < dm { (t, d) } else { (tm, dm) };
                if d < best_d {
                    best_d = d;
                    best_t = t;
                }
            }
            let depth = nodes[e.node].depth;
            if depth >= CURVE_MAX_DEPTH || !(t0 < tm && tm < t1) {
                continue;
            }
            let (l, r) = split_half(&nodes[e.node].pts);
            for (range, pts) in [((t0, tm), l), ((tm, t1), r)] {
                let lb = curve_lb(p, &pts);
                if lb < best_d - eps {
                    nodes.push(Node {
                        root: nodes[e.node].root,
                        range,
                        pts,
                        depth: depth + 1,
                    });
                    heap.push(Entry {
                        lb,
                        seq,
                        node: nodes.len() - 1,
                    });
                    seq += 1;
                }
            }
        }
        CurvePoint {
            t: best_t,
            point: self.eval(best_t),
            distance: best_d,
        }
    }
}

/// Distance from `p` to the triangle `a b c` (Ericson, *Real-Time Collision Detection*,
/// §5.1.5). The barycentric terms use the expanded forms (`vc = |ab|²·d2 − (ab·ac)·d1`, …)
/// instead of differences of products of the large `d_i`: for a far query and a tiny
/// triangle (deep branch-and-bound nodes) the book's form cancels catastrophically.
fn tri_dist(p: Point3, a: Point3, b: Point3, c: Point3) -> f64 {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let (aa, bc, cc) = (ab.dot(ab), ab.dot(ac), ac.dot(ac));
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return ap.length();
    }
    let (d3, d4) = (d1 - aa, d2 - bc);
    if d3 >= 0.0 && d4 <= d3 {
        return (ap - ab).length();
    }
    let vc = aa * d2 - bc * d1;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / aa;
        return (ap - ab * v).length();
    }
    let (d5, d6) = (d1 - bc, d2 - cc);
    if d6 >= 0.0 && d5 <= d6 {
        return (ap - ac).length();
    }
    let vb = cc * d1 - bc * d2;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / cc;
        return (ap - ac * w).length();
    }
    let va = d1 * (bc - cc) + d2 * (bc - aa) + (aa * cc - bc * bc);
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (ap - ab - (ac - ab) * w).length();
    }
    let den = va + vb + vc;
    if den == 0.0 || den.is_nan() {
        return seg_dist(p, a, b)
            .min(seg_dist(p, b, c))
            .min(seg_dist(p, a, c));
    }
    let v = vb / den;
    let w = vc / den;
    (ap - ab * v - ac * w).length()
}

/// `det[b − p, c − p, d − p]` in its affine form `det[b, c, d] − p·(b×c + c×d + d×b)`
/// (all relative to a local origin), accurate for a far `p` and a tiny `b c d`.
#[inline]
fn orient(p: Vec3, b: Vec3, c: Vec3, d: Vec3) -> f64 {
    b.cross(c).dot(d) - p.dot(b.cross(c) + c.cross(d) + d.cross(b))
}

/// Distance from `p` to the convex hull of four points (a tetrahedron, possibly flat):
/// zero inside, otherwise the nearest of its four faces.
fn hull4_dist(p: Point3, k: &[Point3; 4]) -> f64 {
    let o = k[0];
    let l = [Vec3::ZERO, k[1] - o, k[2] - o, k[3] - o];
    let q = p - o;
    let v = orient(l[0], l[1], l[2], l[3]);
    let scale = l[1].length() * l[2].length() * l[3].length();
    if v.abs() > 1e-12 * scale {
        // Inside iff p is on the side of the opposite vertex for every face.
        const FACES: [[usize; 4]; 4] = [[0, 1, 2, 3], [1, 0, 2, 3], [2, 0, 1, 3], [3, 0, 1, 2]];
        let inside = FACES.iter().all(|f| {
            let want = orient(l[f[0]], l[f[1]], l[f[2]], l[f[3]]);
            orient(q, l[f[1]], l[f[2]], l[f[3]]) * want >= 0.0
        });
        if inside {
            return 0.0;
        }
    }
    let f = [
        tri_dist(p, k[0], k[1], k[2]),
        tri_dist(p, k[0], k[2], k[3]),
        tri_dist(p, k[0], k[1], k[3]),
        tri_dist(p, k[1], k[2], k[3]),
    ];
    f.iter().fold(f64::INFINITY, |m, &x| m.min(x))
}

/// Bézier patch node: homogeneous poles row-major `(p+1) × (q+1)` (i along u).
struct Patch {
    /// Index of the Bézier patch this node was cut from.
    root: usize,
    u: (f64, f64),
    v: (f64, f64),
    nu: usize,
    nv: usize,
    pts: Vec<H<Point3>>,
    depth: u32,
}

impl Patch {
    #[inline]
    fn at(&self, i: usize, j: usize) -> H<Point3> {
        self.pts[i * self.nv + j]
    }

    /// Lower bound: max of the world AABB bound and the bounds of two boxes whose first axis
    /// follows the u (resp. v) iso direction and whose third axis is the corner normal. An
    /// edge-aligned box keeps the bound second order for minima on a patch edge (a knot
    /// crease or the domain boundary), which a diagonal or world box would not.
    ///
    /// The bounds are evaluated cheapest first and the evaluation stops as soon as one
    /// reaches `prune` (the node is then discarded anyway).
    fn lb(&self, q: Point3, prune: f64) -> f64 {
        let e: SmallVec<[Point3; 16]> = self.pts.iter().map(|&h| euc(h)).collect();
        let c00 = e[0];
        let c10 = e[(self.nu - 1) * self.nv];
        let c01 = e[self.nv - 1];
        let c11 = e[e.len() - 1];
        let box_lb = |x: Vec3, y: Vec3, z: Vec3| -> f64 {
            let mut lo = [f64::INFINITY; 3];
            let mut hi = [f64::NEG_INFINITY; 3];
            for pt in &e {
                for (k, ax) in [x, y, z].iter().enumerate() {
                    let s = pt.dot(*ax);
                    lo[k] = lo[k].min(s);
                    hi[k] = hi[k].max(s);
                }
            }
            let mut s2 = 0.0;
            for (k, ax) in [x, y, z].iter().enumerate() {
                let s = q.dot(*ax);
                let g = (lo[k] - s).max(s - hi[k]).max(0.0);
                s2 += g * g;
            }
            s2.sqrt()
        };
        let mut lb = box_lb(Vec3::X, Vec3::Y, Vec3::Z);
        if lb >= prune {
            return lb;
        }
        let eu = (c10 - c00) + (c11 - c01);
        let ev = (c01 - c00) + (c11 - c10);
        let n = eu.cross(ev);
        let (nl, lu, lv) = (n.length(), eu.length(), ev.length());
        if !(nl > 1e-12 * lu * lv && nl > 1e-300) {
            return lb;
        }
        let z = n / nl;
        for x in [eu / lu, ev / lv] {
            lb = lb.max(box_lb(x, z.cross(x), z));
            if lb >= prune {
                return lb;
            }
        }
        // Corner hull K = conv(c00, c10, c01, c11): every pole lies within `delta` of K, so
        // the patch does too (convex hull property), and dist(q, K) − delta is a bound that
        // stays second order at patch corners as well.
        let k = [c00, c10, c11, c01];
        let delta = e.iter().fold(0.0f64, |m, &pt| m.max(hull4_dist(pt, &k)));
        lb.max(hull4_dist(q, &k) - delta)
    }

    /// Geometric extent in u and v: the longest control polygon of each direction. The corner
    /// chords would not do: on a patch closed in one direction they are zero, so that
    /// direction would never be split and the search could not converge.
    fn extents(&self) -> (f64, f64) {
        let e: SmallVec<[Point3; 16]> = self.pts.iter().map(|&h| euc(h)).collect();
        let (nu, nv) = (self.nu, self.nv);
        let polygon = |first: usize, stride: usize, n: usize| -> f64 {
            (1..n)
                .map(|k| (e[first + k * stride] - e[first + (k - 1) * stride]).length())
                .sum()
        };
        let ext_u = (0..nv).fold(0.0f64, |m, j| m.max(polygon(j, nv, nu)));
        let ext_v = (0..nu).fold(0.0f64, |m, i| m.max(polygon(i * nv, 1, nv)));
        (ext_u, ext_v)
    }

    /// Halves along the direction with the larger geometric extent.
    fn split(&self) -> Option<(Patch, Patch)> {
        let (nu, nv) = (self.nu, self.nv);
        let (ext_u, ext_v) = self.extents();
        let um = 0.5 * (self.u.0 + self.u.1);
        let vm = 0.5 * (self.v.0 + self.v.1);
        let can_u = self.u.0 < um && um < self.u.1;
        let can_v = self.v.0 < vm && vm < self.v.1;
        let along_u = match (can_u, can_v) {
            (false, false) => return None,
            (true, false) => true,
            (false, true) => false,
            (true, true) => ext_u >= ext_v,
        };
        let mut a = vec![H::zero(); self.pts.len()];
        let mut b = vec![H::zero(); self.pts.len()];
        if along_u {
            let mut col = Vec::with_capacity(nu);
            for j in 0..nv {
                col.clear();
                col.extend((0..nu).map(|i| self.at(i, j)));
                let (l, r) = split_half(&col);
                for i in 0..nu {
                    a[i * nv + j] = l[i];
                    b[i * nv + j] = r[i];
                }
            }
            Some((
                Patch {
                    u: (self.u.0, um),
                    pts: a,
                    ..self.child()
                },
                Patch {
                    u: (um, self.u.1),
                    pts: b,
                    ..self.child()
                },
            ))
        } else {
            for i in 0..nu {
                let (l, r) = split_half(&self.pts[i * nv..(i + 1) * nv]);
                a[i * nv..(i + 1) * nv].copy_from_slice(&l);
                b[i * nv..(i + 1) * nv].copy_from_slice(&r);
            }
            Some((
                Patch {
                    v: (self.v.0, vm),
                    pts: a,
                    ..self.child()
                },
                Patch {
                    v: (vm, self.v.1),
                    pts: b,
                    ..self.child()
                },
            ))
        }
    }

    fn child(&self) -> Patch {
        Patch {
            root: self.root,
            u: self.u,
            v: self.v,
            nu: self.nu,
            nv: self.nv,
            pts: Vec::new(),
            depth: self.depth + 1,
        }
    }
}

impl NurbsSurface<Point3> {
    /// Safeguarded Newton (P&T eq. 6.6–6.8) for the foot point of `q` from `(u, v)`, clamped
    /// to the domain; every accepted step decreases the distance. Returns `(u, v, distance)`.
    pub(crate) fn newton_foot(
        &self,
        q: Point3,
        (u0, v0): (f64, f64),
        angular: f64,
        max_iter: usize,
    ) -> (f64, f64, f64) {
        let [(ua, ub), (va, vb)] = self.domain();
        let (mut u, mut v) = (u0.clamp(ua, ub), v0.clamp(va, vb));
        let mut d = self.derivs(u, v, 2);
        let mut r = d.d[0][0] - q;
        let mut dist = r.length();
        for _ in 0..max_iter {
            let (su, sv) = (d.d[1][0], d.d[0][1]);
            let f = su.dot(r);
            let g = sv.dot(r);
            let (lu, lv) = (su.length(), sv.length());
            if dist == 0.0 || (f.abs() <= angular * lu * dist && g.abs() <= angular * lv * dist) {
                break;
            }
            let j11 = su.dot(su) + r.dot(d.d[2][0]);
            let j12 = su.dot(sv) + r.dot(d.d[1][1]);
            let j22 = sv.dot(sv) + r.dot(d.d[0][2]);
            let det = j11 * j22 - j12 * j12;
            let (mut du, mut dv) = if j11 > 0.0 && det > 0.0 {
                (-(j22 * f - j12 * g) / det, -(j11 * g - j12 * f) / det)
            } else {
                // Not a local minimum model: steepest descent scaled per direction.
                let su2 = su.dot(su).max(1e-300);
                let sv2 = sv.dot(sv).max(1e-300);
                (-f / su2, -g / sv2)
            };
            // Projected step: drop the component that pushes against an active bound.
            if (u <= ua && du < 0.0) || (u >= ub && du > 0.0) {
                du = 0.0;
                if su.dot(su) > 0.0 && j22 > 0.0 {
                    dv = -g / j22;
                }
            }
            if (v <= va && dv < 0.0) || (v >= vb && dv > 0.0) {
                dv = 0.0;
                if j11 > 0.0 && du != 0.0 {
                    du = -f / j11;
                }
            }
            let mut improved = false;
            for _ in 0..30 {
                let un = (u + du).clamp(ua, ub);
                let vn = (v + dv).clamp(va, vb);
                if un == u && vn == v {
                    break;
                }
                let dn = self.derivs(un, vn, 2);
                let rn = dn.d[0][0] - q;
                let distn = rn.length();
                if distn < dist {
                    u = un;
                    v = vn;
                    d = dn;
                    r = rn;
                    dist = distn;
                    improved = true;
                    break;
                }
                du *= 0.5;
                dv *= 0.5;
            }
            if !improved {
                break;
            }
        }
        (u, v, dist)
    }

    /// Closest point on the surface to `p` (global minimum; P&T §6.1 Newton inside a
    /// branch-and-bound over the Bézier patches, see the module docs). `seed` (if any) starts
    /// a Newton iteration first, which only speeds up the search. `tol.angular` is the
    /// orthogonality criterion of the Newton iteration.
    pub fn closest_point(
        &self,
        p: Point3,
        seed: Option<(f64, f64)>,
        tol: &Tolerance,
    ) -> SurfacePoint {
        let angular = tol.angular.clamp(1e-15, 1e-6);
        let [(ua, ub), (va, vb)] = self.domain();
        let scale = self
            .poles()
            .as_slice()
            .iter()
            .fold(p.max_abs(), |m, q| m.max(q.max_abs()));
        let eps = eps_for(scale);
        let mut best = (ua, va, (self.eval(ua, va) - p).length());
        for (u, v) in [(ub, va), (ua, vb), (ub, vb)] {
            let d = (self.eval(u, v) - p).length();
            if d < best.2 {
                best = (u, v, d);
            }
        }
        if let Some(s) = seed {
            let r = self.newton_foot(p, s, angular, 50);
            if r.2 < best.2 {
                best = r;
            }
        }
        let mut nodes: Vec<Patch> = Vec::new();
        let mut roots: Vec<NurbsSurface<Point3>> = Vec::new();
        for strip in self.bezier_strips(UV::U) {
            let ru = strip.domain()[0];
            for patch in strip.bezier_strips(UV::V) {
                let rv = patch.domain()[1];
                let g = patch.poles();
                let (nu, nv) = (g.nu(), g.nv());
                let pts: Vec<H<Point3>> = match patch.weights() {
                    None => g.as_slice().iter().map(|&q| H { p: q, w: 1.0 }).collect(),
                    Some(w) => g
                        .as_slice()
                        .iter()
                        .zip(w.as_slice())
                        .map(|(&q, &w)| H { p: q * w, w })
                        .collect(),
                };
                nodes.push(Patch {
                    root: roots.len(),
                    u: ru,
                    v: rv,
                    nu,
                    nv,
                    pts,
                    depth: 0,
                });
                roots.push(patch);
            }
        }
        let mut heap = BinaryHeap::new();
        let mut seq = 0u64;
        for (i, n) in nodes.iter().enumerate() {
            heap.push(Entry {
                lb: n.lb(p, best.2 - eps),
                seq,
                node: i,
            });
            seq += 1;
        }
        let mut popped = 0;
        while let Some(e) = heap.pop() {
            if e.lb >= best.2 - eps || popped >= MAX_NODES {
                break;
            }
            popped += 1;
            let nd = &nodes[e.node];
            let um = 0.5 * (nd.u.0 + nd.u.1);
            let vm = 0.5 * (nd.v.0 + nd.v.1);
            // Newton on the node's own Bézier patch: a minimum on a knot crease is then a
            // bound of its domain (projected Newton), with one-sided derivatives.
            let root = &roots[nd.root];
            let dm = (root.eval(um, vm) - p).length();
            if dm < best.2 {
                let r = root.newton_foot(p, (um, vm), angular, 50);
                let r = if r.2 < dm { r } else { (um, vm, dm) };
                if r.2 < best.2 {
                    best = r;
                }
            }
            if nd.depth >= SURF_MAX_DEPTH {
                continue;
            }
            let Some((l, r)) = nd.split() else { continue };
            for child in [l, r] {
                let lb = child.lb(p, best.2 - eps);
                if lb < best.2 - eps {
                    nodes.push(child);
                    heap.push(Entry {
                        lb,
                        seq,
                        node: nodes.len() - 1,
                    });
                    seq += 1;
                }
            }
        }
        SurfacePoint {
            u: best.0,
            v: best.1,
            point: self.eval(best.0, best.1),
            distance: best.2,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analytic;
    use amg_core::Plane;

    #[test]
    fn circle_and_sphere_projection() {
        let c = analytic::circle_arc(&Plane::XY, 10.0, 0.0, std::f64::consts::TAU).unwrap();
        let tol = Tolerance::DEFAULT;
        let q = Point3::new(3.0, 4.0, 2.0);
        let r = c.closest_point(q, &tol);
        assert!((r.distance - (25.0f64 + 4.0).sqrt()).abs() < 1e-12, "{r:?}");
        assert!(
            (r.point - Point3::new(6.0, 8.0, 0.0)).length() < 1e-11,
            "{r:?}"
        );
        let s = analytic::sphere(Point3::ZERO, 5.0).unwrap();
        for q in [
            Point3::new(1.0, 2.0, 3.0),
            Point3::new(-7.0, 0.5, 9.0),
            Point3::Z * 6.0,
        ] {
            let r = s.closest_point(q, None, &tol);
            assert!(
                (r.distance - (q.length() - 5.0).abs()).abs() < 1e-10,
                "{q} {r:?}"
            );
        }
    }
}
