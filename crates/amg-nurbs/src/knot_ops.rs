//! Knot-operation kernels on homogeneous control points (P&T ch. 5), shared by curves and
//! surfaces (KH-12).
//!
//! Every kernel works on one "line" of homogeneous points `Pw = (w·P, w)` and its knot vector:
//! a curve is one line, a surface is a set of lines along one direction (P&T A5.3 / A5.5 /
//! A5.7 / A5.10 are the curve kernels applied per row or column; the knot arithmetic is the
//! same for every line). Inputs are valid by construction (the public wrappers in
//! `curve_ops` / `surface_ops` check them), so the kernels index freely.

#![allow(
    clippy::float_cmp,
    reason = "knots are compared exactly: equal values form one multiplicity run"
)]
#![allow(
    clippy::needless_range_loop,
    clippy::explicit_counter_loop,
    reason = "index arithmetic kept in the published P&T form"
)]

use std::ops::{Add, Mul, Sub};

use crate::CtrlPt;
use crate::knots::find_span;

/// Homogeneous control point `(w·P, w)`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct H<P> {
    pub(crate) p: P,
    pub(crate) w: f64,
}

impl<P: CtrlPt> H<P> {
    #[inline]
    pub(crate) fn zero() -> Self {
        H {
            p: P::zero(),
            w: 0.0,
        }
    }

    /// Euclidean distance in homogeneous (DIM + 1)-space (P&T `Distance4D`).
    #[inline]
    pub(crate) fn dist(self, o: Self) -> f64 {
        let d = self - o;
        (d.p.dot(d.p) + d.w * d.w).sqrt()
    }
}

impl<P: CtrlPt> Add for H<P> {
    type Output = Self;
    #[inline]
    fn add(self, o: Self) -> Self {
        H {
            p: self.p + o.p,
            w: self.w + o.w,
        }
    }
}

impl<P: CtrlPt> Sub for H<P> {
    type Output = Self;
    #[inline]
    fn sub(self, o: Self) -> Self {
        H {
            p: self.p - o.p,
            w: self.w - o.w,
        }
    }
}

impl<P: CtrlPt> Mul<f64> for H<P> {
    type Output = Self;
    #[inline]
    fn mul(self, s: f64) -> Self {
        H {
            p: self.p * s,
            w: self.w * s,
        }
    }
}

/// `a·alpha + b·(1 − alpha)` (the corner-cutting step of every knot algorithm).
#[inline]
fn mix<P: CtrlPt>(a: H<P>, b: H<P>, alpha: f64) -> H<P> {
    a * alpha + b * (1.0 - alpha)
}

/// Poles (+ optional weights) to homogeneous points. Non-rational poles get `w = 1` and are
/// not multiplied, so the non-rational path is bit-identical to the plain algorithms.
pub(crate) fn to_h<P: CtrlPt>(poles: &[P], weights: Option<&[f64]>) -> Vec<H<P>> {
    match weights {
        None => poles.iter().map(|&p| H { p, w: 1.0 }).collect(),
        Some(w) => poles
            .iter()
            .zip(w)
            .map(|(&p, &w)| H { p: p * w, w })
            .collect(),
    }
}

/// Homogeneous points back to poles (+ weights if `rational`).
pub(crate) fn from_h<P: CtrlPt>(hs: &[H<P>], rational: bool) -> (Vec<P>, Option<Vec<f64>>) {
    if rational {
        let poles = hs.iter().map(|h| h.p * (1.0 / h.w)).collect();
        (poles, Some(hs.iter().map(|h| h.w).collect()))
    } else {
        (hs.iter().map(|h| h.p).collect(), None)
    }
}

/// Binomial coefficient as f64 (exact for the degrees used here: every partial product is an
/// integer `C(n−k+i, i)` below 2^53).
pub(crate) fn binom(n: usize, k: usize) -> f64 {
    if k > n {
        return 0.0;
    }
    let k = k.min(n - k);
    let mut r = 1.0;
    for i in 1..=k {
        r = r * (n - k + i) as f64 / i as f64;
    }
    r
}

/// A5.1 CurveKnotIns: insert `t` `r` times; `k` is the span (`U[k] <= t < U[k+1]`), `s` the
/// current multiplicity of `t`. Requires `1 <= r <= p − s`.
pub(crate) fn insert<P: CtrlPt>(
    p: usize,
    u: &[f64],
    pw: &[H<P>],
    t: f64,
    k: usize,
    s: usize,
    r: usize,
) -> (Vec<f64>, Vec<H<P>>) {
    let np = pw.len() - 1;
    let mp = np + p + 1;
    let mut uq = Vec::with_capacity(mp + r + 1);
    uq.extend_from_slice(&u[..=k]);
    uq.extend(std::iter::repeat_n(t, r));
    uq.extend_from_slice(&u[k + 1..=mp]);

    let mut q = vec![H::zero(); np + r + 1];
    q[..=k - p].copy_from_slice(&pw[..=k - p]);
    q[k - s + r..=np + r].copy_from_slice(&pw[k - s..=np]);
    let mut rw: Vec<H<P>> = pw[k - p..=k - s].to_vec();
    let mut l = k - p;
    for j in 1..=r {
        l = k - p + j;
        for i in 0..=p - j - s {
            let alpha = (t - u[l + i]) / (u[i + k + 1] - u[l + i]);
            rw[i] = mix(rw[i + 1], rw[i], alpha);
        }
        q[l] = rw[0];
        q[k + r - j - s] = rw[p - j - s];
    }
    if l + 1 < k - s {
        q[l + 1..k - s].copy_from_slice(&rw[1..k - s - l]);
    }
    (uq, q)
}

/// A5.4 RefineKnotVectCurve: insert the sorted, non-empty knots `x` (all inside the domain,
/// resulting multiplicities `<= p`).
pub(crate) fn refine<P: CtrlPt>(
    p: usize,
    u: &[f64],
    pw: &[H<P>],
    x: &[f64],
) -> (Vec<f64>, Vec<H<P>>) {
    let n = pw.len() - 1;
    let m = n + p + 1;
    let r = x.len() - 1;
    let a = find_span(u, p, n + 1, x[0]);
    let b = find_span(u, p, n + 1, x[r]) + 1;
    let mut q = vec![H::zero(); n + r + 2];
    let mut ub = vec![0.0; m + r + 2];
    q[..=a - p].copy_from_slice(&pw[..=a - p]);
    q[b - 1 + r + 1..=n + r + 1].copy_from_slice(&pw[b - 1..=n]);
    ub[..=a].copy_from_slice(&u[..=a]);
    ub[b + p + r + 1..=m + r + 1].copy_from_slice(&u[b + p..=m]);
    let mut i = b + p - 1;
    let mut k = b + p + r;
    for j in (0..=r).rev() {
        while x[j] <= u[i] && i > a {
            q[k - p - 1] = pw[i - p - 1];
            ub[k] = u[i];
            k -= 1;
            i -= 1;
        }
        q[k - p - 1] = q[k - p];
        for l in 1..=p {
            let ind = k - p + l;
            let alfa = ub[k + l] - x[j];
            if alfa == 0.0 {
                q[ind - 1] = q[ind];
            } else {
                let alfa = alfa / (ub[k + l] - u[i - p + l]);
                q[ind - 1] = mix(q[ind - 1], q[ind], alfa);
            }
        }
        ub[k] = x[j];
        k -= 1;
    }
    (ub, q)
}

/// One Bézier segment of a decomposition: parameter range and `p + 1` homogeneous points.
pub(crate) struct BezierSeg<P> {
    pub(crate) range: (f64, f64),
    pub(crate) pts: Vec<H<P>>,
}

/// A5.6 DecomposeCurve: the Bézier segments of every non-empty span, in order.
pub(crate) fn decompose<P: CtrlPt>(p: usize, u: &[f64], pw: &[H<P>]) -> Vec<BezierSeg<P>> {
    let n = pw.len() - 1;
    let m = n + p + 1;
    let mut out = Vec::new();
    let mut a = p;
    let mut b = p + 1;
    let mut cur: Vec<H<P>> = pw[..=p].to_vec();
    let mut alphas = vec![0.0; p.max(1)];
    while b < m {
        let i = b;
        while b < m && u[b + 1] == u[b] {
            b += 1;
        }
        let mult = b - i + 1;
        let mut next = vec![H::zero(); p + 1];
        if mult < p {
            let numer = u[b] - u[a];
            for j in (mult + 1..=p).rev() {
                alphas[j - mult - 1] = numer / (u[a + j] - u[a]);
            }
            let r = p - mult;
            for j in 1..=r {
                let save = r - j;
                let s = mult + j;
                for k in (s..=p).rev() {
                    cur[k] = mix(cur[k], cur[k - 1], alphas[k - s]);
                }
                if b < m {
                    next[save] = cur[p];
                }
            }
        }
        let range = (u[a], u[b]);
        if b < m {
            next[p - mult..=p].copy_from_slice(&pw[b - mult..=b]);
            out.push(BezierSeg { range, pts: cur });
            cur = next;
            a = b;
            b += 1;
        } else {
            out.push(BezierSeg { range, pts: cur });
            break;
        }
    }
    out
}

/// A5.8 RemoveCurveKnot, one copy per call: removes one copy of the knot `u[r]` (`r` = last
/// index of its run, multiplicity `s >= 1`) if `accept(d)` agrees, where `d` is the
/// homogeneous distance between the old pole and its reconstruction at the one redundant
/// equation. Re-inserting the knot reproduces every other old pole exactly, so the curve moves
/// by at most `d` (non-rational; eq. 5.30 scaling for rational curves).
///
/// Deviation from the book: P&T place the redundant equation in the middle of the affected
/// poles and sweep in from both ends. Here it goes where the two sweeps amplify rounding
/// least (left sweep ≈ Π (2 − α)/α, right sweep ≈ Π (1 + α)/(1 − α)), which keeps exact
/// removals exact next to very short spans. Repeated calls remove several copies (equivalent
/// to A5.8's levels).
pub(crate) fn remove_one<P: CtrlPt>(
    p: usize,
    u: &mut Vec<f64>,
    pw: &mut Vec<H<P>>,
    r: usize,
    s: usize,
    accept: impl FnOnce(f64) -> bool,
) -> bool {
    let first = r - p;
    let last = r - s;
    let uu = u[r];
    let ne = last - first + 1;
    let al: Vec<f64> = (first..=last)
        .map(|i| (uu - u[i]) / (u[i + p + 1] - u[i]))
        .collect();
    // log amplification of the left sweep up to equation c, right sweep down to c.
    let mut el = vec![0.0; ne];
    for k in 1..ne {
        el[k] = el[k - 1] + ((2.0 - al[k - 1]) / al[k - 1]).ln();
    }
    let mut er = vec![0.0; ne];
    for k in (0..ne - 1).rev() {
        er[k] = er[k + 1] + ((1.0 + al[k + 1]) / (1.0 - al[k + 1])).ln();
    }
    let mut c = 0;
    for k in 1..ne {
        if el[k].max(er[k]) < el[c].max(er[c]) {
            c = k;
        }
    }
    // q[k] = new pole Q_{first−1+k}, k = 0..=ne.
    let mut q = vec![H::<P>::zero(); ne + 1];
    q[0] = pw[first - 1];
    q[ne] = pw[last + 1];
    for k in 0..c {
        let a = al[k];
        q[k + 1] = (pw[first + k] - q[k] * (1.0 - a)) * (1.0 / a);
    }
    for k in (c + 1..ne).rev() {
        let a = al[k];
        q[k] = (pw[first + k] - q[k + 1] * a) * (1.0 / (1.0 - a));
    }
    let d = pw[first + c].dist(mix(q[c + 1], q[c], al[c]));
    if !(d.is_finite() && accept(d)) {
        return false;
    }
    let mut np = Vec::with_capacity(pw.len() - 1);
    np.extend_from_slice(&pw[..first]);
    np.extend_from_slice(&q[1..ne]);
    np.extend_from_slice(&pw[last + 1..]);
    *pw = np;
    u.remove(r);
    true
}

/// Number of distinct interior knots (spans − 1) of a clamped vector of degree `p`.
fn interior_breaks(p: usize, u: &[f64]) -> usize {
    let m = u.len() - 1;
    let mut c = 0;
    let mut i = p + 1;
    while i < m - p {
        c += 1;
        let v = u[i];
        while i < m - p && u[i] == v {
            i += 1;
        }
    }
    c
}

/// A5.9 DegreeElevateCurve (Bézier-segment method): elevate by `t >= 1`.
#[allow(
    clippy::too_many_lines,
    reason = "one published algorithm, kept in P&T order"
)]
pub(crate) fn elevate<P: CtrlPt>(
    p: usize,
    u: &[f64],
    pw: &[H<P>],
    t: usize,
) -> (Vec<f64>, Vec<H<P>>) {
    let n = pw.len() - 1;
    let m = n + p + 1;
    let ph = p + t;
    let ph2 = ph / 2;
    // Bézier degree-elevation coefficients.
    let mut bez = vec![vec![0.0; p + 1]; ph + 1];
    bez[0][0] = 1.0;
    bez[ph][p] = 1.0;
    for i in 1..=ph2 {
        let inv = 1.0 / binom(ph, i);
        for j in i.saturating_sub(t)..=p.min(i) {
            bez[i][j] = inv * binom(p, j) * binom(t, i - j);
        }
    }
    for i in ph2 + 1..ph {
        for j in i.saturating_sub(t)..=p.min(i) {
            bez[i][j] = bez[ph - i][p - j];
        }
    }
    let segs = interior_breaks(p, u) + 1;
    let nq = n + 1 + t * segs;
    let mut q = vec![H::<P>::zero(); nq];
    let mut uh = vec![0.0; nq + ph + 1];
    let mut bpts: Vec<H<P>> = pw[..=p].to_vec();
    let mut ebpts = vec![H::<P>::zero(); ph + 1];
    let mut next = vec![H::<P>::zero(); p.max(1)];
    let mut alfs = vec![0.0; p.max(1)];

    let mut kind = ph + 1;
    let mut r: isize = -1;
    let mut a = p;
    let mut b = p + 1;
    let mut cind = 1usize;
    let mut ua = u[0];
    q[0] = pw[0];
    uh[..=ph].fill(ua);
    while b < m {
        let i0 = b;
        while b < m && u[b] == u[b + 1] {
            b += 1;
        }
        let mul = b - i0 + 1;
        let ub = u[b];
        let oldr = r;
        r = p as isize - mul as isize;
        let lbz = if oldr > 0 {
            ((oldr + 2) / 2) as usize
        } else {
            1
        };
        let rbz = if r > 0 {
            ph - ((r + 1) / 2) as usize
        } else {
            ph
        };
        if r > 0 {
            // Insert u(b) r times into the current Bézier segment.
            let numer = ub - ua;
            for k in (mul + 1..=p).rev() {
                alfs[k - mul - 1] = numer / (u[a + k] - ua);
            }
            for j in 1..=r as usize {
                let save = r as usize - j;
                let s = mul + j;
                for k in (s..=p).rev() {
                    bpts[k] = mix(bpts[k], bpts[k - 1], alfs[k - s]);
                }
                next[save] = bpts[p];
            }
        }
        // Degree-elevate the Bézier segment.
        for (i, e) in ebpts.iter_mut().enumerate().skip(lbz) {
            let mut acc = H::zero();
            for j in i.saturating_sub(t)..=p.min(i) {
                acc = acc + bpts[j] * bez[i][j];
            }
            *e = acc;
        }
        if oldr > 1 {
            // Remove knot u = U[a] oldr − 1 times.
            let mut first = kind - 2;
            let mut last = kind;
            let den = ub - ua;
            let bet = (ub - uh[kind - 1]) / den;
            for tr in 1..oldr as usize {
                let (mut i, mut j) = (first, last);
                let mut kj = j as isize - kind as isize + 1;
                while j as isize - i as isize > tr as isize {
                    if i < cind {
                        let alf = (ub - uh[i]) / (ua - uh[i]);
                        q[i] = mix(q[i], q[i - 1], alf);
                    }
                    if j >= lbz {
                        let kju = kj as usize;
                        if j as isize - tr as isize <= kind as isize - ph as isize + oldr {
                            let gam = (ub - uh[j - tr]) / den;
                            ebpts[kju] = mix(ebpts[kju], ebpts[kju + 1], gam);
                        } else {
                            ebpts[kju] = mix(ebpts[kju], ebpts[kju + 1], bet);
                        }
                    }
                    i += 1;
                    j -= 1;
                    kj -= 1;
                }
                first -= 1;
                last += 1;
            }
        }
        if a != p {
            // Load the knot ua.
            for _ in 0..(ph as isize - oldr) {
                uh[kind] = ua;
                kind += 1;
            }
        }
        for e in &ebpts[lbz..=rbz] {
            q[cind] = *e;
            cind += 1;
        }
        if b < m {
            let rr = r.max(0) as usize;
            bpts[..rr].copy_from_slice(&next[..rr]);
            bpts[rr..=p].copy_from_slice(&pw[b - p + rr..=b]);
            a = b;
            b += 1;
            ua = ub;
        } else {
            uh[kind..=kind + ph].fill(ub);
        }
    }
    debug_assert_eq!(cind, nq, "A5.9 pole count");
    q.truncate(cind);
    uh.truncate(cind + ph + 1);
    (uh, q)
}

/// Largest ratio of neighbouring non-empty span lengths (1 for a single span). A5.9's
/// knot-removal step extrapolates by about this factor per removed copy.
pub(crate) fn span_ratio(p: usize, u: &[f64]) -> f64 {
    let m = u.len() - 1;
    let mut prev: Option<f64> = None;
    let mut worst = 1.0f64;
    for i in p..m - p {
        let len = u[i + 1] - u[i];
        if len > 0.0 {
            if let Some(l) = prev {
                worst = worst.max(len / l).max(l / len);
            }
            prev = Some(len);
        }
    }
    worst
}

/// Degree elevation by `t >= 1` that stays accurate on strongly non-uniform knots (used
/// instead of [`elevate`] above [`ELEVATE_STABLE_RATIO`]): A5.6 decomposition, Bézier
/// elevation (both convex), then every pole of the result is the blossom of one elevated
/// piece at its knot window `Uh[j+1..=j+p+t]` (any piece in the pole's support gives the same
/// value in exact arithmetic; the piece with the least extrapolation is used). Same knots and
/// pole count as A5.9.
pub(crate) fn elevate_stable<P: CtrlPt>(
    p: usize,
    u: &[f64],
    pw: &[H<P>],
    t: usize,
) -> (Vec<f64>, Vec<H<P>>) {
    let q = p + t;
    let segs = decompose(p, u, pw);
    // Bézier elevation coefficients (P&T eq. 5.36).
    let mut bez = vec![vec![0.0; p + 1]; q + 1];
    for (i, row) in bez.iter_mut().enumerate() {
        for (j, v) in row
            .iter_mut()
            .enumerate()
            .take(p.min(i) + 1)
            .skip(i.saturating_sub(t))
        {
            *v = binom(p, j) * binom(t, i - j) / binom(q, i);
        }
    }
    let pieces: Vec<(f64, f64, Vec<H<P>>)> = segs
        .into_iter()
        .map(|s| {
            let e = (0..=q)
                .map(|i| {
                    let mut acc = H::zero();
                    for j in i.saturating_sub(t)..=p.min(i) {
                        acc = acc + s.pts[j] * bez[i][j];
                    }
                    acc
                })
                .collect();
            (s.range.0, s.range.1, e)
        })
        .collect();
    // Elevated knots: every distinct knot's multiplicity + t.
    let mut uh = Vec::with_capacity(u.len() + t * (pieces.len() + 1));
    let mut i = 0;
    while i < u.len() {
        let v = u[i];
        while i < u.len() && u[i] == v {
            uh.push(v);
            i += 1;
        }
        uh.extend(std::iter::repeat_n(v, t));
    }
    let nq = uh.len() - q - 1;
    let mut out = Vec::with_capacity(nq);
    let mut b = vec![H::<P>::zero(); q + 1];
    for j in 0..nq {
        let args = &uh[j + 1..=j + q];
        let (lo, hi) = (uh[j], uh[j + q + 1]);
        // Pieces inside the support [lo, hi].
        let first = pieces.partition_point(|s| s.0 < lo);
        let mut best = first.min(pieces.len() - 1);
        let mut best_cost = f64::INFINITY;
        for (k, s) in pieces.iter().enumerate().skip(first) {
            if s.1 > hi {
                break;
            }
            // Extrapolation growth of de Casteljau outside the piece: Π (1 + 2·d/len).
            let inv_len = 1.0 / (s.1 - s.0);
            let cost: f64 = args
                .iter()
                .map(|&x| 1.0 + 2.0 * (s.0 - x).max(x - s.1).max(0.0) * inv_len)
                .product();
            if cost < best_cost {
                best_cost = cost;
                best = k;
            }
        }
        let (a0, a1, e) = &pieces[best];
        b.copy_from_slice(e);
        let len = a1 - a0;
        for (r, &x) in args.iter().enumerate() {
            let lam = (x - a0) / len;
            for i in 0..q - r {
                b[i] = mix(b[i + 1], b[i], lam);
            }
        }
        out.push(b[0]);
    }
    (uh, out)
}

/// Neighbouring-span ratio above which [`elevate_stable`] replaces A5.9. Measured on 20 000
/// random curves (degree 1–7, by 1–3): A5.9 stays below 0.006·ε up to ratio 4, reaches
/// 0.05·ε at 8 and 1e5·ε at 2000; the blossom path stays below 0.003·ε everywhere.
pub(crate) const ELEVATE_STABLE_RATIO: f64 = 4.0;

/// Degree elevation by `t >= 1`: A5.9 on well-graded knots, [`elevate_stable`] otherwise.
pub(crate) fn elevate_any<P: CtrlPt>(
    p: usize,
    u: &[f64],
    pw: &[H<P>],
    t: usize,
) -> (Vec<f64>, Vec<H<P>>) {
    if span_ratio(p, u) > ELEVATE_STABLE_RATIO {
        elevate_stable(p, u, pw, t)
    } else {
        elevate(p, u, pw, t)
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp, reason = "exact expected values")]
mod tests {
    use super::*;

    #[test]
    fn binomials() {
        assert_eq!(binom(5, 2), 10.0);
        assert_eq!(binom(25, 12), 5_200_300.0);
        assert_eq!(binom(50, 25), 126_410_606_437_752.0);
        assert_eq!(binom(3, 4), 0.0);
    }

    #[test]
    fn breaks() {
        assert_eq!(interior_breaks(2, &[0., 0., 0., 1., 2., 2., 3., 3., 3.]), 2);
        assert_eq!(interior_breaks(1, &[0., 0., 1., 1.]), 0);
    }
}
