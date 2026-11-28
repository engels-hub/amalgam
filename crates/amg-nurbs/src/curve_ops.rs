//! Curve knot operations (KH-12, P&T ch. 5): insertion, refinement, Bézier decomposition,
//! removal, degree elevation, split / segment / join and continuity at a knot.
//!
//! All shape-preserving operations keep the parameter mapping: `op(c).eval(t) == c.eval(t)`
//! up to rounding for every `t` in the (unchanged) domain.

#![allow(
    clippy::float_cmp,
    reason = "knots are compared exactly: equal values form one multiplicity run"
)]

use amg_core::Tolerance;

use crate::knot_ops::{self, H, from_h, to_h};
use crate::knots::find_span;
use crate::{Continuity, CtrlPt, KnotVector, MAX_DEGREE, NurbsCurve, NurbsError};

/// Relative tolerance (of the domain scale) within which a parameter snaps onto an existing
/// knot. Snapping never changes the shape (any knot value is shape-preserving); it only avoids
/// near-zero spans.
pub(crate) const KNOT_SNAP_REL: f64 = 1e-12;

/// Absolute snap distance for a domain `(a, b)`.
#[inline]
pub(crate) fn snap_eps(a: f64, b: f64) -> f64 {
    KNOT_SNAP_REL * (b - a).max(a.abs()).max(b.abs())
}

/// `t` moved onto the nearest knot of `u` if it lies within `eps` of it.
pub(crate) fn snap(u: &[f64], t: f64, eps: f64) -> f64 {
    let i = u.partition_point(|&k| k < t);
    let mut best = t;
    let mut bd = eps;
    for &k in [i.checked_sub(1).map(|j| u[j]), u.get(i).copied()]
        .iter()
        .flatten()
    {
        let d = (k - t).abs();
        if d <= bd {
            bd = d;
            best = k;
        }
    }
    best
}

/// Exact multiplicity of the knot value `t` and the index of its last copy.
pub(crate) fn mult_last(u: &[f64], t: f64) -> (usize, usize) {
    let hi = u.partition_point(|&k| k <= t);
    let lo = u.partition_point(|&k| k < t);
    (hi - lo, hi.saturating_sub(1))
}

/// Validates insertion parameters and snaps them; returns sorted interior knots (domain ends
/// dropped). `OutOfDomain` for non-finite or outside values, `InvalidKnots` if a resulting
/// multiplicity would exceed `degree`.
pub(crate) fn prepare_refine(
    u: &[f64],
    degree: usize,
    (a, b): (f64, f64),
    ts: &[f64],
) -> Result<Vec<f64>, NurbsError> {
    let eps = snap_eps(a, b);
    let mut x = Vec::with_capacity(ts.len());
    for &t in ts {
        if !(t.is_finite() && t >= a - eps && t <= b + eps) {
            return Err(NurbsError::OutOfDomain(t));
        }
        let t = snap(u, t, eps);
        if t - a > eps && b - t > eps {
            x.push(t);
        }
    }
    x.sort_by(f64::total_cmp);
    let mut i = 0;
    while i < x.len() {
        let v = x[i];
        let mut j = i;
        while j < x.len() && x[j] == v {
            j += 1;
        }
        if mult_last(u, v).0 + (j - i) > degree {
            return Err(NurbsError::InvalidKnots(
                "interior knot multiplicity exceeds the degree",
            ));
        }
        i = j;
    }
    Ok(x)
}

impl<P: CtrlPt> NurbsCurve<P> {
    fn hpoints(&self) -> Vec<H<P>> {
        to_h(self.poles(), self.weights())
    }

    /// Rebuilds a curve of this curve's kind (rational iff `self` stores weights).
    fn rebuild(&self, degree: usize, knots: Vec<f64>, hs: &[H<P>]) -> Self {
        let (poles, weights) = from_h(hs, self.weights().is_some());
        NurbsCurve::new_unchecked(degree, KnotVector::new_unchecked(knots), poles, weights)
    }

    /// Inserts the knot `t` up to `times` times (A5.1); the multiplicity is capped at the
    /// degree. `t` within 1e-12 (relative) of an existing knot snaps onto it; `t` at a domain
    /// end returns an unchanged copy. `OutOfDomain` if `t` is not finite or outside the domain.
    pub fn insert_knot(&self, t: f64, times: usize) -> Result<Self, NurbsError> {
        let (a, b) = self.domain();
        let u = self.knots().as_slice();
        let eps = snap_eps(a, b);
        if !(t.is_finite() && t >= a - eps && t <= b + eps) {
            return Err(NurbsError::OutOfDomain(t));
        }
        let t = snap(u, t, eps);
        if t - a <= eps || b - t <= eps {
            return Ok(self.clone());
        }
        let p = self.degree();
        let (s, _) = mult_last(u, t);
        let r = times.min(p.saturating_sub(s));
        if r == 0 {
            return Ok(self.clone());
        }
        let k = find_span(u, p, self.poles().len(), t);
        let (uq, q) = knot_ops::insert(p, u, &self.hpoints(), t, k, s, r);
        Ok(self.rebuild(p, uq, &q))
    }

    /// Inserts all of `ts` at once (A5.4). Order does not matter; values snap onto existing
    /// knots like [`NurbsCurve::insert_knot`], domain ends are ignored. `OutOfDomain` for a
    /// value outside the domain, `InvalidKnots` if a multiplicity would exceed the degree.
    pub fn refine_knots(&self, ts: &[f64]) -> Result<Self, NurbsError> {
        let p = self.degree();
        let u = self.knots().as_slice();
        let x = prepare_refine(u, p, self.domain(), ts)?;
        if x.is_empty() {
            return Ok(self.clone());
        }
        let (ub, q) = knot_ops::refine(p, u, &self.hpoints(), &x);
        Ok(self.rebuild(p, ub, &q))
    }

    /// Removes up to `times` copies of the interior knot `t` (A5.8, one copy per pass, see
    /// the crate's `knot_ops::remove_one`) while the curve moves by at most `tol` (model
    /// units). Each pass bounds its deviation by the residual of the one redundant equation
    /// (for rational curves scaled by `(1 + |P|max) / w_min`, P&T eq. 5.30); the bounds are
    /// summed over the removed copies, so the total deviation stays `<= tol`. Returns the new curve and the number of copies removed (0 if `t` is not an
    /// interior knot or nothing can be removed).
    pub fn remove_knot(&self, t: f64, times: usize, tol: f64) -> (Self, usize) {
        let (a, b) = self.domain();
        let p = self.degree();
        let eps = snap_eps(a, b);
        let mut u = self.knots().as_slice().to_vec();
        if !t.is_finite() || times == 0 || tol.is_nan() || tol < 0.0 {
            return (self.clone(), 0);
        }
        let t = snap(&u, t, eps);
        let (s, _) = mult_last(&u, t);
        if s == 0 || t - a <= eps || b - t <= eps {
            return (self.clone(), 0);
        }
        // Homogeneous distance → model-space deviation factor (eq. 5.30).
        let scale = match self.weights() {
            None => 1.0,
            Some(w) => {
                let wmin = w.iter().copied().fold(f64::INFINITY, f64::min);
                let pmax = self.poles().iter().map(|q| q.length()).fold(0.0, f64::max);
                (1.0 + pmax) / wmin
            }
        };
        let mut pw = self.hpoints();
        let mut spent = 0.0;
        let mut removed = 0;
        while removed < times.min(s) {
            let (s_now, r_now) = mult_last(&u, t);
            let ok = knot_ops::remove_one(p, &mut u, &mut pw, r_now, s_now, |d| {
                let e = d * scale;
                if spent + e <= tol {
                    spent += e;
                    true
                } else {
                    false
                }
            });
            if !ok {
                break;
            }
            removed += 1;
        }
        if removed == 0 {
            return (self.clone(), 0);
        }
        (self.rebuild(p, u, &pw), removed)
    }

    /// Raises the degree by `by` (A5.9); the shape and parameter mapping are unchanged.
    /// Knot vectors whose neighbouring spans differ by more than 4× use an equivalent
    /// blossom-based path (A5.6 decomposition + Bézier elevation + blossoming) because A5.9's
    /// knot-removal step loses accuracy there; both give the same knots and pole count.
    /// `Degree` if the result would exceed [`MAX_DEGREE`].
    pub fn elevate_degree(&self, by: usize) -> Result<Self, NurbsError> {
        let p = self.degree();
        if by == 0 {
            return Ok(self.clone());
        }
        let ph = p.saturating_add(by);
        if ph > MAX_DEGREE {
            return Err(NurbsError::Degree(ph));
        }
        let u = self.knots().as_slice();
        let (uh, q) = knot_ops::elevate_any(p, u, &self.hpoints(), by);
        Ok(self.rebuild(ph, uh, &q))
    }

    /// The Bézier segments of every non-empty span, in order (A5.6). Each segment keeps the
    /// curve's parameters: segment `i` has the knots `[u_i; p+1] ++ [u_{i+1}; p+1]`.
    pub fn bezier_segments(&self) -> Vec<Self> {
        let p = self.degree();
        knot_ops::decompose(p, self.knots().as_slice(), &self.hpoints())
            .into_iter()
            .map(|seg| {
                let (a, b) = seg.range;
                let mut k = vec![a; p + 1];
                k.resize(2 * (p + 1), b);
                self.rebuild(p, k, &seg.pts)
            })
            .collect()
    }

    /// Splits at `t` (knot insertion to multiplicity `degree`): `left` on `[a, t]`, `right`
    /// on `[t, b]`, sharing the pole at `t` bit-identically. `t` snaps onto a knot within
    /// 1e-12 (relative). `OutOfDomain` unless `a < t < b`.
    pub fn split(&self, t: f64) -> Result<(Self, Self), NurbsError> {
        let (a, b) = self.domain();
        let eps = snap_eps(a, b);
        let p = self.degree();
        if !(t.is_finite() && t - a > eps && b - t > eps) {
            return Err(NurbsError::OutOfDomain(t));
        }
        let t = snap(self.knots().as_slice(), t, eps);
        if !(t - a > eps && b - t > eps) {
            return Err(NurbsError::OutOfDomain(t));
        }
        let full = self.insert_knot(t, p)?;
        let u = full.knots().as_slice();
        // First copy of t; it now has multiplicity exactly p.
        let f = u.partition_point(|&k| k < t);
        let hs = full.hpoints();
        let mut lk = u[..f + p].to_vec();
        lk.push(t);
        let mut rk = vec![t];
        rk.extend_from_slice(&u[f..]);
        let left = full.rebuild(p, lk, &hs[..f]);
        let right = full.rebuild(p, rk, &hs[f - 1..]);
        Ok((left, right))
    }

    /// The piece on `[t0, t1]` (same parameters). `OutOfDomain` unless
    /// `a <= t0 < t1 <= b` (ends within 1e-12 relative count as the domain ends).
    pub fn segment(&self, t0: f64, t1: f64) -> Result<Self, NurbsError> {
        let (a, b) = self.domain();
        let eps = snap_eps(a, b);
        if !(t0.is_finite() && t0 >= a - eps && t0 < b) {
            return Err(NurbsError::OutOfDomain(t0));
        }
        if !(t1.is_finite() && t1 <= b + eps && t1 > t0 + eps) {
            return Err(NurbsError::OutOfDomain(t1));
        }
        let mut c = if t0 - a > eps {
            self.split(t0)?.1
        } else {
            self.clone()
        };
        if b - t1 > eps {
            c = c.split(t1)?.0;
        }
        Ok(c)
    }

    /// G0 join: `other` follows `self` (end of `self` == start of `other` within
    /// `tol.linear`, else `Incompatible`). The lower-degree curve is elevated first, `other`
    /// is shifted to start at `self`'s end parameter (keeping its parameter length), and the
    /// knots are concatenated with multiplicity `degree` at the joint. The joint pole is
    /// `self`'s end pole; for rational input `other`'s weights are scaled so the joint
    /// weights agree (the curve is unchanged by a common weight factor).
    pub fn join(&self, other: &Self, tol: &Tolerance) -> Result<Self, NurbsError> {
        let gap = (other.start() - self.end()).length();
        if gap.is_nan() || gap > tol.linear {
            return Err(NurbsError::Incompatible(format!(
                "curves do not meet: gap {gap:e} mm > {:e} mm",
                tol.linear
            )));
        }
        let p = self.degree().max(other.degree());
        let lhs = self.elevate_degree(p - self.degree())?;
        let rhs = other.elevate_degree(p - other.degree())?;
        let (_, b) = lhs.domain();
        let (c, d) = rhs.domain();
        let rhs = if c.to_bits() == b.to_bits() {
            rhs
        } else {
            rhs.reparametrized((b, b + (d - c)))
        };
        let lu = lhs.knots().as_slice();
        let ru = rhs.knots().as_slice();
        let mut knots = lu[..lu.len() - 1].to_vec();
        knots.extend_from_slice(&ru[p + 1..]);
        let mut poles = lhs.poles().to_vec();
        poles.extend_from_slice(&rhs.poles()[1..]);
        let weights = if lhs.weights().is_none() && rhs.weights().is_none() {
            None
        } else {
            let lw: Vec<f64> = lhs
                .weights()
                .map_or_else(|| vec![1.0; lhs.poles().len()], <[f64]>::to_vec);
            let rw: Vec<f64> = rhs
                .weights()
                .map_or_else(|| vec![1.0; rhs.poles().len()], <[f64]>::to_vec);
            let f = lw[lw.len() - 1] / rw[0];
            let mut w = lw;
            w.extend(rw[1..].iter().map(|&x| x * f));
            Some(w)
        };
        // The joint has multiplicity p. Only a knot of `other` that rounds onto the joint
        // under the shift could exceed that; the validated constructor reports it.
        NurbsCurve::new(p, KnotVector::new(knots)?, poles, weights)
    }

    /// Continuity class at `t`. Parameters that are not interior knots (within 1e-12
    /// relative) give [`Continuity::Cn`]; the domain ends give [`Continuity::C0`].
    ///
    /// At a knot the left and right derivatives are compared: tangent direction within
    /// `tol.angular` rad (G1), first / second derivative vectors and the curvature vector
    /// within `tol.angular` relative (C1 / C2 / G2). [`Continuity::Cn`] means the knot leaves
    /// at least C3 (multiplicity `<= degree − 3`, or matching third derivatives). A joint that
    /// is G2 but not C1 is reported as G1 (the order of [`Continuity`] puts G2 above C1).
    pub fn continuity_at(&self, t: f64, tol: &Tolerance) -> Continuity {
        let (a, b) = self.domain();
        let eps = snap_eps(a, b);
        if !t.is_finite() || t - a <= eps || b - t <= eps {
            return Continuity::C0;
        }
        let u = self.knots().as_slice();
        let t = snap(u, t, eps);
        let (m, _) = mult_last(u, t);
        if m == 0 {
            return Continuity::Cn;
        }
        let p = self.degree();
        if p >= m + 3 {
            return Continuity::Cn;
        }
        let right = self.derivs(t, 3);
        let rev = self.reversed();
        let mut left = rev.derivs((a + b) - t, 3);
        for (k, d) in left.iter_mut().enumerate() {
            if k % 2 == 1 {
                *d = *d * -1.0;
            }
        }
        let rel = tol.angular;
        let close = |x: P, y: P, floor: f64| {
            (x - y).length() <= rel * x.length().max(y.length()).max(floor)
        };
        let (l1, r1) = (left[1], right[1]);
        let (nl, nr) = (l1.length(), r1.length());
        if !(nl > 0.0 && nr > 0.0) {
            // Vanishing tangent: no direction to compare.
            return Continuity::C0;
        }
        // Angle between the unit tangents from their chord (2·sin(θ/2), accurate near 0).
        let chord = (l1 * (1.0 / nl) - r1 * (1.0 / nr)).length();
        let angle = 2.0 * (0.5 * chord).min(1.0).asin();
        if angle > tol.angular {
            return Continuity::C0;
        }
        if !close(l1, r1, 0.0) {
            return Continuity::G1;
        }
        let (l2, r2) = (left[2], right[2]);
        let scale1 = nl.max(nr);
        if !close(l2, r2, scale1) {
            // Curvature vectors κ = (C'' − T (T·C'')) / |C'|².
            let kv = |d1: P, d2: P| {
                let n2 = d1.dot(d1);
                (d2 - d1 * (d1.dot(d2) / n2)) * (1.0 / n2)
            };
            let (kl, kr) = (kv(l1, l2), kv(r1, r2));
            // Curvature floor: a circle the size of the control polygon.
            let p0 = self.poles()[0];
            let size = self
                .poles()
                .iter()
                .map(|&q| (q - p0).length())
                .fold(0.0, f64::max);
            let kfloor = if size > 0.0 { 1.0 / size } else { 0.0 };
            return if close(kl, kr, kfloor) {
                Continuity::G2
            } else {
                Continuity::C1
            };
        }
        let scale2 = scale1.max(l2.length()).max(r2.length());
        if close(left[3], right[3], scale2) {
            Continuity::Cn
        } else {
            Continuity::C2
        }
    }
}
