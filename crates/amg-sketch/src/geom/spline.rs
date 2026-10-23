//! B-splines and NURBS (SK-08): evaluation, queries and the fit-point interpolation map.
//!
//! A [`BSpline2`] is a clamped, non-periodic rational B-spline
//! `C(t) = Σ N_i(t) w_i P_i / Σ N_i(t) w_i` of degree 1..=[`MAX_DEGREE`]. Weights are positive,
//! so the curve lies in the convex hull of its poles.
//!
//! **Parameterisation.** As for every [`Curve2`], the curve parameter runs from `0` to
//! [`BSpline2::span`] in the direction of travel. A spline built from a knot vector
//! `[a, …, b]` is shifted to `[0, b − a]`. Conics are quadratic Béziers on `[0, 1]`.
//!
//! **Fit-point splines** ([`FitMap`]). Cubic interpolation through fit points `Q_k` at frozen
//! parameters `ū_k` (chord length at creation, normalised to `[0, 1]`) with either a natural
//! end (`C'' = 0`) or a given end derivative (a tangent handle `H`, `C'(end) = (H − Q)/Δū`).
//! The poles are then a **linear map** of the fit points and handles, `P = M · [Q; H]`, which is
//! what the solver differentiates.

#![allow(
    clippy::needless_range_loop,
    reason = "numeric kernels index several arrays in lockstep"
)]

use amg_core::{Aabb2, Point2, Vec2};

use super::ellipse::integrate;
use crate::error::SketchError;

/// Largest supported degree.
pub const MAX_DEGREE: usize = 10;

/// Homogeneous pole `(w·x, w·y, w)`.
pub(crate) type Hom = [f64; 3];

fn hom(p: Point2, w: f64) -> Hom {
    [p.x * w, p.y * w, w]
}

fn lerp3(a: Hom, b: Hom, s: f64) -> Hom {
    [
        a[0] + (b[0] - a[0]) * s,
        a[1] + (b[1] - a[1]) * s,
        a[2] + (b[2] - a[2]) * s,
    ]
}

fn unhom(h: Hom) -> (Point2, f64) {
    (Point2::new(h[0] / h[2], h[1] / h[2]), h[2])
}

/// A clamped rational B-spline (see the [module docs](self)).
#[derive(Clone, Debug, PartialEq)]
pub struct BSpline2 {
    degree: usize,
    knots: Vec<f64>,
    poles: Vec<Point2>,
    weights: Vec<f64>,
}

/// Uniform clamped knot vector for `n_poles` poles of degree `degree`: `degree + 1` zeros,
/// interior knots `1, 2, …, n_poles − degree − 1`, then `degree + 1` copies of
/// `n_poles − degree`.
pub fn clamped_uniform_knots(n_poles: usize, degree: usize) -> Vec<f64> {
    let mut k = vec![0.0; degree + 1];
    for i in 1..n_poles.saturating_sub(degree) {
        k.push(i as f64);
    }
    let end = n_poles.saturating_sub(degree) as f64;
    k.extend(std::iter::repeat_n(end, degree + 1));
    k
}

/// Validates a clamped knot vector for `n_poles` poles; `Err` with the reason.
pub fn check_knots(degree: usize, n_poles: usize, knots: &[f64]) -> Result<(), SketchError> {
    if !(1..=MAX_DEGREE).contains(&degree) {
        return Err(SketchError::NotApplicable(
            "spline degree must be between 1 and 10",
        ));
    }
    if n_poles < degree + 1 {
        return Err(SketchError::Degenerate(
            "a spline of degree p needs at least p + 1 poles",
        ));
    }
    if knots.len() != n_poles + degree + 1 {
        return Err(SketchError::NotApplicable(
            "the knot vector must have poles + degree + 1 entries",
        ));
    }
    if let Some(&k) = knots.iter().find(|k| !k.is_finite()) {
        return Err(SketchError::InvalidValue {
            what: "spline knot",
            value: k,
        });
    }
    if knots.windows(2).any(|w| w[1] < w[0]) {
        return Err(SketchError::NotApplicable(
            "the knot vector must be non-decreasing",
        ));
    }
    let (first, last) = (knots[0], knots[knots.len() - 1]);
    if last <= first {
        return Err(SketchError::Degenerate(
            "the spline's parameter range is empty",
        ));
    }
    let clamped = knots[..=degree]
        .iter()
        .all(|&k| k.to_bits() == first.to_bits())
        && knots[knots.len() - degree - 1..]
            .iter()
            .all(|&k| k.to_bits() == last.to_bits());
    if !clamped {
        return Err(SketchError::NotApplicable(
            "the knot vector must be clamped (end knots repeated degree + 1 times)",
        ));
    }
    if knots[degree + 1] <= first || knots[knots.len() - degree - 2] >= last {
        return Err(SketchError::NotApplicable(
            "the end knots must be repeated exactly degree + 1 times",
        ));
    }
    // Interior multiplicity at most `degree` (higher splits the curve).
    let interior = &knots[degree + 1..knots.len() - degree - 1];
    let mut run = 0usize;
    for (i, &k) in interior.iter().enumerate() {
        if i > 0 && k.to_bits() == interior[i - 1].to_bits() {
            run += 1;
        } else {
            run = 1;
        }
        if run > degree {
            return Err(SketchError::NotApplicable(
                "an interior knot is repeated more than degree times",
            ));
        }
    }
    Ok(())
}

impl BSpline2 {
    /// A spline from its parts. `knots` must be clamped with `poles + degree + 1` entries;
    /// weights positive. The knots are shifted to start at 0.
    pub fn new(
        degree: usize,
        knots: Vec<f64>,
        poles: Vec<Point2>,
        weights: Vec<f64>,
    ) -> Result<BSpline2, SketchError> {
        check_knots(degree, poles.len(), &knots)?;
        if weights.len() != poles.len() {
            return Err(SketchError::NotApplicable("one weight per pole is needed"));
        }
        for p in &poles {
            if !p.is_finite() {
                return Err(SketchError::InvalidValue {
                    what: "spline pole",
                    value: if p.x.is_finite() { p.y } else { p.x },
                });
            }
        }
        if let Some(&w) = weights.iter().find(|&&w| !(w.is_finite() && w > 0.0)) {
            return Err(SketchError::InvalidValue {
                what: "spline weight",
                value: w,
            });
        }
        let k0 = knots[0];
        let knots = knots.into_iter().map(|k| k - k0).collect();
        Ok(BSpline2 {
            degree,
            knots,
            poles,
            weights,
        })
    }

    /// The conic through `start` and `end` with the apex (shoulder control point) `apex` and
    /// shape `rho ∈ (0, 1)`: the quadratic rational Bézier with weights `(1, ρ/(1 − ρ), 1)`.
    /// `ρ = ½` is a parabola, smaller values an ellipse, larger a hyperbola. The curve passes
    /// through `S + ρ (apex − S)` with `S = (start + end)/2` (at `t = ½`); see [`conic_weight`].
    pub fn conic(
        start: Point2,
        apex: Point2,
        end: Point2,
        rho: f64,
    ) -> Result<BSpline2, SketchError> {
        BSpline2::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![start, apex, end],
            vec![1.0, conic_weight(rho)?, 1.0],
        )
    }

    /// Degree.
    pub fn degree(&self) -> usize {
        self.degree
    }

    /// The (shifted) knot vector.
    pub fn knots(&self) -> &[f64] {
        &self.knots
    }

    /// The poles.
    pub fn poles(&self) -> &[Point2] {
        &self.poles
    }

    /// The weights, one per pole.
    pub fn weights(&self) -> &[f64] {
        &self.weights
    }

    /// Upper end of the parameter range (the lower end is 0).
    pub fn span(&self) -> f64 {
        self.knots[self.knots.len() - 1]
    }

    /// True if any weight differs from 1 (a rational curve).
    pub fn is_rational(&self) -> bool {
        self.weights.iter().any(|&w| (w - 1.0).abs() > 0.0)
    }

    fn n(&self) -> usize {
        self.poles.len() - 1
    }

    fn hom_poles(&self) -> Vec<Hom> {
        self.poles
            .iter()
            .zip(&self.weights)
            .map(|(&p, &w)| hom(p, w))
            .collect()
    }

    fn from_hom(degree: usize, knots: Vec<f64>, pw: &[Hom]) -> BSpline2 {
        let (poles, weights) = pw.iter().map(|&h| unhom(h)).unzip();
        BSpline2 {
            degree,
            knots,
            poles,
            weights,
        }
    }

    /// Index `i` of the knot span `[U_i, U_{i+1})` containing `t` (`p ≤ i ≤ n`).
    pub(crate) fn find_span(&self, t: f64) -> usize {
        let n = self.n();
        let p = self.degree;
        if t >= self.knots[n + 1] {
            return n;
        }
        if t <= self.knots[p] {
            return p;
        }
        let (mut lo, mut hi) = (p, n + 1);
        while hi - lo > 1 {
            let mid = (lo + hi) / 2;
            if t < self.knots[mid] {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        lo
    }

    /// Non-zero basis functions and their first two derivatives on `span` (Piegl & Tiller
    /// A2.3): `ders[k][j]` is the `k`-th derivative of `N_{span − p + j}`.
    pub(crate) fn basis_derivs(&self, span: usize, t: f64) -> [[f64; MAX_DEGREE + 1]; 3] {
        const D: usize = MAX_DEGREE + 1;
        let p = self.degree;
        let u = &self.knots;
        let mut ndu = [[0.0f64; D]; D];
        let (mut left, mut right) = ([0.0f64; D], [0.0f64; D]);
        ndu[0][0] = 1.0;
        for j in 1..=p {
            left[j] = t - u[span + 1 - j];
            right[j] = u[span + j] - t;
            let mut saved = 0.0;
            for r in 0..j {
                ndu[j][r] = right[r + 1] + left[j - r];
                let temp = ndu[r][j - 1] / ndu[j][r];
                ndu[r][j] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }
            ndu[j][j] = saved;
        }
        let mut ders = [[0.0f64; D]; 3];
        for j in 0..=p {
            ders[0][j] = ndu[j][p];
        }
        let nd = 2.min(p);
        let mut a = [[0.0f64; D]; 2];
        for r in 0..=p {
            let (mut s1, mut s2) = (0usize, 1usize);
            a[0][0] = 1.0;
            for k in 1..=nd {
                let mut d = 0.0;
                let rk = r as isize - k as isize;
                let pk = p as isize - k as isize;
                if r >= k {
                    a[s2][0] = a[s1][0] / ndu[(pk + 1) as usize][rk as usize];
                    d = a[s2][0] * ndu[rk as usize][pk as usize];
                }
                let j1 = if rk >= -1 { 1 } else { (-rk) as usize };
                let j2 = if (r as isize) - 1 <= pk { k - 1 } else { p - r };
                for j in j1..=j2 {
                    let idx = (rk + j as isize) as usize;
                    a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[(pk + 1) as usize][idx];
                    d += a[s2][j] * ndu[idx][pk as usize];
                }
                if (r as isize) <= pk {
                    a[s2][k] = -a[s1][k - 1] / ndu[(pk + 1) as usize][r];
                    d += a[s2][k] * ndu[r][pk as usize];
                }
                ders[k][r] = d;
                std::mem::swap(&mut s1, &mut s2);
            }
        }
        let mut fac = p as f64;
        for k in 1..=nd {
            for j in 0..=p {
                ders[k][j] *= fac;
            }
            fac *= (p - k) as f64;
        }
        ders
    }

    /// A spline from parts that are already valid (`check_knots`, positive weights); used by the
    /// solver rows, which evaluate at every iteration.
    pub(crate) fn unchecked(
        degree: usize,
        knots: Vec<f64>,
        poles: Vec<Point2>,
        weights: Vec<f64>,
    ) -> BSpline2 {
        BSpline2 {
            degree,
            knots,
            poles,
            weights,
        }
    }

    /// The knot span containing `t` (the end spans extend polynomially beyond the range) and the
    /// values of its `degree + 1` non-zero basis functions.
    pub(crate) fn basis_at(&self, t: f64) -> (usize, [f64; MAX_DEGREE + 1]) {
        let span = self.find_span(t);
        (span, self.basis_derivs(span, t)[0])
    }

    /// `(C, C', C'')` at `t` (clamped to the range).
    pub fn derivs(&self, t: f64) -> (Point2, Vec2, Vec2) {
        self.derivs_ext(t.clamp(0.0, self.span()))
    }

    /// [`derivs`](BSpline2::derivs) without clamping: outside the range the end spans'
    /// polynomials continue (the solver's curve parameters may leave `[0, span]` transiently).
    pub(crate) fn derivs_ext(&self, t: f64) -> (Point2, Vec2, Vec2) {
        let span = self.find_span(t);
        let ders = self.basis_derivs(span, t);
        let p = self.degree;
        let mut a = [[0.0f64; 3]; 3];
        for (k, ak) in a.iter_mut().enumerate() {
            for j in 0..=p {
                let i = span - p + j;
                let w = self.weights[i];
                let n = ders[k][j] * w;
                ak[0] += n * self.poles[i].x;
                ak[1] += n * self.poles[i].y;
                ak[2] += n;
            }
        }
        let w = a[0][2];
        let c = Vec2::new(a[0][0], a[0][1]) / w;
        let c1 = (Vec2::new(a[1][0], a[1][1]) - c * a[1][2]) / w;
        let c2 = (Vec2::new(a[2][0], a[2][1]) - c1 * (2.0 * a[1][2]) - c * a[2][2]) / w;
        (c, c1, c2)
    }

    /// Point at `t`.
    pub fn eval(&self, t: f64) -> Point2 {
        self.derivs(t).0
    }

    /// First derivative (direction of travel).
    pub fn derivative(&self, t: f64) -> Vec2 {
        self.derivs(t).1
    }

    /// Signed curvature at `t` (positive = turning left), 1/mm; 0 where the speed vanishes.
    pub fn curvature_at(&self, t: f64) -> f64 {
        let (_, d1, d2) = self.derivs(t);
        let s = d1.length();
        if s > 0.0 {
            d1.perp_dot(d2) / (s * s * s)
        } else {
            0.0
        }
    }

    /// Distinct knot values with their multiplicity, in order.
    fn breaks(&self) -> Vec<(f64, usize)> {
        let mut out: Vec<(f64, usize)> = Vec::new();
        for &k in &self.knots {
            match out.last_mut() {
                Some((v, m)) if v.to_bits() == k.to_bits() => *m += 1,
                _ => out.push((k, 1)),
            }
        }
        out
    }

    /// Parameter intervals between distinct knots (the polynomial/rational pieces).
    pub fn spans(&self) -> Vec<(f64, f64)> {
        let b = self.breaks();
        b.windows(2).map(|w| (w[0].0, w[1].0)).collect()
    }

    /// Boehm knot insertion of `u` once (homogeneous).
    fn insert_knot(degree: usize, knots: &[f64], pw: &[Hom], u: f64) -> (Vec<f64>, Vec<Hom>) {
        let n = pw.len() - 1;
        let p = degree;
        // span k with knots[k] <= u < knots[k+1]
        let mut k = p;
        while k < n && knots[k + 1] <= u {
            k += 1;
        }
        let mut q = Vec::with_capacity(pw.len() + 1);
        for i in 0..=n + 1 {
            if i + p <= k {
                q.push(pw[i]);
            } else if i > k {
                q.push(pw[i - 1]);
            } else {
                let den = knots[i + p] - knots[i];
                let alpha = if den > 0.0 { (u - knots[i]) / den } else { 0.0 };
                q.push(lerp3(pw[i - 1], pw[i], alpha));
            }
        }
        let mut nk = knots.to_vec();
        nk.insert(k + 1, u);
        (nk, q)
    }

    /// The curve as a chain of Bézier pieces: `(lo, hi, p + 1 homogeneous poles)`.
    pub(crate) fn bezier_pieces(&self) -> Vec<(f64, f64, Vec<Hom>)> {
        let p = self.degree;
        let mut knots = self.knots.clone();
        let mut pw = self.hom_poles();
        let br = self.breaks();
        for &(u, m) in &br[1..br.len() - 1] {
            for _ in m..p {
                (knots, pw) = Self::insert_knot(p, &knots, &pw, u);
            }
        }
        let bounds: Vec<f64> = br.iter().map(|b| b.0).collect();
        (0..bounds.len() - 1)
            .map(|i| (bounds[i], bounds[i + 1], pw[i * p..=i * p + p].to_vec()))
            .collect()
    }

    /// Splits a Bézier (homogeneous poles) at local parameter `s`.
    pub(crate) fn bezier_split(pw: &[Hom], s: f64) -> (Vec<Hom>, Vec<Hom>) {
        let n = pw.len();
        let mut work = pw.to_vec();
        let mut left = Vec::with_capacity(n);
        let mut right = vec![[0.0; 3]; n];
        left.push(work[0]);
        right[n - 1] = work[n - 1];
        for r in 1..n {
            for i in 0..n - r {
                work[i] = lerp3(work[i], work[i + 1], s);
            }
            left.push(work[0]);
            right[n - 1 - r] = work[n - 1 - r];
        }
        (left, right)
    }

    /// The piece of the curve from `t0` to `t1` (reversed if `t1 < t0`), with parameter
    /// `t' = t − t0` (resp. travelled from the new start). A full-range request returns a
    /// clone.
    pub fn trimmed(&self, t0: f64, t1: f64) -> BSpline2 {
        let span = self.span();
        let (a, b) = (t0.clamp(0.0, span), t1.clamp(0.0, span));
        if a > b {
            return self.trimmed(b, a).reversed();
        }
        if a <= 0.0 && b >= span {
            return self.clone();
        }
        let p = self.degree;
        let eps = 1e-14 * span;
        let mut poles: Vec<Hom> = Vec::new();
        let mut cuts: Vec<f64> = Vec::new();
        for (lo, hi, pw) in self.bezier_pieces() {
            let (s, e) = (a.max(lo), b.min(hi));
            if e - s <= eps {
                continue;
            }
            let width = hi - lo;
            let (ls, le) = ((s - lo) / width, (e - lo) / width);
            let (head, _) = Self::bezier_split(&pw, le.min(1.0));
            let sub = if ls > 0.0 && le > 0.0 {
                Self::bezier_split(&head, (ls / le).min(1.0)).1
            } else {
                head
            };
            if poles.is_empty() {
                poles.extend(sub);
            } else {
                poles.extend(sub.into_iter().skip(1));
                cuts.push(s - a);
            }
        }
        if poles.is_empty() {
            // A zero-length request: a degenerate constant curve.
            let h = hom(self.eval(a), 1.0);
            poles = vec![h; p + 1];
        }
        let total = (b - a).max(eps);
        let mut knots = vec![0.0; p + 1];
        for c in cuts {
            knots.extend(std::iter::repeat_n(c, p));
        }
        knots.extend(std::iter::repeat_n(total, p + 1));
        Self::from_hom(p, knots, &poles)
    }

    /// The same curve traversed the other way.
    pub fn reversed(&self) -> BSpline2 {
        let span = self.span();
        BSpline2 {
            degree: self.degree,
            knots: self.knots.iter().rev().map(|&k| span - k).collect(),
            poles: self.poles.iter().rev().copied().collect(),
            weights: self.weights.iter().rev().copied().collect(),
        }
    }

    /// Samples per span used by the numeric queries.
    pub(crate) fn samples(&self) -> usize {
        24 * (self.degree + 1)
    }

    /// Arc length, mm.
    pub fn length(&self) -> f64 {
        self.spans()
            .iter()
            .map(|&(lo, hi)| integrate(|t| self.derivative(t).length(), lo, hi, 2 + self.degree))
            .sum()
    }

    /// Tight bounding box (extrema found by sign changes of the coordinate derivatives).
    pub fn bbox(&self) -> Aabb2 {
        let mut b = Aabb2::from_points([self.eval(0.0), self.eval(self.span())]);
        let n = self.samples();
        for (lo, hi) in self.spans() {
            for axis in 0..2 {
                let g = |t: f64| {
                    let d = self.derivative(t);
                    if axis == 0 { d.x } else { d.y }
                };
                let (mut t_prev, mut g_prev) = (lo, g(lo));
                for i in 1..=n {
                    let t = lo + (hi - lo) * i as f64 / n as f64;
                    let gi = g(t);
                    if (g_prev < 0.0 && gi > 0.0) || (g_prev > 0.0 && gi < 0.0) {
                        b.include_point(self.eval(bisect(g, t_prev, t)));
                    }
                    b.include_point(self.eval(t));
                    t_prev = t;
                    g_prev = gi;
                }
            }
        }
        b
    }

    /// Closest point to `p`: `(t, point)`.
    pub fn closest(&self, p: Point2) -> (f64, Point2) {
        let n = self.samples();
        let mut best = (0.0, f64::INFINITY);
        let mut cand: Vec<f64> = Vec::new();
        for (lo, hi) in self.spans() {
            let ts: Vec<f64> = (0..=n)
                .map(|i| lo + (hi - lo) * i as f64 / n as f64)
                .collect();
            let ds: Vec<f64> = ts
                .iter()
                .map(|&t| self.eval(t).distance_squared(p))
                .collect();
            for i in 0..ts.len() {
                let local_min = (i == 0 || ds[i] <= ds[i - 1]) && (i == n || ds[i] <= ds[i + 1]);
                if local_min {
                    // Refine inside the neighbouring samples.
                    let a = ts[i.saturating_sub(1)];
                    let b = ts[(i + 1).min(n)];
                    cand.push(self.refine_closest(p, ts[i], a, b));
                }
            }
        }
        cand.push(0.0);
        cand.push(self.span());
        for t in cand {
            let d = self.eval(t).distance_squared(p);
            if d < best.1 {
                best = (t, d);
            }
        }
        (best.0, self.eval(best.0))
    }

    fn refine_closest(&self, p: Point2, t0: f64, lo: f64, hi: f64) -> f64 {
        let mut t = t0;
        for _ in 0..30 {
            let (c, d1, d2) = self.derivs(t);
            let g = (c - p).dot(d1);
            let h = d1.length_squared() + (c - p).dot(d2);
            if h <= 0.0 {
                break;
            }
            let nt = (t - g / h).clamp(lo, hi);
            if (nt - t).abs() < 1e-15 * (1.0 + t.abs()) {
                t = nt;
                break;
            }
            t = nt;
        }
        t
    }

    /// Parameters `0 = t_0 < … < t_k = span` splitting the curve into pieces on which `y` is
    /// monotone (the roots of `y'` between the end points and at the knots).
    pub(crate) fn monotone_y(&self) -> Vec<f64> {
        let n = self.samples();
        let mut out = vec![0.0];
        for (lo, hi) in self.spans() {
            let g = |t: f64| self.derivative(t).y;
            let (mut t_prev, mut g_prev) = (lo, g(lo));
            for i in 1..=n {
                let t = lo + (hi - lo) * i as f64 / n as f64;
                let gi = g(t);
                if (g_prev < 0.0 && gi > 0.0) || (g_prev > 0.0 && gi < 0.0) {
                    out.push(bisect(g, t_prev, t));
                } else if gi.abs() <= 0.0 && i < n {
                    out.push(t);
                }
                t_prev = t;
                g_prev = gi;
            }
            out.push(hi);
        }
        out.sort_by(f64::total_cmp);
        out.dedup_by(|a, b| (*a - *b).abs() <= 1e-15);
        out
    }

    /// Signed area and first moment (about the sketch origin) of the region between the curve
    /// and the chord from its end back to its start: positive when the curve bulges to the
    /// right of the chord direction (outwards for a CCW loop).
    pub fn segment_area_moment(&self) -> (f64, Vec2) {
        let q0 = self.eval(0.0);
        let q1 = self.eval(self.span());
        let (mut area, mut mx, mut my) = (0.0, 0.0, 0.0);
        for (lo, hi) in self.spans() {
            let panels = 2 + self.degree;
            area += 0.5
                * integrate(
                    |t| {
                        let (c, d, _) = self.derivs(t);
                        (c - q0).perp_dot(d)
                    },
                    lo,
                    hi,
                    panels,
                );
            mx += 0.5
                * integrate(
                    |t| {
                        let (c, d, _) = self.derivs(t);
                        let r = c - q0;
                        r.x * r.x * d.y
                    },
                    lo,
                    hi,
                    panels,
                );
            my -= 0.5
                * integrate(
                    |t| {
                        let (c, d, _) = self.derivs(t);
                        let r = c - q0;
                        r.y * r.y * d.x
                    },
                    lo,
                    hi,
                    panels,
                );
        }
        // Chord back from `q1` to `q0` (relative to `q0`: from `d` to the origin).
        let d = q1 - q0;
        mx -= d.y * d.x * d.x / 6.0;
        my += d.x * d.y * d.y / 6.0;
        (area, Vec2::new(mx, my) + q0 * area)
    }

    /// Self-intersections `(t1, t2, point)` with `t1 < t2`, to within `tol` (mm). The curve is
    /// cut into twelve pieces per knot span and the pieces are intersected; a loop shorter than
    /// a twelfth of a span is not found.
    pub fn self_intersections(&self, tol: f64) -> Vec<(f64, f64, Point2)> {
        super::spline_x::self_intersections(self, tol)
    }

    /// Parameter of the point at arc length `s` from the start (`0..=length`), by bisection on
    /// the arc length function; used to space samples evenly.
    pub fn param_at_length(&self, s: f64) -> f64 {
        let total = self.length();
        if s <= 0.0 {
            return 0.0;
        }
        if s >= total {
            return self.span();
        }
        let len_to = |t: f64| self.trimmed(0.0, t).length();
        let (mut lo, mut hi) = (0.0, self.span());
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            if len_to(mid) < s {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        0.5 * (lo + hi)
    }
}

/// The cubic spline interpolating `points` at chord-length parameters, with the given end
/// tangent vectors (see [`FitMap`]; the handle points are `points[0] + start` and
/// `points[last] + end`) or natural ends.
pub fn fit_spline(
    points: &[Point2],
    start: Option<Vec2>,
    end: Option<Vec2>,
) -> Result<BSpline2, SketchError> {
    let params = chord_params(points)?;
    let map = FitMap::new(&params, start.is_some(), end.is_some())?;
    let mut inputs = points.to_vec();
    if let Some(t) = start {
        inputs.push(points[0] + t);
    }
    if let Some(t) = end {
        inputs.push(points[points.len() - 1] + t);
    }
    let poles = map.poles(&inputs);
    let n = poles.len();
    BSpline2::new(3, map.knots, poles, vec![1.0; n])
}

/// Weight of the conic apex pole for shape parameter `rho ∈ (0, 1)`: `ρ / (1 − ρ)`.
pub fn conic_weight(rho: f64) -> Result<f64, SketchError> {
    if rho.is_finite() && rho > 0.0 && rho < 1.0 {
        Ok(rho / (1.0 - rho))
    } else {
        Err(SketchError::InvalidValue {
            what: "conic rho (must be in (0, 1))",
            value: rho,
        })
    }
}

pub(crate) fn bisect(f: impl Fn(f64) -> f64, mut lo: f64, mut hi: f64) -> f64 {
    let flo = f(lo);
    for _ in 0..100 {
        let mid = 0.5 * (lo + hi);
        if (f(mid) < 0.0) == (flo < 0.0) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

// ---- End geometry (shared by the solver rows) ------------------------------------------------

/// First and second derivative **control points** of the start of a clamped B-spline in
/// homogeneous coordinates: returns `(C'_h(0), C''_h(0))` as linear combinations of the first
/// three homogeneous poles, `coef[k] = (c0, c1, c2)` for `k = 1, 2`.
///
/// `A'(0) = Q_0` with `Q_i = p (P_{i+1} − P_i)/(u_{i+p+1} − u_{i+1})`, and
/// `A''(0) = (p − 1)(Q_1 − Q_0)/(u_{p+1} − u_2)`. Degree 1 has no second derivative.
pub(crate) fn start_derivative_coefs(degree: usize, knots: &[f64]) -> ([f64; 3], [f64; 3]) {
    let p = degree as f64;
    let q0 = p / (knots[degree + 1] - knots[1]);
    let d1 = [-q0, q0, 0.0];
    if degree < 2 {
        return (d1, [0.0; 3]);
    }
    let q1 = p / (knots[degree + 2] - knots[2]);
    let f = (p - 1.0) / (knots[degree + 1] - knots[2]);
    (d1, [f * q0, -f * (q1 + q0), f * q1])
}

/// Signed curvature of the **start** of a rational clamped spline from its first three
/// homogeneous poles, `κ = (A'' − …)`, together with the gradient with respect to the nine
/// inputs `[x0 y0 w0 x1 y1 w1 x2 y2 w2]`. Zero for degree 1 or a vanishing speed.
pub(crate) fn start_curvature(degree: usize, knots: &[f64], v: &[f64; 9]) -> (f64, [f64; 9]) {
    if degree < 2 {
        return (0.0, [0.0; 9]);
    }
    let (c1, c2) = start_derivative_coefs(degree, knots);
    let var = |i: usize| Dual::var(v[i], i);
    // Homogeneous poles (w x, w y, w).
    let pw: Vec<[Dual; 3]> = (0..3)
        .map(|k| {
            let (x, y, w) = (var(3 * k), var(3 * k + 1), var(3 * k + 2));
            [w * x, w * y, w]
        })
        .collect();
    let comb = |c: &[f64; 3]| -> [Dual; 3] {
        let mut o = [Dual::cst(0.0); 3];
        for (k, pk) in pw.iter().enumerate() {
            for (c_i, o_i) in o.iter_mut().enumerate() {
                *o_i = *o_i + pk[c_i] * c[k];
            }
        }
        o
    };
    let (a1, a2) = (comb(&c1), comb(&c2));
    let w = pw[0][2];
    let c = [pw[0][0] / w, pw[0][1] / w];
    let d1 = [(a1[0] - a1[2] * c[0]) / w, (a1[1] - a1[2] * c[1]) / w];
    let d2 = [
        (a2[0] - d1[0] * a1[2] * 2.0 - a2[2] * c[0]) / w,
        (a2[1] - d1[1] * a1[2] * 2.0 - a2[2] * c[1]) / w,
    ];
    let cross = d1[0] * d2[1] - d1[1] * d2[0];
    let s2 = d1[0] * d1[0] + d1[1] * d1[1];
    if s2.v <= 0.0 {
        return (0.0, [0.0; 9]);
    }
    let k = cross / (s2 * s2.sqrt());
    (k.v, k.d)
}

/// Forward-mode dual number with nine partials (the end-curvature inputs).
#[derive(Copy, Clone)]
struct Dual {
    v: f64,
    d: [f64; 9],
}

impl Dual {
    fn cst(v: f64) -> Dual {
        Dual { v, d: [0.0; 9] }
    }
    fn var(v: f64, i: usize) -> Dual {
        let mut d = [0.0; 9];
        d[i] = 1.0;
        Dual { v, d }
    }
    fn sqrt(self) -> Dual {
        let s = self.v.sqrt();
        let k = 0.5 / s;
        Dual {
            v: s,
            d: self.d.map(|x| x * k),
        }
    }
}

impl std::ops::Add for Dual {
    type Output = Dual;
    fn add(self, o: Dual) -> Dual {
        let mut d = self.d;
        for (a, b) in d.iter_mut().zip(o.d) {
            *a += b;
        }
        Dual { v: self.v + o.v, d }
    }
}
impl std::ops::Sub for Dual {
    type Output = Dual;
    fn sub(self, o: Dual) -> Dual {
        let mut d = self.d;
        for (a, b) in d.iter_mut().zip(o.d) {
            *a -= b;
        }
        Dual { v: self.v - o.v, d }
    }
}
impl std::ops::Mul for Dual {
    type Output = Dual;
    fn mul(self, o: Dual) -> Dual {
        let mut d = [0.0; 9];
        for i in 0..9 {
            d[i] = self.d[i] * o.v + self.v * o.d[i];
        }
        Dual { v: self.v * o.v, d }
    }
}
impl std::ops::Mul<f64> for Dual {
    type Output = Dual;
    fn mul(self, o: f64) -> Dual {
        Dual {
            v: self.v * o,
            d: self.d.map(|x| x * o),
        }
    }
}
impl std::ops::Div for Dual {
    type Output = Dual;
    fn div(self, o: Dual) -> Dual {
        let inv = 1.0 / o.v;
        let mut d = [0.0; 9];
        for i in 0..9 {
            d[i] = (self.d[i] - self.v * inv * o.d[i]) * inv;
        }
        Dual { v: self.v * inv, d }
    }
}

// ---- Fit-point interpolation ---------------------------------------------------------------

/// Chord-length parameters of `points`, normalised to `[0, 1]`. `Err(Degenerate)` if two
/// consecutive points coincide.
pub fn chord_params(points: &[Point2]) -> Result<Vec<f64>, SketchError> {
    if points.len() < 2 {
        return Err(SketchError::Degenerate(
            "a fit-point spline needs at least two points",
        ));
    }
    let mut acc = vec![0.0];
    for w in points.windows(2) {
        let d = w[0].distance(w[1]);
        if d <= 1e-9 {
            return Err(SketchError::Degenerate(
                "two consecutive fit points coincide",
            ));
        }
        acc.push(acc[acc.len() - 1] + d);
    }
    let total = acc[acc.len() - 1];
    Ok(acc.into_iter().map(|a| a / total).collect())
}

/// The linear map from fit points (and tangent handles) to the poles of the interpolating
/// cubic B-spline (see the [module docs](self)).
#[derive(Clone, Debug, PartialEq)]
pub struct FitMap {
    /// Clamped knot vector (degree 3) on `[0, 1]`.
    pub knots: Vec<f64>,
    /// `poles[i] = Σ_j m[i][j] · input_j`, inputs = fit points, then the start handle (if any),
    /// then the end handle (if any).
    pub m: Vec<Vec<f64>>,
}

impl FitMap {
    /// Builds the map for fit parameters `params` (increasing, `params[0] = 0`, last `= 1`).
    pub fn new(
        params: &[f64],
        start_handle: bool,
        end_handle: bool,
    ) -> Result<FitMap, SketchError> {
        let np = params.len();
        if np < 2 {
            return Err(SketchError::Degenerate(
                "a fit-point spline needs at least two points",
            ));
        }
        if params.iter().any(|v| !v.is_finite()) || params.windows(2).any(|w| w[1] <= w[0]) {
            return Err(SketchError::NotApplicable(
                "fit parameters must be strictly increasing",
            ));
        }
        let n = np - 1;
        let p = 3;
        let nc = n + 3; // poles
        let mut knots = vec![0.0; 4];
        for j in 0..n.saturating_sub(1) {
            knots.push((params[j] + params[j + 1] + params[j + 2]) / 3.0);
        }
        knots.extend([1.0; 4]);
        debug_assert_eq!(knots.len(), nc + p + 1);
        let dummy = BSpline2 {
            degree: p,
            knots: knots.clone(),
            poles: vec![Point2::ZERO; nc],
            weights: vec![1.0; nc],
        };
        let ni = np + usize::from(start_handle) + usize::from(end_handle);
        let h0 = np;
        let h1 = np + usize::from(start_handle);
        let mut a = vec![vec![0.0; nc]; nc];
        let mut b = vec![vec![0.0; ni]; nc];
        // Row 0: P_0 = Q_0.
        a[0][0] = 1.0;
        b[0][0] = 1.0;
        // Row 1: start condition.
        let u4 = knots[4];
        if start_handle {
            // P_1 - P_0 = (u4 / 3) D_0, D_0 = (H - Q_0) / (ū_1 - ū_0)
            a[1][1] = 1.0;
            a[1][0] = -1.0;
            let k = u4 / 3.0 / (params[1] - params[0]);
            b[1][h0] += k;
            b[1][0] -= k;
        } else {
            // (P_2 - P_1)/u_5 = (P_1 - P_0)/u_4
            a[1][2] = 1.0 / knots[5];
            a[1][1] = -1.0 / knots[5] - 1.0 / u4;
            a[1][0] = 1.0 / u4;
        }
        // Interpolation rows.
        for k in 1..n {
            let span = dummy.find_span(params[k]);
            let ders = dummy.basis_derivs(span, params[k]);
            for j in 0..=p {
                a[1 + k][span - p + j] = ders[0][j];
            }
            b[1 + k][k] = 1.0;
        }
        // Row n + 1: end condition.
        let um = knots[knots.len() - 5]; // u_{m-4}
        let r = n + 1;
        if end_handle {
            // P_{n+2} - P_{n+1} = ((1 - u_{m-4}) / 3) D_n, D_n = (H - Q_n) / (ū_n - ū_{n-1})
            a[r][nc - 1] = 1.0;
            a[r][nc - 2] = -1.0;
            let k = (1.0 - um) / 3.0 / (params[n] - params[n - 1]);
            b[r][h1] += k;
            b[r][n] -= k;
        } else {
            // mirror of the natural start condition
            let u6 = knots[knots.len() - 6];
            let (c, d) = (1.0 - um, 1.0 - u6);
            a[r][nc - 3] = 1.0 / d;
            a[r][nc - 2] = -1.0 / d - 1.0 / c;
            a[r][nc - 1] = 1.0 / c;
        }
        // Last row: P_{n+2} = Q_n.
        a[nc - 1][nc - 1] = 1.0;
        b[nc - 1][n] = 1.0;
        // `r = n + 1` and `nc - 1 = n + 2` are distinct rows.
        let m = solve_dense(a, b).ok_or(SketchError::Degenerate(
            "the fit-point interpolation system is singular",
        ))?;
        Ok(FitMap { knots, m })
    }

    /// Poles for the given inputs (fit points, then handles).
    pub fn poles(&self, inputs: &[Point2]) -> Vec<Point2> {
        self.m
            .iter()
            .map(|row| {
                let mut s = Vec2::ZERO;
                for (c, p) in row.iter().zip(inputs) {
                    s += *p * *c;
                }
                s
            })
            .collect()
    }
}

/// Solves `A X = B` (dense, partial pivoting); `None` if singular.
fn solve_dense(mut a: Vec<Vec<f64>>, mut b: Vec<Vec<f64>>) -> Option<Vec<Vec<f64>>> {
    let n = a.len();
    for c in 0..n {
        let piv = (c..n).max_by(|&i, &j| a[i][c].abs().total_cmp(&a[j][c].abs()))?;
        if a[piv][c].abs() < 1e-14 {
            return None;
        }
        a.swap(c, piv);
        b.swap(c, piv);
        for r in c + 1..n {
            let f = a[r][c] / a[c][c];
            if f == 0.0 {
                continue;
            }
            for k in c..n {
                let v = a[c][k];
                a[r][k] -= f * v;
            }
            for k in 0..b[r].len() {
                let v = b[c][k];
                b[r][k] -= f * v;
            }
        }
    }
    for c in (0..n).rev() {
        for k in 0..b[c].len() {
            let mut s = b[c][k];
            for j in c + 1..n {
                s -= a[c][j] * b[j][k];
            }
            b[c][k] = s / a[c][c];
        }
    }
    Some(b)
}
