//! Curve fitting (KH-13, P&T ch. 9): parametrisation, interpolation and approximation knots,
//! global interpolation (A9.1 with end-derivative rows, closed), constrained least squares
//! (§9.4.1 / A9.6), approximation to a tolerance (adaptive knots + Hoschek parameter
//! correction) and re-approximation onto a given knot vector.
//!
//! All results are non-rational curves. Least squares is affine-invariant: planar data gives a
//! planar fit up to rounding.

mod adaptive;
mod interp;
mod lsq;

#[doc(hidden)]
pub use adaptive::approximate_to_tolerance_uncorrected;
pub use adaptive::{approximate_onto_knots, approximate_to_tolerance};
pub use interp::interpolate;
pub use lsq::approximate_lsq;

use crate::{CtrlPt, KnotVector, MAX_DEGREE, NurbsCurve};

/// How data points are mapped to curve parameters.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum Parametrization {
    /// Equally spaced parameters.
    Uniform,
    /// Proportional to chord length (P&T eq. 9.4–9.5).
    ChordLength,
    /// Proportional to the square root of chord length (P&T eq. 9.6, Lee 1989).
    #[default]
    Centripetal,
}

/// Parameters in `[0, 1]` for `pts` (first `0`, last exactly `1`).
///
/// Uniform spacing is used when the total (chord or centripetal) length is zero or not finite.
/// Coincident consecutive points get equal parameters (interpolation rejects them). An empty
/// slice gives an empty vector, a single point `[0]`.
pub fn parametrize<P: CtrlPt>(pts: &[P], m: Parametrization) -> Vec<f64> {
    let n = pts.len();
    if n <= 1 {
        return vec![0.0; n];
    }
    let uniform = || {
        (0..n)
            .map(|i| i as f64 / (n - 1) as f64)
            .collect::<Vec<_>>()
    };
    let d: Vec<f64> = match m {
        Parametrization::Uniform => return finish(uniform()),
        Parametrization::ChordLength => pts.windows(2).map(|w| (w[1] - w[0]).length()).collect(),
        Parametrization::Centripetal => pts
            .windows(2)
            .map(|w| (w[1] - w[0]).length().sqrt())
            .collect(),
    };
    let total: f64 = d.iter().sum();
    if !(total.is_finite() && total > 0.0) {
        return finish(uniform());
    }
    let mut u = Vec::with_capacity(n);
    u.push(0.0);
    let mut acc = 0.0;
    for di in &d[..n - 2] {
        acc += di;
        u.push((acc / total).min(1.0));
    }
    u.push(1.0);
    finish(u)
}

/// Makes `u` non-decreasing with exact ends `0` / `1`.
fn finish(mut u: Vec<f64>) -> Vec<f64> {
    let n = u.len();
    u[0] = 0.0;
    u[n - 1] = 1.0;
    for i in 1..n {
        if u[i] < u[i - 1] {
            u[i] = u[i - 1];
        }
    }
    u
}

/// True if `params` are finite and strictly increasing.
pub(crate) fn strictly_increasing(params: &[f64]) -> bool {
    params.iter().all(|t| t.is_finite()) && params.windows(2).all(|w| w[0] < w[1])
}

/// Bézier knots of `degree` (clamped into `1..=MAX_DEGREE`) on `(a, b)` if that range is
/// valid, else on `[0, 1]`. Fallback of the knot builders for unusable input.
fn fallback_knots(params: &[f64], degree: usize) -> KnotVector {
    let p = degree.clamp(1, MAX_DEGREE);
    let k = KnotVector::bezier(p);
    match (params.first(), params.last()) {
        (Some(&a), Some(&b)) if a.is_finite() && b.is_finite() && a < b => k.reparametrized((a, b)),
        _ => k,
    }
}

/// Averaging knots (P&T eq. 9.8) for interpolating `params.len()` points with `degree`:
/// `u_{j+p} = (1/p) Σ_{i=j}^{j+p−1} ū_i`, `j = 1 … n − p`, clamped at `ū_0` / `ū_m`.
///
/// Repeated values in `params` (as the end-derivative variants of §9.2.2 use them) are
/// allowed as long as no knot gets a multiplicity above the degree. For unusable input
/// (degree outside `1..=MAX_DEGREE`, fewer than `degree + 1` parameters, non-finite or
/// decreasing values, too-high multiplicities) the Bézier knot vector on `[ū_0, ū_m]` (or
/// `[0, 1]`) is returned.
pub fn averaging_knots(params: &[f64], degree: usize) -> KnotVector {
    let p = degree;
    let m1 = params.len();
    if p == 0 || p > MAX_DEGREE || m1 < p + 1 || !non_decreasing(params) {
        return fallback_knots(params, degree);
    }
    let n = m1 - 1;
    let (a, b) = (params[0], params[n]);
    let mut v = Vec::with_capacity(n + p + 2);
    v.extend(std::iter::repeat_n(a, p + 1));
    let inv = 1.0 / p as f64;
    for j in 1..=n - p {
        let s: f64 = params[j..j + p].iter().sum();
        v.push((s * inv).clamp(a, b));
    }
    v.extend(std::iter::repeat_n(b, p + 1));
    knots_or_fallback(v, p, params)
}

/// Approximation knots (P&T eq. 9.68–9.69) for `n_poles` poles fitted to `params.len()`
/// data points: `d = (m + 1)/(n − p + 1)`, `u_{p+j} = (1 − α) ū_{i−1} + α ū_i` with
/// `i = ⌊j d⌋`, `α = j d − i`. Every knot span then contains data, which keeps the
/// least-squares system regular.
///
/// `n_poles` is clamped into `degree + 1 ..= params.len()`; unusable input gives the Bézier
/// knot vector (see [`averaging_knots`]).
pub fn approximation_knots(params: &[f64], degree: usize, n_poles: usize) -> KnotVector {
    let p = degree;
    let m1 = params.len();
    if p == 0 || p > MAX_DEGREE || m1 < p + 1 || !non_decreasing(params) {
        return fallback_knots(params, degree);
    }
    let m = m1 - 1;
    let n = n_poles.clamp(p + 1, m1) - 1;
    let (a, b) = (params[0], params[m]);
    let d = (m + 1) as f64 / (n - p + 1) as f64;
    let mut v = Vec::with_capacity(n + p + 2);
    v.extend(std::iter::repeat_n(a, p + 1));
    for j in 1..=n - p {
        let jd = j as f64 * d;
        let i = (jd.floor() as usize).clamp(1, m);
        let alpha = jd - i as f64;
        v.push(((1.0 - alpha) * params[i - 1] + alpha * params[i]).clamp(a, b));
    }
    v.extend(std::iter::repeat_n(b, p + 1));
    knots_or_fallback(v, p, params)
}

fn non_decreasing(params: &[f64]) -> bool {
    params.iter().all(|t| t.is_finite())
        && params.windows(2).all(|w| w[0] <= w[1])
        && params[0] < params[params.len() - 1]
}

fn knots_or_fallback(mut v: Vec<f64>, p: usize, params: &[f64]) -> KnotVector {
    for i in 1..v.len() {
        if v[i] < v[i - 1] {
            v[i] = v[i - 1];
        }
    }
    let Ok(k) = KnotVector::new(v) else {
        return fallback_knots(params, p);
    };
    let n = k.len() - p - 1;
    if crate::curve::check_knots(p, &k, n).is_err() {
        return fallback_knots(params, p);
    }
    k
}

/// Optional end-derivative constraints (first and second derivative at each end), in model
/// units per unit of the curve parameter.
#[derive(Clone, Debug, PartialEq)]
pub struct EndDerivs<P> {
    /// `C'(t_min)`.
    pub start: Option<P>,
    /// `C'(t_max)`.
    pub end: Option<P>,
    /// `C''(t_min)`.
    pub start2: Option<P>,
    /// `C''(t_max)`.
    pub end2: Option<P>,
}

impl<P> Default for EndDerivs<P> {
    fn default() -> Self {
        EndDerivs {
            start: None,
            end: None,
            start2: None,
            end2: None,
        }
    }
}

impl<P: Copy> EndDerivs<P> {
    /// `[_, C', C'']` at the start (index = derivative order; order 0 is filled by the caller).
    pub(crate) fn start_orders(&self) -> [Option<P>; 3] {
        [None, self.start, self.start2]
    }

    /// `[_, C', C'']` at the end.
    pub(crate) fn end_orders(&self) -> [Option<P>; 3] {
        [None, self.end, self.end2]
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.start.is_none() && self.end.is_none() && self.start2.is_none() && self.end2.is_none()
    }
}

/// Options of [`interpolate`].
#[derive(Clone, Debug)]
pub struct InterpOpts<P> {
    /// Degree, default 3. Must be at least (number of derivative constraints at an end) + 1.
    pub degree: usize,
    /// Parametrisation of the data, ignored if `params` is `Some`.
    pub parametrization: Parametrization,
    /// Explicit, strictly increasing parameters (one per point; closed: one more, the closing
    /// parameter, unless the last point repeats the first).
    pub params: Option<Vec<f64>>,
    /// End-derivative constraints (open curves only).
    pub ends: EndDerivs<P>,
    /// Periodic interpolation through the points and back to the first, returned in clamped
    /// form (C^{p−1} at the seam).
    pub closed: bool,
}

impl<P> Default for InterpOpts<P> {
    fn default() -> Self {
        InterpOpts {
            degree: 3,
            parametrization: Parametrization::Centripetal,
            params: None,
            ends: EndDerivs::default(),
            closed: false,
        }
    }
}

/// Options of [`approximate_lsq`].
#[derive(Clone, Debug)]
pub struct LsqOpts<P> {
    /// Degree.
    pub degree: usize,
    /// Clamped knot vector of the result (defines the pole count).
    pub knots: KnotVector,
    /// One parameter per data point, inside the knot domain.
    pub params: Vec<f64>,
    /// Interpolate the first and last point exactly.
    pub fix_ends: bool,
    /// Optional end-derivative constraints (satisfied exactly).
    pub ends: EndDerivs<P>,
    /// Optional positive weight per data point (default all 1).
    pub data_weights: Option<Vec<f64>>,
}

/// Result of an approximation.
#[derive(Clone, Debug)]
pub struct Fit<P: CtrlPt> {
    /// The fitted (non-rational) curve.
    pub curve: NurbsCurve<P>,
    /// Largest data deviation, mm (see the fitting functions for what is measured).
    pub max_error: f64,
    /// Root mean square of the data deviations, mm.
    pub rms_error: f64,
    /// Parameter of each data point on `curve`.
    pub params: Vec<f64>,
}

/// Options of [`approximate_onto_knots`].
#[derive(Clone, Debug)]
pub struct OntoOpts<P> {
    /// Required maximum deviation from the source curve, mm (default 1e-5).
    pub tol: f64,
    /// Constrain the end tangents to the source curve's (scaled to the target domain).
    pub fix_end_tangents: bool,
    /// Explicit end-derivative constraints (override `fix_end_tangents` where given).
    pub ends: EndDerivs<P>,
    /// Hoschek parameter-correction passes after the first solve (default 3).
    pub corrections: u8,
}

impl<P> Default for OntoOpts<P> {
    fn default() -> Self {
        OntoOpts {
            tol: 1e-5,
            fix_end_tangents: false,
            ends: EndDerivs::default(),
            corrections: 3,
        }
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp, reason = "exact expected values")]
mod tests {
    use super::*;
    use amg_core::{Point2, Point3};

    /// P&T Ex. 9.1: chord-length parameters {0, 5/17, 9/17, 14/17, 1}, knots
    /// {0, 0, 0, 0, 28/51, 1, 1, 1, 1}.
    #[test]
    fn example_9_1_params_and_knots() {
        let q = [
            Point2::new(0., 0.),
            Point2::new(3., 4.),
            Point2::new(-1., 4.),
            Point2::new(-4., 0.),
            Point2::new(-4., -3.),
        ];
        let u = parametrize(&q, Parametrization::ChordLength);
        let want = [0.0, 5.0 / 17.0, 9.0 / 17.0, 14.0 / 17.0, 1.0];
        for (a, b) in u.iter().zip(want) {
            assert!((a - b).abs() <= 1e-15, "{u:?}");
        }
        let k = averaging_knots(&u, 3);
        let want = [0., 0., 0., 0., 28.0 / 51.0, 1., 1., 1., 1.];
        assert_eq!(k.len(), want.len());
        for (a, b) in k.as_slice().iter().zip(want) {
            assert!((a - b).abs() <= 1e-15, "{k:?}");
        }
    }

    #[test]
    fn parametrize_edge_cases() {
        assert!(parametrize::<Point3>(&[], Parametrization::Centripetal).is_empty());
        assert_eq!(parametrize(&[Point3::ONE], Parametrization::Uniform), [0.0]);
        // All coincident: uniform fallback.
        let u = parametrize(&[Point3::ONE; 3], Parametrization::ChordLength);
        assert_eq!(u, [0.0, 0.5, 1.0]);
        let u = parametrize(
            &[Point3::ZERO, Point3::X, Point3::X * 5.0],
            Parametrization::Centripetal,
        );
        assert!((u[1] - 1.0 / 3.0).abs() < 1e-15, "{u:?}");
        let u = parametrize(
            &[Point3::ZERO, Point3::X, Point3::X * 5.0],
            Parametrization::Uniform,
        );
        assert_eq!(u, [0.0, 0.5, 1.0]);
    }

    #[test]
    fn approximation_knots_have_data_in_every_span() {
        let u: Vec<f64> = (0..101).map(|i| (i as f64 / 100.0).powi(2)).collect();
        for n in [4, 5, 10, 37, 101, 500] {
            let k = approximation_knots(&u, 3, n);
            let nn = n.clamp(4, 101);
            assert_eq!(k.len(), nn + 4);
            let ks = k.as_slice();
            for s in 3..nn {
                if ks[s] < ks[s + 1] {
                    assert!(u.iter().any(|&t| t >= ks[s] && t <= ks[s + 1]));
                }
            }
        }
        // Fallbacks.
        assert_eq!(
            approximation_knots(&[0.0, 1.0], 3, 4),
            KnotVector::bezier(3)
        );
        assert_eq!(
            averaging_knots(&[0.0, f64::NAN, 1.0], 1),
            KnotVector::bezier(1)
        );
        assert_eq!(
            averaging_knots(&[2.0, 4.0], 3),
            KnotVector::bezier(3).reparametrized((2.0, 4.0))
        );
    }
}
