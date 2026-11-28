//! Arc length and its inverse (KH-13): adaptive 5-point Gauss–Legendre per knot span,
//! inverse by safeguarded Newton.

use crate::{CtrlPt, NurbsCurve};

/// 5-point Gauss–Legendre nodes and weights on [−1, 1].
const GL5: [(f64, f64); 5] = [
    (0.0, 0.568_888_888_888_888_9),
    (-0.538_469_310_105_683_1, 0.478_628_670_499_366_47),
    (0.538_469_310_105_683_1, 0.478_628_670_499_366_47),
    (-0.906_179_845_938_664, 0.236_926_885_056_189_08),
    (0.906_179_845_938_664, 0.236_926_885_056_189_08),
];

/// Recursion limit of the adaptive quadrature (per span).
const MAX_DEPTH: u32 = 40;

impl<P: CtrlPt> NurbsCurve<P> {
    #[inline]
    fn speed(&self, t: f64) -> f64 {
        self.derivs(t, 1)[1].length()
    }

    fn gl5(&self, a: f64, b: f64) -> f64 {
        let h = 0.5 * (b - a);
        let m = 0.5 * (a + b);
        GL5.iter()
            .map(|&(x, w)| w * self.speed(m + h * x))
            .sum::<f64>()
            * h
    }

    /// Adaptive GL5 on `[a, b]` given the one-panel estimate `whole`: halves until the two
    /// halves agree with `whole` within `tol` (absolute), then adds the Richardson term
    /// (GL5 is exact to order 10).
    fn adapt(&self, a: f64, b: f64, whole: f64, tol: f64, depth: u32) -> f64 {
        let m = 0.5 * (a + b);
        let l = self.gl5(a, m);
        let r = self.gl5(m, b);
        let diff = l + r - whole;
        if diff.abs() <= tol || depth >= MAX_DEPTH || !(a < m && m < b) {
            return l + r + diff / 1023.0;
        }
        self.adapt(a, m, l, 0.5 * tol, depth + 1) + self.adapt(m, b, r, 0.5 * tol, depth + 1)
    }

    /// Length of `[a, b]` inside one knot span, tolerance `tol` (absolute, mm).
    fn span_len(&self, a: f64, b: f64, tol: f64) -> f64 {
        if b <= a {
            return 0.0;
        }
        let whole = self.gl5(a, b);
        // Floor the tolerance at rounding level so the recursion terminates.
        let tol = tol.max(1e-15 * whole.abs());
        self.adapt(a, b, whole, tol, 0)
    }

    /// Distinct knot-span boundaries of `[t0, t1]` (both included).
    fn breaks(&self, t0: f64, t1: f64) -> Vec<f64> {
        let mut v = vec![t0];
        for &k in self.knots().as_slice() {
            if k > t0 && k < t1 && v.last().is_some_and(|&l| k > l) {
                v.push(k);
            }
        }
        v.push(t1);
        v
    }

    /// Arc length of `range` (default: the whole domain), clamped into the domain; the order
    /// of the range ends does not matter. `tol` is the absolute accuracy in mm (adaptive
    /// 5-point Gauss–Legendre per knot span, P&T-style span-wise integration of `|C'|`).
    pub fn arc_length(&self, range: Option<(f64, f64)>, tol: f64) -> f64 {
        let (a, b) = self.domain();
        let (t0, t1) = range.unwrap_or((a, b));
        let (t0, t1) = (t0.clamp(a, b), t1.clamp(a, b));
        let (t0, t1) = if t0 <= t1 { (t0, t1) } else { (t1, t0) };
        if t0.is_nan() || t1.is_nan() || t0 >= t1 {
            return 0.0;
        }
        let br = self.breaks(t0, t1);
        let tol = if tol.is_finite() && tol > 0.0 {
            tol
        } else {
            1e-12
        };
        let per = tol / (br.len() - 1) as f64;
        br.windows(2).map(|w| self.span_len(w[0], w[1], per)).sum()
    }

    /// Parameter at arc length `s` from the domain start (inverse of [`Self::arc_length`]):
    /// span lookup, then safeguarded Newton on `L(t) − s` with bisection fallback until
    /// `|L(t) − s| <= tol` (mm). `s` outside `[0, length]` gives the domain ends.
    pub fn param_at_length(&self, s: f64, tol: f64) -> f64 {
        let (a, b) = self.domain();
        if s.is_nan() || s <= 0.0 {
            return a;
        }
        let tol = if tol.is_finite() && tol > 0.0 {
            tol
        } else {
            1e-12
        };
        let br = self.breaks(a, b);
        let nspan = (br.len() - 1) as f64;
        let qtol = 0.1 * tol / nspan;
        let mut acc = 0.0;
        for w in br.windows(2) {
            let (lo0, hi0) = (w[0], w[1]);
            let len = self.span_len(lo0, hi0, qtol);
            if acc + len < s {
                acc += len;
                continue;
            }
            let target = s - acc;
            let (mut lo, mut hi) = (lo0, hi0);
            let mut t = lo0 + (hi0 - lo0) * (target / len).clamp(0.0, 1.0);
            for _ in 0..100 {
                let g = self.span_len(lo0, t, qtol) - target;
                if g.abs() <= tol {
                    return t;
                }
                if g > 0.0 {
                    hi = t;
                } else {
                    lo = t;
                }
                let sp = self.speed(t);
                let tn = if sp > 0.0 { t - g / sp } else { f64::NAN };
                t = if tn > lo && tn < hi {
                    tn
                } else {
                    0.5 * (lo + hi)
                };
                if lo >= hi || hi - lo <= f64::EPSILON * hi.abs().max(lo.abs()) {
                    return t;
                }
            }
            return t;
        }
        b
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp, reason = "exact domain ends")]
mod tests {
    use crate::analytic;
    use amg_core::{Plane, Point3};

    #[test]
    fn line_length_and_inverse() {
        let l = analytic::line(Point3::ZERO, Point3::new(3.0, 4.0, 0.0));
        assert!((l.arc_length(None, 1e-14) - 5.0).abs() < 1e-14);
        assert!((l.param_at_length(2.5, 1e-14) - 0.5).abs() < 1e-14);
        assert_eq!(l.param_at_length(-1.0, 1e-12), 0.0);
        assert_eq!(l.param_at_length(9.0, 1e-12), 1.0);
        assert!((l.arc_length(Some((0.75, 0.25)), 1e-14) - 2.5).abs() < 1e-14);
    }

    #[test]
    fn circle_quadrant() {
        let c = analytic::circle_arc(&Plane::XY, 2.0, 0.0, std::f64::consts::FRAC_PI_2).unwrap();
        let want = std::f64::consts::PI;
        let got = c.arc_length(None, 1e-13);
        assert!(((got - want) / want).abs() < 1e-12, "{got}");
    }
}
