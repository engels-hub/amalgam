//! Exact plane-side classification: the sign of `n · (p − o)` for f64 `n`, `p`, `o`.
//!
//! The plane is the one defined by the f64 coefficients it is given, so the sign of every
//! vertex is decided exactly: a floating-point filter first, Shewchuk-style expansion
//! arithmetic when the filter cannot decide. A vertex on the plane is exactly on it, however
//! the cut is rotated or translated.

use amg_core::{Point3, Vec3};

/// Rounding unit of f64 (2^-53).
const U: f64 = 1.110_223_024_625_156_5e-16;
/// Filter bound factor on `Σ |n_i (p_i − o_i)|`: covers the rounding of the subtractions,
/// the products and the two additions with margin.
const FILTER: f64 = 12.0 * U;

#[inline]
fn two_sum(a: f64, b: f64) -> (f64, f64) {
    let s = a + b;
    let bb = s - a;
    (s, (a - (s - bb)) + (b - bb))
}

#[inline]
fn two_product(a: f64, b: f64) -> (f64, f64) {
    let p = a * b;
    (p, a.mul_add(b, -p))
}

/// Sign of the exact sum of `terms` (grow-expansion with zero elimination).
fn exact_sum_sign(terms: &[f64]) -> i8 {
    let mut e: Vec<f64> = Vec::with_capacity(terms.len() + 1);
    for &t in terms {
        let mut q = t;
        let mut next: Vec<f64> = Vec::with_capacity(e.len() + 1);
        for &x in &e {
            let (s, h) = two_sum(q, x);
            q = s;
            if h != 0.0 {
                next.push(h);
            }
        }
        if q != 0.0 {
            next.push(q);
        }
        e = next;
    }
    match e.last() {
        Some(&top) if top > 0.0 => 1,
        Some(&top) if top < 0.0 => -1,
        _ => 0,
    }
}

/// A plane for exact classification.
#[derive(Clone, Copy, Debug)]
pub(crate) struct PlaneSide {
    o: Point3,
    n: Vec3,
    /// Vertices with `|distance| <= tol` count as on the plane (`0` = exact).
    tol: f64,
}

impl PlaneSide {
    pub(crate) fn new(o: Point3, n: Vec3, tol: f64) -> Self {
        Self { o, n, tol }
    }

    /// Exact sign of `n · (p − o)` (`-1`, `0`, `1`).
    pub(crate) fn exact_sign(&self, p: Point3) -> i8 {
        let (n, o) = (self.n, self.o);
        let d = p - o;
        let terms = [n.x * d.x, n.y * d.y, n.z * d.z];
        let approx = terms[0] + terms[1] + terms[2];
        let bound = FILTER * (terms[0].abs() + terms[1].abs() + terms[2].abs());
        if approx > bound {
            return 1;
        }
        if approx < -bound {
            return -1;
        }
        let mut parts = [0.0f64; 12];
        for (k, (pc, oc, nc)) in [(p.x, o.x, n.x), (p.y, o.y, n.y), (p.z, o.z, n.z)]
            .into_iter()
            .enumerate()
        {
            // (p − o) = hi + lo exactly; n·hi and n·lo are two exact products each.
            let (hi, lo) = two_sum(pc, -oc);
            let (a, b) = two_product(nc, hi);
            let (c, d) = two_product(nc, lo);
            parts[4 * k..4 * k + 4].copy_from_slice(&[a, b, c, d]);
        }
        exact_sum_sign(&parts)
    }

    /// Side of `p` and an approximate signed distance whose sign matches the side (never
    /// zero for a non-zero side, so interpolation weights stay finite).
    pub(crate) fn side(&self, p: Point3) -> (i8, f64) {
        let approx = self.n.dot(p - self.o);
        if self.tol > 0.0 {
            if approx.abs() <= self.tol {
                return (0, approx);
            }
            return (if approx > 0.0 { 1 } else { -1 }, approx);
        }
        let s = self.exact_sign(p);
        let d = match s {
            0 => 0.0,
            1 => approx.max(f64::MIN_POSITIVE),
            _ => approx.min(-f64::MIN_POSITIVE),
        };
        (s, d)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_sign_on_the_plane_despite_rounding() {
        // Far-apart coordinates that cancel only in exact arithmetic.
        let ps = PlaneSide::new(Point3::ZERO, Vec3::new(0.0, 0.0, 1.0), 0.0);
        assert_eq!(ps.exact_sign(Point3::new(1e30, -1e30, 0.0)), 0);
        assert_eq!(ps.exact_sign(Point3::new(0.0, 0.0, 1e-300)), 1);
        assert_eq!(ps.exact_sign(Point3::new(0.0, 0.0, -1e-300)), -1);
        // On the plane although the products round; one ulp off is detected either way.
        let n = Vec3::new(0.1, 0.1, 0.1);
        let ps = PlaneSide::new(Point3::new(0.1, 0.2, 0.3), n, 0.0);
        let on = Point3::new(0.1, 0.2, 0.3);
        assert_eq!(ps.exact_sign(on), 0);
        let up = Point3::new(f64::from_bits(on.x.to_bits() + 1), on.y, on.z);
        assert_eq!(ps.exact_sign(up), 1);
        let down = Point3::new(f64::from_bits(on.x.to_bits() - 1), on.y, on.z);
        assert_eq!(ps.exact_sign(down), -1);
    }

    #[test]
    fn expansion_sum_cancels_exactly() {
        assert_eq!(exact_sum_sign(&[1e100, 1.0, -1e100]), 1);
        assert_eq!(exact_sum_sign(&[1e100, -1.0, -1e100]), -1);
        assert_eq!(exact_sum_sign(&[0.1, 0.2, -0.1, -0.2]), 0);
    }

    #[test]
    fn tolerance_snaps_to_zero() {
        let ps = PlaneSide::new(Point3::ZERO, Vec3::Z, 1e-6);
        assert_eq!(ps.side(Point3::new(0.0, 0.0, 5e-7)).0, 0);
        assert_eq!(ps.side(Point3::new(0.0, 0.0, 2e-6)).0, 1);
        assert_eq!(ps.side(Point3::new(0.0, 0.0, -2e-6)).0, -1);
    }
}
