//! B-spline basis functions (P&T A2.2, A2.3).
//!
//! The public functions take arrays sized for [`MAX_DEGREE`]. The evaluators use the
//! const-generic `*_n` versions with small stack buffers (`N` = 4, 8 or 26) so that low
//! degrees do not pay for zeroing 26 × 26 scratch arrays.

use crate::MAX_DEGREE;

/// A2.2: the `degree + 1` non-zero basis values `N_{span-p}, …, N_{span}` at `u`.
///
/// Writes nothing (beyond zeroing `out`) if the arguments are inconsistent:
/// `degree > MAX_DEGREE`, `out.len() <= degree`, or `span` not in `degree..knots.len()-degree-1`.
pub fn basis_funs(knots: &[f64], span: usize, u: f64, degree: usize, out: &mut [f64]) {
    out.iter_mut().for_each(|o| *o = 0.0);
    if degree > MAX_DEGREE || out.len() <= degree || !span_ok(knots, span, degree) {
        return;
    }
    let mut buf = [0.0; MAX_DEGREE + 1];
    basis_funs_n::<{ MAX_DEGREE + 1 }>(knots, span, u, degree, &mut buf);
    out[..=degree].copy_from_slice(&buf[..=degree]);
}

/// A2.3: basis derivatives up to order `n`; `out[k][j]` = k-th derivative of
/// `N_{span-p+j}`. Orders above `degree` are zero. Only `out[..=n]` is written (as far as
/// `out` reaches); inconsistent arguments give zeros.
pub fn ders_basis_funs(
    knots: &[f64],
    span: usize,
    u: f64,
    degree: usize,
    n: usize,
    out: &mut [[f64; MAX_DEGREE + 1]],
) {
    let rows = out.len().min(n + 1);
    for row in out.iter_mut().take(rows) {
        *row = [0.0; MAX_DEGREE + 1];
    }
    if degree > MAX_DEGREE || rows == 0 || !span_ok(knots, span, degree) {
        return;
    }
    ders_basis_funs_n::<{ MAX_DEGREE + 1 }>(knots, span, u, degree, rows - 1, &mut out[..rows]);
}

#[inline]
fn span_ok(knots: &[f64], span: usize, degree: usize) -> bool {
    span >= degree && span + degree < knots.len() && span + 1 < knots.len()
}

/// A2.2 into a stack buffer. Caller guarantees `degree < N` and a valid `span`.
#[inline(always)]
pub(crate) fn basis_funs_n<const N: usize>(
    knots: &[f64],
    span: usize,
    u: f64,
    degree: usize,
    out: &mut [f64; N],
) {
    let mut left = [0.0; N];
    let mut right = [0.0; N];
    out[0] = 1.0;
    for j in 1..=degree {
        left[j] = u - knots[span + 1 - j];
        right[j] = knots[span + j] - u;
        let mut saved = 0.0;
        for r in 0..j {
            let temp = out[r] / (right[r + 1] + left[j - r]);
            out[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        out[j] = saved;
    }
}

/// A2.3 into stack buffers. Caller guarantees `degree < N`, a valid `span` and
/// `out.len() == n + 1`.
#[inline(always)]
pub(crate) fn ders_basis_funs_n<const N: usize>(
    knots: &[f64],
    span: usize,
    u: f64,
    degree: usize,
    n: usize,
    out: &mut [[f64; N]],
) {
    let p = degree;
    let mut ndu = [[0.0; N]; N];
    let mut left = [0.0; N];
    let mut right = [0.0; N];
    ndu[0][0] = 1.0;
    for j in 1..=p {
        left[j] = u - knots[span + 1 - j];
        right[j] = knots[span + j] - u;
        let mut saved = 0.0;
        for r in 0..j {
            ndu[j][r] = right[r + 1] + left[j - r];
            let temp = ndu[r][j - 1] / ndu[j][r];
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }
    for j in 0..=p {
        out[0][j] = ndu[j][p];
    }
    let nn = n.min(p);
    for row in out.iter_mut().skip(1) {
        *row = [0.0; N];
    }
    if nn == 0 {
        return;
    }
    let mut a = [[0.0; N]; 2];
    let pi = p as isize;
    for r in 0..=pi {
        let (mut s1, mut s2) = (0usize, 1usize);
        a[0][0] = 1.0;
        for k in 1..=nn as isize {
            let mut d = 0.0;
            let rk = r - k;
            let pk = pi - k;
            if r >= k {
                a[s2][0] = a[s1][0] / ndu[(pk + 1) as usize][rk as usize];
                d = a[s2][0] * ndu[rk as usize][pk as usize];
            }
            let j1 = if rk >= -1 { 1 } else { -rk };
            let j2 = if r - 1 <= pk { k - 1 } else { pi - r };
            for j in j1..=j2 {
                let ju = j as usize;
                a[s2][ju] = (a[s1][ju] - a[s1][ju - 1]) / ndu[(pk + 1) as usize][(rk + j) as usize];
                d += a[s2][ju] * ndu[(rk + j) as usize][pk as usize];
            }
            if r <= pk {
                a[s2][k as usize] = -a[s1][(k - 1) as usize] / ndu[(pk + 1) as usize][r as usize];
                d += a[s2][k as usize] * ndu[r as usize][pk as usize];
            }
            out[k as usize][r as usize] = d;
            std::mem::swap(&mut s1, &mut s2);
        }
    }
    let mut f = pi as f64;
    for (k, row) in out.iter_mut().enumerate().take(nn + 1).skip(1) {
        for x in &mut row[..=p] {
            *x *= f;
        }
        f *= (p - k) as f64;
    }
}

/// Evaluates `$body` with the const `$n` bound to the smallest stack buffer size that holds
/// `$deg + 1` values (4, 8 or `MAX_DEGREE + 1`).
macro_rules! with_cap {
    ($deg:expr, $n:ident => $body:expr) => {{
        let d: usize = $deg;
        if d < 4 {
            const $n: usize = 4;
            $body
        } else if d < 8 {
            const $n: usize = 8;
            $body
        } else {
            const $n: usize = $crate::MAX_DEGREE + 1;
            $body
        }
    }};
}
pub(crate) use with_cap;

/// A2.3 with `n = 1` into `out0[..=degree]` (values) and `out1[..=degree]` (first
/// derivatives) through the smallest stack buffer. Caller guarantees `degree <= MAX_DEGREE`
/// and a valid `span`.
#[inline]
pub(crate) fn ders1_into(
    knots: &[f64],
    span: usize,
    u: f64,
    degree: usize,
    out0: &mut [f64; MAX_DEGREE + 1],
    out1: &mut [f64; MAX_DEGREE + 1],
) {
    with_cap!(degree, N => {
        let mut b = [[0.0; N]; 2];
        ders_basis_funs_n::<N>(knots, span, u, degree, 1, &mut b);
        out0[..=degree].copy_from_slice(&b[0][..=degree]);
        out1[..=degree].copy_from_slice(&b[1][..=degree]);
    })
}

#[cfg(test)]
#[allow(clippy::float_cmp, reason = "exact expected values")]
mod tests {
    use super::*;

    const EX23: [f64; 11] = [0., 0., 0., 1., 2., 3., 4., 4., 5., 5., 5.];

    /// P&T Ex. 2.3 (p = 2, u = 5/2, span 4): N_{2,2} = 1/8, N_{3,2} = 6/8, N_{4,2} = 1/8.
    #[test]
    fn example_2_3_basis_values() {
        let mut out = [0.0; 3];
        basis_funs(&EX23, 4, 2.5, 2, &mut out);
        assert!((out[0] - 1.0 / 8.0).abs() <= 1e-15, "{out:?}");
        assert!((out[1] - 6.0 / 8.0).abs() <= 1e-15, "{out:?}");
        assert!((out[2] - 1.0 / 8.0).abs() <= 1e-15, "{out:?}");
    }

    /// P&T Ex. 2.4 derivatives at u = 5/2: N'_{2,2} = -1/2, N'_{3,2} = 0, N'_{4,2} = 1/2,
    /// second derivatives 1, -2, 1.
    #[test]
    fn example_2_3_derivatives() {
        let mut out = [[0.0; MAX_DEGREE + 1]; 3];
        ders_basis_funs(&EX23, 4, 2.5, 2, 2, &mut out);
        let want = [[0.125, 0.75, 0.125], [-0.5, 0.0, 0.5], [1.0, -2.0, 1.0]];
        for k in 0..3 {
            for j in 0..3 {
                assert!(
                    (out[k][j] - want[k][j]).abs() <= 1e-15,
                    "k={k} j={j} {out:?}"
                );
            }
        }
    }

    #[test]
    fn inconsistent_args_do_not_panic() {
        let mut out = [1.0; 2];
        basis_funs(&EX23, 4, 2.5, 2, &mut out);
        assert_eq!(out, [0.0, 0.0]);
        let mut out = [0.0; 3];
        basis_funs(&EX23, 9, 2.5, 2, &mut out);
        basis_funs(&EX23, 0, 2.5, 2, &mut out);
        basis_funs(&[], 0, 2.5, 0, &mut out);
        let mut d = [[0.0; MAX_DEGREE + 1]; 1];
        ders_basis_funs(&EX23, 4, 2.5, 2, 5, &mut d);
        ders_basis_funs(&EX23, 4, 2.5, 40, 5, &mut d);
    }
}
