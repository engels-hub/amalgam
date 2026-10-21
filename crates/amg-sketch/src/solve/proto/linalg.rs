//! Small dense linear algebra for the prototype (row-major, `f64`).
//!
//! Hand-written so the prototype needs no new dependency; SK-03 moves to `nalgebra` / `faer`
//! (ADR-006, note §3.2).

/// In-place lower Cholesky factor of the symmetric positive *semi*-definite `m × m` matrix `a`
/// (row-major; only the lower triangle is read). A pivot `≤ rel · max(diag)` marks that row as
/// dependent: its column of `L` is zeroed and the solve gives it `z = 0`. Returns the
/// dependent flags.
///
/// For `A = J Jᵀ` this drops equations that are (numerically) linear combinations of earlier
/// rows, so a consistent redundant system still gets a minimum-norm Gauss–Newton step.
pub(crate) fn chol_semidef(a: &mut [f64], m: usize, rel: f64) -> Vec<bool> {
    let mut dep = vec![false; m];
    let dmax = (0..m).fold(0.0_f64, |acc, i| acc.max(a[i * m + i]));
    if dmax <= 0.0 || !dmax.is_finite() {
        dep.fill(true);
        return dep;
    }
    let tol = rel * dmax;
    for j in 0..m {
        let s = a[j * m + j] - (0..j).map(|k| a[j * m + k] * a[j * m + k]).sum::<f64>();
        if s <= tol || !s.is_finite() {
            dep[j] = true;
            a[j * m..=j * m + j].fill(0.0);
            for i in j + 1..m {
                a[i * m + j] = 0.0;
            }
            continue;
        }
        let d = s.sqrt();
        a[j * m + j] = d;
        for i in j + 1..m {
            let dot: f64 = (0..j).map(|k| a[i * m + k] * a[j * m + k]).sum();
            a[i * m + j] = (a[i * m + j] - dot) / d;
        }
    }
    // Clear the strict upper triangle so `a` is exactly L.
    for i in 0..m {
        a[i * m + i + 1..(i + 1) * m].fill(0.0);
    }
    dep
}

/// Solves `L Lᵀ z = b` with the factor from [`chol_semidef`]; dependent rows get `z = 0`.
pub(crate) fn chol_solve(l: &[f64], dep: &[bool], m: usize, b: &[f64]) -> Vec<f64> {
    let mut y = b[..m].to_vec();
    for i in 0..m {
        if dep[i] {
            y[i] = 0.0;
            continue;
        }
        let row = &l[i * m..i * m + i];
        let s: f64 = row.iter().zip(&y[..i]).map(|(a, b)| a * b).sum();
        y[i] = (y[i] - s) / l[i * m + i];
    }
    for i in (0..m).rev() {
        if dep[i] {
            y[i] = 0.0;
            continue;
        }
        let mut s = y[i];
        for k in i + 1..m {
            s -= l[k * m + i] * y[k];
        }
        y[i] = s / l[i * m + i];
    }
    y
}

/// `A = J Jᵀ + λ I` for a row-major `m × n` `j`.
pub(crate) fn gram(j: &[f64], m: usize, n: usize, lambda: f64) -> Vec<f64> {
    let mut a = vec![0.0; m * m];
    for r in 0..m {
        let jr = &j[r * n..(r + 1) * n];
        for c in 0..=r {
            let jc = &j[c * n..(c + 1) * n];
            let s: f64 = jr.iter().zip(jc).map(|(x, y)| x * y).sum();
            a[r * m + c] = s;
            a[c * m + r] = s;
        }
        a[r * m + r] += lambda;
    }
    a
}

/// `y = Jᵀ z` for a row-major `m × n` `j`.
pub(crate) fn jt_mul(j: &[f64], m: usize, n: usize, z: &[f64]) -> Vec<f64> {
    let mut y = vec![0.0; n];
    for r in 0..m {
        let zr = z[r];
        if zr != 0.0 {
            for (yk, jk) in y.iter_mut().zip(&j[r * n..(r + 1) * n]) {
                *yk += jk * zr;
            }
        }
    }
    y
}

/// `y = J v` for a row-major `m × n` `j`.
pub(crate) fn j_mul(j: &[f64], m: usize, n: usize, v: &[f64]) -> Vec<f64> {
    (0..m)
        .map(|r| {
            j[r * n..(r + 1) * n]
                .iter()
                .zip(v)
                .map(|(a, b)| a * b)
                .sum()
        })
        .collect()
}

pub(crate) fn norm2(v: &[f64]) -> f64 {
    v.iter().map(|x| x * x).sum()
}

pub(crate) fn max_abs(v: &[f64]) -> f64 {
    v.iter().fold(0.0_f64, |m, x| m.max(x.abs()))
}

#[cfg(test)]
#[allow(clippy::float_cmp, reason = "exact small-integer arithmetic")]
mod tests {
    use super::*;

    #[test]
    fn cholesky_solves_spd() {
        #[rustfmt::skip]
        let a0 = [
            4.0, 2.0, 0.0,
            2.0, 5.0, 1.0,
            0.0, 1.0, 3.0,
        ];
        let mut a = a0;
        let dep = chol_semidef(&mut a, 3, 1e-12);
        assert_eq!(dep, [false; 3]);
        let z = chol_solve(&a, &dep, 3, &[2.0, 8.0, 7.0]);
        for (r, want) in [2.0, 8.0, 7.0].iter().enumerate() {
            let got: f64 = (0..3).map(|c| a0[r * 3 + c] * z[c]).sum();
            assert!((got - want).abs() < 1e-12);
        }
    }

    #[test]
    fn cholesky_drops_dependent_rows() {
        // J = [[1, 0], [2, 0], [0, 1]]: row 1 = 2 · row 0.
        let j = [1.0, 0.0, 2.0, 0.0, 0.0, 1.0];
        let mut a = gram(&j, 3, 2, 0.0);
        let dep = chol_semidef(&mut a, 3, 1e-12);
        assert_eq!(dep, [false, true, false]);
        // Consistent right-hand side: J x = (1, 2, 3) has x = (1, 3).
        let z = chol_solve(&a, &dep, 3, &[1.0, 2.0, 3.0]);
        let x = jt_mul(&j, 3, 2, &z);
        assert!((x[0] - 1.0).abs() < 1e-12 && (x[1] - 3.0).abs() < 1e-12);
        assert_eq!(j_mul(&j, 3, 2, &x), vec![x[0], 2.0 * x[0], x[1]]);
    }
}
