//! Dense rank-revealing factorisations for DOF analysis (SK-04, note §3.3).
//!
//! - [`null_space`]: Householder QR with column pivoting of `Aᵀ` (i.e. row pivoting of `A`).
//!   The rank is the number of pivots above `τ = 1e-9 · max(m, n) · max|a_ij|` (the rule of
//!   [`numeric_rank`](crate::solve::numeric_rank)), and the trailing `n − rank` columns of the
//!   orthogonal factor are an orthonormal basis of `null(A)`.
//! - [`rank_abs`]: rank of a small matrix with an absolute pivot threshold (rows of an
//!   orthonormal null basis, per entity).
//!
//! Non-finite entries are treated as zero, so a degenerate row never poisons the rank.

/// Rank of the row-major `m × n` matrix `a` and, if `want_null`, an orthonormal basis of its
/// null space as a row-major `n × (n − rank)` matrix.
pub(crate) fn null_space(a: &[f64], m: usize, n: usize, want_null: bool) -> (usize, Vec<f64>) {
    // `t = Aᵀ`, column-major `n × m`: column `j` of `t` is row `j` of `a`, contiguous.
    let mut t: Vec<f64> = a[..m * n]
        .iter()
        .map(|v| if v.is_finite() { *v } else { 0.0 })
        .collect();
    let amax = t.iter().fold(0.0_f64, |acc, v| acc.max(v.abs()));
    if amax == 0.0 || m == 0 {
        return (0, if want_null { identity(n) } else { Vec::new() });
    }
    let tau = 1e-9 * m.max(n) as f64 * amax;
    let mut norms: Vec<f64> = (0..m)
        .map(|j| t[j * n..(j + 1) * n].iter().map(|v| v * v).sum())
        .collect();
    // Householder vectors `v_k` (stored in place below the diagonal, `v_k[k] = 1` implicit
    // via `beta`), applied as `H = I − beta v vᵀ`.
    let mut betas: Vec<f64> = Vec::new();
    let mut vs: Vec<Vec<f64>> = Vec::new();
    let mut rank = 0;
    for k in 0..m.min(n) {
        // Pivot: the remaining column with the largest norm (recomputed for robustness).
        let mut best = k;
        let mut bv = -1.0;
        for (j, nj) in norms.iter_mut().enumerate().skip(k) {
            *nj = t[j * n + k..(j + 1) * n].iter().map(|v| v * v).sum();
            if *nj > bv {
                bv = *nj;
                best = j;
            }
        }
        if bv.sqrt() <= tau {
            break;
        }
        if best != k {
            for i in 0..n {
                t.swap(k * n + i, best * n + i);
            }
            norms.swap(k, best);
        }
        // Reflector for t[k.., k].
        let col = &t[k * n + k..(k + 1) * n];
        let alpha = col.iter().map(|v| v * v).sum::<f64>().sqrt();
        let x0 = col[0];
        let a0 = if x0 >= 0.0 { -alpha } else { alpha };
        let mut v: Vec<f64> = col.to_vec();
        v[0] -= a0;
        let vn: f64 = v.iter().map(|e| e * e).sum();
        let beta = if vn > 0.0 { 2.0 / vn } else { 0.0 };
        // Apply to the remaining columns.
        for j in k..m {
            let c = &mut t[j * n + k..(j + 1) * n];
            let d: f64 = c.iter().zip(&v).map(|(a, b)| a * b).sum::<f64>() * beta;
            if d != 0.0 {
                for (ci, vi) in c.iter_mut().zip(&v) {
                    *ci -= d * vi;
                }
            }
        }
        betas.push(beta);
        vs.push(v);
        rank += 1;
    }
    if !want_null {
        return (rank, Vec::new());
    }
    // Null basis: Q[:, rank..n] = H_0 H_1 … H_{r−1} E with E the trailing identity columns.
    let d = n - rank;
    // Column-major n × d.
    let mut q = vec![0.0; n * d];
    for c in 0..d {
        q[c * n + rank + c] = 1.0;
    }
    for k in (0..rank).rev() {
        let (v, beta) = (&vs[k], betas[k]);
        for c in 0..d {
            let col = &mut q[c * n + k..(c + 1) * n];
            let dd: f64 = col.iter().zip(v).map(|(a, b)| a * b).sum::<f64>() * beta;
            if dd != 0.0 {
                for (ci, vi) in col.iter_mut().zip(v) {
                    *ci -= dd * vi;
                }
            }
        }
    }
    // Row-major n × d.
    let mut out = vec![0.0; n * d];
    for c in 0..d {
        for i in 0..n {
            out[i * d + c] = q[c * n + i];
        }
    }
    (rank, out)
}

fn identity(n: usize) -> Vec<f64> {
    let mut out = vec![0.0; n * n];
    for i in 0..n {
        out[i * n + i] = 1.0;
    }
    out
}

/// Rank of the row-major `rows × cols` matrix `m` by Gaussian elimination with full pivoting,
/// counting pivots above the absolute threshold `tau`.
pub(crate) fn rank_abs(m: &[f64], rows: usize, cols: usize, tau: f64) -> usize {
    let mut a = m[..rows * cols].to_vec();
    let mut rank = 0;
    let mut cp: Vec<usize> = (0..cols).collect();
    while rank < rows.min(cols) {
        let (mut pi, mut pj, mut pv) = (rank, rank, 0.0);
        for i in rank..rows {
            for j in rank..cols {
                let v = a[i * cols + cp[j]].abs();
                if v > pv {
                    (pi, pj, pv) = (i, j, v);
                }
            }
        }
        if pv <= tau {
            break;
        }
        if pi != rank {
            for j in 0..cols {
                a.swap(pi * cols + j, rank * cols + j);
            }
        }
        cp.swap(pj, rank);
        let pc = cp[rank];
        let piv = a[rank * cols + pc];
        for i in rank + 1..rows {
            let f = a[i * cols + pc] / piv;
            if f != 0.0 {
                for &c in &cp[rank..cols] {
                    a[i * cols + c] -= f * a[rank * cols + c];
                }
            }
        }
        rank += 1;
    }
    rank
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mul(a: &[f64], m: usize, n: usize, q: &[f64], d: usize) -> Vec<f64> {
        let mut out = vec![0.0; m * d];
        for i in 0..m {
            for c in 0..d {
                out[i * d + c] = (0..n).map(|k| a[i * n + k] * q[k * d + c]).sum();
            }
        }
        out
    }

    #[test]
    fn null_space_is_orthonormal_and_annihilated() {
        #[rustfmt::skip]
        let a = [
            1.0, 2.0, 3.0, 0.0,
            2.0, 4.0, 6.0, 0.0,
            0.0, 1.0, 1.0, 1.0,
        ];
        let (r, q) = null_space(&a, 3, 4, true);
        assert_eq!(r, 2);
        let d = 2;
        assert!(mul(&a, 3, 4, &q, d).iter().all(|v| v.abs() < 1e-12));
        for c1 in 0..d {
            for c2 in 0..d {
                let dot: f64 = (0..4).map(|i| q[i * d + c1] * q[i * d + c2]).sum();
                let want = if c1 == c2 { 1.0 } else { 0.0 };
                assert!((dot - want).abs() < 1e-12);
            }
        }
    }

    #[test]
    fn degenerate_inputs() {
        assert_eq!(null_space(&[], 0, 3, true), (0, identity(3)));
        assert_eq!(null_space(&[0.0, f64::NAN], 1, 2, true).0, 0);
        assert_eq!(null_space(&[1.0, 0.0, 0.0, 1e-13], 2, 2, false).0, 1);
        assert_eq!(rank_abs(&[1.0, 1.0, 1.0, 1.0], 2, 2, 1e-9), 1);
        assert_eq!(rank_abs(&[1e-12, 0.0], 1, 2, 1e-9), 0);
    }
}
