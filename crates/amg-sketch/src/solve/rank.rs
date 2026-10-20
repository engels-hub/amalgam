//! Numeric rank and DOF helpers (dense; for tests, builders and small sketches).
//!
//! SK-04 replaces this with rank-revealing QR and per-entity status; the tolerance rule is the
//! same (note §3.3).

use super::residual::build_system;
use crate::error::SketchError;
use crate::model::Sketch;

/// Numeric rank of the row-major `rows × cols` matrix `m`, by Gaussian elimination with full
/// pivoting. A pivot counts if it exceeds `τ = 1e-9 · max(rows, cols) · max|m_ij|`.
///
/// # Panics
/// If `m.len() < rows * cols`.
pub fn numeric_rank(m: &[f64], rows: usize, cols: usize) -> usize {
    let mut a = m[..rows * cols].to_vec();
    let max = a.iter().fold(0.0_f64, |acc, v| acc.max(v.abs()));
    if max == 0.0 || !max.is_finite() {
        return 0;
    }
    let tau = 1e-9 * rows.max(cols) as f64 * max;
    let mut rank = 0;
    let mut col_perm: Vec<usize> = (0..cols).collect();
    while rank < rows.min(cols) {
        // Largest remaining entry.
        let (mut pi, mut pj, mut pv) = (rank, rank, 0.0);
        for i in rank..rows {
            for j in rank..cols {
                let v = a[i * cols + col_perm[j]].abs();
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
        col_perm.swap(pj, rank);
        let pc = col_perm[rank];
        let piv = a[rank * cols + pc];
        for i in rank + 1..rows {
            let f = a[i * cols + pc] / piv;
            if f != 0.0 {
                for &c in &col_perm[rank..cols] {
                    a[i * cols + c] -= f * a[rank * cols + c];
                }
            }
        }
        rank += 1;
    }
    rank
}

/// Total degrees of freedom at the current configuration: free unknowns (entity params not
/// locked or fixed) minus the numeric rank of the Jacobian over them.
///
/// The rank is evaluated at the current values, so a degenerate configuration (zero-length
/// line, coincident centres) can report more DOF than the generic sketch has; SK-04's witness
/// tie-break handles that.
pub fn dof(sketch: &Sketch) -> Result<u32, SketchError> {
    let sys = build_system(sketch)?;
    let x = sketch.params().as_slice();
    let (m, n, f) = (sys.system.rows(), x.len(), sys.free.len());
    let mut j = vec![0.0; m * n];
    sys.system.jacobian_dense(x, &mut j);
    let mut jf = vec![0.0; m * f];
    for i in 0..m {
        for (k, p) in sys.free.iter().enumerate() {
            jf[i * f + k] = j[i * n + p.index()];
        }
    }
    let rank = numeric_rank(&jf, m, f);
    Ok(u32::try_from(f - rank).unwrap_or(u32::MAX))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ranks() {
        assert_eq!(numeric_rank(&[], 0, 0), 0);
        assert_eq!(numeric_rank(&[0.0; 6], 2, 3), 0);
        #[rustfmt::skip]
        let m = [
            1.0, 2.0, 3.0,
            2.0, 4.0, 6.0,
            0.0, 1.0, 1.0,
        ];
        assert_eq!(numeric_rank(&m, 3, 3), 2);
        assert_eq!(numeric_rank(&m, 1, 3), 1);
        let id = [1.0, 0.0, 0.0, 1e-3];
        assert_eq!(numeric_rank(&id, 2, 2), 2);
        assert_eq!(numeric_rank(&[1.0, 0.0, 0.0, 1e-13], 2, 2), 1);
    }
}
