//! Sparse step solve for large blocks: `(Js Jsᵀ + λI) z = F` with `faer`'s sparse Cholesky.
//!
//! The Gram pattern, the list of products that fill it and the symbolic factorisation (AMD
//! ordering) are built once per block and cached with the structure; every iteration only
//! fills values and runs the numeric factorisation. Pivots below `rel · max(diag)` (dependent
//! rows of a redundant block) are replaced by a huge value, which drops that row from the step
//! like the dense semidefinite Cholesky does. Everything runs with `Par::Seq`, so results are
//! deterministic.

use faer::dyn_stack::{MemBuffer, MemStack, StackReq};
use faer::linalg::cholesky::llt::factor::LltRegularization;
use faer::sparse::linalg::cholesky::{
    CholeskySymbolicParams, LltRef, SymbolicCholesky, SymmetricOrdering,
    factorize_symbolic_cholesky,
};
use faer::sparse::{SparseColMatRef, SymbolicSparseColMat};
use faer::{Conj, MatMut, Par, Side};

/// Cached sparse structure of one block's Gram matrix `J Jᵀ` (lower triangle, CSC).
pub(crate) struct SparseGram {
    m: usize,
    pattern: SymbolicSparseColMat<usize>,
    /// Value index of each diagonal entry.
    diag: Vec<usize>,
    /// `(gram value, csr a, csr b)`: `G[g] += J[a] · J[b]`.
    contrib: Vec<[u32; 3]>,
    symbolic: SymbolicCholesky<usize>,
    scratch: StackReq,
}

impl std::fmt::Debug for SparseGram {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SparseGram")
            .field("m", &self.m)
            .field("nnz", &self.diag.len())
            .field("contrib", &self.contrib.len())
            .finish_non_exhaustive()
    }
}

impl SparseGram {
    /// Builds the Gram structure of the CSR pattern (`row_ptr`, `col_idx`) of an `m`-row block
    /// with `n` columns. `None` if `faer` fails (out of memory).
    pub(crate) fn new(m: usize, n: usize, row_ptr: &[u32], col_idx: &[u32]) -> Option<Self> {
        // Rows of each column, with their CSR index.
        let mut by_col: Vec<Vec<(u32, u32)>> = vec![Vec::new(); n];
        for r in 0..m {
            for k in row_ptr[r]..row_ptr[r + 1] {
                by_col[col_idx[k as usize] as usize].push((r as u32, k));
            }
        }
        // Lower-triangle entries `(col, row)` with `row ≥ col`; every diagonal is present.
        let mut keys: Vec<(u32, u32)> = (0..m as u32).map(|i| (i, i)).collect();
        for list in &by_col {
            for (ia, &(ra, _)) in list.iter().enumerate() {
                for &(rb, _) in &list[..ia] {
                    let (lo, hi) = if ra < rb { (ra, rb) } else { (rb, ra) };
                    keys.push((lo, hi));
                }
            }
        }
        keys.sort_unstable();
        keys.dedup();
        let mut col_ptr = vec![0usize; m + 1];
        for &(c, _) in &keys {
            col_ptr[c as usize + 1] += 1;
        }
        for i in 0..m {
            col_ptr[i + 1] += col_ptr[i];
        }
        let row_idx: Vec<usize> = keys.iter().map(|&(_, r)| r as usize).collect();
        let find = |c: u32, r: u32| -> u32 {
            let (a, b) = (col_ptr[c as usize], col_ptr[c as usize + 1]);
            let off = row_idx[a..b].partition_point(|&x| x < r as usize);
            (a + off) as u32
        };
        let diag: Vec<usize> = (0..m as u32).map(|i| find(i, i) as usize).collect();
        let mut contrib = Vec::new();
        for list in &by_col {
            for (ia, &(ra, ka)) in list.iter().enumerate() {
                for &(rb, kb) in &list[..=ia] {
                    let (lo, hi) = if ra < rb { (ra, rb) } else { (rb, ra) };
                    contrib.push([find(lo, hi), ka, kb]);
                }
            }
        }
        // Sorted by target so the accumulation order is fixed and cache friendly.
        contrib.sort_unstable();
        let pattern = SymbolicSparseColMat::new_checked(m, m, col_ptr, None, row_idx);
        let symbolic = factorize_symbolic_cholesky(
            pattern.as_ref(),
            Side::Lower,
            SymmetricOrdering::Amd,
            CholeskySymbolicParams::default(),
        )
        .ok()?;
        let scratch = StackReq::any_of(&[
            symbolic.factorize_numeric_llt_scratch::<f64>(Par::Seq, Default::default()),
            symbolic.solve_in_place_scratch::<f64>(1, Par::Seq),
        ]);
        Some(SparseGram {
            m,
            pattern,
            diag,
            contrib,
            symbolic,
            scratch,
        })
    }

    /// Solves `(Js Jsᵀ + λI) z = f` for the CSR values `js` (scaled Jacobian). Rows whose
    /// pivot is `≤ rel · max(diag)` are dropped (`z ≈ 0`). `None` if the factorisation fails.
    pub(crate) fn solve(&self, js: &[f64], lambda: f64, rel: f64, f: &[f64]) -> Option<Vec<f64>> {
        let mut g = vec![0.0; self.pattern.compute_nnz()];
        for &[t, a, b] in &self.contrib {
            g[t as usize] += js[a as usize] * js[b as usize];
        }
        let mut dmax = 0.0_f64;
        for &d in &self.diag {
            g[d] += lambda;
            dmax = dmax.max(g[d]);
        }
        if dmax <= 0.0 || !dmax.is_finite() {
            return Some(vec![0.0; self.m]);
        }
        let mut l = vec![0.0; self.symbolic.len_val()];
        let mut mem = MemBuffer::try_new(self.scratch).ok()?;
        let stack = MemStack::new(&mut mem);
        let a = SparseColMatRef::new(self.pattern.as_ref(), &g);
        let reg = LltRegularization {
            dynamic_regularization_delta: 1e24 * dmax,
            dynamic_regularization_epsilon: rel * dmax,
        };
        self.symbolic
            .factorize_numeric_llt(
                &mut l,
                a,
                Side::Lower,
                reg,
                Par::Seq,
                stack,
                Default::default(),
            )
            .ok()?;
        let mut z = f[..self.m].to_vec();
        let llt = LltRef::new(&self.symbolic, &l);
        llt.solve_in_place_with_conj(
            Conj::No,
            MatMut::from_column_major_slice_mut(&mut z, self.m, 1),
            Par::Seq,
            stack,
        );
        z.iter().all(|v| v.is_finite()).then_some(z)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_dense_normal_equations() {
        // J (3 × 4), CSR.
        let row_ptr = [0u32, 2, 4, 6];
        let col_idx = [0u32, 1, 1, 2, 2, 3];
        let js = [1.0, 2.0, -1.0, 0.5, 3.0, 1.0];
        let g = SparseGram::new(3, 4, &row_ptr, &col_idx).unwrap();
        let f = [1.0, -2.0, 0.5];
        let z = g.solve(&js, 0.0, 1e-11, &f).unwrap();
        // Dense check: (J Jᵀ) z = f.
        let dense = [
            [1.0, 2.0, 0.0, 0.0],
            [0.0, -1.0, 0.5, 0.0],
            [0.0, 0.0, 3.0, 1.0],
        ];
        for (i, fi) in f.iter().enumerate() {
            let mut s = 0.0;
            for (k, zk) in z.iter().enumerate() {
                let dot: f64 = (0..4).map(|c| dense[i][c] * dense[k][c]).sum();
                s += dot * zk;
            }
            assert!((s - fi).abs() < 1e-12, "{s} vs {fi}");
        }
    }

    #[test]
    fn drops_dependent_rows() {
        // Row 1 = 2 · row 0; consistent right-hand side.
        let row_ptr = [0u32, 1, 2, 3];
        let col_idx = [0u32, 0, 1];
        let js = [1.0, 2.0, 1.0];
        let g = SparseGram::new(3, 2, &row_ptr, &col_idx).unwrap();
        let z = g.solve(&js, 0.0, 1e-11, &[1.0, 2.0, 3.0]).unwrap();
        // Step Jᵀ z must satisfy J x = f.
        let x0 = js[0] * z[0] + js[1] * z[1];
        let x1 = js[2] * z[2];
        assert!(
            (x0 - 1.0).abs() < 1e-9 && (x1 - 3.0).abs() < 1e-9,
            "{x0} {x1}"
        );
    }
}
