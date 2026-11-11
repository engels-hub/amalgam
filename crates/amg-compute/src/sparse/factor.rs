//! Sparse Cholesky (`L·Lᵀ`) and Bunch–Kaufman `L·B·Lᵀ` on faer's supernodal factorisations.
//!
//! The upper-CSR [`SymCsr`] is read by faer as the lower triangle in CSC (`Side::Lower`). Only
//! the column indices are copied once per analysis (faer wants one index type for offsets and
//! indices, and offsets are `usize`); the values are passed without a copy.

use std::cell::Cell;
use std::fmt;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering as AtomicOrdering};

use amg_jobs::JobCtx;
use faer::dyn_stack::{MemBuffer, MemStack, StackReq};
use faer::linalg::cholesky::llt::factor::{LltError, LltRegularization};
use faer::perm::PermRef;
use faer::sparse::linalg::SupernodalThreshold;
use faer::sparse::linalg::cholesky::{
    CholeskySymbolicParams, IntranodeLbltRef, LltRef, SymbolicCholesky, SymbolicCholeskyRaw,
    SymmetricOrdering, factorize_symbolic_cholesky,
};
use faer::sparse::{SparseColMatRef, SymbolicSparseColMat};
use faer::{Conj, MatMut, Par, Side};

use super::SymCsr;
use crate::{ComputeError, ComputeResult};

/// Fill-reducing ordering of the symbolic analysis.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub enum Ordering {
    /// Approximate minimum degree (faer's AMD).
    #[default]
    Amd,
    /// No reordering.
    Identity,
    /// `perm[new] = old`: row `perm[k]` of the input is eliminated `k`-th.
    Custom(Vec<u32>),
}

/// Options of [`SymbolicFactor::analyze`]; they apply to every factorisation of that analysis.
#[derive(Clone, Debug)]
pub struct FactorOpts {
    /// Fill-reducing ordering. Default [`Ordering::Amd`].
    pub ordering: Ordering,
    /// [`PivotReport::below_threshold`] is set when the smallest `d_ii / a_ii` is below this.
    /// Default `1e-12`.
    pub pivot_warn: f64,
    /// Threads for factorisation and solves: `None` = the current rayon pool, `Some(1)` =
    /// sequential. Results are bit-identical across runs for a given value.
    pub threads: Option<usize>,
}

impl Default for FactorOpts {
    fn default() -> Self {
        Self {
            ordering: Ordering::Amd,
            pivot_warn: 1e-12,
            threads: None,
        }
    }
}

/// Size of a factorisation, known exactly after the symbolic analysis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FactorEstimate {
    /// Non-zeros of `L` (lower triangle, diagonal included).
    pub factor_nnz: u64,
    /// Bytes a [`Cholesky`] holds: faer's supernodal value storage (dense diagonal blocks,
    /// `f64`) plus the symbolic structure and permutations. An [`Ldlt`] holds `24·n` more (the
    /// 2×2 off-diagonals and its pivoting permutation). Transient workspace during factoring is
    /// not included.
    pub bytes: u64,
    /// `Σ_j c_j²` over the columns of `L` (`c_j` = non-zeros of column `j`): the classic
    /// Cholesky operation count, a proxy for the factorisation time.
    pub flops: f64,
}

/// Smallest relative pivot of a successful factorisation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PivotReport {
    /// `min_i d_ii / a_ii`, where `d_ii = l_ii²` is the pivot and `a_ii` the input diagonal.
    pub min_ratio: f64,
    /// Row of the minimum, in the original numbering.
    pub row: usize,
    /// `min_ratio < FactorOpts::pivot_warn`: the matrix is close to singular (e.g. a nearly
    /// unconstrained body).
    pub below_threshold: bool,
}

/// Pivot sign counts of an `L·B·Lᵀ` factorisation (Sylvester's law of inertia: the eigenvalue
/// sign counts of the matrix).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Inertia {
    /// Negative eigenvalues.
    pub negative: usize,
    /// Exactly zero pivots.
    pub zero: usize,
    /// Positive eigenvalues.
    pub positive: usize,
}

thread_local! {
    static ANALYSES: Cell<u64> = const { Cell::new(0) };
}

/// Number of [`SymbolicFactor::analyze`] calls made on the calling thread (a diagnostic counter:
/// a test can check that re-factoring on a fixed pattern does not analyse again).
pub fn analyses_on_this_thread() -> u64 {
    ANALYSES.with(Cell::get)
}

/// Symbolic analysis of a pattern; reuse it for every numeric factorisation on that pattern.
pub struct SymbolicFactor {
    n: usize,
    /// The analysed pattern in faer's index type (upper CSR = lower CSC).
    pattern: SymbolicSparseColMat<usize>,
    faer: SymbolicCholesky<usize>,
    perm: Vec<u32>,
    pivot_warn: f64,
    par: Par,
    estimate: FactorEstimate,
    numeric: AtomicU64,
}

impl fmt::Debug for SymbolicFactor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SymbolicFactor")
            .field("n", &self.n)
            .field("estimate", &self.estimate)
            .field("par", &self.par)
            .finish_non_exhaustive()
    }
}

fn faer_failure(what: &str, e: impl fmt::Debug) -> ComputeError {
    ComputeError::InvalidMatrix(format!("{what} failed: {e:?}"))
}

fn buffer(req: StackReq) -> ComputeResult<MemBuffer> {
    MemBuffer::try_new(req).map_err(|e| faer_failure("allocating factorisation workspace", e))
}

impl SymbolicFactor {
    /// Orders and analyses the pattern of `a` (its values are not read).
    ///
    /// The analysis is always supernodal (also for very sparse matrices), so `Cholesky` and
    /// `Ldlt` share one code path and `Ldlt` always pivots.
    ///
    /// # Errors
    /// [`ComputeError::Shape`] for an empty matrix; [`ComputeError::InvalidMatrix`] for an
    /// invalid custom ordering or when faer runs out of memory or index range.
    pub fn analyze(a: &SymCsr, opts: &FactorOpts) -> ComputeResult<Arc<SymbolicFactor>> {
        ANALYSES.with(|c| c.set(c.get() + 1));
        let n = a.n();
        if n == 0 {
            return Err(ComputeError::Shape("cannot factor an empty matrix".into()));
        }
        let row_idx: Vec<usize> = a.cols().iter().map(|&c| c as usize).collect();
        let pattern = SymbolicSparseColMat::new_checked(n, n, a.row_ptr().to_vec(), None, row_idx);

        // faer's `SymmetricOrdering::Identity` skips the conversion of a `Side::Lower` input in
        // the symbolic phase (0.23.2), so the identity is passed as an explicit permutation.
        let (fwd, inv) = match &opts.ordering {
            Ordering::Custom(p) => custom_perm(p, n)?,
            Ordering::Identity => ((0..n).collect(), (0..n).collect()),
            Ordering::Amd => (Vec::new(), Vec::new()),
        };
        let ord = match &opts.ordering {
            Ordering::Amd => SymmetricOrdering::Amd,
            Ordering::Identity | Ordering::Custom(_) => {
                SymmetricOrdering::Custom(PermRef::new_checked(&fwd, &inv, n))
            }
        };
        let params = CholeskySymbolicParams {
            supernodal_flop_ratio_threshold: SupernodalThreshold::FORCE_SUPERNODAL,
            ..Default::default()
        };
        let faer = factorize_symbolic_cholesky(pattern.as_ref(), Side::Lower, ord, params)
            .map_err(|e| faer_failure("symbolic analysis", e))?;
        let perm = match faer.perm() {
            Some(p) => p.arrays().0.iter().map(|&i| i as u32).collect(),
            None => (0..n as u32).collect(),
        };
        let estimate = estimate(&faer);
        let par = match opts.threads {
            Some(1) => Par::Seq,
            Some(t) => Par::rayon(t),
            None => Par::rayon(0),
        };
        Ok(Arc::new(SymbolicFactor {
            n,
            pattern,
            faer,
            perm,
            pivot_warn: opts.pivot_warn,
            par,
            estimate,
            numeric: AtomicU64::new(0),
        }))
    }

    /// Size of the factorisation (exact).
    pub fn estimate(&self) -> FactorEstimate {
        self.estimate
    }

    /// The fill-reducing permutation, `perm[new] = old`.
    pub fn permutation(&self) -> &[u32] {
        &self.perm
    }

    /// Matrix size.
    pub fn n(&self) -> usize {
        self.n
    }

    /// Numeric factorisations ([`Cholesky::factor`], [`Ldlt::factor`]) done on this analysis.
    pub fn numeric_factorizations(&self) -> u64 {
        self.numeric.load(AtomicOrdering::Relaxed)
    }

    /// `a` as faer's lower CSC, after checking it has the analysed pattern.
    fn matrix<'a>(&'a self, a: &'a SymCsr) -> ComputeResult<SparseColMatRef<'a, usize, f64>> {
        let p = self.pattern.as_ref();
        let same = a.n() == self.n
            && a.row_ptr() == p.col_ptr()
            && a.cols().len() == p.row_idx().len()
            && a.cols()
                .iter()
                .zip(p.row_idx())
                .all(|(&c, &r)| c as usize == r);
        if !same {
            return Err(ComputeError::InvalidMatrix(
                "the matrix does not have the analysed pattern".into(),
            ));
        }
        Ok(SparseColMatRef::new(p, a.vals()))
    }

    /// Diagonal blocks as `(first column, width, dense block row count, value offset)`: the
    /// supernodes, or one column each when faer chose its simplicial factorisation (it does so,
    /// despite `FORCE_SUPERNODAL`, when `L` has no off-diagonal entries, i.e. for a diagonal
    /// matrix; the diagonal is then the first value of each column).
    fn supernodes(&self) -> Vec<(usize, usize, usize, usize)> {
        match self.faer.raw() {
            SymbolicCholeskyRaw::Supernodal(s) => (0..s.n_supernodes())
                .map(|k| {
                    let begin = s.supernode_begin()[k];
                    let width = s.supernode_end()[k] - begin;
                    let rows = width + s.supernode(k).pattern().len();
                    (begin, width, rows, s.col_ptr_for_val()[k])
                })
                .collect(),
            SymbolicCholeskyRaw::Simplicial(s) => {
                (0..self.n).map(|j| (j, 1, 1, s.col_ptr()[j])).collect()
            }
        }
    }
}

fn custom_perm(p: &[u32], n: usize) -> ComputeResult<(Vec<usize>, Vec<usize>)> {
    if p.len() != n {
        return Err(ComputeError::InvalidMatrix(format!(
            "custom ordering has {} entries, expected {n}",
            p.len()
        )));
    }
    let fwd: Vec<usize> = p.iter().map(|&i| i as usize).collect();
    let mut inv = vec![usize::MAX; n];
    for (new, &old) in fwd.iter().enumerate() {
        if old >= n || inv[old] != usize::MAX {
            return Err(ComputeError::InvalidMatrix(
                "custom ordering is not a permutation".into(),
            ));
        }
        inv[old] = new;
    }
    Ok((fwd, inv))
}

fn estimate(faer: &SymbolicCholesky<usize>) -> FactorEstimate {
    const W: u64 = size_of::<usize>() as u64;
    let n = faer.nrows() as u64;
    let (mut nnz, mut flops) = (0u64, 0f64);
    let index_bytes = match faer.raw() {
        SymbolicCholeskyRaw::Supernodal(s) => {
            for k in 0..s.n_supernodes() {
                let w = (s.supernode_end()[k] - s.supernode_begin()[k]) as u64;
                let p = s.supernode(k).pattern().len() as u64;
                for j in 0..w {
                    let c = (w - j) + p;
                    nnz += c;
                    flops += (c as f64) * (c as f64);
                }
            }
            let ns = s.n_supernodes() as u64;
            // row_idx + supernode_begin + col_ptr_for_row_idx + col_ptr_for_val.
            (s.row_idx().len() as u64 + 3 * (ns + 1)) * W
        }
        SymbolicCholeskyRaw::Simplicial(s) => {
            for w in s.col_ptr().windows(2) {
                let c = (w[1] - w[0]) as u64;
                nnz += c;
                flops += (c as f64) * (c as f64);
            }
            (s.row_idx().len() as u64 + n + 1) * W
        }
    };
    let perm_bytes = 2 * n * W;
    FactorEstimate {
        factor_nnz: nnz,
        bytes: faer.len_val() as u64 * 8 + index_bytes + perm_bytes,
        flops,
    }
}

/// A column-major `n × nrhs` view of `b` and the solve workspace.
fn rhs_view(b: &mut [f64], n: usize, nrhs: usize) -> MatMut<'_, f64> {
    assert_eq!(
        b.len(),
        n * nrhs,
        "right-hand side has {} entries, expected n · nrhs = {}",
        b.len(),
        n * nrhs
    );
    MatMut::from_column_major_slice_mut(b, n, nrhs)
}

fn solve_buffer(sym: &SymbolicFactor, nrhs: usize) -> MemBuffer {
    MemBuffer::new(sym.faer.solve_in_place_scratch::<f64>(nrhs, sym.par))
}

/// `L·Lᵀ`. Errors with `NotPositiveDefinite { row }` (original numbering).
pub struct Cholesky {
    sym: Arc<SymbolicFactor>,
    values: Vec<f64>,
    pivots: PivotReport,
}

impl fmt::Debug for Cholesky {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Cholesky")
            .field("n", &self.sym.n)
            .field("pivots", &self.pivots)
            .finish_non_exhaustive()
    }
}

impl Cholesky {
    /// Numeric factorisation of `a` on the analysis `sym` (no re-analysis).
    ///
    /// Cancellation is checked before and after the numeric phase; faer's factorisation itself
    /// cannot be interrupted.
    ///
    /// # Errors
    /// [`ComputeError::NotPositiveDefinite`] with the row in the original numbering (`pivot` is
    /// NaN: faer does not report its value); [`ComputeError::InvalidMatrix`] if `a` does not have
    /// the analysed pattern; [`ComputeError::Cancelled`].
    pub fn factor(sym: &Arc<SymbolicFactor>, a: &SymCsr, ctx: &JobCtx) -> ComputeResult<Cholesky> {
        ctx.checkpoint()?;
        let mat = sym.matrix(a)?;
        sym.numeric.fetch_add(1, AtomicOrdering::Relaxed);
        let par = sym.par;
        let mut values = vec![0.0; sym.faer.len_val()];
        let mut mem = buffer(
            sym.faer
                .factorize_numeric_llt_scratch::<f64>(par, Default::default()),
        )?;
        let res = sym.faer.factorize_numeric_llt(
            &mut values,
            mat,
            Side::Lower,
            LltRegularization::default(),
            par,
            MemStack::new(&mut mem),
            Default::default(),
        );
        drop(mem);
        match res {
            Ok(_) => {}
            Err(LltError::NonPositivePivot { index }) => {
                return Err(ComputeError::NotPositiveDefinite {
                    row: sym.perm.get(index).map_or(index, |&r| r as usize),
                    pivot: f64::NAN,
                });
            }
        }
        ctx.checkpoint()?;
        let pivots = pivot_report(sym, &values, &a.diagonal());
        Ok(Cholesky {
            sym: Arc::clone(sym),
            values,
            pivots,
        })
    }

    /// The smallest relative pivot.
    pub fn pivots(&self) -> PivotReport {
        self.pivots
    }

    /// The analysis this factorisation was computed on.
    pub fn symbolic(&self) -> &Arc<SymbolicFactor> {
        &self.sym
    }

    /// Solves `A·x = b`; `b` is overwritten with `x`.
    ///
    /// # Panics
    /// If `b` does not have `n` entries.
    pub fn solve_in_place(&self, b: &mut [f64]) {
        self.solve_many(b, 1);
    }

    /// Solves for `nrhs` right-hand sides at once; `b` is `n × nrhs`, column-major.
    ///
    /// # Panics
    /// If `b` does not have `n · nrhs` entries.
    pub fn solve_many(&self, b: &mut [f64], nrhs: usize) {
        let sym = &*self.sym;
        let rhs = rhs_view(b, sym.n, nrhs);
        let mut mem = solve_buffer(sym, nrhs);
        LltRef::new(&sym.faer, &self.values).solve_in_place_with_conj(
            Conj::No,
            rhs,
            sym.par,
            MemStack::new(&mut mem),
        );
    }
}

fn pivot_report(sym: &SymbolicFactor, values: &[f64], diag: &[f64]) -> PivotReport {
    let mut best = PivotReport {
        min_ratio: f64::INFINITY,
        row: 0,
        below_threshold: false,
    };
    for (begin, width, rows, offset) in sym.supernodes() {
        for j in 0..width {
            let l = values[offset + j * rows + j];
            let row = sym.perm[begin + j] as usize;
            let a = diag[row];
            let ratio = if a > 0.0 { l * l / a } else { 0.0 };
            if ratio < best.min_ratio {
                best.min_ratio = ratio;
                best.row = row;
            }
        }
    }
    // NaN ratios (never expected) count as below the threshold.
    best.below_threshold = best.min_ratio.is_nan() || best.min_ratio < sym.pivot_warn;
    best
}

/// `L·B·Lᵀ` with Bunch–Kaufman pivoting inside supernodes (faer's intranode LBLᵀ), for
/// indefinite matrices. `B` is block diagonal with 1×1 and 2×2 blocks.
pub struct Ldlt {
    sym: Arc<SymbolicFactor>,
    values: Vec<f64>,
    subdiag: Vec<f64>,
    perm_fwd: Vec<usize>,
    perm_inv: Vec<usize>,
}

impl fmt::Debug for Ldlt {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Ldlt")
            .field("n", &self.sym.n)
            .field("inertia", &self.inertia())
            .finish_non_exhaustive()
    }
}

impl Ldlt {
    /// Numeric factorisation of `a` on the analysis `sym` (no re-analysis).
    ///
    /// Pivoting is restricted to each supernode's diagonal block, so a pivot can be zero for a
    /// singular matrix: it is then counted in [`Inertia::zero`] and solves are not finite.
    ///
    /// # Errors
    /// [`ComputeError::InvalidMatrix`] if `a` does not have the analysed pattern;
    /// [`ComputeError::Cancelled`].
    pub fn factor(sym: &Arc<SymbolicFactor>, a: &SymCsr, ctx: &JobCtx) -> ComputeResult<Ldlt> {
        ctx.checkpoint()?;
        let mat = sym.matrix(a)?;
        sym.numeric.fetch_add(1, AtomicOrdering::Relaxed);
        let par = sym.par;
        let n = sym.n;
        let mut values = vec![0.0; sym.faer.len_val()];
        let mut subdiag = vec![0.0; n];
        let mut perm_fwd = vec![0usize; n];
        let mut perm_inv = vec![0usize; n];
        let mut mem = buffer(
            sym.faer
                .factorize_numeric_intranode_lblt_scratch::<f64>(par, Default::default()),
        )?;
        sym.faer.factorize_numeric_intranode_lblt(
            &mut values,
            &mut subdiag,
            &mut perm_fwd,
            &mut perm_inv,
            mat,
            Side::Lower,
            par,
            MemStack::new(&mut mem),
            Default::default(),
        );
        drop(mem);
        if let SymbolicCholeskyRaw::Simplicial(simp) = sym.faer.raw() {
            // Only for a diagonal matrix (see `supernodes`): faer's unpivoted simplicial LDLᵀ
            // stops at a zero pivot, so set D from the diagonal itself.
            let diag = a.diagonal();
            for (j, &k) in simp.col_ptr()[..n].iter().enumerate() {
                values[k] = diag[sym.perm[j] as usize];
            }
        }
        ctx.checkpoint()?;
        Ok(Ldlt {
            sym: Arc::clone(sym),
            values,
            subdiag,
            perm_fwd,
            perm_inv,
        })
    }

    /// The analysis this factorisation was computed on.
    pub fn symbolic(&self) -> &Arc<SymbolicFactor> {
        &self.sym
    }

    /// Pivot sign counts of `B`, which equal the eigenvalue sign counts of `A`. A 2×2 block
    /// with negative determinant has one eigenvalue of each sign; otherwise both have the sign
    /// of its trace. Only exactly zero pivots count as zero.
    pub fn inertia(&self) -> Inertia {
        let mut out = Inertia {
            negative: 0,
            zero: 0,
            positive: 0,
        };
        let mut add = |d: f64| {
            if d > 0.0 {
                out.positive += 1;
            } else if d < 0.0 {
                out.negative += 1;
            } else {
                out.zero += 1;
            }
        };
        for (begin, width, rows, offset) in self.sym.supernodes() {
            let diag = |j: usize| self.values[offset + j * rows + j];
            let mut j = 0;
            while j < width {
                let s = self.subdiag[begin + j];
                if s != 0.0 && j + 1 < width {
                    let (a, c) = (diag(j), diag(j + 1));
                    let det = a * c - s * s;
                    if det < 0.0 {
                        add(-1.0);
                        add(1.0);
                    } else if det > 0.0 {
                        add(a + c);
                        add(a + c);
                    } else {
                        add(0.0);
                        add(a + c);
                    }
                    j += 2;
                } else {
                    add(diag(j));
                    j += 1;
                }
            }
        }
        out
    }

    /// Solves `A·x = b`; `b` is overwritten with `x`.
    ///
    /// # Panics
    /// If `b` does not have `n` entries.
    pub fn solve_in_place(&self, b: &mut [f64]) {
        self.solve_many(b, 1);
    }

    /// Solves for `nrhs` right-hand sides at once; `b` is `n × nrhs`, column-major.
    ///
    /// # Panics
    /// If `b` does not have `n · nrhs` entries.
    pub fn solve_many(&self, b: &mut [f64], nrhs: usize) {
        let sym = &*self.sym;
        let rhs = rhs_view(b, sym.n, nrhs);
        let mut mem = solve_buffer(sym, nrhs);
        let perm = PermRef::new_checked(&self.perm_fwd, &self.perm_inv, sym.n);
        IntranodeLbltRef::new(&sym.faer, &self.values, &self.subdiag, perm)
            .solve_in_place_with_conj(Conj::No, rhs, sym.par, MemStack::new(&mut mem));
    }
}
