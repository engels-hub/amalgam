//! Dependency circuits on the unsubstituted Jacobian (note §3.4 steps 1–3).
//!
//! [`RawSystem`] holds every scalar equation row of a sketch at one configuration: the rows of
//! the equation system and, for every `Fix` constraint, one unit row per parameter it locks
//! (so a fix that is implied by, or duplicates, other constraints shows up in a group). Its
//! columns are the free unknowns plus the parameters locked by `Fix`; projected (locked)
//! parameters are constants.
//!
//! Rows are ordered by owner: entity equations, then constraints by id (a `Fix` at its id), so
//! the later-added constraint of a dependent set is the one flagged. [`circuits`]
//! scans a set of rows in that order; a row whose component orthogonal to the earlier
//! independent rows is negligible is dependent, its coefficients `J_k = Σ c_i J_i` come from
//! the Gram–Schmidt factor `R` by a triangular solve, and `{k} ∪ {i : c_i ≠ 0}` is one circuit
//! (a minimal dependent set). Rows of different connected components have disjoint supports,
//! so circuits never cross components and a component's circuits do not depend on the rest of
//! the sketch.

use std::collections::BTreeMap;

use smallvec::SmallVec;

use crate::ids::{ConstraintId, ParamId};
use crate::model::{ConstraintKind, Sketch};
use crate::solve::proto::linalg::norm2;
use crate::solve::system::{EquationOwner, EquationSystem};

/// Relative size of a row's component orthogonal to the earlier rows below which it is
/// dependent.
const DEP_REL: f64 = 1e-8;

/// Where a raw row comes from.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum RowSrc {
    /// Row `row` of residual block `block` of the equation system.
    Res { block: u32, row: u16 },
    /// The unit row of a parameter locked by a `Fix`.
    Fix { param: ParamId },
}

/// One scalar row.
#[derive(Clone, Debug)]
pub(crate) struct RawRow {
    pub owner: EquationOwner,
    pub src: RowSrc,
    /// Columns the row may touch (structural), ascending, deduplicated.
    pub cols: SmallVec<[u32; 8]>,
}

/// The unsubstituted rows of a sketch (structure only; values come with [`RawSystem::at`]).
#[derive(Debug)]
pub(crate) struct RawSystem {
    pub rows: Vec<RawRow>,
    /// Column of each parameter index (`u32::MAX`: constant).
    pub col: Vec<u32>,
    pub ncols: usize,
    /// Parameters locked by more than one `Fix`.
    pub multi_fixed: bool,
}

/// Values of a [`RawSystem`] at one configuration: residuals and dense-on-demand row
/// gradients.
pub(crate) struct RawValues {
    pub f: Vec<f64>,
    /// Per row: `(column, value)` pairs.
    pub grad: Vec<SmallVec<[(u32, f64); 8]>>,
}

impl RawSystem {
    pub(crate) fn new(sketch: &Sketch, sys: &EquationSystem, free: &[ParamId]) -> RawSystem {
        let n = sketch.params().len();
        let mut col = vec![u32::MAX; n];
        let mut ncols = 0u32;
        let mut fix_params: BTreeMap<ConstraintId, Vec<ParamId>> = BTreeMap::new();
        let mut fix_count: BTreeMap<ParamId, u32> = BTreeMap::new();
        let store = sketch.params();
        for (id, c) in sketch.constraints() {
            let ConstraintKind::Fix { entity } = c.kind else {
                continue;
            };
            let Some(e) = sketch.entity(entity) else {
                continue;
            };
            let mut ps: Vec<ParamId> = e.kind.own_params().into_iter().collect();
            for child in e.kind.children() {
                if let Some(ce) = sketch.entity(child) {
                    ps.extend(ce.kind.own_params());
                }
            }
            ps.retain(|p| p.index() < n && !store.flags(*p).locked);
            ps.sort_unstable();
            ps.dedup();
            for &p in &ps {
                *fix_count.entry(p).or_default() += 1;
            }
            fix_params.insert(id, ps);
        }
        // Columns: free unknowns and fix-locked params, in parameter order.
        let mut is_col = vec![false; n];
        for p in free.iter().chain(fix_count.keys()) {
            if p.index() < n {
                is_col[p.index()] = true;
            }
        }
        for (i, c) in col.iter_mut().enumerate() {
            if is_col[i] {
                *c = ncols;
                ncols += 1;
            }
        }
        let mut rows = Vec::with_capacity(sys.rows());
        let push_block = |rows: &mut Vec<RawRow>, bi: usize| {
            let (owner, res, range) = (sys.owner(bi), sys.residual(bi), sys.row_range(bi));
            let mut cols: SmallVec<[u32; 8]> = res
                .params()
                .iter()
                .filter_map(|p| col.get(p.index()).copied().filter(|&c| c != u32::MAX))
                .collect();
            cols.sort_unstable();
            cols.dedup();
            for r in 0..range.len() {
                rows.push(RawRow {
                    owner,
                    src: RowSrc::Res {
                        block: bi as u32,
                        row: r as u16,
                    },
                    cols: cols.clone(),
                });
            }
        };
        // Merge the system blocks (entities first, then constraints by id) with the fix rows.
        let nb = sys.len();
        let mut bi = 0;
        while bi < nb && matches!(sys.owner(bi), EquationOwner::Entity(_)) {
            push_block(&mut rows, bi);
            bi += 1;
        }
        for (id, ps) in &fix_params {
            while bi < nb && matches!(sys.owner(bi), EquationOwner::Constraint(c) if c < *id) {
                push_block(&mut rows, bi);
                bi += 1;
            }
            for &p in ps {
                rows.push(RawRow {
                    owner: EquationOwner::Constraint(*id),
                    src: RowSrc::Fix { param: p },
                    cols: SmallVec::from_slice(&[col[p.index()]]),
                });
            }
        }
        while bi < nb {
            push_block(&mut rows, bi);
            bi += 1;
        }
        RawSystem {
            rows,
            col,
            ncols: ncols as usize,
            multi_fixed: fix_count.values().any(|&c| c > 1),
        }
    }

    /// Residuals and gradients at `x`.
    pub(crate) fn at(&self, sys: &EquationSystem, x: &[f64]) -> RawValues {
        let mut f = vec![0.0; self.rows.len()];
        let mut grad = vec![SmallVec::new(); self.rows.len()];
        let mut i = 0;
        let mut vals = Vec::new();
        let mut jac = Vec::new();
        while i < self.rows.len() {
            match self.rows[i].src {
                RowSrc::Fix { param } => {
                    grad[i].push((self.col[param.index()], 1.0));
                    i += 1;
                }
                RowSrc::Res { block, .. } => {
                    let (res, range) =
                        (sys.residual(block as usize), sys.row_range(block as usize));
                    let (nr, params) = (range.len(), res.params());
                    vals.clear();
                    vals.resize(nr, 0.0);
                    res.eval(x, &mut vals);
                    jac.clear();
                    jac.resize(nr * params.len(), 0.0);
                    res.jacobian(x, &mut jac);
                    for r in 0..nr {
                        f[i + r] = vals[r];
                        let g = &mut grad[i + r];
                        for (k, p) in params.iter().enumerate() {
                            let c = self.col.get(p.index()).copied().unwrap_or(u32::MAX);
                            let v = jac[r * params.len() + k];
                            if c == u32::MAX || v == 0.0 {
                                continue;
                            }
                            let v = if v.is_finite() { v } else { 0.0 };
                            match g.iter_mut().find(|(cc, _)| *cc == c) {
                                Some(e) => e.1 += v,
                                None => g.push((c, v)),
                            }
                        }
                    }
                    i += nr;
                }
            }
        }
        RawValues { f, grad }
    }

    /// Connected components of the rows (rows sharing a column), each ascending; rows without
    /// columns are singletons.
    pub(crate) fn components(&self) -> Vec<Vec<usize>> {
        let mut parent: Vec<u32> = (0..self.ncols as u32).collect();
        fn find(p: &mut [u32], mut i: usize) -> usize {
            while p[i] as usize != i {
                let q = p[i] as usize;
                p[i] = p[q];
                i = q;
            }
            i
        }
        for r in &self.rows {
            for w in r.cols.windows(2) {
                let (a, b) = (
                    find(&mut parent, w[0] as usize),
                    find(&mut parent, w[1] as usize),
                );
                if a != b {
                    let (lo, hi) = if a < b { (a, b) } else { (b, a) };
                    parent[hi] = lo as u32;
                }
            }
        }
        let mut by_root: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
        let mut singles = Vec::new();
        for (i, r) in self.rows.iter().enumerate() {
            match r.cols.first() {
                Some(&c) => by_root
                    .entry(find(&mut parent, c as usize))
                    .or_default()
                    .push(i),
                None => singles.push(vec![i]),
            }
        }
        let mut out: Vec<Vec<usize>> = by_root.into_values().collect();
        out.extend(singles);
        out.sort_by_key(|c| c[0]);
        out
    }
}

/// Circuits among the rows `ids` (ascending, in scan order) with gradients `grads[i]`
/// (`(column, value)` pairs of row `ids[i]`), each circuit as ascending ids. `None` if
/// `cancel` fired.
pub(crate) fn circuits(
    ids: &[usize],
    grads: &[&[(u32, f64)]],
    cancel: &dyn Fn() -> bool,
) -> Option<Vec<Vec<usize>>> {
    // Local columns.
    let mut cols: Vec<u32> = grads.iter().flat_map(|g| g.iter().map(|e| e.0)).collect();
    cols.sort_unstable();
    cols.dedup();
    let n = cols.len();
    let dense_row = |g: &[(u32, f64)]| {
        let mut out = vec![0.0; n];
        for &(c, val) in g {
            if let Ok(k) = cols.binary_search(&c) {
                out[k] += val;
            }
        }
        out
    };
    let jmax = grads
        .iter()
        .flat_map(|g| g.iter().map(|e| e.1.abs()))
        .fold(0.0_f64, f64::max);
    let zero = 1e-12 * jmax.max(1e-300);

    // Orthonormal basis `q` of the independent rows and the upper-triangular `R` with
    // `J_{indep[j]} = Σ_{l ≤ j} R[l][j] q_l` (column `j` of R stored as `rcol[j]`).
    let mut basis: Vec<Vec<f64>> = Vec::new();
    let mut rcol: Vec<Vec<f64>> = Vec::new();
    let mut indep: Vec<usize> = Vec::new();
    let mut out = Vec::new();
    for (i, (&r, g)) in ids.iter().zip(grads).enumerate() {
        if i % 16 == 15 && cancel() {
            return None;
        }
        let row = dense_row(g);
        let nr = norm2(&row).sqrt();
        let mut coef = vec![0.0; basis.len()];
        if nr > zero {
            let mut w = row;
            for _ in 0..2 {
                for (l, q) in basis.iter().enumerate() {
                    let d: f64 = q.iter().zip(&w).map(|(a, b)| a * b).sum();
                    coef[l] += d;
                    for (wi, qi) in w.iter_mut().zip(q) {
                        *wi -= d * qi;
                    }
                }
            }
            let nw = norm2(&w).sqrt();
            if nw > DEP_REL * nr {
                w.iter_mut().for_each(|e| *e /= nw);
                coef.push(nw);
                basis.push(w);
                rcol.push(coef);
                indep.push(r);
                continue;
            }
        }
        let mut members = vec![r];
        if nr > zero && !indep.is_empty() {
            // Solve R c = coef (back substitution).
            let mut c = coef;
            for jj in (0..indep.len()).rev() {
                c[jj] /= rcol[jj][jj];
                let cj = c[jj];
                for (l, cl) in c.iter_mut().enumerate().take(jj) {
                    *cl -= rcol[jj][l] * cj;
                }
            }
            let cmax = c.iter().fold(0.0_f64, |a, v| a.max(v.abs()));
            members.extend(
                indep
                    .iter()
                    .zip(&c)
                    .filter(|(_, ci)| ci.abs() > 1e-6 * cmax)
                    .map(|(&i, _)| i),
            );
        }
        members.sort_unstable();
        out.push(members);
    }
    Some(out)
}
