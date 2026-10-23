//! Dependency groups on the unsubstituted Jacobian (note §3.4 steps 1–3; SK-04 adds the
//! witness tie-break and leave-one-out refinement).
//!
//! Rows are scanned in system order (entity equations, then constraints by id), so the
//! later-added constraint of a dependent set is the one flagged. A row whose component
//! orthogonal to the earlier independent rows is negligible is dependent; its coefficients
//! `J_k = Σ c_i J_i` come from the Gram–Schmidt factor `R` by a triangular solve, and
//! `{k} ∪ {i : c_i ≠ 0}` is one group (a circuit). Groups are not merged when they share
//! members, so each is a minimal dependent set. If the system is unsatisfied and no dependency
//! is found, the unsatisfied rows form one `Conflicting` group.

use std::collections::BTreeSet;

use crate::ids::ParamId;
use crate::solve::api::{DependencyGroup, GroupKind};
use crate::solve::proto::linalg::norm2;
use crate::solve::system::{EquationOwner, EquationSystem};

/// Relative size of a row's component orthogonal to the earlier rows below which it is
/// dependent.
const DEP_REL: f64 = 1e-8;

/// Groups at `x`; `None` if `cancel` fired.
pub(crate) fn groups(
    sys: &EquationSystem,
    free: &[ParamId],
    x: &[f64],
    tol: f64,
    cancel: &dyn Fn() -> bool,
) -> Option<Vec<DependencyGroup>> {
    let m = sys.rows();
    let n = free.len();
    let mut col = vec![u32::MAX; x.len()];
    for (k, p) in free.iter().enumerate() {
        if p.index() < x.len() {
            col[p.index()] = k as u32;
        }
    }
    let mut j = vec![0.0; m * n];
    let mut block = Vec::new();
    for (_, res, rows) in sys.iter() {
        let params = res.params();
        block.clear();
        block.resize(rows.len() * params.len(), 0.0);
        res.jacobian(x, &mut block);
        for (i, r) in rows.clone().enumerate() {
            for (pj, p) in params.iter().enumerate() {
                let c = col.get(p.index()).copied().unwrap_or(u32::MAX);
                if c != u32::MAX {
                    j[r * n + c as usize] += block[i * params.len() + pj];
                }
            }
        }
    }
    let mut f = vec![0.0; m];
    sys.eval(x, &mut f);
    let jmax = j.iter().fold(0.0_f64, |a, v| a.max(v.abs()));
    let zero = 1e-12 * jmax.max(1e-300);
    let owner = |r: usize| {
        sys.row_owner(r)
            .unwrap_or(EquationOwner::Constraint(crate::ConstraintId(u32::MAX)))
    };

    // Orthonormal basis `q` of the independent rows and the upper-triangular `R` with
    // `J_{indep[j]} = Σ_{l ≤ j} R[l][j] q_l` (column `j` of R stored as `rcol[j]`).
    let mut basis: Vec<Vec<f64>> = Vec::new();
    let mut rcol: Vec<Vec<f64>> = Vec::new();
    let mut indep: Vec<usize> = Vec::new();
    let mut out: BTreeSet<(GroupKind, Vec<EquationOwner>)> = BTreeSet::new();
    for r in 0..m {
        if r % 16 == 0 && cancel() {
            return None;
        }
        let row = &j[r * n..(r + 1) * n];
        let nr = norm2(row).sqrt();
        let mut coef = vec![0.0; basis.len()];
        let mut dependent = nr <= zero;
        if !dependent {
            let mut v = row.to_vec();
            for _ in 0..2 {
                for (l, q) in basis.iter().enumerate() {
                    let d: f64 = q.iter().zip(&v).map(|(a, b)| a * b).sum();
                    coef[l] += d;
                    for (vi, qi) in v.iter_mut().zip(q) {
                        *vi -= d * qi;
                    }
                }
            }
            let nv = norm2(&v).sqrt();
            if nv <= DEP_REL * nr {
                dependent = true;
            } else {
                v.iter_mut().for_each(|e| *e /= nv);
                coef.push(nv);
                basis.push(v);
                rcol.push(coef);
                indep.push(r);
                continue;
            }
        }
        let mut rows = vec![r];
        if !dependent || nr <= zero || indep.is_empty() {
            // Zero row: dependent on its own.
        } else {
            // Solve R c = coef (back substitution).
            let k = indep.len();
            let mut c = coef;
            for jj in (0..k).rev() {
                c[jj] /= rcol[jj][jj];
                let cj = c[jj];
                for (l, cl) in c.iter_mut().enumerate().take(jj) {
                    *cl -= rcol[jj][l] * cj;
                }
            }
            let cmax = c.iter().fold(0.0_f64, |a, v| a.max(v.abs()));
            rows.extend(
                indep
                    .iter()
                    .zip(&c)
                    .filter(|(_, ci)| ci.abs() > 1e-6 * cmax)
                    .map(|(&i, _)| i),
            );
        }
        let satisfied = rows.iter().all(|&i| f[i].abs() <= tol);
        let set: BTreeSet<EquationOwner> = rows.iter().map(|&i| owner(i)).collect();
        out.insert((
            if satisfied {
                GroupKind::Redundant
            } else {
                GroupKind::Conflicting
            },
            set.into_iter().collect(),
        ));
    }
    if out.is_empty() {
        let bad: BTreeSet<EquationOwner> =
            (0..m).filter(|&i| f[i].abs() > tol).map(owner).collect();
        if !bad.is_empty() {
            out.insert((GroupKind::Conflicting, bad.into_iter().collect()));
        }
    }
    let mut groups: Vec<DependencyGroup> = out
        .into_iter()
        .filter_map(|(kind, owners)| {
            let constraints: Vec<_> = owners
                .iter()
                .filter_map(|o| match o {
                    EquationOwner::Constraint(id) => Some(*id),
                    EquationOwner::Entity(_) => None,
                })
                .collect();
            (!constraints.is_empty()).then_some(DependencyGroup { kind, constraints })
        })
        .collect();
    groups.dedup();
    Some(groups)
}
