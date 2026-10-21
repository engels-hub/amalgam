//! Dependency groups on the unsubstituted Jacobian (prototype of note §3.4 steps 1–3).
//!
//! Rows are scanned in system order (arc equations, then constraints by id), so the
//! later-added constraint of a dependent set is the one flagged. Each dependent row `k` is
//! written as `J_k = Σ c_i J_i` over the independent rows before it; `{k} ∪ {i : c_i ≠ 0}` is
//! one group (a circuit). Groups are *not* merged when they share members, so each one is a
//! minimal dependent set. If the system is unsatisfied and no dependency is found, the
//! unsatisfied rows form one `Conflicting` group (like libslvs's failed list).

use std::collections::BTreeSet;

use super::linalg::{chol_semidef, chol_solve, gram, norm2};
use super::plan::Plan;
use super::{ProtoGroup, ProtoGroupKind};
use crate::solve::system::EquationOwner;

/// Relative size of a row's component orthogonal to the earlier rows below which it is
/// dependent.
const DEP_REL: f64 = 1e-8;

pub(crate) fn groups(plan: &Plan, x: &[f64]) -> Vec<ProtoGroup> {
    let sys = &plan.sys.system;
    let m = sys.rows();
    let n_all = x.len();
    let free = &plan.sys.free;
    let n = free.len();
    let mut full = vec![0.0; m * n_all];
    sys.jacobian_dense(x, &mut full);
    let mut j = vec![0.0; m * n];
    for r in 0..m {
        for (k, p) in free.iter().enumerate() {
            j[r * n + k] = full[r * n_all + p.index()];
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
    let mut basis: Vec<Vec<f64>> = Vec::new();
    let mut indep: Vec<usize> = Vec::new();
    let mut out: BTreeSet<ProtoGroup> = BTreeSet::new();
    for r in 0..m {
        let row = &j[r * n..(r + 1) * n];
        let nr = norm2(row).sqrt();
        let mut dependent = nr <= zero;
        if !dependent {
            let mut v = row.to_vec();
            for _ in 0..2 {
                for q in &basis {
                    let d: f64 = q.iter().zip(&v).map(|(a, b)| a * b).sum();
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
                basis.push(v);
                indep.push(r);
            }
        }
        if !dependent {
            continue;
        }
        let mut rows = vec![r];
        if nr > zero && !indep.is_empty() {
            // c = (J_I J_Iᵀ)⁻¹ J_I J_rᵀ.
            let k = indep.len();
            let ji: Vec<f64> = indep
                .iter()
                .flat_map(|&i| j[i * n..(i + 1) * n].iter().copied())
                .collect();
            let mut g = gram(&ji, k, n, 0.0);
            let dep = chol_semidef(&mut g, k, 1e-14);
            let rhs: Vec<f64> = indep
                .iter()
                .map(|&i| {
                    j[i * n..(i + 1) * n]
                        .iter()
                        .zip(row)
                        .map(|(a, b)| a * b)
                        .sum()
                })
                .collect();
            let c = chol_solve(&g, &dep, k, &rhs);
            let cmax = c.iter().fold(0.0_f64, |a, v| a.max(v.abs()));
            rows.extend(
                indep
                    .iter()
                    .zip(&c)
                    .filter(|(_, ci)| ci.abs() > 1e-6 * cmax)
                    .map(|(&i, _)| i),
            );
        }
        let satisfied = rows.iter().all(|&i| f[i].abs() <= plan.tol);
        out.insert(group(
            rows.iter().map(|&i| owner(i)),
            if satisfied {
                ProtoGroupKind::Redundant
            } else {
                ProtoGroupKind::Conflicting
            },
        ));
    }
    if out.is_empty() {
        let bad: Vec<usize> = (0..m).filter(|&i| f[i].abs() > plan.tol).collect();
        if !bad.is_empty() {
            out.insert(group(
                bad.into_iter().map(owner),
                ProtoGroupKind::Conflicting,
            ));
        }
    }
    out.into_iter().collect()
}

fn group(owners: impl Iterator<Item = EquationOwner>, kind: ProtoGroupKind) -> ProtoGroup {
    let set: BTreeSet<EquationOwner> = owners.collect();
    ProtoGroup {
        kind,
        owners: set.into_iter().collect(),
    }
}
