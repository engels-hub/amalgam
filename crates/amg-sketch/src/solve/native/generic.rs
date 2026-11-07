//! Dimension-agnostic entry points (SK-12 3D sketches; S06 ASM-04 may follow): the same
//! substitution, decomposition, block solves, status and rank test, null-space DOF and
//! dependency circuits as the sketch path, on any [`SketchSystem`] over `n` parameter slots.
//!
//! What the sketch path adds on top and this one does not (it needs a [`Sketch`] to re-solve):
//! the witness tie-break (satisfied circuits are reported `Redundant`, never
//! `DegenerateConfiguration`) and leave-one-out refinement of conflicts (the unsatisfied
//! circuits are reported). No structure cache: every call builds the structure.
//!
//! [`Sketch`]: crate::Sketch

use std::collections::{BTreeMap, BTreeSet};

use smallvec::SmallVec;

use super::analysis::{SubAnalysis, grads_of, group_of, sort_groups, substituted};
use super::diagnose::{RawSystem, RowSrc, circuits};
use super::structure::{Slot, Structure, SubstPairs};
use super::{NativeSolver, effective_tol, outcome};
use crate::ids::{ConstraintId, EntityId, ParamId};
use crate::solve::api::{
    Diagnosis, DofAnalysis, DofReport, GroupKind, SolveOptions, SolveReport, SolveStatus,
};
use crate::solve::residual::SketchSystem;
use crate::solve::system::EquationOwner;

/// A system plus what the solver needs to know about its owner.
pub(crate) struct Problem<'a> {
    /// Equations, fixed and free parameters, `L_c`.
    pub sys: &'a SketchSystem,
    /// Number of parameter slots (`x.len()`).
    pub n: usize,
    /// Blocks that are exactly `p − q = 0` (aliased instead of solved).
    pub subst: &'a dyn Fn(EquationOwner) -> Option<SubstPairs>,
    /// Every entity with the parameters that move it (per-entity status).
    pub entities: &'a [(EntityId, SmallVec<[ParamId; 8]>)],
    /// Entities a constraint references (they are `Over` when it is in a group).
    pub constraint_entities: &'a dyn Fn(ConstraintId) -> SmallVec<[EntityId; 4]>,
}

impl Problem<'_> {
    fn structure(&self, solver: &NativeSolver) -> Structure {
        Structure::build(
            self.n,
            0,
            self.sys,
            self.subst,
            solver.dense_max,
            solver.decompose,
        )
    }
}

impl NativeSolver {
    /// Solves `p` starting from `x` and writes the solution back into `x` (unchanged when
    /// cancelled). The report's `driven` is empty.
    pub(crate) fn solve_problem(
        &mut self,
        p: &Problem<'_>,
        x: &mut [f64],
        opts: &SolveOptions,
        cancel: &dyn Fn() -> bool,
    ) -> SolveReport {
        if cancel() {
            return SolveReport::cancelled();
        }
        let st = p.structure(self);
        let tol = effective_tol(opts.tolerance, p.sys.length_scale);
        let Some(run) = self.run(p.sys, &st, x, &|_| 1.0, opts, tol, cancel) else {
            return SolveReport::cancelled();
        };
        let Some(out) = outcome(&p.sys.system, &st, &run.x, opts, tol, cancel) else {
            return SolveReport::cancelled();
        };
        let want_groups = opts.diagnose && out.status != SolveStatus::Converged;
        let analysis = if want_groups || opts.dof == DofAnalysis::PerEntity {
            match analyze_problem(p, &st, &run.x, cancel) {
                Some(a) => Some(a),
                None => return SolveReport::cancelled(),
            }
        } else {
            None
        };
        let (dof, diagnosis) = match (opts.dof, analysis) {
            (DofAnalysis::None, a) => (None, a.filter(|_| want_groups).map(|a| a.1)),
            (DofAnalysis::Total, a) => (
                out.dof.map(|dof| DofReport {
                    dof,
                    entities: BTreeMap::new(),
                }),
                a.filter(|_| want_groups).map(|a| a.1),
            ),
            (DofAnalysis::PerEntity, Some(a)) => (Some(a.0), want_groups.then_some(a.1)),
            (DofAnalysis::PerEntity, None) => (None, None),
        };
        for (i, s) in st.slot.iter().enumerate() {
            if *s != Slot::Unused {
                x[i] = run.x[i];
            }
        }
        SolveReport {
            status: out.status,
            iterations: run.iterations,
            max_residual: out.max_residual,
            unsatisfied: out.unsatisfied,
            driven: BTreeMap::new(),
            dof,
            diagnosis,
        }
    }

    /// DOF report and diagnosis of `p` at `x` (which should satisfy the equations; at an
    /// unsatisfied `x` the unsatisfied rows' circuits are reported as conflicts).
    pub(crate) fn analyze_problem(&self, p: &Problem<'_>, x: &[f64]) -> (DofReport, Diagnosis) {
        let st = p.structure(self);
        analyze_problem(p, &st, x, &|| false).unwrap_or_default()
    }
}

/// Per-entity DOF and circuit diagnosis; `None` if cancelled.
fn analyze_problem(
    p: &Problem<'_>,
    st: &Structure,
    x: &[f64],
    cancel: &dyn Fn() -> bool,
) -> Option<(DofReport, Diagnosis)> {
    let system = &p.sys.system;
    let tol = effective_tol(SolveOptions::default().tolerance, p.sys.length_scale);
    let sa: SubAnalysis = substituted(system, st, x, cancel)?;
    let raw = RawSystem::with_fixes(p.n, system, &p.sys.free, &BTreeMap::new(), &BTreeMap::new());
    let rv = raw.at(system, x);
    let unsat: Vec<bool> = rv.f.iter().map(|v| v.abs() > tol || v.is_nan()).collect();

    // Suspect raw rows (as the sketch path): unsatisfied, substitution-redundant or constant
    // blocks, and rows touching a rank-deficient substituted component or a fixed join.
    let mut comp_of_class = vec![u32::MAX; st.unknowns()];
    for (ci, c) in st.components.iter().enumerate() {
        for &k in &c.unknowns {
            comp_of_class[k as usize] = ci as u32;
        }
    }
    let special: BTreeSet<u32> = st
        .redundant_subst
        .iter()
        .chain(&st.const_blocks)
        .copied()
        .collect();
    let joined: BTreeSet<u32> = st.fixed_joins.iter().flat_map(|j| [j.a, j.b]).collect();
    let param_suspect = |i: usize| match st.slot.get(i) {
        Some(Slot::Unknown(k)) => {
            let ci = comp_of_class[*k as usize];
            ci != u32::MAX && sa.deficient[ci as usize]
        }
        Some(Slot::Fixed { src }) => joined.contains(src) || joined.contains(&(i as u32)),
        _ => false,
    };
    let row_suspect = |i: usize| {
        let RowSrc::Res { block, .. } = raw.rows[i].src else {
            return false;
        };
        unsat[i]
            || special.contains(&block)
            || system
                .residual(block as usize)
                .params()
                .iter()
                .any(|q| param_suspect(q.index()))
    };

    let owners_of = |rows: &[usize]| rows.iter().map(|&i| raw.rows[i].owner).collect::<Vec<_>>();
    let mut groups = Vec::new();
    for rows in raw.components() {
        if !rows.iter().any(|&i| row_suspect(i)) {
            continue;
        }
        let circ = circuits(&rows, &grads_of(&rv, &rows), cancel)?;
        let mut conflict = false;
        for c in &circ {
            let kind = if c.iter().any(|&i| unsat[i]) {
                conflict = true;
                GroupKind::Conflicting
            } else {
                GroupKind::Redundant
            };
            groups.extend(group_of(kind, owners_of(c)));
        }
        if !conflict && rows.iter().any(|&i| unsat[i]) {
            let bad: Vec<usize> = rows.iter().copied().filter(|&i| unsat[i]).collect();
            groups.extend(group_of(GroupKind::Conflicting, owners_of(&bad)));
        }
    }
    let conflicts: Vec<BTreeSet<ConstraintId>> = groups
        .iter()
        .filter(|g| g.kind == GroupKind::Conflicting)
        .map(|g| g.constraints.iter().copied().collect())
        .collect();
    groups.retain(|g| {
        g.kind != GroupKind::Redundant
            || !conflicts
                .iter()
                .any(|c| g.constraints.iter().all(|id| c.contains(id)))
    });
    sort_groups(&mut groups);
    let over: BTreeSet<EntityId> = groups
        .iter()
        .flat_map(|g| g.constraints.iter())
        .flat_map(|&id| (p.constraint_entities)(id))
        .collect();
    let dof = u32::try_from(st.unknowns().saturating_sub(sa.rank)).unwrap_or(u32::MAX);
    Some((
        DofReport {
            dof,
            entities: sa.report_params(p.entities.iter().cloned(), st, &over),
        },
        Diagnosis { groups },
    ))
}
