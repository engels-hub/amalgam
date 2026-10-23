//! The corpus adapter for a production [`SolverBackend`] (SK-03): `solve` gives the status,
//! residual and iterations; DOF (when asked for in the options) and diagnosis groups are mapped
//! back to case indices with [`BuiltCase::constraint_index`].

use super::runner::{BackendOutcome, BackendSkip, CorpusBackend};
use super::{BuiltCase, Case, CaseStatus, ExpectedGroup, GroupKind};
use crate::solve::{self, SolveOptions, SolveReport, SolveStatus, SolverBackend};

/// Runs a [`SolverBackend`] with fixed options over corpus cases.
#[derive(Debug)]
pub struct SolverRunner<B: SolverBackend> {
    /// The backend (its structure cache persists across cases).
    pub backend: B,
    /// Options of every solve.
    pub opts: SolveOptions,
}

impl<B: SolverBackend> SolverRunner<B> {
    /// `backend` with total DOF analysis and diagnosis, so the runner checks both.
    pub fn new(backend: B) -> Self {
        SolverRunner {
            backend,
            opts: SolveOptions {
                dof: solve::DofAnalysis::Total,
                ..SolveOptions::default()
            },
        }
    }
}

/// Maps a solver status to the corpus status (`Cancelled` → `DidNotConverge`).
pub fn case_status(s: SolveStatus) -> CaseStatus {
    match s {
        SolveStatus::Converged => CaseStatus::Converged,
        SolveStatus::ConvergedRedundant => CaseStatus::ConvergedRedundant,
        SolveStatus::Inconsistent => CaseStatus::Inconsistent,
        SolveStatus::DidNotConverge | SolveStatus::Cancelled => CaseStatus::DidNotConverge,
    }
}

/// Diagnosis groups of `report` as case constraint indices.
pub fn case_groups(built: &BuiltCase, report: &SolveReport) -> Option<Vec<ExpectedGroup>> {
    let d = report.diagnosis.as_ref()?;
    Some(
        d.groups
            .iter()
            .filter_map(|g| {
                let mut cs: Vec<u32> = g
                    .constraints
                    .iter()
                    .filter_map(|&id| built.constraint_index(id))
                    .collect();
                cs.sort_unstable();
                cs.dedup();
                (!cs.is_empty()).then_some(ExpectedGroup {
                    kind: match g.kind {
                        solve::GroupKind::Redundant => GroupKind::Redundant,
                        solve::GroupKind::Conflicting => GroupKind::Conflicting,
                        solve::GroupKind::DegenerateConfiguration => {
                            GroupKind::DegenerateConfiguration
                        }
                    },
                    constraints: cs,
                })
            })
            .collect(),
    )
}

impl<B: SolverBackend> CorpusBackend for SolverRunner<B> {
    fn name(&self) -> String {
        self.backend.name().into()
    }

    fn solve(
        &mut self,
        _case: &Case,
        built: &mut BuiltCase,
    ) -> Result<BackendOutcome, BackendSkip> {
        let r = self.backend.solve(&mut built.sketch, &self.opts, &|| false);
        Ok(BackendOutcome {
            status: case_status(r.status),
            dof: r.dof.as_ref().map(|d| d.dof),
            max_residual: Some(r.max_residual),
            iterations: Some(r.iterations),
            groups: case_groups(built, &r),
        })
    }
}
