//! The sketch session's solver worker (spec §13 *Threading*).
//!
//! [`SolverState`] holds the backend (with its structure and analysis caches), its copy of the
//! sketch and an open [`DragSession`]. Every solver call runs in an `amg-jobs` job that takes
//! the state **by value** and hands it back with the result ([`SolverJob`]), so at most one
//! call is in flight per session and none blocks the UI thread. Cursor targets of a drag are
//! coalesced by the session (latest wins).

use std::borrow::Cow;
use std::sync::Arc;

use amg_core::Point2;
use amg_jobs::{Cancelled, Job, JobCtx};
use amg_sketch::solve::{
    ConstraintCheck, DofAnalysis, DragSession, DragSpec, DragStep, NativeSolver, SolveOptions,
    SolveReport, SolveStatus, SolverBackend,
};
use amg_sketch::{ConstraintKind, ProfileOptions, ProfileSet, Sketch, detect_profiles};

/// Solver backend, its sketch copy and the open drag (moved into each job and back).
pub struct SolverState {
    /// The backend (SK-03..05), caches kept across calls.
    pub solver: NativeSolver,
    /// The solver's copy of the sketch (solved values after a solve or drag frame).
    pub sketch: Sketch,
    /// The open drag, if any.
    pub drag: Option<Box<dyn DragSession>>,
}

impl std::fmt::Debug for SolverState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SolverState")
            .field("dragging", &self.drag.is_some())
            .finish_non_exhaustive()
    }
}

impl SolverState {
    /// A fresh backend on `sketch`.
    pub fn new(sketch: Sketch) -> SolverState {
        SolverState {
            solver: NativeSolver::new(),
            sketch,
            drag: None,
        }
    }
}

/// What a [`SolverJob`] does.
#[derive(Debug)]
pub enum SolverOp {
    /// Replace the state's sketch with this one, solve it with per-entity DOF and detect its
    /// profiles.
    Solve(Box<Sketch>),
    /// Check the constraints in order on this sketch (each accepted one is added before the
    /// next is checked); see [`SolverOut::Checked`].
    Check(Box<Sketch>, Vec<ConstraintKind>),
    /// Start a drag (if `begin` is set) and move it to `target`.
    DragStep {
        /// The grab of a new drag; `None` continues the open one.
        begin: Option<DragSpec>,
        /// Cursor target in sketch coordinates.
        target: Point2,
    },
    /// Finish the open drag (a full solve).
    DragFinish,
}

/// The result of a [`SolverJob`].
#[derive(Debug)]
pub enum SolverOut {
    /// A solve: its report (DOF per entity) and the profiles of the solved sketch.
    Solved {
        /// The solve report; `dof` is filled.
        report: SolveReport,
        /// Profiles of the solved sketch (SK-06).
        profiles: Arc<ProfileSet>,
    },
    /// Per checked constraint: `true` if it was accepted (independent).
    Checked(Vec<bool>),
    /// One drag frame (`Err`: the drag could not start).
    DragStepped(Result<DragStep, amg_sketch::SketchError>),
    /// The drag finished; the state's sketch holds the final values.
    DragFinished(SolveReport),
}

/// Error of a solver job (only cancellation).
#[derive(Debug, thiserror::Error)]
pub enum SolverJobError {
    /// The job was cancelled.
    #[error("cancelled")]
    Cancelled,
}

impl From<Cancelled> for SolverJobError {
    fn from(_: Cancelled) -> Self {
        SolverJobError::Cancelled
    }
}

impl amg_core::Diagnostic for SolverJobError {
    fn user_message(&self) -> String {
        "The sketch solve was cancelled".to_owned()
    }
    fn severity(&self) -> amg_core::Severity {
        amg_core::Severity::Info
    }
    fn is_cancelled(&self) -> bool {
        true
    }
}

/// One solver call on the worker pool.
#[derive(Debug)]
pub struct SolverJob {
    /// The state, returned with the output.
    pub state: SolverState,
    /// The call.
    pub op: SolverOp,
}

/// Options of the session solve: per-entity DOF (SK-04) and diagnosis.
pub fn solve_options() -> SolveOptions {
    SolveOptions {
        dof: DofAnalysis::PerEntity,
        ..SolveOptions::default()
    }
}

impl Job for SolverJob {
    type Output = (SolverState, SolverOut);
    type Error = SolverJobError;

    fn label(&self) -> Cow<'static, str> {
        "Sketch solve".into()
    }

    fn run(self, ctx: &JobCtx) -> Result<Self::Output, Self::Error> {
        let SolverJob { mut state, op } = self;
        let cancel = || ctx.is_cancelled();
        let out = match op {
            SolverOp::Solve(sketch) => {
                state.drag = None;
                state.sketch = *sketch;
                let report = state
                    .solver
                    .solve(&mut state.sketch, &solve_options(), &cancel);
                if report.status == SolveStatus::Cancelled {
                    return Err(SolverJobError::Cancelled);
                }
                for (id, v) in &report.driven {
                    // Measured values of driven dimensions are display-only.
                    let _ = state.sketch.set_dimension_value(*id, *v);
                }
                let profiles = Arc::new(detect_profiles(&state.sketch, &ProfileOptions::default()));
                SolverOut::Solved { report, profiles }
            }
            SolverOp::Check(mut sketch, candidates) => {
                let mut accepted = Vec::with_capacity(candidates.len());
                for kind in candidates {
                    ctx.checkpoint()?;
                    let ok = matches!(
                        state.solver.check_constraint(&sketch, &kind),
                        Ok(ConstraintCheck::Independent)
                    ) && sketch.add_constraint(kind).is_ok();
                    accepted.push(ok);
                }
                SolverOut::Checked(accepted)
            }
            SolverOp::DragStep { begin, target } => {
                if let Some(spec) = begin {
                    state.drag = None;
                    match state.solver.begin_drag(&state.sketch, spec) {
                        Ok(d) => state.drag = Some(d),
                        Err(e) => return Ok((state, SolverOut::DragStepped(Err(e)))),
                    }
                }
                match state.drag.as_mut() {
                    Some(d) => {
                        let step = d.step(&mut state.sketch, target, &cancel);
                        SolverOut::DragStepped(Ok(step))
                    }
                    None => SolverOut::DragStepped(Err(amg_sketch::SketchError::NotApplicable(
                        "no drag is open",
                    ))),
                }
            }
            SolverOp::DragFinish => {
                let report = match state.drag.take() {
                    Some(d) => d.finish(&mut state.sketch),
                    None => state
                        .solver
                        .solve(&mut state.sketch, &solve_options(), &cancel),
                };
                SolverOut::DragFinished(report)
            }
        };
        Ok((state, out))
    }
}
