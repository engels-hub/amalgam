//! Public solver types (S03 Interfaces, `amg_sketch::solve`): options, status, reports and the
//! [`SolverBackend`] / [`DragSession`] traits. [`NativeSolver`](super::NativeSolver) is the
//! shipped backend (ADR-007).

use std::collections::BTreeMap;

use amg_core::Point2;

use crate::error::SketchError;
use crate::ids::{ConstraintId, EntityId};
use crate::model::{ConstraintKind, Sketch};

/// Nonlinear iteration used per block.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Algorithm {
    /// The default: [`LevenbergMarquardt`](Algorithm::LevenbergMarquardt) (SK-13 finding:
    /// dogleg was equal or worse on every decision case).
    Auto,
    /// Undamped minimum-norm Gauss–Newton with step halving.
    GaussNewtonMinNorm,
    /// Damped weighted minimum-norm Gauss–Newton; `λ` starts at 0 (first step pure
    /// Gauss–Newton) and adapts ×10 / ÷10 on rejected / accepted steps.
    LevenbergMarquardt,
    /// Powell dogleg with the minimum-norm Gauss–Newton step (PlaneGCS default).
    DogLeg,
}

/// How much DOF analysis a solve runs.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DofAnalysis {
    /// None (the report's `dof` is `None`).
    None,
    /// Total DOF only.
    Total,
    /// Total DOF plus a status per entity (SK-04).
    PerEntity,
}

/// Options of [`SolverBackend::solve`].
#[derive(Clone, Debug)]
pub struct SolveOptions {
    /// Iteration scheme, default [`Algorithm::Auto`].
    pub algorithm: Algorithm,
    /// Max `|residual|` in mm (angular rows are scaled by `L_c`), default `1e-10`. The solver
    /// uses `max(tolerance, 64 ε L_c)` so huge sketches stay reachable in `f64`.
    pub tolerance: f64,
    /// Accepted steps per block, default 50.
    pub max_iterations: u32,
    /// DOF analysis, default [`DofAnalysis::None`].
    pub dof: DofAnalysis,
    /// Run diagnosis when the result is not `Converged`; default true. The numeric rank test
    /// that tells `Converged` from `ConvergedRedundant` runs when this is set or `dof` is not
    /// `None`; without it only structural redundancy is detected.
    pub diagnose: bool,
}

impl Default for SolveOptions {
    fn default() -> Self {
        SolveOptions {
            algorithm: Algorithm::Auto,
            tolerance: 1e-10,
            max_iterations: 50,
            dof: DofAnalysis::None,
            diagnose: true,
        }
    }
}

impl SolveOptions {
    /// Solve only: no rank test, no DOF, no diagnosis (warm solves and drag frames).
    pub fn fast() -> Self {
        SolveOptions {
            diagnose: false,
            ..Self::default()
        }
    }
}

/// Outcome of a solve.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SolveStatus {
    /// Every equation holds and the equations are independent (or the rank test was off).
    Converged,
    /// Every equation holds; some equations are dependent (redundant constraints).
    ConvergedRedundant,
    /// Not satisfied and dependent equations exist: contradictory constraints.
    Inconsistent,
    /// Not satisfied and no dependency found.
    DidNotConverge,
    /// The cancel callback returned true; the sketch is unchanged.
    Cancelled,
}

impl SolveStatus {
    /// `Converged` or `ConvergedRedundant`.
    pub fn is_converged(self) -> bool {
        matches!(
            self,
            SolveStatus::Converged | SolveStatus::ConvergedRedundant
        )
    }
}

/// Constraint state of one entity (UI colour).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ConstraintState {
    /// Some parameter can still move (blue).
    Under,
    /// Fully constrained (black).
    Full,
    /// Part of a redundancy or conflict group (red).
    Over,
}

/// Status of one entity.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct EntityStatus {
    /// State.
    pub state: ConstraintState,
    /// Independent ways the entity can move: the rank of the null-space rows of its own
    /// params and its points' params (a line: up to 4, an arc: up to 5).
    pub free_dof: u8,
}

/// DOF analysis result.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct DofReport {
    /// Free unknowns minus the Jacobian rank.
    pub dof: u32,
    /// Per-entity status (filled for [`DofAnalysis::PerEntity`] by SK-04).
    pub entities: BTreeMap<EntityId, EntityStatus>,
}

/// Kind of a dependency group.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum GroupKind {
    /// Dependent and satisfied.
    Redundant,
    /// Dependent and contradictory.
    Conflicting,
    /// Dependent only at the current (degenerate) configuration (SK-04 witness tie-break).
    DegenerateConfiguration,
}

/// A minimal set of constraints whose equations are linearly dependent.
#[derive(Clone, Debug, PartialEq)]
pub struct DependencyGroup {
    /// Kind.
    pub kind: GroupKind,
    /// Constraints, sorted and distinct.
    pub constraints: Vec<ConstraintId>,
}

/// Dependency groups found by diagnosis.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Diagnosis {
    /// Groups, sorted by kind then constraints.
    pub groups: Vec<DependencyGroup>,
}

/// Result of [`SolverBackend::solve`].
#[derive(Clone, Debug)]
pub struct SolveReport {
    /// Status.
    pub status: SolveStatus,
    /// Linear solves, summed over all blocks.
    pub iterations: u32,
    /// Largest residual of the whole (unsubstituted) system at the end, mm.
    pub max_residual: f64,
    /// Constraints with a residual row above the tolerance, sorted.
    pub unsatisfied: Vec<ConstraintId>,
    /// Measured values of driven dimensions (mm or radians).
    pub driven: BTreeMap<ConstraintId, f64>,
    /// DOF analysis, when asked for.
    pub dof: Option<DofReport>,
    /// Diagnosis, when asked for and the result is not `Converged`.
    pub diagnosis: Option<Diagnosis>,
}

impl SolveReport {
    pub(crate) fn cancelled() -> Self {
        SolveReport {
            status: SolveStatus::Cancelled,
            iterations: 0,
            max_residual: f64::NAN,
            unsatisfied: Vec::new(),
            driven: BTreeMap::new(),
            dof: None,
            diagnosis: None,
        }
    }
}

/// Result of [`SolverBackend::check_constraint`].
#[derive(Clone, Debug, PartialEq)]
pub enum ConstraintCheck {
    /// The candidate removes DOF.
    Independent,
    /// The candidate is implied by these constraints (and satisfied).
    Redundant {
        /// The other members of the dependency group.
        with: Vec<ConstraintId>,
    },
    /// The candidate contradicts these constraints.
    Conflicting {
        /// The other members of the dependency group.
        with: Vec<ConstraintId>,
    },
}

/// What a drag grabs.
#[derive(Copy, Clone, Debug)]
pub struct DragSpec {
    /// The grabbed entity.
    pub entity: EntityId,
    /// Grab point in sketch coordinates.
    pub grab: Point2,
}

/// Result of one drag frame.
#[derive(Copy, Clone, Debug)]
pub struct DragStep {
    /// Status after the frame.
    pub status: SolveStatus,
    /// Linear solves used.
    pub iterations: u32,
    /// Largest residual after the frame, mm.
    pub max_residual: f64,
}

/// How a drag frame is solved (SK-05, ADR-007 near-singular decision).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub enum DragMode {
    /// Weighted minimum norm per frame; a frame that does not converge, or that moves the
    /// non-dragged geometry far more than the cursor moved (a branch jump near a singular
    /// configuration), is redone by feasible SQP from the previous frame. The default.
    #[default]
    Auto,
    /// Weighted minimum norm only (the SolveSpace approach; flips branches near toggles).
    MinNorm,
    /// Feasible SQP only: the cursor is the objective, the constraints stay satisfied.
    Sqp,
}

/// Counters of a drag session ([`NativeDrag::stats`](super::NativeDrag::stats)).
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct DragStats {
    /// Frames solved.
    pub frames: u32,
    /// Frames redone (or solved) by feasible SQP.
    pub sqp_frames: u32,
    /// Frames whose result did not satisfy the equations (the sketch keeps the last good state).
    pub failed_frames: u32,
}

/// An interactive drag (SK-05).
pub trait DragSession: Send {
    /// Moves the grabbed point toward `target` (sketch coords) and re-solves (SK-05).
    fn step(&mut self, sketch: &mut Sketch, target: Point2, cancel: &dyn Fn() -> bool) -> DragStep;
    /// Final full solve; caller turns the parameter delta into `SketchEdit::CommitDrag`.
    fn finish(self: Box<Self>, sketch: &mut Sketch) -> SolveReport;
}

/// A sketch solver. All entry points are synchronous and never run on the UI thread.
pub trait SolverBackend: Send {
    /// Backend name.
    fn name(&self) -> &'static str;
    /// Solves `sketch` in place from its current values (SK-03).
    fn solve(
        &mut self,
        sketch: &mut Sketch,
        opts: &SolveOptions,
        cancel: &dyn Fn() -> bool,
    ) -> SolveReport;
    /// DOF and dependency groups at the current values (SK-04).
    fn analyze(&mut self, sketch: &Sketch) -> (DofReport, Diagnosis);
    /// Would adding `candidate` over-constrain the sketch? (SK-04)
    fn check_constraint(
        &mut self,
        sketch: &Sketch,
        candidate: &ConstraintKind,
    ) -> Result<ConstraintCheck, SketchError>;
    /// Starts dragging (SK-05).
    fn begin_drag(
        &mut self,
        sketch: &Sketch,
        spec: DragSpec,
    ) -> Result<Box<dyn DragSession>, SketchError>;
}
