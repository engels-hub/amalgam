//! Constraint equations and (from SK-03) the solver.
//!
//! - [`system`]: the dimension-agnostic layer ([`Residual`], [`EquationSystem`]).
//! - Residuals of sketch entities, constraints and dimensions with analytic Jacobians
//!   ([`constraint_residual`], [`entity_residual`], [`build_system`]) — SK-02.
//! - Chirality capture ([`orient_constraint`], `Sketch::add_constraint_oriented`).
//! - Dense rank / DOF helpers for tests and builders ([`numeric_rank`], [`dof`]).
//! - The production solver (SK-03, ADR-007): [`NativeSolver`] behind the [`SolverBackend`]
//!   trait, with [`SolveOptions`] / [`SolveReport`] (substitution, components,
//!   Dulmage–Mendelsohn blocks, dense/sparse damped minimum-norm Gauss–Newton/LM, structure
//!   cache, cancellation, driven-dimension measurement).
//! - [`proto`]: the SK-13 prototype solver (ADR-007 benchmark), kept for the comparison
//!   harness.
//!
//! DOF analysis and diagnosis (SK-04) and dragging (SK-05) build on these. The residual
//! definitions do not depend on the ADR-007 solver choice.

mod api;
mod func;
mod native;
mod orient;
pub mod proto;
mod rank;
mod residual;
mod spline;
pub mod system;

pub use api::{
    Algorithm, ConstraintCheck, ConstraintState, DependencyGroup, Diagnosis, DofAnalysis,
    DofReport, DragSession, DragSpec, DragStep, EntityStatus, GroupKind, SolveOptions, SolveReport,
    SolveStatus, SolverBackend,
};
pub use native::{DENSE_MAX, NativeSolver, SolveStats};
pub use orient::orient_constraint;
pub use rank::{dof, numeric_rank};
pub(crate) use residual::candidate_max_residual;
pub use residual::{
    SketchResidual, SketchSystem, build_system, build_system_scaled, characteristic_length,
    constraint_residual, constraint_residual_aux, entity_residual, fixed_params,
};
pub use system::{EquationOwner, EquationSystem, Residual};
