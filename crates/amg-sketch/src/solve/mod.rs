//! Constraint equations and (from SK-03) the solver.
//!
//! - [`system`]: the dimension-agnostic layer ([`Residual`], [`EquationSystem`]).
//! - Residuals of sketch entities, constraints and dimensions with analytic Jacobians
//!   ([`constraint_residual`], [`entity_residual`], [`build_system`]) — SK-02.
//! - Chirality capture ([`orient_constraint`], `Sketch::add_constraint_oriented`).
//! - Dense rank / DOF helpers for tests and builders ([`numeric_rank`], [`dof`]).
//! - [`proto`]: the SK-13 prototype solver (ADR-007 benchmark; SK-03 replaces it).
//!
//! The solver itself (SK-03), DOF analysis and diagnosis (SK-04) and dragging (SK-05) build on
//! these. The residual definitions do not depend on the ADR-007 solver choice.

mod func;
mod orient;
pub mod proto;
mod rank;
mod residual;
pub mod system;

pub use orient::orient_constraint;
pub use rank::{dof, numeric_rank};
pub(crate) use residual::candidate_max_residual;
pub use residual::{
    SketchResidual, SketchSystem, build_system, build_system_scaled, characteristic_length,
    constraint_residual, constraint_residual_aux, entity_residual, fixed_params,
};
pub use system::{EquationOwner, EquationSystem, Residual};
