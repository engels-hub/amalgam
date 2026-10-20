//! 2D sketch entities, constraint solver and profile/region detection.
//!
//! Spec: S03. Lane: B.
//!
//! - [`ids`]: stable, never-reused `u32` ids ([`EntityId`], [`ConstraintId`], [`ParamId`],
//!   [`ExternalLinkId`]).
//! - [`model`]: the parameter vector ([`ParamStore`]), entities, constraints, dimensions and
//!   the [`Sketch`] container with kind validation and serde.
//! - [`geom`]: evaluated 2D curves ([`Curve2`]).
//! - [`error`]: the [`SketchError`] enum.
//!
//! Lengths are mm in sketch-plane coordinates, angles are radians, all `f64`. Collections are
//! `BTreeMap`s, so iteration order (and serialised output) is deterministic.
//!
//! The solver (SK-02..05) and profile detection (SK-06) build on this data model.

#![warn(missing_docs)]

pub mod error;
pub mod geom;
pub mod ids;
pub mod model;

pub use error::SketchError;
pub use geom::{Arc2, Circle2, Curve2};
pub use ids::{ConstraintId, EntityId, ExternalLinkId, ParamId};
pub use model::{
    AngleBranch, Constraint, ConstraintKind, Dimension, DimensionKind, Entity, EntityKind,
    EntityOrigin, HvTarget, ParamFlags, ParamStore, Quantity, RemoveReport, Side, Sketch,
    TangentKind,
};

/// Evaluated geometry is keyed by entity id (replaces S01's `amg_doc::stub::SketchEntityKey`).
pub type SketchEntityKey = EntityId;
