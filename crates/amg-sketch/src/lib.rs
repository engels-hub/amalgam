//! 2D sketch entities, constraint solver and profile/region detection.
//!
//! Spec: S03. Lane: B.
//!
//! - [`ids`]: stable, never-reused `u32` ids ([`EntityId`], [`ConstraintId`], [`ParamId`],
//!   [`ExternalLinkId`]).
//! - [`model`]: the parameter vector ([`ParamStore`]), entities, constraints, dimensions and
//!   the [`Sketch`] container with kind validation and serde.
//! - [`geom`]: evaluated 2D curves ([`Curve2`]), curve queries and [`geom::intersect`].
//! - [`profile`]: profile/region detection ([`detect_profiles`], [`ProfileSet`]).
//! - [`error`]: the [`SketchError`] enum.
//! - [`solve`]: residual equations with analytic Jacobians (SK-02); the solver from SK-03.
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
pub mod profile;
pub mod solve;

pub use error::SketchError;
pub use geom::{Arc2, Circle2, Curve2, CurveHit};
pub use ids::{ConstraintId, EntityId, ExternalLinkId, ParamId};
pub use model::{
    AngleBranch, Constraint, ConstraintKind, Dimension, DimensionKind, End, Entity, EntityKind,
    EntityOrigin, HvTarget, ParamFlags, ParamStore, Quantity, RemoveReport, Side, Sketch,
    TangentKind,
};
pub use profile::{
    Loop, Profile, ProfileEdge, ProfileFingerprint, ProfileId, ProfileIssue, ProfileOptions,
    ProfileSet, VertexKey, detect_profiles,
};

/// Evaluated geometry is keyed by entity id (replaces S01's `amg_doc::stub::SketchEntityKey`).
pub type SketchEntityKey = EntityId;

/// Profile key used by S01 `EntityRef::Profile` (replaces `amg_doc::stub::ProfileKey`).
pub type ProfileKey = ProfileId;
