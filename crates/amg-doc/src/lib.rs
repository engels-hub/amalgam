//! Document model: design tree, timeline, parameters, persistent naming, undo/redo and the `.amg` format.
//!
//! Spec: S01. Lane: E.
//!
//! State is split (ADR-014):
//! - [`Design`] — authored state, persistent collections, snapshotted as [`DesignSnapshot`].
//! - [`Evaluated`] — recompute output (geometry, health), never undone.
//!
//! DOC-01 provides ids, the authored model, the evaluated skeleton, provisional naming types
//! and the [`stub`] placeholders for geometry types of other lanes.

#![warn(missing_docs)]

pub mod eval;
pub mod id;
pub mod model;
pub mod naming;
mod pcoll;
pub mod stub;

pub use eval::{
    BodyGeom, CacheKey, ConstructionGeom, Diagnostic, ErrorCause, EvalBody, Evaluated, Health,
    NodeEval,
};
pub use id::{
    BodyId, BodyRef, ComponentId, ConfigId, ConstructionId, ExtKey, FeatureId, GroupId, JointId,
    OccurrenceId, ParamId, Revision, SketchId,
};
pub use model::{
    BakedBody, BlobRef, BodyProps, Component, Design, DesignMeta, DesignMode, DesignSnapshot,
    ExtRecord, ID_LIMIT, ModelError, Occurrence, UnitPrefs,
};
pub use naming::{
    BodyNaming, EntityRef, GeomClass, GeomFingerprint, MatchQuality, NameSeg, NameSource,
    ResolveError, Resolved, Role, TopoKind, TopoName, TopoRef,
};
