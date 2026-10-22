//! Document model: design tree, timeline, parameters, persistent naming, undo/redo and the `.amg` format.
//!
//! Spec: S01. Lane: E.
//!
//! State is split (ADR-014):
//! - [`Design`] — authored state, persistent collections, snapshotted as [`DesignSnapshot`].
//! - [`Evaluated`] — recompute output (geometry, health), never undone.
//!
//! DOC-01 provides ids, the authored model, the evaluated skeleton, provisional naming types
//! and the [`stub`] placeholders for geometry types of other lanes. DOC-02 adds the [`Feature`]
//! trait, the [`registry`] (`register_feature!`) and the [`Timeline`].

#![warn(missing_docs)]

pub mod cache;
pub mod command;
pub mod document;
pub mod eval;
pub mod expr;
pub mod feature;
pub mod history;
pub mod id;
pub mod model;
pub mod naming;
pub mod param;
mod pcoll;
pub mod recompute;
pub mod registry;
pub mod stub;
#[cfg(any(test, feature = "test-features"))]
pub mod testing;
pub mod timeline;

pub use cache::{CacheStats, GeomCache};
pub use command::{Command, CommandError, CommandOutcome, CreatedId, apply_command};
pub use document::{
    DEFAULT_CACHE_BUDGET, Document, RecomputeError, RecomputeJob, RecomputeStatus, RecomputeSummary,
};
pub use eval::{
    BodyGeom, CacheKey, ConstructionGeom, Diagnostic, ErrorCause, EvalBody, Evaluated, Health,
    NodeEval,
};
pub use expr::{Dim, ExprError};
pub use feature::{
    BodyChange, BodyChangeKind, ComputeMode, Dependency, Feature, FeatureBox, FeatureDataError,
    FeatureError, FeatureMeta, FeatureOutput, MigrationError, RecomputeCtx, UnknownFeature,
};
pub use history::{DEFAULT_UNDO_DEPTH, History, Transaction, UndoEntry, UndoStack};
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
pub use param::{Expr, ExprSrc, Param, ParamError, ParamKind, ParamTable, UnitSpec};
pub use recompute::{ENGINE_FORMAT_VERSION, RecomputeResult, engine_salt, recompute};
pub use registry::{FeatureRegistration, FeatureRegistry, RegistryError, registry, try_registry};
pub use timeline::{FeatureNode, Group, Timeline};

/// Not public API: paths used by `register_feature!`.
#[doc(hidden)]
pub mod __private {
    pub use inventory;
    pub use serde_json;
}
