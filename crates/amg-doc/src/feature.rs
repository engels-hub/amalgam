//! Timeline features (S01 §2): the [`Feature`] trait, [`FeatureBox`], dependencies, outputs.
//!
//! A feature type is a plain `Serialize + Deserialize + Clone` struct that implements
//! [`Feature`] by hand and is registered with [`register_feature!`](crate::register_feature),
//! which supplies the [`FeatureMeta`] half (kind, schema version, JSON, cloning).
//!
//! On disk a feature is `{"kind": "...", "v": N, "data": {...}}`. Loading goes through the
//! [registry](crate::registry): a known kind is migrated to the current version and
//! deserialized; a kind this build doesn't know, or a newer version of a known kind, becomes an
//! [`UnknownFeature`] that keeps the raw data and saves it back unchanged.
//!
//! Feature structs must not contain hash maps (use `BTreeMap`) so their JSON is deterministic.
//! Non-finite floats can't be saved (JSON has no NaN); the design rejects a feature whose JSON
//! doesn't round-trip (see [`Design::add_feature`](crate::Design::add_feature)).

use std::any::Any;
use std::fmt;
use std::sync::Arc;

use amg_core::Tolerance;
use serde::de::Error as _;
use serde::ser::Error as _;
use serde::{Deserialize, Deserializer, Serialize, Serializer};

use crate::eval::{BodyGeom, ConstructionGeom, Diagnostic, Evaluated};
use crate::id::{BodyId, ComponentId, ConstructionId, FeatureId, OccurrenceId, SketchId};
use crate::model::Design;
use crate::naming::{BodyNaming, EntityRef, ResolveError};
use crate::registry::registry;
use crate::stub::SketchGeom;

/// The registration half of a feature type, implemented by
/// [`register_feature!`](crate::register_feature). Don't implement it by hand.
pub trait FeatureMeta {
    /// Registered kind, e.g. `"solid.extrude"`. Stable forever; used in files.
    ///
    /// `&str` rather than `&'static str` so [`UnknownFeature`] can report the kind it read.
    fn kind(&self) -> &str;
    /// Schema version of this kind's `data` (the registered current version).
    fn version(&self) -> u32;
    /// The feature's `data` as JSON (`serde_json` of `Self`).
    fn to_json(&self) -> Result<serde_json::Value, serde_json::Error>;
    /// For downcasting to the concrete type.
    fn as_any(&self) -> &dyn Any;
    /// A copy behind a new `Arc` (editing replaces the node's `Arc`, S01 §2).
    fn clone_arc(&self) -> Arc<dyn Feature>;
}

/// One timeline feature. Immutable once in a design; an edit replaces the whole value.
///
/// Parameter expressions (`exprs` / `exprs_mut`) join this trait with DOC-04 as defaulted
/// methods, and the engine-facing context grows with DOC-03.
pub trait Feature: FeatureMeta + Send + Sync + fmt::Debug + Any {
    /// Everything this feature reads. Every producing feature must come earlier in the
    /// timeline (ordering rule, S01 §2).
    fn inputs(&self) -> Vec<Dependency>;

    /// Checks field values (finite, in range). Called before a feature enters a design and
    /// when a design is loaded, so an invalid feature is never stored.
    fn validate(&self) -> Result<(), FeatureError> {
        Ok(())
    }

    /// Computes the outputs from the state just before this feature.
    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError>;
}

/// Shared handle to a feature; serializes through the registry as `{"kind", "v", "data"}`.
#[derive(Clone)]
pub struct FeatureBox(pub Arc<dyn Feature>);

impl FeatureBox {
    /// Boxes a registered feature.
    pub fn new<F: Feature>(feature: F) -> FeatureBox {
        FeatureBox(Arc::new(feature))
    }

    /// The feature.
    pub fn get(&self) -> &dyn Feature {
        &*self.0
    }

    /// Registered kind.
    pub fn kind(&self) -> &str {
        self.0.kind()
    }

    /// Schema version of the data.
    pub fn version(&self) -> u32 {
        self.0.version()
    }

    /// The concrete feature, if it is a `T`.
    pub fn downcast_ref<T: Feature>(&self) -> Option<&T> {
        self.0.as_any().downcast_ref::<T>()
    }

    /// The preserved raw data, if this kind (or version) is unknown to this build.
    pub fn as_unknown(&self) -> Option<&UnknownFeature> {
        self.downcast_ref::<UnknownFeature>()
    }

    /// True if this is an [`UnknownFeature`].
    pub fn is_unknown(&self) -> bool {
        self.as_unknown().is_some()
    }

    /// Builds a feature from its file parts through the registry.
    ///
    /// - Unknown kind, or `version` newer than registered → [`UnknownFeature`] (data kept).
    /// - Older `version` → the registration's `migrate` runs first.
    /// - Data that doesn't deserialize, or a failed migration → error.
    pub fn from_parts(
        kind: &str,
        version: u32,
        data: serde_json::Value,
    ) -> Result<FeatureBox, FeatureDataError> {
        let unknown = || {
            FeatureBox::new(UnknownFeature {
                kind: kind.to_owned(),
                version,
                raw: data.clone(),
            })
        };
        let Some(reg) = registry().get(kind) else {
            return Ok(unknown());
        };
        if version > reg.version {
            return Ok(unknown());
        }
        let data = if version < reg.version {
            (reg.migrate)(version, data.clone()).map_err(|e| FeatureDataError::Migration {
                kind: kind.to_owned(),
                from: version,
                to: reg.version,
                msg: e.0,
            })?
        } else {
            data.clone()
        };
        let feature = (reg.deserialize)(data).map_err(|e| FeatureDataError::Invalid {
            kind: kind.to_owned(),
            version: reg.version,
            msg: e.to_string(),
        })?;
        debug_assert_eq!(
            feature.kind(),
            kind,
            "register_feature! keeps kinds consistent"
        );
        Ok(FeatureBox(feature))
    }
}

impl std::ops::Deref for FeatureBox {
    type Target = dyn Feature;
    fn deref(&self) -> &dyn Feature {
        &*self.0
    }
}

impl fmt::Debug for FeatureBox {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Debug::fmt(&*self.0, f)
    }
}

impl PartialEq for FeatureBox {
    /// Same kind, version and JSON data (or the same allocation).
    fn eq(&self, other: &FeatureBox) -> bool {
        if Arc::ptr_eq(&self.0, &other.0) {
            return true;
        }
        self.kind() == other.kind()
            && self.version() == other.version()
            && matches!((self.0.to_json(), other.0.to_json()), (Ok(a), Ok(b)) if a == b)
    }
}

/// Wire form of a feature: `{"kind", "v", "data"}` (the same three fields appear inline in a
/// timeline node).
#[derive(Serialize)]
struct WireOut<'a> {
    kind: &'a str,
    v: u32,
    data: serde_json::Value,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct WireIn {
    kind: String,
    v: u32,
    data: serde_json::Value,
}

impl Serialize for FeatureBox {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        let data = self.0.to_json().map_err(S::Error::custom)?;
        WireOut {
            kind: self.kind(),
            v: self.version(),
            data,
        }
        .serialize(s)
    }
}

impl<'de> Deserialize<'de> for FeatureBox {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<FeatureBox, D::Error> {
        let w = WireIn::deserialize(d)?;
        FeatureBox::from_parts(&w.kind, w.v, w.data).map_err(D::Error::custom)
    }
}

/// Why a feature's file data could not be turned into a feature.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
#[non_exhaustive]
pub enum FeatureDataError {
    /// The data doesn't match the registered type.
    #[error("invalid data for feature `{kind}` v{version}: {msg}")]
    Invalid {
        /// Feature kind.
        kind: String,
        /// Schema version the data was read as.
        version: u32,
        /// serde's message.
        msg: String,
    },
    /// A per-kind migration step failed.
    #[error("migrating feature `{kind}` v{from}→v{to} failed: {msg}")]
    Migration {
        /// Feature kind.
        kind: String,
        /// Version in the file.
        from: u32,
        /// Current version.
        to: u32,
        /// The migration's message.
        msg: String,
    },
}

impl amg_core::Diagnostic for FeatureDataError {}

/// Error of a document or per-feature migration step (S01 §8).
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
#[error("migration failed: {0}")]
pub struct MigrationError(pub String);

/// A feature whose kind (or schema version) this build doesn't know. Keeps the raw data so the
/// file saves back unchanged; recompute gives it `Health::Error(UnknownKind)` (DOC-03).
///
/// Its dependencies are unknown, so it can't be moved (see `Timeline::valid_move_range`).
/// Only loading creates one; the design refuses to add or edit unknown features.
#[derive(Clone, Debug, PartialEq)]
pub struct UnknownFeature {
    /// Kind as written in the file.
    pub kind: String,
    /// Schema version as written in the file.
    pub version: u32,
    /// The `data` object, untouched.
    pub raw: serde_json::Value,
}

impl FeatureMeta for UnknownFeature {
    fn kind(&self) -> &str {
        &self.kind
    }
    fn version(&self) -> u32 {
        self.version
    }
    fn to_json(&self) -> Result<serde_json::Value, serde_json::Error> {
        Ok(self.raw.clone())
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn clone_arc(&self) -> Arc<dyn Feature> {
        Arc::new(self.clone())
    }
}

impl Feature for UnknownFeature {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }
    fn compute(&self, _ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Err(FeatureError::UnknownKind {
            kind: self.kind.clone(),
            version: self.version,
        })
    }
}

/// Something a feature reads.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Dependency {
    /// A body, in its state just before the reading feature.
    Body(BodyId),
    /// A sub-entity or other referenceable entity.
    Entity(EntityRef),
    /// A solved sketch.
    Sketch(SketchId),
    /// Construction geometry.
    Construction(ConstructionId),
    /// An occurrence (its placement).
    Occurrence(OccurrenceId),
    /// Any output of a feature.
    Feature(FeatureId),
}

impl Dependency {
    /// The feature whose output this reads (`None` for an occurrence).
    pub fn producer(&self) -> Option<FeatureId> {
        match self {
            Dependency::Body(b) => Some(b.origin),
            Dependency::Entity(e) => Some(entity_producer(e)),
            Dependency::Sketch(s) => Some(s.0),
            Dependency::Construction(c) => Some(c.0),
            Dependency::Occurrence(_) => None,
            Dependency::Feature(f) => Some(*f),
        }
    }

    /// Occurrences this reads (directly or on an in-context path).
    pub fn occurrences(&self) -> Vec<OccurrenceId> {
        match self {
            Dependency::Occurrence(o) => vec![*o],
            Dependency::Entity(e) => {
                let mut out = Vec::new();
                let mut e = e;
                while let EntityRef::InContext { path, target } = e {
                    out.extend_from_slice(path);
                    e = target;
                }
                out
            }
            _ => Vec::new(),
        }
    }
}

/// The feature that produced the entity an `EntityRef` points at.
fn entity_producer(e: &EntityRef) -> FeatureId {
    match e {
        EntityRef::Body(b) => b.origin,
        EntityRef::Topo(t) => t.body.origin,
        EntityRef::SketchCurve { sketch, .. }
        | EntityRef::SketchPoint { sketch, .. }
        | EntityRef::Profile { sketch, .. } => sketch.0,
        EntityRef::Construction(c) => c.0,
        EntityRef::InContext { target, .. } => entity_producer(target),
    }
}

/// What a feature computed.
#[derive(Clone, Debug, Default)]
pub struct FeatureOutput {
    /// Bodies created, modified or deleted.
    pub bodies: Vec<BodyChange>,
    /// Solved sketch (sketch features).
    pub sketch: Option<Arc<SketchGeom>>,
    /// Construction geometry (construction features).
    pub construction: Option<ConstructionGeom>,
    /// Non-fatal issues (health `Warning`).
    pub warnings: Vec<Diagnostic>,
}

/// One body affected by a feature.
#[derive(Clone, Debug)]
pub struct BodyChange {
    /// The body.
    pub id: BodyId,
    /// What happened to it.
    pub change: BodyChangeKind,
}

/// How a feature changed a body.
#[derive(Clone, Debug)]
pub enum BodyChangeKind {
    /// A new body (`id.origin` is the computing feature).
    Created {
        /// Geometry.
        geom: BodyGeom,
        /// Element map.
        naming: Option<BodyNaming>,
    },
    /// An existing body got new geometry.
    Modified {
        /// Geometry.
        geom: BodyGeom,
        /// Element map.
        naming: Option<BodyNaming>,
    },
    /// The body no longer exists after this feature.
    Deleted,
}

/// Full compute or a cheap interactive preview (separate cache keys, never persisted).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum ComputeMode {
    /// Final result.
    Full,
    /// Tool preview.
    Preview,
}

/// Why a feature failed to compute (or to validate).
#[derive(thiserror::Error, Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum FeatureError {
    /// An input does not exist at this point in the timeline.
    #[error("input {0:?} is not available")]
    MissingInput(Box<Dependency>),
    /// A topological reference did not resolve.
    #[error(transparent)]
    Resolve(#[from] ResolveError),
    /// A field has an unusable value.
    #[error("invalid value for {field}: {msg}")]
    InvalidValue {
        /// The field.
        field: &'static str,
        /// Why.
        msg: String,
    },
    /// The geometry kernel failed.
    #[error("geometry kernel failed: {0}")]
    Kernel(String),
    /// The feature kind (or its version) is unknown to this build.
    #[error("feature kind `{kind}` v{version} is not supported by this build")]
    UnknownKind {
        /// Kind as written in the file.
        kind: String,
        /// Version as written in the file.
        version: u32,
    },
    /// The recompute was cancelled.
    #[error("cancelled")]
    Cancelled,
}

impl amg_core::Diagnostic for FeatureError {
    fn is_cancelled(&self) -> bool {
        matches!(self, FeatureError::Cancelled)
    }
}

/// What a feature sees while it computes.
///
/// **Provisional (DOC-02):** the minimum the test features need. DOC-03 owns this type and
/// adds parameter evaluation, sketches, construction, resolution, progress and cancellation
/// through `amg-jobs`.
#[derive(Debug)]
pub struct RecomputeCtx<'a> {
    design: &'a Design,
    before: &'a Evaluated,
    feature: FeatureId,
    context: ComponentId,
    mode: ComputeMode,
}

impl<'a> RecomputeCtx<'a> {
    /// Context for computing `feature` (placed in `context`) against `before`, the evaluated
    /// state just before it.
    pub fn new(
        design: &'a Design,
        before: &'a Evaluated,
        feature: FeatureId,
        context: ComponentId,
        mode: ComputeMode,
    ) -> RecomputeCtx<'a> {
        RecomputeCtx {
            design,
            before,
            feature,
            context,
            mode,
        }
    }

    /// The snapshot being recomputed.
    pub fn design(&self) -> &'a Design {
        self.design
    }

    /// The computing feature.
    pub fn feature_id(&self) -> FeatureId {
        self.feature
    }

    /// Component the feature lives in.
    pub fn context(&self) -> ComponentId {
        self.context
    }

    /// Full or preview.
    pub fn mode(&self) -> ComputeMode {
        self.mode
    }

    /// Model tolerance.
    pub fn tolerance(&self) -> &'a Tolerance {
        &self.design.meta().tolerance
    }

    /// A body in its state just before this feature.
    pub fn body(&self, id: BodyId) -> Result<&'a BodyGeom, FeatureError> {
        self.before
            .body(id)
            .map(|b| &b.geom)
            .ok_or(FeatureError::MissingInput(Box::new(Dependency::Body(id))))
    }

    /// The id of this feature's output body `slot`.
    pub fn new_body_id(&self, slot: u32) -> BodyId {
        BodyId {
            origin: self.feature,
            slot,
        }
    }

    /// Reports progress of a long compute (no-op until DOC-12).
    pub fn progress(&self, _fraction: f32, _msg: &str) {}

    /// Cooperative cancellation point (never cancelled until DOC-12).
    pub fn check_cancelled(&self) -> Result<(), FeatureError> {
        Ok(())
    }
}
