//! Authored state (S01 §1): everything the user decided, as immutable snapshots.
//!
//! A [`Design`] is a handful of persistent maps, so `clone()` is O(1) and a clone shares all
//! unchanged structure with the original. [`DesignSnapshot`] (`Arc<Design>`) is the unit of
//! undo and of `design.json`.
//!
//! **Invariant:** every edit method either succeeds and leaves a design that saves *and* loads,
//! or fails and leaves the design unchanged. Loading checks the same invariants
//! ([`Design::validate`]), so a file can't hold a state the edit API couldn't produce.
//!
//! The timeline and its edits live in [`crate::timeline`] (DOC-02); the parameter table and its
//! edits live in [`crate::param`] (DOC-04).

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use amg_core::{PersistentId, Tolerance, Transform};
use serde::{Deserialize, Serialize};

use crate::eval::BodyGeom;
use crate::id::{BodyId, ComponentId, ExtKey, FeatureId, GroupId, JointId, OccurrenceId, Revision};
use crate::param::ParamTable;
use crate::pcoll::{PMap, entries};
use crate::stub::{AppearanceKey, LengthUnit, MaterialKey};
use crate::timeline::Timeline;

/// Immutable authored state shared between the document, undo stack and recompute jobs.
pub type DesignSnapshot = Arc<Design>;

/// Exclusive upper bound for authored ids (2⁵³): ids stay exact in JSON tools that read
/// numbers as `f64` (JavaScript, some Python paths). Far beyond any real design.
pub const ID_LIMIT: u64 = 1 << 53;

/// The authored state of one design.
///
/// Read through the accessors. The `edit` methods (`add_component`, `set_body_props`, ...)
/// are the low-level operations the command layer (DOC-05) is built on; application code
/// mutates a document only through `Document::apply`. Editing a detached `Design` (a scratch
/// copy for a preview, a test fixture) never affects any document.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(try_from = "DesignRepr")]
pub struct Design {
    pub(crate) meta: DesignMeta,
    root: ComponentId,
    #[serde(serialize_with = "entries::serialize")]
    components: PMap<ComponentId, Arc<Component>>,
    #[serde(serialize_with = "entries::serialize")]
    occurrences: PMap<OccurrenceId, Arc<Occurrence>>,
    #[serde(serialize_with = "entries::serialize")]
    pub(crate) body_props: PMap<BodyId, Arc<BodyProps>>,
    /// Baked bodies: direct mode, base feature, imports (DOC-08, S12).
    #[serde(serialize_with = "entries::serialize")]
    pub(crate) baked: PMap<BodyId, Arc<BakedBody>>,
    #[serde(serialize_with = "entries::serialize")]
    extensions: PMap<ExtKey, Arc<ExtRecord>>,
    /// The parametric timeline (DOC-02; edits in `timeline.rs`).
    pub(crate) timeline: Timeline,
    /// Parameters (DOC-04); edits in [`crate::param`].
    pub(crate) params: ParamTable,
    /// Runtime only: authored changes since the design was created or loaded.
    #[serde(skip)]
    revision: Revision,
}

/// Design-wide settings and the id counter.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DesignMeta {
    /// Document identity (UUID v7): recovery folder, linked designs. Never in cache keys.
    pub uuid: PersistentId,
    /// Design name.
    pub name: String,
    /// Parametric (history) or direct modelling.
    pub mode: DesignMode,
    /// Default display/input units.
    pub default_units: UnitPrefs,
    /// Model tolerance handed to recompute.
    pub tolerance: Tolerance,
    /// Next authored id; monotonic, never reused, below [`ID_LIMIT`].
    pub(crate) next_id: u64,
}

/// Whether the design captures history (Fusion's "Do not capture design history").
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum DesignMode {
    /// Parametric timeline.
    History,
    /// Features apply once and are baked (DOC-08).
    Direct,
}

/// Document unit preferences.
///
/// TODO(FND-10): grows angle/precision preferences once `amg_core::units` lands (S05 request).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct UnitPrefs {
    /// Default length unit (bare numbers in length fields, display).
    pub length: LengthUnit,
}

/// A component: a part definition that occurrences place in the tree.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Component {
    /// Display name.
    pub name: String,
    /// Occurrences placed inside this component, in browser order.
    pub children: Vec<OccurrenceId>,
    /// Joints owned by this component (S06). Always empty until S06 adds the joint table.
    pub joints: Vec<JointId>,
}

/// One placement of `component` inside `parent`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Occurrence {
    /// The placed component.
    pub component: ComponentId,
    /// The component this occurrence lives in.
    pub parent: ComponentId,
    /// Placement relative to `parent`, mm. Always finite.
    pub transform: Transform,
    /// Grounded occurrences do not move under joint solving (S06).
    pub grounded: bool,
    /// Browser visibility.
    pub visible: bool,
    /// Optional name override (default: the component's name).
    pub name: Option<String>,
}

/// User-set properties of a body; geometry lives in evaluated state.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BodyProps {
    /// Name override (default: generated from the creating feature).
    pub name: Option<String>,
    /// Physical material (S17).
    pub material: Option<MaterialKey>,
    /// Appearance override (S17).
    pub appearance: Option<AppearanceKey>,
    /// Visibility.
    pub visible: bool,
    /// Opacity, finite and in `0..=1`.
    pub opacity: f32,
}

impl Default for BodyProps {
    fn default() -> BodyProps {
        BodyProps {
            name: None,
            material: None,
            appearance: None,
            visible: true,
            opacity: 1.0,
        }
    }
}

impl BodyProps {
    fn check(&self) -> Result<(), ModelError> {
        if (0.0..=1.0).contains(&self.opacity) {
            Ok(())
        } else {
            Err(ModelError::InvalidValue {
                field: "opacity",
                msg: format!("{} is not in 0..=1", self.opacity),
            })
        }
    }
}

/// Reference to a binary entry inside the `.amg` container (DOC-07 fixes the layout).
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct BlobRef(pub String);

/// A body whose geometry is authored data rather than a recompute result (direct mode, base
/// feature, imported meshes; DOC-08, S12).
///
/// `geom` is the in-memory payload. It is not part of `design.json`: DOC-07 writes it to
/// `blob` in `bodies/` on save and fills it back in on load (`None` only between
/// deserializing `design.json` and reading `bodies/`).
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BakedBody {
    /// Component the body belongs to (format v2; v1 files get the root component).
    pub component: ComponentId,
    /// Container entry holding the serialized geometry.
    pub blob: BlobRef,
    /// The geometry, when loaded.
    #[serde(skip)]
    pub geom: Option<BodyGeom>,
}

impl PartialEq for BakedBody {
    /// Same blob and the same geometry allocation (geometry is compared by identity).
    fn eq(&self, other: &BakedBody) -> bool {
        self.blob == other.blob
            && self.component == other.component
            && match (&self.geom, &other.geom) {
                (None, None) => true,
                (Some(a), Some(b)) => a.same_as(b),
                _ => false,
            }
    }
}

/// Opaque state owned by another crate (studies, print setups; S01 §11). `amg-doc` never
/// interprets `data`.
///
/// `serde_json::Value` cannot hold NaN or infinities (`Number::from_f64` refuses them), so
/// `data` always serializes losslessly.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExtRecord {
    /// Registered extension kind; equals the key's kind.
    pub kind: String,
    /// Schema version of `data`, migrated by the owning crate.
    pub version: u32,
    /// The record itself.
    pub data: serde_json::Value,
    /// Binary payloads in the container.
    pub blobs: Vec<BlobRef>,
}

/// Why a low-level edit or a loaded design was rejected.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
#[non_exhaustive]
pub enum ModelError {
    /// A referenced id does not exist (or was never allocated).
    #[error("no such {what} {id}")]
    NotFound {
        /// Kind of id.
        what: &'static str,
        /// The raw id.
        id: u64,
    },
    /// The id is allocated to an entity of another type.
    #[error("id {id} belongs to a {owner}, not a {wanted}")]
    IdInUse {
        /// The raw id.
        id: u64,
        /// What owns it.
        owner: &'static str,
        /// What the caller tried to use it as.
        wanted: &'static str,
    },
    /// The edit would make a component contain itself.
    #[error("component {component} cannot be placed inside {parent}: it would contain itself")]
    WouldCycle {
        /// The component being placed.
        component: ComponentId,
        /// The intended parent.
        parent: ComponentId,
    },
    /// The edit would place `feature` before (or make it read) its input `input` (ordering
    /// rule, S01 §2).
    #[error("{feature} would come before its input {input}")]
    WouldBreakDependency {
        /// The feature whose input would come later.
        feature: FeatureId,
        /// The input's producer.
        input: FeatureId,
    },
    /// A value is out of range (non-finite, outside its domain, mismatched kind).
    #[error("invalid {field}: {msg}")]
    InvalidValue {
        /// The offending field.
        field: &'static str,
        /// Why.
        msg: String,
    },
    /// The id space ([`ID_LIMIT`]) is used up.
    #[error("no more ids available")]
    IdsExhausted,
    /// A loaded design violates an invariant.
    #[error("inconsistent design: {0}")]
    Inconsistent(String),
}

impl amg_core::Diagnostic for ModelError {}

fn check_transform(t: &Transform) -> Result<(), ModelError> {
    if t.is_finite() {
        Ok(())
    } else {
        Err(ModelError::InvalidValue {
            field: "transform",
            msg: "contains NaN or infinity".to_owned(),
        })
    }
}

impl Design {
    /// A new history-mode design with a root component named `name` and a fresh UUID v7.
    pub fn new(name: &str) -> Design {
        Design::with_uuid(name, PersistentId::new())
    }

    /// Like [`Design::new`] with an explicit document id (tests, recovery).
    pub fn with_uuid(name: &str, uuid: PersistentId) -> Design {
        let root = ComponentId(1);
        let mut components = PMap::new();
        components.insert(root, Arc::new(Component::named(name)));
        Design {
            meta: DesignMeta {
                uuid,
                name: name.to_owned(),
                mode: DesignMode::History,
                default_units: UnitPrefs::default(),
                tolerance: Tolerance::DEFAULT,
                next_id: 2,
            },
            root,
            components,
            occurrences: PMap::new(),
            body_props: PMap::new(),
            baked: PMap::new(),
            extensions: PMap::new(),
            timeline: Timeline::default(),
            params: ParamTable::default(),
            revision: Revision(0),
        }
    }

    // ---- read access ------------------------------------------------------------------

    /// Design-wide settings.
    pub fn meta(&self) -> &DesignMeta {
        &self.meta
    }

    /// The root component.
    pub fn root(&self) -> ComponentId {
        self.root
    }

    /// A component by id.
    pub fn component(&self, id: ComponentId) -> Option<&Component> {
        self.components.get(&id).map(|c| &**c)
    }

    /// All components in id order.
    pub fn components(&self) -> impl Iterator<Item = (ComponentId, &Component)> + '_ {
        self.components.iter().map(|(k, v)| (*k, &**v))
    }

    /// An occurrence by id.
    pub fn occurrence(&self, id: OccurrenceId) -> Option<&Occurrence> {
        self.occurrences.get(&id).map(|o| &**o)
    }

    /// All occurrences in id order.
    pub fn occurrences(&self) -> impl Iterator<Item = (OccurrenceId, &Occurrence)> + '_ {
        self.occurrences.iter().map(|(k, v)| (*k, &**v))
    }

    /// Properties of a body; the defaults if they were never set.
    pub fn body_props(&self, id: BodyId) -> BodyProps {
        self.body_props
            .get(&id)
            .map_or_else(BodyProps::default, |p| (**p).clone())
    }

    /// Bodies whose properties were set explicitly, in id order.
    pub fn explicit_body_props(&self) -> impl Iterator<Item = (BodyId, &BodyProps)> + '_ {
        self.body_props.iter().map(|(k, v)| (*k, &**v))
    }

    /// A baked body (direct mode / base feature / import).
    pub fn baked(&self, id: BodyId) -> Option<&BakedBody> {
        self.baked.get(&id).map(|b| &**b)
    }

    /// All baked bodies in id order.
    pub fn baked_bodies(&self) -> impl Iterator<Item = (BodyId, &BakedBody)> + '_ {
        self.baked.iter().map(|(k, v)| (*k, &**v))
    }

    /// An extension record.
    pub fn ext(&self, key: &ExtKey) -> Option<&ExtRecord> {
        self.extensions.get(key).map(|r| &**r)
    }

    /// All extension records in key order.
    pub fn exts(&self) -> impl Iterator<Item = (&ExtKey, &ExtRecord)> + '_ {
        self.extensions.iter().map(|(k, v)| (k, &**v))
    }

    /// Authored-change counter (runtime only).
    pub fn revision(&self) -> Revision {
        self.revision
    }

    /// Occurrences on the path from the root down to every placement of `component`
    /// (empty path for the root itself), in id order. Diamonds give one path per placement.
    pub fn occurrence_paths(&self, component: ComponentId) -> Vec<Vec<OccurrenceId>> {
        if component == self.root {
            return vec![Vec::new()];
        }
        let mut out = Vec::new();
        for (oid, occ) in self.occurrences() {
            if occ.component == component {
                for mut p in self.occurrence_paths(occ.parent) {
                    p.push(oid);
                    out.push(p);
                }
            }
        }
        out
    }

    // ---- edit (low level; the command layer builds on these) ----------------------------
    //
    // Each method validates everything before changing anything, and bumps the revision
    // only when the design actually changed.

    /// Creates a component and places one occurrence of it in `parent` (Fusion "New
    /// Component").
    pub fn add_component(
        &mut self,
        parent: ComponentId,
        name: &str,
        transform: Transform,
    ) -> Result<(ComponentId, OccurrenceId), ModelError> {
        self.require_component(parent)?;
        check_transform(&transform)?;
        self.ensure_ids(2)?;
        let id = ComponentId(self.alloc()?);
        self.components.insert(id, Arc::new(Component::named(name)));
        let occ = self.place(parent, id, transform)?;
        Ok((id, occ))
    }

    /// Places another occurrence of an existing component inside `parent`.
    pub fn add_occurrence(
        &mut self,
        parent: ComponentId,
        component: ComponentId,
        transform: Transform,
    ) -> Result<OccurrenceId, ModelError> {
        self.require_component(parent)?;
        self.require_component(component)?;
        check_transform(&transform)?;
        if component == self.root || self.contains(component, parent) {
            return Err(ModelError::WouldCycle { component, parent });
        }
        self.ensure_ids(1)?;
        self.place(parent, component, transform)
    }

    /// Sets (or, with the defaults, clears) the properties of a body. The body's origin must
    /// be a timeline feature, or the body must be baked.
    pub fn set_body_props(&mut self, id: BodyId, props: BodyProps) -> Result<(), ModelError> {
        self.check_body_id(id)?;
        if !self.timeline.contains(id.origin) && !self.baked.contains_key(&id) {
            return Err(ModelError::NotFound {
                what: "feature",
                id: id.origin.0,
            });
        }
        props.check()?;
        if self.body_props(id) == props {
            return Ok(());
        }
        if props == BodyProps::default() {
            self.body_props.remove(&id);
        } else {
            self.body_props.insert(id, Arc::new(props));
        }
        self.bump();
        Ok(())
    }

    /// Sets or removes a baked body. The origin is a timeline feature (imports, base feature)
    /// or, in direct mode (DOC-08), an allocated id that no other entity type owns. Removing
    /// a baked body whose origin is not a timeline feature also removes its properties.
    pub fn set_baked(&mut self, id: BodyId, baked: Option<BakedBody>) -> Result<(), ModelError> {
        self.check_body_id(id)?;
        if let Some(b) = &baked {
            self.require_component(b.component)?;
        }
        if self.baked.get(&id).map(|b| &**b) == baked.as_ref() {
            return Ok(());
        }
        match baked {
            Some(b) => self.baked.insert(id, Arc::new(b)),
            None => {
                if !self.timeline.contains(id.origin) {
                    self.body_props.remove(&id);
                }
                self.baked.remove(&id)
            }
        };
        self.bump();
        Ok(())
    }

    /// Loader hook (DOC-07): attaches the geometry read from `bodies/` to a baked body without
    /// counting as an authored change. No-op for an id that isn't baked.
    pub(crate) fn fill_baked_geom(&mut self, id: BodyId, geom: BodyGeom) {
        if let Some(b) = self.baked.get(&id) {
            let filled = BakedBody {
                component: b.component,
                blob: b.blob.clone(),
                geom: Some(geom),
            };
            self.baked.insert(id, Arc::new(filled));
        }
    }

    /// Allocates a key for a new extension record of `kind`.
    pub fn new_ext_key(&mut self, kind: &str) -> Result<ExtKey, ModelError> {
        Ok(ExtKey {
            kind: kind.to_owned(),
            id: self.alloc()?,
        })
    }

    /// Sets or removes an extension record. The key must come from [`Design::new_ext_key`]
    /// and `record.kind` must equal `key.kind`.
    pub fn set_ext(&mut self, key: ExtKey, record: Option<ExtRecord>) -> Result<(), ModelError> {
        if key.id == 0 || key.id >= self.meta.next_id {
            return Err(ModelError::NotFound {
                what: "extension id",
                id: key.id,
            });
        }
        if let Some(owner) = self.owner_of(key.id).filter(|&o| o != "extension") {
            return Err(ModelError::IdInUse {
                id: key.id,
                owner,
                wanted: "extension",
            });
        }
        if let Some(other) = self.ext_with_id(key.id).filter(|k| k.kind != key.kind) {
            return Err(ModelError::InvalidValue {
                field: "extension key",
                msg: format!("id {} already belongs to kind `{}`", key.id, other.kind),
            });
        }
        if let Some(r) = record.as_ref().filter(|r| r.kind != key.kind) {
            return Err(ModelError::InvalidValue {
                field: "extension kind",
                msg: format!("record kind `{}` != key kind `{}`", r.kind, key.kind),
            });
        }
        if self.extensions.get(&key).map(|r| &**r) == record.as_ref() {
            return Ok(());
        }
        match record {
            Some(r) => self.extensions.insert(key, Arc::new(r)),
            None => self.extensions.remove(&key),
        };
        self.bump();
        Ok(())
    }

    /// Renames the design (and its root component).
    pub fn set_name(&mut self, name: &str) {
        let root_name = self.components.get(&self.root).map(|c| c.name.as_str());
        if self.meta.name == name && root_name == Some(name) {
            return;
        }
        name.clone_into(&mut self.meta.name);
        if let Some(r) = self.components.get_mut(&self.root) {
            name.clone_into(&mut Arc::make_mut(r).name);
        }
        self.bump();
    }

    /// Sets an occurrence's placement relative to its parent.
    pub fn set_occurrence_transform(
        &mut self,
        occ: OccurrenceId,
        transform: Transform,
    ) -> Result<(), ModelError> {
        check_transform(&transform)?;
        let o = self.occurrences.get_mut(&occ).ok_or(ModelError::NotFound {
            what: "occurrence",
            id: occ.0,
        })?;
        if o.transform != transform {
            Arc::make_mut(o).transform = transform;
            self.bump();
        }
        Ok(())
    }

    /// Shows or hides an occurrence in the browser.
    pub fn set_occurrence_visible(
        &mut self,
        occ: OccurrenceId,
        visible: bool,
    ) -> Result<(), ModelError> {
        let o = self.occurrences.get_mut(&occ).ok_or(ModelError::NotFound {
            what: "occurrence",
            id: occ.0,
        })?;
        if o.visible != visible {
            Arc::make_mut(o).visible = visible;
            self.bump();
        }
        Ok(())
    }

    /// Renames a component (the root keeps the design name in sync, see [`Design::set_name`]).
    pub fn rename_component(&mut self, id: ComponentId, name: &str) -> Result<(), ModelError> {
        if id == self.root {
            self.set_name(name);
            return Ok(());
        }
        let c = self.components.get_mut(&id).ok_or(ModelError::NotFound {
            what: "component",
            id: id.0,
        })?;
        if c.name != name {
            name.clone_into(&mut Arc::make_mut(c).name);
            self.bump();
        }
        Ok(())
    }

    // ---- internals -----------------------------------------------------------------------

    pub(crate) fn alloc(&mut self) -> Result<u64, ModelError> {
        let id = self.meta.next_id;
        let next = id.checked_add(1).filter(|&n| n <= ID_LIMIT);
        self.meta.next_id = next.ok_or(ModelError::IdsExhausted)?;
        Ok(id)
    }

    pub(crate) fn ensure_ids(&self, n: u64) -> Result<(), ModelError> {
        match self.meta.next_id.checked_add(n) {
            Some(end) if end <= ID_LIMIT => Ok(()),
            _ => Err(ModelError::IdsExhausted),
        }
    }

    /// Inserts an occurrence; the caller has validated everything.
    fn place(
        &mut self,
        parent: ComponentId,
        component: ComponentId,
        transform: Transform,
    ) -> Result<OccurrenceId, ModelError> {
        let id = OccurrenceId(self.alloc()?);
        self.occurrences.insert(
            id,
            Arc::new(Occurrence {
                component,
                parent,
                transform,
                grounded: false,
                visible: true,
                name: None,
            }),
        );
        if let Some(p) = self.components.get_mut(&parent) {
            Arc::make_mut(p).children.push(id);
        }
        self.bump();
        Ok(id)
    }

    pub(crate) fn bump(&mut self) {
        self.revision = self.revision.next();
    }

    pub(crate) fn require_component(&self, id: ComponentId) -> Result<(), ModelError> {
        if self.components.contains_key(&id) {
            Ok(())
        } else {
            Err(ModelError::NotFound {
                what: "component",
                id: id.0,
            })
        }
    }

    fn ext_with_id(&self, id: u64) -> Option<&ExtKey> {
        self.extensions.keys().find(|k| k.id == id)
    }

    /// The entity type an allocated id belongs to, if any. Timeline nodes and body origins
    /// count as `"feature"`.
    pub(crate) fn owner_of(&self, id: u64) -> Option<&'static str> {
        let feature = origin_range(FeatureId(id));
        if self.components.contains_key(&ComponentId(id)) {
            Some("component")
        } else if self.occurrences.contains_key(&OccurrenceId(id)) {
            Some("occurrence")
        } else if self.ext_with_id(id).is_some() {
            Some("extension")
        } else if self.timeline.group(GroupId(id)).is_some() {
            Some("group")
        } else if self.params.get(crate::id::ParamId(id)).is_some() {
            Some("parameter")
        } else if self.timeline.contains(FeatureId(id))
            || self.body_props.range(feature.clone()).next().is_some()
            || self.baked.range(feature).next().is_some()
        {
            Some("feature")
        } else {
            None
        }
    }

    /// A body origin must be an allocated id that no non-feature entity owns. Body props
    /// additionally need a timeline feature or a baked body (`set_body_props`).
    fn check_body_id(&self, id: BodyId) -> Result<(), ModelError> {
        let raw = id.origin.0;
        if raw == 0 || raw >= self.meta.next_id {
            return Err(ModelError::NotFound {
                what: "feature",
                id: raw,
            });
        }
        match self.owner_of(raw) {
            Some(owner) if owner != "feature" => Err(ModelError::IdInUse {
                id: raw,
                owner,
                wanted: "feature",
            }),
            _ => Ok(()),
        }
    }

    /// True if `inner` is `outer` or appears anywhere below it.
    fn contains(&self, outer: ComponentId, inner: ComponentId) -> bool {
        let mut stack = vec![outer];
        let mut seen = BTreeSet::new();
        while let Some(c) = stack.pop() {
            if c == inner {
                return true;
            }
            if !seen.insert(c) {
                continue;
            }
            if let Some(comp) = self.components.get(&c) {
                stack.extend(
                    comp.children
                        .iter()
                        .filter_map(|o| self.occurrences.get(o))
                        .map(|o| o.component),
                );
            }
        }
        false
    }

    /// Checks every invariant the edit API maintains. Run on load; tests run it after edits.
    pub fn validate(&self) -> Result<(), ModelError> {
        let bad = |m: String| Err(ModelError::Inconsistent(m));
        let next = self.meta.next_id;
        if !(1..=ID_LIMIT).contains(&next) {
            return bad(format!("next_id {next} is outside 1..=2^53"));
        }
        let tol = self.meta.tolerance;
        let positive = |x: f64| x.is_finite() && x > 0.0;
        if !(positive(tol.linear) && positive(tol.angular)) {
            return bad(format!("tolerance {tol:?} must be finite and positive"));
        }
        if !self.components.contains_key(&self.root) {
            return bad(format!("root {} is not a component", self.root));
        }

        // Every authored id has exactly one owner and is below next_id.
        let mut owners: BTreeMap<u64, &'static str> = BTreeMap::new();
        let mut claim = |id: u64, what: &'static str| -> Result<(), ModelError> {
            if id == 0 || id >= next {
                return bad(format!("{what} id {id} is not in 1..next_id ({next})"));
            }
            match owners.insert(id, what) {
                Some(prev) => bad(format!("id {id} is used twice ({prev}, {what})")),
                None => Ok(()),
            }
        };
        for c in self.components.keys() {
            claim(c.0, "component")?;
        }
        for o in self.occurrences.keys() {
            claim(o.0, "occurrence")?;
        }
        for k in self.extensions.keys() {
            claim(k.id, "extension")?;
        }
        for (g, _) in self.timeline.groups() {
            claim(g.0, "group")?;
        }
        for n in self.timeline.nodes() {
            claim(n.id.0, "feature")?;
        }
        for p in self.params.ids() {
            claim(p.0, "parameter")?;
        }
        let origins: BTreeSet<u64> = self
            .body_props
            .keys()
            .chain(self.baked.keys())
            .map(|b| b.origin.0)
            .filter(|&o| !self.timeline.contains(FeatureId(o)))
            .collect();
        for o in origins {
            claim(o, "feature")?;
        }
        if let Err(m) = self
            .params
            .check(&self.meta.default_units, |id| owners.get(&id).copied())
        {
            return bad(m);
        }
        if let Some(b) = self
            .body_props
            .keys()
            .find(|b| !self.timeline.contains(b.origin) && !self.baked.contains_key(*b))
        {
            return bad(format!(
                "body props for {b:?}, whose origin is neither a timeline feature nor baked"
            ));
        }
        if let Some((b, _)) = self
            .baked
            .iter()
            .find(|(_, v)| !self.components.contains_key(&v.component))
        {
            return bad(format!("baked body {b:?} is in a missing component"));
        }
        self.validate_timeline()?;

        for (cid, comp) in self.components() {
            if let Some(j) = comp.joints.first() {
                return bad(format!("{cid} lists {j}, but joints don't exist until S06"));
            }
            for oid in &comp.children {
                match self.occurrences.get(oid) {
                    Some(o) if o.parent == cid => {}
                    _ => return bad(format!("{cid} lists {oid}, which is not placed in it")),
                }
            }
        }
        for (oid, occ) in self.occurrences() {
            for c in [occ.component, occ.parent] {
                if !self.components.contains_key(&c) {
                    return bad(format!("{oid} refers to missing {c}"));
                }
            }
            if occ.component == self.root {
                return bad(format!("{oid} places the root component"));
            }
            let listed = self.components[&occ.parent]
                .children
                .iter()
                .filter(|&&c| c == oid)
                .count();
            if listed != 1 {
                return bad(format!("{oid} is listed {listed} times in {}", occ.parent));
            }
            if check_transform(&occ.transform).is_err() {
                return bad(format!("{oid} has a non-finite transform"));
            }
        }
        for (oid, occ) in self.occurrences() {
            if self.contains(occ.component, occ.parent) {
                return bad(format!("{oid} makes {} contain itself", occ.component));
            }
        }
        // Every component must be reachable from the root.
        let mut reached = BTreeSet::new();
        let mut stack = vec![self.root];
        while let Some(c) = stack.pop() {
            if reached.insert(c) {
                stack.extend(
                    self.components[&c]
                        .children
                        .iter()
                        .map(|o| self.occurrences[o].component),
                );
            }
        }
        if let Some(c) = self.components.keys().find(|c| !reached.contains(c)) {
            return bad(format!("{c} is not reachable from the root {}", self.root));
        }

        for (bid, p) in self.explicit_body_props() {
            if let Err(e) = p.check() {
                return bad(format!("body {bid:?}: {e}"));
            }
        }
        for (key, rec) in self.exts() {
            if rec.kind != key.kind {
                return bad(format!(
                    "extension {} has record kind `{}` but key kind `{}`",
                    key.id, rec.kind, key.kind
                ));
            }
        }
        Ok(())
    }
}

/// All body ids whose origin is `f`.
pub(crate) fn origin_range(f: FeatureId) -> std::ops::RangeInclusive<BodyId> {
    BodyId { origin: f, slot: 0 }..=BodyId {
        origin: f,
        slot: u32::MAX,
    }
}

impl Component {
    fn named(name: &str) -> Component {
        Component {
            name: name.to_owned(),
            children: Vec::new(),
            joints: Vec::new(),
        }
    }
}

/// Wire form of [`Design`]; validated into a `Design` on load.
///
/// `deny_unknown_fields` policy: a field added later (timeline, params, ...) gets
/// `#[serde(default)]` here, so older files without it still load, and the change is recorded
/// with the `FORMAT_VERSION` policy in S01 (DOC-07).
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct DesignRepr {
    meta: DesignMeta,
    root: ComponentId,
    #[serde(deserialize_with = "entries::deserialize")]
    components: PMap<ComponentId, Arc<Component>>,
    #[serde(deserialize_with = "entries::deserialize")]
    occurrences: PMap<OccurrenceId, Arc<Occurrence>>,
    #[serde(deserialize_with = "entries::deserialize")]
    body_props: PMap<BodyId, Arc<BodyProps>>,
    #[serde(deserialize_with = "entries::deserialize")]
    baked: PMap<BodyId, Arc<BakedBody>>,
    #[serde(deserialize_with = "entries::deserialize")]
    extensions: PMap<ExtKey, Arc<ExtRecord>>,
    /// Added by DOC-02 (defaulted, per the wire-format policy).
    #[serde(default)]
    timeline: Timeline,
    /// Added by DOC-04; defaulted so files without it still load.
    #[serde(default)]
    params: ParamTable,
}

impl TryFrom<DesignRepr> for Design {
    type Error = ModelError;
    fn try_from(r: DesignRepr) -> Result<Design, ModelError> {
        let params = r.params.loaded(&r.meta.default_units)?;
        let d = Design {
            meta: r.meta,
            root: r.root,
            components: r.components,
            occurrences: r.occurrences,
            body_props: r.body_props,
            baked: r.baked,
            extensions: r.extensions,
            timeline: r.timeline,
            params,
            revision: Revision(0),
        };
        d.validate()?;
        Ok(d)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::stub::IndexedMesh;
    use amg_core::Vec3;

    fn uuid() -> PersistentId {
        "0190f5a3-0000-7000-8000-000000000001".parse().unwrap()
    }

    fn body(f: u64, slot: u32) -> BodyId {
        BodyId {
            origin: FeatureId(f),
            slot,
        }
    }

    /// Saves and reloads `d`; asserts the reload succeeds, validates and is byte-identical.
    #[track_caller]
    fn rt(d: &Design) -> Design {
        d.validate().unwrap();
        let s = serde_json::to_string(d).unwrap();
        let back: Design = serde_json::from_str(&s)
            .unwrap_or_else(|e| panic!("saved design won't load: {e}\n{s}"));
        assert_eq!(serde_json::to_string(&back).unwrap(), s);
        back
    }

    /// Adds a `test.box` feature to the timeline (body origins must be timeline features).
    fn reserve_feature(d: &mut Design) -> u64 {
        let f = crate::FeatureBox::new(crate::testing::TestBox {
            size: [1.0, 2.0, 3.0],
        });
        let root = d.root();
        d.add_feature(None, f, root, None).unwrap().0
    }

    /// Root + 3 components (one placed twice, one nested), body props, a baked body and an
    /// extension. Round-trips after every edit.
    fn sample() -> Design {
        let mut d = Design::with_uuid("Bracket", uuid());
        rt(&d);
        let root = d.root();
        let (base, _) = d.add_component(root, "Base", Transform::IDENTITY).unwrap();
        rt(&d);
        let (arm, _) = d
            .add_component(
                root,
                "Arm",
                Transform::from_translation(Vec3::new(0.0, 0.0, 10.0)),
            )
            .unwrap();
        rt(&d);
        let (bolt, _) = d
            .add_component(arm, "Bolt", Transform::from_translation(Vec3::X * 5.0))
            .unwrap();
        rt(&d);
        d.add_occurrence(base, bolt, Transform::from_translation(Vec3::Y * -3.25))
            .unwrap();
        rt(&d);
        let f1 = reserve_feature(&mut d);
        let f2 = reserve_feature(&mut d);
        d.set_body_props(
            body(f1, 0),
            BodyProps {
                name: Some("Plate".into()),
                material: Some(MaterialKey("PLA".into())),
                appearance: None,
                visible: true,
                opacity: 0.75,
            },
        )
        .unwrap();
        rt(&d);
        d.set_body_props(
            body(f2, 2),
            BodyProps {
                visible: false,
                ..BodyProps::default()
            },
        )
        .unwrap();
        rt(&d);
        d.set_baked(
            body(f2, 0),
            Some(BakedBody {
                component: d.root(),
                blob: BlobRef(format!("bodies/{f2}-0.mesh")),
                geom: Some(BodyGeom::Mesh(Arc::new(IndexedMesh::default()))),
            }),
        )
        .unwrap();
        rt(&d);
        let key = d.new_ext_key("fem.study").unwrap();
        d.set_ext(
            key,
            Some(ExtRecord {
                kind: "fem.study".into(),
                version: 1,
                data: serde_json::json!({ "name": "Static 1", "loads": [1.5, 2.0], "a": null }),
                blobs: vec![BlobRef("ext/fem.study-9/result.bin".into())],
            }),
        )
        .unwrap();
        rt(&d);
        d.set_name("Bracket v2");
        rt(&d);
        d
    }

    fn component_named(d: &Design, name: &str) -> ComponentId {
        d.components().find(|(_, c)| c.name == name).unwrap().0
    }

    #[test]
    fn builds_a_component_tree() {
        let d = sample();
        assert_eq!(d.components().count(), 4, "root + 3 components");
        assert_eq!(d.occurrences().count(), 4);
        let root = d.component(d.root()).unwrap();
        assert_eq!(root.name, "Bracket v2");
        assert_eq!(root.children.len(), 2);
        let bolt = component_named(&d, "Bolt");
        // Bolt is placed in Arm and in Base: two paths from the root.
        let paths = d.occurrence_paths(bolt);
        assert_eq!(paths.len(), 2);
        assert!(paths.iter().all(|p| p.len() == 2));
        for (oid, occ) in d.occurrences() {
            assert!(d.component(occ.parent).unwrap().children.contains(&oid));
        }
        assert!(d.revision() > Revision(0));
    }

    #[test]
    fn ids_are_monotonic_and_unique() {
        let d = sample();
        let mut ids: Vec<u64> = d
            .components()
            .map(|(c, _)| c.0)
            .chain(d.occurrences().map(|(o, _)| o.0))
            .chain(d.exts().map(|(k, _)| k.id))
            .collect();
        let n = ids.len();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), n);
        assert!(ids.iter().all(|&i| i > 0 && i < d.meta().next_id));
    }

    #[test]
    fn body_props_default_and_clear() {
        let mut d = sample();
        let free = reserve_feature(&mut d);
        assert_eq!(d.body_props(body(free, 0)), BodyProps::default());
        let set: Vec<_> = d.explicit_body_props().map(|(b, _)| b).collect();
        assert_eq!(set.len(), 2);
        assert!((d.body_props(set[0]).opacity - 0.75).abs() < f32::EPSILON);
        d.set_body_props(set[1], BodyProps::default()).unwrap();
        assert_eq!(d.explicit_body_props().count(), 1);
        rt(&d);
    }

    #[test]
    fn baked_body_keeps_geometry_in_memory_but_not_in_json() {
        let d = sample();
        let (id, b) = d.baked_bodies().next().unwrap();
        assert!(b.geom.is_some());
        let back = rt(&d);
        let loaded = back.baked(id).unwrap();
        assert_eq!(loaded.blob, b.blob);
        assert!(loaded.geom.is_none(), "DOC-07 fills geometry from bodies/");
        assert!(!serde_json::to_string(&d).unwrap().contains("geom"));
    }

    #[test]
    fn rejects_cycles_and_missing_ids() {
        let mut d = sample();
        let root = d.root();
        let arm = component_named(&d, "Arm");
        let bolt = component_named(&d, "Bolt");
        let before = serde_json::to_string(&d).unwrap();
        assert_eq!(
            d.add_occurrence(bolt, arm, Transform::IDENTITY),
            Err(ModelError::WouldCycle {
                component: arm,
                parent: bolt
            })
        );
        assert!(matches!(
            d.add_occurrence(arm, arm, Transform::IDENTITY),
            Err(ModelError::WouldCycle { .. })
        ));
        assert!(matches!(
            d.add_occurrence(arm, root, Transform::IDENTITY),
            Err(ModelError::WouldCycle { .. })
        ));
        assert!(matches!(
            d.add_component(ComponentId(12345), "X", Transform::IDENTITY),
            Err(ModelError::NotFound { .. })
        ));
        assert_eq!(
            before,
            serde_json::to_string(&d).unwrap(),
            "failed edits change nothing"
        );
    }

    #[test]
    fn body_ids_must_be_allocated_feature_ids() {
        let mut d = sample();
        let root = d.root();
        let rev = d.revision();
        let next = d.meta().next_id;
        assert_eq!(
            d.set_body_props(body(next, 0), BodyProps::default()),
            Err(ModelError::NotFound {
                what: "feature",
                id: next
            })
        );
        assert!(matches!(
            d.set_body_props(body(0, 0), BodyProps::default()),
            Err(ModelError::NotFound { .. })
        ));
        assert_eq!(
            d.set_body_props(
                body(root.0, 0),
                BodyProps {
                    visible: false,
                    ..BodyProps::default()
                }
            ),
            Err(ModelError::IdInUse {
                id: root.0,
                owner: "component",
                wanted: "feature"
            })
        );
        let blob = BakedBody {
            component: d.root(),
            blob: BlobRef("bodies/x".into()),
            geom: None,
        };
        assert!(d.set_baked(body(next + 5, 0), Some(blob.clone())).is_err());
        let ext = d.exts().next().unwrap().0.id;
        assert!(matches!(
            d.set_baked(body(ext, 0), Some(blob)),
            Err(ModelError::IdInUse {
                owner: "extension",
                ..
            })
        ));
        assert_eq!(d.revision(), rev);
        rt(&d);
    }

    #[test]
    fn rejects_non_finite_values() {
        let mut d = sample();
        let root = d.root();
        let f = reserve_feature(&mut d);
        let rev = d.revision();
        let nan = Transform::from_translation(Vec3::new(f64::NAN, 0.0, 0.0));
        let inf = Transform::from_scale(Vec3::splat(f64::INFINITY));
        for t in [nan, inf] {
            assert!(matches!(
                d.add_component(root, "X", t),
                Err(ModelError::InvalidValue {
                    field: "transform",
                    ..
                })
            ));
            let arm = component_named(&d, "Arm");
            assert!(d.add_occurrence(root, arm, t).is_err());
        }
        for opacity in [f32::NAN, f32::INFINITY, -0.1, 1.5] {
            let p = BodyProps {
                opacity,
                ..BodyProps::default()
            };
            assert!(matches!(
                d.set_body_props(body(f, 0), p),
                Err(ModelError::InvalidValue {
                    field: "opacity",
                    ..
                })
            ));
        }
        assert_eq!(d.revision(), rev);
        // ExtRecord data can't hold non-finite numbers at all.
        assert!(serde_json::json!(f64::NAN).is_null());
        rt(&d);
    }

    #[test]
    fn ext_keys_are_checked() {
        let mut d = sample();
        let root = d.root();
        let rec = |kind: &str| ExtRecord {
            kind: kind.into(),
            version: 1,
            data: serde_json::json!({}),
            blobs: vec![],
        };
        // Id of a component.
        let stolen = ExtKey {
            kind: "fem.study".into(),
            id: root.0,
        };
        assert!(matches!(
            d.set_ext(stolen, Some(rec("fem.study"))),
            Err(ModelError::IdInUse {
                owner: "component",
                ..
            })
        ));
        // Record kind differs from key kind.
        let k = d.new_ext_key("am.print").unwrap();
        assert!(matches!(
            d.set_ext(k.clone(), Some(rec("fem.study"))),
            Err(ModelError::InvalidValue {
                field: "extension kind",
                ..
            })
        ));
        // Same id under another kind.
        d.set_ext(k.clone(), Some(rec("am.print"))).unwrap();
        let alias = ExtKey {
            kind: "other".into(),
            id: k.id,
        };
        assert!(matches!(
            d.set_ext(alias, Some(rec("other"))),
            Err(ModelError::InvalidValue {
                field: "extension key",
                ..
            })
        ));
        rt(&d);
    }

    #[test]
    fn no_op_edits_do_not_bump_the_revision() {
        let mut d = sample();
        let f = reserve_feature(&mut d);
        let rev = d.revision();
        let (bid, props) = d
            .explicit_body_props()
            .map(|(b, p)| (b, p.clone()))
            .next()
            .unwrap();
        d.set_body_props(bid, props).unwrap();
        d.set_body_props(body(f, 0), BodyProps::default()).unwrap();
        d.set_baked(body(f, 0), None).unwrap();
        let (bb, baked) = d
            .baked_bodies()
            .map(|(b, x)| (b, x.clone()))
            .next()
            .unwrap();
        d.set_baked(bb, Some(baked)).unwrap();
        let (key, rec) = d
            .exts()
            .map(|(k, r)| (k.clone(), r.clone()))
            .next()
            .unwrap();
        d.set_ext(key, Some(rec)).unwrap();
        let unused = d.new_ext_key("x").unwrap();
        d.set_ext(unused, None).unwrap();
        let name = d.meta().name.clone();
        d.set_name(&name);
        assert_eq!(d.revision(), rev);
        d.set_name("changed");
        assert_eq!(d.revision(), rev.next());
    }

    #[test]
    fn id_space_is_bounded() {
        let mut d = sample();
        let root = d.root();
        d.meta.next_id = ID_LIMIT - 1;
        let before = d.components().count();
        assert_eq!(
            d.add_component(root, "X", Transform::IDENTITY),
            Err(ModelError::IdsExhausted)
        );
        assert_eq!(
            d.components().count(),
            before,
            "no orphan component left behind"
        );
        d.new_ext_key("a").unwrap();
        assert_eq!(d.new_ext_key("b"), Err(ModelError::IdsExhausted));
        assert_eq!(d.meta().next_id, ID_LIMIT);
        rt(&d);
    }

    #[test]
    fn snapshot_clone_shares_structure_and_is_independent() {
        let a: DesignSnapshot = Arc::new(sample());
        let mut b = (*a).clone();
        b.set_name("Renamed");
        let (bid, _) = b.explicit_body_props().next().unwrap();
        b.set_body_props(bid, BodyProps::default()).unwrap();
        assert_eq!(a.meta().name, "Bracket v2");
        assert_eq!(a.component(a.root()).unwrap().name, "Bracket v2");
        assert_eq!(b.component(b.root()).unwrap().name, "Renamed");
        assert_eq!(a.explicit_body_props().count(), 2);
        // Untouched entries are the same allocation in both snapshots.
        let occ = *a.occurrences.keys().next().unwrap();
        assert!(Arc::ptr_eq(&a.occurrences[&occ], &b.occurrences[&occ]));
    }

    #[test]
    fn serde_round_trip_is_byte_identical() {
        let d = sample();
        let s1 = serde_json::to_string_pretty(&d).unwrap();
        let back: Design = serde_json::from_str(&s1).unwrap();
        let s2 = serde_json::to_string_pretty(&back).unwrap();
        assert_eq!(s1, s2, "save -> load -> save must be byte-identical");
        let back2: Design = serde_json::from_str(&s2).unwrap();
        assert_eq!(s2, serde_json::to_string_pretty(&back2).unwrap());
        // Serializing the same value twice is identical too, and compact form round-trips.
        assert_eq!(s1, serde_json::to_string_pretty(&d).unwrap());
        let c = serde_json::to_string(&d).unwrap();
        assert_eq!(c, serde_json::to_string(&back).unwrap());
        assert_eq!(back.meta(), d.meta());
        assert_eq!(back.revision(), Revision(0), "revision is not persisted");
    }

    #[test]
    fn json_shape_is_stable() {
        let d = Design::with_uuid("P", uuid());
        let v = serde_json::to_value(&d).unwrap();
        let keys: Vec<_> = v.as_object().unwrap().keys().cloned().collect();
        // serde_json's default map is sorted, so only check the set here.
        assert_eq!(
            keys,
            [
                "baked",
                "body_props",
                "components",
                "extensions",
                "meta",
                "occurrences",
                "params",
                "root",
                "timeline"
            ]
        );
        let s = serde_json::to_string(&d).unwrap();
        assert!(
            s.starts_with(
                r#"{"meta":{"uuid":"0190f5a3-0000-7000-8000-000000000001","name":"P","mode":"History","default_units":{"length":"Millimetre"},"tolerance":{"linear":1e-6,"angular":1e-9},"next_id":2},"root":1,"components":[[1,{"name":"P","children":[],"joints":[]}]]"#
            ),
            "{s}"
        );
    }

    #[test]
    fn load_rejects_inconsistent_designs() {
        let good = serde_json::to_value(sample()).unwrap();
        // Indices into the sample: components [root, Base, Arm, Bolt], occurrences in id order
        // [Base in root, Arm in root, Bolt in Arm, Bolt in Base].
        let occ = |v: &serde_json::Value, i: usize| v["occurrences"][i][0].clone();
        type Edit = Box<dyn Fn(&mut serde_json::Value)>;
        let cases: Vec<(&str, Edit)> = vec![
            ("root", Box::new(|v| v["root"] = 4242.into())),
            ("next_id", Box::new(|v| v["meta"]["next_id"] = 2.into())),
            (
                "outside 1..=2^53",
                Box::new(|v| v["meta"]["next_id"] = (1u64 << 63).into()),
            ),
            (
                "tolerance",
                Box::new(|v| v["meta"]["tolerance"]["linear"] = 0.0.into()),
            ),
            (
                "is listed 0 times",
                Box::new(|v| v["components"][0][1]["children"] = serde_json::json!([])),
            ),
            (
                "duplicate key",
                Box::new(|v| {
                    let first = v["occurrences"][0].clone();
                    v["occurrences"].as_array_mut().unwrap().push(first);
                }),
            ),
            ("unknown field", Box::new(|v| v["extra"] = 1.into())),
            (
                "places the root component",
                Box::new(|v| v["occurrences"][0][1]["component"] = 1.into()),
            ),
            (
                "which is not placed in it",
                Box::new(move |v| {
                    // Arm (index 2) claims Base's occurrence too.
                    let o = occ(v, 0);
                    v["components"][2][1]["children"]
                        .as_array_mut()
                        .unwrap()
                        .push(o);
                }),
            ),
            (
                "contain itself",
                Box::new(move |v| {
                    // Turn "Bolt in Base" into "Arm in Bolt": Arm -> Bolt -> Arm.
                    let o = occ(v, 3);
                    v["occurrences"][3][1]["component"] = v["components"][2][0].clone();
                    v["occurrences"][3][1]["parent"] = v["components"][3][0].clone();
                    v["components"][1][1]["children"] = serde_json::json!([]);
                    v["components"][3][1]["children"] = serde_json::json!([o]);
                }),
            ),
            (
                "not reachable from the root",
                Box::new(|v| {
                    // Drop "Base in root": Base (and only its subtree) becomes an orphan.
                    let o = v["occurrences"][0][0].clone();
                    v["occurrences"].as_array_mut().unwrap().remove(0);
                    v["components"][0][1]["children"]
                        .as_array_mut()
                        .unwrap()
                        .retain(|c| *c != o);
                }),
            ),
            (
                "used twice (component, extension)",
                Box::new(|v| v["extensions"][0][0]["id"] = 1.into()),
            ),
            (
                "record kind",
                Box::new(|v| v["extensions"][0][1]["kind"] = "zzz".into()),
            ),
            (
                "joints don't exist",
                Box::new(|v| v["components"][1][1]["joints"] = serde_json::json!([123])),
            ),
            (
                "opacity",
                Box::new(|v| v["body_props"][0][1]["opacity"] = 2.0.into()),
            ),
            (
                "used twice (component, feature)",
                Box::new(|v| v["body_props"][0][0]["origin"] = 1.into()),
            ),
        ];
        for (needle, edit) in cases {
            let mut v = good.clone();
            edit(&mut v);
            let e = serde_json::from_value::<Design>(v)
                .map(|_| "loaded".to_owned())
                .unwrap_or_else(|e| e.to_string());
            assert!(e.contains(needle), "expected `{needle}` in `{e}`");
        }
        serde_json::from_value::<Design>(good).unwrap();
    }

    /// JSON can't carry non-finite numbers, so these branches are reached only by in-memory
    /// corruption; check them directly.
    #[test]
    fn validate_rejects_non_finite_state() {
        let mut d = sample();
        let o = *d.occurrences.keys().next().unwrap();
        Arc::make_mut(d.occurrences.get_mut(&o).unwrap()).transform =
            Transform::from_translation(Vec3::new(0.0, f64::INFINITY, 0.0));
        let e = d.validate().unwrap_err().to_string();
        assert!(e.contains("non-finite transform"), "{e}");
        let mut d = sample();
        d.meta.tolerance.angular = f64::NAN;
        let e = d.validate().unwrap_err().to_string();
        assert!(e.contains("tolerance"), "{e}");
    }

    #[test]
    fn thread_safety() {
        fn assert_send_sync<T: Send + Sync>() {}
        assert_send_sync::<Design>();
        assert_send_sync::<DesignSnapshot>();
    }
}
