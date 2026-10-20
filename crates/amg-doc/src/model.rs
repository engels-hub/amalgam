//! Authored state (S01 §1): everything the user decided, as immutable snapshots.
//!
//! A [`Design`] is a handful of persistent maps, so `clone()` is O(1) and a clone shares all
//! unchanged structure with the original. [`DesignSnapshot`] (`Arc<Design>`) is the unit of
//! undo and of `design.json`.
//!
//! The timeline (DOC-02) and parameter table (DOC-04) join `Design` in their work packages.

use std::sync::Arc;

use amg_core::{PersistentId, Tolerance, Transform};
use serde::{Deserialize, Serialize};

use crate::id::{BodyId, ComponentId, ExtKey, JointId, OccurrenceId, Revision};
use crate::pcoll::{PMap, entries};
use crate::stub::{AppearanceKey, LengthUnit, MaterialKey};

/// Immutable authored state shared between the document, undo stack and recompute jobs.
pub type DesignSnapshot = Arc<Design>;

/// The authored state of one design.
///
/// Read through the accessors. The `edit` methods (`add_component`, `set_body_props`, ...)
/// are the low-level operations the command layer (DOC-05) is built on; application code
/// mutates a document only through `Document::apply`. Editing a detached `Design` (a scratch
/// copy for a preview, a test fixture) never affects any document.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(try_from = "DesignRepr")]
pub struct Design {
    meta: DesignMeta,
    root: ComponentId,
    #[serde(serialize_with = "entries::serialize")]
    components: PMap<ComponentId, Arc<Component>>,
    #[serde(serialize_with = "entries::serialize")]
    occurrences: PMap<OccurrenceId, Arc<Occurrence>>,
    #[serde(serialize_with = "entries::serialize")]
    body_props: PMap<BodyId, Arc<BodyProps>>,
    /// Direct-mode / base-feature bodies (DOC-08).
    #[serde(serialize_with = "entries::serialize")]
    baked: PMap<BodyId, Arc<BlobRef>>,
    #[serde(serialize_with = "entries::serialize")]
    extensions: PMap<ExtKey, Arc<ExtRecord>>,
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
    /// Next authored id; monotonic, never reused.
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
/// TODO(FND-10): grows angle/mass/... preferences once `amg_core::units` lands.
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
    /// Joints owned by this component (S06).
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
    /// Placement relative to `parent`, mm.
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
    /// Opacity, 0..=1.
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

/// Reference to a binary entry inside the `.amg` container (DOC-07 fixes the layout).
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct BlobRef(pub String);

/// Opaque state owned by another crate (studies, print setups; S01 §11). `amg-doc` never
/// interprets `data`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExtRecord {
    /// Registered extension kind.
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
    /// A referenced id does not exist.
    #[error("no such {what} {id}")]
    NotFound {
        /// Kind of id.
        what: &'static str,
        /// The raw id.
        id: u64,
    },
    /// The edit would make a component contain itself.
    #[error("component {component} cannot be placed inside {parent}: it would contain itself")]
    WouldCycle {
        /// The component being placed.
        component: ComponentId,
        /// The intended parent.
        parent: ComponentId,
    },
    /// A loaded design violates an invariant.
    #[error("inconsistent design: {0}")]
    Inconsistent(String),
}

impl amg_core::Diagnostic for ModelError {}

impl Design {
    /// A new history-mode design with a root component named `name` and a fresh UUID v7.
    pub fn new(name: &str) -> Design {
        Design::with_uuid(name, PersistentId::new())
    }

    /// Like [`Design::new`] with an explicit document id (tests, recovery).
    pub fn with_uuid(name: &str, uuid: PersistentId) -> Design {
        let mut d = Design {
            meta: DesignMeta {
                uuid,
                name: name.to_owned(),
                mode: DesignMode::History,
                default_units: UnitPrefs::default(),
                tolerance: Tolerance::DEFAULT,
                next_id: 1,
            },
            root: ComponentId(0),
            components: PMap::new(),
            occurrences: PMap::new(),
            body_props: PMap::new(),
            baked: PMap::new(),
            extensions: PMap::new(),
            revision: Revision(0),
        };
        let root = ComponentId(d.alloc());
        d.components.insert(root, Arc::new(Component::named(name)));
        d.root = root;
        d
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

    /// Baked geometry of a body (direct mode / base feature).
    pub fn baked(&self, id: BodyId) -> Option<&BlobRef> {
        self.baked.get(&id).map(|b| &**b)
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

    /// Creates a component and places one occurrence of it in `parent` (Fusion "New
    /// Component").
    pub fn add_component(
        &mut self,
        parent: ComponentId,
        name: &str,
        transform: Transform,
    ) -> Result<(ComponentId, OccurrenceId), ModelError> {
        self.require_component(parent)?;
        let id = ComponentId(self.alloc());
        self.components.insert(id, Arc::new(Component::named(name)));
        let occ = self.add_occurrence(parent, id, transform)?;
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
        if component == self.root || self.contains(component, parent) {
            return Err(ModelError::WouldCycle { component, parent });
        }
        let id = OccurrenceId(self.alloc());
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

    /// Sets (or, with the defaults, clears) the properties of a body.
    pub fn set_body_props(&mut self, id: BodyId, props: BodyProps) {
        if props == BodyProps::default() {
            self.body_props.remove(&id);
        } else {
            self.body_props.insert(id, Arc::new(props));
        }
        self.bump();
    }

    /// Allocates a key for a new extension record of `kind`.
    pub fn new_ext_key(&mut self, kind: &str) -> ExtKey {
        ExtKey {
            kind: kind.to_owned(),
            id: self.alloc(),
        }
    }

    /// Sets or removes an extension record. The key must come from [`Design::new_ext_key`].
    pub fn set_ext(&mut self, key: ExtKey, record: Option<ExtRecord>) -> Result<(), ModelError> {
        if key.id >= self.meta.next_id {
            return Err(ModelError::NotFound {
                what: "extension id",
                id: key.id,
            });
        }
        match record {
            Some(r) => {
                self.extensions.insert(key, Arc::new(r));
            }
            None => {
                self.extensions.remove(&key);
            }
        }
        self.bump();
        Ok(())
    }

    /// Renames the design (and its root component).
    pub fn set_name(&mut self, name: &str) {
        name.clone_into(&mut self.meta.name);
        if let Some(r) = self.components.get_mut(&self.root) {
            name.clone_into(&mut Arc::make_mut(r).name);
        }
        self.bump();
    }

    // ---- internals -----------------------------------------------------------------------

    fn alloc(&mut self) -> u64 {
        let id = self.meta.next_id;
        self.meta.next_id += 1;
        id
    }

    fn bump(&mut self) {
        self.revision = self.revision.next();
    }

    fn require_component(&self, id: ComponentId) -> Result<(), ModelError> {
        if self.components.contains_key(&id) {
            Ok(())
        } else {
            Err(ModelError::NotFound {
                what: "component",
                id: id.0,
            })
        }
    }

    /// True if `inner` is `outer` or appears anywhere below it.
    fn contains(&self, outer: ComponentId, inner: ComponentId) -> bool {
        let mut stack = vec![outer];
        let mut seen = std::collections::BTreeSet::new();
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

    /// Checks every structural invariant (used on load).
    fn validate(&self) -> Result<(), ModelError> {
        let bad = |m: String| Err(ModelError::Inconsistent(m));
        let next = self.meta.next_id;
        if !self.components.contains_key(&self.root) {
            return bad(format!("root {} is not a component", self.root));
        }
        let ids = self
            .components
            .keys()
            .map(|c| c.0)
            .chain(self.occurrences.keys().map(|o| o.0))
            .chain(self.extensions.keys().map(|k| k.id))
            .chain(
                self.body_props
                    .keys()
                    .chain(self.baked.keys())
                    .map(|b| b.origin.0),
            );
        if let Some(id) = ids.into_iter().find(|&id| id >= next) {
            return bad(format!("id {id} is not below next_id {next}"));
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
        }
        for (cid, comp) in self.components() {
            for oid in &comp.children {
                match self.occurrences.get(oid) {
                    Some(o) if o.parent == cid => {}
                    _ => return bad(format!("{cid} lists {oid}, which is not placed in it")),
                }
            }
        }
        // Acyclic: every component reachable from an occurrence must not contain its parent.
        for (oid, occ) in self.occurrences() {
            if self.contains(occ.component, occ.parent) {
                return bad(format!("{oid} makes {} contain itself", occ.component));
            }
        }
        Ok(())
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
    baked: PMap<BodyId, Arc<BlobRef>>,
    #[serde(deserialize_with = "entries::deserialize")]
    extensions: PMap<ExtKey, Arc<ExtRecord>>,
}

impl TryFrom<DesignRepr> for Design {
    type Error = ModelError;
    fn try_from(r: DesignRepr) -> Result<Design, ModelError> {
        let d = Design {
            meta: r.meta,
            root: r.root,
            components: r.components,
            occurrences: r.occurrences,
            body_props: r.body_props,
            baked: r.baked,
            extensions: r.extensions,
            revision: Revision(0),
        };
        d.validate()?;
        Ok(d)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::id::FeatureId;
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

    /// Root + 3 components (one placed twice, one nested), body props and an extension.
    fn sample() -> Design {
        let mut d = Design::with_uuid("Bracket", uuid());
        let root = d.root();
        let (base, _) = d.add_component(root, "Base", Transform::IDENTITY).unwrap();
        let (arm, arm_occ) = d
            .add_component(
                root,
                "Arm",
                Transform::from_translation(Vec3::new(0.0, 0.0, 10.0)),
            )
            .unwrap();
        let (bolt, _) = d
            .add_component(arm, "Bolt", Transform::from_translation(Vec3::X * 5.0))
            .unwrap();
        d.add_occurrence(base, bolt, Transform::from_translation(Vec3::Y * -3.25))
            .unwrap();
        let _ = arm_occ;
        // Feature ids would come from the timeline (DOC-02); reserve some for the bodies.
        let f = d.meta.next_id;
        d.meta.next_id += 3;
        d.set_body_props(
            body(f, 0),
            BodyProps {
                name: Some("Plate".into()),
                material: Some(MaterialKey("PLA".into())),
                appearance: None,
                visible: true,
                opacity: 0.75,
            },
        );
        d.set_body_props(
            body(f + 1, 2),
            BodyProps {
                visible: false,
                ..BodyProps::default()
            },
        );
        let key = d.new_ext_key("fem.study");
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
        d
    }

    #[test]
    fn builds_a_component_tree() {
        let d = sample();
        assert_eq!(d.components().count(), 4, "root + 3 components");
        assert_eq!(d.occurrences().count(), 4);
        let root = d.component(d.root()).unwrap();
        assert_eq!(root.name, "Bracket");
        assert_eq!(root.children.len(), 2);
        let bolt = d
            .components()
            .find(|(_, c)| c.name == "Bolt")
            .map(|(id, _)| id)
            .unwrap();
        // Bolt is placed in Arm and in Base: two paths from the root.
        let paths = d.occurrence_paths(bolt);
        assert_eq!(paths.len(), 2);
        assert!(paths.iter().all(|p| p.len() == 2));
        for (oid, occ) in d.occurrences() {
            assert!(d.component(occ.parent).unwrap().children.contains(&oid));
        }
        assert!(d.revision() > Revision(0));
        d.validate().unwrap();
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
        let unknown = body(999, 0);
        assert_eq!(d.body_props(unknown), BodyProps::default());
        let set: Vec<_> = d.explicit_body_props().map(|(b, _)| b).collect();
        assert_eq!(set.len(), 2);
        assert!((d.body_props(set[0]).opacity - 0.75).abs() < f32::EPSILON);
        d.set_body_props(set[1], BodyProps::default());
        assert_eq!(d.explicit_body_props().count(), 1);
    }

    #[test]
    fn rejects_cycles_and_missing_ids() {
        let mut d = sample();
        let root = d.root();
        let arm = d.components().find(|(_, c)| c.name == "Arm").unwrap().0;
        let bolt = d.components().find(|(_, c)| c.name == "Bolt").unwrap().0;
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
        let foreign = ExtKey {
            kind: "x".into(),
            id: 1_000_000,
        };
        assert!(d.set_ext(foreign, None).is_err());
    }

    #[test]
    fn snapshot_clone_shares_structure_and_is_independent() {
        let a: DesignSnapshot = Arc::new(sample());
        let mut b = (*a).clone();
        b.set_name("Renamed");
        b.set_body_props(body(1, 0), BodyProps::default());
        assert_eq!(a.meta().name, "Bracket");
        assert_eq!(a.component(a.root()).unwrap().name, "Bracket");
        assert_eq!(b.component(b.root()).unwrap().name, "Renamed");
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
                "root"
            ]
        );
        let s = serde_json::to_string(&d).unwrap();
        assert!(
            s.starts_with(r#"{"meta":{"uuid":"0190f5a3-0000-7000-8000-000000000001","name":"P","mode":"History","default_units":{"length":"Millimetre"},"tolerance":{"linear":1e-6,"angular":1e-9},"next_id":2},"root":1,"components":[[1,{"name":"P","children":[],"joints":[]}]]"#),
            "{s}"
        );
    }

    #[test]
    fn load_rejects_inconsistent_designs() {
        let good = serde_json::to_value(sample()).unwrap();
        type Edit = Box<dyn Fn(&mut serde_json::Value)>;
        let cases: Vec<(&str, Edit)> = vec![
            ("root", Box::new(|v| v["root"] = 4242.into())),
            ("next_id", Box::new(|v| v["meta"]["next_id"] = 2.into())),
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
        ];
        for (needle, edit) in cases {
            let mut v = good.clone();
            edit(&mut v);
            let e = serde_json::from_value::<Design>(v).unwrap_err().to_string();
            assert!(e.contains(needle), "expected `{needle}` in `{e}`");
        }
        serde_json::from_value::<Design>(good).unwrap();
    }

    #[test]
    fn thread_safety() {
        fn assert_send_sync<T: Send + Sync>() {}
        assert_send_sync::<Design>();
        assert_send_sync::<DesignSnapshot>();
    }
}
