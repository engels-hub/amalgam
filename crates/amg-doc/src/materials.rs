//! Materials and appearances inside the design (S17 MAT-06; additive S01 change).
//!
//! A design carries its own copy of every physical material and appearance its bodies use
//! (Fusion's "In this design"), so it opens with identical mass/FEA results on a machine
//! without the user's libraries. `BodyProps.material` / `.appearance` are keys into these
//! tables; a key that doesn't resolve (an unknown id, or a pre-MAT-06 free-form string) is
//! treated as unassigned.
//!
//! - Tables: [`Design::materials`], [`Design::appearances`]; persisted in `design.json` as
//!   `materials` / `appearances` arrays sorted by id, omitted while empty (files without
//!   materials are byte-identical to the format before MAT-06).
//! - Commands: [`Command::SetMaterial`], [`Command::RemoveMaterial`],
//!   [`Command::SetAppearance`], [`Command::RemoveAppearance`]; events
//!   [`DocEvent::MaterialChanged`](crate::DocEvent::MaterialChanged) /
//!   [`DocEvent::AppearanceChanged`](crate::DocEvent::AppearanceChanged).
//! - Command builders shared by the UI, the CLI and Python (S18): [`assign_material`],
//!   [`assign_material_by_id`], [`assign_appearance`], [`remove_unused_materials`],
//!   [`update_from_library`]. Apply the returned list in one transaction
//!   (`Document::transaction`), so it is one undo step.
//! - Mass properties of unassigned bodies: [`Design::mass_material`] with the
//!   `default_material` preference ([`MaterialPrefs`]).

use std::collections::BTreeSet;
use std::fmt;
use std::marker::PhantomData;
use std::sync::Arc;

use amg_materials::{
    Appearance, AppearanceId, Drift, LibrarySet, Material, MaterialId, MaterialPrefs,
};
use serde::de::{Deserializer, Error as _, SeqAccess, Visitor};
use serde::ser::{SerializeSeq, Serializer};
use serde::{Deserialize, Serialize};

use crate::command::{Command, CommandError};
use crate::id::BodyId;
use crate::model::{BodyProps, Design, ModelError};
use crate::pcoll::PMap;
use crate::stub::{AppearanceKey, MaterialKey};

// ---- keys ---------------------------------------------------------------------------------

impl MaterialKey {
    /// The key that refers to material `id`.
    pub fn of(id: &MaterialId) -> MaterialKey {
        MaterialKey(id.as_str().to_owned())
    }

    /// The material id this key holds, or `None` if it is not a valid `MaterialId` (a
    /// free-form key written before MAT-06).
    pub fn id(&self) -> Option<MaterialId> {
        MaterialId::parse(&self.0).ok()
    }
}

impl From<&MaterialId> for MaterialKey {
    fn from(id: &MaterialId) -> MaterialKey {
        MaterialKey::of(id)
    }
}

impl AppearanceKey {
    /// The key that refers to appearance `id`.
    pub fn of(id: &AppearanceId) -> AppearanceKey {
        AppearanceKey(id.as_str().to_owned())
    }

    /// The appearance id this key holds, or `None` if it is not a valid `AppearanceId`.
    pub fn id(&self) -> Option<AppearanceId> {
        AppearanceId::parse(&self.0).ok()
    }
}

impl From<&AppearanceId> for AppearanceKey {
    fn from(id: &AppearanceId) -> AppearanceKey {
        AppearanceKey::of(id)
    }
}

// ---- read access --------------------------------------------------------------------------

/// The material a mass-property caller should use for a body ([`Design::mass_material`]).
#[derive(Clone, Debug, PartialEq)]
pub struct MassMaterial {
    /// The material.
    pub material: Arc<Material>,
    /// True if the body has no (resolvable) material and the `default_material` preference
    /// was used; the UI marks such values as "(default material)".
    pub defaulted: bool,
}

impl Design {
    /// A material copy by id.
    pub fn material(&self, id: &MaterialId) -> Option<&Arc<Material>> {
        self.materials.get(id)
    }

    /// All material copies in id order.
    pub fn materials(&self) -> impl Iterator<Item = &Arc<Material>> + '_ {
        self.materials.values()
    }

    /// An appearance copy by id.
    pub fn appearance(&self, id: &AppearanceId) -> Option<&Arc<Appearance>> {
        self.appearances.get(id)
    }

    /// All appearance copies in id order.
    pub fn appearances(&self) -> impl Iterator<Item = &Arc<Appearance>> + '_ {
        self.appearances.values()
    }

    /// The body's material resolved against the design table; `None` if unassigned (or the
    /// key does not resolve).
    pub fn body_material(&self, body: BodyId) -> Option<&Arc<Material>> {
        let props = self.body_props.get(&body)?;
        self.materials.get(&props.material.as_ref()?.id()?)
    }

    /// The body's appearance: its own appearance key if it resolves, else the default
    /// appearance of its material (if that is in the design table).
    pub fn body_appearance(&self, body: BodyId) -> Option<&Arc<Appearance>> {
        let own = self
            .body_props
            .get(&body)
            .and_then(|p| p.appearance.as_ref())
            .and_then(AppearanceKey::id)
            .and_then(|id| self.appearances.get(&id));
        own.or_else(|| {
            let m = self.body_material(body)?;
            self.appearances.get(m.appearance.as_ref()?)
        })
    }

    /// Bodies whose props reference material `id`, in id order.
    pub fn material_users(&self, id: &MaterialId) -> Vec<BodyId> {
        self.explicit_body_props()
            .filter(|(_, p)| p.material.as_ref().is_some_and(|k| k.0 == id.as_str()))
            .map(|(b, _)| b)
            .collect()
    }

    /// Bodies whose props reference appearance `id` explicitly, in id order.
    pub fn appearance_users(&self, id: &AppearanceId) -> Vec<BodyId> {
        self.explicit_body_props()
            .filter(|(_, p)| p.appearance.as_ref().is_some_and(|k| k.0 == id.as_str()))
            .map(|(b, _)| b)
            .collect()
    }

    /// The material for the mass properties of `body` (S05 INS-03 / INS-10): the assigned
    /// copy, else `prefs.default_material`, looked up in the design table first and then in
    /// `libs`. `None` only if the default material is in neither.
    pub fn mass_material(
        &self,
        body: BodyId,
        prefs: &MaterialPrefs,
        libs: &LibrarySet,
    ) -> Option<MassMaterial> {
        if let Some(m) = self.body_material(body) {
            return Some(MassMaterial {
                material: Arc::clone(m),
                defaulted: false,
            });
        }
        let id = &prefs.default_material;
        let m = self.material(id).or_else(|| libs.material(id))?;
        Some(MassMaterial {
            material: Arc::clone(m),
            defaulted: true,
        })
    }

    // ---- edit (low level; the command layer builds on these) ----------------------------

    /// Inserts or replaces a material copy. The material must pass `Material::validate`.
    pub fn set_material(&mut self, m: Arc<Material>) -> Result<(), ModelError> {
        m.validate().map_err(|e| ModelError::InvalidValue {
            field: "material",
            msg: e.to_string(),
        })?;
        if self.materials.get(&m.id).is_some_and(|c| **c == *m) {
            return Ok(());
        }
        self.materials.insert(m.id.clone(), m);
        self.bump();
        Ok(())
    }

    /// Removes a material copy. Bodies that still reference it become unassigned; the
    /// command layer refuses that ([`Command::RemoveMaterial`]).
    pub fn remove_material(&mut self, id: &MaterialId) -> Result<(), ModelError> {
        if self.materials.remove(id).is_none() {
            return Err(missing("material", id.as_str()));
        }
        self.bump();
        Ok(())
    }

    /// Inserts or replaces an appearance copy. It must pass `Appearance::validate`.
    pub fn set_appearance(&mut self, a: Arc<Appearance>) -> Result<(), ModelError> {
        a.validate().map_err(|e| ModelError::InvalidValue {
            field: "appearance",
            msg: e.to_string(),
        })?;
        if self.appearances.get(&a.id).is_some_and(|c| **c == *a) {
            return Ok(());
        }
        self.appearances.insert(a.id.clone(), a);
        self.bump();
        Ok(())
    }

    /// Removes an appearance copy.
    pub fn remove_appearance(&mut self, id: &AppearanceId) -> Result<(), ModelError> {
        if self.appearances.remove(id).is_none() {
            return Err(missing("appearance", id.as_str()));
        }
        self.bump();
        Ok(())
    }

    /// Load-time checks of the two tables (called by [`Design::validate`]).
    pub(crate) fn validate_materials(&self) -> Result<(), String> {
        for (id, m) in &self.materials {
            if *id != m.id {
                return Err(format!("material table key `{id}` holds `{}`", m.id));
            }
            m.validate().map_err(|e| format!("material `{id}`: {e}"))?;
        }
        for (id, a) in &self.appearances {
            if *id != a.id {
                return Err(format!("appearance table key `{id}` holds `{}`", a.id));
            }
            a.validate()
                .map_err(|e| format!("appearance `{id}`: {e}"))?;
        }
        Ok(())
    }
}

fn missing(what: &'static str, id: &str) -> ModelError {
    ModelError::InvalidValue {
        field: what,
        msg: format!("no {what} `{id}` in this design"),
    }
}

/// Raw origin ids of `bodies` (sorted, deduplicated) for [`CommandError::InUse::by`].
pub(crate) fn origins(bodies: &[BodyId]) -> Vec<u64> {
    let set: BTreeSet<u64> = bodies.iter().map(|b| b.origin.0).collect();
    set.into_iter().collect()
}

// ---- command builders ---------------------------------------------------------------------

/// One transaction's worth of commands assigning `m` to `bodies`:
/// 1. `SetMaterial` if the design has no copy of `m.id` or a different one;
/// 2. `SetAppearance` for the material's default appearance if the design lacks it (looked up
///    in the design, then in the built-in library);
/// 3. `SetBodyProps` for every body whose props change. A body gets the default appearance
///    when it has none, or when its appearance is the default appearance of the material it
///    had before (so re-assigning a material re-colours bodies the user never coloured).
///
/// Same as [`assign_material_in`] with [`LibrarySet::builtin_only`].
pub fn assign_material(design: &Design, bodies: &[BodyId], m: &Arc<Material>) -> Vec<Command> {
    assign_material_in(design, &LibrarySet::builtin_only(), bodies, m)
}

/// [`assign_material`] that looks the default appearance up in `libs` (after the design).
pub fn assign_material_in(
    design: &Design,
    libs: &LibrarySet,
    bodies: &[BodyId],
    m: &Arc<Material>,
) -> Vec<Command> {
    let mut cmds = Vec::new();
    if design.material(&m.id).is_none_or(|c| **c != **m) {
        cmds.push(Command::SetMaterial {
            material: Arc::clone(m),
        });
    }
    let app = m
        .appearance
        .as_ref()
        .and_then(|id| design.appearance(id).or_else(|| libs.appearance(id)));
    let mut seen = BTreeSet::new();
    let mut props_cmds = Vec::new();
    let mut uses_app = false;
    for &body in bodies {
        if !seen.insert(body) {
            continue;
        }
        let old = design.body_props(body);
        let mut new = old.clone();
        new.material = Some(MaterialKey::of(&m.id));
        if let Some(a) = app {
            let old_default = design
                .body_material(body)
                .and_then(|om| om.appearance.as_ref())
                .map(AppearanceKey::of);
            if old.appearance.is_none() || old.appearance == old_default {
                new.appearance = Some(AppearanceKey::of(&a.id));
            }
            uses_app |= new
                .appearance
                .as_ref()
                .is_some_and(|k| k.0 == a.id.as_str());
        }
        if new != old {
            props_cmds.push(Command::SetBodyProps { body, props: new });
        }
    }
    if let Some(a) = app
        && uses_app
        && design.appearance(&a.id).is_none()
    {
        cmds.push(Command::SetAppearance {
            appearance: Arc::clone(a),
        });
    }
    cmds.append(&mut props_cmds);
    cmds
}

/// Phase 1 entry point for the CLI and Python (S18): assigns the material named by `spec` to
/// `bodies`. `spec` is a material id (`amg.builtin/petg.fff`), looked up in the design table
/// and then in `libs`, or else a name matched with `LibrarySet::match_name` ("PLA",
/// "PETG Basic"). Errors with `CommandError::Invalid` if nothing matches.
pub fn assign_material_by_id(
    design: &Design,
    libs: &LibrarySet,
    bodies: &[BodyId],
    spec: &str,
) -> Result<Vec<Command>, CommandError> {
    let by_id = MaterialId::parse(spec)
        .ok()
        .and_then(|id| design.material(&id).or_else(|| libs.material(&id)).cloned());
    let m = by_id
        .or_else(|| libs.match_name(spec).cloned())
        .ok_or_else(|| CommandError::Invalid(format!("no material `{spec}`")))?;
    Ok(assign_material_in(design, libs, bodies, &m))
}

/// Commands setting appearance `a` on `bodies` (`SetAppearance` if the design lacks this copy
/// or holds a different one, then `SetBodyProps` where the props change).
pub fn assign_appearance(design: &Design, bodies: &[BodyId], a: &Arc<Appearance>) -> Vec<Command> {
    let mut cmds = Vec::new();
    if design.appearance(&a.id).is_none_or(|c| **c != **a) {
        cmds.push(Command::SetAppearance {
            appearance: Arc::clone(a),
        });
    }
    let mut seen = BTreeSet::new();
    for &body in bodies {
        if !seen.insert(body) {
            continue;
        }
        let old = design.body_props(body);
        let new = BodyProps {
            appearance: Some(AppearanceKey::of(&a.id)),
            ..old.clone()
        };
        if new != old {
            cmds.push(Command::SetBodyProps { body, props: new });
        }
    }
    cmds
}

/// Removes every table entry no body references: `RemoveMaterial` for each material no body
/// uses, then `RemoveAppearance` for each appearance that no body references and that is not
/// the default appearance of a material that stays. Empty if nothing is unused.
pub fn remove_unused_materials(design: &Design) -> Vec<Command> {
    let mut used_m = BTreeSet::new();
    let mut used_a = BTreeSet::new();
    for (_, p) in design.explicit_body_props() {
        if let Some(id) = p.material.as_ref().and_then(MaterialKey::id) {
            used_m.insert(id);
        }
        if let Some(id) = p.appearance.as_ref().and_then(AppearanceKey::id) {
            used_a.insert(id);
        }
    }
    let mut cmds = Vec::new();
    for m in design.materials() {
        if used_m.contains(&m.id) {
            if let Some(a) = &m.appearance {
                used_a.insert(a.clone());
            }
        } else {
            cmds.push(Command::RemoveMaterial { id: m.id.clone() });
        }
    }
    for a in design.appearances() {
        if !used_a.contains(&a.id) {
            cmds.push(Command::RemoveAppearance { id: a.id.clone() });
        }
    }
    cmds
}

/// "Update from library": commands replacing the design's copy of `id` with the library entry
/// of the same id (and the copy of its default appearance, if the design holds a different
/// one). Empty if the copy is up to date. Also offered for a locally modified copy, where it
/// discards the local edits. Errors if the design has no such material or no library has it.
pub fn update_from_library(
    design: &Design,
    libs: &LibrarySet,
    id: &MaterialId,
) -> Result<Vec<Command>, CommandError> {
    let copy = design
        .material(id)
        .ok_or_else(|| CommandError::Invalid(format!("no material `{id}` in this design")))?;
    match libs.drift(copy) {
        Drift::UpToDate => return Ok(Vec::new()),
        Drift::NotInLibrary => {
            return Err(CommandError::Invalid(format!(
                "material `{id}` is in no loaded library"
            )));
        }
        Drift::LibraryNewer { .. } | Drift::LocallyModified => {}
    }
    let Some(lib) = libs.material(id) else {
        return Err(CommandError::Invalid(format!(
            "material `{id}` is in no loaded library"
        )));
    };
    let mut cmds = vec![Command::SetMaterial {
        material: Arc::clone(lib),
    }];
    if let Some(aid) = &lib.appearance
        && let (Some(la), Some(da)) = (libs.appearance(aid), design.appearance(aid))
        && **la != **da
    {
        cmds.push(Command::SetAppearance {
            appearance: Arc::clone(la),
        });
    }
    Ok(cmds)
}

// ---- serde --------------------------------------------------------------------------------

/// A table value that carries its own key.
pub(crate) trait Keyed {
    type Key: Ord + Clone + fmt::Display;
    fn key(&self) -> &Self::Key;
}

impl Keyed for Material {
    type Key = MaterialId;
    fn key(&self) -> &MaterialId {
        &self.id
    }
}

impl Keyed for Appearance {
    type Key = AppearanceId;
    fn key(&self) -> &AppearanceId {
        &self.id
    }
}

/// `PMap<id, Arc<V>>` as a JSON array of values in id order (each value holds its id).
/// Duplicate ids are rejected on load.
pub(crate) mod table {
    use super::*;

    pub(crate) fn serialize<V, S>(map: &PMap<V::Key, Arc<V>>, s: S) -> Result<S::Ok, S::Error>
    where
        V: Keyed + Serialize,
        S: Serializer,
    {
        let mut seq = s.serialize_seq(Some(map.len()))?;
        for v in map.values() {
            seq.serialize_element(&**v)?;
        }
        seq.end()
    }

    pub(crate) fn deserialize<'de, V, D>(d: D) -> Result<PMap<V::Key, Arc<V>>, D::Error>
    where
        V: Keyed + Deserialize<'de>,
        D: Deserializer<'de>,
    {
        struct Vis<V>(PhantomData<V>);
        impl<'de, V: Keyed + Deserialize<'de>> Visitor<'de> for Vis<V> {
            type Value = PMap<V::Key, Arc<V>>;
            fn expecting(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.write_str("an array of table entries")
            }
            fn visit_seq<A: SeqAccess<'de>>(self, mut seq: A) -> Result<Self::Value, A::Error> {
                let mut map = PMap::new();
                while let Some(v) = seq.next_element::<V>()? {
                    let k = v.key().clone();
                    if map.contains_key(&k) {
                        return Err(A::Error::custom(format_args!("duplicate id `{k}`")));
                    }
                    map.insert(k, Arc::new(v));
                }
                Ok(map)
            }
        }
        d.deserialize_seq(Vis(PhantomData))
    }
}

/// Serde for an `Arc<T>` field as plain `T` (the workspace does not enable serde's `rc`).
pub(crate) mod arc {
    use super::*;

    pub(crate) fn serialize<T: Serialize, S: Serializer>(
        v: &Arc<T>,
        s: S,
    ) -> Result<S::Ok, S::Error> {
        (**v).serialize(s)
    }

    pub(crate) fn deserialize<'de, T: Deserialize<'de>, D: Deserializer<'de>>(
        d: D,
    ) -> Result<Arc<T>, D::Error> {
        T::deserialize(d).map(Arc::new)
    }
}
