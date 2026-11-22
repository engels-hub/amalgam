//! Orca/Bambu flavour (IO-04; spec S12 §6, note `import-export-formats.md` §3.1 and §4).
//!
//! - **Filaments.** A slot's filament is `MaterialSlot::filament`, else its position + 1. An
//!   object's filament is `SceneObject::filament`, else that of its default slot, else 1. The
//!   file gets one single-colour `m:colorgroup` per filament `1..=N` (group id = filament
//!   number, so the groups sort in filament order); an object's `pid` names its filament's
//!   group. Filaments nobody uses get a neutral placeholder colour.
//! - **Per-face filaments** are the unprefixed triangle attribute `paint_color` (whole-triangle
//!   states, [`crate::paint::encode_state`]) plus a triangle `pid`/`p1` into the face's colour
//!   group. Faces on the object's own filament carry neither.
//! - **`Metadata/model_settings.config`** carries object and part names, the `extruder` of every
//!   object and part, the part subtype and the plates of [`Scene::plates`]. The object id is the
//!   build item's resource id and the part id is the id of the mesh resource, which is how
//!   OrcaSlicer matches parts of a 3rd-party file (probe, note §3.1).

use std::collections::{BTreeSet, HashMap};

use amg_mesh::Rgba8;

use super::{GroupOut, IoWarning, MeshOut, NO_SLOT, Plan, escape, push_u64};
use crate::paint::MAX_STATE;
use crate::scene::{PartSubtype, Scene, SceneObject};

/// Colour of a filament that no slot describes.
const PLACEHOLDER: Rgba8 = Rgba8([0x80, 0x80, 0x80, 0xFF]);

/// The filament decisions of one export.
pub(super) struct OrcaPlan {
    /// Colour of filament `k + 1`.
    pub colors: Vec<Rgba8>,
    /// Filament of every material slot.
    pub slot_fil: Vec<u8>,
}

impl OrcaPlan {
    /// Number of colour groups (the largest filament in use).
    pub fn groups(&self) -> u32 {
        self.colors.len() as u32
    }

    /// The filament of `o` (see the module docs).
    pub fn object_filament(&self, o: &SceneObject) -> u8 {
        object_filament(o, &self.slot_fil)
    }
}

fn object_filament(o: &SceneObject, slot_fil: &[u8]) -> u8 {
    o.filament
        .filter(|&f| f > 0)
        .or_else(|| {
            o.material
                .filter(|&s| s != NO_SLOT)
                .and_then(|s| slot_fil.get(usize::from(s)).copied())
        })
        .unwrap_or(1)
        .min(MAX_STATE)
}

/// Decides the filaments in use by the placed objects in `order`.
pub(super) fn plan_filaments(
    scene: &Scene,
    order: &[u32],
    warnings: &mut Vec<IoWarning>,
) -> OrcaPlan {
    let mut clamped = 0u64;
    let slot_fil: Vec<u8> = scene
        .materials
        .iter()
        .enumerate()
        .map(|(k, m)| {
            let f = m
                .filament
                .filter(|&f| f > 0)
                .unwrap_or_else(|| u8::try_from(k + 1).unwrap_or(u8::MAX));
            if f > MAX_STATE {
                clamped += 1;
            }
            f.min(MAX_STATE)
        })
        .collect();
    let mut used: BTreeSet<u8> = BTreeSet::new();
    for &i in order {
        let o = &scene.objects[i as usize];
        let has_slot = o.material.is_some_and(|s| s != NO_SLOT);
        if has_slot || o.filament.is_some() {
            used.insert(object_filament(o, &slot_fil));
        }
        if let Some(fm) = &o.face_materials {
            let seen: BTreeSet<u16> = fm.iter().copied().filter(|&s| s != NO_SLOT).collect();
            for s in seen {
                if let Some(&f) = slot_fil.get(usize::from(s)) {
                    used.insert(f);
                }
            }
        }
    }
    if clamped > 0 {
        warnings.push(IoWarning::UnsupportedContentIgnored {
            what: format!(
                "filaments above {MAX_STATE} (clamped to {MAX_STATE}, the largest paint state)"
            ),
            count: clamped,
        });
    }
    let n = used.last().copied().unwrap_or(0);
    let colors = (1..=n)
        .map(|f| {
            scene
                .materials
                .iter()
                .zip(&slot_fil)
                .find(|&(_, &sf)| sf == f)
                .map_or(PLACEHOLDER, |(m, _)| m.color)
        })
        .collect();
    OrcaPlan { colors, slot_fil }
}

/// Appends ` name="…"`-style text: `<metadata key=".." value=".."/>`.
fn meta(out: &mut Vec<u8>, indent: &str, key: &str, value: &str) {
    out.extend_from_slice(indent.as_bytes());
    out.extend_from_slice(b"<metadata key=\"");
    out.extend_from_slice(key.as_bytes());
    out.extend_from_slice(b"\" value=\"");
    escape(out, value);
    out.extend_from_slice(b"\"/>\n");
}

fn subtype(p: PartSubtype) -> &'static str {
    match p {
        PartSubtype::NormalPart => "normal_part",
        PartSubtype::NegativePart => "negative_part",
        PartSubtype::Modifier => "modifier_part",
        PartSubtype::SupportEnforcer => "support_enforcer",
        PartSubtype::SupportBlocker => "support_blocker",
    }
}

/// The mesh resource ids under resource `id`, in first-use order, each once.
fn leaves(
    id: u32,
    meshes: &HashMap<u32, &MeshOut>,
    groups: &HashMap<u32, &GroupOut>,
    out: &mut Vec<u32>,
    seen: &mut BTreeSet<u32>,
) {
    if meshes.contains_key(&id) {
        if seen.insert(id) {
            out.push(id);
        }
    } else if let Some(g) = groups.get(&id) {
        for (c, _) in &g.children {
            leaves(*c, meshes, groups, out, seen);
        }
    }
}

/// `Metadata/model_settings.config`.
pub(super) fn model_settings(scene: &Scene, plan: &Plan) -> Vec<u8> {
    let meshes: HashMap<u32, &MeshOut> = plan.meshes.iter().map(|m| (m.id, m)).collect();
    let groups: HashMap<u32, &GroupOut> = plan.groups.iter().map(|g| (g.id, g)).collect();
    let extruders = plan.orca.as_ref().is_some_and(|o| o.groups() > 0);
    let mut out: Vec<u8> = Vec::with_capacity(1024);
    out.extend_from_slice(b"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<config>\n");
    let num = |v: u32| {
        let mut b = Vec::new();
        push_u64(&mut b, u64::from(v));
        String::from_utf8(b).unwrap_or_default()
    };

    // One <object> per distinct build-item resource, in first-use order.
    let mut written: BTreeSet<u32> = BTreeSet::new();
    for (k, (id, _)) in plan.items.iter().enumerate() {
        if !written.insert(*id) {
            continue;
        }
        let mut parts = Vec::new();
        leaves(*id, &meshes, &groups, &mut parts, &mut BTreeSet::new());
        let root_node = &scene.roots[plan.item_root[k] as usize];
        let first = parts.first().and_then(|p| meshes.get(p));
        let name = root_node
            .name
            .clone()
            .or_else(|| first.and_then(|m| scene.objects[m.object as usize].name.clone()));
        out.extend_from_slice(b"  <object id=\"");
        push_u64(&mut out, u64::from(*id));
        out.extend_from_slice(b"\">\n");
        if let Some(n) = &name {
            meta(&mut out, "    ", "name", n);
        }
        if extruders && let Some(m) = first {
            meta(&mut out, "    ", "extruder", &num(u32::from(m.fil)));
        }
        for p in &parts {
            let m = meshes[p];
            let o = &scene.objects[m.object as usize];
            out.extend_from_slice(b"    <part id=\"");
            push_u64(&mut out, u64::from(m.id));
            out.extend_from_slice(b"\" subtype=\"");
            out.extend_from_slice(subtype(o.part_subtype).as_bytes());
            out.extend_from_slice(b"\">\n");
            if let Some(n) = o
                .name
                .as_ref()
                .or(name.as_ref().filter(|_| parts.len() == 1))
            {
                meta(&mut out, "      ", "name", n);
            }
            if extruders {
                meta(&mut out, "      ", "extruder", &num(u32::from(m.fil)));
            }
            out.extend_from_slice(b"    </part>\n");
        }
        out.extend_from_slice(b"  </object>\n");
    }

    // Plates. The instance index is the position among the build items of the same resource
    // (what the reader and Orca count).
    if !scene.plates.is_empty() {
        let mut seen: HashMap<u32, u32> = HashMap::new();
        let mut inst: HashMap<u32, (u32, u32)> = HashMap::new(); // root → (resource, instance)
        for (k, (id, _)) in plan.items.iter().enumerate() {
            let n = seen.entry(*id).or_insert(0);
            inst.insert(plan.item_root[k], (*id, *n));
            *n += 1;
        }
        for (p, plate) in scene.plates.iter().enumerate() {
            out.extend_from_slice(b"  <plate>\n");
            meta(&mut out, "    ", "plater_id", &num(p as u32 + 1));
            meta(
                &mut out,
                "    ",
                "plater_name",
                plate.name.as_deref().unwrap_or(""),
            );
            let mut roots = plate.roots.clone();
            roots.sort_unstable();
            roots.dedup();
            for r in roots {
                let Some(&(obj, n)) = inst.get(&r) else {
                    continue;
                };
                out.extend_from_slice(b"    <model_instance>\n");
                meta(&mut out, "      ", "object_id", &num(obj));
                meta(&mut out, "      ", "instance_id", &num(n));
                out.extend_from_slice(b"    </model_instance>\n");
            }
            out.extend_from_slice(b"  </plate>\n");
        }
    }
    out.extend_from_slice(b"</config>\n");
    out
}
