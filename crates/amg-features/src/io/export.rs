//! Export side: selected document bodies → `amg_io::Scene` (world space through the roots).

use std::collections::BTreeMap;
use std::sync::Arc;

use amg_core::Transform;
use amg_doc::{BodyGeom, BodyId, ComponentId, Design, Evaluated};
use amg_io::{MaterialSlot, NO_SLOT, Scene, SceneNode, SceneObject, SlotIndex};
use amg_jobs::JobCtx;
use amg_mesh::{IndexedMesh, Rgba8};

use super::ImportPrepError;
use super::feature::{ImportMeshFeature, face_slots};

/// What to export.
#[derive(Clone, Debug)]
pub enum ExportSelection {
    /// Every body that is visible and has a visible placement.
    AllVisible,
    /// These bodies (every placement, visible or not).
    Bodies(Vec<BodyId>),
    /// The bodies placed through component `.0` (every placement under it).
    Component(ComponentId),
}

/// IO-09 tessellation presets (spec §9). The mapping onto S02 `TessQuality` is IO-09.
#[derive(Copy, Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum ExportRefinement {
    /// Coarse.
    Low,
    /// Default.
    Medium,
    /// Fine.
    High,
    /// Explicit deviations.
    Custom {
        /// Max distance between surface and facets, mm.
        surface_deviation_mm: f64,
        /// Max angle between adjacent facet normals, radians.
        normal_deviation_rad: f64,
    },
}

/// Export preparation: tessellation quality and slicer filaments.
#[derive(Clone, Debug, Default)]
pub struct ExportPrep {
    /// IO-09; required if solids are selected.
    pub refinement: Option<ExportRefinement>,
    /// Per-body override of `refinement` (IO-09).
    pub per_body_refinement: BTreeMap<BodyId, ExportRefinement>,
    /// Filament table from the S16 print setup (AM-01). When set it replaces the material
    /// table: object defaults and face slots are dropped and `body_filament` assigns slots.
    pub filaments: Option<Vec<MaterialSlot>>,
    /// 1-based filament per body (slot `n − 1` of `filaments`).
    pub body_filament: BTreeMap<BodyId, u8>,
}

/// Interns material slots so equal slots share one index.
#[derive(Default)]
struct Slots {
    list: Vec<MaterialSlot>,
}

impl Slots {
    fn intern(&mut self, s: &MaterialSlot) -> SlotIndex {
        if let Some(i) = self.list.iter().position(|x| x == s) {
            return u16::try_from(i).unwrap_or(NO_SLOT);
        }
        self.list.push(s.clone());
        u16::try_from(self.list.len() - 1).unwrap_or(NO_SLOT)
    }
}

/// An imported body that no later feature changed: export it as the object-space mesh with
/// the import transform on the root, like the file had it. `None` if the body is not an
/// import body or was modified.
fn unmodified_import<'d>(
    design: &'d Design,
    id: BodyId,
    mesh: &IndexedMesh,
) -> Option<(&'d ImportMeshFeature, IndexedMesh, Transform)> {
    let (_, node) = design.timeline().node(id.origin)?;
    let f = node.feature.downcast_ref::<ImportMeshFeature>()?;
    let spec = f.bodies.iter().find(|b| b.slot == id.slot)?;
    let raw = match &design.baked(id)?.geom {
        Some(BodyGeom::Mesh(m)) => Arc::clone(m),
        _ => return None,
    };
    if raw.tris != mesh.tris || raw.positions.len() != mesh.positions.len() {
        return None;
    }
    let expected = f.body_mesh(&raw, spec);
    if expected.positions != mesh.positions {
        return None;
    }
    let mut object = f.object_mesh(&raw);
    object.attrs = mesh.attrs.clone();
    object.groups = mesh.groups.clone();
    Some((f, object, f.object_transform(spec)))
}

/// The import feature of a body's origin, if it is one (its slot table survives in-place
/// mesh edits through `tri_properties`).
fn import_feature(design: &Design, id: BodyId) -> Option<&ImportMeshFeature> {
    let (_, node) = design.timeline().node(id.origin)?;
    node.feature.downcast_ref::<ImportMeshFeature>()
}

fn selected(design: &Design, eval: &Evaluated, sel: &ExportSelection) -> Vec<BodyId> {
    match sel {
        ExportSelection::AllVisible => eval
            .bodies()
            .filter(|(id, _)| design.body_props(*id).visible)
            .map(|(id, _)| id)
            .collect(),
        ExportSelection::Bodies(ids) => {
            let mut v = ids.clone();
            v.sort();
            v.dedup();
            v
        }
        ExportSelection::Component(_) => eval.bodies().map(|(id, _)| id).collect(),
    }
}

/// Placements of `id` the selection keeps, as world transforms.
fn placements(
    design: &Design,
    eval: &Evaluated,
    id: BodyId,
    sel: &ExportSelection,
) -> Vec<Transform> {
    let Some(body) = eval.body(id) else {
        return Vec::new();
    };
    eval.instances(id)
        .into_iter()
        .filter(|inst| match sel {
            ExportSelection::AllVisible => inst
                .path
                .iter()
                .all(|o| design.occurrence(*o).is_none_or(|o| o.visible)),
            ExportSelection::Bodies(_) => true,
            ExportSelection::Component(c) => {
                *c == design.root()
                    || *c == body.component
                    || inst
                        .path
                        .iter()
                        .any(|o| design.occurrence(*o).is_some_and(|o| o.component == *c))
            }
        })
        .map(|inst| inst.world)
        .collect()
}

/// Document → [`Scene`] in world space: one object per selected mesh body, one root per
/// placement (occurrence transforms go on the roots, not into the meshes).
///
/// - An imported body no feature changed is exported as its object-space mesh with the import
///   transform on the root, so a re-export reproduces the file's objects and build transforms.
/// - Materials: with `prep.filaments` the filament table and `body_filament`; otherwise the
///   import slot table (object default and per-face slots) and, for other bodies, the body's
///   appearance (name and sRGB colour, S17).
///
/// # Errors
/// [`ImportPrepError::NeedsTessellation`] for solid bodies (IO-09),
/// [`ImportPrepError::UnsupportedBody`] for implicit/form bodies,
/// [`ImportPrepError::EmptySelection`], or a cancellation.
pub fn scene_from_document(
    design: &Design,
    eval: &Evaluated,
    sel: &ExportSelection,
    prep: &ExportPrep,
    ctx: &JobCtx,
) -> Result<Scene, ImportPrepError> {
    let ids = selected(design, eval, sel);
    let mut solids = 0usize;
    let mut scene = Scene::default();
    let mut slots = Slots::default();
    if let Some(f) = &prep.filaments {
        slots.list.clone_from(f);
    }
    for id in ids {
        ctx.checkpoint().map_err(amg_io::IoError::from)?;
        let Some(body) = eval.body(id) else { continue };
        let mesh = match &body.geom {
            BodyGeom::Mesh(m) => Arc::clone(m),
            BodyGeom::Solid(_) => {
                solids += 1;
                continue;
            }
            BodyGeom::Implicit(_) | BodyGeom::Form(_) => {
                return Err(ImportPrepError::UnsupportedBody(id));
            }
        };
        let worlds = placements(design, eval, id, sel);
        if worlds.is_empty() {
            continue;
        }
        let (object_mesh, local) = match unmodified_import(design, id, &mesh) {
            Some((_, m, t)) => (Arc::new(m), t),
            None => (mesh, Transform::IDENTITY),
        };
        let props = design.body_props(id);
        let mut o = SceneObject::new(Arc::clone(&object_mesh));
        o.name.clone_from(&props.name);
        o.source_id = Some(format!("{}:{}", id.origin.0, id.slot));
        if prep.filaments.is_some() {
            if let Some(&n) = prep.body_filament.get(&id) {
                o.filament = Some(n);
                if n >= 1 && usize::from(n) <= slots.list.len() {
                    o.material = Some(u16::from(n - 1));
                }
            }
        } else if let Some(f) = import_feature(design, id) {
            let spec = f.bodies.iter().find(|b| b.slot == id.slot);
            let mut map = |s: u16| {
                f.materials
                    .get(usize::from(s))
                    .map_or(NO_SLOT, |m| slots.intern(m))
            };
            o.material = spec.and_then(|b| b.material).map(&mut map);
            o.filament = spec.and_then(|b| b.filament);
            if let Some(fs) = face_slots(&object_mesh) {
                let mapped: Vec<SlotIndex> = fs
                    .into_iter()
                    .map(|s| if s == NO_SLOT { NO_SLOT } else { map(s) })
                    .collect();
                o.face_materials = Some(Arc::from(mapped));
            }
        } else if let Some(a) = design.body_appearance(id) {
            o.material = Some(slots.intern(&MaterialSlot {
                name: a.name.clone(),
                color: Rgba8(a.base_color.0),
                filament: None,
            }));
        }
        let index =
            u32::try_from(scene.objects.len()).map_err(|_| amg_io::IoError::LimitExceeded {
                what: "objects",
                limit: u64::from(u32::MAX),
                actual: scene.objects.len() as u64,
            })?;
        scene.objects.push(o);
        for w in worlds {
            let mut n = SceneNode::mesh(index, w * local);
            n.name.clone_from(&props.name);
            scene.roots.push(n);
        }
    }
    if solids > 0 {
        return Err(ImportPrepError::NeedsTessellation(solids));
    }
    if scene.objects.is_empty() {
        return Err(ImportPrepError::EmptySelection);
    }
    scene.materials = slots.list;
    scene
        .metadata
        .insert("Title".to_owned(), design.meta().name.clone());
    scene.validate()?;
    Ok(scene)
}
