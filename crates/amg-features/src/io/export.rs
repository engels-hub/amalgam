//! Export side: selected document bodies → `amg_io::Scene` (world space through the roots).

use std::collections::BTreeMap;
use std::sync::Arc;

use amg_brep::{KernelError, TessQuality};
use amg_core::Transform;
use amg_doc::{BodyGeom, BodyId, ComponentId, Design, EvalBody, Evaluated};
use amg_io::{MaterialSlot, NO_SLOT, Scene, SceneNode, SceneObject, SlotIndex};
use amg_jobs::JobCtx;
use amg_mesh::{IndexedMesh, Rgba8};

use super::ImportPrepError;
use super::feature::{ImportMeshFeature, face_slots};
use super::tessellate::{DeviationEstimate, TessCache, TessFailure, tessellate_all};

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

/// Tessellation presets of solid bodies for mesh export (S12 §9, IO-09).
#[derive(Copy, Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum ExportRefinement {
    /// Coarse: 0.1 mm, 30°.
    Low,
    /// Default: 0.025 mm, 15°.
    #[default]
    Medium,
    /// Fine: 0.005 mm, 10°.
    High,
    /// Explicit deviations.
    Custom {
        /// Max distance between surface and facets, mm.
        surface_deviation_mm: f64,
        /// Max angle between adjacent facet normals, radians.
        normal_deviation_rad: f64,
    },
}

impl ExportRefinement {
    /// Max distance between surface and facets, mm.
    pub fn surface_deviation_mm(self) -> f64 {
        match self {
            ExportRefinement::Low => 0.1,
            ExportRefinement::Medium => 0.025,
            ExportRefinement::High => 0.005,
            ExportRefinement::Custom {
                surface_deviation_mm,
                ..
            } => surface_deviation_mm,
        }
    }

    /// Max angle between adjacent facet normals, radians.
    pub fn normal_deviation_rad(self) -> f64 {
        match self {
            ExportRefinement::Low => 30f64.to_radians(),
            ExportRefinement::Medium => 15f64.to_radians(),
            ExportRefinement::High => 10f64.to_radians(),
            ExportRefinement::Custom {
                normal_deviation_rad,
                ..
            } => normal_deviation_rad,
        }
    }

    /// The kernel quality (S02 §8): both deviations as absolute deflections. The kernel
    /// refuses a custom value that is not positive and finite.
    pub fn to_tess(self) -> TessQuality {
        TessQuality {
            linear: self.surface_deviation_mm(),
            angular: self.normal_deviation_rad(),
            relative: false,
            min_size: 0.0,
            max_linear_abs: None,
        }
    }
}

/// One solid body of an export: what was asked and what the mesh is estimated to give.
#[derive(Clone, Debug, PartialEq)]
pub struct BodyDeviation {
    /// The body.
    pub body: BodyId,
    /// Its name, if it has one.
    pub name: Option<String>,
    /// The preset it was meshed with.
    pub refinement: ExportRefinement,
    /// Triangles of the exported mesh (one placement).
    pub triangles: u64,
    /// Estimated deviation from the exact surface.
    pub estimate: DeviationEstimate,
    /// The mesh came from the tessellation cache.
    pub cached: bool,
}

impl BodyDeviation {
    /// True if the estimated surface deviation is above the requested one.
    pub fn exceeds_request(&self) -> bool {
        self.estimate.surface_mm > self.refinement.surface_deviation_mm() * (1.0 + 1e-6)
    }
}

/// The deviation report of an export: one entry per tessellated (solid) body, in body order.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct DeviationReport {
    /// The solid bodies.
    pub bodies: Vec<BodyDeviation>,
}

impl DeviationReport {
    /// The largest estimated surface deviation, mm (0 without solid bodies).
    pub fn max_surface_deviation_mm(&self) -> f64 {
        self.bodies
            .iter()
            .map(|b| b.estimate.surface_mm)
            .fold(0.0, f64::max)
    }
}

/// Export preparation: tessellation quality and slicer filaments.
#[derive(Clone, Debug, Default)]
pub struct ExportPrep {
    /// Tessellation preset of the solid bodies; every selected solid needs one from here or
    /// from `per_body_refinement`.
    pub refinement: Option<ExportRefinement>,
    /// Per-body override of `refinement`.
    pub per_body_refinement: BTreeMap<BodyId, ExportRefinement>,
    /// Filament table from the S16 print setup (AM-01). When set it replaces the material
    /// table: object defaults and face slots are dropped and `body_filament` assigns slots.
    pub filaments: Option<Vec<MaterialSlot>>,
    /// 1-based filament per body (slot `n − 1` of `filaments`).
    pub body_filament: BTreeMap<BodyId, u8>,
    /// Tessellation cache; `None` uses the process-wide [`TessCache::shared`].
    pub cache: Option<Arc<TessCache>>,
}

/// A selected body with its placements and, once meshed, its export mesh.
struct Selected {
    id: BodyId,
    body: Arc<EvalBody>,
    worlds: Vec<Transform>,
    mesh: Option<Arc<IndexedMesh>>,
}

/// Tessellates the solid bodies of `list` (filling their `mesh`) and reports their deviation.
fn tessellate_solids(
    design: &Design,
    list: &mut [Selected],
    prep: &ExportPrep,
    ctx: &JobCtx,
) -> Result<DeviationReport, ImportPrepError> {
    let mut requests = Vec::new();
    // (index into `list`, preset) per request.
    let mut owners = Vec::new();
    let mut missing = 0usize;
    for (i, s) in list.iter().enumerate() {
        let BodyGeom::Solid(shape) = &s.body.geom else {
            continue;
        };
        let Some(refinement) = prep
            .per_body_refinement
            .get(&s.id)
            .copied()
            .or(prep.refinement)
        else {
            missing += 1;
            continue;
        };
        let Some(brep) = shape.brep() else {
            // A body without a B-rep: say that the kernel is missing, if that is why.
            return Err(if amg_brep::OCCT_BACKEND {
                ImportPrepError::NoFaces(s.id)
            } else {
                ImportPrepError::Tessellation {
                    body: s.id,
                    source: KernelError::BackendUnavailable,
                }
            });
        };
        requests.push((brep.clone(), refinement.to_tess()));
        owners.push((i, refinement));
    }
    if missing > 0 {
        return Err(ImportPrepError::NeedsTessellation(missing));
    }
    let mut report = DeviationReport::default();
    if requests.is_empty() {
        return Ok(report);
    }
    let cache = prep.cache.as_ref().unwrap_or_else(|| TessCache::shared());
    let results = tessellate_all(cache, &requests, ctx);
    ctx.checkpoint().map_err(amg_io::IoError::from)?;
    for ((i, refinement), result) in owners.into_iter().zip(results) {
        let id = list[i].id;
        let (t, cached) = result.map_err(|e| match e {
            TessFailure::Kernel(KernelError::Cancelled) => {
                ImportPrepError::Io(amg_io::IoError::Cancelled)
            }
            TessFailure::Kernel(source) => ImportPrepError::Tessellation { body: id, source },
            TessFailure::Mesh(e) => ImportPrepError::Io(e.into()),
            TessFailure::NoTriangles => ImportPrepError::NoFaces(id),
        })?;
        report.bodies.push(BodyDeviation {
            body: id,
            name: design.body_props(id).name.clone(),
            refinement,
            triangles: t.mesh.tris.len() as u64,
            estimate: t.deviation,
            cached,
        });
        list[i].mesh = Some(Arc::clone(&t.mesh));
    }
    Ok(report)
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

/// Document → [`Scene`] in world space: one object per selected mesh or solid body, one root
/// per placement (occurrence transforms go on the roots, not into the meshes).
///
/// - A solid body is tessellated in component space with its [`ExportRefinement`] (BREP-08,
///   welded: a closed solid gives a closed mesh) through the tessellation cache.
/// - An imported body no feature changed is exported as its object-space mesh with the import
///   transform on the root, so a re-export reproduces the file's objects and build transforms.
/// - Materials: with `prep.filaments` the filament table and `body_filament`; otherwise the
///   import slot table (object default and per-face slots) and, for other bodies, the body's
///   appearance (name and sRGB colour, S17).
///
/// Tessellation can take long: call this from a job, not from the UI thread.
///
/// # Errors
/// [`ImportPrepError::NeedsTessellation`] for solid bodies without a refinement,
/// [`ImportPrepError::Tessellation`] when the kernel fails (or this build has none),
/// [`ImportPrepError::NoFaces`] for a solid body with nothing to mesh,
/// [`ImportPrepError::UnsupportedBody`] for implicit/form bodies,
/// [`ImportPrepError::EmptySelection`], or a cancellation.
pub fn scene_from_document(
    design: &Design,
    eval: &Evaluated,
    sel: &ExportSelection,
    prep: &ExportPrep,
    ctx: &JobCtx,
) -> Result<Scene, ImportPrepError> {
    scene_from_document_with_report(design, eval, sel, prep, ctx).map(|(scene, _)| scene)
}

/// [`scene_from_document`] plus the [`DeviationReport`] of the tessellated solid bodies.
///
/// # Errors
/// As [`scene_from_document`].
pub fn scene_from_document_with_report(
    design: &Design,
    eval: &Evaluated,
    sel: &ExportSelection,
    prep: &ExportPrep,
    ctx: &JobCtx,
) -> Result<(Scene, DeviationReport), ImportPrepError> {
    let mut list = Vec::new();
    for id in selected(design, eval, sel) {
        ctx.checkpoint().map_err(amg_io::IoError::from)?;
        let Some(body) = eval.body(id) else { continue };
        let mesh = match &body.geom {
            BodyGeom::Mesh(m) => Some(Arc::clone(m)),
            BodyGeom::Solid(_) => None,
            BodyGeom::Implicit(_) | BodyGeom::Form(_) => {
                return Err(ImportPrepError::UnsupportedBody(id));
            }
        };
        let worlds = placements(design, eval, id, sel);
        if worlds.is_empty() {
            continue;
        }
        list.push(Selected {
            id,
            body: Arc::clone(body),
            worlds,
            mesh,
        });
    }
    let report = tessellate_solids(design, &mut list, prep, ctx)?;
    let mut scene = Scene::default();
    let mut slots = Slots::default();
    if let Some(f) = &prep.filaments {
        slots.list.clone_from(f);
    }
    for Selected {
        id, worlds, mesh, ..
    } in list
    {
        ctx.checkpoint().map_err(amg_io::IoError::from)?;
        let Some(mesh) = mesh else { continue };
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
    if scene.objects.is_empty() {
        return Err(ImportPrepError::EmptySelection);
    }
    scene.materials = slots.list;
    scene
        .metadata
        .insert("Title".to_owned(), design.meta().name.clone());
    scene.validate()?;
    Ok((scene, report))
}
