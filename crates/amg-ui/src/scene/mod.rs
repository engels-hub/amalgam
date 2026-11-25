//! The scene adapter (UI-20, spec §7, note §9): `DocEvent`s → S10 scene updates.
//!
//! [`SceneAdapter`] mirrors the active document's evaluated bodies as display meshes and
//! instances:
//! - Every displayed (occurrence path, body), sketch and overlay owner gets a
//!   [`RenderEntityKey`] (S10 `PickId`), allocated densely and never reused within a session,
//!   also not across `DocumentReplaced`. [`PickMap`] translates render hits back to
//!   [`PickTarget`]s and targets to render ids (highlights).
//! - Geometry goes through the [`BodyTessellator`] registry and is tessellated **and
//!   uploaded on a job**, latest-wins per body; the old mesh stays visible until the new one
//!   arrives, then the instances are repointed and the old mesh removed in one batch.
//!   Property, visibility and placement changes rewrite instances only (no upload).
//! - Preview bodies ([`SceneAdapter::set_preview`]) are drawn on `Layer::Preview` in the
//!   theme's preview colour; highlight layers are id lists resolved through the pick map.
//! - Sketches and construction geometry get pick keys only: they are drawn as RND-07
//!   overlays, not scene meshes.
//!
//! UI-thread work is bookkeeping only: event handling, spawning, and inserting finished
//! uploads ([`SceneBackend::apply`]). No `amg-doc` type crosses into `amg-render`.

pub mod backend;
mod raycast;
pub mod tess;

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::sync::{Arc, OnceLock};

use amg_core::{Point3, Ray3, Transform};
use amg_doc::{
    BodyGeom, BodyId, ComponentId, ConstructionId, Design, DocEvent, EvalBody, Evaluated, OccPath,
    Placements, SketchId, TopoKind,
};
use amg_jobs::{JobError, JobHandle, JobSystem, SpawnOptions};
use amg_render::{Layer, Material, RenderError};

pub use backend::{
    CpuSceneBackend, GpuSceneBackend, HighlightItem, HighlightLayer, InstanceDesc, InstanceSlot,
    MeshSlot, MeshUploader, Range32, RenderSubKind, SceneAccess, SceneBackend, SceneOp,
    UploadedMesh,
};
pub use tess::{BodyTessellator, DisplayMesh, DisplayQuality, MeshTessellator, TessellateError};

use crate::pick::{PickMode, PickTarget};
use crate::theme::ThemeTokens;
use raycast::Bvh;

/// How a workspace shows one body instead of its design placement (additive, AM-11: the
/// Manufacture workspace draws plate items at their plate transforms).
#[derive(Clone, Debug, PartialEq)]
pub struct BodyView {
    /// World transform of every instance of the body (replaces the occurrence placement, so
    /// picking, bounds and highlights follow it).
    pub transform: Transform,
    /// Linear RGBA that replaces the base colour (alpha = opacity); `None` keeps the material.
    pub tint: Option<[f32; 4]>,
    /// RND-08 shading of every instance of the body (additive, AM-12: the overhang overlay);
    /// `None` draws the material.
    pub shading: Option<amg_render::Shading>,
}

/// Per-body view overrides of committed bodies (additive, AM-11). While set, the listed
/// bodies are drawn with their [`BodyView`] and every other committed body is hidden (and not
/// pickable). Preview bodies are not affected.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BodyViews {
    /// The bodies shown, by id.
    pub bodies: BTreeMap<BodyId, BodyView>,
}

/// Per-body base colours (linear RGB) that replace the material colour while set, keeping the
/// body's opacity (additive, INS-10: "Display component colours"). Bodies not listed keep
/// their material; a [`BodyView::tint`] wins over it.
pub type BodyTints = BTreeMap<BodyId, [f32; 3]>;

/// Per-body section display (INS-09): which bodies the view's section planes clip
/// (`Instance::sectioned`) and their cap colours (linear RGB; `None` = RND-06's darkened base
/// colour).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SectionStyle {
    /// Bodies the planes clip; `None` = every body.
    pub clipped: Option<BTreeSet<BodyId>>,
    /// Cap colours by body.
    pub cap_colors: BTreeMap<BodyId, [f32; 3]>,
}

impl SectionStyle {
    /// `(sectioned, cap colour)` of `body`.
    pub fn of(&self, body: BodyId) -> (bool, Option<[f32; 3]>) {
        (
            self.clipped.as_ref().is_none_or(|c| c.contains(&body)),
            self.cap_colors.get(&body).copied(),
        )
    }
}

/// Render-side id of a pick owner: S10's `PickId` (0 means nothing; the adapter starts at 1).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Default)]
pub struct RenderEntityKey(pub u32);

impl RenderEntityKey {
    /// The S10 id written to the id buffer.
    pub fn pick_id(self) -> amg_render::PickId {
        amg_render::PickId(self.0)
    }
}

/// A render-side sub-entity: what S10 picking returns next to the key.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct RenderSub {
    /// Kind.
    pub kind: RenderSubKind,
    /// Sub-id (triangle of a mesh, face/edge/vertex of a B-rep, handle id).
    pub index: u32,
}

#[derive(Clone, Debug, PartialEq)]
enum Owner {
    Body { body: BodyId, instance: u32 },
    Sketch(SketchId),
    Construction(ConstructionId),
    Handles(Vec<PickTarget>),
}

/// Render key + sub-id ↔ [`PickTarget`] (spec §7). Owned by the [`SceneAdapter`].
#[derive(Debug, Default)]
pub struct PickMap {
    owners: HashMap<RenderEntityKey, Owner>,
    bodies: HashMap<(BodyId, u32), RenderEntityKey>,
    sketches: HashMap<SketchId, RenderEntityKey>,
    constructions: HashMap<ConstructionId, RenderEntityKey>,
    generation: u64,
}

fn topo_of(kind: RenderSubKind) -> Option<TopoKind> {
    match kind {
        RenderSubKind::Face => Some(TopoKind::Face),
        RenderSubKind::Edge => Some(TopoKind::Edge),
        RenderSubKind::Vertex => Some(TopoKind::Vertex),
        RenderSubKind::Handle => None,
    }
}

fn render_of(kind: TopoKind) -> Option<RenderSubKind> {
    match kind {
        TopoKind::Face => Some(RenderSubKind::Face),
        TopoKind::Edge => Some(RenderSubKind::Edge),
        TopoKind::Vertex => Some(RenderSubKind::Vertex),
        // A face group is a set of triangles, not one sub-id (MESH lane maps it).
        TopoKind::FaceGroup => None,
    }
}

impl PickMap {
    /// The document target of a render hit (`sub == None`: the whole owner).
    pub fn target(&self, key: RenderEntityKey, sub: Option<RenderSub>) -> Option<PickTarget> {
        match self.owners.get(&key)? {
            Owner::Body { body, instance } => {
                let sub = match sub {
                    None => None,
                    Some(s) => Some((topo_of(s.kind)?, s.index)),
                };
                Some(PickTarget::Body {
                    body: *body,
                    instance: *instance,
                    sub,
                })
            }
            Owner::Sketch(s) => Some(PickTarget::Sketch(*s)),
            Owner::Construction(c) => Some(PickTarget::Construction(*c)),
            Owner::Handles(list) => match sub {
                Some(RenderSub {
                    kind: RenderSubKind::Handle,
                    index,
                }) => list.get(index as usize).copied(),
                _ => None,
            },
        }
    }

    /// The render key and sub-id of a target (highlights). `None` if it is not displayed.
    pub fn render(&self, t: &PickTarget) -> Option<(RenderEntityKey, Option<RenderSub>)> {
        match t {
            PickTarget::Body {
                body,
                instance,
                sub,
            } => {
                let key = *self.bodies.get(&(*body, *instance))?;
                let sub = match sub {
                    None => None,
                    Some((kind, index)) => Some(RenderSub {
                        kind: render_of(*kind)?,
                        index: *index,
                    }),
                };
                Some((key, sub))
            }
            PickTarget::Sketch(s) => self.sketches.get(s).map(|k| (*k, None)),
            PickTarget::Construction(c) => self.constructions.get(c).map(|k| (*k, None)),
            PickTarget::Occurrence(_) => None,
        }
    }

    /// The key of a body instance.
    pub fn body_key(&self, body: BodyId, instance: u32) -> Option<RenderEntityKey> {
        self.bodies.get(&(body, instance)).copied()
    }

    /// Every key of a body (one per occurrence path).
    pub fn body_keys(&self, body: BodyId) -> Vec<RenderEntityKey> {
        let mut v: Vec<_> = self
            .bodies
            .iter()
            .filter(|((b, _), _)| *b == body)
            .map(|(_, k)| *k)
            .collect();
        v.sort_unstable();
        v
    }

    /// Number of live keys.
    pub fn len(&self) -> usize {
        self.owners.len()
    }

    /// True if nothing is mapped.
    pub fn is_empty(&self) -> bool {
        self.owners.is_empty()
    }

    /// The occurrence-path indices a body is displayed through, ascending (UI-04: restoring
    /// a stored body reference).
    pub fn instances_of(&self, body: BodyId) -> Vec<u32> {
        let mut v: Vec<u32> = self
            .bodies
            .keys()
            .filter(|(b, _)| *b == body)
            .map(|(_, i)| *i)
            .collect();
        v.sort_unstable();
        v
    }

    /// Every live key, sorted.
    pub fn keys(&self) -> Vec<RenderEntityKey> {
        let mut v: Vec<_> = self.owners.keys().copied().collect();
        v.sort_unstable();
        v
    }

    /// Bumped on every change (highlight re-resolution).
    pub fn generation(&self) -> u64 {
        self.generation
    }

    fn insert(&mut self, key: RenderEntityKey, owner: Owner) {
        match &owner {
            Owner::Body { body, instance } => {
                self.bodies.insert((*body, *instance), key);
            }
            Owner::Sketch(s) => {
                self.sketches.insert(*s, key);
            }
            Owner::Construction(c) => {
                self.constructions.insert(*c, key);
            }
            Owner::Handles(_) => {}
        }
        self.owners.insert(key, owner);
        self.generation += 1;
    }

    fn remove(&mut self, key: RenderEntityKey) {
        match self.owners.remove(&key) {
            Some(Owner::Body { body, instance }) => {
                self.bodies.remove(&(body, instance));
            }
            Some(Owner::Sketch(s)) => {
                self.sketches.remove(&s);
            }
            Some(Owner::Construction(c)) => {
                self.constructions.remove(&c);
            }
            Some(Owner::Handles(_)) | None => {}
        }
        self.generation += 1;
    }

    fn clear(&mut self) {
        self.owners.clear();
        self.bodies.clear();
        self.sketches.clear();
        self.constructions.clear();
        self.generation += 1;
    }
}

/// A CPU ray-cast hit in render terms.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SceneHit {
    /// The instance's key.
    pub key: RenderEntityKey,
    /// The face sub-id (triangle of a mesh, face of a B-rep).
    pub sub: RenderSub,
    /// Distance along the world ray, mm.
    pub t: f64,
    /// World position, mm.
    pub world: Point3,
}

/// Counters for tests and the diagnostics overlay.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct SceneStats {
    /// Tessellation jobs spawned.
    pub tessellations: u64,
    /// Uploads inserted into the scene.
    pub inserted: u64,
    /// Results dropped because a newer geometry superseded them.
    pub superseded: u64,
    /// Failed tessellations or uploads.
    pub failed: u64,
    /// Full rebuilds (`DocumentReplaced`, document switch).
    pub rebuilds: u64,
    /// Bodies with no tessellator for their geometry.
    pub undisplayable: u64,
}

/// Error of a tessellation job.
#[derive(Debug, thiserror::Error)]
pub enum DisplayError {
    /// The tessellator failed.
    #[error(transparent)]
    Tessellate(#[from] TessellateError),
    /// The upload failed.
    #[error(transparent)]
    Upload(#[from] RenderError),
    /// Cancelled.
    #[error(transparent)]
    Cancelled(#[from] amg_core::Cancelled),
}

type TessOutput = (Arc<DisplayMesh>, UploadedMesh);

#[derive(Debug)]
struct LiveMesh {
    slot: MeshSlot,
    cpu: Arc<DisplayMesh>,
    bvh: OnceLock<Bvh>,
    /// Mesh-local bounds, computed on first use (scene bounds for navigation, UI-03).
    bounds: OnceLock<amg_core::Aabb3>,
}

impl LiveMesh {
    fn local_bounds(&self) -> amg_core::Aabb3 {
        *self.bounds.get_or_init(|| {
            let o = self.cpu.data.origin;
            amg_core::Aabb3::from_points(
                self.cpu
                    .data
                    .positions
                    .iter()
                    .map(|p| o + Point3::new(f64::from(p[0]), f64::from(p[1]), f64::from(p[2]))),
            )
        })
    }
}

#[derive(Debug)]
struct Inst {
    path: u32,
    key: RenderEntityKey,
    slot: InstanceSlot,
    world: Transform,
    visible: bool,
    /// The description last sent (None: not in the scene yet).
    sent: Option<InstanceDesc>,
}

struct Entry {
    geom: BodyGeom,
    component: ComponentId,
    mesh: Option<LiveMesh>,
    pending: Option<JobHandle<TessOutput, DisplayError>>,
    instances: Vec<Inst>,
    visible: bool,
    opacity: f32,
    uploads: u32,
    /// Clipped by the view's section planes, and the cap colour (INS-09, [`SectionStyle`]).
    section: (bool, Option<[f32; 3]>),
    /// Base colour from the [`BodyViews`] override (AM-11).
    view_tint: Option<[f32; 4]>,
    /// RND-08 shading from the [`BodyViews`] override (AM-12).
    view_shading: Option<amg_render::Shading>,
    /// Base colour from [`BodyTints`] (INS-10).
    tint: Option<[f32; 3]>,
}

impl std::fmt::Debug for Entry {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Entry")
            .field("component", &self.component)
            .field("mesh", &self.mesh.as_ref().map(|m| m.slot))
            .field("pending", &self.pending.is_some())
            .field("instances", &self.instances.len())
            .finish_non_exhaustive()
    }
}

/// Committed bodies or preview bodies.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Set {
    Model,
    Preview,
}

/// Allocators, the op list and the pick map: everything an entry update touches besides the
/// entry itself.
struct Core {
    backend: Box<dyn SceneBackend>,
    uploader: Arc<dyn MeshUploader>,
    tessellators: Vec<Arc<dyn BodyTessellator>>,
    quality: DisplayQuality,
    next_key: u32,
    next_mesh: u64,
    next_inst: u64,
    ops: Vec<SceneOp>,
    pick: PickMap,
    paths: Vec<OccPath>,
    path_index: HashMap<OccPath, u32>,
    stats: SceneStats,
    model_material: Material,
    preview_material: Material,
    pending: BTreeSet<(Set, BodyId)>,
    /// Per-body view overrides of committed bodies (AM-11).
    views: Option<Arc<BodyViews>>,
    /// Per-body base colours of committed bodies (INS-10, component colours).
    tints: Option<Arc<BodyTints>>,
}

/// The view of the document an update reads.
struct Src<'a> {
    design: &'a Design,
    placements: &'a Placements,
}

impl Core {
    fn key(&mut self) -> RenderEntityKey {
        self.next_key += 1;
        RenderEntityKey(self.next_key)
    }

    fn path(&mut self, p: &OccPath) -> u32 {
        if let Some(i) = self.path_index.get(p) {
            return *i;
        }
        let i = u32::try_from(self.paths.len()).unwrap_or(u32::MAX);
        self.paths.push(p.clone());
        self.path_index.insert(p.clone(), i);
        i
    }

    fn spawn(&mut self, set: Set, id: BodyId, e: &mut Entry, jobs: &JobSystem) {
        if let Some(old) = e.pending.take() {
            old.cancel();
            self.stats.superseded += 1;
        }
        let Some(t) = tess::find(&self.tessellators, &e.geom) else {
            tracing::debug!(?id, "scene: no tessellator for this body's geometry");
            self.stats.undisplayable += 1;
            self.pending.remove(&(set, id));
            return;
        };
        let geom = e.geom.clone();
        let uploader = Arc::clone(&self.uploader);
        let quality = self.quality;
        let label = match set {
            Set::Model => "Display body",
            Set::Preview => "Display preview body",
        };
        let handle = jobs.spawn_fn(label, SpawnOptions::default(), move |ctx| {
            let mesh = t.tessellate(&geom, quality, ctx)?;
            ctx.checkpoint()?;
            let up = uploader.upload(&mesh, ctx)?;
            Ok::<_, DisplayError>((Arc::new(mesh), up))
        });
        e.pending = Some(handle);
        self.pending.insert((set, id));
        self.stats.tessellations += 1;
    }

    fn material(&self, set: Set, e: &Entry) -> Material {
        match set {
            Set::Model => {
                let mut m = self.model_material;
                m.base_color[3] = e.opacity.clamp(0.0, 1.0);
                m
            }
            Set::Preview => self.preview_material,
        }
    }

    /// Recomputes the entry's instances from the placements and sends what changed.
    fn sync_instances(&mut self, set: Set, id: BodyId, e: &mut Entry, src: &Src<'_>) {
        let mut wanted: Vec<(u32, Transform, bool)> = Vec::new();
        e.tint = match set {
            Set::Model => self.tints.as_ref().and_then(|t| t.get(&id).copied()),
            Set::Preview => None,
        };
        let views = self.views.clone();
        let view = match (set, views.as_deref()) {
            (Set::Model, Some(v)) => Some(v.bodies.get(&id)),
            _ => None,
        };
        e.view_tint = view.flatten().and_then(|v| v.tint);
        e.view_shading = view.flatten().and_then(|v| v.shading.clone());
        for path in src.placements.paths_of(e.component) {
            let Some(world) = src.placements.world(&path) else {
                continue;
            };
            let occ_visible = path
                .iter()
                .all(|o| src.design.occurrence(*o).is_none_or(|occ| occ.visible));
            let (world, occ_visible) = match view {
                None => (world, occ_visible),
                Some(Some(v)) => (v.transform, occ_visible),
                Some(None) => (world, false),
            };
            let idx = self.path(&path);
            wanted.push((idx, world, occ_visible));
        }
        // Drop instances whose path is gone.
        let mut kept = Vec::with_capacity(wanted.len());
        for inst in e.instances.drain(..) {
            if wanted.iter().any(|(p, _, _)| *p == inst.path) {
                kept.push(inst);
            } else {
                if inst.sent.is_some() {
                    self.ops.push(SceneOp::RemoveInstance(inst.slot));
                }
                if set == Set::Model {
                    self.pick.remove(inst.key);
                }
            }
        }
        for (path, world, occ_visible) in wanted {
            let visible = e.visible && occ_visible;
            if let Some(inst) = kept.iter_mut().find(|i| i.path == path) {
                inst.world = world;
                inst.visible = visible;
            } else {
                let key = self.key();
                self.next_inst += 1;
                let slot = InstanceSlot(self.next_inst);
                if set == Set::Model {
                    self.pick.insert(
                        key,
                        Owner::Body {
                            body: id,
                            instance: path,
                        },
                    );
                }
                kept.push(Inst {
                    path,
                    key,
                    slot,
                    world,
                    visible,
                    sent: None,
                });
            }
        }
        e.instances = kept;
        self.send_instances(set, e);
    }

    /// Sends every instance whose description differs from what the backend has.
    fn send_instances(&mut self, set: Set, e: &mut Entry) {
        let Some(mesh) = e.mesh.as_ref().map(|m| m.slot) else {
            return;
        };
        let mut material = self.material(set, e);
        if let Some([r, g, b]) = e.tint {
            material.base_color = [r, g, b, material.base_color[3]];
        }
        if let Some(tint) = e.view_tint {
            material.base_color = tint;
        }
        let sectioned = match set {
            Set::Model => {
                material.cap_color = e.section.1;
                e.section.0
            }
            Set::Preview => true,
        };
        let layer = match set {
            Set::Model => Layer::Model,
            Set::Preview => Layer::Preview,
        };
        let shading = match set {
            Set::Model => e.view_shading.clone().unwrap_or_default(),
            Set::Preview => amg_render::Shading::Material,
        };
        for inst in &mut e.instances {
            let desc = InstanceDesc {
                key: inst.key,
                mesh,
                transform: inst.world,
                visible: inst.visible,
                layer,
                material,
                sectioned,
                shading: shading.clone(),
            };
            if inst.sent.as_ref() != Some(&desc) {
                self.ops.push(SceneOp::SetInstance {
                    slot: inst.slot,
                    desc: desc.clone(),
                });
                inst.sent = Some(desc);
            }
        }
    }

    /// Removes everything of an entry from the scene and the pick map.
    fn drop_entry(&mut self, set: Set, id: BodyId, mut e: Entry) {
        if let Some(p) = e.pending.take() {
            p.cancel();
        }
        self.pending.remove(&(set, id));
        for inst in e.instances.drain(..) {
            if inst.sent.is_some() {
                self.ops.push(SceneOp::RemoveInstance(inst.slot));
            }
            if set == Set::Model {
                self.pick.remove(inst.key);
            }
        }
        if let Some(m) = e.mesh.take() {
            self.ops.push(SceneOp::RemoveMesh(m.slot));
        }
    }

    /// Inserts a finished upload: new mesh, instances repointed, old mesh removed.
    fn install(&mut self, set: Set, e: &mut Entry, cpu: Arc<DisplayMesh>, up: UploadedMesh) {
        self.next_mesh += 1;
        let slot = MeshSlot(self.next_mesh);
        self.ops.push(SceneOp::InsertMesh { slot, mesh: up });
        let old = e.mesh.replace(LiveMesh {
            slot,
            cpu,
            bvh: OnceLock::new(),
            bounds: OnceLock::new(),
        });
        e.uploads += 1;
        self.stats.inserted += 1;
        self.send_instances(set, e);
        if let Some(old) = old {
            self.ops.push(SceneOp::RemoveMesh(old.slot));
        }
    }

    fn flush(&mut self) {
        if !self.ops.is_empty() {
            let ops = std::mem::take(&mut self.ops);
            self.backend.apply(ops);
        }
    }
}

/// `DocEvent` → render scene (spec §7). One per app; it shows the active document.
pub struct SceneAdapter {
    core: Core,
    bodies: BTreeMap<BodyId, Entry>,
    preview: BTreeMap<BodyId, Entry>,
    preview_src: Option<Arc<Evaluated>>,
    overlay: Option<Arc<Placements>>,
    highlight_targets: [Vec<PickTarget>; 4],
    highlight_items: [Vec<HighlightItem>; 4],
    highlight_colors: [[f32; 4]; 4],
    highlight_generation: u64,
    failures: Vec<(BodyId, String)>,
}

impl std::fmt::Debug for SceneAdapter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SceneAdapter")
            .field("bodies", &self.bodies.len())
            .field("preview", &self.preview.len())
            .field("keys", &self.core.pick.len())
            .field("pending", &self.core.pending.len())
            .field("stats", &self.core.stats)
            .finish_non_exhaustive()
    }
}

fn rgba(c: egui::Color32, alpha: f32) -> [f32; 4] {
    let l = egui::Rgba::from(c);
    [l.r(), l.g(), l.b(), alpha]
}

impl SceneAdapter {
    /// An empty adapter over `backend`, converting geometry with `tessellators` (last match
    /// wins, [`UiRegistry::tessellators`](crate::registry::UiRegistry::tessellators)).
    pub fn new(
        backend: Box<dyn SceneBackend>,
        tessellators: Vec<Arc<dyn BodyTessellator>>,
    ) -> SceneAdapter {
        let uploader = backend.uploader();
        let mut a = SceneAdapter {
            core: Core {
                backend,
                uploader,
                tessellators,
                quality: DisplayQuality::Normal,
                next_key: 0,
                next_mesh: 0,
                next_inst: 0,
                ops: Vec::new(),
                pick: PickMap::default(),
                paths: Vec::new(),
                path_index: HashMap::new(),
                stats: SceneStats::default(),
                model_material: Material::default(),
                preview_material: Material::default(),
                pending: BTreeSet::new(),
                views: None,
                tints: None,
            },
            bodies: BTreeMap::new(),
            preview: BTreeMap::new(),
            preview_src: None,
            overlay: None,
            highlight_targets: Default::default(),
            highlight_items: Default::default(),
            highlight_colors: [[0.0; 4]; 4],
            highlight_generation: u64::MAX,
            failures: Vec::new(),
        };
        a.set_theme(&ThemeTokens::dark());
        a
    }

    // ---- document input -------------------------------------------------------------------

    /// Applies drained `DocEvent`s; `design` and `evaluated` are the document's current
    /// state. Geometry changes spawn tessellation jobs on `jobs`; everything else is applied
    /// to the backend before this returns.
    pub fn handle_events(
        &mut self,
        events: &[DocEvent],
        design: &Design,
        evaluated: &Evaluated,
        jobs: &JobSystem,
    ) {
        if events.is_empty() {
            return;
        }
        if events
            .iter()
            .any(|e| matches!(e, DocEvent::DocumentReplaced))
        {
            self.rebuild(design, evaluated, jobs);
            return;
        }
        let mut touched: BTreeSet<BodyId> = BTreeSet::new();
        let mut placements = false;
        let mut sketches: BTreeSet<SketchId> = BTreeSet::new();
        let mut constructions: BTreeSet<ConstructionId> = BTreeSet::new();
        for ev in events {
            match ev {
                DocEvent::BodyAdded(b)
                | DocEvent::BodyRemoved(b)
                | DocEvent::BodyGeometryChanged(b)
                | DocEvent::BodyPropsChanged(b) => {
                    touched.insert(*b);
                }
                DocEvent::ComponentTreeChanged
                | DocEvent::OccurrenceTransformChanged(_)
                | DocEvent::OccurrenceVisibilityChanged(_) => placements = true,
                DocEvent::SketchChanged(s) => {
                    sketches.insert(*s);
                }
                DocEvent::ConstructionChanged(c) => {
                    constructions.insert(*c);
                }
                _ => {}
            }
        }
        let src = Src {
            design,
            placements: self.overlay.as_deref().unwrap_or(evaluated.placements()),
        };
        for id in &touched {
            reconcile(
                &mut self.core,
                &mut self.bodies,
                Set::Model,
                *id,
                evaluated.body(*id),
                &src,
                jobs,
            );
        }
        if placements {
            for (id, e) in &mut self.bodies {
                if !touched.contains(id) {
                    self.core.sync_instances(Set::Model, *id, e, &src);
                }
            }
            // Preview bodies follow the preview's own placements (the next `set_preview`).
        }
        for s in sketches {
            self.sync_sketch(s, evaluated);
        }
        for c in constructions {
            self.sync_construction(c, evaluated);
        }
        self.flush();
    }

    /// Clears the scene and builds it again from `evaluated` (`DocumentReplaced`, document
    /// switch). Keys are fresh: none of the old ones is reused.
    pub fn rebuild(&mut self, design: &Design, evaluated: &Evaluated, jobs: &JobSystem) {
        self.clear();
        self.core.stats.rebuilds += 1;
        let src = Src {
            design,
            placements: self.overlay.as_deref().unwrap_or(evaluated.placements()),
        };
        for (id, b) in evaluated.bodies() {
            reconcile(
                &mut self.core,
                &mut self.bodies,
                Set::Model,
                id,
                Some(b),
                &src,
                jobs,
            );
        }
        let sketches: Vec<SketchId> = evaluated.sketches().map(|(id, _)| id).collect();
        for s in sketches {
            self.sync_sketch(s, evaluated);
        }
        let constructions: Vec<ConstructionId> =
            evaluated.constructions().map(|(id, _)| id).collect();
        for c in constructions {
            self.sync_construction(c, evaluated);
        }
        self.flush();
    }

    /// Removes everything this adapter put into the scene (pending jobs are cancelled).
    pub fn clear(&mut self) {
        for (id, e) in std::mem::take(&mut self.bodies) {
            self.core.drop_entry(Set::Model, id, e);
        }
        for (id, e) in std::mem::take(&mut self.preview) {
            self.core.drop_entry(Set::Preview, id, e);
        }
        self.preview_src = None;
        self.core.pick.clear();
        self.flush();
    }

    fn sync_sketch(&mut self, s: SketchId, evaluated: &Evaluated) {
        let present = evaluated.sketch(s).is_some();
        match (present, self.core.pick.sketches.get(&s).copied()) {
            (true, None) => {
                let k = self.core.key();
                self.core.pick.insert(k, Owner::Sketch(s));
            }
            (false, Some(k)) => self.core.pick.remove(k),
            _ => {}
        }
    }

    fn sync_construction(&mut self, c: ConstructionId, evaluated: &Evaluated) {
        let present = evaluated.construction(c).is_some();
        match (present, self.core.pick.constructions.get(&c).copied()) {
            (true, None) => {
                let k = self.core.key();
                self.core.pick.insert(k, Owner::Construction(c));
            }
            (false, Some(k)) => self.core.pick.remove(k),
            _ => {}
        }
    }

    /// Takes finished tessellations and inserts them (never blocks). Returns true if the
    /// scene changed.
    pub fn poll(&mut self) -> bool {
        let mut changed = false;
        if !self.core.pending.is_empty() {
            let pending: Vec<(Set, BodyId)> = self.core.pending.iter().copied().collect();
            for (set, id) in pending {
                let map = match set {
                    Set::Model => &mut self.bodies,
                    Set::Preview => &mut self.preview,
                };
                let Some(e) = map.get_mut(&id) else {
                    self.core.pending.remove(&(set, id));
                    continue;
                };
                let Some(res) = e.pending.as_mut().and_then(JobHandle::try_take) else {
                    if e.pending.is_none() {
                        self.core.pending.remove(&(set, id));
                    }
                    continue;
                };
                e.pending = None;
                self.core.pending.remove(&(set, id));
                match res {
                    Ok((cpu, up)) => {
                        self.core.install(set, e, cpu, up);
                        changed = true;
                    }
                    Err(JobError::Cancelled) => {}
                    Err(err) => {
                        self.core.stats.failed += 1;
                        tracing::warn!(?id, "scene: cannot display the body: {err}");
                        if set == Set::Model {
                            self.failures.push((id, err.to_string()));
                        }
                    }
                }
            }
        }
        self.flush();
        changed
    }

    /// Failures since the last call (the app shows them as toasts).
    pub fn take_failures(&mut self) -> Vec<(BodyId, String)> {
        std::mem::take(&mut self.failures)
    }

    // ---- preview, placements, highlights, theme -------------------------------------------

    /// Shows the bodies of a preview `Evaluated` that differ from `committed` on the preview
    /// layer (replaced as a whole; `None` clears it). Unchanged preview bodies keep their
    /// upload between calls.
    pub fn set_preview(
        &mut self,
        preview: Option<Arc<Evaluated>>,
        design: &Design,
        committed: &Evaluated,
        jobs: &JobSystem,
    ) {
        if match (&preview, &self.preview_src) {
            (Some(a), Some(b)) => Arc::ptr_eq(a, b),
            (None, None) => true,
            _ => false,
        } {
            return;
        }
        let Some(p) = preview else {
            for (id, e) in std::mem::take(&mut self.preview) {
                self.core.drop_entry(Set::Preview, id, e);
            }
            self.preview_src = None;
            self.flush();
            return;
        };
        let wanted: BTreeMap<BodyId, &Arc<EvalBody>> = p
            .bodies()
            .filter(|(id, b)| committed.body(*id).is_none_or(|c| !Arc::ptr_eq(c, b)))
            .collect();
        let gone: Vec<BodyId> = self
            .preview
            .keys()
            .filter(|id| !wanted.contains_key(id))
            .copied()
            .collect();
        for id in gone {
            if let Some(e) = self.preview.remove(&id) {
                self.core.drop_entry(Set::Preview, id, e);
            }
        }
        let src = Src {
            design,
            placements: p.placements(),
        };
        for (id, b) in &wanted {
            reconcile(
                &mut self.core,
                &mut self.preview,
                Set::Preview,
                *id,
                Some(*b),
                &src,
                jobs,
            );
        }
        self.preview_src = Some(Arc::clone(&p));
        self.flush();
    }

    /// Transient placements for drag frames (S06 ASM-12): dragged instances move without a
    /// recompute. `None` returns to the evaluated placements.
    pub fn set_placement_overlay(
        &mut self,
        overlay: Option<Arc<Placements>>,
        design: &Design,
        evaluated: &Evaluated,
    ) {
        self.overlay = overlay;
        let src = Src {
            design,
            placements: self.overlay.as_deref().unwrap_or(evaluated.placements()),
        };
        for (id, e) in &mut self.bodies {
            self.core.sync_instances(Set::Model, *id, e, &src);
        }
        self.flush();
    }

    /// Sets or clears the per-body view overrides of committed bodies (additive, AM-11; see
    /// [`BodyViews`]). Only instances whose description changes are rewritten (no upload).
    pub fn set_body_views(
        &mut self,
        views: Option<Arc<BodyViews>>,
        design: &Design,
        evaluated: &Evaluated,
    ) {
        self.core.views = views;
        let src = Src {
            design,
            placements: self.overlay.as_deref().unwrap_or(evaluated.placements()),
        };
        for (id, e) in &mut self.bodies {
            self.core.sync_instances(Set::Model, *id, e, &src);
        }
        self.flush();
    }

    /// Sets or clears the per-body base colours of committed bodies (additive, INS-10; see
    /// [`BodyTints`]). Only instances whose description changes are rewritten (no upload);
    /// `None` restores the materials.
    pub fn set_body_tints(&mut self, tints: Option<Arc<BodyTints>>) {
        self.core.tints = tints;
        for (id, e) in &mut self.bodies {
            e.tint = self.core.tints.as_ref().and_then(|t| t.get(id).copied());
            self.core.send_instances(Set::Model, e);
        }
        self.flush();
    }

    /// The per-body base colours in force (INS-10).
    pub fn body_tints(&self) -> Option<&Arc<BodyTints>> {
        self.core.tints.as_ref()
    }

    /// The per-body view overrides in force (AM-11).
    pub fn body_views(&self) -> Option<&Arc<BodyViews>> {
        self.core.views.as_ref()
    }

    /// Replaces a highlight layer (targets that are not displayed are kept and resolved once
    /// they are).
    pub fn set_highlight(&mut self, layer: HighlightLayer, targets: &[PickTarget]) {
        if self.highlight_targets[layer.index()] == targets {
            return;
        }
        self.highlight_targets[layer.index()] = targets.to_vec();
        self.highlight_generation = u64::MAX;
        self.flush();
    }

    /// A highlight layer in render terms, as last sent.
    pub fn highlight(&self, layer: HighlightLayer) -> &[HighlightItem] {
        &self.highlight_items[layer.index()]
    }

    /// Highlight colours and the preview ghost colour from the theme (R-S10-8). Cheap when
    /// nothing changed.
    pub fn set_theme(&mut self, tokens: &ThemeTokens) {
        let colors = [
            rgba(tokens.hover, 1.0),
            rgba(tokens.selected, 1.0),
            rgba(tokens.error, 1.0),
            rgba(tokens.preview, 1.0),
        ];
        if colors == self.highlight_colors {
            return;
        }
        self.highlight_colors = colors;
        for layer in HighlightLayer::ALL {
            self.core.ops.push(SceneOp::HighlightColor {
                layer,
                rgba: colors[layer.index()],
            });
        }
        let pm = Material {
            base_color: rgba(tokens.preview, 0.4),
            ..Material::default()
        };
        if pm != self.core.preview_material {
            self.core.preview_material = pm;
            for e in self.preview.values_mut() {
                self.core.send_instances(Set::Preview, e);
            }
        }
        self.flush();
    }

    /// Which committed bodies the view's section planes clip and their cap colours (RND-06
    /// `Instance::sectioned` / `Material::cap_color`; additive, INS-09). Bodies the style does
    /// not mention are clipped with the default cap colour. Only instances whose description
    /// changes are rewritten (no upload).
    pub fn set_section_style(&mut self, style: &SectionStyle) {
        let mut changed = false;
        for (id, e) in &mut self.bodies {
            let want = style.of(*id);
            if e.section != want {
                e.section = want;
                self.core.send_instances(Set::Model, e);
                changed = true;
            }
        }
        if changed {
            self.flush();
        }
    }

    /// Whether `body` is clipped by the section planes, and its cap colour (INS-09).
    pub fn section_of(&self, body: BodyId) -> Option<(bool, Option<[f32; 3]>)> {
        self.bodies.get(&body).map(|e| e.section)
    }

    /// Display quality of new tessellations.
    pub fn set_quality(&mut self, q: DisplayQuality) {
        self.core.quality = q;
    }

    fn flush(&mut self) {
        if self.highlight_generation != self.core.pick.generation() {
            self.highlight_generation = self.core.pick.generation();
            for layer in HighlightLayer::ALL {
                let items = self.resolve(&self.highlight_targets[layer.index()]);
                if items != self.highlight_items[layer.index()] {
                    self.highlight_items[layer.index()] = items.clone();
                    self.core.ops.push(SceneOp::Highlight { layer, items });
                }
            }
        }
        self.core.flush();
    }

    fn resolve(&self, targets: &[PickTarget]) -> Vec<HighlightItem> {
        targets
            .iter()
            .filter_map(|t| {
                let (key, sub) = self.core.pick.render(t)?;
                Some(HighlightItem {
                    key,
                    sub: sub.map(|s| {
                        (
                            s.kind,
                            Range32 {
                                start: s.index,
                                end: s.index + 1,
                            },
                        )
                    }),
                })
            })
            .collect()
    }

    // ---- overlay owners -------------------------------------------------------------------

    /// A key for an overlay owner whose handle `i` picks `handles[i]` (manipulators, sketch
    /// glyphs drawn through RND-07). Released with [`Self::release_key`].
    pub fn register_handles(&mut self, handles: Vec<PickTarget>) -> RenderEntityKey {
        let k = self.core.key();
        self.core.pick.insert(k, Owner::Handles(handles));
        k
    }

    /// Releases an overlay owner's key (never reused).
    pub fn release_key(&mut self, key: RenderEntityKey) {
        if matches!(self.core.pick.owners.get(&key), Some(Owner::Handles(_))) {
            self.core.pick.remove(key);
        }
    }

    // ---- queries --------------------------------------------------------------------------

    /// The pick map.
    pub fn pick_map(&self) -> &PickMap {
        &self.core.pick
    }

    /// Every occurrence path, indexed by `PickTarget::Body::instance` (append-only for one
    /// adapter; additive, INS-10).
    pub fn occurrence_paths(&self) -> &[OccPath] {
        &self.core.paths
    }

    /// The occurrence path of a `PickTarget::Body::instance` index.
    pub fn occurrence_path(&self, instance: u32) -> Option<&OccPath> {
        self.core.paths.get(instance as usize)
    }

    /// Bodies known to the adapter (displayed or waiting for their mesh).
    pub fn bodies(&self) -> impl Iterator<Item = BodyId> + '_ {
        self.bodies.keys().copied()
    }

    /// Bodies on the preview layer.
    pub fn preview_bodies(&self) -> impl Iterator<Item = BodyId> + '_ {
        self.preview.keys().copied()
    }

    /// The current display mesh of a body.
    pub fn display_mesh(&self, body: BodyId) -> Option<&Arc<DisplayMesh>> {
        self.bodies.get(&body)?.mesh.as_ref().map(|m| &m.cpu)
    }

    /// The scene mesh slot of a body.
    pub fn mesh_slot(&self, body: BodyId) -> Option<MeshSlot> {
        self.bodies.get(&body)?.mesh.as_ref().map(|m| m.slot)
    }

    /// The instance slots of a body, in occurrence-path order.
    pub fn instance_slots(&self, body: BodyId) -> Vec<InstanceSlot> {
        self.bodies
            .get(&body)
            .map(|e| e.instances.iter().map(|i| i.slot).collect())
            .unwrap_or_default()
    }

    /// How often a body's geometry was uploaded since it appeared.
    pub fn uploads_of(&self, body: BodyId) -> u32 {
        self.bodies.get(&body).map_or(0, |e| e.uploads)
    }

    /// Tessellations in flight.
    pub fn pending(&self) -> usize {
        self.core.pending.len()
    }

    /// Counters.
    pub fn stats(&self) -> SceneStats {
        self.core.stats
    }

    /// The backend (downcast with `as_any` in tests).
    pub fn backend(&self) -> &dyn SceneBackend {
        self.core.backend.as_ref()
    }

    /// The last key allocated (keys are dense and increase).
    pub fn last_key(&self) -> RenderEntityKey {
        RenderEntityKey(self.core.next_key)
    }

    /// World bounds of the visible model instances (navigation pivot and fit, UI-03).
    /// [`amg_core::Aabb3::EMPTY`] for an empty scene. The first call per mesh scans its
    /// vertices; later calls only transform the cached box.
    pub fn bounds(&self) -> amg_core::Aabb3 {
        let mut out = amg_core::Aabb3::EMPTY;
        for e in self.bodies.values() {
            let Some(mesh) = &e.mesh else { continue };
            let local = mesh.local_bounds();
            for inst in e.instances.iter().filter(|i| i.visible && i.sent.is_some()) {
                out = out.union(&local.transformed(&inst.world));
            }
        }
        out
    }

    /// The whole-body targets of every displayed instance seen through occurrence `occ`
    /// (UI-04: the highlight of a selected component).
    pub fn occurrence_bodies(&self, occ: amg_doc::OccurrenceId) -> Vec<PickTarget> {
        let mut v: Vec<(BodyId, u32)> = self
            .core
            .pick
            .bodies
            .keys()
            .filter(|(_, instance)| {
                self.core
                    .paths
                    .get(*instance as usize)
                    .is_some_and(|p| p.contains(&occ))
            })
            .copied()
            .collect();
        v.sort_unstable();
        v.into_iter()
            .map(|(body, instance)| PickTarget::Body {
                body,
                instance,
                sub: None,
            })
            .collect()
    }

    /// The body → world transform of the displayed instance `instance` of `body` (`None` if it
    /// is not displayed; additive, INS-08: measurements on picked mesh faces).
    pub fn instance_world(&self, body: BodyId, instance: u32) -> Option<Transform> {
        let key = self.core.pick.body_key(body, instance)?;
        let e = self.bodies.get(&body)?;
        e.instances.iter().find(|i| i.key == key).map(|i| i.world)
    }

    /// World corners and mesh vertex indices of face `face` of the instance `key`, for display
    /// meshes whose faces are their triangles (mesh bodies). `None` for B-rep display meshes
    /// (faces span triangle ranges) and unknown keys. CPU edge and vertex picks (UI-04).
    pub(crate) fn mesh_triangle(
        &self,
        key: RenderEntityKey,
        face: u32,
    ) -> Option<([Point3; 3], [u32; 3])> {
        let PickTarget::Body { body, .. } = self.core.pick.target(key, None)? else {
            return None;
        };
        let e = self.bodies.get(&body)?;
        let mesh = e.mesh.as_ref()?;
        if !mesh.cpu.face_ranges.is_empty() {
            return None;
        }
        let inst = e.instances.iter().find(|i| i.key == key)?;
        let data = &mesh.cpu.data;
        let i = face as usize * 3;
        let idx = data.indices.get(i..i + 3)?;
        let mut corners = [Point3::ZERO; 3];
        for (c, &v) in corners.iter_mut().zip(idx) {
            let p = data.positions.get(v as usize)?;
            let local =
                data.origin + Point3::new(f64::from(p[0]), f64::from(p[1]), f64::from(p[2]));
            *c = inst.world.transform_point3(local);
        }
        Some((corners, [idx[0], idx[1], idx[2]]))
    }

    /// CPU rectangle selection over the display meshes: the keys of visible model instances
    /// with every vertex inside `rect` (`Window`) or at least one (`Crossing`), as seen through
    /// `project` (world → screen, `None` behind the camera). Occlusion is ignored; S10's GPU
    /// rectangle pick counts visible texels only.
    pub fn region(
        &self,
        project: impl Fn(Point3) -> Option<egui::Pos2>,
        rect: egui::Rect,
        mode: crate::pick::RegionMode,
    ) -> Vec<RenderEntityKey> {
        let mut out = Vec::new();
        for e in self.bodies.values() {
            let Some(mesh) = &e.mesh else { continue };
            let o = mesh.cpu.data.origin;
            for inst in e.instances.iter().filter(|i| i.visible && i.sent.is_some()) {
                let mut inside = 0usize;
                let mut total = 0usize;
                for p in &mesh.cpu.data.positions {
                    let local = o + Point3::new(f64::from(p[0]), f64::from(p[1]), f64::from(p[2]));
                    total += 1;
                    if project(inst.world.transform_point3(local)).is_some_and(|s| rect.contains(s))
                    {
                        inside += 1;
                    }
                }
                let hit = match mode {
                    crate::pick::RegionMode::Window => total > 0 && inside == total,
                    crate::pick::RegionMode::Crossing => inside > 0,
                };
                if hit {
                    out.push(inst.key);
                }
            }
        }
        out.sort();
        out
    }

    /// CPU ray cast against the displayed model instances (`world` ray). One hit per
    /// instance (its nearest), front to back; `Nearest` keeps the first. The first cast of a
    /// mesh builds its BVH.
    pub fn raycast(&self, ray: &Ray3, mode: PickMode) -> Vec<SceneHit> {
        let mut hits = Vec::new();
        for e in self.bodies.values() {
            let Some(mesh) = &e.mesh else { continue };
            for inst in e.instances.iter().filter(|i| i.visible && i.sent.is_some()) {
                let Some(local) = raycast::to_local(ray, &inst.world) else {
                    continue;
                };
                let bvh = mesh.bvh.get_or_init(|| Bvh::build(&mesh.cpu));
                let Some((tri, t_local)) = bvh.nearest(&mesh.cpu, &local) else {
                    continue;
                };
                let Some(face) = mesh.cpu.face_of_triangle(tri) else {
                    continue;
                };
                let world = inst.world.transform_point3(local.at(t_local));
                hits.push(SceneHit {
                    key: inst.key,
                    sub: RenderSub {
                        kind: RenderSubKind::Face,
                        index: face,
                    },
                    t: (world - ray.origin).dot(ray.dir.get()),
                    world,
                });
            }
        }
        hits.sort_by(|a, b| a.t.total_cmp(&b.t));
        if mode == PickMode::Nearest {
            hits.truncate(1);
        }
        hits
    }
}

/// Brings one entry of `map` in line with `body` (None: it no longer exists).
fn reconcile(
    core: &mut Core,
    map: &mut BTreeMap<BodyId, Entry>,
    set: Set,
    id: BodyId,
    body: Option<&Arc<EvalBody>>,
    src: &Src<'_>,
    jobs: &JobSystem,
) {
    let Some(b) = body else {
        if let Some(e) = map.remove(&id) {
            core.drop_entry(set, id, e);
        }
        return;
    };
    let props = src.design.body_props(id);
    let (visible, opacity) = match set {
        Set::Model => (props.visible, props.opacity),
        Set::Preview => (true, 1.0),
    };
    match map.get_mut(&id) {
        Some(e) => {
            e.component = b.component;
            e.visible = visible;
            e.opacity = opacity;
            if !e.geom.same_as(&b.geom) {
                e.geom = b.geom.clone();
                core.spawn(set, id, e, jobs);
            }
            core.sync_instances(set, id, e, src);
        }
        None => {
            let mut e = Entry {
                geom: b.geom.clone(),
                component: b.component,
                mesh: None,
                pending: None,
                instances: Vec::new(),
                visible,
                opacity,
                uploads: 0,
                section: (true, None),
                view_tint: None,
                view_shading: None,
                tint: None,
            };
            core.spawn(set, id, &mut e, jobs);
            core.sync_instances(set, id, &mut e, src);
            map.insert(id, e);
        }
    }
}
