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
        for path in src.placements.paths_of(e.component) {
            let Some(world) = src.placements.world(&path) else {
                continue;
            };
            let occ_visible = path
                .iter()
                .all(|o| src.design.occurrence(*o).is_none_or(|occ| occ.visible));
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
        let material = self.material(set, e);
        let layer = match set {
            Set::Model => Layer::Model,
            Set::Preview => Layer::Preview,
        };
        for inst in &mut e.instances {
            let desc = InstanceDesc {
                key: inst.key,
                mesh,
                transform: inst.world,
                visible: inst.visible,
                layer,
                material,
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
            };
            core.spawn(set, id, &mut e, jobs);
            core.sync_instances(set, id, &mut e, src);
            map.insert(id, e);
        }
    }
}
