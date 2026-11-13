//! Where the scene adapter's output goes: S10's `RenderScene` ([`GpuSceneBackend`]) or a
//! recording mirror without a GPU ([`CpuSceneBackend`], tests and `NullViewport`).
//!
//! The adapter never names `amg-doc` types here: a backend sees display meshes, instances
//! and highlight lists only (no `amg-doc` types inside `amg-render`).

use std::any::Any;
use std::collections::{BTreeMap, HashMap};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use amg_core::Transform;
use amg_jobs::JobCtx;
use amg_jobs::gpu::Gpu;
use amg_render::{
    GpuMesh, Instance, InstanceKey, Layer, Material, MeshKey, RenderError, RenderScene,
};

use super::RenderEntityKey;
use super::tess::DisplayMesh;

/// A display mesh allocated by the adapter (never reused within a session).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct MeshSlot(pub u64);

/// An instance allocated by the adapter (never reused within a session).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct InstanceSlot(pub u64);

/// The result of an upload: a GPU mesh, or only its size for the CPU backend.
#[derive(Debug)]
pub struct UploadedMesh {
    gpu: Option<GpuMesh>,
    triangles: u32,
}

impl UploadedMesh {
    /// A mesh on the GPU.
    pub fn gpu(mesh: GpuMesh) -> UploadedMesh {
        UploadedMesh {
            triangles: mesh.triangle_count(),
            gpu: Some(mesh),
        }
    }

    /// No GPU: only the triangle count is kept.
    pub fn cpu(triangles: u32) -> UploadedMesh {
        UploadedMesh {
            gpu: None,
            triangles,
        }
    }

    /// Triangle count.
    pub fn triangles(&self) -> u32 {
        self.triangles
    }

    /// The GPU mesh, if this is one.
    pub fn into_gpu(self) -> Option<GpuMesh> {
        self.gpu
    }
}

/// Uploads display meshes. Runs on job threads (the tessellation job uploads its result).
pub trait MeshUploader: Send + Sync + 'static {
    /// Uploads `mesh`.
    ///
    /// # Errors
    /// The S10 upload errors (`InvalidData`, `TooLarge`, `Cancelled`).
    fn upload(&self, mesh: &DisplayMesh, ctx: &JobCtx) -> Result<UploadedMesh, RenderError>;
}

/// One drawn instance of a display mesh: one per (occurrence path, body).
#[derive(Clone, Debug, PartialEq)]
pub struct InstanceDesc {
    /// The pick id (S10 `PickId`, written to the id buffer by RND-04).
    pub key: RenderEntityKey,
    /// The mesh drawn.
    pub mesh: MeshSlot,
    /// World transform.
    pub transform: Transform,
    /// Drawn at all (body and occurrence visibility).
    pub visible: bool,
    /// Scene layer (`Model`, or `Preview` for preview bodies).
    pub layer: Layer,
    /// Colour and PBR parameters (opacity is `base_color[3]`).
    pub material: Material,
}

/// Highlight layers (S10 R-S10-5).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum HighlightLayer {
    /// Under the cursor.
    Hover,
    /// Selected.
    Selected,
    /// Failing features' bodies.
    Error,
    /// Preview emphasis.
    Preview,
}

impl HighlightLayer {
    /// Every layer.
    pub const ALL: [HighlightLayer; 4] = [
        HighlightLayer::Hover,
        HighlightLayer::Selected,
        HighlightLayer::Error,
        HighlightLayer::Preview,
    ];

    pub(crate) fn index(self) -> usize {
        self as usize
    }
}

/// Render-side sub-entity kinds (S10 `SubKind`; replaced by a re-export when RND-04 lands).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum RenderSubKind {
    /// A face: the triangle index of a mesh, the face index of a B-rep.
    Face,
    /// An edge polyline (`Polyline::id`).
    Edge,
    /// A vertex (`PointData::ids`).
    Vertex,
    /// An overlay handle.
    Handle,
}

/// Half-open sub-id range (S10 `Range32`).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct Range32 {
    /// First sub-id.
    pub start: u32,
    /// One past the last.
    pub end: u32,
}

/// One highlighted thing (S10 `HighlightItem`): a whole pick owner or a sub-id range.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct HighlightItem {
    /// The pick owner.
    pub key: RenderEntityKey,
    /// `None`: the whole owner.
    pub sub: Option<(RenderSubKind, Range32)>,
}

/// A change to the render scene, produced on the UI thread and applied in order.
#[derive(Debug)]
pub enum SceneOp {
    /// Adds a finished upload.
    InsertMesh {
        /// Its slot.
        slot: MeshSlot,
        /// The upload.
        mesh: UploadedMesh,
    },
    /// Removes a mesh (instances were repointed or removed before).
    RemoveMesh(MeshSlot),
    /// Adds the instance, or replaces its description (no geometry upload).
    SetInstance {
        /// Its slot.
        slot: InstanceSlot,
        /// What to draw.
        desc: InstanceDesc,
    },
    /// Removes an instance.
    RemoveInstance(InstanceSlot),
    /// Replaces a highlight layer.
    Highlight {
        /// The layer.
        layer: HighlightLayer,
        /// Its items.
        items: Vec<HighlightItem>,
    },
    /// Sets a highlight layer's colour (theme tokens, linear RGBA).
    HighlightColor {
        /// The layer.
        layer: HighlightLayer,
        /// Linear RGBA.
        rgba: [f32; 4],
    },
}

/// Applies scene ops; owned by the [`SceneAdapter`](super::SceneAdapter). UI thread only.
pub trait SceneBackend: 'static {
    /// The uploader handed to tessellation jobs.
    fn uploader(&self) -> Arc<dyn MeshUploader>;
    /// Applies `ops` in order. Cheap: no geometry work.
    fn apply(&mut self, ops: Vec<SceneOp>);
    /// For tests that inspect a concrete backend.
    fn as_any(&self) -> &dyn Any;
}

// ---- CPU backend --------------------------------------------------------------------------

#[derive(Debug, Default)]
struct CountingUploader {
    uploads: AtomicU64,
}

impl MeshUploader for CountingUploader {
    fn upload(&self, mesh: &DisplayMesh, ctx: &JobCtx) -> Result<UploadedMesh, RenderError> {
        ctx.checkpoint()?;
        self.uploads.fetch_add(1, Ordering::Relaxed);
        Ok(UploadedMesh::cpu(mesh.triangle_count()))
    }
}

/// A recording scene without a GPU: what `RenderScene` would hold (meshes by triangle count,
/// instances, highlights) plus an upload counter.
#[derive(Debug, Default)]
pub struct CpuSceneBackend {
    uploader: Arc<CountingUploader>,
    meshes: BTreeMap<MeshSlot, u32>,
    instances: BTreeMap<InstanceSlot, InstanceDesc>,
    highlights: [Vec<HighlightItem>; 4],
    colors: [[f32; 4]; 4],
    ops: u64,
}

impl CpuSceneBackend {
    /// An empty scene.
    pub fn new() -> CpuSceneBackend {
        CpuSceneBackend::default()
    }

    /// Uploads so far.
    pub fn uploads(&self) -> u64 {
        self.uploader.uploads.load(Ordering::Relaxed)
    }

    /// Live meshes.
    pub fn mesh_count(&self) -> usize {
        self.meshes.len()
    }

    /// Triangle count of a live mesh.
    pub fn mesh_triangles(&self, slot: MeshSlot) -> Option<u32> {
        self.meshes.get(&slot).copied()
    }

    /// Live instances, in slot order.
    pub fn instances(&self) -> impl Iterator<Item = (InstanceSlot, &InstanceDesc)> + '_ {
        self.instances.iter().map(|(k, v)| (*k, v))
    }

    /// Number of live instances.
    pub fn instance_count(&self) -> usize {
        self.instances.len()
    }

    /// A highlight layer as last set.
    pub fn highlight(&self, layer: HighlightLayer) -> &[HighlightItem] {
        &self.highlights[layer.index()]
    }

    /// A highlight layer's colour.
    pub fn highlight_color(&self, layer: HighlightLayer) -> [f32; 4] {
        self.colors[layer.index()]
    }

    /// Ops applied so far.
    pub fn ops_applied(&self) -> u64 {
        self.ops
    }
}

impl SceneBackend for CpuSceneBackend {
    fn uploader(&self) -> Arc<dyn MeshUploader> {
        Arc::clone(&self.uploader) as Arc<dyn MeshUploader>
    }

    fn apply(&mut self, ops: Vec<SceneOp>) {
        for op in ops {
            self.ops += 1;
            match op {
                SceneOp::InsertMesh { slot, mesh } => {
                    self.meshes.insert(slot, mesh.triangles());
                }
                SceneOp::RemoveMesh(slot) => {
                    self.meshes.remove(&slot);
                }
                SceneOp::SetInstance { slot, desc } => {
                    debug_assert!(
                        self.meshes.contains_key(&desc.mesh),
                        "instance points at a missing mesh"
                    );
                    self.instances.insert(slot, desc);
                }
                SceneOp::RemoveInstance(slot) => {
                    self.instances.remove(&slot);
                }
                SceneOp::Highlight { layer, items } => self.highlights[layer.index()] = items,
                SceneOp::HighlightColor { layer, rgba } => self.colors[layer.index()] = rgba,
            }
        }
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

// ---- GPU backend (S10 RenderScene) -------------------------------------------------------

struct GpuUploader {
    gpu: Arc<Gpu>,
    uploads: AtomicU64,
}

impl MeshUploader for GpuUploader {
    fn upload(&self, mesh: &DisplayMesh, ctx: &JobCtx) -> Result<UploadedMesh, RenderError> {
        let m = amg_render::upload_mesh(&self.gpu, &mesh.data, ctx)?;
        self.uploads.fetch_add(1, Ordering::Relaxed);
        Ok(UploadedMesh::gpu(m))
    }
}

/// Reaches the `RenderScene` for one batch of ops (egui's renderer lock, or a test mutex).
pub type SceneAccess = Box<dyn FnMut(&mut dyn FnMut(&mut RenderScene))>;

/// Drives S10's [`RenderScene`]: uploads with `upload_mesh` on the job, inserts with
/// `insert_mesh` / `add_instance` on the UI thread. A geometry swap points the instance at
/// the new mesh before the old one is removed (no frame without the body).
///
/// Highlight layers are kept here until RND-04 adds `RenderScene::set_highlight_layer`.
pub struct GpuSceneBackend {
    uploader: Arc<GpuUploader>,
    access: SceneAccess,
    meshes: HashMap<MeshSlot, MeshKey>,
    instances: HashMap<InstanceSlot, InstanceKey>,
    highlights: [Vec<HighlightItem>; 4],
    colors: [[f32; 4]; 4],
}

impl std::fmt::Debug for GpuSceneBackend {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GpuSceneBackend")
            .field("meshes", &self.meshes.len())
            .field("instances", &self.instances.len())
            .finish_non_exhaustive()
    }
}

impl GpuSceneBackend {
    /// A backend over any way to reach the scene.
    pub fn new(gpu: Arc<Gpu>, access: SceneAccess) -> GpuSceneBackend {
        GpuSceneBackend {
            uploader: Arc::new(GpuUploader {
                gpu,
                uploads: AtomicU64::new(0),
            }),
            access,
            meshes: HashMap::new(),
            instances: HashMap::new(),
            highlights: Default::default(),
            colors: [[0.0; 4]; 4],
        }
    }

    /// The scene of the renderer installed in egui (`egui_glue::install`). `None` if no
    /// renderer is installed.
    pub fn egui(rs: egui_wgpu::RenderState) -> Option<GpuSceneBackend> {
        let gpu = amg_render::egui_glue::with_renderer(&rs, |r| Arc::clone(r.gpu()))?;
        Some(GpuSceneBackend::new(
            gpu,
            Box::new(move |f| {
                amg_render::egui_glue::with_renderer(&rs, |r| f(r.scene_mut()));
            }),
        ))
    }

    /// A scene behind a mutex (headless tests, offscreen renders).
    pub fn shared(gpu: Arc<Gpu>, scene: Arc<Mutex<RenderScene>>) -> GpuSceneBackend {
        GpuSceneBackend::new(
            gpu,
            Box::new(move |f| match scene.lock() {
                Ok(mut s) => f(&mut s),
                Err(p) => f(&mut p.into_inner()),
            }),
        )
    }

    /// Uploads so far.
    pub fn uploads(&self) -> u64 {
        self.uploader.uploads.load(Ordering::Relaxed)
    }

    /// The `RenderScene` key of a slot's mesh.
    pub fn mesh_key(&self, slot: MeshSlot) -> Option<MeshKey> {
        self.meshes.get(&slot).copied()
    }

    /// The `RenderScene` key of a slot's instance.
    pub fn instance_key(&self, slot: InstanceSlot) -> Option<InstanceKey> {
        self.instances.get(&slot).copied()
    }

    /// A highlight layer as last set (drawn once RND-04 lands).
    pub fn highlight(&self, layer: HighlightLayer) -> &[HighlightItem] {
        &self.highlights[layer.index()]
    }

    /// A highlight layer's colour.
    pub fn highlight_color(&self, layer: HighlightLayer) -> [f32; 4] {
        self.colors[layer.index()]
    }
}

impl SceneBackend for GpuSceneBackend {
    fn uploader(&self) -> Arc<dyn MeshUploader> {
        Arc::clone(&self.uploader) as Arc<dyn MeshUploader>
    }

    fn apply(&mut self, ops: Vec<SceneOp>) {
        let mut scene_ops = Vec::with_capacity(ops.len());
        for op in ops {
            match op {
                SceneOp::Highlight { layer, items } => self.highlights[layer.index()] = items,
                SceneOp::HighlightColor { layer, rgba } => self.colors[layer.index()] = rgba,
                other => scene_ops.push(other),
            }
        }
        if scene_ops.is_empty() {
            return;
        }
        let meshes = &mut self.meshes;
        let instances = &mut self.instances;
        let mut pending = Some(scene_ops);
        (self.access)(&mut |scene: &mut RenderScene| {
            let Some(ops) = pending.take() else { return };
            for op in ops {
                apply_gpu(scene, meshes, instances, op);
            }
        });
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

fn apply_gpu(
    scene: &mut RenderScene,
    meshes: &mut HashMap<MeshSlot, MeshKey>,
    instances: &mut HashMap<InstanceSlot, InstanceKey>,
    op: SceneOp,
) {
    match op {
        SceneOp::InsertMesh { slot, mesh } => {
            if let Some(m) = mesh.into_gpu() {
                meshes.insert(slot, scene.insert_mesh(m));
            }
        }
        SceneOp::RemoveMesh(slot) => {
            if let Some(k) = meshes.remove(&slot) {
                scene.remove_mesh(k);
            }
        }
        SceneOp::SetInstance { slot, desc } => {
            let Some(&mesh) = meshes.get(&desc.mesh) else {
                tracing::warn!(
                    ?slot,
                    "scene: instance points at a mesh that is not uploaded"
                );
                return;
            };
            if let Some(inst) = instances.get(&slot).and_then(|k| scene.instance_mut(*k)) {
                inst.mesh = Some(mesh);
                inst.transform = desc.transform;
                inst.visible = desc.visible;
                inst.layer = desc.layer;
                inst.material = desc.material;
                return;
            }
            let key = scene.add_instance(Instance {
                mesh: Some(mesh),
                transform: desc.transform,
                visible: desc.visible,
                layer: desc.layer,
                material: desc.material,
            });
            instances.insert(slot, key);
        }
        SceneOp::RemoveInstance(slot) => {
            if let Some(k) = instances.remove(&slot) {
                scene.remove_instance(k);
            }
        }
        SceneOp::Highlight { .. } | SceneOp::HighlightColor { .. } => {}
    }
}
