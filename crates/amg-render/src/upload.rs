//! GPU mesh upload (design note §5). Runs on job threads; no UI-thread work.

use amg_core::{Aabb3, Point3, Vec3};
use amg_jobs::JobCtx;
use amg_jobs::gpu::Gpu;

use crate::RenderError;

/// Plain mesh data for [`upload_mesh`]: f32 positions relative to `origin`.
#[derive(Clone, Debug, Default)]
pub struct MeshData {
    /// World position of the local frame, mm (RTE rebasing, design note §3).
    pub origin: Point3,
    /// Vertex positions relative to `origin`.
    pub positions: Vec<[f32; 3]>,
    /// Per-vertex unit normals; empty for flat shading from derivatives.
    pub normals: Vec<[f32; 3]>,
    /// Triangle list.
    pub indices: Vec<u32>,
    /// Watertight (section caps are allowed; RND-06).
    pub closed: bool,
}

/// A mesh on the GPU. Immutable and `Send + Sync`; dropping it frees VRAM once in-flight
/// frames finish.
#[derive(Debug)]
pub struct GpuMesh {
    pub(crate) positions: wgpu::Buffer,
    pub(crate) normals: Option<wgpu::Buffer>,
    pub(crate) indices: wgpu::Buffer,
    pub(crate) origin: Point3,
    index_count: u32,
    vertex_count: u32,
    bounds: Aabb3,
    closed: bool,
}

impl GpuMesh {
    /// Triangle count.
    pub fn triangle_count(&self) -> u32 {
        self.index_count / 3
    }

    /// Vertex count.
    pub fn vertex_count(&self) -> u32 {
        self.vertex_count
    }

    /// Index count, for draw calls.
    pub(crate) fn index_count(&self) -> u32 {
        self.index_count
    }

    /// Bounds in the mesh's frame with `origin` applied, f64 mm.
    pub fn bounds(&self) -> Aabb3 {
        self.bounds
    }

    /// Whether per-vertex normals were uploaded.
    pub fn has_normals(&self) -> bool {
        self.normals.is_some()
    }

    /// VRAM used by the buffers.
    pub fn gpu_bytes(&self) -> u64 {
        self.positions.size()
            + self.normals.as_ref().map_or(0, wgpu::Buffer::size)
            + self.indices.size()
    }

    /// True for a watertight mesh.
    pub fn is_closed(&self) -> bool {
        self.closed
    }
}

/// Uploads `data` to the GPU. Intended for a job thread; `wgpu::Device` is `Send + Sync`.
///
/// # Errors
/// [`RenderError::InvalidData`] if the normals or indices have a bad length,
/// [`RenderError::Cancelled`] when `ctx` is cancelled.
pub fn upload_mesh(gpu: &Gpu, data: &MeshData, ctx: &JobCtx) -> Result<GpuMesh, RenderError> {
    if !data.normals.is_empty() && data.normals.len() != data.positions.len() {
        return Err(RenderError::InvalidData(format!(
            "{} normals for {} vertices",
            data.normals.len(),
            data.positions.len()
        )));
    }
    if !data.indices.len().is_multiple_of(3) {
        return Err(RenderError::InvalidData(format!(
            "{} indices is not a whole number of triangles",
            data.indices.len()
        )));
    }
    ctx.checkpoint()?;

    let bounds = Aabb3::from_points(
        data.positions
            .iter()
            .map(|p| data.origin + Vec3::new(f64::from(p[0]), f64::from(p[1]), f64::from(p[2]))),
    );
    let vertex = wgpu::BufferUsages::VERTEX;
    let positions = gpu.upload("amg mesh positions", &data.positions, vertex);
    ctx.checkpoint()?;
    let normals =
        (!data.normals.is_empty()).then(|| gpu.upload("amg mesh normals", &data.normals, vertex));
    ctx.checkpoint()?;
    let indices = gpu.upload("amg mesh indices", &data.indices, wgpu::BufferUsages::INDEX);

    Ok(GpuMesh {
        positions,
        normals,
        indices,
        origin: data.origin,
        index_count: data.indices.len() as u32,
        vertex_count: data.positions.len() as u32,
        bounds,
        closed: data.closed,
    })
}
