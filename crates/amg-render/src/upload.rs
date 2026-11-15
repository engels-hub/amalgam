//! GPU mesh upload (design note §5). Runs on job threads; no UI-thread work.

use std::sync::OnceLock;

use amg_core::{Aabb3, Point3, Vec3};
use amg_jobs::JobCtx;
use amg_jobs::gpu::Gpu;

use crate::RenderError;
use crate::shading::oct_snorm16;

/// Bytes per uploaded normal (`Snorm16x2` octahedral).
pub(crate) const NORMAL_BYTES: u64 = 4;

/// Elements checked or copied between two cancellation checkpoints (about 1 ms of work).
const CHUNK_ELEMENTS: usize = 1 << 18;

/// Plain mesh data for [`upload_mesh`]: f32 positions relative to `origin`.
#[derive(Clone, Debug, Default)]
pub struct MeshData {
    /// World position of the local frame, mm (RTE rebasing, design note §3).
    pub origin: Point3,
    /// Vertex positions relative to `origin`.
    pub positions: Vec<[f32; 3]>,
    /// Per-vertex normals (any non-zero length); empty for flat shading from derivatives.
    /// Uploaded octahedral-encoded as `Snorm16x2` (4 B per vertex, design note §5).
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
    /// Per-vertex B-rep face id (`Uint32`, RND-04 picking); `None` for plain meshes, whose
    /// pick sub-id is the triangle index.
    pub(crate) face_ids: Option<wgpu::Buffer>,
    /// The B-rep face ranges (face highlights redraw their triangles).
    pub(crate) face_ranges: Vec<FaceRange>,
    /// Vertex-pulling pick bind group (indices and positions as storage), made on first use.
    pub(crate) pull_bind: OnceLock<wgpu::BindGroup>,
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
            + self.face_ids.as_ref().map_or(0, wgpu::Buffer::size)
    }

    /// True for a watertight mesh.
    pub fn is_closed(&self) -> bool {
        self.closed
    }

    /// The B-rep face ranges given to [`upload_mesh_with_faces`]; empty for a plain mesh (whose
    /// pick sub-id is the triangle index).
    pub fn face_ranges(&self) -> &[FaceRange] {
        &self.face_ranges
    }

    /// Number of pick sub-ids of the faces: one past the largest face id of a B-rep mesh, the
    /// triangle count of a plain mesh.
    pub fn face_count(&self) -> u32 {
        if self.face_ids.is_some() {
            self.face_ranges
                .iter()
                .map(|r| r.face.saturating_add(1))
                .max()
                .unwrap_or(0)
        } else {
            self.triangle_count()
        }
    }
}

/// The triangles of one B-rep face: `first_tri..first_tri + tri_count` of the index list.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct FaceRange {
    /// First triangle.
    pub first_tri: u32,
    /// Number of triangles.
    pub tri_count: u32,
    /// The face's pick sub-id (the B-rep face index), below [`crate::CAP_SUB`].
    pub face: u32,
}

/// Face id of the vertices that no [`FaceRange`] covers (their triangles report it as sub-id).
pub const UNASSIGNED_FACE: u32 = crate::CAP_SUB - 1;

/// The per-vertex face ids of `faces` over the triangle list `indices` of `vertex_count`
/// vertices. Vertices no range covers get [`UNASSIGNED_FACE`].
///
/// # Errors
/// [`RenderError::InvalidData`] when a range lies outside the triangles, a face id is not below
/// [`UNASSIGNED_FACE`], or a vertex is used by triangles of two faces (B-rep tessellations share
/// no vertex between faces; the face id is a flat vertex attribute); [`RenderError::Cancelled`].
pub(crate) fn face_ids_of(
    vertex_count: usize,
    indices: &[u32],
    faces: &[FaceRange],
    ctx: &JobCtx,
) -> Result<Vec<u32>, RenderError> {
    let triangles = (indices.len() / 3) as u64;
    let mut ids = vec![UNASSIGNED_FACE; vertex_count];
    let mut since_checkpoint = 0usize;
    for (i, r) in faces.iter().enumerate() {
        let end = u64::from(r.first_tri) + u64::from(r.tri_count);
        if end > triangles {
            return Err(RenderError::InvalidData(format!(
                "face range {i} covers triangles {}..{end}, but there are {triangles}",
                r.first_tri
            )));
        }
        if r.face >= UNASSIGNED_FACE {
            return Err(RenderError::InvalidData(format!(
                "face range {i} has face id {}, the limit is {}",
                r.face,
                UNASSIGNED_FACE - 1
            )));
        }
        let first = r.first_tri as usize * 3;
        for &v in &indices[first..first + r.tri_count as usize * 3] {
            let slot = &mut ids[v as usize];
            if *slot != UNASSIGNED_FACE && *slot != r.face {
                return Err(RenderError::InvalidData(format!(
                    "vertex {v} is shared by faces {} and {}; B-rep faces must not share vertices",
                    *slot, r.face
                )));
            }
            *slot = r.face;
            since_checkpoint += 1;
            if since_checkpoint >= CHUNK_ELEMENTS {
                since_checkpoint = 0;
                ctx.checkpoint()?;
            }
        }
    }
    Ok(ids)
}

/// [`upload_mesh`] for a tessellated B-rep body: `faces` gives each face's triangles, and a
/// face pick reports the face id (a flat per-vertex attribute, 4 B per vertex) instead of the
/// triangle index. Face highlights redraw the face's triangles.
///
/// # Errors
/// As [`upload_mesh`], plus [`RenderError::InvalidData`] for bad face ranges (see
/// `face_ids_of`: out of range, a face id at or above [`UNASSIGNED_FACE`], a vertex shared by
/// two faces).
pub fn upload_mesh_with_faces(
    gpu: &Gpu,
    data: &MeshData,
    faces: &[FaceRange],
    ctx: &JobCtx,
) -> Result<GpuMesh, RenderError> {
    let mut mesh = upload_mesh(gpu, data, ctx)?;
    if faces.is_empty() {
        return Ok(mesh);
    }
    let ids = face_ids_of(data.positions.len(), &data.indices, faces, ctx)?;
    let limit = gpu.device().limits().max_buffer_size;
    check_size("face ids", ids.len() as u64 * 4, limit)?;
    mesh.face_ids = Some(upload_chunked(
        gpu,
        "amg mesh face ids",
        &ids,
        wgpu::BufferUsages::VERTEX,
        ctx,
    )?);
    mesh.face_ranges = faces.to_vec();
    Ok(mesh)
}

fn finite3(v: &[f32; 3]) -> bool {
    v[0].is_finite() && v[1].is_finite() && v[2].is_finite()
}

/// Checks `data` and returns its bounds (origin applied, f64). Checkpoints between chunks.
///
/// # Errors
/// [`RenderError::InvalidData`] for bad lengths, an index out of range, or a non-finite
/// position, normal or origin; [`RenderError::Cancelled`].
pub(crate) fn validate(data: &MeshData, ctx: &JobCtx) -> Result<Aabb3, RenderError> {
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
    if u32::try_from(data.positions.len()).is_err() || u32::try_from(data.indices.len()).is_err() {
        return Err(RenderError::InvalidData(
            "more than u32::MAX vertices or indices".into(),
        ));
    }
    if !data.origin.is_finite() {
        return Err(RenderError::InvalidData("the origin is not finite".into()));
    }
    ctx.checkpoint()?;

    let vertex_count = data.positions.len();
    let mut min = [f32::INFINITY; 3];
    let mut max = [f32::NEG_INFINITY; 3];
    for (chunk_index, chunk) in data.positions.chunks(CHUNK_ELEMENTS).enumerate() {
        for (i, p) in chunk.iter().enumerate() {
            if !finite3(p) {
                return Err(RenderError::InvalidData(format!(
                    "vertex {} has a non-finite position {p:?}",
                    chunk_index * CHUNK_ELEMENTS + i
                )));
            }
            for k in 0..3 {
                min[k] = min[k].min(p[k]);
                max[k] = max[k].max(p[k]);
            }
        }
        ctx.checkpoint()?;
    }
    for (chunk_index, chunk) in data.normals.chunks(CHUNK_ELEMENTS).enumerate() {
        if let Some(i) = chunk.iter().position(|n| !finite3(n)) {
            return Err(RenderError::InvalidData(format!(
                "vertex {} has a non-finite normal",
                chunk_index * CHUNK_ELEMENTS + i
            )));
        }
        ctx.checkpoint()?;
    }
    for (chunk_index, chunk) in data.indices.chunks(CHUNK_ELEMENTS).enumerate() {
        if let Some(i) = chunk.iter().position(|&i| i as usize >= vertex_count) {
            return Err(RenderError::InvalidData(format!(
                "index {} is {}, but the mesh has {vertex_count} vertices",
                chunk_index * CHUNK_ELEMENTS + i,
                chunk[i]
            )));
        }
        ctx.checkpoint()?;
    }

    Ok(if vertex_count == 0 {
        Aabb3::EMPTY
    } else {
        let lift = |v: [f32; 3]| {
            data.origin + Vec3::new(f64::from(v[0]), f64::from(v[1]), f64::from(v[2]))
        };
        Aabb3::new(lift(min), lift(max))
    })
}

/// `Err(TooLarge)` when `bytes` exceeds `limit`.
pub(crate) fn check_size(what: &'static str, bytes: u64, limit: u64) -> Result<(), RenderError> {
    if bytes > limit {
        Err(RenderError::TooLarge { what, bytes, limit })
    } else {
        Ok(())
    }
}

/// Creates a buffer holding `data`, copying through the mapped range in chunks so the job can
/// be cancelled between them.
pub(crate) fn upload_chunked<T: bytemuck::Pod>(
    gpu: &Gpu,
    label: &str,
    data: &[T],
    usage: wgpu::BufferUsages,
    ctx: &JobCtx,
) -> Result<wgpu::Buffer, RenderError> {
    let bytes: &[u8] = bytemuck::cast_slice(data);
    let buffer = gpu.device().create_buffer(&wgpu::BufferDescriptor {
        label: Some(label),
        size: bytes.len() as u64,
        usage,
        mapped_at_creation: !bytes.is_empty(),
    });
    if bytes.is_empty() {
        return Ok(buffer);
    }
    {
        let mut mapped = buffer
            .get_mapped_range_mut(..)
            .map_err(|e| RenderError::InvalidData(format!("cannot map {label}: {e}")))?;
        let step = CHUNK_ELEMENTS * size_of::<T>().max(1);
        for (i, src) in bytes.chunks(step).enumerate() {
            let start = i * step;
            mapped.slice(start..start + src.len()).copy_from_slice(src);
            ctx.checkpoint()?;
        }
    }
    buffer.unmap();
    Ok(buffer)
}

/// Creates a buffer of `data.len()` elements of `U`, filling it chunk by chunk through the
/// mapped range: `convert(source chunk, scratch)` appends one `U` per source element to the
/// (cleared) scratch vector, which is then copied in. A checkpoint follows every chunk.
fn upload_mapped<T, U: bytemuck::Pod>(
    gpu: &Gpu,
    label: &str,
    data: &[T],
    usage: wgpu::BufferUsages,
    ctx: &JobCtx,
    convert: impl Fn(&[T], &mut Vec<U>),
) -> Result<wgpu::Buffer, RenderError> {
    let elem = size_of::<U>();
    let bytes = (data.len() * elem) as u64;
    let buffer = gpu.device().create_buffer(&wgpu::BufferDescriptor {
        label: Some(label),
        size: bytes,
        usage,
        mapped_at_creation: bytes != 0,
    });
    if bytes == 0 {
        return Ok(buffer);
    }
    {
        let mut mapped = buffer
            .get_mapped_range_mut(..)
            .map_err(|e| RenderError::InvalidData(format!("cannot map {label}: {e}")))?;
        let mut scratch: Vec<U> = Vec::with_capacity(CHUNK_ELEMENTS.min(data.len()));
        for (i, src) in data.chunks(CHUNK_ELEMENTS).enumerate() {
            scratch.clear();
            convert(src, &mut scratch);
            debug_assert_eq!(scratch.len(), src.len());
            let start = i * CHUNK_ELEMENTS * elem;
            let out: &[u8] = bytemuck::cast_slice(&scratch);
            mapped.slice(start..start + out.len()).copy_from_slice(out);
            ctx.checkpoint()?;
        }
    }
    buffer.unmap();
    Ok(buffer)
}

/// Uploads `data` to the GPU. Intended for a job thread; `wgpu::Device` is `Send + Sync`.
/// Work is done in chunks of about 1 ms with a `ctx.checkpoint()` after each, so cancelling
/// returns quickly.
///
/// # Errors
/// - [`RenderError::InvalidData`]: bad lengths, an index out of range, or a non-finite position;
/// - [`RenderError::TooLarge`]: a buffer would exceed the device's `max_buffer_size`;
/// - [`RenderError::Cancelled`] when `ctx` is cancelled.
pub fn upload_mesh(gpu: &Gpu, data: &MeshData, ctx: &JobCtx) -> Result<GpuMesh, RenderError> {
    let bounds = validate(data, ctx)?;
    let limit = gpu.device().limits().max_buffer_size;
    check_size("positions", data.positions.len() as u64 * 12, limit)?;
    check_size("normals", data.normals.len() as u64 * NORMAL_BYTES, limit)?;
    check_size("indices", data.indices.len() as u64 * 4, limit)?;

    let vertex = wgpu::BufferUsages::VERTEX;
    // `STORAGE` lets the vertex-pulling pick backend read positions and indices (RND-04).
    let positions = upload_chunked(
        gpu,
        "amg mesh positions",
        &data.positions,
        vertex | wgpu::BufferUsages::STORAGE,
        ctx,
    )?;
    let normals = if data.normals.is_empty() {
        None
    } else {
        Some(upload_mapped(
            gpu,
            "amg mesh normals",
            &data.normals,
            vertex,
            ctx,
            |src: &[[f32; 3]], dst: &mut Vec<[i16; 2]>| {
                dst.extend(src.iter().map(|n| oct_snorm16(*n)));
            },
        )?)
    };
    let indices = upload_chunked(
        gpu,
        "amg mesh indices",
        &data.indices,
        wgpu::BufferUsages::INDEX | wgpu::BufferUsages::STORAGE,
        ctx,
    )?;

    Ok(GpuMesh {
        positions,
        normals,
        indices,
        origin: data.origin,
        index_count: data.indices.len() as u32,
        vertex_count: data.positions.len() as u32,
        bounds,
        closed: data.closed,
        face_ids: None,
        face_ranges: Vec::new(),
        pull_bind: OnceLock::new(),
    })
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, reason = "test code")]
mod tests {
    use amg_jobs::CancelToken;

    use super::*;

    fn triangle() -> MeshData {
        MeshData {
            positions: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 2.0, 0.0]],
            indices: vec![0, 1, 2],
            origin: Point3::new(10.0, 0.0, 0.0),
            ..Default::default()
        }
    }

    fn is_invalid(r: Result<Aabb3, RenderError>) -> bool {
        matches!(r, Err(RenderError::InvalidData(_)))
    }

    #[test]
    fn valid_data_gives_bounds_with_the_origin_applied() {
        let b = validate(&triangle(), &JobCtx::detached()).unwrap();
        assert_eq!(b.min, Point3::new(10.0, 0.0, 0.0));
        assert_eq!(b.max, Point3::new(11.0, 2.0, 0.0));
    }

    #[test]
    fn index_out_of_range_is_invalid_data() {
        let mut d = triangle();
        d.indices = vec![0, 1, 3];
        let err = validate(&d, &JobCtx::detached()).unwrap_err();
        assert!(matches!(&err, RenderError::InvalidData(m) if m.contains("index 2")));
    }

    #[test]
    fn non_finite_position_is_invalid_data() {
        for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            let mut d = triangle();
            d.positions[1][2] = bad;
            let err = validate(&d, &JobCtx::detached()).unwrap_err();
            assert!(
                matches!(&err, RenderError::InvalidData(m) if m.contains("vertex 1")),
                "{err}"
            );
        }
    }

    #[test]
    fn bad_normals_origin_and_lengths_are_invalid_data() {
        let ctx = JobCtx::detached();
        let mut d = triangle();
        d.normals = vec![[0.0, 0.0, 1.0]];
        assert!(is_invalid(validate(&d, &ctx)));
        d.normals = vec![[f32::NAN, 0.0, 1.0]; 3];
        assert!(is_invalid(validate(&d, &ctx)));
        let mut d = triangle();
        d.indices.pop();
        assert!(is_invalid(validate(&d, &ctx)));
        let mut d = triangle();
        d.origin = Point3::new(f64::NAN, 0.0, 0.0);
        assert!(is_invalid(validate(&d, &ctx)));
    }

    #[test]
    fn empty_mesh_is_valid_with_empty_bounds() {
        let b = validate(&MeshData::default(), &JobCtx::detached()).unwrap();
        assert!(b.is_empty());
    }

    #[test]
    fn size_over_the_limit_is_too_large() {
        assert!(check_size("indices", 100, 100).is_ok());
        let err = check_size("indices", 101, 100).unwrap_err();
        assert!(matches!(
            err,
            RenderError::TooLarge {
                what: "indices",
                bytes: 101,
                limit: 100
            }
        ));
        assert!(err.to_string().contains("exceeds the device limit 100"));
    }

    #[test]
    fn face_ids_follow_the_face_ranges() {
        let ctx = JobCtx::detached();
        // Two quads (two triangles each) sharing no vertex, plus one uncovered triangle.
        let indices = vec![0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7, 8, 9, 10];
        let faces = [
            FaceRange {
                first_tri: 0,
                tri_count: 2,
                face: 7,
            },
            FaceRange {
                first_tri: 2,
                tri_count: 2,
                face: 3,
            },
        ];
        let ids = face_ids_of(11, &indices, &faces, &ctx).unwrap();
        assert_eq!(&ids[..8], &[7, 7, 7, 7, 3, 3, 3, 3]);
        assert_eq!(&ids[8..], &[UNASSIGNED_FACE; 3]);

        let shared = [
            faces[0],
            FaceRange {
                first_tri: 4,
                tri_count: 1,
                face: 7,
            },
        ];
        assert!(
            face_ids_of(
                11,
                &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7, 2, 9, 10],
                &shared,
                &ctx
            )
            .is_ok()
        );
        let clash = [
            faces[0],
            FaceRange {
                first_tri: 4,
                tri_count: 1,
                face: 9,
            },
        ];
        let err = face_ids_of(
            11,
            &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7, 2, 9, 10],
            &clash,
            &ctx,
        )
        .unwrap_err();
        assert!(
            matches!(&err, RenderError::InvalidData(m) if m.contains("vertex 2")),
            "{err}"
        );
        let out_of_range = [FaceRange {
            first_tri: 4,
            tri_count: 2,
            face: 0,
        }];
        assert!(face_ids_of(11, &indices, &out_of_range, &ctx).is_err());
        let too_big = [FaceRange {
            first_tri: 0,
            tri_count: 1,
            face: UNASSIGNED_FACE,
        }];
        assert!(face_ids_of(11, &indices, &too_big, &ctx).is_err());
    }

    #[test]
    fn cancelled_context_stops_validation() {
        let token = CancelToken::new();
        token.cancel();
        let err = validate(&triangle(), &JobCtx::with_token(token)).unwrap_err();
        assert!(matches!(err, RenderError::Cancelled(_)));
    }
}
