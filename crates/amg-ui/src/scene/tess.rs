//! Body tessellators: `BodyGeom` → display mesh, run inside a job (spec §7, note §9).

use std::ops::Range;
use std::sync::Arc;

use amg_core::Cancelled;
use amg_doc::BodyGeom;
use amg_jobs::JobCtx;
use amg_render::MeshData;

/// Display quality of a tessellation (UI-22 maps these to BREP-08 presets).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub enum DisplayQuality {
    /// Coarse, for previews.
    Preview,
    /// The default display quality.
    #[default]
    Normal,
    /// Fine (close-ups, renders).
    Fine,
}

/// Why a tessellation produced nothing.
#[derive(Debug, thiserror::Error)]
pub enum TessellateError {
    /// The geometry could not be tessellated.
    #[error("{0}")]
    Failed(String),
    /// The job was cancelled (a newer geometry superseded it).
    #[error(transparent)]
    Cancelled(#[from] Cancelled),
}

/// What a tessellator hands to the S10 upload: the mesh data plus the sub-id layout.
///
/// Sub-ids follow S10's scene-adapter contract: with `face_ranges` empty (meshes) the face
/// sub-id of a triangle is its index; otherwise face `f` owns triangles `face_ranges[f]`
/// (B-rep, `TessMesh::face_ranges` order, UI-22).
#[derive(Clone, Debug, Default)]
pub struct DisplayMesh {
    /// Upload data (positions rebased to `origin`, f32).
    pub data: MeshData,
    /// Per-face triangle ranges, sorted and disjoint; empty for meshes.
    pub face_ranges: Vec<Range<u32>>,
}

impl DisplayMesh {
    /// Number of triangles.
    pub fn triangle_count(&self) -> u32 {
        u32::try_from(self.data.indices.len() / 3).unwrap_or(u32::MAX)
    }

    /// Face sub-id of triangle `tri` (the triangle itself when there are no face ranges).
    pub fn face_of_triangle(&self, tri: u32) -> Option<u32> {
        if self.face_ranges.is_empty() {
            return (tri < self.triangle_count()).then_some(tri);
        }
        let i = self.face_ranges.partition_point(|r| r.end <= tri);
        let r = self.face_ranges.get(i)?;
        r.contains(&tri)
            .then(|| u32::try_from(i).unwrap_or(u32::MAX))
    }

    /// Number of face sub-ids.
    pub fn face_count(&self) -> u32 {
        if self.face_ranges.is_empty() {
            self.triangle_count()
        } else {
            u32::try_from(self.face_ranges.len()).unwrap_or(u32::MAX)
        }
    }
}

/// Converts one kind of `BodyGeom` for display (registered with
/// [`Registrar::tessellator`](crate::module::Registrar::tessellator)).
///
/// The most recently registered tessellator that handles a variant wins; the framework's
/// [`MeshTessellator`] is registered by module `app`, which [`UiRegistry::build`] runs first,
/// so a domain module can replace it.
///
/// [`UiRegistry::build`]: crate::registry::UiRegistry::build
pub trait BodyTessellator: Send + Sync + 'static {
    /// Which `BodyGeom` variants this handles.
    fn handles(&self, geom: &BodyGeom) -> bool;
    /// Runs inside a job; call `ctx.checkpoint()` in long loops. The result is uploaded on
    /// the same job.
    ///
    /// # Errors
    /// [`TessellateError::Failed`] when the geometry cannot be displayed;
    /// [`TessellateError::Cancelled`] after cancellation.
    fn tessellate(
        &self,
        geom: &BodyGeom,
        quality: DisplayQuality,
        ctx: &JobCtx,
    ) -> Result<DisplayMesh, TessellateError>;
}

/// Mesh bodies: a direct conversion of the `IndexedMesh` (UI-20). Flat shaded unless the mesh
/// carries vertex normals; rebased to the centre of its bounds (RTE, S10 note §3).
#[derive(Debug, Default)]
pub struct MeshTessellator;

impl BodyTessellator for MeshTessellator {
    fn handles(&self, geom: &BodyGeom) -> bool {
        matches!(geom, BodyGeom::Mesh(_))
    }

    fn tessellate(
        &self,
        geom: &BodyGeom,
        _quality: DisplayQuality,
        ctx: &JobCtx,
    ) -> Result<DisplayMesh, TessellateError> {
        let BodyGeom::Mesh(mesh) = geom else {
            return Err(TessellateError::Failed("not a mesh body".into()));
        };
        ctx.checkpoint()?;
        let bounds = mesh.aabb();
        let origin = if bounds.is_empty() {
            amg_core::Point3::ZERO
        } else {
            bounds.center()
        };
        let smooth = mesh.attrs.vertex_normals.is_some();
        let b = mesh.render_buffers(origin, smooth);
        ctx.checkpoint()?;
        Ok(DisplayMesh {
            data: MeshData {
                origin: b.origin,
                positions: b.positions,
                normals: b.normals,
                indices: b.indices,
                closed: false,
            },
            face_ranges: Vec::new(),
        })
    }
}

/// The tessellator for `geom`: the last registered one that handles it.
pub(crate) fn find(
    list: &[Arc<dyn BodyTessellator>],
    geom: &BodyGeom,
) -> Option<Arc<dyn BodyTessellator>> {
    list.iter().rev().find(|t| t.handles(geom)).cloned()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn face_of_triangle_with_and_without_ranges() {
        let mut m = DisplayMesh::default();
        m.data.indices = (0..30).collect();
        assert_eq!(m.triangle_count(), 10);
        assert_eq!(m.face_of_triangle(3), Some(3));
        assert_eq!(m.face_of_triangle(10), None);
        m.face_ranges = vec![0..4, 4..5, 5..10];
        assert_eq!(m.face_count(), 3);
        assert_eq!(m.face_of_triangle(0), Some(0));
        assert_eq!(m.face_of_triangle(3), Some(0));
        assert_eq!(m.face_of_triangle(4), Some(1));
        assert_eq!(m.face_of_triangle(9), Some(2));
        assert_eq!(m.face_of_triangle(10), None);
    }
}
