//! Sections of solid bodies (INS-11, S05 §3): the B-rep tessellation (BREP-08) that
//! [`section_mesh`](super::section_mesh) cuts.
//!
//! Solid bodies have no cap code of their own: they are tessellated once, in their component
//! space, and the welded mesh goes through the same pipeline as a mesh body, placed by
//! `to_world`. The welded tessellation of a closed solid is closed and oriented, so
//! [`FillRule::Auto`](super::FillRule::Auto) uses the non-zero rule. Surface bodies give open
//! meshes, whose open section curves are reported and not capped.

use std::sync::{Arc, OnceLock};

use amg_brep::{BrepKernel, KernelConfig, KernelError, OcctKernel, Shape, TessQuality};
use amg_jobs::JobCtx;
use amg_mesh::{IndexedMesh, MeshBody};

use crate::inspect::InspectError;

/// The tessellation preset of solid sections: [`TessQuality::export`] (0.01 % of the bounding
/// box diagonal, at most 0.01 mm, 0.2 rad). Cap areas and perimeters are those of this
/// tessellation.
pub fn section_quality() -> TessQuality {
    TessQuality::export()
}

fn kernel() -> &'static OcctKernel {
    static KERNEL: OnceLock<OcctKernel> = OnceLock::new();
    KERNEL.get_or_init(|| OcctKernel::new(KernelConfig::default()))
}

/// The welded tessellation of `shape` at [`section_quality`], in the shape's own space.
///
/// Can take seconds on large parts: call it from a job, and keep the result while the shape is
/// unchanged (a plane change needs no new tessellation).
///
/// # Errors
/// [`InspectError::BackendUnavailable`] without the OCCT backend, [`InspectError::Brep`] for a
/// kernel failure, [`InspectError::Mesh`] if the kernel returns unusable triangles,
/// [`InspectError::Cancelled`].
pub fn tessellate_solid(shape: &Shape, ctx: &JobCtx) -> Result<MeshBody, InspectError> {
    ctx.checkpoint()?;
    let tess = kernel()
        .tessellate(shape, &section_quality(), ctx)
        .map_err(|e| match e {
            KernelError::Cancelled => InspectError::Cancelled,
            KernelError::BackendUnavailable => InspectError::BackendUnavailable,
            other => InspectError::Brep(other.to_string()),
        })?;
    ctx.checkpoint()?;
    let (positions, tris, _) = tess.to_indexed_parts();
    let mesh = IndexedMesh::new(positions, tris)?;
    Ok(MeshBody::from_arc(Arc::new(mesh)))
}
