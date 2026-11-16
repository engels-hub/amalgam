//! Mesh features (S07, lane C): mesh bodies in the timeline.
//!
//! A mesh body is `BodyGeom::Mesh(Arc<IndexedMesh>)` in `amg-doc` (the S01 "BodyKind::Mesh").
//! This module holds the pieces every mesh feature shares and the first two features:
//!
//! - [`MeshEntityRef`] and [`resolve_group`]: a face-group reference that resolves by group id
//!   first and by geometric fingerprint when the id is gone.
//! - [`OutputMode`]: replace the input body or create a new one.
//! - [`MeshFaceGroupsFeature`] (`mesh.face_groups`, MESH-12's segmentation).
//! - [`section_sketch`] (MESH-14): section polylines → sketch curves.
//! - [`shell`] (MESH-10): the robust SDF shell, a plain function over `amg-mesh` /
//!   `amg-implicit` types.
//! - [`MeshRepairFeature`] (`mesh.repair`, MESH-04's pipeline) and [`MeshReverseNormalFeature`]
//!   (`mesh.reverse_normal`).
//! - MESH-26 op features: [`MeshRemeshFeature`] (`mesh.remesh`), [`MeshReduceFeature`]
//!   (`mesh.reduce`), [`MeshSmoothFeature`] (`mesh.smooth`), [`MeshPlaneCutFeature`]
//!   (`mesh.plane_cut`), [`MeshShellFeature`] (`mesh.shell`, quick or robust),
//!   [`MeshSeparateFeature`] (`mesh.separate`) and [`MeshEraseFillFeature`]
//!   (`mesh.erase_fill`). Numeric parameters are `Expr`s (they become model parameters when
//!   the feature enters a design).
//!
//! The feature types are plain serde structs registered with `register_feature!`; call
//! [`link`] from a binary or test so the registrations are linked in.

mod base;
mod cut;
mod error;
mod offset;
mod reduce;
mod refs;
mod remesh;
mod repair;
mod reverse;
pub mod section_sketch;
mod segment;
pub mod shell;
mod smooth;
#[cfg(test)]
mod tests;

pub use base::OutputMode;
pub use cut::{
    CutPlane, FairingParam, KeepSideParam, MeshEraseFillFeature, MeshPlaneCutFeature,
    MeshSeparateFeature, OriginPlane,
};
pub use error::MeshFeatureError;
pub use offset::{DEFAULT_VOXEL_PER_THICKNESS, MIN_VOXEL_PER_DIAGONAL, MeshShellFeature};
pub use reduce::{MeshReduceFeature, ReduceParams, ReduceTarget};
pub use refs::{GroupMatch, MeshEntityRef, ResolvedGroup, resolve_group};
pub use remesh::{EdgeLengthParam, MeshRemeshFeature, RemeshParams, RemeshSizing};
pub use repair::{MeshRepairFeature, RepairParams, SmallShellParams};
pub use reverse::{MeshReverseNormalFeature, reverse_normals};
pub use segment::{MeshFaceGroupsFeature, SegmentParams};
pub use smooth::{MeshSmoothFeature, SmoothMethodParam, SmoothParams};

/// Forces the registrations of this module to be linked (registry `link()` convention).
pub fn link() {}
