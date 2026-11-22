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
//! - [`MeshCombineFeature`] (`mesh.combine`, MESH-27): join / cut / intersect a mesh target
//!   with mesh or solid tools (MESH-05 booleans); implicit operands are refused (they stay
//!   implicit, QUESTIONS #63).
//! - [`wrap`] (MESH-18): the wrap repair, a morphological closing through distance fields
//!   (plain function), and its feature [`MeshWrapFeature`] (`mesh.wrap`).
//!
//! The feature types are plain serde structs registered with `register_feature!`; call
//! [`link`] from a binary or test so the registrations are linked in.

mod base;
mod combine;
mod cut;
mod error;
mod offset;
mod proxy;
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
pub mod wrap;
mod wrap_feature;

pub use base::OutputMode;
pub use combine::{MAX_COMBINE_TOOLS, MeshCombineFeature};
pub use cut::{
    CutPlane, FairingParam, KeepSideParam, MeshEraseFillFeature, MeshPlaneCutFeature,
    MeshSeparateFeature, OriginPlane,
};
pub use error::MeshFeatureError;
pub use offset::{DEFAULT_VOXEL_PER_THICKNESS, MIN_VOXEL_PER_DIAGONAL, MeshShellFeature};
pub use proxy::{
    PREVIEW_PROXY_FACES, PREVIEW_PROXY_TARGET, preview_proxy_limits, set_preview_proxy_limits,
    uses_preview_proxy,
};
pub use reduce::{MeshReduceFeature, ReduceParams, ReduceTarget};
pub use refs::{GroupMatch, MeshEntityRef, ResolvedGroup, resolve_group};
pub use remesh::{EdgeLengthParam, MeshRemeshFeature, RemeshParams, RemeshSizing};
pub use repair::{MeshRepairFeature, RepairParams, SmallShellParams};
pub use reverse::{MeshReverseNormalFeature, reverse_normals};
pub use segment::{MeshFaceGroupsFeature, SegmentParams};
pub use smooth::{MeshSmoothFeature, SmoothMethodParam, SmoothParams};
pub use wrap_feature::{MeshWrapFeature, WRAP_FINE_VOXEL_PER_DIAGONAL};

/// Forces the registrations of this module to be linked (registry `link()` convention).
pub fn link() {}
