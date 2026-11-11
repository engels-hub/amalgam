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
//!
//! The feature types are plain serde structs registered with `register_feature!`; call
//! [`link`] from a binary or test so the registrations are linked in.

mod base;
mod error;
mod refs;
mod repair;
mod reverse;
pub mod section_sketch;
mod segment;
pub mod shell;
#[cfg(test)]
mod tests;

pub use base::OutputMode;
pub use error::MeshFeatureError;
pub use refs::{GroupMatch, MeshEntityRef, ResolvedGroup, resolve_group};
pub use repair::{MeshRepairFeature, RepairParams, SmallShellParams};
pub use reverse::{MeshReverseNormalFeature, reverse_normals};
pub use segment::{MeshFaceGroupsFeature, SegmentParams};

/// Forces the registrations of this module to be linked (registry `link()` convention).
pub fn link() {}
