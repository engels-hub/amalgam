//! Triangle meshes: half-edge/indexed storage, repair, remesh, decimate, booleans and analysis.
//!
//! Spec: S07. Lane: C.
//!
//! - [`IndexedMesh`] is the interchange type (flat arrays, tolerates anything a file holds).
//! - [`HalfEdgeMesh`] is a compact directed-edges structure for topology edits.
//! - [`MeshBody`] is the shared body stored by `amg-doc`, with a binary encoding.
//!
//! Positions are f64 millimetres. Indices are `u32`; [`INVALID`] (`u32::MAX`) marks "none".

#![warn(missing_docs)]

mod attrs;
mod body;
mod error;
mod groups;
mod halfedge;
mod hash;
mod indexed;
mod spatial;
mod weld;

pub use attrs::{MeshAttrs, Rgba8, TriProperty};
pub use body::MeshBody;
pub use error::{MeshError, MeshResult, TopoError};
pub use groups::{
    DEFAULT_GROUP, FaceGroupId, FaceGroupInfo, FaceGroups, GroupFingerprint, GroupOrigin,
    Primitive, PrimitiveKind, ShellSide,
};
pub use halfedge::{ConversionReport, HalfEdgeMesh, NonManifoldPolicy};
pub use indexed::{IndexedMesh, NormalWeighting, Remap, RenderBuffers, WeldOpts};
pub use spatial::{
    Bvh, BvhNode, BvhNodeKind, BvhOpts, ClosestPoint, DEFAULT_WINDING_BETA, MeshSpatial, RayHit,
    TriFeature,
};

/// "No element" marker for `u32` indices.
pub const INVALID: u32 = u32::MAX;

/// Vertex index.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[repr(transparent)]
pub struct VertexId(pub u32);

/// Face index.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[repr(transparent)]
pub struct FaceId(pub u32);

/// Half-edge index (`3 × face + corner`).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[repr(transparent)]
pub struct HalfEdgeId(pub u32);
