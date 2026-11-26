//! Solid modeling features (S04, lane A): the shared framework (body operations, tools,
//! profiles, errors) and the features built on it.
//!
//! - [`common`]: [`BodyOp`], [`OpTarget`], [`ToolBody`], [`SolidTool`], [`ToolRegistration`]
//!   ([`register_tool!`](crate::register_tool)), [`apply_body_op`], [`simplify`], input helpers.
//! - [`profile`]: profile regions → planar faces ([`profile_faces`], [`to_curve2d`]).
//! - [`error`]: [`SolidError`] and [`kernel_failure`] (failing sub-shapes highlighted).
//! - [`extent`]: extents and directions.
//! - [`extrude`]: `solid.extrude` (SOL-01 framework, SOL-02 every start, direction and extent).
//! - [`revolve`]: `solid.revolve` (SOL-03).
//! - [`pattern`]: `solid.pattern` and `solid.mirror` (SOL-08).
//! - [`fillet`]: `solid.fillet` and `solid.chamfer` (SOL-05), and the corpus-harness entry
//!   point [`fillet::harness::run_fillet`].
//!
//! Without the `occt` feature every kernel call fails with "geometry kernel is not available
//! in this build", so solid features compute to a clear `Kernel` error.

pub mod common;
pub mod error;
pub mod extent;
pub mod extrude;
pub mod fillet;
pub mod pattern;
pub mod profile;
pub mod revolve;

pub use common::{
    BodyOp, OpTarget, SolidTool, ToolBody, ToolRegistration, apply_body_op, input_solid, op_kernel,
    resolve_subs, simplify, tangent_chain_edges, tool_of,
};
pub use error::{SolidError, kernel_failure};
pub use extent::{Direction, Extent, ExtentSide, Start, ThinSide, ThinSpec};
pub use extrude::Extrude;
pub use fillet::{
    BodyEdges, Chamfer, ChamferKind, ChamferSet, CornerType, Fillet, FilletRadius, FilletSet,
    PlannedEdge, VarRadius,
};
pub use pattern::{
    AngleSpan, ComputeType, Mirror, PathOrientation, Pattern, PatternDir, PatternKind,
    PatternObjects, Spacing,
};
pub use profile::{
    ProfileFace, ProfileSource, cap_source, face_edge_source, profile_faces, region_source, tag_of,
    tag_source, to_curve2d,
};
pub use revolve::Revolve;

/// Forces this module's feature, tool and dialog registrations to be linked.
pub fn link() {}
