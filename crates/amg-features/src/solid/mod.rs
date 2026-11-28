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
//! - [`primitive`]: `solid.box`, `solid.cylinder`, `solid.sphere` and `solid.torus` (SOL-04).
//! - [`component`]: "New component", component patterns and mirrors (SOL-24).
//! - [`combine`]: `solid.combine` (SOL-07).
//! - [`split`]: `solid.split_body` and `solid.split_face` (SOL-07).
//! - [`moves`]: `solid.move` and `solid.align` (SOL-09).
//! - [`face_ops`]: `solid.offset_face`, `solid.replace_face`, `solid.delete_face` and the
//!   press-pull dispatch (SOL-09).
//! - [`fillet`]: `solid.fillet` and `solid.chamfer` (SOL-05), and the corpus-harness entry
//!   point [`fillet::harness::run_fillet`].
//! - [`shell`]: `solid.shell` and `solid.thicken` (SOL-06).
//! - [`draft`]: `solid.draft` (SOL-06).
//! - [`transform`]: `solid.scale` (SOL-06).
//!
//! Without the `occt` feature every kernel call fails with "geometry kernel is not available
//! in this build", so solid features compute to a clear `Kernel` error.

pub mod combine;
pub mod common;
pub mod component;
pub mod draft;
pub mod error;
pub mod extent;
pub mod extrude;
pub mod face_ops;
pub mod fillet;
pub mod moves;
pub mod pattern;
pub mod primitive;
pub mod profile;
pub mod revolve;
pub mod shell;
pub mod split;
pub mod transform;

pub use combine::Combine;
pub use common::{
    BodyOp, OpTarget, SolidTool, ToolBody, ToolRegistration, apply_body_op, input_solid, op_kernel,
    resolve_subs, simplify, tangent_blend_faces, tangent_chain_edges, tangent_chain_faces, tool_of,
};
pub use draft::{Draft, DraftSides};
pub use error::{SolidError, kernel_failure};
pub use extent::{Direction, Extent, ExtentSide, Start, ThinSide, ThinSpec};
pub use extrude::Extrude;
pub use face_ops::{DeleteFace, OffsetFace, ReplaceFace};
pub use fillet::{
    BodyEdges, Chamfer, ChamferKind, ChamferSet, CornerType, Fillet, FilletRadius, FilletSet,
    PlannedEdge, VarRadius,
};
pub use moves::{Align, Move, MoveSpec};
pub use pattern::{
    AngleSpan, ComputeType, Mirror, PathOrientation, Pattern, PatternDir, PatternKind,
    PatternObjects, Spacing,
};
pub use primitive::{BoxPrim, CylinderPrim, PrimitiveFrame, SpherePrim, TorusPrim};
pub use profile::{
    ProfileFace, ProfileSource, cap_source, face_edge_source, profile_faces, region_source, tag_of,
    tag_source, to_curve2d,
};
pub use revolve::Revolve;
pub use shell::{BodyFaces, PlannedFace, Shell, ShellDirection, Thicken};
pub use split::{SplitBody, SplitFace, SplitFaceTool};
pub use transform::{Scale, ScaleKind};

/// Forces this module's feature, tool and dialog registrations to be linked.
pub fn link() {}
