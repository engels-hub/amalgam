//! Inspection math (S05 §2–§6, lane A): measurement and section caps; later mass properties,
//! analysis fields and interference.
//!
//! Plain functions over `amg-core` / `amg-mesh` types, with no `amg-doc` types, so they are
//! testable without a document (S05 §1). Conventions and tolerances are in
//! `docs/notes/inspection-geometry.md`.
//!
//! - [`measure`] (INS-01): operands, per-entity measures, exact minimum distance, angles,
//!   smart selection (planar regions, feature-edge chains), plane and circle fits, snapping and
//!   result rows.
//! - [`section`] (INS-02): section caps on meshes (oriented loops, CDT with split constraints,
//!   non-zero / even–odd fill, multi-plane clipping), outlines, open curves and stats.

mod cache;
pub mod measure;
pub mod section;

use amg_core::{Diagnostic, Severity};

/// Errors of the inspection functions.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum InspectError {
    /// A mesh query failed.
    #[error(transparent)]
    Mesh(#[from] amg_mesh::MeshError),
    /// The operand combination has no meaning for this measurement.
    #[error("measuring {what} between {a} and {b} is not supported")]
    Unsupported {
        /// What was measured.
        what: &'static str,
        /// First operand kind.
        a: &'static str,
        /// Second operand kind.
        b: &'static str,
    },
    /// An element index is out of range or does not name an element of the body.
    #[error("element {0} does not exist in this body")]
    BadElement(String),
    /// The measurement needs a closed, consistently oriented body.
    #[error("{0} needs a closed, oriented body")]
    NotWatertight(&'static str),
    /// An operand's `to_world` scales, shears or mirrors.
    #[error("transform is not rigid; measurements need rigid occurrence transforms")]
    NonRigidTransform,
    /// Zero-area, zero-length or otherwise degenerate input.
    #[error("degenerate geometry: {0}")]
    Degenerate(&'static str),
    /// More section planes than supported (INS-02).
    #[error("too many section planes ({0} > 6)")]
    TooManyPlanes(usize),
    /// A B-rep query failed (INS-07/10).
    #[error("B-rep query failed: {0}")]
    Brep(String),
    /// A reference does not resolve (INS-07).
    #[error("reference does not resolve: {0}")]
    Resolve(String),
    /// The job was cancelled.
    #[error("operation cancelled")]
    Cancelled,
}

impl Diagnostic for InspectError {
    fn hint(&self) -> Option<String> {
        match self {
            InspectError::NotWatertight(_) => Some("Repair the mesh first (Mesh ▸ Repair)".into()),
            InspectError::NonRigidTransform => {
                Some("Measure the body without the scaled occurrence transform.".into())
            }
            InspectError::Mesh(e) => e.hint(),
            _ => None,
        }
    }

    fn severity(&self) -> Severity {
        if self.is_cancelled() {
            Severity::Info
        } else {
            Severity::Error
        }
    }

    fn is_cancelled(&self) -> bool {
        matches!(
            self,
            InspectError::Cancelled | InspectError::Mesh(amg_mesh::MeshError::Cancelled)
        )
    }
}

impl From<amg_core::Cancelled> for InspectError {
    fn from(_: amg_core::Cancelled) -> Self {
        InspectError::Cancelled
    }
}
