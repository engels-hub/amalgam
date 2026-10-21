//! Mesh errors.

use amg_core::{Cancelled, Diagnostic, Severity};

/// Errors raised by `amg-mesh`.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum MeshError {
    /// The operation needs at least one face or vertex.
    #[error("mesh is empty")]
    Empty,
    /// A triangle references a vertex that does not exist.
    #[error("vertex index {index} out of range (mesh has {len} vertices)")]
    IndexOutOfRange {
        /// The offending index.
        index: u32,
        /// Number of vertices.
        len: usize,
    },
    /// More elements than the `u32` index range can address.
    #[error("mesh too large: {count} elements exceed the u32 index range")]
    TooLarge {
        /// Element count that overflowed.
        count: usize,
    },
    /// An attribute array has the wrong length.
    #[error("attribute `{name}` has {got} entries, expected {expected}")]
    AttributeLength {
        /// Attribute name.
        name: &'static str,
        /// Actual length.
        got: usize,
        /// Expected length.
        expected: usize,
    },
    /// The mesh has non-manifold edges or vertices and the operation needs a manifold.
    #[error("mesh is not manifold: {edges} non-manifold edges, {vertices} non-manifold vertices")]
    NonManifold {
        /// Edges that cannot be paired (more than two faces, or two faces of equal direction).
        edges: usize,
        /// Vertices whose faces form more than one fan.
        vertices: usize,
    },
    /// The mesh has boundary loops and the operation needs a closed mesh.
    #[error("mesh is not closed: {boundary_loops} boundary loops")]
    NotClosed {
        /// Number of boundary loops.
        boundary_loops: usize,
    },
    /// An option value is invalid.
    #[error("invalid option `{name}`: {reason}")]
    InvalidOption {
        /// Option name.
        name: &'static str,
        /// Why it is invalid.
        reason: String,
    },
    /// A half-edge topology edit was rejected.
    #[error("topology operation rejected: {0}")]
    Topology(#[from] TopoError),
    /// The boolean backend failed.
    #[error("boolean failed: {status}")]
    Boolean {
        /// Backend status text.
        status: String,
    },
    /// Built without the boolean backend.
    #[error("boolean backend unavailable (amg-mesh built without the `manifold` feature)")]
    BackendUnavailable,
    /// GPU failure.
    #[error("GPU error: {0}")]
    Gpu(String),
    /// Serialised mesh data is corrupt or of an unsupported version.
    #[error("corrupt mesh data: {0}")]
    Decode(String),
    /// The operation was cancelled.
    #[error("operation cancelled")]
    Cancelled,
}

/// `Result` with [`MeshError`].
pub type MeshResult<T> = Result<T, MeshError>;

impl From<Cancelled> for MeshError {
    fn from(_: Cancelled) -> Self {
        MeshError::Cancelled
    }
}

impl Diagnostic for MeshError {
    fn hint(&self) -> Option<String> {
        match self {
            MeshError::NonManifold { .. } | MeshError::NotClosed { .. } => {
                Some("Run Mesh Repair first.".to_owned())
            }
            MeshError::BackendUnavailable => {
                Some("Rebuild amg-mesh with the `manifold` feature.".to_owned())
            }
            MeshError::Decode(_) => {
                Some("The file may be truncated or from a newer version.".into())
            }
            _ => None,
        }
    }
    fn severity(&self) -> Severity {
        match self {
            MeshError::Cancelled => Severity::Info,
            _ => Severity::Error,
        }
    }
    fn is_cancelled(&self) -> bool {
        matches!(self, MeshError::Cancelled)
    }
}

/// Why a half-edge topology edit was rejected. The mesh is unchanged when one is returned.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[non_exhaustive]
pub enum TopoError {
    /// The operation needs an interior edge.
    #[error("half-edge is a boundary edge")]
    BoundaryEdge,
    /// The collapse would make the mesh non-manifold.
    #[error("collapse violates the link condition")]
    LinkCondition,
    /// The edge (or directed edge) already exists.
    #[error("flip would create a duplicate edge")]
    DuplicateEdge,
    /// A referenced element was deleted.
    #[error("element was deleted")]
    Dead,
    /// The edit would create a non-manifold vertex (a second fan) or a degenerate face.
    #[error("operation would create a non-manifold vertex or degenerate face")]
    NonManifold,
    /// The edit would exceed the `u32` index range.
    #[error("mesh would exceed the u32 index range")]
    TooLarge,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn diagnostics() {
        let e: MeshError = Cancelled.into();
        assert!(e.is_cancelled());
        assert_eq!(e.severity(), Severity::Info);
        let e = MeshError::NonManifold {
            edges: 1,
            vertices: 2,
        };
        assert_eq!(
            e.to_string(),
            "mesh is not manifold: 1 non-manifold edges, 2 non-manifold vertices"
        );
        assert!(e.hint().is_some());
        let e: MeshError = TopoError::LinkCondition.into();
        assert_eq!(
            e.to_string(),
            "topology operation rejected: collapse violates the link condition"
        );
        assert_eq!(e.severity(), Severity::Error);
    }
}
