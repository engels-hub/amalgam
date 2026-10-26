//! [`MeshFeatureError`].

use amg_doc::FeatureError;
use amg_mesh::FaceGroupId;

/// Why a mesh feature failed.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum MeshFeatureError {
    /// A mesh library call failed.
    #[error(transparent)]
    Mesh(#[from] amg_mesh::MeshError),
    /// Signed distance field failure (from `amg_implicit`).
    #[error("signed distance field: {0}")]
    Sdf(String),
    /// B-rep conversion failure (from `amg_brep`).
    #[error("B-rep conversion: {0}")]
    Brep(String),
    /// The input body is not a mesh.
    #[error("input body is not a mesh")]
    NotAMesh,
    /// A referenced face group has neither its id nor a matching fingerprint any more.
    #[error("referenced face group {0:?} no longer resolves")]
    UnresolvedGroup(FaceGroupId),
}

impl amg_core::Diagnostic for MeshFeatureError {
    fn is_cancelled(&self) -> bool {
        matches!(self, MeshFeatureError::Mesh(e) if amg_core::Diagnostic::is_cancelled(e))
    }
}

impl From<MeshFeatureError> for FeatureError {
    fn from(e: MeshFeatureError) -> FeatureError {
        match e {
            MeshFeatureError::Mesh(m) if amg_core::Diagnostic::is_cancelled(&m) => {
                FeatureError::Cancelled
            }
            MeshFeatureError::NotAMesh => FeatureError::InvalidValue {
                field: "body",
                msg: e.to_string(),
            },
            MeshFeatureError::UnresolvedGroup(_) => FeatureError::InvalidValue {
                field: "groups",
                msg: e.to_string(),
            },
            other => FeatureError::Kernel(other.to_string()),
        }
    }
}
