//! Errors of this crate (S16 Interfaces).

use std::path::PathBuf;

/// Errors of the additive-manufacturing crate.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum AmError {
    /// A mesh operation failed.
    #[error(transparent)]
    Mesh(#[from] amg_mesh::MeshError),
    /// An export failed.
    #[error("export failed: {0}")]
    Io(String),
    /// A machine profile is invalid.
    #[error("invalid machine profile `{key}`: {reason}")]
    Machine {
        /// Library key (or file name) of the profile.
        key: String,
        /// What is wrong, naming the field.
        reason: String,
    },
    /// A profile file could not be read.
    #[error("cannot read profile {file}: {reason}")]
    Profile {
        /// The file.
        file: PathBuf,
        /// Why.
        reason: String,
    },
    /// Profile inheritance is broken (AM-10).
    #[error("profile inheritance of `{preset}` is broken: {reason}")]
    Inherits {
        /// The preset.
        preset: String,
        /// Why.
        reason: String,
    },
    /// An item transform is not rigid.
    #[error("item transform is not rigid")]
    NotRigid,
    /// The plate has no items.
    #[error("nothing to export: plate is empty")]
    EmptyPlate,
    /// The slicer executable does not exist.
    #[error("slicer executable not found: {0}")]
    SlicerNotFound(PathBuf),
    /// The slicer could not be started.
    #[error("could not start the slicer: {0}")]
    Launch(String),
    /// Cancelled by the user.
    #[error("operation cancelled")]
    Cancelled,
}

impl From<amg_core::Cancelled> for AmError {
    fn from(_: amg_core::Cancelled) -> Self {
        AmError::Cancelled
    }
}

impl amg_core::Diagnostic for AmError {
    fn hint(&self) -> Option<String> {
        match self {
            AmError::SlicerNotFound(_) | AmError::Launch(_) => {
                Some("Set the slicer path in Preferences → Manufacture".to_owned())
            }
            AmError::Machine { .. } | AmError::Profile { .. } => {
                Some("Fix the named field in the machine profile file".to_owned())
            }
            AmError::EmptyPlate => Some("Add a body to the plate first".to_owned()),
            _ => None,
        }
    }

    fn is_cancelled(&self) -> bool {
        matches!(self, AmError::Cancelled)
    }
}
