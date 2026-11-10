//! Kernel errors. BREP-02 has the variants its operations can produce; BREP-12 completes the
//! enum and adds `amg_core::Diagnostic`.

use crate::shape::SubShape;

/// Result of a fallible kernel call.
pub type KResult<T> = Result<T, KernelError>;

/// What went wrong in the kernel.
#[derive(Clone, Debug, thiserror::Error)]
#[non_exhaustive]
pub enum KernelError {
    /// This build has no OCCT backend (cargo feature `occt` is off).
    #[error("geometry kernel is not available in this build")]
    BackendUnavailable,
    /// An argument is out of range or malformed.
    #[error("invalid input `{what}`: {reason}")]
    InvalidInput {
        /// The argument.
        what: &'static str,
        /// Why it was rejected.
        reason: String,
        /// The sub-shapes it names, if any.
        offending: Vec<SubShape>,
    },
    /// A rigid-only operation got a transform with scale, shear or reflection.
    #[error("transform is not rigid")]
    NonRigid,
    /// OCCT raised an exception.
    #[error("OCCT exception {kind}: {message}")]
    Occt {
        /// The exception type, for example `Standard_ConstructionError`.
        kind: String,
        /// The exception's message.
        message: String,
    },
    /// Reading or writing the stream failed.
    #[error("I/O: {0}")]
    Io(String),
    /// The data is not a shape this build can read.
    #[error("unsupported file content: {0}")]
    Format(String),
}

impl From<std::io::Error> for KernelError {
    fn from(error: std::io::Error) -> Self {
        KernelError::Io(error.to_string())
    }
}
