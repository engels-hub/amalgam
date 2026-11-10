//! Kernel errors (S02 §10 "Structured failures"). Variants carry the offending sub-shapes and
//! implement `amg_core::Diagnostic` with hints for the user.

use amg_core::{Cancelled, Diagnostic, Severity};

use crate::check::CheckReport;
use crate::shape::{Shape, SubShape};

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
    /// A profile loop does not close.
    #[error("profile is not closed (loop {loop_index}, edge {edge_index}, gap {gap:.3e} mm)")]
    OpenLoop {
        /// The loop: 0 = outer, `1 + i` = hole `i`.
        loop_index: u32,
        /// The edge after which the gap is.
        edge_index: u32,
        /// The gap, mm.
        gap: f64,
    },
    /// Points that must be coplanar are not.
    #[error("points are not coplanar (max deviation {max_dev:.3e} mm)")]
    NotPlanar {
        /// The largest distance from the fitted plane, mm.
        max_dev: f64,
    },
    /// A rigid-only operation got a transform with scale, shear or reflection.
    #[error("transform is not rigid")]
    NonRigid,
    /// An operation failed as a whole.
    #[error("{op} failed: {message}")]
    OpFailed {
        /// The operation.
        op: &'static str,
        /// OCCT's reason, for example the BOPAlgo alert names.
        message: String,
        /// The sub-shapes involved.
        offending: Vec<SubShape>,
    },
    /// A local operation (fillet, chamfer, shell, ...) failed on some edges or faces.
    #[error("{op} failed on {} sub-shape(s)", offending.len())]
    LocalOpFailed {
        /// The operation.
        op: &'static str,
        /// OCCT's reason.
        message: String,
        /// The failing edges, vertices or faces of the input.
        offending: Vec<SubShape>,
        /// A partial result, for display as a diagnostic only, never as the op result.
        partial: Option<Shape>,
    },
    /// The validation policy found the op's result invalid.
    #[error("{op} produced an invalid shape")]
    InvalidResult {
        /// The operation.
        op: &'static str,
        /// What the check found.
        report: CheckReport,
    },
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
    /// The job was cancelled.
    #[error("operation cancelled")]
    Cancelled,
}

impl KernelError {
    /// The sub-shapes this error names, in the input's index space; empty if none.
    pub fn offending(&self) -> &[SubShape] {
        match self {
            KernelError::InvalidInput { offending, .. }
            | KernelError::OpFailed { offending, .. }
            | KernelError::LocalOpFailed { offending, .. } => offending,
            _ => &[],
        }
    }
}

impl From<std::io::Error> for KernelError {
    fn from(error: std::io::Error) -> Self {
        KernelError::Io(error.to_string())
    }
}

impl From<Cancelled> for KernelError {
    fn from(_: Cancelled) -> Self {
        KernelError::Cancelled
    }
}

impl Diagnostic for KernelError {
    fn hint(&self) -> Option<String> {
        let hint = match self {
            KernelError::BackendUnavailable => {
                "This build has no geometry kernel; use a build with the `occt` feature."
            }
            KernelError::InvalidInput { .. } => "Check the highlighted input and its values.",
            KernelError::OpenLoop { .. } => {
                "Close the sketch profile: the highlighted edges do not meet."
            }
            KernelError::NotPlanar { .. } => "Make the points lie in one plane.",
            KernelError::NonRigid => {
                "Use a move or rotation here; scaling and mirroring need their own commands."
            }
            KernelError::OpFailed { .. } => {
                "Try a small fuzzy value, or move the bodies so faces do not just touch."
            }
            KernelError::LocalOpFailed { .. } => {
                "Reduce the radius or distance, or leave out the highlighted edges."
            }
            KernelError::InvalidResult { .. } => {
                "Change the inputs slightly; the result had geometry errors."
            }
            KernelError::Occt { .. } => {
                "The geometry kernel rejected the input; check for degenerate geometry."
            }
            KernelError::Io(_) => "Check that the file exists and is readable.",
            KernelError::Format(_) => "The file is damaged or from an unsupported version.",
            KernelError::Cancelled => return None,
        };
        Some(hint.to_owned())
    }

    fn severity(&self) -> Severity {
        match self {
            KernelError::Cancelled => Severity::Info,
            _ => Severity::Error,
        }
    }

    fn is_cancelled(&self) -> bool {
        matches!(self, KernelError::Cancelled)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shape::SubKind;

    #[test]
    fn cancelled_converts_and_is_info() {
        let error = KernelError::from(Cancelled);
        assert!(matches!(error, KernelError::Cancelled));
        assert!(error.is_cancelled());
        assert_eq!(error.severity(), Severity::Info);
        assert_eq!(error.hint(), None);
    }

    #[test]
    fn failures_have_hints_and_offending_shapes() {
        let edge = SubShape {
            kind: SubKind::Edge,
            index: 3,
        };
        let error = KernelError::LocalOpFailed {
            op: "fillet",
            message: "faulty contour".to_owned(),
            offending: vec![edge],
            partial: None,
        };
        assert_eq!(error.to_string(), "fillet failed on 1 sub-shape(s)");
        assert_eq!(error.offending(), &[edge]);
        assert!(!error.is_cancelled());
        assert_eq!(error.severity(), Severity::Error);
        assert!(error.hint().unwrap().contains("radius"));
        assert!(KernelError::NonRigid.offending().is_empty());
    }

    #[test]
    fn every_variant_except_cancel_has_a_hint() {
        let report = CheckReport::valid();
        let errors = [
            KernelError::BackendUnavailable,
            KernelError::InvalidInput {
                what: "x",
                reason: String::new(),
                offending: Vec::new(),
            },
            KernelError::OpenLoop {
                loop_index: 0,
                edge_index: 1,
                gap: 1e-3,
            },
            KernelError::NotPlanar { max_dev: 1.0 },
            KernelError::NonRigid,
            KernelError::OpFailed {
                op: "cut",
                message: String::new(),
                offending: Vec::new(),
            },
            KernelError::InvalidResult { op: "cut", report },
            KernelError::Occt {
                kind: "Standard_Failure".to_owned(),
                message: String::new(),
            },
            KernelError::Io(String::new()),
            KernelError::Format(String::new()),
        ];
        for error in errors {
            assert!(error.hint().is_some(), "{error:?}");
            assert!(!error.user_message().is_empty());
        }
    }
}
