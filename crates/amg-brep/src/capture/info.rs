//! Failure classes and the `error` block of a dump.

use crate::capture::op::SubShapeGeom;
use crate::error::KernelError;
use crate::shape::{Shape, SubShape};

/// What kind of failure a dump records. Replay compares classes: the same class is
/// `Reproduced`, another one `DifferentFailure`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[non_exhaustive]
pub enum FailureClass {
    /// OCCT raised an exception (`KernelError::Occt`).
    OcctException,
    /// The algorithm reported that it is not done (`OpFailed`, `LocalOpFailed` until KH-03
    /// classifies local-op failures further).
    NotDone,
    /// The result failed validation (`InvalidResult`).
    InvalidResult,
    /// The result's tolerance grew above the gate's bound (KH-17).
    ToleranceTooLarge,
    /// A fillet radius does not fit (KH-03).
    RadiusTooLarge,
    /// A fillet vertex blend failed (KH-03).
    VertexBlend,
    /// A fillet walking algorithm failed (KH-03).
    WalkingFailed,
    /// Loft section preprocessing failed (KH-06).
    SectionPreprocess,
    /// Loft surface construction failed (KH-08).
    SurfaceConstruction,
    /// Sewing failed (KH-08).
    SewingFailed,
    /// The result is not a solid where one was required (KH-17).
    NotSolid,
    /// The result intersects itself (KH-17).
    SelfInterference,
    /// A deadline expired (KH-17).
    Timeout,
    /// The job was cancelled. Never recorded; only replay can report it.
    Cancelled,
    /// The process ended inside the op (write-ahead dump recovered at the next start, or a
    /// replay child that died).
    Crash,
    /// The op panicked.
    Panic,
    /// The kernel rejected an argument (`InvalidInput`, `OpenLoop`, `NotPlanar`, `NonRigid`).
    /// Added by KH-01: argument errors are the most common failure of the ops that exist.
    InvalidInput,
    /// Anything else (`Io`, `Format`, `BackendUnavailable`).
    Other,
}

impl FailureClass {
    /// The class of a kernel error.
    pub fn of(e: &KernelError) -> FailureClass {
        match e {
            KernelError::Occt { .. } => FailureClass::OcctException,
            KernelError::OpFailed { .. } => FailureClass::NotDone,
            KernelError::LocalOpFailed { message, .. } => {
                FailureClass::tagged(message).unwrap_or(FailureClass::NotDone)
            }
            KernelError::InvalidResult { .. } => FailureClass::InvalidResult,
            KernelError::InvalidInput { .. }
            | KernelError::OpenLoop { .. }
            | KernelError::NotPlanar { .. }
            | KernelError::NonRigid => FailureClass::InvalidInput,
            KernelError::Cancelled => FailureClass::Cancelled,
            _ => FailureClass::Other,
        }
    }

    /// The class a `LocalOpFailed` message from KH-03's fillet chain starts with
    /// (`"RadiusTooLarge: …"`, [`crate::robust::fillet::FilletFailure::to_kernel_error`]); `None`
    /// for any other message (OCCT's own start with `contour`, `a corner`, `NotDone`, ...).
    pub fn tagged(message: &str) -> Option<FailureClass> {
        const TAGGED: [FailureClass; 10] = [
            FailureClass::InvalidInput,
            FailureClass::SelfInterference,
            FailureClass::RadiusTooLarge,
            FailureClass::VertexBlend,
            FailureClass::WalkingFailed,
            FailureClass::InvalidResult,
            FailureClass::NotSolid,
            FailureClass::ToleranceTooLarge,
            FailureClass::Timeout,
            FailureClass::NotDone,
        ];
        let (head, _) = message.split_once(": ")?;
        TAGGED.into_iter().find(|c| format!("{c:?}") == head)
    }
}

/// The `error` block of a dump: the class, the message and the sub-shapes the kernel blamed.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FailureInfo {
    /// The class.
    pub class: FailureClass,
    /// The error's message.
    pub message: String,
    /// The OCCT exception type for `OcctException`.
    pub occt_type: Option<String>,
    /// The offending sub-shapes, each with its fingerprint where it could be computed on the
    /// op's first input (multi-operand errors name sub-shapes of several operands; their
    /// message says which).
    pub faulty: Vec<(SubShape, Option<SubShapeGeom>)>,
}

impl FailureInfo {
    /// The failure info of `e`; `s` is the shape the offending sub-shapes refer to, if known.
    pub fn from_kernel_error(e: &KernelError, s: Option<&Shape>) -> FailureInfo {
        let message = match e {
            KernelError::InvalidResult { op, report } => {
                format!(
                    "{op} produced an invalid shape: {:?}",
                    report.problem_kinds()
                )
            }
            KernelError::LocalOpFailed { op, message, .. } => format!("{op} failed: {message}"),
            other => other.to_string(),
        };
        let occt_type = match e {
            KernelError::Occt { kind, .. } => Some(kind.clone()),
            _ => None,
        };
        let faulty = e
            .offending()
            .iter()
            .map(|sub| (*sub, s.and_then(|s| SubShapeGeom::of(s, *sub))))
            .collect();
        FailureInfo {
            class: FailureClass::of(e),
            message,
            occt_type,
            faulty,
        }
    }

    /// A failure without a kernel error (crash, panic).
    pub fn new(class: FailureClass, message: impl Into<String>) -> FailureInfo {
        FailureInfo {
            class,
            message: message.into(),
            occt_type: None,
            faulty: Vec::new(),
        }
    }
}

/// One attempt of a strategy chain, as KH-17's `robust` module logs it. `RecordingKernel`
/// records the single direct call, so its dumps have an empty attempts list.
///
/// KH-17 replaces this placeholder by `robust::Attempt` (strategy id, typed outcome) and keeps
/// the serialised field names.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Attempt {
    /// The strategy.
    pub strategy: String,
    /// `accepted`, `failed: <class> <message>` or `rejected: <reason>`.
    pub outcome: String,
    /// Wall time, ms.
    pub millis: u64,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::check::CheckReport;

    #[test]
    fn kernel_errors_map_to_classes() {
        let cases = [
            (
                KernelError::Occt {
                    kind: "Standard_ConstructionError".into(),
                    message: String::new(),
                },
                FailureClass::OcctException,
            ),
            (
                KernelError::OpFailed {
                    op: "boolean",
                    message: "BOPAlgo_AlertBOPNotAllowed".into(),
                    offending: Vec::new(),
                },
                FailureClass::NotDone,
            ),
            (
                KernelError::InvalidResult {
                    op: "revol",
                    report: CheckReport::valid(),
                },
                FailureClass::InvalidResult,
            ),
            (KernelError::NonRigid, FailureClass::InvalidInput),
            (KernelError::Cancelled, FailureClass::Cancelled),
            (KernelError::BackendUnavailable, FailureClass::Other),
        ];
        for (e, class) in cases {
            let info = FailureInfo::from_kernel_error(&e, None);
            assert_eq!(info.class, class, "{e:?}");
            assert!(!info.message.is_empty());
        }
        let occt = FailureInfo::from_kernel_error(
            &KernelError::Occt {
                kind: "StdFail_NotDone".into(),
                message: "x".into(),
            },
            None,
        );
        assert_eq!(occt.occt_type.as_deref(), Some("StdFail_NotDone"));
    }
}
