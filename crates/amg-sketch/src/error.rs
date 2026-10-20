//! Sketch errors.

use amg_core::{Cancelled, Diagnostic, Severity};

use crate::ids::{ConstraintId, EntityId, ParamId};

/// Errors raised by `amg-sketch`.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum SketchError {
    /// The entity does not exist (never existed or was removed).
    #[error("unknown entity {0:?}")]
    UnknownEntity(EntityId),
    /// The constraint does not exist (never existed or was removed).
    #[error("unknown constraint {0:?}")]
    UnknownConstraint(ConstraintId),
    /// The parameter index is outside the parameter store.
    #[error("unknown parameter {0:?}")]
    UnknownParam(ParamId),
    /// A constraint or entity references an entity of the wrong kind.
    #[error("constraint needs {expected}, but entity {entity:?} is a {found}")]
    WrongEntityKind {
        /// The offending entity.
        entity: EntityId,
        /// What was expected ("point", "line", "circle or arc", ...).
        expected: &'static str,
        /// What the entity is.
        found: &'static str,
    },
    /// The constraint exists but is not a dimension.
    #[error("constraint {0:?} is not a dimension")]
    NotADimension(ConstraintId),
    /// A value is outside its domain (negative length, non-finite coordinate, ...).
    #[error("invalid value {value} for {what}")]
    InvalidValue {
        /// What the value is.
        what: &'static str,
        /// The rejected value.
        value: f64,
    },
    /// The entity is locked (projected or fixed).
    #[error("entity {0:?} is locked (projected or fixed)")]
    Locked(EntityId),
    /// The geometry is degenerate for the operation.
    #[error("degenerate geometry: {0}")]
    Degenerate(&'static str),
    /// The operation does not apply to these inputs.
    #[error("operation not applicable: {0}")]
    NotApplicable(&'static str),
    /// The operation was cancelled.
    #[error("cancelled")]
    Cancelled,
}

impl From<Cancelled> for SketchError {
    fn from(_: Cancelled) -> Self {
        SketchError::Cancelled
    }
}

impl Diagnostic for SketchError {
    fn hint(&self) -> Option<String> {
        match self {
            SketchError::WrongEntityKind { expected, .. } => {
                Some(format!("Select a {expected} for this constraint."))
            }
            SketchError::Degenerate(_) => {
                Some("Check for zero-length lines or coincident points.".to_owned())
            }
            _ => None,
        }
    }
    fn severity(&self) -> Severity {
        match self {
            SketchError::Cancelled => Severity::Info,
            _ => Severity::Error,
        }
    }
    fn is_cancelled(&self) -> bool {
        matches!(self, SketchError::Cancelled)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cancelled_converts() {
        fn checkpoint() -> Result<(), SketchError> {
            Err(Cancelled)?
        }
        let e = checkpoint().unwrap_err();
        assert_eq!(e, SketchError::Cancelled);
        assert!(e.is_cancelled());
        assert_eq!(e.severity(), Severity::Info);
    }

    #[test]
    fn messages_and_hints() {
        let e = SketchError::WrongEntityKind {
            entity: EntityId(3),
            expected: "line",
            found: "point",
        };
        assert_eq!(
            e.to_string(),
            "constraint needs line, but entity EntityId(3) is a point"
        );
        assert!(e.hint().is_some());
        assert_eq!(e.severity(), Severity::Error);
        assert!(SketchError::UnknownParam(ParamId(1)).hint().is_none());
    }
}
