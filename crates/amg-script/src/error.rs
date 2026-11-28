//! [`ScriptError`]: every failure of the scripting layer.

use amg_core::{Diagnostic, Severity};
use amg_doc::{CommandError, FormatError, RecomputeError, RegistryError, Revision};
use amg_features::inspect::InspectError;
use amg_features::io::ImportPrepError;
use amg_io::IoError;

/// Why a script operation failed.
///
/// The `From` conversions turn every kind of cancellation into [`ScriptError::Cancelled`], so
/// callers (CLI exit code 130, Python `KeyboardInterrupt`) test one variant.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum ScriptError {
    /// Opening or saving a `.amg` file failed.
    #[error(transparent)]
    Format(FormatError),
    /// A command was refused.
    #[error(transparent)]
    Command(#[from] CommandError),
    /// A recompute produced no result.
    #[error(transparent)]
    Recompute(RecomputeError),
    /// Reading or writing a model file failed.
    #[error(transparent)]
    Io(IoError),
    /// The design could not be turned into an export scene.
    #[error(transparent)]
    Export(ImportPrepError),
    /// A measurement or property computation failed.
    #[error(transparent)]
    Inspect(InspectError),
    /// The feature registry is broken (a build error).
    #[error(transparent)]
    Registry(#[from] RegistryError),
    /// A field value is not acceptable.
    #[error("field `{field}`: {msg}")]
    Field {
        /// The field.
        field: String,
        /// What is wrong with it.
        msg: String,
    },
    /// Something looked up by name or id does not exist (or is not available).
    #[error("no {what} named `{name}`")]
    NotFound {
        /// What was looked up.
        what: &'static str,
        /// The name or id.
        name: String,
    },
    /// An entity handle outlived the document revision it was selected at (SCR-09).
    #[error("entity handle is from revision {held:?}, the document is at {now:?}; select again")]
    StaleEntity {
        /// Revision of the handle.
        held: Revision,
        /// Current revision.
        now: Revision,
    },
    /// The mirrored UI document created different ids (SCR-03).
    #[error("the app document diverged from the script document at batch {seq}")]
    Diverged {
        /// The batch.
        seq: u64,
    },
    /// No usable Python installation (SCR-03).
    #[error("Python 3.12+ not found")]
    PythonNotFound {
        /// Places that were searched.
        searched: Vec<String>,
    },
    /// A Python exception (SCR-03).
    #[error("{message}")]
    Python {
        /// The exception message.
        message: String,
        /// The formatted traceback.
        traceback: String,
    },
    /// The operation was cancelled.
    #[error("cancelled")]
    Cancelled,
}

impl From<amg_core::Cancelled> for ScriptError {
    fn from(_: amg_core::Cancelled) -> Self {
        ScriptError::Cancelled
    }
}

impl From<FormatError> for ScriptError {
    fn from(e: FormatError) -> Self {
        match e {
            FormatError::Cancelled => ScriptError::Cancelled,
            e => ScriptError::Format(e),
        }
    }
}

impl From<RecomputeError> for ScriptError {
    fn from(e: RecomputeError) -> Self {
        match e {
            RecomputeError::Cancelled => ScriptError::Cancelled,
            e => ScriptError::Recompute(e),
        }
    }
}

impl From<IoError> for ScriptError {
    fn from(e: IoError) -> Self {
        if e.is_cancelled() {
            ScriptError::Cancelled
        } else {
            ScriptError::Io(e)
        }
    }
}

impl From<ImportPrepError> for ScriptError {
    fn from(e: ImportPrepError) -> Self {
        if e.is_cancelled() {
            ScriptError::Cancelled
        } else {
            ScriptError::Export(e)
        }
    }
}

impl From<InspectError> for ScriptError {
    fn from(e: InspectError) -> Self {
        if e.is_cancelled() {
            ScriptError::Cancelled
        } else {
            ScriptError::Inspect(e)
        }
    }
}

impl Diagnostic for ScriptError {
    fn user_message(&self) -> String {
        match self {
            ScriptError::Io(e) => e.user_message(),
            ScriptError::Export(e) => e.user_message(),
            ScriptError::Inspect(e) => e.user_message(),
            e => e.to_string(),
        }
    }

    fn hint(&self) -> Option<String> {
        match self {
            ScriptError::PythonNotFound { searched } => Some(format!(
                "Install Python 3.12 or newer, or set AMG_PYTHON_HOME. Searched: {}.",
                if searched.is_empty() {
                    "nothing".to_owned()
                } else {
                    searched.join(", ")
                }
            )),
            ScriptError::StaleEntity { .. } => {
                Some("Run the selector again after changing the document.".to_owned())
            }
            ScriptError::Format(e) => e.hint(),
            ScriptError::Command(e) => e.hint(),
            ScriptError::Io(e) => e.hint(),
            ScriptError::Export(e) => e.hint(),
            ScriptError::Inspect(e) => e.hint(),
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
        matches!(self, ScriptError::Cancelled)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_cancellation_becomes_cancelled() {
        let all: [ScriptError; 6] = [
            amg_core::Cancelled.into(),
            FormatError::Cancelled.into(),
            RecomputeError::Cancelled.into(),
            IoError::Cancelled.into(),
            ImportPrepError::Io(IoError::Cancelled).into(),
            InspectError::Cancelled.into(),
        ];
        for e in all {
            assert!(matches!(e, ScriptError::Cancelled), "{e:?}");
            assert!(e.is_cancelled());
            assert_eq!(e.severity(), Severity::Info);
        }
        let other = ScriptError::from(IoError::Empty);
        assert!(matches!(other, ScriptError::Io(IoError::Empty)));
        assert!(!other.is_cancelled());
        assert!(other.hint().is_some());
    }

    #[test]
    fn python_and_stale_errors_have_hints() {
        let py = ScriptError::PythonNotFound {
            searched: vec!["PATH".into()],
        };
        assert!(py.hint().is_some_and(|h| h.contains("PATH")));
        let stale = ScriptError::StaleEntity {
            held: Revision(1),
            now: Revision(2),
        };
        assert!(stale.hint().is_some());
    }
}
