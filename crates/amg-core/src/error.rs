//! Errors shared by all crates.
//!
//! - `Display` is the short user-facing message; `source()` and `Debug` carry developer
//!   context.
//! - Every library crate defines its own `thiserror` enum, implements [`Diagnostic`] and
//!   `From<Cancelled>`, so `ctx.checkpoint()?` works everywhere.

/// Shared cancellation marker; every crate's error implements `From<Cancelled>`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
#[error("operation cancelled")]
pub struct Cancelled;

/// Errors raised by `amg-core`.
///
/// The `Parse` and `DimensionMismatch` variants of the spec arrive with `units` (FND-10).
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum Error {
    /// Geometry is too small or degenerate for the operation (zero-length vector, collinear
    /// points, ...).
    #[error("degenerate geometry: {what}")]
    Degenerate {
        /// What was degenerate.
        what: &'static str,
    },
    /// An argument is outside its valid domain.
    #[error("invalid argument `{name}`: {reason}")]
    InvalidArgument {
        /// Parameter name.
        name: &'static str,
        /// Why it is invalid.
        reason: String,
    },
    /// A textual id could not be parsed.
    #[error("invalid id `{0}`")]
    InvalidId(String),
    /// The operation was cancelled.
    #[error(transparent)]
    Cancelled(#[from] Cancelled),
}

/// `Result` with [`Error`] as the default error type.
pub type Result<T, E = Error> = std::result::Result<T, E>;

/// Severity of a [`Diagnostic`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity {
    /// Informational; nothing went wrong (for example a user cancel).
    Info,
    /// The operation completed with a caveat.
    Warning,
    /// The operation failed.
    Error,
}

/// Uniform user-facing reporting for all library error enums (UI-10, CLI, Python).
pub trait Diagnostic: std::error::Error + Send + Sync + 'static {
    /// Short sentence for the user. Default: `to_string()`.
    fn user_message(&self) -> String {
        self.to_string()
    }
    /// Optional actionable hint ("Check that the sketch profile is closed").
    fn hint(&self) -> Option<String> {
        None
    }
    /// How serious this is. Default: [`Severity::Error`].
    fn severity(&self) -> Severity {
        Severity::Error
    }
    /// True if this error only means "the user cancelled". Default: `false`.
    fn is_cancelled(&self) -> bool {
        false
    }
}

impl Diagnostic for Cancelled {
    fn severity(&self) -> Severity {
        Severity::Info
    }
    fn is_cancelled(&self) -> bool {
        true
    }
}

impl Diagnostic for Error {
    fn hint(&self) -> Option<String> {
        match self {
            Error::Degenerate { .. } => {
                Some("Check for zero-length edges or coincident points.".to_owned())
            }
            Error::InvalidId(_) => {
                Some("Ids are UUIDs such as 67e55044-10b1-426f-9247-bb680e5fe0c8.".to_owned())
            }
            Error::InvalidArgument { .. } | Error::Cancelled(_) => None,
        }
    }
    fn severity(&self) -> Severity {
        match self {
            Error::Cancelled(_) => Severity::Info,
            _ => Severity::Error,
        }
    }
    fn is_cancelled(&self) -> bool {
        matches!(self, Error::Cancelled(_))
    }
}

/// Renders the message followed by the `source()` chain (debug context), one cause per line,
/// for logs and crash reports.
///
/// ```text
/// cannot load part
///   caused by: invalid id `x`
/// ```
pub fn report(e: &(dyn std::error::Error + 'static)) -> String {
    use std::fmt::Write as _;
    let mut out = e.to_string();
    let mut cur = e.source();
    while let Some(s) = cur {
        // Writing into a String cannot fail.
        let _ = write!(out, "\n  caused by: {s}");
        cur = s.source();
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug, thiserror::Error)]
    #[error("cannot load part")]
    struct Outer(#[source] Error);

    #[test]
    fn cancelled_converts_and_displays() {
        fn checkpoint() -> Result<()> {
            Err(Cancelled)?
        }
        let e = checkpoint().unwrap_err();
        assert_eq!(e.to_string(), "operation cancelled");
        assert!(e.is_cancelled());
        assert_eq!(e.severity(), Severity::Info);
        assert!(Cancelled.is_cancelled());
        assert_eq!(Cancelled.severity(), Severity::Info);
    }

    #[test]
    fn display_messages() {
        assert_eq!(
            Error::Degenerate { what: "x" }.to_string(),
            "degenerate geometry: x"
        );
        assert_eq!(
            Error::InvalidArgument {
                name: "n",
                reason: "negative".into()
            }
            .to_string(),
            "invalid argument `n`: negative"
        );
        assert_eq!(Error::InvalidId("q".into()).to_string(), "invalid id `q`");
    }

    #[test]
    fn diagnostic_defaults_and_hints() {
        let e = Error::Degenerate { what: "x" };
        assert_eq!(e.user_message(), e.to_string());
        assert!(e.hint().is_some());
        assert_eq!(e.severity(), Severity::Error);
        assert!(!e.is_cancelled());
        assert!(Error::InvalidId("q".into()).hint().is_some());
        assert!(
            Error::InvalidArgument {
                name: "n",
                reason: String::new()
            }
            .hint()
            .is_none()
        );
        assert!(Severity::Info < Severity::Warning && Severity::Warning < Severity::Error);
    }

    #[test]
    fn report_renders_source_chain() {
        let e = Outer(Error::InvalidId("x".into()));
        assert_eq!(report(&e), "cannot load part\n  caused by: invalid id `x`");
        assert_eq!(report(&Cancelled), "operation cancelled");
    }
}
