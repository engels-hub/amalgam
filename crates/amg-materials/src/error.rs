//! [`MaterialError`] and its [`Diagnostic`] impl.

use amg_core::Diagnostic;
use amg_core::units::{Dimension, ParseQuantityError};

/// Everything that can go wrong loading, validating or saving materials and libraries.
///
/// `origin` is the file path (or `builtin`) and `path` the TOML key path, for example
/// `material[0].density`, so the UI and CLI can point at the offending line.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum MaterialError {
    /// The file is not valid TOML.
    #[error("{origin}: TOML syntax error: {message}")]
    Toml {
        /// File path or `builtin`.
        origin: String,
        /// The parser's message.
        message: String,
        /// Byte span into the source, if the parser gave one.
        span: Option<std::ops::Range<usize>>,
    },
    /// A quantity string could not be parsed.
    #[error("{origin}: `{path}`: {source}")]
    Quantity {
        /// File path or `builtin`.
        origin: String,
        /// TOML key path.
        path: String,
        /// The unit parser's error.
        #[source]
        source: ParseQuantityError,
    },
    /// A quantity has the wrong dimension (`density = "1.24 MPa"`).
    #[error("{origin}: `{path}`: expected {expected}, got {found}")]
    Dimension {
        /// File path or `builtin`.
        origin: String,
        /// TOML key path.
        path: String,
        /// The dimension the property needs.
        expected: Dimension,
        /// The dimension of the written unit.
        found: Dimension,
    },
    /// A dimensioned property was written as a bare number.
    #[error("{origin}: `{path}` needs a unit")]
    MissingUnit {
        /// File path or `builtin`.
        origin: String,
        /// TOML key path.
        path: String,
    },
    /// A value is out of range, inconsistent, or the file structure is wrong.
    #[error("material `{id}`: invalid `{property}`: {reason}")]
    Invalid {
        /// Material id (or the file for structural problems).
        id: String,
        /// Property or TOML key path.
        property: String,
        /// What is wrong.
        reason: String,
    },
    /// A required property is absent.
    #[error("material `{id}` has no `{property}`")]
    MissingProperty {
        /// Material id.
        id: String,
        /// Property name.
        property: &'static str,
    },
    /// A malformed material, appearance or library id.
    #[error("invalid id `{0}`")]
    InvalidId(String),
    /// Two records in one file share an id.
    #[error("{origin}: duplicate id `{id}`")]
    Duplicate {
        /// File path or `builtin`.
        origin: String,
        /// The repeated id.
        id: String,
    },
    /// A `src` names no `[[source]]` record.
    #[error("{origin}: unknown source `{key}`")]
    UnknownSource {
        /// File path or `builtin`.
        origin: String,
        /// The unresolved key.
        key: String,
    },
    /// The file was written by a newer format version.
    #[error("{origin}: format version {found} is newer than supported {supported}")]
    TooNew {
        /// File path or `builtin`.
        origin: String,
        /// Version in the file.
        found: u32,
        /// Highest version this build reads.
        supported: u32,
    },
    /// Saving or editing the built-in library.
    #[error("library `{0}` is read-only")]
    ReadOnly(String),
    /// Saving a library that was never loaded from or assigned a file.
    #[error("library `{0}` has no file path")]
    NoPath(String),
    /// File system error.
    #[error(transparent)]
    Io(#[from] std::io::Error),
}

impl Diagnostic for MaterialError {
    fn hint(&self) -> Option<String> {
        Some(
            match self {
                MaterialError::Quantity { .. }
                | MaterialError::MissingUnit { .. }
                | MaterialError::Dimension { .. } => {
                    "Write a value with a unit in a string, for example `1.24 g/cm^3`."
                }
                MaterialError::Invalid { reason, .. } if reason.contains("specimen") => {
                    "Z values need a Z** specimen (ISO/ASTM 52921), e.g. `ZXY`."
                }
                MaterialError::Invalid { .. } => "Fix the value; the reason says what is allowed.",
                MaterialError::UnknownSource { .. } => {
                    "Add a `[[source]]` record with that key, or fix the `src` key."
                }
                MaterialError::TooNew { .. } => "Update Amalgam to open this library.",
                MaterialError::ReadOnly(_) => {
                    "Duplicate the material into a user library to edit it."
                }
                MaterialError::Toml { .. } => "Fix the TOML syntax at the reported position.",
                _ => return None,
            }
            .to_owned(),
        )
    }
}
