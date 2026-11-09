//! `MaterialId` / `AppearanceId`: `"<library-id>/<local-id>"`.

use std::sync::Arc;

use crate::error::MaterialError;

/// True for a non-empty `[a-z0-9._-]+` token.
pub(crate) fn valid_token(s: &str) -> bool {
    !s.is_empty()
        && s.bytes().all(|b| {
            b.is_ascii_lowercase() || b.is_ascii_digit() || matches!(b, b'.' | b'_' | b'-')
        })
}

macro_rules! id_type {
    ($(#[$m:meta])* $name:ident) => {
        $(#[$m])*
        #[derive(
            Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize,
            serde::Deserialize,
        )]
        #[serde(try_from = "String", into = "String")]
        pub struct $name(Arc<str>);

        impl $name {
            /// Joins a library id and a local id (both `[a-z0-9._-]+`).
            pub fn new(library: &str, local: &str) -> Result<$name, MaterialError> {
                if !valid_token(library) || !valid_token(local) {
                    return Err(MaterialError::InvalidId(format!("{library}/{local}")));
                }
                Ok($name(format!("{library}/{local}").into()))
            }

            /// Parses `"<library>/<local>"`.
            pub fn parse(s: &str) -> Result<$name, MaterialError> {
                match s.split_once('/') {
                    Some((l, loc)) if valid_token(l) && valid_token(loc) => Ok($name(s.into())),
                    _ => Err(MaterialError::InvalidId(s.to_owned())),
                }
            }

            /// The library part.
            pub fn library(&self) -> &str {
                self.split().0
            }

            /// The local part.
            pub fn local(&self) -> &str {
                self.split().1
            }

            /// The whole id string.
            pub fn as_str(&self) -> &str {
                &self.0
            }

            fn split(&self) -> (&str, &str) {
                // Validated at construction: exactly one `/`.
                self.0.split_once('/').unwrap_or((&self.0, ""))
            }
        }

        impl std::fmt::Display for $name {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                f.write_str(&self.0)
            }
        }

        impl TryFrom<String> for $name {
            type Error = MaterialError;
            fn try_from(s: String) -> Result<$name, MaterialError> {
                $name::parse(&s)
            }
        }

        impl From<$name> for String {
            fn from(id: $name) -> String {
                id.0.as_ref().to_owned()
            }
        }
    };
}

id_type!(
    /// `"<library-id>/<local-id>"`, both `[a-z0-9._-]+`. Library ids: `amg.builtin`,
    /// `user.<name>`, `doc`.
    MaterialId
);
id_type!(
    /// Same scheme as [`MaterialId`], for appearances.
    AppearanceId
);

/// Name used by S01 `BodyProps.material`.
pub type MaterialKey = MaterialId;
/// Name used by S01 `BodyProps.appearance`.
pub type AppearanceKey = AppearanceId;
