//! Feature registry (S01 §2): `inventory` + explicit registration records.
//!
//! Each feature type submits one [`FeatureRegistration`] with
//! [`register_feature!`](crate::register_feature). The registry is built once, on first use,
//! from everything the linker kept, and a broken registry (duplicate or malformed kind) is a
//! startup error: [`registry`] panics, [`try_registry`] reports it.
//!
//! **Linker caveat and the `link()` convention.** `inventory` only sees code that is linked,
//! and rustc links a dependency only if something references it. Every crate that registers
//! features therefore exports
//!
//! ```ignore
//! /// Forces this crate's feature registrations to be linked.
//! pub fn link() {}
//! ```
//!
//! and every binary (`amg-app`, `amg-cli`, `amg-script`) calls `amg_features::link()` at
//! startup (which calls the `link()` of every crate it re-exports features from). A test in each
//! registering crate asserts its expected kinds are present (see `tests/registry_link.rs`).

use std::collections::BTreeMap;
use std::sync::{Arc, OnceLock};

use crate::feature::{Feature, MigrationError};

/// Registration record of one feature kind.
#[derive(Debug)]
pub struct FeatureRegistration {
    /// Kind string, stable forever (lowercase `[a-z0-9_.-]`, e.g. `"solid.extrude"`).
    pub kind: &'static str,
    /// Current schema version of the kind's data (≥ 1).
    pub version: u32,
    /// Deserializes current-version data.
    pub deserialize: fn(serde_json::Value) -> Result<Arc<dyn Feature>, serde_json::Error>,
    /// Migrates data from `from_version` (< `version`) to `version`.
    pub migrate:
        fn(from_version: u32, raw: serde_json::Value) -> Result<serde_json::Value, MigrationError>,
}

inventory::collect!(FeatureRegistration);

/// Default `migrate` for kinds that have never changed their schema.
#[doc(hidden)]
pub fn no_migration(
    from: u32,
    _raw: serde_json::Value,
) -> Result<serde_json::Value, MigrationError> {
    Err(MigrationError(format!("no migration from v{from}")))
}

/// Registers a feature type.
///
/// ```ignore
/// #[derive(Clone, Debug, Serialize, Deserialize)]
/// pub struct Extrude { /* ... */ }
/// impl amg_doc::Feature for Extrude { /* inputs, compute, (validate) */ }
/// amg_doc::register_feature!(Extrude, "solid.extrude", version = 1);
/// amg_doc::register_feature!(Shell, "solid.shell", version = 3, migrate = shell_migrate);
/// ```
///
/// Implements [`FeatureMeta`](crate::FeatureMeta) for the type (it must be
/// `Clone + Serialize + DeserializeOwned` and implement [`Feature`]) and submits its
/// [`FeatureRegistration`]. `migrate` is `fn(u32, serde_json::Value) -> Result<serde_json::Value,
/// MigrationError>` and is called for data older than `version`; without it, older data is an
/// error. Use the macro once per type, in the crate that defines it, and follow the `link()`
/// convention (module docs of [`registry`](crate::registry)).
#[macro_export]
macro_rules! register_feature {
    ($ty:ty, $kind:literal, version = $version:literal $(,)?) => {
        $crate::register_feature!(
            $ty,
            $kind,
            version = $version,
            migrate = $crate::registry::no_migration
        );
    };
    ($ty:ty, $kind:literal, version = $version:literal, migrate = $migrate:path $(,)?) => {
        impl $crate::FeatureMeta for $ty {
            fn kind(&self) -> &str {
                $kind
            }
            fn version(&self) -> u32 {
                $version
            }
            fn to_json(
                &self,
            ) -> ::core::result::Result<
                $crate::__private::serde_json::Value,
                $crate::__private::serde_json::Error,
            > {
                $crate::__private::serde_json::to_value(self)
            }
            fn as_any(&self) -> &dyn ::core::any::Any {
                self
            }
            fn clone_arc(&self) -> ::std::sync::Arc<dyn $crate::Feature> {
                ::std::sync::Arc::new(::core::clone::Clone::clone(self))
            }
        }
        const _: () = {
            fn deserialize(
                v: $crate::__private::serde_json::Value,
            ) -> ::core::result::Result<
                ::std::sync::Arc<dyn $crate::Feature>,
                $crate::__private::serde_json::Error,
            > {
                let f: $ty = $crate::__private::serde_json::from_value(v)?;
                ::core::result::Result::Ok(::std::sync::Arc::new(f))
            }
            $crate::__private::inventory::submit! {
                $crate::FeatureRegistration {
                    kind: $kind,
                    version: $version,
                    deserialize,
                    migrate: $migrate,
                }
            }
        };
    };
}

/// Why the registry could not be built.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
#[non_exhaustive]
pub enum RegistryError {
    /// Two registrations share a kind.
    #[error("feature kind `{0}` is registered twice")]
    DuplicateKind(&'static str),
    /// A kind string is not of the form `[a-z0-9_-]+(\.[a-z0-9_-]+)*`.
    #[error(
        "feature kind `{0}` is malformed (expected lowercase `[a-z0-9_-]` segments joined by `.`)"
    )]
    InvalidKind(&'static str),
    /// Versions start at 1.
    #[error("feature kind `{0}` has version 0")]
    ZeroVersion(&'static str),
}

impl amg_core::Diagnostic for RegistryError {}

/// All feature kinds known to this build.
#[derive(Debug, Default)]
pub struct FeatureRegistry {
    by_kind: BTreeMap<&'static str, &'static FeatureRegistration>,
}

impl FeatureRegistry {
    /// Builds a registry from explicit records (the global one uses every `inventory`
    /// submission).
    pub fn from_registrations(
        regs: impl IntoIterator<Item = &'static FeatureRegistration>,
    ) -> Result<FeatureRegistry, RegistryError> {
        let mut by_kind = BTreeMap::new();
        for r in regs {
            if !valid_kind(r.kind) {
                return Err(RegistryError::InvalidKind(r.kind));
            }
            if r.version == 0 {
                return Err(RegistryError::ZeroVersion(r.kind));
            }
            if by_kind.insert(r.kind, r).is_some() {
                return Err(RegistryError::DuplicateKind(r.kind));
            }
        }
        Ok(FeatureRegistry { by_kind })
    }

    /// The registration of `kind`.
    pub fn get(&self, kind: &str) -> Option<&'static FeatureRegistration> {
        self.by_kind.get(kind).copied()
    }

    /// All registered kinds, sorted.
    pub fn kinds(&self) -> impl Iterator<Item = &'static str> + '_ {
        self.by_kind.keys().copied()
    }

    /// Number of registered kinds.
    pub fn len(&self) -> usize {
        self.by_kind.len()
    }

    /// True if nothing is registered.
    pub fn is_empty(&self) -> bool {
        self.by_kind.is_empty()
    }
}

fn valid_kind(kind: &str) -> bool {
    !kind.is_empty()
        && kind.split('.').all(|seg| {
            !seg.is_empty()
                && seg
                    .bytes()
                    .all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'_' || b == b'-')
        })
}

static REGISTRY: OnceLock<Result<FeatureRegistry, RegistryError>> = OnceLock::new();

/// The process-wide registry, or why it is broken. Binaries call this (or [`registry`]) at
/// startup so a duplicate kind fails immediately rather than on first load.
pub fn try_registry() -> Result<&'static FeatureRegistry, &'static RegistryError> {
    REGISTRY
        .get_or_init(|| FeatureRegistry::from_registrations(inventory::iter::<FeatureRegistration>))
        .as_ref()
}

/// The process-wide registry.
///
/// # Panics
/// If the linked registrations are invalid (duplicate or malformed kind, version 0): a build
/// error that must fail at startup.
pub fn registry() -> &'static FeatureRegistry {
    match try_registry() {
        Ok(r) => r,
        Err(e) => panic!("invalid feature registry: {e}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn de(_: serde_json::Value) -> Result<Arc<dyn Feature>, serde_json::Error> {
        Err(serde::de::Error::custom("unused"))
    }

    static A: FeatureRegistration = FeatureRegistration {
        kind: "solid.extrude",
        version: 1,
        deserialize: de,
        migrate: no_migration,
    };
    static A2: FeatureRegistration = FeatureRegistration {
        kind: "solid.extrude",
        version: 2,
        deserialize: de,
        migrate: no_migration,
    };
    static BAD: FeatureRegistration = FeatureRegistration {
        kind: "Solid.Extrude",
        version: 1,
        deserialize: de,
        migrate: no_migration,
    };
    static ZERO: FeatureRegistration = FeatureRegistration {
        kind: "x",
        version: 0,
        deserialize: de,
        migrate: no_migration,
    };

    #[test]
    fn duplicate_and_malformed_kinds_are_rejected() {
        let r = FeatureRegistry::from_registrations([&A]).unwrap();
        assert_eq!(r.kinds().collect::<Vec<_>>(), ["solid.extrude"]);
        assert_eq!(r.get("solid.extrude").unwrap().version, 1);
        assert!(r.get("nope").is_none());
        assert_eq!(
            FeatureRegistry::from_registrations([&A, &A2]).unwrap_err(),
            RegistryError::DuplicateKind("solid.extrude")
        );
        assert_eq!(
            FeatureRegistry::from_registrations([&BAD]).unwrap_err(),
            RegistryError::InvalidKind("Solid.Extrude")
        );
        assert_eq!(
            FeatureRegistry::from_registrations([&ZERO]).unwrap_err(),
            RegistryError::ZeroVersion("x")
        );
    }

    #[test]
    fn kind_syntax() {
        for ok in [
            "a",
            "solid.extrude",
            "am.print-setup",
            "test.add_mesh",
            "v2.x9",
        ] {
            assert!(valid_kind(ok), "{ok}");
        }
        for bad in ["", ".a", "a.", "a..b", "A", "a b", "a/b", "é"] {
            assert!(!valid_kind(bad), "{bad}");
        }
    }

    #[test]
    fn global_registry_has_the_test_kinds() {
        let r = registry();
        for k in crate::testing::KINDS {
            assert!(r.get(k).is_some(), "{k} not registered");
        }
        assert_eq!(
            no_migration(1, serde_json::Value::Null).unwrap_err().0,
            "no migration from v1"
        );
    }
}
