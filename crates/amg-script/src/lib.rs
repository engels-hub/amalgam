//! Python API and headless batch.
//!
//! Spec: S18. Lane: E.
//!
//! Layers (S18 Design 1): [`ops`] (plain Rust, shared by the CLI and Python) on top of
//! [`host::ScriptDoc`], the document a script works on. The pyo3 module (SCR-01) and the
//! embedded interpreter (SCR-03) sit above them; this crate has no Python yet.

#![warn(missing_docs)]

mod error;
pub mod host;
pub mod ops;

pub use error::ScriptError;

/// Forces every feature registration to be linked (S01 `link()` convention) and checks the
/// registry. Every entry point calls it; [`host::ScriptDoc`] does so itself.
///
/// # Errors
/// [`amg_doc::RegistryError`] if two registrations clash or one is malformed (a build error).
pub fn link() -> Result<(), &'static amg_doc::RegistryError> {
    amg_features::am::link();
    amg_features::assembly::link();
    amg_features::construct::link();
    amg_features::io::link();
    amg_features::mesh::link();
    amg_features::sketch::link();
    amg_features::solid::link();
    amg_doc::try_registry().map(|_| ())
}
