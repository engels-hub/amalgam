//! FFI bridge to OpenCASCADE: a cxx bridge over a coarse C++ shim (S02 §1, ADR-002).
//!
//! The bridge exists only with the `occt` feature. Only `amg-brep` may depend on this crate, and
//! OCCT types never appear in its API: Rust sees the opaque shim types and plain data.
//!
//! Spec: S02. Lane: A.

/// Whether this build compiled the shim and linked OCCT.
pub const OCCT_ENABLED: bool = cfg!(feature = "occt");

#[cfg(feature = "occt")]
mod bridge;

#[cfg(feature = "fixture-gen")]
mod fixtures;

#[cfg(feature = "occt")]
pub use bridge::ffi;

/// The cxx version the bridge is built with, for `amg-brep`'s `UniquePtr` and `Exception`.
#[cfg(feature = "occt")]
pub use cxx;

#[cfg(feature = "fixture-gen")]
pub use fixtures::fixture_ffi;

/// Identifies the OCCT build and the shim, for example `occt-8.0.1;shim-1`.
///
/// The OCCT part is `OCC_VERSION_COMPLETE`, and the shim's `static_assert` fails the C++ build
/// when that differs from the pinned version.
///
/// # Errors
/// Returns the C++ exception if the shim fails to format the string.
#[cfg(feature = "occt")]
pub fn kernel_version() -> Result<&'static str, cxx::Exception> {
    use std::sync::OnceLock;
    static VERSION: OnceLock<String> = OnceLock::new();
    if let Some(version) = VERSION.get() {
        return Ok(version);
    }
    let version = ffi::kernel_version()?;
    Ok(VERSION.get_or_init(|| version))
}

#[cfg(test)]
mod tests {
    #[test]
    fn enabled_flag_matches_feature() {
        assert_eq!(super::OCCT_ENABLED, cfg!(feature = "occt"));
    }
}
