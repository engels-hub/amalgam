//! Test-fixture shapes for `amg-brep`'s generator (`fixture-gen` feature, BREP-02).
// See `bridge.rs`: the macro expands to unsafe FFI glue that cxx checks.
#![allow(unsafe_code, missing_debug_implementations)]

#[cxx::bridge(namespace = "amg::occt")]
pub mod fixture_ffi {
    unsafe extern "C++" {
        include!("amg-occt-sys/cpp/occt_fixtures.hxx");

        type OcShape = crate::bridge::ffi::OcShape;

        /// Builds the named fixture part. Raises on an unknown name.
        fn fixture_shape(name: &str) -> Result<UniquePtr<OcShape>>;
    }
}
