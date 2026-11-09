//! The cxx bridge. Every function returns `Result`, so a C++ exception can't reach
//! `std::terminate`: the shim's `rust::behavior::trycatch` turns each one into an `Err`.
// The bridge macro expands to unsafe FFI glue, which cxx checks against the C++ declarations.
// Opaque C++ types have nothing Rust could print, so they don't implement `Debug`.
#![allow(unsafe_code, missing_debug_implementations)]

#[cxx::bridge(namespace = "amg::occt")]
pub mod ffi {
    unsafe extern "C++" {
        include!("amg-occt-sys/cpp/occt_shim.hxx");

        /// An owned `TopoDS_Shape`. Opaque: OCCT types stay on the C++ side.
        type OcShape;

        /// `occt-<OCC_VERSION_COMPLETE>;shim-<N>`.
        fn kernel_version() -> Result<String>;

        /// An axis-aligned box with one corner at the origin; sizes in mm.
        fn make_box(dx: f64, dy: f64, dz: f64) -> Result<UniquePtr<OcShape>>;

        /// The volume of `shape` in mm³.
        fn volume(shape: &OcShape) -> Result<f64>;

        /// Builds a `gp_Dir` from a zero vector, which raises `Standard_ConstructionError`.
        /// Exists to test the exception path (BREP-01 Done-when 2).
        fn gp_dir_from_zero_vector() -> Result<()>;
    }
}
