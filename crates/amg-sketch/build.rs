//! Build script: only does something with the `slvs` feature (SK-13 libslvs oracle).
//!
//! The `slvs` crate compiles SolveSpace with mimalloc, which calls `OpenProcessToken` &co. on
//! Windows but does not link `advapi32` itself.

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    let slvs = std::env::var_os("CARGO_FEATURE_SLVS").is_some();
    let windows = std::env::var("CARGO_CFG_TARGET_OS").is_ok_and(|os| os == "windows");
    if slvs && windows {
        println!("cargo:rustc-link-lib=advapi32");
    }
}
