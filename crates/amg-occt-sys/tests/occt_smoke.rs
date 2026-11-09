//! BREP-01 acceptance: the bridge links, computes and maps exceptions.
#![cfg(feature = "occt")]

use amg_occt_sys::{ffi, kernel_version};

#[test]
fn box_volume_is_exact() {
    let shape = ffi::make_box(10.0, 20.0, 30.0).unwrap();
    let volume = ffi::volume(&shape).unwrap();
    assert!((volume - 6000.0).abs() <= 1e-9, "volume = {volume}");
}

#[test]
fn standard_failure_becomes_err_with_type_name() {
    let error = ffi::gp_dir_from_zero_vector().unwrap_err();
    let message = error.what();
    assert!(
        message.contains("Standard_ConstructionError"),
        "message = {message}"
    );
}

#[test]
fn invalid_box_input_is_err_not_abort() {
    assert!(ffi::make_box(0.0, 1.0, 1.0).is_err());
}

#[test]
fn kernel_version_names_pinned_occt() {
    let version = kernel_version().unwrap();
    assert!(version.starts_with("occt-8.0.1;"), "version = {version}");
}
