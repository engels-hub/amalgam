//! B-rep kernel: `BrepKernel` trait, OCCT backend, shape handles, tessellation, topology and history.
//!
//! Spec: S02, S20. Lane: A.

/// Whether this build links the OCCT backend (cargo feature `occt`).
pub const OCCT_BACKEND: bool = cfg!(feature = "occt");

#[cfg(test)]
mod tests {
    #[test]
    fn backend_flag_matches_occt_sys() {
        assert_eq!(super::OCCT_BACKEND, amg_occt_sys::OCCT_ENABLED);
    }
}
