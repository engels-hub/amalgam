//! `amg_brep::init` sizes OCCT's thread pool once per process (S02 §7). Its own test binary,
//! so no other test has called `init` first.
#![cfg(feature = "occt")]
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::num::NonZeroUsize;

use amg_brep::{KernelConfig, OcctKernel};
use amg_occt_sys::ffi;

#[test]
fn first_init_sets_the_pool_size_and_later_ones_keep_it() {
    let cfg = KernelConfig {
        occt_threads: NonZeroUsize::new(3),
        ..KernelConfig::default()
    };
    let _kernel = OcctKernel::new(cfg);
    let size = ffi::default_thread_pool_size().unwrap();
    assert_eq!(size, 3, "pool has {size} threads");

    amg_brep::init(&KernelConfig {
        occt_threads: NonZeroUsize::new(5),
        ..KernelConfig::default()
    });
    assert_eq!(ffi::default_thread_pool_size().unwrap(), 3);
}
