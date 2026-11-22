//! IO-16 fuzz target `threemf_package`; the body is [`amg_io_fuzz::harness::threemf_package`].
#![no_main]

libfuzzer_sys::fuzz_target!(|data: &[u8]| amg_io_fuzz::run(
    amg_io_fuzz::harness::threemf_package,
    data
));
