//! IO-16 fuzz target `threemf_model`; the body is [`amg_io_fuzz::harness::threemf_model`].
#![no_main]

libfuzzer_sys::fuzz_target!(|data: &[u8]| amg_io_fuzz::run(
    amg_io_fuzz::harness::threemf_model,
    data
));
