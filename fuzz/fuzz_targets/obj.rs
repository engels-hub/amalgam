//! IO-16 fuzz target `obj`; the body is [`amg_io_fuzz::harness::obj`].
#![no_main]

libfuzzer_sys::fuzz_target!(|data: &[u8]| amg_io_fuzz::run(amg_io_fuzz::harness::obj, data));
