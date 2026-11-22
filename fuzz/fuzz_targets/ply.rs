//! IO-16 fuzz target `ply`; the body is [`amg_io_fuzz::harness::ply`].
#![no_main]

libfuzzer_sys::fuzz_target!(|data: &[u8]| amg_io_fuzz::run(amg_io_fuzz::harness::ply, data));
