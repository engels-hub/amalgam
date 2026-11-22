//! IO-16 fuzz target `paint_decode`; the body is [`amg_io_fuzz::harness::paint_decode`].
#![no_main]

libfuzzer_sys::fuzz_target!(|data: &[u8]| amg_io_fuzz::run(
    amg_io_fuzz::harness::paint_decode,
    data
));
