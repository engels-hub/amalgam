//! IO-16 fuzz target `roundtrip_scene`; the body is [`amg_io_fuzz::harness::roundtrip_scene`].
#![no_main]

libfuzzer_sys::fuzz_target!(|data: &[u8]| amg_io_fuzz::run(
    amg_io_fuzz::harness::roundtrip_scene,
    data
));
