//! S02 budget: `write_bin` + `read_bin` of a 1000-face part in under 30 ms.
//!
//! `budget/brep-bin-roundtrip-1000` uses a shape that came from `read_bin`, the steady state for
//! saved documents. A shape built or read some other way pays one extra `BinTools` round trip on
//! write (see `Occ::write_bin`); `first-write-1000` measures that case.
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "benchmark code")]

use std::hint::black_box;

use amg_brep::Shape;
use criterion::{Criterion, criterion_group, criterion_main};

fn write_bin(shape: &Shape) -> Vec<u8> {
    let mut bytes = Vec::new();
    shape.write_bin(&mut bytes).expect("write_bin");
    bytes
}

fn bench(c: &mut Criterion) {
    let Some(path) = amg_testkit::testdata("brep/plate_with_holes.brep") else {
        return;
    };
    let file = std::fs::File::open(path).expect("open fixture");
    let from_text = Shape::read_text(std::io::BufReader::new(file)).expect("read fixture");
    let from_bin = Shape::read_bin(write_bin(&from_text).as_slice()).expect("read_bin");

    let mut group = c.benchmark_group("budget");
    group.sample_size(30);
    group.bench_function("brep-bin-roundtrip-1000", |b| {
        b.iter(|| black_box(Shape::read_bin(write_bin(&from_bin).as_slice()).expect("read_bin")));
    });
    group.finish();

    let mut group = c.benchmark_group("brep-io");
    group.sample_size(30);
    group.bench_function("first-write-1000", |b| {
        b.iter(|| black_box(write_bin(&from_text)));
    });
    group.bench_function("write-bin-1000", |b| {
        b.iter(|| black_box(write_bin(&from_bin)));
    });
    let bytes = write_bin(&from_bin);
    group.bench_function("read-bin-1000", |b| {
        b.iter(|| black_box(Shape::read_bin(bytes.as_slice()).expect("read_bin")));
    });
    group.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
