//! BREP-12 budget: FFI overhead per op (box, no work) < 20 µs (S02 §Performance targets).
//!
//! `budget/brep-ffi-overhead` builds and drops a box through the bridge: one shim call, one
//! OCCT `BRepPrimAPI_MakeBox`, one `UniquePtr` drop. `ffi/*` isolate the bridge itself and the
//! progress bridge.
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "bench code")]

use std::hint::black_box;
use std::time::Duration;

use amg_brep::{KernelConfig, OcctKernel};
use amg_jobs::JobCtx;
use amg_occt_sys::ffi;
use criterion::{Criterion, criterion_group, criterion_main};

fn ffi_overhead(c: &mut Criterion) {
    let kernel = OcctKernel::new(KernelConfig::default());
    c.bench_function("budget/brep-ffi-overhead", |b| {
        b.iter(|| {
            let shape = ffi::make_box(black_box(10.0), 20.0, 30.0).unwrap();
            black_box(&shape);
        });
    });
    let shape = ffi::make_box(10.0, 20.0, 30.0).unwrap();
    c.bench_function("ffi/shape-is-null", |b| {
        b.iter(|| ffi::shape_is_null(black_box(&shape)).unwrap());
    });
    c.bench_function("ffi/max-tolerance-box", |b| {
        b.iter(|| ffi::shape_max_tolerance(black_box(&shape)).unwrap());
    });
    let ctx = JobCtx::detached();
    c.bench_function("ffi/progress-100-steps", |b| {
        b.iter(|| kernel.progress_probe(100, Duration::ZERO, &ctx).unwrap());
    });
}

criterion_group!(benches, ffi_overhead);
criterion_main!(benches);
