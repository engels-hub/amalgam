//! S20 KH-01 budget: `OnFailure` capture adds < 1 µs per successful call (Done-when 6).
//!
//! - `budget/capture-on-failure-overhead`: `RecordingKernel::guarded` around a call that
//!   succeeds and does no work, so the measured time is the decorator itself (mode check,
//!   clock read, `catch_unwind` frame). Compare with `capture/direct-ok`.
//! - `capture/make-box-direct` vs `capture/make-box-recording` (OCCT builds only): the same
//!   comparison on a real kernel op.
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "bench code")]

use std::hint::black_box;
use std::sync::Arc;

use amg_brep::capture::{CaptureConfig, FailureRecorder, RecordingKernel};
use amg_brep::{KResult, KernelConfig, OcctKernel};
use criterion::{Criterion, criterion_group, criterion_main};

fn recording() -> RecordingKernel<OcctKernel> {
    // The directory is never created: successful calls do no file I/O.
    let dir = std::env::temp_dir().join("amg-kh01-bench-never-created");
    let recorder = FailureRecorder::new(CaptureConfig::in_dir(dir)).unwrap();
    RecordingKernel::new(OcctKernel::new(KernelConfig::default()), Arc::new(recorder))
}

fn overhead(c: &mut Criterion) {
    let k = recording();
    c.bench_function("budget/capture-on-failure-overhead", |b| {
        b.iter(|| {
            let r: KResult<u64> = k.guarded(
                || unreachable!("the op is only built on failure"),
                Vec::new,
                || Ok(black_box(7)),
            );
            black_box(r.ok())
        });
    });
    c.bench_function("capture/direct-ok", |b| {
        b.iter(|| {
            let r: KResult<u64> = Ok(black_box(7));
            black_box(r.ok())
        });
    });
    #[cfg(feature = "occt")]
    {
        use amg_brep::{BrepKernel, Plane, Vec3};
        let direct = OcctKernel::new(KernelConfig::default());
        c.bench_function("capture/make-box-direct", |b| {
            b.iter(|| direct.make_box(&Plane::XY, black_box(Vec3::ONE)).unwrap());
        });
        c.bench_function("capture/make-box-recording", |b| {
            b.iter(|| k.make_box(&Plane::XY, black_box(Vec3::ONE)).unwrap());
        });
    }
}

criterion_group!(benches, overhead);
criterion_main!(benches);
