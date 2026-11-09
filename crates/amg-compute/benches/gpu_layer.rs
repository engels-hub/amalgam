//! CMP-07 Done-when 4: 1 000 empty dispatches recorded and submitted in one `ComputeCtx` cost
//! at most 20 µs of CPU time each. Only recording and submission are measured; the wait for the
//! GPU is outside the measurement. Under `AMG_STRICT_TIMING=1` the bound is asserted. Not run
//! in CI.

#![allow(clippy::unwrap_used, reason = "benchmark code")]

use std::hint::black_box;
use std::time::{Duration, Instant};

use amg_compute::gpu::{ComputeCtx, GpuVec, Kernel};
use amg_jobs::JobCtx;
use amg_jobs::gpu::{Gpu, GpuConfig};
use criterion::{Criterion, Throughput, criterion_group, criterion_main};

const DISPATCHES: u64 = 1000;
const BUDGET: Duration = Duration::from_micros(20);

const EMPTY_WGSL: &str = r"
@group(0) @binding(0) var<storage, read_write> out: array<u32>;
@compute @workgroup_size(1)
fn main() { out[0] = 1u; }
";

fn empty_dispatches(c: &mut Criterion) {
    if !amg_testkit::gpu_available() {
        return;
    }
    let gpu = Gpu::new_blocking(&GpuConfig::from_env()).unwrap();
    eprintln!("adapter: {} ({:?})", gpu.info().name, gpu.info().backend);
    let job = JobCtx::detached();
    let kernel = Kernel::new(&gpu, "empty", EMPTY_WGSL, "main", &[]).unwrap();
    let out = GpuVec::<u32>::zeroed(&gpu, "out", 1).unwrap();
    let bind = kernel.bind_group(&gpu, 0, &[out.binding()]);

    let mut cx = ComputeCtx::new(gpu.clone(), "bench");
    let (mut total, mut batches) = (Duration::ZERO, 0u32);
    let mut group = c.benchmark_group("compute_ctx");
    group.throughput(Throughput::Elements(DISPATCHES));
    group.bench_function("empty_dispatch_record_and_submit", |b| {
        b.iter_custom(|iters| {
            let mut measured = Duration::ZERO;
            for _ in 0..iters {
                let start = Instant::now();
                for _ in 0..DISPATCHES {
                    cx.dispatch(black_box(&kernel), &[&bind], [1, 1, 1]);
                }
                cx.submit();
                measured += start.elapsed();
                cx.finish(&job).unwrap();
                cx.take_timings();
                batches += 1;
            }
            total += measured;
            measured
        });
    });
    group.finish();

    let per_dispatch = total / (batches * DISPATCHES as u32);
    eprintln!("empty dispatch: {per_dispatch:?} CPU each (budget {BUDGET:?}, {batches} batches)");
    if std::env::var_os("AMG_STRICT_TIMING").is_some() {
        assert!(
            per_dispatch <= BUDGET,
            "{per_dispatch:?} per dispatch exceeds the {BUDGET:?} budget"
        );
    }
}

criterion_group!(benches, empty_dispatches);
criterion_main!(benches);
