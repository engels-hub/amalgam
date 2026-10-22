//! MESH-02 budget (S07): analysis without self-intersections of a 1 M-triangle mesh in
//! < 100 ms and of a 10 M-triangle mesh in < 1.2 s.
//! Run with `cargo bench -p amg-mesh --bench mesh_analysis`; the 10 M case (about 1.5 GB of
//! RAM) only runs with `AMG_BENCH_10M=1`.

#![allow(clippy::expect_used)] // benchmark setup panics on bad fixtures

use std::hint::black_box;

use amg_jobs::JobCtx;
use amg_mesh::corpus::bench::{BenchSize, benchmark_sphere};
use amg_mesh::{AnalyzeOpts, analyze};
use criterion::{Criterion, criterion_group, criterion_main};

fn bench(c: &mut Criterion) {
    let opts = AnalyzeOpts::default();
    let ctx = JobCtx::detached();
    let mut g = c.benchmark_group("mesh_analysis");
    g.sample_size(20);
    for (name, size) in [("100k", BenchSize::K100), ("1m", BenchSize::M1)] {
        let m = benchmark_sphere(size, 1);
        g.bench_function(name, |b| {
            b.iter_with_large_drop(|| analyze(black_box(&m), &opts, &ctx).expect("analyze"))
        });
    }
    if std::env::var("AMG_BENCH_10M").is_ok_and(|v| v == "1") {
        let m = benchmark_sphere(BenchSize::M10, 1);
        g.sample_size(10);
        g.bench_function("10m", |b| {
            b.iter_with_large_drop(|| analyze(black_box(&m), &opts, &ctx).expect("analyze"))
        });
    }
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
