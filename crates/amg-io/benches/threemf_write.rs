//! IO-11 Done-when 6: `io/3mf_write_10m` (< 8 s).
//!
//! The scene is the MESH-28 benchmark mesh (displaced geodesic sphere, frequency 708:
//! 10 025 280 triangles), written with the default options (`Decimals(6)`, Deflate level 1,
//! validity `Require`, so the MESH-02 check is included) into memory. The bound is asserted only
//! under `AMG_STRICT_TIMING=1`.
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "bench code")]

mod common;

use std::time::{Duration, Instant};

use amg_io::{ExportOptions, Format, Sink, export};
use amg_jobs::JobCtx;
use criterion::{Criterion, criterion_group, criterion_main};

/// 20 · 708² = 10 025 280 triangles.
const FREQ: u32 = 708;
const BOUND: Duration = Duration::from_secs(8);

fn bench(c: &mut Criterion) {
    let scene = common::scene(common::mesh(FREQ));
    assert_eq!(scene.triangle_count(), 10_025_280);
    let opts = ExportOptions::default();
    let mut out = Vec::with_capacity(300 << 20);

    // One timed write first (reported; asserted under AMG_STRICT_TIMING=1).
    let t0 = Instant::now();
    let r = export(
        &scene,
        Sink::Memory(&mut out),
        Format::ThreeMf,
        &opts,
        &JobCtx::detached(),
    )
    .unwrap();
    let first = t0.elapsed();
    eprintln!(
        "io/3mf_write_10m: first write {first:?}, {} bytes, {} triangles",
        r.bytes, r.triangles
    );
    if common::strict() {
        assert!(first < BOUND, "3MF write of 10 M triangles took {first:?}");
    }

    let mut g = c.benchmark_group("io");
    g.sample_size(10);
    g.measurement_time(Duration::from_secs(60));
    g.bench_function("3mf_write_10m", |b| {
        b.iter(|| {
            out.clear();
            export(
                &scene,
                Sink::Memory(&mut out),
                Format::ThreeMf,
                &opts,
                &JobCtx::detached(),
            )
            .unwrap()
        });
    });
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
