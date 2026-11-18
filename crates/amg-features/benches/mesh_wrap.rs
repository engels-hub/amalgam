//! MESH-18 budget (S07): wrap of a 1 M-triangle scan at 0.5 mm voxels < 30 s. Run with
//! `cargo bench -p amg-features --bench mesh_wrap`.

#![allow(missing_docs, clippy::expect_used, clippy::print_stderr)] // benchmark setup

use std::sync::Arc;
use std::time::Duration;

use amg_features::mesh::wrap::{WrapOpts, wrap};
use amg_jobs::JobCtx;
use amg_mesh::corpus::bench::BenchSize;
use criterion::{Criterion, criterion_group, criterion_main};

#[path = "../tests/wrap_common/mod.rs"]
mod wrap_common;

fn bench(cr: &mut Criterion) {
    let ctx = JobCtx::detached();
    let mut g = cr.benchmark_group("mesh_wrap");
    g.sample_size(10);
    g.measurement_time(Duration::from_secs(60));
    let opts = WrapOpts::new(0.5);
    for (name, size) in [
        ("wrap_scan_100k_100mm_0.5", BenchSize::K100),
        ("wrap_scan_1M_100mm_0.5", BenchSize::M1),
    ] {
        let m = Arc::new(wrap_common::scan_part(size));
        let s = wrap(&m, &opts, &ctx).expect("wrap");
        eprintln!("{name}: {} -> {} tris", m.face_count(), s.face_count());
        g.bench_function(name, |b| {
            b.iter(|| wrap(&m, &opts, &ctx).expect("wrap"));
        });
    }
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
