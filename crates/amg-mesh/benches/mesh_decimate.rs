//! MESH-07 budget (S07): QEM decimation of a 1 M-tri displaced sphere to 100 k in < 2 s.
//! Run with `cargo bench -p amg-mesh --bench mesh_decimate`.

#![allow(clippy::expect_used)] // benchmark setup panics on bad fixtures

use std::hint::black_box;

use amg_core::Point3;
use amg_jobs::JobCtx;
use amg_mesh::{DecimateOpts, DecimateTarget, IndexedMesh, decimate};
use criterion::{Criterion, criterion_group, criterion_main};

/// UV-free displaced sphere: icosphere with 1 310 720 triangles.
fn displaced_sphere() -> IndexedMesh {
    let s = amg_testkit::shapes::icosphere(Point3::ZERO, 10.0, 8);
    let mut m = IndexedMesh::new(s.positions, s.triangles).expect("valid icosphere");
    for p in &mut m.positions {
        let u = *p / 10.0;
        *p *= 1.0 + 0.08 * (3.0 * u.x).sin() * (2.0 * u.y).cos() + 0.04 * (5.0 * u.z).sin();
    }
    m
}

fn bench(c: &mut Criterion) {
    let m = displaced_sphere();
    let opts = DecimateOpts {
        target: DecimateTarget::FaceCount(100_000),
        ..DecimateOpts::default()
    };
    let ctx = JobCtx::detached();
    let mut g = c.benchmark_group("mesh_decimate");
    g.sample_size(10);
    g.bench_function("1_3m_to_100k", |b| {
        b.iter_with_large_drop(|| decimate(black_box(&m), &opts, &ctx).expect("decimate"))
    });
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
