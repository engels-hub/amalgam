//! MESH-08 budget (S07 §Performance): 1 M tris x 10 Taubin iterations < 300 ms.
//! Run with `cargo bench -p amg-mesh --bench smooth`.

#![allow(clippy::expect_used)] // benchmark setup panics on bad fixtures

#[path = "../tests/common/mod.rs"]
mod common;

use std::hint::black_box;

use amg_jobs::JobCtx;
use amg_mesh::{LaplacianWeights, SmoothMethod, SmoothOpts, smooth};
use criterion::{Criterion, criterion_group, criterion_main};

fn bench(c: &mut Criterion) {
    // 500 x 1000 quads = 1 000 000 triangles, closed.
    let m = common::torus(500, 1000, 50.0, 15.0);
    assert_eq!(m.face_count(), 1_000_000);
    let ctx = JobCtx::detached();
    let mut g = c.benchmark_group("smooth_1m");
    g.sample_size(10);
    let taubin = SmoothOpts::new(SmoothMethod::TAUBIN_DEFAULT, 10);
    g.bench_function("taubin_x10", |b| {
        b.iter_with_large_drop(|| smooth(black_box(&m), &taubin, &ctx).expect("smooth"))
    });
    let cot = SmoothOpts::new(
        SmoothMethod::Laplacian {
            lambda: 0.5,
            weights: LaplacianWeights::Cotangent,
        },
        10,
    );
    g.bench_function("laplacian_cotan_x10", |b| {
        b.iter_with_large_drop(|| smooth(black_box(&m), &cot, &ctx).expect("smooth"))
    });
    let bil = SmoothOpts::new(
        SmoothMethod::BilateralNormal {
            sigma_normal: 0.35,
            normal_iterations: 2,
            vertex_iterations: 5,
        },
        1,
    );
    g.bench_function("bilateral_1x(2,5)", |b| {
        b.iter_with_large_drop(|| smooth(black_box(&m), &bil, &ctx).expect("smooth"))
    });
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
