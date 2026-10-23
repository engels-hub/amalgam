//! MESH-06 budget (S07): isotropic remesh of a 1 M-tri mesh to about 1 M tris, 5 iterations,
//! in < 10 s. Run with `cargo bench -p amg-mesh --bench mesh_remesh`.

#![allow(clippy::expect_used, clippy::print_stderr)] // benchmark setup panics on bad fixtures

#[path = "../tests/common/mod.rs"]
mod common;

use std::hint::black_box;

use amg_jobs::JobCtx;
use amg_mesh::corpus::bench::{BenchSize, benchmark_sphere};
use amg_mesh::{EdgeLength, IndexedMesh, RemeshOpts, Sizing, remesh};
use criterion::{Criterion, criterion_group, criterion_main};

fn bench_case(c: &mut Criterion, name: &str, m: &IndexedMesh) {
    let opts = RemeshOpts {
        sizing: Sizing::Uniform(EdgeLength::RelativeToMean(1.0)),
        iterations: 5,
        ..RemeshOpts::default()
    };
    let ctx = JobCtx::detached();
    let out = remesh(m, &opts, &ctx).expect("remesh");
    eprintln!("{name}: {} -> {} triangles", m.tris.len(), out.tris.len());
    let mut g = c.benchmark_group("mesh_remesh");
    g.sample_size(10);
    g.bench_function(name, |b| {
        b.iter_with_large_drop(|| remesh(black_box(m), &opts, &ctx).expect("remesh"));
    });
    g.finish();
}

fn bench(c: &mut Criterion) {
    // Displaced geodesic sphere, 1 003 520 triangles.
    let sphere = benchmark_sphere(BenchSize::M1, 1);
    bench_case(c, "displaced_sphere_1m_5it", &sphere);
    // Anisotropic torus, 1 000 000 triangles: real resampling work.
    let torus = common::torus(1000, 500, 30.0, 8.0);
    bench_case(c, "torus_1m_5it", &torus);
}

criterion_group!(benches, bench);
criterion_main!(benches);
