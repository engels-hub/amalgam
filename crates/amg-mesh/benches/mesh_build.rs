//! MESH-01 budgets (S07 §Performance): 1 M-tri half-edge build < 100 ms, weld of a 1 M-tri
//! soup < 80 ms. Run with `cargo bench -p amg-mesh --bench mesh_build`.

#![allow(clippy::expect_used)] // benchmark setup panics on bad fixtures

#[path = "../tests/common/mod.rs"]
mod common;

use std::hint::black_box;

use amg_mesh::{HalfEdgeMesh, IndexedMesh, MeshBody, NonManifoldPolicy, WeldOpts};
use criterion::{Criterion, criterion_group, criterion_main};

fn bench(c: &mut Criterion) {
    // 500 × 1000 quads = 1 000 000 triangles, closed.
    let m = common::torus(500, 1000, 50.0, 15.0);
    assert_eq!(m.face_count(), 1_000_000);
    let soup = common::soup(&m);

    let mut g = c.benchmark_group("mesh_1m");
    g.sample_size(20);
    g.bench_function("halfedge_build", |b| {
        b.iter_with_large_drop(|| {
            HalfEdgeMesh::from_indexed(black_box(&m), NonManifoldPolicy::SplitVertices)
        })
    });
    g.bench_function("halfedge_build_fail_policy", |b| {
        b.iter_with_large_drop(|| {
            HalfEdgeMesh::from_indexed(black_box(&m), NonManifoldPolicy::Fail)
        })
    });
    g.bench_function("weld_soup_exact", |b| {
        b.iter_with_large_drop(|| IndexedMesh::from_soup(black_box(&soup), WeldOpts::EXACT))
    });
    g.bench_function("weld_soup_tolerance_1e-5", |b| {
        b.iter_with_large_drop(|| IndexedMesh::from_soup(black_box(&soup), WeldOpts::default()))
    });
    let (he, _) = HalfEdgeMesh::from_indexed(&m, NonManifoldPolicy::Fail).expect("manifold");
    g.bench_function("halfedge_to_indexed", |b| {
        b.iter(|| black_box(&he).to_indexed())
    });
    let body = MeshBody::new(m.clone());
    let mut buf = Vec::new();
    body.write_binary(&mut buf).expect("write");
    g.bench_function("body_write_binary", |b| {
        b.iter(|| {
            let mut out = Vec::with_capacity(buf.len());
            black_box(&body).write_binary(&mut out).expect("write");
            out
        })
    });
    g.bench_function("body_read_binary", |b| {
        b.iter(|| MeshBody::read_binary(black_box(buf.as_slice())))
    });
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
