//! MESH-19 budget (S07): self-intersection check of a 1 M-triangle mesh in < 500 ms.
//! Run with `cargo bench -p amg-mesh --bench mesh_selfint`.

#![allow(clippy::expect_used)] // benchmark setup panics on bad fixtures

use std::hint::black_box;
use std::sync::Arc;

use amg_core::Point3;
use amg_jobs::JobCtx;
use amg_mesh::corpus::bench::{BenchSize, benchmark_sphere};
use amg_mesh::{
    BvhOpts, FaceId, IndexedMesh, MeshSpatial, SelfIntersectOpts, WeldOpts, self_intersections,
};
use criterion::{Criterion, criterion_group, criterion_main};

fn bench(c: &mut Criterion) {
    let ctx = JobCtx::detached();
    let clean = benchmark_sphere(BenchSize::M1, 1);
    // Two overlapping displaced spheres (about 200 k triangles) so that real hits are
    // reported.
    let half = benchmark_sphere(BenchSize::K100, 2);
    let n = half.tris.len() as u32;
    let mut soup: Vec<[Point3; 3]> = (0..n).map(|f| half.triangle(FaceId(f))).collect();
    soup.extend((0..n).map(|f| {
        half.triangle(FaceId(f))
            .map(|p| p + Point3::new(3.0, 0.0, 0.0))
    }));
    let crossing = IndexedMesh::from_soup(&soup, WeldOpts::EXACT).expect("merge");

    let mut g = c.benchmark_group("mesh_selfint");
    g.sample_size(10);
    for (name, m) in [("1m_clean", clean), ("200k_crossing", crossing)] {
        let s = MeshSpatial::build(Arc::new(m), &BvhOpts::default(), &ctx).expect("build");
        let found = self_intersections(&s, &SelfIntersectOpts::default(), &ctx).expect("check");
        eprintln!("{name}: {} pairs", found.len());
        g.bench_function(format!("{name}_check"), |b| {
            b.iter_with_large_drop(|| {
                self_intersections(black_box(&s), &SelfIntersectOpts::default(), &ctx)
                    .expect("check")
            })
        });
        let mesh = s.mesh().clone();
        g.bench_function(format!("{name}_build_and_check"), |b| {
            b.iter_with_large_drop(|| {
                let s = MeshSpatial::build(mesh.clone(), &BvhOpts::default(), &ctx).expect("build");
                self_intersections(&s, &SelfIntersectOpts::default(), &ctx).expect("check")
            })
        });
    }
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
