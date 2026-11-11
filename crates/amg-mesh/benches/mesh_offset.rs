//! MESH-10 budget (S07): quick normal-offset shell of 1 M triangles < 300 ms. The budget is the
//! shell build (`quick_shell_*_build`, `offset_shell_normal_unchecked`); the MESH-19
//! self-intersection count of the output is reported and timed separately
//! (`quick_shell_*_si`), and `quick_shell_*` is the full `offset_shell_normal`. Run with
//! `cargo bench -p amg-mesh --bench mesh_offset`.

#![allow(clippy::expect_used, clippy::print_stderr)] // benchmark setup panics on bad fixtures

use amg_jobs::JobCtx;
use amg_mesh::corpus::bench::{BenchSize, benchmark_sphere};
use std::sync::Arc;

use amg_mesh::{
    BvhOpts, MeshSpatial, NormalOffsetOpts, SelfIntersectOpts, ShellSide, offset_shell_normal,
    offset_shell_normal_unchecked, self_intersections,
};
use criterion::{Criterion, criterion_group, criterion_main};

fn bench(cr: &mut Criterion) {
    let ctx = JobCtx::detached();
    let mut g = cr.benchmark_group("mesh_offset");
    g.sample_size(10);
    for (name, size) in [
        ("quick_shell_100k", BenchSize::K100),
        ("quick_shell_1M", BenchSize::M1),
    ] {
        let m = benchmark_sphere(size, 1);
        let opts = NormalOffsetOpts {
            thickness: 0.2,
            side: ShellSide::Inside,
            open_faces: Vec::new(),
        };
        let (s, r) = offset_shell_normal(&m, &opts, &ctx).expect("shell");
        eprintln!(
            "{name}: {} -> {} tris, {} SI pairs",
            m.face_count(),
            s.face_count(),
            r.self_intersecting_pairs
        );
        g.bench_function(format!("{name}_build"), |b| {
            b.iter(|| offset_shell_normal_unchecked(&m, &opts, &ctx).expect("shell"));
        });
        g.bench_function(name, |b| {
            b.iter(|| offset_shell_normal(&m, &opts, &ctx).expect("shell"));
        });
        // The part of the above spent on the MESH-19 count (BVH build + check of the output).
        let out = Arc::new(s);
        g.bench_function(format!("{name}_si"), |b| {
            b.iter(|| {
                let sp =
                    MeshSpatial::build(Arc::clone(&out), &BvhOpts::default(), &ctx).expect("bvh");
                self_intersections(&sp, &SelfIntersectOpts::default(), &ctx).expect("si")
            });
        });
    }
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
