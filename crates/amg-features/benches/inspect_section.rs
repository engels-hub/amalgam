//! INS-02 budget (S05 §Performance targets): section caps, one plane, 1 M tris < 50 ms and
//! 10 M tris < 500 ms. Run with `cargo bench -p amg-features --bench inspect_section`.

#![allow(missing_docs, clippy::expect_used, clippy::print_stderr)] // benchmark setup

use std::time::Duration;

use amg_core::{Dir3, Plane, Point3, Transform, Vec3};
use amg_features::inspect::section::{CapOpts, SectionPlanes, section_mesh};
use amg_jobs::JobCtx;
use amg_mesh::MeshBody;
use amg_mesh::corpus::bench::{BenchSize, benchmark_sphere};
use criterion::{Criterion, criterion_group, criterion_main};

fn bench(cr: &mut Criterion) {
    let ctx = JobCtx::detached();
    let opts = CapOpts::default();
    let planes = SectionPlanes(vec![Plane::from_normal(
        Point3::new(0.3, -0.2, 0.1),
        Dir3::new_unchecked(Vec3::new(0.3, 0.5, 0.8).normalize()),
    )]);
    let mut g = cr.benchmark_group("inspect_section");
    g.sample_size(20).measurement_time(Duration::from_secs(10));
    for (name, size) in [
        ("one_plane_1M", BenchSize::M1),
        ("one_plane_10M", BenchSize::M10),
    ] {
        let body = MeshBody::new(benchmark_sphere(size, 1));
        // Warm the per-mesh orientation flag (computed once per mesh, like a BVH).
        let r = section_mesh(&body, &Transform::IDENTITY, &planes, &opts, &ctx).expect("section");
        eprintln!(
            "{name}: {} faces, cap {} tris, area {:.3} mm², {} loops",
            body.mesh().face_count(),
            r.caps[0].tris.len(),
            r.stats[0].area.get(),
            r.stats[0].loops
        );
        g.bench_function(name, |bn| {
            bn.iter(|| {
                section_mesh(&body, &Transform::IDENTITY, &planes, &opts, &ctx).expect("section")
            });
        });
    }
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
