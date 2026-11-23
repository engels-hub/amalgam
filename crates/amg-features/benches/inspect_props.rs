//! INS-03 budgets (S05 §Performance targets): mass properties of a 1 M-tri mesh < 30 ms and of
//! a 10 M-tri mesh < 300 ms (the closed-mesh check cached, as in the panel after its first
//! run; the first call is printed too). Run with
//! `cargo bench -p amg-features --bench inspect_props`.

#![allow(missing_docs, clippy::expect_used, clippy::print_stderr)] // benchmark setup

use std::time::{Duration, Instant};

use amg_core::{Density, Transform, Vec3};
use amg_features::inspect::props::mesh_mass_props;
use amg_jobs::JobCtx;
use amg_mesh::MeshBody;
use amg_mesh::corpus::bench::geodesic_sphere;
use criterion::{Criterion, criterion_group, criterion_main};

fn bench(cr: &mut Criterion) {
    let ctx = JobCtx::detached();
    let density = Density::g_per_cm3(1.24);
    let t = Transform::from_translation(Vec3::new(10.0, 20.0, 30.0));
    let mut g = cr.benchmark_group("inspect_props");
    g.sample_size(20);
    g.measurement_time(Duration::from_secs(8));
    // 20·k² faces: k = 224 → 1 003 520, k = 707 → 9 996 980.
    for (name, k) in [("1M", 224u32), ("10M", 707)] {
        let body = MeshBody::new(geodesic_sphere(k, 50.0, None));
        let faces = body.mesh().face_count();
        let t0 = Instant::now();
        let first = mesh_mass_props(&body, &t, density, &ctx).expect("props");
        eprintln!(
            "{name}: {faces} faces, first call (with closed-mesh check) {:?}, V = {:?}",
            t0.elapsed(),
            first.volume.map(|v| v.get())
        );
        g.bench_function(format!("mass_props_{name}"), |b| {
            b.iter(|| mesh_mass_props(&body, &t, density, &ctx).expect("props"));
        });
    }
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
