//! AM-02 budgets (S16): `classify_faces` on 1 M triangles < 15 ms; `analyze_overhang` on 1 M
//! triangles with the fine support estimate (≤ 512² rays) < 300 ms and the coarse one
//! (≤ 128² rays) < 40 ms. Run with `cargo bench -p amg-am --bench overhang`. The medians are
//! printed always and asserted only under `AMG_STRICT_TIMING=1`.

#![allow(
    clippy::expect_used,
    clippy::print_stderr,
    clippy::print_stdout,
    missing_docs,
    reason = "benchmark setup"
)]

use std::hint::black_box;
use std::sync::Arc;
use std::time::Instant;

use amg_am::overhang::{OverhangOpts, SupportOpts, analyze_overhang, classify_faces};
use amg_am::place::{Placement, effective_transform};
use amg_core::glam::{DQuat, DVec2, DVec3};
use amg_jobs::JobCtx;
use amg_mesh::corpus::bench::{BenchSize, benchmark_sphere};
use amg_mesh::{BvhOpts, MeshSpatial};
use criterion::{Criterion, criterion_group, criterion_main};

fn median_ms(mut f: impl FnMut()) -> f64 {
    f(); // warm up (thread pool, caches)
    let mut t: Vec<f64> = (0..9)
        .map(|_| {
            let s = Instant::now();
            f();
            s.elapsed().as_secs_f64() * 1e3
        })
        .collect();
    t.sort_by(f64::total_cmp);
    t[t.len() / 2]
}

fn opts(cell: f64, max_rays: u32) -> OverhangOpts {
    OverhangOpts {
        critical_angle: 45f64.to_radians(),
        contact_tolerance: 0.1,
        support: Some(SupportOpts { cell, max_rays }),
    }
}

fn bench(cr: &mut Criterion) {
    let ctx = JobCtx::detached();
    let mesh = Arc::new(benchmark_sphere(BenchSize::M1, 7));
    let s = MeshSpatial::build(mesh.clone(), &BvhOpts::default(), &ctx).expect("bvh");
    let p = Placement::on_plate(
        DQuat::from_axis_angle(DVec3::new(1.0, 2.0, 3.0).normalize(), 0.7),
        DVec2::splat(110.0),
    );
    let t = effective_transform(&mesh, &p);
    // The sphere's footprint is ≈ 21.2 mm wide: these cells give ≈ 512² and ≈ 128² rays.
    let fine = opts(21.2 / 512.0, 512 * 512);
    let coarse = opts(21.2 / 128.0, 128 * 128);
    let class_only = OverhangOpts {
        support: None,
        ..fine.clone()
    };

    let classify_ms = median_ms(|| {
        black_box(classify_faces(&mesh, &t, &class_only, &ctx).expect("classify"));
    });
    let fine_rays = analyze_overhang(&s, &t, &fine, &ctx)
        .expect("fine")
        .metrics
        .support
        .expect("support")
        .rays;
    let coarse_rays = analyze_overhang(&s, &t, &coarse, &ctx)
        .expect("coarse")
        .metrics
        .support
        .expect("support")
        .rays;
    let fine_ms = median_ms(|| {
        black_box(analyze_overhang(&s, &t, &fine, &ctx).expect("fine"));
    });
    let coarse_ms = median_ms(|| {
        black_box(analyze_overhang(&s, &t, &coarse, &ctx).expect("coarse"));
    });
    eprintln!(
        "AM-02 {} tris: classify_faces {classify_ms:.2} ms (budget 15), analyze fine ({fine_rays} rays) {fine_ms:.1} ms (budget 300), coarse ({coarse_rays} rays) {coarse_ms:.1} ms (budget 40)",
        mesh.face_count()
    );
    if std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1") {
        assert!(classify_ms < 15.0, "classify_faces {classify_ms:.2} ms");
        assert!(fine_ms < 300.0, "analyze fine {fine_ms:.1} ms");
        assert!(coarse_ms < 40.0, "analyze coarse {coarse_ms:.1} ms");
    }

    let mut g = cr.benchmark_group("overhang_1M");
    g.sample_size(10);
    g.bench_function("classify_faces", |b| {
        b.iter(|| black_box(classify_faces(&mesh, &t, &class_only, &ctx).expect("classify")));
    });
    g.bench_function("analyze_coarse", |b| {
        b.iter(|| black_box(analyze_overhang(&s, &t, &coarse, &ctx).expect("coarse")));
    });
    g.bench_function("analyze_fine", |b| {
        b.iter(|| black_box(analyze_overhang(&s, &t, &fine, &ctx).expect("fine")));
    });
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
