//! AM-03 budgets (S16): `effective_transform` + `check_fit` on 1 M triangles < 15 ms and
//! `footprint_hull` on 1 M triangles < 40 ms. Run with `cargo bench -p amg-am --bench place`.
//! The medians are printed always and asserted only under `AMG_STRICT_TIMING=1`.

#![allow(
    clippy::expect_used,
    clippy::print_stderr,
    clippy::print_stdout,
    missing_docs,
    reason = "benchmark setup"
)]

use std::hint::black_box;
use std::time::Instant;

use amg_am::machine::{BedShape, BuildVolume};
use amg_am::place::{Placement, check_fit, effective_transform, footprint_hull};
use amg_core::glam::{DQuat, DVec2, DVec3};
use amg_jobs::JobCtx;
use amg_mesh::corpus::bench::{BenchSize, benchmark_sphere};
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

fn bench(cr: &mut Criterion) {
    let mesh = benchmark_sphere(BenchSize::M1, 7);
    let vol = BuildVolume {
        bed: BedShape::Rect {
            size: DVec2::splat(220.0),
        },
        height: 250.0,
        exclude: vec![vec![
            DVec2::new(0.0, 0.0),
            DVec2::new(20.0, 0.0),
            DVec2::new(20.0, 20.0),
        ]],
        slicer_origin: DVec2::ZERO,
    };
    let p = Placement::on_plate(
        DQuat::from_axis_angle(DVec3::new(1.0, 2.0, 3.0).normalize(), 0.7),
        DVec2::splat(110.0),
    );
    let ctx = JobCtx::detached();
    let t = effective_transform(&mesh, &p);

    let fit_ms = median_ms(|| {
        let t = effective_transform(&mesh, &p);
        black_box(check_fit(&mesh, &t, &vol, &ctx).expect("fit"));
    });
    let hull_ms = median_ms(|| {
        black_box(footprint_hull(&mesh, &t));
    });
    eprintln!(
        "AM-03 {} tris: effective_transform+check_fit {fit_ms:.2} ms (budget 15), footprint_hull {hull_ms:.2} ms (budget 40)",
        mesh.face_count()
    );
    if std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1") {
        assert!(
            fit_ms < 15.0,
            "effective_transform+check_fit {fit_ms:.2} ms"
        );
        assert!(hull_ms < 40.0, "footprint_hull {hull_ms:.2} ms");
    }

    let mut g = cr.benchmark_group("place_1M");
    g.sample_size(20);
    g.bench_function("effective_transform_check_fit", |b| {
        b.iter(|| {
            let t = effective_transform(&mesh, &p);
            black_box(check_fit(&mesh, &t, &vol, &ctx).expect("fit"))
        });
    });
    g.bench_function("footprint_hull", |b| {
        b.iter(|| black_box(footprint_hull(&mesh, &t)));
    });
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
