//! INS-01 budgets (S05 §Performance targets, BVHs already built): point → 1 M-tri body
//! < 0.1 ms median; two separated 1 M-tri bodies (gap ≥ 1 mm) < 20 ms; concentric icospheres
//! r = 50 / 50.1 (1 M each) < 3 s. Run with `cargo bench -p amg-features --bench inspect_measure`.

#![allow(missing_docs, clippy::expect_used, clippy::print_stderr)] // benchmark setup

use std::time::Duration;

use amg_core::{Point3, Quat, Transform, Vec3};
use amg_features::inspect::measure::{
    MeasureOpts, MeshElement, MeshOperand, Operand, min_distance, prepare_body,
};
use amg_jobs::JobCtx;
use amg_mesh::corpus::bench::geodesic_sphere;
use amg_mesh::{IndexedMesh, MeshBody};
use criterion::{Criterion, criterion_group, criterion_main};

/// Geodesic sphere with ≈ 1 M faces (k = 224: 20·k² = 1 003 520).
fn sphere(r: f64) -> MeshBody {
    let m: IndexedMesh = geodesic_sphere(224, r, None);
    MeshBody::new(m)
}

fn body(b: &MeshBody, t: Transform) -> Operand {
    Operand::Mesh(MeshOperand {
        body: b.clone(),
        to_world: t,
        element: MeshElement::Body,
    })
}

fn bench(cr: &mut Criterion) {
    let ctx = JobCtx::detached();
    let opts = MeasureOpts::default();
    let a = sphere(50.0);
    let b = sphere(50.1);
    let c = sphere(30.0);
    for m in [&a, &b, &c] {
        prepare_body(m, &ctx).expect("prepare");
    }
    eprintln!(
        "faces: {} / {} / {}",
        a.mesh().face_count(),
        b.mesh().face_count(),
        c.mesh().face_count()
    );
    let mut g = cr.benchmark_group("inspect_measure");

    let p = Operand::Point(Point3::new(31.0, 40.0, 7.0));
    let ba = body(&a, Transform::IDENTITY);
    g.bench_function("point_to_1M_body", |bn| {
        bn.iter(|| min_distance(&p, &ba, &opts, &ctx).expect("distance"));
    });

    // Two 1 M bodies, 2 mm apart along a skew direction.
    let dir = Vec3::new(1.0, 0.3, -0.2).normalize();
    let bc = body(
        &c,
        Transform::from_rotation_translation(Quat::from_rotation_y(0.4), dir * 82.0),
    );
    let d = min_distance(&ba, &bc, &opts, &ctx).expect("distance");
    eprintln!("separated gap: {} mm", d.distance.to_mm());
    g.bench_function("separated_1M_1M", |bn| {
        bn.iter(|| min_distance(&ba, &bc, &opts, &ctx).expect("distance"));
    });

    g.sample_size(10);
    g.measurement_time(Duration::from_secs(40));
    let bb = body(&b, Transform::from_quat(Quat::from_rotation_z(0.01)));
    let d = min_distance(&ba, &bb, &opts, &ctx).expect("distance");
    eprintln!(
        "concentric gap: {} mm, contained {:?}",
        d.distance.to_mm(),
        d.contained
    );
    g.bench_function("concentric_1M_1M_gap_0.1", |bn| {
        bn.iter(|| min_distance(&ba, &bb, &opts, &ctx).expect("distance"));
    });
    g.finish();

    // Cancel latency of the concentric job (budget 50 ms): cancel 100 ms after the start.
    let mut worst = Duration::ZERO;
    for _ in 0..5 {
        let token = amg_jobs::CancelToken::new();
        let jc = JobCtx::with_token(token.clone());
        let (a2, b2) = (ba.clone(), bb.clone());
        let h = std::thread::spawn(move || min_distance(&a2, &b2, &MeasureOpts::default(), &jc));
        std::thread::sleep(Duration::from_millis(100));
        let t0 = std::time::Instant::now();
        token.cancel();
        let r = h.join().expect("join");
        let lat = t0.elapsed();
        eprintln!(
            "cancel: {:?} after {lat:?}",
            r.map(|d| d.distance.to_mm()).err()
        );
        worst = worst.max(lat);
    }
    eprintln!("concentric cancel latency, worst of 5: {worst:?}");
}

criterion_group!(benches, bench);
criterion_main!(benches);
