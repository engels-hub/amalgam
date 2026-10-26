//! MESH-09 budget (S07): a plane cut of a closed 1 M-triangle mesh, caps included, takes
//! < 300 ms. Run with `cargo bench -p amg-mesh --bench mesh_cut`.

#![allow(clippy::expect_used, clippy::print_stderr)] // benchmark setup panics on bad fixtures

#[path = "../tests/cut_common/mod.rs"]
mod lat;

use amg_core::{Dir3, Plane, Point3, Vec3};
use amg_jobs::JobCtx;
use amg_mesh::{IndexedMesh, KeepSide, PlaneCutOpts, plane_cut};
use criterion::{Criterion, criterion_group, criterion_main};

fn plane(origin: Point3, normal: Vec3) -> Plane {
    Plane::from_normal(origin, Dir3::new_unchecked(normal))
}

fn bench(cr: &mut Criterion) {
    let ctx = JobCtx::detached();
    let sphere = {
        let s = amg_testkit::shapes::icosphere(Point3::ZERO, 50.0, 8);
        IndexedMesh::new(s.positions, s.triangles).expect("sphere")
    };
    let boxed = lat::lattice_box(289);
    let diag = Vec3::new(1.0, 1.0, 0.0).normalize();
    let cases: Vec<(&str, &IndexedMesh, Plane)> = vec![
        (
            "sphere_1.3M_oblique",
            &sphere,
            plane(
                Point3::new(3.0, 1.0, -2.0),
                Vec3::new(1.0, 2.0, 3.0).normalize(),
            ),
        ),
        (
            "box_1M_generic",
            &boxed,
            plane(Point3::new(100.5, 0.0, 0.0), Vec3::X),
        ),
        (
            "box_1M_through_vertices",
            &boxed,
            plane(Point3::new(100.0, 0.0, 0.0), Vec3::X),
        ),
        (
            "box_1M_through_diagonals",
            &boxed,
            plane(Point3::new(200.0, 0.0, 0.0), diag),
        ),
    ];
    let mut g = cr.benchmark_group("mesh_cut");
    g.sample_size(10);
    for (name, m, p) in cases {
        let opts = PlaneCutOpts {
            plane: p,
            keep: KeepSide::Both,
            cap: true,
            snap_tolerance: 0.0,
        };
        let r = plane_cut(m, &opts, &ctx).expect("cut");
        eprintln!(
            "{name}: {} faces in, {} + {} out, open loops {}",
            m.face_count(),
            r.positive.as_ref().map_or(0, IndexedMesh::face_count),
            r.negative.as_ref().map_or(0, IndexedMesh::face_count),
            r.open_loops
        );
        g.bench_function(name, |b| {
            b.iter(|| plane_cut(m, &opts, &ctx).expect("cut"));
        });
    }
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
