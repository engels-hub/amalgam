//! BREP-09 measurement queries. S02 sets no budget for them; these benches record their cost
//! for S05 (inspection) and S10 (picking): `mass_props`, `closest_point`, `ray_cast`,
//! `distance` and `eval_surface` on a cylinder and on the 1 030-face `plate_with_holes`.
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "benchmark code")]

use std::f64::consts::TAU;
use std::hint::black_box;

use amg_brep::{
    BrepKernel, Dir3, FaceId, KernelConfig, OcctKernel, Plane, Point2, Point3, Ray3, Shape, Vec3,
};
use criterion::{Criterion, criterion_group, criterion_main};

fn bench(c: &mut Criterion) {
    let kernel = OcctKernel::new(KernelConfig::default());
    let cyl = kernel
        .make_cylinder(&Plane::XY, 5.0, 12.0, TAU)
        .expect("cylinder")
        .shape;
    let sphere = kernel
        .make_sphere(
            &Plane {
                origin: Point3::new(20.0, 0.0, 6.0),
                ..Plane::XY
            },
            3.0,
        )
        .expect("sphere")
        .shape;
    let ray = Ray3 {
        origin: Point3::new(30.0, 1.0, 6.0),
        dir: Dir3::new_unchecked(-Vec3::X),
    };

    let mut group = c.benchmark_group("brep-query");
    group.bench_function("mass-props-cylinder", |b| {
        b.iter(|| black_box(kernel.mass_props(&cyl).unwrap()));
    });
    group.bench_function("closest-point-cylinder", |b| {
        b.iter(|| {
            black_box(
                kernel
                    .closest_point(&cyl, Point3::new(9.0, 2.0, 3.0))
                    .unwrap(),
            )
        });
    });
    group.bench_function("ray-cast-cylinder", |b| {
        b.iter(|| black_box(kernel.ray_cast(&cyl, &ray, 1e3).unwrap()));
    });
    group.bench_function("distance-cylinder-sphere", |b| {
        b.iter(|| black_box(kernel.distance(&cyl, &sphere).unwrap()));
    });
    group.bench_function("eval-surface-cylinder", |b| {
        let side = cyl
            .faces()
            .find(|&f| matches!(cyl.surface(f), amg_brep::geom::Surface::Cylinder { .. }))
            .unwrap_or(FaceId(0));
        b.iter(|| {
            black_box(
                kernel
                    .eval_surface(&cyl, side, Point2::new(1.0, 4.0))
                    .unwrap(),
            )
        });
    });

    if let Some(path) = amg_testkit::testdata("brep/plate_with_holes.brep") {
        let file = std::fs::File::open(path).expect("open fixture");
        let plate = Shape::read_text(std::io::BufReader::new(file)).expect("read fixture");
        let bbox = plate.bbox();
        let down = Ray3 {
            origin: bbox.center() + Vec3::new(0.3, 0.2, bbox.size().z),
            dir: Dir3::new_unchecked(-Vec3::Z),
        };
        group.sample_size(10);
        group.bench_function("mass-props-plate-1030", |b| {
            b.iter(|| black_box(kernel.mass_props(&plate).unwrap()));
        });
        group.bench_function("ray-cast-plate-1030", |b| {
            b.iter(|| black_box(kernel.ray_cast(&plate, &down, 1e3).unwrap()));
        });
        group.bench_function("closest-point-plate-1030", |b| {
            b.iter(|| black_box(kernel.closest_point(&plate, bbox.max + Vec3::ONE).unwrap()));
        });
    }
    group.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
