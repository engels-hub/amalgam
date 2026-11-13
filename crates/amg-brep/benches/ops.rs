//! BREP-14 budgets (S02 "Performance targets"): a prism of a 100-edge profile (< 10 ms), plus
//! the primitives (the BREP-12 `budget/brep-ffi-overhead` note: building a box is OCCT's
//! topology build, not the bridge), a tapered prism and a revol. Validation is off, as for a
//! release build's default policy.
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "bench code")]

use std::f64::consts::TAU;
use std::hint::black_box;

use amg_brep::geom::{Curve2d, Loop2, PlanarProfile, ProfileEdge2, Tag};
use amg_brep::{
    BrepKernel, Dir3, JobCtx, KernelConfig, Line3, OcctKernel, PrismExtent, PrismOpts, Shape,
    Validate,
};
use amg_core::{Plane, Point2, Point3, Tolerance, Vec3};
use criterion::{Criterion, criterion_group, criterion_main};

fn kernel() -> OcctKernel {
    OcctKernel::new(KernelConfig {
        validate: Validate::Off,
        ..KernelConfig::default()
    })
}

/// A star polygon with `n` edges.
fn star(k: &OcctKernel, n: u32) -> Shape {
    let points: Vec<Point2> = (0..n)
        .map(|i| {
            let a = TAU * f64::from(i) / f64::from(n);
            let r = if i % 2 == 0 { 10.0 } else { 7.0 };
            Point2::new(r * a.cos(), r * a.sin())
        })
        .collect();
    let edges = (0..points.len())
        .map(|i| ProfileEdge2 {
            curve: Curve2d::Line {
                a: points[i],
                b: points[(i + 1) % points.len()],
            },
            tag: Tag(i as u64),
        })
        .collect();
    let profile = PlanarProfile {
        plane: Plane::XY,
        outer: Loop2 { edges },
        holes: vec![],
    };
    k.make_planar_faces(&[profile], &Tolerance::DEFAULT)
        .unwrap()
        .remove(0)
        .face
}

fn bench(c: &mut Criterion) {
    let k = kernel();
    let ctx = JobCtx::detached();
    let profile = star(&k, 100);
    let straight = PrismOpts {
        taper: 0.0,
        merge_coplanar: false,
    };
    let up = PrismExtent::Vector(Vec3::new(0.0, 0.0, 5.0));
    c.bench_function("budget/brep-prism-100", |b| {
        b.iter(|| black_box(k.prism(black_box(&profile), &up, &straight, &ctx).unwrap()));
    });
    let small = star(&k, 8);
    let tapered = PrismOpts {
        taper: 0.05,
        merge_coplanar: false,
    };
    c.bench_function("brep/prism-taper-8", |b| {
        b.iter(|| black_box(k.prism(black_box(&small), &up, &tapered, &ctx).unwrap()));
    });
    let axis = Line3 {
        point: Point3::new(20.0, 0.0, 0.0),
        dir: Dir3::Y,
    };
    c.bench_function("brep/revol-100-90deg", |b| {
        b.iter(|| {
            black_box(
                k.revol(black_box(&profile), &axis, TAU / 4.0, &ctx)
                    .unwrap(),
            )
        });
    });
    c.bench_function("brep/make-box", |b| {
        b.iter(|| {
            black_box(
                k.make_box(&Plane::XY, black_box(Vec3::new(10.0, 20.0, 30.0)))
                    .unwrap(),
            )
        });
    });
    c.bench_function("brep/make-sphere", |b| {
        b.iter(|| black_box(k.make_sphere(&Plane::XY, black_box(10.0)).unwrap()));
    });
}

criterion_group! {
    name = benches;
    config = Criterion::default().sample_size(30);
    targets = bench
}
criterion_main!(benches);
