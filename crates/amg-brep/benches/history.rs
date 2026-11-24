//! BREP-05: `budget/brep-sub-geom-all-1000` (S02 §Performance targets: `sub_geom_all` on a
//! 1000-face part < 20 ms) on the 1030-face perforated plate (32 × 32 holes), plus the costs
//! around the history framework: one `sub_geom`, a `compose` of a prism and its unify step,
//! `order_splits` of a boolean history and `per_solid`.
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "benchmark code")]

#[path = "../tests/common/parts.rs"]
mod parts;

use std::f64::consts::TAU;
use std::hint::black_box;

use amg_brep::geom::{Curve2d, Loop2, PlanarProfile, ProfileEdge2, Tag};
use amg_brep::{
    BoolOp, BoolOpts, BrepKernel, JobCtx, KernelConfig, OcctKernel, Plane, Point2, Point3,
    PrismExtent, PrismOpts, SubKind, SubShape, Tolerance, Vec3,
};
use criterion::{Criterion, criterion_group, criterion_main};

fn bench(c: &mut Criterion) {
    let k = OcctKernel::new(KernelConfig::default());
    let ctx = JobCtx::detached();
    let plate = parts::plate(&k, 32, Point3::ZERO);
    assert_eq!(plate.count(SubKind::Face), 1030);
    c.bench_function("budget/brep-sub-geom-all-1000", |b| {
        b.iter(|| black_box(k.sub_geom_all(&plate).unwrap()));
    });

    let mut group = c.benchmark_group("brep-history");
    let hole_wall = SubShape {
        kind: SubKind::Face,
        index: 10,
    };
    group.bench_function("sub-geom-one-face", |b| {
        b.iter(|| black_box(k.sub_geom(&plate, hole_wall).unwrap()));
    });

    // A square with every side split in two: prism, then the unify step.
    let pts = [
        (0.0, 0.0),
        (6.0, 0.0),
        (12.0, 0.0),
        (12.0, 6.0),
        (12.0, 12.0),
        (6.0, 12.0),
        (0.0, 12.0),
        (0.0, 6.0),
    ];
    let edges = (0..pts.len())
        .map(|i| {
            let (a, b) = (pts[i], pts[(i + 1) % pts.len()]);
            ProfileEdge2 {
                curve: Curve2d::Line {
                    a: Point2::new(a.0, a.1),
                    b: Point2::new(b.0, b.1),
                },
                tag: Tag(i as u64),
            }
        })
        .collect();
    let profile = PlanarProfile {
        plane: Plane::XY,
        outer: Loop2 { edges },
        holes: vec![],
    };
    let face = k
        .make_planar_faces(&[profile], &Tolerance::DEFAULT)
        .unwrap()
        .remove(0)
        .face;
    let raw = k
        .prism(
            &face,
            &PrismExtent::Vector(Vec3::new(0.0, 0.0, 5.0)),
            &PrismOpts {
                taper: 0.0,
                merge_coplanar: false,
            },
            &ctx,
        )
        .unwrap();
    let unified = k.unify_coplanar(&raw.shape).unwrap();
    group.bench_function("compose-prism-unify", |b| {
        b.iter(|| black_box(raw.history.compose(&unified.history)));
    });

    // A bar cut by five cylinders across its top: the top face splits into six strips to order.
    let bar = k
        .make_box(&Plane::XY, Vec3::new(60.0, 10.0, 10.0))
        .unwrap()
        .shape;
    let tools: Vec<_> = (0..5)
        .map(|i| {
            let frame = Plane {
                origin: Point3::new(5.0 + 12.0 * f64::from(i), 11.0, 10.0),
                x: amg_brep::Dir3::X,
                y: amg_brep::Dir3::Z,
                normal: amg_brep::Dir3::new_unchecked(-Vec3::Y),
            };
            k.make_cylinder(&frame, 2.0, 12.0, TAU).unwrap().shape
        })
        .collect();
    let cut = k
        .boolean(
            BoolOp::Cut,
            std::slice::from_ref(&bar),
            &tools,
            &BoolOpts::default(),
            &ctx,
        )
        .unwrap();
    group.bench_function("order-splits-cut", |b| {
        b.iter(|| {
            let mut h = cut.history.clone();
            h.order_splits(&cut.shape).unwrap();
            black_box(h)
        });
    });
    group.bench_function("per-solid-cut", |b| {
        b.iter(|| black_box(cut.per_solid().unwrap()));
    });
    group.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
