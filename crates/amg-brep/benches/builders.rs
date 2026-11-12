//! BREP-03 budgets (S02 "Performance targets"): planar faces for a 50-region sketch (< 10 ms)
//! and a 1000-vertex polygon with 10 holes (< 20 ms, Done-when 3). Validation is off, as for a
//! release build's default policy.
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "bench code")]

use std::f64::consts::TAU;
use std::hint::black_box;

use amg_brep::geom::{Curve2d, Loop2, PlanarProfile, ProfileEdge2, Tag};
use amg_brep::{BrepKernel, KernelConfig, OcctKernel, Validate};
use amg_core::{Plane, Point2, Point3, Tolerance};
use criterion::{Criterion, criterion_group, criterion_main};

fn kernel() -> OcctKernel {
    OcctKernel::new(KernelConfig {
        validate: Validate::Off,
        ..KernelConfig::default()
    })
}

/// 50 regions: a square cell with a rounded corner (line, arc) and a circular hole each.
fn sketch() -> Vec<PlanarProfile> {
    (0..50)
        .map(|i| {
            let (x, y) = (f64::from(i % 10) * 12.0, f64::from(i / 10) * 12.0);
            let p = |u: f64, v: f64| Point2::new(x + u, y + v);
            let line = |a: Point2, b: Point2, t: u64| ProfileEdge2 {
                curve: Curve2d::Line { a, b },
                tag: Tag(t),
            };
            let outer = Loop2 {
                edges: vec![
                    line(p(0.0, 0.0), p(10.0, 0.0), 0),
                    line(p(10.0, 0.0), p(10.0, 8.0), 1),
                    ProfileEdge2 {
                        curve: Curve2d::Arc {
                            center: p(8.0, 8.0),
                            radius: 2.0,
                            start_angle: 0.0,
                            sweep: TAU / 4.0,
                        },
                        tag: Tag(2),
                    },
                    line(p(8.0, 10.0), p(0.0, 10.0), 3),
                    line(p(0.0, 10.0), p(0.0, 0.0), 4),
                ],
            };
            let hole = Loop2 {
                edges: vec![ProfileEdge2 {
                    curve: Curve2d::Circle {
                        center: p(4.0, 4.0),
                        radius: 2.0,
                    },
                    tag: Tag(5),
                }],
            };
            PlanarProfile {
                plane: Plane::XY,
                outer,
                holes: vec![hole],
            }
        })
        .collect()
}

fn polygon() -> (Vec<Point3>, Vec<Vec<Point3>>) {
    let outer = (0..1000)
        .map(|i| {
            let t = TAU * f64::from(i) / 1000.0;
            let r = 100.0 + 5.0 * (7.0 * t).sin();
            Point3::new(r * t.cos(), r * t.sin(), 0.0)
        })
        .collect();
    let holes = (0..10)
        .map(|h| {
            let c = TAU * f64::from(h) / 10.0;
            (0..20)
                .map(|i| {
                    let t = TAU * f64::from(i) / 20.0;
                    Point3::new(
                        60.0 * c.cos() + 8.0 * t.cos(),
                        60.0 * c.sin() + 8.0 * t.sin(),
                        0.0,
                    )
                })
                .collect()
        })
        .collect();
    (outer, holes)
}

fn bench(c: &mut Criterion) {
    let kernel = kernel();
    let tol = Tolerance::DEFAULT;
    let profiles = sketch();
    c.bench_function("budget/brep-planar-faces-50", |b| {
        b.iter(|| {
            kernel
                .make_planar_faces(black_box(&profiles), &tol)
                .unwrap()
        });
    });
    let (outer, holes) = polygon();
    c.bench_function("budget/brep-planar-polygon-1000", |b| {
        b.iter(|| {
            kernel
                .make_planar_polygon(black_box(&outer), black_box(&holes), &tol)
                .unwrap()
        });
    });
}

criterion_group! {
    name = benches;
    config = Criterion::default().sample_size(20);
    targets = bench
}
criterion_main!(benches);
