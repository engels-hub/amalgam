//! S20 KH-17 (booleans) and KH-03 (fillets) performance target "strategy chain on success (direct accepted) ≤ raw op +
//! validation (< 5 % overhead)": the raw boolean, the raw boolean plus the gate's checks
//! (`check` + `validate` with closed-solid and volume checks), and `robust_boolean` whose direct
//! attempt is accepted, on a small blind-hole cut and on the 202-face plate pair of the BREP-04
//! budget. Validation policy off, as in a release build.
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "bench code")]

#[path = "../tests/common/parts.rs"]
mod parts;

use std::hint::black_box;
use std::time::Duration;

use amg_brep::robust::{RetryPolicy, RobustCtx, StrategyId, Validation, robust_boolean, validate};
use amg_brep::{
    BoolOp, BoolOpts, BrepKernel, JobCtx, KernelConfig, OcctKernel, Plane, Point3, Shape, Validate,
    Vec3,
};
use criterion::{Criterion, criterion_group, criterion_main};

fn kernel() -> OcctKernel {
    OcctKernel::new(KernelConfig {
        validate: Validate::Off,
        ..KernelConfig::default()
    })
}

fn raw(k: &OcctKernel, a: &Shape, b: &Shape) -> Shape {
    k.boolean(
        BoolOp::Cut,
        std::slice::from_ref(a),
        std::slice::from_ref(b),
        &BoolOpts::default(),
        &JobCtx::detached(),
    )
    .unwrap()
    .shape
}

fn raw_validated(k: &OcctKernel, a: &Shape, b: &Shape) -> f64 {
    let s = raw(k, a, b);
    let (va, vb) = (a.volume().unwrap(), b.volume().unwrap());
    let gate = Validation {
        require_solid: true,
        volume_bounds: Some(((va - vb).max(0.0) * 0.999, va * 1.001)),
        ..Validation::default()
    };
    validate(k, &s, &gate).unwrap()
}

fn robust(k: &OcctKernel, a: &Shape, b: &Shape, p: &RetryPolicy) -> Shape {
    let job = JobCtx::detached();
    let out = robust_boolean(
        k,
        BoolOp::Cut,
        std::slice::from_ref(a),
        std::slice::from_ref(b),
        &BoolOpts::default(),
        p,
        &RobustCtx::new(&job),
    )
    .unwrap();
    assert_eq!(out.report.accepted, Some(StrategyId::Direct));
    out.result.shape
}

fn bench(c: &mut Criterion) {
    let k = kernel();
    let p = RetryPolicy::recompute();
    let block = k.make_box(&Plane::XY, Vec3::splat(10.0)).unwrap().shape;
    let drill = k
        .make_cylinder(
            &Plane {
                origin: Point3::new(5.0, 5.0, 4.0),
                ..Plane::XY
            },
            2.0,
            10.0,
            std::f64::consts::TAU,
        )
        .unwrap()
        .shape;
    let (plate, tool) = parts::plate_pair(&k, 14);
    for (name, a, b) in [
        ("blind-hole", &block, &drill),
        ("plates-202", &plate, &tool),
    ] {
        let mut g = c.benchmark_group(format!("robust/{name}"));
        g.sample_size(20).measurement_time(Duration::from_secs(5));
        g.bench_function("raw", |x| x.iter(|| black_box(raw(&k, a, b))));
        g.bench_function("raw+gate", |x| {
            x.iter(|| black_box(raw_validated(&k, a, b)))
        });
        g.bench_function("robust-direct", |x| {
            x.iter(|| black_box(robust(&k, a, b, &p)));
        });
        g.finish();
    }
}

/// KH-03: the same target for the fillet chain (`robust::fillet::fillet`, direct accepted) on a
/// 10 mm cube with 4 vertical edges at r = 2 and with all 12 edges at r = 1 (8 corner patches).
fn bench_fillet(c: &mut Criterion) {
    use amg_brep::robust::fillet::fillet;
    use amg_brep::{EdgeId, FilletOpts, FilletSpec};
    let k = kernel();
    let p = RetryPolicy::recompute();
    let cube = k.make_box(&Plane::XY, Vec3::splat(10.0)).unwrap().shape;
    let vertical: Vec<(EdgeId, FilletSpec)> = cube
        .edges()
        .filter(|&e| {
            k.edge_ends(&cube, e)
                .is_ok_and(|x| x.t0.get().z.abs() > 0.99)
        })
        .map(|e| (e, FilletSpec::Constant(2.0)))
        .collect();
    let all: Vec<(EdgeId, FilletSpec)> = cube
        .edges()
        .map(|e| (e, FilletSpec::Constant(1.0)))
        .collect();
    let opts = FilletOpts::default();
    let v0 = cube.volume().unwrap();
    for (name, edges) in [("cube-4-vertical", &vertical), ("cube-12-edges", &all)] {
        let raw_fillet = || {
            k.fillet(&cube, edges, &opts, &JobCtx::detached())
                .unwrap()
                .shape
        };
        let mut g = c.benchmark_group(format!("robust-fillet/{name}"));
        g.sample_size(30).measurement_time(Duration::from_secs(5));
        g.bench_function("raw", |x| x.iter(|| black_box(raw_fillet())));
        g.bench_function("raw+gate", |x| {
            x.iter(|| {
                let s = raw_fillet();
                let gate = Validation {
                    require_solid: true,
                    volume_bounds: Some((0.5 * v0, v0)),
                    ..Validation::default()
                };
                black_box(validate(&k, &s, &gate).unwrap())
            });
        });
        g.bench_function("robust-direct", |x| {
            x.iter(|| {
                let job = JobCtx::detached();
                let out = fillet(&k, &cube, edges, &opts, &p, &RobustCtx::new(&job)).unwrap();
                assert_eq!(out.report.accepted, Some(StrategyId::Direct));
                black_box(out.result.shape)
            });
        });
        g.finish();
    }
}

criterion_group!(benches, bench, bench_fillet);
criterion_main!(benches);
