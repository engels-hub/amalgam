//! S20 KH-17 performance target "strategy chain on success (direct accepted) ≤ raw op +
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

criterion_group!(benches, bench);
criterion_main!(benches);
