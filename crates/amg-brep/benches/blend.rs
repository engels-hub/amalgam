//! BREP-06: fillet and chamfer costs (S02 has no budget for them; reported in the Refinement
//! log). A cube edge, all 12 cube edges, a chamfer, the 20-case corpus, `fillet_max_radius`
//! (20 bisection steps) and one edge of the 1030-face perforated plate, where ChFi3d's rebuild of
//! every face dominates. Validation is off, as for a release build's default policy.
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "benchmark code")]

#[path = "../tests/common/blend_corpus.rs"]
mod corpus;
#[path = "../tests/common/parts.rs"]
mod parts;

use std::hint::black_box;

use amg_brep::{
    BrepKernel, ChamferSpec, EdgeId, FilletOpts, FilletSpec, JobCtx, KernelConfig, OcctKernel,
    Point3, SubKind, Validate,
};
use criterion::{Criterion, criterion_group, criterion_main};

fn bench(c: &mut Criterion) {
    let k = OcctKernel::new(KernelConfig {
        validate: Validate::Off,
        ..KernelConfig::default()
    });
    let ctx = JobCtx::detached();
    let cube = corpus::cube(&k);
    let edge = corpus::edge_at(&k, &cube, Point3::new(corpus::A / 2.0, 0.0, 0.0));
    let opts = FilletOpts::default();
    let mut group = c.benchmark_group("brep-blend");
    group.bench_function("fillet-cube-edge", |b| {
        b.iter(|| {
            black_box(
                k.fillet(&cube, &[(edge, FilletSpec::Constant(2.0))], &opts, &ctx)
                    .unwrap(),
            )
        });
    });
    let all: Vec<(EdgeId, FilletSpec)> = cube
        .edges()
        .map(|e| (e, FilletSpec::Constant(2.0)))
        .collect();
    group.bench_function("fillet-cube-12-edges", |b| {
        b.iter(|| black_box(k.fillet(&cube, &all, &opts, &ctx).unwrap()));
    });
    group.bench_function("chamfer-cube-edge", |b| {
        b.iter(|| {
            black_box(
                k.chamfer(&cube, &[(edge, ChamferSpec::Equal(2.0))], &ctx)
                    .unwrap(),
            )
        });
    });
    let cases = corpus::corpus(&k);
    group.bench_function("corpus-21", |b| {
        b.iter(|| {
            for case in &cases {
                black_box(case.run(&k).unwrap());
            }
        });
    });
    group.bench_function("max-radius-cube-edge-20", |b| {
        b.iter(|| black_box(k.fillet_max_radius(&cube, &[edge], 12.0, 20, &ctx).unwrap()));
    });
    group.sample_size(10);
    let plate = parts::plate(&k, 32, Point3::ZERO);
    assert_eq!(plate.count(SubKind::Face), 1030);
    let rim = plate
        .edges()
        .find(|&e| {
            plate.edge_faces(e).len() == 2
                && plate
                    .curve(e)
                    .is_some_and(|(c, _, _)| matches!(c, amg_brep::geom::Curve3::Line { .. }))
        })
        .unwrap();
    group.bench_function("fillet-plate-1030-one-edge", |b| {
        b.iter(|| {
            black_box(
                k.fillet(&plate, &[(rim, FilletSpec::Constant(0.5))], &opts, &ctx)
                    .unwrap(),
            )
        });
    });
    group.finish();
}

criterion_group! {
    name = benches;
    config = Criterion::default().sample_size(20);
    targets = bench
}
criterion_main!(benches);
