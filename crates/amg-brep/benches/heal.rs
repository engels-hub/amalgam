//! BREP-10: heal, unify, sew, make_solid, make_compound and the full check (S02 has no budget
//! for them; reported in the Refinement log). Validation is off, as for a release build's default
//! policy.
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "benchmark code")]

#[path = "../tests/common/heal_corpus.rs"]
mod corpus;

use std::hint::black_box;

use amg_brep::{
    BrepKernel, CheckOpts, HealOpts, JobCtx, KernelConfig, OcctKernel, SewOpts, Transform,
    UnifyOpts, Validate, Vec3,
};
use criterion::{Criterion, criterion_group, criterion_main};

fn bench(c: &mut Criterion) {
    let k = OcctKernel::new(KernelConfig {
        validate: Validate::Off,
        ..KernelConfig::default()
    });
    let ctx = JobCtx::detached();
    let mut group = c.benchmark_group("brep-heal");
    if let Some(gaps) = corpus::defect("wire_gaps") {
        group.bench_function("heal-wire-gaps", |b| {
            b.iter(|| black_box(k.heal(&gaps, &corpus::heal_opts(), &ctx).unwrap()));
        });
    }
    if let Some(filleted) = corpus::fixture("filleted_box") {
        group.bench_function("heal-filleted-box", |b| {
            b.iter(|| black_box(k.heal(&filleted, &HealOpts::default(), &ctx).unwrap()));
        });
        group.bench_function("check-full-filleted-box", |b| {
            let full = CheckOpts {
                self_interference: true,
            };
            b.iter(|| black_box(k.check(&filleted, &full)));
        });
    }
    let (_, _, fused) = corpus::coplanar_fuse(&k);
    group.bench_function("unify-coplanar-fuse", |b| {
        b.iter(|| {
            black_box(
                k.unify_same_domain(&fused.shape, &UnifyOpts::default())
                    .unwrap(),
            )
        });
    });
    let cube = corpus::gapped_cube(&k, 10.0, 1e-4);
    let loose = SewOpts {
        tol: 1e-3,
        ..SewOpts::default()
    };
    group.bench_function("sew-gapped-cube", |b| {
        b.iter(|| black_box(k.sew(&cube, &loose, &ctx).unwrap()));
    });
    if let Some(plate) = corpus::fixture("plate_with_holes") {
        let faces = corpus::faces_apart(&plate);
        group.sample_size(10);
        group.bench_function("sew-plate-1030-apart", |b| {
            b.iter(|| black_box(k.sew(&faces, &SewOpts::default(), &ctx).unwrap()));
        });
        let shell = k
            .sew(
                &faces,
                &SewOpts {
                    make_solid: false,
                    ..SewOpts::default()
                },
                &ctx,
            )
            .unwrap()
            .shape;
        group.bench_function("make-solid-plate-1030", |b| {
            b.iter(|| black_box(k.make_solid(&shell).unwrap()));
        });
        let moved = plate
            .transformed(&Transform::from_translation(Vec3::new(500.0, 0.0, 0.0)))
            .unwrap();
        group.bench_function("make-compound-two-plates", |b| {
            b.iter(|| black_box(k.make_compound(&[plate.clone(), moved.clone()]).unwrap()));
        });
    }
    group.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
