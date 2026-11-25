//! BREP-07: shell, offset and thicken costs (S02 has no budget for them; reported in the
//! Refinement log). A shelled box (one face removed), a hollow box with arc joins, offsets of a
//! box and a sphere, the thickened B-spline face, the 20-case corpus and an offset of the
//! 1030-face perforated plate. Validation is off, as for a release build's default policy.
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "benchmark code")]

#[path = "../tests/common/offset_corpus.rs"]
mod corpus;
#[path = "../tests/common/parts.rs"]
mod parts;

use std::hint::black_box;

use amg_brep::{BrepKernel, JobCtx, KernelConfig, OcctKernel, OffsetJoin, Point3, Validate};
use criterion::{Criterion, criterion_group, criterion_main};

fn bench(c: &mut Criterion) {
    let k = OcctKernel::new(KernelConfig {
        validate: Validate::Off,
        ..KernelConfig::default()
    });
    let ctx = JobCtx::detached();
    let block = corpus::block(&k);
    let top = corpus::top(&k, &block);
    let mut group = c.benchmark_group("brep-offset");
    let inward = corpus::shell_opts(-1.0, OffsetJoin::Arc);
    group.bench_function("shell-box-top", |b| {
        b.iter(|| black_box(k.shell(&block, &[top], &inward, &ctx).unwrap()));
    });
    let outward = corpus::shell_opts(1.0, OffsetJoin::Arc);
    group.bench_function("shell-box-hollow-arc", |b| {
        b.iter(|| black_box(k.shell(&block, &[], &outward, &ctx).unwrap()));
    });
    group.bench_function("offset-box-arc", |b| {
        b.iter(|| black_box(k.offset(&block, 1.0, OffsetJoin::Arc, 0.0, &ctx).unwrap()));
    });
    let sphere = corpus::sphere(&k);
    group.bench_function("offset-sphere", |b| {
        b.iter(|| black_box(k.offset(&sphere, -2.0, OffsetJoin::Arc, 0.0, &ctx).unwrap()));
    });
    let bspline = corpus::bspline_face(&k);
    group.bench_function("thicken-bspline-face", |b| {
        b.iter(|| black_box(k.thicken(&bspline, 1.0, OffsetJoin::Arc, &ctx).unwrap()));
    });
    let cases = corpus::corpus(&k);
    group.bench_function("corpus-20", |b| {
        b.iter(|| {
            for case in &cases {
                black_box(case.run(&k).unwrap());
            }
        });
    });
    group.sample_size(10);
    let plate = parts::plate(&k, 32, Point3::ZERO);
    group.bench_function("offset-plate-1030", |b| {
        b.iter(|| {
            black_box(
                k.offset(&plate, 0.1, OffsetJoin::Intersection, 0.0, &ctx)
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
