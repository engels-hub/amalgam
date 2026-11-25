//! BREP-16: draft, defeature, replace face, split faces and offset faces costs (S02 has no
//! budget for them; reported in the Refinement log). The Done-when cases on the 20 × 16 × 12
//! block, the whole corpus, and three ops on the 1030-face perforated plate (one hole removed,
//! the top pushed up 2 mm by `offset_faces`, the top split by a line). Validation is off, as for
//! a release build's default policy.
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "benchmark code")]

#[path = "../tests/common/local_corpus.rs"]
mod corpus;
#[path = "../tests/common/parts.rs"]
mod parts;

use std::hint::black_box;

use amg_brep::geom::{GeomClass, Surface};
use amg_brep::{BrepKernel, DraftOpts, JobCtx, KernelConfig, OcctKernel, Plane, Point3, Validate};
use amg_core::Dir3;
use criterion::{Criterion, criterion_group, criterion_main};

fn bench(c: &mut Criterion) {
    let k = OcctKernel::new(KernelConfig {
        validate: Validate::Off,
        ..KernelConfig::default()
    });
    let ctx = JobCtx::detached();
    let block = corpus::block(&k);
    let top = corpus::top(&k, &block);
    let sides = corpus::sides(&k, &block);
    let mut group = c.benchmark_group("brep-local");
    let draft = DraftOpts {
        pull: Dir3::Z,
        angle: 3f64.to_radians(),
        neutral: Plane::XY,
    };
    group.bench_function("draft-box-sides", |b| {
        b.iter(|| black_box(k.draft(&block, &sides, &draft, &ctx).unwrap()));
    });
    let filleted = corpus::filleted(&k, 3.0);
    let fillet = corpus::faces_where(&k, &filleted, Some(GeomClass::Cylinder), None);
    group.bench_function("defeature-fillet", |b| {
        b.iter(|| black_box(k.defeature(&filleted, &fillet, &ctx).unwrap()));
    });
    let bossed = corpus::bossed(&k);
    let boss = corpus::boss_faces(&k, &bossed);
    group.bench_function("defeature-boss", |b| {
        b.iter(|| black_box(k.defeature(&bossed, &boss, &ctx).unwrap()));
    });
    let up = Surface::Plane(Plane {
        origin: Point3::new(0.0, 0.0, corpus::C + 3.0),
        ..Plane::XY
    });
    group.bench_function("replace-top-up", |b| {
        b.iter(|| black_box(k.replace_face(&block, top, &up, &ctx).unwrap()));
    });
    let down = Surface::Plane(Plane {
        origin: Point3::new(0.0, 0.0, corpus::C - 4.0),
        ..Plane::XY
    });
    group.bench_function("replace-top-down", |b| {
        b.iter(|| black_box(k.replace_face(&block, top, &down, &ctx).unwrap()));
    });
    let line = corpus::line(
        &k,
        corpus::p(5.0, -2.0, corpus::C),
        corpus::p(5.0, corpus::B + 2.0, corpus::C),
    );
    group.bench_function("split-faces-top-line", |b| {
        b.iter(|| black_box(k.split_faces(&block, &[top], &line, &ctx).unwrap()));
    });
    group.bench_function("offset-faces-top", |b| {
        b.iter(|| black_box(k.offset_faces(&block, &[(top, 3.0)], &ctx).unwrap()));
    });
    let cases = corpus::corpus(&k);
    group.bench_function("corpus", |b| {
        b.iter(|| {
            for case in &cases {
                black_box(case.run(&k).unwrap());
            }
        });
    });
    group.sample_size(10);
    let plate = parts::plate(&k, 32, Point3::ZERO);
    let side = parts::PITCH * 32.0;
    let plate_top = corpus::face_at(
        &k,
        &plate,
        Point3::new(side / 2.0, side / 2.0, parts::THICKNESS),
    );
    let hole = corpus::faces_where(&k, &plate, Some(GeomClass::Cylinder), None)[0];
    group.bench_function("defeature-plate-1030-one-hole", |b| {
        b.iter(|| black_box(k.defeature(&plate, &[hole], &ctx).unwrap()));
    });
    group.bench_function("offset-faces-plate-1030-top", |b| {
        b.iter(|| black_box(k.offset_faces(&plate, &[(plate_top, 2.0)], &ctx).unwrap()));
    });
    let across = corpus::line(
        &k,
        Point3::new(0.4, -1.0, parts::THICKNESS),
        Point3::new(0.4, side + 1.0, parts::THICKNESS),
    );
    group.bench_function("split-faces-plate-1030-top", |b| {
        b.iter(|| black_box(k.split_faces(&plate, &[plate_top], &across, &ctx).unwrap()));
    });
    group.finish();
}

criterion_group! {
    name = benches;
    config = Criterion::default().sample_size(20);
    targets = bench
}
criterion_main!(benches);
