//! SOL-08 Done-when 2: `budget/sol-pattern-100`, a full-mode recompute of a 10×10 feature
//! pattern of a Ø3 cut-extrude in a 100×100×5 plate (99 instances, one boolean; budget:
//! < 300 ms, S04 performance targets).
//!
//! Before criterion runs, a fixed 25-run sample prints the median and a breakdown (the OCCT
//! boolean, the unify, the rest: seed tool, transforms, names); with `AMG_STRICT_TIMING=1` the
//! median must be under 300 ms. Needs the `occt` feature.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::print_stdout,
    missing_docs,
    reason = "bench"
)]

#[path = "../tests/solid_common/counting.rs"]
mod counting;

use std::sync::OnceLock;
use std::time::{Duration, Instant};

use amg_brep::{BoolOp, BoolOpts, BrepKernel, JobCtx, KernelConfig, OcctKernel, UnifyOpts};
use amg_core::Point2;
use amg_doc::stub::ProfileKey;
use amg_doc::{
    BodyGeom, BodyId, Command, ComputeMode, CreatedId, Document, EntityRef, Feature, FeatureBox,
    FeatureId, Health, RecomputeCtx, SketchId,
};
use amg_features::construct::{OriginEntity, RefInput};
use amg_features::sketch::{PlaneInput, SketchFeature};
use amg_features::solid::{
    BodyOp, ComputeType, Extent, Extrude, OpTarget, Pattern, PatternDir, PatternKind,
    PatternObjects, Spacing,
};
use amg_jobs::{JobSystem, JobSystemConfig};
use criterion::{Criterion, criterion_group, criterion_main};

fn jobs() -> &'static JobSystem {
    static JOBS: OnceLock<JobSystem> = OnceLock::new();
    JOBS.get_or_init(|| JobSystem::new(JobSystemConfig::default()).unwrap())
}

fn add(doc: &mut Document, f: FeatureBox) -> FeatureId {
    let root = doc.design().root();
    let o = doc
        .apply(Command::AddFeature {
            at: None,
            feature: f,
            context: root,
            name: None,
        })
        .unwrap();
    doc.recompute_blocking(jobs()).unwrap();
    let id = o
        .created
        .iter()
        .find_map(|c| match c {
            CreatedId::Feature(f) => Some(*f),
            _ => None,
        })
        .unwrap();
    assert_eq!(doc.evaluated().node(id).unwrap().health, Health::Ok);
    id
}

fn profile_refs(doc: &Document, s: FeatureId) -> Vec<EntityRef> {
    doc.evaluated()
        .sketch(SketchId(s))
        .unwrap()
        .profiles
        .profiles
        .iter()
        .map(|p| EntityRef::Profile {
            sketch: SketchId(s),
            profile: ProfileKey(p.id.0),
            fingerprint: None,
        })
        .collect()
}

fn ex(s: &str) -> amg_doc::Expr {
    amg_doc::Expr::Inline(amg_doc::ExprSrc(s.to_owned()))
}

fn dir(axis: OriginEntity) -> PatternDir {
    PatternDir {
        dir: RefInput::Origin(axis),
        count: ex("10"),
        spacing: Spacing::Spacing(ex("10 mm")),
        flip: false,
        symmetric: false,
    }
}

/// The plate with its seed hole, the plate body and the pattern (not yet added).
fn setup() -> (Document, BodyId, Pattern) {
    let mut doc = Document::new("sol-pattern-100");
    let mut sk = SketchFeature::new(PlaneInput::OriginXY);
    sk.edit_sketch(|s| {
        amg_sketch::build::rectangle_2pt(s, Point2::new(0.0, 0.0), Point2::new(100.0, 100.0))
    })
    .unwrap();
    let s = add(&mut doc, FeatureBox::new(sk));
    let base = Extrude::distance(profile_refs(&doc, s), "5 mm", OpTarget::default());
    let e = add(&mut doc, FeatureBox::new(base));
    let plate = BodyId { origin: e, slot: 0 };
    let mut hole = SketchFeature::new(PlaneInput::OriginXY);
    hole.edit_sketch(|s| s.add_circle(Point2::new(5.0, 5.0), 1.5).map(|_| ()))
        .unwrap();
    let hs = add(&mut doc, FeatureBox::new(hole));
    let mut cut = Extrude::distance(
        profile_refs(&doc, hs),
        "1 mm",
        OpTarget {
            op: BodyOp::Cut,
            participants: vec![plate],
        },
    );
    cut.side1.extent = Extent::All;
    let h = add(&mut doc, FeatureBox::new(cut));
    let pattern = Pattern {
        objects: PatternObjects::Features(vec![h]),
        kind: PatternKind::Rectangular {
            d1: dir(OriginEntity::XAxis),
            d2: Some(dir(OriginEntity::YAxis)),
        },
        suppressed: Vec::new(),
        compute: ComputeType::Identical,
        target: None,
    };
    (doc, plate, pattern)
}

fn run(doc: &Document, p: &Pattern) {
    let mut ctx = RecomputeCtx::new(
        doc.design(),
        doc.evaluated(),
        FeatureId(u64::MAX / 2),
        doc.design().root(),
        ComputeMode::Full,
    );
    let out = p.compute(&mut ctx).unwrap();
    assert_eq!(out.bodies.len(), 1);
}

fn median(mut f: impl FnMut()) -> Duration {
    f();
    let mut t: Vec<Duration> = (0..25)
        .map(|_| {
            let s = Instant::now();
            f();
            s.elapsed()
        })
        .collect();
    t.sort();
    t[t.len() / 2]
}

fn ms(d: Duration) -> f64 {
    d.as_secs_f64() * 1e3
}

fn bench(c: &mut Criterion) {
    let (doc, plate, pattern) = setup();
    let full = median(|| run(&doc, &pattern));

    // Breakdown: capture the tool compound, then time the boolean and the unify alone.
    let counting = counting::Counting::new(OcctKernel::new(KernelConfig::default()));
    let mut ctx = RecomputeCtx::new(
        doc.design(),
        doc.evaluated(),
        FeatureId(u64::MAX / 2),
        doc.design().root(),
        ComputeMode::Full,
    )
    .with_kernel(&counting);
    pattern.compute(&mut ctx).unwrap();
    assert_eq!(counting.boolean_calls(), 1);
    let tool = counting.tools.lock().unwrap()[0][0].clone();
    let BodyGeom::Solid(s) = &doc.evaluated().body(plate).unwrap().geom else {
        panic!("a solid");
    };
    let plate_shape = s.brep().unwrap().clone();
    let k = amg_doc::naming::default_kernel();
    let job = JobCtx::detached();
    let cut = || {
        k.boolean(
            BoolOp::Cut,
            std::slice::from_ref(&plate_shape),
            std::slice::from_ref(&tool),
            &BoolOpts::default(),
            &job,
        )
        .unwrap()
    };
    let boolean = median(|| {
        cut();
    });
    let cut_shape = cut().shape;
    let unify = median(|| {
        k.unify_same_domain(&cut_shape, &UnifyOpts::default())
            .unwrap();
    });
    println!(
        "sol-pattern-100: full median {:.2} ms (25 runs, budget < 300 ms); OCCT boolean {:.2} ms, \
         unify {:.2} ms, rest (seed tool, 99 transforms, names) {:.2} ms",
        ms(full),
        ms(boolean),
        ms(unify),
        (ms(full) - ms(boolean) - ms(unify)).max(0.0)
    );
    if std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1") {
        assert!(
            full < Duration::from_millis(300),
            "full median {full:?} over the 300 ms budget"
        );
    }
    let mut g = c.benchmark_group("budget");
    g.sample_size(10);
    g.bench_function("sol-pattern-100", |b| {
        b.iter(|| run(&doc, &pattern));
    });
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
