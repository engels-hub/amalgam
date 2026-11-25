//! SOL-01 Done-when 6: `budget/sol-extrude-preview`, an extrude-cut of a 20-edge profile into a
//! 200-face body in `ComputeMode::Preview` (budget: median < 100 ms, S04 §6), plus the same
//! compute in full mode for the record.
//!
//! Before criterion runs, a fixed 25-run sample prints the medians; with `AMG_STRICT_TIMING=1`
//! the preview median must be under 100 ms. Needs the `occt` feature.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::print_stdout,
    missing_docs,
    reason = "bench"
)]

use std::sync::OnceLock;
use std::time::{Duration, Instant};

use amg_core::Point2;
use amg_doc::stub::ProfileKey;
use amg_doc::{
    BodyId, Command, ComputeMode, CreatedId, Document, EntityRef, Feature, FeatureBox, FeatureId,
    Health, RecomputeCtx, SketchId,
};
use amg_features::sketch::{PlaneInput, SketchFeature};
use amg_features::solid::{BodyOp, Extrude, OpTarget};
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
    o.created
        .iter()
        .find_map(|c| match c {
            CreatedId::Feature(f) => Some(*f),
            _ => None,
        })
        .unwrap()
}

/// A sketch with one regular `sides`-gon of circumradius `r` about `center`.
fn polygon(center: Point2, r: f64, sides: u32) -> SketchFeature {
    let mut f = SketchFeature::new(PlaneInput::OriginXY);
    f.edit_sketch(|s| {
        amg_sketch::build::polygon(s, center, Point2::new(center.x + r, center.y), sides, true)
    })
    .unwrap();
    f
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

/// A 198-gon prism (200 faces) and a 20-gon cut through it.
fn setup() -> (Document, Extrude) {
    let mut doc = Document::new("sol-extrude-preview");
    let base = add(&mut doc, FeatureBox::new(polygon(Point2::ZERO, 50.0, 198)));
    let prism = Extrude::distance(profile_refs(&doc, base), "20 mm", OpTarget::default());
    let body = add(&mut doc, FeatureBox::new(prism));
    assert_eq!(doc.evaluated().node(body).unwrap().health, Health::Ok);
    let b = BodyId {
        origin: body,
        slot: 0,
    };
    let amg_doc::BodyGeom::Solid(s) = &doc.evaluated().body(b).unwrap().geom else {
        panic!("a solid");
    };
    assert_eq!(s.brep().unwrap().count(amg_brep::SubKind::Face), 200);
    let tool = add(
        &mut doc,
        FeatureBox::new(polygon(Point2::new(5.0, 3.0), 12.0, 20)),
    );
    let cut = Extrude::distance(
        profile_refs(&doc, tool),
        "30 mm",
        OpTarget {
            op: BodyOp::Cut,
            participants: vec![b],
        },
    );
    (doc, cut)
}

fn run(doc: &Document, cut: &Extrude, mode: ComputeMode) {
    let mut ctx = RecomputeCtx::new(
        doc.design(),
        doc.evaluated(),
        FeatureId(u64::MAX / 2),
        doc.design().root(),
        mode,
    );
    let out = cut.compute(&mut ctx).unwrap();
    assert_eq!(out.bodies.len(), 1);
}

fn median(doc: &Document, cut: &Extrude, mode: ComputeMode) -> Duration {
    run(doc, cut, mode);
    let mut t: Vec<Duration> = (0..25)
        .map(|_| {
            let s = Instant::now();
            run(doc, cut, mode);
            s.elapsed()
        })
        .collect();
    t.sort();
    t[t.len() / 2]
}

fn bench(c: &mut Criterion) {
    let (doc, cut) = setup();
    let preview = median(&doc, &cut, ComputeMode::Preview);
    let full = median(&doc, &cut, ComputeMode::Full);
    println!(
        "sol-extrude-preview: preview median {:.2} ms, full median {:.2} ms (25 runs, budget < 100 ms)",
        preview.as_secs_f64() * 1e3,
        full.as_secs_f64() * 1e3
    );
    if std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1") {
        assert!(
            preview < Duration::from_millis(100),
            "preview median {preview:?} over the 100 ms budget"
        );
    }
    let mut g = c.benchmark_group("budget");
    g.sample_size(20);
    g.bench_function("sol-extrude-preview", |b| {
        b.iter(|| run(&doc, &cut, ComputeMode::Preview));
    });
    g.bench_function("sol-extrude-full", |b| {
        b.iter(|| run(&doc, &cut, ComputeMode::Full));
    });
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
