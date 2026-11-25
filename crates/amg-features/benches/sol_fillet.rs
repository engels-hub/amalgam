//! SOL-05 Done-when 5: `budget/sol-fillet-preview`, a 10-edge constant fillet on a 200-face
//! body in `ComputeMode::Preview` (budget: median < 100 ms, S04 §6), plus the same compute in
//! full mode for the record.
//!
//! The body is the prism of a 198-vertex star (200 faces; a regular 198-gon's 1.8° edges are
//! too flat for OCCT to blend); the fillet takes every 19th vertical edge (10 edges, none
//! adjacent), radius 0.3 mm. Before criterion runs, a fixed 25-run sample prints the
//! medians; with `AMG_STRICT_TIMING=1` the preview median must be under 100 ms. Needs the
//! `occt` feature.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::print_stdout,
    missing_docs,
    reason = "bench"
)]

use std::sync::OnceLock;
use std::time::{Duration, Instant};

use amg_brep::{BrepKernel, SubKind};
use amg_core::Point2;
use amg_doc::naming::{default_kernel, topo_ref_at};
use amg_doc::stub::ProfileKey;
use amg_doc::{
    BodyId, Command, ComputeMode, CreatedId, Document, EntityRef, Feature, FeatureBox, FeatureId,
    Health, RecomputeCtx, SketchId, TopoKind,
};
use amg_features::sketch::{PlaneInput, SketchFeature};
use amg_features::solid::{Extrude, Fillet, OpTarget};
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

/// A 198-vertex star prism (200 faces) and a fillet of 10 of its vertical edges.
fn setup() -> (Document, Fillet) {
    let mut doc = Document::new("sol-fillet-preview");
    let mut sk = SketchFeature::new(PlaneInput::OriginXY);
    sk.edit_sketch(|s| {
        // A 198-vertex star (radii 50 / 46 alternating): every vertical edge is sharp.
        let pts: Vec<_> = (0..198)
            .map(|i| {
                let a = f64::from(i) * std::f64::consts::TAU / 198.0;
                let r = if i % 2 == 0 { 50.0 } else { 46.0 };
                s.add_point(Point2::new(r * a.cos(), r * a.sin()))
            })
            .collect::<Result<_, _>>()?;
        for i in 0..198 {
            s.add_line_between(pts[i], pts[(i + 1) % 198])?;
        }
        Ok(())
    })
    .unwrap();
    let sk = add(&mut doc, FeatureBox::new(sk));
    let profiles: Vec<EntityRef> = doc
        .evaluated()
        .sketch(SketchId(sk))
        .unwrap()
        .profiles
        .profiles
        .iter()
        .map(|p| EntityRef::Profile {
            sketch: SketchId(sk),
            profile: ProfileKey(p.id.0),
            fingerprint: None,
        })
        .collect();
    let prism = add(
        &mut doc,
        FeatureBox::new(Extrude::distance(profiles, "20 mm", OpTarget::default())),
    );
    assert_eq!(doc.evaluated().node(prism).unwrap().health, Health::Ok);
    let b = BodyId {
        origin: prism,
        slot: 0,
    };
    let amg_doc::BodyGeom::Solid(s) = &doc.evaluated().body(b).unwrap().geom else {
        panic!("a solid");
    };
    let s = s.brep().unwrap();
    assert_eq!(s.count(SubKind::Face), 200);
    let k = default_kernel();
    let mut vertical: Vec<u32> = s
        .edges()
        .filter(|e| {
            let ends = k.edge_ends(s, *e).unwrap();
            (ends.p0.z - ends.p1.z).abs() > 10.0
        })
        .map(|e| e.0)
        .collect();
    vertical.sort_unstable();
    assert_eq!(vertical.len(), 198);
    let edges = (0..10)
        .map(|i| {
            let mut r =
                topo_ref_at(&**doc.evaluated(), k, b, TopoKind::Edge, vertical[i * 19]).unwrap();
            r.key = u32::try_from(i + 1).unwrap();
            r
        })
        .collect();
    (doc, Fillet::constant(edges, "0.3 mm"))
}

fn run(doc: &Document, f: &Fillet, mode: ComputeMode) {
    let mut ctx = RecomputeCtx::new(
        doc.design(),
        doc.evaluated(),
        FeatureId(u64::MAX / 2),
        doc.design().root(),
        mode,
    );
    let out = f.compute(&mut ctx).unwrap();
    assert_eq!(out.bodies.len(), 1);
}

fn median(doc: &Document, f: &Fillet, mode: ComputeMode) -> Duration {
    run(doc, f, mode);
    let mut t: Vec<Duration> = (0..25)
        .map(|_| {
            let s = Instant::now();
            run(doc, f, mode);
            s.elapsed()
        })
        .collect();
    t.sort();
    t[t.len() / 2]
}

fn bench(c: &mut Criterion) {
    let (doc, f) = setup();
    let preview = median(&doc, &f, ComputeMode::Preview);
    let full = median(&doc, &f, ComputeMode::Full);
    println!(
        "sol-fillet-preview: preview median {:.2} ms, full median {:.2} ms (25 runs, budget < 100 ms)",
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
    g.bench_function("sol-fillet-preview", |b| {
        b.iter(|| run(&doc, &f, ComputeMode::Preview));
    });
    g.bench_function("sol-fillet-full", |b| {
        b.iter(|| run(&doc, &f, ComputeMode::Full));
    });
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
