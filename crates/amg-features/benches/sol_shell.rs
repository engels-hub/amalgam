//! SOL-06 Done-when 8: `budget/sol-shell-preview`, a shell (top removed, 0.3 mm inward) of a
//! 100-face body in `ComputeMode::Preview` (budget: median < 100 ms, S04 §6), plus the same
//! compute in full mode and the bare `BrepKernel::shell` call for the record.
//!
//! The body is the prism of a 98-vertex star (radii 50 / 46 mm alternating, height 20 mm):
//! 98 side faces and 2 caps. Before criterion runs, a fixed 25-run sample prints the medians;
//! with `AMG_STRICT_TIMING=1` the preview median must be under 100 ms. Needs the `occt`
//! feature.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::print_stdout,
    missing_docs,
    reason = "bench"
)]

use std::sync::OnceLock;
use std::time::{Duration, Instant};

use amg_brep::{BrepKernel, OffsetJoin, ShellOpts, SubKind, SubShape};
use amg_core::Point2;
use amg_doc::naming::{default_kernel, topo_ref_at};
use amg_doc::stub::ProfileKey;
use amg_doc::{
    BodyId, Command, ComputeMode, CreatedId, Document, EntityRef, Feature, FeatureBox, FeatureId,
    Health, RecomputeCtx, SketchId, TopoKind,
};
use amg_features::sketch::{PlaneInput, SketchFeature};
use amg_features::solid::{Extrude, OpTarget, Shell};
use amg_jobs::{JobCtx, JobSystem, JobSystemConfig};
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

/// The 98-vertex star prism (100 faces), the shell of its top, and the top's face index.
fn setup() -> (Document, Shell, amg_brep::Shape, u32) {
    let mut doc = Document::new("sol-shell-preview");
    let mut sk = SketchFeature::new(PlaneInput::OriginXY);
    sk.edit_sketch(|s| {
        let pts: Vec<_> = (0..98)
            .map(|i| {
                let a = f64::from(i) * std::f64::consts::TAU / 98.0;
                let r = if i % 2 == 0 { 50.0 } else { 46.0 };
                s.add_point(Point2::new(r * a.cos(), r * a.sin()))
            })
            .collect::<Result<_, _>>()?;
        for i in 0..98 {
            s.add_line_between(pts[i], pts[(i + 1) % 98])?;
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
    let s = s.brep().unwrap().clone();
    assert_eq!(s.count(SubKind::Face), 100);
    let k = default_kernel();
    let top = s
        .faces()
        .find(|f| {
            let g = k
                .sub_geom(
                    &s,
                    SubShape {
                        kind: SubKind::Face,
                        index: f.0,
                    },
                )
                .unwrap();
            (g.centroid.z - 20.0).abs() < 1e-9
        })
        .unwrap()
        .0;
    let mut r = topo_ref_at(&**doc.evaluated(), k, b, TopoKind::Face, top).unwrap();
    r.key = 1;
    (doc, Shell::inside(vec![r], "0.3 mm"), s, top)
}

fn profiles_of(doc: &Document, sk: FeatureId) -> Vec<EntityRef> {
    doc.evaluated()
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
        .collect()
}

/// A 120 × 120 × 10 plate with 94 through holes of r 2 (100 faces), the shell of its top
/// (0.3 mm inward) and the top's face index.
fn setup_plate() -> (Document, Shell, amg_brep::Shape, u32) {
    let mut doc = Document::new("sol-shell-plate");
    let mut sk = SketchFeature::new(PlaneInput::OriginXY);
    sk.edit_sketch(|s| {
        amg_sketch::build::rectangle_2pt(s, Point2::new(0.0, 0.0), Point2::new(120.0, 120.0))?;
        Ok(())
    })
    .unwrap();
    let sk = add(&mut doc, FeatureBox::new(sk));
    let pr = profiles_of(&doc, sk);
    let plate = add(
        &mut doc,
        FeatureBox::new(Extrude::distance(pr, "10 mm", OpTarget::default())),
    );
    let b = BodyId {
        origin: plate,
        slot: 0,
    };
    let mut holes = SketchFeature::new(PlaneInput::OriginXY);
    holes
        .edit_sketch(|s| {
            let mut n = 0;
            for i in 0..10 {
                for j in 0..10 {
                    if n == 94 {
                        break;
                    }
                    s.add_circle(
                        Point2::new(15.0 + 10.0 * f64::from(i), 15.0 + 10.0 * f64::from(j)),
                        2.0,
                    )?;
                    n += 1;
                }
            }
            Ok(())
        })
        .unwrap();
    let holes = add(&mut doc, FeatureBox::new(holes));
    let pr = profiles_of(&doc, holes);
    let cut = add(
        &mut doc,
        FeatureBox::new(Extrude::distance(
            pr,
            "10 mm",
            OpTarget {
                op: amg_features::solid::BodyOp::Cut,
                participants: vec![b],
            },
        )),
    );
    assert_eq!(doc.evaluated().node(cut).unwrap().health, Health::Ok);
    let amg_doc::BodyGeom::Solid(s) = &doc.evaluated().body(b).unwrap().geom else {
        panic!("a solid");
    };
    let s = s.brep().unwrap().clone();
    let k = default_kernel();
    let top = s
        .faces()
        .find(|f| {
            let g = k
                .sub_geom(
                    &s,
                    SubShape {
                        kind: SubKind::Face,
                        index: f.0,
                    },
                )
                .unwrap();
            (g.centroid.z - 10.0).abs() < 1e-9
        })
        .unwrap()
        .0;
    println!("plate faces: {}", s.count(SubKind::Face));
    let mut r = topo_ref_at(&**doc.evaluated(), k, b, TopoKind::Face, top).unwrap();
    r.key = 1;
    (doc, Shell::inside(vec![r], "0.3 mm"), s, top)
}

fn run(doc: &Document, f: &Shell, mode: ComputeMode) {
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

fn median_of(mut f: impl FnMut()) -> Duration {
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

fn bench(c: &mut Criterion) {
    let (doc, f, shape, top) = setup();
    let preview = median_of(|| run(&doc, &f, ComputeMode::Preview));
    let full = median_of(|| run(&doc, &f, ComputeMode::Full));
    let opts = ShellOpts {
        thickness: -0.3,
        join: OffsetJoin::Intersection,
        per_face: Vec::new(),
        tol: 0.0,
    };
    let k = default_kernel();
    let raw = median_of(|| {
        k.shell(&shape, &[amg_brep::FaceId(top)], &opts, &JobCtx::detached())
            .unwrap();
    });
    if std::env::var("AMG_SOL06_PROBE").is_ok() {
        for (name, join, t, rm) in [
            ("arc", OffsetJoin::Arc, -0.3, true),
            (
                "intersection, no removed face",
                OffsetJoin::Intersection,
                -0.3,
                false,
            ),
            ("intersection, outward", OffsetJoin::Intersection, 0.3, true),
        ] {
            let o = ShellOpts {
                thickness: t,
                join,
                per_face: Vec::new(),
                tol: 0.0,
            };
            let faces: Vec<amg_brep::FaceId> = if rm {
                vec![amg_brep::FaceId(top)]
            } else {
                Vec::new()
            };
            let m = median_of(|| {
                k.shell(&shape, &faces, &o, &JobCtx::detached()).unwrap();
            });
            println!("probe {name}: {:.2} ms", m.as_secs_f64() * 1e3);
        }
        let off = median_of(|| {
            k.offset(
                &shape,
                -0.3,
                OffsetJoin::Intersection,
                0.0,
                &JobCtx::detached(),
            )
            .unwrap();
        });
        println!("probe offset: {:.2} ms", off.as_secs_f64() * 1e3);
        let (pdoc, pf, _, _) = setup_plate();
        let pm = median_of(|| run(&pdoc, &pf, ComputeMode::Preview));
        println!("probe plate preview: {:.2} ms", pm.as_secs_f64() * 1e3);
    }
    println!(
        "sol-shell-preview: preview median {:.2} ms, full median {:.2} ms, bare kernel shell {:.2} ms (25 runs, budget < 100 ms)",
        preview.as_secs_f64() * 1e3,
        full.as_secs_f64() * 1e3,
        raw.as_secs_f64() * 1e3
    );
    if std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1") {
        assert!(
            preview < Duration::from_millis(100),
            "preview median {preview:?} over the 100 ms budget"
        );
    }
    let mut g = c.benchmark_group("budget");
    g.sample_size(20);
    g.bench_function("sol-shell-preview", |b| {
        b.iter(|| run(&doc, &f, ComputeMode::Preview));
    });
    g.bench_function("sol-shell-full", |b| {
        b.iter(|| run(&doc, &f, ComputeMode::Full));
    });
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
