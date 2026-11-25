//! IO-09: tessellated export of solid bodies (S12 §9, Done-when 1-4). Needs the `occt`
//! feature; `io_solid_without_kernel.rs` covers the default build.
//!
//! - Done-when 1: 11 golden solids of the S02 corpus at Low, Medium and High: the measured
//!   Hausdorff distance to a much finer reference is within the preset's surface deviation.
//! - Done-when 2: every closed solid at every preset passes `check_export` and the 3MF writer's
//!   validity gate, reads back strictly, and passes `validate_3mf.py` when `AMG_3MF_PYTHON`
//!   names a Python with `lib3mf` and `xmlschema` (required with `AMG_REQUIRE_3MF_VALIDATOR=1`).
//! - Done-when 3: a second export of an unchanged body runs no tessellation (counters).
//! - Done-when 4: a 1000-face part to a 3MF file at Medium; the 300 ms bound is asserted under
//!   `AMG_STRICT_TIMING=1` (release).
#![cfg(feature = "occt")]
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::print_stdout,
    reason = "tests"
)]

mod io_tess_common;

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

use amg_core::{Transform, Vec3};
use amg_doc::{Command, Document, FeatureBox};
use amg_features::io::{
    ExportPrep, ExportRefinement, ExportSelection, ImportPrepError, TessCache, scene_from_document,
    scene_from_document_with_report,
};
use amg_io::{ExportOptions, Format, ImportOptions, Scene, Sink, Source, ValidityPolicy};
use amg_jobs::{CancelToken, JobCtx};
use amg_testkit::{TriSoup, hausdorff, metrics};
use io_tess_common::{
    Io09Solid, add_solid, brep_text, document, filleted_boxes_988, fixture, golden_solids,
    recompute, scalloped_wheel_1000, star_wheel_1000,
};

const PRESETS: [(&str, ExportRefinement); 3] = [
    ("low", ExportRefinement::Low),
    ("medium", ExportRefinement::Medium),
    ("high", ExportRefinement::High),
];

/// The "High+" reference: ten times finer than High in distance, 2° in angle.
const REFERENCE: ExportRefinement = ExportRefinement::Custom {
    surface_deviation_mm: 0.0005,
    normal_deviation_rad: 2.0 * std::f64::consts::PI / 180.0,
};

/// Surface samples per mesh of the Hausdorff measurement.
const SAMPLES: usize = 100_000;

fn ctx() -> JobCtx {
    JobCtx::detached()
}

fn cache() -> Arc<TessCache> {
    Arc::new(TessCache::new(1 << 30))
}

fn prep(refinement: ExportRefinement, cache: &Arc<TessCache>) -> ExportPrep {
    ExportPrep {
        refinement: Some(refinement),
        cache: Some(Arc::clone(cache)),
        ..ExportPrep::default()
    }
}

fn scene(doc: &Document, sel: &ExportSelection, prep: &ExportPrep) -> Scene {
    scene_from_document(doc.design(), doc.evaluated(), sel, prep, &ctx()).unwrap()
}

fn soup(scene: &Scene) -> TriSoup<'_> {
    assert_eq!(scene.objects.len(), 1);
    TriSoup {
        positions: &scene.objects[0].mesh.positions,
        triangles: &scene.objects[0].mesh.tris,
    }
}

#[test]
fn presets_map_onto_the_kernel_quality() {
    for (refinement, linear, degrees) in [
        (ExportRefinement::Low, 0.1, 30.0f64),
        (ExportRefinement::Medium, 0.025, 15.0),
        (ExportRefinement::High, 0.005, 10.0),
        (
            ExportRefinement::Custom {
                surface_deviation_mm: 0.3,
                normal_deviation_rad: 7.0f64.to_radians(),
            },
            0.3,
            7.0,
        ),
    ] {
        let q = refinement.to_tess();
        assert_eq!(q.linear, linear);
        assert_eq!(q.angular, degrees.to_radians());
        assert!(!q.relative);
        assert_eq!(q.min_size, 0.0);
        assert_eq!(q.max_linear_abs, None);
    }
    assert_eq!(ExportRefinement::default(), ExportRefinement::Medium);
}

/// Done-when 1.
#[test]
fn measured_deviation_of_golden_solids_is_within_the_preset() {
    amg_features::io::link();
    let solids = golden_solids();
    if solids.is_empty() {
        return;
    }
    assert!(solids.len() >= 10);
    let mut failures = Vec::new();
    for (name, brep) in solids {
        let (doc, _) = document(&[brep]);
        let reference = scene(
            &doc,
            &ExportSelection::AllVisible,
            &prep(REFERENCE, &cache()),
        );
        for (preset, refinement) in PRESETS {
            let c = cache();
            let (out, report) = scene_from_document_with_report(
                doc.design(),
                doc.evaluated(),
                &ExportSelection::AllVisible,
                &prep(refinement, &c),
                &ctx(),
            )
            .unwrap();
            let measured = hausdorff(soup(&out), soup(&reference), SAMPLES);
            let allowed = refinement.surface_deviation_mm();
            let [entry] = report.bodies.as_slice() else {
                panic!("{name}: one entry, got {report:?}");
            };
            assert_eq!(entry.triangles, out.objects[0].mesh.tris.len() as u64);
            assert_eq!(entry.refinement, refinement);
            assert!(!entry.cached);
            assert!(!entry.exceeds_request(), "{name} {preset}: {entry:?}");
            println!(
                "DW1 {name:18} {preset:6} tris {:7} measured {measured:.6} mm (<= {allowed}), \
                 estimate {:.6} mm, normal {:.2} deg, {} kernel call(s)",
                entry.triangles,
                entry.estimate.surface_mm,
                entry.estimate.normal_rad.to_degrees(),
                c.stats().tessellations
            );
            if measured > allowed {
                failures.push(format!("{name} {preset}: {measured} > {allowed}"));
            }
            // The report's estimate follows the measurement: within the reference's own
            // error and a factor of two either way.
            let slack = REFERENCE.surface_deviation_mm();
            let estimate = entry.estimate.surface_mm;
            if estimate > 2.0 * measured + slack || measured > 2.0 * estimate + slack {
                failures.push(format!(
                    "{name} {preset}: estimate {estimate} against measured {measured}"
                ));
            }
        }
    }
    assert!(failures.is_empty(), "{failures:#?}");
}

fn export_3mf(scene: &Scene, path: &Path) {
    let opts = ExportOptions {
        validity: Some(ValidityPolicy::Require),
        ..ExportOptions::default()
    };
    amg_io::export(scene, Sink::Path(path), Format::ThreeMf, &opts, &ctx()).unwrap();
}

/// A fresh directory under the target's temp folder.
fn scratch(tag: &str) -> PathBuf {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("io09-{tag}"));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn validate_3mf(files: &[PathBuf]) {
    let required = std::env::var("AMG_REQUIRE_3MF_VALIDATOR").is_ok_and(|v| v == "1");
    let Some(python) = std::env::var_os("AMG_3MF_PYTHON") else {
        assert!(
            !required,
            "AMG_REQUIRE_3MF_VALIDATOR=1 but AMG_3MF_PYTHON is not set"
        );
        println!("validate_3mf.py skipped: AMG_3MF_PYTHON is not set");
        return;
    };
    let script = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scripts/io/validate_3mf.py");
    let out = std::process::Command::new(python)
        .arg("-I")
        .arg(&script)
        .args(files)
        .output()
        .unwrap();
    let text = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    println!("{text}");
    assert!(out.status.success(), "validate_3mf.py failed:\n{text}");
    assert_eq!(
        text.matches("OK ").count(),
        files.len(),
        "every file reported OK:\n{text}"
    );
}

/// Done-when 2.
#[test]
fn every_closed_solid_passes_check_export_and_3mf_validation() {
    amg_features::io::link();
    let mut solids = golden_solids();
    if solids.is_empty() {
        return;
    }
    let Some(plate) = fixture("plate_with_holes") else {
        return;
    };
    solids.push(("plate_with_holes", plate));
    solids.push(("star_wheel_1000", brep_text(&star_wheel_1000())));
    solids.push(("scalloped_wheel_1000", brep_text(&scalloped_wheel_1000())));
    let dir = scratch("validate");
    let mut files = Vec::new();
    for (name, brep) in &solids {
        let (doc, _) = document(std::slice::from_ref(brep));
        for (preset, refinement) in PRESETS {
            let out = scene(
                &doc,
                &ExportSelection::AllVisible,
                &prep(refinement, &cache()),
            );
            let check = amg_io::check_export(&out, Format::ThreeMf, &ctx()).unwrap();
            assert!(check.is_valid(), "{name} {preset}: {check:?}");
            let m = metrics(soup(&out));
            assert_eq!(m.boundary_edges, 0, "{name} {preset}");
            assert_eq!(m.non_manifold_edges, 0, "{name} {preset}");
            assert!(m.volume > 0.0, "{name} {preset}");
            let path = dir.join(format!("{name}-{preset}.3mf"));
            export_3mf(&out, &path);
            let mut strict = ImportOptions::default();
            strict.threemf.strict = true;
            let back = amg_io::import(Source::Path(&path), None, &strict, &ctx()).unwrap();
            assert_eq!(back.scene.objects.len(), 1, "{name} {preset}");
            assert_eq!(
                back.scene.objects[0].mesh.tris, out.objects[0].mesh.tris,
                "{name} {preset}"
            );
            println!(
                "DW2 {name:18} {preset:6} tris {:7} watertight, 3MF ok",
                out.objects[0].mesh.tris.len()
            );
            files.push(path);
        }
    }
    validate_3mf(&files);
}

/// The Medium export of a few golden solids, pinned (`tests/golden/io/tessellate-*`).
#[test]
fn golden_medium_tessellations() {
    amg_features::io::link();
    let solids = golden_solids();
    if solids.is_empty() {
        return;
    }
    for (name, brep) in solids {
        let (doc, _) = document(&[brep]);
        let out = scene(
            &doc,
            &ExportSelection::AllVisible,
            &prep(ExportRefinement::Medium, &cache()),
        );
        amg_testkit::check_golden("io", &format!("tess_medium_{name}"), soup(&out)).unwrap();
    }
}

/// Done-when 3.
#[test]
fn a_second_export_of_an_unchanged_body_does_not_tessellate() {
    amg_features::io::link();
    let (Some(a), Some(b)) = (fixture("sphere"), fixture("filleted_box")) else {
        return;
    };
    let (mut doc, bodies) = document(&[a, b]);
    let c = cache();
    let medium = prep(ExportRefinement::Medium, &c);
    let all = ExportSelection::AllVisible;
    let run = |doc: &Document, p: &ExportPrep| {
        scene_from_document_with_report(doc.design(), doc.evaluated(), &all, p, &ctx()).unwrap()
    };

    let (first, report) = run(&doc, &medium);
    assert_eq!(c.stats().tessellations, 2);
    assert_eq!(c.stats().hits, 0);
    assert!(report.bodies.iter().all(|b| !b.cached));

    let (second, report) = run(&doc, &medium);
    assert_eq!(c.stats().tessellations, 2, "no tessellation call");
    assert_eq!(c.stats().hits, 2);
    assert!(report.bodies.iter().all(|b| b.cached));
    for (x, y) in first.objects.iter().zip(&second.objects) {
        assert!(Arc::ptr_eq(&x.mesh, &y.mesh));
    }

    // An unrelated feature and a recompute: the old bodies still hit, the new one is meshed.
    let new = add_solid(&mut doc, fixture("cylinder").unwrap());
    recompute(&mut doc);
    run(&doc, &medium);
    assert_eq!(c.stats().tessellations, 3);
    assert_eq!(c.stats().hits, 4);

    // Changing a body's feature gives a new shape: only that body is meshed again.
    let box_text = fixture("box").unwrap();
    doc.apply(Command::EditFeature {
        id: new.origin,
        feature: FeatureBox::new(Io09Solid { brep: box_text }),
    })
    .unwrap();
    recompute(&mut doc);
    let (_, report) = run(&doc, &medium);
    assert_eq!(c.stats().tessellations, 4);
    assert_eq!(c.stats().hits, 6);
    assert_eq!(
        report.bodies.iter().filter(|b| !b.cached).count(),
        1,
        "{report:?}"
    );

    // Another quality is another key; a per-body override only re-meshes that body.
    let mut high_one = medium.clone();
    high_one
        .per_body_refinement
        .insert(bodies[1], ExportRefinement::High);
    let (_, report) = run(&doc, &high_one);
    assert_eq!(c.stats().tessellations, 5);
    assert_eq!(report.bodies[0].refinement, ExportRefinement::Medium);
    assert_eq!(report.bodies[1].refinement, ExportRefinement::High);
    run(&doc, &high_one);
    run(&doc, &medium);
    assert_eq!(c.stats().tessellations, 5);

    // A cleared cache meshes again.
    c.clear();
    run(&doc, &medium);
    assert_eq!(c.stats().tessellations, 8);
}

#[test]
fn a_cache_too_small_for_a_result_keeps_nothing() {
    amg_features::io::link();
    let Some(text) = fixture("box") else { return };
    let (doc, _) = document(&[text]);
    let c = Arc::new(TessCache::new(16));
    for n in 1..=2 {
        scene(
            &doc,
            &ExportSelection::AllVisible,
            &prep(ExportRefinement::Low, &c),
        );
        assert_eq!(c.stats().tessellations, n);
        assert_eq!(c.stats().entries, 0);
    }
}

#[test]
fn placements_go_on_the_roots_and_the_mesh_stays_in_component_space() {
    amg_features::io::link();
    let Some(text) = fixture("box") else { return };
    let mut doc = Document::new("io09-placed");
    let root = doc.design().root();
    let at = Transform::from_translation(Vec3::new(100.0, 0.0, 0.0));
    let out = doc
        .apply(Command::NewComponent {
            parent: root,
            name: "C".to_owned(),
            transform: at,
        })
        .unwrap();
    let component = out
        .created
        .iter()
        .find_map(|c| match c {
            amg_doc::CreatedId::Component(c) => Some(*c),
            _ => None,
        })
        .unwrap();
    io_tess_common::add_feature(
        &mut doc,
        FeatureBox::new(Io09Solid { brep: text.clone() }),
        component,
    );
    recompute(&mut doc);
    let s = scene(
        &doc,
        &ExportSelection::AllVisible,
        &prep(ExportRefinement::Medium, &cache()),
    );
    let bbox = amg_brep::Shape::read_text(text.as_bytes()).unwrap().bbox();
    assert_eq!(s.roots.len(), 1);
    assert_eq!(s.roots[0].transform, at);
    let mesh_box = s.objects[0].mesh.aabb();
    assert!((mesh_box.min - bbox.min).length() < 1e-6);
    assert!((s.world_aabb().min - (bbox.min + at.translation)).length() < 1e-6);
}

#[test]
fn missing_and_invalid_refinements_and_open_surfaces_fail_loudly() {
    amg_features::io::link();
    let (Some(boxed), Some(patch)) = (fixture("box"), fixture("bspline_patch")) else {
        return;
    };
    let (doc, bodies) = document(&[boxed, patch]);
    let all = ExportSelection::AllVisible;
    let run = |p: &ExportPrep| scene_from_document(doc.design(), doc.evaluated(), &all, p, &ctx());

    assert!(matches!(
        run(&ExportPrep::default()),
        Err(ImportPrepError::NeedsTessellation(2))
    ));
    let mut one = ExportPrep::default();
    one.per_body_refinement
        .insert(bodies[0], ExportRefinement::Low);
    assert!(matches!(
        run(&one),
        Err(ImportPrepError::NeedsTessellation(1))
    ));

    let bad = ExportRefinement::Custom {
        surface_deviation_mm: 0.0,
        normal_deviation_rad: 0.2,
    };
    let err = run(&prep(bad, &cache())).unwrap_err();
    assert!(
        matches!(err, ImportPrepError::Tessellation { body, .. } if body == bodies[0]),
        "{err:?}"
    );

    // An open surface body exports, and the 3MF gate reports it.
    let s = run(&prep(ExportRefinement::Medium, &cache())).unwrap();
    let check = amg_io::check_export(&s, Format::ThreeMf, &ctx()).unwrap();
    assert!(!check.is_valid());
    assert!(check.objects.iter().any(|o| o.boundary_edges > 0));
}

#[test]
fn a_cancelled_export_says_cancelled() {
    amg_features::io::link();
    let Some(text) = fixture("torus") else { return };
    let (doc, _) = document(&[text]);
    let token = CancelToken::new();
    token.cancel();
    let cancelled = JobCtx::with_token(token);
    let err = scene_from_document(
        doc.design(),
        doc.evaluated(),
        &ExportSelection::AllVisible,
        &prep(ExportRefinement::Medium, &cache()),
        &cancelled,
    )
    .unwrap_err();
    assert!(amg_core::Diagnostic::is_cancelled(&err), "{err:?}");
}

/// End to end: a cold cache, `scene_from_document`, the validity check and the 3MF file.
fn export_time(doc: &Document, path: &Path) -> (Duration, Duration, usize) {
    let start = Instant::now();
    let out = scene(
        doc,
        &ExportSelection::AllVisible,
        &prep(ExportRefinement::Medium, &cache()),
    );
    let meshed = start.elapsed();
    export_3mf(&out, path);
    let tris = out.objects.iter().map(|o| o.mesh.tris.len()).sum();
    (meshed, start.elapsed(), tris)
}

/// Done-when 4. The bound is asserted on the ordinary parts: the 1 000-face star wheel and
/// 988 faces of filleted boxes. The other two have large planar faces with many curved or
/// hole boundaries, where OCCT's mesher takes almost all of the time (S02 BREP-18): their
/// times are printed, not bounded.
#[test]
fn a_1000_face_part_exports_to_3mf_medium_in_time() {
    amg_features::io::link();
    let dir = scratch("timing");
    let (Some(boxes), Some(plate)) = (filleted_boxes_988(), fixture("plate_with_holes")) else {
        return;
    };
    let one = |shape: &amg_brep::Shape| vec![brep_text(shape)];
    for (name, breps, faces, runs, bounded) in [
        ("star_wheel_1000", one(&star_wheel_1000()), 1000, 5, true),
        ("filleted_boxes_988", boxes, 988, 5, true),
        (
            "scalloped_wheel_1000",
            one(&scalloped_wheel_1000()),
            1000,
            3,
            false,
        ),
        ("plate_with_holes", vec![plate], 1030, 1, false),
    ] {
        let count: usize = breps
            .iter()
            .map(|b| {
                amg_brep::Shape::read_text(b.as_bytes())
                    .unwrap()
                    .faces()
                    .len()
            })
            .sum();
        assert_eq!(count, faces, "{name}");
        let (doc, _) = document(&breps);
        let mut times: Vec<_> = (0..runs)
            .map(|_| export_time(&doc, &dir.join("part.3mf")))
            .collect();
        times.sort_by_key(|t| t.1);
        let (meshed, total, tris) = times[times.len() / 2];
        println!(
            "DW4 {name} medium: {tris} tris, tessellation + scene {meshed:?}, end to end              {total:?} (median of {runs})"
        );
        if bounded && std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1") {
            assert!(total < Duration::from_millis(300), "{name}: {total:?}");
        }
    }
}
