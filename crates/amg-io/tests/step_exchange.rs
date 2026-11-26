//! IO-06: STEP import/export through `amg_io::brep` (needs `--features occt`).
//!
//! - Done-when 1: the NIST MBE PMI models (fetch-only, `scripts/io/fetch_nist_step.sh` into
//!   `$AMG_TESTDATA_DIR/io/step/nist-mbe-pmi/`) and the checked-in BREP-13 fixtures import with
//!   the component tree, part names and colours of the `ForeignModel` BREP-13 reads, and every
//!   part passes BRepCheck.
//! - Done-when 2: import → export AP242 → import keeps the tree, names, colours, volumes
//!   (1e-9 relative) and bboxes (1e-6 mm).
//! - Done-when 3: the owner's Fusion exports under `$AMG_TESTDATA_DIR/fusion/` round-trip the
//!   same way (skipped when absent).
//! - Done-when 4: a 50 MB STEP imports as a job with monotone progress and cancels within
//!   500 ms (asserted under `AMG_STRICT_TIMING=1`).
#![cfg(feature = "occt")]
#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::print_stdout,
    clippy::print_stderr,
    clippy::float_cmp,
    reason = "test code"
)]

mod step_common;

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use amg_brep::io::{read_step, write_step};
use amg_brep::{
    BrepKernel, ForeignModel, ForeignNode, ImportOpts, KernelConfig, OcctKernel, Plane, Shape,
    Transform, Vec3,
};
use amg_core::units::LengthUnit;
use amg_io::brep::{
    BrepContent, BrepImportOptions, BrepScene, StepSchema, export_step, foreign_from_scene,
    import_step, import_step_with, scene_from_foreign,
};
use amg_io::{IoError, IoWarning};
use amg_jobs::{JobCtx, JobError, JobSystem, JobSystemConfig, SpawnOptions};
use step_common::{
    RoundTripStats, assert_matches_foreign, assert_same_scene, external_step_files, fixture,
    strict_timing, summary, tmp,
};

/// The BREP-13 fixtures with assemblies, shared parts, colours, inches and open shapes.
const FIXTURES: [&str; 10] = [
    "box_ap242",
    "cylinder_ap203",
    "coloured_box_ap214",
    "inch_block",
    "bolt_pattern",
    "nested_assembly",
    "two_roots",
    "filleted_box",
    "plate_with_holes",
    "open_shapes",
];

/// The NIST files with tessellated (faceted) geometry instead of B-rep: not B-rep parts.
fn is_tessellated(p: &Path) -> bool {
    p.file_stem()
        .and_then(|s| s.to_str())
        .is_some_and(|s| s.ends_with("-tg"))
}

fn ctx() -> JobCtx {
    JobCtx::detached()
}

fn make_box(x: f64, y: f64, z: f64) -> Shape {
    OcctKernel::new(KernelConfig::default())
        .make_box(&Plane::XY, Vec3::new(x, y, z))
        .unwrap()
        .shape
}

fn plate() -> Shape {
    let path = amg_testkit::testdata_dir().join("brep/plate_with_holes.brep");
    let file = std::fs::File::open(&path).unwrap();
    Shape::read_text(std::io::BufReader::new(file)).unwrap()
}

/// Done-when 1 for one file: tree, names and colours equal BREP-13's `ForeignModel`, and the
/// health report says every part with faces is valid.
fn check_import(path: &Path, require_solids: bool) -> amg_io::brep::BrepImport {
    let started = Instant::now();
    let imported = import_step_with(path, &BrepImportOptions::default(), &ctx())
        .unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    let seconds = started.elapsed().as_secs_f64();
    let model = read_step(path, &ImportOpts::default(), &ctx()).unwrap();
    let what = path.file_name().unwrap().to_string_lossy().into_owned();
    assert_matches_foreign(&imported.scene, &model, &what);
    assert_eq!(imported.health.shapes.len(), imported.scene.objects.len());
    for h in &imported.health.shapes {
        assert!(h.heal_error.is_none(), "{what}: {h:?}");
        assert_eq!(
            h.valid,
            Some(true),
            "{what}: {:?} fails BRepCheck: {h:?}",
            h.name
        );
    }
    assert!(imported.health.all_valid());
    if require_solids {
        let solids: u32 = imported.health.shapes.iter().map(|h| h.solids).sum();
        assert!(solids > 0, "{what}: no solids");
    }
    assert_eq!(imported.report.format, Some(amg_io::Format::Step));
    assert_eq!(imported.report.applied_unit, Some(LengthUnit::Millimetre));
    assert!(
        !imported
            .report
            .warnings
            .iter()
            .any(|w| matches!(w, IoWarning::ShapeInvalid { .. })),
        "{what}: {:?}",
        imported.report.warnings
    );
    let fixed = imported.health.fixed().count();
    println!(
        "{what}: {} ; healed {fixed} part(s); unit {:?}; {seconds:.2} s",
        summary(&imported.scene),
        imported.report.declared_unit
    );
    imported
}

/// Done-when 2 for one scene: export AP242 (and the other schemas when asked), import again,
/// same scene.
fn check_round_trip(scene: &BrepScene, what: &str, schemas: &[StepSchema]) -> RoundTripStats {
    let mut all = RoundTripStats::default();
    for &schema in schemas {
        let out = tmp(&format!("{what}-{schema:?}.step"));
        let report = export_step(scene, schema, &out, &ctx())
            .unwrap_or_else(|e| panic!("{what} {schema:?}: {e}"));
        assert!(report.bytes > 0 && report.files == vec![out.clone()]);
        let head = std::fs::read(&out).unwrap();
        let text = String::from_utf8_lossy(&head[..head.len().min(2000)]).into_owned();
        let tag = match schema {
            StepSchema::Ap203 => "CONFIG_CONTROL_DESIGN",
            StepSchema::Ap214 => "AUTOMOTIVE_DESIGN",
            StepSchema::Ap242 => "AP242_MANAGED_MODEL_BASED_3D_ENGINEERING",
        };
        assert!(text.contains(tag), "{what} {schema:?}: schema header");
        let (back, report) = import_step(&out, &ctx()).unwrap();
        assert_eq!(report.declared_unit, Some(LengthUnit::Millimetre));
        let st = assert_same_scene(scene, &back, &format!("{what} {schema:?}"));
        println!("{what} {schema:?}: {st:?}");
        all.add(&st);
    }
    all
}

#[test]
fn fixtures_import_with_tree_names_and_colours() {
    let mut seen = 0;
    for name in FIXTURES {
        let Some(path) = fixture(name) else { continue };
        seen += 1;
        let imported = check_import(&path, name != "open_shapes");
        match name {
            "bolt_pattern" => {
                // 1 base + 4 instances of one bolt part, one with its own colour.
                assert_eq!(imported.scene.objects.len(), 2);
                let inst = imported.scene.instances();
                assert_eq!(inst.len(), 5);
                assert_eq!(inst.iter().filter(|i| i.0 == inst[1].0).count(), 4);
                assert!(imported.scene.has_assemblies());
            }
            "nested_assembly" => {
                assert!(imported.scene.has_assemblies());
                assert_eq!(imported.scene.objects.len(), 2);
            }
            "inch_block" => assert_eq!(imported.report.declared_unit, Some(LengthUnit::Inch)),
            "coloured_box_ap214" => {
                let o = &imported.scene.objects[0];
                assert!(o.color.is_some());
                assert_eq!(
                    o.face_colors.iter().map(|f| f.0).collect::<Vec<_>>(),
                    [0, 3]
                );
            }
            _ => {}
        }
    }
    println!("{seen} fixtures");
}

/// Done-when 1: the NIST MBE PMI models.
#[test]
fn nist_mbe_pmi_models_import_with_tree_names_and_colours() {
    let Some(files) = external_step_files("io/step/nist-mbe-pmi") else {
        return;
    };
    let mut n = 0;
    for path in files.iter().filter(|p| !is_tessellated(p)) {
        check_import(path, true);
        n += 1;
    }
    println!("NIST MBE PMI: {n} B-rep files imported and checked");
    assert!(n > 0);
}

#[test]
fn fixtures_round_trip_through_ap242() {
    for name in FIXTURES {
        let Some(path) = fixture(name) else { continue };
        let (scene, _) = import_step(&path, &ctx()).unwrap();
        let schemas: &[StepSchema] = if matches!(name, "bolt_pattern" | "coloured_box_ap214") {
            &[StepSchema::Ap242, StepSchema::Ap214, StepSchema::Ap203]
        } else {
            &[StepSchema::Ap242]
        };
        let st = check_round_trip(&scene, name, schemas);
        // The fixtures are written by the kernel itself: every part is at kernel tolerance.
        assert_eq!(st.tolerant, 0, "{name}: {st:?}");
    }
}

/// Done-when 2 on the NIST models.
#[test]
fn nist_mbe_pmi_models_round_trip_through_ap242() {
    let Some(files) = external_step_files("io/step/nist-mbe-pmi") else {
        return;
    };
    let mut n = 0;
    let mut all = RoundTripStats::default();
    for path in files.iter().filter(|p| !is_tessellated(p)) {
        let (scene, _) = import_step(path, &ctx()).unwrap();
        let stem = path.file_stem().unwrap().to_string_lossy().into_owned();
        all.add(&check_round_trip(&scene, &stem, &[StepSchema::Ap242]));
        n += 1;
    }
    println!("NIST MBE PMI: {n} files round-tripped: {all:?}");
}

/// Done-when 3: the owner's Fusion exports (skipped when absent).
#[test]
fn fusion_exports_round_trip() {
    let Some(files) = external_step_files("fusion") else {
        return;
    };
    for path in &files {
        let imported = check_import(path, true);
        let stem = path.file_stem().unwrap().to_string_lossy().into_owned();
        check_round_trip(
            &imported.scene,
            &format!("fusion-{stem}"),
            &[StepSchema::Ap242],
        );
    }
    println!("Fusion exports: {} files round-tripped", files.len());
}

/// A scene built in code: an assembly with a shared, coloured part, an instance colour, a
/// rotated sub-assembly and a part with face colours. Round trip without any fixture.
#[test]
fn built_scene_round_trips_with_instance_colours() {
    let cube = make_box(10.0, 20.0, 30.0);
    let mut model = ForeignModel::new(Vec::new());
    model.prototypes.push(amg_brep::ForeignPrototype {
        name: Some("cube".into()),
        shape: cube.clone(),
        color: Some(amg_io::brep::linear_from_srgb(amg_mesh::Rgba8([
            200, 30, 30, 255,
        ]))),
        face_colors: vec![(
            amg_brep::FaceId(2),
            amg_io::brep::linear_from_srgb(amg_mesh::Rgba8([0, 0, 255, 255])),
        )],
    });
    let inst = |name: &str, x: f64, color: Option<[f32; 4]>| {
        let mut n = ForeignNode::part(
            Some(name),
            cube.clone(),
            Transform::from_translation(Vec3::new(x, 0.0, 0.0)),
        );
        n.instance_of = Some(0);
        n.color = color;
        n
    };
    let green = amg_io::brep::linear_from_srgb(amg_mesh::Rgba8([0, 160, 0, 255]));
    model.roots.push(ForeignNode::assembly(
        Some("top"),
        vec![
            inst("left", 0.0, None),
            inst("right", 50.0, Some(green)),
            ForeignNode::assembly(
                Some("sub"),
                vec![inst("far", 100.0, None)],
                Transform::from_rotation_z(std::f64::consts::FRAC_PI_2),
            ),
        ],
        Transform::IDENTITY,
    ));
    let scene = scene_from_foreign(&model);
    assert_eq!(scene.objects.len(), 1);
    let back = foreign_from_scene(&scene);
    assert_eq!(back.prototypes.len(), 1);
    check_round_trip(&scene, "built", &[StepSchema::Ap242, StepSchema::Ap203]);
    let (re, _) = import_step(&tmp("built-Ap242.step"), &ctx()).unwrap();
    let BrepContent::Group(top) = &re.roots[0].content else {
        panic!("group")
    };
    assert_eq!(top[1].color, Some(amg_mesh::Rgba8([0, 160, 0, 255])));
}

#[test]
fn exports_are_atomic_and_bad_scenes_fail_cleanly() {
    let out = tmp("atomic.step");
    std::fs::write(&out, b"old").unwrap();
    let bad = BrepScene {
        objects: vec![amg_io::brep::BrepObject {
            name: None,
            shape: make_box(1.0, 1.0, 1.0),
            color: None,
            face_colors: Vec::new(),
        }],
        roots: vec![amg_io::brep::BrepNode {
            name: None,
            transform: Transform::from_scale(Vec3::splat(2.0)),
            color: None,
            content: BrepContent::Shape(0),
        }],
        ..BrepScene::default()
    };
    let err = export_step(&bad, StepSchema::Ap242, &out, &ctx()).unwrap_err();
    assert!(matches!(err, IoError::InvalidScene { .. }), "{err:?}");
    assert_eq!(std::fs::read(&out).unwrap(), b"old");
    let leftovers: Vec<_> = std::fs::read_dir(out.parent().unwrap())
        .unwrap()
        .filter_map(Result::ok)
        .filter(|e| e.file_name().to_string_lossy().ends_with(".tmp"))
        .collect();
    assert!(leftovers.is_empty(), "{leftovers:?}");
    let garbage = tmp("garbage.step");
    std::fs::write(&garbage, b"ISO-10303-21;\nthis is not STEP\n").unwrap();
    assert!(matches!(
        import_step(&garbage, &ctx()),
        Err(IoError::Brep(_))
    ));
    assert!(matches!(
        import_step(&tmp("missing.step"), &ctx()),
        Err(IoError::Read { .. })
    ));
}

// ---------------------------------------------------------------- Done-when 4

/// The 50 MB STEP file: 13 separate copies of the 1030-face plate (as BREP-13's test).
fn large_step() -> PathBuf {
    let path = tmp("large-plates.step");
    if std::fs::metadata(&path).is_ok_and(|m| m.len() >= 50_000_000) {
        return path;
    }
    let plate = plate();
    let roots = (0..13)
        .map(|i| {
            let shape = plate
                .transformed(&Transform::from_translation(Vec3::new(
                    0.0,
                    0.0,
                    20.0 * f64::from(i),
                )))
                .unwrap();
            ForeignNode::part(Some(&format!("plate_{i}")), shape, Transform::IDENTITY)
        })
        .collect();
    let started = Instant::now();
    write_step(&path, &ForeignModel::new(roots), StepSchema::Ap214, &ctx()).unwrap();
    let len = std::fs::metadata(&path).unwrap().len();
    println!(
        "wrote {:.1} MB of STEP in {:.1} s",
        len as f64 / 1e6,
        started.elapsed().as_secs_f64()
    );
    assert!(len >= 50_000_000, "{len} bytes");
    path
}

fn job_system() -> JobSystem {
    JobSystem::new(JobSystemConfig::default()).unwrap()
}

fn assert_monotone(samples: &[f64]) {
    assert!(
        samples.windows(2).all(|w| w[0] <= w[1]),
        "progress went back"
    );
}

/// Done-when 4, cancel: the import job cancelled while parsing, during the transfer and while
/// healing returns `Cancelled` (latency < 500 ms under `AMG_STRICT_TIMING=1`).
#[test]
fn large_step_import_reports_progress_and_cancels() {
    let path = large_step();
    let js = job_system();
    for (phase, cancel_at) in [
        ("parse", 0.04_f64),
        ("transfer", 0.32),
        ("heal/check", 0.85),
    ] {
        let returned: Arc<Mutex<Option<(Instant, bool)>>> = Arc::default();
        let in_job = Arc::clone(&returned);
        let job_path = path.clone();
        let mut handle = js.spawn_fn("STEP import", SpawnOptions::default(), move |ctx| {
            let r = import_step_with(&job_path, &BrepImportOptions::default(), ctx);
            *in_job.lock().unwrap() = Some((Instant::now(), matches!(r, Err(IoError::Cancelled))));
            r.map(|_| ())
        });
        let started = Instant::now();
        let mut samples = Vec::new();
        loop {
            let f = f64::from(handle.progress().fraction);
            samples.push(f);
            if f >= cancel_at {
                break;
            }
            assert!(
                started.elapsed() < Duration::from_secs(600),
                "{phase}: stuck at {f}"
            );
            assert!(
                handle.try_take().is_none(),
                "{phase}: ended before the cancel"
            );
            std::thread::sleep(Duration::from_millis(2));
        }
        let cancelled_at = Instant::now();
        handle.cancel();
        let result = loop {
            if let Some(r) = handle.try_take() {
                break r;
            }
            assert!(
                cancelled_at.elapsed() < Duration::from_secs(60),
                "{phase}: no stop"
            );
            std::thread::sleep(Duration::from_micros(200));
        };
        assert!(
            matches!(result, Err(JobError::Cancelled)),
            "{phase}: {result:?}"
        );
        let (at, cancelled) = returned.lock().unwrap().expect("job returned");
        assert!(cancelled, "{phase}: import did not return Cancelled");
        let latency = at.duration_since(cancelled_at);
        assert_monotone(&samples);
        println!(
            "{phase}: cancelled at {:.3} after {:.2} s ({} samples), returned {:.1} ms later",
            samples.last().unwrap(),
            cancelled_at.duration_since(started).as_secs_f64(),
            samples.len(),
            latency.as_secs_f64() * 1e3
        );
        if strict_timing() {
            assert!(latency < Duration::from_millis(500), "{phase}: {latency:?}");
        }
    }
}

/// Done-when 4, progress: a full import of the 50 MB file runs as a job with monotone progress
/// through parse, transfer and heal/check, and gives 13 valid plates.
#[test]
fn large_step_import_completes_with_monotone_progress() {
    let path = large_step();
    let js = job_system();
    let job_path = path.clone();
    let mut handle = js.spawn_fn("STEP import", SpawnOptions::default(), move |ctx| {
        import_step_with(&job_path, &BrepImportOptions::default(), ctx)
    });
    let started = Instant::now();
    let mut samples = Vec::new();
    let result = loop {
        samples.push(f64::from(handle.progress().fraction));
        if let Some(r) = handle.try_take() {
            break r;
        }
        std::thread::sleep(Duration::from_millis(5));
    };
    let imported = result.unwrap();
    let seconds = started.elapsed().as_secs_f64();
    assert_eq!(imported.scene.roots.len(), 13);
    assert!(imported.health.all_valid());
    assert_monotone(&samples);
    let parse = samples.iter().filter(|&&f| f > 0.0 && f < 0.15).count();
    let transfer = samples
        .iter()
        .filter(|&&f| (0.17..0.79).contains(&f))
        .count();
    let heal = samples.iter().filter(|&&f| (0.8..1.0).contains(&f)).count();
    assert!(
        parse >= 3 && transfer >= 3,
        "parse {parse}, transfer {transfer}"
    );
    println!(
        "50 MB import in {seconds:.1} s, {} samples ({parse} parse, {transfer} transfer, {heal} heal/check)",
        samples.len()
    );
}
