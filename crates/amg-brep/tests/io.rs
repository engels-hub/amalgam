//! BREP-13: STEP and IGES read/write through XCAF into `ForeignModel`.
//!
//! The fixtures (`testdata/brep/{step,iges}/`, made by the `gen_brep_io` example from
//! `common/io_models.rs`) are read where they are real files; `*.step` is git-LFS tracked, so an
//! un-fetched pointer is skipped unless `AMG_REQUIRE_TESTDATA=1`. The same models are also built
//! in code and round-tripped through temporary files, so every check runs without LFS too.
#![cfg(feature = "occt")]
#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::print_stdout,
    clippy::print_stderr,
    clippy::float_cmp,
    reason = "test code"
)]

#[path = "common/io_models.rs"]
mod io_models;

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use amg_brep::io::{read_iges, read_step, write_iges, write_step};
use amg_brep::{
    BrepKernel, CheckOpts, ForeignModel, ForeignNode, ImportOpts, JobCtx, KernelError, Shape,
    StepSchema, SubKind, Transform, Vec3,
};
use amg_jobs::{JobError, JobSystem, JobSystemConfig, SpawnOptions};
use io_models::{
    IGES_FIXTURES, STEP_FIXTURES, STRICT, Same, assert_same_model, iges_model, iges_path,
    is_lfs_pointer, kernel, require_testdata, shape_facts, step_model, step_path,
};

fn ctx() -> JobCtx {
    JobCtx::detached()
}

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}

fn tmp(name: &str) -> PathBuf {
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("brep-io");
    std::fs::create_dir_all(&dir).unwrap();
    dir.join(name)
}

/// The fixture at `path` when it is a real file; `None` (skip) for an un-fetched LFS pointer.
fn fixture(path: &Path) -> Option<&Path> {
    assert!(path.is_file(), "fixture {} is missing", path.display());
    if is_lfs_pointer(path) {
        assert!(
            !require_testdata(),
            "{} is an un-fetched git-LFS pointer and AMG_REQUIRE_TESTDATA=1",
            path.display()
        );
        eprintln!("SKIP {}: git-LFS pointer", path.display());
        return None;
    }
    Some(path)
}

/// The comparison of a read against the model it was written from: no prototype indices in the
/// built model, and IGES puts the part colour on every face.
const AGAINST_BUILT: Same = Same {
    instances: false,
    ..STRICT
};

fn count_instances(model: &ForeignModel) -> Vec<usize> {
    let mut counts = vec![0; model.prototypes.len()];
    for node in model.nodes() {
        if let Some(p) = node.instance_of {
            counts[p as usize] += 1;
        }
    }
    counts
}

/// Done-when 1: each of the 10 STEP fixtures reads back as the model it was written from, and
/// writing what was read then reading again gives the same tree, names, colours, solid count and
/// per-solid volume within 1e-9 relative (in the fixture's own schema).
#[test]
fn step_fixtures_round_trip() {
    let k = kernel();
    let mut fixtures_read = 0;
    for (name, schema) in STEP_FIXTURES {
        let built = step_model(&k, name);
        // The fixture file, and the same model written now.
        let fresh = tmp(&format!("{name}.step"));
        write_step(&fresh, &built, schema, &ctx()).unwrap_or_else(|e| panic!("{name}: {e}"));
        let mut sources = vec![fresh];
        if let Some(path) = fixture(&step_path(name)) {
            sources.push(path.to_owned());
            fixtures_read += 1;
        }
        for source in sources {
            let what = format!("{name} ({})", source.display());
            let first = read_step(&source, &ImportOpts::default(), &ctx())
                .unwrap_or_else(|e| panic!("{what}: {e}"));
            assert!(first.warnings.is_empty(), "{what}: {:?}", first.warnings);
            assert_same_model(&built, &first, AGAINST_BUILT, &format!("{what} vs built"));
            let again = tmp(&format!("{name}.again.step"));
            write_step(&again, &first, schema, &ctx()).unwrap();
            let second = read_step(&again, &ImportOpts::default(), &ctx()).unwrap();
            assert_same_model(&first, &second, STRICT, &format!("{what} round trip"));
            for node in first.nodes().into_iter().filter(|n| n.shape.is_some()) {
                let shape = node.shape.as_ref().unwrap();
                let report = k.check(shape, &CheckOpts::default());
                assert!(report.valid, "{what}: {:?} invalid: {report:?}", node.name);
            }
        }
    }
    println!("STEP fixtures: 10 built and round-tripped, {fixtures_read} fixture files read");
}

/// Done-when 1, the coverage of the fixture set: an assembly reusing parts, colours (part,
/// instance, face) and a file in inches, scaled to mm.
#[test]
fn step_fixture_set_covers_assemblies_colours_and_inches() {
    let k = kernel();
    let read = |name: &str| {
        let (_, schema) = STEP_FIXTURES.iter().find(|f| f.0 == name).unwrap();
        let path = tmp(&format!("cover-{name}.step"));
        write_step(&path, &step_model(&k, name), *schema, &ctx()).unwrap();
        read_step(&path, &ImportOpts::default(), &ctx()).unwrap()
    };

    let bolts = read("bolt_pattern");
    assert_eq!(bolts.prototypes.len(), 2);
    let mut counts = count_instances(&bolts);
    counts.sort_unstable();
    assert_eq!(counts, [1, 4], "the bolt is instanced 4 times");
    let pattern = &bolts.roots[0];
    assert_eq!(pattern.name.as_deref(), Some("bolt_pattern"));
    let bolt_shapes: Vec<&Shape> = pattern.children[1..]
        .iter()
        .map(|n| n.shape.as_ref().unwrap())
        .collect();
    assert!(
        bolt_shapes.windows(2).all(|w| w[0].ptr_eq(w[1])),
        "instances share one Shape"
    );
    assert_eq!(
        pattern.children[3].color,
        Some(io_models::YELLOW),
        "instance colour"
    );
    assert_eq!(pattern.children[2].color, None);
    assert_eq!(
        pattern.children[0].color,
        Some(io_models::GREY),
        "part colour"
    );

    let nested = read("nested_assembly");
    let robot = &nested.roots[0];
    assert_eq!(robot.children.len(), 2);
    assert!(robot.children.iter().all(|arm| arm.children.len() == 2));
    assert_eq!(count_instances(&nested), [2, 2]);
    let right = robot.children[1].transform;
    let x = right.transform_vector3(Vec3::X);
    assert!(
        (x - Vec3::Y).length() < 1e-12,
        "arm_right is turned 90°: {x:?}"
    );

    let coloured = read("coloured_box_ap214");
    let node = &coloured.roots[0];
    assert_eq!(node.color, Some(io_models::RED));
    let mut faces = node.face_colors.clone();
    faces.sort_by_key(|c| c.0);
    assert_eq!(
        faces,
        [
            (amg_brep::FaceId(0), io_models::BLUE),
            (amg_brep::FaceId(3), io_models::GREEN)
        ]
    );

    let inch = read("inch_block");
    assert_eq!(inch.source_unit_mm, 25.4);
    let bbox = inch.roots[0].shape.as_ref().unwrap().bbox();
    let size = bbox.max - bbox.min;
    for (got, want) in [size.x, size.y, size.z].into_iter().zip([25.4, 50.8, 76.2]) {
        assert!((got - want).abs() < 1e-9, "inch block size {size:?}");
    }
    let text = std::fs::read_to_string(tmp("cover-inch_block.step")).unwrap();
    assert!(text.contains("INCH"), "the file is written in inches");
    assert_eq!(read("box_ap242").source_unit_mm, 1.0);
}

/// Each schema writes its own FILE_SCHEMA, and keeps the colours.
#[test]
fn schemas_write_their_headers() {
    let k = kernel();
    let model = step_model(&k, "coloured_box_ap214");
    for (schema, header) in [
        (StepSchema::Ap203, "CONFIG_CONTROL_DESIGN"),
        (StepSchema::Ap214, "AUTOMOTIVE_DESIGN"),
        (
            StepSchema::Ap242,
            "AP242_MANAGED_MODEL_BASED_3D_ENGINEERING",
        ),
    ] {
        let path = tmp(&format!("schema-{schema:?}.step"));
        write_step(&path, &model, schema, &ctx()).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let start = text.find("FILE_SCHEMA").unwrap();
        let line = &text[start..start + text[start..].find(';').unwrap()];
        assert!(line.contains(header), "{schema:?}: {line}");
        let back = read_step(&path, &ImportOpts::default(), &ctx()).unwrap();
        // OCCT 8 writes colours in AP203 too (through SHAPE_APPEARANCE_LAYER_MIM).
        assert_eq!(back.roots[0].color, Some(io_models::RED), "{schema:?}");
    }
}

/// A canonical summary of a read: the tree with every f64/f32 bit, and the BinTools bytes of
/// every prototype.
fn digest(model: &ForeignModel) -> (String, Vec<Vec<u8>>) {
    let mut tree = String::new();
    for node in model.nodes() {
        tree.push_str(&format!(
            "{:?} {:?} {:?} {:?} {:?} {}\n",
            node.name,
            node.transform.to_cols_array().map(f64::to_bits),
            node.color.map(|c| c.map(f32::to_bits)),
            node.instance_of,
            node.face_colors,
            node.children.len()
        ));
    }
    let shapes = model
        .prototypes
        .iter()
        .map(|p| {
            let mut bytes = Vec::new();
            p.shape.write_bin(&mut bytes).unwrap();
            bytes
        })
        .collect();
    (tree, shapes)
}

/// Done-when 2: reading one STEP file on 4 threads at once gives identical results (OCCT 8.0.1:
/// no global lock for STEP).
#[test]
fn parallel_step_reads_are_identical() {
    let k = kernel();
    for name in ["bolt_pattern", "filleted_box"] {
        let (_, schema) = STEP_FIXTURES.iter().find(|f| f.0 == name).unwrap();
        let path = tmp(&format!("parallel-{name}.step"));
        write_step(&path, &step_model(&k, name), *schema, &ctx()).unwrap();
        let reference = digest(&read_step(&path, &ImportOpts::default(), &ctx()).unwrap());
        let results: Vec<_> = std::thread::scope(|scope| {
            let handles: Vec<_> = (0..4)
                .map(|_| {
                    scope.spawn(|| {
                        (0..3)
                            .map(|_| {
                                digest(&read_step(&path, &ImportOpts::default(), &ctx()).unwrap())
                            })
                            .collect::<Vec<_>>()
                    })
                })
                .collect();
            handles
                .into_iter()
                .flat_map(|h| h.join().unwrap())
                .collect()
        });
        assert_eq!(results.len(), 12);
        for (i, result) in results.iter().enumerate() {
            assert_eq!(result.0, reference.0, "{name}: tree of read {i}");
            assert!(result.1 == reference.1, "{name}: shapes of read {i} differ");
        }
    }
}

/// Writing one model (shared shapes) from 4 threads at once: every file reads back the same, and
/// the shared inputs keep their bytes (the writers copy them).
#[test]
fn parallel_step_writes_of_one_model() {
    let k = kernel();
    let model = step_model(&k, "bolt_pattern");
    let before = digest(&model);
    let digests: Vec<_> = std::thread::scope(|scope| {
        let handles: Vec<_> = (0..4)
            .map(|t| {
                let model = &model;
                scope.spawn(move || {
                    (0..3)
                        .map(|i| {
                            let path = tmp(&format!("parallel-write-{t}-{i}.step"));
                            write_step(&path, model, StepSchema::Ap242, &ctx()).unwrap();
                            digest(&read_step(&path, &ImportOpts::default(), &ctx()).unwrap())
                        })
                        .collect::<Vec<_>>()
                })
            })
            .collect();
        handles
            .into_iter()
            .flat_map(|h| h.join().unwrap())
            .collect()
    });
    assert!(digests.windows(2).all(|w| w[0] == w[1]), "the reads differ");
    assert!(digest(&model) == before, "an input changed");
}

/// The STEP models the stress tests use: two assemblies with shared prototypes and instance
/// colours (`bolt_pattern`, `nested_assembly`) and per-face colours (`coloured_box_ap214`).
const STRESS_MODELS: [&str; 3] = ["bolt_pattern", "nested_assembly", "coloured_box_ap214"];

/// The schema a STEP fixture is written in.
fn schema_of(name: &str) -> StepSchema {
    STEP_FIXTURES.iter().find(|f| f.0 == name).unwrap().1
}

/// BREP-13 fix: 8 threads × 6 reads at once, spread over the three stress models, each equal to
/// the single-threaded read of its file (tree bits and prototype BinTools bytes).
#[test]
fn parallel_step_reads_stress() {
    let k = kernel();
    let files: Vec<_> = STRESS_MODELS
        .iter()
        .map(|name| {
            let path = tmp(&format!("stress-read-{name}.step"));
            write_step(&path, &step_model(&k, name), schema_of(name), &ctx()).unwrap();
            let reference = digest(&read_step(&path, &ImportOpts::default(), &ctx()).unwrap());
            (path, reference)
        })
        .collect();
    let mismatches: Vec<String> = std::thread::scope(|scope| {
        let handles: Vec<_> = (0..8)
            .map(|t| {
                let files = &files;
                scope.spawn(move || {
                    let mut bad = Vec::new();
                    for i in 0..6 {
                        let (path, reference) = &files[(t + i) % files.len()];
                        let read = read_step(path, &ImportOpts::default(), &ctx()).unwrap();
                        if digest(&read) != *reference {
                            bad.push(format!("thread {t} read {i} of {}", path.display()));
                        }
                    }
                    bad
                })
            })
            .collect();
        handles
            .into_iter()
            .flat_map(|h| h.join().unwrap())
            .collect()
    });
    assert!(mismatches.is_empty(), "reads differ: {mismatches:#?}");
}

/// BREP-13 fix: STEP reads, STEP writes of shared models and IGES writes (global lock, IGES
/// statics) all at once. Every STEP read, of a file written before or during the run, equals the
/// single-threaded read of its model; every IGES round trip equals the single-threaded one; the
/// shared inputs keep their bytes.
#[test]
fn mixed_parallel_step_reads_and_writes() {
    let k = kernel();
    let models: Vec<ForeignModel> = STRESS_MODELS.iter().map(|n| step_model(&k, n)).collect();
    let before: Vec<_> = models.iter().map(digest).collect();
    let references: Vec<_> = STRESS_MODELS
        .iter()
        .zip(&models)
        .map(|(name, model)| {
            let path = tmp(&format!("mixed-ref-{name}.step"));
            write_step(&path, model, schema_of(name), &ctx()).unwrap();
            let reference = digest(&read_step(&path, &ImportOpts::default(), &ctx()).unwrap());
            (path, reference)
        })
        .collect();
    let iges_path = tmp("mixed-ref.igs");
    write_iges(&iges_path, &models[0], &ctx()).unwrap();
    let iges_reference = digest(&read_iges(&iges_path, &ImportOpts::default(), &ctx()).unwrap());
    let mismatches: Vec<String> = std::thread::scope(|scope| {
        let mut handles = Vec::new();
        // 3 readers of the files written before the run.
        for t in 0..3 {
            let references = &references;
            handles.push(scope.spawn(move || {
                let mut bad = Vec::new();
                for i in 0..6 {
                    let (path, reference) = &references[(t + i) % references.len()];
                    let read = read_step(path, &ImportOpts::default(), &ctx()).unwrap();
                    if digest(&read) != *reference {
                        bad.push(format!("reader {t} read {i} of {}", path.display()));
                    }
                }
                bad
            }));
        }
        // 4 writers of the shared models, each reading its file back.
        for t in 0..4 {
            let (models, references) = (&models, &references);
            handles.push(scope.spawn(move || {
                let mut bad = Vec::new();
                for i in 0..4 {
                    let m = (t + i) % models.len();
                    let name = STRESS_MODELS[m];
                    let path = tmp(&format!("mixed-write-{t}-{i}.step"));
                    write_step(&path, &models[m], schema_of(name), &ctx()).unwrap();
                    let read = read_step(&path, &ImportOpts::default(), &ctx()).unwrap();
                    if digest(&read) != references[m].1 {
                        bad.push(format!("writer {t} write {i} of {name}"));
                    }
                }
                bad
            }));
        }
        // 1 IGES writer: the global lock and the IGES statics next to the STEP calls.
        let (models, iges_reference) = (&models, &iges_reference);
        handles.push(scope.spawn(move || {
            let mut bad = Vec::new();
            for i in 0..3 {
                let path = tmp(&format!("mixed-write-{i}.igs"));
                write_iges(&path, &models[0], &ctx()).unwrap();
                let read = read_iges(&path, &ImportOpts::default(), &ctx()).unwrap();
                if digest(&read) != *iges_reference {
                    bad.push(format!("IGES write {i}"));
                }
            }
            bad
        }));
        handles
            .into_iter()
            .flat_map(|h| h.join().unwrap())
            .collect()
    });
    assert!(mismatches.is_empty(), "results differ: {mismatches:#?}");
    let after: Vec<_> = models.iter().map(digest).collect();
    assert!(after == before, "an input changed");
}

/// Done-when 3: the 3 IGES fixtures read back as written (solids, volumes, names, part colour),
/// and a write-read round trip of what was read keeps them.
#[test]
fn iges_fixtures_round_trip() {
    let k = kernel();
    // IGES writes the part colour on every face entity too, so face colours are compared only on
    // the second round trip.
    let against_built = Same {
        face_colors: false,
        ..AGAINST_BUILT
    };
    let mut fixtures_read = 0;
    for name in IGES_FIXTURES {
        let built = iges_model(&k, name);
        let fresh = tmp(&format!("{name}.igs"));
        write_iges(&fresh, &built, &ctx()).unwrap();
        let mut sources = vec![fresh];
        if let Some(path) = fixture(&iges_path(name)) {
            sources.push(path.to_owned());
            fixtures_read += 1;
        }
        for source in sources {
            let what = format!("{name} ({})", source.display());
            let first = read_iges(&source, &ImportOpts::default(), &ctx())
                .unwrap_or_else(|e| panic!("{what}: {e}"));
            assert_same_model(&built, &first, against_built, &format!("{what} vs built"));
            let again = tmp(&format!("{name}.again.igs"));
            write_iges(&again, &first, &ctx()).unwrap();
            let second = read_iges(&again, &ImportOpts::default(), &ctx()).unwrap();
            assert_same_model(&first, &second, STRICT, &format!("{what} round trip"));
            let shape = first.roots[0].shape.as_ref().unwrap();
            assert_eq!(shape.solids().len(), 1, "{what}: IGES keeps the solid");
            assert!(k.check(shape, &CheckOpts::default()).valid, "{what}");
        }
    }
    println!("IGES fixtures: 3 built and round-tripped, {fixtures_read} fixture files read");
}

/// IGES has no assemblies: an assembly is written as its placed parts.
#[test]
fn iges_flattens_assemblies() {
    let k = kernel();
    let path = tmp("flattened.igs");
    write_iges(&path, &step_model(&k, "bolt_pattern"), &ctx()).unwrap();
    let back = read_iges(&path, &ImportOpts::default(), &ctx()).unwrap();
    let solids: usize = back
        .nodes()
        .iter()
        .filter_map(|n| n.shape.as_ref())
        .map(|s| s.solids().len())
        .sum();
    assert_eq!(solids, 5, "base and 4 bolts");
    assert!(
        back.nodes()
            .iter()
            .all(|n| n.children.is_empty() || n.shape.is_none())
    );
}

/// The 50 MB STEP file of Done-when 4: 13 separate copies of the 1030-face plate, made once per
/// target directory.
fn large_step() -> PathBuf {
    let path = tmp("large-plates.step");
    if std::fs::metadata(&path).is_ok_and(|m| m.len() >= 50_000_000) {
        return path;
    }
    let plate = io_models::brep_fixture("plate_with_holes");
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
        "progress went back: {samples:?}"
    );
}

/// Done-when 4, cancellation: a 50 MB STEP read cancelled while parsing, as the transfer starts
/// and during the transfer, returns `Cancelled` within 500 ms (asserted under `AMG_STRICT_TIMING=1`).
#[test]
fn large_step_read_reports_progress_and_cancels() {
    let path = large_step();
    let js = job_system();
    // Job-level fractions: parsing reports 0..0.16, the transfer 0.16..0.8.
    for (phase, cancel_at) in [
        ("parse", 0.04_f64),
        ("transfer start", 0.165),
        ("transfer", 0.32),
    ] {
        let returned: Arc<Mutex<Option<(Instant, bool)>>> = Arc::default();
        let returned_in_job = Arc::clone(&returned);
        let job_path = path.clone();
        let mut handle = js.spawn_fn("large STEP read", SpawnOptions::default(), move |ctx| {
            let opts = ImportOpts {
                heal: false,
                ..ImportOpts::default()
            };
            let result = read_step(&job_path, &opts, ctx);
            let cancelled = matches!(result, Err(KernelError::Cancelled));
            *returned_in_job.lock().unwrap() = Some((Instant::now(), cancelled));
            result.map(|_| ())
        });
        let started = Instant::now();
        let mut samples = Vec::new();
        loop {
            let fraction = f64::from(handle.progress().fraction);
            samples.push(fraction);
            if fraction >= cancel_at {
                break;
            }
            assert!(
                started.elapsed() < Duration::from_secs(300),
                "{phase}: no progress past {fraction}"
            );
            assert!(
                handle.try_take().is_none(),
                "{phase}: the read ended before the cancel"
            );
            std::thread::sleep(Duration::from_millis(2));
        }
        let cancelled_at = Instant::now();
        handle.cancel();
        let result = loop {
            if let Some(result) = handle.try_take() {
                break result;
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
        let (at, was_cancelled) = returned.lock().unwrap().expect("job returned");
        assert!(was_cancelled, "{phase}: read_step did not return Cancelled");
        let latency = at.duration_since(cancelled_at);
        assert_monotone(&samples);
        let intermediate = samples.iter().filter(|&&f| f > 0.0 && f < 1.0).count();
        assert!(
            intermediate >= 3,
            "{phase}: too few progress samples {samples:?}"
        );
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

/// Done-when 4, progress: a full read of the 50 MB file reports monotone progress through the
/// parse and the transfer, and gives the 13 plates.
#[test]
fn large_step_read_completes_with_monotone_progress() {
    let path = large_step();
    let js = job_system();
    let job_path = path.clone();
    let mut handle = js.spawn_fn("large STEP read", SpawnOptions::default(), move |ctx| {
        let opts = ImportOpts {
            heal: false,
            ..ImportOpts::default()
        };
        read_step(&job_path, &opts, ctx)
    });
    let started = Instant::now();
    let mut samples = Vec::new();
    let result = loop {
        samples.push(f64::from(handle.progress().fraction));
        if let Some(result) = handle.try_take() {
            break result;
        }
        std::thread::sleep(Duration::from_millis(5));
    };
    let model = result.unwrap();
    let seconds = started.elapsed().as_secs_f64();
    assert_eq!(model.roots.len(), 13);
    assert_monotone(&samples);
    let parse = samples.iter().filter(|&&f| f > 0.0 && f < 0.15).count();
    let transfer = samples
        .iter()
        .filter(|&&f| (0.17..0.79).contains(&f))
        .count();
    assert!(
        parse >= 3 && transfer >= 3,
        "parse {parse}, transfer {transfer} samples"
    );
    for root in &model.roots {
        let (solids, faces, _, _) = shape_facts(root.shape.as_ref().unwrap());
        assert_eq!((solids, faces), (1, 1030));
    }
    println!(
        "50 MB read in {seconds:.1} s, {} samples ({parse} parsing, {transfer} transfer)",
        samples.len()
    );
}

/// Done-when 5: the owner's Fusion STEP exports under `$AMG_TESTDATA_DIR/fusion/` import as
/// valid solids (skipped when the directory is absent).
#[test]
fn fusion_exports_import_as_valid_solids() {
    let Some(root) = std::env::var_os("AMG_TESTDATA_DIR").map(PathBuf::from) else {
        assert!(
            !require_testdata(),
            "AMG_TESTDATA_DIR is not set and AMG_REQUIRE_TESTDATA=1"
        );
        eprintln!("SKIP: AMG_TESTDATA_DIR is not set");
        return;
    };
    let dir = root.join("fusion");
    let Ok(entries) = std::fs::read_dir(&dir) else {
        eprintln!("SKIP: {} does not exist", dir.display());
        return;
    };
    let k = kernel();
    let mut files = 0;
    for entry in entries {
        let path = entry.unwrap().path();
        let ext = path
            .extension()
            .and_then(|e| e.to_str())
            .map(str::to_ascii_lowercase);
        if !matches!(ext.as_deref(), Some("step" | "stp")) || is_lfs_pointer(&path) {
            continue;
        }
        files += 1;
        let model = read_step(&path, &ImportOpts::default(), &ctx())
            .unwrap_or_else(|e| panic!("{}: {e}", path.display()));
        let mut solids = 0;
        for proto in &model.prototypes {
            for solid in proto.shape.solids() {
                solids += 1;
                let report = k.check(&solid, &CheckOpts::default());
                assert!(
                    report.valid,
                    "{}: {:?}: {report:?}",
                    path.display(),
                    proto.name
                );
                assert!(solid.volume().unwrap() > 0.0, "{}", path.display());
            }
        }
        assert!(solids > 0, "{}: no solids", path.display());
        println!("{}: {solids} valid solids", path.display());
    }
    println!("Fusion exports: {files} files");
}

/// The writers work on private copies: the caller's shapes keep their BinTools bytes.
#[test]
fn writing_leaves_the_inputs_unchanged() {
    let k = kernel();
    for name in ["bolt_pattern", "nested_assembly", "filleted_box"] {
        let model = step_model(&k, name);
        let shapes: Vec<&Shape> = model
            .prototypes
            .iter()
            .map(|p| &p.shape)
            .chain(model.nodes().into_iter().filter_map(|n| n.shape.as_ref()))
            .collect();
        let bytes = |shapes: &[&Shape]| -> Vec<Vec<u8>> {
            shapes
                .iter()
                .map(|s| {
                    let mut out = Vec::new();
                    s.write_bin(&mut out).unwrap();
                    out
                })
                .collect()
        };
        let before = bytes(&shapes);
        write_step(
            &tmp(&format!("inputs-{name}.step")),
            &model,
            StepSchema::Ap242,
            &ctx(),
        )
        .unwrap();
        write_iges(&tmp(&format!("inputs-{name}.igs")), &model, &ctx()).unwrap();
        assert!(bytes(&shapes) == before, "{name}: an input changed");
    }
}

/// Healing on import keeps face colours on the same faces (through the heal history).
#[test]
fn healing_keeps_face_colours_on_their_faces() {
    let k = kernel();
    let path = tmp("heal-colours.step");
    write_step(
        &path,
        &step_model(&k, "filleted_box"),
        StepSchema::Ap242,
        &ctx(),
    )
    .unwrap();
    let raw = read_step(
        &path,
        &ImportOpts {
            heal: false,
            ..ImportOpts::default()
        },
        &ctx(),
    )
    .unwrap();
    let healed = read_step(&path, &ImportOpts::default(), &ctx()).unwrap();
    let face = |model: &ForeignModel| {
        let node = &model.roots[0];
        assert_eq!(node.face_colors.len(), 1, "{:?}", node.face_colors);
        let (f, color) = node.face_colors[0];
        let shape = node.shape.as_ref().unwrap();
        (shape.surface(f).class(), shape.face_area(f), color)
    };
    let (class_raw, area_raw, color_raw) = face(&raw);
    let (class_healed, area_healed, color_healed) = face(&healed);
    assert_eq!(class_raw, class_healed);
    assert!((area_raw - area_healed).abs() <= 1e-9 * area_raw);
    assert_eq!(color_raw, color_healed);
    assert_eq!(color_raw, io_models::YELLOW);
}

/// Names and colours are skipped when the options say so.
#[test]
fn names_and_colours_can_be_skipped() {
    let k = kernel();
    let path = tmp("skip.step");
    write_step(
        &path,
        &step_model(&k, "bolt_pattern"),
        StepSchema::Ap242,
        &ctx(),
    )
    .unwrap();
    let bare = read_step(
        &path,
        &ImportOpts {
            heal: true,
            read_colors: false,
            read_names: false,
        },
        &ctx(),
    )
    .unwrap();
    assert!(
        bare.nodes()
            .iter()
            .all(|n| n.name.is_none() && n.color.is_none())
    );
    assert!(
        bare.prototypes
            .iter()
            .all(|p| p.name.is_none() && p.color.is_none())
    );
    assert_eq!(bare.nodes().len(), 6);
}

#[test]
fn bad_inputs_fail_cleanly() {
    let k = kernel();
    let missing = tmp("does-not-exist.step");
    assert!(matches!(
        read_step(&missing, &ImportOpts::default(), &ctx()),
        Err(KernelError::Io(_))
    ));
    let garbage = tmp("garbage.step");
    std::fs::write(&garbage, b"this is not a STEP file\n").unwrap();
    assert!(matches!(
        read_step(&garbage, &ImportOpts::default(), &ctx()),
        Err(KernelError::Format(_))
    ));
    let garbage_iges = tmp("garbage.igs");
    std::fs::write(&garbage_iges, b"neither is this IGES\n").unwrap();
    assert!(read_iges(&garbage_iges, &ImportOpts::default(), &ctx()).is_err());

    let shape = k
        .make_box(&amg_brep::Plane::XY, Vec3::new(1.0, 1.0, 1.0))
        .unwrap()
        .shape;
    let scaled = ForeignModel::new(vec![ForeignNode::part(
        None,
        shape.clone(),
        Transform::from_scale(Vec3::splat(2.0)),
    )]);
    assert!(matches!(
        write_step(&tmp("scaled.step"), &scaled, StepSchema::Ap242, &ctx()),
        Err(KernelError::InvalidInput { what: "model", .. })
    ));
    let mut bad_face = ForeignNode::part(None, shape, Transform::IDENTITY);
    bad_face.face_colors = vec![(amg_brep::FaceId(6), io_models::RED)];
    assert!(matches!(
        write_step(
            &tmp("bad-face.step"),
            &ForeignModel::new(vec![bad_face]),
            StepSchema::Ap242,
            &ctx()
        ),
        Err(KernelError::InvalidInput { what: "model", .. })
    ));
}

#[test]
fn cancelled_jobs_do_not_run() {
    let k = kernel();
    let token = amg_jobs::CancelToken::new();
    token.cancel();
    let cancelled = JobCtx::with_token(token);
    let path = tmp("cancelled.step");
    let model = step_model(&k, "box_ap242");
    assert!(matches!(
        write_step(&path, &model, StepSchema::Ap242, &cancelled),
        Err(KernelError::Cancelled)
    ));
    write_step(&path, &model, StepSchema::Ap242, &ctx()).unwrap();
    assert!(matches!(
        read_step(&path, &ImportOpts::default(), &cancelled),
        Err(KernelError::Cancelled)
    ));
    assert!(matches!(
        read_iges(&tmp("cancelled.igs"), &ImportOpts::default(), &cancelled),
        Err(KernelError::Io(_) | KernelError::Cancelled)
    ));
}

/// A root that is also an assembly component, and a placed root, are written as parts of their
/// own (their placement baked in).
#[test]
fn placed_and_shared_roots_are_written() {
    let k = kernel();
    let mut model = step_model(&k, "bolt_pattern");
    let mut extra = model.roots[0].children[1].clone();
    extra.name = Some("loose_bolt".into());
    extra.transform = Transform::from_translation(Vec3::new(100.0, 0.0, 0.0));
    model.roots.push(extra);
    let path = tmp("placed-root.step");
    write_step(&path, &model, StepSchema::Ap242, &ctx()).unwrap();
    let back = read_step(&path, &ImportOpts::default(), &ctx()).unwrap();
    assert_eq!(back.roots.len(), 2);
    let loose = &back.roots[1];
    assert_eq!(loose.name.as_deref(), Some("loose_bolt"));
    let bbox = loose.shape.as_ref().unwrap().bbox();
    assert!((bbox.min.x - 98.0).abs() < 1e-6, "{bbox:?}");
    assert_eq!(
        loose.shape.as_ref().unwrap().count(SubKind::Face),
        3,
        "the bolt"
    );
}

/// The writers name no person: OCCT's AP203 person and organisation (the OS user and the host's
/// IP address) and the IGES header author are replaced by `unknown`.
#[test]
fn files_do_not_name_the_user() {
    let k = kernel();
    let model = step_model(&k, "box_ap242");
    let step = tmp("anonymous.step");
    write_step(&step, &model, StepSchema::Ap203, &ctx()).unwrap();
    let iges = tmp("anonymous.igs");
    write_iges(&iges, &model, &ctx()).unwrap();
    let step_text = std::fs::read_to_string(&step).unwrap();
    let iges_text = std::fs::read_to_string(&iges).unwrap();
    assert!(
        step_text.contains("PERSON('unknown',$,$,$,$,$)"),
        "{step_text}"
    );
    assert!(step_text.contains("ORGANIZATION('unknown',"));
    assert!(iges_text.contains("7Hunknown"));
    let user = std::env::var("USERNAME").or_else(|_| std::env::var("USER"));
    if let Some(user) = user.ok().filter(|u| u.len() >= 3) {
        assert!(!step_text.contains(&user), "the STEP file names the user");
        assert!(!iges_text.contains(&user), "the IGES file names the user");
    }
}

/// Names are UTF-8 on both sides (STEP escapes them, IGES keeps 8-bit text).
#[test]
fn non_ascii_names_round_trip() {
    let k = kernel();
    let shape = k
        .make_box(&amg_brep::Plane::XY, Vec3::new(1.0, 2.0, 3.0))
        .unwrap()
        .shape;
    let name = "Bügel µ-Teil 部品";
    let model = ForeignModel::new(vec![ForeignNode::part(
        Some(name),
        shape,
        Transform::IDENTITY,
    )]);
    let path = tmp("utf8.step");
    write_step(&path, &model, StepSchema::Ap242, &ctx()).unwrap();
    let back = read_step(&path, &ImportOpts::default(), &ctx()).unwrap();
    assert_eq!(back.roots[0].name.as_deref(), Some(name));
}
