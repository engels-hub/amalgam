//! S20 KH-01: kernel failure capture.
//!
//! Done-when 1 (forced failures write schema-1 dumps), 2 (every dump replays as `Reproduced`
//! in a fresh process), 3 (indices survive a text `.brep` round trip; a forced re-index falls
//! back to fingerprints), 4 (write-ahead: an abort inside the op leaves `pending/`, recovered as
//! a `Crash` dump that replays), 5 (dedup and retention) and 6 (`OnFailure` does no I/O on
//! success; the < 1 µs overhead is the criterion bench `benches/capture.rs`, asserted here only
//! with `AMG_STRICT_TIMING=1`).
//!
//! Tests 2 and 4 re-run this test binary as a child process (`--exact <child test>`), selected
//! by an environment variable; without it the child tests return at once.
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

#[path = "common/fault.rs"]
mod fault;

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};
#[cfg(feature = "occt")]
use std::process::Command;
use std::sync::Arc;
use std::time::{Duration, Instant};

#[cfg(feature = "occt")]
use amg_brep::BrepKernel;
use amg_brep::capture::{
    CaptureConfig, CaptureMode, FailureClass, FailureInfo, FailureRecorder, KernelOp,
    RecordingKernel, read_manifest,
};
use amg_brep::{Plane, Vec3};

#[cfg(feature = "occt")]
/// Env var: the dump a `replay_child` run replays.
const REPLAY_DUMP: &str = "AMG_KH01_REPLAY_DUMP";
#[cfg(feature = "occt")]
/// Env var: with `1`, the replay child's kernel aborts in `make_sphere`.
const REPLAY_ABORT: &str = "AMG_KH01_REPLAY_ABORT";
#[cfg(feature = "occt")]
/// Env var: the capture directory of a `write_ahead_child` run.
const WRITE_AHEAD_DIR: &str = "AMG_KH01_WRITE_AHEAD_DIR";

/// The keys of a schema-1 manifest (design note §1.1).
const MANIFEST_KEYS: [&str; 20] = [
    "schema",
    "hash",
    "amalgam_version",
    "git_sha",
    "occt_version",
    "kernel_version",
    "policy_version",
    "os",
    "created_utc",
    "last_utc",
    "mode",
    "duration_ms",
    "count",
    "tolerance",
    "retry_policy",
    "op",
    "inputs",
    "error",
    "attempts",
    "context",
];

/// A fresh directory under cargo's per-target temp dir (not `%TEMP%`).
fn scratch(name: &str) -> PathBuf {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR"))
        .join("kh01")
        .join(format!("{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn recorder(dir: &Path, mode: CaptureMode) -> Arc<FailureRecorder> {
    Arc::new(
        FailureRecorder::new(CaptureConfig {
            mode,
            ..CaptureConfig::in_dir(dir)
        })
        .unwrap(),
    )
}

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}

/// Checks a manifest against schema 1: the exact key set, the typed strict parse, and the
/// input files.
fn assert_schema_1(dump: &Path) -> amg_brep::capture::Manifest {
    let text = std::fs::read_to_string(dump.join("manifest.json")).unwrap();
    let value: serde_json::Value = serde_json::from_str(&text).unwrap();
    let keys: BTreeSet<&str> = value
        .as_object()
        .unwrap()
        .keys()
        .map(String::as_str)
        .collect();
    assert_eq!(keys, MANIFEST_KEYS.into_iter().collect(), "{text}");
    assert_eq!(value["schema"], 1);
    let m = read_manifest(dump).unwrap();
    assert_eq!(m.hash.len(), 64);
    assert!(m.count >= 1);
    assert_eq!(m.op.inputs().len(), m.inputs.len());
    for f in &m.inputs {
        assert!(dump.join(f).is_file(), "{f} missing in {}", dump.display());
    }
    let name = dump.file_name().unwrap().to_string_lossy().into_owned();
    assert!(name.contains(m.op.kind()), "{name}");
    assert!(name.contains(&m.hash[..8]), "{name}");
    m
}

/// Dedup and retention through `FailureRecorder::record` (no shapes, so no backend needed).
#[test]
fn dedup_counts_repeats_and_retention_prunes_oldest() {
    let dir = scratch("dedup");
    let rec = FailureRecorder::new(CaptureConfig {
        max_dumps: 3,
        ..CaptureConfig::in_dir(&dir)
    })
    .unwrap();
    let op = |x: f64| KernelOp::MakeBox {
        frame: Plane::XY,
        size: Vec3::new(x, 1.0, 1.0),
    };
    let info = FailureInfo::new(FailureClass::InvalidInput, "size");
    let a = rec.record(&op(0.0), &[], &info, &[]).unwrap();
    let b = rec.record(&op(0.0), &[], &info, &[]).unwrap();
    assert_eq!(a, b);
    assert_eq!(rec.dumps().unwrap().len(), 1);
    assert_eq!(read_manifest(&dir.join(&a)).unwrap().count, 2);
    // A new recorder (a restart) finds the dump again.
    let again = FailureRecorder::new(CaptureConfig::in_dir(&dir)).unwrap();
    assert_eq!(again.record(&op(0.0), &[], &info, &[]).unwrap(), a);
    assert_eq!(read_manifest(&dir.join(&a)).unwrap().count, 3);
    let m = assert_schema_1(&dir.join(&a));
    assert_eq!(m.op, op(0.0));

    // Five more distinct failures with max_dumps = 3: the oldest go first.
    let mut ids = vec![a];
    for i in 1..=5 {
        std::thread::sleep(Duration::from_millis(30));
        ids.push(rec.record(&op(-f64::from(i)), &[], &info, &[]).unwrap());
    }
    let left: Vec<String> = rec
        .dumps()
        .unwrap()
        .iter()
        .map(|d| d.file_name().unwrap().to_string_lossy().into_owned())
        .collect();
    let mut want = ids[3..].to_vec();
    want.sort();
    assert_eq!(left, want);
    // A repeat of a pruned failure is a new dump.
    std::thread::sleep(Duration::from_millis(30));
    let back = rec.record(&op(0.0), &[], &info, &[]).unwrap();
    assert_eq!(read_manifest(&dir.join(&back)).unwrap().count, 1);
    assert_eq!(rec.dumps().unwrap().len(), 3);

    // max_bytes: room for about two dumps.
    let size = |d: &Path| {
        std::fs::read_dir(d)
            .unwrap()
            .flatten()
            .map(|e| e.metadata().unwrap().len())
            .sum::<u64>()
    };
    let one = size(&dir.join(&back));
    let bytes_dir = scratch("dedup-bytes");
    let small = FailureRecorder::new(CaptureConfig {
        max_bytes: 2 * one + one / 2,
        ..CaptureConfig::in_dir(&bytes_dir)
    })
    .unwrap();
    let mut last = String::new();
    for i in 0..6 {
        std::thread::sleep(Duration::from_millis(30));
        last = small
            .record(&op(-100.0 - f64::from(i)), &[], &info, &[])
            .unwrap();
    }
    let dumps = small.dumps().unwrap();
    let total: u64 = dumps.iter().map(|d| size(d)).sum();
    eprintln!(
        "max_bytes {} -> {} dumps, {total} bytes",
        small.config().max_bytes,
        dumps.len()
    );
    assert!(total <= small.config().max_bytes);
    assert_eq!(dumps.len(), 2);
    assert!(dumps.iter().any(|d| d.ends_with(&last)));
}

/// Done-when 6, I/O part (any build): successful calls touch no file, and `Off` never writes.
#[test]
fn successful_calls_do_no_io() {
    let dir = scratch("no-io").join("capture");
    let k = RecordingKernel::new(
        amg_brep::OcctKernel::new(amg_brep::KernelConfig::default()),
        recorder(&dir, CaptureMode::OnFailure),
    );
    for _ in 0..100_000 {
        let r: amg_brep::KResult<u32> = k.guarded(
            || unreachable!("the op is only built on failure"),
            || unreachable!("the inputs are only collected on failure"),
            || Ok(7),
        );
        assert_eq!(r.unwrap(), 7);
    }
    assert!(!dir.exists(), "a successful call created {}", dir.display());
    // Cancelled and BackendUnavailable are not failures.
    let r: amg_brep::KResult<()> = k.guarded(
        || unreachable!(),
        || unreachable!(),
        || Err(amg_brep::KernelError::Cancelled),
    );
    assert!(r.is_err());
    assert!(!dir.exists());
    let off = RecordingKernel::new(
        amg_brep::OcctKernel::new(amg_brep::KernelConfig::default()),
        recorder(&dir, CaptureMode::Off),
    );
    let r: amg_brep::KResult<()> = off.guarded(
        || unreachable!(),
        || unreachable!(),
        || Err(amg_brep::KernelError::NonRigid),
    );
    assert!(r.is_err());
    assert!(!dir.exists());
}

/// Done-when 6, timing part: decorator overhead per successful call. Printed always, asserted
/// (< 1 µs) only with `AMG_STRICT_TIMING=1`; the budget number is the criterion bench.
#[test]
fn on_failure_overhead_is_below_a_microsecond() {
    let dir = scratch("overhead");
    let k = RecordingKernel::new(
        amg_brep::OcctKernel::new(amg_brep::KernelConfig::default()),
        recorder(&dir, CaptureMode::OnFailure),
    );
    const N: u32 = 1_000_000;
    let mut best = f64::MAX;
    for _ in 0..5 {
        let start = Instant::now();
        for i in 0..N {
            let r: amg_brep::KResult<u32> = k.guarded(
                || unreachable!(),
                || unreachable!(),
                || Ok(std::hint::black_box(i)),
            );
            std::hint::black_box(r.ok());
        }
        let wrapped = start.elapsed().as_secs_f64();
        let start = Instant::now();
        for i in 0..N {
            let r: amg_brep::KResult<u32> = Ok(std::hint::black_box(i));
            std::hint::black_box(r.ok());
        }
        let direct = start.elapsed().as_secs_f64();
        best = best.min((wrapped - direct) / f64::from(N));
    }
    eprintln!(
        "RecordingKernel OnFailure overhead: {:.1} ns per call",
        best * 1e9
    );
    if strict_timing() {
        assert!(best < 1e-6, "{best} s per call");
    }
}

#[cfg(feature = "occt")]
mod occt {
    use super::*;
    use crate::fault::FaultKernel;

    use amg_brep::capture::{
        EdgeRecord, ReplayOpts, ReplayOutcome, SubShapeGeom, child_report_line, promote, replay,
        replay_in_child,
    };
    use amg_brep::geom::{
        Curve2d, Curve3, CurveSeg, FaceBounds, Loop2, PlanarProfile, ProfileEdge2, Surface, Tag,
    };
    use amg_brep::{
        BoolOp, BoolOpts, Dir3, EdgeId, JobCtx, KernelConfig, KernelError, Line3, OcctKernel,
        Point2, Point3, PrismExtent, PrismOpts, Shape, SubKind, SubShape, TessQuality, Tolerance,
        Transform, Validate,
    };

    const TOL: Tolerance = Tolerance::DEFAULT;

    fn occt() -> OcctKernel {
        OcctKernel::new(KernelConfig {
            validate: Validate::Always,
            ..KernelConfig::default()
        })
    }

    fn faulty(abort: bool) -> FaultKernel<OcctKernel> {
        FaultKernel {
            inner: occt(),
            abort,
        }
    }

    fn ctx() -> JobCtx {
        JobCtx::detached()
    }

    fn at(x: f64, y: f64, z: f64) -> Plane {
        Plane {
            origin: Point3::new(x, y, z),
            ..Plane::XY
        }
    }

    fn block(k: &dyn BrepKernel, x: f64, size: f64) -> Shape {
        k.make_box(&at(x, 0.0, 0.0), Vec3::splat(size))
            .unwrap()
            .shape
    }

    fn square(k: &dyn BrepKernel, x0: f64, z: f64, s: f64) -> Shape {
        let pts = [
            Point3::new(x0, 0.0, z),
            Point3::new(x0 + s, 0.0, z),
            Point3::new(x0 + s, s, z),
            Point3::new(x0, s, z),
        ];
        k.make_planar_polygon(&pts, &[], &TOL).unwrap()
    }

    fn line(a: Point2, b: Point2) -> ProfileEdge2 {
        ProfileEdge2 {
            curve: Curve2d::Line { a, b },
            tag: Tag(1),
        }
    }

    fn bad_plane() -> Plane {
        // Both axes are unit vectors, so no debug assertion fires, but x is parallel to the
        // normal: the kernel's plane check rejects it.
        Plane {
            x: Dir3::Z,
            ..Plane::XY
        }
    }

    /// Makes one failing call per `KernelOp` variant through `k` (19 calls), returning the
    /// variant names in call order. Every call must fail.
    fn fail_every_op(k: &dyn BrepKernel) -> Vec<&'static str> {
        let base = occt();
        let solid = block(&base, 0.0, 10.0);
        let other = block(&base, 5.0, 10.0);
        let face = square(&base, 0.0, 20.0, 5.0);
        let frame = Plane::XY;
        let mut ops = Vec::new();
        let mut expect_err = |name: &'static str, failed: bool| {
            assert!(failed, "{name} did not fail");
            ops.push(name);
        };
        // An open loop (the last edge stops short).
        let open = PlanarProfile {
            plane: frame,
            outer: Loop2 {
                edges: vec![
                    line(Point2::new(0.0, 0.0), Point2::new(4.0, 0.0)),
                    line(Point2::new(4.0, 0.0), Point2::new(4.0, 4.0)),
                    line(Point2::new(4.0, 4.0), Point2::new(0.0, 3.0)),
                ],
            },
            holes: Vec::new(),
        };
        expect_err(
            "make_planar_faces",
            k.make_planar_faces(&[open], &TOL).is_err(),
        );
        let skew = [
            Point3::ZERO,
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.5),
            Point3::new(0.0, 1.0, 0.0),
        ];
        expect_err(
            "make_planar_polygon",
            k.make_planar_polygon(&skew, &[], &TOL).is_err(),
        );
        expect_err(
            "make_face",
            k.make_face(&Surface::Plane(frame), &FaceBounds::Natural, 1e-6)
                .is_err(),
        );
        let seg = |x0: f64, x1: f64| CurveSeg {
            curve: Curve3::Line {
                origin: Point3::ZERO,
                dir: Dir3::new_unchecked(Vec3::X),
            },
            t0: x0,
            t1: x1,
            tag: Tag(2),
        };
        expect_err(
            "make_wire",
            k.make_wire(&[seg(0.0, 1.0), seg(1.5, 2.0)], &TOL).is_err(),
        );
        expect_err(
            "make_box",
            k.make_box(&frame, Vec3::new(0.0, 1.0, 1.0)).is_err(),
        );
        expect_err(
            "make_cylinder",
            k.make_cylinder(&frame, -1.0, 2.0, std::f64::consts::TAU)
                .is_err(),
        );
        expect_err(
            "make_cone",
            k.make_cone(&frame, 2.0, 2.0, 3.0, std::f64::consts::TAU)
                .is_err(),
        );
        expect_err("make_sphere", k.make_sphere(&frame, 0.0).is_err());
        expect_err(
            "make_torus",
            k.make_torus(&frame, 1.0, 2.0, std::f64::consts::TAU)
                .is_err(),
        );
        expect_err(
            "prism",
            k.prism(
                &face,
                &PrismExtent::Vector(Vec3::ZERO),
                &PrismOpts {
                    taper: 0.0,
                    merge_coplanar: false,
                },
                &ctx(),
            )
            .is_err(),
        );
        // A full turn of a square crossing its axis: the result intersects itself.
        let crossing = square(&base, -2.0, 0.0, 4.0);
        let axis = Line3 {
            point: Point3::ZERO,
            dir: Dir3::new_unchecked(Vec3::Y),
        };
        expect_err(
            "revol",
            k.revol(&crossing, &axis, std::f64::consts::TAU, &ctx())
                .is_err(),
        );
        expect_err("mirrored", k.mirrored(&solid, &frame).is_err());
        // A fuse of a solid and a face: BOPAlgo_AlertBOPNotAllowed.
        expect_err(
            "boolean",
            k.boolean(
                BoolOp::Fuse,
                std::slice::from_ref(&solid),
                std::slice::from_ref(&face),
                &BoolOpts::default(),
                &ctx(),
            )
            .is_err(),
        );
        expect_err(
            "split",
            k.split(
                &[],
                std::slice::from_ref(&other),
                &BoolOpts::default(),
                &ctx(),
            )
            .is_err(),
        );
        expect_err("section", k.section(&solid, &other, &ctx()).is_err());
        expect_err(
            "interference",
            k.interference(&face, &solid, &ctx()).is_err(),
        );
        expect_err(
            "plane_section",
            k.plane_section(&solid, &bad_plane()).is_err(),
        );
        expect_err(
            "project_edges",
            k.project_edges(&solid, &[EdgeId(0), EdgeId(5), EdgeId(11)], &bad_plane())
                .is_err(),
        );
        let bad_quality = TessQuality {
            linear: -1.0,
            ..TessQuality::display()
        };
        expect_err(
            "tessellate",
            k.tessellate(&solid, &bad_quality, &ctx()).is_err(),
        );
        ops
    }

    /// Runs `fail_every_op` through a recording kernel into `dir` and returns the dumps.
    fn record_every_op(dir: &Path) -> Vec<PathBuf> {
        let k = RecordingKernel::new(faulty(false), recorder(dir, CaptureMode::OnFailure))
            .with_tolerance(TOL);
        let ops = fail_every_op(&k);
        let dumps = k.recorder().dumps().unwrap();
        assert_eq!(dumps.len(), ops.len(), "{dumps:?}");
        dumps
    }

    /// Done-when 1.
    #[test]
    fn forced_failures_write_schema_1_dumps() {
        let dir = scratch("forced");
        let dumps = record_every_op(&dir);
        let mut kinds = BTreeSet::new();
        for d in &dumps {
            let m = assert_schema_1(d);
            kinds.insert(m.op.kind());
            assert_eq!(m.mode, CaptureMode::OnFailure);
            assert_eq!(m.tolerance, Some(TOL));
            assert_eq!(m.kernel_version, amg_brep::kernel_version());
            assert!(
                m.occt_version.starts_with("occt-8.0.1"),
                "{}",
                m.occt_version
            );
            for f in &m.inputs {
                let file = std::fs::File::open(d.join(f)).unwrap();
                Shape::read_text(std::io::BufReader::new(file)).unwrap();
            }
            eprintln!(
                "{:>20} {:?}: {}",
                m.op.kind(),
                m.error.class,
                m.error.message
            );
            match m.op.kind() {
                // The three Done-when 1 cases.
                "boolean" => {
                    assert_eq!(m.error.class, FailureClass::NotDone);
                    assert!(m.error.message.contains("BOPAlgo_AlertBOPNotAllowed"));
                    assert_eq!(m.inputs.len(), 2);
                }
                "mirrored" | "section" => {
                    assert_eq!(m.error.class, FailureClass::NotDone);
                    assert!(m.error.message.contains("injected NotDone"));
                }
                // OCCT 8.0.1 refuses the self-intersecting revolution outright.
                "revol" => assert_eq!(m.error.class, FailureClass::OcctException),
                _ => assert_eq!(m.error.class, FailureClass::InvalidInput),
            }
        }
        assert_eq!(kinds.len(), 19, "{kinds:?}");
    }

    /// Replays `dump` in a child process of this test binary.
    fn child_replay(dump: &Path, abort: bool) -> amg_brep::capture::ReplayReport {
        let mut cmd = Command::new(std::env::current_exe().unwrap());
        cmd.args([
            "--exact",
            "occt::replay_child",
            "--nocapture",
            "--test-threads=1",
        ])
        .env(REPLAY_DUMP, dump)
        .env(REPLAY_ABORT, if abort { "1" } else { "0" });
        replay_in_child(dump, cmd).unwrap()
    }

    /// The child side of `child_replay`.
    #[test]
    fn replay_child() {
        let Some(dump) = std::env::var_os(REPLAY_DUMP) else {
            return;
        };
        let abort = std::env::var(REPLAY_ABORT).is_ok_and(|v| v == "1");
        let report = replay(Path::new(&dump), &faulty(abort), &ReplayOpts::validating()).unwrap();
        println!("{}", child_report_line(&report));
    }

    /// Done-when 2: every dump of the 19-op set replays as `Reproduced` in a fresh process.
    #[test]
    fn every_dump_reproduces_in_a_fresh_process() {
        let dir = scratch("replay");
        let dumps = record_every_op(&dir);
        assert!(dumps.len() >= 10);
        let mut reproduced = 0;
        for d in &dumps {
            let report = child_replay(d, false);
            let m = read_manifest(d).unwrap();
            match &report.outcome {
                ReplayOutcome::Reproduced(info) => {
                    assert_eq!(info.class, m.error.class);
                    reproduced += 1;
                }
                other => panic!("{}: {other:?}", d.display()),
            }
            assert!(!report.reindexed_by_fingerprint);
        }
        eprintln!(
            "{reproduced}/{} dumps reproduced in a child process",
            dumps.len()
        );
        assert_eq!(reproduced, dumps.len());
    }

    /// A panic in the op is recorded as `Panic` and then resumed.
    #[test]
    fn panics_are_recorded_and_resumed() {
        let dir = scratch("panic");
        let k = RecordingKernel::new(faulty(false), recorder(&dir, CaptureMode::OnFailure));
        let caught = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            k.make_torus(&Plane::XY, 42.0, 1.0, 1.0)
        }));
        assert!(caught.is_err(), "the panic must propagate");
        let dumps = k.recorder().dumps().unwrap();
        assert_eq!(dumps.len(), 1);
        let m = assert_schema_1(&dumps[0]);
        assert_eq!(m.error.class, FailureClass::Panic);
        assert!(
            m.error.message.contains("injected panic"),
            "{}",
            m.error.message
        );
        let report = replay(&dumps[0], &faulty(false), &ReplayOpts::default()).unwrap();
        assert!(
            matches!(report.outcome, ReplayOutcome::Reproduced(_)),
            "{report:?}"
        );
        // The real kernel builds that torus.
        let report = replay(&dumps[0], &occt(), &ReplayOpts::default()).unwrap();
        assert_eq!(report.outcome, ReplayOutcome::NowSucceeds { valid: true });
    }

    /// Every face and edge fingerprint, in index order.
    fn fingerprints(s: &Shape) -> Vec<SubShapeGeom> {
        [SubKind::Face, SubKind::Edge]
            .into_iter()
            .flat_map(|kind| (0..s.count(kind)).map(move |index| SubShape { kind, index }))
            .map(|sub| SubShapeGeom::of(s, sub).unwrap())
            .collect()
    }

    fn text_round_trip(s: &Shape) -> Shape {
        let mut bytes = Vec::new();
        s.write_text(&mut bytes).unwrap();
        Shape::read_text(bytes.as_slice()).unwrap()
    }

    /// The 20 "real" parts: the 11 `testdata/brep` fixtures plus 9 boolean/sweep results.
    /// (S20 asks for 20 STEP parts; there is no STEP reader before BREP-13, see the log.)
    fn real_parts(k: &dyn BrepKernel) -> Vec<(String, Shape)> {
        let mut parts = Vec::new();
        for name in [
            "box",
            "cylinder",
            "cone",
            "sphere",
            "torus",
            "filleted_box",
            "plate_with_holes",
            "located_compound",
            "bspline_patch",
            "revolved_profile",
            "cylinder_mirrored",
        ] {
            let Some(path) = amg_testkit::testdata(&format!("brep/{name}.brep")) else {
                continue;
            };
            let file = std::fs::File::open(path).unwrap();
            parts.push((
                name.to_owned(),
                Shape::read_text(std::io::BufReader::new(file)).unwrap(),
            ));
        }
        let c = ctx();
        let o = BoolOpts::default();
        let b = |x: f64, s: f64| block(k, x, s);
        let cyl = |x: f64, r: f64| {
            k.make_cylinder(&at(x, 5.0, -1.0), r, 12.0, std::f64::consts::TAU)
                .unwrap()
                .shape
        };
        let sphere = k.make_sphere(&at(10.0, 10.0, 10.0), 6.0).unwrap().shape;
        let torus = k
            .make_torus(&at(5.0, 5.0, 5.0), 4.0, 1.5, std::f64::consts::TAU)
            .unwrap()
            .shape;
        let run =
            |op: BoolOp, a: Shape, t: Vec<Shape>| k.boolean(op, &[a], &t, &o, &c).unwrap().shape;
        parts.push((
            "box-drilled".into(),
            run(BoolOp::Cut, b(0.0, 10.0), vec![cyl(5.0, 2.0)]),
        ));
        parts.push((
            "box-two-holes".into(),
            run(
                BoolOp::Cut,
                b(0.0, 10.0),
                vec![cyl(3.0, 1.0), cyl(7.0, 1.5)],
            ),
        ));
        parts.push((
            "boxes-fused".into(),
            run(BoolOp::Fuse, b(0.0, 10.0), vec![b(5.0, 8.0)]),
        ));
        parts.push((
            "box-sphere-common".into(),
            run(BoolOp::Common, b(0.0, 12.0), vec![sphere.clone()]),
        ));
        parts.push((
            "box-sphere-cut".into(),
            run(BoolOp::Cut, b(0.0, 12.0), vec![sphere]),
        ));
        parts.push((
            "box-torus-cut".into(),
            run(BoolOp::Cut, b(0.0, 10.0), vec![torus]),
        ));
        let prism = k
            .prism(
                &square(k, 0.0, 0.0, 4.0),
                &PrismExtent::Vector(Vec3::new(1.0, 2.0, 7.0)),
                &PrismOpts {
                    taper: 0.05,
                    merge_coplanar: false,
                },
                &c,
            )
            .unwrap()
            .shape;
        parts.push(("tapered-prism".into(), prism.clone()));
        let axis = Line3 {
            point: Point3::new(-1.0, 0.0, 0.0),
            dir: Dir3::new_unchecked(Vec3::Y),
        };
        parts.push((
            "revolved-square".into(),
            k.revol(&square(k, 0.0, 0.0, 3.0), &axis, 2.0, &c)
                .unwrap()
                .shape,
        ));
        parts.push((
            "mirrored-cut".into(),
            k.mirrored(
                &run(BoolOp::Cut, prism, vec![cyl(2.0, 1.0)]),
                &at(-3.0, 0.0, 0.0),
            )
            .unwrap()
            .shape,
        ));
        parts
    }

    /// 200 generated parts: random primitives, every second one cut, fused or intersected
    /// with another random primitive. Deterministic (fixed LCG seed).
    fn generated_parts(k: &dyn BrepKernel, n: usize) -> Vec<Shape> {
        let mut state: u64 = 0x5eed_0000_0000_0001;
        let mut rnd = move || {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            #[allow(clippy::cast_precision_loss, reason = "53 random bits")]
            let u = (state >> 11) as f64 / (1u64 << 53) as f64;
            u
        };
        let tau = std::f64::consts::TAU;
        let prim = |rnd: &mut dyn FnMut() -> f64| -> Option<Shape> {
            let frame = Plane {
                origin: Point3::new(rnd() * 6.0, rnd() * 6.0, rnd() * 6.0),
                ..Plane::XY
            };
            let rot = Transform::from_axis_angle(Vec3::new(rnd(), rnd(), 0.5).normalize(), rnd());
            let r = match (rnd() * 5.0) as u32 {
                0 => k.make_box(
                    &frame,
                    Vec3::new(2.0 + 6.0 * rnd(), 2.0 + 6.0 * rnd(), 2.0 + 6.0 * rnd()),
                ),
                1 => k.make_cylinder(&frame, 1.0 + 3.0 * rnd(), 2.0 + 6.0 * rnd(), tau),
                2 => k.make_cone(
                    &frame,
                    1.0 + 3.0 * rnd(),
                    0.5 * rnd(),
                    2.0 + 6.0 * rnd(),
                    tau,
                ),
                3 => k.make_sphere(&frame, 1.0 + 4.0 * rnd()),
                _ => k.make_torus(&frame, 3.0 + 2.0 * rnd(), 0.5 + rnd(), tau),
            };
            r.ok()?.shape.transformed(&rot).ok()
        };
        let mut parts = Vec::with_capacity(n);
        let mut tries = 0;
        while parts.len() < n {
            tries += 1;
            assert!(tries < 4 * n, "too many failed generations");
            let Some(a) = prim(&mut rnd) else { continue };
            if parts.len() % 2 == 0 {
                parts.push(a);
                continue;
            }
            let Some(b) = prim(&mut rnd) else { continue };
            let op = [BoolOp::Cut, BoolOp::Fuse, BoolOp::Common][parts.len() / 2 % 3];
            if let Ok(r) = k.boolean(op, &[a], &[b], &BoolOpts::default(), &ctx())
                && r.shape.count(SubKind::Face) > 0
            {
                parts.push(r.shape);
            }
        }
        parts
    }

    fn assert_indices_survive(name: &str, s: &Shape) -> usize {
        let before = fingerprints(s);
        let back = text_round_trip(s);
        for kind in [SubKind::Face, SubKind::Edge, SubKind::Vertex] {
            assert_eq!(s.count(kind), back.count(kind), "{name}: {kind:?} count");
        }
        let after = fingerprints(&back);
        let eps = 1e-9 * s.bbox().size().length().max(1.0);
        for (i, (a, b)) in before.iter().zip(&after).enumerate() {
            assert!(
                a.matches(b, eps),
                "{name}: sub-shape {i} moved: {a:?} vs {b:?}"
            );
        }
        before.len()
    }

    /// Done-when 3, first part.
    #[test]
    fn indices_survive_a_text_brep_round_trip() {
        let k = occt();
        let real = real_parts(&k);
        let mut subs = 0;
        for (name, s) in &real {
            subs += assert_indices_survive(name, s);
        }
        let generated = generated_parts(&k, 200);
        for (i, s) in generated.iter().enumerate() {
            subs += assert_indices_survive(&format!("generated-{i}"), s);
        }
        eprintln!(
            "{} real + {} generated parts, {subs} face/edge indices: all survive",
            real.len(),
            generated.len()
        );
        // 11 fixtures (skipped when they are un-fetched LFS pointers, which panics instead
        // with AMG_REQUIRE_TESTDATA=1) plus 9 built parts.
        assert!(real.len() >= 9 && real.len() <= 20, "{}", real.len());
        if real.len() < 20 {
            eprintln!(
                "{} of the 11 testdata/brep fixtures were skipped",
                20 - real.len()
            );
        }
        assert_eq!(generated.len(), 200);
    }

    /// Done-when 3, second part: a forced re-index (edge ids rotated in the manifest) falls back
    /// to fingerprints, restores the ids and reports `reindexed_by_fingerprint`.
    #[test]
    fn forced_reindex_falls_back_to_fingerprints() {
        let dir = scratch("reindex");
        let k = RecordingKernel::new(occt(), recorder(&dir, CaptureMode::OnFailure));
        let solid = block(&occt(), 0.0, 10.0);
        let edges = [EdgeId(0), EdgeId(5), EdgeId(11)];
        assert!(k.project_edges(&solid, &edges, &bad_plane()).is_err());
        let dumps = k.recorder().dumps().unwrap();
        let dump = &dumps[0];
        let report = replay(dump, &occt(), &ReplayOpts::default()).unwrap();
        assert!(matches!(report.outcome, ReplayOutcome::Reproduced(_)));
        assert!(!report.reindexed_by_fingerprint);

        // Rotate the stored ids: each now names another edge of the box.
        let path = dump.join("manifest.json");
        let mut value: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
        for e in value["op"]["edges"].as_array_mut().unwrap() {
            let id = e["id"].as_u64().unwrap();
            e["id"] = serde_json::json!((id + 1) % 12);
        }
        std::fs::write(&path, serde_json::to_string_pretty(&value).unwrap()).unwrap();
        let report = replay(dump, &occt(), &ReplayOpts::default()).unwrap();
        assert!(report.reindexed_by_fingerprint, "{report:?}");
        assert!(matches!(report.outcome, ReplayOutcome::Reproduced(_)));
        let mut op = read_manifest(dump).unwrap().op;
        assert!(op.reindex(std::slice::from_ref(&solid)).unwrap());
        let KernelOp::ProjectEdges { edges: got, .. } = op else {
            panic!("{op:?}")
        };
        let ids: Vec<EdgeId> = got.iter().map(|e: &EdgeRecord| e.id).collect();
        assert_eq!(ids, edges);

        // A fingerprint that matches nothing is an error, not a silent guess.
        value["op"]["edges"][0]["geom"]["measure"] = serde_json::json!(1234.5);
        std::fs::write(&path, serde_json::to_string_pretty(&value).unwrap()).unwrap();
        assert!(matches!(
            replay(dump, &occt(), &ReplayOpts::default()),
            Err(amg_brep::capture::CaptureError::Unmatched { .. })
        ));
    }

    /// The child side of Done-when 4: a write-ahead call that aborts inside `make_sphere`.
    #[test]
    fn write_ahead_child() {
        let Some(dir) = std::env::var_os(WRITE_AHEAD_DIR) else {
            return;
        };
        let k = RecordingKernel::new(
            faulty(true),
            recorder(Path::new(&dir), CaptureMode::WriteAhead),
        );
        let _ = k.make_sphere(&at(1.0, 2.0, 3.0), 4.0);
        unreachable!("make_sphere aborts");
    }

    /// Done-when 4.
    #[test]
    fn write_ahead_turns_a_crash_into_a_replayable_dump() {
        let dir = scratch("write-ahead");
        let out = Command::new(std::env::current_exe().unwrap())
            .args([
                "--exact",
                "occt::write_ahead_child",
                "--nocapture",
                "--test-threads=1",
            ])
            .env(WRITE_AHEAD_DIR, &dir)
            .output()
            .unwrap();
        eprintln!("child exit: {}", out.status);
        assert!(!out.status.success());
        let pending: Vec<PathBuf> = std::fs::read_dir(dir.join("pending"))
            .unwrap()
            .flatten()
            .map(|e| e.path())
            .collect();
        assert_eq!(pending.len(), 1, "{pending:?}");
        assert!(pending[0].join("manifest.json").is_file());

        // A pending dump whose lock is held (an op running in another process) stays.
        let busy = dir.join("pending").join("busy");
        copy_dir(&pending[0], &busy);
        let lock = std::fs::OpenOptions::new()
            .write(true)
            .open(busy.join("lock"))
            .unwrap();
        lock.lock().unwrap();

        let rec = FailureRecorder::new(CaptureConfig::in_dir(&dir)).unwrap();
        assert_eq!(rec.recover_pending().unwrap(), 1);
        assert!(!pending[0].exists());
        assert!(busy.exists());
        drop(lock);

        let dumps = rec.dumps().unwrap();
        assert_eq!(dumps.len(), 1);
        let m = assert_schema_1(&dumps[0]);
        assert_eq!(m.error.class, FailureClass::Crash);
        assert_eq!(m.mode, CaptureMode::WriteAhead);
        assert_eq!(m.op.kind(), "make_sphere");

        // It replays: in a child with the same crashing kernel the crash reproduces ...
        let report = child_replay(&dumps[0], true);
        match &report.outcome {
            ReplayOutcome::Reproduced(info) => assert_eq!(info.class, FailureClass::Crash),
            other => panic!("{other:?}"),
        }
        // ... and on the real kernel it now succeeds.
        let report = replay(&dumps[0], &occt(), &ReplayOpts::validating()).unwrap();
        assert_eq!(report.outcome, ReplayOutcome::NowSucceeds { valid: true });

        // The busy one is recovered once its lock is free; same op and inputs, so it is a
        // repeat of the first dump.
        assert_eq!(rec.recover_pending().unwrap(), 1);
        assert_eq!(rec.dumps().unwrap().len(), 1);
        assert_eq!(read_manifest(&dumps[0]).unwrap().count, 2);
    }

    fn copy_dir(from: &Path, to: &Path) {
        std::fs::create_dir_all(to).unwrap();
        for e in std::fs::read_dir(from).unwrap().flatten() {
            let target = to.join(e.file_name());
            if e.file_type().unwrap().is_dir() {
                copy_dir(&e.path(), &target);
            } else {
                std::fs::copy(e.path(), target).unwrap();
            }
        }
    }

    /// Write-ahead on calls that return: nothing is left in `pending/`; a failure becomes a
    /// normal dump.
    #[test]
    fn write_ahead_cleans_up_after_returning_calls() {
        let dir = scratch("write-ahead-ok");
        let k = RecordingKernel::new(occt(), recorder(&dir, CaptureMode::WriteAhead));
        let solid = block(&k, 0.0, 3.0);
        k.mirrored(&solid, &Plane::XY).unwrap();
        assert!(k.make_sphere(&Plane::XY, -1.0).is_err());
        let pending = std::fs::read_dir(dir.join("pending")).unwrap().count();
        assert_eq!(pending, 0);
        let dumps = k.recorder().dumps().unwrap();
        assert_eq!(dumps.len(), 1);
        let m = assert_schema_1(&dumps[0]);
        assert_eq!(m.error.class, FailureClass::InvalidInput);
        assert_eq!(m.mode, CaptureMode::WriteAhead);
    }

    /// Done-when 5 through the decorator: the same failing call twice is one dump, count 2.
    #[test]
    fn the_same_failure_twice_is_one_dump() {
        let dir = scratch("twice");
        let k = RecordingKernel::new(occt(), recorder(&dir, CaptureMode::OnFailure));
        let solid = block(&occt(), 0.0, 10.0);
        let face = square(&occt(), 0.0, 20.0, 5.0);
        for _ in 0..2 {
            let r = k.boolean(
                BoolOp::Fuse,
                std::slice::from_ref(&solid),
                std::slice::from_ref(&face),
                &BoolOpts::default(),
                &ctx(),
            );
            assert!(matches!(r, Err(KernelError::OpFailed { .. })), "{r:?}");
        }
        let dumps = k.recorder().dumps().unwrap();
        assert_eq!(dumps.len(), 1);
        assert_eq!(read_manifest(&dumps[0]).unwrap().count, 2);
        // A different input is a different dump.
        let moved = square(&occt(), 1.0, 20.0, 5.0);
        let _ = k.boolean(
            BoolOp::Fuse,
            std::slice::from_ref(&solid),
            std::slice::from_ref(&moved),
            &BoolOpts::default(),
            &ctx(),
        );
        assert_eq!(k.recorder().dumps().unwrap().len(), 2);
    }

    /// Done-when 6 on a real op: 1000 successful booleans through the decorator write nothing.
    #[test]
    fn successful_kernel_calls_write_nothing() {
        let dir = scratch("ok-calls").join("capture");
        let k = RecordingKernel::new(occt(), recorder(&dir, CaptureMode::OnFailure));
        let a = block(&k, 0.0, 10.0);
        let b = block(&k, 5.0, 10.0);
        for _ in 0..200 {
            k.boolean(
                BoolOp::Cut,
                std::slice::from_ref(&a),
                std::slice::from_ref(&b),
                &BoolOpts::default(),
                &ctx(),
            )
            .unwrap();
            k.make_box(&Plane::XY, Vec3::ONE).unwrap();
            k.tessellate(&a, &TessQuality::preview(), &ctx()).unwrap();
            k.plane_section(&a, &at(0.0, 0.0, 5.0)).unwrap();
            k.mirrored(&a, &Plane::XY).unwrap();
        }
        assert!(!dir.exists());
    }

    /// `promote` writes a corpus case whose `expect` comes from a replay.
    #[test]
    fn promote_writes_a_corpus_case() {
        let dir = scratch("promote");
        let dumps = record_every_op(&dir.join("capture"));
        let corpus = dir.join("corpus");
        let boolean = dumps
            .iter()
            .find(|d| d.to_string_lossy().contains("_boolean_"))
            .unwrap();
        let case = promote(boolean, &corpus, "bool/promoted/solid-face-fuse", &occt()).unwrap();
        assert_eq!(
            case,
            corpus.join("bool").join("promoted").join("solid-face-fuse")
        );
        let toml: toml::Table = std::fs::read_to_string(case.join("case.toml"))
            .unwrap()
            .parse()
            .unwrap();
        assert_eq!(toml["schema"].as_integer(), Some(1));
        assert_eq!(toml["op"].as_str(), Some("op.json"));
        assert!(toml["source"].as_str().unwrap().starts_with("promoted:"));
        assert_eq!(toml["expect"]["outcome"].as_str(), Some("clean_failure"));
        assert_eq!(toml["expect"]["failure_class"].as_str(), Some("NotDone"));
        let op: KernelOp =
            serde_json::from_str(&std::fs::read_to_string(case.join("op.json")).unwrap()).unwrap();
        assert_eq!(op.kind(), "boolean");
        assert!(case.join("inputs/0.brep").is_file() && case.join("inputs/1.brep").is_file());
        // The injected mirror failure succeeds on the real kernel: a `success` case.
        let mirrored = dumps
            .iter()
            .find(|d| d.to_string_lossy().contains("_mirrored_"))
            .unwrap();
        let case = promote(mirrored, &corpus, "misc/mirror", &occt()).unwrap();
        let toml: toml::Table = std::fs::read_to_string(case.join("case.toml"))
            .unwrap()
            .parse()
            .unwrap();
        assert_eq!(toml["expect"]["outcome"].as_str(), Some("success"));
        // Ids are checked, and an existing case is never overwritten.
        assert!(promote(mirrored, &corpus, "../escape", &occt()).is_err());
        assert!(promote(mirrored, &corpus, "misc/mirror", &occt()).is_err());
    }
}
