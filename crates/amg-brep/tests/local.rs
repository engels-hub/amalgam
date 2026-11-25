//! BREP-16: draft, defeature (delete face), replace face, split faces by a tool, split body (the
//! BREP-04 `split`) and single-face offsets (R-S04-5), with history.
//!
//! Done-when 1 (a 3° draft of the 4 box sides about the bottom plane: face angles 3° ± 1e-9 rad,
//! analytic volume), 2 (deleting a fillet face and a boss gives the pre-feature volume within
//! 1e-6), 3 (replacing the top face by an offset plane changes the height exactly), 4 (a face
//! split by a sketch wire gives 2 faces of the right areas), 5 (every corpus result is
//! BRepCheck-valid with complete history; the completeness test is also in `tests/history.rs`).
//! Goldens: `tests/golden/brep/{draft,defeature,replace,split,offset_faces}_*.toml`.
#![cfg(feature = "occt")]
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

mod common;
#[path = "common/local_corpus.rs"]
mod corpus;
#[path = "common/history.rs"]
mod history_check;

use std::path::PathBuf;

use amg_brep::geom::{GeomClass, Surface};
use amg_brep::{
    BrepKernel, DraftOpts, FaceId, InputSub, JobCtx, KernelConfig, KernelError, OcctKernel,
    OpResult, OpRole, Plane, Relation, Shape, SubKind, SubShape, Validate,
};
use amg_core::Dir3;

use corpus::{A, B, C, Case, LocalOp, block, face_at, p, sides, top};

/// Golden bound on the adaptive volume and area.
const REL: f64 = 1e-6;

fn kernel() -> OcctKernel {
    OcctKernel::new(KernelConfig {
        validate: Validate::Always,
        ..KernelConfig::default()
    })
}

fn ctx() -> JobCtx {
    JobCtx::detached()
}

fn bin(s: &Shape) -> Vec<u8> {
    let mut bytes = Vec::new();
    s.write_bin(&mut bytes).unwrap();
    bytes
}

fn face_sub(f: FaceId) -> SubShape {
    SubShape {
        kind: SubKind::Face,
        index: f.0,
    }
}

fn run_ok(k: &dyn BrepKernel, case: &Case) -> OpResult {
    case.run(k)
        .unwrap_or_else(|e| panic!("{}: {e} ({e:?})", case.name))
}

fn case<'a>(cases: &'a [Case], name: &str) -> &'a Case {
    cases.iter().find(|c| c.name == name).unwrap()
}

/// The faces input face `f` (operand 0) is `Modified` into.
fn modified(r: &OpResult, f: FaceId) -> Vec<u32> {
    r.history
        .of_input(InputSub {
            operand: 0,
            sub: face_sub(f),
        })
        .filter_map(|rel| match rel {
            Relation::Modified(out) => Some(out.iter().map(|s| s.index).collect::<Vec<_>>()),
            Relation::Kept(s) => Some(vec![s.index]),
            _ => None,
        })
        .flatten()
        .collect()
}

// ---- goldens -----------------------------------------------------------------------------

#[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
struct LocalGolden {
    tolerances: GoldenTolerances,
    metrics: LocalMetrics,
}

#[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
struct GoldenTolerances {
    /// Relative bound on volume and area.
    rel: f64,
}

#[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
struct LocalMetrics {
    /// Adaptive `mass_props` volume and area.
    volume: f64,
    area: f64,
    faces: u32,
    edges: u32,
    vertices: u32,
    /// One letter per face class in index order (`common::class_letter`).
    classes: String,
}

fn golden_path(case: &str) -> PathBuf {
    amg_testkit::golden_dir()
        .join("brep")
        .join(format!("{case}.toml"))
}

fn metrics(k: &OcctKernel, s: &Shape) -> LocalMetrics {
    let m = k.mass_props(s).unwrap();
    LocalMetrics {
        volume: m.volume,
        area: m.area,
        faces: s.count(SubKind::Face),
        edges: s.count(SubKind::Edge),
        vertices: s.count(SubKind::Vertex),
        classes: common::face_classes(s),
    }
}

/// Compares `s` with its golden; `AMG_BLESS=1` (never under `CI`) writes it instead.
#[track_caller]
fn check_golden(k: &OcctKernel, case: &str, s: &Shape) {
    let metrics = metrics(k, s);
    let path = golden_path(case);
    if std::env::var("AMG_BLESS").is_ok_and(|v| v == "1") {
        assert!(
            std::env::var_os("CI").is_none_or(|v| v.is_empty()),
            "blessing is refused under CI"
        );
        let golden = LocalGolden {
            tolerances: GoldenTolerances { rel: REL },
            metrics,
        };
        let text = format!(
            "# B-rep metrics of brep/{case} (S02 BREP-16). Written by `AMG_BLESS=1`.\n\n{}",
            toml::to_string(&golden).unwrap()
        );
        std::fs::write(&path, text).unwrap();
        return;
    }
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("{}: {e}; bless it with AMG_BLESS=1", path.display()));
    let golden: LocalGolden = toml::from_str(&text).unwrap();
    let want = &golden.metrics;
    assert_eq!(
        (
            metrics.faces,
            metrics.edges,
            metrics.vertices,
            &metrics.classes
        ),
        (want.faces, want.edges, want.vertices, &want.classes),
        "{case}: topology"
    );
    let rel = golden.tolerances.rel;
    for (what, got, golden) in [
        ("volume", metrics.volume, want.volume),
        ("area", metrics.area, want.area),
    ] {
        let err = (got - golden).abs() / golden.abs();
        assert!(
            err <= rel,
            "{case}: {what} {got} vs golden {golden} ({err:e} > {rel:e})"
        );
    }
}

/// Every corpus case gives valid solids matching its golden and its closed-form volume within
/// 1e-9 relative (Done-when 5: valid; the history half is
/// `every_result_has_a_complete_history`).
#[test]
fn golden_local_ops_match_the_reference_volumes() {
    let k = kernel();
    let cases = corpus::corpus(&k);
    assert!(cases.len() >= 25, "{} cases", cases.len());
    for case in &cases {
        let r = run_ok(&k, case);
        assert!(
            k.check(&r.shape, &Default::default()).valid,
            "{}: invalid",
            case.name
        );
        assert_eq!(r.shape.solids().len(), case.solids, "{}: solids", case.name);
        check_golden(&k, case.name, &r.shape);
        let volume = k.mass_props(&r.shape).unwrap().volume;
        let (want, rel) = case.analytic.unwrap();
        let err = (volume - want).abs() / want;
        eprintln!("{}: volume {volume} vs {want} ({err:.2e})", case.name);
        assert!(err <= rel, "{}: {volume} vs {want} ({err:e})", case.name);
    }
}

// ---- Done-when 1 -------------------------------------------------------------------------

/// The 4 sides of the block drafted 3° about the bottom plane, pull +z: each tilted face's
/// outward normal makes 90° − 3° with the pull (normal · pull = sin 3°) within 1e-9 rad; the
/// bottom keeps its size and the top shrinks by 2 C tan 3° each way; the volume is the
/// prismatoid's.
#[test]
fn a_three_degree_draft_tilts_the_sides_by_three_degrees() {
    let k = kernel();
    let b = block(&k);
    let angle = 3f64.to_radians();
    let opts = DraftOpts {
        pull: Dir3::Z,
        angle,
        neutral: Plane::XY,
    };
    let faces = sides(&k, &b);
    let r = k.draft(&b, &faces, &opts, &ctx()).unwrap();
    assert!(k.check(&r.shape, &Default::default()).valid);
    assert_eq!(r.shape.count(SubKind::Face), 6);
    let mut worst: f64 = 0.0;
    for &f in &faces {
        let out = modified(&r, f);
        assert_eq!(out.len(), 1, "face {f:?}");
        let g = k.sub_geom(&r.shape, face_sub(FaceId(out[0]))).unwrap();
        assert_eq!(g.class, GeomClass::Plane);
        let n = g.axis.unwrap();
        // Angle between the face and the pull direction.
        let measured = n.dot(Dir3::Z.get()).asin();
        worst = worst.max((measured - angle).abs());
    }
    eprintln!("worst face angle error {worst:e} rad");
    assert!(worst <= 1e-9, "{worst:e}");
    let t = angle.tan();
    let v = k.mass_props(&r.shape).unwrap().volume;
    let want =
        C / 6.0 * (A * B + 4.0 * (A - C * t) * (B - C * t) + (A - 2.0 * C * t) * (B - 2.0 * C * t));
    assert!((v - want).abs() / want <= 1e-12, "{v} vs {want}");
    assert!((v - corpus::drafted_block_volume(angle, 0.0)).abs() / want <= 1e-12);
    // The top face shrank symmetrically; the bottom did not move.
    let top_out = modified(&r, top(&k, &b));
    let g = k.sub_geom(&r.shape, face_sub(FaceId(top_out[0]))).unwrap();
    let want_top = (A - 2.0 * C * t) * (B - 2.0 * C * t);
    assert!(
        (g.measure - want_top).abs() <= 1e-9 * want_top,
        "{}",
        g.measure
    );
    // A face OCCT cannot draft (a sphere) is refused on that face.
    let sphere = k.make_sphere(&Plane::XY, 5.0).unwrap().shape;
    match k.draft(&sphere, &[FaceId(0)], &opts, &ctx()) {
        Err(KernelError::InvalidInput { offending, .. }) => {
            assert_eq!(offending, vec![face_sub(FaceId(0))]);
        }
        other => panic!("{other:?}"),
    }
}

// ---- Done-when 2 -------------------------------------------------------------------------

/// Deleting the fillet face of a filleted box and the faces of a boss gives the pre-feature
/// volume within 1e-6 (here within 1e-9), and the deleted faces are `Deleted`.
#[test]
fn deleting_a_fillet_and_a_boss_restores_the_volume() {
    let k = kernel();
    let cases = corpus::corpus(&k);
    for name in [
        "defeature_fillet",
        "defeature_boss",
        "defeature_chamfer",
        "defeature_through_hole",
        "defeature_blind_hole",
    ] {
        let c = case(&cases, name);
        let r = run_ok(&k, c);
        let v = k.mass_props(&r.shape).unwrap().volume;
        let before = k.mass_props(&c.input).unwrap().volume;
        let err = (v - A * B * C).abs() / (A * B * C);
        eprintln!("{name}: {before} -> {v} ({err:.2e})");
        assert!(err <= 1e-6, "{name}: {v}");
        assert_eq!(r.shape.count(SubKind::Face), 6, "{name}");
        let LocalOp::Defeature(faces) = &c.op else {
            unreachable!()
        };
        for &f in faces {
            let rel: Vec<&Relation> = r
                .history
                .of_input(InputSub {
                    operand: 0,
                    sub: face_sub(f),
                })
                .collect();
            assert_eq!(rel, vec![&Relation::Deleted], "{name}: face {f:?}");
        }
    }
    // A fillet face whose neighbours cannot close the gap: the whole top of a box (its sides
    // would have to meet above it) is refused naming it.
    let b = block(&k);
    let t = top(&k, &b);
    match k.defeature(&b, &[t], &ctx()) {
        Err(KernelError::LocalOpFailed { op, offending, .. }) => {
            assert_eq!(op, "defeature");
            assert_eq!(offending, vec![face_sub(t)]);
        }
        other => panic!("{other:?}"),
    }
}

// ---- Done-when 3 -------------------------------------------------------------------------

/// The top face replaced by the plane 3 mm above it: every vertex of the new top is at
/// z = C + 3 within 1e-9, the volume is A B (C + 3), the top is `Modified` into the one new
/// face, and its 4 edges into the new top's edges. Lowering the plane trims the body exactly.
#[test]
fn replacing_the_top_by_an_offset_plane_changes_the_height_exactly() {
    let k = kernel();
    let b = block(&k);
    let t = top(&k, &b);
    for dz in [3.0, -4.0, 0.5, 25.0] {
        let with = Surface::Plane(Plane {
            origin: p(0.0, 0.0, C + dz),
            ..Plane::XY
        });
        let r = k.replace_face(&b, t, &with, &ctx()).unwrap();
        assert!(k.check(&r.shape, &Default::default()).valid);
        assert_eq!(r.shape.count(SubKind::Face), 6);
        let zmax = r
            .shape
            .vertices()
            .map(|v| r.shape.vertex_point(v).z)
            .fold(f64::MIN, f64::max);
        assert!((zmax - (C + dz)).abs() <= 1e-9, "dz {dz}: {zmax}");
        let v = k.mass_props(&r.shape).unwrap().volume;
        let want = A * B * (C + dz);
        assert!((v - want).abs() / want <= 1e-12, "dz {dz}: {v} vs {want}");
        let new_top = modified(&r, t);
        assert_eq!(new_top.len(), 1, "dz {dz}");
        let g = k.sub_geom(&r.shape, face_sub(FaceId(new_top[0]))).unwrap();
        assert!((g.centroid - p(A / 2.0, B / 2.0, C + dz)).length() <= 1e-9);
        // The top's edges go to the new top's edges.
        let top_edges = b.face_edges(t).to_vec();
        let new_edges = r.shape.face_edges(FaceId(new_top[0])).to_vec();
        for e in top_edges {
            let out: Vec<u32> = r
                .history
                .of_input(InputSub {
                    operand: 0,
                    sub: SubShape {
                        kind: SubKind::Edge,
                        index: e.0,
                    },
                })
                .filter_map(|rel| match rel {
                    Relation::Modified(out) => {
                        Some(out.iter().map(|s| s.index).collect::<Vec<_>>())
                    }
                    _ => None,
                })
                .flatten()
                .collect();
            assert_eq!(out.len(), 1, "dz {dz}: edge {e:?} -> {out:?}");
            assert!(
                new_edges.iter().any(|n| n.0 == out[0]),
                "dz {dz}: edge {e:?}"
            );
        }
    }
}

// ---- Done-when 4 -------------------------------------------------------------------------

/// The top face split by a sketch line across it gives 2 faces of 5 × 16 and 15 × 16 mm²; by
/// a sketch circle inside it, the disc and the holed rest. The solid stays one valid solid of
/// the same volume, and only the top is split.
#[test]
fn splitting_a_face_by_a_sketch_wire_gives_two_faces_of_the_right_areas() {
    let k = kernel();
    let cases = corpus::corpus(&k);
    let b = block(&k);
    let t = top(&k, &b);
    for (name, want) in [
        ("split_faces_top_line", [15.0 * B, 5.0 * B]),
        (
            "split_faces_top_circle",
            [
                A * B - std::f64::consts::PI * 9.0,
                std::f64::consts::PI * 9.0,
            ],
        ),
        ("split_faces_top_by_plane", [12.0 * B, 8.0 * B]),
    ] {
        let c = case(&cases, name);
        let r = run_ok(&k, c);
        assert_eq!(r.shape.solids().len(), 1, "{name}");
        assert_eq!(r.shape.count(SubKind::Face), 7, "{name}");
        let pieces = modified(&r, t);
        assert_eq!(pieces.len(), 2, "{name}: {pieces:?}");
        for (i, &f) in pieces.iter().enumerate() {
            let area = k.sub_geom(&r.shape, face_sub(FaceId(f))).unwrap().measure;
            let err = (area - want[i]).abs() / want[i];
            eprintln!("{name}: piece {i} area {area} vs {} ({err:.1e})", want[i]);
            assert!(err <= 1e-9, "{name}: piece {i}: {area} vs {}", want[i]);
        }
        // Every other face is one face.
        for f in b.faces().filter(|&f| f != t) {
            assert_eq!(modified(&r, f).len(), 1, "{name}: face {f:?}");
        }
    }
    // A tool that misses the faces is refused.
    let away = corpus::line(&k, p(5.0, -2.0, C + 5.0), p(5.0, B + 2.0, C + 5.0));
    assert!(matches!(
        k.split_faces(&b, &[t], &away, &ctx()),
        Err(KernelError::InvalidInput { .. })
    ));
}

// ---- Done-when 5 -------------------------------------------------------------------------

/// Every corpus result is valid and has a complete history: every output face has a source or
/// a role, indices in range, operand counts matching (also in `tests/history.rs`).
#[test]
fn every_result_has_a_complete_history() {
    let k = kernel();
    let mut faces = 0;
    for c in corpus::corpus(&k) {
        let r = run_ok(&k, &c);
        assert!(k.check(&r.shape, &Default::default()).valid, "{}", c.name);
        if let Some(e) = history_check::completeness_error(&r, &c.operands()) {
            panic!("{}: {e}", c.name);
        }
        assert!(r.warnings.is_empty(), "{}: {:?}", c.name, r.warnings);
        faces += r.shape.count(SubKind::Face);
    }
    eprintln!("{faces} output faces");
}

/// The roles: single-face offsets label the moved faces `OffsetFace` only; a body split labels
/// its pieces `SplitPiece`; replace, draft, defeature and split faces need no role (every face
/// has a source).
#[test]
fn roles_follow_the_ops() {
    let k = kernel();
    let cases = corpus::corpus(&k);
    let c = case(&cases, "offset_faces_top_and_side");
    let r = run_ok(&k, c);
    let offset: Vec<_> = r
        .history
        .roles
        .iter()
        .filter(|(_, role)| *role == OpRole::OffsetFace)
        .collect();
    assert_eq!(offset.len(), 2, "{:?}", r.history.roles);
    let c = case(&cases, "split_body_face_set");
    let r = run_ok(&k, c);
    let pieces: std::collections::BTreeSet<u32> = r
        .history
        .roles
        .iter()
        .filter_map(|(_, role)| match role {
            OpRole::SplitPiece { index } => Some(*index),
            _ => None,
        })
        .collect();
    assert_eq!(pieces.into_iter().collect::<Vec<_>>(), vec![0, 1, 2]);
    for name in [
        "draft_box_sides",
        "defeature_boss",
        "replace_top_tilted",
        "split_faces_top_line",
    ] {
        let r = run_ok(&k, case(&cases, name));
        assert!(r.history.roles.is_empty(), "{name}: {:?}", r.history.roles);
    }
}

// ---- also ---------------------------------------------------------------------------------

#[test]
fn ten_runs_are_byte_identical() {
    let k = kernel();
    for case in corpus::corpus(&k) {
        let first = run_ok(&k, &case);
        let (bytes, history) = (bin(&first.shape), format!("{:?}", first.history));
        let input = bin(&case.input);
        for run in 1..10 {
            let again = run_ok(&k, &case);
            assert_eq!(bin(&again.shape), bytes, "{} run {run}: bytes", case.name);
            assert_eq!(
                format!("{:?}", again.history),
                history,
                "{} run {run}",
                case.name
            );
        }
        assert_eq!(bin(&case.input), input, "{}: the input changed", case.name);
    }
}

#[test]
fn shared_shapes_survive_concurrent_local_ops() {
    let k = kernel();
    let cases = corpus::corpus(&k);
    let expected: Vec<Vec<u8>> = cases.iter().map(|c| bin(&run_ok(&k, c).shape)).collect();
    let inputs: Vec<Vec<u8>> = cases.iter().map(|c| bin(&c.input)).collect();
    std::thread::scope(|scope| {
        for t in 0..8 {
            let (k, cases, expected) = (&k, &cases, &expected);
            scope.spawn(move || {
                for i in 0..cases.len() {
                    let j = (i + t * 3) % cases.len();
                    let r = run_ok(k, &cases[j]);
                    assert_eq!(bin(&r.shape), expected[j], "{}", cases[j].name);
                }
            });
        }
    });
    for (c, before) in cases.iter().zip(&inputs) {
        assert_eq!(&bin(&c.input), before, "{}: the input changed", c.name);
    }
}

#[test]
fn bad_inputs_are_invalid_input() {
    let k = kernel();
    let b = block(&k);
    let t = top(&k, &b);
    let what = |r: Result<OpResult, KernelError>| match r {
        Err(KernelError::InvalidInput { what, .. }) => what,
        other => panic!("{other:?}"),
    };
    let draft = |faces: &[FaceId], angle: f64, pull: Dir3| {
        k.draft(
            &b,
            faces,
            &DraftOpts {
                pull,
                angle,
                neutral: Plane::XY,
            },
            &ctx(),
        )
    };
    assert_eq!(what(draft(&[], 0.1, Dir3::Z)), "faces");
    assert_eq!(what(draft(&[t, t], 0.1, Dir3::Z)), "faces");
    assert_eq!(what(draft(&[FaceId(6)], 0.1, Dir3::Z)), "faces");
    assert_eq!(what(draft(&[t], 2.0, Dir3::Z)), "opts");
    assert_eq!(what(draft(&[t], f64::NAN, Dir3::Z)), "opts");
    assert_eq!(what(draft(&[t], 0.1, Dir3::X)), "opts");
    let all: Vec<FaceId> = b.faces().collect();
    assert_eq!(what(k.defeature(&b, &all, &ctx())), "faces");
    assert_eq!(what(k.defeature(&b, &[], &ctx())), "faces");
    let face = b.sub_shape(face_sub(t)).unwrap();
    assert_eq!(what(k.defeature(&face, &[FaceId(0)], &ctx())), "shape");
    let flat = Surface::Plane(Plane {
        origin: p(0.0, 0.0, C + 1.0),
        ..Plane::XY
    });
    assert_eq!(what(k.replace_face(&b, FaceId(6), &flat, &ctx())), "face");
    assert_eq!(
        what(k.replace_face(&face, FaceId(0), &flat, &ctx())),
        "shape"
    );
    // A plane the top's normals never meet (vertical, beside the box... parallel to them).
    let beside = Surface::Plane(corpus::frame(p(A + 5.0, 0.0, 0.0), Dir3::X, Dir3::Y));
    assert_eq!(what(k.replace_face(&b, t, &beside, &ctx())), "with");
    assert_eq!(what(k.offset_faces(&b, &[(t, 0.0)], &ctx())), "faces");
    assert_eq!(what(k.offset_faces(&b, &[], &ctx())), "faces");
    assert_eq!(
        what(k.offset_faces(&b, &[(t, 1.0), (t, 2.0)], &ctx())),
        "faces"
    );
    let vertex = b
        .sub_shape(SubShape {
            kind: SubKind::Vertex,
            index: 0,
        })
        .unwrap();
    assert_eq!(what(k.split_faces(&b, &[t], &vertex, &ctx())), "tool");
    assert_eq!(what(k.split_faces(&b, &[], &face, &ctx())), "faces");
}

#[test]
fn cancelled_jobs_do_not_run() {
    let k = kernel();
    let cases = corpus::corpus(&k);
    let job = JobCtx::detached();
    job.cancel_token().cancel();
    for c in &cases {
        let r = match &c.op {
            LocalOp::Draft(f, o) => k.draft(&c.input, f, o, &job),
            LocalOp::Defeature(f) => k.defeature(&c.input, f, &job),
            LocalOp::Replace(f, s) => k.replace_face(&c.input, *f, s, &job),
            LocalOp::SplitFaces(f) => k.split_faces(&c.input, f, c.tool.as_ref().unwrap(), &job),
            LocalOp::SplitBody => continue,
            LocalOp::OffsetFaces(f) => k.offset_faces(&c.input, f, &job),
        };
        assert!(
            matches!(r, Err(KernelError::Cancelled)),
            "{}: {r:?}",
            c.name
        );
    }
}

/// The ops run behind the S20 recording kernel and through `&dyn BrepKernel`.
#[test]
fn local_ops_run_behind_the_recording_kernel() {
    let dir = std::env::temp_dir().join(format!("amg-brep16-capture-{}", std::process::id()));
    let recorder = amg_brep::capture::FailureRecorder::new(amg_brep::capture::CaptureConfig {
        mode: amg_brep::capture::CaptureMode::OnFailure,
        dir: dir.clone(),
        max_bytes: 1 << 20,
        max_dumps: 1,
    })
    .unwrap();
    let rec = amg_brep::capture::RecordingKernel::new(kernel(), std::sync::Arc::new(recorder));
    let k: &dyn BrepKernel = &rec;
    for c in corpus::corpus(k) {
        let r = run_ok(k, &c);
        assert_eq!(r.shape.solids().len(), c.solids, "{}", c.name);
    }
    let b = block(k);
    let t = face_at(k, &b, p(A / 2.0, B / 2.0, C));
    assert!(k.offset_faces(&b, &[(t, 1.0)], &ctx()).is_ok());
    let _ = std::fs::remove_dir_all(dir);
}
