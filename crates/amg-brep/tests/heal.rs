//! BREP-10: healing and assembly (`heal`, `unify_same_domain`, `sew`, `make_solid`,
//! `make_compound`) and the `BOPAlgo_ArgumentAnalyzer` pass of `check`.
//!
//! Done-when 1 (a cube of six gapped polygon faces sews into a valid closed solid of the cube's
//! volume), 2 (the faces of 20 parts, shared or copied apart, sew back into solids of the same
//! volume and face count), 3 (`heal` turns 10 defect fixtures from BRepCheck-invalid into valid),
//! 4 (`unify_same_domain` after a fuse of two coplanar boxes gives 6 faces, and the history maps
//! the merged faces), 5 (`check` lists the expected problem kinds on the broken fixtures). The
//! history completeness of the corpus is in `tests/history.rs`, the concurrency stress in
//! `tests/stress.rs`.
#![cfg(feature = "occt")]
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

mod common;
#[path = "common/heal_corpus.rs"]
mod corpus;
#[path = "common/history.rs"]
mod history_check;

use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;
use std::time::{Duration, Instant};

use amg_brep::{
    BoolOp, BoolOpts, BrepKernel, CheckOpts, FaceId, HealOpts, InputSub, JobCtx, KernelConfig,
    KernelError, KernelWarning, OcctKernel, OpResult, Plane, Relation, SewOpts, Shape, ShapeKind,
    SubKind, SubShape, UnifyOpts, Validate, Vec3,
};

use corpus::{HEALED, defect, faces_apart, faces_of, heal_opts};
use history_check::completeness_error;

fn kernel() -> OcctKernel {
    OcctKernel::new(KernelConfig {
        validate: Validate::Always,
        ..KernelConfig::default()
    })
}

fn raw_kernel() -> OcctKernel {
    OcctKernel::new(KernelConfig {
        validate: Validate::Off,
        ..KernelConfig::default()
    })
}

fn ctx() -> JobCtx {
    JobCtx::detached()
}

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}

fn bin(s: &Shape) -> Vec<u8> {
    let mut bytes = Vec::new();
    s.write_bin(&mut bytes).unwrap();
    bytes
}

fn face(index: u32) -> SubShape {
    SubShape {
        kind: SubKind::Face,
        index,
    }
}

fn volume(k: &OcctKernel, s: &Shape) -> f64 {
    k.mass_props(s).unwrap().volume
}

fn rel(got: f64, want: f64) -> f64 {
    (got - want).abs() / want.abs()
}

fn valid(k: &OcctKernel, s: &Shape) -> bool {
    k.check(s, &CheckOpts::default()).valid
}

fn fixes(r: &OpResult) -> u32 {
    r.warnings
        .iter()
        .find_map(|w| match w {
            KernelWarning::Healed { fixes } => Some(*fixes),
            _ => None,
        })
        .unwrap_or(0)
}

// ---- goldens -----------------------------------------------------------------------------

/// Golden bound on the adaptive volume and area.
const REL: f64 = 1e-6;

#[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
struct Golden {
    tolerances: GoldenTolerances,
    metrics: Metrics,
}

#[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
struct GoldenTolerances {
    /// Relative bound on volume and area.
    rel: f64,
}

#[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
struct Metrics {
    /// Adaptive `mass_props` volume (0 without solids) and area.
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

/// Compares `s` with `tests/golden/brep/<case>.toml`; `AMG_BLESS=1` (never under `CI`) writes
/// it instead.
#[track_caller]
fn check_golden(k: &OcctKernel, case: &str, s: &Shape) {
    let m = k.mass_props(s).unwrap();
    let metrics = Metrics {
        volume: m.volume,
        area: m.area,
        faces: s.count(SubKind::Face),
        edges: s.count(SubKind::Edge),
        vertices: s.count(SubKind::Vertex),
        classes: common::face_classes(s),
    };
    let path = golden_path(case);
    if std::env::var("AMG_BLESS").is_ok_and(|v| v == "1") {
        assert!(
            std::env::var_os("CI").is_none_or(|v| v.is_empty()),
            "blessing is refused under CI"
        );
        let golden = Golden {
            tolerances: GoldenTolerances { rel: REL },
            metrics,
        };
        let text = format!(
            "# B-rep metrics of brep/{case} (S02 BREP-10). Written by `AMG_BLESS=1`.\n\n{}",
            toml::to_string(&golden).unwrap()
        );
        std::fs::write(&path, text).unwrap();
        return;
    }
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("{}: {e}; bless it with AMG_BLESS=1", path.display()));
    let golden: Golden = toml::from_str(&text).unwrap();
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
    for (what, got, golden) in [
        ("volume", metrics.volume, want.volume),
        ("area", metrics.area, want.area),
    ] {
        let err = if golden == 0.0 {
            got.abs()
        } else {
            rel(got, golden)
        };
        assert!(
            err <= golden_bound(&golden_tolerance(&text)),
            "{case}: {what} {got} vs golden {golden} ({err:e})"
        );
    }
}

fn golden_tolerance(text: &str) -> GoldenTolerances {
    toml::from_str::<Golden>(text).unwrap().tolerances
}

fn golden_bound(t: &GoldenTolerances) -> f64 {
    t.rel
}

// ---- Done-when 1 -------------------------------------------------------------------------

/// The six faces of a 10 mm cube, built as separate planar polygons whose edges are 1e-4 mm
/// apart, sew (tolerance 1e-3 mm) into one valid closed solid: 6 faces, 12 edges, 8 vertices,
/// volume 1000 within 1e-6 relative.
#[test]
fn sewing_a_gapped_cube_gives_a_closed_solid() {
    let k = kernel();
    let faces = corpus::gapped_cube(&k, 10.0, 1e-4);
    // The gaps are real: no two faces share a vertex, and the nearest corners of two faces are
    // 1e-4 mm apart.
    let corners: Vec<Vec<_>> = faces
        .iter()
        .map(|f| f.vertices().map(|v| f.vertex_point(v)).collect())
        .collect();
    let mut nearest = f64::INFINITY;
    for (i, a) in corners.iter().enumerate() {
        for b in &corners[i + 1..] {
            for pa in a {
                for pb in b {
                    nearest = nearest.min((*pa - *pb).length());
                }
            }
        }
    }
    assert!(nearest > 0.99e-4, "nearest corners {nearest}");
    let r = k
        .sew(
            &faces,
            &SewOpts {
                tol: 1e-3,
                ..SewOpts::default()
            },
            &ctx(),
        )
        .unwrap();
    assert_eq!(r.shape.kind(), ShapeKind::Solid);
    assert!(valid(&k, &r.shape));
    assert_eq!(
        history_check::counts(&r.shape),
        [6, 12, 8],
        "faces, edges, vertices"
    );
    let v = volume(&k, &r.shape);
    let err = rel(v, 1000.0);
    eprintln!(
        "gapped cube: volume {v} (rel err {err:e}), max tolerance {:e}, warnings {:?}",
        r.shape.max_tolerance(),
        r.warnings
    );
    assert!(err <= 1e-6, "volume {v} ({err:e})");
    assert!(matches!(
        r.warnings.as_slice(),
        [KernelWarning::ToleranceGrown { .. }]
    ));
    check_golden(&k, "sew_gapped_cube", &r.shape);
    // Each input face is `Modified` into one face of the solid, all distinct.
    let mut images = BTreeSet::new();
    for (operand, f) in faces.iter().enumerate() {
        let operand = u16::try_from(operand).unwrap();
        let rel: Vec<&Relation> = r
            .history
            .of_input(InputSub {
                operand,
                sub: face(0),
            })
            .collect();
        match rel.as_slice() {
            [Relation::Modified(out)] if out.len() == 1 => {
                assert!(images.insert(out[0]), "face {operand} shares an image");
            }
            other => panic!("face {operand}: {other:?}"),
        }
        assert_eq!(f.count(SubKind::Face), 1);
    }
    // Two edges merged into one: every input edge maps to one output edge, each output edge
    // has two sources.
    let mut sources: BTreeMap<u32, u32> = BTreeMap::new();
    for (input, relation) in &r.history.entries {
        if input.sub.kind == SubKind::Edge {
            let Relation::Modified(out) = relation else {
                panic!("{input:?}: {relation:?}");
            };
            assert_eq!(out.len(), 1, "{input:?}");
            *sources.entry(out[0].index).or_default() += 1;
        }
    }
    assert_eq!(sources.len(), 12);
    assert!(sources.values().all(|&n| n == 2), "{sources:?}");

    // Without `make_solid` the result is the closed shell; below the gap it stays open.
    let shell = k
        .sew(
            &faces,
            &SewOpts {
                tol: 1e-3,
                make_solid: false,
                ..SewOpts::default()
            },
            &ctx(),
        )
        .unwrap();
    assert_eq!(shell.shape.kind(), ShapeKind::Shell);
    match k.sew(
        &faces,
        &SewOpts {
            tol: 1e-5,
            ..SewOpts::default()
        },
        &ctx(),
    ) {
        Err(KernelError::LocalOpFailed {
            op: "sew",
            message,
            partial,
            ..
        }) => {
            assert!(message.contains("free edge"), "{message}");
            assert!(partial.is_some());
        }
        other => panic!("{other:?}"),
    }
}

// ---- Done-when 2 -------------------------------------------------------------------------

/// 20 parts exploded into their faces (`sub_shape`, still sharing edges; and each face copied
/// apart into new geometry) sew back into solids with the part's solid count, face count and
/// volume.
#[test]
fn sewing_exploded_parts_gives_them_back() {
    let k = kernel();
    let parts = corpus::parts(&k);
    assert!(parts.len() >= 20, "{} parts", parts.len());
    let mut worst: f64 = 0.0;
    for (name, part) in &parts {
        let want = volume(&k, part);
        for (variant, faces) in [("shared", faces_of(part)), ("apart", faces_apart(part))] {
            let started = Instant::now();
            let r = k
                .sew(&faces, &SewOpts::default(), &ctx())
                .unwrap_or_else(|e| panic!("{name} ({variant}): {e}"));
            let took = started.elapsed();
            assert!(valid(&k, &r.shape), "{name} ({variant})");
            assert_eq!(
                r.shape.solids().len(),
                part.solids().len(),
                "{name} ({variant}): solids"
            );
            assert_eq!(
                r.shape.count(SubKind::Face),
                part.count(SubKind::Face),
                "{name} ({variant}): faces"
            );
            let got = volume(&k, &r.shape);
            let err = rel(got, want);
            worst = worst.max(err);
            assert!(
                err <= 1e-9,
                "{name} ({variant}): volume {got} vs {want} ({err:e})"
            );
            // Every face operand is `Modified` into exactly one face, and no two share it.
            let mut images = BTreeSet::new();
            for operand in 0..faces.len() {
                let operand = u16::try_from(operand).unwrap();
                let out: Vec<SubShape> = r
                    .history
                    .of_input(InputSub {
                        operand,
                        sub: face(0),
                    })
                    .flat_map(|rel| match rel {
                        Relation::Modified(out) => out.to_vec(),
                        other => panic!("{name} ({variant}) face {operand}: {other:?}"),
                    })
                    .collect();
                assert_eq!(out.len(), 1, "{name} ({variant}) face {operand}");
                assert!(images.insert(out[0]), "{name} ({variant}) face {operand}");
            }
            if took > Duration::from_millis(200) {
                eprintln!("{name} ({variant}): {} faces in {took:?}", faces.len());
            }
        }
    }
    eprintln!(
        "{} parts sewn back, worst volume error {worst:e}",
        parts.len()
    );
}

/// Sewing orients the shell, and the solid bounds a finite volume even when every face came in
/// reversed (an inside-out box); `make_solid` of the sewn shell does the same.
#[test]
fn sewn_and_made_solids_face_outwards() {
    let k = kernel();
    let Some(inside_out) = defect("inside_out") else {
        return;
    };
    assert!(volume(&raw_kernel(), &inside_out) < 0.0);
    let faces = faces_of(&inside_out);
    let solid = k.sew(&faces, &SewOpts::default(), &ctx()).unwrap();
    assert!(rel(volume(&k, &solid.shape), 6000.0) <= 1e-12);
    let shell = k
        .sew(
            &faces,
            &SewOpts {
                make_solid: false,
                ..SewOpts::default()
            },
            &ctx(),
        )
        .unwrap()
        .shape;
    let made = k.make_solid(&shell).unwrap();
    assert_eq!(made.shape.kind(), ShapeKind::Solid);
    assert!(rel(volume(&k, &made.shape), 6000.0) <= 1e-12);
    assert!(completeness_error(&made, &[&shell]).is_none());
    // Two closed shells give a compound of two solids.
    let (a, b, _) = corpus::coplanar_fuse(&k);
    let moved = b.transformed(&amg_brep::Transform::from_translation(Vec3::new(
        5.0, 0.0, 0.0,
    )));
    let mut faces = faces_apart(&a);
    faces.extend(faces_apart(&moved.unwrap()));
    let shells = k
        .sew(
            &faces,
            &SewOpts {
                make_solid: false,
                ..SewOpts::default()
            },
            &ctx(),
        )
        .unwrap()
        .shape;
    let two = k.make_solid(&shells).unwrap();
    assert_eq!(two.shape.kind(), ShapeKind::Compound);
    assert_eq!(two.shape.solids().len(), 2);
}

// ---- Done-when 3 -------------------------------------------------------------------------

/// `heal` turns each of the 10 defect fixtures from BRepCheck-invalid into a valid shape of the
/// same kind and (for solids) volume, reports its fixes, and keeps a complete history.
#[test]
fn heal_fixes_the_defect_corpus() {
    let k = kernel();
    let raw = raw_kernel();
    for name in HEALED {
        let Some(s) = defect(name) else {
            return;
        };
        let before = raw.check(&s, &CheckOpts::default());
        assert!(!before.valid, "{name}: the fixture is valid");
        let r = k
            .heal(&s, &heal_opts(), &ctx())
            .unwrap_or_else(|e| panic!("{name}: {e} ({e:?})"));
        assert!(valid(&k, &r.shape), "{name}: still invalid");
        assert_eq!(r.shape.kind(), s.kind(), "{name}: kind");
        assert!(
            fixes(&r) > 0,
            "{name}: no fixes reported ({:?})",
            r.warnings
        );
        assert!(
            r.shape.max_tolerance() <= heal_opts().max_tol,
            "{name}: tolerance {}",
            r.shape.max_tolerance()
        );
        if let Some(e) = completeness_error(&r, &[&s]) {
            panic!("{name}: {e}");
        }
        if !s.solids().is_empty() {
            // The volume of the part the defect was made from (a broken shape measures wrong).
            let base = if name.contains("pcurve") || name.contains("lateral") {
                "cylinder"
            } else {
                "box"
            };
            let want = volume(&k, &corpus::fixture(base).unwrap());
            let err = rel(volume(&k, &r.shape), want);
            assert!(err <= 1e-6, "{name}: volume error {err:e}");
        }
        check_golden(&k, &format!("heal_{name}"), &r.shape);
        eprintln!(
            "{name}: {:?} -> valid, {} fixes, max tolerance {:e}",
            before.problem_kinds(),
            fixes(&r),
            r.shape.max_tolerance()
        );
    }
}

/// A valid shape heals into an equal one: same counts and volume, every sub-shape `Modified` 1:1.
/// The box needs no fix at all; on the torus and the filleted box ShapeFix rebuilds faces around
/// their seams and degenerated edges (1 and 18 sub-shapes) without moving any geometry.
#[test]
fn healing_a_valid_shape_changes_nothing() {
    let k = kernel();
    for name in ["box", "filleted_box", "torus"] {
        let Some(s) = corpus::fixture(name) else {
            return;
        };
        let r = k.heal(&s, &HealOpts::default(), &ctx()).unwrap();
        eprintln!(
            "{name}: warnings {:?}, max tolerance {:e} -> {:e}",
            r.warnings,
            s.max_tolerance(),
            r.shape.max_tolerance()
        );
        if name != "box" {
            // ShapeFix re-runs SameParameter on the approximated blend edges and resets their
            // tolerances: fixes, but no tolerance growth and no change of shape.
            assert!(
                r.warnings
                    .iter()
                    .all(|w| matches!(w, KernelWarning::Healed { .. })),
                "{name}: {:?}",
                r.warnings
            );
        } else {
            assert!(r.warnings.is_empty(), "{name}: {:?}", r.warnings);
        }
        assert_eq!(
            history_check::counts(&r.shape),
            history_check::counts(&s),
            "{name}"
        );
        assert!(rel(volume(&k, &r.shape), volume(&k, &s)) <= 1e-12);
        for (input, relation) in &r.history.entries {
            match relation {
                Relation::Modified(out) if out.len() == 1 && out[0].kind == input.sub.kind => {}
                other => panic!("{name} {input:?}: {other:?}"),
            }
        }
        assert_eq!(
            r.history.entries.len() as u32,
            history_check::counts(&s).iter().sum::<u32>()
        );
    }
}

/// `fix_orientation: false` leaves a reversed face reversed (the result stays invalid); the
/// small-edge pass merges the 3e-7 mm edge only when asked.
#[test]
fn heal_options_switch_their_fixes() {
    let raw = raw_kernel();
    let (Some(reversed), Some(small)) = (defect("reversed_face"), defect("small_edge")) else {
        return;
    };
    let keep = HealOpts {
        fix_orientation: false,
        ..heal_opts()
    };
    let r = raw.heal(&reversed, &keep, &ctx()).unwrap();
    assert!(!valid(&raw, &r.shape));
    assert!(matches!(
        kernel().heal(&reversed, &keep, &ctx()),
        Err(KernelError::InvalidResult { op: "heal", .. })
    ));
    let edges = |o: &HealOpts| {
        raw.heal(&small, o, &ctx())
            .unwrap()
            .shape
            .count(SubKind::Edge)
    };
    let merged = edges(&HealOpts {
        tol: 1e-6,
        ..heal_opts()
    });
    let kept = edges(&HealOpts {
        tol: 1e-6,
        fix_small_edges: false,
        ..heal_opts()
    });
    eprintln!("small edge: {kept} edges kept, {merged} after merging");
    assert_eq!(small.count(SubKind::Edge), 5);
    assert_eq!(merged, 4);
    assert_eq!(kept, 5);
}

// ---- Done-when 4 -------------------------------------------------------------------------

/// Two coplanar boxes fused give 10 faces; `unify_same_domain` merges them into the 6 faces of
/// the 20 × 10 × 10 block. Each merged face is `Modified` from both coplanar pieces, and the
/// composed fuse + unify history maps each box's side faces to the merged ones.
#[test]
fn unify_after_fusing_coplanar_boxes_gives_six_faces() {
    let k = kernel();
    let (a, b, fused) = corpus::coplanar_fuse(&k);
    assert_eq!(fused.shape.count(SubKind::Face), 10);
    let r = k
        .unify_same_domain(&fused.shape, &UnifyOpts::default())
        .unwrap();
    assert_eq!(history_check::counts(&r.shape), [6, 12, 8]);
    assert!(rel(volume(&k, &r.shape), 2000.0) <= 1e-12);
    check_golden(&k, "unify_coplanar_fuse", &r.shape);
    assert!(completeness_error(&r, &[&fused.shape]).is_none());

    // unify's own history: 8 side pieces into 4 merged faces, the 2 ends 1:1.
    let mut pieces: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
    for f in 0..10 {
        let out: Vec<SubShape> = r
            .history
            .of_input(InputSub {
                operand: 0,
                sub: face(f),
            })
            .flat_map(|rel| match rel {
                Relation::Modified(out) => out.to_vec(),
                other => panic!("fuse face {f}: {other:?}"),
            })
            .collect();
        assert_eq!(out.len(), 1, "fuse face {f}");
        pieces.entry(out[0].index).or_default().push(f);
    }
    let mut sizes: Vec<usize> = pieces.values().map(Vec::len).collect();
    sizes.sort_unstable();
    assert_eq!(sizes, [1, 1, 2, 2, 2, 2]);

    // Composed: box faces → block faces. Every face of a box but the shared one reaches the
    // block; each merged face has one source in each box, each end face one.
    let composed = fused.history.compose(&r.history);
    let mut sources: BTreeMap<u32, BTreeSet<(u16, u32)>> = BTreeMap::new();
    for (operand, box_) in [(0u16, &a), (1, &b)] {
        let mut reached = 0;
        for f in box_.faces() {
            for relation in composed.of_input(InputSub {
                operand,
                sub: face(f.0),
            }) {
                if let Relation::Modified(out) = relation {
                    for o in out.iter().filter(|o| o.kind == SubKind::Face) {
                        sources.entry(o.index).or_default().insert((operand, f.0));
                        reached += 1;
                    }
                }
            }
        }
        assert_eq!(reached, 5, "box {operand}");
    }
    let mut both = 0;
    for (out, from) in &sources {
        let operands: BTreeSet<u16> = from.iter().map(|(o, _)| *o).collect();
        match from.len() {
            2 => {
                assert_eq!(operands.len(), 2, "block face {out}: {from:?}");
                both += 1;
            }
            1 => {}
            _ => panic!("block face {out}: {from:?}"),
        }
    }
    assert_eq!((sources.len(), both), (6, 4));

    // The default options are `merge_coplanar`'s: the same result as `unify_coplanar`.
    let same = k.unify_coplanar(&fused.shape).unwrap();
    assert_eq!(same.history, r.history);
    assert_eq!(
        k.sub_geom_all(&same.shape).unwrap(),
        k.sub_geom_all(&r.shape).unwrap()
    );
    // Faces only: the collinear edges stay split (20 edges of the fuse's merged faces).
    let faces_only = k
        .unify_same_domain(
            &fused.shape,
            &UnifyOpts {
                edges: false,
                ..UnifyOpts::default()
            },
        )
        .unwrap();
    assert_eq!(faces_only.shape.count(SubKind::Face), 6);
    assert!(faces_only.shape.count(SubKind::Edge) > 12);
}

// ---- Done-when 5 -------------------------------------------------------------------------

/// `check` lists the problem kinds each broken fixture was built with, on the sub-shape that
/// carries the defect; with `self_interference` the analyzer adds its findings (and finds the
/// interference BRepCheck cannot see). Valid parts stay valid with the analyzer on.
#[test]
fn check_lists_the_expected_problem_kinds() {
    let k = raw_kernel();
    let edge = |index| SubShape {
        kind: SubKind::Edge,
        index,
    };
    let vertex = |index| SubShape {
        kind: SubKind::Vertex,
        index,
    };
    // (fixture, self_interference, expected kinds, the defect's own (sub-shape, kind)).
    type Case<'a> = (
        &'a str,
        bool,
        &'a [&'a str],
        Option<(Option<SubShape>, &'a str)>,
    );
    let cases: [Case<'_>; 9] = [
        (
            "open_solid",
            false,
            &["NotClosed"],
            Some((None, "NotClosed")),
        ),
        (
            "reversed_face",
            false,
            &["BadOrientationOfSubshape"],
            Some((None, "BadOrientationOfSubshape")),
        ),
        (
            "same_range_off",
            false,
            &["InvalidSameParameterFlag", "InvalidSameRangeFlag"],
            Some((Some(edge(0)), "InvalidSameRangeFlag")),
        ),
        (
            "vertex_off_edges",
            false,
            &["InvalidPointOnCurve"],
            Some((Some(vertex(0)), "InvalidPointOnCurve")),
        ),
        (
            "missing_pcurve",
            false,
            &[
                "NoCurveOnSurface",
                "SelfIntersectingWire",
                "UnorientableShape",
            ],
            Some((Some(edge(0)), "NoCurveOnSurface")),
        ),
        (
            "bow_tie",
            false,
            &["SelfIntersectingWire", "UnorientableShape"],
            Some((Some(face(0)), "SelfIntersectingWire")),
        ),
        (
            "bow_tie",
            true,
            &[
                "BOPAlgo_SelfIntersect",
                "SelfIntersectingWire",
                "UnorientableShape",
            ],
            Some((Some(edge(0)), "BOPAlgo_SelfIntersect")),
        ),
        ("overlapping_boxes", true, &["BOPAlgo_SelfIntersect"], None),
        (
            "small_edge",
            true,
            &["BOPAlgo_SelfIntersect", "BOPAlgo_TooSmallEdge"],
            Some((Some(edge(0)), "BOPAlgo_TooSmallEdge")),
        ),
    ];
    for (name, self_interference, kinds, located) in cases {
        let Some(s) = defect(name) else {
            return;
        };
        let report = k.check(&s, &CheckOpts { self_interference });
        assert!(!report.valid, "{name}");
        assert_eq!(
            report.problem_kinds(),
            kinds,
            "{name}: {:?}",
            report.problems
        );
        if let Some((sub, kind)) = located {
            assert!(
                report.problems.iter().any(|(s, k)| *s == sub && k == kind),
                "{name}: no {kind} on {sub:?} in {:?}",
                report.problems
            );
        }
        // Without the analyzer, BRepCheck alone: no BOPAlgo kinds.
        let plain = k.check(&s, &CheckOpts::default());
        assert!(
            plain
                .problem_kinds()
                .iter()
                .all(|k| !k.starts_with("BOPAlgo_"))
        );
        assert_eq!(
            k.check(&s, &CheckOpts { self_interference }),
            report,
            "{name}"
        );
    }
    // BRepCheck misses the interference of two overlapping boxes.
    if let Some(s) = defect("overlapping_boxes") {
        assert!(k.check(&s, &CheckOpts::default()).valid);
        let report = k.check(
            &s,
            &CheckOpts {
                self_interference: true,
            },
        );
        let faces: BTreeSet<u32> = report
            .problems
            .iter()
            .filter_map(|(s, _)| s.filter(|s| s.kind == SubKind::Face).map(|s| s.index))
            .collect();
        assert!(faces.iter().any(|&f| f < 6) && faces.iter().any(|&f| f >= 6));
    }
    for name in ["box", "cylinder", "sphere", "torus", "filleted_box", "cone"] {
        let Some(s) = corpus::fixture(name) else {
            return;
        };
        let report = k.check(
            &s,
            &CheckOpts {
                self_interference: true,
            },
        );
        assert!(report.valid, "{name}: {:?}", report.problems);
    }
}

// ---- errors, compounds, determinism, cancellation -----------------------------------------

#[test]
fn bad_inputs_are_invalid_input() {
    let k = kernel();
    let block = k
        .make_box(&Plane::XY, Vec3::new(1.0, 2.0, 3.0))
        .unwrap()
        .shape;
    let invalid = |r: Result<OpResult, KernelError>, want: &str| match r {
        Err(KernelError::InvalidInput { what, .. }) => assert_eq!(what, want),
        other => panic!("expected InvalidInput {want}, got {other:?}"),
    };
    for opts in [
        HealOpts {
            tol: 0.0,
            ..HealOpts::default()
        },
        HealOpts {
            max_tol: 1e-9,
            ..HealOpts::default()
        },
        HealOpts {
            tol: f64::NAN,
            ..HealOpts::default()
        },
    ] {
        invalid(k.heal(&block, &opts, &ctx()), "opts");
    }
    for opts in [
        UnifyOpts {
            lin_tol: -1.0,
            ..UnifyOpts::default()
        },
        UnifyOpts {
            ang_tol: 4.0,
            ..UnifyOpts::default()
        },
    ] {
        invalid(k.unify_same_domain(&block, &opts), "opts");
    }
    invalid(k.sew(&[], &SewOpts::default(), &ctx()), "faces");
    let wire = k
        .make_wire(
            &[amg_brep::geom::CurveSeg {
                curve: amg_brep::geom::Curve3::Line {
                    origin: amg_brep::Point3::ZERO,
                    dir: amg_brep::Dir3::X,
                },
                t0: 0.0,
                t1: 1.0,
                tag: amg_brep::geom::Tag(0),
            }],
            &amg_brep::Tolerance::DEFAULT,
        )
        .unwrap()
        .shape;
    invalid(
        k.sew(std::slice::from_ref(&wire), &SewOpts::default(), &ctx()),
        "faces",
    );
    invalid(
        k.sew(
            &faces_of(&block),
            &SewOpts {
                tol: 0.0,
                ..SewOpts::default()
            },
            &ctx(),
        ),
        "opts",
    );
    invalid(k.make_solid(&block), "shell");
    invalid(k.make_solid(&wire), "shell");
    // An open shell: 5 faces of the box.
    let five: Vec<Shape> = faces_of(&block).into_iter().take(5).collect();
    let open = k
        .sew(
            &five,
            &SewOpts {
                make_solid: false,
                ..SewOpts::default()
            },
            &ctx(),
        )
        .unwrap()
        .shape;
    assert_eq!(open.kind(), ShapeKind::Shell);
    invalid(k.make_solid(&open), "shell");
    match k.sew(&five, &SewOpts::default(), &ctx()) {
        Err(KernelError::LocalOpFailed {
            op: "sew",
            message,
            partial: Some(partial),
            ..
        }) => {
            assert!(message.contains("4 free edge"), "{message}");
            assert_eq!(partial.count(SubKind::Face), 5);
        }
        other => panic!("{other:?}"),
    }
}

/// `make_compound` shares its parts' sub-shapes: a part given twice is indexed once, located
/// copies are distinct, and an empty list gives an empty compound.
#[test]
fn make_compound_shares_its_parts() {
    let k = kernel();
    let block = k
        .make_box(&Plane::XY, Vec3::new(1.0, 2.0, 3.0))
        .unwrap()
        .shape;
    let moved = block
        .transformed(&amg_brep::Transform::from_translation(Vec3::new(
            5.0, 0.0, 0.0,
        )))
        .unwrap();
    let before = (bin(&block), bin(&moved));
    let two = k.make_compound(&[block.clone(), moved.clone()]).unwrap();
    assert_eq!((bin(&block), bin(&moved)), before, "a part changed");
    assert_eq!(two.kind(), ShapeKind::Compound);
    assert_eq!(history_check::counts(&two), [12, 24, 16]);
    assert_eq!(two.solids().len(), 2);
    let twice = k.make_compound(&[block.clone(), block.clone()]).unwrap();
    assert_eq!(history_check::counts(&twice), [6, 12, 8]);
    let empty = k.make_compound(&[]).unwrap();
    assert_eq!(empty.kind(), ShapeKind::Compound);
    assert_eq!(history_check::counts(&empty), [0, 0, 0]);
    assert!(valid(&k, &empty) && valid(&k, &two));
    // The face of the compound is the box's own: same TShape.
    let f = two.sub_shape(face(0)).unwrap();
    assert!(f.is_same(&block.sub_shape(face(0)).unwrap()));
}

/// Every corpus op, 10 runs: identical BinTools bytes and history, inputs unchanged.
#[test]
fn ten_runs_are_byte_identical() {
    let k = kernel();
    for case in corpus::corpus(&k) {
        let inputs: Vec<Vec<u8>> = case.inputs.iter().map(bin).collect();
        let Ok(first) = case.run(&k) else {
            continue;
        };
        let bytes = bin(&first.shape);
        let history = format!("{:?}", first.history);
        for run in 1..10 {
            let again = case.run(&k).unwrap();
            assert!(bin(&again.shape) == bytes, "{} run {run}: bytes", case.name);
            assert_eq!(
                format!("{:?}", again.history),
                history,
                "{} run {run}: history",
                case.name
            );
        }
        let after: Vec<Vec<u8>> = case.inputs.iter().map(bin).collect();
        assert!(after == inputs, "{}: an input changed", case.name);
    }
}

#[test]
fn cancelled_jobs_do_not_run() {
    let k = kernel();
    let cancelled = JobCtx::detached();
    cancelled.cancel_token().cancel();
    let block = k
        .make_box(&Plane::XY, Vec3::new(1.0, 2.0, 3.0))
        .unwrap()
        .shape;
    assert!(matches!(
        k.heal(&block, &HealOpts::default(), &cancelled),
        Err(KernelError::Cancelled)
    ));
    assert!(matches!(
        k.sew(&faces_of(&block), &SewOpts::default(), &cancelled),
        Err(KernelError::Cancelled)
    ));
}

/// A cancel 100 ms into sewing the 1030 faces of the perforated plate (copied apart) returns
/// `Cancelled`; the bound (within 500 ms of the cancel) is asserted under `AMG_STRICT_TIMING=1`.
#[test]
fn cancel_stops_a_running_sew() {
    let k = kernel();
    let Some(plate) = corpus::fixture("plate_with_holes") else {
        return;
    };
    let faces = faces_apart(&plate);
    let started = Instant::now();
    let full = k.sew(&faces, &SewOpts::default(), &ctx()).unwrap();
    let full_time = started.elapsed();
    assert_eq!(full.shape.count(SubKind::Face), 1030);
    let job = JobCtx::detached();
    let token = job.cancel_token();
    let delay = Duration::from_millis(100);
    let canceller = std::thread::spawn(move || {
        std::thread::sleep(delay);
        token.cancel();
        Instant::now()
    });
    let started = Instant::now();
    let r = k.sew(&faces, &SewOpts::default(), &job);
    let stopped = Instant::now();
    let cancelled_at = canceller.join().unwrap();
    eprintln!(
        "sew of 1030 faces: full {full_time:?}; cancelled after {:?}, {:?} after the cancel",
        stopped - started,
        stopped.saturating_duration_since(cancelled_at)
    );
    if full_time > 4 * delay {
        assert!(matches!(r, Err(KernelError::Cancelled)), "{r:?}");
        if strict_timing() {
            assert!(stopped.saturating_duration_since(cancelled_at) < Duration::from_millis(500));
        }
    }
}

/// `heal` is also safe to run on shapes shared with a boolean (reads only), and the operands of a
/// sew may share sub-shapes with each other.
#[test]
fn sew_takes_faces_of_one_shape_and_its_copy() {
    let k = kernel();
    let block = k
        .make_box(&Plane::XY, Vec3::new(4.0, 4.0, 4.0))
        .unwrap()
        .shape;
    let other = k
        .boolean(
            BoolOp::Cut,
            std::slice::from_ref(&block),
            &[k.make_sphere(&Plane::XY, 1.0).unwrap().shape],
            &BoolOpts::default(),
            &ctx(),
        )
        .unwrap()
        .shape;
    let r = k.heal(&other, &HealOpts::default(), &ctx()).unwrap();
    assert!(r.warnings.is_empty(), "{:?}", r.warnings);
    // Mixed operands: a shell of five faces plus the sixth face.
    let mut faces = faces_of(&block);
    let last = faces.pop().unwrap();
    let five = k
        .sew(
            &faces,
            &SewOpts {
                make_solid: false,
                ..SewOpts::default()
            },
            &ctx(),
        )
        .unwrap()
        .shape;
    let r = k
        .sew(&[five.clone(), last.clone()], &SewOpts::default(), &ctx())
        .unwrap();
    assert_eq!(r.shape.kind(), ShapeKind::Solid);
    assert!(rel(volume(&k, &r.shape), 64.0) <= 1e-12);
    assert_eq!(r.history.operand_counts.len(), 2);
    assert!(completeness_error(&r, &[&five, &last]).is_none());
    let _ = FaceId(0);
}
