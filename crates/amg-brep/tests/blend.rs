//! BREP-06: fillets (constant, variable, chord) and chamfers (equal, two distances,
//! distance-angle), `LocalOpFailed { offending }` and `fillet_max_radius`.
//!
//! Done-when 1 (golden volumes of the 20-case corpus in `common/blend_corpus.rs` within 1e-6
//! relative, `tests/golden/brep/blend_*.toml`, and the analytic volume where one exists), 2 (a
//! radius larger than the adjacent face fails on exactly that edge), 3 (`fillet_max_radius` on 5
//! cases within 1 % of the reference largest workable radius), 4 (each blend face maps to its
//! source edge), 5 (10 runs are byte-identical). Done-when 6 is in `tests/stress.rs`; the
//! history completeness of the corpus is in `tests/history.rs`.
#![cfg(feature = "occt")]
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

mod common;
#[path = "common/blend_corpus.rs"]
mod corpus;
#[path = "common/history.rs"]
mod history_check;

use std::f64::consts::FRAC_PI_2;
use std::path::PathBuf;

use amg_brep::{
    BrepKernel, ChamferSpec, EdgeId, FilletOpts, FilletShape, FilletSpec, InputSub, JobCtx,
    KernelConfig, KernelError, OcctKernel, OpResult, OpRole, Plane, Point3, Relation, Shape,
    SubKind, SubShape, Validate, Vec3,
};

use corpus::{A, BlendOp, Case, cube, edge_at, face_at};

/// Done-when 1 bound.
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

fn edge_sub(e: EdgeId) -> SubShape {
    SubShape {
        kind: SubKind::Edge,
        index: e.0,
    }
}

fn face_sub(index: u32) -> SubShape {
    SubShape {
        kind: SubKind::Face,
        index,
    }
}

// ---- goldens -----------------------------------------------------------------------------

#[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
struct BlendGolden {
    tolerances: GoldenTolerances,
    metrics: BlendMetrics,
}

#[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
struct GoldenTolerances {
    /// Relative bound on volume and area.
    rel: f64,
}

#[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
struct BlendMetrics {
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
        .join(format!("blend_{case}.toml"))
}

fn metrics(k: &OcctKernel, s: &Shape) -> BlendMetrics {
    let m = k.mass_props(s).unwrap();
    BlendMetrics {
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
        let golden = BlendGolden {
            tolerances: GoldenTolerances { rel: REL },
            metrics,
        };
        let text = format!(
            "# B-rep metrics of brep/blend_{case} (S02 BREP-06). Written by `AMG_BLESS=1`.\n\n{}",
            toml::to_string(&golden).unwrap()
        );
        std::fs::write(&path, text).unwrap();
        return;
    }
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("{}: {e}; bless it with AMG_BLESS=1", path.display()));
    let golden: BlendGolden = toml::from_str(&text).unwrap();
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
            "{case}: {what} {got} vs golden {golden} ({err:e})"
        );
    }
}

fn run_ok(k: &OcctKernel, case: &Case) -> OpResult {
    case.run(k)
        .unwrap_or_else(|e| panic!("{}: {e} ({e:?})", case.name))
}

/// Done-when 1: every corpus case gives a valid result matching its golden within 1e-6, and
/// its analytic volume where there is one.
#[test]
fn golden_fillets_and_chamfers_match_the_reference_volumes() {
    let k = kernel();
    let cases = corpus::corpus(&k);
    assert!(cases.len() >= 20, "{} cases", cases.len());
    let mut analytic = 0;
    for case in &cases {
        let r = run_ok(&k, case);
        assert!(
            k.check(&r.shape, &Default::default()).valid,
            "{}",
            case.name
        );
        assert_eq!(r.shape.solids().len(), 1, "{}", case.name);
        check_golden(&k, case.name, &r.shape);
        let volume = k.mass_props(&r.shape).unwrap().volume;
        if let Some((want, rel)) = case.analytic {
            let err = (volume - want).abs() / want;
            eprintln!(
                "{}: volume {volume} vs analytic {want} ({err:.2e})",
                case.name
            );
            assert!(err <= rel, "{}: {volume} vs {want} ({err:e})", case.name);
            analytic += 1;
        } else {
            eprintln!("{}: volume {volume} (golden only)", case.name);
        }
    }
    eprintln!("{} cases, {analytic} against analytic volumes", cases.len());
}

/// The radius laws and the asymmetric chamfers do what their specs say.
#[test]
fn variable_radii_and_asymmetric_chamfers_follow_their_specs() {
    let k = kernel();
    let c = cube(&k);
    let edge = edge_at(&k, &c, Point3::new(A / 2.0, 0.0, 0.0));
    let volume = |spec: FilletSpec| {
        let r = k
            .fillet(&c, &[(edge, spec)], &FilletOpts::default(), &ctx())
            .unwrap();
        k.mass_props(&r.shape).unwrap().volume
    };
    let (v1, v3) = (
        volume(FilletSpec::Constant(1.0)),
        volume(FilletSpec::Constant(3.0)),
    );
    let linear = volume(FilletSpec::Variable(vec![(0.0, 1.0), (1.0, 3.0)]));
    assert!(v3 < linear && linear < v1, "{v3} < {linear} < {v1}");
    // A law that is constant is the constant fillet.
    let flat = volume(FilletSpec::Variable(vec![
        (0.0, 2.0),
        (0.5, 2.0),
        (1.0, 2.0),
    ]));
    let two = volume(FilletSpec::Constant(2.0));
    assert!((flat - two).abs() / two < 1e-6, "{flat} vs {two}");

    // TwoDistances: d1 on the reference face. The bottom face loses d1 of its width.
    let bottom = face_at(&k, &c, Point3::new(A / 2.0, A / 2.0, 0.0));
    let front = face_at(&k, &c, Point3::new(A / 2.0, 0.0, A / 2.0));
    for (face, on_bottom) in [(bottom, 1.0), (front, 3.0)] {
        let r = k
            .chamfer(
                &c,
                &[(
                    edge,
                    ChamferSpec::TwoDistances {
                        d1: 1.0,
                        d2: 3.0,
                        face,
                    },
                )],
                &ctx(),
            )
            .unwrap();
        let image = r
            .history
            .of_input(InputSub {
                operand: 0,
                sub: face_sub(bottom.0),
            })
            .find_map(|rel| match rel {
                Relation::Modified(o) => Some(o[0]),
                _ => None,
            })
            .unwrap();
        let area = r.shape.face_area(amg_brep::FaceId(image.index));
        let want = A * (A - on_bottom);
        assert!((area - want).abs() < 1e-9, "{area} vs {want}");
    }
    // DistanceAngle: d on the reference face, d·tan(angle) on the other.
    let angle = 0.3;
    let r = k
        .chamfer(
            &c,
            &[(
                edge,
                ChamferSpec::DistanceAngle {
                    d: 2.0,
                    angle,
                    face: front,
                },
            )],
            &ctx(),
        )
        .unwrap();
    let bottom_image = r
        .history
        .of_input(InputSub {
            operand: 0,
            sub: face_sub(bottom.0),
        })
        .find_map(|rel| match rel {
            Relation::Modified(o) => Some(o[0]),
            _ => None,
        })
        .unwrap();
    let area = r.shape.face_area(amg_brep::FaceId(bottom_image.index));
    let want = A * (A - 2.0 * angle.tan());
    assert!((area - want).abs() < 1e-9, "{area} vs {want}");
}

#[test]
fn bad_specs_are_invalid_input() {
    let k = kernel();
    let c = cube(&k);
    let edge = edge_at(&k, &c, Point3::new(A / 2.0, 0.0, 0.0));
    let top = face_at(&k, &c, Point3::new(A / 2.0, A / 2.0, A));
    let fillet = |edges: &[(EdgeId, FilletSpec)]| {
        k.fillet(&c, edges, &FilletOpts::default(), &ctx())
            .unwrap_err()
    };
    let invalid = |e: KernelError| match e {
        KernelError::InvalidInput {
            what, offending, ..
        } => (what, offending),
        other => panic!("{other:?}"),
    };
    assert_eq!(invalid(fillet(&[])).0, "edges");
    assert_eq!(
        invalid(fillet(&[(EdgeId(99), FilletSpec::Constant(1.0))])).0,
        "edges"
    );
    let (_, off) = invalid(fillet(&[
        (edge, FilletSpec::Constant(1.0)),
        (edge, FilletSpec::Constant(1.0)),
    ]));
    assert_eq!(off, vec![edge_sub(edge)]);
    assert_eq!(
        invalid(fillet(&[(edge, FilletSpec::Constant(-1.0))])).1,
        vec![edge_sub(edge)]
    );
    assert_eq!(
        invalid(fillet(&[(edge, FilletSpec::Variable(vec![(0.0, 1.0)]))])).1,
        vec![edge_sub(edge)]
    );
    let chamfer = |spec: ChamferSpec| k.chamfer(&c, &[(edge, spec)], &ctx()).unwrap_err();
    assert_eq!(
        invalid(chamfer(ChamferSpec::TwoDistances {
            d1: 1.0,
            d2: 1.0,
            face: top
        }))
        .1,
        vec![edge_sub(edge)]
    );
    let bottom = face_at(&k, &c, Point3::new(A / 2.0, A / 2.0, 0.0));
    invalid(chamfer(ChamferSpec::DistanceAngle {
        d: 1.0,
        angle: FRAC_PI_2,
        face: bottom,
    }));
    assert!(matches!(
        k.fillet(
            &c,
            &[(edge, FilletSpec::Constant(1.0))],
            &FilletOpts {
                tol_app_3d: Some(0.0),
                ..FilletOpts::default()
            },
            &ctx()
        ),
        Err(KernelError::InvalidInput { what: "opts", .. })
    ));
    // Two edges of one tangent chain with different radii.
    let slot = corpus::slot_prism(&k);
    let a = edge_at(
        &k,
        &slot,
        Point3::new(corpus::SLOT_LEN / 2.0, -corpus::SLOT_R, corpus::SLOT_H),
    );
    let b = edge_at(
        &k,
        &slot,
        Point3::new(corpus::SLOT_LEN / 2.0, corpus::SLOT_R, corpus::SLOT_H),
    );
    let err = k
        .fillet(
            &slot,
            &[
                (a, FilletSpec::Constant(1.0)),
                (b, FilletSpec::Constant(0.5)),
            ],
            &FilletOpts::default(),
            &ctx(),
        )
        .unwrap_err();
    assert!(
        matches!(err, KernelError::InvalidInput { what: "edges", .. }),
        "{err:?}"
    );
    // ... and with the same radius it is one contour.
    let r = k
        .fillet(
            &slot,
            &[
                (a, FilletSpec::Constant(1.0)),
                (b, FilletSpec::Constant(1.0)),
            ],
            &FilletOpts::default(),
            &ctx(),
        )
        .unwrap();
    assert!(
        r.history
            .roles
            .iter()
            .all(|(_, role)| *role == OpRole::FilletBlend { contour: 0 }),
        "{:?}",
        r.history.roles
    );
}

// ---- Done-when 2 -------------------------------------------------------------------------

fn local_failure(e: KernelError) -> (String, Vec<SubShape>, Option<Shape>) {
    match e {
        KernelError::LocalOpFailed {
            message,
            offending,
            partial,
            ..
        } => (message, offending, partial),
        other => panic!("expected LocalOpFailed, got {other:?}"),
    }
}

/// A 20 × 20 × 4 plate: the top edge along x at the front borders a 4 mm high side face.
fn plate(k: &OcctKernel) -> Shape {
    k.make_box(&Plane::XY, Vec3::new(20.0, 20.0, 4.0))
        .unwrap()
        .shape
}

#[test]
fn a_radius_larger_than_the_adjacent_face_fails_on_that_edge() {
    let k = kernel();
    let s = plate(&k);
    let front_top = edge_at(&k, &s, Point3::new(10.0, 0.0, 4.0));
    let back_bottom = edge_at(&k, &s, Point3::new(10.0, 20.0, 0.0));
    let left_vertical = edge_at(&k, &s, Point3::new(0.0, 20.0, 2.0));

    // One edge, radius 6 against a 4 mm face.
    let (message, offending, partial) = local_failure(
        k.fillet(
            &s,
            &[(front_top, FilletSpec::Constant(6.0))],
            &FilletOpts::default(),
            &ctx(),
        )
        .unwrap_err(),
    );
    eprintln!("one edge: {message}; partial: {}", partial.is_some());
    assert_eq!(offending, vec![edge_sub(front_top)]);

    // With two workable edges that do not touch it, only the failing edge is offending.
    let (message, offending, partial) = local_failure(
        k.fillet(
            &s,
            &[
                (back_bottom, FilletSpec::Constant(1.0)),
                (front_top, FilletSpec::Constant(6.0)),
                (left_vertical, FilletSpec::Constant(1.0)),
            ],
            &FilletOpts::default(),
            &ctx(),
        )
        .unwrap_err(),
    );
    eprintln!("three edges: {message}; partial: {}", partial.is_some());
    assert_eq!(offending, vec![edge_sub(front_top)]);

    // Chamfers: no fault list from OCCT, found by building each contour alone.
    let (message, offending, _) = local_failure(
        k.chamfer(
            &s,
            &[
                (back_bottom, ChamferSpec::Equal(1.0)),
                (front_top, ChamferSpec::Equal(6.0)),
            ],
            &ctx(),
        )
        .unwrap_err(),
    );
    eprintln!("chamfer: {message}");
    assert_eq!(offending, vec![edge_sub(front_top)]);

    // A variable radius above the face height all along.
    let (_, offending, _) = local_failure(
        k.fillet(
            &s,
            &[(
                front_top,
                FilletSpec::Variable(vec![(0.0, 5.0), (1.0, 7.0)]),
            )],
            &FilletOpts::default(),
            &ctx(),
        )
        .unwrap_err(),
    );
    assert_eq!(offending, vec![edge_sub(front_top)]);

    // Finding (recorded in S02): a law that outgrows the face only near one end is reported
    // done by OCCT, with the topology of an ordinary fillet. Logged, not asserted.
    match k.fillet(
        &s,
        &[(
            front_top,
            FilletSpec::Variable(vec![(0.0, 1.0), (1.0, 7.0)]),
        )],
        &FilletOpts::default(),
        &ctx(),
    ) {
        Ok(r) => eprintln!(
            "law 1 -> 7 on a 4 mm face: done, {} faces, volume {}",
            r.shape.count(SubKind::Face),
            k.mass_props(&r.shape).unwrap().volume
        ),
        Err(e) => eprintln!("law 1 -> 7 on a 4 mm face: {e}"),
    }
}

// ---- Done-when 3 -------------------------------------------------------------------------

/// The largest radius on a grid of `hi / steps` that works, scanning down from `hi`.
fn scanned_max(k: &OcctKernel, s: &Shape, edges: &[EdgeId], hi: f64, steps: u32) -> f64 {
    for i in (1..=steps).rev() {
        let r = hi * f64::from(i) / f64::from(steps);
        let specs: Vec<(EdgeId, FilletSpec)> = edges
            .iter()
            .map(|&e| (e, FilletSpec::Constant(r)))
            .collect();
        if let Ok(res) = k.fillet(s, &specs, &FilletOpts::default(), &ctx())
            && k.check(&res.shape, &Default::default()).valid
        {
            return r;
        }
    }
    0.0
}

#[test]
fn fillet_max_radius_is_within_one_percent_of_the_largest_workable_radius() {
    let k = kernel();
    let s = plate(&k);
    let c = cube(&k);
    let slot = corpus::slot_prism(&k);
    let cylinder = k
        .make_cylinder(&Plane::XY, 5.0, 3.0, std::f64::consts::TAU)
        .unwrap()
        .shape;
    let cases: Vec<(&str, Shape, Vec<EdgeId>, f64)> = vec![
        (
            "plate_front_top",
            s.clone(),
            vec![edge_at(&k, &s, Point3::new(10.0, 0.0, 4.0))],
            10.0,
        ),
        (
            "plate_top_pair",
            s.clone(),
            vec![
                edge_at(&k, &s, Point3::new(10.0, 0.0, 4.0)),
                edge_at(&k, &s, Point3::new(20.0, 10.0, 4.0)),
            ],
            10.0,
        ),
        (
            "cube_corner_three",
            c.clone(),
            vec![
                edge_at(&k, &c, Point3::new(A / 2.0, 0.0, 0.0)),
                edge_at(&k, &c, Point3::new(0.0, A / 2.0, 0.0)),
                edge_at(&k, &c, Point3::new(0.0, 0.0, A / 2.0)),
            ],
            12.0,
        ),
        (
            "slot_chain",
            slot.clone(),
            vec![edge_at(
                &k,
                &slot,
                Point3::new(corpus::SLOT_LEN / 2.0, -corpus::SLOT_R, corpus::SLOT_H),
            )],
            8.0,
        ),
        (
            "short_cylinder_rim",
            cylinder.clone(),
            vec![edge_at(&k, &cylinder, Point3::new(0.0, 0.0, 3.0))],
            8.0,
        ),
    ];
    for (name, shape, edges, hi) in &cases {
        let found = k.fillet_max_radius(shape, edges, *hi, 20, &ctx()).unwrap();
        let reference = scanned_max(&k, shape, edges, *hi, 1000);
        let err = (found - reference).abs() / reference;
        eprintln!(
            "{name}: max radius {found:.6} vs scanned {reference:.6} ({:.3} %)",
            err * 100.0
        );
        assert!(reference > 0.0, "{name}: nothing works");
        assert!(err <= 0.01, "{name}: {found} vs {reference}");
        // The answer itself works.
        let specs: Vec<(EdgeId, FilletSpec)> = edges
            .iter()
            .map(|&e| (e, FilletSpec::Constant(found)))
            .collect();
        k.fillet(shape, &specs, &FilletOpts::default(), &ctx())
            .unwrap_or_else(|e| panic!("{name}: {found} fails: {e}"));
    }
    // A workable `hi` comes back as is; bad arguments are refused.
    let e = edge_at(&k, &c, Point3::new(A / 2.0, 0.0, 0.0));
    assert_eq!(
        k.fillet_max_radius(&c, &[e], 1.0, 4, &ctx())
            .unwrap()
            .to_bits(),
        1f64.to_bits()
    );
    assert!(matches!(
        k.fillet_max_radius(&c, &[e], f64::NAN, 4, &ctx()),
        Err(KernelError::InvalidInput { what: "hi", .. })
    ));
    assert!(matches!(
        k.fillet_max_radius(&c, &[e], 1.0, 0, &ctx()),
        Err(KernelError::InvalidInput {
            what: "iterations",
            ..
        })
    ));
    let cancelled = JobCtx::detached();
    cancelled.cancel_token().cancel();
    assert!(matches!(
        k.fillet_max_radius(&c, &[e], 100.0, 20, &cancelled),
        Err(KernelError::Cancelled)
    ));
}

// ---- Done-when 4 -------------------------------------------------------------------------

/// Every blend face has the role of its contour and is `Generated` from an input edge of that
/// contour; every other new face is a corner patch; trimmed faces are `Modified`.
#[test]
fn every_blend_face_maps_to_its_source_edge() {
    let k = kernel();
    let mut blend_faces = 0;
    for case in corpus::corpus(&k) {
        let r = run_ok(&k, &case);
        let requested = case.edges();
        let mut by_contour: std::collections::BTreeMap<u32, Vec<SubShape>> = Default::default();
        for (face, role) in &r.history.roles {
            let sources = r.history.sources_of(*face);
            let generated_by: Vec<SubShape> = sources
                .iter()
                .filter(|(_, rel)| matches!(rel, Relation::Generated(_)))
                .map(|(input, _)| input.sub)
                .collect();
            match role {
                OpRole::FilletBlend { contour } => {
                    assert!(
                        !case.is_chamfer(),
                        "{}: fillet role on a chamfer",
                        case.name
                    );
                    assert!(
                        generated_by.iter().any(|s| s.kind == SubKind::Edge),
                        "{}: blend face {face:?} has no source edge ({sources:?})",
                        case.name
                    );
                    by_contour.entry(*contour).or_default().extend(generated_by);
                    blend_faces += 1;
                }
                OpRole::ChamferFace => {
                    assert!(case.is_chamfer(), "{}", case.name);
                    assert!(
                        generated_by.iter().any(|s| s.kind == SubKind::Edge),
                        "{}: chamfer face {face:?} has no source edge ({sources:?})",
                        case.name
                    );
                    by_contour.entry(0).or_default().extend(generated_by);
                    blend_faces += 1;
                }
                OpRole::CornerPatch => {}
                other => panic!("{}: unexpected role {other:?}", case.name),
            }
        }
        assert!(!by_contour.is_empty(), "{}: no blend faces", case.name);
        // Every requested edge generated a blend face.
        let sources: Vec<SubShape> = by_contour.values().flatten().copied().collect();
        for e in &requested {
            assert!(
                sources.contains(&edge_sub(*e)),
                "{}: edge {e:?} generated no blend face ({:?})",
                case.name,
                r.history.roles
            );
        }
    }
    eprintln!("{blend_faces} blend/chamfer faces checked");

    // The single-edge case in detail: the edge generates exactly the one blend face, its two
    // faces are trimmed (Modified), the far faces pass through 1:1.
    let c = cube(&k);
    let edge = edge_at(&k, &c, Point3::new(A / 2.0, 0.0, 0.0));
    let r = k
        .fillet(
            &c,
            &[(edge, FilletSpec::Constant(2.0))],
            &FilletOpts::default(),
            &ctx(),
        )
        .unwrap();
    let blend: Vec<SubShape> = r
        .history
        .roles
        .iter()
        .filter(|(_, role)| *role == OpRole::FilletBlend { contour: 0 })
        .map(|(f, _)| *f)
        .collect();
    assert_eq!(blend.len(), 1, "{:?}", r.history.roles);
    let rel: Vec<&Relation> = r
        .history
        .of_input(InputSub {
            operand: 0,
            sub: edge_sub(edge),
        })
        .collect();
    assert!(
        rel.iter()
            .any(|x| matches!(x, Relation::Generated(o) if o.as_slice() == blend.as_slice())),
        "{rel:?}"
    );
    assert_eq!(r.shape.count(SubKind::Face), 7);
}

// ---- Done-when 5 -------------------------------------------------------------------------

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
fn cancelled_jobs_do_not_run() {
    let k = kernel();
    let c = cube(&k);
    let edge = edge_at(&k, &c, Point3::new(A / 2.0, 0.0, 0.0));
    let cancelled = JobCtx::detached();
    cancelled.cancel_token().cancel();
    assert!(matches!(
        k.fillet(
            &c,
            &[(edge, FilletSpec::Constant(1.0))],
            &FilletOpts::default(),
            &cancelled
        ),
        Err(KernelError::Cancelled)
    ));
    assert!(matches!(
        k.chamfer(&c, &[(edge, ChamferSpec::Equal(1.0))], &cancelled),
        Err(KernelError::Cancelled)
    ));
}

/// `FilletOpts` reach ChFi3d: the blend surface shape and the approximation tolerances change
/// the variable-radius blend surface, not the volume beyond the approximation.
#[test]
fn opts_reach_the_blend() {
    let k = kernel();
    let c = cube(&k);
    let edge = edge_at(&k, &c, Point3::new(A / 2.0, 0.0, 0.0));
    let blend_surface = |spec: FilletSpec, opts: FilletOpts| {
        let r = k.fillet(&c, &[(edge, spec)], &opts, &ctx()).unwrap();
        let (face, _) = r
            .history
            .roles
            .iter()
            .find(|(_, role)| matches!(role, OpRole::FilletBlend { .. }))
            .cloned()
            .unwrap();
        let surface = r.shape.surface(amg_brep::FaceId(face.index));
        let volume = k.mass_props(&r.shape).unwrap().volume;
        (format!("{surface:?}"), surface.class(), volume)
    };
    let law = || FilletSpec::Variable(vec![(0.0, 1.0), (0.4, 2.5), (1.0, 3.0)]);
    let (fine, _, vf) = blend_surface(law(), FilletOpts::default());
    let (coarse, _, vc) = blend_surface(
        law(),
        FilletOpts {
            tol_app_3d: Some(0.05),
            fleche: Some(0.05),
            ..FilletOpts::default()
        },
    );
    eprintln!(
        "variable blend: default {} chars, volume {vf}; coarse {} chars, volume {vc}",
        fine.len(),
        coarse.len()
    );
    assert_ne!(fine, coarse, "the approximation parameters changed nothing");
    assert!((vc - vf).abs() / vf < 1e-3, "{vc} vs {vf}");
    // The blend shape: a plane-plane constant fillet is an exact cylinder whatever the shape
    // (ChFiKPart), so the variable law shows it.
    let (polynomial, class, vp) = blend_surface(
        law(),
        FilletOpts {
            shape: FilletShape::Polynomial,
            ..FilletOpts::default()
        },
    );
    eprintln!("variable blend, polynomial: {class:?}, volume {vp}");
    assert_ne!(fine, polynomial, "the blend shape changed nothing");
    assert!((vp - vf).abs() / vf < 1e-3, "{vp} vs {vf}");
    let _ = BlendOp::Chamfer(Vec::new());
}

/// S02 Testing: random radii and distances on random edge sets of a box give `Ok(valid)` or a
/// clean `Err` (never a crash), and every error names input edges.
#[test]
fn random_radii_give_valid_results_or_clean_errors() {
    let k = kernel();
    let s = k
        .make_box(&Plane::XY, Vec3::new(12.0, 8.0, 5.0))
        .unwrap()
        .shape;
    let n = s.count(SubKind::Edge);
    let mut state: u64 = 0x0B1E_0D06_5EED_0001;
    let mut next = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        state
    };
    let (mut ok, mut failed) = (0, 0);
    for case in 0..200 {
        let count = 1 + (next() % 4) as u32;
        let mut edges: Vec<EdgeId> = (0..count)
            .map(|_| EdgeId((next() % u64::from(n)) as u32))
            .collect();
        edges.sort();
        edges.dedup();
        let size = |r: u64| 0.2 + (r % 1000) as f64 / 1000.0 * 7.0;
        let result = if case % 3 == 2 {
            let specs: Vec<(EdgeId, ChamferSpec)> = edges
                .iter()
                .map(|&e| (e, ChamferSpec::Equal(size(next()))))
                .collect();
            k.chamfer(&s, &specs, &ctx())
        } else {
            let specs: Vec<(EdgeId, FilletSpec)> = edges
                .iter()
                .map(|&e| (e, FilletSpec::Constant(size(next()))))
                .collect();
            k.fillet(&s, &specs, &FilletOpts::default(), &ctx())
        };
        match result {
            Ok(r) => {
                assert!(k.check(&r.shape, &Default::default()).valid, "case {case}");
                ok += 1;
            }
            Err(KernelError::LocalOpFailed { offending, .. }) => {
                assert!(!offending.is_empty(), "case {case}: no offending edges");
                for o in &offending {
                    assert!(
                        edges.contains(&EdgeId(o.index)),
                        "case {case}: {o:?} not requested"
                    );
                }
                failed += 1;
            }
            Err(e) => panic!("case {case} ({edges:?}): unexpected {e:?}"),
        }
    }
    eprintln!("random blends: {ok} valid, {failed} LocalOpFailed");
    assert!(ok > 20 && failed > 20, "{ok} / {failed}");
}
