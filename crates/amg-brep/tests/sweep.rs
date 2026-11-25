//! BREP-15: pipe-shell sweeps (Frenet, corrected Frenet, fixed, binormal, guide, support; twist
//! and scale laws; transitions; solid or shell) and the raw ThruSections loft (ruled or smooth,
//! point sections, the compatibility check), with the roles `StartCap`/`EndCap`/`Lateral` and
//! `LoftSection`/`Lateral`.
//!
//! Done-when 1 and 3 (golden volumes of the corpus in `common/sweep_corpus.rs`, goldens
//! `tests/golden/brep/{pipe,loft}_*.toml`, against closed forms), 2 (twisted and scaled sweeps
//! against their reference volumes, and the end section's turn and size), 4 (lateral faces come
//! from the section edges), 5 (incompatible sections give `OpFailed`, never a crash). The
//! history completeness of the corpus is in `tests/history.rs`.
#![cfg(feature = "occt")]
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

mod common;
#[path = "common/sweep_corpus.rs"]
mod corpus;

use std::collections::BTreeMap;
use std::f64::consts::{FRAC_PI_2, PI};
use std::path::PathBuf;

use amg_brep::{
    BrepKernel, Dir3, InputSub, JobCtx, KernelConfig, KernelError, LoftSection, OcctKernel,
    OpResult, OpRole, PipeMode, PipeOpts, Relation, Shape, SubKind, SubShape, ThruSectionsOpts,
    Validate, Vec3,
};

use corpus::{Case, SweepOp, circle, p, polyline, rectangle, xy};

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

fn face_sub(index: u32) -> SubShape {
    SubShape {
        kind: SubKind::Face,
        index,
    }
}

// ---- goldens -----------------------------------------------------------------------------

#[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
struct SweepGolden {
    tolerances: GoldenTolerances,
    metrics: SweepMetrics,
}

#[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
struct GoldenTolerances {
    /// Relative bound on volume and area (absolute 1e-9 for a zero golden: shells).
    rel: f64,
}

#[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
struct SweepMetrics {
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

fn metrics(k: &OcctKernel, s: &Shape) -> SweepMetrics {
    let m = k.mass_props(s).unwrap();
    SweepMetrics {
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
        let golden = SweepGolden {
            tolerances: GoldenTolerances { rel: REL },
            metrics,
        };
        let text = format!(
            "# B-rep metrics of brep/{case} (S02 BREP-15). Written by `AMG_BLESS=1`.\n\n{}",
            toml::to_string(&golden).unwrap()
        );
        std::fs::write(&path, text).unwrap();
        return;
    }
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("{}: {e}; bless it with AMG_BLESS=1", path.display()));
    let golden: SweepGolden = toml::from_str(&text).unwrap();
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
        let err = if golden.abs() < 1e-9 {
            (got - golden).abs()
        } else {
            (got - golden).abs() / golden.abs()
        };
        assert!(
            err <= rel,
            "{case}: {what} {got} vs golden {golden} ({err:e})"
        );
    }
}

fn run_ok(k: &dyn BrepKernel, case: &Case) -> OpResult {
    case.run(k)
        .unwrap_or_else(|e| panic!("{}: {e} ({e:?})", case.name))
}

fn case<'a>(cases: &'a [Case], name: &str) -> &'a Case {
    cases.iter().find(|c| c.name == name).unwrap()
}

/// Every corpus case gives a valid result matching its golden; solids are one solid and match
/// their reference volume within the case's bound (Done-when 1, 2, 3).
#[test]
fn golden_sweeps_and_lofts_match_the_reference_volumes() {
    let k = kernel();
    let mut failures = Vec::new();
    for case in corpus::corpus(&k) {
        let r = run_ok(&k, &case);
        assert!(
            k.check(&r.shape, &amg_brep::CheckOpts::default()).valid,
            "{}: invalid",
            case.name
        );
        if case.solid() {
            assert_eq!(r.shape.solids().len(), 1, "{}: one solid", case.name);
        } else {
            assert!(r.shape.solids().is_empty(), "{}: a shell", case.name);
        }
        check_golden(&k, case.name, &r.shape);
        let volume = k.mass_props(&r.shape).unwrap().volume;
        match case.analytic {
            Some((want, bound)) => {
                let err = (volume - want).abs() / want;
                eprintln!(
                    "{}: volume {volume} vs {want} (rel {err:e}, bound {bound:e})",
                    case.name
                );
                if err > bound {
                    failures.push(format!(
                        "{}: volume {volume} vs analytic {want} ({err:e} > {bound:e})",
                        case.name
                    ));
                }
            }
            None => eprintln!("{}: volume {volume} (golden only)", case.name),
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// The cap faces: `(start, end)` roles of a pipe, `LoftSection` 0 and n − 1 of a loft.
fn caps(r: &OpResult) -> Vec<(u32, OpRole)> {
    r.history
        .roles
        .iter()
        .filter(|(_, role)| *role != OpRole::Lateral)
        .map(|(s, role)| (s.index, role.clone()))
        .collect()
}

fn cap_area(r: &OpResult, role: &OpRole) -> f64 {
    let (f, _) = caps(r).into_iter().find(|(_, r)| r == role).unwrap();
    r.shape.face_area(amg_brep::FaceId(f))
}

/// The twist turns the end section by the law's angle (+ = counter-clockwise about the spine
/// tangent), and the scale law grows it by its factor (Done-when 2).
#[test]
fn twist_and_scale_laws_shape_the_end_section() {
    let k = kernel();
    let cases = corpus::corpus(&k);
    // A 6 × 2 rectangle, long side along x, turned by π at the end: the end cap's box is the
    // start's again; turned by π/2 (a three-sample law ending at π/2) the long side is along y.
    let quarter = Case {
        name: "quarter",
        operands: vec![
            rectangle(&k, xy(0.0), 6.0, 2.0),
            polyline(&k, &[p(0.0, 0.0, 0.0), p(0.0, 0.0, 20.0)]),
        ],
        op: SweepOp::Pipe {
            profiles: 1,
            opts: PipeOpts {
                twist: Some(vec![(0.0, 0.0), (0.4, 0.2), (1.0, FRAC_PI_2)]),
                ..PipeOpts::default()
            },
        },
        analytic: None,
    };
    for (c, long_x) in [
        (&quarter, false),
        (case(&cases, "pipe_rectangle_twist_law"), true),
    ] {
        let r = run_ok(&k, c);
        let (end, _) = caps(&r)
            .into_iter()
            .find(|(_, role)| *role == OpRole::EndCap)
            .unwrap();
        let cap = r.shape.sub_shape(face_sub(end)).unwrap();
        let b = cap.bbox();
        let size = b.max - b.min;
        eprintln!("{}: end cap box {size}", c.name);
        let (long, short) = if long_x {
            (size.x, size.y)
        } else {
            (size.y, size.x)
        };
        assert!(
            (long - 6.0).abs() < 1e-3 && (short - 2.0).abs() < 1e-3,
            "{}: {size}",
            c.name
        );
        assert!((b.min.z - 20.0).abs() < 1e-6 && (b.max.z - 20.0).abs() < 1e-6);
    }
    // The sign: +π/2 · 0.5 at mid-length turns the long side towards +y from +x.
    let half = Case {
        name: "half",
        operands: vec![
            rectangle(&k, xy(0.0), 6.0, 2.0),
            polyline(&k, &[p(0.0, 0.0, 0.0), p(0.0, 0.0, 20.0)]),
        ],
        op: SweepOp::Pipe {
            profiles: 1,
            opts: PipeOpts {
                twist: Some(vec![(0.0, 0.0), (1.0, FRAC_PI_2 / 2.0)]),
                ..PipeOpts::default()
            },
        },
        analytic: None,
    };
    let r = run_ok(&k, &half);
    let (end, _) = caps(&r)
        .into_iter()
        .find(|(_, role)| *role == OpRole::EndCap)
        .unwrap();
    let cap = r.shape.sub_shape(face_sub(end)).unwrap();
    // The corner that started at (3, 1) is at 45° more: (3 cos 45° − sin 45°, 3 sin 45° + cos 45°).
    let (c, s) = ((PI / 4.0).cos(), (PI / 4.0).sin());
    let want = p(3.0 * c - s, 3.0 * s + c, 20.0);
    let nearest = cap
        .vertices()
        .map(|v| (cap.vertex_point(v) - want).length())
        .fold(f64::INFINITY, f64::min);
    assert!(nearest < 1e-3, "no end corner at {want} ({nearest})");

    // Scale 1 → 2: the end cap is the circle of radius 4.
    let r = run_ok(&k, case(&cases, "pipe_circle_scale"));
    let start = cap_area(&r, &OpRole::StartCap);
    let end = cap_area(&r, &OpRole::EndCap);
    assert!((start - PI * 4.0).abs() < 1e-6 * PI * 4.0, "start {start}");
    assert!((end - PI * 16.0).abs() < 1e-4 * PI * 16.0, "end {end}");
    let r = run_ok(&k, case(&cases, "pipe_circle_scale_two_edges"));
    assert!((cap_area(&r, &OpRole::EndCap) - PI * 16.0).abs() < 1e-4 * PI * 16.0);
}

fn generated_from(r: &OpResult, face: u32) -> Vec<InputSub> {
    r.history
        .sources_of(face_sub(face))
        .into_iter()
        .filter(|(i, rel)| i.sub.kind == SubKind::Edge && matches!(rel, Relation::Generated(_)))
        .map(|(i, _)| i)
        .collect()
}

/// Lateral faces come from the section edges (Done-when 4): every lateral face of a pipe is
/// `Generated` from a profile edge, every profile edge sweeps at least one face; every lateral
/// face of a loft from an edge of a section wire, and every section edge bounds one. Caps carry
/// `StartCap`/`EndCap` (pipes) and `LoftSection { 0 | n − 1 }` (lofts).
#[test]
fn lateral_faces_come_from_the_section_edges() {
    let k = kernel();
    let mut coverage = BTreeMap::new();
    for c in corpus::corpus(&k) {
        let r = run_ok(&k, &c);
        let laterals: Vec<u32> = r
            .history
            .roles
            .iter()
            .filter(|(_, role)| *role == OpRole::Lateral)
            .map(|(s, _)| s.index)
            .collect();
        assert!(!laterals.is_empty(), "{}: no lateral face", c.name);
        let (sections, n_sections) = match &c.op {
            SweepOp::Pipe { profiles, .. } => (*profiles, *profiles),
            SweepOp::Loft { sections, .. } => (c.operands.len(), sections.len()),
        };
        // Which section operands' edges reach the laterals.
        let mut covered = vec![0u32; sections];
        for &f in &laterals {
            let from = generated_from(&r, f);
            assert!(
                from.iter().any(|i| usize::from(i.operand) < sections),
                "{}: lateral face {f} has no section edge source ({from:?})",
                c.name
            );
            for i in from {
                if usize::from(i.operand) < sections {
                    covered[usize::from(i.operand)] += 1;
                }
            }
        }
        // Every edge of the first section wire sweeps a face.
        let first = (0..c.operands.len().min(sections))
            .find(|&o| c.operands[o].kind() != amg_brep::ShapeKind::Vertex)
            .unwrap();
        for e in c.operands[first].edges() {
            let input = InputSub {
                operand: u16::try_from(first).unwrap(),
                sub: SubShape {
                    kind: SubKind::Edge,
                    index: e.0,
                },
            };
            let faces = r.history.of_input(input).any(|rel| match rel {
                Relation::Generated(out) => out.iter().any(|o| o.kind == SubKind::Face),
                _ => false,
            });
            assert!(
                faces,
                "{}: edge {} of section {first} sweeps nothing",
                c.name, e.0
            );
        }
        // Caps.
        let caps = caps(&r);
        if c.solid() {
            let want: Vec<OpRole> = if c.is_pipe() {
                vec![OpRole::StartCap, OpRole::EndCap]
            } else {
                vec![
                    OpRole::LoftSection { index: 0 },
                    OpRole::LoftSection {
                        index: u32::try_from(n_sections - 1).unwrap(),
                    },
                ]
            };
            let mut got: Vec<OpRole> = caps.iter().map(|(_, r)| r.clone()).collect();
            got.dedup();
            // Point sections have no cap face.
            for role in &got {
                assert!(want.contains(role), "{}: cap role {role:?}", c.name);
            }
            let points = match &c.op {
                SweepOp::Loft { sections, .. } => sections
                    .iter()
                    .filter(|s| matches!(s, LoftSection::Point(_)))
                    .count(),
                SweepOp::Pipe { profiles, .. } => c.operands[..*profiles]
                    .iter()
                    .filter(|s| s.kind() == amg_brep::ShapeKind::Vertex)
                    .count(),
            };
            // A closed spine (the support case's rim) has no caps.
            let closed_spine = match &c.op {
                SweepOp::Pipe { profiles, .. } => {
                    let spine = &c.operands[*profiles];
                    spine.count(SubKind::Edge) == 1 && {
                        let ends = k.edge_ends(spine, amg_brep::EdgeId(0)).unwrap();
                        (ends.p0 - ends.p1).length() < 1e-9
                    }
                }
                SweepOp::Loft { .. } => false,
            };
            let want_caps = if closed_spine { 0 } else { 2 - points };
            assert_eq!(caps.len(), want_caps, "{}: caps {caps:?}", c.name);
        } else {
            assert!(caps.is_empty(), "{}: a shell has no caps", c.name);
        }
        coverage.insert(c.name, covered);
    }
    // The record for DOC-06 (naming note Q2): which sections' edges OCCT reports.
    for (name, covered) in &coverage {
        eprintln!("{name}: lateral faces generated per section operand {covered:?}");
    }
    // The ruled squares: each lateral face is generated from the two sections it joins.
    let r = run_ok(&k, case(&corpus::corpus(&k), "loft_squares_ruled"));
    for (s, role) in &r.history.roles {
        if *role != OpRole::Lateral {
            continue;
        }
        let mut operands: Vec<u16> = generated_from(&r, s.index)
            .iter()
            .map(|i| i.operand)
            .collect();
        operands.sort_unstable();
        operands.dedup();
        assert!(
            operands == [0, 1] || operands == [1, 2],
            "face {}: from sections {operands:?}",
            s.index
        );
    }
}

/// Incompatible sections and impossible sweeps fail with `OpFailed` (or `InvalidInput` for what
/// amg-brep checks itself) and never crash (Done-when 5).
#[test]
fn incompatible_sections_fail_cleanly() {
    let k = kernel();
    let open = polyline(&k, &[p(-2.0, 0.0, 5.0), p(2.0, 0.0, 5.0)]);
    let circle0 = circle(&k, xy(0.0), 2.0);
    let square5 = rectangle(&k, xy(5.0), 3.0, 3.0);
    let loft = |wires: Vec<Shape>, opts: ThruSectionsOpts| {
        let sections: Vec<LoftSection> = (0..wires.len()).map(LoftSection::Wire).collect();
        k.thru_sections(&wires, &sections, &opts, &ctx())
    };
    let op_failed = |name: &str, r: Result<OpResult, KernelError>| match r {
        Err(KernelError::OpFailed { op, message, .. }) => {
            eprintln!("{name}: OpFailed {op}: {message}");
        }
        other => panic!("{name}: {other:?}"),
    };
    // A closed and an open section.
    op_failed(
        "closed and open",
        loft(
            vec![circle0.clone(), open.clone()],
            ThruSectionsOpts::default(),
        ),
    );
    op_failed(
        "closed and open, ruled",
        loft(
            vec![circle0.clone(), open.clone()],
            ThruSectionsOpts {
                ruled: true,
                ..ThruSectionsOpts::default()
            },
        ),
    );
    // Different edge counts without the compatibility check.
    op_failed(
        "edge counts",
        loft(
            vec![circle0.clone(), square5.clone()],
            ThruSectionsOpts {
                check_compatibility: false,
                ..ThruSectionsOpts::default()
            },
        ),
    );
    // Two coincident sections.
    op_failed(
        "coincident sections",
        loft(
            vec![circle0.clone(), circle(&k, xy(0.0), 2.0)],
            ThruSectionsOpts::default(),
        ),
    );
    // A solid pipe of an open profile.
    op_failed(
        "open profile solid",
        k.pipe(
            &[
                open.clone(),
                polyline(&k, &[p(0.0, 0.0, 5.0), p(0.0, 0.0, 20.0)]),
            ],
            1,
            &PipeOpts::default(),
            &ctx(),
        ),
    );
    // A guide that the spine's normal planes never meet.
    op_failed(
        "guide out of reach",
        k.pipe(
            &[
                rectangle(&k, xy(0.0), 2.0, 2.0),
                polyline(&k, &[p(0.0, 0.0, 0.0), p(0.0, 0.0, 20.0)]),
                polyline(&k, &[p(5.0, 0.0, 30.0), p(5.0, 0.0, 50.0)]),
            ],
            1,
            &PipeOpts {
                mode: PipeMode::Guide {
                    wire: 2,
                    contact: false,
                },
                ..PipeOpts::default()
            },
            &ctx(),
        ),
    );
    // A support whose faces do not carry the spine.
    let support = k
        .make_box(&amg_brep::Plane::XY, Vec3::splat(5.0))
        .unwrap()
        .shape;
    match k.pipe(
        &[
            circle(&k, xy(0.0), 1.0),
            polyline(&k, &[p(0.0, 0.0, 0.0), p(0.0, 0.0, 20.0)]),
            support,
        ],
        1,
        &PipeOpts {
            mode: PipeMode::Support { shell: 2 },
            ..PipeOpts::default()
        },
        &ctx(),
    ) {
        Err(KernelError::InvalidInput { what, reason, .. }) => {
            assert_eq!(what, "operands");
            eprintln!("support without the spine: InvalidInput: {reason}");
        }
        other => panic!("support without the spine: {other:?}"),
    }
}

#[test]
fn bad_inputs_are_invalid_input() {
    let k = kernel();
    let c = circle(&k, xy(0.0), 2.0);
    let spine = polyline(&k, &[p(0.0, 0.0, 0.0), p(0.0, 0.0, 10.0)]);
    let solid = k.make_box(&amg_brep::Plane::XY, Vec3::ONE).unwrap().shape;
    let what = |r: Result<OpResult, KernelError>| match r {
        Err(KernelError::InvalidInput { what, .. }) => what,
        other => panic!("{other:?}"),
    };
    let pipe = |operands: &[Shape], profiles: usize, opts: PipeOpts| {
        what(k.pipe(operands, profiles, &opts, &ctx()))
    };
    let d = PipeOpts::default;
    assert_eq!(pipe(&[c.clone(), spine.clone()], 0, d()), "profiles");
    assert_eq!(pipe(std::slice::from_ref(&c), 1, d()), "operands");
    assert_eq!(
        pipe(&[c.clone(), spine.clone(), spine.clone()], 1, d()),
        "operands"
    );
    assert_eq!(pipe(&[solid.clone(), spine.clone()], 1, d()), "profiles");
    assert_eq!(pipe(&[c.clone(), solid.clone()], 1, d()), "spine");
    let guide = |wire| PipeOpts {
        mode: PipeMode::Guide {
            wire,
            contact: false,
        },
        ..d()
    };
    assert_eq!(pipe(&[c.clone(), spine.clone()], 1, guide(2)), "operands");
    assert_eq!(
        pipe(&[c.clone(), spine.clone(), spine.clone()], 1, guide(0)),
        "guide"
    );
    assert_eq!(
        pipe(&[c.clone(), spine.clone(), solid.clone()], 1, guide(2)),
        "guide"
    );
    let law = |twist: Option<Vec<(f64, f64)>>, scale: Option<Vec<(f64, f64)>>, mode| PipeOpts {
        mode,
        twist,
        scale,
        ..d()
    };
    let cf = PipeMode::CorrectedFrenet;
    for bad in [
        vec![(0.0, 0.0)],
        vec![(0.0, 0.0), (0.5, 1.0)],
        vec![(0.0, 0.0), (1.0, f64::NAN)],
        vec![(0.0, 0.0), (0.6, 1.0), (0.4, 1.0), (1.0, 1.0)],
    ] {
        assert_eq!(
            pipe(
                &[c.clone(), spine.clone()],
                1,
                law(Some(bad.clone()), None, cf.clone())
            ),
            "twist"
        );
        assert_eq!(
            pipe(
                &[c.clone(), spine.clone()],
                1,
                law(None, Some(bad), cf.clone())
            ),
            "scale"
        );
    }
    let up = vec![(0.0, 1.0), (1.0, 2.0)];
    assert_eq!(
        pipe(
            &[c.clone(), spine.clone()],
            1,
            law(None, Some(vec![(0.0, 1.0), (1.0, 0.0)]), cf.clone())
        ),
        "scale"
    );
    assert_eq!(
        pipe(
            &[c.clone(), spine.clone()],
            1,
            law(Some(up.clone()), Some(up.clone()), cf.clone())
        ),
        "scale"
    );
    assert_eq!(
        pipe(
            &[c.clone(), spine.clone()],
            1,
            law(
                Some(up.clone()),
                None,
                PipeMode::Fixed {
                    binormal: Dir3::new_unchecked(Vec3::Y)
                }
            )
        ),
        "twist"
    );
    assert_eq!(
        pipe(
            &[c.clone(), c.clone(), spine.clone()],
            2,
            law(None, Some(up), cf)
        ),
        "scale"
    );
    // A point section in the middle.
    let v = corpus::vertex(&k, p(0.0, 0.0, 5.0));
    assert_eq!(
        pipe(&[c.clone(), v, c.clone(), spine.clone()], 3, d()),
        "profiles"
    );

    let lofts = |wires: &[Shape], sections: &[LoftSection], opts: ThruSectionsOpts| {
        what(k.thru_sections(wires, sections, &opts, &ctx()))
    };
    let t = ThruSectionsOpts::default;
    let w = LoftSection::Wire;
    let pt = LoftSection::Point(p(0.0, 0.0, 9.0));
    assert_eq!(lofts(std::slice::from_ref(&c), &[w(0)], t()), "sections");
    assert_eq!(
        lofts(std::slice::from_ref(&c), &[w(0), w(1)], t()),
        "sections"
    );
    assert_eq!(
        lofts(&[c.clone(), spine.clone()], &[w(0), w(1), w(1)], t()),
        "sections"
    );
    assert_eq!(
        lofts(&[c.clone(), c.clone()], &[w(0), w(0), w(1)], t()),
        "sections"
    );
    assert_eq!(
        lofts(&[c.clone(), c.clone()], &[w(0), pt.clone(), w(1)], t()),
        "sections"
    );
    assert_eq!(lofts(&[], &[pt.clone(), pt.clone()], t()), "sections");
    assert_eq!(lofts(&[solid], &[w(0), pt.clone()], t()), "wires");
    assert_eq!(
        lofts(
            std::slice::from_ref(&c),
            &[w(0), LoftSection::Point(p(f64::NAN, 0.0, 0.0))],
            t()
        ),
        "sections"
    );
    assert_eq!(
        lofts(
            &[c],
            &[w(0), pt],
            ThruSectionsOpts {
                max_degree: 26,
                ..t()
            }
        ),
        "opts"
    );
}

#[test]
fn ten_runs_are_byte_identical() {
    let k = kernel();
    for case in corpus::corpus(&k) {
        let first = run_ok(&k, &case);
        let (bytes, history) = (bin(&first.shape), format!("{:?}", first.history));
        let inputs: Vec<Vec<u8>> = case.operands.iter().map(bin).collect();
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
        let after: Vec<Vec<u8>> = case.operands.iter().map(bin).collect();
        assert_eq!(after, inputs, "{}: an input changed", case.name);
    }
}

#[test]
fn cancelled_jobs_do_not_run() {
    let k = kernel();
    let cancelled = JobCtx::detached();
    cancelled.cancel_token().cancel();
    for case in corpus::corpus(&k)
        .iter()
        .take(3)
        .chain(corpus::corpus(&k).iter().rev().take(2))
    {
        assert!(
            matches!(case.run_with(&k, &cancelled), Err(KernelError::Cancelled)),
            "{}",
            case.name
        );
    }
}

/// 8 threads sweep and loft the same shared shapes: every result has the bytes of the
/// single-threaded one, and the inputs are unchanged (category C).
#[test]
fn shared_shapes_survive_concurrent_sweeps() {
    let k = kernel();
    let cases = corpus::corpus(&k);
    let want: Vec<Vec<u8>> = cases.iter().map(|c| bin(&run_ok(&k, c).shape)).collect();
    let inputs: Vec<Vec<Vec<u8>>> = cases
        .iter()
        .map(|c| c.operands.iter().map(bin).collect())
        .collect();
    std::thread::scope(|scope| {
        for t in 0..8 {
            let (k, cases, want) = (&k, &cases, &want);
            scope.spawn(move || {
                for i in 0..cases.len() {
                    let j = (i + t * 3) % cases.len();
                    let got = bin(&run_ok(k, &cases[j]).shape);
                    assert_eq!(got, want[j], "thread {t}: {}", cases[j].name);
                }
            });
        }
    });
    for (c, bytes) in cases.iter().zip(&inputs) {
        let now: Vec<Vec<u8>> = c.operands.iter().map(bin).collect();
        assert_eq!(&now, bytes, "{}", c.name);
    }
}

/// KH-05 runs the raw loft as its baseline through `&dyn BrepKernel`, also behind S20's
/// recording decorator.
#[test]
fn the_raw_loft_runs_behind_the_recording_kernel() {
    let k = kernel();
    let cases = corpus::corpus(&k);
    let dir = std::env::temp_dir().join(format!("amg-brep15-capture-{}", std::process::id()));
    let recorder = amg_brep::capture::FailureRecorder::new(amg_brep::capture::CaptureConfig {
        mode: amg_brep::capture::CaptureMode::Off,
        dir: dir.clone(),
        max_bytes: 1 << 20,
        max_dumps: 1,
    })
    .unwrap();
    let recording =
        amg_brep::capture::RecordingKernel::new(k.clone(), std::sync::Arc::new(recorder));
    let c = case(&cases, "loft_three_circles_smooth");
    let direct = run_ok(&k, c);
    let wrapped = run_ok(&recording, c);
    assert_eq!(bin(&direct.shape), bin(&wrapped.shape));
    let _ = std::fs::remove_dir_all(dir);
}

/// Cancellation reaches inside the loft build (through the progress range): a cancel 100 ms into
/// the ruled loft of 40 × 48 edges returns `Cancelled`. The bound on how soon is asserted only
/// under `AMG_STRICT_TIMING=1`.
#[test]
fn cancel_stops_a_running_loft() {
    let k = kernel();
    let (wires, sections) = corpus::heavy_loft(&k);
    let ruled = ThruSectionsOpts {
        ruled: true,
        ..ThruSectionsOpts::default()
    };
    let started = std::time::Instant::now();
    let full = k.thru_sections(&wires, &sections, &ruled, &ctx()).unwrap();
    let full_time = started.elapsed();
    assert_eq!(full.shape.count(SubKind::Face), 39 * 48 + 2);
    let job = JobCtx::detached();
    let token = job.cancel_token();
    let started = std::time::Instant::now();
    let canceller = std::thread::spawn(move || {
        std::thread::sleep(std::time::Duration::from_millis(100));
        token.cancel();
    });
    let r = k.thru_sections(&wires, &sections, &ruled, &job);
    canceller.join().unwrap();
    let elapsed = started.elapsed();
    assert!(matches!(r, Err(KernelError::Cancelled)), "{r:?}");
    eprintln!("cancelled after {elapsed:?}; the full run takes {full_time:?}");
    if std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1") {
        assert!(elapsed < full_time / 2, "{elapsed:?} vs {full_time:?}");
    }
}
