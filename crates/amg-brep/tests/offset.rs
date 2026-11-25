//! BREP-07: shell (thick solid with removed faces, per-face thickness, arc / intersection join),
//! offset and thicken, with the roles `ShellInner` / `OffsetFace`.
//!
//! Done-when 1 (golden volumes of the corpus in `common/offset_corpus.rs`, goldens
//! `tests/golden/brep/{shell,offset,thicken}_*.toml`, against closed forms within 1e-9
//! relative and the thickened B-spline face within 0.5 % of a quadrature), 2 (inward and
//! outward offsets of a sphere keep it a sphere of radius `R ± d` within 1e-9), 3 (a thickness
//! beyond the radius of curvature fails on exactly those faces), 4 (history roles on 6 cases).
//! The history completeness of the corpus is in `tests/history.rs`.
#![cfg(feature = "occt")]
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

mod common;
#[path = "common/offset_corpus.rs"]
mod corpus;
#[path = "common/parts.rs"]
mod parts;

use std::collections::BTreeMap;
use std::path::PathBuf;

use amg_brep::geom::Surface;
use amg_brep::{
    BrepKernel, Classification, FaceId, InputSub, JobCtx, KernelConfig, KernelError, OcctKernel,
    OffsetJoin, OpResult, OpRole, Plane, Point3, Relation, Shape, ShellOpts, SubKind, SubShape,
    Validate,
};

use corpus::{A, B, C, Case, OffsetOp, block, face_at, shell_opts};

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

fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

// ---- goldens -----------------------------------------------------------------------------

#[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
struct OffsetGolden {
    tolerances: GoldenTolerances,
    metrics: OffsetMetrics,
}

#[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
struct GoldenTolerances {
    /// Relative bound on volume and area.
    rel: f64,
}

#[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
struct OffsetMetrics {
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

fn metrics(k: &OcctKernel, s: &Shape) -> OffsetMetrics {
    let m = k.mass_props(s).unwrap();
    OffsetMetrics {
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
        let golden = OffsetGolden {
            tolerances: GoldenTolerances { rel: REL },
            metrics,
        };
        let text = format!(
            "# B-rep metrics of brep/{case} (S02 BREP-07). Written by `AMG_BLESS=1`.\n\n{}",
            toml::to_string(&golden).unwrap()
        );
        std::fs::write(&path, text).unwrap();
        return;
    }
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("{}: {e}; bless it with AMG_BLESS=1", path.display()));
    let golden: OffsetGolden = toml::from_str(&text).unwrap();
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

fn case<'a>(cases: &'a [Case], name: &str) -> &'a Case {
    cases.iter().find(|c| c.name == name).unwrap()
}

// ---- Done-when 1 -------------------------------------------------------------------------

/// Every corpus case gives one valid solid matching its golden, and its reference volume: the
/// closed forms within 1e-9 relative, the thickened B-spline face within 0.5 %.
#[test]
fn golden_shells_offsets_and_thickens_match_the_reference_volumes() {
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
                "{}: volume {volume} vs reference {want} ({err:.2e})",
                case.name
            );
            assert!(err <= rel, "{}: {volume} vs {want} ({err:e})", case.name);
            analytic += 1;
        } else {
            eprintln!("{}: volume {volume} (golden only)", case.name);
        }
    }
    eprintln!(
        "{} cases, {analytic} against reference volumes",
        cases.len()
    );
    // The headline case of the Done-when: one face removed, 1 mm, 1e-9.
    let r = run_ok(&k, case(&cases, "shell_box_top_inward"));
    let want = A * B * C - (A - 2.0) * (B - 2.0) * (C - 1.0);
    let got = k.mass_props(&r.shape).unwrap().volume;
    assert!((got - want).abs() / want <= 1e-9, "{got} vs {want}");
}

/// A shell without removed faces is a solid with a closed inner void (S04 R-S04-8).
#[test]
fn a_shell_without_removed_faces_is_hollow() {
    let k = kernel();
    let b = block(&k);
    for t in [-1.0, 1.0] {
        let r = k
            .shell(&b, &[], &shell_opts(t, OffsetJoin::Intersection), &ctx())
            .unwrap();
        assert!(r.shape.count(SubKind::Face) >= 12);
        // The centre is in the void, a point inside the wall is material.
        let centre = p(A / 2.0, B / 2.0, C / 2.0);
        assert_eq!(
            k.classify(&r.shape, centre, 1e-7).unwrap(),
            Classification::Outside,
            "t = {t}"
        );
        let wall = if t < 0.0 {
            p(0.5, B / 2.0, C / 2.0)
        } else {
            p(-0.5, B / 2.0, C / 2.0)
        };
        assert_eq!(
            k.classify(&r.shape, wall, 1e-7).unwrap(),
            Classification::Inside,
            "t = {t}"
        );
    }
}

/// Positive thicknesses go along the face normal, negative ones against it.
#[test]
fn thicken_follows_the_normal() {
    let k = kernel();
    let f = corpus::plane_face(&k);
    for (t, lo, hi) in [(2.0, 0.0, 2.0), (-2.0, -2.0, 0.0)] {
        let r = k.thicken(&f, t, OffsetJoin::Arc, &ctx()).unwrap();
        let bb = r.shape.bbox();
        assert!(
            (bb.min.z - lo).abs() < 1e-6 && (bb.max.z - hi).abs() < 1e-6,
            "t = {t}: {bb:?}"
        );
    }
}

// ---- Done-when 2 -------------------------------------------------------------------------

/// Inward and outward offsets of a sphere are spheres of radius `R ± d` about the same centre,
/// within 1e-9, with both joins.
#[test]
fn offsets_of_a_sphere_keep_its_centre_and_radius() {
    let k = kernel();
    let s = corpus::sphere(&k);
    for join in [OffsetJoin::Arc, OffsetJoin::Intersection] {
        for d in [2.0, 0.5, -0.5, -2.0, -4.5] {
            let r = k.offset(&s, d, join, 0.0, &ctx()).unwrap();
            assert_eq!(r.shape.count(SubKind::Face), 1);
            let Surface::Sphere { frame, radius } = r.shape.surface(FaceId(0)) else {
                panic!("{:?}", r.shape.surface(FaceId(0)));
            };
            let want = corpus::SPHERE + d;
            assert!(
                (radius - want).abs() <= 1e-9 * want,
                "d = {d}: radius {radius} vs {want}"
            );
            assert!(frame.origin.length() <= 1e-9, "{:?}", frame.origin);
            let v = k.mass_props(&r.shape).unwrap().volume;
            let vw = 4.0 / 3.0 * std::f64::consts::PI * want.powi(3);
            assert!((v - vw).abs() <= 1e-9 * vw, "d = {d}: volume {v} vs {vw}");
        }
    }
}

// ---- Done-when 3 -------------------------------------------------------------------------

fn local_failure(e: KernelError) -> (&'static str, String, Vec<SubShape>) {
    match e {
        KernelError::LocalOpFailed {
            op,
            message,
            offending,
            partial: _,
        } => (op, message, offending),
        other => panic!("not a local failure: {other:?}"),
    }
}

/// A wall thicker than a convex radius (or a thicken beyond a concave one) collapses the offset
/// of those faces: `LocalOpFailed` names exactly them, and the message gives the radius.
#[test]
fn a_thickness_beyond_the_radius_fails_on_those_faces() {
    let k = kernel();
    // Cylinder r = 6: the lateral face, not the planar caps.
    let cyl = corpus::cylinder(&k);
    let top = face_at(&k, &cyl, p(0.0, 0.0, corpus::H));
    let lateral: Vec<FaceId> = cyl
        .faces()
        .filter(|&f| matches!(cyl.surface(f), Surface::Cylinder { .. }))
        .collect();
    assert_eq!(lateral.len(), 1);
    let e = k
        .shell(&cyl, &[top], &shell_opts(-7.0, OffsetJoin::Arc), &ctx())
        .unwrap_err();
    let (op, message, offending) = local_failure(e);
    assert_eq!(op, "shell");
    assert_eq!(offending, vec![face_sub(lateral[0])], "{message}");
    assert!(message.contains("radius 6"), "{message}");
    // Just inside the radius it works.
    k.shell(&cyl, &[top], &shell_opts(-5.5, OffsetJoin::Arc), &ctx())
        .unwrap();
    // The same radius on one face only, through the per-face thickness.
    let e = k
        .shell(
            &cyl,
            &[top],
            &ShellOpts {
                per_face: vec![(lateral[0], -6.5)],
                ..shell_opts(-1.0, OffsetJoin::Arc)
            },
            &ctx(),
        )
        .unwrap_err();
    assert_eq!(local_failure(e).2, vec![face_sub(lateral[0])]);

    // Filleted box: the 4 fillets (r = 3) collapse under a 4 mm wall, the planes do not.
    let fb = corpus::filleted_block(&k);
    let fillets: Vec<SubShape> = fb
        .faces()
        .filter(|&f| matches!(fb.surface(f), Surface::Cylinder { .. }))
        .map(face_sub)
        .collect();
    assert_eq!(fillets.len(), 4);
    let fb_top = corpus::top(&k, &fb);
    let e = k
        .shell(&fb, &[fb_top], &shell_opts(-4.0, OffsetJoin::Arc), &ctx())
        .unwrap_err();
    let (_, message, offending) = local_failure(e);
    assert_eq!(offending, fillets, "{message}");
    assert!(message.contains("radius 3"), "{message}");
    // Outward, the fillets grow: any thickness works.
    k.shell(&fb, &[fb_top], &shell_opts(4.0, OffsetJoin::Arc), &ctx())
        .unwrap();

    // Sphere r = 5: an inward offset of 5 or more, with either join.
    let s = corpus::sphere(&k);
    for d in [-5.0, -6.0] {
        for join in [OffsetJoin::Arc, OffsetJoin::Intersection] {
            let e = k.offset(&s, d, join, 0.0, &ctx()).unwrap_err();
            let (op, _, offending) = local_failure(e);
            assert_eq!(op, "offset");
            assert_eq!(offending, vec![face_sub(FaceId(0))]);
        }
    }

    // A cylinder patch (r = 10, normal outward) thickened inward by 10 or more.
    let patch = corpus::cylinder_patch(&k);
    let e = k
        .thicken(&patch, -12.0, OffsetJoin::Arc, &ctx())
        .unwrap_err();
    let (op, message, offending) = local_failure(e);
    assert_eq!(op, "thicken");
    assert_eq!(offending, vec![face_sub(FaceId(0))]);
    assert!(message.contains("radius 10"), "{message}");
    k.thicken(&patch, -9.0, OffsetJoin::Arc, &ctx()).unwrap();
}

// ---- Done-when 4 -------------------------------------------------------------------------

/// The input faces a result face comes from, by relation.
fn sources(r: &OpResult, f: u32) -> (Vec<SubShape>, Vec<SubShape>) {
    let (mut modified, mut generated) = (Vec::new(), Vec::new());
    for (input, rel) in r.history.sources_of(face_sub(FaceId(f))) {
        match rel {
            Relation::Modified(_) => modified.push(input.sub),
            Relation::Generated(_) => generated.push(input.sub),
            _ => {}
        }
    }
    (modified, generated)
}

fn plane_of(s: &Shape, f: FaceId) -> Plane {
    match s.surface(f) {
        Surface::Plane(pl) => pl,
        other => panic!("face {f:?} is {other:?}"),
    }
}

/// The distance between two parallel planes (asserts they are parallel).
fn plane_gap(a: &Plane, b: &Plane) -> f64 {
    let (na, nb) = (a.normal.get(), b.normal.get());
    assert!(na.cross(nb).length() < 1e-9, "{a:?} {b:?}");
    (b.origin - a.origin).dot(na).abs()
}

fn role_counts(r: &OpResult) -> BTreeMap<String, u32> {
    let mut m = BTreeMap::new();
    for (_, role) in &r.history.roles {
        *m.entry(format!("{role:?}")).or_insert(0) += 1;
    }
    m
}

/// Checks a shell or thicken of planar faces: no new face is `Modified` from anything, each
/// input face is `Modified` 1:1 into a result face with the same plane, and each planar offset
/// face is `Generated` from one input face whose plane lies `thickness(face)` away.
fn check_planar_skin(
    name: &str,
    r: &OpResult,
    input: &Shape,
    thickness: impl Fn(FaceId) -> f64,
    skin: OpRole,
) {
    let mut offset_faces = 0;
    for (sub, role) in &r.history.roles {
        let (modified, generated) = sources(r, sub.index);
        assert!(modified.is_empty(), "{name} {sub:?}: {modified:?}");
        if *role != skin || !matches!(r.shape.surface(FaceId(sub.index)), Surface::Plane(_)) {
            continue;
        }
        offset_faces += 1;
        let faces: Vec<&SubShape> = generated
            .iter()
            .filter(|s| s.kind == SubKind::Face)
            .collect();
        assert_eq!(faces.len(), 1, "{name} {sub:?}: {generated:?}");
        let src = FaceId(faces[0].index);
        let gap = plane_gap(
            &plane_of(input, src),
            &plane_of(&r.shape, FaceId(sub.index)),
        );
        assert!(
            (gap - thickness(src).abs()).abs() < 1e-9,
            "{sub:?} from {src:?}: gap {gap}"
        );
    }
    assert!(offset_faces > 0);
    for f in input.faces() {
        let modified: Vec<&Relation> = r
            .history
            .of_input(InputSub {
                operand: 0,
                sub: face_sub(f),
            })
            .filter(|rel| matches!(rel, Relation::Modified(_)))
            .collect();
        if let Some(Relation::Modified(out)) = modified.first() {
            assert_eq!(out.len(), 1, "{f:?}: {out:?}");
            let o = FaceId(out[0].index);
            assert!(r.history.role_of(out[0]).is_none(), "{f:?}");
            assert!(plane_gap(&plane_of(input, f), &plane_of(&r.shape, o)) < 1e-12);
        }
    }
}

#[test]
fn history_roles_follow_the_offset_skin() {
    let k = kernel();
    let cases = corpus::corpus(&k);

    // 1. Box, top removed, 1 mm inward: 5 outer faces Modified 1:1, 5 inner ShellInner each
    //    Generated from its outer face 1 mm away, the rim ring Lateral from the removed face.
    let c = case(&cases, "shell_box_top_inward");
    let r = run_ok(&k, c);
    let top = c.removed()[0];
    assert_eq!(r.shape.count(SubKind::Face), 11);
    let counts = role_counts(&r);
    assert_eq!(counts.get("ShellInner"), Some(&5), "{counts:?}");
    assert_eq!(counts.get("Lateral"), Some(&1), "{counts:?}");
    check_planar_skin(c.name, &r, &c.input, |_| 1.0, OpRole::ShellInner);
    let rim = r
        .history
        .roles
        .iter()
        .find(|(_, role)| *role == OpRole::Lateral)
        .unwrap()
        .0;
    assert!(sources(&r, rim.index).1.contains(&face_sub(top)));
    assert!(
        !r.history
            .of_input(InputSub {
                operand: 0,
                sub: face_sub(top),
            })
            .any(|rel| matches!(rel, Relation::Modified(_)))
    );

    // 2. Per-face thickness: each inner face lies at its own face's thickness.
    let c = case(&cases, "shell_box_per_face");
    let r = run_ok(&k, c);
    let OffsetOp::Shell(_, opts) = &c.op else {
        unreachable!()
    };
    let per_face = opts.per_face.clone();
    check_planar_skin(
        c.name,
        &r,
        &c.input,
        |f| {
            per_face
                .iter()
                .find(|(g, _)| *g == f)
                .map_or(1.0, |(_, t)| *t)
        },
        OpRole::ShellInner,
    );

    // 3. Hollow box grown outward with arcs: 6 faces kept 1:1, the skin is 6 planes from the
    //    faces, 12 cylinders from the edges and 8 spheres from the vertices, all ShellInner.
    let c = case(&cases, "shell_box_hollow_outward_arc");
    let r = run_ok(&k, c);
    let counts = role_counts(&r);
    assert_eq!(counts.get("ShellInner"), Some(&26), "{counts:?}");
    assert_eq!(counts.len(), 1, "{counts:?}");
    check_planar_skin(c.name, &r, &c.input, |_| 1.0, OpRole::ShellInner);
    let mut from_kind: BTreeMap<(SubKind, char), u32> = BTreeMap::new();
    for (sub, _) in &r.history.roles {
        let (_, generated) = sources(&r, sub.index);
        assert_eq!(generated.len(), 1, "{sub:?}: {generated:?}");
        let class = common::class_letter(r.shape.surface(FaceId(sub.index)).class());
        *from_kind.entry((generated[0].kind, class)).or_insert(0) += 1;
    }
    assert_eq!(
        from_kind,
        BTreeMap::from([
            ((SubKind::Face, 'P'), 6),
            ((SubKind::Edge, 'Y'), 12),
            ((SubKind::Vertex, 'S'), 8),
        ])
    );

    // 4. Offset box with arcs: every face OffsetFace; planes Modified from their face 1 mm
    //    away, cylinders Generated from edges, spheres from vertices.
    let c = case(&cases, "offset_box_arc");
    let r = run_ok(&k, c);
    assert_eq!(role_counts(&r).get("OffsetFace"), Some(&26));
    let mut by_relation: BTreeMap<(&str, SubKind), u32> = BTreeMap::new();
    for (sub, _) in &r.history.roles {
        let (modified, generated) = sources(&r, sub.index);
        for m in &modified {
            assert_eq!(m.kind, SubKind::Face);
            let gap = plane_gap(
                &plane_of(&c.input, FaceId(m.index)),
                &plane_of(&r.shape, FaceId(sub.index)),
            );
            assert!((gap - 1.0).abs() < 1e-9);
            *by_relation.entry(("modified", m.kind)).or_insert(0) += 1;
        }
        for g in &generated {
            *by_relation.entry(("generated", g.kind)).or_insert(0) += 1;
        }
    }
    assert_eq!(
        by_relation,
        BTreeMap::from([
            (("generated", SubKind::Edge), 12),
            (("generated", SubKind::Vertex), 8),
            (("modified", SubKind::Face), 6),
        ])
    );

    // 5. Sphere offset: its one face Modified into the offset sphere, OffsetFace.
    let r = run_ok(&k, case(&cases, "offset_sphere_in"));
    assert_eq!(
        r.history.roles,
        vec![(face_sub(FaceId(0)), OpRole::OffsetFace)]
    );
    assert_eq!(sources(&r, 0).0, vec![face_sub(FaceId(0))]);

    // 6. Thickened plane: the face Modified 1:1, its offset Generated from it (OffsetFace), the
    //    4 side walls Lateral, each Generated from one boundary edge.
    let c = case(&cases, "thicken_plane_up");
    let r = run_ok(&k, c);
    let counts = role_counts(&r);
    assert_eq!(counts.get("OffsetFace"), Some(&1), "{counts:?}");
    assert_eq!(counts.get("Lateral"), Some(&4), "{counts:?}");
    check_planar_skin(c.name, &r, &c.input, |_| 2.0, OpRole::OffsetFace);
    let mut wall_edges = Vec::new();
    for (sub, role) in &r.history.roles {
        if *role == OpRole::Lateral {
            let (_, generated) = sources(&r, sub.index);
            assert_eq!(generated.len(), 1, "{sub:?}: {generated:?}");
            assert_eq!(generated[0].kind, SubKind::Edge);
            wall_edges.push(generated[0].index);
        }
    }
    wall_edges.sort_unstable();
    assert_eq!(wall_edges, vec![0, 1, 2, 3]);

    // Every case of the corpus: skin faces have a source, the other new faces are Lateral.
    for c in &cases {
        let r = run_ok(&k, c);
        let skin = if c.is_shell() {
            OpRole::ShellInner
        } else {
            OpRole::OffsetFace
        };
        for (sub, role) in &r.history.roles {
            assert!(
                *role == skin || *role == OpRole::Lateral,
                "{}: {sub:?} {role:?}",
                c.name
            );
            if *role == skin {
                let (m, g) = sources(&r, sub.index);
                assert!(!m.is_empty() || !g.is_empty(), "{}: {sub:?}", c.name);
            }
        }
        if c.is_offset() {
            assert_eq!(
                r.history.roles.len() as u32,
                r.shape.count(SubKind::Face),
                "{}",
                c.name
            );
        }
    }
}

// ---- determinism, input checks, cancellation, threads -------------------------------------

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
fn bad_inputs_are_invalid_input() {
    let k = kernel();
    let b = block(&k);
    let top = corpus::top(&k, &b);
    let what = |r: Result<OpResult, KernelError>| match r {
        Err(KernelError::InvalidInput { what, .. }) => what,
        other => panic!("{other:?}"),
    };
    let shell = |remove: &[FaceId], opts: ShellOpts| k.shell(&b, remove, &opts, &ctx());
    let arc = |t| shell_opts(t, OffsetJoin::Arc);
    assert_eq!(what(shell(&[top], arc(0.0))), "opts");
    assert_eq!(what(shell(&[top], arc(f64::NAN))), "opts");
    assert_eq!(what(shell(&[top, top], arc(-1.0))), "remove");
    assert_eq!(what(shell(&[FaceId(6)], arc(-1.0))), "remove");
    let all: Vec<FaceId> = b.faces().collect();
    assert_eq!(what(shell(&all, arc(-1.0))), "remove");
    let with = |per_face: Vec<(FaceId, f64)>| ShellOpts {
        per_face,
        ..arc(-1.0)
    };
    assert_eq!(what(shell(&[top], with(vec![(top, -2.0)]))), "opts");
    assert_eq!(what(shell(&[top], with(vec![(FaceId(0), 2.0)]))), "opts");
    assert_eq!(
        what(shell(
            &[top],
            with(vec![(FaceId(0), -2.0), (FaceId(0), -3.0)])
        )),
        "opts"
    );
    assert_eq!(
        what(shell(
            &[top],
            ShellOpts {
                tol: -1.0,
                ..arc(-1.0)
            }
        )),
        "tol"
    );
    // Shell needs a solid, thicken refuses one.
    let face = corpus::plane_face(&k);
    assert_eq!(what(k.shell(&face, &[], &arc(1.0), &ctx())), "shape");
    assert_eq!(what(k.thicken(&b, 1.0, OffsetJoin::Arc, &ctx())), "shape");
    assert_eq!(
        what(k.thicken(&face, 0.0, OffsetJoin::Arc, &ctx())),
        "thickness"
    );
    assert_eq!(
        what(k.offset(&b, 0.0, OffsetJoin::Arc, 0.0, &ctx())),
        "distance"
    );
    assert_eq!(
        what(k.offset(&b, 1.0, OffsetJoin::Arc, f64::INFINITY, &ctx())),
        "tol"
    );
    // A two-solid compound is not one solid.
    let other = k
        .make_box(
            &Plane {
                origin: p(50.0, 0.0, 0.0),
                ..Plane::XY
            },
            amg_brep::Vec3::splat(5.0),
        )
        .unwrap()
        .shape;
    let two = k
        .boolean(
            amg_brep::BoolOp::Fuse,
            std::slice::from_ref(&b),
            &[other],
            &amg_brep::BoolOpts::default(),
            &ctx(),
        )
        .unwrap()
        .shape;
    assert_eq!(two.solids().len(), 2);
    assert_eq!(what(k.shell(&two, &[], &arc(-1.0), &ctx())), "shape");
    // OCCT offsets connected shapes only: two loose solids fail as a whole.
    let (op, message, _) = local_failure(
        k.offset(&two, 1.0, OffsetJoin::Intersection, 0.0, &ctx())
            .unwrap_err(),
    );
    assert_eq!(op, "offset");
    assert!(message.contains("NotConnectedShell"), "{message}");
}

#[test]
fn cancelled_jobs_do_not_run() {
    let k = kernel();
    let b = block(&k);
    let top = corpus::top(&k, &b);
    let cancelled = JobCtx::detached();
    cancelled.cancel_token().cancel();
    assert!(matches!(
        k.shell(&b, &[top], &shell_opts(-1.0, OffsetJoin::Arc), &cancelled),
        Err(KernelError::Cancelled)
    ));
    assert!(matches!(
        k.offset(&b, 1.0, OffsetJoin::Arc, 0.0, &cancelled),
        Err(KernelError::Cancelled)
    ));
    let f = corpus::plane_face(&k);
    assert!(matches!(
        k.thicken(&f, 1.0, OffsetJoin::Arc, &cancelled),
        Err(KernelError::Cancelled)
    ));
}

/// 8 threads shell, offset and thicken the same shared shapes: every result has the bytes of
/// the single-threaded one, and the inputs are unchanged.
#[test]
fn shared_shapes_survive_concurrent_offsets() {
    let k = kernel();
    let cases = corpus::corpus(&k);
    let want: Vec<Vec<u8>> = cases.iter().map(|c| bin(&run_ok(&k, c).shape)).collect();
    let inputs: Vec<Vec<u8>> = cases.iter().map(|c| bin(&c.input)).collect();
    std::thread::scope(|scope| {
        for t in 0..8 {
            let (k, cases, want) = (&k, &cases, &want);
            scope.spawn(move || {
                for i in 0..cases.len() * 2 {
                    let j = (i + t * 3) % cases.len();
                    let got = bin(&run_ok(k, &cases[j]).shape);
                    assert_eq!(got, want[j], "thread {t}: {}", cases[j].name);
                }
            });
        }
    });
    for (c, bytes) in cases.iter().zip(&inputs) {
        assert_eq!(&bin(&c.input), bytes, "{}", c.name);
    }
}

/// Cancellation reaches inside the offset build (through the progress range): the 1030-face plate
/// takes seconds to offset, a cancel 200 ms in stops it with `Cancelled`.
#[test]
fn cancel_stops_a_running_offset() {
    let k = kernel();
    let plate = parts::plate(&k, 32, Point3::ZERO);
    let job = JobCtx::detached();
    let token = job.cancel_token();
    let started = std::time::Instant::now();
    let canceller = std::thread::spawn(move || {
        std::thread::sleep(std::time::Duration::from_millis(200));
        token.cancel();
    });
    let r = k.offset(&plate, 0.1, OffsetJoin::Intersection, 0.0, &job);
    canceller.join().unwrap();
    let elapsed = started.elapsed();
    assert!(
        matches!(r, Err(KernelError::Cancelled)),
        "{:?}",
        r.map(|_| ())
    );
    eprintln!("cancelled after {elapsed:?}");
    if std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1") {
        assert!(elapsed < std::time::Duration::from_secs(2), "{elapsed:?}");
    }
}
