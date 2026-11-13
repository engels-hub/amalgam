//! BREP-14: primitives on a `Plane` frame, `prism` (vector, infinite, taper, `merge_coplanar`),
//! `revol`, `gtransformed` and `mirrored`.
//!
//! Done-when 1 (analytic volume, area and topology of every primitive, of prisms of five profile
//! kinds and of revols at 90° and 360°, within 1e-9 relative, with goldens in
//! `tests/golden/brep/ops_*.toml`), 2 (a square tapered at 5° against the analytic frustum) and 3
//! (every result BRepCheck-valid: the kernel runs with `Validate::Always`). Done-when 4 is in
//! `tests/stress.rs`. Timing assertions run only with `AMG_STRICT_TIMING=1`.
#![cfg(feature = "occt")]
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

mod common;

use std::f64::consts::{FRAC_PI_2, PI, TAU};
use std::path::PathBuf;

use amg_brep::geom::{Curve2d, Loop2, PlanarProfile, ProfileEdge2, Surface, Tag};
use amg_brep::{
    BrepKernel, Dir3, FaceId, InputSub, JobCtx, KernelConfig, KernelError, Line3, OcctKernel,
    OpResult, OpRole, Plane, Point2, Point3, PrismExtent, PrismOpts, Relation, Shape, ShapeKind,
    SubKind, SubShape, Tolerance, Transform, Validate, Vec2, Vec3,
};
use amg_core::Quat;

const TOL: Tolerance = Tolerance::DEFAULT;

/// Done-when 1 and 2 bound.
const REL: f64 = 1e-9;

fn kernel() -> OcctKernel {
    OcctKernel::new(KernelConfig {
        validate: Validate::Always,
        ..KernelConfig::default()
    })
}

fn ctx() -> JobCtx {
    JobCtx::detached()
}

/// A frame rotated off every axis and moved off the origin.
fn tilted() -> Plane {
    let q = Quat::from_euler(amg_core::glam::EulerRot::ZXY, 0.4, -0.7, 1.1);
    Plane {
        origin: Point3::new(3.0, -2.0, 5.0),
        x: Dir3::new_unchecked(q * Vec3::X),
        y: Dir3::new_unchecked(q * Vec3::Y),
        normal: Dir3::new_unchecked(q * Vec3::Z),
    }
}

fn area(s: &Shape) -> f64 {
    s.faces().map(|f| s.face_area(f)).sum()
}

fn counts(s: &Shape) -> [u32; 3] {
    [
        s.count(SubKind::Face),
        s.count(SubKind::Edge),
        s.count(SubKind::Vertex),
    ]
}

#[track_caller]
fn assert_rel(what: &str, got: f64, want: f64, rel: f64) {
    let err = (got - want).abs() / want.abs().max(f64::MIN_POSITIVE);
    assert!(
        err <= rel,
        "{what}: got {got}, want {want} (relative error {err:e} > {rel:e})"
    );
    eprintln!("{what}: {got} vs {want}, relative error {err:.2e}");
}

/// Volume and area against the analytic values, within [`REL`].
#[track_caller]
fn assert_measures(what: &str, s: &Shape, volume: f64, face_area: f64) {
    // An open shell has no volume to compare.
    if volume != 0.0 {
        assert_rel(&format!("{what} volume"), s.volume().unwrap(), volume, REL);
    }
    assert_rel(&format!("{what} area"), area(s), face_area, REL);
    assert!(
        kernel().check(s, &Default::default()).valid,
        "{what}: invalid"
    );
}

// ---- goldens -----------------------------------------------------------------------------

/// Exact B-rep metrics of one result in `tests/golden/brep/ops_<case>.toml`.
#[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
struct BrepGolden {
    tolerances: GoldenTolerances,
    metrics: BrepMetrics,
}

#[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
struct GoldenTolerances {
    /// Relative bound on volume and area.
    rel: f64,
}

#[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
struct BrepMetrics {
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
        .join(format!("ops_{case}.toml"))
}

/// Compares `s` with its golden; `AMG_BLESS=1` (never under `CI`) writes it instead.
#[track_caller]
fn check_golden(case: &str, s: &Shape) {
    let [faces, edges, vertices] = counts(s);
    let metrics = BrepMetrics {
        // An open shell's signed "volume" means nothing.
        volume: if s.kind() == ShapeKind::Shell {
            0.0
        } else {
            s.volume().unwrap()
        },
        area: area(s),
        faces,
        edges,
        vertices,
        classes: common::face_classes(s),
    };
    let path = golden_path(case);
    let bless = std::env::var("AMG_BLESS").is_ok_and(|v| v == "1");
    if bless {
        assert!(
            std::env::var_os("CI").is_none_or(|v| v.is_empty()),
            "blessing is refused under CI"
        );
        let golden = BrepGolden {
            tolerances: GoldenTolerances { rel: REL },
            metrics,
        };
        let text = format!(
            "# Exact B-rep metrics of brep/ops_{case} (S02 BREP-14). Written by `AMG_BLESS=1`.\n\n{}",
            toml::to_string(&golden).unwrap()
        );
        std::fs::write(&path, text).unwrap();
        return;
    }
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("{}: {e}; bless it with AMG_BLESS=1", path.display()));
    let golden: BrepGolden = toml::from_str(&text).unwrap();
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
    let close = |got: f64, want: f64| (got - want).abs() <= rel * want.abs().max(1.0);
    assert!(
        close(metrics.volume, want.volume),
        "{case}: volume {} vs golden {}",
        metrics.volume,
        want.volume
    );
    assert!(
        close(metrics.area, want.area),
        "{case}: area {} vs golden {}",
        metrics.area,
        want.area
    );
}

// ---- profiles ----------------------------------------------------------------------------

fn edges(curves: Vec<Curve2d>, first_tag: u64) -> Loop2 {
    Loop2 {
        edges: curves
            .into_iter()
            .zip(first_tag..)
            .map(|(curve, t)| ProfileEdge2 { curve, tag: Tag(t) })
            .collect(),
    }
}

fn polygon(points: &[(f64, f64)]) -> Vec<Curve2d> {
    (0..points.len())
        .map(|i| {
            let (a, b) = (points[i], points[(i + 1) % points.len()]);
            Curve2d::Line {
                a: Point2::new(a.0, a.1),
                b: Point2::new(b.0, b.1),
            }
        })
        .collect()
}

fn face(plane: Plane, outer: Vec<Curve2d>, holes: Vec<Vec<Curve2d>>) -> Shape {
    let profile = PlanarProfile {
        plane,
        outer: edges(outer, 0),
        holes: holes
            .into_iter()
            .enumerate()
            .map(|(i, h)| edges(h, 100 * (i as u64 + 1)))
            .collect(),
    };
    kernel()
        .make_planar_faces(&[profile], &TOL)
        .unwrap()
        .remove(0)
        .face
}

fn square(side: f64) -> Shape {
    face(
        Plane::XY,
        polygon(&[(0.0, 0.0), (side, 0.0), (side, side), (0.0, side)]),
        vec![],
    )
}

fn circle(center: (f64, f64), radius: f64) -> Vec<Curve2d> {
    vec![Curve2d::Circle {
        center: Point2::new(center.0, center.1),
        radius,
    }]
}

/// A slot: two lines of length `len` joined by half circles of radius `r` (tangent chain).
fn slot(len: f64, r: f64) -> Vec<Curve2d> {
    vec![
        Curve2d::Line {
            a: Point2::new(0.0, -r),
            b: Point2::new(len, -r),
        },
        Curve2d::Arc {
            center: Point2::new(len, 0.0),
            radius: r,
            start_angle: -FRAC_PI_2,
            sweep: PI,
        },
        Curve2d::Line {
            a: Point2::new(len, r),
            b: Point2::new(0.0, r),
        },
        Curve2d::Arc {
            center: Point2::ZERO,
            radius: r,
            start_angle: FRAC_PI_2,
            sweep: PI,
        },
    ]
}

/// Perimeter of an ellipse: the trapezoid rule is spectrally accurate for periodic integrands.
fn ellipse_perimeter(a: f64, b: f64) -> f64 {
    let n = 4096;
    let h = TAU / f64::from(n);
    (0..n)
        .map(|k| {
            let t = f64::from(k) * h;
            (a * a * t.sin().powi(2) + b * b * t.cos().powi(2)).sqrt()
        })
        .sum::<f64>()
        * h
}

fn straight(v: Vec3) -> (PrismExtent, PrismOpts) {
    (
        PrismExtent::Vector(v),
        PrismOpts {
            taper: 0.0,
            merge_coplanar: false,
        },
    )
}

fn prism(profile: &Shape, v: Vec3) -> OpResult {
    let (extent, opts) = straight(v);
    kernel().prism(profile, &extent, &opts, &ctx()).unwrap()
}

fn tapered(profile: &Shape, v: Vec3, taper: f64) -> KernelResult {
    kernel().prism(
        profile,
        &PrismExtent::Vector(v),
        &PrismOpts {
            taper,
            merge_coplanar: false,
        },
        &ctx(),
    )
}

type KernelResult = Result<OpResult, KernelError>;

// ---- Done-when 1: primitives ---------------------------------------------------------------

#[test]
fn golden_primitives_match_the_analytic_values() {
    let k = kernel();
    let frame = tilted();

    let block = k.make_box(&frame, Vec3::new(10.0, 20.0, 30.0)).unwrap();
    assert_measures("box", &block.shape, 6000.0, 2200.0);
    assert_eq!(counts(&block.shape), [6, 12, 8]);
    assert_eq!(block.shape.kind(), ShapeKind::Solid);
    assert_eq!(block.history, Default::default());
    // The corner sits at the frame origin and the far corner at size along the frame axes.
    let far =
        frame.origin + 10.0 * frame.x.get() + 20.0 * frame.y.get() + 30.0 * frame.normal.get();
    let corners: Vec<Point3> = block
        .shape
        .vertices()
        .map(|v| block.shape.vertex_point(v))
        .collect();
    assert!(corners.iter().any(|p| (*p - frame.origin).length() < 1e-12));
    assert!(corners.iter().any(|p| (*p - far).length() < 1e-12));
    check_golden("box", &block.shape);

    let (r, h) = (5.0, 20.0);
    let cyl = k.make_cylinder(&frame, r, h, TAU).unwrap().shape;
    assert_measures("cylinder", &cyl, PI * r * r * h, TAU * r * h + TAU * r * r);
    assert_eq!(counts(&cyl), [3, 3, 2]);
    check_golden("cylinder", &cyl);

    let quarter = k.make_cylinder(&frame, r, h, FRAC_PI_2).unwrap().shape;
    assert_measures(
        "cylinder 90°",
        &quarter,
        PI * r * r * h / 4.0,
        FRAC_PI_2 * r * h + FRAC_PI_2 * r * r + 2.0 * r * h,
    );
    check_golden("cylinder_quarter", &quarter);

    let (r1, r2, hc): (f64, f64, f64) = (5.0, 2.0, 10.0);
    let slant = (hc * hc + (r1 - r2) * (r1 - r2)).sqrt();
    let cone = k.make_cone(&frame, r1, r2, hc, TAU).unwrap().shape;
    assert_measures(
        "cone frustum",
        &cone,
        PI * hc / 3.0 * (r1 * r1 + r1 * r2 + r2 * r2),
        PI * (r1 + r2) * slant + PI * (r1 * r1 + r2 * r2),
    );
    check_golden("cone", &cone);
    let apex = k.make_cone(&frame, 4.0, 0.0, 6.0, TAU).unwrap().shape;
    assert_measures(
        "full cone",
        &apex,
        PI * 16.0 * 6.0 / 3.0,
        PI * 4.0 * 52.0_f64.sqrt() + PI * 16.0,
    );
    check_golden("cone_apex", &apex);

    let rs = 7.0;
    let sphere = k.make_sphere(&frame, rs).unwrap().shape;
    assert_measures(
        "sphere",
        &sphere,
        4.0 / 3.0 * PI * rs.powi(3),
        4.0 * PI * rs * rs,
    );
    // The centre is the frame origin.
    let bbox = sphere.bbox();
    assert!(((bbox.min + bbox.max) / 2.0 - frame.origin).length() < 1e-6);
    check_golden("sphere", &sphere);

    let (big, small) = (10.0, 3.0);
    let torus = k.make_torus(&frame, big, small, TAU).unwrap().shape;
    assert_measures(
        "torus",
        &torus,
        2.0 * PI * PI * big * small * small,
        4.0 * PI * PI * big * small,
    );
    check_golden("torus", &torus);
    let half = k.make_torus(&frame, big, small, PI).unwrap().shape;
    assert_measures(
        "torus 180°",
        &half,
        PI * PI * big * small * small,
        2.0 * PI * PI * big * small + 2.0 * PI * small * small,
    );
    check_golden("torus_half", &half);
}

#[test]
fn primitive_frames_place_the_geometry() {
    let k = kernel();
    let frame = tilted();
    let cyl = k.make_cylinder(&frame, 5.0, 20.0, TAU).unwrap().shape;
    let lateral = cyl
        .faces()
        .find_map(|f| match cyl.surface(f) {
            Surface::Cylinder { frame, radius } => Some((frame, radius)),
            _ => None,
        })
        .unwrap();
    assert!((lateral.1 - 5.0).abs() < 1e-12);
    // Same axis line as the frame normal through the frame origin.
    let axis = lateral.0.normal.get();
    assert!(axis.cross(frame.normal.get()).length() < 1e-12);
    assert!((lateral.0.origin - frame.origin).cross(axis).length() < 1e-12);
}

#[test]
fn bad_primitive_parameters_are_invalid_input() {
    let k = kernel();
    let f = Plane::XY;
    let invalid = |r: KernelResult| matches!(r, Err(KernelError::InvalidInput { .. }));
    assert!(invalid(k.make_box(&f, Vec3::new(1.0, -1.0, 1.0))));
    assert!(invalid(k.make_cylinder(&f, 1.0, 1.0, 7.0)));
    assert!(invalid(k.make_cone(&f, 1.0, 1.0, 1.0, TAU)));
    assert!(invalid(k.make_sphere(&f, f64::NAN)));
    assert!(invalid(k.make_torus(&f, 1.0, 2.0, TAU)));
    let left = Plane {
        normal: Dir3::NEG_Z,
        ..Plane::XY
    };
    assert!(invalid(k.make_box(&left, Vec3::ONE)));
}

// ---- Done-when 1: prisms of five profile kinds ---------------------------------------------

#[test]
fn golden_prisms_of_five_profile_kinds() {
    // 1. Polygon face.
    let sq = prism(&square(10.0), Vec3::new(0.0, 0.0, 5.0)).shape;
    assert_measures("square prism", &sq, 500.0, 400.0);
    assert_eq!(counts(&sq), [6, 12, 8]);
    check_golden("prism_square", &sq);

    // 2. Circle face.
    let disk = face(Plane::XY, circle((1.0, 2.0), 3.0), vec![]);
    let cyl = prism(&disk, Vec3::new(0.0, 0.0, 4.0)).shape;
    assert_measures("circle prism", &cyl, 36.0 * PI, 18.0 * PI + 24.0 * PI);
    check_golden("prism_circle", &cyl);

    // 3. Face with a hole.
    let holed = face(
        Plane::XY,
        polygon(&[(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)]),
        vec![circle((10.0, 10.0), 4.0)],
    );
    let ring = prism(&holed, Vec3::new(0.0, 0.0, 3.0)).shape;
    let cap = 400.0 - 16.0 * PI;
    assert_measures(
        "holed prism",
        &ring,
        cap * 3.0,
        2.0 * cap + 80.0 * 3.0 + 8.0 * PI * 3.0,
    );
    check_golden("prism_holed", &ring);

    // 4. Ellipse face, on a tilted plane, swept along its normal.
    let frame = tilted();
    let (a, b, h) = (5.0, 3.0, 2.0);
    // As four quarter arcs: `Shape::face_area` integrates each edge with a fixed-order Gauss
    // rule, which is 2.3e-6 short on the side face of one full-turn elliptic edge (checked
    // below) and exact on quarters.
    let quarter = |k: f64| Curve2d::Ellipse {
        center: Point2::new(1.0, -1.0),
        major: Vec2::new(a, 0.0),
        minor_radius: b,
        t0: k * FRAC_PI_2,
        t1: (k + 1.0) * FRAC_PI_2,
    };
    let ellipse = face(
        frame,
        (0..4).map(|k| quarter(f64::from(k))).collect(),
        vec![],
    );
    let elliptic = prism(&ellipse, h * frame.normal.get()).shape;
    let whole = face(
        frame,
        vec![Curve2d::Ellipse {
            center: Point2::new(1.0, -1.0),
            major: Vec2::new(a, 0.0),
            minor_radius: b,
            t0: 0.0,
            t1: TAU,
        }],
        vec![],
    );
    let whole_prism = prism(&whole, h * frame.normal.get()).shape;
    assert_rel(
        "one-edge ellipse prism volume",
        whole_prism.volume().unwrap(),
        PI * a * b * h,
        REL,
    );
    assert_rel(
        "one-edge ellipse prism area (fixed-order quadrature)",
        area(&whole_prism),
        2.0 * PI * a * b + ellipse_perimeter(a, b) * h,
        1e-5,
    );
    assert_measures(
        "ellipse prism",
        &elliptic,
        PI * a * b * h,
        2.0 * PI * a * b + ellipse_perimeter(a, b) * h,
    );
    check_golden("prism_ellipse", &elliptic);

    // 5. Open wire (an L of two lines): a shell.
    let wire = kernel()
        .make_wire(
            &[
                line_seg(Point3::ZERO, Point3::new(10.0, 0.0, 0.0), 1),
                line_seg(Point3::new(10.0, 0.0, 0.0), Point3::new(10.0, 5.0, 0.0), 2),
            ],
            &TOL,
        )
        .unwrap()
        .shape;
    let sheet = prism(&wire, Vec3::new(0.0, 0.0, 7.0));
    assert_eq!(sheet.shape.kind(), ShapeKind::Shell);
    assert_measures("wire prism", &sheet.shape, 0.0, 105.0);
    assert_eq!(counts(&sheet.shape), [2, 7, 6]);
    // No caps on a sheet: both faces are lateral.
    assert_eq!(sheet.history.roles.len(), 2);
    assert!(
        sheet
            .history
            .roles
            .iter()
            .all(|(_, r)| *r == OpRole::Lateral)
    );
    check_golden("prism_wire", &sheet.shape);

    // Also: a slot (tangent lines and arcs), and an oblique vector (Cavalieri).
    let (len, r) = (8.0, 2.0);
    let slotted = prism(
        &face(Plane::XY, slot(len, r), vec![]),
        Vec3::new(0.0, 0.0, 3.0),
    )
    .shape;
    let slot_area = 2.0 * r * len + PI * r * r;
    assert_measures(
        "slot prism",
        &slotted,
        slot_area * 3.0,
        2.0 * slot_area + (2.0 * len + TAU * r) * 3.0,
    );
    check_golden("prism_slot", &slotted);
    let oblique = prism(&square(10.0), Vec3::new(3.0, -4.0, 5.0)).shape;
    let side = |d: Vec3| d.length();
    // Two side faces are parallelograms over the x edges, two over the y edges.
    let v = Vec3::new(3.0, -4.0, 5.0);
    let over_x = 10.0 * side(v.cross(Vec3::X));
    let over_y = 10.0 * side(v.cross(Vec3::Y));
    assert_measures(
        "oblique prism",
        &oblique,
        500.0,
        200.0 + 2.0 * over_x + 2.0 * over_y,
    );
}

fn line_seg(a: Point3, b: Point3, tag: u64) -> amg_brep::geom::CurveSeg {
    amg_brep::geom::CurveSeg {
        curve: amg_brep::geom::Curve3::Line {
            origin: a,
            dir: Dir3::new_unchecked((b - a).normalize()),
        },
        t0: 0.0,
        t1: (b - a).length(),
        tag: Tag(tag),
    }
}

#[test]
fn infinite_prisms_are_valid_and_unbounded() {
    let k = kernel();
    let profile = square(4.0);
    for both in [false, true] {
        let r = k
            .prism(
                &profile,
                &PrismExtent::Infinite { dir: Dir3::Z, both },
                &PrismOpts {
                    taper: 0.0,
                    merge_coplanar: false,
                },
                &ctx(),
            )
            .unwrap();
        let report = k.check(&r.shape, &Default::default());
        // One way, the start cap closes the solid; both ways, nothing does.
        assert_eq!(report.valid, !both, "{:?}", report.problem_kinds());
        let [faces, ..] = counts(&r.shape);
        // Four infinite side planes, plus the start cap one way.
        assert_eq!(faces, if both { 4 } else { 5 }, "both = {both}");
        let caps = r
            .history
            .roles
            .iter()
            .filter(|(_, role)| *role == OpRole::StartCap)
            .count();
        assert_eq!(caps, usize::from(!both));
        assert!(
            r.history
                .roles
                .iter()
                .all(|(_, role)| *role != OpRole::EndCap)
        );
    }
}

#[test]
fn prism_history_names_caps_and_laterals() {
    let profile = square(10.0);
    let r = prism(&profile, Vec3::new(0.0, 0.0, 5.0));
    let h = &r.history;
    assert_eq!(h.operand_counts, vec![[1, 4, 4]]);
    let role_count = |role: OpRole| h.roles.iter().filter(|(_, r)| *r == role).count();
    assert_eq!(role_count(OpRole::StartCap), 1);
    assert_eq!(role_count(OpRole::EndCap), 1);
    assert_eq!(role_count(OpRole::Lateral), 4);
    let face_in = |i| InputSub {
        operand: 0,
        sub: SubShape {
            kind: SubKind::Face,
            index: i,
        },
    };
    // The profile face becomes the start cap (z = 0) and generates the end cap (z = 5).
    let rel: Vec<&Relation> = h.of_input(face_in(0)).collect();
    let z_of = |f: &SubShape| r.shape.sub_shape(*f).unwrap().bbox().max.z;
    match rel.as_slice() {
        [Relation::Modified(start), Relation::Generated(end)] => {
            assert_eq!(h.role_of(start[0]), Some(&OpRole::StartCap));
            assert_eq!(h.role_of(end[0]), Some(&OpRole::EndCap));
            assert!(z_of(&start[0]).abs() < 1e-12);
            assert!((z_of(&end[0]) - 5.0).abs() < 1e-12);
        }
        other => panic!("{other:?}"),
    }
    // Every profile edge generates one lateral face containing it, and its end-cap edge.
    for e in profile.edges() {
        let input = InputSub {
            operand: 0,
            sub: SubShape {
                kind: SubKind::Edge,
                index: e.0,
            },
        };
        let generated: Vec<SubShape> = h
            .of_input(input)
            .filter_map(|r| match r {
                Relation::Generated(outs) => Some(outs.to_vec()),
                _ => None,
            })
            .flatten()
            .collect();
        let faces: Vec<&SubShape> = generated
            .iter()
            .filter(|s| s.kind == SubKind::Face)
            .collect();
        assert_eq!(faces.len(), 1, "{e:?}");
        assert_eq!(generated.len(), 2, "{e:?}: side face and end edge");
        assert_eq!(h.role_of(*faces[0]), Some(&OpRole::Lateral));
        let edge_box = profile.sub_shape(input.sub).unwrap().bbox();
        let side_box = r.shape.sub_shape(*faces[0]).unwrap().bbox();
        assert!((edge_box.min - side_box.min).length() < 1e-9);
        assert_eq!(h.sources_of(*faces[0]).len(), 1);
    }
    // Every output face is explained (BREP-05 completeness, ahead of time).
    for f in r.shape.faces() {
        let o = SubShape {
            kind: SubKind::Face,
            index: f.0,
        };
        assert!(
            !h.sources_of(o).is_empty() && h.role_of(o).is_some(),
            "{f:?}"
        );
    }
}

#[test]
fn merge_coplanar_unifies_split_sides_and_keeps_history() {
    // The bottom side is split by a collinear vertex: two coplanar side faces.
    let profile = face(
        Plane::XY,
        polygon(&[
            (0.0, 0.0),
            (5.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0),
        ]),
        vec![],
    );
    let v = Vec3::new(0.0, 0.0, 10.0);
    let plain = prism(&profile, v);
    assert_eq!(counts(&plain.shape)[0], 7);
    let merged = kernel()
        .prism(
            &profile,
            &PrismExtent::Vector(v),
            &PrismOpts {
                taper: 0.0,
                merge_coplanar: true,
            },
            &ctx(),
        )
        .unwrap();
    assert_eq!(counts(&merged.shape), [6, 12, 8]);
    assert_measures("merged prism", &merged.shape, 1000.0, 600.0);
    // The merged side face is generated by both bottom edges.
    let laterals: Vec<SubShape> = merged
        .history
        .roles
        .iter()
        .filter(|(_, r)| *r == OpRole::Lateral)
        .map(|(s, _)| *s)
        .collect();
    assert_eq!(laterals.len(), 4);
    let sources: Vec<usize> = laterals
        .iter()
        .map(|f| merged.history.sources_of(*f).len())
        .collect();
    assert_eq!(
        sources.iter().filter(|&&n| n == 2).count(),
        1,
        "{sources:?}"
    );
    assert_eq!(
        sources.iter().filter(|&&n| n == 1).count(),
        3,
        "{sources:?}"
    );
    check_golden("prism_merged", &merged.shape);
}

// ---- Done-when 2: taper -------------------------------------------------------------------

fn frustum(a1: f64, a2: f64, h: f64) -> f64 {
    h / 3.0 * (a1 + a2 + (a1 * a2).sqrt())
}

#[test]
fn golden_tapered_square_matches_the_frustum() {
    let (a, h) = (10.0, 10.0);
    for degrees in [5.0_f64, -5.0] {
        let t = degrees.to_radians();
        let r = tapered(&square(a), Vec3::new(0.0, 0.0, h), t).unwrap();
        let top = a + 2.0 * h * t.tan();
        let slant = (h * h + (h * t.tan()).powi(2)).sqrt();
        assert_measures(
            &format!("square tapered {degrees}°"),
            &r.shape,
            frustum(a * a, top * top, h),
            a * a + top * top + 4.0 * (a + top) / 2.0 * slant,
        );
        assert_eq!(counts(&r.shape), [6, 12, 8]);
        // Every side face is a plane at the taper angle to the pull direction.
        for (f, role) in &r.history.roles {
            if *role != OpRole::Lateral {
                continue;
            }
            let Surface::Plane(p) = r.shape.surface(FaceId(f.index)) else {
                panic!("side face {f:?} is not planar");
            };
            let tilt = p.normal.get().dot(Vec3::Z).abs().asin();
            assert!((tilt - t.abs()).abs() < 1e-12, "{tilt} vs {t}");
        }
        let case = if degrees > 0.0 { "out" } else { "in" };
        check_golden(&format!("prism_taper_square_{case}"), &r.shape);
    }
}

#[test]
fn tapered_circles_slots_and_holes_follow_the_offset_region() {
    // Steiner: the region offset by d has area A + P d + π d² (convex, rounded), so a draft of
    // tan t gives V = A h + P tan t h² / 2 + π tan² t h³ / 3.
    let steiner = |a: f64, p: f64, t: f64, h: f64| {
        a * h + p * t.tan() * h * h / 2.0 + PI * t.tan().powi(2) * h.powi(3) / 3.0
    };
    let (r, h) = (4.0, 6.0);
    for degrees in [5.0_f64, -5.0] {
        let t = degrees.to_radians();
        let cone = tapered(
            &face(Plane::XY, circle((0.0, 0.0), r), vec![]),
            Vec3::new(0.0, 0.0, h),
            t,
        )
        .unwrap()
        .shape;
        let top = r + h * t.tan();
        assert_rel(
            &format!("circle tapered {degrees}° volume"),
            cone.volume().unwrap(),
            PI * h / 3.0 * (r * r + r * top + top * top),
            REL,
        );
        assert!(
            cone.faces()
                .any(|f| matches!(cone.surface(f), Surface::Cone { .. }))
        );

        let (len, rs) = (8.0, 2.0);
        let slotted = tapered(
            &face(Plane::XY, slot(len, rs), vec![]),
            Vec3::new(0.0, 0.0, 3.0),
            t,
        )
        .unwrap()
        .shape;
        assert_rel(
            &format!("slot tapered {degrees}° volume"),
            slotted.volume().unwrap(),
            steiner(2.0 * rs * len + PI * rs * rs, 2.0 * len + TAU * rs, t, 3.0),
            REL,
        );
    }
    // A hole shrinks as the outer square grows.
    let t = 5.0_f64.to_radians();
    let (a, rh, hh) = (20.0, 4.0, 3.0);
    let holed = face(
        Plane::XY,
        polygon(&[(0.0, 0.0), (a, 0.0), (a, a), (0.0, a)]),
        vec![circle((10.0, 10.0), rh)],
    );
    let solid = tapered(&holed, Vec3::new(0.0, 0.0, hh), t).unwrap().shape;
    let top = a + 2.0 * hh * t.tan();
    let hole_top = rh - hh * t.tan();
    let want = frustum(a * a, top * top, hh)
        - PI * hh / 3.0 * (rh * rh + rh * hole_top + hole_top * hole_top);
    assert_rel(
        "holed square tapered 5° volume",
        solid.volume().unwrap(),
        want,
        REL,
    );
}

#[test]
fn tapers_are_rejected_where_they_cannot_apply() {
    let k = kernel();
    let taper = PrismOpts {
        taper: 0.1,
        merge_coplanar: false,
    };
    let infinite = PrismExtent::Infinite {
        dir: Dir3::Z,
        both: false,
    };
    assert!(matches!(
        k.prism(&square(1.0), &infinite, &taper, &ctx()),
        Err(KernelError::InvalidInput { what: "taper", .. })
    ));
    let wire = k
        .make_wire(&[line_seg(Point3::ZERO, Point3::X, 0)], &TOL)
        .unwrap()
        .shape;
    assert!(matches!(
        k.prism(&wire, &PrismExtent::Vector(Vec3::Z), &taper, &ctx()),
        Err(KernelError::InvalidInput { what: "taper", .. })
    ));
    let steep = PrismOpts {
        taper: FRAC_PI_2,
        merge_coplanar: false,
    };
    assert!(matches!(
        k.prism(&square(1.0), &PrismExtent::Vector(Vec3::Z), &steep, &ctx()),
        Err(KernelError::InvalidInput { what: "taper", .. })
    ));
    // An ellipse side face is a surface of extrusion, which the draft cannot tilt.
    let ellipse = face(
        Plane::XY,
        vec![Curve2d::Ellipse {
            center: Point2::ZERO,
            major: Vec2::new(5.0, 0.0),
            minor_radius: 3.0,
            t0: 0.0,
            t1: TAU,
        }],
        vec![],
    );
    let err = tapered(&ellipse, Vec3::Z, 0.1).unwrap_err();
    assert!(matches!(err, KernelError::Occt { .. }), "{err:?}");
    // An inward taper that collapses the square fails cleanly.
    let collapsed = tapered(&square(2.0), Vec3::new(0.0, 0.0, 10.0), -0.5);
    assert!(collapsed.is_err(), "{collapsed:?}");
}

#[test]
fn bad_prism_inputs_are_invalid_input() {
    let k = kernel();
    let none = PrismOpts {
        taper: 0.0,
        merge_coplanar: false,
    };
    let block = k.make_box(&Plane::XY, Vec3::ONE).unwrap().shape;
    assert!(matches!(
        k.prism(&block, &PrismExtent::Vector(Vec3::Z), &none, &ctx()),
        Err(KernelError::InvalidInput {
            what: "profile",
            ..
        })
    ));
    assert!(matches!(
        k.prism(
            &square(1.0),
            &PrismExtent::Vector(Vec3::ZERO),
            &none,
            &ctx()
        ),
        Err(KernelError::InvalidInput { what: "extent", .. })
    ));
    // In the profile plane.
    assert!(matches!(
        k.prism(&square(1.0), &PrismExtent::Vector(Vec3::X), &none, &ctx()),
        Err(KernelError::InvalidInput { what: "extent", .. })
    ));
}

// ---- Done-when 1: revol ---------------------------------------------------------------------

/// A rectangle in the XZ plane: x in [5, 8], z in [0, 4].
fn xz_rectangle() -> Shape {
    kernel()
        .make_planar_polygon(
            &[
                Point3::new(5.0, 0.0, 0.0),
                Point3::new(8.0, 0.0, 0.0),
                Point3::new(8.0, 0.0, 4.0),
                Point3::new(5.0, 0.0, 4.0),
            ],
            &[],
            &TOL,
        )
        .unwrap()
}

fn z_axis() -> Line3 {
    Line3 {
        point: Point3::ZERO,
        dir: Dir3::Z,
    }
}

#[test]
fn golden_revols_match_pappus() {
    let k = kernel();
    let profile = xz_rectangle();
    let full = k.revol(&profile, &z_axis(), TAU, &ctx()).unwrap();
    let ring = PI * (64.0 - 25.0);
    assert_measures(
        "revol 360°",
        &full.shape,
        ring * 4.0,
        2.0 * ring + TAU * 8.0 * 4.0 + TAU * 5.0 * 4.0,
    );
    // A full turn has no caps; every face is lateral.
    assert!(
        full.history
            .roles
            .iter()
            .all(|(_, r)| *r == OpRole::Lateral)
    );
    assert_eq!(full.history.roles.len(), counts(&full.shape)[0] as usize);
    check_golden("revol_360", &full.shape);

    let quarter = k.revol(&profile, &z_axis(), FRAC_PI_2, &ctx()).unwrap();
    assert_measures(
        "revol 90°",
        &quarter.shape,
        ring,
        (2.0 * ring + TAU * 8.0 * 4.0 + TAU * 5.0 * 4.0) / 4.0 + 2.0 * 12.0,
    );
    let caps: Vec<&OpRole> = quarter
        .history
        .roles
        .iter()
        .map(|(_, r)| r)
        .filter(|r| **r != OpRole::Lateral)
        .collect();
    assert_eq!(caps.len(), 2, "{caps:?}");
    check_golden("revol_90", &quarter.shape);

    // A triangle with an edge on the axis gives a cone.
    let triangle = k
        .make_planar_polygon(
            &[
                Point3::ZERO,
                Point3::new(4.0, 0.0, 0.0),
                Point3::new(0.0, 0.0, 6.0),
            ],
            &[],
            &TOL,
        )
        .unwrap();
    let cone = k.revol(&triangle, &z_axis(), TAU, &ctx()).unwrap().shape;
    assert_measures(
        "revolved triangle",
        &cone,
        PI * 16.0 * 6.0 / 3.0,
        PI * 4.0 * 52.0_f64.sqrt() + PI * 16.0,
    );
    assert!(
        cone.faces()
            .any(|f| matches!(cone.surface(f), Surface::Cone { .. }))
    );
    check_golden("revol_triangle", &cone);

    // Revolving a wire gives a shell (here an annulus and a cylinder band).
    let wire = k
        .make_wire(
            &[
                line_seg(Point3::new(5.0, 0.0, 0.0), Point3::new(8.0, 0.0, 0.0), 1),
                line_seg(Point3::new(8.0, 0.0, 0.0), Point3::new(8.0, 0.0, 4.0), 2),
            ],
            &TOL,
        )
        .unwrap()
        .shape;
    let band = k.revol(&wire, &z_axis(), TAU, &ctx()).unwrap().shape;
    assert_eq!(band.kind(), ShapeKind::Shell);
    assert_rel(
        "revolved wire area",
        area(&band),
        ring + TAU * 8.0 * 4.0,
        REL,
    );
}

#[test]
fn revol_caps_and_errors() {
    let k = kernel();
    let quarter = k
        .revol(&xz_rectangle(), &z_axis(), FRAC_PI_2, &ctx())
        .unwrap();
    let mut caps: Vec<OpRole> = quarter
        .history
        .roles
        .iter()
        .map(|(_, r)| r.clone())
        .filter(|r| *r != OpRole::Lateral)
        .collect();
    caps.sort_by_key(|r| format!("{r:?}"));
    assert_eq!(caps, vec![OpRole::EndCap, OpRole::StartCap]);
    for angle in [0.0, -1.0, 7.0, f64::NAN] {
        assert!(matches!(
            k.revol(&xz_rectangle(), &z_axis(), angle, &ctx()),
            Err(KernelError::InvalidInput { what: "angle", .. })
        ));
    }
    // A profile crossing the axis self-intersects: invalid result.
    let crossing = k
        .make_planar_polygon(
            &[
                Point3::new(-2.0, 0.0, 0.0),
                Point3::new(3.0, 0.0, 0.0),
                Point3::new(3.0, 0.0, 4.0),
                Point3::new(-2.0, 0.0, 4.0),
            ],
            &[],
            &TOL,
        )
        .unwrap();
    let crossed = k.revol(&crossing, &z_axis(), TAU, &ctx());
    assert!(crossed.is_err(), "{crossed:?}");
}

#[test]
fn cancelled_sweeps_return_cancelled() {
    let token = amg_jobs::CancelToken::new();
    token.cancel();
    let cancelled = JobCtx::with_token(token);
    let k = kernel();
    let (extent, opts) = straight(Vec3::Z);
    assert!(matches!(
        k.prism(&square(1.0), &extent, &opts, &cancelled),
        Err(KernelError::Cancelled)
    ));
    assert!(matches!(
        k.revol(&xz_rectangle(), &z_axis(), 1.0, &cancelled),
        Err(KernelError::Cancelled)
    ));
}

// ---- gtransformed, mirrored -----------------------------------------------------------------

fn modified_one_to_one(r: &OpResult, input: &Shape) {
    assert_eq!(r.history.operand_counts, vec![counts(input)]);
    assert_eq!(counts(&r.shape), counts(input));
    let total: u32 = counts(input).iter().sum();
    assert_eq!(r.history.entries.len(), total as usize);
    let mut seen = std::collections::BTreeSet::new();
    for (input_sub, relation) in &r.history.entries {
        match relation {
            Relation::Modified(outs) => {
                assert_eq!(outs.len(), 1, "{input_sub:?}");
                assert_eq!(outs[0].kind, input_sub.sub.kind);
                assert!(seen.insert(outs[0]), "{outs:?} twice");
            }
            other => panic!("{input_sub:?}: {other:?}"),
        }
    }
}

#[test]
fn gtransformed_scales_uniformly_and_keeps_analytic_geometry() {
    let k = kernel();
    let cyl = k.make_cylinder(&tilted(), 5.0, 20.0, TAU).unwrap().shape;
    let t = Transform::from_scale_rotation_translation(
        Vec3::splat(2.0),
        Quat::from_rotation_y(0.3),
        Vec3::new(1.0, 2.0, 3.0),
    );
    let r = cyl.gtransformed(&t).unwrap();
    assert_rel(
        "scaled volume",
        r.shape.volume().unwrap(),
        8.0 * cyl.volume().unwrap(),
        1e-12,
    );
    assert_eq!(common::face_classes(&r.shape), common::face_classes(&cyl));
    let radius = r
        .shape
        .faces()
        .find_map(|f| match r.shape.surface(f) {
            Surface::Cylinder { radius, .. } => Some(radius),
            _ => None,
        })
        .unwrap();
    assert!((radius - 10.0).abs() < 1e-12, "{radius}");
    modified_one_to_one(&r, &cyl);
    assert!(k.check(&r.shape, &Default::default()).valid);
    // A rigid map through gtransformed matches the located copy.
    let rigid =
        Transform::from_rotation_translation(Quat::from_rotation_x(1.0), Vec3::new(4.0, 0.0, -1.0));
    let moved = cyl.transformed(&rigid).unwrap();
    let copied = cyl.gtransformed(&rigid).unwrap().shape;
    assert!((moved.bbox().min - copied.bbox().min).length() < 1e-9);
    assert_rel(
        "rigid copy volume",
        copied.volume().unwrap(),
        cyl.volume().unwrap(),
        1e-12,
    );
}

#[test]
fn gtransformed_scales_a_sphere_into_an_ellipsoid() {
    let k = kernel();
    let sphere = k.make_sphere(&Plane::XY, 10.0).unwrap().shape;
    let r = sphere
        .gtransformed(&Transform::from_scale(Vec3::new(1.0, 2.0, 3.0)))
        .unwrap();
    assert!(k.check(&r.shape, &Default::default()).valid);
    assert!(
        r.shape
            .faces()
            .all(|f| matches!(r.shape.surface(f), Surface::BSpline(_)))
    );
    let want = 4.0 / 3.0 * PI * 10.0 * 20.0 * 30.0;
    let got = r.shape.volume().unwrap();
    let err = (got - want).abs() / want;
    eprintln!("ellipsoid volume {got} vs {want}: relative error {err:.2e}");
    // `Shape::volume` integrates the rational B-spline faces with a fixed-order rule (see the
    // BREP-14 Refinement log); the geometry itself is checked through its extent.
    assert!(err < 1e-3, "{err}");
    let bbox = r.shape.bbox();
    assert!(
        (bbox.max - Vec3::new(10.0, 20.0, 30.0)).length() < 1e-6,
        "{bbox:?}"
    );
    modified_one_to_one(&r, &sphere);
    assert!(matches!(
        sphere.gtransformed(&Transform::from_scale(Vec3::new(1.0, 0.0, 1.0))),
        Err(KernelError::InvalidInput {
            what: "transform",
            ..
        })
    ));
}

#[test]
fn mirrored_reflects_with_outward_faces() {
    let k = kernel();
    let solid = prism(
        &face(
            tilted(),
            polygon(&[(0.0, 0.0), (6.0, 0.0), (6.0, 2.0), (2.0, 5.0), (0.0, 3.0)]),
            vec![circle((2.0, 2.0), 0.8)],
        ),
        Vec3::new(1.0, 2.0, 7.0),
    )
    .shape;
    let plane = Plane {
        origin: Point3::new(20.0, 0.0, 0.0),
        ..Plane::YZ
    };
    let r = k.mirrored(&solid, &plane).unwrap();
    let volume = r.shape.volume().unwrap();
    assert!(volume > 0.0, "{volume}");
    assert_rel("mirrored volume", volume, solid.volume().unwrap(), 1e-12);
    assert_rel("mirrored area", area(&r.shape), area(&solid), 1e-12);
    assert!(k.check(&r.shape, &Default::default()).valid);
    assert_eq!(common::face_classes(&r.shape), common::face_classes(&solid));
    // Every vertex is reflected about x = 20.
    for v in solid.vertices() {
        let p = solid.vertex_point(v);
        let q = Point3::new(40.0 - p.x, p.y, p.z);
        assert!(
            r.shape
                .vertices()
                .any(|w| (r.shape.vertex_point(w) - q).length() < 1e-9),
            "{p}"
        );
    }
    modified_one_to_one(&r, &solid);
    // `transformed` still rejects the reflection.
    let reflection = Transform::from_mat3_translation(
        amg_core::Mat3::from_diagonal(Vec3::new(-1.0, 1.0, 1.0)),
        Vec3::new(40.0, 0.0, 0.0),
    );
    assert!(matches!(
        solid.transformed(&reflection),
        Err(KernelError::NonRigid)
    ));
    let same = solid.gtransformed(&reflection).unwrap().shape;
    assert_rel("reflected volume", same.volume().unwrap(), volume, 1e-12);
}

// ---- budget ---------------------------------------------------------------------------------

/// S02 budget: a prism of a 100-edge profile in < 10 ms (asserted with `AMG_STRICT_TIMING=1`;
/// the criterion bench is `budget/brep-prism-100` in `benches/ops.rs`).
#[test]
fn prism_of_a_100_edge_profile_is_fast() {
    let n = 100;
    let points: Vec<(f64, f64)> = (0..n)
        .map(|i| {
            let a = TAU * f64::from(i) / f64::from(n);
            let r = if i % 2 == 0 { 10.0 } else { 7.0 };
            (r * a.cos(), r * a.sin())
        })
        .collect();
    let profile = face(Plane::XY, polygon(&points), vec![]);
    let k = OcctKernel::new(KernelConfig {
        validate: Validate::Off,
        ..KernelConfig::default()
    });
    let (extent, opts) = straight(Vec3::new(0.0, 0.0, 5.0));
    let mut times: Vec<f64> = (0..11)
        .map(|_| {
            let start = std::time::Instant::now();
            let r = k.prism(&profile, &extent, &opts, &ctx()).unwrap();
            let elapsed = start.elapsed().as_secs_f64();
            assert_eq!(counts(&r.shape)[0], 102);
            elapsed
        })
        .collect();
    times.sort_by(f64::total_cmp);
    let median = times[times.len() / 2];
    eprintln!("prism of a 100-edge profile: median {:.3} ms", median * 1e3);
    if std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1") {
        assert!(median < 0.010, "median {median} s");
    }
}
