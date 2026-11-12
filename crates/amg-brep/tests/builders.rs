//! BREP-03: face and wire builders (S02 §4) through OCCT.
//!
//! Done-when 1 (five curve kinds, analytic area, tags), 2 (open loop, hole outside), 3 (1000-vertex
//! polygon time, `NotPlanar`), 4 (`Polylines` bounds from tessellated reference faces, B-spline
//! patch), 5 (random profiles, proptest), plus `Loops` bounds, `make_wire` and a golden. Timing is
//! asserted only with `AMG_STRICT_TIMING=1`; the criterion numbers are in `benches/builders.rs`.
#![cfg(feature = "occt")]
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, PI, TAU};
use std::time::{Duration, Instant};

use amg_brep::geom::{
    BSpline2dData, BSplineSurfaceData, Curve2d, Curve3, CurveSeg, FaceBounds, Loop2, PlanarProfile,
    ProfileEdge2, Surface, Tag,
};
use amg_brep::{
    BrepKernel, CheckOpts, Dir3, FaceId, JobCtx, KernelConfig, KernelError, OcctKernel, Shape,
    ShapeKind, SubKind, SubShape, TessQuality, Tolerance, Validate,
};
use amg_core::{Plane, Point2, Point3, Vec2, Vec3};
use amg_testkit::TriSoup;
use proptest::prelude::*;

const TOL: Tolerance = Tolerance::DEFAULT;

fn kernel() -> OcctKernel {
    OcctKernel::new(KernelConfig::default())
}

/// No validation inside the op: the tests check validity themselves (and time without it).
fn raw_kernel() -> OcctKernel {
    OcctKernel::new(KernelConfig {
        validate: Validate::Off,
        ..KernelConfig::default()
    })
}

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}

fn assert_valid(what: &str, shape: &Shape) {
    let report = raw_kernel().check(shape, &CheckOpts::default());
    assert!(report.valid, "{what}: {report:?}");
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs()
}

fn pe(curve: Curve2d, tag: u64) -> ProfileEdge2 {
    ProfileEdge2 {
        curve,
        tag: Tag(tag),
    }
}

fn line(a: (f64, f64), b: (f64, f64)) -> Curve2d {
    Curve2d::Line {
        a: Point2::new(a.0, a.1),
        b: Point2::new(b.0, b.1),
    }
}

fn rect(x0: f64, y0: f64, x1: f64, y1: f64, ccw: bool, tag0: u64) -> Loop2 {
    let mut pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)];
    if !ccw {
        pts.reverse();
    }
    Loop2 {
        edges: (0..4)
            .map(|i| pe(line(pts[i], pts[(i + 1) % 4]), tag0 + i as u64))
            .collect(),
    }
}

// ---- Done-when 1 -------------------------------------------------------------------------

/// Green's-theorem area terms `∮ (x dy − y dx) / 2` of the pieces, in closed form.
fn green_line(a: (f64, f64), b: (f64, f64)) -> f64 {
    0.5 * (a.0 * b.1 - b.0 * a.1)
}

fn green_arc(c: (f64, f64), r: f64, t0: f64, t1: f64) -> f64 {
    0.5 * (r * r * (t1 - t0) + r * (c.0 * (t1.sin() - t0.sin()) - c.1 * (t1.cos() - t0.cos())))
}

fn green_ellipse(c: (f64, f64), a: f64, b: f64, t0: f64, t1: f64) -> f64 {
    0.5 * (a * b * (t1 - t0) + c.0 * b * (t1.sin() - t0.sin()) - c.1 * a * (t1.cos() - t0.cos()))
}

/// Outer: line, arc, ellipse arc, rational B-spline (an exact quarter circle), line. Hole: circle.
/// Tags 10..=14 on the outer edges, 20 on the hole.
fn five_kinds_profile(plane: Plane) -> (PlanarProfile, f64) {
    let outer = Loop2 {
        edges: vec![
            pe(line((0.0, 0.0), (30.0, 0.0)), 10),
            pe(
                Curve2d::Arc {
                    center: Point2::new(30.0, 10.0),
                    radius: 10.0,
                    start_angle: -FRAC_PI_2,
                    sweep: PI,
                },
                11,
            ),
            pe(
                Curve2d::Ellipse {
                    center: Point2::new(15.0, 20.0),
                    major: Vec2::new(15.0, 0.0),
                    minor_radius: 5.0,
                    t0: 0.0,
                    t1: PI,
                },
                12,
            ),
            pe(
                Curve2d::BSpline(BSpline2dData {
                    degree: 2,
                    knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                    poles: vec![
                        Point2::new(0.0, 20.0),
                        Point2::new(-10.0, 20.0),
                        Point2::new(-10.0, 10.0),
                    ],
                    weights: Some(vec![1.0, FRAC_1_SQRT_2, 1.0]),
                }),
                13,
            ),
            pe(line((-10.0, 10.0), (0.0, 0.0)), 14),
        ],
    };
    let hole = Loop2 {
        edges: vec![pe(
            Curve2d::Circle {
                center: Point2::new(15.0, 10.0),
                radius: 4.0,
            },
            20,
        )],
    };
    let area = green_line((0.0, 0.0), (30.0, 0.0))
        + green_arc((30.0, 10.0), 10.0, -FRAC_PI_2, FRAC_PI_2)
        + green_ellipse((15.0, 20.0), 15.0, 5.0, 0.0, PI)
        + green_arc((0.0, 10.0), 10.0, FRAC_PI_2, PI)
        + green_line((-10.0, 10.0), (0.0, 0.0))
        - PI * 16.0;
    (
        PlanarProfile {
            plane,
            outer,
            holes: vec![hole],
        },
        area,
    )
}

fn tilted_plane() -> Plane {
    let normal = Dir3::new_unchecked(Vec3::new(1.0, -2.0, 3.0).normalize());
    Plane::from_normal_x(Point3::new(5.0, -3.0, 2.0), normal, Vec3::X, &TOL).unwrap()
}

#[test]
fn five_curve_kinds_give_a_valid_face_with_the_analytic_area() {
    for plane in [Plane::XY, tilted_plane()] {
        let (profile, area) = five_kinds_profile(plane);
        let built = kernel().make_planar_faces(&[profile], &TOL).unwrap();
        assert_eq!(built.len(), 1);
        let face = &built[0].face;
        assert_eq!(face.kind(), ShapeKind::Face);
        assert_valid("five kinds", face);
        let got = face.face_area(FaceId(0));
        println!(
            "five kinds: area {got:.12} analytic {area:.12}, rel {:.2e}",
            rel(got, area)
        );
        assert!(rel(got, area) < 1e-9, "{got} vs {area}");
        // The face normal is the plane's.
        assert!(matches!(face.surface(FaceId(0)), Surface::Plane(_)));
        // Every edge carries its input tag, one edge per input curve.
        assert_eq!(face.count(SubKind::Edge), 6);
        let tags = &built[0].edge_tags;
        assert_eq!(
            tags.iter().map(|(_, t)| t.0).collect::<Vec<_>>(),
            [10, 11, 12, 13, 14, 20]
        );
        let mut edges: Vec<u32> = tags.iter().map(|(e, _)| e.0).collect();
        edges.sort_unstable();
        edges.dedup();
        assert_eq!(edges, [0, 1, 2, 3, 4, 5]);
        // The hole's edge is in the second wire.
        let wires = face.face_wires(FaceId(0));
        assert_eq!(wires.len(), 2);
        assert_eq!(wires[1], [tags[5].0]);
    }
}

#[test]
fn many_profiles_build_in_one_call() {
    let profiles: Vec<PlanarProfile> = (0..50)
        .map(|i| {
            let x = f64::from(i) * 12.0;
            PlanarProfile {
                plane: Plane::XY,
                outer: rect(x, 0.0, x + 10.0, 10.0, true, 4 * u64::from(i as u32)),
                holes: vec![rect(x + 2.0, 2.0, x + 4.0, 4.0, false, 1000)],
            }
        })
        .collect();
    let built = kernel().make_planar_faces(&profiles, &TOL).unwrap();
    assert_eq!(built.len(), 50);
    for (i, b) in built.iter().enumerate() {
        assert!((b.face.face_area(FaceId(0)) - 96.0).abs() < 1e-9);
        assert_eq!(b.edge_tags.len(), 8);
        assert_eq!(b.edge_tags[0].1, Tag(4 * i as u64));
    }
}

// ---- Done-when 2 -------------------------------------------------------------------------

#[test]
fn an_open_loop_gives_open_loop_with_its_indices() {
    let mut hole = rect(2.0, 2.0, 4.0, 4.0, false, 0);
    // Hole edges: (2,4)→(4,4)→(4,2)→(2,2)→(2,4). Stop edge 3 short of edge 0's start.
    hole.edges[3].curve = line((2.0, 2.0), (2.0, 4.0 - 1e-3));
    let profile = PlanarProfile {
        plane: Plane::XY,
        outer: rect(0.0, 0.0, 10.0, 10.0, true, 0),
        holes: vec![rect(6.0, 6.0, 8.0, 8.0, false, 0), hole],
    };
    match kernel().make_planar_faces(&[profile], &TOL) {
        Err(KernelError::OpenLoop {
            loop_index,
            edge_index,
            gap,
        }) => {
            assert_eq!((loop_index, edge_index), (2, 3));
            assert!((gap - 1e-3).abs() < 1e-12);
        }
        other => panic!("expected OpenLoop, got {other:?}"),
    }
    // The outer loop too.
    let mut outer = rect(0.0, 0.0, 10.0, 10.0, true, 0);
    outer.edges[1].curve = line((10.0, 0.1), (10.0, 10.0));
    let profile = PlanarProfile {
        plane: Plane::XY,
        outer,
        holes: vec![],
    };
    assert!(matches!(
        kernel().make_planar_faces(&[profile], &TOL),
        Err(KernelError::OpenLoop {
            loop_index: 0,
            edge_index: 0,
            ..
        })
    ));
}

#[test]
fn a_hole_outside_the_outer_loop_is_invalid_input() {
    let profile = PlanarProfile {
        plane: Plane::XY,
        outer: rect(0.0, 0.0, 10.0, 10.0, true, 0),
        holes: vec![Loop2 {
            edges: vec![pe(
                Curve2d::Circle {
                    center: Point2::new(30.0, 5.0),
                    radius: 2.0,
                },
                0,
            )],
        }],
    };
    match kernel().make_planar_faces(&[profile], &TOL) {
        Err(KernelError::InvalidInput { what, reason, .. }) => {
            assert_eq!(what, "profiles");
            assert!(reason.contains("loop 1, edge 0"), "{reason}");
        }
        other => panic!("expected InvalidInput, got {other:?}"),
    }
}

// ---- Done-when 3 -------------------------------------------------------------------------

/// A 1000-vertex outer polygon with 10 holes of 20 vertices on a tilted plane.
fn big_polygon() -> (Vec<Point3>, Vec<Vec<Point3>>) {
    let plane = tilted_plane();
    let at = |u: f64, v: f64| plane.origin + plane.x.get() * u + plane.y.get() * v;
    let outer: Vec<Point3> = (0..1000)
        .map(|i| {
            let t = TAU * f64::from(i) / 1000.0;
            let r = 100.0 + 5.0 * (7.0 * t).sin();
            at(r * t.cos(), r * t.sin())
        })
        .collect();
    let holes = (0..10)
        .map(|h| {
            let c = TAU * f64::from(h) / 10.0;
            let (cx, cy) = (60.0 * c.cos(), 60.0 * c.sin());
            (0..20)
                .map(|i| {
                    let t = TAU * f64::from(i) / 20.0;
                    at(cx + 8.0 * t.cos(), cy + 8.0 * t.sin())
                })
                .collect()
        })
        .collect();
    (outer, holes)
}

#[test]
fn a_thousand_vertex_polygon_with_ten_holes_is_fast() {
    let (outer, holes) = big_polygon();
    let kernel = raw_kernel();
    let face = kernel.make_planar_polygon(&outer, &holes, &TOL).unwrap();
    assert_valid("polygon", &face);
    assert_eq!(face.count(SubKind::Edge), 1200);
    assert_eq!(face.face_wires(FaceId(0)).len(), 11);
    let mut times: Vec<Duration> = (0..11)
        .map(|_| {
            let start = Instant::now();
            let face = kernel.make_planar_polygon(&outer, &holes, &TOL).unwrap();
            let took = start.elapsed();
            drop(face);
            took
        })
        .collect();
    times.sort();
    let median = times[5];
    println!(
        "make_planar_polygon 1000 + 10×20 vertices: median {median:?} (min {:?})",
        times[0]
    );
    if strict_timing() {
        assert!(median < Duration::from_millis(20), "{median:?}");
    }
}

#[test]
fn polygon_area_and_hole_orientation() {
    let tilt = |x: f64, y: f64| Point3::new(x, y, 0.25 * x + 0.5 * y);
    let outer = [
        tilt(0.0, 0.0),
        tilt(10.0, 0.0),
        tilt(10.0, 10.0),
        tilt(0.0, 10.0),
    ];
    // Given CCW (wrong way for a hole): turned around.
    let hole = vec![
        tilt(2.0, 2.0),
        tilt(4.0, 2.0),
        tilt(4.0, 4.0),
        tilt(2.0, 4.0),
    ];
    let face = kernel().make_planar_polygon(&outer, &[hole], &TOL).unwrap();
    assert_valid("tilted polygon", &face);
    let stretch = (1.0f64 + 0.25 * 0.25 + 0.5 * 0.5).sqrt();
    let want = 96.0 * stretch;
    assert!(rel(face.face_area(FaceId(0)), want) < 1e-12);
}

#[test]
fn non_planar_points_give_not_planar() {
    let (mut outer, holes) = big_polygon();
    let n = tilted_plane().normal.get();
    outer[500] += n * 1e-3;
    match kernel().make_planar_polygon(&outer, &holes, &TOL) {
        Err(KernelError::NotPlanar { max_dev }) => {
            assert!(max_dev > 5e-4 && max_dev < 1e-3, "{max_dev}");
        }
        other => panic!("expected NotPlanar, got {other:?}"),
    }
    // Within tolerance is fine (and the points are projected).
    let (mut outer, holes) = big_polygon();
    outer[500] += n * 5e-7;
    let face = kernel().make_planar_polygon(&outer, &holes, &TOL).unwrap();
    assert_valid("near-planar polygon", &face);
}

// ---- Done-when 4 -------------------------------------------------------------------------

/// The boundary of `face`'s outer wire as one closed polyline, from a tessellation.
fn boundary_polyline(face: &Shape, q: &TessQuality) -> Vec<Point3> {
    let mesh = kernel().tessellate(face, q, &JobCtx::detached()).unwrap();
    let wires = face.face_wires(FaceId(0));
    let mut out: Vec<Point3> = Vec::new();
    for e in &wires[0] {
        let poly = &mesh.edges[e.0 as usize];
        let pts = &mesh.edge_points[poly.points.start as usize..poly.points.end as usize];
        assert!(pts.len() >= 2, "edge {e:?} has no polyline");
        let mut pts = pts.to_vec();
        if out.is_empty() {
            out = pts;
            continue;
        }
        let touches =
            |a: &[Point3], p: Point3| (a[0] - p).length().min((a[a.len() - 1] - p).length());
        // The first edge's direction is arbitrary: turn it if the second edge meets its start.
        if touches(&pts, out[out.len() - 1]) > 1e-6 && touches(&pts, out[0]) < 1e-6 {
            out.reverse();
        }
        let last = out[out.len() - 1];
        if (pts[pts.len() - 1] - last).length() < (pts[0] - last).length() {
            pts.reverse();
        }
        assert!((pts[0] - last).length() < 1e-6, "edges do not chain");
        out.extend_from_slice(&pts[1..]);
    }
    out
}

fn fine() -> TessQuality {
    TessQuality {
        linear: 0.002,
        angular: 0.05,
        relative: false,
        min_size: 0.0,
        max_linear_abs: None,
    }
}

#[test]
fn polylines_from_tessellated_faces_reproduce_the_area() {
    let frame = Plane::from_normal_x(
        Point3::new(1.0, 2.0, 3.0),
        Dir3::new_unchecked(Vec3::new(0.0, 1.0, 1.0).normalize()),
        Vec3::X,
        &TOL,
    )
    .unwrap();
    let cases = [
        (
            "cylinder",
            Surface::Cylinder { frame, radius: 5.0 },
            [0.3, 2.4, -2.0, 6.0],
        ),
        (
            "cone",
            Surface::Cone {
                frame,
                ref_radius: 4.0,
                half_angle: 0.4,
            },
            [-0.5, 1.9, 0.0, 8.0],
        ),
        (
            "sphere",
            Surface::Sphere { frame, radius: 6.0 },
            [0.2, 2.0, -0.7, 0.9],
        ),
        (
            "torus",
            Surface::Torus {
                frame,
                major: 10.0,
                minor: 3.0,
            },
            [0.1, 1.7, -1.0, 2.2],
        ),
    ];
    let kernel = kernel();
    for (name, surface, [u0, u1, v0, v1]) in cases {
        let reference = kernel
            .make_face(&surface, &FaceBounds::UvRect { u0, u1, v0, v1 }, 1e-6)
            .unwrap()
            .face;
        assert_valid(name, &reference);
        let want = reference.face_area(FaceId(0));
        let polyline = boundary_polyline(&reference, &fine());
        let rebuilt = kernel
            .make_face(
                &surface,
                &FaceBounds::Polylines(vec![polyline.clone()]),
                1e-4,
            )
            .unwrap();
        assert!(rebuilt.edge_tags.is_empty());
        let face = &rebuilt.face;
        assert_valid(name, face);
        let got = face.face_area(FaceId(0));
        println!(
            "{name}: {} polyline points, area {got:.6} vs {want:.6}, rel {:.2e}, {} edges",
            polyline.len(),
            rel(got, want),
            face.count(SubKind::Edge)
        );
        assert!(rel(got, want) < 1e-3, "{name}: {got} vs {want}");
        // The same loop given clockwise gives the same face.
        let mut reversed = polyline;
        reversed.reverse();
        let again = kernel
            .make_face(&surface, &FaceBounds::Polylines(vec![reversed]), 1e-4)
            .unwrap()
            .face;
        assert!(
            rel(again.face_area(FaceId(0)), want) < 1e-3,
            "{name} reversed"
        );
    }
}

#[test]
fn a_polyline_hole_is_cut_out() {
    let surface = Surface::Cylinder {
        frame: Plane::XY,
        radius: 10.0,
    };
    let kernel = kernel();
    let reference = kernel
        .make_face(
            &surface,
            &FaceBounds::UvRect {
                u0: 0.0,
                u1: 2.0,
                v0: 0.0,
                v1: 20.0,
            },
            1e-6,
        )
        .unwrap()
        .face;
    let outer = boundary_polyline(&reference, &fine());
    // A small loop around (u, v) = (1, 10), drawn on the cylinder.
    let hole: Vec<Point3> = (0..64)
        .map(|i| {
            let t = TAU * f64::from(i) / 64.0;
            let (u, v) = (1.0 + 0.2 * t.cos(), 10.0 + 2.0 * t.sin());
            Point3::new(10.0 * u.cos(), 10.0 * u.sin(), v)
        })
        .collect();
    let face = kernel
        .make_face(&surface, &FaceBounds::Polylines(vec![outer, hole]), 1e-4)
        .unwrap()
        .face;
    assert_valid("cylinder with hole", &face);
    assert_eq!(face.face_wires(FaceId(0)).len(), 2);
    // Outer 10·2·20 = 400 minus the hole's ellipse in (u·r, v): π·2·2 ≈ 12.566.
    let want = 400.0 - PI * 2.0 * 2.0;
    let got = face.face_area(FaceId(0));
    assert!(rel(got, want) < 2e-3, "{got} vs {want}");
}

fn bspline_patch() -> Surface {
    let poles = (0..5)
        .flat_map(|i| {
            (0..5).map(move |j| {
                let (x, y) = (f64::from(i) * 2.5, f64::from(j) * 2.5);
                Point3::new(x, y, (0.6 * x).sin() * (0.4 * y).cos())
            })
        })
        .collect();
    Surface::BSpline(BSplineSurfaceData {
        u_degree: 3,
        v_degree: 3,
        u_knots: vec![0.0, 0.5, 1.0],
        u_mults: vec![4, 1, 4],
        v_knots: vec![0.0, 0.5, 1.0],
        v_mults: vec![4, 1, 4],
        poles,
        nu: 5,
        nv: 5,
        weights: Some(vec![1.0; 25]),
        u_periodic: false,
        v_periodic: false,
    })
}

#[test]
fn natural_bounds_on_finite_surfaces_are_valid() {
    let kernel = kernel();
    let patch = kernel
        .make_face(&bspline_patch(), &FaceBounds::Natural, 1e-6)
        .unwrap()
        .face;
    assert_valid("5x5 B-spline patch", &patch);
    assert_eq!(patch.count(SubKind::Edge), 4);
    assert!(matches!(patch.surface(FaceId(0)), Surface::BSpline(_)));
    let area = patch.face_area(FaceId(0));
    assert!(area > 100.0 && area < 120.0, "{area}");

    let sphere = kernel
        .make_face(
            &Surface::Sphere {
                frame: Plane::XY,
                radius: 3.0,
            },
            &FaceBounds::Natural,
            1e-6,
        )
        .unwrap()
        .face;
    assert_valid("sphere", &sphere);
    assert!(rel(sphere.face_area(FaceId(0)), 4.0 * PI * 9.0) < 1e-9);
    let torus = kernel
        .make_face(
            &Surface::Torus {
                frame: Plane::XY,
                major: 5.0,
                minor: 1.0,
            },
            &FaceBounds::Natural,
            1e-6,
        )
        .unwrap()
        .face;
    assert_valid("torus", &torus);
    assert!(rel(torus.face_area(FaceId(0)), 4.0 * PI * PI * 5.0) < 1e-9);
    // Polylines on the B-spline patch: the boundary of a UV sub-rectangle.
    let sub = kernel
        .make_face(
            &bspline_patch(),
            &FaceBounds::UvRect {
                u0: 0.1,
                u1: 0.8,
                v0: 0.2,
                v1: 0.9,
            },
            1e-6,
        )
        .unwrap()
        .face;
    let polyline = boundary_polyline(&sub, &fine());
    let rebuilt = kernel
        .make_face(
            &bspline_patch(),
            &FaceBounds::Polylines(vec![polyline]),
            1e-4,
        )
        .unwrap()
        .face;
    assert_valid("B-spline polyline face", &rebuilt);
    assert!(rel(rebuilt.face_area(FaceId(0)), sub.face_area(FaceId(0))) < 1e-3);
}

#[test]
fn infinite_surfaces_need_bounds() {
    let result = kernel().make_face(
        &Surface::Cylinder {
            frame: Plane::XY,
            radius: 1.0,
        },
        &FaceBounds::Natural,
        1e-6,
    );
    assert!(matches!(
        result,
        Err(KernelError::InvalidInput { what: "bounds", .. })
    ));
}

// ---- Loops bounds and wires --------------------------------------------------------------

fn seg(curve: Curve3, t0: f64, t1: f64, tag: u64) -> CurveSeg {
    CurveSeg {
        curve,
        t0,
        t1,
        tag: Tag(tag),
    }
}

fn zline(x: f64, y: f64) -> Curve3 {
    Curve3::Line {
        origin: Point3::new(x, y, 0.0),
        dir: Dir3::Z,
    }
}

fn zcircle(z: f64, r: f64) -> Curve3 {
    Curve3::Circle {
        frame: Plane {
            origin: Point3::new(0.0, 0.0, z),
            ..Plane::XY
        },
        radius: r,
    }
}

#[test]
fn loops_of_3d_curves_bound_a_cylinder_patch() {
    let (r, a0, a1, h) = (5.0, 0.2, 1.4, 10.0);
    let outer = vec![
        seg(zcircle(0.0, r), a0, a1, 1),
        seg(zline(r * a1.cos(), r * a1.sin()), 0.0, h, 2),
        seg(zcircle(h, r), a1, a0, 3),
        seg(zline(r * a0.cos(), r * a0.sin()), h, 0.0, 4),
    ];
    let surface = Surface::Cylinder {
        frame: Plane::XY,
        radius: r,
    };
    let built = kernel()
        .make_face(&surface, &FaceBounds::Loops(vec![outer.clone()]), 1e-6)
        .unwrap();
    assert_valid("cylinder loops", &built.face);
    let want = r * (a1 - a0) * h;
    assert!(rel(built.face.face_area(FaceId(0)), want) < 1e-9);
    assert_eq!(
        built.edge_tags.iter().map(|(_, t)| t.0).collect::<Vec<_>>(),
        [1, 2, 3, 4]
    );
    // Given the other way round, the face is the same.
    let mut back: Vec<CurveSeg> = outer
        .iter()
        .rev()
        .map(|s| seg(s.curve.clone(), s.t1, s.t0, s.tag.0))
        .collect();
    back.rotate_left(1);
    let again = kernel()
        .make_face(&surface, &FaceBounds::Loops(vec![back]), 1e-6)
        .unwrap();
    assert_valid("cylinder loops reversed", &again.face);
    assert!(rel(again.face.face_area(FaceId(0)), want) < 1e-9);
}

#[test]
fn loops_on_a_plane_with_a_circle_hole() {
    let (w, r) = (10.0, 2.0);
    let corner = |x: f64, y: f64| Point3::new(x, y, 0.0);
    let side = |a: Point3, b: Point3, tag| {
        seg(
            Curve3::Line {
                origin: a,
                dir: Dir3::new_unchecked((b - a).normalize()),
            },
            0.0,
            (b - a).length(),
            tag,
        )
    };
    let outer = vec![
        side(corner(0.0, 0.0), corner(w, 0.0), 1),
        side(corner(w, 0.0), corner(w, w), 2),
        side(corner(w, w), corner(0.0, w), 3),
        side(corner(0.0, w), corner(0.0, 0.0), 4),
    ];
    let hole = vec![seg(
        Curve3::Circle {
            frame: Plane {
                origin: Point3::new(5.0, 5.0, 0.0),
                ..Plane::XY
            },
            radius: r,
        },
        0.0,
        TAU,
        5,
    )];
    let built = kernel()
        .make_face(
            &Surface::Plane(Plane::XY),
            &FaceBounds::Loops(vec![outer, hole]),
            1e-6,
        )
        .unwrap();
    assert_valid("plane loops", &built.face);
    assert!(rel(built.face.face_area(FaceId(0)), w * w - PI * r * r) < 1e-9);
    assert_eq!(built.edge_tags.len(), 5);
}

#[test]
fn wires_carry_their_tags_and_close() {
    let (r, h) = (3.0, 4.0);
    let open = vec![
        seg(zline(r, 0.0), h, 0.0, 7),
        seg(zcircle(0.0, r), 0.0, FRAC_PI_2, 8),
        seg(zline(0.0, r), 0.0, h, 9),
    ];
    let result = kernel().make_wire(&open, &TOL).unwrap();
    let wire = &result.shape;
    assert_eq!(wire.kind(), ShapeKind::Wire);
    assert_eq!(wire.count(SubKind::Edge), 3);
    assert_eq!(wire.count(SubKind::Vertex), 4);
    let mut tags: Vec<u64> = result.history.tags.iter().map(|(_, t)| t.0).collect();
    tags.sort_unstable();
    assert_eq!(tags, [7, 8, 9]);
    for (sub, tag) in &result.history.tags {
        assert_eq!(sub.kind, SubKind::Edge);
        assert_eq!(result.history.tag_of(*sub), Some(*tag));
    }
    // Closing it: back along the top arc.
    let mut closed = open;
    closed.push(seg(zcircle(h, r), FRAC_PI_2, 0.0, 10));
    let result = kernel().make_wire(&closed, &TOL).unwrap();
    assert_eq!(result.shape.count(SubKind::Vertex), 4);
    assert_eq!(result.shape.count(SubKind::Edge), 4);
    assert_valid("closed wire", &result.shape);
    // A gap is reported with its segment.
    let gappy = vec![
        seg(zline(r, 0.0), 0.0, h, 0),
        seg(zcircle(0.0, r), 0.0, FRAC_PI_2, 1),
    ];
    match kernel().make_wire(&gappy, &TOL) {
        Err(KernelError::InvalidInput { what, reason, .. }) => {
            assert_eq!(what, "segs");
            assert!(reason.contains("after segment 0"), "{reason}");
        }
        other => panic!("{other:?}"),
    }
}

// ---- golden ------------------------------------------------------------------------------

/// The Done-when 1 face at the display preset, against `tests/golden/brep/`.
#[test]
fn golden_five_kinds_face() {
    let (profile, _) = five_kinds_profile(Plane::XY);
    let face = kernel()
        .make_planar_faces(&[profile], &TOL)
        .unwrap()
        .remove(0)
        .face;
    let mesh = kernel()
        .tessellate(&face, &TessQuality::display(), &JobCtx::detached())
        .unwrap();
    let (positions, triangles, _) = mesh.to_indexed_parts();
    amg_testkit::assert_golden!(
        "brep",
        "face_profile_five_kinds",
        TriSoup {
            positions: &positions,
            triangles: &triangles,
        }
    );
}

// ---- Done-when 5 -------------------------------------------------------------------------

/// One edge of a random star-shaped loop from `a` to `b`: a line, an arc or a rational quadratic
/// B-spline, bulging by `bulge` × chord (positive = to the right of travel).
fn random_edge(a: Point2, b: Point2, kind: u8, bulge: f64, weight: f64) -> Curve2d {
    let chord = b - a;
    let c = chord.length();
    let right = Vec2::new(chord.y, -chord.x) / c;
    let s = bulge * c;
    match kind % 3 {
        0 => Curve2d::Line { a, b },
        1 if s.abs() > 1e-6 => {
            // Circle through a, b with sagitta s on the right (s > 0) or left.
            let radius = (c * c / 4.0 + s * s) / (2.0 * s.abs());
            let mid = a + chord * 0.5;
            let center = mid - right * (s.signum() * (radius - s.abs()));
            let start_angle = (a - center).y.atan2((a - center).x);
            let half = (c / (2.0 * radius)).clamp(-1.0, 1.0).asin();
            let sweep = if s > 0.0 { 2.0 * half } else { -2.0 * half };
            Curve2d::Arc {
                center,
                radius,
                start_angle,
                sweep,
            }
        }
        _ => {
            let mid = a + chord * 0.5 + right * (2.0 * s);
            Curve2d::BSpline(BSpline2dData {
                degree: 2,
                knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                poles: vec![a, mid, b],
                weights: Some(vec![1.0, weight, 1.0]),
            })
        }
    }
}

#[derive(Clone, Debug)]
struct RandomProfile {
    weights: Vec<f64>,
    radii: Vec<f64>,
    kinds: Vec<u8>,
    bulges: Vec<f64>,
    spline_weight: f64,
    hole: u8,
    defect: u8,
}

fn random_profile() -> impl Strategy<Value = RandomProfile> {
    (4usize..12).prop_flat_map(|n| {
        (
            prop::collection::vec(1.0f64..1.5, n),
            prop::collection::vec(10.0f64..20.0, n),
            prop::collection::vec(0u8..3, n),
            prop::collection::vec(-0.04f64..0.04, n),
            0.5f64..2.0,
            0u8..3,
            0u8..10,
        )
            .prop_map(
                |(weights, radii, kinds, bulges, spline_weight, hole, defect)| RandomProfile {
                    weights,
                    radii,
                    kinds,
                    bulges,
                    spline_weight,
                    hole,
                    defect,
                },
            )
    })
}

fn build_random(p: &RandomProfile) -> PlanarProfile {
    let total: f64 = p.weights.iter().sum();
    let mut angle = 0.0_f64;
    let pts: Vec<Point2> = p
        .weights
        .iter()
        .zip(&p.radii)
        .map(|(w, r)| {
            let q = Point2::new(r * angle.cos(), r * angle.sin());
            angle += TAU * w / total;
            q
        })
        .collect();
    let n = pts.len();
    let mut edges: Vec<ProfileEdge2> = (0..n)
        .map(|i| {
            pe(
                random_edge(
                    pts[i],
                    pts[(i + 1) % n],
                    p.kinds[i],
                    p.bulges[i],
                    p.spline_weight,
                ),
                i as u64,
            )
        })
        .collect();
    let mut holes = match p.hole {
        0 => vec![],
        1 => vec![Loop2 {
            edges: vec![pe(
                Curve2d::Circle {
                    center: Point2::ZERO,
                    radius: 0.8,
                },
                100,
            )],
        }],
        _ => vec![rect(-0.5, -0.5, 0.5, 0.5, false, 100)],
    };
    // Some defects, for the clean-error half.
    match p.defect {
        0 => {
            edges[0].curve = Curve2d::Line {
                a: pts[0],
                b: pts[1] + Vec2::new(0.01, 0.0),
            };
        }
        1 => edges.reverse(),
        2 => holes.push(rect(40.0, 40.0, 41.0, 41.0, false, 200)),
        _ => {}
    }
    PlanarProfile {
        plane: tilted_plane(),
        outer: Loop2 { edges },
        holes,
    }
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 1000,
        failure_persistence: None,
        ..ProptestConfig::default()
    })]

    #[test]
    fn random_profiles_give_valid_faces_or_clean_errors(p in random_profile()) {
        let profile = build_random(&p);
        let kernel = raw_kernel();
        match kernel.make_planar_faces(std::slice::from_ref(&profile), &TOL) {
            Ok(mut built) => {
                prop_assert!(p.defect > 2, "a defective profile was accepted: {p:?}");
                let b = built.remove(0);
                let report = kernel.check(&b.face, &CheckOpts::default());
                prop_assert!(report.valid, "{report:?} for {p:?}");
                let n_edges = profile.outer.edges.len()
                    + profile.holes.iter().map(|h| h.edges.len()).sum::<usize>();
                prop_assert_eq!(b.edge_tags.len(), n_edges);
                prop_assert!(b.face.face_area(FaceId(0)) > 0.0);
            }
            Err(KernelError::OpenLoop { .. } | KernelError::InvalidInput { .. }) => {
                prop_assert!(p.defect <= 2, "a valid profile was rejected: {p:?}");
            }
            Err(other) => prop_assert!(false, "unexpected error {other:?} for {p:?}"),
        }
    }
}

#[test]
fn tags_are_on_the_right_edges() {
    // Each tagged edge's curve matches its input curve's kind and end points.
    let (profile, _) = five_kinds_profile(Plane::XY);
    let built = kernel()
        .make_planar_faces(std::slice::from_ref(&profile), &TOL)
        .unwrap();
    let face = &built[0].face;
    let inputs: Vec<&ProfileEdge2> = profile
        .outer
        .edges
        .iter()
        .chain(&profile.holes[0].edges)
        .collect();
    for ((edge, tag), input) in built[0].edge_tags.iter().zip(inputs) {
        assert_eq!(*tag, input.tag);
        let (curve, _, _) = face.curve(*edge).unwrap();
        let kind_ok = matches!(
            (&input.curve, &curve),
            (Curve2d::Line { .. }, Curve3::Line { .. })
                | (
                    Curve2d::Arc { .. } | Curve2d::Circle { .. },
                    Curve3::Circle { .. }
                )
                | (Curve2d::Ellipse { .. }, Curve3::Ellipse { .. })
                | (Curve2d::BSpline(_), Curve3::BSpline(_))
        );
        assert!(kind_ok, "{tag:?}: {curve:?}");
        let sub = SubShape {
            kind: SubKind::Edge,
            index: edge.0,
        };
        assert!(face.tolerance(sub) < 1e-6);
    }
}
