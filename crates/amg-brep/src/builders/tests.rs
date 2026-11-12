//! Validation and lowering, without OCCT.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use amg_core::{Plane, Point2, Point3, Tolerance, Vec2};

use super::*;
use crate::geom::{Loop2, PlanarProfile, ProfileEdge2};

fn edge(curve: Curve2d, tag: u64) -> ProfileEdge2 {
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

fn rect(x0: f64, y0: f64, x1: f64, y1: f64, ccw: bool) -> Loop2 {
    let mut pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)];
    if !ccw {
        pts.reverse();
    }
    Loop2 {
        edges: (0..4)
            .map(|i| edge(line(pts[i], pts[(i + 1) % 4]), i as u64))
            .collect(),
    }
}

fn profile(outer: Loop2, holes: Vec<Loop2>) -> PlanarProfile {
    PlanarProfile {
        plane: Plane::XY,
        outer,
        holes,
    }
}

const TOL: Tolerance = Tolerance::DEFAULT;

#[test]
fn a_square_with_a_circle_hole_lowers() {
    let hole = Loop2 {
        edges: vec![edge(
            Curve2d::Circle {
                center: Point2::new(5.0, 5.0),
                radius: 2.0,
            },
            9,
        )],
    };
    let (faces, tags) = lower_profiles(
        &[profile(rect(0.0, 0.0, 10.0, 10.0, true), vec![hole])],
        &TOL,
    )
    .unwrap();
    assert_eq!(faces.len(), 1);
    assert_eq!(faces[0].loops.starts, [0, 4, 5]);
    assert_eq!(tags[0], [Tag(0), Tag(1), Tag(2), Tag(3), Tag(9)]);
    // The lone circle of a hole runs clockwise.
    assert!(faces[0].loops.edges[4].reversed);
}

#[test]
fn an_open_loop_names_its_loop_and_edge() {
    let mut hole = rect(2.0, 2.0, 4.0, 4.0, false);
    // Move the start of edge 2, so the gap is after edge 1.
    hole.edges[2].curve = line((4.0, 2.0 + 0.01), (2.0, 2.0));
    let p = profile(
        rect(0.0, 0.0, 10.0, 10.0, true),
        vec![rect(6.0, 6.0, 8.0, 8.0, false), hole],
    );
    match lower_profiles(&[p], &TOL) {
        Err(KernelError::OpenLoop {
            loop_index,
            edge_index,
            gap,
        }) => {
            assert_eq!((loop_index, edge_index), (2, 1));
            assert!((gap - 0.01).abs() < 1e-12, "{gap}");
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn a_hole_outside_the_outer_loop_is_invalid() {
    let p = profile(
        rect(0.0, 0.0, 10.0, 10.0, true),
        vec![rect(20.0, 20.0, 22.0, 22.0, false)],
    );
    match lower_profiles(&[p], &TOL) {
        Err(KernelError::InvalidInput { what, reason, .. }) => {
            assert_eq!(what, "profiles");
            assert!(
                reason.contains("loop 1") && reason.contains("outside"),
                "{reason}"
            );
        }
        other => panic!("{other:?}"),
    }
    // Overlapping the boundary is outside too.
    let p = profile(
        rect(0.0, 0.0, 10.0, 10.0, true),
        vec![rect(8.0, 8.0, 12.0, 9.0, false)],
    );
    assert!(matches!(
        lower_profiles(&[p], &TOL),
        Err(KernelError::InvalidInput { .. })
    ));
}

#[test]
fn wrong_orientations_are_invalid() {
    let cw_outer = profile(rect(0.0, 0.0, 10.0, 10.0, false), vec![]);
    let ccw_hole = profile(
        rect(0.0, 0.0, 10.0, 10.0, true),
        vec![rect(2.0, 2.0, 4.0, 4.0, true)],
    );
    for p in [cw_outer, ccw_hole] {
        match lower_profiles(&[p], &TOL) {
            Err(KernelError::InvalidInput { reason, .. }) => {
                assert!(reason.contains("must run"), "{reason}");
            }
            other => panic!("{other:?}"),
        }
    }
}

#[test]
fn malformed_curves_are_invalid() {
    let bad = [
        Curve2d::Circle {
            center: Point2::ZERO,
            radius: 0.0,
        },
        Curve2d::Arc {
            center: Point2::ZERO,
            radius: 1.0,
            start_angle: 0.0,
            sweep: 7.0,
        },
        Curve2d::Ellipse {
            center: Point2::ZERO,
            major: Vec2::ZERO,
            minor_radius: 1.0,
            t0: 0.0,
            t1: 1.0,
        },
        Curve2d::BSpline(BSpline2dData {
            degree: 2,
            knots: vec![0.0, 1.0],
            poles: vec![Point2::ZERO, Point2::X, Point2::Y],
            weights: None,
        }),
        line((0.0, 0.0), (0.0, 0.0)),
    ];
    for c in bad {
        let p = profile(
            Loop2 {
                edges: vec![edge(c.clone(), 0)],
            },
            vec![],
        );
        assert!(
            matches!(
                lower_profiles(&[p], &TOL),
                Err(KernelError::InvalidInput { .. })
            ),
            "{c:?}"
        );
    }
    // A circle sharing a loop with other edges.
    let mut lp = rect(0.0, 0.0, 10.0, 10.0, true);
    lp.edges.push(edge(
        Curve2d::Circle {
            center: Point2::ZERO,
            radius: 1.0,
        },
        5,
    ));
    assert!(matches!(
        lower_profiles(&[profile(lp, vec![])], &TOL),
        Err(KernelError::InvalidInput { .. })
    ));
}

#[test]
fn arcs_and_ellipses_lower_with_increasing_parameters() {
    let cw_arc = Curve2d::Arc {
        center: Point2::ZERO,
        radius: 2.0,
        start_angle: PI,
        sweep: -FRAC_PI_2,
    };
    let e = lower_curve2(&Plane::XY, &cw_arc, false);
    assert!(e.reversed && e.first < e.last);
    assert!((e.last - PI).abs() < 1e-15 && (e.first - FRAC_PI_2).abs() < 1e-15);
    let full = Curve2d::Ellipse {
        center: Point2::ZERO,
        major: Vec2::new(3.0, 0.0),
        minor_radius: 1.0,
        t0: TAU,
        t1: 0.0,
    };
    let e = lower_curve2(&Plane::XY, &full, false);
    assert!(e.reversed && (e.last - e.first - TAU).abs() < 1e-15);
}

#[test]
fn polygons_merge_fit_and_orient() {
    let tilt = |x: f64, y: f64| Point3::new(x, y, 0.5 * x);
    let outer = [
        tilt(0.0, 0.0),
        tilt(10.0, 0.0),
        tilt(10.0, 0.0) + Point3::splat(1e-8),
        tilt(10.0, 10.0),
        tilt(0.0, 10.0),
        tilt(0.0, 0.0),
    ];
    // A CCW hole gets turned around.
    let hole = vec![tilt(2.0, 2.0), tilt(4.0, 2.0), tilt(4.0, 4.0)];
    let face = lower_polygon(&outer, &[hole], &TOL).unwrap();
    assert_eq!(face.loops.starts, [0, 4, 7]);
    let n = face.plane.normal.get();
    assert!(
        (n - Point3::new(-0.5, 0.0, 1.0).normalize()).length() < 1e-12,
        "{n:?}"
    );

    let bent = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(10.0, 0.0, 0.0),
        Point3::new(10.0, 10.0, 1e-3),
        Point3::new(0.0, 10.0, 0.0),
    ];
    match lower_polygon(&bent, &[], &TOL) {
        Err(KernelError::NotPlanar { max_dev }) => assert!(max_dev > 1e-4, "{max_dev}"),
        other => panic!("{other:?}"),
    }
    let collinear = [Point3::ZERO, Point3::X, Point3::X * 2.0];
    assert!(matches!(
        lower_polygon(&collinear, &[], &TOL),
        Err(KernelError::InvalidInput { .. })
    ));
}

#[test]
fn face_bounds_are_checked() {
    let cyl = Surface::Cylinder {
        frame: Plane::XY,
        radius: 2.0,
    };
    assert!(matches!(
        lower_face(&cyl, &FaceBounds::Natural, 1e-6),
        Err(KernelError::InvalidInput { what: "bounds", .. })
    ));
    assert!(matches!(
        lower_face(
            &cyl,
            &FaceBounds::UvRect {
                u0: 1.0,
                u1: 0.0,
                v0: 0.0,
                v1: 1.0
            },
            1e-6
        ),
        Err(KernelError::InvalidInput { .. })
    ));
    let other = Surface::Other {
        class: crate::geom::GeomClass::Other,
    };
    assert!(lower_face(&other, &FaceBounds::Natural, 1e-6).is_err());
    // An open loop of 3D segments.
    let seg = |a: Point3, b: Point3, tag| CurveSeg {
        curve: Curve3::Line {
            origin: a,
            dir: Dir3::new_unchecked((b - a).normalize()),
        },
        t0: 0.0,
        t1: (b - a).length(),
        tag: Tag(tag),
    };
    let plane = Surface::Plane(Plane::XY);
    let open = vec![
        seg(Point3::ZERO, Point3::X, 0),
        seg(Point3::X, Point3::new(1.0, 1.0, 0.0), 1),
        seg(Point3::new(1.0, 1.0, 0.0), Point3::new(0.0, 0.1, 0.0), 2),
    ];
    match lower_face(&plane, &FaceBounds::Loops(vec![open]), 1e-6) {
        Err(KernelError::OpenLoop {
            loop_index,
            edge_index,
            ..
        }) => assert_eq!((loop_index, edge_index), (0, 2)),
        other => panic!("{other:?}"),
    }
}

#[test]
fn wires_report_gaps_and_closure() {
    let seg = |a: Point3, b: Point3| CurveSeg {
        curve: Curve3::Line {
            origin: a,
            dir: Dir3::new_unchecked((b - a).normalize()),
        },
        t0: 0.0,
        t1: (b - a).length(),
        tag: Tag(0),
    };
    let (_, closed) = lower_wire(
        &[seg(Point3::ZERO, Point3::X), seg(Point3::X, Point3::Y)],
        &TOL,
    )
    .unwrap();
    assert!(!closed);
    let (_, closed) = lower_wire(
        &[
            seg(Point3::ZERO, Point3::X),
            seg(Point3::X, Point3::Y),
            seg(Point3::Y, Point3::ZERO),
        ],
        &TOL,
    )
    .unwrap();
    assert!(closed);
    assert!(
        lower_wire(
            &[seg(Point3::ZERO, Point3::X), seg(Point3::Y, Point3::Z)],
            &TOL
        )
        .is_err()
    );
}
