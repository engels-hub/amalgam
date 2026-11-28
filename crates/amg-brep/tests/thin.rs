//! S04 R-S04-3 (SOL-18): `make_thin_face`, the wall along an open or closed planar chain,
//! through OCCT. Areas are closed forms: a wall's area is its width times the length of its
//! mid line (plus the corner terms of sharp joints).
#![cfg(feature = "occt")]
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use amg_brep::geom::{BSpline2dData, Curve2d, ProfileEdge2, Tag, ThinProfile};
use amg_brep::{
    BrepKernel, CheckOpts, FaceId, KernelConfig, KernelError, OcctKernel, SubKind, ThinFace,
    Tolerance, Validate,
};
use amg_core::{Plane, Point2, Point3, Vec2};

const TOL: Tolerance = Tolerance::DEFAULT;

fn kernel() -> OcctKernel {
    OcctKernel::new(KernelConfig {
        validate: Validate::Off,
        ..KernelConfig::default()
    })
}

fn line(a: (f64, f64), b: (f64, f64)) -> Curve2d {
    Curve2d::Line {
        a: Point2::new(a.0, a.1),
        b: Point2::new(b.0, b.1),
    }
}

fn arc(c: (f64, f64), radius: f64, start_angle: f64, sweep: f64) -> Curve2d {
    Curve2d::Arc {
        center: Point2::new(c.0, c.1),
        radius,
        start_angle,
        sweep,
    }
}

fn profile(curves: Vec<Curve2d>, closed: bool, left: f64, right: f64) -> ThinProfile {
    ThinProfile {
        plane: Plane::XY,
        chain: curves
            .into_iter()
            .zip(10u64..)
            .map(|(curve, t)| ProfileEdge2 { curve, tag: Tag(t) })
            .collect(),
        closed,
        left,
        right,
        end_tags: [Tag(1), Tag(2)],
    }
}

/// The wall, checked valid, with every edge tagged and `left` on the chain's left.
fn built(p: &ThinProfile) -> ThinFace {
    let k = kernel();
    let f = k.make_thin_face(p, &TOL).unwrap();
    let report = k.check(&f.build.face, &CheckOpts::default());
    assert!(report.valid, "{report:?}");
    assert_eq!(f.build.face.count(SubKind::Face), 1);
    assert_eq!(
        f.build.edge_tags.len(),
        f.build.face.count(SubKind::Edge) as usize,
        "{:?}",
        f.build.edge_tags
    );
    f
}

fn area(f: &ThinFace) -> f64 {
    f.build.face.face_area(FaceId(0))
}

fn assert_rel(got: f64, want: f64, rel: f64) {
    assert!((got - want).abs() <= rel * want.abs(), "{got} != {want}");
}

/// A line of 10 followed by a tangent quarter arc of radius 5 turning left.
fn line_arc() -> Vec<Curve2d> {
    vec![
        line((0.0, 0.0), (10.0, 0.0)),
        arc((10.0, 5.0), 5.0, -FRAC_PI_2, FRAC_PI_2),
    ]
}

#[test]
fn open_line_arc_chain_in_each_side_mode() {
    let t = 2.0;
    let len = 10.0 + 5.0 * FRAC_PI_2;
    // (left, right, mid-line offset to the left)
    for (left, right, shift) in [(t, 0.0, t / 2.0), (0.0, t, -t / 2.0), (1.0, 1.0, 0.0)] {
        let f = built(&profile(line_arc(), false, left, right));
        assert_rel(area(&f), t * (len - FRAC_PI_2 * shift), 1e-12);
        let tags: Vec<u64> = f.build.edge_tags.iter().map(|(_, t)| t.0).collect();
        assert_eq!(tags, [10, 11, 10, 11, 1, 2]);
        let left_tags: Vec<u64> = f
            .build
            .edge_tags
            .iter()
            .filter(|(e, _)| f.left.contains(e))
            .map(|(_, t)| t.0)
            .collect();
        assert_eq!(left_tags, [10, 11]);
        // The left boundary of the straight edge lies at y = +left.
        let first = f.build.face.edge_vertices(f.left[0]);
        for v in first {
            let p: Point3 = f.build.face.vertex_point(v);
            assert!((p.y - left).abs() < 1e-9, "{p}");
        }
        // The start cap lies at x = 0, across the chain.
        let cap = f.build.edge_tags[4].0;
        let ends = f.build.face.edge_vertices(cap);
        for v in ends {
            let p = f.build.face.vertex_point(v);
            assert!(p.x.abs() < 1e-9 && p.y >= -right - 1e-9 && p.y <= left + 1e-9);
        }
    }
}

#[test]
fn a_single_straight_edge_and_a_sharp_corner() {
    // One line: its offsets don't define a plane on their own.
    for (left, right) in [(2.0, 0.0), (0.0, 2.0), (1.0, 1.0)] {
        let f = built(&profile(
            vec![line((1.0, 1.0), (1.0, 11.0))],
            false,
            left,
            right,
        ));
        assert_rel(area(&f), 20.0, 1e-12);
    }
    // An L turning left: sharp corners on both sides.
    let l = || {
        vec![
            line((0.0, 0.0), (10.0, 0.0)),
            line((10.0, 0.0), (10.0, 6.0)),
        ]
    };
    for (left, right, want) in [
        (2.0, 0.0, 2.0 * 16.0 - 4.0),
        (0.0, 2.0, 2.0 * 16.0 + 4.0),
        (1.0, 1.0, 2.0 * 16.0),
    ] {
        assert_rel(area(&built(&profile(l(), false, left, right))), want, 1e-12);
    }
}

#[test]
fn closed_chains_make_rings() {
    let rect = || {
        vec![
            line((0.0, 0.0), (20.0, 0.0)),
            line((20.0, 0.0), (20.0, 10.0)),
            line((20.0, 10.0), (0.0, 10.0)),
            line((0.0, 10.0), (0.0, 0.0)),
        ]
    };
    // A counter-clockwise chain: left is inside.
    for (left, right) in [(2.0, 0.0), (0.0, 2.0), (1.0, 1.0)] {
        let f = built(&profile(rect(), true, left, right));
        let outer = (20.0 + 2.0 * right) * (10.0 + 2.0 * right);
        let inner = (20.0 - 2.0 * left) * (10.0 - 2.0 * left);
        assert_rel(area(&f), outer - inner, 1e-12);
        assert_eq!(f.build.edge_tags.len(), 8);
        assert_eq!(f.left.len(), 4);
        assert_eq!(f.build.face.face_wires(FaceId(0)).len(), 2);
    }
    // A lone circle.
    for (left, right) in [(1.0, 0.0), (0.0, 1.0), (0.5, 0.5)] {
        let c = Curve2d::Circle {
            center: Point2::new(3.0, -2.0),
            radius: 4.0,
        };
        let f = built(&profile(vec![c], true, left, right));
        let (ro, ri) = (4.0 + right, 4.0 - left);
        assert_rel(area(&f), PI * (ro * ro - ri * ri), 1e-12);
        for e in &f.left {
            let (curve, t0, t1) = f.build.face.curve(*e).unwrap();
            let p = curve.point_at(0.5 * (t0 + t1)).unwrap();
            let r = (p - Point3::new(3.0, -2.0, 0.0)).length();
            assert!((r - ri).abs() < 1e-9, "{r} != {ri}");
        }
    }
}

#[test]
fn splines_and_ellipses_make_walls() {
    // A parabola-like spline; the centred wall's area is close to width × length.
    let spline = Curve2d::BSpline(BSpline2dData {
        degree: 2,
        knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        poles: vec![
            Point2::new(0.0, 0.0),
            Point2::new(20.0, 20.0),
            Point2::new(40.0, 0.0),
        ],
        weights: None,
    });
    // Length of y = x - x²/40 on 0..40: 20·(√2 + asinh 1).
    let len = 20.0 * (2.0_f64.sqrt() + 1.0_f64.asinh());
    let f = built(&profile(vec![spline.clone()], false, 0.5, 0.5));
    assert_rel(area(&f), len, 1e-8);
    // Both boundaries stay half the width from the curve, and the tolerances stay tight.
    let parabola = |x: f64| Point3::new(x, x - x * x / 40.0, 0.0);
    for (e, tag) in &f.build.edge_tags {
        if tag.0 != 10 {
            continue;
        }
        let (curve, t0, t1) = f.build.face.curve(*e).unwrap();
        for i in 0..=8 {
            let p = curve.point_at(t0 + (t1 - t0) * f64::from(i) / 8.0).unwrap();
            // Nearest point by bisection on the derivative of the squared distance.
            let (mut lo, mut hi) = (-2.0_f64, 42.0_f64);
            for _ in 0..100 {
                let x = 0.5 * (lo + hi);
                let tangent = amg_core::Vec3::new(1.0, 1.0 - x / 20.0, 0.0);
                if (parabola(x) - p).dot(tangent) > 0.0 {
                    hi = x;
                } else {
                    lo = x;
                }
            }
            let d = (parabola(lo) - p).length();
            assert!((d - 0.5).abs() < 1e-7, "{d}");
        }
    }
    assert!(f.build.face.max_tolerance() <= 1e-6);
    // One side only: the mid line is longer by the turn (a quarter) times half the width.
    let f = built(&profile(vec![spline], false, 2.0, 0.0));
    assert_rel(area(&f), 2.0 * (len + FRAC_PI_2), 1e-8);
    let ellipse = Curve2d::Ellipse {
        center: Point2::ZERO,
        major: Vec2::new(10.0, 0.0),
        minor_radius: 6.0,
        t0: 0.0,
        t1: TAU,
    };
    // Centred wall on a closed convex curve: width × length exactly.
    let f = built(&profile(vec![ellipse], true, 0.5, 0.5));
    assert_rel(area(&f), 51.053_997_726_789, 1e-4);
}

#[test]
fn walls_that_cannot_be_built_fail() {
    let k = kernel();
    let circle = Curve2d::Circle {
        center: Point2::ZERO,
        radius: 2.0,
    };
    // Wider than the circle's radius, inwards.
    let e = k
        .make_thin_face(&profile(vec![circle.clone()], true, 3.0, 0.0), &TOL)
        .unwrap_err();
    assert!(matches!(e, KernelError::Occt { .. }), "{e:?}");
    // Wider than a rectangle's inner room.
    let rect = vec![
        line((0.0, 0.0), (20.0, 0.0)),
        line((20.0, 0.0), (20.0, 4.0)),
        line((20.0, 4.0), (0.0, 4.0)),
        line((0.0, 4.0), (0.0, 0.0)),
    ];
    assert!(
        k.make_thin_face(&profile(rect, true, 3.0, 0.0), &TOL)
            .is_err()
    );
    // Static input errors.
    for p in [
        profile(vec![], false, 1.0, 0.0),
        profile(line_arc(), false, 0.0, 0.0),
        profile(line_arc(), false, -1.0, 2.0),
        profile(vec![circle], false, 1.0, 0.0),
        profile(
            vec![line((0.0, 0.0), (1.0, 0.0)), line((1.0, 0.0), (0.0, 0.0))],
            false,
            1.0,
            0.0,
        ),
    ] {
        let e = k.make_thin_face(&p, &TOL).unwrap_err();
        assert!(matches!(e, KernelError::InvalidInput { .. }), "{e:?}");
    }
    let gap = profile(
        vec![line((0.0, 0.0), (1.0, 0.0)), line((2.0, 0.0), (3.0, 0.0))],
        false,
        1.0,
        0.0,
    );
    assert!(matches!(
        k.make_thin_face(&gap, &TOL),
        Err(KernelError::OpenLoop { .. })
    ));
}

#[test]
fn split_edges_and_sharp_corners_stay_exact() {
    // Collinear pieces (a sketch line split by a junction), open and closed.
    let open = || vec![line((0.0, 0.0), (5.0, 0.0)), line((5.0, 0.0), (10.0, 0.0))];
    let rect = || {
        vec![
            line((0.0, 0.0), (5.0, 0.0)),
            line((5.0, 0.0), (20.0, 0.0)),
            line((20.0, 0.0), (20.0, 10.0)),
            line((20.0, 10.0), (0.0, 10.0)),
            line((0.0, 10.0), (0.0, 0.0)),
        ]
    };
    let halves = || vec![arc((0.0, 0.0), 8.0, 0.0, PI), arc((0.0, 0.0), 8.0, PI, PI)];
    for (left, right, ring, annulus) in [
        (2.0, 0.0, 104.0, PI * (64.0 - 36.0)),
        (0.0, 2.0, 136.0, PI * (100.0 - 64.0)),
        (1.0, 1.0, 120.0, PI * (81.0 - 49.0)),
    ] {
        assert_rel(
            area(&built(&profile(open(), false, left, right))),
            20.0,
            1e-12,
        );
        assert_rel(
            area(&built(&profile(rect(), true, left, right))),
            ring,
            1e-12,
        );
        assert_rel(
            area(&built(&profile(halves(), true, left, right))),
            annulus,
            1e-12,
        );
    }
    // Two edges at an interior angle: sharp (mitred) corners down to 15 degrees.
    for deg in [135.0_f64, 60.0, 30.0, 15.0] {
        let (s, c) = deg.to_radians().sin_cos();
        let chain = || {
            vec![
                line((0.0, 0.0), (20.0, 0.0)),
                line((20.0, 0.0), (20.0 - 20.0 * c, 20.0 * s)),
            ]
        };
        let corner = 0.25 / (0.5 * deg.to_radians()).tan();
        for (left, right, want) in [(0.5, 0.0, 20.0 - corner), (0.0, 0.5, 20.0 + corner)] {
            assert_rel(
                area(&built(&profile(chain(), false, left, right))),
                want,
                1e-12,
            );
        }
    }
}

#[test]
fn walls_that_touch_themselves_fail() {
    // A hairpin 2 wide: walls of 1 mm inwards meet in the middle.
    let hairpin = vec![
        line((0.0, 0.0), (10.0, 0.0)),
        line((10.0, 0.0), (10.0, 2.0)),
        line((10.0, 2.0), (0.0, 2.0)),
    ];
    let e = kernel()
        .make_thin_face(&profile(hairpin.clone(), false, 1.0, 0.0), &TOL)
        .unwrap_err();
    assert!(e.to_string().contains("touches itself"), "{e}");
    assert_rel(
        area(&built(&profile(hairpin, false, 0.9, 0.0))),
        18.18,
        1e-12,
    );
}
