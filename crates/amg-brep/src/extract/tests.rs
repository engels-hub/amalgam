#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    reason = "test code; exact results are compared bit for bit"
)]

use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, PI, TAU};

use super::*;

const TOL: f64 = 1e-6;

fn plane(origin: Point3, normal: Vec3) -> Plane {
    Plane::from_normal(origin, Dir3::new_unchecked(normal.normalize()))
}

fn tilted() -> Plane {
    plane(Point3::new(1.0, -2.0, 3.0), Vec3::new(0.3, -0.4, 0.85))
}

fn frame(origin: Point3, normal: Vec3) -> Plane {
    plane(origin, normal)
}

/// Every image piece, sampled, lies on the projection of the original at the matching
/// parameter (the maps are affine in the parameter, piece by piece).
fn assert_image(curve: &Curve3, t0: f64, t1: f64, onto: &Plane, pieces: &[Piece]) {
    let proj = Projector::new(onto);
    assert!(!pieces.is_empty());
    for (image, s0, s1) in pieces {
        for k in 0..=16 {
            let s = s0 + (s1 - s0) * f64::from(k) / 16.0;
            let p = image.point_at(s).unwrap();
            // On the plane …
            assert!(
                (p - onto.origin).dot(onto.normal.get()).abs() < 1e-9,
                "{p:?}"
            );
            // … and on the projected original (closest sample).
            let best = (0..=4000)
                .map(|i| {
                    let t = t0 + (t1 - t0) * f64::from(i) / 4000.0;
                    (proj.point(curve.point_at(t).unwrap()) - p).length()
                })
                .fold(f64::INFINITY, f64::min);
            assert!(
                best < 2e-3 * (t1 - t0).abs().max(1.0),
                "{image:?} at {s}: {best}"
            );
        }
    }
    // End points match exactly.
    let first = &pieces[0];
    let last = &pieces[pieces.len() - 1];
    let a = proj.point(curve.point_at(t0).unwrap());
    let b = proj.point(curve.point_at(t1).unwrap());
    assert!((first.0.point_at(first.1).unwrap() - a).length() < 1e-9);
    assert!((last.0.point_at(last.2).unwrap() - b).length() < 1e-9);
}

/// For a single piece the parameter map is affine: the image at the interpolated parameter is
/// the projection of the original at the interpolated parameter.
fn assert_affine(curve: &Curve3, t0: f64, t1: f64, onto: &Plane, piece: &Piece) {
    let proj = Projector::new(onto);
    let (image, s0, s1) = piece;
    for k in 0..=20 {
        let f = f64::from(k) / 20.0;
        let want = proj.point(curve.point_at(t0 + (t1 - t0) * f).unwrap());
        let got = image.point_at(s0 + (s1 - s0) * f).unwrap();
        assert!((want - got).length() < 1e-9, "{k}: {want:?} vs {got:?}");
    }
}

#[test]
fn lines_stay_lines_with_scaled_parameters() {
    let line = Curve3::Line {
        origin: Point3::new(1.0, 2.0, 3.0),
        dir: Dir3::new_unchecked(Vec3::new(1.0, 1.0, 1.0).normalize()),
    };
    let pieces = project_curve(&line, -2.0, 5.0, &Plane::XY, TOL);
    assert_eq!(pieces.len(), 1);
    let (image, s0, s1) = &pieces[0];
    assert!(matches!(image, Curve3::Line { .. }));
    assert!((s1 - s0 - 7.0 * (2.0f64 / 3.0).sqrt()).abs() < 1e-12);
    assert_affine(&line, -2.0, 5.0, &Plane::XY, &pieces[0]);
    // Along the normal: a point, dropped.
    let vertical = Curve3::Line {
        origin: Point3::ZERO,
        dir: Dir3::Z,
    };
    assert!(project_curve(&vertical, 0.0, 10.0, &Plane::XY, TOL).is_empty());
}

#[test]
fn a_parallel_circle_stays_a_circle_and_keeps_its_x_axis() {
    let circle = Curve3::Circle {
        frame: Plane {
            origin: Point3::new(3.0, 1.0, 7.0),
            ..Plane::XY
        },
        radius: 5.0,
    };
    let pieces = project_curve(&circle, 0.25, 2.0, &Plane::XY, TOL);
    assert_eq!(pieces.len(), 1);
    let (image, s0, s1) = &pieces[0];
    let Curve3::Circle { frame, radius } = image else {
        panic!("{image:?}")
    };
    assert_eq!(*radius, 5.0);
    assert_eq!(frame.origin, Point3::new(3.0, 1.0, 0.0));
    assert_eq!(frame.x, Dir3::X);
    assert_eq!((*s0, *s1), (0.25, 2.0));
    assert_affine(&circle, 0.25, 2.0, &Plane::XY, &pieces[0]);
}

#[test]
fn a_circle_seen_from_below_runs_clockwise() {
    let circle = Curve3::Circle {
        frame: frame(Point3::ZERO, Vec3::NEG_Z),
        radius: 2.0,
    };
    let pieces = project_curve(&circle, 0.0, 1.0, &Plane::XY, TOL);
    let (image, s0, s1) = &pieces[0];
    let Curve3::Circle { frame, .. } = image else {
        panic!("{image:?}")
    };
    assert_eq!(frame.normal, Dir3::Z);
    assert!(s1 < s0, "{s0} {s1}");
    assert_affine(&circle, 0.0, 1.0, &Plane::XY, &pieces[0]);
}

#[test]
fn a_tilted_circle_becomes_an_ellipse_with_analytic_axes() {
    let theta: f64 = 0.6;
    let normal = Vec3::new(0.0, theta.sin(), theta.cos());
    let circle = Curve3::Circle {
        frame: frame(Point3::new(1.0, 2.0, 3.0), normal),
        radius: 4.0,
    };
    let pieces = project_curve(&circle, 0.0, TAU, &Plane::XY, TOL);
    assert_eq!(pieces.len(), 1);
    let (image, s0, s1) = &pieces[0];
    let Curve3::Ellipse {
        frame,
        major_radius,
        minor_radius,
    } = image
    else {
        panic!("{image:?}")
    };
    assert!((major_radius - 4.0).abs() < 1e-12);
    assert!((minor_radius - 4.0 * theta.cos()).abs() < 1e-12);
    assert!(
        frame.x.get().x.abs() > 1.0 - 1e-12,
        "major axis along x: {frame:?}"
    );
    assert_eq!(frame.normal, Dir3::Z);
    assert!(((s1 - s0).abs() - TAU).abs() < 1e-12);
    assert_affine(&circle, 0.0, TAU, &Plane::XY, &pieces[0]);
}

#[test]
fn a_general_ellipse_projects_exactly_onto_a_tilted_plane() {
    let ellipse = Curve3::Ellipse {
        frame: frame(Point3::new(-4.0, 2.0, 9.0), Vec3::new(0.7, 0.2, -0.3)),
        major_radius: 6.0,
        minor_radius: 2.5,
    };
    let onto = tilted();
    for (t0, t1) in [(0.0, TAU), (0.3, 2.1), (-1.0, 4.0)] {
        let pieces = project_curve(&ellipse, t0, t1, &onto, TOL);
        assert_eq!(pieces.len(), 1);
        let (image, ..) = &pieces[0];
        let Curve3::Ellipse {
            frame,
            major_radius,
            minor_radius,
        } = image
        else {
            panic!("{image:?}")
        };
        assert!(major_radius >= minor_radius);
        assert_eq!(frame.normal, onto.normal);
        assert!(frame.x.get().dot(frame.y.get()).abs() < 1e-12);
        assert_affine(&ellipse, t0, t1, &onto, &pieces[0]);
    }
}

#[test]
fn an_edge_on_arc_becomes_segments_split_where_it_turns_back() {
    // In the XZ plane, seen from +Z: x = cos t.
    let circle = Curve3::Circle {
        frame: Plane::XZ,
        radius: 1.0,
    };
    let half = project_curve(&circle, 0.0, PI, &Plane::XY, TOL);
    assert_eq!(half.len(), 1, "{half:?}");
    assert!(matches!(half[0].0, Curve3::Line { .. }));
    assert_image(&circle, 0.0, PI, &Plane::XY, &half);
    let full = project_curve(&circle, 0.0, TAU, &Plane::XY, TOL);
    assert_eq!(full.len(), 2, "{full:?}");
    assert_image(&circle, 0.0, TAU, &Plane::XY, &full);
    let quarter_back = project_curve(&circle, FRAC_PI_2, -FRAC_PI_2, &Plane::XY, TOL);
    assert_eq!(quarter_back.len(), 2, "{quarter_back:?}");
    assert_image(&circle, FRAC_PI_2, -FRAC_PI_2, &Plane::XY, &quarter_back);
    // Through the turning point at π the image runs out to x = -1 and back.
    let back = project_curve(&circle, FRAC_PI_2, PI + FRAC_PI_2, &Plane::XY, TOL);
    assert_eq!(back.len(), 2, "{back:?}");
    assert_image(&circle, FRAC_PI_2, PI + FRAC_PI_2, &Plane::XY, &back);
    let quarter = project_curve(&circle, FRAC_PI_2, PI, &Plane::XY, TOL);
    assert_eq!(quarter.len(), 1, "{quarter:?}");
}

fn quarter_circle_bspline(on: &Plane, r: f64) -> Curve3 {
    let at = |u: f64, v: f64| on.origin + on.x.get() * u + on.y.get() * v;
    Curve3::BSpline(BSplineCurveData {
        degree: 2,
        knots: vec![0.0, 1.0],
        mults: vec![3, 3],
        poles: vec![at(r, 0.0), at(r, r), at(0.0, r)],
        weights: Some(vec![1.0, FRAC_1_SQRT_2, 1.0]),
        periodic: false,
    })
}

#[test]
fn bspline_poles_project_with_weights_kept() {
    let curve = quarter_circle_bspline(
        &frame(Point3::new(0.0, 0.0, 2.0), Vec3::new(0.2, 0.5, 1.0)),
        3.0,
    );
    let onto = tilted();
    let pieces = project_curve(&curve, 0.1, 0.9, &onto, TOL);
    assert_eq!(pieces.len(), 1);
    let (Curve3::BSpline(image), ..) = &pieces[0] else {
        panic!("{pieces:?}")
    };
    assert_eq!(image.weights, Some(vec![1.0, FRAC_1_SQRT_2, 1.0]));
    assert_affine(&curve, 0.1, 0.9, &onto, &pieces[0]);
}

#[test]
fn a_bspline_seen_edge_on_becomes_a_line() {
    // A quarter circle in the XZ plane projects onto the x axis, one way.
    let curve = quarter_circle_bspline(&Plane::XZ, 2.0);
    let pieces = project_curve(&curve, 0.0, 1.0, &Plane::XY, TOL);
    assert_eq!(pieces.len(), 1);
    let (line, s0, s1) = &pieces[0];
    assert!(matches!(line, Curve3::Line { .. }), "{line:?}");
    assert_eq!(*s0, 0.0);
    assert!((s1 - 2.0).abs() < 1e-12);
    assert_image(&curve, 0.0, 1.0, &Plane::XY, &pieces);
}

fn line_seg(a: Point3, b: Point3, tag: u64) -> CurveSeg {
    let d = b - a;
    CurveSeg {
        curve: Curve3::Line {
            origin: a,
            dir: Dir3::new_unchecked(d.normalize()),
        },
        t0: 0.0,
        t1: d.length(),
        tag: Tag(tag),
    }
}

#[test]
fn chains_run_head_to_tail() {
    let p = [
        Point3::ZERO,
        Point3::X,
        Point3::new(1.0, 1.0, 0.0),
        Point3::Y,
    ];
    // A square, shuffled, two sides reversed, plus a separate open segment.
    let mut segs = vec![
        line_seg(p[2], p[3], 2),
        line_seg(Point3::new(5.0, 5.0, 0.0), Point3::new(6.0, 5.0, 0.0), 9),
        line_seg(p[1], p[0], 0),
        line_seg(p[3], p[0], 3),
        line_seg(p[2], p[1], 1),
    ];
    let chains = chain(std::mem::take(&mut segs), 1e-9);
    assert_eq!(chains.len(), 2);
    let square = &chains[0];
    assert_eq!(square.len(), 4);
    for i in 0..4 {
        let next = &square[(i + 1) % 4];
        assert!((square[i].end().unwrap() - next.start().unwrap()).length() < 1e-12);
    }
    assert_eq!(chains[1].len(), 1);
    assert_eq!(chains[1][0].tag, Tag(9));
}

#[test]
fn an_open_chain_grows_at_both_ends() {
    let p: Vec<Point3> = (0..4)
        .map(|i| Point3::new(f64::from(i), 0.0, 0.0))
        .collect();
    let segs = vec![
        line_seg(p[1], p[2], 1),
        line_seg(p[0], p[1], 0),
        line_seg(p[3], p[2], 2),
    ];
    let chains = chain(segs, 1e-9);
    assert_eq!(chains.len(), 1);
    let tags: Vec<u64> = chains[0].iter().map(|s| s.tag.0).collect();
    assert_eq!(tags, [0, 1, 2]);
    assert!((chains[0][2].end().unwrap() - p[3]).length() < 1e-12);
}

fn arc(frame: Plane, t0: f64, t1: f64, tag: u64) -> CurveSeg {
    CurveSeg {
        curve: Curve3::Circle { frame, radius: 5.0 },
        t0,
        t1,
        tag: Tag(tag),
    }
}

#[test]
fn pieces_of_one_circle_merge_across_the_seam() {
    let f = Plane::XY;
    let merged = merge_chain(vec![arc(f, PI, TAU, 0), arc(f, 0.0, PI, 0)], true);
    assert_eq!(merged.len(), 1);
    assert!((merged[0].t0 - PI).abs() < 1e-15 && (merged[0].t1 - 3.0 * PI).abs() < 1e-15);
    // Different faces, or a different circle, stay apart.
    assert_eq!(
        merge_chain(vec![arc(f, PI, TAU, 0), arc(f, 0.0, PI, 1)], true).len(),
        2
    );
    let other = Plane {
        origin: Point3::new(0.0, 0.0, 1e-3),
        ..f
    };
    assert_eq!(
        merge_chain(vec![arc(f, 0.0, PI, 0), arc(other, PI, TAU, 0)], false).len(),
        2
    );
    // Three pieces of a closed loop, the last continuing into the first.
    let three = merge_chain(
        vec![
            arc(f, 1.0, 3.0, 0),
            arc(f, 3.0, 5.0, 0),
            arc(f, 5.0, 1.0 + TAU, 0),
        ],
        true,
    );
    assert_eq!(three.len(), 1);
    assert!((three[0].t1 - three[0].t0 - TAU).abs() < 1e-12);
}

#[test]
fn collinear_line_pieces_merge() {
    let a = line_seg(Point3::ZERO, Point3::X, 4);
    let mut b = a.clone();
    b.t0 = 1.0;
    b.t1 = 2.5;
    let merged = merge_chain(vec![a, b], false);
    assert_eq!(merged.len(), 1);
    assert_eq!((merged[0].t0, merged[0].t1), (0.0, 2.5));
}

#[test]
fn sequences_are_oriented_head_to_tail() {
    let p = [Point3::ZERO, Point3::X, Point3::new(1.0, 1.0, 0.0)];
    let mut segs = vec![
        line_seg(p[1], p[0], 0),
        line_seg(p[2], p[1], 1),
        line_seg(p[2], p[0], 2),
    ];
    orient_in_sequence(&mut segs, 1e-9);
    for i in 0..3 {
        let next = &segs[(i + 1) % 3];
        assert!(
            (segs[i].end().unwrap() - next.start().unwrap()).length() < 1e-12,
            "{i}"
        );
    }
}

#[test]
fn malformed_planes_are_rejected() {
    // `Dir3` cannot hold a non-unit vector, so only the axes and values are checked here.
    let mut bad = Plane::XY;
    bad.x = Dir3::Z;
    assert!(matches!(
        check_plane(&bad),
        Err(KernelError::InvalidInput { what: "plane", .. })
    ));
    let mut bad = Plane::XY;
    bad.origin = Point3::new(f64::NAN, 0.0, 0.0);
    assert!(check_plane(&bad).is_err());
    assert!(check_plane(&tilted()).is_ok());
}
