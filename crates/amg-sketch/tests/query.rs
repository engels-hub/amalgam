//! SK-20: pick, snap candidates and redundancy suppression.

#![allow(clippy::unwrap_used, clippy::expect_used, reason = "test code")]

use std::f64::consts::{FRAC_PI_2, PI};
use std::time::Instant;

use amg_core::{Plane, Point2};
use amg_sketch::model::HvTarget;
use amg_sketch::query::{
    NEW_ENTITY, PickFilter, QueryIndex, SnapCandidate, SnapKind, SnapOptions, pick,
    snap_candidates, snap_candidates_checked, snap_candidates_with, suppress_redundant,
};
use amg_sketch::solve::{NativeSolver, SolveOptions, SolverBackend};
use amg_sketch::{ConstraintKind as K, Dimension, DimensionKind, EntityId, Sketch};

const R: f64 = 1.0;

fn pt(x: f64, y: f64) -> Point2 {
    Point2::new(x, y)
}

fn of_kind(c: &[SnapCandidate], kind: SnapKind) -> Vec<&SnapCandidate> {
    c.iter().filter(|c| c.kind == kind).collect()
}

fn near(a: Point2, b: Point2) -> bool {
    a.distance(b) < 1e-9
}

fn dim(s: &mut Sketch, kind: DimensionKind, value: f64) {
    s.add_constraint(K::Dimension(Dimension {
        kind,
        value,
        driving: true,
        label_pos: Point2::ZERO,
    }))
    .unwrap();
}

// ---- pick -----------------------------------------------------------------------------------

#[test]
fn pick_orders_by_distance_and_prefers_points() {
    let mut s = Sketch::new(Plane::XY);
    let l = s.add_line(pt(0.0, 0.0), pt(10.0, 0.0)).unwrap();
    let far = s.add_line(pt(0.0, 0.8), pt(10.0, 0.8)).unwrap();
    // Cursor above the left end of the first line.
    let hits = pick(&s, pt(0.0, 0.3), R, PickFilter::ALL);
    let ids: Vec<EntityId> = hits.iter().map(|h| h.entity).collect();
    assert_eq!(ids.len(), 4, "two lines and the near end points: {ids:?}");
    // Nearest first: the end point (0.3), then the line below it (0.3), then the far line.
    assert!(
        hits.windows(2)
            .all(|w| w[0].distance <= w[1].distance + 1e-9)
    );
    assert!(ids.iter().position(|&i| i == l) < ids.iter().position(|&i| i == far));
    let first = &hits[0];
    assert!((first.distance - 0.3).abs() < 1e-12);
    assert!(
        matches!(
            s.entity(first.entity).unwrap().kind,
            amg_sketch::EntityKind::Point { .. }
        ),
        "a point wins a tie with the curve it ends"
    );
    let line_hit = hits.iter().find(|h| h.entity == l).unwrap();
    assert!(line_hit.param.abs() < 1e-12, "closest point is the start");
}

#[test]
fn pick_filter_selects_kinds() {
    let mut s = Sketch::new(Plane::XY);
    let l = s.add_line(pt(0.0, 0.0), pt(10.0, 0.0)).unwrap();
    let c = s.add_line(pt(0.0, 0.2), pt(10.0, 0.2)).unwrap();
    s.set_construction(c, true).unwrap();
    let p = pt(5.0, 0.1);
    let only_curves = PickFilter {
        curves: true,
        ..PickFilter::default()
    };
    let hits = pick(&s, p, R, only_curves);
    assert_eq!(hits.len(), 1);
    assert_eq!(hits[0].entity, l);
    let with_construction = PickFilter {
        construction: true,
        ..only_curves
    };
    assert_eq!(pick(&s, p, R, with_construction).len(), 2);
    let only_points = PickFilter {
        points: true,
        ..PickFilter::default()
    };
    assert!(pick(&s, p, R, only_points).is_empty());
    assert!(pick(&s, p, R, PickFilter::default()).is_empty());
}

#[test]
fn pick_covers_every_curve_kind() {
    let mut s = Sketch::new(Plane::XY);
    let line = s.add_line(pt(0.0, 0.0), pt(10.0, 0.0)).unwrap();
    let circle = s.add_circle(pt(30.0, 0.0), 5.0).unwrap();
    let arc = s
        .add_arc(pt(60.0, 0.0), pt(65.0, 0.0), pt(60.0, 5.0))
        .unwrap();
    let ellipse = s.add_ellipse(pt(90.0, 0.0), pt(96.0, 0.0), 3.0).unwrap();
    let earc = s
        .add_elliptic_arc(pt(120.0, 0.0), pt(126.0, 0.0), 3.0, 0.0, PI)
        .unwrap();
    let spline = s
        .add_spline(
            3,
            &[
                pt(150.0, 0.0),
                pt(153.0, 6.0),
                pt(157.0, -6.0),
                pt(160.0, 0.0),
            ],
            None,
            None,
        )
        .unwrap();
    let conic = s
        .add_conic(pt(180.0, 0.0), pt(185.0, 8.0), pt(190.0, 0.0), 0.4)
        .unwrap();
    let curves = PickFilter {
        curves: true,
        ..PickFilter::default()
    };
    let on = |id: EntityId, p: Point2| {
        let hits = pick(&s, p, 0.1, curves);
        assert!(
            hits.iter().any(|h| h.entity == id),
            "{id:?} at {p:?}: {hits:?}"
        );
        assert!(hits.iter().all(|h| h.distance <= 0.1));
    };
    on(line, pt(5.0, 0.05));
    on(circle, pt(35.0, 0.05));
    on(arc, pt(60.0 + 5.0 * 0.5f64.sqrt(), 5.0 * 0.5f64.sqrt()));
    on(ellipse, pt(90.0, 3.05));
    on(earc, pt(120.0, 3.05));
    let sc = s.curve(spline).unwrap();
    on(spline, sc.eval(0.4 * sc.param_range().1));
    on(conic, s.curve(conic).unwrap().eval(0.5));
    // The elliptical arc covers only the upper half, the arc only a quarter.
    assert!(pick(&s, pt(120.0, -3.0), 0.5, curves).is_empty());
    assert!(pick(&s, pt(60.0, -5.0), 0.5, curves).is_empty());
}

// ---- snap kinds -----------------------------------------------------------------------------

#[test]
fn snap_endpoint_and_center() {
    let mut s = Sketch::new(Plane::XY);
    let l = s.add_line(pt(0.0, 0.0), pt(10.0, 0.0)).unwrap();
    let c = s.add_circle(pt(20.0, 20.0), 3.0).unwrap();
    let amg_sketch::EntityKind::Line { end, .. } = s.entity(l).unwrap().kind else {
        panic!()
    };
    let cands = snap_candidates(&s, pt(10.2, 0.1), R, None);
    let ep = of_kind(&cands, SnapKind::Endpoint);
    assert_eq!(ep.len(), 1);
    assert!(near(ep[0].point, pt(10.0, 0.0)));
    assert_eq!(ep[0].entities.as_slice(), &[end]);
    assert_eq!(
        ep[0].suggested,
        Some(K::Coincident {
            a: NEW_ENTITY,
            b: end
        })
    );
    assert_eq!(cands[0].kind, SnapKind::Endpoint, "best candidate first");

    let amg_sketch::EntityKind::Circle { center, .. } = s.entity(c).unwrap().kind else {
        panic!()
    };
    let cands = snap_candidates(&s, pt(20.3, 19.8), R, None);
    let ce = of_kind(&cands, SnapKind::Center);
    assert_eq!(ce.len(), 1);
    assert!(near(ce[0].point, pt(20.0, 20.0)));
    assert_eq!(ce[0].entities.as_slice(), &[center]);
    assert!(of_kind(&cands, SnapKind::Endpoint).is_empty());
    // Out of range: nothing.
    assert!(snap_candidates(&s, pt(50.0, 50.0), R, None).is_empty());
}

#[test]
fn snap_midpoint_of_every_open_curve() {
    let mut s = Sketch::new(Plane::XY);
    let line = s.add_line(pt(0.0, 0.0), pt(10.0, 0.0)).unwrap();
    let arc = s
        .add_arc(pt(30.0, 0.0), pt(35.0, 0.0), pt(30.0, 5.0))
        .unwrap();
    let earc = s
        .add_elliptic_arc(pt(60.0, 0.0), pt(66.0, 0.0), 3.0, 0.0, PI)
        .unwrap();
    let spline = s
        .add_spline(
            3,
            &[pt(90.0, 0.0), pt(93.0, 6.0), pt(97.0, -6.0), pt(100.0, 0.0)],
            None,
            None,
        )
        .unwrap();
    let conic = s
        .add_conic(pt(120.0, 0.0), pt(125.0, 8.0), pt(130.0, 0.0), 0.4)
        .unwrap();
    let mid = |id: EntityId, p: Point2| {
        let c = snap_candidates(&s, p, 0.05, None);
        let m = of_kind(&c, SnapKind::Midpoint);
        assert_eq!(m.len(), 1, "{id:?}: {c:?}");
        assert_eq!(m[0].entities.as_slice(), &[id]);
        m[0].point
    };
    let m = mid(line, pt(5.0, 0.0));
    assert!(near(m, pt(5.0, 0.0)));
    let line_mid = of_kind(
        &snap_candidates(&s, pt(5.0, 0.0), 0.05, None),
        SnapKind::Midpoint,
    )[0]
    .suggested
    .clone();
    assert_eq!(
        line_mid,
        Some(K::Midpoint {
            point: NEW_ENTITY,
            line
        })
    );
    // Arc: midpoint at 45 degrees.
    let h = 5.0 * 0.5f64.sqrt();
    assert!(near(mid(arc, pt(30.0 + h, h)), pt(30.0 + h, h)));
    // Elliptical arc: eccentric angle pi/2, the minor-axis end.
    assert!(near(mid(earc, pt(60.0, 3.0)), pt(60.0, 3.0)));
    // Spline and conic: on the curve, at half the arc length.
    let sp = s.curve(spline).unwrap();
    let got = mid(spline, sp.eval(0.5 * sp.param_range().1));
    assert!(sp.closest(got).1.distance(got) < 1e-9);
    assert!(
        got.distance(pt(95.0, 0.0)) < 1.0,
        "symmetric spline: {got:?}"
    );
    let co = s.curve(conic).unwrap();
    let got = mid(conic, co.eval(0.5));
    assert!(co.closest(got).1.distance(got) < 1e-9);
    assert!((got.x - 125.0).abs() < 1e-6, "symmetric conic: {got:?}");
}

#[test]
fn snap_quadrant_circle_arc_ellipse() {
    let mut s = Sketch::new(Plane::XY);
    let circle = s.add_circle(pt(0.0, 0.0), 5.0).unwrap();
    let arc = s
        .add_arc(pt(30.0, 0.0), pt(35.0, 0.0), pt(30.0, 5.0))
        .unwrap();
    let ellipse = s.add_ellipse(pt(60.0, 0.0), pt(66.0, 0.0), 3.0).unwrap();
    let earc = s
        .add_elliptic_arc(pt(90.0, 0.0), pt(96.0, 0.0), 3.0, 0.0, PI)
        .unwrap();
    let quads = |p: Point2| {
        of_kind(&snap_candidates(&s, p, 0.2, None), SnapKind::Quadrant)
            .into_iter()
            .map(|c| (c.entities[0], c.point, c.suggested.clone()))
            .collect::<Vec<_>>()
    };
    for (p, q) in [
        (pt(5.1, 0.1), pt(5.0, 0.0)),
        (pt(0.1, 5.1), pt(0.0, 5.0)),
        (pt(-5.1, 0.1), pt(-5.0, 0.0)),
        (pt(0.1, -5.1), pt(0.0, -5.0)),
    ] {
        let r = quads(p);
        assert_eq!(r.len(), 1, "{p:?}");
        assert_eq!(r[0].0, circle);
        assert!(near(r[0].1, q));
        assert_eq!(
            r[0].2,
            Some(K::PointOnCurve {
                point: NEW_ENTITY,
                curve: circle
            })
        );
    }
    // Arc from 0 to 90 degrees: the end quadrants coincide with its end points (the end point
    // wins), and 180 / 270 are outside the arc.
    assert!(quads(pt(30.0, 5.1)).is_empty());
    assert!(quads(pt(25.0, 0.0)).is_empty());
    assert!(quads(pt(30.0, -5.0)).is_empty());
    let c = snap_candidates(&s, pt(30.0, 5.1), 0.2, None);
    assert!(
        of_kind(&c, SnapKind::Endpoint)
            .iter()
            .any(|c| near(c.point, pt(30.0, 5.0)))
    );
    let _ = arc;
    // Ellipse: the four axis ends.
    // (The major-axis end at (66, 0) is a point entity: an endpoint snap covers it.)
    for q in [pt(60.0, 3.0), pt(54.0, 0.0), pt(60.0, -3.0)] {
        let r = quads(q + pt(0.05, 0.05));
        assert!(
            r.iter().any(|(e, p, _)| *e == ellipse && near(*p, q)),
            "{q:?}: {r:?}"
        );
    }
    // Elliptical arc over the upper half: no quadrant on the lower half.
    assert!(quads(pt(90.0, -3.0)).is_empty());
    // At eccentric angle pi / 2 the quadrant coincides with the arc's midpoint, which wins.
    let c = snap_candidates(&s, pt(90.0, 3.0), 0.2, None);
    assert!(
        of_kind(&c, SnapKind::Midpoint)
            .iter()
            .any(|c| c.entities[0] == earc && near(c.point, pt(90.0, 3.0))),
        "{c:?}"
    );
}

#[test]
fn snap_on_curve_for_every_kind() {
    let mut s = Sketch::new(Plane::XY);
    let line = s.add_line(pt(0.0, 0.0), pt(10.0, 0.0)).unwrap();
    let ellipse = s.add_ellipse(pt(30.0, 0.0), pt(36.0, 0.0), 3.0).unwrap();
    let spline = s
        .add_spline(
            3,
            &[pt(60.0, 0.0), pt(63.0, 6.0), pt(67.0, -6.0), pt(70.0, 0.0)],
            None,
            None,
        )
        .unwrap();
    let conic = s
        .add_conic(pt(90.0, 0.0), pt(95.0, 8.0), pt(100.0, 0.0), 0.4)
        .unwrap();
    for (id, on, off) in [
        (line, pt(3.0, 0.0), pt(3.0, 0.4)),
        (ellipse, s.curve(ellipse).unwrap().eval(1.0), pt(0.0, 0.0)),
        (
            spline,
            s.curve(spline)
                .unwrap()
                .eval(0.4 * s.curve(spline).unwrap().param_range().1),
            pt(0.0, 0.0),
        ),
        (conic, s.curve(conic).unwrap().eval(0.3), pt(0.0, 0.0)),
    ] {
        let probe = if off == pt(0.0, 0.0) {
            on + pt(0.0, 0.05)
        } else {
            off
        };
        let c = snap_candidates(&s, probe, 0.5, None);
        let o = of_kind(&c, SnapKind::OnCurve);
        assert_eq!(o.len(), 1, "{id:?}: {c:?}");
        assert_eq!(o[0].entities.as_slice(), &[id]);
        let cv = s.curve(id).unwrap();
        assert!(cv.closest(o[0].point).1.distance(o[0].point) < 1e-9);
        assert!(o[0].point.distance(probe) <= 0.5);
        assert_eq!(
            o[0].suggested,
            Some(K::PointOnCurve {
                point: NEW_ENTITY,
                curve: id
            })
        );
    }
}

#[test]
fn snap_intersection_of_line_line_line_circle_and_ellipse_spline() {
    let mut s = Sketch::new(Plane::XY);
    let a = s.add_line(pt(0.0, 0.0), pt(10.0, 10.0)).unwrap();
    let b = s.add_line(pt(0.0, 10.0), pt(10.0, 0.0)).unwrap();
    let c = s.add_circle(pt(40.0, 0.0), 5.0).unwrap();
    let d = s.add_line(pt(30.0, 3.0), pt(50.0, 3.0)).unwrap();
    let e = s.add_ellipse(pt(80.0, 0.0), pt(86.0, 0.0), 3.0).unwrap();
    let sp = s
        .add_spline(
            2,
            &[pt(74.0, -4.0), pt(80.0, 8.0), pt(86.0, -4.0)],
            None,
            None,
        )
        .unwrap();
    let r = snap_candidates(&s, pt(5.2, 4.9), R, None);
    let x = of_kind(&r, SnapKind::Intersection);
    assert_eq!(x.len(), 1, "{r:?}");
    assert!(near(x[0].point, pt(5.0, 5.0)));
    assert_eq!(x[0].entities.as_slice(), &[a, b]);
    assert_eq!(
        x[0].suggested,
        Some(K::PointOnCurve {
            point: NEW_ENTITY,
            curve: a
        })
    );
    assert_eq!(
        x[0].secondary,
        Some(K::PointOnCurve {
            point: NEW_ENTITY,
            curve: b
        })
    );
    assert!(
        of_kind(&r, SnapKind::OnCurve)
            .iter()
            .all(|c| !near(c.point, pt(5.0, 5.0))),
        "covered by the intersection"
    );
    assert_eq!(x[0].constraints(EntityId(99), EntityId(98)).len(), 2);

    let hx = (25.0f64 - 9.0).sqrt();
    let r = snap_candidates(&s, pt(40.0 + hx, 3.1), R, None);
    let x = of_kind(&r, SnapKind::Intersection);
    assert_eq!(x.len(), 1, "{r:?}");
    assert!(near(x[0].point, pt(40.0 + hx, 3.0)));
    assert_eq!(x[0].entities.as_slice(), &[c, d]);

    // Ellipse and quadratic spline cross somewhere on the upper right.
    let ec = s.curve(e).unwrap();
    let sc = s.curve(sp).unwrap();
    let hits = amg_sketch::geom::intersect(&ec, &sc, &amg_core::Tolerance::DEFAULT);
    assert!(!hits.is_empty());
    let h = hits[0].point;
    let r = snap_candidates(&s, h + pt(0.05, 0.05), R, None);
    let x = of_kind(&r, SnapKind::Intersection);
    assert!(
        x.iter()
            .any(|c| near(c.point, h) && c.entities.as_slice() == [e, sp]),
        "{r:?}"
    );
}

#[test]
fn snap_alignment_with_points_and_anchor() {
    let mut s = Sketch::new(Plane::XY);
    let a = s.add_point(pt(0.0, 0.0)).unwrap();
    let b = s.add_point(pt(20.0, 10.0)).unwrap();
    let f = s.add_point(pt(5.0, 30.0)).unwrap();
    // Cursor right of `a` at the same height: horizontal alignment with `a`.
    let r = snap_candidates(&s, pt(12.0, 0.4), R, None);
    let h = of_kind(&r, SnapKind::AlignHorizontal);
    assert_eq!(h.len(), 1, "{r:?}");
    assert!(near(h[0].point, pt(12.0, 0.0)));
    assert_eq!(h[0].entities.as_slice(), &[a]);
    assert_eq!(
        h[0].suggested,
        Some(K::Horizontal(HvTarget::Points(a, NEW_ENTITY)))
    );
    // Vertical alignment with `b`.
    let r = snap_candidates(&s, pt(20.3, 40.0), R, None);
    let v = of_kind(&r, SnapKind::AlignVertical);
    assert_eq!(v.len(), 1, "{r:?}");
    assert!(near(v[0].point, pt(20.0, 40.0)));
    assert_eq!(v[0].entities.as_slice(), &[b]);
    assert_eq!(
        v[0].suggested,
        Some(K::Vertical(HvTarget::Points(b, NEW_ENTITY)))
    );
    // Rubber band anchored at `f` (found by position): level with it, and above `a`.
    let r = snap_candidates(&s, pt(0.2, 29.7), R, Some(pt(5.0, 30.0)));
    let h = of_kind(&r, SnapKind::AlignHorizontal);
    assert_eq!(h.len(), 1);
    assert!(near(h[0].point, pt(0.2, 30.0)));
    assert_eq!(h[0].entities.as_slice(), &[f]);
    let v = of_kind(&r, SnapKind::AlignVertical);
    assert!(
        v.iter()
            .any(|c| near(c.point, pt(0.0, 29.7)) && c.entities.as_slice() == [a])
    );
    // Near enough to a point to snap to it: that is an endpoint snap, not an alignment.
    let r = snap_candidates(&s, pt(0.2, 0.3), R, None);
    assert!(of_kind(&r, SnapKind::AlignHorizontal).is_empty());
    assert_eq!(r[0].kind, SnapKind::Endpoint);
}

#[test]
fn snap_perpendicular_and_tangent_geometry() {
    let mut s = Sketch::new(Plane::XY);
    let line = s.add_line(pt(0.0, 0.0), pt(10.0, 0.0)).unwrap();
    let circle = s.add_circle(pt(30.0, 0.0), 5.0).unwrap();
    let from = pt(4.0, 6.0);
    // Perpendicular to the line from (4, 6): the foot (4, 0).
    let r = snap_candidates(&s, pt(4.2, 0.3), R, Some(from));
    let p = of_kind(&r, SnapKind::Perpendicular);
    assert_eq!(p.len(), 1, "{r:?}");
    assert!(near(p[0].point, pt(4.0, 0.0)));
    assert_eq!(
        p[0].suggested,
        Some(K::Perpendicular {
            a: NEW_ENTITY,
            b: line
        })
    );
    // Feet outside the segment are not offered.
    let r = snap_candidates(&s, pt(14.0, 0.2), R, Some(pt(14.0, 6.0)));
    assert!(of_kind(&r, SnapKind::Perpendicular).is_empty());

    // Tangent from (40, 8) to the circle: both tangent points satisfy (q - c) . (q - f) = 0.
    let f = pt(40.0, 8.0);
    let c = pt(30.0, 0.0);
    let d = (f - c).length();
    let base = (f - c).y.atan2((f - c).x);
    let del = (5.0 / d).acos();
    for sgn in [-1.0, 1.0] {
        let a = base + sgn * del;
        let q = c + pt(a.cos(), a.sin()) * 5.0;
        let r = snap_candidates(&s, q + pt(0.1, 0.1), R, Some(f));
        let t = of_kind(&r, SnapKind::Tangent);
        assert_eq!(t.len(), 1, "{r:?}");
        assert!(near(t[0].point, q));
        assert!((t[0].point - c).dot(t[0].point - f).abs() < 1e-9);
        assert!(matches!(
            t[0].suggested,
            Some(K::Tangent { a, b, .. }) if a == NEW_ENTITY && b == circle
        ));
    }
    // Inside the circle there is no tangent.
    let r = snap_candidates(&s, pt(30.0, 5.0), R, Some(pt(30.0, 1.0)));
    assert!(of_kind(&r, SnapKind::Tangent).is_empty());
    // Perpendicular to a circle: the radial points.
    let r = snap_candidates(&s, pt(35.1, 0.1), R, Some(pt(40.0, 0.0)));
    let p = of_kind(&r, SnapKind::Perpendicular);
    assert!(p.iter().any(|c| near(c.point, pt(35.0, 0.0))), "{r:?}");
}

#[test]
fn snap_tangent_and_perpendicular_on_ellipse_and_spline() {
    let mut s = Sketch::new(Plane::XY);
    let e = s.add_ellipse(pt(0.0, 0.0), pt(8.0, 0.0), 4.0).unwrap();
    let sp = s
        .add_spline(
            3,
            &[pt(30.0, 0.0), pt(33.0, 8.0), pt(37.0, -8.0), pt(40.0, 0.0)],
            None,
            None,
        )
        .unwrap();
    let from = pt(14.0, 6.0);
    let ec = s.curve(e).unwrap();
    // Every reported candidate is a root of the defining equation.
    let r = snap_candidates(&s, pt(4.0, 3.5), 8.0, Some(from));
    let tans = of_kind(&r, SnapKind::Tangent);
    assert!(!tans.is_empty(), "{r:?}");
    for t in &tans {
        let tp = t.point;
        let theta = match &ec {
            amg_sketch::Curve2::Ellipse(e) => e.angle_of(tp),
            _ => unreachable!(),
        };
        let tangent = match &ec {
            amg_sketch::Curve2::Ellipse(e) => e.tangent_at(theta),
            _ => unreachable!(),
        };
        assert!(
            (tp - from).perp_dot(tangent).abs() < 1e-6 * tangent.length() * (tp - from).length()
        );
        assert!(matches!(t.suggested, Some(K::Tangent { a, b, .. }) if a == NEW_ENTITY && b == e));
    }
    let perps = of_kind(&r, SnapKind::Perpendicular);
    for p in &perps {
        let theta = match &ec {
            amg_sketch::Curve2::Ellipse(e) => e.angle_of(p.point),
            _ => unreachable!(),
        };
        let tangent = match &ec {
            amg_sketch::Curve2::Ellipse(e) => e.tangent_at(theta),
            _ => unreachable!(),
        };
        assert!(
            (p.point - from).dot(tangent).abs()
                < 1e-6 * tangent.length() * (p.point - from).length()
        );
    }
    // Spline: roots as well, no tangent suggestion.
    let sc = s.curve(sp).unwrap();
    let r = snap_candidates(&s, pt(35.0, 0.0), 8.0, Some(pt(35.0, 12.0)));
    let tans = of_kind(&r, SnapKind::Tangent);
    let perps = of_kind(&r, SnapKind::Perpendicular);
    assert!(!tans.is_empty() && !perps.is_empty(), "{r:?}");
    for t in tans {
        let (t_par, q) = sc.closest(t.point);
        assert!(q.distance(t.point) < 1e-6);
        let d = sc.derivative(t_par);
        assert!((t.point - pt(35.0, 12.0)).perp_dot(d).abs() < 1e-5 * d.length() * 12.0);
        assert!(t.suggested.is_none());
    }
    for p in perps {
        let (t_par, _) = sc.closest(p.point);
        let d = sc.derivative(t_par);
        assert!((p.point - pt(35.0, 12.0)).dot(d).abs() < 1e-5 * d.length() * 12.0);
    }
}

#[test]
fn suggestions_are_valid_constraints() {
    // Every suggestion, resolved with a real point / line, is accepted by `add_constraint`
    // and the solver keeps the sketch satisfiable.
    let mut base = Sketch::new(Plane::XY);
    let line = base.add_line(pt(0.0, 0.0), pt(10.0, 0.0)).unwrap();
    let other = base.add_line(pt(0.0, 10.0), pt(10.0, 0.0)).unwrap();
    let circle = base.add_circle(pt(30.0, 0.0), 5.0).unwrap();
    let ellipse = base.add_ellipse(pt(60.0, 0.0), pt(66.0, 0.0), 3.0).unwrap();
    let spline = base
        .add_spline(
            3,
            &[pt(90.0, 0.0), pt(93.0, 6.0), pt(97.0, -6.0), pt(100.0, 0.0)],
            None,
            None,
        )
        .unwrap();
    base.add_line(pt(0.0, -5.0), pt(10.0, 5.0)).unwrap();
    let anchor = base.add_point(pt(20.0, 12.0)).unwrap();
    let from = pt(20.0, 12.0);
    let probes = [
        pt(5.0, 0.2),
        pt(5.0, 5.0),
        pt(10.2, 0.1),
        pt(30.2, 5.1),
        pt(33.0, 4.0),
        pt(26.0, 3.5),
        pt(60.0, 3.1),
        pt(64.5, 2.1),
        pt(20.0, 0.1),
        pt(95.0, 0.5),
        pt(5.0, 0.1),
        pt(7.5, 2.6),
        pt(30.1, 0.1),
    ];
    let mut kinds = std::collections::BTreeSet::new();
    let mut count = 0;
    for p in probes {
        let cands = snap_candidates(&base, p, 3.0, Some(from));
        for c in cands {
            let mut s = base.clone();
            let np = s.add_point(c.point).unwrap();
            let nl = s.add_line_between(anchor, np).unwrap();
            let ks = c.constraints(np, nl);
            assert_eq!(
                ks.len(),
                usize::from(c.suggested.is_some()) + usize::from(c.secondary.is_some())
            );
            for k in ks {
                assert!(!k.entities().contains(&NEW_ENTITY));
                s.add_constraint(k.clone())
                    .unwrap_or_else(|e| panic!("{:?} {:?}: {e}", c.kind, k));
                count += 1;
            }
            kinds.insert(format!("{:?}", c.kind));
            let mut solver = NativeSolver::new();
            let rep = solver.solve(&mut s, &SolveOptions::fast(), &|| false);
            assert!(
                rep.status.is_converged(),
                "{:?} {:?}: {:?}",
                c.kind,
                c.point,
                rep
            );
        }
    }
    let _ = (line, other, circle, ellipse, spline);
    assert!(count > 20, "{count}");
    for k in [
        "Endpoint",
        "Midpoint",
        "Quadrant",
        "OnCurve",
        "Intersection",
        "Tangent",
        "Perpendicular",
        "AlignHorizontal",
        "AlignVertical",
        "Center",
    ] {
        assert!(kinds.contains(k), "never exercised: {k} ({kinds:?})");
    }
}

// ---- redundancy -----------------------------------------------------------------------------

/// A fully constrained rectangle `w × h` from the origin; returns its corner points.
fn fixed_rectangle(s: &mut Sketch, w: f64, h: f64) -> [EntityId; 4] {
    let p: Vec<EntityId> = [(0.0, 0.0), (w, 0.0), (w, h), (0.0, h)]
        .iter()
        .map(|&(x, y)| s.add_point(pt(x, y)).unwrap())
        .collect();
    let l: Vec<EntityId> = (0..4)
        .map(|i| s.add_line_between(p[i], p[(i + 1) % 4]).unwrap())
        .collect();
    s.add_constraint(K::Horizontal(HvTarget::Line(l[0])))
        .unwrap();
    s.add_constraint(K::Vertical(HvTarget::Line(l[1]))).unwrap();
    s.add_constraint(K::Horizontal(HvTarget::Line(l[2])))
        .unwrap();
    s.add_constraint(K::Vertical(HvTarget::Line(l[3]))).unwrap();
    s.add_constraint(K::Fix { entity: p[0] }).unwrap();
    dim(s, DimensionKind::Length { line: l[0] }, w);
    dim(s, DimensionKind::Length { line: l[1] }, h);
    [p[0], p[1], p[2], p[3]]
}

#[test]
fn redundant_suggestions_are_suppressed_on_a_fully_constrained_rectangle() {
    let mut s = Sketch::new(Plane::XY);
    let [p0, p1, p2, p3] = fixed_rectangle(&mut s, 20.0, 10.0);
    let mut solver = NativeSolver::new();
    let (dof, _) = solver.analyze(&s);
    assert_eq!(dof.dof, 0, "the rectangle is fully constrained");

    // The corner (20, 10) is being dragged; the cursor is level with (0, 10) and above (20, 0).
    let opts = SnapOptions {
        moving: Some(p2),
        ..SnapOptions::default()
    };
    let cands = snap_candidates_with(&s, pt(25.0, 10.2), R, &opts);
    let h = of_kind(&cands, SnapKind::AlignHorizontal);
    assert_eq!(h.len(), 1, "{cands:?}");
    assert_eq!(h[0].entities.as_slice(), &[p3]);
    assert_eq!(
        h[0].suggested,
        Some(K::Horizontal(HvTarget::Points(p3, p2))),
        "the unfiltered suggestion is there"
    );
    let checked = suppress_redundant(&s, &mut solver, cands.clone());
    let h = of_kind(&checked, SnapKind::AlignHorizontal);
    assert_eq!(h.len(), 1, "the position stays a valid snap");
    assert_eq!(h[0].suggested, None, "redundant: p3 and p2 already share y");

    // Vertical alignment with the corner below it is redundant as well.
    let cands = snap_candidates_with(&s, pt(20.2, 14.0), R, &opts);
    let v = of_kind(&cands, SnapKind::AlignVertical);
    assert_eq!(v.len(), 1);
    assert_eq!(v[0].entities.as_slice(), &[p1]);
    let checked = snap_candidates_checked(&s, &mut solver, pt(20.2, 14.0), R, &opts);
    assert!(
        of_kind(&checked, SnapKind::AlignVertical)[0]
            .suggested
            .is_none()
    );

    // A free point is not constrained by the rectangle: the same alignments are kept.
    let free = s.add_point(pt(40.0, 25.0)).unwrap();
    let opts = SnapOptions {
        moving: Some(free),
        ..SnapOptions::default()
    };
    let checked = snap_candidates_checked(&s, &mut solver, pt(30.0, 10.1), R, &opts);
    let h = of_kind(&checked, SnapKind::AlignHorizontal);
    assert!(!h.is_empty(), "{checked:?}");
    assert!(h.iter().all(|c| c.suggested.is_some()), "{h:?}");

    // Coincidence of the fixed corner with another corner contradicts the dimensions: the
    // candidate is dropped.
    let opts = SnapOptions {
        moving: Some(p2),
        ..SnapOptions::default()
    };
    let cands = snap_candidates_with(&s, pt(0.2, 0.1), R, &opts);
    assert!(
        of_kind(&cands, SnapKind::Endpoint)
            .iter()
            .any(|c| c.entities[0] == p0)
    );
    let checked = suppress_redundant(&s, &mut solver, cands);
    assert!(
        of_kind(&checked, SnapKind::Endpoint)
            .iter()
            .all(|c| c.entities[0] != p0),
        "{checked:?}"
    );

    // Placeholder suggestions are never checked (a new entity cannot be redundant).
    let cands = snap_candidates(&s, pt(25.0, 10.2), R, None);
    let n = cands.len();
    let kept = suppress_redundant(&s, &mut solver, cands.clone());
    assert_eq!(kept.len(), n);
    assert!(
        kept.iter()
            .zip(&cands)
            .all(|(a, b)| a.suggested == b.suggested)
    );
}

#[test]
fn excluded_entities_are_not_snap_targets() {
    let mut s = Sketch::new(Plane::XY);
    let l = s.add_line(pt(0.0, 0.0), pt(10.0, 0.0)).unwrap();
    let m = s.add_point(pt(5.0, 0.0)).unwrap();
    let all = snap_candidates(&s, pt(5.0, 0.2), R, None);
    assert!(all.iter().any(|c| c.entities.as_slice() == [m]));
    let opts = SnapOptions {
        moving: Some(m),
        exclude: &[l],
        ..SnapOptions::default()
    };
    let none = snap_candidates_with(&s, pt(5.0, 0.2), R, &opts);
    assert!(
        none.iter()
            .all(|c| !c.entities.contains(&m) && !c.entities.contains(&l))
    );
}

#[test]
fn index_refresh_follows_the_sketch() {
    let mut s = Sketch::new(Plane::XY);
    let a = s.add_point(pt(0.0, 0.0)).unwrap();
    let mut ix = QueryIndex::new(&s);
    assert!(ix.matches(&s));
    assert_eq!(ix.len(), 1);
    let (x, _y) = match s.entity(a).unwrap().kind {
        amg_sketch::EntityKind::Point { x, y } => (x, y),
        _ => panic!(),
    };
    s.params_mut().set(x, 5.0);
    assert!(!ix.matches(&s));
    // The free functions see the new position.
    assert!(
        pick(&s, pt(5.0, 0.0), 0.1, PickFilter::ALL)
            .iter()
            .any(|h| h.entity == a)
    );
    assert!(pick(&s, pt(0.0, 0.0), 0.1, PickFilter::ALL).is_empty());
    ix.refresh(&s);
    assert!(ix.matches(&s));
    assert_eq!(ix.pick(pt(5.0, 0.0), 0.1, PickFilter::ALL).len(), 1);
    assert!(!ix.is_empty());
}

// ---- timing ---------------------------------------------------------------------------------

/// About 520 entities: rectangles, circles, arcs, ellipses and splines on a grid.
fn big_sketch() -> Sketch {
    let mut s = Sketch::new(Plane::XY);
    let mut cell = 0usize;
    let mut at = || {
        let c = cell;
        cell += 1;
        pt((c % 20) as f64 * 40.0, (c / 20) as f64 * 40.0)
    };
    for _ in 0..40 {
        let o = at();
        let p: Vec<EntityId> = [(0.0, 0.0), (20.0, 0.0), (20.0, 10.0), (0.0, 10.0)]
            .iter()
            .map(|&(x, y)| s.add_point(o + pt(x, y)).unwrap())
            .collect();
        for i in 0..4 {
            s.add_line_between(p[i], p[(i + 1) % 4]).unwrap();
        }
    }
    for _ in 0..20 {
        s.add_circle(at() + pt(10.0, 10.0), 8.0).unwrap();
    }
    for _ in 0..20 {
        let o = at() + pt(10.0, 10.0);
        s.add_arc(o, o + pt(8.0, 0.0), o + pt(0.0, 8.0)).unwrap();
    }
    for _ in 0..10 {
        let o = at() + pt(15.0, 15.0);
        s.add_ellipse(o, o + pt(10.0, 0.0), 5.0).unwrap();
    }
    for _ in 0..10 {
        let o = at();
        s.add_spline(
            3,
            &[o, o + pt(5.0, 20.0), o + pt(25.0, -5.0), o + pt(30.0, 20.0)],
            None,
            None,
        )
        .unwrap();
    }
    assert!(s.entities().count() >= 500);
    s
}

#[test]
fn queries_on_a_large_sketch_find_the_right_things() {
    let s = big_sketch();
    let hits = pick(&s, pt(10.0, 0.2), R, PickFilter::ALL);
    assert!(!hits.is_empty());
    let c = snap_candidates(&s, pt(0.2, 0.2), R, Some(pt(60.0, 60.0)));
    assert_eq!(c[0].kind, SnapKind::Endpoint);
    let _ = FRAC_PI_2;
}

#[test]
fn timing_pick_and_snap_on_500_entities() {
    let strict = std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1");
    let bound_us = if strict { 100.0 } else { 1000.0 };
    let s = big_sketch();
    let ix = QueryIndex::new(&s);
    let probes: Vec<Point2> = (0..400)
        .map(|i| {
            let k = f64::from(i);
            pt((k * 7.31) % 800.0, (k * 3.17) % 400.0)
        })
        .collect();
    let opts = SnapOptions {
        from: Some(pt(100.0, 100.0)),
        ..SnapOptions::default()
    };
    // Median of per-query times over a few passes; both through a held index and through the
    // free functions' cache.
    let measure = |f: &dyn Fn(Point2)| {
        let mut per = Vec::new();
        for _ in 0..5 {
            let t = Instant::now();
            for &p in &probes {
                f(p);
            }
            per.push(t.elapsed().as_secs_f64() * 1e6 / probes.len() as f64);
        }
        per.sort_by(f64::total_cmp);
        per[per.len() / 2]
    };
    let held = measure(&|p| {
        std::hint::black_box(ix.pick(p, 2.0, PickFilter::ALL));
        std::hint::black_box(ix.snap_candidates(p, 2.0, &opts));
    });
    let cached = measure(&|p| {
        std::hint::black_box(pick(&s, p, 2.0, PickFilter::ALL));
        std::hint::black_box(snap_candidates_with(&s, p, 2.0, &opts));
    });
    println!("pick + snap per query: held index {held:.1} us, cached free fns {cached:.1} us");
    assert!(held < bound_us, "held index {held:.1} us");
    assert!(cached < bound_us, "cached {cached:.1} us");
}
