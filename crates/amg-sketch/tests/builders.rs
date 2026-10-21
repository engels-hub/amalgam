//! SK-21 acceptance: every composite builder produces zero residual, the expected constraint
//! set and DOF (SK-02 rank helper), and `Degenerate` on degenerate input.

#![allow(
    clippy::unwrap_used,
    clippy::float_cmp,
    reason = "test helpers; a failure is a test failure"
)]

use std::collections::BTreeMap;

use amg_core::{Plane, Point2};
use amg_sketch::build::{self, Built};
use amg_sketch::solve::{build_system, dof};
use amg_sketch::{ConstraintKind, End, EntityKind, Sketch, SketchError};

fn new() -> Sketch {
    Sketch::new(Plane::XY)
}

fn p(x: f64, y: f64) -> Point2 {
    Point2::new(x, y)
}

/// Largest residual over the whole sketch.
fn residual(s: &Sketch) -> f64 {
    let sys = build_system(s).unwrap();
    sys.system.max_abs_residual(s.params().as_slice())
}

/// Counts constraints by name.
fn counts(s: &Sketch, b: &Built) -> BTreeMap<&'static str, usize> {
    let mut m = BTreeMap::new();
    for id in &b.constraints {
        *m.entry(s.constraint(*id).unwrap().kind.name()).or_insert(0) += 1;
    }
    m
}

fn count_kind(s: &Sketch, b: &Built, f: impl Fn(&EntityKind) -> bool) -> usize {
    b.entities
        .iter()
        .filter(|id| f(&s.entity(**id).unwrap().kind))
        .count()
}

fn check(s: &Sketch, b: &Built, dof_expected: u32) {
    assert!(residual(s) < 1e-9, "residual {}", residual(s));
    assert_eq!(dof(s).unwrap(), dof_expected);
    // Every reported id exists.
    for e in &b.entities {
        assert!(s.entity(*e).is_some());
    }
    for c in &b.constraints {
        assert!(s.constraint(*c).is_some());
    }
}

fn map(v: &[(&'static str, usize)]) -> BTreeMap<&'static str, usize> {
    v.iter().copied().collect()
}

#[test]
fn rectangle_2pt_dof4() {
    let mut s = new();
    let b = build::rectangle_2pt(&mut s, p(1.0, 2.0), p(7.0, 5.0)).unwrap();
    check(&s, &b, 4);
    assert_eq!(counts(&s, &b), map(&[("horizontal", 2), ("vertical", 2)]));
    assert_eq!(
        count_kind(&s, &b, |k| matches!(k, EntityKind::Line { .. })),
        4
    );
    assert_eq!(
        count_kind(&s, &b, |k| matches!(k, EntityKind::Point { .. })),
        4
    );
    // Mirrored corners work as well.
    let b = build::rectangle_2pt(&mut s, p(10.0, 10.0), p(8.0, 4.0)).unwrap();
    assert_eq!(counts(&s, &b), map(&[("horizontal", 2), ("vertical", 2)]));
    assert_eq!(dof(&s).unwrap(), 8);
}

#[test]
fn rectangle_3pt_dof5() {
    for c in [p(2.0, 5.0), p(2.0, -5.0)] {
        let mut s = new();
        let b = build::rectangle_3pt(&mut s, p(0.0, 0.0), p(4.0, 3.0), c).unwrap();
        check(&s, &b, 5);
        assert_eq!(counts(&s, &b), map(&[("perpendicular", 3)]));
        assert_eq!(
            count_kind(&s, &b, |k| matches!(k, EntityKind::Line { .. })),
            4
        );
    }
}

#[test]
fn rectangle_center_dof4() {
    let mut s = new();
    let b = build::rectangle_center(&mut s, p(3.0, 4.0), p(5.0, 1.0)).unwrap();
    check(&s, &b, 4);
    assert_eq!(
        counts(&s, &b),
        map(&[("horizontal", 2), ("vertical", 2), ("midpoint", 1)])
    );
    assert_eq!(
        count_kind(&s, &b, |k| matches!(k, EntityKind::Line { .. })),
        5
    );
    let construction = b
        .entities
        .iter()
        .filter(|id| s.entity(**id).unwrap().construction)
        .count();
    assert_eq!(construction, 1);
}

#[test]
fn polygons_dof4() {
    for inscribed in [true, false] {
        for n in [3u32, 4, 5, 6, 8, 12] {
            let mut s = new();
            let b = build::polygon(&mut s, p(1.0, -2.0), p(4.0, -1.0), n, inscribed).unwrap();
            assert!(residual(&s) < 1e-9, "residual n={n} inscribed={inscribed}");
            assert_eq!(dof(&s).unwrap(), 4, "dof n={n} inscribed={inscribed}");
            let c = counts(&s, &b);
            assert_eq!(c["equal"], n as usize - 1);
            if inscribed {
                assert_eq!(c["point on curve"], n as usize);
            } else {
                assert_eq!(c["tangent"], n as usize);
            }
            let sides = b
                .entities
                .iter()
                .filter(|id| {
                    let e = s.entity(**id).unwrap();
                    matches!(e.kind, EntityKind::Line { .. }) && !e.construction
                })
                .count();
            assert_eq!(sides, n as usize);
        }
    }
}

#[test]
fn polygon_vertex_semantics() {
    let mut s = new();
    // Inscribed: the first vertex is the given point.
    let b = build::polygon(&mut s, p(0.0, 0.0), p(3.0, 0.0), 6, true).unwrap();
    let first = b
        .entities
        .iter()
        .find_map(|id| s.point(*id).filter(|q| (q.x - 3.0).abs() < 1e-12));
    assert!(first.is_some());
    // Circumscribed: the given point is the midpoint of a side.
    let mut s = new();
    let b = build::polygon(&mut s, p(0.0, 0.0), p(2.0, 0.0), 4, false).unwrap();
    let line = b
        .entities
        .iter()
        .find_map(|id| match s.entity(*id).unwrap().kind {
            EntityKind::Line { start, end } => {
                Some((s.point(start).unwrap(), s.point(end).unwrap()))
            }
            _ => None,
        })
        .unwrap();
    let mid = (line.0 + line.1) / 2.0;
    assert!(mid.distance(p(2.0, 0.0)) < 1e-12);
}

fn slot_checks(s: &Sketch, b: &Built, extra: &[(&'static str, usize)]) {
    check(s, b, 5);
    let mut expect = map(&[("tangent", 4), ("equal", 1)]);
    for (k, v) in extra {
        expect.insert(k, *v);
    }
    assert_eq!(counts(s, b), expect);
    assert_eq!(count_kind(s, b, |k| matches!(k, EntityKind::Arc { .. })), 2);
    // The four tangencies are endpoint tangencies.
    for id in &b.constraints {
        if let ConstraintKind::Tangent { kind, .. } = &s.constraint(*id).unwrap().kind {
            assert!(matches!(kind, amg_sketch::TangentKind::AtEndpoint { .. }));
        }
    }
}

#[test]
fn slots_dof5() {
    let mut s = new();
    let b = build::slot_center_to_center(&mut s, p(0.0, 0.0), p(10.0, 4.0), 3.0).unwrap();
    slot_checks(&s, &b, &[]);

    let mut s = new();
    let b = build::slot_center_to_center(&mut s, p(10.0, 4.0), p(0.0, 0.0), 3.0).unwrap();
    slot_checks(&s, &b, &[]);

    let mut s = new();
    let b = build::slot_overall(&mut s, p(0.0, 0.0), p(12.0, 0.0), 4.0).unwrap();
    slot_checks(&s, &b, &[]);
    // Overall length is respected: leftmost point of the left arc is the first input.
    let min_x = b
        .entities
        .iter()
        .filter_map(|id| s.point(*id))
        .map(|q| q.x)
        .fold(f64::INFINITY, f64::min);
    assert!((min_x - 2.0).abs() < 1e-12, "arc centre at {min_x}");

    let mut s = new();
    let b = build::slot_center_point(&mut s, p(5.0, 5.0), p(9.0, 6.0), 2.0).unwrap();
    slot_checks(&s, &b, &[("midpoint", 1)]);
}

#[test]
fn slot_arc_dof6() {
    let mut s = new();
    let b = build::slot_arc(&mut s, p(0.0, 0.0), p(10.0, 0.0), p(0.0, 10.0), 3.0).unwrap();
    check(&s, &b, 6);
    assert_eq!(counts(&s, &b), map(&[("tangent", 4)]));
    assert_eq!(
        count_kind(&s, &b, |k| matches!(k, EntityKind::Arc { .. })),
        5
    );
    // Wide sweep (> pi).
    let mut s = new();
    let b = build::slot_arc(&mut s, p(1.0, 1.0), p(1.0, 11.0), p(-8.0, 2.0), 4.0).unwrap();
    check(&s, &b, 6);
    let _ = b;
}

#[test]
fn circle_and_arc_3pt() {
    let mut s = new();
    let b = build::circle_3pt(&mut s, p(1.0, 0.0), p(0.0, 1.0), p(-1.0, 0.0)).unwrap();
    check(&s, &b, 3);
    assert!(b.constraints.is_empty());
    let circle = b
        .entities
        .iter()
        .find_map(|id| match s.entity(*id).unwrap().kind {
            EntityKind::Circle { center, radius } => {
                Some((s.point(center).unwrap(), s.params().get(radius)))
            }
            _ => None,
        })
        .unwrap();
    assert!(circle.0.length() < 1e-12 && (circle.1 - 1.0).abs() < 1e-12);

    for (a, t, c) in [
        (p(1.0, 0.0), p(0.0, 1.0), p(-1.0, 0.0)),
        (p(-1.0, 0.0), p(0.0, 1.0), p(1.0, 0.0)),
    ] {
        let mut s = new();
        let b = build::arc_3pt(&mut s, a, t, c).unwrap();
        check(&s, &b, 5);
        assert!(b.constraints.is_empty());
        // The mid point lies on the arc (it sweeps CCW through `t`).
        let EntityKind::Arc { center, start, end } = s.entity(b.entities[3]).unwrap().kind else {
            panic!("expected an arc")
        };
        let (cc, sp, ep) = (
            s.point(center).unwrap(),
            s.point(start).unwrap(),
            s.point(end).unwrap(),
        );
        let ang = |q: Point2| (q - cc).y.atan2((q - cc).x);
        let rel = |x: f64, from: f64| (x - from).rem_euclid(std::f64::consts::TAU);
        assert!(rel(ang(t), ang(sp)) < rel(ang(ep), ang(sp)));
    }
}

#[test]
fn circle_tangent_to_two_lines() {
    let mut s = new();
    let a = s.add_line(p(0.0, 0.0), p(10.0, 0.0)).unwrap();
    let b = s.add_line(p(0.0, 0.0), p(0.0, 10.0)).unwrap();
    let base = dof(&s).unwrap();
    let built = build::circle_2_tangent(&mut s, a, b, 2.0, p(3.0, 3.0)).unwrap();
    assert!(residual(&s) < 1e-9);
    assert_eq!(dof(&s).unwrap(), base);
    assert_eq!(counts(&s, &built), map(&[("tangent", 2), ("radius", 1)]));
    let c = built.entities.iter().find_map(|id| s.point(*id)).unwrap();
    assert!(c.distance(p(2.0, 2.0)) < 1e-12);
    // `near` selects the other quadrant.
    let built = build::circle_2_tangent(&mut s, a, b, 2.0, p(-5.0, 1.0)).unwrap();
    let c = built.entities.iter().find_map(|id| s.point(*id)).unwrap();
    assert!(c.distance(p(-2.0, 2.0)) < 1e-12);
    assert!(residual(&s) < 1e-9);
}

#[test]
fn circle_tangent_to_line_and_circle() {
    let mut s = new();
    let line = s.add_line(p(-20.0, 0.0), p(20.0, 0.0)).unwrap();
    let circ = s.add_circle(p(0.0, 4.0), 3.0).unwrap();
    let base = dof(&s).unwrap();
    let b = build::circle_2_tangent(&mut s, line, circ, 1.0, p(5.0, 1.0)).unwrap();
    assert!(residual(&s) < 1e-9);
    assert_eq!(dof(&s).unwrap(), base);
    assert_eq!(b.constraints.len(), 3);
    // Two circles, internal tangency to a larger one.
    let mut s = new();
    let big = s.add_circle(p(0.0, 0.0), 10.0).unwrap();
    let small = s.add_circle(p(12.0, 0.0), 2.0).unwrap();
    let base = dof(&s).unwrap();
    build::circle_2_tangent(&mut s, big, small, 3.0, p(7.0, 5.0)).unwrap();
    assert!(residual(&s) < 1e-9);
    assert_eq!(dof(&s).unwrap(), base);
}

#[test]
fn arc_tangent_to_line_and_arc() {
    for (at, end) in [
        (End::End, p(14.0, 4.0)),
        (End::End, p(14.0, -4.0)),
        (End::Start, p(-4.0, 3.0)),
    ] {
        let mut s = new();
        let l = s.add_line(p(0.0, 0.0), p(10.0, 0.0)).unwrap();
        let base = dof(&s).unwrap();
        let b = build::arc_tangent(&mut s, l, at, end).unwrap();
        assert!(residual(&s) < 1e-9, "line {at:?} {end}");
        assert_eq!(dof(&s).unwrap(), base + 2);
        assert_eq!(counts(&s, &b), map(&[("tangent", 1)]));
        assert_eq!(b.entities.len(), 3);
    }
    for at in [End::Start, End::End] {
        for end in [p(2.0, 9.0), p(-6.0, -6.0), p(9.0, -9.0)] {
            let mut s = new();
            let a = s.add_arc(p(0.0, 0.0), p(5.0, 0.0), p(0.0, 5.0)).unwrap();
            let base = dof(&s).unwrap();
            build::arc_tangent(&mut s, a, at, end).unwrap();
            assert!(residual(&s) < 1e-9, "arc {at:?} {end}");
            assert_eq!(dof(&s).unwrap(), base + 2);
        }
    }
}

#[test]
fn degenerate_inputs() {
    let mut s = new();
    let deg = |r: Result<Built, SketchError>| matches!(r, Err(SketchError::Degenerate(_)));
    assert!(deg(build::rectangle_2pt(&mut s, p(0.0, 0.0), p(5.0, 0.0))));
    assert!(deg(build::rectangle_2pt(&mut s, p(1.0, 1.0), p(1.0, 1.0))));
    assert!(deg(build::rectangle_3pt(
        &mut s,
        p(0.0, 0.0),
        p(0.0, 0.0),
        p(1.0, 1.0)
    )));
    assert!(deg(build::rectangle_3pt(
        &mut s,
        p(0.0, 0.0),
        p(4.0, 0.0),
        p(2.0, 0.0)
    )));
    assert!(deg(build::rectangle_center(
        &mut s,
        p(0.0, 0.0),
        p(3.0, 0.0)
    )));
    assert!(deg(build::polygon(
        &mut s,
        p(0.0, 0.0),
        p(1.0, 0.0),
        2,
        true
    )));
    assert!(deg(build::polygon(
        &mut s,
        p(1.0, 1.0),
        p(1.0, 1.0),
        5,
        true
    )));
    assert!(deg(build::slot_center_to_center(
        &mut s,
        p(1.0, 1.0),
        p(1.0, 1.0),
        2.0
    )));
    assert!(deg(build::slot_center_to_center(
        &mut s,
        p(0.0, 0.0),
        p(5.0, 0.0),
        0.0
    )));
    assert!(deg(build::slot_overall(
        &mut s,
        p(0.0, 0.0),
        p(2.0, 0.0),
        2.0
    )));
    assert!(deg(build::slot_overall(
        &mut s,
        p(0.0, 0.0),
        p(1.0, 0.0),
        2.0
    )));
    assert!(deg(build::slot_center_point(
        &mut s,
        p(1.0, 1.0),
        p(1.0, 1.0),
        1.0
    )));
    assert!(deg(build::slot_arc(
        &mut s,
        p(0.0, 0.0),
        p(1.0, 0.0),
        p(0.0, 5.0),
        3.0
    )));
    assert!(deg(build::slot_arc(
        &mut s,
        p(0.0, 0.0),
        p(5.0, 0.0),
        p(5.0, 0.5),
        3.0
    )));
    assert!(deg(build::slot_arc(
        &mut s,
        p(0.0, 0.0),
        p(5.0, 0.0),
        p(0.0, 0.0),
        1.0
    )));
    assert!(deg(build::circle_3pt(
        &mut s,
        p(0.0, 0.0),
        p(1.0, 1.0),
        p(2.0, 2.0)
    )));
    assert!(deg(build::circle_3pt(
        &mut s,
        p(0.0, 0.0),
        p(0.0, 0.0),
        p(2.0, 2.0)
    )));
    assert!(deg(build::arc_3pt(
        &mut s,
        p(0.0, 0.0),
        p(1.0, 0.0),
        p(2.0, 0.0)
    )));
    assert!(deg(build::arc_3pt(
        &mut s,
        p(0.0, 0.0),
        p(1.0, 0.0),
        p(0.0, 0.0)
    )));
    // Nothing was created by any failed call.
    assert_eq!(s.entities().count(), 0);

    let l1 = s.add_line(p(0.0, 0.0), p(10.0, 0.0)).unwrap();
    let l2 = s.add_line(p(0.0, 1.0), p(10.0, 1.0)).unwrap();
    let l3 = s.add_line(p(0.0, 4.0), p(10.0, 4.0)).unwrap();
    // Parallel lines at distance 1 cannot carry a circle of radius 2; parallel lines at
    // distance 2r have infinitely many.
    assert!(deg(build::circle_2_tangent(
        &mut s,
        l1,
        l2,
        2.0,
        p(5.0, 0.5)
    )));
    assert!(deg(build::circle_2_tangent(
        &mut s,
        l1,
        l3,
        2.0,
        p(5.0, 2.0)
    )));
    assert!(deg(build::arc_tangent(&mut s, l1, End::End, p(10.0, 0.0))));
    assert!(deg(build::arc_tangent(&mut s, l1, End::End, p(20.0, 0.0))));
    let n = s.entities().count();
    assert!(matches!(
        build::circle_2_tangent(&mut s, l1, l1, 1.0, p(0.0, 0.0)),
        Err(SketchError::NotApplicable(_))
    ));
    assert!(matches!(
        build::circle_2_tangent(&mut s, l1, l2, -1.0, p(0.0, 0.0)),
        Err(SketchError::InvalidValue { .. })
    ));
    assert!(matches!(
        build::slot_center_to_center(&mut s, p(0.0, 0.0), p(1.0, 0.0), f64::NAN),
        Err(SketchError::InvalidValue { .. })
    ));
    assert!(matches!(
        build::rectangle_2pt(&mut s, p(f64::NAN, 0.0), p(1.0, 1.0)),
        Err(SketchError::InvalidValue { .. })
    ));
    assert_eq!(s.entities().count(), n);
}
