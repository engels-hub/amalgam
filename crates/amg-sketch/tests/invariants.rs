//! SK-01 invariants from the review of 2026-10-20: projected points survive removals,
//! endpoint tangency names its ends, non-finite input is rejected, degenerate arcs, locked
//! projected points, and retired parameter slots.

#![allow(
    clippy::unwrap_used,
    reason = "test helpers; a failure is a test failure"
)]

use amg_core::{Plane, Point2};
use amg_sketch::{
    ConstraintKind, Dimension, DimensionKind, End, EntityId, EntityKind, EntityOrigin,
    ExternalLinkId, ParamFlags, Sketch, SketchError, TangentKind,
};

fn children(s: &Sketch, id: EntityId) -> Vec<EntityId> {
    s.entity(id).unwrap().kind.children().to_vec()
}

fn projected_point(s: &mut Sketch, p: Point2, link: u32) -> EntityId {
    let x = s.alloc_param(p.x, ParamFlags::default()).unwrap();
    let y = s.alloc_param(p.y, ParamFlags::default()).unwrap();
    s.add_entity(
        EntityKind::Point { x, y },
        false,
        EntityOrigin::Projected(ExternalLinkId(link)),
    )
    .unwrap()
}

#[test]
fn removing_a_curve_keeps_projected_points() {
    let mut s = Sketch::new(Plane::XY);
    let origin = projected_point(&mut s, Point2::ZERO, 0);
    let q = s.add_point(Point2::new(10.0, 0.0)).unwrap();
    let l = s.add_line_between(origin, q).unwrap();
    let report = s.remove_entities(&[l]).unwrap();
    // The user end point is swept, the projected origin stays.
    assert_eq!(report.entities, vec![q, l]);
    assert!(s.entity(origin).is_some());
    // Removing the projected point explicitly still works and takes dependents along.
    let radius = s.alloc_param(1.0, ParamFlags::default()).unwrap();
    let c = s
        .add_entity(
            EntityKind::Circle {
                center: origin,
                radius,
            },
            false,
            EntityOrigin::User,
        )
        .unwrap();
    assert_eq!(
        s.remove_entities(&[origin]).unwrap().entities,
        vec![origin, c]
    );
}

fn tangent_at(a: EntityId, a_end: End, b: EntityId, b_end: End) -> ConstraintKind {
    ConstraintKind::Tangent {
        a,
        b,
        kind: TangentKind::AtEndpoint {
            a_end,
            b_end,
            reversed: false,
        },
    }
}

#[test]
fn tangent_at_endpoint_needs_joined_ends() {
    let mut s = Sketch::new(Plane::XY);
    // Disjoint line and arc.
    let l = s
        .add_line(Point2::new(100.0, 0.0), Point2::new(110.0, 0.0))
        .unwrap();
    let a = s
        .add_arc(
            Point2::new(-50.0, -50.0),
            Point2::new(-40.0, -50.0),
            Point2::new(-50.0, -40.0),
        )
        .unwrap();
    for (ae, be) in [(End::Start, End::Start), (End::End, End::End)] {
        assert!(matches!(
            s.add_constraint(tangent_at(l, ae, a, be)),
            Err(SketchError::NotApplicable(_))
        ));
    }

    // Lens: two arcs sharing both end points (as point entities).
    let c1 = s.add_point(Point2::new(0.0, 0.0)).unwrap();
    let c2 = s.add_point(Point2::new(0.0, 10.0)).unwrap();
    let p1 = s.add_point(Point2::new(-5.0, 5.0)).unwrap();
    let p2 = s.add_point(Point2::new(5.0, 5.0)).unwrap();
    let arc = |s: &mut Sketch, center, start, end| {
        s.add_entity(
            EntityKind::Arc { center, start, end },
            false,
            EntityOrigin::User,
        )
        .unwrap()
    };
    let arc1 = arc(&mut s, c1, p2, p1);
    let arc2 = arc(&mut s, c2, p1, p2);
    // arc1 ends at p1 and arc2 starts there: unambiguous, and stored as given.
    let at_p1 = s
        .add_constraint(tangent_at(arc1, End::End, arc2, End::Start))
        .unwrap();
    assert_eq!(
        s.constraint(at_p1).unwrap().kind,
        tangent_at(arc1, End::End, arc2, End::Start)
    );
    s.add_constraint(tangent_at(arc1, End::Start, arc2, End::End))
        .unwrap();
    // Mismatched ends (p1 vs p2) are rejected.
    assert!(matches!(
        s.add_constraint(tangent_at(arc1, End::End, arc2, End::End)),
        Err(SketchError::NotApplicable(_))
    ));

    // Joined through a chain of coincident constraints: l.end ~ m ~ a.start.
    let l_end = children(&s, l)[1];
    let a_start = children(&s, a)[1];
    let m = s.add_point(Point2::ZERO).unwrap();
    s.add_constraint(ConstraintKind::Coincident { a: l_end, b: m })
        .unwrap();
    assert!(
        s.add_constraint(tangent_at(l, End::End, a, End::Start))
            .is_err()
    );
    let join = s
        .add_constraint(ConstraintKind::Coincident { a: a_start, b: m })
        .unwrap();
    s.add_constraint(tangent_at(l, End::End, a, End::Start))
        .unwrap();
    // Removing the joining coincident keeps the tangent, and the sketch still loads.
    s.remove_constraint(join).unwrap();
    let json = serde_json::to_string(&s).unwrap();
    assert!(serde_json::from_str::<Sketch>(&json).is_ok());
}

#[test]
fn non_finite_input_is_rejected_in_all_builds() {
    let mut s = Sketch::new(Plane::XY);
    let nan = Point2::new(f64::NAN, 0.0);
    let inf = Point2::new(0.0, f64::INFINITY);
    assert!(matches!(
        s.add_point(nan),
        Err(SketchError::InvalidValue { .. })
    ));
    assert!(matches!(
        s.add_line(Point2::ZERO, inf),
        Err(SketchError::InvalidValue { .. })
    ));
    assert!(matches!(
        s.add_arc(Point2::ZERO, Point2::X, nan),
        Err(SketchError::InvalidValue { .. })
    ));
    assert!(matches!(
        s.alloc_param(f64::NEG_INFINITY, ParamFlags::default()),
        Err(SketchError::InvalidValue { .. })
    ));
    assert_eq!(s.entities().count(), 0);
    assert_eq!(s.params().len(), 0);

    // A non-finite value written later (e.g. by a diverged solve) makes saving fail instead of
    // writing a file that cannot be loaded.
    let p = s.add_point(Point2::ZERO).unwrap();
    let x = s.entity(p).unwrap().kind.own_params()[0];
    s.params_mut().set(x, f64::NAN);
    let err = serde_json::to_string(&s).unwrap_err().to_string();
    assert!(err.contains("not finite"), "{err}");

    // Dimension label positions must be finite too.
    let l = s.add_line(Point2::ZERO, Point2::X).unwrap();
    let k = ConstraintKind::Dimension(Dimension {
        kind: DimensionKind::Length { line: l },
        value: 1.0,
        driving: true,
        label_pos: Point2::new(0.0, f64::NAN),
    });
    assert!(matches!(
        s.add_constraint(k),
        Err(SketchError::InvalidValue { .. })
    ));
}

#[test]
fn add_arc_rejects_coinciding_directions() {
    let mut s = Sketch::new(Plane::XY);
    for end in [
        Point2::new(2.0, 1e-17),
        Point2::new(2.0, 0.0),
        Point2::new(3.0, -1e-12),
    ] {
        assert!(matches!(
            s.add_arc(Point2::ZERO, Point2::X, end),
            Err(SketchError::Degenerate(_))
        ));
    }
    // Clearly outside the angular tolerance is fine.
    assert!(
        s.add_arc(Point2::ZERO, Point2::X, Point2::new(1.0, 1e-6))
            .is_ok()
    );
}

#[test]
fn add_entity_arc_radius_must_match() {
    let mut s = Sketch::new(Plane::XY);
    let c = s.add_point(Point2::ZERO).unwrap();
    let st = s.add_point(Point2::new(1.0, 0.0)).unwrap();
    let far = s.add_point(Point2::new(0.0, 5.0)).unwrap();
    let on = s.add_point(Point2::new(0.0, 1.0)).unwrap();
    let at_c = s.add_point(Point2::ZERO).unwrap();
    let mk = |s: &mut Sketch, start, end| {
        s.add_entity(
            EntityKind::Arc {
                center: c,
                start,
                end,
            },
            false,
            EntityOrigin::User,
        )
    };
    assert!(matches!(
        mk(&mut s, st, far),
        Err(SketchError::Degenerate(_))
    ));
    assert!(matches!(
        mk(&mut s, at_c, on),
        Err(SketchError::Degenerate(_))
    ));
    assert!(mk(&mut s, st, on).is_ok());
}

#[test]
fn projected_curves_lock_their_points() {
    let mut s = Sketch::new(Plane::XY);
    let a = s.add_point(Point2::ZERO).unwrap();
    let b = s.add_point(Point2::new(1.0, 0.0)).unwrap();
    let r = s.alloc_param(2.0, ParamFlags::default()).unwrap();
    s.add_entity(
        EntityKind::Line { start: a, end: b },
        false,
        EntityOrigin::Projected(ExternalLinkId(1)),
    )
    .unwrap();
    let c = s
        .add_entity(
            EntityKind::Circle {
                center: b,
                radius: r,
            },
            false,
            EntityOrigin::Projected(ExternalLinkId(2)),
        )
        .unwrap();
    for e in [a, b, c] {
        for p in s.entity(e).unwrap().kind.own_params() {
            assert!(s.params().flags(p).locked, "{e:?}");
        }
    }
}

#[test]
fn dead_param_slots_are_never_reused() {
    let mut s = Sketch::new(Plane::XY);
    let c = s.add_circle(Point2::ZERO, 2.0).unwrap();
    let EntityKind::Circle { radius, .. } = s.entity(c).unwrap().kind else {
        unreachable!()
    };
    s.remove_entities(&[c]).unwrap();
    let ctr = s.add_point(Point2::ZERO).unwrap();
    let reuse = |s: &mut Sketch, center| {
        s.add_entity(
            EntityKind::Circle { center, radius },
            false,
            EntityOrigin::User,
        )
    };
    assert!(matches!(
        reuse(&mut s, ctr),
        Err(SketchError::NotApplicable(_))
    ));
    // 3 retired slots (centre x/y, radius) + 2 live ones; compact drops the retired ones.
    assert_eq!(s.params().len(), 5);
    s.compact();
    assert_eq!(s.params().len(), 2);

    // The retired set survives a save/load round trip.
    let mut t = Sketch::new(Plane::XY);
    let c = t.add_circle(Point2::ZERO, 2.0).unwrap();
    t.remove_entities(&[c]).unwrap();
    let mut back: Sketch = serde_json::from_str(&serde_json::to_string(&t).unwrap()).unwrap();
    let ctr = back.add_point(Point2::ZERO).unwrap();
    assert!(reuse(&mut back, ctr).is_err());
}
