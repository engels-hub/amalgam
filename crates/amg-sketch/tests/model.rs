//! SK-01 data model: creation, kind validation, removal cascade, compaction, revisions.

#![allow(
    clippy::unwrap_used,
    reason = "test helpers; a failure is a test failure"
)]

use amg_core::{Plane, Point2};
use amg_sketch::{
    AngleBranch, ConstraintKind, Curve2, Dimension, DimensionKind, EntityId, EntityKind,
    EntityOrigin, ExternalLinkId, HvTarget, ParamFlags, ParamId, Quantity, Side, Sketch,
    SketchError, TangentKind,
};

/// One entity of every P0 kind (plus spares), all user geometry.
struct Base {
    s: Sketch,
    /// Free point.
    p: EntityId,
    /// Second free point.
    q: EntityId,
    l1: EntityId,
    l2: EntityId,
    l3: EntityId,
    c1: EntityId,
    c2: EntityId,
    a1: EntityId,
    a2: EntityId,
}

fn base() -> Base {
    let mut s = Sketch::new(Plane::XY);
    let p = s.add_point(Point2::new(5.0, 5.0));
    let q = s.add_point(Point2::new(-3.0, 2.0));
    let l1 = s.add_line(Point2::new(0.0, 0.0), Point2::new(10.0, 0.0));
    let l2 = s.add_line(Point2::new(0.0, 4.0), Point2::new(10.0, 6.0));
    let l3 = s.add_line(Point2::new(20.0, -5.0), Point2::new(20.0, 5.0));
    let c1 = s.add_circle(Point2::new(30.0, 0.0), 3.0).unwrap();
    let c2 = s.add_circle(Point2::new(30.0, 10.0), 1.5).unwrap();
    let a1 = s
        .add_arc(
            Point2::new(0.0, 20.0),
            Point2::new(4.0, 20.0),
            Point2::new(0.0, 24.0),
        )
        .unwrap();
    let a2 = s
        .add_arc(
            Point2::new(10.0, 20.0),
            Point2::new(12.0, 20.0),
            Point2::new(8.0, 20.0),
        )
        .unwrap();
    Base {
        s,
        p,
        q,
        l1,
        l2,
        l3,
        c1,
        c2,
        a1,
        a2,
    }
}

fn children(s: &Sketch, id: EntityId) -> Vec<EntityId> {
    s.entity(id).unwrap().kind.children().to_vec()
}

fn dim(kind: DimensionKind, value: f64) -> ConstraintKind {
    ConstraintKind::Dimension(Dimension {
        kind,
        value,
        driving: true,
        label_pos: Point2::new(1.0, 2.0),
    })
}

const BR: AngleBranch = AngleBranch {
    reverse_a: false,
    reverse_b: true,
};

/// Every constraint and dimension variant, in valid form (several forms per variant where the
/// spec lists several entity combinations).
fn valid_constraints(b: &Base) -> Vec<ConstraintKind> {
    use ConstraintKind as C;
    use DimensionKind as D;
    let l1s = children(&b.s, b.l1)[0];
    let a1e = children(&b.s, b.a1)[2];
    vec![
        C::Coincident { a: b.p, b: b.q },
        C::PointOnCurve {
            point: b.p,
            curve: b.l1,
        },
        C::PointOnCurve {
            point: b.p,
            curve: b.c1,
        },
        C::PointOnCurve {
            point: b.p,
            curve: b.a1,
        },
        C::Midpoint {
            point: b.p,
            line: b.l1,
        },
        C::Horizontal(HvTarget::Line(b.l1)),
        C::Horizontal(HvTarget::Points(b.p, b.q)),
        C::Vertical(HvTarget::Line(b.l3)),
        C::Vertical(HvTarget::Points(b.p, l1s)),
        C::Parallel { a: b.l1, b: b.l2 },
        C::Perpendicular { a: b.l1, b: b.l3 },
        C::Collinear { a: b.l1, b: b.l2 },
        C::Tangent {
            a: b.l1,
            b: b.c1,
            kind: TangentKind::External,
        },
        C::Tangent {
            a: b.a1,
            b: b.l2,
            kind: TangentKind::Internal,
        },
        C::Tangent {
            a: b.c1,
            b: b.c2,
            kind: TangentKind::External,
        },
        C::Tangent {
            a: b.c1,
            b: b.a2,
            kind: TangentKind::Internal,
        },
        C::Tangent {
            a: b.l1,
            b: b.a1,
            kind: TangentKind::AtEndpoint { reversed: false },
        },
        C::Tangent {
            a: b.a1,
            b: b.a2,
            kind: TangentKind::AtEndpoint { reversed: true },
        },
        C::Concentric { a: b.c1, b: b.a1 },
        C::Equal { a: b.l1, b: b.l2 },
        C::Equal { a: b.l1, b: b.a1 },
        C::Equal { a: b.c1, b: b.c2 },
        C::Equal { a: b.a1, b: b.c1 },
        C::Symmetric {
            a: b.p,
            b: b.q,
            axis: b.l3,
        },
        C::Symmetric {
            a: b.l1,
            b: b.l2,
            axis: b.l3,
        },
        C::Symmetric {
            a: b.c1,
            b: b.c2,
            axis: b.l3,
        },
        C::Symmetric {
            a: b.a1,
            b: b.a2,
            axis: b.l3,
        },
        C::Fix { entity: b.p },
        C::Fix { entity: b.a1 },
        C::Fix { entity: a1e },
        dim(
            D::Distance {
                a: b.p,
                b: b.q,
                side: Side::Positive,
            },
            3.0,
        ),
        dim(
            D::Distance {
                a: b.p,
                b: b.l1,
                side: Side::Negative,
            },
            3.0,
        ),
        dim(
            D::Distance {
                a: b.l1,
                b: b.l2,
                side: Side::Positive,
            },
            0.0,
        ),
        dim(
            D::Distance {
                a: b.p,
                b: b.c1,
                side: Side::Positive,
            },
            3.0,
        ),
        dim(
            D::Distance {
                a: b.l3,
                b: b.a1,
                side: Side::Positive,
            },
            3.0,
        ),
        dim(
            D::HorizontalDistance {
                a: b.p,
                b: b.q,
                side: Side::Negative,
            },
            8.0,
        ),
        dim(
            D::VerticalDistance {
                a: b.p,
                b: b.q,
                side: Side::Positive,
            },
            3.0,
        ),
        dim(D::Length { line: b.l1 }, 10.0),
        dim(
            D::Angle {
                a: b.l1,
                b: b.l2,
                branch: BR,
            },
            -0.5,
        ),
        dim(D::Radius { curve: b.c1 }, 3.0),
        dim(D::Radius { curve: b.a1 }, 4.0),
        dim(D::Diameter { curve: b.c2 }, 3.0),
        dim(D::ArcLength { arc: b.a1 }, 6.0),
    ]
}

/// One invalid-kind form per variant, with the entity the error must blame.
fn wrong_kind_constraints(b: &Base) -> Vec<(ConstraintKind, EntityId)> {
    use ConstraintKind as C;
    use DimensionKind as D;
    vec![
        (C::Coincident { a: b.p, b: b.l1 }, b.l1),
        (
            C::PointOnCurve {
                point: b.p,
                curve: b.q,
            },
            b.q,
        ),
        (
            C::PointOnCurve {
                point: b.l1,
                curve: b.c1,
            },
            b.l1,
        ),
        (
            C::Midpoint {
                point: b.p,
                line: b.c1,
            },
            b.c1,
        ),
        (C::Horizontal(HvTarget::Line(b.c1)), b.c1),
        (C::Horizontal(HvTarget::Points(b.p, b.l2)), b.l2),
        (C::Vertical(HvTarget::Line(b.a1)), b.a1),
        (C::Vertical(HvTarget::Points(b.c1, b.p)), b.c1),
        (C::Parallel { a: b.l1, b: b.c1 }, b.c1),
        (C::Perpendicular { a: b.p, b: b.l1 }, b.p),
        (C::Collinear { a: b.l1, b: b.a1 }, b.a1),
        (
            C::Tangent {
                a: b.l1,
                b: b.l2,
                kind: TangentKind::External,
            },
            b.l2,
        ),
        (
            C::Tangent {
                a: b.p,
                b: b.c1,
                kind: TangentKind::Internal,
            },
            b.p,
        ),
        (
            C::Tangent {
                a: b.c1,
                b: b.q,
                kind: TangentKind::External,
            },
            b.q,
        ),
        (
            C::Tangent {
                a: b.c1,
                b: b.a1,
                kind: TangentKind::AtEndpoint { reversed: false },
            },
            b.c1,
        ),
        (
            C::Tangent {
                a: b.l1,
                b: b.l2,
                kind: TangentKind::AtEndpoint { reversed: false },
            },
            b.l2,
        ),
        (C::Concentric { a: b.c1, b: b.l1 }, b.l1),
        (C::Equal { a: b.l1, b: b.c1 }, b.c1),
        (C::Equal { a: b.c1, b: b.l1 }, b.l1),
        (C::Equal { a: b.p, b: b.q }, b.p),
        (
            C::Symmetric {
                a: b.p,
                b: b.l1,
                axis: b.l3,
            },
            b.l1,
        ),
        (
            C::Symmetric {
                a: b.c1,
                b: b.a1,
                axis: b.l3,
            },
            b.a1,
        ),
        (
            C::Symmetric {
                a: b.p,
                b: b.q,
                axis: b.c1,
            },
            b.c1,
        ),
        (
            dim(
                D::Distance {
                    a: b.c1,
                    b: b.c2,
                    side: Side::Positive,
                },
                1.0,
            ),
            b.c2,
        ),
        (
            dim(
                D::Distance {
                    a: b.a1,
                    b: b.c1,
                    side: Side::Positive,
                },
                1.0,
            ),
            b.c1,
        ),
        (
            dim(
                D::HorizontalDistance {
                    a: b.p,
                    b: b.l1,
                    side: Side::Positive,
                },
                1.0,
            ),
            b.l1,
        ),
        (
            dim(
                D::VerticalDistance {
                    a: b.c1,
                    b: b.p,
                    side: Side::Positive,
                },
                1.0,
            ),
            b.c1,
        ),
        (dim(D::Length { line: b.a1 }, 1.0), b.a1),
        (
            dim(
                D::Angle {
                    a: b.l1,
                    b: b.c1,
                    branch: BR,
                },
                1.0,
            ),
            b.c1,
        ),
        (dim(D::Radius { curve: b.l1 }, 1.0), b.l1),
        (dim(D::Diameter { curve: b.p }, 1.0), b.p),
        (dim(D::ArcLength { arc: b.c1 }, 1.0), b.c1),
    ]
}

/// Variant names covered by a list of constraint kinds.
fn variant_names<'a>(kinds: impl Iterator<Item = &'a ConstraintKind>) -> Vec<&'static str> {
    let mut v: Vec<_> = kinds.map(ConstraintKind::name).collect();
    v.sort_unstable();
    v.dedup();
    v
}

const ALL_VARIANTS: usize = 13 + 8; // 13 constraint variants besides Dimension + 8 dimensions

#[test]
fn every_p0_entity_kind_is_created() {
    let b = base();
    let s = &b.s;
    let kind = |id| s.entity(id).unwrap().kind.name();
    assert_eq!(kind(b.p), "point");
    assert_eq!(kind(b.l1), "line");
    assert_eq!(kind(b.c1), "circle");
    assert_eq!(kind(b.a1), "arc");
    // 2 points + 3 lines (2 points each) + 2 circles (1 centre each) + 2 arcs (3 points each)
    assert_eq!(s.entities().count(), 2 + 3 * 3 + 2 * 2 + 2 * 4);
    assert_eq!(s.params().len(), 2 * (2 + 6 + 2 + 6) + 2);
    for (_, e) in s.entities() {
        assert!(!e.construction);
        assert_eq!(e.origin, EntityOrigin::User);
    }
    assert_eq!(s.point(b.p), Some(Point2::new(5.0, 5.0)));
    assert_eq!(s.point(b.l1), None);
}

#[test]
fn every_constraint_and_dimension_variant_is_created() {
    let mut b = base();
    let kinds = valid_constraints(&b);
    assert_eq!(variant_names(kinds.iter()).len(), ALL_VARIANTS);
    for k in kinds {
        let id =
            b.s.add_constraint(k.clone())
                .unwrap_or_else(|e| panic!("{k:?}: {e}"));
        let c = b.s.constraint(id).unwrap();
        assert_eq!(c.kind, k);
        assert!(c.aux.is_empty());
    }
    let ids: Vec<u32> = b.s.constraints().map(|(id, _)| id.0).collect();
    assert_eq!(ids, (0..ids.len() as u32).collect::<Vec<_>>());
}

#[test]
fn wrong_entity_kind_is_rejected_for_every_variant() {
    let mut b = base();
    let cases = wrong_kind_constraints(&b);
    // `Fix` accepts any entity kind; its negative test (unknown entity) is in
    // `other_constraint_errors`.
    assert_eq!(
        variant_names(cases.iter().map(|(k, _)| k)).len(),
        ALL_VARIANTS - 1
    );
    let rev = b.s.structure_rev();
    for (k, blamed) in cases {
        match b.s.add_constraint(k.clone()) {
            Err(SketchError::WrongEntityKind { entity, .. }) => {
                assert_eq!(entity, blamed, "{k:?}");
            }
            other => panic!("{k:?}: expected WrongEntityKind, got {other:?}"),
        }
    }
    assert_eq!(b.s.constraints().count(), 0);
    assert_eq!(b.s.structure_rev(), rev, "failed adds change nothing");
}

#[test]
fn other_constraint_errors() {
    use ConstraintKind as C;
    let mut b = base();
    let ghost = EntityId(9999);
    let l1s = children(&b.s, b.l1)[0];
    let c1c = children(&b.s, b.c1)[0];
    assert_eq!(
        b.s.add_constraint(C::Fix { entity: ghost }),
        Err(SketchError::UnknownEntity(ghost))
    );
    assert_eq!(
        b.s.add_constraint(C::Coincident { a: b.p, b: ghost }),
        Err(SketchError::UnknownEntity(ghost))
    );
    for k in [
        C::Coincident { a: b.p, b: b.p },
        C::Parallel { a: b.l1, b: b.l1 },
        C::Symmetric {
            a: b.l1,
            b: b.l2,
            axis: b.l1,
        },
        C::PointOnCurve {
            point: l1s,
            curve: b.l1,
        },
        C::PointOnCurve {
            point: c1c,
            curve: b.c1,
        },
        C::Midpoint {
            point: l1s,
            line: b.l1,
        },
    ] {
        assert!(
            matches!(
                b.s.add_constraint(k.clone()),
                Err(SketchError::NotApplicable(_))
            ),
            "{k:?}"
        );
    }
    use DimensionKind as D;
    for (k, v) in [
        (D::Length { line: b.l1 }, -1.0),
        (
            D::Distance {
                a: b.p,
                b: b.q,
                side: Side::Positive,
            },
            f64::NAN,
        ),
        (D::Radius { curve: b.c1 }, 0.0),
        (D::Diameter { curve: b.c1 }, -2.0),
        (D::ArcLength { arc: b.a1 }, f64::INFINITY),
        (
            D::Angle {
                a: b.l1,
                b: b.l2,
                branch: BR,
            },
            f64::NAN,
        ),
    ] {
        assert!(
            matches!(
                b.s.add_constraint(dim(k.clone(), v)),
                Err(SketchError::InvalidValue { .. })
            ),
            "{k:?} = {v}"
        );
    }
}

#[test]
fn dimension_value_and_driving() {
    let mut b = base();
    let fix =
        b.s.add_constraint(ConstraintKind::Fix { entity: b.p })
            .unwrap();
    let len =
        b.s.add_constraint(dim(DimensionKind::Length { line: b.l1 }, 10.0))
            .unwrap();

    let (srev, vrev) = (b.s.structure_rev(), b.s.values_rev());
    b.s.set_dimension_value(len, 12.5).unwrap();
    assert_eq!(b.s.structure_rev(), srev);
    assert_ne!(b.s.values_rev(), vrev);
    let d = b.s.constraint(len).unwrap().kind.as_dimension().unwrap();
    assert!((d.value - 12.5).abs() < f64::EPSILON);
    assert_eq!(d.kind.quantity(), Quantity::Length);
    assert!(matches!(
        b.s.set_dimension_value(len, -1.0),
        Err(SketchError::InvalidValue { .. })
    ));

    b.s.set_driving(len, false).unwrap();
    assert_ne!(b.s.structure_rev(), srev);
    assert!(
        !b.s.constraint(len)
            .unwrap()
            .kind
            .as_dimension()
            .unwrap()
            .driving
    );

    assert_eq!(
        b.s.set_driving(fix, true),
        Err(SketchError::NotADimension(fix))
    );
    assert_eq!(
        b.s.set_dimension_value(fix, 1.0),
        Err(SketchError::NotADimension(fix))
    );
    let ghost = amg_sketch::ConstraintId(77);
    assert_eq!(
        b.s.set_driving(ghost, true),
        Err(SketchError::UnknownConstraint(ghost))
    );
    assert_eq!(
        b.s.remove_constraint(ghost),
        Err(SketchError::UnknownConstraint(ghost))
    );

    let angle = DimensionKind::Angle {
        a: b.l1,
        b: b.l2,
        branch: BR,
    };
    assert_eq!(angle.quantity(), Quantity::Angle);

    b.s.remove_constraint(fix).unwrap();
    assert!(b.s.constraint(fix).is_none());
    // Ids are never reused.
    let next =
        b.s.add_constraint(ConstraintKind::Fix { entity: b.p })
            .unwrap();
    assert_eq!(next.0, len.0 + 1);
}

#[test]
fn entity_creation_errors() {
    let mut s = Sketch::new(Plane::XY);
    let p = s.add_point(Point2::ZERO);
    let l = s.add_line(Point2::ZERO, Point2::X);
    assert!(matches!(
        s.add_circle(Point2::ZERO, 0.0),
        Err(SketchError::InvalidValue { .. })
    ));
    assert!(matches!(
        s.add_circle(Point2::new(f64::NAN, 0.0), 1.0),
        Err(SketchError::InvalidValue { .. })
    ));
    assert!(matches!(
        s.add_arc(Point2::ZERO, Point2::ZERO, Point2::X),
        Err(SketchError::Degenerate(_))
    ));
    assert!(matches!(
        s.add_arc(Point2::ZERO, Point2::X, Point2::new(1e-9, 0.0)),
        Err(SketchError::Degenerate(_))
    ));
    assert!(matches!(
        s.add_line_between(p, p),
        Err(SketchError::NotApplicable(_))
    ));
    assert!(matches!(
        s.add_line_between(p, l),
        Err(SketchError::WrongEntityKind { entity, .. }) if entity == l
    ));
    assert_eq!(
        s.add_line_between(p, EntityId(500)),
        Err(SketchError::UnknownEntity(EntityId(500)))
    );
    let used = match s.entity(p).unwrap().kind {
        EntityKind::Point { x, .. } => x,
        _ => unreachable!(),
    };
    let fresh = s.alloc_param(1.0, ParamFlags::default());
    assert!(matches!(
        s.add_entity(
            EntityKind::Point { x: used, y: fresh },
            false,
            EntityOrigin::User
        ),
        Err(SketchError::NotApplicable(_))
    ));
    assert!(matches!(
        s.add_entity(
            EntityKind::Point { x: fresh, y: fresh },
            false,
            EntityOrigin::User
        ),
        Err(SketchError::NotApplicable(_))
    ));
    assert_eq!(
        s.add_entity(
            EntityKind::Point {
                x: fresh,
                y: ParamId(999)
            },
            false,
            EntityOrigin::User
        ),
        Err(SketchError::UnknownParam(ParamId(999)))
    );
    let r = s.alloc_param(-1.0, ParamFlags::default());
    assert!(matches!(
        s.add_entity(
            EntityKind::Circle {
                center: p,
                radius: r
            },
            false,
            EntityOrigin::User
        ),
        Err(SketchError::InvalidValue { .. })
    ));
    assert!(matches!(
        s.add_entity(
            EntityKind::Circle {
                center: l,
                radius: fresh
            },
            false,
            EntityOrigin::User
        ),
        Err(SketchError::WrongEntityKind { .. })
    ));
    assert_eq!(
        s.set_construction(EntityId(500), true),
        Err(SketchError::UnknownEntity(EntityId(500)))
    );
}

#[test]
fn generic_add_entity_and_projected_locking() {
    let mut s = Sketch::new(Plane::XZ);
    let link = ExternalLinkId(4);
    let mk = |s: &mut Sketch, x: f64, y: f64| {
        let px = s.alloc_param(x, ParamFlags::default());
        let py = s.alloc_param(y, ParamFlags::default());
        s.add_entity(
            EntityKind::Point { x: px, y: py },
            false,
            EntityOrigin::Projected(link),
        )
        .unwrap()
    };
    let a = mk(&mut s, 0.0, 0.0);
    let b = mk(&mut s, 3.0, 4.0);
    let l = s
        .add_entity(
            EntityKind::Line { start: a, end: b },
            true,
            EntityOrigin::Projected(link),
        )
        .unwrap();
    for (_, e) in s.entities() {
        for p in e.kind.own_params() {
            assert!(s.params().flags(p).locked);
        }
    }
    let e = s.entity(l).unwrap();
    assert!(e.construction);
    assert_eq!(e.origin, EntityOrigin::Projected(link));
    match s.curve(l) {
        Some(Curve2::Line(seg)) => {
            assert_eq!(seg.a, Point2::ZERO);
            assert_eq!(seg.b, Point2::new(3.0, 4.0));
        }
        other => panic!("{other:?}"),
    }
    s.set_construction(l, false).unwrap();
    assert!(!s.entity(l).unwrap().construction);
    let r = s.alloc_param(2.0, ParamFlags::default());
    let c = s.add_entity(
        EntityKind::Circle {
            center: a,
            radius: r,
        },
        false,
        EntityOrigin::Imported,
    );
    assert!(c.is_ok());
    assert!(!s.params().flags(r).locked);
}

#[test]
fn curves_evaluate() {
    let b = base();
    assert!(b.s.curve(b.p).is_none());
    assert!(b.s.curve(EntityId(1234)).is_none());
    match b.s.curve(b.c1) {
        Some(Curve2::Circle(c)) => {
            assert_eq!(c.center, Point2::new(30.0, 0.0));
            assert!((c.radius - 3.0).abs() < f64::EPSILON);
        }
        other => panic!("{other:?}"),
    }
    match b.s.curve(b.a1) {
        Some(Curve2::Arc(a)) => {
            assert!((a.radius - 4.0).abs() < 1e-12);
            assert!(a.start_angle.abs() < 1e-12);
            assert!((a.sweep - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn add_arc_projects_end_onto_radius() {
    let mut s = Sketch::new(Plane::XY);
    let a = s
        .add_arc(
            Point2::new(1.0, 1.0),
            Point2::new(4.0, 1.0),
            Point2::new(1.0, 11.0),
        )
        .unwrap();
    let ch = children(&s, a);
    let (c, st, en) = (
        s.point(ch[0]).unwrap(),
        s.point(ch[1]).unwrap(),
        s.point(ch[2]).unwrap(),
    );
    assert!((c.distance(st) - c.distance(en)).abs() < 1e-15);
    assert_eq!(en, Point2::new(1.0, 4.0));
}

#[test]
fn remove_entities_cascades_to_constraints() {
    let mut b = base();
    let l1 = children(&b.s, b.l1);
    // Second line sharing l1's end point.
    let shared = b.s.add_line_between(l1[1], b.q).unwrap();
    let on_l1 =
        b.s.add_constraint(ConstraintKind::Horizontal(HvTarget::Line(b.l1)))
            .unwrap();
    let pt =
        b.s.add_constraint(ConstraintKind::Coincident { a: l1[0], b: b.p })
            .unwrap();
    let keep =
        b.s.add_constraint(ConstraintKind::Fix { entity: b.q })
            .unwrap();
    let sym =
        b.s.add_constraint(ConstraintKind::Symmetric {
            a: b.c1,
            b: b.c2,
            axis: b.l1,
        })
        .unwrap();
    let len_shared =
        b.s.add_constraint(dim(DimensionKind::Length { line: shared }, 2.0))
            .unwrap();
    let rev = b.s.structure_rev();

    let report = b.s.remove_entities(&[b.l1]).unwrap();
    // l1 and its unshared start point go; the shared end point stays.
    assert_eq!(report.entities, vec![l1[0], b.l1]);
    assert_eq!(report.constraints, vec![on_l1, pt, sym]);
    assert!(b.s.entity(b.l1).is_none() && b.s.entity(l1[0]).is_none());
    assert!(b.s.entity(l1[1]).is_some());
    assert!(b.s.constraint(keep).is_some() && b.s.constraint(len_shared).is_some());
    assert_ne!(b.s.structure_rev(), rev);

    // Removing a point removes the curves built on it, their other points, and constraints.
    let report = b.s.remove_entities(&[b.q]).unwrap();
    assert_eq!(report.entities, vec![b.q, l1[1], shared]);
    assert_eq!(report.constraints, vec![keep, len_shared]);

    // Arc centre: arc and its start/end go.
    let a1 = children(&b.s, b.a1);
    let report = b.s.remove_entities(&[a1[0]]).unwrap();
    let mut expected = vec![a1[0], a1[1], a1[2], b.a1];
    expected.sort();
    assert_eq!(report.entities, expected);

    // Unknown ids remove nothing.
    let before = b.s.entities().count();
    assert_eq!(
        b.s.remove_entities(&[b.c1, EntityId(4242)]).unwrap_err(),
        SketchError::UnknownEntity(EntityId(4242))
    );
    assert_eq!(b.s.entities().count(), before);
    // Ids are never reused.
    let fresh = b.s.add_point(Point2::ZERO);
    assert!(b.s.entities().all(|(id, _)| id <= fresh));
    assert_eq!(fresh.0, b.a2.0 + 2, "after `shared`");
}

#[test]
fn compact_renumbers_params() {
    let mut b = base();
    b.s.remove_entities(&[b.l1, b.c2]).unwrap();
    let _unused = b.s.alloc_param(7.0, ParamFlags::default());
    let before: Vec<(EntityId, Option<Curve2>, Option<Point2>)> =
        b.s.entities()
            .map(|(id, _)| (id, b.s.curve(id), b.s.point(id)))
            .collect();
    let rev = b.s.structure_rev();
    b.s.compact();
    assert_ne!(b.s.structure_rev(), rev);
    // Dropped slots: l1's points (4), c2 centre + radius (3), the unused one (1).
    assert_eq!(b.s.params().len(), 2 * (2 + 6 + 2 + 6) + 2 - 4 - 3);
    let after: Vec<_> =
        b.s.entities()
            .map(|(id, _)| (id, b.s.curve(id), b.s.point(id)))
            .collect();
    assert_eq!(before, after);
    let mut owned: Vec<usize> =
        b.s.entities()
            .flat_map(|(_, e)| e.kind.own_params())
            .map(ParamId::index)
            .collect();
    owned.sort_unstable();
    assert_eq!(owned, (0..b.s.params().len()).collect::<Vec<_>>());
}

#[test]
fn revisions_are_unique_and_bumped() {
    let mut s = Sketch::new(Plane::XY);
    let t = Sketch::new(Plane::XY);
    assert_ne!(s.structure_rev(), t.structure_rev());
    let (sr, vr) = (s.structure_rev(), s.values_rev());
    let p = s.add_point(Point2::ZERO);
    assert_ne!(s.structure_rev(), sr);
    assert_ne!(s.values_rev(), vr);
    let (sr, vr) = (s.structure_rev(), s.values_rev());
    let x = s.entity(p).unwrap().kind.own_params()[0];
    s.params_mut().set(x, 3.0);
    assert_eq!(s.structure_rev(), sr);
    assert_ne!(s.values_rev(), vr);
    assert!((s.params().get(x) - 3.0).abs() < f64::EPSILON);
    let vr = s.values_rev();
    s.set_plane(Plane::YZ);
    assert_eq!(s.plane(), &Plane::YZ);
    assert_ne!(s.values_rev(), vr);
    let clone = s.clone();
    assert_eq!(clone.structure_rev(), s.structure_rev());
}
