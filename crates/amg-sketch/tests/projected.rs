//! SK-09: projected entities (`Sketch::add_projected`, `refresh_projected`,
//! `release_projected`).

#![allow(clippy::unwrap_used, clippy::float_cmp, missing_docs, reason = "tests")]

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use amg_core::{Plane, Point2, Vec2};
use amg_sketch::solve::{NativeSolver, SolveOptions, SolveStatus, SolverBackend};
use amg_sketch::{
    Curve2, EntityId, EntityKind, EntityOrigin, ExternalLinkId, ProjectedGeom, Sketch, SketchError,
};

const LINK: ExternalLinkId = ExternalLinkId(3);

fn p(x: f64, y: f64) -> Point2 {
    Point2::new(x, y)
}

/// One piece of every kind, placed by `at`.
fn pieces(at: f64) -> Vec<ProjectedGeom> {
    vec![
        ProjectedGeom::Point(p(at, 1.0)),
        ProjectedGeom::Line {
            a: p(at, 0.0),
            b: p(at + 4.0, 2.0),
        },
        ProjectedGeom::Circle {
            center: p(at, 10.0),
            radius: 2.0 + at * 0.1,
        },
        // Clockwise: stored counter-clockwise.
        ProjectedGeom::Arc {
            center: p(at, 20.0),
            radius: 3.0,
            start_angle: 2.0 + at * 0.01,
            sweep: -1.5,
        },
        ProjectedGeom::Ellipse {
            center: p(at, 30.0),
            major: Vec2::new(5.0, at * 0.1),
            minor_radius: 2.0,
            t0: 0.0,
            t1: TAU,
        },
        ProjectedGeom::Ellipse {
            center: p(at, 40.0),
            major: Vec2::new(4.0, 1.0),
            minor_radius: 2.0,
            t0: 2.5,
            t1: 0.5,
        },
        ProjectedGeom::Spline {
            degree: 2,
            knots: vec![0.0, 0.0, 0.0, 1.0 + at, 1.0 + at, 1.0 + at],
            poles: vec![p(at, 50.0), p(at + 1.0, 53.0), p(at + 2.0, 50.0)],
            weights: Some(vec![1.0, 0.5 + at * 0.01, 1.0]),
        },
    ]
}

/// Largest distance between two curves at the same fraction of their parameter ranges, the
/// second one run either way (arcs are stored counter-clockwise).
fn sampled(a: &Curve2, b: &Curve2) -> f64 {
    let (la, lb) = (a.param_range().1, b.param_range().1);
    let one_way = |reverse: bool| {
        (0..=16)
            .map(|i| {
                let f = f64::from(i) / 16.0;
                let g = if reverse { 1.0 - f } else { f };
                a.eval(f * la).distance(b.eval(g * lb))
            })
            .fold(0.0, f64::max)
    };
    one_way(false).min(one_way(true))
}

/// How far entity `e` is from `g`.
fn distance(s: &Sketch, e: EntityId, g: &ProjectedGeom) -> f64 {
    if let ProjectedGeom::Point(q) = g {
        return s.point(e).unwrap().distance(*q);
    }
    let mut other = Sketch::new(Plane::XY);
    let o = other.add_projected(LINK, g).unwrap();
    sampled(&s.curve(e).unwrap(), &other.curve(o).unwrap())
}

fn all_locked(s: &Sketch) -> bool {
    s.entities().all(|(_, e)| {
        e.origin == EntityOrigin::Projected(LINK)
            && e.kind
                .own_params()
                .into_iter()
                .all(|q| s.params().flags(q).locked)
    })
}

#[test]
fn every_kind_is_added_locked_and_refreshed_in_place() {
    let mut s = Sketch::new(Plane::XY);
    let first = pieces(0.0);
    let ids: Vec<EntityId> = first
        .iter()
        .map(|g| s.add_projected(LINK, g).unwrap())
        .collect();
    assert!(all_locked(&s));
    let names: Vec<&str> = ids
        .iter()
        .map(|&e| s.entity(e).unwrap().kind.name())
        .collect();
    let wanted: Vec<&str> = first.iter().map(ProjectedGeom::name).collect();
    assert_eq!(names, wanted);
    for (&e, g) in ids.iter().zip(&first) {
        assert!(distance(&s, e, g) < 1e-12, "{g:?}");
    }
    // The clockwise arc runs counter-clockwise from its other end.
    let Some(Curve2::Arc(a)) = s.curve(ids[3]) else {
        panic!("an arc");
    };
    assert!((a.start_angle - 0.5).abs() < 1e-12 && (a.sweep - 1.5).abs() < 1e-12);

    // Refreshing with the same geometry changes nothing, not even the revisions.
    let (values, structure) = (s.values_rev(), s.structure_rev());
    let same: Vec<_> = ids.iter().copied().zip(&first).collect();
    s.refresh_projected(&same).unwrap();
    assert_eq!((s.values_rev(), s.structure_rev()), (values, structure));

    let moved = pieces(7.0);
    let n = s.entities().count();
    let items: Vec<_> = ids.iter().copied().zip(&moved).collect();
    s.refresh_projected(&items).unwrap();
    assert_eq!(s.entities().count(), n);
    assert!(all_locked(&s));
    for (&e, g) in ids.iter().zip(&moved) {
        assert!(distance(&s, e, g) < 1e-9, "{g:?}");
    }

    // What was built through the API loads again.
    let json = serde_json::to_string(&s).unwrap();
    let back: Sketch = serde_json::from_str(&json).unwrap();
    assert_eq!(serde_json::to_string(&back).unwrap(), json);

    // The solver leaves projected geometry alone and does not call its arcs redundant.
    let before = s.params().as_slice().to_vec();
    let report = NativeSolver::new().solve(&mut s, &SolveOptions::default(), &|| false);
    assert_eq!(report.status, SolveStatus::Converged);
    assert_eq!(s.params().as_slice(), before);
}

#[test]
fn a_refresh_is_all_or_nothing_and_keeps_kinds() {
    let mut s = Sketch::new(Plane::XY);
    let line = s.add_projected(LINK, &pieces(0.0)[1]).unwrap();
    let circle = s.add_projected(LINK, &pieces(0.0)[2]).unwrap();
    let user = s.add_point(p(0.0, 0.0)).unwrap();
    let before = s.params().as_slice().to_vec();
    let moved = pieces(5.0);
    for bad in [
        vec![(line, &moved[1]), (circle, &moved[3])],
        vec![(line, &moved[1]), (user, &moved[0])],
        vec![(line, &moved[1]), (EntityId(99), &moved[0])],
    ] {
        assert!(s.refresh_projected(&bad).is_err(), "{bad:?}");
        assert_eq!(s.params().as_slice(), before);
    }
    let degenerate = ProjectedGeom::Circle {
        center: p(0.0, 0.0),
        radius: 0.0,
    };
    assert!(matches!(
        s.refresh_projected(&[(circle, &degenerate)]),
        Err(SketchError::InvalidValue { .. })
    ));
    let n = s.entities().count();
    assert!(s.add_projected(LINK, &degenerate).is_err());
    assert_eq!(s.entities().count(), n, "a refused piece leaves nothing");
}

#[test]
fn a_refreshed_ellipse_keeps_its_major_point_when_the_axes_flip_or_swap() {
    let mut s = Sketch::new(Plane::XY);
    let arc = |major: Vec2, minor_radius: f64, t0: f64, t1: f64| ProjectedGeom::Ellipse {
        center: p(1.0, 2.0),
        major,
        minor_radius,
        t0,
        t1,
    };
    let e = s
        .add_projected(LINK, &arc(Vec2::new(5.0, 0.0), 2.0, 0.4, 2.0))
        .unwrap();
    let EntityKind::EllipticArc { major, .. } = s.entity(e).unwrap().kind else {
        panic!("an elliptical arc");
    };
    let original = s.curve(e).unwrap();
    // The same arc with the major axis reversed, and described from its other axis.
    for same in [
        arc(Vec2::new(-5.0, 0.0), 2.0, 0.4 + PI, 2.0 + PI),
        arc(Vec2::new(0.0, 2.0), 5.0, 0.4 - FRAC_PI_2, 2.0 - FRAC_PI_2),
        arc(Vec2::new(0.0, -2.0), 5.0, 0.4 + FRAC_PI_2, 2.0 + FRAC_PI_2),
    ] {
        s.refresh_projected(&[(e, &same)]).unwrap();
        assert!(s.point(major).unwrap().distance(p(6.0, 2.0)) < 1e-12);
        assert!(sampled(&s.curve(e).unwrap(), &original) < 1e-12);
    }
}

#[test]
fn releasing_unlocks_and_makes_user_geometry() {
    let mut s = Sketch::new(Plane::XY);
    let e = s.add_projected(LINK, &pieces(0.0)[0]).unwrap();
    s.release_projected(e).unwrap();
    let ent = s.entity(e).unwrap();
    assert_eq!(ent.origin, EntityOrigin::User);
    assert!(
        ent.kind
            .own_params()
            .into_iter()
            .all(|q| !s.params().flags(q).locked)
    );
    assert!(s.release_projected(EntityId(9)).is_err());
}
