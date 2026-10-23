//! SK-16 acceptance: sketch fillet and chamfer fixtures.
//!
//! Every fixture checks the new geometry, the constraint set, the whole-sketch DOF against a
//! hand-computed value (SK-02 rank helper `dof`), residual 0 (`build_system`) and that the
//! SK-13 prototype solver (`solve::proto`, the only solver until SK-03) reports `Converged`
//! without moving anything.

#![allow(
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::too_many_lines,
    reason = "test helpers; a failure is a test failure"
)]

use std::collections::BTreeMap;
use std::f64::consts::{FRAC_PI_3, FRAC_PI_4, PI};

use amg_core::{Plane, Point2};
use amg_sketch::build;
use amg_sketch::edit::{self, EditOutcome};
use amg_sketch::solve::proto::{self, ProtoOptions, ProtoStatus};
use amg_sketch::solve::{build_system, dof};
use amg_sketch::{
    ChamferSpec, ConstraintId, ConstraintKind, Curve2, Dimension, DimensionKind, EntityId,
    EntityKind, EntityOrigin, HvTarget, Sketch, SketchError,
};

fn p(x: f64, y: f64) -> Point2 {
    Point2::new(x, y)
}

fn new() -> Sketch {
    Sketch::new(Plane::XY)
}

fn residual(s: &Sketch) -> f64 {
    let sys = build_system(s).unwrap();
    sys.system.max_abs_residual(s.params().as_slice())
}

/// Residual 0 via the SK-02 path, and the prototype solver agrees without moving anything.
fn verify(s: &Sketch) {
    assert!(residual(s) < 1e-9, "residual {}", residual(s));
    let mut t = s.clone();
    let rep = proto::solve(&mut t, &ProtoOptions::default()).unwrap();
    assert_eq!(rep.status, ProtoStatus::Converged, "{rep:?}");
    let moved = s
        .params()
        .as_slice()
        .iter()
        .zip(t.params().as_slice())
        .fold(0.0_f64, |m, (a, b)| m.max((a - b).abs()));
    assert!(moved < 1e-6, "solver moved parameters by {moved}");
    let json = serde_json::to_string(s).unwrap();
    let back: Sketch = serde_json::from_str(&json).unwrap();
    assert_eq!(back.entities().count(), s.entities().count());
}

fn counts(s: &Sketch) -> BTreeMap<&'static str, usize> {
    let mut m = BTreeMap::new();
    for (_, c) in s.constraints() {
        *m.entry(c.kind.name()).or_insert(0) += 1;
    }
    m
}

fn count_named(s: &Sketch, n: &str) -> usize {
    counts(s).get(n).copied().unwrap_or(0)
}

fn map(v: &[(&'static str, usize)]) -> BTreeMap<&'static str, usize> {
    v.iter().copied().collect()
}

fn near(a: Point2, b: Point2) -> bool {
    a.distance(b) < 1e-9
}

type Made = Vec<(EntityId, Curve2)>;

/// Curves (lines and arcs) created by the edit.
fn made(s: &Sketch, o: &EditOutcome) -> Made {
    o.result
        .created
        .iter()
        .filter_map(|&id| s.curve(id).map(|c| (id, c)))
        .collect()
}

fn all_curves(s: &Sketch) -> Made {
    s.entities()
        .filter_map(|(id, _)| s.curve(id).map(|c| (id, c)))
        .collect()
}

fn ends(c: &Curve2) -> (Point2, Point2) {
    (c.eval(0.0), c.eval(c.param_range().1))
}

fn find_line(v: &Made, a: Point2, b: Point2) -> Option<EntityId> {
    v.iter().find_map(|(id, c)| match c {
        Curve2::Line(_) => {
            let (e0, e1) = ends(c);
            ((near(e0, a) && near(e1, b)) || (near(e0, b) && near(e1, a))).then_some(*id)
        }
        _ => None,
    })
}

#[track_caller]
fn line(v: &Made, a: (f64, f64), b: (f64, f64)) -> EntityId {
    find_line(v, p(a.0, a.1), p(b.0, b.1)).unwrap_or_else(|| panic!("no line {a:?}-{b:?} in {v:?}"))
}

/// The arc with this centre and radius.
#[track_caller]
fn arc(v: &Made, c: (f64, f64), r: f64) -> (EntityId, Curve2) {
    v.iter()
        .find(|(_, k)| {
            matches!(k, Curve2::Arc(a) if near(a.center, p(c.0, c.1)) && (a.radius - r).abs() < 1e-9)
        })
        .cloned()
        .unwrap_or_else(|| panic!("no arc {c:?} r={r} in {v:?}"))
}

fn has_end(c: &Curve2, q: (f64, f64)) -> bool {
    let (e0, e1) = ends(c);
    near(e0, p(q.0, q.1)) || near(e1, p(q.0, q.1))
}

fn dims(s: &Sketch, name: &str) -> Vec<f64> {
    s.constraints()
        .filter_map(|(_, c)| c.kind.as_dimension())
        .filter(|d| d.kind.name() == name)
        .map(|d| d.value)
        .collect()
}

fn dim_id(s: &Sketch, name: &str) -> ConstraintId {
    s.constraints()
        .find(|(_, c)| c.kind.as_dimension().is_some_and(|d| d.kind.name() == name))
        .map(|(id, _)| id)
        .unwrap()
}

fn find_in_sketch(s: &Sketch, a: (f64, f64), b: (f64, f64)) -> EntityId {
    line(&all_curves(s), a, b)
}

fn unchanged_on_error(
    s: &mut Sketch,
    f: impl FnOnce(&mut Sketch) -> Result<EditOutcome, SketchError>,
) -> SketchError {
    let before = serde_json::to_string(s).unwrap();
    let err = f(s).unwrap_err();
    assert_eq!(
        serde_json::to_string(s).unwrap(),
        before,
        "sketch changed on error"
    );
    err
}

/// Two lines sharing the corner point (0,0): `a` along +x to (10,0), `b` along +y to (0,10).
fn corner() -> (Sketch, EntityId, EntityId) {
    let mut s = new();
    let c = s.add_point(p(0.0, 0.0)).unwrap();
    let e1 = s.add_point(p(10.0, 0.0)).unwrap();
    let e2 = s.add_point(p(0.0, 10.0)).unwrap();
    let a = s.add_line_between(c, e1).unwrap();
    let b = s.add_line_between(c, e2).unwrap();
    (s, a, b)
}

/// Line y = 0 from (-10,0) to (0,0) and CCW quarter arc about the origin (R = 10) from (0,10)
/// to (-10,0); they share (-10,0). DOF 7.
fn line_arc() -> (Sketch, EntityId, EntityId) {
    let mut s = new();
    let arc = s.add_arc(p(0.0, 0.0), p(0.0, 10.0), p(-10.0, 0.0)).unwrap();
    let EntityKind::Arc { end, .. } = s.entity(arc).unwrap().kind else {
        unreachable!()
    };
    let o = s.add_point(p(0.0, 0.0)).unwrap();
    let l = s.add_line_between(end, o).unwrap();
    (s, l, arc)
}

/// Arc 1: about (0,0), R = 10, from (10,0) CCW to X = (7.5, sqrt(43.75)); arc 2: about (15,0),
/// R = 10, from X CCW to (5,0). They share X. DOF 8.
fn arc_arc() -> (Sketch, EntityId, EntityId) {
    let mut s = new();
    let x = p(7.5, 43.75_f64.sqrt());
    let a1 = s.add_arc(p(0.0, 0.0), p(10.0, 0.0), x).unwrap();
    let EntityKind::Arc { end: xp, .. } = s.entity(a1).unwrap().kind else {
        unreachable!()
    };
    let c2 = s.add_point(p(15.0, 0.0)).unwrap();
    let e2 = s.add_point(p(5.0, 0.0)).unwrap();
    let a2 = s
        .add_entity(
            EntityKind::Arc {
                center: c2,
                start: xp,
                end: e2,
            },
            false,
            EntityOrigin::User,
        )
        .unwrap();
    (s, a1, a2)
}

fn n_points(s: &Sketch) -> usize {
    s.entities()
        .filter(|(_, e)| matches!(e.kind, EntityKind::Point { .. }))
        .count()
}

// ======================================================================================
// Fillet
// ======================================================================================

#[test]
fn fillet_line_line_shared_corner() {
    let (mut s, a, b) = corner();
    assert_eq!(dof(&s).unwrap(), 6);
    let o = edit::fillet_detailed(&mut s, a, b, p(1.0, 1.0), 2.0).unwrap();
    let v = made(&s, &o);
    line(&v, (2.0, 0.0), (10.0, 0.0));
    line(&v, (0.0, 2.0), (0.0, 10.0));
    let (f, fc) = arc(&v, (2.0, 2.0), 2.0);
    assert!(has_end(&fc, (2.0, 0.0)) && has_end(&fc, (0.0, 2.0)));
    assert_eq!(counts(&s), map(&[("tangent", 2), ("radius", 1)]));
    assert_eq!(dims(&s, "radius"), vec![2.0]);
    assert!(s.entity(a).is_none() && s.entity(b).is_none() && s.entity(f).is_some());
    // 3 points -> 5 points; the corner is gone, the arc adds one equation, tangency 2, radius 1.
    assert_eq!(n_points(&s), 5);
    assert_eq!(dof(&s).unwrap(), 6);
    verify(&s);
}

#[test]
fn fillet_single_valid_corner_ignores_a_far_pick() {
    let (mut s, a, b) = corner();
    edit::fillet(&mut s, a, b, p(8.0, 8.0), 1.0).unwrap();
    verify(&s);
}

#[test]
fn fillet_argument_order_does_not_matter() {
    let (mut s, a, b) = corner();
    let o = edit::fillet_detailed(&mut s, b, a, p(1.0, 1.0), 2.0).unwrap();
    let v = made(&s, &o);
    line(&v, (2.0, 0.0), (10.0, 0.0));
    arc(&v, (2.0, 2.0), 2.0);
    assert_eq!(dof(&s).unwrap(), 6);
    verify(&s);
}

#[test]
fn fillet_rectangle_corner_has_dof_4() {
    let mut s = new();
    build::rectangle_2pt(&mut s, p(0.0, 0.0), p(10.0, 6.0)).unwrap();
    assert_eq!(dof(&s).unwrap(), 4);
    let bottom = find_in_sketch(&s, (0.0, 0.0), (10.0, 0.0));
    let right = find_in_sketch(&s, (10.0, 0.0), (10.0, 6.0));
    let o = edit::fillet_detailed(&mut s, bottom, right, p(9.0, 1.0), 2.0).unwrap();
    let v = made(&s, &o);
    line(&v, (0.0, 0.0), (8.0, 0.0));
    line(&v, (10.0, 2.0), (10.0, 6.0));
    arc(&v, (8.0, 2.0), 2.0);
    // H and V migrated to the new pieces; 4 -> 4.
    assert_eq!(
        counts(&s),
        map(&[
            ("horizontal", 2),
            ("vertical", 2),
            ("tangent", 2),
            ("radius", 1)
        ])
    );
    assert!(o.migration.dropped.is_empty());
    assert_eq!(dof(&s).unwrap(), 4);
    verify(&s);
}

#[test]
fn fillet_all_four_rectangle_corners() {
    let mut s = new();
    build::rectangle_2pt(&mut s, p(0.0, 0.0), p(10.0, 6.0)).unwrap();
    // (corner, far end of the first line, far end of the second, pick)
    let corners = [
        ((0.0, 0.0), (10.0, 0.0), (0.0, 6.0), (1.0, 1.0)),
        ((10.0, 0.0), (0.0, 0.0), (10.0, 6.0), (9.0, 1.0)),
        ((10.0, 6.0), (10.0, 0.0), (0.0, 6.0), (9.0, 5.0)),
        ((0.0, 6.0), (10.0, 6.0), (0.0, 0.0), (1.0, 5.0)),
    ];
    for (c, a_far, b_far, nr) in corners {
        let all = all_curves(&s);
        let find_dir = |d: (f64, f64)| {
            all.iter()
                .find(|(_, k)| {
                    matches!(k, Curve2::Line(_)) && has_end(k, c) && {
                        let (e0, e1) = ends(k);
                        let far = if near(e0, p(c.0, c.1)) { e1 } else { e0 };
                        let v = far - p(c.0, c.1);
                        (v.x.abs() > 1e-9) == (d.0 != 0.0) && v.dot(p(d.0, d.1)) > 0.0
                    }
                })
                .map(|(id, _)| *id)
                .unwrap_or_else(|| panic!("no edge from {c:?} towards {d:?}"))
        };
        let dir_a = (a_far.0 - c.0, a_far.1 - c.1);
        let dir_b = (b_far.0 - c.0, b_far.1 - c.1);
        edit::fillet(&mut s, find_dir(dir_a), find_dir(dir_b), p(nr.0, nr.1), 1.0).unwrap();
    }
    let arcs = all_curves(&s)
        .iter()
        .filter(|(_, c)| matches!(c, Curve2::Arc(_)))
        .count();
    assert_eq!(arcs, 4);
    // Each corner keeps the rectangle's 4 DOF (corner point replaced by a determined arc).
    assert_eq!(dof(&s).unwrap(), 4);
    verify(&s);
}

#[test]
fn fillet_determined_by_neighbours_and_radius() {
    // Far ends fixed, one line horizontal, one vertical: DOF 0 before and after.
    let mut s = new();
    let c = s.add_point(p(0.0, 0.0)).unwrap();
    let e1 = s.add_point(p(10.0, 0.0)).unwrap();
    let e2 = s.add_point(p(0.0, 10.0)).unwrap();
    let a = s.add_line_between(c, e1).unwrap();
    let b = s.add_line_between(c, e2).unwrap();
    s.add_constraint(ConstraintKind::Fix { entity: e1 })
        .unwrap();
    s.add_constraint(ConstraintKind::Fix { entity: e2 })
        .unwrap();
    s.add_constraint(ConstraintKind::Horizontal(HvTarget::Line(a)))
        .unwrap();
    s.add_constraint(ConstraintKind::Vertical(HvTarget::Line(b)))
        .unwrap();
    assert_eq!(dof(&s).unwrap(), 0);
    edit::fillet(&mut s, a, b, p(1.0, 1.0), 3.0).unwrap();
    assert_eq!(dof(&s).unwrap(), 0, "the fillet is fully determined");
    verify(&s);
    // Changing the radius moves the arc and re-solves to a consistent sketch.
    let rid = dim_id(&s, "radius");
    s.set_dimension_value(rid, 4.0).unwrap();
    let rep = proto::solve(&mut s, &ProtoOptions::default()).unwrap();
    assert_eq!(rep.status, ProtoStatus::Converged, "{rep:?}");
    assert!(residual(&s) < 1e-9);
    let radii: Vec<f64> = all_curves(&s)
        .iter()
        .filter_map(|(_, c)| match c {
            Curve2::Arc(a) => Some(a.radius),
            _ => None,
        })
        .collect();
    assert_eq!(radii.len(), 1);
    assert!((radii[0] - 4.0).abs() < 1e-6);
}

#[test]
fn fillet_disjoint_lines_that_reach_the_tangent_points() {
    let mut s = new();
    let a = s.add_line(p(1.0, 0.0), p(10.0, 0.0)).unwrap();
    let b = s.add_line(p(0.0, 1.0), p(0.0, 10.0)).unwrap();
    assert_eq!(dof(&s).unwrap(), 8);
    let o = edit::fillet_detailed(&mut s, a, b, p(1.0, 1.0), 1.0).unwrap();
    let v = made(&s, &o);
    arc(&v, (1.0, 1.0), 1.0);
    line(&v, (1.0, 0.0), (10.0, 0.0));
    line(&v, (0.0, 1.0), (0.0, 10.0));
    // The two end points that used to be free are now tied to the fillet: 8 -> 6.
    assert_eq!(dof(&s).unwrap(), 6);
    verify(&s);
}

#[test]
fn fillet_crossing_lines_uses_the_quadrant_near_the_pick() {
    let mut s = new();
    let a = s.add_line(p(-10.0, 0.0), p(10.0, 0.0)).unwrap();
    let b = s.add_line(p(0.0, -10.0), p(0.0, 10.0)).unwrap();
    let o = edit::fillet_detailed(&mut s, a, b, p(-1.0, 1.0), 2.0).unwrap();
    let v = made(&s, &o);
    arc(&v, (-2.0, 2.0), 2.0);
    // Each curve keeps the part away from the filleted quadrant's corner side.
    line(&v, (-10.0, 0.0), (-2.0, 0.0));
    line(&v, (0.0, 2.0), (0.0, 10.0));
    assert_eq!(dof(&s).unwrap(), 6);
    verify(&s);
}

#[test]
fn fillet_line_arc() {
    let (mut s, l, a) = line_arc();
    assert_eq!(dof(&s).unwrap(), 7);
    let o = edit::fillet_detailed(&mut s, l, a, p(-9.0, 1.0), 2.0).unwrap();
    let v = made(&s, &o);
    let tx = -(64.0_f64 - 4.0).sqrt();
    // Fillet circle inside the big circle: centre at distance R - r = 8 from the origin.
    let (_, fc) = arc(&v, (tx, 2.0), 2.0);
    assert!(has_end(&fc, (tx, 0.0)));
    line(&v, (tx, 0.0), (0.0, 0.0));
    let (_, kept) = arc(&v, (0.0, 0.0), 10.0);
    assert!(has_end(&kept, (0.0, 10.0)));
    assert!(has_end(&kept, (tx * 10.0 / 8.0, 2.0 * 10.0 / 8.0)));
    assert_eq!(counts(&s), map(&[("tangent", 2), ("radius", 1)]));
    assert_eq!(dof(&s).unwrap(), 7);
    verify(&s);
}

#[test]
fn fillet_line_arc_lower_candidate_is_not_on_the_arc() {
    // A fillet below the line would touch the circle outside the arc's extent, so the
    // upper one is the only valid fillet whatever `near` says.
    let (mut s, l, a) = line_arc();
    let o = edit::fillet_detailed(&mut s, l, a, p(-9.5, -1.0), 2.0).unwrap();
    let v = made(&s, &o);
    assert!(v.iter().any(
        |(_, c)| matches!(c, Curve2::Arc(a) if a.center.y > 0.0 && (a.radius - 2.0).abs() < 1e-9)
    ));
    assert_eq!(dof(&s).unwrap(), 7);
    verify(&s);
}

#[test]
fn fillet_arc_arc() {
    let (mut s, a1, a2) = arc_arc();
    assert_eq!(dof(&s).unwrap(), 8);
    let o = edit::fillet_detailed(&mut s, a1, a2, p(7.5, 5.0), 2.0).unwrap();
    let v = made(&s, &o);
    // The fillet circle is tangent to both circles (centres (0,0) and (15,0), R = 10).
    let fil = v
        .iter()
        .find_map(|(_, c)| match c {
            Curve2::Arc(a) if (a.radius - 2.0).abs() < 1e-9 => Some(*a),
            _ => None,
        })
        .expect("fillet arc");
    for c in [p(0.0, 0.0), p(15.0, 0.0)] {
        let d = fil.center.distance(c);
        assert!(
            (d - 12.0).abs() < 1e-9 || (d - 8.0).abs() < 1e-9,
            "centre distance {d}"
        );
    }
    assert_eq!(counts(&s), map(&[("tangent", 2), ("radius", 1)]));
    assert_eq!(dof(&s).unwrap(), 8);
    verify(&s);
}

#[test]
fn fillet_arc_arc_separate_arcs() {
    let mut s = new();
    let a1 = s.add_arc(p(0.0, 0.0), p(5.0, 0.0), p(0.0, 5.0)).unwrap();
    let a2 = s.add_arc(p(12.0, 0.0), p(12.0, 5.0), p(7.0, 0.0)).unwrap();
    assert_eq!(dof(&s).unwrap(), 10);
    // Circles of R = 5 about (0,0) and (12,0) are 12 apart: a fillet of r = 2 touches both
    // from outside only if the arcs reach the tangent points (they end at (5,0) / (7,0)).
    let o = edit::fillet_detailed(&mut s, a1, a2, p(6.0, 2.0), 2.0);
    // r = 2: tangent circle centre at distance 7 from both: (6, sqrt(49-36)=3.606), tangent
    // points at angle 31 deg on arc 1 (within its 0..90 extent) and 149 deg on arc 2 (90..180).
    let o = o.unwrap();
    let v = made(&s, &o);
    assert!(
        v.iter()
            .any(|(_, c)| matches!(c, Curve2::Arc(a) if (a.radius - 2.0).abs() < 1e-9))
    );
    // Arcs share no point: both cut points become tied to the fillet, losing 2.
    assert_eq!(dof(&s).unwrap(), 8);
    let _ = (a1, a2);
    verify(&s);
}

#[test]
fn fillet_migrates_curve_constraints_and_reports_dropped_dimensions() {
    let (mut s, a, b) = corner();
    let len = s
        .add_constraint(ConstraintKind::Dimension(Dimension {
            kind: DimensionKind::Length { line: a },
            value: 10.0,
            driving: true,
            label_pos: p(5.0, 0.0),
        }))
        .unwrap();
    s.add_constraint(ConstraintKind::Perpendicular { a, b })
        .unwrap();
    let o = edit::fillet_detailed(&mut s, a, b, p(1.0, 1.0), 2.0).unwrap();
    assert!(o.migration.dropped_dimensions().any(|d| d.id == len));
    assert!(o.result.constraints_removed.contains(&len));
    // Perpendicular(a, b) went through two edits and ends up on the two pieces.
    assert_eq!(count_named(&s, "perpendicular"), 1);
    assert_eq!(o.migration.migrated.len(), 1);
    verify(&s);
}

// ---- fillet: does not fit / errors ------------------------------------------------------------

#[test]
fn fillet_radius_too_large_is_not_applicable() {
    let (mut s, a, b) = corner();
    let e = unchanged_on_error(&mut s, |s| {
        edit::fillet_detailed(s, a, b, p(1.0, 1.0), 20.0)
    });
    assert!(matches!(e, SketchError::NotApplicable(_)), "{e:?}");
    // Exactly as long as the lines: the tangent point is at the far end, nothing is kept.
    let e = unchanged_on_error(&mut s, |s| {
        edit::fillet_detailed(s, a, b, p(1.0, 1.0), 10.0)
    });
    assert!(matches!(e, SketchError::NotApplicable(_)), "{e:?}");
}

#[test]
fn fillet_with_ellipse_is_wrong_kind() {
    let mut s = new();
    let l = s.add_line(p(0.0, 0.0), p(10.0, 0.0)).unwrap();
    let e = s.add_ellipse(p(5.0, 3.0), p(9.0, 3.0), 3.0).unwrap();
    let err = unchanged_on_error(&mut s, |s| edit::fillet_detailed(s, l, e, p(5.0, 0.5), 1.0));
    assert!(
        matches!(err, SketchError::WrongEntityKind { .. }),
        "{err:?}"
    );
}

#[test]
fn fillet_parallel_lines_is_not_applicable() {
    let mut s = new();
    let a = s.add_line(p(0.0, 0.0), p(10.0, 0.0)).unwrap();
    let b = s.add_line(p(0.0, 4.0), p(10.0, 4.0)).unwrap();
    let e = unchanged_on_error(&mut s, |s| edit::fillet_detailed(s, a, b, p(5.0, 2.0), 2.0));
    assert!(matches!(e, SketchError::NotApplicable(_)), "{e:?}");
}

#[test]
fn fillet_lines_that_do_not_reach_is_not_applicable() {
    let mut s = new();
    let a = s.add_line(p(3.0, 0.0), p(10.0, 0.0)).unwrap();
    let b = s.add_line(p(0.0, 3.0), p(0.0, 10.0)).unwrap();
    let e = unchanged_on_error(&mut s, |s| edit::fillet_detailed(s, a, b, p(1.0, 1.0), 2.0));
    assert!(matches!(e, SketchError::NotApplicable(_)), "{e:?}");
}

#[test]
fn fillet_line_arc_that_do_not_meet_is_not_applicable() {
    let mut s = new();
    let l = s.add_line(p(20.0, 0.0), p(30.0, 0.0)).unwrap();
    let a = s.add_arc(p(0.0, 0.0), p(5.0, 0.0), p(0.0, 5.0)).unwrap();
    let e = unchanged_on_error(&mut s, |s| {
        edit::fillet_detailed(s, l, a, p(10.0, 0.0), 1.0)
    });
    assert!(matches!(e, SketchError::NotApplicable(_)), "{e:?}");
}

#[test]
fn fillet_arc_arc_radius_too_large_is_not_applicable() {
    let (mut s, a1, a2) = arc_arc();
    let e = unchanged_on_error(&mut s, |s| {
        edit::fillet_detailed(s, a1, a2, p(7.5, 5.0), 40.0)
    });
    assert!(matches!(e, SketchError::NotApplicable(_)), "{e:?}");
}

#[test]
fn fillet_input_errors() {
    let (mut s, a, b) = corner();
    let c = s.add_circle(p(20.0, 20.0), 3.0).unwrap();
    let pt = s.add_point(p(1.0, 1.0)).unwrap();
    for r in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        let e = unchanged_on_error(&mut s, |s| edit::fillet_detailed(s, a, b, p(1.0, 1.0), r));
        assert!(matches!(e, SketchError::InvalidValue { .. }), "{e:?}");
    }
    let e = unchanged_on_error(&mut s, |s| edit::fillet_detailed(s, a, a, p(1.0, 1.0), 1.0));
    assert!(matches!(e, SketchError::NotApplicable(_)));
    let e = unchanged_on_error(&mut s, |s| edit::fillet_detailed(s, a, c, p(1.0, 1.0), 1.0));
    assert!(matches!(e, SketchError::NotApplicable(_)));
    let e = unchanged_on_error(&mut s, |s| {
        edit::fillet_detailed(s, a, pt, p(1.0, 1.0), 1.0)
    });
    assert!(matches!(e, SketchError::WrongEntityKind { .. }));
    let e = unchanged_on_error(&mut s, |s| {
        edit::fillet_detailed(s, a, EntityId(999), p(1.0, 1.0), 1.0)
    });
    assert!(matches!(e, SketchError::UnknownEntity(_)));
}

// ======================================================================================
// Chamfer
// ======================================================================================

#[test]
fn chamfer_equal_line_line() {
    let (mut s, a, b) = corner();
    assert_eq!(dof(&s).unwrap(), 6);
    let o = edit::chamfer_detailed(&mut s, a, b, p(1.0, 1.0), ChamferSpec::Equal(2.0)).unwrap();
    let v = made(&s, &o);
    line(&v, (2.0, 0.0), (10.0, 0.0));
    line(&v, (0.0, 2.0), (0.0, 10.0));
    line(&v, (2.0, 0.0), (0.0, 2.0));
    // Construction point on both pieces, legs equal, one length.
    assert_eq!(
        counts(&s),
        map(&[("point on curve", 2), ("equal", 1), ("length", 1)])
    );
    assert_eq!(dims(&s, "length"), vec![2.0]);
    assert_eq!(dof(&s).unwrap(), 6);
    verify(&s);
}

#[test]
fn chamfer_two_distances() {
    let (mut s, a, b) = corner();
    let o = edit::chamfer_detailed(
        &mut s,
        a,
        b,
        p(1.0, 1.0),
        ChamferSpec::TwoDistances(2.0, 3.0),
    )
    .unwrap();
    let v = made(&s, &o);
    line(&v, (2.0, 0.0), (10.0, 0.0));
    line(&v, (0.0, 3.0), (0.0, 10.0));
    line(&v, (2.0, 0.0), (0.0, 3.0));
    assert_eq!(counts(&s), map(&[("point on curve", 2), ("length", 2)]));
    let mut l = dims(&s, "length");
    l.sort_by(f64::total_cmp);
    assert_eq!(l, vec![2.0, 3.0]);
    assert_eq!(dof(&s).unwrap(), 6);
    verify(&s);
}

#[test]
fn chamfer_two_distances_follow_argument_order() {
    let (mut s, a, b) = corner();
    let o = edit::chamfer_detailed(
        &mut s,
        b,
        a,
        p(1.0, 1.0),
        ChamferSpec::TwoDistances(2.0, 3.0),
    )
    .unwrap();
    // First curve is now b (the y axis): 2 along b, 3 along a.
    line(&made(&s, &o), (0.0, 2.0), (3.0, 0.0));
    verify(&s);
}

#[test]
fn chamfer_distance_angle_45_matches_equal() {
    let (mut s, a, b) = corner();
    let o = edit::chamfer_detailed(
        &mut s,
        a,
        b,
        p(1.0, 1.0),
        ChamferSpec::DistanceAngle(2.0, FRAC_PI_4),
    )
    .unwrap();
    line(&made(&s, &o), (2.0, 0.0), (0.0, 2.0));
    assert_eq!(
        counts(&s),
        map(&[("point on curve", 2), ("length", 1), ("angle", 1)])
    );
    assert!((dims(&s, "angle")[0] - FRAC_PI_4).abs() < 1e-12);
    assert_eq!(dof(&s).unwrap(), 6);
    verify(&s);
}

#[test]
fn chamfer_distance_angle_60() {
    let (mut s, a, b) = corner();
    let o = edit::chamfer_detailed(
        &mut s,
        a,
        b,
        p(1.0, 1.0),
        ChamferSpec::DistanceAngle(2.0, FRAC_PI_3),
    )
    .unwrap();
    line(&made(&s, &o), (2.0, 0.0), (0.0, 4.0 * FRAC_PI_3.sin()));
    assert_eq!(dof(&s).unwrap(), 6);
    verify(&s);
}

#[test]
fn chamfer_rectangle_corner_has_dof_4() {
    let mut s = new();
    build::rectangle_2pt(&mut s, p(0.0, 0.0), p(10.0, 6.0)).unwrap();
    let top = find_in_sketch(&s, (10.0, 6.0), (0.0, 6.0));
    let left = find_in_sketch(&s, (0.0, 6.0), (0.0, 0.0));
    let o =
        edit::chamfer_detailed(&mut s, top, left, p(1.0, 5.0), ChamferSpec::Equal(1.5)).unwrap();
    line(&made(&s, &o), (1.5, 6.0), (0.0, 4.5));
    assert_eq!(count_named(&s, "horizontal"), 2);
    assert_eq!(count_named(&s, "vertical"), 2);
    assert_eq!(dof(&s).unwrap(), 4);
    verify(&s);
}

#[test]
fn chamfer_length_edit_resolves() {
    let (mut s, a, b) = corner();
    let ends_to_fix: Vec<EntityId> = s
        .entities()
        .filter(|(id, e)| {
            matches!(e.kind, EntityKind::Point { .. })
                && s.point(*id).unwrap().distance(p(0.0, 0.0)) > 1.0
        })
        .map(|(id, _)| id)
        .collect();
    for id in ends_to_fix {
        s.add_constraint(ConstraintKind::Fix { entity: id })
            .unwrap();
    }
    s.add_constraint(ConstraintKind::Horizontal(HvTarget::Line(a)))
        .unwrap();
    s.add_constraint(ConstraintKind::Vertical(HvTarget::Line(b)))
        .unwrap();
    assert_eq!(dof(&s).unwrap(), 0);
    edit::chamfer(&mut s, a, b, p(1.0, 1.0), ChamferSpec::Equal(2.0)).unwrap();
    assert_eq!(dof(&s).unwrap(), 0);
    verify(&s);
    let lid = dim_id(&s, "length");
    s.set_dimension_value(lid, 3.0).unwrap();
    let rep = proto::solve(&mut s, &ProtoOptions::default()).unwrap();
    assert_eq!(rep.status, ProtoStatus::Converged, "{rep:?}");
    assert!(residual(&s) < 1e-9);
    line(&all_curves(&s), (3.0, 0.0), (0.0, 3.0));
}

#[test]
fn chamfer_line_arc() {
    let (mut s, l, a) = line_arc();
    assert_eq!(dof(&s).unwrap(), 7);
    let o = edit::chamfer_detailed(&mut s, l, a, p(-9.5, 1.0), ChamferSpec::Equal(2.0)).unwrap();
    let v = made(&s, &o);
    line(&v, (-8.0, 0.0), (0.0, 0.0));
    let (_, kept) = arc(&v, (0.0, 0.0), 10.0);
    assert!(has_end(&kept, (0.0, 10.0)));
    // The other cut point is 2 from the old corner (-10,0), on the circle, above the axis.
    let (_, cut) = ends(&kept);
    assert!((cut.distance(p(-10.0, 0.0)) - 2.0).abs() < 1e-9);
    assert!(cut.y > 0.0);
    assert_eq!(
        counts(&s),
        map(&[("point on curve", 2), ("equal", 1), ("length", 1)])
    );
    assert_eq!(dof(&s).unwrap(), 7);
    verify(&s);
}

#[test]
fn chamfer_arc_arc() {
    let (mut s, a1, a2) = arc_arc();
    assert_eq!(dof(&s).unwrap(), 8);
    let o = edit::chamfer_detailed(
        &mut s,
        a1,
        a2,
        p(7.5, 5.0),
        ChamferSpec::TwoDistances(2.0, 1.0),
    )
    .unwrap();
    let v = made(&s, &o);
    let x = p(7.5, 43.75_f64.sqrt());
    let (_, k1) = arc(&v, (0.0, 0.0), 10.0);
    let (_, k2) = arc(&v, (15.0, 0.0), 10.0);
    assert!(has_end(&k1, (10.0, 0.0)) && has_end(&k2, (5.0, 0.0)));
    let (_, c1) = ends(&k1);
    let (c2, _) = ends(&k2);
    assert!((c1.distance(x) - 2.0).abs() < 1e-9);
    assert!((c2.distance(x) - 1.0).abs() < 1e-9);
    assert_eq!(dof(&s).unwrap(), 8);
    verify(&s);
}

#[test]
fn chamfer_disjoint_lines() {
    let mut s = new();
    let a = s.add_line(p(1.0, 0.0), p(10.0, 0.0)).unwrap();
    let b = s.add_line(p(0.0, 1.0), p(0.0, 10.0)).unwrap();
    assert_eq!(dof(&s).unwrap(), 8);
    // The corner is the origin; distance 2 lands on both lines and ties their free ends.
    let o = edit::chamfer_detailed(&mut s, a, b, p(1.0, 1.0), ChamferSpec::Equal(2.0)).unwrap();
    line(&made(&s, &o), (2.0, 0.0), (0.0, 2.0));
    // Free ends 8 -> tied to the chamfer: X(2) + 4 + 4 - 4 constraints = 6.
    assert_eq!(dof(&s).unwrap(), 6);
    verify(&s);
}

#[test]
fn chamfer_not_applicable_cases() {
    let (mut s, a, b) = corner();
    for spec in [
        ChamferSpec::Equal(20.0),
        ChamferSpec::TwoDistances(2.0, 11.0),
        ChamferSpec::Equal(10.0),
    ] {
        let e = unchanged_on_error(&mut s, |s| {
            edit::chamfer_detailed(s, a, b, p(1.0, 1.0), spec)
        });
        assert!(
            matches!(e, SketchError::NotApplicable(_)),
            "{spec:?}: {e:?}"
        );
    }
    // An angle whose ray never reaches the second line within its extent.
    let e = unchanged_on_error(&mut s, |s| {
        edit::chamfer_detailed(
            s,
            a,
            b,
            p(1.0, 1.0),
            ChamferSpec::DistanceAngle(2.0, PI - 0.001),
        )
    });
    assert!(matches!(e, SketchError::NotApplicable(_)), "{e:?}");
    // Parallel lines.
    let mut t = new();
    let l1 = t.add_line(p(0.0, 0.0), p(10.0, 0.0)).unwrap();
    let l2 = t.add_line(p(0.0, 3.0), p(10.0, 3.0)).unwrap();
    let e = unchanged_on_error(&mut t, |s| {
        edit::chamfer_detailed(s, l1, l2, p(5.0, 1.0), ChamferSpec::Equal(1.0))
    });
    assert!(matches!(e, SketchError::NotApplicable(_)));
}

#[test]
fn chamfer_input_errors() {
    let (mut s, a, b) = corner();
    for spec in [
        ChamferSpec::Equal(0.0),
        ChamferSpec::Equal(-1.0),
        ChamferSpec::TwoDistances(1.0, f64::NAN),
        ChamferSpec::DistanceAngle(1.0, 0.0),
        ChamferSpec::DistanceAngle(1.0, PI),
        ChamferSpec::DistanceAngle(0.0, 1.0),
    ] {
        let e = unchanged_on_error(&mut s, |s| {
            edit::chamfer_detailed(s, a, b, p(1.0, 1.0), spec)
        });
        assert!(
            matches!(e, SketchError::InvalidValue { .. }),
            "{spec:?}: {e:?}"
        );
    }
    let e = unchanged_on_error(&mut s, |s| {
        edit::chamfer_detailed(s, a, a, p(1.0, 1.0), ChamferSpec::Equal(1.0))
    });
    assert!(matches!(e, SketchError::NotApplicable(_)));
}

#[test]
fn edit_result_lists_created_and_removed() {
    let (mut s, a, b) = corner();
    let before: Vec<EntityId> = s.entities().map(|(i, _)| i).collect();
    let r = edit::fillet(&mut s, a, b, p(1.0, 1.0), 2.0).unwrap();
    for id in &r.created {
        assert!(s.entity(*id).is_some() && !before.contains(id));
    }
    for id in &r.removed {
        assert!(s.entity(*id).is_none() && before.contains(id));
    }
    assert!(r.removed.contains(&a) && r.removed.contains(&b));
    assert_eq!(r.constraints_added.len(), 3);
    for id in &r.constraints_added {
        assert!(s.constraint(*id).is_some());
    }
}
