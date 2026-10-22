//! SK-07 acceptance: trim, extend and break fixtures.
//!
//! Every fixture checks the resulting entities and constraints, that the residual of the
//! edited sketch is 0 (`build_system`, the SK-02 path), and that the SK-13 prototype solver
//! (`solve::proto`, the only solver until SK-03) reports `Converged` without moving anything.

#![allow(
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::too_many_lines,
    reason = "test helpers; a failure is a test failure"
)]

use std::collections::BTreeMap;
use std::f64::consts::{FRAC_PI_2, TAU};

use amg_core::{Plane, Point2};
use amg_sketch::build;
use amg_sketch::edit::{self, EditOutcome};
use amg_sketch::solve::proto::{self, ProtoOptions, ProtoStatus};
use amg_sketch::solve::{build_system, dof};
use amg_sketch::{
    ConstraintId, ConstraintKind, Curve2, Dimension, DimensionKind, End, EntityId, EntityKind,
    EntityOrigin, ExternalLinkId, HvTarget, Side, Sketch, SketchError, TangentKind,
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
    assert!(rep.max_residual < 1e-9);
    let moved = s
        .params()
        .as_slice()
        .iter()
        .zip(t.params().as_slice())
        .fold(0.0_f64, |m, (a, b)| m.max((a - b).abs()));
    assert!(moved < 1e-6, "solver moved parameters by {moved}");
    // Every constraint and entity reference is valid (a serde round trip validates the lot).
    let json = serde_json::to_string(s).unwrap();
    let back: Sketch = serde_json::from_str(&json).unwrap();
    assert_eq!(back.entities().count(), s.entities().count());
}

fn count(s: &Sketch, name: &str) -> usize {
    s.constraints()
        .filter(|(_, c)| c.kind.name() == name)
        .count()
}

fn counts(s: &Sketch) -> BTreeMap<&'static str, usize> {
    let mut m = BTreeMap::new();
    for (_, c) in s.constraints() {
        *m.entry(c.kind.name()).or_insert(0) += 1;
    }
    m
}

fn map(v: &[(&'static str, usize)]) -> BTreeMap<&'static str, usize> {
    v.iter().copied().collect()
}

/// Curves of the created entities, sorted by start (then end) coordinates.
fn pieces(s: &Sketch, o: &EditOutcome) -> Vec<Curve2> {
    let mut v: Vec<Curve2> = o
        .result
        .created
        .iter()
        .filter_map(|&id| s.curve(id))
        .collect();
    v.sort_by(|a, b| key(a).partial_cmp(&key(b)).unwrap());
    v
}

fn key(c: &Curve2) -> [f64; 2] {
    let (t0, _) = c.param_range();
    let q = c.eval(t0);
    [q.x, q.y]
}

fn near(a: Point2, x: f64, y: f64) -> bool {
    a.distance(p(x, y)) < 1e-9
}

#[track_caller]
fn assert_line(c: &Curve2, a: (f64, f64), b: (f64, f64)) {
    let Curve2::Line(seg) = c else {
        panic!("not a line: {c:?}")
    };
    assert!(
        near(seg.a, a.0, a.1) && near(seg.b, b.0, b.1),
        "line {seg:?} != {a:?} -> {b:?}"
    );
}

#[track_caller]
fn assert_arc(c: &Curve2, center: (f64, f64), r: f64, from: (f64, f64), to: (f64, f64)) {
    let Curve2::Arc(a) = c else {
        panic!("not an arc: {c:?}")
    };
    assert!(near(a.center, center.0, center.1), "{a:?}");
    assert!((a.radius - r).abs() < 1e-9, "{a:?}");
    let s = c.eval(0.0);
    let e = c.eval(c.param_range().1);
    assert!(near(s, from.0, from.1), "start {s:?} != {from:?}");
    assert!(near(e, to.0, to.1), "end {e:?} != {to:?}");
}

fn dim(kind: DimensionKind, value: f64) -> ConstraintKind {
    ConstraintKind::Dimension(Dimension {
        kind,
        value,
        driving: true,
        label_pos: p(0.0, 0.0),
    })
}

fn line_ends(s: &Sketch, l: EntityId) -> (EntityId, EntityId) {
    match s.entity(l).unwrap().kind {
        EntityKind::Line { start, end } | EntityKind::Arc { start, end, .. } => (start, end),
        ref k => panic!("not a line/arc: {k:?}"),
    }
}

fn arc_parts(s: &Sketch, a: EntityId) -> (EntityId, EntityId, EntityId) {
    match s.entity(a).unwrap().kind {
        EntityKind::Arc { center, start, end } => (center, start, end),
        ref k => panic!("not an arc: {k:?}"),
    }
}

/// A free horizontal line and nothing else.
fn hline(s: &mut Sketch, x0: f64, x1: f64, y: f64) -> EntityId {
    s.add_line(p(x0, y), p(x1, y)).unwrap()
}

fn vline(s: &mut Sketch, x: f64, y0: f64, y1: f64) -> EntityId {
    s.add_line(p(x, y0), p(x, y1)).unwrap()
}

/// Quarter arc about the origin, radius 5, from (5,0) to (0,5).
fn quarter(s: &mut Sketch) -> EntityId {
    s.add_arc(p(0.0, 0.0), p(5.0, 0.0), p(0.0, 5.0)).unwrap()
}

fn unchanged_on_error(s: &mut Sketch, f: impl FnOnce(&mut Sketch) -> Result<(), SketchError>) {
    let before = serde_json::to_string(s).unwrap();
    let err = f(s);
    assert!(err.is_err());
    assert_eq!(serde_json::to_string(s).unwrap(), before);
}

// ======================================================================================
// Trim: lines
// ======================================================================================

#[test]
fn trim_line_end_part() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 10.0, 0.0);
    let v = vline(&mut s, 4.0, -2.0, 2.0);
    assert_eq!(dof(&s).unwrap(), 8);
    let o = edit::trim_detailed(&mut s, l, p(7.0, 0.0)).unwrap();
    let pc = pieces(&s, &o);
    assert_eq!(pc.len(), 1);
    assert_line(&pc[0], (0.0, 0.0), (4.0, 0.0));
    assert!(s.entity(l).is_none() && s.entity(v).is_some());
    assert_eq!(counts(&s), map(&[("point on curve", 1)]));
    // Removed: the line and its orphaned end point.
    assert_eq!(o.result.removed.len(), 2);
    assert_eq!(dof(&s).unwrap(), 7);
    verify(&s);
}

#[test]
fn trim_line_start_part() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 10.0, 0.0);
    vline(&mut s, 4.0, -2.0, 2.0);
    let o = edit::trim_detailed(&mut s, l, p(1.0, 0.0)).unwrap();
    let pc = pieces(&s, &o);
    assert_eq!(pc.len(), 1);
    assert_line(&pc[0], (4.0, 0.0), (10.0, 0.0));
    assert_eq!(counts(&s), map(&[("point on curve", 1)]));
    verify(&s);
}

#[test]
fn trim_line_middle_two_pieces() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 10.0, 0.0);
    vline(&mut s, 3.0, -2.0, 2.0);
    vline(&mut s, 7.0, -2.0, 2.0);
    assert_eq!(dof(&s).unwrap(), 12);
    let o = edit::trim_detailed(&mut s, l, p(5.0, 0.0)).unwrap();
    let pc = pieces(&s, &o);
    assert_eq!(pc.len(), 2);
    assert_line(&pc[0], (0.0, 0.0), (3.0, 0.0));
    assert_line(&pc[1], (7.0, 0.0), (10.0, 0.0));
    assert_eq!(counts(&s), map(&[("point on curve", 2), ("collinear", 1)]));
    // Same degrees of freedom as before: the pieces stay on one line.
    assert_eq!(dof(&s).unwrap(), 12);
    verify(&s);
}

#[test]
fn trim_line_uses_only_the_nearest_cuts() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 10.0, 0.0);
    for x in [2.0, 5.0, 8.0] {
        vline(&mut s, x, -1.0, 1.0);
    }
    let o = edit::trim_detailed(&mut s, l, p(6.0, 0.0)).unwrap();
    let pc = pieces(&s, &o);
    assert_eq!(pc.len(), 2);
    assert_line(&pc[0], (0.0, 0.0), (5.0, 0.0));
    assert_line(&pc[1], (8.0, 0.0), (10.0, 0.0));
    verify(&s);
}

#[test]
fn trim_line_without_intersections_is_deleted() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 10.0, 0.0);
    hline(&mut s, 0.0, 10.0, 5.0);
    let o = edit::trim_detailed(&mut s, l, p(3.0, 0.0)).unwrap();
    assert!(o.result.created.is_empty());
    assert_eq!(o.result.removed.len(), 3);
    assert_eq!(s.entities().count(), 3);
    verify(&s);
}

#[test]
fn trim_line_at_t_junction_glues_to_the_cutter_end() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 10.0, 0.0);
    let v = s.add_line(p(4.0, 0.0), p(4.0, 3.0)).unwrap();
    let (q, _) = line_ends(&s, v);
    let on = s
        .add_constraint(ConstraintKind::PointOnCurve { point: q, curve: l })
        .unwrap();
    let o = edit::trim_detailed(&mut s, l, p(8.0, 0.0)).unwrap();
    let pc = pieces(&s, &o);
    assert_line(&pc[0], (0.0, 0.0), (4.0, 0.0));
    // The new end is coincident with the cutter's end; the old point-on-line is implied.
    assert_eq!(counts(&s), map(&[("coincident", 1)]));
    assert!(
        o.migration
            .dropped
            .iter()
            .any(|d| d.id == on && d.reason.contains("implied"))
    );
    verify(&s);
}

#[test]
fn trim_line_keeps_horizontal() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 10.0, 1.0);
    vline(&mut s, 4.0, -2.0, 3.0);
    let h = s
        .add_constraint(ConstraintKind::Horizontal(HvTarget::Line(l)))
        .unwrap();
    let o = edit::trim_detailed(&mut s, l, p(8.0, 1.0)).unwrap();
    assert_eq!(count(&s, "horizontal"), 1);
    assert!(o.migration.migrated.iter().any(|&(old, _)| old == h));
    assert!(o.migration.dropped.is_empty());
    verify(&s);
}

#[test]
fn trim_line_drops_length_dimension_and_reports_it() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 10.0, 0.0);
    vline(&mut s, 4.0, -2.0, 2.0);
    let d = s
        .add_constraint(dim(DimensionKind::Length { line: l }, 10.0))
        .unwrap();
    let o = edit::trim_detailed(&mut s, l, p(8.0, 0.0)).unwrap();
    assert_eq!(o.migration.dropped_dimensions().count(), 1);
    assert_eq!(o.migration.dropped_dimensions().next().unwrap().id, d);
    assert!(o.result.constraints_removed.contains(&d));
    assert_eq!(count(&s, "length"), 0);
    verify(&s);
}

#[test]
fn trim_line_keeps_coincident_on_the_surviving_end() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 10.0, 0.0);
    let m = vline(&mut s, 0.0, 0.0, 5.0);
    vline(&mut s, 4.0, -2.0, 2.0);
    let (l0, _) = line_ends(&s, l);
    let (m0, _) = line_ends(&s, m);
    let c = s
        .add_constraint(ConstraintKind::Coincident { a: l0, b: m0 })
        .unwrap();
    let o = edit::trim_detailed(&mut s, l, p(8.0, 0.0)).unwrap();
    // The constraint is on points that survive, so it is untouched.
    assert!(s.constraint(c).is_some());
    assert!(!o.result.constraints_removed.contains(&c));
    verify(&s);
}

#[test]
fn trim_line_drops_coincident_on_the_removed_end() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 10.0, 0.0);
    let n = vline(&mut s, 10.0, 0.0, 5.0);
    vline(&mut s, 4.0, -2.0, 2.0);
    let (_, l1) = line_ends(&s, l);
    let (n0, _) = line_ends(&s, n);
    let c = s
        .add_constraint(ConstraintKind::Coincident { a: l1, b: n0 })
        .unwrap();
    let o = edit::trim_detailed(&mut s, l, p(8.0, 0.0)).unwrap();
    assert!(s.constraint(c).is_none());
    assert!(o.migration.dropped.iter().any(|d| d.id == c));
    assert!(s.entity(n).is_some());
    verify(&s);
}

#[test]
fn trim_line_keeps_point_line_distance() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 10.0, 0.0);
    vline(&mut s, 4.0, -2.0, 2.0);
    let q = s.add_point(p(2.0, 3.0)).unwrap();
    let d = s
        .add_constraint_oriented(dim(
            DimensionKind::Distance {
                a: q,
                b: l,
                side: Side::Positive,
            },
            3.0,
        ))
        .unwrap();
    let o = edit::trim_detailed(&mut s, l, p(8.0, 0.0)).unwrap();
    assert_eq!(count(&s, "distance"), 1);
    assert!(o.migration.migrated.iter().any(|&(old, _)| old == d));
    verify(&s);
}

#[test]
fn trim_line_keeps_perpendicular() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 10.0, 0.0);
    let m = vline(&mut s, 4.0, -2.0, 2.0);
    s.add_constraint(ConstraintKind::Perpendicular { a: l, b: m })
        .unwrap();
    edit::trim(&mut s, l, p(8.0, 0.0)).unwrap();
    assert_eq!(count(&s, "perpendicular"), 1);
    verify(&s);
}

#[test]
fn trim_line_by_circle_removes_the_inside() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 10.0, 0.0);
    let c = s.add_circle(p(5.0, 0.0), 2.0).unwrap();
    let o = edit::trim_detailed(&mut s, l, p(5.0, 0.0)).unwrap();
    let pc = pieces(&s, &o);
    assert_eq!(pc.len(), 2);
    assert_line(&pc[0], (0.0, 0.0), (3.0, 0.0));
    assert_line(&pc[1], (7.0, 0.0), (10.0, 0.0));
    assert_eq!(count(&s, "point on curve"), 2);
    assert!(s.entity(c).is_some());
    verify(&s);
}

#[test]
fn trim_line_by_arc_cutter() {
    let build_it = |pick_x: f64| {
        let mut s = new();
        let l = hline(&mut s, 0.0, 10.0, 0.0);
        // Right half circle about (5,0): crosses the line only at (7,0).
        s.add_arc(p(5.0, 0.0), p(5.0, -2.0), p(5.0, 2.0)).unwrap();
        let o = edit::trim_detailed(&mut s, l, p(pick_x, 0.0)).unwrap();
        (s, o)
    };
    let (s, o) = build_it(9.0);
    let pc = pieces(&s, &o);
    assert_eq!(pc.len(), 1);
    assert_line(&pc[0], (0.0, 0.0), (7.0, 0.0));
    verify(&s);
    let (s, o) = build_it(3.0);
    assert_line(&pieces(&s, &o)[0], (7.0, 0.0), (10.0, 0.0));
    verify(&s);
}

#[test]
fn trim_line_at_tangent_contact() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 10.0, 0.0);
    s.add_circle(p(5.0, 2.0), 2.0).unwrap();
    let o = edit::trim_detailed(&mut s, l, p(8.0, 0.0)).unwrap();
    let pc = pieces(&s, &o);
    assert_eq!(pc.len(), 1);
    assert_line(&pc[0], (0.0, 0.0), (5.0, 0.0));
    verify(&s);
}

#[test]
fn trim_line_ignores_overlapping_cutters() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 10.0, 0.0);
    hline(&mut s, 2.0, 12.0, 0.0);
    vline(&mut s, 6.0, -1.0, 1.0);
    let o = edit::trim_detailed(&mut s, l, p(8.0, 0.0)).unwrap();
    let pc = pieces(&s, &o);
    assert_eq!(pc.len(), 1);
    assert_line(&pc[0], (0.0, 0.0), (6.0, 0.0));
    verify(&s);
}

#[test]
fn trim_rectangle_edge_keeps_the_corner_and_horizontal() {
    let mut s = new();
    let r = build::rectangle_2pt(&mut s, p(0.0, 0.0), p(10.0, 6.0)).unwrap();
    let bottom = r
        .entities
        .iter()
        .copied()
        .find(|&e| {
            matches!(s.curve(e), Some(Curve2::Line(seg))
                if near(seg.a, 0.0, 0.0) && near(seg.b, 10.0, 0.0))
        })
        .unwrap();
    vline(&mut s, 4.0, -2.0, 8.0);
    let before = s.entities().count();
    let o = edit::trim_detailed(&mut s, bottom, p(7.0, 0.0)).unwrap();
    let pc = pieces(&s, &o);
    assert_eq!(pc.len(), 1);
    assert_line(&pc[0], (0.0, 0.0), (4.0, 0.0));
    // The right-hand corner belongs to the right edge and stays.
    assert_eq!(o.result.removed.len(), 1);
    assert_eq!(s.entities().count(), before + 1);
    assert_eq!(count(&s, "horizontal"), 2);
    assert_eq!(count(&s, "vertical"), 2);
    verify(&s);
}

#[test]
fn trim_line_keeps_tangent_at_the_surviving_end() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 10.0, 0.0);
    vline(&mut s, 4.0, -2.0, 2.0);
    build::arc_tangent(&mut s, l, End::End, p(15.0, 5.0)).unwrap();
    assert_eq!(count(&s, "tangent"), 1);
    let o = edit::trim_detailed(&mut s, l, p(1.0, 0.0)).unwrap();
    assert_eq!(count(&s, "tangent"), 1);
    assert!(o.migration.dropped.is_empty());
    let (_, c) = s
        .constraints()
        .find(|(_, c)| c.kind.name() == "tangent")
        .unwrap();
    assert!(matches!(
        c.kind,
        ConstraintKind::Tangent {
            kind: TangentKind::AtEndpoint { .. },
            ..
        }
    ));
    verify(&s);
}

#[test]
fn trim_line_drops_tangent_at_the_cut_off_end() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 10.0, 0.0);
    vline(&mut s, 4.0, -2.0, 2.0);
    build::arc_tangent(&mut s, l, End::End, p(15.0, 5.0)).unwrap();
    let o = edit::trim_detailed(&mut s, l, p(8.0, 0.0)).unwrap();
    assert_eq!(count(&s, "tangent"), 0);
    assert!(
        o.migration
            .dropped
            .iter()
            .any(|d| matches!(d.kind, ConstraintKind::Tangent { .. }))
    );
    verify(&s);
}

// ======================================================================================
// Trim: arcs
// ======================================================================================

#[test]
fn trim_arc_end_part() {
    let mut s = new();
    let a = quarter(&mut s);
    vline(&mut s, 3.0, -1.0, 6.0);
    let o = edit::trim_detailed(&mut s, a, p(0.0, 5.0)).unwrap();
    let pc = pieces(&s, &o);
    assert_eq!(pc.len(), 1);
    assert_arc(&pc[0], (0.0, 0.0), 5.0, (5.0, 0.0), (3.0, 4.0));
    assert_eq!(counts(&s), map(&[("point on curve", 1)]));
    verify(&s);
}

#[test]
fn trim_arc_start_part() {
    let mut s = new();
    let a = quarter(&mut s);
    vline(&mut s, 3.0, -1.0, 6.0);
    let o = edit::trim_detailed(&mut s, a, p(5.0, 0.1)).unwrap();
    let pc = pieces(&s, &o);
    assert_eq!(pc.len(), 1);
    assert_arc(&pc[0], (0.0, 0.0), 5.0, (3.0, 4.0), (0.0, 5.0));
    verify(&s);
}

#[test]
fn trim_arc_middle_two_pieces() {
    let mut s = new();
    let a = quarter(&mut s);
    vline(&mut s, 2.0, -1.0, 6.0);
    vline(&mut s, 4.0, -1.0, 6.0);
    assert_eq!(dof(&s).unwrap(), 13);
    let o = edit::trim_detailed(&mut s, a, p(3.0, 4.0)).unwrap();
    let pc = pieces(&s, &o);
    assert_eq!(pc.len(), 2);
    let y2 = 21.0_f64.sqrt();
    assert_arc(&pc[0], (0.0, 0.0), 5.0, (2.0, y2), (0.0, 5.0));
    assert_arc(&pc[1], (0.0, 0.0), 5.0, (5.0, 0.0), (4.0, 3.0));
    assert_eq!(counts(&s), map(&[("point on curve", 2), ("equal", 1)]));
    assert_eq!(dof(&s).unwrap(), 13);
    verify(&s);
}

#[test]
fn trim_arc_without_intersections_is_deleted() {
    let mut s = new();
    let a = quarter(&mut s);
    vline(&mut s, 8.0, 0.0, 5.0);
    let o = edit::trim_detailed(&mut s, a, p(3.0, 4.0)).unwrap();
    assert!(o.result.created.is_empty());
    assert_eq!(o.result.removed.len(), 4);
    assert_eq!(s.entities().count(), 3);
    verify(&s);
}

#[test]
fn trim_arc_keeps_radius_and_drops_arc_length() {
    let mut s = new();
    let a = quarter(&mut s);
    vline(&mut s, 3.0, -1.0, 6.0);
    let r = s
        .add_constraint(dim(DimensionKind::Radius { curve: a }, 5.0))
        .unwrap();
    let al = s
        .add_constraint(dim(DimensionKind::ArcLength { arc: a }, 5.0 * FRAC_PI_2))
        .unwrap();
    let o = edit::trim_detailed(&mut s, a, p(0.0, 5.0)).unwrap();
    assert_eq!(count(&s, "radius"), 1);
    assert_eq!(count(&s, "arc length"), 0);
    assert!(o.migration.migrated.iter().any(|&(old, _)| old == r));
    assert_eq!(o.migration.dropped_dimensions().next().unwrap().id, al);
    verify(&s);
}

#[test]
fn trim_arc_keeps_concentric_and_equal_radius() {
    let mut s = new();
    let a = quarter(&mut s);
    vline(&mut s, 3.0, -1.0, 6.0);
    let c1 = s.add_circle(p(0.0, 0.0), 5.0).unwrap();
    let c2 = s.add_circle(p(20.0, 0.0), 5.0).unwrap();
    s.add_constraint(ConstraintKind::Concentric { a, b: c1 })
        .unwrap();
    s.add_constraint(ConstraintKind::Equal { a, b: c2 })
        .unwrap();
    let o = edit::trim_detailed(&mut s, a, p(0.0, 5.0)).unwrap();
    assert_eq!(count(&s, "concentric"), 1);
    assert_eq!(count(&s, "equal"), 1);
    assert!(o.migration.dropped.is_empty());
    verify(&s);
}

#[test]
fn trim_arc_drops_equal_length_to_a_line() {
    let mut s = new();
    let a = quarter(&mut s);
    vline(&mut s, 3.0, -1.0, 6.0);
    let m = hline(&mut s, 10.0, 10.0 + 5.0 * FRAC_PI_2, 8.0);
    s.add_constraint(ConstraintKind::Equal { a, b: m }).unwrap();
    let o = edit::trim_detailed(&mut s, a, p(0.0, 5.0)).unwrap();
    assert_eq!(count(&s, "equal"), 0);
    assert_eq!(o.migration.dropped.len(), 1);
    verify(&s);
}

#[test]
fn trim_arc_keeps_coincident_on_its_centre() {
    let mut s = new();
    let a = quarter(&mut s);
    vline(&mut s, 3.0, -1.0, 6.0);
    let origin = s.add_point(p(0.0, 0.0)).unwrap();
    let (c, _, _) = arc_parts(&s, a);
    let k = s
        .add_constraint(ConstraintKind::Coincident { a: c, b: origin })
        .unwrap();
    edit::trim(&mut s, a, p(0.0, 5.0)).unwrap();
    assert!(s.constraint(k).is_some());
    verify(&s);
}

#[test]
fn trim_arc_by_circle_cutter() {
    let mut s = new();
    let a = quarter(&mut s);
    s.add_circle(p(5.0, 0.0), 5.0).unwrap();
    let o = edit::trim_detailed(&mut s, a, p(5.0, 0.1)).unwrap();
    let pc = pieces(&s, &o);
    assert_eq!(pc.len(), 1);
    assert_arc(
        &pc[0],
        (0.0, 0.0),
        5.0,
        (2.5, 75.0_f64.sqrt() / 2.0),
        (0.0, 5.0),
    );
    verify(&s);
}

// ======================================================================================
// Trim: circles
// ======================================================================================

#[test]
fn trim_circle_becomes_an_arc() {
    let mut s = new();
    let c = s.add_circle(p(0.0, 0.0), 5.0).unwrap();
    vline(&mut s, 3.0, -6.0, 6.0);
    let o = edit::trim_detailed(&mut s, c, p(5.0, 0.0)).unwrap();
    let pc = pieces(&s, &o);
    assert_eq!(pc.len(), 1);
    assert_arc(&pc[0], (0.0, 0.0), 5.0, (3.0, 4.0), (3.0, -4.0));
    let Curve2::Arc(arc) = &pc[0] else { panic!() };
    assert!((arc.sweep - (TAU - 2.0 * 4.0_f64.atan2(3.0))).abs() < 1e-9);
    assert_eq!(counts(&s), map(&[("point on curve", 2)]));
    assert_eq!(dof(&s).unwrap(), 4 + 3);
    verify(&s);
}

#[test]
fn trim_circle_with_four_cuts_uses_the_nearest_two() {
    let mut s = new();
    let c = s.add_circle(p(0.0, 0.0), 5.0).unwrap();
    vline(&mut s, 3.0, -6.0, 6.0);
    vline(&mut s, -3.0, -6.0, 6.0);
    let o = edit::trim_detailed(&mut s, c, p(5.0, 0.0)).unwrap();
    let pc = pieces(&s, &o);
    assert_eq!(pc.len(), 1);
    assert_arc(&pc[0], (0.0, 0.0), 5.0, (3.0, 4.0), (3.0, -4.0));
    verify(&s);
    // A pick on the far side removes the other cap.
    let mut s = new();
    let c = s.add_circle(p(0.0, 0.0), 5.0).unwrap();
    vline(&mut s, 3.0, -6.0, 6.0);
    vline(&mut s, -3.0, -6.0, 6.0);
    let o = edit::trim_detailed(&mut s, c, p(0.0, 5.0)).unwrap();
    assert_arc(&pieces(&s, &o)[0], (0.0, 0.0), 5.0, (-3.0, 4.0), (3.0, 4.0));
    verify(&s);
}

#[test]
fn trim_circle_with_one_intersection_is_not_applicable() {
    let mut s = new();
    let c = s.add_circle(p(0.0, 0.0), 5.0).unwrap();
    vline(&mut s, 5.0, -2.0, 2.0);
    unchanged_on_error(&mut s, |s| edit::trim(s, c, p(0.0, 5.0)).map(|_| ()));
    assert!(matches!(
        edit::trim(&mut s, c, p(0.0, 5.0)),
        Err(SketchError::NotApplicable(_))
    ));
}

#[test]
fn trim_circle_keeps_radius_and_diameter() {
    let mut s = new();
    let c = s.add_circle(p(0.0, 0.0), 5.0).unwrap();
    vline(&mut s, 3.0, -6.0, 6.0);
    s.add_constraint(dim(DimensionKind::Radius { curve: c }, 5.0))
        .unwrap();
    let o = edit::trim_detailed(&mut s, c, p(5.0, 0.0)).unwrap();
    assert_eq!(count(&s, "radius"), 1);
    assert!(o.migration.dropped.is_empty());
    verify(&s);
}

#[test]
fn trim_circle_keeps_tangent_line() {
    let mut s = new();
    let c = s.add_circle(p(0.0, 0.0), 5.0).unwrap();
    vline(&mut s, 3.0, -6.0, 6.0);
    let t = hline(&mut s, -8.0, 8.0, 5.0);
    s.add_constraint_oriented(ConstraintKind::Tangent {
        a: t,
        b: c,
        kind: TangentKind::External,
    })
    .unwrap();
    let o = edit::trim_detailed(&mut s, c, p(5.0, 0.0)).unwrap();
    assert_eq!(count(&s, "tangent"), 1);
    assert!(o.migration.dropped.is_empty());
    verify(&s);
}

#[test]
fn trim_circle_splits_point_on_curve_constraints() {
    let mut s = new();
    let c = s.add_circle(p(0.0, 0.0), 5.0).unwrap();
    vline(&mut s, 3.0, -6.0, 6.0);
    let keep = s.add_point(p(-5.0, 0.0)).unwrap();
    let gone = s.add_point(p(5.0, 0.0)).unwrap();
    let k1 = s
        .add_constraint(ConstraintKind::PointOnCurve {
            point: keep,
            curve: c,
        })
        .unwrap();
    let k2 = s
        .add_constraint(ConstraintKind::PointOnCurve {
            point: gone,
            curve: c,
        })
        .unwrap();
    let o = edit::trim_detailed(&mut s, c, p(5.0, 0.0)).unwrap();
    assert!(o.migration.migrated.iter().any(|&(old, _)| old == k1));
    assert!(o.migration.dropped.iter().any(|d| d.id == k2));
    verify(&s);
}

#[test]
fn trim_circle_keeps_centre_coincident() {
    let mut s = new();
    let c = s.add_circle(p(0.0, 0.0), 5.0).unwrap();
    vline(&mut s, 3.0, -6.0, 6.0);
    let origin = s.add_point(p(0.0, 0.0)).unwrap();
    let EntityKind::Circle { center, .. } = s.entity(c).unwrap().kind else {
        panic!()
    };
    let k = s
        .add_constraint(ConstraintKind::Coincident {
            a: center,
            b: origin,
        })
        .unwrap();
    edit::trim(&mut s, c, p(5.0, 0.0)).unwrap();
    assert!(s.constraint(k).is_some());
    verify(&s);
}

// ======================================================================================
// Extend
// ======================================================================================

#[test]
fn extend_line_end() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 6.0, 0.0);
    let v = vline(&mut s, 10.0, -2.0, 2.0);
    let o = edit::extend_detailed(&mut s, l, p(5.0, 0.0)).unwrap();
    let pc = pieces(&s, &o);
    assert_eq!(pc.len(), 1);
    assert_line(&pc[0], (0.0, 0.0), (10.0, 0.0));
    assert_eq!(counts(&s), map(&[("point on curve", 1)]));
    assert!(s.entity(v).is_some());
    assert_eq!(dof(&s).unwrap(), 7);
    verify(&s);
}

#[test]
fn extend_line_start() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 6.0, 0.0);
    vline(&mut s, -4.0, -2.0, 2.0);
    let o = edit::extend_detailed(&mut s, l, p(1.0, 0.0)).unwrap();
    assert_line(&pieces(&s, &o)[0], (-4.0, 0.0), (6.0, 0.0));
    verify(&s);
}

#[test]
fn extend_goes_to_the_nearest_cutter() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 6.0, 0.0);
    vline(&mut s, 12.0, -2.0, 2.0);
    vline(&mut s, 8.0, -2.0, 2.0);
    let o = edit::extend_detailed(&mut s, l, p(6.0, 0.0)).unwrap();
    assert_line(&pieces(&s, &o)[0], (0.0, 0.0), (8.0, 0.0));
    verify(&s);
}

#[test]
fn extend_picks_the_end_nearest_to_the_pick() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 6.0, 0.0);
    vline(&mut s, -4.0, -2.0, 2.0);
    vline(&mut s, 9.0, -2.0, 2.0);
    let o = edit::extend_detailed(&mut s, l, p(5.0, 1.0)).unwrap();
    assert_line(&pieces(&s, &o)[0], (0.0, 0.0), (9.0, 0.0));
    verify(&s);
}

#[test]
fn extend_without_target_is_not_applicable() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 6.0, 0.0);
    vline(&mut s, -4.0, -2.0, 2.0);
    vline(&mut s, 9.0, 1.0, 3.0);
    unchanged_on_error(&mut s, |s| edit::extend(s, l, p(6.0, 0.0)).map(|_| ()));
}

#[test]
fn extend_line_to_a_circle_uses_the_near_side() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 6.0, 0.0);
    s.add_circle(p(10.0, 0.0), 2.0).unwrap();
    let o = edit::extend_detailed(&mut s, l, p(6.0, 0.0)).unwrap();
    assert_line(&pieces(&s, &o)[0], (0.0, 0.0), (8.0, 0.0));
    verify(&s);
}

#[test]
fn extend_line_to_an_arc_only_where_the_arc_is() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 6.0, 0.0);
    // Right half circle about (10,0): its near side (8,0) is not on the arc.
    s.add_arc(p(10.0, 0.0), p(10.0, -2.0), p(10.0, 2.0))
        .unwrap();
    let o = edit::extend_detailed(&mut s, l, p(6.0, 0.0)).unwrap();
    assert_line(&pieces(&s, &o)[0], (0.0, 0.0), (12.0, 0.0));
    verify(&s);
}

#[test]
fn extend_arc_end_along_its_circle() {
    let mut s = new();
    let a = s.add_arc(p(0.0, 0.0), p(5.0, 0.0), p(3.0, 4.0)).unwrap();
    vline(&mut s, -1.0, -6.0, 6.0);
    let o = edit::extend_detailed(&mut s, a, p(3.0, 4.0)).unwrap();
    let pc = pieces(&s, &o);
    assert_eq!(pc.len(), 1);
    assert_arc(&pc[0], (0.0, 0.0), 5.0, (5.0, 0.0), (-1.0, 24.0_f64.sqrt()));
    assert_eq!(counts(&s), map(&[("point on curve", 1)]));
    verify(&s);
}

#[test]
fn extend_arc_start_backwards() {
    let mut s = new();
    let a = s.add_arc(p(0.0, 0.0), p(0.0, 5.0), p(-5.0, 0.0)).unwrap();
    vline(&mut s, 3.0, -6.0, 6.0);
    let o = edit::extend_detailed(&mut s, a, p(0.1, 5.0)).unwrap();
    let pc = pieces(&s, &o);
    assert_arc(&pc[0], (0.0, 0.0), 5.0, (3.0, 4.0), (-5.0, 0.0));
    verify(&s);
}

#[test]
fn extend_arc_never_passes_its_own_other_end() {
    let mut s = new();
    let a = s.add_arc(p(0.0, 0.0), p(5.0, 0.0), p(3.0, 4.0)).unwrap();
    // Crosses the circle only inside the existing arc.
    vline(&mut s, 4.0, 0.0, 6.0);
    unchanged_on_error(&mut s, |s| edit::extend(s, a, p(3.0, 4.0)).map(|_| ()));
}

#[test]
fn extend_circle_is_not_applicable() {
    let mut s = new();
    let c = s.add_circle(p(0.0, 0.0), 5.0).unwrap();
    vline(&mut s, 3.0, -6.0, 6.0);
    unchanged_on_error(&mut s, |s| edit::extend(s, c, p(5.0, 0.0)).map(|_| ()));
}

#[test]
fn extend_keeps_horizontal_and_drops_length() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 6.0, 1.0);
    vline(&mut s, 10.0, -2.0, 3.0);
    s.add_constraint(ConstraintKind::Horizontal(HvTarget::Line(l)))
        .unwrap();
    let d = s
        .add_constraint(dim(DimensionKind::Length { line: l }, 6.0))
        .unwrap();
    let o = edit::extend_detailed(&mut s, l, p(6.0, 1.0)).unwrap();
    assert_eq!(count(&s, "horizontal"), 1);
    assert_eq!(count(&s, "length"), 0);
    assert_eq!(o.migration.dropped_dimensions().next().unwrap().id, d);
    verify(&s);
}

#[test]
fn extend_glues_to_the_cutter_end_when_it_lies_there() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 6.0, 0.0);
    s.add_line(p(10.0, 0.0), p(10.0, 3.0)).unwrap();
    let o = edit::extend_detailed(&mut s, l, p(6.0, 0.0)).unwrap();
    assert_line(&pieces(&s, &o)[0], (0.0, 0.0), (10.0, 0.0));
    assert_eq!(counts(&s), map(&[("coincident", 1)]));
    verify(&s);
}

#[test]
fn extend_keeps_the_start_coincident_and_perpendicular() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 6.0, 0.0);
    let m = vline(&mut s, 0.0, 0.0, 4.0);
    vline(&mut s, 10.0, -2.0, 2.0);
    let (l0, _) = line_ends(&s, l);
    let (m0, _) = line_ends(&s, m);
    let c = s
        .add_constraint(ConstraintKind::Coincident { a: l0, b: m0 })
        .unwrap();
    s.add_constraint(ConstraintKind::Perpendicular { a: l, b: m })
        .unwrap();
    let o = edit::extend_detailed(&mut s, l, p(6.0, 0.0)).unwrap();
    assert!(s.constraint(c).is_some());
    assert_eq!(count(&s, "perpendicular"), 1);
    assert!(o.migration.dropped.is_empty());
    verify(&s);
}

#[test]
fn extend_arc_keeps_radius() {
    let mut s = new();
    let a = s.add_arc(p(0.0, 0.0), p(5.0, 0.0), p(3.0, 4.0)).unwrap();
    vline(&mut s, -1.0, -6.0, 6.0);
    s.add_constraint(dim(DimensionKind::Radius { curve: a }, 5.0))
        .unwrap();
    edit::extend(&mut s, a, p(3.0, 4.0)).unwrap();
    assert_eq!(count(&s, "radius"), 1);
    verify(&s);
}

#[test]
fn extend_start_keeps_the_tangent_at_the_end() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 6.0, 0.0);
    build::arc_tangent(&mut s, l, End::End, p(10.0, 4.0)).unwrap();
    vline(&mut s, -3.0, -2.0, 2.0);
    let o = edit::extend_detailed(&mut s, l, p(0.5, 0.0)).unwrap();
    // The start was extended; the tangent is at the end, which is kept.
    assert_eq!(count(&s, "tangent"), 1);
    assert!(o.migration.dropped.is_empty());
    verify(&s);
}

// ======================================================================================
// Break
// ======================================================================================

#[test]
fn break_line_shares_the_new_point() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 10.0, 0.0);
    let o = edit::break_at_detailed(&mut s, l, p(5.0, 0.3)).unwrap();
    let pc = pieces(&s, &o);
    assert_eq!(pc.len(), 2);
    assert_line(&pc[0], (0.0, 0.0), (5.0, 0.0));
    assert_line(&pc[1], (5.0, 0.0), (10.0, 0.0));
    let new_lines: Vec<EntityId> = o
        .result
        .created
        .iter()
        .copied()
        .filter(|&e| s.curve(e).is_some())
        .collect();
    assert_eq!(line_ends(&s, new_lines[0]).1, line_ends(&s, new_lines[1]).0);
    assert_eq!(counts(&s), map(&[("parallel", 1)]));
    // Three free points, collinear.
    assert_eq!(dof(&s).unwrap(), 5);
    assert_eq!(o.result.removed, vec![l]);
    verify(&s);
}

#[test]
fn break_line_keeps_horizontal_on_the_first_piece() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 10.0, 2.0);
    s.add_constraint(ConstraintKind::Horizontal(HvTarget::Line(l)))
        .unwrap();
    edit::break_at(&mut s, l, p(4.0, 2.0)).unwrap();
    assert_eq!(counts(&s), map(&[("horizontal", 1), ("parallel", 1)]));
    verify(&s);
}

#[test]
fn break_arc_shares_centre_and_point() {
    let mut s = new();
    let a = quarter(&mut s);
    let o = edit::break_at_detailed(&mut s, a, p(3.0, 4.0)).unwrap();
    let pc = pieces(&s, &o);
    assert_eq!(pc.len(), 2);
    assert_arc(&pc[0], (0.0, 0.0), 5.0, (3.0, 4.0), (0.0, 5.0));
    assert_arc(&pc[1], (0.0, 0.0), 5.0, (5.0, 0.0), (3.0, 4.0));
    // The shared centre and joint keep the radii equal: no extra constraint.
    assert_eq!(s.constraints().count(), 0);
    assert_eq!(dof(&s).unwrap(), 6);
    verify(&s);
}

#[test]
fn break_at_an_end_is_degenerate() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 10.0, 0.0);
    unchanged_on_error(&mut s, |s| edit::break_at(s, l, p(-3.0, 0.0)).map(|_| ()));
    assert!(matches!(
        edit::break_at(&mut s, l, p(10.0, 4.0)),
        Err(SketchError::Degenerate(_))
    ));
}

#[test]
fn break_circle_is_not_applicable() {
    let mut s = new();
    let c = s.add_circle(p(0.0, 0.0), 5.0).unwrap();
    unchanged_on_error(&mut s, |s| edit::break_at(s, c, p(5.0, 0.0)).map(|_| ()));
}

#[test]
fn break_splits_point_on_curve_constraints_by_position() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 10.0, 0.0);
    let a = s.add_point(p(2.0, 0.0)).unwrap();
    let b = s.add_point(p(8.0, 0.0)).unwrap();
    for q in [a, b] {
        s.add_constraint(ConstraintKind::PointOnCurve { point: q, curve: l })
            .unwrap();
    }
    let o = edit::break_at_detailed(&mut s, l, p(5.0, 0.0)).unwrap();
    assert!(o.migration.dropped.is_empty());
    let mut owners: Vec<EntityId> = s
        .constraints()
        .filter_map(|(_, c)| match c.kind {
            ConstraintKind::PointOnCurve { curve, .. } => Some(curve),
            _ => None,
        })
        .collect();
    owners.sort();
    owners.dedup();
    assert_eq!(owners.len(), 2, "one point on each piece");
    verify(&s);
}

#[test]
fn break_drops_midpoint_and_length() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 10.0, 0.0);
    let m = s.add_point(p(5.0, 0.0)).unwrap();
    s.add_constraint(ConstraintKind::Midpoint { point: m, line: l })
        .unwrap();
    s.add_constraint(dim(DimensionKind::Length { line: l }, 10.0))
        .unwrap();
    let o = edit::break_at_detailed(&mut s, l, p(3.0, 0.0)).unwrap();
    assert_eq!(o.migration.dropped.len(), 2);
    assert_eq!(o.migration.dropped_dimensions().count(), 1);
    verify(&s);
}

#[test]
fn break_keeps_coincident_at_both_ends() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 10.0, 0.0);
    let m = vline(&mut s, 0.0, 0.0, 3.0);
    let n = vline(&mut s, 10.0, 0.0, 3.0);
    let (l0, l1) = line_ends(&s, l);
    let (m0, _) = line_ends(&s, m);
    let (n0, _) = line_ends(&s, n);
    let c1 = s
        .add_constraint(ConstraintKind::Coincident { a: l0, b: m0 })
        .unwrap();
    let c2 = s
        .add_constraint(ConstraintKind::Coincident { a: l1, b: n0 })
        .unwrap();
    edit::break_at(&mut s, l, p(5.0, 0.0)).unwrap();
    assert!(s.constraint(c1).is_some() && s.constraint(c2).is_some());
    verify(&s);
}

#[test]
fn break_moves_the_end_tangent_to_the_second_piece() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 10.0, 0.0);
    build::arc_tangent(&mut s, l, End::End, p(15.0, 5.0)).unwrap();
    let o = edit::break_at_detailed(&mut s, l, p(4.0, 0.0)).unwrap();
    assert_eq!(count(&s, "tangent"), 1);
    assert!(o.migration.dropped.is_empty());
    let (_, c) = s
        .constraints()
        .find(|(_, c)| c.kind.name() == "tangent")
        .unwrap();
    let ConstraintKind::Tangent { a, .. } = c.kind else {
        panic!()
    };
    // It is on the piece that ends at the old end point (x from 4 to 10).
    assert!(matches!(s.curve(a), Some(Curve2::Line(seg)) if near(seg.a, 4.0, 0.0)));
    verify(&s);
}

#[test]
fn break_arc_keeps_radius() {
    let mut s = new();
    let a = quarter(&mut s);
    s.add_constraint(dim(DimensionKind::Radius { curve: a }, 5.0))
        .unwrap();
    edit::break_at(&mut s, a, p(3.0, 4.0)).unwrap();
    assert_eq!(count(&s, "radius"), 1);
    verify(&s);
}

// ======================================================================================
// Errors and rollback
// ======================================================================================

#[test]
fn rejects_unknown_points_and_projected_entities() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 10.0, 0.0);
    let (pt, _) = line_ends(&s, l);
    assert_eq!(
        edit::trim(&mut s, EntityId(99), p(0.0, 0.0)).unwrap_err(),
        SketchError::UnknownEntity(EntityId(99))
    );
    for r in [
        edit::trim(&mut s, pt, p(0.0, 0.0)),
        edit::extend(&mut s, pt, p(0.0, 0.0)),
        edit::break_at(&mut s, pt, p(0.0, 0.0)),
    ] {
        assert!(matches!(r, Err(SketchError::WrongEntityKind { .. })));
    }
    let a = s.add_point(p(0.0, 3.0)).unwrap();
    let b = s.add_point(p(10.0, 3.0)).unwrap();
    let proj = s
        .add_entity(
            EntityKind::Line { start: a, end: b },
            false,
            EntityOrigin::Projected(ExternalLinkId(0)),
        )
        .unwrap();
    assert_eq!(
        edit::break_at(&mut s, proj, p(5.0, 3.0)).unwrap_err(),
        SketchError::Locked(proj)
    );
}

#[test]
fn constraint_ids_in_the_result_exist_or_are_gone() {
    let mut s = new();
    let l = hline(&mut s, 0.0, 10.0, 0.0);
    vline(&mut s, 3.0, -2.0, 2.0);
    vline(&mut s, 7.0, -2.0, 2.0);
    s.add_constraint(ConstraintKind::Horizontal(HvTarget::Line(l)))
        .unwrap();
    let o = edit::trim(&mut s, l, p(5.0, 0.0)).unwrap();
    for c in &o.constraints_added {
        assert!(s.constraint(*c).is_some());
    }
    for c in &o.constraints_removed {
        assert!(s.constraint(*c).is_none());
    }
    for e in &o.created {
        assert!(s.entity(*e).is_some());
    }
    for e in &o.removed {
        assert!(s.entity(*e).is_none());
    }
    let _: Vec<ConstraintId> = o.constraints_added;
}
