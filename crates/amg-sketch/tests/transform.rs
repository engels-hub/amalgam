//! SK-18 acceptance: mirror, rectangular/circular pattern, move/copy/rotate/scale.
//!
//! 1. A constrained mirror/pattern follows its source when the source is edited (re-solve test
//!    with the SK-13 prototype solver; production solver SK-03 is not built yet).
//! 2. An unconstrained copy is independent.
//! 3. Scale about a point scales dimension values.
//!
//! Constraint sets and DOF are checked with the SK-02 rank helper.

#![allow(
    clippy::unwrap_used,
    clippy::float_cmp,
    reason = "test helpers; a failure is a test failure"
)]

use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, FRAC_PI_6, PI, TAU};

use amg_core::{Plane, Point2, Transform2, Vec2};
use amg_sketch::build;
use amg_sketch::solve::proto::{self, ProtoOptions, ProtoStatus};
use amg_sketch::solve::{build_system, dof};
use amg_sketch::transform::{
    EditResult, mirror, pattern_circ, pattern_rect, rotation_about, scale_about, transform,
    translation,
};
use amg_sketch::{
    ConstraintId, ConstraintKind, Dimension, DimensionKind, EntityId, EntityKind, HvTarget, Sketch,
    SketchError,
};

fn new() -> Sketch {
    Sketch::new(Plane::XY)
}

fn p(x: f64, y: f64) -> Point2 {
    Point2::new(x, y)
}

fn residual(s: &Sketch) -> f64 {
    let sys = build_system(s).unwrap();
    sys.system.max_abs_residual(s.params().as_slice())
}

fn pos(s: &Sketch, id: EntityId) -> Point2 {
    s.point(id).unwrap()
}

fn close(a: Point2, b: Point2) -> bool {
    a.distance(b) < 1e-6
}

fn dim(s: &mut Sketch, kind: DimensionKind, value: f64) -> ConstraintId {
    s.add_constraint_oriented(ConstraintKind::Dimension(Dimension {
        kind,
        value,
        driving: true,
        label_pos: Point2::ZERO,
    }))
    .unwrap()
}

fn solve(s: &mut Sketch) {
    let r = proto::solve(s, &ProtoOptions::default()).unwrap();
    assert!(
        matches!(
            r.status,
            ProtoStatus::Converged | ProtoStatus::ConvergedRedundant
        ),
        "solver status {:?}, residual {}",
        r.status,
        r.max_residual
    );
}

fn kinds(s: &Sketch, ids: &[ConstraintId]) -> std::collections::BTreeMap<&'static str, usize> {
    let mut m = std::collections::BTreeMap::new();
    for id in ids {
        *m.entry(s.constraint(*id).unwrap().kind.name()).or_insert(0) += 1;
    }
    m
}

struct Rect {
    points: [EntityId; 4],
    lines: [EntityId; 4],
}

/// Axis-aligned rectangle `a..b` (H/V constraints; DOF 4).
fn rect(s: &mut Sketch, a: Point2, b: Point2) -> Rect {
    let built = build::rectangle_2pt(s, a, b).unwrap();
    Rect {
        points: built.entities[..4].try_into().unwrap(),
        lines: built.entities[4..].try_into().unwrap(),
    }
}

/// Rectangle fixed at its first corner with width and height dimensions: DOF 0.
/// Returns the rectangle and its (width, height) dimension ids.
fn pinned_rect(s: &mut Sketch, a: Point2, b: Point2) -> (Rect, ConstraintId, ConstraintId) {
    let r = rect(s, a, b);
    s.add_constraint(ConstraintKind::Fix {
        entity: r.points[0],
    })
    .unwrap();
    let w = dim(
        s,
        DimensionKind::Length { line: r.lines[0] },
        (b.x - a.x).abs(),
    );
    let h = dim(
        s,
        DimensionKind::Length { line: r.lines[1] },
        (b.y - a.y).abs(),
    );
    (r, w, h)
}

fn reflect_x(q: Point2) -> Point2 {
    p(-q.x, q.y)
}

/// A fixed vertical axis through the origin.
fn axis(s: &mut Sketch) -> EntityId {
    let a = s.add_line(p(0.0, -10.0), p(0.0, 10.0)).unwrap();
    s.add_constraint(ConstraintKind::Fix { entity: a }).unwrap();
    a
}

fn new_points(s: &Sketch, r: &EditResult) -> Vec<EntityId> {
    r.created
        .iter()
        .copied()
        .filter(|id| matches!(s.entity(*id).unwrap().kind, EntityKind::Point { .. }))
        .collect()
}

// ---- Mirror -------------------------------------------------------------------------------

#[test]
fn mirror_unconstrained_is_exact_and_independent() {
    let mut s = new();
    let ax = s.add_line(p(0.0, -10.0), p(0.0, 10.0)).unwrap();
    let r = rect(&mut s, p(2.0, 1.0), p(6.0, 3.0));
    let dof0 = dof(&s).unwrap();
    let res = mirror(&mut s, &r.lines, ax, false).unwrap();
    // 4 points, 4 lines; the rectangle's 4 H/V constraints survive a reflection.
    assert_eq!(res.created.len(), 8);
    assert_eq!(kinds(&s, &res.constraints_added)["horizontal"], 2);
    assert_eq!(kinds(&s, &res.constraints_added)["vertical"], 2);
    assert!(residual(&s) < 1e-9);
    let imgs = new_points(&s, &res);
    for (src, img) in r.points.iter().zip(&imgs) {
        assert!(close(pos(&s, *img), reflect_x(pos(&s, *src))));
    }
    // Independent: the image is a free rectangle (DOF 4, like the source).
    assert_eq!(dof(&s).unwrap(), dof0 + 4);

    // Moving the source (translate in place) does not move the image.
    let before: Vec<Point2> = imgs.iter().map(|i| pos(&s, *i)).collect();
    transform(&mut s, &r.lines, translation(Vec2::new(3.0, 1.0)), false).unwrap();
    solve(&mut s);
    for (i, b) in imgs.iter().zip(before) {
        assert!(close(pos(&s, *i), b));
    }
}

#[test]
fn mirror_constrained_follows_source() {
    let mut s = new();
    let ax = axis(&mut s);
    let (r, w, h) = pinned_rect(&mut s, p(2.0, 1.0), p(6.0, 3.0));
    assert_eq!(dof(&s).unwrap(), 0);
    let res = mirror(&mut s, &r.lines, ax, true).unwrap();
    // One Symmetric per point pair, no clutter.
    assert_eq!(
        kinds(&s, &res.constraints_added),
        [("symmetric", 4)].into_iter().collect()
    );
    assert!(residual(&s) < 1e-9);
    assert_eq!(dof(&s).unwrap(), 0, "the image adds no freedom");
    let imgs = new_points(&s, &res);

    // Edit the source's dimensions and re-solve: the image follows.
    s.set_dimension_value(w, 7.0).unwrap();
    s.set_dimension_value(h, 1.5).unwrap();
    solve(&mut s);
    assert!(close(pos(&s, r.points[2]), p(9.0, 2.5)));
    for (src, img) in r.points.iter().zip(&imgs) {
        assert!(
            close(pos(&s, *img), reflect_x(pos(&s, *src))),
            "{:?} vs {:?}",
            pos(&s, *img),
            pos(&s, *src)
        );
    }
    assert_eq!(dof(&s).unwrap(), 0);
}

#[test]
fn mirror_arc_swaps_ends_and_circle_gets_equal() {
    let mut s = new();
    let ax = axis(&mut s);
    // CCW arc about (5, 0) from (7, 0) to (5, 2).
    let arc = s.add_arc(p(5.0, 0.0), p(7.0, 0.0), p(5.0, 2.0)).unwrap();
    let circle = s.add_circle(p(4.0, 5.0), 1.5).unwrap();
    let res = mirror(&mut s, &[arc, circle], ax, false).unwrap();
    let EntityKind::Arc { start, end, center } = s.entity(res.created[4]).unwrap().kind else {
        panic!("expected the mirrored arc");
    };
    assert!(close(pos(&s, center), p(-5.0, 0.0)));
    // The image of the CCW arc is CW; swapping makes it CCW again: start' = mirror(end).
    assert!(close(pos(&s, start), p(-5.0, 2.0)));
    assert!(close(pos(&s, end), p(-7.0, 0.0)));
    assert!(residual(&s) < 1e-9);

    // Constrained: Symmetric per point + Equal for the circle; residual zero.
    let mut s2 = new();
    let ax2 = axis(&mut s2);
    let arc = s2.add_arc(p(5.0, 0.0), p(7.0, 0.0), p(5.0, 2.0)).unwrap();
    let circle = s2.add_circle(p(4.0, 5.0), 1.5).unwrap();
    let dof0 = dof(&s2).unwrap();
    let res = mirror(&mut s2, &[arc, circle], ax2, true).unwrap();
    let k = kinds(&s2, &res.constraints_added);
    assert_eq!(k["symmetric"], 4);
    assert_eq!(k["equal"], 1);
    assert!(residual(&s2) < 1e-9);
    assert_eq!(dof(&s2).unwrap(), dof0);
}

#[test]
fn mirror_shares_points_on_the_axis() {
    let mut s = new();
    let ax = axis(&mut s);
    let r = rect(&mut s, p(0.0, 1.0), p(4.0, 3.0)); // left edge on the axis
    let dof0 = dof(&s).unwrap();
    let res = mirror(&mut s, &r.lines, ax, true).unwrap();
    // 2 new points, 3 new lines (the left edge is its own image).
    assert_eq!(res.created.len(), 5);
    let k = kinds(&s, &res.constraints_added);
    assert_eq!(k["symmetric"], 2);
    assert_eq!(k["point on curve"], 2);
    assert!(residual(&s) < 1e-9);
    // The image adds no freedom. The source loses one DOF: the second PointOnCurve is implied
    // by the rectangle's vertical left edge (a consistent, redundant constraint).
    assert_eq!(dof(&s).unwrap(), dof0 - 1);
}

#[test]
fn mirror_errors() {
    let mut s = new();
    let r = rect(&mut s, p(2.0, 1.0), p(6.0, 3.0));
    // Axis must be a line.
    assert!(matches!(
        mirror(&mut s, &r.lines, r.points[0], false),
        Err(SketchError::WrongEntityKind { .. })
    ));
    // Zero-length axis.
    let a = s.add_point(p(1.0, 1.0)).unwrap();
    let b = s.add_point(p(1.0, 1.0)).unwrap();
    let zero = s.add_line_between(a, b).unwrap();
    assert!(matches!(
        mirror(&mut s, &r.lines, zero, false),
        Err(SketchError::Degenerate(_))
    ));
    let ax = s.add_line(p(0.0, 0.0), p(0.0, 1.0)).unwrap();
    assert!(matches!(
        mirror(&mut s, &[], ax, false),
        Err(SketchError::NotApplicable(_))
    ));
    let gone = {
        let l = s.add_line(p(9.0, 9.0), p(9.0, 8.0)).unwrap();
        s.remove_entities(&[l]).unwrap();
        l
    };
    assert!(matches!(
        mirror(&mut s, &[gone], ax, false),
        Err(SketchError::UnknownEntity(_))
    ));
}

// ---- Rectangular pattern ------------------------------------------------------------------

#[test]
fn pattern_rect_constrained_follows_source() {
    let mut s = new();
    let (r, w, _h) = pinned_rect(&mut s, p(0.0, 0.0), p(2.0, 1.0));
    let res = pattern_rect(
        &mut s,
        &r.lines,
        Vec2::new(1.0, 0.0),
        3,
        5.0,
        Some((Vec2::new(0.0, 1.0), 2, 4.0)),
        true,
    )
    .unwrap();
    // 5 copies of (4 points + 4 lines).
    assert_eq!(res.created.len(), 5 * 8);
    // Per copied point two offset constraints, except zero offsets (Vertical/Horizontal).
    let k = kinds(&s, &res.constraints_added);
    assert_eq!(
        k.values().sum::<usize>(),
        5 * 4 * 2,
        "two offset constraints per copied point: {k:?}"
    );
    assert!(residual(&s) < 1e-9);
    assert_eq!(dof(&s).unwrap(), 0, "copies add no freedom");
    let imgs = new_points(&s, &res);
    assert_eq!(imgs.len(), 20);

    // Grid positions (j-major, then i): offsets (5,0), (10,0), (0,4), (5,4), (10,4).
    let offs = [(5.0, 0.0), (10.0, 0.0), (0.0, 4.0), (5.0, 4.0), (10.0, 4.0)];
    for (n, (ox, oy)) in offs.iter().enumerate() {
        for (k, src) in r.points.iter().enumerate() {
            let q = pos(&s, imgs[n * 4 + k]);
            assert!(close(q, pos(&s, *src) + Vec2::new(*ox, *oy)));
        }
    }

    // Editing the source moves every copy.
    s.set_dimension_value(w, 3.0).unwrap();
    solve(&mut s);
    for (n, (ox, oy)) in offs.iter().enumerate() {
        for (k, src) in r.points.iter().enumerate() {
            let q = pos(&s, imgs[n * 4 + k]);
            assert!(close(q, pos(&s, *src) + Vec2::new(*ox, *oy)));
        }
    }
    assert!(close(pos(&s, r.points[1]), p(3.0, 0.0)));
}

#[test]
fn pattern_rect_unconstrained_is_independent() {
    let mut s = new();
    let (r, w, _h) = pinned_rect(&mut s, p(0.0, 0.0), p(2.0, 1.0));
    let res = pattern_rect(&mut s, &r.lines, Vec2::new(0.0, 2.0), 3, 3.0, None, false).unwrap();
    assert_eq!(res.created.len(), 16);
    assert!(residual(&s) < 1e-9);
    // Each copy keeps H/V and the two length dimensions, but is free to move: DOF 2 each.
    assert_eq!(dof(&s).unwrap(), 4);
    let imgs = new_points(&s, &res);
    let before: Vec<Point2> = imgs.iter().map(|i| pos(&s, *i)).collect();
    s.set_dimension_value(w, 5.0).unwrap();
    solve(&mut s);
    assert!(close(pos(&s, r.points[1]), p(5.0, 0.0)));
    for (i, b) in imgs.iter().zip(before) {
        assert!(close(pos(&s, *i), b), "copy moved with the source");
    }
}

#[test]
fn pattern_rect_errors() {
    let mut s = new();
    let r = rect(&mut s, p(0.0, 0.0), p(2.0, 1.0));
    let x = Vec2::new(1.0, 0.0);
    assert!(matches!(
        pattern_rect(&mut s, &r.lines, Vec2::ZERO, 3, 1.0, None, false),
        Err(SketchError::Degenerate(_))
    ));
    assert!(matches!(
        pattern_rect(&mut s, &r.lines, x, 0, 1.0, None, false),
        Err(SketchError::InvalidValue { .. })
    ));
    assert!(matches!(
        pattern_rect(&mut s, &r.lines, x, 1, 1.0, None, false),
        Err(SketchError::NotApplicable(_))
    ));
    assert!(matches!(
        pattern_rect(&mut s, &r.lines, x, 3, 0.0, None, false),
        Err(SketchError::Degenerate(_))
    ));
    assert!(matches!(
        pattern_rect(&mut s, &r.lines, x, 3, f64::NAN, None, false),
        Err(SketchError::InvalidValue { .. })
    ));
    // Nothing was created by the failed calls.
    assert_eq!(s.entities().count(), 8);
}

// ---- Circular pattern ---------------------------------------------------------------------

/// Bolt circle: fixed centre, a circle at distance `d` on the x axis with radius `r`.
fn bolt(s: &mut Sketch) -> (EntityId, EntityId, ConstraintId, ConstraintId) {
    let c = s.add_point(p(0.0, 0.0)).unwrap();
    s.add_constraint(ConstraintKind::Fix { entity: c }).unwrap();
    let hole = s.add_circle(p(20.0, 0.0), 3.0).unwrap();
    let EntityKind::Circle { center, .. } = s.entity(hole).unwrap().kind else {
        unreachable!()
    };
    s.add_constraint(ConstraintKind::Horizontal(HvTarget::Points(c, center)))
        .unwrap();
    let d = dim(
        s,
        DimensionKind::Distance {
            a: c,
            b: center,
            side: amg_sketch::Side::Positive,
        },
        20.0,
    );
    let r = dim(s, DimensionKind::Radius { curve: hole }, 3.0);
    (c, hole, d, r)
}

#[test]
fn pattern_circ_constrained_follows_source() {
    let mut s = new();
    let (c, hole, d, r) = bolt(&mut s);
    assert_eq!(dof(&s).unwrap(), 0);
    let res = pattern_circ(&mut s, &[hole], c, 6, TAU, true).unwrap();
    assert!(residual(&s) < 1e-9);
    assert_eq!(dof(&s).unwrap(), 0, "copies add no freedom");
    let k = kinds(&s, &res.constraints_added);
    // Per copy: Equal (link lines) + angle + Equal (radius).
    assert_eq!(k["equal"], 10);
    assert_eq!(k["angle"], 5);

    let centers: Vec<Point2> = new_points(&s, &res).iter().map(|i| pos(&s, *i)).collect();
    assert_eq!(centers.len(), 5);
    for (k, q) in centers.iter().enumerate() {
        let a = TAU * (k + 1) as f64 / 6.0;
        assert!(close(*q, p(20.0 * a.cos(), 20.0 * a.sin())), "{q:?}");
    }

    // Move the source out to 25 mm and grow it: all copies follow on the bolt circle.
    s.set_dimension_value(d, 25.0).unwrap();
    s.set_dimension_value(r, 4.0).unwrap();
    solve(&mut s);
    for (k, i) in new_points(&s, &res).iter().enumerate() {
        let a = TAU * (k + 1) as f64 / 6.0;
        assert!(close(pos(&s, *i), p(25.0 * a.cos(), 25.0 * a.sin())));
    }
    for id in &res.created {
        if let EntityKind::Circle { radius, .. } = s.entity(*id).unwrap().kind {
            assert!((s.params().get(radius) - 4.0).abs() < 1e-6);
        }
    }
    assert_eq!(dof(&s).unwrap(), 0);
}

#[test]
fn pattern_circ_partial_angle_and_unconstrained() {
    let mut s = new();
    let (c, hole, d, _r) = bolt(&mut s);
    // Three instances over a quarter turn: the last sits at exactly 90 degrees.
    let res = pattern_circ(&mut s, &[hole], c, 3, FRAC_PI_2, false).unwrap();
    let pts = new_points(&s, &res);
    assert!(close(
        pos(&s, pts[0]),
        p(20.0 * FRAC_PI_4.cos(), 20.0 * FRAC_PI_4.sin())
    ));
    assert!(close(pos(&s, pts[1]), p(0.0, 20.0)));
    // Only the radius dimension is internal to the selection: copies are free circles
    // with a radius dimension (DOF 2 each).
    assert_eq!(kinds(&s, &res.constraints_added)["radius"], 2);
    assert_eq!(dof(&s).unwrap(), 4);
    // Independent of the source.
    let before = pos(&s, pts[1]);
    s.set_dimension_value(d, 30.0).unwrap();
    solve(&mut s);
    assert!(close(pos(&s, pts[1]), before));
}

#[test]
fn pattern_circ_lines_use_arc_centre_and_share_the_centre_point() {
    let mut s = new();
    // A spoke from the pattern centre outwards: the centre end point is shared by all copies.
    let c = s.add_point(p(0.0, 0.0)).unwrap();
    s.add_constraint(ConstraintKind::Fix { entity: c }).unwrap();
    let a = s.add_point(p(10.0, 0.0)).unwrap();
    let spoke = s.add_line_between(c, a).unwrap();
    let res = pattern_circ(&mut s, &[spoke], c, 4, TAU, true).unwrap();
    // Copies: 3 end points + 3 spokes, plus the construction links (1 source + 3).
    let spokes = res
        .created
        .iter()
        .filter(|id| matches!(s.entity(**id).unwrap().kind, EntityKind::Line { start, .. } if start == c))
        .count();
    assert_eq!(spokes, 3 + 3 + 1);
    assert!(residual(&s) < 1e-9);
    assert_eq!(dof(&s).unwrap(), 2, "only the source end point is free");

    // Centre given as a circle works too (its centre point is used).
    let mut s = new();
    let hub = s.add_circle(p(0.0, 0.0), 2.0).unwrap();
    let dot = s.add_circle(p(10.0, 0.0), 1.0).unwrap();
    let res = pattern_circ(&mut s, &[dot], hub, 4, TAU, false).unwrap();
    assert_eq!(res.created.len(), 6);
}

#[test]
fn pattern_circ_errors() {
    let mut s = new();
    let (c, hole, _, _) = bolt(&mut s);
    let line = s.add_line(p(0.0, 0.0), p(1.0, 1.0)).unwrap();
    assert!(matches!(
        pattern_circ(&mut s, &[hole], line, 4, TAU, false),
        Err(SketchError::WrongEntityKind { .. })
    ));
    assert!(matches!(
        pattern_circ(&mut s, &[hole], c, 1, TAU, false),
        Err(SketchError::NotApplicable(_))
    ));
    assert!(matches!(
        pattern_circ(&mut s, &[hole], c, 4, 0.0, false),
        Err(SketchError::Degenerate(_))
    ));
    assert!(matches!(
        pattern_circ(&mut s, &[hole], c, 4, f64::INFINITY, false),
        Err(SketchError::InvalidValue { .. })
    ));
}

// ---- Move / copy / rotate / scale ---------------------------------------------------------

#[test]
fn copy_is_independent_and_keeps_internal_constraints() {
    let mut s = new();
    let (r, w, _h) = pinned_rect(&mut s, p(0.0, 0.0), p(4.0, 2.0));
    let res = transform(&mut s, &r.lines, translation(Vec2::new(10.0, 5.0)), true).unwrap();
    assert_eq!(res.created.len(), 8);
    // H/V x4 + two length dimensions; `Fix` is not copied.
    let k = kinds(&s, &res.constraints_added);
    assert_eq!(k["horizontal"], 2);
    assert_eq!(k["vertical"], 2);
    assert_eq!(k["length"], 2);
    assert!(!k.contains_key("fix"));
    assert!(residual(&s) < 1e-9);
    assert_eq!(dof(&s).unwrap(), 2);
    let imgs = new_points(&s, &res);
    assert!(close(pos(&s, imgs[2]), p(14.0, 7.0)));
    // Editing the source does not move the copy.
    s.set_dimension_value(w, 9.0).unwrap();
    solve(&mut s);
    assert!(close(pos(&s, imgs[2]), p(14.0, 7.0)));
    // The source is untouched.
    assert_eq!(s.entities().count(), 16);
}

#[test]
fn move_in_place_moves_and_keeps_constraints() {
    let mut s = new();
    let r = rect(&mut s, p(0.0, 0.0), p(4.0, 2.0));
    let dof0 = dof(&s).unwrap();
    let before = s.constraints().count();
    let res = transform(&mut s, &r.lines, translation(Vec2::new(-3.0, 7.0)), false).unwrap();
    assert!(res.created.is_empty() && res.constraints_removed.is_empty());
    assert!(close(pos(&s, r.points[0]), p(-3.0, 7.0)));
    assert!(close(pos(&s, r.points[2]), p(1.0, 9.0)));
    assert_eq!(s.constraints().count(), before);
    assert!(residual(&s) < 1e-9);
    assert_eq!(dof(&s).unwrap(), dof0);
}

#[test]
fn rotate_about_a_point() {
    let mut s = new();
    let r = rect(&mut s, p(0.0, 0.0), p(4.0, 2.0));
    let w = dim(&mut s, DimensionKind::Length { line: r.lines[0] }, 4.0);
    // 90 degrees about (1, 1): lengths stay, H/V no longer hold and are reported.
    let res = transform(
        &mut s,
        &r.lines,
        rotation_about(p(1.0, 1.0), FRAC_PI_2),
        false,
    )
    .unwrap();
    assert!(close(pos(&s, r.points[0]), p(2.0, 0.0)));
    assert!(close(pos(&s, r.points[1]), p(2.0, 4.0)));
    assert_eq!(res.constraints_removed.len(), 4);
    assert!(
        res.constraints_removed
            .iter()
            .all(|id| s.constraint(*id).is_none())
    );
    assert_eq!(
        s.constraint(w).unwrap().kind.as_dimension().unwrap().value,
        4.0
    );
    assert!(residual(&s) < 1e-9);

    // 30 degrees in a copy of the (now constraint-free) shape.
    let mut s = new();
    let r = rect(&mut s, p(0.0, 0.0), p(4.0, 2.0));
    let res = transform(
        &mut s,
        &r.lines,
        rotation_about(p(-1.0, 0.0), FRAC_PI_6),
        true,
    )
    .unwrap();
    assert_eq!(res.created.len(), 8);
    assert!(
        res.constraints_added.is_empty(),
        "H/V do not hold after rotating"
    );
    let q = pos(&s, new_points(&s, &res)[1]);
    let expect = p(-1.0, 0.0) + Vec2::new(5.0 * FRAC_PI_6.cos(), 5.0 * FRAC_PI_6.sin());
    assert!(close(q, expect));
}

#[test]
fn scale_about_a_point_scales_dimensions() {
    let mut s = new();
    let r = rect(&mut s, p(1.0, 1.0), p(5.0, 3.0));
    let w = dim(&mut s, DimensionKind::Length { line: r.lines[0] }, 4.0);
    let h = dim(&mut s, DimensionKind::Length { line: r.lines[1] }, 2.0);
    let circle = s.add_circle(p(2.0, 2.0), 0.5).unwrap();
    let rad = dim(&mut s, DimensionKind::Radius { curve: circle }, 0.5);
    // An angle dimension between two lines of the rectangle must not change.
    let ang = dim(
        &mut s,
        DimensionKind::Angle {
            a: r.lines[0],
            b: r.lines[1],
            branch: amg_sketch::AngleBranch {
                reverse_a: false,
                reverse_b: false,
            },
        },
        FRAC_PI_2,
    );
    let dof0 = dof(&s).unwrap();
    let mut items = r.lines.to_vec();
    items.push(circle);
    let res = transform(&mut s, &items, scale_about(p(1.0, 1.0), 2.5), false).unwrap();
    assert!(res.constraints_removed.is_empty());
    let value = |id| s.constraint(id).unwrap().kind.as_dimension().unwrap().value;
    assert!((value(w) - 10.0).abs() < 1e-9);
    assert!((value(h) - 5.0).abs() < 1e-9);
    assert!((value(rad) - 1.25).abs() < 1e-9);
    assert!((value(ang) - FRAC_PI_2).abs() < 1e-12);
    // The fixed point of the scale stays; the far corner moves to (1 + 10, 1 + 5).
    assert!(close(pos(&s, r.points[0]), p(1.0, 1.0)));
    assert!(close(pos(&s, r.points[2]), p(11.0, 6.0)));
    let EntityKind::Circle { center, radius } = s.entity(circle).unwrap().kind else {
        unreachable!()
    };
    assert!(close(pos(&s, center), p(1.0 + 2.5, 1.0 + 2.5)));
    assert!((s.params().get(radius) - 1.25).abs() < 1e-9);
    // Dimensions and geometry agree again: zero residual, same DOF, solver is a no-op.
    assert!(residual(&s) < 1e-9);
    assert_eq!(dof(&s).unwrap(), dof0);
    let snapshot = s.params().as_slice().to_vec();
    solve(&mut s);
    for (a, b) in snapshot.iter().zip(s.params().as_slice()) {
        assert!((a - b).abs() < 1e-6);
    }
}

#[test]
fn scale_copy_scales_copied_dimensions_only() {
    let mut s = new();
    let r = rect(&mut s, p(0.0, 0.0), p(4.0, 2.0));
    let w = dim(&mut s, DimensionKind::Length { line: r.lines[0] }, 4.0);
    let res = transform(&mut s, &r.lines, scale_about(p(0.0, 0.0), 0.5), true).unwrap();
    // Source untouched.
    assert_eq!(
        s.constraint(w).unwrap().kind.as_dimension().unwrap().value,
        4.0
    );
    assert!(close(pos(&s, r.points[2]), p(4.0, 2.0)));
    let copied = res
        .constraints_added
        .iter()
        .filter_map(|id| {
            s.constraint(*id)
                .unwrap()
                .kind
                .as_dimension()
                .map(|d| d.value)
        })
        .collect::<Vec<_>>();
    assert_eq!(copied, vec![2.0]);
    // The scale's fixed point (the origin corner) is shared, so 3 new points: (4,0), (4,2), (0,2).
    assert!(close(pos(&s, new_points(&s, &res)[1]), p(2.0, 1.0)));
    assert!(residual(&s) < 1e-9);
}

#[test]
fn transform_rejects_fixed_and_unsupported_transforms() {
    let mut s = new();
    let (r, _, _) = pinned_rect(&mut s, p(0.0, 0.0), p(4.0, 2.0));
    let snapshot = s.params().as_slice().to_vec();
    assert!(matches!(
        transform(&mut s, &r.lines, translation(Vec2::X), false),
        Err(SketchError::Locked(_))
    ));
    assert_eq!(s.params().as_slice(), &snapshot[..], "no partial move");
    // A copy of fixed geometry is fine.
    transform(&mut s, &r.lines, translation(Vec2::X), true).unwrap();

    let t_reflect = Transform2::from_scale(Vec2::new(1.0, -1.0));
    assert!(matches!(
        transform(&mut s, &r.lines, t_reflect, true),
        Err(SketchError::NotApplicable(_))
    ));
    let t_aniso = Transform2::from_scale(Vec2::new(2.0, 1.0));
    assert!(matches!(
        transform(&mut s, &r.lines, t_aniso, true),
        Err(SketchError::NotApplicable(_))
    ));
    let t_shear = Transform2::from_mat2(amg_core::glam::DMat2::from_cols(
        Vec2::new(1.0, 0.0),
        Vec2::new(0.5, 1.0),
    ));
    assert!(matches!(
        transform(&mut s, &r.lines, t_shear, true),
        Err(SketchError::NotApplicable(_))
    ));
    assert!(matches!(
        transform(&mut s, &r.lines, scale_about(Point2::ZERO, 0.0), true),
        Err(SketchError::Degenerate(_))
    ));
    assert!(matches!(
        transform(&mut s, &[], translation(Vec2::X), true),
        Err(SketchError::NotApplicable(_))
    ));
    let nan = translation(Vec2::new(f64::NAN, 0.0));
    assert!(matches!(
        transform(&mut s, &r.lines, nan, true),
        Err(SketchError::InvalidValue { .. })
    ));
}

#[test]
fn tangent_and_arcs_survive_a_rotated_copy() {
    // A line tangent to a circle: the copy keeps the tangency (side re-captured).
    let mut s = new();
    let circle = s.add_circle(p(0.0, 0.0), 2.0).unwrap();
    let line = s.add_line(p(-5.0, 2.0), p(5.0, 2.0)).unwrap();
    s.add_constraint_oriented(ConstraintKind::Tangent {
        a: line,
        b: circle,
        kind: amg_sketch::TangentKind::External,
    })
    .unwrap();
    let arc = s.add_arc(p(0.0, 0.0), p(3.0, 0.0), p(0.0, 3.0)).unwrap();
    let res = transform(
        &mut s,
        &[line, circle, arc],
        rotation_about(p(1.0, 2.0), PI * 0.7),
        true,
    )
    .unwrap();
    assert_eq!(kinds(&s, &res.constraints_added)["tangent"], 1);
    assert!(residual(&s) < 1e-9);
    let EntityKind::Arc { start, end, center } =
        s.entity(*res.created.last().unwrap()).unwrap().kind
    else {
        panic!("expected the copied arc last");
    };
    // Still CCW with the same sweep.
    let (c, a, b) = (pos(&s, center), pos(&s, start), pos(&s, end));
    assert!((a - c).perp_dot(b - c) > 0.0);
}
