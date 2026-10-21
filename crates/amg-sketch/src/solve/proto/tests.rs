#![allow(clippy::unwrap_used, reason = "tests")]

use amg_core::{Plane, Point2};

use super::*;
use crate::model::{ConstraintKind, Dimension, DimensionKind, HvTarget, Side};
use crate::{ConstraintId, EntityId};

fn dim(kind: DimensionKind, value: f64) -> ConstraintKind {
    ConstraintKind::Dimension(Dimension {
        kind,
        value,
        driving: true,
        label_pos: Point2::new(0.0, 0.0),
    })
}

/// Triangle p0 p1 p2 with p0 fixed, horizontal base and three lengths.
fn triangle(c: f64) -> (Sketch, [EntityId; 3]) {
    let mut s = Sketch::new(Plane::XY);
    let p = [
        s.add_point(Point2::new(0.0, 0.0)).unwrap(),
        s.add_point(Point2::new(9.0, 0.5)).unwrap(),
        s.add_point(Point2::new(3.0, 4.0)).unwrap(),
    ];
    let l = [
        s.add_line_between(p[0], p[1]).unwrap(),
        s.add_line_between(p[1], p[2]).unwrap(),
        s.add_line_between(p[2], p[0]).unwrap(),
    ];
    s.add_constraint(ConstraintKind::Fix { entity: p[0] })
        .unwrap();
    s.add_constraint(ConstraintKind::Horizontal(HvTarget::Line(l[0])))
        .unwrap();
    s.add_constraint(dim(DimensionKind::Length { line: l[0] }, 10.0))
        .unwrap();
    s.add_constraint(dim(DimensionKind::Length { line: l[1] }, 8.0))
        .unwrap();
    s.add_constraint(dim(DimensionKind::Length { line: l[2] }, c))
        .unwrap();
    (s, p)
}

/// Rectangle with a fixed corner, H/V sides, two lengths and optional redundant extras.
fn rectangle(redundant: bool) -> Sketch {
    let mut s = Sketch::new(Plane::XY);
    let p = [
        s.add_point(Point2::new(0.0, 0.0)).unwrap(),
        s.add_point(Point2::new(18.0, 0.4)).unwrap(),
        s.add_point(Point2::new(19.0, 11.0)).unwrap(),
        s.add_point(Point2::new(0.5, 9.0)).unwrap(),
    ];
    let l: Vec<EntityId> = (0..4)
        .map(|i| s.add_line_between(p[i], p[(i + 1) % 4]).unwrap())
        .collect();
    s.add_constraint(ConstraintKind::Fix { entity: p[0] })
        .unwrap();
    s.add_constraint(ConstraintKind::Horizontal(HvTarget::Line(l[0])))
        .unwrap();
    s.add_constraint(ConstraintKind::Vertical(HvTarget::Line(l[1])))
        .unwrap();
    s.add_constraint(ConstraintKind::Horizontal(HvTarget::Line(l[2])))
        .unwrap();
    s.add_constraint(ConstraintKind::Vertical(HvTarget::Line(l[3])))
        .unwrap();
    s.add_constraint(dim(DimensionKind::Length { line: l[0] }, 20.0))
        .unwrap();
    s.add_constraint(dim(DimensionKind::Length { line: l[1] }, 10.0))
        .unwrap();
    if redundant {
        s.add_constraint(ConstraintKind::Parallel { a: l[0], b: l[2] })
            .unwrap();
        s.add_constraint(ConstraintKind::Equal { a: l[1], b: l[3] })
            .unwrap();
    }
    s
}

#[test]
fn triangle_converges_to_the_dimensioned_shape() {
    for algorithm in [ProtoAlgorithm::LevenbergMarquardt, ProtoAlgorithm::DogLeg] {
        let (mut s, p) = triangle(6.0);
        let opts = ProtoOptions {
            algorithm,
            ..ProtoOptions::default()
        };
        let r = solve(&mut s, &opts).unwrap();
        assert_eq!(r.status, ProtoStatus::Converged, "{algorithm:?}: {r:?}");
        assert_eq!(r.dof, Some(0));
        assert!(r.max_residual <= r.tolerance);
        let q = s.point(p[2]).unwrap();
        // 10-8-6 is a right triangle: p2 = (3.6, 4.8) on the side of the start.
        assert!(
            (q.x - 3.6).abs() < 1e-9 && (q.y - 4.8).abs() < 1e-9,
            "{q:?}"
        );
        // Substitution removed the horizontal: p1.y aliases the fixed p0.y.
        assert_eq!(r.substituted_rows, 1);
        assert_eq!(r.unknowns, 3);
    }
}

#[test]
fn substitution_shrinks_the_rectangle() {
    let mut s = rectangle(false);
    let r = solve(&mut s, &ProtoOptions::default()).unwrap();
    assert_eq!(r.status, ProtoStatus::Converged);
    assert_eq!(r.dof, Some(0));
    assert_eq!(r.substituted_rows, 4);
    // x: p1/p2 class, y: p2/p3 class (one component: the length rows touch both).
    assert_eq!(r.unknowns, 2);
    let without = solve(
        &mut rectangle(false),
        &ProtoOptions {
            substitution: false,
            ..ProtoOptions::default()
        },
    )
    .unwrap();
    assert_eq!(without.status, ProtoStatus::Converged);
    assert_eq!(without.dof, Some(0));
    assert_eq!(without.unknowns, 6);
}

#[test]
fn redundant_rectangle_reports_two_minimal_groups() {
    let mut s = rectangle(true);
    let r = solve(&mut s, &ProtoOptions::default()).unwrap();
    assert_eq!(r.status, ProtoStatus::ConvergedRedundant, "{r:?}");
    assert_eq!(r.dof, Some(0));
    let c = |i: u32| EquationOwner::Constraint(ConstraintId(i));
    let sets: Vec<Vec<EquationOwner>> = r.groups.iter().map(|g| g.owners.clone()).collect();
    assert!(r.groups.iter().all(|g| g.kind == ProtoGroupKind::Redundant));
    // Constraint ids follow insertion: 1 fix, 2..5 H/V, 6..7 lengths, 8 parallel, 9 equal.
    let ids: Vec<u32> = s.constraints().map(|(id, _)| id.0).collect();
    let (h0, h2, par, eq) = (ids[1], ids[3], ids[7], ids[8]);
    assert!(sets.contains(&vec![c(h0), c(h2), c(par)]), "{sets:?}");
    assert!(sets.contains(&vec![c(h0), c(h2), c(eq)]), "{sets:?}");
}

#[test]
fn impossible_triangle_fails_and_names_the_lengths() {
    let (mut s, _) = triangle(30.0);
    let r = solve(&mut s, &ProtoOptions::default()).unwrap();
    assert!(
        matches!(
            r.status,
            ProtoStatus::Inconsistent | ProtoStatus::DidNotConverge
        ),
        "{r:?}"
    );
    let ids: Vec<u32> = s.constraints().map(|(id, _)| id.0).collect();
    let lengths: Vec<EquationOwner> = ids[2..5]
        .iter()
        .map(|&i| EquationOwner::Constraint(ConstraintId(i)))
        .collect();
    assert!(
        r.groups
            .iter()
            .any(|g| g.kind == ProtoGroupKind::Conflicting && g.owners == lengths),
        "{:?}",
        r.groups
    );
}

#[test]
fn solve_is_deterministic() {
    let run = || {
        let (mut s, _) = triangle(6.0);
        solve(&mut s, &ProtoOptions::default()).unwrap();
        s.params()
            .as_slice()
            .iter()
            .map(|v| v.to_bits())
            .collect::<Vec<_>>()
    };
    let first = run();
    for _ in 0..20 {
        assert_eq!(run(), first);
    }
}

/// Four-bar o1 (fixed) – a – b – o2 (fixed), lengths 4, 8, 6.
fn four_bar() -> (Sketch, [EntityId; 4]) {
    let mut s = Sketch::new(Plane::XY);
    let o1 = s.add_point(Point2::new(0.0, 0.0)).unwrap();
    let o2 = s.add_point(Point2::new(10.0, 0.0)).unwrap();
    let a = s.add_point(Point2::new(2.0, 3.4)).unwrap();
    let b = s.add_point(Point2::new(9.0, 6.0)).unwrap();
    let crank = s.add_line_between(o1, a).unwrap();
    let coupler = s.add_line_between(a, b).unwrap();
    let rocker = s.add_line_between(o2, b).unwrap();
    s.add_constraint(ConstraintKind::Fix { entity: o1 })
        .unwrap();
    s.add_constraint(ConstraintKind::Fix { entity: o2 })
        .unwrap();
    s.add_constraint(dim(DimensionKind::Length { line: crank }, 4.0))
        .unwrap();
    s.add_constraint(dim(DimensionKind::Length { line: coupler }, 8.0))
        .unwrap();
    s.add_constraint(dim(DimensionKind::Length { line: rocker }, 6.0))
        .unwrap();
    (s, [o1, o2, a, b])
}

#[test]
fn drag_methods_follow_the_cursor_on_the_rocker_circle() {
    for method in [DragMethod::WeightedMinNorm, DragMethod::Sqp] {
        let (mut s, [o1, o2, a, b]) = four_bar();
        let r = solve(&mut s, &ProtoOptions::default()).unwrap();
        assert_eq!(r.status, ProtoStatus::Converged);
        assert_eq!(r.dof, Some(1));
        let side = |s: &Sketch| {
            let (p, q, r) = (
                s.point(o1).unwrap(),
                s.point(a).unwrap(),
                s.point(b).unwrap(),
            );
            (q - p).perp_dot(r - p) > 0.0
        };
        let start_side = side(&s);
        let mut d = ProtoDrag::begin(&s, b, method, &ProtoOptions::default()).unwrap();
        let c = s.point(o2).unwrap();
        let ang0 = (s.point(b).unwrap() - c)
            .y
            .atan2((s.point(b).unwrap() - c).x);
        for k in 1..=40 {
            // Toward smaller |b − o1|: stays inside the reachable range (no toggle).
            let t = ang0 + 0.01 * f64::from(k);
            let target = Point2::new(c.x + 6.0 * t.cos(), c.y + 6.0 * t.sin());
            let f = d.step(&mut s, target);
            assert_eq!(
                f.status,
                ProtoStatus::Converged,
                "{method:?} frame {k}: {f:?}"
            );
            let got = s.point(b).unwrap();
            // Weighted min-norm lets the dragged point slip by ~W⁻¹ of the correction.
            assert!(
                (got - target).length() < 1e-3,
                "{method:?}: {got:?} vs {target:?}"
            );
            assert_eq!(side(&s), start_side, "{method:?}: branch flipped at {k}");
        }
    }
}

#[test]
fn fixed_point_drag_stays_put() {
    let (mut s, [o1, ..]) = four_bar();
    solve(&mut s, &ProtoOptions::default()).unwrap();
    let mut d = ProtoDrag::begin(
        &s,
        o1,
        DragMethod::WeightedMinNorm,
        &ProtoOptions::default(),
    )
    .unwrap();
    let f = d.step(&mut s, Point2::new(3.0, 3.0));
    assert_eq!(f.status, ProtoStatus::Converged);
    assert_eq!(s.point(o1).unwrap(), Point2::new(0.0, 0.0));
}

#[test]
fn side_of_point_line_distance_is_kept() {
    let mut s = Sketch::new(Plane::XY);
    let l = s
        .add_line(Point2::new(0.0, 0.0), Point2::new(10.0, 0.0))
        .unwrap();
    let p = s.add_point(Point2::new(5.0, -1.0)).unwrap();
    s.add_constraint(ConstraintKind::Fix { entity: l }).unwrap();
    s.add_constraint(dim(
        DimensionKind::Distance {
            a: p,
            b: l,
            side: Side::Negative,
        },
        3.0,
    ))
    .unwrap();
    let r = solve(&mut s, &ProtoOptions::default()).unwrap();
    assert_eq!(r.status, ProtoStatus::Converged);
    assert!((s.point(p).unwrap().y + 3.0).abs() < 1e-9);
    assert_eq!(r.dof, Some(1));
}
