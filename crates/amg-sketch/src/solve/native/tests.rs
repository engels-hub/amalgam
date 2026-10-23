//! Unit tests of the native solver internals (integration tests: `tests/solver.rs`).

#![allow(clippy::unwrap_used, reason = "tests")]

use super::*;
use crate::model::Sketch;
use amg_core::Point2;

fn never() -> bool {
    false
}

#[test]
fn empty_sketch_converges() {
    let mut s = Sketch::new(amg_core::Plane::XY);
    let r = NativeSolver::new().solve(&mut s, &SolveOptions::default(), &never);
    assert_eq!(r.status, SolveStatus::Converged);
    assert!(r.max_residual <= 0.0);
}

#[test]
fn cache_is_reused_for_value_edits() {
    let mut s = Sketch::new(amg_core::Plane::XY);
    let l = s
        .add_line(Point2::new(0.0, 0.0), Point2::new(10.0, 1.0))
        .unwrap();
    let d = s
        .add_constraint(ConstraintKind::Dimension(crate::model::Dimension {
            kind: DimensionKind::Length { line: l },
            value: 12.0,
            driving: true,
            label_pos: Point2::new(0.0, 0.0),
        }))
        .unwrap();
    let mut solver = NativeSolver::new();
    let r = solver.solve(&mut s, &SolveOptions::default(), &never);
    assert!(r.status.is_converged(), "{r:?}");
    assert!(!solver.stats().cache_hit);
    s.set_dimension_value(d, 13.0).unwrap();
    let r = solver.solve(&mut s, &SolveOptions::default(), &never);
    assert!(r.status.is_converged(), "{r:?}");
    assert!(solver.stats().cache_hit);
}
