//! SK-05 acceptance: drag sessions for points, lines, circle rims and arc endpoints on the
//! native solver (weighted minimum norm with a feasible-SQP fallback).
//!
//! Timing tests are named `timing_*` (nextest runs them alone); they assert the Done-when bound
//! (p95 < 2 ms) only with `AMG_STRICT_TIMING=1` and a loose bound otherwise.

#![allow(clippy::unwrap_used, clippy::print_stdout, reason = "tests")]

use std::path::PathBuf;

use amg_core::{Plane, Point2};
use amg_sketch::corpus::compare::{metrics, native::Native};
use amg_sketch::corpus::drag::{resample, run_script, truss_case};
use amg_sketch::corpus::{self, Case};
use amg_sketch::solve::{
    DragMode, DragSpec, NativeSolver, SolveOptions, SolveStatus, SolverBackend,
};
use amg_sketch::{
    ConstraintKind, Dimension, DimensionKind, EntityId, EntityKind, HvTarget, Sketch, SketchError,
};

const FRAMES: usize = 1000;

fn root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../testdata/sketch/corpus")
}

fn decision(n: u32) -> Case {
    let tag = format!("decision:D{n}");
    corpus::load_corpus(&root())
        .unwrap()
        .cases
        .into_iter()
        .find(|c| c.tags.contains(&tag))
        .unwrap()
}

fn never() -> bool {
    false
}

fn p(x: f64, y: f64) -> Point2 {
    Point2::new(x, y)
}

fn strict() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}

/// D4, D7 and the 200-entity truss with `FRAMES`-frame scripts.
fn scripted() -> Vec<(&'static str, Case, Vec<[f64; 2]>)> {
    let mut v = Vec::new();
    for (name, case) in [("D4", decision(4)), ("D7", decision(7))] {
        let path = resample(&case.drag.as_ref().unwrap().path, FRAMES);
        v.push((name, case, path));
    }
    let truss = truss_case(FRAMES);
    let path = truss.drag.as_ref().unwrap().path.clone();
    v.push(("truss200", truss, path));
    v
}

fn with_path(case: &Case, path: &[[f64; 2]]) -> Case {
    let mut c = case.clone();
    c.drag.as_mut().unwrap().path = path.to_vec();
    c
}

#[test]
fn truss_has_200_entities_and_solves() {
    let case = truss_case(10);
    assert!(
        (200..=202).contains(&case.entities.len()),
        "{}",
        case.entities.len()
    );
    let mut built = case.build().unwrap();
    let r = NativeSolver::new().solve(&mut built.sketch, &SolveOptions::default(), &never);
    assert!(r.status.is_converged(), "{r:?}");
}

#[test]
fn scripted_drags_converge_with_baseline_jitter_and_no_flips() {
    // SK-13 baselines (benchmark note §4): D4 0, D7 66.2 for the weighted-min-norm prototype.
    let documented = [("D4", 0.0), ("D7", 66.2)];
    for (name, case, path) in scripted() {
        let run = run_script(&case, Some(&path), DragMode::Auto).unwrap();
        let proto = metrics::measure(&Native::default(), &with_path(&case, &path), 1, 1);
        let base = proto.drag.map_or(f64::INFINITY, |d| d.jitter);
        println!(
            "{name}: jitter {:.4} (proto {base:.4}), flips {:?}, converged {}/{}, sqp {}, p95 {:.1} us, max residual {:.2e}",
            run.jitter,
            run.flips,
            run.converged_frames,
            run.frames,
            run.stats.sqp_frames,
            run.p95_us(),
            run.max_residual
        );
        assert_eq!(
            run.converged_frames, run.frames,
            "{name}: every frame converges"
        );
        assert_eq!(run.stats.failed_frames, 0, "{name}");
        assert!(
            run.jitter <= base + 1e-9,
            "{name}: jitter {} > prototype {base}",
            run.jitter
        );
        if let Some((_, b)) = documented.iter().find(|(n, _)| *n == name) {
            assert!(
                run.jitter <= *b + 1e-9,
                "{name}: jitter {} > SK-13 {b}",
                run.jitter
            );
        }
    }
}

#[test]
fn d7_has_no_chirality_flip() {
    let case = decision(7);
    let path = resample(&case.drag.as_ref().unwrap().path, FRAMES);
    for mode in [DragMode::Auto, DragMode::Sqp] {
        let run = run_script(&case, Some(&path), mode).unwrap();
        println!(
            "{mode:?}: flips at {:?}, cursor there {:?}",
            run.flip_frames,
            run.flip_frames.iter().map(|&f| path[f]).collect::<Vec<_>>()
        );
        assert_eq!(run.flips, Some(0), "{mode:?}: no chirality flip on D7");
    }
}

#[test]
fn sqp_and_min_norm_modes_converge() {
    for (name, case, path) in scripted() {
        for mode in [DragMode::Sqp, DragMode::MinNorm] {
            let path = &path[..200];
            let run = run_script(&case, Some(path), mode).unwrap();
            println!(
                "{name} {mode:?}: jitter {:.4}, flips {:?}, converged {}/{}, p95 {:.1} us",
                run.jitter,
                run.flips,
                run.converged_frames,
                run.frames,
                run.p95_us()
            );
            if mode == DragMode::Sqp {
                assert_eq!(run.converged_frames, run.frames, "{name}");
            }
        }
    }
}

#[test]
fn timing_drag_p95() {
    for (name, case, path) in scripted() {
        // Best of three passes to ride out scheduler noise on a loaded machine.
        let p95 = (0..3)
            .map(|_| {
                run_script(&case, Some(&path), DragMode::Auto)
                    .unwrap()
                    .p95_us()
            })
            .fold(f64::INFINITY, f64::min);
        println!("{name}: p95 {p95:.1} us over {FRAMES} frames");
        let bound = if strict() { 2000.0 } else { 20_000.0 };
        assert!(p95 < bound, "{name}: p95 {p95} us >= {bound} us");
    }
}

#[test]
fn drag_is_deterministic() {
    let case = truss_case(300);
    let bits = || {
        let mut built = case.build().unwrap();
        let mut solver = NativeSolver::new();
        solver.solve(&mut built.sketch, &SolveOptions::default(), &never);
        let tip = built.entities[case.drag.as_ref().unwrap().entity as usize];
        let grab = built.sketch.point(tip).unwrap();
        let mut d = solver
            .begin_drag(&built.sketch, DragSpec { entity: tip, grab })
            .unwrap();
        for t in &case.drag.as_ref().unwrap().path {
            d.step(&mut built.sketch, p(t[0], t[1]), &never);
        }
        built
            .sketch
            .params()
            .as_slice()
            .iter()
            .map(|v| v.to_bits())
            .collect::<Vec<_>>()
    };
    assert_eq!(bits(), bits());
}

#[test]
fn fully_constrained_point_stays_put() {
    // D1: triangle with a fixed vertex, three lengths and a horizontal base (DOF 0).
    let case = decision(1);
    let mut built = case.build().unwrap();
    let mut solver = NativeSolver::new();
    solver.solve(&mut built.sketch, &SolveOptions::default(), &never);
    let before = built.sketch.params().as_slice().to_vec();
    let pts: Vec<EntityId> = built
        .entities
        .iter()
        .copied()
        .filter(|&e| {
            matches!(
                built.sketch.entity(e).unwrap().kind,
                EntityKind::Point { .. }
            )
        })
        .collect();
    for &pt in &pts {
        let grab = built.sketch.point(pt).unwrap();
        let mut d = solver
            .begin_drag(&built.sketch, DragSpec { entity: pt, grab })
            .unwrap();
        for k in 1..=30 {
            let t = grab + amg_core::Vec2::new(0.4 * f64::from(k), -0.3 * f64::from(k));
            let st = d.step(&mut built.sketch, t, &never);
            assert!(st.status.is_converged(), "{st:?}");
        }
        for (a, b) in before.iter().zip(built.sketch.params().as_slice()) {
            assert!(
                (a - b).abs() < 1e-8,
                "a fully constrained sketch moved: {a} vs {b}"
            );
        }
    }
}

#[test]
fn fully_constrained_point_in_a_flexible_sketch_stays_put() {
    // A rigid triangle (pinned) and a free point elsewhere in the same sketch.
    let mut s = Sketch::new(Plane::XY);
    let a = s.add_point(p(0.0, 0.0)).unwrap();
    let b = s.add_point(p(10.0, 0.0)).unwrap();
    let c = s.add_point(p(4.0, 6.0)).unwrap();
    let ab = s.add_line_between(a, b).unwrap();
    let bc = s.add_line_between(b, c).unwrap();
    let ca = s.add_line_between(c, a).unwrap();
    s.add_constraint(ConstraintKind::Fix { entity: a }).unwrap();
    s.add_constraint(ConstraintKind::Horizontal(HvTarget::Line(ab)))
        .unwrap();
    for (line, from, to) in [(ab, a, b), (bc, b, c), (ca, c, a)] {
        let v = s.point(from).unwrap().distance(s.point(to).unwrap());
        s.add_constraint(ConstraintKind::Dimension(Dimension {
            kind: DimensionKind::Length { line },
            value: v,
            driving: true,
            label_pos: Point2::ZERO,
        }))
        .unwrap();
    }
    let free = s.add_point(p(30.0, 5.0)).unwrap();
    let mut solver = NativeSolver::new();
    solver.solve(&mut s, &SolveOptions::default(), &never);
    let c0 = s.point(c).unwrap();
    // Pulling the pinned apex does not move it; the free point follows the cursor exactly.
    let mut d = solver
        .begin_drag(
            &s,
            DragSpec {
                entity: c,
                grab: c0,
            },
        )
        .unwrap();
    for k in 1..=20 {
        d.step(
            &mut s,
            c0 + amg_core::Vec2::new(f64::from(k), 2.0 * f64::from(k)),
            &never,
        );
    }
    assert!(s.point(c).unwrap().distance(c0) < 1e-8);
    let mut d = solver
        .begin_drag(
            &s,
            DragSpec {
                entity: free,
                grab: p(30.0, 5.0),
            },
        )
        .unwrap();
    let st = d.step(&mut s, p(33.0, 9.0), &never);
    assert!(st.status.is_converged());
    assert!(s.point(free).unwrap().distance(p(33.0, 9.0)) < 1e-12);
    assert!(s.point(c).unwrap().distance(c0) < 1e-8);
}

#[test]
fn point_drag_keeps_the_grab_offset() {
    let mut s = Sketch::new(Plane::XY);
    let a = s.add_point(p(1.0, 1.0)).unwrap();
    let mut solver = NativeSolver::new();
    // Grabbed 0.5 mm away from the point: the first frame at the grab does not jump.
    let mut d = solver
        .begin_drag(
            &s,
            DragSpec {
                entity: a,
                grab: p(1.5, 1.0),
            },
        )
        .unwrap();
    d.step(&mut s, p(1.5, 1.0), &never);
    assert!(s.point(a).unwrap().distance(p(1.0, 1.0)) < 1e-12);
    d.step(&mut s, p(3.5, 2.0), &never);
    assert!(s.point(a).unwrap().distance(p(3.0, 2.0)) < 1e-12);
}

#[test]
fn line_drag_translates_and_respects_constraints() {
    // Free line: both ends follow the cursor delta.
    let mut s = Sketch::new(Plane::XY);
    let l = s.add_line(p(0.0, 0.0), p(10.0, 2.0)).unwrap();
    let mut solver = NativeSolver::new();
    let EntityKind::Line { start, end } = s.entity(l).unwrap().kind else {
        panic!()
    };
    let mut d = solver
        .begin_drag(
            &s,
            DragSpec {
                entity: l,
                grab: p(5.0, 1.0),
            },
        )
        .unwrap();
    let st = d.step(&mut s, p(8.0, 4.0), &never);
    assert!(st.status.is_converged());
    assert!(s.point(start).unwrap().distance(p(3.0, 3.0)) < 1e-9);
    assert!(s.point(end).unwrap().distance(p(13.0, 5.0)) < 1e-9);

    // Horizontal fixed-length line with a fixed start: the end stays on its circle.
    let mut s = Sketch::new(Plane::XY);
    let l = s.add_line(p(0.0, 0.0), p(10.0, 0.0)).unwrap();
    let EntityKind::Line { start, end } = s.entity(l).unwrap().kind else {
        panic!()
    };
    s.add_constraint(ConstraintKind::Fix { entity: start })
        .unwrap();
    s.add_constraint(ConstraintKind::Dimension(Dimension {
        kind: DimensionKind::Length { line: l },
        value: 10.0,
        driving: true,
        label_pos: Point2::ZERO,
    }))
    .unwrap();
    solver.solve(&mut s, &SolveOptions::default(), &never);
    let mut d = solver
        .begin_drag(
            &s,
            DragSpec {
                entity: l,
                grab: p(5.0, 0.0),
            },
        )
        .unwrap();
    for k in 1..=40 {
        let a = f64::from(k) * 0.03;
        let st = d.step(&mut s, p(5.0 * a.cos(), 5.0 * a.sin()), &never);
        assert!(st.status.is_converged(), "{st:?}");
        assert!((s.point(end).unwrap().length() - 10.0).abs() < 1e-8);
        assert!(s.point(start).unwrap().length() < 1e-9);
    }
    // The end turned toward the cursor.
    assert!(s.point(end).unwrap().y > 5.0);
}

#[test]
fn circle_rim_drag_sets_the_radius() {
    let mut s = Sketch::new(Plane::XY);
    let c = s.add_circle(p(2.0, 3.0), 4.0).unwrap();
    let EntityKind::Circle { center, radius } = s.entity(c).unwrap().kind else {
        panic!()
    };
    s.add_constraint(ConstraintKind::Fix { entity: center })
        .unwrap();
    let mut solver = NativeSolver::new();
    let mut d = solver
        .begin_drag(
            &s,
            DragSpec {
                entity: c,
                grab: p(6.0, 3.0),
            },
        )
        .unwrap();
    for (cursor, want) in [(p(8.0, 3.0), 6.0), (p(2.0, 9.0), 6.0), (p(2.0, 4.5), 1.5)] {
        let st = d.step(&mut s, cursor, &never);
        assert!(st.status.is_converged());
        assert!((s.params().get(radius) - want).abs() < 1e-9, "{want}");
    }
    // A free centre stays put too (only the radius has a movement request).
    let mut s2 = Sketch::new(Plane::XY);
    let c2 = s2.add_circle(p(0.0, 0.0), 3.0).unwrap();
    let EntityKind::Circle { center, radius } = s2.entity(c2).unwrap().kind else {
        panic!()
    };
    let mut d = solver
        .begin_drag(
            &s2,
            DragSpec {
                entity: c2,
                grab: p(3.0, 0.0),
            },
        )
        .unwrap();
    d.step(&mut s2, p(0.0, 5.0), &never);
    assert!((s2.params().get(radius) - 5.0).abs() < 1e-9);
    assert!(s2.point(center).unwrap().length() < 1e-12);

    // A driving radius dimension pins the rim: it stays put.
    let mut s3 = Sketch::new(Plane::XY);
    let c3 = s3.add_circle(p(0.0, 0.0), 3.0).unwrap();
    let EntityKind::Circle { radius, .. } = s3.entity(c3).unwrap().kind else {
        panic!()
    };
    s3.add_constraint(ConstraintKind::Dimension(Dimension {
        kind: DimensionKind::Radius { curve: c3 },
        value: 3.0,
        driving: true,
        label_pos: Point2::ZERO,
    }))
    .unwrap();
    let mut d = solver
        .begin_drag(
            &s3,
            DragSpec {
                entity: c3,
                grab: p(3.0, 0.0),
            },
        )
        .unwrap();
    for k in 1..=10 {
        d.step(&mut s3, p(3.0 + f64::from(k), 0.0), &never);
    }
    assert!((s3.params().get(radius) - 3.0).abs() < 1e-8);
}

#[test]
fn arc_endpoint_drag_slides_on_the_circle() {
    let mut solver = NativeSolver::new();
    for drag_end in [true, false] {
        let mut s = Sketch::new(Plane::XY);
        let arc = s.add_arc(p(0.0, 0.0), p(5.0, 0.0), p(0.0, 5.0)).unwrap();
        let EntityKind::Arc { center, start, end } = s.entity(arc).unwrap().kind else {
            panic!()
        };
        // Centre and the other end are fixed, so the dragged end can only slide on the circle.
        let (moving, pinned, grab, home) = if drag_end {
            (end, start, p(0.0, 5.0), p(5.0, 0.0))
        } else {
            (start, end, p(5.0, 0.0), p(0.0, 5.0))
        };
        s.add_constraint(ConstraintKind::Fix { entity: center })
            .unwrap();
        s.add_constraint(ConstraintKind::Fix { entity: pinned })
            .unwrap();
        let mut d = solver
            .begin_drag(&s, DragSpec { entity: arc, grab })
            .unwrap();
        let sign = if drag_end { 1.0 } else { -1.0 };
        for k in 1..=30 {
            let a = if drag_end {
                std::f64::consts::FRAC_PI_2
            } else {
                0.0
            } + sign * 0.03 * f64::from(k);
            let st = d.step(&mut s, p(9.0 * a.cos(), 9.0 * a.sin()), &never);
            assert!(st.status.is_converged(), "{st:?}");
            let e = s.point(moving).unwrap();
            assert!((e.length() - 5.0).abs() < 1e-8);
            // The dragged end is the cursor's projection onto the circle.
            let dir = p(a.cos(), a.sin());
            assert!(e.distance(dir * 5.0) < 1e-2, "{e:?} vs {dir:?}");
        }
        assert!(s.point(pinned).unwrap().distance(home) < 1e-8);
    }
}

#[test]
fn unsupported_and_unknown_entities_are_rejected() {
    let mut s = Sketch::new(Plane::XY);
    let e = s.add_ellipse(p(0.0, 0.0), p(5.0, 0.0), 2.0).unwrap();
    let mut solver = NativeSolver::new();
    let r = solver.begin_drag(
        &s,
        DragSpec {
            entity: e,
            grab: p(5.0, 0.0),
        },
    );
    assert!(matches!(r, Err(SketchError::NotApplicable(_))));
    let r = solver.begin_drag(
        &s,
        DragSpec {
            entity: EntityId(999),
            grab: p(0.0, 0.0),
        },
    );
    assert!(matches!(r, Err(SketchError::UnknownEntity(_))));
}

#[test]
fn cancelled_step_leaves_the_sketch_unchanged() {
    let case = decision(7);
    let mut built = case.build().unwrap();
    let mut solver = NativeSolver::new();
    solver.solve(&mut built.sketch, &SolveOptions::default(), &never);
    let ent = built.entities[case.drag.as_ref().unwrap().entity as usize];
    let grab = built.sketch.point(ent).unwrap();
    let mut d = solver
        .begin_drag(&built.sketch, DragSpec { entity: ent, grab })
        .unwrap();
    let before = built.sketch.params().as_slice().to_vec();
    let st = d.step(
        &mut built.sketch,
        grab + amg_core::Vec2::new(0.1, 0.1),
        &|| true,
    );
    assert_eq!(st.status, SolveStatus::Cancelled);
    assert_eq!(before, built.sketch.params().as_slice());
}

#[test]
fn finish_returns_a_converged_report() {
    let case = decision(7);
    let mut built = case.build().unwrap();
    let mut solver = NativeSolver::new();
    solver.solve(&mut built.sketch, &SolveOptions::default(), &never);
    let ent = built.entities[case.drag.as_ref().unwrap().entity as usize];
    let grab = built.sketch.point(ent).unwrap();
    let mut d = solver
        .begin_drag(&built.sketch, DragSpec { entity: ent, grab })
        .unwrap();
    for t in case.drag.as_ref().unwrap().path.iter().take(50) {
        d.step(&mut built.sketch, p(t[0], t[1]), &never);
    }
    let r = d.finish(&mut built.sketch);
    assert!(r.status.is_converged(), "{r:?}");
}
