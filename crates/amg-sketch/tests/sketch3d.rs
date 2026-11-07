//! SK-12 3D sketch: a 3D polyline with lengths, angles and coincidence to model vertices
//! solves (Done-when 1, golden `tests/golden/sketch/sk12_polyline.json`), and the DOF analysis
//! is correct on hand-derived 3D cases (Done-when 2).
//!
//! `AMG_BLESS=1 cargo nextest run -p amg-sketch -E 'test(polyline_golden)'` rewrites the golden.

#![allow(
    clippy::float_cmp,
    clippy::unwrap_used,
    clippy::too_many_lines,
    reason = "test helpers; exact DOF counts and bit-identical repeats"
)]

use std::path::PathBuf;

use amg_core::Point3;
use amg_sketch::sketch3d::{Axis, Constraint3, Sketch3d, SplineEnd};
use amg_sketch::solve::{
    ConstraintState, DofAnalysis, GroupKind, NativeSolver, SolveOptions, SolveStatus,
};
use amg_sketch::{ConstraintId, EntityId};

fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

fn per_entity() -> SolveOptions {
    SolveOptions {
        dof: DofAnalysis::PerEntity,
        ..SolveOptions::default()
    }
}

fn never() -> bool {
    false
}

/// Largest residual of every constraint, re-evaluated from the solved geometry.
fn max_residual(s: &Sketch3d) -> f64 {
    let sys = s.system().unwrap();
    sys.system.max_abs_residual(s.params())
}

fn angle(u: Point3, v: Point3) -> f64 {
    u.cross(v).length().atan2(u.dot(v))
}

// ---- Done-when 1 ------------------------------------------------------------------------

/// A five-segment spatial polyline from model vertex `V0` to model vertex `V1`: every
/// segment has a length, consecutive segments an angle, one segment is parallel to Z and
/// one is perpendicular to the first. Returns the sketch, the polyline points and the ids of
/// the dimension constraints with their values.
fn polyline() -> (Sketch3d, Vec<EntityId>, Vec<(ConstraintId, f64)>) {
    // Target shape (consistent by construction): its lengths and angles are the dimensions,
    // its end points are the model vertices. The sketch starts from a perturbed copy.
    let target = [
        p(0.0, 0.0, 0.0),
        p(30.0, 0.0, 0.0),
        p(30.0, 0.0, 25.0),
        p(45.0, 20.0, 35.0),
        p(70.0, 25.0, 30.0),
        p(80.0, 50.0, 10.0),
    ];
    let mut s = Sketch3d::new();
    let v0 = s.add_model_vertex(target[0], None).unwrap();
    let v1 = s.add_model_vertex(target[5], None).unwrap();
    let wobble = [
        p(1.5, -2.0, 0.7),
        p(-1.0, 1.2, 2.0),
        p(2.2, 0.8, -1.4),
        p(-0.9, -1.7, 1.1),
        p(1.3, 2.4, -0.6),
        p(-2.1, 0.5, 1.9),
    ];
    let pts: Vec<EntityId> = target
        .iter()
        .zip(wobble)
        .map(|(&t, w)| s.add_point(t + w).unwrap())
        .collect();
    let lines: Vec<EntityId> = pts
        .windows(2)
        .map(|w| s.add_line(w[0], w[1]).unwrap())
        .collect();
    let mut dims = Vec::new();
    s.add_constraint(Constraint3::Coincident { a: pts[0], b: v0 })
        .unwrap();
    s.add_constraint(Constraint3::Coincident { a: pts[5], b: v1 })
        .unwrap();
    for (i, l) in lines.iter().enumerate() {
        let value = target[i].distance(target[i + 1]);
        let id = s
            .add_constraint(Constraint3::Length { line: *l, value })
            .unwrap();
        dims.push((id, value));
    }
    for i in 1..lines.len() {
        // Segment 1 → 2 is the right angle below; the others carry an angle dimension.
        if i == 1 {
            continue;
        }
        let value = angle(target[i] - target[i - 1], target[i + 1] - target[i]);
        let id = s
            .add_constraint(Constraint3::Angle {
                a: lines[i - 1],
                b: lines[i],
                value,
            })
            .unwrap();
        dims.push((id, value));
    }
    s.add_constraint(Constraint3::AlongAxis {
        line: lines[1],
        axis: Axis::Z,
    })
    .unwrap();
    s.add_constraint(Constraint3::Perpendicular {
        a: lines[0],
        b: lines[1],
    })
    .unwrap();
    (s, pts, dims)
}

#[test]
fn polyline_with_lengths_angles_and_model_vertices_solves() {
    let (mut s, pts, dims) = polyline();
    let mut solver = NativeSolver::new();
    let r = s.solve(&mut solver, &per_entity(), &never);
    assert_eq!(r.status, SolveStatus::Converged, "{r:?}");
    assert!(r.unsatisfied.is_empty());
    assert!(max_residual(&s) < 1e-9, "{}", max_residual(&s));
    // Ends sit exactly on the model vertices (substituted, so bit-exact).
    assert_eq!(s.point(pts[0]).unwrap(), p(0.0, 0.0, 0.0));
    assert_eq!(s.point(pts[5]).unwrap(), p(80.0, 50.0, 10.0));
    // Every dimension measures its value on the solved geometry.
    let at = |i: usize| s.point(pts[i]).unwrap();
    for (i, (_, value)) in dims.iter().take(5).enumerate() {
        assert!((at(i).distance(at(i + 1)) - value).abs() < 1e-9);
    }
    for (k, i) in [2usize, 3, 4].into_iter().enumerate() {
        let a = angle(at(i) - at(i - 1), at(i + 1) - at(i));
        assert!((a - dims[5 + k].1).abs() < 1e-9, "angle {i}: {a}");
    }
    let (d1, d2) = (at(1) - at(0), at(2) - at(1));
    assert!(d1.dot(d2).abs() < 1e-9);
    assert!(d2.x.abs() < 1e-12 && d2.y.abs() < 1e-12);
    // 4 interior points (12 unknowns, the ends are constants): 5 lengths + 3 angles + the
    // right angle + 2 axis rows = 11 independent equations, so 1 DOF remains (the polyline can
    // still swing about the V0–V1 chord).
    let dof = r.dof.unwrap();
    assert_eq!(dof.dof, 1);
    let (dof2, diag) = s.analyze(&solver);
    assert_eq!(dof2.dof, 1);
    assert!(diag.groups.is_empty());
}

#[test]
fn polyline_follows_a_moved_model_vertex() {
    let (mut s, pts, _) = polyline();
    let mut solver = NativeSolver::new();
    assert!(
        s.solve(&mut solver, &SolveOptions::default(), &never)
            .status
            .is_converged()
    );
    // `polyline` adds V0 then V1 first.
    let v1 = EntityId(1);
    s.move_model_vertex(v1, p(78.0, 52.0, 12.0)).unwrap();
    let r = s.solve(&mut solver, &SolveOptions::default(), &never);
    assert_eq!(r.status, SolveStatus::Converged, "{r:?}");
    assert_eq!(s.point(pts[5]).unwrap(), p(78.0, 52.0, 12.0));
    assert!(max_residual(&s) < 1e-9);
}

fn golden_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tests/golden/sketch/sk12_polyline.json")
}

#[test]
fn polyline_golden() {
    let (mut s, pts, _) = polyline();
    let r = s.solve(&mut NativeSolver::new(), &SolveOptions::default(), &never);
    assert!(r.status.is_converged());
    let got: Vec<[f64; 3]> = pts
        .iter()
        .map(|&id| s.point(id).unwrap().to_array())
        .collect();
    let path = golden_path();
    if std::env::var("AMG_BLESS").is_ok_and(|v| !v.is_empty() && v != "0") {
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        let text = serde_json::to_string_pretty(&serde_json::json!({ "points": got })).unwrap();
        std::fs::write(&path, text + "\n").unwrap();
        return;
    }
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("{}: {e} (run with AMG_BLESS=1)", path.display()));
    let want: serde_json::Value = serde_json::from_str(&text).unwrap();
    let want: Vec<[f64; 3]> = serde_json::from_value(want["points"].clone()).unwrap();
    assert_eq!(want.len(), got.len());
    for (w, g) in want.iter().zip(&got) {
        for k in 0..3 {
            assert!((w[k] - g[k]).abs() < 1e-9, "golden {w:?} vs solved {g:?}");
        }
    }
}

#[test]
fn same_input_gives_bit_identical_params() {
    let (base, _, _) = polyline();
    let mut first = None;
    let mut solver = NativeSolver::new();
    for i in 0..20 {
        let mut s = base.clone();
        if i % 2 == 0 {
            solver = NativeSolver::new();
        }
        s.solve(&mut solver, &SolveOptions::default(), &never);
        let x = s.params().to_vec();
        match &first {
            None => first = Some(x),
            Some(f) => assert_eq!(f, &x),
        }
    }
}

#[test]
fn cancelled_solve_leaves_the_sketch_unchanged() {
    let (mut s, _, _) = polyline();
    let before = s.clone();
    let r = s.solve(&mut NativeSolver::new(), &SolveOptions::default(), &|| true);
    assert_eq!(r.status, SolveStatus::Cancelled);
    assert_eq!(s, before);
}

#[test]
fn serde_round_trip() {
    let (s, _, _) = polyline();
    let text = serde_json::to_string(&s).unwrap();
    let back: Sketch3d = serde_json::from_str(&text).unwrap();
    assert_eq!(back, s);
}

// ---- Done-when 2: DOF cases ------------------------------------------------------------

/// One hand-derived DOF case: the sketch, the expected total DOF and expected
/// `(entity, state, free_dof)` for selected entities.
struct Case {
    name: &'static str,
    sketch: Sketch3d,
    dof: u32,
    entities: Vec<(EntityId, ConstraintState, u8)>,
    status: SolveStatus,
}

fn cases() -> Vec<Case> {
    use ConstraintState::{Full, Under};
    let mut out = Vec::new();
    let ok = SolveStatus::Converged;

    // 1. One free point: 3 coordinates, nothing else.
    {
        let mut s = Sketch3d::new();
        let a = s.add_point(p(1.0, 2.0, 3.0)).unwrap();
        out.push(Case {
            name: "free point",
            sketch: s,
            dof: 3,
            entities: vec![(a, Under, 3)],
            status: ok,
        });
    }
    // 2. Line from a model vertex with a length: the free end lies on a sphere, 3 − 1 = 2.
    {
        let mut s = Sketch3d::new();
        let v = s.add_model_vertex(p(0.0, 0.0, 0.0), None).unwrap();
        let e = s.add_point(p(3.0, 4.0, 1.0)).unwrap();
        let l = s.add_line(v, e).unwrap();
        s.add_constraint(Constraint3::Length {
            line: l,
            value: 10.0,
        })
        .unwrap();
        out.push(Case {
            name: "vertex + length",
            sketch: s,
            dof: 2,
            entities: vec![(v, Full, 0), (e, Under, 2), (l, Under, 2)],
            status: ok,
        });
    }
    // 3. Case 2 plus parallel to Z: x and y of the end are pinned (substituted), the length
    //    fixes z (isolated ±10 solutions): 0.
    {
        let mut s = Sketch3d::new();
        let v = s.add_model_vertex(p(0.0, 0.0, 0.0), None).unwrap();
        let e = s.add_point(p(0.3, -0.2, 9.0)).unwrap();
        let l = s.add_line(v, e).unwrap();
        s.add_constraint(Constraint3::Length {
            line: l,
            value: 10.0,
        })
        .unwrap();
        s.add_constraint(Constraint3::AlongAxis {
            line: l,
            axis: Axis::Z,
        })
        .unwrap();
        out.push(Case {
            name: "vertex + length + axis",
            sketch: s,
            dof: 0,
            entities: vec![(e, Full, 0), (l, Full, 0)],
            status: ok,
        });
    }
    // 4. Free triangle with three side lengths: a rigid body in 3D, 9 − 3 = 6.
    {
        let mut s = Sketch3d::new();
        let a = s.add_point(p(0.0, 0.0, 0.0)).unwrap();
        let b = s.add_point(p(10.0, 1.0, 0.5)).unwrap();
        let c = s.add_point(p(3.0, 8.0, -1.0)).unwrap();
        for (x, y, v) in [(a, b, 10.0), (b, c, 9.0), (c, a, 8.0)] {
            let l = s.add_line(x, y).unwrap();
            s.add_constraint(Constraint3::Length { line: l, value: v })
                .unwrap();
        }
        out.push(Case {
            name: "rigid triangle",
            sketch: s,
            dof: 6,
            entities: vec![(a, Under, 3)],
            status: ok,
        });
    }
    // 5. Tetrahedron of free points with all six edge lengths: rigid, 12 − 6 = 6.
    {
        let mut s = Sketch3d::new();
        let q = [
            p(0.0, 0.0, 0.0),
            p(10.0, 0.0, 0.0),
            p(4.0, 9.0, 0.0),
            p(4.0, 3.0, 8.0),
        ];
        let ids: Vec<EntityId> = q
            .iter()
            .map(|&x| s.add_point(x + p(0.3, -0.2, 0.1)).unwrap())
            .collect();
        for i in 0..4 {
            for j in i + 1..4 {
                s.add_constraint(Constraint3::Distance {
                    a: ids[i],
                    b: ids[j],
                    value: q[i].distance(q[j]),
                })
                .unwrap();
            }
        }
        out.push(Case {
            name: "rigid tetrahedron",
            sketch: s,
            dof: 6,
            entities: vec![],
            status: ok,
        });
    }
    // 6. Two-bar chain from a model vertex, both lengths and the angle between the bars: the
    //    triangle V–A–B is rigid and pivots about V, so 6 − 3 = 3 rotational DOF.
    {
        let mut s = Sketch3d::new();
        let v = s.add_model_vertex(p(0.0, 0.0, 0.0), None).unwrap();
        let a = s.add_point(p(10.0, 1.0, 0.0)).unwrap();
        let b = s.add_point(p(14.0, 9.0, 3.0)).unwrap();
        let l1 = s.add_line(v, a).unwrap();
        let l2 = s.add_line(a, b).unwrap();
        s.add_constraint(Constraint3::Length {
            line: l1,
            value: 10.0,
        })
        .unwrap();
        s.add_constraint(Constraint3::Length {
            line: l2,
            value: 8.0,
        })
        .unwrap();
        s.add_constraint(Constraint3::Angle {
            a: l1,
            b: l2,
            value: 1.0,
        })
        .unwrap();
        out.push(Case {
            name: "two-bar pivot",
            sketch: s,
            dof: 3,
            // Velocities are ω × r: each point moves on its sphere about V (2), the bar A–B
            // sees all three rotations (3).
            entities: vec![(a, Under, 2), (b, Under, 2), (l1, Under, 2), (l2, Under, 3)],
            status: ok,
        });
    }
    // 7. Point on a line between two model vertices: 3 coordinates + 1 line parameter − 3
    //    equations = 1 (slides along the line).
    {
        let mut s = Sketch3d::new();
        let v0 = s.add_model_vertex(p(0.0, 0.0, 0.0), None).unwrap();
        let v1 = s.add_model_vertex(p(10.0, 5.0, 2.0), None).unwrap();
        let l = s.add_line(v0, v1).unwrap();
        let q = s.add_point(p(4.0, 3.0, 0.0)).unwrap();
        s.add_constraint(Constraint3::PointOnLine { point: q, line: l })
            .unwrap();
        out.push(Case {
            name: "point on fixed line",
            sketch: s,
            dof: 1,
            entities: vec![(q, Under, 1), (l, Full, 0)],
            status: ok,
        });
    }
    // 8. Point on a spline whose four poles are model vertices: 3 + 1 − 3 = 1.
    {
        let mut s = Sketch3d::new();
        let poles: Vec<EntityId> = [
            p(0.0, 0.0, 0.0),
            p(5.0, 8.0, 1.0),
            p(12.0, 6.0, 4.0),
            p(18.0, 0.0, 2.0),
        ]
        .iter()
        .map(|&x| s.add_model_vertex(x, None).unwrap())
        .collect();
        let sp = s.add_spline(3, &poles).unwrap();
        let q = s.add_point(p(9.0, 5.0, 2.0)).unwrap();
        s.add_constraint(Constraint3::PointOnSpline {
            point: q,
            spline: sp,
        })
        .unwrap();
        out.push(Case {
            name: "point on fixed spline",
            sketch: s,
            dof: 1,
            entities: vec![(q, Under, 1), (sp, Full, 0)],
            status: ok,
        });
    }
    // 9. Redundant: a line from a model vertex parallel to Z twice. The second axis
    //    constraint aliases parameters that are already aliased; z stays free: 1 DOF,
    //    `ConvergedRedundant`, one Redundant group of both constraints. The constraints
    //    reference the line only, so the line is `Over` and its end point is not.
    {
        let mut s = Sketch3d::new();
        let v = s.add_model_vertex(p(0.0, 0.0, 0.0), None).unwrap();
        let e = s.add_point(p(0.5, 0.5, 7.0)).unwrap();
        let l = s.add_line(v, e).unwrap();
        s.add_constraint(Constraint3::AlongAxis {
            line: l,
            axis: Axis::Z,
        })
        .unwrap();
        s.add_constraint(Constraint3::AlongAxis {
            line: l,
            axis: Axis::Z,
        })
        .unwrap();
        out.push(Case {
            name: "redundant axis",
            sketch: s,
            dof: 1,
            entities: vec![(e, Under, 1), (l, ConstraintState::Over, 1)],
            status: SolveStatus::ConvergedRedundant,
        });
    }
    // 10. L-shape from a model vertex: two lines V–A and V–B, perpendicular, both lengths:
    //     6 − 3 = 3 (rigid, pivoting about V).
    {
        let mut s = Sketch3d::new();
        let v = s.add_model_vertex(p(1.0, 1.0, 1.0), None).unwrap();
        let a = s.add_point(p(11.0, 2.0, 1.0)).unwrap();
        let b = s.add_point(p(0.0, 9.0, 3.0)).unwrap();
        let l1 = s.add_line(v, a).unwrap();
        let l2 = s.add_line(v, b).unwrap();
        s.add_constraint(Constraint3::Perpendicular { a: l1, b: l2 })
            .unwrap();
        s.add_constraint(Constraint3::Length {
            line: l1,
            value: 10.0,
        })
        .unwrap();
        s.add_constraint(Constraint3::Length {
            line: l2,
            value: 8.0,
        })
        .unwrap();
        out.push(Case {
            name: "perpendicular L",
            sketch: s,
            dof: 3,
            entities: vec![(a, Under, 2), (b, Under, 2)],
            status: ok,
        });
    }
    // 11. Spline tangent to a fixed line at its start, which sits on a model vertex: poles
    //     P1..P3 free (9), tangency = 3 equations + 1 ratio: 9 − 2 = 7. P1 slides on the line.
    {
        let mut s = Sketch3d::new();
        let v0 = s.add_model_vertex(p(-10.0, 0.0, 0.0), None).unwrap();
        let v1 = s.add_model_vertex(p(0.0, 0.0, 0.0), None).unwrap();
        let l = s.add_line(v0, v1).unwrap();
        let p1 = s.add_point(p(4.0, 0.5, 0.2)).unwrap();
        let p2 = s.add_point(p(8.0, 5.0, 2.0)).unwrap();
        let p3 = s.add_point(p(12.0, 4.0, 6.0)).unwrap();
        let sp = s.add_spline(3, &[v1, p1, p2, p3]).unwrap();
        s.add_constraint(Constraint3::Tangent {
            line: l,
            spline: sp,
            end: SplineEnd::Start,
        })
        .unwrap();
        out.push(Case {
            name: "tangent spline",
            sketch: s,
            dof: 7,
            entities: vec![(p1, Under, 1), (p2, Under, 3), (sp, Under, 7), (l, Full, 0)],
            status: ok,
        });
    }
    out
}

/// Every case with the default solver, every block sparse and without the DM split.
#[test]
fn dof_cases_match_hand_derived_values() {
    let solvers = || {
        [
            NativeSolver::new(),
            NativeSolver::new().with_dense_limit(0),
            NativeSolver::new().with_decomposition(false),
        ]
    };
    assert!(cases().len() >= 10);
    for (k, mut solver) in solvers().into_iter().enumerate() {
        for mut case in cases() {
            let r = case.sketch.solve(&mut solver, &per_entity(), &never);
            let tag = format!("{} (solver {k})", case.name);
            assert_eq!(r.status, case.status, "{tag}: {r:?}");
            let dof = r.dof.as_ref().unwrap();
            assert_eq!(dof.dof, case.dof, "{tag}: total DOF");
            let (analysed, diag) = case.sketch.analyze(&solver);
            assert_eq!(&analysed, dof, "{tag}: analyze == solve report");
            for (id, state, free) in &case.entities {
                let st = dof.entities[id];
                assert_eq!(
                    (st.state, st.free_dof),
                    (*state, *free),
                    "{tag}: entity {id:?}"
                );
            }
            if case.status == SolveStatus::ConvergedRedundant {
                assert_eq!(diag.groups.len(), 1, "{tag}: {diag:?}");
                assert_eq!(diag.groups[0].kind, GroupKind::Redundant);
                assert_eq!(
                    diag.groups[0].constraints,
                    vec![ConstraintId(0), ConstraintId(1)]
                );
            } else {
                assert!(diag.groups.is_empty(), "{tag}: {diag:?}");
            }
        }
    }
}

#[test]
fn coincidence_with_two_different_model_vertices_conflicts() {
    let mut s = Sketch3d::new();
    let v0 = s.add_model_vertex(p(0.0, 0.0, 0.0), None).unwrap();
    let v1 = s.add_model_vertex(p(1.0, 0.0, 0.0), None).unwrap();
    let q = s.add_point(p(0.5, 0.5, 0.5)).unwrap();
    let c0 = s
        .add_constraint(Constraint3::Coincident { a: q, b: v0 })
        .unwrap();
    let c1 = s
        .add_constraint(Constraint3::Coincident { a: q, b: v1 })
        .unwrap();
    let r = s.solve(&mut NativeSolver::new(), &SolveOptions::default(), &never);
    assert_eq!(r.status, SolveStatus::Inconsistent, "{r:?}");
    let diag = r.diagnosis.unwrap();
    assert_eq!(diag.groups.len(), 1, "{diag:?}");
    assert_eq!(diag.groups[0].kind, GroupKind::Conflicting);
    assert_eq!(diag.groups[0].constraints, vec![c0, c1]);
}

#[test]
fn invalid_constraints_are_rejected() {
    let mut s = Sketch3d::new();
    let a = s.add_point(p(0.0, 0.0, 0.0)).unwrap();
    let l = s
        .add_line_points(p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0))
        .unwrap();
    assert!(
        s.add_constraint(Constraint3::Length {
            line: a,
            value: 1.0
        })
        .is_err()
    );
    assert!(
        s.add_constraint(Constraint3::Length {
            line: l,
            value: -1.0
        })
        .is_err()
    );
    assert!(
        s.add_constraint(Constraint3::Angle {
            a: l,
            b: l,
            value: 0.0
        })
        .is_err()
    );
    assert!(
        s.add_constraint(Constraint3::Coincident { a, b: a })
            .is_err()
    );
    let v = s.add_model_vertex(p(0.0, 0.0, 0.0), None).unwrap();
    assert!(s.set_fixed(v, false).is_err());
    assert!(s.add_point(p(f64::NAN, 0.0, 0.0)).is_err());
}

#[test]
fn fixed_point_is_not_moved() {
    let mut s = Sketch3d::new();
    let l = s
        .add_line_points(p(0.0, 0.0, 0.0), p(3.0, 4.0, 0.0))
        .unwrap();
    let (a, _) = match s.entity(l).unwrap() {
        amg_sketch::sketch3d::Entity3::Line { start, end } => (*start, *end),
        _ => unreachable!(),
    };
    s.set_fixed(a, true).unwrap();
    s.add_constraint(Constraint3::Length {
        line: l,
        value: 10.0,
    })
    .unwrap();
    let r = s.solve(&mut NativeSolver::new(), &per_entity(), &never);
    assert_eq!(r.status, SolveStatus::Converged);
    assert_eq!(s.point(a).unwrap(), p(0.0, 0.0, 0.0));
    let (a2, b2) = s.line(l).unwrap();
    assert!((a2.distance(b2) - 10.0).abs() < 1e-9);
    assert_eq!(r.dof.unwrap().dof, 2);
}
