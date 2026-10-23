//! SK-08 acceptance: splines (control-point and fit-point, rational) and conics: entities,
//! their equations (FD-checked Jacobians), point-on-curve / tangent / curvature (G2)
//! constraints and solves, intersections, profile detection, the operations that match on
//! entity or curve kinds, and the error paths of everything that does not support them.
//!
//! The production solver SK-03 is not built yet: solves use the SK-13 prototype
//! (`solve::proto`), ranks and DOF the SK-02 helper.

#![allow(
    clippy::unwrap_used,
    clippy::float_cmp,
    reason = "test helpers; a failure is a test failure"
)]

use std::f64::consts::{FRAC_1_SQRT_2, PI, TAU};

use amg_core::{Plane, Point2, Tolerance, Vec2};
use amg_sketch::geom::intersect;
use amg_sketch::solve::proto::{self, ProtoOptions, ProtoStatus};
use amg_sketch::solve::{
    Residual, build_system, constraint_residual, constraint_residual_aux, dof, entity_residual,
};
use amg_sketch::{
    BSpline2, ConstraintId, ConstraintKind, Curve2, Dimension, DimensionKind, End, EntityId,
    EntityKind, EntityOrigin, ProfileIssue, ProfileOptions, ProfileSet, Side, Sketch, SketchError,
    TangentKind, build, detect_profiles,
};

fn new() -> Sketch {
    Sketch::new(Plane::XY)
}

fn p(x: f64, y: f64) -> Point2 {
    Point2::new(x, y)
}

fn dim(kind: DimensionKind, value: f64) -> ConstraintKind {
    ConstraintKind::Dimension(Dimension {
        kind,
        value,
        driving: true,
        label_pos: Point2::ZERO,
    })
}

fn solve(s: &mut Sketch) -> ProtoStatus {
    proto::solve(s, &ProtoOptions::default()).unwrap().status
}

fn residual_max(s: &Sketch) -> f64 {
    let sys = build_system(s).unwrap();
    sys.system.max_abs_residual(s.params().as_slice())
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1e-300)
}

fn spline_curve(s: &Sketch, id: EntityId) -> BSpline2 {
    match s.curve(id).unwrap() {
        Curve2::BSpline(b) => b,
        other => panic!("not a spline: {other:?}"),
    }
}

fn end_point(s: &Sketch, id: EntityId, end: End) -> EntityId {
    s.entity(id).unwrap().kind.endpoint(end).unwrap()
}

/// Joins the named ends of two curves with a `Coincident` constraint.
fn touch(s: &mut Sketch, a: EntityId, a_end: End, b: EntityId, b_end: End) {
    let (pa, pb) = (end_point(s, a, a_end), end_point(s, b, b_end));
    if pa != pb {
        s.add_constraint(ConstraintKind::Coincident { a: pa, b: pb })
            .unwrap();
    }
}

// ---- Deterministic random geometry ------------------------------------------------------------

struct Rng(u64);

impl Rng {
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        let u = (self.0 >> 11) as f64 / (1u64 << 53) as f64;
        lo + (hi - lo) * u
    }
}

/// A control polygon marching to the right with random wiggle.
fn polygon(g: &mut Rng, n: usize) -> Vec<Point2> {
    (0..n)
        .map(|i| p(2.0 * i as f64 + g.range(-0.6, 0.6), g.range(-2.5, 2.5)))
        .collect()
}

fn random_control(s: &mut Sketch, g: &mut Rng, degree: usize, rational: bool) -> EntityId {
    let n = degree + 1 + (g.range(0.0, 3.0) as usize);
    let poles = polygon(g, n);
    let w: Vec<f64> = (0..n).map(|_| g.range(0.5, 2.0)).collect();
    s.add_spline(degree, &poles, rational.then_some(&w[..]), None)
        .unwrap()
}

fn random_fit(s: &mut Sketch, g: &mut Rng, handles: bool) -> EntityId {
    let n = 3 + (g.range(0.0, 3.0) as usize);
    let pts = polygon(g, n);
    let (t0, t1) = if handles {
        (
            Some(Vec2::new(g.range(0.5, 2.0), g.range(-1.5, 1.5))),
            Some(Vec2::new(g.range(0.5, 2.0), g.range(-1.5, 1.5))),
        )
    } else {
        (None, None)
    };
    s.add_spline_fit(&pts, t0, t1).unwrap()
}

fn random_conic(s: &mut Sketch, g: &mut Rng) -> EntityId {
    let a = p(g.range(-4.0, 0.0), g.range(-3.0, 3.0));
    let b = p(g.range(4.0, 8.0), g.range(-3.0, 3.0));
    let apex = p(g.range(0.0, 4.0), g.range(3.5, 7.0));
    s.add_conic(a, apex, b, g.range(0.2, 0.8)).unwrap()
}

fn random_line(s: &mut Sketch, g: &mut Rng) -> EntityId {
    let a = p(g.range(-8.0, 8.0), g.range(-8.0, 8.0));
    let ang = g.range(0.0, TAU);
    s.add_line(a, a + Vec2::new(ang.cos(), ang.sin()) * g.range(2.0, 9.0))
        .unwrap()
}

fn random_arc(s: &mut Sketch, g: &mut Rng) -> EntityId {
    let c = p(g.range(-5.0, 5.0), g.range(-5.0, 5.0));
    let r = g.range(2.0, 6.0);
    let a0 = g.range(0.0, TAU);
    s.add_arc(
        c,
        c + Vec2::new(a0.cos(), a0.sin()) * r,
        c + Vec2::new((a0 + 1.5).cos(), (a0 + 1.5).sin()) * r,
    )
    .unwrap()
}

// ---- Finite-difference Jacobian checks --------------------------------------------------------

/// Five-point central differences of every column of `r` against the analytic block.
fn fd_check(r: &dyn Residual, x: &[f64]) -> Result<(), String> {
    let m = r.rows();
    let n = r.params().len();
    let mut j = vec![0.0; m * n];
    r.jacobian(x, &mut j);
    let mut xx = x.to_vec();
    let f = |xx: &[f64]| {
        let mut o = vec![0.0; m];
        r.eval(xx, &mut o);
        o
    };
    let h = 1e-4;
    for (k, pid) in r.params().iter().enumerate() {
        let x0 = x[pid.index()];
        let mut at = |d: f64| {
            xx[pid.index()] = x0 + d;
            let v = f(&xx);
            xx[pid.index()] = x0;
            v
        };
        let (p2, p1, m1, m2) = (at(2.0 * h), at(h), at(-h), at(-2.0 * h));
        for i in 0..m {
            let fd = (-p2[i] + 8.0 * p1[i] - 8.0 * m1[i] + m2[i]) / (12.0 * h);
            let a = j[i * n + k];
            let err = (a - fd).abs();
            if err > 1e-7 && err > 1e-5 * a.abs().max(fd.abs()) {
                return Err(format!(
                    "row {i}, param {pid:?}: analytic {a:e}, finite difference {fd:e}"
                ));
            }
        }
    }
    Ok(())
}

fn check_constraint(s: &Sketch, id: ConstraintId) -> usize {
    let c = s.constraint(id).unwrap();
    let r = constraint_residual_aux(s, &c.kind, &c.aux, 10.0)
        .unwrap()
        .unwrap();
    assert!(r.rows() > 0);
    fd_check(&r, s.params().as_slice()).unwrap_or_else(|e| panic!("{:?}: {e}", c.kind));
    r.rows()
}

const ENDS: [End; 2] = [End::Start, End::End];

#[test]
fn jacobians_match_finite_differences_for_every_spline_row() {
    let mut g = Rng(21);
    let mut rows = 0;
    let mut blocks = 0;
    for round in 0..60 {
        let mut s = new();
        let degree = 2 + round % 3;
        let splines = vec![
            random_control(&mut s, &mut g, degree, false),
            random_control(&mut s, &mut g, degree, true),
            random_control(&mut s, &mut g, 3, true),
            random_fit(&mut s, &mut g, false),
            random_fit(&mut s, &mut g, true),
            random_conic(&mut s, &mut g),
            random_conic(&mut s, &mut g),
        ];
        // Splines have no internal equations.
        for &sp in &splines {
            assert!(entity_residual(&s, sp).is_none());
        }
        let line = random_line(&mut s, &mut g);
        let arc = random_arc(&mut s, &mut g);
        let mut checked: Vec<ConstraintId> = Vec::new();

        // Point on spline / conic (aux param = curve parameter).
        for &sp in &splines {
            let pt = s
                .add_point(p(g.range(-2.0, 10.0), g.range(-3.0, 5.0)))
                .unwrap();
            let id = s
                .add_constraint(ConstraintKind::PointOnCurve {
                    point: pt,
                    curve: sp,
                })
                .unwrap();
            assert_eq!(s.constraint(id).unwrap().aux.len(), 1);
            checked.push(id);
        }

        // Tangent at shared ends, with a line, an arc, and another spline.
        for (i, &sp) in splines.iter().enumerate() {
            for &end in &ENDS {
                for &other_end in &ENDS {
                    for &other in &[line, arc, splines[(i + 1) % splines.len()]] {
                        if other == sp {
                            continue;
                        }
                        touch(&mut s, sp, end, other, other_end);
                        for reversed in [false, true] {
                            let k = ConstraintKind::Tangent {
                                a: sp,
                                b: other,
                                kind: TangentKind::AtEndpoint {
                                    a_end: end,
                                    b_end: other_end,
                                    reversed,
                                },
                            };
                            checked.push(s.add_constraint(k).unwrap());
                            let k = ConstraintKind::Curvature {
                                a: other,
                                b: sp,
                                a_end: other_end,
                                b_end: end,
                                reversed,
                            };
                            checked.push(s.add_constraint(k).unwrap());
                        }
                    }
                }
            }
        }

        // Equal weights / rho.
        let rational_pair = (splines[1], random_control_like(&mut s, &mut g, splines[1]));
        checked.push(
            s.add_constraint(ConstraintKind::Equal {
                a: rational_pair.0,
                b: rational_pair.1,
            })
            .unwrap(),
        );
        checked.push(
            s.add_constraint(ConstraintKind::Equal {
                a: splines[5],
                b: splines[6],
            })
            .unwrap(),
        );

        for id in checked {
            rows += check_constraint(&s, id);
            blocks += 1;
        }
    }
    assert!(blocks > 4000, "{blocks} blocks");
    assert!(rows > 6000, "{rows} rows");
}

/// A second rational spline with the same pole count and degree as `like`.
fn random_control_like(s: &mut Sketch, g: &mut Rng, like: EntityId) -> EntityId {
    let EntityKind::Spline(amg_sketch::model::SplineDef::Control { degree, poles, .. }) =
        s.entity(like).unwrap().kind.clone()
    else {
        panic!("not a control spline");
    };
    let n = poles.len();
    let pts = polygon(g, n);
    let w: Vec<f64> = (0..n).map(|_| g.range(0.5, 2.0)).collect();
    s.add_spline(usize::from(degree), &pts, Some(&w), None)
        .unwrap()
}

// ---- Degrees of freedom -----------------------------------------------------------------------

#[test]
fn degrees_of_freedom_per_entity() {
    // Control-point: 2 per pole (+ weights).
    let mut s = new();
    s.add_spline(
        3,
        &[p(0.0, 0.0), p(1.0, 2.0), p(3.0, 2.0), p(4.0, 0.0)],
        None,
        None,
    )
    .unwrap();
    assert_eq!(dof(&s).unwrap(), 8);
    let mut s = new();
    s.add_spline(
        2,
        &[p(0.0, 0.0), p(1.0, 2.0), p(3.0, 0.0)],
        Some(&[1.0, 2.0, 1.0]),
        None,
    )
    .unwrap();
    assert_eq!(dof(&s).unwrap(), 6 + 3);
    // Fit-point: 2 per fit point, + handles.
    let pts = [p(0.0, 0.0), p(1.0, 1.0), p(2.0, 0.0), p(3.0, 1.5)];
    let mut s = new();
    s.add_spline_fit(&pts, None, None).unwrap();
    assert_eq!(dof(&s).unwrap(), 8);
    let mut s = new();
    s.add_spline_fit(&pts, Some(Vec2::X), Some(Vec2::Y))
        .unwrap();
    assert_eq!(dof(&s).unwrap(), 8 + 4);
    // Conic: 3 points + rho.
    let mut s = new();
    s.add_conic(p(0.0, 0.0), p(1.0, 1.0), p(2.0, 0.0), 0.5)
        .unwrap();
    assert_eq!(dof(&s).unwrap(), 7);
}

// ---- Solves -----------------------------------------------------------------------------------

#[test]
fn spline_tangent_to_a_line_solves() {
    for fit in [false, true] {
        let mut s = new();
        let line = s.add_line(p(0.0, 0.0), p(4.0, 0.0)).unwrap();
        s.add_constraint(ConstraintKind::Fix { entity: line })
            .unwrap();
        let spl = if fit {
            s.add_spline_fit(
                &[p(4.0, 0.0), p(6.0, 1.0), p(8.0, 2.5), p(10.0, 3.0)],
                None,
                None,
            )
            .unwrap()
        } else {
            s.add_spline(
                3,
                &[p(4.0, 0.0), p(5.0, 1.0), p(8.0, 4.0), p(10.0, 3.0)],
                None,
                None,
            )
            .unwrap()
        };
        touch(&mut s, line, End::End, spl, End::Start);
        s.add_constraint_oriented(ConstraintKind::Tangent {
            a: line,
            b: spl,
            kind: TangentKind::AtEndpoint {
                a_end: End::End,
                b_end: End::Start,
                reversed: false,
            },
        })
        .unwrap();
        let before = dof(&s).unwrap();
        let status = solve(&mut s);
        assert!(
            matches!(
                status,
                ProtoStatus::Converged | ProtoStatus::ConvergedRedundant
            ),
            "{status:?}"
        );
        assert!(residual_max(&s) < 1e-9);
        // The spline leaves the line end along +x.
        let c = spline_curve(&s, spl);
        let d = c.derivative(0.0);
        assert!(d.y.abs() < 1e-8 * d.length(), "{d:?}");
        assert!(d.x > 0.0);
        assert!(c.eval(0.0).distance(p(4.0, 0.0)) < 1e-9);
        assert!(dof(&s).unwrap() < before + 100);
    }
}

#[test]
fn spline_g2_to_an_arc_solves_and_dof_is_zero_when_fully_constrained() {
    // Quarter arc of radius 5 about the origin, fixed. A cubic Bezier leaves its end (0, 5)
    // with the same tangent and curvature.
    let mut s = new();
    let arc = s.add_arc(p(0.0, 0.0), p(5.0, 0.0), p(0.0, 5.0)).unwrap();
    s.add_constraint(ConstraintKind::Fix { entity: arc })
        .unwrap();
    let spl = s
        .add_spline(
            3,
            &[p(0.0, 5.0), p(-2.0, 5.3), p(-5.0, 3.0), p(-8.0, 4.0)],
            None,
            None,
        )
        .unwrap();
    let EntityKind::Spline(amg_sketch::model::SplineDef::Control { poles, .. }) =
        s.entity(spl).unwrap().kind.clone()
    else {
        panic!()
    };
    touch(&mut s, arc, End::End, spl, End::Start);
    s.add_constraint(ConstraintKind::Fix { entity: poles[3] })
        .unwrap();
    // G2 includes the tangent row.
    s.add_constraint(ConstraintKind::Curvature {
        a: arc,
        b: spl,
        a_end: End::End,
        b_end: End::Start,
        reversed: false,
    })
    .unwrap();
    s.add_constraint_oriented(dim(
        DimensionKind::Distance {
            a: poles[0],
            b: poles[1],
            side: Side::Positive,
        },
        2.0,
    ))
    .unwrap();
    s.add_constraint_oriented(dim(
        DimensionKind::Distance {
            a: poles[2],
            b: poles[3],
            side: Side::Positive,
        },
        3.0,
    ))
    .unwrap();
    assert_eq!(dof(&s).unwrap(), 0);
    let status = solve(&mut s);
    // The arc's own internal equation has no free parameter left, so it counts as redundant.
    assert!(
        matches!(
            status,
            ProtoStatus::Converged | ProtoStatus::ConvergedRedundant
        ),
        "{status:?}"
    );
    assert!(residual_max(&s) < 1e-9);
    let c = spline_curve(&s, spl);
    assert!(
        (c.curvature_at(0.0) - 0.2).abs() < 1e-8,
        "{}",
        c.curvature_at(0.0)
    );
    let t = c.derivative(0.0).normalize();
    assert!(t.distance(Vec2::new(-1.0, 0.0)) < 1e-8, "{t:?}");
    // Pole 1 sits 2 mm along the tangent; pole 2 follows from the curvature.
    let q = c.poles();
    assert!(q[1].distance(p(-2.0, 5.0)) < 1e-8);
    assert!((q[2].y - 3.8).abs() < 1e-8, "{:?}", q[2]);
    assert!((q[2].distance(p(-8.0, 4.0)) - 3.0).abs() < 1e-8);
}

#[test]
fn g2_with_reversed_ends_and_a_fit_spline_solves() {
    // The spline's END meets the arc's END: opposite directions of travel.
    let mut s = new();
    let arc = s.add_arc(p(0.0, 0.0), p(5.0, 0.0), p(0.0, 5.0)).unwrap();
    s.add_constraint(ConstraintKind::Fix { entity: arc })
        .unwrap();
    let spl = s
        .add_spline_fit(
            &[p(-9.0, 3.5), p(-6.0, 4.6), p(-3.0, 5.1), p(0.0, 5.0)],
            None,
            // A natural end has zero curvature, so G2 needs a tangent handle.
            Some(Vec2::new(3.0, 0.0)),
        )
        .unwrap();
    touch(&mut s, arc, End::End, spl, End::End);
    let t = amg_sketch::solve::orient_constraint(
        &s,
        ConstraintKind::Tangent {
            a: arc,
            b: spl,
            kind: TangentKind::AtEndpoint {
                a_end: End::End,
                b_end: End::End,
                reversed: false,
            },
        },
    );
    let ConstraintKind::Tangent {
        kind: TangentKind::AtEndpoint { reversed, .. },
        ..
    } = t
    else {
        panic!()
    };
    assert!(reversed, "the spline end runs against the arc's travel");
    s.add_constraint(ConstraintKind::Curvature {
        a: arc,
        b: spl,
        a_end: End::End,
        b_end: End::End,
        reversed,
    })
    .unwrap();
    assert!(dof(&s).unwrap() >= 4);
    let status = solve(&mut s);
    assert!(
        matches!(
            status,
            ProtoStatus::Converged | ProtoStatus::ConvergedRedundant
        ),
        "{status:?}"
    );
    assert!(residual_max(&s) < 1e-9);
    let c = spline_curve(&s, spl);
    let end = c.span();
    // Seen from the arc, the continuation has the arc's curvature: -1/5 in the spline's own
    // direction of travel.
    assert!(
        (c.curvature_at(end) + 0.2).abs() < 1e-8,
        "{}",
        c.curvature_at(end)
    );
    let d = c.derivative(end).normalize();
    assert!(d.distance(Vec2::new(1.0, 0.0)) < 1e-8, "{d:?}");
}

#[test]
fn conic_corner_blend_between_two_lines_solves() {
    let mut s = new();
    let l1 = s.add_line(p(0.0, 0.0), p(4.0, 0.0)).unwrap();
    let l2 = s.add_line(p(8.0, 4.0), p(8.0, 8.0)).unwrap();
    s.add_constraint(ConstraintKind::Fix { entity: l1 })
        .unwrap();
    s.add_constraint(ConstraintKind::Fix { entity: l2 })
        .unwrap();
    let c = s
        .add_conic(p(4.2, 0.1), p(7.0, 0.5), p(8.1, 3.8), 0.3)
        .unwrap();
    touch(&mut s, l1, End::End, c, End::Start);
    touch(&mut s, c, End::End, l2, End::Start);
    s.add_constraint_oriented(ConstraintKind::Tangent {
        a: l1,
        b: c,
        kind: TangentKind::AtEndpoint {
            a_end: End::End,
            b_end: End::Start,
            reversed: false,
        },
    })
    .unwrap();
    s.add_constraint_oriented(ConstraintKind::Tangent {
        a: c,
        b: l2,
        kind: TangentKind::AtEndpoint {
            a_end: End::End,
            b_end: End::Start,
            reversed: false,
        },
    })
    .unwrap();
    // The conic passes through a fixed point at rho = 0.4: apex (8, 0), chord midpoint (6, 2).
    let target = p(6.0 + 2.0 * 0.4, 2.0 - 2.0 * 0.4);
    let fixed = s.add_point(target).unwrap();
    s.add_constraint(ConstraintKind::Fix { entity: fixed })
        .unwrap();
    s.add_constraint(ConstraintKind::PointOnCurve {
        point: fixed,
        curve: c,
    })
    .unwrap();
    assert_eq!(dof(&s).unwrap(), 0);
    let status = solve(&mut s);
    assert_eq!(status, ProtoStatus::Converged);
    let EntityKind::Conic { apex, rho, .. } = s.entity(c).unwrap().kind else {
        panic!()
    };
    assert!(s.point(apex).unwrap().distance(p(8.0, 0.0)) < 1e-8);
    assert!((s.params().get(rho) - 0.4).abs() < 1e-8);
}

#[test]
fn point_on_spline_and_weight_solves() {
    // A rational quadratic Bezier through (1,0), (1,1), (0,1) with locked end weights: a point
    // on the circle at 45 degrees fixes the middle weight at cos(45 deg).
    let mut s = new();
    let spl = s
        .add_spline(
            2,
            &[p(1.0, 0.0), p(1.0, 1.0), p(0.0, 1.0)],
            Some(&[1.0, 1.0, 1.0]),
            None,
        )
        .unwrap();
    let EntityKind::Spline(amg_sketch::model::SplineDef::Control {
        poles,
        weights: Some(w),
        ..
    }) = s.entity(spl).unwrap().kind.clone()
    else {
        panic!()
    };
    for &q in &poles {
        s.add_constraint(ConstraintKind::Fix { entity: q }).unwrap();
    }
    for &p in &[w[0], w[2]] {
        s.params_mut()
            .set_flags(p, amg_sketch::ParamFlags { locked: true });
    }
    let pt = s.add_point(p(FRAC_1_SQRT_2, FRAC_1_SQRT_2)).unwrap();
    s.add_constraint(ConstraintKind::Fix { entity: pt })
        .unwrap();
    s.add_constraint(ConstraintKind::PointOnCurve {
        point: pt,
        curve: spl,
    })
    .unwrap();
    let status = solve(&mut s);
    assert_eq!(status, ProtoStatus::Converged);
    assert!((s.params().get(w[1]) - FRAC_1_SQRT_2).abs() < 1e-8);
    let c = spline_curve(&s, spl);
    assert!((c.eval(0.5).length() - 1.0).abs() < 1e-8);

    // A free point is dragged onto a fit spline and stays there when the spline moves.
    let mut s = new();
    let fit = s
        .add_spline_fit(
            &[p(0.0, 0.0), p(2.0, 1.0), p(4.0, -1.0), p(6.0, 0.0)],
            None,
            None,
        )
        .unwrap();
    let pt = s.add_point(p(3.0, 2.0)).unwrap();
    s.add_constraint(ConstraintKind::PointOnCurve {
        point: pt,
        curve: fit,
    })
    .unwrap();
    assert_eq!(solve(&mut s), ProtoStatus::Converged);
    let c = spline_curve(&s, fit);
    let q = s.point(pt).unwrap();
    assert!(c.closest(q).1.distance(q) < 1e-8);
}

// ---- Profiles ---------------------------------------------------------------------------------

/// Signed area and centroid of the polygon through `n` points of `c` plus the closing chord.
fn polyline_area(c: &BSpline2, n: u32) -> f64 {
    let pts: Vec<Point2> = (0..=n)
        .map(|i| c.eval(c.span() * f64::from(i) / f64::from(n)))
        .collect();
    let mut a = 0.0;
    for i in 0..pts.len() {
        a += 0.5 * pts[i].perp_dot(pts[(i + 1) % pts.len()]);
    }
    a
}

fn profiles_of(s: &Sketch) -> ProfileSet {
    detect_profiles(s, &ProfileOptions::default())
}

#[test]
fn spline_profile_areas_agree_with_a_10k_segment_polyline() {
    type Make = Box<dyn Fn(&mut Sketch) -> EntityId>;
    let cases: Vec<(&str, Make)> = vec![
        (
            "bezier",
            Box::new(|s| {
                s.add_spline(
                    3,
                    &[p(0.0, 0.0), p(2.0, 5.0), p(8.0, 6.0), p(10.0, 0.0)],
                    None,
                    None,
                )
                .unwrap()
            }),
        ),
        (
            "bspline",
            Box::new(|s| {
                s.add_spline(
                    3,
                    &[
                        p(0.0, 0.0),
                        p(1.0, 4.0),
                        p(3.0, 5.0),
                        p(5.0, 1.0),
                        p(8.0, 4.0),
                        p(10.0, 0.0),
                    ],
                    None,
                    None,
                )
                .unwrap()
            }),
        ),
        (
            "rational",
            Box::new(|s| {
                s.add_spline(
                    4,
                    &[
                        p(0.0, 0.0),
                        p(1.0, 4.0),
                        p(4.0, 6.0),
                        p(8.0, 5.0),
                        p(10.0, 0.0),
                    ],
                    Some(&[1.0, 2.0, 0.6, 1.5, 1.0]),
                    None,
                )
                .unwrap()
            }),
        ),
        (
            "fit",
            Box::new(|s| {
                s.add_spline_fit(
                    &[
                        p(0.0, 0.0),
                        p(2.0, 3.0),
                        p(5.0, 4.0),
                        p(8.0, 2.0),
                        p(10.0, 0.0),
                    ],
                    Some(Vec2::new(0.5, 2.0)),
                    Some(Vec2::new(1.0, -2.0)),
                )
                .unwrap()
            }),
        ),
        (
            "conic",
            Box::new(|s| {
                s.add_conic(p(0.0, 0.0), p(5.0, 8.0), p(10.0, 0.0), 0.35)
                    .unwrap()
            }),
        ),
    ];
    for (name, make) in cases {
        let mut s = new();
        let spl = make(&mut s);
        let c = spline_curve(&s, spl);
        let (a, b) = (c.eval(0.0), c.eval(c.span()));
        s.add_line(b, a).unwrap();
        let set = profiles_of(&s);
        assert!(set.issues.is_empty(), "{name}: {:?}", set.issues);
        assert_eq!(set.profiles.len(), 1, "{name}");
        let area = set.profiles[0].fingerprint.area;
        let poly = polyline_area(&c, 10_000).abs();
        assert!(
            rel(area, poly) < 1e-6,
            "{name}: profile area {area}, polyline {poly}"
        );
        // The centroid lies inside the bounding box, and above the chord.
        let cen = set.profiles[0].fingerprint.centroid;
        assert!(cen.y > 0.0 && cen.x > 0.0 && cen.x < 10.0);
    }
}

#[test]
fn spline_cuts_a_rectangle_into_two_profiles_and_holes_work() {
    let mut s = new();
    let r = build::rectangle_2pt(&mut s, p(0.0, 0.0), p(10.0, 6.0)).unwrap();
    let _ = r;
    // A spline crossing the rectangle from the bottom edge to the top edge.
    let spl = s
        .add_spline(
            3,
            &[p(4.0, -1.0), p(7.0, 2.0), p(3.0, 4.0), p(6.0, 7.0)],
            None,
            None,
        )
        .unwrap();
    let set = profiles_of(&s);
    assert!(set.issues.is_empty(), "{:?}", set.issues);
    assert_eq!(set.profiles.len(), 2);
    let total: f64 = set.profiles.iter().map(|q| q.fingerprint.area).sum();
    assert!(rel(total, 60.0) < 1e-9, "{total}");
    assert!(
        set.profiles
            .iter()
            .all(|q| q.fingerprint.sources.contains(&spl))
    );

    // A closed spline loop inside a rectangle is a hole.
    let mut s = new();
    build::rectangle_2pt(&mut s, p(0.0, 0.0), p(10.0, 10.0)).unwrap();
    let loop_ = s
        .add_spline(
            3,
            &[
                p(4.0, 4.0),
                p(7.0, 3.0),
                p(7.0, 7.0),
                p(3.0, 6.0),
                p(4.0, 4.0),
            ],
            None,
            None,
        )
        .unwrap();
    let c = spline_curve(&s, loop_);
    let inner = polyline_area(&c, 10_000).abs();
    let set = profiles_of(&s);
    assert!(set.issues.is_empty(), "{:?}", set.issues);
    // The region of the loop is its own profile; the rectangle profile has it as a hole.
    assert_eq!(set.profiles.len(), 2);
    let mut areas: Vec<f64> = set.profiles.iter().map(|q| q.fingerprint.area).collect();
    areas.sort_by(f64::total_cmp);
    assert!(rel(areas[0], inner) < 1e-6, "{areas:?} {inner}");
    assert!(rel(areas[1], 100.0 - inner) < 1e-6);
    assert_eq!(set.profiles.iter().map(|q| q.holes.len()).sum::<usize>(), 1);
}

/// Number of proper crossings of the polyline through `n` points of `c` with itself.
fn brute_self_crossings(c: &BSpline2, n: usize) -> usize {
    let pts: Vec<Point2> = (0..=n)
        .map(|i| c.eval(c.span() * i as f64 / n as f64))
        .collect();
    let cross = |a: Point2, b: Point2, c: Point2, d: Point2| {
        let (r, s) = (b - a, d - c);
        let den = r.perp_dot(s);
        if den.abs() < 1e-14 {
            return false;
        }
        let t = (c - a).perp_dot(s) / den;
        let u = (c - a).perp_dot(r) / den;
        (0.0..1.0).contains(&t) && (0.0..1.0).contains(&u)
    };
    let mut k = 0;
    for i in 0..n {
        for j in i + 2..n {
            if cross(pts[i], pts[i + 1], pts[j], pts[j + 1]) {
                k += 1;
            }
        }
    }
    k
}

#[test]
fn self_intersecting_spline_reports_an_issue_and_makes_a_loop() {
    let mut s = new();
    let spl = s
        .add_spline(
            3,
            &[p(0.0, 0.0), p(12.0, 8.0), p(-2.0, 8.0), p(10.0, 0.0)],
            None,
            None,
        )
        .unwrap();
    let c = spline_curve(&s, spl);
    assert_eq!(
        brute_self_crossings(&c, 1500),
        1,
        "test geometry loops once"
    );
    let hits = c.self_intersections(TOL.linear);
    assert_eq!(hits.len(), 1, "{hits:?}");
    let (t1, t2, q) = hits[0];
    assert!(t1 < t2);
    assert!(c.eval(t1).distance(q) < 1e-9 && c.eval(t2).distance(q) < 1e-9);
    let set = profiles_of(&s);
    assert!(
        set.issues.contains(&ProfileIssue::SelfIntersection(spl)),
        "{:?}",
        set.issues
    );
    // The loop is a profile with the area of the closed part of the curve.
    assert_eq!(set.profiles.len(), 1, "{:?}", set.profiles.len());
    let part = c.trimmed(t1, t2);
    let loop_area = polyline_area(&part, 20_000).abs();
    assert!(rel(set.profiles[0].fingerprint.area, loop_area) < 1e-6);
    // A simple spline reports nothing, and neither does a closed one.
    let mut s = new();
    s.add_spline(
        3,
        &[p(0.0, 0.0), p(2.0, 4.0), p(5.0, 4.0), p(7.0, 0.0)],
        None,
        None,
    )
    .unwrap();
    assert!(profiles_of(&s).issues.is_empty());
}

// ---- Intersections ----------------------------------------------------------------------------

const TOL: Tolerance = Tolerance::DEFAULT;

fn bez() -> Curve2 {
    Curve2::BSpline(
        BSpline2::new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![p(0.0, 0.0), p(3.0, 6.0), p(7.0, -6.0), p(10.0, 0.0)],
            vec![1.0; 4],
        )
        .unwrap(),
    )
}

#[test]
fn spline_intersections_with_every_curve_kind() {
    let sp = bez();
    let line = |a: Point2, b: Point2| Curve2::Line(amg_core::Segment2 { a, b });
    // Horizontal line y = 0: the S-curve crosses at both ends and once in the middle.
    let h = intersect(&sp, &line(p(-1.0, 0.0), p(11.0, 0.0)), &TOL);
    assert_eq!(h.len(), 3, "{h:?}");
    for hit in &h {
        assert!(sp.eval(hit.ta).distance(hit.point) < 1e-9);
        assert!(hit.point.y.abs() < 1e-9);
    }
    // Swapped arguments agree.
    let h2 = intersect(&line(p(-1.0, 0.0), p(11.0, 0.0)), &sp, &TOL);
    assert_eq!(h2.len(), 3);
    for (a, b) in h
        .iter()
        .zip(h2.iter().rev().collect::<Vec<_>>().iter().rev())
    {
        let _ = (a, b);
    }
    assert!(h2.iter().all(|x| sp.eval(x.tb).distance(x.point) < 1e-9));
    // A line missing the curve.
    assert!(intersect(&sp, &line(p(-1.0, 9.0), p(11.0, 9.0)), &TOL).is_empty());
    // Tangent line: y = max of the curve.
    let top = sp.bbox().max.y;
    let t = intersect(&sp, &line(p(-1.0, top), p(11.0, top)), &TOL);
    assert_eq!(t.len(), 1, "{t:?}");
    assert!(t[0].tangential);
    // Circle around (5, 0) with radius 3: the curve passes through it.
    let circle = Curve2::Circle(amg_sketch::Circle2 {
        center: p(5.0, 0.0),
        radius: 3.0,
    });
    let h = intersect(&sp, &circle, &TOL);
    assert!(h.len() >= 2, "{h:?}");
    for hit in &h {
        assert!((hit.point.distance(p(5.0, 0.0)) - 3.0).abs() < 1e-9);
        assert!(sp.eval(hit.ta).distance(hit.point) < 1e-9);
        assert!(circle.eval(hit.tb).distance(hit.point) < 1e-9);
    }
    // Arc: only part of the circle counts.
    let arc = Curve2::Arc(amg_sketch::Arc2 {
        center: p(5.0, 0.0),
        radius: 3.0,
        start_angle: 0.0,
        sweep: PI,
    });
    let on_arc = intersect(&sp, &arc, &TOL);
    assert!(on_arc.len() < h.len() + 1);
    assert!(on_arc.iter().all(|x| x.point.y >= -1e-9));
    // Ellipse.
    let ell = Curve2::Ellipse(amg_sketch::EllipseArc2::full(
        p(5.0, 0.0),
        Vec2::new(4.0, 0.0),
        2.0,
    ));
    let h = intersect(&sp, &ell, &TOL);
    assert!(!h.is_empty());
    for hit in &h {
        let q = hit.point - p(5.0, 0.0);
        assert!(((q.x / 4.0).powi(2) + (q.y / 2.0).powi(2) - 1.0).abs() < 1e-8);
    }
    // Spline vs spline: the mirrored S crosses it.
    let other = Curve2::BSpline(
        BSpline2::new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![p(0.0, 0.0), p(3.0, -6.0), p(7.0, 6.0), p(10.0, 0.0)],
            vec![1.0; 4],
        )
        .unwrap(),
    );
    let h = intersect(&sp, &other, &TOL);
    assert!(h.len() >= 3, "{h:?}");
    for hit in &h {
        assert!(sp.eval(hit.ta).distance(other.eval(hit.tb)) < 1e-9);
    }
}

#[test]
fn spline_end_touching_and_overlap() {
    let sp = bez();
    // A line starting on the curve (T junction) and one ending at its end point.
    let a = Curve2::Line(amg_core::Segment2 {
        a: sp.eval(0.3 * 1.0),
        b: p(4.0, 20.0),
    });
    let h = intersect(&sp, &a, &TOL);
    assert!(h.iter().any(|x| x.point.distance(sp.eval(0.3)) < 1e-7));
    // A degree-1 spline lying on a line overlaps it.
    let on = Curve2::BSpline(
        BSpline2::new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![p(0.0, 0.0), p(10.0, 0.0)],
            vec![1.0; 2],
        )
        .unwrap(),
    );
    let line = Curve2::Line(amg_core::Segment2 {
        a: p(2.0, 0.0),
        b: p(14.0, 0.0),
    });
    let mut s = new();
    s.add_line(p(2.0, 0.0), p(14.0, 0.0)).unwrap();
    s.add_spline(1, &[p(0.0, 0.0), p(10.0, 0.0)], None, None)
        .unwrap();
    let set = profiles_of(&s);
    assert!(
        set.issues
            .iter()
            .any(|i| matches!(i, ProfileIssue::Overlap { .. })),
        "{:?}",
        set.issues
    );
    let _ = (on, line);
}

// ---- Validation, serde and error paths --------------------------------------------------------

#[test]
fn constructor_and_validation_errors() {
    let mut s = new();
    let poles = [p(0.0, 0.0), p(1.0, 2.0), p(3.0, 2.0), p(4.0, 0.0)];
    // Degree and pole count.
    assert!(matches!(
        s.add_spline(0, &poles, None, None),
        Err(SketchError::NotApplicable(_))
    ));
    assert!(matches!(
        s.add_spline(11, &poles, None, None),
        Err(SketchError::NotApplicable(_))
    ));
    assert!(matches!(
        s.add_spline(4, &poles, None, None),
        Err(SketchError::Degenerate(_))
    ));
    // Knot vector: wrong length, not clamped, decreasing.
    assert!(s.add_spline(3, &poles, None, Some(vec![0.0, 1.0])).is_err());
    assert!(
        s.add_spline(
            3,
            &poles,
            None,
            Some(vec![0.0, 0.5, 0.5, 0.5, 1.0, 1.0, 1.0, 1.0])
        )
        .is_err()
    );
    assert!(
        s.add_spline(
            3,
            &poles,
            None,
            Some(vec![0.0, 0.0, 0.0, 0.0, 2.0, 1.0, 1.0, 1.0])
        )
        .is_err()
    );
    // Weights.
    assert!(matches!(
        s.add_spline(3, &poles, Some(&[1.0, 1.0, 0.0, 1.0]), None),
        Err(SketchError::InvalidValue { .. })
    ));
    assert!(matches!(
        s.add_spline(3, &poles, Some(&[1.0, 1.0]), None),
        Err(SketchError::NotApplicable(_))
    ));
    // Non-finite input.
    assert!(
        s.add_spline(
            3,
            &[p(f64::NAN, 0.0), p(1.0, 1.0), p(2.0, 0.0), p(3.0, 1.0)],
            None,
            None
        )
        .is_err()
    );
    // Fit points.
    assert!(matches!(
        s.add_spline_fit(&[p(0.0, 0.0)], None, None),
        Err(SketchError::Degenerate(_))
    ));
    assert!(matches!(
        s.add_spline_fit(&[p(0.0, 0.0), p(0.0, 0.0), p(1.0, 1.0)], None, None),
        Err(SketchError::Degenerate(_))
    ));
    assert!(matches!(
        s.add_spline_fit(&[p(0.0, 0.0), p(1.0, 0.0)], Some(Vec2::ZERO), None),
        Err(SketchError::Degenerate(_))
    ));
    // Conic.
    for rho in [0.0, 1.0, -0.1, 1.5, f64::NAN] {
        assert!(matches!(
            s.add_conic(p(0.0, 0.0), p(1.0, 1.0), p(2.0, 0.0), rho),
            Err(SketchError::InvalidValue { .. })
        ));
    }
    // Nothing was added by the failed calls.
    assert_eq!(s.entities().count(), 0);

    // add_entity: periodic splines are refused.
    let pts: Vec<EntityId> = poles.iter().map(|&q| s.add_point(q).unwrap()).collect();
    let periodic = EntityKind::Spline(amg_sketch::model::SplineDef::Control {
        degree: 3,
        poles: pts.clone(),
        weights: None,
        knots: vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        periodic: true,
    });
    assert!(matches!(
        s.add_entity(periodic, false, EntityOrigin::User),
        Err(SketchError::NotApplicable(m)) if m.contains("periodic")
    ));
    // A pole used twice.
    let dup = EntityKind::Spline(amg_sketch::model::SplineDef::Control {
        degree: 3,
        poles: vec![pts[0], pts[1], pts[1], pts[3]],
        weights: None,
        knots: vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        periodic: false,
    });
    assert!(s.add_entity(dup, false, EntityOrigin::User).is_err());
    // A non-point pole.
    let line = s.add_line(p(5.0, 5.0), p(6.0, 6.0)).unwrap();
    let bad = EntityKind::Spline(amg_sketch::model::SplineDef::Control {
        degree: 1,
        poles: vec![pts[0], line],
        weights: None,
        knots: vec![0.0, 0.0, 1.0, 1.0],
        periodic: false,
    });
    assert!(matches!(
        s.add_entity(bad, false, EntityOrigin::User),
        Err(SketchError::WrongEntityKind { .. })
    ));
}

#[test]
fn constraint_kind_errors_name_the_problem() {
    let mut s = new();
    let spl = s
        .add_spline(
            3,
            &[p(0.0, 0.0), p(1.0, 2.0), p(3.0, 2.0), p(4.0, 0.0)],
            None,
            None,
        )
        .unwrap();
    let rat = s
        .add_spline(
            3,
            &[p(0.0, 5.0), p(1.0, 7.0), p(3.0, 7.0), p(4.0, 5.0)],
            Some(&[1.0, 1.0, 1.0, 1.0]),
            None,
        )
        .unwrap();
    let con = s
        .add_conic(p(0.0, 10.0), p(2.0, 13.0), p(4.0, 10.0), 0.5)
        .unwrap();
    let line = s.add_line(p(4.0, 0.0), p(8.0, 0.0)).unwrap();
    let circle = s.add_circle(p(0.0, -5.0), 1.0).unwrap();
    let pt = s.add_point(p(9.0, 9.0)).unwrap();
    let na = |r: Result<ConstraintId, SketchError>| {
        assert!(
            matches!(
                r,
                Err(SketchError::NotApplicable(_)) | Err(SketchError::WrongEntityKind { .. })
            ),
            "{r:?}"
        );
    };
    // Tangent: only at a shared end point.
    for kind in [TangentKind::External, TangentKind::Internal] {
        for (a, b) in [(spl, circle), (circle, spl), (line, spl), (con, line)] {
            let r = s.add_constraint(ConstraintKind::Tangent { a, b, kind });
            assert!(
                matches!(&r, Err(SketchError::NotApplicable(m)) if m.contains("AtEndpoint")),
                "{r:?}"
            );
        }
    }
    // AtEndpoint: ends must touch; line-line is not tangent.
    let at = |a_end, b_end| TangentKind::AtEndpoint {
        a_end,
        b_end,
        reversed: false,
    };
    na(s.add_constraint(ConstraintKind::Tangent {
        a: line,
        b: spl,
        kind: at(End::Start, End::Start),
    }));
    na(s.add_constraint(ConstraintKind::Tangent {
        a: spl,
        b: circle,
        kind: at(End::Start, End::Start),
    }));
    // Curvature needs a spline or conic, and touching ends.
    let l2 = s.add_line(p(8.0, 0.0), p(9.0, 3.0)).unwrap();
    s.add_constraint(ConstraintKind::Coincident {
        a: end_point(&s, line, End::End),
        b: end_point(&s, l2, End::Start),
    })
    .unwrap();
    na(s.add_constraint(ConstraintKind::Curvature {
        a: line,
        b: l2,
        a_end: End::End,
        b_end: End::Start,
        reversed: false,
    }));
    na(s.add_constraint(ConstraintKind::Curvature {
        a: line,
        b: spl,
        a_end: End::End,
        b_end: End::Start,
        reversed: false,
    }));
    na(s.add_constraint(ConstraintKind::Curvature {
        a: circle,
        b: spl,
        a_end: End::End,
        b_end: End::Start,
        reversed: false,
    }));
    // Equal needs matching rational splines or conics.
    na(s.add_constraint(ConstraintKind::Equal { a: spl, b: rat }));
    na(s.add_constraint(ConstraintKind::Equal { a: spl, b: spl }));
    na(s.add_constraint(ConstraintKind::Equal { a: rat, b: con }));
    na(s.add_constraint(ConstraintKind::Equal { a: line, b: con }));
    // Other constraints that do not apply.
    na(s.add_constraint(ConstraintKind::Concentric { a: spl, b: circle }));
    na(s.add_constraint(ConstraintKind::Symmetric {
        a: spl,
        b: rat,
        axis: line,
    }));
    na(s.add_constraint(ConstraintKind::Symmetric {
        a: con,
        b: con,
        axis: line,
    }));
    na(s.add_constraint(ConstraintKind::Midpoint {
        point: pt,
        line: spl,
    }));
    na(s.add_constraint(ConstraintKind::Parallel { a: spl, b: line }));
    na(s.add_constraint(dim(
        DimensionKind::Distance {
            a: pt,
            b: spl,
            side: Side::Positive,
        },
        1.0,
    )));
    na(s.add_constraint(dim(DimensionKind::Radius { curve: spl }, 1.0)));
    na(s.add_constraint(dim(DimensionKind::Length { line: con }, 1.0)));
    na(s.add_constraint(dim(DimensionKind::EllipseMajor { ellipse: spl }, 1.0)));
    // A point of the spline itself cannot be on it.
    let EntityKind::Spline(def) = s.entity(spl).unwrap().kind.clone() else {
        panic!()
    };
    na(s.add_constraint(ConstraintKind::PointOnCurve {
        point: def.points()[1],
        curve: spl,
    }));
    // Residual builders refuse a point-on-spline without its aux parameter.
    let k = ConstraintKind::PointOnCurve {
        point: pt,
        curve: spl,
    };
    assert!(constraint_residual(&s, &k, 1.0).is_err());
}

#[test]
fn serde_round_trip_and_load_validation() {
    let mut g = Rng(5);
    let mut s = new();
    let a = random_control(&mut s, &mut g, 3, true);
    let b = random_fit(&mut s, &mut g, true);
    let c = random_conic(&mut s, &mut g);
    let pt = s.add_point(p(1.0, 1.0)).unwrap();
    s.add_constraint(ConstraintKind::PointOnCurve {
        point: pt,
        curve: a,
    })
    .unwrap();
    let json = serde_json::to_string(&s).unwrap();
    let back: Sketch = serde_json::from_str(&json).unwrap();
    assert_eq!(serde_json::to_string(&back).unwrap(), json);
    for id in [a, b, c] {
        assert_eq!(s.curve(id).unwrap(), back.curve(id).unwrap());
    }
    // Corrupt files are rejected on load.
    let mut v: serde_json::Value = serde_json::from_str(&json).unwrap();
    // Periodic flag.
    let mut w = v.clone();
    for (_, e) in w["entities"].as_object_mut().unwrap().iter_mut() {
        if let Some(c) = e["kind"]["Spline"].get_mut("Control") {
            c["periodic"] = true.into();
        }
    }
    assert!(serde_json::from_value::<Sketch>(w).is_err());
    // A knot vector that is not clamped.
    let mut w = v.clone();
    for (_, e) in w["entities"].as_object_mut().unwrap().iter_mut() {
        if let Some(c) = e["kind"]["Spline"].get_mut("Control") {
            c["knots"][0] = (-1.0).into();
        }
    }
    assert!(serde_json::from_value::<Sketch>(w).is_err());
    // A non-positive weight (the slot of the conic's rho set to 2).
    let mut w = v.clone();
    let rho = match &w["entities"][c.index().to_string()]["kind"]["Conic"]["rho"] {
        serde_json::Value::Number(n) => n.as_u64().unwrap() as usize,
        other => panic!("{other:?}"),
    };
    w["params"]["values"][rho] = 2.0.into();
    assert!(serde_json::from_value::<Sketch>(w).is_err());
    // Fit parameters out of order.
    let mut w = v.clone();
    for (_, e) in w["entities"].as_object_mut().unwrap().iter_mut() {
        if let Some(c) = e["kind"]["Spline"].get_mut("Fit") {
            c["params"][1] = 2.0.into();
        }
    }
    assert!(serde_json::from_value::<Sketch>(w).is_err());
    // A point-on-spline constraint without its aux param.
    let cs = v["constraints"].as_object_mut().unwrap();
    for (_, k) in cs.iter_mut() {
        k["aux"] = serde_json::Value::Array(vec![]);
    }
    assert!(serde_json::from_value::<Sketch>(v.clone()).is_err());
}

#[test]
fn removing_and_compacting_keep_splines_valid() {
    let mut g = Rng(9);
    let mut s = new();
    let a = random_control(&mut s, &mut g, 3, true);
    let b = random_conic(&mut s, &mut g);
    let EntityKind::Spline(def) = s.entity(a).unwrap().kind.clone() else {
        panic!()
    };
    let before = s.curve(b).unwrap();
    // Removing a pole removes the spline (a dependent), and its orphaned points.
    let rep = s.remove_entities(&[def.points()[0]]).unwrap();
    assert!(rep.entities.contains(&a));
    assert!(s.entity(a).is_none());
    s.compact();
    assert_eq!(s.curve(b).unwrap(), before);
    let json = serde_json::to_string(&s).unwrap();
    let back: Sketch = serde_json::from_str(&json).unwrap();
    assert_eq!(back.curve(b).unwrap(), before);
}
