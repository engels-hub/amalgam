//! SK-22 acceptance: ellipse and elliptical-arc entities, their equations, point-on / tangent /
//! axis dimensions, `Curve2::Ellipse` in profile detection, and the ellipse paths of the
//! operations that match on entity or curve kinds.
//!
//! Solves use the SK-03 production solver (`solve::NativeSolver`); ranks and DOF of
//! unsolved sketches the SK-02 helper.

#![allow(
    clippy::unwrap_used,
    clippy::float_cmp,
    reason = "test helpers; a failure is a test failure"
)]

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use amg_core::{Plane, Point2, Transform2, Vec2};
use amg_sketch::corpus::reference::RefSystem;
use amg_sketch::solve::{
    DofAnalysis, NativeSolver, SolveOptions, SolveReport, SolveStatus, SolverBackend,
};
use amg_sketch::solve::{
    Residual, build_system, constraint_residual, constraint_residual_aux, dof, entity_residual,
};
use amg_sketch::transform::{
    mirror, pattern_circ, pattern_rect, rotation_about, scale_about, transform,
};
use amg_sketch::{
    ConstraintId, ConstraintKind, Curve2, Dimension, DimensionKind, EntityId, EntityKind, HvTarget,
    ProfileOptions, ProfileSet, Sketch, SketchError, TangentKind, build, detect_profiles, edit,
    offset,
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

fn residual_max(s: &Sketch) -> f64 {
    let sys = build_system(s).unwrap();
    sys.system.max_abs_residual(s.params().as_slice())
}

fn solve(s: &mut Sketch) -> SolveStatus {
    native_solve(s).status
}

fn ellipse_curve(s: &Sketch, id: EntityId) -> amg_sketch::EllipseArc2 {
    match s.curve(id).unwrap() {
        Curve2::Ellipse(e) => e,
        other => panic!("not an ellipse: {other:?}"),
    }
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1e-300)
}

// ---- Deterministic random geometry ------------------------------------------------------------

struct Rng(u64);

impl Rng {
    /// Uniform in `[lo, hi)`.
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        let u = (self.0 >> 11) as f64 / (1u64 << 53) as f64;
        lo + (hi - lo) * u
    }
}

fn random_ellipse(s: &mut Sketch, g: &mut Rng) -> EntityId {
    let c = p(g.range(-5.0, 5.0), g.range(-5.0, 5.0));
    let ang = g.range(0.0, TAU);
    let a = g.range(1.5, 6.0);
    let m = c + Vec2::new(ang.cos(), ang.sin()) * a;
    s.add_ellipse(c, m, g.range(0.5, 6.0)).unwrap()
}

fn random_arc(s: &mut Sketch, g: &mut Rng) -> EntityId {
    let c = p(g.range(-5.0, 5.0), g.range(-5.0, 5.0));
    let ang = g.range(0.0, TAU);
    let a = g.range(1.5, 6.0);
    let m = c + Vec2::new(ang.cos(), ang.sin()) * a;
    let t0 = g.range(-4.0, 4.0);
    let t1 = t0 + g.range(0.5, 5.5);
    s.add_elliptic_arc(c, m, g.range(0.5, 6.0), t0, t1).unwrap()
}

fn random_line(s: &mut Sketch, g: &mut Rng) -> EntityId {
    let a = p(g.range(-8.0, 8.0), g.range(-8.0, 8.0));
    let ang = g.range(0.0, TAU);
    s.add_line(a, a + Vec2::new(ang.cos(), ang.sin()) * g.range(2.0, 9.0))
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
    let h = 1e-3;
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
            if err > 1e-9 && err > 1e-6 * a.abs().max(fd.abs()) {
                return Err(format!(
                    "row {i}, param {pid:?}: analytic {a:e}, finite difference {fd:e}"
                ));
            }
        }
    }
    Ok(())
}

fn check_constraint(s: &Sketch, id: ConstraintId) {
    let c = s.constraint(id).unwrap();
    let r = constraint_residual_aux(s, &c.kind, &c.aux, 10.0)
        .unwrap()
        .unwrap();
    assert!(r.rows() > 0);
    fd_check(&r, s.params().as_slice()).unwrap_or_else(|e| panic!("{:?}: {e}", c.kind));
}

#[test]
fn jacobians_match_finite_differences_for_every_ellipse_row() {
    let mut g = Rng(7);
    let mut checked = 0;
    for _ in 0..120 {
        let mut s = new();
        let e1 = random_ellipse(&mut s, &mut g);
        let e2 = random_ellipse(&mut s, &mut g);
        let a1 = random_arc(&mut s, &mut g);
        let a2 = random_arc(&mut s, &mut g);
        let l = random_line(&mut s, &mut g);
        let circ = s
            .add_circle(p(g.range(-3.0, 3.0), g.range(-3.0, 3.0)), 2.0)
            .unwrap();
        let axis = random_line(&mut s, &mut g);
        let pt = s
            .add_point(p(g.range(-6.0, 6.0), g.range(-6.0, 6.0)))
            .unwrap();

        // Internal equations of an elliptical arc (4 rows).
        let r = entity_residual(&s, a1).unwrap();
        assert_eq!(r.rows(), 4);
        fd_check(&r, s.params().as_slice()).unwrap();
        assert!(entity_residual(&s, e1).is_none());

        let mut kinds = vec![
            ConstraintKind::PointOnCurve {
                point: pt,
                curve: e1,
            },
            ConstraintKind::PointOnCurve {
                point: pt,
                curve: a1,
            },
            ConstraintKind::Concentric { a: e1, b: e2 },
            ConstraintKind::Concentric { a: e1, b: circ },
            ConstraintKind::Concentric { a: a1, b: e2 },
            ConstraintKind::Equal { a: e1, b: e2 },
            ConstraintKind::Equal { a: a1, b: a2 },
            ConstraintKind::Equal { a: e1, b: a2 },
            ConstraintKind::Symmetric { a: e1, b: e2, axis },
            dim(DimensionKind::EllipseMajor { ellipse: e1 }, 3.0),
            dim(DimensionKind::EllipseMinor { ellipse: e1 }, 2.0),
            dim(DimensionKind::EllipseMajor { ellipse: a1 }, 3.0),
            dim(DimensionKind::EllipseMinor { ellipse: a2 }, 2.0),
        ];
        for ext in [TangentKind::External, TangentKind::Internal] {
            for (a, b) in [(l, e1), (e1, l), (l, a1), (a2, l)] {
                kinds.push(ConstraintKind::Tangent { a, b, kind: ext });
            }
        }
        for k in kinds {
            let id = s.add_constraint(k).unwrap();
            check_constraint(&s, id);
            checked += 1;
        }
    }
    assert!(checked > 2000);
}

// ---- Equations hold on consistent geometry ----------------------------------------------------

#[test]
fn residuals_vanish_on_consistent_geometry() {
    let mut g = Rng(11);
    for _ in 0..60 {
        let mut s = new();
        let e = random_ellipse(&mut s, &mut g);
        let arc = random_arc(&mut s, &mut g);
        assert!(
            residual_max(&s) < 1e-12,
            "elliptical arc internal equations"
        );

        // A point on the ellipse at a random angle: the aux parameter starts at the foot.
        let ell = ellipse_curve(&s, e);
        let th = g.range(0.0, TAU);
        let pt = s.add_point(ell.point_at(th)).unwrap();
        let on = s
            .add_constraint(ConstraintKind::PointOnCurve {
                point: pt,
                curve: e,
            })
            .unwrap();
        let aux = s.constraint(on).unwrap().aux.clone();
        assert_eq!(aux.len(), 1);
        assert!(
            residual_max(&s) < 1e-9,
            "point on ellipse: {}",
            residual_max(&s)
        );

        // A tangent line through a point of the ellipse, either orientation of the line.
        for flip in [false, true] {
            let q = ell.point_at(th);
            let d = ell.tangent_at(th).normalize() * 3.0;
            let (a, b) = if flip { (q + d, q - d) } else { (q - d, q + d) };
            let line = s.add_line(a, b).unwrap();
            let c = s
                .add_constraint_oriented(ConstraintKind::Tangent {
                    a: line,
                    b: e,
                    kind: TangentKind::External,
                })
                .unwrap();
            let r = constraint_residual(&s, &s.constraint(c).unwrap().kind, 10.0)
                .unwrap()
                .unwrap();
            let mut out = vec![0.0; r.rows()];
            r.eval(s.params().as_slice(), &mut out);
            assert!(out[0].abs() < 1e-9, "tangent residual {}", out[0]);
            // The same through the arc entity's underlying ellipse.
            let ea = ellipse_curve(&s, arc);
            let qa = ea.point_at(th);
            let da = ea.tangent_at(th).normalize() * 2.0;
            let line_a = s.add_line(qa - da, qa + da).unwrap();
            let ca = s
                .add_constraint_oriented(ConstraintKind::Tangent {
                    b: line_a,
                    a: arc,
                    kind: TangentKind::External,
                })
                .unwrap();
            let r = constraint_residual(&s, &s.constraint(ca).unwrap().kind, 10.0)
                .unwrap()
                .unwrap();
            r.eval(s.params().as_slice(), &mut out);
            assert!(out[0].abs() < 1e-9);
        }

        // Axis dimensions measure the axes.
        let a = ell.semi_major();
        let maj = s
            .add_constraint(dim(DimensionKind::EllipseMajor { ellipse: e }, a))
            .unwrap();
        let min = s
            .add_constraint(dim(
                DimensionKind::EllipseMinor { ellipse: e },
                ell.minor_radius,
            ))
            .unwrap();
        for c in [maj, min] {
            let r = constraint_residual(&s, &s.constraint(c).unwrap().kind, 10.0)
                .unwrap()
                .unwrap();
            let mut out = [0.0];
            r.eval(s.params().as_slice(), &mut out);
            assert!(out[0].abs() < 1e-12);
        }
    }
}

#[test]
fn point_on_ellipse_without_aux_is_a_clear_error() {
    let mut s = new();
    let e = s.add_ellipse(p(0.0, 0.0), p(3.0, 0.0), 2.0).unwrap();
    let pt = s.add_point(p(0.0, 2.0)).unwrap();
    let kind = ConstraintKind::PointOnCurve {
        point: pt,
        curve: e,
    };
    let err = constraint_residual(&s, &kind, 1.0).unwrap_err();
    assert!(matches!(err, SketchError::NotApplicable(_)), "{err:?}");
    // With the aux param it builds.
    s.add_constraint(kind.clone()).unwrap();
    assert_eq!(build_system(&s).unwrap().system.rows(), 2);
}

// ---- Solving ------------------------------------------------------------------------------------

/// A 20 x 10 rectangle with fixed corner, H/V and two length dimensions (DOF 0); returns the
/// four lines (bottom, right, top, left).
fn fixed_rectangle(s: &mut Sketch) -> [EntityId; 4] {
    let pts =
        [p(0.0, 0.0), p(20.0, 0.0), p(20.0, 10.0), p(0.0, 10.0)].map(|q| s.add_point(q).unwrap());
    let l: [EntityId; 4] =
        std::array::from_fn(|i| s.add_line_between(pts[i], pts[(i + 1) % 4]).unwrap());
    s.add_constraint(ConstraintKind::Fix { entity: pts[0] })
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
    l
}

#[test]
fn ellipse_in_rectangle_tangent_four_times_solves() {
    let mut s = new();
    let l = fixed_rectangle(&mut s);
    assert_eq!(dof(&s).unwrap(), 0);
    let c = s.add_point(p(9.0, 5.6)).unwrap();
    let m = s.add_point(p(18.4, 5.9)).unwrap();
    // Build the ellipse over the two existing points.
    let b = s.alloc_param(4.1, Default::default()).unwrap();
    let e = s
        .add_entity(
            EntityKind::Ellipse {
                center: c,
                major: m,
                minor_radius: b,
            },
            false,
            amg_sketch::EntityOrigin::User,
        )
        .unwrap();
    for line in l {
        s.add_constraint_oriented(ConstraintKind::Tangent {
            a: line,
            b: e,
            kind: TangentKind::External,
        })
        .unwrap();
    }
    s.add_constraint(ConstraintKind::Horizontal(HvTarget::Points(c, m)))
        .unwrap();
    // The rectangle is rigid; the ellipse's 5 DOF meet 5 independent rows.
    assert_eq!(dof(&s).unwrap(), 0);
    assert_eq!(solve(&mut s), SolveStatus::Converged);
    assert!(residual_max(&s) < 1e-9);
    let ell = ellipse_curve(&s, e);
    assert!(ell.center.distance(p(10.0, 5.0)) < 1e-7, "{:?}", ell.center);
    assert!((ell.semi_major() - 10.0).abs() < 1e-7);
    assert!((ell.minor_radius - 5.0).abs() < 1e-7);
}

#[test]
fn ellipse_in_rectangle_follows_rectangle_edits() {
    let mut s = new();
    let l = fixed_rectangle(&mut s);
    let e = s.add_ellipse(p(10.0, 5.0), p(19.0, 5.0), 4.0).unwrap();
    for line in l {
        s.add_constraint_oriented(ConstraintKind::Tangent {
            a: line,
            b: e,
            kind: TangentKind::External,
        })
        .unwrap();
    }
    let kinds: Vec<_> = s.entity(e).unwrap().kind.children().to_vec();
    s.add_constraint(ConstraintKind::Horizontal(HvTarget::Points(
        kinds[0], kinds[1],
    )))
    .unwrap();
    assert_eq!(solve(&mut s), SolveStatus::Converged);
    // Make the rectangle 30 x 16: the ellipse follows.
    let dims: Vec<_> = s
        .constraints()
        .filter(|(_, c)| c.kind.as_dimension().is_some())
        .map(|(id, _)| id)
        .collect();
    s.set_dimension_value(dims[0], 30.0).unwrap();
    s.set_dimension_value(dims[1], 16.0).unwrap();
    assert_eq!(solve(&mut s), SolveStatus::Converged);
    let ell = ellipse_curve(&s, e);
    assert!(ell.center.distance(p(15.0, 8.0)) < 1e-6);
    assert!((ell.semi_major() - 15.0).abs() < 1e-6);
    assert!((ell.minor_radius - 8.0).abs() < 1e-6);
}

#[test]
fn axis_dimensions_and_point_on_ellipse_solve() {
    let mut s = new();
    let e = s.add_ellipse(p(1.0, 1.0), p(5.0, 1.2), 2.0).unwrap();
    let [c, m] = [0, 1].map(|i| s.entity(e).unwrap().kind.children()[i]);
    s.add_constraint(ConstraintKind::Fix { entity: c }).unwrap();
    s.add_constraint(ConstraintKind::Horizontal(HvTarget::Points(c, m)))
        .unwrap();
    let major = s
        .add_constraint(dim(DimensionKind::EllipseMajor { ellipse: e }, 4.0))
        .unwrap();
    let minor = s
        .add_constraint(dim(DimensionKind::EllipseMinor { ellipse: e }, 2.0))
        .unwrap();
    // A point on the ellipse, 2 mm right of the centre, in the upper half.
    let q = s.add_point(p(3.0, 2.5)).unwrap();
    s.add_constraint(ConstraintKind::PointOnCurve { point: q, curve: e })
        .unwrap();
    s.add_constraint_oriented(dim(
        DimensionKind::HorizontalDistance {
            a: c,
            b: q,
            side: amg_sketch::Side::Positive,
        },
        2.0,
    ))
    .unwrap();
    assert_eq!(dof(&s).unwrap(), 0);
    assert_eq!(solve(&mut s), SolveStatus::Converged);
    let ell = ellipse_curve(&s, e);
    assert!((ell.semi_major() - 4.0).abs() < 1e-8 && (ell.minor_radius - 2.0).abs() < 1e-8);
    let qp = s.point(q).unwrap();
    assert!((qp.x - 3.0).abs() < 1e-8);
    // y = cy + b sqrt(1 - (dx/a)^2)
    let want = 1.0 + 2.0 * (1.0f64 - 0.25).sqrt();
    assert!((qp.y - want).abs() < 1e-7, "{} vs {want}", qp.y);
    // Editing the axis dimensions moves the ellipse and keeps the point on it.
    s.set_dimension_value(major, 5.0).unwrap();
    s.set_dimension_value(minor, 3.0).unwrap();
    assert_eq!(solve(&mut s), SolveStatus::Converged);
    let qp = s.point(q).unwrap();
    let want = 1.0 + 3.0 * (1.0f64 - 0.16).sqrt();
    assert!((qp.y - want).abs() < 1e-7);
    // The measured axes are consistent with the dimension rows.
    assert!((ellipse_curve(&s, e).semi_major() - 5.0).abs() < 1e-8);
}

#[test]
fn elliptic_arc_ends_follow_the_ellipse() {
    let mut s = new();
    let a = s
        .add_elliptic_arc(p(0.0, 0.0), p(4.0, 0.0), 2.0, 0.3, 2.2)
        .unwrap();
    let (c, m) = match s.entity(a).unwrap().kind {
        EntityKind::EllipticArc { center, major, .. } => (center, major),
        _ => unreachable!(),
    };
    s.add_constraint(ConstraintKind::Fix { entity: c }).unwrap();
    s.add_constraint(ConstraintKind::Fix { entity: m }).unwrap();
    let minor = s
        .add_constraint(dim(DimensionKind::EllipseMinor { ellipse: a }, 2.0))
        .unwrap();
    // Only the two end angles are free.
    assert_eq!(dof(&s).unwrap(), 2);
    s.set_dimension_value(minor, 3.0).unwrap();
    assert!(
        residual_max(&s) > 0.5,
        "the end points no longer lie on the ellipse"
    );
    assert_eq!(native_solve(&mut s).dof.map(|d| d.dof), Some(2));
    let after = ellipse_curve(&s, a);
    assert!((after.minor_radius - 3.0).abs() < 1e-9);
    // The internal equations hold again: both end points lie on the new ellipse.
    assert!(residual_max(&s) < 1e-9);
    let EntityKind::EllipticArc { start, end, .. } = s.entity(a).unwrap().kind else {
        unreachable!()
    };
    for q in [start, end] {
        let q = s.point(q).unwrap();
        let full = amg_sketch::EllipseArc2::full(after.center, after.major, after.minor_radius);
        assert!(Curve2::Ellipse(full).closest(q).1.distance(q) < 1e-8);
    }
}

// ---- Profiles -----------------------------------------------------------------------------------

fn rect_lines(s: &mut Sketch, x0: f64, y0: f64, x1: f64, y1: f64) {
    for (a, b) in [
        (p(x0, y0), p(x1, y0)),
        (p(x1, y0), p(x1, y1)),
        (p(x1, y1), p(x0, y1)),
        (p(x0, y1), p(x0, y0)),
    ] {
        s.add_line(a, b).unwrap();
    }
}

fn profiles(s: &Sketch) -> ProfileSet {
    let set = detect_profiles(s, &ProfileOptions::default());
    assert!(set.issues.is_empty(), "{:?}", set.issues);
    assert!(set.open_chains.is_empty(), "{:?}", set.open_chains);
    set
}

fn sorted_areas(set: &ProfileSet) -> Vec<f64> {
    let mut a: Vec<f64> = set.profiles.iter().map(|q| q.fingerprint.area).collect();
    a.sort_by(f64::total_cmp);
    a
}

fn assert_areas(set: &ProfileSet, mut want: Vec<f64>) {
    want.sort_by(f64::total_cmp);
    let got = sorted_areas(set);
    assert_eq!(got.len(), want.len(), "areas {got:?}, wanted {want:?}");
    for (g, w) in got.iter().zip(&want) {
        assert!(rel(*g, *w) < 1e-9, "areas {got:?}, wanted {want:?}");
    }
}

#[test]
fn ellipse_hole_in_rectangle_has_exact_area() {
    for (major, label) in [(p(16.0, 5.0), "axis-aligned"), (p(13.0, 9.0), "rotated")] {
        let mut s = new();
        rect_lines(&mut s, 0.0, 0.0, 20.0, 10.0);
        let c = p(10.0, 5.0);
        let a = c.distance(major);
        s.add_ellipse(c, major, 3.0).unwrap();
        let set = profiles(&s);
        let ell = PI * a * 3.0;
        assert_areas(&set, vec![200.0 - ell, ell]);
        let outer = set
            .profiles
            .iter()
            .find(|q| !q.holes.is_empty())
            .expect(label);
        assert_eq!(outer.holes.len(), 1);
        assert!(rel(outer.fingerprint.area, 200.0 - ell) < 1e-9, "{label}");
        // Point containment through the ellipse loop.
        assert!(outer.contains(p(1.0, 1.0)));
        assert!(!outer.contains(c));
    }
}

#[test]
fn ellipse_cut_by_a_line_has_segment_areas() {
    let mut s = new();
    rect_lines(&mut s, -10.0, -10.0, 10.0, 10.0);
    s.add_line(p(3.0, -10.0), p(3.0, 10.0)).unwrap();
    s.add_ellipse(p(0.0, 0.0), p(6.0, 0.0), 3.0).unwrap();
    let set = profiles(&s);
    // Segment of the ellipse right of x = 3 (circle segment scaled by a·b).
    let x0 = 0.5f64;
    let seg = 18.0 * (x0.acos() - x0 * (1.0 - x0 * x0).sqrt());
    let total = 18.0 * PI;
    assert_areas(
        &set,
        vec![seg, total - seg, 140.0 - seg, 260.0 - (total - seg)],
    );
}

#[test]
fn two_shifted_ellipses_make_a_lens_and_two_crescents() {
    let mut s = new();
    s.add_ellipse(p(0.0, 0.0), p(2.0, 0.0), 1.0).unwrap();
    s.add_ellipse(p(1.5, 0.0), p(3.5, 0.0), 1.0).unwrap();
    let set = profiles(&s);
    let dp = 1.5 / 2.0;
    let lens = 2.0 * 2.0 * ((dp / 2.0f64).acos() - dp / 2.0 * (1.0 - dp * dp / 4.0).sqrt());
    let one = 2.0 * PI;
    assert_areas(&set, vec![lens, one - lens, one - lens]);
    // The lens is bounded by two elliptical arcs.
    let lens_profile = set
        .profiles
        .iter()
        .find(|q| rel(q.fingerprint.area, lens) < 1e-9)
        .unwrap();
    assert!(
        lens_profile.outer.edges.len() >= 2,
        "two arcs, one split at the seam"
    );
    assert!(
        lens_profile
            .outer
            .edges
            .iter()
            .all(|e| matches!(e.curve, Curve2::Ellipse(_)))
    );
}

#[test]
fn ellipse_inscribed_in_a_rectangle_makes_five_profiles() {
    let mut s = new();
    rect_lines(&mut s, 0.0, 0.0, 20.0, 10.0);
    s.add_ellipse(p(10.0, 5.0), p(20.0, 5.0), 5.0).unwrap();
    let set = profiles(&s);
    let corner = (200.0 - 50.0 * PI) / 4.0;
    assert_areas(&set, vec![corner, corner, corner, corner, 50.0 * PI]);
}

#[test]
fn elliptical_arcs_close_profiles() {
    for (t0, t1, label) in [
        (0.0, PI, "upper half"),
        (PI, TAU, "lower half"),
        (-1.0, 2.14159, "odd"),
    ] {
        let mut s = new();
        let arc = s
            .add_elliptic_arc(p(0.0, 0.0), p(4.0, 0.0), 2.0, t0, t1)
            .unwrap();
        let ell = ellipse_curve(&s, arc);
        let (a, b) = (ell.point_at(ell.t0), ell.point_at(ell.t1));
        s.add_line(a, b).unwrap();
        let set = profiles(&s);
        assert_eq!(set.profiles.len(), 1, "{label}");
        // Area between a chord and the elliptical arc: a b (sweep - sin sweep) / 2.
        let sweep = t1 - t0;
        let want = 0.5 * 8.0 * (sweep - sweep.sin());
        assert!(
            rel(set.profiles[0].fingerprint.area, want) < 1e-9,
            "{label}: {} vs {want}",
            set.profiles[0].fingerprint.area
        );
    }
}

#[test]
fn ellipse_boundary_contains_holes_and_points() {
    let mut s = new();
    s.add_ellipse(p(0.0, 0.0), p(8.0, 6.0), 4.0).unwrap(); // a = 10, rotated
    s.add_circle(p(1.0, 1.0), 2.0).unwrap();
    s.add_circle(p(-5.0, -3.0), 1.0).unwrap();
    let set = profiles(&s);
    let outer = set.profiles.iter().find(|q| q.holes.len() == 2).unwrap();
    assert!(rel(outer.fingerprint.area, PI * 40.0 - PI * 4.0 - PI) < 1e-9);
    assert!(outer.contains(p(5.0, 3.0)));
    assert!(!outer.contains(p(1.0, 1.0)));
    assert!(!outer.contains(p(9.0, 6.0)), "outside the ellipse");
    // Inside the bounding box of the rotated ellipse but outside the curve.
    assert!(!outer.contains(p(7.0, -3.5)));
    // Elliptical arc boundary (a "D"): containment in a half ellipse.
    let mut t = new();
    let arc = t
        .add_elliptic_arc(p(0.0, 0.0), p(8.0, 6.0), 4.0, 0.0, PI)
        .unwrap();
    let ell = ellipse_curve(&t, arc);
    t.add_line(ell.point_at(0.0), ell.point_at(PI)).unwrap();
    let set = profiles(&t);
    let d = &set.profiles[0];
    assert!(d.contains(Curve2::Ellipse(ell).eval(1.0) * 0.5));
    // The mirror point through the centre is on the other side of the chord.
    assert!(!d.contains(-(Curve2::Ellipse(ell).eval(1.0) * 0.5)));
}

#[test]
fn nested_ellipses_make_an_annulus() {
    let mut s = new();
    s.add_ellipse(p(0.0, 0.0), p(5.0, 0.0), 3.0).unwrap();
    s.add_ellipse(p(0.0, 0.0), p(2.0, 0.0), 1.0).unwrap();
    let set = profiles(&s);
    assert_areas(&set, vec![PI * 15.0 - PI * 2.0, PI * 2.0]);
}

// ---- Entity model ---------------------------------------------------------------------------------

#[test]
fn constructors_validate_their_input() {
    let mut s = new();
    assert!(matches!(
        s.add_ellipse(p(0.0, 0.0), p(1.0, 0.0), 0.0),
        Err(SketchError::InvalidValue { .. })
    ));
    assert!(matches!(
        s.add_ellipse(p(0.0, 0.0), p(1.0, 0.0), f64::NAN),
        Err(SketchError::InvalidValue { .. })
    ));
    assert!(matches!(
        s.add_ellipse(p(0.0, 0.0), p(0.0, 0.0), 1.0),
        Err(SketchError::Degenerate(_))
    ));
    assert!(matches!(
        s.add_ellipse(p(f64::INFINITY, 0.0), p(1.0, 0.0), 1.0),
        Err(SketchError::InvalidValue { .. })
    ));
    assert!(matches!(
        s.add_elliptic_arc(p(0.0, 0.0), p(2.0, 0.0), 1.0, 1.0, 1.0),
        Err(SketchError::Degenerate(_))
    ));
    assert!(matches!(
        s.add_elliptic_arc(p(0.0, 0.0), p(2.0, 0.0), 1.0, 0.0, TAU),
        Err(SketchError::Degenerate(_))
    ));
    assert!(matches!(
        s.add_elliptic_arc(p(0.0, 0.0), p(2.0, 0.0), 1.0, 0.0, f64::NAN),
        Err(SketchError::InvalidValue { .. })
    ));
    assert_eq!(s.entities().count(), 0, "failed constructors add nothing");
    // The minor radius may exceed the semi-major length.
    let e = s.add_ellipse(p(0.0, 0.0), p(1.0, 0.0), 3.0).unwrap();
    let c = ellipse_curve(&s, e);
    assert!((c.minor_radius - 3.0).abs() < 1e-15 && (c.semi_major() - 1.0).abs() < 1e-15);
}

#[test]
fn add_entity_checks_the_arc_internal_equations() {
    let mut s = new();
    let a = s
        .add_elliptic_arc(p(0.0, 0.0), p(4.0, 0.0), 2.0, 0.2, 1.0)
        .unwrap();
    let EntityKind::EllipticArc {
        center,
        major,
        minor_radius,
        end,
        t0,
        t1,
        ..
    } = s.entity(a).unwrap().kind.clone()
    else {
        unreachable!()
    };
    // A start point that is not at t0.
    let off = s.add_point(p(1.0, 1.0)).unwrap();
    let bad = EntityKind::EllipticArc {
        center,
        major,
        minor_radius: s.alloc_param(2.0, Default::default()).unwrap(),
        start: off,
        end,
        t0: s.alloc_param(0.2, Default::default()).unwrap(),
        t1: s.alloc_param(1.0, Default::default()).unwrap(),
    };
    assert!(matches!(
        s.add_entity(bad, false, amg_sketch::EntityOrigin::User),
        Err(SketchError::Degenerate(_))
    ));
    // Reusing owned params is refused.
    let reuse = EntityKind::Ellipse {
        center,
        major,
        minor_radius,
    };
    assert!(matches!(
        s.add_entity(reuse, false, amg_sketch::EntityOrigin::User),
        Err(SketchError::NotApplicable(_))
    ));
    let _ = (t0, t1);
    // A non-positive minor radius is refused.
    let b0 = s.alloc_param(-1.0, Default::default()).unwrap();
    let neg = EntityKind::Ellipse {
        center: s.add_point(p(5.0, 5.0)).unwrap(),
        major: s.add_point(p(6.0, 5.0)).unwrap(),
        minor_radius: b0,
    };
    assert!(matches!(
        s.add_entity(neg, false, amg_sketch::EntityOrigin::User),
        Err(SketchError::InvalidValue { .. })
    ));
}

#[test]
fn entity_structure_and_dof() {
    let mut s = new();
    let e = s.add_ellipse(p(0.0, 0.0), p(3.0, 0.0), 1.0).unwrap();
    let a = s
        .add_elliptic_arc(p(0.0, 5.0), p(3.0, 5.0), 1.0, 0.0, 2.0)
        .unwrap();
    let ke = s.entity(e).unwrap().kind.clone();
    let ka = s.entity(a).unwrap().kind.clone();
    assert_eq!((ke.name(), ka.name()), ("ellipse", "elliptical arc"));
    assert_eq!((ke.children().len(), ka.children().len()), (2, 4));
    assert_eq!((ke.own_params().len(), ka.own_params().len()), (1, 3));
    assert!(ke.endpoint(amg_sketch::End::Start).is_none());
    assert!(
        ka.endpoint(amg_sketch::End::Start).is_some()
            && ka.endpoint(amg_sketch::End::End).is_some()
    );
    // Free DOF: ellipse 5, elliptical arc 7 (ranks over the internal equations).
    assert_eq!(dof(&s).unwrap(), 12);
}

#[test]
fn kind_validation_is_clear_about_ellipses() {
    let mut s = new();
    let e = s.add_ellipse(p(0.0, 0.0), p(3.0, 0.0), 1.0).unwrap();
    let e2 = s.add_ellipse(p(0.0, 5.0), p(3.0, 5.0), 1.0).unwrap();
    let arc = s
        .add_elliptic_arc(p(0.0, 9.0), p(3.0, 9.0), 1.0, 0.0, 2.0)
        .unwrap();
    let arc2 = s
        .add_elliptic_arc(p(0.0, 12.0), p(3.0, 12.0), 1.0, 0.0, 2.0)
        .unwrap();
    let circ = s.add_circle(p(8.0, 8.0), 1.0).unwrap();
    let ln = s.add_line(p(-5.0, 20.0), p(5.0, 20.0)).unwrap();
    let pt = s.add_point(p(7.0, 7.0)).unwrap();
    let wrong = |s: &mut Sketch, k: ConstraintKind| match s.add_constraint(k) {
        Err(SketchError::WrongEntityKind { .. }) => {}
        other => panic!("expected WrongEntityKind, got {other:?}"),
    };
    let not_applicable = |s: &mut Sketch, k: ConstraintKind| match s.add_constraint(k) {
        Err(SketchError::NotApplicable(_)) => {}
        other => panic!("expected NotApplicable, got {other:?}"),
    };
    wrong(&mut s, dim(DimensionKind::Radius { curve: e }, 1.0));
    wrong(&mut s, dim(DimensionKind::Diameter { curve: arc }, 1.0));
    wrong(&mut s, dim(DimensionKind::ArcLength { arc }, 1.0));
    wrong(
        &mut s,
        dim(DimensionKind::EllipseMajor { ellipse: circ }, 1.0),
    );
    wrong(
        &mut s,
        dim(DimensionKind::EllipseMinor { ellipse: ln }, 1.0),
    );
    wrong(&mut s, ConstraintKind::Equal { a: e, b: circ });
    wrong(&mut s, ConstraintKind::Equal { a: circ, b: e });
    wrong(&mut s, ConstraintKind::Equal { a: ln, b: e });
    wrong(&mut s, ConstraintKind::Concentric { a: e, b: ln });
    wrong(&mut s, ConstraintKind::Midpoint { point: pt, line: e });
    wrong(&mut s, ConstraintKind::Parallel { a: e, b: ln });
    wrong(&mut s, ConstraintKind::Horizontal(HvTarget::Line(e)));
    wrong(
        &mut s,
        ConstraintKind::Symmetric {
            a: e,
            b: circ,
            axis: ln,
        },
    );
    // Distances to an ellipse are not supported.
    wrong(
        &mut s,
        dim(
            DimensionKind::Distance {
                a: pt,
                b: e,
                side: amg_sketch::Side::Positive,
            },
            1.0,
        ),
    );
    wrong(
        &mut s,
        dim(
            DimensionKind::Distance {
                a: arc,
                b: ln,
                side: amg_sketch::Side::Positive,
            },
            1.0,
        ),
    );
    // Tangency: only line-ellipse.
    not_applicable(
        &mut s,
        ConstraintKind::Tangent {
            a: e,
            b: circ,
            kind: TangentKind::External,
        },
    );
    not_applicable(
        &mut s,
        ConstraintKind::Tangent {
            a: circ,
            b: e,
            kind: TangentKind::Internal,
        },
    );
    not_applicable(
        &mut s,
        ConstraintKind::Tangent {
            a: e,
            b: e2,
            kind: TangentKind::External,
        },
    );
    wrong(
        &mut s,
        ConstraintKind::Tangent {
            a: e,
            b: pt,
            kind: TangentKind::External,
        },
    );
    let at_end = TangentKind::AtEndpoint {
        a_end: amg_sketch::End::Start,
        b_end: amg_sketch::End::Start,
        reversed: false,
    };
    wrong(
        &mut s,
        ConstraintKind::Tangent {
            a: arc,
            b: arc2,
            kind: at_end,
        },
    );
    wrong(
        &mut s,
        ConstraintKind::Tangent {
            a: ln,
            b: arc,
            kind: at_end,
        },
    );
    // Symmetry of elliptical arcs is not supported; full ellipses are.
    not_applicable(
        &mut s,
        ConstraintKind::Symmetric {
            a: arc,
            b: arc2,
            axis: ln,
        },
    );
    s.add_constraint(ConstraintKind::Symmetric {
        a: e,
        b: e2,
        axis: ln,
    })
    .unwrap();
    // What is supported.
    s.add_constraint(ConstraintKind::Concentric { a: e, b: circ })
        .unwrap();
    s.add_constraint(ConstraintKind::Concentric { a: arc, b: e2 })
        .unwrap();
    s.add_constraint(ConstraintKind::Equal { a: arc, b: arc2 })
        .unwrap();
    s.add_constraint(ConstraintKind::Equal { a: e, b: arc })
        .unwrap();
    s.add_constraint(ConstraintKind::PointOnCurve {
        point: pt,
        curve: arc,
    })
    .unwrap();
    s.add_constraint(ConstraintKind::Tangent {
        a: ln,
        b: arc,
        kind: TangentKind::Internal,
    })
    .unwrap();
    // A point cannot be on its own ellipse.
    let EntityKind::Ellipse { center, .. } = s.entity(e).unwrap().kind else {
        unreachable!()
    };
    not_applicable(
        &mut s,
        ConstraintKind::PointOnCurve {
            point: center,
            curve: e,
        },
    );
    // Axis dimensions need positive values.
    assert!(matches!(
        s.add_constraint(dim(DimensionKind::EllipseMajor { ellipse: e }, 0.0)),
        Err(SketchError::InvalidValue { .. })
    ));
    let d = s
        .add_constraint(dim(DimensionKind::EllipseMinor { ellipse: e2 }, 2.0))
        .unwrap();
    assert!(matches!(
        s.set_dimension_value(d, -1.0),
        Err(SketchError::InvalidValue { .. })
    ));
    assert_eq!(
        DimensionKind::EllipseMajor { ellipse: e }.name(),
        "ellipse major radius"
    );
    assert_eq!(
        DimensionKind::EllipseMinor { ellipse: e }.quantity(),
        amg_sketch::Quantity::Length
    );
}

#[test]
fn point_on_ellipse_owns_an_aux_param() {
    let mut s = new();
    let e = s.add_ellipse(p(0.0, 0.0), p(4.0, 0.0), 2.0).unwrap();
    let q = s.add_point(p(0.0, 2.0)).unwrap();
    let r = s.add_point(p(9.0, 9.0)).unwrap();
    let ln = s.add_line(p(5.0, 5.0), p(8.0, 5.0)).unwrap();
    let n0 = s.params().len();
    let on = s
        .add_constraint(ConstraintKind::PointOnCurve { point: q, curve: e })
        .unwrap();
    assert_eq!(s.params().len(), n0 + 1);
    let aux = s.constraint(on).unwrap().aux[0];
    // The start value is the point's angle on the ellipse (pi/2 for the top of the minor axis).
    assert!((s.params().get(aux) - FRAC_PI_2).abs() < 1e-9);
    // A point on a line has no aux param.
    let on_line = s
        .add_constraint(ConstraintKind::PointOnCurve {
            point: r,
            curve: ln,
        })
        .unwrap();
    assert!(s.constraint(on_line).unwrap().aux.is_empty());
    // The aux param is a solver unknown.
    assert!(build_system(&s).unwrap().free.contains(&aux));
    // Removing the constraint retires the slot; compact drops it.
    s.remove_constraint(on).unwrap();
    assert!(
        s.add_entity(
            EntityKind::Ellipse {
                center: q,
                major: r,
                minor_radius: aux
            },
            false,
            amg_sketch::EntityOrigin::User
        )
        .is_err()
    );
    s.compact();
    assert_eq!(s.params().len(), n0);
    // Removing the ellipse removes its constraints, including the point-on-ellipse.
    let on2 = s
        .add_constraint(ConstraintKind::PointOnCurve { point: q, curve: e })
        .unwrap();
    let rep = s.remove_entities(&[e]).unwrap();
    assert!(rep.constraints.contains(&on2));
    assert!(s.entity(e).is_none());
    assert!(s.point(q).is_some(), "the free point survives");
}

#[test]
fn ellipses_round_trip_through_serde_and_load_validation() {
    let mut s = new();
    let e = s.add_ellipse(p(1.0, 2.0), p(4.0, 3.0), 1.5).unwrap();
    let a = s
        .add_elliptic_arc(p(0.0, 8.0), p(3.0, 8.0), 1.0, -0.4, 2.0)
        .unwrap();
    let q = s.add_point(p(1.0, 3.0)).unwrap();
    s.add_constraint(ConstraintKind::PointOnCurve { point: q, curve: e })
        .unwrap();
    s.add_constraint(dim(DimensionKind::EllipseMinor { ellipse: a }, 1.0))
        .unwrap();
    s.add_constraint(dim(DimensionKind::EllipseMajor { ellipse: e }, 3.1))
        .unwrap();
    s.add_constraint(ConstraintKind::Equal { a: e, b: a })
        .unwrap();
    let json = serde_json::to_string(&s).unwrap();
    let back: Sketch = serde_json::from_str(&json).unwrap();
    assert_eq!(serde_json::to_string(&back).unwrap(), json);
    assert_eq!(back.curve(e), s.curve(e));
    assert_eq!(back.curve(a), s.curve(a));

    // A point-on-ellipse constraint without its aux param does not load.
    let mut v: serde_json::Value = serde_json::from_str(&json).unwrap();
    for c in v["constraints"].as_object_mut().unwrap().values_mut() {
        c["aux"] = serde_json::json!([]);
    }
    assert!(serde_json::from_value::<Sketch>(v).is_err());
    // An aux param on a constraint that has none does not load either.
    let mut v: serde_json::Value = serde_json::from_str(&json).unwrap();
    let eq = v["constraints"]
        .as_object_mut()
        .unwrap()
        .values_mut()
        .find(|c| c["kind"].get("Equal").is_some())
        .unwrap();
    eq["aux"] = serde_json::json!([0]);
    assert!(serde_json::from_value::<Sketch>(v).is_err());

    // An elliptical arc whose end is not at t1, or an ellipse with a negative minor radius.
    let mut bad = s.clone();
    let EntityKind::EllipticArc {
        end, minor_radius, ..
    } = bad.entity(a).unwrap().kind
    else {
        unreachable!()
    };
    let EntityKind::Point { x, .. } = bad.entity(end).unwrap().kind else {
        unreachable!()
    };
    let old = bad.params().get(x);
    bad.params_mut().set(x, old + 1.0);
    assert!(serde_json::from_str::<Sketch>(&serde_json::to_string(&bad).unwrap()).is_err());
    bad.params_mut().set(x, old);
    bad.params_mut().set(minor_radius, -1.0);
    assert!(serde_json::from_str::<Sketch>(&serde_json::to_string(&bad).unwrap()).is_err());
}

// ---- Operations ---------------------------------------------------------------------------------

#[test]
fn trim_and_extend_use_an_ellipse_as_the_cutter() {
    // Trim: the part of a line left of the ellipse goes; the new end sits on the ellipse.
    let mut s = new();
    let e = s.add_ellipse(p(0.0, 0.0), p(4.0, 0.0), 2.0).unwrap();
    let l = s.add_line(p(-10.0, 0.0), p(10.0, 0.0)).unwrap();
    let res = edit::trim(&mut s, l, p(-7.0, 0.0)).unwrap();
    assert!(res.removed.contains(&l));
    let piece = res
        .created
        .iter()
        .copied()
        .find(|&id| matches!(s.entity(id).unwrap().kind, EntityKind::Line { .. }))
        .unwrap();
    let Some(Curve2::Line(seg)) = s.curve(piece) else {
        panic!("line expected")
    };
    assert!(seg.a.distance(p(-4.0, 0.0)) < 1e-9 && seg.b.distance(p(10.0, 0.0)) < 1e-9);
    // The new end point is pinned to the ellipse (aux param allocated), and the sketch solves.
    let pin = res
        .constraints_added
        .iter()
        .copied()
        .find(|&c| {
            matches!(
                s.constraint(c).unwrap().kind,
                ConstraintKind::PointOnCurve { curve, .. } if curve == e
            )
        })
        .unwrap();
    assert_eq!(s.constraint(pin).unwrap().aux.len(), 1);
    assert!(residual_max(&s) < 1e-9);
    let before = s.params().as_slice().to_vec();
    assert_eq!(solve(&mut s), SolveStatus::Converged);
    for (a, b) in before.iter().zip(s.params().as_slice()) {
        assert!((a - b).abs() < 1e-8, "the solve moved a parameter");
    }

    // Extend: a short line reaches the ellipse.
    let mut s = new();
    let e = s.add_ellipse(p(0.0, 0.0), p(4.0, 0.0), 2.0).unwrap();
    let l = s.add_line(p(-10.0, 1.0), p(-7.0, 1.0)).unwrap();
    let res = edit::extend(&mut s, l, p(-7.0, 1.0)).unwrap();
    let new_line = res
        .created
        .iter()
        .copied()
        .find(|&id| matches!(s.entity(id).unwrap().kind, EntityKind::Line { .. }))
        .unwrap();
    let Some(Curve2::Line(seg)) = s.curve(new_line) else {
        panic!("line expected")
    };
    let hit = seg.b;
    assert!(hit.x < 0.0 && (hit.y - 1.0).abs() < 1e-12);
    assert!((hit.x * hit.x / 16.0 + hit.y * hit.y / 4.0 - 1.0).abs() < 1e-9);
    assert!(
        s.constraints().any(
            |(_, c)| matches!(c.kind, ConstraintKind::PointOnCurve { curve, .. } if curve == e)
        )
    );
    assert!(residual_max(&s) < 1e-9);

    // A circle cut by an ellipse trims to an arc between two of the four crossings.
    let mut s = new();
    s.add_ellipse(p(0.0, 0.0), p(5.0, 0.0), 2.0).unwrap();
    let circle = s.add_circle(p(0.0, 0.0), 3.0).unwrap();
    let res = edit::trim(&mut s, circle, p(3.0, 0.0)).unwrap();
    assert!(res.removed.contains(&circle));
    let arc = res
        .created
        .iter()
        .copied()
        .find(|&id| matches!(s.entity(id).unwrap().kind, EntityKind::Arc { .. }))
        .unwrap();
    let Some(Curve2::Arc(a)) = s.curve(arc) else {
        panic!("arc expected")
    };
    // The four crossings are at +-alpha and PI +- alpha; the pick at angle 0 sits between
    // -alpha and +alpha, so the long way round survives.
    let y2: f64 = 16.0 / 5.25;
    let alpha = y2.sqrt().atan2((9.0 - y2).sqrt());
    assert!((a.sweep - (TAU - 2.0 * alpha)).abs() < 1e-9, "{}", a.sweep);
}

#[test]
fn trim_extend_and_break_refuse_to_edit_an_ellipse() {
    let mut s = new();
    let e = s.add_ellipse(p(0.0, 0.0), p(4.0, 0.0), 2.0).unwrap();
    let a = s
        .add_elliptic_arc(p(0.0, 9.0), p(4.0, 9.0), 2.0, 0.0, 2.0)
        .unwrap();
    s.add_line(p(-10.0, 0.0), p(10.0, 0.0)).unwrap();
    for id in [e, a] {
        for r in [
            edit::trim(&mut s, id, p(0.0, 2.0)).map(|_| ()),
            edit::extend(&mut s, id, p(0.0, 2.0)).map(|_| ()),
            edit::break_at(&mut s, id, p(0.0, 2.0)).map(|_| ()),
        ] {
            assert!(
                matches!(r, Err(SketchError::WrongEntityKind { .. })),
                "{r:?}"
            );
        }
    }
}

#[test]
fn offset_refuses_ellipses() {
    let mut s = new();
    let e = s.add_ellipse(p(0.0, 0.0), p(4.0, 0.0), 2.0).unwrap();
    let a = s
        .add_elliptic_arc(p(0.0, 9.0), p(4.0, 9.0), 2.0, 0.0, 2.0)
        .unwrap();
    let l = s.add_line(p(0.0, 20.0), p(5.0, 20.0)).unwrap();
    for chain in [vec![e], vec![a], vec![l, e]] {
        let n = s.entities().count();
        let r = offset::offset(&mut s, &chain, 1.0, p(0.0, 3.0), false);
        assert!(matches!(r, Err(SketchError::NotApplicable(_))), "{r:?}");
        assert_eq!(s.entities().count(), n, "a refused offset adds nothing");
    }
}

#[test]
fn circle_2_tangent_refuses_ellipses() {
    let mut s = new();
    let e = s.add_ellipse(p(0.0, 0.0), p(4.0, 0.0), 2.0).unwrap();
    let l = s.add_line(p(-10.0, 5.0), p(10.0, 5.0)).unwrap();
    let r = build::circle_2_tangent(&mut s, e, l, 1.0, p(0.0, 3.5));
    assert!(matches!(r, Err(SketchError::NotApplicable(_))), "{r:?}");
}

#[test]
fn reference_oracle_declines_ellipses() {
    let mut s = new();
    s.add_ellipse(p(0.0, 0.0), p(4.0, 0.0), 2.0).unwrap();
    // The independent oracle models no ellipses: it says so instead of guessing.
    match RefSystem::new(&s) {
        Err(msg) => assert!(msg.contains("ellipse"), "{msg}"),
        Ok(sys) => {
            let u = sys.current(&s);
            assert!(sys.residuals(&s, &u).is_err());
        }
    }
}

/// Distance of `q` from the curve `id`.
fn dist_to(s: &Sketch, id: EntityId, q: Point2) -> f64 {
    let c = s.curve(id).unwrap();
    c.closest(q).1.distance(q)
}

#[test]
fn mirror_unconstrained_copies_are_independent_images() {
    let mut s = new();
    let e = s.add_ellipse(p(1.0, 2.0), p(4.0, 3.0), 1.5).unwrap();
    let a = s
        .add_elliptic_arc(p(0.0, 8.0), p(3.0, 9.0), 1.0, -0.4, 2.0)
        .unwrap();
    let axis = s.add_line(p(5.0, -3.0), p(6.0, 10.0)).unwrap();
    let res = mirror(&mut s, &[e, a], axis, false).unwrap();
    let images: Vec<EntityId> = res
        .created
        .iter()
        .copied()
        .filter(|&id| s.entity(id).unwrap().kind.name().contains("ellip"))
        .collect();
    assert_eq!(images.len(), 2);
    let d = Vec2::new(1.0, 13.0).normalize();
    let refl = |q: Point2| {
        let r = q - p(5.0, -3.0);
        p(5.0, -3.0) + 2.0 * r.dot(d) * d - r
    };
    for (src, img) in [(e, images[0]), (a, images[1])] {
        let c = s.curve(src).unwrap();
        let end = c.param_range().1;
        for k in 0..=20 {
            let q = refl(c.eval(end * f64::from(k) / 20.0));
            assert!(dist_to(&s, img, q) < 1e-9, "{k}");
        }
    }
    // The image arc runs from the mirrored end to the mirrored start.
    let ca = s.curve(a).unwrap();
    let ci = s.curve(images[1]).unwrap();
    assert!(ci.eval(0.0).distance(refl(ca.eval(ca.param_range().1))) < 1e-9);
    assert!(ci.eval(ci.param_range().1).distance(refl(ca.eval(0.0))) < 1e-9);
    // Independent: moving the source does not move the image.
    let before = s.curve(images[0]).unwrap();
    let EntityKind::Ellipse { minor_radius, .. } = s.entity(e).unwrap().kind else {
        unreachable!()
    };
    s.params_mut().set(minor_radius, 2.5);
    assert_eq!(s.curve(images[0]).unwrap(), before);
}

#[test]
fn mirror_constrained_follows_the_source() {
    let mut s = new();
    let e = s.add_ellipse(p(1.0, 2.0), p(4.0, 3.0), 1.5).unwrap();
    let a = s
        .add_elliptic_arc(p(0.0, 8.0), p(3.0, 9.0), 1.0, -0.4, 2.0)
        .unwrap();
    let axis = s.add_line(p(5.0, -3.0), p(5.0, 10.0)).unwrap();
    let dof_before = dof(&s).unwrap();
    let res = mirror(&mut s, &[e, a], axis, true).unwrap();
    assert_eq!(
        dof(&s).unwrap(),
        dof_before,
        "a constrained mirror adds no freedom"
    );
    let images: Vec<EntityId> = res
        .created
        .iter()
        .copied()
        .filter(|&id| s.entity(id).unwrap().kind.name().contains("ellip"))
        .collect();
    assert_eq!(images.len(), 2);
    // Edit the source ellipse's minor radius and move its centre; the image follows.
    let EntityKind::Ellipse {
        center,
        minor_radius,
        ..
    } = s.entity(e).unwrap().kind
    else {
        unreachable!()
    };
    s.params_mut().set(minor_radius, 2.2);
    let EntityKind::Point { x, .. } = s.entity(center).unwrap().kind else {
        unreachable!()
    };
    let x0 = s.params().get(x);
    s.params_mut().set(x, x0 - 0.5);
    // The source is held in place (the solver is not a drag), so the image must follow it.
    s.add_constraint(ConstraintKind::Fix { entity: e }).unwrap();
    s.add_constraint(ConstraintKind::Fix { entity: a }).unwrap();
    s.add_constraint(ConstraintKind::Fix { entity: axis })
        .unwrap();
    let report = native_solve(&mut s);
    assert!(
        matches!(
            report.status,
            SolveStatus::Converged | SolveStatus::ConvergedRedundant
        ),
        "{:?}",
        report.status
    );
    assert!(residual_max(&s) < 1e-8);
    let ci = ellipse_curve(&s, images[0]);
    let cs = ellipse_curve(&s, e);
    assert!((ci.minor_radius - 2.2).abs() < 1e-7);
    assert!(
        (ci.center.x - (10.0 - cs.center.x)).abs() < 1e-7
            && (ci.center.y - cs.center.y).abs() < 1e-7,
        "{ci:?} vs {cs:?}"
    );
    // The image arc is still the mirror of the source arc.
    let src = s.curve(a).unwrap();
    let end = src.param_range().1;
    for k in 0..=10 {
        let q = src.eval(end * f64::from(k) / 10.0);
        assert!(dist_to(&s, images[1], p(10.0 - q.x, q.y)) < 1e-6);
    }
}

#[test]
fn patterns_and_transforms_carry_ellipses() {
    let mut s = new();
    let e = s.add_ellipse(p(1.0, 2.0), p(4.0, 3.0), 1.5).unwrap();
    let a = s
        .add_elliptic_arc(p(0.0, 8.0), p(3.0, 9.0), 1.0, -0.4, 2.0)
        .unwrap();
    let src: Vec<Curve2> = [e, a].iter().map(|&i| s.curve(i).unwrap()).collect();

    // Rectangular pattern, unconstrained: 3 columns.
    let res = pattern_rect(&mut s, &[e, a], Vec2::X, 3, 10.0, None, false).unwrap();
    let imgs: Vec<EntityId> = res
        .created
        .iter()
        .copied()
        .filter(|&id| s.entity(id).unwrap().kind.name().contains("ellip"))
        .collect();
    assert_eq!(imgs.len(), 4);
    for (k, id) in imgs.iter().enumerate() {
        let (which, col) = (k % 2, k / 2 + 1);
        let c = &src[which];
        let end = c.param_range().1;
        for j in 0..=8 {
            let q = c.eval(end * f64::from(j) / 8.0) + Vec2::X * (10.0 * col as f64);
            assert!(dist_to(&s, *id, q) < 1e-9);
        }
    }

    // Constrained circular pattern: the copy rotates about the origin and keeps its axes.
    let mut s = new();
    let e = s.add_ellipse(p(3.0, 0.0), p(5.0, 1.0), 1.0).unwrap();
    let origin = s.add_point(p(0.0, 0.0)).unwrap();
    let before = dof(&s).unwrap();
    let res = pattern_circ(&mut s, &[e], origin, 3, TAU, true).unwrap();
    assert_eq!(dof(&s).unwrap(), before);
    let imgs: Vec<EntityId> = res
        .created
        .iter()
        .copied()
        .filter(|&id| matches!(s.entity(id).unwrap().kind, EntityKind::Ellipse { .. }))
        .collect();
    assert_eq!(imgs.len(), 2);
    let rot = rotation_about(p(0.0, 0.0), TAU / 3.0);
    for q in 0..8 {
        let th = f64::from(q) * 0.8;
        let pt = ellipse_curve(&s, e).point_at(th);
        assert!(dist_to(&s, imgs[0], rot.transform_point2(pt)) < 1e-9);
    }
    // Moving the source's minor radius drags the images (re-solve).
    let EntityKind::Ellipse { minor_radius, .. } = s.entity(e).unwrap().kind else {
        unreachable!()
    };
    s.params_mut().set(minor_radius, 1.7);
    s.add_constraint(ConstraintKind::Fix { entity: e }).unwrap();
    s.add_constraint(ConstraintKind::Fix { entity: origin })
        .unwrap();
    assert!(matches!(
        solve(&mut s),
        SolveStatus::Converged | SolveStatus::ConvergedRedundant
    ));
    for id in &imgs {
        assert!((ellipse_curve(&s, *id).minor_radius - 1.7).abs() < 1e-7);
    }

    // transform: copy with rotation and scale, then in place scale about a point.
    let mut s = new();
    let e = s.add_ellipse(p(1.0, 2.0), p(4.0, 3.0), 1.5).unwrap();
    let a = s
        .add_elliptic_arc(p(0.0, 8.0), p(3.0, 9.0), 1.0, -0.4, 2.0)
        .unwrap();
    let ax = s
        .add_constraint(dim(DimensionKind::EllipseMinor { ellipse: e }, 1.5))
        .unwrap();
    let xf: Transform2 = scale_about(p(0.0, 0.0), 2.0) * rotation_about(p(1.0, 1.0), 0.7);
    let n = s.entities().count();
    let res = transform(&mut s, &[e, a], xf, true).unwrap();
    assert!(s.entities().count() > n);
    let copies: Vec<EntityId> = res
        .created
        .iter()
        .copied()
        .filter(|&id| s.entity(id).unwrap().kind.name().contains("ellip"))
        .collect();
    assert_eq!(copies.len(), 2);
    for (srcid, img) in [(e, copies[0]), (a, copies[1])] {
        let c = s.curve(srcid).unwrap();
        let end = c.param_range().1;
        for j in 0..=8 {
            let q = xf.transform_point2(c.eval(end * f64::from(j) / 8.0));
            assert!(dist_to(&s, img, q) < 1e-9);
        }
    }
    // The copied minor-axis dimension is scaled.
    let scaled = s
        .constraints()
        .filter_map(|(_, c)| c.kind.as_dimension())
        .filter(
            |d| matches!(d.kind, DimensionKind::EllipseMinor { ellipse } if ellipse == copies[0]),
        )
        .map(|d| d.value)
        .next()
        .unwrap();
    assert!((scaled - 3.0).abs() < 1e-12);
    // In place: scale the original about the origin.
    let c0 = s.curve(e).unwrap();
    transform(&mut s, &[e], scale_about(p(0.0, 0.0), 2.0), false).unwrap();
    let c1 = s.curve(e).unwrap();
    assert!((c1.length() - 2.0 * c0.length()).abs() < 1e-9);
    match s.constraint(ax).unwrap().kind.as_dimension() {
        Some(d) => assert!((d.value - 3.0).abs() < 1e-12),
        None => panic!("dimension expected"),
    }
    assert!(residual_max(&s) < 1e-9);
}

/// The SK-03 production solver with total DOF analysis.
fn native_solve(s: &mut Sketch) -> SolveReport {
    NativeSolver::new().solve(
        s,
        &SolveOptions {
            dof: DofAnalysis::Total,
            ..SolveOptions::default()
        },
        &|| false,
    )
}
