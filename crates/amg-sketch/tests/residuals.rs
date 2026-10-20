//! SK-02 acceptance: every P0 constraint and dimension residual passes a central-difference
//! Jacobian check (relative error < 1e-6, absolute floor 1e-9) on 1000 random configurations
//! each, plus sign conventions and system assembly.

#![allow(
    clippy::unwrap_used,
    clippy::float_cmp,
    reason = "test helpers; a failure is a test failure"
)]

use std::f64::consts::{PI, TAU};

use amg_core::{Plane, Point2, Vec2};
use amg_sketch::solve::{
    EquationOwner, Residual, build_system, constraint_residual, entity_residual, fixed_params,
};
use amg_sketch::{
    AngleBranch, ConstraintKind, Dimension, DimensionKind, End, EntityId, EntityKind, EntityOrigin,
    HvTarget, Side, Sketch, TangentKind,
};
use proptest::prelude::*;

// ---- Finite-difference check ------------------------------------------------------------

/// 5-point central difference, `h` = 1e-3 mm (truncation O(h⁴), round-off ~1e-10 here).
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
    for (k, p) in r.params().iter().enumerate() {
        let x0 = x[p.index()];
        let mut at = |d: f64| {
            xx[p.index()] = x0 + d;
            let v = f(&xx);
            xx[p.index()] = x0;
            v
        };
        let (p2, p1, m1, m2) = (at(2.0 * h), at(h), at(-h), at(-2.0 * h));
        for i in 0..m {
            let fd = (-p2[i] + 8.0 * p1[i] - 8.0 * m1[i] + m2[i]) / (12.0 * h);
            let a = j[i * n + k];
            let err = (a - fd).abs();
            if err > 1e-9 && err > 1e-6 * a.abs().max(fd.abs()) {
                return Err(format!(
                    "row {i}, param {p:?}: analytic {a:e}, finite difference {fd:e}"
                ));
            }
        }
    }
    Ok(())
}

// ---- Random geometry ----------------------------------------------------------------------

/// Deterministic geometry from proptest seeds in [−1, 1).
struct Gen<'a> {
    seeds: &'a [f64],
    i: usize,
}

impl Gen<'_> {
    fn next(&mut self) -> f64 {
        let v = self.seeds[self.i % self.seeds.len()];
        self.i += 1;
        v
    }
    fn unit(&mut self) -> f64 {
        0.5 * (self.next() + 1.0)
    }
    fn pt(&mut self) -> Point2 {
        Point2::new(100.0 * self.next(), 100.0 * self.next())
    }
    fn len(&mut self) -> f64 {
        2.0 + 96.0 * self.unit()
    }
    fn radius(&mut self) -> f64 {
        2.0 + 48.0 * self.unit()
    }
    fn ang(&mut self) -> f64 {
        PI * self.next()
    }
    fn sweep(&mut self) -> f64 {
        0.3 + (TAU - 0.6) * self.unit()
    }
}

fn dir(a: f64) -> Vec2 {
    Vec2::new(a.cos(), a.sin())
}

fn point(s: &mut Sketch, g: &mut Gen<'_>) -> EntityId {
    let p = g.pt();
    s.add_point(p).unwrap()
}

fn line(s: &mut Sketch, g: &mut Gen<'_>) -> EntityId {
    let p = g.pt();
    let q = p + dir(g.ang()) * g.len();
    s.add_line(p, q).unwrap()
}

fn circle(s: &mut Sketch, g: &mut Gen<'_>) -> EntityId {
    let c = g.pt();
    let r = g.radius();
    s.add_circle(c, r).unwrap()
}

fn arc(s: &mut Sketch, g: &mut Gen<'_>) -> EntityId {
    let c = g.pt();
    let r = g.radius();
    let t = g.ang();
    let w = g.sweep();
    s.add_arc(c, c + dir(t) * r, c + dir(t + w) * r).unwrap()
}

/// An arc starting at existing point `start`.
fn arc_from(s: &mut Sketch, g: &mut Gen<'_>, start: EntityId) -> EntityId {
    let p = s.point(start).unwrap();
    let r = g.radius();
    let c = p + dir(g.ang()) * r;
    let t = (p - c).to_angle() + g.sweep();
    let center = s.add_point(c).unwrap();
    let end = s.add_point(c + dir(t) * r).unwrap();
    s.add_entity(
        EntityKind::Arc { center, start, end },
        false,
        EntityOrigin::User,
    )
    .unwrap()
}

fn end_of(s: &Sketch, curve: EntityId, end: End) -> EntityId {
    s.entity(curve).unwrap().kind.endpoint(end).unwrap()
}

fn dim(kind: DimensionKind, value: f64) -> ConstraintKind {
    ConstraintKind::Dimension(Dimension {
        kind,
        value,
        driving: true,
        label_pos: Point2::ZERO,
    })
}

fn side(g: &mut Gen<'_>) -> Side {
    if g.next() < 0.0 {
        Side::Positive
    } else {
        Side::Negative
    }
}

fn round_radius(s: &Sketch, e: EntityId) -> f64 {
    match s.curve(e).unwrap() {
        amg_sketch::Curve2::Circle(c) => c.radius,
        amg_sketch::Curve2::Arc(a) => a.radius,
        amg_sketch::Curve2::Line(_) => unreachable!(),
    }
}

fn round_center(s: &Sketch, e: EntityId) -> Point2 {
    match s.curve(e).unwrap() {
        amg_sketch::Curve2::Circle(c) => c.center,
        amg_sketch::Curve2::Arc(a) => a.center,
        amg_sketch::Curve2::Line(_) => unreachable!(),
    }
}

/// Builds the case, adds the constraint (oriented when asked), checks the stored constraint's
/// Jacobian. A builder returning `None` skips a (rare) degenerate configuration.
fn run(
    seeds: &[f64],
    lc: f64,
    oriented: bool,
    build: impl Fn(&mut Sketch, &mut Gen<'_>) -> Option<ConstraintKind>,
) -> Result<(), TestCaseError> {
    let mut s = Sketch::new(Plane::XY);
    let mut g = Gen { seeds, i: 0 };
    let Some(kind) = build(&mut s, &mut g) else {
        return Ok(());
    };
    let id = if oriented {
        s.add_constraint_oriented(kind)
    } else {
        s.add_constraint(kind)
    }
    .map_err(|e| TestCaseError::fail(format!("add: {e}")))?;
    let kind = s.constraint(id).unwrap().kind.clone();
    let r = constraint_residual(&s, &kind, lc).unwrap().unwrap();
    prop_assert!(r.rows() > 0);
    fd_check(&r, s.params().as_slice()).map_err(|e| TestCaseError::fail(format!("{kind:?}: {e}")))
}

macro_rules! fd_tests {
    ($($name:ident: $oriented:expr, |$s:ident, $g:ident| $body:expr;)*) => {
        proptest! {
            #![proptest_config(ProptestConfig::with_cases(1000))]
            $(
                #[test]
                fn $name(seeds in prop::collection::vec(-1.0f64..1.0, 48), lc in 1.0f64..300.0) {
                    run(&seeds, lc, $oriented, |$s: &mut Sketch, $g: &mut Gen<'_>| $body)?;
                }
            )*
        }
    };
}

fd_tests! {
    coincident: false, |s, g| {
        let (a, b) = (point(s, g), point(s, g));
        Some(ConstraintKind::Coincident { a, b })
    };
    point_on_line: false, |s, g| {
        let (point, curve) = (point(s, g), line(s, g));
        Some(ConstraintKind::PointOnCurve { point, curve })
    };
    point_on_circle: false, |s, g| {
        let (point, curve) = (point(s, g), circle(s, g));
        let c = round_center(s, curve);
        (s.point(point)?.distance(c) > 1.0).then_some(ConstraintKind::PointOnCurve { point, curve })
    };
    point_on_arc: false, |s, g| {
        let (point, curve) = (point(s, g), arc(s, g));
        let c = round_center(s, curve);
        (s.point(point)?.distance(c) > 1.0).then_some(ConstraintKind::PointOnCurve { point, curve })
    };
    midpoint: false, |s, g| {
        let (point, line) = (point(s, g), line(s, g));
        Some(ConstraintKind::Midpoint { point, line })
    };
    horizontal_line: false, |s, g| Some(ConstraintKind::Horizontal(HvTarget::Line(line(s, g))));
    horizontal_points: false, |s, g| {
        let (a, b) = (point(s, g), point(s, g));
        Some(ConstraintKind::Horizontal(HvTarget::Points(a, b)))
    };
    vertical_line: false, |s, g| Some(ConstraintKind::Vertical(HvTarget::Line(line(s, g))));
    vertical_points: false, |s, g| {
        let (a, b) = (point(s, g), point(s, g));
        Some(ConstraintKind::Vertical(HvTarget::Points(a, b)))
    };
    parallel: false, |s, g| {
        let (a, b) = (line(s, g), line(s, g));
        Some(ConstraintKind::Parallel { a, b })
    };
    perpendicular: false, |s, g| {
        let (a, b) = (line(s, g), line(s, g));
        Some(ConstraintKind::Perpendicular { a, b })
    };
    collinear: false, |s, g| {
        let (a, b) = (line(s, g), line(s, g));
        Some(ConstraintKind::Collinear { a, b })
    };
    tangent_line_circle_external: false, |s, g| {
        let (a, b) = (line(s, g), circle(s, g));
        Some(ConstraintKind::Tangent { a, b, kind: TangentKind::External })
    };
    tangent_circle_line_internal: false, |s, g| {
        let (a, b) = (circle(s, g), line(s, g));
        Some(ConstraintKind::Tangent { a, b, kind: TangentKind::Internal })
    };
    tangent_line_arc: false, |s, g| {
        let (a, b) = (line(s, g), arc(s, g));
        let kind = if g.next() < 0.0 { TangentKind::External } else { TangentKind::Internal };
        Some(ConstraintKind::Tangent { a, b, kind })
    };
    tangent_circles_external: false, |s, g| {
        let (a, b) = (circle(s, g), circle(s, g));
        (round_center(s, a).distance(round_center(s, b)) > 1.0)
            .then_some(ConstraintKind::Tangent { a, b, kind: TangentKind::External })
    };
    tangent_circles_internal: false, |s, g| {
        let (a, b) = (circle(s, g), circle(s, g));
        let ok = round_center(s, a).distance(round_center(s, b)) > 1.0
            && (round_radius(s, a) - round_radius(s, b)).abs() > 0.1;
        ok.then_some(ConstraintKind::Tangent { a, b, kind: TangentKind::Internal })
    };
    tangent_arc_circle_internal: false, |s, g| {
        let (a, b) = (arc(s, g), circle(s, g));
        let ok = round_center(s, a).distance(round_center(s, b)) > 1.0
            && (round_radius(s, a) - round_radius(s, b)).abs() > 0.1;
        ok.then_some(ConstraintKind::Tangent { a, b, kind: TangentKind::Internal })
    };
    tangent_arcs: false, |s, g| {
        let (a, b) = (arc(s, g), arc(s, g));
        let kind = if g.next() < 0.0 { TangentKind::External } else { TangentKind::Internal };
        let ok = round_center(s, a).distance(round_center(s, b)) > 1.0
            && (round_radius(s, a) - round_radius(s, b)).abs() > 0.1;
        ok.then_some(ConstraintKind::Tangent { a, b, kind })
    };
    tangent_at_endpoint_shared_line_arc: true, |s, g| {
        let l = line(s, g);
        let a = arc_from(s, g, end_of(s, l, End::End));
        Some(ConstraintKind::Tangent {
            a: l,
            b: a,
            kind: TangentKind::AtEndpoint { a_end: End::End, b_end: End::Start, reversed: false },
        })
    };
    tangent_at_endpoint_coincident_arc_line: true, |s, g| {
        let a = arc(s, g);
        let l = line(s, g);
        let (pa, pl) = (end_of(s, a, End::End), end_of(s, l, End::Start));
        s.add_constraint(ConstraintKind::Coincident { a: pa, b: pl }).unwrap();
        Some(ConstraintKind::Tangent {
            a,
            b: l,
            kind: TangentKind::AtEndpoint { a_end: End::End, b_end: End::Start, reversed: false },
        })
    };
    tangent_at_endpoint_shared_arcs: true, |s, g| {
        let a = arc(s, g);
        let b = arc_from(s, g, end_of(s, a, End::Start));
        Some(ConstraintKind::Tangent {
            a,
            b,
            kind: TangentKind::AtEndpoint { a_end: End::Start, b_end: End::Start, reversed: true },
        })
    };
    concentric: false, |s, g| {
        let (a, b) = (circle(s, g), arc(s, g));
        Some(ConstraintKind::Concentric { a, b })
    };
    equal_lines: false, |s, g| {
        let (a, b) = (line(s, g), line(s, g));
        Some(ConstraintKind::Equal { a, b })
    };
    equal_line_arc: false, |s, g| {
        let (a, b) = (line(s, g), arc(s, g));
        Some(ConstraintKind::Equal { a, b })
    };
    equal_arc_line: false, |s, g| {
        let (a, b) = (arc(s, g), line(s, g));
        Some(ConstraintKind::Equal { a, b })
    };
    equal_circle_arc: false, |s, g| {
        let (a, b) = (circle(s, g), arc(s, g));
        Some(ConstraintKind::Equal { a, b })
    };
    equal_arcs: false, |s, g| {
        let (a, b) = (arc(s, g), arc(s, g));
        Some(ConstraintKind::Equal { a, b })
    };
    symmetric_points: false, |s, g| {
        let (a, b, axis) = (point(s, g), point(s, g), line(s, g));
        Some(ConstraintKind::Symmetric { a, b, axis })
    };
    symmetric_lines: false, |s, g| {
        let (a, b, axis) = (line(s, g), line(s, g), line(s, g));
        Some(ConstraintKind::Symmetric { a, b, axis })
    };
    symmetric_circles: false, |s, g| {
        let (a, b, axis) = (circle(s, g), circle(s, g), line(s, g));
        Some(ConstraintKind::Symmetric { a, b, axis })
    };
    symmetric_arcs: false, |s, g| {
        let (a, b, axis) = (arc(s, g), arc(s, g), line(s, g));
        Some(ConstraintKind::Symmetric { a, b, axis })
    };
    distance_point_point: false, |s, g| {
        let (a, b) = (point(s, g), point(s, g));
        let ok = s.point(a)?.distance(s.point(b)?) > 1.0;
        let v = 50.0 * g.unit();
        ok.then_some(dim(DimensionKind::Distance { a, b, side: Side::Positive }, v))
    };
    distance_point_line: false, |s, g| {
        let (a, b) = (point(s, g), line(s, g));
        let side = side(g);
        Some(dim(DimensionKind::Distance { a, b, side }, 50.0 * g.unit()))
    };
    distance_line_point: false, |s, g| {
        let (a, b) = (line(s, g), point(s, g));
        let side = side(g);
        Some(dim(DimensionKind::Distance { a, b, side }, 50.0 * g.unit()))
    };
    distance_line_line: false, |s, g| {
        let (a, b) = (line(s, g), line(s, g));
        let side = side(g);
        Some(dim(DimensionKind::Distance { a, b, side }, 50.0 * g.unit()))
    };
    distance_point_circle: false, |s, g| {
        let (a, b) = (point(s, g), circle(s, g));
        let side = side(g);
        let ok = s.point(a)?.distance(round_center(s, b)) > 1.0;
        ok.then_some(dim(DimensionKind::Distance { a, b, side }, 50.0 * g.unit()))
    };
    distance_arc_point: false, |s, g| {
        let (a, b) = (arc(s, g), point(s, g));
        let side = side(g);
        let ok = s.point(b)?.distance(round_center(s, a)) > 1.0;
        ok.then_some(dim(DimensionKind::Distance { a, b, side }, 50.0 * g.unit()))
    };
    distance_line_circle: false, |s, g| {
        let (a, b) = (line(s, g), circle(s, g));
        let side = side(g);
        Some(dim(DimensionKind::Distance { a, b, side }, 50.0 * g.unit()))
    };
    distance_arc_line: false, |s, g| {
        let (a, b) = (arc(s, g), line(s, g));
        let side = side(g);
        Some(dim(DimensionKind::Distance { a, b, side }, 50.0 * g.unit()))
    };
    horizontal_distance: false, |s, g| {
        let (a, b) = (point(s, g), point(s, g));
        let side = side(g);
        Some(dim(DimensionKind::HorizontalDistance { a, b, side }, 50.0 * g.unit()))
    };
    vertical_distance: false, |s, g| {
        let (a, b) = (point(s, g), point(s, g));
        let side = side(g);
        Some(dim(DimensionKind::VerticalDistance { a, b, side }, 50.0 * g.unit()))
    };
    length: false, |s, g| {
        let line = line(s, g);
        Some(dim(DimensionKind::Length { line }, 50.0 * g.unit()))
    };
    angle: true, |s, g| {
        let (a, b) = (line(s, g), line(s, g));
        let branch = AngleBranch { reverse_a: g.next() < 0.0, reverse_b: g.next() < 0.0 };
        Some(dim(DimensionKind::Angle { a, b, branch }, PI * g.next()))
    };
    radius_circle: false, |s, g| {
        let curve = circle(s, g);
        Some(dim(DimensionKind::Radius { curve }, g.radius()))
    };
    radius_arc: false, |s, g| {
        let curve = arc(s, g);
        Some(dim(DimensionKind::Radius { curve }, g.radius()))
    };
    diameter_circle: false, |s, g| {
        let curve = circle(s, g);
        Some(dim(DimensionKind::Diameter { curve }, g.radius()))
    };
    diameter_arc: false, |s, g| {
        let curve = arc(s, g);
        Some(dim(DimensionKind::Diameter { curve }, g.radius()))
    };
    arc_length: false, |s, g| {
        let arc = arc(s, g);
        Some(dim(DimensionKind::ArcLength { arc }, g.len()))
    };
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(1000))]

    #[test]
    fn arc_internal_equation(seeds in prop::collection::vec(-1.0f64..1.0, 16)) {
        let mut s = Sketch::new(Plane::XY);
        let mut g = Gen { seeds: &seeds, i: 0 };
        let a = arc(&mut s, &mut g);
        // Move the end off the circle so the residual is non-zero.
        let end = end_of(&s, a, End::End);
        if let EntityKind::Point { x, .. } = s.entity(end).unwrap().kind {
            let v = s.params().get(x) + 3.0 * g.next();
            s.params_mut().set(x, v);
        }
        let r = entity_residual(&s, a).unwrap();
        prop_assert_eq!(r.rows(), 1);
        fd_check(&r, s.params().as_slice()).map_err(TestCaseError::fail)?;
    }

    /// Captured orientation makes the residual zero when the value matches the geometry.
    #[test]
    fn oriented_dimensions_start_at_zero(seeds in prop::collection::vec(-1.0f64..1.0, 32)) {
        let mut s = Sketch::new(Plane::XY);
        let mut g = Gen { seeds: &seeds, i: 0 };
        let (p, l1, l2, c) = (point(&mut s, &mut g), line(&mut s, &mut g), line(&mut s, &mut g),
                              circle(&mut s, &mut g));
        let seg = |s: &Sketch, l| match s.curve(l).unwrap() {
            amg_sketch::Curve2::Line(seg) => seg,
            _ => unreachable!(),
        };
        let sd = |s: &Sketch, l, q: Point2| {
            let sg = seg(s, l);
            let d = sg.b - sg.a;
            d.perp_dot(q - sg.a) / d.length()
        };
        let pp = s.point(p).unwrap();
        let (cc, r) = (round_center(&s, c), round_radius(&s, c));
        let s1 = seg(&s, l1);
        let s2 = seg(&s, l2);
        let theta = (s1.b - s1.a).angle_to(s2.b - s2.a);
        let alpha = theta.abs();
        let mut cases = vec![
            dim(DimensionKind::Distance { a: p, b: l1, side: Side::Positive }, sd(&s, l1, pp).abs()),
            dim(DimensionKind::Distance { a: l2, b: l1, side: Side::Negative },
                sd(&s, l2, (s1.a + s1.b) * 0.5).abs()),
            dim(DimensionKind::Distance { a: c, b: p, side: Side::Negative },
                (pp.distance(cc) - r).abs()),
            dim(DimensionKind::Angle { a: l1, b: l2, branch: AngleBranch { reverse_a: false, reverse_b: false } }, alpha),
            dim(DimensionKind::Angle { a: l1, b: l2, branch: AngleBranch { reverse_a: true, reverse_b: false } }, PI - alpha),
            dim(DimensionKind::Angle { a: l2, b: l1, branch: AngleBranch { reverse_a: false, reverse_b: true } }, -alpha),
        ];
        let lc_gap = sd(&s, l1, cc).abs() - r;
        if lc_gap > 0.0 {
            cases.push(dim(DimensionKind::Distance { a: c, b: l1, side: Side::Positive }, lc_gap));
        }
        // Horizontal/vertical distances between the two line starts.
        let (a1, a2) = (end_of(&s, l1, End::Start), end_of(&s, l2, End::Start));
        cases.push(dim(DimensionKind::HorizontalDistance { a: a1, b: a2, side: Side::Positive },
                       (s2.a.x - s1.a.x).abs()));
        cases.push(dim(DimensionKind::VerticalDistance { a: a1, b: a2, side: Side::Negative },
                       (s2.a.y - s1.a.y).abs()));
        for kind in cases {
            let id = s.add_constraint_oriented(kind.clone()).unwrap();
            let stored = s.constraint(id).unwrap().kind.clone();
            let res = constraint_residual(&s, &stored, 100.0).unwrap().unwrap();
            let mut f = vec![0.0; res.rows()];
            res.eval(s.params().as_slice(), &mut f);
            prop_assert!(f.iter().all(|v| v.abs() < 1e-9), "{:?} → {:?}: {:?}", kind, stored, f);
        }
    }
}

// ---- Orientation capture: tangency ---------------------------------------------------------

fn residual_of(s: &Sketch, kind: &ConstraintKind) -> Vec<f64> {
    let r = constraint_residual(s, kind, 10.0).unwrap().unwrap();
    let mut f = vec![0.0; r.rows()];
    r.eval(s.params().as_slice(), &mut f);
    f
}

fn assert_zero(f: &[f64]) {
    assert!(f.iter().all(|v| v.abs() < 1e-9), "{f:?}");
}

fn add_oriented(s: &mut Sketch, kind: ConstraintKind) -> ConstraintKind {
    let id = s.add_constraint_oriented(kind).unwrap();
    s.constraint(id).unwrap().kind.clone()
}

#[test]
fn line_circle_tangency_side_is_captured() {
    let mut s = Sketch::new(Plane::XY);
    let l = s
        .add_line(Point2::new(0.0, 0.0), Point2::new(10.0, 0.0))
        .unwrap();
    let left = s.add_circle(Point2::new(3.0, 2.0), 2.0).unwrap();
    let right = s.add_circle(Point2::new(3.0, -5.0), 5.0).unwrap();
    for (c, expect) in [
        (left, TangentKind::External),
        (right, TangentKind::Internal),
    ] {
        for given in [TangentKind::External, TangentKind::Internal] {
            for (a, b) in [(l, c), (c, l)] {
                let k = add_oriented(&mut s, ConstraintKind::Tangent { a, b, kind: given });
                assert_eq!(k, ConstraintKind::Tangent { a, b, kind: expect });
                assert_zero(&residual_of(&s, &k));
            }
        }
    }
}

#[test]
fn circle_circle_tangency_kind_is_captured() {
    let mut s = Sketch::new(Plane::XY);
    let big = s.add_circle(Point2::ZERO, 10.0).unwrap();
    let outside = s.add_circle(Point2::new(13.0, 0.0), 3.0).unwrap();
    let inside = s
        .add_arc(
            Point2::new(0.0, 7.0),
            Point2::new(0.0, 10.0),
            Point2::new(-3.0, 7.0),
        )
        .unwrap();
    for (c, expect) in [
        (outside, TangentKind::External),
        (inside, TangentKind::Internal),
    ] {
        for given in [TangentKind::External, TangentKind::Internal] {
            let k = add_oriented(
                &mut s,
                ConstraintKind::Tangent {
                    a: big,
                    b: c,
                    kind: given,
                },
            );
            assert_eq!(
                k,
                ConstraintKind::Tangent {
                    a: big,
                    b: c,
                    kind: expect
                }
            );
            assert_zero(&residual_of(&s, &k));
        }
    }
}

#[test]
fn endpoint_tangency_reversal_is_captured() {
    let mut s = Sketch::new(Plane::XY);
    // Line along +x ending at (10, 0); arc about (10, 5) starting there runs CCW = +x too.
    let l = s.add_line(Point2::ZERO, Point2::new(10.0, 0.0)).unwrap();
    let joint = end_of(&s, l, End::End);
    let c = s.add_point(Point2::new(10.0, 5.0)).unwrap();
    let e = s.add_point(Point2::new(15.0, 5.0)).unwrap();
    let a = s
        .add_entity(
            EntityKind::Arc {
                center: c,
                start: joint,
                end: e,
            },
            false,
            EntityOrigin::User,
        )
        .unwrap();
    let k = |reversed| ConstraintKind::Tangent {
        a: l,
        b: a,
        kind: TangentKind::AtEndpoint {
            a_end: End::End,
            b_end: End::Start,
            reversed,
        },
    };
    assert_eq!(add_oriented(&mut s, k(true)), k(false));
    assert_zero(&residual_of(&s, &k(false)));
    // The reversed branch is not a root (chirality): residual is L_c·π.
    assert!((residual_of(&s, &k(true))[0].abs() - 10.0 * PI).abs() < 1e-9);

    // An arc about (10, −5) starting at the joint runs CCW = −x: reversed.
    let c2 = s.add_point(Point2::new(10.0, -5.0)).unwrap();
    let e2 = s.add_point(Point2::new(5.0, -5.0)).unwrap();
    let a2 = s
        .add_entity(
            EntityKind::Arc {
                center: c2,
                start: joint,
                end: e2,
            },
            false,
            EntityOrigin::User,
        )
        .unwrap();
    let k2 = |reversed| ConstraintKind::Tangent {
        a: l,
        b: a2,
        kind: TangentKind::AtEndpoint {
            a_end: End::End,
            b_end: End::Start,
            reversed,
        },
    };
    assert_eq!(add_oriented(&mut s, k2(false)), k2(true));
    assert_zero(&residual_of(&s, &k2(true)));
}

#[test]
fn angle_branch_and_order_are_captured() {
    let mut s = Sketch::new(Plane::XY);
    let a = s.add_line(Point2::ZERO, Point2::new(10.0, 0.0)).unwrap();
    // b points at −30°: measured angle(a, b) = −30°.
    let b = s
        .add_line(
            Point2::ZERO,
            Point2::new(30f64.to_radians().cos(), -(30f64.to_radians().sin())),
        )
        .unwrap();
    let none = AngleBranch {
        reverse_a: false,
        reverse_b: false,
    };
    for v in [30.0, -30.0, 150.0, -150.0] {
        let v = f64::to_radians(v);
        let k = add_oriented(&mut s, dim(DimensionKind::Angle { a, b, branch: none }, v));
        assert_zero(&residual_of(&s, &k));
    }
    // −30° matches without swapping or reversing.
    let k = add_oriented(
        &mut s,
        dim(
            DimensionKind::Angle { a, b, branch: none },
            (-30f64).to_radians(),
        ),
    );
    assert_eq!(
        k,
        dim(
            DimensionKind::Angle { a, b, branch: none },
            (-30f64).to_radians()
        )
    );
}

// ---- Conventions and assembly -----------------------------------------------------------

#[test]
fn sign_conventions() {
    let mut s = Sketch::new(Plane::XY);
    let l = s.add_line(Point2::ZERO, Point2::new(4.0, 0.0)).unwrap();
    let p = s.add_point(Point2::new(1.0, 3.0)).unwrap();
    // Point on line: signed distance, positive on the left.
    assert_eq!(
        residual_of(&s, &ConstraintKind::PointOnCurve { point: p, curve: l }),
        [3.0]
    );
    // Horizontal line from (0,0) to (4,0); vertical: a_x − b_x = −4.
    assert_eq!(
        residual_of(&s, &ConstraintKind::Vertical(HvTarget::Line(l))),
        [-4.0]
    );
    // Midpoint 2p − a − b.
    assert_eq!(
        residual_of(&s, &ConstraintKind::Midpoint { point: p, line: l }),
        [-2.0, 6.0]
    );
    // Distance with side Negative on a left point: −3 − v.
    assert_eq!(
        residual_of(
            &s,
            &dim(
                DimensionKind::Distance {
                    a: p,
                    b: l,
                    side: Side::Negative
                },
                1.0
            )
        ),
        [-4.0]
    );
    // Perpendicular line: parallel residual = L_c · sin(90°) = 10.
    let v = s.add_line(Point2::ZERO, Point2::new(0.0, 2.0)).unwrap();
    let par = residual_of(&s, &ConstraintKind::Parallel { a: l, b: v });
    assert!((par[0] - 10.0).abs() < 1e-12);
    let ang = residual_of(
        &s,
        &dim(
            DimensionKind::Angle {
                a: l,
                b: v,
                branch: AngleBranch {
                    reverse_a: false,
                    reverse_b: false,
                },
            },
            0.0,
        ),
    );
    assert!((ang[0] - 10.0 * PI / 2.0).abs() < 1e-12);
    // Arc length of a quarter arc of radius 2: π.
    let a = s
        .add_arc(Point2::ZERO, Point2::new(2.0, 0.0), Point2::new(0.0, 2.0))
        .unwrap();
    let al = residual_of(&s, &dim(DimensionKind::ArcLength { arc: a }, 0.0));
    assert!((al[0] - PI).abs() < 1e-12);
    // Fix and driven dimensions add no rows.
    assert!(
        constraint_residual(&s, &ConstraintKind::Fix { entity: a }, 1.0)
            .unwrap()
            .is_none()
    );
    let mut driven = dim(DimensionKind::Length { line: l }, 1.0);
    if let ConstraintKind::Dimension(d) = &mut driven {
        d.driving = false;
    }
    assert!(constraint_residual(&s, &driven, 1.0).unwrap().is_none());
}

#[test]
fn wrong_kinds_are_errors() {
    let mut s = Sketch::new(Plane::XY);
    let p = s.add_point(Point2::ZERO).unwrap();
    let q = s.add_point(Point2::X).unwrap();
    let err = constraint_residual(&s, &ConstraintKind::Parallel { a: p, b: q }, 1.0);
    assert!(matches!(
        err,
        Err(amg_sketch::SketchError::WrongEntityKind { .. })
    ));
    let err = constraint_residual(
        &s,
        &ConstraintKind::Coincident {
            a: p,
            b: EntityId(99),
        },
        1.0,
    );
    assert!(matches!(
        err,
        Err(amg_sketch::SketchError::UnknownEntity(_))
    ));
}

#[test]
fn system_assembly_and_dof() {
    use amg_sketch::solve::dof;
    let mut s = Sketch::new(Plane::XY);
    // Rectangle with H/V only: DOF 4.
    let pts: Vec<_> = [(0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (0.0, 5.0)]
        .iter()
        .map(|&(x, y)| s.add_point(Point2::new(x, y)).unwrap())
        .collect();
    let lines: Vec<_> = (0..4)
        .map(|i| s.add_line_between(pts[i], pts[(i + 1) % 4]).unwrap())
        .collect();
    assert_eq!(dof(&s).unwrap(), 8);
    for (i, &l) in lines.iter().enumerate() {
        let t = HvTarget::Line(l);
        s.add_constraint(if i % 2 == 0 {
            ConstraintKind::Horizontal(t)
        } else {
            ConstraintKind::Vertical(t)
        })
        .unwrap();
    }
    assert_eq!(dof(&s).unwrap(), 4);
    // Two dimensions + fix a corner: DOF 0.
    s.add_constraint(dim(DimensionKind::Length { line: lines[0] }, 10.0))
        .unwrap();
    s.add_constraint(dim(DimensionKind::Length { line: lines[1] }, 5.0))
        .unwrap();
    s.add_constraint(ConstraintKind::Fix { entity: pts[0] })
        .unwrap();
    assert_eq!(dof(&s).unwrap(), 0);
    // An arc: 6 params, 1 internal equation → 5.
    let arc = s
        .add_arc(
            Point2::new(30.0, 0.0),
            Point2::new(32.0, 0.0),
            Point2::new(30.0, 2.0),
        )
        .unwrap();
    assert_eq!(dof(&s).unwrap(), 5);

    let sys = build_system(&s).unwrap();
    assert_eq!(sys.system.owner(0), EquationOwner::Entity(arc));
    assert_eq!(sys.system.rows(), 1 + 4 + 2);
    assert_eq!(sys.fixed, fixed_params(&s));
    assert_eq!(sys.fixed.len(), 2);
    assert!(sys.system.max_abs_residual(s.params().as_slice()) < 1e-12);
    assert!(sys.length_scale > 30.0);
}
