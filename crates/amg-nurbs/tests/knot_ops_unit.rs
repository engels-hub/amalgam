//! KH-12 unit tests: error variants of the knot operations, `continuity_at` classes, join,
//! segment, Bézier decomposition counts and the surface split / strips structure.
#![allow(clippy::unwrap_used, clippy::float_cmp, reason = "test code")]

use amg_core::{Plane, Tolerance};
use amg_nurbs::{
    Continuity, KnotVector, MAX_DEGREE, NurbsCurve2, NurbsCurve3, NurbsError, Point2, Point3, UV,
    analytic,
};

fn cubic() -> NurbsCurve3 {
    let poles = (0..7)
        .map(|i| {
            let t = i as f64;
            Point3::new(t, (0.9 * t).sin() * 3.0, (0.4 * t).cos())
        })
        .collect();
    NurbsCurve3::new(3, KnotVector::clamped_uniform(3, 7).unwrap(), poles, None).unwrap()
}

#[test]
fn error_table() {
    let c = cubic();
    assert!(matches!(
        c.insert_knot(f64::NAN, 1),
        Err(NurbsError::OutOfDomain(_))
    ));
    assert!(matches!(
        c.insert_knot(1.5, 1),
        Err(NurbsError::OutOfDomain(_))
    ));
    assert!(matches!(
        c.insert_knot(-0.1, 1),
        Err(NurbsError::OutOfDomain(_))
    ));
    assert!(matches!(
        c.refine_knots(&[0.5, 2.0]),
        Err(NurbsError::OutOfDomain(_))
    ));
    // 0.25 already has multiplicity 1; three more would exceed the degree.
    assert!(matches!(
        c.refine_knots(&[0.25, 0.25, 0.25]),
        Err(NurbsError::InvalidKnots(_))
    ));
    assert!(matches!(
        c.elevate_degree(MAX_DEGREE),
        Err(NurbsError::Degree(_))
    ));
    assert!(matches!(c.split(0.0), Err(NurbsError::OutOfDomain(_))));
    assert!(matches!(c.split(1.0), Err(NurbsError::OutOfDomain(_))));
    assert!(matches!(
        c.split(f64::INFINITY),
        Err(NurbsError::OutOfDomain(_))
    ));
    assert!(matches!(
        c.segment(0.6, 0.4),
        Err(NurbsError::OutOfDomain(_))
    ));
    assert!(matches!(
        c.segment(-1.0, 0.4),
        Err(NurbsError::OutOfDomain(_))
    ));
    assert!(matches!(
        c.segment(0.2, 3.0),
        Err(NurbsError::OutOfDomain(_))
    ));
    let far = c.transformed(&amg_core::Transform::from_translation(
        amg_core::Vec3::X * 100.0,
    ));
    assert!(matches!(
        c.join(&far, &Tolerance::DEFAULT),
        Err(NurbsError::Incompatible(_))
    ));
    // No-ops.
    assert_eq!(c.insert_knot(0.0, 3).unwrap(), c);
    assert_eq!(c.insert_knot(0.5, 0).unwrap(), c);
    assert_eq!(c.refine_knots(&[]).unwrap(), c);
    assert_eq!(c.refine_knots(&[0.0, 1.0]).unwrap(), c);
    assert_eq!(c.elevate_degree(0).unwrap(), c);
    assert_eq!(c.remove_knot(0.33, 1, 1.0).1, 0); // not a knot
    assert_eq!(c.remove_knot(0.0, 1, 1.0).1, 0); // domain end
    assert_eq!(c.remove_knot(0.25, 1, f64::NAN).1, 0);
    assert_eq!(c.segment(0.0, 1.0).unwrap(), c);
}

#[test]
fn insertion_caps_multiplicity_and_snaps() {
    let c = cubic();
    let k = c.insert_knot(0.5, 10).unwrap();
    assert_eq!(k.knots().multiplicity(0.5, 0.0), 3);
    // A parameter within 1e-12 of a knot snaps onto it.
    let s = c.insert_knot(0.25 + 1e-14, 1).unwrap();
    assert_eq!(s.knots().multiplicity(0.25, 0.0), 2);
    let r = c.refine_knots(&[0.75 - 1e-15, 0.1]).unwrap();
    assert_eq!(r.knots().multiplicity(0.75, 0.0), 2);
    assert_eq!(r.poles().len(), c.poles().len() + 2);
}

#[test]
fn split_join_bit_identical_and_segment() {
    let c = cubic();
    let (l, r) = c.split(0.4).unwrap();
    assert_eq!(l.domain(), (0.0, 0.4));
    assert_eq!(r.domain(), (0.4, 1.0));
    assert_eq!(
        l.end().to_array().map(f64::to_bits),
        r.start().to_array().map(f64::to_bits)
    );
    let j = l.join(&r, &Tolerance::DEFAULT).unwrap();
    assert_eq!(j, c.insert_knot(0.4, 3).unwrap());
    // Split at an existing knot of full multiplicity.
    let full = c.insert_knot(0.5, 3).unwrap();
    let (l2, r2) = full.split(0.5).unwrap();
    assert_eq!(l2.poles().len() + r2.poles().len(), full.poles().len() + 1);
    // Joining a shifted second curve keeps its parameter length.
    let shifted = r.reparametrized((7.0, 7.6));
    let j2 = l.join(&shifted, &Tolerance::DEFAULT).unwrap();
    assert_eq!(j2.domain().0, 0.0);
    assert!((j2.domain().1 - 1.0).abs() < 1e-15);
    // Degree mismatch: the lower one is elevated.
    let line = analytic::line(c.end(), c.end() + amg_core::Vec3::Z);
    let j3 = c.join(&line, &Tolerance::DEFAULT).unwrap();
    assert_eq!(j3.degree(), 3);
    assert!((j3.eval(1.5) - line.eval(0.5)).length() < 1e-14);
    assert!((j3.eval(0.3) - c.eval(0.3)).length() < 1e-14);
    let seg = c.segment(0.1, 0.9).unwrap();
    assert_eq!(seg.domain(), (0.1, 0.9));
    assert!((seg.eval(0.55) - c.eval(0.55)).length() < 1e-14);
}

#[test]
fn rational_join_scales_weights() {
    let plane = Plane::XY;
    let a = analytic::circle_arc(&plane, 2.0, 0.0, 1.0).unwrap();
    let b = analytic::circle_arc(&plane, 2.0, 1.0, 1.0).unwrap();
    // Scale b's weights by a common factor: same curve, different joint weight.
    let w: Vec<f64> = b.weights().unwrap().iter().map(|w| w * 3.0).collect();
    let b3 = NurbsCurve3::new(b.degree(), b.knots().clone(), b.poles().to_vec(), Some(w)).unwrap();
    let j = a.join(&b3, &Tolerance::DEFAULT).unwrap();
    assert_eq!(j.domain(), (0.0, 2.0));
    for i in 0..=100 {
        let t = 2.0 * i as f64 / 100.0;
        assert!((j.eval(t).length() - 2.0).abs() < 1e-14, "t = {t}");
    }
    assert!(j.continuity_at(1.0, &Tolerance::DEFAULT) >= Continuity::G1);
}

#[test]
fn bezier_segments_of_a_full_circle() {
    let c = analytic::circle_arc(&Plane::XY, 1.0, 0.0, std::f64::consts::TAU).unwrap();
    let segs = c.bezier_segments();
    assert_eq!(segs.len(), 4);
    for s in &segs {
        assert_eq!(s.degree(), 2);
        assert_eq!(s.poles().len(), 3);
        let (a, b) = s.domain();
        for i in 0..=10 {
            let t = a + (b - a) * i as f64 / 10.0;
            assert!((s.eval(t) - c.eval(t)).length() < 1e-15);
        }
    }
}

#[test]
fn continuity_classes() {
    let tol = Tolerance::DEFAULT;
    let c = cubic();
    // Interior simple knot of a cubic: C2 everywhere, Cn off the knots, C0 at the ends.
    assert_eq!(c.continuity_at(0.25, &tol), Continuity::C2);
    assert_eq!(c.continuity_at(0.3, &tol), Continuity::Cn);
    assert_eq!(c.continuity_at(0.0, &tol), Continuity::C0);
    assert_eq!(c.continuity_at(1.0, &tol), Continuity::C0);
    // Inserting knots does not change the continuity of the shape.
    let k2 = c.insert_knot(0.5, 2).unwrap();
    assert_eq!(k2.continuity_at(0.5, &tol), Continuity::C2);
    let k3 = c.insert_knot(0.5, 3).unwrap();
    assert_eq!(k3.continuity_at(0.5, &tol), Continuity::C2);
    // A new knot inside a span is removable: the shape stays smooth there.
    let k4 = c.insert_knot(0.6, 3).unwrap();
    assert_eq!(k4.continuity_at(0.6, &tol), Continuity::Cn);
    // Elevation keeps the shape, so the knot stays C2 (multiplicity 3 in the quintic).
    let q = c.elevate_degree(2).unwrap();
    assert_eq!(q.continuity_at(0.25, &tol), Continuity::C2);
    // A simple knot of a quintic leaves C4: Cn from the multiplicity alone.
    let q1 = q.refine_knots(&[0.6]).unwrap();
    assert_eq!(q1.continuity_at(0.6, &tol), Continuity::Cn);

    // Kink: C0.
    let poly = |pts: &[(f64, f64)], knots: Vec<f64>, p: usize| {
        NurbsCurve2::new(
            p,
            KnotVector::new(knots).unwrap(),
            pts.iter().map(|&(x, y)| Point2::new(x, y)).collect(),
            None,
        )
        .unwrap()
    };
    let kink = poly(&[(0., 0.), (1., 0.), (1., 1.)], vec![0., 0., 1., 2., 2.], 1);
    assert_eq!(kink.continuity_at(1.0, &tol), Continuity::C0);
    // Collinear but different speed: G1.
    let g1 = poly(&[(0., 0.), (1., 0.), (3., 0.)], vec![0., 0., 1., 2., 2.], 1);
    assert_eq!(g1.continuity_at(1.0, &tol), Continuity::G1);
    // Equal speed straight line: C1 and beyond (degree 1: second derivatives are 0).
    let c1 = poly(&[(0., 0.), (1., 0.), (2., 0.)], vec![0., 0., 1., 2., 2.], 1);
    assert_eq!(c1.continuity_at(1.0, &tol), Continuity::Cn);
    // Two quadratic arcs of one circle at a full-multiplicity joint: G2 (same curvature,
    // the rational parametrisation is not C1 across a 90° joint of the standard form unless
    // the weights match, which they do here: the joint is at least G1).
    let circ = analytic::circle_arc(&Plane::XY, 1.0, 0.0, std::f64::consts::PI).unwrap();
    let (ks, _) = circ.knots().to_knots_mults(0.0);
    assert!(circ.continuity_at(ks[1], &tol) >= Continuity::G2);
    // Quadratic pair with a C1 but not G2 joint (asymmetric, so the curvatures differ).
    let qc = poly(
        &[(0., 0.), (1., 1.), (2., 1.), (3., 3.)],
        vec![0., 0., 0., 1., 2., 2., 2.],
        2,
    );
    assert_eq!(qc.continuity_at(1.0, &tol), Continuity::C1);
    // Same joint after reparametrising one side: tangent direction kept, speed changed.
    let (l, r) = qc.split(1.0).unwrap();
    let r2 = r.reparametrized((1.0, 1.5));
    let joined = l.join(&r2, &tol).unwrap();
    assert_eq!(joined.continuity_at(1.0, &tol), Continuity::G1);
}

#[test]
fn surface_ops_structure() {
    let s = analytic::sphere(Point3::ZERO, 3.0).unwrap();
    let [(u0, u1), (v0, v1)] = s.domain();
    let um = 0.37 * (u0 + u1);
    let (a, b) = s.split(UV::U, um).unwrap();
    assert_eq!(a.domain()[0], (u0, um));
    assert_eq!(b.domain()[0], (um, u1));
    assert_eq!(a.domain()[1], (v0, v1));
    for (x, y) in a
        .boundary(amg_nurbs::Side::UMax)
        .poles()
        .iter()
        .zip(b.boundary(amg_nurbs::Side::UMin).poles())
    {
        assert_eq!(
            x.to_array().map(f64::to_bits),
            y.to_array().map(f64::to_bits)
        );
    }
    let strips = s.bezier_strips(UV::V);
    let (kv, _) = s.knots(UV::V).to_knots_mults(0.0);
    assert_eq!(strips.len(), kv.len() - 1);
    let patches: Vec<_> = strips
        .iter()
        .flat_map(|st| st.bezier_strips(UV::U))
        .collect();
    let (ku, _) = s.knots(UV::U).to_knots_mults(0.0);
    assert_eq!(patches.len(), (ku.len() - 1) * (kv.len() - 1));
    for pch in &patches {
        assert_eq!(pch.poles().nu(), 3);
        assert_eq!(pch.poles().nv(), 3);
        let [(pu0, pu1), (pv0, pv1)] = pch.domain();
        let (u, v) = (0.5 * (pu0 + pu1), 0.5 * (pv0 + pv1));
        assert!((pch.eval(u, v).length() - 3.0).abs() < 1e-13);
    }
    let e = s.elevate_degree(UV::V, 2).unwrap();
    assert_eq!(e.degree(UV::V), 4);
    assert_eq!(e.degree(UV::U), s.degree(UV::U));
    assert!(matches!(
        s.elevate_degree(UV::U, MAX_DEGREE),
        Err(NurbsError::Degree(_))
    ));
    assert!(matches!(
        s.split(UV::V, v0),
        Err(NurbsError::OutOfDomain(_))
    ));
    assert!(matches!(
        s.insert_knot(UV::U, u1 + 1.0, 1),
        Err(NurbsError::OutOfDomain(_))
    ));
    assert!(matches!(
        s.refine_knots(UV::V, &[f64::NAN]),
        Err(NurbsError::OutOfDomain(_))
    ));
    assert_eq!(s.refine_knots(UV::U, &[]).unwrap(), s);
}
