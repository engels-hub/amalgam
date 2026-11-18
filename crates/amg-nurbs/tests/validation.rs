//! KH-07 Done-when 5: constructors reject every invalid input with the right `NurbsError`
//! (table test); no panic in a proptest over random inputs (constructors, evaluation and the
//! KH-07 operations on whatever the constructors accept).
#![allow(clippy::unwrap_used, clippy::float_cmp, reason = "test code")]

use std::mem::discriminant;

use amg_core::{Diagnostic, Point3, Tolerance, Transform};
use amg_nurbs::{Grid, KnotVector, NurbsCurve3, NurbsError, NurbsSurface3, Side, UV};
use proptest::prelude::*;

fn kv(v: &[f64]) -> KnotVector {
    KnotVector::new(v.to_vec()).unwrap()
}

fn pts(n: usize) -> Vec<Point3> {
    (0..n)
        .map(|i| Point3::new(i as f64, (i * i) as f64, 0.0))
        .collect()
}

fn same(a: &NurbsError, b: &NurbsError) -> bool {
    discriminant(a) == discriminant(b)
}

#[test]
fn knot_vector_table() {
    let bad = NurbsError::InvalidKnots("");
    let cases: Vec<(&str, Result<KnotVector, NurbsError>)> = vec![
        ("empty", KnotVector::new(vec![])),
        ("single", KnotVector::new(vec![1.0])),
        ("nan", KnotVector::new(vec![0.0, f64::NAN, 1.0])),
        ("inf", KnotVector::new(vec![0.0, f64::INFINITY])),
        ("decreasing", KnotVector::new(vec![0.0, 1.0, 0.5])),
        ("empty range", KnotVector::new(vec![1.0, 1.0, 1.0])),
        ("mults len", KnotVector::from_knots_mults(&[0.0, 1.0], &[2])),
        (
            "zero mult",
            KnotVector::from_knots_mults(&[0.0, 1.0], &[2, 0]),
        ),
        (
            "not increasing",
            KnotVector::from_knots_mults(&[0.0, 0.0], &[2, 2]),
        ),
        (
            "huge mult",
            KnotVector::from_knots_mults(&[0.0, 1.0], &[usize::MAX, 1]),
        ),
    ];
    for (name, r) in cases {
        let e = r.expect_err(name);
        assert!(same(&e, &bad), "{name}: {e:?}");
        assert!(e.hint().is_some());
    }
    assert!(matches!(
        KnotVector::clamped_uniform(0, 4),
        Err(NurbsError::Degree(0))
    ));
    assert!(matches!(
        KnotVector::clamped_uniform(26, 40),
        Err(NurbsError::Degree(26))
    ));
    assert!(matches!(
        KnotVector::clamped_uniform(3, 3),
        Err(NurbsError::CountMismatch {
            degree: 3,
            poles: 3,
            ..
        })
    ));
}

#[test]
fn curve_table() {
    let k3 = kv(&[0., 0., 0., 0., 1., 1., 1., 1.]);
    let cases: Vec<(&str, Result<NurbsCurve3, NurbsError>, NurbsError)> = vec![
        (
            "degree 0",
            NurbsCurve3::new(0, kv(&[0., 1., 2.]), pts(2), None),
            NurbsError::Degree(0),
        ),
        (
            "degree 26",
            NurbsCurve3::new(26, KnotVector::bezier(26), pts(27), None),
            NurbsError::Degree(26),
        ),
        (
            "too few knots",
            NurbsCurve3::new(3, k3.clone(), pts(5), None),
            NurbsError::CountMismatch {
                degree: 0,
                poles: 0,
                knots: 0,
            },
        ),
        (
            "too few poles",
            NurbsCurve3::new(3, kv(&[0., 0., 0., 0., 1., 1., 1.]), pts(3), None),
            NurbsError::CountMismatch {
                degree: 0,
                poles: 0,
                knots: 0,
            },
        ),
        (
            "unclamped",
            NurbsCurve3::new(2, kv(&[0., 1., 2., 3., 4., 5.]), pts(3), None),
            NurbsError::InvalidKnots(""),
        ),
        (
            "end multiplicity",
            NurbsCurve3::new(1, kv(&[0., 0., 0., 1.]), pts(2), None),
            NurbsError::InvalidKnots(""),
        ),
        (
            "interior multiplicity",
            NurbsCurve3::new(
                2,
                kv(&[0., 0., 0., 0.5, 0.5, 0.5, 1., 1., 1.]),
                pts(6),
                None,
            ),
            NurbsError::InvalidKnots(""),
        ),
        (
            "nan pole",
            NurbsCurve3::new(3, k3.clone(), vec![Point3::NAN; 4], None),
            NurbsError::Degenerate(""),
        ),
        (
            "weights length",
            NurbsCurve3::new(3, k3.clone(), pts(4), Some(vec![1.0; 3])),
            NurbsError::Degenerate(""),
        ),
        (
            "zero weight",
            NurbsCurve3::new(3, k3.clone(), pts(4), Some(vec![1.0, 0.0, 1.0, 1.0])),
            NurbsError::Weight { index: 1 },
        ),
        (
            "negative weight",
            NurbsCurve3::new(3, k3.clone(), pts(4), Some(vec![1.0, 1.0, 1.0, -2.0])),
            NurbsError::Weight { index: 3 },
        ),
        (
            "nan weight",
            NurbsCurve3::new(3, k3.clone(), pts(4), Some(vec![f64::NAN, 1.0, 1.0, 1.0])),
            NurbsError::Weight { index: 0 },
        ),
    ];
    for (name, r, want) in cases {
        let e = r.expect_err(name);
        assert!(same(&e, &want), "{name}: got {e:?}, want {want:?}");
        if let (NurbsError::Weight { index: a }, NurbsError::Weight { index: b }) = (&e, &want) {
            assert_eq!(a, b, "{name}");
        }
        assert!(!e.to_string().is_empty());
    }
    assert!(NurbsCurve3::new(3, k3, pts(4), Some(vec![1.0, 2.0, 1.0, 1.0])).is_ok());
}

#[test]
fn surface_and_grid_table() {
    let k1 = kv(&[0., 0., 1., 1.]);
    let g = Grid::new(2, 2, pts(4)).unwrap();
    assert!(matches!(
        Grid::new(2, 2, pts(3)),
        Err(NurbsError::Degenerate(_))
    ));
    assert!(matches!(
        Grid::new(0, 0, pts(0)),
        Err(NurbsError::Degenerate(_))
    ));
    let cases: Vec<(&str, Result<NurbsSurface3, NurbsError>, NurbsError)> = vec![
        (
            "u degree",
            NurbsSurface3::new([0, 1], [kv(&[0., 1., 2.]), k1.clone()], g.clone(), None),
            NurbsError::Degree(0),
        ),
        (
            "v count",
            NurbsSurface3::new(
                [1, 1],
                [k1.clone(), kv(&[0., 0., 0.5, 1., 1.])],
                g.clone(),
                None,
            ),
            NurbsError::CountMismatch {
                degree: 0,
                poles: 0,
                knots: 0,
            },
        ),
        (
            "nan pole",
            NurbsSurface3::new(
                [1, 1],
                [k1.clone(), k1.clone()],
                Grid::new(2, 2, vec![Point3::INFINITY; 4]).unwrap(),
                None,
            ),
            NurbsError::Degenerate(""),
        ),
        (
            "weight grid size",
            NurbsSurface3::new(
                [1, 1],
                [k1.clone(), k1.clone()],
                g.clone(),
                Some(Grid::new(1, 2, vec![1.0; 2]).unwrap()),
            ),
            NurbsError::Degenerate(""),
        ),
        (
            "bad weight",
            NurbsSurface3::new(
                [1, 1],
                [k1.clone(), k1.clone()],
                g.clone(),
                Some(Grid::new(2, 2, vec![1.0, 1.0, 1.0, 0.0]).unwrap()),
            ),
            NurbsError::Weight { index: 3 },
        ),
    ];
    for (name, r, want) in cases {
        let e = r.expect_err(name);
        assert!(same(&e, &want), "{name}: got {e:?}, want {want:?}");
    }
}

#[test]
fn serde_round_trip_and_validation() {
    let c = NurbsCurve3::new(
        2,
        kv(&[0., 0., 0., 0.4, 1., 1., 1.]),
        pts(4),
        Some(vec![1.0, 0.5, 2.0, 1.0]),
    )
    .unwrap();
    let j = serde_json::to_string(&c).unwrap();
    let back: NurbsCurve3 = serde_json::from_str(&j).unwrap();
    assert_eq!(back, c);
    let s = amg_nurbs::analytic::sphere(Point3::new(1.0, 2.0, 3.0), 4.0).unwrap();
    let j = serde_json::to_string(&s).unwrap();
    let back: NurbsSurface3 = serde_json::from_str(&j).unwrap();
    assert_eq!(back, s);
    // Invalid payloads are rejected on deserialisation.
    let bad = r#"{"degree":2,"knots":[0,0,1,1],"poles":[[0,0,0],[1,0,0]],"weights":null}"#;
    assert!(serde_json::from_str::<NurbsCurve3>(bad).is_err());
    assert!(serde_json::from_str::<KnotVector>("[1.0, 0.0]").is_err());
    let bad_grid = r#"{"nu":2,"nv":2,"data":[1.0,2.0,3.0]}"#;
    assert!(serde_json::from_str::<Grid<f64>>(bad_grid).is_err());
}

// ---- no-panic proptest --------------------------------------------------------------------

fn any_f64() -> impl Strategy<Value = f64> {
    prop_oneof![
        8 => -10.0f64..10.0,
        1 => Just(f64::NAN),
        1 => Just(f64::INFINITY),
        1 => Just(f64::NEG_INFINITY),
        1 => Just(0.0),
        1 => Just(1.0),
        1 => any::<f64>(),
    ]
}

/// Knot vectors that are often valid (sorted, clamped) but sometimes not.
fn knots_for(p: usize, n: usize) -> impl Strategy<Value = Vec<f64>> {
    let m = n + p + 1;
    (
        prop::collection::vec(0.0f64..1.0, m.saturating_sub(2 * (p + 1))),
        prop::bool::weighted(0.8),
        prop::collection::vec(any_f64(), 0..m + 3),
        prop::bool::weighted(0.15),
    )
        .prop_map(move |(mut inner, clamped, raw, dup)| {
            if !clamped {
                return raw;
            }
            inner.sort_by(f64::total_cmp);
            if dup && !inner.is_empty() {
                let x = inner[0];
                inner.iter_mut().take(p + 1).for_each(|k| *k = x);
            }
            let mut v = vec![0.0; p + 1];
            v.extend(inner);
            v.extend(std::iter::repeat_n(1.0, p + 1));
            v
        })
}

fn curve_inputs() -> impl Strategy<Value = (usize, Vec<f64>, Vec<Point3>, Option<Vec<f64>>)> {
    (0usize..30, 1usize..12).prop_flat_map(|(p, n)| {
        (
            Just(p),
            knots_for(p, n),
            prop::collection::vec((any_f64(), any_f64(), any_f64()), n..=n + 1),
            prop::option::of(prop::collection::vec(any_f64(), n..=n + 1)),
        )
            .prop_map(|(p, k, raw, w)| {
                let pts = raw
                    .into_iter()
                    .map(|(x, y, z)| Point3::new(x, y, z))
                    .collect();
                (p, k, pts, w)
            })
    })
}

fn exercise_curve(c: &NurbsCurve3, ts: &[f64]) {
    let tol = Tolerance::DEFAULT;
    for &t in ts {
        let _ = c.eval(t);
        let _ = c.derivs(t, 7);
        let _ = c.curvature(t);
        let _ = c.torsion(t);
    }
    let _ = c.reversed().eval(0.5);
    let _ = c.reparametrized((ts[0], ts[0] + 1.0)).domain();
    let _ = c.transformed(&Transform::from_rotation_z(0.3)).bbox();
    let _ = c.planarity(&tol);
    let _ = c.is_closed(&tol);
    let _ = c.is_rational();
    let _ = c.knots().greville(c.degree());
    let _ = c.knots().to_knots_mults(1e-9);
}

proptest! {
    #![proptest_config(ProptestConfig { cases: 512, ..ProptestConfig::default() })]

    #[test]
    fn curves_never_panic(
        (p, k, poles, w) in curve_inputs(),
        ts in prop::collection::vec(any_f64(), 1..6),
    ) {
        if let Ok(kv) = KnotVector::new(k.clone()) {
            for &t in &ts {
                let _ = kv.find_span(p, poles.len(), t);
                let _ = kv.multiplicity(t, 1e-9);
            }
            let _ = kv.domain(p);
            let _ = kv.is_clamped(p);
            let mut out = [0.0; 30];
            amg_nurbs::basis::basis_funs(kv.as_slice(), p, ts[0], p, &mut out);
            if let Ok(c) = NurbsCurve3::new(p, kv, poles, w) {
                exercise_curve(&c, &ts);
            }
        }
    }

    #[test]
    fn surfaces_never_panic(
        (p, ku, rows) in (0usize..9, 1usize..6).prop_flat_map(|(p, n)| (Just(p), knots_for(p, n), Just(n))),
        (q, kvv, cols) in (0usize..9, 1usize..6).prop_flat_map(|(q, n)| (Just(q), knots_for(q, n), Just(n))),
        raw in prop::collection::vec(any_f64(), 0..120),
        weighted in any::<bool>(),
        uv in (any_f64(), any_f64()),
    ) {
        let n = rows * cols;
        let pts: Vec<Point3> = (0..n)
            .map(|i| Point3::new(
                raw.get(3 * i).copied().unwrap_or(0.5),
                raw.get(3 * i + 1).copied().unwrap_or(-0.5),
                raw.get(3 * i + 2).copied().unwrap_or(1.0),
            ))
            .collect();
        let w = weighted.then(|| Grid::new(rows, cols, (0..n).map(|i| raw.get(i).copied().unwrap_or(1.0)).collect()).unwrap());
        let (Ok(ku), Ok(kvv), Ok(g)) = (KnotVector::new(ku), KnotVector::new(kvv), Grid::new(rows, cols, pts)) else {
            return Ok(());
        };
        if let Ok(s) = NurbsSurface3::new([p, q], [ku, kvv], g, w) {
            let tol = Tolerance::DEFAULT;
            let _ = s.eval(uv.0, uv.1);
            let _ = s.derivs(uv.0, uv.1, 5);
            let _ = s.normal(uv.0, uv.1, &tol);
            for d in [UV::U, UV::V] {
                let _ = s.iso_curve(d, uv.0).eval(uv.1);
                let _ = s.reversed(d).eval(uv.0, uv.1);
            }
            for side in [Side::UMin, Side::UMax, Side::VMin, Side::VMax] {
                let _ = s.boundary(side).eval(uv.0);
            }
            let _ = s.transposed().bbox();
            let _ = s.transformed(&Transform::from_scale(amg_core::Vec3::splat(2.0))).eval(0.0, 0.0);
        }
    }

    #[test]
    fn analytic_never_panics(
        a in any_f64(), b in any_f64(), c in any_f64(), d in any_f64(),
    ) {
        let pl = amg_core::Plane::XY;
        let _ = amg_nurbs::analytic::circle_arc(&pl, a, b, c);
        let _ = amg_nurbs::analytic::ellipse_arc(&pl, a, b, c, d);
        let _ = amg_nurbs::analytic::sphere(Point3::new(a, b, 0.0), c);
        let _ = amg_nurbs::analytic::torus(&pl, a, b);
        let _ = amg_nurbs::analytic::cylinder(&pl, a, b, c);
        let _ = amg_nurbs::analytic::cone(&pl, a, b, c, d);
        let _ = amg_nurbs::analytic::plane_patch(&pl, (a, b), (c, d)).eval(0.5, 0.5);
        let l = amg_nurbs::analytic::line(Point3::new(a, b, c), Point3::splat(d));
        let _ = amg_nurbs::analytic::extrusion(&l, amg_core::Vec3::new(a, b, c)).eval(0.5, 0.5);
        let axis = amg_core::Line3 { point: Point3::new(a, 0.0, 0.0), dir: amg_core::Dir3::Z };
        let _ = amg_nurbs::analytic::revolution(&l, &axis, d);
    }
}
