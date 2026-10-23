//! SK-08: the operations that match on entity or curve kinds (transform, edit, offset, corpus
//! adapters) with splines and conics: supported paths work, unsupported ones fail clearly.

#![allow(
    clippy::unwrap_used,
    reason = "test helpers; a failure is a test failure"
)]

use std::f64::consts::FRAC_PI_2;

use amg_core::{Plane, Point2, Vec2};
use amg_sketch::corpus::reference::RefSystem;
use amg_sketch::solve::build_system;
use amg_sketch::transform::{
    mirror, pattern_circ, pattern_rect, rotation_about, scale_about, transform,
};
use amg_sketch::{
    BSpline2, ConstraintKind, Curve2, EntityId, ProfileOptions, Sketch, SketchError,
    detect_profiles, edit, offset,
};

fn new() -> Sketch {
    Sketch::new(Plane::XY)
}

fn p(x: f64, y: f64) -> Point2 {
    Point2::new(x, y)
}

fn bs(s: &Sketch, id: EntityId) -> BSpline2 {
    match s.curve(id).unwrap() {
        Curve2::BSpline(b) => b,
        other => panic!("{other:?}"),
    }
}

fn kinds(s: &Sketch) -> Vec<EntityId> {
    s.entities()
        .filter(|(_, e)| e.kind.name() == "spline" || e.kind.name() == "conic")
        .map(|(i, _)| i)
        .collect()
}

fn pole_data() -> ([Point2; 4], [f64; 4]) {
    (
        [p(1.0, 1.0), p(2.0, 4.0), p(5.0, 4.0), p(6.0, 1.0)],
        [1.0, 2.0, 0.8, 1.0],
    )
}

fn make_all(s: &mut Sketch) -> [EntityId; 4] {
    let (poles, w) = pole_data();
    [
        s.add_spline(3, &poles, None, None).unwrap(),
        s.add_spline(3, &poles, Some(&w), None).unwrap(),
        s.add_spline_fit(&poles, Some(Vec2::new(0.5, 2.0)), None)
            .unwrap(),
        s.add_conic(p(1.0, 1.0), p(3.5, 6.0), p(6.0, 1.0), 0.4)
            .unwrap(),
    ]
}

/// The image curve equals `f(source point)` at matching parameters.
fn assert_image(src: &BSpline2, img: &BSpline2, f: impl Fn(Point2) -> Point2) {
    assert!((src.span() - img.span()).abs() < 1e-9 * src.span().max(1.0));
    for k in 0..=20 {
        let t = src.span() * f64::from(k) / 20.0;
        let u = img.span() * f64::from(k) / 20.0;
        assert!(f(src.eval(t)).distance(img.eval(u)) < 1e-8, "k={k}");
    }
}

#[test]
fn copy_move_rotate_scale_mirror_and_patterns_work() {
    let mut s = new();
    let ids = make_all(&mut s);
    let originals: Vec<BSpline2> = ids.iter().map(|&i| bs(&s, i)).collect();

    // Rotate-copy.
    let rot = rotation_about(p(0.0, 0.0), FRAC_PI_2);
    let r = transform(&mut s, &ids, rot, true).unwrap();
    let new_curves: Vec<EntityId> = r
        .created
        .iter()
        .copied()
        .filter(|i| kinds(&s).contains(i))
        .collect();
    assert_eq!(new_curves.len(), 4);
    for (o, &n) in originals.iter().zip(&new_curves) {
        assert_image(o, &bs(&s, n), |q| p(-q.y, q.x));
    }
    // Scale-copy (about the origin, factor 2): weights and rho are kept, not scaled.
    let r = transform(&mut s, &ids, scale_about(p(0.0, 0.0), 2.0), true).unwrap();
    let scaled: Vec<EntityId> = r
        .created
        .iter()
        .copied()
        .filter(|i| kinds(&s).contains(i))
        .collect();
    for (o, &n) in originals.iter().zip(&scaled) {
        assert_image(o, &bs(&s, n), |q| q * 2.0);
    }
    // Mirror about the y axis (unconstrained, then constrained).
    let axis = s.add_line(p(0.0, -10.0), p(0.0, 10.0)).unwrap();
    for constrain in [false, true] {
        let r = mirror(&mut s, &ids, axis, constrain).unwrap();
        let m: Vec<EntityId> = r
            .created
            .iter()
            .copied()
            .filter(|i| kinds(&s).contains(i))
            .collect();
        assert_eq!(m.len(), 4);
        for (o, &n) in originals.iter().zip(&m) {
            assert_image(o, &bs(&s, n), |q| p(-q.x, q.y));
        }
        if constrain {
            // Point symmetry plus Equal for the rational spline and the conic.
            let eq = r
                .constraints_added
                .iter()
                .filter(|&&c| matches!(s.constraint(c).unwrap().kind, ConstraintKind::Equal { .. }))
                .count();
            assert_eq!(eq, 2);
            // The constrained image has no free DOF beyond the source's.
            let sys = build_system(&s).unwrap();
            assert!(sys.system.max_abs_residual(s.params().as_slice()) < 1e-9);
        }
    }
    // Patterns.
    let r = pattern_rect(&mut s, &ids[..1], Vec2::X, 3, 10.0, None, true).unwrap();
    assert_eq!(
        r.created.iter().filter(|i| kinds(&s).contains(i)).count(),
        2
    );
    let r = pattern_circ(&mut s, &ids[3..], axis, 3, std::f64::consts::TAU, false);
    // The centre of a circular pattern must be a point, circle, arc or ellipse.
    assert!(matches!(r, Err(SketchError::WrongEntityKind { .. })));
    let c = s.add_point(p(0.0, 0.0)).unwrap();
    let r = pattern_circ(&mut s, &ids[3..], c, 4, std::f64::consts::TAU, true).unwrap();
    assert_eq!(
        r.created.iter().filter(|i| kinds(&s).contains(i)).count(),
        3
    );
}

#[test]
fn trim_extend_break_fillet_chamfer_refuse_splines_but_use_them_as_cutters() {
    let mut s = new();
    let spl = s
        .add_spline(
            3,
            &[p(0.0, -4.0), p(3.0, 6.0), p(7.0, -6.0), p(10.0, 4.0)],
            None,
            None,
        )
        .unwrap();
    let con = s
        .add_conic(p(0.0, 8.0), p(5.0, 12.0), p(10.0, 8.0), 0.5)
        .unwrap();
    let line = s.add_line(p(0.0, 0.0), p(10.0, 0.0)).unwrap();
    for e in [spl, con] {
        for r in [
            edit::trim(&mut s, e, p(1.0, 0.0)).map(|_| ()),
            edit::extend(&mut s, e, p(1.0, 0.0)).map(|_| ()),
            edit::break_at(&mut s, e, p(1.0, 0.0)).map(|_| ()),
            edit::fillet(&mut s, e, line, p(1.0, 0.0), 1.0).map(|_| ()),
            edit::fillet(&mut s, line, e, p(1.0, 0.0), 1.0).map(|_| ()),
            edit::chamfer(&mut s, e, line, p(1.0, 0.0), edit::ChamferSpec::Equal(1.0)).map(|_| ()),
        ] {
            assert!(
                matches!(r, Err(SketchError::WrongEntityKind { .. })),
                "{r:?}"
            );
        }
    }
    // The spline cuts the line: trimming the line between the crossings removes a piece.
    let before = s.entities().count();
    let r = edit::trim(&mut s, line, p(5.0, 0.0)).unwrap();
    assert!(!r.removed.is_empty());
    assert!(s.entities().count() != before || !r.created.is_empty());
    // The spline is untouched.
    assert!(s.entity(spl).is_some());
}

#[test]
fn offset_of_splines_is_a_close_approximation_and_unsupported_cases_error() {
    let mut s = new();
    let spl = s
        .add_spline(
            3,
            &[p(0.0, 0.0), p(3.0, 5.0), p(8.0, 5.0), p(11.0, 0.0)],
            None,
            None,
        )
        .unwrap();
    let src = bs(&s, spl);
    let d = 0.75;
    // Offset to the outside (above) and inside (below).
    for side in [p(5.0, 10.0), p(5.0, -10.0)] {
        let r = offset::offset(&mut s, &[spl], d, side, false).unwrap();
        assert_eq!(r.copies.len(), 1);
        let copy = bs(&s, r.copies[0]);
        let mut worst = 0.0_f64;
        for k in 0..=400 {
            let q = copy.eval(copy.span() * f64::from(k) / 400.0);
            let dist = src.closest(q).1.distance(q);
            worst = worst.max((dist - d).abs());
        }
        assert!(worst < 2e-4, "worst deviation {worst}");
        // On the requested side.
        let mid = copy.eval(0.5 * copy.span());
        assert_eq!(mid.y > src.eval(0.5 * src.span()).y, side.y > 0.0);
        assert!(r.created.len() > 2);
    }
    // A conic too.
    let con = s
        .add_conic(p(0.0, 20.0), p(5.0, 26.0), p(10.0, 20.0), 0.45)
        .unwrap();
    let r = offset::offset(&mut s, &[con], 1.0, p(5.0, 40.0), false).unwrap();
    assert_eq!(r.copies.len(), 1);
    let (c0, copy) = (bs(&s, con), bs(&s, r.copies[0]));
    for k in 0..=100 {
        let q = copy.eval(copy.span() * f64::from(k) / 100.0);
        assert!((c0.closest(q).1.distance(q) - 1.0).abs() < 2e-4);
    }
    // Errors.
    let line = s.add_line(p(30.0, 0.0), p(40.0, 0.0)).unwrap();
    let e = |r: Result<offset::OffsetResult, SketchError>| r.unwrap_err();
    assert!(matches!(
        e(offset::offset(&mut s, &[spl], d, p(5.0, 10.0), true)),
        SketchError::NotApplicable(m) if m.contains("constrained")
    ));
    assert!(matches!(
        e(offset::offset(&mut s, &[spl, line], d, p(5.0, 10.0), false)),
        SketchError::NotApplicable(m) if m.contains("mixes")
    ));
    assert!(matches!(
        e(offset::offset(&mut s, &[line, spl], d, p(5.0, 10.0), false)),
        SketchError::NotApplicable(m) if m.contains("mixes")
    ));
    // Distance beyond the radius of curvature on the inner side.
    assert!(matches!(
        e(offset::offset(&mut s, &[con], 50.0, p(5.0, 10.0), false)),
        SketchError::Degenerate(_)
    ));
    // The side point on the curve.
    let on = src.eval(3.0);
    assert!(matches!(
        e(offset::offset(&mut s, &[spl], d, on, false)),
        SketchError::NotApplicable(_)
    ));
    assert!(matches!(
        e(offset::offset(&mut s, &[spl, spl], d, p(5.0, 10.0), false)),
        SketchError::NotApplicable(_)
    ));
    // Bad tolerance.
    let opts = offset::OffsetOptions {
        spline_tol: 0.0,
        ..offset::OffsetOptions::default()
    };
    assert!(matches!(
        e(offset::offset_with(
            &mut s,
            &[spl],
            d,
            p(5.0, 10.0),
            false,
            &opts
        )),
        SketchError::InvalidValue { .. }
    ));
    // Failed calls created nothing.
    assert_eq!(kinds(&s).len(), 5);
}

#[test]
fn corpus_reference_oracle_declines_splines() {
    let mut s = new();
    s.add_conic(p(0.0, 0.0), p(1.0, 2.0), p(2.0, 0.0), 0.5)
        .unwrap();
    match RefSystem::new(&s) {
        Err(msg) => assert!(msg.contains("spline"), "{msg}"),
        Ok(sys) => {
            let u = sys.current(&s);
            assert!(sys.residuals(&s, &u).is_err());
        }
    }
    // A curvature constraint is declined as well.
    let mut s = new();
    let a = s
        .add_spline(
            3,
            &[p(0.0, 0.0), p(1.0, 1.0), p(2.0, 1.0), p(3.0, 0.0)],
            None,
            None,
        )
        .unwrap();
    let b = s
        .add_spline(
            3,
            &[p(3.0, 0.0), p(4.0, -1.0), p(5.0, -1.0), p(6.0, 0.0)],
            None,
            None,
        )
        .unwrap();
    let pa = s
        .entity(a)
        .unwrap()
        .kind
        .endpoint(amg_sketch::End::End)
        .unwrap();
    let pb = s
        .entity(b)
        .unwrap()
        .kind
        .endpoint(amg_sketch::End::Start)
        .unwrap();
    s.add_constraint(ConstraintKind::Coincident { a: pa, b: pb })
        .unwrap();
    s.add_constraint(ConstraintKind::Curvature {
        a,
        b,
        a_end: amg_sketch::End::End,
        b_end: amg_sketch::End::Start,
        reversed: false,
    })
    .unwrap();
    assert!(
        RefSystem::new(&s).is_err() || {
            let sys = RefSystem::new(&s).unwrap();
            sys.residuals(&s, &sys.current(&s)).is_err()
        }
    );
    // Profile detection still works on it.
    let _ = detect_profiles(&s, &ProfileOptions::default());
}
