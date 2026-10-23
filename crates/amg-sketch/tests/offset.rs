//! SK-17 acceptance: offset of line/arc/circle chains with gap closing, trimming of
//! self-intersecting offsets, and the optional offset constraint.
//!
//! Done when: (1) open/closed chain fixtures, including self-intersecting offsets that get
//! trimmed; (2) with `constrain = true`, editing the offset dimension moves the copy.

#![allow(
    clippy::unwrap_used,
    clippy::float_cmp,
    reason = "test helpers; a failure is a test failure"
)]

use std::f64::consts::{FRAC_PI_2, PI};

use amg_core::{Plane, Point2};
use amg_sketch::offset::{self, CornerStyle, OffsetOptions, OffsetResult};
use amg_sketch::solve::proto::{ProtoOptions, ProtoStatus, solve};
use amg_sketch::solve::{build_system, dof};
use amg_sketch::{
    ConstraintKind, Curve2, EntityId, EntityKind, ProfileOptions, ProfileSet, Sketch, SketchError,
    detect_profiles,
};

fn p(x: f64, y: f64) -> Point2 {
    Point2::new(x, y)
}

fn new() -> Sketch {
    Sketch::new(Plane::XY)
}

/// Polygon through `pts` (consecutive lines share point entities); closed if `close`.
fn poly(s: &mut Sketch, pts: &[(f64, f64)], close: bool) -> Vec<EntityId> {
    let ids: Vec<EntityId> = pts
        .iter()
        .map(|&(x, y)| s.add_point(p(x, y)).unwrap())
        .collect();
    let n = if close { ids.len() } else { ids.len() - 1 };
    (0..n)
        .map(|i| {
            s.add_line_between(ids[i], ids[(i + 1) % ids.len()])
                .unwrap()
        })
        .collect()
}

fn rect(s: &mut Sketch, w: f64, h: f64) -> Vec<EntityId> {
    poly(s, &[(0.0, 0.0), (w, 0.0), (w, h), (0.0, h)], true)
}

/// Stadium: two lines and two end arcs (coincident but unshared points). Radius 2, centres
/// `(0, 2)` and `(10, 2)`.
fn stadium(s: &mut Sketch) -> Vec<EntityId> {
    vec![
        s.add_line(p(0.0, 0.0), p(10.0, 0.0)).unwrap(),
        s.add_arc(p(10.0, 2.0), p(10.0, 0.0), p(10.0, 4.0)).unwrap(),
        s.add_line(p(10.0, 4.0), p(0.0, 4.0)).unwrap(),
        s.add_arc(p(0.0, 2.0), p(0.0, 4.0), p(0.0, 0.0)).unwrap(),
    ]
}

fn curves(s: &Sketch, ids: &[EntityId]) -> Vec<Curve2> {
    ids.iter().map(|&i| s.curve(i).unwrap()).collect()
}

/// A fresh sketch holding only the copy, to run profile detection on it.
fn copy_sketch(s: &Sketch, res: &OffsetResult) -> Sketch {
    let mut c = new();
    for cv in curves(s, &res.copies) {
        match cv {
            Curve2::Line(l) => {
                c.add_line(l.a, l.b).unwrap();
            }
            Curve2::Arc(a) => {
                let (_, t1) = cv.param_range();
                c.add_arc(a.center, cv.eval(0.0), cv.eval(t1)).unwrap();
            }
            Curve2::Circle(k) => {
                c.add_circle(k.center, k.radius).unwrap();
            }
            Curve2::Ellipse(_) => unreachable!("offsets of lines, arcs and circles only"),
        }
    }
    c
}

fn profiles(s: &Sketch, res: &OffsetResult) -> ProfileSet {
    let c = copy_sketch(s, res);
    let set = detect_profiles(&c, &ProfileOptions::default());
    assert!(set.issues.is_empty(), "issues: {:?}", set.issues);
    set
}

/// Areas of the profiles (holes ignored in the count), one profile expected.
fn one_area(s: &Sketch, res: &OffsetResult) -> f64 {
    let set = profiles(s, res);
    assert_eq!(set.profiles.len(), 1, "profiles: {}", set.profiles.len());
    assert!(set.open_chains.is_empty(), "closed result expected");
    set.profiles[0].fingerprint.area
}

fn near(a: f64, b: f64, rel: f64) {
    assert!((a - b).abs() <= rel * b.abs().max(1.0), "got {a}, want {b}");
}

/// Every sample of every copy piece is at least `d` (and, for non-corner pieces, mostly
/// exactly `d`) from the source curves.
fn assert_not_closer_than(src: &[Curve2], copy: &[Curve2], d: f64) {
    for c in copy {
        let (t0, t1) = c.param_range();
        for k in 0..=20 {
            let t = t0 + (t1 - t0) * f64::from(k) / 20.0;
            let q = c.eval(t);
            let dist = src
                .iter()
                .map(|s| s.closest(q).1.distance(q))
                .fold(f64::INFINITY, f64::min);
            assert!(dist >= d - 1e-6, "point {q:?} is {dist} from the source");
        }
    }
}

fn total_length(cs: &[Curve2]) -> f64 {
    cs.iter().map(Curve2::length).sum()
}

fn has_line(cs: &[Curve2], a: Point2, b: Point2) -> bool {
    cs.iter().any(|c| match c {
        Curve2::Line(l) => {
            (l.a.distance(a) < 1e-6 && l.b.distance(b) < 1e-6)
                || (l.a.distance(b) < 1e-6 && l.b.distance(a) < 1e-6)
        }
        _ => false,
    })
}

fn lines(cs: &[Curve2]) -> usize {
    cs.iter().filter(|c| matches!(c, Curve2::Line(_))).count()
}

fn arcs(cs: &[Curve2]) -> usize {
    cs.iter().filter(|c| matches!(c, Curve2::Arc(_))).count()
}

fn run(
    s: &mut Sketch,
    chain: &[EntityId],
    d: f64,
    side: Point2,
    constrain: bool,
) -> (OffsetResult, Vec<Curve2>) {
    let src = curves(s, chain);
    let res = offset::offset(s, chain, d, side, constrain).unwrap();
    let copy = curves(s, &res.copies);
    assert_not_closer_than(&src, &copy, d);
    (res, copy)
}

// ---- Done-when 1: closed chains ----------------------------------------------------------

#[test]
fn rect_outward_extends_to_sharp_corners() {
    let mut s = new();
    let r = rect(&mut s, 10.0, 6.0);
    let (res, copy) = run(&mut s, &r, 2.0, p(-1.0, -1.0), false);
    assert_eq!((lines(&copy), arcs(&copy)), (4, 0));
    near(one_area(&s, &res), 14.0 * 10.0, 1e-9);
    assert!(has_line(&copy, p(-2.0, -2.0), p(12.0, -2.0)));
    assert!(res.removed.is_empty() && res.constraints_added.is_empty());
    // The source is untouched.
    assert!(r.iter().all(|&e| s.entity(e).is_some()));
}

#[test]
fn rect_outward_round_corners() {
    let mut s = new();
    let r = rect(&mut s, 10.0, 6.0);
    let src = curves(&s, &r);
    let opts = OffsetOptions {
        corner: CornerStyle::Round,
        ..OffsetOptions::default()
    };
    let res = offset::offset_with(&mut s, &r, 2.0, p(-1.0, -1.0), false, &opts).unwrap();
    let copy = curves(&s, &res.copies);
    assert_not_closer_than(&src, &copy, 2.0);
    assert_eq!((lines(&copy), arcs(&copy)), (4, 4));
    near(one_area(&s, &res), 60.0 + 32.0 * 2.0 + PI * 4.0, 1e-9);
}

#[test]
fn rect_inward() {
    let mut s = new();
    let r = rect(&mut s, 10.0, 6.0);
    let (res, copy) = run(&mut s, &r, 1.0, p(5.0, 3.0), false);
    assert_eq!(lines(&copy), 4);
    near(one_area(&s, &res), 8.0 * 4.0, 1e-9);
}

#[test]
fn rect_inward_thin_and_collapsed() {
    let mut s = new();
    let r = rect(&mut s, 10.0, 6.0);
    let (res, _) = run(&mut s, &r, 2.9, p(5.0, 3.0), false);
    near(one_area(&s, &res), 4.2 * 0.2, 1e-9);
    // Larger than half the height: nothing is left.
    let n = s.entities().count();
    let e = offset::offset(&mut s, &r, 3.5, p(5.0, 3.0), false).unwrap_err();
    assert!(matches!(e, SketchError::Degenerate(_)), "{e:?}");
    assert_eq!(
        s.entities().count(),
        n,
        "failed offset leaves the sketch alone"
    );
}

#[test]
fn l_shape_outward_and_inward_miter_areas() {
    let pts = [
        (0.0, 0.0),
        (6.0, 0.0),
        (6.0, 2.0),
        (2.0, 2.0),
        (2.0, 6.0),
        (0.0, 6.0),
    ];
    // Rectilinear polygon: area +- perimeter * d + d^2 * (convex - reflex corners).
    let mut s = new();
    let l = poly(&mut s, &pts, true);
    let (res, _) = run(&mut s, &l, 1.0, p(-1.0, -1.0), false);
    near(one_area(&s, &res), 20.0 + 24.0 + 4.0, 1e-9);
    let mut s = new();
    let l = poly(&mut s, &pts, true);
    let (res, _) = run(&mut s, &l, 0.5, p(1.0, 1.0), false);
    near(one_area(&s, &res), 20.0 - 12.0 + 1.0, 1e-9);
}

#[test]
fn narrow_tab_vanishes_when_offset_inward() {
    // A 10x10 square with a 1-wide tab on top. Offsetting inward by 0.8 (> half the tab
    // width) makes the tab's offset edges cross; the loop is trimmed away.
    let mut s = new();
    let t = poly(
        &mut s,
        &[
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (5.5, 10.0),
            (5.5, 13.0),
            (4.5, 13.0),
            (4.5, 10.0),
            (0.0, 10.0),
        ],
        true,
    );
    let (res, copy) = run(&mut s, &t, 0.8, p(2.0, 2.0), false);
    near(one_area(&s, &res), 8.4 * 8.4, 1e-9);
    // No piece strays into the tab.
    for c in &copy {
        let (t0, t1) = c.param_range();
        assert!(c.eval(0.5 * (t0 + t1)).y < 9.3);
    }
    // With a smaller offset the tab survives.
    let mut s = new();
    let t = poly(
        &mut s,
        &[
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (5.5, 10.0),
            (5.5, 13.0),
            (4.5, 13.0),
            (4.5, 10.0),
            (0.0, 10.0),
        ],
        true,
    );
    let (res, _) = run(&mut s, &t, 0.3, p(2.0, 2.0), false);
    // 9.4 x 9.4 square plus a 0.4 x (13 - 0.3 - 9.7) tab.
    near(one_area(&s, &res), 9.4 * 9.4 + 0.4 * 3.0, 1e-9);
}

#[test]
fn stadium_with_arcs_outward_and_inward() {
    let mut s = new();
    let st = stadium(&mut s);
    let (res, copy) = run(&mut s, &st, 1.0, p(5.0, 6.0), false);
    assert_eq!((lines(&copy), arcs(&copy)), (2, 2));
    near(one_area(&s, &res), PI * 9.0 + 60.0, 1e-9);
    let (res, _) = run(&mut s, &st, 1.0, p(5.0, 2.0), false);
    near(one_area(&s, &res), PI + 20.0, 1e-9);
}

#[test]
fn sector_with_miter_at_line_arc_corners() {
    // Quarter disc of radius 5, offset outward by 1 with sharp (extended) corners.
    let mut s = new();
    let l1 = s.add_line(p(0.0, 0.0), p(5.0, 0.0)).unwrap();
    let a = s.add_arc(p(0.0, 0.0), p(5.0, 0.0), p(0.0, 5.0)).unwrap();
    let l2 = s.add_line(p(0.0, 5.0), p(0.0, 0.0)).unwrap();
    let (res, copy) = run(&mut s, &[l2, a, l1], 1.0, p(2.0, -1.0), false);
    assert_eq!((lines(&copy), arcs(&copy)), (2, 1));
    let alpha = (1.0_f64 / 6.0).asin();
    let want = 18.0 * (FRAC_PI_2 + 2.0 * alpha) + (1.0 + 35.0_f64.sqrt());
    near(one_area(&s, &res), want, 1e-9);
}

#[test]
fn circle_inward_outward_and_collapse() {
    let mut s = new();
    let c = s.add_circle(p(3.0, 4.0), 5.0).unwrap();
    let (res, copy) = run(&mut s, &[c], 2.0, p(3.0, 4.0), false);
    let Curve2::Circle(k) = copy[0] else {
        panic!("circle expected")
    };
    assert_eq!((k.center, k.radius), (p(3.0, 4.0), 3.0));
    assert_eq!(res.copies.len(), 1);
    let (_, copy) = run(&mut s, &[c], 2.0, p(20.0, 4.0), false);
    let Curve2::Circle(k) = copy[0] else {
        panic!("circle expected")
    };
    assert_eq!(k.radius, 7.0);
    let e = offset::offset(&mut s, &[c], 5.0, p(3.0, 4.0), false).unwrap_err();
    assert!(matches!(e, SketchError::Degenerate(_)), "{e:?}");
}

// ---- Done-when 1: open chains ------------------------------------------------------------

#[test]
fn open_polyline_outer_corner_extends() {
    let mut s = new();
    let ch = poly(&mut s, &[(0.0, 0.0), (4.0, 0.0), (4.0, 3.0)], false);
    // The chain given in a different order still works.
    let ch = [ch[1], ch[0]];
    let (_, copy) = run(&mut s, &ch, 1.0, p(2.0, -1.0), false);
    assert_eq!(lines(&copy), 2);
    assert!(has_line(&copy, p(0.0, -1.0), p(5.0, -1.0)));
    assert!(has_line(&copy, p(5.0, -1.0), p(5.0, 3.0)));
}

#[test]
fn open_polyline_inner_corner_trims() {
    let mut s = new();
    let ch = poly(&mut s, &[(0.0, 0.0), (4.0, 0.0), (4.0, 3.0)], false);
    let (_, copy) = run(&mut s, &ch, 1.0, p(2.0, 1.0), false);
    assert_eq!(lines(&copy), 2);
    assert!(has_line(&copy, p(0.0, 1.0), p(3.0, 1.0)));
    assert!(has_line(&copy, p(3.0, 1.0), p(3.0, 3.0)));
}

#[test]
fn open_step_chain_self_intersection_is_trimmed() {
    // Up-step; the left offset by 2 has a loop at the inner corner that must go.
    let mut s = new();
    let ch = poly(
        &mut s,
        &[(0.0, 0.0), (5.0, 0.0), (5.0, 1.0), (10.0, 1.0)],
        false,
    );
    let (_, copy) = run(&mut s, &ch, 2.0, p(1.0, 3.0), false);
    assert_eq!(lines(&copy), 3, "{copy:?}");
    assert!(has_line(&copy, p(0.0, 2.0), p(3.0, 2.0)));
    assert!(has_line(&copy, p(3.0, 2.0), p(3.0, 3.0)));
    assert!(has_line(&copy, p(3.0, 3.0), p(10.0, 3.0)));
    near(total_length(&copy), 11.0, 1e-12);
}

#[test]
fn open_chain_with_tangent_arc() {
    let mut s = new();
    let a = s.add_line(p(0.0, 0.0), p(5.0, 0.0)).unwrap();
    let b = s.add_arc(p(5.0, 2.0), p(5.0, 0.0), p(7.0, 2.0)).unwrap();
    let c = s.add_line(p(7.0, 2.0), p(7.0, 6.0)).unwrap();
    let (_, copy) = run(&mut s, &[b, c, a], 1.0, p(2.0, -1.0), false);
    assert_eq!((lines(&copy), arcs(&copy)), (2, 1));
    near(total_length(&copy), 5.0 + 3.0 * FRAC_PI_2 + 4.0, 1e-12);
    // Towards the centre: the arc shrinks to radius 1.
    let (_, copy) = run(&mut s, &[b, c, a], 1.0, p(2.0, 1.0), false);
    near(total_length(&copy), 5.0 + FRAC_PI_2 + 4.0, 1e-12);
}

#[test]
fn open_arc_collapse_is_degenerate() {
    let mut s = new();
    let a = s.add_arc(p(0.0, 0.0), p(3.0, 0.0), p(0.0, 3.0)).unwrap();
    let e = offset::offset(&mut s, &[a], 4.0, p(0.5, 0.5), false).unwrap_err();
    assert!(matches!(e, SketchError::Degenerate(_)), "{e:?}");
    let (_, copy) = run(&mut s, &[a], 1.0, p(0.5, 0.5), false);
    near(total_length(&copy), 2.0 * FRAC_PI_2, 1e-12);
}

#[test]
fn open_chain_round_corner() {
    let mut s = new();
    let ch = poly(&mut s, &[(0.0, 0.0), (4.0, 0.0), (4.0, 3.0)], false);
    let opts = OffsetOptions {
        corner: CornerStyle::Round,
        ..OffsetOptions::default()
    };
    let res = offset::offset_with(&mut s, &ch, 1.0, p(2.0, -1.0), false, &opts).unwrap();
    let copy = curves(&s, &res.copies);
    assert_eq!((lines(&copy), arcs(&copy)), (2, 1));
    near(total_length(&copy), 4.0 + 3.0 + FRAC_PI_2, 1e-12);
}

#[test]
fn miter_limit_rounds_sharp_corners() {
    // A 20 degree wedge: the miter would reach about 5.8 d from the corner.
    let mut s = new();
    let ch = poly(&mut s, &[(10.0, 0.0), (0.0, 0.0), (9.4, 3.42)], false);
    let opts = OffsetOptions {
        miter_limit: 2.0,
        ..OffsetOptions::default()
    };
    let res = offset::offset_with(&mut s, &ch, 1.0, p(5.0, -1.0), false, &opts).unwrap();
    assert_eq!(arcs(&curves(&s, &res.copies)), 1);
    let res = offset::offset(&mut s, &ch, 1.0, p(5.0, -1.0), false).unwrap();
    assert_eq!(arcs(&curves(&s, &res.copies)), 0);
}

#[test]
fn two_separate_chains_are_offset_each() {
    let mut s = new();
    let a = s.add_line(p(0.0, 0.0), p(4.0, 0.0)).unwrap();
    let b = s.add_line(p(0.0, 10.0), p(4.0, 10.0)).unwrap();
    let c = s.add_circle(p(20.0, 0.0), 3.0).unwrap();
    // `side` is above both lines; the circle is offset outward by the same rule (left of its
    // CCW travel is inside; `side` is far to the left, i.e. outside).
    let (res, copy) = run(&mut s, &[a, b, c], 1.0, p(2.0, 12.0), false);
    assert_eq!(res.copies.len(), 3);
    assert!(has_line(&copy, p(0.0, 1.0), p(4.0, 1.0)));
    assert!(has_line(&copy, p(0.0, 11.0), p(4.0, 11.0)));
    assert!(
        copy.iter()
            .any(|c| matches!(c, Curve2::Circle(k) if k.radius == 4.0))
    );
}

#[test]
fn errors() {
    let mut s = new();
    let pt = s.add_point(p(0.0, 0.0)).unwrap();
    let a = s.add_line(p(0.0, 0.0), p(4.0, 0.0)).unwrap();
    let b = s.add_line(p(4.0, 0.0), p(4.0, 3.0)).unwrap();
    let c = s.add_line(p(4.0, 0.0), p(8.0, 0.0)).unwrap();
    let side = p(1.0, 1.0);
    let n = s.entities().count();
    let err = |s: &mut Sketch, ch: &[EntityId], d: f64, side: Point2| {
        offset::offset(s, ch, d, side, true).unwrap_err()
    };
    assert!(matches!(
        err(&mut s, &[a], 0.0, side),
        SketchError::InvalidValue { .. }
    ));
    assert!(matches!(
        err(&mut s, &[a], f64::NAN, side),
        SketchError::InvalidValue { .. }
    ));
    assert!(matches!(
        err(&mut s, &[a], 1.0, p(f64::NAN, 0.0)),
        SketchError::InvalidValue { .. }
    ));
    assert!(matches!(
        err(&mut s, &[pt], 1.0, side),
        SketchError::WrongEntityKind { .. }
    ));
    assert!(matches!(
        err(&mut s, &[EntityId(999)], 1.0, side),
        SketchError::UnknownEntity(_)
    ));
    assert!(matches!(
        err(&mut s, &[a, a], 1.0, side),
        SketchError::NotApplicable(_)
    ));
    assert!(matches!(
        err(&mut s, &[], 1.0, side),
        SketchError::NotApplicable(_)
    ));
    // Three lines meet at (4, 0).
    assert!(matches!(
        err(&mut s, &[a, b, c], 1.0, side),
        SketchError::NotApplicable(_)
    ));
    // The side point is on the chain.
    assert!(matches!(
        err(&mut s, &[a], 1.0, p(2.0, 0.0)),
        SketchError::NotApplicable(_)
    ));
    assert_eq!(s.entities().count(), n);
}

#[test]
fn offset_is_deterministic() {
    let build = || {
        let mut s = new();
        let st = stadium(&mut s);
        let res = offset::offset(&mut s, &st, 1.0, p(5.0, 6.0), true).unwrap();
        (s, res)
    };
    let (s1, r1) = build();
    let (s2, r2) = build();
    assert_eq!(r1.created, r2.created);
    assert_eq!(r1.constraints_added, r2.constraints_added);
    assert_eq!(curves(&s1, &r1.copies), curves(&s2, &r2.copies));
}

// ---- Done-when 2: the offset constraint --------------------------------------------------

/// Fixes every point of the source (so the whole copy must be determined by the source and
/// the offset dimension).
fn pin_points(s: &mut Sketch) {
    let pts: Vec<EntityId> = s
        .entities()
        .filter(|(_, e)| matches!(e.kind, EntityKind::Point { .. }))
        .map(|(i, _)| i)
        .collect();
    for e in pts {
        s.add_constraint(ConstraintKind::Fix { entity: e }).unwrap();
    }
}

fn residual(s: &Sketch) -> f64 {
    let sys = build_system(s).unwrap();
    sys.system.max_abs_residual(s.params().as_slice())
}

fn solved(s: &mut Sketch) {
    let r = solve(s, &ProtoOptions::default()).unwrap();
    assert!(
        matches!(
            r.status,
            ProtoStatus::Converged | ProtoStatus::ConvergedRedundant
        ),
        "{:?}",
        r.status
    );
    assert!(residual(s) < 1e-8, "residual {}", residual(s));
}

/// Constrained offset of `chain`: zero residual, the copy fully determined (DOF 0 with the
/// source pinned), then the dimension edited to `d2` and solved.
fn constrained_edit(
    s: &mut Sketch,
    chain: &[EntityId],
    d: f64,
    d2: f64,
    side: Point2,
    opts: &OffsetOptions,
) -> OffsetResult {
    pin_points(s);
    let before = s.constraints().count();
    let res = offset::offset_with(s, chain, d, side, true, opts).unwrap();
    assert!(s.constraints().count() > before);
    assert!(!res.dimensions.is_empty());
    assert!(residual(s) < 1e-9, "residual after offset {}", residual(s));
    assert_eq!(dof(s).unwrap(), 0, "the copy is fully determined");
    offset::set_offset_distance(s, &res.dimensions, d2).unwrap();
    solved(s);
    assert_eq!(dof(s).unwrap(), 0);
    res
}

#[test]
fn constrained_rect_follows_dimension() {
    let mut s = new();
    let r = rect(&mut s, 10.0, 6.0);
    let res = constrained_edit(
        &mut s,
        &r,
        2.0,
        3.0,
        p(-1.0, -1.0),
        &OffsetOptions::default(),
    );
    assert_eq!(res.dimensions.len(), 4);
    let copy = curves(&s, &res.copies);
    assert!(has_line(&copy, p(-3.0, -3.0), p(13.0, -3.0)));
    near(one_area(&s, &res), 16.0 * 12.0, 1e-8);
}

#[test]
fn constrained_rect_inward_follows_dimension() {
    let mut s = new();
    let r = rect(&mut s, 10.0, 6.0);
    let res = constrained_edit(&mut s, &r, 1.0, 1.5, p(5.0, 3.0), &OffsetOptions::default());
    near(one_area(&s, &res), 7.0 * 3.0, 1e-8);
}

#[test]
fn constrained_round_corners_follow_dimension() {
    let mut s = new();
    let r = rect(&mut s, 10.0, 6.0);
    let opts = OffsetOptions {
        corner: CornerStyle::Round,
        ..OffsetOptions::default()
    };
    let res = constrained_edit(&mut s, &r, 2.0, 1.0, p(-1.0, -1.0), &opts);
    near(one_area(&s, &res), 60.0 + 32.0 + PI, 1e-7);
}

#[test]
fn constrained_open_chain_follows_dimension() {
    let mut s = new();
    let ch = poly(&mut s, &[(0.0, 0.0), (4.0, 0.0), (4.0, 3.0)], false);
    let res = constrained_edit(
        &mut s,
        &ch,
        1.0,
        2.5,
        p(2.0, -1.0),
        &OffsetOptions::default(),
    );
    let copy = curves(&s, &res.copies);
    assert!(has_line(&copy, p(0.0, -2.5), p(6.5, -2.5)));
    assert!(has_line(&copy, p(6.5, -2.5), p(6.5, 3.0)));
}

#[test]
fn constrained_chain_with_tangent_arc_follows_dimension() {
    let mut s = new();
    let a = s.add_line(p(0.0, 0.0), p(5.0, 0.0)).unwrap();
    let b = s.add_arc(p(5.0, 2.0), p(5.0, 0.0), p(7.0, 2.0)).unwrap();
    let c = s.add_line(p(7.0, 2.0), p(7.0, 6.0)).unwrap();
    let res = constrained_edit(
        &mut s,
        &[a, b, c],
        1.0,
        0.5,
        p(2.0, -1.0),
        &OffsetOptions::default(),
    );
    let copy = curves(&s, &res.copies);
    near(total_length(&copy), 5.0 + 2.5 * FRAC_PI_2 + 4.0, 1e-8);
}

#[test]
fn constrained_trimmed_chain_follows_dimension() {
    let mut s = new();
    let ch = poly(
        &mut s,
        &[(0.0, 0.0), (5.0, 0.0), (5.0, 1.0), (10.0, 1.0)],
        false,
    );
    let res = constrained_edit(
        &mut s,
        &ch,
        2.0,
        2.5,
        p(1.0, 3.0),
        &OffsetOptions::default(),
    );
    let copy = curves(&s, &res.copies);
    assert!(has_line(&copy, p(0.0, 2.5), p(2.5, 2.5)));
    assert!(has_line(&copy, p(2.5, 2.5), p(2.5, 3.5)));
    assert!(has_line(&copy, p(2.5, 3.5), p(10.0, 3.5)));
}

#[test]
fn constrained_stadium_follows_dimension() {
    let mut s = new();
    let st = stadium(&mut s);
    let res = constrained_edit(
        &mut s,
        &st,
        1.0,
        1.5,
        p(5.0, 6.0),
        &OffsetOptions::default(),
    );
    near(one_area(&s, &res), PI * 3.5 * 3.5 + 2.0 * 3.5 * 10.0, 1e-7);
}

#[test]
fn constrained_circle_follows_dimension() {
    let mut s = new();
    let c = s.add_circle(p(3.0, 4.0), 5.0).unwrap();
    let EntityKind::Circle { center, .. } = s.entity(c).unwrap().kind else {
        panic!()
    };
    s.add_constraint(ConstraintKind::Fix { entity: center })
        .unwrap();
    s.add_constraint(ConstraintKind::Fix { entity: c }).unwrap();
    let res = offset::offset(&mut s, &[c], 2.0, p(20.0, 4.0), true).unwrap();
    assert!(residual(&s) < 1e-9);
    assert_eq!(dof(&s).unwrap(), 0);
    offset::set_offset_distance(&mut s, &res.dimensions, 3.5).unwrap();
    solved(&mut s);
    let Curve2::Circle(k) = curves(&s, &res.copies)[0] else {
        panic!()
    };
    near(k.radius, 8.5, 1e-9);
    assert_eq!(k.center, p(3.0, 4.0));
}

#[test]
fn constrained_copy_follows_its_source_but_unconstrained_does_not() {
    for constrain in [true, false] {
        let mut s = new();
        let r = rect(&mut s, 10.0, 6.0);
        pin_points(&mut s);
        let res = offset::offset(&mut s, &r, 2.0, p(-1.0, -1.0), constrain).unwrap();
        // Move the right edge of the (pinned) source out by 2 mm.
        let right = r[1];
        let EntityKind::Line { start, end } = s.entity(right).unwrap().kind else {
            panic!()
        };
        let fixes: Vec<_> = s
            .constraints()
            .filter(|(_, c)| {
                matches!(c.kind, ConstraintKind::Fix { entity } if entity == start || entity == end)
            })
            .map(|(i, _)| i)
            .collect();
        for f in fixes {
            s.remove_constraint(f).unwrap();
        }
        for pt in [start, end] {
            let EntityKind::Point { x, .. } = s.entity(pt).unwrap().kind else {
                panic!()
            };
            s.params_mut().set(x, 12.0);
        }
        for pt in [start, end] {
            s.add_constraint(ConstraintKind::Fix { entity: pt })
                .unwrap();
        }
        let _ = solve(&mut s, &ProtoOptions::default()).unwrap();
        let copy = curves(&s, &res.copies);
        let right_x = copy
            .iter()
            .filter_map(|c| match c {
                Curve2::Line(l) if (l.a.x - l.b.x).abs() < 1e-9 && l.a.x > 5.0 => Some(l.a.x),
                _ => None,
            })
            .next()
            .unwrap();
        if constrain {
            assert!((right_x - 14.0).abs() < 1e-6, "follows: {right_x}");
        } else {
            assert!((right_x - 12.0).abs() < 1e-9, "independent: {right_x}");
        }
    }
}

#[test]
fn constrain_false_adds_no_constraints() {
    let mut s = new();
    let r = rect(&mut s, 10.0, 6.0);
    let before = s.constraints().count();
    let res = offset::offset(&mut s, &r, 2.0, p(-1.0, -1.0), false).unwrap();
    assert_eq!(s.constraints().count(), before);
    assert!(res.dimensions.is_empty() && res.constraints_added.is_empty());
}
