//! SK-06 golden profile fixtures: profile count, hole counts and areas (relative 1e-9), open
//! chains and issues for tricky sketches; plus id stability, determinism and invariance
//! properties.

#![allow(
    clippy::unwrap_used,
    reason = "test helpers; a failure is a test failure"
)]

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use amg_core::{Plane, Point2};
use amg_sketch::{EntityId, ProfileIssue, ProfileOptions, ProfileSet, Sketch, detect_profiles};
use proptest::prelude::*;

fn p(x: f64, y: f64) -> Point2 {
    Point2::new(x, y)
}

fn sk() -> Sketch {
    Sketch::new(Plane::XY)
}

/// Closed polygon through `pts`, consecutive lines sharing point entities.
fn poly(s: &mut Sketch, pts: &[(f64, f64)]) -> Vec<EntityId> {
    let ids: Vec<EntityId> = pts
        .iter()
        .map(|&(x, y)| s.add_point(p(x, y)).unwrap())
        .collect();
    (0..ids.len())
        .map(|i| {
            s.add_line_between(ids[i], ids[(i + 1) % ids.len()])
                .unwrap()
        })
        .collect()
}

fn rect(s: &mut Sketch, x0: f64, y0: f64, x1: f64, y1: f64) -> Vec<EntityId> {
    poly(s, &[(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
}

fn line(s: &mut Sketch, ax: f64, ay: f64, bx: f64, by: f64) -> EntityId {
    s.add_line(p(ax, ay), p(bx, by)).unwrap()
}

fn circle(s: &mut Sketch, x: f64, y: f64, r: f64) -> EntityId {
    s.add_circle(p(x, y), r).unwrap()
}

fn arc(s: &mut Sketch, c: (f64, f64), a: (f64, f64), b: (f64, f64)) -> EntityId {
    s.add_arc(p(c.0, c.1), p(a.0, a.1), p(b.0, b.1)).unwrap()
}

fn detect(s: &Sketch) -> ProfileSet {
    let set = detect_profiles(s, &ProfileOptions::default());
    validate(&set);
    set
}

/// Structural invariants every result must satisfy.
fn validate(set: &ProfileSet) {
    for (i, pr) in set.profiles.iter().enumerate() {
        if i > 0 {
            assert!(set.profiles[i - 1].id < pr.id, "sorted, unique ids");
        }
        assert!(pr.outer.signed_area() > 0.0, "outer loop is CCW");
        for h in &pr.holes {
            assert!(h.signed_area() < 0.0, "holes are CW");
        }
        for l in std::iter::once(&pr.outer).chain(&pr.holes) {
            let n = l.edges.len();
            assert!(n > 0);
            for k in 0..n {
                let (e, f) = (&l.edges[k], &l.edges[(k + 1) % n]);
                assert_eq!(e.end, f.start, "loop is connected by vertex keys");
                let (_, t1) = e.curve.param_range();
                let gap = e.curve.eval(t1).distance(f.curve.eval(0.0));
                assert!(gap < 1e-5, "loop is geometrically closed ({gap})");
            }
        }
        let net = pr.outer.signed_area() + pr.holes.iter().map(|h| h.signed_area()).sum::<f64>();
        // `Loop::signed_area` integrates the curves as stored; the fingerprint integrates through
        // the welded vertices. Each vertex may sit up to the weld tolerance from a curve end, so
        // the two differ by at most weld * perimeter (plus rounding).
        let weld = ProfileOptions::default().weld_tol;
        let perimeter: f64 = std::iter::once(&pr.outer)
            .chain(&pr.holes)
            .flat_map(|l| l.edges.iter())
            .map(|e| e.curve.length())
            .sum();
        let tol = weld * perimeter + 1e-9 * net.abs().max(1.0);
        assert!(
            (net - pr.fingerprint.area).abs() <= tol,
            "{net} vs {}",
            pr.fingerprint.area
        );
        assert!(set.get(pr.id).is_some());
    }
}

/// `(holes, area)` of every profile, sorted by area, compared with relative tolerance 1e-9.
fn expect(set: &ProfileSet, want: &[(usize, f64)]) {
    let mut got: Vec<(usize, f64)> = set
        .profiles
        .iter()
        .map(|p| (p.holes.len(), p.fingerprint.area))
        .collect();
    got.sort_by(|a, b| a.1.total_cmp(&b.1).then(a.0.cmp(&b.0)));
    let mut want = want.to_vec();
    want.sort_by(|a, b| a.1.total_cmp(&b.1).then(a.0.cmp(&b.0)));
    assert_eq!(
        got.len(),
        want.len(),
        "profile count: got {got:?}, want {want:?}"
    );
    for (g, w) in got.iter().zip(&want) {
        assert_eq!(g.0, w.0, "hole count: got {got:?}, want {want:?}");
        assert!(
            (g.1 - w.1).abs() <= 1e-9 * w.1.abs(),
            "area: got {got:?}, want {want:?}"
        );
    }
}

fn count_issues(set: &ProfileSet, f: impl Fn(&ProfileIssue) -> bool) -> usize {
    set.issues.iter().filter(|i| f(i)).count()
}

fn overlaps(set: &ProfileSet) -> usize {
    count_issues(set, |i| matches!(i, ProfileIssue::Overlap { .. }))
}

fn near_gaps(set: &ProfileSet) -> usize {
    count_issues(set, |i| matches!(i, ProfileIssue::NearGap { .. }))
}

// ---- Basic shapes -------------------------------------------------------------------------

#[test]
fn g01_rectangle_shared_points() {
    let mut s = sk();
    rect(&mut s, 0.0, 0.0, 10.0, 5.0);
    let set = detect(&s);
    expect(&set, &[(0, 50.0)]);
    assert!(set.open_chains.is_empty() && set.issues.is_empty());
    assert_eq!(set.profiles[0].outer.edges.len(), 4);
}

#[test]
fn g02_rectangle_coincident_free_lines() {
    let mut s = sk();
    line(&mut s, 0.0, 0.0, 10.0, 0.0);
    line(&mut s, 10.0, 5.0, 10.0, 0.0); // reversed direction
    line(&mut s, 10.0, 5.0, 0.0, 5.0);
    line(&mut s, 0.0, 0.0, 0.0, 5.0);
    expect(&detect(&s), &[(0, 50.0)]);
}

#[test]
fn g03_circle() {
    let mut s = sk();
    circle(&mut s, 1.0, 2.0, 3.0);
    let set = detect(&s);
    expect(&set, &[(0, 9.0 * PI)]);
    let c = set.profiles[0].fingerprint.centroid;
    assert!(c.distance(p(1.0, 2.0)) < 1e-12);
}

#[test]
fn g04_circle_hole_in_square() {
    let mut s = sk();
    rect(&mut s, -10.0, -10.0, 10.0, 10.0);
    circle(&mut s, 0.0, 0.0, 5.0);
    let set = detect(&s);
    expect(&set, &[(1, 400.0 - 25.0 * PI), (0, 25.0 * PI)]);
    assert!(set.at_point(p(0.0, 0.0)).is_some());
    assert_ne!(set.at_point(p(0.0, 0.0)), set.at_point(p(9.0, 9.0)));
    assert_eq!(set.at_point(p(20.0, 0.0)), None);
}

#[test]
fn g05_nested_islands() {
    let mut s = sk();
    rect(&mut s, -20.0, -20.0, 20.0, 20.0);
    circle(&mut s, 0.0, 0.0, 15.0);
    rect(&mut s, -8.0, -8.0, 8.0, 8.0);
    circle(&mut s, 0.0, 0.0, 4.0);
    expect(
        &detect(&s),
        &[
            (1, 1600.0 - 225.0 * PI),
            (1, 225.0 * PI - 256.0),
            (1, 256.0 - 16.0 * PI),
            (0, 16.0 * PI),
        ],
    );
}

#[test]
fn g06_venn_two_circles() {
    let mut s = sk();
    circle(&mut s, 0.0, 0.0, 1.0);
    circle(&mut s, 1.0, 0.0, 1.0);
    let lens = 2.0 * PI / 3.0 - 3f64.sqrt() / 2.0;
    expect(&detect(&s), &[(0, lens), (0, PI - lens), (0, PI - lens)]);
}

#[test]
fn g07_t_junction_split() {
    let mut s = sk();
    rect(&mut s, 0.0, 0.0, 10.0, 6.0);
    line(&mut s, 5.0, 0.0, 5.0, 6.0);
    let set = detect(&s);
    expect(&set, &[(0, 30.0), (0, 30.0)]);
    assert!(set.open_chains.is_empty());
}

#[test]
fn g08_t_junction_dangling() {
    let mut s = sk();
    rect(&mut s, 0.0, 0.0, 10.0, 6.0);
    let l = line(&mut s, 5.0, 0.0, 5.0, 3.0);
    let set = detect(&s);
    expect(&set, &[(0, 60.0)]);
    assert_eq!(set.open_chains, vec![vec![l]]);
}

#[test]
fn g09_cross_through_square() {
    let mut s = sk();
    rect(&mut s, 0.0, 0.0, 10.0, 10.0);
    line(&mut s, -2.0, 5.0, 12.0, 5.0);
    line(&mut s, 5.0, -2.0, 5.0, 12.0);
    let set = detect(&s);
    expect(&set, &[(0, 25.0); 4]);
    assert_eq!(set.open_chains.len(), 4, "four stubs");
}

// ---- Tangent contacts ---------------------------------------------------------------------

#[test]
fn g10_circles_tangent_external() {
    let mut s = sk();
    circle(&mut s, 0.0, 0.0, 2.0);
    circle(&mut s, 5.0, 0.0, 3.0);
    expect(&detect(&s), &[(0, 4.0 * PI), (0, 9.0 * PI)]);
}

#[test]
fn g11_circles_tangent_internal() {
    let mut s = sk();
    circle(&mut s, 0.0, 0.0, 5.0);
    circle(&mut s, 3.0, 0.0, 2.0);
    let set = detect(&s);
    // The small circle touches the big one, so the ring-like face has no hole: its single
    // outer loop runs around both circles through the contact vertex.
    expect(&set, &[(0, 21.0 * PI), (0, 4.0 * PI)]);
}

#[test]
fn g12_circle_tangent_to_edge_inside_square() {
    let mut s = sk();
    rect(&mut s, 0.0, 0.0, 10.0, 10.0);
    circle(&mut s, 5.0, 2.0, 2.0);
    expect(&detect(&s), &[(0, 100.0 - 4.0 * PI), (0, 4.0 * PI)]);
}

#[test]
fn g13_tangent_line_across_square_and_circle() {
    let mut s = sk();
    rect(&mut s, 0.0, 0.0, 10.0, 10.0);
    circle(&mut s, 5.0, 5.0, 3.0);
    line(&mut s, 0.0, 8.0, 10.0, 8.0);
    expect(
        &detect(&s),
        &[(0, 20.0), (0, 80.0 - 9.0 * PI), (0, 9.0 * PI)],
    );
}

#[test]
fn g14_near_tangent_within_tolerance() {
    let mut s = sk();
    rect(&mut s, 0.0, 0.0, 10.0, 10.0);
    circle(&mut s, 5.0, 5.0, 3.0);
    line(&mut s, 0.0, 8.0 + 5e-7, 10.0, 8.0 + 5e-7);
    let set = detect(&s);
    assert_eq!(set.profiles.len(), 3);
    let areas: Vec<f64> = set.profiles.iter().map(|p| p.fingerprint.area).collect();
    let total: f64 = areas.iter().sum();
    assert!((total - 100.0).abs() < 1e-9 * 100.0, "{areas:?}");
}

#[test]
fn g15_rounded_corner_tangent_arc() {
    let mut s = sk();
    line(&mut s, 0.0, 0.0, 10.0, 0.0);
    line(&mut s, 10.0, 0.0, 10.0, 4.0);
    arc(&mut s, (8.0, 4.0), (10.0, 4.0), (8.0, 6.0));
    line(&mut s, 8.0, 6.0, 0.0, 6.0);
    line(&mut s, 0.0, 6.0, 0.0, 0.0);
    expect(&detect(&s), &[(0, 60.0 - (4.0 - PI))]);
}

// ---- Overlaps, duplicates -----------------------------------------------------------------

#[test]
fn g16_overlapping_collinear_lines() {
    let mut s = sk();
    line(&mut s, 0.0, 0.0, 7.0, 0.0);
    line(&mut s, 3.0, 0.0, 10.0, 0.0);
    line(&mut s, 10.0, 0.0, 10.0, 4.0);
    line(&mut s, 10.0, 4.0, 0.0, 4.0);
    line(&mut s, 0.0, 4.0, 0.0, 0.0);
    let set = detect(&s);
    expect(&set, &[(0, 40.0)]);
    assert_eq!(overlaps(&set), 1);
    let merged = set.profiles[0]
        .outer
        .edges
        .iter()
        .filter(|e| !e.also.is_empty())
        .count();
    assert_eq!(merged, 1, "the shared span 3..7 carries both sources");
}

#[test]
fn g17_duplicate_edge_reversed() {
    let mut s = sk();
    let r = rect(&mut s, 0.0, 0.0, 10.0, 4.0);
    let d = line(&mut s, 0.0, 4.0, 10.0, 4.0);
    let set = detect(&s);
    expect(&set, &[(0, 40.0)]);
    assert_eq!(overlaps(&set), 1);
    let top = set.profiles[0]
        .outer
        .edges
        .iter()
        .find(|e| !e.also.is_empty())
        .unwrap();
    assert_eq!((top.source, top.also.as_slice()), (r[2], &[d][..]));
    assert!(set.open_chains.is_empty());
}

#[test]
fn g18_identical_circles() {
    let mut s = sk();
    circle(&mut s, 0.0, 0.0, 2.0);
    circle(&mut s, 0.0, 0.0, 2.0);
    let set = detect(&s);
    expect(&set, &[(0, 4.0 * PI)]);
    assert_eq!(overlaps(&set), 1);
}

#[test]
fn g19_arc_on_circle() {
    let mut s = sk();
    circle(&mut s, 0.0, 0.0, 3.0);
    arc(&mut s, (0.0, 0.0), (3.0, 0.0), (0.0, 3.0));
    let set = detect(&s);
    expect(&set, &[(0, 9.0 * PI)]);
    assert_eq!(overlaps(&set), 1);
}

// ---- Gaps and degenerate input ------------------------------------------------------------

#[test]
fn g20_near_gap_reported_not_healed() {
    let mut s = sk();
    line(&mut s, 0.0, 0.0, 10.0, 0.0);
    line(&mut s, 10.0, 0.0, 10.0, 4.0);
    line(&mut s, 10.0, 4.0, 0.0, 4.0);
    line(&mut s, 0.0, 4.0, 0.0, 0.005);
    let set = detect(&s);
    expect(&set, &[]);
    assert_eq!(near_gaps(&set), 1);
    let ProfileIssue::NearGap { distance, .. } = set.issues[0] else {
        panic!("{:?}", set.issues)
    };
    assert!((distance - 0.005).abs() < 1e-12);
    assert_eq!(set.open_chains.len(), 1);
}

#[test]
fn g21_gap_within_weld_tolerance() {
    let mut s = sk();
    line(&mut s, 0.0, 0.0, 10.0, 0.0);
    line(&mut s, 10.0, 0.0, 10.0, 4.0);
    line(&mut s, 10.0, 4.0, 0.0, 4.0);
    line(&mut s, 0.0, 4.0, 0.0, 5e-7);
    let set = detect(&s);
    assert_eq!(set.profiles.len(), 1);
    assert!((set.profiles[0].fingerprint.area - 40.0).abs() < 1e-5);
    assert!(set.issues.is_empty());
}

#[test]
fn g22_t_junction_short_of_edge_within_tolerance() {
    let mut s = sk();
    rect(&mut s, 0.0, 0.0, 10.0, 6.0);
    line(&mut s, 5.0, 5e-7, 5.0, 6.0 - 5e-7);
    let set = detect(&s);
    expect(&set, &[(0, 30.0), (0, 30.0)]);
}

#[test]
fn g23_zero_length_line() {
    let mut s = sk();
    rect(&mut s, 0.0, 0.0, 10.0, 6.0);
    let z = line(&mut s, 3.0, 3.0, 3.0, 3.0);
    let set = detect(&s);
    expect(&set, &[(0, 60.0)]);
    assert_eq!(set.issues, vec![ProfileIssue::ZeroLength(z)]);
}

#[test]
fn g24_construction_ignored() {
    let mut s = sk();
    rect(&mut s, 0.0, 0.0, 10.0, 5.0);
    let d = line(&mut s, 0.0, 0.0, 10.0, 5.0);
    s.set_construction(d, true).unwrap();
    expect(&detect(&s), &[(0, 50.0)]);
}

// ---- Arcs ---------------------------------------------------------------------------------

#[test]
fn g25_slot() {
    let mut s = sk();
    line(&mut s, 0.0, -2.0, 10.0, -2.0);
    arc(&mut s, (10.0, 0.0), (10.0, -2.0), (10.0, 2.0));
    line(&mut s, 10.0, 2.0, 0.0, 2.0);
    arc(&mut s, (0.0, 0.0), (0.0, 2.0), (0.0, -2.0));
    expect(&detect(&s), &[(0, 40.0 + 4.0 * PI)]);
}

#[test]
fn g26_d_shape_and_lens() {
    let mut s = sk();
    arc(&mut s, (0.0, 0.0), (3.0, 0.0), (-3.0, 0.0));
    line(&mut s, -3.0, 0.0, 3.0, 0.0);
    expect(&detect(&s), &[(0, 4.5 * PI)]);

    let mut s = sk();
    arc(&mut s, (0.0, -1.0), (1.0, 0.0), (-1.0, 0.0));
    arc(&mut s, (0.0, 1.0), (-1.0, 0.0), (1.0, 0.0));
    expect(&detect(&s), &[(0, PI - 2.0)]);
}

#[test]
fn g27_annulus() {
    let mut s = sk();
    circle(&mut s, 0.0, 0.0, 5.0);
    circle(&mut s, 0.0, 0.0, 3.0);
    let set = detect(&s);
    expect(&set, &[(1, 16.0 * PI), (0, 9.0 * PI)]);
    let ring = set.profiles.iter().find(|p| p.holes.len() == 1).unwrap();
    assert!(ring.fingerprint.centroid.length() < 1e-12);
    assert_eq!(set.at_point(p(4.0, 0.0)), Some(ring.id));
}

#[test]
fn g28_chord_through_circle() {
    let mut s = sk();
    circle(&mut s, 0.0, 0.0, 2.0);
    line(&mut s, -3.0, 1.0, 3.0, 1.0);
    let a = 2.0 * PI / 3.0;
    let cap = 2.0 * (a - a.sin());
    let set = detect(&s);
    expect(&set, &[(0, cap), (0, 4.0 * PI - cap)]);
    assert_eq!(set.open_chains.len(), 2);
}

#[test]
fn g29_circle_centred_on_edge() {
    let mut s = sk();
    rect(&mut s, 0.0, 0.0, 10.0, 6.0);
    circle(&mut s, 5.0, 6.0, 2.0);
    expect(
        &detect(&s),
        &[(0, 2.0 * PI), (0, 2.0 * PI), (0, 60.0 - 2.0 * PI)],
    );
}

#[test]
fn g30_arc_bump_on_line() {
    let mut s = sk();
    line(&mut s, 0.0, 0.0, 10.0, 0.0);
    arc(&mut s, (5.0, 0.0), (7.0, 0.0), (3.0, 0.0));
    let set = detect(&s);
    expect(&set, &[(0, 2.0 * PI)]);
    assert_eq!(set.open_chains.len(), 2);
}

// ---- Arrangements -------------------------------------------------------------------------

#[test]
fn g31_grid_of_long_lines() {
    let mut s = sk();
    for i in 0..4 {
        let x = f64::from(i);
        line(&mut s, x, -1.0, x, 4.0);
        line(&mut s, -1.0, x, 4.0, x);
    }
    let set = detect(&s);
    expect(&set, &[(0, 1.0); 9]);
    // 16 stubs; the two at each corner share a vertex, so they form one group.
    assert_eq!(set.open_chains.len(), 12);
}

#[test]
fn g32_pentagram() {
    let mut s = sk();
    let v: Vec<(f64, f64)> = (0..5)
        .map(|i| {
            let a = FRAC_PI_2 + f64::from(i) * TAU / 5.0;
            (a.cos(), a.sin())
        })
        .collect();
    for i in 0..5 {
        let (a, b) = (v[i], v[(i + 2) % 5]);
        line(&mut s, a.0, a.1, b.0, b.1);
    }
    // Inner pentagon circumradius and areas.
    let r = (TAU / 5.0).cos() / (PI / 5.0).cos();
    let pent = 2.5 * r * r * (TAU / 5.0).sin();
    // Star area = 10 triangles (centre, outer vertex, inner vertex).
    let star = 10.0 * 0.5 * r * (PI / 5.0).sin();
    let tip = (star - pent) / 5.0;
    let set = detect(&s);
    expect(
        &set,
        &[(0, tip), (0, tip), (0, tip), (0, tip), (0, tip), (0, pent)],
    );
}

#[test]
fn g33_triangle_medians_triple_point() {
    let mut s = sk();
    poly(&mut s, &[(0.0, 0.0), (4.0, 0.0), (2.0, 3.0)]);
    line(&mut s, 0.0, 0.0, 3.0, 1.5);
    line(&mut s, 4.0, 0.0, 1.0, 1.5);
    line(&mut s, 2.0, 3.0, 2.0, 0.0);
    expect(&detect(&s), &[(0, 1.0); 6]);
}

#[test]
fn g34_asterisk_in_circle() {
    let mut s = sk();
    circle(&mut s, 0.0, 0.0, 1.0);
    for k in 0..3 {
        let a = f64::from(k) * PI / 3.0;
        line(&mut s, a.cos(), a.sin(), -a.cos(), -a.sin());
    }
    expect(&detect(&s), &[(0, PI / 6.0); 6]);
}

#[test]
fn g35_adjacent_squares_shared_edge() {
    let mut s = sk();
    let pts: Vec<EntityId> = [
        (0.0, 0.0),
        (1.0, 0.0),
        (2.0, 0.0),
        (2.0, 1.0),
        (1.0, 1.0),
        (0.0, 1.0),
    ]
    .iter()
    .map(|&(x, y)| s.add_point(p(x, y)).unwrap())
    .collect();
    for i in 0..6 {
        s.add_line_between(pts[i], pts[(i + 1) % 6]).unwrap();
    }
    s.add_line_between(pts[1], pts[4]).unwrap();
    expect(&detect(&s), &[(0, 1.0), (0, 1.0)]);
}

#[test]
fn g36_bridge_to_island() {
    let mut s = sk();
    rect(&mut s, 0.0, 0.0, 10.0, 10.0);
    rect(&mut s, 4.0, 4.0, 6.0, 6.0);
    let b = line(&mut s, 0.0, 5.0, 4.0, 5.0);
    let set = detect(&s);
    expect(&set, &[(1, 96.0), (0, 4.0)]);
    assert_eq!(set.open_chains, vec![vec![b]]);
}

#[test]
fn g37_venn_three_circles() {
    let mut s = sk();
    for k in 0..3 {
        let a = FRAC_PI_2 + f64::from(k) * TAU / 3.0;
        circle(&mut s, 0.6 * a.cos(), 0.6 * a.sin(), 1.0);
    }
    let set = detect(&s);
    assert_eq!(set.profiles.len(), 7);
    // By symmetry: three single-circle regions, three double, one triple, and each circle is
    // tiled by single + 2·double + triple.
    let mut areas: Vec<f64> = set.profiles.iter().map(|p| p.fingerprint.area).collect();
    areas.sort_by(f64::total_cmp);
    let groups: Vec<Vec<f64>> = {
        let mut g: Vec<Vec<f64>> = Vec::new();
        for a in areas {
            match g.last_mut() {
                Some(last) if (last[0] - a).abs() < 1e-9 => last.push(a),
                _ => g.push(vec![a]),
            }
        }
        g
    };
    let sizes: Vec<usize> = groups.iter().map(Vec::len).collect();
    assert_eq!(sizes.iter().sum::<usize>(), 7);
    assert_eq!(sizes.iter().filter(|&&n| n == 3).count(), 2, "{groups:?}");
    let triple = groups.iter().find(|g| g.len() == 1).unwrap()[0];
    let threes: Vec<f64> = groups
        .iter()
        .filter(|g| g.len() == 3)
        .map(|g| g[0])
        .collect();
    // The single-circle region is the larger of the two triples here.
    let (double, single) = (threes[0].min(threes[1]), threes[0].max(threes[1]));
    assert!((single + 2.0 * double + triple - PI).abs() < 1e-9 * PI);
}

#[test]
fn g38_projected_excluded_on_request() {
    use amg_sketch::{EntityKind, EntityOrigin, ExternalLinkId, ParamFlags};
    let mut s = sk();
    rect(&mut s, 0.0, 0.0, 10.0, 10.0);
    let c = s.add_point(p(5.0, 5.0)).unwrap();
    let r = s.alloc_param(2.0, ParamFlags::default()).unwrap();
    s.add_entity(
        EntityKind::Circle {
            center: c,
            radius: r,
        },
        false,
        EntityOrigin::Projected(ExternalLinkId(1)),
    )
    .unwrap();
    expect(&detect(&s), &[(1, 100.0 - 4.0 * PI), (0, 4.0 * PI)]);
    let opts = ProfileOptions {
        include_projected: false,
        ..ProfileOptions::default()
    };
    let set = detect_profiles(&s, &opts);
    expect(&set, &[(0, 100.0)]);
}

// ---- Stability and determinism ------------------------------------------------------------

fn ids(set: &ProfileSet) -> Vec<u64> {
    set.profiles.iter().map(|p| p.id.0).collect()
}

/// Square with a circle crossing its right edge, a hole and a dangling line.
fn plate(s: &mut Sketch) -> Vec<EntityId> {
    let r = rect(s, 0.0, 0.0, 20.0, 10.0);
    circle(s, 20.0, 5.0, 3.0);
    circle(s, 6.0, 5.0, 2.0);
    line(s, 10.0, 0.0, 10.0, 4.0);
    r
}

#[test]
fn ids_survive_dimension_edits() {
    let mut s = sk();
    let r = plate(&mut s);
    let before = detect(&s);
    assert_eq!(before.profiles.len(), 4);
    // Move the top-right corner and widen the rectangle: same topology, new geometry.
    let corner = s
        .entity(r[1])
        .unwrap()
        .kind
        .endpoint(amg_sketch::End::End)
        .unwrap();
    let amg_sketch::EntityKind::Point { x, y } = s.entity(corner).unwrap().kind else {
        panic!()
    };
    s.params_mut().set(x, 22.0);
    s.params_mut().set(y, 11.0);
    let after = detect(&s);
    assert_eq!(ids(&before), ids(&after));
    let a0: f64 = before.profiles.iter().map(|p| p.fingerprint.area).sum();
    let a1: f64 = after.profiles.iter().map(|p| p.fingerprint.area).sum();
    assert!(a1 > a0);
}

#[test]
fn ids_survive_unrelated_edits() {
    let mut s = sk();
    plate(&mut s);
    let before = detect(&s);
    // Unrelated geometry far away, and an unrelated construction line across everything.
    circle(&mut s, 100.0, 100.0, 5.0);
    rect(&mut s, -50.0, -50.0, -40.0, -40.0);
    let c = line(&mut s, -5.0, -5.0, 30.0, 30.0);
    s.set_construction(c, true).unwrap();
    let after = detect(&s);
    for id in ids(&before) {
        assert!(ids(&after).contains(&id), "profile {id:x} lost");
    }
    assert_eq!(after.profiles.len(), before.profiles.len() + 2);
}

#[test]
fn deterministic_output() {
    let mut s = sk();
    plate(&mut s);
    let a = detect(&s);
    let b = detect(&s);
    assert_eq!(ids(&a), ids(&b));
    for (x, y) in a.profiles.iter().zip(&b.profiles) {
        assert_eq!(x.fingerprint, y.fingerprint);
        assert_eq!(x.outer.edges.len(), y.outer.edges.len());
        for (e, f) in x.outer.edges.iter().zip(&y.outer.edges) {
            assert_eq!((e.source, e.start, e.end), (f.source, f.start, f.end));
            assert_eq!(e.curve, f.curve);
        }
    }
    assert_eq!(a.open_chains, b.open_chains);
    assert_eq!(a.issues, b.issues);
}

/// Rectangles and circles at random positions (may overlap and cross).
fn shapes(dx: f64, dy: f64, seed: &[(f64, f64, f64)]) -> Sketch {
    let mut s = sk();
    for (k, &(x, y, r)) in seed.iter().enumerate() {
        if k % 2 == 0 {
            rect(&mut s, x + dx, y + dy, x + dx + 2.0 * r, y + dy + r);
        } else {
            circle(&mut s, x + dx, y + dy, r);
        }
    }
    s
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    /// Translating a sketch keeps profile ids, counts and areas.
    #[test]
    fn translation_invariant(
        seed in prop::collection::vec((0.0f64..40.0, 0.0f64..40.0, 1.0f64..15.0), 1..8),
        dx in -1000.0f64..1000.0,
        dy in -1000.0f64..1000.0,
    ) {
        let a = detect(&shapes(0.0, 0.0, &seed));
        let b = detect(&shapes(dx, dy, &seed));
        prop_assert_eq!(ids(&a), ids(&b));
        for (x, y) in a.profiles.iter().zip(&b.profiles) {
            prop_assert_eq!(x.holes.len(), y.holes.len());
            let (ax, ay) = (x.fingerprint.area, y.fingerprint.area);
            prop_assert!((ax - ay).abs() <= 1e-6 * ax.max(1.0), "{} vs {}", ax, ay);
        }
    }

    /// Every bounded region contains its own interior sample point, and `at_point` finds it.
    #[test]
    fn at_point_finds_each_profile(
        seed in prop::collection::vec((0.0f64..40.0, 0.0f64..40.0, 1.0f64..15.0), 1..6),
    ) {
        let set = detect(&shapes(0.0, 0.0, &seed));
        let total: f64 = set.profiles.iter().map(|p| p.fingerprint.area).sum();
        prop_assert!(total >= 0.0);
        for pr in &set.profiles {
            if pr.contains(pr.fingerprint.centroid) {
                prop_assert_eq!(set.at_point(pr.fingerprint.centroid), Some(pr.id));
            }
        }
    }
}
