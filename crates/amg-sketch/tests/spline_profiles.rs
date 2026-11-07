//! SK-23: profile detection on spline boundaries.
//!
//! - The MESH-14 case: an ellipse (semi-axes 16 and 8 mm) as two cubic fit splines of 116 knot
//!   spans each meeting end to end, detected as one profile; with `AMG_STRICT_TIMING=1` in
//!   under 50 ms (it took 264 s before SK-23).
//! - A closed loop of 100 spline edges: one profile; under 100 ms with `AMG_STRICT_TIMING=1`.
//! - The spline self-intersector against a polyline oracle (loops, sharp corners, closed
//!   curves, random control splines), and the pruned spline `bbox` / `closest` against dense
//!   sampling.
//!
//! Without `AMG_STRICT_TIMING=1` the timing tests only check a loose bound (shared runners).

#![allow(
    clippy::unwrap_used,
    clippy::cast_precision_loss,
    reason = "test code; a failure is a test failure"
)]

use std::f64::consts::{PI, TAU};
use std::time::{Duration, Instant};

use amg_core::{Plane, Point2};
use amg_sketch::geom::fit_spline;
use amg_sketch::{BSpline2, ProfileIssue, ProfileOptions, ProfileSet, Sketch, detect_profiles};

fn strict() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}

/// Best of three runs of `detect_profiles`.
fn timed(s: &Sketch) -> (ProfileSet, Duration) {
    let opts = ProfileOptions::default();
    let mut best = Duration::MAX;
    let mut set = detect_profiles(s, &opts);
    for _ in 0..3 {
        let t = Instant::now();
        set = detect_profiles(s, &opts);
        best = best.min(t.elapsed());
    }
    (set, best)
}

fn add(s: &mut Sketch, b: &BSpline2) {
    s.add_spline(
        b.degree(),
        b.poles(),
        Some(b.weights()),
        Some(b.knots().to_vec()),
    )
    .unwrap();
}

/// Half `k` (0: upper, 1: lower) of the ellipse with semi-axes `a`, `b`, as a cubic fit
/// spline through `n + 1` points (`n` knot spans).
fn ellipse_half(a: f64, b: f64, n: usize, k: usize) -> BSpline2 {
    let pts: Vec<Point2> = (0..=n)
        .map(|i| {
            let t = PI * (k as f64 + i as f64 / n as f64);
            Point2::new(a * t.cos(), b * t.sin())
        })
        .collect();
    fit_spline(&pts, None, None).unwrap()
}

/// The MESH-14 tilted-ellipse section: two splines of 116 spans each.
fn two_spline_ellipse(reverse_second: bool) -> Sketch {
    let mut s = Sketch::new(Plane::XY);
    add(&mut s, &ellipse_half(16.0, 8.0, 116, 0));
    let lower = ellipse_half(16.0, 8.0, 116, 1);
    add(
        &mut s,
        &if reverse_second {
            lower.reversed()
        } else {
            lower
        },
    );
    s
}

/// A wavy closed loop of `n` cubic fit splines (five points each) sharing their end points.
fn spline_loop(n: usize) -> Sketch {
    let mut s = Sketch::new(Plane::XY);
    let pt = |th: f64| {
        let r = 50.0 + 3.0 * (7.0 * th).sin();
        Point2::new(r * th.cos(), r * th.sin())
    };
    for i in 0..n {
        let pts: Vec<Point2> = (0..=4)
            .map(|k| pt(TAU * (i as f64 + k as f64 / 4.0) / n as f64))
            .collect();
        s.add_spline_fit(&pts, None, None).unwrap();
    }
    s
}

#[test]
fn ellipse_of_two_splines_is_one_profile_either_direction() {
    let exact = PI * 16.0 * 8.0;
    for reverse in [false, true] {
        let set = detect_profiles(&two_spline_ellipse(reverse), &ProfileOptions::default());
        assert_eq!(set.profiles.len(), 1, "reverse {reverse}");
        assert!(set.open_chains.is_empty(), "{:?}", set.open_chains);
        assert!(
            !set.issues
                .iter()
                .any(|i| matches!(i, ProfileIssue::SelfIntersection(_))),
            "{:?}",
            set.issues
        );
        let p = &set.profiles[0];
        // The shared ends are joins: two edges, no extra vertices.
        assert_eq!(p.outer.edges.len(), 2, "reverse {reverse}");
        assert!(p.holes.is_empty());
        assert!(
            (p.fingerprint.area - exact).abs() / exact < 1e-6,
            "{}",
            p.fingerprint.area
        );
    }
}

#[test]
fn loop_of_100_splines_is_one_profile() {
    let set = detect_profiles(&spline_loop(100), &ProfileOptions::default());
    assert_eq!(set.profiles.len(), 1);
    assert!(set.open_chains.is_empty());
    assert_eq!(set.profiles[0].outer.edges.len(), 100);
    assert!(set.issues.is_empty(), "{:?}", set.issues);
}

#[test]
fn timing_two_spline_ellipse() {
    let (set, t) = timed(&two_spline_ellipse(false));
    assert_eq!(set.profiles.len(), 1);
    let bound = if strict() {
        Duration::from_millis(50)
    } else {
        Duration::from_secs(2)
    };
    println!("two-spline ellipse (2 x 116 spans): {t:?}");
    assert!(t < bound, "{t:?} (bound {bound:?})");
}

#[test]
fn timing_hundred_spline_loop() {
    let (set, t) = timed(&spline_loop(100));
    assert_eq!(set.profiles.len(), 1);
    let bound = if strict() {
        Duration::from_millis(100)
    } else {
        Duration::from_secs(4)
    };
    println!("100-spline loop: {t:?}");
    assert!(t < bound, "{t:?} (bound {bound:?})");
}

// ---- Self-intersections ----------------------------------------------------------------------

/// Proper crossings of the polyline through `n + 1` points of `c` with itself (non-adjacent
/// segments only).
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
    let closed = pts[0].distance(pts[n]) < 1e-9;
    let mut k = 0;
    for i in 0..n {
        for j in i + 2..n {
            if closed && i == 0 && j == n - 1 {
                continue;
            }
            if cross(pts[i], pts[i + 1], pts[j], pts[j + 1]) {
                k += 1;
            }
        }
    }
    k
}

/// Hits are on the curve at both parameters, ordered, and the count matches the oracle.
fn check_self(c: &BSpline2, expected: usize, what: &str) {
    let hits = c.self_intersections(1e-6);
    assert_eq!(hits.len(), expected, "{what}: {hits:?}");
    for &(t1, t2, q) in &hits {
        assert!(t1 < t2, "{what}");
        assert!(c.eval(t1).distance(q) < 1e-6, "{what}");
        assert!(c.eval(t2).distance(q) < 1e-6, "{what}");
    }
}

#[test]
fn self_intersections_of_loops_corners_and_closed_curves() {
    // Prolate cycloid: one crossing per loop.
    let pts: Vec<Point2> = (0..=120)
        .map(|i| {
            let t = -PI + 6.0 * PI * f64::from(i) / 120.0;
            Point2::new(t - 2.0 * t.sin(), -2.0 * t.cos())
        })
        .collect();
    let c = fit_spline(&pts, None, None).unwrap();
    let k = brute_self_crossings(&c, 6000);
    assert_eq!(k, 3, "test geometry loops three times");
    check_self(&c, k, "cycloid");

    // Pentagram as a closed degree-1 spline: five crossings, sharp corners, a corner seam.
    let star: Vec<Point2> = (0..=5)
        .map(|i| {
            let a = PI / 2.0 + 2.0 * TAU * f64::from(i % 5) / 5.0;
            Point2::new(10.0 * a.cos(), 10.0 * a.sin())
        })
        .collect();
    let knots = [0.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 5.0].to_vec();
    let c = BSpline2::new(1, knots, star, vec![1.0; 6]).unwrap();
    check_self(&c, 5, "pentagram");

    // A smooth closed fit spline and a closed teardrop (corner at the seam): no crossings.
    let circ: Vec<Point2> = (0..=24)
        .map(|i| {
            let a = TAU * f64::from(i) / 24.0;
            Point2::new(5.0 * a.cos(), 5.0 * a.sin())
        })
        .collect();
    check_self(&fit_spline(&circ, None, None).unwrap(), 0, "closed circle");
    let drop = [(0.0, 0.0), (6.0, 3.0), (8.0, 0.0), (6.0, -3.0), (0.0, 0.0)];
    let drop: Vec<Point2> = drop.iter().map(|&(x, y)| Point2::new(x, y)).collect();
    check_self(&fit_spline(&drop, None, None).unwrap(), 0, "teardrop");
}

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }

    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.next()
    }
}

/// A random clamped spline of degree 1..=5 with 4..=11 poles, rational every third case.
fn random_spline(g: &mut Rng, case: usize) -> BSpline2 {
    let degree = 1 + case % 5;
    let n = degree + 1 + (g.next() * 7.0) as usize;
    let poles: Vec<Point2> = (0..n)
        .map(|_| Point2::new(g.range(-10.0, 10.0), g.range(-10.0, 10.0)))
        .collect();
    let weights: Vec<f64> = (0..n)
        .map(|_| {
            if case.is_multiple_of(3) {
                g.range(0.3, 3.0)
            } else {
                1.0
            }
        })
        .collect();
    let inner = n - degree - 1;
    let mut knots = vec![0.0; degree + 1];
    let mut k = 0.0;
    for _ in 0..inner {
        k += g.range(0.2, 1.0);
        knots.push(k);
    }
    k += g.range(0.2, 1.0);
    knots.extend(std::iter::repeat_n(k, degree + 1));
    BSpline2::new(degree, knots, poles, weights).unwrap()
}

#[test]
fn self_intersections_of_random_splines_match_a_polyline_oracle() {
    let mut g = Rng(0x005e_ed23);
    let mut checked = 0;
    let mut crossings = 0;
    for case in 0..200 {
        let c = random_spline(&mut g, case);
        // Skip cases where the oracle itself is unsure (near-tangent or near-vertex contacts).
        let k = brute_self_crossings(&c, 1500);
        if k != brute_self_crossings(&c, 3000) {
            continue;
        }
        check_self(&c, k, &format!("case {case}"));
        checked += 1;
        crossings += k;
    }
    assert!(checked >= 150, "{checked}");
    assert!(crossings >= 50, "{crossings}");
}

#[test]
fn pruned_bbox_and_closest_match_dense_sampling() {
    let mut g = Rng(0xb0c5);
    for case in 0..100 {
        let c = random_spline(&mut g, case);
        let n = 20_000;
        // Dense samples plus the knots (corners of degree-1 and C0 splines).
        let mut pts: Vec<Point2> = (0..=n)
            .map(|i| c.eval(c.span() * f64::from(i) / f64::from(n)))
            .collect();
        pts.extend(c.knots().iter().map(|&k| c.eval(k)));
        let b = c.bbox();
        let (mut lo, mut hi) = (pts[0], pts[0]);
        for &p in &pts {
            lo = lo.min(p);
            hi = hi.max(p);
            assert!(
                b.min.x - 1e-9 <= p.x && p.x <= b.max.x + 1e-9,
                "case {case}"
            );
            assert!(
                b.min.y - 1e-9 <= p.y && p.y <= b.max.y + 1e-9,
                "case {case}"
            );
        }
        // Tight: within the sampling error of the dense box.
        assert!(
            (b.min - lo).length() < 1e-3 && (b.max - hi).length() < 1e-3,
            "case {case}: {b:?} vs {lo:?} {hi:?}"
        );
        for _ in 0..10 {
            let q = Point2::new(g.range(-15.0, 15.0), g.range(-15.0, 15.0));
            let (t, p) = c.closest(q);
            assert!(c.eval(t).distance(p) < 1e-12);
            let dense = pts
                .iter()
                .map(|&s| s.distance(q))
                .fold(f64::INFINITY, f64::min);
            assert!(
                p.distance(q) <= dense + 1e-9,
                "case {case}: {} vs {dense}",
                p.distance(q)
            );
        }
    }
}
