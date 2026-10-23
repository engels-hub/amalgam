//! Unit tests of the B-spline kernel (SK-08).

use std::f64::consts::FRAC_PI_2;

use amg_core::{Point2, Vec2};

use super::spline::{
    BSpline2, FitMap, check_knots, chord_params, clamped_uniform_knots, conic_weight, fit_spline,
    start_curvature,
};

fn p(x: f64, y: f64) -> Point2 {
    Point2::new(x, y)
}

fn quarter() -> BSpline2 {
    // Exact quarter circle: rho = w / (1 + w) with w = cos(45 deg).
    let w = std::f64::consts::FRAC_1_SQRT_2;
    BSpline2::conic(p(1.0, 0.0), p(1.0, 1.0), p(0.0, 1.0), w / (1.0 + w)).unwrap()
}

fn cubic() -> BSpline2 {
    let poles = vec![
        p(0.0, 0.0),
        p(1.0, 2.0),
        p(3.0, 3.0),
        p(4.0, 0.5),
        p(6.0, 1.0),
        p(7.0, -1.0),
    ];
    BSpline2::new(3, clamped_uniform_knots(6, 3), poles, vec![1.0; 6]).unwrap()
}

fn rational() -> BSpline2 {
    let poles = vec![
        p(0.0, 0.0),
        p(1.0, 2.0),
        p(3.0, 3.0),
        p(4.0, 0.5),
        p(6.0, 1.0),
    ];
    let w = vec![1.0, 2.0, 0.7, 1.5, 1.0];
    BSpline2::new(3, clamped_uniform_knots(5, 3), poles, w).unwrap()
}

#[test]
fn conic_is_an_exact_quarter_circle() {
    let c = quarter();
    for i in 0..=20 {
        let q = c.eval(f64::from(i) / 20.0);
        assert!((q.length() - 1.0).abs() < 1e-14, "{q:?}");
    }
    assert!((c.length() - FRAC_PI_2).abs() < 1e-13);
    assert!((c.curvature_at(0.37) - 1.0).abs() < 1e-12);
    let (a, m) = c.segment_area_moment();
    assert!((a - 0.5 * (FRAC_PI_2 - 1.0)).abs() < 1e-13, "{a}");
    // Circular segment: centroid on the bisector at distance
    // 4 r sin^3(a/2) / (3 (a - sin a)) from the centre.
    let r = 4.0 / 3.0 * (FRAC_PI_2 / 2.0).sin().powi(3) / (FRAC_PI_2 - FRAC_PI_2.sin());
    let c45 = std::f64::consts::FRAC_1_SQRT_2 * r;
    assert!((m.x - a * c45).abs() < 1e-12 && (m.y - a * c45).abs() < 1e-12);
}

#[test]
fn derivatives_match_finite_differences() {
    for c in [cubic(), rational()] {
        let h = 1e-5;
        for t in [0.1_f64, 0.9, 1.5, 2.2] {
            let t = t.min(c.span() - 0.05);
            let (_, d1, d2) = c.derivs(t);
            let fd1 = (c.eval(t + h) - c.eval(t - h)) / (2.0 * h);
            let fd2 = (c.derivative(t + h) - c.derivative(t - h)) / (2.0 * h);
            assert!((d1 - fd1).length() < 1e-7, "{d1:?} {fd1:?}");
            assert!((d2 - fd2).length() < 1e-6, "{d2:?} {fd2:?}");
        }
    }
}

#[test]
fn trimmed_reversed_and_ends() {
    for c in [cubic(), rational()] {
        let (a, b) = (0.4, c.span() - 0.7);
        let t = c.trimmed(a, b);
        assert!((t.span() - (b - a)).abs() < 1e-12);
        for k in 0..=10 {
            let s = (b - a) * f64::from(k) / 10.0;
            assert!(t.eval(s).distance(c.eval(a + s)) < 1e-12);
        }
        let r = c.trimmed(b, a);
        for k in 0..=10 {
            let s = (b - a) * f64::from(k) / 10.0;
            assert!(r.eval(s).distance(c.eval(b - s)) < 1e-12);
        }
        let rev = c.reversed();
        assert!(rev.eval(0.3).distance(c.eval(c.span() - 0.3)) < 1e-12);
        assert!((t.length() - r.length()).abs() < 1e-12);
    }
}

#[test]
fn start_curvature_matches_and_has_exact_gradient() {
    for c in [cubic(), rational()] {
        let v = pole_inputs(&c);
        let (k, g) = start_curvature(c.degree(), c.knots(), &v);
        assert!(
            (k - c.curvature_at(0.0)).abs() < 1e-12,
            "{k} {}",
            c.curvature_at(0.0)
        );
        for i in 0..9 {
            let h = 1e-6;
            let (mut a, mut b) = (v, v);
            a[i] += h;
            b[i] -= h;
            let fd = (start_curvature(c.degree(), c.knots(), &a).0
                - start_curvature(c.degree(), c.knots(), &b).0)
                / (2.0 * h);
            assert!(
                (fd - g[i]).abs() < 1e-6 * (1.0 + fd.abs()),
                "{i}: {fd} {}",
                g[i]
            );
        }
        // End curvature through the reversed curve.
        let r = c.reversed();
        let (kr, _) = start_curvature(r.degree(), r.knots(), &pole_inputs(&r));
        assert!((-kr - c.curvature_at(c.span())).abs() < 1e-10);
    }
}

fn pole_inputs(c: &BSpline2) -> [f64; 9] {
    let mut v = [0.0; 9];
    for s in 0..3 {
        v[3 * s] = c.poles()[s].x;
        v[3 * s + 1] = c.poles()[s].y;
        v[3 * s + 2] = c.weights()[s];
    }
    v
}

#[test]
fn closest_bbox_length() {
    let c = cubic();
    let q = p(3.5, 2.5);
    let (t, pt) = c.closest(q);
    let brute = (0..=20_000)
        .map(|i| c.span() * f64::from(i) / 20_000.0)
        .map(|s| (c.eval(s).distance(q), s))
        .fold((f64::INFINITY, 0.0), |a, b| if b.0 < a.0 { b } else { a });
    assert!(pt.distance(q) <= brute.0 + 1e-12 && brute.0 - pt.distance(q) < 1e-6);
    assert!((t - brute.1).abs() < 1e-3);
    let bb = c.bbox();
    for i in 0..=2000 {
        let s = c.eval(c.span() * f64::from(i) / 2000.0);
        assert!(s.x >= bb.min.x - 1e-12 && s.x <= bb.max.x + 1e-12);
        assert!(s.y >= bb.min.y - 1e-12 && s.y <= bb.max.y + 1e-12);
    }
    let poly: f64 = (0..20_000)
        .map(|i| {
            let (a, b) = (f64::from(i) / 20_000.0, f64::from(i + 1) / 20_000.0);
            c.eval(a * c.span()).distance(c.eval(b * c.span()))
        })
        .sum();
    assert!((c.length() - poly).abs() < 1e-5 * poly);
}

#[test]
fn segment_area_matches_a_fine_polygon() {
    for c in [cubic(), rational()] {
        let (a, _) = c.segment_area_moment();
        // Polygon through 10^4 curve points closed by the chord.
        let n = 10_000;
        let pts: Vec<Point2> = (0..=n)
            .map(|i| c.eval(c.span() * f64::from(i) / f64::from(n)))
            .collect();
        let mut area = 0.0;
        for i in 0..pts.len() {
            let (u, v) = (pts[i], pts[(i + 1) % pts.len()]);
            area += 0.5 * u.perp_dot(v);
        }
        assert!(((a - area) / area).abs() < 1e-7, "{a} {area}");
    }
}

#[test]
fn fit_spline_interpolates_and_respects_handles() {
    let pts = [
        p(0.0, 0.0),
        p(1.0, 1.0),
        p(2.5, 0.5),
        p(4.0, 2.0),
        p(5.0, 0.0),
    ];
    let natural = fit_spline(&pts, None, None).unwrap();
    let params = chord_params(&pts).unwrap();
    let l = natural.span();
    for (q, u) in pts.iter().zip(&params) {
        assert!(natural.eval(u * l).distance(*q) < 1e-12);
    }
    assert!(natural.derivs(0.0).2.length() < 1e-9);
    assert!(natural.derivs(l).2.length() < 1e-9);
    let t0 = Vec2::new(0.0, 2.0);
    let t1 = Vec2::new(1.0, -1.0);
    let c = fit_spline(&pts, Some(t0), Some(t1)).unwrap();
    for (q, u) in pts.iter().zip(&params) {
        assert!(c.eval(u * l).distance(*q) < 1e-12);
    }
    let d0 = c.derivative(0.0);
    assert!(d0.perp_dot(t0).abs() < 1e-9 * d0.length() * t0.length());
    assert!(d0.dot(t0) > 0.0);
    let d1 = c.derivative(l);
    assert!(d1.perp_dot(t1).abs() < 1e-9 * d1.length() * t1.length());
    // The map is affine-equivariant: rows sum to one for point inputs.
    let map = FitMap::new(&params, true, true).unwrap();
    for row in &map.m {
        assert!((row.iter().sum::<f64>() - 1.0).abs() < 1e-12);
    }
}

#[test]
fn validation_errors() {
    let poles = vec![p(0.0, 0.0), p(1.0, 1.0)];
    let ok_knots = || vec![0.0, 0.0, 1.0, 1.0];
    assert!(BSpline2::new(0, ok_knots(), poles.clone(), vec![1.0; 2]).is_err());
    assert!(BSpline2::new(3, clamped_uniform_knots(2, 1), poles.clone(), vec![1.0; 2]).is_err());
    assert!(BSpline2::new(1, vec![0.0, 0.0, 1.0], poles.clone(), vec![1.0; 2]).is_err());
    assert!(BSpline2::new(1, vec![0.0, 0.5, 1.0, 1.0], poles.clone(), vec![1.0; 2]).is_err());
    assert!(BSpline2::new(1, ok_knots(), poles.clone(), vec![1.0, 0.0]).is_err());
    assert!(BSpline2::new(1, ok_knots(), poles, vec![1.0; 2]).is_ok());
    assert!(conic_weight(0.0).is_err() && conic_weight(1.0).is_err());
    assert!(chord_params(&[p(0.0, 0.0), p(0.0, 0.0)]).is_err());
    assert!(check_knots(2, 4, &[0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]).is_err());
}
