//! KH-07 Done-when 3: derivatives match central finite differences (h = 1e-6) within 1e-6
//! relative; analytic circle tangent/curvature and space-curve curvature/torsion within 1e-12.
//!
//! A circular helix has no exact NURBS form, so the space-curve check uses the twisted cubic
//! `(t, t², t³)` (exact cubic Bézier) with its closed-form curvature and torsion.
#![allow(clippy::unwrap_used, clippy::float_cmp, reason = "test code")]

mod common;

use std::f64::consts::TAU;

use amg_core::{Dir3, Plane, Point3, Tolerance, Vec3};
use amg_nurbs::{KnotVector, NurbsCurve3, analytic};
use common::{Rng, curve, samples, surface};

const H: f64 = 1e-6;

/// Random parameter inside a span longer than 0.05 (so FD truncation stays far below 1e-6),
/// at least `10·H` from its knots.
fn inside(r: &mut Rng, knots: &[f64]) -> Option<f64> {
    let spans: Vec<(f64, f64)> = knots
        .windows(2)
        .filter(|w| w[1] - w[0] > 0.05)
        .map(|w| (w[0], w[1]))
        .collect();
    if spans.is_empty() {
        return None;
    }
    let (a, b) = spans[r.usize(0, spans.len() - 1)];
    Some(r.range(a + 10.0 * H, b - 10.0 * H))
}

fn rel(fd: Vec3, exact: Vec3) -> f64 {
    (fd - exact).length() / exact.length().max(1.0)
}

#[test]
fn curve_derivatives_match_finite_differences() {
    let mut r = Rng(0xd1ff);
    let mut worst = [0.0f64; 4];
    for i in 0..400 {
        let p = 1 + i % 7;
        let c = curve(&mut r, p, i % 2 == 1);
        for _ in 0..10 {
            let Some(t) = inside(&mut r, c.knots().as_slice()) else {
                continue;
            };
            let d = c.derivs(t, 4);
            let dp = c.derivs(t + H, 4);
            let dm = c.derivs(t - H, 4);
            assert!((d[0] - c.eval(t)).length() <= 1e-12 * d[0].length().max(1.0));
            for k in 1..=3 {
                let fd = (dp[k - 1] - dm[k - 1]) / (2.0 * H);
                worst[k] = worst[k].max(rel(fd, d[k]));
            }
        }
    }
    eprintln!(
        "curve FD relative errors (C', C'', C'''): {:e} {:e} {:e}",
        worst[1], worst[2], worst[3]
    );
    assert!(worst.iter().all(|&w| w <= 1e-6), "{worst:?}");
}

#[test]
fn surface_derivatives_match_finite_differences() {
    let mut r = Rng(0x5d1f);
    let mut worst = 0.0f64;
    for i in 0..300 {
        let (p, q) = (1 + i % 5, 1 + (i / 5) % 5);
        let s = surface(&mut r, p, q, i % 2 == 1);
        for _ in 0..5 {
            let (Some(u), Some(v)) = (
                inside(&mut r, s.knots(amg_nurbs::UV::U).as_slice()),
                inside(&mut r, s.knots(amg_nurbs::UV::V).as_slice()),
            ) else {
                continue;
            };
            let d = s.derivs(u, v, 2);
            assert!((d.d[0][0] - s.eval(u, v)).length() <= 1e-12 * d.d[0][0].length().max(1.0));
            let up = s.derivs(u + H, v, 1);
            let um = s.derivs(u - H, v, 1);
            let vp = s.derivs(u, v + H, 1);
            let vm = s.derivs(u, v - H, 1);
            let checks = [
                ((up.d[0][0] - um.d[0][0]) / (2.0 * H), d.d[1][0]),
                ((vp.d[0][0] - vm.d[0][0]) / (2.0 * H), d.d[0][1]),
                ((up.d[1][0] - um.d[1][0]) / (2.0 * H), d.d[2][0]),
                ((vp.d[0][1] - vm.d[0][1]) / (2.0 * H), d.d[0][2]),
                ((vp.d[1][0] - vm.d[1][0]) / (2.0 * H), d.d[1][1]),
                ((up.d[0][1] - um.d[0][1]) / (2.0 * H), d.d[1][1]),
            ];
            for (fd, exact) in checks {
                worst = worst.max(rel(fd, exact));
            }
        }
    }
    eprintln!("surface FD relative error: {worst:e}");
    assert!(worst <= 1e-6, "{worst:e}");
}

#[test]
fn circle_tangent_and_curvature() {
    let tol = Tolerance::DEFAULT;
    let plane = Plane::from_normal(
        Point3::new(1.0, 2.0, 3.0),
        Dir3::new(Vec3::new(0.3, -1.0, 0.4), &tol).unwrap(),
    );
    let mut worst = 0.0f64;
    for &r in &[0.5, 4.0, 80.0] {
        let c = analytic::circle_arc(&plane, r, 0.4, TAU).unwrap();
        let (a, b) = c.domain();
        for t in samples(a, b, 1000, 21) {
            let d = c.derivs(t, 3);
            let radial = (d[0] - plane.origin) / r;
            let tangent = d[1].normalize();
            let want = plane.normal.get().cross(radial);
            worst = worst.max((tangent - want).length());
            worst = worst.max((c.curvature(t) * r - 1.0).abs());
            worst = worst.max((c.torsion(t) * r).abs());
        }
    }
    eprintln!("circle tangent/curvature/torsion relative error: {worst:e}");
    assert!(worst <= 1e-12, "{worst:e}");
}

#[test]
fn twisted_cubic_curvature_and_torsion() {
    let c = NurbsCurve3::new(
        3,
        KnotVector::bezier(3),
        vec![
            Point3::ZERO,
            Point3::new(1.0 / 3.0, 0.0, 0.0),
            Point3::new(2.0 / 3.0, 1.0 / 3.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ],
        None,
    )
    .unwrap();
    let mut worst = 0.0f64;
    for t in samples(0.0, 1.0, 1000, 5) {
        let d = c.derivs(t, 3);
        let want = [
            Vec3::new(t, t * t, t * t * t),
            Vec3::new(1.0, 2.0 * t, 3.0 * t * t),
            Vec3::new(0.0, 2.0, 6.0 * t),
            Vec3::new(0.0, 0.0, 6.0),
        ];
        for k in 0..4 {
            worst = worst.max((d[k] - want[k]).length() / want[k].length().max(1.0));
        }
        let b = want[1].cross(want[2]);
        let kappa = b.length() / want[1].length().powi(3);
        let tau = 12.0 / b.length_squared();
        worst = worst.max((c.curvature(t) - kappa).abs() / kappa);
        worst = worst.max((c.torsion(t) - tau).abs() / tau);
    }
    eprintln!("twisted cubic derivative/curvature/torsion relative error: {worst:e}");
    assert!(worst <= 1e-12, "{worst:e}");
}
