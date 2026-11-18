//! KH-07 Done-when 2: rational circle radius error ≤ 1e-14·r; sphere and torus points on the
//! analytic surface ≤ 1e-13·R (1000 samples). Also checks the other analytic conversions.
#![allow(clippy::unwrap_used, clippy::float_cmp, reason = "test code")]

mod common;

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use amg_core::{Dir3, Line3, Plane, Point3, Tolerance, Vec3};
use amg_nurbs::{NurbsError, UV, analytic};
use common::samples;

fn planes() -> Vec<Plane> {
    vec![
        Plane::XY,
        Plane::XZ,
        Plane::from_normal(
            Point3::new(3.0, -2.0, 1.5),
            Dir3::new(Vec3::new(1.0, 2.0, -0.5), &Tolerance::DEFAULT).unwrap(),
        ),
    ]
}

/// 1000 (u, v) pairs on a 40 × 25 jittered grid covering the domain including its edges.
fn uv_samples(s: &amg_nurbs::NurbsSurface3, seed: u64) -> Vec<(f64, f64)> {
    let [(u0, u1), (v0, v1)] = s.domain();
    let us = samples(u0, u1, 40, seed);
    let vs = samples(v0, v1, 25, seed ^ 0xabcd);
    us.iter()
        .flat_map(|&u| vs.iter().map(move |&v| (u, v)))
        .collect()
}

#[test]
fn circle_radius_error() {
    let mut worst = 0.0f64;
    for pl in planes() {
        for &r in &[1e-3, 1.0, 7.5, 250.0] {
            // A relative 1e-14 bound is only meaningful when the radius is not far below the
            // distance of the centre from the world origin (absolute rounding of the centre).
            if r < pl.origin.length() {
                continue;
            }
            for &(start, sweep) in &[
                (0.0, TAU),
                (0.3, FRAC_PI_2),
                (-1.0, 2.5),
                (2.0, -4.0),
                (0.0, 1e-3),
            ] {
                let c = analytic::circle_arc(&pl, r, start, sweep).unwrap();
                let (a, b) = c.domain();
                for t in samples(a, b, 1000, 7) {
                    let p = c.eval(t);
                    let e = ((p - pl.origin).length() - r).abs() / r;
                    assert!(pl.signed_distance(p).abs() <= 1e-14 * r.max(5.0));
                    worst = worst.max(e);
                }
                // Ends at the requested angles.
                let at = |t: f64| pl.origin + (pl.x.get() * t.cos() + pl.y.get() * t.sin()) * r;
                assert!((c.start() - at(start)).length() <= 1e-14 * r.max(5.0));
                assert!((c.end() - at(start + sweep)).length() <= 1e-14 * r.max(5.0) * 4.0);
            }
        }
    }
    eprintln!("circle: max relative radius error {worst:e}");
    assert!(worst <= 1e-14, "radius error {worst:e}");
}

#[test]
fn ellipse_points() {
    let pl = planes()[2];
    let (rx, ry) = (5.0, 2.0);
    let c = analytic::ellipse_arc(&pl, rx, ry, -0.5, 5.0).unwrap();
    let (a, b) = c.domain();
    for t in samples(a, b, 1000, 3) {
        let l = pl.to_local(c.eval(t));
        let e = (l.x / rx).powi(2) + (l.y / ry).powi(2) - 1.0;
        assert!(e.abs() <= 1e-14, "{e:e}");
    }
}

#[test]
fn sphere_points() {
    let mut worst = 0.0f64;
    for (c, r) in [
        (Point3::ZERO, 1.0),
        (Point3::new(10.0, -5.0, 3.0), 42.0),
        (Point3::ZERO, 1e-2),
    ] {
        let s = analytic::sphere(c, r).unwrap();
        for (u, v) in uv_samples(&s, 11) {
            let e = ((s.eval(u, v) - c).length() - r).abs() / r;
            worst = worst.max(e);
        }
        // Poles are exact at the domain corner (and to rounding elsewhere along the pole row).
        let [(u0, _), (v0, v1)] = s.domain();
        assert_eq!(s.eval(u0, v0), c - Vec3::Z * r);
        assert!((s.eval(1.0, v1) - (c + Vec3::Z * r)).length() <= 1e-14 * (r + c.length()));
    }
    eprintln!("sphere: max relative error {worst:e}");
    assert!(worst <= 1e-13, "sphere error {worst:e}");
}

#[test]
fn torus_points() {
    let mut worst = 0.0f64;
    for pl in planes() {
        for &(big, small) in &[(5.0, 1.0), (100.0, 30.0), (1.0, 0.999)] {
            let t = analytic::torus(&pl, big, small).unwrap();
            for (u, v) in uv_samples(&t, 13) {
                let p = t.eval(u, v) - pl.origin;
                let z = p.dot(pl.normal.get());
                let rho = (p - pl.normal.get() * z).length();
                let d = (((rho - big).powi(2) + z * z).sqrt() - small).abs();
                worst = worst.max(d / big);
            }
        }
    }
    eprintln!("torus: max error / R {worst:e}");
    assert!(worst <= 1e-13, "torus error {worst:e}");
}

#[test]
fn cylinder_cone_revolution_extrusion() {
    let pl = planes()[2];
    let cyl = analytic::cylinder(&pl, 3.0, 10.0, 4.0).unwrap();
    let cone = analytic::cone(&pl, 3.0, 1.0, 10.0, TAU).unwrap();
    let apex = analytic::cone(&pl, 2.0, 0.0, 4.0, PI).unwrap();
    for (u, v) in uv_samples(&cyl, 5) {
        let p = cyl.eval(u, v) - pl.origin;
        let z = p.dot(pl.normal.get());
        assert!(((p - pl.normal.get() * z).length() - 3.0).abs() <= 1e-13);
        assert!((z - 10.0 * v).abs() <= 1e-12);
    }
    for (u, v) in uv_samples(&cone, 5) {
        let p = cone.eval(u, v) - pl.origin;
        let z = p.dot(pl.normal.get());
        let want = 3.0 + (1.0 - 3.0) * z / 10.0;
        assert!(((p - pl.normal.get() * z).length() - want).abs() <= 1e-13);
    }
    let [(_, _), (_, v1)] = apex.domain();
    assert!((apex.eval(1.0, v1) - (pl.origin + pl.normal.get() * 4.0)).length() <= 1e-13);

    // Revolving a line gives a cylinder; extruding a circle the same cylinder.
    let axis = Line3 {
        point: Point3::ZERO,
        dir: Dir3::Z,
    };
    let prof = analytic::line(Point3::new(2.0, 0.0, 0.0), Point3::new(2.0, 0.0, 5.0));
    let rev = analytic::revolution(&prof, &axis, TAU).unwrap();
    let ext = analytic::extrusion(
        &analytic::circle_arc(&Plane::XY, 2.0, 0.0, TAU).unwrap(),
        Vec3::Z * 5.0,
    );
    for (u, v) in uv_samples(&rev, 9) {
        assert!((rev.eval(u, v) - ext.eval(u, v)).length() <= 1e-13);
    }
    assert_eq!(rev.degree(UV::U), 2);

    // Plane patch and ruled surface.
    let pp = analytic::plane_patch(&pl, (-1.0, 2.0), (0.5, 3.0));
    let q = pp.eval(0.5, 0.5);
    assert!(pl.signed_distance(q).abs() <= 1e-14);
    assert!((pl.to_local(q) - amg_core::Point2::new(0.5, 1.75)).length() <= 1e-14);
    let a = analytic::line(Point3::ZERO, Point3::X);
    let b = analytic::line(Point3::Y, Point3::new(1.0, 1.0, 1.0));
    let ruled = analytic::ruled(&a, &b).unwrap();
    assert!((ruled.eval(0.5, 0.5) - Point3::new(0.5, 0.5, 0.25)).length() <= 1e-15);
    let arc = analytic::circle_arc(&Plane::XY, 1.0, 0.0, PI).unwrap();
    assert!(matches!(
        analytic::ruled(&a, &arc),
        Err(NurbsError::Incompatible(_))
    ));
}

#[test]
fn invalid_analytic_inputs() {
    let pl = Plane::XY;
    fn deg<T>(r: Result<T, NurbsError>) -> bool {
        matches!(r, Err(NurbsError::Degenerate(_)))
    }
    assert!(deg(analytic::circle_arc(&pl, 0.0, 0.0, 1.0)));
    assert!(deg(analytic::circle_arc(&pl, -1.0, 0.0, 1.0)));
    assert!(deg(analytic::circle_arc(&pl, 1.0, 0.0, 0.0)));
    assert!(deg(analytic::circle_arc(&pl, 1.0, 0.0, 7.0)));
    assert!(deg(analytic::circle_arc(&pl, f64::NAN, 0.0, 1.0)));
    assert!(deg(analytic::circle_arc(&pl, 1.0, f64::INFINITY, 1.0)));
    assert!(deg(analytic::ellipse_arc(&pl, 1.0, 0.0, 0.0, 1.0)));
    assert!(deg(analytic::ellipse_arc(&pl, 1.0, 1.0, 1.0, 1.0)));
    assert!(deg(analytic::sphere(Point3::ZERO, 0.0)));
    assert!(deg(analytic::sphere(Point3::splat(f64::NAN), 1.0)));
    assert!(deg(analytic::torus(&pl, 1.0, 1.0)));
    assert!(deg(analytic::torus(&pl, 1.0, 0.0)));
    assert!(deg(analytic::cylinder(&pl, 1.0, 0.0, 1.0)));
    assert!(deg(analytic::cone(&pl, 0.0, 0.0, 1.0, 1.0)));
    assert!(deg(analytic::cone(&pl, -1.0, 1.0, 1.0, 1.0)));
    let axis = Line3 {
        point: Point3::ZERO,
        dir: Dir3::Z,
    };
    let prof = analytic::line(Point3::X, Point3::new(1.0, 0.0, 1.0));
    assert!(deg(analytic::revolution(&prof, &axis, 0.0)));
}
