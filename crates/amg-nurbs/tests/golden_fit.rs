//! KH-13 golden curves: fitted geometry against `tests/golden/nurbs/*.json` (serde form of
//! `NurbsCurve`). Knots and poles must match within 1e-9·max(1, D) (room for platform libm
//! differences in `sqrt` / `sin`). `AMG_BLESS=1` rewrites the files from the current result
//! (same switch as `amg-testkit`).
#![allow(clippy::unwrap_used, clippy::expect_used, reason = "test code")]

mod common;

use std::path::PathBuf;

use amg_core::{Point2, Point3};
use amg_nurbs::fit::{self, InterpOpts, Parametrization};
use amg_nurbs::{CtrlPt, NurbsCurve};
use common::Rng;

fn golden_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../tests/golden/nurbs")
        .join(format!("{name}.json"))
}

fn check<P: CtrlPt>(name: &str, c: &NurbsCurve<P>) {
    let path = golden_path(name);
    let json = serde_json::to_string_pretty(c).unwrap();
    if std::env::var("AMG_BLESS").is_ok_and(|v| v == "1") {
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(&path, json + "\n").unwrap();
        return;
    }
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("{}: {e} (run with AMG_BLESS=1 to create)", path.display()));
    let want: NurbsCurve<P> = serde_json::from_str(&text).unwrap();
    assert_eq!(want.degree(), c.degree(), "{name}: degree");
    assert_eq!(want.poles().len(), c.poles().len(), "{name}: pole count");
    let d = want.poles().iter().fold(1.0f64, |m, p| m.max(p.length()));
    let eps = 1e-9 * d;
    for (a, b) in want.knots().as_slice().iter().zip(c.knots().as_slice()) {
        assert!((a - b).abs() <= 1e-12, "{name}: knot {a} vs {b}");
    }
    for (i, (a, b)) in want.poles().iter().zip(c.poles()).enumerate() {
        assert!((*a - *b).length() <= eps, "{name}: pole {i} {a:?} vs {b:?}");
    }
}

#[test]
fn golden_example_9_1() {
    let q = [
        Point2::new(0., 0.),
        Point2::new(3., 4.),
        Point2::new(-1., 4.),
        Point2::new(-4., 0.),
        Point2::new(-4., -3.),
    ];
    let o = InterpOpts {
        degree: 3,
        parametrization: Parametrization::ChordLength,
        ..InterpOpts::default()
    };
    check("interp_example_9_1", &fit::interpolate(&q, &o).unwrap());
}

#[test]
fn golden_closed_and_constrained() {
    let q: Vec<Point3> = (0..12)
        .map(|i| {
            let a = i as f64 / 12.0 * std::f64::consts::TAU;
            Point3::new(20.0 * a.cos(), 12.0 * a.sin(), 3.0 * (3.0 * a).sin())
        })
        .collect();
    let closed = InterpOpts {
        closed: true,
        ..InterpOpts::default()
    };
    check("interp_closed_12", &fit::interpolate(&q, &closed).unwrap());
    let ends = InterpOpts {
        ends: fit::EndDerivs {
            start: Some(Point3::new(0.0, 60.0, 10.0)),
            end: Some(Point3::new(10.0, 60.0, 0.0)),
            start2: None,
            end2: None,
        },
        ..InterpOpts::default()
    };
    check(
        "interp_end_tangents_12",
        &fit::interpolate(&q, &ends).unwrap(),
    );
}

#[test]
fn golden_approximation() {
    let mut r = Rng(0x13_60);
    let pts: Vec<Point3> = (0..400)
        .map(|i| {
            let s = i as f64 / 399.0 * 2.0 * std::f64::consts::TAU;
            Point3::new(
                10.0 * s.cos(),
                10.0 * s.sin(),
                4.0 * s / std::f64::consts::TAU,
            ) + Point3::new(
                r.range(-1e-4, 1e-4),
                r.range(-1e-4, 1e-4),
                r.range(-1e-4, 1e-4),
            )
        })
        .collect();
    let f =
        fit::approximate_to_tolerance(&pts, 3, 1e-3, 200, Parametrization::Centripetal).unwrap();
    check("approx_helix_1e-3", &f.curve);
}
