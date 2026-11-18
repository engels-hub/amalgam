//! KH-07 Done-when 4: differential test against curvo 0.3.3 (dev-only oracle) on 1000 random
//! curves and 1000 random surfaces, degree 1–7, rational and non-rational:
//! points agree within 1e-12·max(1, D) (D = pole bounding-box diagonal).
//! First derivatives are compared too, relative to their magnitude.
#![allow(clippy::unwrap_used, clippy::float_cmp, reason = "test code")]

mod common;

use amg_nurbs::{NurbsCurve3, NurbsSurface3, Point3, UV, Vec3};
use common::{Rng, curve, diag, samples, surface};
use curvo::prelude::{NurbsCurve3D, NurbsSurface3D};

fn to_curvo_curve(c: &NurbsCurve3) -> NurbsCurve3D<f64> {
    let w = c.weights();
    let pts = c
        .poles()
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let wi = w.map_or(1.0, |w| w[i]);
            [p.x * wi, p.y * wi, p.z * wi, wi].into()
        })
        .collect();
    NurbsCurve3D::try_new(c.degree(), pts, c.knots().as_slice().to_vec()).unwrap()
}

fn to_curvo_surface(s: &NurbsSurface3) -> NurbsSurface3D<f64> {
    let g = s.poles();
    let pts = (0..g.nu())
        .map(|i| {
            (0..g.nv())
                .map(|j| {
                    let p = g.get(i, j);
                    let wi = s.weights().map_or(1.0, |w| w.get(i, j));
                    [p.x * wi, p.y * wi, p.z * wi, wi].into()
                })
                .collect()
        })
        .collect();
    NurbsSurface3D::new(
        s.degree(UV::U),
        s.degree(UV::V),
        s.knots(UV::U).as_slice().to_vec(),
        s.knots(UV::V).as_slice().to_vec(),
        pts,
    )
}

#[test]
fn curves_match_curvo() {
    let mut r = Rng(0x000c_07f0);
    let (mut worst, mut worst_d1) = (0.0f64, 0.0f64);
    for i in 0..1000 {
        let p = 1 + i % 7;
        let c = curve(&mut r, p, (i / 7) % 2 == 1);
        let o = to_curvo_curve(&c);
        let eps = 1e-12 * diag(c.poles()).max(1.0);
        let (a, b) = c.domain();
        for t in samples(a, b, 20, i as u64) {
            let ours = c.eval(t);
            let theirs = o.point_at(t);
            let e = (ours - Point3::new(theirs.x, theirs.y, theirs.z)).length();
            assert!(e <= eps, "curve {i} (p={p}) t={t}: {e:e} > {eps:e}");
            worst = worst.max(e / eps * 1e-12);
            let d = c.derivs(t, 1);
            let od = o.rational_derivatives(t, 1);
            let d1 = Vec3::new(od[1].x, od[1].y, od[1].z);
            worst_d1 = worst_d1.max((d[1] - d1).length() / d1.length().max(1.0));
        }
    }
    eprintln!("curvo curves: max point error {worst:e}·max(1,D), d1 relative {worst_d1:e}");
    assert!(worst_d1 <= 1e-10, "first derivative mismatch {worst_d1:e}");
}

#[test]
fn surfaces_match_curvo() {
    let mut r = Rng(0x005c_07f0);
    let (mut worst, mut worst_d1) = (0.0f64, 0.0f64);
    for i in 0..1000 {
        let (p, q) = (1 + i % 7, 1 + (i / 7) % 7);
        let s = surface(&mut r, p, q, (i / 49) % 2 == 1);
        let o = to_curvo_surface(&s);
        let eps = 1e-12 * diag(s.poles().as_slice()).max(1.0);
        let [(u0, u1), (v0, v1)] = s.domain();
        let us = samples(u0, u1, 5, i as u64);
        let vs = samples(v0, v1, 4, !(i as u64));
        for &u in &us {
            for &v in &vs {
                let ours = s.eval(u, v);
                let theirs = o.point_at(u, v);
                let e = (ours - Point3::new(theirs.x, theirs.y, theirs.z)).length();
                assert!(
                    e <= eps,
                    "surface {i} (p={p}, q={q}) ({u}, {v}): {e:e} > {eps:e}"
                );
                worst = worst.max(e / eps * 1e-12);
                let d = s.derivs(u, v, 1);
                let od = o.rational_derivatives(u, v, 1);
                for (k, l) in [(1, 0), (0, 1)] {
                    let x = &od[k][l];
                    let dv = Vec3::new(x.x, x.y, x.z);
                    worst_d1 = worst_d1.max((d.d[k][l] - dv).length() / dv.length().max(1.0));
                }
            }
        }
    }
    eprintln!("curvo surfaces: max point error {worst:e}·max(1,D), d1 relative {worst_d1:e}");
    assert!(worst_d1 <= 1e-10, "first derivative mismatch {worst_d1:e}");
}
