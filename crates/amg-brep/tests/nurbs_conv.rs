//! S20 KH-16: `amg_brep::nurbs_conv`, amg-nurbs ↔ OCCT `Geom_BSpline*`.
//!
//! - Done-when 1: amg-nurbs → OCCT → amg-nurbs round trips are bit-identical (poles, weights,
//!   knots with multiplicities): `round_trip_*_bit_identical`.
//! - Done-when 2: OCCT `D0/D1/D2` vs `eval/derivs` on 10 k random parameters of 200 random
//!   curves and surfaces ≤ 1e-11·max(1, |D|): `occt_d2_matches_*`, plus the periodic unwrap:
//!   `periodic_*_unwrap_matches_occt`.
//! - Done-when 3: OCCT lines, circles, ellipses, planes, cylinders, cones, spheres and tori
//!   convert to exact NURBS (radius error ≤ 1e-12·r): `occt_analytic_*`.
//! - Done-when 4: `nurbs_face` on a skinned patch is BRepCheck-valid and its tessellation lies
//!   within 1e-6 mm of `eval`: `nurbs_face_on_skinned_patch`.
//! - Done-when 5: OCCT 8.0.1 has `GeomFill_Gordon`; the oracle is exercised by
//!   `gordon_oracle_reproduces_network`. The comparison with amg-nurbs' own Gordon surfaces on
//!   the TiGL set is deferred to KH-15 (neither exists yet).
//!
//! Measured numbers print with `--nocapture`.
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::f64::consts::{FRAC_PI_2, TAU};

use amg_brep::geom::{BSplineCurveData, BSplineSurfaceData, Curve3, CurveSeg, Surface, Tag};
use amg_brep::nurbs_conv::{curve_seg_to_nurbs, surface_to_nurbs};
use amg_brep::{KernelError, Plane, Point3, Tolerance, Vec3};
use amg_nurbs::{Grid, KnotVector, NurbsCurve3, NurbsSurface3, UV};

/// SplitMix64: deterministic test data without a dependency.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    fn f(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }

    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.f()
    }

    fn int(&mut self, lo: usize, hi: usize) -> usize {
        lo + (self.next() % (hi - lo + 1) as u64) as usize
    }

    fn point(&mut self, s: f64) -> Point3 {
        Point3::new(self.range(-s, s), self.range(-s, s), self.range(-s, s))
    }
}

/// A random clamped knot vector for `n` poles of degree `p`, with interior multiplicities up
/// to `p`.
fn random_knots(rng: &mut Rng, p: usize, n: usize) -> KnotVector {
    let a = rng.range(-10.0, 10.0);
    let b = a + rng.range(0.5, 20.0);
    let mut interior = n - p - 1;
    let mut values = Vec::new();
    while interior > 0 {
        let m = rng.int(1, p.min(interior));
        let t = rng.range(0.02, 0.98);
        values.push((t, m));
        interior -= m;
    }
    values.sort_by(|x, y| x.0.total_cmp(&y.0));
    values.dedup_by(|x, y| {
        (x.0 - y.0).abs() < 1e-3 && {
            y.1 = (y.1 + x.1).min(p);
            true
        }
    });
    // Rebuild the pole count from the merged multiplicities.
    let mut k = vec![a; p + 1];
    for (t, m) in &values {
        k.extend(std::iter::repeat_n(a + (b - a) * t, *m));
    }
    k.extend(std::iter::repeat_n(b, p + 1));
    KnotVector::new(k).unwrap()
}

fn random_curve(rng: &mut Rng, rational: bool) -> NurbsCurve3 {
    let p = rng.int(1, 9);
    let n = rng.int(p + 1, p + 14);
    let knots = random_knots(rng, p, n);
    let n = knots.len() - p - 1;
    let poles = (0..n).map(|_| rng.point(100.0)).collect();
    let weights = rational.then(|| (0..n).map(|_| rng.range(0.3, 3.0)).collect());
    NurbsCurve3::new(p, knots, poles, weights).unwrap()
}

fn random_surface(rng: &mut Rng, rational: bool) -> NurbsSurface3 {
    let pu = rng.int(1, 6);
    let pv = rng.int(1, 6);
    let ku = {
        let n = rng.int(pu + 1, pu + 6);
        random_knots(rng, pu, n)
    };
    let kv = {
        let n = rng.int(pv + 1, pv + 6);
        random_knots(rng, pv, n)
    };
    let nu = ku.len() - pu - 1;
    let nv = kv.len() - pv - 1;
    let poles = Grid::new(nu, nv, (0..nu * nv).map(|_| rng.point(100.0)).collect()).unwrap();
    let weights = rational
        .then(|| Grid::new(nu, nv, (0..nu * nv).map(|_| rng.range(0.3, 3.0)).collect()).unwrap());
    NurbsSurface3::new([pu, pv], [ku, kv], poles, weights).unwrap()
}

fn bits(p: Point3) -> [u64; 3] {
    [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]
}

fn assert_curve_bits(a: &NurbsCurve3, b: &NurbsCurve3) {
    assert_eq!(a.degree(), b.degree());
    let ka: Vec<u64> = a.knots().as_slice().iter().map(|k| k.to_bits()).collect();
    let kb: Vec<u64> = b.knots().as_slice().iter().map(|k| k.to_bits()).collect();
    assert_eq!(ka, kb, "knots");
    let pa: Vec<[u64; 3]> = a.poles().iter().map(|&p| bits(p)).collect();
    let pb: Vec<[u64; 3]> = b.poles().iter().map(|&p| bits(p)).collect();
    assert_eq!(pa, pb, "poles");
    let wa = a
        .weights()
        .map(|w| w.iter().map(|x| x.to_bits()).collect::<Vec<_>>());
    let wb = b
        .weights()
        .map(|w| w.iter().map(|x| x.to_bits()).collect::<Vec<_>>());
    assert_eq!(wa, wb, "weights");
}

fn assert_surface_bits(a: &NurbsSurface3, b: &NurbsSurface3) {
    for d in [UV::U, UV::V] {
        assert_eq!(a.degree(d), b.degree(d));
        let ka: Vec<u64> = a.knots(d).as_slice().iter().map(|k| k.to_bits()).collect();
        let kb: Vec<u64> = b.knots(d).as_slice().iter().map(|k| k.to_bits()).collect();
        assert_eq!(ka, kb, "knots {d:?}");
    }
    let pa: Vec<[u64; 3]> = a.poles().as_slice().iter().map(|&p| bits(p)).collect();
    let pb: Vec<[u64; 3]> = b.poles().as_slice().iter().map(|&p| bits(p)).collect();
    assert_eq!(pa, pb, "poles");
    let wa = a
        .weights()
        .map(|w| w.as_slice().iter().map(|x| x.to_bits()).collect::<Vec<_>>());
    let wb = b
        .weights()
        .map(|w| w.as_slice().iter().map(|x| x.to_bits()).collect::<Vec<_>>());
    assert_eq!(wa, wb, "weights");
}

// ---------------------------------------------------------------------------------------------
// Pure Rust (both builds)
// ---------------------------------------------------------------------------------------------

#[test]
fn data_round_trip_is_bit_identical_without_occt() {
    let mut rng = Rng(1);
    for i in 0..200 {
        let c = random_curve(&mut rng, i % 2 == 1);
        let data = BSplineCurveData::from(&c);
        assert!(!data.periodic);
        assert_eq!(data.knots.len(), data.mults.len());
        assert_curve_bits(&c, &NurbsCurve3::try_from(&data).unwrap());
        let s = random_surface(&mut rng, i % 2 == 1);
        let data = BSplineSurfaceData::from(&s);
        assert_eq!(data.poles.len(), (data.nu * data.nv) as usize);
        assert_surface_bits(&s, &NurbsSurface3::try_from(&data).unwrap());
    }
}

#[test]
fn equal_weights_are_dropped() {
    let c = NurbsCurve3::new(
        2,
        KnotVector::bezier(2),
        vec![Point3::ZERO, Point3::X, Point3::Y],
        Some(vec![2.0; 3]),
    )
    .unwrap();
    let data = BSplineCurveData::from(&c);
    assert_eq!(data.weights, None);
    let back = NurbsCurve3::try_from(&data).unwrap();
    assert!((back.eval(0.3) - c.eval(0.3)).length() < 1e-15);
}

#[test]
fn malformed_data_is_rejected() {
    let mut data = BSplineCurveData::from(&amg_nurbs::analytic::line(Point3::ZERO, Point3::X));
    data.mults[0] = 5;
    assert!(NurbsCurve3::try_from(&data).is_err());
    let seg = CurveSeg {
        curve: Curve3::Other,
        t0: 0.0,
        t1: 1.0,
        tag: Tag(0),
    };
    assert!(matches!(
        curve_seg_to_nurbs(&seg),
        Err(KernelError::InvalidInput { .. })
    ));
    let other = Surface::Other {
        class: amg_brep::geom::GeomClass::Other,
    };
    assert!(surface_to_nurbs(&other, [(0.0, 1.0), (0.0, 1.0)]).is_err());
    let line = CurveSeg {
        curve: Curve3::Line {
            origin: Point3::ZERO,
            dir: amg_brep::Dir3::X,
        },
        t0: 1.0,
        t1: 1.0,
        tag: Tag(0),
    };
    assert!(curve_seg_to_nurbs(&line).is_err());
}

fn tilted() -> Plane {
    let n = amg_brep::Dir3::new(Vec3::new(1.0, 2.0, 3.0), &Tolerance::DEFAULT).unwrap();
    Plane::from_normal_x(Point3::new(3.0, -2.0, 5.0), n, Vec3::X, &Tolerance::DEFAULT).unwrap()
}

/// Local frame coordinates of `p`.
fn local(f: &Plane, p: Point3) -> Vec3 {
    let d = p - f.origin;
    Vec3::new(d.dot(f.x.get()), d.dot(f.y.get()), d.dot(f.normal.get()))
}

/// Distance of `p` from the analytic curve, and the curve's scale (its radius, or 1 for a
/// line).
fn curve_error(c: &Curve3, p: Point3) -> (f64, f64) {
    match c {
        Curve3::Line { origin, dir } => {
            let d = p - *origin;
            ((d - dir.get() * d.dot(dir.get())).length(), 1.0)
        }
        Curve3::Circle { frame, radius } => {
            let q = local(frame, p);
            let rho = q.x.hypot(q.y);
            ((rho - radius).hypot(q.z), *radius)
        }
        Curve3::Ellipse {
            frame,
            major_radius: a,
            minor_radius: b,
        } => {
            let q = local(frame, p);
            let f = (q.x / a).hypot(q.y / b) - 1.0;
            ((f * b).hypot(q.z), *a)
        }
        _ => unreachable!(),
    }
}

/// Distance of `p` from the analytic surface (radial for revolved ones), and its radius.
fn surface_error(s: &Surface, p: Point3) -> (f64, f64) {
    match s {
        Surface::Plane(f) => (local(f, p).z.abs(), 1.0),
        Surface::Cylinder { frame, radius } => {
            let q = local(frame, p);
            ((q.x.hypot(q.y) - radius).abs(), *radius)
        }
        Surface::Cone {
            frame,
            ref_radius,
            half_angle,
        } => {
            let q = local(frame, p);
            let rho = q.x.hypot(q.y);
            let (sin, cos) = half_angle.sin_cos();
            (
                ((rho - ref_radius) * cos - q.z * sin).abs(),
                ref_radius.max(1.0),
            )
        }
        Surface::Sphere { frame, radius } => ((local(frame, p).length() - radius).abs(), *radius),
        Surface::Torus {
            frame,
            major,
            minor,
        } => {
            let q = local(frame, p);
            (((q.x.hypot(q.y) - major).hypot(q.z) - minor).abs(), *minor)
        }
        _ => unreachable!(),
    }
}

/// The analytic surface point at `(u, v)` (OCCT's parametrisations).
fn surface_at(s: &Surface, u: f64, v: f64) -> Point3 {
    let ring = |f: &Plane, r: f64, h: f64| {
        f.origin + (f.x.get() * u.cos() + f.y.get() * u.sin()) * r + f.normal.get() * h
    };
    match s {
        Surface::Plane(f) => f.origin + f.x.get() * u + f.y.get() * v,
        Surface::Cylinder { frame, radius } => ring(frame, *radius, v),
        Surface::Cone {
            frame,
            ref_radius,
            half_angle,
        } => ring(
            frame,
            ref_radius + v * half_angle.sin(),
            v * half_angle.cos(),
        ),
        Surface::Sphere { frame, radius } => ring(frame, radius * v.cos(), radius * v.sin()),
        Surface::Torus {
            frame,
            major,
            minor,
        } => ring(frame, major + minor * v.cos(), minor * v.sin()),
        _ => unreachable!(),
    }
}

/// Max relative error over a 21×21 grid, and max corner distance from the analytic point.
fn check_surface(s: &Surface, uv: [(f64, f64); 2]) -> (f64, f64) {
    let n = surface_to_nurbs(s, uv).unwrap();
    assert_eq!(n.domain(), uv, "{s:?}");
    let mut worst = 0.0f64;
    for i in 0..=20 {
        for j in 0..=20 {
            let u = uv[0].0 + (uv[0].1 - uv[0].0) * i as f64 / 20.0;
            let v = uv[1].0 + (uv[1].1 - uv[1].0) * j as f64 / 20.0;
            let (e, r) = surface_error(s, n.eval(u, v));
            worst = worst.max(e / r);
        }
    }
    let mut corner = 0.0f64;
    for (u, v) in [
        (uv[0].0, uv[1].0),
        (uv[0].1, uv[1].0),
        (uv[0].0, uv[1].1),
        (uv[0].1, uv[1].1),
    ] {
        corner = corner.max((n.eval(u, v) - surface_at(s, u, v)).length());
    }
    (worst, corner)
}

#[test]
fn analytic_surfaces_are_exact() {
    let f = tilted();
    let cases = [
        (Surface::Plane(f), [(-3.0, 4.0), (1.0, 9.0)]),
        (
            Surface::Cylinder {
                frame: f,
                radius: 7.0,
            },
            [(0.4, 5.1), (-2.0, 12.0)],
        ),
        (
            Surface::Cone {
                frame: f,
                ref_radius: 4.0,
                half_angle: -0.3,
            },
            [(1.0, 1.0 + TAU), (0.0, 6.0)],
        ),
        (
            Surface::Sphere {
                frame: f,
                radius: 5.0,
            },
            [(0.0, TAU), (-FRAC_PI_2, FRAC_PI_2)],
        ),
        (
            Surface::Sphere {
                frame: f,
                radius: 5.0,
            },
            [(2.0, 3.0), (-0.4, 1.2)],
        ),
        (
            Surface::Torus {
                frame: f,
                major: 10.0,
                minor: 3.0,
            },
            [(-1.0, 2.5), (0.3, 0.3 + TAU)],
        ),
    ];
    for (s, uv) in &cases {
        let (worst, corner) = check_surface(s, *uv);
        println!(
            "{:?}: radius error {worst:.2e}·r, corner {corner:.2e} mm",
            s.class()
        );
        assert!(worst <= 1e-12, "{s:?}: {worst:e}");
        assert!(corner <= 1e-12 * 20.0, "{s:?}: corner {corner:e}");
    }
    assert!(
        surface_to_nurbs(
            &Surface::Sphere {
                frame: f,
                radius: 1.0
            },
            [(0.0, 1.0), (0.0, 2.0)]
        )
        .is_err()
    );
}

#[test]
fn analytic_curves_are_exact() {
    let f = tilted();
    let cases = [
        Curve3::Line {
            origin: f.origin,
            dir: f.x,
        },
        Curve3::Circle {
            frame: f,
            radius: 6.0,
        },
        Curve3::Ellipse {
            frame: f,
            major_radius: 9.0,
            minor_radius: 2.5,
        },
    ];
    for c in &cases {
        for (t0, t1) in [(0.0, TAU), (0.7, 2.2), (3.0, -1.0)] {
            let seg = CurveSeg {
                curve: c.clone(),
                t0,
                t1,
                tag: Tag(0),
            };
            let n = curve_seg_to_nurbs(&seg).unwrap();
            assert_eq!(n.domain(), (t0.min(t1), t0.max(t1)));
            let mut worst = 0.0f64;
            for i in 0..=200 {
                let (e, r) =
                    curve_error(c, n.eval(t0.min(t1) + (t0 - t1).abs() * i as f64 / 200.0));
                worst = worst.max(e / r);
            }
            assert!(worst <= 1e-12, "{c:?} [{t0}, {t1}]: {worst:e}");
            let start = seg.start().unwrap();
            let end = seg.end().unwrap();
            assert!((n.start() - start).length() <= 1e-12 * 10.0, "{c:?}");
            assert!((n.end() - end).length() <= 1e-12 * 10.0, "{c:?}");
        }
    }
}

#[test]
fn bspline_segments_trim_and_reverse() {
    let mut rng = Rng(7);
    for _ in 0..50 {
        let c = random_curve(&mut rng, true);
        let (a, b) = c.domain();
        let (t0, t1) = (a + 0.3 * (b - a), a + 0.8 * (b - a));
        let seg = CurveSeg {
            curve: Curve3::BSpline((&c).into()),
            t0: t1,
            t1: t0,
            tag: Tag(0),
        };
        let n = curve_seg_to_nurbs(&seg).unwrap();
        assert_eq!(n.domain(), (t0, t1));
        for i in 0..=10 {
            let t = t0 + (t1 - t0) * i as f64 / 10.0;
            let scale = c.eval(t).length().max(1.0);
            assert!((n.eval(t0 + t1 - t) - c.eval(t)).length() <= 1e-11 * scale);
        }
        // The whole domain passes the curve through unchanged.
        let whole = CurveSeg {
            curve: Curve3::BSpline((&c).into()),
            t0: a,
            t1: b,
            tag: Tag(0),
        };
        assert_curve_bits(&curve_seg_to_nurbs(&whole).unwrap(), &c);
    }
}

// ---------------------------------------------------------------------------------------------
// With OCCT
// ---------------------------------------------------------------------------------------------

#[cfg(feature = "occt")]
mod occt {
    use std::f64::consts::PI;
    use std::time::Instant;

    use amg_brep::geom::FaceBounds;
    use amg_brep::nurbs_conv::{edge_to_nurbs, nurbs_face, oracle, wire_to_chain};
    use amg_brep::{
        BrepKernel, CheckOpts, EdgeId, FaceId, JobCtx, KernelConfig, OcctKernel, Point2, Shape,
        TessQuality,
    };
    use amg_nurbs::fit::{InterpOpts, interpolate};

    use super::*;

    fn kernel() -> OcctKernel {
        OcctKernel::new(KernelConfig::default())
    }

    #[test]
    fn round_trip_curves_bit_identical() {
        let mut rng = Rng(11);
        for i in 0..200 {
            let c = random_curve(&mut rng, i % 2 == 1);
            let data = BSplineCurveData::from(&c);
            let occt = oracle::round_trip_curve(&data).unwrap();
            assert_eq!(occt, data, "curve {i}");
            assert_curve_bits(&c, &NurbsCurve3::try_from(&occt).unwrap());
        }
        println!("200 curves (100 rational, degree 1..=9): bit-identical through OCCT");
    }

    #[test]
    fn round_trip_surfaces_bit_identical() {
        let mut rng = Rng(12);
        for i in 0..200 {
            let s = random_surface(&mut rng, i % 2 == 1);
            let data = BSplineSurfaceData::from(&s);
            let occt = oracle::round_trip_surface(&data).unwrap();
            assert_eq!(occt, data, "surface {i}");
            assert_surface_bits(&s, &NurbsSurface3::try_from(&occt).unwrap());
        }
        println!("200 surfaces (100 rational, degree 1..=6): bit-identical through OCCT");
    }

    fn rel(a: Vec3, b: Vec3) -> f64 {
        (a - b).length() / b.length().max(1.0)
    }

    #[test]
    fn occt_d2_matches_curves() {
        let mut rng = Rng(21);
        let mut worst = [0.0f64; 3];
        let mut count = 0;
        let start = Instant::now();
        for i in 0..200 {
            let c = random_curve(&mut rng, i % 2 == 1);
            let (a, b) = c.domain();
            let ts: Vec<f64> = (0..50).map(|_| rng.range(a, b)).collect();
            let occt = oracle::curve_d2(&(&c).into(), &ts).unwrap();
            for (t, (p, d1, d2)) in ts.iter().zip(occt) {
                let ours = c.derivs(*t, 2);
                for (k, occ) in [p, d1, d2].into_iter().enumerate() {
                    worst[k] = worst[k].max(rel(ours[k], occ));
                }
                count += 1;
            }
        }
        println!(
            "curves: {count} parameters, max rel. error D0 {:.2e}, D1 {:.2e}, D2 {:.2e} ({:?})",
            worst[0],
            worst[1],
            worst[2],
            start.elapsed()
        );
        assert_eq!(count, 10_000);
        assert!(worst.iter().all(|&e| e <= 1e-11), "{worst:?}");
    }

    #[test]
    fn occt_d2_matches_surfaces() {
        let mut rng = Rng(22);
        let mut worst = [0.0f64; 6];
        let mut count = 0;
        for i in 0..200 {
            let s = random_surface(&mut rng, i % 2 == 1);
            let [(ua, ub), (va, vb)] = s.domain();
            let uvs: Vec<(f64, f64)> = (0..50)
                .map(|_| (rng.range(ua, ub), rng.range(va, vb)))
                .collect();
            let occt = oracle::surface_d2(&(&s).into(), &uvs).unwrap();
            for (&(u, v), o) in uvs.iter().zip(occt) {
                let d = s.derivs(u, v, 2).d;
                let pairs = [
                    (d[0][0], o.p),
                    (d[1][0], o.du),
                    (d[0][1], o.dv),
                    (d[2][0], o.duu),
                    (d[0][2], o.dvv),
                    (d[1][1], o.duv),
                ];
                for (k, (ours, occ)) in pairs.into_iter().enumerate() {
                    worst[k] = worst[k].max(rel(ours, occ));
                }
                count += 1;
            }
        }
        println!(
            "surfaces: {count} parameters, max rel. error S {:.2e}, Su {:.2e}, Sv {:.2e}, \
             Suu {:.2e}, Svv {:.2e}, Suv {:.2e}",
            worst[0], worst[1], worst[2], worst[3], worst[4], worst[5]
        );
        assert_eq!(count, 10_000);
        assert!(worst.iter().all(|&e| e <= 1e-11), "{worst:?}");
    }

    /// Random periodic data in OCCT form: equal end multiplicities, `N = m0 + Σ interior`.
    fn random_periodic(rng: &mut Rng, p: usize) -> (Vec<f64>, Vec<u32>, usize) {
        let m0 = rng.int(1, p);
        let n_interior = rng.int(1, 6);
        let mut knots = vec![rng.range(-5.0, 5.0)];
        for _ in 0..=n_interior {
            let last = knots[knots.len() - 1];
            knots.push(last + rng.range(0.2, 3.0));
        }
        let mut mults = vec![m0 as u32];
        mults.extend((0..n_interior).map(|_| rng.int(1, p) as u32));
        mults.push(m0 as u32);
        let mut count: usize = mults[..mults.len() - 1].iter().map(|&m| m as usize).sum();
        while count < p + 1 {
            mults[1] = (mults[1] + 1).min(p as u32);
            knots.insert(1, (knots[0] + knots[1]) * 0.5);
            mults.insert(1, 1);
            count = mults[..mults.len() - 1].iter().map(|&m| m as usize).sum();
        }
        (knots, mults, count)
    }

    #[test]
    fn periodic_curve_unwrap_matches_occt() {
        let mut rng = Rng(31);
        let mut worst = 0.0f64;
        let mut seam = 0.0f64;
        for i in 0..100 {
            let p = rng.int(1, 6);
            let (knots, mults, n) = random_periodic(&mut rng, p);
            let data = BSplineCurveData {
                degree: p as u8,
                knots: knots.clone(),
                mults,
                poles: (0..n).map(|_| rng.point(50.0)).collect(),
                weights: (i % 2 == 1).then(|| (0..n).map(|_| rng.range(0.5, 2.0)).collect()),
                periodic: true,
            };
            let ours = NurbsCurve3::try_from(&data).unwrap();
            assert_eq!(ours.domain(), (knots[0], knots[knots.len() - 1]));
            let ts: Vec<f64> = (0..50)
                .map(|_| rng.range(knots[0], knots[knots.len() - 1]))
                .collect();
            for (t, (pt, d1, d2)) in ts.iter().zip(oracle::curve_d2(&data, &ts).unwrap()) {
                let d = ours.derivs(*t, 2);
                worst = worst
                    .max(rel(d[0], pt))
                    .max(rel(d[1], d1))
                    .max(rel(d[2], d2));
            }
            // OCCT's own SetNotPeriodic gives the same curve (evaluated by OCCT).
            let flat = oracle::curve_not_periodic(&data).unwrap();
            assert!(!flat.periodic);
            for (t, (pt, _, _)) in ts.iter().zip(oracle::curve_d2(&flat, &ts).unwrap()) {
                worst = worst.max(rel(ours.eval(*t), pt));
            }
            // A trimmed range across the seam, one period up: OCCT evaluates periodic curves
            // anywhere on the line.
            let (k0, kl) = (knots[0], knots[knots.len() - 1]);
            let period = kl - k0;
            let (lo, hi) = (kl + 0.6 * period, kl + 1.3 * period);
            let piece = curve_seg_to_nurbs(&CurveSeg {
                curve: Curve3::BSpline(data.clone()),
                t0: hi,
                t1: lo,
                tag: Tag(0),
            })
            .unwrap();
            assert_eq!(piece.domain(), (lo, hi));
            let ts: Vec<f64> = (0..=20).map(|i| lo + (hi - lo) * i as f64 / 20.0).collect();
            for (t, (pt, _, _)) in ts.iter().zip(oracle::curve_d2(&data, &ts).unwrap()) {
                seam = seam.max(rel(piece.eval(lo + hi - t), pt));
            }
        }
        println!("periodic curves: max rel. error vs OCCT {worst:.2e}, across the seam {seam:.2e}");
        assert!(worst <= 1e-11, "{worst:e}");
        assert!(seam <= 1e-11, "{seam:e}");
    }

    #[test]
    fn periodic_surface_unwrap_matches_occt() {
        let mut rng = Rng(32);
        let mut worst = 0.0f64;
        for i in 0..60 {
            let (pu, pv) = (rng.int(1, 4), rng.int(1, 4));
            let (u_periodic, v_periodic) = match i % 3 {
                0 => (true, false),
                1 => (false, true),
                _ => (true, true),
            };
            let (u_knots, u_mults, nu) = if u_periodic {
                random_periodic(&mut rng, pu)
            } else {
                let k = {
                    let n = rng.int(pu + 1, pu + 5);
                    random_knots(&mut rng, pu, n)
                };
                let (k, m) = k.to_knots_mults(0.0);
                let n = m.iter().sum::<usize>() - pu - 1;
                (k, m.into_iter().map(|m| m as u32).collect(), n)
            };
            let (v_knots, v_mults, nv) = if v_periodic {
                random_periodic(&mut rng, pv)
            } else {
                let k = {
                    let n = rng.int(pv + 1, pv + 5);
                    random_knots(&mut rng, pv, n)
                };
                let (k, m) = k.to_knots_mults(0.0);
                let n = m.iter().sum::<usize>() - pv - 1;
                (k, m.into_iter().map(|m| m as u32).collect(), n)
            };
            let data = BSplineSurfaceData {
                u_degree: pu as u8,
                v_degree: pv as u8,
                u_knots: u_knots.clone(),
                u_mults,
                v_knots: v_knots.clone(),
                v_mults,
                poles: (0..nu * nv).map(|_| rng.point(50.0)).collect(),
                nu: nu as u32,
                nv: nv as u32,
                weights: (i % 2 == 1).then(|| (0..nu * nv).map(|_| rng.range(0.5, 2.0)).collect()),
                u_periodic,
                v_periodic,
            };
            let ours = NurbsSurface3::try_from(&data).unwrap();
            let uvs: Vec<(f64, f64)> = (0..50)
                .map(|_| {
                    (
                        rng.range(u_knots[0], u_knots[u_knots.len() - 1]),
                        rng.range(v_knots[0], v_knots[v_knots.len() - 1]),
                    )
                })
                .collect();
            for (&(u, v), o) in uvs.iter().zip(oracle::surface_d2(&data, &uvs).unwrap()) {
                let d = ours.derivs(u, v, 2).d;
                worst = worst
                    .max(rel(d[0][0], o.p))
                    .max(rel(d[1][0], o.du))
                    .max(rel(d[0][1], o.dv));
            }
            // A box one period up in each periodic direction, inside one period.
            let shift = |k: &[f64], periodic: bool| {
                let (a, b) = (k[0], k[k.len() - 1]);
                let t = if periodic { b - a } else { 0.0 };
                (a + 0.2 * (b - a) + t, a + 0.7 * (b - a) + t)
            };
            let uv = [shift(&u_knots, u_periodic), shift(&v_knots, v_periodic)];
            let piece = surface_to_nurbs(&Surface::BSpline(data.clone()), uv).unwrap();
            assert_eq!(piece.domain(), uv);
            let uvs: Vec<(f64, f64)> = (0..25)
                .map(|i| {
                    let (a, b) = (i % 5, i / 5);
                    (
                        uv[0].0 + (uv[0].1 - uv[0].0) * f64::from(a) / 4.0,
                        uv[1].0 + (uv[1].1 - uv[1].0) * f64::from(b) / 4.0,
                    )
                })
                .collect();
            for (&(u, v), o) in uvs.iter().zip(oracle::surface_d2(&data, &uvs).unwrap()) {
                worst = worst.max(rel(piece.eval(u, v), o.p));
            }
        }
        println!("periodic surfaces: max rel. error vs OCCT {worst:.2e}");
        assert!(worst <= 1e-11, "{worst:e}");
    }

    /// Done-when 3 on the faces and edges of one primitive: returns the max radius error / r.
    fn check_shape(k: &OcctKernel, what: &str, shape: &Shape) -> f64 {
        let mut worst = 0.0f64;
        for f in shape.faces() {
            let s = shape.surface(f);
            let [u0, u1, v0, v1] = shape.face_uv_bounds(f);
            let n = surface_to_nurbs(&s, [(u0, u1), (v0, v1)]).unwrap();
            for i in 0..=20 {
                for j in 0..=20 {
                    let u = u0 + (u1 - u0) * i as f64 / 20.0;
                    let v = v0 + (v1 - v0) * j as f64 / 20.0;
                    let (e, r) = surface_error(&s, n.eval(u, v));
                    worst = worst.max(e / r);
                }
            }
            // The patch is the face's: corners coincide with OCCT's evaluation (or, where OCCT
            // has no curvature to report, as at a cone apex, with its parametrisation).
            for (u, v) in [(u0, v0), (u1, v0), (u0, v1), (u1, v1)] {
                let occ = k
                    .eval_surface(shape, f, Point2::new(u, v))
                    .map_or_else(|_| surface_at(&s, u, v), |p| p.point);
                let d = (n.eval(u, v) - occ).length();
                assert!(d <= 1e-11, "{what} face {f:?} corner ({u}, {v}): {d:e}");
            }
        }
        for e in shape.edges() {
            let Some((c, t0, t1)) = shape.curve(e) else {
                continue; // degenerated (sphere poles, cone apex)
            };
            let n = edge_to_nurbs(shape, e).unwrap();
            for i in 0..=100 {
                let (a, b) = n.domain();
                let (err, r) = curve_error(&c, n.eval(a + (b - a) * i as f64 / 100.0));
                worst = worst.max(err / r);
            }
            let ends = [c.point_at(t0).unwrap(), c.point_at(t1).unwrap()];
            assert!((n.start() - ends[0]).length() <= 1e-11, "{what} edge {e:?}");
            assert!((n.end() - ends[1]).length() <= 1e-11, "{what} edge {e:?}");
        }
        worst
    }

    #[test]
    fn occt_analytic_primitives_convert_exactly() {
        let k = kernel();
        let f = tilted();
        let shapes = [
            (
                "box",
                k.make_box(&f, Vec3::new(4.0, 7.0, 2.5)).unwrap().shape,
            ),
            (
                "cylinder",
                k.make_cylinder(&f, 7.0, 12.0, TAU).unwrap().shape,
            ),
            (
                "cylinder 2 rad",
                k.make_cylinder(&f, 3.0, 5.0, 2.0).unwrap().shape,
            ),
            ("cone", k.make_cone(&f, 4.0, 1.5, 6.0, TAU).unwrap().shape),
            (
                "cone apex",
                k.make_cone(&f, 4.0, 0.0, 6.0, PI).unwrap().shape,
            ),
            ("sphere", k.make_sphere(&f, 5.0).unwrap().shape),
            ("torus", k.make_torus(&f, 10.0, 3.0, TAU).unwrap().shape),
            (
                "torus 3 rad",
                k.make_torus(&f, 10.0, 3.0, 3.0).unwrap().shape,
            ),
        ];
        for (what, shape) in &shapes {
            let worst = check_shape(&k, what, shape);
            println!("{what}: max radius error {worst:.2e}·r");
            assert!(worst <= 1e-12, "{what}: {worst:e}");
        }
    }

    #[test]
    fn occt_analytic_ellipses_convert_exactly() {
        let k = kernel();
        let f = tilted();
        // An ellipse edge made by OCCT (`make_wire`) and read back.
        let seg = CurveSeg {
            curve: Curve3::Ellipse {
                frame: f,
                major_radius: 9.0,
                minor_radius: 2.5,
            },
            t0: 0.4,
            t1: 2.9,
            tag: Tag(1),
        };
        let wire = k.make_wire(&[seg], &Tolerance::DEFAULT).unwrap().shape;
        let mut worst = 0.0f64;
        let mut ellipses = 0;
        for e in wire.edges() {
            let (c, _, _) = wire.curve(e).unwrap();
            assert!(matches!(c, Curve3::Ellipse { .. }), "{c:?}");
            let n = edge_to_nurbs(&wire, e).unwrap();
            let (a, b) = n.domain();
            for i in 0..=200 {
                let (err, r) = curve_error(&c, n.eval(a + (b - a) * i as f64 / 200.0));
                worst = worst.max(err / r);
            }
            ellipses += 1;
        }
        // An oblique section of a cylinder: OCCT reports ellipses.
        let cyl = k.make_cylinder(&Plane::XY, 4.0, 20.0, TAU).unwrap().shape;
        let cut = Plane::from_normal(
            Point3::new(0.0, 0.0, 10.0),
            amg_brep::Dir3::new(Vec3::new(0.0, 0.5, 1.0), &Tolerance::DEFAULT).unwrap(),
        );
        for seg in k.plane_section(&cyl, &cut).unwrap() {
            if let Curve3::Ellipse { .. } = &seg.curve {
                let n = curve_seg_to_nurbs(&seg).unwrap();
                let (a, b) = n.domain();
                for i in 0..=200 {
                    let (err, r) = curve_error(&seg.curve, n.eval(a + (b - a) * i as f64 / 200.0));
                    worst = worst.max(err / r);
                }
                ellipses += 1;
            }
        }
        println!("{ellipses} OCCT ellipses: max radius error {worst:.2e}·r");
        assert!(ellipses >= 2, "the section gave no ellipse");
        assert!(worst <= 1e-12, "{worst:e}");
    }

    /// P&T §10.3 skinning: compatible section interpolants (same parameters, so same knots),
    /// then the pole columns interpolated across the sections.
    fn skinned_patch(rational: bool) -> NurbsSurface3 {
        let n_pts = 7;
        let params: Vec<f64> = (0..n_pts).map(|i| i as f64 / (n_pts - 1) as f64).collect();
        let sections: Vec<NurbsCurve3> = (0..4)
            .map(|k| {
                let z = 10.0 * k as f64;
                let pts: Vec<Point3> = params
                    .iter()
                    .map(|&t| {
                        let a = t * PI;
                        let r = 20.0 + 3.0 * k as f64 + 2.0 * (3.0 * a + k as f64).sin();
                        Point3::new(r * a.cos(), r * a.sin(), z + 1.5 * (2.0 * a).cos())
                    })
                    .collect();
                interpolate(
                    &pts,
                    &InterpOpts {
                        degree: 3,
                        params: Some(params.clone()),
                        ..InterpOpts::default()
                    },
                )
                .unwrap()
            })
            .collect();
        let nu = sections[0].poles().len();
        let vparams = vec![0.0, 0.3, 0.7, 1.0];
        let columns: Vec<NurbsCurve3> = (0..nu)
            .map(|i| {
                let pts: Vec<Point3> = sections.iter().map(|c| c.poles()[i]).collect();
                interpolate(
                    &pts,
                    &InterpOpts {
                        degree: 3,
                        params: Some(vparams.clone()),
                        ..InterpOpts::default()
                    },
                )
                .unwrap()
            })
            .collect();
        let nv = columns[0].poles().len();
        let poles = Grid::new(
            nu,
            nv,
            (0..nu).flat_map(|i| columns[i].poles().to_vec()).collect(),
        )
        .unwrap();
        let weights = rational.then(|| {
            Grid::new(
                nu,
                nv,
                (0..nu * nv)
                    .map(|k| 1.0 + 0.4 * ((k as f64) * 0.7).sin())
                    .collect(),
            )
            .unwrap()
        });
        NurbsSurface3::new(
            [3, 3],
            [sections[0].knots().clone(), columns[0].knots().clone()],
            poles,
            weights,
        )
        .unwrap()
    }

    #[test]
    fn nurbs_face_on_skinned_patch() {
        let k = kernel();
        let tol = Tolerance::DEFAULT;
        for rational in [false, true] {
            let s = skinned_patch(rational);
            let [(ua, ub), (va, vb)] = s.domain();
            let bounds = [
                FaceBounds::Natural,
                FaceBounds::UvRect {
                    u0: ua + 0.1 * (ub - ua),
                    u1: ua + 0.8 * (ub - ua),
                    v0: va + 0.2 * (vb - va),
                    v1: vb,
                },
            ];
            for b in &bounds {
                let built = nurbs_face(&k, &s, b, 1e-7).unwrap();
                let face = &built.face;
                let report = k.check(face, &CheckOpts::default());
                assert!(report.valid, "{report:?}");
                // The face carries our data bit-identically.
                assert_eq!(
                    face.surface(FaceId(0)),
                    Surface::BSpline(BSplineSurfaceData::from(&s))
                );
                let q = TessQuality {
                    linear: 0.01,
                    angular: 0.1,
                    relative: false,
                    min_size: 0.0,
                    max_linear_abs: None,
                };
                let mesh = k.tessellate(face, &q, &JobCtx::detached()).unwrap();
                let mut worst = 0.0f64;
                for &p in &mesh.positions {
                    worst = worst.max(s.closest_point(p, None, &tol).distance);
                }
                println!(
                    "skinned patch (rational {rational}, {b:?}): {} vertices, max distance \
                     {worst:.2e} mm",
                    mesh.positions.len()
                );
                assert!(!mesh.positions.is_empty());
                assert!(worst <= 1e-6, "{worst:e}");
            }
        }
    }

    #[test]
    fn wire_to_chain_follows_the_wire() {
        let k = kernel();
        let spline = NurbsCurve3::new(
            3,
            KnotVector::new(vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]).unwrap(),
            vec![
                Point3::new(15.0, 5.0, 0.0),
                Point3::new(18.0, 8.0, 2.0),
                Point3::new(18.0, 12.0, -1.0),
                Point3::new(15.0, 15.0, 0.0),
            ],
            None,
        )
        .unwrap();
        let segs = vec![
            CurveSeg {
                curve: Curve3::Line {
                    origin: Point3::ZERO,
                    dir: amg_brep::Dir3::X,
                },
                t0: 0.0,
                t1: 10.0,
                tag: Tag(1),
            },
            CurveSeg {
                curve: Curve3::Circle {
                    frame: Plane {
                        origin: Point3::new(10.0, 5.0, 0.0),
                        ..Plane::XY
                    },
                    radius: 5.0,
                },
                t0: -FRAC_PI_2,
                t1: 0.0,
                tag: Tag(2),
            },
            CurveSeg {
                curve: Curve3::BSpline((&spline).into()),
                t0: 0.0,
                t1: 1.0,
                tag: Tag(3),
            },
            // Reversed: runs from x = 15 back to x = 0.
            CurveSeg {
                curve: Curve3::Line {
                    origin: Point3::new(0.0, 15.0, 0.0),
                    dir: amg_brep::Dir3::X,
                },
                t0: 15.0,
                t1: 0.0,
                tag: Tag(4),
            },
            CurveSeg {
                curve: Curve3::Line {
                    origin: Point3::new(0.0, 15.0, 0.0),
                    dir: amg_brep::Dir3::new(Vec3::new(0.0, -1.0, 0.0), &Tolerance::DEFAULT)
                        .unwrap(),
                },
                t0: 0.0,
                t1: 15.0,
                tag: Tag(5),
            },
        ];
        let wire = k.make_wire(&segs, &Tolerance::DEFAULT).unwrap().shape;
        let chain = wire_to_chain(&wire).unwrap();
        assert_eq!(chain.len(), segs.len());
        // The chain is connected and closed.
        for i in 0..chain.len() {
            let gap = (chain[i].end() - chain[(i + 1) % chain.len()].start()).length();
            assert!(gap <= 1e-7, "gap {gap:e} after {i}");
        }
        // It runs the input segments in order, from where the wire starts.
        let first = (0..segs.len())
            .find(|&i| (chain[0].start() - segs[i].start().unwrap()).length() <= 1e-7)
            .expect("chain starts at a segment start");
        for (j, c) in chain.iter().enumerate() {
            let seg = &segs[(first + j) % segs.len()];
            let expect = curve_seg_to_nurbs(seg).unwrap();
            for i in 0..=20 {
                let (a, b) = c.domain();
                let p = c.eval(a + (b - a) * i as f64 / 20.0);
                let d = expect.closest_point(p, &Tolerance::DEFAULT).distance;
                assert!(d <= 1e-7, "segment {}: {d:e}", (first + j) % segs.len());
            }
            assert!((c.start() - seg.start().unwrap()).length() <= 1e-7);
            assert!((c.end() - seg.end().unwrap()).length() <= 1e-7);
        }
        // An edge gives itself; a solid is rejected.
        let edge = wire.sub_shape(amg_brep::SubShape {
            kind: amg_brep::SubKind::Edge,
            index: 0,
        });
        assert_eq!(wire_to_chain(&edge.unwrap()).unwrap().len(), 1);
        let solid = k.make_box(&Plane::XY, Vec3::ONE).unwrap().shape;
        assert!(matches!(
            wire_to_chain(&solid),
            Err(KernelError::InvalidInput { .. })
        ));
        assert!(edge_to_nurbs(&solid, EdgeId(1000)).is_err());
    }

    /// Done-when 5 (oracle part): `GeomFill_Gordon` through the iso-curves of a bicubic patch.
    /// The KH-15 comparison (amg-nurbs Gordon vs this oracle on the TiGL networks) lands with
    /// KH-15.
    #[test]
    fn gordon_oracle_reproduces_network() {
        let s = skinned_patch(false);
        let [(ua, ub), (va, vb)] = s.domain();
        let profiles: Vec<NurbsCurve3> = (0..4)
            .map(|i| s.iso_curve(UV::U, ua + (ub - ua) * i as f64 / 3.0))
            .collect();
        let guides: Vec<NurbsCurve3> = (0..4)
            .map(|j| s.iso_curve(UV::V, va + (vb - va) * j as f64 / 3.0))
            .collect();
        let start = Instant::now();
        let g = oracle::gordon(&profiles, &guides, 1e-6).unwrap();
        let elapsed = start.elapsed();
        assert_eq!(g.status, oracle::GordonOracle::DONE, "{g:?}");
        assert!(!g.approximate);
        let gs = g.surface.as_ref().unwrap();
        let tol = Tolerance::DEFAULT;
        let mut worst = 0.0f64;
        for c in profiles.iter().chain(&guides) {
            let (a, b) = c.domain();
            for i in 0..=20 {
                let p = c.eval(a + (b - a) * i as f64 / 20.0);
                worst = worst.max(gs.closest_point(p, None, &tol).distance);
            }
        }
        println!(
            "GeomFill_Gordon: 4 x 4 network reproduced within {worst:.2e} mm (OCCT reports \
             profile {:.2e}, guide {:.2e}), {:?}",
            g.max_profile_deviation, g.max_guide_deviation, elapsed
        );
        assert!(worst <= 1e-6, "{worst:e}");
    }
}

#[cfg(not(feature = "occt"))]
#[test]
fn oracles_need_occt() {
    use amg_brep::nurbs_conv::oracle;
    let c = BSplineCurveData::from(&amg_nurbs::analytic::line(Point3::ZERO, Point3::X));
    assert!(matches!(
        oracle::round_trip_curve(&c),
        Err(KernelError::BackendUnavailable)
    ));
    assert!(matches!(
        oracle::gordon(&[], &[], 1e-6),
        Err(KernelError::BackendUnavailable)
    ));
}
