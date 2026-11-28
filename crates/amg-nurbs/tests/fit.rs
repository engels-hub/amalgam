//! KH-13 Done-when 1–3 and 5: interpolation accuracy (random sweep + P&T Ex. 9.1),
//! approximation to tolerance with and without Hoschek correction on noisy circles, helices
//! and sine curves, reported vs measured error, constrained least squares, approximation onto
//! knots, and arc length. Measured worst values are printed (`--nocapture`).
#![allow(
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::print_stdout,
    clippy::needless_range_loop,
    reason = "test code; the dense KKT reference keeps textbook index loops"
)]

mod common;

use amg_core::{Plane, Point2, Point3};
use amg_nurbs::fit::{
    self, EndDerivs, InterpOpts, LsqOpts, OntoOpts, Parametrization, approximate_lsq,
    approximate_onto_knots, approximate_to_tolerance, interpolate,
};
use amg_nurbs::{CtrlPt, KnotVector, NurbsCurve, NurbsCurve3, NurbsError, analytic};
use common::{Rng, diag};

const PARAMS: [Parametrization; 3] = [
    Parametrization::Uniform,
    Parametrization::ChordLength,
    Parametrization::Centripetal,
];

fn rel(a: Point3, b: Point3) -> f64 {
    (a - b).length() / b.length().max(1e-300)
}

/// Done-when 1: data reproduced ≤ 1e-12·max(1, D), end derivatives within 1e-10 relative,
/// over 3000 random open / closed / derivative-constrained cases (degree 1–5).
#[test]
fn interpolation_reproduces_data_and_end_derivatives() {
    let mut r = Rng(0x13_0001);
    let (mut worst_pt, mut worst_d1, mut worst_d2, mut worst_closed) =
        (0.0f64, 0.0f64, 0.0f64, 0.0f64);
    let mut cases = [0usize; 4];
    for case in 0..3000 {
        let p = r.usize(1, 5);
        let n = r.usize(p + 1, 40);
        // A random walk keeps consecutive points apart (the data of a real section).
        let mut pts = Vec::with_capacity(n);
        let mut x = r.point(50.0);
        for _ in 0..n {
            x += r.point(10.0) + Point3::new(1.0, 0.0, 0.0);
            pts.push(x);
        }
        let d = diag(&pts).max(1.0);
        let m = PARAMS[case % 3];
        let kind = case % 4;
        let mut o = InterpOpts {
            degree: p,
            parametrization: m,
            ..InterpOpts::default()
        };
        match kind {
            1 if p >= 2 => {
                o.ends.start = Some(r.point(30.0));
                o.ends.end = Some(r.point(30.0));
            }
            2 if p >= 3 => {
                o.ends = EndDerivs {
                    start: Some(r.point(30.0)),
                    end: Some(r.point(30.0)),
                    start2: Some(r.point(100.0)),
                    end2: Some(r.point(100.0)),
                };
            }
            3 => o.closed = true,
            _ => {}
        }
        if o.closed && n < p + 1 {
            continue;
        }
        let c = interpolate(&pts, &o).unwrap();
        cases[kind] += 1;
        let u = if o.closed {
            let mut lp = pts.clone();
            lp.push(pts[0]);
            fit::parametrize(&lp, m)
        } else {
            fit::parametrize(&pts, m)
        };
        for (q, &t) in pts.iter().zip(&u) {
            let e = (c.eval(t) - *q).length() / d;
            if o.closed {
                worst_closed = worst_closed.max(e);
            } else {
                worst_pt = worst_pt.max(e);
            }
            assert!(e <= 1e-12, "case {case} p={p} kind={kind}: {e:e}");
        }
        let (a, b) = c.domain();
        let (da, db) = (c.derivs(a, 2), c.derivs(b, 2));
        for (got, want) in [(da[1], o.ends.start), (db[1], o.ends.end)] {
            if let Some(w) = want {
                let e = rel(got, w);
                worst_d1 = worst_d1.max(e);
                assert!(e <= 1e-10, "case {case}: D1 {e:e}");
            }
        }
        for (got, want) in [(da[2], o.ends.start2), (db[2], o.ends.end2)] {
            if let Some(w) = want {
                let e = rel(got, w);
                worst_d2 = worst_d2.max(e);
                assert!(e <= 1e-10, "case {case}: D2 {e:e}");
            }
        }
        if o.closed {
            assert_eq!(c.start(), c.end());
        }
    }
    println!(
        "interpolation: cases {cases:?}; points {worst_pt:.2e}·D, closed {worst_closed:.2e}·D, \
         D1 {worst_d1:.2e}, D2 {worst_d2:.2e} relative"
    );
}

/// Done-when 1: P&T Ex. 9.1 (chord length, cubic) gives the book's parameters
/// {0, 5/17, 9/17, 14/17, 1} and knots {0, 0, 0, 0, 28/51, 1, 1, 1, 1}, and the solved curve
/// reproduces the five data points.
#[test]
fn example_9_1() {
    let q = [
        Point2::new(0., 0.),
        Point2::new(3., 4.),
        Point2::new(-1., 4.),
        Point2::new(-4., 0.),
        Point2::new(-4., -3.),
    ];
    let u = fit::parametrize(&q, Parametrization::ChordLength);
    for (a, b) in u
        .iter()
        .zip([0.0, 5.0 / 17.0, 9.0 / 17.0, 14.0 / 17.0, 1.0])
    {
        assert!((a - b).abs() <= 1e-15);
    }
    let k = fit::averaging_knots(&u, 3);
    for (a, b) in k
        .as_slice()
        .iter()
        .zip([0., 0., 0., 0., 28.0 / 51.0, 1., 1., 1., 1.])
    {
        assert!((a - b).abs() <= 1e-15);
    }
    let c = interpolate(
        &q,
        &InterpOpts {
            degree: 3,
            parametrization: Parametrization::ChordLength,
            ..InterpOpts::default()
        },
    )
    .unwrap();
    assert_eq!(c.knots(), &k);
    for (pt, &t) in q.iter().zip(&u) {
        assert!((c.eval(t) - *pt).length() <= 1e-12 * 8.0);
    }
}

// ---- approximation to tolerance -------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
enum Shape {
    Circle,
    Helix,
    Sine,
}

/// 1000 samples of the shape (mm) with uniform noise of amplitude `noise` per coordinate.
fn noisy(shape: Shape, noise: f64, seed: u64) -> Vec<Point3> {
    let mut r = Rng(seed);
    (0..1000)
        .map(|i| {
            let s = i as f64 / 999.0;
            let base = match shape {
                Shape::Circle => {
                    // Open arc of 350° (a closed circle has coincident ends).
                    let a = s * 350f64.to_radians();
                    Point3::new(10.0 * a.cos(), 10.0 * a.sin(), 0.0)
                }
                Shape::Helix => {
                    let a = s * 2.0 * std::f64::consts::TAU;
                    Point3::new(
                        10.0 * a.cos(),
                        10.0 * a.sin(),
                        4.0 * a / std::f64::consts::TAU,
                    )
                }
                Shape::Sine => {
                    let x = s * 40.0;
                    Point3::new(x, 5.0 * (x * std::f64::consts::TAU / 10.0).sin(), 0.0)
                }
            };
            base + Point3::new(
                r.range(-noise, noise),
                r.range(-noise, noise),
                r.range(-noise, noise),
            )
        })
        .collect()
}

/// Independent point-to-curve distance: nearest of `n` uniform samples, then golden-section
/// search between its neighbours.
fn sampled_distance<P: CtrlPt>(c: &NurbsCurve<P>, samples: &[(f64, P)], q: P) -> f64 {
    let mut bi = 0;
    let mut bd = f64::INFINITY;
    for (i, &(_, s)) in samples.iter().enumerate() {
        let d = (s - q).length();
        if d < bd {
            bd = d;
            bi = i;
        }
    }
    let lo_i = bi.saturating_sub(1);
    let hi_i = (bi + 1).min(samples.len() - 1);
    let (mut lo, mut hi) = (samples[lo_i].0, samples[hi_i].0);
    let g = 0.5 * (5f64.sqrt() - 1.0);
    let f = |t: f64| (c.eval(t) - q).length();
    let mut x1 = hi - g * (hi - lo);
    let mut x2 = lo + g * (hi - lo);
    let (mut f1, mut f2) = (f(x1), f(x2));
    for _ in 0..80 {
        if f1 < f2 {
            hi = x2;
            x2 = x1;
            f2 = f1;
            x1 = hi - g * (hi - lo);
            f1 = f(x1);
        } else {
            lo = x1;
            x1 = x2;
            f1 = f2;
            x2 = lo + g * (hi - lo);
            f2 = f(x2);
        }
    }
    bd.min(f1).min(f2)
}

fn curve_samples<P: CtrlPt>(c: &NurbsCurve<P>, n: usize) -> Vec<(f64, P)> {
    let (a, b) = c.domain();
    (0..n)
        .map(|i| {
            let t = a + (b - a) * i as f64 / (n - 1) as f64;
            (t, c.eval(t))
        })
        .collect()
}

/// Done-when 2 + 3: `approximate_to_tolerance` meets `tol` ∈ {1e-3, 1e-5} on 1000 noisy samples
/// (noise tol/10) of circles, helices and sine curves with at most 1.5× the poles of the
/// uncorrected reference; its `max_error` is within 1 % of the error measured independently
/// (2000 curve samples + golden-section polish per data point).
#[test]
fn approximate_to_tolerance_meets_tol_with_few_poles() {
    for (si, shape) in [Shape::Circle, Shape::Helix, Shape::Sine]
        .into_iter()
        .enumerate()
    {
        for tol in [1e-3, 1e-5] {
            let pts = noisy(shape, 0.1 * tol, 0x13_0100 + si as u64);
            let f =
                approximate_to_tolerance(&pts, 3, tol, 500, Parametrization::Centripetal).unwrap();
            let reference = fit::approximate_to_tolerance_uncorrected(
                &pts,
                3,
                tol,
                500,
                Parametrization::Centripetal,
            )
            .unwrap();
            let n = f.curve.poles().len();
            let nr = reference.curve.poles().len();
            assert!(f.max_error <= tol, "{shape:?} {tol}: {}", f.max_error);
            assert!(
                n as f64 <= 1.5 * nr as f64,
                "{shape:?} {tol}: {n} poles vs reference {nr}"
            );
            let samples = curve_samples(&f.curve, 2000);
            let measured = pts
                .iter()
                .map(|&q| sampled_distance(&f.curve, &samples, q))
                .fold(0.0f64, f64::max);
            let rel = (f.max_error - measured).abs() / measured;
            assert!(
                rel <= 0.01,
                "{shape:?} {tol}: reported {} measured {measured}",
                f.max_error
            );
            assert_eq!(f.curve.start(), pts[0]);
            assert_eq!(f.curve.end(), pts[999]);
            println!(
                "to_tolerance {shape:?} tol {tol:e}: {n} poles (reference {nr}, ratio {:.2}), \
                 max_error {:.3e}, measured {measured:.3e} (rel diff {rel:.1e}), rms {:.2e}",
                n as f64 / nr as f64,
                f.max_error,
                f.rms_error
            );
        }
    }
}

#[test]
fn approximate_to_tolerance_errors() {
    let pts = noisy(Shape::Sine, 0.0, 1);
    assert!(matches!(
        approximate_to_tolerance(&pts, 3, 1e-9, 8, Parametrization::Centripetal),
        Err(NurbsError::ToleranceNotMet { .. })
    ));
    assert!(matches!(
        approximate_to_tolerance(&pts, 3, 1e-3, 3, Parametrization::Centripetal),
        Err(NurbsError::TooManyPoles {
            needed: 4,
            limit: 3
        })
    ));
    assert!(matches!(
        approximate_to_tolerance(&pts, 3, 0.0, 30, Parametrization::Centripetal),
        Err(NurbsError::Degenerate(_))
    ));
    assert!(matches!(
        approximate_to_tolerance(&pts[..3], 3, 1e-3, 30, Parametrization::Centripetal),
        Err(NurbsError::Degenerate(_))
    ));
    assert!(matches!(
        approximate_to_tolerance(&pts, 0, 1e-3, 30, Parametrization::Centripetal),
        Err(NurbsError::Degree(0))
    ));
    // 2D works the same.
    let p2: Vec<Point2> = pts.iter().map(|p| Point2::new(p.x, p.y)).collect();
    let f = approximate_to_tolerance(&p2, 3, 1e-4, 200, Parametrization::ChordLength).unwrap();
    assert!(f.max_error <= 1e-4);
    // Determinism: bit-identical on a second run.
    let g = approximate_to_tolerance(&p2, 3, 1e-4, 200, Parametrization::ChordLength).unwrap();
    assert_eq!(f.curve, g.curve);
    assert_eq!(f.max_error.to_bits(), g.max_error.to_bits());
}

// ---- constrained least squares -------------------------------------------------------------

/// Dense KKT reference of A9.6 (Lagrange multipliers), for comparing the elimination path.
fn kkt_reference(
    pts: &[Point3],
    o: &LsqOpts<Point3>,
    cons: &[(f64, usize, Point3)],
) -> Vec<Point3> {
    let p = o.degree;
    let k = o.knots.as_slice();
    let n = k.len() - p - 1;
    let nc = cons.len();
    let dim = n + nc;
    let basis_row = |t: f64, order: usize| -> Vec<f64> {
        let span = o.knots.find_span(p, n, t);
        let mut d = [[0.0; amg_nurbs::MAX_DEGREE + 1]; 3];
        amg_nurbs::basis::ders_basis_funs(k, span, t, p, order, &mut d);
        let mut row = vec![0.0; n];
        for j in 0..=p {
            row[span - p + j] = d[order][j];
        }
        row
    };
    let mut a = vec![vec![0.0; dim]; dim];
    let mut rhs = vec![Point3::ZERO; dim];
    for (q, (&t, w)) in pts.iter().zip(
        o.params.iter().zip(
            o.data_weights
                .clone()
                .unwrap_or_else(|| vec![1.0; pts.len()]),
        ),
    ) {
        let row = basis_row(t, 0);
        for i in 0..n {
            rhs[i] += *q * (w * row[i]);
            for j in 0..n {
                a[i][j] += w * row[i] * row[j];
            }
        }
    }
    for (c, &(t, order, v)) in cons.iter().enumerate() {
        let row = basis_row(t, order);
        for j in 0..n {
            a[n + c][j] = row[j];
            a[j][n + c] = row[j];
        }
        rhs[n + c] = v;
    }
    // Gaussian elimination with partial pivoting.
    for col in 0..dim {
        let piv = (col..dim)
            .max_by(|&x, &y| a[x][col].abs().total_cmp(&a[y][col].abs()))
            .unwrap();
        a.swap(col, piv);
        rhs.swap(col, piv);
        for r in col + 1..dim {
            let f = a[r][col] / a[col][col];
            for j in col..dim {
                a[r][j] -= f * a[col][j];
            }
            let rc = rhs[col];
            rhs[r] -= rc * f;
        }
    }
    let mut x = vec![Point3::ZERO; dim];
    for r in (0..dim).rev() {
        let mut s = rhs[r];
        for j in r + 1..dim {
            s -= x[j] * a[r][j];
        }
        x[r] = s / a[r][r];
    }
    x.truncate(n);
    x
}

#[test]
fn constrained_lsq_matches_kkt_and_meets_constraints() {
    let pts = noisy(Shape::Helix, 1e-3, 7);
    let params = fit::parametrize(&pts, Parametrization::Centripetal);
    let knots = fit::approximation_knots(&params, 3, 30);
    let mut r = Rng(9);
    let weights: Vec<f64> = (0..pts.len()).map(|_| r.range(0.5, 2.0)).collect();
    let d0 = Point3::new(1.0, 60.0, 8.0);
    let d1 = Point3::new(-2.0, 60.0, 8.0);
    let s2 = Point3::new(-200.0, 3.0, 0.0);
    let o = LsqOpts {
        degree: 3,
        knots: knots.clone(),
        params: params.clone(),
        fix_ends: true,
        ends: EndDerivs {
            start: Some(d0),
            end: Some(d1),
            start2: Some(s2),
            end2: None,
        },
        data_weights: Some(weights),
    };
    let f = approximate_lsq(&pts, &o).unwrap();
    let cons = [
        (0.0, 0, pts[0]),
        (0.0, 1, d0),
        (0.0, 2, s2),
        (1.0, 0, pts[999]),
        (1.0, 1, d1),
    ];
    let want = kkt_reference(&pts, &o, &cons);
    let scale = diag(&want).max(1.0);
    let worst = f
        .curve
        .poles()
        .iter()
        .zip(&want)
        .map(|(a, b)| (*a - *b).length())
        .fold(0.0f64, f64::max);
    assert!(worst <= 1e-9 * scale, "elimination vs KKT: {worst:e}");
    let da = f.curve.derivs(0.0, 2);
    let db = f.curve.derivs(1.0, 2);
    assert_eq!(f.curve.start(), pts[0]);
    assert!((f.curve.end() - pts[999]).length() <= 1e-12 * scale);
    assert!(rel(da[1], d0) <= 1e-10 && rel(db[1], d1) <= 1e-10 && rel(da[2], s2) <= 1e-10);
    // Reported error = measured parametric error.
    let measured = pts
        .iter()
        .zip(&params)
        .map(|(q, &t)| (f.curve.eval(t) - *q).length())
        .fold(0.0f64, f64::max);
    assert_eq!(f.max_error, measured);
    println!("constrained LSQ vs dense KKT: {:.2e}·D", worst / scale);

    // Unconstrained + free ends, and error cases.
    let o2 = LsqOpts {
        fix_ends: false,
        ends: EndDerivs::default(),
        data_weights: None,
        ..o.clone()
    };
    let g = approximate_lsq(&pts, &o2).unwrap();
    assert!(g.rms_error <= f.rms_error);
    let bad = |o: LsqOpts<Point3>| approximate_lsq(&pts, &o).unwrap_err();
    assert!(matches!(
        bad(LsqOpts {
            params: params[..10].to_vec(),
            ..o2.clone()
        }),
        NurbsError::Degenerate(_)
    ));
    let mut out = params.clone();
    out[5] = 2.0;
    assert!(matches!(
        bad(LsqOpts {
            params: out,
            ..o2.clone()
        }),
        NurbsError::OutOfDomain(_)
    ));
    assert!(matches!(
        bad(LsqOpts {
            data_weights: Some(vec![0.0; 1000]),
            ..o2.clone()
        }),
        NurbsError::Weight { index: 0 }
    ));
    assert!(matches!(
        bad(LsqOpts {
            degree: 4,
            ..o2.clone()
        }),
        NurbsError::CountMismatch { .. } | NurbsError::InvalidKnots(_)
    ));
    // Data in one span only: singular.
    let few: Vec<f64> = (0..1000).map(|i| 0.01 * i as f64 / 999.0).collect();
    assert!(matches!(
        bad(LsqOpts {
            params: few,
            ..o2.clone()
        }),
        NurbsError::Singular(_)
    ));
    // Too many constraints for the poles.
    let tiny = LsqOpts {
        knots: KnotVector::bezier(3),
        ends: EndDerivs {
            start: Some(d0),
            end: Some(d1),
            start2: Some(s2),
            end2: Some(s2),
        },
        ..o.clone()
    };
    assert!(matches!(bad(tiny), NurbsError::Degenerate(_)));
}

// ---- approximate onto knots -----------------------------------------------------------------

/// Done-when 3 for `approximate_onto_knots`: reported `max_error` within 1 % of the deviation
/// measured at 2000 source samples; end points exact, end tangents kept.
#[test]
fn approximate_onto_knots_reports_its_error() {
    let src = analytic::circle_arc(&Plane::XY, 10.0, 0.3, 4.0).unwrap();
    for n in [8, 12, 20, 40] {
        let knots = KnotVector::clamped_uniform(3, n).unwrap();
        let o = OntoOpts {
            tol: 1.0,
            fix_end_tangents: true,
            ..OntoOpts::default()
        };
        let f = approximate_onto_knots(&src, 3, &knots, &o).unwrap();
        assert_eq!(f.curve.start(), src.start());
        assert_eq!(f.curve.end(), src.end());
        let scale = 4.0; // source domain length / target domain length
        let t_src = src.derivs(0.0, 1)[1] * scale;
        assert!(rel(f.curve.derivs(0.0, 1)[1], t_src) <= 1e-10);
        let samples = curve_samples(&f.curve, 2000);
        let (a, b) = src.domain();
        let measured = (0..2000)
            .map(|i| {
                let s = a + (b - a) * i as f64 / 1999.0;
                sampled_distance(&f.curve, &samples, src.eval(s))
            })
            .fold(0.0f64, f64::max);
        let rel_diff = (f.max_error - measured) / measured;
        assert!(
            rel_diff.abs() <= 0.01,
            "n={n}: reported {} measured {measured}",
            f.max_error
        );
        println!(
            "onto_knots circle arc, {n} poles: max_error {:.3e}, measured {measured:.3e} \
             (rel diff {rel_diff:.1e})",
            f.max_error
        );
    }
    let knots = KnotVector::clamped_uniform(3, 8).unwrap();
    assert!(matches!(
        approximate_onto_knots(
            &src,
            3,
            &knots,
            &OntoOpts {
                tol: 1e-9,
                ..OntoOpts::default()
            }
        ),
        Err(NurbsError::ToleranceNotMet { .. })
    ));
    // Exact reproduction when the target space contains the source (degree elevation).
    let cubic = NurbsCurve3::new(
        3,
        KnotVector::new(vec![0., 0., 0., 0., 0.4, 1., 1., 1., 1.]).unwrap(),
        vec![
            Point3::ZERO,
            Point3::new(1., 2., 0.),
            Point3::new(3., 2., 1.),
            Point3::new(4., 0., 1.),
            Point3::new(6., 1., 0.),
        ],
        None,
    )
    .unwrap();
    let f = approximate_onto_knots(
        &cubic,
        3,
        cubic.knots(),
        &OntoOpts {
            tol: 1e-12,
            ..OntoOpts::default()
        },
    )
    .unwrap();
    assert!(f.max_error <= 1e-12, "{}", f.max_error);
}

// ---- arc length ----------------------------------------------------------------------------

/// Done-when 5: circle arcs (rational quadratic, A7.1) are within 1e-12 relative of r·θ; the
/// inverse returns the parameter of a given length.
#[test]
fn arc_length_of_circle_arcs() {
    let mut worst = 0.0f64;
    let mut worst_inv = 0.0f64;
    for &r in &[0.01, 1.0, 10.0, 2500.0] {
        for &sweep in &[
            0.1,
            1.0,
            std::f64::consts::FRAC_PI_2,
            3.0,
            5.5,
            std::f64::consts::TAU,
        ] {
            let c = analytic::circle_arc(&Plane::XY, r, 0.7, sweep).unwrap();
            let want = r * sweep;
            let got = c.arc_length(None, 1e-13 * want);
            let e = ((got - want) / want).abs();
            worst = worst.max(e);
            assert!(e <= 1e-12, "r={r} sweep={sweep}: {e:e}");
            // Partial ranges: the knot domain of an arc is [0, |sweep|] but not arc-length
            // parametrised, so compare with the angle of the end points.
            let (a, b) = c.domain();
            let t = a + 0.37 * (b - a);
            let pt = c.eval(t);
            let ang = (pt.y.atan2(pt.x) - 0.7).rem_euclid(std::f64::consts::TAU);
            let got = c.arc_length(Some((a, t)), 1e-13 * want);
            let e = ((got - r * ang) / (r * ang)).abs();
            worst = worst.max(e);
            assert!(e <= 1e-12, "partial r={r} sweep={sweep}: {e:e}");
            for frac in [0.0, 0.1, 0.5, 0.93, 1.0] {
                let s = frac * want;
                let tt = c.param_at_length(s, 1e-12 * want);
                let back = c.arc_length(Some((a, tt)), 1e-13 * want);
                let e = (back - s).abs() / want;
                worst_inv = worst_inv.max(e);
                assert!(e <= 1e-11, "inverse r={r} sweep={sweep} frac={frac}: {e:e}");
            }
        }
    }
    println!("arc length: circle arcs {worst:.2e} relative; inverse {worst_inv:.2e}·L");
}
