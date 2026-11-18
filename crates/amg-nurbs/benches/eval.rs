//! KH-07 Done-when 6 (S20 / nurbs note §9 budgets): cubic curve point < 60 ns, bicubic surface
//! point + first derivatives < 250 ns; also rational surface point + normal (< 400 ns, KH-07
//! reports it, the budget is shared with later WPs). Run with
//! `cargo bench -p amg-nurbs --bench eval`. Medians are printed always and asserted only under
//! `AMG_STRICT_TIMING=1`.

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::print_stderr,
    clippy::print_stdout,
    missing_docs,
    reason = "benchmark setup"
)]

use std::hint::black_box;
use std::time::Instant;

use amg_core::{Point3, Tolerance};
use amg_nurbs::{Grid, KnotVector, NurbsCurve3, NurbsSurface3, analytic};
use criterion::{Criterion, criterion_group, criterion_main};

fn cubic_curve() -> NurbsCurve3 {
    let n = 12;
    let poles = (0..n)
        .map(|i| {
            let t = i as f64;
            Point3::new(t, (0.7 * t).sin() * 5.0, (0.3 * t).cos() * 3.0)
        })
        .collect();
    NurbsCurve3::new(3, KnotVector::clamped_uniform(3, n).unwrap(), poles, None).unwrap()
}

fn bicubic_surface() -> NurbsSurface3 {
    let (nu, nv) = (8, 8);
    let data = (0..nu * nv)
        .map(|k| {
            let (i, j) = ((k / nv) as f64, (k % nv) as f64);
            Point3::new(i, j, (0.5 * i).sin() + (0.4 * j).cos())
        })
        .collect();
    NurbsSurface3::new(
        [3, 3],
        [
            KnotVector::clamped_uniform(3, nu).unwrap(),
            KnotVector::clamped_uniform(3, nv).unwrap(),
        ],
        Grid::new(nu, nv, data).unwrap(),
        None,
    )
    .unwrap()
}

/// 1024 parameters in `[0, 1]` (golden-ratio sequence).
fn params() -> Vec<f64> {
    (0..1024)
        .map(|i| (i as f64 * 0.618_033_988_749_895).fract())
        .collect()
}

/// Median ns per call over 15 runs of all `params`.
fn median_ns(ts: &[f64], mut f: impl FnMut(f64)) -> f64 {
    for &t in ts {
        f(t);
    }
    let mut v: Vec<f64> = (0..15)
        .map(|_| {
            let s = Instant::now();
            for _ in 0..20 {
                for &t in ts {
                    f(black_box(t));
                }
            }
            s.elapsed().as_secs_f64() * 1e9 / (20 * ts.len()) as f64
        })
        .collect();
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

fn bench(cr: &mut Criterion) {
    let c = cubic_curve();
    let s = bicubic_surface();
    let sph = analytic::sphere(Point3::ZERO, 10.0).unwrap();
    let tol = Tolerance::DEFAULT;
    let ts = params();
    let [(u0, u1), (v0, v1)] = sph.domain();

    let curve_ns = median_ns(&ts, |t| {
        black_box(c.eval(t));
    });
    let surf_ns = median_ns(&ts, |t| {
        black_box(s.derivs(t, 1.0 - t, 1));
    });
    let rat_ns = median_ns(&ts, |t| {
        let (u, v) = (u0 + (u1 - u0) * t, v0 + (v1 - v0) * (0.1 + 0.8 * t));
        black_box(sph.eval(u, v));
        black_box(sph.normal(u, v, &tol));
    });
    println!(
        "amg-nurbs eval medians: cubic point {curve_ns:.1} ns (< 60), bicubic point + d1 \
         {surf_ns:.1} ns (< 250), rational sphere point + normal {rat_ns:.1} ns (< 400)"
    );
    if std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1") {
        assert!(curve_ns < 60.0, "cubic point {curve_ns:.1} ns");
        assert!(
            surf_ns < 250.0,
            "bicubic point + first derivatives {surf_ns:.1} ns"
        );
        assert!(rat_ns < 400.0, "rational point + normal {rat_ns:.1} ns");
    }

    let mut g = cr.benchmark_group("nurbs_eval");
    g.bench_function("cubic_curve_point", |b| {
        let mut i = 0;
        b.iter(|| {
            i = (i + 1) & 1023;
            black_box(c.eval(black_box(ts[i])))
        });
    });
    g.bench_function("bicubic_point_d1", |b| {
        let mut i = 0;
        b.iter(|| {
            i = (i + 1) & 1023;
            black_box(s.derivs(black_box(ts[i]), black_box(1.0 - ts[i]), 1))
        });
    });
    g.bench_function("rational_sphere_point_normal", |b| {
        let mut i = 0;
        b.iter(|| {
            i = (i + 1) & 1023;
            let (u, v) = (u0 + (u1 - u0) * ts[i], v0 + (v1 - v0) * (0.1 + 0.8 * ts[i]));
            black_box((sph.eval(u, v), sph.normal(u, v, &tol)))
        });
    });
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
