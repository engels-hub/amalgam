//! KH-13 Done-when 6 (S20 / nurbs note §9 budgets): interpolate 200 points with a cubic
//! < 100 µs; `approximate_to_tolerance` of 1000 points to 1e-5 mm < 5 ms (noisy helix, noise
//! 1e-6 mm). Also reported (no budget): the same fit on a noisy sine and circle arc, the
//! uncorrected reference fit, curve / surface closest point and arc length. Run with
//! `cargo bench -p amg-nurbs --bench fit`. Medians are printed always and asserted only under
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

use amg_core::{Plane, Point3, Tolerance};
use amg_nurbs::analytic;
use amg_nurbs::fit::{self, InterpOpts, Parametrization};
use criterion::{Criterion, criterion_group, criterion_main};

/// SplitMix64 noise in [-a, a].
fn noise(state: &mut u64, a: f64) -> f64 {
    *state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = *state;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^= z >> 31;
    a * (2.0 * ((z >> 11) as f64 / (1u64 << 53) as f64) - 1.0)
}

fn sampled(n: usize, amp: f64, f: impl Fn(f64) -> Point3) -> Vec<Point3> {
    let mut st = 0x13_0006u64;
    (0..n)
        .map(|i| {
            let p = f(i as f64 / (n - 1) as f64);
            p + Point3::new(
                noise(&mut st, amp),
                noise(&mut st, amp),
                noise(&mut st, amp),
            )
        })
        .collect()
}

fn helix(s: f64) -> Point3 {
    let a = s * 2.0 * std::f64::consts::TAU;
    Point3::new(
        10.0 * a.cos(),
        10.0 * a.sin(),
        4.0 * a / std::f64::consts::TAU,
    )
}

/// Median µs per call over 31 runs of `reps` calls.
fn median_us(reps: usize, mut f: impl FnMut()) -> f64 {
    for _ in 0..reps {
        f();
    }
    let mut v: Vec<f64> = (0..31)
        .map(|_| {
            let s = Instant::now();
            for _ in 0..reps {
                f();
            }
            s.elapsed().as_secs_f64() * 1e6 / reps as f64
        })
        .collect();
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

fn bench(cr: &mut Criterion) {
    let p200 = sampled(200, 0.0, |s| {
        let x = 100.0 * s;
        Point3::new(x, 10.0 * (x * 0.2).sin(), 5.0 * (x * 0.05).cos())
    });
    let o = InterpOpts::<Point3>::default();
    let helix1000 = sampled(1000, 1e-6, helix);
    let sine1000 = sampled(1000, 1e-6, |s| {
        let x = 40.0 * s;
        Point3::new(x, 5.0 * (x * std::f64::consts::TAU / 10.0).sin(), 0.0)
    });
    let arc1000 = sampled(1000, 1e-6, |s| {
        let a = s * 350f64.to_radians();
        Point3::new(10.0 * a.cos(), 10.0 * a.sin(), 0.0)
    });
    let m = Parametrization::Centripetal;
    let fit_helix = || fit::approximate_to_tolerance(&helix1000, 3, 1e-5, 500, m).unwrap();

    let interp_us = median_us(200, || {
        black_box(fit::interpolate(black_box(&p200), &o).unwrap());
    });
    let fit_us = median_us(5, || {
        black_box(fit_helix());
    });
    let fit_sine_us = median_us(5, || {
        black_box(fit::approximate_to_tolerance(&sine1000, 3, 1e-5, 500, m).unwrap());
    });
    let fit_arc_us = median_us(5, || {
        black_box(fit::approximate_to_tolerance(&arc1000, 3, 1e-5, 500, m).unwrap());
    });
    let ref_us = median_us(5, || {
        black_box(fit::approximate_to_tolerance_uncorrected(&helix1000, 3, 1e-5, 500, m).unwrap());
    });
    let fh = fit_helix();
    let tol = Tolerance::DEFAULT;
    let qc = Point3::new(3.0, -4.0, 2.5);
    let cp_curve_us = median_us(200, || {
        black_box(fh.curve.closest_point(black_box(qc), &tol));
    });
    let sph = analytic::sphere(Point3::ZERO, 5.0).unwrap();
    let tor = analytic::torus(&Plane::XY, 10.0, 3.0).unwrap();
    let qs = Point3::new(7.0, 3.0, 4.0);
    let cp_sphere_us = median_us(200, || {
        black_box(sph.closest_point(black_box(qs), None, &tol));
    });
    let cp_torus_us = median_us(200, || {
        black_box(tor.closest_point(black_box(qs), None, &tol));
    });
    let arc = analytic::circle_arc(&Plane::XY, 10.0, 0.0, 5.0).unwrap();
    let len_us = median_us(200, || {
        black_box(arc.arc_length(None, black_box(1e-10)));
    });
    let inv_us = median_us(200, || {
        black_box(arc.param_at_length(black_box(31.0), 1e-10));
    });
    println!(
        "amg-nurbs fit medians: interpolate 200 points (cubic, centripetal) {interp_us:.1} µs \
         (< 100); approximate_to_tolerance 1000 points to 1e-5 mm: helix {:.2} ms (< 5, {} \
         poles), sine {:.2} ms, circle arc {:.2} ms; uncorrected reference (helix) {:.2} ms; \
         closest point: 131-pole helix fit {cp_curve_us:.1} µs, sphere {cp_sphere_us:.1} µs, \
         torus {cp_torus_us:.1} µs; arc length of a 5 rad arc {len_us:.1} µs, inverse \
         {inv_us:.1} µs",
        fit_us / 1e3,
        fh.curve.poles().len(),
        fit_sine_us / 1e3,
        fit_arc_us / 1e3,
        ref_us / 1e3,
    );
    if std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1") {
        assert!(
            interp_us < 100.0,
            "interpolate 200 points {interp_us:.1} µs"
        );
        assert!(fit_us < 5000.0, "fit 1000 points {:.2} ms", fit_us / 1e3);
    }

    let mut g = cr.benchmark_group("nurbs_fit");
    g.bench_function("interpolate_200", |b| {
        b.iter(|| black_box(fit::interpolate(black_box(&p200), &o).unwrap()));
    });
    g.sample_size(20);
    g.bench_function("approximate_1000_to_1e-5", |b| {
        b.iter(|| black_box(fit_helix()));
    });
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
