//! KH-13 Done-when 4: curve and surface `closest_point` never exceed the dense-sampling
//! minimum plus 1e-9 mm. 64 random curves × 80 queries and 64 random surfaces × 80 queries
//! (10 240 queries; degree 1–7 curves, 1–4 surfaces, half rational). A third of the queries
//! are random points around the bounding box, a third lie up to 1 mm off the curve or
//! surface and a third on it. The sample set always includes the generating parameter, so
//! for the last two kinds the bound is the offset itself. Larger run:
//! `KH13_CLOSEST_CASES=1000 cargo nextest run -p amg-nurbs --test closest_props
//! --run-ignored all` (1000 cases per kind, 160 000 queries).
#![allow(
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::print_stdout,
    reason = "test code"
)]

mod common;

use std::sync::atomic::{AtomicU64, Ordering};

use amg_core::{Point3, Tolerance};
use amg_nurbs::{NurbsCurve3, NurbsSurface3};
use common::{Rng, curve, diag, surface};
use proptest::prelude::*;

static WORST_CURVE: AtomicU64 = AtomicU64::new(0);
static WORST_SURF: AtomicU64 = AtomicU64::new(0);
static QUERIES: AtomicU64 = AtomicU64::new(0);

/// Records `excess` (may be negative: we beat the samples) as the max of `excess.max(0)`.
fn record(slot: &AtomicU64, excess: f64) {
    slot.fetch_max(excess.max(0.0).to_bits(), Ordering::Relaxed);
}

fn read(slot: &AtomicU64) -> f64 {
    f64::from_bits(slot.load(Ordering::Relaxed))
}

const QUERIES_PER_CASE: usize = 80;
const CURVE_SAMPLES: usize = 20_000;
const SURF_SAMPLES: usize = 150;

fn query(r: &mut Rng, k: usize, center: Point3, half: f64, on: Point3) -> Point3 {
    match k % 3 {
        0 => center + r.point(1.5 * half),
        1 => on + r.point(1.0 / 3f64.sqrt()) * r.f(),
        _ => on,
    }
}

fn check_curve(c: &NurbsCurve3, seed: u64) {
    let tol = Tolerance::DEFAULT;
    let (a, b) = c.domain();
    let samples: Vec<Point3> = (0..CURVE_SAMPLES)
        .map(|i| c.eval(a + (b - a) * i as f64 / (CURVE_SAMPLES - 1) as f64))
        .collect();
    let bb = amg_core::Aabb3::from_points(c.poles().iter().copied());
    let half = 0.5 * diag(c.poles()).max(1.0);
    let mut r = Rng(seed);
    for k in 0..QUERIES_PER_CASE {
        let t0 = r.range(a, b);
        let on = c.eval(t0);
        let q = query(&mut r, k, bb.center(), half, on);
        let dense = samples
            .iter()
            .fold((on - q).length(), |m, s| m.min((*s - q).length()));
        let res = c.closest_point(q, &tol);
        assert!(res.t >= a && res.t <= b);
        assert!((res.point - c.eval(res.t)).length() <= 1e-12 * half);
        assert!(((res.point - q).length() - res.distance).abs() <= 1e-12 * half);
        let excess = res.distance - dense;
        record(&WORST_CURVE, excess);
        assert!(
            excess <= 1e-9,
            "curve seed {seed} query {k} {q}: {} vs dense {dense} (excess {excess:e})",
            res.distance
        );
    }
    QUERIES.fetch_add(QUERIES_PER_CASE as u64, Ordering::Relaxed);
}

fn check_surface(s: &NurbsSurface3, seed: u64) {
    let tol = Tolerance::DEFAULT;
    let [(ua, ub), (va, vb)] = s.domain();
    let n = SURF_SAMPLES;
    let mut samples = Vec::with_capacity(n * n);
    for i in 0..n {
        let u = ua + (ub - ua) * i as f64 / (n - 1) as f64;
        for j in 0..n {
            let v = va + (vb - va) * j as f64 / (n - 1) as f64;
            samples.push(s.eval(u, v));
        }
    }
    let poles = s.poles().as_slice();
    let bb = amg_core::Aabb3::from_points(poles.iter().copied());
    let half = 0.5 * diag(poles).max(1.0);
    let mut r = Rng(seed);
    for k in 0..QUERIES_PER_CASE {
        let (u0, v0) = (r.range(ua, ub), r.range(va, vb));
        let on = s.eval(u0, v0);
        let q = query(&mut r, k, bb.center(), half, on);
        let dense = samples
            .iter()
            .fold((on - q).length(), |m, p| m.min((*p - q).length()));
        let seed_uv = (k % 5 == 0).then(|| (r.range(ua, ub), r.range(va, vb)));
        let res = s.closest_point(q, seed_uv, &tol);
        assert!(res.u >= ua && res.u <= ub && res.v >= va && res.v <= vb);
        assert!(((res.point - q).length() - res.distance).abs() <= 1e-12 * half);
        let excess = res.distance - dense;
        record(&WORST_SURF, excess);
        assert!(
            excess <= 1e-9,
            "surface seed {seed} query {k} {q}: {} vs dense {dense} (excess {excess:e})",
            res.distance
        );
    }
    QUERIES.fetch_add(QUERIES_PER_CASE as u64, Ordering::Relaxed);
}

fn cases() -> u32 {
    std::env::var("KH13_CLOSEST_CASES")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(64)
}

proptest! {
    #![proptest_config(ProptestConfig { cases: 64, ..ProptestConfig::default() })]

    #[test]
    fn curve_closest_point_beats_dense_sampling(seed in any::<u64>()) {
        let mut r = Rng(seed);
        let p = r.usize(1, 7);
        let rational = seed % 2 == 0;
        let c = curve(&mut r, p, rational);
        check_curve(&c, seed ^ 0xC0FFEE);
    }

    #[test]
    fn surface_closest_point_beats_dense_sampling(seed in any::<u64>()) {
        let mut r = Rng(seed);
        let (p, q) = (r.usize(1, 4), r.usize(1, 4));
        let rational = seed % 2 == 0;
        let s = surface(&mut r, p, q, rational);
        check_surface(&s, seed ^ 0xBEEF);
    }
}

/// Deterministic sweep with the measured worst excess printed (`--nocapture`).
#[test]
fn closest_point_measured_worst() {
    for i in 0..16u64 {
        let mut r = Rng(0x13_4000 + i);
        let p = r.usize(1, 7);
        let c = curve(&mut r, p, i % 2 == 0);
        check_curve(&c, i);
        let (p, q) = (r.usize(1, 4), r.usize(1, 4));
        let s = surface(&mut r, p, q, i % 2 == 1);
        check_surface(&s, i);
    }
    println!(
        "closest point: worst excess over dense sampling: curves {:.2e} mm, surfaces {:.2e} mm \
         ({} queries in this binary so far)",
        read(&WORST_CURVE),
        read(&WORST_SURF),
        QUERIES.load(Ordering::Relaxed)
    );
}

/// Larger run, cases from `KH13_CLOSEST_CASES` (default 64 per kind).
#[test]
#[ignore = "long: run with --run-ignored all, KH13_CLOSEST_CASES sets the case count"]
fn closest_point_large_run() {
    let n = u64::from(cases());
    for i in 0..n {
        let mut r = Rng(0x13_9000 + i);
        let p = r.usize(1, 7);
        let c = curve(&mut r, p, i % 2 == 0);
        check_curve(&c, i);
        let (p, q) = (r.usize(1, 4), r.usize(1, 4));
        let s = surface(&mut r, p, q, i % 2 == 1);
        check_surface(&s, i);
    }
    println!(
        "closest point large run ({n} cases per kind): curves {:.2e} mm, surfaces {:.2e} mm",
        read(&WORST_CURVE),
        read(&WORST_SURF)
    );
}
