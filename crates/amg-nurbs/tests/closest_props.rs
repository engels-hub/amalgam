//! KH-13 Done-when 4: curve and surface `closest_point` never exceed the dense-sampling
//! minimum plus 1e-9 mm. 64 random curves × 80 queries and 64 random surfaces × 80 queries
//! (10 240 queries; degree 1–7 curves, 1–4 surfaces, half rational). A third of the queries
//! are random points around the bounding box, a third lie up to 1 mm off the curve or
//! surface and a third on it. The sample set always includes the generating parameter, so
//! for the last two kinds the bound is the offset itself.
//!
//! The proptests run from a fixed seed, so a run is reproducible; `AMG_CLOSEST_SEED=<u64>`
//! draws another sample. Failure persistence is off: a failing case becomes an explicit test
//! (KH-13 fix).
//!
//! Larger runs (ignored):
//! - many seeds, default 50 × 10 240 queries (`AMG_CLOSEST_SEEDS` sets the count):
//!   `cargo nextest run -p amg-nurbs --test closest_props --run-ignored ignored-only
//!   --no-capture closest_point_seed_sweep`
//! - many cases: `KH13_CLOSEST_CASES=1000 cargo nextest run -p amg-nurbs --test closest_props
//!   --run-ignored all` (1000 cases per kind, 160 000 queries).
#![allow(
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::print_stdout,
    reason = "test code"
)]

mod common;

use std::sync::atomic::{AtomicU64, Ordering};

use amg_core::{Point3, Tolerance};
use amg_nurbs::{Grid, KnotVector, NurbsCurve3, NurbsSurface3};
use common::{Rng, curve, diag, surface};
use proptest::prelude::*;
use proptest::test_runner::{RngSeed, TestRunner};

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
/// Seed of the proptests (fixed: the merge gate must not depend on the draw).
const SEED: u64 = 0x4b48_3133; // "KH13"

fn env_u64(name: &str, default: u64) -> u64 {
    std::env::var(name)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

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

/// The curve of proptest case `seed`, checked with its 80 queries.
fn curve_case(seed: u64) {
    let mut r = Rng(seed);
    let p = r.usize(1, 7);
    let rational = seed.is_multiple_of(2);
    let c = curve(&mut r, p, rational);
    check_curve(&c, seed ^ 0xC0FFEE);
}

/// The surface of proptest case `seed`, checked with its 80 queries.
fn surface_case(seed: u64) {
    let mut r = Rng(seed);
    let (p, q) = (r.usize(1, 4), r.usize(1, 4));
    let rational = seed.is_multiple_of(2);
    let s = surface(&mut r, p, q, rational);
    check_surface(&s, seed ^ 0xBEEF);
}

/// Control-net layouts whose corner chords vanish in one or both directions.
#[derive(Debug, Clone, Copy)]
enum Layout {
    /// First and last pole row coincide (closed in u).
    ClosedU,
    /// First and last pole column coincide (closed in v).
    ClosedV,
    /// The edges at both ends of u each collapse to a point (a spindle).
    Spindle,
    /// All four boundary edges collapse to one point (a balloon).
    Balloon,
    /// All four corners coincide, the edges between them do not.
    PinchedCorners,
}

const LAYOUTS: [Layout; 5] = [
    Layout::ClosedU,
    Layout::ClosedV,
    Layout::Spindle,
    Layout::Balloon,
    Layout::PinchedCorners,
];

/// A single-patch Bézier surface of degree 2–4 × 2–4 with the given degeneracy.
fn degenerate_surface(r: &mut Rng, layout: Layout) -> NurbsSurface3 {
    let (p, q) = (r.usize(2, 4), r.usize(2, 4));
    let (nu, nv) = (p + 1, q + 1);
    let mut g: Vec<Point3> = (0..nu * nv).map(|_| r.point(100.0)).collect();
    let (a, b) = (r.point(100.0), r.point(100.0));
    for i in 0..nu {
        for j in 0..nv {
            let (first_u, last_u) = (i == 0, i == nu - 1);
            let (first_v, last_v) = (j == 0, j == nv - 1);
            let moved = match layout {
                Layout::ClosedU => last_u.then(|| g[j]),
                Layout::ClosedV => last_v.then(|| g[i * nv]),
                Layout::Spindle if first_u => Some(a),
                Layout::Spindle => last_u.then_some(b),
                Layout::Balloon => (first_u || last_u || first_v || last_v).then_some(a),
                Layout::PinchedCorners => ((first_u || last_u) && (first_v || last_v)).then_some(a),
            };
            if let Some(x) = moved {
                g[i * nv + j] = x;
            }
        }
    }
    let grid = Grid::new(nu, nv, g).unwrap();
    let knots = [KnotVector::bezier(p), KnotVector::bezier(q)];
    NurbsSurface3::new([p, q], knots, grid, None).unwrap()
}

fn cases() -> u32 {
    std::env::var("KH13_CLOSEST_CASES")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(64)
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 64,
        failure_persistence: None,
        rng_seed: RngSeed::Fixed(env_u64("AMG_CLOSEST_SEED", SEED)),
        ..ProptestConfig::default()
    })]

    #[test]
    fn curve_closest_point_beats_dense_sampling(seed in any::<u64>()) {
        curve_case(seed);
    }

    #[test]
    fn surface_closest_point_beats_dense_sampling(seed in any::<u64>()) {
        surface_case(seed);
    }
}

/// KH-13 fix regression (seen in KH-16's runs of the proptest): a query on a rational (3, 2)
/// surface returned 1.6e-9 mm. Newton from a far midpoint stopped short there, and the search
/// reached its depth limit before a closer midpoint could restart it.
#[test]
fn surface_closest_point_regression_newton_stall_and_depth_limit() {
    surface_case(7_560_825_692_560_342_496);
}

/// KH-13 fix regression: on a patch whose corners coincide in one direction (closed, or
/// collapsed edges) that direction was never split, and the result could be tens of mm off.
#[test]
fn closest_point_on_closed_and_collapsed_patches() {
    for (l, layout) in LAYOUTS.into_iter().enumerate() {
        for i in 0..2u64 {
            let seed = 0x13_C000 + 16 * l as u64 + i;
            let s = degenerate_surface(&mut Rng(seed), layout);
            check_surface(&s, seed);
        }
    }
}

/// The curve and surface proptests over seeds `0..AMG_CLOSEST_SEEDS` (default 50, 10 240
/// queries each), plus one degenerate patch of each layout per seed (400 queries): the
/// KH-13 fix evidence. A seed stops at its first violating case; all seeds are reported.
#[test]
#[ignore = "long: 50 seeds of 10 640 queries; see the module docs"]
fn closest_point_seed_sweep() {
    let seeds = env_u64("AMG_CLOSEST_SEEDS", 50);
    let mut failures = Vec::new();
    for seed in 0..seeds {
        let config = ProptestConfig {
            cases: 64,
            failure_persistence: None,
            rng_seed: RngSeed::Fixed(seed),
            ..ProptestConfig::default()
        };
        for (kind, case) in [("curve", curve_case as fn(u64)), ("surface", surface_case)] {
            let run = TestRunner::new(config.clone()).run(&any::<u64>(), |s| {
                case(s);
                Ok(())
            });
            if let Err(e) = run {
                failures.push(format!("seed {seed} {kind}: {e}"));
            }
        }
        for (l, layout) in LAYOUTS.into_iter().enumerate() {
            let case = 0x13_D000_0000 + 16 * seed + l as u64;
            let s = degenerate_surface(&mut Rng(case), layout);
            if std::panic::catch_unwind(|| check_surface(&s, case)).is_err() {
                failures.push(format!("seed {seed} {layout:?}"));
            }
        }
    }
    println!(
        "closest point seed sweep: {seeds} seeds, {} queries, {} failing runs, worst excess: \
         curves {:.2e} mm, surfaces {:.2e} mm",
        QUERIES.load(Ordering::Relaxed),
        failures.len(),
        read(&WORST_CURVE),
        read(&WORST_SURF)
    );
    assert!(failures.is_empty(), "{failures:#?}");
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
