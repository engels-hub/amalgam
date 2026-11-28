//! KH-12 Done-when 5 (S20 / nurbs note §9 budgets): refine 20 knots into a 50-pole cubic
//! < 10 µs; degree elevation 3 → 5 of a 50-pole cubic < 20 µs. Also reported (no budget):
//! insertion of one knot, Bézier decomposition, split + join, removal round trip and the
//! blossom elevation path on uneven knots. Run with `cargo bench -p amg-nurbs --bench
//! knot_ops`. Medians are printed always and asserted only under `AMG_STRICT_TIMING=1`.

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
use amg_nurbs::{KnotVector, NurbsCurve3};
use criterion::{Criterion, criterion_group, criterion_main};

fn cubic50(knots: KnotVector) -> NurbsCurve3 {
    let n = 50;
    let poles = (0..n)
        .map(|i| {
            let t = i as f64;
            Point3::new(t, (0.7 * t).sin() * 5.0, (0.3 * t).cos() * 3.0)
        })
        .collect();
    NurbsCurve3::new(3, knots, poles, None).unwrap()
}

/// 50-pole cubic with uneven (geometric) spans: takes the blossom elevation path.
fn uneven_knots() -> KnotVector {
    let spans = 50 - 3;
    let mut v = vec![0.0; 4];
    let mut x = 0.0;
    for i in 0..spans - 1 {
        x += if i % 2 == 0 { 1.0 } else { 6.0 };
        v.push(x);
    }
    x += 1.0;
    v.extend(std::iter::repeat_n(x, 4));
    KnotVector::new(v).unwrap()
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
    let c = cubic50(KnotVector::clamped_uniform(3, 50).unwrap());
    let cu = cubic50(uneven_knots());
    // 20 new knots spread over the domain (between the existing ones).
    let xs: Vec<f64> = (0..20).map(|i| (i as f64 + 0.37) / 20.0).collect();
    let tol = Tolerance::DEFAULT;

    let refine_us = median_us(200, || {
        black_box(c.refine_knots(black_box(&xs)).unwrap());
    });
    let elevate_us = median_us(200, || {
        black_box(c.elevate_degree(black_box(2)).unwrap());
    });
    let elevate_uneven_us = median_us(100, || {
        black_box(cu.elevate_degree(black_box(2)).unwrap());
    });
    let insert_us = median_us(500, || {
        black_box(c.insert_knot(black_box(0.4321), 1).unwrap());
    });
    let decompose_us = median_us(200, || {
        black_box(c.bezier_segments());
    });
    let split_join_us = median_us(200, || {
        let (l, r) = c.split(black_box(0.4321)).unwrap();
        black_box(l.join(&r, &tol).unwrap());
    });
    let ins = c.insert_knot(0.4321, 2).unwrap();
    let remove_us = median_us(200, || {
        black_box(ins.remove_knot(black_box(0.4321), 2, 1e-9));
    });
    println!(
        "amg-nurbs knot op medians (50-pole cubic): refine 20 knots {refine_us:.2} µs (< 10), \
         elevate 3→5 {elevate_us:.2} µs (< 20), elevate 3→5 uneven knots (blossom path) \
         {elevate_uneven_us:.2} µs, insert 1 knot {insert_us:.2} µs, Bézier decomposition \
         {decompose_us:.2} µs, split + join {split_join_us:.2} µs, remove 2 copies \
         {remove_us:.2} µs"
    );
    if std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1") {
        assert!(refine_us < 10.0, "refine 20 knots {refine_us:.2} µs");
        assert!(elevate_us < 20.0, "elevate 3→5 {elevate_us:.2} µs");
    }

    let mut g = cr.benchmark_group("nurbs_knot_ops");
    g.bench_function("refine_20_into_50", |b| {
        b.iter(|| black_box(c.refine_knots(black_box(&xs)).unwrap()));
    });
    g.bench_function("elevate_3_to_5_of_50", |b| {
        b.iter(|| black_box(c.elevate_degree(black_box(2)).unwrap()));
    });
    g.bench_function("elevate_3_to_5_of_50_uneven", |b| {
        b.iter(|| black_box(cu.elevate_degree(black_box(2)).unwrap()));
    });
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
