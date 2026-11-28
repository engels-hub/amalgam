//! SOL-29 Done-when 2: `budget/sol-recompute-50`, the recompute of a 50-feature part after an
//! edit to its first sketch (budget: < 500 ms, vision). The measured time is the edit command
//! plus the full recompute, in a document whose cache holds the previous states; every sample
//! uses a width not seen before, so nothing downstream is a cache hit.
//!
//! Before criterion runs, a fixed 15-edit sample prints the median and the 5 slowest features
//! (median compute time per feature); with `AMG_STRICT_TIMING=1` the median must be under
//! 500 ms. Needs the `occt` feature.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::print_stdout,
    missing_docs,
    reason = "bench"
)]

#[path = "../tests/solid_recompute/part.rs"]
mod part;
#[path = "../tests/solid_common/mod.rs"]
mod solid_common;

use std::time::{Duration, Instant};

use criterion::{Criterion, criterion_group, criterion_main};
use part::{FEATURES, Part, WIDTH, build_part};

const SAMPLES: usize = 15;
const BUDGET: Duration = Duration::from_millis(500);

/// Widths that never repeat, within 100–104 mm.
#[derive(Default)]
struct Widths(u32);

impl Widths {
    fn next(&mut self) -> f64 {
        self.0 += 1;
        WIDTH + f64::from(self.0 % 40_000) * 1e-4
    }
}

fn ms(d: Duration) -> f64 {
    d.as_secs_f64() * 1e3
}

fn median(mut t: Vec<Duration>) -> Duration {
    t.sort();
    t[t.len() / 2]
}

/// Runs the fixed sample and prints the total median and the slowest features.
fn report(p: &mut Part, widths: &mut Widths) -> Duration {
    p.set_width(widths.next());
    let mut totals = Vec::with_capacity(SAMPLES);
    let mut per_feature = vec![Vec::with_capacity(SAMPLES); p.features.len()];
    for _ in 0..SAMPLES {
        let w = widths.next();
        let started = Instant::now();
        p.set_width(w);
        totals.push(started.elapsed());
        for (i, (id, _)) in p.features.iter().enumerate() {
            per_feature[i].push(p.doc.evaluated().node(*id).expect("evaluated").duration);
        }
    }
    assert!(p.unhealthy().is_empty(), "{:?}", p.unhealthy());
    let mut features: Vec<(Duration, usize)> =
        per_feature.into_iter().map(median).zip(0..).collect();
    let computed = features.iter().filter(|f| f.0 > Duration::ZERO).count();
    let sum: Duration = features.iter().map(|f| f.0).sum();
    features.sort_by(|a, b| b.cmp(a));
    let (lo, hi) = (
        totals.iter().min().expect("samples"),
        totals.iter().max().expect("samples"),
    );
    let (lo, hi) = (*lo, *hi);
    let total = median(totals);
    println!(
        "sol-recompute-50: median {:.1} ms (min {:.1}, max {:.1}; {SAMPLES} edits, budget < {:.0} ms); \
         {computed} of {FEATURES} features computed, {:.1} ms in feature computes",
        ms(total),
        ms(lo),
        ms(hi),
        ms(BUDGET),
        ms(sum),
    );
    println!("sol-recompute-50: 5 slowest features (median compute time):");
    for (d, i) in features.iter().take(5) {
        println!(
            "  {:>8.1} ms  {:>4.1} %  #{:<2} {}",
            ms(*d),
            100.0 * d.as_secs_f64() / sum.as_secs_f64(),
            i + 1,
            p.features[*i].1
        );
    }
    total
}

fn bench(c: &mut Criterion) {
    let mut p = build_part();
    assert_eq!(p.features.len(), FEATURES);
    let mut widths = Widths::default();
    let total = report(&mut p, &mut widths);
    if std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1") {
        assert!(
            total < BUDGET,
            "median {total:?} over the {BUDGET:?} budget"
        );
    }
    let mut g = c.benchmark_group("budget");
    g.sample_size(10);
    g.bench_function("sol-recompute-50", |b| {
        b.iter(|| p.set_width(widths.next()));
    });
    g.finish();
    assert!(p.unhealthy().is_empty(), "{:?}", p.unhealthy());
}

criterion_group!(benches, bench);
criterion_main!(benches);
