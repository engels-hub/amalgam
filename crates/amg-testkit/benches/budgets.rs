//! Placeholder criterion benches, one `budget/<id>` per entry of
//! [`amg_testkit::budget::BUDGETS`] (S00 §5, FND-08).
//!
//! The placeholders measure nothing real; they reserve the bench names and keep the
//! bench job wired. When an owner adds the real `budget/<id>` bench in its own crate, its
//! owner adds the id to `REAL` below (a cross-lane edit to this lane-E file) so the placeholder
//! stops running; two benches with one name would overwrite each other's criterion report.
//!
//! Run: `cargo bench -p amg-testkit --bench budgets` (`-- --test` runs each once).

#![allow(missing_docs, reason = "criterion macros")]

use std::hint::black_box;
use std::time::Duration;

use amg_testkit::budget::{BENCH_GROUP, BUDGETS};
use criterion::{Criterion, criterion_group, criterion_main};

/// Budget ids whose real bench lives in the owning crate.
const REAL: &[&str] = &[];

fn placeholders(c: &mut Criterion) {
    for id in REAL {
        assert!(
            amg_testkit::budget::get(id).is_some(),
            "REAL lists unknown budget id {id:?}"
        );
    }
    let mut g = c.benchmark_group(BENCH_GROUP);
    g.warm_up_time(Duration::from_millis(200))
        .measurement_time(Duration::from_millis(500));
    for budget in BUDGETS.iter().filter(|b| !REAL.contains(&b.id)) {
        g.bench_function(budget.id, |b| {
            b.iter(|| black_box(budget.limit).max_duration());
        });
    }
    g.finish();
}

criterion_group!(benches, placeholders);
criterion_main!(benches);
