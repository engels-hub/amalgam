//! SK-13 / ADR-007 solver comparison (note §5.1 item 4): cold solve, warm solve (one driving
//! dimension ±1 %) and one drag frame, per option, on the decision cases D1–D10.
//!
//! `cargo bench -p amg-sketch --bench solver_compare [--features slvs,ezpz-bench]`.
//! Names: `sk13/<cold|warm|drag>/<Dn>/<option>`. The median/p95 tables in
//! `docs/notes/sketch-solver-benchmark.md` come from `examples/sk13_report.rs`, which shares
//! the same adapters.

#![allow(clippy::unwrap_used, reason = "bench harness")]

use std::path::PathBuf;

use amg_sketch::ConstraintKind;
use amg_sketch::corpus::compare::{self, Loaded};
use amg_sketch::corpus::{self, Case};
use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};

fn decision_cases() -> Vec<(String, Case)> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../testdata/sketch/corpus");
    let mut v: Vec<(u32, Case)> = corpus::load_corpus(&root)
        .unwrap()
        .cases
        .into_iter()
        .filter_map(|c| {
            let n = c
                .tags
                .iter()
                .find_map(|t| t.strip_prefix("decision:D"))
                .and_then(|n| n.parse().ok())?;
            Some((n, c))
        })
        .collect();
    v.sort_by_key(|(n, _)| *n);
    v.into_iter().map(|(n, c)| (format!("D{n}"), c)).collect()
}

fn bench(c: &mut Criterion) {
    let cases = decision_cases();
    let mut g = c.benchmark_group("sk13");
    g.sample_size(30);
    for (name, case) in &cases {
        let built = case.build().unwrap();
        let dim = case
            .constraints
            .iter()
            .enumerate()
            .find_map(|(i, k)| match k {
                ConstraintKind::Dimension(d) if d.driving => Some((i as u32, d.value)),
                _ => None,
            });
        for opt in compare::contenders() {
            let Ok(mut warm) = opt.load(&built) else {
                continue;
            };
            let label = opt.name();
            g.bench_function(BenchmarkId::new(format!("cold/{name}"), &label), |b| {
                b.iter_batched(
                    || built.clone(),
                    |bc| {
                        let mut l = opt.load(&bc).unwrap();
                        l.solve()
                    },
                    criterion::BatchSize::SmallInput,
                );
            });
            if let Some((ci, v0)) = dim {
                let _ = warm.solve();
                let mut flip = false;
                g.bench_function(BenchmarkId::new(format!("warm/{name}"), &label), |b| {
                    b.iter(|| {
                        flip = !flip;
                        let v = if flip { v0 * 1.01 } else { v0 };
                        warm.set_dimension(ci, v).unwrap();
                        warm.solve()
                    });
                });
            }
            if let Some(script) = &case.drag {
                let mut l: Box<dyn Loaded> = opt.load(&built).unwrap();
                let _ = l.solve();
                let mut k = 0usize;
                // Walk the path forward and back so every frame is a small step.
                let n = script.path.len();
                g.bench_function(BenchmarkId::new(format!("drag/{name}"), &label), |b| {
                    b.iter(|| {
                        let i = k % (2 * n);
                        let at = if i < n { i } else { 2 * n - 1 - i };
                        k += 1;
                        l.drag(script.entity, script.path[at])
                    });
                });
            }
        }
    }
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
