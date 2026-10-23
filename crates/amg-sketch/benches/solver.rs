//! SK-03 production solver: cold and warm solves of the decision cases D1–D10 and of a long
//! dimensioned chain (one sparse block), with the default `SolveOptions` (rank test and
//! diagnosis on) and with `SolveOptions::fast()`.
//!
//! `cargo bench -p amg-sketch --bench solver`. Names: `sk03/<cold|warm|warm-fast>/<case>`.
//! Done-when 2 is `sk03/warm/D9` (median < 5 ms): one driving dimension alternates between its
//! value and +1 %, the solver keeps its structure cache.

#![allow(clippy::unwrap_used, reason = "bench harness")]

use std::path::PathBuf;

use amg_core::{Plane, Point2};
use amg_sketch::corpus::{self, Case};
use amg_sketch::solve::{NativeSolver, SolveOptions, SolverBackend};
use amg_sketch::{ConstraintId, ConstraintKind, Dimension, DimensionKind, Sketch};
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

/// Open chain of `n` dimensioned segments from a fixed origin (one block of `2n` unknowns).
fn chain(n: usize) -> (Sketch, ConstraintId) {
    let mut s = Sketch::new(Plane::XY);
    let mut prev = s.add_point(Point2::new(0.0, 0.0)).unwrap();
    s.add_constraint(ConstraintKind::Fix { entity: prev })
        .unwrap();
    let mut first = None;
    for i in 0..n {
        let t = i as f64;
        let q = s
            .add_point(Point2::new(10.0 * (t + 1.0), 3.0 * (0.37 * t).sin()))
            .unwrap();
        let l = s.add_line_between(prev, q).unwrap();
        let id = s
            .add_constraint(ConstraintKind::Dimension(Dimension {
                kind: DimensionKind::Length { line: l },
                value: 10.0,
                driving: true,
                label_pos: Point2::ZERO,
            }))
            .unwrap();
        first.get_or_insert(id);
        prev = q;
    }
    NativeSolver::new().solve(&mut s, &SolveOptions::default(), &|| false);
    (s, first.unwrap())
}

fn bench_one(
    g: &mut criterion::BenchmarkGroup<'_, criterion::measurement::WallTime>,
    name: &str,
    sketch: &Sketch,
    dim: Option<(ConstraintId, f64)>,
) {
    let never = || false;
    g.bench_function(BenchmarkId::new("cold", name), |b| {
        b.iter_batched(
            || sketch.clone(),
            |mut s| NativeSolver::new().solve(&mut s, &SolveOptions::default(), &never),
            criterion::BatchSize::SmallInput,
        );
    });
    let Some((id, v0)) = dim else {
        return;
    };
    for (label, opts) in [
        ("warm", SolveOptions::default()),
        ("warm-fast", SolveOptions::fast()),
    ] {
        let mut s = sketch.clone();
        let mut solver = NativeSolver::new();
        solver.solve(&mut s, &opts, &never);
        let mut flip = false;
        g.bench_function(BenchmarkId::new(label, name), |b| {
            b.iter(|| {
                flip = !flip;
                let v = if flip { v0 * 1.01 } else { v0 };
                s.set_dimension_value(id, v).unwrap();
                solver.solve(&mut s, &opts, &never)
            });
        });
    }
}

fn bench(c: &mut Criterion) {
    let mut g = c.benchmark_group("sk03");
    g.sample_size(50);
    for (name, case) in decision_cases() {
        let built = case.build().unwrap();
        let dim = case
            .constraints
            .iter()
            .enumerate()
            .find_map(|(i, k)| match k {
                ConstraintKind::Dimension(d) if d.driving => Some((built.constraints[i], d.value)),
                _ => None,
            });
        bench_one(&mut g, &name, &built.sketch, dim);
    }
    for n in [300, 1000] {
        let (s, id) = chain(n);
        bench_one(&mut g, &format!("chain{n}"), &s, Some((id, 10.0)));
    }
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
