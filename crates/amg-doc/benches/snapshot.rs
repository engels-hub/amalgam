//! DOC-01: persistent-collection backends for authored state (S01 §1).
//!
//! Compares `imbl::OrdMap` (the backend), `rpds::RedBlackTreeMapSync` (fallback) and per-entity
//! `Arc` copy-on-write (`Arc<Vec<Arc<T>>>`, baseline) at 10³/10⁴/10⁵ entries:
//! snapshot clone, one mutation after a clone, 100 mutations after a clone, ordered iteration,
//! and diff after one mutation. Plus `Design` clone / one `set_body_props` after a clone.
//!
//! Targets at 10⁴ entries: clone < 1 µs, one mutation after a clone < 5 µs.

#![allow(clippy::unwrap_used, missing_docs, reason = "benchmark harness")]

use std::hint::black_box;
use std::sync::Arc;

use amg_doc::{BodyId, BodyProps, Design, FeatureId};
use criterion::{BatchSize, BenchmarkId, Criterion, criterion_group, criterion_main};

type V = Arc<BodyProps>;

fn key(i: usize) -> BodyId {
    BodyId {
        origin: FeatureId(i as u64 / 4 + 1),
        slot: (i % 4) as u32,
    }
}

fn props(i: usize) -> V {
    Arc::new(BodyProps {
        name: Some(format!("Body{i}")),
        ..BodyProps::default()
    })
}

const SIZES: [usize; 3] = [1_000, 10_000, 100_000];

fn imbl_map(n: usize) -> imbl::OrdMap<BodyId, V> {
    (0..n).map(|i| (key(i), props(i))).collect()
}

fn rpds_map(n: usize) -> rpds::RedBlackTreeMapSync<BodyId, V> {
    let mut m = rpds::RedBlackTreeMapSync::new_sync();
    for i in 0..n {
        m.insert_mut(key(i), props(i));
    }
    m
}

/// Sorted by key, so index = position; mutation is `Arc::make_mut` on the whole vector.
fn arc_vec(n: usize) -> Arc<Vec<(BodyId, V)>> {
    Arc::new((0..n).map(|i| (key(i), props(i))).collect())
}

fn mutation_targets(n: usize, count: usize) -> Vec<usize> {
    (0..count)
        .map(|j| (j * n / count + n / (2 * count)) % n)
        .collect()
}

fn bench_clone(c: &mut Criterion) {
    let mut g = c.benchmark_group("clone");
    for n in SIZES {
        let a = imbl_map(n);
        let b = rpds_map(n);
        let v = arc_vec(n);
        g.bench_with_input(BenchmarkId::new("imbl", n), &n, |bch, _| {
            bch.iter(|| black_box(&a).clone());
        });
        g.bench_with_input(BenchmarkId::new("rpds", n), &n, |bch, _| {
            bch.iter(|| black_box(&b).clone());
        });
        g.bench_with_input(BenchmarkId::new("arc_vec", n), &n, |bch, _| {
            bch.iter(|| Arc::clone(black_box(&v)));
        });
    }
    g.finish();
}

fn bench_mutate(c: &mut Criterion, count: usize) {
    let mut g = c.benchmark_group(format!("mutate_{count}_after_clone"));
    for n in SIZES {
        let targets = mutation_targets(n, count);
        let new_val = props(usize::MAX);
        let a = imbl_map(n);
        let b = rpds_map(n);
        let v = arc_vec(n);
        g.bench_with_input(BenchmarkId::new("imbl", n), &n, |bch, _| {
            bch.iter_batched(
                || a.clone(),
                |mut m| {
                    for &i in &targets {
                        m.insert(key(i), new_val.clone());
                    }
                    m
                },
                BatchSize::SmallInput,
            );
        });
        g.bench_with_input(BenchmarkId::new("rpds", n), &n, |bch, _| {
            bch.iter_batched(
                || b.clone(),
                |mut m| {
                    for &i in &targets {
                        m.insert_mut(key(i), new_val.clone());
                    }
                    m
                },
                BatchSize::SmallInput,
            );
        });
        g.bench_with_input(BenchmarkId::new("arc_vec", n), &n, |bch, _| {
            bch.iter_batched(
                || Arc::clone(&v),
                |mut m| {
                    let vec = Arc::make_mut(&mut m);
                    for &i in &targets {
                        vec[i].1 = new_val.clone();
                    }
                    m
                },
                BatchSize::SmallInput,
            );
        });
    }
    g.finish();
}

fn bench_mutate_1(c: &mut Criterion) {
    bench_mutate(c, 1);
}

fn bench_mutate_100(c: &mut Criterion) {
    bench_mutate(c, 100);
}

fn bench_iterate(c: &mut Criterion) {
    let mut g = c.benchmark_group("iterate_ordered");
    for n in SIZES {
        let a = imbl_map(n);
        let b = rpds_map(n);
        let v = arc_vec(n);
        g.bench_with_input(BenchmarkId::new("imbl", n), &n, |bch, _| {
            bch.iter(|| a.iter().filter(|(_, p)| p.visible).count());
        });
        g.bench_with_input(BenchmarkId::new("rpds", n), &n, |bch, _| {
            bch.iter(|| b.iter().filter(|(_, p)| p.visible).count());
        });
        g.bench_with_input(BenchmarkId::new("arc_vec", n), &n, |bch, _| {
            bch.iter(|| v.iter().filter(|(_, p)| p.visible).count());
        });
    }
    g.finish();
}

fn bench_diff(c: &mut Criterion) {
    let mut g = c.benchmark_group("diff_after_1_mutation");
    for n in SIZES {
        let i = n / 2;
        let new_val = props(usize::MAX);
        let a = imbl_map(n);
        let mut a2 = a.clone();
        a2.insert(key(i), new_val.clone());
        let b = rpds_map(n);
        let mut b2 = b.clone();
        b2.insert_mut(key(i), new_val.clone());
        let v = arc_vec(n);
        let mut v2 = Arc::clone(&v);
        Arc::make_mut(&mut v2)[i].1 = new_val.clone();
        g.bench_with_input(BenchmarkId::new("imbl", n), &n, |bch, _| {
            bch.iter(|| {
                let d = a.diff(&a2).count();
                assert_eq!(d, 1);
                d
            });
        });
        // rpds has no diff: walk both in order and compare by pointer (same key set here).
        g.bench_with_input(BenchmarkId::new("rpds", n), &n, |bch, _| {
            bch.iter(|| {
                let d = b
                    .iter()
                    .zip(b2.iter())
                    .filter(|((_, x), (_, y))| !Arc::ptr_eq(x, y))
                    .count();
                assert_eq!(d, 1);
                d
            });
        });
        g.bench_with_input(BenchmarkId::new("arc_vec", n), &n, |bch, _| {
            bch.iter(|| {
                let d = v
                    .iter()
                    .zip(v2.iter())
                    .filter(|((_, x), (_, y))| !Arc::ptr_eq(x, y))
                    .count();
                assert_eq!(d, 1);
                d
            });
        });
    }
    g.finish();
}

fn bench_design(c: &mut Criterion) {
    let mut g = c.benchmark_group("design");
    for n in SIZES {
        let mut d = Design::with_uuid("Bench", amg_core::PersistentId::NIL);
        for i in 0..n {
            d.set_body_props(key(i), (*props(i)).clone());
        }
        let snap = Arc::new(d);
        g.bench_with_input(BenchmarkId::new("snapshot_clone", n), &n, |bch, _| {
            bch.iter(|| Design::clone(black_box(&snap)));
        });
        let p = (*props(usize::MAX)).clone();
        g.bench_with_input(
            BenchmarkId::new("set_body_props_after_clone", n),
            &n,
            |bch, _| {
                bch.iter_batched(
                    || (*snap).clone(),
                    |mut d| {
                        d.set_body_props(key(n / 2), p.clone());
                        d
                    },
                    BatchSize::SmallInput,
                );
            },
        );
    }
    g.finish();
}

criterion_group!(
    benches,
    bench_clone,
    bench_mutate_1,
    bench_mutate_100,
    bench_iterate,
    bench_diff,
    bench_design
);
criterion_main!(benches);
