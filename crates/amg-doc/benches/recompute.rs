//! DOC-03: engine overhead of a recompute pass in which every feature hits the cache
//! (S01 §3). Target: < 5 ms for 200 cached features.
//!
//! `chain_200` has each feature read the previous one's body (deep key chaining);
//! `wide_200` has 200 independent features.

#![allow(clippy::unwrap_used, missing_docs, reason = "benchmark harness")]

use std::hint::black_box;

use amg_doc::testing::TestCounter;
use amg_doc::{ComputeMode, Dependency, Design, FeatureBox, GeomCache, recompute};
use amg_jobs::JobCtx;
use criterion::{Criterion, criterion_group, criterion_main};

fn design(n: usize, chained: bool) -> Design {
    let mut d = Design::new("bench");
    let root = d.root();
    let mut prev = None;
    for i in 0..n {
        let deps = match (chained, prev) {
            (true, Some(p)) => vec![Dependency::Body(amg_doc::BodyId { origin: p, slot: 0 })],
            _ => Vec::new(),
        };
        let f = FeatureBox::new(TestCounter {
            deps,
            value: i64::try_from(i).unwrap(),
            fail: false,
        });
        prev = Some(d.add_feature(None, f, root, None).unwrap());
    }
    d
}

fn bench(c: &mut Criterion) {
    let job = JobCtx::detached();
    for (name, chained) in [("chain_200", true), ("wide_200", false)] {
        let d = design(200, chained);
        let cache = GeomCache::new(1 << 30);
        let warm = recompute(&d, &cache, ComputeMode::Full, &job).unwrap();
        assert_eq!(warm.computed.len(), 200);
        c.bench_function(&format!("recompute/all_cached_{name}"), |b| {
            b.iter(|| {
                let r = recompute(black_box(&d), &cache, ComputeMode::Full, &job).unwrap();
                assert!(r.computed.is_empty());
                black_box(r)
            });
        });
    }
}

criterion_group!(benches, bench);
criterion_main!(benches);
