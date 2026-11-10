//! ASM-01 budgets (S06): `world_transform` of a depth-10 path < 1 µs; building `Placements`
//! for 10 000 occurrences < 2 ms. Timing assertions only under `AMG_STRICT_TIMING=1`.

#![allow(clippy::unwrap_used, missing_docs, reason = "benchmark harness")]

use std::hint::black_box;
use std::time::Instant;

use amg_core::{Quat, Transform, Vec3};
use amg_doc::{Design, OccurrenceId, Placements};
use criterion::{Criterion, criterion_group, criterion_main};

fn strict() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}

/// A chain of `depth` nested components and a design with `n` occurrences spread over a
/// 100-way fan-out at the root, the chain below the first of them.
fn design(n: usize, depth: usize) -> (Design, Vec<OccurrenceId>) {
    let mut d = Design::new("asm01-bench");
    let root = d.root();
    let t = |i: usize| {
        Transform::from_rotation_translation(
            Quat::from_rotation_z(i as f64 * 0.01),
            Vec3::new(i as f64, 1.0, 2.0),
        )
    };
    let mut parent = root;
    let mut chain = Vec::new();
    for i in 0..depth {
        let (c, o) = d.add_component(parent, &format!("L{i}"), t(i)).unwrap();
        chain.push(o);
        parent = c;
    }
    let (leaf, _) = d.add_component(root, "leaf", Transform::IDENTITY).unwrap();
    for i in depth..n {
        d.add_occurrence(root, leaf, t(i)).unwrap();
    }
    (d, chain)
}

fn bench(c: &mut Criterion) {
    let (d, chain) = design(10_000, 10);
    let p = Placements::from_design(&d);
    assert_eq!(chain.len(), 10);
    assert!(p.world(&chain).is_some());

    c.bench_function("asm01/world_transform_depth10", |b| {
        b.iter(|| black_box(p.world(black_box(&chain))));
    });
    c.bench_function("asm01/placements_10k", |b| {
        b.iter(|| black_box(Placements::from_design(black_box(&d))));
    });

    if strict() {
        let t0 = Instant::now();
        let reps = 100_000u32;
        for _ in 0..reps {
            black_box(p.world(black_box(&chain)));
        }
        let per = t0.elapsed() / reps;
        assert!(
            per.as_nanos() < 1_000,
            "world_transform depth 10: {per:?} (budget 1 µs)"
        );

        let mut times: Vec<_> = (0..50)
            .map(|_| {
                let t0 = Instant::now();
                black_box(Placements::from_design(black_box(&d)));
                t0.elapsed()
            })
            .collect();
        times.sort();
        let median = times[times.len() / 2];
        assert!(
            median.as_micros() < 2_000,
            "Placements for 10k: {median:?} (budget 2 ms)"
        );
    }
}

criterion_group!(benches, bench);
criterion_main!(benches);
