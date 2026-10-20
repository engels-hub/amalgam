//! DOC-04: evaluating a chain of 1 000 parameters (S01 §5). Target: < 2 ms.
//!
//! `edit_head` re-evaluates all 999 dependents after changing the first parameter;
//! `rebuild_from_json` is a full load (parse + topological evaluation of every parameter).

#![allow(clippy::unwrap_used, missing_docs, reason = "benchmark harness")]

use std::hint::black_box;

use amg_doc::Design;
use criterion::{BatchSize, Criterion, criterion_group, criterion_main};

fn chain(n: usize) -> Design {
    let mut d = Design::new("bench");
    d.add_param("p0", "1 mm", None, "").unwrap();
    for i in 1..n {
        d.add_param(&format!("p{i}"), &format!("p{} + 1 mm", i - 1), None, "")
            .unwrap();
    }
    d
}

fn params(c: &mut Criterion) {
    let base = chain(1000);
    let head = base.params().by_name("p0").unwrap();
    let last = base.params().by_name("p999").unwrap();

    c.bench_function("params/edit_head_1000_chain", |b| {
        let mut flip = false;
        b.iter_batched(
            || base.clone(),
            |mut d| {
                flip = !flip;
                d.set_param_expr(head, if flip { "2 mm" } else { "3 mm" })
                    .unwrap();
                black_box(d.params().value(last));
                d
            },
            BatchSize::LargeInput,
        );
    });

    let json = serde_json::to_string(&base).unwrap();
    c.bench_function("params/rebuild_from_json_1000_chain", |b| {
        b.iter(|| {
            let d: Design = serde_json::from_str(black_box(&json)).unwrap();
            d
        });
    });
}

criterion_group!(benches, params);
criterion_main!(benches);
