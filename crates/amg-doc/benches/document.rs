//! DOC-12: `Document::poll` cost (S01 budget: < 1 ms, never blocks).
//!
//! `idle` polls an up-to-date document. `supersede` edits a parameter and polls while a slow
//! recompute is running, so every iteration cancels one job and spawns the next.

#![allow(clippy::unwrap_used, missing_docs, reason = "benchmark harness")]

use amg_doc::testing::TestSlow;
use amg_doc::{Command, Design, Document, Expr, FeatureBox};
use amg_jobs::{JobSystem, JobSystemConfig};
use criterion::{Criterion, criterion_group, criterion_main};

fn bench(c: &mut Criterion) {
    let jobs = JobSystem::new(JobSystemConfig::default()).unwrap();
    let mut d = Design::new("bench");
    let root = d.root();
    let mut first = None;
    for i in 0..50 {
        let p = d.add_param(&format!("ms{i}"), "1", None, "").unwrap();
        first.get_or_insert(p);
        let f = FeatureBox::new(TestSlow {
            ms: Expr::Param(p),
            tag: i,
            panic: false,
        });
        d.add_feature(None, f, root, None).unwrap();
    }
    let p0 = first.unwrap();
    let mut doc = Document::from_design(d);
    doc.recompute_blocking(&jobs).unwrap();

    c.bench_function("document/poll_idle", |b| b.iter(|| doc.poll(&jobs)));

    let mut n = 0u32;
    c.bench_function("document/edit_poll_supersede", |b| {
        b.iter(|| {
            n += 1;
            doc.apply(Command::SetParamExpr {
                id: p0,
                expr: format!("{}", 1000 + n),
            })
            .unwrap();
            doc.poll(&jobs);
        });
    });
}

criterion_group!(benches, bench);
criterion_main!(benches);
