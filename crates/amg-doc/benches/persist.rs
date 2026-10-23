//! DOC-14: opening with and without the persisted cache, and the cost of journaling a commit.
//!
//! `persist/open_cached_50` loads a saved 50-feature design and recomputes it (all lazy cache
//! hits); `persist/open_no_cache_50` does the same from a compact file (50 computes of cheap test
//! features, so this mostly shows the per-entry cost of the lazy zip reads against the engine).
//! `journal/commit` is `Document::apply` with the recovery journal attached versus without.

#![allow(clippy::unwrap_used, missing_docs, reason = "benchmark harness")]

use amg_doc::testing::TestCounter;
use amg_doc::{
    BodyId, Command, Dependency, Design, Document, FeatureBox, RecoveryConfig, RecoverySession,
    SaveOptions, load, save,
};
use amg_jobs::{JobSystem, JobSystemConfig};
use criterion::{Criterion, criterion_group, criterion_main};

fn chain(n: usize) -> Design {
    let mut d = Design::new("bench");
    let root = d.root();
    let mut last = None;
    for i in 0..n {
        let deps = last
            .map(|f| vec![Dependency::Body(BodyId { origin: f, slot: 0 })])
            .unwrap_or_default();
        let f = FeatureBox::new(TestCounter {
            deps,
            value: i as i64,
            fail: false,
        });
        last = Some(d.add_feature(None, f, root, None).unwrap());
    }
    d
}

fn bench(c: &mut Criterion) {
    let jobs = JobSystem::new(JobSystemConfig::default()).unwrap();
    let dir = std::env::temp_dir().join(format!("amg-doc-bench-persist-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();

    let mut doc = Document::from_design(chain(50));
    doc.recompute_blocking(&jobs).unwrap();
    let full = dir.join("full.amg");
    let compact = dir.join("compact.amg");
    let with = SaveOptions {
        cache: Some(doc.cache_export()),
        ..SaveOptions::default()
    };
    save(&full, &doc.snapshot(), &with).unwrap();
    save(&compact, &doc.snapshot(), &SaveOptions::default()).unwrap();

    c.bench_function("persist/open_cached_50", |b| {
        b.iter(|| {
            let mut d = load(&full).unwrap().into_document();
            d.recompute_blocking(&jobs).unwrap();
            assert!(d.last_run().unwrap().computed.is_empty());
        });
    });
    c.bench_function("persist/open_no_cache_50", |b| {
        b.iter(|| {
            let mut d = load(&compact).unwrap().into_document();
            d.recompute_blocking(&jobs).unwrap();
        });
    });

    // Journaling: the same edit with and without a session attached.
    let mut plain = Document::new("plain");
    let p = plain
        .apply(Command::AddParam {
            name: "w".into(),
            expr: "1".into(),
            unit: None,
            comment: String::new(),
        })
        .unwrap()
        .created;
    let _ = p;
    let mut journaled = Document::new("journaled");
    let session = RecoverySession::start(
        &RecoveryConfig::new(dir.join("recovery")),
        journaled.design().meta().uuid,
    )
    .unwrap();
    session.attach(&mut journaled);
    let mut n = 0u64;
    let mut edit = |d: &mut Document| {
        n += 1;
        d.apply(Command::AddParam {
            name: format!("p{n}"),
            expr: "1".into(),
            unit: None,
            comment: String::new(),
        })
        .unwrap();
    };
    c.bench_function("journal/commit_plain", |b| b.iter(|| edit(&mut plain)));
    c.bench_function("journal/commit_journaled", |b| {
        b.iter(|| edit(&mut journaled));
    });
    session.flush();
    drop(session);
    let _ = std::fs::remove_dir_all(&dir);
}

criterion_group!(benches, bench);
criterion_main!(benches);
