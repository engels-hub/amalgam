//! DOC-05: `History::apply` and `undo` on a 200-feature design. Budget: each < 1 ms.

#![allow(clippy::unwrap_used, missing_docs, reason = "benchmark harness")]

use amg_doc::testing::TestBox;
use amg_doc::{Command, Design, FeatureBox, FeatureId, History};
use criterion::{Criterion, criterion_group, criterion_main};

fn design_with(n: usize) -> History {
    amg_doc::testing::link();
    let mut d = Design::new("bench");
    let root = d.root();
    for i in 0..n {
        let f = FeatureBox::new(TestBox {
            size: [1.0 + i as f64, 1.0, 1.0],
        });
        d.add_feature(None, f, root, None).unwrap();
    }
    History::new(d)
}

fn bench(c: &mut Criterion) {
    let mut h = design_with(200);
    let target = h.design().timeline().node_at(100).unwrap().id;
    let mut w = 0.0;
    c.bench_function("apply_edit_feature_200", |b| {
        b.iter(|| {
            w += 1.0;
            let feature = FeatureBox::new(TestBox {
                size: [w, 2.0, 3.0],
            });
            h.apply(Command::EditFeature {
                id: target,
                feature,
            })
            .unwrap()
        });
    });

    let mut h = design_with(200);
    let ids: Vec<FeatureId> = h.design().timeline().nodes().map(|n| n.id).collect();
    c.bench_function("apply_then_undo_suppress_200", |b| {
        let mut i = 0;
        b.iter(|| {
            i = (i + 1) % ids.len();
            h.apply(Command::SetSuppressed {
                ids: vec![ids[i]],
                suppressed: true,
            })
            .unwrap();
            h.undo()
        });
    });

    let mut h = design_with(200);
    for (i, &id) in ids.iter().enumerate().take(50) {
        h.apply(Command::RenameFeature {
            id,
            name: Some(format!("n{i}")),
        })
        .unwrap();
    }
    c.bench_function("undo_redo_pair", |b| {
        b.iter(|| {
            h.undo();
            h.redo()
        });
    });
}

criterion_group!(benches, bench);
criterion_main!(benches);
