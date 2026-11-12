//! UI-02 budget (S11 §4): S-key search over 1 000 commands < 1 ms per keystroke.
//! Run with `cargo bench -p amg-ui --bench search`.
//!
//! Timing is reported by criterion only; the budget is asserted in the unit test
//! `search_1000_commands_budget` under `AMG_STRICT_TIMING=1`.

#![allow(clippy::expect_used)] // benchmark setup

use std::hint::black_box;

use amg_ui::UiRegistry;
use amg_ui::command::{CmdAction, CmdId, CommandDef};
use amg_ui::module::{Registrar, UiModule};
use amg_ui::search::{Recency, SearchIndex};
use criterion::{Criterion, criterion_group, criterion_main};

const VERBS: [&str; 10] = [
    "Create", "Modify", "Extrude", "Remesh", "Smooth", "Offset", "Project", "Measure", "Export",
    "Import",
];
const NOUNS: [&str; 10] = [
    "Sketch", "Body", "Mesh", "Plane", "Profile", "Surface", "Shell", "Lattice", "Support",
    "Toolpath",
];

fn register(r: &mut Registrar<'_>) {
    for i in 0..1000 {
        let id: &'static str = Box::leak(format!("syn.cmd{i}").into_boxed_str());
        let label = format!("{} {} {}", VERBS[i % 10], NOUNS[(i / 10) % 10], i / 100);
        r.command(CommandDef::new(id, label, CmdAction::Run(|_| {})).with_keywords(&["synthetic"]));
    }
}

fn bench(c: &mut Criterion) {
    let module = UiModule {
        id: "syn",
        register,
    };
    let reg = UiRegistry::from_modules([&module]);
    let index = SearchIndex::build(&reg);
    assert_eq!(index.len(), 1000);
    let mut recency = Recency::new();
    for i in 0..20 {
        let id: &'static str = Box::leak(format!("syn.cmd{}", i * 37).into_boxed_str());
        recency.record(CmdId(id));
    }
    c.bench_function("search_1000_commands_per_keystroke", |b| {
        b.iter(|| {
            for q in ["r", "re", "rem", "remsh", "remsh mesh"] {
                black_box(index.search(black_box(q), &recency));
            }
        });
    });
    c.bench_function("search_1000_commands_single_query", |b| {
        b.iter(|| black_box(index.search(black_box("remsh"), &recency)));
    });
    c.bench_function("index_build_1000_commands", |b| {
        b.iter(|| black_box(SearchIndex::build(&reg)));
    });
}

criterion_group!(benches, bench);
criterion_main!(benches);
