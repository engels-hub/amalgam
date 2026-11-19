//! UI-08 Done-when 5: the timeline's per-frame cost with 500 nodes (budget < 1 ms).
//!
//! 500 trivial features in the root component; the strip is drawn alone in a 1280 × 34 pt
//! panel, so the horizontal `ScrollArea` lays out only the slots in view. `steady_frame` is a
//! frame without document changes (the cached nodes are drawn); `rebuild` invalidates the node
//! cache first (what a document change costs). `AMG_STRICT_TIMING=1` asserts the 1 ms budget
//! on the steady frame's median.

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "bench code")]

use std::time::{Duration, Instant};

use amg_doc::{Command, Dependency, FeatureBox, FeatureError, FeatureOutput, RecomputeCtx};
use amg_ui::CommitBatch;
use amg_ui::testing::AppHarness;
use criterion::{Criterion, criterion_group, criterion_main};
use serde::{Deserialize, Serialize};

const NODES: usize = 500;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Nothing {
    n: u32,
}

impl amg_doc::Feature for Nothing {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, _ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Ok(FeatureOutput::default())
    }
}

amg_doc::register_feature!(Nothing, "uibench.timeline_nothing", version = 1);

fn setup() -> AppHarness {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    let root = h.document().design().root();
    let commands = (0..NODES)
        .map(|i| Command::AddFeature {
            at: None,
            feature: FeatureBox::new(Nothing {
                n: u32::try_from(i).unwrap(),
            }),
            context: root,
            name: None,
        })
        .collect();
    h.app_mut().submit(CommitBatch {
        label: "Nodes".into(),
        commands,
    });
    h.drive_until_idle();
    assert_eq!(h.document().design().timeline().len(), NODES);
    h
}

/// One frame of the timeline alone in a 1280 × 34 pt strip.
fn frame(ctx: &egui::Context, h: &mut AppHarness) {
    let input = egui::RawInput {
        screen_rect: Some(egui::Rect::from_min_size(
            egui::Pos2::ZERO,
            egui::vec2(1280.0, 34.0),
        )),
        ..egui::RawInput::default()
    };
    let _ = ctx.run_ui(input, |ui| h.app_mut().timeline_ui(ui));
}

fn bench(c: &mut Criterion) {
    let mut h = setup();
    let ctx = egui::Context::default();
    frame(&ctx, &mut h);
    frame(&ctx, &mut h);
    assert_eq!(h.app().timeline().len(), NODES);
    let drawn = h.app().timeline().drawn().len();
    assert!(
        drawn > 0 && drawn < NODES,
        "only the slots in view are drawn: {drawn}"
    );
    let rebuilds = h.app().timeline().rebuilds();

    let mut g = c.benchmark_group("timeline_500_nodes");
    g.bench_function("steady_frame", |b| b.iter(|| frame(&ctx, &mut h)));
    assert_eq!(
        h.app().timeline().rebuilds(),
        rebuilds,
        "steady frames never rebuild"
    );
    g.bench_function("rebuild", |b| {
        b.iter(|| {
            h.app_mut().timeline_mut().invalidate();
            frame(&ctx, &mut h);
        });
    });
    g.finish();

    if std::env::var("AMG_STRICT_TIMING").as_deref() == Ok("1") {
        let mut times: Vec<Duration> = (0..200)
            .map(|_| {
                let t = Instant::now();
                frame(&ctx, &mut h);
                t.elapsed()
            })
            .collect();
        times.sort();
        let median = times[times.len() / 2];
        assert!(
            median < Duration::from_millis(1),
            "timeline frame median {median:?} ≥ 1 ms"
        );
    }
}

criterion_group!(benches, bench);
criterion_main!(benches);
