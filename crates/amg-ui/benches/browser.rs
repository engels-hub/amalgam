//! UI-07 Done-when 3: the browser's per-frame cost with 10 000 bodies (budget < 1 ms).
//!
//! One feature makes 10 000 one-triangle mesh bodies in the root component; the Bodies
//! folder is expanded, so the tree has 10 000 rows and `show_rows` lays out only those in
//! view. `steady_frame` is a frame without document changes (the cached tree is drawn);
//! `rebuild` forces a full rebuild of the tree first (what an affecting event costs).
//! `AMG_STRICT_TIMING=1` asserts the 1 ms budget on the steady frame's median.

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "bench code")]

use std::sync::Arc;
use std::time::{Duration, Instant};

use amg_core::Point3;
use amg_doc::stub::IndexedMesh;
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, Command, Dependency, FeatureBox, FeatureError,
    FeatureOutput, RecomputeCtx,
};
use amg_ui::CommitBatch;
use amg_ui::browser::NodeKey;
use amg_ui::testing::AppHarness;
use criterion::{Criterion, criterion_group, criterion_main};
use serde::{Deserialize, Serialize};

const BODIES: u32 = 10_000;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct ManyBodies {
    count: u32,
}

impl amg_doc::Feature for ManyBodies {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let mesh = Arc::new(
            IndexedMesh::new(
                vec![
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(1.0, 0.0, 0.0),
                    Point3::new(0.0, 1.0, 0.0),
                ],
                vec![[0, 1, 2]],
            )
            .expect("triangle"),
        );
        let bodies = (0..self.count)
            .map(|i| BodyChange {
                id: ctx.new_body_id(i),
                change: BodyChangeKind::Created {
                    geom: BodyGeom::Mesh(Arc::clone(&mesh)),
                    naming: None,
                },
            })
            .collect();
        Ok(FeatureOutput {
            bodies,
            ..FeatureOutput::default()
        })
    }
}

amg_doc::register_feature!(ManyBodies, "uibench.browser_many_bodies", version = 1);

fn setup() -> AppHarness {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    let root = h.document().design().root();
    h.app_mut().submit(CommitBatch {
        label: "Bodies".into(),
        commands: vec![Command::AddFeature {
            at: None,
            feature: FeatureBox::new(ManyBodies { count: BODIES }),
            context: root,
            name: None,
        }],
    });
    h.drive_until_idle();
    assert_eq!(h.document().evaluated().bodies().count(), BODIES as usize);
    let key = NodeKey::owned(format!("bodies/{}", root.0));
    h.app_mut().browser_mut().set_expanded(&key, true);
    h
}

/// One frame of the browser alone in a 230 × 900 pt panel.
fn frame(ctx: &egui::Context, h: &mut AppHarness) {
    let input = egui::RawInput {
        screen_rect: Some(egui::Rect::from_min_size(
            egui::Pos2::ZERO,
            egui::vec2(230.0, 900.0),
        )),
        ..egui::RawInput::default()
    };
    let _ = ctx.run_ui(input, |ui| h.app_mut().browser_ui(ui));
}

fn bench(c: &mut Criterion) {
    let mut h = setup();
    let ctx = egui::Context::default();
    frame(&ctx, &mut h);
    frame(&ctx, &mut h);
    let rows = h.app().browser().row_count();
    assert!(
        rows > BODIES as usize,
        "the Bodies folder is expanded: {rows} rows"
    );
    let rebuilds = h.app().browser().rebuilds();

    let mut g = c.benchmark_group("browser_10k_bodies");
    g.bench_function("steady_frame", |b| b.iter(|| frame(&ctx, &mut h)));
    assert_eq!(
        h.app().browser().rebuilds(),
        rebuilds,
        "steady frames never rebuild"
    );
    g.sample_size(20);
    g.bench_function("rebuild", |b| {
        b.iter(|| {
            h.app_mut().browser_mut().invalidate();
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
            "browser frame median {median:?} ≥ 1 ms"
        );
    }
}

criterion_group!(benches, bench);
criterion_main!(benches);
