//! UI-12 Done-when 4: the Parameters dialog's per-frame cost with 1 000 parameters
//! (budget < 2 ms).
//!
//! The dialog content is drawn alone in an 880 × 440 pt area; the table lays out only the rows
//! in view. `steady_frame` is a frame without document changes; `rebuild` gives every frame a
//! new revision, so the table lines are rebuilt as after an edit. `AMG_STRICT_TIMING=1`
//! asserts the budget on both medians.

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "bench code")]

use std::time::{Duration, Instant};

use amg_core::Unit;
use amg_doc::Revision;
use amg_doc::model::Design;
use amg_ui::UiRegistry;
use amg_ui::params::ParametersView;
use criterion::{Criterion, criterion_group, criterion_main};

const PARAMS: usize = 1000;

/// User parameters; every fourth one references its predecessor.
fn design() -> Design {
    let mut d = Design::new("params");
    for i in 0..PARAMS {
        let expr = if i % 4 == 0 && i > 0 {
            format!("p{} + 1 mm", i - 1)
        } else {
            format!("{i} mm")
        };
        d.add_param(&format!("p{i}"), &expr, Some(Unit::Millimetre), "note")
            .expect("parameter");
    }
    d
}

fn frame(
    ctx: &egui::Context,
    view: &mut ParametersView,
    design: &Design,
    registry: &UiRegistry,
    revision: u64,
) {
    let input = egui::RawInput {
        screen_rect: Some(egui::Rect::from_min_size(
            egui::Pos2::ZERO,
            egui::vec2(880.0, 440.0),
        )),
        ..egui::RawInput::default()
    };
    let mut out = ctx.run_ui(input, |ui| {
        let _ = view.show(ui, design, Revision(revision), registry);
    });
    // There is no renderer to apply the font texture.
    out.textures_delta.clear();
}

fn bench(c: &mut Criterion) {
    let design = design();
    let registry = UiRegistry::build();
    let ctx = egui::Context::default();
    let mut view = ParametersView::default();
    frame(&ctx, &mut view, &design, &registry, 0);
    frame(&ctx, &mut view, &design, &registry, 0);
    assert_eq!(view.rows().len(), PARAMS + 1);

    let mut g = c.benchmark_group("params_1000");
    g.bench_function("steady_frame", |b| {
        b.iter(|| frame(&ctx, &mut view, &design, &registry, 0));
    });
    let mut revision = 0;
    g.bench_function("rebuild", |b| {
        b.iter(|| {
            revision += 1;
            frame(&ctx, &mut view, &design, &registry, revision);
        });
    });
    g.finish();

    if std::env::var("AMG_STRICT_TIMING").as_deref() == Ok("1") {
        for rebuild in [false, true] {
            let mut times: Vec<Duration> = (0..200)
                .map(|_| {
                    revision += u64::from(rebuild);
                    let t = Instant::now();
                    frame(&ctx, &mut view, &design, &registry, revision);
                    t.elapsed()
                })
                .collect();
            times.sort();
            let median = times[times.len() / 2];
            assert!(
                median < Duration::from_millis(2),
                "params frame (rebuild: {rebuild}) median {median:?} ≥ 2 ms"
            );
        }
    }
}

criterion_group!(benches, bench);
criterion_main!(benches);
