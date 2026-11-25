//! IO-09 Done-when 4: a 1000-face part to a 3MF file at Medium takes under 300 ms end to end.
//!
//! `budget/io-3mf-1000-medium` is `scene_from_document` with a cold tessellation cache, the
//! validity check and the 3MF writer on the 1 000-face star wheel (`-scalloped`: the wheel
//! with every second flank a cylinder); `io-3mf/cached` is the same
//! with a warm cache. The 1 030-face `plate_with_holes` (the S02 budget fixture) is timed once
//! and printed: OCCT's planar mesher dominates it until BREP-18.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::print_stderr,
    reason = "benchmark code"
)]

#[path = "../tests/io_tess_common/mod.rs"]
mod io_tess_common;

use std::hint::black_box;
use std::path::Path;
use std::sync::Arc;
use std::time::Instant;

use amg_doc::Document;
use amg_features::io::{
    ExportPrep, ExportRefinement, ExportSelection, TessCache, scene_from_document,
};
use amg_io::{ExportOptions, Format, Sink};
use amg_jobs::JobCtx;
use criterion::{Criterion, criterion_group, criterion_main};
use io_tess_common::{
    brep_text, document, filleted_boxes_988, fixture, scalloped_wheel_1000, star_wheel_1000,
};

/// Exports `doc` to `path` and returns the seconds spent preparing the scene and in total.
fn export(doc: &Document, cache: &Arc<TessCache>, path: &Path) -> (f64, f64) {
    let ctx = JobCtx::detached();
    let prep = ExportPrep {
        refinement: Some(ExportRefinement::Medium),
        cache: Some(Arc::clone(cache)),
        ..ExportPrep::default()
    };
    let start = Instant::now();
    let scene = scene_from_document(
        doc.design(),
        doc.evaluated(),
        &ExportSelection::AllVisible,
        &prep,
        &ctx,
    )
    .unwrap();
    let meshed = start.elapsed().as_secs_f64();
    let report = amg_io::export(
        &scene,
        Sink::Path(path),
        Format::ThreeMf,
        &ExportOptions::default(),
        &ctx,
    )
    .unwrap();
    black_box(report);
    (meshed, start.elapsed().as_secs_f64())
}

fn cold() -> Arc<TessCache> {
    Arc::new(TessCache::new(1 << 30))
}

fn bench(c: &mut Criterion) {
    amg_features::io::link();
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR"));
    let path = dir.join("io09-bench.3mf");
    let (wheel, _) = document(&[brep_text(&star_wheel_1000())]);

    if let Some(plate) = fixture("plate_with_holes") {
        let (doc, _) = document(&[plate]);
        let (meshed, total) = export(&doc, &cold(), &path);
        eprintln!(
            "plate_with_holes (1030 faces) medium, once: tessellation + scene {:.0} ms, end to \
             end {:.0} ms",
            meshed * 1e3,
            total * 1e3
        );
    }
    let (scalloped, _) = document(&[brep_text(&scalloped_wheel_1000())]);
    let mut parts = vec![
        ("star_wheel_1000", &wheel),
        ("scalloped_wheel_1000", &scalloped),
    ];
    let boxes = filleted_boxes_988().map(|b| document(&b).0);
    if let Some(boxes) = &boxes {
        parts.push(("filleted_boxes_988 (38 bodies)", boxes));
    }
    for (name, doc) in &parts {
        let (meshed, total) = export(doc, &cold(), &path);
        eprintln!(
            "{name} medium, first run: tessellation + scene {:.1} ms, end to end {:.1} ms",
            meshed * 1e3,
            total * 1e3
        );
    }

    let mut group = c.benchmark_group("budget");
    group.sample_size(20);
    group.bench_function("io-3mf-1000-medium", |b| {
        b.iter(|| export(&wheel, &cold(), &path));
    });
    group.bench_function("io-3mf-1000-medium-scalloped", |b| {
        b.iter(|| export(&scalloped, &cold(), &path));
    });
    group.finish();

    let warm = cold();
    export(&wheel, &warm, &path);
    let mut group = c.benchmark_group("io-3mf");
    group.sample_size(20);
    group.bench_function("cached", |b| b.iter(|| export(&wheel, &warm, &path)));
    group.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
