//! S02 budget (BREP-08 Done-when 1): tessellating a 1000-face part at the display preset takes
//! under 100 ms (median), including the private topology copy.
//!
//! `budget/brep-tess-1000` meshes the 1 030-face `plate_with_holes` fixture through
//! `BrepKernel::tessellate`. Before the criterion groups run, the copy / mesh / copy-out split
//! of `tessellate_profiled` is printed (median of 15 runs), which is the copy share the
//! Done-when asks to record. `brep-tess/*` covers the other presets and `tessellate_many`.
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "benchmark code")]
#![allow(clippy::print_stderr, reason = "benchmark report")]

use std::hint::black_box;

use amg_brep::{BrepKernel, JobCtx, KernelConfig, OcctKernel, Shape, TessQuality};
use criterion::{Criterion, criterion_group, criterion_main};

fn median(mut values: Vec<f64>) -> f64 {
    values.sort_by(f64::total_cmp);
    values[values.len() / 2]
}

fn bench(c: &mut Criterion) {
    let Some(path) = amg_testkit::testdata("brep/plate_with_holes.brep") else {
        return;
    };
    let file = std::fs::File::open(path).expect("open fixture");
    let plate = Shape::read_text(std::io::BufReader::new(file)).expect("read fixture");
    let kernel = OcctKernel::new(KernelConfig::default());
    let ctx = JobCtx::detached();
    let display = TessQuality::display();

    let runs: Vec<_> = (0..15)
        .map(|_| {
            kernel
                .tessellate_profiled(&plate, &display, &ctx)
                .expect("tessellate")
                .1
        })
        .collect();
    let copy = median(runs.iter().map(|p| p.copy_seconds).collect());
    let mesh = median(runs.iter().map(|p| p.mesh_seconds).collect());
    let extract = median(runs.iter().map(|p| p.extract_seconds).collect());
    let total = copy + mesh + extract;
    eprintln!(
        "brep-tess-1000 split (median of 15, {} faces, deflection {:.4} mm): copy {:.2} ms \
         ({:.1} %), mesh {:.2} ms, copy-out {:.2} ms, sum {:.2} ms",
        plate.faces().len(),
        runs[0].deflection,
        copy * 1e3,
        100.0 * copy / total,
        mesh * 1e3,
        extract * 1e3,
        total * 1e3
    );

    let mut group = c.benchmark_group("budget");
    group.sample_size(30);
    group.bench_function("brep-tess-1000", |b| {
        b.iter(|| {
            black_box(
                kernel
                    .tessellate(&plate, &display, &ctx)
                    .expect("tessellate"),
            )
        });
    });
    group.finish();

    let mut group = c.benchmark_group("brep-tess");
    group.sample_size(20);
    group.bench_function("1000-preview", |b| {
        b.iter(|| {
            black_box(
                kernel
                    .tessellate(&plate, &TessQuality::preview(), &ctx)
                    .expect("tessellate"),
            )
        });
    });
    group.bench_function("1000-export", |b| {
        b.iter(|| {
            black_box(
                kernel
                    .tessellate(&plate, &TessQuality::export(), &ctx)
                    .expect("tessellate"),
            )
        });
    });
    let bodies = vec![plate.clone(); 8];
    group.bench_function("many-8x1000-display", |b| {
        b.iter(|| black_box(kernel.tessellate_many(&bodies, &display, &ctx)));
    });
    let mesh = kernel
        .tessellate(&plate, &display, &ctx)
        .expect("tessellate");
    group.bench_function("to-indexed-parts-1000", |b| {
        b.iter(|| black_box(mesh.to_indexed_parts()));
    });
    group.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
