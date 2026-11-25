//! BREP-15: pipe and thru_sections costs (S02 has no budget for them; reported in the
//! Refinement log). A cylinder sweep, a square along the two-turn helix, a 90° twist (with its
//! auxiliary spine), a scale law, the smooth loft through three circles, the 25-case corpus and
//! the 40-section loft of 48-gons (smooth and ruled). Validation is off, as for a release
//! build's default policy.
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "benchmark code")]

#[path = "../tests/common/sweep_corpus.rs"]
mod corpus;

use std::hint::black_box;

use amg_brep::{BrepKernel, JobCtx, KernelConfig, OcctKernel, ThruSectionsOpts, Validate};
use criterion::{Criterion, criterion_group, criterion_main};

fn bench(c: &mut Criterion) {
    let k = OcctKernel::new(KernelConfig {
        validate: Validate::Off,
        ..KernelConfig::default()
    });
    let ctx = JobCtx::detached();
    let cases = corpus::corpus(&k);
    let mut group = c.benchmark_group("brep-sweep");
    for name in [
        "pipe_circle_line",
        "pipe_square_helix",
        "pipe_square_twist",
        "pipe_circle_scale",
        "pipe_square_corner_right",
        "loft_three_circles_smooth",
        "loft_squares_ruled",
    ] {
        let case = cases.iter().find(|c| c.name == name).unwrap();
        group.bench_function(name, |b| b.iter(|| black_box(case.run(&k).unwrap())));
    }
    group.bench_function("corpus-25", |b| {
        b.iter(|| {
            for case in &cases {
                black_box(case.run(&k).unwrap());
            }
        });
    });
    group.sample_size(10);
    let (wires, sections) = corpus::heavy_loft(&k);
    let smooth = ThruSectionsOpts::default();
    group.bench_function("loft-40x48-smooth", |b| {
        b.iter(|| black_box(k.thru_sections(&wires, &sections, &smooth, &ctx).unwrap()));
    });
    let ruled = ThruSectionsOpts {
        ruled: true,
        ..ThruSectionsOpts::default()
    };
    group.bench_function("loft-40x48-ruled", |b| {
        b.iter(|| black_box(k.thru_sections(&wires, &sections, &ruled, &ctx).unwrap()));
    });
    group.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
