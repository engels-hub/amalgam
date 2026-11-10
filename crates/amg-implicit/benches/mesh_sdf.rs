//! IMP-03 budget (S08 Performance targets): 2 M signed `MeshSdf` queries on a 1 M-tri mesh
//! (band 2 mm) < 2 s. Run with `cargo bench -p amg-implicit --bench mesh_sdf`.
//!
//! - `signed_2m_band`: all points within the band (the narrow-band sampling workload, worst
//!   case for the closest-point search). This is the budget case.
//! - `signed_2m_box`: points uniform in the bounding box + 5 mm (most beyond the band).

#![allow(clippy::expect_used, reason = "benchmark setup")]

#[path = "../tests/common/mod.rs"]
mod common;

use std::hint::black_box;

use amg_implicit::MeshSdf;
use amg_mesh::iso::ScalarField;
use criterion::{Criterion, criterion_group, criterion_main};
use rayon::prelude::*;

fn bench(c: &mut Criterion) {
    let m = common::budget_mesh();
    assert_eq!(m.face_count(), 1_003_520);
    let band_pts = common::band_points(&m, 2_000_000, 2.0, 11);
    let box_pts = common::box_points(&m, 2_000_000, 5.0, 12);
    let sdf = MeshSdf::new(common::spatial(m), 2.0);
    let mut out = vec![0.0; 2_000_000];

    let mut g = c.benchmark_group("mesh_sdf_1m");
    g.sample_size(10);
    g.bench_function("signed_2m_band", |b| {
        b.iter(|| sdf.values(black_box(&band_pts), &mut out))
    });
    g.bench_function("signed_2m_box", |b| {
        b.iter(|| sdf.values(black_box(&box_pts), &mut out))
    });
    // Parts of the band case, for profiling.
    g.bench_function("parts/unsigned_2m_band", |b| {
        b.iter(|| band_pts.par_iter().map(|&p| sdf.unsigned(p)).sum::<f64>())
    });
    g.bench_function("parts/is_inside_2m_band", |b| {
        b.iter(|| band_pts.par_iter().filter(|&&p| sdf.is_inside(p)).count())
    });
    for beta in [1.0, 2.0] {
        g.bench_function(format!("parts/winding_beta{beta}_2m_band"), |b| {
            b.iter(|| sdf.spatial().winding_numbers(black_box(&band_pts), beta))
        });
    }
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
