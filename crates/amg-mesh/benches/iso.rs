//! MESH-17 budget (S07): 400³-effective narrow-band extraction of a 100 mm part < 2 s.
//! Run with `cargo bench -p amg-mesh --bench iso`.

#![allow(clippy::expect_used)] // benchmark setup panics on bad fixtures

use std::hint::black_box;

use amg_core::{Aabb3, Point3};
use amg_jobs::JobCtx;
use amg_mesh::iso::{IsoOpts, ScalarField, extract_isosurface};
use criterion::{Criterion, criterion_group, criterion_main};

/// Hollow sphere (outer radius 45 mm, 3 mm wall): max(s, -(s + t)), 1-Lipschitz.
struct Shell;
impl ScalarField for Shell {
    fn value(&self, p: Point3) -> f64 {
        let s = p.length() - 45.0;
        s.max(-(s + 3.0))
    }
    fn may_cross(&self, b: &Aabb3, iso: f64) -> bool {
        self.value(b.center()).abs() - iso.abs() <= b.size().length() * 0.5
    }
}

fn bench(c: &mut Criterion) {
    let bounds = Aabb3::new(Point3::splat(-50.0), Point3::splat(50.0));
    let ctx = JobCtx::detached();
    let mut g = c.benchmark_group("iso_100mm_part");
    g.sample_size(10);
    // 100 mm / 0.25 mm = 400 cells per axis.
    let opts = IsoOpts::new(bounds, 0.25);
    g.bench_function("shell_400cubed", |b| {
        b.iter_with_large_drop(|| extract_isosurface(&Shell, black_box(&opts), &ctx).expect("iso"))
    });
    let mut projected = IsoOpts::new(bounds, 0.5);
    projected.project_steps = 2;
    g.bench_function("shell_200cubed_project2", |b| {
        b.iter_with_large_drop(|| {
            extract_isosurface(&Shell, black_box(&projected), &ctx).expect("iso")
        })
    });
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
