//! `hausdorff` on two 1.3M-triangle icospheres with 100 k samples per mesh.
//! S00 target: < 2 s (BVH builds included). Run: `cargo bench -p amg-testkit --bench hausdorff`.

#![allow(missing_docs, reason = "criterion macros")]

use std::time::Duration;

use amg_core::Point3;
use amg_testkit::{hausdorff, shapes};
use criterion::{Criterion, criterion_group, criterion_main};

fn bench(c: &mut Criterion) {
    // Subdivision 8: 20 * 4^8 = 1 310 720 triangles each.
    let a = shapes::icosphere(Point3::ZERO, 10.0, 8);
    let b = shapes::icosphere(Point3::new(0.01, 0.0, 0.0), 10.5, 8);
    let mut g = c.benchmark_group("testkit");
    g.sample_size(10).measurement_time(Duration::from_secs(20));
    g.bench_function("hausdorff_1m_100k", |bch| {
        bch.iter(|| hausdorff(a.soup(), b.soup(), 100_000));
    });
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
