//! MESH-05 budget (S07): the union of two 1 M-triangle offset spheres takes < 1 s end-to-end,
//! conversion to and from Manifold included. Needs the `manifold` feature:
//! `cargo bench -p amg-mesh --features manifold --bench mesh_boolean`.

#![allow(clippy::expect_used, clippy::print_stderr)] // benchmark setup panics on bad fixtures

use amg_jobs::JobCtx;
use amg_mesh::corpus::bench::{BenchSize, boolean_pair};
use amg_mesh::{BoolOp, BoolOpts, boolean};
use criterion::{Criterion, criterion_group, criterion_main};

fn bench(cr: &mut Criterion) {
    let ctx = JobCtx::detached();
    let opts = BoolOpts::default();
    let mut g = cr.benchmark_group("mesh_boolean");
    g.sample_size(10);
    for (name, size) in [("100k", BenchSize::K100), ("1M", BenchSize::M1)] {
        let (a, b) = boolean_pair(size, 7);
        for op in [BoolOp::Union, BoolOp::Difference, BoolOp::Intersection] {
            let (r, rep) = boolean(op, &a, &b, &opts, &ctx).expect("boolean");
            eprintln!(
                "{name} {op:?}: {} + {} faces in, {} out, {} ms",
                a.face_count(),
                b.face_count(),
                r.face_count(),
                rep.millis
            );
            g.bench_function(format!("{op:?}_{name}"), |bch| {
                bch.iter(|| boolean(op, &a, &b, &opts, &ctx).expect("boolean"));
            });
        }
    }
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
