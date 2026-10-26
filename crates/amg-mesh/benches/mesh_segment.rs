//! MESH-12 budget (S07): face-group segmentation of a 1 M-triangle mesh takes < 2 s.
//! Run with `cargo bench -p amg-mesh --bench mesh_segment`.

#![allow(clippy::expect_used, clippy::print_stderr)] // benchmark setup panics on bad fixtures

#[path = "../tests/segment_common/mod.rs"]
mod parts;

use amg_jobs::JobCtx;
use amg_mesh::{SegmentOpts, segment_face_groups};
use criterion::{Criterion, criterion_group, criterion_main};

fn bench(cr: &mut Criterion) {
    let ctx = JobCtx::detached();
    let cases = [
        ("l_bracket_1.1M", parts::l_bracket_fine(15, false)),
        ("l_bracket_1.1M_noise", parts::l_bracket_fine(15, true)),
        ("shaft_1.1M", parts::shaft_fine(1200, 0.1, false)),
        ("shaft_1.1M_noise", parts::shaft_fine(1200, 0.1, true)),
    ];
    let mut g = cr.benchmark_group("mesh_segment");
    g.sample_size(10);
    for (name, p) in cases {
        let opts = SegmentOpts::default();
        let mut m = p.mesh.clone();
        let groups = segment_face_groups(&mut m, &opts, &ctx).expect("segment");
        eprintln!("{name}: {} faces, {groups} groups", p.mesh.face_count());
        g.bench_function(name, |b| {
            b.iter_batched(
                || p.mesh.clone(),
                |mut m| segment_face_groups(&mut m, &opts, &ctx).expect("segment"),
                criterion::BatchSize::LargeInput,
            );
        });
    }
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
