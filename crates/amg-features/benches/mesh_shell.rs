//! MESH-10 budget (S07): robust SDF shell of a 1 M-triangle, 100 mm part at 0.25 mm voxels
//! < 20 s (2 mm wall, inside). Run with `cargo bench -p amg-features --bench mesh_shell`.

#![allow(missing_docs, clippy::expect_used, clippy::print_stderr)] // benchmark setup

use std::sync::Arc;
use std::time::Duration;

use amg_features::mesh::shell::{SdfShellOpts, offset_shell_sdf};
use amg_jobs::JobCtx;
use amg_mesh::corpus::bench::{BenchSize, benchmark_sphere};
use amg_mesh::{IndexedMesh, ShellSide};
use criterion::{Criterion, criterion_group, criterion_main};

/// The S07 benchmark sphere scaled to a ~100 mm part (radius 50 mm, 6 % relief).
fn part(size: BenchSize) -> Arc<IndexedMesh> {
    let mut m = benchmark_sphere(size, 1);
    for p in &mut m.positions {
        *p *= 5.0;
    }
    Arc::new(m)
}

fn bench(cr: &mut Criterion) {
    let ctx = JobCtx::detached();
    let mut g = cr.benchmark_group("mesh_shell");
    g.sample_size(10);
    g.measurement_time(Duration::from_secs(60));
    let opts = SdfShellOpts::new(2.0, ShellSide::Inside, 0.25);
    for (name, size) in [
        ("robust_shell_100k_100mm_0.25", BenchSize::K100),
        ("robust_shell_1M_100mm_0.25", BenchSize::M1),
    ] {
        let m = part(size);
        let s = offset_shell_sdf(&m, &opts, &ctx).expect("shell");
        eprintln!("{name}: {} -> {} tris", m.face_count(), s.face_count());
        g.bench_function(name, |b| {
            b.iter(|| offset_shell_sdf(&m, &opts, &ctx).expect("shell"));
        });
    }
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
