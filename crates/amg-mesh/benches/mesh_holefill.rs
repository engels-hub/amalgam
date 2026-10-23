//! MESH-20 budget (S07): a 500-edge hole fills in < 200 ms (default options: triangulation,
//! Liepa refinement, bi-harmonic fairing). Run with `cargo bench -p amg-mesh --bench mesh_holefill`.

#![allow(clippy::expect_used, clippy::print_stderr)] // benchmark setup panics on bad fixtures

#[path = "../tests/holefill_common/mod.rs"]
mod hf;

use amg_core::Point3;
use amg_jobs::JobCtx;
use amg_mesh::{HalfEdgeMesh, HoleFillOpts, IndexedMesh, NonManifoldPolicy, fill_holes};
use criterion::{BatchSize, Criterion, criterion_group, criterion_main};

/// A tube with its first ring closed by a centre fan, so exactly one hole (of `n` edges) is left.
fn one_hole(n: u32, wobble: f64) -> IndexedMesh {
    let m = hf::tube(n, 6, 1.0, wobble, 0.05);
    let (mut positions, mut tris) = (m.positions, m.tris);
    let centre = (0..n).map(|i| positions[i as usize]).sum::<Point3>() / f64::from(n);
    let c = positions.len() as u32;
    positions.push(centre);
    for i in 0..n {
        tris.push([c, (i + 1) % n, i]);
    }
    IndexedMesh::new(positions, tris).expect("valid")
}

fn bench(cr: &mut Criterion) {
    let ctx = JobCtx::detached();
    let opts = HoleFillOpts::default();
    let mut g = cr.benchmark_group("mesh_holefill");
    g.sample_size(10);
    for (name, n, wobble) in [
        ("hole_500_nonplanar", 500, 0.4),
        ("hole_500_planar", 500, 0.0),
        ("hole_250_nonplanar_dp", 250, 0.4),
        ("hole_100_nonplanar_dp", 100, 0.4),
    ] {
        let m = one_hole(n, wobble);
        let (he, _) =
            HalfEdgeMesh::from_indexed(&m, NonManifoldPolicy::SplitVertices).expect("halfedge");
        let mut probe = he.clone();
        let rep = fill_holes(&mut probe, &opts, &ctx).expect("fill");
        eprintln!("{name}: {} new faces", rep.new_faces);
        g.bench_function(name, |b| {
            b.iter_batched(
                || he.clone(),
                |mut h| fill_holes(&mut h, &opts, &ctx).expect("fill"),
                BatchSize::LargeInput,
            );
        });
    }
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
