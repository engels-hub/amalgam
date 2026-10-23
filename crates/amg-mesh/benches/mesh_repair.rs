//! MESH-04 budget (S07): repair of a typical 1 M-triangle mesh in < 3 s. "Typical" is a
//! displaced sphere with a handful of every defect class (holes, an unwelded seam, flipped,
//! duplicate and degenerate faces, fins, debris). Run with
//! `cargo bench -p amg-mesh --bench mesh_repair`.

#![allow(clippy::expect_used, clippy::print_stderr)] // benchmark setup panics on bad fixtures

use std::hint::black_box;

use amg_jobs::JobCtx;
use amg_mesh::corpus::synth::{DefectSpec, Shape, SynthSpec, generate};
use amg_mesh::{IndexedMesh, RepairOpts, repair};
use criterion::{Criterion, criterion_group, criterion_main};

fn typical(frequency: u32) -> IndexedMesh {
    generate(&SynthSpec {
        shape: Shape::DisplacedSphere {
            frequency,
            radius: 50.0,
            amplitude: 0.06,
        },
        defects: vec![
            DefectSpec::Holes {
                count: 10,
                max_boundary: 60,
            },
            DefectSpec::UnweldedSeam { jitter: 5e-6 },
            DefectSpec::FlippedFaces { count: 200 },
            DefectSpec::DuplicateFaces { count: 100 },
            DefectSpec::ZeroAreaSlivers { count: 50 },
            DefectSpec::NeedleSlivers { count: 50 },
            DefectSpec::NonManifoldFins { count: 20 },
            DefectSpec::FloatingShells { count: 5 },
        ],
        seed: 1,
    })
    .mesh
}

fn bench(c: &mut Criterion) {
    let ctx = JobCtx::detached();
    let mut g = c.benchmark_group("mesh_repair");
    g.sample_size(10);
    for (name, frequency) in [("100k", 71), ("1m", 224)] {
        let m = typical(frequency);
        let opts = RepairOpts::default();
        let (_, rep) = repair(&m, &opts, &ctx).expect("repair");
        eprintln!("{name}: {} faces", m.tris.len());
        for s in &rep.steps {
            eprintln!(
                "  round {} {:?}: {} changed, {} ms",
                s.round, s.kind, s.changed, s.millis
            );
        }
        g.bench_function(name, |b| {
            b.iter_with_large_drop(|| repair(black_box(&m), &opts, &ctx).expect("repair"))
        });
    }
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
