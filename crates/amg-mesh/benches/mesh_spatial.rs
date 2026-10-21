//! MESH-03 budgets (S07): 1 M-tri BVH build < 150 ms, 1 M fast-winding-number queries on a
//! 1 M-tri mesh < 1.5 s. Run with `cargo bench -p amg-mesh --bench mesh_spatial`.

#![allow(clippy::expect_used)] // benchmark setup panics on bad fixtures

#[path = "../tests/common/mod.rs"]
mod common;

use std::hint::black_box;
use std::sync::Arc;

use amg_core::{Aabb3, Dir3, Point3, Ray3};
use amg_jobs::JobCtx;
use amg_mesh::{Bvh, BvhOpts, MeshSpatial};
use criterion::{Criterion, criterion_group, criterion_main};
use obvhs::ploc::{PlocBuilder, PlocSearchDistance, SortPrecision};
use rayon::prelude::*;

fn bench(c: &mut Criterion) {
    let m = Arc::new(common::torus(500, 1000, 50.0, 15.0));
    assert_eq!(m.face_count(), 1_000_000);
    let ctx = JobCtx::detached();
    let opts = BvhOpts::default();
    let bounds: Vec<Aabb3> = m
        .tris
        .iter()
        .map(|t| Aabb3::from_points(t.map(|i| m.positions[i as usize])))
        .collect();

    let mut g = c.benchmark_group("spatial_1m");
    g.sample_size(10);

    // obvhs PLOC reference on the same boxes (f32), excluding box construction.
    let ob: Vec<obvhs::aabb::Aabb> = bounds
        .iter()
        .map(|b| obvhs::aabb::Aabb {
            min: [b.min.x as f32, b.min.y as f32, b.min.z as f32].into(),
            max: [b.max.x as f32, b.max.y as f32, b.max.z as f32].into(),
        })
        .collect();
    for (name, dist, thresh) in [
        ("obvhs_ploc_minimum", PlocSearchDistance::Minimum, 0),
        ("obvhs_ploc_low", PlocSearchDistance::Low, 2),
        ("obvhs_ploc_medium", PlocSearchDistance::Medium, 3),
    ] {
        g.bench_function(name, |b| {
            b.iter_with_large_drop(|| {
                let idx: Vec<u32> = (0..ob.len() as u32).collect();
                PlocBuilder::with_capacity(ob.len()).build(
                    dist,
                    black_box(&ob),
                    idx,
                    SortPrecision::U64,
                    thresh,
                )
            })
        });
    }
    g.bench_function("bvh_build", |b| {
        b.iter_with_large_drop(|| Bvh::build(black_box(&bounds), &opts, &ctx).expect("bvh"))
    });
    g.bench_function("mesh_spatial_build", |b| {
        b.iter_with_large_drop(|| MeshSpatial::build(m.clone(), &opts, &ctx).expect("spatial"))
    });

    let s = MeshSpatial::build(m.clone(), &opts, &ctx).expect("spatial");
    g.bench_function("winding_tree_build", |b| {
        b.iter_with_large_drop(|| {
            let fresh = MeshSpatial::build(m.clone(), &opts, &ctx).expect("spatial");
            fresh.prepare_winding(&ctx).expect("tree");
            fresh
        })
    });
    s.prepare_winding(&ctx).expect("tree");

    // 1 M points in a box around the torus, deterministic.
    let pts: Vec<Point3> = (0..1_000_000u64)
        .map(|i| {
            let h = |k: u64| {
                let mut z = (i * 3 + k).wrapping_add(0x9E37_79B9_7F4A_7C15);
                z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
                z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
                ((z ^ (z >> 31)) >> 11) as f64 / (1u64 << 53) as f64
            };
            Point3::new(
                (h(0) - 0.5) * 140.0,
                (h(1) - 0.5) * 140.0,
                (h(2) - 0.5) * 40.0,
            )
        })
        .collect();
    g.bench_function("fwn_1m_queries_parallel", |b| {
        b.iter(|| s.winding_numbers(black_box(&pts), 2.0))
    });
    g.bench_function("closest_point_1m_parallel", |b| {
        b.iter(|| {
            pts.par_iter()
                .map(|&p| s.closest_point(p, f64::INFINITY).map_or(0.0, |c| c.dist_sq))
                .sum::<f64>()
        })
    });
    g.bench_function("ray_cast_1m_parallel", |b| {
        b.iter(|| {
            pts.par_iter()
                .filter(|&&p| {
                    s.ray_cast(
                        &Ray3 {
                            origin: p,
                            dir: Dir3::Z,
                        },
                        f64::INFINITY,
                    )
                    .is_some()
                })
                .count()
        })
    });
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
