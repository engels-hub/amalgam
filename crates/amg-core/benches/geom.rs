//! FND-03 budget: `ray_triangle` throughput ≥ 50 M tests/s (single thread).
//!
//! Each iteration tests 64 rays against a 32×32 grid mesh (2048 triangles), so
//! criterion's `Elements` throughput reads directly as ray/triangle tests per second. About
//! 1 in 2000 tests is a hit, as in picking.

use std::hint::black_box;

use amg_core::{Dir3, Point3, Ray3, Tolerance, Triangle3, Vec3, intersect::ray_triangle};
use criterion::{Criterion, Throughput, criterion_group, criterion_main};

fn mesh(n: i32) -> Vec<Triangle3> {
    let mut tris = Vec::new();
    for i in 0..n {
        for j in 0..n {
            let (x, y) = (f64::from(i), f64::from(j));
            let z = |x: f64, y: f64| 0.1 * (x * 0.7).sin() * (y * 0.3).cos();
            let p = |x: f64, y: f64| Point3::new(x, y, z(x, y));
            let (p00, p10, p01, p11) = (p(x, y), p(x + 1.0, y), p(x, y + 1.0), p(x + 1.0, y + 1.0));
            tris.push(Triangle3 {
                a: p00,
                b: p10,
                c: p11,
            });
            tris.push(Triangle3 {
                a: p00,
                b: p11,
                c: p01,
            });
        }
    }
    tris
}

fn rays(n: usize, extent: f64) -> Vec<Ray3> {
    let tol = Tolerance::DEFAULT;
    (0..n)
        .filter_map(|i| {
            let f = i as f64;
            let target = Point3::new(
                (f * 0.618_034).fract() * extent,
                (f * 0.414_214).fract() * extent,
                0.0,
            );
            let origin =
                Point3::new(extent * 0.5, extent * 0.5, 40.0) + Vec3::new(f.sin(), f.cos(), 0.0);
            Some(Ray3 {
                origin,
                dir: Dir3::new(target - origin, &tol)?,
            })
        })
        .collect()
}

fn ray_triangle_throughput(c: &mut Criterion) {
    let tris = mesh(32);
    let rays = rays(64, 32.0);
    let mut g = c.benchmark_group("geom");
    g.throughput(Throughput::Elements((tris.len() * rays.len()) as u64));
    g.bench_function("ray_triangle", |b| {
        b.iter(|| {
            let mut hits = 0u32;
            for r in &rays {
                for t in &tris {
                    hits += u32::from(ray_triangle(black_box(r), black_box(t)).is_some());
                }
            }
            hits
        })
    });
    g.finish();
}

criterion_group!(benches, ray_triangle_throughput);
criterion_main!(benches);
