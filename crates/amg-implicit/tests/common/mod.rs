//! Shared helpers for the `amg-implicit` tests and benches.

#![allow(dead_code, clippy::expect_used, reason = "shared test helpers")]

use std::sync::Arc;

use amg_core::{Aabb3, Point3};
use amg_jobs::JobCtx;
use amg_mesh::corpus::bench::{BenchSize, benchmark_sphere};
use amg_mesh::{BvhOpts, FaceId, IndexedMesh, MeshSpatial};

/// Small deterministic generator (SplitMix64).
#[derive(Clone, Debug)]
pub struct Rng(pub u64);

impl Rng {
    pub fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    pub fn f(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    pub fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.f()
    }
    pub fn below(&mut self, n: usize) -> usize {
        (self.next() % n as u64) as usize
    }
    pub fn in_box(&mut self, b: &Aabb3) -> Point3 {
        Point3::new(
            self.range(b.min.x, b.max.x),
            self.range(b.min.y, b.max.y),
            self.range(b.min.z, b.max.z),
        )
    }
}

pub fn spatial(m: IndexedMesh) -> Arc<MeshSpatial> {
    Arc::new(
        MeshSpatial::build(Arc::new(m), &BvhOpts::default(), &JobCtx::detached()).expect("bvh"),
    )
}

/// A point on a random face, moved along the face normal by a uniform offset in
/// `[-max_off, max_off]`. Faces are picked uniformly by index.
pub fn near_surface(m: &IndexedMesh, rng: &mut Rng, max_off: f64) -> Point3 {
    loop {
        let f = rng.below(m.face_count()) as u32;
        let [a, b, c] = m.triangle(FaceId(f));
        let n = (b - a).cross(c - a);
        let len = n.length();
        if len.is_nan() || len <= 0.0 {
            continue;
        }
        let (mut u, mut v) = (rng.f(), rng.f());
        if u + v > 1.0 {
            (u, v) = (1.0 - u, 1.0 - v);
        }
        let q = a + (b - a) * u + (c - a) * v;
        return q + n / len * rng.range(-max_off, max_off);
    }
}

/// The 1 M-triangle budget mesh: the S07 displaced benchmark sphere (1 003 520 triangles)
/// scaled by 5 to a ~100 mm part (radius 50 mm, 6 % relief).
pub fn budget_mesh() -> IndexedMesh {
    let mut m = benchmark_sphere(BenchSize::M1, 1);
    for p in &mut m.positions {
        *p *= 5.0;
    }
    m
}

/// `n` deterministic query points within `band` of the surface of `m`.
pub fn band_points(m: &IndexedMesh, n: usize, band: f64, seed: u64) -> Vec<Point3> {
    let mut rng = Rng(seed);
    (0..n).map(|_| near_surface(m, &mut rng, band)).collect()
}

/// `n` deterministic query points uniform in the mesh bounding box grown by `pad`.
pub fn box_points(m: &IndexedMesh, n: usize, pad: f64, seed: u64) -> Vec<Point3> {
    let b = m.aabb();
    let b = Aabb3::new(b.min - Point3::splat(pad), b.max + Point3::splat(pad));
    let mut rng = Rng(seed);
    (0..n).map(|_| rng.in_box(&b)).collect()
}

/// Mean edge length of `m`.
pub fn mean_edge(m: &IndexedMesh) -> f64 {
    let mut sum = 0.0;
    for t in &m.tris {
        for k in 0..3 {
            sum += (m.positions[t[k] as usize] - m.positions[t[(k + 1) % 3] as usize]).length();
        }
    }
    sum / (3.0 * m.face_count().max(1) as f64)
}
