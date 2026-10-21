//! MESH-03: cross-check ray cast and closest point against `parry3d-f64` (dev-only oracle).

#![allow(clippy::expect_used)] // test helpers

#[path = "common/mod.rs"]
mod common;

use std::sync::Arc;

use amg_core::{Dir3, Point3, Ray3};
use amg_jobs::JobCtx;
use amg_mesh::{BvhOpts, IndexedMesh, MeshSpatial};
use parry3d_f64::math::Vector;
use parry3d_f64::query::{PointQuery, Ray, RayCast};
use parry3d_f64::shape::TriMesh;

/// SplitMix64.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    fn f(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn point(&mut self, half: f64) -> Point3 {
        Point3::new(
            (self.f() * 2.0 - 1.0) * half,
            (self.f() * 2.0 - 1.0) * half,
            (self.f() * 2.0 - 1.0) * half,
        )
    }
}

fn random_soup(rng: &mut Rng, n: usize) -> IndexedMesh {
    let mut p = Vec::new();
    let mut t = Vec::new();
    for i in 0..n {
        let c = rng.point(10.0);
        for _ in 0..3 {
            p.push(c + rng.point(1.5));
        }
        let b = 3 * i as u32;
        t.push([b, b + 1, b + 2]);
    }
    IndexedMesh::new(p, t).expect("soup")
}

fn v(p: Point3) -> Vector {
    Vector::new(p.x, p.y, p.z)
}

fn cross_check(m: &IndexedMesh, seed: u64, n: usize) {
    let ours = MeshSpatial::build(
        Arc::new(m.clone()),
        &BvhOpts::default(),
        &JobCtx::detached(),
    )
    .expect("build");
    let theirs = TriMesh::new(m.positions.iter().map(|&p| v(p)).collect(), m.tris.clone())
        .expect("parry mesh");
    let mut rng = Rng(seed);
    let (mut hit_mismatch, mut hits) = (0, 0);
    for _ in 0..n {
        // closest point
        let p = rng.point(14.0);
        let a = ours.closest_point(p, f64::INFINITY).expect("cp");
        let b = theirs.project_local_point(v(p), false);
        let d = (b.point - v(p)).length();
        assert!(
            (a.dist_sq.sqrt() - d).abs() < 1e-9 * (1.0 + d),
            "closest point {} vs parry {d}",
            a.dist_sq.sqrt()
        );
        // ray cast
        let dir = loop {
            let d = rng.point(1.0);
            if d.length() > 0.1 && d.length() < 1.0 {
                break d.normalize();
            }
        };
        let o = rng.point(14.0);
        let mine = ours.ray_cast(
            &Ray3 {
                origin: o,
                dir: Dir3::new_unchecked(dir),
            },
            1e9,
        );
        let their = theirs.cast_local_ray(&Ray::new(v(o), v(dir)), 1e9, false);
        match (mine, their) {
            (None, None) => {}
            (Some(h), Some(t)) => {
                hits += 1;
                assert!((h.t - t).abs() < 1e-8 * (1.0 + t), "t {} vs parry {t}", h.t);
            }
            _ => hit_mismatch += 1,
        }
    }
    // Only edge-grazing rays may disagree on hit/miss.
    assert!(
        hit_mismatch * 1000 <= n,
        "{hit_mismatch} hit/miss mismatches of {n}"
    );
    assert!(hits > 0);
}

#[test]
fn matches_parry_on_soup() {
    let mut rng = Rng(11);
    let m = random_soup(&mut rng, 800);
    cross_check(&m, 12, 10_000);
}

#[test]
fn matches_parry_on_torus() {
    cross_check(&common::torus(40, 24, 6.0, 2.0), 13, 10_000);
}
