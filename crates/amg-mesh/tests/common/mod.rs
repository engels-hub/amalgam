//! Mesh generators shared by the integration tests and benches.
#![allow(dead_code, clippy::expect_used)] // also compiled into the benches

use amg_core::{Point3, Vec3};
use amg_mesh::{FaceId, IndexedMesh};

/// Closed torus with `nu × nv` quads split into `2 nu nv` outward triangles.
pub fn torus(nu: u32, nv: u32, major: f64, minor: f64) -> IndexedMesh {
    let mut p = Vec::with_capacity((nu * nv) as usize);
    for i in 0..nu {
        let u = f64::from(i) / f64::from(nu) * std::f64::consts::TAU;
        for j in 0..nv {
            let v = f64::from(j) / f64::from(nv) * std::f64::consts::TAU;
            let r = major + minor * v.cos();
            p.push(Point3::new(r * u.cos(), r * u.sin(), minor * v.sin()));
        }
    }
    let id = |i: u32, j: u32| (i % nu) * nv + (j % nv);
    let mut t = Vec::with_capacity(2 * (nu * nv) as usize);
    for i in 0..nu {
        for j in 0..nv {
            let (a, b, c, d) = (id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1));
            t.push([a, b, c]);
            t.push([a, c, d]);
        }
    }
    IndexedMesh::new(p, t).expect("valid torus")
}

/// Open `w × h` grid in the XY plane (unit cells); `diag(i)` picks the diagonal of cell `i`.
pub fn grid(w: u32, h: u32, diag: impl Fn(usize) -> bool) -> IndexedMesh {
    let mut p = Vec::new();
    for y in 0..=h {
        for x in 0..=w {
            p.push(Point3::new(f64::from(x), f64::from(y), 0.0));
        }
    }
    let id = |x: u32, y: u32| y * (w + 1) + x;
    let mut t = Vec::new();
    for y in 0..h {
        for x in 0..w {
            let (a, b, c, d) = (id(x, y), id(x + 1, y), id(x + 1, y + 1), id(x, y + 1));
            if diag((y * w + x) as usize) {
                t.push([a, b, c]);
                t.push([a, c, d]);
            } else {
                t.push([a, b, d]);
                t.push([b, c, d]);
            }
        }
    }
    IndexedMesh::new(p, t).expect("valid grid")
}

/// The mesh as a triangle soup.
pub fn soup(m: &IndexedMesh) -> Vec<[Point3; 3]> {
    (0..m.face_count() as u32)
        .map(|f| m.triangle(FaceId(f)))
        .collect()
}

/// SplitMix64.
pub struct Rng(pub u64);

impl Rng {
    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }
    /// Uniform in [0, 1).
    pub fn f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    pub fn below(&mut self, n: u64) -> u64 {
        self.next_u64() % n
    }
    pub fn vec3(&mut self) -> Vec3 {
        Vec3::new(self.f64() - 0.5, self.f64() - 0.5, self.f64() - 0.5)
    }
}
