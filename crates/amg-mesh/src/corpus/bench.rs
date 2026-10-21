//! Benchmark meshes: displaced geodesic (subdivided icosahedron) spheres at 100 k / 1 M / 10 M
//! triangles, and the offset sphere pair for the 1 M x 1 M boolean scenario.
//!
//! A frequency-`k` geodesic sphere has `20 k^2` triangles and `10 k^2 + 2` vertices, so the three
//! sizes are `k = 71, 224, 707` (100 820 / 1 003 520 / 9 996 980 triangles). Output is a pure
//! function of `(size, seed)`: vertices are computed independently (rayon only splits the work),
//! so it is bit-identical for any thread count on one platform and toolchain. (`sin`/`cos` come
//! from the platform libm; across platforms the topology is identical and positions may differ
//! by one ulp.)

use amg_core::{Point3, Vec3};
use rand_chacha::ChaCha8Rng;
use rand_chacha::rand_core::{RngCore, SeedableRng};
use rayon::prelude::*;

use crate::IndexedMesh;

/// Benchmark mesh size (S07 §Performance targets).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum BenchSize {
    /// 100 820 triangles.
    K100,
    /// 1 003 520 triangles.
    M1,
    /// 9 996 980 triangles.
    M10,
}

impl BenchSize {
    /// Geodesic frequency.
    pub const fn frequency(self) -> u32 {
        match self {
            BenchSize::K100 => 71,
            BenchSize::M1 => 224,
            BenchSize::M10 => 707,
        }
    }

    /// Triangle count (`20 k^2`).
    pub const fn triangles(self) -> usize {
        let k = self.frequency() as usize;
        20 * k * k
    }
}

/// Radial displacement: a sum of six seeded plane waves, `r = R (1 + sum_n w_n sin(f_n d.u_n + p_n))`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Displacement {
    /// Peak relative radius change (sum of the six weights), e.g. `0.06`.
    pub amplitude: f64,
    /// Seed for directions, frequencies and phases.
    pub seed: u64,
}

struct Waves {
    dir: [Vec3; 6],
    freq: [f64; 6],
    phase: [f64; 6],
    weight: f64,
}

fn unit_f64(r: &mut ChaCha8Rng) -> f64 {
    (r.next_u64() >> 11) as f64 / (1u64 << 53) as f64
}

impl Waves {
    fn new(d: Displacement) -> Waves {
        let mut r = ChaCha8Rng::seed_from_u64(d.seed);
        let mut w = Waves {
            dir: [Vec3::X; 6],
            freq: [0.0; 6],
            phase: [0.0; 6],
            weight: d.amplitude / 6.0,
        };
        for n in 0..6 {
            let z = 2.0 * unit_f64(&mut r) - 1.0;
            let t = std::f64::consts::TAU * unit_f64(&mut r);
            let s = (1.0 - z * z).sqrt();
            w.dir[n] = Vec3::new(s * t.cos(), s * t.sin(), z);
            w.freq[n] = 2.0 + 6.0 * unit_f64(&mut r);
            w.phase[n] = std::f64::consts::TAU * unit_f64(&mut r);
        }
        w
    }

    fn scale(&self, d: Vec3) -> f64 {
        let mut s = 0.0;
        for n in 0..6 {
            s += (self.freq[n] * d.dot(self.dir[n]) + self.phase[n]).sin();
        }
        1.0 + self.weight * s
    }
}

const ICO_FACES: [[usize; 3]; 20] = [
    [0, 11, 5],
    [0, 5, 1],
    [0, 1, 7],
    [0, 7, 10],
    [0, 10, 11],
    [1, 5, 9],
    [5, 11, 4],
    [11, 10, 2],
    [10, 7, 6],
    [7, 1, 8],
    [3, 9, 4],
    [3, 4, 2],
    [3, 2, 6],
    [3, 6, 8],
    [3, 8, 9],
    [4, 9, 5],
    [2, 4, 11],
    [6, 2, 10],
    [8, 6, 7],
    [9, 8, 1],
];

fn ico_vertices() -> [Vec3; 12] {
    let t = (1.0 + 5.0f64.sqrt()) / 2.0;
    [
        Vec3::new(-1.0, t, 0.0),
        Vec3::new(1.0, t, 0.0),
        Vec3::new(-1.0, -t, 0.0),
        Vec3::new(1.0, -t, 0.0),
        Vec3::new(0.0, -1.0, t),
        Vec3::new(0.0, 1.0, t),
        Vec3::new(0.0, -1.0, -t),
        Vec3::new(0.0, 1.0, -t),
        Vec3::new(t, 0.0, -1.0),
        Vec3::new(t, 0.0, 1.0),
        Vec3::new(-t, 0.0, -1.0),
        Vec3::new(-t, 0.0, 1.0),
    ]
    .map(Vec3::normalize)
}

/// Subdivided icosahedron projected onto a sphere of `radius`, optionally displaced. Frequency
/// `k >= 1`; outward counter-clockwise triangles, shared vertices, closed and manifold.
///
/// # Panics
/// If `k == 0`, or the mesh would not fit `u32` indices (`k > 1000`).
pub fn geodesic_sphere(k: u32, radius: f64, displacement: Option<Displacement>) -> IndexedMesh {
    assert!((1..=1000).contains(&k), "geodesic frequency out of range");
    let ico = ico_vertices();
    let ku = k as usize;
    // 30 edges: (u < v) -> index.
    let mut edge_index = [[usize::MAX; 12]; 12];
    let mut edge_ends: Vec<(usize, usize)> = Vec::new();
    for f in ICO_FACES {
        for e in 0..3 {
            let (a, b) = (f[e], f[(e + 1) % 3]);
            let (u, v) = (a.min(b), a.max(b));
            if edge_index[u][v] == usize::MAX {
                edge_index[u][v] = edge_ends.len();
                edge_ends.push((u, v));
            }
        }
    }
    let per_edge = ku - 1;
    let per_face = if ku >= 2 { (ku - 1) * (ku - 2) / 2 } else { 0 };
    let n_vert = 12 + 30 * per_edge + 20 * per_face;
    let edge_base = 12;
    let face_base = 12 + 30 * per_edge;

    let waves = displacement.map(Waves::new);
    let place = |d: Vec3| -> Point3 {
        let d = d.normalize();
        let r = radius * waves.as_ref().map_or(1.0, |w| w.scale(d));
        d * r
    };

    let mut positions = vec![Point3::ZERO; n_vert];
    {
        let (corners, rest) = positions.split_at_mut(12);
        for (c, v) in corners.iter_mut().zip(ico) {
            *c = place(v);
        }
        let (edges, interior) = rest.split_at_mut(30 * per_edge);
        if per_edge > 0 {
            edges
                .par_chunks_mut(per_edge)
                .enumerate()
                .for_each(|(e, chunk)| {
                    let (u, v) = edge_ends[e];
                    for (mi, p) in chunk.iter_mut().enumerate() {
                        let m = (mi + 1) as f64;
                        *p = place(ico[u] * (ku as f64 - m) + ico[v] * m);
                    }
                });
        }
        if per_face > 0 {
            interior
                .par_chunks_mut(per_face)
                .enumerate()
                .for_each(|(f, chunk)| {
                    let [a, b, c] = ICO_FACES[f].map(|i| ico[i]);
                    let mut o = 0;
                    for i in 1..ku - 1 {
                        for j in 1..ku - i {
                            let w = (ku - i - j) as f64;
                            chunk[o] = place(a * w + b * i as f64 + c * j as f64);
                            o += 1;
                        }
                    }
                });
        }
    }

    // Global id of lattice point (i, j) of face f.
    let id = |f: usize, i: usize, j: usize| -> u32 {
        let [a, b, c] = ICO_FACES[f];
        let edge_pt = |p: usize, q: usize, m: usize| -> usize {
            // point at m steps from p towards q
            let (u, v) = (p.min(q), p.max(q));
            let e = edge_index[u][v];
            let mm = if p == u { m } else { ku - m };
            edge_base + e * per_edge + mm - 1
        };
        let g = if i == 0 && j == 0 {
            a
        } else if i == ku {
            b
        } else if j == ku {
            c
        } else if j == 0 {
            edge_pt(a, b, i)
        } else if i + j == ku {
            edge_pt(b, c, j)
        } else if i == 0 {
            edge_pt(c, a, ku - j)
        } else {
            let before = (i - 1) * (ku - 1) - (i - 1) * i / 2;
            face_base + f * per_face + before + (j - 1)
        };
        g as u32
    };

    let mut tris = vec![[0u32; 3]; 20 * ku * ku];
    tris.par_chunks_mut(ku * ku)
        .enumerate()
        .for_each(|(f, chunk)| {
            let mut o = 0;
            for i in 0..ku {
                for j in 0..ku - i {
                    chunk[o] = [id(f, i, j), id(f, i + 1, j), id(f, i, j + 1)];
                    o += 1;
                    if i + j + 2 <= ku {
                        chunk[o] = [id(f, i + 1, j), id(f, i + 1, j + 1), id(f, i, j + 1)];
                        o += 1;
                    }
                }
            }
        });
    IndexedMesh {
        groups: crate::FaceGroups::new(tris.len()),
        positions,
        tris,
        attrs: crate::MeshAttrs::default(),
    }
}

/// Displaced geodesic sphere of radius 10 mm with 6 % relief, the S07 benchmark input.
pub fn benchmark_sphere(size: BenchSize, seed: u64) -> IndexedMesh {
    geodesic_sphere(
        size.frequency(),
        10.0,
        Some(Displacement {
            amplitude: 0.06,
            seed,
        }),
    )
}

/// Two overlapping benchmark spheres (radius 10 mm, centres 7 mm apart along x) with
/// independent relief, for the `size` x `size` boolean scenario of the vision doc.
pub fn boolean_pair(size: BenchSize, seed: u64) -> (IndexedMesh, IndexedMesh) {
    let a = benchmark_sphere(size, seed);
    let mut b = benchmark_sphere(size, seed.wrapping_add(1));
    let shift = Vec3::new(7.0, 0.0, 0.0);
    b.positions.par_iter_mut().for_each(|p| *p += shift);
    (a, b)
}
