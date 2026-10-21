#![allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]

use std::collections::HashMap;

use amg_core::{Point3, Vec3};
use amg_jobs::{CancelToken, JobCtx};
use amg_testkit::{TriSoup, assert_golden, shapes};

use super::*;
use crate::{FaceGroupInfo, GroupOrigin, WeldOpts};

/// SplitMix64 + Box-Muller, deterministic.
struct Rng(u64);

impl Rng {
    fn f64(&mut self) -> f64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        ((z ^ (z >> 31)) >> 11) as f64 / (1u64 << 53) as f64
    }
    fn gauss(&mut self) -> f64 {
        let (u, v) = (self.f64().max(1e-300), self.f64());
        (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()
    }
}

fn add_noise(m: &mut IndexedMesh, sigma: f64, seed: u64) {
    let mut r = Rng(seed);
    for p in &mut m.positions {
        *p += sigma * Vec3::new(r.gauss(), r.gauss(), r.gauss());
    }
}

fn icosphere(subdiv: u32, radius: f64) -> IndexedMesh {
    let s = shapes::icosphere(Point3::ZERO, radius, subdiv);
    IndexedMesh::new(s.positions, s.triangles).unwrap()
}

/// Cube surface with `n × n` quads (2 triangles each) per side, welded; edge length `size`.
fn cube_grid(n: u32, size: f64) -> IndexedMesh {
    let h = size / f64::from(n);
    let mut tris: Vec<[Point3; 3]> = Vec::new();
    for axis in 0..3 {
        for side in 0..2 {
            let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
            let pt = |i: u32, j: u32| {
                let mut c = [0.0; 3];
                c[axis] = f64::from(side * n) * h;
                c[u] = f64::from(i) * h;
                c[v] = f64::from(j) * h;
                Point3::new(c[0], c[1], c[2])
            };
            for i in 0..n {
                for j in 0..n {
                    let (a, b, c, d) = (pt(i, j), pt(i + 1, j), pt(i + 1, j + 1), pt(i, j + 1));
                    if side == 1 {
                        tris.push([a, b, c]);
                        tris.push([a, c, d]);
                    } else {
                        tris.push([a, c, b]);
                        tris.push([a, d, c]);
                    }
                }
            }
        }
    }
    IndexedMesh::from_soup(&tris, WeldOpts::EXACT).unwrap()
}

fn ctx() -> JobCtx {
    JobCtx::detached()
}

fn radial_rms(m: &IndexedMesh, r: f64) -> f64 {
    let s: f64 = m.positions.iter().map(|p| (p.length() - r).powi(2)).sum();
    (s / m.positions.len() as f64).sqrt()
}

/// Distance of `p` to the surface of the box `[0, size]³`.
fn box_surface_dist(p: Point3, size: f64) -> f64 {
    let c = Point3::splat(size / 2.0);
    let q = (p - c).abs() - Vec3::splat(size / 2.0);
    (q.max(Vec3::ZERO).length() + q.max_element().min(0.0)).abs()
}

fn cube_rms(m: &IndexedMesh, size: f64) -> f64 {
    let s: f64 = m
        .positions
        .iter()
        .map(|&p| box_surface_dist(p, size).powi(2))
        .sum();
    (s / m.positions.len() as f64).sqrt()
}

#[test]
fn taubin_sphere_keeps_volume_and_cuts_noise() {
    let r = 10.0;
    let mut m = icosphere(5, r);
    add_noise(&mut m, 0.01 * r, 7);
    let before_rms = radial_rms(&m, r);
    let v0 = m.signed_volume();
    let opts = SmoothOpts::new(SmoothMethod::TAUBIN_DEFAULT, 50);
    let s = smooth(&m, &opts, &ctx()).unwrap();
    let after_rms = radial_rms(&s, r);
    let dv = (s.signed_volume() - v0) / v0;
    assert!(dv.abs() < 0.01, "volume change {dv}");
    assert!(
        before_rms / after_rms >= 3.0,
        "noise {before_rms} -> {after_rms}"
    );
    assert_eq!(s.tris, m.tris);
    s.validate().unwrap();
}

#[test]
fn laplacian_shrinks_taubin_does_not() {
    let r = 10.0;
    let m = icosphere(3, r);
    let v0 = m.signed_volume();
    let lap = SmoothOpts::new(
        SmoothMethod::Laplacian {
            lambda: 0.5,
            weights: LaplacianWeights::Uniform,
        },
        30,
    );
    let shrunk = smooth(&m, &lap, &ctx()).unwrap();
    assert!(shrunk.signed_volume() < 0.8 * v0);
}

#[test]
fn cotangent_is_less_tangential_than_uniform() {
    // Move one vertex of a sphere tangentially: uniform Laplacian drags the mesh sideways,
    // cotangent smoothing of an already smooth surface should barely change the area.
    let r = 10.0;
    let m = icosphere(3, r);
    let cot = SmoothOpts::new(
        SmoothMethod::Laplacian {
            lambda: 0.5,
            weights: LaplacianWeights::Cotangent,
        },
        1,
    );
    let s = smooth(&m, &cot, &ctx()).unwrap();
    // Every vertex moves inward only (normal direction) on a sphere.
    for (a, b) in m.positions.iter().zip(&s.positions) {
        let d = *b - *a;
        let n = a.normalize();
        assert!(d.length() < 0.2);
        assert!(
            d.cross(n).length() < 0.05 * d.length().max(1e-9) + 0.05,
            "tangential drift"
        );
    }
}

#[test]
fn bilateral_cube_keeps_edges_and_cuts_noise() {
    let size = 10.0;
    let clean = cube_grid(20, size);
    let mut m = clean.clone();
    add_noise(&mut m, 0.05, 11);
    let before = cube_rms(&m, size);
    let opts = SmoothOpts {
        fix_boundary: false,
        ..SmoothOpts::new(
            SmoothMethod::BilateralNormal {
                sigma_normal: 0.35,
                normal_iterations: 5,
                vertex_iterations: 10,
            },
            3,
        )
    };
    let s = smooth(&m, &opts, &ctx()).unwrap();
    let after = cube_rms(&s, size);
    assert!(before / after >= 3.0, "noise {before} -> {after}");

    // Dihedral angle over edges lying on the 12 cube edges.
    let mut edges: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for (f, t) in s.tris.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            edges.entry((a.min(b), a.max(b))).or_default().push(f);
        }
    }
    let on_boundary = |p: Point3| {
        (0..3)
            .filter(|&i| p[i].abs() < 1e-9 || (p[i] - size).abs() < 1e-9)
            .count()
            >= 2
    };
    let (mut sum, mut cnt) = (0.0, 0);
    for (&(a, b), fs) in &edges {
        if fs.len() == 2
            && on_boundary(clean.positions[a as usize])
            && on_boundary(clean.positions[b as usize])
        {
            let n0 = s.face_normal(FaceId(fs[0] as u32)).normalize();
            let n1 = s.face_normal(FaceId(fs[1] as u32)).normalize();
            let on_same_cube_edge = clean
                .face_normal(FaceId(fs[0] as u32))
                .dot(clean.face_normal(FaceId(fs[1] as u32)))
                .abs()
                < 1e-9;
            if on_same_cube_edge {
                let d = (std::f64::consts::PI - n0.dot(n1).clamp(-1.0, 1.0).acos()).to_degrees();
                assert!((d - 90.0).abs() < 5.0, "feature dihedral {d}");
                sum += d;
                cnt += 1;
            }
        }
    }
    assert!(cnt > 100);
    let mean = sum / cnt as f64;
    assert!((mean - 90.0).abs() < 5.0, "mean feature dihedral {mean}");
}

#[test]
fn fix_boundary_selection_and_groups() {
    // Open grid: boundary stays put.
    let tris: Vec<[Point3; 3]> = {
        let mut v = Vec::new();
        let p = |i: u32, j: u32| {
            Point3::new(
                f64::from(i),
                f64::from(j),
                if (i, j) == (2, 2) { 1.0 } else { 0.0 },
            )
        };
        for i in 0..4 {
            for j in 0..4 {
                v.push([p(i, j), p(i + 1, j), p(i + 1, j + 1)]);
                v.push([p(i, j), p(i + 1, j + 1), p(i, j + 1)]);
            }
        }
        v
    };
    let m = IndexedMesh::from_soup(&tris, WeldOpts::EXACT).unwrap();
    let lap = SmoothMethod::Laplacian {
        lambda: 1.0,
        weights: LaplacianWeights::Uniform,
    };
    let s = smooth(&m, &SmoothOpts::new(lap, 3), &ctx()).unwrap();
    let mut moved = 0;
    for (v, (a, b)) in m.positions.iter().zip(&s.positions).enumerate() {
        let edge = a.x == 0.0 || a.x == 4.0 || a.y == 0.0 || a.y == 4.0;
        if edge {
            assert_eq!(a, b, "boundary vertex {v} moved");
        } else if a != b {
            moved += 1;
        }
    }
    assert!(moved > 0);

    // Selection: only vertices of selected faces move.
    let sel = SmoothOpts {
        selection: Some(vec![FaceId(0)]),
        fix_boundary: false,
        ..SmoothOpts::new(lap, 1)
    };
    let s = smooth(&m, &sel, &ctx()).unwrap();
    let touched: Vec<u32> = m.tris[0].to_vec();
    for (v, (a, b)) in m.positions.iter().zip(&s.positions).enumerate() {
        if !touched.contains(&(v as u32)) {
            assert_eq!(a, b);
        }
    }

    // Group borders: split the grid into two groups; vertices on the seam stay.
    let mut g = m.clone();
    let id = g
        .groups
        .alloc(FaceGroupInfo::new(GroupOrigin::Segmentation));
    for f in 0..g.tris.len() {
        let c: f64 = g.tris[f]
            .iter()
            .map(|&v| g.positions[v as usize].x)
            .sum::<f64>()
            / 3.0;
        if c > 2.0 {
            g.groups.set(FaceId(f as u32), id);
        }
    }
    let o = SmoothOpts {
        preserve_group_borders: true,
        fix_boundary: false,
        ..SmoothOpts::new(lap, 2)
    };
    let s = smooth(&g, &o, &ctx()).unwrap();
    for (a, b) in g.positions.iter().zip(&s.positions) {
        if a.x == 2.0 {
            assert_eq!(a, b);
        }
    }
}

#[test]
fn rejects_bad_options_and_cancels() {
    let m = icosphere(1, 1.0);
    for method in [
        SmoothMethod::Laplacian {
            lambda: 0.0,
            weights: LaplacianWeights::Uniform,
        },
        SmoothMethod::Taubin {
            lambda: 0.5,
            mu: -0.4,
        },
        SmoothMethod::BilateralNormal {
            sigma_normal: -1.0,
            normal_iterations: 1,
            vertex_iterations: 1,
        },
    ] {
        let e = smooth(&m, &SmoothOpts::new(method, 1), &ctx()).unwrap_err();
        assert!(matches!(e, MeshError::InvalidOption { .. }));
    }
    let bad = SmoothOpts {
        selection: Some(vec![FaceId(9999)]),
        ..SmoothOpts::new(SmoothMethod::TAUBIN_DEFAULT, 1)
    };
    assert!(matches!(
        smooth(&m, &bad, &ctx()),
        Err(MeshError::IndexOutOfRange { .. })
    ));
    let tok = CancelToken::new();
    tok.cancel();
    let c = JobCtx::with_token(tok);
    let e = smooth(&m, &SmoothOpts::new(SmoothMethod::TAUBIN_DEFAULT, 5), &c).unwrap_err();
    assert!(matches!(e, MeshError::Cancelled));
}

#[test]
fn deterministic_across_runs() {
    let mut m = icosphere(3, 5.0);
    add_noise(&mut m, 0.05, 3);
    let o = SmoothOpts::new(SmoothMethod::TAUBIN_DEFAULT, 5);
    assert_eq!(
        smooth(&m, &o, &ctx()).unwrap(),
        smooth(&m, &o, &ctx()).unwrap()
    );
}

fn soup_of(m: &IndexedMesh) -> TriSoup<'_> {
    TriSoup {
        positions: &m.positions,
        triangles: &m.tris,
    }
}

#[test]
fn golden_smooth_taubin_sphere() {
    let mut m = icosphere(3, 10.0);
    add_noise(&mut m, 0.1, 21);
    let s = smooth(
        &m,
        &SmoothOpts::new(SmoothMethod::TAUBIN_DEFAULT, 20),
        &ctx(),
    )
    .unwrap();
    assert_golden!("mesh", "smooth_taubin_sphere", soup_of(&s));
}

#[test]
fn golden_smooth_bilateral_cube() {
    let mut m = cube_grid(10, 10.0);
    add_noise(&mut m, 0.05, 5);
    let o = SmoothOpts {
        fix_boundary: false,
        ..SmoothOpts::new(
            SmoothMethod::BilateralNormal {
                sigma_normal: 0.35,
                normal_iterations: 5,
                vertex_iterations: 10,
            },
            2,
        )
    };
    let s = smooth(&m, &o, &ctx()).unwrap();
    assert_golden!("mesh", "smooth_bilateral_cube", soup_of(&s));
}
