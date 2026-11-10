//! IMP-03: `MeshSdf` against brute force, its Lipschitz and `may_cross` properties, thread
//! independence, face groups and the 2 M-query budget.

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    reason = "test code"
)]

mod common;

use std::sync::{Arc, LazyLock};
use std::time::Instant;

use amg_core::{Aabb3, Point3, Vec3};
use amg_implicit::MeshSdf;
use amg_jobs::JobCtx;
use amg_mesh::corpus::synth::{Shape, SynthSpec, generate};
use amg_mesh::corpus::{FileStatus, Manifest, corpus_root, quick_stats, read_mesh};
use amg_mesh::iso::{IsoOpts, ScalarField, extract_isosurface};
use amg_mesh::{FaceGroupInfo, FaceId, GroupOrigin, IndexedMesh};
use common::{Rng, mean_edge, near_surface, spatial};
use proptest::prelude::*;
use rayon::prelude::*;

fn require_testdata() -> bool {
    std::env::var("AMG_REQUIRE_TESTDATA").is_ok_and(|v| v == "1")
}

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}

fn synth(shape: Shape) -> IndexedMesh {
    generate(&SynthSpec {
        shape,
        defects: vec![],
        seed: 1,
    })
    .mesh
}

/// Closed, outward test meshes of different character (convex, genus 1, knotted and thin,
/// genus 16 with flat faces and sharp edges).
fn closed_meshes() -> Vec<(&'static str, IndexedMesh)> {
    vec![
        (
            "icosphere",
            synth(Shape::Icosphere {
                frequency: 16,
                radius: 10.0,
            }),
        ),
        (
            "displaced_sphere",
            synth(Shape::DisplacedSphere {
                frequency: 16,
                radius: 10.0,
                amplitude: 0.06,
            }),
        ),
        (
            "torus",
            synth(Shape::Torus {
                major_segments: 40,
                minor_segments: 20,
                major: 10.0,
                minor: 3.0,
            }),
        ),
        (
            "torus_knot",
            synth(Shape::TorusKnot {
                segments: 240,
                sides: 12,
            }),
        ),
        (
            "box_with_holes",
            synth(Shape::BoxWithHoles {
                cells: 16,
                holes_per_side: 4,
                size: 40.0,
            }),
        ),
    ]
}

/// Closest squared distance to a triangle by plane projection plus edges (independent of
/// the MESH-03 kernel). A plane-projection inside test is not used: on near-degenerate
/// slivers the noisy normal lets a projected point far off the triangle pass it.
fn brute_closest_tri(p: Point3, [a, b, c]: [Point3; 3]) -> f64 {
    let seg = |s: Point3, e: Point3| {
        let d = e - s;
        let l2 = d.length_squared();
        let t = if l2 > 0.0 {
            ((p - s).dot(d) / l2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        (p - (s + d * t)).length_squared()
    };
    let mut best = seg(a, b).min(seg(b, c)).min(seg(c, a));
    // Interior candidate: project onto the plane whose normal comes from the two shortest
    // edges (the corner opposite the longest edge). That cross product is well conditioned
    // even on slivers, unlike one built from a long edge.
    let (la, lb, lc) = (
        (c - b).length_squared(),
        (a - c).length_squared(),
        (b - a).length_squared(),
    );
    let o = if la >= lb && la >= lc {
        a
    } else if lb >= lc {
        b
    } else {
        c
    };
    let [x, y] = if o == a {
        [b, c]
    } else if o == b {
        [c, a]
    } else {
        [a, b]
    };
    let n = (x - o).cross(y - o);
    let nl = n.length();
    if nl > 0.0 {
        let nh = n / nl;
        let q = p - nh * (p - o).dot(nh);
        let inside = (b - a).cross(q - a).dot(nh) >= 0.0
            && (c - b).cross(q - b).dot(nh) >= 0.0
            && (a - c).cross(q - c).dot(nh) >= 0.0;
        if inside {
            best = best.min((p - q).length_squared());
        }
    }
    best
}

fn brute_distance(m: &IndexedMesh, p: Point3) -> f64 {
    (0..m.face_count() as u32)
        .map(|f| brute_closest_tri(p, m.triangle(FaceId(f))))
        .fold(f64::INFINITY, f64::min)
        .sqrt()
}

#[derive(Debug, Default)]
struct BruteReport {
    points: usize,
    sign_checked: usize,
    worst_abs_err: f64,
}

/// Done-when (a): on `n` random points within the band, `|signed|` equals the brute-force
/// closest distance within 1e-9 mm, and the sign equals the exact winding number's for
/// points at least one mean edge length off the surface.
fn check_brute(name: &str, m: &IndexedMesh, band: f64, n: usize, seed: u64) -> BruteReport {
    let s = spatial(m.clone());
    let sdf = MeshSdf::new(s.clone(), band);
    let edge = mean_edge(m);
    let mut rng = Rng(seed);
    let pts: Vec<Point3> = (0..n).map(|_| near_surface(m, &mut rng, band)).collect();
    let rows: Vec<(f64, bool)> = pts
        .par_iter()
        .map(|&p| {
            let want = brute_distance(m, p).min(band);
            let got = sdf.signed(p);
            let err = (got.abs() - want).abs();
            assert!(
                err <= 1e-9,
                "{name}: |signed| {} vs brute {want} at {p:?}",
                got.abs()
            );
            assert_eq!(sdf.unsigned(p), got.abs(), "{name}: unsigned != |signed|");
            let checked = want >= edge && {
                let inside = s.winding_number_exact(p) > 0.5;
                assert_eq!(
                    got < 0.0,
                    inside,
                    "{name}: sign at {p:?} (d = {want}, mean edge {edge})"
                );
                true
            };
            (err, checked)
        })
        .collect();
    BruteReport {
        points: n,
        sign_checked: rows.iter().filter(|r| r.1).count(),
        worst_abs_err: rows.iter().map(|r| r.0).fold(0.0, f64::max),
    }
}

#[test]
fn matches_brute_force_on_synthetic_closed_meshes() {
    for (i, (name, m)) in closed_meshes().into_iter().enumerate() {
        let b = m.aabb();
        let band = 0.1 * (b.max - b.min).length();
        let r = check_brute(name, &m, band, 10_000, 100 + i as u64);
        eprintln!("{name}: {r:?} (band {band:.3})");
        assert!(
            r.sign_checked > r.points / 10,
            "{name}: too few sign checks"
        );
    }
}

/// Committed corpus models above this size are skipped by the brute-force test (10 k points
/// × faces gets slow in the dev profile); the count is printed.
const CORPUS_MAX_FACES: usize = 120_000;

#[test]
fn matches_brute_force_on_corpus_watertight_models() {
    let manifest = Manifest::load().expect("corpus.toml");
    let root = corpus_root();
    let (mut checked, mut lfs, mut big, mut open, mut sign_checks) = (0, 0, 0, 0, 0usize);
    let mut worst = 0.0f64;
    for (i, e) in manifest.meshes.iter().enumerate() {
        let path = match e.locate(&root) {
            FileStatus::Ready(p) => p,
            FileStatus::LfsPointer(p) => {
                assert!(
                    !require_testdata(),
                    "{} is an un-fetched git-LFS pointer and AMG_REQUIRE_TESTDATA=1",
                    p.display()
                );
                lfs += 1;
                continue;
            }
            FileStatus::NotFetched(_) | FileStatus::Missing(_) => continue,
        };
        if e.faces > CORPUS_MAX_FACES {
            big += 1;
            continue;
        }
        let m = read_mesh(&path).expect("read");
        let q = quick_stats(&m);
        if q.boundary_edges != 0 || q.non_manifold_edges != 0 || q.inconsistent_edges != 0 {
            open += 1;
            continue;
        }
        let b = m.aabb();
        let band = 0.05 * (b.max - b.min).length();
        let r = check_brute(&e.id, &m, band, 10_000, 1_000 + i as u64);
        assert!(r.worst_abs_err <= 1e-9);
        worst = worst.max(r.worst_abs_err);
        sign_checks += r.sign_checked;
        checked += 1;
    }
    eprintln!(
        "corpus: {checked} watertight models checked ({sign_checks} sign checks, worst \
         |d| error {worst:e} mm), {open} not watertight, {big} over {CORPUS_MAX_FACES} faces \
         skipped, {lfs} LFS pointers skipped"
    );
    if checked == 0 {
        assert!(!require_testdata(), "no corpus model was available");
        eprintln!("SKIP: no corpus file is available (git lfs pull)");
    }
}

/// The two-tier sign (β = 1 estimate, β = 2 when ambiguous) equals the β = 2 sign of MESH-03
/// on closed meshes, for points near and far from the surface.
#[test]
fn two_tier_sign_agrees_with_beta_2() {
    for (i, (name, m)) in closed_meshes().into_iter().enumerate() {
        let b = m.aabb();
        let diag = (b.max - b.min).length();
        let mut pts = common::band_points(&m, 20_000, 0.05 * diag, 300 + i as u64);
        pts.extend(common::box_points(&m, 20_000, 0.2 * diag, 400 + i as u64));
        let s = spatial(m);
        let sdf = MeshSdf::new(s.clone(), 0.05 * diag);
        let bad = pts
            .par_iter()
            .filter(|&&p| sdf.is_inside(p) != s.is_inside(p))
            .count();
        assert_eq!(bad, 0, "{name}: {bad} sign differences");
    }
}

#[test]
fn analytic_sphere_values_and_clamping() {
    let m = synth(Shape::Icosphere {
        frequency: 32,
        radius: 10.0,
    });
    let sdf = MeshSdf::new(spatial(m), 2.0);
    assert!(sdf.is_closed());
    assert_eq!(sdf.band(), 2.0);
    // A fine icosphere is within ~0.02 mm of the analytic sphere.
    for (p, want) in [
        (Point3::new(11.0, 0.0, 0.0), 1.0),
        (Point3::new(0.0, -9.0, 0.0), -1.0),
        (Point3::new(0.0, 0.0, 10.5), 0.5),
    ] {
        assert!(
            (sdf.signed(p) - want).abs() < 0.03,
            "{p:?}: {}",
            sdf.signed(p)
        );
    }
    // Beyond the band: exactly ±band with the right sign.
    assert_eq!(sdf.signed(Point3::new(30.0, 0.0, 0.0)), 2.0);
    assert_eq!(sdf.signed(Point3::ZERO), -2.0);
    assert_eq!(sdf.unsigned(Point3::ZERO), 2.0);
    assert!(sdf.signed(Point3::NAN).is_nan());
    assert!(sdf.unsigned(Point3::NAN).is_nan());
    assert_eq!(sdf.value(Point3::ZERO), -2.0);
    // values() equals value() point by point.
    let pts = common::box_points(sdf.spatial().mesh(), 5_000, 3.0, 3);
    let mut out = vec![0.0; pts.len()];
    sdf.values(&pts, &mut out);
    for (p, v) in pts.iter().zip(&out) {
        assert_eq!(v.to_bits(), sdf.signed(*p).to_bits());
    }
}

#[test]
fn band_edge_cases_and_empty_mesh() {
    let m = synth(Shape::Icosphere {
        frequency: 8,
        radius: 5.0,
    });
    let s = spatial(m);
    // Negative and NaN bands are 0: the field is ±0.
    for band in [-1.0, f64::NAN, -0.0] {
        let sdf = MeshSdf::new(s.clone(), band);
        assert_eq!(sdf.band().to_bits(), 0.0f64.to_bits());
        assert_eq!(sdf.signed(Point3::new(7.0, 0.0, 0.0)), 0.0);
    }
    // An infinite band is the plain signed distance.
    let inf = MeshSdf::new(s.clone(), f64::INFINITY);
    for p in [Point3::new(40.0, 1.0, 2.0), Point3::new(0.5, 0.2, 0.1)] {
        assert_eq!(
            inf.signed(p),
            s.signed_distance(p, f64::INFINITY).expect("d")
        );
    }
    // Far from the surface, the 1-Lipschitz bound still prunes.
    assert!(!inf.may_cross(&Aabb3::new(Point3::splat(30.0), Point3::splat(31.0)), 0.0));
    let across = Aabb3::new(Point3::new(4.5, -0.5, -0.5), Point3::new(5.5, 0.5, 0.5));
    assert!(inf.may_cross(&across, 0.0));
    // Empty mesh: everything is outside, nothing crosses.
    let e = MeshSdf::new(spatial(IndexedMesh::default()), 1.0);
    assert!(e.is_closed());
    assert_eq!(e.signed(Point3::ZERO), 1.0);
    assert_eq!(e.closest_group(Point3::ZERO), None);
    assert!(!e.may_cross(&Aabb3::new(Point3::splat(-1.0), Point3::splat(1.0)), 0.0));
}

/// A unit-ish cube (half size `h`) with one face group per side, in the order
/// -X, +X, -Y, +Y, -Z, +Z.
fn grouped_cube(h: f64) -> IndexedMesh {
    let mut p = Vec::new();
    for i in 0..8u32 {
        p.push(Point3::new(
            if i & 1 == 0 { -h } else { h },
            if i & 2 == 0 { -h } else { h },
            if i & 4 == 0 { -h } else { h },
        ));
    }
    // quads per side, then oriented outward by the normal test
    let quads: [[u32; 4]; 6] = [
        [0, 2, 6, 4],
        [1, 3, 7, 5],
        [0, 1, 5, 4],
        [2, 3, 7, 6],
        [0, 1, 3, 2],
        [4, 5, 7, 6],
    ];
    let mut tris = Vec::new();
    for q in quads {
        for mut t in [[q[0], q[1], q[2]], [q[0], q[2], q[3]]] {
            let [a, b, c] = t.map(|i| p[i as usize]);
            let n = (b - a).cross(c - a);
            if n.dot((a + b + c) / 3.0) < 0.0 {
                t.swap(1, 2);
            }
            tris.push(t);
        }
    }
    let mut m = IndexedMesh::new(p, tris).expect("cube");
    for side in 0..6u32 {
        let g = m.groups.alloc(FaceGroupInfo::new(GroupOrigin::User));
        m.groups.set(FaceId(2 * side), g);
        m.groups.set(FaceId(2 * side + 1), g);
    }
    m
}

#[test]
fn closest_group_follows_the_nearest_face() {
    let m = grouped_cube(5.0);
    let groups: Vec<_> = (0..6u32)
        .map(|s| m.groups.group_of(FaceId(2 * s)))
        .collect();
    let sdf = MeshSdf::new(spatial(m), 1.0);
    assert!(sdf.is_closed());
    let probe = [
        (Point3::new(-5.5, 0.3, -1.0), 0),
        (Point3::new(4.6, 1.0, 2.0), 1),
        (Point3::new(0.2, -5.9, 0.0), 2),
        (Point3::new(2.0, 4.2, -3.0), 3),
        (Point3::new(-1.0, 1.0, -5.01), 4),
        (Point3::new(0.0, 0.0, 5.99), 5),
    ];
    for (p, side) in probe {
        assert_eq!(sdf.closest_group(p), Some(groups[side]), "{p:?}");
        assert!(sdf.signed(p).abs() < 1.0);
    }
    // Beyond the band: None, also deep inside.
    assert_eq!(sdf.closest_group(Point3::new(7.0, 0.0, 0.0)), None);
    assert_eq!(sdf.closest_group(Point3::ZERO), None);
    assert_eq!(sdf.closest_group(Point3::NAN), None);
    // Box corner: exact distance ties between three faces resolve deterministically.
    let corner = Point3::splat(5.5);
    assert_eq!(sdf.closest_group(corner), sdf.closest_group(corner));
    assert!((sdf.signed(corner) - 0.5 * 3f64.sqrt()).abs() < 1e-12);
}

#[test]
fn content_hash_covers_mesh_and_band() {
    let m = grouped_cube(5.0);
    let s = spatial(m.clone());
    let a = MeshSdf::new(s.clone(), 1.0).content_hash();
    assert_eq!(a, MeshSdf::new(spatial(m.clone()), 1.0).content_hash());
    assert_ne!(a, MeshSdf::new(s.clone(), 1.5).content_hash());
    assert_eq!(
        MeshSdf::new(s.clone(), 0.0).content_hash(),
        MeshSdf::new(s, -0.0).content_hash()
    );
    let mut moved = m;
    moved.positions[0].x += 1e-9;
    assert_ne!(a, MeshSdf::new(spatial(moved), 1.0).content_hash());
}

#[test]
fn open_mesh_keeps_the_winding_sign_and_never_prunes() {
    let mut m = synth(Shape::Icosphere {
        frequency: 12,
        radius: 10.0,
    });
    // Punch a hole: drop the faces near the +Z pole.
    let keep: Vec<[u32; 3]> = m
        .tris
        .iter()
        .copied()
        .filter(|t| t.iter().all(|&v| m.positions[v as usize].z < 9.0))
        .collect();
    let dropped = m.tris.len() - keep.len();
    assert!(dropped > 0);
    m = IndexedMesh::new(m.positions, keep).expect("open");
    let sdf = MeshSdf::new(spatial(m), 2.0);
    assert!(!sdf.is_closed());
    // Away from the hole the sign is still right (generalized winding number).
    assert_eq!(sdf.signed(Point3::ZERO), -2.0);
    assert!((sdf.signed(Point3::new(0.0, 0.0, -11.0)) - 1.0).abs() < 1e-9);
    assert!(sdf.signed(Point3::new(0.0, 0.0, -9.0)) < 0.0);
    // The sign can jump off the surface, so nothing is pruned below the band...
    let far = Aabb3::new(Point3::splat(30.0), Point3::splat(31.0));
    assert!(sdf.may_cross(&far, 0.0));
    // ...but levels beyond the band are never reached.
    assert!(!sdf.may_cross(&far, 2.5));
}

#[test]
fn may_cross_prunes_far_boxes() {
    let m = synth(Shape::Icosphere {
        frequency: 16,
        radius: 10.0,
    });
    let sdf = MeshSdf::new(spatial(m), 2.0);
    let cube = |c: Point3, h: f64| Aabb3::new(c - Vec3::splat(h), c + Vec3::splat(h));
    // On the surface: may cross.
    assert!(sdf.may_cross(&cube(Point3::new(10.0, 0.0, 0.0), 0.5), 0.0));
    // Outside the band, inside and outside the solid: cannot cross 0.
    assert!(!sdf.may_cross(&cube(Point3::new(20.0, 0.0, 0.0), 1.0), 0.0));
    assert!(!sdf.may_cross(&cube(Point3::ZERO, 2.0), 0.0));
    // Within the band but the level set of iso = 1.5 is farther than the box reaches.
    assert!(!sdf.may_cross(&cube(Point3::new(10.6, 0.0, 0.0), 0.1), 1.5));
    assert!(sdf.may_cross(&cube(Point3::new(10.6, 0.0, 0.0), 0.1), 0.5));
    // Levels beyond the band are never reached; exactly ±band is reached outside the band.
    assert!(!sdf.may_cross(&cube(Point3::new(20.0, 0.0, 0.0), 1.0), 2.5));
    assert!(sdf.may_cross(&cube(Point3::new(20.0, 0.0, 0.0), 1.0), 2.0));
    assert!(sdf.may_cross(&cube(Point3::new(20.0, 0.0, 0.0), 1.0), f64::NAN));
    // Degenerate boxes are conservative.
    assert!(sdf.may_cross(&Aabb3::new(Point3::splat(30.0), Point3::NAN), 0.0));
}

#[test]
fn isosurface_of_a_mesh_sdf_is_closed_and_matches_the_input() {
    let m = synth(Shape::Torus {
        major_segments: 64,
        minor_segments: 32,
        major: 10.0,
        minor: 3.0,
    });
    let vol_in = m.signed_volume();
    let b = m.aabb();
    let pad = Vec3::splat(2.0);
    let sdf = MeshSdf::new(spatial(m), 1.0);
    let opts = IsoOpts::new(Aabb3::new(b.min - pad, b.max + pad), 0.25);
    let out = extract_isosurface(&sdf, &opts, &JobCtx::detached()).expect("iso");
    let q = quick_stats(&out);
    assert!(out.face_count() > 1000);
    assert_eq!(
        (q.boundary_edges, q.non_manifold_edges, q.inconsistent_edges),
        (0, 0, 0)
    );
    assert_eq!(q.components, 1);
    let rel = (out.signed_volume() - vol_in).abs() / vol_in;
    assert!(rel < 0.02, "volume {} vs {vol_in}", out.signed_volume());
}

#[test]
fn identical_across_thread_counts() {
    // 120 k triangles: above the parallel cut-offs of the BVH and winding-tree builds.
    let m = Arc::new(synth(Shape::Torus {
        major_segments: 300,
        minor_segments: 200,
        major: 20.0,
        minor: 6.0,
    }));
    assert!(m.face_count() >= 100_000);
    let mut pts = common::band_points(&m, 3_000, 2.0, 21);
    pts.extend(common::box_points(&m, 3_000, 4.0, 22));
    let run = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .expect("pool");
        pool.install(|| {
            // Built inside the pool, and the first queries run inside a parallel loop.
            let sdf = MeshSdf::new(spatial((*m).clone()), 2.0);
            let mut vals = vec![0.0; pts.len()];
            sdf.values(&pts, &mut vals);
            let rows: Vec<(u64, u64, Option<u32>, bool)> = pts
                .par_iter()
                .map(|&p| {
                    let c = Aabb3::new(p - Vec3::splat(0.7), p + Vec3::splat(0.7));
                    (
                        sdf.signed(p).to_bits(),
                        sdf.unsigned(p).to_bits(),
                        sdf.closest_group(p).map(|g| g.0),
                        sdf.may_cross(&c, 0.0),
                    )
                })
                .collect();
            let bits: Vec<u64> = vals.iter().map(|v| v.to_bits()).collect();
            (bits, rows, sdf.content_hash())
        })
    };
    let a = run(1);
    assert_eq!(a, run(4));
    assert_eq!(a, run(16));
}

static PROP_MESHES: LazyLock<Vec<(&'static str, MeshSdf)>> = LazyLock::new(|| {
    closed_meshes()
        .into_iter()
        .map(|(n, m)| {
            let b = m.aabb();
            let band = 0.08 * (b.max - b.min).length();
            (n, MeshSdf::new(spatial(m), band))
        })
        .collect()
});

proptest! {
    #![proptest_config(ProptestConfig::with_cases(48))]

    /// Done-when (b): the field is 1-Lipschitz (1e-9 slack) on closed meshes, including pairs
    /// that straddle the surface and the band edge.
    #[test]
    fn prop_field_is_1_lipschitz(which in 0usize..5, seed in any::<u64>()) {
        let (name, sdf) = &PROP_MESHES[which];
        let band = sdf.band();
        let mesh = sdf.spatial().mesh();
        let mut rng = Rng(seed);
        for k in 0..300 {
            let a = near_surface(mesh, &mut rng, 1.5 * band);
            let dir = Vec3::new(rng.range(-1.0, 1.0), rng.range(-1.0, 1.0), rng.range(-1.0, 1.0));
            if dir.length() < 1e-3 {
                continue;
            }
            // log-uniform step from 1e-6 to 2 bands
            let len = band * 2.0 * 10f64.powf(-6.0 * rng.f());
            let b = a + dir.normalize() * len;
            let (fa, fb) = (sdf.signed(a), sdf.signed(b));
            let dist = (a - b).length();
            prop_assert!(
                (fa - fb).abs() <= dist + 1e-9,
                "{name} pair {k}: |{fa} - {fb}| > {dist}"
            );
            prop_assert!((sdf.unsigned(a) - sdf.unsigned(b)).abs() <= dist + 1e-9);
        }
    }

    /// Done-when (b): `may_cross` never returns false for a box where 64³ dense sampling
    /// finds a crossing of `iso`.
    #[test]
    fn prop_may_cross_is_conservative(
        which in 0usize..5,
        seed in any::<u64>(),
        size in 0.01f64..1.5,
        iso_frac in prop_oneof![Just(0.0), -1.0f64..1.0],
    ) {
        let (name, sdf) = &PROP_MESHES[which];
        let band = sdf.band();
        let mesh = sdf.spatial().mesh();
        let mut rng = Rng(seed);
        let c = near_surface(mesh, &mut rng, 2.0 * band);
        let h = size * band;
        let half = Vec3::new(h * rng.range(0.3, 1.0), h * rng.range(0.3, 1.0), h * rng.range(0.3, 1.0));
        let b = Aabb3::new(c - half, c + half);
        let iso = iso_frac * band;
        const N: usize = 64;
        let pts: Vec<Point3> = (0..N * N * N)
            .map(|i| {
                let (x, y, z) = (i % N, (i / N) % N, i / (N * N));
                let t = |k: usize| k as f64 / (N - 1) as f64;
                b.min + (b.max - b.min) * Vec3::new(t(x), t(y), t(z))
            })
            .collect();
        let mut v = vec![0.0; pts.len()];
        sdf.values(&pts, &mut v);
        let below = v.iter().any(|&x| x < iso);
        let above = v.iter().any(|&x| x >= iso);
        if below && above {
            prop_assert!(sdf.may_cross(&b, iso), "{name}: box {b:?} crosses iso {iso}");
        }
    }
}

/// Done-when (c): 2 M signed queries on a 1 M-tri mesh (band 2 mm) < 2 s. Measured always,
/// asserted only under `AMG_STRICT_TIMING=1` (use `--release`); the criterion bench is
/// `benches/mesh_sdf.rs`.
#[test]
fn budget_2m_queries_on_1m_tris() {
    if !strict_timing() {
        eprintln!("SKIP: set AMG_STRICT_TIMING=1 (release build) to run the 1 M-tri budget");
        return;
    }
    let m = common::budget_mesh();
    let pts = common::band_points(&m, 2_000_000, 2.0, 11);
    let sdf = MeshSdf::new(spatial(m), 2.0);
    let mut out = vec![0.0; pts.len()];
    let mut best = f64::INFINITY;
    for _ in 0..3 {
        let t = Instant::now();
        sdf.values(&pts, &mut out);
        best = best.min(t.elapsed().as_secs_f64());
    }
    eprintln!("2 M signed queries on 1 M tris: {best:.3} s");
    assert!(best < 2.0, "{best:.3} s, budget 2 s");
}
