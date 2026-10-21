//! MESH-03: BVH and spatial queries against brute force.

#![allow(clippy::expect_used)] // helpers outside #[test] fns

#[path = "common/mod.rs"]
mod common;

use std::sync::Arc;

use amg_core::{Aabb3, Dir3, Point3, Ray3, Vec3};
use amg_jobs::{CancelToken, JobCtx};
use amg_mesh::{
    Bvh, BvhNodeKind, BvhOpts, FaceId, IndexedMesh, MeshError, MeshSpatial, RayHit, TriFeature,
};
use proptest::prelude::*;

/// Small deterministic generator (SplitMix64).
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
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.f()
    }
    fn point(&mut self, half: f64) -> Point3 {
        Point3::new(
            self.range(-half, half),
            self.range(-half, half),
            self.range(-half, half),
        )
    }
    fn dir(&mut self) -> Dir3 {
        loop {
            let v = self.point(1.0);
            if v.length() > 0.1 && v.length() < 1.0 {
                return Dir3::new_unchecked(v.normalize());
            }
        }
    }
}

fn ctx() -> JobCtx {
    JobCtx::detached()
}

fn spatial(m: IndexedMesh) -> MeshSpatial {
    MeshSpatial::build(Arc::new(m), &BvhOpts::default(), &ctx()).expect("build")
}

/// Random small triangles scattered in a box (an unstructured soup).
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

/// UV sphere, outward oriented.
fn uv_sphere(nu: u32, nv: u32, r: f64) -> IndexedMesh {
    let mut p = vec![Point3::new(0.0, 0.0, r)];
    for j in 1..nv {
        let th = std::f64::consts::PI * f64::from(j) / f64::from(nv);
        for i in 0..nu {
            let ph = std::f64::consts::TAU * f64::from(i) / f64::from(nu);
            p.push(Point3::new(
                r * th.sin() * ph.cos(),
                r * th.sin() * ph.sin(),
                r * th.cos(),
            ));
        }
    }
    p.push(Point3::new(0.0, 0.0, -r));
    let south = p.len() as u32 - 1;
    let ring = |j: u32, i: u32| 1 + (j - 1) * nu + (i % nu);
    let mut t = Vec::new();
    for i in 0..nu {
        t.push([0, ring(1, i), ring(1, i + 1)]);
    }
    for j in 1..nv - 1 {
        for i in 0..nu {
            let (a, b, c, d) = (
                ring(j, i),
                ring(j + 1, i),
                ring(j + 1, i + 1),
                ring(j, i + 1),
            );
            t.push([a, b, c]);
            t.push([a, c, d]);
        }
    }
    for i in 0..nu {
        t.push([south, ring(nv - 1, i + 1), ring(nv - 1, i)]);
    }
    IndexedMesh::new(p, t).expect("sphere")
}

fn tri_of(m: &IndexedMesh, f: u32) -> [Point3; 3] {
    m.triangle(FaceId(f))
}

/// Möller–Trumbore, both sides. Returns `t`.
fn brute_hit(o: Point3, d: Vec3, [a, b, c]: [Point3; 3]) -> Option<f64> {
    let e1 = b - a;
    let e2 = c - a;
    let p = d.cross(e2);
    let det = e1.dot(p);
    if det.abs() < 1e-14 {
        return None;
    }
    let inv = 1.0 / det;
    let s = o - a;
    let u = s.dot(p) * inv;
    let q = s.cross(e1);
    let v = d.dot(q) * inv;
    let t = e2.dot(q) * inv;
    (u >= 0.0 && v >= 0.0 && u + v <= 1.0 && t >= 0.0).then_some(t)
}

/// Closest point by plane projection + edge fallback (independent of the crate's kernel).
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
    let n = (b - a).cross(c - a);
    let n2 = n.length_squared();
    if n2 > 0.0 {
        let q = p - n * ((p - a).dot(n) / n2);
        let inside = (b - a).cross(q - a).dot(n) >= 0.0
            && (c - b).cross(q - b).dot(n) >= 0.0
            && (a - c).cross(q - c).dot(n) >= 0.0;
        if inside {
            best = best.min((p - q).length_squared());
        }
    }
    best
}

fn brute_ray(m: &IndexedMesh, o: Point3, d: Vec3) -> Vec<(f64, u32)> {
    let mut v: Vec<(f64, u32)> = (0..m.face_count() as u32)
        .filter_map(|f| brute_hit(o, d, tri_of(m, f)).map(|t| (t, f)))
        .collect();
    v.sort_by(|a, b| a.0.total_cmp(&b.0));
    v
}

fn check_ray_queries(m: &IndexedMesh, rng: &mut Rng, n: usize) {
    let s = MeshSpatial::build(Arc::new(m.clone()), &BvhOpts::default(), &ctx()).expect("build");
    let mut all = Vec::new();
    for _ in 0..n {
        let o = rng.point(14.0);
        let dir = rng.dir();
        let ray = Ray3 { origin: o, dir };
        let want = brute_ray(m, o, dir.get());
        let got = s.ray_cast(&ray, f64::INFINITY);
        match (want.first(), got) {
            (None, None) => {}
            (Some(&(t, _)), Some(h)) => {
                assert!((h.t - t).abs() <= 1e-9 * (1.0 + t), "t {} vs {t}", h.t);
                // the reported barycentrics reproduce the hit point
                let [a, b, c] = tri_of(m, h.face.0);
                let q = a * h.bary.x + b * h.bary.y + c * h.bary.z;
                assert!((q - ray.at(h.t)).length() < 1e-8);
            }
            // Edge-grazing ambiguity: brute force (tolerance based) and the watertight test may
            // disagree only for hits within rounding of an edge. Random rays: essentially never.
            (w, g) => panic!("hit mismatch: brute {w:?} vs bvh {g:?}"),
        }
        all.clear();
        s.ray_cast_all(&ray, f64::INFINITY, &mut all);
        assert_eq!(all.len(), want.len(), "ray_cast_all count");
        for (h, w) in all.iter().zip(&want) {
            assert!((h.t - w.0).abs() <= 1e-9 * (1.0 + w.0));
        }
        assert!(all.windows(2).all(|w| w[0].t <= w[1].t));
        // t_max clips
        if let Some(&(t, _)) = want.first() {
            assert!(s.ray_cast(&ray, t * 0.5).is_none() || t == 0.0);
        }
    }
}

#[test]
fn ray_cast_matches_brute_force_soup() {
    let mut rng = Rng(1);
    let m = random_soup(&mut rng, 600);
    check_ray_queries(&m, &mut rng, 10_000);
}

#[test]
fn ray_cast_matches_brute_force_closed_meshes() {
    let mut rng = Rng(2);
    check_ray_queries(&common::torus(40, 24, 6.0, 2.0), &mut rng, 3_000);
    check_ray_queries(&uv_sphere(30, 20, 5.0), &mut rng, 3_000);
}

#[test]
fn ray_axis_aligned_and_edge_cases() {
    // Axis-aligned rays against a cube-like sphere never miss a closed surface.
    let s = spatial(uv_sphere(16, 12, 3.0));
    for dir in [
        Dir3::X,
        Dir3::Y,
        Dir3::Z,
        Dir3::NEG_X,
        Dir3::NEG_Y,
        Dir3::NEG_Z,
    ] {
        let o = Point3::ZERO;
        let h = s
            .ray_cast(&Ray3 { origin: o, dir }, f64::INFINITY)
            .expect("hit");
        assert!((h.t - 3.0).abs() < 0.2, "t = {}", h.t);
    }
    // through the pole vertex (shared by many faces): watertight, never misses
    let h = s.ray_cast(
        &Ray3 {
            origin: Point3::new(0.0, 0.0, 10.0),
            dir: Dir3::NEG_Z,
        },
        f64::INFINITY,
    );
    assert!((h.expect("pole hit").t - 7.0).abs() < 1e-9);
    // from outside, pointing away
    let miss = s.ray_cast(
        &Ray3 {
            origin: Point3::new(0.0, 0.0, 10.0),
            dir: Dir3::Z,
        },
        f64::INFINITY,
    );
    assert!(miss.is_none());
}

#[test]
fn ray_watertight_over_shared_edges() {
    // Rays aimed exactly at mesh vertices and edge midpoints must hit.
    let m = uv_sphere(24, 16, 4.0);
    let s = spatial(m.clone());
    let mut n = 0;
    for f in 0..m.face_count() as u32 {
        let [a, b, c] = tri_of(&m, f);
        for target in [a, b, c, (a + b) * 0.5, (b + c) * 0.5, (c + a) * 0.5] {
            let o = target * 2.0;
            let dir = Dir3::new_unchecked((target - o).normalize());
            let h = s.ray_cast(&Ray3 { origin: o, dir }, f64::INFINITY);
            assert!(h.is_some(), "leaked through an edge/vertex");
            n += 1;
        }
    }
    assert!(n > 1000);
}

#[test]
fn closest_point_matches_brute_force() {
    let mut rng = Rng(3);
    for m in [random_soup(&mut rng, 500), common::torus(30, 20, 6.0, 2.0)] {
        let s = spatial(m.clone());
        for _ in 0..10_000 {
            let p = rng.point(14.0);
            let want = (0..m.face_count() as u32)
                .map(|f| brute_closest_tri(p, tri_of(&m, f)))
                .fold(f64::INFINITY, f64::min);
            let got = s.closest_point(p, f64::INFINITY).expect("some");
            assert!(
                (got.dist_sq - want).abs() <= 1e-9 * (1.0 + want),
                "{} vs {want}",
                got.dist_sq
            );
            // point and barycentrics are consistent
            let [a, b, c] = tri_of(&m, got.face.0);
            let q = a * got.bary.x + b * got.bary.y + c * got.bary.z;
            assert!((q - got.point).length() < 1e-9);
            assert!(((got.point - p).length_squared() - got.dist_sq).abs() < 1e-9);
            // max_dist clips
            if want > 0.0 {
                assert!(s.closest_point(p, want.sqrt() * 0.99).is_none());
            }
        }
    }
}

#[test]
fn closest_point_features() {
    let m = IndexedMesh::new(
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
        vec![[0, 1, 2]],
    )
    .expect("tri");
    let s = spatial(m);
    let c = |p: Point3| s.closest_point(p, f64::INFINITY).expect("cp").feature;
    assert_eq!(c(Point3::new(-1.0, -1.0, 1.0)), TriFeature::Vertex(0));
    assert_eq!(c(Point3::new(2.0, -1.0, 0.0)), TriFeature::Vertex(1));
    assert_eq!(c(Point3::new(-1.0, 3.0, 0.0)), TriFeature::Vertex(2));
    assert_eq!(c(Point3::new(0.5, -1.0, 0.0)), TriFeature::Edge(0));
    assert_eq!(c(Point3::new(1.0, 1.0, 0.0)), TriFeature::Edge(1));
    assert_eq!(c(Point3::new(-1.0, 0.5, 0.0)), TriFeature::Edge(2));
    assert_eq!(c(Point3::new(0.2, 0.2, 5.0)), TriFeature::Face);
}

#[test]
fn faces_in_aabb_matches_brute_force() {
    let mut rng = Rng(4);
    let m = random_soup(&mut rng, 800);
    let s = spatial(m.clone());
    for _ in 0..500 {
        let b = Aabb3::new(rng.point(12.0), rng.point(12.0));
        let mut got = Vec::new();
        s.faces_in_aabb(&b, &mut got);
        let want: Vec<FaceId> = (0..m.face_count() as u32)
            .filter(|&f| {
                let [p, q, r] = tri_of(&m, f);
                let (lo, hi) = (p.min(q).min(r), p.max(q).max(r));
                !(lo.cmpgt(b.max).any() || hi.cmplt(b.min).any())
            })
            .map(FaceId)
            .collect();
        assert_eq!(got, want);
    }
}

#[test]
fn bvh_structure_invariants() {
    let mut rng = Rng(5);
    let m = random_soup(&mut rng, 5_000);
    let s = spatial(m.clone());
    let bvh = s.bvh();
    let mut seen = vec![false; m.face_count()];
    let mut max_leaf = 0;
    let root = bvh.root().expect("root");
    let mut stack = vec![root];
    while let Some(i) = stack.pop() {
        let n = bvh.node(i);
        match n.kind {
            BvhNodeKind::Inner { left, right } => {
                for c in [left, right] {
                    let cb = bvh.node(c).bounds;
                    assert!(cb.min.cmpge(n.bounds.min).all() && cb.max.cmple(n.bounds.max).all());
                    stack.push(c);
                }
            }
            BvhNodeKind::Leaf { first, count } => {
                max_leaf = max_leaf.max(count);
                for slot in first..first + count {
                    let f = bvh.prim(slot) as usize;
                    assert!(!seen[f]);
                    seen[f] = true;
                    let [a, b, c] = tri_of(&m, f as u32);
                    for v in [a, b, c] {
                        assert!(v.cmpge(n.bounds.min).all() && v.cmple(n.bounds.max).all());
                    }
                }
            }
        }
    }
    assert!(seen.iter().all(|&x| x));
    assert!(max_leaf <= 4);
    assert!(bvh.depth() < 60);
}

#[test]
fn degenerate_inputs() {
    // empty mesh
    let s = spatial(IndexedMesh::default());
    assert!(s.closest_point(Point3::ZERO, f64::INFINITY).is_none());
    assert!(
        s.ray_cast(
            &Ray3 {
                origin: Point3::ZERO,
                dir: Dir3::X
            },
            1e9
        )
        .is_none()
    );
    assert!(s.winding_number(Point3::ZERO).abs() < 1e-300);
    // 5 000 identical triangles: leaves stay <= 4 and depth stays bounded
    let p = vec![Point3::ZERO, Point3::X, Point3::Y];
    let m = IndexedMesh::new(p, vec![[0, 1, 2]; 5_000]).expect("dup");
    let s = spatial(m);
    assert!(s.bvh().depth() < 40);
    let cp = s
        .closest_point(Point3::new(0.2, 0.2, 1.0), f64::INFINITY)
        .expect("cp");
    assert_eq!(cp.face, FaceId(0)); // lowest id wins ties
    // bad options and boxes
    let one = [Aabb3::new(Point3::ZERO, Point3::ONE)];
    let bad = BvhOpts {
        max_leaf_size: 0,
        ..BvhOpts::default()
    };
    assert!(matches!(
        Bvh::build(&one, &bad, &ctx()),
        Err(MeshError::InvalidOption { .. })
    ));
    let nan = [Aabb3::new(Point3::ZERO, Point3::NAN)];
    assert!(Bvh::build(&nan, &BvhOpts::default(), &ctx()).is_err());
}

#[test]
fn cancellation() {
    let tok = CancelToken::new();
    tok.cancel();
    let c = JobCtx::with_token(tok);
    let m = Arc::new(common::torus(300, 300, 10.0, 3.0));
    assert!(matches!(
        MeshSpatial::build(m, &BvhOpts::default(), &c),
        Err(MeshError::Cancelled)
    ));
}

#[test]
fn winding_number_sphere() {
    let s = spatial(uv_sphere(48, 32, 5.0));
    assert!((s.winding_number(Point3::new(0.3, 0.2, -0.1)) - 1.0).abs() < 0.01);
    assert!(s.winding_number(Point3::new(9.0, 1.0, 2.0)).abs() < 0.01);
    assert!(s.is_inside(Point3::new(1.0, 1.0, 1.0)));
    assert!(!s.is_inside(Point3::new(6.0, 0.0, 0.0)));
    // signed distance: negative inside, ~ r - |p|
    let d = s
        .signed_distance(Point3::new(2.0, 0.0, 0.0), 10.0)
        .expect("d");
    assert!((d + 3.0).abs() < 0.05, "{d}");
    let d = s
        .signed_distance(Point3::new(8.0, 0.0, 0.0), 10.0)
        .expect("d");
    assert!((d - 3.0).abs() < 0.05, "{d}");
    assert!(
        s.signed_distance(Point3::new(30.0, 0.0, 0.0), 10.0)
            .is_none()
    );
}

/// Done-when: fast winding number within 0.01 of exact at beta = 2 for points at least one
/// mean edge length from the surface.
#[test]
fn fast_winding_matches_exact() {
    let mut rng = Rng(6);
    for m in [
        uv_sphere(60, 40, 5.0),
        common::torus(60, 30, 8.0, 3.0),
        random_soup(&mut Rng(9), 400), // open soup: generalized winding number
    ] {
        let s = spatial(m.clone());
        let mean_edge = {
            let mut sum = 0.0;
            for t in &m.tris {
                for k in 0..3 {
                    sum += (m.positions[t[k] as usize] - m.positions[t[(k + 1) % 3] as usize])
                        .length();
                }
            }
            sum / (3.0 * m.face_count() as f64)
        };
        let mut worst: f64 = 0.0;
        let mut checked = 0;
        while checked < 1_000 {
            let p = rng.point(14.0);
            let Some(cp) = s.closest_point(p, f64::INFINITY) else {
                continue;
            };
            if cp.dist_sq.sqrt() < mean_edge {
                continue;
            }
            checked += 1;
            let fast = s.winding_number(p);
            let exact = s.winding_number_exact(p);
            worst = worst.max((fast - exact).abs());
        }
        assert!(worst < 0.01, "worst error {worst} (mean edge {mean_edge})");
        // larger beta is not worse than beta = 2 on a sample
        let p = rng.point(14.0);
        let e = s.winding_number_exact(p);
        assert!((s.winding_number_with_beta(p, 4.0) - e).abs() < 0.01);
    }
}

#[test]
fn identical_across_thread_counts() {
    let big = common::torus(300, 200, 20.0, 6.0); // 120 k triangles, above the parallel cutoffs
    let mesh = Arc::new(big);
    let mut rng = Rng(7);
    let pts: Vec<Point3> = (0..2_000).map(|_| rng.point(30.0)).collect();
    let rays: Vec<Ray3> = (0..500)
        .map(|_| Ray3 {
            origin: rng.point(30.0),
            dir: rng.dir(),
        })
        .collect();
    let run = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .expect("pool");
        pool.install(|| {
            let s = MeshSpatial::build(mesh.clone(), &BvhOpts::default(), &ctx()).expect("build");
            let nodes: Vec<_> = (0..s.bvh().node_count() as u32)
                .map(|i| format!("{:?}", s.bvh().node(i)))
                .collect();
            let prims: Vec<u32> = (0..s.bvh().prim_count() as u32)
                .map(|i| s.bvh().prim(i))
                .collect();
            let w: Vec<u64> = s
                .winding_numbers(&pts, 2.0)
                .iter()
                .map(|x| x.to_bits())
                .collect();
            let hits: Vec<Option<RayHit>> =
                rays.iter().map(|r| s.ray_cast(r, f64::INFINITY)).collect();
            let cps: Vec<u64> = pts
                .iter()
                .map(|&p| {
                    s.closest_point(p, f64::INFINITY)
                        .expect("cp")
                        .dist_sq
                        .to_bits()
                })
                .collect();
            (nodes, prims, w, hits, cps)
        })
    };
    let a = run(1);
    assert_eq!(a, run(4));
    assert_eq!(a, run(16));
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(24))]

    #[test]
    fn prop_queries_equal_brute_force(seed in any::<u64>(), n in 1usize..250) {
        let mut rng = Rng(seed);
        let m = random_soup(&mut rng, n);
        let s = spatial(m.clone());
        for _ in 0..40 {
            let p = rng.point(14.0);
            let want = (0..m.face_count() as u32)
                .map(|f| brute_closest_tri(p, tri_of(&m, f)))
                .fold(f64::INFINITY, f64::min);
            let got = s.closest_point(p, f64::INFINITY).expect("some").dist_sq;
            prop_assert!((got - want).abs() <= 1e-9 * (1.0 + want));
            let dir = rng.dir();
            let w = brute_ray(&m, p, dir.get());
            let g = s.ray_cast(&Ray3 { origin: p, dir }, f64::INFINITY);
            prop_assert_eq!(!w.is_empty(), g.is_some());
            if let (Some(w), Some(g)) = (w.first(), g) {
                prop_assert!((w.0 - g.t).abs() <= 1e-9 * (1.0 + w.0));
            }
        }
    }
}
