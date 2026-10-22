//! MESH-19: self-intersection detection against exact brute force, an independent integer
//! oracle, synthetic cases and the corpus manifest flags.

#![allow(clippy::expect_used, clippy::indexing_slicing)]

#[path = "common/mod.rs"]
mod common;

use std::sync::Arc;

use amg_core::Point3;
use amg_jobs::{CancelToken, JobCtx};
use amg_mesh::corpus::bench::geodesic_sphere;
use amg_mesh::corpus::{Defect, FileStatus, Manifest, corpus_root, read_mesh};
use amg_mesh::{
    BvhOpts, FaceId, IndexedMesh, MeshError, MeshSpatial, SelfIntersectOpts, WeldOpts,
    faces_intersect, self_intersections,
};
use rayon::prelude::*;

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
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
}

fn ctx() -> JobCtx {
    JobCtx::detached()
}

fn spatial(m: IndexedMesh) -> MeshSpatial {
    MeshSpatial::build(Arc::new(m), &BvhOpts::default(), &ctx()).expect("build")
}

fn run(m: &IndexedMesh) -> Vec<[FaceId; 2]> {
    self_intersections(&spatial(m.clone()), &SelfIntersectOpts::default(), &ctx()).expect("run")
}

fn brute(m: &IndexedMesh) -> Vec<[FaceId; 2]> {
    let n = m.tris.len() as u32;
    let mut v: Vec<[FaceId; 2]> = (0..n)
        .into_par_iter()
        .flat_map_iter(|a| {
            (a + 1..n)
                .filter(move |&b| faces_intersect(m, FaceId(a), FaceId(b)))
                .map(move |b| [FaceId(a), FaceId(b)])
        })
        .collect();
    v.sort_unstable();
    v
}

fn mesh(pts: &[[f64; 3]], tris: &[[u32; 3]]) -> IndexedMesh {
    IndexedMesh::new(
        pts.iter().map(|p| Point3::new(p[0], p[1], p[2])).collect(),
        tris.to_vec(),
    )
    .expect("mesh")
}

fn pairs(m: &IndexedMesh) -> Vec<(u32, u32)> {
    run(m).iter().map(|p| (p[0].0, p[1].0)).collect()
}

// ----------------------------------------------------------------------------- hand cases

#[test]
fn hand_cases() {
    // Two disjoint triangles far apart.
    let m = mesh(
        &[
            [0., 0., 0.],
            [1., 0., 0.],
            [0., 1., 0.],
            [5., 5., 5.],
            [6., 5., 5.],
            [5., 6., 5.],
        ],
        &[[0, 1, 2], [3, 4, 5]],
    );
    assert!(pairs(&m).is_empty());
    // A triangle pierced by a second one (crossing).
    let m = mesh(
        &[
            [0., 0., 0.],
            [4., 0., 0.],
            [0., 4., 0.],
            [1., 1., -1.],
            [1., 1., 1.],
            [3., 1., 0.5],
        ],
        &[[0, 1, 2], [3, 4, 5]],
    );
    assert_eq!(pairs(&m), vec![(0, 1)]);
    // Touching at a point (apex on the face) counts: T-junction style contact.
    let m = mesh(
        &[
            [0., 0., 0.],
            [4., 0., 0.],
            [0., 4., 0.],
            [1., 1., 0.],
            [1., 1., 2.],
            [2., 1., 2.],
        ],
        &[[0, 1, 2], [3, 4, 5]],
    );
    assert_eq!(pairs(&m), vec![(0, 1)]);
    // Just missing by a hair (not touching): exact predicates see the gap.
    let m = mesh(
        &[
            [0., 0., 0.],
            [4., 0., 0.],
            [0., 4., 0.],
            [1., 1., 1e-300],
            [1., 1., 2.],
            [2., 1., 2.],
        ],
        &[[0, 1, 2], [3, 4, 5]],
    );
    assert!(pairs(&m).is_empty());
    // Edge neighbours of a flat fan (coplanar, opposite sides): not an intersection.
    let m = mesh(
        &[[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [1., 1., 0.]],
        &[[0, 1, 2], [1, 3, 2]],
    );
    assert!(pairs(&m).is_empty());
    // Edge neighbours folded onto the same side (coplanar): overlap. Opposite side: fine.
    let m = mesh(
        &[[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0.5, -2., 0.]],
        &[[0, 1, 2], [0, 1, 3]],
    );
    let m2 = mesh(
        &[[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0.5, 0.5, 0.]],
        &[[0, 1, 2], [0, 1, 3]],
    );
    assert!(pairs(&m).is_empty());
    assert_eq!(pairs(&m2), vec![(0, 1)]);
    // A vertex-adjacent pair that merely shares its apex: fine; one that also overlaps: not.
    let m = mesh(
        &[
            [0., 0., 0.],
            [1., 0., 0.],
            [0., 1., 0.],
            [-1., 0., 0.],
            [0., -1., 0.],
        ],
        &[[0, 1, 2], [0, 3, 4]],
    );
    assert!(pairs(&m).is_empty());
    let m = mesh(
        &[
            [0., 0., 0.],
            [2., 0., 0.],
            [0., 2., 0.],
            [1., 0.2, 0.],
            [0.2, 1., 0.],
        ],
        &[[0, 1, 2], [0, 3, 4]],
    );
    assert_eq!(pairs(&m), vec![(0, 1)]);
    // Shared vertex, second triangle pierces the first away from the vertex.
    let m = mesh(
        &[
            [0., 0., 0.],
            [4., 0., 0.],
            [0., 4., 0.],
            [1., 1., 1.],
            [1., 1., -1.],
        ],
        &[[0, 1, 2], [0, 3, 4]],
    );
    assert_eq!(pairs(&m), vec![(0, 1)]);
    // Duplicate faces and degenerate faces.
    let m = mesh(
        &[[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [2., 0., 0.]],
        &[[0, 1, 2], [2, 1, 0], [0, 1, 3]],
    );
    assert_eq!(
        pairs(&m),
        vec![(0, 1)],
        "duplicate reported, collinear skipped"
    );
}

#[test]
fn empty_and_single() {
    assert!(run(&mesh(&[], &[])).is_empty());
    assert!(
        run(&mesh(
            &[[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]],
            &[[0, 1, 2]]
        ))
        .is_empty()
    );
}

// --------------------------------------------------------------------- brute-force parity

fn random_soup(rng: &mut Rng, n: usize, size: f64, spread: f64) -> IndexedMesh {
    let mut p = Vec::new();
    let mut t = Vec::new();
    for i in 0..n as u32 {
        let c = [rng.f() * spread, rng.f() * spread, rng.f() * spread];
        for _ in 0..3 {
            p.push(Point3::new(
                c[0] + (rng.f() - 0.5) * size,
                c[1] + (rng.f() - 0.5) * size,
                c[2] + (rng.f() - 0.5) * size,
            ));
        }
        t.push([3 * i, 3 * i + 1, 3 * i + 2]);
    }
    IndexedMesh::new(p, t).expect("soup")
}

#[test]
fn soup_10k_equals_exact_brute_force() {
    let mut rng = Rng(7);
    // Roughly 1 % of the triangles cross something.
    let m = random_soup(&mut rng, 10_000, 1.0, 40.0);
    let fast = run(&m);
    assert!(!fast.is_empty(), "test soup should contain intersections");
    assert_eq!(fast, brute(&m));
}

#[test]
fn dense_soup_equals_brute_force() {
    let mut rng = Rng(11);
    let m = random_soup(&mut rng, 1_500, 1.0, 5.0);
    let fast = run(&m);
    assert!(fast.len() > 200);
    assert_eq!(fast, brute(&m));
}

/// Lattice triangles drawn from a small vertex pool: heavy sharing, coplanarity and touching.
fn lattice_pool(rng: &mut Rng, n: usize, side: u64) -> IndexedMesh {
    let mut p = Vec::new();
    for x in 0..side {
        for y in 0..side {
            for z in 0..side {
                p.push(Point3::new(x as f64, y as f64, z as f64));
            }
        }
    }
    let np = p.len() as u64;
    let mut t = Vec::new();
    while t.len() < n {
        let (a, b, c) = (rng.below(np), rng.below(np), rng.below(np));
        if a != b && b != c && a != c {
            t.push([a as u32, b as u32, c as u32]);
        }
    }
    IndexedMesh::new(p, t).expect("pool")
}

#[test]
fn lattice_pool_equals_brute_force() {
    let mut rng = Rng(3);
    for side in [3, 4] {
        let m = lattice_pool(&mut rng, 700, side);
        let fast = run(&m);
        assert!(!fast.is_empty());
        assert_eq!(fast, brute(&m), "side {side}");
    }
}

// ------------------------------------------------- independent integer (i128) oracle

type V = [i128; 3];

fn sub(a: V, b: V) -> V {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn cross(a: V, b: V) -> V {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn dot(a: V, b: V) -> i128 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn iv(p: Point3) -> V {
    [p.x as i128, p.y as i128, p.z as i128]
}

/// Closed segment `pq` against closed triangle `abc` with exact rational arithmetic.
fn seg_tri_int(p: V, q: V, a: V, b: V, c: V) -> bool {
    let n = cross(sub(b, a), sub(c, a));
    let dp = dot(n, sub(p, a));
    let dq = dot(n, sub(q, a));
    if (dp > 0 && dq > 0) || (dp < 0 && dq < 0) {
        return false;
    }
    if dp == 0 && dq == 0 {
        // Coplanar: closed segment vs closed triangle by SAT in the plane.
        // Axes: the three triangle-edge normals, the segment normal; all within the plane.
        let tri = [a, b, c];
        let mut axes: Vec<V> = Vec::new();
        for k in 0..3 {
            axes.push(cross(n, sub(tri[(k + 1) % 3], tri[k])));
        }
        axes.push(cross(n, sub(q, p)));
        for ax in axes {
            if ax == [0, 0, 0] {
                continue;
            }
            let (mut lo, mut hi) = (i128::MAX, i128::MIN);
            for t in tri {
                let d = dot(ax, t);
                lo = lo.min(d);
                hi = hi.max(d);
            }
            let (s0, s1) = (dot(ax, p), dot(ax, q));
            if s0.max(s1) < lo || s0.min(s1) > hi {
                return false;
            }
        }
        // A degenerate (point) segment lies on the first axis test; add the along-segment axis.
        let d = sub(q, p);
        if d != [0, 0, 0] {
            let (mut lo, mut hi) = (i128::MAX, i128::MIN);
            for t in tri {
                let s = dot(d, t);
                lo = lo.min(s);
                hi = hi.max(s);
            }
            if dot(d, p).max(dot(d, q)) < lo || dot(d, p).min(dot(d, q)) > hi {
                return false;
            }
        }
        return true;
    }
    // x = p + (dp / (dp - dq)) (q - p) = X / D with D > 0.
    let mut dd = dp - dq;
    let mut xx = [
        p[0] * dd + dp * (q[0] - p[0]),
        p[1] * dd + dp * (q[1] - p[1]),
        p[2] * dd + dp * (q[2] - p[2]),
    ];
    if dd < 0 {
        dd = -dd;
        xx = [-xx[0], -xx[1], -xx[2]];
    }
    let tri = [a, b, c];
    (0..3).all(|k| {
        let (u, v) = (tri[k], tri[(k + 1) % 3]);
        let rel = [xx[0] - u[0] * dd, xx[1] - u[1] * dd, xx[2] - u[2] * dd];
        dot(n, cross(sub(v, u), rel)) >= 0
    })
}

/// Closed triangles by the separating axis theorem (13 axes incl. in-plane edge normals).
fn tri_tri_sat(a: [V; 3], b: [V; 3]) -> bool {
    let na = cross(sub(a[1], a[0]), sub(a[2], a[0]));
    let nb = cross(sub(b[1], b[0]), sub(b[2], b[0]));
    let mut axes = vec![na, nb];
    for i in 0..3 {
        for j in 0..3 {
            axes.push(cross(sub(a[(i + 1) % 3], a[i]), sub(b[(j + 1) % 3], b[j])));
        }
    }
    for i in 0..3 {
        axes.push(cross(na, sub(a[(i + 1) % 3], a[i])));
        axes.push(cross(nb, sub(b[(i + 1) % 3], b[i])));
    }
    for ax in axes {
        if ax == [0, 0, 0] {
            continue;
        }
        let pa = a.map(|v| dot(ax, v));
        let pb = b.map(|v| dot(ax, v));
        let (amin, amax) = (pa[0].min(pa[1]).min(pa[2]), pa[0].max(pa[1]).max(pa[2]));
        let (bmin, bmax) = (pb[0].min(pb[1]).min(pb[2]), pb[0].max(pb[1]).max(pb[2]));
        if amax < bmin || bmax < amin {
            return false;
        }
    }
    true
}

fn oracle(m: &IndexedMesh, fa: usize, fb: usize) -> bool {
    let (ia, ib) = (m.tris[fa], m.tris[fb]);
    let a = ia.map(|i| iv(m.positions[i as usize]));
    let b = ib.map(|i| iv(m.positions[i as usize]));
    let area0 = |t: [V; 3]| cross(sub(t[1], t[0]), sub(t[2], t[0])) == [0, 0, 0];
    if area0(a) || area0(b) {
        return false;
    }
    let shared: Vec<(usize, usize)> = (0..3)
        .flat_map(|i| (0..3).map(move |j| (i, j)))
        .filter(|&(i, j)| ia[i] == ib[j])
        .collect();
    match shared.len() {
        0 => tri_tri_sat(a, b),
        1 => {
            let (i, j) = shared[0];
            seg_tri_int(a[(i + 1) % 3], a[(i + 2) % 3], b[0], b[1], b[2])
                || seg_tri_int(b[(j + 1) % 3], b[(j + 2) % 3], a[0], a[1], a[2])
        }
        2 => {
            let (i0, i1) = (shared[0].0, shared[1].0);
            let third_a = 3 - i0 - i1;
            let third_b = 3 - shared[0].1 - shared[1].1;
            let (u, v) = (a[i0], a[i1]);
            let na = cross(sub(a[1], a[0]), sub(a[2], a[0]));
            if dot(na, sub(b[third_b], a[0])) != 0 {
                return false;
            }
            let side = |p: V| dot(na, cross(sub(v, u), sub(p, u))).signum();
            side(a[third_a]) == side(b[third_b])
        }
        _ => true,
    }
}

#[test]
fn matches_independent_integer_oracle() {
    let mut rng = Rng(99);
    for (seed_side, n) in [(3u64, 500usize), (4, 500), (5, 400)] {
        let m = lattice_pool(&mut rng, n, seed_side);
        let fast = run(&m);
        let mut want = Vec::new();
        for a in 0..m.tris.len() {
            for b in a + 1..m.tris.len() {
                if oracle(&m, a, b) {
                    want.push([FaceId(a as u32), FaceId(b as u32)]);
                }
            }
        }
        assert!(!want.is_empty());
        assert_eq!(fast, want, "side {seed_side}");
    }
    // Random integer-coordinate soups (no sharing) hit the SAT oracle with touching cases.
    for seed in 0..3 {
        let mut r = Rng(1000 + seed);
        let mut p = Vec::new();
        let mut t = Vec::new();
        for i in 0..600u32 {
            for _ in 0..3 {
                p.push(Point3::new(
                    r.below(7) as f64,
                    r.below(7) as f64,
                    r.below(7) as f64,
                ));
            }
            t.push([3 * i, 3 * i + 1, 3 * i + 2]);
        }
        let m = IndexedMesh::new(p, t).expect("m");
        let want: Vec<[FaceId; 2]> = (0..600)
            .flat_map(|a| (a + 1..600).map(move |b| (a, b)))
            .filter(|&(a, b)| oracle(&m, a, b))
            .map(|(a, b)| [FaceId(a as u32), FaceId(b as u32)])
            .collect();
        assert_eq!(run(&m), want, "soup seed {seed}");
    }
}

// ----------------------------------------------------------------- synthetic and clean

fn merged(a: &IndexedMesh, b: &IndexedMesh, offset: Point3) -> IndexedMesh {
    let mut soup: Vec<[Point3; 3]> = (0..a.tris.len() as u32)
        .map(|f| a.triangle(FaceId(f)))
        .collect();
    soup.extend((0..b.tris.len() as u32).map(|f| b.triangle(FaceId(f)).map(|p| p + offset)));
    IndexedMesh::from_soup(&soup, WeldOpts::EXACT).expect("merge")
}

#[test]
fn intersecting_spheres_found_and_clean_shapes_are_clean() {
    let s = geodesic_sphere(12, 10.0, None);
    assert!(run(&s).is_empty(), "a sphere has no self-intersections");
    assert!(run(&common::torus(40, 24, 6.0, 2.0)).is_empty());
    let apart = merged(&s, &s, Point3::new(25.0, 0.0, 0.0));
    assert!(run(&apart).is_empty());
    let touching_far = merged(&s, &s, Point3::new(20.5, 0.0, 0.0));
    assert!(run(&touching_far).is_empty());
    let both = merged(&s, &s, Point3::new(12.0, 0.0, 0.0));
    let found = run(&both);
    assert!(found.len() > 20, "found {}", found.len());
    let split = s.tris.len() as u32;
    assert!(
        found.iter().all(|p| p[0].0 < split && p[1].0 >= split),
        "each pair crosses between the two spheres"
    );
    // Equals brute force on the merged shape.
    assert_eq!(found, brute(&both));
}

#[test]
fn deterministic_across_threads_and_prefix_and_cancel() {
    let mut rng = Rng(5);
    let m = random_soup(&mut rng, 6_000, 1.0, 20.0);
    let want = run(&m);
    assert!(want.len() > 50);
    for threads in [1, 4, 16] {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .expect("pool");
        assert_eq!(pool.install(|| run(&m)), want, "{threads} threads");
    }
    // `max_pairs` gives a deterministic prefix of the full set's size, all genuine pairs.
    let s = spatial(m.clone());
    let opts = SelfIntersectOpts {
        max_pairs: Some(10),
    };
    let a = self_intersections(&s, &opts, &ctx()).expect("capped");
    let b = self_intersections(&s, &opts, &ctx()).expect("capped");
    assert_eq!(a, b);
    assert_eq!(a.len(), 10);
    assert!(a.iter().all(|p| want.contains(p)));
    // Cancellation.
    let token = CancelToken::new();
    token.cancel();
    let r = self_intersections(
        &s,
        &SelfIntersectOpts::default(),
        &JobCtx::with_token(token),
    );
    assert!(matches!(r, Err(MeshError::Cancelled)));
}

// ------------------------------------------------------------------------------- corpus

#[test]
fn corpus_self_intersection_flags() {
    let manifest = Manifest::load().expect("corpus.toml");
    let root = corpus_root();
    let mut report = Vec::new();
    let (mut checked, mut clean_checked) = (0, 0);
    for e in &manifest.meshes {
        let FileStatus::Ready(path) = e.locate(&root) else {
            continue;
        };
        let Ok(m) = read_mesh(&path) else {
            continue;
        };
        let found = run(&m);
        let flagged = e.defects.contains(&Defect::SelfIntersections);
        checked += 1;
        let line = format!(
            "{} {} faces={} found={} flag={}",
            e.id,
            e.file,
            m.tris.len(),
            found.len(),
            flagged
        );
        if e.defects.is_empty() {
            clean_checked += 1;
            assert!(found.is_empty(), "clean control reports pairs: {line}");
        }
        if (!found.is_empty()) != flagged {
            report.push(line);
        }
    }
    eprintln!("checked {checked} models ({clean_checked} clean); mismatches:");
    for l in &report {
        eprintln!("  {l}");
    }
    assert!(
        report.is_empty(),
        "{} manifest flag mismatches",
        report.len()
    );
}
