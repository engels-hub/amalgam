//! MESH-21 tests: self-intersection removal (delete + k-ring + refill, iterated).

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::print_stderr
)]

use std::sync::Arc;

use amg_core::{Point3, Vec3};
use amg_jobs::{CancelToken, JobCtx};
use amg_mesh::corpus::{Defect, FileStatus, Manifest, corpus_root, read_mesh};
use amg_mesh::{
    AnalyzeOpts, BvhOpts, FaceId, IndexedMesh, MeshError, MeshSpatial, RemoveSelfIntersectionsOpts,
    RepairOpts, SelfIntersectOpts, analyze, remove_self_intersections,
    remove_self_intersections_with, repair, self_intersections,
};
use amg_testkit::{TriSoup, shapes};
use proptest::prelude::*;

fn ctx() -> JobCtx {
    JobCtx::detached()
}

fn require_testdata() -> bool {
    std::env::var("AMG_REQUIRE_TESTDATA").is_ok_and(|v| v == "1")
}

fn soup(m: &IndexedMesh) -> TriSoup<'_> {
    TriSoup {
        positions: &m.positions,
        triangles: &m.tris,
    }
}

fn pairs(m: &IndexedMesh) -> Vec<[FaceId; 2]> {
    let s = MeshSpatial::build(Arc::new(m.clone()), &BvhOpts::default(), &ctx()).unwrap();
    self_intersections(&s, &SelfIntersectOpts::default(), &ctx()).unwrap()
}

fn diag(m: &IndexedMesh) -> f64 {
    let b = m.aabb();
    (b.max - b.min).length()
}

fn icosphere(radius: f64, sub: u32) -> IndexedMesh {
    let s = shapes::icosphere(Point3::ZERO, radius, sub);
    IndexedMesh::new(s.positions, s.triangles).unwrap()
}

fn torus(big: f64, small: f64, nu: u32, nv: u32) -> IndexedMesh {
    let mut p = Vec::new();
    for i in 0..nu {
        let u = f64::from(i) / f64::from(nu) * std::f64::consts::TAU;
        for j in 0..nv {
            let v = f64::from(j) / f64::from(nv) * std::f64::consts::TAU;
            let r = big + small * v.cos();
            p.push(Point3::new(r * u.cos(), r * u.sin(), small * v.sin()));
        }
    }
    let id = |i: u32, j: u32| (i % nu) * nv + (j % nv);
    let mut t = Vec::new();
    for i in 0..nu {
        for j in 0..nv {
            let (a, b, c, d) = (id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1));
            t.push([a, b, c]);
            t.push([a, c, d]);
        }
    }
    IndexedMesh::new(p, t).unwrap()
}

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
}

/// Mean edge length.
fn mean_edge(m: &IndexedMesh) -> f64 {
    let (mut s, mut n) = (0.0, 0usize);
    for t in &m.tris {
        for k in 0..3 {
            s += m.positions[t[k] as usize].distance(m.positions[t[(k + 1) % 3] as usize]);
            n += 1;
        }
    }
    s / n as f64
}

/// Moves `count` random vertices sideways by `shift` mean edge lengths (and slightly inwards),
/// which folds the faces around them over their neighbours.
fn fold_vertices(m: &IndexedMesh, count: usize, shift: f64, seed: u64) -> IndexedMesh {
    fold_vertices_where(m, count, shift, seed, &|_| true)
}

/// [`fold_vertices`] restricted to vertices for which `allow` holds.
fn fold_vertices_where(
    m: &IndexedMesh,
    count: usize,
    shift: f64,
    seed: u64,
    allow: &dyn Fn(Point3) -> bool,
) -> IndexedMesh {
    let mut out = m.clone();
    let e = mean_edge(m);
    let mut rng = Rng(seed);
    let c = {
        let b = m.aabb();
        (b.min + b.max) * 0.5
    };
    for _ in 0..count {
        let v = (rng.next() % m.positions.len() as u64) as usize;
        let p = out.positions[v];
        if !allow(p) {
            continue;
        }
        let radial = (p - c).normalize_or_zero();
        let mut t = Vec3::new(rng.f() - 0.5, rng.f() - 0.5, rng.f() - 0.5);
        t -= radial * t.dot(radial);
        let t = t.normalize_or_zero();
        out.positions[v] = p + t * (shift * e) - radial * (0.8 * e);
    }
    out
}

/// First seed from `seed` on whose fold actually self-intersects.
fn bad_mesh(base: &IndexedMesh, count: usize, shift: f64, seed: u64) -> IndexedMesh {
    (seed..seed + 200)
        .map(|s| fold_vertices(base, count, shift, s))
        .find(|m| !pairs(m).is_empty())
        .expect("some seed self-intersects")
}

/// Largest distance from a vertex of `out` to the surface of `base`, and from a vertex of `base`
/// to `out`, ignoring the vertices that `folded` moved (a moved vertex that took part in no
/// intersection legitimately stays as a spike; one that did is removed together with its
/// neighbourhood). What is measured is the fidelity of the refilled patches.
fn patch_deviation(base: &IndexedMesh, folded: &IndexedMesh, out: &IndexedMesh) -> f64 {
    use std::collections::HashSet;
    let key = |p: Point3| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
    let moved: HashSet<[u64; 3]> = base
        .positions
        .iter()
        .zip(&folded.positions)
        .filter(|(a, b)| a != b)
        .map(|(_, b)| key(*b))
        .collect();
    let moved_idx: HashSet<usize> = base
        .positions
        .iter()
        .zip(&folded.positions)
        .enumerate()
        .filter(|(_, (a, b))| a != b)
        .map(|(i, _)| i)
        .collect();
    let sb = MeshSpatial::build(Arc::new(base.clone()), &BvhOpts::default(), &ctx()).unwrap();
    let so = MeshSpatial::build(Arc::new(out.clone()), &BvhOpts::default(), &ctx()).unwrap();
    let mut worst = 0.0f64;
    for p in out.positions.iter().filter(|p| !moved.contains(&key(**p))) {
        worst = worst.max(sb.closest_point(*p, 1e9).unwrap().dist_sq.sqrt());
    }
    for (i, p) in base.positions.iter().enumerate() {
        if !moved_idx.contains(&i) {
            worst = worst.max(so.closest_point(*p, 1e9).unwrap().dist_sq.sqrt());
        }
    }
    worst
}

/// One-sided distance output -> input: vertices and face centres of `out` to the surface of
/// `m`.
fn one_sided(out: &IndexedMesh, m: &IndexedMesh) -> f64 {
    let sp = MeshSpatial::build(Arc::new(m.clone()), &BvhOpts::default(), &ctx()).unwrap();
    let mut worst = 0.0f64;
    let mut probe = |p: Point3| {
        if let Some(h) = sp.closest_point(p, f64::INFINITY) {
            worst = worst.max(h.dist_sq.sqrt());
        }
    };
    for p in &out.positions {
        probe(*p);
    }
    for t in &out.tris {
        let q = t.map(|i| out.positions[i as usize]);
        probe((q[0] + q[1] + q[2]) / 3.0);
    }
    worst
}

fn watertight(m: &IndexedMesh) -> bool {
    analyze(m, &AnalyzeOpts::default(), &ctx())
        .unwrap()
        .is_watertight()
}

#[test]
fn clean_mesh_is_returned_unchanged() {
    let s = icosphere(10.0, 3);
    let (out, rep) = remove_self_intersections_with(&s, &Default::default(), &ctx()).unwrap();
    assert_eq!(out, s);
    assert_eq!((rep.pairs_before, rep.pairs_after, rep.rounds), (0, 0, 0));
}

#[test]
fn folded_sphere_becomes_intersection_free() {
    let s = icosphere(10.0, 4);
    let bad = bad_mesh(&s, 12, 4.0, 7);
    let before = pairs(&bad).len();
    assert!(before > 0, "precondition: the fold must self-intersect");
    let (out, rep) = remove_self_intersections_with(&bad, &Default::default(), &ctx()).unwrap();
    out.validate().unwrap();
    eprintln!("folded sphere: {rep:?}");
    assert_eq!(rep.pairs_before, before);
    assert_eq!(rep.pairs_after, 0, "{rep:?}");
    assert!(pairs(&out).is_empty());
    assert!(watertight(&out));
    let h = patch_deviation(&s, &bad, &out);
    eprintln!("sphere patch deviation: {h}");
    assert!(h <= 0.01 * diag(&s), "hausdorff {h}");
}

#[test]
fn folded_torus_becomes_intersection_free() {
    let t = torus(8.0, 3.0, 60, 30);
    let bad = bad_mesh(&t, 6, 3.0, 11);
    let before = pairs(&bad).len();
    assert!(before > 0, "precondition");
    let (out, rep) = remove_self_intersections_with(&bad, &Default::default(), &ctx()).unwrap();
    eprintln!("folded torus: {rep:?}");
    assert_eq!(rep.pairs_after, 0, "{rep:?}");
    assert!(watertight(&out));
    let r = analyze(&out, &AnalyzeOpts::default(), &ctx()).unwrap();
    assert_eq!(r.shells.len(), 1);
    let h = patch_deviation(&t, &bad, &out);
    eprintln!("torus patch deviation: {h}");
    assert!(h <= 0.01 * diag(&t), "hausdorff {h}");
}

/// Two closed shells that overlap: not fixable locally, but the result must stay watertight,
/// not get worse, and report what is left.
#[test]
fn overlapping_spheres_never_get_worse() {
    let a = icosphere(10.0, 3);
    let mut b = icosphere(10.0, 3);
    for p in &mut b.positions {
        p.x += 12.0;
    }
    let mut m = a;
    m.append(&b).unwrap();
    let before = pairs(&m).len();
    assert!(before > 0);
    let (out, left) = remove_self_intersections(&m, 4, &ctx()).unwrap();
    out.validate().unwrap();
    assert!(left <= before);
    assert_eq!(left, pairs(&out).len());
    assert!(watertight(&out));
}

#[test]
fn only_cut_holes_are_filled() {
    // A fold on a sphere that also has a pre-existing hole far away: the hole stays open.
    let s = icosphere(10.0, 4);
    let hole_center = Point3::new(0.0, 0.0, -10.0);
    let bad = (5..300)
        .map(|sd| fold_vertices_where(&s, 6, 4.0, sd, &|p| p.distance(hole_center) > 8.0))
        .find(|m| !pairs(m).is_empty())
        .unwrap();
    let keep: Vec<[u32; 3]> = bad
        .tris
        .iter()
        .copied()
        .filter(|t| {
            let c = t
                .map(|i| bad.positions[i as usize])
                .into_iter()
                .sum::<Point3>()
                / 3.0;
            c.distance(hole_center) > 2.0
        })
        .collect();
    let holed = IndexedMesh::new(bad.positions.clone(), keep).unwrap();
    let loops_before = analyze(&holed, &AnalyzeOpts::default(), &ctx())
        .unwrap()
        .boundary_loops
        .len();
    assert_eq!(loops_before, 1);
    let (out, rep) = remove_self_intersections_with(&holed, &Default::default(), &ctx()).unwrap();
    assert_eq!(rep.pairs_after, 0, "{rep:?}");
    let loops_after = analyze(&out, &AnalyzeOpts::default(), &ctx())
        .unwrap()
        .boundary_loops
        .len();
    assert_eq!(loops_after, 1, "the old hole must stay open");
}

#[test]
fn deterministic_and_cancellable() {
    let bad = bad_mesh(&icosphere(10.0, 4), 8, 4.0, 3);
    let (a, _) = remove_self_intersections(&bad, 4, &ctx()).unwrap();
    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(1)
        .build()
        .unwrap();
    let (b, _) = pool
        .install(|| remove_self_intersections(&bad, 4, &ctx()))
        .unwrap();
    assert_eq!(a.content_hash(), b.content_hash());

    let token = CancelToken::new();
    token.cancel();
    let r = remove_self_intersections(&bad, 4, &JobCtx::with_token(token));
    assert!(matches!(r, Err(MeshError::Cancelled)));
}

/// A deviation budget no patch can meet rejects every round and leaves the mesh as it was.
#[test]
fn deviation_budget_is_enforced() {
    let bad = bad_mesh(&icosphere(10.0, 4), 12, 4.0, 7);
    let opts = RemoveSelfIntersectionsOpts {
        max_deviation: Some(1e-9),
        max_deviation_edges: 0.0,
        ..Default::default()
    };
    let (out, rep) = remove_self_intersections_with(&bad, &opts, &ctx()).unwrap();
    assert_eq!(out, bad);
    assert!(rep.rejected_for_deviation > 0, "{rep:?}");
    assert_eq!(rep.pairs_after, rep.pairs_before);
    // A generous budget accepts, and the spent budget bounds the measured distance.
    let opts = RemoveSelfIntersectionsOpts {
        max_deviation: Some(0.05),
        ..Default::default()
    };
    let (out, rep) = remove_self_intersections_with(&bad, &opts, &ctx()).unwrap();
    assert_eq!(rep.pairs_after, 0, "{rep:?}");
    assert!(rep.deviation <= 0.05 * diag(&bad) + 1e-9);
    assert!(one_sided(&out, &bad) <= rep.deviation + 1e-9);
}

#[test]
fn zero_rounds_changes_nothing() {
    let bad = bad_mesh(&icosphere(10.0, 3), 6, 4.0, 9);
    let n = pairs(&bad).len();
    let (out, left) = remove_self_intersections(&bad, 0, &ctx()).unwrap();
    assert_eq!(out, bad);
    assert_eq!(left, n);
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(24))]

    /// Random folds on a sphere or torus: never worse, watertight stays watertight, and the
    /// reported remainder is exact.
    #[test]
    fn random_folds(seed in 0u64..10_000, count in 1usize..20, shift in 3.0f64..5.0, on_torus in any::<bool>()) {
        let base = if on_torus { torus(8.0, 3.0, 48, 24) } else { icosphere(10.0, 3) };
        let bad = fold_vertices(&base, count, shift, seed);
        let before = pairs(&bad).len();
        let (out, rep) = remove_self_intersections_with(&bad, &RemoveSelfIntersectionsOpts::default(), &ctx()).unwrap();
        out.validate().unwrap();
        prop_assert_eq!(rep.pairs_before, before);
        prop_assert!(rep.pairs_after <= before);
        prop_assert_eq!(rep.pairs_after, pairs(&out).len());
        prop_assert!(watertight(&out), "watertight input became open");
    }
}

#[test]
fn golden_folded_sphere() {
    let bad = bad_mesh(&icosphere(10.0, 4), 12, 4.0, 7);
    let (out, _) = remove_self_intersections(&bad, 6, &ctx()).unwrap();
    amg_testkit::assert_golden!("mesh", "selfint_repair_sphere_r10_folds", soup(&out));
}

#[test]
fn golden_folded_torus() {
    let bad = bad_mesh(&torus(8.0, 3.0, 60, 30), 6, 3.0, 11);
    let (out, _) = remove_self_intersections(&bad, 6, &ctx()).unwrap();
    amg_testkit::assert_golden!("mesh", "selfint_repair_torus_folds", soup(&out));
}

// ------------------------------------------------------------------------------- corpus

/// The repair pipeline (MESH-04) up to, but not including, hole filling and the
/// self-intersection removal under test: weld, degenerate and duplicate faces, non-manifold
/// elements, debris shells and orientation.
fn pre_clean(m: IndexedMesh) -> IndexedMesh {
    let opts = RepairOpts {
        fill_holes: None,
        remove_self_intersections: false,
        ..RepairOpts::default()
    };
    repair(&m, &opts, &ctx()).unwrap().0
}

/// At least 85 % of the corpus models flagged `self_intersections` and `expect_repairable` become
/// intersection-free; a watertight model stays watertight. The distance of the result to the
/// input is reported (see the S07 spec: the 1 % Hausdorff bar is not met on this corpus).
#[test]
fn corpus_self_intersecting_models() {
    const MAX_FACES: usize = 60_000;
    let manifest = Manifest::load().unwrap();
    let root = corpus_root();
    let (mut tested, mut free, mut free_strict, mut skipped_clean) =
        (0usize, 0usize, 0usize, 0usize);
    let mut fails = Vec::new();
    for e in manifest.meshes.iter() {
        if !e.defects.contains(&Defect::SelfIntersections)
            || !e.expect_repairable
            || e.faces > MAX_FACES
        {
            continue;
        }
        let path = match e.locate(&root) {
            FileStatus::Ready(p) => p,
            FileStatus::LfsPointer(p) => {
                assert!(
                    !require_testdata(),
                    "{} is an un-fetched git-LFS pointer",
                    p.display()
                );
                continue;
            }
            _ => continue,
        };
        let Ok(raw) = read_mesh(&path) else {
            continue;
        };
        let m = pre_clean(raw);
        let before_report = analyze(&m, &AnalyzeOpts::default(), &ctx()).unwrap();
        let n_before = pairs(&m).len();
        if n_before == 0 {
            skipped_clean += 1;
            continue;
        }
        let started = std::time::Instant::now();
        let (out, rep) =
            remove_self_intersections_with(&m, &RemoveSelfIntersectionsOpts::default(), &ctx())
                .unwrap();
        let ms = started.elapsed().as_secs_f64() * 1e3;
        tested += 1;
        let after = analyze(&out, &AnalyzeOpts::default(), &ctx()).unwrap();
        let dev = one_sided(&out, &m);
        eprintln!(
            "CORPUS {}: {} faces, pairs {} -> {}, rounds {}/{}, out->in {:.3} % diag ({:.2} edges), wt {} -> {}, {:.0} ms",
            e.id,
            m.tris.len(),
            n_before,
            rep.pairs_after,
            rep.accepted_rounds,
            rep.rounds,
            dev / diag(&m) * 100.0,
            dev / mean_edge(&m),
            before_report.is_watertight(),
            after.is_watertight(),
            ms
        );
        eprintln!(
            "   rejected: {} deviation, {} not better, {} build failed",
            rep.rejected_for_deviation, rep.rejected_not_better, rep.build_failed
        );
        if before_report.is_watertight() && !after.is_watertight() {
            fails.push(format!("{}: watertight became open", e.id));
        }
        if rep.pairs_after == 0 {
            free += 1;
            if dev <= 0.01 * diag(&m) {
                free_strict += 1;
            }
        }
    }
    eprintln!(
        "corpus: {free}/{tested} intersection-free ({free_strict} of them within 1 % of the diagonal; {skipped_clean} already clean)"
    );
    assert!(fails.is_empty(), "{fails:#?}");
    if tested == 0 {
        return; // clone without `git lfs pull`
    }
    assert!(tested >= 20, "only {tested} corpus models usable");
    // Flagged models that weld / duplicate removal / orientation already cleaned count as done.
    let (done, all) = (free + skipped_clean, tested + skipped_clean);
    eprintln!("corpus: {done}/{all} flagged models intersection-free after cleaning + repair");
    assert!(
        done as f64 >= 0.85 * all as f64,
        "{done}/{all} intersection-free, need >= 85 %"
    );
}
