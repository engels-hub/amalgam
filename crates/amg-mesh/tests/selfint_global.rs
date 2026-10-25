//! MESH-29 tests: global self-intersection removal (cut along the intersection curves, drop the
//! inner sheets).

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::print_stderr
)]

use std::sync::Arc;

use amg_core::Point3;
use amg_jobs::{CancelToken, JobCtx};
use amg_mesh::corpus::{Defect, FileStatus, Manifest, corpus_root, read_mesh};
use amg_mesh::{
    AnalyzeOpts, AutoSelfIntersectionOpts, BvhOpts, FaceId, GlobalSelfIntersectionOpts,
    IndexedMesh, MeshError, MeshSpatial, RemoveSelfIntersectionsOpts, SelfIntersectOpts,
    SelfIntersectionMethod, WeldOpts, analyze, remove_self_intersections_auto,
    remove_self_intersections_global, remove_self_intersections_with, self_intersections,
};
use amg_testkit::{TriSoup, shapes};

fn ctx() -> JobCtx {
    JobCtx::detached()
}

fn require_testdata() -> bool {
    std::env::var("AMG_REQUIRE_TESTDATA").is_ok_and(|v| v == "1")
}

fn pairs(m: &IndexedMesh) -> Vec<[FaceId; 2]> {
    let s = MeshSpatial::build(Arc::new(m.clone()), &BvhOpts::default(), &ctx()).unwrap();
    self_intersections(&s, &SelfIntersectOpts::default(), &ctx()).unwrap()
}

fn diag(m: &IndexedMesh) -> f64 {
    let b = m.aabb();
    (b.max - b.min).length()
}

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

fn watertight(m: &IndexedMesh) -> bool {
    analyze(m, &AnalyzeOpts::default(), &ctx())
        .unwrap()
        .is_watertight()
}

fn soup(m: &IndexedMesh) -> TriSoup<'_> {
    TriSoup {
        positions: &m.positions,
        triangles: &m.tris,
    }
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

fn icosphere(c: Point3, radius: f64, sub: u32) -> IndexedMesh {
    let s = shapes::icosphere(c, radius, sub);
    IndexedMesh::new(s.positions, s.triangles).unwrap()
}

fn cube(min: Point3, size: f64) -> IndexedMesh {
    let s = shapes::cube(min, size);
    IndexedMesh::new(s.positions, s.triangles).unwrap()
}

/// Two shells in one mesh.
fn merged(a: &IndexedMesh, b: &IndexedMesh) -> IndexedMesh {
    let mut m = a.clone();
    m.append(b).unwrap();
    m
}

/// `V - E + F`.
fn euler(m: &IndexedMesh) -> i64 {
    let mut e = std::collections::HashSet::new();
    for t in &m.tris {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            e.insert((a.min(b), a.max(b)));
        }
    }
    m.positions.len() as i64 - e.len() as i64 + m.tris.len() as i64
}

fn run(m: &IndexedMesh) -> (IndexedMesh, amg_mesh::GlobalSelfIntersectionReport) {
    remove_self_intersections_global(m, &GlobalSelfIntersectionOpts::default(), &ctx()).unwrap()
}

#[test]
fn clean_mesh_is_returned_unchanged() {
    let s = icosphere(Point3::ZERO, 10.0, 3);
    let (out, rep) = run(&s);
    assert_eq!(out, s);
    assert_eq!((rep.pairs_before, rep.pairs_after), (0, 0));
}

#[test]
fn overlapping_spheres_become_a_union() {
    let a = icosphere(Point3::ZERO, 10.0, 3);
    let b = icosphere(Point3::new(7.3, 1.1, 0.4), 8.0, 3);
    let m = merged(&a, &b);
    assert!(!pairs(&m).is_empty());
    let (out, rep) = run(&m);
    assert!(rep.accepted, "{rep:?}");
    assert_eq!(pairs(&out).len(), 0, "{rep:?}");
    assert!(watertight(&out));
    assert_eq!(euler(&out), 2, "union of two overlapping balls is a sphere");
    let (va, vb, vo) = (a.signed_volume(), b.signed_volume(), out.signed_volume());
    assert!(vo > va.max(vb) && vo < va + vb);
    // The output lies on the input surface (up to the general-position move).
    assert!(one_sided(&out, &m) < 1e-8 * diag(&m));
    assert!(out.tris.len() < m.tris.len());
}

#[test]
fn overlapping_cubes_become_a_union() {
    let a = cube(Point3::ZERO, 10.0);
    let b = cube(Point3::new(4.3, 5.1, 6.7), 9.0);
    let m = merged(&a, &b);
    let (out, rep) = run(&m);
    assert_eq!(pairs(&out).len(), 0, "{rep:?}");
    assert!(watertight(&out));
    assert_eq!(euler(&out), 2);
    // |A u B| = |A| + |B| - |A n B|; the intersection is 5.7 x 4.9 x 3.3.
    let want = 1000.0 + 729.0 - 5.7 * 4.9 * 3.3;
    assert!(
        (out.signed_volume() - want).abs() < 1e-6 * want,
        "{}",
        out.signed_volume()
    );
}

#[test]
fn three_spheres_chain() {
    let a = icosphere(Point3::ZERO, 10.0, 3);
    let b = icosphere(Point3::new(9.0, 0.7, 0.3), 8.0, 3);
    let c = icosphere(Point3::new(17.8, -1.9, 1.3), 7.0, 3);
    let m = merged(&merged(&a, &b), &c);
    let (out, rep) = run(&m);
    assert_eq!(pairs(&out).len(), 0, "{rep:?}");
    assert!(watertight(&out));
    assert_eq!(euler(&out), 2);
    assert!(one_sided(&out, &m) < 1e-8 * diag(&m));
}

/// Cubes that touch exactly along a face have no clean answer to "do they intersect"; the
/// general-position move turns the contact into a clear gap or a clear overlap.
#[test]
fn cubes_touching_in_a_face_are_resolved() {
    let a = cube(Point3::ZERO, 10.0);
    let b = cube(Point3::new(10.0, 0.0, 0.0), 10.0);
    let m = merged(&a, &b);
    assert!(!pairs(&m).is_empty(), "exact contact is reported");
    let (out, rep) = run(&m);
    assert_eq!(pairs(&out).len(), 0, "{rep:?}");
    assert!(watertight(&out));
    assert!((out.signed_volume() - 2000.0).abs() < 1e-5 * 2000.0);
}

/// Several shells overlapping a bigger one, as a model assembled from parts.
#[test]
fn sphere_with_two_cubes_through_it() {
    let s = icosphere(Point3::ZERO, 10.0, 3);
    let c1 = cube(Point3::new(3.1, -2.2, -1.3), 12.0);
    let c2 = cube(Point3::new(-14.7, -3.3, 1.9), 9.0);
    let m = merged(&merged(&s, &c1), &c2);
    let (out, rep) = run(&m);
    assert_eq!(pairs(&out).len(), 0, "{rep:?}");
    assert!(watertight(&out), "{rep:?}");
    assert!(out.signed_volume() > s.signed_volume());
}

#[test]
fn nested_shell_without_contact_is_left_alone() {
    let a = icosphere(Point3::ZERO, 10.0, 3);
    let inner = icosphere(Point3::ZERO, 3.0, 2);
    let b = icosphere(Point3::new(12.0, 0.0, 0.0), 6.0, 3);
    let m = merged(&merged(&a, &inner), &b);
    let (out, _) = run(&m);
    assert_eq!(pairs(&out).len(), 0);
    // The nested shell touches nothing, so it is still there.
    assert!(out.positions.iter().any(|p| p.length() < 3.5));
}

/// The fallback entry point: overlapping solids go global, a clean mesh is left alone.
#[test]
fn auto_picks_the_method() {
    let a = icosphere(Point3::ZERO, 10.0, 3);
    let b = icosphere(Point3::new(7.3, 1.1, 0.4), 8.0, 3);
    let m = merged(&a, &b);
    let (out, rep) =
        remove_self_intersections_auto(&m, &AutoSelfIntersectionOpts::default(), &ctx()).unwrap();
    assert_eq!(rep.method, SelfIntersectionMethod::Global, "{rep:?}");
    assert_eq!((rep.pairs_before > 0, rep.pairs_after), (true, 0));
    assert_eq!(pairs(&out).len(), 0);
    assert!(watertight(&out));

    let (same, rep) =
        remove_self_intersections_auto(&a, &AutoSelfIntersectionOpts::default(), &ctx()).unwrap();
    assert_eq!(rep.method, SelfIntersectionMethod::Unchanged);
    assert_eq!(same, a);
}

#[test]
fn deterministic_idempotent_and_cancellable() {
    let a = icosphere(Point3::ZERO, 10.0, 3);
    let b = icosphere(Point3::new(7.3, 1.1, 0.4), 8.0, 3);
    let m = merged(&a, &b);
    let (x, _) = run(&m);
    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(1)
        .build()
        .unwrap();
    let (y, _) = pool.install(|| run(&m));
    assert_eq!(x.content_hash(), y.content_hash());
    let (z, rep) = run(&x);
    assert_eq!(z, x);
    assert_eq!(rep.pairs_before, 0);
    let t = CancelToken::new();
    t.cancel();
    let r = remove_self_intersections_global(
        &m,
        &GlobalSelfIntersectionOpts::default(),
        &JobCtx::with_token(t),
    );
    assert!(matches!(r, Err(MeshError::Cancelled)));
}

#[test]
fn golden_two_spheres() {
    let a = icosphere(Point3::ZERO, 10.0, 3);
    let b = icosphere(Point3::new(7.3, 1.1, 0.4), 8.0, 3);
    let (out, _) = run(&merged(&a, &b));
    amg_testkit::assert_golden!("mesh", "selfint_global_two_spheres", soup(&out));
}

#[test]
fn golden_two_cubes() {
    let a = cube(Point3::ZERO, 10.0);
    let b = cube(Point3::new(4.3, 5.1, 6.7), 9.0);
    let (out, _) = run(&merged(&a, &b));
    amg_testkit::assert_golden!("mesh", "selfint_global_two_cubes", soup(&out));
}

// ------------------------------------------------------------------------------- corpus

/// Makes the orientation consistent per edge-connected component (edges shared by more than
/// two faces are not crossed) and outward (positive volume); a stand-in for the repair
/// pipeline's orient step.
fn orient(m: &mut IndexedMesh) {
    use std::collections::HashMap;
    let mut edge: HashMap<(u32, u32), Vec<u32>> = HashMap::new();
    for (f, t) in m.tris.iter().enumerate() {
        for c in 0..3 {
            let (a, b) = (t[c], t[(c + 1) % 3]);
            edge.entry((a.min(b), a.max(b))).or_default().push(f as u32);
        }
    }
    let mut seen = vec![false; m.tris.len()];
    for seed in 0..m.tris.len() {
        if seen[seed] {
            continue;
        }
        seen[seed] = true;
        let mut stack = vec![seed];
        let mut comp = vec![seed];
        while let Some(f) = stack.pop() {
            let t = m.tris[f];
            for c in 0..3 {
                let (a, b) = (t[c], t[(c + 1) % 3]);
                let nb = &edge[&(a.min(b), a.max(b))];
                if nb.len() != 2 {
                    continue;
                }
                let g = if nb[0] as usize == f { nb[1] } else { nb[0] } as usize;
                if seen[g] {
                    continue;
                }
                seen[g] = true;
                let u = m.tris[g];
                // Consistent when g runs b -> a.
                let same = (0..3).any(|k| u[k] == a && u[(k + 1) % 3] == b);
                if same {
                    m.tris[g].swap(1, 2);
                }
                stack.push(g);
                comp.push(g);
            }
        }
        let vol: f64 = comp
            .iter()
            .map(|&f| {
                let [a, b, c] = m.tris[f].map(|i| m.positions[i as usize]);
                a.dot(b.cross(c))
            })
            .sum();
        if vol < 0.0 {
            for &f in &comp {
                m.tris[f].swap(1, 2);
            }
        }
    }
}

/// What `repair` will do before this step: weld, drop degenerate and duplicate faces, orient
/// each shell consistently and outward.
fn pre_clean(mut m: IndexedMesh) -> IndexedMesh {
    m.weld(WeldOpts::default()).unwrap();
    for _ in 0..2 {
        let r = analyze(&m, &AnalyzeOpts::default(), &ctx()).unwrap();
        let mut drop = vec![false; m.tris.len()];
        for f in r.degenerate_faces.iter().chain(&r.duplicate_faces) {
            drop[f.0 as usize] = true;
        }
        m.tris = m
            .tris
            .iter()
            .zip(&drop)
            .filter(|(_, d)| !**d)
            .map(|(t, _)| *t)
            .collect();
        m.groups = amg_mesh::FaceGroups::new(m.tris.len());
        m.attrs = amg_mesh::MeshAttrs::default();
    }
    orient(&mut m);
    m
}

/// The corpus models flagged `self_intersections` and `expect_repairable` (at most 60 k faces)
/// that still intersect after the cleaning a repair pipeline does first. `AMG_ONLY=<id suffix>`
/// restricts the run to one model.
fn flagged_models() -> Vec<(String, IndexedMesh)> {
    const MAX_FACES: usize = 60_000;
    let manifest = Manifest::load().unwrap();
    let root = corpus_root();
    let mut out = Vec::new();
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
        if std::env::var("AMG_ONLY").is_ok_and(|v| !e.id.ends_with(&v)) {
            continue;
        }
        let Ok(raw) = read_mesh(&path) else {
            continue;
        };
        let m = pre_clean(raw);
        if !pairs(&m).is_empty() {
            out.push((e.id.clone(), m));
        }
    }
    out
}

/// Corpus models that MESH-21 leaves unrepaired or repairs with a Hausdorff distance above 1 % of
/// the bounding-box diagonal: at least 80 % become intersection-free within 1 % of the diagonal;
/// a watertight model stays watertight.
#[test]
fn corpus_models_left_by_local_repair() {
    let models = flagged_models();
    let (mut hard, mut good) = (0usize, 0usize);
    let mut fails = Vec::new();
    for (id, m) in &models {
        let n_before = pairs(m).len();
        let d = diag(m);
        let wt_before = watertight(m);
        let (local, lrep) =
            remove_self_intersections_with(m, &RemoveSelfIntersectionsOpts::default(), &ctx())
                .unwrap();
        let ldev = one_sided(&local, m);
        if lrep.pairs_after == 0 && ldev <= 0.01 * d {
            continue; // MESH-21 handles it
        }
        hard += 1;
        let started = std::time::Instant::now();
        let (out, rep) = run(m);
        let ms = started.elapsed().as_secs_f64() * 1e3;
        let dev = one_sided(&out, m);
        let wt_after = watertight(&out);
        eprintln!(
            "HARD {id}: {} faces, local pairs {n_before}->{} ({:.2} % diag), global pairs ->{} (cut {} / not cut {}, fallback {}, drop {} patches/{} faces, after-drop {}, kept {:?}), out->in {:.4} % diag ({:.2} edges), watertight {wt_before}->{wt_after}, {ms:.0} ms",
            m.tris.len(),
            lrep.pairs_after,
            ldev / d * 100.0,
            rep.pairs_after,
            rep.pairs_cut,
            rep.pairs_shared_edge + rep.pairs_degenerate + rep.pairs_other,
            rep.faces_fallback,
            rep.patches_dropped,
            rep.faces_dropped,
            rep.pairs_after_drop,
            rep.candidate,
            dev / d * 100.0,
            dev / mean_edge(m),
        );
        if wt_before && !wt_after {
            fails.push(format!("{id}: watertight became open"));
        }
        if rep.pairs_after == 0 && dev <= 0.01 * d {
            good += 1;
        }
    }
    eprintln!(
        "corpus: {good}/{hard} hard models are intersection-free within 1 % of the diagonal ({} flagged with pairs)",
        models.len()
    );
    assert!(fails.is_empty(), "{fails:#?}");
    if models.is_empty() {
        return; // clone without `git lfs pull`
    }
    assert!(hard >= 1, "no model left by the local repair");
    assert!(
        good as f64 >= 0.8 * hard as f64,
        "{good}/{hard} within the bar, need >= 80 %"
    );
}

/// The fallback entry point is never worse than the local refill alone on the whole flagged
/// set, and never opens a watertight model.
#[test]
fn corpus_auto_never_worse_than_local() {
    let models = flagged_models();
    let (mut local_free, mut auto_free, mut auto_free_strict) = (0usize, 0usize, 0usize);
    for (id, m) in &models {
        let (_, lrep) =
            remove_self_intersections_with(m, &RemoveSelfIntersectionsOpts::default(), &ctx())
                .unwrap();
        let (out, rep) =
            remove_self_intersections_auto(m, &AutoSelfIntersectionOpts::default(), &ctx())
                .unwrap();
        assert!(
            rep.pairs_after <= lrep.pairs_after,
            "{id}: {} pairs, local {}",
            rep.pairs_after,
            lrep.pairs_after
        );
        assert_eq!(pairs(&out).len(), rep.pairs_after, "{id}");
        if watertight(m) {
            assert!(watertight(&out), "{id}: watertight became open");
        }
        local_free += usize::from(lrep.pairs_after == 0);
        auto_free += usize::from(rep.pairs_after == 0);
        auto_free_strict +=
            usize::from(rep.pairs_after == 0 && one_sided(&out, m) <= 0.01 * diag(m));
    }
    eprintln!(
        "corpus auto: {auto_free}/{} intersection-free ({auto_free_strict} within 1 % of the diagonal), local alone {local_free}",
        models.len()
    );
}
