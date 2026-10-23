//! MESH-06 tests: isotropic remeshing.

#![allow(clippy::unwrap_used, clippy::float_cmp, clippy::cast_precision_loss)]

mod common;

use std::collections::{HashMap, HashSet};

use amg_core::Point3;
use amg_jobs::{CancelToken, JobCtx};
use amg_mesh::corpus::{Defect, FileStatus, Manifest, corpus_root, read_mesh};
use amg_mesh::{
    EdgeLength, FaceGroupInfo, FaceId, GroupOrigin, IndexedMesh, MeshError, RemeshOpts, Sizing,
    WeldOpts, remesh,
};
use amg_testkit::{TriSoup, hausdorff, metrics, shapes};

fn soup(m: &IndexedMesh) -> TriSoup<'_> {
    TriSoup {
        positions: &m.positions,
        triangles: &m.tris,
    }
}

fn icosphere(sub: u32) -> IndexedMesh {
    let s = shapes::icosphere(Point3::ZERO, 10.0, sub);
    IndexedMesh::new(s.positions, s.triangles).unwrap()
}

fn displaced_sphere(sub: u32) -> IndexedMesh {
    let mut m = icosphere(sub);
    for p in &mut m.positions {
        let u = *p / 10.0;
        *p *= 1.0 + 0.08 * (3.0 * u.x).sin() * (2.0 * u.y).cos() + 0.04 * (5.0 * u.z).sin();
    }
    m
}

/// Cube of edge `size`, every face split into `n x n` quads.
fn subdivided_cube(n: u32, size: f64) -> IndexedMesh {
    let mut tris = Vec::new();
    let axes = [
        (Point3::X, Point3::Y, Point3::Z),
        (Point3::Y, Point3::Z, Point3::X),
        (Point3::Z, Point3::X, Point3::Y),
    ];
    for (u, v, w) in axes {
        for (origin, flip) in [(Point3::ZERO, true), (w * size, false)] {
            let at = |i: u32, j: u32| {
                origin
                    + u * size * f64::from(i) / f64::from(n)
                    + v * size * f64::from(j) / f64::from(n)
            };
            for i in 0..n {
                for j in 0..n {
                    let (a, b, c, d) = (at(i, j), at(i + 1, j), at(i + 1, j + 1), at(i, j + 1));
                    if flip {
                        tris.push([a, c, b]);
                        tris.push([a, d, c]);
                    } else {
                        tris.push([a, b, c]);
                        tris.push([a, c, d]);
                    }
                }
            }
        }
    }
    IndexedMesh::from_soup(&tris, WeldOpts::EXACT).unwrap()
}

fn opts(l: f64) -> RemeshOpts {
    RemeshOpts {
        sizing: Sizing::Uniform(EdgeLength::Absolute(l)),
        ..RemeshOpts::default()
    }
}

fn run(m: &IndexedMesh, o: &RemeshOpts) -> IndexedMesh {
    let out = remesh(m, o, &JobCtx::detached()).unwrap();
    out.validate().unwrap();
    out
}

fn unique_edges(m: &IndexedMesh) -> HashSet<(u32, u32)> {
    let mut e = HashSet::new();
    for t in &m.tris {
        for k in 0..3 {
            e.insert((t[k].min(t[(k + 1) % 3]), t[k].max(t[(k + 1) % 3])));
        }
    }
    e
}

/// Fractions of triangles with min angle >= 30 deg and of edges within [0.8 L, 1.33 L].
fn quality(m: &IndexedMesh, l: f64) -> (f64, f64) {
    let mut good_tri = 0usize;
    for t in &m.tris {
        let p = t.map(|i| m.positions[i as usize]);
        let mut min = f64::MAX;
        for k in 0..3 {
            let (a, b, c) = (p[k], p[(k + 1) % 3], p[(k + 2) % 3]);
            let (u, v) = ((b - a).normalize_or_zero(), (c - a).normalize_or_zero());
            min = min.min(u.dot(v).clamp(-1.0, 1.0).acos());
        }
        if min >= 30f64.to_radians() {
            good_tri += 1;
        }
    }
    let edges = unique_edges(m);
    let good_edge = edges
        .iter()
        .filter(|&&(a, b)| {
            let len = m.positions[a as usize].distance(m.positions[b as usize]);
            (0.8 * l..=1.33 * l).contains(&len)
        })
        .count();
    (
        good_tri as f64 / m.tris.len() as f64,
        good_edge as f64 / edges.len() as f64,
    )
}

fn shells(m: &IndexedMesh) -> usize {
    fn find(p: &mut [usize], mut x: usize) -> usize {
        while p[x] != x {
            p[x] = p[p[x]];
            x = p[x];
        }
        x
    }
    let mut parent: Vec<usize> = (0..m.positions.len()).collect();
    for t in &m.tris {
        for k in 1..3 {
            let (a, b) = (
                find(&mut parent, t[0] as usize),
                find(&mut parent, t[k] as usize),
            );
            parent[a] = b;
        }
    }
    let used: HashSet<usize> = m
        .tris
        .iter()
        .flatten()
        .map(|&v| find(&mut parent, v as usize))
        .collect();
    used.len()
}

/// Checks the Done-when metrics for a closed, feature-free shape.
fn check_smooth_shape(name: &str, m: &IndexedMesh, l: f64) -> IndexedMesh {
    let out = run(m, &opts(l));
    let (tri, edge) = quality(&out, l);
    let h = hausdorff(soup(m), soup(&out), 50_000);
    let (mi, mo) = (metrics(soup(m)), metrics(soup(&out)));
    eprintln!(
        "{name}: {} -> {} tris, min-angle>=30: {tri:.3}, edges in band: {edge:.3}, hausdorff/L {:.3}",
        m.tris.len(),
        out.tris.len(),
        h / l
    );
    assert!(
        tri >= 0.95,
        "{name}: only {tri} of triangles have min angle >= 30"
    );
    assert!(
        edge >= 0.90,
        "{name}: only {edge} of edges in [0.8 L, 1.33 L]"
    );
    assert!(h <= 0.5 * l, "{name}: hausdorff {h} > 0.5 L = {}", 0.5 * l);
    assert_eq!(
        (mo.boundary_edges, mo.non_manifold_edges, mo.euler),
        (mi.boundary_edges, mi.non_manifold_edges, mi.euler),
        "{name}: topology changed"
    );
    assert_eq!(shells(&out), shells(m), "{name}: shell count changed");
    out
}

#[test]
fn sphere_fine_to_coarse() {
    check_smooth_shape("icosphere4", &icosphere(4), 1.5);
}

#[test]
fn sphere_coarse_to_fine() {
    check_smooth_shape("icosphere2", &icosphere(2), 1.0);
}

#[test]
fn displaced_sphere_remeshes() {
    check_smooth_shape("displaced5", &displaced_sphere(5), 0.8);
}

#[test]
fn torus_keeps_genus() {
    check_smooth_shape("torus", &common::torus(60, 40, 30.0, 8.0), 2.5);
}

#[test]
fn anisotropic_torus() {
    check_smooth_shape("aniso_torus", &common::torus(200, 16, 30.0, 8.0), 2.0);
}

#[test]
fn two_shells_stay_two() {
    let mut m = icosphere(3);
    let mut b = icosphere(3);
    b.transform(&amg_core::Transform::from_translation(Point3::new(
        40.0, 0.0, 0.0,
    )));
    m.append(&b).unwrap();
    let out = check_smooth_shape("two_spheres", &m, 1.5);
    assert_eq!(shells(&out), 2);
}

/// Distance from `p` to segment `a`-`b`.
fn seg_dist(p: Point3, a: Point3, b: Point3) -> f64 {
    let d = b - a;
    let t = ((p - a).dot(d) / d.length_squared()).clamp(0.0, 1.0);
    p.distance(a + d * t)
}

#[test]
fn cube_feature_edges_stay_exact() {
    let m = subdivided_cube(10, 10.0);
    let l = 1.7;
    let out = run(&m, &opts(l));
    assert_ne!(out.tris.len(), m.tris.len());
    let (tri, edge) = quality(&out, l);
    eprintln!("cube: {} tris, {tri:.3} {edge:.3}", out.tris.len());
    let bb = out.aabb();
    assert!(bb.min.distance(Point3::ZERO) < 1e-3);
    assert!(bb.max.distance(Point3::splat(10.0)) < 1e-3);
    assert!((out.signed_volume() / 1000.0 - 1.0).abs() < 1e-6);
    let corner = |c: u32| {
        Point3::new(
            f64::from(c & 1) * 10.0,
            f64::from((c >> 1) & 1) * 10.0,
            f64::from((c >> 2) & 1) * 10.0,
        )
    };
    for c in 0..8 {
        assert!(out.positions.iter().any(|p| p.distance(corner(c)) < 1e-3));
    }
    // Every one of the 12 cube edges is covered by mesh edges lying on it.
    let edges = unique_edges(&out);
    for c in 0..8u32 {
        for bit in 0..3 {
            if c & (1 << bit) != 0 {
                continue;
            }
            let (a, b) = (corner(c), corner(c | (1 << bit)));
            let covered: f64 = edges
                .iter()
                .filter_map(|&(x, y)| {
                    let (px, py) = (out.positions[x as usize], out.positions[y as usize]);
                    (seg_dist(px, a, b) < 1e-3 && seg_dist(py, a, b) < 1e-3)
                        .then(|| px.distance(py))
                })
                .sum();
            assert!(
                (covered - 10.0).abs() < 1e-3,
                "edge {c}/{bit} covered {covered}"
            );
        }
    }
    assert!(tri >= 0.95 && edge >= 0.85, "{tri} {edge}");
    let (mi, mo) = (metrics(soup(&m)), metrics(soup(&out)));
    assert_eq!(
        (mo.boundary_edges, mo.non_manifold_edges, mo.euler),
        (0, 0, mi.euler)
    );
}

fn two_group_grid() -> IndexedMesh {
    let mut m = common::grid(40, 40, |i| i % 2 == 0);
    let g = m.groups.alloc(FaceGroupInfo::new(GroupOrigin::User));
    for f in 0..m.face_count() as u32 {
        let t = m.triangle(FaceId(f));
        if (t[0].x + t[1].x + t[2].x) / 3.0 < 20.0 {
            m.groups.set(FaceId(f), g);
        }
    }
    m
}

#[test]
fn group_borders_and_boundary_are_kept() {
    let m = two_group_grid();
    let out = run(&m, &opts(1.5));
    assert_ne!(out.tris.len(), m.tris.len());
    let (tri, edge) = quality(&out, 1.5);
    eprintln!("grid: {} tris, {tri:.3} {edge:.3}", out.tris.len());
    assert!(out.positions.iter().all(|p| p.z == 0.0));
    let mut by_edge = HashMap::<(u32, u32), Vec<u32>>::new();
    for (f, t) in out.tris.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            by_edge
                .entry((a.min(b), a.max(b)))
                .or_default()
                .push(f as u32);
        }
    }
    let (mut border_len, mut perimeter) = (0.0, 0.0);
    for (&(a, b), fs) in &by_edge {
        let (pa, pb) = (out.positions[a as usize], out.positions[b as usize]);
        if fs.len() == 1 {
            perimeter += pa.distance(pb);
            let on_rim =
                |p: Point3| p.x.min(40.0 - p.x).abs() < 1e-9 || p.y.min(40.0 - p.y).abs() < 1e-9;
            assert!(on_rim(pa) && on_rim(pb));
        } else if out.groups.group_of(FaceId(fs[0])) != out.groups.group_of(FaceId(fs[1])) {
            assert!((pa.x - 20.0).abs() < 1e-9 && (pb.x - 20.0).abs() < 1e-9);
            border_len += pa.distance(pb);
        }
    }
    assert!((border_len - 40.0).abs() < 1e-9, "border {border_len}");
    assert!((perimeter - 160.0).abs() < 1e-9, "perimeter {perimeter}");
    assert!((out.area() - 1600.0).abs() < 1e-6);
    // Every face keeps the group of its side.
    let mut side = [None, None];
    for (f, t) in out.tris.iter().enumerate() {
        let cx = t.iter().map(|&i| out.positions[i as usize].x).sum::<f64>() / 3.0;
        let g = out.groups.group_of(FaceId(f as u32));
        let s = usize::from(cx > 20.0);
        assert_eq!(*side[s].get_or_insert(g), g);
    }
    assert_ne!(side[0], side[1]);
}

#[test]
fn free_boundary_still_valid() {
    let m = common::grid(30, 30, |i| i % 3 == 0);
    let out = run(
        &m,
        &RemeshOpts {
            preserve_boundary: false,
            ..opts(2.0)
        },
    );
    let mt = metrics(soup(&out));
    assert_eq!((mt.non_manifold_edges, mt.euler), (0, 1));
}

fn triple_key(p: [Point3; 3]) -> [[u64; 3]; 3] {
    let mut k = p.map(|v| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]);
    k.sort_unstable();
    k
}

#[test]
fn selection_leaves_other_faces_untouched() {
    let m = icosphere(3);
    let sel: Vec<FaceId> = (0..m.face_count() as u32)
        .map(FaceId)
        .filter(|&f| m.triangle(f).iter().map(|p| p.x).sum::<f64>() > 0.0)
        .collect();
    let chosen: HashSet<u32> = sel.iter().map(|f| f.0).collect();
    let out = run(
        &m,
        &RemeshOpts {
            selection: Some(sel),
            ..opts(0.8)
        },
    );
    assert_ne!(out.tris.len(), m.tris.len());
    let after: HashSet<_> = (0..out.face_count() as u32)
        .map(|f| triple_key(out.triangle(FaceId(f))))
        .collect();
    for f in 0..m.face_count() as u32 {
        if !chosen.contains(&f) {
            assert!(
                after.contains(&triple_key(m.triangle(FaceId(f)))),
                "face {f} changed"
            );
        }
    }
    let (mi, mo) = (metrics(soup(&m)), metrics(soup(&out)));
    assert_eq!(
        (mo.boundary_edges, mo.non_manifold_edges, mo.euler),
        (0, 0, mi.euler)
    );
}

#[test]
fn zero_iterations_and_unprojected_run() {
    let m = icosphere(2);
    let same = run(
        &m,
        &RemeshOpts {
            iterations: 0,
            ..opts(1.0)
        },
    );
    assert_eq!(same.tris, m.tris);
    let out = run(
        &m,
        &RemeshOpts {
            project_to_input: false,
            ..opts(1.0)
        },
    );
    assert_eq!(metrics(soup(&out)).euler, 2);
}

#[test]
fn relative_sizing_keeps_triangle_count_near() {
    let m = common::torus(100, 60, 30.0, 8.0);
    let out = run(&m, &RemeshOpts::default());
    let ratio = out.tris.len() as f64 / m.tris.len() as f64;
    assert!((0.6..1.5).contains(&ratio), "{ratio}");
}

#[test]
fn bit_identical_across_thread_counts() {
    let m = displaced_sphere(4);
    let hash_with = |threads: usize| {
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap()
            .install(|| run(&m, &opts(0.9)).content_hash())
    };
    let h1 = hash_with(1);
    assert_eq!(h1, hash_with(4));
    assert_eq!(h1, hash_with(13));
}

#[test]
fn errors_and_cancel() {
    let ctx = JobCtx::detached();
    assert!(matches!(
        remesh(&IndexedMesh::default(), &opts(1.0), &ctx),
        Err(MeshError::Empty)
    ));
    let m = icosphere(2);
    for bad in [0.0, -1.0, f64::NAN] {
        assert!(matches!(
            remesh(&m, &opts(bad), &ctx),
            Err(MeshError::InvalidOption { .. })
        ));
    }
    let adaptive = RemeshOpts {
        sizing: Sizing::Adaptive {
            tolerance: 0.1,
            min_length: 0.1,
            max_length: 2.0,
        },
        ..RemeshOpts::default()
    };
    assert!(matches!(
        remesh(&m, &adaptive, &ctx),
        Err(MeshError::InvalidOption { .. })
    ));
    let bad_sel = RemeshOpts {
        selection: Some(vec![FaceId(1_000_000)]),
        ..opts(1.0)
    };
    assert!(matches!(
        remesh(&m, &bad_sel, &ctx),
        Err(MeshError::InvalidOption { .. })
    ));
    // Three triangles on one edge.
    let fin = IndexedMesh::new(
        vec![Point3::ZERO, Point3::X, Point3::Y, -Point3::Y, Point3::Z],
        vec![[0, 1, 2], [1, 0, 3], [0, 1, 4]],
    )
    .unwrap();
    assert!(matches!(
        remesh(&fin, &opts(1.0), &ctx),
        Err(MeshError::NonManifold { .. })
    ));
    let tok = CancelToken::new();
    tok.cancel();
    assert!(matches!(
        remesh(&icosphere(4), &opts(1.0), &JobCtx::with_token(tok)),
        Err(MeshError::Cancelled)
    ));
}

fn require_testdata() -> bool {
    std::env::var("AMG_REQUIRE_TESTDATA").is_ok_and(|v| v == "1")
}

/// Committed corpus models without topological defects (a model may have several shells).
/// Prints the Done-when metrics for two sizings; topology must always be unchanged.
#[test]
fn corpus_watertight_models() {
    let manifest = Manifest::load().unwrap();
    let root = corpus_root();
    let (mut tested, mut passed, mut fails) = (0, 0, Vec::new());
    for e in manifest.committed() {
        if e.defects.iter().any(|d| *d != Defect::MultipleShells)
            || !(500..=60_000).contains(&e.faces)
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
        let Ok(mut m) = read_mesh(&path) else {
            continue;
        };
        m.weld(WeldOpts::default()).unwrap();
        let before = metrics(soup(&m));
        if before.boundary_edges != 0 || before.non_manifold_edges != 0 {
            continue;
        }
        let bb = m.aabb();
        let diag = (bb.max - bb.min).length();
        tested += 1;
        for (label, l) in [("diag/60", diag / 60.0), ("diag/120", diag / 120.0)] {
            let Ok(out) = remesh(&m, &opts(l), &JobCtx::detached()) else {
                fails.push(format!("{}: remesh failed", e.id));
                continue;
            };
            let (tri, edge) = quality(&out, l);
            let h = hausdorff(soup(&m), soup(&out), 50_000);
            let mo = metrics(soup(&out));
            let ok = tri >= 0.95 && edge >= 0.90 && h <= 0.5 * l;
            eprintln!(
                "CORPUS {} {label}: {} -> {} tris, L {l:.3}, angle {tri:.3}, edges {edge:.3}, hausdorff/L {:.3} {}",
                e.id,
                m.tris.len(),
                out.tris.len(),
                h / l,
                if ok { "PASS" } else { "FAIL" }
            );
            if ok && label == "diag/120" {
                passed += 1;
            }
            if (mo.boundary_edges, mo.non_manifold_edges, mo.euler) != (0, 0, before.euler)
                || shells(&out) != shells(&m)
            {
                fails.push(format!("{}: topology changed", e.id));
            }
        }
    }
    eprintln!("corpus models remeshed: {tested}");
    if tested == 0 && !require_testdata() {
        // Clone without `git lfs pull` (the CI test job): nothing to check here.
        return;
    }
    assert!(tested >= 10, "only {tested} corpus models usable");
    // Models with features thinner than L or wedges sharper than 30 degrees cannot meet the
    // quality bars (see the MESH-06 refinement log).
    assert!(
        passed * 100 >= tested * 85,
        "{passed} of {tested} models meet the quality bars"
    );
    assert!(fails.is_empty(), "{fails:#?}");
}

#[test]
fn golden_remesh_sphere() {
    let out = run(&icosphere(4), &opts(1.5));
    amg_testkit::assert_golden!("mesh", "remesh_sphere_r10_l1.5", soup(&out));
}

#[test]
fn golden_remesh_cube() {
    let out = run(&subdivided_cube(10, 10.0), &opts(1.7));
    amg_testkit::assert_golden!("mesh", "remesh_cube_l1.7", soup(&out));
}

#[test]
fn golden_remesh_torus() {
    let out = run(&common::torus(60, 40, 30.0, 8.0), &opts(2.5));
    amg_testkit::assert_golden!("mesh", "remesh_torus_l2.5", soup(&out));
}
