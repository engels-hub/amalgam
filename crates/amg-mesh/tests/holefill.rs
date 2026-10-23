//! MESH-20 tests: hole filling (Liepa).

#![allow(clippy::unwrap_used, clippy::float_cmp, clippy::cast_precision_loss)]

#[path = "holefill_common/mod.rs"]
mod hf;

use std::collections::{HashMap, HashSet};

use amg_core::Point3;
use amg_jobs::{CancelToken, JobCtx};
use amg_mesh::corpus::{Defect, FileStatus, Manifest, corpus_root, read_mesh};
use amg_mesh::{
    AnalyzeOpts, Fairing, GroupOrigin, HalfEdgeMesh, HoleFillOpts, HoleFillReport, IndexedMesh,
    MeshError, NonManifoldPolicy, WeldOpts, analyze, fill_holes, fill_holes_indexed,
};
use amg_testkit::{TriSoup, shapes};
use proptest::prelude::*;

fn soup(m: &IndexedMesh) -> TriSoup<'_> {
    TriSoup {
        positions: &m.positions,
        triangles: &m.tris,
    }
}

fn ctx() -> JobCtx {
    JobCtx::detached()
}

fn fill(m: &IndexedMesh, opts: &HoleFillOpts) -> (IndexedMesh, HoleFillReport) {
    let (out, rep) = fill_holes_indexed(m, opts, &ctx()).unwrap();
    out.validate().unwrap();
    (out, rep)
}

fn icosphere(radius: f64, sub: u32) -> IndexedMesh {
    let s = shapes::icosphere(Point3::ZERO, radius, sub);
    IndexedMesh::new(s.positions, s.triangles).unwrap()
}

/// Asserts the mesh is closed, manifold and consistently oriented, with no degenerate faces.
fn assert_watertight(label: &str, m: &IndexedMesh) {
    let r = analyze(m, &AnalyzeOpts::default(), &ctx()).unwrap();
    assert!(
        r.boundary_loops.is_empty(),
        "{label}: {} open loops",
        r.boundary_loops.len()
    );
    assert!(r.is_watertight(), "{label}: not watertight");
    assert!(
        r.non_manifold_edges.is_empty(),
        "{label}: non-manifold edges"
    );
    assert!(
        r.non_manifold_vertices.is_empty(),
        "{label}: non-manifold vertices"
    );
    assert!(r.duplicate_faces.is_empty(), "{label}: duplicate faces");
}

fn edge_len(m: &IndexedMesh, a: u32, b: u32) -> f64 {
    m.positions[a as usize].distance(m.positions[b as usize])
}

/// Mean length of the patch's interior edges (both sides new) and of the old edges at the
/// patch (old faces touching a vertex of a new face).
fn patch_vs_surround(before: &IndexedMesh, after: &IndexedMesh) -> (f64, f64, usize) {
    let old = before.tris.len();
    let mut count: HashMap<(u32, u32), (u32, u32)> = HashMap::new();
    let mut new_vertex = HashSet::new();
    for (f, t) in after.tris.iter().enumerate() {
        for c in 0..3 {
            let (a, b) = (t[c], t[(c + 1) % 3]);
            let e = count.entry((a.min(b), a.max(b))).or_default();
            if f >= old {
                e.1 += 1;
                new_vertex.insert(a);
            } else {
                e.0 += 1;
            }
        }
    }
    let (mut ps, mut pn, mut ss, mut sn) = (0.0, 0usize, 0.0, 0usize);
    for (&(a, b), &(o, n)) in &count {
        if n == 2 {
            ps += edge_len(after, a, b);
            pn += 1;
        } else if o > 0 && (new_vertex.contains(&a) || new_vertex.contains(&b)) {
            ss += edge_len(after, a, b);
            sn += 1;
        }
    }
    (ps / pn.max(1) as f64, ss / sn.max(1) as f64, pn)
}

#[test]
fn single_triangle_hole() {
    let s = icosphere(10.0, 2);
    let cut = hf::without_faces(&s, |f| f.0 == 7);
    let (out, rep) = fill(&cut, &HoleFillOpts::default());
    assert_eq!((rep.filled, rep.failed), (1, 0));
    assert!(rep.new_faces >= 1);
    assert_watertight("triangle hole", &out);
}

/// Every hole size from 3 to 2000 boundary edges: watertight, nothing non-manifold, Euler 2.
#[test]
fn tube_holes_of_all_sizes_are_filled() {
    for n in [
        3u32, 4, 5, 6, 7, 8, 10, 12, 16, 24, 40, 64, 100, 180, 250, 251, 400, 500, 1000, 2000,
    ] {
        for (wobble, bumps) in [(0.0, 0.0), (0.4, 0.05)] {
            let t = hf::tube(n, 6, 1.0, wobble, bumps);
            let mut opts = HoleFillOpts::default();
            if n >= 1000 {
                opts.fairing = Fairing::Harmonic; // keeps the 2000-edge cases quick
            }
            let started = std::time::Instant::now();
            let (out, rep) = fill(&t, &opts);
            let label = format!("tube n={n} wobble={wobble}");
            assert_eq!(rep.filled, 2, "{label}: {rep:?}");
            assert_eq!(rep.failed + rep.skipped_too_large, 0, "{label}: {rep:?}");
            assert_watertight(&label, &out);
            let r = analyze(&out, &AnalyzeOpts::default(), &ctx()).unwrap();
            assert_eq!(
                out.vertex_count() as i64 - 3 * out.tris.len() as i64 / 2 + out.tris.len() as i64,
                2,
                "{label}: Euler"
            );
            assert_eq!(r.shells.len(), 1, "{label}");
            eprintln!(
                "{label}: +{} faces in {:.1} ms",
                rep.new_faces,
                started.elapsed().as_secs_f64() * 1e3
            );
        }
    }
}

#[test]
fn without_refinement_the_patch_is_a_plain_triangulation() {
    for n in [5u32, 9, 31, 120, 300] {
        let t = hf::tube(n, 4, 1.0, 0.3, 0.05);
        let opts = HoleFillOpts {
            refine: false,
            ..HoleFillOpts::default()
        };
        let (out, rep) = fill(&t, &opts);
        // An n-gon triangulated without new vertices has n - 2 triangles (a fan adds one vertex
        // and n triangles).
        assert!(
            rep.new_faces == 2 * (n as usize - 2) || rep.new_faces == 2 * n as usize,
            "n={n}: {rep:?}"
        );
        assert_watertight("no refine", &out);
    }
}

/// The patch edge length follows the surroundings (within 20 %).
#[test]
fn patch_edge_length_matches_the_surround() {
    for n in [24u32, 40, 64, 100, 250, 400] {
        for (wobble, bumps) in [(0.0, 0.0), (0.3, 0.05)] {
            let t = hf::tube(n, 6, 1.0, wobble, bumps);
            let (out, _) = fill(&t, &HoleFillOpts::default());
            let (patch, around, count) = patch_vs_surround(&t, &out);
            let ratio = patch / around;
            eprintln!(
                "n={n} wobble={wobble}: patch {patch:.3}, surround {around:.3}, ratio {ratio:.3}, {count} edges"
            );
            assert!(
                (ratio - 1.0).abs() <= 0.2,
                "n={n} wobble={wobble}: patch/surround edge length {ratio:.3}"
            );
        }
    }
}

/// Holes cut out of a sphere: the fair patch stays within 2 % of the radius.
#[test]
fn fair_fill_of_a_sphere_cap_stays_on_the_sphere() {
    let r = 20.0;
    let s = icosphere(r, 5);
    for (rho, fairing, bound) in [
        (3.0, Fairing::Biharmonic, 0.02),
        (5.0, Fairing::Biharmonic, 0.02),
        (3.0, Fairing::Harmonic, 0.02),
    ] {
        let cut = hf::without_faces(&s, |f| {
            let t = s.tris[f.0 as usize];
            let c = (s.positions[t[0] as usize]
                + s.positions[t[1] as usize]
                + s.positions[t[2] as usize])
                / 3.0;
            c.distance(Point3::new(0.0, 0.0, r)) < rho
        });
        let opts = HoleFillOpts {
            fairing,
            ..HoleFillOpts::default()
        };
        let (out, rep) = fill(&cut, &opts);
        assert_eq!(rep.filled, 1);
        assert_watertight("sphere cap", &out);
        let dev = out
            .positions
            .iter()
            .skip(cut.positions.len())
            .map(|p| (p.length() - r).abs() / r)
            .fold(0.0, f64::max);
        eprintln!(
            "rho {rho} {fairing:?}: {} new vertices, max deviation {:.4} r",
            out.positions.len() - cut.positions.len(),
            dev
        );
        assert!(dev < bound, "rho {rho} {fairing:?}: deviation {dev} r");
    }
}

/// Fairing improves on the raw patch.
#[test]
fn fairing_beats_no_fairing() {
    let r = 20.0;
    let s = icosphere(r, 5);
    let cut = hf::without_faces(&s, |f| {
        let t = s.tris[f.0 as usize];
        let c =
            (s.positions[t[0] as usize] + s.positions[t[1] as usize] + s.positions[t[2] as usize])
                / 3.0;
        c.distance(Point3::new(0.0, 0.0, r)) < 4.0
    });
    let dev = |fairing| {
        let opts = HoleFillOpts {
            fairing,
            ..HoleFillOpts::default()
        };
        let (out, _) = fill(&cut, &opts);
        out.positions
            .iter()
            .skip(cut.positions.len())
            .map(|p| (p.length() - r).abs())
            .fold(0.0, f64::max)
    };
    let (none, harm, bi) = (
        dev(Fairing::None),
        dev(Fairing::Harmonic),
        dev(Fairing::Biharmonic),
    );
    eprintln!("deviation none {none:.4}, harmonic {harm:.4}, biharmonic {bi:.4}");
    assert!(
        bi < harm && harm < none + 1e-9 || bi < none,
        "{none} {harm} {bi}"
    );
}

/// A planar loop is filled with a CDT: no new vertices, total patch area is the polygon area.
#[test]
fn planar_concave_loop_uses_the_constrained_triangulation() {
    // L-shaped prism without its top: the rim is a planar concave 8-gon, subdivided to 40 edges.
    let outline: [(f64, f64); 6] = [
        (0.0, 0.0),
        (4.0, 0.0),
        (4.0, 2.0),
        (2.0, 2.0),
        (2.0, 4.0),
        (0.0, 4.0),
    ];
    let mut rim = Vec::new();
    for k in 0..outline.len() {
        let (a, b) = (outline[k], outline[(k + 1) % outline.len()]);
        let len = ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt();
        let steps = (len * 3.0) as usize;
        for s in 0..steps {
            let t = s as f64 / steps as f64;
            rim.push((a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t));
        }
    }
    let n = rim.len();
    let mut positions = Vec::new();
    for z in [0.0, -1.0] {
        for &(x, y) in &rim {
            positions.push(Point3::new(x, y, z));
        }
    }
    let mut tris = Vec::new();
    for i in 0..n {
        let j = (i + 1) % n;
        let (a, b, c, d) = (i as u32, j as u32, (n + j) as u32, (n + i) as u32);
        // The outline is counter-clockwise from +z, so outward side walls are a, d, c, b.
        tris.push([a, d, c]);
        tris.push([a, c, b]);
    }
    let m = IndexedMesh::new(positions, tris).unwrap();
    let opts = HoleFillOpts {
        refine: false,
        ..HoleFillOpts::default()
    };
    let (out, rep) = fill(&m, &opts);
    assert_eq!(rep.filled, 2, "{rep:?}");
    assert_eq!(rep.new_faces, 2 * (n - 2), "no Steiner points expected");
    assert_eq!(out.positions.len(), m.positions.len());
    assert_watertight("L prism", &out);
    let area: f64 = (m.tris.len()..out.tris.len())
        .map(|f| {
            let t = out.tris[f];
            let p = t.map(|i| out.positions[i as usize]);
            0.5 * (p[1] - p[0]).cross(p[2] - p[0]).length()
        })
        .sum();
    assert!(
        (area - 2.0 * 12.0).abs() < 1e-9,
        "patch area {area}, polygon 2 x 12"
    );
    // The top cap faces up (+z), the bottom cap down.
    let (mut up, mut down) = (0, 0);
    for f in m.tris.len()..out.tris.len() {
        let p = out.tris[f].map(|i| out.positions[i as usize]);
        let nz = (p[1] - p[0]).cross(p[2] - p[0]).z;
        if nz > 0.0 {
            up += 1;
        } else {
            down += 1;
        }
    }
    assert_eq!((up, down), (n - 2, n - 2));
}

#[test]
fn volume_of_filled_tube_is_sane() {
    // Filled tube = closed shell with positive signed volume.
    let t = hf::tube(40, 8, 1.0, 0.0, 0.0);
    let (out, _) = fill(&t, &HoleFillOpts::default());
    let r = analyze(&out, &AnalyzeOpts::default(), &ctx()).unwrap();
    let v = r.volume.unwrap();
    let rad = 1.0 / (2.0 * (std::f64::consts::PI / 40.0).sin());
    let expect = std::f64::consts::PI * rad * rad * 7.0;
    assert!(
        v > 0.0 && (v - expect).abs() < 0.02 * expect,
        "volume {v}, cylinder {expect}"
    );
    assert!(!r.is_inverted);
}

#[test]
fn options_report_and_groups() {
    let t = hf::tube(30, 5, 1.0, 0.0, 0.0);
    // Too large: left open and counted.
    let opts = HoleFillOpts {
        max_boundary_edges: 29,
        ..HoleFillOpts::default()
    };
    let (out, rep) = fill(&t, &opts);
    assert_eq!(
        (rep.filled, rep.skipped_too_large, rep.new_faces),
        (0, 2, 0)
    );
    assert_eq!(rep.new_group, None);
    assert_eq!(out.tris.len(), t.tris.len());
    // Boundary exactly at the limit is filled.
    let opts = HoleFillOpts {
        max_boundary_edges: 30,
        ..HoleFillOpts::default()
    };
    let (_, rep) = fill(&t, &opts);
    assert_eq!(rep.filled, 2);
    // The new faces form one `Fill` group.
    let (mut he, _) = HalfEdgeMesh::from_indexed(&t, NonManifoldPolicy::SplitVertices).unwrap();
    let before = he.face_count();
    let rep = fill_holes(&mut he, &HoleFillOpts::default(), &ctx()).unwrap();
    let g = rep.new_group.expect("group");
    assert_eq!(he.groups().info(g).unwrap().origin, GroupOrigin::Fill);
    assert_eq!(he.groups().faces_of(g).count(), rep.new_faces);
    assert_eq!(he.face_count(), before + rep.new_faces);
    he.check_invariants().unwrap();
    assert!(he.boundary_loops().is_empty());
    // Invalid option.
    let bad = HoleFillOpts {
        density_factor: 0.0,
        ..HoleFillOpts::default()
    };
    assert!(matches!(
        fill_holes_indexed(&t, &bad, &ctx()),
        Err(MeshError::InvalidOption { .. })
    ));
    // Closed mesh: untouched.
    let s = icosphere(5.0, 2);
    let (out, rep) = fill(&s, &HoleFillOpts::default());
    assert_eq!(rep, HoleFillReport::default());
    assert_eq!(out.content_hash(), s.content_hash());
}

#[test]
fn cancellation() {
    let t = hf::tube(60, 5, 1.0, 0.2, 0.0);
    let tok = CancelToken::new();
    tok.cancel();
    assert!(matches!(
        fill_holes_indexed(&t, &HoleFillOpts::default(), &JobCtx::with_token(tok)),
        Err(MeshError::Cancelled)
    ));
}

#[test]
fn deterministic_across_threads_and_runs() {
    let t = hf::tube(150, 6, 1.0, 0.3, 0.05);
    let run = |threads: usize| {
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap()
            .install(|| fill(&t, &HoleFillOpts::default()).0.content_hash())
    };
    let h = run(1);
    assert_eq!(run(1), h);
    assert_eq!(run(8), h);
}

/// A hole next to an existing chord: the only valid fills avoid duplicating the existing edge.
#[test]
fn existing_chords_are_not_duplicated() {
    // Quad hole a b c d whose diagonal a-c already exists as an edge elsewhere: a tetrahedron
    // pair sharing the vertices. Build a closed octahedron and remove two adjacent faces: the
    // resulting quad hole has the diagonal between two vertices that are not yet connected, so
    // fill must produce a valid manifold.
    let s = icosphere(5.0, 0);
    for seed in 0..s.tris.len() as u32 {
        let ball = hf::face_ball(&s, seed, 1);
        let cut = hf::without_faces(&s, |f| ball[f.0 as usize]);
        let (out, rep) = fill(&cut, &HoleFillOpts::default());
        assert_eq!(rep.failed, 0, "seed {seed}");
        assert_watertight(&format!("ico0 seed {seed}"), &out);
    }
}

fn require_testdata() -> bool {
    std::env::var("AMG_REQUIRE_TESTDATA").is_ok_and(|v| v == "1")
}

/// Corpus models whose only defects are holes: every loop of up to 2000 edges is filled.
#[test]
fn corpus_models_with_holes() {
    let manifest = Manifest::load().unwrap();
    let root = corpus_root();
    let (mut tested, mut closed, mut fails) = (0, 0, Vec::new());
    for e in manifest.committed() {
        if !e.defects.contains(&Defect::Holes) || e.faces > 150_000 {
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
        let before = analyze(&m, &AnalyzeOpts::default(), &ctx()).unwrap();
        if before.boundary_loops.is_empty() || !before.is_manifold {
            continue;
        }
        let started = std::time::Instant::now();
        let Ok((out, rep)) = fill_holes_indexed(&m, &HoleFillOpts::default(), &ctx()) else {
            fails.push(format!("{}: fill failed", e.id));
            continue;
        };
        tested += 1;
        let after = analyze(&out, &AnalyzeOpts::default(), &ctx()).unwrap();
        let small = before
            .boundary_loops
            .iter()
            .filter(|l| l.len() <= 2000)
            .count();
        let open_small = after
            .boundary_loops
            .iter()
            .filter(|l| l.len() <= 2000)
            .count();
        eprintln!(
            "CORPUS {}: {} loops ({} <= 2000) -> {} open, filled {}, failed {}, {:.0} ms",
            e.id,
            before.boundary_loops.len(),
            small,
            after.boundary_loops.len(),
            rep.filled,
            rep.failed,
            started.elapsed().as_secs_f64() * 1e3
        );
        if open_small != 0
            || rep.failed != 0
            || !after.is_manifold
            || after.non_manifold_edges.len() > before.non_manifold_edges.len()
        {
            fails.push(format!(
                "{}: {open_small} small loops left, {rep:?}, manifold {}",
                e.id, after.is_manifold
            ));
        }
        if after.boundary_loops.is_empty() {
            closed += 1;
        }
    }
    eprintln!("corpus models filled: {tested} ({closed} closed)");
    if tested == 0 && !require_testdata() {
        return; // clone without `git lfs pull` (the CI test job)
    }
    assert!(tested >= 5, "only {tested} corpus models usable");
    assert!(fails.is_empty(), "{fails:#?}");
}

#[test]
fn golden_tube_wobbly_rim() {
    let t = hf::tube(48, 5, 1.0, 0.3, 0.04);
    let (out, _) = fill(&t, &HoleFillOpts::default());
    amg_testkit::assert_golden!("mesh", "holefill_tube48_wobbly", soup(&out));
}

#[test]
fn golden_sphere_cap() {
    let r = 20.0;
    let s = icosphere(r, 5);
    let cut = hf::without_faces(&s, |f| {
        let t = s.tris[f.0 as usize];
        let c =
            (s.positions[t[0] as usize] + s.positions[t[1] as usize] + s.positions[t[2] as usize])
                / 3.0;
        c.distance(Point3::new(0.0, 0.0, r)) < 4.0
    });
    let (out, _) = fill(&cut, &HoleFillOpts::default());
    amg_testkit::assert_golden!("mesh", "holefill_sphere_r20_cap4", soup(&out));
}

#[test]
fn golden_planar_tube_cap() {
    let t = hf::tube(64, 4, 1.0, 0.0, 0.03);
    let (out, _) = fill(&t, &HoleFillOpts::default());
    amg_testkit::assert_golden!("mesh", "holefill_tube64_planar", soup(&out));
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(48))]

    /// Random holes (breadth-first balls of faces) in a sphere and a torus always fill to a
    /// closed manifold.
    #[test]
    fn random_holes_fill_to_watertight(
        seed in 0u32..4000,
        k in 0usize..9,
        torus in any::<bool>(),
        fairing in 0u8..3,
        refine in any::<bool>(),
        alpha in 1.0f64..2.0,
    ) {
        let base = if torus { gen_torus() } else { icosphere(10.0, 3) };
        let seed = seed % base.tris.len() as u32;
        let ball = hf::face_ball(&base, seed, k);
        prop_assume!(ball.iter().filter(|b| **b).count() < base.tris.len() / 3);
        let cut = hf::without_faces(&base, |f| ball[f.0 as usize]);
        let opts = HoleFillOpts {
            fairing: [Fairing::None, Fairing::Harmonic, Fairing::Biharmonic][fairing as usize],
            refine,
            density_factor: alpha,
            ..HoleFillOpts::default()
        };
        let (out, rep) = fill(&cut, &opts);
        prop_assert_eq!(rep.failed, 0);
        let r = analyze(&out, &AnalyzeOpts::default(), &ctx()).unwrap();
        prop_assert!(r.boundary_loops.is_empty());
        prop_assert!(r.non_manifold_edges.is_empty());
        prop_assert!(r.is_oriented);
    }
}

fn gen_torus() -> IndexedMesh {
    let (nu, nv) = (40u32, 20u32);
    let mut p = Vec::new();
    for i in 0..nu {
        let u = f64::from(i) / f64::from(nu) * std::f64::consts::TAU;
        for j in 0..nv {
            let v = f64::from(j) / f64::from(nv) * std::f64::consts::TAU;
            let r = 20.0 + 6.0 * v.cos();
            p.push(Point3::new(r * u.cos(), r * u.sin(), 6.0 * v.sin()));
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
