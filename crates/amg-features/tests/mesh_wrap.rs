//! MESH-18: wrap repair (`amg_features::mesh::wrap::wrap`).

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::cast_precision_loss,
    clippy::print_stderr,
    missing_docs,
    reason = "tests"
)]

use std::sync::Arc;
use std::time::Instant;

use amg_core::{Point3, Transform, Vec3};
use amg_features::mesh::MeshFeatureError;
use amg_features::mesh::wrap::{DEFAULT_OFFSET_VOXELS, WrapOpts, wrap};
use amg_jobs::{CancelToken, JobCtx};
use amg_mesh::corpus::{FileStatus, Manifest, corpus_root, read_mesh};
use amg_mesh::{
    AnalyzeOpts, BvhOpts, IndexedMesh, MeshError, MeshSpatial, SelfIntersectOpts, analyze,
    self_intersections, separate_shells,
};
use amg_testkit::{TriSoup, hausdorff, one_sided_hausdorff, shapes};

mod wrap_common;
use wrap_common::scan_part;

fn ctx() -> JobCtx {
    JobCtx::detached()
}

fn require_testdata() -> bool {
    std::env::var("AMG_REQUIRE_TESTDATA").is_ok_and(|v| v == "1")
}

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}

fn soup(m: &IndexedMesh) -> TriSoup<'_> {
    TriSoup {
        positions: &m.positions,
        triangles: &m.tris,
    }
}

fn mesh(s: shapes::TriMesh) -> IndexedMesh {
    IndexedMesh::new(s.positions, s.triangles).unwrap()
}

fn icosphere(c: Point3, radius: f64, sub: u32) -> IndexedMesh {
    mesh(shapes::icosphere(c, radius, sub))
}

fn cube(min: Point3, size: f64) -> IndexedMesh {
    mesh(shapes::cube(min, size))
}

/// The meshes as one soup (no welding across them).
fn concat(parts: &[IndexedMesh]) -> IndexedMesh {
    let mut positions = Vec::new();
    let mut tris = Vec::new();
    for p in parts {
        let base = positions.len() as u32;
        positions.extend_from_slice(&p.positions);
        tris.extend(p.tris.iter().map(|t| t.map(|i| i + base)));
    }
    IndexedMesh::new(positions, tris).unwrap()
}

fn spatial(m: &IndexedMesh) -> Arc<MeshSpatial> {
    Arc::new(MeshSpatial::build(Arc::new(m.clone()), &BvhOpts::default(), &ctx()).unwrap())
}

fn run(m: &IndexedMesh, opts: &WrapOpts) -> IndexedMesh {
    wrap(&Arc::new(m.clone()), opts, &ctx()).unwrap()
}

/// Closed, manifold, oriented, outward and SI-free (MESH-02 + MESH-19).
fn assert_clean(name: &str, s: &IndexedMesh) {
    let a = analyze(s, &AnalyzeOpts::default(), &ctx()).unwrap();
    assert!(a.is_watertight(), "{name}: {a:?}");
    let pairs = self_intersections(&spatial(s), &SelfIntersectOpts::default(), &ctx()).unwrap();
    assert!(
        pairs.is_empty(),
        "{name}: {} intersecting pairs",
        pairs.len()
    );
    assert!(s.signed_volume() > 0.0, "{name}: not outward");
}

/// Hausdorff readings of a wrap `out` of `input` (mm):
/// - `out_to_in`: largest distance from a point of the result to the input surface
///   (area-sampled plus every vertex);
/// - `uncovered`: largest distance from an input point (vertices and face centroids) to the
///   result's solid (0 inside it). The input's interior faces (overlaps, internal walls) are
///   inside the envelope, so this is the meaningful reverse direction for a soup.
struct Fit {
    out_to_in: f64,
    uncovered: f64,
}

fn fit(input: &IndexedMesh, out: &IndexedMesh) -> Fit {
    let sin = spatial(input);
    let vmax = out
        .positions
        .iter()
        .map(|&p| sin.closest_point(p, f64::INFINITY).unwrap().dist_sq.sqrt())
        .fold(0.0, f64::max);
    let out_to_in = one_sided_hausdorff(soup(out), soup(input), 20_000).max(vmax);
    let sout = spatial(out);
    let probes = input
        .positions
        .iter()
        .copied()
        .chain(input.tris.iter().map(|t| {
            t.iter()
                .map(|&i| input.positions[i as usize])
                .sum::<Point3>()
                / 3.0
        }));
    let uncovered = probes
        .filter(|p| p.is_finite() && !sout.is_inside(*p))
        .map(|p| sout.closest_point(p, f64::INFINITY).unwrap().dist_sq.sqrt())
        .fold(0.0, f64::max);
    Fit {
        out_to_in,
        uncovered,
    }
}

/// Done-when bound: 2 × (offset + voxel).
fn bound(opts: &WrapOpts) -> f64 {
    2.0 * (opts.radius() + opts.voxel_size)
}

/// Wrap, check clean + the Hausdorff bound, return the result.
fn check(name: &str, m: &IndexedMesh, opts: &WrapOpts) -> IndexedMesh {
    let t0 = Instant::now();
    let s = run(m, opts);
    let dt = t0.elapsed();
    assert_clean(name, &s);
    let f = fit(m, &s);
    eprintln!(
        "{name}: {} -> {} tris, {} shells, out->in {:.4}, uncovered {:.4} (bound {:.4}), {dt:?}",
        m.face_count(),
        s.face_count(),
        separate_shells(&s).len(),
        f.out_to_in,
        f.uncovered,
        bound(opts)
    );
    assert!(
        f.out_to_in <= bound(opts),
        "{name}: out->in {}",
        f.out_to_in
    );
    assert!(
        f.uncovered <= bound(opts),
        "{name}: uncovered {}",
        f.uncovered
    );
    s
}

#[test]
fn sphere_grows_by_about_one_skin() {
    let m = icosphere(Point3::ZERO, 10.0, 5);
    let opts = WrapOpts::new(0.25);
    let s = check("sphere", &m, &opts);
    assert_eq!(separate_shells(&s).len(), 1);
    // A convex body's closing is itself: the result is the sphere grown by one voxel skin.
    let h = hausdorff(soup(&m), soup(&s), 20_000);
    assert!(h <= 0.25 + 0.5 * 0.25, "symmetric hausdorff {h}");
    let r = 10.25;
    let expect = 4.0 / 3.0 * std::f64::consts::PI * r * r * r;
    let v = s.signed_volume();
    assert!((v - expect).abs() < 0.02 * expect, "{v} vs {expect}");
}

#[test]
fn overlapping_and_inverted_soup_becomes_one_solid() {
    // Two overlapping cubes and a sphere poking through, one of them inside out: interior
    // faces, crossings and inconsistent orientation.
    let mut b = cube(Point3::new(4.0, 1.0, 1.0), 8.0);
    amg_mesh::reverse_normals(&mut b, None);
    let m = concat(&[
        cube(Point3::ZERO, 8.0),
        b,
        icosphere(Point3::new(10.0, 5.0, 5.0), 3.5, 3),
    ]);
    let opts = WrapOpts::new(0.25);
    let s = check("soup", &m, &opts);
    assert_eq!(separate_shells(&s).len(), 1);
}

#[test]
fn open_sheet_becomes_a_thin_slab() {
    // A single open square (two triangles) has no volume; the skin keeps a ~2-voxel slab.
    let m = IndexedMesh::new(
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(10.0, 10.0, 0.0),
            Point3::new(0.0, 10.0, 0.0),
        ],
        vec![[0, 1, 2], [0, 2, 3]],
    )
    .unwrap();
    let opts = WrapOpts::new(0.25);
    let s = check("sheet", &m, &opts);
    assert_eq!(separate_shells(&s).len(), 1);
    let bb = s.aabb();
    let thick = bb.max.z - bb.min.z;
    assert!(thick > 0.25 && thick < 0.25 * 3.0, "slab thickness {thick}");
}

#[test]
fn holes_and_cavities_are_closed() {
    // A sphere with a small hole (faces near the top removed): wraps to one closed shell.
    let mut m = icosphere(Point3::ZERO, 10.0, 4);
    let keep: Vec<[u32; 3]> = m
        .tris
        .iter()
        .copied()
        .filter(|t| t.iter().map(|&i| m.positions[i as usize].z).sum::<f64>() / 3.0 < 9.7)
        .collect();
    assert!(keep.len() < m.tris.len());
    m.tris = keep;
    m.groups = amg_mesh::FaceGroups::new(m.tris.len());
    m.compact();
    let opts = WrapOpts {
        offset: Some(2.0),
        ..WrapOpts::new(0.25)
    };
    let s = check("holed sphere", &m, &opts);
    assert_eq!(separate_shells(&s).len(), 1, "hole closed, no inner wall");
    let full = 4.0 / 3.0 * std::f64::consts::PI * 1000.0;
    assert!(s.signed_volume() > full, "filled: {}", s.signed_volume());

    // A hollow ball (outer sphere + inward inner sphere) with an island inside: filled solid.
    let mut inner = icosphere(Point3::ZERO, 7.0, 3);
    amg_mesh::reverse_normals(&mut inner, None);
    let hollow = concat(&[
        icosphere(Point3::ZERO, 10.0, 4),
        inner,
        icosphere(Point3::ZERO, 2.0, 2),
    ]);
    let s = check("hollow ball", &hollow, &WrapOpts::new(0.25));
    assert_eq!(separate_shells(&s).len(), 1);
    assert!(s.signed_volume() > full);
}

#[test]
fn gaps_close_below_twice_the_offset() {
    let pair = |gap: f64| {
        concat(&[
            cube(Point3::ZERO, 6.0),
            cube(Point3::new(6.0 + gap, 0.0, 0.0), 6.0),
        ])
    };
    let opts = WrapOpts {
        offset: Some(1.0),
        ..WrapOpts::new(0.2)
    };
    // 1 mm gap < 2 x 1 mm: bridged.
    let s = check("gap 1", &pair(1.0), &opts);
    assert_eq!(separate_shells(&s).len(), 1);
    // 3 mm gap > 2 x (1 mm + skin): stays open.
    let s = check("gap 3", &pair(3.0), &opts);
    assert_eq!(separate_shells(&s).len(), 2);
}

#[test]
fn offsets_and_decimation() {
    let mut c = cube(Point3::ZERO, 10.0);
    c.transform(&Transform::from_translation(Vec3::new(0.3, -0.7, 1.1)));
    // Default offset, a larger one, and an offset below one voxel (no erosion at all).
    for offset in [None, Some(1.5), Some(0.15)] {
        let opts = WrapOpts {
            offset,
            ..WrapOpts::new(0.25)
        };
        check(&format!("cube offset {offset:?}"), &c, &opts);
    }
    let raw = run(
        &c,
        &WrapOpts {
            simplify_tolerance: Some(0.0),
            ..WrapOpts::new(0.25)
        },
    );
    let dec = run(&c, &WrapOpts::new(0.25));
    assert_clean("raw", &raw);
    assert!(dec.face_count() * 2 < raw.face_count());
    assert_eq!(WrapOpts::new(2.0).radius(), DEFAULT_OFFSET_VOXELS * 2.0);
    assert_eq!(WrapOpts::new(2.0).erosion(), 1.0);
}

#[test]
fn errors_and_cancellation() {
    let m = Arc::new(cube(Point3::ZERO, 5.0));
    let bad = |o: WrapOpts| {
        matches!(
            wrap(&m, &o, &ctx()),
            Err(MeshFeatureError::Mesh(MeshError::InvalidOption { .. }))
        )
    };
    assert!(bad(WrapOpts::new(0.0)));
    assert!(bad(WrapOpts::new(f64::NAN)));
    assert!(bad(WrapOpts {
        offset: Some(-1.0),
        ..WrapOpts::new(0.5)
    }));
    assert!(bad(WrapOpts {
        simplify_tolerance: Some(f64::INFINITY),
        ..WrapOpts::new(0.5)
    }));
    let empty = Arc::new(IndexedMesh::new(vec![], vec![]).unwrap());
    assert!(matches!(
        wrap(&empty, &WrapOpts::new(0.5), &ctx()),
        Err(MeshFeatureError::Mesh(MeshError::Empty))
    ));
    let tok = CancelToken::new();
    tok.cancel();
    let r = wrap(&m, &WrapOpts::new(0.5), &JobCtx::with_token(tok));
    assert!(amg_core::Diagnostic::is_cancelled(&r.unwrap_err()));
}

#[test]
fn identical_across_thread_counts() {
    let m = concat(&[
        cube(Point3::ZERO, 6.0),
        icosphere(Point3::new(6.0, 3.0, 3.0), 2.5, 3),
    ]);
    let opts = WrapOpts::new(0.25);
    let go = |n: usize| {
        rayon::ThreadPoolBuilder::new()
            .num_threads(n)
            .build()
            .unwrap()
            .install(|| run(&m, &opts).content_hash())
    };
    let h = go(1);
    assert_eq!(h, go(4));
    assert_eq!(h, go(13));
    assert_eq!(h, go(4));
}

#[test]
fn golden_wraps() {
    let s = run(&icosphere(Point3::ZERO, 10.0, 4), &WrapOpts::new(0.25));
    amg_testkit::assert_golden!("mesh", "wrap_sphere_r10", soup(&s));
    let mut b = cube(Point3::new(4.0, 1.0, 1.0), 8.0);
    amg_mesh::reverse_normals(&mut b, None);
    let soup_in = concat(&[cube(Point3::ZERO, 8.0), b]);
    let s = run(&soup_in, &WrapOpts::new(0.3));
    amg_testkit::assert_golden!("mesh", "wrap_cube_soup", soup(&s));
}

/// Done-when: 100 % of the corpus models (including `expect_repairable = false`) give a
/// watertight, SI-free result with Hausdorff to the input <= 2 x (offset + voxel).
#[test]
fn corpus_all_models() {
    let manifest = Manifest::load().expect("corpus.toml");
    let root = corpus_root();
    let (mut checked, mut hopeless, mut lfs) = (0usize, 0usize, 0usize);
    let (mut worst_out, mut worst_cov) = (0.0f64, 0.0f64);
    let mut failed: Vec<String> = Vec::new();
    for e in &manifest.meshes {
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
        let m = read_mesh(&path).expect("read");
        let h = m.aabb().diagonal() / 80.0;
        let opts = WrapOpts::new(h);
        let t0 = Instant::now();
        let s = match wrap(&Arc::new(m.clone()), &opts, &ctx()) {
            Ok(s) => s,
            Err(err) => {
                failed.push(format!("{}: {err}", e.id));
                continue;
            }
        };
        let dt = t0.elapsed();
        let a = analyze(&s, &AnalyzeOpts::default(), &ctx()).unwrap();
        let si = self_intersections(&spatial(&s), &SelfIntersectOpts::default(), &ctx())
            .unwrap()
            .len();
        let f = fit(&m, &s);
        let (ro, rc) = (f.out_to_in / bound(&opts), f.uncovered / bound(&opts));
        eprintln!(
            "{}{}: {} faces -> {} tris, out->in {ro:.3}, uncovered {rc:.3} of the bound, {dt:?}",
            e.id,
            if e.expect_repairable {
                ""
            } else {
                " (hopeless)"
            },
            m.face_count(),
            s.face_count(),
        );
        if !a.is_watertight() || si != 0 || s.signed_volume() <= 0.0 || ro > 1.0 || rc > 1.0 {
            failed.push(format!(
                "{}: watertight {}, {si} SI pairs, volume {}, out->in {ro:.3}, uncovered {rc:.3}",
                e.id,
                a.is_watertight(),
                s.signed_volume()
            ));
        }
        worst_out = worst_out.max(ro);
        worst_cov = worst_cov.max(rc);
        checked += 1;
        if !e.expect_repairable {
            hopeless += 1;
        }
    }
    eprintln!(
        "corpus: {checked} models wrapped ({hopeless} with expect_repairable = false), worst \
         out->in {worst_out:.3} and uncovered {worst_cov:.3} of 2 x (offset + voxel); {lfs} LFS \
         pointers skipped; failed: {failed:?}"
    );
    assert!(failed.is_empty(), "{failed:#?}");
    if checked == 0 {
        assert!(!require_testdata(), "no corpus model was available");
        eprintln!("SKIP: no corpus file is available (git lfs pull)");
    }
}

/// S07 budget: wrap of a 1 M-triangle scan at 0.5 mm voxels < 30 s (asserted only with
/// `AMG_STRICT_TIMING=1`, where the case also runs; criterion bench `benches/mesh_wrap.rs`).
#[test]
fn timing_wrap_1m_scan() {
    if !strict_timing() {
        eprintln!("SKIP: set AMG_STRICT_TIMING=1 (and use --release) to run the budget case");
        return;
    }
    let m = Arc::new(scan_part(amg_mesh::corpus::bench::BenchSize::M1));
    let opts = WrapOpts::new(0.5);
    let t0 = Instant::now();
    let s = wrap(&m, &opts, &ctx()).unwrap();
    let dt = t0.elapsed();
    eprintln!(
        "wrap 1M-tri scan, 0.5 mm voxel: {} -> {} tris, {dt:?}",
        m.face_count(),
        s.face_count()
    );
    assert!(dt.as_secs_f64() < 30.0, "{dt:?}");
}
