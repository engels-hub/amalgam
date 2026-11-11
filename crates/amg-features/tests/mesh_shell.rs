//! MESH-10 part 2: robust SDF shell (`amg_features::mesh::shell::offset_shell_sdf`).

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
use amg_features::mesh::shell::{SdfShellOpts, offset_shell_sdf};
use amg_implicit::MeshSdf;
use amg_jobs::{CancelToken, JobCtx};
use amg_mesh::corpus::bench::{BenchSize, benchmark_sphere};
use amg_mesh::corpus::{FileStatus, Manifest, corpus_root, quick_stats, read_mesh};
use amg_mesh::{
    AnalyzeOpts, BvhOpts, FaceId, GroupOrigin, IndexedMesh, MeshError, MeshSpatial,
    SelfIntersectOpts, ShellSide, analyze, reverse_normals, self_intersections, separate_shells,
};
use amg_testkit::{TriSoup, shapes};

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

fn icosphere(radius: f64, sub: u32) -> IndexedMesh {
    let s = shapes::icosphere(Point3::ZERO, radius, sub);
    IndexedMesh::new(s.positions, s.triangles).unwrap()
}

fn cube(size: f64) -> IndexedMesh {
    let s = shapes::cube(Point3::ZERO, size);
    IndexedMesh::new(s.positions, s.triangles).unwrap()
}

fn torus(nu: u32, nv: u32, major: f64, minor: f64) -> IndexedMesh {
    let mut p = Vec::new();
    for i in 0..nu {
        let u = f64::from(i) / f64::from(nu) * std::f64::consts::TAU;
        for j in 0..nv {
            let v = f64::from(j) / f64::from(nv) * std::f64::consts::TAU;
            let r = major + minor * v.cos();
            p.push(Point3::new(r * u.cos(), r * u.sin(), minor * v.sin()));
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

fn spatial(m: &IndexedMesh) -> Arc<MeshSpatial> {
    Arc::new(MeshSpatial::build(Arc::new(m.clone()), &BvhOpts::default(), &ctx()).unwrap())
}

/// Closed, manifold, oriented and SI-free (MESH-02 + MESH-19).
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

/// Largest distance of an output vertex from its nearer offset level, mm: every vertex
/// should lie on `s = lo` or `s = hi` of the input's signed distance `s`.
fn level_error(m: &IndexedMesh, s: &IndexedMesh, opts: &SdfShellOpts) -> f64 {
    let (lo, hi) = opts.levels();
    // Same orientation as the shell uses (an inside-out input is flipped).
    let mut m = m.clone();
    if m.signed_volume() < 0.0 {
        reverse_normals(&mut m, None);
    }
    let sdf = MeshSdf::new(spatial(&m), f64::INFINITY);
    s.positions
        .iter()
        .map(|&p| {
            let d = sdf.signed(p);
            (d - lo).abs().min((d - hi).abs())
        })
        .fold(0.0, f64::max)
}

/// Measured wall thickness: distances from the vertices of the inner wall to the outer wall
/// (min, max), for a body whose shell is two nested surfaces.
fn wall_thickness(s: &IndexedMesh) -> (f64, f64) {
    let shells = separate_shells(s);
    assert_eq!(shells.len(), 2, "expected an outer and an inner wall");
    let outer = spatial(&shells[0]);
    shells[1]
        .positions
        .iter()
        .map(|&p| {
            outer
                .closest_point(p, f64::INFINITY)
                .unwrap()
                .dist_sq
                .sqrt()
        })
        .fold((f64::INFINITY, 0.0f64), |(a, b), d| (a.min(d), b.max(d)))
}

fn run(m: &IndexedMesh, opts: &SdfShellOpts) -> IndexedMesh {
    offset_shell_sdf(&Arc::new(m.clone()), opts, &ctx()).unwrap()
}

/// Closed, SI-free, every vertex within one voxel of its level, and the measured wall
/// thickness within one voxel of the target.
fn check_shape(name: &str, m: &IndexedMesh, t: f64, side: ShellSide, h: f64) -> IndexedMesh {
    let opts = SdfShellOpts::new(t, side, h);
    let t0 = Instant::now();
    let s = run(m, &opts);
    let dt = t0.elapsed();
    assert_clean(name, &s);
    let le = level_error(m, &s, &opts);
    let (wmin, wmax) = wall_thickness(&s);
    eprintln!(
        "{name} {side:?} t={t} h={h}: {} tris, level error {le:.4}, wall [{wmin:.4}, {wmax:.4}], \
         {dt:?}",
        s.face_count()
    );
    assert!(le <= h, "{name} {side:?}: level error {le} > voxel {h}");
    assert!(
        wmin >= t - h && wmax <= t + h,
        "{name} {side:?}: wall [{wmin}, {wmax}] not within {t} ± {h}"
    );
    s
}

#[test]
fn sphere_all_sides() {
    let m = icosphere(10.0, 5);
    for side in [ShellSide::Inside, ShellSide::Outside, ShellSide::Both] {
        let s = check_shape("sphere r10", &m, 2.0, side, 0.25);
        let (ro, ri) = match side {
            ShellSide::Inside => (10.0, 8.0),
            ShellSide::Outside => (12.0, 10.0),
            ShellSide::Both => (11.0, 9.0),
        };
        let expect = 4.0 / 3.0 * std::f64::consts::PI * (ro * ro * ro - ri * ri * ri);
        let v = s.signed_volume();
        assert!(
            (v - expect).abs() < 0.02 * expect,
            "{side:?}: {v} vs {expect}"
        );
        let g = s.groups.group_of(FaceId(0));
        assert_eq!(
            s.groups.info(g).unwrap().origin,
            GroupOrigin::Shell { side }
        );
    }
}

#[test]
fn cube_and_torus() {
    // Off-lattice placement so no face sits on a lattice plane by construction.
    let mut c = cube(12.0);
    c.transform(&Transform::from_translation(Vec3::new(0.3, -0.7, 1.1)));
    check_shape("cube 12", &c, 1.5, ShellSide::Inside, 0.3);
    check_shape("cube 12", &c, 1.5, ShellSide::Outside, 0.3);
    let t = torus(96, 48, 10.0, 4.0);
    check_shape("torus", &t, 1.2, ShellSide::Inside, 0.2);
    check_shape("torus", &t, 1.2, ShellSide::Both, 0.2);
}

#[test]
fn lattice_aligned_box_is_clean() {
    // A box whose faces land exactly on lattice planes for an unshifted lattice: exact zeros
    // at lattice points. The lattice shift keeps the result SI-free.
    let c = cube(8.0);
    for h in [0.5, 0.25] {
        let s = run(&c, &SdfShellOpts::new(1.0, ShellSide::Inside, h));
        assert_clean("aligned cube", &s);
    }
}

#[test]
fn inside_out_input_is_flipped() {
    let mut m = icosphere(5.0, 4);
    reverse_normals(&mut m, None);
    let s = run(&m, &SdfShellOpts::new(1.0, ShellSide::Inside, 0.2));
    assert_clean("flipped sphere", &s);
    let expect = 4.0 / 3.0 * std::f64::consts::PI * (125.0 - 64.0);
    assert!((s.signed_volume() - expect).abs() < 0.03 * expect);
}

#[test]
fn thin_wall_region_vanishes_cleanly() {
    // Thickness larger than the torus tube: the inner wall disappears, a solid remains.
    let t = torus(64, 32, 10.0, 1.0);
    let s = run(&t, &SdfShellOpts::new(3.0, ShellSide::Inside, 0.25));
    assert_clean("thick torus", &s);
    assert_eq!(separate_shells(&s).len(), 1);
}

#[test]
fn simplify_tolerance() {
    let m = icosphere(10.0, 4);
    let raw = run(
        &m,
        &SdfShellOpts {
            simplify_tolerance: Some(0.0),
            ..SdfShellOpts::new(2.0, ShellSide::Inside, 0.25)
        },
    );
    let dec = run(&m, &SdfShellOpts::new(2.0, ShellSide::Inside, 0.25));
    assert_clean("raw", &raw);
    eprintln!(
        "raw {} tris, decimated {}",
        raw.face_count(),
        dec.face_count()
    );
    assert!(dec.face_count() * 2 < raw.face_count());
}

#[test]
fn errors() {
    let m = Arc::new(cube(5.0));
    let bad = |o: SdfShellOpts| {
        matches!(
            offset_shell_sdf(&m, &o, &ctx()),
            Err(MeshFeatureError::Mesh(MeshError::InvalidOption { .. }))
        )
    };
    assert!(bad(SdfShellOpts::new(0.0, ShellSide::Inside, 0.5)));
    assert!(bad(SdfShellOpts::new(1.0, ShellSide::Inside, -0.5)));
    assert!(bad(SdfShellOpts::new(f64::NAN, ShellSide::Inside, 0.5)));
    assert!(bad(SdfShellOpts {
        simplify_tolerance: Some(f64::NAN),
        ..SdfShellOpts::new(1.0, ShellSide::Inside, 0.5)
    }));
    let empty = Arc::new(IndexedMesh::new(vec![], vec![]).unwrap());
    assert!(matches!(
        offset_shell_sdf(
            &empty,
            &SdfShellOpts::new(1.0, ShellSide::Inside, 0.5),
            &ctx()
        ),
        Err(MeshFeatureError::Mesh(MeshError::Empty))
    ));
    let tok = CancelToken::new();
    tok.cancel();
    let r = offset_shell_sdf(
        &m,
        &SdfShellOpts::new(1.0, ShellSide::Inside, 0.5),
        &JobCtx::with_token(tok),
    );
    assert!(amg_core::Diagnostic::is_cancelled(&r.unwrap_err()));
}

#[test]
fn identical_across_thread_counts() {
    let m = torus(64, 32, 10.0, 3.0);
    let opts = SdfShellOpts::new(1.0, ShellSide::Both, 0.25);
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
}

#[test]
fn golden_sdf_shells() {
    let s = run(
        &icosphere(10.0, 4),
        &SdfShellOpts::new(2.0, ShellSide::Inside, 0.25),
    );
    amg_testkit::assert_golden!("mesh", "shell_sdf_sphere_r10_inside", soup(&s));
    let mut c = cube(6.0);
    c.transform(&Transform::from_translation(Vec3::new(1.1, 2.2, 3.3)));
    let s = run(&c, &SdfShellOpts::new(1.0, ShellSide::Outside, 0.2));
    amg_testkit::assert_golden!("mesh", "shell_sdf_cube6_outside", soup(&s));
}

/// Done-when: closed, manifold and SI-free on 100 % of the watertight corpus models; every
/// vertex within one voxel of its offset level.
#[test]
fn corpus_watertight_models() {
    let manifest = Manifest::load().expect("corpus.toml");
    let root = corpus_root();
    let (mut checked, mut lfs, mut open) = (0usize, 0usize, 0usize);
    let mut worst = 0.0f64;
    let mut over: Vec<(String, f64)> = Vec::new();
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
        let q = quick_stats(&m);
        if q.boundary_edges != 0 || q.non_manifold_edges != 0 || q.inconsistent_edges != 0 {
            open += 1;
            continue;
        }
        let diag = m.aabb().diagonal();
        let h = diag / 80.0;
        let opts = SdfShellOpts::new(4.0 * h, ShellSide::Inside, h);
        let t0 = Instant::now();
        let s = offset_shell_sdf(&Arc::new(m.clone()), &opts, &ctx())
            .unwrap_or_else(|err| panic!("{}: {err}", e.id));
        let dt = t0.elapsed();
        assert_clean(&e.id, &s);
        let le = level_error(&m, &s, &opts) / h;
        eprintln!(
            "{}: {} faces -> {} tris, level error {le:.3} voxel, {dt:?}",
            e.id,
            m.face_count(),
            s.face_count()
        );
        worst = worst.max(le);
        if le > 1.0 {
            over.push((e.id.clone(), le));
        }
        checked += 1;
    }
    eprintln!(
        "corpus: {checked} watertight models shelled (all closed, manifold, SI-free), worst \
         level error {worst:.3} voxel, over one voxel: {over:?}; {open} not watertight, {lfs} \
         LFS pointers skipped"
    );
    assert!(over.is_empty(), "level error over one voxel: {over:?}");
    if checked == 0 {
        assert!(!require_testdata(), "no corpus model was available");
        eprintln!("SKIP: no corpus file is available (git lfs pull)");
    }
}

/// S07 budget: robust shell of a 1 M-triangle, 100 mm part at 0.25 mm voxels < 20 s (asserted
/// only with `AMG_STRICT_TIMING=1`, where the case also runs; criterion bench
/// `benches/mesh_shell.rs`).
#[test]
fn timing_robust_shell_1m_100mm() {
    if !strict_timing() {
        eprintln!("SKIP: set AMG_STRICT_TIMING=1 (and use --release) to run the budget case");
        return;
    }
    let mut m = benchmark_sphere(BenchSize::M1, 1);
    for p in &mut m.positions {
        *p *= 5.0;
    }
    let m = Arc::new(m);
    let opts = SdfShellOpts::new(2.0, ShellSide::Inside, 0.25);
    let t0 = Instant::now();
    let s = offset_shell_sdf(&m, &opts, &ctx()).unwrap();
    let dt = t0.elapsed();
    eprintln!(
        "robust shell 1M tris, 100 mm, 0.25 mm voxel: {} tris, {dt:?}",
        s.face_count()
    );
    assert!(dt.as_secs_f64() < 20.0, "{dt:?}");
}
