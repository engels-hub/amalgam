//! MESH-10 part 1: quick normal-offset shell (`offset_shell_normal`).

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::cast_precision_loss,
    clippy::print_stderr
)]

#[path = "common/mod.rs"]
mod common;

use std::time::Instant;

use amg_core::{Point3, Vec3};
use amg_jobs::{CancelToken, JobCtx};
use amg_mesh::corpus::bench::{BenchSize, benchmark_sphere};
use amg_mesh::{
    AnalysisReport, AnalyzeOpts, FaceId, GroupOrigin, IndexedMesh, MeshError, NormalOffsetOpts,
    ShellReport, ShellSide, analyze, offset_shell_normal, offset_shell_normal_unchecked,
    separate_shells,
};
use amg_testkit::{TriSoup, shapes};

fn ctx() -> JobCtx {
    JobCtx::detached()
}

fn soup(m: &IndexedMesh) -> TriSoup<'_> {
    TriSoup {
        positions: &m.positions,
        triangles: &m.tris,
    }
}

fn report(m: &IndexedMesh) -> AnalysisReport {
    analyze(m, &AnalyzeOpts::default(), &ctx()).unwrap()
}

fn icosphere(radius: f64, sub: u32) -> IndexedMesh {
    let s = shapes::icosphere(Point3::ZERO, radius, sub);
    IndexedMesh::new(s.positions, s.triangles).unwrap()
}

fn cube(size: f64) -> IndexedMesh {
    let s = shapes::cube(Point3::ZERO, size);
    IndexedMesh::new(s.positions, s.triangles).unwrap()
}

fn shell(m: &IndexedMesh, t: f64, side: ShellSide) -> (IndexedMesh, ShellReport) {
    offset_shell_normal(
        m,
        &NormalOffsetOpts {
            thickness: t,
            side,
            open_faces: Vec::new(),
        },
        &ctx(),
    )
    .unwrap()
}

fn sphere_vol(r: f64) -> f64 {
    4.0 / 3.0 * std::f64::consts::PI * r.powi(3)
}

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}

#[test]
fn cube_inside_is_exact() {
    let m = cube(10.0);
    let (s, r) = shell(&m, 1.0, ShellSide::Inside);
    let a = report(&s);
    assert!(a.is_watertight(), "{a:?}");
    assert_eq!(a.shells.len(), 2);
    assert_eq!(r.self_intersecting_pairs, 0);
    // The corner stretch moves every face plane by exactly t.
    assert!((s.signed_volume() - (1000.0 - 512.0)).abs() < 1e-9);
    let shells = separate_shells(&s);
    let inner = shells[1].aabb();
    assert!((inner.min - Point3::splat(1.0)).abs().max_element() < 1e-12);
    assert!((inner.max - Point3::splat(9.0)).abs().max_element() < 1e-12);
}

#[test]
fn cube_outside_and_both_volumes() {
    let m = cube(10.0);
    let (s, r) = shell(&m, 1.0, ShellSide::Outside);
    assert!(report(&s).is_watertight());
    assert_eq!(r.self_intersecting_pairs, 0);
    assert!((s.signed_volume() - (1728.0 - 1000.0)).abs() < 1e-9);
    let (s, r) = shell(&m, 2.0, ShellSide::Both);
    assert!(report(&s).is_watertight());
    assert_eq!(r.self_intersecting_pairs, 0);
    assert!((s.signed_volume() - (1728.0 - 512.0)).abs() < 1e-9);
}

#[test]
fn sphere_sides() {
    let m = icosphere(10.0, 5);
    let v_in = m.signed_volume();
    let ratio = v_in / sphere_vol(10.0);
    for (side, ro, ri) in [
        (ShellSide::Inside, 10.0, 9.0),
        (ShellSide::Outside, 11.0, 10.0),
        (ShellSide::Both, 10.5, 9.5),
    ] {
        let (s, r) = shell(&m, 1.0, side);
        let a = report(&s);
        assert!(a.is_watertight(), "{side:?}: {a:?}");
        assert_eq!(a.shells.len(), 2, "{side:?}");
        assert_eq!(r.self_intersecting_pairs, 0, "{side:?}");
        let expect = ratio * (sphere_vol(ro) - sphere_vol(ri));
        let v = s.signed_volume();
        assert!(
            (v - expect).abs() < 2e-3 * expect,
            "{side:?}: {v} vs {expect}"
        );
        // Wall thickness: inner vertices at distance t from the outer sphere's vertices.
        let shells = separate_shells(&s);
        let rad = |m: &IndexedMesh| m.positions.iter().map(|p| p.length()).fold(0.0, f64::max);
        let w = rad(&shells[0]) - rad(&shells[1]);
        assert!((w - 1.0).abs() < 1e-3, "{side:?}: wall {w}");
    }
}

#[test]
fn torus_inside_is_clean() {
    let m = common::torus(96, 48, 10.0, 3.0);
    let (s, r) = shell(&m, 1.0, ShellSide::Inside);
    assert!(report(&s).is_watertight());
    assert_eq!(r.self_intersecting_pairs, 0);
    let pi2 = std::f64::consts::PI.powi(2);
    let expect = 2.0 * pi2 * 10.0 * (9.0 - 4.0);
    let v = s.signed_volume();
    assert!((v - expect).abs() < 0.01 * expect, "{v} vs {expect}");
}

#[test]
fn open_shell_has_a_rim() {
    let m = cube(10.0);
    // The two faces of the top (z = 10) side.
    let top: Vec<FaceId> = (0..m.face_count() as u32)
        .map(FaceId)
        .filter(|&f| m.triangle(f).iter().all(|p| p.z == 10.0))
        .collect();
    assert_eq!(top.len(), 2);
    let (s, r) = offset_shell_normal(
        &m,
        &NormalOffsetOpts {
            thickness: 1.0,
            side: ShellSide::Inside,
            open_faces: top,
        },
        &ctx(),
    )
    .unwrap();
    let a = report(&s);
    assert!(a.is_watertight(), "{a:?}");
    assert_eq!(a.shells.len(), 1);
    assert_eq!(r.self_intersecting_pairs, 0);
    // A box open at the top: 10³ minus the 8 × 8 × 9 cavity.
    assert!((s.signed_volume() - (1000.0 - 576.0)).abs() < 1e-9);
    // 10 + 10 input faces, 4 boundary edges → 8 rim triangles.
    assert_eq!(s.face_count(), 28);
}

#[test]
fn open_input_sheet_becomes_a_slab() {
    let m = common::grid(4, 3, |i| i % 2 == 0);
    let (s, r) = shell(&m, 0.5, ShellSide::Outside);
    let a = report(&s);
    assert!(a.is_watertight(), "{a:?}");
    assert_eq!(r.self_intersecting_pairs, 0);
    let v = s.signed_volume().abs();
    assert!((v - 12.0 * 0.5).abs() < 1e-9, "{v}");
}

#[test]
fn groups() {
    let m = icosphere(5.0, 2);
    let g0 = m.groups.group_of(FaceId(0));
    let nf = m.face_count();
    for side in [ShellSide::Inside, ShellSide::Outside, ShellSide::Both] {
        let (s, _) = shell(&m, 0.5, side);
        assert_eq!(s.face_count(), 2 * nf);
        let (orig, off) = match side {
            ShellSide::Outside => (nf, 0),
            _ => (0, nf),
        };
        assert_eq!(s.groups.group_of(FaceId(orig as u32)), g0);
        let g = s.groups.group_of(FaceId(off as u32));
        let want = if side == ShellSide::Outside {
            ShellSide::Outside
        } else {
            ShellSide::Inside
        };
        assert_eq!(
            s.groups.info(g).unwrap().origin,
            GroupOrigin::Shell { side: want }
        );
        s.groups.validate(s.face_count()).unwrap();
    }
}

#[test]
fn self_intersections_are_counted() {
    // A uniform offset of a convex body stays clean even when it turns inside out
    // (concentric spheres do not cross).
    let (_, r) = shell(&icosphere(2.0, 3), 5.0, ShellSide::Inside);
    assert_eq!(r.self_intersecting_pairs, 0);
    // Two cubes 1 mm apart, offset outward by 1 mm: the offset copies overlap.
    let mut m = cube(4.0);
    let mut b = cube(4.0);
    b.transform(&amg_core::Transform::from_translation(Vec3::new(
        5.0, 0.3, 0.2,
    )));
    m.append(&b).unwrap();
    let (_, r) = shell(&m, 1.0, ShellSide::Outside);
    eprintln!(
        "two cubes, gap 1, t = 1: {} pairs",
        r.self_intersecting_pairs
    );
    assert!(r.self_intersecting_pairs > 0);
}

#[test]
fn errors() {
    let m = cube(1.0);
    for t in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        let e = offset_shell_normal(
            &m,
            &NormalOffsetOpts {
                thickness: t,
                side: ShellSide::Inside,
                open_faces: vec![],
            },
            &ctx(),
        );
        assert!(matches!(e, Err(MeshError::InvalidOption { .. })), "{t}");
    }
    let o = |open_faces| NormalOffsetOpts {
        thickness: 0.1,
        side: ShellSide::Inside,
        open_faces,
    };
    assert!(matches!(
        offset_shell_normal(&m, &o(vec![FaceId(12)]), &ctx()),
        Err(MeshError::InvalidOption { .. })
    ));
    let all = (0..12).map(FaceId).collect();
    assert!(matches!(
        offset_shell_normal(&m, &o(all), &ctx()),
        Err(MeshError::Empty)
    ));
    let tok = CancelToken::new();
    tok.cancel();
    assert!(matches!(
        offset_shell_normal(&m, &o(vec![]), &JobCtx::with_token(tok)),
        Err(MeshError::Cancelled)
    ));
}

#[test]
fn identical_across_thread_counts() {
    let m = common::torus(64, 32, 10.0, 3.0);
    let run = |n: usize| {
        rayon::ThreadPoolBuilder::new()
            .num_threads(n)
            .build()
            .unwrap()
            .install(|| shell(&m, 0.8, ShellSide::Both).0.content_hash())
    };
    let h = run(1);
    assert_eq!(h, run(4));
    assert_eq!(h, run(13));
}

#[test]
fn unreferenced_vertices_are_dropped() {
    let mut m = cube(2.0);
    m.positions.push(Point3::new(100.0, 0.0, 0.0));
    let (s, _) = shell(&m, 0.5, ShellSide::Inside);
    assert_eq!(s.vertex_count(), 16);
    assert!(s.aabb().max.x <= 2.0);
}

#[test]
fn golden_quick_shells() {
    let (s, _) = shell(&icosphere(10.0, 3), 1.5, ShellSide::Inside);
    amg_testkit::assert_golden!("mesh", "shell_quick_sphere_r10_inside", soup(&s));
    let mut c = cube(6.0);
    c.transform(&amg_core::Transform::from_translation(Vec3::new(
        1.0, 2.0, 3.0,
    )));
    let (s, _) = shell(&c, 1.0, ShellSide::Both);
    amg_testkit::assert_golden!("mesh", "shell_quick_cube6_both", soup(&s));
}

/// S07 budget: quick shell build of 1 M triangles < 300 ms (asserted only with
/// `AMG_STRICT_TIMING=1`). The SI count is reported and timed separately (coordinator reading
/// of the budget, S07 log 2026-11-11); criterion bench `benches/mesh_offset.rs`.
#[test]
fn timing_quick_shell_1m() {
    let m = benchmark_sphere(BenchSize::M1, 1);
    let opts = NormalOffsetOpts {
        thickness: 0.2,
        side: ShellSide::Inside,
        open_faces: Vec::new(),
    };
    let t0 = Instant::now();
    let built = offset_shell_normal_unchecked(&m, &opts, &ctx()).unwrap();
    let t_build = t0.elapsed();
    let t1 = Instant::now();
    let (s, r) = offset_shell_normal(&m, &opts, &ctx()).unwrap();
    let t_full = t1.elapsed();
    eprintln!(
        "quick shell {} tris -> {} tris, {} SI pairs: build {t_build:?}, with SI count          {t_full:?}",
        m.face_count(),
        s.face_count(),
        r.self_intersecting_pairs
    );
    assert_eq!(s.face_count(), 2 * m.face_count());
    assert_eq!(built.content_hash(), s.content_hash());
    if strict_timing() {
        assert!(t_build.as_secs_f64() < 0.3, "{t_build:?}");
    }
}
