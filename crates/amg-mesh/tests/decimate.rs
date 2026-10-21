//! MESH-07 tests: QEM decimation.

#![allow(clippy::unwrap_used, clippy::float_cmp)]

mod common;

use amg_core::{Point2, Point3};
use amg_jobs::{CancelToken, JobCtx};
use amg_mesh::{
    DecimateOpts, DecimateTarget, FaceGroupInfo, FaceId, GroupOrigin, IndexedMesh, MeshError,
    WeldOpts, decimate,
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

/// Icosphere with a smooth low-frequency radial displacement.
fn displaced_sphere(sub: u32) -> IndexedMesh {
    let mut m = icosphere(sub);
    for p in &mut m.positions {
        let u = *p / 10.0;
        let d = 1.0 + 0.08 * (3.0 * u.x).sin() * (2.0 * u.y).cos() + 0.04 * (5.0 * u.z).sin();
        *p *= d;
    }
    m
}

/// Cube of edge `size` with every face split into `n x n` quads.
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
                    // u x v = w: counter-clockwise (outward) on the far face.
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

fn run(m: &IndexedMesh, opts: &DecimateOpts) -> IndexedMesh {
    let (out, rep) = decimate(m, opts, &JobCtx::detached()).unwrap();
    assert_eq!(rep.faces_before, m.face_count());
    assert_eq!(rep.faces_after, out.face_count());
    out.validate().unwrap();
    out
}

fn ratio(r: f64) -> DecimateOpts {
    DecimateOpts {
        target: DecimateTarget::Ratio(r),
        ..DecimateOpts::default()
    }
}

#[test]
fn closed_manifold_stays_closed_and_manifold() {
    let m = icosphere(4);
    let out = run(&m, &ratio(0.1));
    assert!(out.face_count() <= 512);
    assert!(out.face_count() >= 500);
    let mt = metrics(soup(&out));
    assert_eq!(mt.boundary_edges, 0);
    assert_eq!(mt.non_manifold_edges, 0);
    assert_eq!(mt.euler, 2);
    assert!((out.signed_volume() / m.signed_volume() - 1.0).abs() < 0.02);
}

#[test]
fn torus_keeps_genus() {
    let m = common::torus(60, 40, 30.0, 8.0);
    let out = run(&m, &ratio(0.2));
    let mt = metrics(soup(&out));
    assert_eq!(
        (mt.boundary_edges, mt.non_manifold_edges, mt.euler),
        (0, 0, 0)
    );
}

#[test]
fn displaced_sphere_hausdorff_within_budget() {
    let m = displaced_sphere(6); // 81 920 tris
    let out = run(&m, &ratio(0.1));
    assert!(out.face_count() <= 8192);
    let bb = m.aabb();
    let diag = (bb.max - bb.min).length();
    let h = hausdorff(soup(&m), soup(&out), 50_000);
    assert!(h <= 0.002 * diag, "hausdorff {h} > 0.2 % of {diag}");
}

#[test]
fn face_count_target_is_hit() {
    let m = icosphere(5);
    let out = run(
        &m,
        &DecimateOpts {
            target: DecimateTarget::FaceCount(1000),
            ..DecimateOpts::default()
        },
    );
    assert!(
        (999..=1000).contains(&out.face_count()),
        "{}",
        out.face_count()
    );
}

#[test]
fn max_error_target_bounds_deviation() {
    let m = displaced_sphere(5);
    let tol = 0.05;
    let (out, rep) = decimate(
        &m,
        &DecimateOpts {
            target: DecimateTarget::MaxError(tol),
            ..DecimateOpts::default()
        },
        &JobCtx::detached(),
    )
    .unwrap();
    assert!(out.face_count() < m.face_count() / 2);
    assert!(out.face_count() > 12);
    assert!(rep.max_quadric_error <= tol + 1e-12);
    assert!(hausdorff(soup(&m), soup(&out), 20_000) <= 4.0 * tol);
}

#[test]
fn feature_edges_keep_cube_exact() {
    let m = subdivided_cube(10, 10.0);
    assert_eq!(m.face_count(), 1200);
    let out = run(&m, &ratio(0.01));
    assert!(out.face_count() <= 60, "{}", out.face_count());
    let bb = out.aabb();
    assert!(bb.min.distance(Point3::ZERO) < 1e-9);
    assert!(bb.max.distance(Point3::splat(10.0)) < 1e-9);
    assert!((out.signed_volume() - 1000.0).abs() < 1e-6);
    assert!(hausdorff(soup(&m), soup(&out), 20_000) < 1e-6);
    // All 8 corners survive.
    for c in 0..8u32 {
        let corner = Point3::new(
            f64::from(c & 1) * 10.0,
            f64::from((c >> 1) & 1) * 10.0,
            f64::from((c >> 2) & 1) * 10.0,
        );
        assert!(out.positions.iter().any(|p| p.distance(corner) < 1e-9));
    }
}

#[test]
fn without_feature_angle_cube_may_round() {
    let m = subdivided_cube(10, 10.0);
    let out = run(
        &m,
        &DecimateOpts {
            feature_angle: None,
            preserve_boundary: false,
            ..ratio(0.01)
        },
    );
    assert!(out.face_count() < m.face_count());
    let mt = metrics(soup(&out));
    assert_eq!((mt.boundary_edges, mt.non_manifold_edges), (0, 0));
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
fn group_borders_are_kept() {
    let m = two_group_grid();
    let out = run(
        &m,
        &DecimateOpts {
            feature_angle: None,
            ..ratio(0.05)
        },
    );
    assert!(out.face_count() < 160);
    let mut border = 0;
    let mut border_len = 0.0;
    let mut edges = std::collections::HashMap::<(u32, u32), Vec<u32>>::new();
    for (f, t) in out.tris.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            edges
                .entry((a.min(b), a.max(b)))
                .or_default()
                .push(f as u32);
        }
    }
    for (&(a, b), fs) in &edges {
        if fs.len() == 2 && out.groups.group_of(FaceId(fs[0])) != out.groups.group_of(FaceId(fs[1]))
        {
            border += 1;
            let (pa, pb) = (out.positions[a as usize], out.positions[b as usize]);
            assert!((pa.x - 20.0).abs() < 1e-9 && (pb.x - 20.0).abs() < 1e-9);
            border_len += pa.distance(pb);
        }
    }
    assert!(border >= 1);
    assert!(
        (border_len - 40.0).abs() < 1e-9,
        "border length {border_len}"
    );
    // Outer boundary kept as well.
    let bb = out.aabb();
    assert_eq!(
        (bb.min, bb.max),
        (Point3::ZERO, Point3::new(40.0, 40.0, 0.0))
    );
    let perimeter: f64 = edges
        .iter()
        .filter(|(_, f)| f.len() == 1)
        .map(|(&(a, b), _)| out.positions[a as usize].distance(out.positions[b as usize]))
        .sum();
    assert!((perimeter - 160.0).abs() < 1e-9);
    // Both groups survive.
    let mut seen = std::collections::HashSet::new();
    for f in 0..out.face_count() as u32 {
        seen.insert(out.groups.group_of(FaceId(f)));
    }
    assert_eq!(seen.len(), 2);
}

#[test]
fn bit_identical_across_thread_counts() {
    let m = displaced_sphere(5);
    let hash_with = |threads: usize| {
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap()
            .install(|| run(&m, &ratio(0.1)).content_hash())
    };
    let h1 = hash_with(1);
    assert_eq!(h1, hash_with(4));
    assert_eq!(h1, hash_with(13));
    assert_eq!(h1, run(&m, &ratio(0.1)).content_hash());
}

#[test]
fn attributes_follow_options() {
    let mut m = icosphere(3);
    let nf = m.face_count();
    m.attrs.corner_uvs = Some(vec![[Point2::ZERO, Point2::X, Point2::Y]; nf]);
    m.attrs.vertex_normals = Some(vec![Point3::Z; m.vertex_count()]);
    let (out, rep) = decimate(&m, &ratio(0.25), &JobCtx::detached()).unwrap();
    assert_eq!(rep.dropped_attributes, vec!["vertex_normals"]);
    assert_eq!(
        out.attrs.corner_uvs.as_ref().unwrap().len(),
        out.face_count()
    );
    let (out, rep) = decimate(
        &m,
        &DecimateOpts {
            preserve_attributes: false,
            ..ratio(0.25)
        },
        &JobCtx::detached(),
    )
    .unwrap();
    assert_eq!(rep.dropped_attributes, vec!["vertex_normals", "corner_uvs"]);
    assert!(out.attrs.corner_uvs.is_none());
}

#[test]
fn errors_and_noops() {
    let ctx = JobCtx::detached();
    let empty = IndexedMesh::default();
    assert!(matches!(
        decimate(&empty, &ratio(0.5), &ctx),
        Err(MeshError::Empty)
    ));
    let m = icosphere(2);
    assert!(matches!(
        decimate(&m, &ratio(0.0), &ctx),
        Err(MeshError::InvalidOption { .. })
    ));
    assert!(matches!(
        decimate(&m, &ratio(f64::NAN), &ctx),
        Err(MeshError::InvalidOption { .. })
    ));
    let (same, rep) = decimate(&m, &ratio(1.0), &ctx).unwrap();
    assert_eq!(rep.faces_after, m.face_count());
    assert_eq!(same.tris, m.tris);
    let tok = CancelToken::new();
    tok.cancel();
    assert!(matches!(
        decimate(&icosphere(5), &ratio(0.1), &JobCtx::with_token(tok)),
        Err(MeshError::Cancelled)
    ));
}

#[test]
fn open_mesh_boundary_is_kept() {
    let m = common::grid(30, 30, |i| i % 3 == 0);
    let out = run(&m, &ratio(0.05));
    let bb = out.aabb();
    assert_eq!(
        (bb.min, bb.max),
        (Point3::ZERO, Point3::new(30.0, 30.0, 0.0))
    );
    let mt = metrics(soup(&out));
    assert_eq!((mt.non_manifold_edges, mt.euler), (0, 1));
    assert!((out.area() - 900.0).abs() < 1e-6);
}

#[test]
fn golden_decimate_displaced_sphere() {
    let out = run(&displaced_sphere(5), &ratio(0.05));
    amg_testkit::assert_golden!("mesh", "decimate_displaced_sphere_s5_5pct", soup(&out));
}

#[test]
fn golden_decimate_cube() {
    let out = run(&subdivided_cube(10, 10.0), &ratio(0.01));
    amg_testkit::assert_golden!("mesh", "decimate_cube_grid10", soup(&out));
}

#[test]
fn golden_decimate_torus() {
    let out = run(&common::torus(80, 50, 30.0, 8.0), &ratio(0.1));
    amg_testkit::assert_golden!("mesh", "decimate_torus_80x50_10pct", soup(&out));
}
