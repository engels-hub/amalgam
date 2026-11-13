#![allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)] // tests, exact expected values

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use amg_core::{Dir3, Plane, Point2, Point3, Quat, Transform, Vec3};
use amg_jobs::JobCtx;
use amg_mesh::{IndexedMesh, MeshBody};
use amg_testkit::shapes;

use super::*;

fn ctx() -> JobCtx {
    JobCtx::detached()
}

fn body(positions: Vec<Point3>, tris: Vec<[u32; 3]>) -> MeshBody {
    MeshBody::new(IndexedMesh::new(positions, tris).unwrap())
}

fn cube(min: Point3, size: f64) -> (Vec<Point3>, Vec<[u32; 3]>) {
    let s = shapes::cube(min, size);
    (s.positions, s.triangles)
}

fn merged(parts: &[(Vec<Point3>, Vec<[u32; 3]>)]) -> MeshBody {
    let (mut p, mut t) = (Vec::new(), Vec::new());
    for (pp, tt) in parts {
        let off = p.len() as u32;
        p.extend_from_slice(pp);
        t.extend(tt.iter().map(|f| f.map(|i| i + off)));
    }
    body(p, t)
}

/// Closed `n`-gon prism of circumradius `r` from z = 0 to `h`, outward CCW faces.
fn prism(n: usize, r: f64, h: f64) -> MeshBody {
    let mut p = Vec::new();
    for z in [0.0, h] {
        for i in 0..n {
            let a = TAU * i as f64 / n as f64;
            p.push(Point3::new(r * a.cos(), r * a.sin(), z));
        }
    }
    let (cb, ct) = (p.len() as u32, p.len() as u32 + 1);
    p.push(Point3::new(0.0, 0.0, 0.0));
    p.push(Point3::new(0.0, 0.0, h));
    let n32 = n as u32;
    let mut t = Vec::new();
    for i in 0..n32 {
        let j = (i + 1) % n32;
        t.push([cb, j, i]);
        t.push([ct, n32 + i, n32 + j]);
        t.push([i, j, n32 + j]);
        t.push([i, n32 + j, n32 + i]);
    }
    body(p, t)
}

fn plane(origin: Point3, normal: Vec3) -> Plane {
    Plane::from_normal(origin, Dir3::new_unchecked(normal.normalize()))
}

fn one(b: &MeshBody, p: Plane) -> (Cap, SectionStats) {
    let r = section_mesh(
        b,
        &Transform::IDENTITY,
        &SectionPlanes(vec![p]),
        &CapOpts::default(),
        &ctx(),
    )
    .unwrap();
    (r.caps[0].clone(), r.stats[0].clone())
}

fn tri_area(c: &Cap) -> f64 {
    c.tris
        .iter()
        .map(|t| {
            let [a, b, d] = t.map(|i| c.uv[i as usize]);
            0.5 * cross2(b - a, d - a)
        })
        .sum()
}

fn shoelace(pl: &Plane, l: &Polyline3) -> f64 {
    let uv: Vec<Point2> = l.points.iter().map(|&p| pl.to_local(p)).collect();
    let n = uv.len();
    (0..n).map(|k| 0.5 * cross2(uv[k], uv[(k + 1) % n])).sum()
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs()
}

#[test]
fn cylinder_perpendicular_cap_area() {
    let (n, r) = (128usize, 10.0);
    let b = prism(n, r, 20.0);
    let p = plane(Point3::new(0.0, 0.0, 7.3), Vec3::Z);
    let (cap, stats) = one(&b, p);
    let exact = 0.5 * n as f64 * r * r * (TAU / n as f64).sin();
    assert!(
        rel(stats.area.get(), exact) < 1e-9,
        "{} vs {exact}",
        stats.area.get()
    );
    assert!(rel(tri_area(&cap), exact) < 1e-9);
    assert_eq!((stats.loops, stats.open_loops), (1, 0));
    assert_eq!(cap.outlines.len(), 1);
    assert!(cap.outlines[0].closed && cap.open.is_empty());
    let perim = n as f64 * 2.0 * r * (PI / n as f64).sin();
    assert!(rel(stats.perimeter.get(), perim) < 1e-9);
    // Every cap triangle faces +normal (CCW in the plane frame).
    for t in &cap.tris {
        let [a, b2, c] = t.map(|i| cap.uv[i as usize]);
        assert!(cross2(b2 - a, c - a) > 0.0);
    }
    // uv is the plane-local position.
    for (w, uv) in cap.positions.iter().zip(&cap.uv) {
        assert!((p.to_local(*w) - *uv).length() < 1e-12);
    }
}

#[test]
fn cylinder_axial_cap_area() {
    let (r, h) = (10.0, 20.0);
    let b = prism(128, r, h);
    // Through ring vertices 0 and 64 and the axis.
    let (_, stats) = one(&b, plane(Point3::ZERO, Vec3::Y));
    assert!(
        rel(stats.area.get(), 2.0 * r * h) < 1e-9,
        "{}",
        stats.area.get()
    );
    assert!(rel(stats.perimeter.get(), 2.0 * (2.0 * r + h)) < 1e-9);
    let e = stats.extent;
    assert!(
        (e.max.x - e.min.x - 2.0 * r).abs() < 1e-9 || (e.max.y - e.min.y - 2.0 * r).abs() < 1e-9
    );
}

#[test]
fn overlapping_shells_cap_their_union() {
    let b = merged(&[
        cube(Point3::ZERO, 10.0),
        cube(Point3::new(5.0, 5.0, 2.0), 10.0),
    ]);
    let p = plane(Point3::new(0.0, 0.0, 6.0), Vec3::Z);
    let (cap, stats) = one(&b, p);
    assert!(rel(stats.area.get(), 175.0) < 1e-12, "{}", stats.area.get());
    assert_eq!(stats.loops, 2);
    assert!(rel(tri_area(&cap), 175.0) < 1e-12);
    // Even–odd leaves the overlap empty.
    let r = section_mesh(
        &b,
        &Transform::IDENTITY,
        &SectionPlanes(vec![p]),
        &CapOpts {
            fill_rule: FillRule::EvenOdd,
            ..CapOpts::default()
        },
        &ctx(),
    )
    .unwrap();
    assert!(rel(r.stats[0].area.get(), 150.0) < 1e-12);
}

#[test]
fn hole_and_island_shoelace_matches_cap_area() {
    // A 40 mm cube with a 128-gon through-hole would need booleans; nest shells instead:
    // outer cube, an inverted inner cube (a cavity) and a small cube inside the cavity.
    let (op, ot) = cube(Point3::new(-20.0, -20.0, -20.0), 40.0);
    let (ip, it) = cube(Point3::new(-10.0, -10.0, -10.0), 20.0);
    let inv: Vec<[u32; 3]> = it.iter().map(|t| [t[0], t[2], t[1]]).collect();
    let island = cube(Point3::new(-3.0, -3.0, -3.0), 6.0);
    let b = merged(&[(op, ot), (ip, inv), island]);
    for (o, n) in [
        (Point3::new(0.0, 0.0, 1.0), Vec3::Z),
        (Point3::new(0.5, -0.2, 0.3), Vec3::new(0.3, 0.5, 0.8)),
    ] {
        let p = plane(o, n);
        let (cap, stats) = one(&b, p);
        assert_eq!(stats.loops, 3);
        let s: f64 = cap.outlines.iter().map(|l| shoelace(&p, l)).sum();
        assert!(
            rel(stats.area.get(), s) < 1e-9,
            "{} vs {s}",
            stats.area.get()
        );
        assert!(rel(tri_area(&cap), s) < 1e-9);
    }
    let (_, stats) = one(&b, plane(Point3::new(0.0, 0.0, 1.0), Vec3::Z));
    assert!(rel(stats.area.get(), 1600.0 - 400.0 + 36.0) < 1e-12);
}

#[test]
fn six_planes_box_caps_stay_inside_the_other_half_spaces() {
    let b = merged(&[cube(Point3::new(-10.0, -10.0, -10.0), 20.0)]);
    let axes = [
        Vec3::X,
        Vec3::NEG_X,
        Vec3::Y,
        Vec3::NEG_Y,
        Vec3::Z,
        Vec3::NEG_Z,
    ];
    // Axis-aligned: the kept region is [-5, 5]³, every cap a 10 × 10 square.
    let planes: Vec<Plane> = axes.iter().map(|&a| plane(a * 5.0, a)).collect();
    let r = section_mesh(
        &b,
        &Transform::IDENTITY,
        &SectionPlanes(planes.clone()),
        &CapOpts::default(),
        &ctx(),
    )
    .unwrap();
    assert_eq!(r.caps.len(), 6);
    for (i, s) in r.stats.iter().enumerate() {
        assert_eq!(s.plane_index as usize, i);
        assert!(rel(s.area.get(), 100.0) < 1e-12, "{i}: {}", s.area.get());
    }
    // Tilted, seeded planes.
    let mut seed = 0x9e37_79b9_7f4a_7c15u64;
    let mut rnd = || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 11) as f64 / (1u64 << 53) as f64 - 0.5
    };
    for _ in 0..20 {
        let planes: Vec<Plane> = axes
            .iter()
            .map(|&a| {
                let n = (a + Vec3::new(rnd(), rnd(), rnd()) * 0.6).normalize();
                plane(n * (5.0 + 2.0 * rnd()), n)
            })
            .collect();
        let r = section_mesh(
            &b,
            &Transform::IDENTITY,
            &SectionPlanes(planes.clone()),
            &CapOpts::default(),
            &ctx(),
        )
        .unwrap();
        for (i, cap) in r.caps.iter().enumerate() {
            assert!(!cap.tris.is_empty());
            for (j, q) in planes.iter().enumerate() {
                if j == i {
                    continue;
                }
                for &v in &cap.positions {
                    assert!(q.signed_distance(v) <= 1e-9, "cap {i} outside plane {j}");
                }
                for l in &cap.outlines {
                    for &v in &l.points {
                        assert!(q.signed_distance(v) <= 1e-9);
                    }
                }
            }
            for &v in &cap.positions {
                assert!(planes[i].signed_distance(v).abs() < 1e-9);
            }
            assert!(rel(tri_area(cap), r.stats[i].area.get()) < 1e-12);
        }
    }
}

#[test]
fn open_cube_reports_open_curve_and_no_cap_through_the_gap() {
    let (p, t) = cube(Point3::ZERO, 10.0);
    // Drop the two faces of the top side (z = 10).
    let t: Vec<[u32; 3]> = t
        .into_iter()
        .filter(|f| !f.iter().all(|&i| p[i as usize].z == 10.0))
        .collect();
    assert_eq!(t.len(), 10);
    let b = body(p, t);
    let (cap, stats) = one(&b, plane(Point3::new(5.0, 0.0, 0.0), Vec3::X));
    assert!(cap.tris.is_empty());
    assert_eq!((stats.loops, stats.open_loops), (0, 1));
    assert_eq!(cap.open.len(), 1);
    assert!(!cap.open[0].closed);
    assert_eq!(stats.area.get(), 0.0);
    // A plane below the gap still cuts a closed loop: capped with the even–odd rule.
    let (cap, stats) = one(&b, plane(Point3::new(0.0, 0.0, 5.0), Vec3::Z));
    assert_eq!((stats.loops, stats.open_loops), (1, 0));
    assert!(rel(stats.area.get(), 100.0) < 1e-12);
    assert!(!cap.tris.is_empty());
}

#[test]
fn transforms_mirrors_and_empty_sections() {
    let b = prism(32, 10.0, 20.0);
    let p0 = plane(Point3::new(0.0, 0.0, 5.0), Vec3::Z);
    let (_, base) = one(&b, p0);
    let rot = Transform::from_rotation_translation(
        Quat::from_axis_angle(Vec3::new(1.0, 2.0, 3.0).normalize(), 0.7),
        Vec3::new(3.0, -4.0, 5.0),
    );
    let mirror = rot * Transform::from_scale(Vec3::new(-1.0, 1.0, 1.0));
    for t in [rot, mirror] {
        let pw = Plane::from_normal(
            t.transform_point3(p0.origin),
            Dir3::new_unchecked(t.transform_vector3(Vec3::Z).normalize()),
        );
        let r = section_mesh(
            &b,
            &t,
            &SectionPlanes(vec![pw]),
            &CapOpts::default(),
            &ctx(),
        )
        .unwrap();
        assert!(rel(r.stats[0].area.get(), base.area.get()) < 1e-9);
        assert!(rel(tri_area(&r.caps[0]), base.area.get()) < 1e-9);
        let s: f64 = r.caps[0].outlines.iter().map(|l| shoelace(&pw, l)).sum();
        assert!(
            rel(s, base.area.get()) < 1e-9,
            "loops keep material on the left"
        );
    }
    // A plane that misses the body.
    let (cap, stats) = one(&b, plane(Point3::new(0.0, 0.0, 50.0), Vec3::Z));
    assert!(cap.tris.is_empty() && cap.outlines.is_empty());
    assert_eq!(stats.extent, Aabb2::EMPTY);
    // No planes.
    let r = section_mesh(
        &b,
        &Transform::IDENTITY,
        &SectionPlanes(Vec::new()),
        &CapOpts::default(),
        &ctx(),
    )
    .unwrap();
    assert!(r.caps.is_empty() && r.stats.is_empty());
}

#[test]
fn errors() {
    let b = prism(8, 10.0, 20.0);
    let p = plane(Point3::ZERO, Vec3::Z);
    let id = Transform::IDENTITY;
    let o = CapOpts::default();
    assert!(matches!(
        section_mesh(&b, &id, &SectionPlanes(vec![p; 7]), &o, &ctx()),
        Err(InspectError::TooManyPlanes(7))
    ));
    let mut bad = p;
    bad.origin.x = f64::NAN;
    assert!(matches!(
        section_mesh(&b, &id, &SectionPlanes(vec![bad]), &o, &ctx()),
        Err(InspectError::Degenerate(_))
    ));
    let flat = Transform::from_scale(Vec3::new(1.0, 1.0, 0.0));
    assert!(matches!(
        section_mesh(&b, &flat, &SectionPlanes(vec![p]), &o, &ctx()),
        Err(InspectError::Degenerate(_))
    ));
    let neg = CapOpts {
        snap_tolerance: -1.0,
        ..o
    };
    assert!(matches!(
        section_mesh(&b, &id, &SectionPlanes(vec![p]), &neg, &ctx()),
        Err(InspectError::Degenerate(_))
    ));
    let token = amg_jobs::CancelToken::new();
    token.cancel();
    let c = JobCtx::with_token(token);
    assert!(matches!(
        section_mesh(&b, &id, &SectionPlanes(vec![p]), &o, &c),
        Err(InspectError::Cancelled)
    ));
}

#[test]
fn section_plane_parameterisation() {
    let base = Plane::XY;
    let p = section_plane(&base, 10.0, [0.0, 0.0], false);
    assert_eq!(p.origin, Point3::new(0.0, 0.0, 10.0));
    assert_eq!((p.x, p.y, p.normal), (base.x, base.y, base.normal));
    let f = section_plane(&base, 10.0, [0.0, 0.0], true);
    assert_eq!(f.normal.get(), Vec3::NEG_Z);
    assert_eq!(f.x.get(), Vec3::X);
    // Tilt about x by 90°: +Z turns to −Y.
    let t = section_plane(&base, 0.0, [FRAC_PI_2, 0.0], false);
    assert!((t.normal.get() - Vec3::NEG_Y).length() < 1e-15);
    assert!((t.x.get() - Vec3::X).length() < 1e-15);
    // Then about the base y by 90°: −Y stays, x turns to −Z.
    let t = section_plane(&base, 0.0, [FRAC_PI_2, FRAC_PI_2], false);
    assert!((t.normal.get() - Vec3::NEG_Y).length() < 1e-15);
    assert!((t.x.get() - Vec3::NEG_Z).length() < 1e-15);
    // Right-handed, unit frame for an arbitrary base.
    let base = plane(Point3::new(1.0, 2.0, 3.0), Vec3::new(0.2, -0.4, 0.9));
    let t = section_plane(&base, -2.5, [0.3, -0.7], true);
    assert!((t.normal.get() - t.x.get().cross(t.y.get())).length() < 1e-14);
    assert!(t.x.get().dot(t.y.get()).abs() < 1e-14);
    assert!((t.origin - (base.origin - base.normal.get() * 2.5)).length() < 1e-14);
}
