//! AM-03: placement, face-down, fit check, hull overlap and build-volume geometry.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::print_stderr,
    clippy::cast_precision_loss,
    missing_docs,
    reason = "tests"
)]

use amg_am::AmError;
use amg_am::machine::{BedShape, BuildVolume};
use amg_am::place::{
    EXCLUDE_THICKNESS, PLATE_THICKNESS, Placement, ZMode, build_volume_geometry, centre_on_plate,
    check_fit, effective_transform, face_down_rotation, footprint_hull, hulls_overlap,
    pick_plane_normal, rotate_about_build_axis,
};
use amg_core::glam::{DQuat, DVec2, DVec3};
use amg_core::{Aabb3, Transform};
use amg_jobs::{CancelToken, JobCtx};
use amg_mesh::corpus::{FileStatus, Manifest, corpus_root, read_mesh};
use amg_mesh::{FaceGroupInfo, FaceId, GroupOrigin, IndexedMesh, Primitive};

// ------------------------------------------------------------------------- helpers

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
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.f()
    }
    fn quat(&mut self) -> DQuat {
        let z = self.range(-1.0, 1.0);
        let t = self.range(0.0, std::f64::consts::TAU);
        let s = (1.0 - z * z).sqrt();
        DQuat::from_axis_angle(
            DVec3::new(s * t.cos(), s * t.sin(), z),
            self.range(0.0, std::f64::consts::TAU),
        )
    }
}

fn require_testdata() -> bool {
    std::env::var("AMG_REQUIRE_TESTDATA").is_ok_and(|v| v == "1")
}

fn mesh_of(t: amg_testkit::shapes::TriMesh) -> IndexedMesh {
    IndexedMesh::new(t.positions, t.triangles).unwrap()
}

fn cube(min: DVec3, size: f64) -> IndexedMesh {
    mesh_of(amg_testkit::shapes::cube(min, size))
}

fn sphere(c: DVec3, r: f64, sub: u32) -> IndexedMesh {
    mesh_of(amg_testkit::shapes::icosphere(c, r, sub))
}

/// Regular `n`-gon prism of radius `r`, height `h`, with fan caps.
fn prism(n: usize, r: f64, h: f64) -> IndexedMesh {
    let mut pos = Vec::new();
    for z in [0.0, h] {
        for i in 0..n {
            let a = std::f64::consts::TAU * i as f64 / n as f64;
            pos.push(DVec3::new(r * a.cos(), r * a.sin(), z));
        }
    }
    pos.push(DVec3::new(0.0, 0.0, 0.0));
    pos.push(DVec3::new(0.0, 0.0, h));
    let (n32, bc, tc) = (n as u32, 2 * n as u32, 2 * n as u32 + 1);
    let mut tris = Vec::new();
    for i in 0..n32 {
        let j = (i + 1) % n32;
        tris.push([i, j, n32 + j]);
        tris.push([i, n32 + j, n32 + i]);
        tris.push([bc, j, i]);
        tris.push([tc, n32 + i, n32 + j]);
    }
    IndexedMesh::new(pos, tris).unwrap()
}

fn angle_between(a: DVec3, b: DVec3) -> f64 {
    a.cross(b).length().atan2(a.dot(b))
}

fn transformed(m: &IndexedMesh, t: &Transform) -> Vec<DVec3> {
    m.positions.iter().map(|p| t.transform_point3(*p)).collect()
}

/// Corpus meshes (up to `count`, at most `max_faces` each); un-fetched LFS pointers are
/// skipped unless `AMG_REQUIRE_TESTDATA=1`.
fn corpus_meshes(count: usize, max_faces: usize) -> Vec<(String, IndexedMesh)> {
    let manifest = Manifest::load().expect("corpus.toml");
    let root = corpus_root();
    let mut out = Vec::new();
    for e in manifest.meshes.iter().filter(|e| e.faces <= max_faces) {
        if out.len() >= count {
            break;
        }
        match e.locate(&root) {
            FileStatus::Ready(p) => {
                if let Ok(m) = read_mesh(&p) {
                    out.push((e.id.clone(), m));
                }
            }
            FileStatus::LfsPointer(p) => {
                assert!(
                    !require_testdata(),
                    "{} is an un-fetched git-LFS pointer and AMG_REQUIRE_TESTDATA=1",
                    p.display()
                );
            }
            FileStatus::NotFetched(_) => {}
            FileStatus::Missing(p) => panic!("committed corpus file {} is missing", p.display()),
        }
    }
    if out.is_empty() {
        eprintln!("SKIP corpus: no corpus mesh available (LFS pointers); synthetic meshes only");
    }
    out
}

fn synthetic_meshes() -> Vec<(String, IndexedMesh)> {
    vec![
        ("cube".into(), cube(DVec3::new(3.0, -4.0, 5.0), 10.0)),
        ("sphere".into(), sphere(DVec3::new(1.0, 2.0, 3.0), 12.0, 3)),
        ("prism12".into(), prism(12, 9.0, 17.0)),
    ]
}

/// Reference hull: plain monotone chain over every transformed vertex, no pre-filter.
fn reference_hull(points: &[DVec2]) -> Vec<DVec2> {
    let mut pts = points.to_vec();
    pts.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    pts.dedup();
    if pts.len() < 3 {
        return pts;
    }
    let cross = |o: DVec2, a: DVec2, b: DVec2| (a - o).perp_dot(b - o);
    let mut h: Vec<DVec2> = Vec::new();
    for &p in &pts {
        while h.len() >= 2 && cross(h[h.len() - 2], h[h.len() - 1], p) <= 0.0 {
            h.pop();
        }
        h.push(p);
    }
    let lower = h.len() + 1;
    for &p in pts.iter().rev().skip(1) {
        while h.len() >= lower && cross(h[h.len() - 2], h[h.len() - 1], p) <= 0.0 {
            h.pop();
        }
        h.push(p);
    }
    h.pop();
    h
}

// ------------------------------------------------------- effective transform (item 1)

fn check_placements(name: &str, m: &IndexedMesh, rng: &mut Rng, n: usize) {
    for k in 0..n {
        let rot = rng.quat();
        let xy = DVec2::new(rng.range(-100.0, 300.0), rng.range(-100.0, 300.0));
        let elevation = if k % 3 == 0 { rng.range(0.0, 8.0) } else { 0.0 };
        let p = Placement {
            rotation: rot,
            xy,
            z: ZMode::OnPlate { elevation },
        };
        let t = effective_transform(m, &p);
        let pts = transformed(m, &t);
        let bb = Aabb3::from_points(pts.iter().copied());
        assert!(
            (bb.min.z - elevation).abs() < 1e-9,
            "{name} #{k}: min z {} vs {elevation}",
            bb.min.z
        );
        let c = bb.center();
        assert!(
            (c.x - xy.x).abs() < 1e-9 && (c.y - xy.y).abs() < 1e-9,
            "{name} #{k}: centre ({}, {}) vs {xy}",
            c.x,
            c.y
        );
        // Rigid: the rotation part is exactly the (normalised) placement rotation.
        let q = DQuat::from_mat3(&t.matrix3);
        assert!(q.dot(rot.normalize()).abs() > 1.0 - 1e-12);
    }
}

#[test]
fn thousand_random_rotations_land_on_plate_at_xy() {
    let mut rng = Rng(1);
    let mut meshes = synthetic_meshes();
    meshes.extend(corpus_meshes(6, 60_000));
    for (name, m) in &meshes {
        check_placements(name, m, &mut rng, 1000);
    }
}

#[test]
fn fixed_z_mode_and_empty_mesh() {
    let m = cube(DVec3::ZERO, 10.0);
    let p = Placement {
        rotation: DQuat::from_rotation_x(0.3),
        xy: DVec2::new(50.0, 60.0),
        z: ZMode::Fixed { z: 7.5 },
    };
    let t = effective_transform(&m, &p);
    let bb = Aabb3::from_points(transformed(&m, &t));
    let q = Placement {
        z: ZMode::OnPlate { elevation: 0.0 },
        ..p
    };
    let bb0 = Aabb3::from_points(transformed(&m, &effective_transform(&m, &q)));
    assert!((bb.min.z - (bb0.min.z + 7.5)).abs() < 1e-9);
    let e = IndexedMesh::new(Vec::new(), Vec::new()).unwrap();
    let te = effective_transform(&e, &p);
    assert_eq!(te.translation, DVec3::new(50.0, 60.0, 7.5));
    let fit = check_fit(&e, &te, &vol_rect(), &JobCtx::detached()).unwrap();
    assert!(fit.fits && fit.footprint_hull.is_empty() && fit.aabb.is_empty());
}

#[test]
fn effective_transform_is_independent_of_thread_count() {
    let m = sphere(DVec3::new(2.0, 3.0, 4.0), 30.0, 6); // 40 962 vertices: parallel path
    let p = Placement::on_plate(
        DQuat::from_euler(glam_euler(), 0.4, 1.1, -0.7),
        DVec2::new(80.0, 90.0),
    );
    let run = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap();
        pool.install(|| {
            let t = effective_transform(&m, &p);
            let fit = check_fit(&m, &t, &vol_rect(), &JobCtx::detached()).unwrap();
            (t, fit)
        })
    };
    let (t1, f1) = run(1);
    let (t4, f4) = run(4);
    assert_eq!(t1, t4);
    assert_eq!(f1, f4);
}

fn glam_euler() -> amg_core::glam::EulerRot {
    amg_core::glam::EulerRot::XYZ
}

// ------------------------------------------------------------------ face down (item 2)

fn check_face_down(m: &IndexedMesh, current: DQuat, what: &str) {
    for f in 0..m.face_count() {
        let n = m.face_normal(FaceId(f as u32));
        let q = face_down_rotation(current, n);
        let got = q * n;
        let err = angle_between(got, DVec3::NEG_Z);
        assert!(err < 1e-12, "{what} face {f}: {err} rad");
        assert!((q.length() - 1.0).abs() < 1e-12);
    }
}

#[test]
fn face_down_on_cube_and_prism_faces() {
    let mut rng = Rng(2);
    let cube = cube(DVec3::ZERO, 10.0);
    let prism = prism(12, 8.0, 15.0);
    for m in [&cube, &prism] {
        check_face_down(m, DQuat::IDENTITY, "identity");
        for _ in 0..20 {
            check_face_down(m, rng.quat(), "random");
        }
        // Axis-aligned currents hit the anti-parallel case exactly for some faces.
        for q in [
            DQuat::from_rotation_x(std::f64::consts::PI),
            DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2),
            DQuat::from_rotation_x(-std::f64::consts::FRAC_PI_2),
            DQuat::from_rotation_y(std::f64::consts::FRAC_PI_2),
        ] {
            check_face_down(m, q, "axis");
        }
    }
}

#[test]
fn face_down_anti_parallel_and_already_down() {
    // Face pointing exactly up: 180 degrees about plate X.
    let up = face_down_rotation(DQuat::IDENTITY, DVec3::Z);
    assert!(angle_between(up * DVec3::Z, DVec3::NEG_Z) < 1e-15);
    assert!(
        (up * DVec3::X - DVec3::X).length() < 1e-15,
        "X is the turn axis"
    );
    // Slightly off anti-parallel stays accurate.
    for eps in [1e-3, 1e-6, 1e-9, 1e-12] {
        let n = DVec3::new(eps, 0.0, 1.0).normalize();
        let q = face_down_rotation(DQuat::IDENTITY, n);
        assert!(angle_between(q * n, DVec3::NEG_Z) < 1e-12, "eps {eps}");
    }
    // Already facing down: unchanged, yaw kept.
    let yaw = DQuat::from_rotation_z(0.9);
    let q = face_down_rotation(yaw, DVec3::NEG_Z);
    assert!(q.dot(yaw).abs() > 1.0 - 1e-14);
    // A tilted face is levelled by the shortest arc; the arc axis is horizontal, so the
    // rotation about plate Z changes only by the part the arc forces.
    let cur = DQuat::from_rotation_z(0.5);
    let n = DVec3::new(0.0, 0.0, -1.0);
    assert!(face_down_rotation(cur, n).dot(cur).abs() > 1.0 - 1e-14);
    // Zero normal is a no-op.
    assert_eq!(face_down_rotation(yaw, DVec3::ZERO), yaw);
}

// ------------------------------------------------------------------------ fit (item 3)

fn vol_rect() -> BuildVolume {
    BuildVolume {
        bed: BedShape::Rect {
            size: DVec2::new(220.0, 220.0),
        },
        height: 250.0,
        exclude: Vec::new(),
        slicer_origin: DVec2::ZERO,
    }
}

fn dist_seg(p: DVec2, a: DVec2, b: DVec2) -> f64 {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0);
    (p - (a + ab * t)).length()
}

fn in_poly(p: DVec2, poly: &[DVec2]) -> bool {
    // Winding-free crossing count along +x.
    let mut inside = false;
    for i in 0..poly.len() {
        let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
        if (a.y <= p.y) != (b.y <= p.y) {
            let x = a.x + (p.y - a.y) / (b.y - a.y) * (b.x - a.x);
            if x > p.x {
                inside = !inside;
            }
        }
    }
    inside
}

/// Independent bed distance: 0 inside, else distance to the outline (exact circle).
fn oracle_out(bed: &BedShape, p: DVec2) -> f64 {
    match bed {
        BedShape::Circle { diameter } => {
            ((p - DVec2::splat(diameter / 2.0)).length() - diameter / 2.0).max(0.0)
        }
        BedShape::Rect { size } => {
            let d = DVec2::new(
                (-p.x).max(p.x - size.x).max(0.0),
                (-p.y).max(p.y - size.y).max(0.0),
            );
            d.length()
        }
        BedShape::Polygon { points } => {
            if in_poly(p, points) {
                0.0
            } else {
                (0..points.len())
                    .map(|i| dist_seg(p, points[i], points[(i + 1) % points.len()]))
                    .fold(f64::INFINITY, f64::min)
            }
        }
    }
}

/// Max of `f` on a segment by repeated zoomed dense sampling (independent of the library).
fn zoom_max(f: &dyn Fn(DVec2) -> f64, a: DVec2, b: DVec2) -> f64 {
    let (mut lo, mut hi) = (0.0f64, 1.0f64);
    let mut best = 0.0f64;
    for _ in 0..4 {
        let n = 4000;
        let mut bt = lo;
        for i in 0..=n {
            let t = lo + (hi - lo) * i as f64 / n as f64;
            let v = f(a + (b - a) * t);
            if v > best {
                best = v;
                bt = t;
            }
        }
        let w = (hi - lo) / n as f64 * 2.0;
        (lo, hi) = ((bt - w).max(0.0), (bt + w).min(1.0));
    }
    best
}

/// Area of the convex polygon `subject` clipped by convex CCW `clip` (Sutherland–Hodgman).
fn clip_area(subject: &[DVec2], clip: &[DVec2]) -> f64 {
    let mut out: Vec<DVec2> = subject.to_vec();
    for i in 0..clip.len() {
        let (a, b) = (clip[i], clip[(i + 1) % clip.len()]);
        let inside = |p: DVec2| (b - a).perp_dot(p - a) >= 0.0;
        let input = std::mem::take(&mut out);
        for j in 0..input.len() {
            let (p, q) = (input[j], input[(j + 1) % input.len()]);
            let (ip, iq) = (inside(p), inside(q));
            if ip != iq {
                let (s, e) = (q - p, b - a);
                let t = (a - p).perp_dot(e) / s.perp_dot(e);
                out.push(p + s * t);
            }
            if iq {
                out.push(q);
            }
        }
        if out.is_empty() {
            return 0.0;
        }
    }
    let mut s = 0.0;
    for i in 0..out.len() {
        s += out[i].perp_dot(out[(i + 1) % out.len()]);
    }
    s.abs() * 0.5
}

fn rect_poly(lo: DVec2, hi: DVec2) -> Vec<DVec2> {
    vec![lo, DVec2::new(hi.x, lo.y), hi, DVec2::new(lo.x, hi.y)]
}

fn fit_case(name: &str, vol: &BuildVolume, meshes: &[IndexedMesh], seed: u64) {
    let mut rng = Rng(seed);
    let ctx = JobCtx::detached();
    let (mut fits_n, mut nofit_n, mut hit_n, mut skipped) = (0, 0, 0, 0);
    let (lo, hi) = vol.bed_rect();
    for k in 0..200 {
        let m = &meshes[k % meshes.len()];
        let p = Placement {
            rotation: rng.quat(),
            xy: DVec2::new(
                rng.range(lo.x - 30.0, hi.x + 30.0),
                rng.range(lo.y - 30.0, hi.y + 30.0),
            ),
            z: ZMode::OnPlate {
                elevation: rng.range(0.0, 260.0 - 250.0 + 6.0)
                    * if k % 5 == 0 { 40.0 } else { 0.0 },
            },
        };
        let t = effective_transform(m, &p);
        let rep = check_fit(m, &t, vol, &ctx).unwrap();
        let pts = transformed(m, &t);
        let xy: Vec<DVec2> = pts.iter().map(|p| p.truncate()).collect();
        let hull = reference_hull(&xy);
        assert_eq!(rep.footprint_hull, hull, "{name} #{k}: hull");
        // Height.
        let max_z = pts.iter().map(|p| p.z).fold(f64::NEG_INFINITY, f64::max);
        let h_excess = (max_z - vol.height).max(0.0);
        assert!(
            (rep.height_excess - h_excess).abs() < 1e-6,
            "{name} #{k}: height"
        );
        // Outside distance: vertices for convex beds, dense edge sampling for non-convex.
        let convex = !matches!(&vol.bed, BedShape::Polygon { .. });
        let f = |q: DVec2| oracle_out(&vol.bed, q);
        let mut out = pts.iter().map(|p| f(p.truncate())).fold(0.0, f64::max);
        if !convex {
            for i in 0..hull.len() {
                out = out.max(zoom_max(&f, hull[i], hull[(i + 1) % hull.len()]));
            }
        }
        assert!(
            (rep.max_outside_xy - out).abs() < 1.01e-6,
            "{name} #{k}: outside {} vs {out}",
            rep.max_outside_xy
        );
        // Exclusion: clipped area of the convex keep-out against the hull.
        let mut hits = false;
        let mut ambiguous = false;
        for e in &vol.exclude {
            let a = clip_area(e, &hull);
            if a > 1e-4 {
                hits = true;
            } else if a > 0.0 {
                ambiguous = true;
            }
        }
        if ambiguous && !hits {
            skipped += 1;
            continue;
        }
        assert_eq!(rep.hits_exclude, hits, "{name} #{k}: exclude");
        let near_tol = (out - 1e-6).abs() < 2e-7 || (h_excess - 1e-6).abs() < 2e-7;
        if near_tol {
            skipped += 1;
            continue;
        }
        let expect = h_excess <= 1e-6 && out <= 1e-6 && !hits;
        assert_eq!(rep.fits, expect, "{name} #{k}: fits");
        if rep.fits {
            fits_n += 1;
        } else {
            nofit_n += 1;
        }
        hit_n += usize::from(rep.hits_exclude);
    }
    eprintln!("{name}: fits {fits_n}, not {nofit_n}, exclude hits {hit_n}, skipped {skipped}");
    assert!(fits_n >= 10 && nofit_n >= 10, "{name}: need both outcomes");
}

#[test]
fn check_fit_matches_brute_force_on_rect_polygon_and_circle_beds() {
    let meshes = vec![
        cube(DVec3::new(-5.0, -5.0, -5.0), 10.0),
        sphere(DVec3::new(1.0, 1.0, 1.0), 8.0, 2),
        prism(12, 7.0, 25.0),
    ];
    let rect = BuildVolume {
        exclude: vec![rect_poly(
            DVec2::new(150.0, 150.0),
            DVec2::new(190.0, 190.0),
        )],
        ..vol_rect()
    };
    fit_case("rect", &rect, &meshes, 11);
    let l_shape = BuildVolume {
        bed: BedShape::Polygon {
            points: vec![
                DVec2::new(0.0, 0.0),
                DVec2::new(200.0, 0.0),
                DVec2::new(200.0, 100.0),
                DVec2::new(100.0, 100.0),
                DVec2::new(100.0, 200.0),
                DVec2::new(0.0, 200.0),
            ],
        },
        height: 250.0,
        exclude: vec![rect_poly(DVec2::new(20.0, 20.0), DVec2::new(45.0, 45.0))],
        slicer_origin: DVec2::ZERO,
    };
    fit_case("l-polygon", &l_shape, &meshes, 12);
    let circle = BuildVolume {
        bed: BedShape::Circle { diameter: 200.0 },
        height: 250.0,
        exclude: vec![rect_poly(
            DVec2::new(120.0, 100.0),
            DVec2::new(150.0, 130.0),
        )],
        slicer_origin: DVec2::ZERO,
    };
    fit_case("circle", &circle, &meshes, 13);
}

#[test]
fn check_fit_notch_crossing_with_vertices_inside() {
    // A long thin bar whose ends are on the two arms of the L bed, crossing the notch.
    let bar = cube(DVec3::ZERO, 1.0);
    let mut m = bar;
    for p in &mut m.positions {
        *p = DVec3::new(p.x * 140.0, p.y * 4.0, p.z * 4.0);
    }
    let l = BuildVolume {
        bed: BedShape::Polygon {
            points: vec![
                DVec2::new(0.0, 0.0),
                DVec2::new(200.0, 0.0),
                DVec2::new(200.0, 100.0),
                DVec2::new(100.0, 100.0),
                DVec2::new(100.0, 200.0),
                DVec2::new(0.0, 200.0),
            ],
        },
        height: 100.0,
        exclude: Vec::new(),
        slicer_origin: DVec2::ZERO,
    };
    // Rotate 45 deg: ends at (30, 180)-ish on the vertical arm and (180, 30) on the
    // horizontal arm, so the bar passes over the notch although all vertices are inside.
    let rot = DQuat::from_rotation_z(-std::f64::consts::FRAC_PI_4);
    let p = Placement::on_plate(rot, DVec2::new(100.0, 100.0));
    let t = effective_transform(&m, &p);
    let ctx = JobCtx::detached();
    let rep = check_fit(&m, &t, &l, &ctx).unwrap();
    let vertices_inside = transformed(&m, &t)
        .iter()
        .all(|q| oracle_out(&l.bed, q.truncate()) == 0.0);
    assert!(vertices_inside, "setup: all vertices inside the bed");
    assert!(
        !rep.fits && rep.max_outside_xy > 1.0,
        "notch crossing detected: {rep:?}"
    );
}

#[test]
fn check_fit_height_and_cancel() {
    let m = cube(DVec3::ZERO, 10.0);
    let vol = BuildVolume {
        height: 8.0,
        ..vol_rect()
    };
    let t = effective_transform(
        &m,
        &Placement::on_plate(DQuat::IDENTITY, DVec2::new(100.0, 100.0)),
    );
    let rep = check_fit(&m, &t, &vol, &JobCtx::detached()).unwrap();
    assert!(!rep.fits);
    assert!((rep.height_excess - 2.0).abs() < 1e-9);
    assert_eq!(rep.max_outside_xy, 0.0);
    let tok = CancelToken::new();
    tok.cancel();
    let r = check_fit(&m, &t, &vol, &JobCtx::with_token(tok));
    assert!(matches!(r, Err(AmError::Cancelled)));
}

#[test]
fn footprint_hull_matches_reference_incl_parallel_path() {
    let mut rng = Rng(5);
    for (m, name) in [
        (sphere(DVec3::new(2.0, 1.0, 0.0), 20.0, 3), "small"),
        (sphere(DVec3::new(2.0, 1.0, 0.0), 20.0, 6), "parallel"),
        (prism(12, 9.0, 4.0), "prism"),
    ] {
        for _ in 0..5 {
            let p = Placement::on_plate(rng.quat(), DVec2::new(50.0, 60.0));
            let t = effective_transform(&m, &p);
            let xy: Vec<DVec2> = transformed(&m, &t).iter().map(|q| q.truncate()).collect();
            assert_eq!(footprint_hull(&m, &t), reference_hull(&xy), "{name}");
        }
    }
}

// --------------------------------------------------------- modification (item 4)

#[test]
fn scaling_the_mesh_keeps_item_on_plate_at_same_xy() {
    let m = sphere(DVec3::new(7.0, -3.0, 40.0), 10.0, 3);
    let mut rng = Rng(9);
    for _ in 0..20 {
        let p = Placement::on_plate(rng.quat(), DVec2::new(120.0, 80.0));
        let mut m2 = m.clone();
        let about = DVec3::new(30.0, -12.0, 55.0); // off-centre
        for q in &mut m2.positions {
            *q = about + (*q - about) * 1.1;
        }
        let bb = Aabb3::from_points(transformed(&m2, &effective_transform(&m2, &p)));
        assert!(bb.min.z.abs() < 1e-9);
        let c = bb.center();
        assert!((c.x - 120.0).abs() < 1e-9 && (c.y - 80.0).abs() < 1e-9);
        // Same rotation: the placement itself was not touched.
        assert_eq!(p.xy, DVec2::new(120.0, 80.0));
    }
}

// ---------------------------------------------------------- placement helpers

#[test]
fn rotate_about_axis_and_centre_on_plate() {
    let m = cube(DVec3::new(1.0, 2.0, 3.0), 10.0);
    let vol = vol_rect();
    let p = Placement::on_plate(DQuat::from_rotation_x(0.4), DVec2::new(30.0, 40.0));
    let q = rotate_about_build_axis(&p, 0.7);
    assert_eq!(q.xy, p.xy);
    assert_eq!(q.z, p.z);
    let back = rotate_about_build_axis(&q, -0.7);
    assert!(back.rotation.dot(p.rotation).abs() > 1.0 - 1e-14);
    let bb = Aabb3::from_points(transformed(&m, &effective_transform(&m, &q)));
    assert!(bb.min.z.abs() < 1e-9 && (bb.center().x - 30.0).abs() < 1e-9);
    let c = centre_on_plate(&q, &vol);
    assert_eq!(c.xy, DVec2::new(110.0, 110.0));
    assert_eq!(c.rotation, q.rotation);
    let circle = BuildVolume {
        bed: BedShape::Circle { diameter: 150.0 },
        ..vol_rect()
    };
    assert_eq!(centre_on_plate(&q, &circle).xy, DVec2::new(75.0, 75.0));
}

#[test]
fn pick_plane_normal_rules() {
    // Cube: every face is its own group candidate with spread 0 only if grouped per side.
    let mut cube = cube(DVec3::ZERO, 10.0);
    for f in 0..cube.face_count() {
        // Default group spans all faces: spread is large, so the triangle normal is used.
        let n = pick_plane_normal(&cube, FaceId(f as u32));
        assert!((n - cube.face_normal(FaceId(f as u32))).length() < 1e-15);
    }
    // Group the two triangles of each side together: mean == triangle normal.
    let mut by_normal: Vec<(DVec3, amg_mesh::FaceGroupId)> = Vec::new();
    for f in 0..cube.face_count() {
        let n = cube.face_normal(FaceId(f as u32));
        let g = match by_normal.iter().find(|(m, _)| (*m - n).length() < 1e-9) {
            Some((_, g)) => *g,
            None => {
                let g = cube.groups.alloc(FaceGroupInfo::new(GroupOrigin::User));
                by_normal.push((n, g));
                g
            }
        };
        cube.groups.set(FaceId(f as u32), g);
    }
    for f in 0..cube.face_count() {
        let n = pick_plane_normal(&cube, FaceId(f as u32));
        assert!((n - cube.face_normal(FaceId(f as u32))).length() < 1e-12);
    }

    // A nearly flat 6x6 grid in one group.
    let grid = |amp: f64| {
        let n = 6usize;
        let mut pos = Vec::new();
        for j in 0..=n {
            for i in 0..=n {
                pos.push(DVec3::new(i as f64, j as f64, amp * ((i + j) % 2) as f64));
            }
        }
        let mut tris = Vec::new();
        let idx = |i: usize, j: usize| (j * (n + 1) + i) as u32;
        for j in 0..n {
            for i in 0..n {
                tris.push([idx(i, j), idx(i + 1, j), idx(i + 1, j + 1)]);
                tris.push([idx(i, j), idx(i + 1, j + 1), idx(i, j + 1)]);
            }
        }
        IndexedMesh::new(pos, tris).unwrap()
    };
    let flat = grid(0.004); // normals within ~0.5 degrees
    let n = pick_plane_normal(&flat, FaceId(7));
    assert!(
        angle_between(n, DVec3::Z) < 0.5f64.to_radians(),
        "mean normal is nearly +Z"
    );
    assert!(
        (n - flat.face_normal(FaceId(7))).length() > 1e-6,
        "mean differs from the triangle's"
    );
    let rough = grid(0.05); // ~3 degrees
    let n = pick_plane_normal(&rough, FaceId(7));
    assert!((n - rough.face_normal(FaceId(7))).length() < 1e-15);
    // The same rough mesh with a fitted plane primitive uses the group mean.
    let mut fitted = rough.clone();
    let g = fitted.groups.alloc(FaceGroupInfo {
        name: None,
        origin: GroupOrigin::User,
        primitive: Some(Primitive::Plane {
            origin: DVec3::ZERO,
            normal: DVec3::Z,
        }),
    });
    for f in 0..fitted.face_count() {
        fitted.groups.set(FaceId(f as u32), g);
    }
    let n = pick_plane_normal(&fitted, FaceId(7));
    assert!(angle_between(n, DVec3::Z) < 1.5f64.to_radians());
    assert!((n - fitted.face_normal(FaceId(7))).length() > 1e-4);
    // Out-of-range pick.
    assert_eq!(pick_plane_normal(&flat, FaceId(10_000)), DVec3::Z);
}

// ----------------------------------------------------------------------- overlap

fn convex_poly(rng: &mut Rng, centre: DVec2, r: f64) -> Vec<DVec2> {
    let pts: Vec<DVec2> = (0..12)
        .map(|_| centre + DVec2::new(rng.range(-r, r), rng.range(-r, r)))
        .collect();
    reference_hull(&pts)
}

fn poly_dist(a: &[DVec2], b: &[DVec2]) -> f64 {
    let mut best = f64::INFINITY;
    for (p, q) in [(a, b), (b, a)] {
        for &x in p {
            for i in 0..q.len() {
                best = best.min(dist_seg(x, q[i], q[(i + 1) % q.len()]));
            }
        }
    }
    best
}

#[test]
fn hulls_overlap_matches_clipping_and_distance() {
    let mut rng = Rng(21);
    let (mut yes, mut no) = (0, 0);
    for k in 0..1500 {
        let a = convex_poly(&mut rng, DVec2::ZERO, 10.0);
        let off = DVec2::new(rng.range(-30.0, 30.0), rng.range(-30.0, 30.0));
        let b = convex_poly(&mut rng, off, 10.0);
        let gap = [0.0, 0.5, 2.0][k % 3];
        let area = clip_area(&a, &b);
        let d = if area > 0.0 { 0.0 } else { poly_dist(&a, &b) };
        if (d - gap).abs() < 1e-9 || (area > 0.0 && area < 1e-9) {
            continue;
        }
        let expect = area > 1e-9 || d < gap;
        assert_eq!(hulls_overlap(&a, &b, gap), expect, "case {k} gap {gap}");
        if expect {
            yes += 1;
        } else {
            no += 1;
        }
    }
    assert!(yes > 100 && no > 100, "{yes}/{no}");
}

#[test]
fn hulls_overlap_fixed_cases() {
    let sq = |x: f64, y: f64, s: f64| rect_poly(DVec2::new(x, y), DVec2::new(x + s, y + s));
    let a = sq(0.0, 0.0, 1.0);
    // Touching: not an overlap at gap 0, closer than any positive gap.
    assert!(!hulls_overlap(&a, &sq(1.0, 0.0, 1.0), 0.0));
    assert!(hulls_overlap(&a, &sq(1.0, 0.0, 1.0), 0.1));
    assert!(hulls_overlap(&a, &sq(0.5, 0.5, 1.0), 0.0));
    assert!(!hulls_overlap(&a, &sq(3.0, 0.0, 1.0), 1.0));
    assert!(hulls_overlap(&a, &sq(3.0, 0.0, 1.0), 2.5));
    // Corner to corner: SAT axis gap is 0.5 but the true distance is 0.707.
    let c = sq(1.5, 1.5, 1.0);
    assert!(!hulls_overlap(&a, &c, 0.6));
    assert!(hulls_overlap(&a, &c, 0.8));
    assert!(!hulls_overlap(&[], &a, 1.0));
}

// --------------------------------------------------------------------- geometry

#[test]
fn build_volume_geometry_rect_polygon_circle() {
    let rect = BuildVolume {
        exclude: vec![rect_poly(DVec2::new(0.0, 0.0), DVec2::new(30.0, 20.0))],
        ..vol_rect()
    };
    let g = build_volume_geometry(&rect, 10.0);
    g.plate.validate().unwrap();
    assert!((g.plate.signed_volume() - 220.0 * 220.0 * PLATE_THICKNESS).abs() < 1e-6);
    let top = g
        .plate
        .positions
        .iter()
        .map(|p| p.z)
        .fold(f64::NEG_INFINITY, f64::max);
    assert_eq!(top, 0.0);
    // 23 lines each way (0, 10, .. 220), each a full-length chord.
    assert_eq!(g.grid.len(), 46);
    assert!(g.grid.iter().all(|l| {
        l.points.len() == 2 && !l.closed && (l.points[0] - l.points[1]).length() - 220.0 < 1e-9
    }));
    assert!(g.grid.iter().all(|l| l.points.iter().all(|p| p.z == 0.0)));
    assert_eq!(g.outline.len(), 2 + 4);
    let tops: Vec<f64> = g
        .outline
        .iter()
        .filter(|l| l.closed)
        .map(|l| l.points[0].z)
        .collect();
    assert_eq!(tops, vec![0.0, 250.0]);
    assert_eq!(g.exclude.len(), 1);
    g.exclude[0].validate().unwrap();
    assert!((g.exclude[0].signed_volume() - 600.0 * EXCLUDE_THICKNESS).abs() < 1e-9);

    let l = BuildVolume {
        bed: BedShape::Polygon {
            points: vec![
                DVec2::new(0.0, 0.0),
                DVec2::new(200.0, 0.0),
                DVec2::new(200.0, 100.0),
                DVec2::new(100.0, 100.0),
                DVec2::new(100.0, 200.0),
                DVec2::new(0.0, 200.0),
            ],
        },
        ..vol_rect()
    };
    let g = build_volume_geometry(&l, 10.0);
    g.plate.validate().unwrap();
    assert!((g.plate.signed_volume() - 30_000.0 * PLATE_THICKNESS).abs() < 1e-6);
    // x = 150 only meets the lower arm; x = 50 spans the full height.
    let chord_x = |x: f64| -> Vec<f64> {
        g.grid
            .iter()
            .filter(|l| l.points[0].x == x && l.points[1].x == x)
            .map(|l| (l.points[1].y - l.points[0].y).abs())
            .collect()
    };
    assert_eq!(chord_x(150.0), vec![100.0]);
    assert_eq!(chord_x(50.0), vec![200.0]);
    assert!(g.grid.iter().all(|l| {
        l.points
            .iter()
            .all(|p| l.points.len() == 2 && !l.closed && p.z == 0.0)
    }));
    assert_eq!(g.outline.len(), 2 + 6);

    let c = BuildVolume {
        bed: BedShape::Circle { diameter: 200.0 },
        ..vol_rect()
    };
    let g = build_volume_geometry(&c, 10.0);
    g.plate.validate().unwrap();
    assert!(g.plate.signed_volume() > 0.0);
    // The centre line of the circle is a full-diameter chord, and no grid point leaves it.
    assert!(
        g.grid.iter().any(|l| l.points[0].x == 100.0
            && ((l.points[1].y - l.points[0].y).abs() - 200.0).abs() < 1e-9)
    );
    for l in &g.grid {
        for p in &l.points {
            assert!((p.truncate() - DVec2::splat(100.0)).length() <= 100.0 + 1e-9);
        }
    }
    assert_eq!(g.outline.len(), 2 + 4);
    // A zero or negative step gives no grid; the rest is still produced.
    assert!(build_volume_geometry(&c, 0.0).grid.is_empty());
    assert!(build_volume_geometry(&c, f64::NAN).grid.is_empty());
}
