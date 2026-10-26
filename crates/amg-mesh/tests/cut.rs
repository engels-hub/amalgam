//! MESH-09 tests: plane cut with CDT caps, section, separate shells, erase & fill, reverse
//! normals.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::cast_precision_loss,
    clippy::print_stderr
)]

#[path = "common/mod.rs"]
mod common;
#[path = "cut_common/mod.rs"]
mod lat;

use amg_core::{Dir3, Plane, Point2, Point3, Transform, Vec3};
use amg_jobs::{CancelToken, JobCtx};
use amg_mesh::corpus::{FileStatus, Manifest, corpus_root, read_mesh};
use amg_mesh::{
    AnalysisReport, AnalyzeOpts, FaceGroupId, FaceGroupInfo, FaceId, GroupOrigin, HoleFillOpts,
    IndexedMesh, KeepSide, MeshAttrs, MeshError, NormalWeighting, PlaneCutOpts, PlaneCutResult,
    RepairOpts, Rgba8, TriProperty, WeldOpts, analyze, erase_and_fill, plane_cut, repair,
    reverse_normals, section, separate_shells,
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

fn plane(origin: Point3, normal: Vec3) -> Plane {
    Plane::from_normal(origin, Dir3::new_unchecked(normal))
}

fn opts(plane: Plane) -> PlaneCutOpts {
    PlaneCutOpts {
        plane,
        keep: KeepSide::Both,
        cap: true,
        snap_tolerance: 0.0,
    }
}

fn cut(m: &IndexedMesh, p: Plane) -> PlaneCutResult {
    plane_cut(m, &opts(p), &ctx()).unwrap()
}

fn assert_watertight(label: &str, m: &IndexedMesh) {
    m.validate().unwrap();
    let r = report(m);
    assert!(r.boundary_loops.is_empty(), "{label}: open loops");
    assert!(
        r.non_manifold_edges.is_empty(),
        "{label}: non-manifold edges"
    );
    assert!(
        r.non_manifold_vertices.is_empty(),
        "{label}: non-manifold vertices"
    );
    assert_eq!(r.orientation_conflicts, 0, "{label}: orientation");
    assert!(r.is_watertight(), "{label}: not watertight");
    assert!(r.volume.unwrap() > 0.0, "{label}: not outward");
}

fn cap_area(m: &IndexedMesh, g: FaceGroupId) -> f64 {
    m.groups.faces_of(g).map(|f| m.face_area(f)).sum()
}

fn signed_area(points: &[Point3]) -> f64 {
    let mut a = 0.0;
    for k in 0..points.len() {
        let (p, q) = (points[k], points[(k + 1) % points.len()]);
        a += p.x * q.y - q.x * p.y;
    }
    a / 2.0
}

/// Cuts and checks: halves closed and manifold, volumes add up to 1e-9, one shared `Cap`
/// group with equal area in both halves. Returns the result and the cap area.
fn check_cut(label: &str, m: &IndexedMesh, p: Plane) -> (PlaneCutResult, f64) {
    let v = m.signed_volume();
    let r = cut(m, p);
    assert_eq!(r.open_loops, 0, "{label}");
    let mut vol = 0.0;
    let mut areas = Vec::new();
    for (name, h) in [("positive", &r.positive), ("negative", &r.negative)] {
        if let Some(h) = h {
            assert_watertight(&format!("{label} {name}"), h);
            vol += h.signed_volume();
            let g = r.cap_group.expect("cap group");
            assert_eq!(
                h.groups.info(g).unwrap().origin,
                GroupOrigin::Cap,
                "{label}"
            );
            assert!(h.groups.faces_of(g).count() > 0, "{label}: empty cap");
            areas.push(cap_area(h, g));
        }
    }
    assert!(
        ((vol - v) / v).abs() < 1e-9,
        "{label}: volumes {vol} vs {v}"
    );
    if areas.len() == 2 {
        assert!(
            (areas[0] - areas[1]).abs() <= 1e-9 * areas[0].max(1.0),
            "{label}: caps differ {areas:?}"
        );
    }
    (r, areas.first().copied().unwrap_or(0.0))
}

#[test]
fn sphere_generic_planes() {
    let m = icosphere(10.0, 4);
    for (o, n) in [
        (Point3::new(0.0, 0.0, 1.3), Vec3::Z),
        (
            Point3::new(0.4, 0.1, -2.0),
            Vec3::new(1.0, 2.0, 3.0).normalize(),
        ),
        (Point3::new(9.9, 0.0, 0.0), Vec3::X),
    ] {
        let (r, area) = check_cut("sphere", &m, plane(o, n));
        assert!(r.positive.is_some() && r.negative.is_some());
        assert!(area > 0.0);
    }
    // The cap of z = 0 is a polygon inscribed in the radius-10 circle.
    let (_, area) = check_cut("equator", &m, Plane::XY);
    assert!(
        (area - std::f64::consts::PI * 100.0).abs() < 0.01 * 314.0,
        "{area}"
    );
}

#[test]
fn plane_outside_gives_one_side() {
    let m = icosphere(10.0, 3);
    let r = cut(&m, plane(Point3::new(0.0, 0.0, 11.0), Vec3::Z));
    assert!(r.positive.is_none());
    assert_eq!(r.negative.as_ref().unwrap().face_count(), m.face_count());
    assert!(r.cap_group.is_none());
    let r = cut(&m, plane(Point3::new(0.0, 0.0, -11.0), Vec3::Z));
    assert!(r.negative.is_none());
    assert_eq!(r.positive.as_ref().unwrap().face_count(), m.face_count());
}

#[test]
fn keep_one_side_and_no_cap() {
    let m = icosphere(10.0, 3);
    let mut o = opts(Plane::XY);
    o.keep = KeepSide::Positive;
    let r = plane_cut(&m, &o, &ctx()).unwrap();
    assert!(r.positive.is_some() && r.negative.is_none());
    assert_watertight("pos", r.positive.as_ref().unwrap());
    o.keep = KeepSide::Negative;
    let r = plane_cut(&m, &o, &ctx()).unwrap();
    assert!(r.positive.is_none() && r.negative.is_some());
    o.cap = false;
    let r = plane_cut(&m, &o, &ctx()).unwrap();
    assert!(r.cap_group.is_none() && r.open_loops == 0);
    assert_eq!(report(&r.negative.unwrap()).boundary_loops.len(), 1);
}

#[test]
fn lattice_box_through_vertices_and_edges() {
    let n = 6u32;
    let m = lat::lattice_box(n);
    assert_eq!(m.face_count(), 12 * (n * n) as usize);
    let nf = f64::from(n);
    let d2 = Vec3::new(1.0, 1.0, 0.0).normalize();
    let d3 = Vec3::new(1.0, 1.0, 1.0).normalize();
    let planes: Vec<(&str, Plane)> = vec![
        // Along lattice lines: through vertices, cell edges and cell diagonals.
        ("x=2", plane(Point3::new(2.0, 0.0, 0.0), Vec3::X)),
        ("y=3", plane(Point3::new(0.0, 3.0, 0.0), Vec3::Y)),
        ("z=1", plane(Point3::new(0.0, 0.0, 1.0), Vec3::Z)),
        ("-x=-4", plane(Point3::new(4.0, 0.0, 0.0), -Vec3::X)),
        ("x+y=5", plane(Point3::new(5.0, 0.0, 0.0), d2)),
        ("x+y=6", plane(Point3::new(6.0, 0.0, 0.0), d2)),
        (
            "x-y=1",
            plane(
                Point3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, -1.0, 0.0).normalize(),
            ),
        ),
        ("x+y+z=7", plane(Point3::new(7.0, 0.0, 0.0), d3)),
        // Generic.
        ("x=2.5", plane(Point3::new(2.5, 0.0, 0.0), Vec3::X)),
        // Corner cuts: close to a corner of the box.
        ("x+y+z=1", plane(Point3::new(1.0, 0.0, 0.0), d3)),
        ("x+y+z=2", plane(Point3::new(2.0, 0.0, 0.0), d3)),
        (
            "x+y+z=3n-1",
            plane(Point3::new(3.0 * nf - 1.0, 0.0, 0.0), d3),
        ),
    ];
    for (label, p) in planes {
        let (r, area) = check_cut(label, &m, p);
        eprintln!("{label}: section area {area}");
        assert!(r.positive.is_some() && r.negative.is_some(), "{label}");
    }
    // Axis-aligned section areas are exact.
    let (_, a) = check_cut("x=2", &m, plane(Point3::new(2.0, 0.0, 0.0), Vec3::X));
    assert!((a - nf * nf).abs() < 1e-9, "{a}");
    // A diagonal plane through vertex lines: section is a rectangle n × n·sqrt(2).
    let (_, a) = check_cut("x+y=6", &m, plane(Point3::new(6.0, 0.0, 0.0), d2));
    assert!((a - nf * nf * 2f64.sqrt()).abs() < 1e-9, "{a}");
}

#[test]
fn coplanar_faces_and_touching_planes() {
    let n = 4u32;
    let m = lat::lattice_box(n);
    let v = m.signed_volume();
    let nf = f64::from(n);
    // The plane contains a whole side: the box is entirely on one side.
    for (o, normal, positive_side) in [
        (Point3::ZERO, Vec3::X, true),
        (Point3::new(nf, 0.0, 0.0), Vec3::X, false),
        (Point3::ZERO, -Vec3::X, false),
        (Point3::new(0.0, 0.0, nf), Vec3::Z, false),
    ] {
        let r = cut(&m, plane(o, normal));
        let (whole, none) = if positive_side {
            (r.positive.as_ref(), r.negative.as_ref())
        } else {
            (r.negative.as_ref(), r.positive.as_ref())
        };
        assert!(none.is_none(), "{o:?} {normal:?}");
        let w = whole.unwrap();
        assert_watertight("coplanar", w);
        assert!((w.signed_volume() - v).abs() < 1e-9);
        assert!(r.cap_group.is_none());
    }
    // Touching along an edge and at corners: one side is empty.
    let s = 1.0 / 2f64.sqrt();
    for (o, normal) in [
        (Point3::ZERO, Vec3::new(-s, -s, 0.0)),
        (Point3::ZERO, -Vec3::new(1.0, 1.0, 1.0).normalize()),
        (
            Point3::new(nf, nf, nf),
            Vec3::new(1.0, 1.0, 1.0).normalize(),
        ),
    ] {
        let r = cut(&m, plane(o, normal));
        let sides = [&r.positive, &r.negative];
        assert_eq!(
            sides.iter().filter(|s| s.is_some()).count(),
            1,
            "{o:?} {normal:?}"
        );
        let w = sides.iter().find_map(|s| s.as_ref()).unwrap();
        assert_watertight("touch", w);
        assert!((w.signed_volume() - v).abs() < 1e-9);
    }
}

#[test]
fn snap_tolerance_treats_near_vertices_as_on_the_plane() {
    let m = lat::lattice_box(4);
    let v = m.signed_volume();
    // 1e-7 above the lattice plane x = 2: exact classification splits cells, snapping does not.
    let p = plane(Point3::new(2.0 + 1e-7, 0.0, 0.0), Vec3::X);
    let exact = cut(&m, p);
    let mut o = opts(p);
    o.snap_tolerance = 1e-6;
    let snapped = plane_cut(&m, &o, &ctx()).unwrap();
    assert!(
        exact.positive.as_ref().unwrap().face_count()
            > snapped.positive.as_ref().unwrap().face_count()
    );
    for h in [&snapped.positive, &snapped.negative] {
        assert_watertight("snapped", h.as_ref().unwrap());
    }
    let vol = snapped.positive.unwrap().signed_volume() + snapped.negative.unwrap().signed_volume();
    assert!(((vol - v) / v).abs() < 1e-9);
    let mut bad = opts(p);
    bad.snap_tolerance = -1.0;
    assert!(matches!(
        plane_cut(&m, &bad, &ctx()),
        Err(MeshError::InvalidOption { .. })
    ));
}

#[test]
fn torus_annulus_cap_has_a_hole() {
    let m = common::torus(48, 24, 10.0, 3.0);
    // z = 0 separates the top from the bottom half; the section is an annulus.
    let (r, area) = check_cut("torus z=0", &m, Plane::XY);
    let expected = std::f64::consts::PI * (13.0f64.powi(2) - 7.0f64.powi(2));
    assert!(
        (area - expected).abs() < 0.05 * expected,
        "{area} vs {expected}"
    );
    assert_eq!(report(&r.positive.unwrap()).shells.len(), 1);
    // x = 0 cuts the ring in two places: two discs, one C-shaped shell per side.
    let (r, _) = check_cut("torus x=0", &m, Plane::YZ);
    assert_eq!(report(r.positive.as_ref().unwrap()).shells.len(), 1);
    let sec = section(&m, &Plane::YZ, 0.0);
    assert_eq!(sec.len(), 2);
    assert!(sec.iter().all(|c| c.closed));
}

fn hollow_sphere() -> IndexedMesh {
    let mut m = icosphere(10.0, 3);
    let mut inner = icosphere(5.0, 2);
    for t in &mut inner.tris {
        t.swap(1, 2);
    }
    m.append(&inner).unwrap();
    m
}

#[test]
fn hollow_body_cap_is_an_annulus() {
    let m = hollow_sphere();
    assert!(m.signed_volume() > 0.0);
    let (r, area) = check_cut("hollow", &m, plane(Point3::new(0.0, 0.0, 0.7), Vec3::Z));
    let r_out = (100.0f64 - 0.49).sqrt();
    let r_in = (25.0f64 - 0.49).sqrt();
    let expected = std::f64::consts::PI * (r_out * r_out - r_in * r_in);
    assert!(
        (area - expected).abs() < 0.08 * expected,
        "{area} vs {expected}"
    );
    // Outer wall, inner wall and cap are one connected shell.
    assert_eq!(report(&r.positive.unwrap()).shells.len(), 1);
    // A plane that misses the cavity gives a full disc.
    let (_, disc) = check_cut(
        "hollow high",
        &m,
        plane(Point3::new(0.0, 0.0, 8.0), Vec3::Z),
    );
    assert!(
        (disc - std::f64::consts::PI * 36.0).abs() < 0.1 * 113.0,
        "{disc}"
    );
}

#[test]
fn open_input_leaves_open_loops() {
    let m = icosphere(10.0, 3);
    let keep: Vec<[u32; 3]> = (0..m.face_count() as u32)
        .filter(|&f| m.face_normal(FaceId(f)).z > -0.6)
        .map(|f| m.tris[f as usize])
        .collect();
    let mut bowl = IndexedMesh::new(m.positions.clone(), keep).unwrap();
    bowl.compact();
    // A horizontal cut above the rim is a closed ring: capped.
    let r = cut(&bowl, plane(Point3::new(0.0, 0.0, 2.0), Vec3::Z));
    assert_eq!(r.open_loops, 0);
    // A vertical cut passes through the open rim: the section chain does not close.
    let r = cut(&bowl, Plane::YZ);
    assert!(r.open_loops > 0, "{}", r.open_loops);
    assert!(r.cap_group.is_none());
    let sec = section(&bowl, &Plane::YZ, 0.0);
    assert!(sec.iter().any(|c| !c.closed));
}

#[test]
fn section_orientation_and_shape() {
    let m = icosphere(10.0, 4);
    let s = section(&m, &plane(Point3::new(0.0, 0.0, 3.0), Vec3::Z), 0.0);
    assert_eq!(s.len(), 1);
    assert!(s[0].closed);
    assert!(s[0].points.iter().all(|p| (p.z - 3.0).abs() < 1e-9));
    // Counter-clockwise seen from +Z: material on the left.
    let area = signed_area(&s[0].points);
    assert!(area > 0.0);
    assert!((area - std::f64::consts::PI * 91.0).abs() < 0.01 * 286.0);
    // Seen from the other side the same loop runs the other way round.
    let flipped = section(&m, &plane(Point3::new(0.0, 0.0, 3.0), -Vec3::Z), 0.0);
    assert!(signed_area(&flipped[0].points) < 0.0);
    assert!(section(&m, &plane(Point3::new(0.0, 0.0, 30.0), Vec3::Z), 0.0).is_empty());
    assert!(section(&IndexedMesh::default(), &Plane::XY, 0.0).is_empty());
}

#[test]
fn section_loops_touching_at_a_vertex_stay_apart() {
    // Two unit cubes sharing one vertical edge (non-manifold edge, welded): the section at
    // z = 0.5 is two squares that meet in the shared edge's cut vertex.
    let mut a = lat::lattice_box(1);
    let mut b = lat::lattice_box(1);
    b.transform(&Transform::from_translation(Vec3::new(1.0, 1.0, 0.0)));
    a.append(&b).unwrap();
    a.weld(WeldOpts { tolerance: 1e-9 }).unwrap();
    a.compact();
    let p = plane(Point3::new(0.0, 0.0, 0.5), Vec3::Z);
    let s = section(&a, &p, 0.0);
    assert_eq!(s.len(), 2, "{s:?}");
    for c in &s {
        assert!(c.closed);
        assert!(c.points.len() >= 4);
        assert!((signed_area(&c.points) - 1.0).abs() < 1e-12);
    }
    let r = cut(&a, p);
    assert_eq!(r.open_loops, 0);
    let g = r.cap_group.unwrap();
    for h in [r.positive.as_ref().unwrap(), r.negative.as_ref().unwrap()] {
        assert!((cap_area(h, g) - 2.0).abs() < 1e-12);
        assert!(report(h).boundary_loops.is_empty());
    }
    let vol = r.positive.unwrap().signed_volume() + r.negative.unwrap().signed_volume();
    assert!((vol - a.signed_volume()).abs() < 1e-12);
}

#[test]
fn two_shells_cut_into_two_shells() {
    let m = three_shells();
    let (r, _) = check_cut("shells", &m, plane(Point3::new(0.0, 0.0, 0.1), Vec3::Z));
    assert_eq!(report(r.positive.as_ref().unwrap()).shells.len(), 3);
}

#[test]
fn attributes_are_interpolated() {
    let mut m = lat::lattice_box(2);
    // Colour and UV are linear in the position, so interpolation must reproduce them.
    let colour = |p: Point3| {
        Rgba8([
            (p.x * 100.0) as u8,
            (p.y * 100.0) as u8,
            (p.z * 100.0) as u8,
            255,
        ])
    };
    m.attrs = MeshAttrs {
        vertex_normals: Some(
            m.positions
                .iter()
                .map(|p| (*p - Point3::ONE).normalize_or_zero())
                .collect(),
        ),
        vertex_colors: Some(m.positions.iter().map(|p| colour(*p)).collect()),
        corner_uvs: Some(
            m.tris
                .iter()
                .map(|t| {
                    t.map(|v| {
                        let p = m.positions[v as usize];
                        Point2::new(p.x, p.y)
                    })
                })
                .collect(),
        ),
        tri_properties: Some(vec![
            TriProperty {
                group: 7,
                index: [1, 2, 3]
            };
            m.tris.len()
        ]),
    };
    m.validate().unwrap();
    // x = 0.5 cuts lattice edges at their midpoints.
    let r = cut(&m, plane(Point3::new(0.5, 0.0, 0.0), Vec3::X));
    let cap = r.cap_group.unwrap();
    for h in [r.positive.as_ref().unwrap(), r.negative.as_ref().unwrap()] {
        h.validate().unwrap();
        let a = &h.attrs;
        for (p, c) in h.positions.iter().zip(a.vertex_colors.as_ref().unwrap()) {
            let want = colour(*p);
            for k in 0..3 {
                assert!(
                    i32::from(c.0[k]).abs_diff(i32::from(want.0[k])) <= 1,
                    "{p:?} {c:?} {want:?}"
                );
            }
        }
        for (t, uv) in h.tris.iter().zip(a.corner_uvs.as_ref().unwrap()) {
            for (k, c) in uv.iter().enumerate() {
                let p = h.positions[t[k] as usize];
                // Cap faces carry zero UVs; everything else follows the position.
                if *c != Point2::ZERO {
                    assert!((c.x - p.x).abs() < 1e-12 && (c.y - p.y).abs() < 1e-12);
                }
            }
        }
        for f in h.groups.faces_of(cap) {
            assert_eq!(
                a.tri_properties.as_ref().unwrap()[f.0 as usize],
                TriProperty::NONE
            );
            assert_eq!(
                a.corner_uvs.as_ref().unwrap()[f.0 as usize],
                [Point2::ZERO; 3]
            );
        }
        assert!(
            a.tri_properties
                .as_ref()
                .unwrap()
                .iter()
                .all(|p| p.group == 7 || *p == TriProperty::NONE)
        );
        for n in a.vertex_normals.as_ref().unwrap() {
            assert!((n.length() - 1.0).abs() < 1e-9 || *n == Vec3::ZERO);
        }
    }
}

#[test]
fn original_groups_survive_and_cap_id_is_new() {
    let mut m = icosphere(10.0, 3);
    let g1 = m
        .groups
        .alloc(FaceGroupInfo::new(GroupOrigin::Segmentation));
    for f in 0..50u32 {
        m.groups.set(FaceId(f), g1);
    }
    let before = m.groups.next_id();
    let r = cut(&m, plane(Point3::new(0.0, 0.0, 1.0), Vec3::X));
    assert_eq!(r.cap_group, Some(FaceGroupId(before)));
    let (p, n) = (r.positive.unwrap(), r.negative.unwrap());
    for h in [&p, &n] {
        assert!(h.groups.info(g1).is_some());
    }
    assert!(p.groups.faces_of(g1).count() + n.groups.faces_of(g1).count() >= 50);
}

#[test]
fn deterministic_across_thread_counts() {
    let m = icosphere(10.0, 5);
    let p = plane(
        Point3::new(0.3, -0.2, 1.1),
        Vec3::new(1.0, 2.0, 0.5).normalize(),
    );
    let hash = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap();
        pool.install(|| {
            let r = cut(&m, p);
            (
                r.positive.unwrap().content_hash(),
                r.negative.unwrap().content_hash(),
            )
        })
    };
    let h1 = hash(1);
    assert_eq!(h1, hash(4));
    assert_eq!(h1, hash(13));
}

#[test]
fn cancel_and_empty() {
    let m = icosphere(10.0, 3);
    let t = CancelToken::new();
    t.cancel();
    let c = JobCtx::with_token(t);
    assert!(matches!(
        plane_cut(&m, &opts(Plane::XY), &c),
        Err(MeshError::Cancelled)
    ));
    assert!(matches!(
        plane_cut(&IndexedMesh::default(), &opts(Plane::XY), &ctx()),
        Err(MeshError::Empty)
    ));
}

// ------------------------------------------------------------------------------- shells

fn three_shells() -> IndexedMesh {
    let mut a = icosphere(10.0, 3);
    let mut b = icosphere(4.0, 2);
    b.transform(&Transform::from_translation(Vec3::new(30.0, 0.0, 0.0)));
    let mut c = icosphere(1.0, 1);
    c.transform(&Transform::from_translation(Vec3::new(0.0, 30.0, 0.0)));
    a.append(&b).unwrap();
    a.append(&c).unwrap();
    a
}

#[test]
fn separate_shells_matches_analysis() {
    let m = three_shells();
    let an = report(&m);
    let shells = separate_shells(&m);
    assert_eq!(shells.len(), an.shells.len());
    assert_eq!(shells.len(), 3);
    let areas: Vec<f64> = shells.iter().map(IndexedMesh::area).collect();
    assert!(areas.windows(2).all(|w| w[0] >= w[1]), "{areas:?}");
    assert_eq!(
        shells.iter().map(IndexedMesh::face_count).sum::<usize>(),
        m.face_count()
    );
    let vol: f64 = shells.iter().map(IndexedMesh::signed_volume).sum();
    assert!((vol - m.signed_volume()).abs() < 1e-9 * vol);
    for s in &shells {
        assert_watertight("shell", s);
        assert_eq!(report(s).shells.len(), 1);
    }
    // One shell: the same mesh.
    let one = separate_shells(&icosphere(10.0, 2));
    assert_eq!(one.len(), 1);
    assert_eq!(one[0].content_hash(), icosphere(10.0, 2).content_hash());
    assert!(separate_shells(&IndexedMesh::default()).is_empty());
}

#[test]
fn separate_shells_after_a_cut() {
    let m = common::torus(40, 20, 10.0, 3.0);
    let r = cut(&m, Plane::YZ);
    for h in [r.positive.unwrap(), r.negative.unwrap()] {
        assert_eq!(separate_shells(&h).len(), report(&h).shells.len());
    }
    let m = three_shells();
    let r = cut(&m, plane(Point3::new(0.0, 0.0, 0.5), Vec3::Z));
    let h = r.positive.unwrap();
    assert_eq!(separate_shells(&h).len(), report(&h).shells.len());
}

#[test]
fn separate_shells_keeps_attributes_and_groups() {
    let mut m = three_shells();
    m.attrs.vertex_colors = Some(
        (0..m.positions.len())
            .map(|i| Rgba8([i as u8, 0, 0, 255]))
            .collect(),
    );
    m.attrs.corner_uvs = Some(vec![[Point2::ONE; 3]; m.tris.len()]);
    let g = m
        .groups
        .alloc(FaceGroupInfo::new(GroupOrigin::Segmentation));
    let last = m.face_count() as u32 - 1;
    m.groups.set(FaceId(last), g);
    let shells = separate_shells(&m);
    for s in &shells {
        s.validate().unwrap();
        assert_eq!(
            s.attrs.vertex_colors.as_ref().unwrap().len(),
            s.positions.len()
        );
    }
    assert_eq!(
        shells
            .iter()
            .map(|s| s.groups.faces_of(g).count())
            .sum::<usize>(),
        1
    );
}

// --------------------------------------------------------------------------- erase & fill

#[test]
fn erase_and_fill_sphere_patch() {
    let m = icosphere(10.0, 3);
    let v = m.signed_volume();
    let faces: Vec<FaceId> = (0..m.face_count() as u32)
        .filter(|&f| m.face_normal(FaceId(f)).z > 0.9)
        .map(FaceId)
        .collect();
    assert!(faces.len() > 5);
    let out = erase_and_fill(&m, &faces, &HoleFillOpts::default(), &ctx()).unwrap();
    assert_watertight("erase&fill", &out);
    let fill = out
        .groups
        .ids()
        .find(|g| out.groups.info(*g).unwrap().origin == GroupOrigin::Fill)
        .expect("fill group");
    assert!(out.groups.faces_of(fill).count() > 0);
    assert!((out.signed_volume() - v).abs() < 0.1 * v);
    // Erasing nothing returns the input; bad ids are rejected.
    assert_eq!(
        erase_and_fill(&m, &[], &HoleFillOpts::default(), &ctx()).unwrap(),
        m
    );
    assert!(matches!(
        erase_and_fill(&m, &[FaceId(u32::MAX)], &HoleFillOpts::default(), &ctx()),
        Err(MeshError::InvalidOption { .. })
    ));
    // Loops over the limit are an error, not a silently open result.
    let small = HoleFillOpts {
        max_boundary_edges: 3,
        ..HoleFillOpts::default()
    };
    assert!(matches!(
        erase_and_fill(&m, &faces, &small, &ctx()),
        Err(MeshError::NotClosed { .. })
    ));
}

#[test]
fn erase_and_fill_single_face_and_vertex_fan() {
    let m = icosphere(10.0, 2);
    let out = erase_and_fill(&m, &[FaceId(7)], &HoleFillOpts::default(), &ctx()).unwrap();
    assert_watertight("one face", &out);
    assert_eq!(out.face_count(), m.face_count());
    // All faces around a vertex: the vertex disappears.
    let around: Vec<FaceId> = (0..m.face_count() as u32)
        .filter(|&f| m.tris[f as usize].contains(&0))
        .map(FaceId)
        .collect();
    let plain = HoleFillOpts {
        refine: false,
        ..HoleFillOpts::default()
    };
    let out = erase_and_fill(&m, &around, &plain, &ctx()).unwrap();
    assert_watertight("vertex fan", &out);
    assert_eq!(out.vertex_count(), m.vertex_count() - 1);
}

#[test]
fn erase_and_fill_leaves_old_holes_alone() {
    let m = icosphere(10.0, 3);
    let kept: Vec<[u32; 3]> = (0..m.face_count() as u32)
        .filter(|&f| m.face_normal(FaceId(f)).z >= -0.95)
        .map(|f| m.tris[f as usize])
        .collect();
    let mut open = IndexedMesh::new(m.positions.clone(), kept).unwrap();
    open.compact();
    assert_eq!(report(&open).boundary_loops.len(), 1);
    let top: Vec<FaceId> = (0..open.face_count() as u32)
        .filter(|&f| open.face_normal(FaceId(f)).z > 0.95)
        .map(FaceId)
        .collect();
    let out = erase_and_fill(&open, &top, &HoleFillOpts::default(), &ctx()).unwrap();
    assert_eq!(report(&out).boundary_loops.len(), 1, "the old hole stays");
}

// ---------------------------------------------------------------------- reverse normals

#[test]
fn reverse_normals_negates_volume() {
    let mut m = icosphere(10.0, 3);
    m.attrs.vertex_normals = Some(m.vertex_normals(NormalWeighting::Angle));
    m.attrs.corner_uvs = Some(
        (0..m.tris.len())
            .map(|i| {
                [
                    Point2::new(i as f64, 0.0),
                    Point2::new(0.0, 1.0),
                    Point2::new(2.0, 2.0),
                ]
            })
            .collect(),
    );
    m.attrs.tri_properties = Some(vec![
        TriProperty {
            group: 1,
            index: [1, 2, 3]
        };
        m.tris.len()
    ]);
    let v = m.signed_volume();
    let before = m.clone();
    reverse_normals(&mut m, None);
    m.validate().unwrap();
    assert!((m.signed_volume() + v).abs() < 1e-9 * v);
    assert_eq!(
        m.attrs.corner_uvs.as_ref().unwrap()[3][1],
        before.attrs.corner_uvs.as_ref().unwrap()[3][2]
    );
    assert_eq!(m.attrs.tri_properties.as_ref().unwrap()[0].index, [1, 3, 2]);
    let (n0, n1) = (
        m.attrs.vertex_normals.as_ref().unwrap()[5],
        before.attrs.vertex_normals.as_ref().unwrap()[5],
    );
    assert!(n0.dot(n1) < -0.99);
    reverse_normals(&mut m, None);
    assert_eq!(m, before);
    // Partial: only the listed faces flip; duplicates and bad ids are harmless.
    reverse_normals(&mut m, Some(&[FaceId(0), FaceId(0), FaceId(u32::MAX)]));
    assert_eq!(
        m.tris[0],
        [before.tris[0][0], before.tris[0][2], before.tris[0][1]]
    );
    assert_eq!(&m.tris[1..], &before.tris[1..]);
}

// ----------------------------------------------------------------------- golden / scale

#[test]
fn golden_cut_sphere() {
    let m = icosphere(10.0, 4);
    let r = cut(
        &m,
        plane(
            Point3::new(0.4, 0.1, 2.0),
            Vec3::new(1.0, 2.0, 3.0).normalize(),
        ),
    );
    amg_testkit::assert_golden!(
        "mesh",
        "cut_sphere_r10_pos",
        soup(r.positive.as_ref().unwrap())
    );
    amg_testkit::assert_golden!(
        "mesh",
        "cut_sphere_r10_neg",
        soup(r.negative.as_ref().unwrap())
    );
}

#[test]
fn golden_cut_lattice_box_through_vertices() {
    let m = lat::lattice_box(6);
    let r = cut(
        &m,
        plane(
            Point3::new(5.0, 0.0, 0.0),
            Vec3::new(1.0, 1.0, 0.0).normalize(),
        ),
    );
    amg_testkit::assert_golden!(
        "mesh",
        "cut_box6_diagonal_pos",
        soup(r.positive.as_ref().unwrap())
    );
    amg_testkit::assert_golden!(
        "mesh",
        "cut_box6_diagonal_neg",
        soup(r.negative.as_ref().unwrap())
    );
}

#[test]
fn golden_cut_torus_annulus() {
    let m = common::torus(48, 24, 10.0, 3.0);
    let r = cut(&m, Plane::XY);
    amg_testkit::assert_golden!(
        "mesh",
        "cut_torus_z0_pos",
        soup(r.positive.as_ref().unwrap())
    );
}

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}

/// Done-when: a closed 1 M-triangle mesh gives closed manifold halves whose volumes add up to
/// within 1e-9, also for planes through vertices and edges; under 300 ms (asserted with
/// `AMG_STRICT_TIMING=1`, measured in `benches/mesh_cut.rs`).
#[test]
fn million_triangles() {
    let m = lat::lattice_box(289);
    assert!(m.face_count() >= 1_000_000);
    let v = m.signed_volume();
    let s = Vec3::new(1.0, 1.0, 0.0).normalize();
    let planes = [
        plane(Point3::new(100.0, 0.0, 0.0), Vec3::X),
        plane(Point3::new(200.0, 0.0, 0.0), s),
        plane(Point3::new(100.5, 0.0, 0.0), Vec3::X),
    ];
    for p in planes {
        let started = std::time::Instant::now();
        let r = cut(&m, p);
        let ms = started.elapsed().as_secs_f64() * 1e3;
        eprintln!("1M cut: {ms:.0} ms");
        let (pos, neg) = (r.positive.unwrap(), r.negative.unwrap());
        let vol = pos.signed_volume() + neg.signed_volume();
        assert!(((vol - v) / v).abs() < 1e-9);
        for h in [&pos, &neg] {
            assert!(report(h).is_watertight());
        }
        if strict_timing() {
            assert!(ms < 300.0, "cut took {ms:.0} ms");
        }
    }
}

// ------------------------------------------------------------------------------- corpus

fn require_testdata() -> bool {
    std::env::var("AMG_REQUIRE_TESTDATA").is_ok_and(|v| v == "1")
}

/// Cuts repaired, self-intersection-free corpus models through their bounding-box centre and
/// through a vertex: halves stay closed and the volumes add up.
#[test]
fn corpus_models_cut_cleanly() {
    let manifest = Manifest::load().unwrap();
    let root = corpus_root();
    let (mut tested, mut capped) = (0usize, 0usize);
    for e in manifest.meshes.iter() {
        if !e.expect_repairable || e.faces > 20_000 {
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
        let Ok(raw) = read_mesh(&path) else { continue };
        let (m, _) = repair(&raw, &RepairOpts::default(), &ctx()).unwrap();
        let o = AnalyzeOpts {
            self_intersections: true,
            ..AnalyzeOpts::default()
        };
        let rep = analyze(&m, &o, &ctx()).unwrap();
        if !rep.is_watertight()
            || rep.is_inverted
            || rep.shells.len() != 1
            || rep
                .self_intersecting_pairs
                .as_ref()
                .is_none_or(|p| !p.is_empty())
        {
            continue;
        }
        tested += 1;
        let v = m.signed_volume();
        let centre = m.aabb().center();
        for origin in [centre, m.positions[m.positions.len() / 2]] {
            let r = cut(&m, plane(origin, Vec3::new(0.3, 0.5, 0.8).normalize()));
            let mut vol = 0.0;
            for h in [&r.positive, &r.negative].into_iter().flatten() {
                vol += h.signed_volume();
                if r.open_loops == 0 {
                    assert!(report(h).is_watertight(), "{}", e.id);
                }
            }
            if r.open_loops == 0 {
                capped += 1;
                assert!(((vol - v) / v).abs() < 1e-9, "{}: {vol} vs {v}", e.id);
            } else {
                eprintln!("{}: {} loops left open", e.id, r.open_loops);
            }
        }
    }
    eprintln!("corpus cut: {tested} models, {capped} fully capped cuts");
    if require_testdata() {
        assert!(tested >= 20, "only {tested} corpus models usable");
    }
}
