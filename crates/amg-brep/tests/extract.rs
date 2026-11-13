//! BREP-17: plane sections and edge projection (S02 §3) on the `testdata/brep` fixtures.
//!
//! Done-when 1 (cylinder sections: one exact circle, an ellipse with analytic axes), 2 (filleted
//! box top projected onto XY: lines and circle arcs with the right radii and end points), 3
//! (sections of the B-spline patch within 1e-6 mm of the true intersection), plus chaining,
//! ordered face loops (S12 IO-08), errors and a golden of a section turned back into a face.
#![cfg(feature = "occt")]
#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    reason = "test code; exact radii are compared bit for bit"
)]

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use amg_brep::geom::{Curve3, CurveSeg, FaceBounds, Surface, Tag};
use amg_brep::{
    BrepKernel, Dir3, EdgeId, JobCtx, KernelConfig, KernelError, OcctKernel, Plane, Point3, Shape,
    SubKind, TessQuality, Vec3,
};
use amg_testkit::TriSoup;

fn load(name: &str) -> Option<Shape> {
    let path = amg_testkit::testdata(&format!("brep/{name}.brep"))?;
    let file = std::fs::File::open(path).expect("open fixture");
    Some(Shape::read_text(std::io::BufReader::new(file)).expect("read fixture"))
}

fn kernel() -> OcctKernel {
    OcctKernel::new(KernelConfig::default())
}

fn plane(origin: Point3, normal: Vec3) -> Plane {
    Plane::from_normal(origin, Dir3::new_unchecked(normal.normalize()))
}

fn close(a: Point3, b: Point3, tol: f64) -> bool {
    (a - b).length() <= tol
}

/// Each chain of `segs` (as returned) runs head to tail; returns the chains' segment counts and
/// whether each is closed.
fn chains(segs: &[CurveSeg], tol: f64) -> Vec<(usize, bool)> {
    let mut out = Vec::new();
    let mut first = 0;
    for i in 0..segs.len() {
        let last = i + 1 == segs.len();
        let joins = !last && close(segs[i].end().unwrap(), segs[i + 1].start().unwrap(), tol);
        if !joins {
            let closed = close(segs[i].end().unwrap(), segs[first].start().unwrap(), tol);
            out.push((i + 1 - first, closed));
            first = i + 1;
        }
    }
    out
}

// ---- Done-when 1 -------------------------------------------------------------------------

#[test]
fn a_perpendicular_cylinder_section_is_one_exact_circle() {
    let Some(cyl) = load("cylinder") else { return };
    for z in [0.5, 7.0, 19.5] {
        let segs = kernel()
            .plane_section(&cyl, &plane(Point3::new(0.0, 0.0, z), Vec3::Z))
            .unwrap();
        assert_eq!(segs.len(), 1, "z = {z}: {segs:?}");
        let seg = &segs[0];
        let Curve3::Circle { frame, radius } = &seg.curve else {
            panic!("z = {z}: {seg:?}")
        };
        assert_eq!(*radius, 5.0, "exact radius");
        assert!(
            close(frame.origin, Point3::new(3.0, 1.0, z), 1e-12),
            "{frame:?}"
        );
        assert!(frame.normal.get().cross(Vec3::Z).length() < 1e-12);
        assert!(((seg.t1 - seg.t0).abs() - TAU).abs() < 1e-12, "{seg:?}");
        // The lateral face (the only face it crosses).
        let face = u32::try_from(seg.tag.0).unwrap();
        assert!(matches!(
            cyl.surface(amg_brep::FaceId(face)),
            Surface::Cylinder { .. }
        ));
    }
}

#[test]
fn an_oblique_cylinder_section_is_an_ellipse_with_analytic_axes() {
    let Some(cyl) = load("cylinder") else { return };
    let r = 5.0;
    let center = Point3::new(3.0, 1.0, 10.0);
    for theta in [0.2_f64, 0.5, 0.9] {
        let n = Vec3::new(0.0, theta.sin(), theta.cos());
        let segs = kernel().plane_section(&cyl, &plane(center, n)).unwrap();
        assert_eq!(segs.len(), 1, "θ = {theta}: {segs:?}");
        let Curve3::Ellipse {
            frame,
            major_radius,
            minor_radius,
        } = &segs[0].curve
        else {
            panic!("θ = {theta}: {:?}", segs[0])
        };
        let major_err = (major_radius - r / theta.cos()).abs();
        let minor_err = (minor_radius - r).abs();
        assert!(major_err < 1e-9, "major {major_radius} (err {major_err:e})");
        assert!(minor_err < 1e-9, "minor {minor_radius} (err {minor_err:e})");
        assert!(close(frame.origin, center, 1e-9), "{frame:?}");
        // Major axis in the plane, across the tilt; minor axis along x.
        let major_dir = Vec3::new(0.0, theta.cos(), -theta.sin());
        assert!(frame.x.get().cross(major_dir).length() < 1e-9, "{frame:?}");
        assert!(frame.normal.get().cross(n).length() < 1e-9, "{frame:?}");
        assert!(((segs[0].t1 - segs[0].t0).abs() - TAU).abs() < 1e-9);
        println!("θ = {theta}: major err {major_err:e}, minor err {minor_err:e}");
    }
}

#[test]
fn sections_through_caps_close_into_one_loop() {
    let Some(cyl) = load("cylinder") else { return };
    // A plane containing the axis: two lines on the lateral face, two on the caps.
    let segs = kernel()
        .plane_section(&cyl, &plane(Point3::new(3.0, 1.0, 0.0), Vec3::X))
        .unwrap();
    assert_eq!(segs.len(), 4, "{segs:?}");
    assert!(segs.iter().all(|s| matches!(s.curve, Curve3::Line { .. })));
    assert_eq!(chains(&segs, 1e-6), [(4, true)]);
    let length: f64 = segs.iter().map(|s| (s.t1 - s.t0).abs()).sum();
    assert!((length - 60.0).abs() < 1e-9, "{length}");
}

#[test]
fn spheres_and_tori_give_circles() {
    let Some(sphere) = load("sphere") else { return };
    let segs = kernel()
        .plane_section(&sphere, &plane(Point3::new(0.0, 0.0, 3.0), Vec3::Z))
        .unwrap();
    assert_eq!(segs.len(), 1, "{segs:?}");
    let Curve3::Circle { radius, .. } = segs[0].curve else {
        panic!("{segs:?}")
    };
    assert!((radius - (49.0f64 - 9.0).sqrt()).abs() < 1e-9, "{radius}");

    let Some(torus) = load("torus") else { return };
    let segs = kernel()
        .plane_section(&torus, &plane(Point3::ZERO, Vec3::Z))
        .unwrap();
    let mut radii: Vec<f64> = segs
        .iter()
        .map(|s| match s.curve {
            Curve3::Circle { radius, .. } => radius,
            ref other => panic!("{other:?}"),
        })
        .collect();
    radii.sort_by(f64::total_cmp);
    assert_eq!(radii.len(), 2, "{segs:?}");
    assert!((radii[0] - 15.0).abs() < 1e-9 && (radii[1] - 25.0).abs() < 1e-9);
    assert_eq!(chains(&segs, 1e-6), [(1, true), (1, true)]);
}

#[test]
fn a_plane_that_misses_gives_nothing() {
    let Some(cyl) = load("cylinder") else { return };
    let segs = kernel()
        .plane_section(&cyl, &plane(Point3::new(0.0, 0.0, 50.0), Vec3::Z))
        .unwrap();
    assert!(segs.is_empty(), "{segs:?}");
}

// ---- Done-when 2 -------------------------------------------------------------------------

/// The filleted box is 10 × 20 × 30 with every edge rounded by 1 mm.
const FILLET: f64 = 1.0;
const TOP: f64 = 30.0;

/// Edges that lie entirely in the top fillet band (`z ≥ 29`), degenerated pole edges included.
fn top_edges(s: &Shape) -> Vec<EdgeId> {
    s.edges()
        .filter(|&e| {
            let [a, b] = s.edge_vertices(e);
            let ends = [s.vertex_point(a), s.vertex_point(b)];
            let mid = s
                .curve(e)
                .and_then(|(c, t0, t1)| c.point_at(0.5 * (t0 + t1)))
                .unwrap_or(ends[0]);
            ends.iter()
                .chain([&mid])
                .all(|p| p.z >= TOP - FILLET - 1e-7)
        })
        .collect()
}

#[test]
fn filleted_box_top_projects_to_lines_and_arcs() {
    let Some(fb) = load("filleted_box") else {
        return;
    };
    let edges = top_edges(&fb);
    let degenerated = edges.iter().filter(|e| fb.curve(**e).is_none()).count();
    let segs = kernel().project_edges(&fb, &edges, &Plane::XY).unwrap();
    let corners = [(1.0, 1.0), (9.0, 1.0), (1.0, 19.0), (9.0, 19.0)];
    let mut arcs = 0;
    let mut lines = 0;
    for seg in &segs {
        // End points are the projected vertices of the segment's edge.
        let e = EdgeId(u32::try_from(seg.tag.0).unwrap());
        let [a, b] = fb.edge_vertices(e);
        let ends = [fb.vertex_point(a), fb.vertex_point(b)].map(|p| Point3::new(p.x, p.y, 0.0));
        for p in [seg.start().unwrap(), seg.end().unwrap()] {
            assert!(p.z.abs() < 1e-12, "{seg:?}");
            assert!(
                ends.iter().any(|q| close(p, *q, 1e-7)),
                "{e:?}: {p:?} is not one of {ends:?}"
            );
        }
        match &seg.curve {
            Curve3::Circle { frame, radius } => {
                arcs += 1;
                assert!((radius - FILLET).abs() < 1e-9, "{radius}");
                assert!(
                    corners
                        .iter()
                        .any(|&(x, y)| close(frame.origin, Point3::new(x, y, 0.0), 1e-9)),
                    "{frame:?}"
                );
                assert_eq!(frame.normal, Dir3::Z);
                assert!(
                    ((seg.t1 - seg.t0).abs() - FRAC_PI_2).abs() < 1e-9,
                    "{seg:?}"
                );
            }
            Curve3::Line { .. } => lines += 1,
            other => panic!("{e:?}: {other:?}"),
        }
    }
    // 4 top-face lines, 4 lines where the top fillets meet the sides, 8 corner arcs in vertical
    // planes (seen edge-on, so lines), 4 horizontal corner arcs.
    assert_eq!(arcs, 4, "{segs:?}");
    assert_eq!(lines, 16, "{segs:?}");
    assert_eq!(segs.len() + degenerated, edges.len());
    assert!(degenerated > 0, "the pole edges are skipped");
}

#[test]
fn face_wires_project_to_ordered_closed_loops() {
    let Some(fb) = load("filleted_box") else {
        return;
    };
    // Every face of the filleted box, wire by wire, onto a tilted plane: head to tail, closed.
    let onto = plane(Point3::new(2.0, 3.0, 4.0), Vec3::new(0.2, -0.3, 0.9));
    for f in fb.faces() {
        for wire in fb.face_wires(f) {
            let segs = kernel().project_edges(&fb, &wire, &onto).unwrap();
            if segs.is_empty() {
                continue;
            }
            for c in chains(&segs, 1e-6) {
                assert!(c.1, "{f:?}: {segs:?}");
            }
        }
    }
    let Some(plate) = load("plate_with_holes") else {
        return;
    };
    // A cap: the planar face with the 1024 hole loops.
    let top = plate
        .faces()
        .max_by_key(|f| plate.face_wires(*f).len())
        .unwrap();
    let wires = plate.face_wires(top);
    assert_eq!(wires.len(), 1025);
    let outer = kernel()
        .project_edges(&plate, &wires[0], &Plane::XY)
        .unwrap();
    assert_eq!(chains(&outer, 1e-6), [(4, true)], "outer loop first");
    for hole in &wires[1..] {
        let segs = kernel().project_edges(&plate, hole, &Plane::XY).unwrap();
        assert_eq!(chains(&segs, 1e-6), [(1, true)]);
        assert!(matches!(segs[0].curve, Curve3::Circle { .. }));
    }
}

// ---- Done-when 3 -------------------------------------------------------------------------

/// The `bspline_patch` fixture: a bicubic Bezier over [0, 30]² with x = 30u, y = 30v and the
/// four inner poles raised to 5, so z = 45 u(1−u) v(1−v).
fn patch_z(x: f64, y: f64) -> f64 {
    let (u, v) = (x / 30.0, y / 30.0);
    45.0 * u * (1.0 - u) * v * (1.0 - v)
}

/// The largest distance of samples of `segs` from the true intersection: off the plane, and
/// off the surface (vertical distance, which bounds the distance to the surface).
fn section_error(segs: &[CurveSeg], cut: &Plane) -> f64 {
    let mut worst: f64 = 0.0;
    for seg in segs {
        for k in 0..=200 {
            let t = seg.t0 + (seg.t1 - seg.t0) * f64::from(k) / 200.0;
            let p = seg.curve.point_at(t).unwrap();
            assert!(
                (-1e-6..=30.0 + 1e-6).contains(&p.x) && (-1e-6..=30.0 + 1e-6).contains(&p.y),
                "{p:?}"
            );
            let off_plane = (p - cut.origin).dot(cut.normal.get()).abs();
            let off_surface = (p.z - patch_z(p.x, p.y)).abs();
            worst = worst.max(off_plane).max(off_surface);
        }
    }
    worst
}

#[test]
fn bspline_patch_sections_follow_the_true_intersection() {
    let Some(patch) = load("bspline_patch") else {
        return;
    };
    let cuts = [
        plane(Point3::new(12.0, 0.0, 0.0), Vec3::X),
        plane(Point3::new(0.0, 21.0, 0.0), Vec3::Y),
        plane(Point3::new(15.0, 15.0, 2.0), Vec3::new(1.0, -0.5, 0.2)),
        plane(Point3::new(15.0, 15.0, 1.5), Vec3::new(0.05, 0.1, 1.0)),
    ];
    for (i, cut) in cuts.iter().enumerate() {
        let segs = kernel().plane_section(&patch, cut).unwrap();
        assert!(!segs.is_empty(), "cut {i}");
        assert!(
            segs.iter()
                .all(|s| matches!(s.curve, Curve3::BSpline(_) | Curve3::Line { .. })),
            "cut {i}: {segs:?}"
        );
        let err = section_error(&segs, cut);
        println!(
            "cut {i}: {} segment(s), max deviation {err:e} mm",
            segs.len()
        );
        assert!(err < 1e-6, "cut {i}: {err:e}");
        // One open chain from boundary to boundary.
        let c = chains(&segs, 1e-6);
        assert_eq!(c.len(), 1, "cut {i}: {c:?}");
        for p in [
            segs[0].start().unwrap(),
            segs[segs.len() - 1].end().unwrap(),
        ] {
            let on_boundary = [p.x, p.y, 30.0 - p.x, 30.0 - p.y]
                .iter()
                .any(|d| d.abs() < 1e-6);
            assert!(on_boundary || c[0].1, "cut {i}: {p:?}");
        }
    }
}

// ---- other contracts -----------------------------------------------------------------------

#[test]
fn box_sections_are_line_loops_tagged_with_faces() {
    let Some(block) = load("box") else { return };
    let cut = plane(Point3::new(5.0, 10.0, 15.0), Vec3::new(0.1, 0.2, 1.0));
    let segs = kernel().plane_section(&block, &cut).unwrap();
    assert_eq!(segs.len(), 4, "{segs:?}");
    assert_eq!(chains(&segs, 1e-6), [(4, true)]);
    let mut faces: Vec<Tag> = segs.iter().map(|s| s.tag).collect();
    faces.sort();
    faces.dedup();
    assert_eq!(faces.len(), 4, "one side face per line");
    for s in &segs {
        let f = amg_brep::FaceId(u32::try_from(s.tag.0).unwrap());
        assert!(matches!(block.surface(f), Surface::Plane(_)));
    }
    // Deterministic.
    assert_eq!(kernel().plane_section(&block, &cut).unwrap(), segs);
}

#[test]
fn bad_inputs_are_rejected() {
    let Some(block) = load("box") else { return };
    let n = block.count(SubKind::Edge);
    let result = kernel().project_edges(&block, &[EdgeId(n)], &Plane::XY);
    assert!(
        matches!(result, Err(KernelError::InvalidInput { what: "edges", .. })),
        "{result:?}"
    );
    let skew = Plane {
        x: Dir3::Z,
        ..Plane::XY
    };
    let result = kernel().plane_section(&block, &skew);
    assert!(
        matches!(result, Err(KernelError::InvalidInput { what: "plane", .. })),
        "{result:?}"
    );
}

#[test]
fn bspline_edges_project_exactly() {
    let Some(patch) = load("bspline_patch") else {
        return;
    };
    let onto = plane(Point3::new(0.0, 0.0, -5.0), Vec3::new(0.3, 0.1, 1.0));
    let edges: Vec<EdgeId> = patch.edges().collect();
    let segs = kernel().project_edges(&patch, &edges, &onto).unwrap();
    assert_eq!(segs.len(), edges.len());
    // The boundary edges of the patch lie in z = 0 (straight lines in x/y), so they come back as
    // lines (recognised from their B-spline form) whose ends project from the vertices.
    for seg in &segs {
        assert!(matches!(seg.curve, Curve3::Line { .. }), "{seg:?}");
    }
    assert_eq!(chains(&segs, 1e-6), [(4, true)]);
}

/// Done-when 2's projection is exactly the rounded rectangle when the box is cut at mid-height:
/// the section turned back into a face (BREP-03) has the analytic area. Golden of its mesh.
#[test]
fn golden_filleted_box_mid_section_face() {
    let Some(fb) = load("filleted_box") else {
        return;
    };
    let cut = plane(Point3::new(0.0, 0.0, 15.0), Vec3::Z);
    let segs = kernel().plane_section(&fb, &cut).unwrap();
    assert_eq!(chains(&segs, 1e-6), [(8, true)], "{segs:?}");
    let arcs = segs
        .iter()
        .filter(|s| matches!(s.curve, Curve3::Circle { .. }))
        .count();
    assert_eq!(arcs, 4);
    let face = kernel()
        .make_face(&Surface::Plane(cut), &FaceBounds::Loops(vec![segs]), 1e-6)
        .unwrap()
        .face;
    let area = face.face_area(amg_brep::FaceId(0));
    let exact = 10.0 * 20.0 - (4.0 - PI) * FILLET * FILLET;
    println!(
        "section face area {area} vs exact {exact} (err {:e})",
        (area - exact).abs()
    );
    assert!((area - exact).abs() < 1e-9, "{area} vs {exact}");
    let mesh = kernel()
        .tessellate(&face, &TessQuality::display(), &JobCtx::detached())
        .unwrap();
    let (positions, triangles, _) = mesh.to_indexed_parts();
    amg_testkit::assert_golden!(
        "brep",
        "section_filleted_box_mid",
        TriSoup {
            positions: &positions,
            triangles: &triangles,
        }
    );
}
