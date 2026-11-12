//! BREP-08: tessellation (S02 §8) on the `testdata/brep` fixtures.
//!
//! Done-when 2 (watertight indexed parts, volume within the deflection bound), 5
//! (`face_of_triangle` against `face_ranges`), the `TessMesh` layout guarantees, determinism,
//! `tessellate_many`, `tessellate_edge`, cancellation and the geometry goldens
//! (`tests/golden/brep/`). Done-when 3 (sphere convergence) is a unit test in `src/tess.rs`,
//! Done-when 4 is in `tests/stress.rs`, Done-when 1 in `benches/tess.rs`.
#![cfg(feature = "occt")]
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

mod common;

use std::time::{Duration, Instant};

use amg_brep::{
    BrepKernel, EdgeId, FaceId, JobCtx, KernelConfig, KernelError, OcctKernel, Shape, SubKind,
    TessMesh, TessQuality,
};
use amg_core::Point3;
use amg_jobs::CancelToken;
use amg_testkit::{TriSoup, metrics, signed_volume};
use common::{FIXTURES, OPEN_FIXTURES};

fn load(name: &str) -> Option<Shape> {
    let path = amg_testkit::testdata(&format!("brep/{name}.brep"))?;
    let file = std::fs::File::open(path).expect("open fixture");
    Some(Shape::read_text(std::io::BufReader::new(file)).expect("read fixture"))
}

fn kernel() -> OcctKernel {
    OcctKernel::new(KernelConfig::default())
}

fn closed_fixtures() -> impl Iterator<Item = &'static str> {
    FIXTURES
        .into_iter()
        .filter(|name| !OPEN_FIXTURES.contains(name))
}

/// Exact surface area of `shape` (sum of the faces' `BRepGProp` areas).
fn exact_area(shape: &Shape) -> f64 {
    shape.faces().map(|f| shape.face_area(f)).sum()
}

/// The layout guarantees of `TessMesh` (S02 §8 and the S10 request).
fn check_layout(name: &str, shape: &Shape, mesh: &TessMesh) {
    let n_vertices = mesh.positions.len();
    assert_eq!(mesh.normals.len(), n_vertices, "{name}");
    assert_eq!(mesh.weld.len(), n_vertices, "{name}");
    assert!(mesh.uvs.is_none(), "{name}");
    for n in &mesh.normals {
        assert!((n.length() - 1.0).abs() < 1e-9, "{name}: normal {n:?}");
    }
    // One range per face, in index order, tiling the triangle list.
    assert_eq!(
        mesh.face_ranges.len(),
        shape.count(SubKind::Face) as usize,
        "{name}"
    );
    let mut next = 0;
    for range in &mesh.face_ranges {
        assert_eq!(range.start, next, "{name}");
        assert!(range.end >= range.start, "{name}");
        next = range.end;
    }
    assert_eq!(next as usize, mesh.tris.len(), "{name}");
    // Vertices are per face: each face's triangles use a block no other face touches.
    let mut owner = vec![u32::MAX; n_vertices];
    for (f, range) in mesh.face_ranges.iter().enumerate() {
        for tri in &mesh.tris[range.start as usize..range.end as usize] {
            for &v in tri {
                let slot = &mut owner[v as usize];
                assert!(
                    *slot == u32::MAX || *slot == f as u32,
                    "{name}: vertex {v} shared by faces {} and {f}",
                    *slot
                );
                *slot = f as u32;
            }
        }
    }
    // One polyline per edge, in index order.
    assert_eq!(mesh.edges.len(), shape.count(SubKind::Edge) as usize);
    for (e, polyline) in mesh.edges.iter().enumerate() {
        assert_eq!(polyline.edge, EdgeId(e as u32), "{name}");
        assert!(
            polyline.points.end as usize <= mesh.edge_points.len(),
            "{name}"
        );
    }
}

#[test]
fn every_fixture_tessellates_with_the_documented_layout() {
    let kernel = kernel();
    for name in FIXTURES {
        let Some(shape) = load(name) else { return };
        let (mesh, profile) = kernel
            .tessellate_profiled(&shape, &TessQuality::display(), &JobCtx::detached())
            .unwrap();
        assert!(mesh.triangle_count() > 0, "{name}");
        assert_eq!(profile.weld_mismatches, 0, "{name}");
        check_layout(name, &shape, &mesh);
        eprintln!(
            "{name}: {} tris, {} vertices, deflection {:.4} mm; copy {:.2} ms, mesh {:.2} ms, \
             extract {:.2} ms",
            mesh.triangle_count(),
            mesh.positions.len(),
            profile.deflection,
            profile.copy_seconds * 1e3,
            profile.mesh_seconds * 1e3,
            profile.extract_seconds * 1e3
        );
    }
}

/// Done-when 5.
#[test]
fn face_of_triangle_matches_face_ranges() {
    let kernel = kernel();
    for name in FIXTURES {
        let Some(shape) = load(name) else { return };
        let mesh = kernel
            .tessellate(&shape, &TessQuality::preview(), &JobCtx::detached())
            .unwrap();
        for (f, range) in mesh.face_ranges.iter().enumerate() {
            for tri in range.clone() {
                assert_eq!(
                    mesh.face_of_triangle(tri),
                    FaceId(f as u32),
                    "{name} tri {tri}"
                );
            }
        }
        let (_, tris, faces) = mesh.to_indexed_parts();
        assert_eq!(tris.len(), faces.len());
        assert!(faces.windows(2).all(|w| w[0] <= w[1]), "{name}");
    }
}

/// Done-when 2: watertight, and the enclosed volume within the deflection bound.
#[test]
fn indexed_parts_of_closed_fixtures_are_watertight() {
    let kernel = kernel();
    for quality in [TessQuality::preview(), TessQuality::display()] {
        for name in closed_fixtures() {
            let Some(shape) = load(name) else { return };
            let (mesh, profile) = kernel
                .tessellate_profiled(&shape, &quality, &JobCtx::detached())
                .unwrap();
            let (positions, triangles, _) = mesh.to_indexed_parts();
            let soup = TriSoup {
                positions: &positions,
                triangles: &triangles,
            };
            let m = metrics(soup);
            assert_eq!(m.boundary_edges, 0, "{name}: {m:?}");
            assert_eq!(m.non_manifold_edges, 0, "{name}: {m:?}");
            // Every weld id is used once: no duplicate positions left.
            assert_eq!(m.vertices, positions.len(), "{name}");

            let exact = shape.volume().unwrap();
            let area = exact_area(&shape);
            let volume = signed_volume(soup);
            // A surface within `d` of the exact one encloses a volume within `d · area`.
            let bound = profile.deflection * area;
            eprintln!(
                "{name} ({:.4} mm): volume {volume:.6} vs {exact:.6} (err {:.3e}, bound {bound:.3e}), \
                 euler {}",
                profile.deflection,
                (volume - exact).abs(),
                m.euler
            );
            assert!(volume > 0.0, "{name}: triangles face inwards");
            assert!(
                (volume - exact).abs() <= bound,
                "{name}: volume {volume} vs exact {exact}, bound {bound}"
            );
            // Convex parts mesh inscribed; concave ones (the plate's holes) are not.
            if ["box", "cylinder", "cone", "sphere"].contains(&name) {
                assert!(m.area <= area * (1.0 + 1e-9), "{name}: inscribed mesh");
            }
        }
    }
}

#[test]
fn open_fixtures_have_boundaries_but_no_non_manifold_edges() {
    let kernel = kernel();
    for name in OPEN_FIXTURES {
        let Some(shape) = load(name) else { return };
        let mesh = kernel
            .tessellate(&shape, &TessQuality::display(), &JobCtx::detached())
            .unwrap();
        let (positions, triangles, _) = mesh.to_indexed_parts();
        let m = metrics(TriSoup {
            positions: &positions,
            triangles: &triangles,
        });
        assert!(m.boundary_edges > 0, "{name}: {m:?}");
        assert_eq!(m.non_manifold_edges, 0, "{name}: {m:?}");
    }
}

/// Vertex normals agree with the triangles' orientation (both face out of the material).
#[test]
fn normals_face_outwards() {
    let kernel = kernel();
    for name in closed_fixtures() {
        let Some(shape) = load(name) else { return };
        let mesh = kernel
            .tessellate(&shape, &TessQuality::display(), &JobCtx::detached())
            .unwrap();
        for tri in &mesh.tris {
            let [a, b, c] = tri.map(|v| mesh.positions[v as usize]);
            let geometric = (b - a).cross(c - a);
            if geometric.length() < 1e-12 {
                continue;
            }
            for &v in tri {
                assert!(
                    mesh.normals[v as usize].dot(geometric.normalize()) > 0.0,
                    "{name}: vertex normal against triangle {tri:?}"
                );
            }
        }
    }
}

/// The box: six planar faces, two triangles each, with exact face normals.
#[test]
fn box_mesh_is_minimal_and_exact() {
    let Some(shape) = load("box") else { return };
    let mesh = kernel()
        .tessellate(&shape, &TessQuality::display(), &JobCtx::detached())
        .unwrap();
    assert_eq!(mesh.triangle_count(), 12);
    assert_eq!(mesh.positions.len(), 24);
    for range in &mesh.face_ranges {
        assert_eq!(range.len(), 2);
    }
    let (positions, tris, _) = mesh.to_indexed_parts();
    assert_eq!(positions.len(), 8);
    assert_eq!(tris.len(), 12);
    // Each edge polyline is the straight segment between its vertices.
    for polyline in &mesh.edges {
        let points =
            &mesh.edge_points[polyline.points.start as usize..polyline.points.end as usize];
        assert_eq!(points.len(), 2, "{polyline:?}");
        let [v0, v1] = shape.edge_vertices(polyline.edge);
        let ends = [shape.vertex_point(v0), shape.vertex_point(v1)];
        for (p, q) in points.iter().zip(ends) {
            assert!(p.distance(q) < 1e-9, "{p:?} vs {q:?}");
        }
    }
}

/// Edge polylines run from vertex to vertex and stay on the edge within the deflection.
#[test]
fn edge_polylines_follow_their_edges() {
    let kernel = kernel();
    for name in FIXTURES {
        let Some(shape) = load(name) else { return };
        let (mesh, profile) = kernel
            .tessellate_profiled(&shape, &TessQuality::display(), &JobCtx::detached())
            .unwrap();
        for polyline in &mesh.edges {
            let e = polyline.edge;
            let points =
                &mesh.edge_points[polyline.points.start as usize..polyline.points.end as usize];
            if shape.curve(e).is_none() {
                assert!(points.is_empty(), "{name}: degenerated {e:?}");
                continue;
            }
            assert!(
                points.len() >= 2,
                "{name}: {e:?} has {} points",
                points.len()
            );
            let [v0, v1] = shape.edge_vertices(e);
            let tol = shape.tolerance(amg_brep::SubShape {
                kind: SubKind::Edge,
                index: e.0,
            }) + 1e-9;
            assert!(
                points[0].distance(shape.vertex_point(v0)) <= tol,
                "{name} {e:?}"
            );
            assert!(
                points[points.len() - 1].distance(shape.vertex_point(v1)) <= tol,
                "{name} {e:?}"
            );
            // The independent `tessellate_edge` discretisation lies within the deflection of
            // the polyline's vertices' curve: compare lengths.
            let reference = kernel
                .tessellate_edge(&shape, e, &TessQuality::display())
                .unwrap();
            let length =
                |pts: &[Point3]| -> f64 { pts.windows(2).map(|w| w[0].distance(w[1])).sum() };
            let (a, b) = (length(points), length(&reference));
            assert!(
                (a - b).abs() <= 2.0 * profile.deflection.max(1e-6) * points.len() as f64,
                "{name} {e:?}: polyline length {a} vs {b}"
            );
        }
    }
}

#[test]
fn tessellation_is_deterministic() {
    let kernel = kernel();
    for name in [
        "filleted_box",
        "torus",
        "located_compound",
        "cylinder_mirrored",
    ] {
        let Some(shape) = load(name) else { return };
        let first = kernel
            .tessellate(&shape, &TessQuality::display(), &JobCtx::detached())
            .unwrap();
        for _ in 0..3 {
            let again = kernel
                .tessellate(&shape, &TessQuality::display(), &JobCtx::detached())
                .unwrap();
            assert_eq!(format!("{again:?}"), format!("{first:?}"), "{name}");
        }
        // A second, independently read copy meshes the same.
        let other = load(name).unwrap();
        let other = kernel
            .tessellate(&other, &TessQuality::display(), &JobCtx::detached())
            .unwrap();
        assert_eq!(format!("{other:?}"), format!("{first:?}"), "{name}");
    }
}

#[test]
fn tessellate_many_matches_tessellate() {
    let kernel = kernel();
    let shapes: Vec<Shape> = FIXTURES.into_iter().filter_map(load).collect();
    if shapes.len() != FIXTURES.len() {
        return;
    }
    let q = TessQuality::display();
    let many = kernel.tessellate_many(&shapes, &q, &JobCtx::detached());
    assert_eq!(many.len(), shapes.len());
    for (shape, got) in shapes.iter().zip(many) {
        let one = kernel.tessellate(shape, &q, &JobCtx::detached()).unwrap();
        assert_eq!(format!("{:?}", got.unwrap()), format!("{one:?}"));
    }
    assert!(
        kernel
            .tessellate_many(&[], &q, &JobCtx::detached())
            .is_empty()
    );
}

#[test]
fn finer_quality_gives_more_triangles() {
    let Some(shape) = load("torus") else { return };
    let kernel = kernel();
    let counts: Vec<usize> = [
        TessQuality::preview(),
        TessQuality::display(),
        TessQuality::export(),
    ]
    .iter()
    .map(|q| {
        kernel
            .tessellate(&shape, q, &JobCtx::detached())
            .unwrap()
            .triangle_count()
    })
    .collect();
    assert!(counts.windows(2).all(|w| w[0] < w[1]), "{counts:?}");
    // `min_size` coarsens nothing it does not have to; an absolute quality works too.
    let absolute = TessQuality {
        linear: 0.05,
        relative: false,
        min_size: 0.01,
        ..TessQuality::display()
    };
    let mesh = kernel
        .tessellate(&shape, &absolute, &JobCtx::detached())
        .unwrap();
    assert!(mesh.triangle_count() > 0);
}

#[test]
fn bad_qualities_are_invalid_input() {
    let Some(shape) = load("box") else { return };
    let kernel = kernel();
    let bad = TessQuality {
        linear: -1.0,
        ..TessQuality::display()
    };
    let result = kernel.tessellate(&shape, &bad, &JobCtx::detached());
    assert!(
        matches!(result, Err(KernelError::InvalidInput { .. })),
        "{result:?}"
    );
    let result = kernel.tessellate_edge(&shape, EdgeId(0), &bad);
    assert!(
        matches!(result, Err(KernelError::InvalidInput { .. })),
        "{result:?}"
    );
}

#[test]
fn tessellate_edge_samples_the_curve() {
    let Some(shape) = load("cylinder") else {
        return;
    };
    let kernel = kernel();
    let q = TessQuality {
        linear: 0.001,
        relative: false,
        ..TessQuality::display()
    };
    let mut circles = 0;
    for e in shape.edges() {
        let Some((amg_brep::geom::Curve3::Circle { frame, radius }, _, _)) = shape.curve(e) else {
            continue;
        };
        circles += 1;
        let points = kernel.tessellate_edge(&shape, e, &q).unwrap();
        assert!(points.len() > 8, "{e:?}: {} points", points.len());
        for p in &points {
            let local = *p - frame.origin;
            let height = local.dot(frame.normal.get());
            let radial = (local - frame.normal.get() * height).length();
            assert!((radial - radius).abs() < 1e-9, "{p:?}");
        }
        // Chords stay within the linear deflection: sagitta r(1 - cos(θ/2)).
        for w in points.windows(2) {
            let chord = w[0].distance(w[1]);
            let half = (chord / (2.0 * radius)).asin();
            assert!(
                radius * (1.0 - half.cos()) <= q.linear * 1.01,
                "chord {chord}"
            );
        }
        let [v0, v1] = shape.edge_vertices(e);
        assert!(points[0].distance(shape.vertex_point(v0)) < 1e-6);
        assert!(points[points.len() - 1].distance(shape.vertex_point(v1)) < 1e-6);
    }
    assert!(circles >= 2);
    // Out of range.
    let result = kernel.tessellate_edge(&shape, EdgeId(999), &q);
    assert!(
        matches!(result, Err(KernelError::InvalidInput { .. })),
        "{result:?}"
    );
}

#[test]
fn degenerated_edges_are_invalid_input_for_tessellate_edge() {
    let Some(shape) = load("sphere") else { return };
    let kernel = kernel();
    let degenerated: Vec<EdgeId> = shape
        .edges()
        .filter(|&e| shape.curve(e).is_none())
        .collect();
    assert!(!degenerated.is_empty());
    for e in degenerated {
        let result = kernel.tessellate_edge(&shape, e, &TessQuality::display());
        assert!(
            matches!(result, Err(KernelError::InvalidInput { .. })),
            "{result:?}"
        );
    }
}

#[test]
fn a_cancelled_context_stops_tessellation() {
    let Some(shape) = load("plate_with_holes") else {
        return;
    };
    let kernel = kernel();
    let token = CancelToken::new();
    token.cancel();
    let ctx = JobCtx::with_token(token);
    let result = kernel.tessellate(&shape, &TessQuality::display(), &ctx);
    assert!(matches!(result, Err(KernelError::Cancelled)), "{result:?}");
    let many = kernel.tessellate_many(&[shape.clone(), shape], &TessQuality::display(), &ctx);
    assert!(
        many.iter()
            .all(|r| matches!(r, Err(KernelError::Cancelled)))
    );
}

/// Cancellation reaches into `BRepMesh_IncrementalMesh` through the progress bridge.
#[test]
fn cancel_during_meshing_returns_cancelled() {
    let Some(shape) = load("plate_with_holes") else {
        return;
    };
    let kernel = kernel();
    let fine = TessQuality::export();
    let start = Instant::now();
    kernel
        .tessellate(&shape, &fine, &JobCtx::detached())
        .unwrap();
    let full = start.elapsed();
    if full < Duration::from_millis(40) {
        eprintln!("export mesh takes only {full:?}; nothing to interrupt");
        return;
    }
    let token = CancelToken::new();
    let ctx = JobCtx::with_token(token.clone());
    let canceller = std::thread::spawn({
        let token = token.clone();
        move || {
            std::thread::sleep(full / 4);
            token.cancel();
        }
    });
    let start = Instant::now();
    let result = kernel.tessellate(&shape, &fine, &ctx);
    let took = start.elapsed();
    canceller.join().unwrap();
    eprintln!(
        "full export mesh {full:?}; cancelled after {:?}, returned after {took:?}",
        full / 4
    );
    assert!(matches!(result, Err(KernelError::Cancelled)), "{result:?}");
    if std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1") {
        assert!(took < full * 3 / 4, "{took:?} of {full:?}");
    }
}

/// `TessQuality::display` on a few fixtures, against `tests/golden/brep/`.
#[test]
fn golden_brep_display_tessellations() {
    let kernel = kernel();
    for name in [
        "box",
        "cylinder",
        "sphere",
        "torus",
        "filleted_box",
        "cylinder_mirrored",
    ] {
        let Some(shape) = load(name) else { return };
        let mesh = kernel
            .tessellate(&shape, &TessQuality::display(), &JobCtx::detached())
            .unwrap();
        let (positions, triangles, _) = mesh.to_indexed_parts();
        amg_testkit::assert_golden!(
            "brep",
            &format!("tess_display_{name}"),
            TriSoup {
                positions: &positions,
                triangles: &triangles,
            }
        );
    }
}

/// Done-when 1 in a test: the median of 11 display meshes of the 1 030-face plate, topology
/// copy included. Printed always, asserted (< 100 ms) only with `AMG_STRICT_TIMING=1`; the
/// criterion number is `budget/brep-tess-1000` in `benches/tess.rs`.
#[test]
fn plate_display_mesh_time() {
    let Some(shape) = load("plate_with_holes") else {
        return;
    };
    let kernel = kernel();
    let mut runs: Vec<(Duration, f64)> = (0..11)
        .map(|_| {
            let start = Instant::now();
            let (_, profile) = kernel
                .tessellate_profiled(&shape, &TessQuality::display(), &JobCtx::detached())
                .unwrap();
            (start.elapsed(), profile.copy_seconds)
        })
        .collect();
    runs.sort_by_key(|run| run.0);
    let (median, copy) = runs[runs.len() / 2];
    eprintln!(
        "plate_with_holes display: median {median:?}, topology copy {:.2} ms ({:.1} %)",
        copy * 1e3,
        100.0 * copy / median.as_secs_f64()
    );
    if std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1") {
        assert!(median < Duration::from_millis(100), "{median:?}");
    }
}

/// Done-when 1 context (prints only): ~1000 ordinary faces mesh well inside the budget; the
/// plate misses it because of its two 1024-hole planar caps (OCCT's `BRepMesh_Delaun`), see
/// the S02 Refinement log. Medians of 7.
#[test]
fn thousand_ordinary_faces_mesh_time() {
    let (Some(plate), Some(filleted)) = (load("plate_with_holes"), load("filleted_box")) else {
        return;
    };
    let kernel = kernel();
    let q = TessQuality::display();
    let median = |f: &dyn Fn()| {
        let mut runs: Vec<Duration> = (0..7)
            .map(|_| {
                let start = Instant::now();
                f();
                start.elapsed()
            })
            .collect();
        runs.sort();
        runs[3]
    };
    // 38 filleted boxes = 988 faces, side by side (rigid moves share the TShapes).
    let boxes: Vec<Shape> = (0..38)
        .map(|i| {
            filleted
                .transformed(&amg_core::Transform::from_translation(amg_core::Vec3::new(
                    15.0 * f64::from(i),
                    0.0,
                    0.0,
                )))
                .unwrap()
        })
        .collect();
    let many = median(&|| {
        for r in kernel.tessellate_many(&boxes, &q, &JobCtx::detached()) {
            r.unwrap();
        }
    });
    // The plate's faces without its two big caps: 1028 faces as standalone shapes.
    let faces: Vec<Shape> = plate
        .faces()
        .filter(|&f| plate.face_edges(f).len() < 100)
        .map(|f| {
            plate
                .sub_shape(amg_brep::SubShape {
                    kind: SubKind::Face,
                    index: f.0,
                })
                .unwrap()
        })
        .collect();
    // Same absolute deflection as the whole plate gets.
    let plate_q = TessQuality {
        linear: 0.001 * plate.bbox().diagonal(),
        relative: false,
        ..q
    };
    let rest = median(&|| {
        for r in kernel.tessellate_many(&faces, &plate_q, &JobCtx::detached()) {
            r.unwrap();
        }
    });
    eprintln!(
        "display: 38 filleted boxes (988 faces) via tessellate_many {many:?}; plate without its \
         2 caps ({} faces) via tessellate_many {rest:?}",
        faces.len()
    );
}
