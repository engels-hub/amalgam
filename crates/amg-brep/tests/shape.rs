//! BREP-02 acceptance: shape handle, sub-shape index, geometry queries and `.brep` I/O.
#![cfg(feature = "occt")]
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

mod common;

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::path::Path;
use std::process::Command;

use amg_brep::geom::{Curve3, GeomClass, Surface};
use amg_brep::{EdgeId, FaceId, KernelError, Shape, ShapeKind, SubKind, SubShape, Transform, Vec3};
use common::{FIXTURES, OPEN_FIXTURES, face_classes};

const PROBE_OUTPUT: &str = "AMG_BREP_PROBE_OUTPUT";
const PROBE_CHURN: &str = "AMG_BREP_PROBE_CHURN";

fn load(name: &str) -> Option<Shape> {
    let path = amg_testkit::testdata(&format!("brep/{name}.brep"))?;
    let file = std::fs::File::open(path).expect("open fixture");
    Some(Shape::read_text(std::io::BufReader::new(file)).expect("read fixture"))
}

fn write_bin(shape: &Shape) -> Vec<u8> {
    let mut bytes = Vec::new();
    shape.write_bin(&mut bytes).expect("write_bin");
    bytes
}

fn assert_near(a: Vec3, b: Vec3, eps: f64) {
    assert!((a - b).abs().max_element() <= eps, "{a:?} != {b:?}");
}

/// Everything index-dependent, in index order, with floats as exact bit patterns.
fn fingerprint(shape: &Shape) -> String {
    let mut out = String::new();
    for f in shape.faces() {
        let area = shape.face_area(f).to_bits();
        let edges = shape.face_edges(f);
        writeln!(out, "f{} {area:x} {edges:?}", f.0).unwrap();
    }
    for e in shape.edges() {
        let (faces, vertices) = (shape.edge_faces(e), shape.edge_vertices(e));
        writeln!(out, "e{} {faces:?} {vertices:?}", e.0).unwrap();
    }
    for v in shape.vertices() {
        let p = shape.vertex_point(v);
        let bits = [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
        writeln!(out, "v{} {bits:x?} {:?}", v.0, shape.vertex_edges(v)).unwrap();
    }
    out
}

fn all_fingerprints() -> Option<String> {
    let mut out = String::new();
    for name in FIXTURES {
        writeln!(out, "# {name}\n{}", fingerprint(&load(name)?)).unwrap();
    }
    Some(out)
}

// ---- Done-when 1 ----

#[test]
fn counts_and_face_classes_match_expectations() {
    let Some(path) = amg_testkit::testdata("brep/expected.json") else {
        return;
    };
    let text = std::fs::read_to_string(path).expect("read expectations");
    let expected: BTreeMap<String, serde_json::Value> =
        serde_json::from_str(&text).expect("parse expectations");
    assert_eq!(expected.len(), FIXTURES.len());
    let mut has_large_part = false;
    for name in FIXTURES {
        let Some(shape) = load(name) else { return };
        let want = &expected[name];
        assert_eq!(
            u64::from(shape.count(SubKind::Face)),
            want["faces"].as_u64().unwrap(),
            "{name} faces"
        );
        assert_eq!(
            u64::from(shape.count(SubKind::Edge)),
            want["edges"].as_u64().unwrap(),
            "{name} edges"
        );
        assert_eq!(
            u64::from(shape.count(SubKind::Vertex)),
            want["vertices"].as_u64().unwrap(),
            "{name} vertices"
        );
        assert_eq!(
            face_classes(&shape),
            want["face_classes"].as_str().unwrap(),
            "{name} classes"
        );
        has_large_part |= shape.count(SubKind::Face) > 500;
    }
    assert!(has_large_part, "need a fixture with more than 500 faces");
}

// ---- Done-when 2 ----

/// Not a test of its own: the child half of `indices_match_across_processes`.
#[test]
fn probe_child() {
    let Ok(output) = std::env::var(PROBE_OUTPUT) else {
        return;
    };
    // Shift the allocator's state so that any address-dependent ordering would show.
    let churn: usize = std::env::var(PROBE_CHURN).unwrap().parse().unwrap();
    let mut held = Vec::new();
    for i in 0..churn {
        held.push(vec![0u8; 16 + (i * 37) % 4096]);
        if i % 3 == 0 {
            held.swap_remove(i / 2);
        }
    }
    let prints = all_fingerprints().expect("fixtures present in the child");
    std::fs::write(output, prints).unwrap();
    drop(held);
}

#[test]
fn indices_match_across_processes() {
    let Some(expected) = all_fingerprints() else {
        return;
    };
    let exe = std::env::current_exe().unwrap();
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR"));
    for (run, churn) in [0usize, 50_000].into_iter().enumerate() {
        let output = dir.join(format!("brep_probe_{}_{run}.txt", std::process::id()));
        let status = Command::new(&exe)
            .args(["--exact", "probe_child", "--nocapture"])
            .env(PROBE_OUTPUT, &output)
            .env(PROBE_CHURN, churn.to_string())
            .status()
            .unwrap();
        assert!(status.success(), "child process failed");
        let got = std::fs::read_to_string(&output).unwrap();
        std::fs::remove_file(&output).unwrap();
        assert!(got == expected, "indices differ in child run {run}");
    }
}

#[test]
fn indices_survive_a_binary_round_trip() {
    for name in FIXTURES {
        let Some(shape) = load(name) else { return };
        let copy = Shape::read_bin(write_bin(&shape).as_slice()).unwrap();
        assert!(
            fingerprint(&shape) == fingerprint(&copy),
            "{name}: indices changed"
        );
    }
}

// ---- Done-when 3 ----

#[test]
fn binary_rewrite_is_byte_identical() {
    for name in FIXTURES {
        let Some(shape) = load(name) else { return };
        let first = write_bin(&shape);
        let copy = Shape::read_bin(first.as_slice()).unwrap();
        assert!(
            write_bin(&copy) == first,
            "{name}: bytes changed on rewrite"
        );
    }
}

// ---- Done-when 5 ----

#[test]
fn adjacency_invariants_hold() {
    for name in FIXTURES {
        let Some(shape) = load(name) else { return };
        let open = OPEN_FIXTURES.contains(&name);
        for f in shape.faces() {
            for &e in shape.face_edges(f) {
                assert!(
                    shape.edge_faces(e).contains(&f),
                    "{name}: {f:?} lists {e:?} but not the reverse"
                );
            }
        }
        for e in shape.edges() {
            let faces = shape.edge_faces(e);
            for &f in faces {
                assert!(
                    shape.face_edges(f).contains(&e),
                    "{name}: {e:?} lists {f:?} but not the reverse"
                );
            }
            match faces {
                [_, _] => {}
                [f] => assert!(
                    open || shape.is_seam(e, *f) || shape.curve(e).is_none(),
                    "{name}: {e:?} has one face and is neither seam nor degenerated"
                ),
                _ => panic!("{name}: {e:?} has {} faces", faces.len()),
            }
            for v in shape.edge_vertices(e) {
                assert!(
                    shape.vertex_edges(v).contains(&e),
                    "{name}: {e:?} ends at {v:?} but not the reverse"
                );
            }
        }
        for v in shape.vertices() {
            for &e in shape.vertex_edges(v) {
                assert!(
                    shape.edge_vertices(e).contains(&v),
                    "{name}: {v:?} lists {e:?} but not the reverse"
                );
            }
        }
    }
}

#[test]
fn face_wires_list_every_edge_of_the_face() {
    for name in FIXTURES {
        let Some(shape) = load(name) else { return };
        for f in shape.faces() {
            let mut from_wires: Vec<EdgeId> = shape.face_wires(f).concat();
            from_wires.sort_unstable();
            from_wires.dedup();
            let mut from_edges = shape.face_edges(f).to_vec();
            from_edges.sort_unstable();
            assert_eq!(from_wires, from_edges, "{name}: {f:?}");
        }
    }
}

#[test]
fn seam_appears_twice_in_its_wire_and_once_in_face_edges() {
    let Some(shape) = load("cylinder") else {
        return;
    };
    let lateral = shape
        .faces()
        .find(|&f| shape.surface(f).class() == GeomClass::Cylinder)
        .unwrap();
    let seam = shape
        .face_edges(lateral)
        .iter()
        .copied()
        .find(|&e| shape.is_seam(e, lateral))
        .expect("the lateral face has a seam");
    let wires = shape.face_wires(lateral);
    assert_eq!(wires.len(), 1);
    assert_eq!(wires[0].iter().filter(|&&e| e == seam).count(), 2);
    assert_eq!(
        shape
            .face_edges(lateral)
            .iter()
            .filter(|&&e| e == seam)
            .count(),
        1
    );
    assert_eq!(shape.edge_faces(seam), [lateral]);
}

#[test]
fn holes_come_after_the_outer_wire() {
    let Some(shape) = load("plate_with_holes") else {
        return;
    };
    let top = shape
        .faces()
        .max_by_key(|&f| shape.face_wires(f).len())
        .unwrap();
    let wires = shape.face_wires(top);
    assert_eq!(wires.len(), 1025, "outer wire plus 1024 holes");
    assert_eq!(wires[0].len(), 4, "the outer wire is the rectangle");
    assert!(wires[1..].iter().all(|wire| wire.len() == 1));
}

// ---- Done-when 6 ----

#[test]
fn cylinder_reports_radius_and_axis_within_1e12() {
    let Some(shape) = load("cylinder") else {
        return;
    };
    let lateral = shape
        .faces()
        .find(|&f| shape.surface(f).class() == GeomClass::Cylinder)
        .unwrap();
    let Surface::Cylinder { frame, radius } = shape.surface(lateral) else {
        panic!("not a cylinder");
    };
    assert!((radius - 5.0).abs() <= 1e-12, "radius {radius}");
    assert_near(frame.normal.get(), Vec3::Z, 1e-12);
    assert!((frame.origin.x - 3.0).abs() <= 1e-12);
    assert!((frame.origin.y - 1.0).abs() <= 1e-12);
}

#[test]
fn location_is_applied_to_surface_and_curve() {
    let Some(shape) = load("cylinder") else {
        return;
    };
    let offset = Vec3::new(10.0, -4.0, 2.5);
    let moved = shape
        .transformed(&Transform::from_translation(offset))
        .unwrap();
    let lateral = moved
        .faces()
        .find(|&f| moved.surface(f).class() == GeomClass::Cylinder)
        .unwrap();
    let Surface::Cylinder { frame, radius } = moved.surface(lateral) else {
        panic!("not a cylinder");
    };
    assert!((radius - 5.0).abs() <= 1e-12);
    assert!((frame.origin.x - 13.0).abs() <= 1e-12);
    assert!((frame.origin.y + 3.0).abs() <= 1e-12);
}

#[test]
fn mirrored_cylinder_still_reports_a_right_handed_frame() {
    let Some(shape) = load("cylinder_mirrored") else {
        return;
    };
    let lateral = shape
        .faces()
        .find(|&f| shape.surface(f).class() == GeomClass::Cylinder)
        .unwrap();
    let Surface::Cylinder { frame, radius } = shape.surface(lateral) else {
        panic!("not a cylinder");
    };
    assert!((radius - 5.0).abs() <= 1e-12);
    assert_near(
        frame.x.get().cross(frame.y.get()),
        frame.normal.get(),
        1e-12,
    );
    assert!(
        frame.normal.get().z.abs() > 1.0 - 1e-12,
        "axis stays along Z"
    );
    assert!(
        (frame.origin.x + 3.0).abs() <= 1e-12,
        "mirrored about X = 0"
    );
    assert!((frame.origin.y - 1.0).abs() <= 1e-12);
}

// ---- geometry queries ----

#[test]
fn analytic_surfaces_report_their_parameters() {
    let cone = load("cone");
    let sphere = load("sphere");
    let torus = load("torus");
    let (Some(cone), Some(sphere), Some(torus)) = (cone, sphere, torus) else {
        return;
    };

    let f = cone
        .faces()
        .find(|&f| cone.surface(f).class() == GeomClass::Cone)
        .unwrap();
    let Surface::Cone {
        ref_radius,
        half_angle,
        ..
    } = cone.surface(f)
    else {
        panic!("not a cone");
    };
    assert!((ref_radius - 5.0).abs() <= 1e-12);
    assert!(
        (half_angle.tan() + 0.3).abs() <= 1e-12,
        "half angle {half_angle}"
    );

    let Surface::Sphere { radius, .. } = sphere.surface(FaceId(0)) else {
        panic!("not a sphere");
    };
    assert!((radius - 7.0).abs() <= 1e-12);

    let Surface::Torus { major, minor, .. } = torus.surface(FaceId(0)) else {
        panic!("not a torus");
    };
    assert!((major - 20.0).abs() <= 1e-12);
    assert!((minor - 5.0).abs() <= 1e-12);
}

#[test]
fn bspline_patch_reports_poles_and_knots() {
    let Some(shape) = load("bspline_patch") else {
        return;
    };
    let Surface::BSpline(data) = shape.surface(FaceId(0)) else {
        panic!("not a B-spline");
    };
    assert_eq!((data.u_degree, data.v_degree), (3, 3));
    assert_eq!((data.nu, data.nv), (4, 4));
    assert_eq!(data.poles.len(), 16);
    assert_eq!(data.u_knots, [0.0, 1.0]);
    assert_eq!(data.u_mults, [4, 4]);
    assert_eq!(data.weights, None);
    // Row-major with u outer: the second row starts at x = 10.
    assert_near(data.poles[4], Vec3::new(10.0, 0.0, 0.0), 1e-12);
    assert_near(data.poles[1], Vec3::new(0.0, 10.0, 0.0), 1e-12);
}

#[test]
fn box_edges_are_lines_with_consistent_ends() {
    let Some(shape) = load("box") else { return };
    for e in shape.edges() {
        let (curve, t0, t1) = shape.curve(e).expect("box edges have curves");
        let Curve3::Line { origin, dir } = curve else {
            panic!("not a line");
        };
        let [a, b] = shape.edge_vertices(e);
        assert_near(origin + dir.get() * t0, shape.vertex_point(a), 1e-9);
        assert_near(origin + dir.get() * t1, shape.vertex_point(b), 1e-9);
    }
}

#[test]
fn degenerated_edges_have_no_curve() {
    let Some(shape) = load("sphere") else { return };
    let missing = shape.edges().filter(|&e| shape.curve(e).is_none()).count();
    assert_eq!(missing, 2, "both pole edges are degenerated");
}

#[test]
fn circle_edges_report_radius() {
    let Some(shape) = load("cylinder") else {
        return;
    };
    let circles: Vec<f64> = shape
        .edges()
        .filter_map(|e| match shape.curve(e)?.0 {
            Curve3::Circle { radius, .. } => Some(radius),
            _ => None,
        })
        .collect();
    assert_eq!(circles.len(), 2);
    assert!(circles.iter().all(|r| (r - 5.0).abs() <= 1e-12));
}

#[test]
fn box_face_areas_and_bbox_are_exact() {
    let Some(shape) = load("box") else { return };
    let total: f64 = shape.faces().map(|f| shape.face_area(f)).sum();
    assert!((total - 2200.0).abs() <= 1e-9, "total area {total}");
    let bbox = shape.bbox();
    assert_near(bbox.min, Vec3::ZERO, 1e-6);
    assert_near(bbox.max, Vec3::new(10.0, 20.0, 30.0), 1e-6);
    assert_eq!(bbox, shape.bbox(), "cached");
}

#[test]
fn uv_bounds_and_orientation_are_available() {
    let Some(shape) = load("box") else { return };
    let reversed = shape.faces().filter(|&f| shape.face_reversed(f)).count();
    assert_eq!(reversed, 3, "half of a box's planes oppose their surface");
    for f in shape.faces() {
        let [umin, umax, vmin, vmax] = shape.face_uv_bounds(f);
        assert!(umax > umin && vmax > vmin, "{f:?}");
    }
    let tolerance = shape.tolerance(SubShape {
        kind: SubKind::Face,
        index: 0,
    });
    assert!(tolerance > 0.0 && tolerance < 1e-3, "tolerance {tolerance}");
}

// ---- handle ----

#[test]
fn handle_identity_and_kinds() {
    let Some(shape) = load("box") else { return };
    assert_eq!(shape.kind(), ShapeKind::Solid);
    assert!(!shape.is_null());
    let clone = shape.clone();
    assert!(shape.ptr_eq(&clone));
    let copy = Shape::read_bin(write_bin(&shape).as_slice()).unwrap();
    assert!(!shape.ptr_eq(&copy));
    assert!(!shape.is_same(&copy), "a re-read shape has new TShapes");
    assert!(shape.is_same(&clone));
    assert!(format!("{shape:?}").contains("faces: 6"));
}

#[test]
fn iterators_are_exact_sized_and_zero_based() {
    let Some(shape) = load("box") else { return };
    assert_eq!(shape.faces().len(), 6);
    assert_eq!(shape.edges().len(), 12);
    assert_eq!(shape.vertices().len(), 8);
    assert_eq!(shape.faces().next(), Some(FaceId(0)));
    assert_eq!(shape.faces().last(), Some(FaceId(5)));
}

#[test]
fn solids_and_sub_shapes() {
    let Some(compound) = load("located_compound") else {
        return;
    };
    assert_eq!(compound.kind(), ShapeKind::Compound);
    let solids = compound.solids();
    assert_eq!(solids.len(), 2);
    assert!(solids.iter().all(|s| s.kind() == ShapeKind::Solid));
    assert_eq!(solids[0].count(SubKind::Face), 6);

    let face = compound
        .sub_shape(SubShape {
            kind: SubKind::Face,
            index: 3,
        })
        .unwrap();
    assert_eq!(face.kind(), ShapeKind::Face);
    let edge = compound
        .sub_shape(SubShape {
            kind: SubKind::Edge,
            index: 0,
        })
        .unwrap();
    assert_eq!(edge.kind(), ShapeKind::Edge);

    let missing = compound.sub_shape(SubShape {
        kind: SubKind::Vertex,
        index: 16,
    });
    assert!(matches!(missing, Err(KernelError::InvalidInput { .. })));
}

#[test]
fn same_tshape_at_two_locations_gets_distinct_indices() {
    let Some(compound) = load("located_compound") else {
        return;
    };
    let xs: Vec<f64> = compound
        .vertices()
        .map(|v| compound.vertex_point(v).x)
        .collect();
    assert_eq!(xs.iter().filter(|&&x| x >= 100.0).count(), 8);
    assert_eq!(xs.iter().filter(|&&x| x < 100.0).count(), 8);
}

#[test]
#[should_panic(expected = "out of range")]
fn out_of_range_id_panics() {
    let Some(shape) = load("box") else {
        panic!("out of range (fixture missing)");
    };
    let _ = shape.face_edges(FaceId(6));
}

// ---- transforms ----

#[test]
fn rigid_transform_keeps_indices_and_shares_topology() {
    let Some(shape) = load("filleted_box") else {
        return;
    };
    let rotate = Transform::from_rotation_z(0.75);
    let moved = shape.transformed(&rotate).unwrap();
    assert!(!moved.is_same(&shape));
    assert_eq!(face_classes(&moved), face_classes(&shape));
    for f in shape.faces() {
        assert_eq!(shape.face_edges(f), moved.face_edges(f));
        let drift = (shape.face_area(f) - moved.face_area(f)).abs();
        assert!(drift <= 1e-9, "{f:?} area drift {drift}");
    }
    for v in shape.vertices() {
        assert_near(
            moved.vertex_point(v),
            rotate.transform_point3(shape.vertex_point(v)),
            1e-9,
        );
    }
}

#[test]
fn identity_transform_is_same_shape() {
    let Some(shape) = load("box") else { return };
    let moved = shape.transformed(&Transform::IDENTITY).unwrap();
    assert!(moved.is_same(&shape));
}

#[test]
fn non_rigid_transforms_are_rejected() {
    let Some(shape) = load("box") else { return };
    for t in [
        Transform::from_scale(Vec3::splat(2.0)),
        Transform::from_scale(Vec3::new(-1.0, 1.0, 1.0)),
    ] {
        assert!(matches!(shape.transformed(&t), Err(KernelError::NonRigid)));
    }
}

// ---- serialization ----

#[test]
fn binary_header_is_amgb_plus_format_byte() {
    let Some(shape) = load("box") else { return };
    let bytes = write_bin(&shape);
    assert_eq!(&bytes[..4], b"AMGB");
    assert_eq!(bytes[4], amg_brep::FORMAT_VERSION);
}

#[test]
fn text_round_trip_keeps_counts_and_never_writes_triangulation() {
    let Some(shape) = load("filleted_box") else {
        return;
    };
    let mut text = Vec::new();
    shape.write_text(&mut text).unwrap();
    assert!(!String::from_utf8_lossy(&text).contains("Triangulations 1"));
    let copy = Shape::read_text(text.as_slice()).unwrap();
    assert_eq!(copy.count(SubKind::Face), shape.count(SubKind::Face));
    assert_eq!(copy.count(SubKind::Edge), shape.count(SubKind::Edge));
    assert_eq!(face_classes(&copy), face_classes(&shape));
}

#[test]
fn unreadable_data_is_a_format_error_not_a_crash() {
    let Some(shape) = load("box") else { return };
    let bytes = write_bin(&shape);
    let truncated = &bytes[..bytes.len() / 2];
    assert!(matches!(
        Shape::read_bin(truncated),
        Err(KernelError::Format(_))
    ));
    let garbage = [b"AMGB".as_slice(), &[amg_brep::FORMAT_VERSION], &[7u8; 64]].concat();
    assert!(matches!(
        Shape::read_bin(garbage.as_slice()),
        Err(KernelError::Format(_))
    ));
    assert!(matches!(
        Shape::read_text(&b"not a brep file"[..]),
        Err(KernelError::Format(_))
    ));
}

#[test]
fn estimated_bytes_grows_with_the_part() {
    let small = load("box");
    let large = load("plate_with_holes");
    let (Some(small), Some(large)) = (small, large) else {
        return;
    };
    assert!(small.estimated_bytes() > 0);
    assert!(large.estimated_bytes() > 50 * small.estimated_bytes());
}
