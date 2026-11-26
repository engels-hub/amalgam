//! INS-07: measure on construction, origin and sketch geometry, on B-rep solids and between
//! meshes and solids.
//!
//! Reference geometry runs everywhere; the B-rep tests need the `occt` feature (AMG_OCCT_DIR).

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::print_stderr,
    missing_docs,
    reason = "test code, exact values"
)]

use std::sync::{Arc, OnceLock};

use amg_core::{Dir3, LengthUnit, Line3, Plane, Point3, Segment3, Transform, Vec3};
use amg_doc::{Command, ConstructionId, CreatedId, Document, EntityRef, Expr, ExprSrc, FeatureBox};
use amg_features::construct::resolve::resolve_in;
use amg_features::construct::{
    ConstructPlane, OriginEntity, PlaneDef, RefGeom, RefInput, ResolvedRef,
};
use amg_features::inspect::InspectError;
use amg_features::inspect::measure::{
    APPROXIMATE, AngleKind, Cost, DisplayPrefs, Exactness, MeasureOpts, MeasureReport, MeshElement,
    MeshOperand, Operand, OperandKind, WitnessFeature, estimated_cost, measure, measure_one,
    min_distance, report_rows,
};
use amg_jobs::{JobCtx, JobSystem, JobSystemConfig};
use amg_mesh::{FaceId, IndexedMesh, MeshBody};

fn ctx() -> JobCtx {
    JobCtx::detached()
}

fn opts() -> MeasureOpts {
    MeasureOpts::default()
}

fn jobs() -> &'static JobSystem {
    static JOBS: OnceLock<JobSystem> = OnceLock::new();
    JOBS.get_or_init(|| JobSystem::new(JobSystemConfig::default()).unwrap())
}

fn geom(g: RefGeom) -> Operand {
    Operand::Ref(ResolvedRef::new(g))
}

fn both(a: &Operand, b: &Operand) -> MeasureReport {
    measure(a, Some(b), &opts(), &ctx()).unwrap()
}

/// The minimum distance, checked symmetric, with witnesses that span it.
#[track_caller]
fn distance(a: &Operand, b: &Operand) -> f64 {
    let d = min_distance(a, b, &opts(), &ctx()).unwrap();
    let back = min_distance(b, a, &opts(), &ctx()).unwrap();
    let mm = d.distance.to_mm();
    assert!((back.distance.to_mm() - mm).abs() < 1e-9, "symmetric");
    assert!(
        (d.delta().length() - mm).abs() < 1e-7,
        "witnesses {d:?} span the distance"
    );
    mm
}

/// A closed cube mesh `[min, min + s]³`.
fn cube(min: Point3, s: f64) -> MeshBody {
    let v = (0..8)
        .map(|i| {
            min + Vec3::new(
                f64::from(i & 1),
                f64::from((i >> 1) & 1),
                f64::from((i >> 2) & 1),
            ) * s
        })
        .collect();
    let t = vec![
        [0, 2, 3],
        [0, 3, 1],
        [4, 5, 7],
        [4, 7, 6],
        [0, 1, 5],
        [0, 5, 4],
        [2, 6, 7],
        [2, 7, 3],
        [0, 4, 6],
        [0, 6, 2],
        [1, 3, 7],
        [1, 7, 5],
    ];
    MeshBody::from_arc(Arc::new(IndexedMesh::new(v, t).unwrap()))
}

fn mesh_op(body: &MeshBody, element: MeshElement) -> Operand {
    Operand::Mesh(MeshOperand {
        body: body.clone(),
        to_world: Transform::IDENTITY,
        element,
    })
}

/// A construction plane `distance` above the origin XY plane, built as a feature and resolved
/// for a tool.
fn offset_plane(distance: &str) -> Operand {
    let mut doc = Document::new("ins07");
    let root = doc.design().root();
    let out = doc
        .apply(Command::AddFeature {
            at: None,
            feature: FeatureBox::new(ConstructPlane {
                def: PlaneDef::Offset {
                    base: RefInput::Origin(OriginEntity::XYPlane),
                    distance: Expr::Inline(ExprSrc(distance.to_owned())),
                },
            }),
            context: root,
            name: None,
        })
        .unwrap();
    let id = out
        .created
        .iter()
        .find_map(|c| match c {
            CreatedId::Feature(f) => Some(*f),
            _ => None,
        })
        .unwrap();
    doc.recompute_blocking(jobs()).unwrap();
    let r = RefInput::Ref(EntityRef::Construction(ConstructionId(id)));
    Operand::Ref(resolve_in(doc.evaluated(), doc.design(), &r).unwrap())
}

#[test]
fn construction_and_origin_geometry_measure_exactly() {
    let plane = offset_plane("12.5 mm");
    // A construction plane and a vertex: exact.
    let m = cube(Point3::new(1.0, 2.0, 3.0), 4.0);
    let vertex = mesh_op(&m, MeshElement::Vertex(7));
    let r = both(&plane, &vertex);
    let pair = r.pair.as_ref().unwrap();
    assert_eq!(pair.min_distance.distance.to_mm(), 5.5);
    assert_eq!(pair.min_distance.exactness, Exactness::Exact);
    assert_eq!(pair.min_distance.b.feature, WitnessFeature::Vertex(7));
    assert_eq!(pair.min_distance.a.point, Point3::new(5.0, 6.0, 12.5));
    assert_eq!(r.first.kind, OperandKind::Construction);
    assert_eq!(r.first.plane.unwrap().plane.normal, Dir3::Z);
    assert_eq!(
        distance(&plane, &Operand::Point(Point3::new(-7.0, 3.0, -0.5))),
        13.0
    );
    // The whole cube, one face, one edge: exact against the plane.
    assert_eq!(distance(&plane, &mesh_op(&m, MeshElement::Body)), 5.5);
    let top = MeshElement::Faces(vec![FaceId(2), FaceId(3)].into());
    assert_eq!(distance(&mesh_op(&m, top), &plane), 5.5);
    let crossing = offset_plane("4 mm");
    let d = min_distance(&crossing, &mesh_op(&m, MeshElement::Body), &opts(), &ctx()).unwrap();
    assert_eq!(d.distance.to_mm(), 0.0);
    assert!(d.intersecting);
    assert!((d.b.point.z - 4.0).abs() < 1e-12, "{d:?}");

    // Origin entities.
    let doc = Document::new("origin");
    let origin =
        |e| Operand::Ref(resolve_in(doc.evaluated(), doc.design(), &RefInput::Origin(e)).unwrap());
    assert_eq!(
        distance(&origin(OriginEntity::Point), &vertex),
        Vec3::new(5.0, 6.0, 7.0).length()
    );
    assert_eq!(distance(&origin(OriginEntity::YZPlane), &vertex), 5.0);
    let x = origin(OriginEntity::XAxis);
    assert!((distance(&x, &vertex) - 6.0_f64.hypot(7.0)).abs() < 1e-12);
    // The unbounded axis against the body: its nearest edge, along y = 2, z = 3.
    assert!((distance(&x, &mesh_op(&m, MeshElement::Body)) - 2.0_f64.hypot(3.0)).abs() < 1e-12);
    // A plane and an axis: angle and parallel distance.
    let r = both(&plane, &x);
    assert_eq!(r.pair.as_ref().unwrap().min_distance.distance.to_mm(), 12.5);
    let angle = r.pair.unwrap().angle.unwrap();
    assert_eq!(angle.kind, AngleKind::LinePlane);
    assert!(angle.angle.to_rad().abs() < 1e-15);
}

#[test]
fn sketch_entities_report_length_radius_and_distance() {
    let on_sketch = |g| {
        Operand::Ref(ResolvedRef {
            geom: g,
            context_plane: Some(Plane::XY),
            quality: None,
        })
    };
    let circle = on_sketch(RefGeom::Circle {
        center: Point3::new(30.0, 0.0, 0.0),
        normal: Dir3::Z,
        radius: 6.25,
        arc: None,
    });
    let e = measure_one(&circle, &opts(), &ctx()).unwrap();
    assert_eq!(e.kind, OperandKind::Sketch);
    assert_eq!(e.radius.unwrap().to_mm(), 6.25);
    assert_eq!(e.circle.unwrap().circle.radius, 6.25);
    let rows = report_rows(
        &measure(&circle, None, &opts(), &ctx()).unwrap(),
        &DisplayPrefs::default(),
    );
    let text = |key: &str| rows.iter().find(|r| r.key == key).unwrap().text.clone();
    assert_eq!(text("first.diameter"), "12.5 mm");
    let line = on_sketch(RefGeom::Segment(Segment3 {
        a: Point3::new(0.0, -3.0, 0.0),
        b: Point3::new(0.0, 9.0, 0.0),
    }));
    let e = measure_one(&line, &opts(), &ctx()).unwrap();
    assert_eq!(e.length.unwrap().to_mm(), 12.0);
    // Circle to line has no closed form: polylines, marked, within their deflection.
    let r = both(&circle, &line);
    let d = r.pair.as_ref().unwrap().min_distance;
    let Exactness::Tessellated { deflection } = d.exactness else {
        panic!("{d:?}");
    };
    assert!(
        (d.distance.to_mm() - 23.75).abs() <= deflection,
        "{} ± {deflection}",
        d.distance.to_mm()
    );
    let rows = report_rows(&r, &DisplayPrefs::default());
    assert!(rows[0].text.starts_with(APPROXIMATE), "{}", rows[0].text);
    // Circle to a plane and to a point are closed forms.
    let wall = geom(RefGeom::Plane(Plane::YZ));
    let d = min_distance(&circle, &wall, &opts(), &ctx()).unwrap();
    assert_eq!(d.exactness, Exactness::Exact);
    assert!((d.distance.to_mm() - 23.75).abs() < 1e-12);
    // Two circles, concentric radii 6.25 and 2.
    let inner = on_sketch(RefGeom::Circle {
        center: Point3::new(30.0, 0.0, 0.0),
        normal: Dir3::Z,
        radius: 2.0,
        arc: None,
    });
    let d = min_distance(&circle, &inner, &opts(), &ctx()).unwrap();
    assert!((d.distance.to_mm() - 4.25).abs() < 0.01, "{d:?}");
    // A circle against a mesh.
    let m = cube(Point3::new(0.0, 0.0, 0.0), 4.0);
    let d = min_distance(&mesh_op(&m, MeshElement::Body), &circle, &opts(), &ctx()).unwrap();
    assert!((d.distance.to_mm() - 19.75).abs() < 0.01, "{d:?}");
}

#[test]
fn inline_costs_and_unsupported_pairs() {
    let p = geom(RefGeom::Point(Point3::ONE));
    let plane = geom(RefGeom::Plane(Plane::XY));
    let line = geom(RefGeom::Line(Line3 {
        point: Point3::ZERO,
        dir: Dir3::X,
    }));
    assert_eq!(estimated_cost(&p, Some(&plane)), Cost::Inline);
    assert_eq!(estimated_cost(&line, Some(&plane)), Cost::Inline);
    let circle = geom(RefGeom::Circle {
        center: Point3::ZERO,
        normal: Dir3::Z,
        radius: 1.0,
        arc: None,
    });
    assert_eq!(estimated_cost(&circle, Some(&plane)), Cost::Job);
    let m = cube(Point3::ZERO, 1.0);
    assert_eq!(
        estimated_cost(&mesh_op(&m, MeshElement::Body), Some(&p)),
        Cost::Job
    );
    let sphere = geom(RefGeom::Sphere {
        center: Point3::new(0.0, 0.0, 25.0),
        radius: 10.0,
    });
    assert_eq!(distance(&sphere, &Operand::Point(Point3::ZERO)), 15.0);
    assert!(matches!(
        min_distance(&sphere, &plane, &opts(), &ctx()),
        Err(InspectError::Unsupported { .. })
    ));
    // A scaled mesh stays an error next to reference geometry.
    let scaled = Operand::Mesh(MeshOperand {
        body: m,
        to_world: Transform::from_scale(Vec3::splat(2.0)),
        element: MeshElement::Body,
    });
    assert!(matches!(
        min_distance(&scaled, &plane, &opts(), &ctx()),
        Err(InspectError::NonRigidTransform)
    ));
    // Rows use the display unit for reference geometry too.
    let r = both(&p, &plane);
    let prefs = DisplayPrefs {
        length: LengthUnit::Inch,
        precision: 4,
        ..DisplayPrefs::default()
    };
    assert_eq!(report_rows(&r, &prefs)[0].text, "0.0394 in");
}

#[cfg(feature = "occt")]
mod solid {
    use std::f64::consts::{PI, TAU};

    use amg_brep::geom::{Curve3, Surface};
    use amg_brep::{BrepKernel, KernelConfig, OcctKernel, Shape, SubKind, SubShape};
    use amg_features::inspect::measure::{BrepOperand, EntityMeasure};

    use super::*;

    fn kernel() -> &'static OcctKernel {
        static KERNEL: OnceLock<OcctKernel> = OnceLock::new();
        KERNEL.get_or_init(|| OcctKernel::new(KernelConfig::default()))
    }

    /// A frame tilted off every axis.
    fn tilt() -> Transform {
        Transform::from_translation(Vec3::new(3.0, -7.0, 11.0))
            * Transform::from_axis_angle(Vec3::new(1.0, 2.0, 3.0).normalize(), 0.7)
    }

    fn make_box(size: Vec3) -> Shape {
        kernel().make_box(&Plane::XY, size).unwrap().shape
    }

    fn op(shape: &Shape, sub: Option<(SubKind, u32)>, to_world: Transform) -> Operand {
        Operand::Brep(BrepOperand {
            shape: shape.clone(),
            sub: sub.map(|(kind, index)| SubShape { kind, index }),
            to_world,
        })
    }

    fn one(o: &Operand) -> EntityMeasure {
        measure_one(o, &opts(), &ctx()).unwrap()
    }

    /// The planar face whose outward normal is `n` in the shape's own space.
    fn face(shape: &Shape, n: Vec3) -> u32 {
        shape
            .faces()
            .find(|&f| match shape.surface(f) {
                Surface::Plane(p) => {
                    let sign = if shape.face_reversed(f) { -1.0 } else { 1.0 };
                    (p.normal.get() * sign - n).length() < 1e-9
                }
                _ => false,
            })
            .unwrap()
            .0
    }

    #[test]
    fn box_edges_faces_and_parallel_distance() {
        let size = Vec3::new(10.0, 20.0, 30.0);
        let b = make_box(size);
        // Largest errors: edge length, face area, parallel-face distance.
        let mut worst = [0.0f64; 3];
        for t in [Transform::IDENTITY, tilt()] {
            let mut lengths: Vec<f64> = b
                .edges()
                .map(|e| {
                    let m = one(&op(&b, Some((SubKind::Edge, e.0)), t));
                    assert_eq!(m.kind, OperandKind::BrepEdge);
                    assert_eq!(m.exactness, Exactness::Exact);
                    assert!(m.direction.is_some());
                    m.length.unwrap().to_mm()
                })
                .collect();
            lengths.sort_by(f64::total_cmp);
            let expected = [10.0, 20.0, 30.0];
            for (i, l) in lengths.iter().enumerate() {
                worst[0] = worst[0].max((l - expected[i / 4]).abs());
                assert!((l - expected[i / 4]).abs() < 1e-9, "edge {l}");
            }
            for (n, area, gap) in [
                (Vec3::X, 600.0, 10.0),
                (Vec3::Y, 300.0, 20.0),
                (Vec3::Z, 200.0, 30.0),
            ] {
                let a = op(&b, Some((SubKind::Face, face(&b, n))), t);
                let z = op(&b, Some((SubKind::Face, face(&b, -n))), t);
                let m = one(&a);
                worst[1] = worst[1].max((m.area.unwrap().get() - area).abs());
                assert!((m.area.unwrap().get() - area).abs() < 1e-9, "{m:?}");
                let normal = m.plane.unwrap().plane.normal.get();
                assert!((normal - t.transform_vector3(n)).length() < 1e-12);
                let r = both(&a, &z);
                let pair = r.pair.unwrap();
                worst[2] = worst[2].max((pair.min_distance.distance.to_mm() - gap).abs());
                assert!(
                    (pair.min_distance.distance.to_mm() - gap).abs() < 1e-9,
                    "faces {n}: {}",
                    pair.min_distance.distance.to_mm()
                );
                assert_eq!(pair.min_distance.exactness, Exactness::Exact);
                assert_eq!(pair.min_distance.a.feature, WitnessFeature::Brep);
                assert!((pair.angle.unwrap().angle.to_rad() - PI).abs() < 1e-12);
                assert!((pair.center_distance.to_mm() - gap).abs() < 1e-9);
            }
            let body = one(&op(&b, None, t));
            assert_eq!(body.kind, OperandKind::BrepBody);
            assert!((body.volume.unwrap().get() - 6000.0).abs() < 1e-9);
            assert!((body.area.unwrap().get() - 2200.0).abs() < 1e-9);
            assert_eq!(body.face_count, Some(6));
            let centre = t.transform_point3(size * 0.5);
            assert!((body.centroid.unwrap() - centre).length() < 1e-9);
        }
        eprintln!("box worst errors (edge, area, parallel faces): {worst:?}");
    }

    #[test]
    fn cylinder_radius_and_circle_diameter_are_exact() {
        let radius = 6.3;
        let c = kernel()
            .make_cylinder(&Plane::XY, radius, 12.0, TAU)
            .unwrap()
            .shape;
        for t in [Transform::IDENTITY, tilt()] {
            let side = c
                .faces()
                .find(|&f| matches!(c.surface(f), Surface::Cylinder { .. }))
                .unwrap();
            let m = one(&op(&c, Some((SubKind::Face, side.0)), t));
            assert_eq!(m.radius.unwrap().to_mm(), radius, "bit-exact");
            assert!((m.area.unwrap().get() - TAU * radius * 12.0).abs() < 1e-9);
            let circles: Vec<u32> = c
                .edges()
                .filter(|&e| matches!(c.curve(e), Some((Curve3::Circle { .. }, ..))))
                .map(|e| e.0)
                .collect();
            assert_eq!(circles.len(), 2);
            for e in &circles {
                let o = op(&c, Some((SubKind::Edge, *e)), t);
                let m = one(&o);
                assert_eq!(m.circle.unwrap().circle.radius, radius, "bit-exact");
                assert!((m.length.unwrap().to_mm() - TAU * radius).abs() < 1e-9);
                let rows = report_rows(
                    &measure(&o, None, &opts(), &ctx()).unwrap(),
                    &DisplayPrefs::default(),
                );
                let d = rows.iter().find(|r| r.key == "first.diameter").unwrap();
                assert_eq!(d.value.value, 2.0 * radius);
                assert_eq!(d.text, "12.6 mm");
            }
            // The centres of the two rims are 12 apart.
            let rim = |e: u32| op(&c, Some((SubKind::Edge, e)), t);
            let pair = both(&rim(circles[0]), &rim(circles[1])).pair.unwrap();
            assert!((pair.center_distance.to_mm() - 12.0).abs() < 1e-9);
        }
    }

    #[test]
    fn planar_faces_at_30_degrees() {
        let a = make_box(Vec3::new(10.0, 10.0, 10.0));
        let b = make_box(Vec3::new(4.0, 5.0, 6.0));
        let angle = 30f64.to_radians();
        for t in [Transform::IDENTITY, tilt()] {
            let turned = t
                * Transform::from_translation(Vec3::new(0.0, 0.0, 40.0))
                * Transform::from_rotation_x(angle);
            let first = op(&a, Some((SubKind::Face, face(&a, Vec3::Z))), t);
            let second = op(&b, Some((SubKind::Face, face(&b, Vec3::Z))), turned);
            let got = both(&first, &second).pair.unwrap().angle.unwrap();
            assert_eq!(got.kind, AngleKind::PlanePlane);
            eprintln!("30 deg error: {:.2e} rad", got.angle.to_rad() - angle);
            assert!(
                (got.angle.to_rad() - angle).abs() <= 1e-12,
                "{}",
                got.angle.to_rad() - angle
            );
            assert!((got.supplement.to_rad() - (PI - angle)).abs() <= 1e-12);
        }
    }

    #[test]
    fn sphere_to_plane_is_15() {
        let centre = Point3::new(4.0, -3.0, 25.0);
        let frame = Plane {
            origin: centre,
            ..Plane::XY
        };
        let sphere = kernel().make_sphere(&frame, 10.0).unwrap().shape;
        let slab = make_box(Vec3::new(2.0, 2.0, 1.0));
        let under = Transform::from_translation(Vec3::new(50.0, 50.0, -1.0));
        // A construction plane, a planar face of another solid beside the sphere, and the
        // sphere's face against the body.
        let plane = offset_plane("0 mm");
        for sub in [None, Some((SubKind::Face, 0))] {
            let s = op(&sphere, sub, Transform::IDENTITY);
            let d = distance(&s, &plane);
            eprintln!(
                "sphere ({sub:?}) to construction plane: error {:.2e}",
                d - 15.0
            );
            assert!((d - 15.0).abs() <= 1e-7, "{d}");
            let found = min_distance(&s, &plane, &opts(), &ctx()).unwrap();
            assert_eq!(found.exactness, Exactness::Exact);
            assert!((found.a.point - Point3::new(4.0, -3.0, 15.0)).length() < 1e-6);
            assert!((found.b.point - Point3::new(4.0, -3.0, 0.0)).length() < 1e-6);
        }
        let wide = make_box(Vec3::new(100.0, 100.0, 1.0));
        let top = op(
            &wide,
            Some((SubKind::Face, face(&wide, Vec3::Z))),
            Transform::from_translation(Vec3::new(-50.0, -50.0, -1.0)),
        );
        let d = distance(&op(&sphere, None, Transform::IDENTITY), &top);
        eprintln!("sphere to planar face: error {:.2e}", d - 15.0);
        assert!((d - 15.0).abs() <= 1e-7, "{d}");
        assert_eq!(
            one(&op(&sphere, Some((SubKind::Face, 0)), Transform::IDENTITY))
                .radius
                .unwrap()
                .to_mm(),
            10.0
        );
        // A plane far from the shape's projection is still unbounded.
        let mut far = Plane::XY;
        far.origin = Point3::new(1.0e4, -2.0e4, 0.0);
        let d = distance(&geom(RefGeom::Plane(far)), &op(&slab, None, under));
        assert_eq!(d, 0.0);
    }

    #[test]
    fn construction_geometry_against_solids() {
        let b = make_box(Vec3::new(10.0, 20.0, 30.0));
        let body = op(&b, None, Transform::IDENTITY);
        // A construction plane and a B-rep vertex: exact.
        let plane = offset_plane("37.25 mm");
        for v in b.vertices() {
            let o = op(&b, Some((SubKind::Vertex, v.0)), Transform::IDENTITY);
            let z = one(&o).position.unwrap().z;
            let d = min_distance(&plane, &o, &opts(), &ctx()).unwrap();
            assert_eq!(d.distance.to_mm(), 37.25 - z, "exact");
            assert_eq!(d.exactness, Exactness::Exact);
            assert_eq!(estimated_cost(&plane, Some(&o)), Cost::Inline);
        }
        assert!((distance(&plane, &body) - 7.25).abs() < 1e-9);
        assert_eq!(distance(&offset_plane("15 mm"), &body), 0.0);
        let top = op(
            &b,
            Some((SubKind::Face, face(&b, Vec3::Z))),
            Transform::IDENTITY,
        );
        assert!((distance(&plane, &top) - 7.25).abs() < 1e-9);
        assert_eq!(
            both(&plane, &top)
                .pair
                .unwrap()
                .angle
                .unwrap()
                .angle
                .to_rad(),
            0.0
        );
        // Axis, sketch segment, circle and point.
        let axis = geom(RefGeom::Line(Line3 {
            point: Point3::new(13.0, 24.0, -500.0),
            dir: Dir3::Z,
        }));
        assert!((distance(&axis, &body) - 5.0).abs() < 1e-9);
        let seg = geom(RefGeom::Segment(Segment3 {
            a: Point3::new(13.0, 5.0, 5.0),
            b: Point3::new(20.0, 5.0, 5.0),
        }));
        assert!((distance(&body, &seg) - 3.0).abs() < 1e-9);
        let circle = geom(RefGeom::Circle {
            center: Point3::new(-10.0, 5.0, 5.0),
            normal: Dir3::Z,
            radius: 2.0,
            arc: None,
        });
        let d = min_distance(&circle, &body, &opts(), &ctx()).unwrap();
        assert!((d.distance.to_mm() - 8.0).abs() < 1e-9, "{d:?}");
        assert_eq!(d.exactness, Exactness::Exact);
        // A point inside the solid is measured to its boundary.
        let inside = Operand::Point(Point3::new(5.0, 4.0, 15.0));
        assert!((distance(&inside, &body) - 4.0).abs() < 1e-9);
        // Two solids: exact, and 0 when one holds the other.
        let other = op(
            &b,
            None,
            Transform::from_translation(Vec3::new(13.0, 24.0, 0.0)),
        );
        assert!((distance(&body, &other) - 5.0).abs() < 1e-9);
        let small = make_box(Vec3::ONE);
        let nested = op(&small, None, Transform::from_translation(Vec3::splat(4.0)));
        let d = min_distance(&body, &nested, &opts(), &ctx()).unwrap();
        assert_eq!((d.distance.to_mm(), d.contained), (0.0, None));
        assert!(d.intersecting);
    }

    #[test]
    fn bad_operands_are_errors() {
        let b = make_box(Vec3::ONE);
        let bad = op(&b, Some((SubKind::Face, 6)), Transform::IDENTITY);
        assert!(matches!(
            measure_one(&bad, &opts(), &ctx()),
            Err(InspectError::BadElement(_))
        ));
        let bad = op(&b, Some((SubKind::Vertex, 99)), Transform::IDENTITY);
        assert!(matches!(
            min_distance(&bad, &offset_plane("1 mm"), &opts(), &ctx()),
            Err(InspectError::BadElement(_))
        ));
        let mirrored = op(&b, None, Transform::from_scale(Vec3::new(-1.0, 1.0, 1.0)));
        assert!(matches!(
            measure_one(&mirrored, &opts(), &ctx()),
            Err(InspectError::NonRigidTransform)
        ));
    }

    #[test]
    fn mesh_to_solid_is_approximate_within_the_deflection() {
        let radius = 10.0;
        let sphere = kernel().make_sphere(&Plane::XY, radius).unwrap().shape;
        let body = op(&sphere, None, Transform::IDENTITY);
        let face = op(&sphere, Some((SubKind::Face, 0)), Transform::IDENTITY);
        // Cubes whose nearest corner lies in directions that miss the sphere's vertices, and
        // a face-on cube.
        for corner in [
            Point3::new(12.0, 13.0, 14.0),
            Point3::new(9.3, 0.7, 8.1),
            Point3::new(15.0, -1.0, -1.0),
        ] {
            let mesh = cube(corner, 2.0);
            // The point of the cube nearest to the sphere centre.
            let nearest = Point3::ZERO.clamp(corner, corner + Vec3::splat(2.0));
            let exact = nearest.length() - radius;
            for solid in [&body, &face] {
                for element in [MeshElement::Body, MeshElement::Vertex(0)] {
                    let vertex = element == MeshElement::Vertex(0);
                    let mine = mesh_op(&mesh, element);
                    let r = both(&mine, solid);
                    let found = r.pair.as_ref().unwrap().min_distance;
                    let d = found.distance.to_mm();
                    if vertex {
                        // A point against a solid is exact.
                        assert_eq!(found.exactness, Exactness::Exact);
                        assert!((d - (corner.length() - radius)).abs() < 1e-7, "{d}");
                        continue;
                    }
                    let Exactness::Tessellated { deflection } = found.exactness else {
                        panic!("{found:?}");
                    };
                    eprintln!(
                        "corner {corner}: {d:.7} vs {exact:.7}, error {:.2e}, deflection \
                         {deflection:.2e}",
                        d - exact
                    );
                    assert!(deflection > 0.0 && deflection <= 0.01);
                    assert!(
                        (d - exact).abs() <= deflection,
                        "{d} vs {exact} ± {deflection}"
                    );
                    assert_eq!(found.b.feature, WitnessFeature::Brep);
                    let rows = report_rows(&r, &DisplayPrefs::default());
                    let distance = rows.iter().find(|r| r.key == "distance").unwrap();
                    assert!(distance.text.starts_with("≈ "), "{}", distance.text);
                    // Single-entity values stay exact.
                    let volume = rows.iter().find(|r| r.key == "first.volume").unwrap();
                    assert!(!volume.text.contains('≈'));
                    // Swapped operands give the same pair.
                    let back = min_distance(solid, &mine, &opts(), &ctx()).unwrap();
                    assert_eq!(back.distance.to_mm(), d);
                    assert_eq!(back.a, found.b);
                }
            }
        }
        // A mesh inside the solid: the surface distance, and `contained`.
        let inner = cube(Point3::splat(-1.0), 2.0);
        let d = min_distance(&mesh_op(&inner, MeshElement::Body), &body, &opts(), &ctx()).unwrap();
        assert_eq!(d.contained, Some(true), "{d:?}");
        assert!((d.distance.to_mm() - (radius - 3f64.sqrt())).abs() < 0.01);
        // Against an edge of a solid.
        let b = make_box(Vec3::new(10.0, 10.0, 10.0));
        let edge = b.edges().find(|&e| {
            let [u, v] = b.edge_vertices(e);
            let (p, q) = (b.vertex_point(u), b.vertex_point(v));
            p.x == 10.0 && q.x == 10.0 && p.y == 10.0 && q.y == 10.0
        });
        let edge = op(
            &b,
            Some((SubKind::Edge, edge.unwrap().0)),
            Transform::IDENTITY,
        );
        let mesh = cube(Point3::new(13.0, 14.0, 2.0), 2.0);
        let d = min_distance(&mesh_op(&mesh, MeshElement::Body), &edge, &opts(), &ctx()).unwrap();
        assert!((d.distance.to_mm() - 5.0).abs() < 1e-9, "{d:?}");
        assert!(matches!(d.exactness, Exactness::Tessellated { .. }));
    }
}
