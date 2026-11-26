//! INS-07: the Measure tool on construction, origin and sketch geometry and on solid bodies,
//! in the real app (`AppHarness`, kittest, CPU scene backend and CPU picking).
//!
//! Construction, origin, sketch and mesh picks run everywhere; solid picks need the `occt`
//! feature (AMG_OCCT_DIR).

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "test code, exact values"
)]

use std::sync::Arc;

use amg_core::{Point2, Point3, Vec3};
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, Command, Dependency, Document, Expr, Feature, FeatureBox,
    FeatureError, FeatureOutput, RecomputeCtx,
};
use amg_features::construct::{ConstructPlane, OriginEntity, PlaneDef, RefInput};
use amg_features::inspect::measure::{MeasureOpts, OperandKind, measure};
use amg_features::sketch::{PlaneInput, SketchFeature};
use amg_jobs::JobCtx;
use amg_mesh::IndexedMesh;
use amg_render::{Camera, Projection};
use amg_ui::CommitBatch;
use amg_ui::inspect::{MEASURE_CMD, MeasureSnapshot, measure_snapshot};
use amg_ui::testing::AppHarness;
use amg_ui::testing::egui_kittest::kittest::Queryable as _;
use egui::accesskit::Role;
use serde::{Deserialize, Serialize};

// ---- fixtures -----------------------------------------------------------------------------

/// A closed 4 mm cube mesh with its low corner at (30, 2, 3).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Cube;

impl Feature for Cube {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let v = (0..8)
            .map(|i| {
                Point3::new(30.0, 2.0, 3.0)
                    + Vec3::new(
                        f64::from(i & 1),
                        f64::from((i >> 1) & 1),
                        f64::from((i >> 2) & 1),
                    ) * 4.0
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
        let mesh = IndexedMesh::new(v, t).unwrap();
        Ok(output(ctx, BodyGeom::Mesh(Arc::new(mesh))))
    }
}

amg_doc::register_feature!(Cube, "uitest.ins07_cube", version = 1);

fn output(ctx: &mut RecomputeCtx<'_>, geom: BodyGeom) -> FeatureOutput {
    FeatureOutput {
        bodies: vec![BodyChange {
            id: ctx.new_body_id(0),
            change: BodyChangeKind::Created { geom, naming: None },
        }],
        ..FeatureOutput::default()
    }
}

fn plane(base: OriginEntity, distance: &str) -> FeatureBox {
    FeatureBox::new(ConstructPlane {
        def: PlaneDef::Offset {
            base: RefInput::Origin(base),
            distance: Expr::Inline(amg_doc::ExprSrc(distance.to_owned())),
        },
    })
}

/// The app on an empty document.
fn harness() -> AppHarness {
    amg_features::construct::link();
    let mut h = AppHarness::builder()
        .size(egui::vec2(1600.0, 1000.0))
        .build();
    let id = h
        .app_mut()
        .add_document("ins07".into(), Document::new("ins07"));
    h.app_mut().activate(id);
    h.drive_until_idle();
    h
}

fn add(h: &mut AppHarness, feature: FeatureBox) {
    let root = h.document().design().root();
    h.app_mut().submit(CommitBatch {
        label: "Add".into(),
        commands: vec![Command::AddFeature {
            at: None,
            feature,
            context: root,
            name: None,
        }],
    });
    h.drive_until_idle();
}

/// Looks down the Z axis at (x, y) from `height`.
fn look_down(h: &mut AppHarness, x: f64, y: f64, height: f64) {
    h.app_mut().active_document_mut().camera = Camera {
        eye: Point3::new(x, y, height),
        target: Point3::new(x, y, 0.0),
        up: amg_core::Dir3::Y,
        projection: Projection::Perspective { fov_y: 0.8 },
    };
    h.drive_until_idle();
}

fn start(h: &mut AppHarness) {
    h.run_command(MEASURE_CMD);
    h.kittest().step();
    assert_eq!(h.app().active_tool(), Some(MEASURE_CMD), "Measure started");
    h.drive_until_idle();
}

fn snap(h: &mut AppHarness) -> MeasureSnapshot {
    h.kittest().step();
    measure_snapshot(&h.kittest().ctx).expect("the Measure dialog was drawn")
}

fn result(h: &mut AppHarness) -> MeasureSnapshot {
    h.drive_until_idle();
    let s = snap(h);
    assert!(s.running.is_none(), "no job left: {s:?}");
    assert!(s.error.is_none(), "no error: {:?}", s.error);
    // The result is what the API gives for the same operands.
    if !s.three_point
        && let Some(report) = &s.report
    {
        let api = measure(
            &s.operands[0],
            s.operands.get(1),
            &MeasureOpts::default(),
            &JobCtx::detached(),
        )
        .unwrap();
        assert_eq!(report, &api);
    }
    s
}

fn click(h: &mut AppHarness, p: Point3) {
    let pos = h.world_to_screen(p);
    h.click_at(pos, egui::PointerButton::Primary, egui::Modifiers::NONE);
    h.drive_until_idle();
}

fn button(h: &mut AppHarness, role: Role, label: &str) {
    h.kittest().step();
    h.kittest().get_by_role_and_label(role, label).click();
    h.kittest().step();
    h.drive_until_idle();
}

fn origin(h: &mut AppHarness, label: &str) {
    h.kittest().step();
    h.kittest()
        .get_by_role_and_label(Role::Button, "Add origin")
        .click();
    h.kittest().step();
    h.drive_until_idle();
    button(h, Role::Button, label);
}

#[track_caller]
fn text(s: &MeasureSnapshot, key: &str) -> String {
    s.rows
        .iter()
        .find(|r| r.key == key)
        .unwrap_or_else(|| panic!("row {key} in {:?}", s.rows))
        .text
        .clone()
}

fn value(s: &MeasureSnapshot, key: &str) -> f64 {
    s.rows.iter().find(|r| r.key == key).unwrap().value.value
}

// ---- construction, origin, sketch -----------------------------------------------------------

#[test]
fn construction_and_origin_geometry_measure_in_the_view() {
    let mut h = harness();
    h.run_command(MEASURE_CMD);
    h.drive_until_idle();
    assert_eq!(h.app().active_tool(), None, "nothing to measure yet");
    add(&mut h, plane(OriginEntity::XYPlane, "2.5 mm"));
    look_down(&mut h, 0.0, 0.0, 80.0);
    // A construction plane alone enables Measure.
    start(&mut h);
    click(&mut h, Point3::new(3.0, 4.0, 2.5));
    let s = result(&mut h);
    assert_eq!(s.kinds, [Some(OperandKind::Construction)]);
    origin(&mut h, "XY plane");
    let s = result(&mut h);
    assert_eq!(s.kinds, [Some(OperandKind::Construction); 2]);
    assert_eq!(value(&s, "distance"), 2.5);
    assert_eq!(text(&s, "distance"), "2.5 mm");
    assert_eq!(value(&s, "angle"), 0.0);

    // A mesh vertex against the plane: exact.
    add(&mut h, FeatureBox::new(Cube));
    look_down(&mut h, 20.0, 4.0, 80.0);
    button(&mut h, Role::Button, "Restart");
    click(&mut h, Point3::new(30.02, 2.02, 7.0));
    // The plane is square and centred on the body now: click it beside the cube.
    let s = result(&mut h);
    assert_eq!(s.kinds, [Some(OperandKind::Vertex)], "{s:?}");
    click(&mut h, Point3::new(24.0, 4.0, 2.5));
    let s = result(&mut h);
    assert_eq!(
        s.kinds,
        [Some(OperandKind::Vertex), Some(OperandKind::Construction)]
    );
    assert_eq!(value(&s, "distance"), 4.5);
    assert!(s.inline, "a point against a plane is measured inline");

    // The Construction check box turns the picks off.
    button(&mut h, Role::Button, "Restart");
    button(&mut h, Role::CheckBox, "Construction");
    assert!(!snap(&mut h).construction);
    click(&mut h, Point3::new(24.0, 4.0, 2.5));
    assert!(result(&mut h).kinds.is_empty());
}

#[test]
fn sketch_curves_and_points_are_picked_on_their_plane() {
    let mut h = harness();
    let mut f = SketchFeature::new(PlaneInput::OriginXY);
    f.edit_sketch(|s| {
        s.add_circle(Point2::new(0.0, 0.0), 6.0)?;
        s.add_line(Point2::new(10.0, -5.0), Point2::new(10.0, 7.0))
    })
    .unwrap();
    add(&mut h, FeatureBox::new(f));
    look_down(&mut h, 2.0, 0.0, 50.0);
    start(&mut h);
    // The circle's rim.
    click(&mut h, Point3::new(0.0, 6.0, 0.0));
    let s = result(&mut h);
    assert_eq!(s.kinds, [Some(OperandKind::Sketch)]);
    assert_eq!(text(&s, "first.diameter"), "12 mm");
    // The line: circle to line has no closed form.
    click(&mut h, Point3::new(10.0, 2.0, 0.0));
    let s = result(&mut h);
    assert_eq!(s.kinds, [Some(OperandKind::Sketch); 2]);
    assert_eq!(text(&s, "second.length"), "12 mm");
    assert!((value(&s, "distance") - 4.0).abs() < 0.01);
    assert!(text(&s, "distance").starts_with("≈ "));
    assert!(
        h.kittest()
            .query_all_by_label_contains("computed on a tessellation")
            .next()
            .is_some()
    );
    // The two end points of the line: a third pick starts over.
    click(&mut h, Point3::new(10.0, -5.0, 0.0));
    click(&mut h, Point3::new(10.0, 7.0, 0.0));
    let s = result(&mut h);
    assert_eq!(value(&s, "distance"), 12.0);
    assert!(!text(&s, "distance").contains('≈'));
    // Sketch off: nothing is picked.
    button(&mut h, Role::Button, "Restart");
    button(&mut h, Role::CheckBox, "Sketch");
    click(&mut h, Point3::new(0.0, 6.0, 0.0));
    assert!(result(&mut h).kinds.is_empty());
}

#[test]
fn three_point_circle_takes_sketch_points() {
    let mut h = harness();
    let mut f = SketchFeature::new(PlaneInput::OriginXY);
    f.edit_sketch(|s| {
        s.add_point(Point2::new(5.0, 0.0))?;
        s.add_point(Point2::new(0.0, 5.0))?;
        s.add_point(Point2::new(-5.0, 0.0))
    })
    .unwrap();
    add(&mut h, FeatureBox::new(f));
    look_down(&mut h, 0.0, 0.0, 40.0);
    start(&mut h);
    button(&mut h, Role::CheckBox, "3-point circle");
    for p in [[5.0, 0.0], [0.0, 5.0], [-5.0, 0.0]] {
        click(&mut h, Point3::new(p[0], p[1], 0.0));
    }
    let s = result(&mut h);
    assert!((value(&s, "first.radius") - 5.0).abs() < 1e-9);
}

// ---- solids ---------------------------------------------------------------------------------

#[cfg(feature = "occt")]
mod solid {
    use std::sync::OnceLock;

    use amg_brep::geom::Curve3;
    use amg_brep::{BrepKernel, KernelConfig, OcctKernel};
    use amg_core::Plane;
    use amg_features::inspect::measure::Exactness;
    use amg_ui::pick::PickTarget;
    use amg_ui::select::SelItem;

    use super::*;

    /// A 10 × 20 × 30 box at the origin, or a cylinder r 4, height 12 on (50, 0, 0).
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    struct Solid {
        cylinder: bool,
    }

    impl Feature for Solid {
        fn inputs(&self) -> Vec<Dependency> {
            Vec::new()
        }

        fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
            static KERNEL: OnceLock<OcctKernel> = OnceLock::new();
            let k = KERNEL.get_or_init(|| OcctKernel::new(KernelConfig::default()));
            let built = if self.cylinder {
                let frame = Plane {
                    origin: Point3::new(50.0, 0.0, 0.0),
                    ..Plane::XY
                };
                k.make_cylinder(&frame, 4.0, 12.0, std::f64::consts::TAU)
            } else {
                k.make_box(&Plane::XY, Vec3::new(10.0, 20.0, 30.0))
            };
            let shape = amg_doc::stub::Shape::from_brep(built.unwrap().shape);
            Ok(output(ctx, BodyGeom::Solid(shape)))
        }
    }

    amg_doc::register_feature!(Solid, "uitest.ins07_solid", version = 1);

    #[test]
    fn solid_faces_edges_vertices_and_construction_planes() {
        let mut h = harness();
        add(&mut h, FeatureBox::new(Solid { cylinder: false }));
        look_down(&mut h, 8.0, 10.0, 100.0);
        // A solid alone enables Measure.
        start(&mut h);
        click(&mut h, Point3::new(5.0, 10.0, 30.0));
        let s = result(&mut h);
        assert_eq!(s.kinds, [Some(OperandKind::BrepFace)], "{s:?}");
        assert_eq!(text(&s, "first.area"), "200 mm^2");
        assert!(!s.inline);

        // The owner check: a solid face against a construction plane.
        add(&mut h, plane(OriginEntity::XYPlane, "-10 mm"));
        click(&mut h, Point3::new(14.0, 10.0, -10.0));
        let s = result(&mut h);
        assert_eq!(
            s.kinds,
            [Some(OperandKind::BrepFace), Some(OperandKind::Construction)],
            "{s:?}"
        );
        assert!((value(&s, "distance") - 40.0).abs() < 1e-9);
        assert_eq!(text(&s, "distance"), "40 mm");
        assert_eq!(value(&s, "angle"), 0.0);

        // An edge (picked just inside the face) and a vertex (within 8 px).
        button(&mut h, Role::Button, "Restart");
        click(&mut h, Point3::new(5.0, 0.08, 30.0));
        let s = result(&mut h);
        assert_eq!(s.kinds, [Some(OperandKind::BrepEdge)], "{s:?}");
        assert_eq!(text(&s, "first.length"), "10 mm");
        click(&mut h, Point3::new(0.15, 19.85, 30.0));
        let s = result(&mut h);
        assert_eq!(
            s.kinds,
            [Some(OperandKind::BrepEdge), Some(OperandKind::BrepVertex)],
            "{s:?}"
        );
        assert!((value(&s, "distance") - 20.0).abs() < 1e-9);

        // Body filter: volume.
        button(&mut h, Role::Button, "Restart");
        for label in ["Face", "Edge", "Vertex", "Body"] {
            h.kittest().step();
            h.kittest()
                .get_by_role_and_label(Role::CheckBox, label)
                .click();
            h.kittest().step();
            h.drive_until_idle();
        }
        click(&mut h, Point3::new(5.0, 10.0, 30.0));
        let s = result(&mut h);
        assert_eq!(s.kinds, [Some(OperandKind::BrepBody)], "{s:?}");
        assert_eq!(text(&s, "first.volume"), "6000 mm^3");
    }

    #[test]
    fn cylinder_rim_mesh_distance_and_preselection() {
        let mut h = harness();
        add(&mut h, FeatureBox::new(Solid { cylinder: true }));
        add(&mut h, FeatureBox::new(Cube));
        look_down(&mut h, 42.0, 2.0, 70.0);
        start(&mut h);
        // The top rim, from just inside the cap.
        click(&mut h, Point3::new(50.0, 3.95, 12.0));
        let s = result(&mut h);
        assert_eq!(s.kinds, [Some(OperandKind::BrepEdge)], "{s:?}");
        assert_eq!(value(&s, "first.diameter"), 8.0);
        assert_eq!(text(&s, "first.diameter"), "8 mm");
        let rim = match &s.operands[0] {
            amg_features::inspect::measure::Operand::Brep(b) => b.clone(),
            other => panic!("{other:?}"),
        };
        let edge = amg_brep::EdgeId(rim.sub.unwrap().index);
        assert!(matches!(
            rim.shape.curve(edge),
            Some((Curve3::Circle { .. }, ..))
        ));
        // The cylinder's cap and a face of the mesh cube: approximate, within the deflection.
        button(&mut h, Role::Button, "Restart");
        click(&mut h, Point3::new(50.0, 0.0, 12.0));
        click(&mut h, Point3::new(32.8, 3.0, 7.0));
        let s = result(&mut h);
        assert_eq!(
            s.kinds,
            [Some(OperandKind::BrepFace), Some(OperandKind::Faces)],
            "{s:?}"
        );
        let found = s
            .report
            .as_ref()
            .unwrap()
            .pair
            .as_ref()
            .unwrap()
            .min_distance;
        let Exactness::Tessellated { deflection } = found.exactness else {
            panic!("{found:?}");
        };
        // Cap z = 12, radius 4 about (50, 0); cube top z = 7, x <= 34, y in [2, 6]: the
        // closest points are the cube corner and the rim towards it.
        let exact_corner = {
            let corner = Point3::new(34.0, 2.0, 7.0);
            let dir = (corner - Point3::new(50.0, 0.0, 7.0)).normalize();
            let rim = Point3::new(50.0, 0.0, 12.0) + dir * 4.0;
            rim.distance(corner)
        };
        assert!(
            (value(&s, "distance") - exact_corner).abs() <= deflection,
            "{} vs {exact_corner} ± {deflection}",
            value(&s, "distance")
        );
        assert!(text(&s, "distance").starts_with("≈ "));
        assert!(!text(&s, "first.area").contains('≈'));

        // A preselected body is Selection 1.
        h.app_mut().finish_tool(amg_ui::tool::FinishReason::Cancel);
        h.drive_until_idle();
        let body = h
            .document()
            .evaluated()
            .bodies()
            .find(|(_, b)| matches!(b.geom, BodyGeom::Solid(_)))
            .unwrap()
            .0;
        let sel = &mut h.app_mut().active_document_mut().selection;
        sel.clear();
        sel.add(
            SelItem::from_target(PickTarget::Body {
                body,
                instance: 0,
                sub: None,
            })
            .unwrap(),
        );
        start(&mut h);
        let s = result(&mut h);
        assert_eq!(s.kinds, [Some(OperandKind::BrepBody)], "{s:?}");
    }
}
