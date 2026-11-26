//! CON-04: the Construct UI in the real app (`AppHarness`, kittest, CPU scene backend and CPU
//! picking).
//!
//! - All 20 variants are created from the Construct menu and from S-key search, edited,
//!   deleted from the browser, and every step is undone and redone. Without the `occt` feature
//!   the variants that need a B-rep face get a reference that cannot resolve, so their node
//!   shows an error (create / edit / delete / undo are the same); with it they use the faces
//!   of an extruded cylinder and every node is healthy.
//! - The view draws construction geometry sized by `display_extent`, hidden geometry is neither
//!   drawn nor picked, dialogs preview it and the offset arrow sits on the base plane.
//! - Construction planes are picked in the viewport as sketch planes and as Section Analysis
//!   bases; both follow an edit of the plane.
//! - The manual script (offset plane → sketch on it → extrude → edit the offset → the extrude
//!   follows); the extrude needs `occt`.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::print_stderr,
    missing_docs,
    reason = "test code"
)]

use std::sync::Arc;

use amg_core::{Point2, Point3, Transform, Vec3};
use amg_doc::naming::TopoKind;
#[cfg(feature = "occt")]
use amg_doc::naming::{self, ProfileFp};
use amg_doc::stub::SketchEntityKey;
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, Command, ConstructionGeom, ConstructionId,
    CreatedId, Dependency, Document, EntityRef, Expr, Feature, FeatureBox, FeatureError, FeatureId,
    FeatureOutput, Health, RecomputeCtx, SketchId,
};
use amg_features::construct::display::{ConstructionDisplay, display_extent};
use amg_features::construct::{
    AxisDef, ConstructAxis, ConstructPlane, ConstructPoint, OriginEntity, PlaneDef, PointDef,
    RefInput,
};
use amg_features::dialog::FieldValue;
use amg_features::inspect::doc::{SectionBase, analyses};
use amg_features::sketch::{PlaneInput, SketchFeature};
use amg_mesh::IndexedMesh;
use amg_render::{Camera, Projection};
use amg_ui::browser::NodeKey;
use amg_ui::command::{CmdId, Placement};
use amg_ui::construct::{COMMANDS, ConstructionLayer};
use amg_ui::feature_dialog::GenericFeatureTool;
use amg_ui::inspect::{SECTION_CMD, section_tool_snapshot};
use amg_ui::module::GroupId;
use amg_ui::pick::{PickMode, PickTarget};
use amg_ui::search::{Recency, SearchIndex};
use amg_ui::sketch::CREATE_SKETCH_CMD;
use amg_ui::testing::AppHarness;
use amg_ui::testing::egui_kittest::kittest::{NodeT as _, Queryable as _};
use amg_ui::{CommitBatch, SelItem};
use egui::accesskit::Role;
use serde::{Deserialize, Serialize};

// ---- fixtures -------------------------------------------------------------------------------

/// A 20 mm cube mesh with its corner at the origin.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Cube {
    size: u32,
}

impl Feature for Cube {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let s = f64::from(self.size);
        let v = (0..8)
            .map(|i| {
                Point3::new(
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
        Ok(FeatureOutput {
            bodies: vec![BodyChange {
                id: ctx.new_body_id(0),
                change: BodyChangeKind::Created {
                    geom: BodyGeom::Mesh(Arc::new(IndexedMesh::new(v, t).unwrap())),
                    naming: None,
                },
            }],
            ..FeatureOutput::default()
        })
    }
}

amg_doc::register_feature!(Cube, "uitest.con04_cube", version = 1);

fn expr(s: &str) -> Expr {
    Expr::Inline(amg_doc::ExprSrc(s.to_owned()))
}

fn add(doc: &mut Document, f: FeatureBox) -> FeatureId {
    let root = doc.design().root();
    add_in(doc, f, root)
}

fn add_in(doc: &mut Document, f: FeatureBox, context: amg_doc::ComponentId) -> FeatureId {
    let out = doc
        .apply(Command::AddFeature {
            at: None,
            feature: f,
            context,
            name: None,
        })
        .unwrap();
    out.created
        .iter()
        .find_map(|c| match c {
            CreatedId::Feature(f) => Some(*f),
            _ => None,
        })
        .expect("a feature was created")
}

fn offset_plane(base: OriginEntity, distance: &str) -> FeatureBox {
    FeatureBox::new(ConstructPlane {
        def: PlaneDef::Offset {
            base: RefInput::Origin(base),
            distance: expr(distance),
        },
    })
}

/// The app on `doc`, recomputed, looking down the Z axis at (x, y) from `height`.
fn harness(doc: Document, x: f64, y: f64, height: f64) -> AppHarness {
    amg_features::construct::link();
    let mut h = AppHarness::builder()
        .size(egui::vec2(1600.0, 1000.0))
        .build();
    let id = h.app_mut().add_document("con04".into(), doc);
    h.app_mut().activate(id);
    h.drive_until_idle();
    h.app_mut().active_document_mut().camera = Camera {
        eye: Point3::new(x, y, height),
        target: Point3::new(x, y, 0.0),
        up: amg_core::Dir3::Y,
        projection: Projection::Perspective { fov_y: 0.8 },
    };
    h.drive_until_idle();
    h
}

fn tool(h: &mut AppHarness) -> &mut GenericFeatureTool {
    h.app_mut()
        .active_tool_as::<GenericFeatureTool>()
        .expect("the generic dialog is active")
}

fn applied(h: &AppHarness) -> usize {
    h.document().history().stack().applied()
}

fn undo(h: &mut AppHarness) {
    h.run_command(CmdId("edit.undo"));
    h.drive_until_idle();
}

fn redo(h: &mut AppHarness) {
    h.run_command(CmdId("edit.redo"));
    h.drive_until_idle();
}

fn click_button(h: &mut AppHarness, label: &str) {
    h.kittest()
        .get_by_role_and_label(Role::Button, label)
        .click();
    h.drive_until_idle();
}

fn ok(h: &mut AppHarness) {
    let node = h.kittest().get_by_role_and_label(Role::Button, "OK");
    assert!(
        !node.accesskit_node().is_disabled(),
        "OK is disabled: {:?}",
        h.app_mut()
            .active_tool_as::<GenericFeatureTool>()
            .map(|t| (t.ok_state().clone(), t.values().clone()))
    );
    click_button(h, "OK");
    assert!(h.app().active_tool().is_none(), "the dialog closed");
}

fn def_of<F: Feature + Clone>(h: &AppHarness, id: FeatureId) -> F {
    let (_, node) = h.document().design().timeline().node(id).expect("a node");
    node.feature
        .get()
        .as_any()
        .downcast_ref::<F>()
        .unwrap_or_else(|| panic!("{} is another kind", node.feature.kind()))
        .clone()
}

/// The reference inputs and variant of a construction feature (expressions become model
/// parameters on commit, so definitions are compared through these).
fn refs_of(h: &AppHarness, id: FeatureId) -> (String, Vec<RefInput>) {
    let (_, node) = h.document().design().timeline().node(id).expect("a node");
    let f = node.feature.get().as_any();
    let name = |d: String| {
        d.split([' ', '{', '('])
            .next()
            .unwrap_or_default()
            .to_owned()
    };
    if let Some(p) = f.downcast_ref::<ConstructPlane>() {
        let refs = p.def.refs().into_iter().cloned().collect();
        (name(format!("{:?}", p.def)), refs)
    } else if let Some(a) = f.downcast_ref::<ConstructAxis>() {
        let refs = a.def.refs().into_iter().cloned().collect();
        (name(format!("{:?}", a.def)), refs)
    } else {
        let p = f.downcast_ref::<ConstructPoint>().expect("construction");
        let refs = p.def.refs().into_iter().cloned().collect();
        (name(format!("{:?}", p.def)), refs)
    }
}

fn health(h: &AppHarness, id: FeatureId) -> Health {
    h.document()
        .evaluated()
        .node(id)
        .expect("evaluated")
        .health
        .clone()
}

fn geometry(h: &AppHarness, id: FeatureId) -> Option<ConstructionGeom> {
    h.document()
        .evaluated()
        .construction(ConstructionId(id))
        .copied()
}

fn plane_z(h: &AppHarness, id: FeatureId) -> f64 {
    match geometry(h, id) {
        Some(ConstructionGeom::Plane(p)) => p.origin.z,
        other => panic!("not a plane: {other:?}"),
    }
}

// ---- the 20 variants ------------------------------------------------------------------------

/// The shared inputs of the variant table.
struct Fixture {
    /// Lines: along X and Y through the origin, and one crossing the YZ plane at y = 3.
    lines: [EntityRef; 3],
    /// Circles: radius 4 about (30, 0) and radius 2 about (30, 20).
    circles: [EntityRef; 2],
    /// Points (1, 1), (10, 1), (1, 10), (60, 0), (61, 0).
    points: [EntityRef; 5],
    /// The plane 5 mm above XY.
    plane: EntityRef,
    /// A cylindrical face (radius 3 about (60, 0)) and the bottom face under it.
    side: EntityRef,
    bottom: EntityRef,
    /// The cube whose items stand for unresolvable face picks without OCCT.
    cube: BodyId,
}

impl Fixture {
    /// A selection item for `e`. Sketch entities have no pick target of their own yet (only
    /// whole sketches), so each gets a distinct stand-in: the selection is deduplicated by
    /// target.
    fn item(&self, e: &EntityRef) -> SelItem {
        let sub = |kind, index| PickTarget::Body {
            body: self.cube,
            instance: 0,
            sub: Some((kind, index)),
        };
        let index = |k: u64| u32::try_from(k).expect("a small key");
        let target = match e {
            EntityRef::Construction(c) => PickTarget::Construction(*c),
            EntityRef::SketchCurve { curve, .. } => sub(TopoKind::Edge, index(curve.0)),
            EntityRef::SketchPoint { point, .. } => sub(TopoKind::Vertex, index(point.0)),
            EntityRef::Topo(t) => sub(t.kind, t.key + 100),
            _ => panic!("no item for {e:?}"),
        };
        SelItem {
            target,
            entity: e.clone(),
        }
    }
}

#[cfg(feature = "occt")]
fn cylinder_faces(doc: &mut Document) -> (EntityRef, EntityRef) {
    use amg_brep::geom::Surface;
    let mut f = SketchFeature::new(PlaneInput::OriginXY);
    f.edit_sketch(|s| s.add_circle(Point2::new(60.0, 0.0), 3.0))
        .unwrap();
    let sketch = add(doc, FeatureBox::new(f));
    let jobs = amg_jobs::JobSystem::new(amg_jobs::JobSystemConfig::default()).unwrap();
    doc.recompute_blocking(&jobs).unwrap();
    let g = Arc::clone(doc.evaluated().sketch(SketchId(sketch)).unwrap());
    let region = &g.profiles.profiles[0];
    let profile = EntityRef::Profile {
        sketch: SketchId(sketch),
        profile: amg_doc::stub::ProfileKey(region.id.0),
        fingerprint: Some(ProfileFp(region.fingerprint.clone())),
    };
    let extrude = add(
        doc,
        FeatureBox::new(amg_features::solid::Extrude::distance(
            vec![profile],
            "10 mm",
            amg_features::solid::OpTarget::default(),
        )),
    );
    doc.recompute_blocking(&jobs).unwrap();
    let body = BodyId {
        origin: extrude,
        slot: 0,
    };
    let (mut side, mut bottom) = (None, None);
    {
        let eval = doc.evaluated();
        let geom = &eval.body(body).expect("the cylinder").geom;
        let BodyGeom::Solid(solid) = geom else {
            panic!("not a solid");
        };
        let shape = solid.brep().expect("B-rep");
        for face in shape.faces() {
            let r = || {
                EntityRef::Topo(
                    naming::topo_ref_at(
                        &**eval,
                        naming::default_kernel(),
                        body,
                        TopoKind::Face,
                        face.0,
                    )
                    .expect("named"),
                )
            };
            match shape.surface(face) {
                Surface::Cylinder { .. } => side = Some(r()),
                Surface::Plane(_) => {
                    let low = shape
                        .face_edges(face)
                        .iter()
                        .flat_map(|&e| shape.edge_vertices(e))
                        .all(|v| shape.vertex_point(v).z.abs() < 1e-9);
                    if low {
                        bottom = Some(r());
                    }
                }
                _ => {}
            }
        }
    }
    (side.expect("side face"), bottom.expect("bottom face"))
}

/// Face references that cannot resolve (there is no B-rep kernel): invented faces of the cube.
#[cfg(not(feature = "occt"))]
fn cylinder_faces(cube: BodyId) -> (EntityRef, EntityRef) {
    use amg_doc::naming::{NameSeg, Role, TopoName, TopoRef};
    let face = |key: u16| {
        EntityRef::Topo(TopoRef {
            body: cube,
            kind: TopoKind::Face,
            name: TopoName::new(vec![NameSeg::Gen {
                feature: FeatureId(9999),
                role: Role::new("Gone", key.into()),
                from: None,
            }]),
            fingerprint: None,
            key: u32::from(key),
        })
    };
    (face(1), face(2))
}

fn fixture() -> (Document, Fixture) {
    amg_features::construct::link();
    amg_features::sketch::link();
    let mut doc = Document::new("variants");
    let cube_feature = add(&mut doc, FeatureBox::new(Cube { size: 2 }));
    let cube = BodyId {
        origin: cube_feature,
        slot: 0,
    };
    let mut f = SketchFeature::new(PlaneInput::OriginXY);
    let p = Point2::new;
    let (lines, circles, points) = f
        .edit_sketch(|s| {
            Ok((
                [
                    s.add_line(p(0.0, 0.0), p(10.0, 0.0))?,
                    s.add_line(p(0.0, 0.0), p(0.0, 10.0))?,
                    s.add_line(p(-5.0, 3.0), p(10.0, 3.0))?,
                ],
                [
                    s.add_circle(p(30.0, 0.0), 4.0)?,
                    s.add_circle(p(30.0, 20.0), 2.0)?,
                ],
                [
                    s.add_point(p(1.0, 1.0))?,
                    s.add_point(p(10.0, 1.0))?,
                    s.add_point(p(1.0, 10.0))?,
                    s.add_point(p(60.0, 0.0))?,
                    s.add_point(p(61.0, 0.0))?,
                ],
            ))
        })
        .unwrap();
    let sketch = SketchId(add(&mut doc, FeatureBox::new(f)));
    let curve = |e: amg_sketch::EntityId| EntityRef::SketchCurve {
        sketch,
        curve: SketchEntityKey(u64::from(e.0)),
    };
    let point = |e: amg_sketch::EntityId| EntityRef::SketchPoint {
        sketch,
        point: SketchEntityKey(u64::from(e.0)),
    };
    let plane = add(&mut doc, offset_plane(OriginEntity::XYPlane, "5 mm"));
    #[cfg(feature = "occt")]
    let (side, bottom) = cylinder_faces(&mut doc);
    #[cfg(not(feature = "occt"))]
    let (side, bottom) = cylinder_faces(cube);
    let fixture = Fixture {
        lines: lines.map(curve),
        circles: circles.map(curve),
        points: points.map(point),
        plane: EntityRef::Construction(ConstructionId(plane)),
        side,
        bottom,
        cube,
    };
    (doc, fixture)
}

/// A change made in the edit dialog.
enum Edit {
    /// Chooses an option of a source chooser (or any choice field).
    Choose(&'static str, &'static str),
    /// Retypes an expression.
    Type(&'static str, &'static str),
    /// Replaces the item of a selection field.
    Pick(&'static str, EntityRef),
}

/// One variant's lifecycle.
struct Case {
    cmd: &'static str,
    /// The Rust variant name of the definition.
    def: &'static str,
    /// Selected before the command runs.
    preselect: Vec<EntityRef>,
    /// Source choosers set in the dialog.
    choose: Vec<(&'static str, &'static str)>,
    /// The expected inputs.
    refs: Vec<RefInput>,
    /// The edit, and the inputs afterwards.
    edit: Edit,
    edited: Vec<RefInput>,
    /// Needs a real B-rep face to compute.
    needs_faces: bool,
}

fn cases(f: &Fixture) -> Vec<Case> {
    use OriginEntity as O;
    let r = |e: &EntityRef| RefInput::Ref(e.clone());
    let o = RefInput::Origin;
    let [x, y, low] = &f.lines;
    let [c1, c2] = &f.circles;
    let [p0, p1, p2, centre, near] = &f.points;
    let plane = &f.plane;
    let case = |cmd,
                def,
                preselect: Vec<&EntityRef>,
                choose: Vec<(&'static str, &'static str)>,
                refs: Vec<RefInput>,
                edit,
                edited: Vec<RefInput>,
                needs_faces| Case {
        cmd,
        def,
        preselect: preselect.into_iter().cloned().collect(),
        choose,
        refs,
        edit,
        edited,
        needs_faces,
    };
    vec![
        case(
            "construct.offset_plane",
            "Offset",
            vec![],
            vec![("offset_base_from", "origin_xy")],
            vec![o(O::XYPlane)],
            Edit::Type("offset_distance", "25 mm"),
            vec![o(O::XYPlane)],
            false,
        ),
        case(
            "construct.plane_at_angle",
            "AtAngle",
            vec![x],
            vec![],
            vec![r(x)],
            Edit::Type("at_angle_angle", "30 deg"),
            vec![r(x)],
            false,
        ),
        case(
            "construct.tangent_plane",
            "Tangent",
            vec![&f.side],
            // The reference plane must not be perpendicular to the cylinder axis.
            vec![("tangent_reference_from", "origin_xz")],
            vec![r(&f.side), o(O::XZPlane)],
            Edit::Choose("tangent_reference_from", "origin_yz"),
            vec![r(&f.side), o(O::YZPlane)],
            true,
        ),
        case(
            "construct.midplane",
            "Midplane",
            vec![plane],
            vec![("mid_b_from", "origin_xy")],
            vec![r(plane), o(O::XYPlane)],
            Edit::Choose("mid_b_from", "origin_xz"),
            vec![r(plane), o(O::XZPlane)],
            false,
        ),
        case(
            "construct.plane_two_edges",
            "TwoEdges",
            vec![x, y],
            vec![],
            vec![r(x), r(y)],
            Edit::Choose("edges_b_from", "origin_y"),
            vec![r(x), o(O::YAxis)],
            false,
        ),
        case(
            "construct.plane_three_points",
            "ThreePoints",
            vec![p0, p1, p2],
            vec![],
            vec![r(p0), r(p1), r(p2)],
            Edit::Choose("points_a_from", "origin_point"),
            vec![o(O::Point), r(p1), r(p2)],
            false,
        ),
        case(
            "construct.plane_tangent_at_point",
            "TangentAtPoint",
            vec![&f.side, near],
            vec![],
            vec![r(&f.side), r(near)],
            Edit::Choose("tap_point_from", "origin_point"),
            vec![r(&f.side), o(O::Point)],
            true,
        ),
        case(
            "construct.plane_along_path",
            "AlongPath",
            vec![x],
            vec![],
            vec![r(x)],
            Edit::Type("fraction", "0.25"),
            vec![r(x)],
            false,
        ),
        case(
            "construct.axis_revolved",
            "ThroughRevolved",
            vec![c1],
            vec![],
            vec![r(c1)],
            Edit::Pick("rev_entity", c2.clone()),
            vec![r(c2)],
            false,
        ),
        case(
            "construct.axis_perpendicular_at_point",
            "PerpendicularAtPoint",
            vec![&f.bottom, centre],
            vec![],
            vec![r(&f.bottom), r(centre)],
            Edit::Pick("perp_point", near.clone()),
            vec![r(&f.bottom), r(near)],
            true,
        ),
        case(
            "construct.axis_two_planes",
            "ThroughTwoPlanes",
            vec![],
            vec![
                ("planes_a_from", "origin_xy"),
                ("planes_b_from", "origin_xz"),
            ],
            vec![o(O::XYPlane), o(O::XZPlane)],
            Edit::Choose("planes_b_from", "origin_yz"),
            vec![o(O::XYPlane), o(O::YZPlane)],
            false,
        ),
        case(
            "construct.axis_two_points",
            "ThroughTwoPoints",
            vec![p0, p1],
            vec![],
            vec![r(p0), r(p1)],
            Edit::Choose("line_b_from", "origin_point"),
            vec![r(p0), o(O::Point)],
            false,
        ),
        case(
            "construct.axis_through_edge",
            "ThroughEdge",
            vec![x],
            vec![],
            vec![r(x)],
            Edit::Choose("axis_edge_from", "origin_z"),
            vec![o(O::ZAxis)],
            false,
        ),
        case(
            "construct.axis_perpendicular_to_face",
            "PerpendicularToFaceAtPoint",
            vec![&f.side, near],
            vec![],
            vec![r(&f.side), r(near)],
            Edit::Choose("perpf_point_from", "origin_point"),
            vec![r(&f.side), o(O::Point)],
            true,
        ),
        case(
            "construct.point_at_vertex",
            "AtVertex",
            vec![p1],
            vec![],
            vec![r(p1)],
            Edit::Choose("vertex_from", "origin_point"),
            vec![o(O::Point)],
            false,
        ),
        case(
            "construct.point_two_edges",
            "ThroughTwoEdges",
            vec![x, y],
            vec![],
            vec![r(x), r(y)],
            Edit::Choose("cross_b_from", "origin_y"),
            vec![r(x), o(O::YAxis)],
            false,
        ),
        case(
            "construct.point_three_planes",
            "ThroughThreePlanes",
            vec![plane],
            vec![("three_b_from", "origin_xz"), ("three_c_from", "origin_yz")],
            vec![r(plane), o(O::XZPlane), o(O::YZPlane)],
            Edit::Choose("three_a_from", "origin_xy"),
            vec![o(O::XYPlane), o(O::XZPlane), o(O::YZPlane)],
            false,
        ),
        case(
            "construct.point_at_center",
            "AtCenter",
            vec![c1],
            vec![],
            vec![r(c1)],
            Edit::Pick("center_entity", c2.clone()),
            vec![r(c2)],
            false,
        ),
        case(
            "construct.point_edge_plane",
            "EdgeAndPlane",
            vec![low],
            vec![("ep_plane_from", "origin_yz")],
            vec![r(low), o(O::YZPlane)],
            Edit::Choose("ep_edge_from", "origin_x"),
            vec![o(O::XAxis), o(O::YZPlane)],
            false,
        ),
        case(
            "construct.point_along_path",
            "AlongPath",
            vec![x],
            vec![],
            vec![r(x)],
            Edit::Type("fraction", "0.25"),
            vec![r(x)],
            false,
        ),
    ]
}

/// Opens the dialog from the toolbar's Construct menu.
fn from_menu(h: &mut AppHarness, label: &str) {
    h.kittest().get_by_label("CONSTRUCT ⏷").click();
    h.drive_until_idle();
    click_button(h, label);
}

/// Opens the dialog with the S-key search: type the command's name, Enter.
fn from_search(h: &mut AppHarness, id: CmdId, label: &str) {
    // The popup's focused text field keeps repainting (caret blink): step, don't wait for idle.
    h.kittest().key_press(egui::Key::S);
    h.kittest().run_steps(3);
    assert!(h.app().search().is_open(), "S opens the search popup");
    h.kittest().event(egui::Event::Text(label.to_owned()));
    h.kittest().run_steps(3);
    assert_eq!(
        h.app().search().results().first().map(|r| r.id),
        Some(id),
        "search {label}"
    );
    h.kittest().key_press(egui::Key::Enter);
    h.drive_until_idle();
}

/// Sets the source choosers of the open dialog.
fn fill(h: &mut AppHarness, case: &Case) {
    for (key, option) in &case.choose {
        let t = tool(h);
        *t.values_mut()
            .0
            .iter_mut()
            .find(|(k, _)| k.0 == *key)
            .unwrap_or_else(|| panic!("no field {key}"))
            .1 = FieldValue::Choice(option);
    }
    h.drive_until_idle();
}

fn new_feature(h: &AppHarness, before: &[FeatureId]) -> FeatureId {
    let new: Vec<FeatureId> = h
        .document()
        .design()
        .timeline()
        .nodes()
        .map(|n| n.id)
        .filter(|id| !before.contains(id))
        .collect();
    assert_eq!(new.len(), 1, "one new feature");
    new[0]
}

fn feature_ids(h: &AppHarness) -> Vec<FeatureId> {
    h.document()
        .design()
        .timeline()
        .nodes()
        .map(|n| n.id)
        .collect()
}

/// Right-clicks the browser row of construction feature `id` and chooses "Delete".
fn delete_from_browser(h: &mut AppHarness, id: FeatureId) {
    let root = h.document().design().root();
    let folder = NodeKey::owned(format!("constructions/{}", root.0));
    h.app_mut().browser_mut().set_expanded(&folder, true);
    let key = NodeKey::owned(format!("construction/{}", id.0));
    h.app_mut().browser_mut().reveal(&key);
    h.drive_until_idle();
    let label = h
        .app()
        .browser()
        .node(&key)
        .unwrap_or_else(|| panic!("no browser row for {id:?}"))
        .label
        .clone();
    h.kittest().get_by_label(&label).click_secondary();
    h.drive_until_idle();
    click_button(h, "Delete");
}

/// Right-clicks the timeline node of `id` and chooses "Delete" (geometry that failed to
/// compute has no browser row).
fn delete_from_timeline(h: &mut AppHarness, id: FeatureId) {
    let at = h
        .app()
        .timeline()
        .drawn_node(id)
        .expect("the node is drawn")
        .rect
        .center();
    h.click_at(at, egui::PointerButton::Secondary, egui::Modifiers::NONE);
    h.drive_until_idle();
    click_button(h, "Delete");
}

#[test]
fn every_variant_is_created_edited_deleted_and_undone() {
    let (doc, f) = fixture();
    let mut h = harness(doc, 20.0, 5.0, 200.0);
    let cases = cases(&f);
    assert_eq!(cases.len(), 20);
    let mut covered: Vec<&str> = cases.iter().map(|c| c.cmd).collect();
    covered.sort_unstable();
    let mut all: Vec<&str> = COMMANDS.iter().map(|c| c.id.0).collect();
    all.sort_unstable();
    assert_eq!(covered, all);

    for case in &cases {
        let id = CmdId(case.cmd);
        let label = h
            .app()
            .registry()
            .command(id)
            .unwrap_or_else(|| panic!("no command {}", case.cmd))
            .label
            .to_string();
        let base = applied(&h);
        let before = feature_ids(&h);
        let items: Vec<SelItem> = case.preselect.iter().map(|e| f.item(e)).collect();
        let healthy = !case.needs_faces || cfg!(feature = "occt");

        // Created from the menu.
        h.app_mut()
            .active_document_mut()
            .selection
            .set(items.clone());
        h.drive_until_idle();
        from_menu(&mut h, &label);
        assert_eq!(h.app().active_tool(), Some(id), "{}", case.cmd);
        fill(&mut h, case);
        ok(&mut h);
        assert_eq!(applied(&h), base + 1, "{}: one undo step", case.cmd);
        let node = new_feature(&h, &before);
        let created = (case.def.to_owned(), case.refs.clone());
        assert_eq!(refs_of(&h, node), created, "{}", case.cmd);
        if healthy {
            assert_eq!(health(&h, node), Health::Ok, "{}", case.cmd);
            assert!(geometry(&h, node).is_some(), "{}", case.cmd);
        } else {
            assert!(matches!(health(&h, node), Health::Error(_)), "{}", case.cmd);
        }
        let first = geometry(&h, node);
        // Undone and redone.
        undo(&mut h);
        assert_eq!(feature_ids(&h), before, "{}: undo create", case.cmd);
        redo(&mut h);
        assert_eq!(refs_of(&h, node), created);
        assert_eq!(geometry(&h, node), first);

        // Edited through the same dialog.
        h.app_mut().active_document_mut().selection.clear();
        h.app_mut().edit_feature(node);
        h.drive_until_idle();
        assert_eq!(tool(&mut h).editing(), Some(node), "{}", case.cmd);
        match &case.edit {
            Edit::Choose(key, option) => {
                let t = tool(&mut h);
                let field = t.values_mut().0.iter_mut().find(|(k, _)| k.0 == *key);
                *field.expect("the chooser").1 = FieldValue::Choice(option);
            }
            Edit::Type(key, text) => {
                let t = tool(&mut h);
                let field = t.values_mut().0.iter_mut().find(|(k, _)| k.0 == *key);
                let FieldValue::Expr { src, .. } = field.expect("the expression").1 else {
                    panic!("{key} is not an expression");
                };
                (*text).clone_into(src);
            }
            Edit::Pick(key, e) => {
                let item = f.item(e);
                assert!(tool(&mut h).set_selection(key, vec![item]));
            }
        }
        h.drive_until_idle();
        ok(&mut h);
        assert_eq!(applied(&h), base + 2, "{}: one undo step", case.cmd);
        let edited = (case.def.to_owned(), case.edited.clone());
        assert_eq!(refs_of(&h, node), edited, "{}", case.cmd);
        if healthy {
            assert_eq!(health(&h, node), Health::Ok, "{}: edited", case.cmd);
        }
        let second = geometry(&h, node);
        if healthy && matches!(case.edit, Edit::Type(..)) {
            assert_ne!(second, first, "{}: the edit moved it", case.cmd);
        }
        undo(&mut h);
        assert_eq!(refs_of(&h, node), created, "{}: undo edit", case.cmd);
        assert_eq!(geometry(&h, node), first);
        redo(&mut h);
        assert_eq!(refs_of(&h, node), edited);
        assert_eq!(geometry(&h, node), second);

        // Deleted from the browser (from the timeline if it has no geometry).
        if healthy {
            delete_from_browser(&mut h, node);
        } else {
            delete_from_timeline(&mut h, node);
        }
        assert_eq!(applied(&h), base + 3, "{}: one undo step", case.cmd);
        assert_eq!(feature_ids(&h), before, "{}: delete", case.cmd);
        assert!(geometry(&h, node).is_none());
        undo(&mut h);
        assert_eq!(refs_of(&h, node), edited, "{}: undo delete", case.cmd);
        assert_eq!(geometry(&h, node), second);
        redo(&mut h);
        assert_eq!(feature_ids(&h), before);

        // Back to the fixture, then created again from the S-key search.
        for _ in 0..3 {
            undo(&mut h);
        }
        assert_eq!(applied(&h), base);
        h.app_mut().active_document_mut().selection.set(items);
        h.drive_until_idle();
        from_search(&mut h, id, &label);
        assert_eq!(h.app().active_tool(), Some(id), "{}: search", case.cmd);
        fill(&mut h, case);
        ok(&mut h);
        let node = new_feature(&h, &before);
        assert_eq!(refs_of(&h, node), created, "{}: search", case.cmd);
        undo(&mut h);
        assert_eq!(feature_ids(&h), before);
    }
}

#[test]
fn the_commands_are_in_both_construct_groups_and_searchable() {
    let h = AppHarness::new();
    let reg = h.app().registry();
    for group in ["design.solid.construct", "design.surface.construct"] {
        let ids: Vec<&str> = reg
            .group_commands(GroupId(group))
            .map(|(_, c)| c.id.0)
            .collect();
        for c in &COMMANDS {
            assert!(ids.contains(&c.id.0), "{} in {group}", c.id.0);
        }
        // Menu order is the variant order, before the generic dialogs; nothing is pinned.
        assert_eq!(ids[..20], COMMANDS.map(|c| c.id.0));
        assert!(
            reg.group_commands(GroupId(group))
                .all(|(pinned, _)| !pinned)
        );
    }
    let index = SearchIndex::build(reg);
    for c in &COMMANDS {
        let def = reg.command(c.id).unwrap();
        assert!(
            def.placements
                .iter()
                .any(|p| matches!(p, Placement::Toolbar { .. }))
        );
        let hits = index.search(&def.label, &Recency::new());
        assert_eq!(
            hits.first().map(|x| x.id),
            Some(c.id),
            "search {}",
            def.label
        );
    }
}

// ---- dialog behaviour -----------------------------------------------------------------------

#[test]
fn a_dialog_picks_in_the_viewport_previews_and_has_an_offset_arrow() {
    let mut doc = Document::new("pick");
    amg_features::construct::link();
    let base = add(&mut doc, offset_plane(OriginEntity::XYPlane, "5 mm"));
    let mut h = harness(doc, 0.0, 0.0, 120.0);
    h.run_command(CmdId("construct.offset_plane"));
    h.drive_until_idle();
    assert_eq!(
        tool(&mut h).active_field().map(|k| k.0),
        Some("offset_base")
    );
    // Click the plane in the viewport.
    h.click_world(Point3::new(6.0, 4.0, 5.0));
    h.drive_until_idle();
    assert_eq!(
        tool(&mut h).values().get("offset_base"),
        Some(&FieldValue::Selection(vec![EntityRef::Construction(
            ConstructionId(base)
        )]))
    );
    // The preview draws the new plane 10 mm above the base, unpickable.
    let layer = h.app().doc_layer::<ConstructionLayer>().unwrap();
    assert_eq!(layer.shown().len(), 1);
    assert_eq!(layer.preview().len(), 1);
    assert_eq!(layer.preview()[0].handle, None);
    assert!((layer.preview()[0].shape.center().z - 15.0).abs() < 1e-9);
    // The offset arrow starts on the base plane and points along its normal.
    let gizmo = tool(&mut h).gizmo_of("offset_distance").expect("the arrow");
    let amg_ui::manip::GizmoPrim::Arrow { origin, dir, .. } = gizmo else {
        panic!("{gizmo:?}");
    };
    assert!((origin - Point3::new(0.0, 0.0, 15.0)).length() < 1e-9);
    assert!((dir.get() - Vec3::Z).length() < 1e-12);
    let before = feature_ids(&h);
    ok(&mut h);
    let node = new_feature(&h, &before);
    assert_eq!(plane_z(&h, node), 15.0);
    let layer = h.app().doc_layer::<ConstructionLayer>().unwrap();
    assert!(layer.preview().is_empty());
    assert_eq!(layer.shown().len(), 2);
}

#[test]
fn a_preselection_fills_the_inputs_it_fits_and_is_not_copied_into_the_empty_one() {
    let mut doc = Document::new("pre");
    amg_features::construct::link();
    let a = add(&mut doc, offset_plane(OriginEntity::XYPlane, "5 mm"));
    let mut h = harness(doc, 0.0, 0.0, 120.0);
    let c = ConstructionId(a);
    h.app_mut()
        .active_document_mut()
        .selection
        .set(vec![SelItem {
            target: PickTarget::Construction(c),
            entity: EntityRef::Construction(c),
        }]);
    h.drive_until_idle();
    h.run_command(CmdId("construct.midplane"));
    h.drive_until_idle();
    let t = tool(&mut h);
    assert_eq!(
        t.values().get("mid_a"),
        Some(&FieldValue::Selection(vec![EntityRef::Construction(c)]))
    );
    assert_eq!(t.active_field().map(|k| k.0), Some("mid_b"));
    assert_eq!(
        t.values().get("mid_b"),
        Some(&FieldValue::Selection(Vec::new()))
    );
    assert!(!t.ok_state().is_enabled());
}

// ---- display ----------------------------------------------------------------------------------

#[test]
fn the_view_draws_display_extents_with_pick_ids_and_skips_hidden_geometry() {
    let mut doc = Document::new("display");
    amg_features::construct::link();
    add(&mut doc, FeatureBox::new(Cube { size: 40 }));
    let plane = add(&mut doc, offset_plane(OriginEntity::XYPlane, "50 mm"));
    let axis = add(
        &mut doc,
        FeatureBox::new(ConstructAxis {
            def: AxisDef::ThroughEdge {
                edge: RefInput::Origin(OriginEntity::ZAxis),
            },
        }),
    );
    let point = add(
        &mut doc,
        FeatureBox::new(ConstructPoint {
            def: PointDef::AtVertex {
                point: RefInput::Origin(OriginEntity::Point),
            },
        }),
    );
    let mut h = harness(doc, 20.0, 20.0, 300.0);
    let bounds = h.scene().bounds();
    assert!(
        (bounds.max - Point3::splat(40.0)).length() < 1e-6,
        "{bounds:?}"
    );
    for id in [plane, axis, point] {
        let c = ConstructionId(id);
        let geom = geometry(&h, id).unwrap();
        let key = h
            .scene()
            .pick_map()
            .render(&PickTarget::Construction(c))
            .expect("a pick key")
            .0;
        let layer = h.app().doc_layer::<ConstructionLayer>().unwrap();
        assert_eq!(layer.display_of(c), Some(display_extent(&geom, &bounds)));
        let shown = layer.shown().iter().find(|s| s.id == c).unwrap();
        assert_eq!(shown.handle, Some(key.pick_id()));
        // The GPU pick of that overlay handle maps back to the geometry.
        let sub = amg_ui::scene::RenderSub {
            kind: amg_ui::scene::RenderSubKind::Handle,
            index: 0,
        };
        assert_eq!(
            h.scene().pick_map().target(key, Some(sub)),
            Some(PickTarget::Construction(c))
        );
    }
    // 1.2 × 40 mm planes; 1.2 × diagonal axes.
    let layer = h.app().doc_layer::<ConstructionLayer>().unwrap();
    let Some(ConstructionDisplay::PlaneSquare { half_size, .. }) =
        layer.display_of(ConstructionId(plane))
    else {
        panic!("no square");
    };
    assert!((half_size - 24.0).abs() < 1e-6);
    assert!(h.app().tool_overlays().primitive_count() >= 3);

    // The plane is picked in the viewport (beside the cube) and selected.
    h.click_world(Point3::new(-3.0, 20.0, 50.0));
    h.drive_until_idle();
    let selected: Vec<PickTarget> = h
        .app()
        .active_document()
        .selection
        .items()
        .iter()
        .map(|i| i.target)
        .collect();
    assert_eq!(selected, [PickTarget::Construction(ConstructionId(plane))]);

    // Hidden through the browser's record: not drawn, not picked.
    let cmd = amg_ui::browser::visibility::set_hidden_command(
        h.document().design(),
        &[amg_ui::browser::visibility::construction_key(
            ConstructionId(plane),
        )],
        true,
    )
    .unwrap();
    h.app_mut().submit(CommitBatch {
        label: "Hide".into(),
        commands: vec![cmd],
    });
    h.drive_until_idle();
    let layer = h.app().doc_layer::<ConstructionLayer>().unwrap();
    assert_eq!(layer.display_of(ConstructionId(plane)), None);
    assert_eq!(layer.shown().len(), 2);
    let ray = h
        .camera_view()
        .ray(h.world_to_screen(Point3::new(-3.0, 20.0, 50.0)));
    assert!(
        h.pick_ray(ray, PickMode::AllAlongRay)
            .iter()
            .all(|x| x.target != PickTarget::Construction(ConstructionId(plane)))
    );
    assert!(
        h.scene()
            .construction_picks()
            .iter()
            .all(|c| c.id != ConstructionId(plane))
    );
}

#[test]
fn geometry_in_a_placed_component_is_drawn_and_referenced_where_it_is_seen() {
    let mut doc = Document::new("placed");
    amg_features::construct::link();
    let root = doc.design().root();
    let out = doc
        .apply(Command::NewComponent {
            parent: root,
            name: "Part".into(),
            transform: Transform::from_translation(Vec3::new(0.0, 0.0, 30.0)),
        })
        .unwrap();
    let component = out
        .created
        .iter()
        .find_map(|c| match c {
            CreatedId::Component(c) => Some(*c),
            _ => None,
        })
        .expect("a component");
    let plane = add_in(
        &mut doc,
        offset_plane(OriginEntity::XYPlane, "5 mm"),
        component,
    );
    let h = harness(doc, 0.0, 0.0, 200.0);
    let c = ConstructionId(plane);
    let layer = h.app().doc_layer::<ConstructionLayer>().unwrap();
    assert!((layer.display_of(c).unwrap().center().z - 35.0).abs() < 1e-9);
    let (design, eval) = (h.document().design(), h.document().evaluated());
    let r = amg_ui::construct::world_ref(design, c);
    assert!(matches!(r, EntityRef::InContext { .. }), "{r:?}");
    let world = amg_ui::construct::base_plane(design, eval, &r).unwrap();
    assert!((world.origin.z - 35.0).abs() < 1e-9);
}

// ---- sketch planes --------------------------------------------------------------------------

/// Picks construction plane `at` (a point on it) with Create Sketch; returns the new sketch.
fn sketch_on(h: &mut AppHarness, at: Point3) -> FeatureId {
    let before = feature_ids(h);
    h.run_command(CREATE_SKETCH_CMD);
    h.drive_until_idle();
    h.hover_at(h.world_to_screen(at));
    h.drive_until_idle();
    h.click_world(at);
    h.drive_until_idle();
    new_feature(h, &before)
}

fn sketch_z(h: &AppHarness, sketch: FeatureId) -> f64 {
    h.document()
        .evaluated()
        .sketch(SketchId(sketch))
        .expect("evaluated sketch")
        .plane
        .origin
        .z
}

/// Edits the offset of construction plane `plane` through its dialog.
fn edit_offset(h: &mut AppHarness, plane: FeatureId, text: &str) {
    h.app_mut().edit_feature(plane);
    h.drive_until_idle();
    let t = tool(h);
    let field = t
        .values_mut()
        .0
        .iter_mut()
        .find(|(k, _)| k.0 == "offset_distance");
    let FieldValue::Expr { src, param } = field.expect("the offset").1 else {
        panic!("not an expression");
    };
    assert!(param.is_some(), "bound to the model parameter");
    text.clone_into(src);
    h.drive_until_idle();
    ok(h);
}

#[test]
fn a_construction_plane_picked_in_the_view_is_a_sketch_plane_that_follows_edits() {
    let mut doc = Document::new("sketch");
    amg_features::construct::link();
    let plane = add(&mut doc, offset_plane(OriginEntity::XYPlane, "15 mm"));
    let mut h = harness(doc, 0.0, 0.0, 150.0);
    let sketch = sketch_on(&mut h, Point3::new(7.0, 6.0, 15.0));
    let f: SketchFeature = def_of(&h, sketch);
    assert_eq!(
        f.plane,
        PlaneInput::Ref(EntityRef::Construction(ConstructionId(plane)))
    );
    assert_eq!(health(&h, sketch), Health::Ok);
    assert_eq!(sketch_z(&h, sketch), 15.0);
    h.run_command(amg_ui::sketch::FINISH_SKETCH_CMD);
    h.drive_until_idle();
    edit_offset(&mut h, plane, "40 mm");
    assert_eq!(sketch_z(&h, sketch), 40.0);
    undo(&mut h);
    assert_eq!(sketch_z(&h, sketch), 15.0);
}

// ---- section bases --------------------------------------------------------------------------

#[test]
fn a_construction_plane_is_a_section_base_that_follows_edits() {
    let mut doc = Document::new("section");
    amg_features::construct::link();
    add(&mut doc, FeatureBox::new(Cube { size: 20 }));
    // Above the cube, so the pick sees the plane and not the cube.
    let plane = add(&mut doc, offset_plane(OriginEntity::XYPlane, "25 mm"));
    let mut h = harness(doc, 10.0, 10.0, 200.0);
    let c = ConstructionId(plane);
    h.run_command(SECTION_CMD);
    h.drive_until_idle();
    click_button(&mut h, "Pick face");
    h.click_world(Point3::new(4.0, 6.0, 25.0));
    h.drive_until_idle();
    let ctx = h.kittest().ctx.clone();
    let snap = section_tool_snapshot(&ctx).expect("the dialog");
    assert!(
        matches!(&snap.draft.base, SectionBase::Ref { entity: EntityRef::Construction(e), .. } if *e == c),
        "{:?}",
        snap.draft.base
    );
    assert!(!snap.picking);
    assert_eq!(h.app().view_sections().planes[0].plane.origin.z, 25.0);
    click_button(&mut h, "OK");
    let record = |h: &AppHarness| {
        analyses(h.document().design())
            .into_iter()
            .next()
            .expect("one record")
            .1
            .unwrap()
    };
    assert!(matches!(record(&h).base, SectionBase::Ref { .. }));

    // The section follows the plane into the cube; the caps measure it.
    edit_offset(&mut h, plane, "12 mm");
    let planes = h.app().view_sections().planes.clone();
    assert_eq!(planes.len(), 1);
    assert_eq!(planes[0].plane.origin.z, 12.0);
    let stats = h
        .app()
        .doc_layer::<amg_ui::inspect::SectionLayer>()
        .unwrap()
        .snapshot();
    assert!(stats.current);
    assert!((stats.stats[0].area - 400.0).abs() < 1e-9, "{stats:?}");

    // A deleted base keeps the stored plane (S05 §3).
    h.app_mut().submit(CommitBatch {
        label: "Delete".into(),
        commands: vec![Command::DeleteFeatures { ids: vec![plane] }],
    });
    h.drive_until_idle();
    assert_eq!(h.app().view_sections().planes[0].plane.origin.z, 25.0);
}

#[test]
fn a_selected_construction_plane_starts_a_section_on_it() {
    let mut doc = Document::new("section-pre");
    amg_features::construct::link();
    add(&mut doc, FeatureBox::new(Cube { size: 20 }));
    let plane = add(&mut doc, offset_plane(OriginEntity::YZPlane, "3 mm"));
    let mut h = harness(doc, 0.0, 0.0, 120.0);
    let c = ConstructionId(plane);
    h.app_mut()
        .active_document_mut()
        .selection
        .set(vec![SelItem {
            target: PickTarget::Construction(c),
            entity: EntityRef::Construction(c),
        }]);
    h.drive_until_idle();
    h.run_command(SECTION_CMD);
    h.drive_until_idle();
    let ctx = h.kittest().ctx.clone();
    let snap = section_tool_snapshot(&ctx).expect("the dialog");
    assert!(matches!(snap.draft.base, SectionBase::Ref { .. }));
    let planes = h.app().view_sections().planes.clone();
    assert_eq!(planes[0].plane.origin.x, 3.0);
    assert_eq!(planes[0].plane.normal, amg_core::Dir3::X);
}

// ---- the manual script ------------------------------------------------------------------------

/// Offset plane → sketch on it → (rectangle) → extrude → edit the offset → the extrude
/// follows. The extrude needs the B-rep kernel.
#[cfg(feature = "occt")]
#[test]
fn offset_plane_sketch_extrude_edit_the_offset_and_the_extrude_follows() {
    amg_features::construct::link();
    amg_features::sketch::link();
    let mut h = harness(Document::new("script"), 0.0, 0.0, 150.0);
    // Offset plane from the menu.
    from_menu(&mut h, "Offset Plane");
    let t = tool(&mut h);
    let field = t
        .values_mut()
        .0
        .iter_mut()
        .find(|(k, _)| k.0 == "offset_base_from");
    *field.unwrap().1 = FieldValue::Choice("origin_xy");
    h.drive_until_idle();
    ok(&mut h);
    let plane = feature_ids(&h)[0];
    assert_eq!(plane_z(&h, plane), 10.0);
    // Sketch on it, picked in the viewport, with a rectangle.
    let sketch = sketch_on(&mut h, Point3::new(6.0, 6.0, 10.0));
    h.run_command(amg_ui::sketch::FINISH_SKETCH_CMD);
    h.drive_until_idle();
    let mut f: SketchFeature = def_of(&h, sketch);
    f.edit_sketch(|s| {
        amg_sketch::build::rectangle_2pt(s, Point2::new(0.0, 0.0), Point2::new(8.0, 6.0))
    })
    .unwrap();
    h.app_mut().submit(CommitBatch {
        label: "Rectangle".into(),
        commands: vec![Command::EditFeature {
            id: sketch,
            feature: FeatureBox::new(f),
        }],
    });
    h.drive_until_idle();
    // Extrude through the generic dialog.
    let g = Arc::clone(h.document().evaluated().sketch(SketchId(sketch)).unwrap());
    let region = &g.profiles.profiles[0];
    let profile = EntityRef::Profile {
        sketch: SketchId(sketch),
        profile: amg_doc::stub::ProfileKey(region.id.0),
        fingerprint: Some(ProfileFp(region.fingerprint.clone())),
    };
    h.run_command(CmdId("feature.solid.extrude"));
    h.drive_until_idle();
    let before = feature_ids(&h);
    assert!(tool(&mut h).set_selection(
        "profiles",
        vec![SelItem {
            target: PickTarget::Sketch(SketchId(sketch)),
            entity: profile,
        }]
    ));
    h.drive_until_idle();
    ok(&mut h);
    let extrude = new_feature(&h, &before);
    let bounds = |h: &AppHarness| {
        let body = BodyId {
            origin: extrude,
            slot: 0,
        };
        let eval = h.document().evaluated();
        let BodyGeom::Solid(solid) = &eval.body(body).expect("the extrusion").geom else {
            panic!("not a solid");
        };
        solid.brep().unwrap().bbox()
    };
    let b = bounds(&h);
    assert!(
        (b.min.z - 10.0).abs() < 1e-6 && (b.max.z - 20.0).abs() < 1e-6,
        "{b:?}"
    );
    // Edit the offset: the sketch and the extrude follow.
    edit_offset(&mut h, plane, "30 mm");
    assert_eq!(health(&h, extrude), Health::Ok);
    assert_eq!(sketch_z(&h, sketch), 30.0);
    let b = bounds(&h);
    assert!(
        (b.min.z - 30.0).abs() < 1e-6 && (b.max.z - 40.0).abs() < 1e-6,
        "{b:?}"
    );
    eprintln!(
        "manual script: the extrusion spans z = {} .. {} mm",
        b.min.z, b.max.z
    );
    undo(&mut h);
    let b = bounds(&h);
    assert!((b.min.z - 10.0).abs() < 1e-6, "{b:?}");

    // Solid edges and faces picked in the viewport are construction inputs: an axis through
    // the front edge of the top face (z = 20), then that face offset to 30. (In this order: the
    // offset plane would then be in front of the edge.)
    h.app_mut().active_document_mut().selection.clear();
    h.drive_until_idle();
    let before = feature_ids(&h);
    h.run_command(CmdId("construct.axis_through_edge"));
    h.drive_until_idle();
    h.click_world(Point3::new(4.0, 0.0, 20.0));
    h.drive_until_idle();
    ok(&mut h);
    let on_edge = new_feature(&h, &before);
    assert_eq!(health(&h, on_edge), Health::Ok);
    let Some(ConstructionGeom::Axis(axis)) = geometry(&h, on_edge) else {
        panic!("no axis");
    };
    assert!(axis.dir.get().cross(Vec3::X).length() < 1e-9, "{axis:?}");
    assert!(axis.point.y.abs() < 1e-9 && (axis.point.z - 20.0).abs() < 1e-9);
    let before = feature_ids(&h);
    h.run_command(CmdId("construct.offset_plane"));
    h.drive_until_idle();
    h.click_world(Point3::new(4.0, 3.0, 20.0));
    h.drive_until_idle();
    ok(&mut h);
    let on_face = new_feature(&h, &before);
    assert_eq!(health(&h, on_face), Health::Ok);
    assert!((plane_z(&h, on_face) - 30.0).abs() < 1e-9);
}

// ---- GPU picking through the overlay handles --------------------------------------------------

mod gpu {
    use std::cell::RefCell;
    use std::rc::Rc;

    use amg_jobs::gpu::{Gpu, GpuConfig};
    use amg_render::{Renderer, RendererConfig, ViewId};
    use amg_ui::pick::{GpuPickService, PickService};
    use amg_ui::viewport::{NullViewport, ViewportBackend, ViewportFrame};

    use super::*;

    /// `NullViewport` plus what the app's viewport does for picking: the S10 view follows the
    /// camera and gets the frame's overlays, and is encoded while picks are pending.
    struct GpuViewport {
        r: Rc<RefCell<Renderer>>,
        view: ViewId,
    }

    impl ViewportBackend for GpuViewport {
        fn show(&mut self, ui: &mut egui::Ui, frame: &ViewportFrame<'_>) -> egui::Response {
            let resp = NullViewport.show(ui, frame);
            let ppp = ui.ctx().pixels_per_point();
            #[allow(
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss,
                reason = "a positive view size"
            )]
            let size = [
                (resp.rect.width() * ppp).round() as u32,
                (resp.rect.height() * ppp).round() as u32,
            ];
            let mut r = self.r.borrow_mut();
            r.set_view_size(self.view, size);
            if let Some(v) = r.view_mut(self.view) {
                v.camera = *frame.camera;
            }
            r.scene_mut().begin_overlays().clone_from(frame.overlays);
            resp
        }

        fn pick_service(&mut self) -> Option<Box<dyn PickService>> {
            let r = Rc::clone(&self.r);
            Some(Box::new(GpuPickService::new(
                Box::new(move |f| f(&mut r.borrow_mut())),
                self.view,
            )))
        }

        fn after_frame(&mut self) {
            let mut r = self.r.borrow_mut();
            if r.pending_picks() > 0 {
                let cap = r.capture(self.view).expect("encode");
                let _ = cap.wait().expect("frame");
            }
        }
    }

    #[test]
    fn gpu_picks_construction_planes_axes_and_points_by_their_handles() {
        if !amg_testkit::gpu_available() {
            eprintln!("no GPU adapter: skipped");
            return;
        }
        let mut cfg = GpuConfig::from_env();
        cfg.optional_features |= amg_render::OPTIONAL_FEATURES;
        let gpu = Gpu::new_blocking(&cfg).expect("adapter probed, device opens");
        eprintln!("adapter: {} ({:?})", gpu.info().name, gpu.info().backend);
        let mut renderer = Renderer::new(Arc::clone(&gpu), &RendererConfig::default()).unwrap();
        let view = renderer.create_view([64, 64]);
        assert!(renderer.prepare_view(view), "pipeline variants");
        let r = Rc::new(RefCell::new(renderer));

        amg_features::construct::link();
        let mut doc = Document::new("gpu");
        let plane = add(&mut doc, offset_plane(OriginEntity::XYPlane, "5 mm"));
        // An axis and a point above the plane.
        let high = add(&mut doc, offset_plane(OriginEntity::XYPlane, "30 mm"));
        let axis = add(
            &mut doc,
            FeatureBox::new(ConstructAxis {
                def: AxisDef::ThroughTwoPlanes {
                    a: RefInput::Ref(EntityRef::Construction(ConstructionId(high))),
                    b: RefInput::Origin(OriginEntity::XZPlane),
                },
            }),
        );
        let point = add(
            &mut doc,
            FeatureBox::new(ConstructPoint {
                def: PointDef::ThroughThreePlanes {
                    a: RefInput::Ref(EntityRef::Construction(ConstructionId(high))),
                    b: RefInput::Origin(OriginEntity::YZPlane),
                    c: RefInput::Origin(OriginEntity::XZPlane),
                },
            }),
        );
        // `high` only carries the axis and the point: hidden, so it covers nothing.
        let hide = amg_ui::browser::visibility::set_hidden_command(
            doc.design(),
            &[amg_ui::browser::visibility::construction_key(
                ConstructionId(high),
            )],
            true,
        )
        .unwrap();
        doc.apply(hide).unwrap();

        let mut h = AppHarness::builder()
            .size(egui::vec2(1600.0, 1000.0))
            .viewport(Box::new(GpuViewport {
                r: Rc::clone(&r),
                view,
            }))
            .build();
        let id = h.app_mut().add_document("gpu".into(), doc);
        h.app_mut().activate(id);
        h.app_mut().active_document_mut().camera = Camera {
            eye: Point3::new(0.0, 0.0, 120.0),
            target: Point3::ZERO,
            up: amg_core::Dir3::Y,
            projection: Projection::Perspective { fov_y: 0.8 },
        };
        h.drive_until_idle();
        assert!(r.borrow_mut().prepare_view(view));
        for (at, want) in [
            (Point3::new(6.0, 5.0, 5.0), plane),
            (Point3::new(7.0, 0.0, 30.0), axis),
            (Point3::new(0.0, 0.0, 30.0), point),
        ] {
            h.app_mut().active_document_mut().selection.clear();
            h.drive_until_idle();
            h.click_world(at);
            h.drive_until_idle();
            let selected: Vec<PickTarget> = h
                .app()
                .active_document()
                .selection
                .items()
                .iter()
                .map(|i| i.target)
                .collect();
            assert_eq!(
                selected,
                [PickTarget::Construction(ConstructionId(want))],
                "at {at:?}"
            );
        }
        drop(h);
    }
}
