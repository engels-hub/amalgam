//! UI-06 tests: manipulators (`amg_ui::manip`) and their binding in the generic feature dialog,
//! kittest on the real app with the `NullViewport` (no kernel needed: the extrude arrow is
//! anchored on the evaluated sketch).
//!
//! - Done-when 1: dragging the extrude arrow by a scripted world delta of 12.5 mm writes the
//!   moved value (`10 mm` + 12.5 mm = `22.5 mm`, prefs precision) into the Distance field, with a
//!   heads-up value during the drag; editing the field moves the arrow (`world_to_screen`).
//! - Done-when 2: `Ctrl`-drag snaps to 1 mm (arrow) and 5° (ring of a test registration).
//! - Done-when 3: every one of the triad's 10 parts hit-tests as itself in 6 views (unit test on
//!   `CameraView`).
//! - Done-when 4: the gizmo screen size stays 90 ± 2 pt over a 1000× zoom range (perspective
//!   and orthographic, on `CameraView` and in the app).

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use amg_core::{Dir3, Mat3, Point2, Point3, Quat, Vec3};
use amg_doc::stub::ProfileKey;
use amg_doc::{
    Command, CreatedId, Dependency, Design, Document, EntityRef, Evaluated, Expr, Feature,
    FeatureBox, FeatureError, FeatureId, FeatureOutput, RecomputeCtx, SketchId,
};
use amg_features::dialog::{
    CommandMeta, DialogInit, DialogRegistration, DialogSchema, FieldErrors, FieldKey, FieldKind,
    FieldSpec, FieldValue, FieldValues, ManipAnchor, ManipKind,
};
use amg_features::sketch::{PlaneInput, SketchFeature};
use amg_render::{Camera, Projection};
use amg_sketch::build;
use amg_ui::command::CmdId;
use amg_ui::feature_dialog::GenericFeatureTool;
use amg_ui::input::CameraView;
use amg_ui::manip::{
    self, GIZMO_SIZE_PT, GizmoPart, GizmoPrim, HIT_TOLERANCE_PT, PartForm, TriadPart,
};
use amg_ui::testing::AppHarness;
use amg_ui::testing::egui_kittest::kittest::Queryable as _;
use amg_ui::{FinishReason, PickTarget, SelItem};
use serde::{Deserialize, Serialize};

const EXTRUDE: CmdId = CmdId("feature.solid.extrude");
const UI06: CmdId = CmdId("feature.uitest.ui06_manips");

// ---- helpers --------------------------------------------------------------------------------

fn add(doc: &mut Document, f: FeatureBox) -> FeatureId {
    let root = doc.design().root();
    let out = doc
        .apply(Command::AddFeature {
            at: None,
            feature: f,
            context: root,
            name: None,
        })
        .unwrap();
    out.created
        .iter()
        .find_map(|c| match c {
            CreatedId::Feature(f) => Some(*f),
            _ => None,
        })
        .unwrap()
}

/// A document with a 20 × 10 rectangle sketch on XY at the origin.
fn rect_doc() -> (Document, FeatureId) {
    let mut doc = Document::new("ui06");
    let mut s = SketchFeature::new(PlaneInput::OriginXY);
    s.edit_sketch(|sk| build::rectangle_2pt(sk, Point2::new(0.0, 0.0), Point2::new(20.0, 10.0)))
        .unwrap();
    let id = add(&mut doc, FeatureBox::new(s));
    (doc, id)
}

fn profile(eval: &Evaluated, s: FeatureId) -> EntityRef {
    let g = eval.sketch(SketchId(s)).unwrap();
    let p = &g.profiles.profiles[0];
    EntityRef::Profile {
        sketch: SketchId(s),
        profile: ProfileKey(p.id.0),
        fingerprint: Some(amg_doc::naming::ProfileFp(p.fingerprint.clone())),
    }
}

fn set_camera(h: &mut AppHarness, eye: Point3, target: Point3) {
    h.app_mut().active_document_mut().camera = Camera {
        eye,
        target,
        up: Dir3::Z,
        projection: Projection::Perspective { fov_y: 0.8 },
    };
    h.drive_until_idle();
}

/// The app on the rectangle with the Extrude dialog open on its profile (preselected), seen
/// obliquely so the distance arrow (+Z) is not end-on.
fn extrude_open() -> AppHarness {
    let (doc, s) = rect_doc();
    let mut h = AppHarness::builder()
        .size(egui::vec2(1400.0, 900.0))
        .build();
    let id = h.app_mut().add_document("ui06".into(), doc);
    h.app_mut().activate(id);
    h.drive_until_idle();
    set_camera(
        &mut h,
        Point3::new(90.0, -115.0, 75.0),
        Point3::new(10.0, 5.0, 5.0),
    );
    let item = SelItem {
        target: PickTarget::Sketch(SketchId(s)),
        entity: profile(h.document().evaluated(), s),
    };
    h.app_mut().active_document_mut().selection.set(vec![item]);
    h.run_command(EXTRUDE);
    h.drive_until_idle();
    assert_eq!(h.app().active_tool(), Some(EXTRUDE));
    h
}

fn tool(h: &mut AppHarness) -> &mut GenericFeatureTool {
    h.app_mut()
        .active_tool_as::<GenericFeatureTool>()
        .expect("the generic dialog is active")
}

fn field_text(h: &mut AppHarness, key: &'static str) -> String {
    match tool(h).values().get(key) {
        Some(FieldValue::Expr { src, .. }) => src.clone(),
        other => panic!("{key}: {other:?}"),
    }
}

fn arrow(h: &mut AppHarness) -> (Point3, Dir3) {
    match tool(h).gizmo_of("distance") {
        Some(GizmoPrim::Arrow { origin, dir, .. }) => (origin, dir),
        other => panic!("no distance arrow: {other:?}"),
    }
}

/// Replaces the text of the expression field `label` (as UI-05's tests do).
fn retype(h: &mut AppHarness, label: &str, text: &str) {
    h.kittest().get_by_label(label).focus();
    h.kittest().step();
    h.kittest()
        .key_press_modifiers(egui::Modifiers::COMMAND, egui::Key::A);
    h.kittest().step();
    h.kittest().get_by_label(label).type_text(text);
    h.kittest().step();
    let ctx = h.kittest().ctx.clone();
    ctx.memory_mut(|m| {
        if let Some(id) = m.focused() {
            m.surrender_focus(id);
        }
    });
    h.kittest().step();
}

/// Presses at `from`, moves to `mid` then `to` (one frame each), and returns whether
/// `heads_up` was shown at `mid`; releases at `to`.
fn drag_with(
    h: &mut AppHarness,
    mods: egui::Modifiers,
    from: egui::Pos2,
    to: egui::Pos2,
    heads_up: &str,
) -> bool {
    h.hover_at(from);
    h.button(from, egui::PointerButton::Primary, true, mods);
    let mut shown = false;
    for i in 1..=6_u8 {
        let t = f32::from(i) / 6.0;
        h.kittest()
            .event(egui::Event::PointerMoved(from + (to - from) * t));
        h.kittest().step();
        if i == 3 {
            h.kittest().step();
            shown = h.kittest().query_by_label(heads_up).is_some();
        }
    }
    h.button(to, egui::PointerButton::Primary, false, mods);
    h.drive_until_idle();
    shown
}

fn close(a: egui::Pos2, b: egui::Pos2, tol: f32) -> bool {
    a.distance(b) <= tol
}

// ---- Done-when 1: drag ↔ field, both ways ----------------------------------------------------

#[test]
fn dw1_dragging_the_extrude_arrow_writes_the_field_and_editing_the_field_moves_it() {
    let mut h = extrude_open();
    assert_eq!(field_text(&mut h, "distance"), "10 mm");
    // The arrow sits on the profile centroid moved by the distance, along +Z.
    let (origin, dir) = arrow(&mut h);
    assert!(
        origin.distance(Point3::new(10.0, 5.0, 10.0)) < 1e-6,
        "{origin:?}"
    );
    assert!(dir.get().distance(Vec3::Z) < 1e-9);
    // It is drawn as a tool overlay.
    assert!(h.app().tool_overlays().primitive_count() > 0);

    // Grab the middle of the shaft and drag by 12.5 mm along the axis.
    let cam = h.camera_view();
    let size = manip::world_size(&cam, origin);
    let grab = origin + dir.get() * (0.5 * size);
    let from = h.world_to_screen(grab);
    let to = h.world_to_screen(grab + dir.get() * 12.5);
    let before = h.document().history().revision();
    let heads_up = drag_with(&mut h, egui::Modifiers::NONE, from, to, "Heads-up Distance");
    assert!(
        heads_up,
        "the heads-up value follows the cursor during the drag"
    );
    assert_eq!(field_text(&mut h, "distance"), "22.5 mm");
    assert_eq!(tool(&mut h).dragged_field(), None, "the drag ended");
    assert!(
        h.kittest().query_by_label("Heads-up Distance").is_none(),
        "the heads-up value goes with the drag"
    );
    assert_eq!(
        h.document().history().revision(),
        before,
        "dragging only edits the dialog"
    );
    // The dialog shows the new value; the arrow followed.
    assert!(h.kittest().query_by_label_contains("22.50 mm").is_some());
    let (moved, _) = arrow(&mut h);
    assert!(
        moved.distance(Point3::new(10.0, 5.0, 22.5)) < 1e-6,
        "{moved:?}"
    );

    // Prefs precision: an odd delta is written with two decimals at most.
    let cam = h.camera_view();
    let size = manip::world_size(&cam, moved);
    let grab = moved + Vec3::Z * (0.5 * size);
    let from = h.world_to_screen(grab);
    let to = h.world_to_screen(grab + Vec3::Z * 3.3333);
    drag_with(&mut h, egui::Modifiers::NONE, from, to, "Heads-up Distance");
    assert_eq!(field_text(&mut h, "distance"), "25.83 mm");

    // Editing the field moves the arrow, on screen too.
    retype(&mut h, "Distance", "30 mm");
    h.drive_until_idle();
    let (origin, _) = arrow(&mut h);
    let want = Point3::new(10.0, 5.0, 30.0);
    assert!(origin.distance(want) < 1e-6, "{origin:?}");
    let cam = h.camera_view();
    let g = tool(&mut h).gizmo_of("distance").unwrap();
    let shape = manip::shape(&g, &cam);
    let base = shape.part(GizmoPart::Arrow).unwrap().points()[0];
    assert!(close(base, h.world_to_screen(want), 0.01), "{base:?}");
    // An expression works too, and a drag then replaces it.
    retype(&mut h, "Distance", "2 * 4 mm");
    h.drive_until_idle();
    let (origin, _) = arrow(&mut h);
    assert!(origin.distance(Point3::new(10.0, 5.0, 8.0)) < 1e-6);
    let size = manip::world_size(&cam, origin);
    let grab = origin + Vec3::Z * (0.5 * size);
    let (from, to) = (
        h.world_to_screen(grab),
        h.world_to_screen(grab + Vec3::Z * 2.0),
    );
    drag_with(&mut h, egui::Modifiers::NONE, from, to, "Heads-up Distance");
    assert_eq!(field_text(&mut h, "distance"), "10 mm");
    h.app_mut().finish_tool(FinishReason::Cancel);
}

#[test]
fn a_drag_commits_with_ok_and_esc_during_a_drag_restores_the_value() {
    let mut h = extrude_open();
    let (origin, dir) = arrow(&mut h);
    let cam = h.camera_view();
    let grab = origin + dir.get() * (0.5 * manip::world_size(&cam, origin));
    let from = h.world_to_screen(grab);
    let to = h.world_to_screen(grab + dir.get() * 5.0);
    // Esc in the middle of a drag: the value from before it, and the dialog stays open.
    h.hover_at(from);
    h.button(
        from,
        egui::PointerButton::Primary,
        true,
        egui::Modifiers::NONE,
    );
    for i in 1..=4_u8 {
        let t = f32::from(i) / 4.0;
        h.kittest()
            .event(egui::Event::PointerMoved(from + (to - from) * t));
        h.kittest().step();
    }
    assert_eq!(field_text(&mut h, "distance"), "15 mm");
    h.key(egui::Key::Escape);
    h.button(
        to,
        egui::PointerButton::Primary,
        false,
        egui::Modifiers::NONE,
    );
    h.drive_until_idle();
    assert_eq!(
        h.app().active_tool(),
        Some(EXTRUDE),
        "Esc ended the drag only"
    );
    assert_eq!(field_text(&mut h, "distance"), "10 mm");
    // A press on the arrow does not deselect the profile.
    h.click_at(from, egui::PointerButton::Primary, egui::Modifiers::NONE);
    h.drive_until_idle();
    assert!(matches!(
        tool(&mut h).values().get("profiles"),
        Some(FieldValue::Selection(p)) if p.len() == 1
    ));
    // Drag, then OK: one undo entry with the dragged distance.
    drag_with(&mut h, egui::Modifiers::NONE, from, to, "Heads-up Distance");
    assert_eq!(field_text(&mut h, "distance"), "15 mm");
    let applied = h.document().history().stack().applied();
    h.kittest().get_by_label("OK").click();
    h.drive_until_idle();
    assert_eq!(h.app().active_tool(), None);
    assert_eq!(h.document().history().stack().applied(), applied + 1);
    let node = h.document().design().timeline().nodes().last().unwrap();
    let ext = node
        .feature
        .downcast_ref::<amg_features::solid::Extrude>()
        .unwrap();
    let amg_features::solid::Extent::Distance(e) = &ext.side1.extent else {
        panic!("a distance extent");
    };
    let design = h.document().design();
    let v = design
        .params()
        .resolve(e, amg_doc::Dim::LENGTH, &design.meta().default_units)
        .unwrap()
        .value;
    assert!((v - 15.0).abs() < 1e-9, "{v}");
}

// ---- Done-when 2: Ctrl snapping ----------------------------------------------------------------

#[test]
fn dw2_ctrl_drag_snaps_the_arrow_to_1_mm() {
    let mut h = extrude_open();
    let (origin, dir) = arrow(&mut h);
    let cam = h.camera_view();
    let grab = origin + dir.get() * (0.5 * manip::world_size(&cam, origin));
    let from = h.world_to_screen(grab);
    let to = h.world_to_screen(grab + dir.get() * 12.3);
    drag_with(&mut h, egui::Modifiers::CTRL, from, to, "Heads-up Distance");
    assert_eq!(
        field_text(&mut h, "distance"),
        "22 mm",
        "10 + 12.3 snaps to 22"
    );
    // Without Ctrl the same drag is not snapped.
    let (origin, _) = arrow(&mut h);
    let grab = origin + dir.get() * (0.5 * manip::world_size(&cam, origin));
    let from = h.world_to_screen(grab);
    let to = h.world_to_screen(grab + dir.get() * 1.7);
    drag_with(&mut h, egui::Modifiers::NONE, from, to, "Heads-up Distance");
    assert_eq!(field_text(&mut h, "distance"), "23.7 mm");
    h.app_mut().finish_tool(FinishReason::Cancel);
}

// A test registration with an angle ring, a triad and a plane handle.

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Ui06Manips {
    angle: Expr,
    lift: Expr,
    slide: Expr,
}

impl Feature for Ui06Manips {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, _ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Ok(FeatureOutput::default())
    }

    fn exprs(&self) -> Vec<&Expr> {
        vec![&self.angle, &self.lift, &self.slide]
    }

    fn exprs_mut(&mut self) -> Vec<(&mut Expr, amg_core::Dimension)> {
        vec![
            (&mut self.angle, amg_core::Dimension::ANGLE),
            (&mut self.lift, amg_core::Dimension::LENGTH),
            (&mut self.slide, amg_core::Dimension::LENGTH),
        ]
    }
}

amg_doc::register_feature!(Ui06Manips, "uitest.ui06_manips", version = 1);

fn manips_schema() -> &'static DialogSchema {
    static S: std::sync::OnceLock<DialogSchema> = std::sync::OnceLock::new();
    S.get_or_init(|| {
        let mut angle = FieldSpec::new(
            "angle",
            "Angle",
            FieldKind::Expr {
                dim: amg_doc::Dim::ANGLE,
                default: "30 deg",
            },
        );
        angle.manipulator = Some(ManipKind::Angle);
        let mut lift = FieldSpec::new(
            "lift",
            "Lift",
            FieldKind::Expr {
                dim: amg_doc::Dim::LENGTH,
                default: "0 mm",
            },
        );
        lift.manipulator = Some(ManipKind::Triad);
        let mut slide = FieldSpec::new(
            "slide",
            "Slide",
            FieldKind::Expr {
                dim: amg_doc::Dim::LENGTH,
                default: "0 mm",
            },
        );
        slide.manipulator = Some(ManipKind::Plane);
        DialogSchema {
            kind: "uitest.ui06_manips",
            title: "UI06 Manipulators".into(),
            help: None,
            fields: vec![angle, lift, slide],
        }
    })
}

fn manips_defaults(_: &DialogInit<'_>) -> FieldValues {
    FieldValues::default()
}

fn manips_read(f: &dyn Feature, design: &Design) -> Option<FieldValues> {
    let m = f.as_any().downcast_ref::<Ui06Manips>()?;
    Some(
        FieldValues::default()
            .with("angle", FieldValue::from_expr(&m.angle, design))
            .with("lift", FieldValue::from_expr(&m.lift, design))
            .with("slide", FieldValue::from_expr(&m.slide, design)),
    )
}

fn manips_build(v: &FieldValues, _old: Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors> {
    Ok(FeatureBox::new(Ui06Manips {
        angle: v.expr("angle").map_err(FieldErrors::from)?,
        lift: v.expr("lift").map_err(FieldErrors::from)?,
        slide: v.expr("slide").map_err(FieldErrors::from)?,
    }))
}

fn manips_anchors(_: &dyn Feature, _: &Design, _: &Evaluated) -> Vec<ManipAnchor> {
    vec![
        ManipAnchor {
            field: FieldKey("angle"),
            origin: Point3::ZERO,
            axis: Dir3::Z,
            ref_dir: Some(Dir3::X),
            base: 0.0,
        },
        ManipAnchor {
            field: FieldKey("lift"),
            origin: Point3::new(40.0, 0.0, 0.0),
            axis: Dir3::Z,
            ref_dir: Some(Dir3::X),
            base: 0.0,
        },
        ManipAnchor {
            field: FieldKey("slide"),
            origin: Point3::new(-40.0, 0.0, 0.0),
            axis: Dir3::Z,
            ref_dir: Some(Dir3::X),
            base: 0.0,
        },
    ]
}

inventory::submit! {
    DialogRegistration {
        kind: "uitest.ui06_manips",
        schema: manips_schema,
        defaults: manips_defaults,
        read: manips_read,
        build: manips_build,
        anchors: manips_anchors,
        command: CommandMeta {
            workspace: "design",
            tab: "solid",
            group: "create",
            order: 901,
            pinned: false,
            icon_svg: &[],
            shortcut: None,
            keywords: &[],
        },
        new_component: None,
    }
}

fn manips_open() -> AppHarness {
    let (doc, _) = rect_doc();
    let mut h = AppHarness::builder()
        .size(egui::vec2(1400.0, 900.0))
        .build();
    let id = h.app_mut().add_document("ui06m".into(), doc);
    h.app_mut().activate(id);
    h.drive_until_idle();
    set_camera(&mut h, Point3::new(15.0, -170.0, 160.0), Point3::ZERO);
    h.run_command(UI06);
    h.drive_until_idle();
    assert_eq!(h.app().active_tool(), Some(UI06));
    h
}

#[test]
fn dw2_ctrl_drag_snaps_the_ring_to_5_degrees() {
    let mut h = manips_open();
    let Some(GizmoPrim::Ring {
        center,
        axis,
        ref_dir,
        ..
    }) = tool(&mut h).gizmo_of("angle")
    else {
        panic!("an angle ring");
    };
    assert!(center.distance(Point3::ZERO) < 1e-9 && axis == Dir3::Z);
    // The pointer shows the field's 30°.
    let a30 = Vec3::new(30f64.to_radians().cos(), 30f64.to_radians().sin(), 0.0);
    assert!(ref_dir.get().distance(a30) < 1e-9, "{ref_dir:?}");
    let cam = h.camera_view();
    let r = 0.5 * manip::world_size(&cam, center);
    let at = |deg: f64| {
        let a = deg.to_radians();
        Point3::new(r * a.cos(), r * a.sin(), 0.0)
    };
    let from = h.world_to_screen(at(30.0));
    let to = h.world_to_screen(at(67.0));
    drag_with(&mut h, egui::Modifiers::CTRL, from, to, "Heads-up Angle");
    assert_eq!(field_text(&mut h, "angle"), "65 deg", "30 + 37 snaps to 65");
    let from = h.world_to_screen(at(65.0));
    let to = h.world_to_screen(at(33.5));
    drag_with(&mut h, egui::Modifiers::NONE, from, to, "Heads-up Angle");
    assert_eq!(field_text(&mut h, "angle"), "33.5 deg");
    h.app_mut().finish_tool(FinishReason::Cancel);
}

#[test]
fn triad_and_plane_handle_drive_length_fields() {
    let mut h = manips_open();
    let cam = h.camera_view();
    // Triad: drag its z arrow (the anchor axis) up by 4 mm, then Ctrl-drag the xy square
    // (in-plane moves have no component along z: the value stays).
    let Some(GizmoPrim::Triad { origin, frame, .. }) = tool(&mut h).gizmo_of("lift") else {
        panic!("a triad");
    };
    let shown = manip::facing_frame(&frame, &cam);
    let size = manip::world_size(&cam, origin);
    let grab = TriadPart::AxisZ.grab_point(origin, &shown, size);
    assert_eq!(
        manip::hit_test(
            &tool(&mut h).manipulators().gizmos(),
            &cam,
            h.world_to_screen(grab)
        ),
        Some((1, GizmoPart::Triad(TriadPart::AxisZ)))
    );
    let (from, to) = (
        h.world_to_screen(grab),
        h.world_to_screen(grab + Vec3::Z * 4.0),
    );
    drag_with(&mut h, egui::Modifiers::NONE, from, to, "Heads-up Lift");
    assert_eq!(field_text(&mut h, "lift"), "4 mm");
    let Some(GizmoPrim::Triad { origin, .. }) = tool(&mut h).gizmo_of("lift") else {
        panic!("a triad");
    };
    assert!(origin.distance(Point3::new(40.0, 0.0, 4.0)) < 1e-6);
    // Plane handle: in-plane drag along its reference direction (+X).
    let Some(GizmoPrim::PlaneHandle { origin, .. }) = tool(&mut h).gizmo_of("slide") else {
        panic!("a plane handle");
    };
    let (from, to) = (
        h.world_to_screen(origin),
        h.world_to_screen(origin + Vec3::new(6.0, 3.0, 0.0)),
    );
    drag_with(&mut h, egui::Modifiers::NONE, from, to, "Heads-up Slide");
    assert_eq!(field_text(&mut h, "slide"), "6 mm");
    h.app_mut().finish_tool(FinishReason::Cancel);
}

// ---- Done-when 3: triad hit tests in 6 views ----------------------------------------------------

fn view(eye: Point3, target: Point3, ortho: bool, ppp: f32) -> CameraView {
    let dist = eye.distance(target);
    CameraView::new(
        Camera {
            eye,
            target,
            up: Dir3::Z,
            projection: if ortho {
                Projection::Orthographic {
                    height: dist * 0.75,
                }
            } else {
                Projection::Perspective { fov_y: 0.8 }
            },
        },
        egui::Rect::from_min_size(egui::pos2(40.0, 30.0), egui::vec2(1000.0, 700.0)),
        ppp,
    )
}

#[test]
fn dw3_each_triad_part_hit_tests_as_itself_in_six_views() {
    let origin = Point3::new(5.0, -3.0, 2.0);
    let tilted = Mat3::from_quat(Quat::from_axis_angle(
        Vec3::new(1.0, 2.0, 0.5).normalize(),
        0.6,
    ));
    let views = [
        (
            "iso +++",
            view(origin + Vec3::new(100.0, 100.0, 100.0), origin, false, 1.0),
            Mat3::IDENTITY,
        ),
        (
            "iso -+- ortho",
            view(origin + Vec3::new(-90.0, 120.0, -70.0), origin, true, 1.0),
            Mat3::IDENTITY,
        ),
        (
            "front-left high",
            view(origin + Vec3::new(-60.0, -150.0, 90.0), origin, false, 1.5),
            Mat3::IDENTITY,
        ),
        (
            "back-right low",
            view(origin + Vec3::new(140.0, 80.0, -50.0), origin, false, 1.0),
            Mat3::IDENTITY,
        ),
        (
            "tilted frame",
            view(origin + Vec3::new(70.0, -110.0, 130.0), origin, false, 1.0),
            tilted,
        ),
        (
            "tilted ortho off-centre",
            view(
                origin + Vec3::new(-120.0, -40.0, 60.0),
                origin + Vec3::new(8.0, 5.0, 0.0),
                true,
                2.0,
            ),
            tilted,
        ),
    ];
    let mut checked = 0;
    for (name, cam, frame) in &views {
        let g = GizmoPrim::Triad {
            origin,
            frame: *frame,
            id: 4,
            hot_part: None,
        };
        let shape = manip::shape(&g, cam);
        assert_eq!(
            shape.parts.len(),
            10,
            "{name}: every part is in front of the camera"
        );
        let shown = manip::facing_frame(frame, cam);
        for part in TriadPart::ALL {
            let p = cam
                .project(part.grab_point(origin, &shown, shape.world_size))
                .unwrap();
            assert_eq!(
                manip::hit_test(std::slice::from_ref(&g), cam, p),
                Some((4, GizmoPart::Triad(part))),
                "{name}: {part:?} at {p:?}"
            );
            checked += 1;
        }
        // Outside the tolerance: nothing.
        let c = cam.project(origin).unwrap();
        let far = c + egui::vec2(GIZMO_SIZE_PT + 3.0 * HIT_TOLERANCE_PT, GIZMO_SIZE_PT);
        assert_eq!(
            manip::hit_test(std::slice::from_ref(&g), cam, far),
            None,
            "{name}"
        );
    }
    assert_eq!(checked, 60);
    // Within the tolerance of an arm's tip still hits it; beyond it does not.
    let (_, cam, frame) = &views[0];
    let g = GizmoPrim::Triad {
        origin,
        frame: *frame,
        id: 4,
        hot_part: None,
    };
    let shape = manip::shape(&g, cam);
    let PartForm::Polyline(arm) = shape.part(GizmoPart::Triad(TriadPart::AxisX)).unwrap() else {
        panic!("an axis is a polyline");
    };
    let (a, b) = (arm[0], arm[1]);
    let out = (b - a).normalized();
    assert_eq!(
        manip::hit_test(
            std::slice::from_ref(&g),
            cam,
            b + out * (HIT_TOLERANCE_PT - 1.0)
        ),
        Some((4, GizmoPart::Triad(TriadPart::AxisX)))
    );
    assert_eq!(
        manip::hit_test(
            std::slice::from_ref(&g),
            cam,
            b + out * (HIT_TOLERANCE_PT + 1.0)
        ),
        None
    );
}

#[test]
fn triad_drags_move_and_rotate() {
    let origin = Point3::ZERO;
    let cam = view(Point3::new(100.0, 80.0, 120.0), origin, false, 1.0);
    let g = GizmoPrim::Triad {
        origin,
        frame: Mat3::IDENTITY,
        id: 0,
        hot_part: None,
    };
    let s = manip::world_size(&cam, origin);
    let shown = manip::facing_frame(&Mat3::IDENTITY, &cam);
    let drag = |part: TriadPart, to: Point3| {
        let from = cam.project(part.grab_point(origin, &shown, s)).unwrap();
        let mut d = manip::GizmoDrag::begin(&g, GizmoPart::Triad(part), &cam, from).unwrap();
        d.update(&cam, cam.project(to).unwrap()).unwrap()
    };
    let p = TriadPart::AxisY.grab_point(origin, &shown, s);
    match drag(TriadPart::AxisY, p + Vec3::Y * 3.0) {
        manip::DragValue::Move(v) => assert!(v.distance(Vec3::Y * 3.0) < 1e-3, "{v:?}"),
        other => panic!("{other:?}"),
    }
    let p = TriadPart::PlaneZX.grab_point(origin, &shown, s);
    match drag(TriadPart::PlaneZX, p + Vec3::new(2.0, 0.0, -1.0)) {
        manip::DragValue::Move(v) => {
            assert!(v.distance(Vec3::new(2.0, 0.0, -1.0)) < 1e-3, "{v:?}");
        }
        other => panic!("{other:?}"),
    }
    // RotZ: a quarter arc in xy; 20° further round it.
    let r = 0.75 * s;
    let to = Point3::new(
        r * 65f64.to_radians().cos(),
        r * 65f64.to_radians().sin(),
        0.0,
    );
    match drag(TriadPart::RotZ, to) {
        manip::DragValue::Angle(a) => assert!((a.to_degrees() - 20.0).abs() < 0.05, "{a}"),
        other => panic!("{other:?}"),
    }
    match drag(TriadPart::Free, Point3::ZERO) {
        manip::DragValue::Move(v) => assert!(v.length() < 1e-6),
        other => panic!("{other:?}"),
    }
}

// ---- Done-when 4: constant screen size ---------------------------------------------------------

/// Screen sizes of an arrow (length), a ring (diameter) and a triad arm (tip to centre), with the
/// view looking along +Y at `target` from `dist`.
fn sizes(dist: f64, ortho: bool, at: Point3) -> [f32; 3] {
    let target = Point3::new(0.0, 0.0, 0.0);
    let cam = CameraView::new(
        Camera {
            eye: target - Vec3::Y * dist,
            target,
            up: Dir3::Z,
            projection: if ortho {
                Projection::Orthographic { height: dist }
            } else {
                Projection::Perspective { fov_y: 0.8 }
            },
        },
        egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(1200.0, 800.0)),
        1.25,
    );
    let arrow = GizmoPrim::Arrow {
        origin: at,
        dir: Dir3::X,
        id: 0,
        hot: false,
    };
    let PartForm::Polyline(a) = manip::shape(&arrow, &cam)
        .part(GizmoPart::Arrow)
        .unwrap()
        .clone()
    else {
        panic!()
    };
    let ring = GizmoPrim::Ring {
        center: at,
        axis: Dir3::Y,
        ref_dir: Dir3::X,
        id: 1,
        hot: false,
    };
    let PartForm::Polyline(r) = manip::shape(&ring, &cam)
        .part(GizmoPart::Ring)
        .unwrap()
        .clone()
    else {
        panic!()
    };
    let n = r.len() - 1;
    let diameter = (0..n / 2)
        .map(|i| r[i].distance(r[i + n / 2]))
        .fold(0.0_f32, f32::max);
    let triad = GizmoPrim::Triad {
        origin: at,
        frame: Mat3::IDENTITY,
        id: 2,
        hot_part: None,
    };
    let t = manip::shape(&triad, &cam);
    let PartForm::Polyline(arm) = t.part(GizmoPart::Triad(TriadPart::AxisZ)).unwrap().clone()
    else {
        panic!()
    };
    let c = cam.project(at).unwrap();
    [a[0].distance(a[1]), diameter, c.distance(arm[1])]
}

#[test]
fn dw4_gizmo_screen_size_stays_90_pt_over_a_1000x_zoom() {
    let mut worst = 0.0_f32;
    for ortho in [false, true] {
        for k in 0..=12 {
            // 20 mm to 20 m in 13 steps (1000×).
            let dist = 20.0 * 10f64.powf(f64::from(k) * 0.25);
            for at in [Point3::ZERO, Point3::new(0.15 * dist, 0.0, -0.1 * dist)] {
                for s in sizes(dist, ortho, at) {
                    worst = worst.max((s - GIZMO_SIZE_PT).abs());
                    assert!(
                        (s - GIZMO_SIZE_PT).abs() <= 2.0,
                        "{s} pt at {dist} mm (ortho {ortho}, at {at:?})"
                    );
                }
            }
        }
    }
    println!("UI-06 gizmo size: worst deviation from 90 pt over 1000x zoom: {worst} pt");
}

#[test]
fn dw4_the_extrude_arrow_keeps_its_screen_size_in_the_app() {
    let mut h = extrude_open();
    let mut lengths = Vec::new();
    for dist in [60.0, 1900.0, 60_000.0] {
        // From the side: the arrow (+Z) is across the view.
        set_camera(
            &mut h,
            Point3::new(10.0, 5.0 - dist, 10.0),
            Point3::new(10.0, 5.0, 10.0),
        );
        let cam = h.camera_view();
        let g = tool(&mut h).gizmo_of("distance").unwrap();
        let PartForm::Polyline(p) = manip::shape(&g, &cam)
            .part(GizmoPart::Arrow)
            .unwrap()
            .clone()
        else {
            panic!()
        };
        lengths.push(p[0].distance(p[1]));
    }
    println!("UI-06 extrude arrow length at 60 mm / 1.9 m / 60 m: {lengths:?} pt");
    for l in lengths {
        assert!((l - GIZMO_SIZE_PT).abs() <= 2.0, "{l}");
    }
    h.app_mut().finish_tool(FinishReason::Cancel);
}
