//! ASM-02: the Move / Copy tool for occurrences, the Position panel and the Capture / Revert /
//! Cancel prompt (`amg_ui::assembly::{moving, position}`) in the real app (`AppHarness`,
//! kittest, CPU scene backend and pick service).
//!
//! The frame-time check of Done-when 5 asserts < 16 ms only under `AMG_STRICT_TIMING=1`; it
//! always prints its numbers.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    missing_docs,
    reason = "test code"
)]

use std::borrow::Cow;
use std::f64::consts::FRAC_PI_2;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use amg_core::{Dir3, Mat3, Point3, Transform, Vec3};
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, Command, ComponentId, Dependency, Design,
    Feature, FeatureBox, FeatureError, FeatureId, FeatureOutput, OccurrenceId, RecomputeCtx,
    register_feature,
};
use amg_mesh::IndexedMesh;
use amg_render::{Camera, Projection};
use amg_ui::CommitBatch;
use amg_ui::assembly::moving::field_labels;
use amg_ui::assembly::{
    AssemblyLayer, MoveMode, MoveTool, PositionPanel, PositionPrompt, cmd, guard_tool_start,
    position_labels,
};
use amg_ui::command::{CmdAction, CmdId, CommandDef};
use amg_ui::input::ViewportEvent;
use amg_ui::manip::{self, DragValue, TriadPart};
use amg_ui::module::{Registrar, UiModule};
use amg_ui::pick::PickTarget;
use amg_ui::select::SelItem;
use amg_ui::testing::AppHarness;
use amg_ui::testing::egui_kittest::kittest::{NodeT, Queryable};
use amg_ui::tool::{FinishReason, InputResult, Tool, ToolCx};
use egui::accesskit::Role;
use serde::{Deserialize, Serialize};

// ---- fixtures ---------------------------------------------------------------------------------

/// A 10 mm tetrahedron mesh body at x = 20·serial: vertices (x,0,0), (x+10,0,0), (x,10,0),
/// (x,0,10).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Tet {
    serial: u32,
}

fn tet_vertices(serial: u32) -> [Point3; 4] {
    let x = 20.0 * f64::from(serial);
    [
        Point3::new(x, 0.0, 0.0),
        Point3::new(x + 10.0, 0.0, 0.0),
        Point3::new(x, 10.0, 0.0),
        Point3::new(x, 0.0, 10.0),
    ]
}

impl Feature for Tet {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let tris = vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]];
        let mesh = IndexedMesh::new(tet_vertices(self.serial).to_vec(), tris).unwrap();
        Ok(FeatureOutput {
            bodies: vec![BodyChange {
                id: ctx.new_body_id(0),
                change: BodyChangeKind::Created {
                    geom: BodyGeom::Mesh(Arc::new(mesh)),
                    naming: None,
                },
            }],
            ..FeatureOutput::default()
        })
    }
}

register_feature!(Tet, "uitest.asm02_tet", version = 1);

// A stand-in for ASM-11's Joint Origin: a position-dependent tool that asks the guard first.
const JOINT_ORIGIN: CmdId = CmdId("uitest.asm02_joint_origin");

struct JointOriginStandIn;

impl Tool for JointOriginStandIn {
    fn id(&self) -> CmdId {
        JOINT_ORIGIN
    }
    fn label(&self) -> Cow<'static, str> {
        "Joint Origin".into()
    }
    fn start(&mut self, cx: &mut ToolCx<'_>) {
        guard_tool_start(cx, JOINT_ORIGIN);
    }
    fn input(&mut self, _ev: &ViewportEvent, _cx: &mut ToolCx<'_>) -> InputResult {
        InputResult::Ignored
    }
    fn finish(&mut self, _reason: FinishReason, _cx: &mut ToolCx<'_>) -> Option<CommitBatch> {
        None
    }
}

fn register_test_module(r: &mut Registrar<'_>) {
    r.command(CommandDef::new(
        JOINT_ORIGIN.0,
        "Joint Origin",
        CmdAction::StartTool(|_| Box::new(JointOriginStandIn)),
    ));
}

inventory::submit! { UiModule { id: "uitest_asm02", register: register_test_module } }

fn harness() -> AppHarness {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    h
}

fn commit(h: &mut AppHarness, commands: Vec<Command>) {
    h.app_mut().submit(CommitBatch {
        label: "Fixture".into(),
        commands,
    });
    h.drive_until_idle();
}

fn design(h: &AppHarness) -> &Design {
    h.document().design()
}

fn root(h: &AppHarness) -> ComponentId {
    design(h).root()
}

/// Adds a component named `name` in `parent` at `transform`; returns it and its occurrence.
fn component(
    h: &mut AppHarness,
    parent: ComponentId,
    name: &str,
    transform: Transform,
) -> (ComponentId, OccurrenceId) {
    commit(
        h,
        vec![Command::NewComponent {
            parent,
            name: name.into(),
            transform,
        }],
    );
    design(h)
        .occurrences()
        .find(|(_, o)| {
            design(h)
                .component(o.component)
                .is_some_and(|c| c.name == name)
        })
        .map(|(id, o)| (o.component, id))
        .expect("the new component")
}

fn component_in_root(
    h: &mut AppHarness,
    name: &str,
    transform: Transform,
) -> (ComponentId, OccurrenceId) {
    let r = root(h);
    component(h, r, name, transform)
}

fn tet_in_root(h: &mut AppHarness, serial: u32) -> (FeatureId, BodyId) {
    let r = root(h);
    tet(h, r, serial)
}

fn tet(h: &mut AppHarness, context: ComponentId, serial: u32) -> (FeatureId, BodyId) {
    commit(
        h,
        vec![Command::AddFeature {
            at: None,
            feature: FeatureBox::new(Tet { serial }),
            context,
            name: None,
        }],
    );
    let f = design(h).timeline().nodes().last().unwrap().id;
    let b = h
        .document()
        .evaluated()
        .bodies()
        .map(|(id, _)| id)
        .max()
        .expect("a body");
    (f, b)
}

fn select_occurrences(h: &mut AppHarness, occs: &[OccurrenceId], body: BodyId) {
    let items = occs
        .iter()
        .map(|&o| SelItem {
            target: PickTarget::Occurrence(o),
            entity: amg_doc::EntityRef::InContext {
                path: vec![o],
                target: Box::new(amg_doc::EntityRef::Body(body)),
            },
        })
        .collect();
    h.app_mut().active_document_mut().selection.set(items);
    h.drive_until_idle();
}

fn undo_depth(h: &AppHarness) -> usize {
    h.document().history().stack().applied()
}

fn look(h: &mut AppHarness, target: Point3, from: Vec3, dist: f64) {
    let dir = from.normalize();
    let up = if dir.z.abs() > 0.9 { Dir3::Y } else { Dir3::Z };
    h.app_mut().active_document_mut().camera = Camera {
        eye: target + dir * dist,
        target,
        up,
        projection: Projection::Perspective { fov_y: 0.8 },
    };
    h.drive_until_idle();
}

fn start_move(h: &mut AppHarness) {
    h.run_command(cmd::MOVE);
    h.drive_until_idle();
    assert_eq!(h.app().active_tool(), Some(cmd::MOVE), "Move/Copy started");
}

fn tool(h: &mut AppHarness) -> &mut MoveTool {
    h.app_mut()
        .active_tool_as::<MoveTool>()
        .expect("the move tool")
}

fn ok(h: &mut AppHarness) {
    h.app_mut().finish_tool(FinishReason::Commit);
    h.drive_until_idle();
}

/// World transform of occurrence path `path` in the evaluated placements.
fn world(h: &AppHarness, path: &[OccurrenceId]) -> Transform {
    h.document()
        .evaluated()
        .placements()
        .world(path)
        .expect("a valid path")
}

fn assert_close(a: Transform, b: Transform, tol: f64, what: &str) {
    assert!(a.abs_diff_eq(b, tol), "{what}: {a:?} != {b:?}");
}

fn buttons(h: &mut AppHarness, label: &str) -> usize {
    h.kittest()
        .query_all_by_role_and_label(Role::Button, label)
        .count()
}

fn click_button(h: &mut AppHarness, label: &str) {
    h.kittest()
        .get_by_role_and_label(Role::Button, label)
        .click();
    h.drive_until_idle();
}

fn json(d: &Design) -> serde_json::Value {
    serde_json::to_value(d).unwrap()
}

/// A design with component A (one tet) placed at a rotated, translated spot.
fn one_component(h: &mut AppHarness) -> (OccurrenceId, BodyId) {
    let t =
        Transform::from_translation(Vec3::new(5.0, -3.0, 2.0)) * Transform::from_rotation_x(0.3);
    let (c, o) = component_in_root(h, "A", t);
    let (_, b) = tet(h, c, 0);
    (o, b)
}

/// A scripted move of 12.5 mm along X committed as one pending move.
fn scripted_move(h: &mut AppHarness, o: OccurrenceId, b: BodyId, dx: f64) {
    select_occurrences(h, &[o], b);
    start_move(h);
    tool(h).drag_part(TriadPart::AxisX, DragValue::Move(Vec3::X * dx), false);
    ok(h);
}

// ---- Done-when 1 ------------------------------------------------------------------------------

#[test]
fn dw1_scripted_triad_drag_gives_the_exact_pending_transform_in_one_undo_entry() {
    let mut h = harness();
    // A at a rotated spot, B inside A at another, the tet in B: the move goes through the
    // parent's world transform.
    let ta =
        Transform::from_translation(Vec3::new(10.0, 20.0, 30.0)) * Transform::from_rotation_x(0.3);
    let tb =
        Transform::from_translation(Vec3::new(5.0, 0.0, 0.0)) * Transform::from_rotation_z(0.2);
    let (a_comp, a) = component_in_root(&mut h, "A", ta);
    let (b_comp, b) = component(&mut h, a_comp, "B", tb);
    let (_, body) = tet(&mut h, b_comp, 0);
    select_occurrences(&mut h, &[b], body);
    start_move(&mut h);
    let pivot = (ta * tb).translation;
    assert!(
        tool(&mut h).pivot().distance(pivot) < 1e-12,
        "triad at B's origin"
    );
    assert_eq!(tool(&mut h).targets(), vec![b]);

    let depth = undo_depth(&h);
    tool(&mut h).drag_part(TriadPart::AxisX, DragValue::Move(Vec3::X * 12.5), false);
    tool(&mut h).drag_part(TriadPart::RotZ, DragValue::Angle(FRAC_PI_2), false);
    // The fields show the move.
    let v = tool(&mut h).values();
    assert!(
        (v[0] - 12.5).abs() < 1e-12 && v[1].abs() < 1e-12 && v[2].abs() < 1e-12,
        "{v:?}"
    );
    assert!(
        (v[5] - FRAC_PI_2).abs() < 1e-12 && v[3].abs() < 1e-12 && v[4].abs() < 1e-12,
        "{v:?}"
    );
    // Nothing is committed while the tool runs; the scene shows the preview.
    assert_eq!(undo_depth(&h), depth);
    assert!(!design(&h).has_pending_positions());
    assert!(tool(&mut h).preview_placements().is_some());
    ok(&mut h);

    let moved = pivot + Vec3::X * 12.5;
    let delta = Transform::from_translation(moved)
        * Transform::from_rotation_z(FRAC_PI_2)
        * Transform::from_translation(-moved)
        * Transform::from_translation(Vec3::X * 12.5);
    let expected = ta.inverse() * delta * ta * tb;
    assert_eq!(undo_depth(&h), depth + 1, "one undo entry");
    assert_eq!(h.document().history().undo_label(), Some("Move"));
    let pending = design(&h).pending_positions().get(b).expect("b is pending");
    assert_close(pending, expected, 1e-12, "pending local transform");
    assert!(
        design(&h).pending_positions().get(a).is_none(),
        "A not moved"
    );
    // The evaluated placements follow (seeded from pending), the world point moved as asked.
    assert_close(
        world(&h, &[a, b]),
        delta * ta * tb,
        1e-12,
        "world placement",
    );
    // Undo restores the design.
    let id = h.app().active_document().id;
    h.app_mut().request_undo(id);
    h.drive_until_idle();
    assert!(!design(&h).has_pending_positions());
}

#[test]
fn dw1_a_ctrl_drag_of_the_triad_arm_snaps_and_commits_once() {
    let mut h = harness();
    let (o, b) = one_component(&mut h);
    select_occurrences(&mut h, &[o], b);
    start_move(&mut h);
    let origin = tool(&mut h).pivot();
    // Seen from the +X+Y+Z octant no triad axis is flipped.
    look(&mut h, origin, Vec3::new(1.0, 0.8, 0.6), 150.0);
    let cam = h.camera_view();
    let size = manip::world_size(&cam, origin);
    let grab = TriadPart::AxisX.grab_point(origin, &Mat3::IDENTITY, size);
    let from = h.world_to_screen(grab);
    let to = h.world_to_screen(grab + Vec3::X * 12.4);
    let depth = undo_depth(&h);
    let before = world(&h, &[o]);
    h.drag(
        egui::PointerButton::Primary,
        egui::Modifiers::CTRL,
        from,
        to,
        8,
    );
    h.drive_until_idle();
    let v = tool(&mut h).values();
    assert!(
        (v[0] - 12.0).abs() < 1e-12,
        "Ctrl snaps 12.4 mm to 12 mm: {v:?}"
    );
    assert_eq!(undo_depth(&h), depth, "a drag alone commits nothing");
    // The field shows it; OK commits one pending move.
    let field = h
        .kittest()
        .query_all_by_role_and_label(Role::TextInput, field_labels::X)
        .next()
        .expect("the X field")
        .accesskit_node()
        .value();
    assert_eq!(field.as_deref(), Some("12 mm"));
    click_button(&mut h, "OK");
    assert_eq!(undo_depth(&h), depth + 1);
    assert_close(
        world(&h, &[o]),
        Transform::from_translation(Vec3::X * 12.0) * before,
        1e-12,
        "moved 12 mm along X",
    );
}

#[test]
fn typing_a_field_moves_and_direct_mode_commits_set_occurrence_transforms() {
    let mut h = harness();
    let (o, b) = one_component(&mut h);
    let before = world(&h, &[o]);
    select_occurrences(&mut h, &[o], b);
    start_move(&mut h);
    tool(&mut h).set_values([0.0, 0.0, 7.0, 0.0, 0.0, 0.0]);
    ok(&mut h);
    assert_close(
        world(&h, &[o]),
        Transform::from_translation(Vec3::Z * 7.0) * before,
        1e-12,
        "Z field",
    );
    // Direct mode: the move is placed at once, nothing pending.
    // (Switching to Direct mode bakes the bodies, so it goes through `Document::apply`.)
    h.app_mut()
        .active_document_mut()
        .document
        .apply(Command::SetDesignMode {
            mode: amg_doc::DesignMode::Direct,
        })
        .unwrap();
    h.drive_until_idle();
    assert!(
        !design(&h).has_pending_positions(),
        "Direct mode folds pending moves"
    );
    let before = world(&h, &[o]);
    start_move(&mut h);
    tool(&mut h).drag_part(TriadPart::AxisY, DragValue::Move(Vec3::Y * 3.0), false);
    ok(&mut h);
    assert!(!design(&h).has_pending_positions());
    assert_eq!(h.document().history().undo_label(), Some("Move"));
    assert_close(
        design(&h).occurrence(o).unwrap().transform,
        Transform::from_translation(Vec3::Y * 3.0) * before,
        1e-12,
        "base transform",
    );
}

#[test]
fn create_copy_adds_an_occurrence_at_the_moved_place() {
    let mut h = harness();
    let (o, b) = one_component(&mut h);
    let before = world(&h, &[o]);
    let n = design(&h).occurrences().count();
    select_occurrences(&mut h, &[o], b);
    start_move(&mut h);
    tool(&mut h).set_copy(true);
    tool(&mut h).drag_part(TriadPart::AxisX, DragValue::Move(Vec3::X * 40.0), false);
    ok(&mut h);
    assert_eq!(design(&h).occurrences().count(), n + 1);
    assert!(!design(&h).has_pending_positions());
    assert_close(world(&h, &[o]), before, 1e-12, "the original stays");
    let copy = design(&h).occurrences().map(|(id, _)| id).max().unwrap();
    assert_close(
        world(&h, &[copy]),
        Transform::from_translation(Vec3::X * 40.0) * before,
        1e-12,
        "the copy",
    );
}

// ---- Done-when 2 ------------------------------------------------------------------------------

#[test]
fn dw2_a_grounded_occurrence_cannot_be_moved_and_the_tool_says_why() {
    let mut h = harness();
    let (o, b) = one_component(&mut h);
    commit(
        &mut h,
        vec![Command::SetOccurrenceGrounded {
            occ: o,
            grounded: true,
        }],
    );
    select_occurrences(&mut h, &[o], b);
    start_move(&mut h);
    let why = "A is grounded and cannot be moved. Unground it first.";
    assert!(
        h.kittest().query_all_by_label(why).next().is_some(),
        "the dialog shows why"
    );
    // No triad is drawn, a script drag and OK commit nothing.
    assert!(h.app().tool_overlays().is_empty(), "no triad");
    let depth = undo_depth(&h);
    tool(&mut h).drag_part(TriadPart::AxisX, DragValue::Move(Vec3::X * 5.0), false);
    ok(&mut h);
    assert_eq!(undo_depth(&h), depth);
    assert!(!design(&h).has_pending_positions());
    assert!(design(&h).occurrence(o).unwrap().grounded);
}

// ---- Done-when 3 ------------------------------------------------------------------------------

#[test]
fn dw3_point_to_point_puts_a_vertex_on_a_vertex() {
    let mut h = harness();
    let (ca, a) = component_in_root(&mut h, "A", Transform::IDENTITY);
    let tb =
        Transform::from_translation(Vec3::new(0.0, 30.0, 5.0)) * Transform::from_rotation_z(0.25);
    let (cb, _b) = component_in_root(&mut h, "B", tb);
    let (_, body_a) = tet(&mut h, ca, 0);
    let (_, _body_b) = tet(&mut h, cb, 1);
    look(
        &mut h,
        Point3::new(12.0, 18.0, 6.0),
        Vec3::new(1.0, 1.0, 1.0),
        140.0,
    );
    select_occurrences(&mut h, &[a], body_a);
    start_move(&mut h);
    tool(&mut h).set_mode(MoveMode::PointToPoint);
    h.drive_until_idle();

    // Vertex 1 of A, clicked just inside the slanted face next to it.
    let va = tet_vertices(0);
    let from_world = va[1] * 0.97 + va[2] * 0.015 + va[3] * 0.015;
    let from_target = va[1];
    // Vertex 3 of B, seen through B's placement.
    let vb = tet_vertices(1);
    let to_target = tb.transform_point3(vb[3]);
    let to_world = tb.transform_point3(vb[1] * 0.015 + vb[2] * 0.015 + vb[3] * 0.97);
    let p = h.world_to_screen(from_world);
    h.click_at(p, egui::PointerButton::Primary, egui::Modifiers::NONE);
    h.drive_until_idle();
    let got = tool(&mut h).from_point().expect("the from point");
    assert!(
        got.distance(from_target) < 1e-12,
        "snapped to the vertex: {got:?}"
    );
    let p = h.world_to_screen(to_world);
    h.click_at(p, egui::PointerButton::Primary, egui::Modifiers::NONE);
    h.drive_until_idle();
    assert_eq!(
        tool(&mut h).targets(),
        vec![a],
        "point picks keep the target"
    );
    ok(&mut h);

    let moved = world(&h, &[a]).transform_point3(from_target);
    let err = moved.distance(to_target);
    eprintln!("ASM-02 point to point: vertex error {err:e} mm");
    assert!(err < 1e-9, "vertex on vertex: {moved:?} vs {to_target:?}");
}

// ---- Done-when 4 ------------------------------------------------------------------------------

#[test]
fn dw4_moving_recomputes_no_feature_in_the_moved_component() {
    let mut h = harness();
    let (ca, a) = component_in_root(&mut h, "A", Transform::IDENTITY);
    let (fa, body) = tet(&mut h, ca, 0);
    let (fb, _) = tet(&mut h, ca, 1);
    let key_before = h.document().evaluated().node(fa).map(|n| n.health.clone());
    scripted_move(&mut h, a, body, 12.5);
    assert!(design(&h).has_pending_positions());
    let run = h.document().last_run().expect("a recompute ran");
    let in_moved: Vec<FeatureId> = design(&h).features_in(ca).collect();
    assert_eq!(in_moved, vec![fa, fb]);
    assert!(
        run.computed.iter().all(|f| !in_moved.contains(f)),
        "computed {:?}",
        run.computed
    );
    eprintln!(
        "ASM-02 move: computed {} features, {} from the cache",
        run.computed.len(),
        run.cached.len()
    );
    assert_eq!(
        h.document().evaluated().node(fa).map(|n| n.health.clone()),
        key_before
    );
}

// ---- Done-when 5 ------------------------------------------------------------------------------

/// Records how long each `amg_ui.frame` span was entered (the UI thread's frame time).
#[derive(Clone, Default)]
struct FrameTimes(Arc<Mutex<Vec<Duration>>>);

struct Entered(Instant);

impl<S> tracing_subscriber::Layer<S> for FrameTimes
where
    S: tracing::Subscriber + for<'a> tracing_subscriber::registry::LookupSpan<'a>,
{
    fn on_enter(&self, id: &tracing::span::Id, ctx: tracing_subscriber::layer::Context<'_, S>) {
        if let Some(span) = ctx.span(id)
            && span.name() == "amg_ui.frame"
        {
            span.extensions_mut().replace(Entered(Instant::now()));
        }
    }

    fn on_exit(&self, id: &tracing::span::Id, ctx: tracing_subscriber::layer::Context<'_, S>) {
        if let Some(span) = ctx.span(id)
            && let Some(Entered(t)) = span.extensions().get::<Entered>()
        {
            self.0.lock().unwrap().push(t.elapsed());
        }
    }
}

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").as_deref() == Ok("1")
}

#[test]
fn dw5_frame_time_while_dragging_100_occurrences() {
    use tracing_subscriber::layer::SubscriberExt as _;

    let mut h = harness();
    let (c, first) = component_in_root(&mut h, "Part", Transform::IDENTITY);
    let (_, body) = tet(&mut h, c, 0);
    // 99 more placements on a 10 × 10 grid, 15 mm apart.
    let adds = (1..100)
        .map(|i| Command::AddOccurrence {
            parent: root(&h),
            component: c,
            transform: Transform::from_translation(Vec3::new(
                15.0 * f64::from(i % 10),
                15.0 * f64::from(i / 10),
                0.0,
            )),
            name: None,
        })
        .collect();
    commit(&mut h, adds);
    let occs: Vec<OccurrenceId> = design(&h).occurrences().map(|(id, _)| id).collect();
    assert_eq!(occs.len(), 100);
    assert_eq!(
        h.scene().instance_slots(body).len(),
        100,
        "100 instances on screen"
    );
    look(
        &mut h,
        Point3::new(70.0, 70.0, 0.0),
        Vec3::new(1.0, 0.8, 0.9),
        420.0,
    );
    select_occurrences(&mut h, &occs, body);
    start_move(&mut h);
    assert_eq!(tool(&mut h).targets().len(), 100);
    let origin = tool(&mut h).pivot();
    assert!(origin.distance(world(&h, &[first]).translation) < 1e-12);
    let cam = h.camera_view();
    let size = manip::world_size(&cam, origin);
    let grab = TriadPart::AxisX.grab_point(origin, &Mat3::IDENTITY, size);
    let from = h.world_to_screen(grab);
    let to = h.world_to_screen(grab + Vec3::X * 30.0);

    let times = FrameTimes::default();
    let subscriber = tracing_subscriber::registry().with(times.clone());
    let guard = tracing::subscriber::set_default(subscriber);
    let t0 = Instant::now();
    h.drag(
        egui::PointerButton::Primary,
        egui::Modifiers::NONE,
        from,
        to,
        40,
    );
    let total = t0.elapsed();
    drop(guard);
    h.drive_until_idle();
    let moved = tool(&mut h).values()[0];
    assert!(moved > 20.0, "the drag moved the triad: {moved}");
    // The preview moved every instance without a recompute.
    let preview = tool(&mut h)
        .preview_placements()
        .cloned()
        .expect("a preview");
    assert!(
        (preview.world(&[first]).unwrap().translation.x - moved).abs() < 1e-9,
        "the preview follows"
    );

    let mut t = times.0.lock().unwrap().clone();
    t.sort();
    let max = *t.last().unwrap();
    let p50 = t[t.len() / 2];
    let p95 = t[t.len() * 95 / 100];
    let over = t.iter().filter(|d| **d > Duration::from_millis(16)).count();
    eprintln!(
        "ASM-02 drag of 100 occurrences: {} frames in {total:?}; amg_ui.frame p50 {p50:?}, p95 {p95:?}, max {max:?}, {over} over 16 ms",
        t.len()
    );
    if strict_timing() {
        assert!(max < Duration::from_millis(16), "max frame {max:?}");
    }
    ok(&mut h);
    assert_eq!(design(&h).pending_positions().moves().count(), 100);
}

// ---- Done-when 6 ------------------------------------------------------------------------------

#[test]
fn dw6_position_panel_captures_and_reverts() {
    let mut h = harness();
    let (o, b) = one_component(&mut h);
    assert_eq!(buttons(&mut h, position_labels::REVERT), 0, "no panel yet");
    scripted_move(&mut h, o, b, 12.5);

    // After the move the panel shows.
    assert!(h.app().dialog::<PositionPanel>().is_some());
    assert!(
        h.app()
            .doc_layer::<AssemblyLayer>()
            .unwrap()
            .position_panel_open()
    );
    assert_eq!(buttons(&mut h, position_labels::CAPTURE), 1);
    assert_eq!(buttons(&mut h, position_labels::REVERT), 1);
    let nodes = design(&h).timeline().nodes().count();
    let moved = world(&h, &[o]);

    // Capture Position: one timeline node, the panel goes away, the place stays.
    click_button(&mut h, position_labels::CAPTURE);
    assert_eq!(design(&h).timeline().nodes().count(), nodes + 1);
    assert_eq!(
        design(&h).timeline().nodes().last().unwrap().feature.kind(),
        amg_doc::positions::CAPTURE_POSITION_KIND
    );
    assert!(!design(&h).has_pending_positions());
    assert!(h.app().dialog::<PositionPanel>().is_none());
    assert_eq!(buttons(&mut h, position_labels::REVERT), 0, "panel hidden");
    assert_close(world(&h, &[o]), moved, 1e-12, "captured where it was moved");

    // Another move, then Revert: back to the captured place, the panel goes away.
    scripted_move(&mut h, o, b, -4.0);
    assert_eq!(buttons(&mut h, position_labels::REVERT), 1);
    assert!(world(&h, &[o]).translation.distance(moved.translation) > 3.9);
    click_button(&mut h, position_labels::REVERT);
    assert!(!design(&h).has_pending_positions());
    assert_close(world(&h, &[o]), moved, 1e-12, "reverted");
    assert_close(design(&h).seed_transform(o).unwrap(), moved, 1e-12, "seed");
    assert_eq!(buttons(&mut h, position_labels::REVERT), 0, "panel hidden");
    assert_eq!(design(&h).timeline().nodes().count(), nodes + 1);
}

// ---- Done-when 7 ------------------------------------------------------------------------------

fn drag_marker_before(h: &mut AppHarness, node: FeatureId) {
    let marker = h.app().timeline().marker_rect().expect("marker drawn");
    let rect = h
        .app()
        .timeline()
        .drawn_node(node)
        .expect("node drawn")
        .rect;
    let to = egui::pos2(rect.left() - 2.0, marker.center().y);
    let from = marker.center();
    h.hover_at(from);
    h.button(
        from,
        egui::PointerButton::Primary,
        true,
        egui::Modifiers::NONE,
    );
    for i in 1..=6_u8 {
        let t = f32::from(i) / 6.0;
        h.hover_at(from + (to - from) * t);
    }
    h.hover_at(to);
    h.button(
        to,
        egui::PointerButton::Primary,
        false,
        egui::Modifiers::NONE,
    );
    h.drive_until_idle();
}

fn prompt_shown(h: &mut AppHarness) -> bool {
    h.app().dialog::<PositionPrompt>().is_some()
        && h.kittest()
            .query_all_by_role_and_label(Role::Button, position_labels::CANCEL)
            .next()
            .is_some()
}

#[test]
fn dw7_the_marker_drag_asks_and_each_answer_does_what_it_says() {
    let mut h = harness();
    let (ca, a) = component_in_root(&mut h, "A", Transform::IDENTITY);
    let (_, body) = tet(&mut h, ca, 0);
    let (_, _) = tet_in_root(&mut h, 1);
    let (last, _) = tet_in_root(&mut h, 2);
    let n = design(&h).timeline().nodes().count();
    assert_eq!(design(&h).timeline().marker(), n);
    scripted_move(&mut h, a, body, 12.5);
    let before = json(design(&h));
    let depth = undo_depth(&h);

    // Cancel: design, pending and marker unchanged.
    drag_marker_before(&mut h, last);
    assert!(prompt_shown(&mut h), "the marker drag asks first");
    assert_eq!(design(&h).timeline().marker(), n, "not moved yet");
    click_button(&mut h, position_labels::CANCEL);
    assert!(!prompt_shown(&mut h));
    assert_eq!(json(design(&h)), before, "Cancel changes nothing");
    assert_eq!(undo_depth(&h), depth);
    assert!(design(&h).has_pending_positions());

    // Revert: pending cleared, then the marker moves.
    drag_marker_before(&mut h, last);
    assert!(prompt_shown(&mut h));
    click_button(&mut h, position_labels::REVERT);
    h.drive_until_idle();
    assert!(!prompt_shown(&mut h));
    assert!(!design(&h).has_pending_positions());
    assert_eq!(
        design(&h).timeline().marker(),
        n - 1,
        "then the marker moved"
    );
    assert_eq!(design(&h).timeline().nodes().count(), n, "no capture node");
    assert_close(world(&h, &[a]), Transform::IDENTITY, 1e-12, "reverted");

    // Without pending moves the marker moves at once.
    let end = design(&h).timeline().nodes().count();
    commit(&mut h, vec![Command::SetMarker { index: end }]);
    scripted_move(&mut h, a, body, 6.0);
    let moved = world(&h, &[a]);

    // Capture: one capture node, then the marker moves where it was dragged.
    drag_marker_before(&mut h, last);
    assert!(prompt_shown(&mut h));
    click_button(&mut h, position_labels::CAPTURE);
    h.drive_until_idle();
    assert!(!prompt_shown(&mut h));
    assert!(!design(&h).has_pending_positions());
    assert_eq!(
        design(&h).timeline().nodes().count(),
        n + 1,
        "a capture node"
    );
    assert_eq!(
        design(&h).timeline().marker(),
        n - 1,
        "marker where it was dragged"
    );
    // Rolling forward again shows the captured place.
    let end = design(&h).timeline().nodes().count();
    commit(&mut h, vec![Command::SetMarker { index: end }]);
    assert_close(world(&h, &[a]), moved, 1e-12, "captured");
}

#[test]
fn dw7_joint_origin_with_a_pending_move_asks_first() {
    let mut h = harness();
    let (o, b) = one_component(&mut h);
    scripted_move(&mut h, o, b, 12.5);
    let before = json(design(&h));

    // Cancel: no tool, nothing changed.
    h.run_command(JOINT_ORIGIN);
    h.drive_until_idle();
    assert!(prompt_shown(&mut h), "Joint Origin asks first");
    assert_eq!(h.app().active_tool(), None, "the tool waits for the answer");
    click_button(&mut h, position_labels::CANCEL);
    assert!(!prompt_shown(&mut h));
    assert_eq!(h.app().active_tool(), None);
    assert_eq!(json(design(&h)), before);

    // Capture: a capture node, then Joint Origin starts.
    let nodes = design(&h).timeline().nodes().count();
    h.run_command(JOINT_ORIGIN);
    h.drive_until_idle();
    assert!(prompt_shown(&mut h));
    click_button(&mut h, position_labels::CAPTURE);
    h.drive_until_idle();
    assert!(!design(&h).has_pending_positions());
    assert_eq!(design(&h).timeline().nodes().count(), nodes + 1);
    assert_eq!(
        h.app().active_tool(),
        Some(JOINT_ORIGIN),
        "then the tool runs"
    );
    h.app_mut().finish_tool(FinishReason::Cancel);
    h.drive_until_idle();

    // Revert: back, then the tool starts; without pending moves it starts at once.
    scripted_move(&mut h, o, b, 3.0);
    h.run_command(JOINT_ORIGIN);
    h.drive_until_idle();
    assert!(prompt_shown(&mut h));
    click_button(&mut h, position_labels::REVERT);
    h.drive_until_idle();
    assert!(!design(&h).has_pending_positions());
    assert_eq!(h.app().active_tool(), Some(JOINT_ORIGIN));
    h.app_mut().finish_tool(FinishReason::Cancel);
    h.run_command(JOINT_ORIGIN);
    h.drive_until_idle();
    assert!(!prompt_shown(&mut h));
    assert_eq!(h.app().active_tool(), Some(JOINT_ORIGIN));
}
