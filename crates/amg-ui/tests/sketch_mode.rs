//! UI-09 sketch mode: Done-when 1-5 plus the session's other behaviour (plane pick, palette,
//! Esc, Ctrl, heads-up input, tangent-arc drag-back). Kittest on the real app with the
//! `NullViewport` and `CpuPickService`.

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use amg_core::{Dir3, Point2, Point3};
use amg_doc::{FeatureId, Health};
use amg_render::{Camera, Projection};
use amg_sketch::solve::ConstraintState;
use amg_sketch::{ConstraintKind, DimensionKind, EntityKind, HvTarget, Sketch};
use amg_ui::design::SKETCH_CONTEXT;
use amg_ui::sketch::{
    CREATE_SKETCH_CMD, FINISH_SKETCH_CMD, LINE_CMD, SKETCH_SELECT_CMD, SketchLayer, SketchSession,
};
use amg_ui::testing::AppHarness;
use amg_ui::testing::egui_kittest::kittest::Queryable as _;

const PRIM: egui::PointerButton = egui::PointerButton::Primary;

fn top_down(x: f64, y: f64, height: f64) -> Camera {
    Camera {
        eye: Point3::new(x, y, height),
        target: Point3::new(x, y, 0.0),
        up: Dir3::Y,
        projection: Projection::Perspective { fov_y: 0.8 },
    }
}

fn session(h: &AppHarness) -> &SketchSession {
    h.app()
        .doc_layer::<SketchLayer>()
        .expect("sketch layer")
        .session()
        .expect("an open sketch session")
}

fn applied(h: &AppHarness) -> usize {
    h.document().history().stack().applied()
}

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").as_deref() == Ok("1")
}

/// Create Sketch on XY through the tool's dialog; returns the new feature.
fn open_xy_sketch(h: &mut AppHarness) -> FeatureId {
    h.drive_until_idle();
    h.run_command(CREATE_SKETCH_CMD);
    assert_eq!(h.app().active_tool(), Some(CREATE_SKETCH_CMD));
    h.kittest().get_by_label("XY").click();
    h.drive_until_idle();
    let id = session(h).feature();
    assert!(h.app().active_document().contexts.contains(&SKETCH_CONTEXT));
    assert_eq!(h.app().active_tool(), Some(SKETCH_SELECT_CMD));
    // A fixed top view so that screen distances map to known sketch distances.
    h.app_mut().active_document_mut().camera = top_down(20.0, 15.0, 150.0);
    h.drive_until_idle();
    id
}

fn screen(h: &AppHarness, x: f64, y: f64) -> egui::Pos2 {
    h.world_to_screen(Point3::new(x, y, 0.0))
}

fn click(h: &mut AppHarness, x: f64, y: f64, mods: egui::Modifiers) {
    let p = screen(h, x, y);
    h.click_at(p, PRIM, mods);
    h.drive_until_idle();
}

/// Draws the closed chain (0,0) → (40,0) → (40,30) → (0,30) → (0,0) with the Line tool.
fn draw_rectangle(h: &mut AppHarness) {
    h.run_command(LINE_CMD);
    assert_eq!(h.app().active_tool(), Some(LINE_CMD));
    for (x, y) in [
        (0.0, 0.0),
        (40.0, 0.0),
        (40.0, 30.0),
        (0.0, 30.0),
        (0.0, 0.0),
    ] {
        click(h, x, y, egui::Modifiers::NONE);
    }
}

fn count(s: &Sketch, f: impl Fn(&ConstraintKind) -> bool) -> usize {
    s.constraints().filter(|(_, c)| f(&c.kind)).count()
}

fn lines(s: &Sketch) -> usize {
    s.entities()
        .filter(|(_, e)| matches!(e.kind, EntityKind::Line { .. }))
        .count()
}

// ---- Done-when 1: inference on a closed chain ---------------------------------------------

#[test]
fn rectangle_with_inference_gets_coincident_horizontal_vertical_and_one_profile() {
    let mut h = AppHarness::new();
    let id = open_xy_sketch(&mut h);
    let before = applied(&h);
    draw_rectangle(&mut h);
    let s = session(&h);
    let sk = &s.working().sketch;
    assert_eq!(lines(sk), 4);
    let coincident = count(sk, |k| matches!(k, ConstraintKind::Coincident { .. }));
    let horizontal = count(sk, |k| matches!(k, ConstraintKind::Horizontal(_)));
    let vertical = count(sk, |k| matches!(k, ConstraintKind::Vertical(_)));
    println!(
        "constraints: {coincident} coincident, {horizontal} horizontal, {vertical} vertical, {} total",
        sk.constraints().count()
    );
    assert_eq!((coincident, horizontal, vertical), (4, 2, 2));
    assert_eq!(sk.constraints().count(), 8, "nothing else was inferred");
    // The H/V constraints are on the lines (the line direction inference).
    assert_eq!(
        count(sk, |k| matches!(
            k,
            ConstraintKind::Horizontal(HvTarget::Line(_))
                | ConstraintKind::Vertical(HvTarget::Line(_))
        )),
        4
    );
    assert_eq!(s.profiles().profiles.len(), 1, "one closed profile");
    let area = s.profiles().profiles[0].outer.signed_area().abs();
    assert!((area - 1200.0).abs() < 0.5, "profile area {area}");
    // Each segment was one undo entry (the closing one after its constraint check).
    assert_eq!(applied(&h) - before, 4);
    // The document's recompute sees the same profile.
    let geom = h
        .document()
        .evaluated()
        .sketch(amg_doc::SketchId(id))
        .cloned()
        .expect("solved sketch");
    assert_eq!(geom.profiles.profiles.len(), 1);
}

// ---- Done-when 2: DOF and colours -----------------------------------------------------------

#[test]
fn rectangle_has_four_dof_and_every_entity_is_under_constrained() {
    let mut h = AppHarness::new();
    open_xy_sketch(&mut h);
    draw_rectangle(&mut h);
    let s = session(&h);
    let dof = s.dof().expect("DOF report");
    assert_eq!(dof.dof, 4);
    let ents: Vec<_> = s.display().entities().map(|(e, _)| e).collect();
    assert_eq!(ents.len(), 12, "4 lines and 8 points");
    for e in ents {
        assert_eq!(s.entity_state(e), Some(ConstraintState::Under), "{e:?}");
    }
    // The status bar shows the DOF.
    h.run_command(SKETCH_SELECT_CMD);
    h.drive_until_idle();
    assert!(
        h.kittest()
            .query_all_by_label_contains("DOF: 4 ·")
            .next()
            .is_some(),
        "status bar"
    );
    assert!(h.kittest().query_by_label("DOF: 4").is_some(), "palette");
}

// ---- Done-when 3: dragging a corner ---------------------------------------------------------

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

#[test]
fn timing_dragging_a_corner_for_1000_frames_is_one_undo_entry() {
    use tracing_subscriber::layer::SubscriberExt as _;
    let times = FrameTimes::default();
    let subscriber = tracing_subscriber::registry().with(times.clone());
    let _guard = tracing::subscriber::set_default(subscriber);

    let mut h = AppHarness::new();
    open_xy_sketch(&mut h);
    draw_rectangle(&mut h);
    h.key(egui::Key::Escape); // leave the Line tool: the select & drag tool comes back
    h.drive_until_idle();
    assert_eq!(h.app().active_tool(), Some(SKETCH_SELECT_CMD));
    let before = applied(&h);

    let (from, to) = (screen(&h, 40.0, 30.0), screen(&h, 55.0, 45.0));
    times.0.lock().unwrap().clear();
    let start = Instant::now();
    h.drag(PRIM, egui::Modifiers::NONE, from, to, 1000);
    let total = start.elapsed();
    let mut t = times.0.lock().unwrap().clone();
    h.drive_until_idle();

    t.sort();
    let p50 = t[t.len() / 2];
    let p95 = t[t.len() * 95 / 100];
    let max = *t.last().unwrap();
    let s = session(&h);
    println!(
        "{} frames in {total:?}: UI-thread frame p50 {p50:?}, p95 {p95:?}, max {max:?}; {} drag \
         solves",
        t.len(),
        s.stats().drag_frames
    );
    assert!(t.len() >= 1000, "{} frames", t.len());
    assert!(s.stats().drag_frames > 0);
    assert_eq!(
        applied(&h) - before,
        1,
        "the release is exactly one undo entry"
    );
    assert_eq!(
        h.document().undo_label(),
        Some("Drag sketch geometry"),
        "the drag's entry"
    );
    // The corner followed the cursor; the rectangle stayed one (H/V kept).
    let sk = &s.working().sketch;
    let near = sk
        .entities()
        .filter_map(|(e, _)| sk.point(e))
        .filter(|p| p.distance(Point2::new(55.0, 45.0)) < 0.5)
        .count();
    assert_eq!(near, 2, "both coincident corner points moved to the target");
    assert_eq!(s.profiles().profiles.len(), 1);
    let area = s.profiles().profiles[0].outer.signed_area().abs();
    assert!((area - 55.0 * 45.0).abs() < 1.0, "area {area}");
    if strict_timing() {
        assert!(p95 < Duration::from_millis(2), "p95 {p95:?}");
        assert!(max < Duration::from_millis(16), "a frame took {max:?}");
    }
}

// ---- Done-when 4: undo inside the session ---------------------------------------------------

#[test]
fn undo_and_redo_inside_the_session_reload_the_working_copy() {
    let mut h = AppHarness::new();
    open_xy_sketch(&mut h);
    draw_rectangle(&mut h);
    let reloads = session(&h).stats().reloads;
    h.app_mut().with_cx(|cx| cx.undo());
    h.drive_until_idle();
    let s = session(&h);
    assert_eq!(lines(&s.working().sketch), 3, "the closing line is undone");
    assert_eq!(lines(s.display()), 3, "and no longer drawn");
    assert!(s.stats().reloads > reloads);
    assert!(s.profiles().profiles.is_empty(), "the chain is open again");
    assert_eq!(
        s.dof().map(|d| d.dof),
        Some(5),
        "an open chain of 3: 12 - 4 - 3"
    );
    h.app_mut().with_cx(|cx| cx.undo());
    h.drive_until_idle();
    assert_eq!(lines(&session(&h).working().sketch), 2);
    h.app_mut().with_cx(|cx| cx.redo());
    h.app_mut().with_cx(|cx| cx.redo());
    h.drive_until_idle();
    let s = session(&h);
    assert_eq!(lines(&s.working().sketch), 4);
    assert_eq!(s.dof().map(|d| d.dof), Some(4));
    assert_eq!(s.profiles().profiles.len(), 1);
}

// ---- Done-when 5: Finish Sketch -------------------------------------------------------------

#[test]
fn finish_sketch_leaves_sketch_mode_and_the_timeline_shows_a_healthy_sketch() {
    let mut h = AppHarness::new();
    let id = open_xy_sketch(&mut h);
    draw_rectangle(&mut h);
    h.run_command(FINISH_SKETCH_CMD);
    h.drive_until_idle();
    assert!(!h.app().active_document().contexts.contains(&SKETCH_CONTEXT));
    assert!(
        h.app()
            .doc_layer::<SketchLayer>()
            .unwrap()
            .session()
            .is_none()
    );
    assert_eq!(h.app().active_tool(), None, "back to the select tool");
    assert!(h.kittest().query_by_label("Sketch Palette").is_none());
    // The finished sketch is still drawn (its 4 lines), outside sketch mode.
    assert!(h.app().tool_overlays().primitive_count() >= 4);
    let node = h.document().evaluated().node(id).expect("evaluated node");
    assert_eq!(node.health, Health::Ok);
    let timeline = h.document().design().timeline();
    assert_eq!(timeline.node(id).unwrap().1.feature.kind(), "sketch.sketch");
    // The node is drawn in the timeline (and the browser) under its display name.
    assert!(h.kittest().query_all_by_label("Sketch1").next().is_some());
    // Editing it again opens a new session on the same sketch.
    h.app_mut().edit_feature(id);
    h.drive_until_idle();
    assert_eq!(session(&h).feature(), id);
    assert_eq!(lines(&session(&h).working().sketch), 4);
}

// ---- more session behaviour ----------------------------------------------------------------

#[test]
fn clicking_the_xy_plane_in_the_viewport_creates_the_sketch_and_looks_at_it() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    h.app_mut().active_document_mut().camera = top_down(0.0, 0.0, 200.0);
    h.drive_until_idle();
    h.run_command(CREATE_SKETCH_CMD);
    let p = screen(&h, 10.0, 5.0);
    h.click_at(p, PRIM, egui::Modifiers::NONE);
    h.drive_until_idle();
    let s = session(&h);
    assert_eq!(*s.plane(), amg_core::Plane::XY);
    assert!(h.kittest().query_by_label("Sketch Palette").is_some());
    assert!(
        h.kittest().query_by_label("SKETCH").is_some(),
        "the SKETCH tab shows"
    );
    // Look at: the camera looks along -Z at the plane.
    let cam = h.app().active_document().camera;
    let dir = (cam.target - cam.eye).normalize();
    assert!(dir.z < -0.999, "view direction {dir:?}");
}

#[test]
fn esc_with_no_sketch_tool_leaves_the_session_and_ctrl_suppresses_inference() {
    let mut h = AppHarness::new();
    open_xy_sketch(&mut h);
    h.run_command(LINE_CMD);
    click(&mut h, 0.0, 0.0, egui::Modifiers::NONE);
    // Nearly horizontal with Ctrl: no horizontal constraint, the point stays where it is.
    click(&mut h, 40.0, 0.6, egui::Modifiers::CTRL);
    let sk = session(&h).working().sketch.clone();
    assert_eq!(lines(&sk), 1);
    assert_eq!(
        count(&sk, |k| matches!(k, ConstraintKind::Horizontal(_))),
        0
    );
    // Without Ctrl the same move is inferred horizontal (and a glyph is shown while hovering).
    h.hover_at(screen(&h, 70.0, 0.9));
    h.drive_until_idle();
    let inf = session(&h).inference.clone().expect("inference");
    assert_eq!(inf.glyph(), Some("H"));
    assert!(!inf.guides(&sk, None).is_empty() || inf.line_axis == Some(true));
    click(&mut h, 70.0, 0.9, egui::Modifiers::NONE);
    let sk = session(&h).working().sketch.clone();
    assert_eq!(
        count(&sk, |k| matches!(k, ConstraintKind::Horizontal(_))),
        1
    );
    // Esc ends the chain, Esc again ends the Line tool, Esc in the select tool leaves sketch.
    for _ in 0..3 {
        h.key(egui::Key::Escape);
        h.drive_until_idle();
    }
    assert!(!h.app().active_document().contexts.contains(&SKETCH_CONTEXT));
}

#[test]
fn heads_up_length_becomes_a_driving_dimension() {
    let mut h = AppHarness::new();
    open_xy_sketch(&mut h);
    h.run_command(LINE_CMD);
    click(&mut h, 0.0, 0.0, egui::Modifiers::NONE);
    h.hover_at(screen(&h, 10.0, 10.0));
    h.kittest().event(egui::Event::Text("25".into()));
    h.kittest().step();
    assert_eq!(
        session(&h).heads_up.as_ref().map(|hu| hu.fields[0].clone()),
        Some("25".to_owned())
    );
    h.key(egui::Key::Enter);
    h.drive_until_idle();
    let s = session(&h);
    let sk = &s.working().sketch;
    assert_eq!(lines(sk), 1);
    let dims: Vec<_> = sk
        .constraints()
        .filter_map(|(id, c)| match &c.kind {
            ConstraintKind::Dimension(d) => Some((id, d.clone())),
            _ => None,
        })
        .collect();
    assert_eq!(dims.len(), 1);
    assert!(matches!(dims[0].1.kind, DimensionKind::Length { .. }));
    assert!(dims[0].1.driving);
    assert!((dims[0].1.value - 25.0).abs() < 1e-9);
    // The line points at the cursor (45 degrees) and is 25 mm long.
    let (a, b) = sk
        .entities()
        .find_map(|(_, e)| match e.kind {
            EntityKind::Line { start, end } => Some((sk.point(start)?, sk.point(end)?)),
            _ => None,
        })
        .unwrap();
    assert!((a.distance(b) - 25.0).abs() < 1e-6);
    assert!(((b - a).normalize() - Point2::new(1.0, 1.0).normalize()).length() < 1e-3);
    // The dimension is in the feature's expressions (a model parameter after the commit).
    assert_eq!(s.working().dims.len(), 1);
}

#[test]
fn dragging_back_from_the_chain_end_draws_a_tangent_arc() {
    let mut h = AppHarness::new();
    open_xy_sketch(&mut h);
    h.run_command(LINE_CMD);
    click(&mut h, 0.0, 0.0, egui::Modifiers::NONE);
    click(&mut h, 20.0, 0.0, egui::Modifiers::NONE);
    let (from, to) = (screen(&h, 20.0, 0.0), screen(&h, 30.0, 10.0));
    h.drag(PRIM, egui::Modifiers::NONE, from, to, 12);
    h.drive_until_idle();
    let sk = session(&h).working().sketch.clone();
    let arcs = sk
        .entities()
        .filter(|(_, e)| matches!(e.kind, EntityKind::Arc { .. }))
        .count();
    assert_eq!(arcs, 1);
    assert_eq!(
        count(&sk, |k| matches!(k, ConstraintKind::Tangent { .. })),
        1
    );
    assert_eq!(session(&h).status().map(|s| s.is_converged()), Some(true));
}

#[test]
fn palette_toggles_change_the_view() {
    let mut h = AppHarness::new();
    open_xy_sketch(&mut h);
    assert!(h.app().view_sections().planes.is_empty());
    h.kittest().get_by_label("Slice").click();
    h.drive_until_idle();
    assert!(session(&h).palette.slice);
    assert_eq!(
        h.app().view_sections().planes.len(),
        1,
        "slice cuts at the sketch plane"
    );
    let grid_on = h.app().tool_overlays().primitive_count();
    h.kittest().get_by_label("Sketch grid").click();
    h.drive_until_idle();
    assert!(!session(&h).palette.grid);
    assert!(
        h.app().tool_overlays().primitive_count() < grid_on,
        "the grid is gone"
    );
    h.run_command(FINISH_SKETCH_CMD);
    assert!(
        h.app()
            .doc_layer::<SketchLayer>()
            .unwrap()
            .session()
            .is_none()
    );
}
