//! UI-03 tests: the tool router (event order, Esc / Enter, replace semantics), navigation over
//! S10's `NavController` while a tool is active, latest-wins hover picks with highlights (CPU
//! and GPU), the `PreviewDriver` debounce and frame times while a long job runs.
//!
//! Everything runs the real app in `AppHarness` (kittest). `gpu_*` tests drive a real S10
//! renderer and skip without an adapter. `timing_*` tests print their measurement and assert
//! the budget only under `AMG_STRICT_TIMING=1`.

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::borrow::Cow;
use std::cell::{Cell, RefCell};
use std::rc::Rc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use amg_core::{Dir3, Point3};
use amg_doc::stub::IndexedMesh;
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, Command, Dependency, Document, FeatureBox, FeatureError,
    FeatureOutput, RecomputeCtx,
};
use amg_jobs::{Job, JobCtx, SpawnOptions};
use amg_render::{Camera, Projection};
use amg_ui::CommitBatch;
use amg_ui::command::{CmdAction, CmdId, CommandDef};
use amg_ui::input::ViewportEvent;
use amg_ui::module::{Registrar, UiModule};
use amg_ui::pick::PickTarget;
use amg_ui::scene::HighlightLayer;
use amg_ui::testing::AppHarness;
use amg_ui::tool::{FinishReason, InputResult, Tool, ToolCx, ToolInit};
use serde::{Deserialize, Serialize};

// ---- test features -----------------------------------------------------------------------

/// A flat `n × n` grid at `z = 0` from `(x0, 0)`, 1 mm cells.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct ToolGrid {
    n: u32,
    x0: f64,
}

fn grid_mesh(n: u32, x0: f64) -> IndexedMesh {
    let mut pos = Vec::new();
    for j in 0..=n {
        for i in 0..=n {
            pos.push(Point3::new(x0 + f64::from(i), f64::from(j), 0.0));
        }
    }
    let w = n + 1;
    let mut tris = Vec::new();
    for j in 0..n {
        for i in 0..n {
            let a = j * w + i;
            tris.push([a, a + 1, a + w + 1]);
            tris.push([a, a + w + 1, a + w]);
        }
    }
    IndexedMesh::new(pos, tris).expect("valid grid")
}

fn grid_output(ctx: &mut RecomputeCtx<'_>, n: u32, x0: f64) -> FeatureOutput {
    FeatureOutput {
        bodies: vec![BodyChange {
            id: ctx.new_body_id(0),
            change: BodyChangeKind::Created {
                geom: BodyGeom::Mesh(Arc::new(grid_mesh(n, x0))),
                naming: None,
            },
        }],
        ..FeatureOutput::default()
    }
}

impl amg_doc::Feature for ToolGrid {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Ok(grid_output(ctx, self.n, self.x0))
    }
}

amg_doc::register_feature!(ToolGrid, "uitest.tool_grid", version = 1);

/// Counts its computations (previews of it: the document never contains one).
static COUNTED_COMPUTES: AtomicU64 = AtomicU64::new(0);

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Counted {
    tag: u32,
}

impl amg_doc::Feature for Counted {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        COUNTED_COMPUTES.fetch_add(1, Ordering::SeqCst);
        Ok(grid_output(ctx, 4, 20.0))
    }
}

amg_doc::register_feature!(Counted, "uitest.counted", version = 1);

fn add_feature(f: FeatureBox, doc: &Document) -> Command {
    Command::AddFeature {
        at: None,
        feature: f,
        context: doc.design().root(),
        name: None,
    }
}

// ---- scripted test tools (registered through inventory) -----------------------------------

thread_local! {
    static LOG: RefCell<Vec<String>> = const { RefCell::new(Vec::new()) };
}

fn log(s: impl Into<String>) {
    LOG.with(|l| l.borrow_mut().push(s.into()));
}

fn take_log() -> Vec<String> {
    LOG.with(|l| std::mem::take(&mut *l.borrow_mut()))
}

fn describe(ev: &ViewportEvent) -> String {
    match ev {
        ViewportEvent::Hover { .. } => "hover".into(),
        ViewportEvent::Press { button, .. } => format!("press:{button:?}"),
        ViewportEvent::Release { button, .. } => format!("release:{button:?}"),
        ViewportEvent::Click { button, count, .. } => format!("click:{button:?}:{count}"),
        ViewportEvent::DragStart { button, .. } => format!("drag_start:{button:?}"),
        ViewportEvent::Drag { button, .. } => format!("drag:{button:?}"),
        ViewportEvent::DragEnd { button, .. } => format!("drag_end:{button:?}"),
        ViewportEvent::Scroll { .. } => "scroll".into(),
        ViewportEvent::Key { key, pressed, .. } => format!("key:{key:?}:{pressed}"),
        ViewportEvent::Text(t) => format!("text:{t}"),
    }
}

/// Logs every event and ignores it; commits one `AddFeature` on `Commit`.
struct Script {
    name: &'static str,
    commit_on_replace: bool,
}

impl Tool for Script {
    fn id(&self) -> CmdId {
        CmdId(self.name)
    }

    fn label(&self) -> Cow<'static, str> {
        Cow::Borrowed("Script")
    }

    fn start(&mut self, _cx: &mut ToolCx<'_>) {
        log(format!("{}:start", self.name));
    }

    fn input(&mut self, ev: &ViewportEvent, _cx: &mut ToolCx<'_>) -> InputResult {
        log(describe(ev));
        InputResult::Ignored
    }

    fn commit_on_replace(&self) -> bool {
        self.commit_on_replace
    }

    fn finish(&mut self, reason: FinishReason, cx: &mut ToolCx<'_>) -> Option<CommitBatch> {
        log(format!("{}:finish:{reason:?}", self.name));
        (reason == FinishReason::Commit).then(|| {
            let root = cx.env().design.root();
            CommitBatch {
                label: "Script commit".into(),
                commands: vec![Command::AddFeature {
                    at: None,
                    feature: FeatureBox::new(ToolGrid { n: 2, x0: 0.0 }),
                    context: root,
                    name: None,
                }],
            }
        })
    }
}

const SCRIPT: CmdId = CmdId("uitest.script");
const OTHER: CmdId = CmdId("uitest.other");

fn start_script(_: &ToolInit<'_>) -> Box<dyn Tool> {
    Box::new(Script {
        name: "uitest.script",
        commit_on_replace: false,
    })
}

fn start_other(_: &ToolInit<'_>) -> Box<dyn Tool> {
    Box::new(Script {
        name: "uitest.other",
        commit_on_replace: true,
    })
}

inventory::submit! { UiModule { id: "zz_tools_test", register } }

fn register(r: &mut Registrar<'_>) {
    r.command(CommandDef::new(
        SCRIPT.0,
        "Script tool",
        CmdAction::StartTool(start_script),
    ));
    r.command(CommandDef::new(
        OTHER.0,
        "Other tool",
        CmdAction::StartTool(start_other),
    ));
}

// ---- helpers ------------------------------------------------------------------------------

fn top_down(x: f64, y: f64, height: f64) -> Camera {
    Camera {
        eye: Point3::new(x, y, height),
        target: Point3::new(x, y, 0.0),
        up: Dir3::Y,
        projection: Projection::Perspective { fov_y: 0.8 },
    }
}

fn centre(h: &AppHarness) -> egui::Pos2 {
    h.camera_view().rect.center()
}

fn applied(h: &AppHarness) -> usize {
    h.document().history().stack().applied()
}

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").as_deref() == Ok("1")
}

/// Steps until `done` holds; returns the number of frames stepped.
fn frames_until(h: &mut AppHarness, max: u32, done: impl Fn(&AppHarness) -> bool) -> Option<u32> {
    for n in 0..=max {
        if done(h) {
            return Some(n);
        }
        h.kittest().step();
    }
    None
}

/// A harness with a 10 × 10 mm grid body at the origin, seen from above.
fn grid_harness() -> AppHarness {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    let cmd = add_feature(FeatureBox::new(ToolGrid { n: 10, x0: 0.0 }), h.document());
    h.app_mut().submit(CommitBatch {
        label: "Grid".into(),
        commands: vec![cmd],
    });
    h.app_mut().active_document_mut().camera = top_down(5.0, 5.0, 40.0);
    h.drive_until_idle();
    assert_eq!(h.scene().bodies().count(), 1);
    h
}

// ---- Done-when 1: event order, Esc, Enter, replace ------------------------------------------

#[test]
fn scripted_tool_sees_events_in_order_and_esc_enter_and_replace_finish_it() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    take_log();
    h.run_command(SCRIPT);
    assert_eq!(h.app().active_tool(), Some(SCRIPT));
    assert_eq!(take_log(), ["uitest.script:start"]);

    // Events arrive in order.
    let c = centre(&h);
    let prim = egui::PointerButton::Primary;
    h.hover_at(c);
    h.drag(prim, egui::Modifiers::NONE, c, c + egui::vec2(40.0, 0.0), 4);
    h.click_at(c + egui::vec2(40.0, 0.0), prim, egui::Modifiers::NONE);
    h.key(egui::Key::A);
    let mut got = take_log();
    got.dedup();
    assert_eq!(
        got,
        [
            "hover",
            "press:Primary",
            "drag_start:Primary",
            "drag:Primary",
            "release:Primary",
            "drag_end:Primary",
            "press:Primary",
            "release:Primary",
            "click:Primary:1",
            "key:A:true",
            "key:A:false",
        ]
    );

    // Esc cancels: nothing is committed.
    let before = applied(&h);
    h.key(egui::Key::Escape);
    h.drive_until_idle();
    assert_eq!(
        take_log(),
        ["key:Escape:true", "uitest.script:finish:Cancel"]
    );
    assert_eq!(h.app().active_tool(), None);
    assert_eq!(applied(&h), before);

    // Enter commits exactly one undo entry.
    h.run_command(SCRIPT);
    take_log();
    h.key(egui::Key::Enter);
    h.drive_until_idle();
    let log = take_log();
    assert!(
        log.contains(&"uitest.script:finish:Commit".to_owned()),
        "{log:?}"
    );
    assert_eq!(h.app().active_tool(), None);
    assert_eq!(applied(&h), before + 1);
    assert_eq!(h.document().history().undo_label(), Some("Script commit"));

    // Starting another command replaces the active tool.
    h.run_command(SCRIPT);
    take_log();
    h.run_command(OTHER);
    assert_eq!(
        take_log(),
        ["uitest.script:finish:Replaced", "uitest.other:start"]
    );
    assert_eq!(h.app().active_tool(), Some(OTHER));
    assert_eq!(applied(&h), before + 1, "Replaced commits nothing");

    // A tool that commits on replace commits instead.
    h.run_command(SCRIPT);
    assert_eq!(
        take_log(),
        ["uitest.other:finish:Commit", "uitest.script:start"]
    );
    h.drive_until_idle();
    assert_eq!(applied(&h), before + 2);

    // Esc cancels the tool that replaced the other one.
    h.key(egui::Key::Escape);
    assert_eq!(h.app().active_tool(), None);
}

// ---- Done-when 2: navigation while a tool ignores the event -------------------------------

#[test]
fn middle_drag_navigates_while_a_tool_that_ignores_it_is_active() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    h.run_command(SCRIPT);
    take_log();
    let c = centre(&h);
    let mid = egui::PointerButton::Middle;

    // Fusion: Shift+MMB orbits (about the target: the scene is empty).
    let before = h.app().active_document().camera;
    h.drag(mid, egui::Modifiers::SHIFT, c, c + egui::vec2(80.0, 0.0), 6);
    let after = h.app().active_document().camera;
    let log = take_log();
    assert!(
        log.contains(&"drag_start:Middle".to_owned()),
        "offered first: {log:?}"
    );
    assert!(
        !log.iter()
            .any(|e| e == "drag:Middle" || e == "drag_end:Middle"),
        "navigation owns the rest of the drag: {log:?}"
    );
    assert_eq!(h.app().active_tool(), Some(SCRIPT), "the tool stays active");
    let d0 = before.eye - before.target;
    let d1 = after.eye - after.target;
    assert!(
        after.target.distance(before.target) < 1e-9,
        "orbit about the target"
    );
    assert!(
        (d0.length() - d1.length()).abs() < 1e-6,
        "orbit keeps the distance"
    );
    let turned = d0.normalize().dot(d1.normalize()).clamp(-1.0, 1.0).acos();
    assert!(turned > 0.1, "the view turned by {turned} rad");

    // MMB pans: target and eye move together.
    let before = after;
    h.drag(mid, egui::Modifiers::NONE, c, c + egui::vec2(0.0, 50.0), 5);
    let after = h.app().active_document().camera;
    assert!(after.target.distance(before.target) > 1.0, "panned");
    assert!(((after.eye - after.target) - (before.eye - before.target)).length() < 1e-6);

    // The wheel zooms (up = in).
    let before = after;
    h.hover_at(c);
    h.kittest().event(egui::Event::MouseWheel {
        unit: egui::MouseWheelUnit::Line,
        delta: egui::vec2(0.0, 2.0),
        phase: egui::TouchPhase::Move,
        modifiers: egui::Modifiers::NONE,
    });
    h.kittest().step();
    let after = h.app().active_document().camera;
    let ratio = after.eye.distance(after.target) / before.eye.distance(before.target);
    assert!(
        (ratio - 0.81).abs() < 1e-6,
        "two wheel steps zoom by 0.9², got {ratio}"
    );
    assert!(
        take_log().contains(&"scroll".to_owned()),
        "the tool saw the wheel first"
    );

    // A primary drag the tool ignores does not move the camera (no binding).
    let before = after;
    h.drag(
        egui::PointerButton::Primary,
        egui::Modifiers::NONE,
        c,
        c + egui::vec2(60.0, 0.0),
        4,
    );
    assert_eq!(h.app().active_document().camera, before);
}

// ---- Done-when 3: hover highlight within two frames (CPU) ---------------------------------

#[test]
fn hover_highlights_within_two_frames_and_click_selects_with_cpu_picking() {
    let mut h = grid_harness();
    let body = h.scene().bodies().next().unwrap();
    let on = h.world_to_screen(Point3::new(5.0, 5.0, 0.0));
    let off = h.world_to_screen(Point3::new(20.0, 5.0, 0.0));
    assert!(h.camera_view().rect.contains(off));

    let mut worst = 0;
    for round in 0..5 {
        // Move onto the body: the event frame requests the pick, the next frame shows it.
        let p = on + egui::vec2(round as f32, 0.0);
        h.hover_at(p);
        let n = frames_until(&mut h, 10, |h| {
            !h.scene().highlight(HighlightLayer::Hover).is_empty()
        })
        .expect("hover highlight appears");
        worst = worst.max(n);
        let hit = h.app().hover().expect("hover hit").clone();
        assert_eq!(hit.at, p, "the position the pick was taken at");
        assert_eq!(
            hit.item.target,
            PickTarget::Body {
                body,
                instance: 0,
                sub: None
            }
        );
        assert!(hit.world.z.abs() < 1e-6 && (hit.world.x - 5.0).abs() < 0.5);
        // Off the body: the highlight goes away as fast.
        h.hover_at(off);
        let n = frames_until(&mut h, 10, |h| {
            h.scene().highlight(HighlightLayer::Hover).is_empty()
        })
        .expect("hover highlight clears");
        worst = worst.max(n);
    }
    println!("CPU hover: highlight after at most {worst} frame(s) after the event frame");
    assert!(
        worst <= 1,
        "the event frame + {worst} frames (limit: 2 frames in total)"
    );

    // Click selects the body (whole-body priority), the selection is highlighted.
    h.click_at(on, egui::PointerButton::Primary, egui::Modifiers::NONE);
    h.drive_until_idle();
    let sel: Vec<PickTarget> = h
        .app()
        .active_document()
        .selection
        .items()
        .iter()
        .map(|i| i.target)
        .collect();
    assert_eq!(
        sel,
        [PickTarget::Body {
            body,
            instance: 0,
            sub: None
        }]
    );
    assert_eq!(h.scene().highlight(HighlightLayer::Selected).len(), 1);
    // A click on empty space clears it; Esc with no tool clears too.
    h.click_at(off, egui::PointerButton::Primary, egui::Modifiers::NONE);
    h.drive_until_idle();
    assert!(h.app().active_document().selection.is_empty());
    h.click_at(on, egui::PointerButton::Primary, egui::Modifiers::NONE);
    h.drive_until_idle();
    assert_eq!(h.app().active_document().selection.len(), 1);
    h.key(egui::Key::Escape);
    h.drive_until_idle();
    assert!(h.app().active_document().selection.is_empty());
    assert!(h.scene().highlight(HighlightLayer::Selected).is_empty());
}

// ---- Done-when 3 on the GPU: S10 picking through GpuPickService ----------------------------

mod gpu {
    use super::*;
    use amg_jobs::gpu::{Gpu, GpuConfig};
    use amg_render::{Renderer, RendererConfig, ViewId};
    use amg_ui::pick::{GpuPickService, PickService};
    use amg_ui::scene::GpuSceneBackend;
    use amg_ui::viewport::{NullViewport, ViewportBackend, ViewportFrame};

    /// Draws like `NullViewport`, keeps an S10 view in sync with the document camera and,
    /// after each frame with picks pending, encodes and submits the view (what egui's paint
    /// callback does in the app).
    struct GpuViewport {
        r: Rc<RefCell<Renderer>>,
        view: ViewId,
        encodes: Rc<Cell<u32>>,
    }

    impl ViewportBackend for GpuViewport {
        fn show(&mut self, ui: &mut egui::Ui, frame: &ViewportFrame<'_>) -> egui::Response {
            let resp = NullViewport.show(ui, frame);
            let ppp = ui.ctx().pixels_per_point();
            #[allow(
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss,
                reason = "test"
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
                self.encodes.set(self.encodes.get() + 1);
            }
        }
    }

    #[test]
    fn gpu_hover_highlights_within_two_frames_with_s10_picking() {
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
        let encodes = Rc::new(Cell::new(0));
        let mut h = AppHarness::builder()
            .viewport(Box::new(GpuViewport {
                r: Rc::clone(&r),
                view,
                encodes: Rc::clone(&encodes),
            }))
            .build();
        let rs = Rc::clone(&r);
        h.app_mut().set_scene_backend(Box::new(GpuSceneBackend::new(
            Arc::clone(&gpu),
            Box::new(move |f| f(rs.borrow_mut().scene_mut())),
        )));
        h.drive_until_idle();
        let cmd = add_feature(FeatureBox::new(ToolGrid { n: 10, x0: 0.0 }), h.document());
        h.app_mut().submit(CommitBatch {
            label: "Grid".into(),
            commands: vec![cmd],
        });
        h.app_mut().active_document_mut().camera = top_down(5.0, 5.0, 40.0);
        h.drive_until_idle();
        assert_eq!(r.borrow().scene().instance_count(), 1);
        assert!(r.borrow_mut().prepare_view(view));
        let body = h.scene().bodies().next().unwrap();
        let on = h.world_to_screen(Point3::new(5.0, 5.0, 0.0));
        let off = h.world_to_screen(Point3::new(20.0, 5.0, 0.0));

        let mut worst = 0;
        for round in 0..10 {
            let p = on + egui::vec2(round as f32 * 3.0, round as f32);
            h.hover_at(p);
            let n = frames_until(&mut h, 20, |h| {
                !h.scene().highlight(HighlightLayer::Hover).is_empty()
            })
            .expect("hover highlight appears");
            worst = worst.max(n);
            assert_eq!(
                h.app().hover().unwrap().item.target,
                PickTarget::Body {
                    body,
                    instance: 0,
                    sub: None
                }
            );
            assert!(
                !r.borrow()
                    .scene()
                    .highlight_layer(amg_render::HighlightLayer::Hover)
                    .is_empty(),
                "the render scene draws the hover layer"
            );
            h.hover_at(off);
            let n = frames_until(&mut h, 20, |h| {
                h.scene().highlight(HighlightLayer::Hover).is_empty()
            })
            .expect("hover highlight clears");
            worst = worst.max(n);
        }
        println!(
            "GPU hover ({}): highlight after at most {worst} frame(s) after the event frame, {} encodes",
            gpu.info().name,
            encodes.get()
        );
        assert!(
            worst <= 1,
            "the event frame + {worst} frames (limit: 2 frames in total)"
        );
        // Clicking through S10 selects the body as well.
        h.click_at(on, egui::PointerButton::Primary, egui::Modifiers::NONE);
        h.drive_until_idle();
        assert_eq!(h.app().active_document().selection.len(), 1);
        drop(h);
    }
}

// ---- Done-when 4: PreviewDriver debounce, latest wins, no undo entry ----------------------

#[test]
fn twenty_rapid_preview_sets_compute_one_preview_outside_the_undo_stack() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    COUNTED_COMPUTES.store(0, Ordering::SeqCst);
    let t0 = Instant::now();
    h.app_mut().preview_driver().set_clock(Some(t0));
    for tag in 0..20 {
        let cmd = add_feature(FeatureBox::new(Counted { tag }), h.document());
        h.app_mut().preview_driver().set(cmd);
        h.kittest().step();
        assert!(matches!(
            h.app_mut().preview_driver().state(),
            amg_ui::preview::PreviewState::Pending
        ));
    }
    assert_eq!(
        h.app_mut().preview_driver().spawned(),
        0,
        "still debouncing"
    );
    // The debounce (150 ms) runs out after the last set.
    h.app_mut()
        .preview_driver()
        .set_clock(Some(t0 + Duration::from_millis(200)));
    h.drive_until_idle();
    let d = h.app_mut().preview_driver();
    assert_eq!(d.spawned(), 1);
    assert!(matches!(d.state(), amg_ui::preview::PreviewState::Ready(_)));
    assert_eq!(
        COUNTED_COMPUTES.load(Ordering::SeqCst),
        1,
        "exactly one preview computed"
    );
    assert_eq!(
        h.scene().preview_bodies().count(),
        1,
        "shown on the preview layer"
    );
    assert!(!h.document().history().can_undo(), "no undo entry");
    assert_eq!(h.document().design().timeline().nodes().count(), 0);

    h.app_mut().preview_driver().clear();
    h.drive_until_idle();
    assert_eq!(h.scene().preview_bodies().count(), 0, "cleared");
    assert!(!h.document().history().can_undo());
}

#[test]
fn a_newer_preview_cancels_the_one_in_flight() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    let d = h.app_mut().preview_driver();
    d.set_debounce(Duration::ZERO);
    let cmd = add_feature(FeatureBox::new(ToolGrid { n: 3, x0: 0.0 }), h.document());
    h.app_mut().preview_driver().set(cmd);
    h.kittest().step(); // spawned
    let cmd = add_feature(FeatureBox::new(ToolGrid { n: 5, x0: 0.0 }), h.document());
    h.app_mut().preview_driver().set(cmd);
    h.drive_until_idle();
    let d = h.app_mut().preview_driver();
    assert_eq!(d.spawned(), 2);
    let amg_ui::preview::PreviewState::Ready(ev) = d.state().clone() else {
        panic!("ready")
    };
    let (_, b) = ev.bodies().next().unwrap();
    let BodyGeom::Mesh(m) = &b.geom else {
        panic!("mesh")
    };
    assert_eq!(m.face_count(), 50, "the newest command's preview");
}

// ---- Done-when 5: no UI-thread frame over 16 ms while a 2 s job runs -----------------------

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

/// Runs for 2 s, checking for cancellation every 10 ms.
struct TwoSeconds;

impl Job for TwoSeconds {
    type Output = ();
    type Error = amg_core::Error;
    fn label(&self) -> Cow<'static, str> {
        "Two-second test job".into()
    }
    fn run(self, ctx: &JobCtx) -> Result<(), amg_core::Error> {
        let t0 = Instant::now();
        while t0.elapsed() < Duration::from_secs(2) {
            ctx.checkpoint()?;
            ctx.progress(t0.elapsed().as_secs_f32() / 2.0);
            std::thread::sleep(Duration::from_millis(10));
        }
        Ok(())
    }
}

#[test]
fn timing_no_ui_frame_over_16ms_while_a_two_second_job_runs() {
    use tracing_subscriber::layer::SubscriberExt as _;

    let times = FrameTimes::default();
    let subscriber = tracing_subscriber::registry().with(times.clone());
    let _guard = tracing::subscriber::set_default(subscriber);

    let mut h = grid_harness();
    h.run_command(SCRIPT);
    let done: Rc<Cell<bool>> = Rc::new(Cell::new(false));
    let d = Rc::clone(&done);
    h.app_mut()
        .spawn_tracked(TwoSeconds, SpawnOptions::default(), move |res, _cx| {
            d.set(res.is_ok());
        });
    times.0.lock().unwrap().clear();
    let start = Instant::now();
    let c = centre(&h);
    let mut i = 0u32;
    // Hover sweeps, a Shift+MMB orbit and wheel zooms while the job runs, like a user would.
    while !done.get() {
        let x = (i % 40) as f32 * 4.0 - 80.0;
        match i % 50 {
            10 => h.kittest().event(egui::Event::PointerButton {
                pos: c,
                button: egui::PointerButton::Middle,
                pressed: true,
                modifiers: egui::Modifiers::SHIFT,
            }),
            30 => h.kittest().event(egui::Event::PointerButton {
                pos: c + egui::vec2(x, 0.0),
                button: egui::PointerButton::Middle,
                pressed: false,
                modifiers: egui::Modifiers::SHIFT,
            }),
            40 => h.kittest().event(egui::Event::MouseWheel {
                unit: egui::MouseWheelUnit::Line,
                delta: egui::vec2(0.0, if i % 100 < 50 { 1.0 } else { -1.0 }),
                phase: egui::TouchPhase::Move,
                modifiers: egui::Modifiers::NONE,
            }),
            _ => h
                .kittest()
                .event(egui::Event::PointerMoved(c + egui::vec2(x, 0.0))),
        }
        h.kittest().step();
        i += 1;
        assert!(
            start.elapsed() < Duration::from_secs(60),
            "the job never finished"
        );
        std::thread::sleep(Duration::from_millis(2));
    }
    let total = start.elapsed();
    let mut t = times.0.lock().unwrap().clone();
    t.sort();
    let max = *t.last().unwrap();
    let p50 = t[t.len() / 2];
    let p99 = t[t.len() * 99 / 100];
    println!(
        "{} frames in {total:?} while the job ran: UI-thread frame p50 {p50:?}, p99 {p99:?}, max {max:?}",
        t.len()
    );
    assert!(total >= Duration::from_secs(2));
    assert!(t.len() > 50, "the UI kept running frames: {}", t.len());
    assert_eq!(h.app().active_tool(), Some(SCRIPT));
    if strict_timing() {
        assert!(max < Duration::from_millis(16), "a frame took {max:?}");
    }
}
