//! Tools and viewport input routing (spec §6, Interfaces §A `amg_ui::tool`).
//!
//! One tool is active per document; without one the default [`SelectTool`] runs. A [`Tool`]
//! gets [`ViewportEvent`]s, can draw a floating dialog and finishes with a [`FinishReason`]. A
//! commit returns a [`CommitBatch`], applied at the end of the frame as one undo entry.
//!
//! **Routing priority** (note §4): text focus → popup → manipulator → active tool → navigation
//! → `SelectTool` → shortcuts.
//! - Text focus and popups: key events are not routed while a text field has focus or the
//!   search popup is open, and a press counts for the viewport only where no other layer
//!   (window, popup, toast) covers it.
//! - Manipulators come with UI-06 (they will sit before the tool here).
//! - Key and text events go to the active tool first, before the shortcut map. An ignored
//!   `Esc` cancels the tool (no tool: clears the selection), an ignored `Enter` commits it.
//! - Pointer events go to the active tool; a drag it ignores goes to the [`Navigator`] (the
//!   preset's bindings), which then owns that button until release. Without an active tool,
//!   what navigation does not take goes to the `SelectTool`.
//!
//! **Hover** picks are latest-wins and never block: each frame with pointer motion requests one
//! point pick at the newest position; [`ToolCx::hover`] is the newest answer with the position
//! it was taken at. The hovered body is put on the Hover highlight layer, the selection on the
//! Selected layer.
//!
//! **Replace semantics:** starting a command whose action is [`CmdAction::StartTool`] finishes
//! the active tool with `Replaced` (or `Commit` if [`Tool::commit_on_replace`]) first.
//!
//! [`CmdAction::StartTool`]: crate::command::CmdAction::StartTool
//! [`Navigator`]: crate::nav::Navigator

use std::borrow::Cow;
use std::time::Instant;

use amg_core::Point3;

use crate::app::{AmalgamApp, AppCx, CmdEnv, CommitBatch};
use crate::command::CmdId;
use crate::input::{CameraView, InputSynth, ViewportEvent, key_events};
use crate::pick::{
    PickFrame, PickHitUi, PickMode, PickService, PickTarget, PickTicket, RegionMode,
};
use crate::preview::PreviewDriver;
use crate::scene::{HighlightLayer, SceneAdapter};
use crate::select::{EntityFilter, SelItem, SelectPriority, Selection, item_for_hit};

/// Radius of hover and click picks, points (a hint: S10 uses its configured radius).
pub const PICK_RADIUS_PT: f32 = 3.0;

/// What a tool is started with.
#[derive(Debug)]
pub struct ToolInit<'a> {
    /// The read-only app view.
    pub env: CmdEnv<'a>,
    /// The selection when the command started.
    pub preselection: &'a [SelItem],
    /// The feature being edited (UI-05 "Edit feature"); `None` for a new one.
    pub edit: Option<amg_doc::FeatureId>,
}

/// Whether a tool used an event.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum InputResult {
    /// Used; nothing after the tool sees it.
    Consumed,
    /// Not used; navigation / the select tool / shortcuts may take it.
    Ignored,
}

/// Why a tool finishes.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FinishReason {
    /// OK / Enter: apply.
    Commit,
    /// Cancel / Esc.
    Cancel,
    /// Another command started.
    Replaced,
}

/// An interactive tool (a command in progress).
pub trait Tool: 'static {
    /// The command that started it.
    fn id(&self) -> CmdId;
    /// Dialog title and status text.
    fn label(&self) -> Cow<'static, str>;
    /// Once, when the tool becomes active.
    fn start(&mut self, _cx: &mut ToolCx<'_>) {}
    /// A viewport, key or text event.
    fn input(&mut self, ev: &ViewportEvent, cx: &mut ToolCx<'_>) -> InputResult;
    /// Once per frame (poll jobs/previews, update state).
    fn update(&mut self, _cx: &mut ToolCx<'_>) {}
    /// Contents of the floating dialog. Return false for tools without a dialog.
    fn dialog_ui(&mut self, _ui: &mut egui::Ui, _cx: &mut ToolCx<'_>) -> bool {
        false
    }
    /// What the tool picks (intersected with the selection filter bar).
    fn pick_filter(&self) -> EntityFilter {
        EntityFilter::ALL
    }
    /// Status-bar hint.
    fn status_hint(&self) -> Option<Cow<'static, str>> {
        None
    }
    /// If true, starting another command commits this tool instead of cancelling it.
    fn commit_on_replace(&self) -> bool {
        false
    }
    /// The tool's RND-07 overlays for this frame (immediate mode): called once per frame
    /// after [`Tool::update`] with an empty list, which the viewport then draws (additive,
    /// INS-08).
    fn overlay(&mut self, _out: &mut amg_render::OverlayList, _cx: &ToolCx<'_>) {}
    /// `Commit` → Some(batch) to apply (or None if nothing to do); `Cancel`/`Replaced` → None.
    fn finish(&mut self, reason: FinishReason, cx: &mut ToolCx<'_>) -> Option<CommitBatch>;
}

/// The newest hover pick.
#[derive(Clone, Debug, PartialEq)]
pub struct HoverHit {
    /// Cursor position the pick was taken at.
    pub at: egui::Pos2,
    /// What is under it.
    pub item: SelItem,
    /// World point, mm.
    pub world: Point3,
    /// Distance along the pick ray, mm.
    pub depth: f32,
}

/// Per-document state a [`ToolCx`] reaches besides the app.
#[derive(Debug)]
pub(crate) struct ToolFrame {
    pub(crate) hover: Option<HoverHit>,
    pub(crate) view: Option<CameraView>,
    pub(crate) preview: PreviewDriver,
    pub(crate) finish: Option<FinishReason>,
    pub(crate) cursor: Option<egui::CursorIcon>,
    /// `tool.pick_filter() ∩ selection filter` (∩ the select priority's filter for the select
    /// tool).
    pub(crate) filter: EntityFilter,
    /// The select priority hits are lifted with (`Default` for other tools).
    pub(crate) priority: SelectPriority,
    /// The filter of the active dialog's active `SelectionField` (UI-15), read after the
    /// dialog was drawn; intersected into the active tool's filter.
    pub(crate) dialog_filter: Option<EntityFilter>,
    /// The keyboard modifiers of the newest routed frame (additive, UI-09).
    pub(crate) mods: egui::Modifiers,
}

impl Default for ToolFrame {
    fn default() -> Self {
        ToolFrame {
            hover: None,
            view: None,
            preview: PreviewDriver::default(),
            finish: None,
            cursor: None,
            filter: EntityFilter::ALL,
            priority: SelectPriority::Default,
            dialog_filter: None,
            mods: egui::Modifiers::NONE,
        }
    }
}

/// What a tool works with: the app (deferred mutation), the frame's camera, hover, preview and
/// picking.
pub struct ToolCx<'a> {
    app: AppCx<'a>,
    frame: &'a mut ToolFrame,
    picks: &'a mut dyn PickService,
    scene: &'a SceneAdapter,
}

impl std::fmt::Debug for ToolCx<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ToolCx")
            .field("app", &self.app)
            .field("frame", &self.frame)
            .finish_non_exhaustive()
    }
}

impl<'a> ToolCx<'a> {
    pub(crate) fn new(
        app: AppCx<'a>,
        frame: &'a mut ToolFrame,
        picks: &'a mut dyn PickService,
        scene: &'a SceneAdapter,
    ) -> ToolCx<'a> {
        ToolCx {
            app,
            frame,
            picks,
            scene,
        }
    }

    /// The app context (submit, notify, jobs, commands).
    pub fn app(&mut self) -> &mut AppCx<'a> {
        &mut self.app
    }

    /// The read-only view.
    pub fn env(&self) -> CmdEnv<'_> {
        self.app.env()
    }

    /// Ask the router to finish the tool at end of frame (e.g. after the last click).
    pub fn request_finish(&mut self, reason: FinishReason) {
        self.frame.finish = Some(reason);
        self.app.request_repaint();
    }

    /// The newest hover pick and where it was taken.
    pub fn hover(&self) -> Option<&HoverHit> {
        self.frame.hover.as_ref()
    }

    /// The camera of the newest frame (`None` before the viewport was first drawn).
    pub fn camera(&self) -> Option<&CameraView> {
        self.frame.view.as_ref()
    }

    /// The document's preview driver.
    pub fn preview(&mut self) -> &mut PreviewDriver {
        &mut self.frame.preview
    }

    /// The selection.
    pub fn selection(&self) -> &Selection {
        self.app.selection()
    }

    /// The selection, mutable.
    pub fn selection_mut(&mut self) -> &mut Selection {
        self.app.selection_mut()
    }

    /// Cursor icon over the viewport while this tool is active.
    pub fn set_cursor(&mut self, icon: egui::CursorIcon) {
        self.frame.cursor = Some(icon);
    }

    /// The effective pick filter (`tool.pick_filter() ∩` the selection filter bar).
    pub fn pick_filter(&self) -> EntityFilter {
        self.frame.filter
    }

    /// Requests a point pick with the effective filter (additive). Answered by
    /// [`Self::poll_pick`] in a later frame.
    pub fn pick_point(&mut self, pos: egui::Pos2, mode: PickMode) -> PickTicket {
        let filter = self.frame.filter;
        let t = self.picks.request_point(pos, PICK_RADIUS_PT, filter, mode);
        self.app.request_repaint();
        t
    }

    /// The answer to [`Self::pick_point`] or [`Self::pick_region`], exactly once (additive).
    pub fn poll_pick(&mut self, t: PickTicket) -> Option<Vec<PickHitUi>> {
        self.picks.poll(t)
    }

    /// Requests a rectangle pick (`rect` in points) with the effective filter (additive,
    /// UI-04). Answered by [`Self::poll_pick`] in a later frame.
    pub fn pick_region(&mut self, rect: egui::Rect, mode: RegionMode) -> PickTicket {
        let filter = self.frame.filter;
        let t = self.picks.request_region(rect, filter, mode);
        self.app.request_repaint();
        t
    }

    /// What a hit selects under the effective filter and select priority (additive, UI-04;
    /// see [`item_for_hit`]).
    pub fn item_for(&self, hit: PickTarget) -> Option<SelItem> {
        item_for_hit(
            hit,
            self.frame.filter,
            self.frame.priority,
            self.app.env().evaluated,
            self.scene,
        )
    }

    /// The keyboard modifiers held in the newest routed viewport frame (additive, UI-09:
    /// Ctrl suppresses sketch inference while hovering).
    pub fn modifiers(&self) -> egui::Modifiers {
        self.frame.mods
    }

    /// The scene adapter (pick map, occurrence paths; additive, UI-04).
    pub fn scene(&self) -> &SceneAdapter {
        self.scene
    }
}

/// The active tool and the command that started it.
pub(crate) struct ActiveTool {
    pub(crate) id: CmdId,
    pub(crate) tool: Box<dyn Tool>,
}

/// Who owns the events of a held button.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Owner {
    Tool,
    Nav,
    Select,
}

/// Per-document router state.
#[derive(Default)]
pub(crate) struct ToolState {
    pub(crate) active: Option<ActiveTool>,
    pub(crate) select: SelectTool,
    pub(crate) frame: ToolFrame,
    synth: InputSynth,
    owners: Vec<(egui::PointerButton, Owner)>,
    hover_tickets: Vec<(PickTicket, egui::Pos2)>,
    hover_shown: Vec<PickTarget>,
    /// The hovered timeline error node's entities last written to the Error layer (UI-08); the
    /// layer is only rewritten when this changes, so `AppCx::set_highlight` users (MESH-23
    /// defects) keep it otherwise.
    errors_shown: Vec<PickTarget>,
    selection_shown: Option<u64>,
    /// The active tool draws a dialog (learnt from its first `dialog_ui`).
    pub(crate) dialog: Option<bool>,
    last_tick: Option<Instant>,
    /// The pointer was over the viewport in the last routed frame.
    pointer_inside: bool,
    /// The active tool's overlays of this frame ([`Tool::overlay`]; additive, INS-08).
    pub(crate) overlay: amg_render::OverlayList,
    /// The view's section planes and per-body style from the document layers (additive,
    /// INS-09).
    pub(crate) sections: crate::layer::ViewSections,
    /// The right-click marking menu (UI-21).
    pub(crate) marking: crate::marking::MarkingState,
}

impl std::fmt::Debug for ToolState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ToolState")
            .field("active", &self.active.as_ref().map(|a| a.id))
            .field("frame", &self.frame)
            .field("owners", &self.owners)
            .field("hover_tickets", &self.hover_tickets.len())
            .finish_non_exhaustive()
    }
}

impl ToolState {
    /// Hover picks and select clicks waiting for an answer.
    pub(crate) fn picks_pending(&self) -> bool {
        !self.hover_tickets.is_empty() || self.select.picks_pending()
    }

    fn owner(&self, b: egui::PointerButton) -> Option<Owner> {
        self.owners.iter().find(|(o, _)| *o == b).map(|(_, w)| *w)
    }

    fn set_owner(&mut self, b: egui::PointerButton, w: Owner) {
        self.owners.retain(|(o, _)| *o != b);
        self.owners.push((b, w));
    }

    fn release_owner(&mut self, b: egui::PointerButton) {
        self.owners.retain(|(o, _)| *o != b);
    }
}

/// Runs `f` with the active tool (or the select tool) and a [`ToolCx`] on the active document.
fn with_tool<R>(
    app: &mut AmalgamApp,
    ts: &mut ToolState,
    f: impl FnOnce(&mut dyn Tool, &mut ToolCx<'_>) -> R,
) -> R {
    let ToolState {
        active,
        select,
        frame,
        ..
    } = ts;
    let sel = &app.docs[app.active].selection;
    // The active selection field of the tool's dialog narrows what the tool picks (UI-15).
    let dialog = match (active.is_some(), frame.dialog_filter) {
        (true, Some(f)) => f,
        _ => EntityFilter::ALL,
    };
    let (tool, priority): (&mut dyn Tool, SelectPriority) = match active {
        Some(a) => (a.tool.as_mut(), SelectPriority::Default),
        None => (select, sel.priority()),
    };
    frame.filter = tool.pick_filter() & sel.filter() & priority.pick_filter() & dialog;
    frame.priority = priority;
    let (cx, picks, scene) = app.tool_parts();
    let mut tcx = ToolCx::new(cx, frame, picks, scene);
    f(tool, &mut tcx)
}

/// Takes the active document's router state out for the duration of `f`.
fn with_state<R>(app: &mut AmalgamApp, f: impl FnOnce(&mut AmalgamApp, &mut ToolState) -> R) -> R {
    let active = app.active;
    let mut ts = std::mem::take(&mut app.docs[active].tools);
    let r = f(app, &mut ts);
    // `f` may not switch documents; put the state back where it came from.
    app.docs[active].tools = ts;
    r
}

/// Runs the active document's layers (created on first use) in registration order.
fn run_doc_layers(
    app: &mut AmalgamApp,
    overlays: &mut amg_render::OverlayList,
    sections: &mut crate::layer::ViewSections,
    body_views: &mut Option<std::sync::Arc<crate::scene::BodyViews>>,
    active_tool: Option<CmdId>,
    body_tints: &mut Option<std::sync::Arc<crate::scene::BodyTints>>,
) {
    let active = app.active;
    app.docs[active].layers.ensure(app.registry.doc_layers());
    for i in 0..app.docs[active].layers.list.len() {
        let Some(mut layer) = app.docs[active].layers.list[i].1.take() else {
            continue;
        };
        {
            let (mut cx, _, _) = app.tool_parts();
            let mut out = crate::layer::LayerOut {
                overlays: &mut *overlays,
                sections: &mut *sections,
                body_views: &mut *body_views,
                active_tool,
                body_tints: &mut *body_tints,
            };
            layer.update(&mut cx, &mut out);
        }
        app.docs[active].layers.list[i].1 = Some(layer);
    }
}

/// Finishes the active tool with `reason`; a commit's batch is submitted (one undo entry at
/// the end of the frame). The preview is cleared.
fn finish_active(app: &mut AmalgamApp, ts: &mut ToolState, reason: FinishReason) {
    let Some(mut a) = ts.active.take() else {
        return;
    };
    tracing::debug!(tool = %a.id, ?reason, "tool finished");
    let (cx, picks, scene) = app.tool_parts();
    let mut tcx = ToolCx::new(cx, &mut ts.frame, picks, scene);
    let batch = a.tool.finish(reason, &mut tcx);
    if reason == FinishReason::Commit
        && let Some(b) = batch
    {
        tcx.app.submit(b);
    }
    tcx.app.request_repaint();
    ts.frame.preview.clear();
    ts.frame.finish = None;
    ts.frame.cursor = None;
    ts.frame.dialog_filter = None;
    ts.dialog = None;
    ts.owners.clear();
    ts.overlay.clear();
}

impl AmalgamApp {
    /// Starts a tool on the active document (the `StartTool` action of command `id`). The
    /// active tool finishes first with `Replaced`, or `Commit` if it commits on replace.
    pub(crate) fn start_tool_with(
        &mut self,
        id: CmdId,
        factory: fn(&ToolInit<'_>) -> Box<dyn Tool>,
    ) {
        self.start_tool_editing(id, factory, None);
    }

    /// Like `start_tool_with`, with `ToolInit::edit` set (edit an existing feature).
    pub(crate) fn start_tool_editing(
        &mut self,
        id: CmdId,
        factory: fn(&ToolInit<'_>) -> Box<dyn Tool>,
        edit: Option<amg_doc::FeatureId>,
    ) {
        with_state(self, |app, ts| {
            if let Some(a) = &ts.active {
                let reason = if a.tool.commit_on_replace() {
                    FinishReason::Commit
                } else {
                    FinishReason::Replaced
                };
                finish_active(app, ts, reason);
            }
            let tool = {
                let doc = &app.docs[app.active];
                let env = app.env_for(doc, ts.frame.hover.as_ref().map(|h| &h.item), None);
                factory(&ToolInit {
                    env,
                    preselection: doc.selection.items(),
                    edit,
                })
            };
            tracing::debug!(tool = %id, "tool started");
            ts.select.reset();
            ts.owners.clear();
            ts.active = Some(ActiveTool { id, tool });
            with_tool(app, ts, |t, cx| t.start(cx));
        });
    }

    /// The command of the active tool of the active document (`None`: the select tool).
    pub fn active_tool(&self) -> Option<CmdId> {
        self.docs[self.active].tools.active.as_ref().map(|a| a.id)
    }

    /// Finishes the active tool of the active document (as Esc / Enter / the dialog buttons
    /// do). Does nothing without one.
    pub fn finish_tool(&mut self, reason: FinishReason) {
        with_state(self, |app, ts| finish_active(app, ts, reason));
    }

    /// The newest hover pick of the active document.
    pub fn hover(&self) -> Option<&HoverHit> {
        self.docs[self.active].tools.frame.hover.as_ref()
    }

    /// The preview driver of the active document.
    pub fn preview_driver(&mut self) -> &mut PreviewDriver {
        &mut self.docs[self.active].tools.frame.preview
    }

    /// Picks, hover picks, select clicks or previews of the active document are pending
    /// (idle detection for tests and hosts).
    pub fn input_busy(&self) -> bool {
        let ts = &self.docs[self.active].tools;
        self.picks.pending() > 0 || ts.picks_pending() || ts.frame.preview.is_busy()
    }

    /// The active tool's overlays of the active document's last routed frame
    /// ([`Tool::overlay`]; additive, INS-08).
    pub fn tool_overlays(&self) -> &amg_render::OverlayList {
        &self.docs[self.active].tools.overlay
    }

    /// The section planes and per-body section style of the active document's last routed
    /// frame, from its document layers (additive, INS-09).
    pub fn view_sections(&self) -> &crate::layer::ViewSections {
        &self.docs[self.active].tools.sections
    }

    /// The active document's layer of type `T` (additive, INS-09; tests and scripts).
    pub fn doc_layer<T: crate::layer::DocLayer>(&self) -> Option<&T> {
        self.docs[self.active].layers.get::<T>()
    }

    /// The status hint of the active tool.
    pub(crate) fn tool_hint(&self) -> Option<Cow<'static, str>> {
        let a = self.docs[self.active].tools.active.as_ref()?;
        Some(a.tool.status_hint().unwrap_or_else(|| a.tool.label()))
    }

    /// `logic` part of the router: pick answers, hover, tool `update`, preview, finish
    /// requests, camera transitions and highlight layers. Never blocks.
    pub(crate) fn route_logic(&mut self) {
        let active = self.active;
        let view = self.docs[active].view.clone();
        self.picks.begin_frame(&PickFrame {
            scene: &self.scene,
            view: view.as_ref(),
        });
        self.docs[active].bounds = self.scene.bounds();
        if *self.docs[active].instance_paths != *self.scene.occurrence_paths() {
            self.docs[active].instance_paths = self.scene.occurrence_paths().to_vec();
        }
        with_state(self, |app, ts| {
            // Hover: the newest answer wins.
            let mut newest: Option<(Vec<PickHitUi>, egui::Pos2)> = None;
            let picks = &mut app.picks;
            ts.hover_tickets.retain(|(t, pos)| match picks.poll(*t) {
                Some(hits) => {
                    newest = Some((hits, *pos));
                    false
                }
                None => true,
            });
            if let Some((hits, at)) = newest {
                let eval = app.docs[active].document.evaluated();
                let (filter, priority) = (ts.frame.filter, ts.frame.priority);
                ts.frame.hover = hits.into_iter().find_map(|h| {
                    Some(HoverHit {
                        at,
                        item: item_for_hit(h.target, filter, priority, eval, &app.scene)?,
                        world: h.world,
                        depth: h.depth,
                    })
                });
            }
            if !ts.pointer_inside {
                ts.frame.hover = None;
            }
            with_tool(app, ts, |t, cx| t.update(cx));
            if let Some(reason) = ts.frame.finish.take() {
                finish_active(app, ts, reason);
            }
            // The active tool's overlays (INS-08); the select tool has none.
            let mut overlay = std::mem::take(&mut ts.overlay);
            overlay.clear();
            if ts.active.is_some() {
                with_tool(app, ts, |t, cx| t.overlay(&mut overlay, cx));
            }
            // Panel overlays (INS-03).
            {
                let env = crate::app::env_of_doc(app.workspace, &app.docs[active], &app.jobs);
                for panel in app.panels.values_mut() {
                    panel.overlay(&mut overlay, &env);
                }
            }
            // Document layers (INS-09): overlays and the view's section planes.
            let mut sections = crate::layer::ViewSections::default();
            let mut body_views = None;
            let active_tool = ts.active.as_ref().map(|a| a.id);
            let mut body_tints = None;
            run_doc_layers(
                app,
                &mut overlay,
                &mut sections,
                &mut body_views,
                active_tool,
                &mut body_tints,
            );
            // Body colours (INS-10): compared with what the scene shows, like the views.
            let same_tints = match (&body_tints, app.scene.body_tints()) {
                (None, None) => true,
                (Some(a), Some(b)) => std::sync::Arc::ptr_eq(a, b) || a == b,
                _ => false,
            };
            if !same_tints {
                app.scene.set_body_tints(body_tints);
            }
            if sections.style != ts.sections.style {
                app.scene.set_section_style(&sections.style);
            }
            // Body view overrides (AM-11): compared with what the scene shows, so a document
            // switch never keeps another document's overrides.
            let same = match (&body_views, app.scene.body_views()) {
                (None, None) => true,
                (Some(a), Some(b)) => std::sync::Arc::ptr_eq(a, b) || a == b,
                _ => false,
            };
            if !same {
                let doc = &app.docs[active];
                app.scene.set_body_views(
                    body_views,
                    doc.document.design(),
                    doc.document.evaluated(),
                );
            }
            ts.sections = sections;
            ts.overlay = overlay;
            // Preview → scene preview layer.
            let up = {
                let doc = &app.docs[active];
                ts.frame.preview.update(&doc.document, &app.jobs)
            };
            if ts.frame.preview.take_dirty() {
                let doc = &app.docs[active];
                app.scene.set_preview(
                    ts.frame.preview.shown().cloned(),
                    doc.document.design(),
                    doc.document.evaluated(),
                    &app.jobs,
                );
            }
            if let (Some(d), Some(ctx)) = (up.wake_after, &app.ctx) {
                ctx.request_repaint_after(d);
            }
            // Camera transitions.
            let doc = &mut app.docs[active];
            if doc.nav.is_animating() {
                let now = Instant::now();
                let dt = ts
                    .last_tick
                    .map_or(std::time::Duration::from_millis(16), |t| now - t);
                ts.last_tick = Some(now);
                if doc.nav.tick(&mut doc.camera, dt)
                    && let Some(ctx) = &app.ctx
                {
                    ctx.request_repaint();
                }
            } else {
                ts.last_tick = None;
            }
            // Highlights.
            // The viewport's hover, else the browser row under the pointer (UI-07), else the
            // timeline node's output bodies (UI-08).
            let hover = match ts
                .frame
                .hover
                .as_ref()
                .map(|h| h.item.target)
                .or_else(|| app.browser_hover_target())
            {
                Some(t) => vec![t],
                None => app.timeline_hover_targets(),
            };
            if hover != ts.hover_shown {
                let list = highlight_targets(&app.scene, hover.iter());
                app.scene.set_highlight(HighlightLayer::Hover, &list);
                ts.hover_shown = hover;
            }
            // A hovered error node's diagnostic entities (UI-08).
            let errors = app.timeline_error_targets();
            if errors != ts.errors_shown {
                app.scene.set_highlight(HighlightLayer::Error, &errors);
                ts.errors_shown = errors;
            }
            let sel = &app.docs[active].selection;
            if ts.selection_shown != Some(sel.revision()) {
                ts.selection_shown = Some(sel.revision());
                let list = highlight_targets(&app.scene, sel.items().iter().map(|i| &i.target));
                app.scene.set_highlight(HighlightLayer::Selected, &list);
            }
            if ts.picks_pending()
                && let Some(ctx) = &app.ctx
            {
                ctx.request_repaint();
            }
        });
    }

    /// Key and text events of this frame → the active tool, then Esc / Enter. Runs before
    /// the shortcut map; consumed key presses are removed from the frame's input.
    pub(crate) fn route_keys(&mut self, ctx: &egui::Context) {
        if self.search.is_open() || ctx.egui_wants_keyboard_input() {
            return;
        }
        let events = ctx.input(key_events);
        if events.is_empty() {
            return;
        }
        with_state(self, |app, ts| {
            for ev in events {
                let used = with_tool(app, ts, |t, cx| t.input(&ev, cx));
                let ViewportEvent::Key {
                    key,
                    pressed: true,
                    mods,
                } = ev
                else {
                    continue;
                };
                let handled = used == InputResult::Consumed
                    || match key {
                        // The marking menu is a popup: it closes before the tool cancels.
                        egui::Key::Escape if ts.marking.close() => true,
                        egui::Key::Escape if ts.active.is_some() => {
                            finish_active(app, ts, FinishReason::Cancel);
                            true
                        }
                        egui::Key::Escape if ts.select.close_through() => true,
                        egui::Key::Escape => {
                            let sel = &mut app.docs[app.active].selection;
                            let had = !sel.is_empty();
                            sel.clear();
                            had
                        }
                        egui::Key::Enter if ts.active.is_some() => {
                            finish_active(app, ts, FinishReason::Commit);
                            true
                        }
                        _ => false,
                    };
                if handled {
                    crate::shortcut::consume_press(ctx, key, mods);
                    ctx.request_repaint();
                }
            }
        });
    }

    /// Pointer events of this frame over the viewport `response` → tool / navigation /
    /// select tool; hover picks; view-cube and nav-bar camera actions.
    pub(crate) fn route_viewport(&mut self, ctx: &egui::Context, response: &egui::Response) {
        let active = self.active;
        let rect = response.rect;
        let ppp = ctx.pixels_per_point();
        let view = CameraView::new(self.docs[active].camera, rect, ppp);
        self.picks.set_view(&view);
        self.docs[active].view = Some(view.clone());
        let bounds = self.docs[active].bounds;
        for action in self.viewport.take_camera_actions() {
            let doc = &mut self.docs[active];
            let _ = doc.nav.apply(action, &mut doc.camera, bounds, view.size_px);
            ctx.request_repaint();
        }
        let excluded = self.viewport.input_exclusions();
        let layer = response.layer_id;
        with_state(self, |app, ts| {
            ts.frame.view = Some(view.clone());
            let accept = &|p: egui::Pos2| {
                rect.contains(p)
                    && !excluded.iter().any(|r| r.contains(p))
                    && ctx.layer_id_at(p).is_none_or(|l| l == layer)
            };
            // `accept` reads the context (layers): never call it under the input lock.
            let input = ctx.input(Clone::clone);
            ts.frame.mods = input.modifiers;
            let events = ts.synth.pointer_events(&input, &view, accept);
            ts.pointer_inside =
                input.pointer.latest_pos().is_some_and(accept) || ts.synth.is_pressed();
            if !ts.pointer_inside && ts.frame.hover.is_some() {
                ts.frame.hover = None;
                ctx.request_repaint();
            }
            let mut hover_at = None;
            for ev in &events {
                if let ViewportEvent::Hover { pos, .. } = ev {
                    hover_at = Some(*pos);
                }
                route_pointer(app, ts, ev, &view, bounds);
            }
            if let Some(pos) = hover_at {
                let filter = ts.frame.filter;
                let t = app
                    .picks
                    .request_point(pos, PICK_RADIUS_PT, filter, PickMode::Nearest);
                ts.hover_tickets.push((t, pos));
                ctx.request_repaint();
            }
            if !events.is_empty() {
                ctx.request_repaint();
            }
            if let Some(c) = ts.frame.cursor
                && response.contains_pointer()
            {
                ctx.set_cursor_icon(c);
            }
            if ts.active.is_none() {
                match ts.select.box_rect() {
                    Some((r, mode)) => {
                        paint_box(ctx, rect, r, mode, &app.tokens);
                        ts.frame.cursor = Some(egui::CursorIcon::Crosshair);
                    }
                    None => ts.frame.cursor = None,
                }
            }
        });
        // The camera may have moved: later readers of this frame see the new one.
        let cam = self.docs[active].camera;
        if cam != view.camera {
            let v = CameraView::new(cam, rect, ppp);
            self.picks.set_view(&v);
            self.docs[active].tools.frame.view = Some(v.clone());
            self.docs[active].view = Some(v);
        }
    }

    /// The active tool's floating dialog, anchored to the top right of the viewport.
    pub(crate) fn draw_tool_dialog(&mut self, ctx: &egui::Context) {
        let active = self.active;
        let Some(a) = &self.docs[active].tools.active else {
            return;
        };
        if self.docs[active].tools.dialog == Some(false) {
            return;
        }
        let label = a.tool.label();
        let anchor = self.docs[active]
            .view
            .as_ref()
            .map_or(egui::pos2(0.0, 0.0), |v| v.rect.right_top());
        with_state(self, |app, ts| {
            let mut shown = None;
            egui::Window::new(label.as_ref())
                .id(egui::Id::new("amg_tool_dialog"))
                .default_pos(anchor + egui::vec2(-12.0, 12.0))
                .pivot(egui::Align2::RIGHT_TOP)
                .collapsible(false)
                .resizable(false)
                .show(ctx, |ui| {
                    shown = Some(with_tool(app, ts, |t, cx| t.dialog_ui(ui, cx)));
                });
            ts.frame.dialog_filter = crate::widgets::active_selection_filter(ctx);
            if let Some(s) = shown {
                ts.dialog = Some(s);
                if !s {
                    ctx.request_repaint();
                }
            }
            if let Some(reason) = ts.frame.finish.take() {
                finish_active(app, ts, reason);
            }
        });
    }
}

/// Highlight targets for selected or hovered targets: an occurrence lights up every body seen
/// through it.
fn highlight_targets<'t>(
    scene: &SceneAdapter,
    targets: impl Iterator<Item = &'t PickTarget>,
) -> Vec<PickTarget> {
    let mut out = Vec::new();
    for t in targets {
        match t {
            PickTarget::Occurrence(o) => out.extend(scene.occurrence_bodies(*o)),
            other => out.push(*other),
        }
    }
    out
}

/// `targets` with every body target repeated for each displayed instance of its body
/// (`AppCx::set_highlight`, additive MESH-23).
pub(crate) fn all_instances(scene: &SceneAdapter, targets: &[PickTarget]) -> Vec<PickTarget> {
    let mut out = Vec::with_capacity(targets.len());
    let mut instances: Option<(amg_doc::BodyId, Vec<u32>)> = None;
    for t in targets {
        match *t {
            PickTarget::Body { body, sub, .. } => {
                if instances.as_ref().is_none_or(|(b, _)| *b != body) {
                    instances = Some((body, scene.pick_map().instances_of(body)));
                }
                match &instances {
                    // Not displayed yet: kept as given, resolved once it is.
                    Some((_, list)) if list.is_empty() => out.push(*t),
                    Some((_, list)) => out.extend(list.iter().map(|&instance| PickTarget::Body {
                        body,
                        instance,
                        sub,
                    })),
                    None => {}
                }
            }
            other => out.push(other),
        }
    }
    out
}

/// The selection box: a window has a solid outline, a crossing a dashed one, both over a light
/// fill.
fn paint_box(
    ctx: &egui::Context,
    clip: egui::Rect,
    r: egui::Rect,
    mode: RegionMode,
    tokens: &crate::theme::ThemeTokens,
) {
    let painter = ctx
        .layer_painter(egui::LayerId::new(
            egui::Order::Foreground,
            egui::Id::new("amg_select_box"),
        ))
        .with_clip_rect(clip);
    let color = match mode {
        RegionMode::Window => tokens.accent,
        RegionMode::Crossing => tokens.hover,
    };
    painter.rect_filled(r, 0.0, color.gamma_multiply(0.12));
    let stroke = egui::Stroke::new(1.0, color);
    match mode {
        RegionMode::Window => {
            painter.rect_stroke(r, 0.0, stroke, egui::StrokeKind::Inside);
        }
        RegionMode::Crossing => {
            let pts = [
                r.left_top(),
                r.right_top(),
                r.right_bottom(),
                r.left_bottom(),
                r.left_top(),
            ];
            painter.extend(egui::Shape::dashed_line(&pts, stroke, 4.0, 3.0));
        }
    }
}

impl AmalgamApp {
    /// The select-through list of the active document, when open (UI-04).
    pub fn select_through(&self) -> Option<&crate::select::SelectThrough> {
        let ts = &self.docs[self.active].tools;
        if ts.active.is_some() {
            return None;
        }
        ts.select.select_through()
    }

    /// Chooses entry `index` of the open select-through list: it replaces the selection and
    /// the list closes. False without a list or for an index out of range (UI-04).
    pub fn choose_select_through(&mut self, index: usize) -> bool {
        let active = self.active;
        let doc = &mut self.docs[active];
        let Some(entry) = doc
            .tools
            .select
            .select_through()
            .and_then(|l| l.entries.get(index))
            .map(|e| e.item.clone())
        else {
            return false;
        };
        doc.tools.select.close_through();
        doc.selection.set(vec![entry]);
        if let Some(ctx) = &self.ctx {
            ctx.request_repaint();
        }
        true
    }

    /// The select-through list next to the long-pressed point.
    pub(crate) fn draw_select_through(&mut self, ctx: &egui::Context) {
        let Some(list) = self.select_through().cloned() else {
            return;
        };
        let mut chosen = None;
        let selection = &self.docs[self.active].selection;
        egui::Area::new(egui::Id::new("amg_select_through"))
            .order(egui::Order::Foreground)
            .fixed_pos(list.at + egui::vec2(12.0, 12.0))
            .show(ctx, |ui| {
                egui::Frame::popup(ui.style()).show(ui, |ui| {
                    ui.label(egui::RichText::new("Select through").strong());
                    for (i, e) in list.entries.iter().enumerate() {
                        let on = selection.contains(&e.item.target);
                        if ui.selectable_label(on, &e.label).clicked() {
                            chosen = Some(i);
                        }
                    }
                });
            });
        if let Some(i) = chosen {
            self.choose_select_through(i);
        }
    }
}

/// One pointer event through tool → navigation → select tool.
fn route_pointer(
    app: &mut AmalgamApp,
    ts: &mut ToolState,
    ev: &ViewportEvent,
    view: &CameraView,
    bounds: amg_core::Aabb3,
) {
    let has_tool = ts.active.is_some();
    let anchor = |ts: &ToolState, pos: egui::Pos2| {
        ts.frame
            .hover
            .as_ref()
            .filter(|h| h.at.distance(pos) <= 8.0)
            .map(|h| h.world)
    };
    let nav = |app: &mut AmalgamApp, ev: &ViewportEvent, anchor: Option<Point3>| {
        let doc = &mut app.docs[app.active];
        doc.nav.input(ev, &mut doc.camera, view, anchor, bounds)
    };
    let tool =
        |app: &mut AmalgamApp, ts: &mut ToolState| with_tool(app, ts, |t, cx| t.input(ev, cx));
    // The open marking menu / a tracked right press (popup step, UI-21).
    if crate::marking::intercept(app, ts, ev) {
        return;
    }
    match ev {
        ViewportEvent::Press { button, pos, .. } => {
            let used = tool(app, ts) == InputResult::Consumed;
            if used && has_tool {
                ts.set_owner(*button, Owner::Tool);
            } else if !used && crate::marking::starts_gesture(ev) {
                crate::marking::begin(app, ts, *pos);
            }
        }
        ViewportEvent::DragStart { button, from, .. } => match ts.owner(*button) {
            Some(Owner::Tool) => {
                tool(app, ts);
            }
            _ => {
                if has_tool && tool(app, ts) == InputResult::Consumed {
                    ts.set_owner(*button, Owner::Tool);
                } else if nav(app, ev, anchor(ts, *from)) == InputResult::Consumed {
                    ts.set_owner(*button, Owner::Nav);
                    // The pointer moved away from the hover answer.
                    ts.frame.hover = None;
                } else if !has_tool && tool(app, ts) == InputResult::Consumed {
                    ts.set_owner(*button, Owner::Select);
                }
            }
        },
        ViewportEvent::Drag { button, .. } | ViewportEvent::Release { button, .. } => {
            match ts.owner(*button) {
                Some(Owner::Nav) => {
                    nav(app, ev, None);
                }
                Some(Owner::Tool | Owner::Select) | None => {
                    tool(app, ts);
                }
            }
        }
        ViewportEvent::DragEnd { button, .. } => {
            match ts.owner(*button) {
                Some(Owner::Nav) => {
                    nav(app, ev, None);
                }
                _ => {
                    tool(app, ts);
                }
            }
            ts.release_owner(*button);
        }
        ViewportEvent::Click { button, .. } => {
            if ts.owner(*button) != Some(Owner::Nav) {
                tool(app, ts);
            }
            ts.release_owner(*button);
        }
        ViewportEvent::Scroll { pos, .. } => {
            if !has_tool || tool(app, ts) == InputResult::Ignored {
                let a = anchor(ts, *pos);
                if nav(app, ev, a) == InputResult::Ignored && !has_tool {
                    tool(app, ts);
                }
            }
        }
        ViewportEvent::Hover { .. } => {
            tool(app, ts);
        }
        ViewportEvent::Key { .. } | ViewportEvent::Text(_) => {}
    }
}

/// The default tool lives in [`crate::select`] (UI-04); re-exported here for UI-03 paths.
pub use crate::select::{SELECT_TOOL, SelectTool};
