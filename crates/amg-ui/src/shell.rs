//! The fixed shell regions (note §2): app bar, toolbar, browser, timeline, status bar and the
//! central dock area.
//!
//! Every interactive element is a standard egui widget with a text label, so kittest finds it
//! through AccessKit (`get_by_label`).

use std::collections::BTreeMap;

use egui::RichText;

use crate::app::{AmalgamApp, AppCx, CmdEnv, CommandQueue, DocTab, ShellRequest};
use crate::command::{CmdId, CommandDef};
use crate::design::SKETCH_CONTEXT;
use crate::dock::{DockContent, DockTab};
use crate::jobs::JobTracker;
use crate::module::{PanelId, WorkspaceId};
use crate::notify::Notifications;
use crate::panel::Panel;
use crate::registry::UiRegistry;
use crate::shortcut::{ShortcutMap, active_scopes, consume_press, key_presses};
use crate::theme::{Theme, ThemeTokens};
use crate::viewport::{ViewportBackend, ViewportFrame};

/// Height of the status bar, points.
const STATUS_BAR_HEIGHT: f32 = 22.0;
/// Height of the timeline strip, points.
const TIMELINE_HEIGHT: f32 = 34.0;
/// Default browser width, points.
const BROWSER_WIDTH: f32 = 230.0;

pub(crate) fn draw(app: &mut AmalgamApp, ui: &mut egui::Ui) {
    // Tools see key events before the shortcut map (routing priority, UI-03).
    app.route_keys(ui.ctx());
    shortcuts(app, ui.ctx());
    egui::Panel::top("amg_app_bar").show(ui, |ui| app_bar(app, ui));
    egui::Panel::top("amg_toolbar").show(ui, |ui| toolbar(app, ui));
    if app.viewport.device_lost() {
        egui::Panel::top("amg_gpu_banner")
            .frame(
                egui::Frame::new()
                    .fill(app.tokens.error)
                    .inner_margin(egui::Margin::symmetric(8, 4)),
            )
            .show(ui, gpu_banner);
    }
    egui::Panel::bottom("amg_status_bar")
        .exact_size(STATUS_BAR_HEIGHT)
        .show(ui, |ui| status_bar(app, ui));
    // The timeline is hidden in Direct mode (UI-08).
    if app.active_document().document.design().meta().mode != amg_doc::DesignMode::Direct {
        egui::Panel::bottom("amg_timeline")
            .exact_size(TIMELINE_HEIGHT)
            .show(ui, |ui| app.timeline_ui(ui));
    }
    egui::Panel::left("amg_browser")
        .default_size(BROWSER_WIDTH)
        .show(ui, |ui| browser(app, ui));
    let viewport = egui::CentralPanel::no_frame()
        .show(ui, |ui| dock_area(app, ui))
        .inner;
    if let Some(response) = viewport {
        app.route_viewport(ui.ctx(), &response);
    }
    app.draw_tool_dialog(ui.ctx());
    app.draw_dialogs(ui.ctx());
    crate::select::bar::show(app, ui.ctx());
    app.draw_select_through(ui.ctx());
    app.draw_marking_menu(ui.ctx());
    search_popup(app, ui.ctx());
    job_panel(app, ui.ctx());
    toasts(app, ui.ctx());
    app.draw_file_dialogs(ui.ctx());
}

/// The text of the device-lost banner (saving needs no GPU, so nothing is disabled).
const GPU_LOST_TEXT: &str = "GPU device lost — save your work and restart Amalgam";

fn gpu_banner(ui: &mut egui::Ui) {
    ui.add(egui::Label::new(
        RichText::new(GPU_LOST_TEXT)
            .strong()
            .color(egui::Color32::WHITE),
    ));
}

/// The expandable job panel (bottom right, above the status bar and timeline).
fn job_panel(app: &mut AmalgamApp, ctx: &egui::Context) {
    if app.jobs_panel_open {
        crate::jobs::panel(ctx, &app.jobs, STATUS_BAR_HEIGHT + TIMELINE_HEIGHT + 16.0);
    }
}

/// Drops the toasts that timed out, draws the rest and applies what the user clicked.
fn toasts(app: &mut AmalgamApp, ctx: &egui::Context) {
    if let Some(next) = app.notifications.expire() {
        ctx.request_repaint_after(next);
    }
    let actions = crate::notify::show(ctx, &app.notifications, &app.tokens);
    let changed = actions.open_log || !actions.dismissed.is_empty();
    for id in actions.dismissed {
        app.notifications.dismiss(id);
    }
    if actions.open_log {
        app.open_log_folder();
    }
    if changed {
        ctx.request_repaint();
    }
}

/// The S-key search popup; a picked command is queued like any other.
fn search_popup(app: &mut AmalgamApp, ctx: &egui::Context) {
    if !app.search.is_open() {
        return;
    }
    let env = crate::app::env_of_doc(app.workspace, &app.docs[app.active], &app.jobs);
    let picked = crate::search::show(
        ctx,
        &mut app.search,
        &app.registry,
        &app.recency,
        &env,
        &app.shortcuts,
    );
    if let Some(id) = picked {
        app.request_command(id);
    }
}

/// Looks up this frame's key presses in the [`ShortcutMap`] and queues the commands.
///
/// Skipped while the search popup is open (it owns the keys). While a text field has focus only
/// Ctrl/Alt chords fire, and never the ones a text field edits with.
fn shortcuts(app: &mut AmalgamApp, ctx: &egui::Context) {
    if app.search.is_open() {
        return;
    }
    let sketch = app.docs[app.active].contexts.contains(&SKETCH_CONTEXT);
    let scopes = active_scopes(sketch, app.viewport_hovered);
    let text_focus = ctx.egui_wants_keyboard_input();
    for (key, mods) in key_presses(ctx) {
        if let Some(cmd) = app
            .shortcuts
            .resolve_for_input(key, mods, &scopes, text_focus)
        {
            consume_press(ctx, key, mods);
            app.request_command(cmd);
        }
    }
}

/// A button for a command, shared by the toolbar and the menus: label, shortcut text,
/// checked state for toggles, and the disabled reason as tooltip.
fn command_button(
    ui: &mut egui::Ui,
    def: &CommandDef,
    env: &CmdEnv<'_>,
    shortcuts: &ShortcutMap,
) -> bool {
    let enablement = def.enablement(env);
    let mut button = if def.is_toggle() {
        {
            let on = def.checked(env);
            crate::icons::button(ui, def.icon, def.label.as_ref(), enablement.is_enabled())
                .selected(on)
                .frame_when_inactive(on)
                .frame(true)
        }
    } else {
        crate::icons::button(ui, def.icon, def.label.as_ref(), enablement.is_enabled())
    };
    if let Some(sc) = shortcuts.display_of(def.id) {
        button = button.shortcut_text(sc);
    }
    let resp = ui.add_enabled(enablement.is_enabled(), button);
    let resp = match enablement.reason() {
        Some(reason) => resp.on_disabled_hover_text(reason),
        None if !def.tooltip.is_empty() => resp.on_hover_text(def.tooltip.as_ref()),
        None => resp,
    };
    resp.clicked()
}

/// The command buttons of a menu level and its submenus, recursively.
fn menu_level(
    ui: &mut egui::Ui,
    reg: &UiRegistry,
    path: &str,
    env: &CmdEnv<'_>,
    shortcuts: &ShortcutMap,
    run: &mut Option<CmdId>,
) {
    for def in reg.menu_commands(path) {
        if command_button(ui, def, env, shortcuts) {
            *run = Some(def.id);
            ui.close();
        }
    }
    for child in reg.menu_children(path) {
        let sub = if path.is_empty() {
            child.to_owned()
        } else {
            format!("{path}/{child}")
        };
        ui.menu_button(child, |ui| menu_level(ui, reg, &sub, env, shortcuts, run));
    }
}

fn env_of<'a>(app: &'a AmalgamApp) -> CmdEnv<'a> {
    crate::app::env_of_doc(app.workspace, &app.docs[app.active], &app.jobs)
}

/// Menu slots that always exist in the File menu; lane G fills them (IO-01) with
/// `Placement::Menu { path: "File/Import" | "File/Export", .. }`.
const FILE_SLOTS: [&str; 2] = ["Import", "Export"];

/// The File menu (UI-19): the registered File commands (New, Open…, Save…, Close), Open
/// Recent, the Import/Export slots and any other registered File submenu.
fn file_menu(
    ui: &mut egui::Ui,
    reg: &UiRegistry,
    env: &CmdEnv<'_>,
    shortcuts: &ShortcutMap,
    recent: &[std::path::PathBuf],
    run: &mut Option<CmdId>,
    open_recent: &mut Option<std::path::PathBuf>,
) {
    for def in reg.menu_commands("File") {
        if command_button(ui, def, env, shortcuts) {
            *run = Some(def.id);
            ui.close();
        }
    }
    ui.menu_button("Open Recent", |ui| {
        if recent.is_empty() {
            ui.weak("No recent files");
        }
        for p in recent {
            let name = p.file_name().map_or_else(
                || p.display().to_string(),
                |n| n.to_string_lossy().into_owned(),
            );
            if ui
                .button(name)
                .on_hover_text(p.display().to_string())
                .clicked()
            {
                *open_recent = Some(p.clone());
                ui.close();
            }
        }
    });
    ui.separator();
    let children = reg.menu_children("File");
    for slot in FILE_SLOTS {
        if !children.contains(&slot) {
            ui.menu_button(slot, |ui| {
                ui.weak("No formats registered yet");
            });
        }
    }
    for child in children {
        let sub = format!("File/{child}");
        ui.menu_button(child, |ui| menu_level(ui, reg, &sub, env, shortcuts, run));
    }
}

fn app_bar(app: &mut AmalgamApp, ui: &mut egui::Ui) {
    let mut run: Option<CmdId> = None;
    let mut open_recent: Option<std::path::PathBuf> = None;
    ui.horizontal(|ui| {
        {
            let reg = &app.registry;
            let env = env_of(app);
            let recent = app.files.recent.files();
            ui.menu_button("File", |ui| {
                file_menu(
                    ui,
                    reg,
                    &env,
                    &app.shortcuts,
                    recent,
                    &mut run,
                    &mut open_recent,
                );
            });
            for top in reg.menu_children("") {
                if top != "File" {
                    ui.menu_button(top, |ui| {
                        menu_level(ui, reg, top, &env, &app.shortcuts, &mut run);
                    });
                }
            }
        }
        ui.separator();

        let doc = app.active_document();
        let id = doc.id;
        let history = doc.document.history();
        let (can_undo, can_redo) = (history.can_undo(), history.can_redo());
        let undo_tip = doc.document.undo_label().map_or_else(
            || "Undo (Ctrl+Z)".to_owned(),
            |l| format!("Undo {l} (Ctrl+Z)"),
        );
        let redo_tip = doc.document.redo_label().map_or_else(
            || "Redo (Ctrl+Y)".to_owned(),
            |l| format!("Redo {l} (Ctrl+Y)"),
        );
        if ui
            .add_enabled(
                can_undo,
                crate::icons::button(ui, crate::icons::ids::UNDO, "Undo", can_undo),
            )
            .on_hover_text(undo_tip)
            .on_disabled_hover_text("Nothing to undo")
            .clicked()
        {
            app.request_undo(id);
        }
        if ui
            .add_enabled(
                can_redo,
                crate::icons::button(ui, crate::icons::ids::REDO, "Redo", can_redo),
            )
            .on_hover_text(redo_tip)
            .on_disabled_hover_text("Nothing to redo")
            .clicked()
        {
            app.request_redo(id);
        }
        ui.separator();

        let mut activate = None;
        for (i, d) in app.documents().iter().enumerate() {
            if ui
                .selectable_label(i == app.active, d.name.as_str())
                .clicked()
            {
                activate = Some(d.id);
            }
        }
        if let Some(d) = activate {
            app.activate(d);
        }
        if ui.button("+").on_hover_text("New document").clicked() {
            app.new_document();
        }

        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.menu_button("Theme", |ui| {
                for t in Theme::ALL {
                    if ui.radio(app.theme == t, t.label()).clicked() {
                        app.set_theme(t);
                        ui.close();
                    }
                }
            });
        });
    });
    if let Some(p) = open_recent {
        app.open_path(p);
    }
    if let Some(c) = run {
        app.request_command(c);
    }
}

fn toolbar(app: &mut AmalgamApp, ui: &mut egui::Ui) {
    let reg = &app.registry;
    let ws = app.workspace;
    let contexts = &app.docs[app.active].contexts;
    let active_tab = app.active_tab();
    let mut switch_to: Option<WorkspaceId> = None;
    let mut select = None;
    let mut run: Option<CmdId> = None;
    let env = env_of(app);
    let shortcuts = &app.shortcuts;

    ui.horizontal(|ui| {
        let ws_label = reg.workspace(ws).map_or("WORKSPACE", |w| w.label);
        ui.menu_button(format!("{} ⏷", ws_label.to_uppercase()), |ui| {
            for w in reg.workspaces() {
                if ui.selectable_label(w.id == ws, w.label).clicked() {
                    switch_to = Some(w.id);
                    ui.close();
                }
            }
        });
        ui.separator();
        for t in reg.visible_tabs(ws, contexts) {
            let text = t.label.to_uppercase();
            let text = if t.contextual.is_some() {
                RichText::new(text).strong()
            } else {
                RichText::new(text)
            };
            if ui
                .selectable_label(Some(t.id) == active_tab, text)
                .clicked()
            {
                select = Some(t.id);
            }
        }
    });
    ui.horizontal(|ui| {
        let groups = active_tab.map(|t| reg.groups(t)).unwrap_or_default();
        if groups.is_empty() {
            ui.weak("No tools in this tab yet");
        }
        for g in groups {
            for (pinned, def) in reg.group_commands(g.id) {
                if pinned && command_button(ui, def, &env, shortcuts) {
                    run = Some(def.id);
                }
            }
            ui.menu_button(format!("{} ⏷", g.label.to_uppercase()), |ui| {
                let mut any = false;
                for (_, def) in reg.group_commands(g.id) {
                    any = true;
                    if command_button(ui, def, &env, shortcuts) {
                        run = Some(def.id);
                        ui.close();
                    }
                }
                if !any {
                    ui.weak("No commands yet");
                }
            });
        }
    });

    if let Some(t) = select {
        app.select_tab(t);
    }
    if let Some(c) = run {
        app.request_command(c);
    }
    if let Some(w) = switch_to {
        app.set_workspace(w);
    }
}

fn browser(app: &mut AmalgamApp, ui: &mut egui::Ui) {
    ui.strong("BROWSER");
    ui.separator();
    app.browser_ui(ui);
}

fn status_bar(app: &mut AmalgamApp, ui: &mut egui::Ui) {
    let doc = &app.docs[app.active];
    let jobs = app.jobs.jobs().len();
    let mut toggle_jobs = false;
    ui.horizontal_centered(|ui| {
        match app.status.as_deref() {
            Some(msg) => ui.colored_label(app.tokens.error, msg),
            None => ui.label("Ready"),
        };
        ui.separator();
        let recompute = match doc.document.recompute_status() {
            amg_doc::RecomputeStatus::Idle => "Up to date".to_owned(),
            amg_doc::RecomputeStatus::Stale => "Recompute pending".to_owned(),
            amg_doc::RecomputeStatus::Running { progress, .. } => {
                format!("Recomputing {:.0} %", progress * 100.0)
            }
            amg_doc::RecomputeStatus::Failed(_) => "Recompute failed".to_owned(),
        };
        ui.label(recompute);
        if let Some(hint) = app.tool_hint() {
            ui.separator();
            ui.label(hint.as_ref());
        }
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.label("mm");
            ui.separator();
            // A clickable label, not a button: the status bar keeps its UI-01 metrics.
            let text = RichText::new(format!("Jobs: {jobs}"));
            let text = if app.jobs_panel_open {
                text.strong()
            } else {
                text
            };
            if ui
                .add(egui::Label::new(text).sense(egui::Sense::click()))
                .on_hover_cursor(egui::CursorIcon::PointingHand)
                .on_hover_text("Show or hide the job panel")
                .clicked()
            {
                toggle_jobs = true;
            }
        });
    });
    if toggle_jobs {
        app.set_jobs_panel_open(!app.jobs_panel_open);
    }
}

/// The dock area; returns the viewport tab's response when it was drawn this frame.
fn dock_area(app: &mut AmalgamApp, ui: &mut egui::Ui) -> Option<egui::Response> {
    let ws = app.workspace;
    let AmalgamApp {
        registry,
        jobs,
        ctx,
        docs,
        active,
        tokens,
        layouts,
        panels,
        viewport,
        queue,
        requests,
        viewport_hovered,
        notifications,
        tracker,
        ..
    } = app;
    let dock = layouts.get_mut(&ws)?;
    let mut viewport_response = None;
    let mut content = ShellDock {
        registry,
        jobs,
        ctx: ctx.as_ref(),
        doc: &mut docs[*active],
        tokens,
        panels,
        viewport: viewport.as_mut(),
        queue,
        requests,
        workspace: ws,
        viewport_hovered,
        notifications,
        tracker,
        viewport_response: &mut viewport_response,
    };
    dock.show(ui, &mut content);
    viewport_response
}

struct ShellDock<'a> {
    registry: &'a UiRegistry,
    jobs: &'a amg_jobs::JobSystem,
    ctx: Option<&'a egui::Context>,
    doc: &'a mut DocTab,
    tokens: &'a ThemeTokens,
    panels: &'a mut BTreeMap<PanelId, Box<dyn Panel>>,
    viewport: &'a mut dyn ViewportBackend,
    queue: &'a mut CommandQueue,
    requests: &'a mut Vec<ShellRequest>,
    workspace: WorkspaceId,
    viewport_hovered: &'a mut bool,
    notifications: &'a mut Notifications,
    tracker: &'a mut JobTracker,
    viewport_response: &'a mut Option<egui::Response>,
}

impl DockContent for ShellDock<'_> {
    fn title(&mut self, tab: DockTab) -> egui::WidgetText {
        match tab {
            DockTab::Viewport => "Viewport".into(),
            DockTab::Panel(p) => self.registry.panel(p).map_or(p.0, |d| d.title).into(),
        }
    }

    fn ui(&mut self, ui: &mut egui::Ui, tab: DockTab) {
        match tab {
            DockTab::Viewport => {
                let frame = ViewportFrame {
                    tokens: self.tokens,
                    doc: self.doc.id,
                    camera: &self.doc.camera,
                    overlays: &self.doc.tools.overlay,
                    sections: &self.doc.tools.sections.planes,
                };
                *self.viewport_response = Some(self.viewport.show(ui, &frame));
                *self.viewport_hovered = ui.rect_contains_pointer(ui.max_rect());
            }
            DockTab::Panel(p) => {
                let Some(panel) = self.panels.get_mut(&p) else {
                    ui.weak("Panel not available");
                    return;
                };
                let mut cx = AppCx::new(
                    self.doc,
                    self.queue,
                    self.requests,
                    self.registry,
                    self.jobs,
                    self.notifications,
                    self.tracker,
                    self.workspace,
                    self.ctx,
                );
                panel.ui(ui, &mut cx);
            }
        }
    }
}
