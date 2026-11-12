//! The fixed shell regions (note §2): app bar, toolbar, browser, timeline, status bar and the
//! central dock area.
//!
//! Every interactive element is a standard egui widget with a text label, so kittest finds it
//! through AccessKit (`get_by_label`).

use std::collections::BTreeMap;

use egui::{Button, RichText};

use crate::app::{AmalgamApp, AppCx, CmdEnv, CommandQueue, DocTab, ShellRequest};
use crate::command::{CmdId, CommandDef};
use crate::design::SKETCH_CONTEXT;
use crate::dock::{DockContent, DockTab};
use crate::module::{PanelId, WorkspaceId};
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
    shortcuts(app, ui.ctx());
    egui::Panel::top("amg_app_bar").show(ui, |ui| app_bar(app, ui));
    egui::Panel::top("amg_toolbar").show(ui, |ui| toolbar(app, ui));
    egui::Panel::bottom("amg_status_bar")
        .exact_size(STATUS_BAR_HEIGHT)
        .show(ui, |ui| status_bar(app, ui));
    egui::Panel::bottom("amg_timeline")
        .exact_size(TIMELINE_HEIGHT)
        .show(ui, |ui| timeline(app, ui));
    egui::Panel::left("amg_browser")
        .default_size(BROWSER_WIDTH)
        .show(ui, |ui| browser(app, ui));
    egui::CentralPanel::no_frame().show(ui, |ui| dock_area(app, ui));
    search_popup(app, ui.ctx());
}

/// The S-key search popup; a picked command is queued like any other.
fn search_popup(app: &mut AmalgamApp, ctx: &egui::Context) {
    if !app.search.is_open() {
        return;
    }
    let doc = &app.docs[app.active];
    let env = CmdEnv {
        workspace: app.workspace,
        contexts: &doc.contexts,
        design: doc.document.design(),
        evaluated: doc.document.evaluated(),
        jobs: &app.jobs,
    };
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
        Button::selectable(def.checked(env), def.label.as_ref())
    } else {
        Button::new(def.label.as_ref())
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
    let doc = &app.docs[app.active];
    CmdEnv {
        workspace: app.workspace,
        contexts: &doc.contexts,
        design: doc.document.design(),
        evaluated: doc.document.evaluated(),
        jobs: &app.jobs,
    }
}

fn app_bar(app: &mut AmalgamApp, ui: &mut egui::Ui) {
    let mut run: Option<CmdId> = None;
    let mut new_doc = false;
    ui.horizontal(|ui| {
        {
            let reg = &app.registry;
            let env = env_of(app);
            ui.menu_button("File", |ui| {
                if ui.button("New document").clicked() {
                    new_doc = true;
                    ui.close();
                }
                menu_level(ui, reg, "File", &env, &app.shortcuts, &mut run);
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
            .add_enabled(can_undo, Button::new("Undo"))
            .on_hover_text(undo_tip)
            .on_disabled_hover_text("Nothing to undo")
            .clicked()
        {
            app.request_undo(id);
        }
        if ui
            .add_enabled(can_redo, Button::new("Redo"))
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
    if new_doc {
        app.new_document();
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
    let doc = &app.docs[app.active];
    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| {
            egui::CollapsingHeader::new(doc.name.as_str())
                .id_salt(("amg_browser_root", doc.id))
                .default_open(true)
                .show(ui, |ui| {
                    for node in [
                        "Document Settings",
                        "Named Views",
                        "Origin",
                        "Bodies",
                        "Sketches",
                    ] {
                        ui.label(node);
                    }
                });
        });
}

fn timeline(app: &mut AmalgamApp, ui: &mut egui::Ui) {
    let doc = &app.docs[app.active];
    ui.horizontal_centered(|ui| {
        for (label, tip) in [
            ("⏮", "Go to start"),
            ("⏴", "Step back"),
            ("⏵", "Step forward"),
            ("⏭", "Go to end"),
        ] {
            let _ = ui
                .add_enabled(false, Button::new(label))
                .on_disabled_hover_text(tip);
        }
        ui.separator();
        let tl = doc.document.design().timeline();
        if tl.is_empty() {
            ui.weak("Timeline is empty");
        }
        for node in tl.nodes() {
            let name = node
                .name
                .clone()
                .unwrap_or_else(|| node.feature.kind().to_owned());
            let _ = ui.add(Button::new(name).small());
        }
    });
}

fn status_bar(app: &mut AmalgamApp, ui: &mut egui::Ui) {
    let doc = &app.docs[app.active];
    let lost = app.viewport.device_lost();
    let jobs = app.jobs.jobs().len();
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
        if lost {
            ui.separator();
            ui.colored_label(
                app.tokens.error,
                "GPU device lost — save your work and restart Amalgam",
            );
        }
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.label("mm");
            ui.separator();
            ui.label(format!("Jobs: {jobs}"));
        });
    });
}

fn dock_area(app: &mut AmalgamApp, ui: &mut egui::Ui) {
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
        ..
    } = app;
    let Some(dock) = layouts.get_mut(&ws) else {
        return;
    };
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
    };
    dock.show(ui, &mut content);
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
                };
                self.viewport.show(ui, &frame);
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
                    self.workspace,
                    self.ctx,
                );
                panel.ui(ui, &mut cx);
            }
        }
    }
}
