//! The fixed shell regions (note §2): app bar, toolbar, browser, timeline, status bar and the
//! central dock area.
//!
//! Every interactive element is a standard egui widget with a text label, so kittest finds it
//! through AccessKit (`get_by_label`).

use std::collections::BTreeMap;

use egui::{Button, RichText};

use crate::app::{AmalgamApp, AppCx, CommandQueue, DocTab, ShellRequest};
use crate::dock::{DockContent, DockTab};
use crate::module::{PanelId, WorkspaceId};
use crate::panel::Panel;
use crate::registry::UiRegistry;
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
}

/// Undo/redo keys of the active document (UI-02 moves these into the `ShortcutMap`).
fn shortcuts(app: &mut AmalgamApp, ctx: &egui::Context) {
    if ctx.egui_wants_keyboard_input() {
        return;
    }
    let undo = egui::KeyboardShortcut::new(egui::Modifiers::COMMAND, egui::Key::Z);
    let redo = egui::KeyboardShortcut::new(egui::Modifiers::COMMAND, egui::Key::Y);
    let redo_shift = egui::KeyboardShortcut::new(
        egui::Modifiers::COMMAND | egui::Modifiers::SHIFT,
        egui::Key::Z,
    );
    let id = app.active_document().id;
    let (u, r) = ctx.input_mut(|i| {
        let r = i.consume_shortcut(&redo_shift) | i.consume_shortcut(&redo);
        (i.consume_shortcut(&undo), r)
    });
    if u {
        app.request_undo(id);
    }
    if r {
        app.request_redo(id);
    }
}

fn app_bar(app: &mut AmalgamApp, ui: &mut egui::Ui) {
    ui.horizontal(|ui| {
        ui.menu_button("File", |ui| {
            if ui.button("New document").clicked() {
                app.new_document();
                ui.close();
            }
        });
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
}

fn toolbar(app: &mut AmalgamApp, ui: &mut egui::Ui) {
    let reg = &app.registry;
    let ws = app.workspace;
    let contexts = &app.docs[app.active].contexts;
    let active_tab = app.active_tab();
    let mut switch_to: Option<WorkspaceId> = None;
    let mut select = None;

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
            ui.menu_button(format!("{} ⏷", g.label.to_uppercase()), |ui| {
                ui.weak("No commands yet");
            });
        }
    });

    if let Some(t) = select {
        app.select_tab(t);
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
