//! [`DialogFrame`] (spec §9): help link, preview-health banner, body, OK / Cancel.

use amg_doc::Health;

use crate::command::Enablement;

/// What the user did with the dialog's buttons this frame.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DialogAction {
    /// Nothing.
    None,
    /// OK was clicked (only possible while OK is enabled).
    Ok,
    /// Cancel was clicked.
    Cancel,
}

/// The frame of a tool or feature dialog.
#[derive(Debug)]
#[must_use = "call `show` or `show_inside`"]
pub struct DialogFrame<'a> {
    title: &'a str,
    help: Option<&'a str>,
    health: Option<&'a Health>,
    ok: Enablement,
}

impl<'a> DialogFrame<'a> {
    /// A dialog titled `title`, OK enabled, no banner.
    pub fn new(title: &'a str) -> Self {
        DialogFrame {
            title,
            help: None,
            health: None,
            ok: Enablement::Enabled,
        }
    }

    /// The preview health: a warning or error is shown as a banner above the body.
    pub fn banner(mut self, health: Option<&'a Health>) -> Self {
        self.health = health;
        self
    }

    /// Whether OK is enabled; a `Disabled` reason is shown next to the buttons and as OK's
    /// tooltip.
    pub fn ok_enabled(mut self, e: Enablement) -> Self {
        self.ok = e;
        self
    }

    /// A help link shown at the top (additive).
    pub fn help(mut self, url: &'a str) -> Self {
        self.help = Some(url);
        self
    }

    /// Shows the dialog in its own window.
    pub fn show(self, ctx: &egui::Context, body: impl FnOnce(&mut egui::Ui)) -> DialogAction {
        let mut action = DialogAction::None;
        egui::Window::new(self.title)
            .id(egui::Id::new(("amg_dialog_frame", self.title)))
            .collapsible(false)
            .resizable(false)
            .show(ctx, |ui| {
                action = self.show_inside(ui, body);
            });
        action
    }

    /// Shows the dialog in `ui` (e.g. a tool's `dialog_ui`, which the router already puts in
    /// a titled window; additive).
    pub fn show_inside(self, ui: &mut egui::Ui, body: impl FnOnce(&mut egui::Ui)) -> DialogAction {
        let mut action = DialogAction::None;
        ui.vertical(|ui| {
            if let Some(url) = self.help {
                ui.hyperlink_to("Help", url);
            }
            if let Some(h) = self.health {
                health_banner(ui, h);
            }
            body(ui);
            ui.separator();
            ui.horizontal(|ui| {
                let enabled = self.ok == Enablement::Enabled;
                let mut ok = ui.add_enabled(enabled, egui::Button::new("OK"));
                if let Enablement::Disabled(why) = &self.ok {
                    ok = ok.on_disabled_hover_text(why.as_ref());
                }
                if ok.clicked() && enabled {
                    action = DialogAction::Ok;
                }
                if ui.button("Cancel").clicked() {
                    action = DialogAction::Cancel;
                }
                if let Enablement::Disabled(why) = &self.ok {
                    ui.colored_label(ui.visuals().warn_fg_color, why.as_ref());
                }
            });
        });
        action
    }
}

/// The banner for a warning or error preview health (nothing for the others).
fn health_banner(ui: &mut egui::Ui, h: &Health) {
    let (color, lines): (egui::Color32, Vec<String>) = match h {
        Health::Warning(ds) => (
            ui.visuals().warn_fg_color,
            ds.iter().map(|d| format!("⚠ {}", d.message)).collect(),
        ),
        Health::Error(d) => (
            ui.visuals().error_fg_color,
            vec![format!("⛔ {}", d.message)],
        ),
        _ => return,
    };
    egui::Frame::new()
        .stroke(egui::Stroke::new(1.0, color))
        .corner_radius(4.0)
        .inner_margin(6.0)
        .show(ui, |ui| {
            for l in lines {
                ui.colored_label(color, l);
            }
        });
}

/// A collapsible group of fields (open by default).
pub fn group<R>(
    ui: &mut egui::Ui,
    title: &str,
    body: impl FnOnce(&mut egui::Ui) -> R,
) -> egui::CollapsingResponse<R> {
    egui::CollapsingHeader::new(title)
        .default_open(true)
        .show(ui, body)
}
