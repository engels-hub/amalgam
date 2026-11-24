//! The Sketch Palette (spec §13): a floating window on the right while a sketch is open, with
//! the display and snap options (grid, snap, show profile / points / dimensions /
//! constraints, slice), the DOF of the sketch and the Finish Sketch button.

use std::borrow::Cow;

use crate::app::AppCx;
use crate::dialog::AppDialog;

use super::session::{SketchLayer, with_session};

/// The palette window (opened with the session, closed when it ends).
#[derive(Debug, Default)]
pub struct SketchPaletteDialog {
    /// The session it belongs to (`SketchLayer::opened` when it was opened).
    pub session: u32,
}

/// Window title.
pub const PALETTE_TITLE: &str = "Sketch Palette";

impl AppDialog for SketchPaletteDialog {
    fn title(&self) -> Cow<'static, str> {
        PALETTE_TITLE.into()
    }

    fn show(&mut self, ctx: &egui::Context, cx: &mut AppCx<'_>) -> bool {
        if cx.doc_layer::<SketchLayer>().map(|l| l.opened()) != Some(self.session) {
            return false;
        }
        let Some(mut pal) = with_session(cx, |s, _| s.palette) else {
            return false;
        };
        let dof = with_session(cx, |s, _| s.dof().map(|d| d.dof)).flatten();
        let mut finish = false;
        egui::Window::new(PALETTE_TITLE)
            .id(egui::Id::new("amg_sketch_palette"))
            .anchor(egui::Align2::RIGHT_BOTTOM, egui::vec2(-12.0, -48.0))
            .resizable(false)
            .collapsible(true)
            .show(ctx, |ui| {
                ui.strong("Options");
                ui.checkbox(&mut pal.grid, "Sketch grid");
                ui.checkbox(&mut pal.snap, "Snap");
                ui.checkbox(&mut pal.slice, "Slice");
                ui.checkbox(&mut pal.show_profile, "Show profile");
                ui.checkbox(&mut pal.show_points, "Show points");
                ui.checkbox(&mut pal.show_dimensions, "Show dimensions");
                ui.checkbox(&mut pal.show_constraints, "Show constraints");
                ui.separator();
                match dof {
                    Some(n) => ui.label(format!("DOF: {n}")),
                    None => ui.label("DOF: …"),
                };
                if ui.button("Finish Sketch").clicked() {
                    finish = true;
                }
            });
        with_session(cx, |s, _| s.palette = pal);
        if finish && let Some(l) = cx.doc_layer::<SketchLayer>() {
            l.request_finish();
        }
        true
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}
