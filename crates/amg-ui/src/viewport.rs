//! The viewport tab: a backend draws the 3D view of the active document.
//!
//! The app uses [`RenderViewport`] (S10 `egui_glue`, shared device); tests use
//! [`NullViewport`], which paints only the theme's background so no GPU is needed.

use amg_render::egui_glue::{self, Viewport};
use amg_render::{Background, ViewId};
use egui_wgpu::RenderState;

use crate::app::DocTabId;
use crate::theme::ThemeTokens;

/// What the shell hands the viewport backend each frame.
#[derive(Debug)]
pub struct ViewportFrame<'a> {
    /// Current theme colours (background, highlights).
    pub tokens: &'a ThemeTokens,
    /// The document shown.
    pub doc: DocTabId,
}

/// Draws the 3D view into the viewport tab.
pub trait ViewportBackend: 'static {
    /// Fills `ui`'s available rect with the view.
    fn show(&mut self, ui: &mut egui::Ui, frame: &ViewportFrame<'_>) -> egui::Response;
    /// True if the GPU device was lost (status bar banner, UI-10).
    fn device_lost(&self) -> bool {
        false
    }
}

/// A viewport that paints the theme's background gradient and nothing else (tests, kittest).
#[derive(Debug, Default)]
pub struct NullViewport;

impl ViewportBackend for NullViewport {
    fn show(&mut self, ui: &mut egui::Ui, frame: &ViewportFrame<'_>) -> egui::Response {
        let (rect, response) =
            ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
        paint_gradient(
            ui.painter(),
            rect,
            frame.tokens.background_top,
            frame.tokens.background_bottom,
        );
        response
            .widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Other, true, "Viewport"));
        response
    }
}

/// Paints a vertical gradient.
pub fn paint_gradient(
    painter: &egui::Painter,
    rect: egui::Rect,
    top: egui::Color32,
    bottom: egui::Color32,
) {
    let mut mesh = egui::Mesh::default();
    mesh.colored_vertex(rect.left_top(), top);
    mesh.colored_vertex(rect.right_top(), top);
    mesh.colored_vertex(rect.right_bottom(), bottom);
    mesh.colored_vertex(rect.left_bottom(), bottom);
    mesh.add_triangle(0, 1, 2);
    mesh.add_triangle(0, 2, 3);
    painter.add(egui::Shape::mesh(mesh));
}

/// The real viewport: S10's renderer drawn through `egui_glue` on egui's own device.
pub struct RenderViewport {
    rs: RenderState,
    view: ViewId,
    background: Option<[f32; 3]>,
    lost: bool,
}

impl std::fmt::Debug for RenderViewport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RenderViewport")
            .field("view", &self.view)
            .field("lost", &self.lost)
            .finish_non_exhaustive()
    }
}

impl RenderViewport {
    /// Wraps an installed renderer (`egui_glue::install`) and one of its views.
    pub fn new(rs: RenderState, view: ViewId) -> RenderViewport {
        RenderViewport {
            rs,
            view,
            background: None,
            lost: false,
        }
    }

    /// The view drawn.
    pub fn view(&self) -> ViewId {
        self.view
    }
}

impl ViewportBackend for RenderViewport {
    fn show(&mut self, ui: &mut egui::Ui, frame: &ViewportFrame<'_>) -> egui::Response {
        let bg = frame.tokens.background_linear();
        if self.background != Some(bg) {
            // Only on theme change: a short write lock on egui's renderer.
            let view = self.view;
            egui_glue::with_renderer(&self.rs, |r| {
                if let Some(v) = r.view_mut(view) {
                    v.style.background = Background::Solid(bg);
                }
            });
            self.background = Some(bg);
        }
        let out = Viewport { view: self.view }.show(ui, &self.rs);
        self.lost = out.lost;
        out.response
            .widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Other, true, "Viewport"));
        out.response
    }

    fn device_lost(&self) -> bool {
        self.lost
    }
}
