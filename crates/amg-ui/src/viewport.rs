//! The viewport tab: a backend draws the 3D view of the active document.
//!
//! The app uses [`RenderViewport`] (S10 `egui_glue`, shared device); tests use
//! [`NullViewport`], which paints only the theme's background so no GPU is needed.
//!
//! UI-03: the frame carries the document's camera, which [`RenderViewport`] writes into its S10
//! view; it also draws the view cube and the nav bar and hands their [`CameraAction`]s to the
//! router, and supplies a [`GpuPickService`] for its view.

use amg_render::egui_glue::{self, CameraAction, Viewport};
use amg_render::{Background, Camera, ViewId};
use egui_wgpu::RenderState;

use crate::app::DocTabId;
use crate::pick::{GpuPickService, PickService};
use crate::theme::ThemeTokens;

/// What the shell hands the viewport backend each frame.
#[derive(Debug)]
pub struct ViewportFrame<'a> {
    /// Current theme colours (background, highlights).
    pub tokens: &'a ThemeTokens,
    /// The document shown.
    pub doc: DocTabId,
    /// The document's camera (UI-03).
    pub camera: &'a Camera,
}

/// Draws the 3D view into the viewport tab.
pub trait ViewportBackend: 'static {
    /// Fills `ui`'s available rect with the view.
    fn show(&mut self, ui: &mut egui::Ui, frame: &ViewportFrame<'_>) -> egui::Response;
    /// True if the GPU device was lost (status bar banner, UI-10).
    fn device_lost(&self) -> bool {
        false
    }
    /// View-cube and nav-bar actions since the last call (UI-03; the router applies them).
    fn take_camera_actions(&mut self) -> Vec<CameraAction> {
        Vec::new()
    }
    /// Areas of the viewport covered by the backend's own widgets (view cube, nav bar), in
    /// points: presses there are not viewport input.
    fn input_exclusions(&self) -> Vec<egui::Rect> {
        Vec::new()
    }
    /// The pick service for this backend's view (`None`: the app uses `CpuPickService`).
    /// Called once by `AmalgamApp::new`.
    fn pick_service(&mut self) -> Option<Box<dyn PickService>> {
        None
    }
    /// After the frame's layout and end-of-frame work (headless backends render here; egui
    /// paints the real one through its callback).
    fn after_frame(&mut self) {}
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
    actions: Vec<CameraAction>,
    exclusions: Vec<egui::Rect>,
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
            actions: Vec::new(),
            exclusions: Vec::new(),
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
        let set_bg = self.background != Some(bg);
        let view = self.view;
        let camera = *frame.camera;
        let ppp = ui.ctx().pixels_per_point();
        // A short write lock on egui's renderer: the camera of this frame, the cube layout and
        // (on theme change) the background.
        let style = egui_glue::with_renderer(&self.rs, |r| {
            let v = r.view_mut(view)?;
            v.camera = camera;
            v.show_view_cube = true;
            v.view_cube = amg_render::nav::ViewCube::for_scale(ppp);
            if set_bg {
                v.style.background = Background::Solid(bg);
            }
            Some(v.style)
        })
        .flatten();
        self.background = Some(bg);
        let out = Viewport { view: self.view }.show(ui, &self.rs);
        self.lost = out.lost;
        let labels =
            egui_glue::with_renderer(&self.rs, |r| r.project_labels(view)).unwrap_or_default();
        egui_glue::paint_labels(ui, &out, &labels);
        self.exclusions.clear();
        let cube = egui_glue::view_cube_rect(&out);
        self.exclusions.push(cube);
        if let Some(a) = egui_glue::view_cube(ui, &out, &camera) {
            self.actions.push(a);
        }
        if let Some(style) = style {
            let bar = egui::Rect::from_center_size(
                egui::pos2(out.rect.center().x, out.rect.bottom() - 18.0),
                egui::vec2(360.0, 28.0),
            );
            self.exclusions.push(bar);
            let mut bar_ui = ui.new_child(egui::UiBuilder::new().max_rect(bar));
            match egui_glue::nav_bar(&mut bar_ui, &style, &camera) {
                // The visual style belongs to the view, not the camera: applied here.
                Some(CameraAction::SetVisualStyle(vs)) => {
                    egui_glue::with_renderer(&self.rs, |r| {
                        if let Some(v) = r.view_mut(view) {
                            v.style.visual = vs;
                        }
                    });
                }
                Some(a) => self.actions.push(a),
                None => {}
            }
        }
        out.response
            .widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Other, true, "Viewport"));
        out.response
    }

    fn device_lost(&self) -> bool {
        self.lost
    }

    fn take_camera_actions(&mut self) -> Vec<CameraAction> {
        std::mem::take(&mut self.actions)
    }

    fn input_exclusions(&self) -> Vec<egui::Rect> {
        self.exclusions.clone()
    }

    fn pick_service(&mut self) -> Option<Box<dyn PickService>> {
        Some(Box::new(GpuPickService::egui(self.rs.clone(), self.view)))
    }
}
