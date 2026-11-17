//! The egui integration: one shared device and a paint callback (design note §1).
//!
//! [`install`] stores the [`Renderer`] in egui-wgpu's callback resources. Each [`Viewport`]
//! adds a paint callback whose `prepare` records the 3D passes on egui's encoder and whose
//! `paint` copies the final image 1:1 into egui's single-sample pass.

use std::collections::HashMap;
use std::sync::Arc;

use amg_jobs::gpu::Gpu;
use egui_wgpu::{CallbackResources, CallbackTrait, RenderState, ScreenDescriptor};

use crate::renderer::{
    FINAL_FORMAT_RAW, FULLSCREEN_WGSL, fullscreen_pipeline, texture_bind_group, texture_bind_layout,
};
use amg_core::Aabb3;

use crate::{
    Camera, ProjectedLabel, Projection, RenderError, Renderer, RendererConfig, ViewId, ViewStyle,
    VisualStyle, nav,
};

/// True if egui draws with the device of `gpu`.
///
/// Compares the instance as well: `wgpu::Device` equality only compares an id that is local to
/// its instance, so devices of two separate instances can compare equal.
pub fn shares_device(rs: &RenderState, gpu: &Gpu) -> bool {
    rs.instance == *gpu.instance() && rs.device == *gpu.device()
}

/// Puts a new [`Renderer`] into `rs.renderer.callback_resources`.
///
/// `egui_msaa_samples` is the sample count of egui's own pass (0 or 1 when it has none).
///
/// # Errors
/// [`RenderError::DeviceMismatch`] if `gpu` is not the device egui draws with.
pub fn install(
    rs: &RenderState,
    gpu: Arc<Gpu>,
    cfg: &RendererConfig,
    egui_msaa_samples: u32,
) -> Result<(), RenderError> {
    if !shares_device(rs, &gpu) {
        return Err(RenderError::DeviceMismatch);
    }
    let renderer = Renderer::new(gpu, cfg)?;
    let composite = Composite::new(&rs.device, rs.target_format, egui_msaa_samples.max(1));
    rs.renderer.write().callback_resources.insert(GlueState {
        renderer,
        composite,
        binds: HashMap::new(),
    });
    Ok(())
}

/// Short write lock on egui's renderer; `None` if [`install`] was not called.
pub fn with_renderer<R>(rs: &RenderState, f: impl FnOnce(&mut Renderer) -> R) -> Option<R> {
    rs.renderer
        .write()
        .callback_resources
        .get_mut::<GlueState>()
        .map(|g| f(&mut g.renderer))
}

/// A 3D viewport widget for one [`ViewId`].
#[derive(Clone, Copy, Debug)]
pub struct Viewport {
    /// The view to show.
    pub view: ViewId,
}

impl Viewport {
    /// Allocates all available space and adds the paint callback; the view is resized to the
    /// allocated pixel size during `prepare`.
    pub fn show(&self, ui: &mut egui::Ui, rs: &RenderState) -> ViewportResponse {
        let (rect, response) =
            ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
        let pixels_per_point = ui.ctx().pixels_per_point();
        let size_px = [
            (rect.width() * pixels_per_point).round().max(0.0) as u32,
            (rect.height() * pixels_per_point).round().max(0.0) as u32,
        ];
        if ui.is_rect_visible(rect) {
            ui.painter().add(egui_wgpu::Callback::new_paint_callback(
                rect,
                ViewportCallback {
                    view: self.view,
                    size_px,
                },
            ));
        }
        let lost = rs
            .renderer
            .read()
            .callback_resources
            .get::<GlueState>()
            .is_some_and(|g| g.renderer.gpu().is_lost());
        ViewportResponse {
            response,
            rect,
            pixels_per_point,
            size_px,
            lost,
        }
    }
}

/// What [`Viewport::show`] allocated.
#[derive(Clone, Debug)]
pub struct ViewportResponse {
    /// Interaction response of the viewport area.
    pub response: egui::Response,
    /// The area in points.
    pub rect: egui::Rect,
    /// Points to pixels.
    pub pixels_per_point: f32,
    /// The area in pixels.
    pub size_px: [u32; 2],
    /// The GPU device is lost.
    pub lost: bool,
}

/// Linear RGBA (straight alpha) to an egui colour (sRGB, premultiplied).
fn color32(c: [f32; 4]) -> egui::Color32 {
    egui::Color32::from(egui::Rgba::from_rgba_unmultiplied(
        c[0].clamp(0.0, 1.0),
        c[1].clamp(0.0, 1.0),
        c[2].clamp(0.0, 1.0),
        c[3].clamp(0.0, 1.0),
    ))
}

/// Padding of a label's background box, points.
const LABEL_PADDING: f32 = 3.0;

/// Paints `labels` (from [`Renderer::project_labels`]) over the viewport with egui text, so the
/// glyphs are rasterised by egui at the screen pixel size: crisp at every zoom (design note §12).
/// Each label is centred on its projected position (physical pixels of the view, converted with
/// `resp.pixels_per_point`), with an optional rounded background, clipped to the viewport.
pub fn paint_labels(ui: &egui::Ui, resp: &ViewportResponse, labels: &[ProjectedLabel]) {
    let painter = ui.painter_at(resp.rect);
    let ppp = resp.pixels_per_point.max(1e-6);
    for label in labels {
        let pos = resp.rect.min + egui::vec2(label.pos_px[0], label.pos_px[1]) / ppp;
        let color = color32(label.style.color);
        let galley = painter.layout_no_wrap(
            label.text.clone(),
            egui::FontId::proportional(label.style.size_pt.max(1.0)),
            color,
        );
        let rect = egui::Align2::CENTER_CENTER.anchor_size(pos, galley.size());
        if let Some(bg) = label.style.background {
            painter.rect_filled(rect.expand(LABEL_PADDING), LABEL_PADDING, color32(bg));
        }
        painter.galley(rect.min, galley, color);
    }
}

/// What a navigation widget asks the shell to do (S11 §Navigation). Camera actions can be
/// applied with [`apply_camera_action`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CameraAction {
    /// Animate to a standard view (the view-cube home button sends `Home`).
    SetView(nav::StandardView),
    /// A click on a view-cube region: animate to [`nav::view_for_region`].
    Region(nav::CubeRegion),
    /// A drag on the view cube: orbit about the target by this many physical pixels.
    Orbit {
        /// Cursor movement this frame, physical px (x right, y down).
        delta_px: [f32; 2],
    },
    /// Fit the visible scene.
    Fit,
    /// Switch the lens, keeping the apparent size at the target.
    SetProjection(Projection),
    /// Change the view's visual style.
    SetVisualStyle(VisualStyle),
    /// Make the next primary-button drag orbit, pan or zoom (nav-bar tool buttons).
    StartMode(nav::NavMode),
}

/// Side of the view-cube home button, points.
const HOME_BUTTON_PT: f32 = 16.0;

/// The view-cube inset rectangle of `resp` in points (the layout of
/// [`nav::ViewCube::for_scale`] at the viewport's scale, as the renderer draws it when
/// `View::show_view_cube` is set with that cube).
pub fn view_cube_rect(resp: &ViewportResponse) -> egui::Rect {
    let ppp = resp.pixels_per_point.max(1e-6);
    let [x, y, w, h] = nav::ViewCube::for_scale(ppp).rect_px(resp.size_px);
    egui::Rect::from_min_size(
        resp.rect.min + egui::vec2(x as f32, y as f32) / ppp,
        egui::vec2(w as f32, h as f32) / ppp,
    )
}

/// View-cube inset interaction (the cube is drawn by the renderer, `View::show_view_cube`; it
/// is hit-tested here with [`nav::view_cube_hit`]). A click on a region returns
/// [`CameraAction::Region`], a drag [`CameraAction::Orbit`], the home button in the inset's
/// top-left corner [`CameraAction::SetView`]`(Home)`. Returns the action for the shell to
/// forward (S11 §Navigation).
pub fn view_cube(ui: &mut egui::Ui, resp: &ViewportResponse, cam: &Camera) -> Option<CameraAction> {
    let ppp = resp.pixels_per_point.max(1e-6);
    let rect = view_cube_rect(resp);
    if rect.width() <= 0.0 || rect.height() <= 0.0 {
        return None;
    }
    let id = resp.response.id.with("amg_view_cube");
    let cube = ui.interact(rect, id, egui::Sense::click_and_drag());
    // Registered after the cube, so it wins where they overlap.
    let home_rect = egui::Rect::from_min_size(rect.min, egui::vec2(HOME_BUTTON_PT, HOME_BUTTON_PT));
    let home = ui
        .interact(home_rect, id.with("home"), egui::Sense::click())
        .on_hover_text("Home view");
    paint_home_button(ui, home_rect, home.hovered());

    if home.clicked() {
        return Some(CameraAction::SetView(nav::StandardView::Home));
    }
    if cube.dragged() {
        let d = cube.drag_delta() * ppp;
        if d != egui::Vec2::ZERO {
            return Some(CameraAction::Orbit {
                delta_px: [d.x, d.y],
            });
        }
    }
    if cube.clicked() {
        let pos = cube.interact_pointer_pos()?;
        let px = (pos - rect.min) * ppp;
        let side = (rect.width() * ppp).round() as u32;
        let region = nav::view_cube_hit(cam, [f64::from(px.x), f64::from(px.y)], side)?;
        return Some(CameraAction::Region(region));
    }
    None
}

/// A small house outline.
fn paint_home_button(ui: &egui::Ui, rect: egui::Rect, hovered: bool) {
    let painter = ui.painter_at(rect.expand(1.0));
    let color = if hovered {
        egui::Color32::from_rgb(40, 110, 230)
    } else {
        egui::Color32::from_gray(120)
    };
    let r = rect.shrink(3.0);
    let (l, t, rt, b) = (r.left(), r.top(), r.right(), r.bottom());
    let mid = r.center().x;
    let eave = t + r.height() * 0.45;
    let stroke = egui::Stroke::new(1.5, color);
    painter.add(egui::Shape::closed_line(
        vec![
            egui::pos2(mid, t),
            egui::pos2(rt, eave),
            egui::pos2(rt - 2.0, eave),
            egui::pos2(rt - 2.0, b),
            egui::pos2(l + 2.0, b),
            egui::pos2(l + 2.0, eave),
            egui::pos2(l, eave),
        ],
        stroke,
    ));
}

/// Fusion's visual styles in menu order, with their labels.
const VISUAL_STYLES: [(VisualStyle, &str); 6] = [
    (VisualStyle::Shaded, "Shaded"),
    (VisualStyle::ShadedVisibleEdges, "Shaded with visible edges"),
    (VisualStyle::ShadedHiddenEdges, "Shaded with hidden edges"),
    (VisualStyle::Wireframe, "Wireframe"),
    (
        VisualStyle::WireframeHiddenEdges,
        "Wireframe with hidden edges",
    ),
    (
        VisualStyle::WireframeVisibleEdges,
        "Wireframe with visible edges only",
    ),
];

/// Fusion-style bottom nav bar: orbit, pan, zoom, fit, a projection toggle and the visual style
/// menu. Returns the action of the button clicked this frame.
pub fn nav_bar(ui: &mut egui::Ui, style: &ViewStyle, cam: &Camera) -> Option<CameraAction> {
    let mut action = None;
    ui.horizontal(|ui| {
        for (label, mode, tip) in [
            ("Orbit", nav::NavMode::Orbit, "Drag to orbit"),
            ("Pan", nav::NavMode::Pan, "Drag to pan"),
            ("Zoom", nav::NavMode::Zoom, "Drag to zoom"),
        ] {
            if ui.button(label).on_hover_text(tip).clicked() {
                action = Some(CameraAction::StartMode(mode));
            }
        }
        if ui.button("Fit").on_hover_text("Fit the view").clicked() {
            action = Some(CameraAction::Fit);
        }
        let (label, other) = match cam.projection {
            Projection::Perspective { .. } => (
                "Perspective",
                Projection::Orthographic {
                    height: nav_apparent_height(cam),
                },
            ),
            Projection::Orthographic { .. } => (
                "Orthographic",
                Projection::Perspective {
                    fov_y: nav::DEFAULT_FOV_Y,
                },
            ),
        };
        if ui
            .button(label)
            .on_hover_text("Switch perspective / orthographic")
            .clicked()
        {
            action = Some(CameraAction::SetProjection(other));
        }
        let current = VISUAL_STYLES
            .iter()
            .find(|(v, _)| *v == style.visual)
            .map_or("Visual style", |(_, l)| *l);
        egui::ComboBox::from_id_salt("amg_visual_style")
            .selected_text(current)
            .show_ui(ui, |ui| {
                for (v, l) in VISUAL_STYLES {
                    if ui.selectable_label(style.visual == v, l).clicked() && style.visual != v {
                        action = Some(CameraAction::SetVisualStyle(v));
                    }
                }
            });
    });
    action
}

/// Height of the view at the target (what an orthographic switch keeps), mm.
fn nav_apparent_height(cam: &Camera) -> f64 {
    match cam.projection {
        Projection::Perspective { fov_y } => {
            2.0 * cam.eye.distance(cam.target) * (fov_y * 0.5).tan()
        }
        Projection::Orthographic { height } => height,
    }
}

/// Applies a camera action with `nav` (animated where the action is a view change), with
/// `bounds` the visible scene and `size_px` the view size. Returns the actions that are not
/// camera changes ([`CameraAction::SetVisualStyle`], [`CameraAction::StartMode`]) for the shell.
pub fn apply_camera_action(
    nav: &mut nav::NavController,
    cam: &mut Camera,
    action: CameraAction,
    bounds: Aabb3,
    size_px: [u32; 2],
) -> Option<CameraAction> {
    match action {
        CameraAction::SetView(v) => nav.set_view(cam, v, bounds, size_px, true),
        CameraAction::Region(r) => {
            nav.set_view(cam, nav::view_for_region(r), bounds, size_px, true);
        }
        CameraAction::Orbit { delta_px } => {
            nav.begin(
                nav::NavMode::Orbit,
                cam,
                [0.0, 0.0],
                Some(cam.target),
                bounds,
            );
            nav.drag(
                cam,
                [f64::from(delta_px[0]), f64::from(delta_px[1])],
                size_px,
            );
            nav.end();
        }
        CameraAction::Fit => nav.fit(cam, bounds, size_px, true),
        CameraAction::SetProjection(p) => nav.set_projection(cam, p),
        CameraAction::SetVisualStyle(_) | CameraAction::StartMode(_) => return Some(action),
    }
    None
}

/// Everything the callbacks share, in egui-wgpu's type map.
struct GlueState {
    renderer: Renderer,
    composite: Composite,
    /// Per view: the generation of the targets the bind group was made for.
    binds: HashMap<ViewId, (u64, wgpu::BindGroup)>,
}

/// Copies a view's final image into egui's pass.
struct Composite {
    pipeline: wgpu::RenderPipeline,
    layout: wgpu::BindGroupLayout,
    source_format: wgpu::TextureFormat,
}

impl Composite {
    fn new(device: &wgpu::Device, target: wgpu::TextureFormat, samples: u32) -> Composite {
        let layout = texture_bind_layout(device, "amg composite");
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("amg composite"),
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("amg composite"),
            source: wgpu::ShaderSource::Wgsl(FULLSCREEN_WGSL.into()),
        });
        let pipeline = fullscreen_pipeline(
            device,
            "amg composite",
            &pipeline_layout,
            &module,
            target,
            samples,
        );
        Composite {
            pipeline,
            layout,
            source_format: composite_source_format(target),
        }
    }

    fn bind(&self, device: &wgpu::Device, final_texture: &wgpu::Texture) -> wgpu::BindGroup {
        let view = final_texture.create_view(&wgpu::TextureViewDescriptor {
            label: Some("amg composite source"),
            format: Some(self.source_format),
            ..Default::default()
        });
        texture_bind_group(device, &self.layout, &view)
    }
}

/// How the sRGB final image must be read for a target format: an sRGB target re-encodes on
/// write, so it gets decoded linear values; any other target gets the encoded bytes as they
/// are, because egui blends in gamma space.
fn composite_source_format(target: wgpu::TextureFormat) -> wgpu::TextureFormat {
    if target.is_srgb() {
        FINAL_FORMAT_RAW.add_srgb_suffix()
    } else {
        FINAL_FORMAT_RAW
    }
}

/// The per-frame callback of one [`Viewport`].
struct ViewportCallback {
    view: ViewId,
    size_px: [u32; 2],
}

impl CallbackTrait for ViewportCallback {
    fn prepare(
        &self,
        device: &wgpu::Device,
        _queue: &wgpu::Queue,
        _screen: &ScreenDescriptor,
        egui_encoder: &mut wgpu::CommandEncoder,
        resources: &mut CallbackResources,
    ) -> Vec<wgpu::CommandBuffer> {
        let Some(state) = resources.get_mut::<GlueState>() else {
            return Vec::new();
        };
        state.renderer.set_view_size(self.view, self.size_px);
        if let Err(e) = state.renderer.encode(self.view, egui_encoder) {
            tracing::error!("viewport encode failed: {e}");
            return Vec::new();
        }
        let generation = state.renderer.view_generation(self.view);
        match (generation, state.renderer.final_texture(self.view)) {
            (Some(generation), Some(texture)) => {
                let fresh = state
                    .binds
                    .get(&self.view)
                    .is_some_and(|b| b.0 == generation);
                if !fresh {
                    let bind = state.composite.bind(device, texture);
                    state.binds.insert(self.view, (generation, bind));
                }
            }
            _ => {
                state.binds.remove(&self.view);
            }
        }
        Vec::new()
    }

    fn paint(
        &self,
        info: egui::PaintCallbackInfo,
        render_pass: &mut wgpu::RenderPass<'static>,
        resources: &CallbackResources,
    ) {
        let Some(state) = resources.get::<GlueState>() else {
            return;
        };
        let Some((_, bind)) = state.binds.get(&self.view) else {
            return;
        };
        let viewport = info.viewport_in_pixels();
        let clip = info.clip_rect_in_pixels();
        if viewport.width_px <= 0
            || viewport.height_px <= 0
            || clip.width_px <= 0
            || clip.height_px <= 0
        {
            return;
        }
        render_pass.set_viewport(
            viewport.left_px as f32,
            viewport.top_px as f32,
            viewport.width_px as f32,
            viewport.height_px as f32,
            0.0,
            1.0,
        );
        render_pass.set_scissor_rect(
            clip.left_px as u32,
            clip.top_px as u32,
            clip.width_px as u32,
            clip.height_px as u32,
        );
        render_pass.set_pipeline(&state.composite.pipeline);
        render_pass.set_bind_group(0, bind, &[]);
        render_pass.draw(0..3, 0..1);
    }
}

#[cfg(test)]
mod tests {
    use amg_core::{Transform, Vec3};
    use amg_jobs::JobCtx;
    use amg_jobs::gpu::GpuConfig;
    use egui::mutex::RwLock;

    use super::*;
    use crate::{Instance, MeshData, upload_mesh};

    fn gpu() -> Option<Arc<Gpu>> {
        if !amg_testkit::gpu_available() {
            return None;
        }
        Some(Gpu::new_blocking(&GpuConfig::from_env()).expect("adapter probed, device opens"))
    }

    fn render_state(gpu: &Gpu, format: wgpu::TextureFormat) -> RenderState {
        RenderState {
            adapter: gpu.adapter().clone(),
            available_adapters: vec![gpu.adapter().clone()],
            instance: gpu.instance().clone(),
            device: gpu.device().clone(),
            queue: gpu.queue().clone(),
            target_format: format,
            renderer: Arc::new(RwLock::new(egui_wgpu::Renderer::new(
                gpu.device(),
                format,
                egui_wgpu::RendererOptions::default(),
            ))),
            surface_config: egui_wgpu::SurfaceConfig::HIGH_THROUGHPUT,
        }
    }

    /// Done-when 1 (RND-07): labels are egui text shapes centred on their projected anchors.
    #[test]
    fn paint_labels_emits_egui_text_at_the_projected_positions() {
        let ctx = egui::Context::default();
        let ppp = 2.0;
        ctx.set_pixels_per_point(ppp);
        let labels = vec![
            ProjectedLabel {
                pos_px: [100.0, 60.0],
                depth: 0.5,
                text: "12.50 mm".into(),
                style: crate::LabelStyle {
                    background: Some([0.0, 0.0, 0.0, 0.8]),
                    ..crate::LabelStyle::default()
                },
            },
            ProjectedLabel {
                pos_px: [300.0, 200.0],
                depth: 0.5,
                text: "X".into(),
                style: crate::LabelStyle::default(),
            },
        ];
        let viewport = egui::Rect::from_min_size(egui::pos2(10.0, 20.0), egui::vec2(200.0, 150.0));
        let frame = || {
            ctx.run_ui(egui::RawInput::default(), |ui| {
                let response = ui.allocate_rect(viewport, egui::Sense::hover());
                let resp = ViewportResponse {
                    response,
                    rect: viewport,
                    pixels_per_point: ppp,
                    size_px: [400, 300],
                    lost: false,
                };
                paint_labels(ui, &resp, &labels);
            })
        };
        // Two frames: the first one sets up the fonts.
        frame().drop_without_applying_deltas();
        let mut out = frame();
        let shapes = std::mem::take(&mut out.shapes);
        out.drop_without_applying_deltas();
        let mut texts = Vec::new();
        let mut boxes = 0;
        for clipped in &shapes {
            match &clipped.shape {
                egui::Shape::Text(t) => {
                    texts.push((t.galley.text().to_owned(), t.visual_bounding_rect()))
                }
                egui::Shape::Rect(r) if r.fill.a() > 0 && r.rect.width() < 100.0 => boxes += 1,
                _ => {}
            }
        }
        for (label, want) in labels
            .iter()
            .zip([egui::pos2(60.0, 50.0), egui::pos2(160.0, 120.0)])
        {
            let (_, rect) = texts
                .iter()
                .find(|(t, _)| *t == label.text)
                .unwrap_or_else(|| panic!("no text shape for {:?}: {texts:?}", label.text));
            assert!(
                (rect.center() - want).length() < 2.0,
                "{:?} centred at {:?}, expected {want:?}",
                label.text,
                rect.center()
            );
        }
        assert_eq!(boxes, 1, "one label has a background box");
    }

    /// A headless egui driver for the navigation widgets: a 400×300 pt viewport at 2 px/pt.
    struct Harness {
        ctx: egui::Context,
        time: f64,
    }

    const PPP: f32 = 2.0;
    const VIEWPORT_MIN: egui::Pos2 = egui::pos2(10.0, 20.0);
    const VIEWPORT_PX: [u32; 2] = [800, 600];

    impl Harness {
        fn new() -> Harness {
            let ctx = egui::Context::default();
            ctx.set_pixels_per_point(PPP);
            let mut h = Harness { ctx, time: 0.0 };
            // The first frame sets up the fonts.
            h.frame(Vec::new(), &mut |_, _| {});
            h
        }

        fn viewport() -> egui::Rect {
            egui::Rect::from_min_size(
                VIEWPORT_MIN,
                egui::vec2(VIEWPORT_PX[0] as f32, VIEWPORT_PX[1] as f32) / PPP,
            )
        }

        /// Runs one frame with `events`; `f` gets the ui and the viewport response.
        fn frame(
            &mut self,
            events: Vec<egui::Event>,
            f: &mut dyn FnMut(&mut egui::Ui, &ViewportResponse),
        ) -> Vec<egui::epaint::ClippedShape> {
            self.time += 1.0 / 60.0;
            let input = egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(640.0, 480.0),
                )),
                time: Some(self.time),
                events,
                ..Default::default()
            };
            let mut out = self.ctx.run_ui(input, |ui| {
                let rect = Harness::viewport();
                let response = ui.allocate_rect(rect, egui::Sense::click_and_drag());
                let resp = ViewportResponse {
                    response,
                    rect,
                    pixels_per_point: PPP,
                    size_px: VIEWPORT_PX,
                    lost: false,
                };
                f(ui, &resp);
            });
            let shapes = std::mem::take(&mut out.shapes);
            out.drop_without_applying_deltas();
            shapes
        }

        fn button(pos: egui::Pos2, pressed: bool) -> egui::Event {
            egui::Event::PointerButton {
                pos,
                button: egui::PointerButton::Primary,
                pressed,
                modifiers: egui::Modifiers::default(),
            }
        }

        /// Move, press, release at `pos`; every action `f` returned.
        fn click(
            &mut self,
            pos: egui::Pos2,
            f: &mut dyn FnMut(&mut egui::Ui, &ViewportResponse) -> Option<CameraAction>,
        ) -> Vec<CameraAction> {
            let mut actions = Vec::new();
            for events in [
                vec![egui::Event::PointerMoved(pos)],
                vec![Harness::button(pos, true)],
                vec![Harness::button(pos, false)],
                vec![],
            ] {
                self.frame(events, &mut |ui, resp| actions.extend(f(ui, resp)));
            }
            actions
        }
    }

    fn standard_camera(v: nav::StandardView) -> Camera {
        Camera {
            eye: v.eye_direction() * 80.0,
            target: amg_core::Point3::ZERO,
            up: amg_core::Dir3::new_unchecked(v.up()),
            projection: Projection::Perspective { fov_y: 0.8 },
        }
    }

    /// Where (points) the cube-surface point `p` (cube units) of the inset of `cam` is.
    fn cube_point(cam: &Camera, p: Vec3) -> egui::Pos2 {
        let cube = nav::ViewCube::for_scale(PPP);
        let [x, y, w, _] = cube.rect_px(VIEWPORT_PX);
        let q = nav::view_cube_camera(cam).project(p, [w, w]).unwrap();
        VIEWPORT_MIN + egui::vec2(x as f32 + q[0] as f32, y as f32 + q[1] as f32) / PPP
    }

    /// Done-when 8 (RND-11): a click on each cube face region emits the matching action, face-on
    /// and from the Home view (faces, edges and the corner it shows).
    #[test]
    fn view_cube_clicks_emit_the_region_actions() {
        use nav::{CubeRegion, StandardView as V};
        let mut h = Harness::new();
        for face in [V::Front, V::Back, V::Left, V::Right, V::Top, V::Bottom] {
            let cam = standard_camera(face);
            let pos = cube_point(&cam, face.eye_direction());
            let actions = h.click(pos, &mut |ui, resp| view_cube(ui, resp, &cam));
            assert_eq!(
                actions,
                vec![CameraAction::Region(CubeRegion::Face(face))],
                "{face:?} at {pos:?}"
            );
        }
        // From Home: the three faces, three edges and the corner it shows. Points on the
        // visible faces: face centres, 0.8 toward an edge, 0.8 toward a corner.
        let home = standard_camera(V::Home);
        let (fx, fy, fz) = (Vec3::X, Vec3::NEG_Y, Vec3::Z);
        let cases = [
            (fx, CubeRegion::Face(V::Right)),
            (fy, CubeRegion::Face(V::Front)),
            (fz, CubeRegion::Face(V::Top)),
            (
                fy + fx * 0.8,
                CubeRegion::Edge {
                    a: V::Right,
                    b: V::Front,
                },
            ),
            (
                fz + fx * 0.8,
                CubeRegion::Edge {
                    a: V::Right,
                    b: V::Top,
                },
            ),
            (
                fz + fy * 0.8,
                CubeRegion::Edge {
                    a: V::Front,
                    b: V::Top,
                },
            ),
            (
                fz + (fx + fy) * 0.8,
                CubeRegion::Corner { x: 1, y: -1, z: 1 },
            ),
        ];
        for (p, region) in cases {
            let pos = cube_point(&home, p);
            let actions = h.click(pos, &mut |ui, resp| view_cube(ui, resp, &home));
            assert_eq!(actions, vec![CameraAction::Region(region)], "{region:?}");
        }
        // Outside the cube silhouette (but inside the inset): nothing.
        let rect = {
            let mut r = egui::Rect::NOTHING;
            h.frame(Vec::new(), &mut |_, resp| r = view_cube_rect(resp));
            r
        };
        let off = egui::pos2(rect.right() - 3.0, rect.bottom() - 3.0);
        assert!(
            h.click(off, &mut |ui, resp| view_cube(ui, resp, &home))
                .is_empty()
        );
        // The home button.
        let actions = h.click(rect.min + egui::vec2(8.0, 8.0), &mut |ui, resp| {
            view_cube(ui, resp, &home)
        });
        assert_eq!(actions, vec![CameraAction::SetView(V::Home)]);
    }

    /// A drag on the cube orbits by the cursor movement in physical pixels.
    #[test]
    fn view_cube_drag_emits_orbit() {
        let mut h = Harness::new();
        let cam = standard_camera(nav::StandardView::Home);
        let start = cube_point(&cam, Vec3::new(0.0, -1.0, 0.0));
        let mut actions = Vec::new();
        let mut f = |ui: &mut egui::Ui, resp: &ViewportResponse| {
            actions.extend(view_cube(ui, resp, &cam));
        };
        h.frame(vec![egui::Event::PointerMoved(start)], &mut f);
        h.frame(vec![Harness::button(start, true)], &mut f);
        for i in 1..=5 {
            let p = start + egui::vec2(-4.0 * i as f32, 2.0 * i as f32);
            h.frame(vec![egui::Event::PointerMoved(p)], &mut f);
        }
        let end = start + egui::vec2(-20.0, 10.0);
        h.frame(vec![Harness::button(end, false)], &mut f);
        let mut total = [0.0f32; 2];
        for a in &actions {
            let CameraAction::Orbit { delta_px } = a else {
                panic!("unexpected {a:?} in {actions:?}");
            };
            total[0] += delta_px[0];
            total[1] += delta_px[1];
        }
        assert!(!actions.is_empty());
        // egui starts a drag after a small threshold, so the first move may be swallowed.
        assert!(total[0] <= -20.0 && total[0] >= -40.0 * PPP, "{total:?}");
        assert!(total[1] >= 10.0 && total[1] <= 20.0 * PPP, "{total:?}");
    }

    /// The nav bar's buttons emit their actions.
    #[test]
    fn nav_bar_buttons_emit_actions() {
        let mut h = Harness::new();
        let style = ViewStyle::default();
        let cam = standard_camera(nav::StandardView::Home);
        let mut bar = |ui: &mut egui::Ui, _: &ViewportResponse| nav_bar(ui, &style, &cam);
        // Find the buttons' text.
        let shapes = h.frame(Vec::new(), &mut |ui, resp| {
            bar(ui, resp);
        });
        let find = |text: &str| {
            shapes
                .iter()
                .find_map(|c| match &c.shape {
                    egui::Shape::Text(t) if t.galley.text() == text => {
                        Some(t.visual_bounding_rect().center())
                    }
                    _ => None,
                })
                .unwrap_or_else(|| panic!("no {text:?} button"))
        };
        let (fit, pan, persp) = (find("Fit"), find("Pan"), find("Perspective"));
        assert_eq!(h.click(fit, &mut bar), vec![CameraAction::Fit]);
        assert_eq!(
            h.click(pan, &mut bar),
            vec![CameraAction::StartMode(nav::NavMode::Pan)]
        );
        let actions = h.click(persp, &mut bar);
        let [CameraAction::SetProjection(Projection::Orthographic { height })] = actions[..] else {
            panic!("{actions:?}");
        };
        // The apparent height at the target.
        let want = 2.0 * 80.0 * (0.4f64).tan();
        assert!((height - want).abs() < 1e-9);
    }

    /// `apply_camera_action` animates region clicks and orbits cube drags about the target.
    #[test]
    fn apply_camera_action_drives_the_controller() {
        let mut nav = nav::NavController::new();
        let mut cam = standard_camera(nav::StandardView::Home);
        let bounds = Aabb3::new(amg_core::Point3::splat(-5.0), amg_core::Point3::splat(5.0));
        let region = CameraAction::Region(nav::CubeRegion::Face(nav::StandardView::Top));
        assert_eq!(
            apply_camera_action(&mut nav, &mut cam, region, bounds, VIEWPORT_PX),
            None
        );
        assert!(nav.is_animating());
        while nav.tick(&mut cam, std::time::Duration::from_millis(16)) {}
        assert!((cam.forward().get() - Vec3::NEG_Z).length() < 1e-12);
        let before = cam.target;
        let orbit = CameraAction::Orbit {
            delta_px: [100.0, 0.0],
        };
        apply_camera_action(&mut nav, &mut cam, orbit, bounds, VIEWPORT_PX);
        assert_eq!(cam.target, before, "orbits about the target");
        let style = CameraAction::SetVisualStyle(VisualStyle::Wireframe);
        assert_eq!(
            apply_camera_action(&mut nav, &mut cam, style, bounds, VIEWPORT_PX),
            Some(style)
        );
    }

    #[test]
    fn gpu_install_rejects_a_different_device() {
        let (Some(a), Some(b)) = (gpu(), gpu()) else {
            return;
        };
        let rs = render_state(&a, wgpu::TextureFormat::Rgba8Unorm);
        let err = install(&rs, b, &RendererConfig::default(), 1).unwrap_err();
        assert!(matches!(err, RenderError::DeviceMismatch), "{err}");
        assert!(with_renderer(&rs, |_| ()).is_none());

        install(&rs, Arc::clone(&a), &RendererConfig::default(), 1).unwrap();
        let view = with_renderer(&rs, |r| r.create_view([4, 4])).unwrap();
        assert!(with_renderer(&rs, |r| r.view(view).is_some()).unwrap());
    }

    /// Reads back the raw bytes of a composite into a target of `format`.
    fn composite_bytes(gpu: &Gpu, source: &wgpu::Texture, format: wgpu::TextureFormat) -> Vec<u8> {
        let device = gpu.device();
        let composite = Composite::new(device, format, 1);
        let bind = composite.bind(device, source);
        let [width, height] = [source.width(), source.height()];
        let extent = wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        };
        let target = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("composite target"),
            size: extent,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let view = target.create_view(&Default::default());
        let row = (width * 4).next_multiple_of(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT);
        let bytes = u64::from(row) * u64::from(height);
        let buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("composite readback"),
            size: bytes,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        let mut encoder = device.create_command_encoder(&Default::default());
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("composite"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_pipeline(&composite.pipeline);
            pass.set_bind_group(0, &bind, &[]);
            pass.draw(0..3, 0..1);
        }
        encoder.copy_texture_to_buffer(
            target.as_image_copy(),
            wgpu::TexelCopyBufferInfo {
                buffer: &buffer,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(row),
                    rows_per_image: Some(height),
                },
            },
            extent,
        );
        gpu.queue().submit([encoder.finish()]);
        let padded = gpu.readback::<u8>(&buffer, 0..bytes).wait().unwrap();
        padded
            .chunks_exact(row as usize)
            .flat_map(|l| l[..width as usize * 4].iter().copied())
            .collect()
    }

    #[test]
    fn gpu_composite_preserves_the_image_on_both_target_kinds() {
        let Some(gpu) = gpu() else {
            return;
        };
        let mut renderer = Renderer::new(Arc::clone(&gpu), &RendererConfig::default()).unwrap();
        let view = renderer.create_view([32, 32]);
        let tri = MeshData {
            positions: vec![
                [-200.0, 0.0, -200.0],
                [200.0, 0.0, -200.0],
                [0.0, 0.0, 200.0],
            ],
            normals: vec![[0.0, -1.0, 0.0]; 3],
            indices: vec![0, 1, 2],
            ..Default::default()
        };
        let mesh = upload_mesh(&gpu, &tri, &JobCtx::detached()).unwrap();
        let key = renderer.scene_mut().insert_mesh(mesh);
        renderer
            .scene_mut()
            .add_instance(Instance::new(key, Transform::from_translation(Vec3::ZERO)));
        let image = renderer.capture(view).unwrap().wait().unwrap();
        let source = renderer.final_texture(view).unwrap().clone();

        for format in [
            wgpu::TextureFormat::Rgba8Unorm,
            wgpu::TextureFormat::Rgba8UnormSrgb,
        ] {
            let got = composite_bytes(&gpu, &source, format);
            for (a, b) in image.pixels.iter().zip(&got) {
                assert!(a.abs_diff(*b) <= 1, "{format:?}: {a} vs {b}");
            }
        }
    }
}
