//! RND-13 on the real viewport backend ([`RenderViewport`]): the diagnostics overlay is off by
//! default behind its "Stats" toggle, and a lost device leaves only the placeholder. Headless
//! egui frames over a real device; `gpu_*` tests skip without an adapter.

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::sync::Arc;

use amg_jobs::gpu::{Gpu, GpuConfig};
use amg_render::egui_glue::{self, DEVICE_LOST_TITLE};
use amg_render::{Camera, OverlayList, Projection, RendererConfig, wgpu};
use amg_ui::app::DocTabId;
use amg_ui::theme::ThemeTokens;
use amg_ui::viewport::{RenderViewport, ViewportBackend, ViewportFrame};
use egui::mutex::RwLock;
use egui_wgpu::RenderState;

fn gpu() -> Option<Arc<Gpu>> {
    if !amg_testkit::gpu_available() {
        return None;
    }
    Some(Gpu::new_blocking(&GpuConfig::from_env()).expect("adapter probed, device opens"))
}

fn render_state(gpu: &Gpu) -> RenderState {
    let format = wgpu::TextureFormat::Rgba8Unorm;
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

/// The texts of one frame and where each is.
struct Frame {
    texts: Vec<(String, egui::Pos2)>,
    repaint: bool,
}

impl Frame {
    fn find(&self, needle: &str) -> Option<egui::Pos2> {
        self.texts
            .iter()
            .find(|(t, _)| t.contains(needle))
            .map(|&(_, p)| p)
    }
}

struct Harness {
    ctx: egui::Context,
    viewport: RenderViewport,
    time: f64,
}

impl Harness {
    fn new(gpu: &Arc<Gpu>) -> Harness {
        let rs = render_state(gpu);
        egui_glue::install(&rs, Arc::clone(gpu), &RendererConfig::default(), 1).unwrap();
        let view = egui_glue::with_renderer(&rs, |r| r.create_view([4, 4])).unwrap();
        Harness {
            ctx: egui::Context::default(),
            viewport: RenderViewport::new(rs, view),
            time: 0.0,
        }
    }

    fn frame(&mut self, events: Vec<egui::Event>) -> Frame {
        self.time += 1.0 / 60.0;
        let input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(800.0, 600.0),
            )),
            time: Some(self.time),
            events,
            ..Default::default()
        };
        let tokens = ThemeTokens::dark();
        let camera = Camera {
            eye: amg_core::Point3::new(0.0, -200.0, 100.0),
            target: amg_core::Point3::ZERO,
            up: amg_core::Dir3::Z,
            projection: Projection::Perspective { fov_y: 0.8 },
        };
        let overlays = OverlayList::default();
        let frame = ViewportFrame {
            tokens: &tokens,
            doc: DocTabId(1),
            camera: &camera,
            overlays: &overlays,
            sections: &[],
        };
        let viewport = &mut self.viewport;
        let mut out = self.ctx.run_ui(input, |ui| {
            viewport.show(ui, &frame);
        });
        let texts = out
            .shapes
            .iter()
            .filter_map(|c| match &c.shape {
                egui::Shape::Text(t) => Some((
                    t.galley.text().to_owned(),
                    t.visual_bounding_rect().center(),
                )),
                _ => None,
            })
            .collect();
        let repaint = out
            .viewport_output
            .values()
            .any(|v| v.repaint_delay.is_zero());
        out.shapes.clear();
        out.drop_without_applying_deltas();
        Frame { texts, repaint }
    }

    fn click(&mut self, pos: egui::Pos2) -> Frame {
        let button = |pressed| egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed,
            modifiers: egui::Modifiers::default(),
        };
        self.frame(vec![egui::Event::PointerMoved(pos)]);
        self.frame(vec![button(true)]);
        self.frame(vec![button(false)]);
        self.frame(Vec::new())
    }
}

#[test]
fn gpu_diagnostics_overlay_is_off_by_default_and_toggles() {
    let Some(gpu) = gpu() else { return };
    let mut h = Harness::new(&gpu);
    h.frame(Vec::new());
    let frame = h.frame(Vec::new());
    assert!(!h.viewport.diagnostics_shown());
    assert!(frame.find("VRAM").is_none(), "off by default");
    let toggle = frame.find("Stats").expect("the toggle");
    assert!(
        h.viewport
            .input_exclusions()
            .iter()
            .any(|r| r.contains(toggle)),
        "a press on the toggle is not viewport input"
    );

    let on = h.click(toggle);
    assert!(h.viewport.diagnostics_shown());
    assert!(on.find("VRAM").is_some() && on.find("triangles").is_some());

    let off = h.click(toggle);
    assert!(!h.viewport.diagnostics_shown());
    assert!(off.find("VRAM").is_none());
}

#[test]
fn gpu_lost_device_leaves_only_the_placeholder() {
    let Some(gpu) = gpu() else { return };
    let mut h = Harness::new(&gpu);
    h.viewport.show_diagnostics(true);
    h.frame(Vec::new());
    let before = h.frame(Vec::new());
    assert!(!h.viewport.device_lost());
    assert!(before.find("Fit").is_some() && before.find("VRAM").is_some());

    gpu.device().destroy();
    let lost = h.frame(Vec::new());
    assert!(h.viewport.device_lost());
    assert!(lost.repaint, "the shell draws its banner on the next frame");
    assert!(lost.find(DEVICE_LOST_TITLE).is_some());
    for gone in ["Fit", "Stats", "VRAM"] {
        assert!(lost.find(gone).is_none(), "{gone}");
    }
    assert!(h.viewport.input_exclusions().is_empty());
    assert!(h.viewport.take_camera_actions().is_empty());
}
