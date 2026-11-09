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
use crate::{RenderError, Renderer, RendererConfig, ViewId};

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
