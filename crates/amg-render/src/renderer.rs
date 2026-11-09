//! The renderer: per-view offscreen targets, the frame recording and headless capture.

use std::collections::VecDeque;
use std::sync::Arc;
use std::sync::atomic::{AtomicU8, Ordering};
use std::time::Instant;

use amg_core::{Dir3, Mat3, Mat4, Point3, SlotMap};
use amg_jobs::gpu::{Gpu, Readback};

use crate::camera::{Camera, Projection};
use crate::scene::RenderScene;
use crate::{MeshKey, RenderError, ViewId};

const HDR_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba16Float;
const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;
const FINAL_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8UnormSrgb;
/// The same bytes as [`FINAL_FORMAT`], read without the sRGB decode.
pub(crate) const FINAL_FORMAT_RAW: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;

const MESH_WGSL: &str = concat!(
    include_str!("shaders/camera.wgsl"),
    include_str!("shaders/mesh.wgsl")
);
pub(crate) const FULLSCREEN_WGSL: &str = include_str!("shaders/fullscreen.wgsl");

/// In-flight timestamp readbacks per view; a frame with none free is simply not timed.
const TIMER_SLOTS: usize = 4;
/// Begin and end of the opaque pass, begin and end of the final pass.
const TIMER_QUERIES: u32 = 4;
/// Finished frames kept for [`Renderer::drain_gpu_timings`].
const TIMER_HISTORY: usize = 256;

const TIMER_FREE: u8 = 0;
const TIMER_IN_FLIGHT: u8 = 1;
const TIMER_MAPPED: u8 = 2;
const TIMER_FAILED: u8 = 3;

/// Renderer settings.
#[derive(Clone, Debug)]
pub struct RendererConfig {
    /// MSAA samples of the 3D passes: 1 or 4. Default 4.
    pub msaa_samples: u32,
}

impl Default for RendererConfig {
    fn default() -> Self {
        RendererConfig { msaa_samples: 4 }
    }
}

/// Time spent creating one core pipeline.
#[derive(Clone, Copy, Debug)]
pub struct PipelineTiming {
    /// Pipeline name.
    pub label: &'static str,
    /// Milliseconds on the calling thread (shader compilation included).
    pub ms: f64,
}

/// What one [`Renderer::encode`] drew.
#[derive(Clone, Copy, Debug, Default)]
pub struct FrameStats {
    /// Instances drawn.
    pub instances_drawn: u32,
    /// Triangles drawn.
    pub triangles: u64,
}

/// GPU time of one finished frame; needs `TIMESTAMP_QUERY`.
#[derive(Clone, Debug, Default)]
pub struct GpuTimings {
    /// From the start of the first pass to the end of the last, milliseconds.
    pub total_ms: f32,
    /// Per-pass times, milliseconds.
    pub passes: Vec<(&'static str, f32)>,
}

/// Final image of a view: sRGB RGBA8, tightly packed, top row first.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RgbaImage {
    /// Width and height.
    pub size: [u32; 2],
    /// `4 * width * height` bytes.
    pub pixels: Vec<u8>,
}

impl RgbaImage {
    /// The RGBA bytes of the pixel at column `x`, row `y`.
    ///
    /// # Panics
    /// If the pixel is outside the image.
    pub fn pixel(&self, x: u32, y: u32) -> [u8; 4] {
        assert!(x < self.size[0] && y < self.size[1], "pixel out of range");
        let i = (y as usize * self.size[0] as usize + x as usize) * 4;
        [
            self.pixels[i],
            self.pixels[i + 1],
            self.pixels[i + 2],
            self.pixels[i + 3],
        ]
    }
}

/// A [`Renderer::capture`] in flight.
#[derive(Debug)]
pub struct Capture {
    readback: Readback<u8>,
    size: [u32; 2],
    padded_row: u32,
}

impl Capture {
    /// Non-blocking poll: `Some` once the copy has finished, and `None` after that.
    pub fn try_get(&mut self) -> Option<Result<RgbaImage, RenderError>> {
        let bytes = self.readback.try_get()?;
        Some(
            bytes
                .map(|b| unpad(&b, self.size, self.padded_row))
                .map_err(RenderError::from),
        )
    }

    /// Blocks until the copy has finished. Job and test threads only.
    ///
    /// # Errors
    /// [`RenderError::Gpu`] on device loss or a failed map.
    pub fn wait(self) -> Result<RgbaImage, RenderError> {
        let bytes = self.readback.wait()?;
        Ok(unpad(&bytes, self.size, self.padded_row))
    }
}

fn unpad(bytes: &[u8], size: [u32; 2], padded_row: u32) -> RgbaImage {
    let row = size[0] as usize * 4;
    let mut pixels = Vec::with_capacity(row * size[1] as usize);
    for line in bytes.chunks_exact(padded_row as usize) {
        pixels.extend_from_slice(&line[..row]);
    }
    RgbaImage { size, pixels }
}

/// View background.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Background {
    /// One linear RGB colour.
    Solid([f32; 3]),
}

/// How a view shades the scene.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViewStyle {
    /// View background.
    pub background: Background,
    /// Facet shading from derivatives even for meshes that have normals.
    pub flat_shading: bool,
}

impl Default for ViewStyle {
    fn default() -> Self {
        ViewStyle {
            background: Background::Solid([0.05, 0.06, 0.08]),
            flat_shading: false,
        }
    }
}

/// One viewport: camera and style. Its offscreen targets live in the [`Renderer`].
#[derive(Clone, Debug)]
pub struct View {
    /// The camera.
    pub camera: Camera,
    /// Shading options.
    pub style: ViewStyle,
    size_px: [u32; 2],
}

impl View {
    /// Current size of the view, pixels.
    pub fn size_px(&self) -> [u32; 2] {
        self.size_px
    }
}

/// One record of the instance storage buffer (128 bytes, `shaders/mesh.wgsl`).
#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
struct InstanceRecord {
    model_rte: [[f32; 4]; 4],
    normal_from_local: [[f32; 4]; 3],
    color: [f32; 4],
}

#[derive(Debug)]
struct MeshPipelines {
    smooth: wgpu::RenderPipeline,
    flat: wgpu::RenderPipeline,
}

/// Offscreen targets of one view at one size and sample count.
#[derive(Debug)]
struct ViewTargets {
    size: [u32; 2],
    samples: u32,
    /// The MSAA colour target; `None` at one sample (the pass draws into `resolved` directly).
    msaa_color: Option<wgpu::TextureView>,
    depth: wgpu::TextureView,
    resolved: wgpu::TextureView,
    final_texture: wgpu::Texture,
    final_view: wgpu::TextureView,
    final_bind: wgpu::BindGroup,
}

#[derive(Debug)]
struct TimerSlot {
    staging: wgpu::Buffer,
    state: Arc<AtomicU8>,
    /// Frame number, so slots that finish together are harvested in order.
    frame: u64,
}

/// Non-blocking GPU timestamps of one view.
#[derive(Debug)]
struct GpuTimer {
    queries: wgpu::QuerySet,
    resolve: wgpu::Buffer,
    slots: Vec<TimerSlot>,
    period_ns: f64,
    next_frame: u64,
    finished: VecDeque<GpuTimings>,
    newest: Option<GpuTimings>,
}

impl GpuTimer {
    fn new(gpu: &Gpu) -> Option<GpuTimer> {
        if !gpu.has(wgpu::Features::TIMESTAMP_QUERY) {
            return None;
        }
        let device = gpu.device();
        let bytes = u64::from(TIMER_QUERIES) * size_of::<u64>() as u64;
        let queries = device.create_query_set(&wgpu::QuerySetDescriptor {
            label: Some("amg view timer"),
            ty: wgpu::QueryType::Timestamp,
            count: TIMER_QUERIES,
        });
        let resolve = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("amg view timer resolve"),
            size: bytes,
            usage: wgpu::BufferUsages::QUERY_RESOLVE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        let slots = (0..TIMER_SLOTS)
            .map(|_| TimerSlot {
                staging: device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("amg view timer staging"),
                    size: bytes,
                    usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                }),
                state: Arc::new(AtomicU8::new(TIMER_FREE)),
                frame: 0,
            })
            .collect();
        Some(GpuTimer {
            queries,
            resolve,
            slots,
            period_ns: f64::from(gpu.queue().get_timestamp_period()),
            next_frame: 0,
            finished: VecDeque::with_capacity(TIMER_HISTORY),
            newest: None,
        })
    }

    /// Reads every slot whose copy has finished, oldest frame first.
    fn harvest(&mut self) {
        let mut ready: Vec<usize> = (0..self.slots.len())
            .filter(|&i| self.slots[i].state.load(Ordering::Acquire) >= TIMER_MAPPED)
            .collect();
        ready.sort_by_key(|&i| self.slots[i].frame);
        for i in ready {
            let slot = &self.slots[i];
            let mapped = slot.state.load(Ordering::Acquire) == TIMER_MAPPED;
            let ticks = mapped
                .then(|| {
                    slot.staging
                        .get_mapped_range(..)
                        .ok()
                        .map(|v| bytemuck::pod_collect_to_vec::<u8, u64>(&v))
                })
                .flatten();
            if mapped {
                slot.staging.unmap();
            }
            slot.state.store(TIMER_FREE, Ordering::Release);
            if let Some(t) = ticks {
                self.record(&t);
            }
        }
    }

    fn record(&mut self, ticks: &[u64]) {
        let ms = |a: usize, b: usize| {
            (ticks[b].saturating_sub(ticks[a]) as f64 * self.period_ns * 1e-6) as f32
        };
        let timing = GpuTimings {
            total_ms: ms(0, 3),
            passes: vec![("opaque", ms(0, 1)), ("final", ms(2, 3))],
        };
        if self.finished.len() == TIMER_HISTORY {
            self.finished.pop_front();
        }
        self.finished.push_back(timing.clone());
        self.newest = Some(timing);
    }

    /// Takes a free slot for this frame.
    fn begin(&mut self) -> Option<usize> {
        let index = self
            .slots
            .iter()
            .position(|s| s.state.load(Ordering::Acquire) == TIMER_FREE)?;
        self.slots[index]
            .state
            .store(TIMER_IN_FLIGHT, Ordering::Release);
        self.slots[index].frame = self.next_frame;
        self.next_frame += 1;
        Some(index)
    }

    /// Records the resolve and copy, and maps the staging buffer once `encoder` is submitted.
    fn finish(&self, slot: usize, encoder: &mut wgpu::CommandEncoder) {
        let slot = &self.slots[slot];
        encoder.resolve_query_set(&self.queries, 0..TIMER_QUERIES, &self.resolve, 0);
        encoder.copy_buffer_to_buffer(&self.resolve, 0, &slot.staging, 0, self.resolve.size());
        let state = Arc::clone(&slot.state);
        encoder.map_buffer_on_submit(&slot.staging, wgpu::MapMode::Read, .., move |r| {
            let done = if r.is_ok() {
                TIMER_MAPPED
            } else {
                TIMER_FAILED
            };
            state.store(done, Ordering::Release);
        });
    }
}

#[derive(Debug)]
struct ViewSlot {
    view: View,
    targets: Option<ViewTargets>,
    /// Bumped whenever the targets are rebuilt, so `egui_glue` can refresh its bind groups.
    generation: u64,
    camera_buffer: wgpu::Buffer,
    instance_buffer: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    timer: Option<GpuTimer>,
}

/// One per app; owns the pipelines, the scene and all views. `Send + Sync`.
#[derive(Debug)]
pub struct Renderer {
    gpu: Arc<Gpu>,
    msaa_samples: u32,
    mesh_shader: wgpu::ShaderModule,
    mesh_layout: wgpu::PipelineLayout,
    view_bind_layout: wgpu::BindGroupLayout,
    /// Indexed by [`Renderer::pipeline_slot`]; the other sample count is built on first use.
    mesh_pipelines: [Option<MeshPipelines>; 2],
    final_pipeline: wgpu::RenderPipeline,
    final_bind_layout: wgpu::BindGroupLayout,
    core_timings: Vec<PipelineTiming>,
    scene: RenderScene,
    views: SlotMap<ViewId, ViewSlot>,
    records: Vec<InstanceRecord>,
    draws: Vec<(MeshKey, u32)>,
}

impl Renderer {
    /// Creates the renderer and its core pipelines synchronously.
    ///
    /// # Errors
    /// [`RenderError::InvalidData`] if `cfg.msaa_samples` is not 1 or 4.
    pub fn new(gpu: Arc<Gpu>, cfg: &RendererConfig) -> Result<Renderer, RenderError> {
        check_samples(cfg.msaa_samples)?;
        let device = gpu.device();

        let view_bind_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("amg view"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::VERTEX,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
            ],
        });
        let mesh_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("amg mesh"),
            source: wgpu::ShaderSource::Wgsl(MESH_WGSL.into()),
        });
        let mesh_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("amg mesh"),
            bind_group_layouts: &[Some(&view_bind_layout)],
            immediate_size: 0,
        });

        let final_bind_layout = texture_bind_layout(device, "amg final");
        let fullscreen = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("amg fullscreen"),
            source: wgpu::ShaderSource::Wgsl(FULLSCREEN_WGSL.into()),
        });
        let final_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("amg final"),
                bind_group_layouts: &[Some(&final_bind_layout)],
                immediate_size: 0,
            });

        let mut core_timings = Vec::new();
        let start = Instant::now();
        let final_pipeline = fullscreen_pipeline(
            device,
            "amg final",
            &final_pipeline_layout,
            &fullscreen,
            FINAL_FORMAT,
            1,
        );
        core_timings.push(PipelineTiming {
            label: "final",
            ms: start.elapsed().as_secs_f64() * 1e3,
        });

        let mut renderer = Renderer {
            msaa_samples: cfg.msaa_samples,
            mesh_shader,
            mesh_layout,
            view_bind_layout,
            mesh_pipelines: [None, None],
            final_pipeline,
            final_bind_layout,
            core_timings,
            scene: RenderScene::default(),
            views: SlotMap::with_key(),
            records: Vec::new(),
            draws: Vec::new(),
            gpu,
        };
        renderer.ensure_mesh_pipelines(cfg.msaa_samples);
        Ok(renderer)
    }

    fn pipeline_slot(samples: u32) -> usize {
        usize::from(samples != 1)
    }

    fn ensure_mesh_pipelines(&mut self, samples: u32) {
        let slot = Self::pipeline_slot(samples);
        if self.mesh_pipelines[slot].is_some() {
            return;
        }
        let device = self.gpu.device();
        let mut timed = |label: &'static str, flat: bool| {
            let start = Instant::now();
            let pipeline =
                mesh_pipeline(device, &self.mesh_layout, &self.mesh_shader, samples, flat);
            self.core_timings.push(PipelineTiming {
                label,
                ms: start.elapsed().as_secs_f64() * 1e3,
            });
            pipeline
        };
        let smooth = timed("mesh smooth", false);
        let flat = timed("mesh flat", true);
        self.mesh_pipelines[slot] = Some(MeshPipelines { smooth, flat });
    }

    /// Time spent creating each core pipeline so far.
    pub fn core_pipeline_timings(&self) -> &[PipelineTiming] {
        &self.core_timings
    }

    /// The shared GPU.
    pub fn gpu(&self) -> &Arc<Gpu> {
        &self.gpu
    }

    /// The scene.
    pub fn scene(&self) -> &RenderScene {
        &self.scene
    }

    /// The scene, mutably.
    pub fn scene_mut(&mut self) -> &mut RenderScene {
        &mut self.scene
    }

    /// Current MSAA sample count.
    pub fn msaa_samples(&self) -> u32 {
        self.msaa_samples
    }

    /// Switches the MSAA sample count of all views. The view targets are rebuilt on their next
    /// `encode`; the pipelines of a sample count not used before are compiled here, so this
    /// briefly blocks the first time (RND-02 moves variants onto jobs).
    ///
    /// # Errors
    /// [`RenderError::InvalidData`] unless `samples` is 1 or 4.
    pub fn set_msaa_samples(&mut self, samples: u32) -> Result<(), RenderError> {
        check_samples(samples)?;
        self.ensure_mesh_pipelines(samples);
        self.msaa_samples = samples;
        Ok(())
    }

    /// Adds a view of `size_px` with a default camera.
    pub fn create_view(&mut self, size_px: [u32; 2]) -> ViewId {
        let device = self.gpu.device();
        let camera_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("amg camera"),
            size: size_of::<crate::CameraUniform>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let instance_buffer = instance_buffer(device, 16);
        let bind_group = view_bind_group(
            device,
            &self.view_bind_layout,
            &camera_buffer,
            &instance_buffer,
        );
        self.views.insert(ViewSlot {
            view: View {
                camera: default_camera(),
                style: ViewStyle::default(),
                size_px,
            },
            targets: None,
            generation: 0,
            camera_buffer,
            instance_buffer,
            bind_group,
            timer: GpuTimer::new(&self.gpu),
        })
    }

    /// Removes a view and its targets.
    pub fn remove_view(&mut self, id: ViewId) {
        self.views.remove(id);
    }

    /// The view, if it exists.
    pub fn view(&self, id: ViewId) -> Option<&View> {
        self.views.get(id).map(|s| &s.view)
    }

    /// Mutable access to the view.
    pub fn view_mut(&mut self, id: ViewId) -> Option<&mut View> {
        self.views.get_mut(id).map(|s| &mut s.view)
    }

    /// Sets the size of a view. Its targets are rebuilt on the next `encode`.
    pub fn set_view_size(&mut self, id: ViewId, size_px: [u32; 2]) {
        if let Some(slot) = self.views.get_mut(id) {
            slot.view.size_px = size_px;
        }
    }

    /// Records all passes for `id` into `encoder`. Never blocks. A view with a zero dimension
    /// records nothing.
    ///
    /// # Errors
    /// [`RenderError::UnknownKey`] for a removed view, [`RenderError::Gpu`] on device loss.
    pub fn encode(
        &mut self,
        id: ViewId,
        encoder: &mut wgpu::CommandEncoder,
    ) -> Result<FrameStats, RenderError> {
        // Non-blocking: reports device loss and runs finished map callbacks.
        self.gpu.poll()?;

        let samples = self.msaa_samples;
        let mut records = std::mem::take(&mut self.records);
        let mut draws = std::mem::take(&mut self.draws);
        let camera = self
            .views
            .get(id)
            .ok_or(RenderError::UnknownKey("view"))?
            .view
            .camera;
        collect_instances(&self.scene, &camera, &mut records, &mut draws);

        let gpu = Arc::clone(&self.gpu);
        let device = gpu.device();
        let queue = gpu.queue();
        let final_pipeline = &self.final_pipeline;
        let final_bind_layout = &self.final_bind_layout;
        let view_bind_layout = &self.view_bind_layout;
        let scene = &self.scene;
        let pipelines = self.mesh_pipelines[Self::pipeline_slot(samples)]
            .as_ref()
            .ok_or_else(|| {
                RenderError::InvalidData("mesh pipelines for this sample count are missing".into())
            })?;
        let slot = self
            .views
            .get_mut(id)
            .ok_or(RenderError::UnknownKey("view"))?;
        let size = slot.view.size_px;

        let mut stats = FrameStats::default();
        if size[0] != 0 && size[1] != 0 {
            if slot
                .targets
                .as_ref()
                .is_none_or(|t| t.size != size || t.samples != samples)
            {
                slot.targets = Some(create_targets(device, final_bind_layout, size, samples));
                slot.generation += 1;
            }

            queue.write_buffer(
                &slot.camera_buffer,
                0,
                bytemuck::bytes_of(&camera.uniform(size, camera.default_near())),
            );
            let needed = (records.len().max(1) * size_of::<InstanceRecord>()) as u64;
            if slot.instance_buffer.size() < needed {
                slot.instance_buffer = instance_buffer(device, (records.len() * 2).max(16) as u64);
                slot.bind_group = view_bind_group(
                    device,
                    view_bind_layout,
                    &slot.camera_buffer,
                    &slot.instance_buffer,
                );
            }
            if !records.is_empty() {
                queue.write_buffer(&slot.instance_buffer, 0, bytemuck::cast_slice(&records));
            }

            let timer_slot = slot.timer.as_mut().and_then(|t| {
                t.harvest();
                t.begin()
            });

            let Some(targets) = slot.targets.as_ref() else {
                return Ok(stats);
            };
            let flat_view = slot.view.style.flat_shading;
            let Background::Solid([r, g, b]) = slot.view.style.background;
            {
                let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: Some("amg opaque"),
                    color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                        view: targets.msaa_color.as_ref().unwrap_or(&targets.resolved),
                        depth_slice: None,
                        resolve_target: targets.msaa_color.as_ref().map(|_| &targets.resolved),
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Clear(wgpu::Color {
                                r: f64::from(r),
                                g: f64::from(g),
                                b: f64::from(b),
                                a: 1.0,
                            }),
                            store: if targets.msaa_color.is_some() {
                                wgpu::StoreOp::Discard
                            } else {
                                wgpu::StoreOp::Store
                            },
                        },
                    })],
                    depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                        view: &targets.depth,
                        depth_ops: Some(wgpu::Operations {
                            load: wgpu::LoadOp::Clear(0.0),
                            store: wgpu::StoreOp::Discard,
                        }),
                        stencil_ops: None,
                    }),
                    timestamp_writes: timer_slot.and(slot.timer.as_ref()).map(|t| {
                        wgpu::RenderPassTimestampWrites {
                            query_set: &t.queries,
                            beginning_of_pass_write_index: Some(0),
                            end_of_pass_write_index: Some(1),
                        }
                    }),
                    occlusion_query_set: None,
                    multiview_mask: None,
                });
                pass.set_bind_group(0, &slot.bind_group, &[]);
                for &(key, index) in &draws {
                    let Some(mesh) = scene.mesh(key) else {
                        continue;
                    };
                    let normals = mesh.normals.as_ref().filter(|_| !flat_view);
                    pass.set_pipeline(if normals.is_some() {
                        &pipelines.smooth
                    } else {
                        &pipelines.flat
                    });
                    pass.set_vertex_buffer(0, mesh.positions.slice(..));
                    if let Some(n) = normals {
                        pass.set_vertex_buffer(1, n.slice(..));
                    }
                    pass.set_index_buffer(mesh.indices.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..mesh.index_count(), 0, index..index + 1);
                    stats.instances_drawn += 1;
                    stats.triangles += u64::from(mesh.triangle_count());
                }
            }
            {
                let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: Some("amg final"),
                    color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                        view: &targets.final_view,
                        depth_slice: None,
                        resolve_target: None,
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                            store: wgpu::StoreOp::Store,
                        },
                    })],
                    depth_stencil_attachment: None,
                    timestamp_writes: timer_slot.and(slot.timer.as_ref()).map(|t| {
                        wgpu::RenderPassTimestampWrites {
                            query_set: &t.queries,
                            beginning_of_pass_write_index: Some(2),
                            end_of_pass_write_index: Some(3),
                        }
                    }),
                    occlusion_query_set: None,
                    multiview_mask: None,
                });
                pass.set_pipeline(final_pipeline);
                pass.set_bind_group(0, &targets.final_bind, &[]);
                pass.draw(0..3, 0..1);
            }
            if let (Some(index), Some(timer)) = (timer_slot, slot.timer.as_ref()) {
                timer.finish(index, encoder);
            }
        }

        records.clear();
        draws.clear();
        self.records = records;
        self.draws = draws;
        Ok(stats)
    }

    /// Final single-sample `Rgba8UnormSrgb` image of the last `encode`.
    pub fn final_texture(&self, id: ViewId) -> Option<&wgpu::Texture> {
        self.views
            .get(id)?
            .targets
            .as_ref()
            .map(|t| &t.final_texture)
    }

    /// Bumped whenever the targets of `id` are rebuilt.
    #[cfg(feature = "egui")]
    pub(crate) fn view_generation(&self, id: ViewId) -> Option<u64> {
        self.views.get(id).map(|s| s.generation)
    }

    /// GPU pass timings of the newest finished frame; `None` without `TIMESTAMP_QUERY` or
    /// before the first frame finishes.
    pub fn gpu_timings(&self, id: ViewId) -> Option<GpuTimings> {
        self.views.get(id)?.timer.as_ref()?.newest.clone()
    }

    /// Takes every finished frame's timings since the last call (bounded history, oldest
    /// first), so a benchmark sees each frame exactly once.
    pub fn drain_gpu_timings(&mut self, id: ViewId) -> Vec<GpuTimings> {
        self.views
            .get_mut(id)
            .and_then(|s| s.timer.as_mut())
            .map_or_else(Vec::new, |t| t.finished.drain(..).collect())
    }

    /// Headless: encodes the view, submits it and starts reading back the final image.
    ///
    /// # Errors
    /// [`RenderError::UnknownKey`] for a removed view, [`RenderError::InvalidData`] for a view
    /// with a zero dimension, [`RenderError::Gpu`] on device loss.
    pub fn capture(&mut self, id: ViewId) -> Result<Capture, RenderError> {
        let gpu = Arc::clone(&self.gpu);
        let device = gpu.device();
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("amg capture"),
        });
        self.encode(id, &mut encoder)?;
        let slot = self.views.get(id).ok_or(RenderError::UnknownKey("view"))?;
        let [width, height] = slot.view.size_px;
        let targets = slot
            .targets
            .as_ref()
            .filter(|_| width != 0 && height != 0)
            .ok_or_else(|| RenderError::InvalidData("the view has a zero dimension".into()))?;
        let padded_row = (width * 4).next_multiple_of(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT);
        let bytes = u64::from(padded_row) * u64::from(height);
        let buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("amg capture"),
            size: bytes,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        encoder.copy_texture_to_buffer(
            targets.final_texture.as_image_copy(),
            wgpu::TexelCopyBufferInfo {
                buffer: &buffer,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(padded_row),
                    rows_per_image: Some(height),
                },
            },
            wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
        );
        gpu.queue().submit([encoder.finish()]);
        Ok(Capture {
            readback: gpu.readback::<u8>(&buffer, 0..bytes),
            size: [width, height],
            padded_row,
        })
    }
}

fn check_samples(samples: u32) -> Result<(), RenderError> {
    if samples == 1 || samples == 4 {
        Ok(())
    } else {
        Err(RenderError::InvalidData(format!(
            "MSAA sample count must be 1 or 4, got {samples}"
        )))
    }
}

fn default_camera() -> Camera {
    Camera {
        eye: Point3::new(0.0, -200.0, 100.0),
        target: Point3::ZERO,
        up: Dir3::Z,
        projection: Projection::Perspective {
            fov_y: 45f64.to_radians(),
        },
    }
}

/// Fills `records` and `draws` from the visible instances, in f64 and relative to the eye.
fn collect_instances(
    scene: &RenderScene,
    camera: &Camera,
    records: &mut Vec<InstanceRecord>,
    draws: &mut Vec<(MeshKey, u32)>,
) {
    records.clear();
    draws.clear();
    for inst in scene.instances().filter(|i| i.visible) {
        let Some((key, mesh)) = inst.mesh.and_then(|k| Some((k, scene.mesh(k)?))) else {
            continue;
        };
        // M = T(-eye) * world_from_local * T(origin), design note §3.
        let model = Mat4::from_translation(-camera.eye)
            * Mat4::from(inst.transform)
            * Mat4::from_translation(mesh.origin);
        let normal = Mat3::from(inst.transform.matrix3).inverse().transpose();
        draws.push((key, records.len() as u32));
        records.push(InstanceRecord {
            model_rte: model.as_mat4().to_cols_array_2d(),
            normal_from_local: [
                normal.x_axis.as_vec3().extend(0.0).to_array(),
                normal.y_axis.as_vec3().extend(0.0).to_array(),
                normal.z_axis.as_vec3().extend(0.0).to_array(),
            ],
            color: inst.material.base_color,
        });
    }
}

fn instance_buffer(device: &wgpu::Device, count: u64) -> wgpu::Buffer {
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("amg instances"),
        size: count * size_of::<InstanceRecord>() as u64,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    })
}

fn view_bind_group(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    camera: &wgpu::Buffer,
    instances: &wgpu::Buffer,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("amg view"),
        layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: camera.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: instances.as_entire_binding(),
            },
        ],
    })
}

/// One unfilterable 2D texture at binding 0, read with `textureLoad`.
pub(crate) fn texture_bind_layout(device: &wgpu::Device, label: &str) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some(label),
        entries: &[wgpu::BindGroupLayoutEntry {
            binding: 0,
            visibility: wgpu::ShaderStages::FRAGMENT,
            ty: wgpu::BindingType::Texture {
                sample_type: wgpu::TextureSampleType::Float { filterable: false },
                view_dimension: wgpu::TextureViewDimension::D2,
                multisampled: false,
            },
            count: None,
        }],
    })
}

pub(crate) fn texture_bind_group(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    view: &wgpu::TextureView,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("amg copy source"),
        layout,
        entries: &[wgpu::BindGroupEntry {
            binding: 0,
            resource: wgpu::BindingResource::TextureView(view),
        }],
    })
}

/// A pipeline that copies its source texture over the whole target, without blending.
pub(crate) fn fullscreen_pipeline(
    device: &wgpu::Device,
    label: &str,
    layout: &wgpu::PipelineLayout,
    module: &wgpu::ShaderModule,
    format: wgpu::TextureFormat,
    samples: u32,
) -> wgpu::RenderPipeline {
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some(label),
        layout: Some(layout),
        vertex: wgpu::VertexState {
            module,
            entry_point: Some("vs_fullscreen"),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            buffers: &[],
        },
        primitive: wgpu::PrimitiveState::default(),
        depth_stencil: None,
        multisample: wgpu::MultisampleState {
            count: samples,
            ..Default::default()
        },
        fragment: Some(wgpu::FragmentState {
            module,
            entry_point: Some("fs_copy"),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            targets: &[Some(wgpu::ColorTargetState {
                format,
                blend: None,
                write_mask: wgpu::ColorWrites::ALL,
            })],
        }),
        multiview_mask: None,
        cache: None,
    })
}

fn mesh_pipeline(
    device: &wgpu::Device,
    layout: &wgpu::PipelineLayout,
    module: &wgpu::ShaderModule,
    samples: u32,
    flat: bool,
) -> wgpu::RenderPipeline {
    let float3 = |location| wgpu::VertexAttribute {
        format: wgpu::VertexFormat::Float32x3,
        offset: 0,
        shader_location: location,
    };
    let positions = [float3(0)];
    let normals = [float3(1)];
    let stride = size_of::<[f32; 3]>() as u64;
    let buffer = |attributes| {
        Some(wgpu::VertexBufferLayout {
            array_stride: stride,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes,
        })
    };
    let two = [buffer(&positions), buffer(&normals)];
    let one = [buffer(&positions)];
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some(if flat {
            "amg mesh flat"
        } else {
            "amg mesh smooth"
        }),
        layout: Some(layout),
        vertex: wgpu::VertexState {
            module,
            entry_point: Some(if flat { "vs_flat" } else { "vs_smooth" }),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            buffers: if flat { &one } else { &two },
        },
        primitive: wgpu::PrimitiveState {
            topology: wgpu::PrimitiveTopology::TriangleList,
            front_face: wgpu::FrontFace::Ccw,
            cull_mode: Some(wgpu::Face::Back),
            ..Default::default()
        },
        depth_stencil: Some(wgpu::DepthStencilState {
            format: DEPTH_FORMAT,
            depth_write_enabled: Some(true),
            depth_compare: Some(wgpu::CompareFunction::GreaterEqual),
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState::default(),
        }),
        multisample: wgpu::MultisampleState {
            count: samples,
            ..Default::default()
        },
        fragment: Some(wgpu::FragmentState {
            module,
            entry_point: Some(if flat { "fs_flat" } else { "fs_smooth" }),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            targets: &[Some(wgpu::ColorTargetState {
                format: HDR_FORMAT,
                blend: None,
                write_mask: wgpu::ColorWrites::ALL,
            })],
        }),
        multiview_mask: None,
        cache: None,
    })
}

fn create_targets(
    device: &wgpu::Device,
    final_bind_layout: &wgpu::BindGroupLayout,
    size: [u32; 2],
    samples: u32,
) -> ViewTargets {
    let target = |label, format, sample_count, usage, view_formats: &'static [_]| {
        device.create_texture(&wgpu::TextureDescriptor {
            label: Some(label),
            size: wgpu::Extent3d {
                width: size[0],
                height: size[1],
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count,
            dimension: wgpu::TextureDimension::D2,
            format,
            usage,
            view_formats,
        })
    };
    let attach = wgpu::TextureUsages::RENDER_ATTACHMENT;
    let default_view = wgpu::TextureViewDescriptor::default();

    let msaa_color = (samples > 1).then(|| {
        target("amg msaa colour", HDR_FORMAT, samples, attach, &[]).create_view(&default_view)
    });
    let depth = target("amg depth", DEPTH_FORMAT, samples, attach, &[]).create_view(&default_view);
    let resolved = target(
        "amg resolved",
        HDR_FORMAT,
        1,
        attach | wgpu::TextureUsages::TEXTURE_BINDING,
        &[],
    )
    .create_view(&default_view);
    let final_texture = target(
        "amg final",
        FINAL_FORMAT,
        1,
        attach | wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_SRC,
        &[FINAL_FORMAT_RAW],
    );
    let final_view = final_texture.create_view(&default_view);
    let final_bind = texture_bind_group(device, final_bind_layout, &resolved);
    ViewTargets {
        size,
        samples,
        msaa_color,
        depth,
        resolved,
        final_texture,
        final_view,
        final_bind,
    }
}
