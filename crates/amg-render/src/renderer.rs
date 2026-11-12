//! The renderer: per-view offscreen targets, the frame recording and headless capture.

use std::collections::VecDeque;
use std::sync::Arc;
use std::sync::atomic::{AtomicU8, Ordering};
use std::time::{Duration, Instant};

use amg_core::{Dir3, Point3, SlotMap};
use amg_jobs::JobSystem;
use amg_jobs::gpu::{Gpu, Readback};

use crate::camera::{Camera, Projection};
use crate::cull::{DrawPlan, Frustum, MeshDraw};
use crate::pipelines::{
    DEPTH_FORMAT, FINAL_FORMAT, HDR_FORMAT, NORMAL_FORMAT, PipelineFactory, PipelineKey,
    PipelineKind, Variants, core_keys,
};
use crate::scene::{LayerMask, RECORD_BYTES, RenderScene};
use crate::shading::Lighting;
use crate::{RenderError, ViewId};

pub(crate) use crate::pipelines::FULLSCREEN_WGSL;

/// The same bytes as [`FINAL_FORMAT`], read without the sRGB decode.
pub(crate) const FINAL_FORMAT_RAW: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;

/// Instance records the buffer holds before the first growth.
const INITIAL_INSTANCE_CAPACITY: u32 = 64;
/// Visibility entries (`u32`) a view's list holds before the first growth.
const INITIAL_VISIBLE_CAPACITY: u32 = 64;

/// In-flight timestamp readbacks per view; a frame with none free is simply not timed.
const TIMER_SLOTS: usize = 4;
/// Begin and end of up to three passes (depth pre-pass, opaque, final).
const TIMER_QUERIES: u32 = 6;
/// Finished frames kept for [`Renderer::drain_gpu_timings`].
const TIMER_HISTORY: usize = 256;

const TIMER_FREE: u8 = 0;
const TIMER_IN_FLIGHT: u8 = 1;
const TIMER_MAPPED: u8 = 2;
const TIMER_FAILED: u8 = 3;

const PASSES_PLAIN: &[&str] = &["opaque", "final"];
const PASSES_PREPASS: &[&str] = &["depth prepass", "opaque", "final"];

/// How long [`Renderer::wait_for_variants`] waits by default in headless helpers.
const VARIANT_WAIT: Duration = Duration::from_secs(60);

/// Renderer settings.
#[derive(Clone, Debug)]
pub struct RendererConfig {
    /// MSAA samples of the 3D passes: 1 or 4. Default 4.
    pub msaa_samples: u32,
    /// Draw a depth-only pre-pass before the opaque pass (which then tests `Equal` and shades
    /// each sample once). Default off: the RND-02 bench measured it slower on the owner GPU
    /// (S10 Refinement log).
    pub depth_prepass: bool,
    /// Job system whose Interactive pool builds pipeline variants. `None`: a short-lived
    /// thread per variant.
    pub jobs: Option<JobSystem>,
}

impl Default for RendererConfig {
    fn default() -> Self {
        RendererConfig {
            msaa_samples: 4,
            depth_prepass: false,
            jobs: None,
        }
    }
}

/// Time spent creating one pipeline (or the shader modules).
#[derive(Clone, Debug)]
pub struct PipelineTiming {
    /// Pipeline family: `mesh`, `depth prepass`, `background`, `external layer`, `fxaa`,
    /// `final`, or `shader modules`.
    pub label: &'static str,
    /// The variant (override constants, cull mode, depth test, sample count).
    pub variant: String,
    /// Milliseconds on the building thread (shader compilation included).
    pub ms: f64,
}

/// What one [`Renderer::encode`] drew.
#[derive(Clone, Copy, Debug, Default)]
pub struct FrameStats {
    /// Instances drawn.
    pub instances_drawn: u32,
    /// Triangles drawn.
    pub triangles: u64,
    /// Instances that were drawable but outside the view frustum.
    pub culled: u32,
}

/// GPU time of one finished frame; needs `TIMESTAMP_QUERY`.
#[derive(Clone, Debug, Default)]
pub struct GpuTimings {
    /// From the start of the first pass to the end of the last, milliseconds.
    pub total_ms: f32,
    /// Per-pass times, milliseconds: `depth prepass` (when on), `opaque` (background, meshes,
    /// external layer, MSAA resolve), `final` (copy or FXAA into the sRGB image).
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
    /// A vertical gradient (linear RGB), `top` at the top edge of the view.
    Gradient {
        /// Colour at the top edge.
        top: [f32; 3],
        /// Colour at the bottom edge.
        bottom: [f32; 3],
    },
}

/// How a view shades the scene.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViewStyle {
    /// View background. Default: a dark blue-grey gradient.
    pub background: Background,
    /// Facet shading from derivatives even for meshes that have normals.
    pub flat_shading: bool,
    /// Lighting model. Default [`Lighting::Studio`].
    pub lighting: Lighting,
    /// FXAA on the resolved image (useful at 1× MSAA). Default off.
    pub fxaa: bool,
}

impl Default for ViewStyle {
    fn default() -> Self {
        ViewStyle {
            background: Background::Gradient {
                top: [0.16, 0.18, 0.22],
                bottom: [0.03, 0.035, 0.045],
            },
            flat_shading: false,
            lighting: Lighting::Studio,
            fxaa: false,
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
    /// Which instance layers this view draws. Default: all.
    pub layers: LayerMask,
    size_px: [u32; 2],
}

impl View {
    /// Current size of the view, pixels.
    pub fn size_px(&self) -> [u32; 2] {
        self.size_px
    }
}

/// Per-view style uniform (`StyleUniform` in `shaders/mesh.wgsl`).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, bytemuck::Pod, bytemuck::Zeroable)]
pub(crate) struct StyleUniform {
    bg_top: [f32; 4],
    bg_bottom: [f32; 4],
}

impl StyleUniform {
    fn new(style: &ViewStyle) -> StyleUniform {
        let (top, bottom) = match style.background {
            Background::Solid(c) => (c, c),
            Background::Gradient { top, bottom } => (top, bottom),
        };
        StyleUniform {
            bg_top: [top[0], top[1], top[2], 1.0],
            bg_bottom: [bottom[0], bottom[1], bottom[2], 1.0],
        }
    }
}

/// Offscreen targets of one view at one size and sample count.
#[derive(Debug)]
struct ViewTargets {
    size: [u32; 2],
    samples: u32,
    /// The MSAA colour target; `None` at one sample (the pass draws into `resolved` directly).
    msaa_color: Option<wgpu::TextureView>,
    /// Octahedral view normal, `samples` samples (design note §2; read by post passes).
    normal: wgpu::Texture,
    normal_view: wgpu::TextureView,
    depth: wgpu::TextureView,
    resolved: wgpu::TextureView,
    final_texture: wgpu::Texture,
    final_view: wgpu::TextureView,
    /// `resolved` for the plain copy (`textureLoad`).
    final_bind: wgpu::BindGroup,
    /// `resolved` + bilinear sampler for FXAA.
    post_bind: wgpu::BindGroup,
}

#[derive(Debug)]
struct TimerSlot {
    staging: wgpu::Buffer,
    state: Arc<AtomicU8>,
    /// Frame number, so slots that finish together are harvested in order.
    frame: u64,
    /// The timed passes of this frame, in query order.
    passes: &'static [&'static str],
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
                passes: PASSES_PLAIN,
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
            let passes = slot.passes;
            if let Some(t) = ticks {
                self.record(&t, passes);
            }
        }
    }

    fn record(&mut self, ticks: &[u64], passes: &'static [&'static str]) {
        let ms = |a: usize, b: usize| {
            (ticks[b].saturating_sub(ticks[a]) as f64 * self.period_ns * 1e-6) as f32
        };
        let last = 2 * passes.len() - 1;
        let timing = GpuTimings {
            total_ms: ms(0, last),
            passes: passes
                .iter()
                .enumerate()
                .map(|(i, &name)| (name, ms(2 * i, 2 * i + 1)))
                .collect(),
        };
        if self.finished.len() == TIMER_HISTORY {
            self.finished.pop_front();
        }
        self.finished.push_back(timing.clone());
        self.newest = Some(timing);
    }

    /// Takes a free slot for this frame.
    fn begin(&mut self, passes: &'static [&'static str]) -> Option<usize> {
        let index = self
            .slots
            .iter()
            .position(|s| s.state.load(Ordering::Acquire) == TIMER_FREE)?;
        let slot = &mut self.slots[index];
        slot.state.store(TIMER_IN_FLIGHT, Ordering::Release);
        slot.frame = self.next_frame;
        slot.passes = passes;
        self.next_frame += 1;
        Some(index)
    }

    fn writes(&self, pass: usize) -> wgpu::RenderPassTimestampWrites<'_> {
        wgpu::RenderPassTimestampWrites {
            query_set: &self.queries,
            beginning_of_pass_write_index: Some(2 * pass as u32),
            end_of_pass_write_index: Some(2 * pass as u32 + 1),
        }
    }

    /// Records the resolve and copy, and maps the staging buffer once `encoder` is submitted.
    fn finish(&self, slot: usize, encoder: &mut wgpu::CommandEncoder) {
        let slot = &self.slots[slot];
        let count = 2 * slot.passes.len() as u32;
        encoder.resolve_query_set(&self.queries, 0..count, &self.resolve, 0);
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
    style_buffer: wgpu::Buffer,
    /// Instance slots of the visible instances in draw order (`u32`).
    visible_buffer: wgpu::Buffer,
    visible_capacity: u32,
    bind_group: wgpu::BindGroup,
    /// The [`InstanceGpu::generation`] `bind_group` was built for.
    bound_generation: u64,
    timer: Option<GpuTimer>,
}

/// One per app; owns the pipelines, the scene and all views. `Send + Sync`.
#[derive(Debug)]
pub struct Renderer {
    gpu: Arc<Gpu>,
    /// Sample count asked for; `active_samples` follows once its core pipelines are built.
    requested_samples: u32,
    active_samples: u32,
    requested_prepass: bool,
    active_prepass: bool,
    view_bind_layout: wgpu::BindGroupLayout,
    variants: Variants,
    final_pipeline: wgpu::RenderPipeline,
    final_bind_layout: wgpu::BindGroupLayout,
    post_bind_layout: wgpu::BindGroupLayout,
    post_sampler: wgpu::Sampler,
    external_bind_layout: wgpu::BindGroupLayout,
    core_timings: Vec<PipelineTiming>,
    scene: RenderScene,
    views: SlotMap<ViewId, ViewSlot>,
    instance_gpu: InstanceGpu,
    /// Cumulative bytes written to the instance storage buffer.
    instance_bytes_uploaded: u64,
    runs: Vec<std::ops::Range<u32>>,
    plan: DrawPlan,
    /// Pipeline chosen for each group of `plan` (the wanted variant or its core fallback).
    group_pipelines: Vec<PipelineKey>,
}

impl Renderer {
    /// Creates the renderer and its core pipelines synchronously (Studio meshes, background,
    /// external layer and final copy at `cfg.msaa_samples`); every other variant is built
    /// lazily in the background.
    ///
    /// # Errors
    /// [`RenderError::InvalidData`] if `cfg.msaa_samples` is not 1 or 4.
    pub fn new(gpu: Arc<Gpu>, cfg: &RendererConfig) -> Result<Renderer, RenderError> {
        check_samples(cfg.msaa_samples)?;
        let device = gpu.device();

        let buffer_entry = |binding, visibility, ty| wgpu::BindGroupLayoutEntry {
            binding,
            visibility,
            ty: wgpu::BindingType::Buffer {
                ty,
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        };
        let storage = wgpu::BufferBindingType::Storage { read_only: true };
        let view_bind_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("amg view"),
            entries: &[
                buffer_entry(
                    0,
                    wgpu::ShaderStages::VERTEX_FRAGMENT,
                    wgpu::BufferBindingType::Uniform,
                ),
                buffer_entry(1, wgpu::ShaderStages::VERTEX, storage),
                buffer_entry(2, wgpu::ShaderStages::VERTEX, storage),
                buffer_entry(
                    3,
                    wgpu::ShaderStages::FRAGMENT,
                    wgpu::BufferBindingType::Uniform,
                ),
            ],
        });
        let external_bind_layout = external_bind_layout(device);
        let post_bind_layout = post_bind_layout(device);
        let final_bind_layout = texture_bind_layout(device, "amg final");
        let instance_gpu = InstanceGpu::new(device, INITIAL_INSTANCE_CAPACITY, 0);

        let mut core_timings = Vec::new();
        let start = Instant::now();
        let factory = PipelineFactory::new(
            device,
            &view_bind_layout,
            &external_bind_layout,
            &post_bind_layout,
        );
        let fullscreen = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("amg fullscreen"),
            source: wgpu::ShaderSource::Wgsl(FULLSCREEN_WGSL.into()),
        });
        core_timings.push(PipelineTiming {
            label: "shader modules",
            variant: "mesh, external layer, post, fullscreen".into(),
            ms: start.elapsed().as_secs_f64() * 1e3,
        });

        let start = Instant::now();
        let final_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("amg final"),
                bind_group_layouts: &[Some(&final_bind_layout)],
                immediate_size: 0,
            });
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
            variant: "copy x1".into(),
            ms: start.elapsed().as_secs_f64() * 1e3,
        });

        let mut variants = Variants::new(factory, cfg.jobs.clone());
        for key in core_keys(cfg.msaa_samples, cfg.depth_prepass) {
            core_timings.push(variants.build_now(key));
        }
        let post_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("amg post bilinear"),
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });
        let total: f64 = core_timings.iter().map(|t| t.ms).sum();
        tracing::debug!(
            ms = total,
            pipelines = core_timings.len(),
            "core pipeline set created"
        );

        Ok(Renderer {
            requested_samples: cfg.msaa_samples,
            active_samples: cfg.msaa_samples,
            requested_prepass: cfg.depth_prepass,
            active_prepass: cfg.depth_prepass,
            view_bind_layout,
            variants,
            final_pipeline,
            final_bind_layout,
            post_bind_layout,
            post_sampler,
            external_bind_layout,
            core_timings,
            scene: RenderScene::default(),
            views: SlotMap::with_key(),
            instance_gpu,
            instance_bytes_uploaded: 0,
            runs: Vec::new(),
            plan: DrawPlan::default(),
            group_pipelines: Vec::new(),
            gpu,
        })
    }

    /// Time spent creating the shader modules and each core pipeline in [`Renderer::new`].
    pub fn core_pipeline_timings(&self) -> &[PipelineTiming] {
        &self.core_timings
    }

    /// Time spent building each pipeline variant in the background so far (finished builds
    /// only, in completion order).
    pub fn variant_timings(&self) -> &[PipelineTiming] {
        self.variants.timings()
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

    /// The requested MSAA sample count (see [`Renderer::active_msaa_samples`]).
    pub fn msaa_samples(&self) -> u32 {
        self.requested_samples
    }

    /// The sample count frames are drawn with now. It follows [`Renderer::msaa_samples`] once
    /// the core pipelines of the requested count are built.
    pub fn active_msaa_samples(&self) -> u32 {
        self.active_samples
    }

    /// Switches the MSAA sample count of all views. Never blocks: the pipelines of a sample
    /// count not used before are built in the background, and frames keep the previous count
    /// until they are ready; the view targets are then rebuilt on the next `encode`.
    ///
    /// # Errors
    /// [`RenderError::InvalidData`] unless `samples` is 1 or 4.
    pub fn set_msaa_samples(&mut self, samples: u32) -> Result<(), RenderError> {
        check_samples(samples)?;
        self.requested_samples = samples;
        self.settle_config();
        Ok(())
    }

    /// Turns the depth pre-pass on or off (switches like [`Renderer::set_msaa_samples`]).
    pub fn set_depth_prepass(&mut self, on: bool) {
        self.requested_prepass = on;
        self.settle_config();
    }

    /// Whether frames are drawn with a depth pre-pass now.
    pub fn depth_prepass_active(&self) -> bool {
        self.active_prepass
    }

    /// Pipeline variants still being built in the background.
    pub fn pending_variants(&mut self) -> usize {
        self.variants.poll();
        self.settle_config();
        self.variants.pending()
    }

    /// Requests every variant `id` needs with its current style (all geometry kinds), at the
    /// active and the requested configuration, without drawing. Non-blocking.
    pub fn request_view_variants(&mut self, id: ViewId) {
        self.settle_config();
        let Some(style) = self.views.get(id).map(|s| s.view.style) else {
            return;
        };
        let configs = [
            (self.active_samples, self.active_prepass),
            (self.requested_samples, self.requested_prepass),
        ];
        for (samples, prepass) in configs {
            for flat in [false, true] {
                for double_sided in [false, true] {
                    self.variants.request(PipelineKey::mesh(
                        style.lighting,
                        flat,
                        double_sided,
                        prepass,
                        samples,
                    ));
                }
            }
        }
        if style.fxaa {
            self.variants.request(fxaa_key());
        }
    }

    /// Blocks until no pipeline variant is pending (or `timeout` passes) and applies a pending
    /// sample-count or pre-pass switch. Returns true when nothing is pending. Job, test and
    /// headless threads only; the UI thread lets frames fall back instead.
    pub fn wait_for_variants(&mut self, timeout: Duration) -> bool {
        let done = self.variants.wait(timeout);
        self.settle_config();
        done
    }

    /// [`Renderer::request_view_variants`] then [`Renderer::wait_for_variants`] (60 s): after
    /// it, `id` renders exactly as its style says. For headless renders and tests.
    pub fn prepare_view(&mut self, id: ViewId) -> bool {
        self.request_view_variants(id);
        self.wait_for_variants(VARIANT_WAIT)
    }

    /// Switches to the requested sample count / pre-pass once their core pipelines are ready,
    /// and requests them otherwise.
    fn settle_config(&mut self) {
        let want = (self.requested_samples, self.requested_prepass);
        if want == (self.active_samples, self.active_prepass) {
            return;
        }
        self.variants.poll();
        let keys = core_keys(want.0, want.1);
        if keys.iter().all(|&k| self.variants.has(k)) {
            (self.active_samples, self.active_prepass) = want;
            return;
        }
        for k in keys {
            if self.variants.failed(k) {
                tracing::warn!(variant = %k.describe(), "core pipeline failed; keeping the current configuration");
            }
            self.variants.request(k);
        }
    }

    /// Adds a view of `size_px` with a default camera.
    pub fn create_view(&mut self, size_px: [u32; 2]) -> ViewId {
        let device = self.gpu.device();
        let uniform = |label, size| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            })
        };
        let camera_buffer = uniform("amg camera", size_of::<crate::CameraUniform>() as u64);
        let style_buffer = uniform("amg view style", size_of::<StyleUniform>() as u64);
        let visible_buffer = storage_buffer(
            device,
            "amg visible instances",
            u64::from(INITIAL_VISIBLE_CAPACITY) * 4,
        );
        let bind_group = view_bind_group(
            device,
            &self.view_bind_layout,
            &camera_buffer,
            &self.instance_gpu.buffer,
            &visible_buffer,
            &style_buffer,
        );
        self.views.insert(ViewSlot {
            view: View {
                camera: default_camera(),
                style: ViewStyle::default(),
                layers: LayerMask::ALL,
                size_px,
            },
            targets: None,
            generation: 0,
            camera_buffer,
            style_buffer,
            visible_buffer,
            visible_capacity: INITIAL_VISIBLE_CAPACITY,
            bind_group,
            bound_generation: self.instance_gpu.generation,
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

    /// Uploads the records of new or changed instances (one `write_buffer` per run of adjacent
    /// changed slots) and grows the instance buffer when the scene outgrew it. A scene with no
    /// changes writes nothing.
    fn sync_instances(&mut self) {
        self.scene.flush_instances(&mut self.runs);
        let records = self.scene.table().records();
        let len = records.len() as u32;
        let device = self.gpu.device();
        let queue = self.gpu.queue();
        if len > self.instance_gpu.capacity {
            let capacity = len.next_power_of_two().max(INITIAL_INSTANCE_CAPACITY);
            self.instance_gpu =
                InstanceGpu::new(device, capacity, self.instance_gpu.generation + 1);
            queue.write_buffer(&self.instance_gpu.buffer, 0, bytemuck::cast_slice(records));
            self.instance_bytes_uploaded += u64::from(len) * RECORD_BYTES;
        } else {
            for run in &self.runs {
                let part = &records[run.start as usize..run.end as usize];
                queue.write_buffer(
                    &self.instance_gpu.buffer,
                    u64::from(run.start) * RECORD_BYTES,
                    bytemuck::cast_slice(part),
                );
                self.instance_bytes_uploaded += part.len() as u64 * RECORD_BYTES;
            }
        }
    }

    /// Total bytes written to the instance storage buffer so far. Changing one instance adds
    /// exactly 128 on the next `encode`; an unchanged scene adds nothing.
    pub fn instance_bytes_uploaded(&self) -> u64 {
        self.instance_bytes_uploaded
    }

    /// Records all passes for `id` into `encoder`. Never blocks. A view with a zero dimension
    /// records nothing.
    ///
    /// Passes: an optional depth pre-pass; the opaque pass (gradient background, meshes with
    /// the view's lighting into colour + octahedral normal, the external layer, MSAA resolve);
    /// the final pass (copy or FXAA into the sRGB image). A pipeline variant that is not built
    /// yet is requested and the draw falls back to the core Studio pipeline.
    ///
    /// Several views may be encoded into one encoder (one submit, one frame): the scene's
    /// instance records are shared, each view has its own camera, visibility list and targets.
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
        if !self.views.contains_key(id) {
            return Err(RenderError::UnknownKey("view"));
        }
        self.sync_instances();
        self.variants.poll();
        self.settle_config();

        let samples = self.active_samples;
        let prepass = self.active_prepass;
        let gpu = Arc::clone(&self.gpu);
        let device = gpu.device();
        let queue = gpu.queue();
        let scene = &self.scene;
        let plan = &mut self.plan;
        let variants = &mut self.variants;
        let group_pipelines = &mut self.group_pipelines;
        let slot = self
            .views
            .get_mut(id)
            .ok_or(RenderError::UnknownKey("view"))?;
        let size = slot.view.size_px;

        let mut stats = FrameStats::default();
        if size[0] == 0 || size[1] == 0 {
            return Ok(stats);
        }
        if slot
            .targets
            .as_ref()
            .is_none_or(|t| t.size != size || t.samples != samples)
        {
            slot.targets = Some(create_targets(
                device,
                &self.final_bind_layout,
                &self.post_bind_layout,
                &self.post_sampler,
                size,
                samples,
            ));
            slot.generation += 1;
        }

        let camera = slot.view.camera;
        let style = slot.view.style;
        queue.write_buffer(
            &slot.camera_buffer,
            0,
            bytemuck::bytes_of(&camera.uniform(size, camera.default_near())),
        );
        queue.write_buffer(
            &slot.style_buffer,
            0,
            bytemuck::bytes_of(&StyleUniform::new(&style)),
        );

        let frustum = Frustum::from_view_proj(&(camera.proj_matrix(size) * camera.view_matrix()));
        plan.build(
            scene.table(),
            |k| {
                scene
                    .mesh(k)
                    .filter(|m| m.index_count() > 0)
                    .map(|m| MeshDraw {
                        has_normals: m.has_normals(),
                        closed: m.is_closed(),
                    })
            },
            &frustum,
            camera.eye,
            slot.view.layers,
            style.flat_shading,
        );
        stats.culled = plan.culled;

        // Choose each group's pipeline; request variants that are not built yet.
        group_pipelines.clear();
        for g in &plan.groups {
            let want = PipelineKey::mesh(style.lighting, g.flat, g.double_sided, prepass, samples);
            let chosen = if variants.has(want) {
                want
            } else {
                variants.request(want);
                PipelineKey::mesh(Lighting::Studio, g.flat, g.double_sided, prepass, samples)
            };
            group_pipelines.push(chosen);
        }
        let use_fxaa = style.fxaa && {
            let ready = variants.has(fxaa_key());
            if !ready {
                variants.request(fxaa_key());
            }
            ready
        };
        let variants = &*variants;

        let mut rebind = slot.bound_generation != self.instance_gpu.generation;
        let needed = plan.slots.len() as u32;
        if needed > slot.visible_capacity {
            slot.visible_capacity = needed.next_power_of_two().max(INITIAL_VISIBLE_CAPACITY);
            slot.visible_buffer = storage_buffer(
                device,
                "amg visible instances",
                u64::from(slot.visible_capacity) * 4,
            );
            rebind = true;
        }
        if rebind {
            slot.bind_group = view_bind_group(
                device,
                &self.view_bind_layout,
                &slot.camera_buffer,
                &self.instance_gpu.buffer,
                &slot.visible_buffer,
                &slot.style_buffer,
            );
            slot.bound_generation = self.instance_gpu.generation;
        }
        if !plan.slots.is_empty() {
            queue.write_buffer(&slot.visible_buffer, 0, bytemuck::cast_slice(&plan.slots));
        }

        let external_bind = scene.external_layer().and_then(|layer| {
            if layer.size_px != size {
                tracing::debug!(
                    layer = ?layer.size_px,
                    view = ?size,
                    "external layer skipped: its size differs from the view"
                );
                return None;
            }
            Some(device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("amg external layer"),
                layout: &self.external_bind_layout,
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: wgpu::BindingResource::TextureView(&layer.color),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: wgpu::BindingResource::TextureView(&layer.depth),
                    },
                ],
            }))
        });

        let passes = if prepass {
            PASSES_PREPASS
        } else {
            PASSES_PLAIN
        };
        let timer_slot = slot.timer.as_mut().and_then(|t| {
            t.harvest();
            t.begin(passes)
        });
        let timer = timer_slot.and(slot.timer.as_ref());
        let pass_index = |name: &str| passes.iter().position(|&p| p == name).unwrap_or(0);

        let Some(targets) = slot.targets.as_ref() else {
            return Ok(stats);
        };
        let draw_meshes = |pass: &mut wgpu::RenderPass<'_>, depth_only: bool| {
            let mut drawn = (0u32, 0u64);
            let mut bound = None;
            for (group, &key) in plan.groups.iter().zip(group_pipelines.iter()) {
                let Some(mesh) = scene.mesh(group.mesh) else {
                    continue;
                };
                let key = if depth_only {
                    PipelineKey {
                        kind: PipelineKind::Depth {
                            flat: group.flat,
                            double_sided: group.double_sided,
                        },
                        samples,
                    }
                } else {
                    key
                };
                if bound != Some(key) {
                    let Some(pipeline) = variants.get(key) else {
                        continue;
                    };
                    pass.set_pipeline(pipeline);
                    bound = Some(key);
                }
                pass.set_vertex_buffer(0, mesh.positions.slice(..));
                if let Some(n) = mesh.normals.as_ref().filter(|_| !group.flat) {
                    pass.set_vertex_buffer(1, n.slice(..));
                }
                pass.set_index_buffer(mesh.indices.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(
                    0..mesh.index_count(),
                    0,
                    group.first..group.first + group.count,
                );
                drawn.0 += group.count;
                drawn.1 += u64::from(mesh.triangle_count()) * u64::from(group.count);
            }
            drawn
        };

        if prepass {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("amg depth prepass"),
                color_attachments: &[],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &targets.depth,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(0.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: timer.map(|t| t.writes(pass_index("depth prepass"))),
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_bind_group(0, &slot.bind_group, &[]);
            draw_meshes(&mut pass, true);
        }
        {
            let clear = match style.background {
                Background::Solid([r, g, b]) => wgpu::Color {
                    r: f64::from(r),
                    g: f64::from(g),
                    b: f64::from(b),
                    a: 1.0,
                },
                Background::Gradient { .. } => wgpu::Color::BLACK,
            };
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("amg opaque"),
                color_attachments: &[
                    Some(wgpu::RenderPassColorAttachment {
                        view: targets.msaa_color.as_ref().unwrap_or(&targets.resolved),
                        depth_slice: None,
                        resolve_target: targets.msaa_color.as_ref().map(|_| &targets.resolved),
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Clear(clear),
                            store: if targets.msaa_color.is_some() {
                                wgpu::StoreOp::Discard
                            } else {
                                wgpu::StoreOp::Store
                            },
                        },
                    }),
                    Some(wgpu::RenderPassColorAttachment {
                        view: &targets.normal_view,
                        depth_slice: None,
                        resolve_target: None,
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT),
                            store: wgpu::StoreOp::Store,
                        },
                    }),
                ],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &targets.depth,
                    depth_ops: Some(wgpu::Operations {
                        load: if prepass {
                            wgpu::LoadOp::Load
                        } else {
                            wgpu::LoadOp::Clear(0.0)
                        },
                        store: wgpu::StoreOp::Discard,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: timer.map(|t| t.writes(pass_index("opaque"))),
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_bind_group(0, &slot.bind_group, &[]);
            if matches!(style.background, Background::Gradient { .. })
                && let Some(p) = variants.get(PipelineKey {
                    kind: PipelineKind::Background,
                    samples,
                })
            {
                pass.set_pipeline(p);
                pass.draw(0..3, 0..1);
            }
            let (instances, triangles) = draw_meshes(&mut pass, false);
            stats.instances_drawn = instances;
            stats.triangles = triangles;
            if let (Some(bind), Some(p)) = (
                &external_bind,
                variants.get(PipelineKey {
                    kind: PipelineKind::External,
                    samples,
                }),
            ) {
                pass.set_pipeline(p);
                pass.set_bind_group(0, bind, &[]);
                pass.draw(0..3, 0..1);
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
                timestamp_writes: timer.map(|t| t.writes(pass_index("final"))),
                occlusion_query_set: None,
                multiview_mask: None,
            });
            match variants.get(fxaa_key()).filter(|_| use_fxaa) {
                Some(fxaa) => {
                    pass.set_pipeline(fxaa);
                    pass.set_bind_group(0, &targets.post_bind, &[]);
                }
                None => {
                    pass.set_pipeline(&self.final_pipeline);
                    pass.set_bind_group(0, &targets.final_bind, &[]);
                }
            }
            pass.draw(0..3, 0..1);
        }
        if let (Some(index), Some(timer)) = (timer_slot, slot.timer.as_ref()) {
            timer.finish(index, encoder);
        }
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

    /// The `Rg16Float` octahedral view-normal target of the last `encode` (as many samples as
    /// [`Renderer::active_msaa_samples`]; `(0, 0)` where nothing was drawn). Post passes read
    /// it; at one sample it also has `COPY_SRC` for readback.
    pub fn normal_texture(&self, id: ViewId) -> Option<&wgpu::Texture> {
        self.views.get(id)?.targets.as_ref().map(|t| &t.normal)
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
            .map_or_else(Vec::new, |t| {
                t.harvest();
                t.finished.drain(..).collect()
            })
    }

    /// Headless: encodes the view, submits it and starts reading back the final image.
    ///
    /// # Errors
    /// [`RenderError::UnknownKey`] for a removed view, [`RenderError::InvalidData`] for a view
    /// with a zero dimension, [`RenderError::Gpu`] on device loss.
    pub fn capture(&mut self, id: ViewId) -> Result<Capture, RenderError> {
        let mut captures = self.capture_many(&[id])?;
        captures
            .pop()
            .ok_or_else(|| RenderError::InvalidData("no capture was produced".into()))
    }

    /// Headless: encodes every view in `ids` into one command encoder (one frame, one submit)
    /// and starts reading back each final image, in the order of `ids`.
    ///
    /// # Errors
    /// As [`Renderer::capture`]; nothing is submitted when any view fails.
    pub fn capture_many(&mut self, ids: &[ViewId]) -> Result<Vec<Capture>, RenderError> {
        let gpu = Arc::clone(&self.gpu);
        let device = gpu.device();
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("amg capture"),
        });
        let mut pending = Vec::with_capacity(ids.len());
        for &id in ids {
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
            pending.push((buffer, bytes, [width, height], padded_row));
        }
        gpu.queue().submit([encoder.finish()]);
        Ok(pending
            .into_iter()
            .map(|(buffer, bytes, size, padded_row)| Capture {
                readback: gpu.readback::<u8>(&buffer, 0..bytes),
                size,
                padded_row,
            })
            .collect())
    }
}

fn fxaa_key() -> PipelineKey {
    PipelineKey {
        kind: PipelineKind::Fxaa,
        samples: 1,
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

/// The scene-wide instance storage buffer of 128-byte records.
#[derive(Debug)]
struct InstanceGpu {
    buffer: wgpu::Buffer,
    /// Capacity in records.
    capacity: u32,
    /// Bumped when `buffer` is replaced, so views rebuild their bind groups.
    generation: u64,
}

impl InstanceGpu {
    fn new(device: &wgpu::Device, capacity: u32, generation: u64) -> InstanceGpu {
        InstanceGpu {
            buffer: storage_buffer(device, "amg instances", u64::from(capacity) * RECORD_BYTES),
            capacity,
            generation,
        }
    }
}

fn storage_buffer(device: &wgpu::Device, label: &str, bytes: u64) -> wgpu::Buffer {
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some(label),
        size: bytes,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    })
}

fn view_bind_group(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    camera: &wgpu::Buffer,
    instances: &wgpu::Buffer,
    visible: &wgpu::Buffer,
    style: &wgpu::Buffer,
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
            wgpu::BindGroupEntry {
                binding: 2,
                resource: visible.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 3,
                resource: style.as_entire_binding(),
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

/// A filterable 2D texture at binding 0 and a filtering sampler at binding 1 (FXAA).
fn post_bind_layout(device: &wgpu::Device) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("amg post"),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Float { filterable: true },
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                count: None,
            },
        ],
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

/// Two unfilterable 2D textures (colour at 0, depth at 1) read with `textureLoad`.
fn external_bind_layout(device: &wgpu::Device) -> wgpu::BindGroupLayout {
    let texture = |binding| wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::FRAGMENT,
        ty: wgpu::BindingType::Texture {
            sample_type: wgpu::TextureSampleType::Float { filterable: false },
            view_dimension: wgpu::TextureViewDimension::D2,
            multisampled: false,
        },
        count: None,
    };
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("amg external layer"),
        entries: &[texture(0), texture(1)],
    })
}

fn create_targets(
    device: &wgpu::Device,
    final_bind_layout: &wgpu::BindGroupLayout,
    post_bind_layout: &wgpu::BindGroupLayout,
    post_sampler: &wgpu::Sampler,
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
    let normal_usage = attach
        | wgpu::TextureUsages::TEXTURE_BINDING
        | if samples == 1 {
            wgpu::TextureUsages::COPY_SRC
        } else {
            wgpu::TextureUsages::empty()
        };
    let normal = target("amg normal", NORMAL_FORMAT, samples, normal_usage, &[]);
    let normal_view = normal.create_view(&default_view);
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
    let post_bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("amg post"),
        layout: post_bind_layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::TextureView(&resolved),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::Sampler(post_sampler),
            },
        ],
    });
    ViewTargets {
        size,
        samples,
        msaa_color,
        normal,
        normal_view,
        depth,
        resolved,
        final_texture,
        final_view,
        final_bind,
        post_bind,
    }
}
