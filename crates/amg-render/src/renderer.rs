//! The renderer: per-view offscreen targets, the frame recording and headless capture.

use std::collections::{HashMap, VecDeque};
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::{Arc, OnceLock};
use std::time::{Duration, Instant};

use amg_core::{Dir3, Point3, SlotMap};
use amg_jobs::JobSystem;
use amg_jobs::gpu::{Gpu, GpuError, Readback};

use crate::analysis::{
    ANALYSIS_RECORD_BYTES, AnalysisOutput, analysis_bind_layout, create_lut, lut_sampler,
};
use crate::camera::{Camera, DepthRange, Projection};
use crate::cull::{DrawPlan, Frustum, MeshDraw};
use crate::diagnostics::{Diagnostics, GpuErrorLog, VramUsage, texture_bytes};
use crate::highlight::HighlightLayer;
use crate::lines::{
    GpuEdges, LINE_RECORD_BYTES, LineDraw, LineGeometry, LinePass, LinePlan, LineView, VisualStyle,
};
use crate::nav::ViewCube;
use crate::overlay::{
    HandleHighlights, LayerDraws, OverlayGpu, OverlayInset, ProjectedLabel, Tessellator, TriDraw,
};
use crate::pick::{
    InFlight, MAP_DONE, MAP_FAILED, MAP_PENDING, OwnerSizes, PickBackend, PickFilter, PickRegion,
    PickRequest, PickResult, PickTicket, PointReadback, QueuedPick, ReadbackKind, RectReadback,
    build_rect_table, point_region,
};
use crate::pipelines::{
    DEPTH_FORMAT, FINAL_FORMAT, HDR_FORMAT, Layouts, NORMAL_FORMAT, PipelineFactory, PipelineKey,
    PipelineKind, Variants, core_keys,
};
use crate::pipelines::{FaceMode, PICK_ID_FORMAT, PICK_POS_FORMAT};
use crate::scene::Layer;
use crate::scene::{LayerMask, RECORD_BYTES, RenderScene};
use crate::section::{SECTION_UNIFORM_BYTES, SectionUniform};
use crate::shading::Lighting;
use crate::upload::GpuMesh;
use crate::{InstanceKey, MeshKey, PickId, RenderError, ViewId};

pub(crate) use crate::pipelines::FULLSCREEN_WGSL;

/// The same bytes as [`FINAL_FORMAT`], read without the sRGB decode.
pub(crate) const FINAL_FORMAT_RAW: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;

/// Instance records the buffer holds before the first growth.
const INITIAL_INSTANCE_CAPACITY: u32 = 64;
/// Visibility entries (`u32`) a view's list holds before the first growth.
const INITIAL_VISIBLE_CAPACITY: u32 = 64;
/// Highlight-state words the buffer holds before the first growth.
const INITIAL_STATE_WORDS: u64 = 64;

/// In-flight timestamp readbacks per view; a frame with none free is simply not timed.
const TIMER_SLOTS: usize = 4;
/// Begin and end of up to seven passes (depth pre-pass, opaque, cap depth, edges, silhouettes,
/// final, overlays).
const TIMER_QUERIES: u32 = 14;
/// Finished frames kept for [`Renderer::drain_gpu_timings`].
const TIMER_HISTORY: usize = 256;

const TIMER_FREE: u8 = 0;
const TIMER_IN_FLIGHT: u8 = 1;
const TIMER_MAPPED: u8 = 2;
const TIMER_FAILED: u8 = 3;

const PASSES_PLAIN: &[&str] = &["opaque", "final"];
/// The timed passes of a frame, indexed by `prepass | caps << 1 | silhouettes << 2`. With caps
/// the opaque pass ends after the caps, the `cap depth` pass follows, and `edges` holds the
/// external layer, the lines and points, and the MSAA resolve.
const PASS_SETS: [&[&str]; 8] = [
    PASSES_PLAIN,
    &["depth prepass", "opaque", "final"],
    &["opaque", "cap depth", "edges", "final"],
    &["depth prepass", "opaque", "cap depth", "edges", "final"],
    &["opaque", "silhouettes", "final"],
    &["depth prepass", "opaque", "silhouettes", "final"],
    &["opaque", "cap depth", "edges", "silhouettes", "final"],
    &[
        "depth prepass",
        "opaque",
        "cap depth",
        "edges",
        "silhouettes",
        "final",
    ],
];

/// The timed passes of a frame, indexed by `prepass | caps << 1 | silhouettes << 2 |
/// overlays << 3`: [`PASS_SETS`], plus the `overlays` pass (RND-07) after `final`.
fn pass_set(index: usize) -> &'static [&'static str] {
    static SETS: OnceLock<Vec<&'static [&'static str]>> = OnceLock::new();
    SETS.get_or_init(|| {
        (0..16)
            .map(|i| {
                let mut v = PASS_SETS[i & 7].to_vec();
                if i & 8 != 0 {
                    v.push("overlays");
                }
                let leaked: &'static [&'static str] = Box::leak(v.into_boxed_slice());
                leaked
            })
            .collect()
    })[index & 15]
}

/// Silhouette thresholds (design note §6): normal angle above 35°, or a linear-depth jump above
/// 2 % of the nearer depth.
const SILHOUETTE_COS: f32 = 0.819_152;
const SILHOUETTE_DEPTH_JUMP: f32 = 0.02;

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
    /// Draw the triangle wireframe of mesh bodies with `POLYGON_MODE_LINE` when the device has
    /// it. Off, or without the feature, the wireframe styles show their feature edges. Default
    /// on.
    pub polygon_mode_line: bool,
    /// Point-pick radius, physical pixels (RND-04): vertices count within it, edges within it
    /// minus one pixel. Default 5.
    pub pick_radius_px: u32,
    /// How plain meshes report triangle indices to picks. Default [`PickBackend::Auto`].
    pub pick_backend: PickBackend,
}

impl Default for RendererConfig {
    fn default() -> Self {
        RendererConfig {
            msaa_samples: 4,
            depth_prepass: false,
            jobs: None,
            polygon_mode_line: true,
            pick_radius_px: 5,
            pick_backend: PickBackend::Auto,
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
    /// Edge segments drawn (each instance's edge set counted once, whatever the passes).
    pub edge_segments: u64,
    /// Instances that were drawable but outside the view frustum.
    pub culled: u32,
    /// Instances drawn clipped by the section planes (RND-06).
    pub clipped: u32,
    /// Clipped closed instances whose back faces were drawn as caps.
    pub capped: u32,
    /// Instances skipped because a section plane removes them entirely.
    pub sectioned_away: u32,
}

/// GPU time of one finished frame; needs `TIMESTAMP_QUERY`.
#[derive(Clone, Debug, Default)]
pub struct GpuTimings {
    /// From the start of the first pass to the end of the last, milliseconds.
    pub total_ms: f32,
    /// Per-pass times, milliseconds: `depth prepass` (when on, or the depth-only faces of the
    /// hidden-line wireframe styles), `opaque` (background, meshes, external layer, edges and
    /// points, MSAA resolve), `silhouettes` (when on), `final` (copy or FXAA into the sRGB
    /// image). When section caps are drawn, `opaque` ends after the caps and is followed by
    /// `cap depth` and `edges` (external layer, lines and points, resolve).
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
    /// Which faces and edges are drawn. Default [`VisualStyle::ShadedVisibleEdges`].
    pub visual: VisualStyle,
    /// Outline post pass from the normal and depth targets (design note §6). Default off.
    pub silhouettes: bool,
    /// Linear RGB of the silhouette outline. Default: the default edge colour.
    pub silhouette_color: [f32; 3],
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
            visual: VisualStyle::default(),
            silhouettes: false,
            silhouette_color: [0.015, 0.017, 0.02],
        }
    }
}

/// Per-view silhouette parameters (`SilhouetteUniform` in `shaders/silhouette.wgsl`).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, bytemuck::Pod, bytemuck::Zeroable)]
struct SilhouetteUniform {
    depth_coeffs: [f32; 4],
    color: [f32; 4],
    thresholds: [f32; 4],
}

impl SilhouetteUniform {
    fn new(proj: &[[f32; 4]; 4], color: [f32; 3]) -> SilhouetteUniform {
        SilhouetteUniform {
            depth_coeffs: [proj[2][2], proj[3][2], proj[2][3], proj[3][3]],
            color: [color[0], color[1], color[2], 1.0],
            thresholds: [SILHOUETTE_COS, SILHOUETTE_DEPTH_JUMP, 0.0, 0.0],
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
    /// Apply the scene's section planes (RND-06). Default true.
    pub sections_enabled: bool,
    /// Overlay lists drawn in sub-rectangles with their own cameras, over everything (RND-07):
    /// the rendering hook of the view cube (RND-11). Default empty.
    pub insets: Vec<OverlayInset>,
    /// Draw the view cube (RND-11) as one more inset after `insets`, rebuilt every frame from
    /// the camera with [`View::view_cube`]. Default false.
    pub show_view_cube: bool,
    /// Layout, letter size and hover region of the view cube. Set it from the display scale
    /// with [`ViewCube::for_scale`] so it matches `egui_glue::view_cube`. Default: scale 1.
    pub view_cube: ViewCube,
    size_px: [u32; 2],
}

impl View {
    /// Current size of the view, pixels.
    pub fn size_px(&self) -> [u32; 2] {
        self.size_px
    }

    /// The view-cube inset of this frame, if `show_view_cube`.
    pub fn view_cube_inset(&self) -> Option<OverlayInset> {
        self.show_view_cube
            .then(|| self.view_cube.inset(&self.camera, self.size_px))
    }

    /// `insets` plus the view cube (borrowed when there is no cube).
    fn all_insets(&self) -> std::borrow::Cow<'_, [OverlayInset]> {
        match self.view_cube_inset() {
            None => std::borrow::Cow::Borrowed(&self.insets),
            Some(cube) => {
                let mut all = self.insets.clone();
                all.push(cube);
                std::borrow::Cow::Owned(all)
            }
        }
    }
}

/// The depth range of a view of `camera` over `scene` (RND-11 near-plane policy): orthographic
/// views fit it to the visible instances of `layers` and the overlays.
fn view_depth_range(scene: &RenderScene, camera: &Camera, layers: LayerMask) -> DepthRange {
    match camera.projection {
        Projection::Perspective { .. } => camera.default_depth_range(),
        Projection::Orthographic { .. } => scene
            .view_depth_extent(camera.eye, camera.forward().get(), layers)
            .map_or_else(
                || camera.default_depth_range(),
                |(lo, hi)| camera.depth_range_for_extent(lo, hi),
            ),
    }
}

/// Per-view style uniform (`StyleUniform` in `shaders/mesh.wgsl`).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, bytemuck::Pod, bytemuck::Zeroable)]
pub(crate) struct StyleUniform {
    bg_top: [f32; 4],
    bg_bottom: [f32; 4],
    /// Highlight colours of the layers in [`HighlightLayer`] order (RND-04).
    highlight: [[f32; 4]; 4],
}

impl StyleUniform {
    fn new(style: &ViewStyle, highlight: [[f32; 4]; 4]) -> StyleUniform {
        let (top, bottom) = match style.background {
            Background::Solid(c) => (c, c),
            Background::Gradient { top, bottom } => (top, bottom),
        };
        StyleUniform {
            bg_top: [top[0], top[1], top[2], 1.0],
            bg_bottom: [bottom[0], bottom[1], bottom[2], 1.0],
            highlight,
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
    /// `depth` has `TEXTURE_BINDING` (silhouettes or overlays read it).
    depth_sampled: bool,
    /// Own depth of the overlay pass (single-sampled, cleared per frame; RND-07).
    overlay_depth: wgpu::TextureView,
    /// `depth` for the overlay pass (group 2); `Some` when `depth_sampled`.
    overlay_scene_bind: Option<wgpu::BindGroup>,
    /// The normal target for the cap depth pass (group 1).
    cap_marks_bind: wgpu::BindGroup,
    /// Normal + depth + silhouette uniform for the silhouettes pass; `Some` only when the
    /// depth target was made sampleable (`TEXTURE_BINDING` can cost depth compression, so it
    /// is set only while silhouettes are on).
    silhouette_bind: Option<wgpu::BindGroup>,
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

    /// VRAM of the resolve and staging buffers.
    fn gpu_bytes(&self) -> u64 {
        self.resolve.size() + self.slots.iter().map(|s| s.staging.size()).sum::<u64>()
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

impl ViewTargets {
    /// VRAM of the targets.
    fn gpu_bytes(&self) -> u64 {
        [&self.depth, &self.overlay_depth, &self.resolved]
            .into_iter()
            .chain(self.msaa_color.as_ref())
            .map(|v| texture_bytes(v.texture()))
            .sum::<u64>()
            + texture_bytes(&self.normal)
            + texture_bytes(&self.final_texture)
    }
}

#[derive(Debug)]
struct ViewSlot {
    view: View,
    /// What the last successful `encode` drew (diagnostics).
    last_stats: FrameStats,
    targets: Option<ViewTargets>,
    /// Bumped whenever the targets are rebuilt, so `egui_glue` can refresh its bind groups.
    generation: u64,
    camera_buffer: wgpu::Buffer,
    style_buffer: wgpu::Buffer,
    silhouette_buffer: wgpu::Buffer,
    /// The view's section planes (`SectionUniform`), rewritten every frame.
    section_buffer: wgpu::Buffer,
    /// Instance slots of the visible instances in draw order (`u32`): the mesh plan's, then the
    /// line plan's.
    visible_buffer: wgpu::Buffer,
    visible_capacity: u32,
    bind_group: wgpu::BindGroup,
    /// The [`InstanceGpu::generation`] `bind_group` was built for.
    bound_generation: u64,
    timer: Option<GpuTimer>,
    /// Pick targets at the view size, made on the first pick (RND-04).
    pick_targets: Option<PickTargets>,
    /// Instance slots of the pick pass in draw order.
    pick_visible: wgpu::Buffer,
    pick_visible_capacity: u32,
    /// GPU time of pick passes (`["pick"]`).
    pick_timer: Option<GpuTimer>,
    /// This frame's overlay geometry and its buffers (RND-07).
    overlay: OverlayGpu,
}

/// The id, depth and depth-test targets of a view's pick passes (single-sampled).
#[derive(Debug)]
struct PickTargets {
    size: [u32; 2],
    ids: wgpu::Texture,
    ids_view: wgpu::TextureView,
    positions: wgpu::Texture,
    positions_view: wgpu::TextureView,
    depth_test: wgpu::TextureView,
    /// `ids` and `positions` for the pick cap depth pass.
    fix_bind: wgpu::BindGroup,
}

impl ViewSlot {
    /// VRAM of the view's buffers (targets are counted apart).
    fn buffer_bytes(&self) -> u64 {
        let fixed = [
            &self.camera_buffer,
            &self.style_buffer,
            &self.silhouette_buffer,
            &self.section_buffer,
            &self.visible_buffer,
            &self.pick_visible,
        ];
        let o = &self.overlay;
        let overlay = [
            &o.records,
            &o.lines,
            &o.segments,
            &o.segment_ids,
            &o.points,
            &o.point_ids,
            &o.tris,
        ];
        fixed.into_iter().map(wgpu::Buffer::size).sum::<u64>()
            + overlay
                .into_iter()
                .filter_map(|b| b.get())
                .chain(&o.inset_cameras)
                .map(wgpu::Buffer::size)
                .sum::<u64>()
            + [&self.timer, &self.pick_timer]
                .into_iter()
                .flatten()
                .map(GpuTimer::gpu_bytes)
                .sum::<u64>()
    }
}

impl PickTargets {
    /// VRAM of the targets.
    fn gpu_bytes(&self) -> u64 {
        texture_bytes(&self.ids)
            + texture_bytes(&self.positions)
            + texture_bytes(self.depth_test.texture())
    }
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
    /// Group 1 of the line and point pipelines (one storage buffer).
    geometry_bind_layout: wgpu::BindGroupLayout,
    /// Silhouette inputs at one sample (`[0]`) and several (`[1]`).
    silhouette_bind_layouts: [wgpu::BindGroupLayout; 2],
    /// Cap depth input (the normal target) at one sample (`[0]`) and several (`[1]`).
    cap_marks_bind_layouts: [wgpu::BindGroupLayout; 2],
    /// `POLYGON_MODE_LINE` is available and enabled.
    triangle_wire: bool,
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
    line_plan: LinePlan,
    /// Pipeline of each draw of `line_plan`; `None` while it is being built.
    line_pipelines: Vec<Option<PipelineKey>>,
    /// RND-08: group 1 of the analysis pipelines and its parts.
    analysis_bind_layout: wgpu::BindGroupLayout,
    analysis_bind: wgpu::BindGroup,
    /// The [`InstanceGpu::generation`] `analysis_bind` was built for.
    analysis_bind_generation: u64,
    lut: wgpu::Texture,
    lut_view: wgpu::TextureView,
    lut_sampler: wgpu::Sampler,
    /// x = animation time (16-byte uniform).
    analysis_globals: wgpu::Buffer,
    /// Groups 2/3 when an analysis pipeline has no scalar or displacement field.
    dummy_field_bind: wgpu::BindGroup,
    analysis_output: AnalysisOutput,
    animation_time: f32,
    /// The animation time in `analysis_globals`; `None` before the first write.
    written_time: Option<f32>,
    /// Cumulative bytes written to the analysis record buffer.
    analysis_bytes_uploaded: u64,
    /// Group 1 of the vertex-pulling face pick (indices, positions).
    pull_bind_layout: wgpu::BindGroupLayout,
    /// The pick id and depth targets, for the pick cap depth pass.
    pick_targets_bind_layout: wgpu::BindGroupLayout,
    pick_radius: u32,
    pick_backend: PickBackend,
    picks: PickQueue,
    /// `encode` calls so far.
    encodes: u64,
    /// The rectangle-select compute pipeline, built in the background (`None`: it failed).
    rect_pipeline: RectPipeline,
    /// RND-07: group 2 of the overlay pipelines (the scene depth) at one sample and several.
    overlay_depth_layouts: [wgpu::BindGroupLayout; 2],
    /// Group 3 of the overlay pipelines (image + sampler).
    overlay_image_layout: wgpu::BindGroupLayout,
    overlay_sampler: wgpu::Sampler,
    /// Group 3 for untextured overlay draws (a 1x1 white image).
    white_image_bind: wgpu::BindGroup,
    /// VRAM of the resources made once (LUT, globals, dummy field, white image).
    fixed_bytes: u64,
    /// RND-13: the uncaptured-error log, once [`Renderer::capture_errors`] installed it.
    errors: Option<GpuErrorLog>,
}

/// The rectangle-select compute pipeline once built.
type RectPipeline = Arc<OnceLock<Option<Arc<wgpu::ComputePipeline>>>>;

/// Pick requests waiting for an `encode`, in flight, and answered.
#[derive(Debug, Default)]
struct PickQueue {
    next: u64,
    queued: Vec<QueuedPick>,
    in_flight: Vec<InFlight>,
    done: Vec<(PickTicket, Result<PickResult, RenderError>)>,
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
                    wgpu::ShaderStages::VERTEX_FRAGMENT,
                    wgpu::BufferBindingType::Uniform,
                ),
                // Line records (RND-12; the fragment stage reads the section cap colour).
                buffer_entry(4, wgpu::ShaderStages::VERTEX_FRAGMENT, storage),
                // Section planes (RND-06).
                buffer_entry(
                    5,
                    wgpu::ShaderStages::VERTEX_FRAGMENT,
                    wgpu::BufferBindingType::Uniform,
                ),
                // Edge and point highlight states (RND-04).
                buffer_entry(6, wgpu::ShaderStages::VERTEX, storage),
            ],
        });
        let geometry_bind_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("amg line geometry"),
                // Segments or points, and their pick sub-ids (RND-04).
                entries: &[
                    buffer_entry(0, wgpu::ShaderStages::VERTEX, storage),
                    buffer_entry(1, wgpu::ShaderStages::VERTEX, storage),
                ],
            });
        let pull_bind_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("amg pick pull"),
            entries: &[
                buffer_entry(2, wgpu::ShaderStages::VERTEX, storage),
                buffer_entry(3, wgpu::ShaderStages::VERTEX, storage),
            ],
        });
        let pick_targets_bind_layout = pick_targets_bind_layout(device);
        let silhouette_bind_layouts = [
            silhouette_bind_layout(device, false),
            silhouette_bind_layout(device, true),
        ];
        let cap_marks_bind_layouts = [
            cap_marks_bind_layout(device, false),
            cap_marks_bind_layout(device, true),
        ];
        let external_bind_layout = external_bind_layout(device);
        let post_bind_layout = post_bind_layout(device);
        let final_bind_layout = texture_bind_layout(device, "amg final");
        let instance_gpu = InstanceGpu::new(device, INITIAL_INSTANCE_CAPACITY, 0);
        let analysis_layout = analysis_bind_layout(device);
        let overlay_depth_layouts = [false, true].map(|multisampled| {
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("amg overlay scene depth"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Depth,
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled,
                    },
                    count: None,
                }],
            })
        });
        let overlay_image_layout = overlay_image_bind_layout(device);

        let mut core_timings = Vec::new();
        let start = Instant::now();
        let factory = PipelineFactory::new(
            device,
            Layouts {
                view: &view_bind_layout,
                external: &external_bind_layout,
                post: &post_bind_layout,
                geometry: &geometry_bind_layout,
                silhouette: [&silhouette_bind_layouts[0], &silhouette_bind_layouts[1]],
                cap_marks: [&cap_marks_bind_layouts[0], &cap_marks_bind_layouts[1]],
                analysis: &analysis_layout,
                pull: &pull_bind_layout,
                pick_targets: &pick_targets_bind_layout,
                overlay_depth: [&overlay_depth_layouts[0], &overlay_depth_layouts[1]],
                overlay_image: &overlay_image_layout,
            },
        );
        let fullscreen = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("amg fullscreen"),
            source: wgpu::ShaderSource::Wgsl(FULLSCREEN_WGSL.into()),
        });
        core_timings.push(PipelineTiming {
            label: "shader modules",
            variant: "mesh, external layer, post, lines, silhouettes, cap depth, fullscreen".into(),
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
        let lut = create_lut(&gpu);
        let lut_view = lut.create_view(&wgpu::TextureViewDescriptor {
            label: Some("amg colormap LUT"),
            dimension: Some(wgpu::TextureViewDimension::D2Array),
            ..Default::default()
        });
        let lut_sampler = lut_sampler(device);
        let analysis_globals = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("amg analysis globals"),
            size: 16,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let analysis_bind = analysis_bind_group(
            device,
            &analysis_layout,
            &instance_gpu,
            &lut_view,
            &lut_sampler,
            &analysis_globals,
        );
        let dummy_field = storage_buffer(device, "amg no field", 16);
        // Binding 1 (pick sub-ids, RND-04) is unused by the analysis shader; bind the same buffer.
        let dummy_field_bind =
            geometry_bind(device, &geometry_bind_layout, &dummy_field, &dummy_field);
        let overlay_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("amg overlay images"),
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });
        let white = crate::overlay::upload_image(
            &gpu,
            &RgbaImage {
                size: [1, 1],
                pixels: vec![255; 4],
            },
        )?;
        let white_image_bind =
            overlay_image_bind(device, &overlay_image_layout, &white.view, &overlay_sampler);
        let fixed_bytes =
            texture_bytes(&lut) + analysis_globals.size() + dummy_field.size() + white.gpu_bytes();
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
            geometry_bind_layout,
            silhouette_bind_layouts,
            cap_marks_bind_layouts,
            triangle_wire: cfg.polygon_mode_line && gpu.has(wgpu::Features::POLYGON_MODE_LINE),
            core_timings,
            scene: RenderScene::default(),
            views: SlotMap::with_key(),
            instance_gpu,
            instance_bytes_uploaded: 0,
            runs: Vec::new(),
            plan: DrawPlan::default(),
            group_pipelines: Vec::new(),
            line_plan: LinePlan::default(),
            line_pipelines: Vec::new(),
            analysis_bind_layout: analysis_layout,
            analysis_bind,
            analysis_bind_generation: 0,
            lut,
            lut_view,
            lut_sampler,
            analysis_globals,
            dummy_field_bind,
            analysis_output: AnalysisOutput::Lit,
            animation_time: 0.0,
            written_time: None,
            analysis_bytes_uploaded: 0,
            pull_bind_layout,
            pick_targets_bind_layout,
            pick_radius: cfg.pick_radius_px,
            pick_backend: cfg.pick_backend,
            picks: PickQueue::default(),
            encodes: 0,
            rect_pipeline: spawn_rect_pipeline(&gpu),
            overlay_depth_layouts,
            overlay_image_layout,
            overlay_sampler,
            white_image_bind,
            fixed_bytes,
            errors: None,
            gpu,
        })
    }

    /// Whether mesh bodies draw their triangle wireframe through `POLYGON_MODE_LINE` in the
    /// wireframe styles (the device has the feature and [`RendererConfig::polygon_mode_line`]
    /// is on). Otherwise they draw their edge set (feature edges).
    pub fn triangle_wireframe(&self) -> bool {
        self.triangle_wire
    }

    /// Sets the animation time τ (seconds) of animated deformations ([`crate::Animation`]).
    /// One 16-byte uniform write on the next `encode`; the app advances it every frame while
    /// [`RenderScene::is_animating`].
    pub fn set_animation_time(&mut self, seconds: f32) {
        self.animation_time = seconds;
    }

    /// The animation time τ, seconds.
    pub fn animation_time(&self) -> f32 {
        self.animation_time
    }

    /// Test mode of the analysis and field pipelines: unlit colours or per-fragment classes
    /// ([`AnalysisOutput`]) instead of lit colours. Takes effect on the next `encode` (the
    /// variants are built lazily like any other).
    pub fn set_analysis_output(&mut self, out: AnalysisOutput) {
        self.analysis_output = out;
    }

    /// The analysis output mode.
    pub fn analysis_output(&self) -> AnalysisOutput {
        self.analysis_output
    }

    /// Total bytes written to the analysis record buffer (128 per changed instance).
    pub fn analysis_bytes_uploaded(&self) -> u64 {
        self.analysis_bytes_uploaded
    }

    /// The colormap LUT: `Rgba8UnormSrgb`, 256 × 1, one layer per [`crate::ColorMap`] in
    /// [`crate::ColorMap::ALL`] order, with `COPY_SRC` (tests compare it to
    /// [`crate::colormap_lut`]).
    pub fn colormap_lut_texture(&self) -> &wgpu::Texture {
        &self.lut
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
        let Some((style, sections_enabled)) = self
            .views
            .get(id)
            .map(|s| (s.view.style, s.view.sections_enabled))
        else {
            return;
        };
        // Clipped variants only when the view applies section planes.
        let sectioned = sections_enabled && !self.scene.active_sections().is_empty();
        let clip_modes: &[bool] = if sectioned { &[false, true] } else { &[false] };
        let configs = [
            (self.active_samples, self.active_prepass),
            (self.requested_samples, self.requested_prepass),
        ];
        // Analysis programmes the scene's instances use, per geometry kind.
        let mut shades = Vec::new();
        for inst in self.scene.table().instances() {
            let shade = self.scene.group_shade(inst, self.analysis_output);
            let Some(mesh) = inst.mesh.and_then(|k| self.scene.mesh(k)) else {
                continue;
            };
            if shade.key.is_material() {
                continue;
            }
            let kind = (
                shade.key,
                style.flat_shading || !mesh.has_normals(),
                !mesh.is_closed(),
            );
            if !shades.contains(&kind) {
                shades.push(kind);
            }
        }
        for (samples, prepass) in configs {
            for clip in clip_modes.iter().copied() {
                for &(shade, flat, double_sided) in &shades {
                    self.variants.request(
                        PipelineKey::mesh(style.lighting, flat, double_sided, false, samples)
                            .with_clip(clip)
                            .with_shade(shade),
                    );
                }
                for flat in [false, true] {
                    for double_sided in [false, true] {
                        // Clipped groups skip the pre-pass and test depth themselves.
                        let equal = prepass && !clip;
                        self.variants.request(
                            PipelineKey::mesh(style.lighting, flat, double_sided, equal, samples)
                                .with_clip(clip),
                        );
                        if clip {
                            // The fallback while another lighting model is being built.
                            self.variants.request(
                                PipelineKey::mesh(
                                    Lighting::Studio,
                                    flat,
                                    double_sided,
                                    false,
                                    samples,
                                )
                                .with_clip(true),
                            );
                        }
                        if style.visual.depth_only_faces() {
                            self.variants.request(PipelineKey {
                                kind: PipelineKind::Depth {
                                    flat,
                                    double_sided,
                                    clipped: clip,
                                },
                                samples,
                            });
                        }
                    }
                }
                for pass in [LinePass::Hidden, LinePass::Visible, LinePass::Always] {
                    self.variants.request(PipelineKey {
                        kind: PipelineKind::Line {
                            pass,
                            clipped: clip,
                        },
                        samples,
                    });
                    self.variants.request(PipelineKey {
                        kind: PipelineKind::Point {
                            pass,
                            clipped: clip,
                        },
                        samples,
                    });
                    if self.triangle_wire && style.visual.is_wireframe() {
                        self.variants.request(PipelineKey {
                            kind: PipelineKind::Wire {
                                pass,
                                clipped: clip,
                            },
                            samples,
                        });
                    }
                }
            }
            if sectioned {
                self.variants.request(PipelineKey {
                    kind: PipelineKind::CapDepth,
                    samples,
                });
                self.variants.request(PipelineKey {
                    kind: PipelineKind::CapOccluder,
                    samples,
                });
            }
            if style.silhouettes {
                self.variants.request(PipelineKey {
                    kind: PipelineKind::Silhouette,
                    samples,
                });
            }
        }
        if style.fxaa {
            self.variants.request(fxaa_key());
        }
        // Face-range highlight redraws use the depth-`Equal` mesh pipelines.
        if self.scene.has_sub_highlights() {
            for samples in [self.active_samples, self.requested_samples] {
                for flat in [false, true] {
                    for double_sided in [false, true] {
                        self.variants.request(PipelineKey::mesh(
                            style.lighting,
                            flat,
                            double_sided,
                            true,
                            samples,
                        ));
                    }
                }
            }
        }
        // RND-07 overlays (cheap; requested for every view so the first tool frame draws them).
        for samples in [self.active_samples, self.requested_samples] {
            for key in overlay_keys(samples) {
                self.variants.request(key);
            }
        }
        self.request_pick_variants(sectioned);
    }

    /// World-anchored labels of the scene's overlay list and of the view's insets, projected
    /// into view `id` for egui painting (`egui_glue::paint_labels`). Anchors behind the eye are
    /// dropped. Empty for an unknown view.
    pub fn project_labels(&self, id: ViewId) -> Vec<ProjectedLabel> {
        let Some(slot) = self.views.get(id) else {
            return Vec::new();
        };
        let mut out = Vec::new();
        self.scene.overlays().project_labels(
            &slot.view.camera,
            slot.view.size_px,
            [0.0, 0.0],
            &mut out,
        );
        for inset in slot.view.all_insets().iter() {
            let [x, y, w, h] = inset.rect_px;
            inset.overlays.project_labels(
                &inset.camera,
                [w.max(1), h.max(1)],
                [f64::from(x), f64::from(y)],
                &mut out,
            );
        }
        out
    }

    /// Blocks until no pipeline variant is pending (or `timeout` passes) and applies a pending
    /// sample-count or pre-pass switch. Returns true when nothing is pending. Job, test and
    /// headless threads only; the UI thread lets frames fall back instead.
    pub fn wait_for_variants(&mut self, timeout: Duration) -> bool {
        let start = Instant::now();
        let mut done = self.variants.wait(timeout);
        // The rectangle-pick compute pipeline is built on its own thread.
        while self.rect_pipeline.get().is_none() {
            if start.elapsed() >= timeout {
                done = false;
                break;
            }
            std::thread::sleep(Duration::from_millis(1));
        }
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

    /// Sends the device's uncaptured wgpu errors to `tracing`, each distinct one once, instead
    /// of wgpu's default handler, which panics ([`GpuErrorLog`]). `egui_glue::install` calls it;
    /// headless users opt in, so tests keep failing on invalid GPU use. Installs once; later
    /// calls return the same log.
    pub fn capture_errors(&mut self) -> GpuErrorLog {
        self.errors
            .get_or_insert_with(|| GpuErrorLog::install(self.gpu.device()))
            .clone()
    }

    /// The log installed by [`Renderer::capture_errors`].
    pub fn error_log(&self) -> Option<&GpuErrorLog> {
        self.errors.as_ref()
    }

    /// VRAM held by the scene and the renderer, by category.
    pub fn vram_usage(&self) -> VramUsage {
        let instances = &self.instance_gpu;
        VramUsage {
            scene: self.scene.vram(),
            view_targets: self
                .views
                .iter()
                .filter_map(|(_, s)| s.targets.as_ref())
                .map(ViewTargets::gpu_bytes)
                .sum(),
            pick_targets: self
                .views
                .iter()
                .filter_map(|(_, s)| s.pick_targets.as_ref())
                .map(PickTargets::gpu_bytes)
                .sum(),
            instance_buffers: instances.buffer.size()
                + instances.lines.size()
                + instances.analysis.size()
                + instances.states.size(),
            view_buffers: self.views.iter().map(|(_, s)| s.buffer_bytes()).sum(),
            other: self.fixed_bytes,
        }
    }

    /// The data of the diagnostics overlay for view `id`: adapter, what the last frame drew, GPU
    /// pass timings, VRAM by category and the error count. `None` for an unknown view.
    pub fn diagnostics(&self, id: ViewId) -> Option<Diagnostics> {
        let slot = self.views.get(id)?;
        let info = self.gpu.info();
        Some(Diagnostics {
            adapter: info.name.clone(),
            backend: format!("{:?}", info.backend),
            frame: slot.last_stats,
            timings: self.gpu_timings(id),
            vram: self.vram_usage(),
            gpu_errors: self.errors.as_ref().map_or(0, GpuErrorLog::total),
        })
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
        let silhouette_buffer = uniform("amg silhouettes", size_of::<SilhouetteUniform>() as u64);
        let section_buffer = uniform("amg sections", SECTION_UNIFORM_BYTES);
        let visible_buffer = storage_buffer(
            device,
            "amg visible instances",
            u64::from(INITIAL_VISIBLE_CAPACITY) * 4,
        );
        let bind_group = view_bind_group(
            device,
            &self.view_bind_layout,
            &camera_buffer,
            &self.instance_gpu,
            &visible_buffer,
            &style_buffer,
            &section_buffer,
        );
        self.views.insert(ViewSlot {
            view: View {
                camera: default_camera(),
                style: ViewStyle::default(),
                layers: LayerMask::ALL,
                sections_enabled: true,
                insets: Vec::new(),
                show_view_cube: false,
                view_cube: ViewCube::default(),
                size_px,
            },
            last_stats: FrameStats::default(),
            targets: None,
            generation: 0,
            camera_buffer,
            style_buffer,
            silhouette_buffer,
            section_buffer,
            visible_buffer,
            visible_capacity: INITIAL_VISIBLE_CAPACITY,
            bind_group,
            bound_generation: self.instance_gpu.generation,
            timer: GpuTimer::new(&self.gpu),
            pick_targets: None,
            pick_visible: storage_buffer(
                device,
                "amg pick visible",
                u64::from(INITIAL_VISIBLE_CAPACITY) * 4,
            ),
            pick_visible_capacity: INITIAL_VISIBLE_CAPACITY,
            pick_timer: GpuTimer::new(&self.gpu),
            overlay: OverlayGpu::default(),
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
        let line_records = self.scene.table().line_records();
        let analysis_records = self.scene.table().analysis_records();
        let len = records.len() as u32;
        let device = self.gpu.device();
        let queue = self.gpu.queue();
        if len > self.instance_gpu.capacity {
            let capacity = len.next_power_of_two().max(INITIAL_INSTANCE_CAPACITY);
            self.instance_gpu =
                InstanceGpu::new(device, capacity, self.instance_gpu.generation + 1);
            queue.write_buffer(&self.instance_gpu.buffer, 0, bytemuck::cast_slice(records));
            queue.write_buffer(
                &self.instance_gpu.lines,
                0,
                bytemuck::cast_slice(line_records),
            );
            queue.write_buffer(
                &self.instance_gpu.analysis,
                0,
                bytemuck::cast_slice(analysis_records),
            );
            self.instance_bytes_uploaded += u64::from(len) * RECORD_BYTES;
            self.analysis_bytes_uploaded += u64::from(len) * ANALYSIS_RECORD_BYTES;
        } else {
            for run in &self.runs {
                let range = run.start as usize..run.end as usize;
                let part = &records[range.clone()];
                queue.write_buffer(
                    &self.instance_gpu.buffer,
                    u64::from(run.start) * RECORD_BYTES,
                    bytemuck::cast_slice(part),
                );
                // The line records are a separate buffer and not counted in
                // `instance_bytes_uploaded` (that counter is the 128-byte records only).
                queue.write_buffer(
                    &self.instance_gpu.lines,
                    u64::from(run.start) * LINE_RECORD_BYTES,
                    bytemuck::cast_slice(&line_records[range.clone()]),
                );
                queue.write_buffer(
                    &self.instance_gpu.analysis,
                    u64::from(run.start) * ANALYSIS_RECORD_BYTES,
                    bytemuck::cast_slice(&analysis_records[range]),
                );
                self.instance_bytes_uploaded += part.len() as u64 * RECORD_BYTES;
                self.analysis_bytes_uploaded += part.len() as u64 * ANALYSIS_RECORD_BYTES;
            }
        }
        let (states, revision) = self.scene.highlight_states();
        if revision != self.instance_gpu.states_revision {
            let words = states.len() as u64;
            if words > self.instance_gpu.states_capacity {
                let capacity = words.next_power_of_two().max(INITIAL_STATE_WORDS);
                self.instance_gpu.states =
                    storage_buffer(device, "amg highlight states", 4 * capacity);
                self.instance_gpu.states_capacity = capacity;
                // Views rebuild their bind groups.
                self.instance_gpu.generation += 1;
            }
            if !states.is_empty() {
                queue.write_buffer(&self.instance_gpu.states, 0, bytemuck::cast_slice(states));
            }
            self.instance_gpu.states_revision = revision;
        }
        if self.analysis_bind_generation != self.instance_gpu.generation {
            self.analysis_bind = analysis_bind_group(
                device,
                &self.analysis_bind_layout,
                &self.instance_gpu,
                &self.lut_view,
                &self.lut_sampler,
                &self.analysis_globals,
            );
            self.analysis_bind_generation = self.instance_gpu.generation;
        }
        if self.written_time != Some(self.animation_time) {
            queue.write_buffer(
                &self.analysis_globals,
                0,
                bytemuck::cast_slice(&[self.animation_time, 0.0, 0.0, 0.0]),
            );
            self.written_time = Some(self.animation_time);
        }
    }

    /// Total bytes written to the instance storage buffer so far. Changing one instance adds
    /// exactly 128 on the next `encode`; an unchanged scene adds nothing.
    pub fn instance_bytes_uploaded(&self) -> u64 {
        self.instance_bytes_uploaded
    }
    /// Test readback (RND-08 Done-when 6): the mesh-local positions of instance `k` after its
    /// deformation, computed on the GPU by the WGSL `deformed_position` the vertex shader uses,
    /// from the renderer's own buffers (mesh positions, displacement field, the instance's
    /// analysis record, the animation time). Flushes pending instance changes first. Blocking;
    /// test and job threads only.
    ///
    /// # Errors
    /// [`RenderError::UnknownKey`] for a removed instance or mesh; [`RenderError::InvalidData`]
    /// when the instance has no live deformation; [`RenderError::Gpu`] on device loss.
    pub fn deformed_positions(&mut self, k: InstanceKey) -> Result<Vec<[f32; 3]>, RenderError> {
        use crate::analysis::testing::{Params, run, storage_init, uniform_init};

        self.sync_instances();
        let slot = self
            .scene
            .table()
            .slot(k)
            .ok_or(RenderError::UnknownKey("instance"))?;
        let inst = self
            .scene
            .instance(k)
            .ok_or(RenderError::UnknownKey("instance"))?;
        let mesh = inst
            .mesh
            .and_then(|m| self.scene.mesh(m))
            .ok_or(RenderError::UnknownKey("mesh"))?;
        let shade = self.scene.group_shade(inst, AnalysisOutput::Lit);
        let field = shade.fields[1]
            .and_then(|f| self.scene.field(f))
            .ok_or_else(|| RenderError::InvalidData("the instance has no deformation".into()))?;
        let n = mesh.vertex_count();
        if n == 0 {
            return Ok(Vec::new());
        }
        let gpu = &self.gpu;
        let bytes = u64::from(n) * 12;
        let positions = storage_init(gpu, "amg deform positions", &[]);
        let positions = if positions.size() >= bytes {
            positions
        } else {
            storage_init(gpu, "amg deform positions", &vec![0u8; bytes as usize])
        };
        let mut encoder = gpu
            .device()
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("amg deform readback"),
            });
        encoder.copy_buffer_to_buffer(&mesh.positions, 0, &positions, 0, bytes);
        gpu.queue().submit([encoder.finish()]);
        let out = storage_init(gpu, "amg deformed", &vec![0u8; bytes as usize]);
        let params = uniform_init(
            gpu,
            Params {
                count: n,
                slot,
                layer: 0,
                pad: 0,
            },
        );
        let raw = run(
            gpu,
            "DEFORM",
            "deform_kernel",
            &[
                positions.as_entire_binding(),
                field.buffer.as_entire_binding(),
                self.instance_gpu.analysis.as_entire_binding(),
                self.analysis_globals.as_entire_binding(),
                out.as_entire_binding(),
                params.as_entire_binding(),
            ],
            n,
            &out,
            bytes,
        )?;
        Ok(bytemuck::pod_collect_to_vec::<u8, [f32; 3]>(&raw))
    }

    /// Records all passes for `id` into `encoder`. Never blocks. A view with a zero dimension
    /// records nothing.
    ///
    /// Passes: an optional depth pre-pass (also the depth-only faces of the hidden-line
    /// wireframe styles); the opaque pass (gradient background, meshes with the view's lighting
    /// into colour + octahedral normal, the external layer, then hidden, visible and
    /// always-on-top edges, points and triangle wireframes per [`ViewStyle::visual`], MSAA
    /// resolve); the optional silhouettes pass over the resolved image; the final pass (copy or
    /// FXAA into the sRGB image). A mesh variant that is not built yet is requested and the draw
    /// falls back to the core Studio pipeline; line, point, silhouette and depth-only variants
    /// are requested and skipped until they are ready.
    ///
    /// Several views may be encoded into one encoder (one submit, one frame): the scene's
    /// instance records are shared, each view has its own camera, visibility list and targets.
    ///
    /// # Errors
    /// [`RenderError::UnknownKey`] for a removed view, [`RenderError::Gpu`] on device loss
    /// ([`GpuError::DeviceLost`], on this and every later call; the device is not re-created).
    /// wgpu reports a `destroy()`ed device lost only once its submitted work has finished, so a
    /// call in that window may still succeed.
    ///
    /// Pick requests of the view ([`Renderer::request_pick`]) are recorded after the frame
    /// (RND-04); their results arrive through [`Renderer::poll_picks`] once the encoder is
    /// submitted and the GPU finished it.
    pub fn encode(
        &mut self,
        id: ViewId,
        encoder: &mut wgpu::CommandEncoder,
    ) -> Result<FrameStats, RenderError> {
        self.encodes += 1;
        let stats = self.encode_view(id, encoder)?;
        if let Some(slot) = self.views.get_mut(id) {
            slot.last_stats = stats;
        }
        self.record_picks(id, encoder);
        Ok(stats)
    }

    fn encode_view(
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
        let triangle_wire = self.triangle_wire;
        let gpu = Arc::clone(&self.gpu);
        let device = gpu.device();
        let queue = gpu.queue();
        let scene = &self.scene;
        let plan = &mut self.plan;
        let line_plan = &mut self.line_plan;
        let variants = &mut self.variants;
        let group_pipelines = &mut self.group_pipelines;
        let line_pipelines = &mut self.line_pipelines;
        let geometry_layout = &self.geometry_bind_layout;
        let analysis_bind = &self.analysis_bind;
        let dummy_field_bind = &self.dummy_field_bind;
        let analysis_output = self.analysis_output;
        let slot = self
            .views
            .get_mut(id)
            .ok_or(RenderError::UnknownKey("view"))?;
        let size = slot.view.size_px;

        let mut stats = FrameStats::default();
        if size[0] == 0 || size[1] == 0 {
            return Ok(stats);
        }
        // RND-11: the view cube is one more inset, rebuilt from the camera every frame.
        let insets = slot.view.all_insets();
        // RND-07: once a view draws overlays its depth target stays sampleable (the overlay
        // pass reads it for the depth modes), so a tool starting or stopping never rebuilds it.
        let has_overlays =
            !scene.overlays().is_empty() || insets.iter().any(|i| !i.overlays.is_empty());
        slot.overlay.sampled_depth |= has_overlays;
        let depth_sampled = slot.view.style.silhouettes || slot.overlay.sampled_depth;
        if slot.targets.as_ref().is_none_or(|t| {
            t.size != size
                || t.samples != samples
                || t.silhouette_bind.is_some() != slot.view.style.silhouettes
                || t.depth_sampled != depth_sampled
        }) {
            slot.targets = Some(create_targets(
                device,
                TargetLayouts {
                    final_bind: &self.final_bind_layout,
                    post_bind: &self.post_bind_layout,
                    post_sampler: &self.post_sampler,
                    silhouette_bind: &self.silhouette_bind_layouts[usize::from(samples > 1)],
                    silhouette_buffer: &slot.silhouette_buffer,
                    cap_marks_bind: &self.cap_marks_bind_layouts[usize::from(samples > 1)],
                    overlay_depth: &self.overlay_depth_layouts[usize::from(samples > 1)],
                },
                size,
                samples,
                depth_sampled,
                slot.view.style.silhouettes,
            ));
            slot.generation += 1;
        }

        let camera = slot.view.camera;
        let style = slot.view.style;
        let depth_range = view_depth_range(scene, &camera, slot.view.layers);
        let camera_uniform = camera.uniform_in(size, depth_range);
        queue.write_buffer(&slot.camera_buffer, 0, bytemuck::bytes_of(&camera_uniform));
        queue.write_buffer(
            &slot.style_buffer,
            0,
            bytemuck::bytes_of(&StyleUniform::new(
                &style,
                HighlightLayer::ALL.map(|l| scene.highlight_color(l)),
            )),
        );

        let sections = if slot.view.sections_enabled {
            scene.active_sections()
        } else {
            Vec::new()
        };
        let sectioned = !sections.is_empty();
        queue.write_buffer(
            &slot.section_buffer,
            0,
            bytemuck::bytes_of(&SectionUniform::new(&sections, &camera, size)),
        );

        // RND-07: tessellate this view's overlays (constant pixel size needs its camera).
        slot.overlay.geometry.clear();
        if has_overlays {
            let image_ok = |k| scene.image(k).is_some();
            let mut tess = Tessellator {
                geometry: &mut slot.overlay.geometry,
                highlights: HandleHighlights {
                    layers: HighlightLayer::ALL.map(|l| scene.highlight_layer(l)),
                    colors: HighlightLayer::ALL.map(|l| scene.highlight_color(l)),
                },
                image_ok: &image_ok,
            };
            tess.layer(scene.overlays(), &camera, size, false);
            for inset in insets.iter() {
                let [_, _, w, h] = inset.rect_px;
                tess.layer(&inset.overlays, &inset.camera, [w.max(1), h.max(1)], true);
            }
        }

        let frustum = Frustum::from_view_proj(
            &(camera.proj_matrix_in(size, depth_range) * camera.view_matrix()),
        );
        plan.build_sectioned(
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
            |inst| scene.group_shade(inst, analysis_output),
            &frustum,
            camera.eye,
            slot.view.layers,
            style.flat_shading,
            &sections,
        );
        stats.culled = plan.culled;
        stats.sectioned_away = plan.sectioned_away;
        let visual = style.visual;
        let shade = visual.shades_faces();

        // Choose each group's pipeline; request variants that are not built yet.
        group_pipelines.clear();
        // Whether each group is drawn into the depth pre-pass. Clipped groups never are: their
        // back faces are section caps, so they test and write depth themselves (a clipped
        // depth pre-pass measured slower: its discards defeat early depth testing too).
        let mut group_prepass = Vec::with_capacity(plan.groups.len());
        if shade {
            for g in &plan.groups {
                // Analysis groups skip the pre-pass too (deformation moves their depth).
                let equal = prepass && !g.clipped && g.shade.key.is_material();
                let want =
                    PipelineKey::mesh(style.lighting, g.flat, g.double_sided, equal, samples)
                        .with_clip(g.clipped)
                        .with_shade(g.shade.key);
                let core =
                    PipelineKey::mesh(Lighting::Studio, g.flat, g.double_sided, equal, samples);
                let chosen = if variants.has(want) {
                    want
                } else {
                    variants.request(want);
                    // A clipped Studio variant before the unclipped core one.
                    let studio = core.with_clip(g.clipped);
                    if variants.has(studio) {
                        studio
                    } else {
                        variants.request(studio);
                        core
                    }
                };
                group_pipelines.push(chosen);
                group_prepass.push(equal);
            }
        }
        // Section caps: the back faces of the clipped closed groups, in the shaded styles, drawn
        // by their clipped mesh pipeline. Until it and the cap depth pipeline are built, cut
        // closed bodies show their front faces only.
        let cap_depth_key = PipelineKey {
            kind: PipelineKind::CapDepth,
            samples,
        };
        // A single plane's kept region is unbounded along every ray that enters it, so the
        // occluders past its exit are needed only with two planes or more.
        let occluder_key = PipelineKey {
            kind: PipelineKind::CapOccluder,
            samples,
        };
        let bounded = sections.len() >= 2;
        let capped_group = |i: usize| {
            let g = &plan.groups[i];
            g.clipped
                && !g.double_sided
                && group_pipelines
                    .get(i)
                    .is_some_and(|k| matches!(k.kind, PipelineKind::Mesh { clipped: true, .. }))
        };
        let cap_groups = shade && (0..plan.groups.len()).any(capped_group);
        let caps = cap_groups && {
            let mut ready = true;
            let keys = [cap_depth_key, occluder_key];
            for &key in &keys[..if bounded { 2 } else { 1 }] {
                if !variants.has(key) {
                    variants.request(key);
                    ready = false;
                }
            }
            ready
        };
        for (i, g) in plan.groups.iter().enumerate() {
            if g.clipped {
                stats.clipped += g.count;
                if cap_groups && capped_group(i) {
                    stats.capped += g.count;
                }
            }
        }
        // The hidden-line wireframe styles draw the faces into depth only. Until those
        // pipelines are built, the style draws like `Wireframe`.
        let wire_depth = visual.depth_only_faces() && !plan.groups.is_empty() && {
            let mut ready = true;
            for g in &plan.groups {
                let key = PipelineKey {
                    kind: PipelineKind::Depth {
                        flat: g.flat,
                        double_sided: g.double_sided,
                        clipped: g.clipped,
                    },
                    samples,
                };
                if !variants.has(key) {
                    variants.request(key);
                    ready = false;
                }
            }
            ready
        };
        let depth_pass = (prepass && shade) || wire_depth;

        line_plan.build(
            scene.table(),
            &frustum,
            LineView {
                style: visual,
                layers: slot.view.layers,
                triangle_wire,
                has_depth: shade || wire_depth,
            },
            |k| scene.mesh(k).is_some_and(|m| m.index_count() > 0),
            |k| scene.edge_set(k).map(GpuEdges::segment_count),
            |k| scene.point_set(k).is_some_and(|p| p.point_count() > 0),
        );
        stats.edge_segments = line_plan.segments;
        line_pipelines.clear();
        for draw in &line_plan.draws {
            let key = line_key(draw, samples).with_clip(sectioned);
            line_pipelines.push(if variants.has(key) {
                Some(key)
            } else {
                variants.request(key);
                None
            });
        }
        let silhouette_key = PipelineKey {
            kind: PipelineKind::Silhouette,
            samples,
        };
        let silhouettes = style.silhouettes && {
            let ready = variants.has(silhouette_key);
            if !ready {
                variants.request(silhouette_key);
            }
            ready
        };
        if silhouettes {
            queue.write_buffer(
                &slot.silhouette_buffer,
                0,
                bytemuck::bytes_of(&SilhouetteUniform::new(
                    &camera_uniform.proj,
                    style.silhouette_color,
                )),
            );
        }
        let use_fxaa = style.fxaa && {
            let ready = variants.has(fxaa_key());
            if !ready {
                variants.request(fxaa_key());
            }
            ready
        };
        // Face-range highlights (RND-04): the highlighted triangles of drawn instances again, with
        // the same vertex stage and depth `Equal`, so they cover exactly the pixels the faces drew.
        let mut hl_entries: Vec<u32> = Vec::new();
        let mut hl_draws: Vec<HighlightDraw> = Vec::new();
        let face_highlights = scene.face_highlights();
        if shade && !face_highlights.is_empty() {
            let mut group_of: HashMap<u32, usize> = HashMap::new();
            for (i, g) in plan.groups.iter().enumerate() {
                for &s in &plan.slots[g.first as usize..(g.first + g.count) as usize] {
                    group_of.insert(s, i);
                }
            }
            for fh in face_highlights {
                let Some(&i) = group_of.get(&fh.slot) else {
                    continue;
                };
                let g = &plan.groups[i];
                let key = PipelineKey::mesh(style.lighting, g.flat, g.double_sided, true, samples);
                if !variants.has(key) {
                    variants.request(key);
                    continue;
                }
                hl_draws.push(HighlightDraw {
                    key,
                    mesh: g.mesh,
                    flat: g.flat,
                    tris: fh.tris.clone(),
                    entry: hl_entries.len() as u32,
                });
                hl_entries.push(fh.slot | (fh.bits << SLOT_BITS));
            }
        }
        let draw_overlays = !slot.overlay.geometry.is_empty() && {
            let mut ready = true;
            for key in overlay_keys(samples) {
                if !variants.has(key) {
                    variants.request(key);
                    ready = false;
                }
            }
            ready
        };
        let variants = &*variants;

        let mut rebind = slot.bound_generation != self.instance_gpu.generation;
        let line_base = plan.slots.len() as u32;
        let hl_base = line_base + line_plan.slots.len() as u32;
        let needed = hl_base + hl_entries.len() as u32;
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
                &self.instance_gpu,
                &slot.visible_buffer,
                &slot.style_buffer,
                &slot.section_buffer,
            );
            slot.bound_generation = self.instance_gpu.generation;
        }
        if !plan.slots.is_empty() {
            queue.write_buffer(&slot.visible_buffer, 0, bytemuck::cast_slice(&plan.slots));
        }
        if !line_plan.slots.is_empty() {
            queue.write_buffer(
                &slot.visible_buffer,
                u64::from(line_base) * 4,
                bytemuck::cast_slice(&line_plan.slots),
            );
        }
        if !hl_entries.is_empty() {
            queue.write_buffer(
                &slot.visible_buffer,
                u64::from(hl_base) * 4,
                bytemuck::cast_slice(&hl_entries),
            );
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

        let passes = pass_set(
            usize::from(depth_pass)
                | usize::from(caps) << 1
                | usize::from(silhouettes) << 2
                | usize::from(draw_overlays) << 3,
        );
        let timer_slot = slot.timer.as_mut().and_then(|t| {
            t.harvest();
            t.begin(passes)
        });
        let timer = timer_slot.and(slot.timer.as_ref());
        let pass_index = |name: &str| passes.iter().position(|&p| p == name).unwrap_or(0);

        let Some(targets) = slot.targets.as_ref() else {
            return Ok(stats);
        };
        // `depth_only`: the depth pre-pass (the groups in `group_prepass`), or the depth-only
        // faces pass of a wireframe style (`wire_depth`, every group).
        let draw_meshes = |pass: &mut wgpu::RenderPass<'_>, depth_only: bool| {
            let mut drawn = (0u32, 0u64);
            let mut bound = None;
            for (i, group) in plan.groups.iter().enumerate() {
                if depth_only && !wire_depth && !group_prepass.get(i).copied().unwrap_or(false) {
                    continue;
                }
                let Some(mesh) = scene.mesh(group.mesh) else {
                    continue;
                };
                let key = if depth_only {
                    PipelineKey {
                        kind: PipelineKind::Depth {
                            flat: group.flat,
                            double_sided: group.double_sided,
                            clipped: group.clipped,
                        },
                        samples,
                    }
                } else if let Some(&key) = group_pipelines.get(i) {
                    key
                } else {
                    continue;
                };
                if bound != Some(key) {
                    let Some(pipeline) = variants.get(key) else {
                        continue;
                    };
                    pass.set_pipeline(pipeline);
                    bound = Some(key);
                }
                if key.is_analysis() {
                    pass.set_bind_group(1, analysis_bind, &[]);
                    for (group, field) in [(2, group.shade.fields[0]), (3, group.shade.fields[1])] {
                        let bind =
                            field
                                .and_then(|k| scene.field(k))
                                .map_or(dummy_field_bind, |f| {
                                    f.bind.get_or_init(|| {
                                        geometry_bind(device, geometry_layout, &f.buffer, &f.buffer)
                                    })
                                });
                        pass.set_bind_group(group, bind, &[]);
                    }
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
        let draw_lines = |pass: &mut wgpu::RenderPass<'_>| {
            let mut bound = None;
            for (draw, key) in line_plan.draws.iter().zip(line_pipelines.iter()) {
                let Some(key) = *key else {
                    continue;
                };
                let instances = line_base + draw.first..line_base + draw.first + draw.count;
                let pipeline = || variants.get(key);
                match draw.geometry {
                    LineGeometry::Wire(m) => {
                        let (Some(mesh), Some(p)) = (scene.mesh(m), pipeline()) else {
                            continue;
                        };
                        if bound != Some(key) {
                            pass.set_pipeline(p);
                            bound = Some(key);
                        }
                        pass.set_vertex_buffer(0, mesh.positions.slice(..));
                        pass.set_index_buffer(mesh.indices.slice(..), wgpu::IndexFormat::Uint32);
                        pass.draw_indexed(0..mesh.index_count(), 0, instances);
                    }
                    LineGeometry::Edges(k) => {
                        let (Some(edges), Some(p)) = (scene.edge_set(k), pipeline()) else {
                            continue;
                        };
                        if bound != Some(key) {
                            pass.set_pipeline(p);
                            bound = Some(key);
                        }
                        let bind = edges.bind.get_or_init(|| {
                            geometry_bind(device, geometry_layout, &edges.segments, &edges.ids)
                        });
                        pass.set_bind_group(1, bind, &[]);
                        pass.draw(0..edges.segment_count() * 6, instances);
                    }
                    LineGeometry::Points(k) => {
                        let (Some(points), Some(p)) = (scene.point_set(k), pipeline()) else {
                            continue;
                        };
                        if bound != Some(key) {
                            pass.set_pipeline(p);
                            bound = Some(key);
                        }
                        let bind = points.bind.get_or_init(|| {
                            geometry_bind(device, geometry_layout, &points.points, &points.ids)
                        });
                        pass.set_bind_group(1, bind, &[]);
                        pass.draw(0..points.point_count() * 6, instances);
                    }
                }
            }
        };

        // Back faces of the cut closed instances (positions only), with the cap pipeline.
        // With a bounded kept region, the front faces of the cut closed groups past its exit,
        // depth only, before the meshes (so a cap candidate behind them never writes colour).
        let draw_occluders = |pass: &mut wgpu::RenderPass<'_>| {
            if !bounded {
                return;
            }
            if let Some(pipeline) = variants.get(occluder_key) {
                pass.set_pipeline(pipeline);
                for group in plan.groups.iter().filter(|g| g.clipped && !g.double_sided) {
                    let Some(mesh) = scene.mesh(group.mesh) else {
                        continue;
                    };
                    pass.set_vertex_buffer(0, mesh.positions.slice(..));
                    pass.set_index_buffer(mesh.indices.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(
                        0..mesh.index_count(),
                        0,
                        group.first..group.first + group.count,
                    );
                }
            }
        };

        if depth_pass {
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
            // With caps the pass ends after them (everything stored) so the cap depth pass can
            // read the normal target; `opaque_attachments` then describes the edges pass too.
            let first = opaque_attachments(targets, clear, caps, true);
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("amg opaque"),
                color_attachments: &first.color,
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &targets.depth,
                    depth_ops: Some(wgpu::Operations {
                        load: if depth_pass {
                            wgpu::LoadOp::Load
                        } else {
                            wgpu::LoadOp::Clear(0.0)
                        },
                        // The cap depth, silhouettes and overlay passes read the depth.
                        store: if silhouettes || caps || draw_overlays {
                            wgpu::StoreOp::Store
                        } else {
                            wgpu::StoreOp::Discard
                        },
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
            if caps {
                draw_occluders(&mut pass);
            }
            if shade {
                let (instances, triangles) = draw_meshes(&mut pass, false);
                stats.instances_drawn = instances;
                stats.triangles = triangles;
                draw_highlights(&mut pass, &hl_draws, hl_base, scene, variants);
            }
            if caps {
                drop(pass);
                // Cap samples get the depth of their plane.
                let mut fix = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: Some("amg cap depth"),
                    color_attachments: &[],
                    depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                        view: &targets.depth,
                        depth_ops: Some(wgpu::Operations {
                            load: wgpu::LoadOp::Load,
                            store: wgpu::StoreOp::Store,
                        }),
                        stencil_ops: None,
                    }),
                    timestamp_writes: timer.map(|t| t.writes(pass_index("cap depth"))),
                    occlusion_query_set: None,
                    multiview_mask: None,
                });
                if let Some(p) = variants.get(cap_depth_key) {
                    fix.set_pipeline(p);
                    fix.set_bind_group(0, &slot.bind_group, &[]);
                    fix.set_bind_group(1, &targets.cap_marks_bind, &[]);
                    fix.draw(0..3, 0..1);
                }
                drop(fix);
                let second = opaque_attachments(targets, clear, caps, false);
                pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: Some("amg edges"),
                    color_attachments: &second.color,
                    depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                        view: &targets.depth,
                        depth_ops: Some(wgpu::Operations {
                            load: wgpu::LoadOp::Load,
                            store: if silhouettes || draw_overlays {
                                wgpu::StoreOp::Store
                            } else {
                                wgpu::StoreOp::Discard
                            },
                        }),
                        stencil_ops: None,
                    }),
                    timestamp_writes: timer.map(|t| t.writes(pass_index("edges"))),
                    occlusion_query_set: None,
                    multiview_mask: None,
                });
                pass.set_bind_group(0, &slot.bind_group, &[]);
            }
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
            if !line_plan.draws.is_empty() {
                pass.set_bind_group(0, &slot.bind_group, &[]);
                draw_lines(&mut pass);
            }
        }
        if let Some(p) = variants
            .get(silhouette_key)
            .filter(|_| silhouettes && targets.silhouette_bind.is_some())
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("amg silhouettes"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &targets.resolved,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Load,
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: timer.map(|t| t.writes(pass_index("silhouettes"))),
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_pipeline(p);
            pass.set_bind_group(0, targets.silhouette_bind.as_ref(), &[]);
            pass.draw(0..3, 0..1);
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
        if draw_overlays && let Some(scene_bind) = targets.overlay_scene_bind.as_ref() {
            let timestamp = timer.map(|t| (&t.queries, 2 * pass_index("overlays") as u32));
            record_overlay_pass(
                &OverlayPassCx {
                    device,
                    queue,
                    variants,
                    samples,
                    scene,
                    view_layout: &self.view_bind_layout,
                    geometry_layout,
                    image_layout: &self.overlay_image_layout,
                    sampler: &self.overlay_sampler,
                    white: &self.white_image_bind,
                    buffers: OverlayBuffers {
                        camera: &slot.camera_buffer,
                        visible: &slot.visible_buffer,
                        style: &slot.style_buffer,
                        sections: &slot.section_buffer,
                        states: &self.instance_gpu.states,
                    },
                    final_view: &targets.final_view,
                    overlay_depth: &targets.overlay_depth,
                    scene_bind,
                    insets: &insets,
                    size,
                    timestamp,
                },
                &mut slot.overlay,
                encoder,
            );
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

/// The scene-wide instance storage buffers: 128-byte instance records, the parallel 96-byte
/// line records and the parallel 128-byte analysis records (RND-08).
#[derive(Debug)]
struct InstanceGpu {
    buffer: wgpu::Buffer,
    lines: wgpu::Buffer,
    analysis: wgpu::Buffer,
    /// Edge and point highlight states (`u32`, RND-04).
    states: wgpu::Buffer,
    /// Capacity of `states` in words.
    states_capacity: u64,
    /// The scene highlight-state revision `states` holds.
    states_revision: u64,
    /// Capacity in records.
    capacity: u32,
    /// Bumped when the buffers are replaced, so views rebuild their bind groups.
    generation: u64,
}

impl InstanceGpu {
    fn new(device: &wgpu::Device, capacity: u32, generation: u64) -> InstanceGpu {
        InstanceGpu {
            buffer: storage_buffer(device, "amg instances", u64::from(capacity) * RECORD_BYTES),
            states: storage_buffer(device, "amg highlight states", 4 * INITIAL_STATE_WORDS),
            states_capacity: INITIAL_STATE_WORDS,
            states_revision: 0,
            lines: storage_buffer(
                device,
                "amg line records",
                u64::from(capacity) * LINE_RECORD_BYTES,
            ),
            analysis: storage_buffer(
                device,
                "amg analysis records",
                u64::from(capacity) * ANALYSIS_RECORD_BYTES,
            ),
            capacity,
            generation,
        }
    }
}

/// The pipeline of a line-plan draw.
fn line_key(draw: &LineDraw, samples: u32) -> PipelineKey {
    let pass = draw.pass;
    let kind = match draw.geometry {
        LineGeometry::Wire(_) => PipelineKind::Wire {
            pass,
            clipped: false,
        },
        LineGeometry::Edges(_) => PipelineKind::Line {
            pass,
            clipped: false,
        },
        LineGeometry::Points(_) => PipelineKind::Point {
            pass,
            clipped: false,
        },
    };
    PipelineKey { kind, samples }
}

/// Group 1 of the line and point pipelines: their segment or point buffer.
fn geometry_bind(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    buffer: &wgpu::Buffer,
    ids: &wgpu::Buffer,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("amg line geometry"),
        layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: buffer.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: ids.as_entire_binding(),
            },
        ],
    })
}

/// Group 1 of the analysis pipelines.
fn analysis_bind_group(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    instances: &InstanceGpu,
    lut: &wgpu::TextureView,
    sampler: &wgpu::Sampler,
    globals: &wgpu::Buffer,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("amg analysis"),
        layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: instances.analysis.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::TextureView(lut),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: wgpu::BindingResource::Sampler(sampler),
            },
            wgpu::BindGroupEntry {
                binding: 3,
                resource: globals.as_entire_binding(),
            },
        ],
    })
}

/// Colour attachments of the opaque pass. Without caps it is one pass that clears, resolves
/// and keeps only the normal target. With caps it is split: the `first` part clears and stores
/// everything (the cap depth pass reads the normal target), the second loads and resolves.
struct OpaqueAttachments<'a> {
    color: [Option<wgpu::RenderPassColorAttachment<'a>>; 2],
}

fn opaque_attachments<'a>(
    targets: &'a ViewTargets,
    clear: wgpu::Color,
    split: bool,
    first: bool,
) -> OpaqueAttachments<'a> {
    let msaa = targets.msaa_color.is_some();
    let resolve = !split || !first;
    OpaqueAttachments {
        color: [
            Some(wgpu::RenderPassColorAttachment {
                view: targets.msaa_color.as_ref().unwrap_or(&targets.resolved),
                depth_slice: None,
                resolve_target: targets
                    .msaa_color
                    .as_ref()
                    .filter(|_| resolve)
                    .map(|_| &targets.resolved),
                ops: wgpu::Operations {
                    load: if first {
                        wgpu::LoadOp::Clear(clear)
                    } else {
                        wgpu::LoadOp::Load
                    },
                    store: if msaa && resolve {
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
                    load: if first {
                        wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT)
                    } else {
                        wgpu::LoadOp::Load
                    },
                    store: wgpu::StoreOp::Store,
                },
            }),
        ],
    }
}

/// The normal target (marks of cap samples), multisampled or not.
fn cap_marks_bind_layout(device: &wgpu::Device, multisampled: bool) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("amg cap marks"),
        entries: &[wgpu::BindGroupLayoutEntry {
            binding: 0,
            visibility: wgpu::ShaderStages::FRAGMENT,
            ty: wgpu::BindingType::Texture {
                sample_type: wgpu::TextureSampleType::Float { filterable: false },
                view_dimension: wgpu::TextureViewDimension::D2,
                multisampled,
            },
            count: None,
        }],
    })
}

/// Normal (0) and depth (1) targets, multisampled or not, and the silhouette uniform (2).
fn silhouette_bind_layout(device: &wgpu::Device, multisampled: bool) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("amg silhouettes"),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Float { filterable: false },
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Depth,
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 2,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
        ],
    })
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
    instances: &InstanceGpu,
    visible: &wgpu::Buffer,
    style: &wgpu::Buffer,
    sections: &wgpu::Buffer,
) -> wgpu::BindGroup {
    view_bind_group_parts(
        device,
        layout,
        &ViewBuffers {
            camera,
            instances: &instances.buffer,
            visible,
            style,
            lines: &instances.lines,
            sections,
            states: &instances.states,
        },
    )
}

/// The buffers of a group-0 view bind group.
struct ViewBuffers<'a> {
    camera: &'a wgpu::Buffer,
    instances: &'a wgpu::Buffer,
    visible: &'a wgpu::Buffer,
    style: &'a wgpu::Buffer,
    lines: &'a wgpu::Buffer,
    sections: &'a wgpu::Buffer,
    states: &'a wgpu::Buffer,
}

/// A group-0 view bind group over explicit buffers (the overlay pass binds its own instance and
/// line records, RND-07).
fn view_bind_group_parts(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    b: &ViewBuffers<'_>,
) -> wgpu::BindGroup {
    let ViewBuffers {
        camera,
        instances,
        visible,
        style,
        lines,
        sections,
        states,
    } = *b;
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
            wgpu::BindGroupEntry {
                binding: 4,
                resource: lines.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 5,
                resource: sections.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 6,
                resource: states.as_entire_binding(),
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

/// Layouts and per-view resources the target bind groups need.
struct TargetLayouts<'a> {
    final_bind: &'a wgpu::BindGroupLayout,
    post_bind: &'a wgpu::BindGroupLayout,
    post_sampler: &'a wgpu::Sampler,
    /// For the sample count of the targets.
    silhouette_bind: &'a wgpu::BindGroupLayout,
    silhouette_buffer: &'a wgpu::Buffer,
    /// For the sample count of the targets.
    cap_marks_bind: &'a wgpu::BindGroupLayout,
    /// For the sample count of the targets (RND-07).
    overlay_depth: &'a wgpu::BindGroupLayout,
}

fn create_targets(
    device: &wgpu::Device,
    layouts: TargetLayouts<'_>,
    size: [u32; 2],
    samples: u32,
    sampled_depth: bool,
    silhouettes: bool,
) -> ViewTargets {
    let TargetLayouts {
        final_bind: final_bind_layout,
        post_bind: post_bind_layout,
        post_sampler,
        silhouette_bind: silhouette_bind_layout,
        silhouette_buffer,
        cap_marks_bind: cap_marks_bind_layout,
        overlay_depth: overlay_depth_layout,
    } = layouts;
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
    let cap_marks_bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("amg cap marks"),
        layout: cap_marks_bind_layout,
        entries: &[wgpu::BindGroupEntry {
            binding: 0,
            resource: wgpu::BindingResource::TextureView(&normal_view),
        }],
    });
    // Sampled by the silhouettes pass only when they are on, and by the overlay pass.
    let sampled_depth = sampled_depth || silhouettes;
    let depth_usage = if sampled_depth {
        attach | wgpu::TextureUsages::TEXTURE_BINDING
    } else {
        attach
    };
    let depth =
        target("amg depth", DEPTH_FORMAT, samples, depth_usage, &[]).create_view(&default_view);
    let overlay_depth =
        target("amg overlay depth", DEPTH_FORMAT, 1, attach, &[]).create_view(&default_view);
    let overlay_scene_bind = sampled_depth.then(|| {
        device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("amg overlay scene depth"),
            layout: overlay_depth_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::TextureView(&depth),
            }],
        })
    });
    let silhouette_bind = silhouettes.then(|| {
        device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("amg silhouettes"),
            layout: silhouette_bind_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&normal_view),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::TextureView(&depth),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: silhouette_buffer.as_entire_binding(),
                },
            ],
        })
    });
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
        depth_sampled: sampled_depth,
        overlay_depth,
        overlay_scene_bind,
        cap_marks_bind,
        silhouette_bind,
        resolved,
        final_texture,
        final_view,
        final_bind,
        post_bind,
    }
}

/// The overlay pipelines of a view at `samples` scene samples (RND-07).
fn overlay_keys(samples: u32) -> [PipelineKey; 4] {
    [
        PipelineKind::OverlayTri { write: true },
        PipelineKind::OverlayTri { write: false },
        PipelineKind::OverlayLine,
        PipelineKind::OverlayPoint,
    ]
    .map(|kind| PipelineKey { kind, samples })
}

/// The overlay handle pick pipelines.
fn overlay_pick_keys() -> [PipelineKey; 3] {
    [
        PipelineKind::PickOverlayTri,
        PipelineKind::PickOverlayLine,
        PipelineKind::PickOverlayPoint,
    ]
    .map(pick_key)
}

/// Group 3 of the overlay pipelines: a filterable image and its sampler.
fn overlay_image_bind_layout(device: &wgpu::Device) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("amg overlay image"),
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

fn overlay_image_bind(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    view: &wgpu::TextureView,
    sampler: &wgpu::Sampler,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("amg overlay image"),
        layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::TextureView(view),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::Sampler(sampler),
            },
        ],
    })
}

/// The view's buffers the overlay group-0 bind groups share with the scene passes.
struct OverlayBuffers<'a> {
    camera: &'a wgpu::Buffer,
    visible: &'a wgpu::Buffer,
    style: &'a wgpu::Buffer,
    sections: &'a wgpu::Buffer,
    states: &'a wgpu::Buffer,
}

impl OverlayBuffers<'_> {
    /// Group 0 over the overlay records with `camera`.
    fn bind(
        &self,
        device: &wgpu::Device,
        layout: &wgpu::BindGroupLayout,
        camera: &wgpu::Buffer,
        gpu: &OverlayGpu,
    ) -> Option<wgpu::BindGroup> {
        Some(view_bind_group_parts(
            device,
            layout,
            &ViewBuffers {
                camera,
                instances: gpu.records.get()?,
                visible: self.visible,
                style: self.style,
                lines: gpu.lines.get()?,
                sections: self.sections,
                states: self.states,
            },
        ))
    }
}

struct OverlayPassCx<'a> {
    device: &'a wgpu::Device,
    queue: &'a wgpu::Queue,
    variants: &'a Variants,
    samples: u32,
    scene: &'a RenderScene,
    view_layout: &'a wgpu::BindGroupLayout,
    geometry_layout: &'a wgpu::BindGroupLayout,
    image_layout: &'a wgpu::BindGroupLayout,
    sampler: &'a wgpu::Sampler,
    white: &'a wgpu::BindGroup,
    buffers: OverlayBuffers<'a>,
    final_view: &'a wgpu::TextureView,
    overlay_depth: &'a wgpu::TextureView,
    scene_bind: &'a wgpu::BindGroup,
    insets: &'a [OverlayInset],
    size: [u32; 2],
    /// The view timer's query set and the begin index of the `overlays` pass.
    timestamp: Option<(&'a wgpu::QuerySet, u32)>,
}

/// Uploads the frame's overlay geometry and records the overlay passes into the final image:
/// one for the view's list, then one per inset (viewport and scissor of its rectangle, the
/// overlay depth cleared again). Each pass draws opaque then translucent triangles, lines, then
/// points.
fn record_overlay_pass(
    cx: &OverlayPassCx<'_>,
    gpu: &mut OverlayGpu,
    encoder: &mut wgpu::CommandEncoder,
) {
    let device = cx.device;
    gpu.upload(device, cx.queue);
    while gpu.inset_cameras.len() < cx.insets.len() {
        gpu.inset_cameras
            .push(device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("amg inset camera"),
                size: size_of::<crate::CameraUniform>() as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }));
    }
    for (inset, buffer) in cx.insets.iter().zip(&gpu.inset_cameras) {
        let [_, _, w, h] = inset.rect_px;
        let uniform = inset
            .camera
            .uniform([w.max(1), h.max(1)], inset.camera.default_near());
        cx.queue
            .write_buffer(buffer, 0, bytemuck::bytes_of(&uniform));
    }
    let (Some(segments), Some(segment_ids), Some(points), Some(point_ids), Some(tris)) = (
        gpu.segments.get(),
        gpu.segment_ids.get(),
        gpu.points.get(),
        gpu.point_ids.get(),
        gpu.tris.get(),
    ) else {
        return;
    };
    let segment_bind = geometry_bind(device, cx.geometry_layout, segments, segment_ids);
    let point_bind = geometry_bind(device, cx.geometry_layout, points, point_ids);
    let geometry = &gpu.geometry;
    // Layer i > 0 is inset i - 1; empty layers and insets outside the view are skipped.
    let layers: Vec<(usize, &LayerDraws, Option<[u32; 4]>)> = geometry
        .layers
        .iter()
        .enumerate()
        .filter(|(_, l)| !l.segments.is_empty() || !l.points.is_empty() || !l.tri_draws.is_empty())
        .filter_map(|(i, l)| {
            if i == 0 {
                return Some((i, l, None));
            }
            let [x, y, w, h] = cx.insets.get(i - 1)?.rect_px;
            let (x1, y1) = ((x + w).min(cx.size[0]), (y + h).min(cx.size[1]));
            (x < x1 && y < y1 && w > 0 && h > 0).then_some((i, l, Some([x, y, w, h])))
        })
        .collect();
    let count = layers.len();
    for (n, (i, layer, rect)) in layers.into_iter().enumerate() {
        let camera = if i == 0 {
            cx.buffers.camera
        } else {
            &gpu.inset_cameras[i - 1]
        };
        let Some(bind0) = cx.buffers.bind(device, cx.view_layout, camera, gpu) else {
            return;
        };
        let timestamp_writes = cx.timestamp.and_then(|(query_set, begin)| {
            let (first, last) = (n == 0, n + 1 == count);
            (first || last).then_some(wgpu::RenderPassTimestampWrites {
                query_set,
                beginning_of_pass_write_index: first.then_some(begin),
                end_of_pass_write_index: last.then_some(begin + 1),
            })
        });
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("amg overlays"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: cx.final_view,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Load,
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: cx.overlay_depth,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(0.0),
                    store: wgpu::StoreOp::Discard,
                }),
                stencil_ops: None,
            }),
            timestamp_writes,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        if let Some([x, y, w, h]) = rect {
            pass.set_viewport(x as f32, y as f32, w as f32, h as f32, 0.0, 1.0);
            let (x1, y1) = ((x + w).min(cx.size[0]), (y + h).min(cx.size[1]));
            pass.set_scissor_rect(x, y, x1 - x, y1 - y);
        }
        pass.set_bind_group(0, &bind0, &[]);
        pass.set_bind_group(1, &segment_bind, &[]);
        pass.set_bind_group(2, cx.scene_bind, &[]);
        pass.set_bind_group(3, cx.white, &[]);
        let key = |kind| PipelineKey {
            kind,
            samples: cx.samples,
        };
        if !layer.tri_draws.is_empty() {
            pass.set_vertex_buffer(0, tris.slice(..));
            for d in &geometry.tri_draws[layer.tri_draws.clone()] {
                let Some(p) = cx
                    .variants
                    .get(key(PipelineKind::OverlayTri { write: d.write }))
                else {
                    continue;
                };
                pass.set_pipeline(p);
                let image = d.image.and_then(|k| cx.scene.image(k));
                let bind = image.map_or(cx.white, |img| {
                    img.bind.get_or_init(|| {
                        overlay_image_bind(device, cx.image_layout, &img.view, cx.sampler)
                    })
                });
                pass.set_bind_group(3, bind, &[]);
                pass.draw(d.first..d.first + d.count, 0..1);
            }
            pass.set_bind_group(3, cx.white, &[]);
        }
        if !layer.segments.is_empty()
            && let Some(p) = cx.variants.get(key(PipelineKind::OverlayLine))
        {
            pass.set_pipeline(p);
            pass.draw(6 * layer.segments.start..6 * layer.segments.end, 0..1);
        }
        if !layer.points.is_empty()
            && let Some(p) = cx.variants.get(key(PipelineKind::OverlayPoint))
        {
            pass.set_pipeline(p);
            pass.set_bind_group(1, &point_bind, &[]);
            pass.draw(6 * layer.points.start..6 * layer.points.end, 0..1);
        }
    }
}

/// Overlay handles drawn into a point pick (the view's own list, not the insets).
struct OverlayPick<'a> {
    bind: &'a wgpu::BindGroup,
    segments: &'a wgpu::BindGroup,
    points: &'a wgpu::BindGroup,
    tris: &'a wgpu::Buffer,
    layer: &'a LayerDraws,
    tri_draws: &'a [TriDraw],
}

/// Bits of a visible-list entry above the instance slot (`SLOT_BITS` in `shaders/scene.wgsl`).
const SLOT_BITS: u32 = 28;

/// The timed passes of a pick (one begin, one end).
const PICK_PASSES: &[&str] = &["pick"];

/// One face-range highlight redraw.
#[derive(Clone, Debug)]
struct HighlightDraw {
    key: PipelineKey,
    mesh: MeshKey,
    flat: bool,
    tris: std::ops::Range<u32>,
    /// Its visible-list entry, after the highlight base.
    entry: u32,
}

/// Redraws the highlighted triangles with the depth-`Equal` mesh pipelines (bind group 0 is
/// set).
fn draw_highlights(
    pass: &mut wgpu::RenderPass<'_>,
    draws: &[HighlightDraw],
    base: u32,
    scene: &RenderScene,
    variants: &Variants,
) {
    let mut bound = None;
    for d in draws {
        let (Some(mesh), Some(pipeline)) = (scene.mesh(d.mesh), variants.get(d.key)) else {
            continue;
        };
        if bound != Some(d.key) {
            pass.set_pipeline(pipeline);
            bound = Some(d.key);
        }
        pass.set_vertex_buffer(0, mesh.positions.slice(..));
        if let Some(n) = mesh.normals.as_ref().filter(|_| !d.flat) {
            pass.set_vertex_buffer(1, n.slice(..));
        }
        pass.set_index_buffer(mesh.indices.slice(..), wgpu::IndexFormat::Uint32);
        let instance = base + d.entry;
        pass.draw_indexed(d.tris.start * 3..d.tris.end * 3, 0, instance..instance + 1);
    }
}

/// A run of pickable instances of one mesh in the pick pass.
#[derive(Clone, Copy, Debug)]
struct PickGroup {
    mesh: MeshKey,
    mode: FaceMode,
    double_sided: bool,
    clipped: bool,
    first: u32,
    count: u32,
}

impl PickGroup {
    fn key(&self) -> PipelineKey {
        pick_key(PipelineKind::PickFace {
            mode: self.mode,
            double_sided: self.double_sided,
            clipped: self.clipped,
        })
    }

    /// A cut closed mesh: drawn without culling, its back faces are cap candidates.
    fn capped(&self) -> bool {
        self.clipped && !self.double_sided
    }
}

/// A run of pickable edge or point sets in the pick pass.
#[derive(Clone, Copy, Debug)]
struct PickLines {
    geometry: LineGeometry,
    always: bool,
    first: u32,
    count: u32,
}

/// The pick-pass draws of one view: the view's own draw plan (same culling and section
/// classification) without the instances that are not pickable, hidden-pass lines and
/// triangle wireframes.
#[derive(Debug, Default)]
struct PickPlan {
    groups: Vec<PickGroup>,
    lines: Vec<PickLines>,
    slots: Vec<u32>,
    owners: Vec<(PickId, OwnerSizes)>,
}

fn pick_key(kind: PipelineKind) -> PipelineKey {
    PipelineKey { kind, samples: 1 }
}

fn line_pick_key(l: &PickLines, clipped: bool) -> PipelineKey {
    let always = l.always;
    pick_key(match l.geometry {
        LineGeometry::Points(_) => PipelineKind::PickPoint { clipped, always },
        _ => PipelineKind::PickLine { clipped, always },
    })
}

fn build_pick_plan(
    plan: &DrawPlan,
    line_plan: &LinePlan,
    scene: &RenderScene,
    mode: FaceMode,
) -> PickPlan {
    let instances = scene.table().instances();
    let pickable = |slot: u32| {
        let i = &instances[slot as usize];
        i.pickable && i.layer != Layer::Ghost
    };
    let mut out = PickPlan::default();
    for g in &plan.groups {
        let Some(mesh) = scene.mesh(g.mesh) else {
            continue;
        };
        let first = out.slots.len() as u32;
        out.slots.extend(
            plan.slots[g.first as usize..(g.first + g.count) as usize]
                .iter()
                .copied()
                .filter(|&s| pickable(s)),
        );
        let count = out.slots.len() as u32 - first;
        if count > 0 {
            out.groups.push(PickGroup {
                mesh: g.mesh,
                mode: if mesh.face_ids.is_some() {
                    FaceMode::Attribute
                } else {
                    mode
                },
                double_sided: g.double_sided,
                clipped: g.clipped,
                first,
                count,
            });
        }
    }
    for d in &line_plan.draws {
        let always = match d.pass {
            LinePass::Hidden => continue,
            LinePass::Visible => false,
            LinePass::Always => true,
        };
        if matches!(d.geometry, LineGeometry::Wire(_)) {
            continue;
        }
        let first = out.slots.len() as u32;
        out.slots.extend(
            line_plan.slots[d.first as usize..(d.first + d.count) as usize]
                .iter()
                .copied()
                .filter(|&s| pickable(s)),
        );
        let count = out.slots.len() as u32 - first;
        if count > 0 {
            out.lines.push(PickLines {
                geometry: d.geometry,
                always,
                first,
                count,
            });
        }
    }
    let mut seen = std::collections::HashSet::new();
    for &slot in &out.slots {
        if !seen.insert(slot) {
            continue;
        }
        let inst = &instances[slot as usize];
        out.owners.push((
            inst.pick,
            OwnerSizes {
                faces: inst
                    .mesh
                    .and_then(|k| scene.mesh(k))
                    .map_or(0, GpuMesh::face_count),
                edges: inst
                    .edges
                    .and_then(|k| scene.edge_set(k))
                    .map_or(0, |e| e.id_count),
                vertices: inst
                    .points
                    .and_then(|k| scene.point_set(k))
                    .map_or(0, |p| p.id_count),
            },
        ));
    }
    out
}

/// Rectangle-select parameters (`RectUniform` in `shaders/pick_rect.wgsl`).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, bytemuck::Pod, bytemuck::Zeroable)]
struct RectUniform {
    min: [u32; 2],
    max: [u32; 2],
    size: [u32; 2],
    picks: u32,
    pad: u32,
}

const RECT_WGSL: &str = include_str!("shaders/pick_rect.wgsl");

/// Builds the rectangle-select compute pipeline on a background thread.
fn spawn_rect_pipeline(gpu: &Arc<Gpu>) -> RectPipeline {
    let cell: RectPipeline = Arc::default();
    let (out, gpu) = (Arc::clone(&cell), Arc::clone(gpu));
    let spawned = std::thread::Builder::new()
        .name("amg-render rect pick".into())
        .spawn(move || {
            let desc =
                amg_jobs::gpu::ComputePipelineDesc::new("amg pick rect", RECT_WGSL, "rect_flags");
            let built = gpu.compute_pipeline(&desc);
            if let Err(e) = &built {
                tracing::error!(error = %e, "rectangle pick pipeline failed");
            }
            let _ = out.set(built.ok());
        });
    if let Err(e) = spawned {
        tracing::error!(error = %e, "cannot spawn the rectangle pick pipeline build");
        let _ = cell.set(None);
    }
    cell
}

/// Pick id (uint) and depth (unfilterable float) textures for the pick cap depth pass.
fn pick_targets_bind_layout(device: &wgpu::Device) -> wgpu::BindGroupLayout {
    let texture = |binding, sample_type| wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::FRAGMENT,
        ty: wgpu::BindingType::Texture {
            sample_type,
            view_dimension: wgpu::TextureViewDimension::D2,
            multisampled: false,
        },
        count: None,
    };
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("amg pick targets"),
        entries: &[
            texture(0, wgpu::TextureSampleType::Uint),
            texture(1, wgpu::TextureSampleType::Float { filterable: false }),
        ],
    })
}

fn create_pick_targets(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    size: [u32; 2],
) -> PickTargets {
    let texture = |label, format| {
        device.create_texture(&wgpu::TextureDescriptor {
            label: Some(label),
            size: wgpu::Extent3d {
                width: size[0],
                height: size[1],
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT
                | wgpu::TextureUsages::COPY_SRC
                | wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        })
    };
    let ids = texture("amg pick ids", PICK_ID_FORMAT);
    let positions = texture("amg pick positions", PICK_POS_FORMAT);
    let depth_test = texture("amg pick depth test", DEPTH_FORMAT);
    let ids_view = ids.create_view(&wgpu::TextureViewDescriptor::default());
    let positions_view = positions.create_view(&wgpu::TextureViewDescriptor::default());
    let fix_bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("amg pick targets"),
        layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::TextureView(&ids_view),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::TextureView(&positions_view),
            },
        ],
    });
    PickTargets {
        size,
        ids,
        ids_view,
        positions,
        positions_view,
        depth_test: depth_test.create_view(&wgpu::TextureViewDescriptor::default()),
        fix_bind,
    }
}

/// Maps `staging` once the encoder is submitted; the state flips to `MAP_DONE` / `MAP_FAILED`.
fn map_on_submit(encoder: &mut wgpu::CommandEncoder, staging: &wgpu::Buffer) -> Arc<AtomicU8> {
    let state = Arc::new(AtomicU8::new(MAP_PENDING));
    let done = Arc::clone(&state);
    encoder.map_buffer_on_submit(staging, wgpu::MapMode::Read, .., move |r| {
        done.store(
            if r.is_ok() { MAP_DONE } else { MAP_FAILED },
            Ordering::Release,
        );
    });
    state
}

/// Group 1 of the vertex-pulling face pick.
fn pull_bind(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    indices: &wgpu::Buffer,
    positions: &wgpu::Buffer,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("amg pick pull"),
        layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 2,
                resource: indices.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 3,
                resource: positions.as_entire_binding(),
            },
        ],
    })
}

/// What one pick request needs from its view and the renderer while it is recorded.
struct PickContext<'a> {
    device: &'a wgpu::Device,
    scene: &'a RenderScene,
    variants: &'a Variants,
    plan: &'a PickPlan,
    targets: &'a PickTargets,
    bind: &'a wgpu::BindGroup,
    geometry_layout: &'a wgpu::BindGroupLayout,
    pull_layout: &'a wgpu::BindGroupLayout,
    /// The view's planes apply (line pipelines clip).
    sectioned: bool,
    /// Two planes or more (occluders needed for caps).
    bounded: bool,
    /// Point picks: the view pixel at the target's top-left texel (the viewport is offset by it,
    /// so the view transform is unchanged). Rectangles render the whole view.
    viewport: Option<([i64; 2], [u32; 2])>,
    filter: PickFilter,
    query_set: Option<&'a wgpu::QuerySet>,
    /// A compute pass follows the render passes (it writes the end timestamp).
    compute_follows: bool,
    /// Overlay handles (RND-07), drawn last.
    overlay: Option<OverlayPick<'a>>,
}

/// Records the render passes of one pick: faces (with occluders before a bounded cut), then
/// edges and points; with caps and lines, a cap depth pass in between.
fn record_pick_passes(cx: &PickContext<'_>, encoder: &mut wgpu::CommandEncoder) {
    let (device, scene, variants, plan, targets) =
        (cx.device, cx.scene, cx.variants, cx.plan, cx.targets);
    let faces = cx.filter.draws_faces();
    let lines: Vec<PickLines> = plan
        .lines
        .iter()
        .copied()
        .filter(|l| match l.geometry {
            LineGeometry::Edges(_) => cx.filter.contains(PickFilter::EDGES),
            LineGeometry::Points(_) => cx.filter.contains(PickFilter::VERTICES),
            LineGeometry::Wire(_) => false,
        })
        .collect();
    let capped = faces && plan.groups.iter().any(PickGroup::capped);
    let fix = capped && !lines.is_empty();
    let writes = |begin: bool, end: bool| {
        cx.query_set.map(|qs| wgpu::RenderPassTimestampWrites {
            query_set: qs,
            beginning_of_pass_write_index: begin.then_some(0),
            end_of_pass_write_index: end.then_some(1),
        })
    };
    let set_viewport = |pass: &mut wgpu::RenderPass<'_>| {
        if let Some((origin, size)) = cx.viewport {
            pass.set_viewport(
                -(origin[0] as f32),
                -(origin[1] as f32),
                size[0] as f32,
                size[1] as f32,
                0.0,
                1.0,
            );
        }
    };
    let color = |load: wgpu::LoadOp<wgpu::Color>| {
        let ops = wgpu::Operations {
            load,
            store: wgpu::StoreOp::Store,
        };
        [
            Some(wgpu::RenderPassColorAttachment {
                view: &targets.ids_view,
                depth_slice: None,
                resolve_target: None,
                ops,
            }),
            Some(wgpu::RenderPassColorAttachment {
                view: &targets.positions_view,
                depth_slice: None,
                resolve_target: None,
                ops,
            }),
        ]
    };
    let depth_attachment = |load: wgpu::LoadOp<f32>| {
        Some(wgpu::RenderPassDepthStencilAttachment {
            view: &targets.depth_test,
            depth_ops: Some(wgpu::Operations {
                load,
                store: wgpu::StoreOp::Store,
            }),
            stencil_ops: None,
        })
    };
    let draw_lines = |pass: &mut wgpu::RenderPass<'_>| {
        let mut bound = None;
        for l in &lines {
            let key = line_pick_key(l, cx.sectioned);
            let Some(pipeline) = variants.get(key) else {
                continue;
            };
            let (bind, vertices) = match l.geometry {
                LineGeometry::Edges(k) => {
                    let Some(e) = scene.edge_set(k) else {
                        continue;
                    };
                    let bind = e.bind.get_or_init(|| {
                        geometry_bind(device, cx.geometry_layout, &e.segments, &e.ids)
                    });
                    (bind, e.segment_count() * 6)
                }
                LineGeometry::Points(k) => {
                    let Some(p) = scene.point_set(k) else {
                        continue;
                    };
                    let bind = p.bind.get_or_init(|| {
                        geometry_bind(device, cx.geometry_layout, &p.points, &p.ids)
                    });
                    (bind, p.point_count() * 6)
                }
                LineGeometry::Wire(_) => continue,
            };
            if bound != Some(key) {
                pass.set_pipeline(pipeline);
                bound = Some(key);
            }
            pass.set_bind_group(1, bind, &[]);
            pass.draw(0..vertices, l.first..l.first + l.count);
        }
    };
    let draw_overlay = |pass: &mut wgpu::RenderPass<'_>| {
        let Some(o) = &cx.overlay else {
            return;
        };
        pass.set_bind_group(0, o.bind, &[]);
        if !o.layer.tri_draws.is_empty()
            && let Some(p) = variants.get(pick_key(PipelineKind::PickOverlayTri))
        {
            pass.set_pipeline(p);
            pass.set_vertex_buffer(0, o.tris.slice(..));
            for d in o.tri_draws {
                pass.draw(d.first..d.first + d.count, 0..1);
            }
        }
        if !o.layer.segments.is_empty()
            && let Some(p) = variants.get(pick_key(PipelineKind::PickOverlayLine))
        {
            pass.set_pipeline(p);
            pass.set_bind_group(1, o.segments, &[]);
            pass.draw(6 * o.layer.segments.start..6 * o.layer.segments.end, 0..1);
        }
        if !o.layer.points.is_empty()
            && let Some(p) = variants.get(pick_key(PipelineKind::PickOverlayPoint))
        {
            pass.set_pipeline(p);
            pass.set_bind_group(1, o.points, &[]);
            pass.draw(6 * o.layer.points.start..6 * o.layer.points.end, 0..1);
        }
    };
    let last_end = !cx.compute_follows;
    {
        let attachments = color(wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT));
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("amg pick"),
            color_attachments: &attachments,
            depth_stencil_attachment: depth_attachment(wgpu::LoadOp::Clear(0.0)),
            timestamp_writes: writes(true, last_end && !fix),
            occlusion_query_set: None,
            multiview_mask: None,
        });
        set_viewport(&mut pass);
        pass.set_bind_group(0, cx.bind, &[]);
        if faces {
            if cx.bounded
                && capped
                && let Some(p) = variants.get(pick_key(PipelineKind::PickOccluder))
            {
                pass.set_pipeline(p);
                for g in plan.groups.iter().filter(|g| g.capped()) {
                    let Some(mesh) = scene.mesh(g.mesh) else {
                        continue;
                    };
                    pass.set_vertex_buffer(0, mesh.positions.slice(..));
                    pass.set_index_buffer(mesh.indices.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..mesh.index_count(), 0, g.first..g.first + g.count);
                }
            }
            let max_binding = device.limits().max_storage_buffer_binding_size;
            for g in &plan.groups {
                let (Some(mesh), Some(pipeline)) = (scene.mesh(g.mesh), variants.get(g.key()))
                else {
                    continue;
                };
                let instances = g.first..g.first + g.count;
                match g.mode {
                    FaceMode::Attribute | FaceMode::Primitive => {
                        pass.set_pipeline(pipeline);
                        pass.set_vertex_buffer(0, mesh.positions.slice(..));
                        if g.mode == FaceMode::Attribute {
                            let Some(face_ids) = mesh.face_ids.as_ref() else {
                                continue;
                            };
                            pass.set_vertex_buffer(1, face_ids.slice(..));
                        }
                        pass.set_index_buffer(mesh.indices.slice(..), wgpu::IndexFormat::Uint32);
                        pass.draw_indexed(0..mesh.index_count(), 0, instances);
                    }
                    FaceMode::Pulling => {
                        if mesh.indices.size() > max_binding || mesh.positions.size() > max_binding
                        {
                            tracing::warn!(
                                triangles = mesh.triangle_count(),
                                "mesh too large for vertex-pulling picks; not pickable"
                            );
                            continue;
                        }
                        let pull = mesh.pull_bind.get_or_init(|| {
                            pull_bind(device, cx.pull_layout, &mesh.indices, &mesh.positions)
                        });
                        pass.set_pipeline(pipeline);
                        pass.set_bind_group(1, pull, &[]);
                        pass.draw(0..mesh.index_count(), instances);
                    }
                }
            }
        }
        if !fix {
            draw_lines(&mut pass);
            draw_overlay(&mut pass);
        }
    }
    if fix {
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("amg pick cap depth"),
                color_attachments: &[],
                depth_stencil_attachment: depth_attachment(wgpu::LoadOp::Load),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            if let Some(p) = variants.get(pick_key(PipelineKind::PickFix)) {
                set_viewport(&mut pass);
                pass.set_pipeline(p);
                pass.set_bind_group(0, &targets.fix_bind, &[]);
                pass.draw(0..3, 0..1);
            }
        }
        let attachments = color(wgpu::LoadOp::Load);
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("amg pick edges"),
            color_attachments: &attachments,
            depth_stencil_attachment: depth_attachment(wgpu::LoadOp::Load),
            timestamp_writes: writes(false, last_end),
            occlusion_query_set: None,
            multiview_mask: None,
        });
        set_viewport(&mut pass);
        pass.set_bind_group(0, cx.bind, &[]);
        draw_lines(&mut pass);
        draw_overlay(&mut pass);
    }
}

/// Copies both (region-sized) pick targets into a staging buffer: returns it, the byte offset of
/// the positions and the padded row sizes of ids and positions.
fn record_point_readback(
    encoder: &mut wgpu::CommandEncoder,
    device: &wgpu::Device,
    targets: &PickTargets,
) -> (wgpu::Buffer, u64, u32, u32) {
    let extent = targets.size;
    let ids_row = (extent[0] * 8).next_multiple_of(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT);
    let pos_row = (extent[0] * 16).next_multiple_of(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT);
    let pos_offset = u64::from(ids_row) * u64::from(extent[1]);
    let staging = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("amg pick readback"),
        size: pos_offset + u64::from(pos_row) * u64::from(extent[1]),
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    for (texture, offset, row) in [
        (&targets.ids, 0, ids_row),
        (&targets.positions, pos_offset, pos_row),
    ] {
        encoder.copy_texture_to_buffer(
            wgpu::TexelCopyTextureInfo {
                texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::TexelCopyBufferInfo {
                buffer: &staging,
                layout: wgpu::TexelCopyBufferLayout {
                    offset,
                    bytes_per_row: Some(row),
                    rows_per_image: Some(extent[1]),
                },
            },
            wgpu::Extent3d {
                width: extent[0],
                height: extent[1],
                depth_or_array_layers: 1,
            },
        );
    }
    (staging, pos_offset, ids_row, pos_row)
}

/// Records the rectangle compute pass over the full-view ids and the flag readback; `None` when
/// nothing has flags.
#[allow(
    clippy::too_many_arguments,
    reason = "plain inputs of one recording step"
)]
fn record_rect(
    encoder: &mut wgpu::CommandEncoder,
    device: &wgpu::Device,
    pipeline: &wgpu::ComputePipeline,
    targets: &PickTargets,
    table: &crate::pick::RectTable,
    rect: ([u32; 2], [u32; 2]),
    size: [u32; 2],
    query_set: Option<&wgpu::QuerySet>,
) -> wgpu::Buffer {
    use wgpu::util::DeviceExt as _;
    let clamp = |v: [u32; 2]| [v[0].min(size[0]), v[1].min(size[1])];
    let uniform = RectUniform {
        min: clamp(rect.0),
        max: clamp(rect.1),
        size,
        picks: (table.table.len() / 8) as u32,
        pad: 0,
    };
    let uniform = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("amg pick rect"),
        contents: bytemuck::bytes_of(&uniform),
        usage: wgpu::BufferUsages::UNIFORM,
    });
    let table_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("amg pick rect table"),
        contents: bytemuck::cast_slice(&table.table),
        usage: wgpu::BufferUsages::STORAGE,
    });
    let bytes = u64::from(table.flags) * 4;
    let flags = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("amg pick rect flags"),
        size: bytes,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    });
    let layout = pipeline.get_bind_group_layout(0);
    let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("amg pick rect"),
        layout: &layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::TextureView(&targets.ids_view),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: uniform.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: table_buffer.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 3,
                resource: flags.as_entire_binding(),
            },
        ],
    });
    {
        let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("amg pick rect"),
            timestamp_writes: query_set.map(|qs| wgpu::ComputePassTimestampWrites {
                query_set: qs,
                beginning_of_pass_write_index: None,
                end_of_pass_write_index: Some(1),
            }),
        });
        pass.set_pipeline(pipeline);
        pass.set_bind_group(0, &bind, &[]);
        pass.dispatch_workgroups(size[0].div_ceil(8), size[1].div_ceil(8), 1);
    }
    let staging = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("amg pick rect readback"),
        size: bytes,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    encoder.copy_buffer_to_buffer(&flags, 0, &staging, 0, bytes);
    staging
}

impl Renderer {
    /// Queues a pick on `id`. It is rendered by the view's next [`Renderer::encode`] and
    /// answered by [`Renderer::poll_picks`] once that work has finished on the GPU (normally by
    /// the next frame). Suppressing hover picks during camera drags is the caller's job.
    pub fn request_pick(&mut self, id: ViewId, req: PickRequest) -> PickTicket {
        let ticket = PickTicket(self.picks.next);
        self.picks.next += 1;
        if self.views.contains_key(id) {
            self.picks.queued.push(QueuedPick {
                ticket,
                view: id,
                request: req,
            });
        } else {
            self.picks
                .done
                .push((ticket, Err(RenderError::UnknownKey("view"))));
        }
        ticket
    }

    /// Non-blocking: polls the readbacks and returns the picks answered since the last call,
    /// each ticket exactly once.
    pub fn poll_picks(&mut self) -> Vec<(PickTicket, Result<PickResult, RenderError>)> {
        if self.pending_picks() != 0
            && let Err(e) = self.gpu.poll()
        {
            let picks = &mut self.picks;
            let mut failed: Vec<PickTicket> = picks.in_flight.drain(..).map(|f| f.ticket).collect();
            // After a device loss no `encode` will record the queued picks either.
            if matches!(e, GpuError::DeviceLost(_)) {
                failed.extend(picks.queued.drain(..).map(|q| q.ticket));
            }
            for ticket in failed {
                let error = match &e {
                    GpuError::DeviceLost(m) => GpuError::DeviceLost(m.clone()).into(),
                    other => RenderError::InvalidData(other.to_string()),
                };
                picks.done.push((ticket, Err(error)));
            }
        }
        let views = &self.views;
        let picks = &mut self.picks;
        let mut gone = Vec::new();
        picks.queued.retain(|q| {
            let live = views.contains_key(q.view);
            if !live {
                gone.push(q.ticket);
            }
            live
        });
        for ticket in gone {
            picks
                .done
                .push((ticket, Err(RenderError::UnknownKey("view"))));
        }
        if picks.in_flight.iter().any(InFlight::ready) {
            let table = self.scene.table();
            let mut owner: HashMap<PickId, InstanceKey> = HashMap::new();
            for (inst, &key) in table.instances().iter().zip(table.keys()) {
                owner.entry(inst.pick).or_insert(key);
            }
            let lookup = |p: PickId| owner.get(&p).copied();
            let (ready, pending): (Vec<_>, Vec<_>) =
                picks.in_flight.drain(..).partition(InFlight::ready);
            picks.in_flight = pending;
            for f in ready {
                let ticket = f.ticket;
                picks.done.push((ticket, f.finish(&lookup)));
            }
        }
        std::mem::take(&mut self.picks.done)
    }

    /// Picks queued or in flight.
    pub fn pending_picks(&self) -> usize {
        self.picks.queued.len() + self.picks.in_flight.len()
    }

    /// The point-pick radius, physical pixels.
    pub fn pick_radius_px(&self) -> u32 {
        self.pick_radius
    }

    /// `encode` calls so far (pick latency is counted in these).
    pub fn encode_count(&self) -> u64 {
        self.encodes
    }

    /// The backend plain meshes use for triangle picks on this device:
    /// [`PickBackend::PrimitiveIndex`] or [`PickBackend::VertexPulling`].
    pub fn pick_backend(&self) -> PickBackend {
        match self.face_mode() {
            FaceMode::Primitive => PickBackend::PrimitiveIndex,
            _ => PickBackend::VertexPulling,
        }
    }

    /// GPU time of every finished pick since the last call (the passes of the first pick of
    /// each frame; needs `TIMESTAMP_QUERY`), oldest first. One pass, `pick`.
    pub fn drain_pick_timings(&mut self, id: ViewId) -> Vec<GpuTimings> {
        self.views
            .get_mut(id)
            .and_then(|s| s.pick_timer.as_mut())
            .map_or_else(Vec::new, |t| {
                t.harvest();
                t.finished.drain(..).collect()
            })
    }

    fn face_mode(&self) -> FaceMode {
        let prim = self.variants.factory().has_primitive_index();
        match self.pick_backend {
            PickBackend::VertexPulling => FaceMode::Pulling,
            PickBackend::Auto | PickBackend::PrimitiveIndex if prim => FaceMode::Primitive,
            PickBackend::PrimitiveIndex | PickBackend::Auto => FaceMode::Pulling,
        }
    }

    /// Requests the pick pipelines (non-blocking; once they are built, picks are answered
    /// within two `encode` calls).
    fn request_pick_variants(&mut self, sectioned: bool) {
        let mode = self.face_mode();
        let clips: &[bool] = if sectioned { &[false, true] } else { &[false] };
        for &clipped in clips {
            for double_sided in [false, true] {
                for m in [FaceMode::Attribute, mode] {
                    self.variants.request(pick_key(PipelineKind::PickFace {
                        mode: m,
                        double_sided,
                        clipped,
                    }));
                }
            }
            for always in [false, true] {
                self.variants
                    .request(pick_key(PipelineKind::PickLine { clipped, always }));
                self.variants
                    .request(pick_key(PipelineKind::PickPoint { clipped, always }));
            }
        }
        if sectioned {
            self.variants.request(pick_key(PipelineKind::PickOccluder));
            self.variants.request(pick_key(PipelineKind::PickFix));
        }
        for key in overlay_pick_keys() {
            self.variants.request(key);
        }
    }

    /// Records the queued picks of `id` after its frame (that frame's draw plans are still in
    /// `self.plan` / `self.line_plan`). Picks whose pipelines are not built yet stay queued.
    fn record_picks(&mut self, id: ViewId, encoder: &mut wgpu::CommandEncoder) {
        let Some(view) = self.views.get(id).map(|s| s.view.clone()) else {
            return;
        };
        let sections = if view.sections_enabled {
            self.scene.active_sections()
        } else {
            Vec::new()
        };
        let sectioned = !sections.is_empty();
        self.request_pick_variants(sectioned);
        if !self.picks.queued.iter().any(|q| q.view == id) {
            return;
        }
        let (mine, rest): (Vec<_>, Vec<_>) =
            self.picks.queued.drain(..).partition(|q| q.view == id);
        self.picks.queued = rest;
        let size = view.size_px;
        // The same depth range as the view's frames (RND-11), so pick depth matches the image.
        let depth_range = view_depth_range(&self.scene, &view.camera, view.layers);
        if size[0] == 0 || size[1] == 0 {
            for q in mine {
                self.picks.done.push((
                    q.ticket,
                    Err(RenderError::InvalidData(
                        "the view has a zero dimension".into(),
                    )),
                ));
            }
            return;
        }

        let plan = build_pick_plan(&self.plan, &self.line_plan, &self.scene, self.face_mode());
        let bounded = sections.len() >= 2;
        let capped = plan.groups.iter().any(PickGroup::capped);
        let mut keys: Vec<PipelineKey> = plan.groups.iter().map(PickGroup::key).collect();
        keys.extend(plan.lines.iter().map(|l| line_pick_key(l, sectioned)));
        if capped {
            keys.push(pick_key(PipelineKind::PickFix));
            if bounded {
                keys.push(pick_key(PipelineKind::PickOccluder));
            }
        }
        // RND-07: the view's overlay handles from this frame's tessellation.
        let handles = self
            .views
            .get(id)
            .and_then(|s| s.overlay.geometry.layers.first())
            .is_some_and(|l| l.pickable);
        if handles {
            keys.extend(overlay_pick_keys());
        }
        let mut ready = true;
        for &k in &keys {
            if self.variants.has(k) {
                continue;
            }
            if self.variants.failed(k) {
                tracing::warn!(variant = %k.describe(), "pick pipeline failed; that geometry is not pickable");
            } else {
                self.variants.request(k);
                ready = false;
            }
        }
        let rect_pipeline = self.rect_pipeline.get().cloned();

        let gpu = Arc::clone(&self.gpu);
        let device = gpu.device();
        let queue = gpu.queue();
        let Some(slot) = self.views.get_mut(id) else {
            return;
        };
        // Point picks draw into a `(2r + 1)²` target kept by the view; rectangles into full-view
        // targets made for the request.
        let side = 2 * self.pick_radius + 1;
        if slot
            .pick_targets
            .as_ref()
            .is_none_or(|t| t.size != [side, side])
        {
            slot.pick_targets = Some(create_pick_targets(
                device,
                &self.pick_targets_bind_layout,
                [side, side],
            ));
        }
        let needed = plan.slots.len() as u32;
        if needed > slot.pick_visible_capacity {
            slot.pick_visible_capacity = needed.next_power_of_two().max(INITIAL_VISIBLE_CAPACITY);
            slot.pick_visible = storage_buffer(
                device,
                "amg pick visible",
                u64::from(slot.pick_visible_capacity) * 4,
            );
        }
        if !plan.slots.is_empty() {
            queue.write_buffer(&slot.pick_visible, 0, bytemuck::cast_slice(&plan.slots));
        }
        let Some(region_targets) = slot.pick_targets.as_ref() else {
            return;
        };
        let mut timed = false;
        for q in mine {
            let rect = match (q.request.region, &rect_pipeline) {
                (PickRegion::Rect { .. }, None) => {
                    // The compute pipeline is still being built.
                    self.picks.queued.push(q);
                    continue;
                }
                (PickRegion::Rect { .. }, Some(None)) => {
                    self.picks.done.push((
                        q.ticket,
                        Err(RenderError::InvalidData(
                            "the rectangle pick pipeline could not be built".into(),
                        )),
                    ));
                    continue;
                }
                (PickRegion::Rect { min, max, mode }, Some(Some(p))) => {
                    Some((Arc::clone(p), min, max, mode))
                }
                (PickRegion::Point { .. }, _) => None,
            };
            if !ready {
                self.picks.queued.push(q);
                continue;
            }
            let filter = q.request.filter;
            let camera = view.camera;
            let rect_targets;
            let (uniform, viewport, point, targets) = match q.request.region {
                PickRegion::Point { px } => {
                    let px = [f64::from(px[0]), f64::from(px[1])];
                    let (cursor, origin) = point_region(px, self.pick_radius, size);
                    // The centre of the cursor texel samples the exact cursor ray.
                    let shift = [
                        f64::from(cursor[0]) + 0.5 - px[0],
                        f64::from(cursor[1]) + 0.5 - px[1],
                    ];
                    let mut uniform = camera.uniform_shifted(size, depth_range, shift);
                    // The view pixel of the target's top-left texel (`surface_point`, pick.wgsl).
                    uniform.eye_hi[3] = origin[0] as f32;
                    uniform.eye_lo[3] = origin[1] as f32;
                    (uniform, Some((origin, size)), Some(px), region_targets)
                }
                PickRegion::Rect { .. } => {
                    rect_targets =
                        create_pick_targets(device, &self.pick_targets_bind_layout, size);
                    (
                        camera.uniform_in(size, depth_range),
                        None,
                        None,
                        &rect_targets,
                    )
                }
            };
            let camera_buffer = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("amg pick camera"),
                size: size_of::<crate::CameraUniform>() as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            queue.write_buffer(&camera_buffer, 0, bytemuck::bytes_of(&uniform));
            let bind = view_bind_group(
                device,
                &self.view_bind_layout,
                &camera_buffer,
                &self.instance_gpu,
                &slot.pick_visible,
                &slot.style_buffer,
                &slot.section_buffer,
            );
            // Overlay handles in point picks that ask for them.
            let overlay_parts = (handles
                && point.is_some()
                && filter.contains(PickFilter::HANDLES))
            .then(|| {
                let o = &slot.overlay;
                let bind = OverlayBuffers {
                    camera: &camera_buffer,
                    visible: &slot.pick_visible,
                    style: &slot.style_buffer,
                    sections: &slot.section_buffer,
                    states: &self.instance_gpu.states,
                }
                .bind(device, &self.view_bind_layout, &camera_buffer, o)?;
                let segments = geometry_bind(
                    device,
                    &self.geometry_bind_layout,
                    o.segments.get()?,
                    o.segment_ids.get()?,
                );
                let points = geometry_bind(
                    device,
                    &self.geometry_bind_layout,
                    o.points.get()?,
                    o.point_ids.get()?,
                );
                Some((bind, segments, points))
            })
            .flatten();
            let overlay = overlay_parts.as_ref().and_then(|(bind, segments, points)| {
                let o = &slot.overlay;
                let layer = o.geometry.layers.first()?;
                Some(OverlayPick {
                    bind,
                    segments,
                    points,
                    tris: o.tris.get()?,
                    layer,
                    tri_draws: &o.geometry.tri_draws[layer.tri_draws.clone()],
                })
            });
            let timer_slot = if timed {
                None
            } else {
                slot.pick_timer.as_mut().and_then(|t| {
                    t.harvest();
                    t.begin(PICK_PASSES)
                })
            };
            timed |= timer_slot.is_some();
            let query_set = timer_slot.and(slot.pick_timer.as_ref()).map(|t| &t.queries);
            // Built first: an empty table runs no compute pass, so the render passes then write
            // the end timestamp.
            let rect_table = rect
                .as_ref()
                .map(|_| build_rect_table(&plan.owners, filter));
            let compute_follows = rect_table.as_ref().is_some_and(|t| t.flags != 0);
            record_pick_passes(
                &PickContext {
                    device,
                    scene: &self.scene,
                    variants: &self.variants,
                    plan: &plan,
                    targets,
                    bind: &bind,
                    geometry_layout: &self.geometry_bind_layout,
                    pull_layout: &self.pull_bind_layout,
                    sectioned,
                    bounded,
                    viewport,
                    filter,
                    query_set,
                    compute_follows,
                    overlay,
                },
                encoder,
            );
            let (staging, kind) = match (point, rect) {
                (Some(px), _) => {
                    let (staging, pos_offset, ids_row, pos_row) =
                        record_point_readback(encoder, device, targets);
                    let readback = PointReadback {
                        px,
                        width: side,
                        height: side,
                        centre: [self.pick_radius, self.pick_radius],
                        radius: self.pick_radius,
                        pos_offset,
                        ids_row,
                        pos_row,
                        camera,
                        size,
                        sections: sections.clone(),
                    };
                    (Some(staging), ReadbackKind::Point(readback))
                }
                (None, Some((pipeline, min, max, mode))) => {
                    let table = rect_table.unwrap_or_default();
                    if table.flags == 0 {
                        (None, ReadbackKind::Empty(PickResult::Rect(Vec::new())))
                    } else {
                        let staging = record_rect(
                            encoder,
                            device,
                            &pipeline,
                            targets,
                            &table,
                            (min, max),
                            size,
                            query_set,
                        );
                        let readback = RectReadback {
                            entries: table.entries,
                            mode,
                        };
                        (Some(staging), ReadbackKind::Rect(readback))
                    }
                }
                (None, None) => (None, ReadbackKind::Empty(PickResult::Point(Vec::new()))),
            };
            if let (Some(index), Some(t)) = (timer_slot, slot.pick_timer.as_ref()) {
                t.finish(index, encoder);
            }
            let state = match &staging {
                Some(s) => map_on_submit(encoder, s),
                None => Arc::new(AtomicU8::new(MAP_DONE)),
            };
            self.picks.in_flight.push(InFlight {
                ticket: q.ticket,
                filter,
                staging,
                state,
                kind,
            });
        }
    }
}
