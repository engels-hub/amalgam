# S10 — Rendering & viewport

| | |
|---|---|
| **Status** | Refined |
| **Priority** | P0 |
| **Crates** | `amg-render` |
| **Lane** | D |
| **Depends on** | S00 (FND-09 `Gpu`, FND-04 jobs, FND-06 testkit); S17 (MAT-02) for RND-10. Fed by the S11 scene adapter (UI-20/UI-22), which uses S01 DOC-09, S07 MESH-02 and S02 BREP-08 |
| **Parity** | Viewport, visual styles, view cube, section analysis, zebra/draft/curvature analyses, display component colours (`03-fusion-parity.md` Inspection rows; AM overhang row) |

## Summary
A wgpu viewport for CAD and mesh work. It offers:
- fast shaded + edges rendering of large meshes and tessellated B-rep (5 M triangles at
  ≥ 60 fps);
- precise GPU picking of faces, edges and vertices;
- selection highlighting;
- section planes with watertight caps;
- transparency and x-ray;
- analysis overlays (zebra, draft, overhang, curvature/thickness fields) and FEA result fringe
  plots with deformation animation;
- compositing of implicit ray-march layers;
- headless rendering for thumbnails and visual regression tests;
- an egui widget that draws on the **same** `wgpu::Device` that compute uses (FND-09).

`amg-render` never sees `amg-doc` types. It is fed through `RenderScene`, a renderer-owned
mirror that the S11 scene adapter (`amg-ui`, UI-20/UI-22) keeps in sync from `DocEvent`s. Evidence, formulas and
per-pass detail are in the design note
[`docs/notes/viewport-rendering.md`](../notes/viewport-rendering.md).

## Scope
**In:**
- `amg-render`:
  - `Renderer`, `View`, `Camera` (f64);
  - the off-thread upload API (`upload_mesh` / `upload_edges` / `upload_points` / fields);
  - `RenderScene` (meshes, edge sets, point sets, fields, instances, sections, external
    layer, overlays);
  - the frame graph (opaque, caps, external, edges, WBOIT, post, overlays);
  - visual styles, picking, highlight;
  - section planes and caps;
  - analysis shaders, colormaps and legends;
  - deformation;
  - navigation math, presets and view cube;
  - headless capture, thumbnails and the golden-image harness;
  - device-loss handling and GPU timings;
  - the egui glue (`egui_glue`, cargo feature `egui`, default on): viewport widget, view
    cube, nav bar.

**Out:**
- The `DocEvent` → `RenderScene` scene adapter → S11:
  - UI-20: mesh bodies, `RenderEntityKey` = our `PickId`, `PickMap`, highlight and preview
    feed;
  - UI-22: solid bodies via BREP-08.

  Sketches and construction geometry are fed as RND-07 overlays by S11 (UI-09). The contract
  is §Design "Contract for the scene adapter".
- Viewport input routing, tool state machine, `PickService` and selection sets → S11
  UI-03/UI-04. They call the `nav` and pick APIs defined here.
- Navigation preset tables (button/modifier → orbit/pan/zoom) and their preferences UI → S11
  UI-03/UI-11. S10 provides the camera math they drive.
- Producing analysis data:
  - curvature values → S05 INS-05, with S07/S02 data;
  - wall thickness → S07 MESH-11;
  - overhang semantics, support area and orientation → S16 AM-02;
  - FEA result surfaces and values → S13 FEM-06;
  - ray-march layer → S08 IMP-05.

  The renderer only displays them.
- Tessellation (S02 BREP-08) and mesh f32 upload prep (S07 `render_buffers`).
- Photoreal rendering (parity P3), drawings (P3).

## Design
Detail and formulas: design note §1–§18. Decisions:

1. **One device, egui paint callback.**
   - `amg-app` builds eframe with `egui_wgpu::WgpuSetup::Existing(WgpuSetupExisting { instance,
     adapter, device, queue })` taken from the FND-09 `Gpu`.
   - `egui_glue::install` puts the `Renderer` into egui-wgpu's `callback_resources`.
   - Each `Viewport` widget adds an `egui_wgpu::Callback`:
     - `prepare` records every 3D pass into the view's own offscreen targets (MSAA, depth, ids)
       on egui's encoder;
     - `paint` copies the final `Rgba8UnormSrgb` image 1:1 into egui's single-sample pass,
       with viewport and scissor from `PaintCallbackInfo`.
   - egui's pass keeps no MSAA and no depth.
   - `install` fails with `DeviceMismatch` unless `render_state.device == *gpu.device()`.
2. **Precision.**
   - Uploads are rebased to a per-mesh origin.
   - Per-instance matrices are computed relative-to-eye in f64 and then cast to f32.
   - Depth is reversed-Z Depth32Float with an infinite far plane.
   - Pick world positions are reconstructed in f64.
3. **Uploads never touch the UI thread.** `upload_*` run in `amg-jobs` Interactive jobs
   (`wgpu::Device` is `Send + Sync`). The UI thread only inserts the finished `GpuMesh` into
   the scene (≤ 50 µs).
4. **Frame graph** (note §2):
   1. opaque forward PBR-lite (no depth pre-pass unless the RND-02 bench says otherwise);
   2. back-face section caps;
   3. external layer composite;
   4. wide-line edges and points (visible, then dashed hidden);
   5. WBOIT transparency;
   6. resolve + WBOIT composite + silhouettes + optional FXAA;
   7. overlays and view cube.

   The architecture's "Forward+" is not needed: there are only 2–3 fixed lights.
5. **Section planes** use fragment `discard` (wgpu `CLIP_DISTANCES` is unavailable on DX12),
   up to 4 planes. Caps are drawn in one pass from visible back faces by ray–plane
   intersection, writing `frag_depth`. There's no stencil, the cap colour is per body, and
   caps are pickable. Caps are drawn only for instances marked `closed`.
6. **Picking.**
   - Id texel `Rg32Uint` = (`PickId`, `kind << 30 | sub`) plus R32Float depth, rendered on
     demand into a scissored `(2r+1)²` region.
   - Results arrive ≤ 2 frames later.
   - Priority: vertex, then edge, then face.
   - Face sub-id: B-rep uses a flat per-vertex face id; meshes use `@builtin(primitive_index)`
     when `PRIMITIVE_INDEX` exists, else a vertex-pulling fallback. Both are tested.
   - Rectangle select (window/crossing) uses a compute pass over a full-view id render.
7. **Lines** are screen-space quads with a capsule SDF: any width, AA, round joins, dashes.
   Depth offset is in view space, proportional to the pixel size at depth. Native 1-px lines
   are never used.
8. **Text.** Screen-space labels are painted by egui at anchors the renderer projects. This
   gives crisp text at all zooms with no MSDF pipeline (decision; note §12).
9. **Analyses** are shader variants with formulas fixed in note §11:
   - zebra, draft and overhang are computed from the normal, so they are live under rotation;
   - curvature, thickness and FEA use per-vertex scalar fields with a colormap LUT, bands,
     isolines and a legend;
   - FEA displacement is a per-vertex vector stream × scale × animation.
10. **Navigation.**
    - World up is +Z.
    - Orbit is about the pick point, zoom is to the cursor point, and pan keeps the grabbed
      point under the cursor.
    - Also: fit, standard views, animated `look_at_plane` (sketch mode), projection switch, a
      26-region view cube, and 300 ms animated transitions.
    - All of this is pure f64 math in `amg_render::nav`, unit-tested without a GPU.
    - Which button does what (Fusion/SolidWorks/Blender presets) is S11's `NavPreset` table,
      which calls `NavController`.
11. **Headless.** `Renderer` works without a surface. Goldens are PNGs rendered on WARP.
12. **Device loss** is detected and reported. The viewport shows a placeholder and S11 offers
    save-and-restart. There's no automatic device re-creation.

### Contract for the scene adapter (S11 UI-20 / UI-22)
This answers S11 requests R-S10-1..10.
- **Ids.** The adapter allocates dense `PickId(u32)`, which is S11's `RenderEntityKey`, one per
  displayed (occurrence path, body), sketch or overlay owner. Sub-ids:
  - meshes: triangle index (`MeshData::face_ranges` empty);
  - B-rep: face index (via `face_ranges`), edge index (`Polyline::id`), vertex index
    (`PointData::ids`);
  - overlay handles, which is how sketch curves and points drawn as RND-07 overlays are
    picked: their handle id.

  The adapter's `PickMap` translates hits back to document entities.
- **Uploads.** The adapter runs `upload_*` inside its own jobs (`BodyTessellator`) and inserts
  the results with `RenderScene::insert_*` on the UI thread. To swap geometry atomically, it
  points the instance at the new `MeshKey` and then removes the old one.
- **Instances.** One per (occurrence path, body). Meshes are shared across occurrences of the
  same component. `BodyProps` map as follows:
  - appearance → `Material`;
  - `opacity` → `base_color.a`;
  - `visible` → `Instance::visible`.

  Props changes never re-upload geometry.
- **Highlights** are per layer (`Hover`, `Selected`, `Error`, `Preview`) as
  `(PickId, Option<(SubKind, sub)>)` lists, with colours from theme tokens (R-S10-5/8).
  Preview bodies use `Layer::Preview` (ghost style, R-S10-6).
- **Camera data for S11's `CameraView`:** `Camera::view_matrix`, `Camera::proj_matrix` (f64)
  and `ViewportResponse::rect`.
- **Picking.** Results are visible-surface only, from the id buffer.
  - `Nearest` is `PickResult::Point[0]`.
  - S11's `AllAlongRay` (select-through) cannot come from an id buffer. S11 serves it from a
    CPU ray cast in its `PickService` (domain BVH / `amg-brep` ray hit), and S10 returns the
    front-most hits only.

## Interfaces
Signatures are for the public API other crates use. Bodies are omitted. Every public item gets
a doc comment (`#![warn(missing_docs)]`).

```rust
// ===== amg_render (crate root) =====
pub use wgpu;                                     // == workspace wgpu 30.0.1 (same as amg_jobs::gpu)
use amg_core::{Aabb3, Dir3, Plane, Point3, Ray3, Transform, Vec3, Cancelled};
use amg_jobs::{gpu::{Gpu, GpuError, Readback}, JobCtx};
use std::{ops::Range, sync::Arc};

/// OR these into `GpuConfig::optional_features` before `Gpu::new` (all have fallbacks).
pub const OPTIONAL_FEATURES: wgpu::Features = wgpu::Features::PRIMITIVE_INDEX
    .union(wgpu::Features::POLYGON_MODE_LINE)
    .union(wgpu::Features::TIMESTAMP_QUERY);

amg_core::define_id! {
    pub struct ViewId; pub struct MeshKey; pub struct EdgesKey; pub struct PointsKey;
    pub struct FieldKey; pub struct InstanceKey; pub struct SectionKey; pub struct ImageKey;
}
/// Dense id chosen by the adapter, written to the id buffer. 0 = nothing.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Default)]
pub struct PickId(pub u32);
/// Sub-id used for section caps (kind Face).
pub const CAP_SUB: u32 = (1 << 30) - 1;

#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum RenderError {
    #[error(transparent)] Gpu(#[from] GpuError),        // incl. DeviceLost
    #[error("the egui device is not the shared Gpu device")] DeviceMismatch,
    #[error("{what} too large: {bytes} bytes exceeds the device limit {limit}")]
    TooLarge { what: &'static str, bytes: u64, limit: u64 },
    #[error("field has {got} values, mesh has {expected} vertices")] FieldLength { got: usize, expected: usize },
    #[error("invalid mesh data: {0}")] InvalidData(String),   // index out of range, NaN position, …
    #[error("unknown {0} key")] UnknownKey(&'static str),
    #[error(transparent)] Cancelled(#[from] Cancelled),
}
impl amg_core::Diagnostic for RenderError { /* hints: update driver, AMG_WGPU_BACKEND */ }

// ===== renderer =====
#[derive(Clone, Debug)]
pub struct RendererConfig {
    pub msaa_samples: u32,           // 1 or 4; default 4
    pub pick_radius_px: u32,         // default 5
    pub pick_backend: PickBackend,   // default Auto
}
impl Default for RendererConfig { fn default() -> Self; }
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum PickBackend { #[default] Auto, PrimitiveIndex, VertexPulling }

/// Send + Sync. One per app; owns pipelines, the scene and all views.
pub struct Renderer { /* Arc<Gpu>, pipelines, RenderScene, SlotMap<ViewId, ViewTargets>, pick queue */ }
impl Renderer {
    /// Creates core pipelines synchronously (≤ 150 ms); other variants lazily on jobs.
    pub fn new(gpu: Arc<Gpu>, cfg: &RendererConfig) -> Result<Renderer, RenderError>;
    pub fn gpu(&self) -> &Arc<Gpu>;
    pub fn scene(&self) -> &RenderScene;
    pub fn scene_mut(&mut self) -> &mut RenderScene;
    pub fn create_view(&mut self, size_px: [u32; 2]) -> ViewId;
    pub fn remove_view(&mut self, id: ViewId);
    pub fn view(&self, id: ViewId) -> Option<&View>;
    pub fn view_mut(&mut self, id: ViewId) -> Option<&mut View>;
    /// Records all passes for `id` (and pending picks) into `encoder`. Never blocks.
    pub fn encode(&mut self, id: ViewId, encoder: &mut wgpu::CommandEncoder) -> Result<FrameStats, RenderError>;
    /// Final single-sample Rgba8UnormSrgb image of the last `encode`.
    pub fn final_texture(&self, id: ViewId) -> Option<&wgpu::Texture>;
    pub fn request_pick(&mut self, id: ViewId, req: PickRequest) -> PickTicket;
    /// Non-blocking (polls readbacks); each ticket is returned exactly once.
    pub fn poll_picks(&mut self) -> Vec<(PickTicket, Result<PickResult, RenderError>)>;
    /// World-anchored labels of the current overlay list, projected for egui painting.
    pub fn project_labels(&self, id: ViewId) -> Vec<ProjectedLabel>;
    /// GPU pass timings of the newest finished frame (needs TIMESTAMP_QUERY).
    pub fn gpu_timings(&self, id: ViewId) -> Option<GpuTimings>;
    /// Headless: encodes, submits and starts reading back `final`.
    pub fn capture(&mut self, id: ViewId) -> Result<Capture, RenderError>;
}
#[derive(Clone, Copy, Debug, Default)]
pub struct FrameStats { pub instances_drawn: u32, pub triangles: u64, pub edge_segments: u64, pub culled: u32 }
#[derive(Clone, Debug, Default)]
pub struct GpuTimings { pub total_ms: f32, pub passes: Vec<(&'static str, f32)> }
pub struct Capture { /* Readback<u8>, size, padded row */ }
impl Capture {
    pub fn try_get(&mut self) -> Option<Result<RgbaImage, RenderError>>;   // non-blocking
    pub fn wait(self) -> Result<RgbaImage, RenderError>;                   // job/test threads only
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RgbaImage { pub size: [u32; 2], pub pixels: Vec<u8> /* sRGB RGBA8, tightly packed */ }
impl RgbaImage { pub fn to_png(&self) -> Vec<u8>; pub fn from_png(bytes: &[u8]) -> Result<RgbaImage, RenderError>; }

// ===== upload (job threads; all inputs are plain data, no amg-mesh/amg-brep types) =====
/// Positions are f32 relative to `origin` (S07 `RenderBuffers` maps 1:1).
#[derive(Clone, Debug, Default)]
pub struct MeshData {
    pub origin: Point3,
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,       // empty → flat shading from derivatives
    pub indices: Vec<u32>,            // triangle list
    pub face_ranges: Vec<FaceRange>,  // empty → sub-id = triangle index (meshes); else B-rep faces
    pub closed: bool,                 // watertight → section caps allowed
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FaceRange { pub first_tri: u32, pub tri_count: u32, pub face: u32 }
#[derive(Clone, Debug, Default)]
pub struct EdgeData { pub origin: Point3, pub points: Vec<[f32; 3]>, pub polylines: Vec<Polyline> }
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Polyline { pub first: u32, pub len: u32, pub id: u32 /* edge sub-id */, pub closed: bool }
#[derive(Clone, Debug, Default)]
pub struct PointData { pub origin: Point3, pub points: Vec<[f32; 3]>, pub ids: Vec<u32> /* empty → index */ }

pub fn upload_mesh(gpu: &Gpu, data: &MeshData, ctx: &JobCtx) -> Result<GpuMesh, RenderError>;
pub fn upload_edges(gpu: &Gpu, data: &EdgeData, ctx: &JobCtx) -> Result<GpuEdges, RenderError>;
pub fn upload_points(gpu: &Gpu, data: &PointData, ctx: &JobCtx) -> Result<GpuPoints, RenderError>;
/// One value per mesh vertex (curvature, thickness, stress, …).
pub fn upload_scalar_field(gpu: &Gpu, mesh: &GpuMesh, values: &[f32], ctx: &JobCtx) -> Result<GpuField, RenderError>;
/// One displacement per mesh vertex, in mm, in the mesh's local frame.
pub fn upload_vector_field(gpu: &Gpu, mesh: &GpuMesh, values: &[[f32; 3]], ctx: &JobCtx) -> Result<GpuField, RenderError>;
/// RGBA8 sRGB image for image planes / sprites (IO-10 canvases, glyph icons).
pub fn upload_image(gpu: &Gpu, img: &RgbaImage) -> Result<GpuImage, RenderError>;

/// GPU-resident, immutable, Send + Sync. Dropping frees VRAM once in-flight frames finish.
pub struct GpuMesh { /* buffers, counts, local bounds, origin, flags */ }
impl GpuMesh {
    pub fn triangle_count(&self) -> u32; pub fn vertex_count(&self) -> u32;
    pub fn bounds(&self) -> Aabb3;  /* world-of-mesh frame (origin applied), f64 */
    pub fn gpu_bytes(&self) -> u64; pub fn is_closed(&self) -> bool;
}
pub struct GpuEdges { /* … */ }  pub struct GpuPoints { /* … */ }
pub struct GpuField { /* … */ }  pub struct GpuImage { /* … */ }

// ===== scene =====
/// Renderer-owned mirror of what to draw. Mutations are cheap (no geometry work).
pub struct RenderScene { /* SlotMaps + dirty sets + revision */ }
impl RenderScene {
    pub fn insert_mesh(&mut self, m: GpuMesh) -> MeshKey;
    pub fn insert_edges(&mut self, e: GpuEdges) -> EdgesKey;
    pub fn insert_points(&mut self, p: GpuPoints) -> PointsKey;
    pub fn insert_field(&mut self, f: GpuField) -> FieldKey;
    pub fn insert_image(&mut self, i: GpuImage) -> ImageKey;
    pub fn remove_mesh(&mut self, k: MeshKey) -> bool;     // likewise remove_edges/points/field/image
    pub fn remove_edges(&mut self, k: EdgesKey) -> bool;
    pub fn remove_points(&mut self, k: PointsKey) -> bool;
    pub fn remove_field(&mut self, k: FieldKey) -> bool;
    pub fn remove_image(&mut self, k: ImageKey) -> bool;
    pub fn add_instance(&mut self, inst: Instance) -> InstanceKey;
    pub fn instance(&self, k: InstanceKey) -> Option<&Instance>;
    /// Marks the instance dirty (re-uploads its 128 B record only).
    pub fn instance_mut(&mut self, k: InstanceKey) -> Option<&mut Instance>;
    pub fn remove_instance(&mut self, k: InstanceKey) -> bool;
    /// Replaces the whole item list of one highlight layer (S11 R-S10-5).
    pub fn set_highlight_layer(&mut self, layer: HighlightLayer, items: &[HighlightItem]);
    pub fn set_highlight_color(&mut self, layer: HighlightLayer, rgba: [f32; 4]);
    pub fn add_section(&mut self, s: SectionPlane) -> SectionKey;   // max 4 active
    pub fn section_mut(&mut self, k: SectionKey) -> Option<&mut SectionPlane>;
    pub fn remove_section(&mut self, k: SectionKey) -> bool;
    pub fn set_external_layer(&mut self, layer: Option<ExternalLayer>);
    /// Immediate-mode overlays: cleared by `begin_overlays`, refilled by the active tool each frame.
    pub fn begin_overlays(&mut self) -> &mut OverlayList;
    pub fn visible_bounds(&self) -> Aabb3;
    pub fn gpu_bytes(&self) -> u64;
    pub fn revision(&self) -> u64;
}
#[derive(Clone, Debug)]
pub struct Instance {
    pub mesh: Option<MeshKey>, pub edges: Option<EdgesKey>, pub points: Option<PointsKey>,
    pub transform: Transform,         // world_from_mesh-local (origin handled internally), f64
    pub pick: PickId, pub pickable: bool, pub visible: bool,
    pub layer: Layer,
    pub material: Material,
    pub shading: Shading,
    pub sectioned: bool,              // affected by section planes
    pub edge_style: LineStyle, pub point_style: PointStyle,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Layer { #[default] Model, Preview, Ghost /* WBOIT, not pickable */, Construction }
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Material { pub base_color: [f32; 4] /* linear; a < 1 → WBOIT */, pub metallic: f32,
                      pub roughness: f32, pub cap_color: Option<[f32; 3]> }
#[derive(Clone, Debug, PartialEq)]
pub enum Shading { Material, Field(FieldBinding), Analysis(Analysis) }
#[derive(Clone, Debug, PartialEq)]
pub struct FieldBinding {
    pub scalar: Option<FieldKey>, pub map: ColorMap, pub range: (f32, f32),
    pub bands: Option<u32>, pub isolines: Option<u32>,
    pub below: OutOfRange, pub above: OutOfRange,
    pub deformation: Option<Deformation>,
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum OutOfRange { Clamp, Color([f32; 4]) }
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Deformation { pub field: FieldKey, pub scale: f32, pub animation: Animation }
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Animation { Static, Sine { period_s: f32 }, Ramp { period_s: f32 } }
/// Formulas: design note §11 (the single definition shared with S05/S16).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Analysis {
    Zebra { stripes: u32, vertical: bool, world_locked: bool },
    Draft { pull: Dir3, max_angle: f64, required: f64 },
    Overhang { build_up: Dir3, critical: f64, plate_height: f64, plate_tol: f64 },
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ColorMap { #[default] Viridis, Turbo, Rainbow, BlueWhiteRed, Grey }
#[derive(Clone, Debug, PartialEq)]
pub struct Legend { pub map: ColorMap, pub min: f32, pub max: f32, pub ticks: Vec<f32>, pub bands: Option<u32> }
/// "Nice" ticks (1, 2, 2.5, 5 × 10^k) inside [min, max], ≤ `max_ticks`.
pub fn legend(binding: &FieldBinding, max_ticks: usize) -> Legend;
/// Linear RGB of `map` at t ∈ [0, 1] (CPU reference; used for legends and tests).
pub fn colormap_rgb(map: ColorMap, t: f32) -> [f32; 3];

/// Draw precedence: Error > Selected > Hover > Preview (Selected+Hover is drawn lighter).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum HighlightLayer { Hover, Selected, Error, Preview }
/// `sub == None` highlights the whole pick owner (body, overlay owner).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct HighlightItem { pub pick: PickId, pub sub: Option<(SubKind, Range32)> }
/// Half-open sub-id range (a B-rep face, a run of mesh triangles, one edge: `n..n+1`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Range32 { pub start: u32, pub end: u32 }
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SectionPlane { pub plane: Plane /* keeps the side opposite plane.normal */,
                          pub caps: bool, pub hatch: Hatch, pub enabled: bool }
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hatch { pub angle: f64, pub spacing_px: f32, pub color: [f32; 4] }
/// Foreign colour+depth (IMP-05 ray march). Contract: design note §14.
#[derive(Clone, Debug)]
pub struct ExternalLayer { pub color: wgpu::TextureView /* Rgba16Float premultiplied */,
                           pub depth: wgpu::TextureView /* R32Float reversed-Z NDC, 0 = empty */,
                           pub size_px: [u32; 2], pub pick: PickId }

// ===== lines, points, overlays (RND-07, RND-12) =====
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LineStyle { pub width_px: f32, pub color: [f32; 4], pub dash: Option<(f32, f32)> /* on, off px */,
                       pub depth: DepthMode }
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PointStyle { pub size_px: f32, pub color: [f32; 4], pub depth: DepthMode }
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DepthMode { Test, AlwaysOnTop, XRay { hidden_alpha: f32 } }
pub struct OverlayList { /* per-frame vertex/instance arenas */ }
impl OverlayList {
    pub fn polyline(&mut self, pts: &[Point3], style: LineStyle, handle: Option<(PickId, u32)>);
    pub fn arrow(&mut self, origin: Point3, dir: Dir3, length_px: f32, style: LineStyle, handle: Option<(PickId, u32)>);
    pub fn ring(&mut self, center: Point3, normal: Dir3, radius_px: f32, style: LineStyle, handle: Option<(PickId, u32)>);
    pub fn triad(&mut self, frame: &Transform, size_px: f32, handle_base: Option<PickId>);
    pub fn plane_quad(&mut self, plane: &Plane, half_size: [f64; 2], fill: [f32; 4], border: LineStyle, handle: Option<(PickId, u32)>);
    pub fn grid(&mut self, plane: &Plane, extent: [f64; 2], color: [f32; 4]);   // adaptive spacing
    pub fn image_plane(&mut self, img: ImageKey, corners: [Point3; 4], opacity: f32);
    pub fn points(&mut self, pts: &[Point3], style: PointStyle, handle: Option<(PickId, u32)>);
    /// `uv` selects a sub-rectangle of `img` (icon atlas glyphs, R-S10-7).
    pub fn sprite(&mut self, at: Point3, img: ImageKey, uv: [[f32; 2]; 2], size_px: f32, handle: Option<(PickId, u32)>);
    /// Filled planar polygon (profiles, regions); triangulated by the caller-supplied fan/CDT order.
    pub fn polygon(&mut self, pts: &[Point3], tris: &[[u32; 3]], fill: [f32; 4], depth: DepthMode, handle: Option<(PickId, u32)>);
    pub fn label(&mut self, anchor: Point3, text: impl Into<String>, style: LabelStyle);
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LabelStyle { pub size_pt: f32, pub color: [f32; 4], pub offset_px: [f32; 2], pub background: Option<[f32; 4]> }
#[derive(Clone, Debug, PartialEq)]
pub struct ProjectedLabel { pub pos_px: [f32; 2], pub depth: f32, pub text: String, pub style: LabelStyle }

// ===== camera & view =====
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Camera { pub eye: Point3, pub target: Point3, pub up: Dir3, pub projection: Projection }
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Projection { Perspective { fov_y: f64 }, Orthographic { height: f64 /* mm */ } }
impl Camera {
    pub fn forward(&self) -> Dir3;
    /// Ray through pixel `px` (origin top-left, y down) for a view of `size_px`.
    pub fn ray(&self, px: [f64; 2], size_px: [u32; 2]) -> Ray3;
    /// Pixel position + reversed-Z depth; None if behind the eye.
    pub fn project(&self, p: Point3, size_px: [u32; 2]) -> Option<[f64; 3]>;
    pub fn unproject(&self, px: [f64; 2], depth: f32, size_px: [u32; 2]) -> Point3;
    /// World size of one pixel at the depth of `p` (mm/px).
    pub fn pixel_size_at(&self, p: Point3, size_px: [u32; 2]) -> f64;
    pub fn uniform(&self, size_px: [u32; 2], near: f64) -> CameraUniform;
    /// f64 world→view and view→clip (reversed-Z) for S11 `CameraView` and CPU hit tests.
    pub fn view_matrix(&self) -> amg_core::Mat4;
    pub fn proj_matrix(&self, size_px: [u32; 2]) -> amg_core::Mat4;
}
/// Layout fixed in `shaders/camera.wgsl` (design note §14). Pod for external producers.
#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct CameraUniform {
    pub view_rte: [[f32; 4]; 4], pub proj: [[f32; 4]; 4], pub inv_view_proj_rte: [[f32; 4]; 4],
    pub eye_hi: [f32; 4], pub eye_lo: [f32; 4], pub size_px: [f32; 2], pub pixel_size_at_1mm: f32, pub ortho: u32,
}
#[derive(Clone, Debug)]
pub struct View { pub camera: Camera, pub style: ViewStyle, pub sections_enabled: bool, pub show_view_cube: bool }
impl View { pub fn size_px(&self) -> [u32; 2]; }
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViewStyle { pub visual: VisualStyle, pub xray: bool, pub silhouettes: bool, pub fxaa: bool,
                       pub lighting: Lighting, pub background: Background }
/// Fusion's six visual styles.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum VisualStyle { Shaded, #[default] ShadedVisibleEdges, ShadedHiddenEdges,
                       Wireframe, WireframeHiddenEdges, WireframeVisibleEdges }
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub enum Lighting { #[default] Studio, Matcap, Flat }
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Background { Solid([f32; 3]), Gradient { top: [f32; 3], bottom: [f32; 3] } }

// ===== picking =====
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct PickTicket(pub u64);
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PickRequest { pub region: PickRegion, pub filter: PickFilter }
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PickRegion { Point { px: [f32; 2] }, Rect { min: [u32; 2], max: [u32; 2], mode: RectMode } }
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RectMode { Window, Crossing }
bitflags::bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
    pub struct PickFilter: u8 { const BODIES = 1; const FACES = 2; const EDGES = 4; const VERTICES = 8; const HANDLES = 16; }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum SubKind { Face, Edge, Vertex, Handle }
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PickHit { pub pick: PickId, pub instance: Option<InstanceKey>, pub kind: SubKind, pub sub: u32,
                     pub depth: f32, pub world: Point3, pub distance_px: f32 }
#[derive(Clone, Debug, PartialEq)]
pub enum PickResult {
    /// Best hit first (vertex > edge > face, then distance), then the other distinct hits.
    Point(Vec<PickHit>),
    Rect(Vec<(PickId, SubKind, u32)>),
}

// ===== navigation (RND-11) — pure f64 math, no GPU =====
pub mod nav {
    use super::*;
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum StandardView { Front, Back, Left, Right, Top, Bottom, Home, Iso { x: i8, y: i8, z: i8 } }
    #[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
    pub enum OrbitMode { #[default] Free, Constrained }
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum NavMode { Orbit, Pan, Zoom }
    /// Holds drag state and the running transition. Inputs in physical pixels.
    #[derive(Clone, Debug, Default)]
    pub struct NavController { /* drag anchor, pivot, transition */ }
    impl NavController {
        /// `anchor` = world point under the cursor from a pick (None on background).
        pub fn begin(&mut self, mode: NavMode, cam: &Camera, cursor_px: [f64; 2], anchor: Option<Point3>, scene: Aabb3);
        pub fn drag(&mut self, cam: &mut Camera, cursor_px: [f64; 2], size_px: [u32; 2]);
        pub fn end(&mut self);
        pub fn zoom(&mut self, cam: &mut Camera, steps: f64, cursor_px: [f64; 2], anchor: Option<Point3>, size_px: [u32; 2]);
        pub fn fit(&mut self, cam: &mut Camera, bounds: Aabb3, size_px: [u32; 2], animate: bool);
        pub fn set_view(&mut self, cam: &mut Camera, v: StandardView, bounds: Aabb3, size_px: [u32; 2], animate: bool);
        pub fn set_projection(&mut self, cam: &mut Camera, p: Projection);   // keeps apparent size at target
        /// Sketch "look at": view along −plane.normal with plane.x to the right, fitting `bounds`.
        pub fn look_at_plane(&mut self, cam: &mut Camera, plane: &Plane, bounds: Aabb3, size_px: [u32; 2], animate: bool);
        pub fn set_orbit_mode(&mut self, m: OrbitMode);
        /// Advances transitions; returns true while animating (request repaint).
        pub fn tick(&mut self, cam: &mut Camera, dt: std::time::Duration) -> bool;
    }
    #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
    pub enum CubeRegion { Face(StandardView), Edge { a: StandardView, b: StandardView }, Corner { x: i8, y: i8, z: i8 }, Home }
    /// Hit test in the view cube inset (`px` relative to the inset's top-left).
    pub fn view_cube_hit(cam: &Camera, px: [f64; 2], inset_px: u32) -> Option<CubeRegion>;
    pub fn view_for_region(r: CubeRegion) -> StandardView;
}

// ===== egui glue (cargo feature "egui", default) =====
#[cfg(feature = "egui")]
pub mod egui_glue {
    use super::*;
    /// Puts the Renderer into `render_state.renderer.callback_resources`.
    /// Errors: DeviceMismatch if egui's device is not `gpu.device()`.
    pub fn install(rs: &egui_wgpu::RenderState, gpu: Arc<Gpu>, cfg: &RendererConfig,
                   egui_msaa_samples: u32) -> Result<(), RenderError>;
    /// Short write lock on egui's renderer; None if `install` was not called.
    pub fn with_renderer<R>(rs: &egui_wgpu::RenderState, f: impl FnOnce(&mut Renderer) -> R) -> Option<R>;
    #[derive(Clone, Copy, Debug)]
    pub struct Viewport { pub view: ViewId }
    impl Viewport {
        /// Allocates all available space, resizes the view and adds the paint callback.
        pub fn show(&self, ui: &mut egui::Ui, rs: &egui_wgpu::RenderState) -> ViewportResponse;
    }
    #[derive(Clone, Debug)]
    pub struct ViewportResponse { pub response: egui::Response, pub rect: egui::Rect,
                                  pub pixels_per_point: f32, pub size_px: [u32; 2], pub lost: bool }
    /// Paints `project_labels(view)` with egui (crisp at any zoom).
    pub fn paint_labels(ui: &egui::Ui, resp: &ViewportResponse, labels: &[ProjectedLabel]);
    /// View-cube inset interaction (drawn by the renderer; hit-tested here). Returns the
    /// action for the shell to forward (S11 §Navigation).
    pub fn view_cube(ui: &mut egui::Ui, resp: &ViewportResponse, cam: &Camera) -> Option<CameraAction>;
    /// Fusion-style bottom nav bar (orbit, pan, zoom, fit, projection, visual style menu).
    pub fn nav_bar(ui: &mut egui::Ui, style: &ViewStyle, cam: &Camera) -> Option<CameraAction>;
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub enum CameraAction { SetView(nav::StandardView), Region(nav::CubeRegion), Orbit { delta_px: [f32; 2] },
                            Fit, SetProjection(Projection), SetVisualStyle(VisualStyle), StartMode(nav::NavMode) }
}
```

**Needed from other specs (requests, additive):**
- **S00 (FND-02/FND-09):**
  - workspace `wgpu = "30.0.1"` (replacing `"29"`);
  - add `egui`, `egui-wgpu`, `eframe = "0.37.0"` (eframe as a dev-dep of `amg-render`, a dep
    of `amg-app`) and `bitflags = "2"`, `png = "0.18"`;
  - `GpuConfig` gains `dx12_compiler: wgpu::Dx12Compiler` (default `Fxc`) in case RND-01's
    measurement adopts `StaticDxc`;
  - the FND-09 egui doctest becomes a real doctest, since egui-wgpu is then a workspace dep;
  - `amg-app` ORs `amg_render::OPTIONAL_FEATURES` into `GpuConfig::optional_features`.
- **S07** (consumed by S11's mesh tessellator, UI-20):
  - `RenderBuffers.normals` is empty when `smooth_normals == false`, and vertices stay shared
    (no duplication; the renderer shades flat from derivatives);
  - new `amg_mesh::analysis::feature_edges(&IndexedMesh, crease_angle: f64) -> Vec<[u32; 2]>`
    (boundary, non-manifold, crease and face-group border edges; ≤ 50 ms at 1 M tris);
  - new per-vertex `vertex_curvature(&IndexedMesh, CurvatureKind) -> Vec<f64>` (mean, Gaussian
    or max-abs principal) for INS-05 curvature maps.
- **S02 (BREP-08, consumed by S11 UI-22):** `TessMesh` carries:
  - f64 positions;
  - per-vertex normals;
  - triangles with **no vertex shared between faces**;
  - `face_ranges: Vec<(first_tri, tri_count)>` in in-shape face-index order;
  - `edges: Vec<Vec<DVec3>>` polylines in in-shape edge-index order;
  - vertex positions in in-shape vertex order.

  Optional per-vertex principal curvatures (for INS-05).
- **S11:**
  - `amg-app` uses `egui_wgpu::SurfaceConfig::HIGH_THROUGHPUT`, egui multisampling 0, no
    depth buffer;
  - UI-03 drives `nav::NavController` from its `NavPreset` tables and implements `PickService`
    over `request_pick`/`poll_picks`, with select-through (`AllAlongRay`) via a CPU ray cast;
  - UI-20/UI-22 follow §Design "Contract for the scene adapter";
  - the S11 overlay list (UI-09 sketches, UI-06 gizmos) maps 1:1 onto `OverlayList`;
  - **labels are egui-painted** (`project_labels` + `paint_labels`, with background) instead
    of the MSDF labels asked for in R-S10-7;
  - device-lost UX (save and restart).

## Work packages
| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| RND-01 | **Phase-0 spike: mesh spinning in an egui window.** `amg-render` skeleton (`Renderer`, `View` targets: Rgba16Float ×4 MSAA colour, Depth32Float reversed-Z, resolve, final Rgba8UnormSrgb), f64 `Camera` with RTE matrices, one Lambert+headlight pipeline (smooth normals, or flat via derivatives when normals are empty), `upload_mesh` (positions/normals/indices), `egui_glue::{install, with_renderer, Viewport}` with the `CallbackTrait` prepare/paint flow and sRGB-aware composite, `capture`. Example `crates/amg-render/examples/spin.rs` (eframe 0.37, `WgpuSetup::Existing` from `Gpu::new_blocking(GpuConfig::from_env())`): a displaced icosphere generator (`--tris N`, default 1 M), optional `--stl <file>` (binary STL, in-example reader), auto-turntable 30°/s, LMB-drag orbit, wheel dolly, side panel showing adapter, backend, triangles, CPU and GPU frame ms, an MSAA 1/4 toggle and a flat/smooth toggle; `--bench <frames>` prints p50/p99. Measures FXC vs `StaticDxc` core pipeline creation time. | FND-09 | P0 | Todo | 1. On the owner's RTX 5060 Ti / DX12, `cargo run -p amg-render --release --example spin -- --bench 600` at 1920×1080, 4× MSAA, 1 M tris prints frame-time p99 ≤ 16.7 ms (CPU wall) and GPU p50 ≤ 4 ms. The output is pasted into the PR. 2. `install` returns `DeviceMismatch` when given a different `Gpu` (test, WARP); with the same one, the example asserts `rs.device == *gpu.device()`. 3. `gpu_spin_smoke` (WARP in CI): a 256² capture of a sphere has a non-background centre pixel and background corners, and two captures are bit-identical. 4. RTE test: a 1 mm cube at the origin and at (1e5, 1e5, 0) mm, with the camera at the same relative pose, gives captures equal within 1 LSB per channel. 5. Debug build with validation: 10 s of continuous resizing plus MSAA toggling logs zero wgpu validation errors (recorded in the PR). 6. FXC vs StaticDxc timings and the resulting choice are recorded in the Refinement log. 7. fmt, clippy `-D warnings` and nextest pass. |
| RND-02 | Shading and visual-style framework: PBR-lite (GGX, studio key/fill + hemisphere), procedural matcap, oct-encoded normals, `normal` target, MSAA 1/4, optional FXAA, gradient background, pipeline variants via `override` constants and lazy variant creation on jobs, GPU timestamps (`gpu_timings`), `Material`. Bench `benches/viewport.rs` (headless). | RND-01 | P0 | Todo | 1. `cargo bench -p amg-render --bench viewport` (owner GPU, 2560×1440, 4× MSAA, Studio, a 5 M-tri displaced sphere, 600-frame orbit) reports GPU p99 ≤ 9 ms and wall p99 ≤ 16.6 ms; with 1 M tris, GPU p50 ≤ 3 ms. 2. Depth pre-pass on/off measured, and the faster one kept (recorded). 3. A flat-shaded mesh without normals uses exactly 12 B/vertex + 12 B/tri of VRAM (`gpu_bytes` test). 4. Headless tests (WARP): Lambert brightness at known normals matches the CPU reference within 2/255; MSAA 4 reduces edge-pixel aliasing (a staircase metric on a tilted quad); the core pipeline set is created in ≤ 150 ms on the owner GPU (logged). |
| RND-03 | `RenderScene`: slotmap keys, `Instance` storage buffer with dirty tracking, shared meshes across instances, layers, visibility, CPU frustum culling and front-to-back sort, multiple views, `upload_*` helpers with `ctx.checkpoint()` and `TooLarge`/`InvalidData` checks, `CameraUniform` + `shaders/camera.wgsl`, `ExternalLayer` composite pass, `visible_bounds`, `gpu_bytes`. | RND-01 | P0 | Todo | 1. Criterion: `encode` of 10 k instances over 100 meshes takes ≤ 2 ms p99 CPU. 2. Changing one transform rewrites exactly 128 B (upload counter). 3. `upload_mesh` of 5 M tris on a job thread ≤ 300 ms (owner machine); `insert_mesh` ≤ 50 µs. Cancelling mid-upload returns `Cancelled` within 20 ms. 4. ExternalLayer test: a synthetic layer whose depth plane cuts a cube composites with correct occlusion on both sides (pixel assertions). 5. Two views with different cameras render one scene in one frame. 6. 1000 add/remove cycles return `gpu_bytes` to baseline. 7. `InvalidData` for an index out of range and for a NaN position. |
| RND-04 | GPU picking: id + depth pick passes (B-rep face-id attribute; mesh `primitive_index` or vertex-pulling fallback; edges and points widened by the radius), scissored point pick, priority resolution, f64 world reconstruction, rectangle window/crossing via compute flags, hover/selection highlight (instance tint, face-range redraw with `depth_compare: Equal`, edge/point state buffers). | RND-12, RND-03 | P0 | Todo | 1. On a synthetic box-with-hole test part (edges known): 1000 random cursor positions within 3 px of a projected 1-px edge return that edge in ≥ 99.5 %, and positions > 6 px from every edge or vertex return no edge in ≥ 99.5 %. 2. 1000 random interior face pixels (≥ 2 px from edges) return the correct face (100 %), and mesh triangle sub-ids are correct (100 %). 3. `PrimitiveIndex` and `VertexPulling` give identical results on the same scenes (WARP). 4. Results arrive ≤ 2 `encode` calls after the request (counted). 5. World position is within 1e-6 × distance of the analytic hit. 6. The rectangle test on a 3×3 box grid gives the expected window vs crossing sets. 7. Highlighted faces show the highlight colour with no z-fighting pixels (pixel test). 8. The pick pass costs ≤ 2 ms GPU at 5 M tris (owner bench). |
| RND-05 | Transparency: WBOIT accum/revealage targets and composite, per-instance opacity (`base_color.a`), `Layer::Ghost`, x-ray style (all model instances at α 0.25 + dimmed always-on-top edges). | RND-02, RND-03 | P1 | Todo | 1. Three overlapping translucent quads rendered in all 6 submission orders differ by ≤ 2/255 per channel. 2. An opaque object behind and in front of translucent ones is correctly occluded. 3. X-ray shows hidden edges at the configured alpha. 4. Overhead ≤ 2 ms at 1440p with 1 M translucent tris (owner bench), and 0 when nothing is translucent (pass skipped). |
| RND-06 | Section planes: up to 4 planes, `discard` in every `clipped` model/edge/pick pipeline, back-face ray–plane caps with `frag_depth`, per-instance `cap_color`, screen-constant hatching, `closed` gating, cap picking (`CAP_SUB`). | RND-02, RND-03 | P0 | Todo | 1. Closed sphere r = 10 cut by z = 0, viewed from +Z: every pixel inside the projected circle shrunk by 1 px is cap or hatch colour, and none is background (watertight). 2. A cube cut by 2 orthogonal planes gives an L-shaped cap; pixel test against analytic polygons. 3. An open mesh draws no cap. 4. Hatch spacing is 8 ± 1 px at zooms 0.01×–100×. 5. Picking a clipped-away region returns the geometry behind it; picking a cap returns `CAP_SUB` with a world point on the plane (≤ 1e-6 mm). 6. Overhead ≤ 1 ms at 5 M tris (owner bench). |
| RND-07 | Overlay layer: `OverlayList` (polylines, arrows, rings, triads, plane quads, adaptive grid, image planes, points, sprites, pickable handles), constant pixel size, `DepthMode` incl. x-ray dimming, own overlay depth, `project_labels` + `egui_glue::paint_labels` (egui text), view-cube rendering hook for RND-11. | RND-12, RND-03 | P0 | Todo | 1. Labels: projected anchors match `Camera::project` within 0.5 px for 20 random cameras, and text is egui-rasterised (crisp at every zoom by construction). 2. Arrow and ring sizes stay within ±1 px of the requested size across view distances 1e-2..1e5 mm. 3. Handle picking returns `SubKind::Handle` with the handle id. 4. Grid minor lines are never closer than 8 px. 5. An image plane shows a test PNG with correct orientation (corner-pixel test). 6. Overlays cost ≤ 0.5 ms GPU with 1000 primitives. |
| RND-08 | Analyses and fields: `Shading::{Field, Analysis}`, zebra, draft and overhang shaders (note §11), scalar-field colormaps (Viridis, Turbo, Rainbow, BlueWhiteRed, Grey; LUT texture array), bands, isolines, out-of-range handling, `legend`, `colormap_rgb`, vector-field deformation with `Animation`. | RND-02, RND-03 | P0 | Todo | 1. A test-mode readback of a per-pixel class target: overhang classification on an icosphere matches the note §11 CPU formula for 100 % of pixels whose normal is > 0.5° from the threshold or plate band. 2. Zebra on a sphere shows N ± 1 stripes along the profile. 3. Draft sign is correct on a ±10° drafted prism. 4. LUT bytes equal the embedded reference tables, and `colormap_rgb` agrees with the shader within 1/255. 5. `legend` gives nice ticks (table test). 6. Deformed positions (test readback) match the CPU `p + s·A·d` within 1e-4 relative. 7. Switching shading never re-uploads geometry (counter). 8. A 5 M-tri mesh with field shading stays inside the RND-02 budget (bench). |
| RND-09 | Headless and visual regression: `assert_render_golden!` over `tests/golden/render/` (amg-testkit golden dir, `AMG_BLESS=1`), WARP-generated goldens, thresholds per note §15, diff and actual PNGs on failure, `thumbnail(scene, size) -> png bytes` with a Home-view fit. | RND-02 | P1 | Todo | 1. ≥ 10 golden scenes (6 visual styles, a section, a field, an analysis, overlays) pass in CI on WARP. 2. A deliberate 1-px camera shift fails with diff images written. 3. Bless round-trip works. 4. A 256² thumbnail of a 1 M-tri scene takes < 100 ms on the owner GPU. |
| RND-10 | Appearance materials: PBR textures (base, normal, roughness/metal), image-based environment lighting (prefiltered cube, BRDF LUT), mapping from S17 appearances | RND-02, MAT-02 | P2 | Todo | 1. A material-ball scene matches its golden image. 2. Assigning an S17 appearance changes the rendering with no geometry re-upload. 3. With IBL on, the 5 M-tri budget holds. |
| RND-11 | Navigation and view cube: `nav` module (orbit free/constrained about pivot, zoom-to-cursor, pan with grabbed point, fit, standard views, `look_at_plane`, projection switch, 300 ms transitions, near-plane policy), view-cube inset rendering (via RND-07) + `view_cube_hit` + face labels, `egui_glue::{view_cube, nav_bar}` emitting `CameraAction`. | RND-03, RND-07 | P0 | Todo | Unit tests (no GPU): 1. Orbit keeps the pivot pixel fixed (≤ 1e-6 px). 2. Zoom-to-cursor keeps the anchor pixel fixed (≤ 1e-6 px), perspective and ortho, over 1e-3..1e5 mm distances. 3. Pan keeps the grabbed point under the cursor (≤ 1e-6 px). 4. Fit: all 8 bound corners are inside the view, and the bounding sphere spans ≥ 90 % of the smaller viewport dimension. 5. Projection switch keeps apparent size at the target within 0.1 %. 6. Table test of all 26 cube regions + Home. 7. `look_at_plane` puts plane x to screen right and the normal toward the viewer (±1e-9). 8. Widget test (headless `egui::Context`): a click on each cube face region emits the matching `CameraAction`. 9. Headless (WARP): a 0.01 mm feature 1e4 mm from the origin, zoomed to fill the view, is stable within 0.5 px over a 360° orbit. |
| RND-12 | Line and point renderer: segment-quad lines with capsule SDF (width, AA, round joins), dashes, view-space depth offset, hidden-edge pass, point sprites, silhouettes post pass, edge-based visual styles (all six) using `POLYGON_MODE_LINE` for mesh wireframe with feature-edge fallback. | RND-02, RND-03 | P0 | Todo | 1. 1-px and 3-px polylines with 170° turns have no gaps (pixel connectivity test). 2. Edges on their own faces are visible for ≥ 99 % of edge pixels at view distances 1e-2..1e5 mm, and an edge 0.5 mm behind a face is hidden. 3. Hidden-edge styles show dashes at a known occluded edge. 4. Cylinder silhouettes are present (pixel test). 5. The RND-02 bench with 5 M tris + 100 k edge segments in ShadedVisibleEdges has GPU p99 ≤ 12 ms and wall p99 ≤ 16.6 ms on the owner GPU (**vision budget**). |
| RND-13 | Robustness and diagnostics: device-lost detection (`encode` → `Err(DeviceLost)`, `ViewportResponse::lost`, placeholder paint), an optional diagnostics overlay (pass timings, triangles, VRAM by category), and a validation-error capture to logs (`on_uncaptured_error` → tracing). | RND-03 | P1 | Todo | 1. `device.destroy()` in a test makes the next `encode` return `DeviceLost` with no panic, and `Viewport` paints the placeholder. 2. VRAM counters equal the sum of `gpu_bytes`. 3. A forced validation error appears in the log once, with its label. |
| RND-14 | Large-mesh scaling: meshopt clusters (≤ 128 tris) built in the upload job, GPU cluster frustum/back-face-cone culling + multi-draw-indirect (fallback: per-cluster CPU culling), buffer chunking beyond `max_buffer_size`, optional LOD (meshopt simplify) for distant instances | RND-02, RND-03 | P2 | Todo | 1. A 20 M-tri mesh orbits at GPU p99 ≤ 12 ms (2560×1440, 4× MSAA, owner GPU). 2. No regression > 5 % on the 5 M bench. 3. Picking is still exact (RND-04 tests pass on clustered meshes). |

**Notes on the table:**
- RND-01's dependency is **FND-09**, the shared `Gpu` (repointed from FND-04, as agreed in the
  S00 refinement). Its gate is ADR-004/005, cleared 2026-10-20.
- RND-01 replaces the old "spin a 1 M-tri mesh" goal with measurable criteria. Navigation and
  the view cube moved from RND-01 to the new **RND-11**, so the spike stays at 2–3 days.
- Edges, silhouettes and line rendering moved from RND-02 to the new **RND-12**. RND-02 keeps
  shading, MSAA and the bench. The 5 M-tri vision budget is proved twice: shading-only in
  RND-02, with edges in RND-12.
- RND-04 now depends on RND-12 (edge picking needs the line renderer) and RND-03.
- Text: MSDF dropped in favour of egui-painted labels (see Refinement log).

## Testing
- **Unit tests (no GPU):**
  - camera math: project/unproject round-trip ≤ 1e-9 relative, ray through pixel centres;
  - all `nav` behaviour (RND-11);
  - `legend` ticks, `colormap_rgb` tables;
  - pick resolution priority, on synthetic id/depth arrays;
  - `MeshData` validation;
  - instance dirty tracking.
- **GPU tests** (`gpu_*`, nextest group `gpu`):
  - run on WARP in CI (`AMG_WGPU_BACKEND=warp`, `AMG_REQUIRE_GPU=1`) and on DX12 locally;
  - pixel-assertion tests per WP (Done-when);
  - both pick backends forced via `RendererConfig::pick_backend`.
- **Golden images (RND-09):** `tests/golden/render/*.png`, blessed on WARP, thresholds in note
  §15.
- **Benchmarks** (owner machine; WARP is too slow, so CI runs them only with
  `cargo bench -- --test`):
  - `benches/viewport.rs`: 1 M / 5 M / 20 M tris, styles, fields, sections, picking;
  - `benches/scene.rs`: encode with N instances, upload throughput.

  Results go in the PR description. The vision row "Viewport, 5 M-triangle mesh, orbit
  ≥ 60 fps" maps to FND-08 `budget/viewport_5m`, implemented by the RND-02/RND-12 bench.
- **Determinism:** two captures of the same scene on WARP are bit-identical.

## Performance targets
Reference machine (vision doc: RTX 5060 Ti, DX12), release build. Primary resolution is
2560×1440 at 4× MSAA (owner question 1). Derivation: note §18.

| Scenario | Target |
|---|---|
| 5 M tris, ShadedVisibleEdges, orbit (**vision budget**) | GPU p99 ≤ 12 ms, wall p99 ≤ 16.6 ms (≥ 60 fps) |
| 5 M tris, Shaded only | GPU p99 ≤ 9 ms |
| 1 M tris, 1920×1080, 4× MSAA (RND-01 spike) | wall p99 ≤ 16.7 ms, GPU p50 ≤ 4 ms (≤ 3 ms after RND-02) |
| 20 M tris (RND-14) | GPU p99 ≤ 12 ms |
| CPU `encode`, 10 k instances | ≤ 2 ms p99 |
| `upload_mesh` 5 M tris (job thread) | ≤ 300 ms; UI insert ≤ 50 µs |
| Pick result latency / GPU cost at 5 M tris | ≤ 2 frames / ≤ 2 ms |
| Section caps, WBOIT, overlays (each, incremental) | ≤ 1 ms, ≤ 2 ms, ≤ 0.5 ms |
| Core pipeline creation at start-up (cold-start budget < 2 s) | ≤ 150 ms |
| VRAM per triangle of a closed smooth mesh | ≈ 20 B (16 B/vertex × V≈T/2 + 12 B/tri) |

## Open questions
1. **(owner)** Monitor resolution and refresh rate for the 60 fps budget. The targets assume
   2560×1440 at 4× MSAA. If the main monitor is 4K, either the 5 M budget is set at 4K with
   2× MSAA, or it stays at 1440p-equivalent. *Recommended:* measure at the owner's native
   resolution with 4× MSAA, and fall back to 2× MSAA at 4K if the target is missed.
2. **(owner, shared with S11 Open question 1)** egui 0.37 vs 0.36. egui_dock 0.21.1 requires
   egui ^0.36. egui-wgpu 0.36 and 0.37 both use wgpu 30 with an identical `CallbackTrait`, so
   `amg-render` works with either version, and RND-01 pins whatever S11 decides (S11 default:
   0.36.2 until egui_dock 0.22). The workspace has one egui version.
3. Resolved: DX12 has no `CLIP_DISTANCES`, so section planes use `discard`, and caps use a
   one-pass ray–plane back-face method (note §10).
4. Resolved: MSDF text is dropped for Phase 1; egui paints screen-space labels (note §12).
5. Resolved in RND-01 (measurement, not an owner decision): whether the FXC shader compiler is
   fast enough at start-up or `StaticDxc` is adopted (note §16).
6. Resolved: there is no automatic device re-creation after device loss. The app offers save
   and restart (note §17).

## Refinement log
**2026-10-20 — S10 refinement (lane D agent).**

*Researched (docs.rs, crates.io, release notes, papers; versions as of 2026-10-20):*
- **egui-wgpu 0.37.0** (2026-10-15, MSRV 1.95, `wgpu ^30`):
  - `CallbackTrait` (`prepare`/`finish_prepare`/`paint` with `RenderPass<'static>`), identical
    in 0.36.1;
  - `Callback::new_paint_callback`;
  - `WgpuSetup::{CreateNew, Existing}`, `WgpuSetupExisting { instance, adapter, device, queue }`;
  - `RenderState` fields (incl. `target_format`, `renderer: Arc<RwLock<Renderer>>`);
  - `RendererOptions` (msaa/depth/dithering/predictable);
  - `WgpuConfiguration { surface: SurfaceConfig, wgpu_setup, on_surface_status }`;
  - `Renderer::register_native_texture` (the fallback integration).
- **eframe 0.37.0:** `NativeOptions { renderer, wgpu_options, multisampling, depth_buffer, … }`,
  `CreationContext::wgpu_render_state`.
- **epaint:** `PaintCallbackInfo { viewport, clip_rect, pixels_per_point, screen_size_px }`.
- **wgpu 30.0.1** (2026-08-22):
  - feature backend table: `PRIMITIVE_INDEX` (DX12/Vulkan, perf caveat), `CLIP_DISTANCES`
    (not DX12), `SHADER_F64` (Vulkan only), `CONSERVATIVE_RASTERIZATION` (Vulkan only),
    `POLYGON_MODE_LINE`;
  - `Device` is `PartialEq + Send + Sync`;
  - `poll(PollType)`, device-lost callback, error scopes;
  - v30 breaking changes: optional vertex buffer layouts, explicit `@interpolate(flat)` for
    integer varyings, `Queue::present`, fallible `get_mapped_range`, `dispatch_workgroups`;
  - `Dx12Compiler` (FXC default; `StaticDxc` via `static-dxc` / `mach-dxcompiler-rs`).
- **Other crates:** egui_dock 0.21.1 requires egui ^0.36; `meshopt` 0.7.0, `png` 0.18.1,
  `fdsm` 0.8.0 licences.
- **Papers and prior art:**
  - McGuire & Bavoil 2013 (WBOIT);
  - Cigolle et al. 2014 (oct normals);
  - reversed-Z (Reed 2015);
  - relative-to-eye rendering;
  - Fusion visual styles, analyses and view cube behaviour.

*Decided:* see Design 1–12. Key choices:
- `CallbackTrait` offscreen flow over a native-texture image;
- RTE + reversed-Z;
- off-thread upload via `Send + Sync` device;
- no depth pre-pass by default;
- discard-based sections with one-pass caps;
- `Rg32Uint` id buffer with a vertex-pulling fallback;
- capsule-SDF wide lines;
- egui text instead of MSDF;
- analysis formulas as single definitions (note §11);
- world up +Z.

*Changed:*
- Status Draft → Refined.
- Scope, concrete Interfaces, Testing, Performance targets, Open questions added.
- Depends-on row is now explicit.
- The scene adapter is **not** in S10. A first draft of this refinement had adapter WPs, but
  S11 (refined in parallel) owns the adapter as UI-20/UI-22, so they were dropped and S10
  defines the contract instead (§Design). S11's requests R-S10-1..10 are answered there and in
  Interfaces:
  - highlight layers, preview layer, icon sprites with UV, filled polygons;
  - `Camera::view_matrix`/`proj_matrix`, `look_at_plane`, `view_cube`/`nav_bar` widgets;
  - device-lost flag, offscreen capture.

  Two deviations: labels are egui-painted rather than MSDF, and select-through is a CPU ray
  cast in S11's `PickService`, not an id-buffer feature. Navigation preset tables stay in S11
  (`NavPreset`).
- WPs:
  - RND-01 re-scoped as a concrete Phase-0 spike, dep FND-04 → **FND-09**;
  - RND-02 split (lines/edges/silhouettes → new **RND-12**);
  - RND-04 deps → RND-12, RND-03;
  - RND-05..08 gain RND-03 where they need scene features;
  - RND-07: MSDF → egui labels, plus image planes (IO-10) and grid (AM-03);
  - new **RND-11** (navigation + view cube + nav bar, moved out of RND-01), **RND-12**
    (lines/points/silhouettes), **RND-13** (device loss + diagnostics), **RND-14**
    (meshlets/LOD, P2).
- Every WP has a measurable Done-when.

*Design note created:* [`docs/notes/viewport-rendering.md`](../notes/viewport-rendering.md).

*Roadmap / cross-spec follow-ups (not edited here):*
- **Phase placement:**
  - Phase 0: RND-01;
  - Phase 1, lane D: RND-02, 03, 04, 06, 07, 08, 11, 12;
  - Phase 2: RND-13;
  - Phase 3: RND-05, RND-09 (RND-09 may move to Phase 2; it only needs RND-02);
  - Phase 5: RND-10, RND-14.

  No WP moves earlier than its deps.
- **Dependants to repoint (in their specs):**
  - UI-03 should also depend on RND-11;
  - AM-03 on RND-07 (grid/plate);
  - IO-10 on RND-07 (image planes);
  - IMP-05 keeps RND-03 (`ExternalLayer`).
- Requests to S00, S02, S07 and S11: §Interfaces "Needed from other specs".

## Interface changes

### 2026-10-20: requests from the wave-2 refinements (S02, S05, S10, S11, S12, S16, S17, S20)

Additive requests recorded by the integration pass; the owning lane confirms or adjusts them when it implements the affected WP.

- **From S05 (inspection).**
  - RND-06: S05 asks for up to **6** planes (box sections from combined section analyses;
    QUESTIONS #29) instead of 4; per-body scope (a section affects only selected bodies); hatch
    styles none / 45° lines / cross with spacing in px and angle; cap colour = body colour or
    uniform; outline polylines. S05 INS-02 computes geometric caps on the CPU for section stats;
    whether RND-06 also displays those CPU cap meshes (plane-local uv in mm) or only its own GPU
    caps is lane D's call.
  - RND-04: a pick returns the mesh face index (primitive id), the world-space f64 pick ray and
    world units per pixel at the hit depth (snap radii).
  - RND-07: measurement dimension overlay (witness lines, markers, leader + label, always on top),
    angle arc and circle display, centre-of-mass glyph, construction-geometry display (plane
    squares, axis segments, point markers) with pick ids, curvature-comb polylines.
  - RND-08: accept 0/1 fields and NaN (drawn grey), zebra stripe controls, per-body overlay enable.
- **From S16 (AM).**
  - RND-08 overhang mode per body instance with uniforms (f32, body-local) `build_dir`,
    `cos_critical`, `contact_plane: vec4`, `contact_tolerance`, and colours for supported /
    overhang ramp (severity = clamp((c − cos α)/(1 − cos α), 0, 1)) / contact; flat face normal.
    The rule in `docs/notes/viewport-rendering.md` §11 must match `docs/notes/am-build-prep.md`
    §1.2–1.3 (`−n·d > cos α`, strict); the AM-02 golden file
    `tests/golden/am/overhang_classes.toml` (5 000 cases) is the arbiter.
  - Per-face scalar or class buffer indexed by `primitive_index`, so the exact CPU classes can be
    shown after the fine job (per-vertex scalars stay as fallback).
  - RND-03/RND-07 helper items: non-pickable meshes (flat colour + alpha), polyline sets (colour,
    width) and a per-instance tint colour, for plate, grid, volume box and exclude areas.
  - Benchmark: overhang overlay on, 5 M triangles, ≥ 60 fps (AM-12 Done-when).
  - S10's request that AM-03 depend on RND-07 is not applied: AM-03 is placement maths in
    `amg-am`; AM-11 (the UI that draws the plate) already depends on RND-07.
- **From S12 (import/export).** IO-10 now depends on RND-07 (`image_plane`); applied in S12's WP
  table. IO-15 writes 3MF thumbnails through `Renderer::capture` once RND-09 lands (Phase 3);
  until then no thumbnail is written.
- **From S17 (materials).** RND-10 depends on MAT-02 (unchanged). `amg-render` receives plain
  linear floats via the `amg-ui::materials` adapter, with no crate dependency.
