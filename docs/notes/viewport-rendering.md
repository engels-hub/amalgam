# Viewport rendering: egui integration, frame graph, precision, picking, analysis shaders

Design note for [S10 — Rendering & viewport](../specs/S10-rendering.md). Written during S10
refinement (2026-10-20). The spec holds the decisions, WPs and the API. This note holds the
evidence, the formulas and the per-pass detail that implementing agents need.

Pinned versions (2026-10-20): **wgpu 30.0.1**, **egui / egui-wgpu / eframe 0.37.0** (released
2026-10-15; egui-wgpu 0.36 and 0.37 both require `wgpu ^30`).

## 1. Integration with egui-wgpu 0.37 (APIs checked on docs.rs)

### 1.1 One device
`amg-app` creates `amg_jobs::gpu::Gpu` first (FND-09). It ORs `amg_render::OPTIONAL_FEATURES`
into `GpuConfig::optional_features`, then hands the same handles to eframe:

```rust
let gpu = Gpu::new_blocking(&cfg)?;                       // before the event loop (allowed)
let setup = egui_wgpu::WgpuSetup::Existing(egui_wgpu::WgpuSetupExisting {
    instance: gpu.instance().clone(), adapter: gpu.adapter().clone(),
    device: gpu.device().clone(),     queue: gpu.queue().clone(),
});
let native = eframe::NativeOptions {
    renderer: eframe::Renderer::Wgpu,
    wgpu_options: egui_wgpu::WgpuConfiguration { wgpu_setup: setup, ..Default::default() },
    multisampling: 0, depth_buffer: 0, stencil_buffer: 0,  // egui's own pass: no MSAA, no depth
    ..Default::default()
};
// In the app creator: cc.wgpu_render_state: Option<egui_wgpu::RenderState>
amg_render::egui_glue::install(cc.wgpu_render_state.as_ref().unwrap(), gpu.clone(), &rcfg, 1)?;
```

Verified facts:
- `WgpuSetupExisting` has exactly four public fields: `instance`, `adapter`, `device`, `queue`
  (wgpu 30 types).
- `RenderState` has the public fields `adapter`, `available_adapters`, `instance`, `device`,
  `queue`, `target_format: wgpu::TextureFormat` and `renderer: Arc<RwLock<egui_wgpu::Renderer>>`.
- `wgpu::Device` implements `PartialEq`/`Eq`/`Hash` and is `Send + Sync`. `install` checks
  `render_state.device == *gpu.device()` and fails with `RenderError::DeviceMismatch` otherwise.
- `WgpuConfiguration` has the fields `surface: SurfaceConfig { present_mode,
  desired_maximum_frame_latency }`, `wgpu_setup` and `on_surface_status`.
  `SurfaceConfig::HIGH_THROUGHPUT` is documented as the preset for "GUIs with substantial extra
  GPU work", which is us. `amg-app` uses it (S11 request).
- `RendererOptions { msaa_samples, depth_stencil_format, dithering, predictable_texture_filtering }`.
  egui's pass stays single-sample with no depth. The viewport has its own targets (§2).

### 1.2 Paint callback
`egui_wgpu::CallbackTrait` (0.37, unchanged from 0.36):

```rust
pub trait CallbackTrait: Send + Sync {
    fn prepare(&self, device: &Device, queue: &Queue, screen: &ScreenDescriptor,
               egui_encoder: &mut CommandEncoder, res: &mut CallbackResources) -> Vec<CommandBuffer> { .. }
    fn finish_prepare(&self, device: &Device, queue: &Queue,
               egui_encoder: &mut CommandEncoder, res: &mut CallbackResources) -> Vec<CommandBuffer> { .. }
    fn paint(&self, info: PaintCallbackInfo, render_pass: &mut RenderPass<'static>, res: &CallbackResources);
}
// egui_wgpu::Callback::new_paint_callback(rect: egui::Rect, cb: impl CallbackTrait + 'static) -> egui::PaintCallback
// PaintCallbackInfo { viewport, clip_rect, pixels_per_point, screen_size_px }, viewport_in_pixels(), clip_rect_in_pixels()
```

`prepare` runs for every callback before egui's render pass. `paint` draws into egui's own
render pass. `Renderer::callback_resources` is a type map shared by all callbacks.

**Chosen flow, per viewport and per frame:**
1. `Viewport::show(ui, rs)` allocates the rect and computes `size_px = round(rect.size() ×
   pixels_per_point)`. It adds `ViewportCallback { view, size_px }` via `new_paint_callback`.
2. `prepare` gets the `amg_render::Renderer` from `callback_resources`, resizes the view
   targets if needed, and calls `Renderer::encode(view, egui_encoder)`. That records every 3D
   pass into the view's offscreen targets and ends with the final single-sample
   `Rgba8UnormSrgb` image. It also records pending pick passes and readback copies.
3. `paint` sets the viewport and scissor from `info.viewport_in_pixels()` and
   `clip_rect_in_pixels()`. It then draws one fullscreen triangle that copies the final image
   1:1 (nearest `textureLoad`). The composite pipeline is built for `rs.target_format` and
   egui's sample count.

**Why not `register_native_texture` + `egui::Image`?** It works too (rendering would move into
`update()` with our own encoder and `queue.submit`). The callback keeps three advantages:
- rendering is sequenced inside egui's frame, so no extra submit is needed;
- the target size tracks the exact pixel rect, so there's no re-registration on resize;
- `paint` respects egui's clip rect.

The native-texture route is the documented fallback if a future egui-wgpu changes the trait.

**sRGB.** egui-wgpu prefers a non-sRGB framebuffer format (`preferred_framebuffer_format`) and
blends in gamma space. Our final image is already sRGB-encoded. If `target_format.is_srgb()`
the composite shader decodes it to linear first, so the hardware re-encode is a no-op round
trip. Otherwise it passes the encoded value straight through. A test covers both formats.

**Locking.** `RenderState::renderer` is an `epaint` `RwLock`. The UI thread takes the write
lock briefly during `update()`, through `egui_glue::with_renderer`, to mutate the
`RenderScene` (insert finished uploads, set transforms, highlights, camera). eframe calls
`update` and paint on the same thread, so there's no contention. Heavy work never happens
under the lock (§5).

**Several viewports.** Each `ViewId` owns its own targets and camera, and all views share one
`RenderScene`. Fusion's split view (four viewports) is just four `Viewport` widgets.

### 1.3 wgpu 30 changes that affect this crate
From the v30.0.0 release notes:
- `VertexState::buffers` is `&[Option<VertexBufferLayout>]`.
- Integer shader I/O **no longer defaults to flat**. Every `u32` varying (pick ids, face ids)
  needs an explicit `@interpolate(flat)`.
- `SurfaceTexture::present()` became `Queue::present(..)`. eframe handles the surface, so this
  only matters for a custom winit loop.
- `Buffer::get_mapped_range*` returns `Result<_, MapRangeError>`. The FND-09 `Readback` maps
  that error.
- `dispatch` is now `dispatch_workgroups`.
- `RenderPipelineDescriptor` now has `multiview_mask: Option<NonZeroU32>` (we pass `None`)
  and `cache: Option<&PipelineCache>`.
- `Device::poll(PollType) -> Result<PollStatus, PollError>`;
  `set_device_lost_callback(Fn(DeviceLostReason, String))`;
  `push_error_scope(..) -> ErrorScopeGuard`.

Feature availability (wgpu 30 docs), which drives several decisions:

| Feature | Backends | Used for |
|---|---|---|
| `PRIMITIVE_INDEX` | Vulkan (geometryShader), DX12, Metal/GL partial; "may hurt performance" | pick pass only (§7) |
| `POLYGON_MODE_LINE` | DX12, Vulkan, Metal | wireframe of triangle meshes |
| `TIMESTAMP_QUERY` | all | GPU pass timings, benches |
| `CLIP_DISTANCES` | Vulkan, Metal, GL, **not DX12** | not used: section planes use `discard` (§10) |
| `CONSERVATIVE_RASTERIZATION` | Vulkan only | not used |
| `SHADER_F64` | Vulkan only | not used by the renderer |

`OPTIONAL_FEATURES = PRIMITIVE_INDEX | POLYGON_MODE_LINE | TIMESTAMP_QUERY`. All are optional
and each has a fallback.

## 2. Frame graph

Per view (sizes in device pixels; *S* = MSAA sample count, default 4):

| # | Pass | Targets | Notes |
|---|---|---|---|
| 1 | Opaque | `color` Rgba16Float ×S, `normal` Rg16Float ×S (oct-encoded view normal), `depth` Depth32Float ×S (reversed-Z) | Front-to-back instance order. No depth pre-pass (decision §18). |
| 2 | Section caps | same | Back-face cap pipeline writes `frag_depth` (§10). |
| 3 | External layer | same | Fullscreen, depth-tested composite of an `ExternalLayer` (IMP-05 ray-march) (§14). |
| 4 | Edges and points | `color`, `depth` (test, no write) | Wide-line quads with view-space depth offset; the hidden-edge variant uses `depth_compare: Less` and a dash pattern (§6). |
| 5 | Transparent (WBOIT) | `accum` Rgba16Float ×S, `reveal` R8Unorm ×S; depth test, no write | §9. Skipped when nothing is translucent. |
| 6 | Resolve + post | `resolved` Rgba16Float ×1 | Resolve, WBOIT composite, silhouettes (from sample 0 of `normal`/`depth`), optional FXAA. |
| 7 | Overlays | `final` Rgba8UnormSrgb ×1, own `overlay_depth` Depth32Float ×1 | Gizmos, grid, handles, view cube inset. Depth is cleared, and an x-ray mode samples the scene depth for dimming. |
| — | Pick (on demand) | `pick_id` Rg32Uint ×1, `pick_depth` R32Float ×1, scissored to the request | §7. |

Reversed-Z throughout: clear depth 0, `depth_compare: GreaterEqual`. Perspective uses an
infinite far plane.

## 3. Precision: f64 model, f32 GPU

- **Relative-to-eye (RTE).** Uploads are rebased to a per-mesh `origin` (S07 `RenderBuffers`
  does this). Each frame, per instance, the CPU computes in f64
  `M = T(−eye) · world_from_local · T(origin)` and casts it to f32 via
  `amg_core::gpu::transform_to_f32`. The view matrix holds rotation only. A vertex therefore
  reaches the shader as a small eye-relative f32 offset, whatever its distance from the world
  origin: a 1 mm part 10⁵ mm from the origin renders exactly like one at the origin (RND-01
  test).
- **Depth.** Reversed-Z with Depth32Float gives near-uniform relative precision. With
  near = 10⁻³ × distance(eye, pivot), clamped to ≥ 10⁻⁴ mm, there's no visible z-fighting from
  1 µm features to 1 m build plates. Orthographic near/far come from the visible scene bounds
  projected on the view direction, plus a 1 % margin.
- **Picking world position.** This is reconstructed in f64 from the pick depth and the f64
  camera (`Camera::unproject`), not from f32 matrices. Error is ≤ 1 depth ulp ≈ 10⁻⁷ × distance.

## 4. Shading

- **PBR-lite:**
  - two camera-locked directional "studio" lights (key and fill) plus a hemisphere ambient
    (sky/ground colours, world up = +Z);
  - GGX specular with Schlick Fresnel; the material is `base_color`, `metallic`, `roughness`;
  - no shadows or IBL (IBL comes in RND-10).
- **Matcap.** An optional procedural matcap (computed in the shader, no image assets) gives
  the "clay" look many mesh users prefer.
- **Flat vs smooth.** If a mesh comes without normals (`normals` empty), the fragment shader
  computes `normalize(cross(dpdx(p_view), dpdy(p_view)))`. This gives flat STL-style faceting
  without duplicating vertices. With normals, vertex normals are oct-encoded to `Snorm16x2`.
- **Pipeline variants.** These use WGSL `override` constants (bool/u32 flags): style, shading
  kind, flat or smooth, clipped. No preprocessor crate is needed. Shared code is assembled
  from `include_str!` snippets.

## 5. Vertex layout, memory, upload

| Stream | Format | Bytes |
|---|---|---|
| position (rebased) | `Float32x3` | 12 / vertex |
| normal (optional) | `Snorm16x2` octahedral | 4 / vertex |
| scalar field (optional, RND-08) | `Float32` | 4 / vertex |
| displacement (optional, RND-08) | `Float32x3` | 12 / vertex |
| face id (B-rep only) | `Uint32`, `@interpolate(flat)` | 4 / vertex |
| indices | `Uint32` | 12 / triangle |

A closed 5 M-triangle mesh has ≈ 2.5 M vertices: 40 MB of vertices + 60 MB of indices
≈ **100 MB VRAM**. The pick fallback adds nothing (§7). B-rep tessellations don't share
vertices across faces (BREP-08 per-face ranges), so a per-vertex face id is exact. For meshes,
the sub-id is the triangle index.

**Upload is never on the UI thread.**
1. The S11 scene adapter (UI-20/UI-22, `BodyTessellator`) runs an Interactive job:
   - `IndexedMesh::render_buffers` (S07: 20 ms at 1 M, 250 ms at 10 M);
   - `MeshData`;
   - `amg_render::upload_mesh(&gpu, &data, ctx)`, which creates buffers with
     `mapped_at_creation` and copies in chunks with `ctx.checkpoint()` between chunks.
   `wgpu::Device` is `Send + Sync`, so this is legal.
2. The job returns a `GpuMesh`, and the UI thread only does `scene.insert_mesh(gpu_mesh)`
   (≤ 50 µs).

Freed meshes are dropped immediately. wgpu keeps the buffers alive until in-flight submissions
finish.

**Limits.** `upload_mesh` returns `RenderError::TooLarge` when a buffer would exceed
`device.limits().max_buffer_size`. DX12 adapters report far more than the 256 MiB WebGPU
default, so 5 M (and 20 M) triangles fit. RND-14 adds chunked clusters beyond that.

## 6. Lines, points, edges, silhouettes

- **Wide lines.** Every segment is an instance of a 4-vertex quad. It is expanded in screen
  space to `width/2 + 1 px` feather on each side and extended by the half-width at both ends.
  The fragment shader evaluates the capsule SDF, so joins are round and gap-free at any turn
  angle, with analytic AA from the distance. Native wgpu lines (1 px, no AA, no width) are
  never used.
- **Depth offset.** In the vertex shader, both segment ends move toward the eye by
  `ε = k_px × pixel_size_at(depth)` (`k_px = 1.5`, where pixel size =
  `2·d·tan(fov/2)/height_px`, or `ortho_height/height_px`). This scales with distance, unlike
  constant NDC bias, so edges lying on their faces win the depth test from 10⁻² to 10⁵ mm view
  distance without poking through nearby faces.
- **Dashes.** A per-vertex `arc_len` (mm, f32) is converted to pixels with the pixel size at
  that depth, so the pattern is `on_px`/`off_px` in screen space (approximate under strong
  perspective, which is acceptable).
- **Hidden edges.** These are drawn in a second edge draw with `depth_compare: Less`
  (reversed-Z: behind the surface), dashed, at 40 % alpha. This gives the Fusion styles
  "Shaded with Hidden Edges" and "Wireframe with Hidden Edges".
- **Points** are instanced quads with a circle SDF. B-rep vertices and sketch points use them.
- **Silhouettes** come from a post pass on sample 0 of `normal` and `depth`, at resolved
  resolution, using Roberts-cross differences. An outline pixel is one where the normal angle
  difference is > 35° or the linear-depth jump is > 2 % of depth. Silhouettes mostly matter
  for meshes and curved B-rep faces, whose silhouette is not a model edge.
- **Mesh "edges".** The renderer shows whatever `EdgeData` the adapter gives it. For mesh
  bodies, the adapter supplies boundary edges, non-manifold edges and face-group borders
  (S07 request `feature_edges`). A full triangle wireframe of a mesh uses `POLYGON_MODE_LINE`
  (Wireframe style); without the feature, the wireframe styles fall back to feature edges only.

## 7. Picking

**Id texel.** `Rg32Uint`:
- `r` = `PickId` (u32, dense, allocated by the adapter; 0 = nothing);
- `g` = `kind << 30 | sub`, where kind is 0 face/triangle, 1 edge, 2 vertex, 3 overlay
  handle, and `sub < 2³⁰`.

Section caps report kind 0 with `sub = CAP_SUB = 2³⁰ − 1`. `pick_depth` (R32Float) holds the
reversed-Z depth.

**Face sub-id in the pick shader:**
- **B-rep:** the per-vertex flat face id (§5). No feature is needed.
- **Mesh with `PRIMITIVE_INDEX`:** `@builtin(primitive_index)` equals the triangle index,
  because `render_buffers` preserves face order. The builtin is only used in the pick
  pipeline; the docs warn it can enable geometry processing.
- **Mesh without it (fallback "vertex pulling"):** a non-indexed draw of `3·T` vertices. The
  vertex shader reads `indices[vertex_index]` and `positions[..]` from storage buffers and
  passes `vertex_index / 3` flat. The buffers get `STORAGE` usage at upload when the fallback
  is active. Index storage bindings must be ≤ `max_storage_buffer_binding_size`; a 5 M-tri
  index buffer is 60 MB, under the 128 MiB default.

`RendererConfig::pick_backend` can force either path, so tests run both on WARP.

**Point pick.**
1. `request_pick(view, Point { px, radius_px: r })` records the pick pass on the next encode.
   The pass is scissored to the `(2r+1)²` square and has the full viewport transform. It draws
   faces, then edges widened by `r` px, then points widened by `r` px, with depth test on.
2. A copy to a staging buffer goes through `Gpu::readback`.
3. `poll_picks()` returns the result when it's mapped: **≤ 2 frames** after the request.
4. CPU resolution: the nearest vertex texel within `r_v = r` wins. Otherwise the nearest edge
   texel within `r_e = r − 1` wins. Otherwise the face under the centre texel.
5. Hit positions are reconstructed in f64 (§3).

During camera drags, hover picks are suppressed, so picking never runs while orbiting.

**Rectangle select.**
1. The request renders the full-view pick pass.
2. A compute pass walks every id texel and does `atomicOr(flags[offset[pick] + sub],
   inside ? 1 : 2)`. Offsets are per `PickId` and are sized for the requested sub-kind (bodies,
   faces, edges, vertices). Triangle-level flags (4 B/triangle) are allocated only when the
   filter asks for mesh triangles.
3. The flags are read back.
4. Window = `flags == 1`, i.e. visible pixels only inside the rectangle. Crossing =
   `flags & 1`.

Only visible pixels count: occluded entities are not box-selected. That matches Fusion's
default and the visible-only semantics of S11 UI-04.

**Highlight.**
- Whole-instance hover/selection is a tint plus an edge colour change via instance flags.
- Sub-entity highlight builds, per instance, a small index buffer of the highlighted triangle
  ranges (B-rep face ranges, mesh triangle or face-group lists). It is redrawn after the
  opaque pass with `depth_compare: Equal` and the same vertex transform, so there's no z-fight.
- Edge and point highlight uses a per-instance state buffer (u8 packed into u32) indexed by
  edge or point id.

## 8. Instances and culling
- Per-instance data lives in one storage buffer (128 B: RTE matrix 64, normal matrix 36 padded
  to 48, pick id, flags, material index), indexed by `instance_index` (set through
  `first_instance` of direct draws, core).
- Only dirty instances are rewritten (`queue.write_buffer` of 128 B each).
- CPU frustum culling uses world bounding spheres. Draws are sorted by pipeline, then
  front-to-back.
- 10 k instances → ≤ 2 ms CPU `encode`. Multi-draw-indirect GPU culling is RND-14 territory.

## 9. Transparency: weighted blended OIT (McGuire & Bavoil, JCGT 2013)
- Accumulation: `accum += (c·a·w, a·w)` with blend `One, One` on Rgba16Float.
- Revealage: `reveal *= (1−a)` with blend `Zero, OneMinusSrc` on R8Unorm.
- Weight: `w = clamp(10 / (1e-5 + (z/5)⁴ + (z/200)⁶), 1e-2, 3e3)`, eq. 9 of the paper. Here
  `z = view depth / scene_radius × 100`, so the curve sees CAD scales normalised.
- Composite in post: `rgb = accum.rgb / max(accum.a, 1e-5)` over the opaque colour with
  `α = 1 − reveal`.
- Order-independent by construction, which the RND-05 test checks (permutations).
- **X-ray** renders every model instance through this path at α = 0.25, with edges drawn
  always-on-top at 60 %.
- **Ghost** layer (previews of removed geometry, S14 keep/remove overlays) uses this path too.

## 10. Section planes and caps
DX12 has no `CLIP_DISTANCES`, so every model pipeline with the `clipped` override does
`if any(dot(n_k, p) − d_k > 0) { discard; }` for up to 4 planes. The planes are a uniform, in
eye-relative coordinates.

**Caps without stencil.** For a closed body cut by the planes, any visible back-face fragment
means we're looking into the cut interior. The cap pipeline draws the body's back faces
(`cull_mode: Front`) with the same clipping. Per fragment:
1. Take the view ray `o + t·v`, where `o` is the eye in RTE space (zero) and `v` points at the
   fragment.
2. The kept region is ∩ₖ {`n_k·x ≤ d_k`}. Only planes with `n_k·o > d_k` (the eye is on their
   cut side) can be entered by the ray. The ray enters the kept region at
   `t_in = max over those planes of (d_k − n_k·o)/(n_k·v)`. If no plane qualifies, the eye is
   already inside the kept region and there is no cap for this fragment.
3. If `t_in` < the back-face `t`, the cap point is `x = o + t_in·v` on plane `k*`, the argmax.
   Output the cap colour (instance `cap_color`, default darkened base colour) with a hatch
   pattern computed in plane `k*`'s 2D frame (`amg_core::Plane` x/y axes). Write
   `@builtin(frag_depth)` = depth of `x`.
4. Otherwise the eye is inside the body, or the body is open: draw the plain back face with
   flipped normal.

This needs one pass, no stencil, any number of bodies, a per-body cap colour and correct depth,
so edges and picking work on caps. `frag_depth` disables early-Z only for this pipeline. Caps
are drawn only for instances flagged `closed` by the adapter (watertight meshes and solids).
Open meshes show back faces and no cap.

**Hatch.** Lines at angle θ, spacing in screen px:
`h = fract(dot(x_plane, dir_perp) / (spacing_px · pixel_size))`, AA via `fwidth`. Spacing stays
constant on screen across zoom.

**Pick.** The pick variant of the cap pipeline writes kind 0 with `sub = CAP_SUB`.

## 11. Analysis shaders and scalar fields (RND-08)

All formulas are in world space with unit normal `n` (flipped for back faces). One definition
is used everywhere; S16 AM-02 and S05 INS-05 implement their CPU counterparts with these exact
formulas, and an `amg-ui` test cross-checks them.

- **Zebra.** `r = reflect(−v, n)`, axis `a` = world up (horizontal stripes) or camera right
  (vertical stripes). View-locked uses `v`, `a` in view space; world-locked uses world space.
  `t = acos(clamp(dot(r, a), −1, 1)) / π × N`. Stripe = `step(0.5, fract(t))`, with AA via
  `fwidth(t)`. Continuity of stripes across an edge shows G1 and G2 continuity, as in Fusion's
  zebra.
- **Draft.** `β = asin(clamp(dot(n, p), −1, 1))`, where `p` is the pull direction. Colours on a
  diverging map over `[−β_max, +β_max]`. A band highlights `|β| < β_req` (insufficient draft),
  and the sign gives positive or negative draft.
- **Overhang** (S16 semantics, owned by AM-02):
  - `b` = build up direction, `α` = critical self-supporting angle measured from horizontal
    (FFF default 45°, resin set by AM-01 profiles).
  - Down-facing means `n·b < 0`. The surface inclination from horizontal is
    `φ = acos(−n·b)`: 0 for a ceiling, 90° for a vertical wall.
  - Overhang ⇔ `n·b < 0 ∧ φ < α`. Severity is `(α − φ)/α`, shown on a red ramp.
  - Fragments with height `dot(x, b) ≤ plate_height + plate_tol` are plate contact and get a
    neutral colour.
  - All of this is computed live in the shader from `b`, so rotating the part
    (AM-02 "live with orientation") costs no re-upload.
- **Curvature map / thickness / FEA results.** These are per-vertex scalar fields from the
  domain:
  - curvature: S07 mesh request, S02 B-rep request;
  - thickness: MESH-11;
  - stress and displacement: FEM-06.

  `t = (v − min)/(max − min)` goes through a colormap LUT (256×1 `Rgba8UnormSrgb` texture array,
  one layer per map). Optional bands: `t' = (floor(t·N) + 0.5)/N`. Optional isolines:
  `line = 1 − smoothstep(0, 1.5·fwidth(t·N), min(fract(t·N), 1 − fract(t·N)))`. Out-of-range
  values are clamped or get a fixed colour (grey or transparent).
- **Deformation.** `p' = p + s · A(τ) · d`, where `d` is the per-vertex displacement and
  `A(τ) = sin(2πτ/T)` (mode shapes) or `A = min(τ/T, 1)` (static ramp) or `A = 1`. Normals are
  not recomputed: shading on very large displacements is approximate, which Fusion accepts too.
  The scale `s` is a uniform, so the animation costs one uniform write per frame.
- **Colormaps:**
  - `Viridis` (matplotlib, CC0) and `Turbo` (Google, Apache-2.0) are embedded as 256-entry
    tables.
  - `Rainbow` (Fusion-like blue→cyan→green→yellow→red), `BlueWhiteRed` (diverging) and `Grey`
    are defined in code.
  - Default for FEA is `Rainbow` (Fusion users expect it); default for analyses is `Viridis`.
- **Legend.** A pure function returns ticks at "nice" numbers (1, 2, 2.5, 5 × 10ᵏ) inside
  `[min, max]`, plus the map and band count. `amg-ui` paints it with egui, so the renderer
  draws no 2D UI.

## 12. Overlays and text
- **Gizmos and helpers** (RND-07) are an immediate-mode `OverlayList` rebuilt each frame by the
  active tool: arrows, rings, triads, plane quads, adaptive grid, image planes, sprites and
  pickable handles.
- **Constant pixel size.** The scale is `size_px × pixel_size(depth of anchor)`.
- **Text.** All on-canvas text (dimension values, labels, view-cube letters, legend) is
  **screen-space text painted by egui** at anchors the renderer projects
  (`Renderer::project_labels`). egui rasterises glyphs at the exact pixel size, so text is
  crisp at every zoom with no font pipeline. Fusion's dimensions are screen-space, not
  depth-tested, so nothing is lost.
- **MSDF rejected for now.** MSDF (`fdsm` 0.8, MIT) is the option kept for any later
  depth-tested 3D text; it would add `image` + `nalgebra` deps and a glyph atlas for no Phase 1
  benefit.
- **Grid.** World-plane grid lines in the fragment shader (`fwidth`-based AA). Minor/major
  spacing is chosen per zoom so minor lines are ≥ 8 px apart, and there is a distance fade.
  The build-plate rectangle for AM-03 is an overlay quad plus border lines.

## 13. Navigation and view cube (RND-11)
World up is **+Z**, the AM build direction. Front view looks along +Y with the camera at −Y.
Home is the iso view from direction `(1, −1, 1)`.

- **Orbit** about the pivot `P`, which is the pick point under the cursor at drag start (from
  the pick depth), or the scene bounds centre when the cursor is on background.
  - Free orbit: yaw about world Z, pitch about camera right.
  - Constrained (turntable) orbit: the same with pitch clamped to ±89.9°.
  - Angle = 180° per viewport height of drag.
  - Rotating eye and target about `P` keeps `P` on the same pixel.
- **Zoom to cursor** around the world point `W` under the cursor (pick depth, or the target
  plane on background).
  - Perspective: `eye' = W + (eye − W)·f` with `f = 0.9^steps`. The orientation is unchanged,
    so `W` stays on the same view ray, which means the same pixel exactly.
  - Orthographic: `height' = height·f` and `target' = W + (target − W)·f` in the view plane.
- **Pan.** The grabbed point `G` (pick depth at drag start) stays under the cursor:
  `Δ = G − unproject(cursor_now, depth_G)`, and eye and target move by `Δ`.
- **Fit.** Bounding sphere `(c, r)` of visible bounds. Perspective distance =
  `1.05·r / sin(min(fov_x, fov_y)/2)`. Orthographic height = `2.1·r / min(1, aspect)`.
- **Projection switch.** `ortho_height = 2·|eye − target|·tan(fov_y/2)`, so the apparent size
  at the target is preserved.
- **Transitions** (standard views, fit, view cube) take 300 ms with smoothstep easing:
  slerp the orientation quaternion, lerp the target, lerp the log of the distance or height.
- **Presets** are S11's `NavPreset` tables (UI-03/UI-11), which call `NavController`. They're
  listed here for reference:
  - Fusion: MMB pan, Shift+MMB orbit, wheel zoom; the default.
  - SolidWorks: MMB orbit, Ctrl+MMB pan, wheel zoom.
  - Blender: MMB orbit, Shift+MMB pan, Ctrl+MMB zoom.
  - Wheel direction is invertible.
- **View cube.** A 110 logical-px inset, top-right, drawn in the overlay pass with an
  orthographic camera that shares the view's orientation.
  - Hit test is analytic: ray against a cube `[−1, 1]³`. The hit point's coordinates with
    `|c| > 0.6` mark the faces, edges or corners it touches, giving 26 regions.
  - Click animates to the region's view. Drag orbits. A home button returns to Home.
  - Face letters are egui text at the projected face centre, shown when `n·view > 0.25`.

## 14. External layer contract (for IMP-05 ray marching)
An `ExternalLayer` has two textures at the view size:
- `color`: Rgba16Float, premultiplied linear;
- `depth`: R32Float, holding the **reversed-Z NDC depth of this view's projection**, 0 = empty.

The producer (amg-implicit WGSL compute on the shared `Gpu`, wired by `amg-ui`) gets the
camera as a plain `CameraUniform` (`#[repr(C)]`, `bytemuck::Pod`). Its WGSL struct is fixed in
`amg-render/shaders/camera.wgsl`, and IMP-05 copies that layout, since amg-implicit cannot
depend on amg-render:

```wgsl
struct Camera {
  view_rte: mat4x4<f32>,       // rotation only (eye at origin)
  proj: mat4x4<f32>,           // reversed-Z, infinite far for perspective
  inv_view_proj_rte: mat4x4<f32>,
  eye_hi: vec3<f32>, _p0: f32, // eye world position split hi/lo (f64 → 2×f32)
  eye_lo: vec3<f32>, _p1: f32,
  size_px: vec2<f32>, pixel_size_at_1mm: f32, ortho: u32,
};
```

Pass 3 composites the layer with `depth_compare: GreaterEqual` and writes depth. Edges,
transparency and picking then treat it like opaque geometry. Picking an external layer returns
the instance-less `PickId` the adapter registered for it.

## 15. Headless rendering and visual regression (RND-09)
- `Renderer` needs no surface. `capture(view)` copies `final` to a staging buffer with rows
  padded to 256 B (`COPY_BYTES_PER_ROW_ALIGNMENT`), and `Capture::try_get` unpads.
- **Goldens.** PNGs under `tests/golden/render/<case>.png`, blessed with `AMG_BLESS=1`
  (amg-testkit), **generated on WARP** (the CI backend, deterministic).
  - A pixel mismatches if any channel differs by > 3.
  - The test fails if > 0.1 % of pixels mismatch or any pixel differs by > 48.
  - On failure, `<case>_actual.png` and `<case>_diff.png` are written under
    `target/render-diff/`.
  - Local DX12 runs compare with the same thresholds and are expected to pass for the
    deliberately simple scenes. If one doesn't, the case is marked `warp_only`.
- **Thumbnail.** `thumbnail(scene, size)` fits the Home view and returns PNG bytes (DOC-07
  `thumbnail.png`, `amg-cli`).

## 16. Shader compilation and start-up
- wgpu 30's default DX12 compiler is **FXC**, which the docs call "old, slow and unmaintained".
- The `static-dxc` cargo feature gives `Dx12Compiler::StaticDxc` via `mach-dxcompiler-rs` (MIT
  wrapper, prebuilt DXC downloaded at build time). `DynamicDxc` needs a shipped
  `dxcompiler.dll`.
- RND-01 measures core-pipeline creation time with FXC and with StaticDxc. If FXC exceeds
  150 ms for the core set, or any pipeline exceeds 50 ms, StaticDxc is adopted. That needs a
  `GpuConfig` field for the DX12 compiler (S00 request), and the THIRD_PARTY row is added then.
- Core pipelines (opaque, composite, edges) are created synchronously in `Renderer::new`
  (≤ 150 ms). All other variants are created on the Interactive pool on first need; pipeline
  creation is thread-safe. Until a variant is ready, the frame falls back to plain shading for
  that draw.

## 17. Device loss
- `Gpu::is_lost()` flips via `set_device_lost_callback`.
- `encode` then returns `RenderError::Gpu(GpuError::DeviceLost)`, and `Viewport` paints a
  placeholder ("GPU device lost — save and restart").
- Re-creating the device would mean rebuilding every GPU resource of render and compute, so it
  is out of scope. `amg-app` (S11) offers save-and-restart, and the S01 journal guarantees no
  work is lost.

## 18. Budget derivation (RTX 5060 Ti: GB206, 36 SMs, 448 GB/s)
- **Geometry.** NVIDIA setup and raster handles ≈ 1 triangle/clock/GPC. GB206 has 3 GPCs at
  ≈ 2.5 GHz, so ≈ 7.5 G tri/s peak and conservatively 2–3 G tri/s sustained. 5 M triangles
  take ≈ 2–2.5 ms per geometry pass.
- **No depth pre-pass.** With PBR-lite shading and front-to-back sorting, a pre-pass would add
  a full extra geometry pass (≈ 2 ms) to save shading the CAD use case barely overdraws. RND-02
  measures both and keeps the pre-pass only if it wins.
- **Bandwidth** at 2560×1440, 4× MSAA: colour 8 B + normal 4 B + depth 4 B = 16 B/sample, so
  3.7 M px × 4 × 16 B ≈ 236 MB per frame worst case. At 60 fps that is ≈ 14 GB/s, ≈ 3 % of
  448 GB/s.
- **Target GPU split** for 5 M tris + edges at 1440p 4× MSAA, ≤ 12 ms total (headroom under
  16.6 ms for egui, compositor and jitter):

  | Pass | Budget |
  |---|---|
  | Opaque | ≤ 6 ms |
  | Edges | ≤ 1 ms |
  | Caps | ≤ 1 ms |
  | Post | ≤ 1 ms |
  | Overlays | ≤ 0.5 ms |
  | Composite + egui | ≤ 1.5 ms |

- **CPU.** Encode ≤ 2 ms, instance updates ≤ 0.5 ms.
- If RND-02 measures > 12 ms GPU, the first fallbacks, in order: drop the normal target when
  silhouettes are off, drop MSAA to 2×, and pull RND-14 cluster culling forward.

## 19. References
- docs.rs: `egui-wgpu` 0.37.0 (`CallbackTrait`, `Callback`, `RenderState`, `WgpuSetupExisting`,
  `RendererOptions`, `WgpuConfiguration`, `SurfaceConfig`) and 0.36.1 (same `CallbackTrait`);
  `eframe` 0.37.0 (`NativeOptions`, `CreationContext::wgpu_render_state`); `epaint` 0.37.0
  `PaintCallbackInfo`; `wgpu` 30.0.1 (`Features`, `Device`, `RenderPipelineDescriptor`,
  `Dx12Compiler`).
- wgpu v30.0.0 release notes (github.com/gfx-rs/wgpu/releases/tag/v30.0.0).
- crates.io (2026-10-20): wgpu 30.0.1 (2026-08-22), egui-wgpu/eframe 0.37.0 (2026-10-15, MSRV
  1.95), egui_dock 0.21.1 (requires egui ^0.36), meshopt 0.7.0, png 0.18.1, fdsm 0.8.0.
- M. McGuire, L. Bavoil, *Weighted Blended Order-Independent Transparency*, JCGT 2(2), 2013.
- Z. Cigolle et al., *A Survey of Efficient Representations for Independent Unit Vectors*,
  JCGT 3(2), 2014 (octahedral normals).
- E. Persson, *Creating vast game worlds* (relative-to-eye rendering), SIGGRAPH 2012;
  N. Reed, *Depth Precision Visualized*, 2015 (reversed-Z).
- egui `custom3d_wgpu` demo (paint-callback resource pattern).
- Fusion help: visual styles, zebra/draft/curvature map analyses, section analysis, view cube.
