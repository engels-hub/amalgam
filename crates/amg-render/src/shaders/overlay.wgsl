// Overlays (RND-07, design note §12). Appended to `camera.wgsl`, `scene.wgsl`, `section.wgsl` and
// `lines.wgsl`: overlay lines and points reuse the RND-12 wide-line and point stages
// (`line_vertex_slot`, `line_color`, `point_vertex_slot`, `point_color`).
//
// Drawn after the final copy into the single-sampled final image (Rgba8UnormSrgb, alpha blended)
// with the overlay's own depth target (Depth32Float, reversed-Z, cleared per frame), so overlays
// occlude each other. The depth mode against the scene is applied here from the scene depth
// target (`SCENE_DEPTH_TEXTURE`: `texture_depth_2d` or `texture_depth_multisampled_2d`, sample 0):
//   MODE_TEST    hidden where the scene surface is in front,
//   MODE_ALWAYS  drawn over the scene,
//   MODE_XRAY    dimmed to its x-ray alpha where hidden.
//
// Overlay geometry is eye-relative world space: every overlay instance record is the identity
// at the eye, so `to_view` in `lines.wgsl` turns it into view space unchanged. Lines and points
// take their record slot from `geometry_ids` (one draw for all), the line record carries the
// style, `hidden.x` the x-ray alpha and `hidden.w` the depth mode.

@group(2) @binding(0) var scene_depth: SCENE_DEPTH_TEXTURE;
@group(3) @binding(0) var overlay_image: texture_2d<f32>;
@group(3) @binding(1) var overlay_sampler: sampler;

const MODE_TEST: u32 = 0u;
const MODE_ALWAYS: u32 = 1u;
const MODE_XRAY: u32 = 2u;

const KIND_FLAT: u32 = 0u;
const KIND_IMAGE: u32 = 1u;
const KIND_GRID: u32 = 2u;

// An overlay fragment counts as hidden when it is this much (relative reversed-Z depth) behind
// the scene, so geometry lying on a surface stays visible.
const SCENE_DEPTH_TOLERANCE: f32 = 1e-5;
// Grid: minor lines are never closer than GRID_MIN_PX and fade in up to GRID_FULL_PX.
const GRID_MIN_PX: f32 = 8.0;
const GRID_FULL_PX: f32 = 40.0;

// `c` with the depth mode applied at fragment position `frag`; discards hidden MODE_TEST parts.
fn overlay_mode(c: vec4<f32>, frag: vec4<f32>, mode: u32, hidden_alpha: f32) -> vec4<f32> {
    if mode == MODE_ALWAYS {
        return c;
    }
    let scene = textureLoad(scene_depth, vec2<i32>(frag.xy), 0);
    if frag.z < scene * (1.0 - SCENE_DEPTH_TOLERANCE) {
        if mode == MODE_TEST {
            discard;
        }
        return vec4<f32>(c.rgb, c.a * hidden_alpha);
    }
    return c;
}

@vertex
fn vs_overlay_line(@builtin(vertex_index) vertex: u32) -> LineOut {
    return line_vertex_slot(vertex, geometry_ids[vertex / 6u]);
}

@fragment
fn fs_overlay_line(in: LineOut) -> @location(0) vec4<f32> {
    let lines = line_records[in.slot];
    let c = line_color(in);
    return overlay_mode(c, in.clip, u32(lines.hidden.w + 0.5), lines.hidden.x);
}

@vertex
fn vs_overlay_point(@builtin(vertex_index) vertex: u32) -> PointOut {
    return point_vertex_slot(vertex, geometry_ids[vertex / 6u]);
}

@fragment
fn fs_overlay_point(in: PointOut) -> @location(0) vec4<f32> {
    let lines = line_records[in.slot];
    let c = point_color(in);
    return overlay_mode(c, in.clip, u32(lines.hidden.w + 0.5), lines.hidden.y);
}

struct TriIn {
    // Eye-relative world position, mm.
    @location(0) pos: vec3<f32>,
    // Linear RGBA, straight alpha (shaded and highlighted on the CPU).
    @location(1) color: vec4<f32>,
    // Image coordinates, or plane coordinates (mm, relative to a snapped offset) of a grid.
    @location(2) uv: vec2<f32>,
    // x = kind, y = depth mode, z = x-ray alpha, w = grid line width px.
    @location(3) params: vec4<f32>,
    // Pick id and handle sub-id (the pick pass, `overlay_pick.wgsl`).
    @location(4) pick: vec2<u32>,
}

struct TriOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) color: vec4<f32>,
    @location(1) uv: vec2<f32>,
    @location(2) @interpolate(flat) params: vec4<f32>,
}

@vertex
fn vs_overlay_tri(in: TriIn) -> TriOut {
    var out: TriOut;
    out.clip = camera.proj * (camera.view_rte * vec4<f32>(in.pos, 1.0));
    out.color = in.color;
    out.uv = in.uv;
    out.params = in.params;
    return out;
}

// Coverage of the adaptive grid at plane coordinates `uv` with screen derivatives `dx`, `dy`:
// lines every `s` and `10 s`, `s` the smallest power of ten whose screen spacing is at least
// GRID_MIN_PX at this pixel; each set fades in with its spacing up to GRID_FULL_PX.
fn grid_coverage(uv: vec2<f32>, dx: vec2<f32>, dy: vec2<f32>, width_px: f32) -> f32 {
    // Plane mm per pixel across the u lines and across the v lines.
    let g = max(vec2<f32>(length(vec2<f32>(dx.x, dy.x)), length(vec2<f32>(dx.y, dy.y))), vec2<f32>(1e-30));
    let gmax = max(g.x, g.y);
    let level = ceil(log2(GRID_MIN_PX * gmax) / log2(10.0));
    var coverage = 0.0;
    var s = pow(10.0, level);
    for (var k = 0; k < 2; k++) {
        let spacing_px = s / gmax;
        let fade = smoothstep(GRID_MIN_PX, GRID_FULL_PX, spacing_px);
        // Distance in pixels to the nearest u line and v line.
        let f = abs(fract(uv / s + vec2<f32>(0.5)) - vec2<f32>(0.5)) * s / g;
        let d = min(f.x, f.y);
        coverage = max(coverage, clamp(0.5 * width_px + 0.5 - d, 0.0, 1.0) * fade);
        s = s * 10.0;
    }
    return coverage;
}

@fragment
fn fs_overlay_tri(in: TriOut) -> @location(0) vec4<f32> {
    // Derivatives and the implicit-LOD sample in uniform control flow.
    let dx = dpdx(in.uv);
    let dy = dpdy(in.uv);
    let texel = textureSample(overlay_image, overlay_sampler, in.uv);
    let kind = u32(in.params.x + 0.5);
    var c = in.color;
    if kind == KIND_IMAGE {
        c = c * texel;
    } else if kind == KIND_GRID {
        c = vec4<f32>(c.rgb, c.a * grid_coverage(in.uv, dx, dy, in.params.w));
    }
    if c.a <= 0.0 {
        discard;
    }
    return overlay_mode(c, in.clip, u32(in.params.y + 0.5), in.params.z);
}
