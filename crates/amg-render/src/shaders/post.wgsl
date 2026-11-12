// Final pass with FXAA (RND-02, design note §2 pass 6). Appended to `fullscreen.wgsl`, whose
// `vs_fullscreen` and `source` (the resolved Rgba16Float image) it reuses; `source_sampler` is
// bilinear and clamps to the edge.
//
// A compact form of the "console" FXAA of T. Lottes' whitepaper: four half-texel diagonal taps
// detect a luma edge, the edge direction comes from their differences, and two or four taps
// along it blend the edge. Pixels with a small local contrast are passed through unchanged.

@group(0) @binding(1) var source_sampler: sampler;

const FXAA_EDGE_THRESHOLD: f32 = 0.125;
const FXAA_EDGE_THRESHOLD_MIN: f32 = 0.0312;
const FXAA_REDUCE_MUL: f32 = 0.125;
const FXAA_REDUCE_MIN: f32 = 0.0078125;
const FXAA_SPAN_MAX: f32 = 8.0;

// Perceptual luma of a linear colour (approximate gamma 2).
fn fxaa_luma(c: vec3<f32>) -> f32 {
    return sqrt(dot(clamp(c, vec3<f32>(0.0), vec3<f32>(1.0)), vec3<f32>(0.299, 0.587, 0.114)));
}

fn fxaa_tap(uv: vec2<f32>) -> vec4<f32> {
    return textureSampleLevel(source, source_sampler, uv, 0.0);
}

@fragment
fn fs_fxaa(in: FullscreenOut) -> @location(0) vec4<f32> {
    let size = vec2<f32>(textureDimensions(source));
    let rcp = 1.0 / size;
    // Centre of the destination pixel.
    let uv = (floor(in.uv * size) + vec2<f32>(0.5)) * rcp;
    let centre = fxaa_tap(uv);
    let m = fxaa_luma(centre.rgb);
    let nw = fxaa_luma(fxaa_tap(uv + vec2<f32>(-0.5, -0.5) * rcp).rgb);
    let ne = fxaa_luma(fxaa_tap(uv + vec2<f32>(0.5, -0.5) * rcp).rgb);
    let sw = fxaa_luma(fxaa_tap(uv + vec2<f32>(-0.5, 0.5) * rcp).rgb);
    let se = fxaa_luma(fxaa_tap(uv + vec2<f32>(0.5, 0.5) * rcp).rgb);
    let lo = min(m, min(min(nw, ne), min(sw, se)));
    let hi = max(m, max(max(nw, ne), max(sw, se)));
    if hi - lo < max(FXAA_EDGE_THRESHOLD_MIN, hi * FXAA_EDGE_THRESHOLD) {
        return centre;
    }
    var dir = vec2<f32>(-((nw + ne) - (sw + se)), (nw + sw) - (ne + se));
    let reduce = max((nw + ne + sw + se) * 0.25 * FXAA_REDUCE_MUL, FXAA_REDUCE_MIN);
    let scale = 1.0 / (min(abs(dir.x), abs(dir.y)) + reduce);
    dir = clamp(dir * scale, vec2<f32>(-FXAA_SPAN_MAX), vec2<f32>(FXAA_SPAN_MAX)) * rcp;
    let a = 0.5 * (fxaa_tap(uv + dir * (1.0 / 3.0 - 0.5)) + fxaa_tap(uv + dir * (2.0 / 3.0 - 0.5)));
    let b = 0.5 * a + 0.25 * (fxaa_tap(uv - dir * 0.5) + fxaa_tap(uv + dir * 0.5));
    let lb = fxaa_luma(b.rgb);
    if lb < lo || lb > hi {
        return a;
    }
    return b;
}
