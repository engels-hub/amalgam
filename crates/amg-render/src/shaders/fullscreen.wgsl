// One fullscreen triangle with `uv` in [0, 1], y down. Copies texels 1:1.
struct FullscreenOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) uv: vec2<f32>,
}

@vertex
fn vs_fullscreen(@builtin(vertex_index) index: u32) -> FullscreenOut {
    let uv = vec2<f32>(f32((index << 1u) & 2u), f32(index & 2u));
    var out: FullscreenOut;
    out.clip = vec4<f32>(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0, 0.0, 1.0);
    out.uv = uv;
    return out;
}

@group(0) @binding(0) var source: texture_2d<f32>;

@fragment
fn fs_copy(in: FullscreenOut) -> @location(0) vec4<f32> {
    let size = vec2<f32>(textureDimensions(source));
    let texel = vec2<i32>(min(floor(in.uv * size), size - vec2<f32>(1.0, 1.0)));
    return textureLoad(source, texel, 0);
}
