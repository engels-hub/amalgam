// Composite of an `ExternalLayer` (design note §14): premultiplied colour over the opaque
// pass, depth-tested against and written into the view's reversed-Z depth. Texels with depth 0
// are empty. Both layer textures have the view's size, so pixel coordinates index them 1:1.

@group(0) @binding(0) var layer_color: texture_2d<f32>;
@group(0) @binding(1) var layer_depth: texture_2d<f32>;

@vertex
fn vs_external(@builtin(vertex_index) index: u32) -> @builtin(position) vec4<f32> {
    let uv = vec2<f32>(f32((index << 1u) & 2u), f32(index & 2u));
    return vec4<f32>(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0, 0.0, 1.0);
}

struct ExternalOut {
    @location(0) color: vec4<f32>,
    @builtin(frag_depth) depth: f32,
}

@fragment
fn fs_external(@builtin(position) pos: vec4<f32>) -> ExternalOut {
    let texel = vec2<i32>(pos.xy);
    let depth = textureLoad(layer_depth, texel, 0).r;
    if depth <= 0.0 {
        discard;
    }
    var out: ExternalOut;
    out.color = textureLoad(layer_color, texel, 0);
    out.depth = depth;
    return out;
}
