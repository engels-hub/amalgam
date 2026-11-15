// Pick cap depth (RND-04): after the face pick, every section-cap texel (`CAP_SUB`) gets the
// depth of its cap point (stored in the pick depth target), so edges and points behind the cap
// lose the depth test like in the drawn image (`cap_depth.wgsl`). Depth attachment only.

@group(0) @binding(0) var pick_ids: texture_2d<u32>;
@group(0) @binding(1) var pick_positions: texture_2d<f32>;

@vertex
fn vs_pick_fix(@builtin(vertex_index) index: u32) -> @builtin(position) vec4<f32> {
    let uv = vec2<f32>(f32((index << 1u) & 2u), f32(index & 2u));
    return vec4<f32>(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0, 0.0, 1.0);
}

@fragment
fn fs_pick_fix(@builtin(position) pos: vec4<f32>) -> @builtin(frag_depth) f32 {
    let texel = vec2<i32>(pos.xy);
    let id = textureLoad(pick_ids, texel, 0);
    // Kind face (0) with the cap sub-id.
    if id.x == 0u || id.y != 0x3fffffffu {
        discard;
    }
    return textureLoad(pick_positions, texel, 0).w;
}
