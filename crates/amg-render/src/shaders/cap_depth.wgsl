// Cap depth pass (RND-06, design note §10): a fullscreen pass at the view's sample count, run
// per sample, that moves the depth of every cap sample onto its section plane. A cap sample is
// one whose normal-target value carries `section_mark` (x > 2, the plane in it). The pass has
// only the depth attachment (compare Always, write); unmarked samples are discarded.
//
// Appended after `camera.wgsl`, `scene.wgsl` and `section.wgsl`. `MARK_TEXTURE` is replaced by
// the Rust side with the multisampled or single-sampled texture type.

@group(1) @binding(0) var marks: MARK_TEXTURE;

@vertex
fn vs_cap_depth(@builtin(vertex_index) index: u32) -> @builtin(position) vec4<f32> {
    let uv = vec2<f32>(f32((index << 1u) & 2u), f32(index & 2u));
    return vec4<f32>(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0, 0.0, 1.0);
}

fn unproject_rte(ndc: vec3<f32>) -> vec3<f32> {
    let h = camera.inv_view_proj_rte * vec4<f32>(ndc, 1.0);
    return h.xyz / h.w;
}

@fragment
fn fs_cap_depth(
    @builtin(position) pos: vec4<f32>,
    @builtin(sample_index) sample: u32,
) -> @builtin(frag_depth) f32 {
    let mark = textureLoad(marks, vec2<i32>(pos.xy), i32(sample)).x;
    if mark <= 2.0 {
        discard;
    }
    let plane_index = min(u32(floor((mark + 2.0) / 4.0)) - 1u, MAX_SECTIONS - 1u);
    let plane = sections.planes[plane_index].plane;
    let ndc = vec2<f32>(pos.x / camera.size_px.x * 2.0 - 1.0, 1.0 - pos.y / camera.size_px.y * 2.0);
    // Two points on the pixel's view ray (reversed-Z: 1 = near plane).
    let a = unproject_rte(vec3<f32>(ndc, 1.0));
    let b = unproject_rte(vec3<f32>(ndc, 0.5));
    let dir = b - a;
    let t = (plane.w - dot(plane.xyz, a)) / dot(plane.xyz, dir);
    let x = a + dir * t;
    let clip = camera.proj * (camera.view_rte * vec4<f32>(x, 1.0));
    return clamp(clip.z / clip.w, 0.0, 1.0);
}
