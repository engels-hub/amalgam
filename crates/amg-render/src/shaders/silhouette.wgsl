// Silhouettes post pass (RND-12, design note §6): a fullscreen triangle over the resolved image
// that reads sample 0 of the view-normal and depth targets with Roberts-cross differences. A pixel
// is an outline where the two diagonal pairs differ by a normal angle above the threshold, by a
// relative linear-depth jump above the threshold, or where geometry meets the background.
//
// `NORMAL_TEXTURE` and `DEPTH_TEXTURE` are replaced by the Rust side with the multisampled or
// single-sampled texture types; `textureLoad(t, p, 0)` reads sample 0 or mip 0 alike.

struct SilhouetteUniform {
    // Depth-row and w-row coefficients of the projection: ndc = (x z + y) / (z' z + w') with
    // (x, y, z', w') = (proj[2].z, proj[3].z, proj[2].w, proj[3].w).
    depth_coeffs: vec4<f32>,
    color: vec4<f32>,
    // x = cosine of the normal-angle threshold, y = relative depth-jump threshold.
    thresholds: vec4<f32>,
}

@group(0) @binding(0) var normals: NORMAL_TEXTURE;
@group(0) @binding(1) var depths: DEPTH_TEXTURE;
@group(0) @binding(2) var<uniform> params: SilhouetteUniform;

struct SilhouetteVsOut {
    @builtin(position) clip: vec4<f32>,
}

@vertex
fn vs_silhouette(@builtin(vertex_index) index: u32) -> SilhouetteVsOut {
    let uv = vec2<f32>(f32((index << 1u) & 2u), f32(index & 2u));
    var out: SilhouetteVsOut;
    out.clip = vec4<f32>(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0, 0.0, 1.0);
    return out;
}

fn sil_oct_decode(e: vec2<f32>) -> vec3<f32> {
    var n = vec3<f32>(e.x, e.y, 1.0 - abs(e.x) - abs(e.y));
    let t = max(-n.z, 0.0);
    n.x += select(t, -t, n.x >= 0.0);
    n.y += select(t, -t, n.y >= 0.0);
    return normalize(n);
}

struct Texel {
    normal: vec3<f32>,
    // Distance along the view axis, mm; 0 = background.
    dist: f32,
}

fn texel(p: vec2<i32>, size: vec2<i32>) -> Texel {
    let q = clamp(p, vec2<i32>(0), size - vec2<i32>(1));
    let depth = textureLoad(depths, q, 0);
    var out: Texel;
    var e = textureLoad(normals, q, 0).xy;
    // Section caps mark their samples by adding 4 (plane + 1) to x (RND-06 `section_mark`).
    if e.x > 2.0 {
        e.x -= 4.0 * floor((e.x + 2.0) / 4.0);
    }
    out.normal = sil_oct_decode(e);
    out.dist = 0.0;
    if depth > 0.0 {
        let c = params.depth_coeffs;
        let z = (c.y - depth * c.w) / (depth * c.z - c.x);
        out.dist = max(-z, 1e-30);
    }
    return out;
}

fn differs(a: Texel, b: Texel) -> bool {
    let a_bg = a.dist == 0.0;
    let b_bg = b.dist == 0.0;
    if a_bg || b_bg {
        return a_bg != b_bg;
    }
    if abs(a.dist - b.dist) > params.thresholds.y * min(a.dist, b.dist) {
        return true;
    }
    return dot(a.normal, b.normal) < params.thresholds.x;
}

@fragment
fn fs_silhouette(in: SilhouetteVsOut) -> @location(0) vec4<f32> {
    let size = vec2<i32>(textureDimensions(depths));
    let p = vec2<i32>(in.clip.xy);
    let t00 = texel(p, size);
    let t11 = texel(p + vec2<i32>(1, 1), size);
    let t10 = texel(p + vec2<i32>(1, 0), size);
    let t01 = texel(p + vec2<i32>(0, 1), size);
    if !(differs(t00, t11) || differs(t10, t01)) {
        discard;
    }
    return params.color;
}
