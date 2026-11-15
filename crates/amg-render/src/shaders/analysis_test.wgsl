// GPU test kernels of `amg_render::analysis::testing`: they run the functions of
// `analysis_common.wgsl` (prepended) on plain inputs, so tests check the exact WGSL the
// analysis pipelines use. Each kernel is compiled in its own module with only its bindings
// (KERNEL selects one at text level).

struct Params {
    count: u32,
    slot: u32,
    layer: u32,
    pad: u32,
}

//#if OVERHANG
// 12 floats per case: n.xyz, d.xyz, cos α, height, tol, 0, 0, 0.
@group(0) @binding(0) var<storage, read> cases: array<f32>;
@group(0) @binding(1) var<storage, read_write> classes: array<u32>;
@group(0) @binding(2) var<uniform> params: Params;

@compute @workgroup_size(64)
fn overhang_kernel(@builtin(global_invocation_id) id: vec3<u32>) {
    let i = id.x;
    if i >= params.count {
        return;
    }
    let b = 12u * i;
    let n = vec3<f32>(cases[b], cases[b + 1u], cases[b + 2u]);
    let d = vec3<f32>(cases[b + 3u], cases[b + 4u], cases[b + 5u]);
    classes[i] = overhang_class(n, d, cases[b + 6u], cases[b + 7u], cases[b + 8u]);
}
//#endif

//#if LUT
// 4 floats per case: value, min, max, bands.
@group(0) @binding(0) var<storage, read> cases: array<f32>;
@group(0) @binding(1) var<storage, read_write> colors: array<vec4<f32>>;
@group(0) @binding(2) var<uniform> params: Params;
@group(0) @binding(3) var lut: texture_2d_array<f32>;
@group(0) @binding(4) var lut_sampler: sampler;

@compute @workgroup_size(64)
fn lut_kernel(@builtin(global_invocation_id) id: vec3<u32>) {
    let i = id.x;
    if i >= params.count {
        return;
    }
    let b = 4u * i;
    let t = field_t(cases[b], cases[b + 1u], cases[b + 2u], u32(cases[b + 3u]));
    let rgb = textureSampleLevel(lut, lut_sampler, vec2<f32>(colormap_u(t), 0.5), params.layer, 0.0).rgb;
    colors[i] = vec4<f32>(rgb, t);
}
//#endif

//#if DEFORM
// The renderer's own buffers: the mesh positions (copied), the displacement field, the analysis
// records and the globals (animation time).
@group(0) @binding(0) var<storage, read> positions: array<f32>;
@group(0) @binding(1) var<storage, read> displacements: array<f32>;
@group(0) @binding(2) var<storage, read> records: array<AnalysisRecord>;
@group(0) @binding(3) var<uniform> globals: vec4<f32>;
@group(0) @binding(4) var<storage, read_write> deformed: array<f32>;
@group(0) @binding(5) var<uniform> params: Params;

@compute @workgroup_size(64)
fn deform_kernel(@builtin(global_invocation_id) id: vec3<u32>) {
    let i = id.x;
    if i >= params.count {
        return;
    }
    let p = vec3<f32>(positions[3u * i], positions[3u * i + 1u], positions[3u * i + 2u]);
    let d = vec3<f32>(displacements[3u * i], displacements[3u * i + 1u], displacements[3u * i + 2u]);
    let q = deformed_position(p, d, records[params.slot].deform, globals.x);
    deformed[3u * i] = q.x;
    deformed[3u * i + 1u] = q.y;
    deformed[3u * i + 2u] = q.z;
}
//#endif
