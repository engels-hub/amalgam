// Scene data shared by the mesh pass (`mesh.wgsl`) and the line, point and wireframe passes
// (`lines.wgsl`): the instance records, the per-view style and the group-0 view bindings.

// One record per instance (128 bytes, `amg_render::scene::InstanceRecord`).
struct InstanceRecord {
    // Linear-part columns in xyz; w is the high part of the world translation.
    lin0: vec4<f32>,
    lin1: vec4<f32>,
    lin2: vec4<f32>,
    // Low part of the world translation.
    t_lo: vec3<f32>,
    // pack2x16unorm(metallic, roughness).
    material: u32,
    // Normal-matrix columns in xyz; w reserved (pick id, flags).
    nrm0: vec4<f32>,
    nrm1: vec4<f32>,
    nrm2: vec4<f32>,
    color: vec4<f32>,
}

// Per-view style (`amg_render::renderer::StyleUniform`).
struct StyleUniform {
    bg_top: vec4<f32>,
    bg_bottom: vec4<f32>,
}

@group(0) @binding(0) var<uniform> camera: CameraUniform;
@group(0) @binding(1) var<storage, read> instances: array<InstanceRecord>;
// Instance slots of this view's visible instances in draw order; `instance_index` indexes it.
@group(0) @binding(2) var<storage, read> visible: array<u32>;
@group(0) @binding(3) var<uniform> style: StyleUniform;
// Line and point style of each instance, parallel to `instances` (RND-12,
// `amg_render::lines::LineRecord`, 96 bytes).
struct LineRecord {
    edge_color: vec4<f32>,
    point_color: vec4<f32>,
    // Edge-set origin minus the record origin, in the instance's local frame; w = width px.
    edge_delta: vec4<f32>,
    // Point-set origin minus the record origin; w = point diameter px.
    point_delta: vec4<f32>,
    // xy = (on, off) px of the visible pass, zw = (on, off) px of the hidden pass; 0 = solid.
    dash: vec4<f32>,
    // x = alpha factor of hidden edges, y = of hidden points; zw reserved.
    hidden: vec4<f32>,
}
@group(0) @binding(4) var<storage, read> line_records: array<LineRecord>;
