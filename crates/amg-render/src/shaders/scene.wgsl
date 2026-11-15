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
    // Normal-matrix columns, each followed by a u32 (RND-04): the pick id, the whole-instance
    // highlight bits (`HL_*`) and the index + 1 of the instance's edge and point highlight
    // states in `highlight_states` (0 = none).
    nrm0: vec3<f32>,
    pick: u32,
    nrm1: vec3<f32>,
    flags: u32,
    nrm2: vec3<f32>,
    states: u32,
    color: vec4<f32>,
}

// Per-view style (`amg_render::renderer::StyleUniform`).
struct StyleUniform {
    bg_top: vec4<f32>,
    bg_bottom: vec4<f32>,
    // Highlight colours (linear RGB) of the layers Hover, Selected, Error, Preview (RND-04).
    highlight: array<vec4<f32>, 4>,
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

// Highlight states of edges and points (RND-04). An instance whose `states` is non-zero owns
// the region at `states - 1`: [edge count E, point count P, E edge states, P point states],
// each state a set of `HL_*` bits.
@group(0) @binding(6) var<storage, read> highlight_states: array<u32>;

// Highlight layer bits (`amg_render::HighlightLayer` order).
const HL_HOVER: u32 = 1u;
const HL_SELECTED: u32 = 2u;
const HL_ERROR: u32 = 4u;
// Entries of `visible` may carry highlight bits above the slot (face-range redraws).
const SLOT_MASK: u32 = 0x0fffffffu;
const SLOT_BITS: u32 = 28u;

// The instance slot of a visible-list entry.
fn slot_of(instance: u32) -> u32 {
    return visible[instance] & SLOT_MASK;
}

// Highlight bits carried by a visible-list entry.
fn entry_highlight(instance: u32) -> u32 {
    return visible[instance] >> SLOT_BITS;
}

// `rgb` with the highlight of `bits` applied: Error > Selected > Hover > Preview, and Selected
// together with Hover drawn lighter. The colour replaces the base colour (shading still applies).
fn apply_highlight(rgb: vec3<f32>, bits: u32) -> vec3<f32> {
    if bits == 0u {
        return rgb;
    }
    if (bits & HL_ERROR) != 0u {
        return style.highlight[2].rgb;
    }
    if (bits & HL_SELECTED) != 0u {
        if (bits & HL_HOVER) != 0u {
            return mix(style.highlight[1].rgb, vec3<f32>(1.0), 0.35);
        }
        return style.highlight[1].rgb;
    }
    if (bits & HL_HOVER) != 0u {
        return style.highlight[0].rgb;
    }
    return style.highlight[3].rgb;
}

// Highlight bits of edge (`point == false`) or point `id` of the instance `record`.
fn element_highlight(record: InstanceRecord, id: u32, point: bool) -> u32 {
    if record.states == 0u {
        return 0u;
    }
    let base = record.states - 1u;
    let edges = highlight_states[base];
    let points = highlight_states[base + 1u];
    if point {
        if id >= points {
            return 0u;
        }
        return highlight_states[base + 2u + edges + id];
    }
    if id >= edges {
        return 0u;
    }
    return highlight_states[base + 2u + id];
}
