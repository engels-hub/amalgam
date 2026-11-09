// Lambert + headlight (RND-01). Positions arrive relative to the mesh origin; `model_rte` maps
// them to eye-relative world space, so `view_rte` is a pure rotation (design note §3).

// One record per instance (128 bytes, design note §8).
struct InstanceRecord {
    model_rte: mat4x4<f32>,
    normal_from_local: mat3x3<f32>,
    color: vec4<f32>,
}

@group(0) @binding(0) var<uniform> camera: CameraUniform;
@group(0) @binding(1) var<storage, read> instances: array<InstanceRecord>;

struct VsOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) view_pos: vec3<f32>,
    @location(1) view_normal: vec3<f32>,
    @location(2) color: vec4<f32>,
}

fn transform(local: vec3<f32>, instance: u32) -> VsOut {
    let rte = instances[instance].model_rte * vec4<f32>(local, 1.0);
    let view = camera.view_rte * rte;
    var out: VsOut;
    out.clip = camera.proj * view;
    out.view_pos = view.xyz;
    out.view_normal = vec3<f32>(0.0, 0.0, 1.0);
    out.color = instances[instance].color;
    return out;
}

@vertex
fn vs_smooth(
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @builtin(instance_index) instance: u32,
) -> VsOut {
    var out = transform(position, instance);
    let world = instances[instance].normal_from_local * normal;
    out.view_normal = (camera.view_rte * vec4<f32>(world, 0.0)).xyz;
    return out;
}

@vertex
fn vs_flat(
    @location(0) position: vec3<f32>,
    @builtin(instance_index) instance: u32,
) -> VsOut {
    return transform(position, instance);
}

fn shade(normal: vec3<f32>, view_pos: vec3<f32>, color: vec4<f32>) -> vec4<f32> {
    var to_light = vec3<f32>(0.0, 0.0, 1.0);
    if camera.ortho == 0u {
        to_light = normalize(-view_pos);
    }
    let lambert = max(dot(normal, to_light), 0.0);
    return vec4<f32>(color.rgb * (0.12 + 0.88 * lambert), color.a);
}

@fragment
fn fs_smooth(in: VsOut) -> @location(0) vec4<f32> {
    return shade(normalize(in.view_normal), in.view_pos, in.color);
}

@fragment
fn fs_flat(in: VsOut) -> @location(0) vec4<f32> {
    // Screen y points down, so dpdy x dpdx faces the viewer.
    let normal = normalize(cross(dpdy(in.view_pos), dpdx(in.view_pos)));
    return shade(normal, in.view_pos, in.color);
}
