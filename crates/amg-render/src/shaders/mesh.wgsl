// Lambert + headlight (RND-01), instanced from the scene storage buffer (RND-03). Positions
// arrive relative to the mesh origin; the instance record holds the camera-independent
// world-from-local linear part and the split translation, so `eye_relative` yields the
// eye-relative position and `view_rte` is a pure rotation (design note §3).

// One record per instance (128 bytes, `amg_render::scene::InstanceRecord`).
struct InstanceRecord {
    // Linear-part columns in xyz; w is the high part of the world translation.
    lin0: vec4<f32>,
    lin1: vec4<f32>,
    lin2: vec4<f32>,
    // Low part of the world translation.
    t_lo: vec4<f32>,
    // Normal-matrix columns in xyz; w reserved (pick id, flags).
    nrm0: vec4<f32>,
    nrm1: vec4<f32>,
    nrm2: vec4<f32>,
    color: vec4<f32>,
}

@group(0) @binding(0) var<uniform> camera: CameraUniform;
@group(0) @binding(1) var<storage, read> instances: array<InstanceRecord>;
// Instance slots of this view's visible instances in draw order; `instance_index` indexes it.
@group(0) @binding(2) var<storage, read> visible: array<u32>;

struct VsOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) view_pos: vec3<f32>,
    @location(1) view_normal: vec3<f32>,
    @location(2) color: vec4<f32>,
}

fn transform(local: vec3<f32>, record: InstanceRecord) -> VsOut {
    let linear = mat3x3<f32>(record.lin0.xyz, record.lin1.xyz, record.lin2.xyz);
    let t_hi = vec3<f32>(record.lin0.w, record.lin1.w, record.lin2.w);
    let rte = linear * local + eye_relative(camera, t_hi, record.t_lo.xyz);
    let view = camera.view_rte * vec4<f32>(rte, 1.0);
    var out: VsOut;
    out.clip = camera.proj * view;
    out.view_pos = view.xyz;
    out.view_normal = vec3<f32>(0.0, 0.0, 1.0);
    out.color = record.color;
    return out;
}

@vertex
fn vs_smooth(
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @builtin(instance_index) instance: u32,
) -> VsOut {
    let record = instances[visible[instance]];
    var out = transform(position, record);
    let normal_matrix = mat3x3<f32>(record.nrm0.xyz, record.nrm1.xyz, record.nrm2.xyz);
    out.view_normal = (camera.view_rte * vec4<f32>(normal_matrix * normal, 0.0)).xyz;
    return out;
}

@vertex
fn vs_flat(
    @location(0) position: vec3<f32>,
    @builtin(instance_index) instance: u32,
) -> VsOut {
    return transform(position, instances[visible[instance]]);
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
