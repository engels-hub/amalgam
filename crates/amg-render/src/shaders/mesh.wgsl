// Opaque mesh pass (RND-02), instanced from the scene storage buffer (RND-03). Positions arrive
// relative to the mesh origin; the instance record holds the camera-independent world-from-local
// linear part and the split translation, so `eye_relative` yields the eye-relative position and
// `view_rte` is a pure rotation (design note §3).
//
// Pipeline variants are WGSL override constants (design note §4):
//   LIGHTING      0 Studio, 1 Matcap, 2 Flat (`amg_render::Lighting`).
// Facet normals from screen derivatives (meshes without normals, or the view's flat toggle) use
// the vertex stage `vs_flat`, which reads positions only and passes a zero normal; the fragment
// stage is the same for both, so the backend compiles it once per lighting model.
// Cull mode (closed meshes cull back faces, open ones are drawn double-sided), depth compare
// (Equal after a depth pre-pass) and the sample count are pipeline state.
//
// Targets: 0 = colour (Rgba16Float, linear), 1 = view normal (Rg16Float, octahedral).

override LIGHTING: u32 = 0u;

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

struct VsOut {
    // Invariant so the Equal depth test after a depth pre-pass sees bit-identical depth.
    @builtin(position) @invariant clip: vec4<f32>,
    @location(0) view_pos: vec3<f32>,
    @location(1) view_normal: vec3<f32>,
    @location(2) color: vec4<f32>,
    @location(3) @interpolate(flat) material: vec2<f32>,
}

fn transform(local: vec3<f32>, record: InstanceRecord) -> VsOut {
    let linear = mat3x3<f32>(record.lin0.xyz, record.lin1.xyz, record.lin2.xyz);
    let t_hi = vec3<f32>(record.lin0.w, record.lin1.w, record.lin2.w);
    let rte = linear * local + eye_relative(camera, t_hi, record.t_lo);
    let view = camera.view_rte * vec4<f32>(rte, 1.0);
    var out: VsOut;
    out.clip = camera.proj * view;
    out.view_pos = view.xyz;
    // Zero = facet normal from derivatives (`vs_smooth` overwrites it with a unit normal).
    out.view_normal = vec3<f32>(0.0);
    out.color = record.color;
    out.material = unpack2x16unorm(record.material);
    return out;
}

@vertex
fn vs_smooth(
    @location(0) position: vec3<f32>,
    // Snorm16x2 octahedral normal.
    @location(1) normal_oct: vec2<f32>,
    @builtin(instance_index) instance: u32,
) -> VsOut {
    let record = instances[visible[instance]];
    var out = transform(position, record);
    let normal_matrix = mat3x3<f32>(record.nrm0.xyz, record.nrm1.xyz, record.nrm2.xyz);
    let normal = oct_decode(normal_oct);
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

struct FsOut {
    @location(0) color: vec4<f32>,
    @location(1) normal: vec2<f32>,
}

@fragment
fn fs_main(in: VsOut, @builtin(front_facing) front: bool) -> FsOut {
    // Screen y points down, so dpdy x dpdx faces the viewer. Evaluated in uniform control flow.
    let facet = normalize(cross(dpdy(in.view_pos), dpdx(in.view_pos)));
    var n = facet;
    // An interpolated unit normal is never shorter than 0.5; a zero one marks `vs_flat`.
    if dot(in.view_normal, in.view_normal) > 0.25 {
        // Back faces of double-sided (open) meshes are lit from their visible side.
        n = normalize(in.view_normal) * select(-1.0, 1.0, front);
    }
    var v = vec3<f32>(0.0, 0.0, 1.0);
    if camera.ortho == 0u {
        v = normalize(-in.view_pos);
    }
    let up = (camera.view_rte * vec4<f32>(0.0, 0.0, 1.0, 0.0)).xyz;
    var out: FsOut;
    out.color = vec4<f32>(shade(LIGHTING, n, v, up, in.color.rgb, in.material), in.color.a);
    out.normal = oct_encode(n);
    return out;
}

// Gradient background: a fullscreen triangle at the far plane (reversed-Z depth 0), drawn first
// with depth test Always and no depth write; top colour at the top edge.
struct BackgroundOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) t: f32,
}

@vertex
fn vs_background(@builtin(vertex_index) index: u32) -> BackgroundOut {
    let uv = vec2<f32>(f32((index << 1u) & 2u), f32(index & 2u));
    var out: BackgroundOut;
    out.clip = vec4<f32>(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0, 0.0, 1.0);
    out.t = uv.y;
    return out;
}

@fragment
fn fs_background(in: BackgroundOut) -> FsOut {
    var out: FsOut;
    out.color = vec4<f32>(mix(style.bg_top.rgb, style.bg_bottom.rgb, clamp(in.t, 0.0, 1.0)), 1.0);
    out.normal = vec2<f32>(0.0);
    return out;
}
