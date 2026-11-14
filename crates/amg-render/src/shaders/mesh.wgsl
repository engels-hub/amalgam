// Opaque mesh pass (RND-02), instanced from the scene storage buffer (RND-03). Positions arrive
// relative to the mesh origin; the instance record holds the camera-independent world-from-local
// linear part and the split translation, so `eye_relative` yields the eye-relative position and
// `view_rte` is a pure rotation (design note §3).
//
// Pipeline variants are WGSL override constants (design note §4):
//   LIGHTING      0 Studio, 1 Matcap, 2 Flat (`amg_render::Lighting`).
//   CLIP          true: discard fragments removed by the view's section planes (RND-06,
//                 `section.wgsl`). Off, the shader has no discard and keeps early depth.
//   CAPS          true (clipped closed meshes, drawn without culling): back faces become
//                 section caps (see "Section caps" below).
// Facet normals from screen derivatives (meshes without normals, or the view's flat toggle) use
// the vertex stage `vs_flat`, which reads positions only and passes a zero normal; the fragment
// stage is the same for both, so the backend compiles it once per lighting model.
// Cull mode (closed meshes cull back faces, open ones are drawn double-sided), depth compare
// (Equal after a depth pre-pass) and the sample count are pipeline state.
//
// Targets: 0 = colour (Rgba16Float, linear), 1 = view normal (Rg16Float, octahedral).

override LIGHTING: u32 = 0u;
override CLIP: bool = false;
override CAPS: bool = false;

// `InstanceRecord`, `StyleUniform` and the group-0 bindings are in `scene.wgsl`.

struct VsOut {
    // Invariant so the Equal depth test after a depth pre-pass sees bit-identical depth.
    @builtin(position) @invariant clip: vec4<f32>,
    @location(0) view_pos: vec3<f32>,
    @location(1) view_normal: vec3<f32>,
    @location(2) color: vec4<f32>,
    @location(3) @interpolate(flat) material: vec2<f32>,
    // Eye-relative world position (section clipping).
    @location(4) rte: vec3<f32>,
    // Instance slot, for the section cap colour (CAPS only).
    @location(5) @interpolate(flat) slot: u32,
}

fn transform(local: vec3<f32>, record: InstanceRecord) -> VsOut {
    let linear = mat3x3<f32>(record.lin0.xyz, record.lin1.xyz, record.lin2.xyz);
    let t_hi = vec3<f32>(record.lin0.w, record.lin1.w, record.lin2.w);
    let rte = linear * local + eye_relative(camera, t_hi, record.t_lo);
    let view = camera.view_rte * vec4<f32>(rte, 1.0);
    var out: VsOut;
    out.clip = camera.proj * view;
    out.view_pos = view.xyz;
    out.rte = rte;
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
    let slot = visible[instance];
    let record = instances[slot];
    var out = transform(position, record);
    if CAPS {
        out.slot = slot;
    }
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
    let slot = visible[instance];
    let record = instances[slot];
    var out = transform(position, record);
    if CAPS {
        out.slot = slot;
    }
    return out;
}

struct FsOut {
    @location(0) color: vec4<f32>,
    @location(1) normal: vec2<f32>,
}

@fragment
fn fs_main(in: VsOut, @builtin(front_facing) front: bool) -> FsOut {
    // Screen y points down, so dpdy x dpdx faces the viewer. Evaluated in uniform control flow.
    let facet = normalize(cross(dpdy(in.view_pos), dpdx(in.view_pos)));
    if CAPS && !front {
        // A back face of a cut closed mesh: a cap, or a clipped / shaded back face.
        let cap = section_cap_fragment(in.rte, cap_color(line_records[in.slot].hidden.w, in.color.rgb));
        if cap.keep == 2u {
            var out: FsOut;
            out.color = cap.color;
            out.normal = cap.normal;
            return out;
        }
        if CLIP && cap.keep == 0u {
            discard;
        }
    } else if CLIP && section_clipped(in.rte) {
        discard;
    }
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

// Depth-only faces of a clipped instance (hidden-line wireframe styles): the section planes
// discard; with `CAPS` (a closed mesh drawn without culling) back faces that are cap
// candidates are kept. Nothing is written but depth.
@fragment
fn fs_depth_clip(in: VsOut, @builtin(front_facing) front: bool) {
    if CAPS && !front {
        if !section_back_face_kept(in.rte) {
            discard;
        }
    } else if section_clipped(in.rte) {
        discard;
    }
}

// Section caps (RND-06, design note §10). Clipped closed meshes are drawn without culling
// (`CAPS`), so their back faces go through the depth test against their own depth: only back
// faces seen through the cut survive. A pixel shows a cap when the first surface the view ray
// meets after entering the kept region is a back face: the ray is then inside the body at the
// entry point, which is the cap point. Those back faces are the visible kept ones and, when the
// kept region is bounded along the ray (two or more planes, e.g. a box section inside a body),
// the ones past the region's exit; front faces past the exit are drawn as depth-only occluders
// before (`fs_cap_occluder`), so a back face behind them (the ray left the body before) is not
// taken for a cap.
// The cap fragment shows the instance's cap colour with the plane's hatch and marks the plane
// in the normal target (`section_mark`); the cap depth pass then moves the depth of marked
// samples onto the plane (`cap_depth.wgsl`), so edges and later passes test against the cap.
// A ray starting inside the kept region (eye inside the body) or a plane without caps shows a
// kept back face shaded with its normal flipped towards the viewer.
//
// Testing the back face's own depth is what keeps holes right: through the bore of a cut tube
// the inner wall (a kept front face) hides the outer wall's back face, so the bore is not
// painted with a cap that lies in front of it.
struct CapOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) rte: vec3<f32>,
}

// Cap colour of a line record: `hidden.w` = sRGB 8-bit r | g << 8 | b << 16 as an exact float,
// or negative for the default (the base colour darkened).
const CAP_DARKEN: f32 = 0.55;

fn srgb_to_linear(c: vec3<f32>) -> vec3<f32> {
    let lo = c / 12.92;
    let hi = pow((c + vec3<f32>(0.055)) / 1.055, vec3<f32>(2.4));
    return select(hi, lo, c <= vec3<f32>(0.04045));
}

fn cap_color(packed: f32, base: vec3<f32>) -> vec3<f32> {
    if packed < 0.0 {
        return base * CAP_DARKEN;
    }
    let u = u32(packed);
    let c = vec3<f32>(f32(u & 255u), f32((u >> 8u) & 255u), f32((u >> 16u) & 255u)) / 255.0;
    return srgb_to_linear(c);
}

struct CapFragment {
    // 0 = discard, 1 = shade as a back face (eye inside the body, or no cap), 2 = cap.
    keep: u32,
    color: vec4<f32>,
    normal: vec2<f32>,
}

// The back-face fragment at the eye-relative point `x` of a cut closed instance with cap colour
// `cap_rgb`. It uses no derivatives (the hatch antialiasing takes the analytic gradient), so it
// runs inside the non-uniform `front_facing` branch.
fn section_cap_fragment(x: vec3<f32>, cap_rgb: vec3<f32>) -> CapFragment {
    var out: CapFragment;
    let ray = section_ray(x);
    let hit = section_cap(ray);
    let plane = sections.planes[hit.plane];
    // A cap when the ray entered the kept region before this back face (kept, or past the exit).
    if hit.found && hit.t <= 1.0001 && plane.params.x > 0.5 {
        let p = ray.origin + ray.dir * hit.t;
        let uv = section_hatch_coords(hit.plane, p);
        let g = section_hatch_gradient(hit.plane, p);
        var coverage = section_hatch_line(uv.x, g.x);
        if plane.params.y > 0.5 {
            coverage = max(coverage, section_hatch_line(uv.y, g.y));
        }
        let rgb = mix(cap_rgb, plane.color.rgb, clamp(plane.color.a * coverage, 0.0, 1.0));
        out.keep = 2u;
        out.color = vec4<f32>(rgb, 1.0);
        let n = normalize((camera.view_rte * vec4<f32>(plane.plane.xyz, 0.0)).xyz);
        out.normal = section_mark(oct_encode(n), hit.plane);
        return out;
    }
    out.keep = select(1u, 0u, section_clipped(x));
    return out;
}

// Whether a back face of a cut closed instance at the eye-relative point `x` is drawn: as a
// cap, or as a kept back face (`section_cap_fragment` without the colour).
fn section_back_face_kept(x: vec3<f32>) -> bool {
    let hit = section_cap(section_ray(x));
    let cap = hit.found && hit.t <= 1.0001 && sections.planes[hit.plane].params.x > 0.5;
    return cap || !section_clipped(x);
}

@vertex
fn vs_cap(
    @location(0) position: vec3<f32>,
    @builtin(instance_index) instance: u32,
) -> CapOut {
    let base = transform(position, instances[visible[instance]]);
    var out: CapOut;
    out.clip = base.clip;
    out.rte = base.rte;
    return out;
}

// Front faces of cut closed instances past the exit of the kept region: depth only (the
// pipeline masks every colour write), drawn before the meshes so cap candidates behind them
// fail the depth test.
@fragment
fn fs_cap_occluder(in: CapOut) -> FsOut {
    let hit = section_cap(section_ray(in.rte));
    if !(hit.found && hit.t_out < 0.9999) {
        discard;
    }
    var out: FsOut;
    out.color = vec4<f32>(0.0);
    out.normal = vec2<f32>(0.0);
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
