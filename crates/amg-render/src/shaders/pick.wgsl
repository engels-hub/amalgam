// Picking (RND-04, design note §7). Appended after `camera.wgsl`, `scene.wgsl`, `section.wgsl`
// and `lines.wgsl` (the pick line and point stages reuse `vs_line` / `vs_point`).
//
// Targets: 0 = id (`Rg32Uint`: pick id, `kind << 30 | sub`), 1 = position (`Rgba32Float`: xyz = the
// eye-relative world point of the hit, w = its reversed-Z depth). Faces store the interpolated
// surface point, edges the nearest point of the centre line and points the centre, both without
// the depth offset of the drawn lines; a section cap stores the cap point on its plane.
// The depth attachment tests `GreaterEqual` (lines and points: their visible-pass compare) and is
// written. Point picks render the `(2r + 1)²` square around the cursor into a target of that
// size (the viewport is offset, so the view transform is unchanged).
//
// Face sub-ids: a B-rep mesh has a flat per-vertex face id (`vs_pick_face`); a plain mesh uses
// `@builtin(primitive_index)` (`fs_pick_prim`, in the module with `enable primitive_index;`) or,
// without the feature, a non-indexed draw that pulls indices and positions from storage
// buffers and passes `vertex_index / 3` (`vs_pick_pull`).
//
// Override constants:
//   CLIP   discard what the section planes remove (also used by the line and point stages).
//   CAPS   a cut closed mesh drawn without culling: back faces seen through the cut are caps
//          (`CAP_SUB`), the same test as the drawn cap (`section_cap`, `mesh.wgsl`).

override CAPS: bool = false;

const KIND_FACE: u32 = 0u;
const KIND_EDGE: u32 = 1u;
const KIND_VERTEX: u32 = 2u;
const SUB_MASK: u32 = 0x3fffffffu;
// Pick sub-id of a section cap (`amg_render::CAP_SUB`).
const CAP_SUB: u32 = 0x3fffffffu;

// Index and position buffers of a mesh for the vertex-pulling backend.
@group(1) @binding(2) var<storage, read> pull_indices: array<u32>;
@group(1) @binding(3) var<storage, read> pull_positions: array<f32>;

struct PickOut {
    @location(0) id: vec2<u32>,
    @location(1) pos: vec4<f32>,
}

struct PickVs {
    @builtin(position) clip: vec4<f32>,
    // Eye-relative world position (section clipping and caps).
    @location(0) rte: vec3<f32>,
    @location(1) @interpolate(flat) pick: u32,
    // Face id (B-rep) or triangle index (vertex pulling).
    @location(2) @interpolate(flat) sub: u32,
}

fn pick_vertex(local: vec3<f32>, instance: u32) -> PickVs {
    let record = instances[slot_of(instance)];
    let linear = mat3x3<f32>(record.lin0.xyz, record.lin1.xyz, record.lin2.xyz);
    let t_hi = vec3<f32>(record.lin0.w, record.lin1.w, record.lin2.w);
    let rte = linear * local + eye_relative(camera, t_hi, record.t_lo);
    var out: PickVs;
    out.clip = camera.proj * (camera.view_rte * vec4<f32>(rte, 1.0));
    out.rte = rte;
    out.pick = record.pick;
    out.sub = 0u;
    return out;
}

@vertex
fn vs_pick_face(
    @location(0) position: vec3<f32>,
    @location(1) face: u32,
    @builtin(instance_index) instance: u32,
) -> PickVs {
    var out = pick_vertex(position, instance);
    out.sub = face;
    return out;
}

@vertex
fn vs_pick_plain(
    @location(0) position: vec3<f32>,
    @builtin(instance_index) instance: u32,
) -> PickVs {
    return pick_vertex(position, instance);
}

@vertex
fn vs_pick_pull(
    @builtin(vertex_index) vertex: u32,
    @builtin(instance_index) instance: u32,
) -> PickVs {
    let i = pull_indices[vertex];
    let p = vec3<f32>(pull_positions[3u * i], pull_positions[3u * i + 1u], pull_positions[3u * i + 2u]);
    var out = pick_vertex(p, instance);
    out.sub = vertex / 3u;
    return out;
}

fn clip_depth(rte: vec3<f32>) -> f32 {
    let c = camera.proj * (camera.view_rte * vec4<f32>(rte, 1.0));
    return c.z / c.w;
}

// The eye-relative point `p` of a line or point moved back by the depth offset of `depth_offset`
// (`lines.wgsl`), away from the eye along its view ray (perspective) or the view axis (ortho).
fn undo_depth_offset(p: vec3<f32>, half_width: f32) -> vec3<f32> {
    let forward = -vec3<f32>(camera.view_rte[0].z, camera.view_rte[1].z, camera.view_rte[2].z);
    let v = (camera.view_rte * vec4<f32>(p, 1.0)).xyz;
    let eps = (LINE_DEPTH_PX + half_width) * pixel_size(v);
    if camera.ortho != 0u {
        return p + forward * eps;
    }
    let len = length(p);
    if len <= 0.0 {
        return p;
    }
    return p * (1.0 + eps / len);
}

// The exact surface point under the pixel centre: the interpolated position lies on the
// triangle's plane but, through the rasteriser's sub-pixel snapping, not exactly on the pixel's
// ray, so the ray through the pixel centre is intersected with that plane (normal from
// screen derivatives). The pick camera puts the view pixel of the target's top-left texel in
// `eye_hi.w`, `eye_lo.w` (point picks render a small target with an offset viewport).
fn surface_point(in: PickVs, normal: vec3<f32>) -> vec3<f32> {
    let px = in.clip.xy + vec2<f32>(camera.eye_hi.w, camera.eye_lo.w);
    let ndc = vec2<f32>(px.x / camera.size_px.x * 2.0 - 1.0, 1.0 - px.y / camera.size_px.y * 2.0);
    let ha = camera.inv_view_proj_rte * vec4<f32>(ndc, 1.0, 1.0);
    let hb = camera.inv_view_proj_rte * vec4<f32>(ndc, 0.5, 1.0);
    let a = ha.xyz / ha.w;
    let dir = hb.xyz / hb.w - a;
    let nd = dot(normal, dir);
    if abs(nd) <= 1e-6 * length(normal) * length(dir) {
        return in.rte;
    }
    return a + dir * (dot(normal, in.rte - a) / nd);
}

// The id of a face fragment with sub-id `sub`; discards what the planes remove. With `CAPS`, a
// back face where the view ray entered the kept region before it is a cap.
fn face_out(in: PickVs, front: bool, sub: u32, normal: vec3<f32>) -> PickOut {
    var out: PickOut;
    out.id = vec2<u32>(in.pick, (KIND_FACE << 30u) | (sub & SUB_MASK));
    out.pos = vec4<f32>(surface_point(in, normal), in.clip.z);
    if CAPS && !front {
        let ray = section_ray(in.rte);
        let hit = section_cap(ray);
        if hit.found && hit.t <= 1.0001 && sections.planes[hit.plane].params.x > 0.5 {
            out.id = vec2<u32>(in.pick, (KIND_FACE << 30u) | CAP_SUB);
            let p = ray.origin + ray.dir * hit.t;
            out.pos = vec4<f32>(p, clamp(clip_depth(p), 0.0, 1.0));
            return out;
        }
        if section_clipped(in.rte) {
            discard;
        }
        return out;
    }
    if CLIP && section_clipped(in.rte) {
        discard;
    }
    return out;
}

@fragment
fn fs_pick_face(in: PickVs, @builtin(front_facing) front: bool) -> PickOut {
    let normal = cross(dpdx(in.rte), dpdy(in.rte));
    return face_out(in, front, in.sub, normal);
}

// Front faces of cut closed instances past the exit of a bounded kept region, depth only (the
// pipeline masks both colour targets), so a back face behind them is not taken for a cap.
@fragment
fn fs_pick_occluder(in: PickVs) -> PickOut {
    let hit = section_cap(section_ray(in.rte));
    if !(hit.found && hit.t_out < 0.9999) {
        discard;
    }
    var out: PickOut;
    out.id = vec2<u32>(0u);
    out.pos = vec4<f32>(0.0);
    return out;
}

// Pick id of an edge or point fragment (`vs_line` / `vs_point` outputs plus the ids).
struct PickLineOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) @interpolate(linear) coord: vec2<f32>,
    @location(1) @interpolate(flat) shape: vec4<f32>,
    @location(2) @interpolate(flat) dash: vec4<f32>,
    @location(4) @interpolate(flat) rte_a: vec3<f32>,
    @location(5) @interpolate(flat) rte_b: vec3<f32>,
    @location(6) @interpolate(flat) section: vec3<f32>,
    // x = pick id, y = kind << 30 | sub.
    @location(7) @interpolate(flat) id: vec2<u32>,
}

@vertex
fn vs_pick_line(
    @builtin(vertex_index) vertex: u32,
    @builtin(instance_index) instance: u32,
) -> PickLineOut {
    let line = line_vertex(vertex, instance);
    var out: PickLineOut;
    out.clip = line.clip;
    out.coord = line.coord;
    out.shape = line.shape;
    out.dash = line.dash;
    out.rte_a = line.rte_a;
    out.rte_b = line.rte_b;
    out.section = line.section;
    let record = instances[slot_of(instance)];
    out.id = vec2<u32>(record.pick, (KIND_EDGE << 30u) | (geometry_ids[vertex / 6u] & SUB_MASK));
    return out;
}

// A pick texel is an edge texel when its centre lies within the half width (at least half a
// pixel) of the centre line plus a quarter pixel: every 1-px edge marks at least one texel per
// step along it. Dashes are ignored (the whole edge is pickable).
@fragment
fn fs_pick_line(in: PickLineOut) -> PickOut {
    let len = in.shape.x;
    let half = in.shape.y;
    let along = in.coord.x;
    if (along < 0.0 && in.dash.z > 0.5) || (along > len && in.dash.w > 0.5) {
        discard;
    }
    let t = clamp(along, 0.0, len);
    if in.section.z > 0.5 {
        var s = 0.0;
        if len > 0.0 {
            s = t / len;
        }
        if camera.ortho == 0u {
            s = (s / in.section.y) / ((1.0 - s) / in.section.x + s / in.section.y);
        }
        if section_clipped(mix(in.rte_a, in.rte_b, s)) {
            discard;
        }
    }
    let dist = length(vec2<f32>(along - t, in.coord.y));
    if dist > max(half, 0.5) + 0.25 {
        discard;
    }
    // The nearest point of the centre line, perspective-correct, without the depth offset.
    var s = 0.0;
    if len > 0.0 {
        s = t / len;
    }
    if camera.ortho == 0u {
        s = (s / in.section.y) / ((1.0 - s) / in.section.x + s / in.section.y);
    }
    let p = undo_depth_offset(mix(in.rte_a, in.rte_b, s), half);
    var out: PickOut;
    out.id = in.id;
    out.pos = vec4<f32>(p, clip_depth(p));
    return out;
}

struct PickPointOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) @interpolate(linear) offset: vec2<f32>,
    @location(1) @interpolate(flat) radius: f32,
    @location(3) @interpolate(flat) rte: vec4<f32>,
    @location(7) @interpolate(flat) id: vec2<u32>,
}

@vertex
fn vs_pick_point(
    @builtin(vertex_index) vertex: u32,
    @builtin(instance_index) instance: u32,
) -> PickPointOut {
    let point = point_vertex(vertex, instance);
    var out: PickPointOut;
    out.clip = point.clip;
    out.offset = point.offset;
    out.radius = point.radius;
    out.rte = point.rte;
    let record = instances[slot_of(instance)];
    out.id = vec2<u32>(record.pick, (KIND_VERTEX << 30u) | (geometry_ids[vertex / 6u] & SUB_MASK));
    return out;
}

@fragment
fn fs_pick_point(in: PickPointOut) -> PickOut {
    if in.rte.w > 0.5 && section_clipped(in.rte.xyz) {
        discard;
    }
    if length(in.offset) > max(in.radius, 0.5) + 0.25 {
        discard;
    }
    let p = undo_depth_offset(in.rte.xyz, in.radius);
    var out: PickOut;
    out.id = in.id;
    out.pos = vec4<f32>(p, clip_depth(p));
    return out;
}
