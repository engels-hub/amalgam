// Overlay handles in the point-pick pass (RND-07). Appended to the pick module (`camera.wgsl`,
// `scene.wgsl`, `section.wgsl`, `lines.wgsl`, `pick.wgsl`): handles are lines, points and
// triangles of overlay records with a non-zero pick id. Their id texel is
// `(pick, KIND_HANDLE << 30 | sub)`, the sub-id riding in the record's `material` lane (lines and
// points) or the vertex `pick` attribute (triangles). Always-on-top and x-ray handles write the
// nearest depth, so they win over the scene; depth-tested ones use their own depth.

const KIND_HANDLE: u32 = 3u;
const OVERLAY_MODE_TEST: u32 = 0u;

struct OverlayPickOut {
    @location(0) id: vec2<u32>,
    @location(1) pos: vec4<f32>,
    @builtin(frag_depth) depth: f32,
}

fn handle_depth(mode: u32, z: f32) -> f32 {
    if mode == OVERLAY_MODE_TEST {
        return z;
    }
    return 1.0;
}

struct OverlayPickLineOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) @interpolate(linear) coord: vec2<f32>,
    @location(1) @interpolate(flat) shape: vec4<f32>,
    @location(2) @interpolate(flat) dash: vec4<f32>,
    @location(4) @interpolate(flat) rte_a: vec3<f32>,
    @location(5) @interpolate(flat) rte_b: vec3<f32>,
    @location(6) @interpolate(flat) section: vec3<f32>,
    @location(7) @interpolate(flat) id: vec2<u32>,
    @location(8) @interpolate(flat) mode: u32,
}

@vertex
fn vs_pick_overlay_line(@builtin(vertex_index) vertex: u32) -> OverlayPickLineOut {
    let slot = geometry_ids[vertex / 6u];
    let line = line_vertex_slot(vertex, slot);
    let record = instances[slot];
    var out: OverlayPickLineOut;
    out.clip = line.clip;
    if record.pick == 0u {
        out.clip = vec4<f32>(0.0, 0.0, 0.0, 1.0);
    }
    out.coord = line.coord;
    out.shape = line.shape;
    out.dash = line.dash;
    out.rte_a = line.rte_a;
    out.rte_b = line.rte_b;
    out.section = line.section;
    out.id = vec2<u32>(record.pick, (KIND_HANDLE << 30u) | (record.material & SUB_MASK));
    out.mode = u32(line_records[slot].hidden.w + 0.5);
    return out;
}

@fragment
fn fs_pick_overlay_line(in: OverlayPickLineOut) -> OverlayPickOut {
    var p: PickLineOut;
    p.clip = in.clip;
    p.coord = in.coord;
    p.shape = in.shape;
    p.dash = in.dash;
    p.rte_a = in.rte_a;
    p.rte_b = in.rte_b;
    p.section = in.section;
    p.id = in.id;
    let hit = pick_line(p);
    var out: OverlayPickOut;
    out.id = hit.id;
    out.pos = hit.pos;
    out.depth = handle_depth(in.mode, in.clip.z);
    return out;
}

struct OverlayPickPointOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) @interpolate(linear) offset: vec2<f32>,
    @location(1) @interpolate(flat) radius: f32,
    @location(3) @interpolate(flat) rte: vec4<f32>,
    @location(7) @interpolate(flat) id: vec2<u32>,
    @location(8) @interpolate(flat) mode: u32,
}

@vertex
fn vs_pick_overlay_point(@builtin(vertex_index) vertex: u32) -> OverlayPickPointOut {
    let slot = geometry_ids[vertex / 6u];
    let point = point_vertex_slot(vertex, slot);
    let record = instances[slot];
    var out: OverlayPickPointOut;
    out.clip = point.clip;
    if record.pick == 0u {
        out.clip = vec4<f32>(0.0, 0.0, 0.0, 1.0);
    }
    out.offset = point.offset;
    out.radius = point.radius;
    out.rte = point.rte;
    out.id = vec2<u32>(record.pick, (KIND_HANDLE << 30u) | (record.material & SUB_MASK));
    out.mode = u32(line_records[slot].hidden.w + 0.5);
    return out;
}

@fragment
fn fs_pick_overlay_point(in: OverlayPickPointOut) -> OverlayPickOut {
    var p: PickPointOut;
    p.clip = in.clip;
    p.offset = in.offset;
    p.radius = in.radius;
    p.rte = in.rte;
    p.id = in.id;
    let hit = pick_point(p);
    var out: OverlayPickOut;
    out.id = hit.id;
    out.pos = hit.pos;
    out.depth = handle_depth(in.mode, in.clip.z);
    return out;
}

struct OverlayTriIn {
    @location(0) pos: vec3<f32>,
    @location(3) params: vec4<f32>,
    @location(4) pick: vec2<u32>,
}

struct OverlayPickTriOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) rte: vec3<f32>,
    @location(1) @interpolate(flat) id: vec2<u32>,
    @location(2) @interpolate(flat) mode: u32,
}

@vertex
fn vs_pick_overlay_tri(in: OverlayTriIn) -> OverlayPickTriOut {
    var out: OverlayPickTriOut;
    out.clip = camera.proj * (camera.view_rte * vec4<f32>(in.pos, 1.0));
    if in.pick.x == 0u {
        out.clip = vec4<f32>(0.0, 0.0, 0.0, 1.0);
    }
    out.rte = in.pos;
    out.id = vec2<u32>(in.pick.x, (KIND_HANDLE << 30u) | (in.pick.y & SUB_MASK));
    out.mode = u32(in.params.y + 0.5);
    return out;
}

@fragment
fn fs_pick_overlay_tri(in: OverlayPickTriOut) -> OverlayPickOut {
    var out: OverlayPickOut;
    out.id = in.id;
    out.pos = vec4<f32>(in.rte, in.clip.z);
    out.depth = handle_depth(in.mode, in.clip.z);
    return out;
}
