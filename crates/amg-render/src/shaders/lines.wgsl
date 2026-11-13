// Lines, points and the triangle wireframe (RND-12, design note §6). Appended to `camera.wgsl`
// and `scene.wgsl`; drawn inside the opaque pass after the meshes, so the targets are the same:
// 0 = colour (Rgba16Float, alpha blended), 1 = view normal (Rg16Float, write mask empty).
//
// Wide lines: every segment is two triangles (six vertices, `vertex_index / 6` = segment) expanded
// in screen space to the half width plus a 1 px feather and extended by the same amount past both
// ends. The fragment stage evaluates the capsule distance, so joins are round, any width works and
// the edge is antialiased analytically. Native wgpu lines are never used for edges.
//
// Depth offset: each end moves toward the eye by `(LINE_DEPTH_PX + half width)` pixels of world
// size at its depth (view space, along the eye ray, so the screen position is unchanged). This
// scales with distance, so an edge lying on its face wins the GreaterEqual test at any zoom.
//
// Override constants:
//   HIDDEN   false: visible pass (dash and colour of the style); true: hidden pass (depth test
//            Less; the hidden dash and alpha factor of the record).
// The depth compare (GreaterEqual, Less, Always) is pipeline state.

override HIDDEN: bool = false;

// Pixels of world size an edge is moved toward the eye, plus its half width.
const LINE_DEPTH_PX: f32 = 1.5;
// Feather band outside the half width, px (room for the 1 px analytic antialiasing).
const FEATHER_PX: f32 = 1.0;

// Segments of one edge set: two entries per segment, `xyz` = local position relative to the
// edge-set origin, `w` = arc length (mm) from the start of its polyline. The sign bit of `w` marks
// a smooth join at that end (the neighbour continues almost straight; the cap is skipped there so
// translucent lines do not double-blend at every vertex of a tessellated curve).
@group(1) @binding(0) var<storage, read> geometry: array<vec4<f32>>;

fn line_record(instance: u32) -> LineRecord {
    return line_records[visible[instance]];
}

// Eye-relative view-space position of `local` (relative to the record origin).
fn to_view(record: InstanceRecord, local: vec3<f32>) -> vec3<f32> {
    let linear = mat3x3<f32>(record.lin0.xyz, record.lin1.xyz, record.lin2.xyz);
    let t_hi = vec3<f32>(record.lin0.w, record.lin1.w, record.lin2.w);
    let rte = linear * local + eye_relative(camera, t_hi, record.t_lo);
    return (camera.view_rte * vec4<f32>(rte, 1.0)).xyz;
}

// World size of one pixel at view-space position `v`, mm.
fn pixel_size(v: vec3<f32>) -> f32 {
    if camera.ortho != 0u {
        return camera.pixel_size_at_1mm;
    }
    return max(-v.z, 0.0) * camera.pixel_size_at_1mm;
}

// `v` moved `eps` mm toward the eye along its view ray (perspective) or the view axis (ortho).
fn toward_eye(v: vec3<f32>, eps: f32) -> vec3<f32> {
    if camera.ortho != 0u {
        return v + vec3<f32>(0.0, 0.0, eps);
    }
    let len = length(v);
    if len <= 0.0 {
        return v;
    }
    return v * max(1.0 - eps / len, 1e-3);
}

fn depth_offset(v: vec3<f32>, half_width: f32) -> vec3<f32> {
    return toward_eye(v, (LINE_DEPTH_PX + half_width) * pixel_size(v));
}

// Screen position in pixels (origin at the view centre, y up) of a clip-space point.
fn to_px(clip: vec4<f32>) -> vec2<f32> {
    return clip.xy / clip.w * 0.5 * camera.size_px;
}

fn from_px(px: vec2<f32>, clip: vec4<f32>) -> vec4<f32> {
    return vec4<f32>(px / (0.5 * camera.size_px) * clip.w, clip.z, clip.w);
}

fn is_smooth(w: f32) -> bool {
    return (bitcast<u32>(w) >> 31u) != 0u;
}

struct LineOut {
    @builtin(position) clip: vec4<f32>,
    // x = px along the segment from its start, y = px across (signed).
    @location(0) @interpolate(linear) coord: vec2<f32>,
    // x = length px, y = half width px, z/w = dash coordinate (px) at the start / end.
    @location(1) @interpolate(flat) shape: vec4<f32>,
    // xy = dash (on, off) px; z = 1 if the start join is smooth, w = 1 if the end join is.
    @location(2) @interpolate(flat) dash: vec4<f32>,
    @location(3) @interpolate(flat) color: vec4<f32>,
}

// Corners of the quad as (end, side): two triangles (0, 1, 2) and (2, 1, 3).
fn quad_corner(i: u32) -> vec2<f32> {
    switch i {
        case 0u: { return vec2<f32>(0.0, -1.0); }
        case 1u: { return vec2<f32>(0.0, 1.0); }
        case 2u, 3u: { return vec2<f32>(1.0, -1.0); }
        case 4u: { return vec2<f32>(0.0, 1.0); }
        default: { return vec2<f32>(1.0, 1.0); }
    }
}

@vertex
fn vs_line(
    @builtin(vertex_index) vertex: u32,
    @builtin(instance_index) instance: u32,
) -> LineOut {
    var out: LineOut;
    let record = instances[visible[instance]];
    let lines = line_record(instance);
    let seg = vertex / 6u;
    let corner = quad_corner(vertex % 6u);
    let a = geometry[2u * seg];
    let b = geometry[2u * seg + 1u];
    let half = 0.5 * lines.edge_delta.w;
    var va = depth_offset(to_view(record, a.xyz + lines.edge_delta.xyz), half);
    var vb = depth_offset(to_view(record, b.xyz + lines.edge_delta.xyz), half);

    // Clip the segment against the near plane (perspective; `proj[3].z` is the near distance).
    if camera.ortho == 0u {
        let near_z = -camera.proj[3].z * 1.0001;
        let a_in = va.z <= near_z;
        let b_in = vb.z <= near_z;
        if !a_in && !b_in {
            out.clip = vec4<f32>(0.0, 0.0, 0.0, 1.0);
            out.color = vec4<f32>(0.0);
            return out;
        }
        if !a_in {
            va = mix(va, vb, (near_z - va.z) / (vb.z - va.z));
        } else if !b_in {
            vb = mix(vb, va, (near_z - vb.z) / (va.z - vb.z));
        }
    }

    let ca = camera.proj * vec4<f32>(va, 1.0);
    let cb = camera.proj * vec4<f32>(vb, 1.0);
    let sa = to_px(ca);
    let sb = to_px(cb);
    let d = sb - sa;
    let len = length(d);
    var dir = vec2<f32>(1.0, 0.0);
    if len > 1e-4 {
        dir = d / len;
    }
    let normal = vec2<f32>(-dir.y, dir.x);
    let ext = half + FEATHER_PX;
    var px: vec2<f32>;
    var clip: vec4<f32>;
    if corner.x == 0.0 {
        px = sa - dir * ext;
        clip = ca;
    } else {
        px = sb + dir * ext;
        clip = cb;
    }
    px += normal * corner.y * ext;
    out.clip = from_px(px, clip);
    out.coord = vec2<f32>(dot(px - sa, dir), corner.y * ext);

    // Dash coordinate: arc length in pixels of world size at each end (approximate under strong
    // perspective; exact for orthographic views).
    let da = abs(a.w) / max(pixel_size(va), 1e-30);
    let db = abs(b.w) / max(pixel_size(vb), 1e-30);
    out.shape = vec4<f32>(len, half, da, db);
    var color = lines.edge_color;
    var dash = lines.dash.xy;
    if HIDDEN {
        dash = lines.dash.zw;
        color.a *= lines.hidden.x;
    }
    out.dash = vec4<f32>(dash, select(0.0, 1.0, is_smooth(a.w)), select(0.0, 1.0, is_smooth(b.w)));
    out.color = color;
    return out;
}

// Coverage of a dash pattern at coordinate `u` px: 1 inside an "on" run, 0 inside an "off" run,
// with a 1 px linear ramp at the run ends.
fn dash_coverage(u: f32, on: f32, off: f32) -> f32 {
    if on <= 0.0 || off <= 0.0 {
        return 1.0;
    }
    let period = on + off;
    let m = u - floor(u / period) * period;
    var s: f32;
    if m < on {
        s = min(m, on - m);
    } else {
        s = -min(m - on, period - m);
    }
    return clamp(s + 0.5, 0.0, 1.0);
}

struct LineFsOut {
    @location(0) color: vec4<f32>,
    @location(1) normal: vec2<f32>,
}

@fragment
fn fs_line(in: LineOut) -> LineFsOut {
    let len = in.shape.x;
    let half = in.shape.y;
    let along = in.coord.x;
    // A smooth join leaves its cap to the neighbouring segment.
    if (along < 0.0 && in.dash.z > 0.5) || (along > len && in.dash.w > 0.5) {
        discard;
    }
    let t = clamp(along, 0.0, len);
    let dist = length(vec2<f32>(along - t, in.coord.y));
    var coverage = clamp(half + 0.5 - dist, 0.0, 1.0);
    var u = in.shape.z;
    if len > 0.0 {
        u = mix(in.shape.z, in.shape.w, t / len);
    }
    coverage *= dash_coverage(u, in.dash.x, in.dash.y);
    if coverage <= 0.0 {
        discard;
    }
    var out: LineFsOut;
    out.color = vec4<f32>(in.color.rgb, in.color.a * coverage);
    out.normal = vec2<f32>(0.0);
    return out;
}

// Point sprites: a screen-aligned square per point (six vertices) with a circle distance.
struct PointOut {
    @builtin(position) clip: vec4<f32>,
    // Offset from the centre, px.
    @location(0) @interpolate(linear) offset: vec2<f32>,
    // x = radius px.
    @location(1) @interpolate(flat) radius: f32,
    @location(2) @interpolate(flat) color: vec4<f32>,
}

@vertex
fn vs_point(
    @builtin(vertex_index) vertex: u32,
    @builtin(instance_index) instance: u32,
) -> PointOut {
    var out: PointOut;
    let record = instances[visible[instance]];
    let lines = line_record(instance);
    let p = geometry[vertex / 6u];
    let corner = quad_corner(vertex % 6u) * vec2<f32>(2.0, 1.0) - vec2<f32>(1.0, 0.0);
    let radius = 0.5 * lines.point_delta.w;
    let v = depth_offset(to_view(record, p.xyz + lines.point_delta.xyz), radius);
    if camera.ortho == 0u && v.z > -camera.proj[3].z * 1.0001 {
        out.clip = vec4<f32>(0.0, 0.0, 0.0, 1.0);
        out.color = vec4<f32>(0.0);
        return out;
    }
    let clip = camera.proj * vec4<f32>(v, 1.0);
    let ext = radius + FEATHER_PX;
    let px = to_px(clip) + corner * ext;
    out.clip = from_px(px, clip);
    out.offset = corner * ext;
    out.radius = radius;
    var color = lines.point_color;
    if HIDDEN {
        color.a *= lines.hidden.y;
    }
    out.color = color;
    return out;
}

@fragment
fn fs_point(in: PointOut) -> LineFsOut {
    let coverage = clamp(in.radius + 0.5 - length(in.offset), 0.0, 1.0);
    if coverage <= 0.0 {
        discard;
    }
    var out: LineFsOut;
    out.color = vec4<f32>(in.color.rgb, in.color.a * coverage);
    out.normal = vec2<f32>(0.0);
    return out;
}

// Triangle wireframe of a mesh (`POLYGON_MODE_LINE`, 1 px native lines): the mesh positions,
// moved toward the eye like the wide lines so they win against their own faces.
struct WireOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) @interpolate(flat) color: vec4<f32>,
}

@vertex
fn vs_wire(
    @location(0) position: vec3<f32>,
    @builtin(instance_index) instance: u32,
) -> WireOut {
    let record = instances[visible[instance]];
    let lines = line_record(instance);
    let v = depth_offset(to_view(record, position), 0.5);
    var out: WireOut;
    out.clip = camera.proj * vec4<f32>(v, 1.0);
    var color = lines.edge_color;
    if HIDDEN {
        color.a *= lines.hidden.x;
    }
    out.color = color;
    return out;
}

@fragment
fn fs_wire(in: WireOut) -> LineFsOut {
    var out: LineFsOut;
    out.color = in.color;
    out.normal = vec2<f32>(0.0);
    return out;
}
