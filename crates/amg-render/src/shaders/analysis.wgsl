// Analysis and field pipelines (RND-08, design note §11). Appended to the mesh module
// (`mesh.wgsl`: `transform`, `FsOut`, caps, `cap_color`) and `analysis_common.wgsl`.
//
// Override constants (besides LIGHTING, CLIP and CAPS of `mesh.wgsl`):
//   MODE        1 field, 2 zebra, 3 draft, 4 overhang (`amg_render::analysis::ShadeMode`).
//   HAS_SCALAR  a scalar field is bound (group 2); off, a field instance keeps its material
//               colour.
//   HAS_DEFORM  a displacement field is bound (group 3).
//   OUT         0 lit, 1 unlit, 2 classes (`amg_render::AnalysisOutput`).
//
// Fields are read by vertex index (`vertex_index` of the indexed draw is the index value), so
// the streams need no vertex-buffer layouts and a mesh without fields pays nothing.

override MODE: u32 = 1u;
override HAS_SCALAR: bool = false;
override HAS_DEFORM: bool = false;
override OUT: u32 = 0u;

@group(1) @binding(0) var<storage, read> analysis: array<AnalysisRecord>;
@group(1) @binding(1) var lut: texture_2d_array<f32>;
@group(1) @binding(2) var lut_sampler: sampler;
// x = animation time τ, seconds.
@group(1) @binding(3) var<uniform> analysis_globals: vec4<f32>;
@group(2) @binding(0) var<storage, read> scalars: array<f32>;
// Three floats per vertex.
@group(3) @binding(0) var<storage, read> displacements: array<f32>;

struct AnalysisVsOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) view_pos: vec3<f32>,
    // View-space normal (zero from `vs_analysis_flat`).
    @location(1) view_normal: vec3<f32>,
    @location(2) color: vec4<f32>,
    @location(3) @interpolate(flat) material: vec2<f32>,
    @location(4) rte: vec3<f32>,
    @location(5) @interpolate(flat) slot: u32,
    // Mesh-local position (after deformation).
    @location(6) local: vec3<f32>,
    // World-space normal (normal matrix applied, not normalised; zero when flat).
    @location(7) world_normal: vec3<f32>,
    @location(8) scalar: f32,
}

fn analysis_vertex(position: vec3<f32>, vid: u32, instance: u32) -> AnalysisVsOut {
    let slot = visible[instance];
    let record = instances[slot];
    var local = position;
    if HAS_DEFORM {
        let d = vec3<f32>(displacements[3u * vid], displacements[3u * vid + 1u], displacements[3u * vid + 2u]);
        local = deformed_position(position, d, analysis[slot].deform, analysis_globals.x);
    }
    let base = transform(local, record);
    var out: AnalysisVsOut;
    out.clip = base.clip;
    out.view_pos = base.view_pos;
    out.view_normal = vec3<f32>(0.0);
    out.color = base.color;
    out.material = base.material;
    out.rte = base.rte;
    out.slot = slot;
    out.local = local;
    out.world_normal = vec3<f32>(0.0);
    out.scalar = 0.0;
    if HAS_SCALAR {
        out.scalar = scalars[vid];
    }
    return out;
}

@vertex
fn vs_analysis_smooth(
    @location(0) position: vec3<f32>,
    @location(1) normal_oct: vec2<f32>,
    @builtin(vertex_index) vid: u32,
    @builtin(instance_index) instance: u32,
) -> AnalysisVsOut {
    var out = analysis_vertex(position, vid, instance);
    let record = instances[out.slot];
    let normal_matrix = mat3x3<f32>(record.nrm0.xyz, record.nrm1.xyz, record.nrm2.xyz);
    let world = normal_matrix * oct_decode(normal_oct);
    out.world_normal = world;
    out.view_normal = (camera.view_rte * vec4<f32>(world, 0.0)).xyz;
    return out;
}

@vertex
fn vs_analysis_flat(
    @location(0) position: vec3<f32>,
    @builtin(vertex_index) vid: u32,
    @builtin(instance_index) instance: u32,
) -> AnalysisVsOut {
    return analysis_vertex(position, vid, instance);
}

// World direction of a view-space vector (`view_rte` is a rotation).
fn view_to_world(v: vec3<f32>) -> vec3<f32> {
    let r = mat3x3<f32>(camera.view_rte[0].xyz, camera.view_rte[1].xyz, camera.view_rte[2].xyz);
    return transpose(r) * v;
}

fn lut_rgb(layer: u32, t: f32) -> vec3<f32> {
    return textureSampleLevel(lut, lut_sampler, vec2<f32>(colormap_u(t), 0.5), layer, 0.0).rgb;
}

@fragment
fn fs_analysis(in: AnalysisVsOut, @builtin(front_facing) front: bool) -> FsOut {
    // Derivatives first, in uniform control flow. Both facets face the viewer.
    let facet_view = normalize(cross(dpdy(in.view_pos), dpdx(in.view_pos)));
    let facet_world = normalize(cross(dpdy(in.rte), dpdx(in.rte)));
    let rec = analysis[in.slot];
    let side = select(-1.0, 1.0, front);
    let has_smooth = dot(in.world_normal, in.world_normal) > 1e-20;
    // Normals: `n_*` face the viewer (lighting, zebra); `geo` is the outward surface normal
    // (draft; the overhang uses the flat facet, as S16 asks).
    var n_view = facet_view;
    var n_world = facet_world;
    var geo = facet_world * side;
    if has_smooth {
        geo = normalize(in.world_normal);
        n_world = geo * side;
        n_view = normalize(in.view_normal) * side;
    }
    var v_view = vec3<f32>(0.0, 0.0, 1.0);
    if camera.ortho == 0u {
        v_view = normalize(-in.view_pos);
    }

    // The quantity whose screen footprint antialiases stripes or isolines.
    var aa_t = 0.0;
    if MODE == MODE_ZEBRA {
        let vertical = (rec.mode.y & FLAG_ZEBRA_VERTICAL) != 0u;
        if (rec.mode.y & FLAG_ZEBRA_WORLD) != 0u {
            let axis = select(vec3<f32>(0.0, 0.0, 1.0), vec3<f32>(1.0, 0.0, 0.0), vertical);
            aa_t = zebra_t(n_world, view_to_world(v_view), axis, rec.dir.w);
        } else {
            let axis = select(vec3<f32>(0.0, 1.0, 0.0), vec3<f32>(1.0, 0.0, 0.0), vertical);
            aa_t = zebra_t(n_view, v_view, axis, rec.dir.w);
        }
    } else if MODE == MODE_FIELD {
        let span = rec.params.y - rec.params.x;
        let t = select(0.5, (in.scalar - rec.params.x) / span, span != 0.0);
        aa_t = t * rec.params.z;
    }
    let aa_fw = fwidth(aa_t);

    if CAPS && !front {
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

    var rgb = in.color.rgb;
    var lit = OUT == 0u;
    var cls = 0u;
    if MODE == MODE_FIELD {
        if HAS_SCALAR {
            let v = in.scalar;
            let lo = rec.params.x;
            let hi = rec.params.y;
            if v != v {
                rgb = NAN_COLOR;
            } else if v < lo && rec.below.w >= 0.0 {
                if rec.below.w < 0.5 {
                    discard;
                }
                rgb = rec.below.rgb;
            } else if v > hi && rec.above.w >= 0.0 {
                if rec.above.w < 0.5 {
                    discard;
                }
                rgb = rec.above.rgb;
            } else {
                rgb = lut_rgb(rec.mode.z, field_t(v, lo, hi, rec.mode.w));
                if rec.params.z > 0.0 && v >= lo && v <= hi {
                    rgb = mix(rgb, ISOLINE_COLOR, ISOLINE_ALPHA * isoline(aa_t, aa_fw));
                }
            }
        }
    } else if MODE == MODE_ZEBRA {
        let light = zebra_light(aa_t, aa_fw);
        rgb = mix(ZEBRA_DARK, ZEBRA_LIGHT, light);
        cls = select(0u, 1u, light >= 0.5);
        lit = false;
    } else if MODE == MODE_DRAFT {
        let beta = draft_angle(geo, rec.dir.xyz);
        let t = clamp(beta / max(rec.params.x, 1e-6) * 0.5 + 0.5, 0.0, 1.0);
        rgb = lut_rgb(rec.mode.z, t);
        cls = select(1u, 0u, beta > 0.0);
        if abs(beta) < rec.params.y {
            rgb = DRAFT_INSUFFICIENT_COLOR;
            cls = 2u;
        }
    } else if MODE == MODE_OVERHANG {
        let n = facet_world * side;
        let height = dot(in.local, rec.height.xyz) + rec.height.w;
        cls = overhang_class(n, rec.dir.xyz, rec.dir.w, height, rec.params.x);
        if cls == CLASS_OVERHANG {
            let c = -dot(n, normalize(rec.dir.xyz));
            rgb = mix(OVERHANG_LOW_COLOR, OVERHANG_HIGH_COLOR, overhang_severity(c, rec.dir.w));
        } else if cls == CLASS_CONTACT {
            rgb = CONTACT_COLOR;
        }
    }

    var out: FsOut;
    if OUT == 2u {
        out.color = vec4<f32>(f32(cls) * 0.25, 0.0, 0.0, 1.0);
    } else if lit {
        let up = (camera.view_rte * vec4<f32>(0.0, 0.0, 1.0, 0.0)).xyz;
        out.color = vec4<f32>(shade(LIGHTING, n_view, v_view, up, rgb, in.material), in.color.a);
    } else {
        out.color = vec4<f32>(rgb, in.color.a);
    }
    out.normal = oct_encode(n_view);
    return out;
}
