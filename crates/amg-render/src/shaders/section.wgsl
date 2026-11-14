// Section planes (RND-06, design note §10): the per-view uniform and the shared clip and cap
// functions. Appended after `camera.wgsl` and `scene.wgsl` in the mesh and line modules (and the
// cap depth pass); RND-04's pick pipelines reuse `section_clipped`, `section_ray` and
// `section_cap` so a cap pick (`CAP_SUB`) sees exactly the cap that was drawn.
//
// Planes are in eye-relative (RTE) coordinates: an RTE point `x` is clipped when
// `dot(n, x) > d` for any active plane, so the kept region is the intersection of the kept
// half-spaces. DX12 has no `CLIP_DISTANCES`, so the clipped pipeline variants `discard`.

const MAX_SECTIONS: u32 = 6u;

// `amg_render::section::SectionPlaneGpu` (80 bytes).
struct SectionPlaneGpu {
    // xyz = unit normal (world axes), w = d.
    plane: vec4<f32>,
    // Hatch coordinate in periods: u = dot(x, hatch_u.xyz) + hatch_u.w.
    hatch_u: vec4<f32>,
    // The crossing lines (zero without a cross hatch).
    hatch_v: vec4<f32>,
    // Hatch colour, linear RGBA (alpha 0 = no hatch).
    color: vec4<f32>,
    // x = caps (0/1), y = cross (0/1).
    params: vec4<f32>,
}

// `amg_render::section::SectionUniform` (496 bytes).
struct SectionUniform {
    // x = number of active planes.
    info: vec4<u32>,
    planes: array<SectionPlaneGpu, 6>,
}

@group(0) @binding(5) var<uniform> sections: SectionUniform;

// Whether the eye-relative point `x` is removed by an active plane.
fn section_clipped(x: vec3<f32>) -> bool {
    let count = min(sections.info.x, MAX_SECTIONS);
    for (var i = 0u; i < count; i++) {
        let p = sections.planes[i].plane;
        if dot(p.xyz, x) > p.w {
            return true;
        }
    }
    return false;
}

// The view ray through the eye-relative point `x`, as `origin + t * dir` with `x` at t = 1:
// from the eye (perspective) or from the near plane along the view axis (orthographic, whose
// near plane lies behind the eye).
struct SectionRay {
    origin: vec3<f32>,
    dir: vec3<f32>,
}

fn section_ray(x: vec3<f32>) -> SectionRay {
    var ray: SectionRay;
    if camera.ortho == 0u {
        ray.origin = vec3<f32>(0.0);
        ray.dir = x;
    } else {
        // World forward = −(third row of the view rotation).
        let f = -vec3<f32>(camera.view_rte[0].z, camera.view_rte[1].z, camera.view_rte[2].z);
        // View z of the near plane (depth 1): depth = proj[2].z * z + proj[3].z.
        let start = -(1.0 - camera.proj[3].z) / camera.proj[2].z;
        let along = dot(x, f) - start;
        ray.origin = x - f * along;
        ray.dir = f * along;
    }
    return ray;
}

// Where the ray passes through the kept region (design note §10). Only planes whose removed
// side holds the ray origin can be entered; the entry is the largest of their crossing
// parameters. Planes whose kept side holds the origin can only be left; the exit is the
// smallest of those crossings (unbounded for a single plane seen from its removed side).
struct CapHit {
    // The ray starts outside the kept region and passes through it (t <= t_out).
    found: bool,
    // The plane the ray enters through.
    plane: u32,
    // Entry and exit parameters.
    t: f32,
    t_out: f32,
}

fn section_cap(ray: SectionRay) -> CapHit {
    var hit: CapHit;
    hit.found = false;
    hit.plane = 0u;
    hit.t = -1.0;
    hit.t_out = 3.4e38;
    let count = min(sections.info.x, MAX_SECTIONS);
    for (var i = 0u; i < count; i++) {
        let p = sections.planes[i].plane;
        let so = dot(p.xyz, ray.origin) - p.w;
        let nv = dot(p.xyz, ray.dir);
        if so > 0.0 {
            if nv >= 0.0 {
                // Starts on the removed side and moves away: never enters.
                hit.found = false;
                hit.t_out = -1.0;
                return hit;
            }
            let t = -so / nv;
            if t > hit.t {
                hit.t = t;
                hit.plane = i;
                hit.found = true;
            }
        } else if nv > 0.0 {
            hit.t_out = min(hit.t_out, -so / nv);
        }
    }
    hit.found = hit.found && hit.t <= hit.t_out;
    return hit;
}

// Hatch coordinates (periods) of the eye-relative cap point `x` on plane `i`.
fn section_hatch_coords(i: u32, x: vec3<f32>) -> vec2<f32> {
    let p = sections.planes[i];
    return vec2<f32>(dot(x, p.hatch_u.xyz) + p.hatch_u.w, dot(x, p.hatch_v.xyz) + p.hatch_v.w);
}

// Screen gradients (periods per pixel) of both hatch coordinates at the eye-relative point `p`
// on plane `i`, analytically: one pixel along the screen axes at the depth of `p`, projected
// onto the plane along the view ray. No derivatives, so it may run in non-uniform control flow
// (back faces in the mesh pass).
fn section_hatch_gradient(i: u32, p: vec3<f32>) -> vec2<f32> {
    let plane = sections.planes[i];
    let n = plane.plane.xyz;
    let right = vec3<f32>(camera.view_rte[0].x, camera.view_rte[1].x, camera.view_rte[2].x);
    let up = vec3<f32>(camera.view_rte[0].y, camera.view_rte[1].y, camera.view_rte[2].y);
    let forward = -vec3<f32>(camera.view_rte[0].z, camera.view_rte[1].z, camera.view_rte[2].z);
    var ps = camera.pixel_size_at_1mm;
    var r = forward;
    if camera.ortho == 0u {
        ps *= max(dot(p, forward), 0.0);
        r = normalize(p);
    }
    let nr = dot(n, r);
    if abs(nr) < 1e-6 {
        return vec2<f32>(1e9);
    }
    let dx = right * ps - r * (dot(n, right) * ps / nr);
    let dy = up * ps - r * (dot(n, up) * ps / nr);
    let gu = length(vec2<f32>(dot(dx, plane.hatch_u.xyz), dot(dy, plane.hatch_u.xyz)));
    let gv = length(vec2<f32>(dot(dx, plane.hatch_v.xyz), dot(dy, plane.hatch_v.xyz)));
    return vec2<f32>(gu, gv);
}

// Coverage of 1 px hatch lines at coordinate `u` whose screen gradient is `g` periods per px.
// Lines closer than 3 px fade out rather than alias.
fn section_hatch_line(u: f32, g: f32) -> f32 {
    if g <= 0.0 || g > 0.34 {
        return 0.0;
    }
    let f = fract(u);
    let dist_px = min(f, 1.0 - f) / g;
    return clamp(1.0 - dist_px, 0.0, 1.0);
}

// Normal-target mark of a cap sample (read by the cap depth pass and the silhouettes): the
// octahedral x plus 4 (plane + 1). Octahedral values lie in [-1, 1], so marks are > 2.
fn section_mark(oct: vec2<f32>, plane: u32) -> vec2<f32> {
    return vec2<f32>(oct.x + 4.0 * f32(plane + 1u), oct.y);
}
