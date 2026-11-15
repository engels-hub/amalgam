// Analysis formulas shared by the analysis pipelines (`analysis.wgsl`) and the GPU test kernels
// (`amg_render::analysis::testing`): design note §11, one definition. No bindings here. The
// colour and flag constants (NAN_COLOR, ZEBRA_LIGHT, FLAG_*, ...) are generated from
// `amg_render::analysis` and prepended.

const ANALYSIS_PI: f32 = 3.14159265358979;
const ANALYSIS_TAU: f32 = 6.28318530717959;

const MODE_FIELD: u32 = 1u;
const MODE_ZEBRA: u32 = 2u;
const MODE_DRAFT: u32 = 3u;
const MODE_OVERHANG: u32 = 4u;

const CLASS_SUPPORTED: u32 = 0u;
const CLASS_OVERHANG: u32 = 1u;
const CLASS_CONTACT: u32 = 2u;
const CLASS_DEGENERATE: u32 = 3u;

// One record per instance (128 bytes, `amg_render::analysis::AnalysisRecord`).
struct AnalysisRecord {
    // x = mode, y = flags, z = colormap layer, w = bands (0 = continuous).
    mode: vec4<u32>,
    // World direction (draft pull, overhang build direction); w = zebra stripes or cos α.
    dir: vec4<f32>,
    // Overhang: height above the plate = dot(local, xyz) + w.
    height: vec4<f32>,
    // Field (min, max, isolines, 0); draft (max angle, required, 0, 0); overhang (tol, 0, 0, 0).
    params: vec4<f32>,
    // Out-of-range colours; w < 0 = clamp.
    below: vec4<f32>,
    above: vec4<f32>,
    // (scale, animation kind 0 static / 1 sine / 2 ramp, period s, 0).
    deform: vec4<f32>,
    reserved: vec4<f32>,
}

// Overhang class of S16 AM-02 (`docs/notes/am-build-prep.md` §1.2, evaluated in this order):
// degenerate for a zero normal; with c = −n·d, contact when c > 0 and height ≤ tol; overhang
// when c > cos α (strict); else supported. `n` and `d` need not be unit length.
fn overhang_class(n: vec3<f32>, d: vec3<f32>, cos_critical: f32, height: f32, tol: f32) -> u32 {
    if dot(n, n) == 0.0 {
        return CLASS_DEGENERATE;
    }
    let c = -dot(normalize(n), normalize(d));
    if c > 0.0 && height <= tol {
        return CLASS_CONTACT;
    }
    if c > cos_critical {
        return CLASS_OVERHANG;
    }
    return CLASS_SUPPORTED;
}

// S16 severity: clamp((c − cos α) / (1 − cos α), 0, 1).
fn overhang_severity(c: f32, cos_critical: f32) -> f32 {
    return clamp((c - cos_critical) / max(1.0 - cos_critical, 1e-6), 0.0, 1.0);
}

// Zebra parameter t = acos(r·a) / π × N with r = reflect(−v, n); light where fract(t) ≥ 0.5.
fn zebra_t(n: vec3<f32>, v: vec3<f32>, axis: vec3<f32>, stripes: f32) -> f32 {
    let r = reflect(-v, n);
    return acos(clamp(dot(r, axis), -1.0, 1.0)) / ANALYSIS_PI * stripes;
}

// Antialiased light coverage of the stripe at `t` with screen footprint `fw` = fwidth(t).
fn zebra_light(t: f32, fw: f32) -> f32 {
    let x = fract(t);
    // Distance to the nearest stripe edge (at fract 0 and 0.5), in units of t.
    let e = min(abs(x - 0.5), min(x, 1.0 - x));
    let s = select(-1.0, 1.0, x >= 0.5);
    return clamp(0.5 + s * e / max(fw, 1e-6), 0.0, 1.0);
}

// Draft angle β = asin(clamp(n·p, −1, 1)).
fn draft_angle(n: vec3<f32>, pull: vec3<f32>) -> f32 {
    return asin(clamp(dot(n, pull), -1.0, 1.0));
}

// Deformation amplitude s · A(τ) of `deform` (scale, kind, period, 0) at time `time`.
fn deform_amplitude(deform: vec4<f32>, time: f32) -> f32 {
    let kind = u32(deform.y + 0.5);
    let period = max(deform.z, 1e-6);
    var a = 1.0;
    if kind == 1u {
        a = sin(ANALYSIS_TAU * time / period);
    } else if kind == 2u {
        a = min(time / period, 1.0);
    }
    return deform.x * a;
}

// p' = p + s · A(τ) · d (design note §11).
fn deformed_position(p: vec3<f32>, d: vec3<f32>, deform: vec4<f32>, time: f32) -> vec3<f32> {
    return p + deform_amplitude(deform, time) * d;
}

// Texture coordinate of t ∈ [0, 1] in a 256-entry LUT: entry centres at (i + 0.5) / 256, so a
// linear sample interpolates entries i and i + 1 as `colormap_rgb` does.
fn colormap_u(t: f32) -> f32 {
    return (clamp(t, 0.0, 1.0) * 255.0 + 0.5) / 256.0;
}

// Field value → colormap parameter: (v − min) / (max − min), banded to the centre of its band
// when `bands` > 0.
fn field_t(v: f32, lo: f32, hi: f32, bands: u32) -> f32 {
    let span = hi - lo;
    var t = select(0.5, (v - lo) / span, span != 0.0);
    t = clamp(t, 0.0, 1.0);
    if bands > 0u {
        let n = f32(bands);
        t = (floor(min(t * n, n - 0.5)) + 0.5) / n;
    }
    return t;
}

// Isoline coverage at t·N with screen footprint `fw` = fwidth(t·N).
fn isoline(tn: f32, fw: f32) -> f32 {
    let f = fract(tn);
    return 1.0 - smoothstep(0.0, 1.5 * max(fw, 1e-6), min(f, 1.0 - f));
}
