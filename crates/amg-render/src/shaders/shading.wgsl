// Lighting models and octahedral normals (design note §4). The CPU references are in
// `amg_render::shading` (`shade_reference`, `oct_encode`, `oct_decode`); the constants KEY_DIR,
// KEY_RGB, FILL_DIR, FILL_RGB, SKY_RGB, GROUND_RGB, AMBIENT_SPECULAR, MATCAP_DIR and
// MIN_ROUGHNESS are generated from that module and prepended, so both sides share one
// definition. All vectors are in view space (x right, y up, z towards the viewer).

const PI: f32 = 3.14159265358979;

// Octahedral encoding into [-1, 1]^2 (Cigolle et al. 2014).
fn oct_encode(n: vec3<f32>) -> vec2<f32> {
    let p = n.xy / (abs(n.x) + abs(n.y) + abs(n.z));
    if n.z < 0.0 {
        let s = select(vec2<f32>(-1.0), vec2<f32>(1.0), p >= vec2<f32>(0.0));
        return (vec2<f32>(1.0) - abs(p.yx)) * s;
    }
    return p;
}

fn oct_decode(e: vec2<f32>) -> vec3<f32> {
    var n = vec3<f32>(e, 1.0 - abs(e.x) - abs(e.y));
    let t = max(-n.z, 0.0);
    n.x += select(t, -t, n.x >= 0.0);
    n.y += select(t, -t, n.y >= 0.0);
    return normalize(n);
}

fn pow5(x: f32) -> f32 {
    let x2 = x * x;
    return x2 * x2 * x;
}

// PBR-lite: key + fill (GGX, Schlick Fresnel, Schlick-GGX visibility with k = a/2) plus a
// hemisphere ambient. `mr` = (metallic, roughness).
fn shade_studio(n: vec3<f32>, v: vec3<f32>, up: vec3<f32>, base: vec3<f32>, mr: vec2<f32>) -> vec3<f32> {
    let metallic = clamp(mr.x, 0.0, 1.0);
    let r = clamp(mr.y, MIN_ROUGHNESS, 1.0);
    let a = r * r;
    let a2 = a * a;
    let diffuse = base * (1.0 - metallic);
    let f0 = mix(vec3<f32>(0.04), base, metallic);
    let nv = max(dot(n, v), 1e-4);
    let k = a * 0.5;
    let gv = nv / (nv * (1.0 - k) + k);
    var out = vec3<f32>(0.0);
    for (var i = 0u; i < 2u; i++) {
        var l = KEY_DIR;
        var rgb = KEY_RGB;
        if i == 1u {
            l = FILL_DIR;
            rgb = FILL_RGB;
        }
        let nl = max(dot(n, l), 0.0);
        let h = normalize(l + v);
        let nh = max(dot(n, h), 0.0);
        let vh = max(dot(v, h), 0.0);
        let fresnel = f0 + (vec3<f32>(1.0) - f0) * pow5(1.0 - vh);
        let d = nh * nh * (a2 - 1.0) + 1.0;
        let ggx = a2 / (PI * d * d);
        let g = gv * (nl / (nl * (1.0 - k) + k));
        let spec = fresnel * (ggx * g / max(4.0 * nl * nv, 1e-4));
        let brdf = diffuse * (vec3<f32>(1.0) - fresnel) / PI + spec;
        out += brdf * rgb * nl;
    }
    let t = 0.5 + 0.5 * dot(n, up);
    let hemi = mix(GROUND_RGB, SKY_RGB, t);
    return out + hemi * (diffuse + f0 * AMBIENT_SPECULAR);
}

// Procedural "clay" matcap from the view normal only.
fn shade_matcap(n: vec3<f32>, base: vec3<f32>) -> vec3<f32> {
    let wrap = clamp((dot(n, MATCAP_DIR) + 0.35) / 1.35, 0.0, 1.0);
    let h = normalize(MATCAP_DIR + vec3<f32>(0.0, 0.0, 1.0));
    let s = max(dot(n, h), 0.0);
    let s2 = s * s;
    let s4 = s2 * s2;
    let s8 = s4 * s4;
    let s16 = s8 * s8;
    let spec = s16 * s16 * s16 * 0.35;
    let e = 1.0 - clamp(n.z, 0.0, 1.0);
    let rim = e * e * e * 0.18;
    return base * (0.10 + 0.85 * wrap * wrap) + vec3<f32>(spec + rim);
}

// Headlight Lambert (the RND-01 look).
fn shade_flat(n: vec3<f32>, v: vec3<f32>, base: vec3<f32>) -> vec3<f32> {
    return base * (0.12 + 0.88 * max(dot(n, v), 0.0));
}

// `lighting`: 0 Studio, 1 Matcap, 2 Flat (`amg_render::Lighting::shader_id`).
fn shade(lighting: u32, n: vec3<f32>, v: vec3<f32>, up: vec3<f32>, base: vec3<f32>, mr: vec2<f32>) -> vec3<f32> {
    if lighting == 1u {
        return shade_matcap(n, base);
    }
    if lighting == 2u {
        return shade_flat(n, v, base);
    }
    return shade_studio(n, v, up, base, mr);
}
