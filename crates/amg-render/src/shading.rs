//! Lighting models and normal encoding shared by the shaders and their CPU references
//! (design note §4).
//!
//! The WGSL in `shaders/shading.wgsl` implements exactly these formulas with the same constants;
//! the CPU versions here are the references the headless tests compare pixels against.
//!
//! All vectors are in **view space**: x right, y up, z towards the viewer.

use amg_core::glam::{Vec2 as F2, Vec3 as F3};

use crate::scene::Material;

/// How a view lights the model.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Default)]
pub enum Lighting {
    /// PBR-lite: two camera-locked studio lights (key and fill) with GGX specular and Schlick
    /// Fresnel, plus a hemisphere ambient (sky above world +Z, ground below).
    #[default]
    Studio,
    /// A procedural "clay" matcap computed from the view normal (no image asset). Tinted by
    /// the base colour; metallic and roughness are ignored.
    Matcap,
    /// One headlight, diffuse only (Lambert): `base · (0.12 + 0.88 · max(n·v, 0))`. The
    /// cheapest model and the RND-01 look.
    Flat,
}

impl Lighting {
    /// Value of the WGSL `override LIGHTING` constant.
    pub(crate) fn shader_id(self) -> u32 {
        match self {
            Lighting::Studio => 0,
            Lighting::Matcap => 1,
            Lighting::Flat => 2,
        }
    }
}

/// Direction towards the studio key light (view space, unit), upper left in front.
pub(crate) const KEY_DIR: [f32; 3] = [-0.453_413_4, 0.604_551_3, 0.654_930_5];
/// Irradiance of the key light (linear RGB).
pub(crate) const KEY_RGB: [f32; 3] = [2.40, 2.35, 2.28];
/// Direction towards the studio fill light (view space, unit), right, slightly below.
pub(crate) const FILL_DIR: [f32; 3] = [0.652_418_4, -0.150_558_1, 0.742_753_3];
/// Irradiance of the fill light (linear RGB).
pub(crate) const FILL_RGB: [f32; 3] = [0.62, 0.66, 0.72];
/// Hemisphere ambient towards world +Z.
pub(crate) const SKY_RGB: [f32; 3] = [0.30, 0.33, 0.38];
/// Hemisphere ambient towards world −Z.
pub(crate) const GROUND_RGB: [f32; 3] = [0.10, 0.09, 0.08];
/// Share of the specular colour lit by the hemisphere (there is no IBL before RND-10).
pub(crate) const AMBIENT_SPECULAR: f32 = 0.5;
/// Direction towards the matcap's light (view space, unit).
pub(crate) const MATCAP_DIR: [f32; 3] = [-0.400_923_2, 0.551_269_4, 0.731_684_8];
/// Smallest roughness used, so the GGX lobe stays finite.
pub(crate) const MIN_ROUGHNESS: f32 = 0.045;

const PI: f32 = std::f32::consts::PI;

/// Linear RGB of a surface point (CPU reference of the shader, before the target's f16
/// quantisation and the final sRGB encode).
///
/// - `n`: unit surface normal facing the viewer;
/// - `v`: unit direction from the surface to the eye (`(0, 0, 1)` in orthographic views);
/// - `up`: world +Z in view space (for the hemisphere ambient).
///
/// Values above 1 are clamped by the final 8-bit target.
pub fn shade_reference(
    lighting: Lighting,
    material: &Material,
    n: [f32; 3],
    v: [f32; 3],
    up: [f32; 3],
) -> [f32; 3] {
    let n = F3::from_array(n);
    let v = F3::from_array(v);
    let base = F3::new(
        material.base_color[0],
        material.base_color[1],
        material.base_color[2],
    );
    let rgb = match lighting {
        Lighting::Studio => studio(
            n,
            v,
            F3::from_array(up),
            base,
            material.metallic,
            material.roughness,
        ),
        Lighting::Matcap => matcap(n, base),
        Lighting::Flat => base * (0.12 + 0.88 * n.dot(v).max(0.0)),
    };
    rgb.to_array()
}

fn studio(n: F3, v: F3, up: F3, base: F3, metallic: f32, roughness: f32) -> F3 {
    let metallic = metallic.clamp(0.0, 1.0);
    let r = roughness.clamp(MIN_ROUGHNESS, 1.0);
    let a = r * r;
    let a2 = a * a;
    let diffuse = base * (1.0 - metallic);
    let f0 = F3::splat(0.04).lerp(base, metallic);
    let nv = n.dot(v).max(1e-4);
    let k = a * 0.5;
    let gv = nv / (nv * (1.0 - k) + k);
    let mut out = F3::ZERO;
    for (dir, rgb) in [(KEY_DIR, KEY_RGB), (FILL_DIR, FILL_RGB)] {
        let l = F3::from_array(dir);
        let nl = n.dot(l).max(0.0);
        let h = (l + v).normalize();
        let nh = n.dot(h).max(0.0);
        let vh = v.dot(h).max(0.0);
        let m = 1.0 - vh;
        let fresnel = f0 + (F3::ONE - f0) * (m * m * m * m * m);
        let d = nh * nh * (a2 - 1.0) + 1.0;
        let ggx = a2 / (PI * d * d);
        let g = gv * (nl / (nl * (1.0 - k) + k));
        let spec = fresnel * (ggx * g / (4.0 * nl * nv).max(1e-4));
        let brdf = diffuse * (F3::ONE - fresnel) / PI + spec;
        out += brdf * F3::from_array(rgb) * nl;
    }
    let t = 0.5 + 0.5 * n.dot(up);
    let hemi = F3::from_array(GROUND_RGB).lerp(F3::from_array(SKY_RGB), t);
    out + hemi * (diffuse + f0 * AMBIENT_SPECULAR)
}

fn matcap(n: F3, base: F3) -> F3 {
    let l = F3::from_array(MATCAP_DIR);
    let wrap = ((n.dot(l) + 0.35) / 1.35).clamp(0.0, 1.0);
    let h = (l + F3::Z).normalize();
    let s = n.dot(h).max(0.0);
    // s^48 by squaring: s^16 · s^32.
    let s2 = s * s;
    let s4 = s2 * s2;
    let s8 = s4 * s4;
    let s16 = s8 * s8;
    let spec = s16 * s16 * s16 * 0.35;
    let e = 1.0 - n.z.clamp(0.0, 1.0);
    let rim = e * e * e * 0.18;
    base * (0.10 + 0.85 * wrap * wrap) + F3::splat(spec + rim)
}

fn sign_not_zero(x: f32) -> f32 {
    if x >= 0.0 { 1.0 } else { -1.0 }
}

/// Octahedral encoding of a direction into `[-1, 1]²` (Cigolle et al. 2014). The input need
/// not be unit length; a zero vector encodes as `+Z`.
pub fn oct_encode(n: [f32; 3]) -> [f32; 2] {
    let n = F3::from_array(n);
    let l1 = n.x.abs() + n.y.abs() + n.z.abs();
    if l1 <= f32::MIN_POSITIVE || !l1.is_finite() {
        return [0.0, 0.0];
    }
    let p = F2::new(n.x, n.y) / l1;
    let p = if n.z < 0.0 {
        F2::new(
            (1.0 - p.y.abs()) * sign_not_zero(p.x),
            (1.0 - p.x.abs()) * sign_not_zero(p.y),
        )
    } else {
        p
    };
    p.to_array()
}

/// Inverse of [`oct_encode`]: a unit direction.
pub fn oct_decode(e: [f32; 2]) -> [f32; 3] {
    let mut n = F3::new(e[0], e[1], 1.0 - e[0].abs() - e[1].abs());
    let t = (-n.z).max(0.0);
    n.x += if n.x >= 0.0 { -t } else { t };
    n.y += if n.y >= 0.0 { -t } else { t };
    n.normalize().to_array()
}

/// [`oct_encode`] quantised to the `Snorm16x2` vertex format (4 bytes per normal).
pub(crate) fn oct_snorm16(n: [f32; 3]) -> [i16; 2] {
    let q = |v: f32| (v.clamp(-1.0, 1.0) * 32767.0).round() as i16;
    let e = oct_encode(n);
    [q(e[0]), q(e[1])]
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, reason = "test code")]
#[allow(clippy::float_cmp, reason = "exact results of exact inputs")]
mod tests {
    use super::*;

    fn unit(v: [f32; 3]) -> F3 {
        F3::from_array(v).normalize()
    }

    #[test]
    fn light_directions_are_unit_vectors() {
        for d in [KEY_DIR, FILL_DIR, MATCAP_DIR] {
            assert!((F3::from_array(d).length() - 1.0).abs() < 1e-6, "{d:?}");
        }
    }

    /// Angle between two directions, accurate for tiny angles (f32 `acos` near 1 is not).
    fn angle(a: F3, b: F3) -> f32 {
        let (a, b) = (a.as_dvec3(), b.as_dvec3());
        a.cross(b).length().atan2(a.dot(b)) as f32
    }

    #[test]
    fn oct_round_trip_is_accurate_over_the_sphere() {
        let mut worst = 0.0f32;
        let mut worst_q = 0.0f32;
        // A Fibonacci sphere plus the axes and octant diagonals (the fold lines).
        let mut dirs: Vec<F3> = (0..20_000)
            .map(|i| {
                let z = 1.0 - 2.0 * (i as f32 + 0.5) / 20_000.0;
                let r = (1.0 - z * z).sqrt();
                let phi = i as f32 * 2.399_963;
                F3::new(r * phi.cos(), r * phi.sin(), z)
            })
            .collect();
        for x in [-1.0, 0.0, 1.0] {
            for y in [-1.0, 0.0, 1.0] {
                for z in [-1.0, 0.0, 1.0] {
                    let d = F3::new(x, y, z);
                    if d != F3::ZERO {
                        dirs.push(d.normalize());
                    }
                }
            }
        }
        for d in dirs {
            let back = F3::from_array(oct_decode(oct_encode(d.to_array())));
            worst = worst.max(angle(back, d));
            let q = oct_snorm16(d.to_array());
            let e = [f32::from(q[0]) / 32767.0, f32::from(q[1]) / 32767.0];
            let back_q = F3::from_array(oct_decode(e));
            worst_q = worst_q.max(angle(back_q, d));
        }
        assert!(worst < 1e-5, "f32 round trip {worst} rad");
        assert!(worst_q < 1e-4, "Snorm16 round trip {worst_q} rad");
    }

    #[test]
    fn oct_encode_handles_degenerate_input() {
        assert_eq!(oct_encode([0.0; 3]), [0.0, 0.0]);
        assert_eq!(oct_decode([0.0, 0.0]), [0.0, 0.0, 1.0]);
        let e = oct_encode([0.0, 0.0, -5.0]);
        let d = oct_decode(e);
        assert!((F3::from_array(d) - F3::NEG_Z).length() < 1e-6, "{d:?}");
    }

    #[test]
    fn flat_lighting_is_the_headlight_lambert() {
        let m = Material {
            base_color: [0.5, 0.25, 1.0, 1.0],
            ..Material::default()
        };
        let facing = shade_reference(
            Lighting::Flat,
            &m,
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
        );
        assert_eq!(facing, [0.5, 0.25, 1.0]);
        let n = unit([0.0, 0.6, 0.8]);
        let tilted = shade_reference(
            Lighting::Flat,
            &m,
            n.to_array(),
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
        );
        assert!((tilted[0] - 0.5 * (0.12 + 0.88 * 0.8)).abs() < 1e-6);
    }

    #[test]
    fn studio_is_brighter_towards_the_key_and_the_sky() {
        let m = Material::default();
        let v = [0.0, 0.0, 1.0];
        let up = [0.0, 1.0, 0.0];
        let lum = |c: [f32; 3]| c[0] + c[1] + c[2];
        let key = lum(shade_reference(Lighting::Studio, &m, KEY_DIR, v, up));
        let away = lum(shade_reference(
            Lighting::Studio,
            &m,
            unit([0.5, -0.6, 0.62]).to_array(),
            v,
            up,
        ));
        assert!(key > away, "{key} vs {away}");
        // Facing away from every light: only the hemisphere remains; sky above ground.
        let sky = shade_reference(Lighting::Studio, &m, [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], up);
        let ground = shade_reference(Lighting::Studio, &m, [0.0, -1.0, 0.0], [0.0, 0.0, 1.0], up);
        assert!(lum(sky) > lum(ground));
        for c in [key, away] {
            assert!(c.is_finite() && c > 0.0);
        }
    }

    #[test]
    fn studio_rough_dielectric_stays_in_display_range_and_metal_is_tinted() {
        let v = [0.0, 0.0, 1.0];
        let up = [0.0, 1.0, 0.0];
        let white = Material {
            base_color: [1.0; 4],
            metallic: 0.0,
            roughness: 0.8,
            cap_color: None,
        };
        let c = shade_reference(Lighting::Studio, &white, [0.0, 0.0, 1.0], v, up);
        assert!(c.iter().all(|&x| x > 0.5 && x < 1.3), "{c:?}");
        let gold = Material {
            base_color: [1.0, 0.7, 0.2, 1.0],
            metallic: 1.0,
            roughness: 0.3,
            cap_color: None,
        };
        let c = shade_reference(Lighting::Studio, &gold, [0.0, 0.0, 1.0], v, up);
        assert!(
            c[0] > c[1] && c[1] > c[2],
            "metal reflects its own colour: {c:?}"
        );
        // A mirror-like metal at grazing angles stays finite.
        let mirror = Material {
            roughness: 0.0,
            ..gold
        };
        let c = shade_reference(
            Lighting::Studio,
            &mirror,
            unit([0.999, 0.0, 0.04]).to_array(),
            v,
            up,
        );
        assert!(c.iter().all(|x| x.is_finite()), "{c:?}");
    }

    #[test]
    fn matcap_depends_on_the_view_normal_only() {
        let m = Material::default();
        let a = shade_reference(
            Lighting::Matcap,
            &m,
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
        );
        let b = shade_reference(
            Lighting::Matcap,
            &m,
            [0.0, 0.0, 1.0],
            unit([0.3, 0.1, 0.9]).to_array(),
            [1.0, 0.0, 0.0],
        );
        assert_eq!(a, b);
        let lit = shade_reference(
            Lighting::Matcap,
            &m,
            MATCAP_DIR,
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
        );
        let unlit = shade_reference(
            Lighting::Matcap,
            &m,
            unit([0.6, -0.7, 0.2]).to_array(),
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
        );
        assert!(lit[0] > unlit[0]);
    }
}
