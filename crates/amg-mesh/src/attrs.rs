//! Optional mesh attributes in the vertex, corner and face domains.
//!
//! Attribute policy (every op documents deviations):
//! - vertex attributes are interpolated at new vertices;
//! - corner attributes are interpolated inside the parent face;
//! - face attributes are inherited from the parent face.

use amg_core::{Point2, Vec3};

use crate::INVALID;
use crate::error::{MeshError, MeshResult};

/// sRGB colour with linear alpha.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub struct Rgba8(pub [u8; 4]);

impl Rgba8 {
    /// Opaque white, the colour of vertices created without a parent.
    pub const WHITE: Rgba8 = Rgba8([255; 4]);

    /// Component-wise linear interpolation in byte space, rounded to nearest.
    pub fn lerp(self, o: Rgba8, t: f64) -> Rgba8 {
        let t = t.clamp(0.0, 1.0);
        let mut out = [0u8; 4];
        for (i, c) in out.iter_mut().enumerate() {
            let v = f64::from(self.0[i]) + (f64::from(o.0[i]) - f64::from(self.0[i])) * t;
            // In [0, 255] by construction.
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            {
                *c = v.round() as u8;
            }
        }
        Rgba8(out)
    }
}

/// 3MF triangle property: property group id + per-corner index (all equal when uniform).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub struct TriProperty {
    /// 3MF property group (resource) id.
    pub group: u32,
    /// Property index per corner.
    pub index: [u32; 3],
}

impl TriProperty {
    /// "No property": used for faces created without a parent face.
    pub const NONE: TriProperty = TriProperty {
        group: INVALID,
        index: [INVALID; 3],
    };
}

/// Optional attribute arrays. `None` means the attribute is absent.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct MeshAttrs {
    /// Per-vertex normals; `len == positions.len()`.
    pub vertex_normals: Option<Vec<Vec3>>,
    /// Per-vertex colours; `len == positions.len()`.
    pub vertex_colors: Option<Vec<Rgba8>>,
    /// Per-corner UVs; `len == tris.len()`.
    pub corner_uvs: Option<Vec<[Point2; 3]>>,
    /// Per-face 3MF property references; `len == tris.len()`.
    pub tri_properties: Option<Vec<TriProperty>>,
}

fn check_len(name: &'static str, got: Option<usize>, expected: usize) -> MeshResult<()> {
    match got {
        Some(got) if got != expected => Err(MeshError::AttributeLength {
            name,
            got,
            expected,
        }),
        _ => Ok(()),
    }
}

impl MeshAttrs {
    /// Checks every present array against the vertex and face counts.
    pub fn validate(&self, vertices: usize, faces: usize) -> MeshResult<()> {
        check_len(
            "vertex_normals",
            self.vertex_normals.as_ref().map(Vec::len),
            vertices,
        )?;
        check_len(
            "vertex_colors",
            self.vertex_colors.as_ref().map(Vec::len),
            vertices,
        )?;
        check_len("corner_uvs", self.corner_uvs.as_ref().map(Vec::len), faces)?;
        check_len(
            "tri_properties",
            self.tri_properties.as_ref().map(Vec::len),
            faces,
        )
    }

    /// True if no attribute is present.
    pub fn is_empty(&self) -> bool {
        self.vertex_normals.is_none()
            && self.vertex_colors.is_none()
            && self.corner_uvs.is_none()
            && self.tri_properties.is_none()
    }

    /// Appends a vertex whose attributes copy vertex `src`.
    pub(crate) fn push_vertex_copy(&mut self, src: usize) {
        if let Some(n) = &mut self.vertex_normals {
            n.push(n[src]);
        }
        if let Some(c) = &mut self.vertex_colors {
            c.push(c[src]);
        }
    }

    /// Appends a vertex without a parent (zero normal, white).
    pub(crate) fn push_vertex_default(&mut self) {
        if let Some(n) = &mut self.vertex_normals {
            n.push(Vec3::ZERO);
        }
        if let Some(c) = &mut self.vertex_colors {
            c.push(Rgba8::WHITE);
        }
    }

    /// Appends a vertex interpolated between `a` and `b` at parameter `t` in [0, 1].
    pub(crate) fn push_vertex_lerp(&mut self, a: usize, b: usize, t: f64) {
        if let Some(n) = &mut self.vertex_normals {
            let v = n[a].lerp(n[b], t);
            n.push(v.try_normalize().unwrap_or(v));
        }
        if let Some(c) = &mut self.vertex_colors {
            let v = c[a].lerp(c[b], t);
            c.push(v);
        }
    }

    /// Sets vertex `dst` to the interpolation between `a` and `b` at `t`.
    pub(crate) fn set_vertex_lerp(&mut self, dst: usize, a: usize, b: usize, t: f64) {
        if let Some(n) = &mut self.vertex_normals {
            let v = n[a].lerp(n[b], t);
            n[dst] = v.try_normalize().unwrap_or(v);
        }
        if let Some(c) = &mut self.vertex_colors {
            c[dst] = c[a].lerp(c[b], t);
        }
    }

    /// Appends a face whose face attributes copy face `src` and whose corners are given.
    pub(crate) fn push_face(&mut self, src: Option<usize>, uvs: [Point2; 3], prop_index: [u32; 3]) {
        if let Some(u) = &mut self.corner_uvs {
            u.push(uvs);
        }
        if let Some(p) = &mut self.tri_properties {
            let v = match src {
                Some(s) => TriProperty {
                    group: p[s].group,
                    index: prop_index,
                },
                None => TriProperty::NONE,
            };
            p.push(v);
        }
    }

    /// Corner data of face `f`: (uvs, property indices).
    pub(crate) fn corners(&self, f: usize) -> ([Point2; 3], [u32; 3]) {
        let uv = self.corner_uvs.as_ref().map_or([Point2::ZERO; 3], |u| u[f]);
        let pi = self
            .tri_properties
            .as_ref()
            .map_or([INVALID; 3], |p| p[f].index);
        (uv, pi)
    }

    /// Overwrites the corner data of face `f`.
    pub(crate) fn set_corners(&mut self, f: usize, uvs: [Point2; 3], prop_index: [u32; 3]) {
        if let Some(u) = &mut self.corner_uvs {
            u[f] = uvs;
        }
        if let Some(p) = &mut self.tri_properties {
            p[f].index = prop_index;
        }
    }

    /// Keeps vertex `i` where `keep_v[i]` and face `f` where `keep_f[f]` (order preserved).
    pub(crate) fn retain(&mut self, keep_v: &[bool], keep_f: &[bool]) {
        fn filt<T: Copy>(v: &mut Option<Vec<T>>, keep: &[bool]) {
            if let Some(v) = v {
                let mut i = 0;
                v.retain(|_| {
                    i += 1;
                    keep[i - 1]
                });
            }
        }
        filt(&mut self.vertex_normals, keep_v);
        filt(&mut self.vertex_colors, keep_v);
        filt(&mut self.corner_uvs, keep_f);
        filt(&mut self.tri_properties, keep_f);
    }

    /// Appends `other`'s arrays. An attribute present on only one side is kept, and the other
    /// side is filled with defaults (zero normal, white, zero UV, [`TriProperty::NONE`]).
    pub(crate) fn append(
        &mut self,
        other: &MeshAttrs,
        (self_v, self_f): (usize, usize),
        (other_v, other_f): (usize, usize),
    ) {
        fn app<T: Copy>(
            a: &mut Option<Vec<T>>,
            b: &Option<Vec<T>>,
            a_len: usize,
            b_len: usize,
            def: T,
        ) {
            match (a.as_mut(), b) {
                (Some(a), Some(b)) => a.extend_from_slice(b),
                (Some(a), None) => a.resize(a_len + b_len, def),
                (None, Some(b)) => {
                    let mut v = vec![def; a_len];
                    v.extend_from_slice(b);
                    *a = Some(v);
                }
                (None, None) => {}
            }
        }
        app(
            &mut self.vertex_normals,
            &other.vertex_normals,
            self_v,
            other_v,
            Vec3::ZERO,
        );
        app(
            &mut self.vertex_colors,
            &other.vertex_colors,
            self_v,
            other_v,
            Rgba8::WHITE,
        );
        app(
            &mut self.corner_uvs,
            &other.corner_uvs,
            self_f,
            other_f,
            [Point2::ZERO; 3],
        );
        app(
            &mut self.tri_properties,
            &other.tri_properties,
            self_f,
            other_f,
            TriProperty::NONE,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rgba_lerp() {
        let a = Rgba8([0, 100, 255, 255]);
        let b = Rgba8([255, 0, 255, 0]);
        assert_eq!(a.lerp(b, 0.0), a);
        assert_eq!(a.lerp(b, 1.0), b);
        assert_eq!(a.lerp(b, 0.5), Rgba8([128, 50, 255, 128]));
    }

    #[test]
    fn validate_lengths() {
        let mut a = MeshAttrs {
            vertex_normals: Some(vec![Vec3::Z; 3]),
            ..Default::default()
        };
        assert!(a.validate(3, 1).is_ok());
        assert!(matches!(
            a.validate(4, 1),
            Err(MeshError::AttributeLength {
                name: "vertex_normals",
                got: 3,
                expected: 4
            })
        ));
        a.tri_properties = Some(vec![TriProperty::NONE; 2]);
        assert!(a.validate(3, 1).is_err());
        assert!(!a.is_empty());
        assert!(MeshAttrs::default().is_empty());
    }

    #[test]
    fn append_fills_defaults() {
        let mut a = MeshAttrs {
            vertex_colors: Some(vec![Rgba8([1, 2, 3, 4])]),
            ..Default::default()
        };
        let b = MeshAttrs {
            corner_uvs: Some(vec![[Point2::ONE; 3]]),
            ..Default::default()
        };
        a.append(&b, (1, 1), (2, 1));
        assert_eq!(
            a.vertex_colors,
            Some(vec![Rgba8([1, 2, 3, 4]), Rgba8::WHITE, Rgba8::WHITE])
        );
        assert_eq!(
            a.corner_uvs,
            Some(vec![[Point2::ZERO; 3], [Point2::ONE; 3]])
        );
        assert!(a.vertex_normals.is_none());
    }
}
