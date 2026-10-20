//! f64 math aliases over `glam`, the GPU conversion boundary and the unit direction [`Dir3`].
//!
//! The aliases are aliases, not newtypes, so glam's operators work unchanged. Points and
//! vectors share `DVec3`; the name says which one is meant.

use crate::error::{Error, Result};
use crate::tolerance::Tolerance;

/// Escape hatch to the underlying math crate; prefer the aliases below.
pub use glam;

/// Position in model space, mm.
pub type Point3 = glam::DVec3;
/// Displacement or direction in model space, mm (not necessarily unit length).
pub type Vec3 = glam::DVec3;
/// Position in a 2D parameter or sketch space, mm.
pub type Point2 = glam::DVec2;
/// Displacement in a 2D parameter or sketch space, mm.
pub type Vec2 = glam::DVec2;
/// 3x3 matrix (column-major).
pub type Mat3 = glam::DMat3;
/// 4x4 matrix (column-major).
pub type Mat4 = glam::DMat4;
/// Rotation quaternion.
pub type Quat = glam::DQuat;
/// Rigid or affine 3D transform, mm.
pub type Transform = glam::DAffine3;
/// Rigid or affine 2D transform, mm.
pub type Transform2 = glam::DAffine2;

/// Lossy f64 → f32 conversions: the only sanctioned path from model data to GPU data.
pub mod gpu {
    use super::{Point3, Transform, Vec3};

    /// Converts a vector or point to f32 (lossy; rounds each component to nearest).
    pub fn to_f32(v: Vec3) -> glam::Vec3 {
        v.as_vec3()
    }

    /// Replaces the contents of `dst` with `src` converted to f32 triples.
    ///
    /// `dst` is cleared first, so a buffer can be reused across frames without reallocating.
    pub fn to_f32_slice(src: &[Point3], dst: &mut Vec<[f32; 3]>) {
        dst.clear();
        dst.reserve(src.len());
        dst.extend(src.iter().map(|p| p.as_vec3().to_array()));
    }

    /// Converts an affine transform to an f32 4x4 matrix (lossy).
    pub fn transform_to_f32(t: &Transform) -> glam::Mat4 {
        glam::DMat4::from(*t).as_mat4()
    }
}

/// Bound on `| |v| - 1 |` within which a vector is accepted as already normalised and kept
/// bit-for-bit (so serialised directions round-trip exactly).
const UNIT_EPS: f64 = 4.0 * f64::EPSILON;

/// Unit-length direction; the invariant is checked once at construction.
///
/// Serialises as a plain `Vec3`; deserialising goes through [`TryFrom<Vec3>`].
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(try_from = "Vec3", into = "Vec3")]
pub struct Dir3(Vec3);

impl Dir3 {
    /// +X.
    pub const X: Dir3 = Dir3(Vec3::X);
    /// +Y.
    pub const Y: Dir3 = Dir3(Vec3::Y);
    /// +Z.
    pub const Z: Dir3 = Dir3(Vec3::Z);
    /// −X.
    pub const NEG_X: Dir3 = Dir3(Vec3::NEG_X);
    /// −Y.
    pub const NEG_Y: Dir3 = Dir3(Vec3::NEG_Y);
    /// −Z.
    pub const NEG_Z: Dir3 = Dir3(Vec3::NEG_Z);

    /// Normalises `v`. Returns `None` if `|v| <= tol.linear` (cannot normalise) or `v` is not
    /// finite.
    pub fn new(v: Vec3, tol: &Tolerance) -> Option<Dir3> {
        let len = v.length();
        if !len.is_finite() || len <= tol.linear {
            return None;
        }
        if (len - 1.0).abs() <= UNIT_EPS {
            Some(Dir3(v))
        } else {
            Some(Dir3(v / len))
        }
    }

    /// Wraps `v` without normalising. `v` must already be unit length (debug-asserted).
    pub fn new_unchecked(v: Vec3) -> Dir3 {
        debug_assert!(
            (v.length() - 1.0).abs() <= 1e-9,
            "Dir3::new_unchecked: |v| = {} is not 1",
            v.length()
        );
        Dir3(v)
    }

    /// The unit vector.
    pub fn get(self) -> Vec3 {
        self.0
    }

    /// A unit direction orthogonal to `self`, chosen deterministically (same input → same
    /// output).
    pub fn any_orthogonal(self) -> Dir3 {
        Dir3(self.0.any_orthonormal_vector())
    }
}

impl std::ops::Neg for Dir3 {
    type Output = Dir3;
    fn neg(self) -> Dir3 {
        Dir3(-self.0)
    }
}

impl TryFrom<Vec3> for Dir3 {
    type Error = Error;
    /// Normalises with [`Tolerance::DEFAULT`]; `Err(Degenerate)` for zero or non-finite input.
    fn try_from(v: Vec3) -> Result<Dir3> {
        Dir3::new(v, &Tolerance::DEFAULT).ok_or(Error::Degenerate {
            what: "direction vector has zero length",
        })
    }
}

impl From<Dir3> for Vec3 {
    fn from(d: Dir3) -> Vec3 {
        d.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn bits(v: Vec3) -> [u64; 3] {
        v.to_array().map(f64::to_bits)
    }

    #[test]
    fn to_f32_converts_components() {
        let v = gpu::to_f32(Vec3::new(1.5, -2.0, 1e-3));
        assert_eq!(v, glam::Vec3::new(1.5, -2.0, 1e-3_f32));
    }

    #[test]
    fn to_f32_slice_replaces_contents() {
        let mut dst = vec![[9.0_f32; 3]; 5];
        gpu::to_f32_slice(&[Point3::new(1.0, 2.0, 3.0), Point3::ZERO], &mut dst);
        assert_eq!(dst, vec![[1.0, 2.0, 3.0], [0.0, 0.0, 0.0]]);
    }

    #[test]
    fn transform_to_f32_matches_f64() {
        let t = Transform::from_scale_rotation_translation(
            Vec3::splat(2.0),
            Quat::from_rotation_z(0.3),
            Vec3::new(1.0, 2.0, 3.0),
        );
        let m = gpu::transform_to_f32(&t);
        let p = Point3::new(0.5, -1.0, 2.0);
        let a = t.transform_point3(p).as_vec3();
        let b = m.transform_point3(p.as_vec3());
        assert!((a - b).length() < 1e-5);
    }

    #[test]
    fn dir3_new_normalises_and_rejects_zero() {
        let tol = Tolerance::DEFAULT;
        let d = Dir3::new(Vec3::new(3.0, 0.0, 4.0), &tol).unwrap();
        assert_relative_eq!(d.get(), Vec3::new(0.6, 0.0, 0.8), epsilon = 1e-15);
        assert!(Dir3::new(Vec3::splat(1e-7), &tol).is_none());
        assert!(Dir3::new(Vec3::new(f64::NAN, 0.0, 0.0), &tol).is_none());
        assert!(Dir3::new(Vec3::new(f64::INFINITY, 0.0, 0.0), &tol).is_none());
    }

    #[test]
    fn dir3_new_keeps_unit_vectors_bit_exact() {
        let v = Vec3::new(0.6, 0.0, 0.8);
        let d = Dir3::new(v, &Tolerance::DEFAULT).unwrap();
        assert_eq!(bits(d.get()), bits(v));
    }

    #[test]
    fn dir3_new_unchecked_and_get() {
        assert_eq!(Dir3::new_unchecked(Vec3::Y).get(), Vec3::Y);
        assert_eq!(Dir3::Z.get(), Vec3::Z);
        assert_eq!(Dir3::NEG_X.get(), -Vec3::X);
    }

    #[test]
    #[should_panic(expected = "not 1")]
    #[cfg(debug_assertions)]
    fn dir3_new_unchecked_asserts_in_debug() {
        let _ = Dir3::new_unchecked(Vec3::new(2.0, 0.0, 0.0));
    }

    #[test]
    fn dir3_any_orthogonal_is_unit_orthogonal_and_deterministic() {
        for v in [
            Vec3::X,
            Vec3::Y,
            Vec3::Z,
            Vec3::NEG_Z,
            Vec3::new(1.0, 2.0, 3.0),
        ] {
            let d = Dir3::new(v, &Tolerance::DEFAULT).unwrap();
            let o = d.any_orthogonal();
            assert!(o.get().dot(d.get()).abs() < 1e-15);
            assert!((o.get().length() - 1.0).abs() < 1e-15);
            assert_eq!(o, d.any_orthogonal());
        }
    }

    #[test]
    fn dir3_neg() {
        assert_eq!(-Dir3::X, Dir3::NEG_X);
        assert_eq!(-Dir3::NEG_Y, Dir3::Y);
    }

    #[test]
    fn dir3_try_from_and_into() {
        let d = Dir3::try_from(Vec3::new(0.0, 5.0, 0.0)).unwrap();
        assert_eq!(d, Dir3::Y);
        assert!(matches!(
            Dir3::try_from(Vec3::ZERO),
            Err(Error::Degenerate { .. })
        ));
        let v: Vec3 = Dir3::Z.into();
        assert_eq!(v, Vec3::Z);
    }

    #[test]
    fn dir3_serde_round_trip_and_validation() {
        let d = Dir3::new(Vec3::new(1.0, 2.0, 3.0), &Tolerance::DEFAULT).unwrap();
        let s = serde_json::to_string(&d).unwrap();
        let back: Dir3 = serde_json::from_str(&s).unwrap();
        assert_eq!(bits(back.get()), bits(d.get()));
        // Non-unit input is normalised; zero is rejected.
        let n: Dir3 = serde_json::from_str("[0.0, 0.0, 2.0]").unwrap();
        assert_eq!(n, Dir3::Z);
        assert!(serde_json::from_str::<Dir3>("[0.0, 0.0, 0.0]").is_err());
    }
}
