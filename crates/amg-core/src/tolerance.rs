//! Explicit geometric tolerances. There is no global tolerance: a [`Tolerance`] travels in
//! the doc/recompute context (S01) and is passed to every comparison that needs one.

use crate::math::{Dir3, Point3};

/// Linear and angular tolerance for geometric comparisons.
///
/// The defaults (1e-6 mm, 1e-9 rad) are deliberately coarser than OCCT's
/// `Precision::Confusion()` (1e-7) and `Precision::Angular()` (1e-12), so values that come back
/// from OCCT compare equal.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Tolerance {
    /// Distance below which two points coincide, mm. Default 1e-6.
    pub linear: f64,
    /// Angle below which two directions are parallel, rad. Default 1e-9.
    pub angular: f64,
}

impl Tolerance {
    /// Model tolerance: linear 1e-6 mm, angular 1e-9 rad.
    pub const DEFAULT: Tolerance = Tolerance {
        linear: 1e-6,
        angular: 1e-9,
    };

    /// Mesh tolerance for vertex welding: linear 1e-5 mm, angular 1e-9 rad.
    pub const fn mesh() -> Tolerance {
        Tolerance {
            linear: 1e-5,
            angular: 1e-9,
        }
    }

    /// `|a - b| <= linear` (lengths in mm).
    pub fn eq_len(&self, a: f64, b: f64) -> bool {
        (a - b).abs() <= self.linear
    }

    /// `|a| <= linear` (length in mm).
    pub fn is_zero_len(&self, a: f64) -> bool {
        a.abs() <= self.linear
    }

    /// The points are at most `linear` apart.
    pub fn eq_point(&self, a: Point3, b: Point3) -> bool {
        a.distance_squared(b) <= self.linear * self.linear
    }

    /// `|a - b| <= angular` (angles in rad; no wrapping at 2π).
    pub fn eq_angle(&self, a: f64, b: f64) -> bool {
        (a - b).abs() <= self.angular
    }

    /// The directions are parallel, pointing the same or opposite way (`sin θ <= angular`).
    pub fn is_parallel(&self, a: Dir3, b: Dir3) -> bool {
        a.get().cross(b.get()).length() <= self.angular
    }

    /// The directions are parallel and point the same way.
    pub fn is_codirectional(&self, a: Dir3, b: Dir3) -> bool {
        self.is_parallel(a, b) && a.get().dot(b.get()) > 0.0
    }

    /// The directions are perpendicular (`|cos θ| <= angular`).
    pub fn is_perpendicular(&self, a: Dir3, b: Dir3) -> bool {
        a.get().dot(b.get()).abs() <= self.angular
    }
}

impl Default for Tolerance {
    fn default() -> Self {
        Tolerance::DEFAULT
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Vec3;

    const T: Tolerance = Tolerance::DEFAULT;

    fn dir(x: f64, y: f64, z: f64) -> Dir3 {
        Dir3::new(Vec3::new(x, y, z), &T).unwrap()
    }

    #[test]
    fn defaults() {
        assert_eq!(Tolerance::default(), T);
        assert!((T.linear - 1e-6).abs() < f64::EPSILON);
        let m = Tolerance::mesh();
        assert!((m.linear - 1e-5).abs() < f64::EPSILON);
        assert!((m.angular - 1e-9).abs() < f64::EPSILON);
    }

    #[test]
    fn eq_len() {
        assert!(T.eq_len(1.0, 1.0 + 5e-7));
        assert!(!T.eq_len(1.0, 1.0 + 2e-6));
    }

    #[test]
    fn is_zero_len() {
        assert!(T.is_zero_len(-9e-7));
        assert!(!T.is_zero_len(1.1e-6));
    }

    #[test]
    fn eq_point() {
        let a = Point3::new(1.0, 2.0, 3.0);
        assert!(T.eq_point(a, a + Vec3::new(5e-7, 5e-7, 0.0)));
        assert!(!T.eq_point(a, a + Vec3::new(1e-6, 1e-6, 0.0)));
    }

    #[test]
    fn eq_angle() {
        assert!(T.eq_angle(0.5, 0.5 + 5e-10));
        assert!(!T.eq_angle(0.5, 0.5 + 2e-9));
    }

    #[test]
    fn is_parallel() {
        assert!(T.is_parallel(Dir3::X, Dir3::X));
        assert!(T.is_parallel(Dir3::X, Dir3::NEG_X));
        assert!(T.is_parallel(Dir3::X, dir(1.0, 5e-10, 0.0)));
        assert!(!T.is_parallel(Dir3::X, dir(1.0, 1e-8, 0.0)));
    }

    #[test]
    fn is_codirectional() {
        assert!(T.is_codirectional(Dir3::Z, dir(0.0, 1e-10, 1.0)));
        assert!(!T.is_codirectional(Dir3::Z, Dir3::NEG_Z));
        assert!(!T.is_codirectional(Dir3::Z, Dir3::X));
    }

    #[test]
    fn is_perpendicular() {
        assert!(T.is_perpendicular(Dir3::X, Dir3::Y));
        assert!(T.is_perpendicular(Dir3::X, dir(5e-10, 1.0, 0.0)));
        assert!(!T.is_perpendicular(Dir3::X, dir(1e-8, 1.0, 0.0)));
        assert!(!T.is_perpendicular(Dir3::X, Dir3::X));
    }

    #[test]
    fn serde_round_trip() {
        let s = serde_json::to_string(&Tolerance::mesh()).unwrap();
        let back: Tolerance = serde_json::from_str(&s).unwrap();
        assert_eq!(back, Tolerance::mesh());
    }
}
