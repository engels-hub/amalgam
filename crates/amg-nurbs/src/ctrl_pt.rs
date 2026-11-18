//! [`CtrlPt`]: the control-point space (2D or 3D).

use amg_core::{Point2, Point3};

mod sealed {
    pub trait Sealed {}
    impl Sealed for amg_core::Point2 {}
    impl Sealed for amg_core::Point3 {}
}

/// Control-point space of a curve or surface. Sealed: implemented for [`Point2`] and
/// [`Point3`] only.
pub trait CtrlPt:
    Copy
    + std::fmt::Debug
    + PartialEq
    + Send
    + Sync
    + 'static
    + std::ops::Add<Output = Self>
    + std::ops::Sub<Output = Self>
    + std::ops::Mul<f64, Output = Self>
    + serde::Serialize
    + serde::de::DeserializeOwned
    + sealed::Sealed
{
    /// Number of coordinates.
    const DIM: usize;
    /// The origin / zero vector.
    fn zero() -> Self;
    /// Dot product.
    fn dot(self, o: Self) -> f64;
    /// Euclidean length.
    fn length(self) -> f64;
    /// True if every coordinate is finite.
    #[doc(hidden)]
    fn is_finite_pt(self) -> bool;
    /// Component-wise maximum of absolute values (for scale estimates).
    #[doc(hidden)]
    fn max_abs(self) -> f64;
}

impl CtrlPt for Point2 {
    const DIM: usize = 2;
    #[inline]
    fn zero() -> Self {
        Point2::ZERO
    }
    #[inline]
    fn dot(self, o: Self) -> f64 {
        Point2::dot(self, o)
    }
    #[inline]
    fn length(self) -> f64 {
        Point2::length(self)
    }
    #[inline]
    fn is_finite_pt(self) -> bool {
        self.is_finite()
    }
    #[inline]
    fn max_abs(self) -> f64 {
        self.abs().max_element()
    }
}

impl CtrlPt for Point3 {
    const DIM: usize = 3;
    #[inline]
    fn zero() -> Self {
        Point3::ZERO
    }
    #[inline]
    fn dot(self, o: Self) -> f64 {
        Point3::dot(self, o)
    }
    #[inline]
    fn length(self) -> f64 {
        Point3::length(self)
    }
    #[inline]
    fn is_finite_pt(self) -> bool {
        self.is_finite()
    }
    #[inline]
    fn max_abs(self) -> f64 {
        self.abs().max_element()
    }
}
