//! Foundation types: f64 math aliases, units, tolerances, ids, primitives and errors.
//!
//! Spec: S00. Lane: E.
//!
//! Everything is re-exported at the crate root, so callers write `amg_core::Point3`,
//! `amg_core::Tolerance`, `amg_core::intersect::ray_triangle` and so on.
//!
//! Geometry is f64, millimetres and radians. f32 appears only through [`gpu`].
//!
//! The `units` module (`Dimension`, `Quantity`, typed quantities, parsing and formatting) is
//! work package FND-10 and is not part of this crate yet.

#![warn(missing_docs)]

pub mod error;
pub mod geom;
pub mod id;
pub mod math;
pub mod tolerance;
// `pub mod units;` lands with FND-10.

pub use error::{Cancelled, Diagnostic, Error, Result, Severity, report};
pub use geom::{Aabb2, Aabb3, Line2, Line3, Plane, Ray3, Segment2, Segment3, Triangle3, intersect};
pub use id::{DenseSlotMap, Key, KeyData, PersistentId, SecondaryMap, SlotMap, SparseSecondaryMap};
pub use math::{
    Dir3, Mat3, Mat4, Point2, Point3, Quat, Transform, Transform2, Vec2, Vec3, glam, gpu,
};
pub use tolerance::Tolerance;

#[doc(hidden)]
pub mod __private {
    pub use slotmap;
}
