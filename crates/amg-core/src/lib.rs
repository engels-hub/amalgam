//! Foundation types: f64 math aliases, units, tolerances, ids, primitives and errors.
//!
//! Spec: S00. Lane: E.
//!
//! Everything is re-exported at the crate root, so callers write `amg_core::Point3`,
//! `amg_core::Tolerance`, `amg_core::intersect::ray_triangle` and so on.
//!
//! Geometry is f64, millimetres and radians. f32 appears only through [`gpu`].
//!
//! Physical quantities at API boundaries (parameters, dialogs, materials, FEA) go through
//! [`units`]: canonical values in the coherent mm–t–s–K–rad system, parsing and formatting.

#![warn(missing_docs)]

pub mod error;
pub mod geom;
pub mod id;
pub mod math;
pub mod tolerance;
pub mod units;

pub use error::{Cancelled, Diagnostic, Error, Result, Severity, report};
pub use geom::{Aabb2, Aabb3, Line2, Line3, Plane, Ray3, Segment2, Segment3, Triangle3, intersect};
pub use id::{DenseSlotMap, Key, KeyData, PersistentId, SecondaryMap, SlotMap, SparseSecondaryMap};
pub use math::{
    Dir3, Mat3, Mat4, Point2, Point3, Quat, Transform, Transform2, Vec2, Vec3, glam, gpu,
};
pub use tolerance::Tolerance;
pub use units::{
    Angle, Area, Density, Dimension, Energy, Force, FormatSpec, FormatStyle, Length, LengthUnit,
    Mass, ParseQuantityError, ParsedQuantity, Pressure, Quantity, Temperature, Time, Unit, Volume,
    format_quantity, parse_quantity,
};

#[doc(hidden)]
pub mod __private {
    pub use slotmap;
}
