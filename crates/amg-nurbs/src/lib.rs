//! Pure-Rust NURBS curves and surfaces: evaluation, knot ops, fitting, skinning, Gordon surfaces.
//!
//! Spec: S20. Lane: A. Design: `docs/notes/nurbs-library.md`.
//!
//! The crate follows *The NURBS Book* (Piegl & Tiller, 2nd ed., 1997; "P&T" below) on
//! `amg_core` glam f64 types. Curves and surfaces are always **clamped** and keep their
//! rational weights separately from the poles (a 1:1 copy to OCCT `Geom_BSpline*`).
//!
//! KH-07 provides the core: [`KnotVector`], [`basis`], [`NurbsCurve`], [`NurbsSurface`] +
//! [`Grid`] with validation, evaluation and derivatives (A2.1–A2.3, A3.2, A3.6, A4.1–A4.4),
//! reverse / reparametrize / transform / bbox, and the exact [`analytic`] conversions.
//! Knot operations (KH-12), fitting and closest point (KH-13), skinning (KH-14) and Gordon
//! surfaces (KH-15) build on it.
//!
//! Nothing panics on user-reachable input: constructors validate every invariant and return
//! [`NurbsError`]; evaluation clamps parameters into the domain.

#![warn(missing_docs)]
#![deny(unsafe_op_in_unsafe_fn)]

pub mod analytic;
pub mod basis;
mod ctrl_pt;
mod curve;
mod error;
mod grid;
mod knots;
mod surface;

pub use amg_core::{Aabb3, Dir3, Line3, Plane, Point2, Point3, Tolerance, Transform, Vec3};
pub use ctrl_pt::CtrlPt;
pub use curve::{Continuity, CurvePoint, NurbsCurve, NurbsCurve2, NurbsCurve3};
pub use error::NurbsError;
pub use grid::Grid;
pub use knots::KnotVector;
pub use surface::{NurbsSurface, NurbsSurface3, Side, SurfaceDerivs, SurfacePoint, UV};

/// Highest supported degree (= OCCT `Geom_BSplineCurve::MaxDegree()`).
pub const MAX_DEGREE: usize = 25;

/// Highest curve derivative order returned by [`NurbsCurve::derivs`].
pub const MAX_CURVE_DERIV: usize = 4;

/// Highest surface derivative order returned by [`NurbsSurface::derivs`].
pub const MAX_SURFACE_DERIV: usize = 2;
