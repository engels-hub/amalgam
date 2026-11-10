//! Geometry as plain data, so queries never expose OCCT (S02 §3).
//!
//! BREP-02 covers what queries return. BREP-03 adds the builder inputs (`Curve2d`,
//! `PlanarProfile`, `CurveSeg`, `FaceBounds`, `Tag`).

use amg_core::{Dir3, Plane, Point3};

/// A 3D curve, untrimmed. The edge's parameter range comes with it from `Shape::curve`.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Curve3 {
    /// Parameter = distance from `origin`.
    Line {
        /// Point at parameter 0.
        origin: Point3,
        /// Direction of increasing parameter.
        dir: Dir3,
    },
    /// Parameter = angle from `frame.x`, counter-clockwise about `frame.normal`.
    Circle {
        /// Centre and orientation.
        frame: Plane,
        /// Radius, mm.
        radius: f64,
    },
    /// Parameter = eccentric angle from `frame.x`, the major axis.
    Ellipse {
        /// Centre and orientation.
        frame: Plane,
        /// Major radius, mm.
        major_radius: f64,
        /// Minor radius, mm.
        minor_radius: f64,
    },
    /// B-spline (a Bezier curve is reported as its B-spline form).
    BSpline(BSplineCurveData),
    /// Offset, hyperbola, parabola or other; use `tessellate_edge` (BREP-08).
    Other,
}

/// A B-spline curve in OCCT's knots-and-multiplicities form.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct BSplineCurveData {
    /// Polynomial degree.
    pub degree: u8,
    /// Distinct knots, increasing.
    pub knots: Vec<f64>,
    /// Multiplicity of each knot.
    pub mults: Vec<u32>,
    /// Control points.
    pub poles: Vec<Point3>,
    /// One per pole; `None` when non-rational.
    pub weights: Option<Vec<f64>>,
    /// Whether the curve is periodic.
    pub periodic: bool,
}

/// An untrimmed surface. A face's trim comes from `Shape::face_uv_bounds` and its wires.
///
/// Frames are always right-handed (`normal = x × y`) and keep the surface's `x` and `y` axes, so
/// `u` keeps its meaning. A surface with a left-handed OCCT frame (only reflections produce one)
/// therefore reports `normal` opposite to OCCT's axis, and its `v` runs against `normal`.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Surface {
    /// `origin + u·x + v·y`.
    Plane(Plane),
    /// Axis along `frame.normal` through `frame.origin`.
    Cylinder {
        /// Axis and angular reference.
        frame: Plane,
        /// Radius, mm.
        radius: f64,
    },
    /// Axis along `frame.normal` through `frame.origin`.
    Cone {
        /// Axis and angular reference.
        frame: Plane,
        /// Radius at `frame.origin`, mm.
        ref_radius: f64,
        /// Semi-angle, rad; negative when the radius shrinks along the axis.
        half_angle: f64,
    },
    /// Centre at `frame.origin`.
    Sphere {
        /// Centre and parametrisation axes.
        frame: Plane,
        /// Radius, mm.
        radius: f64,
    },
    /// Axis along `frame.normal` through `frame.origin`.
    Torus {
        /// Axis and angular reference.
        frame: Plane,
        /// Major radius, mm.
        major: f64,
        /// Minor radius, mm.
        minor: f64,
    },
    /// B-spline (a Bezier surface is reported as its B-spline form).
    BSpline(BSplineSurfaceData),
    /// Revolution, extrusion, offset or other: read-only.
    Other {
        /// Always [`GeomClass::Other`] for now.
        class: GeomClass,
    },
}

impl Surface {
    /// The surface's class, as used in fingerprints and fixture expectations.
    pub fn class(&self) -> GeomClass {
        match self {
            Surface::Plane(_) => GeomClass::Plane,
            Surface::Cylinder { .. } => GeomClass::Cylinder,
            Surface::Cone { .. } => GeomClass::Cone,
            Surface::Sphere { .. } => GeomClass::Sphere,
            Surface::Torus { .. } => GeomClass::Torus,
            Surface::BSpline(_) => GeomClass::BSpline,
            Surface::Other { class } => *class,
        }
    }
}

/// A B-spline surface in OCCT's knots-and-multiplicities form.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct BSplineSurfaceData {
    /// Degree in `u`.
    pub u_degree: u8,
    /// Degree in `v`.
    pub v_degree: u8,
    /// Distinct `u` knots, increasing.
    pub u_knots: Vec<f64>,
    /// Multiplicity of each `u` knot.
    pub u_mults: Vec<u32>,
    /// Distinct `v` knots, increasing.
    pub v_knots: Vec<f64>,
    /// Multiplicity of each `v` knot.
    pub v_mults: Vec<u32>,
    /// Row-major with the `u` index outer: `nu × nv` entries.
    pub poles: Vec<Point3>,
    /// Pole rows along `u`.
    pub nu: u32,
    /// Pole columns along `v`.
    pub nv: u32,
    /// One per pole; `None` when non-rational.
    pub weights: Option<Vec<f64>>,
    /// Whether the surface is periodic in `u`.
    pub u_periodic: bool,
    /// Whether the surface is periodic in `v`.
    pub v_periodic: bool,
}

/// Mirrors `amg_doc::GeomClass` (S01) variant for variant.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum GeomClass {
    /// Planar face.
    Plane,
    /// Cylindrical face.
    Cylinder,
    /// Conical face.
    Cone,
    /// Spherical face.
    Sphere,
    /// Toroidal face.
    Torus,
    /// B-spline or Bezier face.
    BSpline,
    /// Any other face.
    Other,
    /// Straight edge.
    Line,
    /// Circular edge.
    Circle,
    /// Elliptical edge.
    Ellipse,
    /// Any other edge.
    Curve,
    /// Vertex.
    Point,
}
