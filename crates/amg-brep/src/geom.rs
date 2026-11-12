//! Geometry as plain data, so queries never expose OCCT (S02 §3).
//!
//! BREP-02 covers what queries return. BREP-03 adds the builder inputs (`Curve2d`,
//! `PlanarProfile`, `CurveSeg`, `FaceBounds`, `Tag`).

use amg_core::{Dir3, Plane, Point2, Point3, Vec2};

/// Caller-chosen tag carried from builder inputs to the edges they create (naming hook, S02 §6).
/// S04 puts the sketch entity key here.
#[derive(
    Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize,
)]
pub struct Tag(pub u64);

/// A 2D profile curve in a plane's local `(u, v)`. Mirrors `amg_sketch::geom::Curve2` variant for
/// variant and field for field (S03), so S04 converts with a total `match`.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Curve2d {
    /// Segment from `a` to `b`.
    Line {
        /// Start point.
        a: Point2,
        /// End point.
        b: Point2,
    },
    /// Full circle, counter-clockwise from `+u`. A circle has no stored direction, so in a
    /// profile it must be a loop by itself, and it runs the way its loop must run (outer CCW,
    /// hole CW). An oriented full circle is an `Arc` with `sweep = ±2π` (what S03 produces).
    Circle {
        /// Centre.
        center: Point2,
        /// Radius, mm.
        radius: f64,
    },
    /// Circular arc from `start_angle` by `sweep` (`> 0` CCW, `|sweep| ∈ (0, 2π]`).
    Arc {
        /// Centre.
        center: Point2,
        /// Radius, mm.
        radius: f64,
        /// Angle of the start point from `+u`, rad.
        start_angle: f64,
        /// Signed sweep, rad.
        sweep: f64,
    },
    /// Elliptical arc `P(t) = center + major·cos t + perp(major)·(minor_radius/|major|)·sin t`
    /// from `t0` to `t1` (`t1 < t0` runs clockwise, `|t1 − t0| = 2π` is a full ellipse).
    /// `minor_radius` may exceed `|major|` (S03 convention).
    Ellipse {
        /// Centre.
        center: Point2,
        /// Semi-major vector (centre to the end of the named axis), mm.
        major: Vec2,
        /// The other semi-axis, mm.
        minor_radius: f64,
        /// Eccentric angle at the start, rad.
        t0: f64,
        /// Eccentric angle at the end, rad.
        t1: f64,
    },
    /// B-spline or NURBS over its whole knot range, start to end.
    BSpline(BSpline2dData),
}

/// A 2D B-spline in full-knot-vector form (S03 `BSpline2`).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct BSpline2dData {
    /// Degree, `1..`.
    pub degree: u8,
    /// Full knot vector, non-decreasing, `poles.len() + degree + 1` entries.
    pub knots: Vec<f64>,
    /// Control points.
    pub poles: Vec<Point2>,
    /// One positive weight per pole; `None` when non-rational.
    pub weights: Option<Vec<f64>>,
}

/// One edge of a profile loop, oriented along the loop.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ProfileEdge2 {
    /// The curve, run from its start to its end.
    pub curve: Curve2d,
    /// Copied to the created edge ([`crate::FaceBuild::edge_tags`]).
    pub tag: Tag,
}

/// A closed loop: each edge ends where the next starts (within `Tolerance::linear`), the last
/// ends where the first starts.
#[derive(Clone, Debug, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct Loop2 {
    /// Edges in loop order.
    pub edges: Vec<ProfileEdge2>,
}

/// One planar region: outer loop CCW, holes CW (S03 convention), in `plane`'s local `(u, v)`.
/// Holes must lie inside the outer loop.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PlanarProfile {
    /// The sketch plane; the face normal is `plane.normal`.
    pub plane: Plane,
    /// Outer boundary, counter-clockwise about `plane.normal`.
    pub outer: Loop2,
    /// Holes, clockwise.
    pub holes: Vec<Loop2>,
}

/// A trimmed 3D curve: the input of wires (sweep paths, rails) and face loops. The edge runs
/// from parameter `t0` to `t1`; `t1 < t0` runs the curve backwards (a reversed edge).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct CurveSeg {
    /// The carrier curve (`Curve3::Other` is rejected).
    pub curve: Curve3,
    /// Start parameter.
    pub t0: f64,
    /// End parameter.
    pub t1: f64,
    /// Copied to the created edge.
    pub tag: Tag,
}

/// How [`crate::BrepKernel::make_face`] bounds its surface.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum FaceBounds {
    /// The full parameter domain. Infinite surfaces (plane, cylinder, cone) are rejected.
    Natural,
    /// A rectangle of the parameter domain.
    UvRect {
        /// Lower `u`.
        u0: f64,
        /// Upper `u`.
        u1: f64,
        /// Lower `v`.
        v0: f64,
        /// Upper `v`.
        v1: f64,
    },
    /// Closed loops of 3D curves on (within `tol` of) the surface; the first is the outer loop.
    /// Pcurves come from projection. Orientation is fixed up, so loops may run either way.
    Loops(Vec<Vec<CurveSeg>>),
    /// Closed polylines near the surface; the first is the outer loop. Each is projected to
    /// `(u, v)`, split at corners and approximated by B-spline pcurves within `tol`.
    /// Orientation is fixed up, and a repeated closing point is ignored.
    Polylines(Vec<Vec<Point3>>),
}

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

impl Curve3 {
    /// The point at parameter `t`, for sampling extracted curves (BREP-17). `None` for `Other`
    /// and for periodic B-splines (extraction never returns either).
    pub fn point_at(&self, t: f64) -> Option<Point3> {
        crate::builders::eval::point3(self, t)
    }
}

impl CurveSeg {
    /// The start point (at `t0`); `None` as for [`Curve3::point_at`].
    pub fn start(&self) -> Option<Point3> {
        self.curve.point_at(self.t0)
    }

    /// The end point (at `t1`); `None` as for [`Curve3::point_at`].
    pub fn end(&self) -> Option<Point3> {
        self.curve.point_at(self.t1)
    }
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
