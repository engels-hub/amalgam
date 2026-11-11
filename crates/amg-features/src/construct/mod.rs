//! Construction geometry (S05 §7, lane A): construction planes, axes and points as timeline
//! features.
//!
//! The module has two layers (S05 §7, note `docs/notes/inspection-geometry.md` §6):
//!
//! - [`math`]: pure f64 functions over [`RefGeom`] (world-space geometry of a resolved
//!   reference). They know nothing about documents and are tested on analytic inputs.
//! - [`resolve`]: [`RefInput`] → [`ResolvedRef`] inside a recompute. CON-01 resolves origin
//!   entities and other construction geometry; B-rep, sketch and mesh references join with
//!   CON-03.
//!
//! [`ConstructPlane`] (`construct.plane` v1, CON-01), [`ConstructAxis`] (`construct.axis` v1)
//! and [`ConstructPoint`] (`construct.point` v1, CON-02) are the features. Their compute goes
//! through [`ConstructInputs`], so tests can evaluate any [`PlaneDef`], [`AxisDef`] or
//! [`PointDef`] against pre-resolved geometry (cylinders, curves, ...) without a B-rep kernel.
//!
//! Call [`link`] from a binary or test so the registrations are linked in.

mod axis;
pub mod curves;
pub mod math;
mod plane;
mod point;
pub mod resolve;
#[cfg(test)]
mod tests;

use std::fmt;
use std::sync::Arc;

use amg_core::{Dir3, Line3, Plane, Point2, Point3, Segment3, Tolerance, Vec3};
use amg_doc::{Diagnostic, EntityRef, Expr, FeatureError, MatchQuality};
use serde::{Deserialize, Serialize};

pub use axis::{AxisDef, ConstructAxis};
pub use plane::{ConstructPlane, PlaneDef};
pub use point::{ConstructPoint, PointDef};

/// Forces this module's feature registrations to be linked (registry `link()` convention).
pub fn link() {}

/// An origin entity of the context component. These need no feature.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum OriginEntity {
    /// The origin point.
    Point,
    /// The X axis.
    XAxis,
    /// The Y axis.
    YAxis,
    /// The Z axis.
    ZAxis,
    /// The XY plane (normal +Z).
    XYPlane,
    /// The YZ plane (normal +X).
    YZPlane,
    /// The XZ plane (normal −Y, `Plane::XZ`).
    XZPlane,
}

/// One input of a construction feature.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum RefInput {
    /// An origin entity of the context component.
    Origin(OriginEntity),
    /// Any referenceable entity (B-rep, sketch, construction, mesh face group).
    Ref(EntityRef),
}

impl RefInput {
    /// The entity reference, if this is not an origin entity.
    pub fn entity(&self) -> Option<&EntityRef> {
        match self {
            RefInput::Origin(_) => None,
            RefInput::Ref(r) => Some(r),
        }
    }
}

/// A parametric curve in world space (B-rep edge or sketch curve; adapters in [`curves`]).
pub trait CurveEval: Send + Sync + fmt::Debug {
    /// Parameter range `(start, end)`, `start < end`.
    fn range(&self) -> (f64, f64);
    /// Point at `t`.
    fn eval(&self, t: f64) -> Point3;
    /// First derivative at `t`.
    fn d1(&self, t: f64) -> Vec3;
    /// Second derivative at `t`.
    fn d2(&self, t: f64) -> Vec3;
    /// Arc length of the whole range, mm.
    fn length(&self) -> f64;
    /// Parameter at arc length `s` from the start (`s ∈ [0, length]`).
    fn param_at_length(&self, s: f64) -> f64;
    /// Parameter of the point of the curve closest to `p`.
    fn closest_param(&self, p: Point3) -> f64;
}

/// A parametric surface in world space (B-rep face).
pub trait SurfaceEval: Send + Sync + fmt::Debug {
    /// Point at `uv`.
    fn eval(&self, uv: Point2) -> Point3;
    /// Outward normal (face orientation applied); None at singular points.
    fn normal(&self, uv: Point2) -> Option<Dir3>;
    /// Parameters of the point of the face closest to `p`; None if the query fails.
    fn closest_uv(&self, p: Point3) -> Option<Point2>;
}

/// World-space geometry of a resolved reference.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub enum RefGeom {
    /// A vertex, sketch point or construction point.
    Point(Point3),
    /// An infinite line (construction axis, origin axis).
    Line(Line3),
    /// A straight edge or sketch line, directed `a → b`.
    Segment(Segment3),
    /// A circle or arc. `arc` is `(start, end)` in radians, measured in the frame
    /// `Plane::from_normal(center, normal)` (counter-clockwise about `normal`); `None` is the
    /// full circle.
    Circle {
        /// Centre.
        center: Point3,
        /// Normal of the circle's plane.
        normal: Dir3,
        /// Radius, mm.
        radius: f64,
        /// Arc range, or `None` for the full circle.
        arc: Option<(f64, f64)>,
    },
    /// A plane (planar face, construction or origin plane).
    Plane(Plane),
    /// A cylindrical face.
    Cylinder {
        /// The axis.
        axis: Line3,
        /// Radius, mm.
        radius: f64,
    },
    /// A conical face opening along `+axis` from `apex`.
    Cone {
        /// Apex.
        apex: Point3,
        /// Axis direction (the cone opens this way).
        axis: Dir3,
        /// Half angle, rad, in (0, π/2).
        half_angle: f64,
    },
    /// A spherical face.
    Sphere {
        /// Centre.
        center: Point3,
        /// Radius, mm.
        radius: f64,
    },
    /// A toroidal face.
    Torus {
        /// Axis; `axis.point` is the centre.
        axis: Line3,
        /// Major radius, mm.
        major: f64,
        /// Minor radius, mm.
        minor: f64,
    },
    /// A free-form curve.
    Curve(Arc<dyn CurveEval>),
    /// A free-form surface.
    Surface(Arc<dyn SurfaceEval>),
}

impl RefGeom {
    /// Short name of the geometry kind, for [`ConstructError::WrongInput`].
    pub fn kind_name(&self) -> &'static str {
        match self {
            RefGeom::Point(_) => "point",
            RefGeom::Line(_) => "line",
            RefGeom::Segment(_) => "straight edge",
            RefGeom::Circle { .. } => "circle",
            RefGeom::Plane(_) => "plane",
            RefGeom::Cylinder { .. } => "cylinder",
            RefGeom::Cone { .. } => "cone",
            RefGeom::Sphere { .. } => "sphere",
            RefGeom::Torus { .. } => "torus",
            RefGeom::Curve(_) => "curve",
            RefGeom::Surface(_) => "surface",
        }
    }
}

/// A resolved reference: geometry plus context.
#[derive(Clone, Debug)]
pub struct ResolvedRef {
    /// World-space geometry.
    pub geom: RefGeom,
    /// The sketch plane for sketch entities, or the first adjacent planar face for B-rep edges.
    pub context_plane: Option<Plane>,
    /// How the reference matched (DOC-06); `None` for references that need no naming
    /// (origin entities, construction geometry).
    pub quality: Option<MatchQuality>,
}

impl ResolvedRef {
    /// Geometry without context or naming quality.
    pub fn new(geom: RefGeom) -> ResolvedRef {
        ResolvedRef {
            geom,
            context_plane: None,
            quality: None,
        }
    }
}

/// Why a construction failed (note §6).
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum ConstructError {
    /// An input has the wrong kind of geometry.
    #[error("expected {expected}, got {got}")]
    WrongInput {
        /// What the construction needs.
        expected: &'static str,
        /// What it got ([`RefGeom::kind_name`]).
        got: &'static str,
    },
    /// The inputs are parallel.
    #[error("inputs are parallel")]
    Parallel,
    /// The inputs coincide.
    #[error("inputs coincide")]
    Coincident,
    /// The points are collinear.
    #[error("points are collinear")]
    Collinear,
    /// Two lines are not coplanar.
    #[error("lines are not coplanar (gap {distance} mm)")]
    Skew {
        /// Shortest distance between the lines, mm.
        distance: f64,
    },
    /// A point is not on the face.
    #[error("point is {distance} mm off the face")]
    NotOnFace {
        /// Distance from the face, mm (infinite if the closest-point query failed).
        distance: f64,
    },
    /// An edge that must be straight is curved.
    #[error("edge is not straight")]
    NotStraight,
    /// The inputs do not intersect.
    #[error("no intersection")]
    NoIntersection,
    /// A path position is outside the path.
    #[error("position is outside the path")]
    OutOfRange,
    /// The surface type is not supported by this construction.
    #[error("unsupported surface for this construction")]
    Unsupported,
    /// A degenerate input (zero radius, singular point, ...).
    #[error("degenerate input: {0}")]
    Degenerate(&'static str),
}

impl From<ConstructError> for FeatureError {
    fn from(e: ConstructError) -> Self {
        FeatureError::InvalidValue {
            field: "inputs",
            msg: e.to_string(),
        }
    }
}

/// A position along a path.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum PathPosition {
    /// Fraction of the arc length, 0..1 (dimensionless).
    Fraction(Expr),
    /// Arc length from the start, mm.
    Distance(Expr),
}

/// Everything a construction feature reads while computing: resolved references, expression
/// values and the model tolerance.
///
/// The recompute implementation wraps [`amg_doc::RecomputeCtx`] and [`resolve::resolve_input`];
/// tests supply pre-resolved [`RefGeom`]s.
pub trait ConstructInputs {
    /// Resolves one input.
    fn resolve(&self, r: &RefInput) -> Result<ResolvedRef, FeatureError>;
    /// A length expression in mm.
    fn length(&self, e: &Expr) -> Result<f64, FeatureError>;
    /// An angle expression in radians.
    fn angle(&self, e: &Expr) -> Result<f64, FeatureError>;
    /// A dimensionless expression.
    fn scalar(&self, e: &Expr) -> Result<f64, FeatureError>;
    /// Model tolerance.
    fn tolerance(&self) -> Tolerance;
}

/// [`ConstructInputs`] over a recompute context.
#[derive(Debug)]
pub struct CtxInputs<'c, 'a> {
    /// The context.
    pub ctx: &'c amg_doc::RecomputeCtx<'a>,
}

impl ConstructInputs for CtxInputs<'_, '_> {
    fn resolve(&self, r: &RefInput) -> Result<ResolvedRef, FeatureError> {
        resolve::resolve_input(self.ctx, r)
    }
    fn length(&self, e: &Expr) -> Result<f64, FeatureError> {
        self.ctx.eval_length(e)
    }
    fn angle(&self, e: &Expr) -> Result<f64, FeatureError> {
        self.ctx.eval_angle(e)
    }
    fn scalar(&self, e: &Expr) -> Result<f64, FeatureError> {
        self.ctx.eval_scalar(e)
    }
    fn tolerance(&self) -> Tolerance {
        *self.ctx.tolerance()
    }
}

/// Resolves `r`, recording a health warning if it matched by fingerprint only (S05 §7).
fn resolve_tracked(
    inputs: &dyn ConstructInputs,
    r: &RefInput,
    warnings: &mut Vec<Diagnostic>,
) -> Result<ResolvedRef, FeatureError> {
    let resolved = inputs.resolve(r)?;
    if resolved.quality == Some(MatchQuality::FingerprintOnly) {
        warnings.push(Diagnostic {
            message: "a reference matched by geometry only; check the result".to_owned(),
            cause: None,
            entities: r.entity().cloned().into_iter().collect(),
        });
    }
    Ok(resolved)
}

/// The dependency a reference input adds to `Feature::inputs`.
fn dependency(r: &RefInput) -> Option<amg_doc::Dependency> {
    match r {
        RefInput::Origin(_) => None,
        RefInput::Ref(EntityRef::Construction(c)) => Some(amg_doc::Dependency::Construction(*c)),
        RefInput::Ref(other) => Some(amg_doc::Dependency::Entity(other.clone())),
    }
}
