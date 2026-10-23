//! Sketch data model: parameter vector, entities, constraints and dimensions.
//!
//! All solver unknowns are `f64` entries of one [`ParamStore`]. Entities own parameters
//! (a point owns `x, y`, a circle owns `r`) or reference other entities (a line references
//! its two end points). Constraints reference entities and expand to residual equations in
//! the solver (SK-02).

use amg_core::Point2;
use serde::{Deserialize, Serialize};
use smallvec::SmallVec;

use crate::ids::{EntityId, ExternalLinkId, ParamId};

mod sketch;
mod validate;

pub use sketch::{RemoveReport, Sketch};

// ---- Parameters -------------------------------------------------------------------------

/// Per-parameter flags.
#[derive(Copy, Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ParamFlags {
    /// The solver never changes this parameter (projected geometry).
    pub locked: bool,
}

/// The sketch's parameter vector. A [`ParamId`] indexes it directly.
///
/// Slots are allocated by [`Sketch`] and never move; removed entities leave their slots
/// behind until [`Sketch::compact`].
///
/// Serialising fails if any value is not finite (for example after a diverged solve), so a
/// saved sketch always loads.
#[derive(Clone, Debug, Default, Deserialize)]
pub struct ParamStore {
    values: Vec<f64>,
    flags: Vec<ParamFlags>,
}

impl ParamStore {
    /// Value of `p`.
    ///
    /// # Panics
    /// If `p` is not in the store (`!self.contains(p)`).
    pub fn get(&self, p: ParamId) -> f64 {
        self.values[p.index()]
    }

    /// Sets the value of `p`.
    ///
    /// # Panics
    /// If `p` is not in the store.
    pub fn set(&mut self, p: ParamId, v: f64) {
        self.values[p.index()] = v;
    }

    /// Flags of `p`.
    ///
    /// # Panics
    /// If `p` is not in the store.
    pub fn flags(&self, p: ParamId) -> ParamFlags {
        self.flags[p.index()]
    }

    /// Sets the flags of `p`.
    ///
    /// # Panics
    /// If `p` is not in the store.
    pub fn set_flags(&mut self, p: ParamId, flags: ParamFlags) {
        self.flags[p.index()] = flags;
    }

    /// True if `p` indexes a slot of this store.
    pub fn contains(&self, p: ParamId) -> bool {
        p.index() < self.values.len()
    }

    /// All values, indexed by `ParamId::index`.
    pub fn as_slice(&self) -> &[f64] {
        &self.values
    }

    /// Number of slots (including slots of removed entities until `compact`).
    pub fn len(&self) -> usize {
        self.values.len()
    }

    /// True if there are no slots.
    pub fn is_empty(&self) -> bool {
        self.values.is_empty()
    }

    fn push(&mut self, value: f64, flags: ParamFlags) -> ParamId {
        let id = ParamId(u32::try_from(self.values.len()).unwrap_or(u32::MAX));
        self.values.push(value);
        self.flags.push(flags);
        id
    }
}

impl Serialize for ParamStore {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        #[derive(Serialize)]
        struct Repr<'a> {
            values: &'a [f64],
            flags: &'a [ParamFlags],
        }
        if let Some((i, v)) = self.values.iter().enumerate().find(|(_, v)| !v.is_finite()) {
            return Err(serde::ser::Error::custom(format_args!(
                "sketch parameter {i} is not finite ({v})"
            )));
        }
        Repr {
            values: &self.values,
            flags: &self.flags,
        }
        .serialize(s)
    }
}

// ---- Entities ---------------------------------------------------------------------------

/// Where an entity comes from.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum EntityOrigin {
    /// Drawn by the user.
    User,
    /// Projected from 3D geometry; its own params are locked.
    Projected(ExternalLinkId),
    /// Imported (DXF/SVG).
    Imported,
}

/// Geometry of an entity. Splines and conics (SK-08) are added by their work package.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum EntityKind {
    /// Point with its own `x, y` params (2 DOF).
    Point {
        /// x coordinate, mm.
        x: ParamId,
        /// y coordinate, mm.
        y: ParamId,
    },
    /// Line segment between two point entities (4 DOF).
    Line {
        /// Start point entity.
        start: EntityId,
        /// End point entity.
        end: EntityId,
    },
    /// Full circle (3 DOF).
    Circle {
        /// Centre point entity.
        center: EntityId,
        /// Radius param, mm.
        radius: ParamId,
    },
    /// Arc, CCW from `start` to `end`; `|start − center| = |end − center|` is an internal
    /// equation (5 DOF).
    Arc {
        /// Centre point entity.
        center: EntityId,
        /// Start point entity.
        start: EntityId,
        /// End point entity.
        end: EntityId,
    },
    /// Ellipse (5 DOF): centre, the end point of the semi-major axis, and the minor radius.
    ///
    /// The curve is `P(t) = c + m·cos t + w·sin t` with `m = major − center` and
    /// `w = perp(m)·(b / |m|)` (CCW from `m`). `b` may exceed `|m|`.
    Ellipse {
        /// Centre point entity.
        center: EntityId,
        /// Major-axis end point entity (`|major − center|` is the semi-major radius).
        major: EntityId,
        /// Minor radius param `b`, mm.
        minor_radius: ParamId,
    },
    /// Elliptical arc from eccentric angle `t0` to `t1` (CCW, sweep `(t1 − t0) mod 2π` in
    /// `(0, 2π]`). `start = P(t0)` and `end = P(t1)` are internal equations (7 DOF).
    EllipticArc {
        /// Centre point entity.
        center: EntityId,
        /// Major-axis end point entity.
        major: EntityId,
        /// Minor radius param `b`, mm.
        minor_radius: ParamId,
        /// Start point entity.
        start: EntityId,
        /// End point entity.
        end: EntityId,
        /// Eccentric angle of the start point, rad.
        t0: ParamId,
        /// Eccentric angle of the end point, rad.
        t1: ParamId,
    },
}

impl EntityKind {
    /// Short lowercase name ("point", "line", "circle", "arc", "ellipse", "elliptical arc").
    pub fn name(&self) -> &'static str {
        match self {
            EntityKind::Point { .. } => "point",
            EntityKind::Line { .. } => "line",
            EntityKind::Circle { .. } => "circle",
            EntityKind::Arc { .. } => "arc",
            EntityKind::Ellipse { .. } => "ellipse",
            EntityKind::EllipticArc { .. } => "elliptical arc",
        }
    }

    /// Entities this one is built on (its points), in declaration order.
    pub fn children(&self) -> SmallVec<[EntityId; 3]> {
        match *self {
            EntityKind::Point { .. } => SmallVec::new(),
            EntityKind::Line { start, end } => [start, end].into_iter().collect(),
            EntityKind::Circle { center, .. } => [center].into_iter().collect(),
            EntityKind::Arc { center, start, end } => [center, start, end].into_iter().collect(),
            EntityKind::Ellipse { center, major, .. } => [center, major].into_iter().collect(),
            EntityKind::EllipticArc {
                center,
                major,
                start,
                end,
                ..
            } => [center, major, start, end].into_iter().collect(),
        }
    }

    /// Parameters this entity owns directly (not those of its child points).
    pub fn own_params(&self) -> SmallVec<[ParamId; 2]> {
        match *self {
            EntityKind::Point { x, y } => [x, y].into_iter().collect(),
            EntityKind::Circle { radius, .. } => [radius].into_iter().collect(),
            EntityKind::Ellipse { minor_radius, .. } => [minor_radius].into_iter().collect(),
            EntityKind::EllipticArc {
                minor_radius,
                t0,
                t1,
                ..
            } => [minor_radius, t0, t1].into_iter().collect(),
            EntityKind::Line { .. } | EntityKind::Arc { .. } => SmallVec::new(),
        }
    }

    /// The point entity at end `end` of a line, arc or elliptical arc; `None` for points,
    /// circles and full ellipses.
    pub fn endpoint(&self, end: End) -> Option<EntityId> {
        match (self, end) {
            (
                EntityKind::Line { start, .. }
                | EntityKind::Arc { start, .. }
                | EntityKind::EllipticArc { start, .. },
                End::Start,
            ) => Some(*start),
            (
                EntityKind::Line { end, .. }
                | EntityKind::Arc { end, .. }
                | EntityKind::EllipticArc { end, .. },
                End::End,
            ) => Some(*end),
            _ => None,
        }
    }

    fn for_each_param_mut(&mut self, mut f: impl FnMut(&mut ParamId)) {
        match self {
            EntityKind::Point { x, y } => {
                f(x);
                f(y);
            }
            EntityKind::Circle { radius, .. } => f(radius),
            EntityKind::Ellipse { minor_radius, .. } => f(minor_radius),
            EntityKind::EllipticArc {
                minor_radius,
                t0,
                t1,
                ..
            } => {
                f(minor_radius);
                f(t0);
                f(t1);
            }
            EntityKind::Line { .. } | EntityKind::Arc { .. } => {}
        }
    }

    pub(crate) fn is_point(&self) -> bool {
        matches!(self, EntityKind::Point { .. })
    }
    pub(crate) fn is_line(&self) -> bool {
        matches!(self, EntityKind::Line { .. })
    }
    pub(crate) fn is_arc(&self) -> bool {
        matches!(self, EntityKind::Arc { .. })
    }
    /// Circle or arc.
    pub(crate) fn is_round(&self) -> bool {
        matches!(self, EntityKind::Circle { .. } | EntityKind::Arc { .. })
    }
    /// Ellipse or elliptical arc.
    pub(crate) fn is_ellipse(&self) -> bool {
        matches!(
            self,
            EntityKind::Ellipse { .. } | EntityKind::EllipticArc { .. }
        )
    }
    /// Circle, arc, ellipse or elliptical arc: curves with a centre point.
    pub(crate) fn has_center(&self) -> bool {
        self.is_round() || self.is_ellipse()
    }
    /// Any curve (line, circle, arc, ellipse, elliptical arc).
    pub(crate) fn is_curve(&self) -> bool {
        !self.is_point()
    }
}

/// One sketch entity.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Entity {
    /// Geometry.
    pub kind: EntityKind,
    /// Construction geometry is excluded from profiles.
    pub construction: bool,
    /// Where the entity comes from.
    pub origin: EntityOrigin,
}

// ---- Constraints ------------------------------------------------------------------------

/// Target of a horizontal/vertical constraint.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum HvTarget {
    /// A line.
    Line(EntityId),
    /// Two points.
    Points(EntityId, EntityId),
}

/// One end of a curve with endpoints (line or arc).
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum End {
    /// The `start` point.
    Start,
    /// The `end` point.
    End,
}

/// Which tangency a tangent constraint keeps.
///
/// **Line–circle/arc** (either argument order; an arc uses its circle). With the line oriented
/// `start → end`, `d = end − start` and the signed centre distance
/// `s = cross(d, center − start) / |d|` (positive = centre left of the line):
/// - `External`: `s = +r` (circle on the left of the oriented line),
/// - `Internal`: `s = −r` (circle on the right).
///
/// **Circle/arc–circle/arc:** `External` is `|c1 − c2| = r1 + r2`, `Internal` is
/// `|c1 − c2| = |r1 − r2|`.
///
/// **`AtEndpoint`:** line/arc–line/arc (at least one arc), tangent at end `a_end` of `a` and
/// end `b_end` of `b`. When the constraint is added, those two points must be the same point
/// entity or be joined by a chain of `Coincident` constraints. With `t_a`, `t_b` the curves'
/// directions of travel there (line: `end − start`; arc: CCW tangent), the residual is
/// `cross(t_a, t_b)`, and `reversed == false` means `dot(t_a, t_b) > 0`. The stored ends keep
/// the residual well defined even if a joining `Coincident` is removed later.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum TangentKind {
    /// Line: circle left of the oriented line. Circles: touching from outside.
    External,
    /// Line: circle right of the oriented line. Circles: one inside the other.
    Internal,
    /// Curve–curve tangency at a shared endpoint.
    AtEndpoint {
        /// Which end of `a` is shared.
        a_end: End,
        /// Which end of `b` is shared.
        b_end: End,
        /// The directions of travel are opposite at the shared point.
        reversed: bool,
    },
}

/// Side of a signed distance, captured at creation so geometry does not flip (chirality).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Side {
    /// Positive side.
    Positive,
    /// Negative side.
    Negative,
}

/// Which oriented directions an angle dimension measures between.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AngleBranch {
    /// Use `start − end` instead of `end − start` for line `a`.
    pub reverse_a: bool,
    /// Use `start − end` instead of `end − start` for line `b`.
    pub reverse_b: bool,
}

/// Geometric constraint or dimension. The curvature (G2) constraint arrives with SK-08.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ConstraintKind {
    /// Point–point coincidence.
    Coincident {
        /// First point.
        a: EntityId,
        /// Second point.
        b: EntityId,
    },
    /// Point on a line, circle, arc, ellipse or elliptical arc (the full ellipse). On an
    /// ellipse the constraint owns one auxiliary param, the curve parameter `t` of the point
    /// ([`Constraint::aux`]).
    PointOnCurve {
        /// The point.
        point: EntityId,
        /// The curve (line, circle, arc, ellipse or elliptical arc).
        curve: EntityId,
    },
    /// Point at the midpoint of a line.
    Midpoint {
        /// The point.
        point: EntityId,
        /// The line.
        line: EntityId,
    },
    /// Horizontal line or point pair.
    Horizontal(HvTarget),
    /// Vertical line or point pair.
    Vertical(HvTarget),
    /// Line–line parallel.
    Parallel {
        /// First line.
        a: EntityId,
        /// Second line.
        b: EntityId,
    },
    /// Line–line perpendicular.
    Perpendicular {
        /// First line.
        a: EntityId,
        /// Second line.
        b: EntityId,
    },
    /// Line–line collinear.
    Collinear {
        /// First line.
        a: EntityId,
        /// Second line.
        b: EntityId,
    },
    /// Tangency: line–circle/arc, circle/arc–circle/arc, line–ellipse/elliptical arc (the
    /// full ellipse), or line/arc–line/arc at a shared endpoint (`TangentKind::AtEndpoint`).
    Tangent {
        /// First curve.
        a: EntityId,
        /// Second curve.
        b: EntityId,
        /// Which tangency.
        kind: TangentKind,
    },
    /// Circle/arc/ellipse pair with coincident centres.
    Concentric {
        /// First circle or arc.
        a: EntityId,
        /// Second circle or arc.
        b: EntityId,
    },
    /// Equal length (line–line, line–arc), radius (circle/arc–circle/arc) or both axes
    /// (ellipse/elliptical arc–ellipse/elliptical arc).
    Equal {
        /// First entity.
        a: EntityId,
        /// Second entity.
        b: EntityId,
    },
    /// `a` and `b` (two points, or two curves of the same kind; not elliptical arcs) mirror
    /// about line `axis`.
    Symmetric {
        /// First entity.
        a: EntityId,
        /// Second entity.
        b: EntityId,
        /// Mirror line.
        axis: EntityId,
    },
    /// Locks all params of an entity (removed from the unknowns).
    Fix {
        /// The fixed entity.
        entity: EntityId,
    },
    /// Driving or driven dimension.
    Dimension(Dimension),
}

impl ConstraintKind {
    /// Short name of the variant ("coincident", "distance", ...).
    pub fn name(&self) -> &'static str {
        match self {
            ConstraintKind::Coincident { .. } => "coincident",
            ConstraintKind::PointOnCurve { .. } => "point on curve",
            ConstraintKind::Midpoint { .. } => "midpoint",
            ConstraintKind::Horizontal(_) => "horizontal",
            ConstraintKind::Vertical(_) => "vertical",
            ConstraintKind::Parallel { .. } => "parallel",
            ConstraintKind::Perpendicular { .. } => "perpendicular",
            ConstraintKind::Collinear { .. } => "collinear",
            ConstraintKind::Tangent { .. } => "tangent",
            ConstraintKind::Concentric { .. } => "concentric",
            ConstraintKind::Equal { .. } => "equal",
            ConstraintKind::Symmetric { .. } => "symmetric",
            ConstraintKind::Fix { .. } => "fix",
            ConstraintKind::Dimension(d) => d.kind.name(),
        }
    }

    /// Every entity the constraint references, in declaration order.
    pub fn entities(&self) -> SmallVec<[EntityId; 3]> {
        let list: &[EntityId] = match self {
            ConstraintKind::Coincident { a, b }
            | ConstraintKind::Parallel { a, b }
            | ConstraintKind::Perpendicular { a, b }
            | ConstraintKind::Collinear { a, b }
            | ConstraintKind::Tangent { a, b, .. }
            | ConstraintKind::Concentric { a, b }
            | ConstraintKind::Equal { a, b } => &[*a, *b],
            ConstraintKind::PointOnCurve {
                point,
                curve: other,
            }
            | ConstraintKind::Midpoint { point, line: other } => &[*point, *other],
            ConstraintKind::Horizontal(t) | ConstraintKind::Vertical(t) => match t {
                HvTarget::Line(l) => &[*l],
                HvTarget::Points(a, b) => &[*a, *b],
            },
            ConstraintKind::Symmetric { a, b, axis } => &[*a, *b, *axis],
            ConstraintKind::Fix { entity } => &[*entity],
            ConstraintKind::Dimension(d) => return d.kind.entities(),
        };
        SmallVec::from_slice(list)
    }

    /// The dimension, if this is one.
    pub fn as_dimension(&self) -> Option<&Dimension> {
        match self {
            ConstraintKind::Dimension(d) => Some(d),
            _ => None,
        }
    }
}

/// Dimension kinds.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum DimensionKind {
    /// Distance: point–point, point–line, line–line (parallel), or point/line to a circle or
    /// arc edge. Signed forms keep `side`.
    Distance {
        /// First entity.
        a: EntityId,
        /// Second entity.
        b: EntityId,
        /// Side of the signed distance.
        side: Side,
    },
    /// Horizontal distance between two points.
    HorizontalDistance {
        /// First point.
        a: EntityId,
        /// Second point.
        b: EntityId,
        /// Sign of `b.x − a.x`.
        side: Side,
    },
    /// Vertical distance between two points.
    VerticalDistance {
        /// First point.
        a: EntityId,
        /// Second point.
        b: EntityId,
        /// Sign of `b.y − a.y`.
        side: Side,
    },
    /// Length of a line.
    Length {
        /// The line.
        line: EntityId,
    },
    /// Angle between two oriented lines.
    Angle {
        /// First line.
        a: EntityId,
        /// Second line.
        b: EntityId,
        /// Which directions are measured.
        branch: AngleBranch,
    },
    /// Radius of a circle or arc.
    Radius {
        /// The circle or arc.
        curve: EntityId,
    },
    /// Diameter of a circle or arc.
    Diameter {
        /// The circle or arc.
        curve: EntityId,
    },
    /// Arc length of an arc.
    ArcLength {
        /// The arc.
        arc: EntityId,
    },
    /// Semi-major radius `|major − center|` of an ellipse or elliptical arc (SK-22).
    EllipseMajor {
        /// The ellipse or elliptical arc.
        ellipse: EntityId,
    },
    /// Minor radius `b` of an ellipse or elliptical arc (SK-22).
    EllipseMinor {
        /// The ellipse or elliptical arc.
        ellipse: EntityId,
    },
}

/// What a dimension's `value` measures.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Quantity {
    /// mm.
    Length,
    /// rad.
    Angle,
}

impl DimensionKind {
    /// Whether `value` is a length (mm) or an angle (rad).
    pub fn quantity(&self) -> Quantity {
        match self {
            DimensionKind::Angle { .. } => Quantity::Angle,
            _ => Quantity::Length,
        }
    }

    /// Short name ("distance", "radius", ...).
    pub fn name(&self) -> &'static str {
        match self {
            DimensionKind::Distance { .. } => "distance",
            DimensionKind::HorizontalDistance { .. } => "horizontal distance",
            DimensionKind::VerticalDistance { .. } => "vertical distance",
            DimensionKind::Length { .. } => "length",
            DimensionKind::Angle { .. } => "angle",
            DimensionKind::Radius { .. } => "radius",
            DimensionKind::Diameter { .. } => "diameter",
            DimensionKind::ArcLength { .. } => "arc length",
            DimensionKind::EllipseMajor { .. } => "ellipse major radius",
            DimensionKind::EllipseMinor { .. } => "ellipse minor radius",
        }
    }

    /// Every entity the dimension references, in declaration order.
    pub fn entities(&self) -> SmallVec<[EntityId; 3]> {
        match *self {
            DimensionKind::Distance { a, b, .. }
            | DimensionKind::HorizontalDistance { a, b, .. }
            | DimensionKind::VerticalDistance { a, b, .. }
            | DimensionKind::Angle { a, b, .. } => [a, b].into_iter().collect(),
            DimensionKind::Length { line: e }
            | DimensionKind::Radius { curve: e }
            | DimensionKind::Diameter { curve: e }
            | DimensionKind::ArcLength { arc: e }
            | DimensionKind::EllipseMajor { ellipse: e }
            | DimensionKind::EllipseMinor { ellipse: e } => [e].into_iter().collect(),
        }
    }
}

/// A dimension: one equation when driving, a measurement when driven.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Dimension {
    /// What is measured.
    pub kind: DimensionKind,
    /// mm or rad, written by `amg-features::sketch` from the S01 `Expr` before each solve.
    pub value: f64,
    /// Driving dimensions constrain; driven ones only measure.
    pub driving: bool,
    /// Label placement in sketch coordinates (UI only).
    pub label_pos: Point2,
}

/// A stored constraint.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Constraint {
    /// What it constrains.
    pub kind: ConstraintKind,
    /// Auxiliary params owned by the constraint: the curve parameter `t` of a point-on-ellipse
    /// constraint (`Sketch::add_constraint` allocates it). Empty for every other constraint.
    pub aux: SmallVec<[ParamId; 1]>,
}
