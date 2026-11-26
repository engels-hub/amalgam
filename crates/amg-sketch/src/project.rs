//! Projected geometry (SK-09): the plain-data form of what a projection link puts in a sketch.
//!
//! `amg-sketch` knows nothing about the 3D model. The timeline feature
//! (`amg-features::sketch`) resolves its references, computes [`ProjectedGeom`]s in sketch
//! coordinates and writes them with [`Sketch::add_projected`](crate::Sketch::add_projected) and
//! [`Sketch::refresh_projected`](crate::Sketch::refresh_projected). The entities they create have
//! [`EntityOrigin::Projected`](crate::EntityOrigin) and locked parameters, so the solver and the
//! edit operations never move them.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use amg_core::{Point2, Tolerance, Vec2};
use serde::{Deserialize, Serialize};

use crate::error::SketchError;
use crate::geom::{EllipseArc2, check_knots};

/// How close a sweep must be to a full turn to count as one.
const FULL_TURN: f64 = 1e-9;

/// One piece of projected geometry in sketch coordinates (mm, rad): one point or curve entity.
///
/// Angles follow [`Curve2`](crate::Curve2): a negative sweep (or `t1 < t0`) runs clockwise.
/// The sketch stores arcs counter-clockwise, so the direction is not kept; a sweep of a full
/// turn makes a circle or a full ellipse.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ProjectedGeom {
    /// A point.
    Point(Point2),
    /// A segment from `a` to `b`.
    Line {
        /// Start.
        a: Point2,
        /// End.
        b: Point2,
    },
    /// A full circle.
    Circle {
        /// Centre.
        center: Point2,
        /// Radius, mm.
        radius: f64,
    },
    /// A circular arc.
    Arc {
        /// Centre.
        center: Point2,
        /// Radius, mm.
        radius: f64,
        /// Angle of the start point from `+x`.
        start_angle: f64,
        /// Signed sweep.
        sweep: f64,
    },
    /// An ellipse or elliptical arc in the parameterisation of
    /// [`EllipseArc2`](crate::EllipseArc2).
    Ellipse {
        /// Centre.
        center: Point2,
        /// Centre to the end of the named axis, mm.
        major: Vec2,
        /// The other semi-axis, mm.
        minor_radius: f64,
        /// Eccentric angle at the start.
        t0: f64,
        /// Eccentric angle at the end.
        t1: f64,
    },
    /// A clamped, non-periodic B-spline over its whole knot range.
    Spline {
        /// Degree, `1..=10`.
        degree: u8,
        /// Full knot vector (`poles + degree + 1` entries).
        knots: Vec<f64>,
        /// Control points.
        poles: Vec<Point2>,
        /// One positive weight per pole; `None` when non-rational.
        weights: Option<Vec<f64>>,
    },
}

impl ProjectedGeom {
    /// Short name of the entity this makes ("point", "line", "circle", "arc", "ellipse",
    /// "elliptical arc", "spline"), as [`EntityKind::name`](crate::EntityKind::name).
    pub fn name(&self) -> &'static str {
        match self {
            ProjectedGeom::Point(_) => "point",
            ProjectedGeom::Line { .. } => "line",
            ProjectedGeom::Circle { .. } => "circle",
            ProjectedGeom::Arc { sweep, .. } if is_full(*sweep) => "circle",
            ProjectedGeom::Arc { .. } => "arc",
            ProjectedGeom::Ellipse { t0, t1, .. } if is_full(t1 - t0) => "ellipse",
            ProjectedGeom::Ellipse { .. } => "elliptical arc",
            ProjectedGeom::Spline { .. } => "spline",
        }
    }

    /// Checks that the geometry can be a sketch entity: finite values, positive radii, an arc
    /// that sweeps, a valid knot vector and positive weights.
    ///
    /// # Errors
    /// `InvalidValue`, `Degenerate` or `NotApplicable`.
    pub fn validate(&self) -> Result<(), SketchError> {
        let finite = |what, v: f64| {
            if v.is_finite() {
                Ok(())
            } else {
                Err(SketchError::InvalidValue { what, value: v })
            }
        };
        let point = |p: Point2| {
            finite("projected coordinate", p.x)?;
            finite("projected coordinate", p.y)
        };
        let radius = |r: f64| {
            if r.is_finite() && r > Tolerance::DEFAULT.linear {
                Ok(())
            } else {
                Err(SketchError::InvalidValue {
                    what: "projected radius",
                    value: r,
                })
            }
        };
        let sweeps = |s: f64| {
            finite("projected angle", s)?;
            if s.abs() > Tolerance::DEFAULT.angular {
                Ok(())
            } else {
                Err(SketchError::Degenerate("a projected arc has no sweep"))
            }
        };
        match self {
            ProjectedGeom::Point(p) => point(*p),
            ProjectedGeom::Line { a, b } => {
                point(*a)?;
                point(*b)
            }
            ProjectedGeom::Circle { center, radius: r } => {
                point(*center)?;
                radius(*r)
            }
            ProjectedGeom::Arc {
                center,
                radius: r,
                start_angle,
                sweep,
            } => {
                point(*center)?;
                radius(*r)?;
                finite("projected angle", *start_angle)?;
                sweeps(*sweep)
            }
            ProjectedGeom::Ellipse {
                center,
                major,
                minor_radius,
                t0,
                t1,
            } => {
                point(*center)?;
                point(*major)?;
                radius(major.length())?;
                radius(*minor_radius)?;
                finite("projected angle", *t0)?;
                sweeps(t1 - t0)
            }
            ProjectedGeom::Spline {
                degree,
                knots,
                poles,
                weights,
            } => {
                for &p in poles {
                    point(p)?;
                }
                check_knots(usize::from(*degree), poles.len(), knots)?;
                if let Some(w) = weights {
                    if w.len() != poles.len() {
                        return Err(SketchError::NotApplicable("one weight per pole is needed"));
                    }
                    if let Some(&bad) = w.iter().find(|&&v| !(v.is_finite() && v > 0.0)) {
                        return Err(SketchError::InvalidValue {
                            what: "spline weight",
                            value: bad,
                        });
                    }
                }
                Ok(())
            }
        }
    }
}

pub(crate) fn is_full(sweep: f64) -> bool {
    sweep.abs() >= TAU - FULL_TURN
}

/// A circular arc as `(start angle, end angle)` counter-clockwise.
pub(crate) fn ccw_angles(start_angle: f64, sweep: f64) -> (f64, f64) {
    if sweep < 0.0 {
        (start_angle + sweep, start_angle)
    } else {
        (start_angle, start_angle + sweep)
    }
}

/// The counter-clockwise form of an ellipse piece, expressed on the axis frame closest to
/// `previous` (the entity's major vector before a refresh), so that its major point does not
/// jump when the source reports the same ellipse with flipped or swapped axes.
pub(crate) fn ellipse_form(
    center: Point2,
    major: Vec2,
    minor_radius: f64,
    t0: f64,
    t1: f64,
    previous: Option<Vec2>,
) -> EllipseArc2 {
    let mut e = EllipseArc2 {
        center,
        major,
        minor_radius,
        t0: t0.min(t1),
        t1: t0.max(t1),
    };
    let Some(old) = previous else {
        return e;
    };
    // The same ellipse on the frame turned by a quarter turn: m' = w, b' = |m|, t' = t − π/2.
    let turned = |e: EllipseArc2| EllipseArc2 {
        major: e.minor(),
        minor_radius: e.semi_major(),
        t0: e.t0 - FRAC_PI_2,
        t1: e.t1 - FRAC_PI_2,
        ..e
    };
    let alignment = |e: &EllipseArc2| e.major.normalize_or_zero().dot(old.normalize_or_zero());
    let quarter = turned(e);
    if alignment(&quarter).abs() > alignment(&e).abs() + 1e-9 {
        e = quarter;
    }
    if alignment(&e) < 0.0 {
        e = EllipseArc2 {
            major: -e.major,
            t0: e.t0 + PI,
            t1: e.t1 + PI,
            ..e
        };
    }
    // Keep the start angle near zero, whole turns apart.
    let turns = (e.t0 / TAU).round() * TAU;
    e.t0 -= turns;
    e.t1 -= turns;
    e
}
