//! Evaluated 2D sketch curves.
//!
//! `Segment2` and `Aabb2` are the `amg-core` types; circles and arcs are sketch-specific.
//! Curve queries (`eval`, `closest`, `bbox`, ...) and [`intersect`] are in `query` (SK-06);
//! ellipses and elliptical arcs (SK-22) are [`EllipseArc2`]; B-splines, NURBS and conics (SK-08)
//! are [`BSpline2`].

use amg_core::{Point2, Segment2};

mod ellipse;
mod query;
mod spline;
#[cfg(test)]
mod spline_tests;
mod spline_x;

pub use ellipse::EllipseArc2;
pub(crate) use spline::start_curvature;
pub use spline::{
    BSpline2, FitMap, MAX_DEGREE, check_knots, chord_params, clamped_uniform_knots, conic_weight,
    fit_spline,
};

pub(crate) use query::intersect_detailed;
pub use query::{CurveHit, intersect};

/// Full circle.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Circle2 {
    /// Centre, mm.
    pub center: Point2,
    /// Radius, mm.
    pub radius: f64,
}

/// Circular arc from `start_angle` sweeping by `sweep`. `sweep > 0` is CCW,
/// `|sweep| ∈ (0, 2π]`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Arc2 {
    /// Centre, mm.
    pub center: Point2,
    /// Radius, mm.
    pub radius: f64,
    /// Angle of the start point, rad.
    pub start_angle: f64,
    /// Signed sweep, rad.
    pub sweep: f64,
}

impl Arc2 {
    /// The CCW arc of the sketch arc entity: radius `|start − center|`, sweeping CCW from
    /// `start` to the direction of `end`. Coinciding start/end directions give a full turn
    /// (`sweep = 2π`).
    pub fn from_center_start_end(center: Point2, start: Point2, end: Point2) -> Arc2 {
        let ds = start - center;
        let de = end - center;
        let start_angle = ds.y.atan2(ds.x);
        let mut sweep = (de.y.atan2(de.x) - start_angle).rem_euclid(std::f64::consts::TAU);
        if sweep <= 0.0 {
            sweep = std::f64::consts::TAU;
        }
        Arc2 {
            center,
            radius: ds.length(),
            start_angle,
            sweep,
        }
    }
}

/// Evaluated geometry of one sketch curve entity.
#[derive(Clone, Debug, PartialEq)]
pub enum Curve2 {
    /// Line segment from the line's start to its end point.
    Line(Segment2),
    /// Full circle.
    Circle(Circle2),
    /// CCW arc.
    Arc(Arc2),
    /// Ellipse or elliptical arc (SK-22).
    Ellipse(EllipseArc2),
    /// B-spline, NURBS or conic (SK-08).
    BSpline(BSpline2),
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, PI, TAU};

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-12
    }

    #[test]
    fn arc_quarter_ccw() {
        let a = Arc2::from_center_start_end(
            Point2::new(1.0, 1.0),
            Point2::new(3.0, 1.0),
            Point2::new(1.0, 5.0),
        );
        assert!(close(a.radius, 2.0));
        assert!(close(a.start_angle, 0.0));
        assert!(close(a.sweep, FRAC_PI_2));
    }

    #[test]
    fn arc_three_quarters_wraps() {
        let a = Arc2::from_center_start_end(Point2::ZERO, Point2::Y, Point2::X);
        assert!(close(a.start_angle, FRAC_PI_2));
        assert!(close(a.sweep, 1.5 * PI));
    }

    #[test]
    fn arc_same_direction_is_full_turn() {
        let a = Arc2::from_center_start_end(Point2::ZERO, Point2::X, Point2::new(2.0, 0.0));
        assert!(close(a.sweep, TAU));
    }
}
