//! [`CurveEval`] adapters for the analytic path kinds of [`RefGeom`]: straight edges and
//! circles/arcs. Free-form curves come with their own `CurveEval` (CON-03).

use std::f64::consts::TAU;
use std::sync::Arc;

use amg_core::{Dir3, Plane, Point3, Segment3, Vec3};

use super::{ConstructError, CurveEval, RefGeom};

/// A straight segment parameterised by arc length: `t ∈ [0, length]`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SegmentCurve {
    a: Point3,
    dir: Vec3,
    len: f64,
}

impl SegmentCurve {
    /// The segment `s`; `Degenerate` if it has zero length.
    pub fn new(s: &Segment3) -> Result<SegmentCurve, ConstructError> {
        let len = s.length();
        if !len.is_finite() || len <= 0.0 {
            return Err(ConstructError::Degenerate("zero-length edge"));
        }
        Ok(SegmentCurve {
            a: s.a,
            dir: (s.b - s.a) / len,
            len,
        })
    }
}

impl CurveEval for SegmentCurve {
    fn range(&self) -> (f64, f64) {
        (0.0, self.len)
    }
    fn eval(&self, t: f64) -> Point3 {
        self.a + self.dir * t
    }
    fn d1(&self, _t: f64) -> Vec3 {
        self.dir
    }
    fn d2(&self, _t: f64) -> Vec3 {
        Vec3::ZERO
    }
    fn length(&self) -> f64 {
        self.len
    }
    fn param_at_length(&self, s: f64) -> f64 {
        s
    }
    fn closest_param(&self, p: Point3) -> f64 {
        (p - self.a).dot(self.dir).clamp(0.0, self.len)
    }
}

/// A circle or arc parameterised by angle in the frame `Plane::from_normal(center, normal)`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CircleCurve {
    frame: Plane,
    radius: f64,
    start: f64,
    end: f64,
}

impl CircleCurve {
    /// Circle of `radius` about `center`/`normal`; `arc = (start, end)` radians, `None` for the
    /// full circle `(0, 2π)`. `Degenerate` for a non-positive radius or an empty arc.
    pub fn new(
        center: Point3,
        normal: Dir3,
        radius: f64,
        arc: Option<(f64, f64)>,
    ) -> Result<CircleCurve, ConstructError> {
        if !radius.is_finite() || radius <= 0.0 {
            return Err(ConstructError::Degenerate("circle radius is not positive"));
        }
        let (start, end) = arc.unwrap_or((0.0, TAU));
        if !(start.is_finite() && end.is_finite() && end > start) {
            return Err(ConstructError::Degenerate("empty arc"));
        }
        Ok(CircleCurve {
            frame: Plane::from_normal(center, normal),
            radius,
            start,
            end,
        })
    }

    fn radial(&self, t: f64) -> Vec3 {
        self.frame.x.get() * t.cos() + self.frame.y.get() * t.sin()
    }
}

impl CurveEval for CircleCurve {
    fn range(&self) -> (f64, f64) {
        (self.start, self.end)
    }
    fn eval(&self, t: f64) -> Point3 {
        self.frame.origin + self.radial(t) * self.radius
    }
    fn d1(&self, t: f64) -> Vec3 {
        (self.frame.y.get() * t.cos() - self.frame.x.get() * t.sin()) * self.radius
    }
    fn d2(&self, t: f64) -> Vec3 {
        -self.radial(t) * self.radius
    }
    fn length(&self) -> f64 {
        (self.end - self.start) * self.radius
    }
    fn param_at_length(&self, s: f64) -> f64 {
        self.start + s / self.radius
    }
    fn closest_param(&self, p: Point3) -> f64 {
        let uv = self.frame.to_local(p);
        let a = uv.y.atan2(uv.x);
        // Into [start, start + 2π).
        let t = self.start + (a - self.start).rem_euclid(TAU);
        if t <= self.end {
            return t;
        }
        // Outside the arc: the nearer end, measured around the circle.
        if t - self.end <= self.start + TAU - t {
            self.end
        } else {
            self.start
        }
    }
}

/// The path curve of `g`: free-form curves as they are, straight edges and circles through the
/// adapters. Infinite lines and non-curves are `WrongInput`.
pub fn curve_of(g: &RefGeom) -> Result<Arc<dyn CurveEval>, ConstructError> {
    match g {
        RefGeom::Curve(c) => Ok(Arc::clone(c)),
        RefGeom::Segment(s) => Ok(Arc::new(SegmentCurve::new(s)?)),
        RefGeom::Circle {
            center,
            normal,
            radius,
            arc,
        } => Ok(Arc::new(CircleCurve::new(*center, *normal, *radius, *arc)?)),
        other => Err(ConstructError::WrongInput {
            expected: "edge or sketch curve",
            got: other.kind_name(),
        }),
    }
}
