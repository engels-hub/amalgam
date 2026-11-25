//! [`CurveEval`] adapters: straight edges and circles/arcs for the analytic path kinds of
//! [`RefGeom`], and [`FreeCurve`] for elliptic and B-spline B-rep edges and sketch curves
//! (CON-03).

use std::f64::consts::TAU;
use std::sync::Arc;

use amg_core::{Dir3, Plane, Point3, Segment3, Vec2, Vec3};
use amg_nurbs::NurbsCurve3;
use amg_sketch::Curve2;

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

/// Quadrature pieces per knot span.
const PIECES_PER_SPAN: u32 = 16;
/// Quadrature pieces of an ellipse, which has a single span but strongly varying speed.
const ELLIPSE_PIECES: u32 = 64;
/// Samples per piece that start the closest-point iteration.
const SAMPLES_PER_PIECE: u32 = 4;
/// Iteration cap of the safeguarded Newton solves; they stop earlier once converged.
const MAX_ITERATIONS: u32 = 100;

/// 8-point Gauss–Legendre nodes and weights on [−1, 1] (positive half).
const GAUSS: [(f64, f64); 4] = [
    (0.183_434_642_495_649_8, 0.362_683_783_378_362),
    (0.525_532_409_916_329, 0.313_706_645_877_887_3),
    (0.796_666_477_413_626_7, 0.222_381_034_453_374_5),
    (0.960_289_856_497_536_2, 0.101_228_536_290_376_26),
];

#[derive(Clone, Debug)]
enum Form {
    /// A 3D B-spline (B-rep edge).
    Nurbs(NurbsCurve3),
    /// An elliptic B-rep edge; the parameter is the eccentric angle from `frame.x`.
    Ellipse {
        frame: Plane,
        major: f64,
        minor: f64,
    },
    /// A sketch curve (ellipse or B-spline) on its sketch plane.
    Planar { curve: Curve2, plane: Plane },
}

impl Form {
    /// `(C, C', C'')` at `t`.
    fn derivs(&self, t: f64) -> (Point3, Vec3, Vec3) {
        match self {
            Form::Nurbs(c) => {
                let d = c.derivs(t, 2);
                (d[0], d[1], d[2])
            }
            Form::Ellipse {
                frame,
                major,
                minor,
            } => {
                let (s, c) = t.sin_cos();
                let (x, y) = (frame.x.get() * *major, frame.y.get() * *minor);
                let radial = x * c + y * s;
                (frame.origin + radial, y * c - x * s, -radial)
            }
            Form::Planar { curve, plane } => {
                let vector = |v: Vec2| plane.x.get() * v.x + plane.y.get() * v.y;
                let p = curve.eval(t);
                let d2 = match curve {
                    Curve2::Line(_) => Vec2::ZERO,
                    // The parameter is an angle turning at unit rate (either way), so C''
                    // points back to the centre.
                    Curve2::Circle(c) => c.center - p,
                    Curve2::Arc(a) => a.center - p,
                    Curve2::Ellipse(e) => e.center - p,
                    Curve2::BSpline(b) => b.derivs(t).2,
                };
                (plane.to_world(p), vector(curve.derivative(t)), vector(d2))
            }
        }
    }
}

/// A free-form curve over [`range`](CurveEval::range), with its arc length tabulated per
/// quadrature piece (8-point Gauss–Legendre).
#[derive(Clone, Debug)]
pub struct FreeCurve {
    form: Form,
    /// Piece boundaries, increasing, from the start to the end of the range.
    knots: Vec<f64>,
    /// Arc length from the start to each boundary.
    lengths: Vec<f64>,
}

impl FreeCurve {
    /// The B-spline `curve` from `t0` to `t1`.
    pub fn nurbs(curve: NurbsCurve3, t0: f64, t1: f64) -> Result<FreeCurve, ConstructError> {
        let breaks = curve.knots().as_slice().to_vec();
        FreeCurve::new(Form::Nurbs(curve), t0, t1, &breaks, PIECES_PER_SPAN)
    }

    /// The ellipse in `frame` (major axis along `frame.x`) between the eccentric angles `t0`
    /// and `t1`.
    pub fn ellipse(
        frame: Plane,
        major: f64,
        minor: f64,
        t0: f64,
        t1: f64,
    ) -> Result<FreeCurve, ConstructError> {
        if !(major > 0.0 && minor > 0.0 && major.is_finite() && minor.is_finite()) {
            return Err(ConstructError::Degenerate("ellipse radius is not positive"));
        }
        let form = Form::Ellipse {
            frame,
            major,
            minor,
        };
        FreeCurve::new(form, t0, t1, &[], ELLIPSE_PIECES)
    }

    /// The whole sketch curve `curve` on `plane`, with the sketch's parameterisation
    /// (`Curve2::param_range`).
    pub fn planar(curve: Curve2, plane: Plane) -> Result<FreeCurve, ConstructError> {
        let (t0, t1) = curve.param_range();
        let (breaks, pieces): (Vec<f64>, u32) = match &curve {
            Curve2::BSpline(b) => (
                b.spans().into_iter().map(|(a, _)| a).collect(),
                PIECES_PER_SPAN,
            ),
            _ => (Vec::new(), ELLIPSE_PIECES),
        };
        FreeCurve::new(Form::Planar { curve, plane }, t0, t1, &breaks, pieces)
    }

    fn new(
        form: Form,
        t0: f64,
        t1: f64,
        breaks: &[f64],
        pieces: u32,
    ) -> Result<FreeCurve, ConstructError> {
        if !(t0.is_finite() && t1.is_finite() && t1 > t0) {
            return Err(ConstructError::Degenerate("empty curve range"));
        }
        let mut spans = vec![t0];
        spans.extend(breaks.iter().copied().filter(|&k| k > t0 && k < t1));
        spans.push(t1);
        spans.dedup();
        let mut knots = vec![t0];
        for w in spans.windows(2) {
            for i in 1..pieces {
                knots.push(w[0] + (w[1] - w[0]) * f64::from(i) / f64::from(pieces));
            }
            knots.push(w[1]);
        }
        let mut curve = FreeCurve {
            form,
            knots,
            lengths: vec![0.0],
        };
        let mut total = 0.0;
        for i in 1..curve.knots.len() {
            total += curve.arc(curve.knots[i - 1], curve.knots[i]);
            curve.lengths.push(total);
        }
        if !(total.is_finite() && total > 0.0) {
            return Err(ConstructError::Degenerate("zero-length curve"));
        }
        Ok(curve)
    }

    /// Arc length between two parameters of one piece.
    fn arc(&self, a: f64, b: f64) -> f64 {
        let (mid, half) = (0.5 * (a + b), 0.5 * (b - a));
        let speed = |t: f64| self.form.derivs(t).1.length();
        GAUSS
            .iter()
            .map(|&(x, w)| w * (speed(mid - half * x) + speed(mid + half * x)))
            .sum::<f64>()
            * half
    }
}

/// The root of an increasing `f` in `[lo, hi]`, from `t`: Newton on `(f, f')`, falling back to
/// bisection whenever a step leaves the bracket.
fn solve_increasing(mut lo: f64, mut hi: f64, mut t: f64, f: impl Fn(f64) -> (f64, f64)) -> f64 {
    for _ in 0..MAX_ITERATIONS {
        let (v, d) = f(t);
        if v > 0.0 {
            hi = t;
        } else if v < 0.0 {
            lo = t;
        } else {
            return t;
        }
        let newton = t - v / d;
        // A step that rounds to `t` has converged; otherwise it must shrink the bracket.
        let next = if d > 0.0 && (newton.to_bits() == t.to_bits() || (newton > lo && newton < hi)) {
            newton
        } else {
            0.5 * (lo + hi)
        };
        if next.to_bits() == t.to_bits() {
            break;
        }
        t = next;
    }
    t
}

impl CurveEval for FreeCurve {
    fn range(&self) -> (f64, f64) {
        (self.knots[0], self.knots[self.knots.len() - 1])
    }
    fn eval(&self, t: f64) -> Point3 {
        self.form.derivs(t).0
    }
    fn d1(&self, t: f64) -> Vec3 {
        self.form.derivs(t).1
    }
    fn d2(&self, t: f64) -> Vec3 {
        self.form.derivs(t).2
    }
    fn length(&self) -> f64 {
        self.lengths[self.lengths.len() - 1]
    }
    fn param_at_length(&self, s: f64) -> f64 {
        let (t0, t1) = self.range();
        if s <= 0.0 {
            return t0;
        }
        if s >= self.length() {
            return t1;
        }
        // The piece with lengths[i - 1] <= s < lengths[i].
        let i = self.lengths.partition_point(|&l| l <= s);
        let (a, b) = (self.knots[i - 1], self.knots[i]);
        let rest = s - self.lengths[i - 1];
        let guess = a + (b - a) * rest / (self.lengths[i] - self.lengths[i - 1]);
        solve_increasing(a, b, guess, |t| {
            (self.arc(a, t) - rest, self.form.derivs(t).1.length())
        })
    }
    fn closest_param(&self, p: Point3) -> f64 {
        let n = SAMPLES_PER_PIECE;
        let samples: Vec<f64> = self
            .knots
            .windows(2)
            .flat_map(|w| (0..n).map(move |i| w[0] + (w[1] - w[0]) * f64::from(i) / f64::from(n)))
            .chain([self.range().1])
            .collect();
        let distance = |t: f64| self.eval(t).distance_squared(p);
        let mut best = 0;
        for i in 1..samples.len() {
            if distance(samples[i]) < distance(samples[best]) {
                best = i;
            }
        }
        let lo = samples[best.saturating_sub(1)];
        let hi = samples[(best + 1).min(samples.len() - 1)];
        // Half the derivative of the squared distance, and its derivative.
        let t = solve_increasing(lo, hi, samples[best], |t| {
            let (c, d1, d2) = self.form.derivs(t);
            ((c - p).dot(d1), d1.length_squared() + (c - p).dot(d2))
        });
        if distance(t) <= distance(samples[best]) {
            t
        } else {
            samples[best]
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
