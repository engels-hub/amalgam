//! Plain-Rust evaluation of builder input curves: endpoints for the closure checks and samples
//! for the orientation and containment checks. OCCT builds the real geometry.

use amg_core::{Point2, Point3, Vec2};

use crate::geom::{BSpline2dData, BSplineCurveData, Curve2d, Curve3};

/// `perp(v)`: `v` turned 90° counter-clockwise.
pub(crate) fn perp(v: Vec2) -> Vec2 {
    Vec2::new(-v.y, v.x)
}

/// De Boor on homogeneous points `[w·x, w·y, w·z, w]` of a full knot vector.
fn de_boor(degree: usize, knots: &[f64], poles: &[[f64; 4]], t: f64) -> [f64; 4] {
    let n = poles.len();
    // The span: the largest k in [degree, n - 1] with knots[k] <= t.
    let mut k = degree;
    while k + 1 < n && knots[k + 1] <= t {
        k += 1;
    }
    let mut d: Vec<[f64; 4]> = (0..=degree).map(|j| poles[j + k - degree]).collect();
    for r in 1..=degree {
        for j in (r..=degree).rev() {
            let i = j + k - degree;
            let denom = knots[i + degree + 1 - r] - knots[i];
            let alpha = if denom > 0.0 {
                (t - knots[i]) / denom
            } else {
                0.0
            };
            let prev = d[j - 1];
            for (c, value) in d[j].iter_mut().enumerate() {
                *value = (1.0 - alpha) * prev[c] + alpha * *value;
            }
        }
    }
    d[degree]
}

/// The parameter range `[knots[p], knots[n]]` of a full knot vector.
pub(crate) fn bspline2_range(b: &BSpline2dData) -> (f64, f64) {
    let p = usize::from(b.degree);
    (b.knots[p], b.knots[b.poles.len()])
}

fn bspline2_point(b: &BSpline2dData, t: f64) -> Point2 {
    let hom: Vec<[f64; 4]> = b
        .poles
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let w = b.weights.as_ref().map_or(1.0, |w| w[i]);
            [p.x * w, p.y * w, 0.0, w]
        })
        .collect();
    let h = de_boor(usize::from(b.degree), &b.knots, &hom, t);
    Point2::new(h[0] / h[3], h[1] / h[3])
}

/// Expands distinct knots and multiplicities into a full knot vector.
pub(crate) fn full_knots(knots: &[f64], mults: &[u32]) -> Vec<f64> {
    knots
        .iter()
        .zip(mults)
        .flat_map(|(k, m)| std::iter::repeat_n(*k, *m as usize))
        .collect()
}

/// Splits a full knot vector into distinct knots and multiplicities.
pub(crate) fn distinct_knots(full: &[f64]) -> (Vec<f64>, Vec<u32>) {
    let mut knots: Vec<f64> = Vec::new();
    let mut mults: Vec<u32> = Vec::new();
    for &k in full {
        match knots.last() {
            Some(&last) if k <= last => {
                if let Some(m) = mults.last_mut() {
                    *m += 1;
                }
            }
            _ => {
                knots.push(k);
                mults.push(1);
            }
        }
    }
    (knots, mults)
}

/// The ellipse centre, unit major axis, major and minor radius, and the parameter shift that
/// turns the S03 eccentric angle into OCCT's (major ≥ minor) parameter.
pub(crate) fn ellipse_axes(
    center: Point2,
    major: Vec2,
    minor: f64,
) -> (Point2, Vec2, f64, f64, f64) {
    let a = major.length();
    let u = major / a;
    if minor <= a {
        (center, u, a, minor, 0.0)
    } else {
        // P = c + a·cos t·u + b·sin t·w = c + b·cos(t − π/2)·w + a·sin(t − π/2)·(−u).
        (center, perp(u), minor, a, -std::f64::consts::FRAC_PI_2)
    }
}

/// The point at fraction `s ∈ [0, 1]` of the way along `c`, in its direction of travel.
pub(crate) fn point2(c: &Curve2d, s: f64) -> Point2 {
    match c {
        Curve2d::Line { a, b } => *a + (*b - *a) * s,
        Curve2d::Circle { center, radius } => {
            let t = std::f64::consts::TAU * s;
            *center + Vec2::new(t.cos(), t.sin()) * *radius
        }
        Curve2d::Arc {
            center,
            radius,
            start_angle,
            sweep,
        } => {
            let t = start_angle + sweep * s;
            *center + Vec2::new(t.cos(), t.sin()) * *radius
        }
        Curve2d::Ellipse {
            center,
            major,
            minor_radius,
            t0,
            t1,
        } => {
            let t = t0 + (t1 - t0) * s;
            let a = major.length();
            *center + *major * t.cos() + perp(*major) * (minor_radius / a) * t.sin()
        }
        Curve2d::BSpline(b) => {
            let (t0, t1) = bspline2_range(b);
            bspline2_point(b, t0 + (t1 - t0) * s)
        }
    }
}

/// How many chords sample `c` for the orientation and containment checks.
pub(crate) fn segments2(c: &Curve2d) -> usize {
    const PER_RAD: f64 = 64.0 / std::f64::consts::PI;
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "small positive counts"
    )]
    let by_angle = |angle: f64| ((angle.abs() * PER_RAD).ceil() as usize).clamp(4, 256);
    match c {
        Curve2d::Line { .. } => 1,
        Curve2d::Circle { .. } => by_angle(std::f64::consts::TAU),
        Curve2d::Arc { sweep, .. } => by_angle(*sweep),
        Curve2d::Ellipse { t0, t1, .. } => by_angle(t1 - t0),
        Curve2d::BSpline(b) => (8 * (b.poles.len() - usize::from(b.degree)) + 8).min(512),
    }
}

/// The point at parameter `t` of a 3D curve. `None` for `Other` and periodic B-splines, which
/// the builders reject before evaluating.
pub(crate) fn point3(c: &Curve3, t: f64) -> Option<Point3> {
    match c {
        Curve3::Line { origin, dir } => Some(*origin + dir.get() * t),
        Curve3::Circle { frame, radius } => {
            Some(frame.origin + (frame.x.get() * t.cos() + frame.y.get() * t.sin()) * *radius)
        }
        Curve3::Ellipse {
            frame,
            major_radius,
            minor_radius,
        } => Some(
            frame.origin
                + frame.x.get() * (major_radius * t.cos())
                + frame.y.get() * (minor_radius * t.sin()),
        ),
        Curve3::BSpline(b) if !b.periodic => Some(bspline3_point(b, t)),
        Curve3::BSpline(_) | Curve3::Other => None,
    }
}

fn bspline3_point(b: &BSplineCurveData, t: f64) -> Point3 {
    let knots = full_knots(&b.knots, &b.mults);
    let hom: Vec<[f64; 4]> = b
        .poles
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let w = b.weights.as_ref().map_or(1.0, |w| w[i]);
            [p.x * w, p.y * w, p.z * w, w]
        })
        .collect();
    let h = de_boor(usize::from(b.degree), &knots, &hom, t);
    Point3::new(h[0] / h[3], h[1] / h[3], h[2] / h[3])
}

/// The parameter range of a non-periodic 3D B-spline.
pub(crate) fn bspline3_range(b: &BSplineCurveData) -> (f64, f64) {
    let knots = full_knots(&b.knots, &b.mults);
    let p = usize::from(b.degree);
    (knots[p], knots[b.poles.len()])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close2(a: Point2, b: Point2) -> bool {
        (a - b).length() < 1e-12
    }

    #[test]
    fn rational_quadratic_is_a_quarter_circle() {
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let b = BSpline2dData {
            degree: 2,
            knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            poles: vec![
                Point2::new(1.0, 0.0),
                Point2::new(1.0, 1.0),
                Point2::new(0.0, 1.0),
            ],
            weights: Some(vec![1.0, w, 1.0]),
        };
        let c = Curve2d::BSpline(b);
        assert!(close2(point2(&c, 0.0), Point2::new(1.0, 0.0)));
        assert!(close2(point2(&c, 1.0), Point2::new(0.0, 1.0)));
        for i in 0..=10 {
            let p = point2(&c, f64::from(i) / 10.0);
            assert!((p.length() - 1.0).abs() < 1e-12, "{p:?}");
        }
    }

    #[test]
    fn knot_forms_round_trip() {
        let full = [0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0];
        let (k, m) = distinct_knots(&full);
        assert_eq!(m, [3, 1, 3]);
        assert_eq!(full_knots(&k, &m), full);
    }

    #[test]
    fn swapped_ellipse_axes_keep_the_points() {
        let center = Point2::new(1.0, 2.0);
        let major = Vec2::new(0.0, 2.0);
        let minor = 5.0;
        let c = Curve2d::Ellipse {
            center,
            major,
            minor_radius: minor,
            t0: 0.3,
            t1: 2.0,
        };
        let (o, u, a, b, shift) = ellipse_axes(center, major, minor);
        assert!(a >= b);
        for i in 0..=8 {
            let s = f64::from(i) / 8.0;
            let t = 0.3 + 1.7 * s + shift;
            let q = o + u * (a * t.cos()) + perp(u) * (b * t.sin());
            assert!(close2(point2(&c, s), q));
        }
    }
}
