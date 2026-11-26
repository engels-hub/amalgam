//! Closed forms over [`RefGeom`] (construction, origin and sketch geometry; S05 §2 "Phase 2",
//! INS-07) and the polylines that stand in for curves without one.
//!
//! Planes and lines are unbounded. Point distances are closed forms for every analytic kind
//! (free-form curves and surfaces through their evaluators); between two non-points only the
//! pairs of lines, segments, planes and circle–plane have closed forms here.

use std::f64::consts::{PI, TAU};

use amg_core::{Plane, Point3, Tolerance, Vec3};
use amg_jobs::JobCtx;

use crate::construct::curves::CircleCurve;
use crate::construct::{CurveEval, RefGeom};
use crate::inspect::InspectError;

/// Squared sine below which two directions are parallel.
const PARALLEL_SQ: f64 = 1e-24;
/// Largest number of segments of a curve polyline.
const MAX_SEGMENTS: usize = 1 << 16;
/// Equal parameter intervals a free-form curve starts from.
const CURVE_PIECES: usize = 64;
/// Largest halvings of one starting interval.
const MAX_DEPTH: u32 = 14;

/// A closest pair: distance, point on the first, point on the second (world).
pub(super) type Pair = (f64, Point3, Point3);

fn pair(a: Point3, b: Point3) -> Pair {
    (a.distance(b), a, b)
}

/// `(q, p)` swapped.
pub(super) fn swapped((d, a, b): Pair) -> Pair {
    (d, b, a)
}

/// The unit vector along `v`, or a unit vector perpendicular to `axis` if `v` vanishes.
fn radial(v: Vec3, axis: Vec3) -> Vec3 {
    let len = v.length();
    if len > 1e-12 {
        v / len
    } else {
        axis.any_orthonormal_vector()
    }
}

/// The point of `g` closest to `p`.
///
/// # Errors
/// [`InspectError::Degenerate`] when a free-form query or an arc fails,
/// [`InspectError::Unsupported`] for geometry kinds added later.
pub(super) fn closest_to_point(g: &RefGeom, p: Point3) -> Result<Point3, InspectError> {
    Ok(match g {
        RefGeom::Point(q) => *q,
        RefGeom::Line(l) => l.point + l.dir.get() * (p - l.point).dot(l.dir.get()),
        RefGeom::Segment(s) => {
            let d = s.b - s.a;
            let ll = d.length_squared();
            let t = if ll > 0.0 {
                ((p - s.a).dot(d) / ll).clamp(0.0, 1.0)
            } else {
                0.0
            };
            s.a + d * t
        }
        RefGeom::Plane(plane) => plane.project(p),
        RefGeom::Circle {
            center,
            normal,
            radius,
            arc,
        } => {
            let c = circle_curve(*center, *normal, *radius, *arc)?;
            c.eval(c.closest_param(p))
        }
        RefGeom::Cylinder { axis, radius } => {
            let n = axis.dir.get();
            let foot = axis.point + n * (p - axis.point).dot(n);
            foot + radial(p - foot, n) * *radius
        }
        RefGeom::Cone {
            apex,
            axis,
            half_angle,
        } => {
            // In (axial, radial) coordinates the generator is the ray (cos α, sin α).
            let n = axis.get();
            let h = (p - *apex).dot(n);
            let off = p - *apex - n * h;
            let (sin, cos) = half_angle.sin_cos();
            let t = (h * cos + off.length() * sin).max(0.0);
            *apex + n * (t * cos) + radial(off, n) * (t * sin)
        }
        RefGeom::Sphere { center, radius } => *center + radial(p - *center, Vec3::Z) * *radius,
        RefGeom::Torus { axis, major, minor } => {
            let n = axis.dir.get();
            let off = p - axis.point - n * (p - axis.point).dot(n);
            let ring = axis.point + radial(off, n) * *major;
            ring + radial(p - ring, n) * *minor
        }
        RefGeom::Curve(c) => c.eval(c.closest_param(p)),
        RefGeom::Surface(s) => {
            let uv = s
                .closest_uv(p)
                .ok_or(InspectError::Degenerate("no closest point on the surface"))?;
            let q = s.eval(uv);
            if !q.is_finite() {
                return Err(InspectError::Degenerate("no closest point on the surface"));
            }
            q
        }
    })
}

fn circle_curve(
    center: Point3,
    normal: amg_core::Dir3,
    radius: f64,
    arc: Option<(f64, f64)>,
) -> Result<CircleCurve, InspectError> {
    CircleCurve::new(center, normal, radius, arc)
        .map_err(|_| InspectError::Degenerate("the circle is empty"))
}

/// A line or segment as `origin + t·dir`, `t` in `range`.
#[derive(Clone, Copy)]
struct Linear {
    origin: Point3,
    dir: Vec3,
    range: (f64, f64),
}

impl Linear {
    fn of(g: &RefGeom) -> Option<Linear> {
        match g {
            RefGeom::Line(l) => Some(Linear {
                origin: l.point,
                dir: l.dir.get(),
                range: (f64::NEG_INFINITY, f64::INFINITY),
            }),
            RefGeom::Segment(s) => Some(Linear {
                origin: s.a,
                dir: s.b - s.a,
                range: (0.0, 1.0),
            }),
            _ => None,
        }
    }

    fn at(self, t: f64) -> Point3 {
        self.origin + self.dir * t
    }

    fn clamp(self, t: f64) -> f64 {
        t.clamp(self.range.0, self.range.1)
    }
}

/// Closest points of two lines or segments (Ericson 2005 §5.1.9, with unbounded ranges).
fn linear_linear(a: Linear, b: Linear) -> Pair {
    let r = a.origin - b.origin;
    let (aa, bb) = (a.dir.length_squared(), b.dir.length_squared());
    let (c, f) = (a.dir.dot(r), b.dir.dot(r));
    if aa <= 0.0 {
        let t = if bb > 0.0 { b.clamp(f / bb) } else { 0.0 };
        return pair(a.origin, b.at(t));
    }
    if bb <= 0.0 {
        return pair(a.at(a.clamp(-c / aa)), b.origin);
    }
    let ab = a.dir.dot(b.dir);
    // |d₁ × d₂|², which keeps its digits where `aa·bb − ab²` cancels.
    let denom = a.dir.cross(b.dir).length_squared();
    let mut s = if denom > PARALLEL_SQ * aa * bb {
        a.clamp((ab * f - c * bb) / denom)
    } else {
        a.clamp(0.0)
    };
    let mut t = (ab * s + f) / bb;
    if t < b.range.0 || t > b.range.1 {
        t = b.clamp(t);
        s = a.clamp((ab * t - c) / aa);
    }
    pair(a.at(s), b.at(t))
}

/// Closest points of a line or segment and a plane: `(distance, on the line, on the plane)`.
fn linear_plane(a: Linear, p: &Plane) -> Pair {
    let n = p.normal.get();
    let along = a.dir.dot(n);
    let at = |t: f64| {
        let q = a.at(t);
        pair(q, p.project(q))
    };
    if along * along <= PARALLEL_SQ * a.dir.length_squared() {
        return at(a.clamp(0.0));
    }
    let hit = -p.signed_distance(a.origin) / along;
    if (a.range.0..=a.range.1).contains(&hit) {
        let q = a.at(hit);
        (0.0, q, q)
    } else {
        at(a.clamp(hit))
    }
}

fn plane_plane(a: &Plane, b: &Plane) -> Pair {
    let (na, nb) = (a.normal.get(), b.normal.get());
    let line = na.cross(nb);
    if line.length_squared() <= PARALLEL_SQ {
        return pair(a.origin, b.project(a.origin));
    }
    // Walk in `a` from its origin, perpendicular to the intersection line, until on `b`.
    let m = na.cross(line).normalize();
    let q = a.origin - m * (b.signed_distance(a.origin) / m.dot(nb));
    (0.0, q, q)
}

/// A circle or arc against a plane, in closed form: the signed distance along the circle is
/// `s₀ + R·cos(t − φ)`.
fn circle_plane(c: &CircleCurve, p: &Plane) -> Pair {
    let (start, end) = c.range();
    let centre = c.eval(start) + c.d2(start);
    let s0 = p.signed_distance(centre);
    // d2(t) = −radius·radial(t).
    let n = p.normal.get();
    let (u, v) = (-c.d2(0.0).dot(n), -c.d2(PI / 2.0).dot(n));
    let (amplitude, phase) = (u.hypot(v), v.atan2(u));
    let mut candidates = vec![start, end];
    let mut add = |t: f64| {
        let t = start + (t - start).rem_euclid(TAU);
        if t <= end {
            candidates.push(t);
        }
    };
    add(phase);
    add(phase + PI);
    let crossing = amplitude > 0.0 && s0.abs() <= amplitude;
    if crossing {
        let half = (-s0 / amplitude).clamp(-1.0, 1.0).acos();
        add(phase + half);
        add(phase - half);
    }
    let mut best = (f64::INFINITY, centre, centre);
    for t in candidates {
        let q = c.eval(t);
        let found = pair(q, p.project(q));
        if found.0 < best.0 {
            best = found;
        }
    }
    best
}

/// The closed form between two geometries, `None` if there is none here (curves, circles
/// against lines or circles, surfaces).
///
/// # Errors
/// As [`closest_to_point`].
pub(super) fn exact_pair(a: &RefGeom, b: &RefGeom) -> Result<Option<Pair>, InspectError> {
    if let RefGeom::Point(p) = a {
        return Ok(Some(pair(*p, closest_to_point(b, *p)?)));
    }
    if let RefGeom::Point(p) = b {
        return Ok(Some(pair(closest_to_point(a, *p)?, *p)));
    }
    if let (Some(x), Some(y)) = (Linear::of(a), Linear::of(b)) {
        return Ok(Some(linear_linear(x, y)));
    }
    Ok(match (a, b) {
        (RefGeom::Plane(x), RefGeom::Plane(y)) => Some(plane_plane(x, y)),
        (RefGeom::Plane(p), other) => exact_pair(other, &RefGeom::Plane(*p))?.map(swapped),
        (other, RefGeom::Plane(p)) => match (Linear::of(other), other) {
            (Some(l), _) => Some(linear_plane(l, p)),
            (
                None,
                RefGeom::Circle {
                    center,
                    normal,
                    radius,
                    arc,
                },
            ) => Some(circle_plane(
                &circle_curve(*center, *normal, *radius, *arc)?,
                p,
            )),
            _ => None,
        },
        _ => None,
    })
}

/// The points of a circle, arc or free-form curve, no farther than `deflection` from it.
/// `None` for other geometry.
///
/// # Errors
/// [`InspectError::Degenerate`], [`InspectError::Cancelled`].
pub(super) fn polyline(
    g: &RefGeom,
    deflection: f64,
    ctx: &JobCtx,
) -> Result<Option<Vec<Point3>>, InspectError> {
    ctx.checkpoint()?;
    match g {
        RefGeom::Circle {
            center,
            normal,
            radius,
            arc,
        } => {
            let c = circle_curve(*center, *normal, *radius, *arc)?;
            let (start, end) = c.range();
            // A chord spanning θ has the sagitta r·(1 − cos θ/2).
            let step = 2.0 * (1.0 - (deflection / radius).min(1.0)).acos();
            let n = ((end - start) / step.max(Tolerance::DEFAULT.angular))
                .ceil()
                .clamp(8.0, MAX_SEGMENTS as f64) as usize;
            Ok(Some(
                (0..=n)
                    .map(|i| c.eval(start + (end - start) * (i as f64 / n as f64)))
                    .collect(),
            ))
        }
        RefGeom::Curve(c) => {
            let (start, end) = c.range();
            let at = |i: usize| start + (end - start) * (i as f64 / CURVE_PIECES as f64);
            let mut points = vec![c.eval(start)];
            for i in 0..CURVE_PIECES {
                ctx.checkpoint()?;
                subdivide(&**c, (at(i), at(i + 1)), deflection, 0, &mut points);
            }
            if points.iter().any(|p| !p.is_finite()) {
                return Err(InspectError::Degenerate("the curve has no finite points"));
            }
            Ok(Some(points))
        }
        _ => Ok(None),
    }
}

/// Appends the points of `(t0, t1]`, halving until the quarter points lie within `deflection`
/// of the chord.
fn subdivide(
    c: &dyn CurveEval,
    (t0, t1): (f64, f64),
    deflection: f64,
    depth: u32,
    out: &mut Vec<Point3>,
) {
    let chord = amg_core::Segment3 {
        a: c.eval(t0),
        b: c.eval(t1),
    };
    let flat = [0.25, 0.5, 0.75].iter().all(|f| {
        let p = c.eval(t0 + (t1 - t0) * f);
        amg_core::intersect::point_segment_distance(p, &chord) <= deflection
    });
    if flat || depth >= MAX_DEPTH || out.len() >= MAX_SEGMENTS {
        out.push(chord.b);
    } else {
        let mid = 0.5 * (t0 + t1);
        subdivide(c, (t0, mid), deflection, depth + 1, out);
        subdivide(c, (mid, t1), deflection, depth + 1, out);
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, reason = "test code")]

    use amg_core::{Dir3, Line3, Segment3};

    use super::*;

    fn seg(a: [f64; 3], b: [f64; 3]) -> RefGeom {
        RefGeom::Segment(Segment3 {
            a: a.into(),
            b: b.into(),
        })
    }

    fn line(p: [f64; 3], d: Dir3) -> RefGeom {
        RefGeom::Line(Line3 {
            point: p.into(),
            dir: d,
        })
    }

    #[track_caller]
    fn distance(a: &RefGeom, b: &RefGeom, expected: f64) {
        for (x, y) in [(a, b), (b, a)] {
            let (d, p, q) = exact_pair(x, y).unwrap().unwrap();
            assert!((d - expected).abs() < 1e-12, "{d} != {expected}");
            assert!((p.distance(q) - d).abs() < 1e-12, "witnesses span it");
        }
    }

    #[test]
    fn lines_segments_and_planes() {
        let x = line([0.0, 0.0, 0.0], Dir3::X);
        let y3 = line([5.0, 0.0, 3.0], Dir3::Y);
        distance(&x, &y3, 3.0);
        distance(&x, &line([7.0, 4.0, 0.0], Dir3::X), 4.0);
        // Segments: the ends clamp.
        distance(
            &seg([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
            &seg([4.0, 4.0, 0.0], [4.0, 9.0, 0.0]),
            5.0,
        );
        distance(
            &seg([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
            &seg([3.0, 0.0, 0.0], [5.0, 0.0, 0.0]),
            2.0,
        );
        distance(&seg([2.0, 2.0, 2.0], [2.0, 2.0, 2.0]), &x, 8.0_f64.sqrt());
        let xy = RefGeom::Plane(Plane::XY);
        distance(&seg([0.0, 0.0, 2.0], [1.0, 1.0, 7.0]), &xy, 2.0);
        distance(&seg([0.0, 0.0, -2.0], [1.0, 1.0, 7.0]), &xy, 0.0);
        distance(&line([1.0, 1.0, 6.0], Dir3::X), &xy, 6.0);
        distance(&line([1.0, 1.0, 6.0], Dir3::Z), &xy, 0.0);
        let mut z4 = Plane::XY;
        z4.origin.z = 4.0;
        distance(&xy, &RefGeom::Plane(z4.flipped()), 4.0);
        distance(&xy, &RefGeom::Plane(Plane::XZ), 0.0);
    }

    #[test]
    fn points_against_every_analytic_kind() {
        let p = RefGeom::Point(Point3::new(10.0, 0.0, 4.0));
        let z = Line3 {
            point: Point3::ZERO,
            dir: Dir3::Z,
        };
        distance(&p, &line([0.0, 0.0, 0.0], Dir3::Z), 10.0);
        distance(&p, &RefGeom::Plane(Plane::XY), 4.0);
        distance(
            &p,
            &RefGeom::Cylinder {
                axis: z,
                radius: 3.0,
            },
            7.0,
        );
        distance(
            &p,
            &RefGeom::Sphere {
                center: Point3::new(10.0, 0.0, 29.0),
                radius: 10.0,
            },
            15.0,
        );
        distance(
            &p,
            &RefGeom::Torus {
                axis: z,
                major: 7.0,
                minor: 2.0,
            },
            3.0,
        );
        // A 45° cone: (10, 4) is 3·√2 from the generator ρ = h.
        distance(
            &p,
            &RefGeom::Cone {
                apex: Point3::ZERO,
                axis: Dir3::Z,
                half_angle: std::f64::consts::FRAC_PI_4,
            },
            3.0 * 2.0_f64.sqrt(),
        );
        let circle = RefGeom::Circle {
            center: Point3::ZERO,
            normal: Dir3::Z,
            radius: 7.0,
            arc: None,
        };
        distance(&p, &circle, 5.0);
    }

    #[test]
    fn circles_and_arcs_against_planes() {
        // A circle in the XZ plane around (0, 0, 10), radius 4.
        let circle = |arc| RefGeom::Circle {
            center: Point3::new(0.0, 0.0, 10.0),
            normal: Dir3::Y,
            radius: 4.0,
            arc,
        };
        distance(&circle(None), &RefGeom::Plane(Plane::XY), 6.0);
        let mut z12 = Plane::XY;
        z12.origin.z = 12.0;
        distance(&circle(None), &RefGeom::Plane(z12), 0.0);
        // The upper half only (z ≥ 10): its ends are the closest points.
        let top = CircleCurve::new(Point3::new(0.0, 0.0, 10.0), Dir3::Y, 4.0, None)
            .unwrap()
            .closest_param(Point3::new(0.0, 0.0, 100.0));
        distance(
            &circle(Some((top - PI / 2.0, top + PI / 2.0))),
            &RefGeom::Plane(Plane::XY),
            10.0,
        );
    }

    #[test]
    fn polylines_stay_within_the_deflection() {
        let g = RefGeom::Circle {
            center: Point3::ZERO,
            normal: Dir3::Z,
            radius: 10.0,
            arc: None,
        };
        let pts = polyline(&g, 1e-3, &JobCtx::detached()).unwrap().unwrap();
        assert!(pts.first().unwrap().distance(*pts.last().unwrap()) < 1e-12);
        for w in pts.windows(2) {
            let sagitta = 10.0 - ((w[0] + w[1]) * 0.5).length();
            assert!((0.0..=1e-3 + 1e-12).contains(&sagitta), "{sagitta}");
        }
        assert!(
            polyline(&RefGeom::Point(Point3::ZERO), 1e-3, &JobCtx::detached())
                .unwrap()
                .is_none()
        );
    }
}
