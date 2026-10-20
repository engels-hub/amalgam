//! Intersection and closest-point queries between the primitives of [`crate::geom`].

use super::{Aabb3, Line3, Plane, Ray3, Segment2, Segment3, Triangle3};
use crate::math::{Dir3, Point2, Point3};
use crate::tolerance::Tolerance;

/// A ray hit: parameter `t >= 0` along the ray (mm) and the hit point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayHit {
    /// Distance along the ray, mm.
    pub t: f64,
    /// `ray.at(t)`.
    pub point: Point3,
}

/// A ray/triangle hit: `t >= 0` along the ray and barycentric weights of `(a, b, c)`
/// (non-negative, summing to 1).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TriHit {
    /// Distance along the ray, mm.
    pub t: f64,
    /// Weights of vertices `a`, `b`, `c`: hit point = `bary[0]·a + bary[1]·b + bary[2]·c`.
    pub bary: [f64; 3],
}

/// Result of [`segment2_segment2`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Seg2Intersection {
    /// The segments do not meet (within tolerance).
    None,
    /// The segments meet at one point: `p = a.a + t·(a.b - a.a) = b.a + u·(b.b - b.a)`,
    /// `t, u ∈ [0, 1]`.
    Point {
        /// Parameter on the first segment.
        t: f64,
        /// Parameter on the second segment.
        u: f64,
        /// The intersection point.
        p: Point2,
    },
    /// The segments are collinear and overlap over `[t0, t1]` (parameters on the first
    /// segment, `0 <= t0 < t1 <= 1`).
    Overlap {
        /// Start of the overlap on the first segment.
        t0: f64,
        /// End of the overlap on the first segment.
        t1: f64,
    },
}

/// Ray/plane hit. `None` if the ray is parallel to the plane within `tol.angular` or the
/// plane is behind the ray origin.
pub fn ray_plane(r: &Ray3, p: &Plane, tol: &Tolerance) -> Option<RayHit> {
    let n = p.normal.get();
    let denom = r.dir.get().dot(n);
    if denom.abs() <= tol.angular {
        return None;
    }
    let t = (p.origin - r.origin).dot(n) / denom;
    if t < 0.0 || !t.is_finite() {
        return None;
    }
    Some(RayHit { t, point: r.at(t) })
}

/// Slab test; returns `(t_enter, t_exit)` clipped to `t >= 0`, or `None` if the ray misses.
///
/// Zero direction components are handled explicitly (no `0 · inf` NaNs): such a ray hits
/// only if its origin lies within that slab. A ray starting inside the box gives
/// `t_enter = 0`. An empty box is never hit.
pub fn ray_aabb(r: &Ray3, b: &Aabb3) -> Option<(f64, f64)> {
    if b.is_empty() {
        return None;
    }
    let o = r.origin;
    let d = r.dir.get();
    let mut t_enter = 0.0_f64;
    let mut t_exit = f64::INFINITY;
    for i in 0..3 {
        if d[i] == 0.0 {
            if o[i] < b.min[i] || o[i] > b.max[i] {
                return None;
            }
            continue;
        }
        let inv = 1.0 / d[i];
        let t0 = (b.min[i] - o[i]) * inv;
        let t1 = (b.max[i] - o[i]) * inv;
        let (near, far) = if t0 <= t1 { (t0, t1) } else { (t1, t0) };
        t_enter = t_enter.max(near);
        t_exit = t_exit.min(far);
        if t_enter > t_exit {
            return None;
        }
    }
    Some((t_enter, t_exit))
}

/// Watertight ray/triangle intersection (Woop, Benthin, Wald 2013); both sides hit.
///
/// Rays through a shared edge or vertex of a consistent mesh hit at least one of the
/// adjacent triangles: the 2D edge functions of a shared edge are exact negations of each
/// other, and a zero edge function counts as inside. Degenerate triangles never hit.
pub fn ray_triangle(r: &Ray3, tri: &Triangle3) -> Option<TriHit> {
    let d = r.dir.get();
    // Axis of the largest |d| component becomes z; keep winding by swapping x/y if needed.
    let ad = d.abs();
    let kz = if ad.x >= ad.y && ad.x >= ad.z {
        0
    } else if ad.y >= ad.z {
        1
    } else {
        2
    };
    let mut kx = (kz + 1) % 3;
    let mut ky = (kx + 1) % 3;
    if d[kz] < 0.0 {
        std::mem::swap(&mut kx, &mut ky);
    }
    let sx = d[kx] / d[kz];
    let sy = d[ky] / d[kz];
    let sz = 1.0 / d[kz];

    let a = tri.a - r.origin;
    let b = tri.b - r.origin;
    let c = tri.c - r.origin;
    let ax = a[kx] - sx * a[kz];
    let ay = a[ky] - sy * a[kz];
    let bx = b[kx] - sx * b[kz];
    let by = b[ky] - sy * b[kz];
    let cx = c[kx] - sx * c[kz];
    let cy = c[ky] - sy * c[kz];

    let u = cx * by - cy * bx;
    let v = ax * cy - ay * cx;
    let w = bx * ay - by * ax;
    if (u < 0.0 || v < 0.0 || w < 0.0) && (u > 0.0 || v > 0.0 || w > 0.0) {
        return None;
    }
    let det = u + v + w;
    if det == 0.0 || !det.is_finite() {
        return None;
    }
    let az = sz * a[kz];
    let bz = sz * b[kz];
    let cz = sz * c[kz];
    let t = (u * az + v * bz + w * cz) / det;
    if t < 0.0 || !t.is_finite() {
        return None;
    }
    Some(TriHit {
        t,
        bary: [u / det, v / det, w / det],
    })
}

/// Intersection line of two planes; `None` if they are parallel within `tol.angular`.
///
/// The returned point is the point of the line closest to the world origin, and the
/// direction is `a.normal × b.normal` normalised.
pub fn plane_plane(a: &Plane, b: &Plane, tol: &Tolerance) -> Option<Line3> {
    let na = a.normal.get();
    let nb = b.normal.get();
    let dir = na.cross(nb);
    let len2 = dir.length_squared();
    if len2.sqrt() <= tol.angular {
        return None;
    }
    let da = na.dot(a.origin);
    let db = nb.dot(b.origin);
    // Ericson, Real-Time Collision Detection §5.4.4.
    let point = (nb * da - na * db).cross(dir) / len2;
    Some(Line3 {
        point,
        dir: Dir3::new_unchecked(dir / len2.sqrt()),
    })
}

/// Point where a line crosses a plane; `None` if parallel within `tol.angular`.
pub fn line_plane(l: &Line3, p: &Plane, tol: &Tolerance) -> Option<Point3> {
    let n = p.normal.get();
    let denom = l.dir.get().dot(n);
    if denom.abs() <= tol.angular {
        return None;
    }
    let t = (p.origin - l.point).dot(n) / denom;
    Some(l.point + l.dir.get() * t)
}

/// Closest points between segments (Ericson, *Real-Time Collision Detection* §5.1.9).
///
/// Returns `(s, t, pa, pb)` with `pa = a.a + s·(a.b - a.a)`, `pb = b.a + t·(b.b - b.a)` and
/// `s, t ∈ [0, 1]`. Zero-length segments are handled as points.
pub fn segment_segment(a: &Segment3, b: &Segment3) -> (f64, f64, Point3, Point3) {
    let d1 = a.b - a.a;
    let d2 = b.b - b.a;
    let r = a.a - b.a;
    let aa = d1.length_squared();
    let e = d2.length_squared();
    let f = d2.dot(r);
    let (s, t) = if aa <= 0.0 && e <= 0.0 {
        (0.0, 0.0)
    } else if aa <= 0.0 {
        (0.0, (f / e).clamp(0.0, 1.0))
    } else {
        let c = d1.dot(r);
        if e <= 0.0 {
            ((-c / aa).clamp(0.0, 1.0), 0.0)
        } else {
            let bb = d1.dot(d2);
            let denom = aa * e - bb * bb;
            // Near-parallel: any s works; pick 0 and let the clamps below fix t.
            let mut s = if denom > 1e-14 * aa * e {
                ((bb * f - c * e) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let mut t = (bb * s + f) / e;
            if t < 0.0 {
                t = 0.0;
                s = (-c / aa).clamp(0.0, 1.0);
            } else if t > 1.0 {
                t = 1.0;
                s = ((bb - c) / aa).clamp(0.0, 1.0);
            }
            (s, t)
        }
    };
    (s, t, a.a + d1 * s, b.a + d2 * t)
}

/// Closest points `(on a, on b)` of two infinite lines; `None` if parallel within
/// `tol.angular`.
///
/// Uses `n = da × db` directly (`s = ((w × db)·n) / |n|²`), which stays accurate for
/// nearly parallel lines where `1 - (da·db)²` would cancel to zero.
pub fn line_line(a: &Line3, b: &Line3, tol: &Tolerance) -> Option<(Point3, Point3)> {
    let da = a.dir.get();
    let db = b.dir.get();
    let n = da.cross(db);
    let n2 = n.length_squared();
    if n2.sqrt() <= tol.angular || n2 == 0.0 {
        return None;
    }
    let w = b.point - a.point;
    let s = w.cross(db).dot(n) / n2;
    let t = w.cross(da).dot(n) / n2;
    Some((a.point + da * s, b.point + db * t))
}

/// Distance from `p` to the closest point of segment `s`.
pub fn point_segment_distance(p: Point3, s: &Segment3) -> f64 {
    s.closest_point(p).1.distance(p)
}

/// Closest point of the (filled) triangle to `p` (Ericson, *Real-Time Collision Detection*
/// §5.1.5).
///
/// Degenerate and sliver triangles (`|ab × ac| <= 1e-8 · max_edge²`, i.e. height below about
/// 1e-8 of the longest edge) use the closest point of their edges instead: Ericson's region
/// tests are ill-conditioned there, and the edges are within the sliver's height of the
/// exact answer.
pub fn point_triangle_closest(p: Point3, tri: &Triangle3) -> Point3 {
    let (a, b, c) = (tri.a, tri.b, tri.c);
    let ab = b - a;
    let ac = c - a;
    let max_edge2 = ab
        .length_squared()
        .max(ac.length_squared())
        .max((c - b).length_squared());
    let n2 = ab.cross(ac).length_squared();
    if n2.is_nan() || n2 <= SLIVER * SLIVER * max_edge2 * max_edge2 {
        return closest_on_edges(p, a, b, c);
    }
    let ap = p - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return a + ab * v;
    }
    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return a + ac * w;
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + (c - b) * w;
    }
    let sum = va + vb + vc;
    if sum > 0.0 && sum.is_finite() {
        let v = vb / sum;
        let w = vc / sum;
        return a + ab * v + ac * w;
    }
    closest_on_edges(p, a, b, c)
}

/// Relative height below which [`point_triangle_closest`] treats a triangle as a sliver.
const SLIVER: f64 = 1e-8;

/// Closest point of the three edges of `(a, b, c)` to `p`.
fn closest_on_edges(p: Point3, a: Point3, b: Point3, c: Point3) -> Point3 {
    [
        Segment3 { a, b },
        Segment3 { a: b, b: c },
        Segment3 { a: c, b: a },
    ]
    .iter()
    .map(|s| s.closest_point(p).1)
    .min_by(|x, y| x.distance_squared(p).total_cmp(&y.distance_squared(p)))
    .unwrap_or(a)
}

/// Intersection of two 2D segments with tolerance.
///
/// Endpoints within `tol.linear` of the other segment count as touching. Segments whose
/// directions differ by at most `tol.angular` are treated as parallel; collinear parallel
/// segments give [`Seg2Intersection::Overlap`] (or `Point` if they only touch). Zero-length
/// segments are treated as points.
pub fn segment2_segment2(a: &Segment2, b: &Segment2, tol: &Tolerance) -> Seg2Intersection {
    let lin = tol.linear;
    let r = a.b - a.a;
    let s = b.b - b.a;
    let qp = b.a - a.a;
    let rl = r.length();
    let sl = s.length();

    if rl <= lin && sl <= lin {
        return if a.a.distance(b.a) <= lin {
            Seg2Intersection::Point {
                t: 0.0,
                u: 0.0,
                p: a.a,
            }
        } else {
            Seg2Intersection::None
        };
    }
    if rl <= lin {
        let u = ((a.a - b.a).dot(s) / s.length_squared()).clamp(0.0, 1.0);
        return if (b.a + s * u).distance(a.a) <= lin {
            Seg2Intersection::Point { t: 0.0, u, p: a.a }
        } else {
            Seg2Intersection::None
        };
    }
    if sl <= lin {
        let t = (qp.dot(r) / r.length_squared()).clamp(0.0, 1.0);
        return if (a.a + r * t).distance(b.a) <= lin {
            Seg2Intersection::Point { t, u: 0.0, p: b.a }
        } else {
            Seg2Intersection::None
        };
    }

    let denom = r.perp_dot(s);
    if denom.abs() <= tol.angular * rl * sl {
        // Parallel: collinear only if b.a is within tolerance of a's line.
        if qp.perp_dot(r).abs() > lin * rl {
            return Seg2Intersection::None;
        }
        let rr = r.length_squared();
        let t0 = qp.dot(r) / rr;
        let t1 = (b.b - a.a).dot(r) / rr;
        let lo = t0.min(t1).max(0.0);
        let hi = t0.max(t1).min(1.0);
        let eps = lin / rl;
        if hi < lo - eps {
            return Seg2Intersection::None;
        }
        if hi - lo <= eps {
            let t = (0.5 * (lo + hi)).clamp(0.0, 1.0);
            let p = a.a + r * t;
            let u = ((p - b.a).dot(s) / s.length_squared()).clamp(0.0, 1.0);
            return Seg2Intersection::Point { t, u, p };
        }
        return Seg2Intersection::Overlap { t0: lo, t1: hi };
    }

    let t = qp.perp_dot(s) / denom;
    let u = qp.perp_dot(r) / denom;
    let et = lin / rl;
    let eu = lin / sl;
    if t >= -et && t <= 1.0 + et && u >= -eu && u <= 1.0 + eu {
        let t = t.clamp(0.0, 1.0);
        let u = u.clamp(0.0, 1.0);
        return Seg2Intersection::Point {
            t,
            u,
            p: a.a + r * t,
        };
    }
    // The lines cross outside the segments, but at a shallow angle an endpoint can still be
    // within `lin` of the other segment: check all four, closest wins.
    let rr = r.length_squared();
    let ss = s.length_squared();
    let on_a = |q: Point2| (q - a.a).dot(r) / rr;
    let on_b = |q: Point2| (q - b.a).dot(s) / ss;
    let candidates = [
        (0.0, on_b(a.a).clamp(0.0, 1.0)),
        (1.0, on_b(a.b).clamp(0.0, 1.0)),
        (on_a(b.a).clamp(0.0, 1.0), 0.0),
        (on_a(b.b).clamp(0.0, 1.0), 1.0),
    ];
    candidates
        .into_iter()
        .map(|(t, u)| (t, u, (a.a + r * t).distance(b.a + s * u)))
        .filter(|&(_, _, d)| d <= lin)
        .min_by(|x, y| x.2.total_cmp(&y.2))
        .map_or(Seg2Intersection::None, |(t, u, _)| {
            Seg2Intersection::Point {
                t,
                u,
                p: a.a + r * t,
            }
        })
}
