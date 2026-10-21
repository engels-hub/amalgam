//! Exact-ish f64 triangle kernels: watertight ray test, closest point, solid angle.

use amg_core::{Point3, Vec3};

/// Which part of a triangle a closest point lies on. Vertex `i` is corner `i` of the face;
/// edge `i` runs from corner `i` to corner `(i + 1) % 3`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TriFeature {
    /// Corner 0, 1 or 2.
    Vertex(u8),
    /// Edge 0 (a-b), 1 (b-c) or 2 (c-a).
    Edge(u8),
    /// Interior of the face.
    Face,
}

/// Precomputed per-ray constants for the watertight test (Woop, Benthin & Wald 2013).
#[derive(Clone, Copy, Debug)]
pub(crate) struct RayPre {
    pub origin: Point3,
    kx: usize,
    ky: usize,
    kz: usize,
    sx: f64,
    sy: f64,
    sz: f64,
}

impl RayPre {
    pub fn new(origin: Point3, dir: Vec3) -> Self {
        let a = dir.abs();
        let kz = if a.x >= a.y && a.x >= a.z {
            0
        } else if a.y >= a.z {
            1
        } else {
            2
        };
        let mut kx = (kz + 1) % 3;
        let mut ky = (kx + 1) % 3;
        if dir[kz] < 0.0 {
            std::mem::swap(&mut kx, &mut ky);
        }
        let sz = 1.0 / dir[kz];
        Self {
            origin,
            kx,
            ky,
            kz,
            sx: dir[kx] * sz,
            sy: dir[ky] * sz,
            sz,
        }
    }

    /// Intersects the triangle; returns `(t, bary)` with `bary` the weights of `a, b, c`.
    /// Both faces are hit. Hits on a shared edge are reported for (at least) one of the two
    /// triangles; degenerate triangles are never hit.
    #[inline]
    pub fn hit(&self, a: Point3, b: Point3, c: Point3) -> Option<(f64, Vec3)> {
        let (kx, ky, kz) = (self.kx, self.ky, self.kz);
        let pa = a - self.origin;
        let pb = b - self.origin;
        let pc = c - self.origin;
        let ax = pa[kx] - self.sx * pa[kz];
        let ay = pa[ky] - self.sy * pa[kz];
        let bx = pb[kx] - self.sx * pb[kz];
        let by = pb[ky] - self.sy * pb[kz];
        let cx = pc[kx] - self.sx * pc[kz];
        let cy = pc[ky] - self.sy * pc[kz];
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
        let t = (u * self.sz * pa[kz] + v * self.sz * pb[kz] + w * self.sz * pc[kz]) / det;
        if !t.is_finite() || t < 0.0 {
            return None;
        }
        let inv = 1.0 / det;
        Some((t, Vec3::new(u * inv, v * inv, w * inv)))
    }
}

/// Closest point on triangle `abc` to `p` (Ericson, *Real-Time Collision Detection* §5.1.5).
/// Returns `(point, bary, feature)`. Degenerate triangles fall back to the closest of their
/// edges/vertices.
pub(crate) fn closest_on_tri(
    p: Point3,
    a: Point3,
    b: Point3,
    c: Point3,
) -> (Point3, Vec3, TriFeature) {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return (a, Vec3::X, TriFeature::Vertex(0));
    }
    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return (b, Vec3::Y, TriFeature::Vertex(1));
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let den = d1 - d3;
        let v = if den != 0.0 { d1 / den } else { 0.0 };
        return (a + ab * v, Vec3::new(1.0 - v, v, 0.0), TriFeature::Edge(0));
    }
    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return (c, Vec3::Z, TriFeature::Vertex(2));
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let den = d2 - d6;
        let w = if den != 0.0 { d2 / den } else { 0.0 };
        return (a + ac * w, Vec3::new(1.0 - w, 0.0, w), TriFeature::Edge(2));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let den = (d4 - d3) + (d5 - d6);
        let w = if den != 0.0 { (d4 - d3) / den } else { 0.0 };
        return (
            b + (c - b) * w,
            Vec3::new(0.0, 1.0 - w, w),
            TriFeature::Edge(1),
        );
    }
    let den = va + vb + vc;
    if den == 0.0 || !den.is_finite() {
        return (a, Vec3::X, TriFeature::Vertex(0));
    }
    let inv = 1.0 / den;
    let (v, w) = (vb * inv, vc * inv);
    (
        a + ab * v + ac * w,
        Vec3::new(1.0 - v - w, v, w),
        TriFeature::Face,
    )
}

/// Signed solid angle of `abc` seen from `q` divided by 4π (Van Oosterom & Strackee 1983).
/// Positive when `q` is on the inner side of a counter-clockwise (outward) triangle.
#[inline]
pub(crate) fn solid_angle_over_4pi(q: Point3, a: Point3, b: Point3, c: Point3) -> f64 {
    let (a, b, c) = (a - q, b - q, c - q);
    let (la, lb, lc) = (a.length(), b.length(), c.length());
    let num = a.dot(b.cross(c));
    let den = la * lb * lc + a.dot(b) * lc + a.dot(c) * lb + b.dot(c) * la;
    2.0 * num.atan2(den) / (4.0 * std::f64::consts::PI)
}
