//! Geometric primitives: bounding boxes, planes, rays, lines, segments and triangles.
//!
//! Intersection and distance queries live in [`intersect`].

use crate::error::{Error, Result};
use crate::math::{Dir3, Point2, Point3, Transform, Transform2, Vec2, Vec3};
use crate::tolerance::Tolerance;

pub mod intersect;

// ---- Aabb3 ------------------------------------------------------------------------------

/// Axis-aligned 3D bounding box, mm. Empty when any `min` component exceeds `max`.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Aabb3 {
    /// Minimum corner.
    pub min: Point3,
    /// Maximum corner.
    pub max: Point3,
}

impl Aabb3 {
    /// The empty box (`min = +inf`, `max = -inf`); the identity for [`Aabb3::union`].
    pub const EMPTY: Aabb3 = Aabb3 {
        min: Point3::INFINITY,
        max: Point3::NEG_INFINITY,
    };

    /// Box spanned by two corners (component-wise min/max, so any order works).
    pub fn new(a: Point3, b: Point3) -> Aabb3 {
        Aabb3 {
            min: a.min(b),
            max: a.max(b),
        }
    }

    /// Smallest box containing all points; [`Aabb3::EMPTY`] for no points.
    pub fn from_points<I: IntoIterator<Item = Point3>>(pts: I) -> Aabb3 {
        let mut b = Aabb3::EMPTY;
        for p in pts {
            b.include_point(p);
        }
        b
    }

    /// True if the box contains no point.
    pub fn is_empty(&self) -> bool {
        self.min.cmpgt(self.max).any()
    }

    /// Centre point. All components are NaN for an empty box.
    pub fn center(&self) -> Point3 {
        (self.min + self.max) * 0.5
    }

    /// Edge lengths; zero for an empty box.
    pub fn size(&self) -> Vec3 {
        if self.is_empty() {
            Vec3::ZERO
        } else {
            self.max - self.min
        }
    }

    /// Length of the diagonal; zero for an empty box.
    pub fn diagonal(&self) -> f64 {
        self.size().length()
    }

    /// Smallest box containing both boxes.
    pub fn union(&self, o: &Aabb3) -> Aabb3 {
        Aabb3 {
            min: self.min.min(o.min),
            max: self.max.max(o.max),
        }
    }

    /// Grows the box to contain `p`.
    pub fn include_point(&mut self, p: Point3) {
        self.min = self.min.min(p);
        self.max = self.max.max(p);
    }

    /// Box grown by `by` on every side (shrunk if negative). An empty box stays empty.
    pub fn expanded(&self, by: f64) -> Aabb3 {
        Aabb3 {
            min: self.min - Vec3::splat(by),
            max: self.max + Vec3::splat(by),
        }
    }

    /// True if `p` lies inside the box grown by `tol.linear`.
    pub fn contains(&self, p: Point3, tol: &Tolerance) -> bool {
        let t = Vec3::splat(tol.linear);
        p.cmpge(self.min - t).all() && p.cmple(self.max + t).all()
    }

    /// True if the boxes overlap or are at most `tol.linear` apart. False if either is empty.
    pub fn intersects(&self, o: &Aabb3, tol: &Tolerance) -> bool {
        if self.is_empty() || o.is_empty() {
            return false;
        }
        let t = Vec3::splat(tol.linear);
        self.min.cmple(o.max + t).all() && o.min.cmple(self.max + t).all()
    }

    /// Conservative bounds of the transformed box (Arvo's method). Empty stays empty.
    pub fn transformed(&self, t: &Transform) -> Aabb3 {
        if self.is_empty() {
            return Aabb3::EMPTY;
        }
        let m = t.matrix3;
        let mut lo = t.translation;
        let mut hi = t.translation;
        for j in 0..3 {
            let col = m.col(j);
            let a = col * self.min[j];
            let b = col * self.max[j];
            lo += a.min(b);
            hi += a.max(b);
        }
        Aabb3 { min: lo, max: hi }
    }
}

// ---- Aabb2 ------------------------------------------------------------------------------

/// Axis-aligned 2D bounding box, mm. Empty when any `min` component exceeds `max`.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Aabb2 {
    /// Minimum corner.
    pub min: Point2,
    /// Maximum corner.
    pub max: Point2,
}

impl Aabb2 {
    /// The empty box (`min = +inf`, `max = -inf`); the identity for [`Aabb2::union`].
    pub const EMPTY: Aabb2 = Aabb2 {
        min: Point2::INFINITY,
        max: Point2::NEG_INFINITY,
    };

    /// Box spanned by two corners (component-wise min/max).
    pub fn new(a: Point2, b: Point2) -> Aabb2 {
        Aabb2 {
            min: a.min(b),
            max: a.max(b),
        }
    }

    /// Smallest box containing all points; [`Aabb2::EMPTY`] for no points.
    pub fn from_points<I: IntoIterator<Item = Point2>>(pts: I) -> Aabb2 {
        let mut b = Aabb2::EMPTY;
        for p in pts {
            b.include_point(p);
        }
        b
    }

    /// True if the box contains no point.
    pub fn is_empty(&self) -> bool {
        self.min.cmpgt(self.max).any()
    }

    /// Centre point. All components are NaN for an empty box.
    pub fn center(&self) -> Point2 {
        (self.min + self.max) * 0.5
    }

    /// Edge lengths; zero for an empty box.
    pub fn size(&self) -> Vec2 {
        if self.is_empty() {
            Vec2::ZERO
        } else {
            self.max - self.min
        }
    }

    /// Length of the diagonal; zero for an empty box.
    pub fn diagonal(&self) -> f64 {
        self.size().length()
    }

    /// Smallest box containing both boxes.
    pub fn union(&self, o: &Aabb2) -> Aabb2 {
        Aabb2 {
            min: self.min.min(o.min),
            max: self.max.max(o.max),
        }
    }

    /// Grows the box to contain `p`.
    pub fn include_point(&mut self, p: Point2) {
        self.min = self.min.min(p);
        self.max = self.max.max(p);
    }

    /// Box grown by `by` on every side (shrunk if negative). An empty box stays empty.
    pub fn expanded(&self, by: f64) -> Aabb2 {
        Aabb2 {
            min: self.min - Vec2::splat(by),
            max: self.max + Vec2::splat(by),
        }
    }

    /// True if `p` lies inside the box grown by `tol.linear`.
    pub fn contains(&self, p: Point2, tol: &Tolerance) -> bool {
        let t = Vec2::splat(tol.linear);
        p.cmpge(self.min - t).all() && p.cmple(self.max + t).all()
    }

    /// True if the boxes overlap or are at most `tol.linear` apart. False if either is empty.
    pub fn intersects(&self, o: &Aabb2, tol: &Tolerance) -> bool {
        if self.is_empty() || o.is_empty() {
            return false;
        }
        let t = Vec2::splat(tol.linear);
        self.min.cmple(o.max + t).all() && o.min.cmple(self.max + t).all()
    }

    /// Conservative bounds of the transformed box (Arvo's method). Empty stays empty.
    pub fn transformed2(&self, t: &Transform2) -> Aabb2 {
        if self.is_empty() {
            return Aabb2::EMPTY;
        }
        let m = t.matrix2;
        let mut lo = t.translation;
        let mut hi = t.translation;
        for j in 0..2 {
            let col = m.col(j);
            let a = col * self.min[j];
            let b = col * self.max[j];
            lo += a.min(b);
            hi += a.max(b);
        }
        Aabb2 { min: lo, max: hi }
    }
}

// ---- Plane ------------------------------------------------------------------------------

/// Oriented plane: a right-handed frame (`x × y = normal`) with local `(u, v)` ↔ world
/// mapping `origin + u·x + v·y`.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Plane {
    /// Origin of the local frame, on the plane.
    pub origin: Point3,
    /// Local u axis.
    pub x: Dir3,
    /// Local v axis.
    pub y: Dir3,
    /// Unit normal, `x × y`.
    pub normal: Dir3,
}

impl Plane {
    /// World XY plane: u = +X, v = +Y, normal = +Z (top view).
    pub const XY: Plane = Plane {
        origin: Point3::ZERO,
        x: Dir3::X,
        y: Dir3::Y,
        normal: Dir3::Z,
    };
    /// World YZ plane: u = +Y, v = +Z, normal = +X (right view).
    pub const YZ: Plane = Plane {
        origin: Point3::ZERO,
        x: Dir3::Y,
        y: Dir3::Z,
        normal: Dir3::X,
    };
    /// World XZ plane: u = +X, v = +Z, normal = −Y (front view, right-handed).
    pub const XZ: Plane = Plane {
        origin: Point3::ZERO,
        x: Dir3::X,
        y: Dir3::Z,
        normal: Dir3::NEG_Y,
    };

    /// Plane through `origin` with `normal`; `x` is chosen deterministically from `normal`
    /// (same input → same frame).
    pub fn from_normal(origin: Point3, normal: Dir3) -> Plane {
        let x = normal.any_orthogonal();
        let y = Dir3::new_unchecked(normal.get().cross(x.get()));
        Plane {
            origin,
            x,
            y,
            normal,
        }
    }

    /// Plane through `origin` with `normal`, whose `x` is `x_hint` projected into the plane.
    ///
    /// `Err(InvalidArgument)` if `x_hint` is (nearly) zero or parallel to `normal` within
    /// `tol.angular`.
    pub fn from_normal_x(
        origin: Point3,
        normal: Dir3,
        x_hint: Vec3,
        tol: &Tolerance,
    ) -> Result<Plane> {
        let hint_len = x_hint.length();
        let n = normal.get();
        let proj = x_hint - n * n.dot(x_hint);
        let proj_len = proj.length();
        if hint_len.is_nan() || hint_len <= tol.linear || proj_len <= hint_len * tol.angular {
            return Err(Error::InvalidArgument {
                name: "x_hint",
                reason: "zero or parallel to the plane normal".to_owned(),
            });
        }
        let x = Dir3::new_unchecked(proj / proj_len);
        let y = Dir3::new_unchecked(n.cross(x.get()));
        Ok(Plane {
            origin,
            x,
            y,
            normal,
        })
    }

    /// Plane through three points: origin `a`, x towards `b`, normal `(b-a) × (c-a)`.
    ///
    /// `Err(Degenerate)` if `a` and `b` coincide or `c` is within `tol.linear` of line `ab`.
    pub fn from_points(a: Point3, b: Point3, c: Point3, tol: &Tolerance) -> Result<Plane> {
        let ab = b - a;
        let x = Dir3::new(ab, tol).ok_or(Error::Degenerate {
            what: "plane points coincide",
        })?;
        let n = ab.cross(c - a);
        let n_len = n.length();
        // n_len / |ab| is the distance from c to the line ab.
        if n_len.is_nan() || n_len <= tol.linear * ab.length() {
            return Err(Error::Degenerate {
                what: "plane points are collinear",
            });
        }
        let normal = Dir3::new_unchecked(n / n_len);
        let y = Dir3::new_unchecked(normal.get().cross(x.get()));
        Ok(Plane {
            origin: a,
            x,
            y,
            normal,
        })
    }

    /// Signed distance from the plane, positive on the `normal` side.
    pub fn signed_distance(&self, p: Point3) -> f64 {
        (p - self.origin).dot(self.normal.get())
    }

    /// Orthogonal projection of `p` onto the plane.
    pub fn project(&self, p: Point3) -> Point3 {
        p - self.normal.get() * self.signed_distance(p)
    }

    /// Local `(u, v)` of the projection of `p`.
    pub fn to_local(&self, p: Point3) -> Point2 {
        let d = p - self.origin;
        Point2::new(d.dot(self.x.get()), d.dot(self.y.get()))
    }

    /// World point of local `(u, v)`.
    pub fn to_world(&self, uv: Point2) -> Point3 {
        self.origin + self.x.get() * uv.x + self.y.get() * uv.y
    }

    /// Transform from local `(u, v, w)` (w along `normal`) to world.
    pub fn to_transform(&self) -> Transform {
        Transform::from_cols(self.x.get(), self.y.get(), self.normal.get(), self.origin)
    }

    /// The same plane facing the other way: `normal` and `y` negated, `x` kept (stays
    /// right-handed).
    pub fn flipped(&self) -> Plane {
        Plane {
            origin: self.origin,
            x: self.x,
            y: -self.y,
            normal: -self.normal,
        }
    }
}

// ---- Rays, lines, segments, triangles ---------------------------------------------------

/// Half-line `origin + t·dir`, `t >= 0`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ray3 {
    /// Start point.
    pub origin: Point3,
    /// Unit direction.
    pub dir: Dir3,
}

impl Ray3 {
    /// Point at parameter `t` (distance in mm, since `dir` is unit).
    pub fn at(&self, t: f64) -> Point3 {
        self.origin + self.dir.get() * t
    }
}

/// Infinite 3D line through `point` along `dir`.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Line3 {
    /// A point on the line.
    pub point: Point3,
    /// Unit direction.
    pub dir: Dir3,
}

/// 3D segment from `a` (t = 0) to `b` (t = 1).
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Segment3 {
    /// Start point.
    pub a: Point3,
    /// End point.
    pub b: Point3,
}

impl Segment3 {
    /// Distance from `a` to `b`.
    pub fn length(&self) -> f64 {
        self.a.distance(self.b)
    }

    /// Closest point to `p`: `(t, point)` with `t ∈ [0, 1]`. A zero-length segment gives
    /// `(0, a)`.
    pub fn closest_point(&self, p: Point3) -> (f64, Point3) {
        let ab = self.b - self.a;
        let len2 = ab.length_squared();
        if len2 <= 0.0 {
            return (0.0, self.a);
        }
        let t = ((p - self.a).dot(ab) / len2).clamp(0.0, 1.0);
        (t, self.a + ab * t)
    }
}

/// Infinite 2D line through `point` along `dir`.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Line2 {
    /// A point on the line.
    pub point: Point2,
    /// Direction; callers keep it unit length.
    pub dir: Vec2,
}

/// 2D segment from `a` (t = 0) to `b` (t = 1).
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Segment2 {
    /// Start point.
    pub a: Point2,
    /// End point.
    pub b: Point2,
}

/// Triangle with vertices `a`, `b`, `c` (counter-clockwise around the normal).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Triangle3 {
    /// First vertex.
    pub a: Point3,
    /// Second vertex.
    pub b: Point3,
    /// Third vertex.
    pub c: Point3,
}

impl Triangle3 {
    /// Unit normal `(b-a) × (c-a)` normalised; `None` if the triangle is degenerate (its
    /// height over the longest edge is at most `tol.linear`).
    pub fn normal(&self, tol: &Tolerance) -> Option<Dir3> {
        let n = (self.b - self.a).cross(self.c - self.a);
        let n_len = n.length();
        let longest = self
            .a
            .distance(self.b)
            .max(self.b.distance(self.c))
            .max(self.c.distance(self.a));
        if n_len.is_nan() || n_len <= tol.linear * longest {
            return None;
        }
        Some(Dir3::new_unchecked(n / n_len))
    }

    /// Area, mm².
    pub fn area(&self) -> f64 {
        0.5 * (self.b - self.a).cross(self.c - self.a).length()
    }
}

#[cfg(test)]
mod tests;
