//! Pure construction math (note `docs/notes/inspection-geometry.md` §6). All results are in
//! world space, f64, and deterministic: the same input gives a bit-identical frame.
//!
//! CON-01 holds the plane constructions and the input helpers they share; CON-02 adds the
//! axis and point constructions.

use std::f64::consts::TAU;

use amg_core::{Dir3, Line3, Plane, Point3, Tolerance, Transform, Vec3, intersect};

use super::{ConstructError, CurveEval, OriginEntity, RefGeom};

/// Curvature below which a path counts as straight (`plane_along_path`), 1/mm.
const STRAIGHT_CURVATURE: f64 = 1e-12;

/// Interior samples used to decide whether a free-form curve is straight (`line_of`).
const STRAIGHT_SAMPLES: u32 = 8;

/// Equal parameter intervals scanned for sign changes when intersecting a free-form curve
/// with a plane (`point_curve_plane`).
const CURVE_PLANE_SAMPLES: u32 = 64;

/// Bisection steps after a sign change is found (`point_curve_plane`); the loop also stops
/// once the bracket can no longer shrink in f64.
const CURVE_PLANE_BISECTIONS: u32 = 200;

fn wrong(expected: &'static str, g: &RefGeom) -> ConstructError {
    ConstructError::WrongInput {
        expected,
        got: g.kind_name(),
    }
}

/// Unit vector of `v`, or `err` if it is (nearly) zero.
fn unit(v: Vec3, tol: &Tolerance, err: ConstructError) -> Result<Dir3, ConstructError> {
    Dir3::new(v, tol).ok_or(err)
}

/// `u` rotated by `angle` about `axis` (right-hand rule), for `u ⟂ axis` (Rodrigues).
fn rotate_perp(u: Vec3, axis: Vec3, angle: f64) -> Vec3 {
    u * angle.cos() + axis.cross(u) * angle.sin()
}

/// The plane through `origin` with in-plane axis `x` and `normal` (`x ⟂ normal`, both unit).
fn frame(origin: Point3, x: Dir3, normal: Dir3) -> Plane {
    let y = Dir3::new_unchecked(normal.get().cross(x.get()));
    Plane {
        origin,
        x,
        y,
        normal,
    }
}

/// Re-normalised image of a unit direction under a rigid transform (falls back to `d` for a
/// degenerate transform).
fn map_dir(t: &Transform, d: Dir3) -> Dir3 {
    Dir3::new(t.transform_vector3(d.get()), &Tolerance::DEFAULT).unwrap_or(d)
}

/// World geometry of an origin entity of a component placed by `component_to_world`
/// (rigid).
pub fn origin_geom(e: OriginEntity, component_to_world: &Transform) -> RefGeom {
    let t = component_to_world;
    let o = t.transform_point3(Point3::ZERO);
    let axis = |d: Dir3| {
        RefGeom::Line(Line3 {
            point: o,
            dir: map_dir(t, d),
        })
    };
    let plane = |p: Plane| {
        let x = map_dir(t, p.x);
        let y_img = map_dir(t, p.y).get();
        // Keep the frame orthonormal and right-handed under rounding.
        let normal = Dir3::new(x.get().cross(y_img), &Tolerance::DEFAULT).unwrap_or(p.normal);
        RefGeom::Plane(frame(o, x, normal))
    };
    match e {
        OriginEntity::Point => RefGeom::Point(o),
        OriginEntity::XAxis => axis(Dir3::X),
        OriginEntity::YAxis => axis(Dir3::Y),
        OriginEntity::ZAxis => axis(Dir3::Z),
        OriginEntity::XYPlane => plane(Plane::XY),
        OriginEntity::YZPlane => plane(Plane::YZ),
        OriginEntity::XZPlane => plane(Plane::XZ),
    }
}

/// `base` moved by `distance` along its normal, same frame. `distance` may be negative.
pub fn plane_offset(base: &Plane, distance: f64) -> Plane {
    Plane {
        origin: base.origin + base.normal.get() * distance,
        ..*base
    }
}

/// The plane containing `line`, rotated by `angle` about `line.dir` (right-hand rule) from the
/// zero plane.
///
/// The zero plane is `zero` (the caller passes the line's `context_plane` when the user picked
/// none). Without one, it is the plane containing the line and the first of world Z, Y, X that
/// is not parallel to it, with normal `line.dir × w`. The result's origin is `line.point` and
/// its x axis `line.dir`.
///
/// `Parallel` if the zero plane does not contain the line.
pub fn plane_at_angle(
    line: &Line3,
    zero: Option<&Plane>,
    angle: f64,
    tol: &Tolerance,
) -> Result<Plane, ConstructError> {
    let d = line.dir.get();
    let n0 = match zero {
        Some(z) => {
            let n = z.normal.get();
            if d.dot(n).abs() > tol.angular || z.signed_distance(line.point).abs() > tol.linear {
                return Err(ConstructError::Parallel);
            }
            // Exactly perpendicular to the line (removes the tolerated tilt).
            unit(n - d * d.dot(n), tol, ConstructError::Parallel)?
        }
        None => {
            let w = [Dir3::Z, Dir3::Y, Dir3::X]
                .into_iter()
                .find(|w| !tol.is_parallel(line.dir, *w))
                .ok_or(ConstructError::Degenerate("line direction"))?;
            unit(d.cross(w.get()), tol, ConstructError::Parallel)?
        }
    };
    let n = Dir3::new_unchecked(rotate_perp(n0.get(), d, angle).normalize());
    Ok(frame(line.point, line.dir, n))
}

/// `u₀`: `reference`'s normal projected perpendicular to `axis`; `Parallel` if the normal is
/// parallel to the axis.
fn reference_dir(reference: &Plane, axis: Dir3, tol: &Tolerance) -> Result<Vec3, ConstructError> {
    let a = axis.get();
    let n = reference.normal.get();
    let u = n - a * a.dot(n);
    if u.length() <= tol.angular {
        return Err(ConstructError::Parallel);
    }
    Ok(u.normalize())
}

/// A plane tangent to a cylinder, cone or sphere face (note §6).
///
/// - Cylinder: the contact direction is `u₀` (the reference normal projected ⟂ to the axis)
///   rotated by `angle` about the axis; the normal is that direction and the plane passes
///   through `axis.point + r·dir`. x = the axis direction.
/// - Cone: the same direction; the plane contains the generator along it. Its origin is the
///   generator point at the axial height of `reference.origin` (the apex if that height is not
///   positive). x = the generator direction (away from the apex).
/// - Sphere: the contact direction is `reference.normal` rotated by `angle` about
///   `reference.x`; x = `reference.x`.
///
/// `Parallel` if the reference normal is parallel to a cylinder/cone axis, `Unsupported` for
/// other surfaces, `WrongInput` for non-surfaces.
pub fn plane_tangent(
    surface: &RefGeom,
    reference: &Plane,
    angle: f64,
    tol: &Tolerance,
) -> Result<Plane, ConstructError> {
    match surface {
        RefGeom::Cylinder { axis, radius } => {
            check_radius(*radius)?;
            let a = axis.dir.get();
            let dir = rotate_perp(reference_dir(reference, axis.dir, tol)?, a, angle).normalize();
            let n = Dir3::new_unchecked(dir);
            Ok(frame(axis.point + dir * *radius, axis.dir, n))
        }
        RefGeom::Cone {
            apex,
            axis,
            half_angle,
        } => {
            check_half_angle(*half_angle)?;
            let a = axis.get();
            let dir = rotate_perp(reference_dir(reference, *axis, tol)?, a, angle).normalize();
            let (s, c) = half_angle.sin_cos();
            let g = Dir3::new_unchecked((a * c + dir * s).normalize());
            let n = Dir3::new_unchecked((dir * c - a * s).normalize());
            let h = (reference.origin - *apex).dot(a);
            let origin = if h > tol.linear {
                *apex + g.get() * (h / c)
            } else {
                *apex
            };
            Ok(frame(origin, g, n))
        }
        RefGeom::Sphere { center, radius } => {
            check_radius(*radius)?;
            let x = reference.x;
            let dir = rotate_perp(reference.normal.get(), x.get(), angle).normalize();
            Ok(frame(*center + dir * *radius, x, Dir3::new_unchecked(dir)))
        }
        RefGeom::Plane(_) | RefGeom::Torus { .. } | RefGeom::Surface(_) => {
            Err(ConstructError::Unsupported)
        }
        other => Err(wrong("cylinder, cone or sphere face", other)),
    }
}

fn check_radius(r: f64) -> Result<(), ConstructError> {
    if r.is_finite() && r > 0.0 {
        Ok(())
    } else {
        Err(ConstructError::Degenerate("radius is not positive"))
    }
}

fn check_half_angle(a: f64) -> Result<(), ConstructError> {
    if a.is_finite() && a > 0.0 && a < std::f64::consts::FRAC_PI_2 {
        Ok(())
    } else {
        Err(ConstructError::Degenerate(
            "cone half angle outside (0, π/2)",
        ))
    }
}

/// The plane midway between two planes or two points (note §6).
///
/// - Parallel planes (normals within `angular`, same or opposite): halfway, with the first
///   plane's frame.
/// - Intersecting planes: the bisector containing the intersection line, normal `n₁ − n₂`
///   (normalised) if the planes face each other (`n₁·n₂ < 0`), else `n₁ + n₂`. Origin = the
///   point of the intersection line closest to the world origin, x = its direction.
/// - Two points: normal along `b − a`, through the midpoint, x from `Plane::from_normal`.
///
/// `Coincident` if the inputs are the same plane or point.
pub fn midplane(a: &RefGeom, b: &RefGeom, tol: &Tolerance) -> Result<Plane, ConstructError> {
    match (a, b) {
        (RefGeom::Plane(p), RefGeom::Plane(q)) => {
            let n1 = p.normal.get();
            if tol.is_parallel(p.normal, q.normal) {
                let d = (q.origin - p.origin).dot(n1);
                if d.abs() <= tol.linear {
                    return Err(ConstructError::Coincident);
                }
                return Ok(plane_offset(p, 0.5 * d));
            }
            let line = intersect::plane_plane(p, q, tol).ok_or(ConstructError::Parallel)?;
            let n2 = q.normal.get();
            let n = if n1.dot(n2) < 0.0 { n1 - n2 } else { n1 + n2 };
            let n = unit(n, tol, ConstructError::Degenerate("bisector normal"))?;
            Ok(frame(line.point, line.dir, n))
        }
        (RefGeom::Point(p), RefGeom::Point(q)) => {
            // `Dir3::new` refuses |b − a| <= linear.
            let n = unit(*q - *p, tol, ConstructError::Coincident)?;
            Ok(Plane::from_normal((*p + *q) * 0.5, n))
        }
        (RefGeom::Plane(_), other) => Err(wrong("plane", other)),
        (RefGeom::Point(_), other) => Err(wrong("point", other)),
        (other, _) => Err(wrong("two planes or two points", other)),
    }
}

/// The plane containing two lines (note §6). Origin `a.point`, x `a.dir`.
///
/// - Parallel lines: normal `a.dir × (b.point − a.point)`.
/// - Intersecting lines: normal `a.dir × b.dir`.
///
/// `Coincident` for the same line, `Skew { distance }` if the lines miss by more than
/// `linear`.
pub fn plane_two_lines(a: &Line3, b: &Line3, tol: &Tolerance) -> Result<Plane, ConstructError> {
    let d1 = a.dir.get();
    let w = b.point - a.point;
    if tol.is_parallel(a.dir, b.dir) {
        let perp = w - d1 * d1.dot(w);
        if perp.length() <= tol.linear {
            return Err(ConstructError::Coincident);
        }
        let n = unit(d1.cross(perp), tol, ConstructError::Coincident)?;
        return Ok(frame(a.point, a.dir, n));
    }
    let c = d1.cross(b.dir.get());
    let c_len = c.length();
    let distance = w.dot(c).abs() / c_len;
    if distance > tol.linear {
        return Err(ConstructError::Skew { distance });
    }
    Ok(frame(a.point, a.dir, Dir3::new_unchecked(c / c_len)))
}

/// `Plane::from_points(a, b, c)`: origin `a`, x along `b − a`, normal `(b−a)×(c−a)`.
/// `Collinear` if the points do not span a plane (including coincident points).
pub fn plane_three_points(
    a: Point3,
    b: Point3,
    c: Point3,
    tol: &Tolerance,
) -> Result<Plane, ConstructError> {
    Plane::from_points(a, b, c, tol).map_err(|_| ConstructError::Collinear)
}

/// The plane tangent to a face at the point of the face closest to `p`, normal = the face's
/// outward normal there, x from `Plane::from_normal`.
///
/// Analytic faces carry no orientation in [`RefGeom`]: their normal points away from the axis
/// or centre (a plane uses its own normal). `NotOnFace { distance: ∞ }` if a free-form face's
/// closest-point query fails, `Degenerate` at a point where the normal is undefined (on an
/// axis, at a centre or apex).
pub fn plane_tangent_at_point(
    surface: &RefGeom,
    p: Point3,
    tol: &Tolerance,
) -> Result<Plane, ConstructError> {
    let (q, n) = closest_on_face(surface, p, tol)?;
    Ok(Plane::from_normal(q, n))
}

/// Closest point of a face to `p` and the face normal there.
fn closest_on_face(
    surface: &RefGeom,
    p: Point3,
    tol: &Tolerance,
) -> Result<(Point3, Dir3), ConstructError> {
    let on_axis = ConstructError::Degenerate("point on the surface axis or centre");
    match surface {
        RefGeom::Plane(pl) => Ok((pl.project(p), pl.normal)),
        RefGeom::Sphere { center, radius } => {
            check_radius(*radius)?;
            let n = unit(p - *center, tol, on_axis)?;
            Ok((*center + n.get() * *radius, n))
        }
        RefGeom::Cylinder { axis, radius } => {
            check_radius(*radius)?;
            let a = axis.dir.get();
            let v = p - axis.point;
            let h = v.dot(a);
            let n = unit(v - a * h, tol, on_axis)?;
            Ok((axis.point + a * h + n.get() * *radius, n))
        }
        RefGeom::Cone {
            apex,
            axis,
            half_angle,
        } => {
            check_half_angle(*half_angle)?;
            let a = axis.get();
            let v = p - *apex;
            let h = v.dot(a);
            let u = unit(v - a * h, tol, on_axis)?.get();
            let rho = (v - a * h).length();
            let (s, c) = half_angle.sin_cos();
            // In the half-plane (axis, u): generator g = (c, s), outward normal (−s, c).
            let t = h * c + rho * s;
            if t <= tol.linear {
                return Err(ConstructError::Degenerate("closest point is the cone apex"));
            }
            let g = a * c + u * s;
            let n = Dir3::new_unchecked((u * c - a * s).normalize());
            Ok((*apex + g * t, n))
        }
        RefGeom::Torus { axis, major, minor } => {
            check_radius(*major)?;
            check_radius(*minor)?;
            let a = axis.dir.get();
            let v = p - axis.point;
            let h = v.dot(a);
            let u = unit(v - a * h, tol, on_axis)?;
            let ring = axis.point + u.get() * *major;
            let n = unit(
                p - ring,
                tol,
                ConstructError::Degenerate("point on the torus ring"),
            )?;
            Ok((ring + n.get() * *minor, n))
        }
        RefGeom::Surface(s) => {
            let uv = s.closest_uv(p).ok_or(ConstructError::NotOnFace {
                distance: f64::INFINITY,
            })?;
            let n = s
                .normal(uv)
                .ok_or(ConstructError::Degenerate("singular point on the face"))?;
            Ok((s.eval(uv), n))
        }
        other => Err(wrong("face", other)),
    }
}

/// The plane normal to a path at parameter `t`: normal = unit tangent, origin `C(t)`,
/// x = the principal normal if the curvature exceeds 1e-12, else `Plane::from_normal`.
///
/// `OutOfRange` if `t` is outside the curve's range, `Degenerate` for a zero tangent.
pub fn plane_along_path(
    curve: &dyn CurveEval,
    t: f64,
    tol: &Tolerance,
) -> Result<Plane, ConstructError> {
    let (t0, t1) = curve.range();
    if !(t >= t0 && t <= t1) {
        return Err(ConstructError::OutOfRange);
    }
    let d1 = curve.d1(t);
    let tangent = unit(d1, tol, ConstructError::Degenerate("zero path tangent"))?;
    let d2 = curve.d2(t);
    let origin = curve.eval(t);
    let speed = d1.length();
    let curvature = d1.cross(d2).length() / (speed * speed * speed);
    if curvature > STRAIGHT_CURVATURE {
        let tv = tangent.get();
        let pn = d2 - tv * tv.dot(d2);
        let pn_len = pn.length();
        if pn_len > 0.0 {
            return Ok(frame(origin, Dir3::new_unchecked(pn / pn_len), tangent));
        }
    }
    Ok(Plane::from_normal(origin, tangent))
}

/// The curve parameter of a path position: a fraction of the arc length (`by_distance =
/// false`, value in [0, 1]) or an arc length from the start (value in [0, length]).
/// `OutOfRange` outside those intervals.
pub fn path_param(
    curve: &dyn CurveEval,
    fraction_or_distance: f64,
    by_distance: bool,
) -> Result<f64, ConstructError> {
    let v = fraction_or_distance;
    let len = curve.length();
    let s = if by_distance {
        if !(v >= 0.0 && v <= len) {
            return Err(ConstructError::OutOfRange);
        }
        v
    } else {
        if !(0.0..=1.0).contains(&v) {
            return Err(ConstructError::OutOfRange);
        }
        v * len
    };
    let (t0, t1) = curve.range();
    Ok(curve.param_at_length(s).clamp(t0, t1))
}

/// The line of a straight input: a line, a segment (directed `a → b`) or a straight free-form
/// curve (directed start → end). `NotStraight` for circles and curved curves.
pub fn line_of(g: &RefGeom, tol: &Tolerance) -> Result<Line3, ConstructError> {
    match g {
        RefGeom::Line(l) => Ok(*l),
        RefGeom::Segment(s) => Ok(Line3 {
            point: s.a,
            dir: unit(
                s.b - s.a,
                tol,
                ConstructError::Degenerate("zero-length edge"),
            )?,
        }),
        RefGeom::Curve(c) => {
            let (t0, t1) = c.range();
            let (p0, p1) = (c.eval(t0), c.eval(t1));
            let dir = unit(p1 - p0, tol, ConstructError::NotStraight)?;
            let line = Line3 { point: p0, dir };
            let straight = (1..STRAIGHT_SAMPLES).all(|i| {
                let t = t0 + (t1 - t0) * f64::from(i) / f64::from(STRAIGHT_SAMPLES);
                let v = c.eval(t) - p0;
                (v - dir.get() * v.dot(dir.get())).length() <= tol.linear
            });
            if straight {
                Ok(line)
            } else {
                Err(ConstructError::NotStraight)
            }
        }
        RefGeom::Circle { .. } => Err(ConstructError::NotStraight),
        other => Err(wrong("line or straight edge", other)),
    }
}

/// The plane of a planar input: a plane, or the plane of a circle (`Plane::from_normal`).
pub fn plane_of(g: &RefGeom) -> Result<Plane, ConstructError> {
    match g {
        RefGeom::Plane(p) => Ok(*p),
        RefGeom::Circle { center, normal, .. } => Ok(Plane::from_normal(*center, *normal)),
        other => Err(wrong("plane", other)),
    }
}

/// The point of a point input (vertex, sketch point, construction point).
pub fn point_of(g: &RefGeom) -> Result<Point3, ConstructError> {
    match g {
        RefGeom::Point(p) => Ok(*p),
        other => Err(wrong("point", other)),
    }
}

// ---- axes (CON-02) ---------------------------------------------------------------------

/// The axis of a revolved input: a cylinder, cone or torus face, or a circle / arc (its
/// normal through the centre). The direction is the surface axis as stored; a cone's axis
/// passes through its apex.
///
/// `Unsupported` for faces without an axis (plane, sphere, free-form), `Degenerate` for a
/// non-positive radius or an invalid cone angle, `WrongInput` for non-faces.
pub fn axis_of(g: &RefGeom) -> Result<Line3, ConstructError> {
    match g {
        RefGeom::Cylinder { axis, radius } => {
            check_radius(*radius)?;
            Ok(*axis)
        }
        RefGeom::Cone {
            apex,
            axis,
            half_angle,
        } => {
            check_half_angle(*half_angle)?;
            Ok(Line3 {
                point: *apex,
                dir: *axis,
            })
        }
        RefGeom::Torus { axis, major, minor } => {
            check_radius(*major)?;
            check_radius(*minor)?;
            Ok(*axis)
        }
        RefGeom::Circle {
            center,
            normal,
            radius,
            ..
        } => {
            check_radius(*radius)?;
            Ok(Line3 {
                point: *center,
                dir: *normal,
            })
        }
        RefGeom::Plane(_) | RefGeom::Sphere { .. } | RefGeom::Surface(_) => {
            Err(ConstructError::Unsupported)
        }
        other => Err(wrong(
            "cylinder, cone or torus face, or circular edge",
            other,
        )),
    }
}

/// The axis along a face's outward normal (note §6).
///
/// - `on_face = true` ("perpendicular at point"): `p` must lie on the face
///   (`NotOnFace { distance }` if it is more than `linear` away); the axis passes through
///   `p`.
/// - `on_face = false` ("perpendicular to face at point"): the axis passes through the face
///   point closest to `p`.
///
/// The direction is the outward normal at the closest point (analytic faces: away from the
/// axis or centre, as in [`plane_tangent_at_point`]). `Degenerate` where the normal is
/// undefined, `NotOnFace { distance: ∞ }` if a free-form closest-point query fails.
pub fn axis_perpendicular_at_point(
    surface: &RefGeom,
    p: Point3,
    on_face: bool,
    tol: &Tolerance,
) -> Result<Line3, ConstructError> {
    let (q, n) = closest_on_face(surface, p, tol)?;
    if !on_face {
        return Ok(Line3 { point: q, dir: n });
    }
    let distance = p.distance(q);
    if distance > tol.linear {
        return Err(ConstructError::NotOnFace { distance });
    }
    Ok(Line3 { point: p, dir: n })
}

/// The intersection line of two planes (`intersect::plane_plane`): direction
/// `a.normal × b.normal`, point = the point of the line closest to the world origin.
/// `Parallel` for parallel (including coincident) planes.
pub fn axis_two_planes(a: &Plane, b: &Plane, tol: &Tolerance) -> Result<Line3, ConstructError> {
    intersect::plane_plane(a, b, tol).ok_or(ConstructError::Parallel)
}

/// The line through `a` towards `b`. `Coincident` if the points are within `linear`.
pub fn axis_two_points(a: Point3, b: Point3, tol: &Tolerance) -> Result<Line3, ConstructError> {
    Ok(Line3 {
        point: a,
        dir: unit(b - a, tol, ConstructError::Coincident)?,
    })
}

// ---- points (CON-02) -------------------------------------------------------------------

/// The intersection of two lines: the midpoint of their closest points
/// (`intersect::line_line`).
///
/// `Coincident` for the same line, `Parallel` for distinct parallel lines, and
/// `Skew { distance }` if the lines miss each other by more than `linear` (the distance is
/// `|(b.point − a.point)·n| / |n|` with `n = a.dir × b.dir`, as in [`plane_two_lines`]).
pub fn point_two_lines(a: &Line3, b: &Line3, tol: &Tolerance) -> Result<Point3, ConstructError> {
    let w = b.point - a.point;
    if tol.is_parallel(a.dir, b.dir) {
        let d = a.dir.get();
        let perp = w - d * d.dot(w);
        return Err(if perp.length() <= tol.linear {
            ConstructError::Coincident
        } else {
            ConstructError::Parallel
        });
    }
    let (pa, pb) = intersect::line_line(a, b, tol).ok_or(ConstructError::Parallel)?;
    let c = a.dir.get().cross(b.dir.get());
    let distance = w.dot(c).abs() / c.length();
    if distance > tol.linear {
        return Err(ConstructError::Skew { distance });
    }
    Ok((pa + pb) * 0.5)
}

/// The unique common point of three planes (`nᵢ·x = nᵢ·oᵢ`).
///
/// Solved relative to `a.origin` with the triple-product form of Cramer's rule:
/// `x = a.origin + (d₂ (n₃×n₁) + d₃ (n₁×n₂)) / det`, `dᵢ = nᵢ·(oᵢ − a.origin)` (so d₁ = 0),
/// `det = n₁·(n₂×n₃)`. `Degenerate` if `|det| ≤ angular` (two planes parallel, or all three
/// sharing a direction).
pub fn point_three_planes(
    a: &Plane,
    b: &Plane,
    c: &Plane,
    tol: &Tolerance,
) -> Result<Point3, ConstructError> {
    let (n1, n2, n3) = (a.normal.get(), b.normal.get(), c.normal.get());
    let det = n1.dot(n2.cross(n3));
    if det.is_nan() || det.abs() <= tol.angular {
        return Err(ConstructError::Degenerate(
            "the planes do not meet in a single point",
        ));
    }
    let o = a.origin;
    let d2 = n2.dot(b.origin - o);
    let d3 = n3.dot(c.origin - o);
    Ok(o + (n3.cross(n1) * d2 + n1.cross(n2) * d3) / det)
}

/// The centre of a circle / arc, sphere or torus.
///
/// `Unsupported` for faces without a centre (plane, cylinder, cone, free-form), `WrongInput`
/// otherwise.
pub fn center_of(g: &RefGeom) -> Result<Point3, ConstructError> {
    match g {
        RefGeom::Circle { center, .. } | RefGeom::Sphere { center, .. } => Ok(*center),
        RefGeom::Torus { axis, .. } => Ok(axis.point),
        RefGeom::Plane(_)
        | RefGeom::Cylinder { .. }
        | RefGeom::Cone { .. }
        | RefGeom::Surface(_) => Err(ConstructError::Unsupported),
        other => Err(wrong("circle, arc, sphere or torus", other)),
    }
}

/// Where an edge (line, straight edge, circle / arc or free-form curve) crosses a plane.
///
/// - Line: `intersect::line_plane`.
/// - Segment: the same, if the crossing lies on the segment (within `linear`).
/// - Circle / arc: the analytic roots of `signed_distance(C(t)) = 0`; the one with the
///   smallest arc parameter (frame `Plane::from_normal(center, normal)`, as in [`RefGeom`]).
/// - Free-form curve: the first sign change of the signed distance over 64 equal parameter
///   intervals, refined by bisection; a sample within `linear` of the plane counts as a root.
///   A curve that only touches the plane between samples is missed.
///
/// `NoIntersection` if there is none, `Coincident` if a line, segment or circle lies in the
/// plane (every point is an intersection).
pub fn point_curve_plane(
    g: &RefGeom,
    p: &Plane,
    tol: &Tolerance,
) -> Result<Point3, ConstructError> {
    match g {
        RefGeom::Line(l) => line_plane_point(l, p, tol),
        RefGeom::Segment(s) => {
            let l = line_of(g, tol)?;
            let x = line_plane_point(&l, p, tol)?;
            let t = (x - s.a).dot(l.dir.get());
            if t < -tol.linear || t > s.length() + tol.linear {
                return Err(ConstructError::NoIntersection);
            }
            Ok(x)
        }
        RefGeom::Circle {
            center,
            normal,
            radius,
            arc,
        } => circle_plane_point(*center, *normal, *radius, *arc, p, tol),
        RefGeom::Curve(c) => curve_plane_point(&**c, p, tol),
        other => Err(wrong("edge, sketch curve or axis", other)),
    }
}

fn line_plane_point(l: &Line3, p: &Plane, tol: &Tolerance) -> Result<Point3, ConstructError> {
    if let Some(x) = intersect::line_plane(l, p, tol) {
        return Ok(x);
    }
    Err(if p.signed_distance(l.point).abs() <= tol.linear {
        ConstructError::Coincident
    } else {
        ConstructError::NoIntersection
    })
}

fn circle_plane_point(
    center: Point3,
    normal: Dir3,
    radius: f64,
    arc: Option<(f64, f64)>,
    p: &Plane,
    tol: &Tolerance,
) -> Result<Point3, ConstructError> {
    check_radius(radius)?;
    let (start, end) = arc.unwrap_or((0.0, TAU));
    if !(start.is_finite() && end.is_finite() && end > start) {
        return Err(ConstructError::Degenerate("empty arc"));
    }
    let f = Plane::from_normal(center, normal);
    let n = p.normal.get();
    // signed_distance(C(t)) = d0 + A cos t + B sin t = d0 + R cos(t − φ).
    let d0 = p.signed_distance(center);
    let a = radius * f.x.get().dot(n);
    let b = radius * f.y.get().dot(n);
    let r = a.hypot(b);
    if r <= radius * tol.angular {
        return Err(if d0.abs() <= tol.linear {
            ConstructError::Coincident
        } else {
            ConstructError::NoIntersection
        });
    }
    let k = -d0 / r;
    // A plane within `linear` of tangency still touches the circle.
    if k.abs() > 1.0 + tol.linear / r {
        return Err(ConstructError::NoIntersection);
    }
    let phi = b.atan2(a);
    let delta = k.clamp(-1.0, 1.0).acos();
    // Angular slack at the arc ends, so a crossing exactly at an end is kept.
    let slack = tol.linear / radius;
    let t = [phi - delta, phi + delta]
        .into_iter()
        // Into [start − slack, start − slack + 2π).
        .map(|t| start - slack + (t - start + slack).rem_euclid(TAU))
        .filter(|t| *t <= end + slack)
        .min_by(f64::total_cmp)
        .ok_or(ConstructError::NoIntersection)?
        .clamp(start, end);
    Ok(center + (f.x.get() * t.cos() + f.y.get() * t.sin()) * radius)
}

fn curve_plane_point(
    c: &dyn CurveEval,
    p: &Plane,
    tol: &Tolerance,
) -> Result<Point3, ConstructError> {
    let (t0, t1) = c.range();
    let f = |t: f64| p.signed_distance(c.eval(t));
    let at = |i: u32| {
        if i == CURVE_PLANE_SAMPLES {
            t1
        } else {
            t0 + (t1 - t0) * f64::from(i) / f64::from(CURVE_PLANE_SAMPLES)
        }
    };
    let (mut lo, mut f_lo) = (t0, f(t0));
    if f_lo.abs() <= tol.linear {
        return Ok(c.eval(lo));
    }
    for i in 1..=CURVE_PLANE_SAMPLES {
        let hi = at(i);
        let f_hi = f(hi);
        if f_hi.abs() <= tol.linear {
            return Ok(c.eval(hi));
        }
        if (f_lo < 0.0) != (f_hi < 0.0) {
            return Ok(c.eval(bisect(&f, lo, hi, f_lo)));
        }
        (lo, f_lo) = (hi, f_hi);
    }
    Err(ConstructError::NoIntersection)
}

/// A root of `f` in `[a, b]`, given a sign change (`f(a) = fa`).
fn bisect(f: &dyn Fn(f64) -> f64, mut a: f64, mut b: f64, mut fa: f64) -> f64 {
    for _ in 0..CURVE_PLANE_BISECTIONS {
        let m = 0.5 * (a + b);
        if m <= a || m >= b {
            break;
        }
        let fm = f(m);
        if fm == 0.0 {
            return m;
        }
        if (fa < 0.0) == (fm < 0.0) {
            (a, fa) = (m, fm);
        } else {
            b = m;
        }
    }
    0.5 * (a + b)
}
