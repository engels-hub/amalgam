//! Exact NURBS forms of lines, conics and the analytic surfaces (P&T ch. 7–8).
//!
//! Conventions:
//! - arcs use rational quadratic segments of at most 90° (A7.1, middle weight `cos(Δθ/2)`);
//!   their knot domain is `[0, |sweep|]`, so the parameter equals the swept angle at every
//!   segment boundary;
//! - revolved surfaces (A8.1) have u = the rotation (degree 2) and v = the profile;
//! - cylinders and cones have u = the angle, v = the height `[0, 1]`;
//! - spheres, tori, cylinders and cones have their normal `S_u × S_v` pointing outward.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use amg_core::{Line3, Plane, Point3, Vec3};

use crate::surface::surface_unchecked;
use crate::{Grid, KnotVector, NurbsCurve, NurbsCurve3, NurbsError, NurbsSurface3};

/// Slack on the 2π sweep limit and on the 90° segment split.
const ANGLE_EPS: f64 = 1e-12;

/// Rational quadratic arc `center + cos θ·ax + sin θ·ay`, θ ∈ start..start+sweep.
/// Returns knots on `[0, |sweep|]`, `2·n + 1` poles and weights. `ax`/`ay` may be zero
/// (collapsed arc). Caller validates `sweep`.
fn arc(
    center: Point3,
    ax: Vec3,
    ay: Vec3,
    start: f64,
    sweep: f64,
) -> (KnotVector, Vec<Point3>, Vec<f64>) {
    let len = sweep.abs();
    let n = ((len / FRAC_PI_2) - ANGLE_EPS).ceil().max(1.0) as usize;
    let dt = sweep / n as f64;
    let w1 = (0.5 * dt).cos();
    let at = |t: f64, s: f64| center + (ax * t.cos() + ay * t.sin()) * s;
    let mut poles = Vec::with_capacity(2 * n + 1);
    let mut weights = Vec::with_capacity(2 * n + 1);
    for i in 0..n {
        let t = start + dt * i as f64;
        poles.push(at(t, 1.0));
        weights.push(1.0);
        poles.push(at(t + 0.5 * dt, 1.0 / w1));
        weights.push(w1);
    }
    let full = (len - TAU).abs() <= ANGLE_EPS;
    poles.push(if full {
        poles[0]
    } else {
        at(start + sweep, 1.0)
    });
    weights.push(1.0);
    let mut k = vec![0.0; 3];
    for i in 1..n {
        let a = len * i as f64 / n as f64;
        k.push(a);
        k.push(a);
    }
    k.extend([len; 3]);
    (KnotVector::new_unchecked(k), poles, weights)
}

/// [`arc`] that rejects overflowing input (huge radii) instead of producing non-finite poles.
fn finite_arc(
    center: Point3,
    ax: Vec3,
    ay: Vec3,
    start: f64,
    sweep: f64,
) -> Result<(KnotVector, Vec<Point3>, Vec<f64>), NurbsError> {
    let r = arc(center, ax, ay, start, sweep);
    if r.1.iter().all(|p| p.is_finite()) {
        Ok(r)
    } else {
        Err(NurbsError::Degenerate(
            "arc pole is not finite (input too large)",
        ))
    }
}

fn check_sweep(sweep: f64) -> Result<(), NurbsError> {
    if !(sweep.is_finite() && sweep != 0.0) {
        return Err(NurbsError::Degenerate("sweep angle is zero or not finite"));
    }
    if sweep.abs() > TAU + ANGLE_EPS {
        return Err(NurbsError::Degenerate("sweep angle exceeds a full turn"));
    }
    Ok(())
}

fn check_positive(x: f64, what: &'static str) -> Result<(), NurbsError> {
    if x.is_finite() && x > 0.0 {
        Ok(())
    } else {
        Err(NurbsError::Degenerate(what))
    }
}

fn check_point(p: Point3) -> Result<(), NurbsError> {
    if p.is_finite() {
        Ok(())
    } else {
        Err(NurbsError::Degenerate("point is not finite"))
    }
}

fn check_plane(pl: &Plane) -> Result<(), NurbsError> {
    check_point(pl.origin)
}

/// Straight segment from `a` (t = 0) to `b` (t = 1), degree 1. Non-finite input coordinates
/// are replaced by 0.
pub fn line(a: Point3, b: Point3) -> NurbsCurve3 {
    let fix = |p: Point3| if p.is_finite() { p } else { Point3::ZERO };
    NurbsCurve::new_unchecked(1, KnotVector::bezier(1), vec![fix(a), fix(b)], None)
}

/// Circular arc (A7.1) of `radius` in `plane`, from angle `start` (from `plane.x` toward
/// `plane.y`, rad) over `sweep` (negative = clockwise, `0 < |sweep| <= 2π`). Domain
/// `[0, |sweep|]`.
pub fn circle_arc(
    plane: &Plane,
    radius: f64,
    start: f64,
    sweep: f64,
) -> Result<NurbsCurve3, NurbsError> {
    check_plane(plane)?;
    check_positive(radius, "radius must be positive and finite")?;
    check_sweep(sweep)?;
    if !start.is_finite() {
        return Err(NurbsError::Degenerate("start angle is not finite"));
    }
    let (k, p, w) = finite_arc(
        plane.origin,
        plane.x.get() * radius,
        plane.y.get() * radius,
        start,
        sweep,
    )?;
    Ok(NurbsCurve::new_unchecked(2, k, p, Some(w)))
}

/// Elliptic arc `origin + rx cos t·x + ry sin t·y` for eccentric angle `t` from `t0` to `t1`
/// (`0 < |t1 − t0| <= 2π`). Domain `[0, |t1 − t0|]`.
pub fn ellipse_arc(
    plane: &Plane,
    rx: f64,
    ry: f64,
    t0: f64,
    t1: f64,
) -> Result<NurbsCurve3, NurbsError> {
    check_plane(plane)?;
    check_positive(rx, "ellipse radius must be positive and finite")?;
    check_positive(ry, "ellipse radius must be positive and finite")?;
    if !t0.is_finite() || !t1.is_finite() {
        return Err(NurbsError::Degenerate("ellipse angle is not finite"));
    }
    check_sweep(t1 - t0)?;
    let (k, p, w) = finite_arc(
        plane.origin,
        plane.x.get() * rx,
        plane.y.get() * ry,
        t0,
        t1 - t0,
    )?;
    Ok(NurbsCurve::new_unchecked(2, k, p, Some(w)))
}

/// Bilinear patch of `plane` covering local `u ∈ u.0..u.1`, `v ∈ v.0..v.1`, on the parameter
/// domain `[0, 1]²` (`S(0,0) = plane.to_world(u.0, v.0)`). Non-finite input gives a patch at
/// the plane origin.
pub fn plane_patch(plane: &Plane, u: (f64, f64), v: (f64, f64)) -> NurbsSurface3 {
    let corner = |a: f64, b: f64| {
        let p = plane.to_world(amg_core::Point2::new(a, b));
        if p.is_finite() { p } else { Point3::ZERO }
    };
    let poles = Grid::from_fn(2, 2, |i, j| {
        corner(
            if i == 0 { u.0 } else { u.1 },
            if j == 0 { v.0 } else { v.1 },
        )
    });
    surface_unchecked(
        [1, 1],
        [KnotVector::bezier(1), KnotVector::bezier(1)],
        poles,
        None,
    )
}

/// Extrusion of `c` along `dir`: `S(u, v) = C(u) + v·dir`, `v ∈ [0, 1]`. A non-finite `dir` is
/// treated as zero.
pub fn extrusion(c: &NurbsCurve3, dir: Vec3) -> NurbsSurface3 {
    let dir = if dir.is_finite() { dir } else { Vec3::ZERO };
    let pc = c.poles();
    let poles = Grid::from_fn(pc.len(), 2, |i, j| if j == 0 { pc[i] } else { pc[i] + dir });
    let poles = if poles.as_slice().iter().all(|p| p.is_finite()) {
        poles
    } else {
        Grid::from_fn(pc.len(), 2, |i, _| pc[i])
    };
    let weights = c.weights().map(|w| Grid::from_fn(w.len(), 2, |i, _| w[i]));
    surface_unchecked(
        [c.degree(), 1],
        [c.knots().clone(), KnotVector::bezier(1)],
        poles,
        weights,
    )
}

/// Ruled surface `S(u, v) = (1 − v)·A(u) + v·B(u)` in homogeneous space. Both curves must
/// already be compatible: same degree and knots (within `1e-12` of the domain length).
/// General compatibility (degree elevation + knot union) arrives with KH-12/KH-14.
pub fn ruled(a: &NurbsCurve3, b: &NurbsCurve3) -> Result<NurbsSurface3, NurbsError> {
    let (ka, kb) = (a.knots().as_slice(), b.knots().as_slice());
    let (d0, d1) = a.domain();
    let tol = 1e-12 * (d1 - d0).abs().max(1.0);
    if a.degree() != b.degree()
        || ka.len() != kb.len()
        || ka.iter().zip(kb).any(|(x, y)| (x - y).abs() > tol)
    {
        return Err(NurbsError::Incompatible(
            "ruled surface needs curves with the same degree and knots".to_owned(),
        ));
    }
    let n = a.poles().len();
    let (pa, pb) = (a.poles(), b.poles());
    let poles = Grid::from_fn(n, 2, |i, j| if j == 0 { pa[i] } else { pb[i] });
    let weights = (a.weights().is_some() || b.weights().is_some()).then(|| {
        let wa = a.weights();
        let wb = b.weights();
        Grid::from_fn(n, 2, |i, j| {
            let w = if j == 0 { wa } else { wb };
            w.map_or(1.0, |w| w[i])
        })
    });
    Ok(surface_unchecked(
        [a.degree(), 1],
        [a.knots().clone(), KnotVector::bezier(1)],
        poles,
        weights,
    ))
}

/// Surface of revolution (A8.1): `profile` rotated about `axis` by `angle` (rad, right-handed
/// about `axis.dir`, `0 < |angle| <= 2π`). u = rotation (domain `[0, |angle|]`), v = profile.
pub fn revolution(
    profile: &NurbsCurve3,
    axis: &Line3,
    angle: f64,
) -> Result<NurbsSurface3, NurbsError> {
    check_point(axis.point)?;
    check_sweep(angle)?;
    let dir = axis.dir.get();
    let pp = profile.poles();
    let pw = profile.weights();
    let mut columns = Vec::with_capacity(pp.len());
    let mut knots_u = None;
    let mut arc_w = Vec::new();
    for &p in pp {
        let o = axis.point + dir * (p - axis.point).dot(dir);
        let x = p - o;
        let y = dir.cross(x);
        let (k, poles, w) = arc(o, x, y, 0.0, angle);
        // The radial point itself is exact (the arc's first pole is `o + x` up to rounding).
        let mut poles = poles;
        poles[0] = p;
        if (angle.abs() - TAU).abs() <= ANGLE_EPS {
            let last = poles.len() - 1;
            poles[last] = p;
        }
        if knots_u.is_none() {
            knots_u = Some(k);
            arc_w = w;
        }
        columns.push(poles);
    }
    let knots_u = knots_u.ok_or(NurbsError::Degenerate("profile has no poles"))?;
    let nu = arc_w.len();
    let nv = pp.len();
    let poles = Grid::from_fn(nu, nv, |i, j| columns[j][i]);
    if poles.as_slice().iter().any(|p| !p.is_finite()) {
        return Err(NurbsError::Degenerate("revolved pole is not finite"));
    }
    let weights = Grid::from_fn(nu, nv, |i, j| arc_w[i] * pw.map_or(1.0, |w| w[j]));
    Ok(surface_unchecked(
        [2, profile.degree()],
        [knots_u, profile.knots().clone()],
        poles,
        Some(weights),
    ))
}

/// Cylinder of radius `r` on `plane` (axis = `plane.normal`), height `h > 0`, swept from
/// `plane.x` by `angle` (`0 < |angle| <= 2π`). u = angle, v = height in `[0, 1]`.
pub fn cylinder(plane: &Plane, r: f64, h: f64, angle: f64) -> Result<NurbsSurface3, NurbsError> {
    check_positive(h, "height must be positive and finite")?;
    let base = circle_arc(plane, r, 0.0, angle)?;
    Ok(extrusion(&base, plane.normal.get() * h))
}

/// Cone frustum on `plane`: radius `r0` at the plane, `r1` at height `h > 0` along the
/// normal, swept by `angle`. Radii are `>= 0` and not both zero (one zero gives an apex).
/// u = angle, v = height in `[0, 1]`.
pub fn cone(
    plane: &Plane,
    r0: f64,
    r1: f64,
    h: f64,
    angle: f64,
) -> Result<NurbsSurface3, NurbsError> {
    check_plane(plane)?;
    check_positive(h, "height must be positive and finite")?;
    check_sweep(angle)?;
    let ok = |r: f64| r.is_finite() && r >= 0.0;
    if !ok(r0) || !ok(r1) || (r0 == 0.0 && r1 == 0.0) {
        return Err(NurbsError::Degenerate(
            "cone radii must be >= 0, finite and not both zero",
        ));
    }
    let (x, y, n) = (plane.x.get(), plane.y.get(), plane.normal.get());
    let (k, p0, w) = finite_arc(plane.origin, x * r0, y * r0, 0.0, angle)?;
    let (_, p1, _) = finite_arc(plane.origin + n * h, x * r1, y * r1, 0.0, angle)?;
    let a = NurbsCurve::new_unchecked(2, k.clone(), p0, Some(w.clone()));
    let b = NurbsCurve::new_unchecked(2, k, p1, Some(w));
    ruled(&a, &b)
}

/// Full sphere of radius `r`: the meridian semicircle (−90°..90° in the XZ plane, through
/// `center + r·X`) revolved 2π about the Z axis through `center`. u = longitude
/// `[0, 2π]`, v = latitude parameter `[0, π]` (south to north pole).
pub fn sphere(center: Point3, r: f64) -> Result<NurbsSurface3, NurbsError> {
    check_point(center)?;
    check_positive(r, "radius must be positive and finite")?;
    let (k, mut p, w) = finite_arc(center, Vec3::X * r, Vec3::Z * r, -FRAC_PI_2, PI)?;
    // Exact poles on the axis (cos(±π/2) is not exactly 0).
    let last = p.len() - 1;
    p[0] = center - Vec3::Z * r;
    p[last] = center + Vec3::Z * r;
    let meridian = NurbsCurve::new_unchecked(2, k, p, Some(w));
    let axis = Line3 {
        point: center,
        dir: amg_core::Dir3::Z,
    };
    let mut s = revolution(&meridian, &axis, TAU)?;
    // Collapse the pole rows exactly onto the axis.
    s = snap_axis_poles(s, center, r);
    Ok(s)
}

/// Sets the v-end pole columns of a sphere to the exact poles (they are on the axis, so every
/// rotation of them is the same point).
fn snap_axis_poles(s: NurbsSurface3, center: Point3, r: f64) -> NurbsSurface3 {
    let nv = s.poles().nv();
    let poles = Grid::from_fn(s.poles().nu(), nv, |i, j| {
        if j == 0 {
            center - Vec3::Z * r
        } else if j == nv - 1 {
            center + Vec3::Z * r
        } else {
            s.poles().get(i, j)
        }
    });
    surface_unchecked(
        [s.degree(crate::UV::U), s.degree(crate::UV::V)],
        [s.knots(crate::UV::U).clone(), s.knots(crate::UV::V).clone()],
        poles,
        s.weights().cloned(),
    )
}

/// Ring torus on `plane` (axis = `plane.normal` through `plane.origin`): tube of radius
/// `minor` around a circle of radius `major`, `0 < minor < major`. u = rotation about the
/// axis `[0, 2π]`, v = tube angle `[0, 2π]` (starting at the outer equator).
pub fn torus(plane: &Plane, major: f64, minor: f64) -> Result<NurbsSurface3, NurbsError> {
    check_plane(plane)?;
    check_positive(major, "major radius must be positive and finite")?;
    check_positive(minor, "minor radius must be positive and finite")?;
    if minor >= major {
        return Err(NurbsError::Degenerate(
            "torus minor radius must be below the major radius",
        ));
    }
    let (x, n) = (plane.x.get(), plane.normal.get());
    let (k, p, w) = finite_arc(plane.origin + x * major, x * minor, n * minor, 0.0, TAU)?;
    let tube = NurbsCurve::new_unchecked(2, k, p, Some(w));
    revolution(
        &tube,
        &Line3 {
            point: plane.origin,
            dir: plane.normal,
        },
        TAU,
    )
}

#[cfg(test)]
#[allow(clippy::float_cmp, reason = "exact expected values")]
mod tests {
    use super::*;
    use amg_core::Tolerance;

    #[test]
    fn quarter_circle_is_one_segment() {
        let c = circle_arc(&Plane::XY, 2.0, 0.0, FRAC_PI_2).unwrap();
        assert_eq!(c.poles().len(), 3);
        assert!((c.eval(FRAC_PI_4_DOMAIN).length() - 2.0).abs() < 1e-14);
        let full = circle_arc(&Plane::XY, 2.0, 0.3, TAU).unwrap();
        assert_eq!(full.poles().len(), 9);
        assert!(full.is_closed(&Tolerance::DEFAULT));
        assert_eq!(full.start(), full.end());
        let cw = circle_arc(&Plane::XY, 1.0, 0.0, -PI).unwrap();
        assert!((cw.eval(0.5 * PI) - Point3::new(0.0, -1.0, 0.0)).length() < 1e-15);
    }

    const FRAC_PI_4_DOMAIN: f64 = std::f64::consts::FRAC_PI_4;

    #[test]
    fn outward_normals() {
        let tol = Tolerance::DEFAULT;
        let s = sphere(Point3::ZERO, 3.0).unwrap();
        let [(u0, u1), (v0, v1)] = s.domain();
        for (u, v) in [(0.3, 0.7), (u0, v0), (u1, v1), (1.0, 2.0)] {
            let p = s.eval(u, v);
            let n = s.normal(u, v, &tol).unwrap().get();
            assert!(n.dot(p.normalize()) > 0.99, "u={u} v={v} p={p} n={n}");
        }
        let t = torus(&Plane::XY, 5.0, 1.0).unwrap();
        let p = t.eval(0.4, 0.9);
        let ring = Vec3::new(p.x, p.y, 0.0).normalize() * 5.0;
        assert!(
            t.normal(0.4, 0.9, &tol)
                .unwrap()
                .get()
                .dot((p - ring).normalize())
                > 0.999
        );
        let c = cylinder(&Plane::XY, 2.0, 4.0, TAU).unwrap();
        let p = c.eval(1.0, 0.5);
        assert!(
            c.normal(1.0, 0.5, &tol)
                .unwrap()
                .get()
                .dot(Vec3::new(p.x, p.y, 0.0).normalize())
                > 0.999
        );
        assert!((p.z - 2.0).abs() < 1e-14);
    }
}
