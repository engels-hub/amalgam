//! B-rep faces, edges and vertices → [`RefGeom`] through the S02 analytic types.

use std::f64::consts::TAU;
use std::sync::Arc;

use amg_brep::geom::{BSplineCurveData, Curve3, Surface};
use amg_brep::{BrepKernel, EdgeId, FaceId, Shape, SubKind, VertexId};
use amg_core::{Dir3, Line3, Plane, Point2, Point3, Segment3, Tolerance, Transform, Vec3};
use amg_doc::naming::{self, TopoKind};
use amg_nurbs::{KnotVector, NurbsCurve3};

use super::super::curves::FreeCurve;
use super::super::{ConstructError, RefGeom, ResolvedRef, SurfaceEval, math};
use super::Fail;

/// Face parameters (as fractions of the UV bounds) tried when a surface query needs a regular
/// point.
const UV_SAMPLES: [(f64, f64); 5] = [
    (0.5, 0.5),
    (0.3, 0.4),
    (0.7, 0.6),
    (0.15, 0.85),
    (0.85, 0.15),
];

fn geometry(e: impl std::fmt::Display) -> Fail {
    Fail::Geometry(e.to_string())
}

/// Sub-entity `(kind, index)` of `shape` placed by the rigid transform `t`, and the face
/// orientation (`true`: the outward normal points towards the axis or centre).
pub(super) fn sub_entity(
    kernel: &dyn BrepKernel,
    tol: &Tolerance,
    shape: &Shape,
    kind: TopoKind,
    index: u32,
    t: &Transform,
) -> Result<(ResolvedRef, bool), Fail> {
    let sub = kind.sub_kind().ok_or(Fail::Unsupported("a face group"))?;
    if index >= shape.count(sub) {
        return Err(Fail::Missing(format!("{sub:?} {index}")));
    }
    // A rigid move keeps every sub-shape index.
    let moved;
    let shape = if *t == Transform::IDENTITY {
        shape
    } else {
        moved = shape.transformed(t).map_err(geometry)?;
        &moved
    };
    match sub {
        SubKind::Face => {
            let (geom, reversed) = face(kernel, tol, shape, FaceId(index))?;
            Ok((ResolvedRef::new(geom), reversed))
        }
        SubKind::Edge => {
            let e = EdgeId(index);
            let mut resolved = ResolvedRef::new(edge(shape, e)?);
            resolved.context_plane = shape
                .edge_faces(e)
                .iter()
                .find_map(|&f| plane_of_face(shape, f));
            Ok((resolved, false))
        }
        SubKind::Vertex => {
            let p = shape.vertex_point(VertexId(index));
            Ok((ResolvedRef::new(RefGeom::Point(p)), false))
        }
    }
}

/// The plane of a planar face, with its normal out of the material.
fn plane_of_face(shape: &Shape, f: FaceId) -> Option<Plane> {
    match shape.surface(f) {
        Surface::Plane(p) if shape.face_reversed(f) => Some(p.flipped()),
        Surface::Plane(p) => Some(p),
        _ => None,
    }
}

fn axis(frame: &Plane) -> Line3 {
    Line3 {
        point: frame.origin,
        dir: frame.normal,
    }
}

/// The UV point at fractions `(a, b)` of the face's parameter bounds.
fn uv_at(bounds: [f64; 4], (a, b): (f64, f64)) -> Point2 {
    let [u0, u1, v0, v1] = bounds;
    Point2::new(u0 + (u1 - u0) * a, v0 + (v1 - v0) * b)
}

fn face(
    kernel: &dyn BrepKernel,
    tol: &Tolerance,
    shape: &Shape,
    f: FaceId,
) -> Result<(RefGeom, bool), Fail> {
    if let Some(p) = plane_of_face(shape, f) {
        return Ok((RefGeom::Plane(p), false));
    }
    let geom = match shape.surface(f) {
        Surface::Cylinder { frame, radius } => RefGeom::Cylinder {
            axis: axis(&frame),
            radius,
        },
        Surface::Cone { frame, .. } => cone(kernel, shape, f, &frame)?,
        Surface::Sphere { frame, radius } => RefGeom::Sphere {
            center: frame.origin,
            radius,
        },
        Surface::Torus {
            frame,
            major,
            minor,
        } => RefGeom::Torus {
            axis: axis(&frame),
            major,
            minor,
        },
        // Planes returned above; everything else keeps the exact surface behind the adapter,
        // whose normals already point out of the material.
        Surface::Plane(_) | Surface::BSpline(_) | Surface::Other { .. } => {
            let surface = BrepSurface {
                shape: shape.clone(),
                face: f,
            };
            return Ok((RefGeom::Surface(Arc::new(surface)), false));
        }
    };
    // Compare the face's outward normal with the analytic surface's "away" direction. This
    // also covers the left-handed frames of mirrored shapes.
    let bounds = shape.face_uv_bounds(f);
    let mut error = None;
    for sample in UV_SAMPLES {
        match kernel.eval_surface(shape, f, uv_at(bounds, sample)) {
            Ok(s) => {
                if let Ok(away) = math::axis_perpendicular_at_point(&geom, s.point, false, tol) {
                    let reversed = s.normal.get().dot(away.dir.get()) < 0.0;
                    return Ok((geom, reversed));
                }
            }
            Err(e) => error = Some(e),
        }
    }
    Err(error.map_or_else(
        || geometry(ConstructError::Degenerate("the face has no regular point")),
        geometry,
    ))
}

/// The cone of a conical face, from two surface points: this holds for either handedness of
/// the surface frame.
fn cone(kernel: &dyn BrepKernel, shape: &Shape, f: FaceId, frame: &Plane) -> Result<RefGeom, Fail> {
    let bounds = shape.face_uv_bounds(f);
    let n = frame.normal.get();
    // Height along the axis and distance from it.
    let sample = |v: f64| -> Result<(f64, f64), Fail> {
        let p: Point3 = kernel
            .eval_surface(shape, f, uv_at(bounds, (0.5, v)))
            .map_err(geometry)?
            .point;
        let d = p - frame.origin;
        let h = d.dot(n);
        Ok((h, (d - n * h).length()))
    };
    let ((h0, r0), (h1, r1)) = (sample(0.25)?, sample(0.75)?);
    let slope = (r1 - r0) / (h1 - h0);
    if !slope.is_finite() || slope == 0.0 {
        return Err(geometry(ConstructError::Degenerate("cone angle")));
    }
    let apex = frame.origin + n * (h0 - r0 / slope);
    let axis: Dir3 = if slope > 0.0 {
        frame.normal
    } else {
        -frame.normal
    };
    Ok(RefGeom::Cone {
        apex,
        axis,
        half_angle: slope.abs().atan(),
    })
}

/// The circle or arc about `normal` that starts in direction `start` (from the centre) and
/// sweeps `sweep > 0` counter-clockwise, in the angle convention of [`RefGeom::Circle`].
pub(super) fn circle(
    center: Point3,
    normal: Dir3,
    radius: f64,
    start: Vec3,
    sweep: f64,
) -> RefGeom {
    let arc = (sweep < TAU - Tolerance::DEFAULT.angular).then(|| {
        let f = Plane::from_normal(center, normal);
        let a = start
            .dot(f.y.get())
            .atan2(start.dot(f.x.get()))
            .rem_euclid(TAU);
        (a, a + sweep)
    });
    RefGeom::Circle {
        center,
        normal,
        radius,
        arc,
    }
}

/// An edge's geometry, directed along its curve's parameter.
fn edge(shape: &Shape, e: EdgeId) -> Result<RefGeom, Fail> {
    let (curve, t0, t1) = shape
        .curve(e)
        .ok_or_else(|| Fail::Geometry("the edge is degenerate".to_owned()))?;
    Ok(match curve {
        Curve3::Line { origin, dir } => RefGeom::Segment(Segment3 {
            a: origin + dir.get() * t0,
            b: origin + dir.get() * t1,
        }),
        Curve3::Circle { frame, radius } => circle(
            frame.origin,
            frame.normal,
            radius,
            frame.x.get() * t0.cos() + frame.y.get() * t0.sin(),
            t1 - t0,
        ),
        Curve3::Ellipse {
            frame,
            major_radius,
            minor_radius,
        } => RefGeom::Curve(Arc::new(
            FreeCurve::ellipse(frame, major_radius, minor_radius, t0, t1).map_err(geometry)?,
        )),
        Curve3::BSpline(data) => RefGeom::Curve(Arc::new(
            FreeCurve::nurbs(nurbs(data)?, t0, t1).map_err(geometry)?,
        )),
        Curve3::Other => {
            return Err(Fail::Unsupported(
                "an offset, parabolic, hyperbolic or other special edge",
            ));
        }
    })
}

fn nurbs(d: BSplineCurveData) -> Result<NurbsCurve3, Fail> {
    if d.periodic {
        return Err(Fail::Unsupported("a periodic B-spline edge"));
    }
    let mults: Vec<usize> = d.mults.iter().map(|&m| m as usize).collect();
    let knots = KnotVector::from_knots_mults(&d.knots, &mults).map_err(geometry)?;
    NurbsCurve3::new(usize::from(d.degree), knots, d.poles, d.weights).map_err(geometry)
}

/// A free-form face as a [`SurfaceEval`], evaluated by the default kernel.
#[derive(Debug)]
struct BrepSurface {
    shape: Shape,
    face: FaceId,
}

impl SurfaceEval for BrepSurface {
    /// NaN if the kernel can't evaluate the surface at or next to `uv`.
    fn eval(&self, uv: Point2) -> Point3 {
        let kernel = naming::default_kernel();
        if let Ok(s) = kernel.eval_surface(&self.shape, self.face, uv) {
            return s.point;
        }
        // The kernel refuses singular points (a pole): step towards the middle of the face.
        let centre = uv_at(self.shape.face_uv_bounds(self.face), (0.5, 0.5));
        kernel
            .eval_surface(&self.shape, self.face, uv + (centre - uv) * 1e-9)
            .map_or(Point3::NAN, |s| s.point)
    }
    fn normal(&self, uv: Point2) -> Option<Dir3> {
        naming::default_kernel()
            .eval_surface(&self.shape, self.face, uv)
            .ok()
            .map(|s| s.normal)
    }
    fn closest_uv(&self, p: Point3) -> Option<Point2> {
        naming::default_kernel()
            .project_point_on_face(&self.shape, self.face, p)
            .ok()
            .map(|(uv, _)| uv)
    }
}

#[cfg(all(test, feature = "occt"))]
mod tests;
