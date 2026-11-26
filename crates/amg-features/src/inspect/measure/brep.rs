//! B-rep operands (S05 §2 "Phase 2", INS-07): exact entity values and distances through the
//! S02 queries, finite stand-ins for unbounded reference geometry, and the BREP-08
//! tessellation for mixed mesh–B-rep distances.
//!
//! Only `amg_brep`'s public API is used. Without the `occt` feature no real shape exists, and
//! every kernel call reports [`InspectError::BackendUnavailable`].

use std::sync::{Arc, OnceLock};

use amg_brep::geom::{Curve3, CurveSeg, Surface, Tag};
use amg_brep::{
    BrepKernel, DistanceResult, EdgeId, FaceId, KernelConfig, KernelError, MassProps, OcctKernel,
    Shape, SubKind, SubShape, TessQuality, VertexId,
};
use amg_core::{Area, Dir3, Length, Plane, Point3, Tolerance, Transform, Volume};
use amg_jobs::JobCtx;
use amg_mesh::{IndexedMesh, MeshBody};

use super::prims::Prim;
use super::{Circle3, CircleFit, EntityMeasure, MeasureOpts, PlaneFit, transform_plane};
use crate::construct::RefGeom;
use crate::inspect::InspectError;

/// Linear deflection of the fine tessellation as a fraction of the bounds' diagonal (the
/// BREP-08 fine preset, [`TessQuality::export`]).
const FINE_RELATIVE: f64 = 1e-4;
/// Largest linear deflection of the fine tessellation, mm.
const FINE_MAX: f64 = 0.01;
/// How much wider than the other shape a stand-in for a plane or line is.
const COVER: f64 = 1.5;

/// A B-rep shape or one of its faces, edges or vertices.
#[derive(Clone, Debug)]
pub struct BrepOperand {
    /// The shape (cheap clone).
    pub shape: Shape,
    /// `None` means the whole shape.
    pub sub: Option<SubShape>,
    /// Shape → world; must be rigid ([`InspectError::NonRigidTransform`] otherwise).
    pub to_world: Transform,
}

fn kernel() -> &'static OcctKernel {
    static KERNEL: OnceLock<OcctKernel> = OnceLock::new();
    KERNEL.get_or_init(|| OcctKernel::new(KernelConfig::default()))
}

fn error(e: KernelError) -> InspectError {
    match e {
        KernelError::Cancelled => InspectError::Cancelled,
        KernelError::BackendUnavailable => InspectError::BackendUnavailable,
        KernelError::NonRigid => InspectError::NonRigidTransform,
        other => InspectError::Brep(other.to_string()),
    }
}

/// Checks the transform and the sub-shape index.
fn check(op: &BrepOperand) -> Result<(), InspectError> {
    if !super::distance::is_rigid(&op.to_world) {
        return Err(InspectError::NonRigidTransform);
    }
    if let Some(s) = op.sub
        && s.index >= op.shape.count(s.kind)
    {
        return Err(InspectError::BadElement(format!(
            "{:?} {}",
            s.kind, s.index
        )));
    }
    Ok(())
}

/// The world position of a vertex operand, `None` for other operands.
pub(super) fn vertex(op: &BrepOperand) -> Result<Option<Point3>, InspectError> {
    check(op)?;
    Ok(match op.sub {
        Some(SubShape {
            kind: SubKind::Vertex,
            index,
        }) => Some(
            op.to_world
                .transform_point3(op.shape.vertex_point(VertexId(index))),
        ),
        _ => None,
    })
}

/// The operand as a stand-alone shape in world space.
fn placed(op: &BrepOperand) -> Result<Shape, InspectError> {
    check(op)?;
    let moved = if op.to_world == Transform::IDENTITY {
        op.shape.clone()
    } else {
        op.shape.transformed(&op.to_world).map_err(error)?
    };
    match op.sub {
        // A rigid move keeps every sub-shape index.
        Some(s) => moved.sub_shape(s).map_err(error),
        None => Ok(moved),
    }
}

fn mass_props(s: &Shape) -> Result<MassProps, InspectError> {
    kernel().mass_props(s).map_err(error)
}

/// Fills the single-entity results of `op` (exact: S02 mass properties and analytic types).
pub(super) fn measure_one(
    op: &BrepOperand,
    e: &mut EntityMeasure,
    ctx: &JobCtx,
) -> Result<(), InspectError> {
    if let Some(p) = vertex(op)? {
        e.position = Some(p);
        return Ok(());
    }
    let t = &op.to_world;
    ctx.checkpoint()?;
    match op.sub {
        Some(SubShape {
            kind: SubKind::Edge,
            index,
        }) => edge(op, EdgeId(index), e)?,
        Some(SubShape {
            kind: SubKind::Face,
            index,
        }) => {
            let f = FaceId(index);
            let alone = op.shape.sub_shape(SubShape {
                kind: SubKind::Face,
                index,
            });
            let props = mass_props(&alone.map_err(error)?)?;
            e.area = Some(Area::from_canonical(props.area));
            e.centroid = Some(t.transform_point3(props.centroid));
            e.face_count = Some(1);
            match op.shape.surface(f) {
                Surface::Plane(p) => {
                    // The normal out of the material.
                    let p = if op.shape.face_reversed(f) {
                        p.flipped()
                    } else {
                        p
                    };
                    e.plane = Some(PlaneFit {
                        plane: transform_plane(&p, t),
                        max_deviation: 0.0,
                    });
                }
                Surface::Cylinder { radius, .. } | Surface::Sphere { radius, .. } => {
                    e.radius = Some(Length::mm(radius));
                }
                _ => {}
            }
        }
        // Vertices returned above.
        _ => {
            let shape = placed(op)?;
            let props = mass_props(&shape)?;
            e.area = Some(Area::from_canonical(props.area));
            e.centroid = Some(props.centroid);
            e.face_count = Some(shape.count(SubKind::Face) as usize);
            // Without solids the volume is 0 (a surface body).
            if props.volume > 0.0 {
                e.volume = Some(Volume::from_canonical(props.volume));
            }
            e.bbox = Some(shape.bbox());
        }
    }
    ctx.checkpoint()?;
    Ok(())
}

fn edge(op: &BrepOperand, id: EdgeId, e: &mut EntityMeasure) -> Result<(), InspectError> {
    let t = &op.to_world;
    let sub = SubShape {
        kind: SubKind::Edge,
        index: id.0,
    };
    let Some((curve, t0, t1)) = op.shape.curve(id) else {
        return Err(InspectError::Degenerate("the edge has no curve"));
    };
    let dir = |d: Dir3| Dir3::new(t.transform_vector3(d.get()), &Tolerance::DEFAULT);
    let length = match &curve {
        Curve3::Line { dir: d, .. } => {
            e.direction = dir(*d);
            t1 - t0
        }
        Curve3::Circle { frame, radius } => {
            if let Some(normal) = dir(frame.normal) {
                e.circle = Some(CircleFit {
                    circle: Circle3 {
                        center: t.transform_point3(frame.origin),
                        normal,
                        radius: *radius,
                    },
                    rms: 0.0,
                    max_residual: 0.0,
                });
            }
            e.radius = Some(Length::mm(*radius));
            radius * (t1 - t0)
        }
        _ => kernel().sub_geom(&op.shape, sub).map_err(error)?.measure,
    };
    e.length = Some(Length::mm(length));
    let alone = op.shape.sub_shape(sub).map_err(error)?;
    e.centroid = Some(t.transform_point3(mass_props(&alone)?.centroid));
    Ok(())
}

/// The point of the operand's boundary nearest to the world point `p`: `(distance, point)`.
pub(super) fn closest_point(op: &BrepOperand, p: Point3) -> Result<(f64, Point3), InspectError> {
    let r = kernel().closest_point(&placed(op)?, p).map_err(error)?;
    Ok((r.distance, r.on_a))
}

/// The exact distance between two B-rep operands (`BRepExtrema`): 0 where they touch or
/// overlap, also for a shape inside a solid.
pub(super) fn distance(
    a: &BrepOperand,
    b: &BrepOperand,
    ctx: &JobCtx,
) -> Result<DistanceResult, InspectError> {
    let (a, b) = (placed(a)?, placed(b)?);
    ctx.checkpoint()?;
    let r = kernel().distance(&a, &b).map_err(error)?;
    // The query itself can't be interrupted.
    ctx.checkpoint()?;
    Ok(r)
}

/// A wire through the points, skipping repeated ones; `None` if they are all one point.
fn polyline_wire(points: &[Point3], tol: &Tolerance) -> Result<Option<Shape>, InspectError> {
    let mut segments = Vec::with_capacity(points.len());
    let mut from = points.first().copied();
    for &to in points.iter().skip(1) {
        let Some(a) = from else { break };
        let d = to - a;
        let Some(dir) = Dir3::new(d, tol) else {
            continue;
        };
        segments.push(CurveSeg {
            curve: Curve3::Line { origin: a, dir },
            t0: 0.0,
            t1: d.length(),
            tag: Tag(0),
        });
        from = Some(to);
    }
    if segments.is_empty() {
        return Ok(None);
    }
    let wire = kernel().make_wire(&segments, tol).map_err(error)?;
    Ok(Some(wire.shape))
}

/// The distance from the operand to unbounded or bounded reference geometry (not a point):
/// `(distance, on the shape, on the geometry)` and the deflection if a polyline stood in for
/// the geometry.
///
/// A plane is stood in for by a square and a line by a segment that reach past the projection
/// of the shape's bounds: the nearest point of a plane or line to any point of the shape lies
/// within it, so the distance is exact.
pub(super) fn distance_to_ref(
    op: &BrepOperand,
    g: &RefGeom,
    opts: &MeasureOpts,
    ctx: &JobCtx,
) -> Result<(f64, Point3, Point3, Option<f64>), InspectError> {
    let shape = placed(op)?;
    let bounds = shape.bbox();
    if bounds.is_empty() {
        return Err(InspectError::Degenerate("the shape is empty"));
    }
    let (centre, reach) = (bounds.center(), COVER * (0.5 * bounds.diagonal()) + 1.0);
    let tol = opts.tol;
    let mut deflection = None;
    let stand_in = match g {
        RefGeom::Plane(p) => {
            let c = p.project(centre);
            let (x, y) = (p.x.get() * reach, p.y.get() * reach);
            Some(
                kernel()
                    .make_planar_polygon(&[c - x - y, c + x - y, c + x + y, c - x + y], &[], &tol)
                    .map_err(error)?,
            )
        }
        RefGeom::Line(l) => {
            let d = l.dir.get();
            let c = l.point + d * (centre - l.point).dot(d);
            polyline_wire(&[c - d * reach, c + d * reach], &tol)?
        }
        RefGeom::Segment(s) => polyline_wire(&[s.a, s.b], &tol)?,
        RefGeom::Circle {
            center,
            normal,
            radius,
            arc,
        } => {
            let (t0, t1) = arc.unwrap_or((0.0, std::f64::consts::TAU));
            let segment = CurveSeg {
                curve: Curve3::Circle {
                    frame: Plane::from_normal(*center, *normal),
                    radius: *radius,
                },
                t0,
                t1,
                tag: Tag(0),
            };
            Some(kernel().make_wire(&[segment], &tol).map_err(error)?.shape)
        }
        RefGeom::Curve(_) => {
            let d = fine_deflection(bounds.diagonal());
            let points = super::analytic::polyline(g, d, ctx)?
                .ok_or(InspectError::Degenerate("the curve has no points"))?;
            deflection = Some(d);
            polyline_wire(&points, &tol)?
        }
        _ => {
            return Err(InspectError::Unsupported {
                what: "distance",
                a: "B-rep geometry",
                b: g.kind_name(),
            });
        }
    };
    let stand_in = stand_in.ok_or(InspectError::Degenerate("the curve is a single point"))?;
    ctx.checkpoint()?;
    let r = kernel().distance(&shape, &stand_in).map_err(error)?;
    ctx.checkpoint()?;
    Ok((r.distance, r.on_a, r.on_b, deflection))
}

/// The linear deflection of the fine tessellation for a shape with this bounds diagonal, mm.
pub(super) fn fine_deflection(diagonal: f64) -> f64 {
    (FINE_RELATIVE * diagonal).clamp(1e-7, FINE_MAX)
}

/// A B-rep operand as a tessellation in world space.
pub(super) enum Tessellation {
    /// A body (closed if the solid is).
    Body(MeshBody),
    /// The triangles of one face.
    Face(MeshBody),
    /// The segments of one edge.
    Edge(Vec<Prim>),
}

/// Tessellates the operand (not a vertex) with the fine preset; also the deflection used.
pub(super) fn tessellate(
    op: &BrepOperand,
    ctx: &JobCtx,
) -> Result<(Tessellation, f64), InspectError> {
    let shape = placed(op)?;
    let deflection = fine_deflection(shape.bbox().diagonal());
    let quality = TessQuality {
        linear: deflection,
        relative: false,
        max_linear_abs: None,
        ..TessQuality::export()
    };
    ctx.checkpoint()?;
    if matches!(op.sub, Some(s) if s.kind == SubKind::Edge) {
        let points = kernel()
            .tessellate_edge(&shape, EdgeId(0), &quality)
            .map_err(error)?;
        let segments = points.windows(2).map(|w| Prim::Seg([w[0], w[1]])).collect();
        return Ok((Tessellation::Edge(segments), deflection));
    }
    let tess = kernel().tessellate(&shape, &quality, ctx).map_err(error)?;
    let (positions, tris, _) = tess.to_indexed_parts();
    let mesh = IndexedMesh::new(positions, tris)?;
    ctx.checkpoint()?;
    let body = MeshBody::from_arc(Arc::new(mesh));
    Ok((
        if op.sub.is_some() {
            Tessellation::Face(body)
        } else {
            Tessellation::Body(body)
        },
        deflection,
    ))
}
