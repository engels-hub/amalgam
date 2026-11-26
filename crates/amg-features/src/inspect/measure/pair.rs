//! Minimum distance when an operand is reference or B-rep geometry (S05 §2 "Phase 2",
//! INS-07). Points, lines, segments, planes and circle–plane pairs are closed forms; B-rep
//! pairs are `BRepExtrema`; a mesh against a B-rep, and curves without a closed form, are
//! measured on polylines or the fine tessellation and marked [`Exactness::Tessellated`].

use amg_core::{Aabb3, Length, Plane, Point3, Segment3};
use amg_jobs::JobCtx;

use super::brep::{self, BrepOperand, Tessellation};
use super::distance::{Lowered, check_mesh, closest, lower, lower_prims, surface_point, vertex};
use super::prims::{Prim, PrimFeature};
use super::{
    Exactness, MeasureOpts, MeshElement, MeshOperand, MinDistance, Operand, Witness,
    WitnessFeature, analytic, containment,
};
use crate::construct::RefGeom;
use crate::inspect::InspectError;

/// How much wider than the other operand a stand-in segment for a line is.
const COVER: f64 = 1.5;

/// An operand as the distance dispatch sees it. The order of the variants is the order the
/// pairs are written in.
#[derive(Clone, Copy)]
enum Side<'a> {
    /// Any single point: free, mesh vertex or surface point, reference point, B-rep vertex.
    Point(Point3, WitnessFeature),
    /// Any other mesh element.
    Mesh(&'a Operand),
    /// Reference geometry that is not a point.
    Ref(&'a RefGeom),
    /// A B-rep shape, face or edge.
    Brep(&'a BrepOperand),
}

impl Side<'_> {
    fn rank(&self) -> u8 {
        match self {
            Side::Point(..) => 0,
            Side::Mesh(_) => 1,
            Side::Ref(_) => 2,
            Side::Brep(_) => 3,
        }
    }
}

fn side(op: &Operand) -> Result<Side<'_>, InspectError> {
    Ok(match op {
        Operand::Point(p) => Side::Point(*p, WitnessFeature::Point),
        Operand::Mesh(mo) => {
            check_mesh(mo)?;
            let m = mo.body.mesh();
            let world = |p| mo.to_world.transform_point3(p);
            match &mo.element {
                MeshElement::Vertex(v) => {
                    Side::Point(world(vertex(m, *v)?), WitnessFeature::Vertex(*v))
                }
                MeshElement::SurfacePoint { face, bary } => Side::Point(
                    world(surface_point(m, *face, bary)?),
                    WitnessFeature::Face(*face),
                ),
                _ => Side::Mesh(op),
            }
        }
        Operand::Ref(r) => match r.geom {
            RefGeom::Point(p) => Side::Point(p, WitnessFeature::Point),
            _ => Side::Ref(&r.geom),
        },
        Operand::Brep(b) => match brep::vertex(b)? {
            Some(p) => Side::Point(p, WitnessFeature::Brep),
            None => Side::Brep(b),
        },
    })
}

/// A closest pair in world space.
struct Found {
    distance: f64,
    a: Witness,
    b: Witness,
    contained: Option<bool>,
    exactness: Exactness,
}

impl Found {
    fn new(
        distance: f64,
        (pa, fa): (Point3, WitnessFeature),
        (pb, fb): (Point3, WitnessFeature),
    ) -> Found {
        Found {
            distance,
            a: Witness {
                point: pa,
                feature: fa,
            },
            b: Witness {
                point: pb,
                feature: fb,
            },
            contained: None,
            exactness: Exactness::Exact,
        }
    }

    fn swapped(self) -> Found {
        Found {
            a: self.b,
            b: self.a,
            ..self
        }
    }

    fn approximate(mut self, deflection: Option<f64>) -> Found {
        if let Some(deflection) = deflection {
            self.exactness = Exactness::Tessellated { deflection };
        }
        self
    }
}

/// [`super::min_distance`] for pairs with a reference or B-rep operand.
pub(super) fn min_distance(
    a: &Operand,
    b: &Operand,
    opts: &MeasureOpts,
    ctx: &JobCtx,
) -> Result<MinDistance, InspectError> {
    ctx.checkpoint()?;
    let (sa, sb) = (side(a)?, side(b)?);
    let found = if sa.rank() <= sb.rank() {
        ordered(sa, sb, opts, ctx)?
    } else {
        ordered(sb, sa, opts, ctx)?.swapped()
    };
    Ok(MinDistance {
        distance: Length::mm(found.distance),
        a: found.a,
        b: found.b,
        intersecting: found.distance <= opts.tol.linear,
        contained: found.contained,
        exactness: found.exactness,
    })
}

/// The closest pair of `a` and `b`, with `a.rank() ≤ b.rank()`.
fn ordered(
    a: Side<'_>,
    b: Side<'_>,
    opts: &MeasureOpts,
    ctx: &JobCtx,
) -> Result<Found, InspectError> {
    Ok(match (a, b) {
        (Side::Point(p, fa), Side::Point(q, fb)) => Found::new(p.distance(q), (p, fa), (q, fb)),
        (Side::Point(p, fa), Side::Mesh(op)) => {
            let point = lower_prims(vec![Prim::Point(p)], ctx)?;
            let mut found = engine(&point, &lower(op, ctx)?, ctx)?;
            found.a.feature = fa;
            found
        }
        (Side::Point(p, fa), Side::Ref(g)) => {
            let q = analytic::closest_to_point(g, p)?;
            Found::new(p.distance(q), (p, fa), (q, WitnessFeature::Point))
        }
        (Side::Point(p, fa), Side::Brep(s)) => {
            let (d, q) = brep::closest_point(s, p)?;
            Found::new(d, (p, fa), (q, WitnessFeature::Brep))
        }
        (Side::Mesh(x), Side::Mesh(y)) => engine(&lower(x, ctx)?, &lower(y, ctx)?, ctx)?,
        (Side::Mesh(op), Side::Ref(g)) => mesh_ref(&lower(op, ctx)?, g, ctx)?,
        (Side::Mesh(op), Side::Brep(s)) => mesh_brep(&lower(op, ctx)?, s, opts, ctx)?,
        (Side::Ref(x), Side::Ref(y)) => ref_ref(x, y, ctx)?,
        (Side::Ref(g), Side::Brep(s)) => {
            let (d, on_shape, on_ref, deflection) = brep::distance_to_ref(s, g, opts, ctx)?;
            Found::new(
                d,
                (on_ref, WitnessFeature::Point),
                (on_shape, WitnessFeature::Brep),
            )
            .approximate(deflection)
        }
        (Side::Brep(x), Side::Brep(y)) => {
            let r = brep::distance(x, y, ctx)?;
            Found::new(
                r.distance,
                (r.on_a, WitnessFeature::Brep),
                (r.on_b, WitnessFeature::Brep),
            )
        }
        // Excluded by the rank order.
        (_, Side::Point(..))
        | (Side::Ref(_) | Side::Brep(_), Side::Mesh(_))
        | (Side::Brep(_), Side::Ref(_)) => {
            return Err(InspectError::Degenerate("operands out of order"));
        }
    })
}

/// The primitive-set distance of INS-01, in world space.
fn engine(a: &Lowered, b: &Lowered, ctx: &JobCtx) -> Result<Found, InspectError> {
    let c = closest(a, b, ctx)?.ok_or(InspectError::Degenerate("an operand is empty"))?;
    Ok(Found::new(
        c.dist_sq.sqrt(),
        (a.to_world.transform_point3(c.pa), c.fa),
        (a.to_world.transform_point3(c.pb), c.fb),
    ))
}

fn segments(points: &[Point3]) -> Vec<Prim> {
    points.windows(2).map(|w| Prim::Seg([w[0], w[1]])).collect()
}

fn world_bounds(set: &Lowered, ctx: &JobCtx) -> Result<Aabb3, InspectError> {
    let mut bounds = Aabb3::EMPTY;
    for i in 0..set.len() as u32 {
        if i.is_multiple_of(4096) {
            ctx.checkpoint()?;
        }
        let (min, max) = set.world_prim(i).bounds();
        bounds = bounds.union(&Aabb3 { min, max });
    }
    Ok(bounds)
}

/// The part of the line `point + t·dir` that holds the nearest point of the line to every point
/// of `set`.
fn line_stand_in(l: &amg_core::Line3, set: &Lowered, ctx: &JobCtx) -> Result<Prim, InspectError> {
    let bounds = world_bounds(set, ctx)?;
    if bounds.is_empty() {
        return Err(InspectError::Degenerate("an operand is empty"));
    }
    let d = l.dir.get();
    let c = l.point + d * (bounds.center() - l.point).dot(d);
    let reach = COVER * (0.5 * bounds.diagonal()) + 1.0;
    Ok(Prim::Seg([c - d * reach, c + d * reach]))
}

/// A primitive set against a plane, exactly: the nearest corner, or a point where a primitive
/// crosses the plane (the first in primitive order). The set is `a`.
fn set_plane(set: &Lowered, plane: &Plane, ctx: &JobCtx) -> Result<Found, InspectError> {
    let mut best: Option<(f64, Point3, WitnessFeature)> = None;
    for i in 0..set.len() as u32 {
        if i.is_multiple_of(4096) {
            ctx.checkpoint()?;
        }
        let prim = set.world_prim(i);
        let corners: &[Point3] = match &prim {
            Prim::Point(p) => std::slice::from_ref(p),
            Prim::Seg(s) => s,
            Prim::Tri(t) => t,
        };
        for (k, &p) in (0u8..).zip(corners) {
            let s = plane.signed_distance(p);
            for (l, &q) in (0u8..).zip(corners).skip(usize::from(k) + 1) {
                let t = plane.signed_distance(q);
                if s * t < 0.0 {
                    let x = p.lerp(q, s / (s - t));
                    let feature = set.witness_feature(i, PrimFeature::Edge(k, l));
                    return Ok(Found::new(0.0, (x, feature), (x, WitnessFeature::Point)));
                }
            }
            if best.is_none_or(|b| s.abs() < b.0) {
                let feature = set.witness_feature(i, PrimFeature::Corner(k));
                best = Some((s.abs(), p, feature));
            }
        }
    }
    let (d, p, feature) = best.ok_or(InspectError::Degenerate("an operand is empty"))?;
    Ok(Found::new(
        d,
        (p, feature),
        (plane.project(p), WitnessFeature::Point),
    ))
}

/// The size a curve's polyline deflection scales with.
fn curve_size(g: &RefGeom) -> f64 {
    match g {
        RefGeom::Circle { radius, .. } => 2.0 * radius,
        RefGeom::Curve(c) => c.length(),
        _ => 0.0,
    }
}

fn unsupported(a: &'static str, b: &'static str) -> InspectError {
    InspectError::Unsupported {
        what: "distance",
        a,
        b,
    }
}

/// A mesh element (`a`) against reference geometry (`b`).
fn mesh_ref(mesh: &Lowered, g: &RefGeom, ctx: &JobCtx) -> Result<Found, InspectError> {
    let prims = match g {
        RefGeom::Plane(p) => return set_plane(mesh, p, ctx),
        RefGeom::Line(l) => vec![line_stand_in(l, mesh, ctx)?],
        RefGeom::Segment(Segment3 { a, b }) => vec![Prim::Seg([*a, *b])],
        RefGeom::Circle { .. } | RefGeom::Curve(_) => {
            let d = brep::fine_deflection(curve_size(g));
            let points = analytic::polyline(g, d, ctx)?
                .ok_or(InspectError::Degenerate("the curve has no points"))?;
            let mut found = engine(mesh, &lower_prims(segments(&points), ctx)?, ctx)?;
            found.b.feature = WitnessFeature::Point;
            return Ok(found.approximate(Some(d)));
        }
        _ => return Err(unsupported("mesh geometry", g.kind_name())),
    };
    engine(mesh, &lower_prims(prims, ctx)?, ctx)
}

/// A mesh element (`a`) against a B-rep shape, face or edge (`b`) through its tessellation.
fn mesh_brep(
    mesh: &Lowered,
    shape: &BrepOperand,
    opts: &MeasureOpts,
    ctx: &JobCtx,
) -> Result<Found, InspectError> {
    let (tess, deflection) = brep::tessellate(shape, ctx)?;
    let element = |body, element| {
        let op = Operand::Mesh(MeshOperand {
            body,
            to_world: amg_core::Transform::IDENTITY,
            element,
        });
        lower(&op, ctx)
    };
    let other = match tess {
        Tessellation::Body(body) => element(body, MeshElement::Body)?,
        Tessellation::Face(body) => {
            let faces = (0..body.mesh().face_count() as u32)
                .map(amg_mesh::FaceId)
                .collect();
            element(body, MeshElement::Faces(faces))?
        }
        Tessellation::Edge(segments) => lower_prims(segments, ctx)?,
    };
    let mut found = engine(mesh, &other, ctx)?;
    // The indices are the tessellation's: they name nothing in the document.
    found.b.feature = WitnessFeature::Brep;
    found.contained = containment(mesh, &other, found.distance <= opts.tol.linear, ctx)?;
    Ok(found.approximate(Some(deflection)))
}

/// Two reference geometries, neither a point.
fn ref_ref(a: &RefGeom, b: &RefGeom, ctx: &JobCtx) -> Result<Found, InspectError> {
    if let Some((d, p, q)) = analytic::exact_pair(a, b)? {
        return Ok(Found::new(
            d,
            (p, WitnessFeature::Point),
            (q, WitnessFeature::Point),
        ));
    }
    let no = || unsupported(a.kind_name(), b.kind_name());
    let curve = |g: &RefGeom| matches!(g, RefGeom::Circle { .. } | RefGeom::Curve(_));
    // One side at least is a curve: put it first.
    if !curve(a) {
        return if curve(b) {
            Ok(ref_ref(b, a, ctx)?.swapped())
        } else {
            Err(no())
        };
    }
    let deflection = brep::fine_deflection(curve_size(a));
    let points = analytic::polyline(a, deflection, ctx)?.ok_or_else(no)?;
    let first = lower_prims(segments(&points), ctx)?;
    let mut total = deflection;
    let second = match b {
        RefGeom::Plane(p) => return Ok(set_plane(&first, p, ctx)?.approximate(Some(deflection))),
        RefGeom::Line(l) => vec![line_stand_in(l, &first, ctx)?],
        RefGeom::Segment(Segment3 { a, b }) => vec![Prim::Seg([*a, *b])],
        RefGeom::Circle { .. } | RefGeom::Curve(_) => {
            let d = brep::fine_deflection(curve_size(b));
            total += d;
            segments(&analytic::polyline(b, d, ctx)?.ok_or_else(no)?)
        }
        _ => return Err(no()),
    };
    Ok(engine(&first, &lower_prims(second, ctx)?, ctx)?.approximate(Some(total)))
}
