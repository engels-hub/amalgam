//! Measurement on mesh bodies (S05 §2, INS-01; conventions in
//! `docs/notes/inspection-geometry.md` §2).
//!
//! - [`measure_one`] / [`measure`]: per-entity results and pair results (minimum distance,
//!   angle, centre distance), all in world space.
//! - [`min_distance`]: exact minimum distance with witness points and features, by best-first
//!   simultaneous BVH traversal and exact triangle–triangle distance. The result is the
//!   lexicographic minimum of `(distance, a primitive, b primitive)`, so it is the same for
//!   any traversal order or thread count. It is computed in operand A's local frame, with B's
//!   primitives mapped by `A.to_world⁻¹ · B.to_world`.
//! - Smart selection: [`grow_planar_region`], [`grow_feature_chain`]; fits:
//!   [`fit_plane_faces`], [`circle_through`], [`fit_circle`].
//! - Picking support: [`snap_on_face`]. Display: [`report_rows`], [`area_unit`],
//!   [`volume_unit`].
//!
//! A whole-body operand uses the body's BVH. Until S07 adds `MeshBody::spatial()`, the BVH
//! and the watertight flag are cached per mesh inside this module; [`prepare_body`] builds
//! them ahead of time.

mod display;
mod distance;
mod fit;
pub mod prims;
mod select;
#[cfg(test)]
mod tests;

use std::sync::Arc;

use amg_core::{
    Aabb3, Angle, Area, Dir3, Length, Plane, Point3, Ray3, Tolerance, Transform, Vec3, Volume,
};
use amg_jobs::JobCtx;
use amg_mesh::{FaceGroupId, FaceId, IndexedMesh, MeshBody};

pub use display::{DisplayPrefs, MeasureRow, area_unit, report_rows, volume_unit};
pub use fit::{circle_through, fit_circle, fit_plane_faces};
pub use select::{grow_feature_chain, grow_planar_region, snap_on_face};

use super::InspectError;
use distance::{Lowered, closest, element_faces, lower, polyline_segments, surface_point, vertex};

/// Sub-element of a mesh body; indices are body-local.
#[derive(Clone, Debug, PartialEq)]
pub enum MeshElement {
    /// The whole body.
    Body,
    /// One face group.
    FaceGroup(FaceGroupId),
    /// Sorted, deduplicated. One picked triangle, or a smart planar region.
    Faces(Arc<[FaceId]>),
    /// A triangle edge given by its two vertex indices.
    Edge {
        /// First vertex.
        a: u32,
        /// Second vertex.
        b: u32,
    },
    /// Vertex chain: a boundary loop (`AnalysisReport::boundary_loops`) or a feature-edge
    /// chain.
    Polyline {
        /// Vertices in chain order.
        verts: Arc<[u32]>,
        /// The last vertex connects back to the first.
        closed: bool,
    },
    /// One vertex.
    Vertex(u32),
    /// A point on a face.
    SurfacePoint {
        /// The face.
        face: FaceId,
        /// Barycentric weights of the face's corners.
        bary: [f64; 3],
    },
}

/// A mesh element with its body and a rigid body → world transform.
#[derive(Clone, Debug)]
pub struct MeshOperand {
    /// The body (cheap clone).
    pub body: MeshBody,
    /// Body → world; must be rigid ([`InspectError::NonRigidTransform`] otherwise).
    pub to_world: Transform,
    /// The element measured.
    pub element: MeshElement,
}

/// What is measured.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub enum Operand {
    /// A free world point.
    Point(Point3),
    /// A mesh element.
    Mesh(MeshOperand),
    // INS-07 adds `Ref(construct::ResolvedRef)` and `Brep(BrepOperand)`.
}

/// Tolerances of a measurement (S05 §8).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MeasureOpts {
    /// Linear / angular tolerance; `Tolerance::mesh()` by default.
    pub tol: Tolerance,
    /// Planarity and straightness tolerance, mm (1e-3).
    pub planar_tolerance: f64,
    /// Accepted RMS residual of a circle fit, mm (1e-2).
    pub circle_tolerance: f64,
}

impl Default for MeasureOpts {
    fn default() -> Self {
        MeasureOpts {
            tol: Tolerance::mesh(),
            planar_tolerance: 1e-3,
            circle_tolerance: 1e-2,
        }
    }
}

/// Operand kind (for labels and filters).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[allow(missing_docs, reason = "names are self-explanatory")]
pub enum OperandKind {
    Point,
    Vertex,
    SurfacePoint,
    Edge,
    Polyline,
    Faces,
    FaceGroup,
    Body,
    Construction,
    Sketch,
    BrepFace,
    BrepEdge,
    BrepVertex,
    BrepBody,
}

impl OperandKind {
    /// Lower-case name for messages.
    pub fn name(self) -> &'static str {
        match self {
            OperandKind::Point => "point",
            OperandKind::Vertex => "vertex",
            OperandKind::SurfacePoint => "surface point",
            OperandKind::Edge => "edge",
            OperandKind::Polyline => "polyline",
            OperandKind::Faces => "faces",
            OperandKind::FaceGroup => "face group",
            OperandKind::Body => "body",
            OperandKind::Construction => "construction geometry",
            OperandKind::Sketch => "sketch entity",
            OperandKind::BrepFace => "B-rep face",
            OperandKind::BrepEdge => "B-rep edge",
            OperandKind::BrepVertex => "B-rep vertex",
            OperandKind::BrepBody => "B-rep body",
        }
    }
}

impl Operand {
    /// The operand's kind.
    pub fn kind(&self) -> OperandKind {
        match self {
            Operand::Point(_) => OperandKind::Point,
            Operand::Mesh(m) => match m.element {
                MeshElement::Body => OperandKind::Body,
                MeshElement::FaceGroup(_) => OperandKind::FaceGroup,
                MeshElement::Faces(_) => OperandKind::Faces,
                MeshElement::Edge { .. } => OperandKind::Edge,
                MeshElement::Polyline { .. } => OperandKind::Polyline,
                MeshElement::Vertex(_) => OperandKind::Vertex,
                MeshElement::SurfacePoint { .. } => OperandKind::SurfacePoint,
            },
        }
    }
}

/// A circle in space.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Circle3 {
    /// Centre.
    pub center: Point3,
    /// Unit normal of the circle's plane.
    pub normal: Dir3,
    /// Radius, mm.
    pub radius: f64,
}

/// An accepted circle fit.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CircleFit {
    /// The circle.
    pub circle: Circle3,
    /// RMS geometric residual, mm.
    pub rms: f64,
    /// Largest geometric residual, mm.
    pub max_residual: f64,
}

/// A best-fit plane.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlaneFit {
    /// The plane (normal = area-weighted face normal).
    pub plane: Plane,
    /// Largest vertex distance from the plane, mm.
    pub max_deviation: f64,
}

/// Whether a result is exact or comes from a tessellation (INS-07).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Exactness {
    /// Exact for the given geometry.
    Exact,
    /// Computed on a tessellation with this deflection, mm.
    Tessellated {
        /// Chordal deflection, mm.
        deflection: f64,
    },
}

/// Single-entity results (world space). `None` = not applicable to this element.
#[derive(Clone, Debug, PartialEq)]
pub struct EntityMeasure {
    /// Operand kind.
    pub kind: OperandKind,
    /// Point / vertex position.
    pub position: Option<Point3>,
    /// Edge or polyline length.
    pub length: Option<Length>,
    /// Face set or body area.
    pub area: Option<Area>,
    /// Body volume (watertight bodies only).
    pub volume: Option<Volume>,
    /// Length centroid (edges, polylines) or area centroid (faces, bodies).
    pub centroid: Option<Point3>,
    /// Best-fit plane of a planar face set.
    pub plane: Option<PlaneFit>,
    /// Straight edge / chain, directed first → last vertex.
    pub direction: Option<Dir3>,
    /// Accepted circle fit of a closed chain.
    pub circle: Option<CircleFit>,
    /// INS-07: cylinder / sphere face, circular edge.
    pub radius: Option<Length>,
    /// World bounds (bodies).
    pub bbox: Option<Aabb3>,
    /// Number of faces (face sets, bodies).
    pub face_count: Option<usize>,
    /// Exactness of the values.
    pub exactness: Exactness,
}

impl EntityMeasure {
    fn empty(kind: OperandKind) -> Self {
        EntityMeasure {
            kind,
            position: None,
            length: None,
            area: None,
            volume: None,
            centroid: None,
            plane: None,
            direction: None,
            circle: None,
            radius: None,
            bbox: None,
            face_count: None,
            exactness: Exactness::Exact,
        }
    }

    /// Centre for [`PairMeasure::center_distance`] (note §2.6): the circle centre if a fit was
    /// accepted, else the position, else the centroid.
    pub fn center(&self) -> Option<Point3> {
        self.circle
            .map(|c| c.circle.center)
            .or(self.position)
            .or(self.centroid)
    }
}

/// Mesh feature holding a witness point.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WitnessFeature {
    /// A free point (or the operand is a point).
    Point,
    /// A mesh vertex.
    Vertex(u32),
    /// A mesh edge, vertex indices ascending.
    Edge(u32, u32),
    /// The interior of a face.
    Face(FaceId),
    /// INS-07.
    Brep,
}

/// A witness point (world) and the feature it lies on.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Witness {
    /// World position.
    pub point: Point3,
    /// Feature of the operand.
    pub feature: WitnessFeature,
}

/// Minimum distance between two operands.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MinDistance {
    /// The distance.
    pub distance: Length,
    /// Witness on operand A.
    pub a: Witness,
    /// Witness on operand B.
    pub b: Witness,
    /// `distance ≤ tol.linear`.
    pub intersecting: bool,
    /// `Some(true)`: one watertight body lies inside the other (the distance is still the
    /// surface distance). `None` unless both operands are watertight bodies.
    pub contained: Option<bool>,
    /// Exactness.
    pub exactness: Exactness,
}

impl MinDistance {
    /// `b − a` (world).
    pub fn delta(&self) -> Vec3 {
        self.b.point - self.a.point
    }
}

/// Which pair of directions an angle is between.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AngleKind {
    /// Two lines (undirected), [0, π/2].
    LineLine,
    /// Two planes (oriented normals), [0, π].
    PlanePlane,
    /// Line and plane, [0, π/2].
    LinePlane,
}

/// An angle and its supplement. Ranges and conventions: note §2.5.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AngleMeasure {
    /// Kind.
    pub kind: AngleKind,
    /// The angle.
    pub angle: Angle,
    /// π − angle (line–line, plane–plane) or π/2 − angle (line–plane).
    pub supplement: Angle,
}

/// Results for a pair of operands.
#[derive(Clone, Debug, PartialEq)]
pub struct PairMeasure {
    /// Minimum distance.
    pub min_distance: MinDistance,
    /// Angle, when both entities have a direction or plane.
    pub angle: Option<AngleMeasure>,
    /// Distance between the entities' centres (note §2.6).
    pub center_distance: Length,
}

/// Everything measured for one or two operands.
#[derive(Clone, Debug, PartialEq)]
pub struct MeasureReport {
    /// First operand.
    pub first: EntityMeasure,
    /// Second operand.
    pub second: Option<EntityMeasure>,
    /// Pair results.
    pub pair: Option<PairMeasure>,
}

/// Where a measurement may run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cost {
    /// Proven < 0.1 ms: may run on the UI thread.
    Inline,
    /// Spawn an `Interactive` job.
    Job,
}

/// Largest primitive count of an operand measured inline.
const INLINE_MAX: usize = 64;

fn inline_prims(op: &Operand) -> Option<usize> {
    match op {
        Operand::Point(_) => Some(1),
        Operand::Mesh(m) => match &m.element {
            MeshElement::Vertex(_)
            | MeshElement::SurfacePoint { .. }
            | MeshElement::Edge { .. } => Some(1),
            MeshElement::Polyline { verts, .. } => Some(verts.len()),
            MeshElement::Faces(f) => Some(f.len()),
            // A group needs a scan of the face → group map, a body its BVH.
            MeshElement::FaceGroup(_) | MeshElement::Body => None,
        },
    }
}

/// `Inline` only for work proven < 0.1 ms: no BVH traversal and ≤ 64 primitives per operand.
pub fn estimated_cost(a: &Operand, b: Option<&Operand>) -> Cost {
    let ok = |o: &Operand| inline_prims(o).is_some_and(|n| n <= INLINE_MAX);
    if ok(a) && b.is_none_or(ok) {
        Cost::Inline
    } else {
        Cost::Job
    }
}

/// Builds and caches the BVH and watertight flag of `body`, so later measurements of the whole
/// body do not pay for them.
///
/// # Errors
/// [`InspectError::Mesh`] (invalid mesh), [`InspectError::Cancelled`].
pub fn prepare_body(body: &MeshBody, ctx: &JobCtx) -> Result<(), InspectError> {
    let d = super::cache::derived(body, ctx)?;
    d.watertight(ctx)?;
    d.min_prims();
    Ok(())
}

/// Single-entity measurement in world space.
///
/// # Errors
/// [`InspectError::NonRigidTransform`], [`InspectError::BadElement`],
/// [`InspectError::Cancelled`].
pub fn measure_one(
    a: &Operand,
    opts: &MeasureOpts,
    ctx: &JobCtx,
) -> Result<EntityMeasure, InspectError> {
    let mut e = EntityMeasure::empty(a.kind());
    let mo = match a {
        Operand::Point(p) => {
            e.position = Some(*p);
            return Ok(e);
        }
        Operand::Mesh(mo) => mo,
    };
    distance::check_mesh(mo)?;
    let m: &IndexedMesh = mo.body.mesh();
    let t = &mo.to_world;
    let w = |p: Point3| t.transform_point3(p);
    match &mo.element {
        MeshElement::Vertex(v) => e.position = Some(w(vertex(m, *v)?)),
        MeshElement::SurfacePoint { face, bary } => {
            e.position = Some(w(surface_point(m, *face, bary)?));
        }
        MeshElement::Edge { a: va, b: vb } => {
            let (pa, pb) = (w(vertex(m, *va)?), w(vertex(m, *vb)?));
            if va == vb {
                return Err(InspectError::BadElement(format!("edge {va}–{vb}")));
            }
            e.length = Some(Length::mm(pa.distance(pb)));
            e.centroid = Some((pa + pb) * 0.5);
            e.direction = Dir3::new(pb - pa, &opts.tol);
        }
        MeshElement::Polyline { verts, closed } => {
            let segs = polyline_segments(m, verts, *closed)?;
            let pts: Vec<Point3> = verts.iter().map(|&v| w(m.positions[v as usize])).collect();
            let mut len = 0.0;
            let mut c = Vec3::ZERO;
            for &(i, j) in &segs {
                let (p, q) = (w(m.positions[i as usize]), w(m.positions[j as usize]));
                let l = p.distance(q);
                len += l;
                c += (p + q) * (0.5 * l);
            }
            e.length = Some(Length::mm(len));
            e.centroid = Some(if len > 0.0 { c / len } else { pts[0] });
            if !*closed {
                e.direction = straight_direction(&pts, opts);
            } else if pts.len() >= 5 {
                e.circle = fit_circle(&pts, opts);
            }
        }
        el @ (MeshElement::Faces(_) | MeshElement::FaceGroup(_)) => {
            let faces: Vec<FaceId> = element_faces(m, el)?
                .unwrap_or_default()
                .into_iter()
                .map(FaceId)
                .collect();
            let (area, centroid) = area_centroid(m, faces.iter().copied());
            e.area = Some(Area::from_canonical(area));
            e.centroid = centroid.map(w);
            e.face_count = Some(faces.len());
            e.plane = fit_plane_faces(m, &faces)
                .filter(|f| f.max_deviation <= opts.planar_tolerance)
                .map(|f| PlaneFit {
                    plane: transform_plane(&f.plane, t),
                    max_deviation: f.max_deviation,
                });
        }
        MeshElement::Body => {
            let d = super::cache::derived(&mo.body, ctx)?;
            let (area, centroid) = area_centroid(m, (0..m.face_count() as u32).map(FaceId));
            ctx.checkpoint()?;
            e.area = Some(Area::from_canonical(area));
            e.centroid = centroid.map(w);
            e.face_count = Some(m.face_count());
            if d.watertight(ctx)? {
                e.volume = Some(Volume::from_canonical(m.signed_volume().abs()));
            }
            let used = used_vertices(m);
            e.bbox = Some(Aabb3::from_points(
                used.into_iter().map(|v| w(m.positions[v as usize])),
            ));
        }
    }
    Ok(e)
}

/// Vertices referenced by a face, ascending.
fn used_vertices(m: &IndexedMesh) -> Vec<u32> {
    let mut used = vec![false; m.vertex_count()];
    for t in &m.tris {
        for &v in t {
            used[v as usize] = true;
        }
    }
    (0..m.vertex_count() as u32)
        .filter(|&v| used[v as usize])
        .collect()
}

fn transform_plane(p: &Plane, t: &Transform) -> Plane {
    let dir = |d: Dir3| Dir3::new_unchecked(t.transform_vector3(d.get()).normalize());
    let normal = dir(p.normal);
    let x = dir(p.x);
    Plane {
        origin: t.transform_point3(p.origin),
        x,
        y: Dir3::new_unchecked(normal.get().cross(x.get())),
        normal,
    }
}

/// Total area and area-weighted centroid (local) of faces, summed in the given order.
fn area_centroid(m: &IndexedMesh, faces: impl Iterator<Item = FaceId>) -> (f64, Option<Point3>) {
    let mut area = 0.0;
    let mut c = Vec3::ZERO;
    for f in faces {
        let [a, b, p] = m.triangle(f);
        let ar = 0.5 * (b - a).cross(p - a).length();
        area += ar;
        c += (a + b + p) * (ar / 3.0);
    }
    (area, (area > 0.0).then(|| c / area))
}

/// Direction first → last if every vertex lies within `planar_tolerance` of the end-to-end
/// segment (note §2.4).
fn straight_direction(pts: &[Point3], opts: &MeasureOpts) -> Option<Dir3> {
    let (first, last) = (*pts.first()?, *pts.last()?);
    let dir = Dir3::new(last - first, &opts.tol)?;
    let seg = amg_core::Segment3 { a: first, b: last };
    pts.iter()
        .all(|p| amg_core::intersect::point_segment_distance(*p, &seg) <= opts.planar_tolerance)
        .then_some(dir)
}

/// Exact minimum distance between two operands (note §2.2), with witnesses in world space.
///
/// # Errors
/// [`InspectError::NonRigidTransform`], [`InspectError::BadElement`],
/// [`InspectError::Degenerate`] (empty body), [`InspectError::Cancelled`].
pub fn min_distance(
    a: &Operand,
    b: &Operand,
    opts: &MeasureOpts,
    ctx: &JobCtx,
) -> Result<MinDistance, InspectError> {
    let la = lower(a, ctx)?;
    let lb = lower(b, ctx)?;
    let c = closest(&la, &lb, ctx)?.ok_or(InspectError::Degenerate("an operand is empty"))?;
    let distance = c.dist_sq.sqrt();
    let intersecting = distance <= opts.tol.linear;
    let contained = containment(&la, &lb, intersecting, ctx)?;
    Ok(MinDistance {
        distance: Length::mm(distance),
        a: Witness {
            point: la.to_world.transform_point3(c.pa),
            feature: c.fa,
        },
        b: Witness {
            point: la.to_world.transform_point3(c.pb),
            feature: c.fb,
        },
        intersecting,
        contained,
        exactness: Exactness::Exact,
    })
}

/// `Some(inside)` when both operands are watertight bodies (note §2.2).
fn containment(
    a: &Lowered,
    b: &Lowered,
    intersecting: bool,
    ctx: &JobCtx,
) -> Result<Option<bool>, InspectError> {
    let (Some(da), Some(db)) = (&a.body, &b.body) else {
        return Ok(None);
    };
    if !da.watertight(ctx)? || !db.watertight(ctx)? {
        return Ok(None);
    }
    if intersecting {
        return Ok(Some(false));
    }
    // x's first used vertex, in y's frame, tested by ray parity against y (three rays).
    let inside =
        |x: &Lowered, dx: &super::cache::Derived, y: &Lowered, dy: &super::cache::Derived| {
            let t = y.to_world.inverse() * x.to_world;
            let mx = dx.spatial.mesh();
            let v = mx.tris[0][0];
            ray_parity_inside(&dy.spatial, t.transform_point3(mx.positions[v as usize]))
        };
    Ok(Some(inside(a, da, b, db) || inside(b, db, a, da)))
}

/// Majority vote of three ray-parity tests (crossings counted once per distinct `t`).
fn ray_parity_inside(s: &amg_mesh::MeshSpatial, p: Point3) -> bool {
    const DIRS: [Vec3; 3] = [
        Vec3::new(0.577_215_664_9, 0.367_879_441_2, 0.729_019_996_1),
        Vec3::new(-0.302_775_637_7, 0.841_470_984_8, -0.447_213_595_5),
        Vec3::new(0.235_711_252_1, -0.618_033_988_7, -0.750_120_869_3),
    ];
    let mut hits = Vec::new();
    let votes = DIRS
        .iter()
        .filter(|d| {
            let Some(dir) = Dir3::new(**d, &Tolerance::DEFAULT) else {
                return false;
            };
            hits.clear();
            s.ray_cast_all(&Ray3 { origin: p, dir }, f64::INFINITY, &mut hits);
            let mut n = 0usize;
            let mut last = f64::NEG_INFINITY;
            for h in &hits {
                if h.t - last > 1e-12 * (1.0 + h.t.abs()) {
                    n += 1;
                    last = h.t;
                }
            }
            n % 2 == 1
        })
        .count();
    votes >= 2
}

/// Angle between two entities (note §2.5), using their straight-line direction or best-fit
/// plane. `None` if either has neither.
pub fn angle_between(a: &EntityMeasure, b: &EntityMeasure) -> Option<AngleMeasure> {
    use std::f64::consts::{FRAC_PI_2, PI};
    let atan = |u: Vec3, v: Vec3| u.cross(v).length().atan2(u.dot(v));
    let line = |e: &EntityMeasure| e.direction.map(Dir3::get);
    let plane = |e: &EntityMeasure| e.plane.map(|p| p.plane.normal.get());
    let (kind, angle, supplement) = if let (Some(u), Some(v)) = (line(a), line(b)) {
        let t = atan(u, v);
        let t = t.min(PI - t);
        (AngleKind::LineLine, t, PI - t)
    } else if let (Some(u), Some(v)) = (plane(a), plane(b)) {
        let t = atan(u, v);
        (AngleKind::PlanePlane, t, PI - t)
    } else {
        let (d, n) = match (line(a), plane(b), plane(a), line(b)) {
            (Some(d), Some(n), _, _) | (_, _, Some(n), Some(d)) => (d, n),
            _ => return None,
        };
        // asin(|d·n|) via atan2: the angle between the line and its projection on the plane.
        let t = d.dot(n).abs().atan2(d.cross(n).length());
        (AngleKind::LinePlane, t, FRAC_PI_2 - t)
    };
    Some(AngleMeasure {
        kind,
        angle: Angle::rad(angle),
        supplement: Angle::rad(supplement),
    })
}

/// Measures one operand, or two with the pair results.
///
/// # Errors
/// As [`measure_one`] and [`min_distance`].
pub fn measure(
    a: &Operand,
    b: Option<&Operand>,
    opts: &MeasureOpts,
    ctx: &JobCtx,
) -> Result<MeasureReport, InspectError> {
    let first = measure_one(a, opts, ctx)?;
    let Some(b) = b else {
        return Ok(MeasureReport {
            first,
            second: None,
            pair: None,
        });
    };
    let second = measure_one(b, opts, ctx)?;
    let min_distance = min_distance(a, b, opts, ctx)?;
    let angle = angle_between(&first, &second);
    let center_distance = match (first.center(), second.center()) {
        (Some(p), Some(q)) => p.distance(q),
        _ => min_distance.distance.to_mm(),
    };
    Ok(MeasureReport {
        first,
        second: Some(second),
        pair: Some(PairMeasure {
            min_distance,
            angle,
            center_distance: Length::mm(center_distance),
        }),
    })
}

/// Options of [`grow_planar_region`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RegionOpts {
    /// Largest angle between a face normal and the seed normal, rad (0.5°).
    pub max_normal_angle: f64,
    /// Largest vertex distance from the seed plane, mm (1e-3).
    pub planar_tolerance: f64,
    /// Do not cross face-group borders.
    pub stop_at_group_borders: bool,
    /// Largest region size (5 M).
    pub max_faces: usize,
}

/// Snapping radii (mm at the hit; the tool converts pixels through RND-04's world-per-pixel).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SnapOpts {
    /// Vertex snap radius, mm.
    pub vertex_radius: f64,
    /// Edge snap radius, mm.
    pub edge_radius: f64,
}

/// Result of [`snap_on_face`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Snapped {
    /// Snapped to a vertex.
    Vertex(u32),
    /// Snapped to edge `a → b` at parameter `t ∈ [0, 1]`.
    Edge {
        /// First vertex.
        a: u32,
        /// Second vertex.
        b: u32,
        /// Parameter from `a`.
        t: f64,
    },
    /// On the face.
    Face {
        /// The face.
        face: FaceId,
        /// Barycentric weights of its corners.
        bary: [f64; 3],
    },
}
