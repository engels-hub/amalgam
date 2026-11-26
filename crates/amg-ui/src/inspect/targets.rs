//! Picks on everything but mesh bodies (INS-07): B-rep faces, edges, vertices and bodies,
//! construction geometry, origin entities and sketch entities → measure operands.
//!
//! - **Solids.** The pick service gives the B-rep face or edge under the cursor. A vertex of
//!   it within [`VERTEX_SNAP_PX`] wins; otherwise the dialog's filter decides between the edge,
//!   a point on it, the face and the body. Points are projected onto the exact curve or surface
//!   (closed forms inline, free-form faces in the measure job). B-rep faces are whole already,
//!   so smart selection does not apply to them.
//! - **Construction geometry** is picked through the view (CON-04) and resolved by CON-03 for
//!   the occurrence it was seen in. **Origin** entities are not drawn: the dialog offers them.
//! - **Sketch entities** are not picked by the view: [`sketch_pick`] intersects the pick ray
//!   with each visible sketch plane and takes a point within [`VERTEX_SNAP_PX`], else a curve
//!   within [`EDGE_SNAP_PX`].
//!
//! Everything here is O(1) or O(sketch entities) and needs no kernel query beyond reading the
//! shape's own geometry.

use std::sync::Arc;

use amg_brep::geom::{Curve3, Surface};
use amg_brep::{EdgeId, FaceId, Shape, SubKind, SubShape, VertexId};
use amg_core::{Line3, Point3, Ray3, Segment3, Transform};
use amg_doc::stub::SketchEntityKey;
use amg_doc::{
    BodyId, ConstructionId, Design, EntityRef, Evaluated, OccurrenceId, SketchId, TopoKind,
};
use amg_features::construct::resolve::resolve_in;
use amg_features::construct::{OriginEntity, RefGeom, RefInput, ResolvedRef};
use amg_features::inspect::InspectError;
use amg_features::inspect::measure::{BrepOperand, MeasureOpts, Operand, min_distance};
use amg_jobs::JobCtx;

use super::snap::{EDGE_SNAP_PX, MeasureFilter, Resolved, VERTEX_SNAP_PX, mm_per_px};
use crate::browser::visibility;
use crate::input::CameraView;
use crate::pick::PickTarget;

/// Segments of the highlighted polyline of a sketch curve.
const CURVE_SEGMENTS: u32 = 64;

/// The origin entities the dialog offers, with their labels.
pub const ORIGIN_ENTITIES: [(OriginEntity, &str); 7] = [
    (OriginEntity::Point, "Origin point"),
    (OriginEntity::XAxis, "X axis"),
    (OriginEntity::YAxis, "Y axis"),
    (OriginEntity::ZAxis, "Z axis"),
    (OriginEntity::XYPlane, "XY plane"),
    (OriginEntity::YZPlane, "YZ plane"),
    (OriginEntity::XZPlane, "XZ plane"),
];

/// A solid body instance under a pick.
#[derive(Clone, Debug)]
pub struct SolidHit {
    /// The body.
    pub body: BodyId,
    /// The displayed instance it was picked through.
    pub instance: u32,
    /// Its shape.
    pub shape: Shape,
    /// Shape → world (the instance transform).
    pub to_world: Transform,
}

impl SolidHit {
    fn target(&self, sub: Option<(TopoKind, u32)>) -> PickTarget {
        PickTarget::Body {
            body: self.body,
            instance: self.instance,
            sub,
        }
    }

    fn operand(&self, sub: Option<(SubKind, u32)>) -> Operand {
        Operand::Brep(BrepOperand {
            shape: self.shape.clone(),
            sub: sub.map(|(kind, index)| SubShape { kind, index }),
            to_world: self.to_world,
        })
    }

    fn resolved(&self, operand: Operand, target: PickTarget, at: Point3, label: &str) -> Resolved {
        Resolved {
            operand,
            body: Some((self.body, self.instance)),
            at,
            faces: None,
            chain: Vec::new(),
            target: Some(target),
            label: Some(label.to_owned()),
            marked: true,
        }
    }

    /// The whole body.
    pub fn whole(&self, at: Point3) -> Resolved {
        self.resolved(self.operand(None), self.target(None), at, "solid body")
    }

    /// Face `f`; `None` if it is out of range.
    pub fn face(&self, f: u32, at: Point3) -> Option<Resolved> {
        (f < self.shape.count(SubKind::Face)).then(|| {
            let mut r = self.resolved(
                self.operand(Some((SubKind::Face, f))),
                self.target(Some((TopoKind::Face, f))),
                at,
                "B-rep face",
            );
            r.faces = Some(Arc::from([amg_mesh::FaceId(f)]));
            r
        })
    }

    /// Edge `e`; `None` if it is out of range.
    pub fn edge(&self, e: u32, at: Point3) -> Option<Resolved> {
        (e < self.shape.count(SubKind::Edge)).then(|| {
            self.resolved(
                self.operand(Some((SubKind::Edge, e))),
                self.target(Some((TopoKind::Edge, e))),
                at,
                "B-rep edge",
            )
        })
    }

    /// Vertex `v`; `None` if it is out of range.
    pub fn vertex(&self, v: u32) -> Option<Resolved> {
        (v < self.shape.count(SubKind::Vertex)).then(|| {
            let at = self
                .to_world
                .transform_point3(self.shape.vertex_point(VertexId(v)));
            self.resolved(
                self.operand(Some((SubKind::Vertex, v))),
                self.target(Some((TopoKind::Vertex, v))),
                at,
                "B-rep vertex",
            )
        })
    }

    fn point(&self, at: Point3, sub: (TopoKind, u32)) -> Resolved {
        self.resolved(Operand::Point(at), self.target(Some(sub)), at, "point")
    }
}

/// A free point on a free-form face, to be projected in the measure job.
#[derive(Clone, Debug)]
pub struct SurfacePick {
    solid: SolidHit,
    face: u32,
    /// The hit on the display tessellation (the marker until the job finished).
    pub at: Point3,
}

impl SurfacePick {
    /// The display target.
    pub fn target(&self) -> PickTarget {
        self.solid.target(Some((TopoKind::Face, self.face)))
    }

    /// The point of the face nearest to the hit (a kernel query: run it in a job).
    ///
    /// # Errors
    /// As [`min_distance`].
    pub fn resolve(&self, ctx: &JobCtx) -> Result<Resolved, InspectError> {
        let face = self.solid.operand(Some((SubKind::Face, self.face)));
        let found = min_distance(
            &Operand::Point(self.at),
            &face,
            &MeasureOpts::default(),
            ctx,
        )?;
        Ok(self.solid.point(found.b.point, (TopoKind::Face, self.face)))
    }
}

/// What a pick on a solid selects.
#[derive(Clone, Debug)]
pub enum SolidChoice {
    /// Ready to measure.
    Ready(Box<Resolved>),
    /// A free point on a free-form face.
    Surface(Box<SurfacePick>),
}

/// The point of analytic geometry `g` (shape space) nearest to the world point `p`.
fn project(g: RefGeom, to_world: &Transform, p: Point3) -> Option<Point3> {
    let local = to_world.inverse().transform_point3(p);
    let found = min_distance(
        &Operand::Point(local),
        &Operand::Ref(ResolvedRef::new(g)),
        &MeasureOpts::default(),
        &JobCtx::detached(),
    )
    .ok()?;
    Some(to_world.transform_point3(found.b.point))
}

/// The point on edge `e` nearest to `p`, for lines and circles.
fn point_on_edge(solid: &SolidHit, e: u32, p: Point3) -> Option<Point3> {
    let g = match solid.shape.curve(EdgeId(e))? {
        (Curve3::Line { origin, dir }, t0, t1) => RefGeom::Segment(Segment3 {
            a: origin + dir.get() * t0,
            b: origin + dir.get() * t1,
        }),
        (Curve3::Circle { frame, radius }, ..) => RefGeom::Circle {
            center: frame.origin,
            normal: frame.normal,
            radius,
            arc: None,
        },
        _ => return None,
    };
    project(g, &solid.to_world, p)
}

/// The point on face `f` nearest to `p`, for planes, cylinders, spheres and tori.
fn point_on_face(solid: &SolidHit, f: u32, p: Point3) -> Option<Point3> {
    let axis = |point, dir| Line3 { point, dir };
    let g = match solid.shape.surface(FaceId(f)) {
        Surface::Plane(plane) => RefGeom::Plane(plane),
        Surface::Cylinder { frame, radius } => RefGeom::Cylinder {
            axis: axis(frame.origin, frame.normal),
            radius,
        },
        Surface::Sphere { frame, radius } => RefGeom::Sphere {
            center: frame.origin,
            radius,
        },
        Surface::Torus {
            frame,
            major,
            minor,
        } => RefGeom::Torus {
            axis: axis(frame.origin, frame.normal),
            major,
            minor,
        },
        _ => return None,
    };
    project(g, &solid.to_world, p)
}

/// The vertex of the picked face or edge within the vertex snap radius of `p`.
fn snapped_vertex(
    solid: &SolidHit,
    kind: TopoKind,
    index: u32,
    p: Point3,
    radius: f64,
) -> Option<u32> {
    let shape = &solid.shape;
    let edges: &[EdgeId] = match kind {
        TopoKind::Face => shape.face_edges(FaceId(index)),
        _ => &[EdgeId(index)],
    };
    let count = shape.count(SubKind::Vertex);
    edges
        .iter()
        .flat_map(|&e| shape.edge_vertices(e))
        .filter(|v| v.0 < count)
        .map(|v| {
            let q = solid.to_world.transform_point3(shape.vertex_point(v));
            (q.distance(p), v.0)
        })
        .filter(|(d, _)| *d <= radius)
        .min_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)))
        .map(|(_, v)| v)
}

/// What a hit on the face or edge `(kind, index)` of a solid at the world point `at` selects
/// under `filter`: vertex, edge, point, face, body, in that order. `mm` is the size of a pixel
/// at the hit.
pub fn solid_pick(
    solid: &SolidHit,
    (kind, index): (TopoKind, u32),
    at: Point3,
    mm: f64,
    filter: MeasureFilter,
) -> Option<SolidChoice> {
    let count = match kind {
        TopoKind::Face => solid.shape.count(SubKind::Face),
        TopoKind::Edge => solid.shape.count(SubKind::Edge),
        TopoKind::Vertex => solid.shape.count(SubKind::Vertex),
        TopoKind::FaceGroup => 0,
    };
    if index >= count || filter.is_empty() {
        return None;
    }
    let ready = |r: Resolved| Some(SolidChoice::Ready(Box::new(r)));
    if kind == TopoKind::Vertex {
        return solid
            .vertex(index)
            .filter(|_| filter.vertex || filter.point)
            .and_then(ready);
    }
    if (filter.vertex || filter.point)
        && let Some(v) = snapped_vertex(solid, kind, index, at, VERTEX_SNAP_PX * mm)
        && let Some(r) = solid.vertex(v)
    {
        // A point snaps to the vertex position, as on meshes.
        return ready(if filter.vertex {
            r
        } else {
            solid.point(r.at, (kind, index))
        });
    }
    if kind == TopoKind::Edge {
        let on_edge = point_on_edge(solid, index, at);
        if filter.edge {
            return ready(solid.edge(index, on_edge.unwrap_or(at))?);
        }
        if filter.point
            && let Some(p) = on_edge
        {
            return ready(solid.point(p, (kind, index)));
        }
    }
    // The face under the cursor (an edge's first face).
    let face = match kind {
        TopoKind::Face => index,
        _ => solid.shape.edge_faces(EdgeId(index)).first()?.0,
    };
    if filter.face {
        ready(solid.face(face, at)?)
    } else if filter.point {
        match point_on_face(solid, face, at) {
            Some(p) => ready(solid.point(p, (TopoKind::Face, face))),
            None => Some(SolidChoice::Surface(Box::new(SurfacePick {
                solid: solid.clone(),
                face,
                at,
            }))),
        }
    } else if filter.body {
        ready(solid.whole(at))
    } else {
        None
    }
}

/// A resolved reference as a selection.
fn reference(r: ResolvedRef, target: Option<PickTarget>, at: Point3, label: &str) -> Resolved {
    Resolved {
        operand: Operand::Ref(r),
        body: None,
        at,
        faces: None,
        chain: Vec::new(),
        target,
        label: Some(label.to_owned()),
        marked: false,
    }
}

/// Every occurrence path from the root to `component` whose occurrences are all visible.
fn visible_paths(design: &Design, component: amg_doc::ComponentId) -> Vec<Vec<OccurrenceId>> {
    design
        .occurrence_paths(component)
        .into_iter()
        .filter(|path| {
            path.iter()
                .all(|o| design.occurrence(*o).is_some_and(|o| o.visible))
        })
        .collect()
}

/// `e` seen through `path`.
fn through(path: Vec<OccurrenceId>, e: EntityRef) -> EntityRef {
    if path.is_empty() {
        e
    } else {
        EntityRef::InContext {
            path,
            target: Box::new(e),
        }
    }
}

/// Construction geometry `id` as seen in the occurrence nearest to the picked point `at`
/// (`None`: its first visible occurrence).
pub fn construction_pick(
    design: &Design,
    eval: &Evaluated,
    id: ConstructionId,
    at: Option<Point3>,
) -> Option<Resolved> {
    let (_, node) = design.timeline().node(id.0)?;
    let mut best: Option<(f64, ResolvedRef)> = None;
    for path in visible_paths(design, node.context) {
        let input = RefInput::Ref(through(path, EntityRef::Construction(id)));
        let Ok(r) = resolve_in(eval, design, &input) else {
            continue;
        };
        let d = at.map_or(0.0, |p| {
            min_distance(
                &Operand::Point(p),
                &Operand::Ref(r.clone()),
                &MeasureOpts::default(),
                &JobCtx::detached(),
            )
            .map_or(f64::INFINITY, |d| d.distance.to_mm())
        });
        if best.as_ref().is_none_or(|b| d < b.0) {
            best = Some((d, r));
        }
    }
    let (_, r) = best?;
    let (at, label, marked) = match (&r.geom, at) {
        (RefGeom::Point(p), _) => (*p, "construction point", true),
        (RefGeom::Line(l), at) => (at.unwrap_or(l.point), "construction axis", at.is_some()),
        (RefGeom::Plane(p), at) => (at.unwrap_or(p.origin), "construction plane", at.is_some()),
        (_, at) => (at.unwrap_or(Point3::ZERO), "construction geometry", false),
    };
    let mut resolved = reference(r, Some(PickTarget::Construction(id)), at, label);
    resolved.marked = marked;
    Some(resolved)
}

/// An origin entity of the root component.
pub fn origin_pick(design: &Design, eval: &Evaluated, e: OriginEntity) -> Option<Resolved> {
    let r = resolve_in(eval, design, &RefInput::Origin(e)).ok()?;
    let label = ORIGIN_ENTITIES.iter().find(|(o, _)| *o == e)?.1;
    let at = Point3::ZERO;
    let mut resolved = reference(r, None, at, label);
    resolved.marked = e == OriginEntity::Point;
    Some(resolved)
}

/// The best sketch entity under the ray so far.
struct SketchFound {
    /// 0: a point, 1: a curve.
    class: u8,
    /// Distance along the ray to the sketch plane.
    t: f64,
    /// Distance from the ray's plane hit to the entity.
    off: f64,
    sketch: SketchId,
    path: Vec<OccurrenceId>,
    entity: u32,
}

/// The sketch point or curve under `ray`, with the distance along the ray to its sketch plane.
/// Points (`points`) win over curves (`curves`); among several sketches the nearest plane wins.
pub fn sketch_pick(
    design: &Design,
    eval: &Evaluated,
    view: &CameraView,
    ray: &Ray3,
    points: bool,
    curves: bool,
) -> Option<(f64, Resolved)> {
    let mut hidden = None;
    let mut best: Option<SketchFound> = None;
    for (id, geom) in eval.sketches() {
        let hidden = hidden.get_or_insert_with(|| visibility::hidden(design));
        if hidden.contains(&visibility::sketch_key(id)) {
            continue;
        }
        let Some((_, node)) = design.timeline().node(id.0) else {
            continue;
        };
        for path in visible_paths(design, node.context) {
            let to_world = if path.is_empty() {
                Transform::IDENTITY
            } else {
                match eval.world_transform(&path) {
                    Some(t) => t,
                    None => continue,
                }
            };
            // The ray in the sketch's component space.
            let back = to_world.inverse();
            let origin = back.transform_point3(ray.origin);
            let dir = back.transform_vector3(ray.dir.get());
            let denom = dir.dot(geom.plane.normal.get());
            if denom.abs() < 1e-9 {
                continue;
            }
            let t = (geom.plane.origin - origin).dot(geom.plane.normal.get()) / denom;
            if t < 0.0 {
                continue;
            }
            let hit = origin + dir * t;
            let uv = geom.plane.to_local(hit);
            let mm = mm_per_px(view, to_world.transform_point3(hit));
            for (entity, _) in geom.sketch.entities() {
                let found = if let Some(p) = geom.sketch.point(entity) {
                    let d = p.distance(uv);
                    (points && d <= VERTEX_SNAP_PX * mm).then_some((0u8, d))
                } else if let Some(c) = geom.sketch.curve(entity) {
                    let d = c.closest(uv).1.distance(uv);
                    (curves && d <= EDGE_SNAP_PX * mm).then_some((1, d))
                } else {
                    None
                };
                let Some((class, d)) = found else {
                    continue;
                };
                if best.as_ref().is_none_or(|b| {
                    class
                        .cmp(&b.class)
                        .then(t.total_cmp(&b.t))
                        .then(d.total_cmp(&b.off))
                        .is_lt()
                }) {
                    best = Some(SketchFound {
                        class,
                        t,
                        off: d,
                        sketch: id,
                        path: path.clone(),
                        entity: entity.0,
                    });
                }
            }
        }
    }
    let SketchFound {
        class,
        t,
        sketch,
        path,
        entity,
        ..
    } = best?;
    let key = SketchEntityKey(u64::from(entity));
    let target = through(
        path.clone(),
        if class == 0 {
            EntityRef::SketchPoint { sketch, point: key }
        } else {
            EntityRef::SketchCurve { sketch, curve: key }
        },
    );
    let r = resolve_in(eval, design, &RefInput::Ref(target)).ok()?;
    let g = eval.sketch(sketch)?;
    let to_world = if path.is_empty() {
        Transform::IDENTITY
    } else {
        eval.world_transform(&path)?
    };
    let world = |uv| to_world.transform_point3(g.plane.to_world(uv));
    let id = amg_sketch::EntityId(entity);
    let (at, chain) = match g.sketch.curve(id) {
        Some(c) => {
            let (t0, t1) = c.param_range();
            let chain = (0..=CURVE_SEGMENTS)
                .map(|i| world(c.eval(t0 + (t1 - t0) * f64::from(i) / f64::from(CURVE_SEGMENTS))))
                .collect();
            let local = to_world.inverse();
            let hit = local.transform_point3(ray.origin + ray.dir.get() * t);
            (world(c.closest(g.plane.to_local(hit)).1), chain)
        }
        None => (world(g.sketch.point(id)?), Vec::new()),
    };
    let label = if class == 0 {
        "sketch point"
    } else {
        "sketch curve"
    };
    let mut resolved = reference(r, Some(PickTarget::Sketch(sketch)), at, label);
    resolved.chain = chain;
    resolved.marked = true;
    Some((t, resolved))
}
