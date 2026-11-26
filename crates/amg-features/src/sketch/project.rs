//! Projected geometry (S03 SK-09): resolving a [`ProjectionLink`]'s source, computing it in
//! sketch coordinates and refreshing the link's locked entities at recompute.
//!
//! | Source | `Project` / `Include` | `Intersect` | `IntersectionCurve` |
//! |---|---|---|---|
//! | B-rep edge | its projection | its points on the plane | |
//! | B-rep face | the projection of every edge | | its section |
//! | body | | | its section |
//! | B-rep vertex, sketch point, construction point | the point | the point, if on the plane | |
//! | sketch curve (earlier sketch) | its projection | its points on the plane | |
//! | construction axis | | its point on the plane | |
//!
//! `Include` is `Project` for a source that already lies in the sketch plane. References go
//! through persistent naming (DOC-06), so a doubtful resolution is a warning on the sketch.
//! Solid sources and curves other than lines need the geometry kernel.
//!
//! **Matching.** A link's entities correspond one to one, in order, to the pieces of its
//! source: the edges of a face in the order of their persistent names, section curves in the
//! order of their faces' names. A refresh moves the entities and never adds or removes any; if
//! the number or kind of the pieces changes, the link keeps its last geometry and warns.

use std::f64::consts::{FRAC_PI_2, TAU};

use amg_brep::geom::{BSplineCurveData, Curve3, CurveSeg, Tag};
use amg_brep::{EdgeId, FaceId, Shape, SubKind, VertexId};
use amg_core::{Dir3, Plane, Point2, Point3, Transform, Vec2, Vec3};
use amg_doc::naming::{ResolveError, TopoKind, TopoName, TopoRef};
use amg_doc::{
    BodyGeom, BodyId, ComputeMode, ConstructionGeom, Design, Diagnostic, EntityRef, ErrorCause,
    Evaluated, FeatureError, FeatureId, RecomputeCtx, SketchId,
};
use amg_sketch::{BSpline2, Curve2, EntityId, ExternalLinkId, ProjectedGeom, Sketch};
use std::collections::BTreeMap;

use super::{ProjectionLink, ProjectionMode, entity_ref};

/// Smallest distance from the sketch plane at which geometry still lies in it, mm.
const ON_PLANE: f64 = 1e-6;

/// How far an occurrence transform may stray from a rotation plus translation.
const RIGID_EPS: f64 = 1e-9;

/// Samples per knot span when looking for the points of a B-spline on the plane.
const SAMPLES_PER_SPAN: u32 = 64;

/// Why a source could not be projected.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum ProjectionError {
    /// A topological reference did not resolve.
    #[error(transparent)]
    Resolve(#[from] ResolveError),
    /// The referenced body, sketch, entity, construction or occurrence is not there.
    #[error("{0} is not available")]
    Missing(String),
    /// The mode does not apply to this kind of source.
    #[error("{mode:?} does not apply to {source_kind}")]
    NotApplicable {
        /// The requested mode.
        mode: ProjectionMode,
        /// What the source is.
        source_kind: &'static str,
    },
    /// A source or curve that can't be projected.
    #[error("{0} can't be projected")]
    Unsupported(&'static str),
    /// The geometry kernel failed (or is not available in this build).
    #[error("{0}")]
    Kernel(String),
    /// The projection or intersection has no geometry.
    #[error("{0}")]
    Empty(&'static str),
    /// `Include` of geometry away from the sketch plane.
    #[error("the source is {0:.4} mm off the sketch plane; use Project")]
    NotInPlane(f64),
    /// The source no longer gives the pieces the link's entities stand for.
    #[error("the source changed shape ({0}); project it again")]
    ShapeChanged(String),
}

fn kernel_error(e: impl std::fmt::Display) -> ProjectionError {
    ProjectionError::Kernel(e.to_string())
}

type Pieces = Vec<ProjectedGeom>;

/// What `source` gives in a sketch on `plane`, computed against `before`, the evaluated state
/// the sketch feature `sketch` reads: the geometry for
/// [`SketchEdit::AddProjection`](super::SketchEdit::AddProjection). For tools; recompute does
/// the same for every link.
///
/// # Errors
/// [`ProjectionError`].
pub fn project_reference(
    design: &Design,
    before: &Evaluated,
    sketch: FeatureId,
    plane: &Plane,
    source: &EntityRef,
    mode: ProjectionMode,
) -> Result<Vec<ProjectedGeom>, ProjectionError> {
    let context = design
        .timeline()
        .node(sketch)
        .map_or_else(|| design.root(), |(_, n)| n.context);
    let ctx = RecomputeCtx::new(design, before, sketch, context, ComputeMode::Full);
    geometry(&ctx, plane, source, mode)
}

/// Refreshes the entities of every link in `sketch` (already on `plane`). A link that fails
/// keeps its geometry and gives one warning.
pub(super) fn refresh(
    ctx: &RecomputeCtx<'_>,
    plane: &Plane,
    sketch: &mut Sketch,
    links: &BTreeMap<ExternalLinkId, ProjectionLink>,
) -> Result<Vec<Diagnostic>, FeatureError> {
    let mut warnings = Vec::new();
    let sketch_id = SketchId(ctx.feature_id());
    for (id, link) in links {
        ctx.check_cancelled()?;
        let result = geometry(ctx, plane, &link.source, link.mode)
            .and_then(|pieces| apply(sketch, link, &pieces));
        if let Err(e) = result {
            let mut entities = vec![link.source.clone()];
            entities.extend(
                link.entities
                    .iter()
                    .filter(|&&e| sketch.entity(e).is_some())
                    .map(|&e| entity_ref(sketch_id, sketch, e)),
            );
            warnings.push(Diagnostic {
                message: format!(
                    "projection {} was not refreshed: {e}; its last geometry is kept",
                    id.0
                ),
                cause: match e {
                    ProjectionError::Resolve(r) => Some(ErrorCause::Resolve(r)),
                    _ => None,
                },
                entities,
            });
        }
    }
    Ok(warnings)
}

/// Writes `pieces` into the link's entities; removed entities keep their place in the list.
fn apply(
    sketch: &mut Sketch,
    link: &ProjectionLink,
    pieces: &[ProjectedGeom],
) -> Result<(), ProjectionError> {
    if pieces.len() != link.entities.len() {
        return Err(ProjectionError::ShapeChanged(format!(
            "{} piece(s) instead of {}",
            pieces.len(),
            link.entities.len()
        )));
    }
    let items: Vec<(EntityId, &ProjectedGeom)> = link
        .entities
        .iter()
        .copied()
        .zip(pieces)
        .filter(|(e, _)| sketch.entity(*e).is_some())
        .collect();
    sketch
        .refresh_projected(&items)
        .map_err(|e| ProjectionError::ShapeChanged(e.to_string()))
}

/// The pieces of `source` in sketch coordinates.
fn geometry(
    ctx: &RecomputeCtx<'_>,
    plane: &Plane,
    source: &EntityRef,
    mode: ProjectionMode,
) -> Result<Pieces, ProjectionError> {
    let target = Target { ctx, plane, mode };
    target.entity(source, &mut Vec::new())
}

/// One projection request.
struct Target<'c, 'a> {
    ctx: &'c RecomputeCtx<'a>,
    plane: &'c Plane,
    mode: ProjectionMode,
}

impl Target<'_, '_> {
    fn not_applicable(&self, source_kind: &'static str) -> ProjectionError {
        ProjectionError::NotApplicable {
            mode: self.mode,
            source_kind,
        }
    }

    fn on_plane_tol(&self) -> f64 {
        ON_PLANE.max(10.0 * self.ctx.tolerance().linear)
    }

    fn height(&self, p: Point3) -> f64 {
        (p - self.plane.origin).dot(self.plane.normal.get())
    }

    fn entity(
        &self,
        source: &EntityRef,
        path: &mut Vec<amg_doc::OccurrenceId>,
    ) -> Result<Pieces, ProjectionError> {
        match source {
            EntityRef::InContext {
                path: below,
                target,
            } => {
                path.extend_from_slice(below);
                self.entity(target, path)
            }
            EntityRef::Topo(r) => self.topo(r, &self.placement(path)?),
            EntityRef::Body(b) => {
                if self.mode != ProjectionMode::IntersectionCurve {
                    return Err(self.not_applicable("a body"));
                }
                let shape = moved(self.solid(*b)?, &self.placement(path)?)?;
                self.section(*b, &shape, None)
            }
            EntityRef::SketchCurve { sketch, curve } => {
                let (plane, geom) = self.sketch(*sketch, &self.placement(path)?)?;
                let curve = u32::try_from(curve.0)
                    .ok()
                    .and_then(|e| geom.curve(EntityId(e)))
                    .ok_or_else(|| ProjectionError::Missing(format!("sketch curve {}", curve.0)))?;
                self.sketch_curve(&plane, &curve)
            }
            EntityRef::SketchPoint { sketch, point } => {
                let (plane, geom) = self.sketch(*sketch, &self.placement(path)?)?;
                let p = u32::try_from(point.0)
                    .ok()
                    .and_then(|e| geom.point(EntityId(e)))
                    .ok_or_else(|| ProjectionError::Missing(format!("sketch point {}", point.0)))?;
                self.point(plane.to_world(p))
            }
            EntityRef::Construction(c) => {
                let t = self.placement(path)?;
                let geom = self.ctx.construction(*c).map_err(|_| {
                    ProjectionError::Missing(format!("construction geometry of {}", c.0))
                })?;
                match geom {
                    ConstructionGeom::Point(p) => self.point(t.transform_point3(*p)),
                    ConstructionGeom::Axis(l) => {
                        if self.mode != ProjectionMode::Intersect {
                            return Err(self.not_applicable("a construction axis"));
                        }
                        let origin = t.transform_point3(l.point);
                        let dir = t.transform_vector3(l.dir.get());
                        let rate = dir.dot(self.plane.normal.get());
                        if rate.abs() <= self.ctx.tolerance().angular {
                            return Err(ProjectionError::Empty(
                                "the axis does not cross the sketch plane",
                            ));
                        }
                        let p = origin - dir * (self.height(origin) / rate);
                        Ok(vec![ProjectedGeom::Point(self.plane.to_local(p))])
                    }
                    ConstructionGeom::Plane(_) => {
                        Err(ProjectionError::Unsupported("a construction plane"))
                    }
                }
            }
            EntityRef::Profile { .. } => Err(ProjectionError::Unsupported("a profile")),
        }
    }

    /// The rigid transform of the occurrence path below the sketch's component.
    fn placement(&self, path: &[amg_doc::OccurrenceId]) -> Result<Transform, ProjectionError> {
        if path.is_empty() {
            return Ok(Transform::IDENTITY);
        }
        let missing = || ProjectionError::Missing("the occurrence path".to_owned());
        if !self.ctx.design().is_valid_path(self.ctx.context(), path) {
            return Err(missing());
        }
        let t = path
            .iter()
            .try_fold(Transform::IDENTITY, |acc, o| {
                Some(acc * self.ctx.occurrence_transform(*o)?)
            })
            .ok_or_else(missing)?;
        let m = t.matrix3;
        let rigid = (m.transpose() * m).abs_diff_eq(amg_core::Mat3::IDENTITY, RIGID_EPS)
            && (m.determinant() - 1.0).abs() <= RIGID_EPS
            && t.translation.is_finite();
        if rigid {
            Ok(t)
        } else {
            Err(ProjectionError::Unsupported(
                "geometry placed by a non-rigid occurrence",
            ))
        }
    }

    /// A sketch's plane (placed by `t`) and solved geometry.
    fn sketch(&self, id: SketchId, t: &Transform) -> Result<(Plane, &Sketch), ProjectionError> {
        let geom = self
            .ctx
            .before()
            .sketch(id)
            .ok_or_else(|| ProjectionError::Missing(format!("sketch {}", id.0)))?;
        let p = geom.plane;
        let turned = |d: Dir3| Dir3::new_unchecked(t.transform_vector3(d.get()).normalize());
        let plane = if *t == Transform::IDENTITY {
            p
        } else {
            Plane {
                origin: t.transform_point3(p.origin),
                x: turned(p.x),
                y: turned(p.y),
                normal: turned(p.normal),
            }
        };
        Ok((plane, &geom.sketch))
    }

    fn solid(&self, id: BodyId) -> Result<&Shape, ProjectionError> {
        match self.ctx.before().body(id).map(|b| &b.geom) {
            Some(BodyGeom::Solid(solid)) => solid.brep().ok_or(ProjectionError::Unsupported(
                "a body without B-rep geometry",
            )),
            Some(_) => Err(ProjectionError::Unsupported("a body that is not a solid")),
            None => Err(ProjectionError::Missing(format!(
                "body {} of {}",
                id.slot, id.origin
            ))),
        }
    }

    fn topo(&self, r: &TopoRef, t: &Transform) -> Result<Pieces, ProjectionError> {
        let found = self.ctx.resolve(r)?;
        let kind = found
            .kind
            .sub_kind()
            .ok_or(ProjectionError::Unsupported("a mesh face group"))?;
        let shape = moved(self.solid(found.body)?, t)?;
        if found.index >= shape.count(kind) {
            return Err(ProjectionError::Missing(format!(
                "{kind:?} {}",
                found.index
            )));
        }
        match kind {
            SubKind::Vertex => self.point(shape.vertex_point(VertexId(found.index))),
            SubKind::Edge => {
                let e = EdgeId(found.index);
                match self.mode {
                    ProjectionMode::Project => self.edge(&shape, e),
                    ProjectionMode::Include => {
                        self.require_edge_in_plane(&shape, e)?;
                        self.edge(&shape, e)
                    }
                    ProjectionMode::Intersect => {
                        let (curve, t0, t1) = shape
                            .curve(e)
                            .ok_or(ProjectionError::Unsupported("a degenerate edge"))?;
                        self.intersect(&CurveSeg {
                            curve,
                            t0,
                            t1,
                            tag: Tag(0),
                        })
                    }
                    ProjectionMode::IntersectionCurve => Err(self.not_applicable("an edge")),
                }
            }
            SubKind::Face => {
                let f = FaceId(found.index);
                match self.mode {
                    ProjectionMode::Project => self.face(found.body, &shape, f),
                    ProjectionMode::Include => {
                        for e in face_edges(&shape, f) {
                            self.require_edge_in_plane(&shape, e)?;
                        }
                        self.face(found.body, &shape, f)
                    }
                    ProjectionMode::IntersectionCurve => self.section(found.body, &shape, Some(f)),
                    ProjectionMode::Intersect => Err(self.not_applicable("a face")),
                }
            }
        }
    }

    /// A point source.
    fn point(&self, p: Point3) -> Result<Pieces, ProjectionError> {
        let off = self.height(p).abs();
        match self.mode {
            ProjectionMode::Project => {}
            ProjectionMode::Include if off > self.on_plane_tol() => {
                return Err(ProjectionError::NotInPlane(off));
            }
            ProjectionMode::Intersect if off > self.on_plane_tol() => {
                return Err(ProjectionError::Empty(
                    "the point is not on the sketch plane",
                ));
            }
            ProjectionMode::Include | ProjectionMode::Intersect => {}
            ProjectionMode::IntersectionCurve => return Err(self.not_applicable("a point")),
        }
        Ok(vec![ProjectedGeom::Point(self.plane.to_local(p))])
    }

    /// The projection of one edge: a point if the whole edge collapses to one.
    fn edge(&self, shape: &Shape, e: EdgeId) -> Result<Pieces, ProjectionError> {
        let pieces = self.edge_pieces(shape, e)?;
        if !pieces.is_empty() {
            return Ok(pieces);
        }
        if shape.curve(e).is_none() {
            return Err(ProjectionError::Empty("the edge is degenerate"));
        }
        let [v, _] = shape.edge_vertices(e);
        Ok(vec![ProjectedGeom::Point(
            self.plane.to_local(shape.vertex_point(v)),
        )])
    }

    fn edge_pieces(&self, shape: &Shape, e: EdgeId) -> Result<Pieces, ProjectionError> {
        self.ctx
            .kernel()
            .project_edges(shape, &[e], self.plane)
            .map_err(kernel_error)?
            .iter()
            .map(|s| self.local(s))
            .collect()
    }

    /// The projection of every edge of a face, in the order of the edges' persistent names.
    /// Edges that collapse to a point add nothing.
    fn face(&self, body: BodyId, shape: &Shape, f: FaceId) -> Result<Pieces, ProjectionError> {
        let naming = self.ctx.naming(body);
        let mut edges: Vec<(Option<&TopoName>, EdgeId)> = face_edges(shape, f)
            .into_iter()
            .map(|e| (naming.and_then(|n| n.name_of(TopoKind::Edge, e.0)), e))
            .collect();
        edges.sort_by(|a, b| a.0.cmp(&b.0));
        let mut out = Vec::new();
        for (_, e) in edges {
            out.extend(self.edge_pieces(shape, e)?);
        }
        if out.is_empty() {
            return Err(ProjectionError::Empty("the face projects to nothing"));
        }
        Ok(out)
    }

    /// The section of `shape` (or of its face `only`) with the sketch plane, ordered by the
    /// faces' persistent names, then by position.
    fn section(
        &self,
        body: BodyId,
        shape: &Shape,
        only: Option<FaceId>,
    ) -> Result<Pieces, ProjectionError> {
        let naming = self.ctx.naming(body);
        let segs = self
            .ctx
            .kernel()
            .plane_section(shape, self.plane)
            .map_err(kernel_error)?;
        let mut keyed = Vec::new();
        for s in segs {
            if only.is_some_and(|f| s.tag != Tag(u64::from(f.0))) {
                continue;
            }
            let name = u32::try_from(s.tag.0)
                .ok()
                .and_then(|f| naming?.name_of(TopoKind::Face, f));
            let mid = s
                .curve
                .point_at(0.5 * (s.t0 + s.t1))
                .map_or(Point2::ZERO, |p| self.plane.to_local(p));
            keyed.push((name, mid, self.local(&s)?));
        }
        if keyed.is_empty() {
            return Err(ProjectionError::Empty("the sketch plane does not cut it"));
        }
        keyed.sort_by(|a, b| {
            a.0.cmp(&b.0)
                .then(a.1.x.total_cmp(&b.1.x))
                .then(a.1.y.total_cmp(&b.1.y))
        });
        Ok(keyed.into_iter().map(|(_, _, g)| g).collect())
    }

    fn require_edge_in_plane(&self, shape: &Shape, e: EdgeId) -> Result<(), ProjectionError> {
        let Some((curve, t0, t1)) = shape.curve(e) else {
            return Ok(()); // degenerate: its vertex is on a neighbouring edge
        };
        let seg = CurveSeg {
            curve,
            t0,
            t1,
            tag: Tag(0),
        };
        self.require_in_plane(&seg)
    }

    fn require_in_plane(&self, seg: &CurveSeg) -> Result<(), ProjectionError> {
        let mut off: f64 = 0.0;
        for i in 0..=SAMPLES_PER_SPAN {
            let t = seg.t0 + (seg.t1 - seg.t0) * f64::from(i) / f64::from(SAMPLES_PER_SPAN);
            let p = seg
                .curve
                .point_at(t)
                .ok_or(ProjectionError::Unsupported("this special curve"))?;
            off = off.max(self.height(p).abs());
        }
        if off > self.on_plane_tol() {
            Err(ProjectionError::NotInPlane(off))
        } else {
            Ok(())
        }
    }

    /// A curve of an earlier sketch on `plane`. Lines need no kernel.
    fn sketch_curve(&self, plane: &Plane, curve: &Curve2) -> Result<Pieces, ProjectionError> {
        if self.mode == ProjectionMode::IntersectionCurve {
            return Err(self.not_applicable("a sketch curve"));
        }
        if let Curve2::Line(s) = curve {
            let (a, b) = (plane.to_world(s.a), plane.to_world(s.b));
            if self.mode == ProjectionMode::Intersect {
                let length = a.distance(b);
                let Some(dir) = (length > 0.0).then(|| Dir3::new_unchecked((b - a) / length))
                else {
                    return self.point(a);
                };
                return self.intersect(&CurveSeg {
                    curve: Curve3::Line { origin: a, dir },
                    t0: 0.0,
                    t1: length,
                    tag: Tag(0),
                });
            }
            if self.mode == ProjectionMode::Include {
                let off = self.height(a).abs().max(self.height(b).abs());
                if off > self.on_plane_tol() {
                    return Err(ProjectionError::NotInPlane(off));
                }
            }
            let (a, b) = (self.plane.to_local(a), self.plane.to_local(b));
            return Ok(vec![if a.distance(b) <= self.ctx.tolerance().linear {
                ProjectedGeom::Point(a)
            } else {
                ProjectedGeom::Line { a, b }
            }]);
        }
        let seg = world_seg(plane, curve)?;
        if self.mode == ProjectionMode::Intersect {
            return self.intersect(&seg);
        }
        if self.mode == ProjectionMode::Include {
            self.require_in_plane(&seg)?;
        }
        let wire = self
            .ctx
            .kernel()
            .make_wire(std::slice::from_ref(&seg), self.ctx.tolerance())
            .map_err(kernel_error)?;
        self.edge(&wire.shape, EdgeId(0))
    }

    /// A trimmed curve lying in the sketch plane, in sketch coordinates.
    fn local(&self, seg: &CurveSeg) -> Result<ProjectedGeom, ProjectionError> {
        let at = |p: Point3| self.plane.to_local(p);
        let direction = |d: Dir3| at(self.plane.origin + d.get());
        // +1 if the curve's frame turns the same way as the sketch plane.
        let turn = |n: Dir3| {
            if n.get().dot(self.plane.normal.get()) < 0.0 {
                -1.0
            } else {
                1.0
            }
        };
        let end = |t: f64| {
            seg.curve
                .point_at(t)
                .map(at)
                .ok_or(ProjectionError::Unsupported("this special curve"))
        };
        Ok(match &seg.curve {
            Curve3::Line { .. } => ProjectedGeom::Line {
                a: end(seg.t0)?,
                b: end(seg.t1)?,
            },
            Curve3::Circle { frame, radius } => {
                let center = at(frame.origin);
                let from = end(seg.t0)? - center;
                let sweep = turn(frame.normal) * (seg.t1 - seg.t0);
                if sweep.abs() >= TAU - self.ctx.tolerance().angular {
                    ProjectedGeom::Circle {
                        center,
                        radius: *radius,
                    }
                } else {
                    ProjectedGeom::Arc {
                        center,
                        radius: *radius,
                        start_angle: from.y.atan2(from.x),
                        sweep,
                    }
                }
            }
            Curve3::Ellipse {
                frame,
                major_radius,
                minor_radius,
            } => {
                let sign = turn(frame.normal);
                let (t0, mut t1) = (sign * seg.t0, sign * seg.t1);
                if (t1 - t0).abs() >= TAU - self.ctx.tolerance().angular {
                    t1 = t0 + TAU.copysign(t1 - t0);
                }
                ProjectedGeom::Ellipse {
                    center: at(frame.origin),
                    major: direction(frame.x).normalize() * *major_radius,
                    minor_radius: *minor_radius,
                    t0,
                    t1,
                }
            }
            Curve3::BSpline(data) => spline(self.plane, data, seg.t0, seg.t1)?,
            Curve3::Other => return Err(ProjectionError::Unsupported("this special curve")),
        })
    }

    /// The points of a curve on the sketch plane, in order along it.
    fn intersect(&self, seg: &CurveSeg) -> Result<Pieces, ProjectionError> {
        let tol = self.on_plane_tol();
        let in_plane = || ProjectionError::Empty("the curve lies in the sketch plane; use Include");
        let (lo, hi) = (seg.t0.min(seg.t1), seg.t0.max(seg.t1));
        let point = |t: f64| {
            seg.curve
                .point_at(t)
                .ok_or(ProjectionError::Unsupported("this special curve"))
        };
        let n = self.plane.normal.get();
        let mut roots = match &seg.curve {
            Curve3::Line { .. } => {
                let (f0, f1) = (self.height(point(lo)?), self.height(point(hi)?));
                if f0.abs() <= tol && f1.abs() <= tol {
                    return Err(in_plane());
                }
                if f0.abs() <= tol {
                    vec![lo]
                } else if f1.abs() <= tol {
                    vec![hi]
                } else if f0 * f1 < 0.0 {
                    vec![lo + (hi - lo) * f0 / (f0 - f1)]
                } else {
                    Vec::new()
                }
            }
            Curve3::Circle { frame, radius } => {
                let a = frame.x.get().dot(n) * radius;
                let b = frame.y.get().dot(n) * radius;
                trig_roots(self.height(frame.origin), a, b, (lo, hi), tol).ok_or_else(in_plane)?
            }
            Curve3::Ellipse {
                frame,
                major_radius,
                minor_radius,
            } => {
                let a = frame.x.get().dot(n) * major_radius;
                let b = frame.y.get().dot(n) * minor_radius;
                trig_roots(self.height(frame.origin), a, b, (lo, hi), tol).ok_or_else(in_plane)?
            }
            Curve3::BSpline(data) => {
                let spans = u32::try_from(data.knots.len().saturating_sub(1)).unwrap_or(1);
                let f = |t: f64| point(t).map(|p| self.height(p));
                sampled_roots(f, (lo, hi), spans.max(1) * SAMPLES_PER_SPAN, tol)?
                    .ok_or_else(in_plane)?
            }
            Curve3::Other => return Err(ProjectionError::Unsupported("this special curve")),
        };
        roots.sort_by(f64::total_cmp);
        if seg.t1 < seg.t0 {
            roots.reverse();
        }
        let mut points: Vec<Point3> = Vec::new();
        for t in roots {
            let p = point(t)?;
            // A closed curve meets its start again at its end.
            if points.iter().all(|q| q.distance(p) > 10.0 * tol) {
                points.push(p);
            }
        }
        if points.is_empty() {
            return Err(ProjectionError::Empty(
                "the curve does not meet the sketch plane",
            ));
        }
        Ok(points
            .into_iter()
            .map(|p| ProjectedGeom::Point(self.plane.to_local(p)))
            .collect())
    }
}

/// `shape` placed by the rigid transform `t` (sub-shape indices are kept).
fn moved(shape: &Shape, t: &Transform) -> Result<Shape, ProjectionError> {
    if *t == Transform::IDENTITY {
        Ok(shape.clone())
    } else {
        shape.transformed(t).map_err(kernel_error)
    }
}

/// The edges of a face, wire by wire, each once (a seam appears twice in its wire).
fn face_edges(shape: &Shape, f: FaceId) -> Vec<EdgeId> {
    let mut out: Vec<EdgeId> = Vec::new();
    for e in shape.face_wires(f).into_iter().flatten() {
        if !out.contains(&e) {
            out.push(e);
        }
    }
    out
}

/// The parameters in `[lo, hi]` where `h + a·cos t + b·sin t = 0`; `None` if it is zero
/// everywhere (within `tol`).
fn trig_roots(h: f64, a: f64, b: f64, (lo, hi): (f64, f64), tol: f64) -> Option<Vec<f64>> {
    let r = a.hypot(b);
    if r <= tol {
        return (h.abs() > tol).then(Vec::new);
    }
    if h.abs() > r + tol {
        return Some(Vec::new());
    }
    let phase = b.atan2(a);
    let candidates = if h.abs() >= r - tol {
        // Touching: one point, at the end of the amplitude opposite to `h`.
        vec![if h > 0.0 {
            phase + 2.0 * FRAC_PI_2
        } else {
            phase
        }]
    } else {
        let d = (-h / r).acos();
        vec![phase - d, phase + d]
    };
    let eps = tol / r;
    Some(
        candidates
            .into_iter()
            .filter_map(|t| {
                // The first turn of `t` at or after the start of the range.
                let t = lo - eps + (t - lo + eps).rem_euclid(TAU);
                (t <= hi + eps).then(|| t.clamp(lo, hi))
            })
            .collect(),
    )
}

/// The roots of `f` on `[lo, hi]` from `n` samples: sign changes (bisected) and touching
/// minima. `None` if every sample is within `tol` of zero.
fn sampled_roots(
    f: impl Fn(f64) -> Result<f64, ProjectionError>,
    (lo, hi): (f64, f64),
    n: u32,
    tol: f64,
) -> Result<Option<Vec<f64>>, ProjectionError> {
    let at = |i: u32| lo + (hi - lo) * f64::from(i) / f64::from(n);
    let values = (0..=n).map(|i| f(at(i))).collect::<Result<Vec<_>, _>>()?;
    if values.iter().all(|v| v.abs() <= tol) {
        return Ok(None);
    }
    let mut roots = Vec::new();
    for i in 0..=n {
        let k = i as usize;
        let v = values[k];
        let prev = k.checked_sub(1).map(|j| values[j]);
        let next = values.get(k + 1).copied();
        // A crossing between this sample and the next.
        if let Some(w) = next
            && v * w < 0.0
        {
            let (mut a, mut b, mut fa) = (at(i), at(i + 1), v);
            for _ in 0..80 {
                let m = 0.5 * (a + b);
                let fm = f(m)?;
                if fa * fm <= 0.0 {
                    b = m;
                } else {
                    (a, fa) = (m, fm);
                }
            }
            roots.push(0.5 * (a + b));
            continue;
        }
        // A sample nearest to the plane among its neighbours, without a crossing: refine the
        // minimum of |f| and keep it if the curve touches the plane there.
        let nearest = prev.is_none_or(|p| v.abs() <= p.abs() && v * p >= 0.0)
            && next.is_none_or(|w| v.abs() < w.abs() && v * w >= 0.0);
        if nearest {
            let (mut a, mut b) = (at(i.saturating_sub(1)), at((i + 1).min(n)));
            for _ in 0..100 {
                let (c, d) = (a + (b - a) / 3.0, b - (b - a) / 3.0);
                if f(c)?.abs() <= f(d)?.abs() {
                    b = d;
                } else {
                    a = c;
                }
            }
            let t = 0.5 * (a + b);
            let t = if v.abs() <= f(t)?.abs() { at(i) } else { t };
            if f(t)?.abs() <= tol {
                roots.push(t);
            }
        }
    }
    Ok(Some(roots))
}

/// A sketch curve on `plane` as a 3D curve segment.
fn world_seg(plane: &Plane, curve: &Curve2) -> Result<CurveSeg, ProjectionError> {
    let dir = |v: Vec2| -> Vec3 { plane.to_world(v) - plane.origin };
    let frame = |center: Point2, x: Vec2| {
        let x = dir(x.normalize());
        let normal = plane.normal;
        Plane {
            origin: plane.to_world(center),
            x: Dir3::new_unchecked(x),
            y: Dir3::new_unchecked(normal.get().cross(x).normalize()),
            normal,
        }
    };
    let seg = |curve, t0, t1| CurveSeg {
        curve,
        t0,
        t1,
        tag: Tag(0),
    };
    Ok(match curve {
        Curve2::Line(_) => return Err(ProjectionError::Unsupported("a line as a conic")),
        Curve2::Circle(c) => seg(
            Curve3::Circle {
                frame: frame(c.center, Vec2::X),
                radius: c.radius,
            },
            0.0,
            TAU,
        ),
        Curve2::Arc(a) => seg(
            Curve3::Circle {
                frame: frame(a.center, Vec2::X),
                radius: a.radius,
            },
            a.start_angle,
            a.start_angle + a.sweep,
        ),
        Curve2::Ellipse(e) => {
            // The kernel wants major ≥ minor: otherwise turn the frame by a quarter turn.
            let (x, major_radius, minor_radius, shift) = if e.minor_radius <= e.semi_major() {
                (e.major, e.semi_major(), e.minor_radius, 0.0)
            } else {
                (e.minor(), e.minor_radius, e.semi_major(), FRAC_PI_2)
            };
            seg(
                Curve3::Ellipse {
                    frame: frame(e.center, x),
                    major_radius,
                    minor_radius,
                },
                e.t0 - shift,
                e.t1 - shift,
            )
        }
        Curve2::BSpline(b) => {
            let mut knots: Vec<f64> = Vec::new();
            let mut mults: Vec<u32> = Vec::new();
            for &k in b.knots() {
                match (knots.last(), mults.last_mut()) {
                    (Some(&last), Some(m)) if last.to_bits() == k.to_bits() => *m += 1,
                    _ => {
                        knots.push(k);
                        mults.push(1);
                    }
                }
            }
            let data = BSplineCurveData {
                degree: u8::try_from(b.degree()).unwrap_or(u8::MAX),
                knots,
                mults,
                poles: b.poles().iter().map(|&p| plane.to_world(p)).collect(),
                weights: b.is_rational().then(|| b.weights().to_vec()),
                periodic: false,
            };
            seg(Curve3::BSpline(data), 0.0, b.span())
        }
    })
}

/// A B-spline lying in `plane`, trimmed to `[t0, t1]`, in sketch coordinates.
fn spline(
    plane: &Plane,
    data: &BSplineCurveData,
    t0: f64,
    t1: f64,
) -> Result<ProjectedGeom, ProjectionError> {
    if data.periodic {
        return Err(ProjectionError::Unsupported("a periodic B-spline"));
    }
    let knots: Vec<f64> = data
        .knots
        .iter()
        .zip(&data.mults)
        .flat_map(|(&k, &m)| std::iter::repeat_n(k, m as usize))
        .collect();
    let first = knots.first().copied().unwrap_or(0.0);
    let poles: Vec<Point2> = data.poles.iter().map(|&p| plane.to_local(p)).collect();
    let weights = data
        .weights
        .clone()
        .unwrap_or_else(|| vec![1.0; poles.len()]);
    let whole = BSpline2::new(usize::from(data.degree), knots, poles, weights)
        .map_err(|_| ProjectionError::Unsupported("this B-spline"))?;
    let piece = whole.trimmed(t0 - first, t1 - first);
    Ok(ProjectedGeom::Spline {
        degree: u8::try_from(piece.degree()).unwrap_or(u8::MAX),
        knots: piece.knots().to_vec(),
        poles: piece.poles().to_vec(),
        weights: data.weights.is_some().then(|| piece.weights().to_vec()),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(found: &[f64], expected: &[f64]) {
        assert_eq!(found.len(), expected.len(), "{found:?} vs {expected:?}");
        let mut found = found.to_vec();
        found.sort_by(f64::total_cmp);
        for (f, e) in found.iter().zip(expected) {
            assert!((f - e).abs() < 1e-6, "{found:?} vs {expected:?}");
        }
    }

    #[test]
    fn trig_roots_cross_touch_miss_and_lie_in_the_plane() {
        let full = (0.0, TAU);
        // cos t = 0.5.
        let r = trig_roots(-0.5, 1.0, 0.0, full, 1e-9).unwrap();
        close(&r, &[TAU / 6.0, 5.0 * TAU / 6.0]);
        // Only the root inside a trimmed range, also when the range is past a whole turn.
        let r = trig_roots(-0.5, 1.0, 0.0, (TAU, TAU + 2.0), 1e-9).unwrap();
        close(&r, &[TAU + TAU / 6.0]);
        // Touching from both sides.
        close(&trig_roots(-1.0, 1.0, 0.0, full, 1e-9).unwrap(), &[0.0]);
        close(
            &trig_roots(1.0, 1.0, 0.0, full, 1e-9).unwrap(),
            &[TAU / 2.0],
        );
        assert!(trig_roots(2.0, 1.0, 0.0, full, 1e-9).unwrap().is_empty());
        assert!(trig_roots(0.0, 0.0, 0.0, full, 1e-9).is_none());
        assert!(trig_roots(1.0, 0.0, 0.0, full, 1e-9).unwrap().is_empty());
    }

    #[test]
    fn sampled_roots_find_crossings_touches_and_ends() {
        let roots = |f: fn(f64) -> f64| sampled_roots(|t| Ok(f(t)), (0.0, 4.0), 64, 1e-9).unwrap();
        close(&roots(|t| (t - 1.3) * (t - 2.7)).unwrap(), &[1.3, 2.7]);
        close(&roots(|t| (t - 1.7) * (t - 1.7)).unwrap(), &[1.7]);
        close(&roots(|t| t).unwrap(), &[0.0]);
        close(&roots(|t| t - 4.0).unwrap(), &[4.0]);
        close(&roots(|t| t - 2.0).unwrap(), &[2.0]);
        assert!(roots(|t| t + 1.0).unwrap().is_empty());
        assert!(roots(|_| 0.0).is_none());
    }
}
