//! Reference resolution: [`RefInput`] → [`ResolvedRef`] (S05 §7, CON-03).
//!
//! | Reference | [`RefGeom`] | `context_plane` |
//! |---|---|---|
//! | origin entity | point, line or plane | |
//! | construction geometry | plane, line or point | |
//! | planar B-rep face | `Plane`, normal out of the material | |
//! | other B-rep face | `Cylinder`, `Cone`, `Sphere`, `Torus`, else `Surface` | |
//! | B-rep edge | `Segment`, `Circle`, else `Curve` | first adjacent planar face |
//! | B-rep vertex | `Point` | |
//! | sketch curve | `Segment`, `Circle`, else `Curve` | the sketch plane |
//! | sketch point | `Point` | the sketch plane |
//! | planar mesh face group | `Plane` fitted to its faces | |
//!
//! Bodies and profiles are not construction inputs. Topological references go through DOC-06
//! naming.
//!
//! **Spaces.** Inside a recompute ([`resolve_input`]) the result is in the referenced entity's
//! component space, which is the computing feature's space for its own component; an
//! `EntityRef::InContext` reference applies its occurrence path (from the context component),
//! with the positions captured at that timeline node. For tools ([`resolve_in`],
//! [`resolve_in_at`]) the context is the root component or the given occurrence path, and the
//! result is in world space with the solved placements.
//!
//! A reference that does not resolve inside a recompute is highlighted on the failing node
//! (`Diagnostic::entities`).

mod brep;
mod sketch;

use amg_brep::BrepKernel;
use amg_core::{Dir3, Line3, Mat3, Plane, Tolerance, Transform};
use amg_doc::naming::{self, Resolved, TopoKind, TopoRef};
use amg_doc::stub::SketchGeom;
use amg_doc::{
    BodyGeom, BodyId, ConstructionGeom, ConstructionId, Dependency, Design, EntityRef, Evaluated,
    FeatureError, OccurrenceId, RecomputeCtx, ResolveError, SketchId,
};
use amg_mesh::{FaceGroupId, FaceId, IndexedMesh};

use super::{RefGeom, RefInput, ResolvedRef, math};
use crate::inspect::measure::{MeasureOpts, fit_plane_faces};

/// How far an occurrence transform may stray from a rotation plus translation.
const RIGID_EPS: f64 = 1e-9;

/// Why a reference did not resolve for a tool.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum InspectResolveError {
    /// A topological reference did not resolve.
    #[error(transparent)]
    Resolve(#[from] ResolveError),
    /// A body, profile or other entity that gives no construction geometry.
    #[error("entity kind not supported as construction input")]
    Unsupported,
    /// The referenced sketch, entity, construction or occurrence does not exist.
    #[error("{0} does not exist")]
    Missing(String),
    /// The entity exists but has no usable geometry (a curved mesh face group, a solid
    /// without B-rep data, ...).
    #[error("{0}")]
    Geometry(String),
}

/// A failed resolution, before it becomes a feature or tool error.
#[derive(Debug)]
enum Fail {
    Resolve(ResolveError),
    /// A whole input of the feature is not there.
    Input(Box<Dependency>),
    Unsupported(&'static str),
    Missing(String),
    Geometry(String),
}

impl From<Fail> for FeatureError {
    fn from(f: Fail) -> FeatureError {
        let invalid = |msg| FeatureError::InvalidValue {
            field: "inputs",
            msg,
        };
        match f {
            Fail::Resolve(e) => FeatureError::Resolve(e),
            Fail::Input(d) => FeatureError::MissingInput(d),
            Fail::Unsupported(what) => invalid(format!("{what} can't be a construction input")),
            Fail::Missing(what) => invalid(format!("{what} does not exist")),
            Fail::Geometry(msg) => invalid(msg),
        }
    }
}

impl From<Fail> for InspectResolveError {
    fn from(f: Fail) -> InspectResolveError {
        match f {
            Fail::Resolve(e) => InspectResolveError::Resolve(e),
            Fail::Input(d) => InspectResolveError::Missing(format!("{d:?}")),
            Fail::Unsupported(_) => InspectResolveError::Unsupported,
            Fail::Missing(what) => InspectResolveError::Missing(what),
            Fail::Geometry(msg) => InspectResolveError::Geometry(msg),
        }
    }
}

/// The state references are resolved against.
trait Source {
    fn topo(&self, r: &TopoRef) -> Result<Resolved, ResolveError>;
    fn body(&self, id: BodyId) -> Option<&BodyGeom>;
    fn sketch(&self, id: SketchId) -> Option<&SketchGeom>;
    fn construction(&self, id: ConstructionId) -> Option<&ConstructionGeom>;
    fn kernel(&self) -> &dyn BrepKernel;
    fn tolerance(&self) -> Tolerance;
    /// The transform of the occurrence path `path` below the context into the result space;
    /// `None` if the path is not valid.
    fn transform(&self, path: &[OccurrenceId]) -> Option<Transform>;
}

/// Inside a recompute: the state just before the computing feature, in its component space.
struct InRecompute<'c, 'a> {
    ctx: &'c RecomputeCtx<'a>,
}

impl Source for InRecompute<'_, '_> {
    fn topo(&self, r: &TopoRef) -> Result<Resolved, ResolveError> {
        self.ctx.resolve(r)
    }
    fn body(&self, id: BodyId) -> Option<&BodyGeom> {
        self.ctx.before().body(id).map(|b| &b.geom)
    }
    fn sketch(&self, id: SketchId) -> Option<&SketchGeom> {
        self.ctx.sketch(id).ok()
    }
    fn construction(&self, id: ConstructionId) -> Option<&ConstructionGeom> {
        self.ctx.construction(id).ok()
    }
    fn kernel(&self) -> &dyn BrepKernel {
        self.ctx.kernel()
    }
    fn tolerance(&self) -> Tolerance {
        *self.ctx.tolerance()
    }
    fn transform(&self, path: &[OccurrenceId]) -> Option<Transform> {
        if path.is_empty() {
            return Some(Transform::IDENTITY);
        }
        if !self.ctx.design().is_valid_path(self.ctx.context(), path) {
            return None;
        }
        path.iter().try_fold(Transform::IDENTITY, |acc, o| {
            Some(acc * self.ctx.occurrence_transform(*o)?)
        })
    }
}

/// For tools: an evaluated document, in world space, with the context at the end of `base`.
struct InEvaluated<'a> {
    eval: &'a Evaluated,
    design: &'a Design,
    base: &'a [OccurrenceId],
}

impl Source for InEvaluated<'_> {
    fn topo(&self, r: &TopoRef) -> Result<Resolved, ResolveError> {
        naming::resolve(self.eval, naming::default_kernel(), &self.tolerance(), r)
            .result
            .and_then(|v| {
                v.first().copied().ok_or(ResolveError::NotFound {
                    kind: r.kind,
                    removed_by: None,
                })
            })
    }
    fn body(&self, id: BodyId) -> Option<&BodyGeom> {
        self.eval.body(id).map(|b| &b.geom)
    }
    fn sketch(&self, id: SketchId) -> Option<&SketchGeom> {
        self.eval.sketch(id).map(|s| &**s)
    }
    fn construction(&self, id: ConstructionId) -> Option<&ConstructionGeom> {
        self.eval.construction(id)
    }
    fn kernel(&self) -> &dyn BrepKernel {
        naming::default_kernel()
    }
    fn tolerance(&self) -> Tolerance {
        self.design.meta().tolerance
    }
    fn transform(&self, path: &[OccurrenceId]) -> Option<Transform> {
        let full: Vec<OccurrenceId> = self.base.iter().chain(path).copied().collect();
        if full.is_empty() {
            return Some(Transform::IDENTITY);
        }
        if !self.design.is_valid_path(self.design.root(), &full) {
            return None;
        }
        self.eval.world_transform(&full)
    }
}

/// A resolved entity and the face orientation of
/// [`ConstructInputs::resolve_oriented`](super::ConstructInputs::resolve_oriented).
type Oriented = (ResolvedRef, bool);

/// Resolves one construction input against the state just before the computing feature. A
/// failure highlights the reference on the node.
pub fn resolve_input(ctx: &RecomputeCtx<'_>, r: &RefInput) -> Result<ResolvedRef, FeatureError> {
    resolve_input_oriented(ctx, r).map(|(resolved, _)| resolved)
}

/// [`resolve_input`] plus whether an analytic face's outward normal points towards the axis or
/// centre of its geometry (a hole).
pub fn resolve_input_oriented(
    ctx: &RecomputeCtx<'_>,
    r: &RefInput,
) -> Result<(ResolvedRef, bool), FeatureError> {
    resolve_from(&InRecompute { ctx }, r).map_err(|fail| {
        ctx.highlight(r.entity().cloned());
        fail.into()
    })
}

/// The plane of a plane input (origin or construction plane, planar face or face group), for
/// features that sit on a plane, such as sketches.
pub fn resolve_plane_input(ctx: &RecomputeCtx<'_>, r: &RefInput) -> Result<Plane, FeatureError> {
    let resolved = resolve_input(ctx, r)?;
    math::plane_of(&resolved.geom).map_err(|e| {
        ctx.highlight(r.entity().cloned());
        e.into()
    })
}

/// For tools outside recompute (measure, section analysis): resolves against an `Evaluated`,
/// with the root component as the context. World space.
pub fn resolve_in(
    eval: &Evaluated,
    design: &Design,
    r: &RefInput,
) -> Result<ResolvedRef, InspectResolveError> {
    resolve_in_at(eval, design, &[], r)
}

/// [`resolve_in`] with the context component placed by the occurrence path `path` from the
/// root: its origin entities and references come out in world space through that placement.
pub fn resolve_in_at(
    eval: &Evaluated,
    design: &Design,
    path: &[OccurrenceId],
    r: &RefInput,
) -> Result<ResolvedRef, InspectResolveError> {
    let source = InEvaluated {
        eval,
        design,
        base: path,
    };
    Ok(resolve_from(&source, r)?.0)
}

/// Whether resolving `r` goes through DOC-06 naming (which records its own warnings).
pub(super) fn is_topological(r: &EntityRef) -> bool {
    match r {
        EntityRef::Topo(_) => true,
        EntityRef::InContext { target, .. } => is_topological(target),
        _ => false,
    }
}

fn resolve_from(source: &dyn Source, r: &RefInput) -> Result<Oriented, Fail> {
    match r {
        RefInput::Origin(e) => {
            let t = placement(source, &[])?;
            Ok((ResolvedRef::new(math::origin_geom(*e, &t)), false))
        }
        RefInput::Ref(entity) => resolve_entity(source, entity, &mut Vec::new()),
    }
}

/// The rigid transform of `path` into the result space.
fn placement(source: &dyn Source, path: &[OccurrenceId]) -> Result<Transform, Fail> {
    let t = source
        .transform(path)
        .ok_or_else(|| Fail::Missing("the occurrence path".to_owned()))?;
    let m = t.matrix3;
    let rigid = (m.transpose() * m).abs_diff_eq(Mat3::IDENTITY, RIGID_EPS)
        && (m.determinant() - 1.0).abs() <= RIGID_EPS
        && t.translation.is_finite();
    if rigid {
        Ok(t)
    } else {
        Err(Fail::Geometry(
            "the occurrence transform is not rigid".to_owned(),
        ))
    }
}

/// Resolves `r`, seen through the occurrence path `path` below the context.
fn resolve_entity(
    source: &dyn Source,
    r: &EntityRef,
    path: &mut Vec<OccurrenceId>,
) -> Result<Oriented, Fail> {
    match r {
        EntityRef::InContext {
            path: below,
            target,
        } => {
            path.extend_from_slice(below);
            resolve_entity(source, target, path)
        }
        EntityRef::Construction(c) => {
            let t = placement(source, path)?;
            let geom = source
                .construction(*c)
                .ok_or(Fail::Input(Box::new(Dependency::Construction(*c))))?;
            let geom = match geom {
                ConstructionGeom::Plane(p) => RefGeom::Plane(math::transform_plane(&t, p)),
                ConstructionGeom::Axis(l) => RefGeom::Line(moved_line(&t, l)),
                ConstructionGeom::Point(p) => RefGeom::Point(t.transform_point3(*p)),
            };
            Ok((ResolvedRef::new(geom), false))
        }
        EntityRef::Topo(topo) => resolve_topo(source, topo, path),
        EntityRef::SketchCurve { sketch, curve } => {
            let t = placement(source, path)?;
            let g = source
                .sketch(*sketch)
                .ok_or(Fail::Input(Box::new(Dependency::Sketch(*sketch))))?;
            Ok((sketch::curve(g, *curve, &t)?, false))
        }
        EntityRef::SketchPoint { sketch, point } => {
            let t = placement(source, path)?;
            let g = source
                .sketch(*sketch)
                .ok_or(Fail::Input(Box::new(Dependency::Sketch(*sketch))))?;
            Ok((sketch::point(g, *point, &t)?, false))
        }
        EntityRef::Body(_) => Err(Fail::Unsupported("a body")),
        EntityRef::Profile { .. } => Err(Fail::Unsupported("a profile")),
    }
}

fn moved_line(t: &Transform, l: &Line3) -> Line3 {
    if *t == Transform::IDENTITY {
        return *l;
    }
    let d = t.transform_vector3(l.dir.get());
    Line3 {
        point: t.transform_point3(l.point),
        dir: Dir3::new(d, &Tolerance::DEFAULT).unwrap_or(l.dir),
    }
}

fn resolve_topo(source: &dyn Source, r: &TopoRef, path: &[OccurrenceId]) -> Result<Oriented, Fail> {
    let found = source.topo(r).map_err(Fail::Resolve)?;
    let t = placement(source, path)?;
    let geom = source
        .body(found.body)
        .ok_or(Fail::Input(Box::new(Dependency::Body(found.body))))?;
    let (mut resolved, reversed) = match (geom, found.kind) {
        (BodyGeom::Solid(solid), kind) if kind != TopoKind::FaceGroup => {
            let shape = solid
                .brep()
                .ok_or_else(|| Fail::Geometry("the body has no B-rep geometry".to_owned()))?;
            brep::sub_entity(
                source.kernel(),
                &source.tolerance(),
                shape,
                kind,
                found.index,
                &t,
            )?
        }
        (BodyGeom::Mesh(mesh), TopoKind::FaceGroup) => {
            let plane = group_plane(mesh, FaceGroupId(found.index))?;
            let plane = math::transform_plane(&t, &plane);
            (ResolvedRef::new(RefGeom::Plane(plane)), false)
        }
        _ => return Err(Fail::Unsupported("this kind of body sub-entity")),
    };
    resolved.quality = Some(found.quality);
    Ok((resolved, reversed))
}

/// The plane fitted to a mesh face group, if all its vertices lie within the planar tolerance.
fn group_plane(mesh: &IndexedMesh, group: FaceGroupId) -> Result<Plane, Fail> {
    let faces: Vec<FaceId> = mesh.groups.faces_of(group).collect();
    if faces.is_empty() {
        return Err(Fail::Missing(format!("face group {}", group.0)));
    }
    let fit = fit_plane_faces(mesh, &faces)
        .ok_or_else(|| Fail::Geometry("the face group is not planar".to_owned()))?;
    let limit = MeasureOpts::default().planar_tolerance;
    if fit.max_deviation > limit {
        return Err(Fail::Geometry(format!(
            "the face group is not planar (it deviates {:.4} mm, limit {limit} mm)",
            fit.max_deviation
        )));
    }
    Ok(fit.plane)
}
