//! The sketch feature (`sketch.sketch`, S03 SK-19, lane B).
//!
//! [`SketchFeature`] is the timeline feature around an [`amg_sketch::Sketch`]:
//!
//! - **Plane.** An origin plane, a construction plane or a planar face ([`PlaneInput`]; faces in
//!   the S03 planar-face frame, see `face_frame`).
//! - **Dimensions.** The value of every driving dimension is an S01 [`Expr`] in
//!   [`SketchFeature::dims`]. `Feature::exprs_mut` exposes them, so the command layer turns each
//!   inline expression into a model parameter `d<n>` (Fusion behaviour).
//! - **Recompute.** `compute` resolves the plane, evaluates the expressions with a unit check
//!   (length → mm, angle → rad), solves warm-started from the stored values and detects
//!   profiles. A solve that fails keeps the stored geometry and reports a `Warning` with the
//!   diagnosis; it never fails downstream features.
//! - **Projections.** Every [`ProjectionLink`] is resolved again before the solve and its
//!   locked entities move to the new geometry (SK-09, [`project_reference`]). A source that
//!   does not resolve, or no longer gives the same pieces, is a `Warning`, and its entities
//!   keep the stored geometry ([`SketchFeature::commit_solved`] keeps that geometry current).
//! - **Edits.** [`SketchFeature::apply`] edits a clone of the feature ([`SketchEdit`]); the
//!   caller commits it with `Command::EditFeature`, so one edit is one undo entry.

mod edit;
mod project;

use std::collections::BTreeMap;
use std::sync::Arc;

use amg_core::{Dimension as Dim, Plane};
use amg_doc::stub::{SketchEntityKey, SketchGeom};
use amg_doc::{
    ConstructionGeom, Dependency, Diagnostic, EntityRef, Expr, Feature, FeatureError,
    FeatureOutput, RecomputeCtx, SketchId, register_feature,
};
use amg_sketch::solve::{NativeSolver, SolveOptions, SolveReport, SolveStatus, SolverBackend};
use amg_sketch::{
    ConstraintId, ConstraintKind, EntityId, EntityKind, EntityOrigin, ExternalLinkId,
    ProfileOptions, ProfileSet, Quantity, Sketch, detect_profiles,
};
use serde::{Deserialize, Serialize};

pub use edit::SketchEdit;
pub use project::{ProjectionError, project_reference};

/// Forces this module's feature registration to be linked (registry `link()` convention).
pub fn link() {}

/// Where the sketch lives. `Ref` is an `EntityRef::Construction(..)` plane or an
/// `EntityRef::Topo(..)` planar face (resolved by CON-03 `resolve_plane_input`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum PlaneInput {
    /// World XY.
    OriginXY,
    /// World YZ.
    OriginYZ,
    /// World XZ.
    OriginXZ,
    /// A construction plane or planar face.
    Ref(EntityRef),
}

/// What a projected link imports (SK-09).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProjectionMode {
    /// Project the source onto the sketch plane.
    Project,
    /// Include source geometry lying in the plane.
    Include,
    /// Intersection points of the source with the plane.
    Intersect,
    /// Intersection curve of a surface with the plane.
    IntersectionCurve,
}

/// A projection source and the sketch entities it produced (refreshed by SK-09).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ProjectionLink {
    /// The projected model entity.
    pub source: EntityRef,
    /// How it is projected.
    pub mode: ProjectionMode,
    /// The locked sketch entities that carry the result, one per piece of the source in
    /// projection order. A removed entity keeps its place, so the others still match.
    pub entities: Vec<EntityId>,
}

/// The sketch feature. Immutable inside a design; edits go through [`SketchFeature::apply`].
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SketchFeature {
    /// Plane or reference the sketch is drawn on.
    pub plane: PlaneInput,
    /// Authored geometry with the last stored values (the warm start, and the geometry kept
    /// when a solve fails).
    pub sketch: Sketch,
    /// Value expression of every driving dimension. Kept in step with the sketch by every edit.
    pub dims: BTreeMap<ConstraintId, Expr>,
    /// Projected geometry (SK-09): the source of every [`EntityOrigin::Projected`] entity.
    #[serde(default)]
    pub links: BTreeMap<ExternalLinkId, ProjectionLink>,
}

impl SketchFeature {
    /// An empty sketch on `plane`.
    pub fn new(plane: PlaneInput) -> SketchFeature {
        SketchFeature {
            plane,
            sketch: Sketch::new(Plane::XY),
            dims: BTreeMap::new(),
            links: BTreeMap::new(),
        }
    }

    /// The class (length or angle) of dimension `id`; `None` if `id` is not a dimension.
    fn quantity(&self, id: ConstraintId) -> Option<Quantity> {
        match self.sketch.constraint(id)?.kind {
            ConstraintKind::Dimension(ref d) => Some(d.kind.quantity()),
            _ => None,
        }
    }

    fn resolve_plane(&self, ctx: &RecomputeCtx<'_>) -> Result<Plane, FeatureError> {
        match &self.plane {
            PlaneInput::OriginXY => Ok(Plane::XY),
            PlaneInput::OriginYZ => Ok(Plane::YZ),
            PlaneInput::OriginXZ => Ok(Plane::XZ),
            PlaneInput::Ref(EntityRef::Construction(c)) => match ctx.construction(*c)? {
                ConstructionGeom::Plane(p) => Ok(*p),
                _ => Err(FeatureError::InvalidValue {
                    field: "plane",
                    msg: "the referenced construction is not a plane".to_owned(),
                }),
            },
            // A planar face (or face group), resolved through DOC-06 naming (S01 DOC-13).
            PlaneInput::Ref(r) => {
                let input = crate::construct::RefInput::Ref(r.clone());
                let p = crate::construct::resolve::resolve_plane_input(ctx, &input)?;
                Ok(face_frame(&p, ctx.tolerance()))
            }
        }
    }
}

/// The S03 planar-face sketch frame: the face plane's normal, origin at the projection of the
/// world origin, x along the projection of world X (world Y, then Z, if X is normal to the
/// face). The frame depends only on the plane, not on the face's surface parametrisation, so a
/// face moved along its normal keeps the sketch's in-plane coordinates.
fn face_frame(p: &Plane, tol: &amg_core::Tolerance) -> Plane {
    let origin = p.project(amg_core::Point3::ZERO);
    [amg_core::Vec3::X, amg_core::Vec3::Y, amg_core::Vec3::Z]
        .into_iter()
        .find_map(|x| Plane::from_normal_x(origin, p.normal, x, tol).ok())
        .unwrap_or(Plane::from_normal(origin, p.normal))
}

impl Feature for SketchFeature {
    fn inputs(&self) -> Vec<Dependency> {
        let mut deps = Vec::new();
        if let PlaneInput::Ref(r) = &self.plane {
            deps.push(match r {
                EntityRef::Construction(c) => Dependency::Construction(*c),
                other => Dependency::Entity(other.clone()),
            });
        }
        deps.extend(
            self.links
                .values()
                .map(|l| Dependency::Entity(l.source.clone())),
        );
        deps
    }

    /// Projection sources: a lost one is a warning, and the sketch keeps its last geometry.
    fn input_is_optional(&self, dep: &Dependency) -> bool {
        let Dependency::Entity(e) = dep else {
            return false;
        };
        let is_plane = matches!(&self.plane, PlaneInput::Ref(r) if r == e);
        !is_plane && self.links.values().any(|l| l.source == *e)
    }

    fn exprs(&self) -> Vec<&Expr> {
        self.dims.values().collect()
    }

    fn exprs_mut(&mut self) -> Vec<(&mut Expr, Dim)> {
        let sketch = &self.sketch;
        self.dims
            .iter_mut()
            .filter_map(|(id, e)| {
                let dim = match sketch.constraint(*id)?.kind {
                    ConstraintKind::Dimension(ref d) => match d.kind.quantity() {
                        Quantity::Length => Dim::LENGTH,
                        Quantity::Angle => Dim::ANGLE,
                    },
                    _ => return None,
                };
                Some((e, dim))
            })
            .collect()
    }

    fn validate(&self) -> Result<(), FeatureError> {
        for id in self.dims.keys() {
            if self.quantity(*id).is_none() {
                return Err(FeatureError::InvalidValue {
                    field: "dims",
                    msg: format!("{id:?} is not a dimension of the sketch"),
                });
            }
        }
        for (link, l) in &self.links {
            let foreign = l.entities.iter().any(|&e| {
                self.sketch
                    .entity(e)
                    .is_some_and(|e| e.origin != EntityOrigin::Projected(*link))
            });
            if foreign {
                return Err(FeatureError::InvalidValue {
                    field: "links",
                    msg: format!("projection {} lists geometry it did not project", link.0),
                });
            }
        }
        if let PlaneInput::Ref(EntityRef::InContext { .. }) = &self.plane {
            return Err(FeatureError::InvalidValue {
                field: "plane",
                msg: "an in-context plane reference is not supported".to_owned(),
            });
        }
        Ok(())
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let plane = self.resolve_plane(ctx)?;
        // The stored geometry on the resolved plane, with the projections brought up to date.
        let mut base = self.sketch.clone();
        base.set_plane(plane);
        let mut warnings = project::refresh(ctx, &plane, &mut base, &self.links)?;
        let mut work = base.clone();
        for (id, expr) in &self.dims {
            let value = match self.quantity(*id) {
                Some(Quantity::Length) => ctx.eval_length(expr)?,
                Some(Quantity::Angle) => ctx.eval_angle(expr)?,
                None => {
                    return Err(FeatureError::InvalidValue {
                        field: "dims",
                        msg: format!("{id:?} is not a dimension of the sketch"),
                    });
                }
            };
            work.set_dimension_value(*id, value)
                .map_err(|e| FeatureError::InvalidValue {
                    field: "dims",
                    msg: format!("{id:?}: {e}"),
                })?;
        }

        let cancel = || ctx.check_cancelled().is_err();
        let report = NativeSolver::new().solve(&mut work, &SolveOptions::default(), &cancel);
        if report.status == SolveStatus::Cancelled {
            return Err(FeatureError::Cancelled);
        }

        let sketch_id = SketchId(ctx.feature_id());
        let solved = if report.status.is_converged() {
            for (id, v) in &report.driven {
                // A measured value outside the dimension's input domain is only displayed.
                let _ = work.set_dimension_value(*id, *v);
            }
            if report.status == SolveStatus::ConvergedRedundant {
                warnings.push(diagnose(sketch_id, &work, &report));
            }
            work
        } else {
            warnings.push(diagnose(sketch_id, &work, &report));
            // Fusion behaviour: keep the last good geometry (projections stay current).
            base
        };

        let profiles = detect_profiles(&solved, &ProfileOptions::default());
        if let Some(w) = profile_warning(&profiles) {
            warnings.push(w);
        }
        Ok(FeatureOutput {
            sketch: Some(Arc::new(SketchGeom {
                plane,
                sketch: Arc::new(solved),
                report,
                profiles: Arc::new(profiles),
            })),
            warnings,
            ..FeatureOutput::default()
        })
    }
}

register_feature!(SketchFeature, "sketch.sketch", version = 1);

fn entity_ref(sketch_id: SketchId, sketch: &Sketch, e: EntityId) -> EntityRef {
    let key = SketchEntityKey(u64::from(e.0));
    match sketch.entity(e).map(|e| &e.kind) {
        Some(EntityKind::Point { .. }) => EntityRef::SketchPoint {
            sketch: sketch_id,
            point: key,
        },
        _ => EntityRef::SketchCurve {
            sketch: sketch_id,
            curve: key,
        },
    }
}

/// Warning for a solve that did not converge (or converged with redundant constraints).
fn diagnose(sketch_id: SketchId, sketch: &Sketch, report: &SolveReport) -> Diagnostic {
    let ids = |cs: &[ConstraintId]| {
        cs.iter()
            .map(|c| format!("c{}", c.0))
            .collect::<Vec<_>>()
            .join(", ")
    };
    let mut message = match report.status {
        SolveStatus::ConvergedRedundant => {
            "sketch has redundant constraints (it still solves)".to_owned()
        }
        SolveStatus::Inconsistent => {
            "sketch constraints contradict each other; the previous geometry is kept".to_owned()
        }
        _ => "sketch did not solve; the previous geometry is kept".to_owned(),
    };
    if !report.unsatisfied.is_empty() {
        message.push_str(&format!(
            "; unsatisfied: {} (residual {:.3e} mm)",
            ids(&report.unsatisfied),
            report.max_residual
        ));
    }
    let mut involved: Vec<ConstraintId> = report.unsatisfied.clone();
    if let Some(d) = &report.diagnosis {
        for g in &d.groups {
            message.push_str(&format!("; {:?}: {}", g.kind, ids(&g.constraints)));
            involved.extend(&g.constraints);
        }
    }
    involved.sort_unstable();
    involved.dedup();
    let mut entities: Vec<EntityId> = involved
        .iter()
        .filter_map(|c| sketch.constraint(*c))
        .flat_map(|c| c.kind.entities())
        .collect();
    entities.sort_unstable();
    entities.dedup();
    Diagnostic {
        message,
        cause: None,
        entities: entities
            .into_iter()
            .map(|e| entity_ref(sketch_id, sketch, e))
            .collect(),
    }
}

/// One warning for profile problems (gaps, overlaps, ...), if any.
fn profile_warning(profiles: &ProfileSet) -> Option<Diagnostic> {
    if profiles.issues.is_empty() {
        return None;
    }
    Some(Diagnostic {
        message: format!(
            "{} profile issue(s): {:?}",
            profiles.issues.len(),
            profiles.issues
        ),
        cause: None,
        entities: Vec::new(),
    })
}

#[cfg(test)]
mod tests {
    use amg_core::{Dir3, Point3, Tolerance, Vec3};

    use super::*;

    #[test]
    fn face_frame_projects_the_world_origin_and_world_x() {
        let tol = Tolerance::default();
        // A +Z face at z = 40 whose surface frame has its own origin and x.
        let face = Plane::from_normal_x(
            Point3::new(30.0, 20.0, 40.0),
            Dir3::Z,
            Vec3::new(0.0, 1.0, 0.0),
            &tol,
        )
        .unwrap();
        let f = face_frame(&face, &tol);
        assert!((f.origin - Point3::new(0.0, 0.0, 40.0)).length() < 1e-12);
        assert!((f.x.get() - Vec3::X).length() < 1e-12);
        assert!((f.normal.get() - Vec3::Z).length() < 1e-12);
        // Local and world x, y coincide on a +Z face, at any height (note §11.2).
        let l = f.to_local(Point3::new(12.0, -7.0, 40.0));
        assert!((l.x - 12.0).abs() < 1e-12 && (l.y + 7.0).abs() < 1e-12);
        // A face normal to world X takes world Y as its x axis.
        let side = Plane::from_normal(Point3::new(100.0, 5.0, 5.0), Dir3::X);
        let s = face_frame(&side, &tol);
        assert!((s.origin - Point3::new(100.0, 0.0, 0.0)).length() < 1e-12);
        assert!((s.x.get() - Vec3::Y).length() < 1e-12);
    }
}
