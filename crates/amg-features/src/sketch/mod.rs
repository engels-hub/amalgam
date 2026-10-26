//! The sketch feature (`sketch.sketch`, S03 SK-19, lane B).
//!
//! [`SketchFeature`] is the timeline feature around an [`amg_sketch::Sketch`]:
//!
//! - **Plane.** An origin plane or a construction plane ([`PlaneInput`]); planar faces wait for
//!   the B-rep kernel.
//! - **Dimensions.** The value of every driving dimension is an S01 [`Expr`] in
//!   [`SketchFeature::dims`]. `Feature::exprs_mut` exposes them, so the command layer turns each
//!   inline expression into a model parameter `d<n>` (Fusion behaviour).
//! - **Recompute.** `compute` resolves the plane, evaluates the expressions with a unit check
//!   (length → mm, angle → rad), solves warm-started from the stored values and detects
//!   profiles. A solve that fails keeps the stored geometry and reports a `Warning` with the
//!   diagnosis; it never fails downstream features.
//! - **Edits.** [`SketchFeature::apply`] edits a clone of the feature ([`SketchEdit`]); the
//!   caller commits it with `Command::EditFeature`, so one edit is one undo entry.

mod edit;

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
    ConstraintId, ConstraintKind, EntityId, EntityKind, ExternalLinkId, ProfileOptions, ProfileSet,
    Quantity, Sketch, detect_profiles,
};
use serde::{Deserialize, Serialize};

pub use edit::SketchEdit;

/// Forces this module's feature registration to be linked (registry `link()` convention).
pub fn link() {}

/// Where the sketch lives. `Ref` is an `EntityRef::Construction(..)` plane or an
/// `EntityRef::Topo(..)` planar face (not resolvable before the B-rep kernel lands).
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
    /// The locked sketch entities that carry the result.
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
    /// Projected geometry (SK-09).
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
            PlaneInput::Ref(_) => Err(FeatureError::InvalidValue {
                field: "plane",
                msg: "sketching on a planar face needs the B-rep kernel".to_owned(),
            }),
        }
    }
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
        let mut work = self.sketch.clone();
        work.set_plane(plane);
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
        let mut warnings = Vec::new();
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
            // Fusion behaviour: keep the last good geometry.
            let mut kept = self.sketch.clone();
            kept.set_plane(plane);
            kept
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
