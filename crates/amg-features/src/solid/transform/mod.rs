//! `solid.scale` (S04 SOL-06, BREP-14 `Shape::gtransformed`): bodies scaled about a point,
//! uniformly or by world x, y, z factors.
//!
//! A uniform scale keeps analytic surfaces; a non-uniform one turns them into B-splines (a
//! sphere becomes an exact rational ellipsoid). Factors must be finite and positive (a
//! negative factor would mirror; use `solid.mirror`).
//!
//! Naming (naming note §5.2 "Scale"): none of its own; the `gtransformed` history is 1:1, so
//! every face, edge and vertex keeps its name.

mod dialog;

use amg_core::{Transform, Vec3};
use amg_doc::naming::OperandNaming;
use amg_doc::{
    BodyChange, BodyChangeKind, BodyId, Dependency, Dim, EntityRef, Expr, Feature, FeatureError,
    FeatureOutput, RecomputeCtx, register_feature,
};
use serde::{Deserialize, Serialize};

use super::common::{finish_naming, live_body, op_kernel, solid_geom};
use super::error::kernel_failure;
use super::shell::{inline, invalid, positive};
use crate::construct::resolve::resolve_input;
use crate::construct::{OriginEntity, RefInput, math};

/// Uniform or per-axis scale factors (dimensionless).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ScaleKind {
    /// One factor.
    Uniform(Expr),
    /// Factors along world x, y and z.
    NonUniform([Expr; 3]),
}

/// `solid.scale`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Scale {
    /// Bodies to scale.
    #[serde(default)]
    pub bodies: Vec<BodyId>,
    /// The fixed point (origin point, vertex, sketch or construction point).
    pub point: RefInput,
    /// The factors.
    pub kind: ScaleKind,
}

impl Scale {
    /// `bodies` scaled by `factor` (such as `"2"`) about the origin.
    pub fn uniform(bodies: Vec<BodyId>, factor: &str) -> Scale {
        Scale {
            bodies,
            point: RefInput::Origin(OriginEntity::Point),
            kind: ScaleKind::Uniform(inline(factor)),
        }
    }

    /// `bodies` scaled by world x, y, z factors about the origin.
    pub fn non_uniform(bodies: Vec<BodyId>, factors: [&str; 3]) -> Scale {
        Scale {
            bodies,
            point: RefInput::Origin(OriginEntity::Point),
            kind: ScaleKind::NonUniform(factors.map(inline)),
        }
    }

    /// The transform: scale about the point.
    ///
    /// # Errors
    /// Expression and resolution errors; [`super::SolidError::Invalid`] for a factor that is
    /// not finite and positive.
    pub fn transform(&self, ctx: &RecomputeCtx<'_>) -> Result<Transform, FeatureError> {
        let f = match &self.kind {
            ScaleKind::Uniform(e) => Vec3::splat(positive("scale", ctx.eval_scalar(e)?)?),
            ScaleKind::NonUniform([x, y, z]) => Vec3::new(
                positive("scale_x", ctx.eval_scalar(x)?)?,
                positive("scale_y", ctx.eval_scalar(y)?)?,
                positive("scale_z", ctx.eval_scalar(z)?)?,
            ),
        };
        let r = resolve_input(ctx, &self.point)?;
        let p = math::point_of(&r.geom).map_err(|e| {
            ctx.highlight(self.point.entity().cloned());
            FeatureError::from(e)
        })?;
        Ok(Transform::from_translation(p)
            * Transform::from_scale(f)
            * Transform::from_translation(-p))
    }
}

impl Default for Scale {
    fn default() -> Scale {
        Scale::uniform(Vec::new(), "1")
    }
}

impl Feature for Scale {
    fn inputs(&self) -> Vec<Dependency> {
        let mut d: Vec<Dependency> = Vec::new();
        for b in &self.bodies {
            let dep = Dependency::Body(*b);
            if !d.contains(&dep) {
                d.push(dep);
            }
        }
        d.extend(super::pattern::ref_dependency(&self.point));
        d
    }

    fn input_is_optional(&self, dep: &Dependency) -> bool {
        matches!(dep, Dependency::Body(_))
    }

    fn exprs(&self) -> Vec<&Expr> {
        match &self.kind {
            ScaleKind::Uniform(e) => vec![e],
            ScaleKind::NonUniform(v) => v.iter().collect(),
        }
    }

    fn exprs_mut(&mut self) -> Vec<(&mut Expr, Dim)> {
        match &mut self.kind {
            ScaleKind::Uniform(e) => vec![(e, Dim::NONE)],
            ScaleKind::NonUniform(v) => v.iter_mut().map(|e| (e, Dim::NONE)).collect(),
        }
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        if self.bodies.is_empty() {
            return Err(invalid("bodies", "select bodies to scale"));
        }
        let t = self.transform(ctx)?;
        let k = op_kernel(ctx);
        let mut out = FeatureOutput::default();
        let mut done: Vec<BodyId> = Vec::new();
        for &id in &self.bodies {
            ctx.check_cancelled()?;
            let Some(p) = live_body(ctx, id, &mut out)? else {
                continue;
            };
            if done.contains(&p.id) {
                continue;
            }
            done.push(p.id);
            let r = p
                .shape
                .gtransformed(&t)
                .map_err(|e| kernel_failure(ctx, Some(p.id), e, &mut out))?;
            let mut b = ctx.naming_builder();
            b.from_history(&[(0, OperandNaming::Named(p.naming))], &r.history);
            let naming = finish_naming(b, k, &r.shape, &mut out)?;
            out.bodies.push(BodyChange {
                id: p.id,
                change: BodyChangeKind::Modified {
                    geom: solid_geom(r.shape),
                    naming: Some(naming),
                },
            });
        }
        if done.is_empty() {
            ctx.highlight(self.bodies.iter().map(|b| EntityRef::Body(*b)));
            return Err(invalid("bodies", "no body is left to scale"));
        }
        Ok(out)
    }
}

register_feature!(Scale, "solid.scale", version = 1);
