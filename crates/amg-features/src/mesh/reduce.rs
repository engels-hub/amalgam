//! `mesh.reduce`: QEM decimation (MESH-07) as a feature.

use amg_core::Dimension;
use amg_doc::{BodyId, Dependency, Expr, Feature, FeatureError, FeatureOutput, RecomputeCtx};
use amg_mesh::{DecimateOpts, DecimateTarget, decimate};
use serde::{Deserialize, Serialize};

use super::base::{check_range, emit, input_mesh, positive_length, same_or_new, warning, with_job};
use super::{MeshFeatureError, OutputMode};

/// What [`MeshReduceFeature`] aims for.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ReduceTarget {
    /// Stop at this many faces (a plain number, rounded; at least 4).
    FaceCount(Expr),
    /// Stop at this fraction of the input face count (plain number in `(0, 1]`).
    Ratio(Expr),
    /// Collapse while the error stays below this length.
    MaxError(Expr),
}

/// Parameters of [`MeshReduceFeature`]; the defaults equal [`DecimateOpts::default`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReduceParams {
    /// Goal.
    pub target: ReduceTarget,
    /// Hard cap on the error of a collapse (a length), combined with `target`.
    pub max_error: Option<Expr>,
    /// Keep open boundaries in place.
    pub preserve_boundary: bool,
    /// Keep borders between face groups in place.
    pub preserve_group_borders: bool,
    /// Edges sharper than this angle are protected (`None`: no feature edges).
    pub feature_angle: Option<Expr>,
    /// Penalty weight of protected edges (plain number, > 0).
    pub boundary_weight: Expr,
    /// Smallest triangle quality a collapse may create (plain number in `[0, 1]`).
    pub min_triangle_quality: Expr,
    /// Keep vertex colours and corner UVs.
    pub preserve_attributes: bool,
}

impl Default for ReduceParams {
    fn default() -> Self {
        let o = DecimateOpts::default();
        ReduceParams {
            target: ReduceTarget::Ratio(Expr::Inline("0.5".into())),
            max_error: None,
            preserve_boundary: o.preserve_boundary,
            preserve_group_borders: o.preserve_group_borders,
            feature_angle: Some(Expr::Inline("45 deg".into())),
            boundary_weight: Expr::Inline("1000".into()),
            min_triangle_quality: Expr::Inline("0.1".into()),
            preserve_attributes: o.preserve_attributes,
        }
    }
}

/// Reduces the face count of a mesh body (Garland–Heckbert QEM with boundary, group-border and
/// feature penalties). Face groups are kept.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MeshReduceFeature {
    /// The mesh body.
    pub body: BodyId,
    /// How to reduce.
    pub params: ReduceParams,
    /// Replace the body or create a new one.
    pub output: OutputMode,
}

impl MeshReduceFeature {
    /// Reduce to `ratio` (a plain-number expression) of the faces, replacing the body.
    pub fn ratio(body: BodyId, ratio: &str) -> MeshReduceFeature {
        MeshReduceFeature {
            body,
            params: ReduceParams {
                target: ReduceTarget::Ratio(Expr::Inline(ratio.into())),
                ..ReduceParams::default()
            },
            output: OutputMode::Replace,
        }
    }
}

impl Feature for MeshReduceFeature {
    fn inputs(&self) -> Vec<Dependency> {
        vec![Dependency::Body(self.body)]
    }

    fn exprs(&self) -> Vec<&Expr> {
        let p = &self.params;
        let mut v = vec![match &p.target {
            ReduceTarget::FaceCount(e) | ReduceTarget::Ratio(e) | ReduceTarget::MaxError(e) => e,
        }];
        v.extend(p.max_error.as_ref());
        v.extend(p.feature_angle.as_ref());
        v.push(&p.boundary_weight);
        v.push(&p.min_triangle_quality);
        v
    }

    fn exprs_mut(&mut self) -> Vec<(&mut Expr, Dimension)> {
        let p = &mut self.params;
        let mut v = vec![match &mut p.target {
            ReduceTarget::FaceCount(e) | ReduceTarget::Ratio(e) => (e, Dimension::NONE),
            ReduceTarget::MaxError(e) => (e, Dimension::LENGTH),
        }];
        v.extend(p.max_error.as_mut().map(|e| (e, Dimension::LENGTH)));
        v.extend(p.feature_angle.as_mut().map(|e| (e, Dimension::ANGLE)));
        v.push((&mut p.boundary_weight, Dimension::NONE));
        v.push((&mut p.min_triangle_quality, Dimension::NONE));
        v
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let p = &self.params;
        let target = match &p.target {
            ReduceTarget::FaceCount(e) => {
                let n = check_range(
                    "params.target",
                    ctx.eval_scalar(e)?.round(),
                    4.0,
                    true,
                    f64::from(u32::MAX),
                    true,
                )?;
                // In range [4, u32::MAX] after the check, so the cast is exact.
                #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                DecimateTarget::FaceCount(n as usize)
            }
            ReduceTarget::Ratio(e) => DecimateTarget::Ratio(check_range(
                "params.target",
                ctx.eval_scalar(e)?,
                0.0,
                false,
                1.0,
                true,
            )?),
            ReduceTarget::MaxError(e) => {
                DecimateTarget::MaxError(positive_length(ctx, e, "params.target")?)
            }
        };
        let max_error = match &p.max_error {
            Some(e) => Some(positive_length(ctx, e, "params.max_error")?),
            None => None,
        };
        let feature_angle = match &p.feature_angle {
            Some(e) => Some(check_range(
                "params.feature_angle",
                ctx.eval_angle(e)?,
                0.0,
                false,
                std::f64::consts::PI,
                true,
            )?),
            None => None,
        };
        let opts = DecimateOpts {
            target,
            max_error,
            preserve_boundary: p.preserve_boundary,
            preserve_group_borders: p.preserve_group_borders,
            feature_angle,
            boundary_weight: check_range(
                "params.boundary_weight",
                ctx.eval_scalar(&p.boundary_weight)?,
                0.0,
                false,
                f64::INFINITY,
                false,
            )?,
            min_triangle_quality: check_range(
                "params.min_triangle_quality",
                ctx.eval_scalar(&p.min_triangle_quality)?,
                0.0,
                true,
                1.0,
                true,
            )?,
            preserve_attributes: p.preserve_attributes,
        };
        let input = input_mesh(ctx, self.body)?;
        ctx.progress(0.0, "reducing mesh");
        let (mesh, report) =
            with_job(ctx, |job| decimate(&input, &opts, job)).map_err(MeshFeatureError::from)?;
        let mut out = FeatureOutput::default();
        if !report.dropped_attributes.is_empty() {
            out.warnings.push(warning(format!(
                "reduce dropped these attributes: {}",
                report.dropped_attributes.join(", ")
            )));
        }
        let wanted = match target {
            DecimateTarget::FaceCount(n) => Some(n),
            #[allow(
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss,
                clippy::cast_precision_loss
            )]
            DecimateTarget::Ratio(r) => Some((r * report.faces_before as f64).ceil() as usize),
            DecimateTarget::MaxError(_) => None,
        };
        // MESH-07 may end one face below a count target; more than 10 % above it means the
        // constraints (boundaries, features, quality, max_error) stopped the collapses.
        if let Some(n) = wanted
            && report.faces_after * 10 > n.max(4) * 11
            && max_error.is_none()
        {
            out.warnings.push(warning(format!(
                "reduce stopped at {} faces, target {n}: the remaining collapses would break \
                 a protected edge or the triangle quality",
                report.faces_after
            )));
        }
        emit(
            ctx,
            self.body,
            self.output,
            same_or_new(&input, mesh),
            &mut out,
        );
        Ok(out)
    }
}

amg_doc::register_feature!(MeshReduceFeature, "mesh.reduce", version = 1);
