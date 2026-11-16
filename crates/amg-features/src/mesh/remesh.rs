//! `mesh.remesh`: isotropic / curvature-adaptive remesh (MESH-06, MESH-22) as a feature.

use amg_core::Dimension;
use amg_doc::{BodyId, Dependency, Expr, Feature, FeatureError, FeatureOutput, RecomputeCtx};
use amg_mesh::{EdgeLength, RemeshOpts, Sizing, remesh};
use serde::{Deserialize, Serialize};

use super::base::{
    check_range, emit, group_faces, input_mesh, positive_length, same_or_new, with_job,
};
use super::{MeshEntityRef, MeshFeatureError, OutputMode};

/// Target edge length of a uniform remesh.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum EdgeLengthParam {
    /// A length.
    Absolute(Expr),
    /// A factor (plain number) times the mean edge length of the input.
    RelativeToMean(Expr),
}

/// Sizing field of [`MeshRemeshFeature`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum RemeshSizing {
    /// One target edge length everywhere (MESH-06).
    Uniform(EdgeLengthParam),
    /// Curvature-adaptive edge length (MESH-22); all three are lengths.
    Adaptive {
        /// Largest deviation from the input.
        tolerance: Expr,
        /// Shortest edge.
        min_length: Expr,
        /// Longest edge.
        max_length: Expr,
    },
}

/// Parameters of [`MeshRemeshFeature`]; the defaults equal [`RemeshOpts::default`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RemeshParams {
    /// Target edge length.
    pub sizing: RemeshSizing,
    /// Split / collapse / flip / relax rounds (at least 1).
    pub iterations: u32,
    /// Edges sharper than this angle are kept (`None`: no feature edges).
    pub feature_angle: Option<Expr>,
    /// Keep open boundaries on their input curve.
    pub preserve_boundary: bool,
    /// Keep borders between face groups on their input curves.
    pub preserve_group_borders: bool,
    /// Project vertices back onto the input surface.
    pub project_to_input: bool,
    /// Remesh only these face groups (`None`: the whole body).
    pub groups: Option<Vec<MeshEntityRef>>,
}

impl Default for RemeshParams {
    fn default() -> Self {
        let o = RemeshOpts::default();
        RemeshParams {
            sizing: RemeshSizing::Uniform(EdgeLengthParam::RelativeToMean(Expr::Inline(
                "1".into(),
            ))),
            iterations: o.iterations,
            feature_angle: Some(Expr::Inline("45 deg".into())),
            preserve_boundary: o.preserve_boundary,
            preserve_group_borders: o.preserve_group_borders,
            project_to_input: o.project_to_input,
            groups: None,
        }
    }
}

/// Remeshes a mesh body towards a target edge length with feature edges, group borders and
/// boundaries kept. Face groups are inherited by the new faces.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MeshRemeshFeature {
    /// The mesh body.
    pub body: BodyId,
    /// How to remesh.
    pub params: RemeshParams,
    /// Replace the body or create a new one.
    pub output: OutputMode,
}

impl MeshRemeshFeature {
    /// Uniform remesh to `edge_length` (a length expression), replacing the body.
    pub fn new(body: BodyId, edge_length: &str) -> MeshRemeshFeature {
        MeshRemeshFeature {
            body,
            params: RemeshParams {
                sizing: RemeshSizing::Uniform(EdgeLengthParam::Absolute(Expr::Inline(
                    edge_length.into(),
                ))),
                ..RemeshParams::default()
            },
            output: OutputMode::Replace,
        }
    }
}

impl Feature for MeshRemeshFeature {
    fn inputs(&self) -> Vec<Dependency> {
        vec![Dependency::Body(self.body)]
    }

    fn exprs(&self) -> Vec<&Expr> {
        let mut v = match &self.params.sizing {
            RemeshSizing::Uniform(EdgeLengthParam::Absolute(e))
            | RemeshSizing::Uniform(EdgeLengthParam::RelativeToMean(e)) => vec![e],
            RemeshSizing::Adaptive {
                tolerance,
                min_length,
                max_length,
            } => vec![tolerance, min_length, max_length],
        };
        v.extend(self.params.feature_angle.as_ref());
        v
    }

    fn exprs_mut(&mut self) -> Vec<(&mut Expr, Dimension)> {
        let mut v = match &mut self.params.sizing {
            RemeshSizing::Uniform(EdgeLengthParam::Absolute(e)) => vec![(e, Dimension::LENGTH)],
            RemeshSizing::Uniform(EdgeLengthParam::RelativeToMean(e)) => {
                vec![(e, Dimension::NONE)]
            }
            RemeshSizing::Adaptive {
                tolerance,
                min_length,
                max_length,
            } => vec![
                (tolerance, Dimension::LENGTH),
                (min_length, Dimension::LENGTH),
                (max_length, Dimension::LENGTH),
            ],
        };
        v.extend(
            self.params
                .feature_angle
                .as_mut()
                .map(|e| (e, Dimension::ANGLE)),
        );
        v
    }

    fn validate(&self) -> Result<(), FeatureError> {
        if self.params.iterations == 0 {
            return Err(FeatureError::InvalidValue {
                field: "params.iterations",
                msg: "must be at least 1".to_owned(),
            });
        }
        match &self.params.groups {
            Some(g) if g.is_empty() => Err(FeatureError::InvalidValue {
                field: "params.groups",
                msg: "an empty list remeshes nothing; use None for the whole body".to_owned(),
            }),
            _ => Ok(()),
        }
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let p = &self.params;
        let sizing = match &p.sizing {
            RemeshSizing::Uniform(EdgeLengthParam::Absolute(e)) => Sizing::Uniform(
                EdgeLength::Absolute(positive_length(ctx, e, "params.sizing")?),
            ),
            RemeshSizing::Uniform(EdgeLengthParam::RelativeToMean(e)) => {
                Sizing::Uniform(EdgeLength::RelativeToMean(check_range(
                    "params.sizing",
                    ctx.eval_scalar(e)?,
                    0.0,
                    false,
                    f64::INFINITY,
                    false,
                )?))
            }
            RemeshSizing::Adaptive {
                tolerance,
                min_length,
                max_length,
            } => {
                let tolerance = positive_length(ctx, tolerance, "params.sizing.tolerance")?;
                let min_length = positive_length(ctx, min_length, "params.sizing.min_length")?;
                let max_length = positive_length(ctx, max_length, "params.sizing.max_length")?;
                if min_length > max_length {
                    return Err(FeatureError::InvalidValue {
                        field: "params.sizing",
                        msg: format!(
                            "min_length {min_length} mm exceeds max_length {max_length} mm"
                        ),
                    });
                }
                Sizing::Adaptive {
                    tolerance,
                    min_length,
                    max_length,
                }
            }
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
        let input = input_mesh(ctx, self.body)?;
        let mut out = FeatureOutput::default();
        let selection = match &p.groups {
            Some(refs) => Some(group_faces(ctx, &input, refs, &mut out)?),
            None => None,
        };
        let opts = RemeshOpts {
            sizing,
            iterations: p.iterations,
            feature_angle,
            preserve_boundary: p.preserve_boundary,
            preserve_group_borders: p.preserve_group_borders,
            project_to_input: p.project_to_input,
            selection,
        };
        ctx.progress(0.0, "remeshing");
        let mesh =
            with_job(ctx, |job| remesh(&input, &opts, job)).map_err(MeshFeatureError::from)?;
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

amg_doc::register_feature!(MeshRemeshFeature, "mesh.remesh", version = 1);
