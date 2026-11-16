//! `mesh.smooth`: Laplacian, Taubin and bilateral-normal smoothing (MESH-08) as a feature.

use amg_core::Dimension;
use amg_doc::{BodyId, Dependency, Expr, Feature, FeatureError, FeatureOutput, RecomputeCtx};
use amg_mesh::{LaplacianWeights, SmoothMethod, SmoothOpts, smooth};
use serde::{Deserialize, Serialize};

use super::base::{check_range, emit, group_faces, input_mesh, same_or_new, with_job};
use super::{MeshEntityRef, MeshFeatureError, OutputMode};

/// Smoothing algorithm of [`MeshSmoothFeature`]; numbers are plain-number expressions.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum SmoothMethodParam {
    /// Explicit Laplacian steps (shrinks the mesh).
    Laplacian {
        /// Step size in `(0, 1]`.
        lambda: Expr,
        /// Cotangent instead of uniform weights.
        cotangent: bool,
    },
    /// Taubin λ|μ (volume-preserving low-pass).
    Taubin {
        /// Shrinking step in `(0, 1]`.
        lambda: Expr,
        /// Inflating step, below `-lambda`.
        mu: Expr,
    },
    /// Feature-preserving bilateral filtering of face normals.
    BilateralNormal {
        /// Range sigma on the normal difference (> 0), e.g. 0.35.
        sigma_normal: Expr,
        /// Normal filtering passes per iteration.
        normal_iterations: u32,
        /// Vertex update passes per iteration.
        vertex_iterations: u32,
    },
}

/// Parameters of [`MeshSmoothFeature`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SmoothParams {
    /// Algorithm.
    pub method: SmoothMethodParam,
    /// Outer iterations (at least 1).
    pub iterations: u32,
    /// Keep boundary vertices fixed.
    pub fix_boundary: bool,
    /// Keep vertices fixed that touch more than one face group.
    pub preserve_group_borders: bool,
    /// Smooth only these face groups (`None`: the whole body).
    pub groups: Option<Vec<MeshEntityRef>>,
}

impl Default for SmoothParams {
    /// Ten Taubin 0.5 / -0.53 rounds with fixed boundaries.
    fn default() -> Self {
        SmoothParams {
            method: SmoothMethodParam::Taubin {
                lambda: Expr::Inline("0.5".into()),
                mu: Expr::Inline("-0.53".into()),
            },
            iterations: 10,
            fix_boundary: true,
            preserve_group_borders: false,
            groups: None,
        }
    }
}

/// Smooths a mesh body. Topology, attributes and face groups are unchanged; only vertex
/// positions move.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MeshSmoothFeature {
    /// The mesh body.
    pub body: BodyId,
    /// How to smooth.
    pub params: SmoothParams,
    /// Replace the body or create a new one.
    pub output: OutputMode,
}

impl MeshSmoothFeature {
    /// Default smoothing (Taubin) of `body`, replacing it.
    pub fn new(body: BodyId) -> MeshSmoothFeature {
        MeshSmoothFeature {
            body,
            params: SmoothParams::default(),
            output: OutputMode::Replace,
        }
    }
}

impl Feature for MeshSmoothFeature {
    fn inputs(&self) -> Vec<Dependency> {
        vec![Dependency::Body(self.body)]
    }

    fn exprs(&self) -> Vec<&Expr> {
        match &self.params.method {
            SmoothMethodParam::Laplacian { lambda, .. } => vec![lambda],
            SmoothMethodParam::Taubin { lambda, mu } => vec![lambda, mu],
            SmoothMethodParam::BilateralNormal { sigma_normal, .. } => vec![sigma_normal],
        }
    }

    fn exprs_mut(&mut self) -> Vec<(&mut Expr, Dimension)> {
        match &mut self.params.method {
            SmoothMethodParam::Laplacian { lambda, .. } => vec![(lambda, Dimension::NONE)],
            SmoothMethodParam::Taubin { lambda, mu } => {
                vec![(lambda, Dimension::NONE), (mu, Dimension::NONE)]
            }
            SmoothMethodParam::BilateralNormal { sigma_normal, .. } => {
                vec![(sigma_normal, Dimension::NONE)]
            }
        }
    }

    fn validate(&self) -> Result<(), FeatureError> {
        let bad = |field, msg: &str| FeatureError::InvalidValue {
            field,
            msg: msg.to_owned(),
        };
        if self.params.iterations == 0 {
            return Err(bad("params.iterations", "must be at least 1"));
        }
        if let SmoothMethodParam::BilateralNormal {
            normal_iterations,
            vertex_iterations,
            ..
        } = self.params.method
            && (normal_iterations == 0 || vertex_iterations == 0)
        {
            return Err(bad("params.method", "bilateral passes must be at least 1"));
        }
        match &self.params.groups {
            Some(g) if g.is_empty() => Err(bad(
                "params.groups",
                "an empty list smooths nothing; use None for the whole body",
            )),
            _ => Ok(()),
        }
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let p = &self.params;
        let step = |e: &Expr| {
            check_range(
                "params.method.lambda",
                ctx.eval_scalar(e)?,
                0.0,
                false,
                1.0,
                true,
            )
        };
        let method = match &p.method {
            SmoothMethodParam::Laplacian { lambda, cotangent } => SmoothMethod::Laplacian {
                lambda: step(lambda)?,
                weights: if *cotangent {
                    LaplacianWeights::Cotangent
                } else {
                    LaplacianWeights::Uniform
                },
            },
            SmoothMethodParam::Taubin { lambda, mu } => {
                let lambda = step(lambda)?;
                let mu = check_range(
                    "params.method.mu",
                    ctx.eval_scalar(mu)?,
                    f64::NEG_INFINITY,
                    false,
                    -lambda,
                    false,
                )?;
                SmoothMethod::Taubin { lambda, mu }
            }
            SmoothMethodParam::BilateralNormal {
                sigma_normal,
                normal_iterations,
                vertex_iterations,
            } => SmoothMethod::BilateralNormal {
                sigma_normal: check_range(
                    "params.method.sigma_normal",
                    ctx.eval_scalar(sigma_normal)?,
                    0.0,
                    false,
                    f64::INFINITY,
                    false,
                )?,
                normal_iterations: *normal_iterations,
                vertex_iterations: *vertex_iterations,
            },
        };
        let input = input_mesh(ctx, self.body)?;
        let mut out = FeatureOutput::default();
        let selection = match &p.groups {
            Some(refs) => Some(group_faces(ctx, &input, refs, &mut out)?),
            None => None,
        };
        let opts = SmoothOpts {
            method,
            iterations: p.iterations,
            fix_boundary: p.fix_boundary,
            preserve_group_borders: p.preserve_group_borders,
            selection,
        };
        ctx.progress(0.0, "smoothing mesh");
        let mesh =
            with_job(ctx, |job| smooth(&input, &opts, job)).map_err(MeshFeatureError::from)?;
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

amg_doc::register_feature!(MeshSmoothFeature, "mesh.smooth", version = 1);
