//! `mesh.face_groups`: segment a mesh body into face groups (MESH-12).

use amg_doc::{
    BodyId, Dependency, Diagnostic, Expr, Feature, FeatureError, FeatureOutput, RecomputeCtx,
};
use amg_jobs::JobCtx;
use amg_mesh::{SegmentOpts, segment_face_groups};
use serde::{Deserialize, Serialize};

use super::base::{emit, input_mesh};
use super::{MeshFeatureError, OutputMode};

/// Parameters of [`MeshFaceGroupsFeature`]; the defaults equal [`SegmentOpts::default`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SegmentParams {
    /// Largest angle between the normals of adjacent faces of one group (an angle).
    pub angle_threshold: Expr,
    /// Largest curvature difference (1/mm, a plain number) between a face and the seed of its
    /// group; `None`: curvature only orders the seeds.
    pub curvature_threshold: Option<Expr>,
    /// Groups with fewer faces are merged into a neighbour (`0` or `1`: no merging).
    pub min_group_faces: u32,
}

impl Default for SegmentParams {
    fn default() -> Self {
        SegmentParams {
            angle_threshold: Expr::Inline("30 deg".into()),
            curvature_threshold: None,
            min_group_faces: u32::try_from(SegmentOpts::default().min_group_faces)
                .unwrap_or(u32::MAX),
        }
    }
}

/// Splits a mesh body into face groups by region growing on normals and curvature, merging
/// small regions. The groups replace the body's earlier ones (their ids stay unused in the
/// table, so a reference to an old group resolves by fingerprint, with a warning).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MeshFaceGroupsFeature {
    /// The mesh body to segment.
    pub body: BodyId,
    /// How to segment.
    pub opts: SegmentParams,
    /// Replace the body or create a new one.
    pub output: OutputMode,
}

impl MeshFaceGroupsFeature {
    /// Segmentation with the default options, replacing the body.
    pub fn new(body: BodyId) -> MeshFaceGroupsFeature {
        MeshFaceGroupsFeature {
            body,
            opts: SegmentParams::default(),
            output: OutputMode::Replace,
        }
    }
}

impl Feature for MeshFaceGroupsFeature {
    fn inputs(&self) -> Vec<Dependency> {
        vec![Dependency::Body(self.body)]
    }

    fn exprs(&self) -> Vec<&Expr> {
        let mut v = vec![&self.opts.angle_threshold];
        v.extend(self.opts.curvature_threshold.as_ref());
        v
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let angle = ctx.eval_angle(&self.opts.angle_threshold)?;
        if !(angle.is_finite() && angle > 0.0 && angle < std::f64::consts::PI) {
            return Err(FeatureError::InvalidValue {
                field: "opts.angle_threshold",
                msg: format!("must be between 0 and 180 degrees, got {angle} rad"),
            });
        }
        let curvature = match &self.opts.curvature_threshold {
            Some(e) => {
                let c = ctx.eval_scalar(e)?;
                if !(c.is_finite() && c >= 0.0) {
                    return Err(FeatureError::InvalidValue {
                        field: "opts.curvature_threshold",
                        msg: format!("must be finite and not negative, got {c}"),
                    });
                }
                Some(c)
            }
            None => None,
        };
        let opts = SegmentOpts {
            angle_threshold: angle,
            curvature_threshold: curvature,
            min_group_faces: self.opts.min_group_faces as usize,
        };
        let input = input_mesh(ctx, self.body)?;
        let detached = JobCtx::detached();
        let job = ctx.job().unwrap_or(&detached);
        ctx.progress(0.0, "segmenting face groups");
        let mut mesh = (*input).clone();
        let count = segment_face_groups(&mut mesh, &opts, job).map_err(MeshFeatureError::from)?;
        let mut out = FeatureOutput::default();
        let replaced = input
            .groups
            .per_face()
            .iter()
            .collect::<std::collections::BTreeSet<_>>()
            .len();
        if replaced > 1 {
            out.warnings.push(Diagnostic {
                message: format!(
                    "{replaced} existing face groups were replaced by {count} segmented groups"
                ),
                cause: None,
                entities: Vec::new(),
            });
        }
        emit(
            ctx,
            self.body,
            self.output,
            std::sync::Arc::new(mesh),
            &mut out,
        );
        Ok(out)
    }
}

amg_doc::register_feature!(MeshFaceGroupsFeature, "mesh.face_groups", version = 1);
