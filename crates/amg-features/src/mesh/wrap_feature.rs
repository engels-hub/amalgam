//! `mesh.wrap`: replace a hopeless mesh body by a watertight envelope (MESH-18).

use std::sync::Arc;

use amg_core::Dimension;
use amg_doc::{BodyId, Dependency, Expr, Feature, FeatureError, FeatureOutput, RecomputeCtx};
use serde::{Deserialize, Serialize};

use super::OutputMode;
use super::base::{emit, input_mesh, positive_length, warning, with_job};
use super::wrap::{WrapOpts, wrap};

/// A voxel below this fraction of the bounding-box diagonal gets a warning (a large lattice:
/// slow and memory hungry).
pub const WRAP_FINE_VOXEL_PER_DIAGONAL: f64 = 1.0 / 2000.0;

/// Wraps a mesh body: the body becomes the morphological closing of its surface at `voxel`
/// resolution (dilate by `offset`, keep the outer component, erode). Always closed, manifold
/// and free of self-intersections, whatever the input's defects; gaps and holes narrower than
/// about twice the offset are closed.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MeshWrapFeature {
    /// The mesh body.
    pub body: BodyId,
    /// Lattice spacing (a length, > 0).
    pub voxel: Expr,
    /// Dilation radius (a length, > 0). `None`: 1.5 voxels.
    #[serde(default)]
    pub offset: Option<Expr>,
    /// Replace the body or create a new one.
    #[serde(default)]
    pub output: OutputMode,
}

impl MeshWrapFeature {
    /// A wrap of `body` at `voxel` (a length expression) with the default offset, replacing
    /// the body.
    pub fn new(body: BodyId, voxel: &str) -> MeshWrapFeature {
        MeshWrapFeature {
            body,
            voxel: Expr::Inline(voxel.into()),
            offset: None,
            output: OutputMode::Replace,
        }
    }
}

impl Feature for MeshWrapFeature {
    fn inputs(&self) -> Vec<Dependency> {
        vec![Dependency::Body(self.body)]
    }

    fn exprs(&self) -> Vec<&Expr> {
        let mut v = vec![&self.voxel];
        v.extend(self.offset.as_ref());
        v
    }

    fn exprs_mut(&mut self) -> Vec<(&mut Expr, Dimension)> {
        let mut v = vec![(&mut self.voxel, Dimension::LENGTH)];
        v.extend(self.offset.as_mut().map(|e| (e, Dimension::LENGTH)));
        v
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let h = positive_length(ctx, &self.voxel, "voxel")?;
        let offset = match &self.offset {
            Some(e) => Some(positive_length(ctx, e, "offset")?),
            None => None,
        };
        let input = input_mesh(ctx, self.body)?;
        let mut out = FeatureOutput::default();
        let diag = input.aabb().size().length();
        if h < WRAP_FINE_VOXEL_PER_DIAGONAL * diag {
            out.warnings.push(warning(format!(
                "the voxel ({h} mm) is under 1/2000 of the body's size ({diag:.1} mm); the \
                 wrap is slow and needs a lot of memory"
            )));
        }
        let opts = WrapOpts {
            voxel_size: h,
            offset,
            simplify_tolerance: None,
        };
        ctx.progress(0.0, "wrap");
        let mesh = with_job(ctx, |job| wrap(&input, &opts, job))?;
        emit(ctx, self.body, self.output, Arc::new(mesh), &mut out);
        Ok(out)
    }
}

amg_doc::register_feature!(MeshWrapFeature, "mesh.wrap", version = 1);
