//! `mesh.shell`: hollow a mesh body with a wall (MESH-10 quick normal offset or robust SDF
//! shell).

use std::sync::Arc;

use amg_core::Dimension;
use amg_doc::{BodyId, Dependency, Expr, Feature, FeatureError, FeatureOutput, RecomputeCtx};
use amg_mesh::{NormalOffsetOpts, ShellSide, offset_shell_normal};
use serde::{Deserialize, Serialize};

use super::base::{emit, input_mesh, positive_length, warning, with_job};
use super::shell::{SdfShellOpts, offset_shell_sdf};
use super::{MeshFeatureError, OutputMode};

/// Default robust-shell voxel: this fraction of the wall thickness ...
pub const DEFAULT_VOXEL_PER_THICKNESS: f64 = 0.25;
/// ... but at least this fraction of the body's bounding-box diagonal (bounds the lattice).
pub const MIN_VOXEL_PER_DIAGONAL: f64 = 1.0 / 400.0;

/// Shells a mesh body: the body becomes a wall of `thickness` on `side` of its surface.
///
/// The quick shell offsets along vertex normals (fast, keeps the input copy exactly, but may
/// self-intersect: its health is a warning with the count then). The robust shell extracts
/// the wall from the signed distance field at `voxel` resolution: always closed, manifold and
/// free of self-intersections, the wall accurate to about one voxel.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MeshShellFeature {
    /// The mesh body.
    pub body: BodyId,
    /// Wall thickness (a length, > 0).
    pub thickness: Expr,
    /// Which side of the surface the wall goes to.
    pub side: ShellSide,
    /// Robust SDF shell instead of the quick normal offset.
    pub robust: bool,
    /// Robust shell lattice spacing (a length). `None`: a quarter of the thickness, but at
    /// least 1/400 of the bounding-box diagonal. Ignored by the quick shell.
    pub voxel: Option<Expr>,
    /// Replace the body or create a new one.
    #[serde(default)]
    pub output: OutputMode,
}

impl MeshShellFeature {
    /// A quick inside shell of `thickness` (a length expression), replacing the body.
    pub fn quick(body: BodyId, thickness: &str) -> MeshShellFeature {
        MeshShellFeature {
            body,
            thickness: Expr::Inline(thickness.into()),
            side: ShellSide::Inside,
            robust: false,
            voxel: None,
            output: OutputMode::Replace,
        }
    }

    /// A robust inside shell of `thickness` with the default voxel, replacing the body.
    pub fn robust(body: BodyId, thickness: &str) -> MeshShellFeature {
        MeshShellFeature {
            robust: true,
            ..MeshShellFeature::quick(body, thickness)
        }
    }
}

impl Feature for MeshShellFeature {
    fn inputs(&self) -> Vec<Dependency> {
        vec![Dependency::Body(self.body)]
    }

    fn exprs(&self) -> Vec<&Expr> {
        let mut v = vec![&self.thickness];
        v.extend(self.voxel.as_ref());
        v
    }

    fn exprs_mut(&mut self) -> Vec<(&mut Expr, Dimension)> {
        let mut v = vec![(&mut self.thickness, Dimension::LENGTH)];
        v.extend(self.voxel.as_mut().map(|e| (e, Dimension::LENGTH)));
        v
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let t = positive_length(ctx, &self.thickness, "thickness")?;
        let voxel = match &self.voxel {
            Some(e) => Some(positive_length(ctx, e, "voxel")?),
            None => None,
        };
        let input = input_mesh(ctx, self.body)?;
        let mut out = FeatureOutput::default();
        let mesh = if self.robust {
            let h = voxel.unwrap_or_else(|| {
                (DEFAULT_VOXEL_PER_THICKNESS * t)
                    .max(MIN_VOXEL_PER_DIAGONAL * input.aabb().size().length())
            });
            if h > 0.5 * t {
                out.warnings.push(warning(format!(
                    "the wall ({t} mm) is thinner than two voxels ({h} mm); it may break up"
                )));
            }
            ctx.progress(0.0, "robust shell");
            with_job(ctx, |job| {
                offset_shell_sdf(&input, &SdfShellOpts::new(t, self.side, h), job)
            })?
        } else {
            let opts = NormalOffsetOpts {
                thickness: t,
                side: self.side,
                open_faces: Vec::new(),
            };
            ctx.progress(0.0, "quick shell");
            let (mesh, report) = with_job(ctx, |job| offset_shell_normal(&input, &opts, job))
                .map_err(MeshFeatureError::from)?;
            if report.self_intersecting_pairs > 0 {
                out.warnings.push(warning(format!(
                    "the quick shell has {} self-intersecting face pairs; use the robust shell \
                     or a thinner wall",
                    report.self_intersecting_pairs
                )));
            }
            mesh
        };
        emit(ctx, self.body, self.output, Arc::new(mesh), &mut out);
        Ok(out)
    }
}

amg_doc::register_feature!(MeshShellFeature, "mesh.shell", version = 1);
