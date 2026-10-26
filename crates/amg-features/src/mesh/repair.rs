//! `mesh.repair`: the MESH-04 repair pipeline as a timeline feature.

use std::sync::Arc;

use amg_doc::{
    BodyId, Dependency, Diagnostic, Expr, Feature, FeatureError, FeatureOutput, RecomputeCtx,
};
use amg_jobs::JobCtx;
use amg_mesh::{
    HoleFillOpts, IndexedMesh, RepairOpts, RepairReport, SmallShellFilter, UnresolvedIssue,
    WeldOpts, repair,
};
use serde::{Deserialize, Serialize};

use super::base::{emit, input_mesh};
use super::{MeshFeatureError, OutputMode};

/// Debris filter of the repair pipeline (see [`SmallShellFilter`]).
#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SmallShellParams {
    /// Shells with at most this many faces are debris candidates.
    pub max_faces: u32,
    /// ... if their size is below this fraction of the largest shell's.
    pub max_volume_fraction: f64,
}

/// Parameters of [`MeshRepairFeature`]; the defaults equal [`RepairOpts::default`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RepairParams {
    /// Merge vertices on boundary edges within `weld_tolerance`.
    pub weld: bool,
    /// Weld distance (a length).
    pub weld_tolerance: Expr,
    /// Remove faces with zero area or a repeated vertex.
    pub remove_degenerate: bool,
    /// Remove faces repeated with the same vertex set.
    pub remove_duplicates: bool,
    /// Split non-manifold edges and vertices.
    pub fix_non_manifold: bool,
    /// Remove debris shells (`None`: keep them).
    pub remove_small_shells: Option<SmallShellParams>,
    /// Make the orientation consistent and outward.
    pub orient: bool,
    /// Fill boundary loops (defaults of [`HoleFillOpts`]).
    pub fill_holes: bool,
    /// Remove self-intersections.
    pub remove_self_intersections: bool,
    /// Pipeline rounds at most.
    pub max_rounds: u32,
}

impl Default for RepairParams {
    fn default() -> Self {
        let o = RepairOpts::default();
        RepairParams {
            weld: o.weld.is_some(),
            weld_tolerance: Expr::Inline("0.00001 mm".into()),
            remove_degenerate: o.remove_degenerate,
            remove_duplicates: o.remove_duplicates,
            fix_non_manifold: o.fix_non_manifold,
            remove_small_shells: o.remove_small_shells.map(|s| SmallShellParams {
                max_faces: u32::try_from(s.max_faces).unwrap_or(u32::MAX),
                max_volume_fraction: s.max_volume_fraction,
            }),
            orient: o.orient,
            fill_holes: o.fill_holes.is_some(),
            remove_self_intersections: o.remove_self_intersections,
            max_rounds: o.max_rounds,
        }
    }
}

impl RepairParams {
    /// The library options, with `weld_tolerance` already evaluated (mm).
    fn to_opts(&self, weld_tolerance: f64) -> RepairOpts {
        RepairOpts {
            weld: self.weld.then_some(WeldOpts {
                tolerance: weld_tolerance,
            }),
            remove_degenerate: self.remove_degenerate,
            remove_duplicates: self.remove_duplicates,
            fix_non_manifold: self.fix_non_manifold,
            remove_small_shells: self.remove_small_shells.map(|s| SmallShellFilter {
                max_faces: s.max_faces as usize,
                max_volume_fraction: s.max_volume_fraction,
            }),
            orient: self.orient,
            fill_holes: self.fill_holes.then(HoleFillOpts::default),
            remove_self_intersections: self.remove_self_intersections,
            max_rounds: self.max_rounds,
        }
    }
}

/// Repairs a mesh body (weld, degenerate and duplicate faces, non-manifold, debris, orientation,
/// holes, self-intersections). Face-group ids are kept; hole patches get a new `Fill` group.
/// A mesh that needs no repair comes back as the same allocation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MeshRepairFeature {
    /// The mesh body to repair.
    pub body: BodyId,
    /// What to repair.
    pub opts: RepairParams,
    /// Replace the body or create a new one.
    pub output: OutputMode,
}

impl MeshRepairFeature {
    /// Repair with the default options, replacing the body.
    pub fn new(body: BodyId) -> MeshRepairFeature {
        MeshRepairFeature {
            body,
            opts: RepairParams::default(),
            output: OutputMode::Replace,
        }
    }
}

/// Runs the pipeline; returns the input `Arc` unchanged when the result is identical.
fn run(
    m: &Arc<IndexedMesh>,
    opts: &RepairOpts,
    job: &JobCtx,
) -> Result<(Arc<IndexedMesh>, RepairReport), MeshFeatureError> {
    let (out, report) = repair(m, opts, job)?;
    if out.content_hash() == m.content_hash() {
        Ok((Arc::clone(m), report))
    } else {
        Ok((Arc::new(out), report))
    }
}

fn warnings(report: &RepairReport) -> Vec<Diagnostic> {
    report
        .unresolved
        .iter()
        .map(|u| Diagnostic {
            message: match u {
                UnresolvedIssue::HoleTooLarge { boundary_edges } => {
                    format!("repair left an open boundary loop of {boundary_edges} edges")
                }
                UnresolvedIssue::SelfIntersections { pairs } => {
                    format!("repair left {pairs} self-intersecting face pairs")
                }
                UnresolvedIssue::NonManifold { edges } => {
                    format!("repair left {edges} non-manifold edges")
                }
            },
            cause: None,
            entities: Vec::new(),
        })
        .collect()
}

impl Feature for MeshRepairFeature {
    fn inputs(&self) -> Vec<Dependency> {
        vec![Dependency::Body(self.body)]
    }

    fn exprs(&self) -> Vec<&Expr> {
        vec![&self.opts.weld_tolerance]
    }

    fn validate(&self) -> Result<(), FeatureError> {
        let bad = |field, msg: &str| FeatureError::InvalidValue {
            field,
            msg: msg.to_owned(),
        };
        if self.opts.max_rounds == 0 {
            return Err(bad("opts.max_rounds", "must be at least 1"));
        }
        if let Some(s) = &self.opts.remove_small_shells
            && !(s.max_volume_fraction.is_finite() && s.max_volume_fraction >= 0.0)
        {
            return Err(bad(
                "opts.remove_small_shells",
                "max_volume_fraction must be finite and not negative",
            ));
        }
        Ok(())
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let tol = ctx.eval_length(&self.opts.weld_tolerance)?;
        if !(tol.is_finite() && tol >= 0.0) {
            return Err(FeatureError::InvalidValue {
                field: "opts.weld_tolerance",
                msg: format!("must be finite and not negative, got {tol}"),
            });
        }
        let input = input_mesh(ctx, self.body)?;
        let detached = JobCtx::detached();
        let job = ctx.job().unwrap_or(&detached);
        ctx.progress(0.0, "repairing mesh");
        let (mesh, report) =
            run(&input, &self.opts.to_opts(tol), job).map_err(FeatureError::from)?;
        let mut out = FeatureOutput {
            warnings: warnings(&report),
            ..FeatureOutput::default()
        };
        emit(ctx, self.body, self.output, mesh, &mut out);
        Ok(out)
    }
}

amg_doc::register_feature!(MeshRepairFeature, "mesh.repair", version = 1);
