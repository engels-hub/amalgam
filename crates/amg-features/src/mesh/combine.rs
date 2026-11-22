//! `mesh.combine`: join, cut or intersect a mesh target with tool bodies (MESH-27).
//!
//! The boolean is MESH-05's `boolean_n` (Manifold, ADR-003): `target op tools`, where a
//! difference subtracts the union of the tools. Operands that are not closed oriented
//! 2-manifolds are repaired first (`BoolOpts::repair_inputs`) and each repaired operand is a
//! health warning.
//!
//! Operand kinds (architecture §Geometry representations, QUESTIONS #63):
//! - **Mesh target, mesh tools:** a plain mesh boolean.
//! - **Mesh target, solid tool:** the solid is tessellated (S02 BREP-08, `TessQuality::export`)
//!   and the result is a mesh; the feature warns that the solid was converted to mesh. This is
//!   the only "converted to mesh" warning: it applies only to a mesh target.
//! - **Any implicit operand** (target or tool): a boolean with an implicit body stays implicit
//!   (the other operands enter as mesh SDFs; that is S08's Implicit Combine, IMP-08). This
//!   feature never meshes an implicit body: it fails with
//!   [`MeshFeatureError::ImplicitOperand`] and leaves every body as it was.
//! - **Solid or form target:** not a mesh combine ([`MeshFeatureError::NotAMesh`]); solid ∘
//!   solid is the B-rep combine (S02).
//!
//! With `keep_tools` off the tool bodies are deleted by the feature; with it on they stay.

use std::sync::{Arc, OnceLock};

use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, Dependency, Feature, FeatureError, FeatureOutput,
    RecomputeCtx,
};
use amg_mesh::{BoolOp, BoolOpts, IndexedMesh, boolean_n};
use serde::{Deserialize, Serialize};

use super::base::{emit, input_mesh, warning, with_job};
use super::{MeshFeatureError, OutputMode};

/// Most tool bodies one combine takes (Manifold's batch boolean takes 256 operands).
pub const MAX_COMBINE_TOOLS: usize = 255;

/// Joins (`Union`), cuts (`Difference`) or intersects (`Intersection`) a mesh body with tool
/// bodies (the S07 `MeshCombineFeature`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MeshCombineFeature {
    /// The mesh body that receives the result (its face groups keep their ids).
    pub target: BodyId,
    /// Tool bodies: meshes, or solids (tessellated, with a warning). At least one.
    pub tools: Vec<BodyId>,
    /// The operation.
    pub op: BoolOp,
    /// Keep the tool bodies (otherwise the feature deletes them).
    pub keep_tools: bool,
    /// Replace the target (default) or put the result in a new body (slot 0).
    #[serde(default)]
    pub output: OutputMode,
}

impl MeshCombineFeature {
    /// `target op tools`, deleting the tools and replacing the target.
    pub fn new(target: BodyId, tools: Vec<BodyId>, op: BoolOp) -> MeshCombineFeature {
        MeshCombineFeature {
            target,
            tools,
            op,
            keep_tools: false,
            output: OutputMode::Replace,
        }
    }
}

/// The role of an operand in messages.
fn role(index: usize) -> String {
    if index == 0 {
        "the target body".to_owned()
    } else {
        format!("tool body {index}")
    }
}

/// The process-wide B-rep kernel used to tessellate solid tools.
fn kernel() -> &'static amg_brep::OcctKernel {
    static KERNEL: OnceLock<amg_brep::OcctKernel> = OnceLock::new();
    KERNEL.get_or_init(|| amg_brep::OcctKernel::new(amg_brep::KernelConfig::default()))
}

/// The tessellation of a solid tool body as an indexed mesh (BREP-08 weld: watertight for a
/// closed solid).
fn tessellate_solid(
    ctx: &RecomputeCtx<'_>,
    shape: &amg_doc::stub::Shape,
) -> Result<IndexedMesh, FeatureError> {
    use amg_brep::BrepKernel;
    let Some(brep) = shape.brep() else {
        return Err(MeshFeatureError::Brep(
            "the solid tool has no B-rep geometry to tessellate".to_owned(),
        )
        .into());
    };
    let tess = with_job(ctx, |job| {
        kernel().tessellate(brep, &amg_brep::TessQuality::export(), job)
    })
    .map_err(|e| match e {
        amg_brep::KernelError::Cancelled => FeatureError::Cancelled,
        other => FeatureError::from(MeshFeatureError::Brep(other.to_string())),
    })?;
    let (positions, tris, _faces) = tess.to_indexed_parts();
    Ok(IndexedMesh::new(positions, tris).map_err(MeshFeatureError::Mesh)?)
}

impl Feature for MeshCombineFeature {
    fn inputs(&self) -> Vec<Dependency> {
        std::iter::once(self.target)
            .chain(self.tools.iter().copied())
            .map(Dependency::Body)
            .collect()
    }

    fn validate(&self) -> Result<(), FeatureError> {
        if self.tools.is_empty() {
            return Err(FeatureError::InvalidValue {
                field: "tools",
                msg: "select at least one tool body".to_owned(),
            });
        }
        if self.tools.len() > MAX_COMBINE_TOOLS {
            return Err(FeatureError::InvalidValue {
                field: "tools",
                msg: format!("at most {MAX_COMBINE_TOOLS} tool bodies"),
            });
        }
        if self.tools.contains(&self.target) {
            return Err(FeatureError::InvalidValue {
                field: "tools",
                msg: "the target body cannot also be a tool".to_owned(),
            });
        }
        let mut seen = std::collections::BTreeSet::new();
        if !self.tools.iter().all(|t| seen.insert(*t)) {
            return Err(FeatureError::InvalidValue {
                field: "tools",
                msg: "a tool body is listed twice".to_owned(),
            });
        }
        Ok(())
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        self.validate()?;
        let mut out = FeatureOutput::default();
        // Implicit operands first: nothing is meshed if any operand is implicit.
        for (i, id) in std::iter::once(&self.target).chain(&self.tools).enumerate() {
            if matches!(ctx.body(*id)?, BodyGeom::Implicit(_)) {
                return Err(MeshFeatureError::ImplicitOperand(role(i)).into());
            }
        }
        let target = input_mesh(ctx, self.target)?;
        let mut tools: Vec<Arc<IndexedMesh>> = Vec::with_capacity(self.tools.len());
        let mut converted = Vec::new();
        for (k, id) in self.tools.iter().enumerate() {
            ctx.check_cancelled()?;
            match ctx.body(*id)? {
                BodyGeom::Mesh(_) => tools.push(input_mesh(ctx, *id)?),
                BodyGeom::Solid(shape) => {
                    ctx.progress(0.0, "tessellate solid tool");
                    tools.push(Arc::new(tessellate_solid(ctx, shape)?));
                    converted.push(k + 1);
                }
                _ => {
                    return Err(FeatureError::InvalidValue {
                        field: "tools",
                        msg: format!("{} is neither a mesh nor a solid", role(k + 1)),
                    });
                }
            }
        }
        if !converted.is_empty() {
            let which: Vec<String> = converted.iter().map(|&i| role(i)).collect();
            out.warnings.push(warning(format!(
                "{} converted to mesh for the combine (the target is a mesh body)",
                which.join(", ")
            )));
        }
        ctx.progress(0.1, "combine");
        let operands: Vec<&IndexedMesh> = std::iter::once(&*target)
            .chain(tools.iter().map(|t| &**t))
            .collect();
        let (mesh, report) = with_job(ctx, |job| {
            boolean_n(
                self.op,
                &operands,
                &BoolOpts {
                    repair_inputs: true,
                },
                job,
            )
        })
        .map_err(MeshFeatureError::Mesh)?;
        for &i in &report.repaired_inputs {
            out.warnings.push(warning(format!(
                "{} was not a closed manifold mesh and was repaired before the combine",
                role(i)
            )));
        }
        if mesh.face_count() == 0 {
            out.warnings.push(warning(
                "the combine result is empty (the bodies do not overlap)".to_owned(),
            ));
        }
        emit(ctx, self.target, self.output, Arc::new(mesh), &mut out);
        if !self.keep_tools {
            out.bodies.extend(self.tools.iter().map(|&id| BodyChange {
                id,
                change: BodyChangeKind::Deleted,
            }));
        }
        Ok(out)
    }
}

amg_doc::register_feature!(MeshCombineFeature, "mesh.combine", version = 1);
