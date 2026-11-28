//! `solid.mirror` (S04 §5, SOL-08): bodies (new body or join) and features, reflected about a
//! plane through [`BrepKernel::mirrored`] (R-S04-6). A reflection can't be a located copy
//! (`Shape::transformed` rejects it with `NonRigid`), so the copy is new geometry named
//! through its 1:1 `Modified` history, then `Inst { self, 1 }` on every face (naming note §4.6;
//! a mirrored feature follows the feature-pattern rules with `k = 1`).

use amg_brep::{BrepKernel, OpResult, Shape};
use amg_core::Plane;
use amg_doc::naming::OperandNaming;
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, BodyNaming, ContextBodies, Dependency, EntityRef,
    Feature, FeatureError, FeatureId, FeatureOutput, OccurrenceId, RecomputeCtx, register_feature,
};
use serde::{Deserialize, Serialize};

use super::{
    PatternObjects, common_target, instances_compound, invalid, not_yet, ref_dependency,
    require_objects, seed_reads, seed_tools,
};
use crate::construct::RefInput;
use crate::construct::resolve::resolve_plane_input;
use crate::solid::common::{
    BodyOp, OpTarget, ToolBody, apply_body_op, finish_naming, input_solid, op_kernel,
};
use crate::solid::error::kernel_failure;

/// `solid.mirror`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Mirror {
    /// What is mirrored (bodies or features; components are SOL-24).
    pub objects: PatternObjects,
    /// The mirror plane: an origin or construction plane, or a planar face.
    pub plane: RefInput,
    /// Body mirrors: `NewBody` (a separate mirrored copy) or `Join` (fused with its source).
    #[serde(default)]
    pub op: BodyOp,
    /// Feature mirrors: the body operation of the mirrored tools (`None`: the seeds' own).
    #[serde(default)]
    pub target: Option<OpTarget>,
}

/// The mirror instance number (naming note §5.2).
const MIRROR_INSTANCE: u32 = 1;

/// `shape` reflected about `plane`, named from `naming` through the 1:1 history plus
/// `Inst { self, 1 }`.
fn mirrored_copy(
    ctx: &RecomputeCtx<'_>,
    k: &dyn BrepKernel,
    shape: &Shape,
    naming: &BodyNaming,
    plane: &Plane,
    body: Option<BodyId>,
    out: &mut FeatureOutput,
) -> Result<(Shape, BodyNaming), FeatureError> {
    let r: OpResult = k
        .mirrored(shape, plane)
        .map_err(|e| kernel_failure(ctx, body, e, out))?;
    let mut b = ctx.naming_builder();
    b.from_history(&[(0, OperandNaming::Named(naming))], &r.history);
    let mut scratch = FeatureOutput::default();
    let n = finish_naming(b, k, &r.shape, &mut scratch)?;
    Ok((
        r.shape,
        n.instanced_names(ctx.feature_id(), MIRROR_INSTANCE),
    ))
}

impl Mirror {
    fn bodies(
        &self,
        ctx: &RecomputeCtx<'_>,
        plane: &Plane,
        bodies: &[BodyId],
        out: &mut FeatureOutput,
    ) -> Result<(), FeatureError> {
        let k = op_kernel(ctx);
        let mut parts = Vec::with_capacity(bodies.len());
        for &b in bodies {
            ctx.check_cancelled()?;
            let (shape, naming) = input_solid(ctx, b)?;
            parts.push(mirrored_copy(ctx, k, shape, naming, plane, Some(b), out)?);
        }
        match self.op {
            BodyOp::NewBody => {
                for (slot, (s, n)) in (0u32..).zip(parts) {
                    out.bodies.push(BodyChange {
                        id: ctx.new_body_id(slot),
                        change: BodyChangeKind::Created {
                            geom: BodyGeom::Solid(amg_doc::stub::Shape::from_brep(s)),
                            naming: Some(n),
                        },
                    });
                }
                Ok(())
            }
            BodyOp::Join => {
                let (shape, naming) = instances_compound(ctx, k, parts)?;
                apply_body_op(
                    ctx,
                    ToolBody {
                        shape,
                        naming,
                        target: OpTarget {
                            op: BodyOp::Join,
                            participants: bodies.to_vec(),
                        },
                    },
                    out,
                )
            }
            BodyOp::Cut | BodyOp::Intersect => Err(invalid(
                "op",
                "a body mirror makes a new body or joins its source",
            )),
        }
    }

    fn features(
        &self,
        ctx: &RecomputeCtx<'_>,
        plane: &Plane,
        seeds: &[FeatureId],
        out: &mut FeatureOutput,
    ) -> Result<(), FeatureError> {
        let tools = seed_tools(ctx, seeds)?;
        let target = common_target(self.target.as_ref(), &tools)?;
        let k = op_kernel(ctx);
        let mut parts = Vec::with_capacity(tools.len());
        for t in &tools {
            ctx.check_cancelled()?;
            parts.push(mirrored_copy(
                ctx, k, &t.shape, &t.naming, plane, None, out,
            )?);
        }
        let (shape, naming) = instances_compound(ctx, k, parts)?;
        apply_body_op(
            ctx,
            ToolBody {
                shape,
                naming,
                target,
            },
            out,
        )
    }
}

impl Feature for Mirror {
    fn inputs(&self) -> Vec<Dependency> {
        let mut d = Vec::new();
        match &self.objects {
            PatternObjects::Bodies(b) => d.extend(b.iter().map(|b| Dependency::Body(*b))),
            PatternObjects::Features(f) => d.extend(f.iter().map(|f| Dependency::Feature(*f))),
            PatternObjects::Faces(t) => {
                d.extend(
                    t.iter()
                        .cloned()
                        .map(|t| Dependency::Entity(EntityRef::Topo(t))),
                );
            }
            PatternObjects::Occurrences(o) => {
                d.extend(o.iter().map(|o: &OccurrenceId| Dependency::Occurrence(*o)));
            }
        }
        d.extend(ref_dependency(&self.plane));
        if let Some(t) = &self.target {
            d.extend(t.inputs());
        }
        d
    }

    fn context_reads(&self) -> Option<ContextBodies> {
        seed_reads(&self.objects)
    }

    fn validate(&self) -> Result<(), FeatureError> {
        // Empty selections build (the dialog's `min: 1` keeps OK disabled) and fail in
        // `compute`, as for fillet edges.
        match &self.objects {
            PatternObjects::Bodies(_) if !matches!(self.op, BodyOp::NewBody | BodyOp::Join) => Err(
                invalid("op", "a body mirror makes a new body or joins its source"),
            ),
            PatternObjects::Bodies(_) if self.target.is_some() => Err(invalid(
                "target",
                "a body mirror has no separate body operation; use `op`",
            )),
            _ => Ok(()),
        }
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let mut out = FeatureOutput::default();
        require_objects(&self.objects)?;
        let plane = resolve_plane_input(ctx, &self.plane)?;
        match &self.objects {
            PatternObjects::Bodies(b) => self.bodies(ctx, &plane, b, &mut out)?,
            PatternObjects::Features(f) => self.features(ctx, &plane, f, &mut out)?,
            PatternObjects::Faces(_) => return Err(not_yet("face mirror")),
            PatternObjects::Occurrences(_) => return Err(not_yet("component mirror")),
        }
        Ok(out)
    }
}

register_feature!(Mirror, "solid.mirror", version = 1);
