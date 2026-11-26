//! `solid.combine` (S04 §3, SOL-07): a target body joined with, cut by or intersected with
//! tool bodies, in one N-ary boolean simplified like every body operation (naming note §5.1).
//!
//! - **Join** keeps the whole result in the target body, also when a tool doesn't touch it (a
//!   body of several lumps, as Fusion does; TNP-23 expects the tool's edges in the target).
//! - **Cut / intersect** follow SOL-01's piece rule: piece 0 keeps the target's id, further
//!   pieces are `BodyId { self, k }` (`k` from 1) with `CreatedFrom { derived_from: target }`.
//!   A cut that touches nothing fails [`SolidError::NothingToCut`]; an empty common part
//!   [`SolidError::NothingToIntersect`].
//! - Tools are `Consumed { into: target }` unless `keep_tools` (naming note §6.1).
//!
//! Combine has no names of its own: target and tool faces keep theirs through the history
//! (naming note §5.2, "Combine" row).

mod dialog;

use amg_brep::{BoolOp, BoolOpts, BrepKernel, InputSub, OpResult, Relation, ShapeHistory, SubKind};
use amg_doc::naming::OperandNaming;
use amg_doc::{
    BodyChange, BodyChangeKind, BodyId, Dependency, Diagnostic, EntityRef, Feature, FeatureError,
    FeatureOutput, RecomputeCtx, register_feature,
};
use serde::{Deserialize, Serialize};

use super::common::{
    BodyOp, Participant, boxes_meet, contributes, finish_naming, live_body, op_kernel, pieces,
    simplify, solid_geom, touches, with_job,
};
use super::error::{SolidError, kernel_failure, kerr};

/// `solid.combine`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Combine {
    /// The body that is changed (`None` until picked: the dialog builds without one and the
    /// feature then fails to compute).
    #[serde(default)]
    pub target: Option<BodyId>,
    /// The tool bodies, in order.
    #[serde(default)]
    pub tools: Vec<BodyId>,
    /// `Join`, `Cut` or `Intersect`.
    pub op: BodyOp,
    /// Keep the tool bodies instead of consuming them into the target.
    #[serde(default)]
    pub keep_tools: bool,
}

impl Combine {
    /// A combine of `tools` into `target` that consumes the tools.
    pub fn new(target: BodyId, tools: Vec<BodyId>, op: BodyOp) -> Combine {
        Combine {
            target: Some(target),
            tools,
            op,
            keep_tools: false,
        }
    }
}

fn invalid(field: &'static str, msg: impl Into<String>) -> FeatureError {
    SolidError::Invalid {
        field,
        msg: msg.into(),
    }
    .into()
}

fn bool_op(op: BodyOp) -> Result<BoolOp, FeatureError> {
    match op {
        BodyOp::Join => Ok(BoolOp::Fuse),
        BodyOp::Cut => Ok(BoolOp::Cut),
        BodyOp::Intersect => Ok(BoolOp::Common),
        BodyOp::NewBody => Err(invalid("op", "combine joins, cuts or intersects")),
    }
}

/// Whether the tools left the target (operand 0) alone: every face of it `Kept` and nothing
/// from any tool in the result.
fn untouched(h: &ShapeHistory, tools: usize) -> bool {
    let faces_kept = h
        .entries
        .iter()
        .filter(|(InputSub { operand, sub }, _)| *operand == 0 && sub.kind == SubKind::Face)
        .all(|(_, r)| matches!(r, Relation::Kept(_)));
    faces_kept && (1..=tools).all(|i| !contributes(h, u16::try_from(i).unwrap_or(u16::MAX)))
}

impl Combine {
    /// The live tools: successors followed, gone ones dropped (warning), duplicates and the
    /// target itself refused or removed.
    fn live_tools<'a>(
        &self,
        ctx: &RecomputeCtx<'a>,
        target: BodyId,
        out: &mut FeatureOutput,
    ) -> Result<Vec<Participant<'a>>, FeatureError> {
        let mut v: Vec<Participant<'a>> = Vec::new();
        for &id in &self.tools {
            let Some(p) = live_body(ctx, id, out)? else {
                continue;
            };
            if p.id == target {
                ctx.highlight([EntityRef::Body(id)]);
                return Err(invalid("tools", "the target body can't be its own tool"));
            }
            if !v.iter().any(|q| q.id == p.id) {
                v.push(p);
            }
        }
        if v.is_empty() {
            return Err(invalid("tools", "no tool body is left to combine"));
        }
        Ok(v)
    }
}

impl Feature for Combine {
    fn inputs(&self) -> Vec<Dependency> {
        self.target
            .iter()
            .chain(&self.tools)
            .map(|b| Dependency::Body(*b))
            .collect()
    }

    fn input_is_optional(&self, dep: &Dependency) -> bool {
        crate::solid::fillet::lineage_optional(dep)
    }

    fn validate(&self) -> Result<(), FeatureError> {
        // An empty pick builds and validates (the dialog's `min: 1` keeps OK off) and fails in
        // `compute`, as for fillet edges.
        bool_op(self.op).map(|_| ())
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let mut out = FeatureOutput::default();
        let bop = bool_op(self.op)?;
        let Some(target_ref) = self.target else {
            return Err(invalid("target", "select the target body"));
        };
        if self.tools.is_empty() {
            return Err(invalid("tools", "select at least one tool body"));
        }
        let Some(target) = live_body(ctx, target_ref, &mut out)? else {
            ctx.highlight([EntityRef::Body(target_ref)]);
            return Err(invalid("target", "the target body no longer exists"));
        };
        let tools = self.live_tools(ctx, target.id, &mut out)?;
        let k = op_kernel(ctx);
        let tol = ctx.tolerance().linear;
        if self.op == BodyOp::Cut && !tools.iter().any(|t| boxes_meet(target.shape, t.shape, tol)) {
            return Err(SolidError::NothingToCut.into());
        }
        ctx.check_cancelled()?;
        let shapes: Vec<amg_brep::Shape> = tools.iter().map(|t| t.shape.clone()).collect();
        let r = with_job(ctx, |job| {
            k.boolean(
                bop,
                std::slice::from_ref(target.shape),
                &shapes,
                &BoolOpts::default(),
                job,
            )
        })
        .and_then(|r| simplify(k, r))
        .map_err(|e| kernel_failure(ctx, Some(target.id), e, &mut out))?;
        let mut operands: Vec<(u16, OperandNaming<'_>)> =
            vec![(0, OperandNaming::Named(target.naming))];
        for (i, t) in (1u16..).zip(&tools) {
            operands.push((i, OperandNaming::Named(t.naming)));
        }
        match self.op {
            BodyOp::Join => {
                for t in &tools {
                    if !touches(k, target.shape, t.shape, tol).map_err(kerr)? {
                        out.warnings.push(Diagnostic {
                            message: format!(
                                "Tool body {:?} doesn't touch the target; it is a separate lump of the combined body.",
                                t.id
                            ),
                            cause: None,
                            entities: vec![EntityRef::Body(t.id)],
                        });
                    }
                }
                let mut b = ctx.naming_builder();
                b.from_history(&operands, &r.history);
                let naming = finish_naming(b, k, &r.shape, &mut out)?;
                out.bodies.push(BodyChange {
                    id: target.id,
                    change: BodyChangeKind::Modified {
                        geom: solid_geom(r.shape),
                        naming: Some(naming),
                    },
                });
            }
            BodyOp::Cut | BodyOp::Intersect => {
                if self.op == BodyOp::Cut && untouched(&r.history, tools.len()) {
                    return Err(SolidError::NothingToCut.into());
                }
                emit_pieces(ctx, k, target.id, &operands, r, self.op, &mut out)?;
            }
            BodyOp::NewBody => unreachable!("refused by bool_op"),
        }
        if !self.keep_tools {
            for t in &tools {
                out.bodies.push(BodyChange {
                    id: t.id,
                    change: BodyChangeKind::Consumed { into: target.id },
                });
            }
        }
        Ok(out)
    }
}

/// The pieces of a cut or intersect: piece 0 keeps the target's id, the others are new
/// bodies split off from it.
fn emit_pieces(
    ctx: &RecomputeCtx<'_>,
    k: &dyn BrepKernel,
    target: BodyId,
    operands: &[(u16, OperandNaming<'_>)],
    r: OpResult,
    op: BodyOp,
    out: &mut FeatureOutput,
) -> Result<(), FeatureError> {
    let rs = pieces(r).map_err(kerr)?;
    if rs.is_empty() {
        ctx.highlight([EntityRef::Body(target)]);
        return Err(if op == BodyOp::Intersect {
            SolidError::NothingToIntersect.into()
        } else {
            invalid("tools", "the tools remove the whole target body")
        });
    }
    for (slot, piece) in (0u32..).zip(rs) {
        let mut b = ctx.naming_builder();
        b.from_history(operands, &piece.history);
        let naming = finish_naming(b, k, &piece.shape, out)?;
        let change = if slot == 0 {
            BodyChange {
                id: target,
                change: BodyChangeKind::Modified {
                    geom: solid_geom(piece.shape),
                    naming: Some(naming),
                },
            }
        } else {
            BodyChange {
                id: ctx.new_body_id(slot),
                change: BodyChangeKind::CreatedFrom {
                    geom: solid_geom(piece.shape),
                    naming: Some(naming),
                    derived_from: target,
                },
            }
        };
        out.bodies.push(change);
    }
    Ok(())
}

register_feature!(Combine, "solid.combine", version = 1);

#[cfg(test)]
mod tests {
    use super::*;
    use amg_brep::SubShape;
    use amg_doc::FeatureId;

    fn body(origin: u64) -> BodyId {
        BodyId {
            origin: FeatureId(origin),
            slot: 0,
        }
    }

    #[test]
    fn inputs_list_target_and_every_tool() {
        let c = Combine::new(body(1), vec![body(2), body(3)], BodyOp::Cut);
        assert_eq!(
            c.inputs(),
            vec![
                Dependency::Body(body(1)),
                Dependency::Body(body(2)),
                Dependency::Body(body(3))
            ]
        );
    }

    #[test]
    fn new_body_is_not_a_combine_op() {
        let c = Combine::new(body(1), vec![body(2)], BodyOp::NewBody);
        assert!(c.validate().is_err());
        assert!(
            Combine::new(body(1), vec![body(2)], BodyOp::Join)
                .validate()
                .is_ok()
        );
    }

    #[test]
    fn untouched_needs_every_tool_silent() {
        let f0 = SubShape {
            kind: SubKind::Face,
            index: 0,
        };
        let mut h = ShapeHistory {
            entries: vec![(
                InputSub {
                    operand: 0,
                    sub: f0,
                },
                Relation::Kept(f0),
            )],
            ..ShapeHistory::default()
        };
        assert!(untouched(&h, 2));
        h.entries.push((
            InputSub {
                operand: 2,
                sub: f0,
            },
            Relation::Kept(f0),
        ));
        assert!(!untouched(&h, 2));
    }

    #[test]
    fn json_without_keep_tools_defaults_to_false() {
        let c: Combine = serde_json::from_str(
            r#"{"target":{"origin":1,"slot":0},"tools":[{"origin":2,"slot":0}],"op":"Join"}"#,
        )
        .unwrap();
        assert_eq!(c, Combine::new(body(1), vec![body(2)], BodyOp::Join));
    }
}
