//! `solid.split_body`: a body split by a plane, a face or another body (`BrepKernel::split`).
//!
//! Naming (note §5.2 "Split body"): the whole split result is named once with
//! `SplitOrder::PieceSlot`, so a face cut in two is `name + Split { self, slot, n }` in piece
//! order; the faces a plane or face tool leaves are `Gen { self, SplitFace#0, Sel(key) |
//! Construction(id) }`; a body tool's faces keep the tool body's names. Pieces are ordered by
//! the signed centroid distance along a planar tool's normal, else by volume (descending)
//! then centroid (x, y, z). Piece 0 keeps the `BodyId`, piece `k ≥ 1` is `BodyId { self, k }`
//! with `CreatedFrom { derived_from }` (TNP-21).

use amg_brep::{BoolOpts, InputSub, Relation, SubKind};
use amg_doc::naming::{OperandNaming, Role, SplitOrder, TopoKind};
use amg_doc::{
    BodyChange, BodyChangeKind, BodyId, Dependency, EntityRef, Feature, FeatureError,
    FeatureOutput, RecomputeCtx, register_feature,
};
use serde::{Deserialize, Serialize};

use super::{invalid, no_split, split_tool, tool_dependency};
use crate::construct::{OriginEntity, RefInput};
use crate::solid::common::{
    finish_naming, identity_result, live_body, op_kernel, solid_geom, with_job,
};
use crate::solid::error::{kernel_failure, kerr};

/// `solid.split_body`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SplitBody {
    /// The body to split (`None` until picked; the feature then fails to compute).
    #[serde(default)]
    pub body: Option<BodyId>,
    /// The splitting tool: an origin or construction plane, a face or a body.
    pub tool: RefInput,
    /// Extend a face tool to its untrimmed surface.
    #[serde(default)]
    pub extend_tool: bool,
}

impl SplitBody {
    /// A split of `body` by `tool`, not extended.
    pub fn new(body: BodyId, tool: RefInput) -> SplitBody {
        SplitBody {
            body: Some(body),
            tool,
            extend_tool: false,
        }
    }
}

impl Default for SplitBody {
    fn default() -> SplitBody {
        SplitBody {
            body: None,
            tool: RefInput::Origin(OriginEntity::YZPlane),
            extend_tool: false,
        }
    }
}

/// Piece order key: signed distance along the tool normal, or (−volume, centroid), each
/// quantised to 1e-6 (mm, mm³) so round-off never swaps equal pieces.
fn order_key(plane: Option<&amg_core::Plane>, m: &amg_brep::MassProps) -> [i64; 4] {
    #[allow(
        clippy::cast_possible_truncation,
        reason = "model-sized values, far below 2^63 µm"
    )]
    fn q(x: f64) -> i64 {
        (x * 1e6).round() as i64
    }
    let c = m.centroid;
    match plane {
        Some(p) => [q((c - p.origin).dot(p.normal.get())), 0, 0, 0],
        None => [q(-m.volume), q(c.x), q(c.y), q(c.z)],
    }
}

impl Feature for SplitBody {
    fn inputs(&self) -> Vec<Dependency> {
        self.body
            .map(Dependency::Body)
            .into_iter()
            .chain(tool_dependency(&self.tool))
            .collect()
    }

    fn input_is_optional(&self, dep: &Dependency) -> bool {
        crate::solid::fillet::lineage_optional(dep)
    }

    fn validate(&self) -> Result<(), FeatureError> {
        match (&self.tool, self.body) {
            (RefInput::Ref(EntityRef::Body(t)), Some(b)) if *t == b => {
                Err(invalid("tool", "a body can't split itself"))
            }
            _ => Ok(()),
        }
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let mut out = FeatureOutput::default();
        let Some(body_ref) = self.body else {
            return Err(invalid("body", "select the body to split"));
        };
        let Some(p) = live_body(ctx, body_ref, &mut out)? else {
            ctx.highlight([EntityRef::Body(body_ref)]);
            return Err(invalid("body", "the body no longer exists"));
        };
        if let RefInput::Ref(EntityRef::Body(t)) = &self.tool
            && crate::solid::common::follow(ctx, *t) == Some(p.id)
        {
            return Err(invalid("tool", "a body can't split itself"));
        }
        let k = op_kernel(ctx);
        let w = p.shape.bbox().expanded(1.0);
        let tool = split_tool(ctx, k, &self.tool, self.extend_tool, &w)?;
        ctx.check_cancelled()?;
        let split = with_job(ctx, |job| {
            k.split(
                std::slice::from_ref(p.shape),
                std::slice::from_ref(&tool.shape),
                &BoolOpts::default(),
                job,
            )
        })
        .map_err(|e| kernel_failure(ctx, Some(p.id), e, &mut out))?;
        // Per-solid restrictions of an identity history: compound index → piece index.
        let solids = identity_result(&split.shape).per_solid().map_err(kerr)?;
        if solids.len() < 2 {
            return Err(no_split(ctx, &self.tool));
        }
        let mut keyed = Vec::with_capacity(solids.len());
        for (i, s) in solids.iter().enumerate() {
            let m = k.mass_props(&s.shape).map_err(kerr)?;
            keyed.push((order_key(tool.plane.as_ref(), &m), i));
        }
        keyed.sort_unstable();
        let mut slot_of = vec![0u32; solids.len()];
        for (slot, (_, i)) in (0u32..).zip(&keyed) {
            slot_of[*i] = slot;
        }
        let nf = split.shape.count(SubKind::Face) as usize;
        let mut piece_of_face = vec![u32::MAX; nf];
        for (i, s) in solids.iter().enumerate() {
            for (InputSub { operand, sub }, rel) in &s.history.entries {
                if *operand == 0
                    && sub.kind == SubKind::Face
                    && matches!(rel, Relation::Kept(_))
                    && let Some(x) = piece_of_face.get_mut(sub.index as usize)
                {
                    *x = (*x).min(slot_of[i]);
                }
            }
        }

        // Names of the whole result, then of each piece through its restriction.
        let mut b = ctx.naming_builder();
        let tool_operand = match tool.naming {
            Some(n) => OperandNaming::Named(n),
            None => OperandNaming::Anonymous,
        };
        b.from_history(
            &[(0, OperandNaming::Named(p.naming)), (1, tool_operand)],
            &split.history,
        );
        b.split_order(SplitOrder::PieceSlot, Some(piece_of_face));
        if tool.naming.is_none() {
            for face in 0..split.shape.count(SubKind::Face) {
                let o = amg_brep::SubShape {
                    kind: SubKind::Face,
                    index: face,
                };
                let from_tool = split.history.sources_of(o).iter().any(|(i, rel)| {
                    i.operand == 1
                        && i.sub.kind == SubKind::Face
                        && matches!(rel, Relation::Modified(_) | Relation::Kept(_))
                });
                if from_tool {
                    b.generated(
                        TopoKind::Face,
                        face,
                        Role::new("SplitFace", 0),
                        tool.source.clone(),
                    );
                }
            }
        }
        let whole = finish_naming(b, k, &split.shape, &mut out)?;
        let mut ordered: Vec<(u32, amg_brep::OpResult)> = solids
            .into_iter()
            .enumerate()
            .map(|(i, s)| (slot_of[i], s))
            .collect();
        ordered.sort_by_key(|(slot, _)| *slot);
        for (slot, piece) in ordered {
            let mut nb = ctx.naming_builder();
            nb.from_history(&[(0, OperandNaming::Named(&whole))], &piece.history);
            let naming = finish_naming(nb, k, &piece.shape, &mut out)?;
            let change = if slot == 0 {
                BodyChangeKind::Modified {
                    geom: solid_geom(piece.shape),
                    naming: Some(naming),
                }
            } else {
                BodyChangeKind::CreatedFrom {
                    geom: solid_geom(piece.shape),
                    naming: Some(naming),
                    derived_from: p.id,
                }
            };
            out.bodies.push(BodyChange {
                id: if slot == 0 {
                    p.id
                } else {
                    ctx.new_body_id(slot)
                },
                change,
            });
        }
        Ok(out)
    }
}

register_feature!(SplitBody, "solid.split_body", version = 1);

#[cfg(test)]
mod tests {
    use super::*;
    use amg_doc::FeatureId;

    fn body(origin: u64) -> BodyId {
        BodyId {
            origin: FeatureId(origin),
            slot: 0,
        }
    }

    #[test]
    fn inputs_list_the_body_and_a_body_tool_as_bodies() {
        let s = SplitBody::new(body(1), RefInput::Ref(EntityRef::Body(body(2))));
        assert_eq!(
            s.inputs(),
            vec![Dependency::Body(body(1)), Dependency::Body(body(2))]
        );
        let s = SplitBody::new(body(1), RefInput::Origin(OriginEntity::XYPlane));
        assert_eq!(s.inputs(), vec![Dependency::Body(body(1))]);
    }

    #[test]
    fn a_body_cannot_split_itself() {
        let s = SplitBody::new(body(1), RefInput::Ref(EntityRef::Body(body(1))));
        assert!(s.validate().is_err());
        assert!(SplitBody::default().validate().is_ok());
    }

    #[test]
    fn json_without_extend_defaults_to_false() {
        let s: SplitBody =
            serde_json::from_str(r#"{"body":{"origin":1,"slot":0},"tool":{"Origin":"YZPlane"}}"#)
                .unwrap();
        assert_eq!(
            s,
            SplitBody::new(body(1), RefInput::Origin(OriginEntity::YZPlane))
        );
    }
}
