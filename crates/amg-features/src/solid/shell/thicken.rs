//! `solid.thicken` (S04 SOL-06, BREP-07): faces (of sheet or solid bodies) swept along their
//! normals into solids, then applied through the body operation ([`OpTarget`]).
//!
//! - The faces of one body are thickened together: the whole sheet when every face of a sheet
//!   body is picked, a single face as itself, else a compound of the picked faces.
//! - `ThinSide::Side1` grows along the face normals, `Side2` against them, `Center` half each
//!   way (the faces are first offset back by half the thickness).
//! - Naming (naming note §5.2 "Thicken / offset"): the far faces `Offset#0` from their face,
//!   the side walls `ThickenSide#0` from their boundary edge (both `Topo*`); the faces themselves
//!   keep their names in the new solid.

use amg_brep::{BrepKernel, KernelError, OffsetJoin, OpRole, Shape, SubKind, SubShape};
use amg_doc::naming::{OperandNaming, Role, TopoKind};
use amg_doc::{
    BodyId, BodyNaming, Dependency, Dim, EntityRef, Expr, Feature, FeatureError, FeatureOutput,
    RecomputeCtx, SetPolicy, TopoName, TopoRef, register_feature,
};
use serde::{Deserialize, Serialize};

use super::{inline, invalid, positive};
use crate::register_tool;
use crate::solid::common::{
    OpTarget, SolidTool, ToolBody, apply_body_op, finish_naming, input_solid, op_kernel,
    resolve_subs, with_job,
};
use crate::solid::error::{kernel_failure, kerr};
use crate::solid::extent::ThinSide;
use crate::solid::extrude::compound_of;

/// `solid.thicken`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Thicken {
    /// Faces to thicken (set-valued; may span bodies).
    pub faces: Vec<TopoRef>,
    /// Thickness (length).
    pub thickness: Expr,
    /// Along the normals (`Side1`), against them (`Side2`) or half each way (`Center`).
    #[serde(default)]
    pub side: ThinSide,
    /// The body operation of the thickened solids.
    #[serde(default)]
    pub target: OpTarget,
}

impl Thicken {
    /// A new body from `faces` thickened by `thickness` (such as `"1 mm"`) along their normals.
    pub fn new_body(faces: Vec<TopoRef>, thickness: &str) -> Thicken {
        Thicken {
            faces,
            thickness: inline(thickness),
            side: ThinSide::Side1,
            target: OpTarget::default(),
        }
    }
}

/// Index maps between an operand built from faces of a body and the body.
struct SubMap {
    /// Body index of each operand face, edge, vertex.
    to_body: [Vec<u32>; 3],
}

impl SubMap {
    fn body_sub(&self, s: SubShape) -> Option<SubShape> {
        let i = match s.kind {
            SubKind::Face => 0,
            SubKind::Edge => 1,
            SubKind::Vertex => 2,
        };
        let index = *self.to_body[i].get(usize::try_from(s.index).ok()?)?;
        Some(SubShape {
            kind: s.kind,
            index,
        })
    }
}

/// Maps every sub-shape of `op` (built from `faces` of `body`, sharing their TShapes) to the
/// body's index space.
fn sub_map(body: &Shape, op: &Shape, faces: &[u32]) -> Result<SubMap, FeatureError> {
    let mut cand: [Vec<u32>; 3] = [faces.to_vec(), Vec::new(), Vec::new()];
    for &f in faces {
        for &e in body.face_edges(amg_brep::FaceId(f)) {
            if !cand[1].contains(&e.0) {
                cand[1].push(e.0);
            }
            for v in body.edge_vertices(e) {
                if !cand[2].contains(&v.0) {
                    cand[2].push(v.0);
                }
            }
        }
    }
    let kinds = [SubKind::Face, SubKind::Edge, SubKind::Vertex];
    let mut to_body: [Vec<u32>; 3] = [Vec::new(), Vec::new(), Vec::new()];
    for (i, kind) in kinds.into_iter().enumerate() {
        let body_subs: Vec<(u32, Shape)> = cand[i]
            .iter()
            .map(|&index| body.sub_shape(SubShape { kind, index }).map(|s| (index, s)))
            .collect::<Result<_, _>>()
            .map_err(kerr)?;
        for index in 0..op.count(kind) {
            let s = op.sub_shape(SubShape { kind, index }).map_err(kerr)?;
            let Some((b, _)) = body_subs.iter().find(|(_, x)| x.is_same(&s)) else {
                return Err(FeatureError::Kernel(format!(
                    "thicken: {kind:?} {index} of the faces is not on the body"
                )));
            };
            to_body[i].push(*b);
        }
    }
    Ok(SubMap { to_body })
}

/// The names of `op`'s sub-shapes, from the body's.
fn sub_naming(naming: &BodyNaming, map: &SubMap) -> Result<BodyNaming, FeatureError> {
    let kinds = [TopoKind::Face, TopoKind::Edge, TopoKind::Vertex];
    let mut lists: [Vec<TopoName>; 3] = [Vec::new(), Vec::new(), Vec::new()];
    for (i, kind) in kinds.into_iter().enumerate() {
        for &b in &map.to_body[i] {
            let n = naming.name_of(kind, b).ok_or_else(|| {
                FeatureError::Kernel(format!("thicken: {kind:?} {b} has no name"))
            })?;
            lists[i].push(n.clone());
        }
    }
    let [f, e, v] = lists;
    Ok(BodyNaming::new(f, e, v))
}

/// `e` with its offending sub-shapes moved from the operand to the body's index space.
fn to_body_space(e: KernelError, map: &SubMap) -> KernelError {
    let m = |subs: Vec<SubShape>| subs.into_iter().filter_map(|s| map.body_sub(s)).collect();
    match e {
        KernelError::LocalOpFailed {
            op,
            message,
            offending,
            partial,
        } => KernelError::LocalOpFailed {
            op,
            message,
            offending: m(offending),
            partial,
        },
        KernelError::OpFailed {
            op,
            message,
            offending,
        } => KernelError::OpFailed {
            op,
            message,
            offending: m(offending),
        },
        KernelError::InvalidInput {
            what,
            reason,
            offending,
        } => KernelError::InvalidInput {
            what,
            reason,
            offending: m(offending),
        },
        other => other,
    }
}

impl Thicken {
    /// One body's faces thickened and named.
    fn thicken_body(
        &self,
        ctx: &RecomputeCtx<'_>,
        k: &dyn BrepKernel,
        body: BodyId,
        faces: &[u32],
        t: f64,
        out: &mut FeatureOutput,
    ) -> Result<(Shape, BodyNaming), FeatureError> {
        let (shape, naming) = input_solid(ctx, body)?;
        let nf = shape.count(SubKind::Face);
        let sheet = shape.solids().is_empty();
        let op = if sheet && u32::try_from(faces.len()).ok() == Some(nf) {
            shape.clone()
        } else if let [f] = faces {
            shape
                .sub_shape(SubShape {
                    kind: SubKind::Face,
                    index: *f,
                })
                .map_err(kerr)?
        } else {
            let parts = faces
                .iter()
                .map(|&index| {
                    shape.sub_shape(SubShape {
                        kind: SubKind::Face,
                        index,
                    })
                })
                .collect::<Result<Vec<_>, _>>()
                .map_err(kerr)?;
            k.make_compound(&parts).map_err(kerr)?
        };
        let map = sub_map(shape, &op, faces)?;
        let op_names = sub_naming(naming, &map)?;
        let fail = |e, out: &mut FeatureOutput| {
            kernel_failure(ctx, Some(body), to_body_space(e, &map), out)
        };
        let join = OffsetJoin::Intersection;
        let r = match self.side {
            side @ (ThinSide::Side1 | ThinSide::Side2) => {
                let t = if side == ThinSide::Side1 { t } else { -t };
                with_job(ctx, |job| k.thicken(&op, t, join, job)).map_err(|e| fail(e, out))?
            }
            ThinSide::Center => {
                let back = with_job(ctx, |job| k.offset(&op, -0.5 * t, join, 0.0, job))
                    .map_err(|e| fail(e, out))?;
                let th =
                    with_job(ctx, |job| k.thicken(&back.shape, t, join, job)).map_err(|e| {
                        fail(crate::solid::error::remap_offending(e, &back.history), out)
                    })?;
                // The offset faces are the thickened solid's base: drop the offset role so
                // they keep their names (Modified, not re-generated).
                let mut first = back.history;
                first.roles.clear();
                amg_brep::OpResult {
                    history: first.compose(&th.history),
                    shape: th.shape,
                    warnings: th.warnings,
                }
            }
        };
        let mut b = ctx.naming_builder();
        b.from_history(&[(0, OperandNaming::Named(&op_names))], &r.history);
        b.role(OpRole::OffsetFace, Role::new("Offset", 0));
        b.role(OpRole::Lateral, Role::new("ThickenSide", 0));
        let names = finish_naming(b, k, &r.shape, out)?;
        Ok((r.shape, names))
    }
}

impl SolidTool for Thicken {
    fn tool(&self, ctx: &RecomputeCtx<'_>) -> Result<ToolBody, FeatureError> {
        let t = positive("thickness", ctx.eval_length(&self.thickness)?)?;
        if self.faces.is_empty() {
            return Err(invalid("faces", "select faces to thicken"));
        }
        let k = op_kernel(ctx);
        let mut out = FeatureOutput::default();
        let mut parts: Vec<(Shape, BodyNaming)> = Vec::new();
        for (body, subs) in resolve_subs(ctx, &self.faces, SetPolicy::DropMissing)? {
            ctx.check_cancelled()?;
            let mut faces: Vec<u32> = Vec::new();
            for (s, _) in subs {
                if s.kind != SubKind::Face {
                    return Err(invalid("faces", "select faces"));
                }
                faces.push(s.index);
            }
            parts.push(self.thicken_body(ctx, k, body, &faces, t, &mut out)?);
        }
        let (shape, naming) = compound_of(ctx, k, parts, &mut out)?;
        Ok(ToolBody {
            shape,
            naming,
            target: self.target.clone(),
        })
    }
}

impl Feature for Thicken {
    fn inputs(&self) -> Vec<Dependency> {
        let mut d = super::face_inputs(std::iter::empty(), &self.faces);
        for dep in self.target.inputs() {
            if !d.contains(&dep) {
                d.push(dep);
            }
        }
        d
    }

    fn input_is_optional(&self, dep: &Dependency) -> bool {
        // Picked faces follow body lineage (SOL-07); participants stay strict as for extrude.
        matches!(dep, Dependency::Entity(EntityRef::Topo(_)))
            || (matches!(dep, Dependency::Body(b) if self.faces.iter().any(|r| r.body == *b))
                && !self.target.inputs().any(|p| p == *dep))
    }

    fn exprs(&self) -> Vec<&Expr> {
        vec![&self.thickness]
    }

    fn exprs_mut(&mut self) -> Vec<(&mut Expr, Dim)> {
        vec![(&mut self.thickness, Dim::LENGTH)]
    }

    fn validate(&self) -> Result<(), FeatureError> {
        if self.faces.iter().any(|r| r.kind != TopoKind::Face) {
            return Err(invalid("faces", "select faces"));
        }
        Ok(())
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let tool = self.tool(ctx)?;
        let mut out = FeatureOutput::default();
        apply_body_op(ctx, tool, &mut out)?;
        Ok(out)
    }
}

register_feature!(Thicken, "solid.thicken", version = 1);
register_tool!(Thicken, "solid.thicken");

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solid::common::BodyOp;

    #[test]
    fn inputs_list_face_bodies_once_and_participants() {
        let b = |o: u64| BodyId {
            origin: amg_doc::FeatureId(o),
            slot: 0,
        };
        let face = |o: u64, i: u32| TopoRef {
            body: b(o),
            kind: TopoKind::Face,
            name: TopoName::new(Vec::new()),
            key: i,
            fingerprint: None,
        };
        let mut t = Thicken::new_body(vec![face(1, 1), face(1, 2)], "1 mm");
        t.target = OpTarget {
            op: BodyOp::Join,
            participants: vec![b(2)],
        };
        let d = t.inputs();
        assert_eq!(
            d.iter().filter(|x| **x == Dependency::Body(b(1))).count(),
            1
        );
        assert!(d.contains(&Dependency::Body(b(2))));
        assert!(t.input_is_optional(&Dependency::Body(b(1))));
        assert!(!t.input_is_optional(&Dependency::Body(b(2))));
    }
}
