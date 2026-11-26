//! Direct face edits (S04 SOL-09): `solid.offset_face` (R-S04-5), `solid.replace_face`,
//! `solid.delete_face` (heal or remove), and the press-pull command dispatch
//! ([`dialog::press_pull`]).
//!
//! - **Offset face** moves the picked faces along their outward normals by one distance
//!   (`BrepKernel::offset_faces`, neighbours re-intersected), one kernel call per body.
//! - **Replace face** puts each picked face on a new surface (`BrepKernel::replace_face`): an
//!   origin or construction plane, a planar face, or any face's untrimmed surface. With
//!   `tangent_chain` the faces tangent to the picks are replaced too. One kernel call per face,
//!   the histories composed.
//! - **Delete face** with `heal` removes the faces and closes the gap by extending the
//!   neighbours (`BrepKernel::defeature`, Fusion "Delete"); without it the faces are just
//!   dropped and the rest is sewn into an open shell (Fusion "Remove"), so the body becomes a
//!   surface body whose free edges are the removed faces' boundary.
//!
//! Naming (naming note §5.2): the ops have no names of their own. Moved, replaced and healed
//! faces keep their names through `Modified` histories (a replaced face keeps the replaced
//! face's name); deleted faces leave tombstones.
//!
//! Picks resolve with `SetPolicy::DropMissing` and are grouped per body.

pub(crate) mod dialog;

use amg_brep::{
    BrepKernel, EdgeId, FaceId, InputSub, KResult, OpResult, Relation, SewOpts, Shape,
    ShapeHistory, SubKind, SubShape, VertexId, geom::Surface,
};
use amg_doc::naming::OperandNaming;
use amg_doc::{
    BodyChange, BodyChangeKind, BodyId, Dependency, Dim, EntityRef, Expr, ExprSrc, Feature,
    FeatureError, FeatureOutput, RecomputeCtx, SetPolicy, TopoKind, TopoRef, register_feature,
};
use serde::{Deserialize, Serialize};

use super::common::{finish_naming, input_solid, op_kernel, resolve_subs, solid_geom, with_job};
use super::error::{SolidError, kernel_failure, kerr};
use crate::construct::RefInput;
use crate::construct::resolve::resolve_plane_input;

pub use dialog::press_pull;

// ---- data -------------------------------------------------------------------------------------

/// `solid.offset_face` (R-S04-5): faces moved along their outward normals.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OffsetFace {
    /// The faces (any bodies).
    #[serde(default)]
    pub faces: Vec<TopoRef>,
    /// The distance along the outward normal (negative: inward), mm.
    pub distance: Expr,
}

/// `solid.replace_face`: faces put on a new surface.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReplaceFace {
    /// The faces to replace.
    #[serde(default)]
    pub faces: Vec<TopoRef>,
    /// The new surface: an origin or construction plane, or a face (its untrimmed surface).
    pub with: RefInput,
    /// Also replace the faces tangent to the picked ones.
    #[serde(default)]
    pub tangent_chain: bool,
}

/// `solid.delete_face`. `heal = false` leaves an open (sheet) body (Fusion "Remove").
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DeleteFace {
    /// The faces to delete.
    #[serde(default)]
    pub faces: Vec<TopoRef>,
    /// Close the gap by extending the neighbours (else leave it open).
    #[serde(default = "yes")]
    pub heal: bool,
}

fn yes() -> bool {
    true
}

impl OffsetFace {
    /// An offset of `faces` by `distance` (expression text, e.g. `"2 mm"`).
    pub fn new(faces: Vec<TopoRef>, distance: &str) -> OffsetFace {
        OffsetFace {
            faces,
            distance: Expr::Inline(ExprSrc(distance.to_owned())),
        }
    }
}

impl ReplaceFace {
    /// `faces` replaced by the surface of `with`, no tangent chain.
    pub fn new(faces: Vec<TopoRef>, with: RefInput) -> ReplaceFace {
        ReplaceFace {
            faces,
            with,
            tangent_chain: false,
        }
    }
}

impl DeleteFace {
    /// A delete of `faces`, healed or not.
    pub fn new(faces: Vec<TopoRef>, heal: bool) -> DeleteFace {
        DeleteFace { faces, heal }
    }
}

fn invalid(field: &'static str, msg: impl Into<String>) -> FeatureError {
    SolidError::Invalid {
        field,
        msg: msg.into(),
    }
    .into()
}

/// `Dependency::Body` per body (once) and `Dependency::Entity` per face.
fn face_inputs(refs: &[TopoRef]) -> Vec<Dependency> {
    let mut d: Vec<Dependency> = Vec::new();
    for r in refs {
        let b = Dependency::Body(r.body);
        if !d.contains(&b) {
            d.push(b);
        }
        d.push(Dependency::Entity(EntityRef::Topo(r.clone())));
    }
    d
}

/// The picked faces per body (at least one face somewhere).
fn picked_faces(
    ctx: &RecomputeCtx<'_>,
    refs: &[TopoRef],
) -> Result<Vec<(BodyId, Vec<FaceId>)>, FeatureError> {
    if refs.is_empty() {
        return Err(invalid("faces", "select at least one face"));
    }
    let groups = resolve_subs(ctx, refs, SetPolicy::DropMissing)?;
    let mut out = Vec::new();
    for (body, subs) in groups {
        let mut faces = Vec::new();
        for (s, _) in subs {
            if s.kind != SubKind::Face {
                return Err(invalid("faces", "select faces"));
            }
            faces.push(FaceId(s.index));
        }
        if !faces.is_empty() {
            out.push((body, faces));
        }
    }
    if out.is_empty() {
        return Err(invalid("faces", "none of the selected faces is left"));
    }
    Ok(out)
}

/// Names `r` from `naming` through its one-operand history and appends `Modified`.
fn emit_modified(
    ctx: &RecomputeCtx<'_>,
    k: &dyn BrepKernel,
    body: BodyId,
    naming: &amg_doc::BodyNaming,
    r: OpResult,
    out: &mut FeatureOutput,
) -> Result<(), FeatureError> {
    let mut b = ctx.naming_builder();
    b.from_history(&[(0, OperandNaming::Named(naming))], &r.history);
    let n = finish_naming(b, k, &r.shape, out)?;
    out.bodies.push(BodyChange {
        id: body,
        change: BodyChangeKind::Modified {
            geom: solid_geom(r.shape),
            naming: Some(n),
        },
    });
    Ok(())
}

// ---- offset face --------------------------------------------------------------------------------

impl Feature for OffsetFace {
    fn inputs(&self) -> Vec<Dependency> {
        face_inputs(&self.faces)
    }

    fn input_is_optional(&self, dep: &Dependency) -> bool {
        super::fillet::lineage_optional(dep)
    }

    fn exprs(&self) -> Vec<&Expr> {
        vec![&self.distance]
    }

    fn exprs_mut(&mut self) -> Vec<(&mut Expr, Dim)> {
        vec![(&mut self.distance, Dim::LENGTH)]
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let mut out = FeatureOutput::default();
        let d = ctx.eval_length(&self.distance)?;
        if !d.is_finite() {
            return Err(invalid("distance", "the distance is not finite"));
        }
        let groups = picked_faces(ctx, &self.faces)?;
        if d.abs() <= ctx.tolerance().linear {
            out.warnings.push(super::common::warning(
                "The offset distance is zero; the faces stay where they are.".into(),
            ));
            return Ok(out);
        }
        let k = op_kernel(ctx);
        for (body, faces) in groups {
            ctx.check_cancelled()?;
            let (shape, naming) = input_solid(ctx, body)?;
            let moves: Vec<(FaceId, f64)> = faces.iter().map(|f| (*f, d)).collect();
            let r = with_job(ctx, |job| k.offset_faces(shape, &moves, job))
                .map_err(|e| kernel_failure(ctx, Some(body), e, &mut out))?;
            emit_modified(ctx, k, body, naming, r, &mut out)?;
        }
        Ok(out)
    }
}

register_feature!(OffsetFace, "solid.offset_face", version = 1);

// ---- replace face -------------------------------------------------------------------------------

/// The faces G1-continuous with `seeds` across shared edges: a neighbour joins when the outward
/// normals of both faces agree within `angular` rad at the edge's two ends and middle. Seeds
/// first (input order), then the propagated faces breadth-first.
///
/// # Errors
/// Kernel errors of the edge and surface queries.
pub(crate) fn tangent_faces(
    k: &dyn BrepKernel,
    s: &Shape,
    seeds: &[FaceId],
    angular: f64,
) -> KResult<Vec<FaceId>> {
    let cos = angular.cos();
    let normal_at = |f: FaceId, p: amg_core::Point3| -> KResult<amg_core::Vec3> {
        let (uv, _) = k.project_point_on_face(s, f, p)?;
        Ok(k.eval_surface(s, f, uv)?.normal.get())
    };
    let smooth = |e: EdgeId, a: FaceId, b: FaceId| -> KResult<bool> {
        let ends = k.edge_ends(s, e)?;
        let mid = k
            .tessellate_edge(s, e, &amg_brep::TessQuality::preview())
            .ok()
            .and_then(|pts| pts.get(pts.len() / 2).copied())
            .unwrap_or_else(|| ends.p0.lerp(ends.p1, 0.5));
        for p in [ends.p0, mid, ends.p1] {
            let (na, nb) = (normal_at(a, p)?, normal_at(b, p)?);
            if na.dot(nb) < cos {
                return Ok(false);
            }
        }
        Ok(true)
    };
    let mut chain: Vec<FaceId> = Vec::new();
    for &f in seeds {
        if !chain.contains(&f) {
            chain.push(f);
        }
    }
    let mut next = 0;
    while next < chain.len() {
        let f = chain[next];
        next += 1;
        for &e in s.face_edges(f) {
            let &[a, b] = s.edge_faces(e) else {
                continue;
            };
            if a == b {
                continue;
            }
            let o = if a == f { b } else { a };
            if chain.contains(&o) {
                continue;
            }
            if smooth(e, f, o).unwrap_or(false) {
                chain.push(o);
            }
        }
    }
    Ok(chain)
}

/// The tangent-chain angle of replace face (as fillet's `CHAIN_ANGLE`).
const CHAIN_ANGLE: f64 = 1e-3;

impl ReplaceFace {
    /// The replacing surface.
    fn surface(&self, ctx: &RecomputeCtx<'_>) -> Result<Surface, FeatureError> {
        if let RefInput::Ref(EntityRef::Topo(t)) = &self.with
            && t.kind == TopoKind::Face
        {
            let r = ctx.resolve(t)?;
            let (shape, _) = input_solid(ctx, r.body)?;
            return Ok(shape.surface(FaceId(r.index)));
        }
        Ok(Surface::Plane(resolve_plane_input(ctx, &self.with)?))
    }
}

/// `faces` of `shape` replaced one after the other, histories composed (operand 0 = `shape`).
fn replace_all(
    ctx: &RecomputeCtx<'_>,
    k: &dyn BrepKernel,
    shape: &Shape,
    faces: &[FaceId],
    with: &Surface,
) -> KResult<OpResult> {
    let mut cur = super::common::identity_result(shape);
    for &f in faces {
        // Where the face is now (its images after the earlier replacements).
        let now: Vec<FaceId> = cur
            .history
            .of_input(InputSub {
                operand: 0,
                sub: SubShape {
                    kind: SubKind::Face,
                    index: f.0,
                },
            })
            .flat_map(|r| match r {
                Relation::Kept(s) => vec![*s],
                Relation::Modified(v) | Relation::Generated(v) => v.iter().copied().collect(),
                Relation::Deleted => Vec::new(),
            })
            .filter(|s| s.kind == SubKind::Face)
            .map(|s| FaceId(s.index))
            .collect();
        for g in now {
            if g.0 >= cur.shape.count(SubKind::Face) {
                continue;
            }
            let r = with_job(ctx, |job| k.replace_face(&cur.shape, g, with, job))?;
            let history = cur.history.compose(&r.history);
            let mut warnings = cur.warnings;
            warnings.extend(r.warnings);
            cur = OpResult {
                shape: r.shape,
                history,
                warnings,
            };
        }
    }
    Ok(cur)
}

impl Feature for ReplaceFace {
    fn inputs(&self) -> Vec<Dependency> {
        let mut d = face_inputs(&self.faces);
        match &self.with {
            RefInput::Ref(EntityRef::Topo(t)) => {
                let b = Dependency::Body(t.body);
                if !d.contains(&b) {
                    d.push(b);
                }
                d.push(Dependency::Entity(EntityRef::Topo(t.clone())));
            }
            other => d.extend(super::pattern::ref_dependency(other)),
        }
        d
    }

    fn input_is_optional(&self, dep: &Dependency) -> bool {
        super::fillet::lineage_optional(dep)
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let mut out = FeatureOutput::default();
        let groups = picked_faces(ctx, &self.faces)?;
        let with = self.surface(ctx)?;
        let k = op_kernel(ctx);
        for (body, faces) in groups {
            ctx.check_cancelled()?;
            let (shape, naming) = input_solid(ctx, body)?;
            let faces = if self.tangent_chain {
                tangent_faces(k, shape, &faces, CHAIN_ANGLE).map_err(kerr)?
            } else {
                faces
            };
            let r = replace_all(ctx, k, shape, &faces, &with)
                .map_err(|e| kernel_failure(ctx, Some(body), e, &mut out))?;
            emit_modified(ctx, k, body, naming, r, &mut out)?;
        }
        Ok(out)
    }
}

register_feature!(ReplaceFace, "solid.replace_face", version = 1);

// ---- delete face --------------------------------------------------------------------------------

/// `shape` without `faces`, the rest sewn into an open shell (operand 0 = `shape`): kept faces
/// and their edges and vertices pass through (`Kept` into a compound of the kept faces, then
/// the sewing's `Modified`); the removed faces, and edges and vertices only they used, are
/// `Deleted`.
pub(crate) fn remove_faces(
    k: &dyn BrepKernel,
    shape: &Shape,
    faces: &[FaceId],
    job: &amg_jobs::JobCtx,
) -> KResult<OpResult> {
    let n = shape.count(SubKind::Face);
    let kept: Vec<FaceId> = shape.faces().filter(|f| !faces.contains(f)).collect();
    if kept.is_empty() {
        return Err(amg_brep::KernelError::InvalidInput {
            what: "faces",
            reason: "every face would be removed".into(),
            offending: Vec::new(),
        });
    }
    let parts: Vec<Shape> = kept
        .iter()
        .map(|f| {
            shape.sub_shape(SubShape {
                kind: SubKind::Face,
                index: f.0,
            })
        })
        .collect::<KResult<_>>()?;
    let compound = k.make_compound(&parts)?;
    // Body → compound: match sub-shapes face by face (shared TShapes).
    let sub = |s: &Shape, kind: SubKind, index: u32| s.sub_shape(SubShape { kind, index });
    let mut face_map: Vec<Option<u32>> = vec![None; n as usize];
    let mut edge_map: Vec<Option<u32>> = vec![None; shape.count(SubKind::Edge) as usize];
    let mut vert_map: Vec<Option<u32>> = vec![None; shape.count(SubKind::Vertex) as usize];
    for (ci, f) in (0u32..).zip(&kept) {
        face_map[f.0 as usize] = Some(ci);
        let cedges = compound.face_edges(FaceId(ci)).to_vec();
        let cshapes: Vec<Shape> = cedges
            .iter()
            .map(|e| sub(&compound, SubKind::Edge, e.0))
            .collect::<KResult<_>>()?;
        for &e in shape.face_edges(*f) {
            if edge_map[e.0 as usize].is_some() {
                continue;
            }
            let es = sub(shape, SubKind::Edge, e.0)?;
            let Some(j) = cshapes.iter().position(|c| c.is_same(&es)) else {
                continue;
            };
            let ce = cedges[j];
            edge_map[e.0 as usize] = Some(ce.0);
            let cv = compound.edge_vertices(ce);
            let cvs: Vec<Option<Shape>> = cv
                .iter()
                .map(|v| {
                    (v.0 != u32::MAX)
                        .then(|| sub(&compound, SubKind::Vertex, v.0))
                        .transpose()
                })
                .collect::<KResult<_>>()?;
            for v in shape.edge_vertices(e) {
                if v == VertexId(u32::MAX) || vert_map[v.0 as usize].is_some() {
                    continue;
                }
                let vs = sub(shape, SubKind::Vertex, v.0)?;
                if let Some(j) = cvs
                    .iter()
                    .position(|c| c.as_ref().is_some_and(|c| c.is_same(&vs)))
                {
                    vert_map[v.0 as usize] = Some(cv[j].0);
                }
            }
        }
    }
    let mut entries = Vec::new();
    for (kind, map) in [
        (SubKind::Face, &face_map),
        (SubKind::Edge, &edge_map),
        (SubKind::Vertex, &vert_map),
    ] {
        for (i, m) in (0u32..).zip(map.iter()) {
            let input = InputSub {
                operand: 0,
                sub: SubShape { kind, index: i },
            };
            let rel = match m {
                Some(o) => Relation::Kept(SubShape { kind, index: *o }),
                None => Relation::Deleted,
            };
            entries.push((input, rel));
        }
    }
    let first = ShapeHistory {
        entries,
        roles: Vec::new(),
        tags: Vec::new(),
        operand_counts: vec![[n, shape.count(SubKind::Edge), shape.count(SubKind::Vertex)]],
    };
    let sewn = k.sew(
        std::slice::from_ref(&compound),
        &SewOpts {
            make_solid: false,
            ..SewOpts::default()
        },
        job,
    )?;
    Ok(OpResult {
        history: first.compose(&sewn.history),
        shape: sewn.shape,
        warnings: sewn.warnings,
    })
}

impl Feature for DeleteFace {
    fn inputs(&self) -> Vec<Dependency> {
        face_inputs(&self.faces)
    }

    fn input_is_optional(&self, dep: &Dependency) -> bool {
        super::fillet::lineage_optional(dep)
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let mut out = FeatureOutput::default();
        let groups = picked_faces(ctx, &self.faces)?;
        let k = op_kernel(ctx);
        for (body, faces) in groups {
            ctx.check_cancelled()?;
            let (shape, naming) = input_solid(ctx, body)?;
            let r = if self.heal {
                with_job(ctx, |job| k.defeature(shape, &faces, job))
            } else {
                with_job(ctx, |job| remove_faces(k, shape, &faces, job))
            }
            .map_err(|e| kernel_failure(ctx, Some(body), e, &mut out))?;
            emit_modified(ctx, k, body, naming, r, &mut out)?;
        }
        Ok(out)
    }
}

register_feature!(DeleteFace, "solid.delete_face", version = 1);

#[cfg(test)]
mod tests {
    use super::*;
    use amg_doc::FeatureId;

    #[test]
    fn inputs_list_each_body_once_and_every_face() {
        let body = BodyId {
            origin: FeatureId(1),
            slot: 0,
        };
        let face = |key: u32| TopoRef {
            body,
            kind: TopoKind::Face,
            name: amg_doc::naming::TopoName::default(),
            fingerprint: None,
            key,
        };
        let o = OffsetFace::new(vec![face(1), face(2)], "1 mm");
        let d = o.inputs();
        assert_eq!(d.len(), 3);
        assert_eq!(d[0], Dependency::Body(body));
        let r = ReplaceFace::new(vec![face(1)], RefInput::Ref(EntityRef::Topo(face(3))));
        assert_eq!(r.inputs().len(), 3, "the body once, both faces");
        let json = serde_json::to_string(&DeleteFace::new(vec![], true)).unwrap();
        let back: DeleteFace = serde_json::from_str(r#"{"faces":[]}"#).unwrap();
        assert!(back.heal, "heal defaults to on ({json})");
    }
}
