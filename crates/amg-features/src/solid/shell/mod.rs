//! `solid.shell` (S04 SOL-06, BREP-07): a solid hollowed to walls of a given thickness, open
//! at the removed faces; no removed face gives a hollow body with a closed void (R-S04-8).
//! Also `solid.thicken` ([`thicken`]).
//!
//! - **Bodies.** The bodies of the removed faces, plus `bodies` (shelled with no opening when
//!   none of their faces is picked).
//! - **Direction.** `Inside`: walls of `inside` grow into the body (its faces stay the outside).
//!   `Outside`: walls of `outside` grow outward (its faces become the inside). `Both`: the
//!   kept faces are first moved out by `outside` (`offset_faces`, neighbours re-intersected,
//!   removed faces stay where they are), then that body is shelled inward by
//!   `inside + outside`, so the walls straddle the original faces.
//! - **Tangent chain.** Each removed face grows along the curved faces tangent to it
//!   ([`tangent_blend_faces`]): the top of a box with filleted top edges removes the fillet
//!   ring too.
//! - Joins are sharp (`OffsetJoin::Intersection`), as Fusion's shell.
//! - A wall thicker than a curvature radius it offsets towards (a convex fillet for an inward
//!   wall, a concave one for an outward wall) fails the kernel's collapse check; the faces
//!   are highlighted (`kernel_failure`).
//!
//! Naming (naming note §5.2 "Shell"): wall faces `ShellInner#0` from the face they offset
//! (`Topo*`, a propagated input); the rim at each opening `ShellRim#0` from the removed face
//! (`Selection(key)` when picked, `Topo*` when added by the tangent chain); arc-join faces
//! `ShellJoin#0` from their edge. The body's own faces keep their names (`Modified` 1:1).

pub(crate) mod dialog;
pub mod thicken;

use amg_brep::{
    FaceId, InputSub, OffsetJoin, OpResult, OpRole, Relation, ShellOpts, SubKind, SubShape,
};
use amg_doc::naming::{NameSource, OperandNaming, Role, TopoKind};
use amg_doc::{
    BodyChange, BodyChangeKind, BodyId, BodyNaming, Dependency, Dim, EntityRef, Expr, ExprSrc,
    Feature, FeatureError, FeatureOutput, RecomputeCtx, SetPolicy, TopoRef, register_feature,
};
use serde::{Deserialize, Serialize};

use super::common::{
    finish_naming, input_solid, live_body, op_kernel, resolve_subs, solid_geom,
    tangent_blend_faces, with_job,
};
use super::error::{SolidError, kernel_failure, kerr, remap_offending};

pub use thicken::Thicken;

/// Largest angle (rad) between two faces' normals along their shared edge for the tangent
/// chain to cross it (as the fillet chain, SOL-05).
pub const FACE_CHAIN_ANGLE: f64 = 1e-3;

pub(crate) fn inline(src: &str) -> Expr {
    Expr::Inline(ExprSrc(src.to_owned()))
}

pub(crate) fn invalid(field: &'static str, msg: impl Into<String>) -> FeatureError {
    SolidError::Invalid {
        field,
        msg: msg.into(),
    }
    .into()
}

pub(crate) fn positive(field: &'static str, v: f64) -> Result<f64, FeatureError> {
    if v.is_finite() && v > 0.0 {
        Ok(v)
    } else {
        Err(invalid(field, format!("must be larger than 0, got {v}")))
    }
}

/// Which side of the faces the walls grow on.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ShellDirection {
    /// Into the body, by `inside`.
    #[default]
    Inside,
    /// Out of the body, by `outside`.
    Outside,
    /// `inside` into and `outside` out of the body.
    Both,
}

fn yes() -> bool {
    true
}

/// `solid.shell`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Shell {
    /// Bodies shelled without an opening unless some of their faces are in `remove`.
    #[serde(default)]
    pub bodies: Vec<BodyId>,
    /// Faces to remove (set-valued; may span bodies).
    #[serde(default)]
    pub remove: Vec<TopoRef>,
    /// Inside wall thickness (length).
    pub inside: Expr,
    /// Outside wall thickness (length).
    pub outside: Expr,
    /// Which side the walls grow on.
    #[serde(default)]
    pub direction: ShellDirection,
    /// Grow each removed face along the curved faces tangent to it.
    #[serde(default = "yes")]
    pub tangent_chain: bool,
}

impl Shell {
    /// An inward shell of `thickness` (such as `"1 mm"`) opening at `remove`, tangent chain on.
    pub fn inside(remove: Vec<TopoRef>, thickness: &str) -> Shell {
        Shell {
            bodies: Vec::new(),
            remove,
            inside: inline(thickness),
            outside: inline(thickness),
            direction: ShellDirection::Inside,
            tangent_chain: true,
        }
    }

    /// The removed faces per body, in kernel order: picks first (with their keys), then the
    /// faces the tangent chain adds; bodies with no removed face are hollowed.
    ///
    /// # Errors
    /// Resolution errors; [`SolidError::Invalid`] for picks that are not faces or nothing to
    /// shell.
    pub fn plan(&self, ctx: &RecomputeCtx<'_>) -> Result<Vec<BodyFaces>, FeatureError> {
        let mut plan = plan_faces(ctx, &self.remove, self.tangent_chain, "faces")?;
        let mut out = FeatureOutput::default();
        for &b in &self.bodies {
            let Some(p) = live_body(ctx, b, &mut out)? else {
                continue;
            };
            if !plan.iter().any(|x| x.body == p.id) {
                plan.push(BodyFaces {
                    body: p.id,
                    faces: Vec::new(),
                });
            }
        }
        if plan.is_empty() {
            return Err(invalid(
                "faces",
                "select faces to remove or bodies to hollow",
            ));
        }
        Ok(plan)
    }
}

/// One face of a plan: the face (input index space of the body) and the selection key of a
/// picked face (`None` for a face added by the tangent chain).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct PlannedFace {
    /// The face.
    pub face: FaceId,
    /// Its selection key when picked.
    pub key: Option<u32>,
}

/// The faces a shell removes (or a draft tilts) on one body.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BodyFaces {
    /// The body (as it is just before the feature).
    pub body: BodyId,
    /// Its faces, picks first.
    pub faces: Vec<PlannedFace>,
}

impl BodyFaces {
    /// Just the face ids, in kernel order.
    pub fn face_ids(&self) -> Vec<FaceId> {
        self.faces.iter().map(|f| f.face).collect()
    }
}

/// Picked faces per body, tangent blends added (shared by shell and draft).
pub(crate) fn plan_faces(
    ctx: &RecomputeCtx<'_>,
    refs: &[TopoRef],
    tangent_chain: bool,
    field: &'static str,
) -> Result<Vec<BodyFaces>, FeatureError> {
    if refs.is_empty() {
        return Ok(Vec::new());
    }
    let mut plan: Vec<BodyFaces> = Vec::new();
    for (body, subs) in resolve_subs(ctx, refs, SetPolicy::DropMissing)? {
        let mut faces: Vec<PlannedFace> = Vec::new();
        for (sub, key) in subs {
            if sub.kind != SubKind::Face {
                return Err(invalid(field, "select faces"));
            }
            if !faces.iter().any(|f| f.face.0 == sub.index) {
                faces.push(PlannedFace {
                    face: FaceId(sub.index),
                    key: Some(key),
                });
            }
        }
        plan.push(BodyFaces { body, faces });
    }
    if tangent_chain {
        let k = op_kernel(ctx);
        for p in &mut plan {
            let (shape, _) = input_solid(ctx, p.body)?;
            let chain =
                tangent_blend_faces(k, shape, &p.face_ids(), FACE_CHAIN_ANGLE).map_err(kerr)?;
            for f in chain {
                if !p.faces.iter().any(|x| x.face == f) {
                    p.faces.push(PlannedFace { face: f, key: None });
                }
            }
        }
    }
    Ok(plan)
}

fn shell_opts(thickness: f64) -> ShellOpts {
    ShellOpts {
        thickness,
        join: OffsetJoin::Intersection,
        per_face: Vec::new(),
        tol: 0.0,
    }
}

/// The image of input face `f` (operand 0) in a 1:1 history.
fn image_of(r: &OpResult, f: FaceId) -> Option<FaceId> {
    r.history
        .of_input(InputSub {
            operand: 0,
            sub: SubShape {
                kind: SubKind::Face,
                index: f.0,
            },
        })
        .find_map(|rel| match rel {
            Relation::Modified(v) | Relation::Generated(v) => v.first().copied(),
            Relation::Kept(s) => Some(*s),
            Relation::Deleted => None,
        })
        .map(|s| FaceId(s.index))
}

impl Shell {
    fn thicknesses(&self, ctx: &RecomputeCtx<'_>) -> Result<(f64, f64), FeatureError> {
        let (i, o) = match self.direction {
            ShellDirection::Inside => (positive("inside", ctx.eval_length(&self.inside)?)?, 0.0),
            ShellDirection::Outside => (0.0, positive("outside", ctx.eval_length(&self.outside)?)?),
            ShellDirection::Both => (
                positive("inside", ctx.eval_length(&self.inside)?)?,
                positive("outside", ctx.eval_length(&self.outside)?)?,
            ),
        };
        Ok((i, o))
    }

    /// Shells one body; returns the result and its names.
    fn shell_body(
        &self,
        ctx: &RecomputeCtx<'_>,
        p: &BodyFaces,
        (inside, outside): (f64, f64),
        out: &mut FeatureOutput,
    ) -> Result<(OpResult, BodyNaming), FeatureError> {
        let (shape, _) = input_solid(ctx, p.body)?;
        let removed = p.face_ids();
        let fail = |e: amg_brep::KernelError, out: &mut FeatureOutput| {
            kernel_failure(ctx, Some(p.body), e, out)
        };
        let r = self.shell_raw(ctx, p, (inside, outside), out, &fail)?;
        let walls =
            r.0.history
                .roles
                .iter()
                .any(|(s, role)| s.kind == SubKind::Face && *role == OpRole::ShellInner);
        if !walls {
            // OCCT returns the body unchanged when it can't open it, e.g. at a removed planar
            // face tangent to fillets that stay.
            tangent_guard(ctx, p.body, shape, &removed)?;
            return Err(FeatureError::Kernel(
                "shell made no walls (the kernel returned the body unchanged)".into(),
            ));
        }
        Ok(r)
    }

    fn shell_raw(
        &self,
        ctx: &RecomputeCtx<'_>,
        p: &BodyFaces,
        (inside, outside): (f64, f64),
        out: &mut FeatureOutput,
        fail: &dyn Fn(amg_brep::KernelError, &mut FeatureOutput) -> FeatureError,
    ) -> Result<(OpResult, BodyNaming), FeatureError> {
        let k = op_kernel(ctx);
        let (shape, naming) = input_solid(ctx, p.body)?;
        let removed = p.face_ids();
        match self.direction {
            ShellDirection::Inside | ShellDirection::Outside => {
                let t = if self.direction == ShellDirection::Inside {
                    -inside
                } else {
                    outside
                };
                let r = with_job(ctx, |job| k.shell(shape, &removed, &shell_opts(t), job))
                    .map_err(|e| fail(e, out))?;
                let n = name_shell(ctx, naming, &r, p, out)?;
                Ok((r, n))
            }
            ShellDirection::Both => {
                let moved: Vec<(FaceId, f64)> = shape
                    .faces()
                    .filter(|f| !removed.contains(f))
                    .map(|f| (f, outside))
                    .collect();
                let grown = with_job(ctx, |job| k.offset_faces(shape, &moved, job))
                    .map_err(|e| fail(e, out))?;
                let mut b = ctx.naming_builder();
                b.from_history(&[(0, OperandNaming::Named(naming))], &grown.history);
                let grown_names = finish_naming(b, k, &grown.shape, out)?;
                let mapped = BodyFaces {
                    body: p.body,
                    faces: p
                        .faces
                        .iter()
                        .map(|f| {
                            image_of(&grown, f.face)
                                .map(|face| PlannedFace { face, key: f.key })
                                .ok_or_else(|| {
                                    FeatureError::Kernel(format!(
                                        "shell: removed face {} has no image",
                                        f.face.0
                                    ))
                                })
                        })
                        .collect::<Result<_, _>>()?,
                };
                let r = with_job(ctx, |job| {
                    k.shell(
                        &grown.shape,
                        &mapped.face_ids(),
                        &shell_opts(-(inside + outside)),
                        job,
                    )
                })
                .map_err(|e| fail(remap_offending(e, &grown.history), out))?;
                let n = name_shell(ctx, &grown_names, &r, &mapped, out)?;
                Ok((r, n))
            }
        }
    }
}

/// After a shell that made no walls: when removed faces meet kept faces tangentially, a field
/// error with the kept tangent faces highlighted (OCCT can't open a shell at a planar face
/// tangent to fillets that stay; a removed fillet tangent to a kept wall is fine).
fn tangent_guard(
    ctx: &RecomputeCtx<'_>,
    body: BodyId,
    shape: &amg_brep::Shape,
    removed: &[FaceId],
) -> Result<(), FeatureError> {
    if removed.is_empty() {
        return Ok(());
    }
    let k = op_kernel(ctx);
    let chain =
        super::common::tangent_chain_faces(k, shape, removed, FACE_CHAIN_ANGLE).map_err(kerr)?;
    let kept: Vec<FaceId> = chain
        .into_iter()
        .filter(|f| !removed.contains(f))
        .filter(|f| {
            shape
                .face_edges(*f)
                .iter()
                .any(|e| shape.edge_faces(*e).iter().any(|g| removed.contains(g)))
        })
        .collect();
    if kept.is_empty() {
        return Ok(());
    }
    ctx.highlight(kept.iter().filter_map(|f| {
        amg_doc::naming::topo_ref_at(ctx.before(), ctx.kernel(), body, TopoKind::Face, f.0)
            .map(EntityRef::Topo)
    }));
    Err(invalid(
        "faces",
        "a removed face is tangent to faces that stay. Turn on Tangent chain or remove the highlighted faces too.",
    ))
}

/// Names a shell result (module docs).
fn name_shell(
    ctx: &RecomputeCtx<'_>,
    naming: &BodyNaming,
    r: &OpResult,
    p: &BodyFaces,
    out: &mut FeatureOutput,
) -> Result<BodyNaming, FeatureError> {
    let k = op_kernel(ctx);
    let mut b = ctx.naming_builder();
    b.from_history(&[(0, OperandNaming::Named(naming))], &r.history);
    b.role(OpRole::ShellInner, Role::new("ShellInner", 0));
    b.role(OpRole::Lateral, Role::new("ShellRim", 0));
    for f in &p.faces {
        if let Some(key) = f.key {
            b.selection(
                0,
                SubShape {
                    kind: SubKind::Face,
                    index: f.face.0,
                },
                key,
            );
        }
    }
    // Arc-join faces: offset-skin faces generated from an edge or a vertex, not a face.
    for (sub, role) in &r.history.roles {
        if sub.kind != SubKind::Face || *role != OpRole::ShellInner {
            continue;
        }
        let sources = r.history.sources_of(*sub);
        if sources.iter().any(|(i, _)| i.sub.kind == SubKind::Face) {
            continue;
        }
        let from = sources
            .iter()
            .find(|(i, _)| i.sub.kind == SubKind::Edge)
            .and_then(|(i, _)| naming.name_of(TopoKind::Edge, i.sub.index))
            .map(|n| NameSource::Topo(n.clone()));
        b.generated(TopoKind::Face, sub.index, Role::new("ShellJoin", 0), from);
    }
    finish_naming(b, k, &r.shape, out)
}

/// `Dependency::Body` once per body, then `Dependency::Entity` per face ref.
pub(crate) fn face_inputs<'a>(
    bodies: impl Iterator<Item = &'a BodyId>,
    refs: &'a [TopoRef],
) -> Vec<Dependency> {
    let mut d: Vec<Dependency> = Vec::new();
    for b in bodies.chain(refs.iter().map(|r| &r.body)) {
        let dep = Dependency::Body(*b);
        if !d.contains(&dep) {
            d.push(dep);
        }
    }
    d.extend(
        refs.iter()
            .map(|r| Dependency::Entity(EntityRef::Topo(r.clone()))),
    );
    d
}

impl Feature for Shell {
    fn inputs(&self) -> Vec<Dependency> {
        face_inputs(self.bodies.iter(), &self.remove)
    }

    fn input_is_optional(&self, dep: &Dependency) -> bool {
        super::fillet::lineage_optional(dep)
    }

    fn exprs(&self) -> Vec<&Expr> {
        vec![&self.inside, &self.outside]
    }

    fn exprs_mut(&mut self) -> Vec<(&mut Expr, Dim)> {
        vec![
            (&mut self.inside, Dim::LENGTH),
            (&mut self.outside, Dim::LENGTH),
        ]
    }

    fn validate(&self) -> Result<(), FeatureError> {
        if self.remove.iter().any(|r| r.kind != TopoKind::Face) {
            return Err(invalid("faces", "select faces"));
        }
        Ok(())
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let t = self.thicknesses(ctx)?;
        let plan = self.plan(ctx)?;
        let mut out = FeatureOutput::default();
        for p in &plan {
            ctx.check_cancelled()?;
            let (r, naming) = self.shell_body(ctx, p, t, &mut out)?;
            out.bodies.push(BodyChange {
                id: p.body,
                change: BodyChangeKind::Modified {
                    geom: solid_geom(r.shape),
                    naming: Some(naming),
                },
            });
        }
        Ok(out)
    }
}

register_feature!(Shell, "solid.shell", version = 1);
