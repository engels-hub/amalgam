//! Body operations, tools and input helpers shared by every solid feature (S04 §3, §5; note §2).

use std::sync::OnceLock;

use amg_brep::{
    BoolOp, BoolOpts, BrepKernel, InputSub, KResult, KernelConfig, OcctKernel, OpResult, Relation,
    ShapeHistory, SubKind, SubShape, UnifyOpts, Validate,
};
use amg_doc::naming::{NamingBuilder, OperandNaming};
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, BodyNaming, ComputeMode, Dependency, Diagnostic,
    EntityRef, Feature, FeatureError, FeatureOutput, RecomputeCtx, SetPolicy, TopoRef,
};
use amg_jobs::JobCtx;
use serde::{Deserialize, Serialize};

use super::error::{SolidError, kernel_failure, kerr};

// ---- body operations ------------------------------------------------------------------------

/// What a feature does with its tool (note §2.3). "New component" is a dialog option, not a
/// stored value.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum BodyOp {
    /// Each connected tool solid becomes a new body.
    #[default]
    NewBody,
    /// Fuse with every participant the tool touches.
    Join,
    /// Cut every participant the tool touches.
    Cut,
    /// Keep the common part of each participant with the tool.
    Intersect,
}

/// A feature's body operation and the bodies it applies to (Fusion's "Objects to cut").
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OpTarget {
    /// The operation.
    pub op: BodyOp,
    /// The bodies it may change, in priority order (a join keeps the first touched one's id).
    #[serde(default)]
    pub participants: Vec<BodyId>,
}

impl OpTarget {
    /// `Dependency::Body` per participant (none for a new body: participants are ignored).
    pub fn inputs(&self) -> impl Iterator<Item = Dependency> + '_ {
        let used = self.op != BodyOp::NewBody;
        self.participants
            .iter()
            .filter(move |_| used)
            .map(|b| Dependency::Body(*b))
    }
}

/// A feature's tool before its body operation.
#[derive(Clone, Debug)]
pub struct ToolBody {
    /// One solid or a compound of solids, in tool order.
    pub shape: amg_brep::Shape,
    /// Its names, by the producing feature's rules.
    pub naming: BodyNaming,
    /// The body operation to apply.
    pub target: OpTarget,
}

/// Implemented by every additive/subtractive feature: the tool it applies (S04 §5).
pub trait SolidTool {
    /// Builds the tool against the state just before `ctx`'s feature.
    ///
    /// # Errors
    /// Input and kernel failures.
    fn tool(&self, ctx: &RecomputeCtx<'_>) -> Result<ToolBody, FeatureError>;
}

/// The tool function of one feature kind (inventory), so pattern and mirror can rebuild any
/// seed's tool from its `FeatureBox`.
pub struct ToolRegistration {
    /// The feature kind.
    pub kind: &'static str,
    /// Downcasts and calls [`SolidTool::tool`].
    pub tool: fn(&dyn Feature, &RecomputeCtx<'_>) -> Result<ToolBody, FeatureError>,
}

impl std::fmt::Debug for ToolRegistration {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ToolRegistration")
            .field("kind", &self.kind)
            .finish_non_exhaustive()
    }
}

inventory::collect!(ToolRegistration);

/// Registers a [`SolidTool`] type for `tool_of`: `register_tool!(Extrude, "solid.extrude")`
/// next to `register_feature!`.
#[macro_export]
macro_rules! register_tool {
    ($ty:ty, $kind:literal $(,)?) => {
        const _: () = {
            fn tool(
                f: &dyn $crate::__private::amg_doc::Feature,
                ctx: &$crate::__private::amg_doc::RecomputeCtx<'_>,
            ) -> ::core::result::Result<
                $crate::solid::ToolBody,
                $crate::__private::amg_doc::FeatureError,
            > {
                match f.as_any().downcast_ref::<$ty>() {
                    ::core::option::Option::Some(f) => $crate::solid::SolidTool::tool(f, ctx),
                    ::core::option::Option::None => ::core::result::Result::Err(
                        $crate::solid::SolidError::NotPatternable(f.kind().to_owned()).into(),
                    ),
                }
            }
            $crate::__private::inventory::submit! {
                $crate::solid::ToolRegistration { kind: $kind, tool }
            }
        };
    };
}

/// The seed's tool built in `ctx` (pattern, mirror).
///
/// # Errors
/// [`SolidError::NotPatternable`] if the seed's kind has no [`ToolRegistration`]; else the
/// tool's own errors.
pub fn tool_of(seed: &dyn Feature, ctx: &RecomputeCtx<'_>) -> Result<ToolBody, FeatureError> {
    super::link();
    let kind = seed.kind();
    match inventory::iter::<ToolRegistration>
        .into_iter()
        .find(|r| r.kind == kind)
    {
        Some(r) => (r.tool)(seed, ctx),
        None => Err(SolidError::NotPatternable(kind.to_owned()).into()),
    }
}

// ---- kernel access --------------------------------------------------------------------------

/// The kernel for this compute: `ctx.kernel()`, except that a preview on the default kernel
/// uses a copy with validation off (§6: no BRepCheck in preview).
pub fn op_kernel<'a>(ctx: &RecomputeCtx<'a>) -> &'a dyn BrepKernel {
    let k = ctx.kernel();
    if ctx.mode() != ComputeMode::Preview {
        return k;
    }
    let default: &dyn BrepKernel = amg_doc::naming::default_kernel();
    if !std::ptr::addr_eq(k, default) {
        return k;
    }
    static PREVIEW: OnceLock<OcctKernel> = OnceLock::new();
    PREVIEW.get_or_init(|| {
        OcctKernel::new(KernelConfig {
            validate: Validate::Off,
            ..amg_doc::naming::default_kernel().config().clone()
        })
    })
}

/// Runs `f` with the compute's job (a detached one outside a job).
pub(crate) fn with_job<T>(ctx: &RecomputeCtx<'_>, f: impl FnOnce(&JobCtx) -> T) -> T {
    match ctx.job() {
        Some(j) => f(j),
        None => f(&JobCtx::detached()),
    }
}

/// `unify_same_domain { faces, edges }` on `r.shape`, its history composed into `r.history`
/// and the split order applied again (note §2.4). A result without faces passes unchanged.
///
/// # Errors
/// The unify's kernel error.
pub fn simplify(k: &dyn BrepKernel, r: OpResult) -> KResult<OpResult> {
    if r.shape.is_null() || r.shape.count(SubKind::Face) == 0 {
        return Ok(r);
    }
    let u = k.unify_same_domain(&r.shape, &UnifyOpts::default())?;
    let mut history = r.history.compose(&u.history);
    history.order_splits(&u.shape)?;
    let mut warnings = r.warnings;
    warnings.extend(u.warnings);
    Ok(OpResult {
        shape: u.shape,
        history,
        warnings,
    })
}

/// Names `shape` with `b`, warning on partial history.
pub(crate) fn finish_naming(
    b: NamingBuilder,
    k: &dyn BrepKernel,
    shape: &amg_brep::Shape,
    out: &mut FeatureOutput,
) -> Result<BodyNaming, FeatureError> {
    let (naming, report) = b
        .finish_report(k, shape)
        .map_err(|e| FeatureError::Kernel(format!("naming: {e}")))?;
    if report.partial_history() {
        out.warnings.push(warning(format!(
            "Kernel history is incomplete for {} face(s); references to them are weak.",
            report.unexplained.len()
        )));
    }
    Ok(naming)
}

pub(crate) fn warning(message: String) -> Diagnostic {
    Diagnostic {
        message,
        cause: None,
        entities: Vec::new(),
    }
}

// ---- inputs ---------------------------------------------------------------------------------

/// The solid and names of `body` just before this feature.
///
/// # Errors
/// `MissingInput` if the body doesn't exist; [`SolidError::WrongBodyKind`] for a mesh (or
/// other non-B-rep) body or one without an element map.
pub fn input_solid<'a>(
    ctx: &RecomputeCtx<'a>,
    body: BodyId,
) -> Result<(&'a amg_brep::Shape, &'a BodyNaming), FeatureError> {
    let BodyGeom::Solid(s) = ctx.body(body)? else {
        return Err(SolidError::WrongBodyKind.into());
    };
    let shape = s.brep().ok_or(SolidError::WrongBodyKind)?;
    let naming = ctx.naming(body).ok_or(SolidError::WrongBodyKind)?;
    Ok((shape, naming))
}

/// Picks of one body: `(sub-shape, selection key)` in input order.
pub type BodyPicks = (BodyId, Vec<(SubShape, u32)>);

/// Resolved picks grouped by body, in input order: `(sub-shape, selection key)` per entity
/// (key `0` → position + 1, note §5.1).
///
/// # Errors
/// The resolution error under `policy`.
pub fn resolve_subs(
    ctx: &RecomputeCtx<'_>,
    refs: &[TopoRef],
    policy: SetPolicy,
) -> Result<Vec<BodyPicks>, FeatureError> {
    let members = ctx.resolve_set(refs, policy)?;
    let mut out: Vec<BodyPicks> = Vec::new();
    for (pos, (r, member)) in refs.iter().zip(&members).enumerate() {
        let key = if r.key != 0 {
            r.key
        } else {
            u32::try_from(pos + 1).unwrap_or(u32::MAX)
        };
        for x in member {
            let Some(kind) = x.kind.sub_kind() else {
                return Err(SolidError::WrongBodyKind.into());
            };
            let sub = SubShape {
                kind,
                index: x.index,
            };
            let slot = match out.iter().position(|(b, _)| *b == x.body) {
                Some(i) => i,
                None => {
                    out.push((x.body, Vec::new()));
                    out.len() - 1
                }
            };
            if !out[slot].1.iter().any(|(s, _)| *s == sub) {
                out[slot].1.push((sub, key));
            }
        }
    }
    Ok(out)
}

// ---- topology helpers -------------------------------------------------------------------------

/// Edges G1-continuous with `seeds` across shared vertices (S04 §2, R-S04-1): starting from
/// the seeds, an edge joins the chain when, at a vertex it shares with a chain edge, the two
/// edges leave the vertex in opposite directions within `angular` rad (their tangents from
/// [`BrepKernel::edge_ends`]). Only edges between two distinct faces join (seams and free
/// edges can't be blended). Returns the seeds first (input order, duplicates removed), then
/// the propagated edges in breadth-first order.
///
/// # Errors
/// The kernel error of a seed's `edge_ends` (an out-of-range or degenerated seed). A
/// neighbour whose tangent is undefined is skipped.
pub fn tangent_chain_edges(
    k: &dyn BrepKernel,
    s: &amg_brep::Shape,
    seeds: &[amg_brep::EdgeId],
    angular: f64,
) -> KResult<Vec<amg_brep::EdgeId>> {
    use amg_brep::{EdgeEnds, EdgeId};
    use std::collections::BTreeMap;

    /// The directions in which `e` leaves its two end vertices: `t0` at the start, `-t1` at
    /// the end.
    fn leaving(ends: &EdgeEnds) -> [amg_core::Vec3; 2] {
        [ends.t0.get(), -ends.t1.get()]
    }

    let mut cache: BTreeMap<EdgeId, Option<EdgeEnds>> = BTreeMap::new();
    let mut ends_of = |e: EdgeId, strict: bool| -> KResult<Option<EdgeEnds>> {
        if let Some(v) = cache.get(&e) {
            return Ok(v.clone());
        }
        let v = match k.edge_ends(s, e) {
            Ok(v) => Some(v),
            Err(err) if strict => return Err(err),
            Err(_) => None,
        };
        cache.insert(e, v.clone());
        Ok(v)
    };
    let blendable = |e: EdgeId| matches!(*s.edge_faces(e), [a, b] if a != b);
    let cos = angular.cos();
    let mut chain: Vec<EdgeId> = Vec::new();
    for &e in seeds {
        ends_of(e, true)?;
        if !chain.contains(&e) {
            chain.push(e);
        }
    }
    let mut next = 0;
    while next < chain.len() {
        let e = chain[next];
        next += 1;
        let Some(ee) = ends_of(e, false)? else {
            continue;
        };
        let verts = s.edge_vertices(e);
        let dirs = leaving(&ee);
        for (i, &v) in verts.iter().enumerate() {
            for &o in s.vertex_edges(v) {
                if o == e || chain.contains(&o) || !blendable(o) {
                    continue;
                }
                let Some(oe) = ends_of(o, false)? else {
                    continue;
                };
                let overts = s.edge_vertices(o);
                let odirs = leaving(&oe);
                let smooth = (0..2).any(|j| overts[j] == v && dirs[i].dot(odirs[j]) <= -cos);
                if smooth {
                    chain.push(o);
                }
            }
        }
    }
    Ok(chain)
}

/// A participant as it is just before this feature: itself, or its live successor (a body
/// consumed or split since the reference was stored), `None` if it is gone.
fn follow(ctx: &RecomputeCtx<'_>, id: BodyId) -> Option<BodyId> {
    let before = ctx.before();
    let mut cur = id;
    for _ in 0..64 {
        if before.body(cur).is_some() {
            return Some(cur);
        }
        cur = *before.successors(cur).first()?;
    }
    None
}

struct Participant<'a> {
    id: BodyId,
    shape: &'a amg_brep::Shape,
    naming: &'a BodyNaming,
}

/// The live, deduplicated participants (§3): successors followed, missing ones dropped with a
/// warning; each must be a solid of the context component.
fn participants<'a>(
    ctx: &RecomputeCtx<'a>,
    target: &OpTarget,
    out: &mut FeatureOutput,
) -> Result<Vec<Participant<'a>>, FeatureError> {
    let mut v: Vec<Participant<'a>> = Vec::new();
    for &id in &target.participants {
        let Some(live) = follow(ctx, id) else {
            out.warnings.push(Diagnostic {
                message: format!("Body {id:?} no longer exists; it was left out of the operation."),
                cause: None,
                entities: vec![EntityRef::Body(id)],
            });
            continue;
        };
        if v.iter().any(|p| p.id == live) {
            continue;
        }
        let body = ctx
            .before()
            .body(live)
            .ok_or_else(|| FeatureError::MissingInput(Box::new(Dependency::Body(live))))?;
        if body.component != ctx.context() {
            return Err(SolidError::OtherComponent(live).into());
        }
        let (shape, naming) = input_solid(ctx, live)?;
        v.push(Participant {
            id: live,
            shape,
            naming,
        });
    }
    Ok(v)
}

/// Whether the boxes of `a` and `b` overlap (grown by `tol`).
fn boxes_meet(a: &amg_brep::Shape, b: &amg_brep::Shape, tol: f64) -> bool {
    let (x, y) = (a.bbox(), b.bbox());
    (0..3).all(|i| {
        let (xmin, xmax) = (x.min.to_array()[i], x.max.to_array()[i]);
        let (ymin, ymax) = (y.min.to_array()[i], y.max.to_array()[i]);
        xmin <= ymax + tol && ymin <= xmax + tol
    })
}

/// Whether the tool touches or overlaps `p` (box test, then the exact distance).
fn touches(
    k: &dyn BrepKernel,
    p: &amg_brep::Shape,
    tool: &amg_brep::Shape,
    tol: f64,
) -> KResult<bool> {
    if !boxes_meet(p, tool, tol) {
        return Ok(false);
    }
    Ok(k.distance(p, tool)?.distance <= tol)
}

/// Whether operand `operand` has any output in `h` (it contributed to the result).
fn contributes(h: &ShapeHistory, operand: u16) -> bool {
    h.entries.iter().any(|(InputSub { operand: o, .. }, r)| {
        *o == operand
            && match r {
                Relation::Modified(v) | Relation::Generated(v) => !v.is_empty(),
                Relation::Kept(_) => true,
                Relation::Deleted => false,
            }
    })
}

/// Whether a boolean left operand 0 alone: every face of it `Kept` and nothing from operand
/// 1 in the result.
fn untouched(h: &ShapeHistory) -> bool {
    let faces_kept = h
        .entries
        .iter()
        .filter(|(i, _)| i.operand == 0 && i.sub.kind == SubKind::Face)
        .all(|(_, r)| matches!(r, Relation::Kept(_)));
    faces_kept && !contributes(h, 1)
}

fn solid_geom(s: amg_brep::Shape) -> BodyGeom {
    BodyGeom::Solid(amg_doc::stub::Shape::from_brep(s))
}

/// The per-solid results of `r` (one entry for a lone solid, none for an empty result).
fn pieces(r: OpResult) -> KResult<Vec<OpResult>> {
    match r.shape.kind() {
        amg_brep::ShapeKind::Solid => Ok(vec![r]),
        _ => r.per_solid(),
    }
}

/// An identity history over `shape` (operand 0 = the shape itself): every sub-shape `Kept`.
fn identity_result(shape: &amg_brep::Shape) -> OpResult {
    let counts = [
        shape.count(SubKind::Face),
        shape.count(SubKind::Edge),
        shape.count(SubKind::Vertex),
    ];
    let mut entries = Vec::new();
    for (kind, n) in [
        (SubKind::Face, counts[0]),
        (SubKind::Edge, counts[1]),
        (SubKind::Vertex, counts[2]),
    ] {
        for index in 0..n {
            let sub = SubShape { kind, index };
            entries.push((InputSub { operand: 0, sub }, Relation::Kept(sub)));
        }
    }
    OpResult {
        shape: shape.clone(),
        history: ShapeHistory {
            entries,
            roles: Vec::new(),
            tags: Vec::new(),
            operand_counts: vec![counts],
        },
        warnings: Vec::new(),
    }
}

/// Applies the tool's body operation (§3, note §2.3) and appends the body changes to `out`.
///
/// - `NewBody`: each tool solid becomes `BodyId { self, k }`.
/// - `Join`: one fuse of the touched participants and the tool (simplified); each result
///   solid keeps the id of its first contributing participant, the other contributors are
///   `Consumed { into }`, solids made of tool only are new bodies. Nothing touched → new
///   bodies and a warning.
/// - `Cut`: each touched participant is cut separately (simplified); piece 0 keeps its id,
///   further pieces are `CreatedFrom { BodyId { self, k } }`; a vanished body is `Deleted`
///   with a warning. Nothing touched → [`SolidError::NothingToCut`].
/// - `Intersect`: each participant becomes its common part with the tool; an empty part
///   deletes it (warning). All empty → [`SolidError::NothingToIntersect`].
///
/// # Errors
/// As above; participant errors ([`SolidError::WrongBodyKind`],
/// [`SolidError::OtherComponent`]); kernel failures through [`kernel_failure`].
pub fn apply_body_op(
    ctx: &RecomputeCtx<'_>,
    tool: ToolBody,
    out: &mut FeatureOutput,
) -> Result<(), FeatureError> {
    let k = op_kernel(ctx);
    let tol = ctx.tolerance().linear;
    if tool.target.op == BodyOp::NewBody {
        return new_bodies(ctx, k, tool, 0, out);
    }
    let parts = participants(ctx, &tool.target, out)?;
    ctx.check_cancelled()?;
    match tool.target.op {
        BodyOp::NewBody => unreachable!("handled above"),
        BodyOp::Join => join(ctx, k, tol, tool, &parts, out),
        BodyOp::Cut => cut(ctx, k, tol, tool, &parts, out),
        BodyOp::Intersect => intersect(ctx, k, tol, tool, &parts, out),
    }
}

/// Every tool solid as a new body, from slot `first` on.
fn new_bodies(
    ctx: &RecomputeCtx<'_>,
    k: &dyn BrepKernel,
    tool: ToolBody,
    first: u32,
    out: &mut FeatureOutput,
) -> Result<(), FeatureError> {
    if tool.shape.kind() == amg_brep::ShapeKind::Solid {
        out.bodies.push(BodyChange {
            id: ctx.new_body_id(first),
            change: BodyChangeKind::Created {
                geom: solid_geom(tool.shape),
                naming: Some(tool.naming),
            },
        });
        return Ok(());
    }
    let solids = identity_result(&tool.shape).per_solid().map_err(kerr)?;
    if solids.is_empty() {
        return Err(SolidError::Invalid {
            field: "profiles",
            msg: "the tool has no solid".into(),
        }
        .into());
    }
    for (slot, r) in (first..).zip(solids) {
        let mut b = ctx.naming_builder();
        b.from_history(&[(0, OperandNaming::Named(&tool.naming))], &r.history);
        let naming = finish_naming(b, k, &r.shape, out)?;
        out.bodies.push(BodyChange {
            id: ctx.new_body_id(slot),
            change: BodyChangeKind::Created {
                geom: solid_geom(r.shape),
                naming: Some(naming),
            },
        });
    }
    Ok(())
}

fn join(
    ctx: &RecomputeCtx<'_>,
    k: &dyn BrepKernel,
    tol: f64,
    tool: ToolBody,
    parts: &[Participant<'_>],
    out: &mut FeatureOutput,
) -> Result<(), FeatureError> {
    let mut touched: Vec<&Participant<'_>> = Vec::new();
    for p in parts {
        if touches(k, p.shape, &tool.shape, tol).map_err(kerr)? {
            touched.push(p);
        }
    }
    if touched.is_empty() {
        out.warnings
            .push(warning("Nothing to join; created a new body.".to_owned()));
        return new_bodies(ctx, k, tool, 0, out);
    }
    let objects: Vec<amg_brep::Shape> = touched.iter().map(|p| p.shape.clone()).collect();
    let fused = with_job(ctx, |job| {
        k.boolean(
            BoolOp::Fuse,
            &objects,
            std::slice::from_ref(&tool.shape),
            &BoolOpts::default(),
            job,
        )
    });
    let body0 = (touched.len() == 1).then(|| touched[0].id);
    let r = fused
        .and_then(|r| simplify(k, r))
        .map_err(|e| kernel_failure(ctx, body0, e, out))?;
    let tool_operand = u16::try_from(touched.len()).unwrap_or(u16::MAX);
    let mut operands: Vec<(u16, OperandNaming<'_>)> = touched
        .iter()
        .enumerate()
        .map(|(i, p)| {
            (
                u16::try_from(i).unwrap_or(u16::MAX),
                OperandNaming::Named(p.naming),
            )
        })
        .collect();
    operands.push((tool_operand, OperandNaming::Named(&tool.naming)));
    let mut next_slot = 0u32;
    for piece in pieces(r).map_err(kerr)? {
        let contributors: Vec<BodyId> = touched
            .iter()
            .enumerate()
            .filter(|(i, _)| contributes(&piece.history, u16::try_from(*i).unwrap_or(u16::MAX)))
            .map(|(_, p)| p.id)
            .collect();
        let mut b = ctx.naming_builder();
        b.from_history(&operands, &piece.history);
        let naming = finish_naming(b, k, &piece.shape, out)?;
        match contributors.split_first() {
            Some((&keep, rest)) => {
                out.bodies.push(BodyChange {
                    id: keep,
                    change: BodyChangeKind::Modified {
                        geom: solid_geom(piece.shape),
                        naming: Some(naming),
                    },
                });
                for &other in rest {
                    out.bodies.push(BodyChange {
                        id: other,
                        change: BodyChangeKind::Consumed { into: keep },
                    });
                }
            }
            None => {
                out.bodies.push(BodyChange {
                    id: ctx.new_body_id(next_slot),
                    change: BodyChangeKind::Created {
                        geom: solid_geom(piece.shape),
                        naming: Some(naming),
                    },
                });
                next_slot += 1;
            }
        }
    }
    Ok(())
}

/// One boolean of participant `p` with the tool, simplified, highlight on failure.
fn bool_one(
    ctx: &RecomputeCtx<'_>,
    k: &dyn BrepKernel,
    op: BoolOp,
    p: &Participant<'_>,
    tool: &amg_brep::Shape,
    out: &mut FeatureOutput,
) -> Result<OpResult, FeatureError> {
    with_job(ctx, |job| {
        k.boolean(
            op,
            std::slice::from_ref(p.shape),
            std::slice::from_ref(tool),
            &BoolOpts::default(),
            job,
        )
    })
    .and_then(|r| simplify(k, r))
    .map_err(|e| kernel_failure(ctx, Some(p.id), e, out))
}

/// Body changes for the pieces of participant `p` after a cut or intersect.
fn emit_pieces(
    ctx: &RecomputeCtx<'_>,
    k: &dyn BrepKernel,
    p: &Participant<'_>,
    tool: &ToolBody,
    rs: Vec<OpResult>,
    next_slot: &mut u32,
    out: &mut FeatureOutput,
) -> Result<(), FeatureError> {
    for (i, r) in rs.into_iter().enumerate() {
        let mut b = ctx.naming_builder();
        b.from_history(
            &[
                (0, OperandNaming::Named(p.naming)),
                (1, OperandNaming::Named(&tool.naming)),
            ],
            &r.history,
        );
        let naming = finish_naming(b, k, &r.shape, out)?;
        let change = if i == 0 {
            BodyChange {
                id: p.id,
                change: BodyChangeKind::Modified {
                    geom: solid_geom(r.shape),
                    naming: Some(naming),
                },
            }
        } else {
            let id = ctx.new_body_id(*next_slot);
            *next_slot += 1;
            BodyChange {
                id,
                change: BodyChangeKind::CreatedFrom {
                    geom: solid_geom(r.shape),
                    naming: Some(naming),
                    derived_from: p.id,
                },
            }
        };
        out.bodies.push(change);
    }
    Ok(())
}

fn deleted(p: &Participant<'_>, why: &str, out: &mut FeatureOutput) {
    out.bodies.push(BodyChange {
        id: p.id,
        change: BodyChangeKind::Deleted,
    });
    out.warnings.push(Diagnostic {
        message: format!("Body {:?} {why}; it was removed.", p.id),
        cause: None,
        entities: vec![EntityRef::Body(p.id)],
    });
}

fn cut(
    ctx: &RecomputeCtx<'_>,
    k: &dyn BrepKernel,
    tol: f64,
    tool: ToolBody,
    parts: &[Participant<'_>],
    out: &mut FeatureOutput,
) -> Result<(), FeatureError> {
    let mut any = false;
    // Split-off pieces are numbered from 1, so the first split body's piece k is slot k.
    let mut next_slot = 1u32;
    for p in parts {
        if !boxes_meet(p.shape, &tool.shape, tol) {
            continue;
        }
        ctx.check_cancelled()?;
        let r = bool_one(ctx, k, BoolOp::Cut, p, &tool.shape, out)?;
        if untouched(&r.history) {
            continue;
        }
        any = true;
        let rs = pieces(r).map_err(kerr)?;
        if rs.is_empty() {
            deleted(p, "was cut away completely", out);
            continue;
        }
        emit_pieces(ctx, k, p, &tool, rs, &mut next_slot, out)?;
    }
    if any {
        Ok(())
    } else {
        Err(SolidError::NothingToCut.into())
    }
}

fn intersect(
    ctx: &RecomputeCtx<'_>,
    k: &dyn BrepKernel,
    tol: f64,
    tool: ToolBody,
    parts: &[Participant<'_>],
    out: &mut FeatureOutput,
) -> Result<(), FeatureError> {
    let mut any = false;
    let mut next_slot = 1u32;
    for p in parts {
        let rs = if boxes_meet(p.shape, &tool.shape, tol) {
            ctx.check_cancelled()?;
            let r = bool_one(ctx, k, BoolOp::Common, p, &tool.shape, out)?;
            pieces(r).map_err(kerr)?
        } else {
            Vec::new()
        };
        if rs.is_empty() {
            deleted(p, "has no common part with the tool", out);
            continue;
        }
        any = true;
        emit_pieces(ctx, k, p, &tool, rs, &mut next_slot, out)?;
    }
    if any {
        Ok(())
    } else {
        Err(SolidError::NothingToIntersect.into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use amg_doc::FeatureId;

    fn sub(kind: SubKind, index: u32) -> SubShape {
        SubShape { kind, index }
    }

    fn body(slot: u32) -> BodyId {
        BodyId {
            origin: FeatureId(1),
            slot,
        }
    }

    #[test]
    fn new_body_targets_have_no_body_inputs() {
        let t = OpTarget {
            op: BodyOp::NewBody,
            participants: vec![body(0)],
        };
        assert_eq!(t.inputs().count(), 0);
        let t = OpTarget {
            op: BodyOp::Cut,
            participants: vec![body(0), body(1)],
        };
        assert_eq!(
            t.inputs().collect::<Vec<_>>(),
            vec![Dependency::Body(body(0)), Dependency::Body(body(1))]
        );
    }

    #[test]
    fn untouched_needs_all_faces_kept_and_no_tool_output() {
        let f0 = sub(SubKind::Face, 0);
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
        assert!(untouched(&h));
        h.entries.push((
            InputSub {
                operand: 1,
                sub: f0,
            },
            Relation::Modified(std::iter::once(sub(SubKind::Face, 1)).collect()),
        ));
        assert!(!untouched(&h));
        assert!(contributes(&h, 1));
        assert!(!contributes(&h, 2));
    }

    #[test]
    fn op_target_serde_defaults_participants() {
        let t: OpTarget = serde_json::from_str(r#"{"op":"Join"}"#).unwrap();
        assert_eq!(t.op, BodyOp::Join);
        assert!(t.participants.is_empty());
    }
}
