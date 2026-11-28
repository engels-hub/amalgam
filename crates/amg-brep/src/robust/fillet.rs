//! The fillet strategy chain and its diagnostics (S20 KH-03, design §4, design note
//! `kernel-robustness.md` §3.1–§3.2).
//!
//! [`fillet`] runs, in this fixed order and through the validation gate (BRepCheck, closed
//! solids, the tolerance cap, volume bounds from the per-edge blend areas):
//!
//! 1. `Direct`: the caller's [`FilletOpts`];
//! 2. `FilletShapeVariant(0..=2)`: `ChFi3d_QuasiAngular`, `ChFi3d_Polynomial`, then the caller's
//!    shape with looser approximation (`TolApp3d = Fleche = 1e-5`), each skipped when it equals
//!    the caller's options;
//! 3. `UnifySameDomain`: same-domain faces merged first, the edges remapped through its history;
//! 4. `Heal(Light)`: `ShapeFix_Shape` first, edges remapped likewise;
//! 5. `RegroupChains`: one tangent chain per call, the longest chain first;
//! 6. `RegroupRadius`: one radius group per call, the largest radius first;
//! 7. `PerEdgeSequential`: one edge per call, the largest radius first.
//!
//! Sequential strategies remap the remaining edges through each step's history and compose the
//! histories, so the result's history is from the caller's shape. A strategy that does not apply
//! (one chain, one radius, nothing to unify) is not run and costs no attempt.
//!
//! When every strategy fails, [`fillet`] **isolates** the failing edges: OCCT's own report
//! first (faulty contours and vertices, `LocalOpFailed::offending`), then each edge alone, then
//! delta debugging (ddmin) over the set, at most `2·n` kernel calls. Each failing edge gets a
//! [`FailureClass`] and, under the `recompute` and `exhaustive` presets, its largest workable
//! radius ([`max_radius`]).

use std::collections::BTreeMap;
use std::time::Duration;

use amg_core::{Diagnostic, Severity};
use amg_jobs::JobCtx;

use super::chain::{Chain, Strategy, record_failure, with_deadline};
use super::heal::heal_capped;
use super::{
    HealLevel, RetryPolicy, RobustCtx, RobustError, RobustOutput, RobustReport, StrategyId,
    Validation, is_final, validate,
};
use crate::capture::{EdgeRecord, FailureClass, InputRef, KernelOp};
use crate::error::{KResult, KernelError};
use crate::history::{InputSub, OpResult, Relation, ShapeHistory};
use crate::kernel::BrepKernel;
use crate::opts::{FilletOpts, FilletShape, FilletSpec, UnifyOpts};
use crate::shape::{EdgeId, Shape, SubKind, SubShape};

/// Bisection steps of [`max_radius`] (relative precision `2^-12` ≈ 2.4e-4).
pub const MAX_RADIUS_ITERATIONS: u32 = 12;
/// The factor above `r̂` that must fail for a [`max_radius`] answer to be exact.
pub const MAX_RADIUS_CHECK: f64 = 1.02;
/// At most this many failing edges get a max radius inside [`fillet`].
pub const MAX_RADIUS_EDGES: usize = 8;
/// The radius fraction of the classification probe when no max radius is computed: an edge
/// that fails at its radius but works alone at `r / 16` is `RadiusTooLarge`.
const PROBE_FRACTION: f64 = 1.0 / 16.0;

/// One edge the fillet fails on.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FailingEdge {
    /// The edge of the caller's shape.
    pub edge: EdgeId,
    /// Why it fails.
    pub class: FailureClass,
    /// The largest constant radius that fillets this edge alone, mm, when computed.
    pub max_radius: Option<f64>,
    /// `max_radius` did not pass its check (`r̂` works and `1.02·r̂` fails).
    pub approximate: bool,
}

/// What a failed fillet found: the failing edges and the chain's attempts (SOL-20 highlights
/// the edges and shows the max radius).
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FilletDiagnosis {
    /// The failing edges, in edge-index order; never empty for a failure with edges.
    pub failing: Vec<FailingEdge>,
    /// The chain's attempts.
    pub report: RobustReport,
}

/// A fillet that failed with every strategy.
#[derive(Debug, thiserror::Error)]
#[error("fillet failed on {} edge(s)", .diagnosis.failing.len())]
pub struct FilletFailure {
    /// The failing edges and the attempts.
    pub diagnosis: FilletDiagnosis,
    /// The overall class: the first failing edge's class, or the class of the error that
    /// stopped the chain (`InvalidInput`, `Timeout`, `Cancelled`, ...).
    pub class: FailureClass,
    /// What failed, with the failing edges and their max radii.
    pub message: String,
    /// The kernel error of the direct attempt or of the error that stopped the chain, if any.
    pub error: Option<Box<KernelError>>,
}

impl Diagnostic for FilletFailure {
    fn severity(&self) -> Severity {
        if self.class == FailureClass::Cancelled {
            Severity::Info
        } else {
            Severity::Error
        }
    }

    fn is_cancelled(&self) -> bool {
        self.class == FailureClass::Cancelled
    }

    fn hint(&self) -> Option<String> {
        let hint = match self.class {
            FailureClass::Cancelled => return None,
            FailureClass::RadiusTooLarge => match self.smallest_max_radius() {
                Some(r) => {
                    format!("Reduce the radius to at most {r:.3} mm on the highlighted edges.")
                }
                None => "Reduce the radius on the highlighted edges.".to_owned(),
            },
            FailureClass::VertexBlend => {
                "The corner where the highlighted edges meet cannot be blended; fillet them in \
                 separate features or change one radius."
                    .to_owned()
            }
            FailureClass::InvalidInput => {
                "Check the highlighted edges and their values.".to_owned()
            }
            FailureClass::Timeout => {
                "The fillet took too long; select fewer edges or use \"try harder\".".to_owned()
            }
            _ => "Reduce the radius, or leave out the highlighted edges.".to_owned(),
        };
        Some(hint)
    }
}

impl FilletFailure {
    /// The smallest computed max radius among the failing edges.
    pub fn smallest_max_radius(&self) -> Option<f64> {
        self.diagnosis
            .failing
            .iter()
            .filter_map(|f| f.max_radius)
            .reduce(f64::min)
    }

    /// The failure as a kernel error (what [`super::HardenedKernel`] returns): the kernel's own
    /// error for argument errors and `Cancelled`, otherwise `LocalOpFailed { op: "fillet" }` with
    /// the failing edges in `offending` and a message that starts with the class
    /// (`"RadiusTooLarge: …"`, read back by [`FailureClass::of`]).
    pub fn to_kernel_error(&self) -> KernelError {
        if self.class == FailureClass::Cancelled {
            return KernelError::Cancelled;
        }
        if let Some(e) = self.error.as_deref()
            && is_final(e)
        {
            return e.clone();
        }
        KernelError::LocalOpFailed {
            op: "fillet",
            message: format!("{:?}: {}", self.class, self.message),
            offending: self
                .diagnosis
                .failing
                .iter()
                .map(|f| edge_sub(f.edge))
                .collect(),
            partial: None,
        }
    }
}

fn edge_sub(e: EdgeId) -> SubShape {
    SubShape {
        kind: SubKind::Edge,
        index: e.0,
    }
}

/// How deep [`fillet`] diagnoses a failure, from the policy.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Depth {
    /// OCCT's report only (`exact_only`: one attempt).
    Report,
    /// The report, per-edge probes and ddmin, one classification probe per edge
    /// (`interactive`).
    Probe,
    /// `Probe` plus [`max_radius`] for up to [`MAX_RADIUS_EDGES`] edges (`recompute`,
    /// `exhaustive`).
    Full,
}

impl Depth {
    fn of(p: &RetryPolicy) -> Depth {
        match p.max_attempts {
            0 | 1 => Depth::Report,
            2..=11 => Depth::Probe,
            _ => Depth::Full,
        }
    }
}

/// Fillets `edges` of `s` through the strategy chain (module docs). The accepted result's
/// history is from `s` (operand 0); every non-direct acceptance adds a report warning.
///
/// Failures are dumped through `cx.recorder` (op `fillet`, the chain's attempts). Under the
/// `interactive` and `exact_only` presets the failing edges get no max radius (SOL-20 runs
/// [`max_radius`] as a background job); `exact_only` names the edges from OCCT's report only.
///
/// # Errors
/// [`FilletFailure`] with the failing edges (never empty when `edges` is not) and the attempts.
/// Argument errors (`InvalidInput`), `BackendUnavailable` and cancellation stop the chain at
/// once; a deadline expiry aborts it with `Timeout`.
#[allow(
    clippy::result_large_err,
    reason = "the S20 Interfaces signature; a failure carries its diagnosis by value"
)]
pub fn fillet(
    k: &dyn BrepKernel,
    s: &Shape,
    edges: &[(EdgeId, FilletSpec)],
    base: &FilletOpts,
    p: &RetryPolicy,
    cx: &RobustCtx<'_>,
) -> Result<RobustOutput, FilletFailure> {
    let gate = gate(k, s, edges, p);
    let chain = Chain::new(k, "fillet", p, cx);
    let direct = Strategy::new(
        StrategyId::Direct,
        gate.clone(),
        "the direct fillet",
        move |ctx| k.fillet(s, edges, base, ctx),
    );
    let rest = std::iter::once_with(|| later_strategies(k, s, edges, base, &gate, cx)).flatten();
    match chain.run(std::iter::once(direct).chain(rest)) {
        Ok(out) => Ok(out),
        Err(e) => {
            record_failure(cx, &dump_op(s, edges, base), &[s], &e);
            Err(isolate(k, s, edges, base, p, cx, e))
        }
    }
}

/// The capture form of the op.
fn dump_op(s: &Shape, edges: &[(EdgeId, FilletSpec)], base: &FilletOpts) -> KernelOp {
    KernelOp::Fillet {
        shape: InputRef(0),
        edges: edges
            .iter()
            .map(|(e, f)| (EdgeRecord::of(s, *e), f.clone()))
            .collect(),
        opts: base.clone(),
    }
}

/// The gate of every attempt: BRepCheck, closed solids when `s` has solids, the cap (or the
/// input's own tolerance when larger), and the volume bounds `V ± (1.5·Σ L·A_max + slack)` when
/// every edge's blend area could be bounded.
fn gate(
    k: &dyn BrepKernel,
    s: &Shape,
    edges: &[(EdgeId, FilletSpec)],
    p: &RetryPolicy,
) -> Validation {
    let mut v = p.validation.clone();
    v.max_tolerance = v.max_tolerance.max(s.max_tolerance());
    v.require_solid = !s.solids().is_empty();
    // Out-of-range edges are the kernel's to reject (`InvalidInput`), not the gate's to index.
    let in_range = edges.iter().all(|(e, _)| e.0 < s.count(SubKind::Edge));
    if v.require_solid
        && in_range
        && let Ok(volume) = s.volume()
        && let Some(change) = volume_change_bound(k, s, &propagated(k, s, edges))
    {
        let slack = 1.5 * change + 1e-6 * volume.abs() + 1e-9;
        v.volume_bounds = Some((volume - slack, volume + slack));
    }
    v
}

/// The angle within which [`propagated`] follows tangent edges, rad: looser than OCCT's own
/// propagation, so the volume bound covers every edge OCCT blends.
const PROPAGATION_ANGLE: f64 = 2e-2;

/// `edges` plus every edge of `s` that `MakeFillet` blends with them because it continues one
/// tangentially (G1 within [`PROPAGATION_ANGLE`] at a shared vertex), with the spec of the edge
/// it continues.
fn propagated(
    k: &dyn BrepKernel,
    s: &Shape,
    edges: &[(EdgeId, FilletSpec)],
) -> Vec<(EdgeId, FilletSpec)> {
    let mut at_vertex: BTreeMap<u32, Vec<EdgeId>> = BTreeMap::new();
    for e in s.edges() {
        if matches!(*s.edge_faces(e), [a, b] if a != b) {
            for v in s.edge_vertices(e) {
                at_vertex.entry(v.0).or_default().push(e);
            }
        }
    }
    let leaving = |e: EdgeId| {
        k.edge_ends(s, e).ok().map(|ends| {
            let [v0, v1] = s.edge_vertices(e);
            [(v0, ends.t0.get()), (v1, -ends.t1.get())]
        })
    };
    let cos = PROPAGATION_ANGLE.cos();
    let mut out: Vec<(EdgeId, FilletSpec)> = edges.to_vec();
    let mut next = 0;
    while next < out.len() {
        let (e, spec) = out[next].clone();
        next += 1;
        let Some(dirs) = leaving(e) else { continue };
        for (v, d) in dirs {
            for &other in at_vertex.get(&v.0).map_or(&[][..], Vec::as_slice) {
                if other == e || out.iter().any(|(x, _)| *x == other) {
                    continue;
                }
                let Some(od) = leaving(other) else { continue };
                if od.iter().any(|(ov, t)| *ov == v && d.dot(*t) <= -cos) {
                    out.push((other, spec.clone()));
                }
            }
        }
    }
    out
}

/// `Σ L·A_max` over the edges: the largest cross-section a blend of radius `r` can add or remove
/// at an edge whose face normals are `θ` apart is `r²(tan(θ/2) − θ/2)` (the region between the
/// rolling-ball arc and the faces), times the edge length. `None` when an edge's faces or normals
/// cannot be evaluated.
fn volume_change_bound(
    k: &dyn BrepKernel,
    s: &Shape,
    edges: &[(EdgeId, FilletSpec)],
) -> Option<f64> {
    let mut total = 0.0;
    for (e, spec) in edges {
        let theta = normal_angle(k, s, *e)?;
        let len = edge_length(k, s, *e).filter(|l| l.is_finite())?;
        let r = match spec {
            FilletSpec::Constant(r) => *r,
            FilletSpec::Variable(law) => law.iter().map(|(_, r)| *r).fold(0.0, f64::max),
            // chord = 2 r sin(θ/2)
            FilletSpec::Chord(c) => *c / (2.0 * (0.5 * theta).sin().max(1e-3)),
        };
        let half = 0.5 * theta.min(std::f64::consts::PI - 1e-2);
        total += len * r * r * (half.tan() - half).max(0.0);
    }
    Some(total)
}

/// Edge `e`'s length, mm: from the parameter range for lines and circles (cheap), else the
/// kernel's measure.
fn edge_length(k: &dyn BrepKernel, s: &Shape, e: EdgeId) -> Option<f64> {
    use crate::geom::Curve3;
    match s.curve(e) {
        Some((Curve3::Line { .. }, t0, t1)) => Some((t1 - t0).abs()),
        Some((Curve3::Circle { radius, .. }, t0, t1)) => Some(radius * (t1 - t0).abs()),
        _ => k.sub_geom(s, edge_sub(e)).ok().map(|g| g.measure),
    }
}

/// The angle between the two face normals at edge `e`'s start, rad (plane normals directly,
/// other faces by projecting the point).
fn normal_angle(k: &dyn BrepKernel, s: &Shape, e: EdgeId) -> Option<f64> {
    let &[f1, f2] = s.edge_faces(e) else {
        return None;
    };
    let p = k.edge_ends(s, e).ok()?.p0;
    let normal = |f| {
        if let crate::geom::Surface::Plane(plane) = s.surface(f) {
            let n = plane.normal.get();
            return Some(if s.face_reversed(f) { -n } else { n });
        }
        let (uv, _) = k.project_point_on_face(s, f, p).ok()?;
        Some(k.eval_surface(s, f, uv).ok()?.normal.get())
    };
    let (n1, n2) = (normal(f1)?, normal(f2)?);
    Some(n1.dot(n2).clamp(-1.0, 1.0).acos())
}

/// The strategies after `Direct`, built only when it failed.
fn later_strategies<'s>(
    k: &'s dyn BrepKernel,
    s: &'s Shape,
    edges: &'s [(EdgeId, FilletSpec)],
    base: &'s FilletOpts,
    gate: &Validation,
    cx: &'s RobustCtx<'s>,
) -> Vec<Strategy<'s>> {
    let mut out = Vec::new();
    for (n, opts) in shape_variants(base).into_iter().enumerate() {
        let detail = format!("{:?} blends", opts.shape);
        out.push(Strategy::new(
            StrategyId::FilletShapeVariant(u8::try_from(n).unwrap_or(u8::MAX)),
            gate.clone(),
            detail,
            move |ctx| k.fillet(s, edges, &opts, ctx),
        ));
    }
    out.push(Strategy::new(
        StrategyId::UnifySameDomain,
        gate.clone(),
        "same-domain faces unified first",
        move |ctx| {
            let unified = k.unify_same_domain(s, &UnifyOpts::default())?;
            ctx.checkpoint()?;
            if unified.shape.count(SubKind::Face) == s.count(SubKind::Face)
                && unified.shape.count(SubKind::Edge) == s.count(SubKind::Edge)
            {
                return Err(not_applicable("nothing to unify"));
            }
            refillet(k, s, unified, edges, base, ctx)
        },
    ));
    out.push(Strategy::new(
        StrategyId::Heal(HealLevel::Light),
        gate.clone(),
        "the shape healed first",
        move |ctx| {
            let sub = RobustCtx {
                job: ctx,
                recorder: None,
                tol: cx.tol,
            };
            let healed = heal_capped(k, s, HealLevel::Light, 1e-3, &sub)
                .map_err(|e| e.error.map_or_else(|| not_applicable(&e.message), |b| *b))?;
            refillet(k, s, healed.result, edges, base, ctx)
        },
    ));
    // Sequential strategies split the edges along OCCT's contours (a tangent chain is always
    // blended whole), and are left out when their order repeats an earlier one.
    let chains = tangent_chains(k, s, edges);
    if chains.len() < 2 {
        return out;
    }
    let groups = radius_groups(&chains);
    let singles = per_contour(&chains);
    let mut tried: Vec<Vec<Group>> = vec![chains.clone()];
    out.push(Strategy::new(
        StrategyId::RegroupChains,
        gate.clone(),
        format!("{} tangent chains one at a time", chains.len()),
        move |ctx| sequential(k, s, &chains, base, ctx),
    ));
    if groups.len() > 1 && !tried.contains(&groups) {
        tried.push(groups.clone());
        out.push(Strategy::new(
            StrategyId::RegroupRadius,
            gate.clone(),
            format!("{} radius groups one at a time", groups.len()),
            move |ctx| sequential(k, s, &groups, base, ctx),
        ));
    }
    if !tried.contains(&singles) {
        out.push(Strategy::new(
            StrategyId::PerEdgeSequential,
            gate.clone(),
            "one edge (contour) at a time",
            move |ctx| sequential(k, s, &singles, base, ctx),
        ));
    }
    out
}

/// The `ChFi3d` variants after the caller's options, skipping any equal to them.
fn shape_variants(base: &FilletOpts) -> Vec<FilletOpts> {
    let variants = [
        FilletOpts {
            shape: FilletShape::QuasiAngular,
            ..base.clone()
        },
        FilletOpts {
            shape: FilletShape::Polynomial,
            ..base.clone()
        },
        FilletOpts {
            shape: base.shape,
            tol_app_3d: Some(1e-5),
            fleche: Some(1e-5),
        },
    ];
    variants.into_iter().filter(|v| v != base).collect()
}

fn not_applicable(why: &str) -> KernelError {
    KernelError::OpFailed {
        op: "fillet",
        message: format!("strategy does not apply: {why}"),
        offending: Vec::new(),
    }
}

/// The fillet of `edges` (edges of the caller's shape) on `pre.shape`, whose history from the
/// caller's shape is `pre.history`; the result's history is composed.
fn refillet(
    k: &dyn BrepKernel,
    s: &Shape,
    pre: OpResult,
    edges: &[(EdgeId, FilletSpec)],
    base: &FilletOpts,
    ctx: &JobCtx,
) -> KResult<OpResult> {
    let mapped =
        remap(k, s, &pre.shape, &pre.history, edges).map_err(|why| not_applicable(&why))?;
    let next = k.fillet(&pre.shape, &mapped, base, ctx)?;
    Ok(chain_results(pre, next))
}

/// `pre` then `next` as one result from `pre`'s input.
fn chain_results(pre: OpResult, next: OpResult) -> OpResult {
    let mut history = pre.history.compose(&next.history);
    if let Err(e) = history.order_splits(&next.shape) {
        tracing::debug!("robust fillet: split order not applied: {e}");
    }
    let mut warnings = pre.warnings;
    warnings.extend(next.warnings);
    OpResult {
        shape: next.shape,
        history,
        warnings,
    }
}

/// `edges` (of a history's operand 0) mapped to the history's output: each edge to its one
/// modified or kept image, or to every piece of a split (constant and chord specs only).
/// An error when an edge is gone, became something else, or two edges merge with different
/// specs.
fn remap(
    k: &dyn BrepKernel,
    s: &Shape,
    out_shape: &Shape,
    h: &ShapeHistory,
    edges: &[(EdgeId, FilletSpec)],
) -> Result<Vec<(EdgeId, FilletSpec)>, String> {
    let mut out: Vec<(EdgeId, FilletSpec)> = Vec::new();
    for (e, spec) in edges {
        let input = InputSub {
            operand: 0,
            sub: edge_sub(*e),
        };
        let mut images: Vec<SubShape> = Vec::new();
        let relations: Vec<&Relation> = h.of_input(input).collect();
        for r in &relations {
            match r {
                Relation::Kept(o) => images.push(*o),
                Relation::Modified(os) => images.extend(os.iter().copied()),
                Relation::Generated(_) | Relation::Deleted => {}
            }
        }
        images.retain(|o| o.kind == SubKind::Edge);
        if images.is_empty() {
            // OCCT's fillet history drops a neighbour edge that a blend trims (only its
            // `Generated` faces survive): find its pieces on the same line instead.
            images = on_same_line(k, s, *e, out_shape);
        }
        images.sort();
        images.dedup();
        if images.is_empty() || (images.len() > 1 && matches!(spec, FilletSpec::Variable(_))) {
            return Err(format!("edge {} became {relations:?}", e.0));
        }
        for o in images {
            let id = EdgeId(o.index);
            match out.iter().find(|(x, _)| *x == id) {
                Some((_, other)) if other != spec => {
                    return Err(format!(
                        "edges merge into edge {} with different specs",
                        id.0
                    ));
                }
                Some(_) => {}
                None => out.push((id, spec.clone())),
            }
        }
    }
    Ok(out)
}

/// The edges of `out` (between two faces) that lie on straight edge `e` of `s`: lines whose ends
/// are within `1e-6·(1 + L)` mm of `e`'s segment. Empty for a curved `e`.
fn on_same_line(k: &dyn BrepKernel, s: &Shape, e: EdgeId, out: &Shape) -> Vec<SubShape> {
    use crate::geom::GeomClass;
    let Ok(g) = k.sub_geom(s, edge_sub(e)) else {
        return Vec::new();
    };
    let Ok(ends) = k.edge_ends(s, e) else {
        return Vec::new();
    };
    if g.class != GeomClass::Line {
        return Vec::new();
    }
    let (a, dir, len) = (ends.p0, ends.t0.get(), g.measure);
    let tol = 1e-6 * (1.0 + len);
    let on_segment = |p: amg_core::Point3| {
        let t = (p - a).dot(dir);
        let off = (p - a) - dir * t;
        off.length() <= tol && t >= -tol && t <= len + tol
    };
    out.edges()
        .filter(|&x| matches!(*out.edge_faces(x), [f1, f2] if f1 != f2))
        .filter(|&x| {
            k.sub_geom(out, edge_sub(x))
                .is_ok_and(|gx| gx.class == GeomClass::Line && gx.measure > tol)
                && k.edge_ends(out, x)
                    .is_ok_and(|ex| on_segment(ex.p0) && on_segment(ex.p1))
        })
        .map(edge_sub)
        .collect()
}

/// Fillets `groups` (edges of `s`) one group per call, remapping the later groups through each
/// step's history.
fn sequential(
    k: &dyn BrepKernel,
    s: &Shape,
    groups: &[Vec<(EdgeId, FilletSpec)>],
    base: &FilletOpts,
    ctx: &JobCtx,
) -> KResult<OpResult> {
    let mut current = OpResult {
        shape: s.clone(),
        history: super::identity_history(s),
        warnings: Vec::new(),
    };
    for group in groups {
        ctx.checkpoint()?;
        current = refillet(k, s, current, group, base, ctx)?;
    }
    Ok(current)
}

/// The radius a spec is ordered by: constant and chord values, a variable law's largest radius.
fn spec_size(spec: &FilletSpec) -> f64 {
    match spec {
        FilletSpec::Constant(r) | FilletSpec::Chord(r) => *r,
        FilletSpec::Variable(law) => law.iter().map(|(_, r)| *r).fold(0.0, f64::max),
    }
}

type Group = Vec<(EdgeId, FilletSpec)>;

/// The largest spec size of a group.
fn group_size(g: &[(EdgeId, FilletSpec)]) -> f64 {
    g.iter().map(|(_, sp)| spec_size(sp)).fold(0.0, f64::max)
}

/// The contours one call at a time, the largest radius first, then the lower first edge.
fn per_contour(contours: &[Group]) -> Vec<Group> {
    let mut v = contours.to_vec();
    v.sort_by(|a, b| {
        group_size(b)
            .total_cmp(&group_size(a))
            .then(first_edge(a).cmp(&first_edge(b)))
    });
    v
}

/// The contours merged by spec (a contour with mixed specs stays alone), the largest radius
/// first (ties: the lower first edge), edges in contour order within a group.
fn radius_groups(contours: &[Group]) -> Vec<Group> {
    let uniform = |g: &Group| g.iter().all(|(_, sp)| *sp == g[0].1);
    let mut groups: Vec<Group> = Vec::new();
    for c in contours.iter().filter(|c| !c.is_empty()) {
        match groups
            .iter_mut()
            .find(|g| uniform(c) && uniform(g) && g[0].1 == c[0].1)
        {
            Some(g) => g.extend(c.iter().cloned()),
            None => groups.push(c.clone()),
        }
    }
    groups.sort_by(|a, b| {
        group_size(b)
            .total_cmp(&group_size(a))
            .then(first_edge(a).cmp(&first_edge(b)))
    });
    groups
}

fn first_edge(g: &[(EdgeId, FilletSpec)]) -> EdgeId {
    g.iter().map(|(e, _)| *e).min().unwrap_or(EdgeId(u32::MAX))
}

/// The requested edges grouped into the contours `MakeFillet` builds: two requested edges are
/// in one contour when one is reached from the other by tangent propagation ([`propagated`],
/// through edges that were not requested too). The longest contour first (by edge count, then
/// the lower first edge index); edges in input order within a contour.
fn tangent_chains(k: &dyn BrepKernel, s: &Shape, edges: &[(EdgeId, FilletSpec)]) -> Vec<Group> {
    fn root(parent: &mut [usize], mut i: usize) -> usize {
        while parent[i] != i {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        i
    }
    let n = edges.len();
    let mut parent: Vec<usize> = (0..n).collect();
    for i in 0..n {
        let reach = propagated(k, s, &edges[i..=i]);
        for (j, (ej, _)) in edges.iter().enumerate().skip(i + 1) {
            if reach.iter().any(|(x, _)| x == ej) {
                let (ri, rj) = (root(&mut parent, i), root(&mut parent, j));
                parent[ri.max(rj)] = ri.min(rj);
            }
        }
    }
    let mut by_root: BTreeMap<usize, Group> = BTreeMap::new();
    for (i, e) in edges.iter().enumerate() {
        let r = root(&mut parent, i);
        by_root.entry(r).or_default().push(e.clone());
    }
    let mut chains: Vec<Group> = by_root.into_values().collect();
    chains.sort_by(|a, b| {
        b.len()
            .cmp(&a.len())
            .then(first_edge(a).cmp(&first_edge(b)))
    });
    chains
}

// ---- isolation (design note §3.2) -------------------------------------------------------------

/// What one diagnostic fillet of a subset gave.
enum Probe {
    /// A result that passes the gate.
    Works,
    /// An error, or a result the gate refused.
    Fails,
    /// Cancelled or out of time: stop probing.
    Stop,
}

struct Prober<'a> {
    k: &'a dyn BrepKernel,
    s: &'a Shape,
    base: &'a FilletOpts,
    gate: Validation,
    deadline: Duration,
    job: &'a JobCtx,
    calls: usize,
}

impl Prober<'_> {
    fn run(&mut self, subset: &[(EdgeId, FilletSpec)]) -> Probe {
        if self.job.is_cancelled() {
            return Probe::Stop;
        }
        self.calls += 1;
        let (r, late) = with_deadline(self.job, self.deadline, |ctx| {
            self.k.fillet(self.s, subset, self.base, ctx)
        });
        if late || self.job.is_cancelled() {
            return Probe::Stop;
        }
        match r {
            Ok(out) => match validate(self.k, &out.shape, &self.gate) {
                Ok(_) => Probe::Works,
                Err(_) => Probe::Fails,
            },
            Err(KernelError::Cancelled) => Probe::Stop,
            Err(_) => Probe::Fails,
        }
    }

    fn fails(&mut self, subset: &[(EdgeId, FilletSpec)]) -> Option<bool> {
        match self.run(subset) {
            Probe::Works => Some(false),
            Probe::Fails => Some(true),
            Probe::Stop => None,
        }
    }
}

/// Turns the chain's error into a [`FilletFailure`] with the failing edges.
#[allow(
    clippy::too_many_lines,
    reason = "one pass: report, probes, ddmin, classes"
)]
fn isolate(
    k: &dyn BrepKernel,
    s: &Shape,
    edges: &[(EdgeId, FilletSpec)],
    base: &FilletOpts,
    p: &RetryPolicy,
    cx: &RobustCtx<'_>,
    e: RobustError,
) -> FilletFailure {
    let requested: Vec<EdgeId> = edges.iter().map(|(x, _)| *x).collect();
    let spec_of = |x: EdgeId| {
        edges
            .iter()
            .find(|(y, _)| *y == x)
            .map(|(_, sp)| sp.clone())
            .unwrap_or(FilletSpec::Constant(0.0))
    };
    let reported: Vec<EdgeId> = e
        .error
        .as_deref()
        .map(|err| {
            let mut v: Vec<EdgeId> = err
                .offending()
                .iter()
                .filter(|o| o.kind == SubKind::Edge)
                .map(|o| EdgeId(o.index))
                .filter(|x| requested.contains(x))
                .collect();
            v.sort();
            v.dedup();
            v
        })
        .unwrap_or_default();
    let stopped = matches!(
        e.class,
        FailureClass::Cancelled | FailureClass::Timeout | FailureClass::InvalidInput
    ) || e.error.as_deref().is_some_and(is_final);
    let depth = Depth::of(p);
    let mut all = requested.clone();
    all.sort();
    all.dedup();

    // Without probing (or when the chain was stopped), the report or every edge.
    let fallback = |class: FailureClass| -> Vec<FailingEdge> {
        let set = if reported.is_empty() { &all } else { &reported };
        set.iter()
            .map(|&edge| FailingEdge {
                edge,
                class,
                max_radius: None,
                approximate: false,
            })
            .collect()
    };
    if stopped || depth == Depth::Report || edges.is_empty() {
        let failing = fallback(e.class);
        return failure(e, failing);
    }

    let mut probe = Prober {
        k,
        s,
        base,
        gate: Validation {
            volume_bounds: None,
            ..gate(k, s, &[], p)
        },
        deadline: p.attempt_deadline,
        job: cx.job,
        calls: 0,
    };
    let budget = 2 * all.len().max(1);
    let with_specs = |set: &[EdgeId]| -> Vec<(EdgeId, FilletSpec)> {
        set.iter().map(|&x| (x, spec_of(x))).collect()
    };

    // 1. OCCT's report, when it names a proper subset; 2. each edge alone; 3. ddmin.
    let mut failing_set: Vec<EdgeId> = Vec::new();
    let mut together = false;
    if !reported.is_empty() && reported.len() < all.len() {
        failing_set.clone_from(&reported);
    } else if all.len() == 1 {
        failing_set.clone_from(&all);
    } else {
        for &x in &all {
            if probe.calls >= budget {
                break;
            }
            match probe.fails(&with_specs(&[x])) {
                Some(true) => failing_set.push(x),
                Some(false) => {}
                None => break,
            }
        }
        if failing_set.is_empty() {
            together = true;
            failing_set = ddmin(&all, budget, &mut |set: &[EdgeId]| {
                if probe.calls >= budget {
                    return None;
                }
                probe.fails(&with_specs(set))
            });
        }
    }
    if failing_set.is_empty() {
        failing_set.clone_from(&all);
    }

    // Classes and max radii.
    let message_class = e
        .error
        .as_deref()
        .map_or(e.class, |err| class_from_message(err).unwrap_or(e.class));
    let shared_vertex = together && shares_vertex(s, &failing_set);
    let mut failing = Vec::new();
    for (i, &edge) in failing_set.iter().enumerate() {
        let spec = spec_of(edge);
        let requested_r = spec_size(&spec);
        let mut fe = FailingEdge {
            edge,
            class: if shared_vertex {
                FailureClass::VertexBlend
            } else {
                message_class
            },
            max_radius: None,
            approximate: false,
        };
        if !together && requested_r > 0.0 && !cx.job.is_cancelled() {
            if depth == Depth::Full && i < MAX_RADIUS_EDGES {
                let (r, late) = with_deadline(cx.job, p.attempt_deadline, |ctx| {
                    let sub = RobustCtx {
                        job: ctx,
                        recorder: None,
                        tol: cx.tol,
                    };
                    max_radius(k, s, edge, requested_r, &sub)
                });
                match r {
                    Ok((r_hat, approx)) if !late => {
                        fe.max_radius = Some(r_hat);
                        fe.approximate = approx;
                        if r_hat < requested_r * (1.0 - 1e-9) {
                            fe.class = FailureClass::RadiusTooLarge;
                        }
                    }
                    Ok(_) => {}
                    Err(err) => {
                        if let Some(c) = err.error.as_deref().and_then(class_from_message) {
                            fe.class = c;
                        }
                    }
                }
            } else {
                let small = scaled(&spec, PROBE_FRACTION);
                if let Probe::Works = probe.run(&[(edge, small)]) {
                    fe.class = FailureClass::RadiusTooLarge;
                }
            }
        }
        failing.push(fe);
    }
    failing.sort_by_key(|f| f.edge);
    failure(e, failing)
}

/// `spec` with every radius (or the chord) scaled by `f`.
fn scaled(spec: &FilletSpec, f: f64) -> FilletSpec {
    match spec {
        FilletSpec::Constant(r) => FilletSpec::Constant(r * f),
        FilletSpec::Chord(c) => FilletSpec::Chord(c * f),
        FilletSpec::Variable(law) => {
            FilletSpec::Variable(law.iter().map(|(t, r)| (*t, r * f)).collect())
        }
    }
}

fn shares_vertex(s: &Shape, set: &[EdgeId]) -> bool {
    set.iter().enumerate().any(|(i, a)| {
        let va = s.edge_vertices(*a);
        set[i + 1..]
            .iter()
            .any(|b| s.edge_vertices(*b).iter().any(|v| va.contains(v)))
    })
}

/// The fillet class a local-op error message names: a failed corner → `VertexBlend`, a walking
/// failure → `WalkingFailed`; `None` otherwise.
fn class_from_message(e: &KernelError) -> Option<FailureClass> {
    match e {
        KernelError::LocalOpFailed { message, .. } => {
            if message.contains("a corner failed") {
                Some(FailureClass::VertexBlend)
            } else if message.contains("ChFiDS_WalkingFailure") {
                Some(FailureClass::WalkingFailed)
            } else {
                None
            }
        }
        _ => None,
    }
}

fn failure(e: RobustError, failing: Vec<FailingEdge>) -> FilletFailure {
    let class = failing.first().map_or(e.class, |f| f.class);
    let edges: Vec<String> = failing
        .iter()
        .map(|f| match f.max_radius {
            Some(r) => format!(
                "edge {} {:?} (max radius {r:.6} mm{})",
                f.edge.0,
                f.class,
                if f.approximate { ", approximate" } else { "" }
            ),
            None => format!("edge {} {:?}", f.edge.0, f.class),
        })
        .collect();
    let message = if edges.is_empty() {
        e.message.clone()
    } else {
        format!("{}; {}", edges.join(", "), e.message)
    };
    FilletFailure {
        diagnosis: FilletDiagnosis {
            failing,
            report: e.report,
        },
        class,
        message,
        error: e.error,
    }
}

/// Delta debugging (Zeller's ddmin) of a failing `set`: a smaller subset that still fails, 1-
/// minimal when `fails` never runs out (`None` = stop, keep the current set). Subsets of one
/// edge are not probed (the caller already found every single edge working).
fn ddmin(
    set: &[EdgeId],
    budget: usize,
    fails: &mut dyn FnMut(&[EdgeId]) -> Option<bool>,
) -> Vec<EdgeId> {
    let mut c: Vec<EdgeId> = set.to_vec();
    let mut n = 2usize;
    let mut tries = 0usize;
    'outer: while c.len() >= 2 && tries < budget {
        let chunks: Vec<Vec<EdgeId>> = split(&c, n);
        for chunk in &chunks {
            if chunk.len() < 2 {
                continue;
            }
            if tries >= budget {
                break 'outer;
            }
            tries += 1;
            match fails(chunk) {
                Some(true) => {
                    c.clone_from(chunk);
                    n = 2;
                    continue 'outer;
                }
                Some(false) => {}
                None => break 'outer,
            }
        }
        if n > 2 {
            for i in 0..chunks.len() {
                let complement: Vec<EdgeId> = chunks
                    .iter()
                    .enumerate()
                    .filter(|(j, _)| *j != i)
                    .flat_map(|(_, ch)| ch.iter().copied())
                    .collect();
                if complement.len() < 2 {
                    continue;
                }
                if tries >= budget {
                    break 'outer;
                }
                tries += 1;
                match fails(&complement) {
                    Some(true) => {
                        c = complement;
                        n = (n - 1).max(2);
                        continue 'outer;
                    }
                    Some(false) => {}
                    None => break 'outer,
                }
            }
        }
        if n >= c.len() {
            break;
        }
        n = (2 * n).min(c.len());
    }
    c
}

/// `c` split into `n` nearly equal consecutive chunks.
fn split(c: &[EdgeId], n: usize) -> Vec<Vec<EdgeId>> {
    let n = n.clamp(1, c.len().max(1));
    let mut out = Vec::with_capacity(n);
    let mut start = 0;
    for i in 0..n {
        let end = start + (c.len() - start) / (n - i);
        out.push(c[start..end].to_vec());
        start = end;
    }
    out
}

/// The largest constant radius `r̂ ≤ requested` that fillets `edge` of `s` alone (S02
/// `fillet_max_radius`, [`MAX_RADIUS_ITERATIONS`] bisection steps, BRepCheck-valid), and whether
/// it is approximate: exact when a fillet at `r̂` passes the gate (BRepCheck, closed solids, the
/// 1e-3 mm cap) and one at [`MAX_RADIUS_CHECK`]`·r̂` does not. `requested` itself, not
/// approximate, when the edge works alone at `requested`. Cancellation reaches every call;
/// callers bound the time (SOL-20 runs it as a background job).
///
/// # Errors
/// A [`RobustError`] with the kernel's error: `InvalidInput` for a bad edge or radius,
/// `LocalOpFailed { op: "fillet_max_radius" }` (class `NotDone`) when no radius works,
/// `Cancelled`.
pub fn max_radius(
    k: &dyn BrepKernel,
    s: &Shape,
    edge: EdgeId,
    requested: f64,
    cx: &RobustCtx<'_>,
) -> Result<(f64, bool), RobustError> {
    let err = |e: KernelError| RobustError {
        class: FailureClass::of(&e),
        message: e.to_string(),
        report: RobustReport::default(),
        error: Some(Box::new(e)),
    };
    let r_hat = k
        .fillet_max_radius(s, &[edge], requested, MAX_RADIUS_ITERATIONS, cx.job)
        .map_err(err)?;
    if r_hat >= requested {
        return Ok((requested, false));
    }
    let mut gate = Validation::with_cap(1e-3_f64.max(s.max_tolerance()));
    gate.require_solid = !s.solids().is_empty();
    let works = |r: f64| -> Result<bool, RobustError> {
        match k.fillet(
            s,
            &[(edge, FilletSpec::Constant(r))],
            &FilletOpts::default(),
            cx.job,
        ) {
            Ok(out) => Ok(validate(k, &out.shape, &gate).is_ok()),
            Err(KernelError::Cancelled) => Err(err(KernelError::Cancelled)),
            Err(_) => Ok(false),
        }
    };
    let exact = works(r_hat)? && !works(MAX_RADIUS_CHECK * r_hat)?;
    Ok((r_hat, !exact))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn e(i: u32) -> EdgeId {
        EdgeId(i)
    }

    #[test]
    fn ddmin_finds_a_failing_pair() {
        // Fails exactly when edges 3 and 6 are both in the set.
        let set: Vec<EdgeId> = (0..8).map(e).collect();
        let pair = |s: &[EdgeId]| s.contains(&e(3)) && s.contains(&e(6));
        let mut calls = 0;
        let found = ddmin(&set, 64, &mut |s: &[EdgeId]| {
            calls += 1;
            Some(pair(s))
        });
        assert_eq!(found, vec![e(3), e(6)]);
        assert!(calls <= 64, "{calls}");
        // A tight budget is never exceeded and still returns a failing set.
        for budget in [2, 4, 8] {
            let mut calls = 0;
            let found = ddmin(&set, budget, &mut |s: &[EdgeId]| {
                calls += 1;
                Some(pair(s))
            });
            assert!(calls <= budget, "{calls} > {budget}");
            assert!(pair(&found), "{found:?}");
        }
    }

    #[test]
    fn ddmin_stops_when_told_to() {
        let set: Vec<EdgeId> = (0..6).map(e).collect();
        let found = ddmin(&set, 12, &mut |_| None);
        assert_eq!(found, set);
    }

    #[test]
    fn split_covers_the_set() {
        let set: Vec<EdgeId> = (0..7).map(e).collect();
        let parts = split(&set, 3);
        assert_eq!(parts.len(), 3);
        assert_eq!(parts.concat(), set);
    }

    #[test]
    fn groups_are_deterministic() {
        let edges = [
            (e(4), FilletSpec::Constant(1.0)),
            (e(1), FilletSpec::Constant(2.0)),
            (e(2), FilletSpec::Constant(1.0)),
            (e(0), FilletSpec::Chord(2.0)),
        ];
        let contours: Vec<Group> = edges.iter().map(|x| vec![x.clone()]).collect();
        let groups = radius_groups(&contours);
        assert_eq!(groups.len(), 3);
        // The two 2.0 groups first, the lower first edge (the chord's edge 0) leading.
        assert_eq!(groups[0][0].0, e(0));
        assert_eq!(groups[1][0].0, e(1));
        assert_eq!(
            groups[2],
            vec![
                (e(4), FilletSpec::Constant(1.0)),
                (e(2), FilletSpec::Constant(1.0))
            ]
        );
        let singles = per_contour(&contours);
        let order: Vec<EdgeId> = singles.iter().map(|g| g[0].0).collect();
        assert_eq!(order, vec![e(0), e(1), e(2), e(4)]);
    }

    #[test]
    fn variants_skip_the_callers_options() {
        let base = FilletOpts::default();
        let v = shape_variants(&base);
        assert_eq!(v.len(), 3);
        let q = FilletOpts {
            shape: FilletShape::QuasiAngular,
            ..FilletOpts::default()
        };
        assert_eq!(shape_variants(&q).len(), 2);
    }

    #[test]
    fn failures_carry_their_class_into_kernel_errors() {
        let f = FilletFailure {
            diagnosis: FilletDiagnosis {
                failing: vec![FailingEdge {
                    edge: e(5),
                    class: FailureClass::RadiusTooLarge,
                    max_radius: Some(2.5),
                    approximate: false,
                }],
                report: RobustReport::default(),
            },
            class: FailureClass::RadiusTooLarge,
            message: "edge 5".into(),
            error: None,
        };
        let ke = f.to_kernel_error();
        assert_eq!(FailureClass::of(&ke), FailureClass::RadiusTooLarge);
        assert_eq!(ke.offending(), &[edge_sub(e(5))]);
        assert!(f.hint().unwrap().contains("2.500"));
        let json = serde_json::to_string(&f.diagnosis).unwrap();
        assert_eq!(
            serde_json::from_str::<FilletDiagnosis>(&json).unwrap(),
            f.diagnosis
        );
    }

    #[test]
    fn policies_set_the_depth() {
        assert_eq!(Depth::of(&RetryPolicy::exact_only()), Depth::Report);
        assert_eq!(Depth::of(&RetryPolicy::interactive()), Depth::Probe);
        assert_eq!(Depth::of(&RetryPolicy::recompute()), Depth::Full);
        assert_eq!(Depth::of(&RetryPolicy::exhaustive()), Depth::Full);
    }
}
