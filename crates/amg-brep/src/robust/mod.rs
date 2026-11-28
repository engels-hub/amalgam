//! The robustness framework (S20 KH-17, design note `kernel-robustness.md` §2).
//!
//! Every strategy result passes the **validation gate** ([`validate`]); strategies run in a
//! **fixed order with fixed parameters** ([`chain::Chain`]), and a per-attempt deadline that
//! expires **aborts the chain** with [`FailureClass::Timeout`] instead of falling through, so
//! the machine's speed never decides which geometry wins. Healing ([`heal`]) runs on copies and
//! composes its history with the op's, so persistent naming still resolves.
//!
//! - [`robust_boolean`] / [`robust_split`]: exact, then the fuzzy ladder (with gluing for
//!   coincident faces), then healed operands; OCCT's non-destructive mode is always on.
//! - [`HardenedKernel`]: the opt-in [`BrepKernel`] decorator that routes `boolean` and `split`
//!   through them and reports non-direct acceptances as [`crate::KernelWarning::Strategy`].
//! - [`RetryPolicy`] presets: [`RetryPolicy::interactive`], [`RetryPolicy::recompute`] (the
//!   default), [`RetryPolicy::exhaustive`], [`RetryPolicy::exact_only`].
//! - [`POLICY_VERSION`] is part of [`crate::kernel_version`]: changing a default chain or ladder
//!   bumps it, which invalidates S01's caches.

use std::fmt;
use std::time::Duration;

use amg_core::{Aabb3, Diagnostic, Point2, Point3, Severity, Tolerance};
use amg_jobs::JobCtx;

use crate::capture::{self, FailureClass, FailureInfo, FailureRecorder};
use crate::check::CheckOpts;
use crate::error::KernelError;
use crate::history::OpResult;
use crate::kernel::BrepKernel;
use crate::query::Classification;
use crate::shape::{Shape, SubKind, SubShape};

mod boolean;
pub mod chain;
pub mod corpus;
mod hardened;
mod heal;

pub use boolean::{robust_boolean, robust_split};
pub use hardened::HardenedKernel;
pub use heal::{compose_operands, heal, identity_history};

/// Version of the robustness policy, the third part of [`crate::kernel_version`]. Bumped by any
/// change of a default chain, ladder or gate that can change results.
///
/// - 0: no robustness layer (S02 BREP-12).
/// - 1: KH-17 (exact → fuzzy ladder `1e-5/1e-4/1e-3 mm` with glue → healed operands).
/// - 2: KH-17 fix (the boolean gate refuses results whose solids overlap,
///   [`Validation::disjoint_solids`]).
pub const POLICY_VERSION: u32 = 2;

/// The fuzzy value above which an accepted rung adds a health warning, mm (design §3.4).
pub const FUZZY_WARN_ABOVE: f64 = 1e-5;

/// How hard an op is retried (design note §2.6).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RetryPolicy {
    /// Fuzzy values tried after the exact attempt, mm, ascending. Rungs above
    /// `min(tol_cap, 1e-3 × smallest input edge)` are skipped.
    pub fuzzy_ladder: Vec<f64>,
    /// The largest tolerance any strategy may introduce, mm.
    pub tol_cap: f64,
    /// The strongest healing level tried after the ladder.
    pub heal: HealLevel,
    /// Attempts per op, the direct one included.
    pub max_attempts: u8,
    /// Wall time per attempt. Expiry aborts the chain with `Timeout`; it never falls through to
    /// a later strategy.
    pub attempt_deadline: Duration,
    /// The validation gate's settings (op-specific fields are set by each op).
    pub validation: Validation,
}

impl RetryPolicy {
    /// Dialog preview: exact + `1e-5`, light healing, 4 attempts, 2 s per attempt.
    pub fn interactive() -> Self {
        RetryPolicy {
            fuzzy_ladder: vec![1e-5],
            tol_cap: 1e-3,
            heal: HealLevel::Light,
            max_attempts: 4,
            attempt_deadline: Duration::from_secs(2),
            validation: Validation::with_cap(1e-3),
        }
    }

    /// Timeline recompute (the default): the full ladder, unify healing, 12 attempts, 20 s.
    pub fn recompute() -> Self {
        RetryPolicy {
            fuzzy_ladder: vec![1e-5, 1e-4, 1e-3],
            tol_cap: 1e-3,
            heal: HealLevel::Unify,
            max_attempts: 12,
            attempt_deadline: Duration::from_secs(20),
            validation: Validation::with_cap(1e-3),
        }
    }

    /// Harness, replay and "try harder": the full ladder, full healing, 32 attempts, 60 s.
    pub fn exhaustive() -> Self {
        RetryPolicy {
            fuzzy_ladder: vec![1e-5, 1e-4, 1e-3],
            tol_cap: 1e-3,
            heal: HealLevel::Full,
            max_attempts: 32,
            attempt_deadline: Duration::from_secs(60),
            validation: Validation::with_cap(1e-3),
        }
    }

    /// Raw OCCT for scripts: the direct attempt only, no healing, 60 s.
    pub fn exact_only() -> Self {
        RetryPolicy {
            fuzzy_ladder: Vec::new(),
            tol_cap: 1e-3,
            heal: HealLevel::None,
            max_attempts: 1,
            attempt_deadline: Duration::from_secs(60),
            validation: Validation::with_cap(1e-3),
        }
    }
}

impl Default for RetryPolicy {
    fn default() -> Self {
        Self::recompute()
    }
}

/// Healing levels, weakest first (design note §2.5).
#[derive(
    Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize,
)]
pub enum HealLevel {
    /// No healing.
    None,
    /// `ShapeFix_Shape` + `ShapeFix_Wireframe` at the linear tolerance, capped at `tol_cap`.
    Light,
    /// `Light` + `ShapeUpgrade_UnifySameDomain`.
    Unify,
    /// `Unify` + re-sewing at `tol_cap` (+ solid from shell).
    Full,
}

impl HealLevel {
    /// The levels from `Light` up to `self`, in order.
    pub fn ladder(self) -> impl Iterator<Item = HealLevel> {
        [HealLevel::Light, HealLevel::Unify, HealLevel::Full]
            .into_iter()
            .filter(move |l| *l <= self)
    }
}

/// The validation gate's settings (design note §2.2).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Validation {
    /// `BRepCheck_Analyzer` must pass.
    pub brepcheck: bool,
    /// Every face must belong to a solid, every solid must be closed (BRepCheck's shell check;
    /// a free-edge scan when `brepcheck` is off) and enclose a positive volume. An empty result
    /// passes (a cut that removes everything).
    pub require_solid: bool,
    /// The largest sub-shape tolerance allowed, mm.
    pub max_tolerance: f64,
    /// Bounds on the total solid volume, mm³.
    pub volume_bounds: Option<(f64, f64)>,
    /// Run `BOPAlgo_ArgumentAnalyzer`'s self-interference test too (slow; lofts).
    pub self_interference: bool,
    /// The result's solids must not overlap, as the pieces of a boolean or a split never do:
    /// points sampled just inside each solid (near its faces) must not be inside another one
    /// whose box meets it. Catches a fuse that silently returns its operands unmerged, which
    /// the volume bounds alone accept (KH-17 fix).
    #[serde(default)]
    pub disjoint_solids: bool,
}

impl Validation {
    /// BRepCheck on, tolerance cap `cap`, nothing op-specific.
    pub fn with_cap(cap: f64) -> Self {
        Validation {
            brepcheck: true,
            require_solid: false,
            max_tolerance: cap,
            volume_bounds: None,
            self_interference: false,
            disjoint_solids: false,
        }
    }
}

impl Default for Validation {
    fn default() -> Self {
        Validation::with_cap(1e-3)
    }
}

/// Which strategy an attempt ran. Numbers in variants index the policy's ladder (0-based) or a
/// strategy's variant list, so they are stable across machines.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[non_exhaustive]
pub enum StrategyId {
    /// The op as asked.
    Direct,
    /// Fuzzy value `fuzzy_ladder[rung]`.
    Fuzzy {
        /// Ladder index.
        rung: u8,
    },
    /// Fuzzy value `fuzzy_ladder[rung]` with `Glue::Shift`.
    Glue {
        /// Ladder index.
        rung: u8,
    },
    /// The op on operands healed at this level.
    Heal(HealLevel),
    /// A `ChFi3d` shape or approximation variant (KH-03).
    FilletShapeVariant(u8),
    /// Unify same-domain faces first (KH-03; also a healing step).
    UnifySameDomain,
    /// Regroup fillet edges by tangent chain (KH-03).
    RegroupChains,
    /// Regroup fillet edges by radius (KH-03).
    RegroupRadius,
    /// Per-edge sequential fillets (KH-03).
    PerEdgeSequential,
    /// Analytic fast-path fillet (KH-04).
    AnalyticFastPath,
    /// Corner patches (KH-19).
    CornerPatch,
    /// Own loft builder (KH-08).
    LoftOwn,
    /// OCCT `ThruSections` (KH-08).
    LoftThruSections,
    /// OCCT `MakePipeShell` (KH-08).
    LoftPipeShell,
    /// OCCT `GeomFill_Gordon` (KH-18).
    LoftOcctGordon,
    /// A strategy injected by a test or a caller of [`chain::Chain`].
    Custom(u8),
}

impl StrategyId {
    /// A short stable name (`KernelWarning::Strategy::name`).
    pub fn name(&self) -> &'static str {
        match self {
            StrategyId::Direct => "direct",
            StrategyId::Fuzzy { .. } => "fuzzy",
            StrategyId::Glue { .. } => "glue",
            StrategyId::Heal(_) => "heal",
            StrategyId::FilletShapeVariant(_) => "fillet_shape_variant",
            StrategyId::UnifySameDomain => "unify_same_domain",
            StrategyId::RegroupChains => "regroup_chains",
            StrategyId::RegroupRadius => "regroup_radius",
            StrategyId::PerEdgeSequential => "per_edge_sequential",
            StrategyId::AnalyticFastPath => "analytic_fast_path",
            StrategyId::CornerPatch => "corner_patch",
            StrategyId::LoftOwn => "loft_own",
            StrategyId::LoftThruSections => "loft_thru_sections",
            StrategyId::LoftPipeShell => "loft_pipe_shell",
            StrategyId::LoftOcctGordon => "loft_occt_gordon",
            StrategyId::Custom(_) => "custom",
        }
    }
}

impl fmt::Display for StrategyId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            StrategyId::Fuzzy { rung } | StrategyId::Glue { rung } => {
                write!(f, "{}[{rung}]", self.name())
            }
            StrategyId::Heal(level) => write!(f, "heal[{level:?}]"),
            StrategyId::FilletShapeVariant(n) | StrategyId::Custom(n) => {
                write!(f, "{}[{n}]", self.name())
            }
            other => f.write_str(other.name()),
        }
    }
}

/// Why the gate refused a result.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Rejection {
    /// BRepCheck problems (distinct names, sorted).
    Check(Vec<String>),
    /// Faces outside a solid, an open solid, or a solid without positive volume.
    NotSolid,
    /// A sub-shape tolerance above the cap, mm.
    ToleranceTooLarge {
        /// The largest tolerance.
        max: f64,
        /// The cap.
        cap: f64,
    },
    /// Total solid volume outside the bounds, mm³.
    VolumeOutOfBounds {
        /// The volume.
        got: f64,
        /// Lower bound.
        lo: f64,
        /// Upper bound.
        hi: f64,
    },
    /// Self-interference findings.
    SelfInterference(usize),
    /// Two solids of the result overlap (indices into [`Shape::solids`]).
    OverlappingSolids(usize, usize),
}

impl Rejection {
    /// The failure class of a rejected final result.
    pub fn class(&self) -> FailureClass {
        match self {
            Rejection::Check(_)
            | Rejection::VolumeOutOfBounds { .. }
            | Rejection::OverlappingSolids(..) => FailureClass::InvalidResult,
            Rejection::NotSolid => FailureClass::NotSolid,
            Rejection::ToleranceTooLarge { .. } => FailureClass::ToleranceTooLarge,
            Rejection::SelfInterference(_) => FailureClass::SelfInterference,
        }
    }
}

impl fmt::Display for Rejection {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Rejection::Check(kinds) => write!(f, "check: {}", kinds.join(", ")),
            Rejection::NotSolid => f.write_str("not a closed solid"),
            Rejection::ToleranceTooLarge { max, cap } => {
                write!(f, "tolerance {max:e} mm above the cap {cap:e} mm")
            }
            Rejection::VolumeOutOfBounds { got, lo, hi } => {
                write!(f, "volume {got} mm³ outside [{lo}, {hi}]")
            }
            Rejection::SelfInterference(n) => write!(f, "{n} self-interference finding(s)"),
            Rejection::OverlappingSolids(a, b) => write!(f, "solids {a} and {b} overlap"),
        }
    }
}

/// The outcome of one attempt.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum AttemptOutcome {
    /// Passed the gate; the chain stops here.
    Accepted,
    /// The op failed (or the deadline expired: class `Timeout`).
    Failed(FailureInfo),
    /// The op returned a result the gate refused.
    Rejected(Rejection),
}

/// One attempt of a chain.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Attempt {
    /// The strategy.
    pub strategy: StrategyId,
    /// What happened.
    pub outcome: AttemptOutcome,
    /// Wall time, ms (the only field that differs between runs).
    pub millis: u32,
}

impl From<&Attempt> for capture::Attempt {
    /// The dump form (KH-01's manifest schema 1 keeps its string fields).
    fn from(a: &Attempt) -> capture::Attempt {
        let outcome = match &a.outcome {
            AttemptOutcome::Accepted => "accepted".to_owned(),
            AttemptOutcome::Failed(info) => format!("failed: {:?} {}", info.class, info.message),
            AttemptOutcome::Rejected(r) => format!("rejected: {r}"),
        };
        capture::Attempt {
            strategy: a.strategy.to_string(),
            outcome,
            millis: u64::from(a.millis),
        }
    }
}

/// What a chain did (design note §2.3): part of [`RobustOutput`], [`RobustError`] and dumps.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RobustReport {
    /// Every attempt, in order.
    pub attempts: Vec<Attempt>,
    /// The accepted strategy, if any.
    pub accepted: Option<StrategyId>,
    /// The accepted result's largest tolerance, mm (0 without one).
    pub max_tolerance: f64,
    /// Health warnings: one per non-direct acceptance, plus fuzzy above 1e-5 mm.
    pub warnings: Vec<String>,
}

impl RobustReport {
    /// The report with every `millis` set to 0, for comparisons across runs.
    #[must_use]
    pub fn without_timing(&self) -> RobustReport {
        let mut r = self.clone();
        for a in &mut r.attempts {
            a.millis = 0;
        }
        r
    }

    /// The attempts in KH-01's dump form.
    pub fn dump_attempts(&self) -> Vec<capture::Attempt> {
        self.attempts.iter().map(capture::Attempt::from).collect()
    }
}

/// A robust op's result and how it was obtained.
#[derive(Clone, Debug)]
pub struct RobustOutput {
    /// The accepted result, with its history from the caller's inputs.
    pub result: OpResult,
    /// The chain's report.
    pub report: RobustReport,
}

/// The context of a robust op.
pub struct RobustCtx<'a> {
    /// Progress and cancellation, passed through to the kernel ops.
    pub job: &'a JobCtx,
    /// Where final failures are dumped, if anywhere.
    pub recorder: Option<&'a FailureRecorder>,
    /// The model tolerance (healing precision).
    pub tol: Tolerance,
}

impl fmt::Debug for RobustCtx<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RobustCtx")
            .field("job", self.job)
            .field("recorder", &self.recorder.is_some())
            .field("tol", &self.tol)
            .finish()
    }
}

impl<'a> RobustCtx<'a> {
    /// `job` with no recorder and the default tolerance.
    pub fn new(job: &'a JobCtx) -> Self {
        RobustCtx {
            job,
            recorder: None,
            tol: Tolerance::DEFAULT,
        }
    }
}

/// A robust op that failed with every strategy, timed out or was cancelled.
#[derive(Debug, thiserror::Error)]
#[error("{class:?}: {message}")]
pub struct RobustError {
    /// The class: `Timeout`, `Cancelled`, the direct attempt's class, or the class of the
    /// argument error that stopped the chain.
    pub class: FailureClass,
    /// What failed.
    pub message: String,
    /// The attempts.
    pub report: RobustReport,
    /// The kernel error of the direct attempt, or of the error that stopped the chain (an
    /// argument error, `BackendUnavailable`, `Cancelled`), when there was one; `None` when the
    /// direct result was rejected by the gate. Lets [`HardenedKernel`] return the kernel's own
    /// error.
    pub error: Option<Box<KernelError>>,
}

impl Diagnostic for RobustError {
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
            FailureClass::Timeout => {
                "The operation took too long; simplify the bodies or use \"try harder\"."
            }
            FailureClass::Cancelled => return None,
            FailureClass::InvalidInput => "Check the highlighted input and its values.",
            FailureClass::ToleranceTooLarge => {
                "Faces almost but not exactly touch; move them apart or snap them together."
            }
            FailureClass::NotSolid | FailureClass::InvalidResult => {
                "The result would be broken; adjust the bodies so they overlap clearly."
            }
            _ => "Try moving the bodies so faces do not just touch, or use \"try harder\".",
        };
        Some(hint.to_owned())
    }
}

/// The validation gate (design note §2.2): BRepCheck (with the optional self-interference
/// check), solid closure and volume, the tolerance cap, the volume bounds and the disjoint-solid
/// check, in that order. Returns the shape's largest tolerance.
///
/// # Errors
/// The first [`Rejection`] found.
pub fn validate(k: &dyn BrepKernel, s: &Shape, v: &Validation) -> Result<f64, Rejection> {
    if v.brepcheck || v.self_interference {
        let report = k.check(
            s,
            &CheckOpts {
                self_interference: v.self_interference,
            },
        );
        if !report.valid {
            let kinds = report.problem_kinds();
            let interfering = report
                .problems
                .iter()
                .filter(|(_, p)| p == "BOPAlgo_SelfIntersect")
                .count();
            if interfering > 0 && kinds.len() == 1 {
                return Err(Rejection::SelfInterference(interfering));
            }
            return Err(Rejection::Check(
                kinds.into_iter().map(str::to_owned).collect(),
            ));
        }
    }
    let solids = s.solids();
    if v.require_solid {
        let faces = s.count(SubKind::Face);
        let in_solids: u32 = solids.iter().map(|x| x.count(SubKind::Face)).sum();
        if faces > 0 && (solids.is_empty() || in_solids < faces) {
            return Err(Rejection::NotSolid);
        }
        for solid in &solids {
            // With BRepCheck on, its shell check already finds open shells (and knows internal
            // edges, such as a line-contact edge embedded in a face, which this topology-only
            // scan would count as free); the scan stands in for it when BRepCheck is off.
            let open = !v.brepcheck && has_free_edges(solid);
            if open || solid.volume().map_or(true, |vol| vol <= 0.0) {
                return Err(Rejection::NotSolid);
            }
        }
    }
    let max = s.max_tolerance();
    if max > v.max_tolerance {
        return Err(Rejection::ToleranceTooLarge {
            max,
            cap: v.max_tolerance,
        });
    }
    if let Some((lo, hi)) = v.volume_bounds {
        let got = s.volume().unwrap_or(f64::NAN);
        if got.is_nan() || got < lo || got > hi {
            return Err(Rejection::VolumeOutOfBounds { got, lo, hi });
        }
    }
    if v.disjoint_solids
        && let Some((a, b)) = overlapping_solids(k, &solids)
    {
        return Err(Rejection::OverlappingSolids(a, b));
    }
    Ok(max)
}

/// The face parameters sampled per face by [`overlapping_solids`] (fractions of the face's
/// parameter box).
const SAMPLE_UV: [(f64, f64); 5] = [
    (0.5, 0.5),
    (0.25, 0.25),
    (0.75, 0.25),
    (0.25, 0.75),
    (0.75, 0.75),
];

/// The first pair of solids `(i, j)`, `i < j`, with a point strictly inside both, if any.
/// Only pairs whose boxes overlap are tested; the points are face samples moved inwards, kept
/// only where the classifier puts them inside their own solid, so a hit is always a real
/// overlap (a sampled point can only miss one).
fn overlapping_solids(k: &dyn BrepKernel, solids: &[Shape]) -> Option<(usize, usize)> {
    if solids.len() < 2 {
        return None;
    }
    let boxes: Vec<Aabb3> = solids.iter().map(Shape::bbox).collect();
    let tols: Vec<f64> = solids.iter().map(|s| s.max_tolerance().max(1e-7)).collect();
    for i in 0..solids.len() {
        for j in i + 1..solids.len() {
            let tol = tols[i].max(tols[j]);
            if !boxes_overlap(&boxes[i], &boxes[j], 2.0 * tol) {
                continue;
            }
            let inside =
                |p: Point3, s: &Shape| matches!(k.classify(s, p, tol), Ok(Classification::Inside));
            let hit = |a: usize, b: usize| {
                interior_samples(k, &solids[a], &boxes[b], tol)
                    .into_iter()
                    .any(|p| inside(p, &solids[b]))
            };
            if hit(i, j) || hit(j, i) {
                return Some((i, j));
            }
        }
    }
    None
}

/// Whether two boxes share an interior region deeper than `margin` on every axis.
fn boxes_overlap(a: &Aabb3, b: &Aabb3, margin: f64) -> bool {
    a.min.x + margin < b.max.x
        && b.min.x + margin < a.max.x
        && a.min.y + margin < b.max.y
        && b.min.y + margin < a.max.y
        && a.min.z + margin < b.max.z
        && b.min.z + margin < a.max.z
}

/// Points inside solid `s` (by its own classifier), sampled at [`SAMPLE_UV`] on each face whose
/// box meets `near`, moved inwards along the normal by `max(1e-3 × box diagonal, 100 × tol)`.
fn interior_samples(k: &dyn BrepKernel, s: &Shape, near: &Aabb3, tol: f64) -> Vec<Point3> {
    let depth = (1e-3 * s.bbox().diagonal()).max(100.0 * tol);
    let mut out = Vec::new();
    for f in s.faces() {
        let meets = s
            .sub_shape(SubShape {
                kind: SubKind::Face,
                index: f.0,
            })
            .is_ok_and(|face| boxes_overlap(&face.bbox(), near, -depth));
        if !meets {
            continue;
        }
        let [u0, u1, v0, v1] = s.face_uv_bounds(f);
        if ![u0, u1, v0, v1].iter().all(|x| x.is_finite()) {
            continue;
        }
        for (a, b) in SAMPLE_UV {
            let uv = Point2::new(u0 + a * (u1 - u0), v0 + b * (v1 - v0));
            let Ok(sp) = k.eval_surface(s, f, uv) else {
                continue;
            };
            let p = sp.point - sp.normal.get() * depth;
            if matches!(k.classify(s, p, tol), Ok(Classification::Inside)) {
                out.push(p);
            }
        }
    }
    out
}

/// Whether a solid has an edge with fewer than two faces that is neither a seam nor
/// degenerated (an open shell).
fn has_free_edges(solid: &Shape) -> bool {
    solid.edges().any(|e| {
        let faces = solid.edge_faces(e);
        match faces {
            [] => solid.curve(e).is_some(),
            [f] => solid.curve(e).is_some() && !solid.is_seam(e, *f),
            _ => false,
        }
    })
}

/// Whether a kernel error is an argument or environment error that no other strategy can fix:
/// the chain stops on it.
pub(crate) fn is_final(e: &KernelError) -> bool {
    matches!(
        e,
        KernelError::InvalidInput { .. }
            | KernelError::OpenLoop { .. }
            | KernelError::NotPlanar { .. }
            | KernelError::NonRigid
            | KernelError::BackendUnavailable
            | KernelError::Cancelled
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[allow(clippy::float_cmp, reason = "the presets are exact constants")]
    fn presets_follow_the_design_table() {
        let i = RetryPolicy::interactive();
        assert_eq!(i.fuzzy_ladder, vec![1e-5]);
        assert_eq!((i.heal, i.max_attempts), (HealLevel::Light, 4));
        assert_eq!(i.attempt_deadline, Duration::from_secs(2));
        let r = RetryPolicy::default();
        assert_eq!(r, RetryPolicy::recompute());
        assert_eq!(r.fuzzy_ladder, vec![1e-5, 1e-4, 1e-3]);
        assert_eq!((r.heal, r.max_attempts), (HealLevel::Unify, 12));
        assert_eq!(r.attempt_deadline, Duration::from_secs(20));
        let x = RetryPolicy::exhaustive();
        assert_eq!((x.heal, x.max_attempts), (HealLevel::Full, 32));
        assert_eq!(x.attempt_deadline, Duration::from_secs(60));
        let e = RetryPolicy::exact_only();
        assert!(e.fuzzy_ladder.is_empty());
        assert_eq!((e.heal, e.max_attempts), (HealLevel::None, 1));
        for p in [i, r, x, e] {
            assert_eq!(p.tol_cap, 1e-3);
            assert_eq!(p.validation.max_tolerance, p.tol_cap);
            assert!(p.validation.brepcheck && !p.validation.self_interference);
            let json = serde_json::to_string(&p).unwrap();
            assert_eq!(serde_json::from_str::<RetryPolicy>(&json).unwrap(), p);
        }
    }

    #[test]
    fn heal_ladder_is_ordered() {
        assert_eq!(HealLevel::None.ladder().count(), 0);
        assert_eq!(
            HealLevel::Unify.ladder().collect::<Vec<_>>(),
            vec![HealLevel::Light, HealLevel::Unify]
        );
        assert_eq!(HealLevel::Full.ladder().count(), 3);
    }

    #[test]
    fn strategy_names_are_stable() {
        assert_eq!(StrategyId::Direct.to_string(), "direct");
        assert_eq!(StrategyId::Fuzzy { rung: 2 }.to_string(), "fuzzy[2]");
        assert_eq!(StrategyId::Glue { rung: 1 }.name(), "glue");
        assert_eq!(
            StrategyId::Heal(HealLevel::Unify).to_string(),
            "heal[Unify]"
        );
    }

    #[test]
    fn attempts_convert_to_the_dump_form() {
        let a = Attempt {
            strategy: StrategyId::Fuzzy { rung: 0 },
            outcome: AttemptOutcome::Rejected(Rejection::ToleranceTooLarge {
                max: 2e-3,
                cap: 1e-3,
            }),
            millis: 7,
        };
        let d = capture::Attempt::from(&a);
        assert_eq!(d.strategy, "fuzzy[0]");
        assert!(
            d.outcome.starts_with("rejected: tolerance"),
            "{}",
            d.outcome
        );
        assert_eq!(d.millis, 7);
        let report = RobustReport {
            attempts: vec![a],
            ..RobustReport::default()
        };
        assert_eq!(report.without_timing().attempts[0].millis, 0);
        let json = serde_json::to_string(&report).unwrap();
        assert_eq!(serde_json::from_str::<RobustReport>(&json).unwrap(), report);
    }

    #[test]
    fn errors_have_hints() {
        let e = RobustError {
            class: FailureClass::Timeout,
            message: "x".into(),
            report: RobustReport::default(),
            error: None,
        };
        assert!(e.hint().is_some());
        assert!(e.to_string().starts_with("Timeout"));
    }
}
