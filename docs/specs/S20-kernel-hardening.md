# S20 — Kernel hardening (fillet & loft robustness, Rust rewrites)

| | |
|---|---|
| **Status** | Ready |
| **Priority** | P0 (failure capture) / P1 (`amg-nurbs`, fillet + loft hardening, robustness framework) / P3 (Rust rewrites, last phase) |
| **Crates** | `amg-nurbs` (all of it; ADR-013); in `amg-brep` (shared with S02) the modules `amg_brep::capture`, `amg_brep::robust::{fillet, loft, corpus}` and `amg_brep::nurbs_conv`, plus their `amg-occt-sys` shims; `testdata/kernel/`, `tests/golden/kernel/` |
| **Lane** | A |
| **Depends on** | S00 (FND-03 core types, FND-04 jobs, FND-06 testkit/testdata, FND-07 diag); S02 (BREP-02/03/04/05/06/09/10/12/15; requests R1–R17 in §Interfaces); S04 (SOL-01, SOL-05 consume `robust::fillet`; SOL-11 consumes `robust::loft`); S03 (SK-06 profiles feed loft sections) |
| **Parity** | Fillet and loft must be **more reliable than Fusion's** (parity rows "Fillet …" and "Revolve, sweep, loft"). Loft instability was a specific pain point for the owner |
| **Design notes** | [`docs/notes/nurbs-library.md`](../notes/nurbs-library.md) (`amg-nurbs`: build vs reuse, algorithms, accuracy contract); [`docs/notes/kernel-robustness.md`](../notes/kernel-robustness.md) (failure capture, robustness framework, fillet/loft strategy chains, corpora, harness); `docs/notes/kernel-metrics.md` (written by KH-02/KH-05) |

## Summary
OCCT's weakest operations for our use are **fillets** (legacy `ChFi3d`, built on the old
`TopOpeBRep` boolean structures) and **lofts** (`BRepOffsetAPI_ThruSections`, with fragile
section compatibility and no real rails or end conditions). This spec makes both measurably
robust, driven by failure corpora and success-rate metrics, in three stages:

1. **Now (Phases 1–3):**
   - capture every kernel failure and make it replayable;
   - a shared robustness framework (validation gate, tolerance escalation, healing, retry
     policies);
   - smarter strategies around OCCT fillets, and a fast-path fillet for analytic edges on
     the modern booleans;
   - our own loft section preprocessing and surface construction in pure Rust (`amg-nurbs`).
2. **Ongoing:** report or patch isolated bugs upstream in OCCT.
3. **Last (Phase 7, after all other parity work):** a Rust rewrite of the fillet engine, then
   progressive replacement of other OCCT operations where the corpora show it pays off.

## Scope
**In:**
- `amg-nurbs`, pure Rust with no OCCT: curves/surfaces, evaluation, knot ops, analytic
  conversions, fitting, compatibility, skinning with end conditions, Gordon surfaces,
  rotation-minimising frames, continuity checks.
- `amg_brep::capture`: failure dumps, write-ahead mode, replay, promote-to-corpus.
- `amg_brep::robust`:
  - `RetryPolicy`, validation gate, fuzzy-tolerance escalation, healing levels;
  - the fillet strategy chain, diagnostics, analytic fast path and corner patches;
  - the loft pipeline (preprocessing, builder, rails/centerline, fallback chain, diagnostics);
  - the corpus generators and harness.
- `amg_brep::nurbs_conv`: `amg-nurbs` ↔ OCCT `Geom_BSpline*` conversion and faces from
  NURBS surfaces.
- Kernel corpora and the regression baseline; upstream OCCT reports.

**Out:**
- The `BrepKernel` trait, shape handles, raw OCCT ops (`ThruSections`, `MakeFillet`, booleans,
  sewing, ShapeFix, `.brep` I/O) → S02. S20 only *requests* what it needs (§Interfaces).
- The Fillet/Loft features, their dialogs and the preview rendering of S20 diagnostics →
  S04 (SOL-05, SOL-11) on the S11 dialog framework.
- The CLI binary → lane E. S20 provides library entry points plus `amg-brep` examples, and
  requests thin `amg-cli kernel …` subcommands.
- Sketch splines (`amg_sketch::BSpline2`, S03 SK-08) stay in `amg-sketch`, because the
  layering rule doesn't allow `amg-sketch` → `amg-nurbs` (see report/Open question 4).

## Design
Full detail is in the two design notes. This section records the decisions.

### 1. `amg-nurbs` is built, not reused (KH-07, KH-12..KH-15)
On 2026-10-20 the candidates on crates.io were:
- `curvo` 0.3.3, MIT, very active;
- `truck-geometry` 0.5.0, Apache-2.0, last release 2024-09;
- `monstertruck-geometry` 0.4.1, Apache-2.0;
- `brepkit-math` 4.2.2, AGPL-3.0.

None provides approximate section compatibility, end-conditioned skinning or a correct Gordon
surface:
- curvo's loft is an exact knot union.
- monstertruck's Gordon assumes uniform intersection parameters and a degree-1 tensor term.
- curvo uses `anyhow` and nalgebra in its public API, and truck/monstertruck use cgmath. Both
  clash with workspace conventions (ADR-006, `thiserror`).

So `amg-nurbs` follows *The NURBS Book* (Piegl & Tiller) directly on `amg_core` glam f64
types. curvo is a pinned **dev-dependency oracle**, and OCCT is the oracle inside `amg-brep`
tests. Evidence table: nurbs note §2.

Data model and algorithms are in nurbs note §3–§7:
- clamped NURBS with separate weights (a 1:1 copy to OCCT);
- P&T A2.1–A5.10 for evaluation and knot ops;
- §9 for fitting, §10.3 for skinning;
- Piegl & Tiller 2002 for approximate compatibility;
- Gordon after TiGL / Siggel 2019;
- double-reflection rotation-minimising frames.

The accuracy contract is nurbs note §8 and is repeated in the WP Done-when cells below.

### 2. Failure capture (KH-01)
`RecordingKernel<K: BrepKernel>` forwards every call. On `Err`, or on a result rejected by
the validation gate, it writes a dump to `%LOCALAPPDATA%\Amalgam\kernel-failures\`:
`manifest.json` + `inputs/*.brep`. The manifest holds the op as serde `KernelOp`, versions,
tolerance, error class, the attempts log and a dedup hash.
- **Modes:** `OnFailure` is the owner default and costs nothing on success. `WriteAhead`
  (debug) also catches hard crashes and hangs.
- **Replay:** `capture::replay` re-runs a dump and classifies it as `Reproduced`,
  `DifferentFailure` or `NowSucceeds`.
- `amg-cli kernel replay|promote|harness` are thin wrappers (lane E). Until those exist they
  ship as `amg-brep` examples.
- Format, dedup, retention and index-stability rules: robustness note §1.

### 3. Robustness framework (KH-17)
Rules (robustness note §2):
1. **Valid or loud.** Every strategy result passes a validation gate: BRepCheck,
   closed/positive-volume solid, `max_tolerance ≤ cap`, op-specific volume bounds and an
   optional self-interference check.
2. **Deterministic chains.** Fixed order and fixed parameters. Deadlines abort the chain
   (`Timeout`) instead of falling through, so machine speed never changes the geometry.
3. Healing (`ShapeFix_Shape`, `ShapeUpgrade_UnifySameDomain`, re-sewing) runs only on copies,
   after the direct attempt failed. Its history is composed so naming still resolves.
4. **Tolerance escalation for booleans:** exact, then fuzzy `1e-5 → 1e-4 → 1e-3 mm`
   (provisional; capped at `min(tol_cap, 1e-3 × smallest input edge)`), with OCCT
   non-destructive mode and gluing for coincident faces. Any rung above 1e-5 mm adds a health
   warning.
5. **Policy presets:**
   - `interactive()` (preview);
   - `recompute()` (default);
   - `exhaustive()` (harness / "try harder");
   - `exact_only()` (scripts).

   `POLICY_VERSION` goes into `amg_brep::kernel_version()`, so a policy change invalidates
   S01 caches.

### 4. Fillet hardening (KH-03, KH-04, KH-19)
- **Strategy wrapper (KH-03).** Before reporting failure, try in this order:
  1. direct;
  2. `ChFi3d` shape variants and approximation parameters;
  3. unify same-domain faces;
  4. light heal;
  5. regroup edges by tangent chain (longest first);
  6. regroup by radius (largest first);
  7. per-edge sequential fillets;
  8. isolate the failing edge(s).

  Isolation uses OCCT's faulty-contour report first, then delta debugging (ddmin). It
  reports the edge, the reason class and the **max workable radius** (12-step bisection,
  checked so that `r̂` succeeds and `1.02·r̂` fails).
- **Analytic fast path (KH-04).** Constant-radius fillets between plane, cylinder and cone
  faces:
  1. Build the exact blend surface (cylinder, torus or cone, or a swept circle along an
     analytic spine).
  2. Trim it and merge it with the modern General Fuse booleans and the fuzzy ladder.

  This runs before the strategy chain. Cases it doesn't cover fall back to `ChFi3d` and the
  chain.
- **Corner patches (KH-19).** At 3-edge vertices: setback the blends, then fill the hole with
  a G1 patch, either OCCT `GeomPlate` or our own N-sided patch (Open question 2).

### 5. Loft hardening (owner priority; KH-06, KH-08, KH-18)
Loft failures mostly come from *section preparation*, not from the surface math, so we own both
(robustness note §4).
1. **Section preprocessing (KH-06, pure Rust on `amg-nurbs` chains):**
   - closed/open validation with clear errors;
   - consistent orientation;
   - merge tangent edge chains into single curves;
   - start-vertex alignment minimising total twist (cyclic shift + direction);
   - **edge-count matching** by splitting at arc-length-proportional points instead of
     OCCT's ad-hoc splitting;
   - point sections (apex) at start/end;
   - **user point mapping** as hard split points;
   - twist and crossing warnings.
2. **Surface construction (`amg-nurbs`):**
   - Sections become compatible exactly when the knot union stays small. Otherwise they are
     **approximated onto a shared knot vector** within `fit_tol = 1e-5 mm`, with ≤ 200 poles.
   - Skinning (P&T §10.3) builds **one patch per matched segment index**, with one global
     v-parametrisation (centripetal), so neighbouring patches share bit-identical boundaries.
   - **End conditions:** free, direction, tangent (G1), smooth (G2), point tangent, each with
     a weight. G1/G2 targets come from adjacent faces through the `CrossBoundaryField` trait.
   - **Rails:** Gordon surfaces per patch between split lines (KH-18).
   - **Centerline:** rotation-minimising frames plus intermediate sections (KH-18).
3. **Topology (KH-08):**
   - Patches become `Geom_BSplineSurface` faces (KH-16).
   - Caps are planar faces from boundary iso-curves when the sections are planar, otherwise
     the section face or a filling face.
   - OCCT sewing, solid creation and the validation gate.
4. **Fallback chain:**
   1. own builder;
   2. OCCT `ThruSections` on the *preprocessed* wires;
   3. OCCT `MakePipeShell` with multiple sections;
   4. for rails, OCCT `GeomFill_Gordon` if the OCCT pin is ≥ 8.0.0p1 (R15).

   The feature records which engine succeeded. Preprocessing errors are reported directly and
   don't fall through.
5. **Diagnostics for the dialog:** correspondence lines, twist warning, offending section,
   engine used, max section deviation and self-interference points (`LoftDiagnostics`;
   rendered by SOL-11).

### 6. Corpora, harness and metrics (KH-02, KH-05)
Robustness note §5–§6.
- **Committed cases:** handmade cases plus deterministic synthetic generators (seed lists,
  not files), with **exact volume oracles** where geometry allows:
  - fillets on vertical edges of extruded polygons;
  - translated or similar ruled lofts;
  - TiGL curve networks (Apache-2.0).
- **External cases:** owner and harvested cases live under `$AMG_TESTDATA_DIR/kernel/…`
  (QUESTIONS #7, #9, and the #19 licence rule).
- **Harness:** process-isolated, so an OCCT access violation counts as a `Crash`.
- **Metrics:** success rate, clean-failure rate, invalid-Ok count, crashes, volume deviation
  and p50/p95 time per strategy and tag. They are recorded in `docs/notes/kernel-metrics.md`.
- **Regression gate:** `tests/golden/kernel/*-baseline.toml` lists the passing case ids. The
  CI `golden` job runs a ~40-case smoke subset and fails on any regression. Every hardening WP
  shows its before/after table.

### 7. Rust rewrite (Phase 7 — only after everything else in the roadmap)
- **Fillet engine in Rust** on top of `amg-nurbs`: rolling-ball spine computation, blend
  surfaces (constant, variable, chord), vertex blends, trimming (our own or OCCT intersection
  routines), and topology rebuild. To ship, it must beat KH-03 + KH-04 + KH-19 on the fillet
  corpus.
- **G2 (curvature-continuous) fillets** (owner, QUESTIONS #55, 2026-11-09): needed, and OCCT
  cannot build them, so they are part of this engine: a G2 blend profile (e.g. a curvature-
  continuous cubic/quintic or conic-blend cross-section) on constant and variable radius edges
  and at tangent-chain ends, exposed as a continuity option (G1 default, G2) on `solid.fillet`
  (S04 SOL-05 ships G1 only and S04 adds the dialog option when KH-10 lands). Verified by
  curvature-comb sampling across the blend boundary.
- **Progressive kernel replacement:** the corpora decide which further OCCT operations
  (booleans, shell/offset, surface–surface intersection) are worth replacing with Rust behind
  `BrepKernel`. A full Rust kernel, possibly building on or contributing to `truck` /
  `monstertruck`, is a long-term option.
- KH-10 and KH-11 are placeholders. They are refined into 1–3-day WPs before Phase 7 starts;
  the G2 blend profile is one of KH-10's WPs.

## Interfaces
All signatures are public API that other crates rely on. Bodies are omitted. Both crates set
`#![warn(missing_docs)]`.

### `amg-nurbs` (depends on `amg-core` only; `nalgebra`, `smallvec`, `serde`, `thiserror` internal/public as noted)
```rust
use amg_core::{Aabb3, Cancelled, Dir3, Plane, Point2, Point3, Tolerance, Transform, Vec3, Line3};

pub const MAX_DEGREE: usize = 25; // = Geom_BSplineCurve::MaxDegree()

// ---- control-point space ---------------------------------------------------------------
/// Sealed: implemented for Point2 and Point3 only.
pub trait CtrlPt: Copy + std::fmt::Debug + PartialEq + Send + Sync + 'static
    + std::ops::Add<Output = Self> + std::ops::Sub<Output = Self> + std::ops::Mul<f64, Output = Self>
    + serde::Serialize + serde::de::DeserializeOwned + sealed::Sealed
{
    const DIM: usize;
    fn zero() -> Self;
    fn dot(self, o: Self) -> f64;
    fn length(self) -> f64;
}

// ---- knots ------------------------------------------------------------------------------
/// Validated, finite, non-decreasing knot vector.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(try_from = "Vec<f64>", into = "Vec<f64>")]
pub struct KnotVector(Vec<f64>);
impl KnotVector {
    pub fn new(knots: Vec<f64>) -> Result<KnotVector, NurbsError>;
    pub fn bezier(degree: usize) -> KnotVector;                         // [0;p+1] ++ [1;p+1]
    pub fn clamped_uniform(degree: usize, n_poles: usize) -> Result<KnotVector, NurbsError>;
    pub fn from_knots_mults(knots: &[f64], mults: &[usize]) -> Result<KnotVector, NurbsError>;
    /// Distinct knots + multiplicities (OCCT form); knots closer than `tol` merge.
    pub fn to_knots_mults(&self, tol: f64) -> (Vec<f64>, Vec<usize>);
    pub fn as_slice(&self) -> &[f64];
    pub fn len(&self) -> usize;
    pub fn domain(&self, degree: usize) -> (f64, f64);
    /// A2.1; `u == u_max` maps to the last non-empty span.
    pub fn find_span(&self, degree: usize, n_poles: usize, u: f64) -> usize;
    pub fn multiplicity(&self, u: f64, tol: f64) -> usize;
    pub fn is_clamped(&self, degree: usize) -> bool;
    pub fn reparametrized(&self, range: (f64, f64)) -> KnotVector;     // affine map
    /// Merge keeping the max multiplicity of each knot (`tol` on the knot values).
    pub fn union(&self, other: &KnotVector, tol: f64) -> KnotVector;
    /// Knots that must be inserted into `self` to obtain `target` (target ⊇ self).
    pub fn missing_from(&self, target: &KnotVector, tol: f64) -> Vec<f64>;
    pub fn greville(&self, degree: usize) -> Vec<f64>;
}
pub mod basis {
    /// A2.2: the `degree + 1` non-zero basis values at `u` in span `span`.
    pub fn basis_funs(knots: &[f64], span: usize, u: f64, degree: usize, out: &mut [f64]);
    /// A2.3: derivatives up to `n`; `out[k][j]` = k-th derivative of N_{span-p+j}.
    pub fn ders_basis_funs(knots: &[f64], span: usize, u: f64, degree: usize, n: usize,
                           out: &mut [[f64; MAX_DEGREE + 1]]);
}

// ---- curves -----------------------------------------------------------------------------
/// Clamped B-spline/NURBS curve. Invariants: 1 <= degree <= MAX_DEGREE,
/// knots.len() == poles.len() + degree + 1, clamped, weights (if any) > 0 and same length.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct NurbsCurve<P: CtrlPt = Point3> { /* degree, knots, poles: Vec<P>, weights: Option<Vec<f64>> */ }
pub type NurbsCurve2 = NurbsCurve<Point2>;
pub type NurbsCurve3 = NurbsCurve<Point3>;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CurvePoint<P> { pub t: f64, pub point: P, pub distance: f64 }
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
pub enum Continuity { C0, G1, C1, G2, C2, Cn }

impl<P: CtrlPt> NurbsCurve<P> {
    pub fn new(degree: usize, knots: KnotVector, poles: Vec<P>, weights: Option<Vec<f64>>)
        -> Result<Self, NurbsError>;
    pub fn degree(&self) -> usize;
    pub fn knots(&self) -> &KnotVector;
    pub fn poles(&self) -> &[P];
    pub fn weights(&self) -> Option<&[f64]>;
    pub fn is_rational(&self) -> bool;      // false if all weights equal
    pub fn domain(&self) -> (f64, f64);
    pub fn eval(&self, t: f64) -> P;                                   // A4.1 / A4.2
    /// `[C, C', …, C^(n)]`, rational-correct (A4.2); n <= 4.
    pub fn derivs(&self, t: f64, n: usize) -> smallvec::SmallVec<[P; 5]>;
    pub fn start(&self) -> P;
    pub fn end(&self) -> P;
    pub fn is_closed(&self, tol: &Tolerance) -> bool;
    pub fn reversed(&self) -> Self;
    pub fn reparametrized(&self, range: (f64, f64)) -> Self;
    pub fn insert_knot(&self, t: f64, times: usize) -> Result<Self, NurbsError>;   // A5.1
    pub fn refine_knots(&self, ts: &[f64]) -> Result<Self, NurbsError>;            // A5.4
    /// A5.8; removes up to `times` copies keeping deviation <= tol; returns copies removed.
    pub fn remove_knot(&self, t: f64, times: usize, tol: f64) -> (Self, usize);
    pub fn elevate_degree(&self, by: usize) -> Result<Self, NurbsError>;          // A5.9
    pub fn split(&self, t: f64) -> Result<(Self, Self), NurbsError>;
    pub fn segment(&self, t0: f64, t1: f64) -> Result<Self, NurbsError>;
    /// G0 join (end of self == start of other within tol); knots concatenated.
    pub fn join(&self, other: &Self, tol: &Tolerance) -> Result<Self, NurbsError>;
    pub fn bezier_segments(&self) -> Vec<Self>;                                    // A5.6
    pub fn continuity_at(&self, t: f64, tol: &Tolerance) -> Continuity;
    pub fn arc_length(&self, range: Option<(f64, f64)>, tol: f64) -> f64;
    pub fn param_at_length(&self, s: f64, tol: f64) -> f64;
    pub fn closest_point(&self, p: P, tol: &Tolerance) -> CurvePoint<P>;
}
impl NurbsCurve3 {
    pub fn transformed(&self, t: &Transform) -> Self;
    pub fn bbox(&self) -> Aabb3;                        // control-hull bound (conservative)
    pub fn curvature(&self, t: f64) -> f64;
    pub fn torsion(&self, t: f64) -> f64;
    /// Some(plane) if all poles lie within `tol` of one plane.
    pub fn planarity(&self, tol: &Tolerance) -> Option<Plane>;
}

// ---- surfaces ---------------------------------------------------------------------------
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum UV { U, V }
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum Side { UMin, UMax, VMin, VMax }
/// Row-major grid: `get(i, j)` with i along u (0..nu), j along v (0..nv).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Grid<T> { /* nu, nv, data: Vec<T> */ }
impl<T: Copy> Grid<T> {
    pub fn new(nu: usize, nv: usize, data: Vec<T>) -> Result<Self, NurbsError>;
    pub fn nu(&self) -> usize; pub fn nv(&self) -> usize;
    pub fn get(&self, i: usize, j: usize) -> T;
    pub fn row(&self, i: usize) -> &[T];        // fixed i, all j
    pub fn column(&self, j: usize) -> Vec<T>;   // fixed j, all i
    pub fn as_slice(&self) -> &[T];
}
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct NurbsSurface<P: CtrlPt = Point3> { /* degree: [usize;2], knots: [KnotVector;2], poles: Grid<P>, weights: Option<Grid<f64>> */ }
pub type NurbsSurface3 = NurbsSurface<Point3>;
/// `d[k][l]` = ∂^{k+l} S / ∂u^k ∂v^l, k + l <= order.
#[derive(Clone, Debug, PartialEq)]
pub struct SurfaceDerivs { pub d: [[Vec3; 3]; 3], pub order: usize }
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfacePoint { pub u: f64, pub v: f64, pub point: Point3, pub distance: f64 }

impl NurbsSurface3 {
    pub fn new(degree: [usize; 2], knots: [KnotVector; 2], poles: Grid<Point3>,
               weights: Option<Grid<f64>>) -> Result<Self, NurbsError>;
    pub fn degree(&self, d: UV) -> usize;
    pub fn knots(&self, d: UV) -> &KnotVector;
    pub fn poles(&self) -> &Grid<Point3>;
    pub fn weights(&self) -> Option<&Grid<f64>>;
    pub fn domain(&self) -> [(f64, f64); 2];
    pub fn eval(&self, u: f64, v: f64) -> Point3;                       // A4.3
    pub fn derivs(&self, u: f64, v: f64, order: usize) -> SurfaceDerivs; // A3.6 / A4.4, order <= 2
    pub fn normal(&self, u: f64, v: f64, tol: &Tolerance) -> Option<Dir3>;
    pub fn iso_curve(&self, d: UV, t: f64) -> NurbsCurve3;
    pub fn boundary(&self, side: Side) -> NurbsCurve3;
    pub fn insert_knot(&self, d: UV, t: f64, times: usize) -> Result<Self, NurbsError>;
    pub fn refine_knots(&self, d: UV, ts: &[f64]) -> Result<Self, NurbsError>;
    pub fn elevate_degree(&self, d: UV, by: usize) -> Result<Self, NurbsError>;
    pub fn split(&self, d: UV, t: f64) -> Result<(Self, Self), NurbsError>;
    pub fn reversed(&self, d: UV) -> Self;
    pub fn transposed(&self) -> Self;
    pub fn transformed(&self, t: &Transform) -> Self;
    pub fn bbox(&self) -> Aabb3;
    pub fn closest_point(&self, p: Point3, seed: Option<(f64, f64)>, tol: &Tolerance) -> SurfacePoint;
}

// ---- exact analytic conversions (KH-07) -------------------------------------------------
pub mod analytic {
    use super::*;
    pub fn line(a: Point3, b: Point3) -> NurbsCurve3;
    /// A7.1; ≤ 90° per rational quadratic segment. `start`/`sweep` in the plane's frame (rad).
    pub fn circle_arc(plane: &Plane, radius: f64, start: f64, sweep: f64) -> Result<NurbsCurve3, NurbsError>;
    pub fn ellipse_arc(plane: &Plane, rx: f64, ry: f64, t0: f64, t1: f64) -> Result<NurbsCurve3, NurbsError>;
    pub fn plane_patch(plane: &Plane, u: (f64, f64), v: (f64, f64)) -> NurbsSurface3;
    pub fn extrusion(c: &NurbsCurve3, dir: Vec3) -> NurbsSurface3;
    pub fn ruled(a: &NurbsCurve3, b: &NurbsCurve3) -> Result<NurbsSurface3, NurbsError>; // compat first
    /// A8.1; axis = Line3, angle in rad.
    pub fn revolution(profile: &NurbsCurve3, axis: &Line3, angle: f64) -> Result<NurbsSurface3, NurbsError>;
    pub fn cylinder(plane: &Plane, r: f64, h: f64, angle: f64) -> Result<NurbsSurface3, NurbsError>;
    pub fn cone(plane: &Plane, r0: f64, r1: f64, h: f64, angle: f64) -> Result<NurbsSurface3, NurbsError>;
    pub fn sphere(center: Point3, r: f64) -> Result<NurbsSurface3, NurbsError>;
    pub fn torus(plane: &Plane, major: f64, minor: f64) -> Result<NurbsSurface3, NurbsError>;
}

// ---- fitting (KH-13) --------------------------------------------------------------------
pub mod fit {
    use super::*;
    #[derive(Clone, Copy, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
    pub enum Parametrization { Uniform, ChordLength, #[default] Centripetal }
    pub fn parametrize<P: CtrlPt>(pts: &[P], m: Parametrization) -> Vec<f64>;       // in [0,1]
    pub fn averaging_knots(params: &[f64], degree: usize) -> KnotVector;            // eq. 9.8
    pub fn approximation_knots(params: &[f64], degree: usize, n_poles: usize) -> KnotVector; // eq. 9.68–9.69
    #[derive(Clone, Debug, PartialEq)]
    pub struct EndDerivs<P> { pub start: Option<P>, pub end: Option<P>, pub start2: Option<P>, pub end2: Option<P> }
    impl<P> Default for EndDerivs<P> { fn default() -> Self; }
    #[derive(Clone, Debug)]
    pub struct InterpOpts<P> {
        pub degree: usize,                    // default 3
        pub parametrization: Parametrization, // ignored if `params` is Some
        pub params: Option<Vec<f64>>,
        pub ends: EndDerivs<P>,
        pub closed: bool,                     // periodic interpolation, then clamped
    }
    /// A9.1 (+ §9.2.2/9.2.3 derivative rows). Data points reproduced to rounding.
    pub fn interpolate<P: CtrlPt>(pts: &[P], o: &InterpOpts<P>) -> Result<NurbsCurve<P>, NurbsError>;
    #[derive(Clone, Debug)]
    pub struct LsqOpts<P> {
        pub degree: usize,
        pub knots: KnotVector,
        pub params: Vec<f64>,
        pub fix_ends: bool,                   // interpolate first/last point exactly
        pub ends: EndDerivs<P>,               // optional derivative constraints
        pub data_weights: Option<Vec<f64>>,
    }
    #[derive(Clone, Debug)]
    pub struct Fit<P: CtrlPt> { pub curve: NurbsCurve<P>, pub max_error: f64, pub rms_error: f64, pub params: Vec<f64> }
    /// §9.4.1 / A9.6 (constrained, Lagrange multipliers).
    pub fn approximate_lsq<P: CtrlPt>(pts: &[P], o: &LsqOpts<P>) -> Result<Fit<P>, NurbsError>;
    /// Adaptive knot insertion + Hoschek parameter correction until max_error <= tol.
    pub fn approximate_to_tolerance<P: CtrlPt>(pts: &[P], degree: usize, tol: f64, max_poles: usize,
        m: Parametrization) -> Result<Fit<P>, NurbsError>;
    #[derive(Clone, Debug)]
    pub struct OntoOpts<P> { pub tol: f64, pub fix_end_tangents: bool, pub ends: EndDerivs<P>, pub corrections: u8 /* default 3 */ }
    /// Re-approximate `c` on the given degree + knot vector (shared-knot compatibility).
    pub fn approximate_onto_knots<P: CtrlPt>(c: &NurbsCurve<P>, degree: usize, knots: &KnotVector,
        o: &OntoOpts<P>) -> Result<Fit<P>, NurbsError>;
}

// ---- compatibility + skinning (KH-14) ---------------------------------------------------
pub mod skin {
    use super::*;
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct CompatOpts {
        pub fit_tol: f64,          // default 1e-5 mm
        pub max_poles: usize,      // default 200
        /// Exact path allowed while the union has <= this many poles; default max(64, 2 × max input).
        pub exact_pole_limit: Option<usize>,
    }
    impl Default for CompatOpts { fn default() -> Self; }
    #[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub enum CompatMethod { Exact, Approximated { max_error: f64 } }
    #[derive(Clone, Debug)]
    pub struct Compatible<P: CtrlPt> { pub curves: Vec<NurbsCurve<P>>, pub method: CompatMethod }
    pub fn make_compatible<P: CtrlPt>(curves: &[NurbsCurve<P>], o: &CompatOpts) -> Result<Compatible<P>, NurbsError>;

    /// Desired cross-boundary derivatives along a section (implemented by amg-brep for faces).
    pub trait CrossBoundaryField: Sync {
        /// Unit direction of dS/dv at section point `p` with section tangent `du`
        /// (in the target tangent plane, perpendicular to `du`, toward the loft interior).
        fn direction(&self, p: Point3, du: Vec3) -> Option<Dir3>;
        /// Normal curvature of the target surface along `dir` at `p`, with the target normal (G2).
        fn normal_curvature(&self, p: Point3, dir: Dir3) -> Option<(f64, Dir3)> { None }
    }
    #[derive(Clone, Copy)]
    pub enum EndCondition<'a> {
        Free,
        Direction { dir: Dir3, weight: f64 },
        Tangent { field: &'a dyn CrossBoundaryField, weight: f64 },   // G1
        Smooth { field: &'a dyn CrossBoundaryField, weight: f64 },    // G2
        PointTangent { normal: Dir3, weight: f64 },                   // apex sections only
    }
    #[derive(Clone, Debug, PartialEq)]
    pub enum Section { Chain(Vec<NurbsCurve3>), Point(Point3) }
    #[derive(Clone, Debug, PartialEq)]
    pub enum VParams { Centripetal, ChordLength, Uniform, Explicit(Vec<f64>) }
    #[derive(Clone, Copy)]
    pub struct SkinOpts<'a> {
        pub degree_v: usize,          // default 3; reduced to fit the section count
        pub v_params: &'a VParams,
        pub start: EndCondition<'a>,
        pub end: EndCondition<'a>,
        pub closed: bool,             // loop back to the first section
        pub ruled: bool,              // degree_v = 1
        pub compat: &'a CompatOpts,
    }
    #[derive(Clone, Debug)]
    pub struct LoftSurfaces {
        pub patches: Vec<NurbsSurface3>,   // one per matched segment index
        pub v_params: Vec<f64>,
        pub compat: Vec<CompatMethod>,     // per patch
        pub max_section_error: f64,
    }
    /// Sections must already be matched (same segment count per Chain; KH-06).
    pub fn loft_surfaces(sections: &[Section], o: &SkinOpts<'_>) -> Result<LoftSurfaces, NurbsError>;
}

// ---- Gordon + frames (KH-15) ------------------------------------------------------------
pub mod gordon {
    use super::*;
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct GordonOpts { pub net_tol: f64 /* 1e-4 mm */, pub compat: skin::CompatOpts, pub degree: [usize; 2] /* [3,3] */ }
    #[derive(Clone, Debug)]
    pub struct Gordon { pub surface: NurbsSurface3, pub profile_params: Vec<f64>, pub guide_params: Vec<f64>, pub max_network_error: f64 }
    pub fn gordon(profiles: &[NurbsCurve3], guides: &[NurbsCurve3], o: &GordonOpts) -> Result<Gordon, NurbsError>;
}
pub mod frames {
    use super::*;
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct Frame { pub origin: Point3, pub tangent: Dir3, pub normal: Dir3, pub binormal: Dir3 }
    /// Double reflection (Wang et al. 2008) at the given parameters, starting with `normal0`.
    pub fn rotation_minimizing(c: &NurbsCurve3, params: &[f64], normal0: Dir3, tol: &Tolerance)
        -> Result<Vec<Frame>, NurbsError>;
    /// Insert `per_gap` intermediate sections along `centerline` (nurbs note §7).
    pub fn centerline_sections(sections: &[skin::Section], centerline: &NurbsCurve3, per_gap: usize,
        o: &skin::CompatOpts, tol: &Tolerance) -> Result<Vec<skin::Section>, NurbsError>;
}

// ---- quality checks (KH-14) -------------------------------------------------------------
pub mod check {
    use super::*;
    #[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct ContinuityReport { pub g0_max: f64, pub g1_max_angle: f64, pub g2_max_rel: Option<f64>, pub samples: usize }
    pub fn boundary_continuity(a: &NurbsSurface3, sa: Side, b: &NurbsSurface3, sb: Side, samples: usize) -> ContinuityReport;
    pub fn curve_surface_deviation(c: &NurbsCurve3, s: &NurbsSurface3, samples: usize, tol: &Tolerance) -> f64;
    pub fn curve_curve_deviation(a: &NurbsCurve3, b: &NurbsCurve3, samples: usize, tol: &Tolerance) -> f64;
}

// ---- errors -----------------------------------------------------------------------------
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum NurbsError {
    #[error("invalid knot vector: {0}")] InvalidKnots(&'static str),
    #[error("{poles} poles and {knots} knots do not fit degree {degree}")]
    CountMismatch { degree: usize, poles: usize, knots: usize },
    #[error("degree {0} outside 1..={MAX_DEGREE}")] Degree(usize),
    #[error("weight {index} is not positive")] Weight { index: usize },
    #[error("parameter {0} is outside the curve domain")] OutOfDomain(f64),
    #[error("degenerate geometry: {0}")] Degenerate(&'static str),
    #[error("singular system in {0}")] Singular(&'static str),
    #[error("approximation reached {achieved:e} mm, needed {tol:e} mm")] ToleranceNotMet { tol: f64, achieved: f64 },
    #[error("needs {needed} poles, limit is {limit}")] TooManyPoles { needed: usize, limit: usize },
    #[error("sections are incompatible: {0}")] Incompatible(String),
    #[error("profile {profile} and guide {guide} miss by {distance:e} mm")]
    NetworkMiss { profile: usize, guide: usize, distance: f64 },
    #[error("profile {profile} crosses guide {guide} more than once")]
    NetworkMultiple { profile: usize, guide: usize },
    #[error(transparent)] Cancelled(#[from] Cancelled),
}
impl amg_core::Diagnostic for NurbsError { /* hints: "reduce section complexity", "rail misses section N" */ }
```

### `amg_brep::nurbs_conv` (KH-16, OCCT side)
Built on S02's plain geometry types (`geom::Curve3`, `CurveSeg`, `Surface`,
`BSplineCurveData`, `BSplineSurfaceData`) and `BrepKernel::make_face`. Because of the orphan rule
the conversions live in `amg-brep`, not in `amg-nurbs`. No OCCT type is public.
```rust
use amg_nurbs::{NurbsCurve3, NurbsSurface3};
impl From<&NurbsCurve3> for geom::BSplineCurveData { /* knots → (knots, mults), periodic = false */ }
impl From<&NurbsSurface3> for geom::BSplineSurfaceData { /* row-major u-outer, same as Grid */ }
/// Periodic data is unwrapped and clamped (KH-12 knot insertion); exact.
impl TryFrom<&geom::BSplineCurveData> for NurbsCurve3 { type Error = amg_nurbs::NurbsError; }
impl TryFrom<&geom::BSplineSurfaceData> for NurbsSurface3 { type Error = amg_nurbs::NurbsError; }
/// Exact NURBS of a trimmed analytic or B-spline curve (`Curve3::Other` → Err).
pub fn curve_seg_to_nurbs(c: &geom::CurveSeg) -> KResult<NurbsCurve3>;
/// Exact NURBS of an analytic/B-spline surface over a UV box (`Surface::Other` → Err).
pub fn surface_to_nurbs(s: &geom::Surface, uv: [(f64, f64); 2]) -> KResult<NurbsSurface3>;
pub fn edge_to_nurbs(s: &Shape, e: EdgeId) -> KResult<NurbsCurve3>;
/// Edges of a wire shape in wire order, each oriented along the wire (S02 request R16).
pub fn wire_to_chain(wire: &Shape) -> KResult<Vec<NurbsCurve3>>;
/// `k.make_face(&Surface::BSpline(s.into()), bounds, tol)`.
pub fn nurbs_face(k: &dyn BrepKernel, s: &NurbsSurface3, bounds: &geom::FaceBounds, tol: f64) -> KResult<FaceBuild>;
```

### `amg_brep::capture` (KH-01)
```rust
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum CaptureMode { Off, #[default] OnFailure, WriteAhead }
#[derive(Clone, Debug)]
pub struct CaptureConfig { pub mode: CaptureMode, pub dir: std::path::PathBuf, pub max_bytes: u64 /* 1 GiB */, pub max_dumps: u32 /* 1000 */ }
impl CaptureConfig {
    /// Default dir %LOCALAPPDATA%\Amalgam\kernel-failures; env AMG_KERNEL_CAPTURE, AMG_KERNEL_CAPTURE_DIR.
    pub fn from_env() -> CaptureConfig;
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub struct InputRef(pub u32);
/// One variant per BrepKernel operation (S02 param types). Every WP adding an op adds a variant.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
#[serde(tag = "op", rename_all = "snake_case")]
#[non_exhaustive]
pub enum KernelOp {
    Boolean { kind: BoolOp, objects: Vec<InputRef>, tools: Vec<InputRef>, opts: BoolOpts },
    Fillet { shape: InputRef, edges: Vec<(EdgeRecord, FilletSpec)>, opts: FilletOpts },
    Chamfer { shape: InputRef, edges: Vec<(EdgeRecord, ChamferSpec)> },
    ThruSections { wires: Vec<InputRef>, sections: Vec<LoftSection>, opts: ThruSectionsOpts },
    Pipe { operands: Vec<InputRef>, profiles: usize, opts: PipeOpts },
    Loft { spec: robust::loft::LoftSpecRecord },
    Sew { faces: Vec<InputRef>, opts: SewOpts },
    Heal { shape: InputRef, opts: HealOpts },
    UnifySameDomain { shape: InputRef, opts: UnifyOpts },
    // … extrude, revolve, pipe, shell, draft, offset as S02 adds them
}
/// Edge id plus a fingerprint used when indices do not survive reload.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct EdgeRecord { pub id: EdgeId, pub geom: SubShapeGeom /* S02: class, length, centroid, axis, bbox */ }
pub type DumpId = String; // directory name
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[non_exhaustive]
pub enum FailureClass {
    OcctException, NotDone, InvalidResult, ToleranceTooLarge, RadiusTooLarge, VertexBlend,
    WalkingFailed, SectionPreprocess, SurfaceConstruction, SewingFailed, NotSolid, SelfInterference,
    Timeout, Cancelled, Crash, Panic, Other,
}
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct FailureInfo { pub class: FailureClass, pub message: String, pub occt_type: Option<String>,
                        pub faulty: Vec<(SubShape, SubShapeGeom)> }
impl FailureInfo { pub fn from_kernel_error(e: &KernelError, s: Option<&Shape>) -> FailureInfo; }

pub struct FailureRecorder { /* config, dedup index */ }
impl FailureRecorder {
    pub fn new(cfg: CaptureConfig) -> std::io::Result<FailureRecorder>;
    /// Moves leftover write-ahead dumps (crashes) into the capture dir; call at startup.
    pub fn recover_pending(&self) -> std::io::Result<usize>;
    pub fn record(&self, op: &KernelOp, inputs: &[&Shape], err: &FailureInfo,
                  attempts: &[robust::Attempt]) -> std::io::Result<DumpId>;
}
/// BrepKernel decorator: forwards every call, records failures per `CaptureConfig`.
pub struct RecordingKernel<K: BrepKernel> { /* inner, recorder */ }
impl<K: BrepKernel> RecordingKernel<K> { pub fn new(inner: K, recorder: std::sync::Arc<FailureRecorder>) -> Self; }
impl<K: BrepKernel> BrepKernel for RecordingKernel<K> { /* forwards */ }

#[derive(Clone, Debug, Default)]
pub struct ReplayOpts { pub robust: Option<robust::RetryPolicy>, pub validate: bool }
#[derive(Clone, Debug, serde::Serialize)]
pub enum ReplayOutcome { Reproduced(FailureInfo), DifferentFailure(FailureInfo), NowSucceeds { valid: bool } }
#[derive(Clone, Debug, serde::Serialize)]
pub struct ReplayReport { pub dump: std::path::PathBuf, pub outcome: ReplayOutcome, pub millis: u64, pub reindexed_by_fingerprint: bool }
pub fn replay(dump_dir: &std::path::Path, k: &dyn BrepKernel, o: &ReplayOpts) -> Result<ReplayReport, CaptureError>;
/// Dump → corpus case directory (case.toml + op.json + inputs/), `expect` from a replay.
pub fn promote(dump_dir: &std::path::Path, corpus_dir: &std::path::Path, id: &str, k: &dyn BrepKernel)
    -> Result<std::path::PathBuf, CaptureError>;
#[derive(Debug, thiserror::Error)] pub enum CaptureError { /* Io, Schema, Version, Kernel(KernelError) */ }
```

### `amg_brep::robust` (KH-17, KH-03, KH-04, KH-19, KH-06, KH-08, KH-18)
```rust
pub const POLICY_VERSION: u32 = 1; // bumped by any change of default chains/ladders

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RetryPolicy {
    pub fuzzy_ladder: Vec<f64>,          // mm, after the exact attempt; default [1e-5, 1e-4, 1e-3]
    pub tol_cap: f64,                    // mm, default 1e-3
    pub heal: HealLevel,
    pub max_attempts: u8,
    pub attempt_deadline: std::time::Duration, // expiry aborts the chain (Timeout), never falls through
    pub validation: Validation,
}
impl RetryPolicy {
    pub fn interactive() -> Self; pub fn recompute() -> Self; pub fn exhaustive() -> Self; pub fn exact_only() -> Self;
}
impl Default for RetryPolicy { fn default() -> Self { Self::recompute() } }
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
pub enum HealLevel { None, Light, Unify, Full }
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Validation {
    pub brepcheck: bool,                 // default true
    pub require_solid: bool,             // set per op
    pub max_tolerance: f64,              // default = tol_cap
    pub volume_bounds: Option<(f64, f64)>,
    pub self_interference: bool,         // default false; loft sets true
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[non_exhaustive]
pub enum StrategyId {
    Direct, Fuzzy { rung: u8 }, Glue { rung: u8 }, Heal(HealLevel), FilletShapeVariant(u8),
    UnifySameDomain, RegroupChains, RegroupRadius, PerEdgeSequential, AnalyticFastPath, CornerPatch,
    LoftOwn, LoftThruSections, LoftPipeShell, LoftOcctGordon,
}
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub enum Rejection {
    Check(Vec<String>), NotSolid, ToleranceTooLarge { max: f64, cap: f64 },
    VolumeOutOfBounds { got: f64, lo: f64, hi: f64 }, SelfInterference(usize),
}
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub enum AttemptOutcome { Accepted, Failed(capture::FailureInfo), Rejected(Rejection) }
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Attempt { pub strategy: StrategyId, pub outcome: AttemptOutcome, pub millis: u32 }
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct RobustReport { pub attempts: Vec<Attempt>, pub accepted: Option<StrategyId>, pub max_tolerance: f64, pub warnings: Vec<String> }
#[derive(Clone, Debug)]
pub struct RobustOutput { pub result: OpResult, pub report: RobustReport }
pub struct RobustCtx<'a> {
    pub job: &'a amg_jobs::JobCtx,             // progress + cancellation, passed through to S02 ops
    pub recorder: Option<&'a capture::FailureRecorder>,
    pub tol: amg_core::Tolerance,
}
#[derive(Debug, thiserror::Error)]
#[error("{class:?}: {message}")]
pub struct RobustError { pub class: capture::FailureClass, pub message: String, pub report: RobustReport }
impl amg_core::Diagnostic for RobustError { /* hint per class */ }

pub fn validate(k: &dyn BrepKernel, s: &Shape, v: &Validation) -> Result<f64 /* max tol */, Rejection>;
pub fn robust_boolean(k: &dyn BrepKernel, op: BoolOp, objects: &[Shape], tools: &[Shape], base: &BoolOpts, p: &RetryPolicy,
    cx: &RobustCtx<'_>) -> Result<RobustOutput, RobustError>;
pub fn heal(k: &dyn BrepKernel, s: &Shape, level: HealLevel, cx: &RobustCtx<'_>) -> Result<RobustOutput, RobustError>;

pub mod fillet {
    use super::*;
    #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
    pub struct FailingEdge { pub edge: EdgeId, pub class: capture::FailureClass, pub max_radius: Option<f64>, pub approximate: bool }
    #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
    pub struct FilletDiagnosis { pub failing: Vec<FailingEdge>, pub report: RobustReport }
    #[derive(Debug, thiserror::Error)]
    #[error("fillet failed on {} edge(s)", .diagnosis.failing.len())]
    pub struct FilletFailure { pub diagnosis: FilletDiagnosis }
    /// Analytic fast path → strategy chain (design §4). Edges by S02 EdgeId.
    pub fn fillet(k: &dyn BrepKernel, s: &Shape, edges: &[(EdgeId, FilletSpec)], base: &FilletOpts,
        p: &RetryPolicy, cx: &RobustCtx<'_>) -> Result<RobustOutput, FilletFailure>;
    /// Max workable radius per failing edge: S02 `fillet_max_radius` (12 iterations) on the edge
    /// alone, then the 1.02·r̂ consistency check; ≤ 8 edges. Run as a background job.
    pub fn max_radius(k: &dyn BrepKernel, s: &Shape, edge: EdgeId, requested: f64,
        cx: &RobustCtx<'_>) -> Result<(f64, bool /* approximate */), RobustError>;
    /// True if the edge set is fully inside the analytic fast-path scope (KH-04).
    pub fn analytic_scope(k: &dyn BrepKernel, s: &Shape, edges: &[(EdgeId, FilletSpec)]) -> bool;
}

pub mod loft {
    use super::*;
    use amg_nurbs::skin::{CompatOpts, VParams};
    // Types holding `Shape` are not serde; `LoftSpecRecord` is the serializable form.
    #[derive(Clone, Debug)]
    pub enum SectionInput { Wire(Shape) /* a wire shape */, Face { shape: Shape, face: FaceId } /* outer wire */, Point(amg_core::Point3) }
    #[derive(Clone, Debug)]
    pub enum EndSpec {
        Free,
        Direction { angle: f64, weight: f64 },                       // relative to the section plane normal
        Tangent { face: (Shape, FaceId), weight: f64 },              // G1
        Smooth { face: (Shape, FaceId), weight: f64 },               // G2
        PointTangent { normal: amg_core::Dir3, weight: f64 },
    }
    /// A user correspondence: parameter on section a ↔ parameter on section a+1 (arc-length fraction in [0,1)).
    #[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct MapPoint { pub section: usize, pub s_this: f64, pub s_next: f64 }
    #[derive(Clone, Debug)]
    pub enum Guide { None, Rails(Vec<Shape /* edge or wire */>), Centerline { path: Shape /* edge or wire */, per_gap: u8 } }
    #[derive(Clone, Debug)]
    pub struct LoftSpec {
        pub sections: Vec<SectionInput>,
        pub guide: Guide,
        pub start: EndSpec, pub end: EndSpec,
        pub mapping: Vec<MapPoint>,
        pub closed: bool, pub ruled: bool, pub solid: bool,
        pub v_params: VParams,
        pub compat: CompatOpts,
        pub merge_tangent_edges: bool,     // default true
    }
    /// Serializable form for failure dumps (shapes → InputRef).
    #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
    pub struct LoftSpecRecord { /* LoftSpec with Shape replaced by capture::InputRef */ }
    #[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
    pub enum LoftEngine { Own, OcctThruSections, OcctPipeShell, OcctGordon }
    #[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
    pub struct LoftDiagnostics {
        pub correspondence: Vec<Vec<(amg_core::Point3, amg_core::Point3)>>, // per gap, per matched vertex
        pub twist_deg: Vec<f64>,           // per gap
        pub warnings: Vec<LoftWarning>,
        pub offending_section: Option<usize>,
        pub self_interference: Vec<amg_core::Point3>,
    }
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub enum LoftWarning { HighTwist { gap: usize, deg: f64 }, CrossingCorrespondence { gap: usize }, Approximated { max_error: f64 }, NonPlanarCap { section: usize } }
    #[derive(Clone, Debug)]
    pub struct LoftOutput { pub result: OpResult, pub engine: LoftEngine, pub max_section_error: f64,
                            pub diagnostics: LoftDiagnostics, pub report: RobustReport }
    #[derive(Debug, thiserror::Error)]
    #[non_exhaustive]
    pub enum LoftError {
        #[error("section {section}: {reason}")] Section { section: usize, reason: SectionProblem },
        #[error("rail {rail} misses section {section} by {distance:e} mm")] RailMiss { rail: usize, section: usize, distance: f64 },
        #[error("loft failed with every engine")] AllEnginesFailed { diagnostics: LoftDiagnostics, report: RobustReport },
        #[error(transparent)] Cancelled(#[from] amg_core::Cancelled),
    }
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub enum SectionProblem { MixedClosedOpen, SelfIntersecting, ZeroLength, PointNotAtEnd, NotConnected, MappingOutOfRange }
    /// KH-06 preprocessing only (fast; used by the dialog for live correspondence lines).
    pub fn prepare(k: &dyn BrepKernel, spec: &LoftSpec, tol: &amg_core::Tolerance)
        -> Result<(Vec<amg_nurbs::skin::Section>, LoftDiagnostics), LoftError>;
    /// Full pipeline with fallback chain (design §5).
    pub fn build(k: &dyn BrepKernel, spec: &LoftSpec, p: &RetryPolicy, cx: &RobustCtx<'_>) -> Result<LoftOutput, LoftError>;
}
```

`HardenedKernel` is the decorator that S02 §10 expects. It lets features keep taking
`&dyn BrepKernel`:
```rust
/// Routes `boolean`/`split` through `robust_boolean` and `fillet` through `robust::fillet::fillet`
/// under `policy`; every other method forwards. Non-direct acceptances become
/// `KernelWarning::Strategy` entries in `OpResult::warnings` (S02 request R17).
/// Callers that need the full `RobustReport` or `LoftOutput` call `robust::*` directly
/// (SOL-05, SOL-11).
pub struct HardenedKernel<K: BrepKernel> { /* inner: K, policy: RetryPolicy, recorder: Option<Arc<capture::FailureRecorder>> */ }
impl<K: BrepKernel> HardenedKernel<K> {
    pub fn new(inner: K, policy: RetryPolicy, recorder: Option<std::sync::Arc<capture::FailureRecorder>>) -> Self;
}
impl<K: BrepKernel> BrepKernel for HardenedKernel<K> { /* … */ }
```

### Expected from S02 (requests; S02 owns the API, S20 assumes these)
The S02 refinement running in parallel already takes most of these up (draft of 2026-10-20).
The "S02 draft" column records what was seen there. Anything not marked "yes" is still an
open request.
| # | Request | Needed by | S02 draft |
|---|---|---|---|
| R1 | Text `.brep` write/read (`Shape::write_text`/`read_text`, no triangulation) in **BREP-02**, so KH-01 can run in Phase 1 | KH-01 | yes (BREP-02) |
| R2 | Deterministic in-shape indices (`FaceId`/`EdgeId`/`VertexId`) that survive a `.brep` round trip, plus `sub_geom` fingerprints | KH-01, KH-02 | yes (BREP-02/05; S20 KH-01 adds the text-`.brep` round-trip test) |
| R3 | Every op parameter type is `Clone + Debug + PartialEq + Serialize + Deserialize`; shape inputs only as `&Shape`/`&[Shape]` | KH-01 | yes (BREP-12 compile test) |
| R4 | Object-safe `BrepKernel: Send + Sync`, forwardable by decorators; `Shape: Send + Sync` (ADR-014) | KH-01, KH-17 | yes |
| R5 | `check(&Shape, &CheckOpts { self_interference })`, `Shape::max_tolerance()`, `mass_props` | KH-17 | yes (BREP-10/12/09) |
| R6 | `BoolOpts { fuzzy, glue, … }` (non-destructive always on), coincident-face detection usable to decide gluing | KH-17 | partly: `BoolOpts` yes; coincidence query: S20 derives it from `sub_geom` (plane/axis equality), so no request remains |
| R7 | `FilletOpts { shape, tol_app_3d, fleche }`; `LocalOpFailed { offending }` with faulty contours/vertices as `SubShape`s; `fillet_max_radius` | KH-03 | yes (BREP-06) |
| R8 | `heal`, `unify_same_domain`, `sew` (+ make solid) as ops with history; `ShapeHistory::compose` | KH-17, KH-03, KH-08 | yes (BREP-05/10) |
| R9 | Progress/cancel bridge into OCCT (`Message_ProgressRange` ↔ `JobCtx`) | KH-17 | yes (BREP-12) |
| R10 | Structured `KernelError` (`Occt { kind }`, `OpFailed`, `LocalOpFailed`, `InvalidResult { report }`, `Cancelled`), mapped by S20 to `FailureClass` | KH-01 | yes |
| R11 | Raw `thru_sections` (ruled/smooth, point sections, `max_degree`, compatibility check) and multi-section `pipe` as baseline + fallback engines | KH-05, KH-08 | yes (BREP-15) |
| R12 | `kernel_version()` includes `robust::POLICY_VERSION` | KH-17 | yes |
| R13 | Fast-path classification data: face `Surface` params, edge `Curve3` type, edge↔face adjacency, `eval_surface` with curvatures (also used by `CrossBoundaryField` for G2) | KH-04, KH-08 | yes (BREP-02/09); S20 computes dihedral angle/convexity itself |
| R14 | `amg-occt-sys` accepts additive S20 shim files (`plate.cpp` for `GeomPlate` in KH-19, `gordon.cpp` for `GeomFill_Gordon` in KH-16/KH-18) under S02's bridge and CATEGORY conventions | KH-16, KH-18, KH-19 | open |
| R15 | Bump the pin from OCCT 8.0.0 to **8.0.1** (current vcpkg port; S02 already notes it fixes the `Standard_Strtod` race). 8.0.0p1 replaced 8.0.0's experimental `GeomFill_Gordon`, and 8.0.1 made fillet reconstruction state thread-local. Both matter to S20 (Gordon oracle/engine, parallel harness). With 8.0.0, KH-16/KH-18 skip the OCCT Gordon parts | KH-16, KH-18, KH-02 | open (S02 pins 8.0.0) |
| R16 | Ordered wire traversal (`BRepTools_WireExplorer`): `Shape::wire_edges(&self) -> Vec<(EdgeId, bool /* reversed */)>` | KH-16 | open (confirm) |
| R17 | Additive `KernelWarning::Strategy { name: &'static str, detail: String }` so `HardenedKernel` can report a non-direct acceptance through `OpResult::warnings` | KH-17 | done (KH-17, 2026-11-28; S02 log note) |

## Work packages
| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| KH-01 | Kernel failure capture: `capture` module (`CaptureConfig`, `FailureRecorder`, `RecordingKernel`, `KernelOp` for the ops existing at the time, `EdgeRecord` fingerprints), `OnFailure` + `WriteAhead` modes, dedup + retention, `replay`, `promote`; `kernel-replay` example (and `amg-cli kernel replay` if lane E accepts) | BREP-02, BREP-12, BREP-04, FND-04 | P0 | Done | 1. Forced failures (too-large fillet radius, boolean on an invalid argument, an injected `NotDone`) write a dump whose `manifest.json` validates against schema 1. 2. Replaying each dump **in a fresh process** gives `Reproduced` for 100 % of dumps in a ≥ 10-dump test set covering every op variant present. 3. Index stability: on 20 STEP parts + 200 generated parts, all edge/face indices survive `.brep` write → read; a forced re-index falls back to fingerprints and reports `reindexed_by_fingerprint`. 4. `WriteAhead`: a child process aborted inside the op leaves `pending/`, and `recover_pending` turns it into a `Crash` dump that replays. 5. The same failure twice gives one dump with `count = 2`; retention prunes to `max_dumps`/`max_bytes`. 6. `OnFailure` adds no file I/O and < 1 µs per successful call (criterion). |
| KH-07 | `amg-nurbs` core: `CtrlPt`, `KnotVector`, `basis`, `NurbsCurve`/`NurbsSurface` + `Grid` with validation, eval + derivatives (A2.1–A2.3, A3.2, A3.6, A4.1–A4.4), reverse/reparametrize/transform/bbox, `analytic` module (A7.1, ellipse, A8.1, extrusion, ruled-with-equal-knots, cylinder, cone, sphere, torus), `NurbsError`; curvo dev-dep oracle | FND-03 | P1 | Done | 1. P&T Ex. 2.3 basis values to 1e-15, partition of unity to 1e-14. 2. Rational circle radius error ≤ 1e-14·r; sphere and torus points on the analytic surface ≤ 1e-13·R (1000 samples). 3. Derivatives match central finite differences within 1e-6 relative and analytic circle/helix tangent and curvature within 1e-12. 4. Differential test vs curvo 0.3.3 on 1000 random curves/surfaces (degree 1–7, rational and non-rational): ≤ 1e-12·max(1, D). 5. Constructors reject every invalid input with the right `NurbsError` (table test); no panic in a proptest over random inputs. 6. Benchmarks: cubic point < 60 ns, bicubic point + first derivatives < 250 ns. |
| KH-12 | `amg-nurbs` knot operations: insertion (A5.1/A5.3), refinement (A5.4/A5.5), Bézier decomposition (A5.6/A5.7), removal (A5.8), degree elevation (A5.9/A5.10), split/segment/join, `continuity_at`, `KnotVector::union`/`missing_from` | KH-07 | P1 | Todo | 1. Proptests (512 cases): insertion, refinement, elevation, decomposition and split + join keep the shape within 1e-12·max(1, D) at 1000 samples. 2. Removal keeps deviation ≤ `tol` and removes the knot when the curve was produced by insertion (round trip). 3. Elevating by k then removing knots gives the original pole count back for Bézier inputs. 4. Matches curvo `try_refine_knot`/`try_elevate_degree` ≤ 1e-12·D. 5. Benchmarks: refine 20 knots into 50 poles < 10 µs; elevate 3→5 of 50 poles < 20 µs. |
| KH-13 | `amg-nurbs` fitting: `parametrize`, averaging/approximation knots, `interpolate` (A9.1 with D1/D2 end rows, closed), `approximate_lsq` (A9.6 constrained), `approximate_to_tolerance` (adaptive + Hoschek), `approximate_onto_knots`, arc length + inverse, curve/surface `closest_point` | KH-12 | P1 | Todo | 1. Interpolation reproduces data ≤ 1e-12·D and end derivatives within 1e-10 relative; P&T Ex. 9.1 data gives the book's parameters and knots. 2. `approximate_to_tolerance` on 1000 noisy samples of circles, helices and sine curves meets `tol` ∈ {1e-3, 1e-5} mm with ≤ 1.5× the pole count of a reference fit (the reference is the same algorithm without parameter correction, which proves that correction helps). 3. Reported `max_error` is within 1 % of the measured error at 2000 samples. 4. Closest point never exceeds the dense-sampling minimum + 1e-9 mm (10 k proptest queries, curves and surfaces). 5. Arc length of a circle arc is within 1e-12 relative. 6. Benchmarks: interpolate 200 points < 100 µs; fit 1000 points to 1e-5 mm < 5 ms. |
| KH-14 | `amg-nurbs` compatibility + skinning: `make_compatible` (exact/approximated, rules in nurbs note §5.2), `loft_surfaces` (one patch per segment, shared v-params, shared corner derivatives), all `EndCondition`s with `CrossBoundaryField`, ruled/closed, `check` module | KH-13 | P1 | Todo | 1. Exact path: outputs equal the inputs ≤ 1e-12·D; approximated path: ≤ `fit_tol` (1e-5 mm) with ≤ `max_poles`; it falls back to approximation exactly when the union exceeds `exact_pole_limit` (unit test at the boundary). 2. Skins reproduce every section at its v-parameter (≤ 1e-12·D exact path, ≤ `fit_tol` approximated). 3. Neighbouring patches have bit-identical shared boundaries (`to_bits`), and G1 across tangent-merged segments is ≤ 1e-8 rad. 4. G1 end condition against analytic planes/cylinders: angle ≤ 1e-8 rad at 100 boundary samples; G2: normal-curvature mismatch ≤ 1e-6 relative; `PointTangent` apex tangent plane ⊥ `normal` within 1e-9 rad. 5. Two sections without end conditions give a ruled surface. 6. Benchmarks: 6 sections × 8 segments exact < 2 ms; approximated < 10 ms. |
| KH-15 | `amg-nurbs` Gordon surfaces (network sort, Fritsch–Carlson reparametrisation, compatible families, boolean sum, closed networks) + `frames::rotation_minimizing` + `frames::centerline_sections` | KH-14 | P1 | Todo | 1. Every network curve is reproduced within `max(fit_tol, 1e-12·D)` on synthetic networks (bilinear, 3×3 … 6×6, non-uniform intersection parameters, closed) and all TiGL `CurveNetworks` fixtures (Apache-2.0, attributed). 2. Networks that miss by > `net_tol` give `NetworkMiss` with the right indices; a doubly-crossing guide gives `NetworkMultiple`. 3. A counter-test shows the uniform-parameter/degree-1-tensor shortcut fails these fixtures (guards the design choice). 4. RMF: orthonormal to 1e-12; planar curve normal constant to 1e-12; helix rotation vs Frenet = −∫τ ds within 1e-6 rad at 1000 steps. 5. Centerline sections: original sections are reproduced exactly; intermediate frames follow the RMF. 6. Gordon 5×5 < 20 ms. |
| KH-16 | `amg_brep::nurbs_conv`: amg-nurbs ↔ OCCT `Geom_BSplineCurve/Surface` (+ knots/mults), `curve_seg_to_nurbs`, `surface_to_nurbs`, `edge_to_nurbs`, `wire_to_chain`, `nurbs_face` (on S02 `make_face`); OCCT differential tests (incl. `GeomFill_Gordon` on TiGL networks when OCCT ≥ 8.0.0p1) | KH-07, KH-12, BREP-03 | P1 | Todo | 1. amg-nurbs → OCCT → amg-nurbs round-trips bit-identically (poles, weights, knots). 2. OCCT `D0/D1/D2` vs `eval/derivs` on 10 k random parameters of 200 random curves/surfaces ≤ 1e-11·max(1, D). 3. Lines, circles, ellipses, planes, cylinders, cones, spheres and tori from OCCT convert to exact NURBS (radius error ≤ 1e-12·r). 4. `nurbs_face` on a skinned patch gives a BRepCheck-valid face whose OCCT tessellation lies within 1e-6 mm of `eval`. 5. With OCCT ≥ 8.0.0p1, KH-15 Gordon results lie within 1e-4 mm of `GeomFill_Gordon` on the TiGL set (or the differences are explained in the PR). |
| KH-17 | `amg_brep::robust` framework: `RetryPolicy` presets, `validate` gate, `Attempt`/`RobustReport`, deterministic chain runner (deadline aborts), fuzzy ladder + glue + non-destructive in `robust_boolean`, `heal` levels with composed history, `POLICY_VERSION`, `HardenedKernel` decorator; boolean near-coincidence generator + proptests | KH-01, BREP-04, BREP-10 | P1 | Review | 1. On the BREP-04 boolean corpus plus 100 generated near-coincident cases (offsets 0…1e-4 mm, tangent cylinders, coplanar faces), `robust_boolean` gives valid `Ok` on ≥ 95 % of the generated cases with zero invalid `Ok`. Baseline raw OCCT and the improvement are recorded in `kernel-metrics.md`. 2. `exact_only` reproduces raw OCCT bit-identically. 3. Determinism: identical `.brep` bytes and `RobustReport` across two runs and across 1 vs 8 rayon threads. 4. A deadline expiry yields `Timeout` and never a later strategy's result (test with an injected slow strategy). 5. Healing history composition: a test feature filleting after `Unify` resolves names through the composed history (`ShapeHistory` test). 6. Every non-direct acceptance adds a warning. |
| KH-02 | Fillet corpus + harness: `robust::corpus` generators (`prism_vertical_edges` with exact volume oracle, `prism_mixed_edges`, `boolean_stack`, `radius_too_large`), ≥ 50 handmade cases, case schema, process-isolated harness (`kernel-harness` example / `amg-cli kernel harness`), report JSON, `docs/notes/kernel-metrics.md`, baseline files + smoke-subset CI test | KH-01, BREP-06 | P1 | Todo | 1. ≥ 200 cases (≥ 120 synthetic from `seeds.toml`, ≥ 50 handmade committed, the rest harvested/owner under `AMG_TESTDATA_DIR` when present). 2. The exact-volume oracle agrees with a high-resolution tessellation volume within 1e-4 relative on 20 cases (validates the formula). 3. Baseline raw `MakeFillet` success, clean-failure, invalid-Ok and crash counts plus p50/p95 time are recorded in `kernel-metrics.md`. 4. The smoke subset (≈ 40 cases) runs in CI `golden` in < 2 min and fails when a baseline-passing case is broken on purpose. 5. An OCCT access violation in one case (injected) is reported as `Crash` and the run completes. |
| KH-03 | Fillet strategy wrapper + diagnostics: `robust::fillet::fillet` chain (design §4), failing-edge isolation (OCCT report, then ddmin), `max_radius`, `FilletDiagnosis` for SOL-05 | KH-02, KH-17, SOL-05 | P1 | Todo | 1. Full-corpus success rate ≥ baseline + 10 percentage points **and** ≥ 50 % of baseline failures fixed. 2. Zero regressions (every baseline-passing case still passes) and zero invalid `Ok`. 3. 100 % of failures name ≥ 1 failing edge and a `FailureClass`. 4. On `radius_too_large`, `max_radius` is within 2 % of the analytic limit; on other failures `r̂` succeeds and `1.02·r̂` fails in ≥ 95 % of cases (else flagged `approximate`). 5. Proptest: 256 generated parts give valid `Ok` or a classified `Err`, never a panic. 6. Before/after table in `kernel-metrics.md`. |
| KH-04 | Analytic fast-path fillet: edge classification, exact blend surfaces (cylinder/torus/cone/pipe), trimming tools, General Fuse booleans with the fuzzy ladder, tangent chains; runs before the KH-03 chain | KH-03, KH-16, BREP-09 | P1 | Todo | 1. ≥ 98 % success on the analytic-edge subset **without** 3-edge corners (tag `analytic & !corner3`). 2. Exact-oracle cases match volume within 1e-6 relative; all results BRepCheck-valid with `max_tolerance ≤ 1e-5 mm`. 3. p50 time ≤ raw `MakeFillet` p50 on the same subset. 4. Out-of-scope cases fall through unchanged (the chain result is bit-identical to KH-03 alone). 5. Zero regressions on the full corpus. |
| KH-19 | 3-edge corner patches: setback blends, G1 N-sided fill with OCCT `GeomPlate` and our own patch candidate, sewing; decides Open question 2 | KH-04 | P1 | Todo | 1. ≥ 95 % success on the `corner3` subset (convex box corners, mixed convex/concave, unequal radii). 2. G1 deviation along patch boundaries ≤ 0.5° at 50 samples per boundary, G0 ≤ 1e-5 mm. 3. Both patch candidates are measured; the winner and its numbers are recorded in `kernel-metrics.md`, and Open question 2 is closed. 4. Zero regressions on the full corpus. |
| KH-05 | Loft corpus + harness: generators (`loft_translated`/`loft_similar_ruled` exact oracles, `loft_random_stack`, `loft_tangent_chains`, `loft_apex`, `loft_rails`, `loft_centerline`), ≥ 30 handmade classics, owner lofts (`$AMG_TESTDATA_DIR/kernel/loft/owner/`, QUESTIONS #7), failure-taxonomy label per case | KH-01, KH-02, BREP-15 | P1 | Todo | 1. ≥ 150 cases (≥ 100 synthetic, ≥ 30 handmade; owner cases when supplied). 2. Each failing case is labelled with its failure class (preprocessing vs surface vs sewing vs validity). 3. Baselines for raw `ThruSections` and `MakePipeShell` are recorded in `kernel-metrics.md`. 4. Loft smoke subset in CI with its own baseline file. |
| KH-06 | Section preprocessing (`robust::loft::prepare`): validation, orientation, tangent-chain merge, twist-minimising start alignment, arc-length edge-count matching, point sections, user mapping, twist/crossing warnings, `LoftDiagnostics` | KH-05, KH-12, KH-13, KH-16 | P1 | Todo | 1. Run on the corpus with KH-08's builder stubbed by `ThruSections` on the prepared wires, **zero** cases fail in the preprocessing classes (orientation, start vertex/twist, edge count, tangent chains, point sections). 2. Proptests: matched segment counts equal; arc length preserved within 1e-9 relative; user-mapped points correspond exactly; alignment invariant under cyclic relabelling and reversal of inputs. 3. Every `SectionProblem` variant has a fixture with the right section index. 4. `prepare` takes < 5 ms for 6 sections × 32 segments (dialog live preview). |
| KH-08 | Loft builder: `robust::loft::build` (prepare → `loft_surfaces` → faces → caps → sewing → solid → validation incl. self-interference), end conditions from adjacent faces (`CrossBoundaryField` impl), fallback chain (Own → ThruSections → PipeShell), `LoftOutput`, failure capture | KH-06, KH-14, KH-16, KH-17, BREP-09, BREP-10, SOL-01 | P1 | Todo | 1. ≥ 98 % success on the loft corpus excluding rails/centerline cases. 2. 100 % of `Ok` results are BRepCheck-valid; closed sections give closed solids. 3. Sections reproduced within `fit_tol` (1e-5 mm); exact-oracle volumes within 1e-6 relative. 4. G1/G2 end conditions to adjacent planar/cylindrical faces measured on the B-rep: ≤ 1e-6 rad / ≤ 1e-5 relative. 5. `LoftReport` engine recorded; failures are captured as dumps. 6. Typical loft (≤ 6 sections × ≤ 8 edges) builds in < 100 ms (preview budget, S04) and < 300 ms including validation. |
| KH-18 | Loft rails + centerline in the builder: rail intersection and splitting, per-patch Gordon networks, centerline frames + intermediate sections, optional OCCT `GeomFill_Gordon` engine (OCCT ≥ 8.0.0p1) | KH-08, KH-15 | P1 | Todo | 1. ≥ 98 % success on the full loft corpus including rails and centerline cases, with zero invalid `Ok`. 2. Rails reproduced within `fit_tol`; lateral faces watertight (sewing adds no tolerance above 1e-5 mm). 3. A rail missing a section gives `RailMiss` with indices. 4. Centerline lofts keep sections exact and show no twist flip (frame angle continuity ≤ 5° between neighbouring intermediate sections). |
| KH-09 | Upstream OCCT bug reports/patches for isolated issues found by the harnesses | KH-02 | P3 | Todo | Ongoing. Each report links a minimal Draw script or a C++ reproducer and its corpus case id; the list is kept in `kernel-metrics.md`. |
| KH-10 | **Phase 7:** Rust fillet engine, including G2 (curvature-continuous) blends as a continuity option (QUESTIONS #55; placeholder, refine into 1–3-day WPs before Phase 7) | KH-04, KH-19, KH-15 | P3 | Todo | 1. Beats KH-03 + KH-04 + KH-19 success rate on the full corpus with zero regressions. 2. G2 option: on constant and variable radius corpus cases the curvature across both blend boundaries is continuous (max jump < 1e-3 of the local 1/R on 50 samples per boundary), blend volumes match the exact-volume oracle where one exists, and G1 results are unchanged. |
| KH-11 | **Phase 7:** corpus-driven replacement of further OCCT ops (booleans, offset, SSI) with Rust (placeholder; refine before Phase 7) | KH-10 | P3 | Todo | each replacement beats OCCT on its corpus |
| KH-20 | Kernel speed for interactive solid ops: profile and speed up the OCCT boolean behind extrude-cut and the offset behind shell (parallel mode, fuzzy/glue options, local operation on the faces the tool touches, simple-offset path for previews where valid), with no change in results on the existing goldens | SOL-02, SOL-06, KH-17 | P1 | Todo | 1. `budget/sol-extrude-cut` (100-edge profile into a 200-face body, full mode) median < 80 ms; today 121 ms, of which the boolean is about 105 ms. 2. `budget/sol-shell-preview` (100-face body) median < 100 ms; today 369 ms, of which `BrepKernel::shell` is 363 ms. 3. All BREP-04, BREP-07, SOL-02 and SOL-06 goldens still pass with the same tolerances, and `HardenedKernel` results stay deterministic. 4. Before/after numbers recorded in `kernel-metrics.md`, measured on an idle machine. |

Status values: `Todo`, `Refining`, `Ready`, `In progress (agent/branch)`, `Review`, `Done`, `Blocked (reason)`.
KH-01 (Phase 1 background) and KH-07 (`amg-nurbs`, not gated by ADR-002) are specified first and
can start as soon as the spec is `Ready` and their deps are `Done`. Every other KH WP except KH-12..KH-15
touches OCCT through `amg-brep`, and that gate is cleared (2026-10-20).

**Proposed phase placement** (roadmap change requested; see report):
- Phase 1, lane-A background: KH-01, KH-07.
- Phase 2: KH-12, KH-13, KH-16, KH-17, KH-02, KH-03, KH-05.
- Phase 3: KH-14, KH-15, KH-04, KH-19, KH-06, KH-08, KH-18 (the loft builder lands with SOL-11).
- Ongoing from Phase 2: KH-09.
- Phase 7: KH-10, KH-11.

## Testing
- **`amg-nurbs` unit tests:** P&T worked examples (Ex. 2.3 basis, Ex. 9.1 interpolation),
  analytic conics and quadrics, finite-difference derivative checks, and one table-driven
  test per error variant.
- **`amg-nurbs` property tests** (proptest, fixed CI seed, 512 cases each):
  - shape invariance of every knot op;
  - affine invariance (transform poles = transform points);
  - reverse ∘ reverse = id;
  - split + join = id;
  - interpolation reproduces its data;
  - fit error ≤ tol;
  - closest point ≤ sampled minimum;
  - determinism (two runs give identical `to_bits`).
- **Differential oracles:** curvo 0.3.3 (dev-dep) for eval, knot ops and closest point;
  OCCT `Geom_BSpline*` and `GeomFill_Gordon` in `amg-brep` tests (KH-16).
- **Golden corpora:**
  - `testdata/kernel/{fillet,loft}` (committed handmade cases + seed lists);
  - `$AMG_TESTDATA_DIR/kernel/…` (owner/harvested, skipped when absent, through
    `amg_testkit::testdata`);
  - baselines in `tests/golden/kernel/*-baseline.toml`;
  - CI `golden` job runs the smoke subsets.
- **Robustness proptests** (robustness note §7): fillet/boolean/loft give a valid `Ok` or a
  classified `Err`, never a panic or an invalid `Ok`; determinism across thread counts;
  dump → replay round trip.
- **Process isolation:** the harness runs OCCT in worker processes. Crashes are counted, not
  fatal.
- **Fusion comparison:** owner loft cases record how Fusion failed (QUESTIONS #7). KH-08/KH-18
  report per case whether we succeed where Fusion failed.

## Performance targets
| Item | Target | Protects |
|---|---|---|
| `amg-nurbs` cubic curve point / bicubic point + 1st derivs | < 60 ns / < 250 ns | loft preview, Phase 7 fillet engine |
| compat + skin, 6 sections × 8 segments (exact / approximated) | < 2 ms / < 10 ms | loft preview < 100 ms (S04) |
| Gordon 5×5 network | < 20 ms | loft with rails preview |
| `loft::prepare` 6 × 32 segments | < 5 ms | live correspondence lines in the dialog |
| `loft::build` typical (≤ 6 × ≤ 8) | < 100 ms preview, < 300 ms with validation | recompute < 500 ms (vision) |
| Fillet analytic fast path | p50 ≤ raw `MakeFillet` p50 | recompute < 500 ms |
| Strategy chain on success (direct accepted) | ≤ raw op + validation (< 5 % overhead) | recompute |
| `RecordingKernel` in `OnFailure`, successful call | < 1 µs overhead, no I/O | everything |
| Smoke harness (≈ 40 + 40 cases) | < 2 min in CI | developer loop |

## Open questions
1. Owner: any old Fusion files whose lofts failed? Export their section sketches (DXF/STEP) for KH-05.
   **Answered 2026-10-20:** yes, private, supplied later (before KH-05). Section sketches and rails
   (DXF/STEP) plus a note on how Fusion failed go under `$AMG_TESTDATA_DIR/kernel/loft/owner/`
   (S00 Open question 2); not committed.
2. Own N-sided corner patch vs OCCT `GeomPlate` for KH-19 (moved from KH-04). Decided on corpus
   results inside KH-19 (Done-when 3). Not an owner question.
3. **(owner, proposed for QUESTIONS.md)** Automatic fuzzy-tolerance escalation cap: may recompute
   escalate up to **1e-3 mm** on its own, showing a health warning (recommended; far below
   FFF/MSLA resolution)? Or should it stop at 1e-5 mm and require an explicit "try harder"
   above that? Affects KH-17 defaults (`RetryPolicy::recompute`).
   **Resolved 2026-10-20 (owner, QUESTIONS #40):** auto up to 1e-3 mm with a health warning
   (`tol_cap` default 1e-3 mm, as specified).
4. **(architecture, proposed ADR change)** The architecture allow-list keeps `amg-sketch` and
   `amg-subd` from depending on `amg-nurbs`. S09 FRM-06 (SubD→NURBS) should build on
   `amg-nurbs` (ADR-013 "NURBS math shared later by Form"), so `amg-subd → amg-nurbs`
   should be allowed. `amg-sketch → amg-nurbs` would let SK-08 reuse evaluation and fitting,
   but S03 is `Ready` with its own `BSpline2`. Recommendation: allow `subd → nurbs` now and
   leave sketch as is.
   **Resolved 2026-10-20 (owner, QUESTIONS #41, #42):** `amg-subd → amg-nurbs` allowed;
   `amg-sketch` unchanged. Applied in architecture §Crates and the layering test allow-list.
5. Tangent-merge angle threshold for loft sections (provisional `1e3 × Tolerance::angular` =
   1e-6 rad). It may need to be looser (about 1e-4 rad) for DXF-imported sketches. KH-06 measures
   on the corpus and records the value.
6. **Resolved 2026-10-20 (owner, QUESTIONS #27):** the vcpkg baseline bump to a tag shipping
   OCCT 8.0.1 (R15) is scheduled as S00 WP FND-13, before BREP-01. KH-16 and KH-18 can rely on
   the 8.0.1 Gordon engine once FND-13 is Done.

## Refinement log
### 2026-10-20 — S20 refinement (lane A agent)
- **Read:** AGENTS, vision, architecture, ADRs (002 amended, 009, 012, 013, 014), roadmap,
  QUESTIONS (answers binding: #7 owner lofts, #9 testdata hybrid, #19 licence rule), S00, S01,
  S03, S04, S07 Interfaces/requests, S02 Draft (being refined in parallel, so S20 only
  *requests* from it: §Interfaces R1–R17). The final pass was aligned with the parallel S02 draft
  of the same day: its WP numbering (BREP-12 infra, BREP-15 raw loft/pipe, BREP-03 face builders),
  `JobCtx`, `geom::*` types, and the `HardenedKernel` decorator S02 §10 expects. R1–R5 and R7–R13
  are already taken up there; R14–R17 stay open.
- **Build vs reuse (crates.io 2026-10-20), decision: build:**
  - `curvo` 0.3.3: MIT, 99 releases since 2024-04, last 2026-10-19. Exact knot-union loft, no
    end conditions or Gordon, `anyhow` errors, nalgebra in the public API.
  - `truck-geometry` 0.5.0: Apache-2.0, crates.io release 2024-09-20 while the repo stays
    active, cgmath.
  - `monstertruck-geometry` 0.4.1: Apache-2.0, 2026-09-19. Its Gordon uses uniform
    intersection parameters and a degree-1 tensor term, and it panics on bad input.
  - `brepkit-math` 4.2.2: AGPL-3.0-only.
  - curvo stays as a pinned dev-dependency oracle. TiGL (Apache-2.0) is the reference for
    Gordon networks and supplies its `CurveNetworks` test data. Details:
    `docs/notes/nurbs-library.md` §2.
- **OCCT finding:**
  - 8.0.0p1 (2026-06-17) added a reworked `GeomFill_Gordon` / `GeomFill_NetworkSurface`.
  - 8.0.1 (2026-07-30, the current vcpkg port) made fillet reconstruction thread-local and
    improved Gordon and UnifySameDomain/ShapeFix robustness.
  - Request R15: S02 should bump the pin from 8.0.0 to 8.0.1. With 8.0.0 the OCCT Gordon engine
    and oracle are skipped.
- **New design notes:** `docs/notes/nurbs-library.md`, `docs/notes/kernel-robustness.md`.
  `kernel-metrics.md` stays a KH-02 deliverable.
- **New robustness framework WP KH-17:** validation gate, deterministic chains (deadline
  aborts), fuzzy ladder with cap, healing levels with composed history, policy presets,
  `POLICY_VERSION` in the cache salt. It covers the "tolerance escalation / ShapeFix / retry"
  strategies the Draft only mentioned.
- **WP split (IDs kept stable, new IDs from KH-12):**
  - KH-07 split into KH-07 (core, eval, analytic), KH-12 (knot ops), KH-13 (fitting, closest
    point), KH-14 (compatibility, skinning, end conditions) and KH-15 (Gordon, RMF,
    centerline).
  - KH-16 added for amg-nurbs ↔ OCCT conversion and faces.
  - KH-04 split: corner patches → KH-19.
  - KH-08 split: rails/centerline → KH-18.
  - KH-10/11 marked as placeholders to refine before Phase 7.
- **Deps changed:**
  - KH-01 now needs BREP-02 + BREP-12 + BREP-04 (+FND-04) instead of BREP-01, because it needs shapes,
    indices and at least one real op. `.brep` I/O is requested in BREP-02 (R1).
  - KH-03 adds KH-17.
  - KH-05 adds KH-02 (harness reuse).
  - KH-06 now uses KH-12/13/16 instead of SK-06. Sections arrive as wires/faces, and SK-06 is
    only needed by SOL-11.
  - KH-08 adds KH-14/16/17.
- **Design decisions:**
  - one lateral patch per matched segment, with one global v-parametrisation, so shared
    boundaries are bit-identical;
  - shared corner end-derivatives;
  - `fit_tol` 1e-5 mm and ≤ 200 poles for approximate compatibility;
  - end-condition fields through the `CrossBoundaryField` trait, so amg-nurbs stays OCCT-free;
  - exact-volume oracles for fillet and loft generators;
  - a process-isolated harness;
  - the regression baseline is "no baseline-passing case may fail", stronger than a rate.
- **Owner/architecture questions proposed** (not written to QUESTIONS.md by this agent): Open
  questions 3 and 4.
- Status Draft → Refined.

### 2026-11-09 - owner answers moved in
- QUESTIONS #55: G2 fillets are needed; added to Design §7 and KH-10 (scope and Done-when 2). S04 SOL-05 ships G1 and points here.

### 2026-11-17 - KH-01 failure capture landed (lane A)
- **Landed in `amg-brep`:** `pub mod capture` (`info`, `op`, `recorder`, `recording`,
  `replay`, `time`); `tests/capture.rs` with the fault-injecting decorator
  `tests/common/fault.rs`; criterion bench `benches/capture.rs`; example `kernel-replay`
  (`replay [--validate] [--isolated]`, `list`, `recover`, `promote`). New `amg-brep` deps
  `blake3` and `serde_json` are already workspace deps listed in `THIRD_PARTY.md`.
- **Dump format:** `<dir>/<UTC>_<op>_<hash8>/manifest.json` (schema 1, `deny_unknown_fields`,
  20 keys) + `inputs/<n>.brep` (text BRep). Dedup key = blake3 over the op JSON and the input
  bytes; a repeat bumps `count`/`last_utc`. Retention prunes least recently seen dumps until
  `max_dumps` and `max_bytes` both hold (never the dump just written). `WriteAhead` writes to
  `pending/<hash8>-<pid>-<seq>/` holding an OS file lock (`File::lock`), so
  `recover_pending` skips dumps of calls still running in another process.
- **Done-when evidence** (OCCT 8.0.1, `--features occt`):
  1. 19 forced failures, one per `KernelOp` variant (boolean solid+face fuse ->
     `BOPAlgo_AlertBOPNotAllowed`/`NotDone`; injected `OpFailed` in `mirrored`/`section`;
     argument errors; self-intersecting revol -> `OcctException`), all schema-1 valid.
     **Deviation:** no fillet op exists yet (BREP-06), so the "too-large fillet radius" case
     is not possible; BREP-06 adds the `Fillet` variant and that case.
  2. 19/19 dumps `Reproduced` when replayed in a child process (`replay_in_child`).
  3. 20 parts (11 `testdata/brep` fixtures + 9 boolean/sweep results) + 200 generated
     boolean parts: all 6 513 face/edge fingerprints survive text `.brep` write -> read.
     **Deviation:** no STEP reader exists yet (BREP-13) and `testdata/` has no STEP files,
     so the fixtures stand in for "20 STEP parts". A forced re-index (ids rotated in the
     manifest) restores the ids by fingerprint and reports `reindexed_by_fingerprint`; a
     fingerprint without a unique match is `CaptureError::Unmatched`.
  4. A child process aborting inside `make_sphere` (`WriteAhead`) leaves `pending/`;
     `recover_pending` turns it into a `Crash` dump that reproduces as `Crash` in a child and
     replays as `NowSucceeds { valid: true }` on the real kernel. A pending dump with a held
     lock is left alone.
  5. The same failing boolean twice = 1 dump, `count = 2` (also across a recorder restart);
     `max_dumps = 3` and `max_bytes` pruning tested.
  6. Successful calls create no directory or file (100 000 guarded calls + 1 000 real ops).
     Criterion: `budget/capture-on-failure-overhead` 36.6 ns vs `capture/direct-ok` 5.3 ns
     (about 31 ns overhead, budget 1 us); `make_box` 29.1 us direct vs 29.7 us recording.
- **Interface deviations from the sketch above:** `KernelOp` covers the ops that exist
  (primitives, planar faces/polygon, face, wire, prism, revol, mirrored, boolean, split,
  section, interference, plane_section, project_edges, tessellate); read-only queries are
  forwarded without capture. `SubShapeGeom` is KH-01's own (class, measure, centroid, bbox)
  until BREP-05 provides S02's; `EdgeRecord.geom` and the `faulty` fingerprints are
  `Option`. `FailureClass::InvalidInput` added (argument errors). `ReplayOpts` has only
  `validate` (`robust` comes with KH-17); `Attempt` is a placeholder KH-17 replaces with
  `robust::Attempt` keeping the field names. Added `replay_in_child`, `child_report_line`,
  `read_manifest`, `Manifest`, `DumpContext`, `FailureRecorder::dumps`,
  `RecordingKernel::with_tolerance`. `CaptureError` adds `Unmatched`, `Child`, `InvalidId`.
- **Leftovers:** `amg-cli kernel replay|promote` subcommands (lane E, thin wrappers over
  `capture::{replay, replay_in_child, promote}`; see Interface changes); the fillet case of
  Done-when 1 with BREP-06; the STEP part set of Done-when 3 once BREP-13 reads STEP; wiring
  `RecordingKernel` + `recover_pending` at app startup (app/lane D).

### 2026-11-18 - KH-07 amg-nurbs core landed (lane A)
- **Landed** in `crates/amg-nurbs` (deps: `amg-core`, `serde`, `thiserror`, `smallvec` 1.16.3;
  dev: `curvo =0.3.3`, `criterion`, `proptest`, `approx`, `serde_json`): sealed `CtrlPt`
  (`Point2`/`Point3`), `KnotVector` (validation, `bezier`, `clamped_uniform`, knots/mults both
  ways, `domain`, A2.1 `find_span`, `multiplicity`, `is_clamped`, `reparametrized`, `greville`),
  `basis::{basis_funs, ders_basis_funs}` (A2.2/A2.3), `Grid`, `NurbsCurve` (eval A4.1/A4.2,
  `derivs` A3.2/A4.2 up to order 4, start/end, `is_closed`, `reversed`, `reparametrized`;
  3D: `transformed`, `bbox`, `curvature`, `torsion`, `planarity`), `NurbsSurface` (eval A4.3,
  `derivs` A3.6/A4.4 up to order 2, `normal`, `iso_curve`, `boundary`, `reversed`, `transposed`,
  `transformed`, `bbox`), `analytic` (line, A7.1 circle arc, ellipse arc, plane patch, extrusion,
  ruled with equal knots, A8.1 revolution, cylinder, cone, sphere, torus), `NurbsError` +
  `Diagnostic` hints, `CurvePoint`/`Continuity`/`SurfacePoint` types.
- **Measured** (tests print them): Ex. 2.3 basis exact to 1e-15; partition of unity 1.1e-15;
  circle radius error 5.9e-16·r; sphere / torus ≤ 1.7e-15·R; FD derivative check ≤ 9.5e-8
  relative; circle tangent/curvature/torsion 5.2e-15; curvo on 1000 curves + 1000 surfaces
  (degree 1–7, half rational): points ≤ 6.2e-16·max(1, D), first derivatives ≤ 1e-14 relative.
  Benchmarks (criterion `benches/eval.rs`, asserted under `AMG_STRICT_TIMING=1`; shared machine
  at 46–60 % CPU load from other workers): cubic point 27–30 ns (< 60), bicubic point + first
  derivatives 185–238 ns (< 250), rational sphere point + normal 234–274 ns (< 400). Re-measure
  on an idle machine; bicubic + d1 has the least margin.
- **Decisions / deviations:**
  - Evaluation clamps parameters into the domain (no `OutOfDomain` on the hot path);
    `derivs` clamps the order (curve 4, surface 2: `MAX_CURVE_DERIV`, `MAX_SURFACE_DERIV`).
  - Clamped means end multiplicity exactly `degree + 1`; interior multiplicity ≤ `degree`
    (`InvalidKnots` otherwise). Weight/pole length mismatches and grid size mismatches are
    `Degenerate(..)` (no dedicated variant). `KnotVector::new` also rejects an empty range.
  - Arcs (and so cylinders, cones, revolutions, spheres, tori) have the knot domain
    `[0, |sweep|]`; lines and plane patches `[0, 1]`; cylinder/cone height v ∈ `[0, 1]`.
    Revolution: u = rotation, v = profile; quadric normals point outward. Torus requires
    `0 < minor < major`.
  - `Deserialize` for curves, surfaces and grids is hand-written so it re-validates (same
    serde shape as the derive).
  - `NurbsSurface::normal` steps toward the domain centre at collapsed edges (sphere poles).
  - Space-curve check of Done-when 3 uses the twisted cubic: a circular helix has no exact
    NURBS form (an interpolated helix arrives with KH-13).
  - Evaluators use stack buffers sized 4 / 8 / 26 by degree (`basis::with_cap!`).
- **Left for later WPs (signatures in Interfaces, not yet implemented):** KH-12 curve/surface
  knot ops (`insert_knot`, `refine_knots`, `remove_knot`, `elevate_degree`, `split`,
  `segment`, `join`, `bezier_segments`, `continuity_at`, `KnotVector::union`/`missing_from`);
  KH-13 `arc_length`, `param_at_length`, `closest_point` (curve and surface). `ruled` on
  non-matching knots returns `Incompatible` until KH-12/KH-14 add compatibility.
  `nalgebra` not needed yet (no dense solves in KH-07).

### 2026-11-26 - KH-20 added (owner decision)
- SOL-02 Done-when 6 (extrude-cut 121 ms vs 80 ms) and SOL-06 Done-when 8 (shell preview 369 ms vs 100 ms) missed only inside OCCT (`BRepAlgoAPI_Cut` about 105 ms, `BRepOffset_MakeOffset` 363 ms). The owner accepted both misses and closed the two WPs; the budgets move to the new KH-20. Measurements are in the SOL-02 and SOL-06 entries of S04 and in the handoff reports.

### 2026-11-28 - KH-17 robustness framework landed (lane A)
- **Landed in `amg-brep`:** `pub mod robust` with `POLICY_VERSION = 1` (moved from `kernel.rs`;
  `amg_brep::POLICY_VERSION` re-exports it, so `kernel_version()` is now `…;policy-1`),
  `RetryPolicy` and its four presets (robustness note §2.6 table), `HealLevel`, `Validation`,
  `StrategyId`, `Rejection`, `AttemptOutcome`, `Attempt`, `RobustReport`, `RobustOutput`,
  `RobustCtx`, `RobustError` (+ `Diagnostic`), `validate`, `robust_boolean`, `robust_split`,
  `heal`, `HardenedKernel`; submodules `robust::chain` (the chain runner) and `robust::corpus`
  (the near-coincidence generator). Tests `tests/robust.rs` (9) plus 15 unit tests, the
  `HardenedKernel` forwarding test in `tests/api_contract.rs`, bench `benches/robust.rs`,
  `docs/notes/kernel-metrics.md` (created; notes README status updated). No new dependency.
- **Chain rules as built:** strategies run in a fixed order through `chain::Chain`. Each
  attempt gets a `JobCtx` whose token is a child of the caller's. A process-wide deadline
  thread cancels it when `attempt_deadline` passes. An expiry (or an attempt that returned after
  the deadline) aborts the chain with `Timeout`, and later strategies are never built. Argument
  errors, `BackendUnavailable` and `Cancelled` stop the chain at once. `max_attempts` counts
  the direct attempt. The final error carries the direct attempt's class.
- **`robust_boolean` order:**
  1. `Direct` (the caller's `BoolOpts`).
  2. `Fuzzy { rung }` for each ladder value above `base.fuzzy` and at most
     `min(tol_cap, 1e-3 × shortest input edge)`.
  3. From the second rung on, `Glue { rung }` (`Glue::Shift`) when the operands have
     coincident planar or cylindrical faces within the rung. KH-17 detects this itself from
     `Shape::surface` plus face boxes, as R6 allowed.
  4. `Heal(level)` for `Light ..= policy.heal`: operands healed at that level, the op with the
     base options, the healing history composed per operand (`robust::compose_operands`).

  Gate per attempt:
  - `require_solid` when every operand is a solid body;
  - tolerance cap `max(input tolerance, min(cap, 10 × fuzzy))`;
  - volume bounds for solid operands (`V(∪) ∈ [max Vᵢ, ΣV]`, `V(A∖B) ≥ V(A) − V(B)`,
    `V(∩) ≤ min`), 1e-3 relative slack plus `2 × fuzzy × operand area`.

  Every non-direct acceptance adds a report warning; a rung above 1e-5 mm adds a second one.
- **Done-when evidence** (OCCT 8.0.1, `--features occt`; tables in `kernel-metrics.md`):
  1. Met. BREP-04 corpus, 131 result cases: raw and robust both 131 valid. 100 generated
     near-coincident cases (10 families, offsets `0…±1e-4` mm, tilts up to 1e-3 rad, analytic
     volume oracles):
     - raw OCCT: 99 "valid" `Ok`, 1 invalid `Ok` (`BadOrientationOfSubshape`) and 3 valid
       `Ok`s with tolerances of 1.4e-3, 3.3e-2 and 5.4e-2 mm;
     - `robust_boolean` (`recompute`): **99 valid `Ok` (99 %), 0 invalid `Ok`, 0 above the
       cap, 1 classified `ToleranceTooLarge` error**; 3 non-direct acceptances;
     - usable results 96 → 99.

     Proptest `near_coincident_booleans_are_valid_or_classified` (48 cases): valid `Ok` or a
     classified `Err`.
  2. Met. `exact_only` is byte-identical (text `.brep`, history, warnings) to the raw op on all
     136 corpus cases and 97 of the 100 generated cases. The other 3 are the raw results above
     the 1e-3 mm cap, which the gate refuses (the test asserts these are gate rejections only).
  3. Met. 119 inputs (3 of them non-direct) give identical bytes, history and
     `RobustReport::without_timing()` over 3 runs on 1- and 8-thread rayon pools.
  4. Met. A real 1030-face cut (about 10 s) under a 300 ms deadline gives `Timeout` after
     341 ms (through `Chain`; the succeeding strategy behind it never runs) and after 568 ms
     (through `robust_boolean`). `HardenedKernel` reports it as `OpFailed`. A unit test with a
     cooperative slow closure covers the same without OCCT.
  5. Met (`healing_history_composes_through_unify_and_a_fillet`). Two fused boxes (10 faces,
     split top and front) → `robust::heal(Unify)` (attempts `Heal(Light)`, `UnifySameDomain`,
     6 faces) → a 2 mm fillet on the merged front-top edge. The composed history maps both top
     pieces to the trimmed top face (160 mm², +z) and both edge pieces to the
     `FilletBlend` face.
  6. Met (`non_direct_acceptances_warn`, plus an assertion over both corpora): with an injected
     exact-boolean failure, `Fuzzy { rung: 0 }` is accepted with 1 warning (2 for a 1e-4 rung,
     "above 1e-5 mm"). `HardenedKernel` adds `KernelWarning::Strategy { name: "fuzzy", .. }` for
     `boolean` and `split`. A direct acceptance adds none.
- **Performance** (S20 target "direct accepted ≤ raw + validation, < 5 %"; criterion): blind
  hole raw 1.66 ms, raw + gate 2.87 ms, robust 2.93 ms (+2.2 %); 202-face plate cut 409 / 687 /
  704 ms (+2.4 %). A per-attempt watchdog thread and an eager operand-area sum had cost +15.7 % /
  +7.3 %; both were removed.
- **Deviations from the Interfaces sketch:**
  - `RobustError` gains `error: Option<Box<KernelError>>`, the direct attempt's kernel error
    (boxed to keep `Result` small), so `HardenedKernel` returns the kernel's own error.
  - `StrategyId::Custom(u8)` added for injected strategies.
  - Additive helpers: `StrategyId::name` + `Display`, `Rejection::class` + `Display`,
    `HealLevel::ladder`, `Validation::with_cap`, `RobustCtx::new`,
    `RobustReport::{without_timing, dump_attempts}`, `FUZZY_WARN_ABOVE`,
    `robust::{identity_history, compose_operands}`, `HardenedKernel::{with_tolerance, inner,
    policy}`, and `robust_split` (the decorator also routes `split`).
  - `capture::Attempt` stays as the dump form (manifest schema 1 unchanged), with
    `From<&robust::Attempt>`, instead of being replaced.
  - `Validation::require_solid` relies on BRepCheck's shell check for closure when `brepcheck`
    is on. A topology-only free-edge scan miscounted the line-contact edge OCCT embeds in a face
    (corpus `tangent_cyl_on_box_fuse`); the scan only runs with `brepcheck` off. An empty
    result passes.
  - `HealLevel::Full` is `Unify` + `sew` at `tol_cap` with `make_solid` (sewing's solid step
    already orients through `ShapeFix_Solid::SolidFromShell`); no separate `ShapeFix_Solid`.
  - Attempts run with `JobCtx::with_token(child)`, so there is no progress inside an attempt;
    the chain reports progress per attempt on the caller's context. `amg-jobs` has no public
    way to derive a context with a child token that keeps the job's progress.
  - `HardenedKernel::fillet` forwards unchanged until KH-03's `robust::fillet::fillet`. Not
    wired into `amg-features`; it is opt-in.
- **Findings:** OCCT 8.0.1 already handles plain near-coincidence. Of the 10 families, only
  shifted coplanar copies (an invalid orientation) and slightly skewed coaxial cylinders
  (results with tolerances up to 5.4e-2 mm that BRepCheck accepts) go wrong. So the gate's
  tolerance cap matters as much as the ladder. `Full` healing does not recover the one
  remaining case.
- **Leftovers:** KH-03 routes `HardenedKernel::fillet`; wiring `HardenedKernel` into the
  recompute context (later WP); harvested tracker shapes for the corpus (BREP-04 leftover);
  progress inside attempts (an `amg-jobs` addition, see Interface changes); a corpus case
  where only the `Heal` strategies succeed (none of the 236 cases needs them, so that path is
  covered by `compose_operands` unit tests and the attempt log of a failing case).

## Interface changes

### 2026-10-20: requests from the wave-2 refinements (S02, S05, S10, S11, S12, S16, S17, S20)

Additive requests recorded by the integration pass; the owning lane confirms or adjusts them when it implements the affected WP.

- **From S02 (B-rep kernel).** KH-01 → BREP-12 and KH-05 → BREP-15 are in the WP table. S02
  covers R3, R5, R7, R8 and `max_tolerance`; S20 records R1–R13 as covered. Failure capture stays
  in S20 (S02 removed its own hook). KH-07/KH-08 add `impl From<&amg_nurbs::…> for
  amg_brep::geom::BSplineSurfaceData` inside `amg-brep`. On OCCT 8.0.0, text `.brep` reads take
  the global lock (fine for replay; QUESTIONS #27). Open S20 requests R14–R17 are recorded under
  S02 Interface changes.

### 2026-11-17: KH-01 -> lane E (`amg-cli`)
- Request: `amg-cli kernel replay|promote` as thin wrappers over `amg_brep::capture::{replay,
  replay_in_child, promote, FailureRecorder::recover_pending}`. Until then the `amg-brep`
  example `kernel-replay` provides the same commands. No code in lane E was changed.

### 2026-11-28: KH-17 -> S02 (`amg-brep` shared API, lane A) and `amg-jobs`
- `KernelWarning::Strategy { name: &'static str, detail: String }` added (R17, additive; no
  exhaustive match on `KernelWarning` exists outside `amg-brep`). Recorded in S02's Interface
  changes.
- `POLICY_VERSION` now lives in `amg_brep::robust` (value 1); `amg_brep::POLICY_VERSION` is a
  re-export, so `kernel_version()` reads `…;policy-1` and S01 caches salted with it are
  invalidated once.
- Proposed for `amg-jobs` (not changed): `JobCtx::with_child_token(&self) -> (JobCtx,
  CancelToken)`, a context that keeps the job's progress range but can be cancelled on its own,
  so robust attempts can report progress inside OCCT ops.
