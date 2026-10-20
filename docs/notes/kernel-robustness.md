# Kernel robustness: failure capture, strategies, corpora — design note

| | |
|---|---|
| **Spec** | [S20](../specs/S20-kernel-hardening.md) (KH-01..KH-06, KH-08, KH-16..KH-19) |
| **Decides** | failure-dump format and replay; the robustness framework (retry policy, validation gate, tolerance escalation, healing); fillet and loft strategy chains; corpus layout, oracles and harness metrics |
| **Produced by** | S20 refinement (2026-10-20) |
| **Needed before** | KH-01 (§1), KH-17 (§2), KH-02/KH-05 (§5, §6) |
| **Status** | Written. Numbers marked *provisional* are re-tuned by KH-02/KH-05 measurements and recorded in `kernel-metrics.md` |

The `amg-brep` crate is shared with S02. Everything here lives in two `amg-brep` modules
owned by S20: `amg_brep::capture` and `amg_brep::robust` (with submodules `fillet`, `loft`,
`corpus`). It builds on S02's `BrepKernel` trait and only *requests* additions from S02 (S20
§Interfaces, "Expected from S02").

## 1. Failure capture and replay (KH-01)

### 1.1 What is recorded
Every kernel call made through `RecordingKernel<K>` (a decorator that implements `BrepKernel`
by forwarding) or through `robust::*` can be captured as a **dump**:
```
<capture dir>/                       # default %LOCALAPPDATA%\Amalgam\kernel-failures
  20261020T140311Z_fillet_3f2a9c1e/  # UTC time, op kind, first 8 hex of the dump hash
    manifest.json
    inputs/0.brep  inputs/1.brep …   # BRepTools text format, triangulation stripped
```
`manifest.json` (`schema = 1`) holds:
- `amalgam_version`, `git_sha`, `occt_version`, `kernel_version` (S02), `policy_version`
  (`robust::POLICY_VERSION`), `os`, `created_utc`;
- `op`: the serialized `KernelOp` (§1.2) whose shape arguments are `InputRef(n)` indices into
  `inputs/`;
- `tolerance` (`amg_core::Tolerance`) and the `RetryPolicy` in force;
- `error`: `{ class: FailureClass, message, occt_type, faulty: [sub-shape ids] }`;
- `attempts`: the strategy log (§2.3), so a dump shows what was already tried;
- `mode` (`on_failure` / `write_ahead`), `duration_ms`, `count` (dedup counter);
- optional `context`: feature kind and the document's `PersistentId` (no file path, no names).

The **dump hash** is blake3 over the canonical op JSON plus the input `.brep` bytes. When a
dump with the same hash exists, `count` is incremented instead of writing a new directory.

### 1.2 `KernelOp`
`KernelOp` is a serde enum with one variant per `BrepKernel` operation, holding S02's
parameter types (which must be `Clone + Debug + Serialize + Deserialize`, S02 request R3).
It is `#[non_exhaustive]`. **Rule:** every WP (S02 or S20) that adds a kernel operation adds
its `KernelOp` variant and replay arm in the same PR, with a one-line round-trip test.

### 1.3 Modes
| Mode | Cost on success | Catches |
|---|---|---|
| `Off` | none | nothing |
| `OnFailure` (owner default) | none: shape handles are refcount clones held until the call returns | `Err`, invalid results rejected by the validation gate, panics caught by `amg-jobs` |
| `WriteAhead` (debug) | `.brep` write per call (~1–20 ms) | additionally hard crashes and hangs: inputs go to `pending/<hash>/` first and are deleted on return; a later start moves leftovers to the capture dir with `error.class = Crash` |

Configuration: `CaptureConfig { mode, dir, max_bytes: 1 GiB, max_dumps: 1000 }`, overridable
by `AMG_KERNEL_CAPTURE=off|on_failure|write_ahead` and `AMG_KERNEL_CAPTURE_DIR`. Retention
prunes the oldest dumps first. Dumps never leave the machine.

### 1.4 Replay
`capture::replay(dir, kernel, &ReplayOpts)` reloads the inputs, re-runs the op (raw by
default, or through `robust` with `--robust`), and classifies the result:
- `Reproduced` (same `FailureClass`);
- `DifferentFailure`;
- `NowSucceeds` (with validity).

`amg-cli kernel replay <dir>… [--robust] [--json]` is a thin clap wrapper (lane E change,
flagged in the S20 interface requests). Until lane E wires it, KH-01 ships it as the
`amg-brep` example `kernel-replay`.
`amg-cli kernel promote <dump> --corpus <dir> --id <name>` turns a dump into a corpus case
(§5.2) with `expect` taken from the replay.

**Index stability.** Edges and faces are referenced by S02's in-shape indices
(`TopTools_IndexedMapOfShape` order). Replay needs those indices to survive
`BRepTools::Write` → `Read`. KH-01 proves it on 20 STEP parts and 200 generated parts. As a
guard, every `EdgeId` in a dump also stores a fingerprint (midpoint, length, curve type). If
the reloaded index map disagrees, replay re-matches by fingerprint and warns.

## 2. Robustness framework (KH-17)

### 2.1 Principles
1. **Valid or loud** (vision: "robust over clever"). A strategy's result is accepted only if
   it passes the **validation gate** (§2.2). An invalid `Ok` from OCCT counts as a failure.
2. **Deterministic** (vision §Principles):
   - Strategies run in a fixed order with fixed parameters.
   - A strategy is skipped only for deterministic reasons (input class, an earlier result).
   - Wall-clock time never selects which geometry wins. A per-attempt deadline that expires
     **aborts the whole chain** with `FailureClass::Timeout` instead of falling through. So a
     slow machine fails where a fast one succeeds, but it never produces *different* geometry.
     Effort limits inside strategies are counts (bisection steps, edges), not durations.
3. **Never mutate authored geometry silently.** Healing works on copies inside the op. When
   a non-direct strategy won, the result carries `RobustReport.warnings`. `amg-features`
   surfaces them as feature health `Warning` (S01).
4. **Cacheable.** `POLICY_VERSION` changes whenever a default chain or ladder changes. S02's
   `kernel_version()` includes it, so S01's `ENGINE_SALT` invalidates caches.

### 2.2 Validation gate
`robust::validate(kernel, shape, &Validation)` checks, in this order:
1. `BRepCheck_Analyzer` valid (S02 `check`).
2. For solid-producing ops: closed shell(s), a solid with positive volume, and no free edges.
3. `max_tolerance(shape) ≤ Validation::max_tolerance`. This uses
   `ShapeAnalysis_ShapeTolerance`, the max over vertices, edges and faces. Fuzzy booleans and
   sewing inflate sub-shape tolerances, which is where "valid but sloppy" hides.
4. Optional op-specific bounds. A fillet's volume change lies between 0 and the sum of the
   per-edge prism bounds `A_max · L`. A boolean result volume obeys `V(A∪B) ≤ V(A) + V(B)`,
   and so on.
5. Optional `BOPAlgo_ArgumentAnalyzer` self-interference check (loft results; S02 request R5).

### 2.3 Attempts and reports
Each attempt logs `{ strategy: StrategyId, outcome, millis }` with outcome `Accepted`,
`Failed(FailureClass, msg)` or `Rejected(Rejection)`. `RobustReport` collects the attempts,
the accepted strategy, the result's max tolerance and the warnings. The report is part of
`RobustOutput`, the failure dump, and the harness JSON.

### 2.4 Tolerance escalation (fuzzy booleans)
A boolean runs first exact (`fuzzy = 0`), then along a **fuzzy ladder**. The default is
`[1e-5, 1e-4, 1e-3] mm` (*provisional*; owner question on the cap). The ladder is capped at
`min(tol_cap, 1e-3 · smallest input edge length)`.
- Each rung uses OCCT `BOPAlgo_Options::SetFuzzyValue`, with `SetNonDestructive(true)`.
  Rung ≥ 2 also tries `SetGlue(BOPAlgo_GlueShift)` when the inputs share coincident faces
  (detected by S02's coplanarity query).
- The accepted result's `max_tolerance` must stay `≤ 10 × rung`.
- Any rung above `1e-5 mm` adds a warning ("built with 1e-4 mm fuzzy tolerance").
- `RetryPolicy::exact_only()` disables the ladder. Scripts can use it, and Python exposes it.

### 2.5 Healing
`HealLevel` = `None` < `Light` < `Unify` < `Full`. Healing runs only **after** the direct
attempt failed, on copies of the inputs, then the op is retried:
- `Light` — `ShapeFix_Shape` at precision `Tolerance::linear` with `MaxTolerance = tol_cap`
  (fixes small edges, wire order, missing pcurves, face orientation);
- `Unify` — `Light` + `ShapeUpgrade_UnifySameDomain` (merge coplanar/co-cylindrical faces and
  collinear edges, history kept);
- `Full` — `Unify` + re-sewing (`BRepBuilderAPI_Sewing` at `tol_cap`) + `ShapeFix_Solid`.

Healing changes topology, so the strategy composes the healing history with the op history
(S02 request R8, `ShapeHistory::compose`). Persistent naming then still resolves through it.

### 2.6 Retry policy presets
| Preset | Used by | Ladder | Heal | Max attempts | Attempt deadline |
|---|---|---|---|---|---|
| `interactive()` | dialog preview | exact + 1e-5 | `Light` | 4 | 2 s |
| `recompute()` (default) | timeline recompute | full ladder | `Unify` | 12 | 20 s |
| `exhaustive()` | harness, replay `--robust`, "try harder" button | full ladder | `Full` | 32 | 60 s |
| `exact_only()` | scripts asking for raw OCCT | none | `None` | 1 | 60 s |

Preview and recompute can pick different strategies. That is allowed because preview results
are never cached as final geometry.

## 3. Fillet strategies (KH-03, KH-04, KH-19)

### 3.1 Strategy chain (KH-03), fixed order
1. **Direct**: `BRepFilletAPI_MakeFillet` with S02 defaults.
2. **Shape variant**: retry with `ChFi3d_QuasiAngular`, then `ChFi3d_Polynomial` (default is
   `ChFi3d_Rational`), plus looser approximation parameters (`SetParams`: `TolApp3d = 1e-5`,
   `Fleche = 1e-5`) (S02 request R7).
3. **Unify**: `ShapeUpgrade_UnifySameDomain` on the input, then remap the requested edges
   through its history. Fixes fillets that fail on split faces (common after booleans).
4. **Heal** (`Light`) and retry.
5. **Regroup by chain**: fillet tangent-continuous chains one chain at a time, in
   deterministic order (longest chain first, then lowest first-edge index).
6. **Regroup by radius**: largest radius first, one radius group per call.
7. **Per-edge sequential**: one edge per call, largest radius first. Each step remaps the
   remaining edges through the step's history.
8. **Isolate**: when everything fails, find the failing edge set (§3.2). Return
   `FilletFailure` with diagnostics instead of a shape.

Every step's output goes through the validation gate, with volume bounds from §2.2(4).

### 3.2 Diagnostics
- **Failing edges.** Use OCCT's own report first (`NbFaultyContours`, `FaultyContour`,
  `StripeStatus`, `NbFaultyVertices`). Otherwise per-edge probing: each edge alone, then
  delta debugging (ddmin) over the set. Capped at `2·n` kernel calls.
- **Max workable radius** per failing edge: bisection on `[0, r_requested]` with 12 steps
  (relative precision 2.4e-4). The result `r̂` must succeed and `1.02·r̂` must fail,
  otherwise it is reported as "approximate". It runs as a cancellable background job,
  bounded by 12 calls per edge and at most 8 edges.
- `FilletDiagnosis { failing: Vec<FailingEdge { edge, class, max_radius: Option<f64> }>, attempts }`
  feeds SOL-05's dialog highlight.

### 3.3 Analytic fast path (KH-04)
Scope: constant-radius fillets on edges whose two faces are planes, cylinders or cones, where
the blend is exactly a cylinder, torus or cone patch (or a pipe of a circle along the spine
for general plane/cylinder pairs), and the edge set has **no vertex where ≥ 3 filleted edges
meet** (KH-19 handles those).
1. Classify each edge with S02 geometry queries (surface type, axis, dihedral angle,
   convexity, straight or circular spine).
2. Build the blend surface exactly:
   - plane/plane with a straight spine → cylinder;
   - plane/plane with a circular spine → torus;
   - plane/cylinder (coaxial) → torus;
   - plane/cone → cone or torus;
   - otherwise use a pipe (`BRepOffsetAPI_MakePipe`) of the rolling-ball circle along the
     analytic spine curve.
3. Make the trimming tool:
   - convex edge: a solid "wedge minus blend" bounded by the two contact lines;
   - concave edge: the blend's material wedge.
4. Combine with the **modern General Fuse booleans** (`BRepAlgoAPI_Cut` / `Fuse` with the
   §2.4 ladder), not the old `TopOpeBRep` code that `ChFi3d` uses internally.
5. At vertices where exactly two filleted edges meet tangentially (chains) or where one meets
   a non-filleted face, the boolean resolves the end cap by itself.

Anything outside scope falls back to the §3.1 chain. The fast path sits **in front of**
step 1 of the chain.

### 3.4 Corner patches (KH-19)
At vertices where 3 filleted edges meet (convex box corner and its generalisations):
- Setback the three blends by the setback distance (default `r`, Fusion "setback" option).
- Fill the 3-sided hole with a G1 patch. The first candidate is OCCT `GeomPlate_BuildPlateSurface`
  with tangency constraints on the three boundaries. The alternative is our own N-sided patch
  (Gregory/Coons-type triangle split into 3 quads via a central point, built in `amg-nurbs`).
- Open question 2 (spec) decides which to keep, on corpus G1 deviation and success rate.
- Sew blends and patch.

## 4. Loft pipeline (KH-06, KH-08, KH-18)

Stages (`robust::loft`):
1. **Extract** each section wire as a chain of `amg_nurbs::NurbsCurve3` (KH-16 conversion),
   or a point.
2. **Preprocess** (KH-06, pure Rust over the chains):
   - **Validate** closed/open consistency (error `MixedClosedOpen { section }`), planarity
     report, self-intersection of each section (2D test in its plane when planar), zero-length
     segments.
   - **Orientation:** closed planar sections are oriented CCW about a common reference
     direction (the first section normal, or the centerline tangent). Open sections are
     oriented so their start points are closest in sequence.
   - **Tangent-chain merge:** consecutive segments meeting G1 (angle ≤ `Tolerance::angular ×
     1e3`, *provisional*) are joined into one curve (`NurbsCurve::join`). This gives fewer,
     cleaner lateral faces.
   - **Start-vertex alignment:** for closed sections, choose the cyclic shift and direction
     of section `s+1` that minimises total twist. Twist = Σ over matched vertex pairs of the
     angle between the radial vectors from the section centroids, plus a corresponding-point
     distance term. This is O(n²) per section pair, with n ≤ 64 vertices. Ties go to the
     lowest shift.
   - **Edge-count matching:** sections with fewer segments are split at the
     **arc-length-proportional** positions of the reference section's vertices. The
     reference is the section with the most segments. A vertex is never split twice, and an
     existing vertex within 2 % arc length is reused.
   - **Point sections** (apex) are allowed only at the ends. They become degenerate
     compatible "curves" with optional `PointTangent`.
   - **User mapping** (`LoftSpec::mapping`): user-fixed correspondences (point on section `a`
     ↔ point on section `b`) override alignment. They are hard split points that matching
     honours.
   - **Twist warning:** if the minimal twist exceeds 90° between neighbours, or any
     correspondence line crosses another in projection.
   - Output: `PreparedSections` (matched chains) + `LoftDiagnostics` (correspondence lines,
     twist per gap, warnings).
3. **Surface construction** (`amg_nurbs::skin::loft_surfaces`, or Gordon when rails exist,
   or centerline): one patch per matched segment index.
4. **Topology:**
   - Lateral faces come from the patches (KH-16 `make_face`).
   - Caps are planar faces from the first and last iso-curves when the sections are planar.
     Otherwise use a `BRepFill_Filling`/plate face, or reuse the original section face when
     the section came from a face.
   - Sew at `max(fit_tol, Tolerance::linear)`. Make the solid when sections are closed.
   - Run the validation gate including self-interference (§2.2(5)).
5. **Fallback chain** (`LoftEngine` recorded in `LoftReport`):
   1. own builder;
   2. OCCT `BRepOffsetAPI_ThruSections` on the **preprocessed** wires (smoothing on, ruled
      if the user chose ruled);
   3. OCCT `BRepOffsetAPI_MakePipeShell` with all sections along the centerline, or along a
      spine through the section centroids;
   4. (if BREP-01 pins OCCT ≥ 8.0.0p1, rails only) OCCT `GeomFill_Gordon` as a surface
      fallback for step 1's Gordon.

   The first valid result wins. Preprocessing failures are reported directly. They don't fall
   through, because every engine would fail on the same bad input.
6. **Diagnostics for the dialog** (SOL-11 renders them): correspondence lines, the twist
   warning, the offending section index, the engine used, the max section deviation, and
   self-interference points.

**Rails (KH-18):**
- Each rail must intersect every section within `net_tol`.
- Sections are split at rail intersection points (hard split points in matching).
- Every patch between consecutive split lines is a Gordon surface. Its guides are the bounding
  rails, plus skinned v-iso-curves at split lines that have no rail, so the network is
  complete.
- Shared boundaries between patches are the same guide curve, so faces stay watertight.

**Centerline (KH-18):** RMF frames + intermediate sections (nurbs note §7).

## 5. Corpora (KH-02, KH-05)

### 5.1 Layout and licensing
```
testdata/kernel/fillet/{handmade,synthetic}/…    # committed (LFS for *.brep; S00 request)
testdata/kernel/loft/{handmade,synthetic,tigl}/…
$AMG_TESTDATA_DIR/kernel/fillet/{owner,harvested}/…   # private / unclear licence, sha256 manifest
$AMG_TESTDATA_DIR/kernel/loft/{owner,harvested}/…     # owner's Fusion lofts (QUESTIONS #7)
```
Licensing follows the S07 rule (QUESTIONS #19):
- Commit only our own cases (handmade, generated) and permissively licensed data, such as
  TiGL `CurveNetworks` (Apache-2.0), with attribution in `testdata/kernel/README.md`.
- Cases harvested from the FreeCAD forum/tracker, OCCT tracker attachments and the OCCT test
  data have unclear or mixed licences. They stay under `AMG_TESTDATA_DIR`, with a `source`
  URL each.
- OCCT's `tests/bugs/*` Draw scripts are LGPL, so they are **re-implemented** as generators
  from their geometric description, not copied.

Synthetic cases are **not stored as files**. `seeds.toml` lists `(generator, seed, params)`
and the generators (`robust::corpus::gen`, deterministic, behind the `corpus` cargo feature)
rebuild them. Only handmade and promoted cases are `.brep` files.

### 5.2 Case schema (`case.toml`)
```toml
schema = 1
id = "fillet/handmade/bracket-07"
op = "op.json"                 # KernelOp JSON with InputRef(0..) → inputs/0.brep …
source = "handmade"            # handmade | synthetic | owner | harvested:<url> | promoted:<dump hash>
licence = "LicenseRef-Amalgam-Personal"
tags = ["analytic", "convex", "chain"]   # subset filters for the harness
[expect]
outcome = "success"            # success | clean_failure
volume = 12345.678             # optional analytic or reference value, mm³
volume_rel_tol = 1e-6
faces = 23                     # optional
failure_class = "RadiusTooLarge"   # for clean_failure
```
`clean_failure` cases are geometrically impossible requests. Examples: a radius larger than
the adjacent face, or a loft with sections that cannot be matched. The correct answer is a
classified `Err`, never a shape.

### 5.3 Generators and their oracles
| Generator | Produces | Oracle |
|---|---|---|
| `prism_vertical_edges` | extruded random simple polygon (3–12 vertices), random subset of vertical edges filleted, `r < setback limit` | **exact volume**: `V = A·h − h·Σ r²(cot(α/2) − (π−α)/2)` over convex edges with interior angle α, plus the same with exterior angle `2π−β` added for concave edges; faces count |
| `prism_mixed_edges` | as above plus top-face edges (creates corner blends) | validity, volume within the per-edge bounds of §2.2(4), success vs baseline |
| `boolean_stack` | unions/cuts of boxes and cylinders (holes, bosses, pockets), random edge sets | validity, volume bounds; tags `analytic` when all faces are plane/cyl/cone |
| `radius_too_large` | prism edge with `r` > setback limit | `clean_failure`, `RadiusTooLarge`, max radius ≈ limit (analytic) within 2 % |
| `loft_translated` | identical sections translated along a line (2–6 sections) | **exact**: prism volume `A·h` |
| `loft_similar_ruled` | two similar sections scaled about the axis, ruled | **exact** frustum `h/3·(A₁ + A₂ + √(A₁A₂))` |
| `loft_random_stack` | 2–6 planar sections from {circle, ellipse, rounded rect, n-gon 3–8, star, slot, spline blob}, random rotation ±30°, random start vertex and direction, random segment counts | validity, closed solid, sections reproduced within `fit_tol`, no self-interference, monotone area-weighted volume bounds |
| `loft_tangent_chains` | sections made of line + arc chains with G1 joints | G1 across tangent-merged faces ≤ 1e-6 rad on samples |
| `loft_apex` | dome / cone apex point sections | validity; `PointTangent` normal check |
| `loft_rails` / `loft_centerline` | as random stack plus 1–4 rails through section vertices / a centerline | rails reproduced within `fit_tol`; RMF checks |
| `tigl_networks` (fixture set) | TiGL `CurveNetworks` | Gordon reproduces the network; vs OCCT `GeomFill_Gordon` |

Targets: fillet ≥ 200 cases (about 120 synthetic, ≥ 50 handmade, rest harvested/owner).
Loft ≥ 150 cases (about 100 synthetic, ≥ 30 handmade classics, owner + harvested). Handmade
classics include: twisted squares, 4-edge to 3-edge sections, a circle to a square, a star to
a circle, mixed open sections, D-shapes with tangent joints, nearly coincident sections,
reversed section direction, and the Fusion failure modes the owner describes.

## 6. Harness and metrics (KH-02, KH-05)

`amg-cli kernel harness --corpus <dir>[,…] --strategy <name|all> --jobs N --report out.json`
(an `amg-brep` example until lane E wires it):
- Runs every case in **worker processes** (one per job slot, 50 cases per process before
  recycling). An access violation inside OCCT then counts as a `Crash` for that case and does
  not kill the run. A case timeout (default 60 s) is enforced by the parent.
- Per case and strategy it records: `outcome` (`Ok`/`Err`/`Crash`/`Timeout`), `valid`,
  `failure_class`, volume deviation, `max_tolerance`, attempts, `millis`, accepted strategy.
- Aggregates per strategy and tag:
  - **success rate** = valid `Ok` / cases expecting success;
  - **clean-failure rate** = classified `Err` / cases expecting `clean_failure`;
  - **invalid-Ok count** (target 0);
  - **crash count** (target 0 for our own code paths);
  - **volume deviation** p50/max where an oracle exists;
  - **time** p50/p95.
- Writes `out.json` (schema 1) and appends a dated row per strategy to
  `docs/notes/kernel-metrics.md` (created by KH-02, already listed in the notes README).
- **Regression gate:** `tests/golden/kernel/{fillet,loft}-baseline.toml` stores the set of
  case ids each strategy passes. A CI `golden` test runs the `smoke`-tagged subset (≈ 40
  cases, < 2 min) and fails if any baseline-passing case now fails. The full corpus runs on
  demand (`workflow_dispatch`) and before closing any hardening WP. Every hardening WP's PR
  shows the before/after table (S20 Design: "every hardening WP must move these numbers").

## 7. Property tests

| Property | Where |
|---|---|
| `robust::fillet` on `prism_*` and `boolean_stack` inputs returns valid `Ok` or a classified `Err`. Never panic, never an invalid `Ok`. Exact-oracle cases match volume within 1e-6 relative | KH-03/KH-04 (proptest, 256 cases per run, fixed seed in CI) |
| `robust_boolean` on near-coincident inputs: two boxes with face offsets drawn from `{0, ±1e-7, ±1e-6, ±1e-5, ±1e-4}` mm and random tiny rotations returns valid `Ok` with the expected face count, or a classified `Err` | KH-17 |
| Determinism: the same input twice gives bit-identical `.brep` output and identical `RobustReport`, independent of rayon thread count | KH-17 |
| Loft preprocessing invariants: matched segment counts equal; total arc length preserved within 1e-9 relative; correspondence of user-mapped points exact; alignment is invariant under cyclic relabelling of the input vertices | KH-06 |
| Loft on `loft_random_stack` returns valid `Ok` with sections reproduced, or a classified preprocessing `Err` | KH-08 |
| Dump → replay round-trip reproduces the failure class for generated failing inputs | KH-01 |

## 8. References
- OCCT 8.0.1 release notes (2026-07-30): thread-local fillet reconstruction state,
  `GeomFill_Gordon` improvements, `ShapeUpgrade_UnifySameDomain` and `ShapeFix_Face`
  hardening. OCCT 8.0.0p1 (2026-06-17): `GeomFill_Gordon` rework.
- OCCT user guides: Boolean Operations (fuzzy, gluing, non-destructive), Shape Healing.
- A. Zeller, R. Hildebrandt, "Simplifying and isolating failure-inducing input", *IEEE TSE*
  28(2), 2002 (ddmin, failing-edge isolation).
