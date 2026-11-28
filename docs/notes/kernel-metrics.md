# Kernel metrics: corpus pass rates and strategy timings

Measured numbers for the S20 hardening WPs (design note `kernel-robustness.md` §6). Each
hardening WP adds a dated section with its before/after table; the KH-02 / KH-05 harnesses
will append their rows here as well. Numbers come from the tests and benches named in each
section, on the shared development machine (16 logical cores, other workers running), OCCT 8.0.1.

Column meanings (robustness note §6):
- **valid Ok**: `Ok` that passes an independent check: BRepCheck, every face in a closed solid
  with positive volume, and the case's volume oracle within its slack.
- **invalid Ok**: `Ok` that fails that check (target 0).
- **valid Ok above 1e-3 mm tol**: valid `Ok`s whose largest sub-shape tolerance exceeds the
  1e-3 mm cap ("valid but sloppy"; the robust gate refuses them).
- **Err**: a classified error (`FailureClass` other than `Other`).
- **non-direct**: accepted by a strategy other than the direct op.

## 2026-11-28 — KH-17: booleans, raw OCCT vs `robust_boolean`

Source: `cargo nextest run -p amg-brep --features occt -E 'binary(robust)' --no-capture`
(`robust_boolean_beats_raw_occt_on_near_coincident_cases`). Raw = `OcctKernel::boolean` /
`split` with the validation policy off (so OCCT's own invalid results are visible); robust =
`robust_boolean` / `robust_split` with `RetryPolicy::recompute()` (and `exhaustive()`). The
BREP-04 corpus rows count the 131 cases that expect a result (the 5 expected-error cases stay
errors under both). The near-coincident set is `robust::corpus::near_coincident(100, 0x4b483137)`:
10 families × 10 cases, face offsets `{0, ±1e-7, ±1e-6, ±1e-5, ±1e-4}` mm, rotations
`{0, ±1e-7, ±1e-6}` rad and tilts `{±1e-6 … ±1e-3}` rad, each with an analytic volume oracle.
Times are wall times of the whole call (debug-optimised test profile), not criterion.

| set | cases | valid Ok | invalid Ok | valid Ok above 1e-3 mm tol | Err | non-direct | p50 ms | p95 ms |
|---|---|---|---|---|---|---|---|---|
| BREP-04 corpus, raw OCCT | 131 | 131 (100.0 %) | 0 | 0 | 0 | 0 | 2.90 | 6.80 |
| BREP-04 corpus, robust recompute | 131 | 131 (100.0 %) | 0 | 0 | 0 | 0 | 4.29 | 9.56 |
| near-coincident 100, raw OCCT | 100 | 99 (99.0 %) | 1 | 3 | 0 | 0 | 2.70 | 14.12 |
| near-coincident 100, robust recompute | 100 | 99 (99.0 %) | 0 | 0 | 1 | 3 | 4.03 | 18.68 |
| near-coincident 100, robust exhaustive | 100 | 99 (99.0 %) | 0 | 0 | 1 | 3 | 4.07 | 15.43 |

Per family (near-coincident set; every family not listed is 10/10 valid for both):

| family | raw | robust recompute |
|---|---|---|
| `ShiftedCopy` (a copy of the box moved by `(d, d, 0)`) | 9 valid, **1 invalid Ok** (`BadOrientationOfSubshape`, near-035) | 10 valid (1 via the fuzzy ladder) |
| `SkewCoaxial` (coaxial cylinders, second axis tilted 1e-6…1e-3 rad) | 10 "valid", **3 above the tolerance cap** (1.4e-3, 3.3e-2, 5.4e-2 mm) | 9 valid within the cap (2 via the ladder), 1 `ToleranceTooLarge` error (near-039: every strategy keeps 5.4e-2 mm) |

**Reading.** OCCT 8.0.1 handles plain near-coincidence (offsets down to 1e-7 mm, tangent
cylinders, flush pockets, point and line contact) without help: raw and robust both reach 100 %
there. The robust layer's gain is in correctness, not in pass rate. Raw OCCT returned
4 unusable `Ok`s (1 invalid, 3 "valid" with tolerances up to 54× the cap). `robust_boolean`
fixes 3 of them (1 invalid → valid through `fuzzy 1e-5 mm`, 2 sloppy → within the cap through
the ladder) and reports the fourth as a classified `ToleranceTooLarge` error instead of
returning it. Usable results (valid and within the cap): **96 → 99** of 100, invalid `Ok`
**1 → 0**. `exhaustive()` (adds `Full` healing) does not recover near-039. Done-when 1 target
(≥ 95 % valid `Ok`, zero invalid `Ok`) is met.

**exact_only** (Done-when 2): `RetryPolicy::exact_only()` gives byte-identical `.brep` text,
history and warnings to the raw op on all 136 corpus cases and on 97 of the 100 generated
cases. The remaining 3 are the `SkewCoaxial` results above the 1e-3 mm cap, which the gate
refuses (`exact_only` is the raw op plus the gate).

**Determinism** (Done-when 3): 119 inputs (the 100 generated cases + the 19 boolean corpus
cases flagged `determinism`; 3 of them accept a non-direct strategy) give identical `.brep`
bytes, history and `RobustReport` (timing removed) over three runs, on 1-thread and 8-thread
rayon pools.

**Deadline** (Done-when 4): a 1030-face plate cut (about 10 s to finish) under a 300 ms
attempt deadline returns `Timeout` after 341 ms through `Chain` (the strategy behind it never
runs) and after 568 ms through `robust_boolean` (operand volumes for the gate included).

### Overhead of the direct path (S20 target: ≤ raw op + validation, < 5 %)

`cargo bench -p amg-brep --features occt --bench robust`, criterion medians, release:

| case | raw | raw + gate | robust (direct accepted) | robust vs raw + gate |
|---|---|---|---|---|
| blind hole (box − cylinder) | 1.66 ms | 2.87 ms | 2.93 ms | +2.2 % |
| 202-face plate pair cut | 409 ms | 687 ms | 704 ms | +2.4 % |

The gate itself (BRepCheck, closed-solid and volume checks) costs 70–75 % on top of the raw
op. A first version spawned one deadline thread per attempt and summed every operand face
area up front: +15.7 % / +7.3 %. A single process-wide deadline thread and computing the
areas only for fuzzy attempts brought it under 5 %.

## 2026-11-28 — KH-20: kernel speed of extrude-cut and shell

Measured on the shared development machine with another worker building (13–16 GB free RAM),
**not on an idle machine**; the owner re-measures. Release (`bench`) profile, OCCT 8.0.1 with
8 OCCT threads (the default, half the cores). "Before" is the same binary with the KH-20 fast
paths off (`AMG_KERNEL_FAST_PATHS=0`), so both columns ran under the same load. 25-run medians
come from the fixed samples the benches print before criterion runs; criterion medians from
two full criterion runs each.

### Budgets (S04 benches, `amg-features`)

| bench (budget) | before | after | change |
|---|---|---|---|
| `budget/sol-extrude-cut`, 25-run medians (< 80 ms) | 116.3, 116.4, 121.7, 122.1, 122.9 ms | 101.4, 102.2, 104.8, 106.0, 106.4 ms | median 121.7 → 104.8 ms (−14 %) |
| `budget/sol-extrude-cut`, criterion | 112.9, 113.0 ms | 99.7, 99.9 ms | −11.6 % |
| `budget/sol-shell-preview`, 25-run (< 100 ms) | 336.3 ms (KH-20 start), 369 ms (SOL-06) | 311.6, 336.2 ms | no change (no valid fast path, below) |
| `budget/sol-shell-preview`, criterion | — | 315.0, 313.6 ms | |
| `budget/sol-shell-full`, criterion | — | 336.7, 337.8 ms | |

Neither budget is met. The extrude-cut's boolean alone is above 80 ms (below).

### Kernel level (`cargo bench -p amg-brep --features occt --bench kernel_speed`)

Same inputs as the S04 benches, built in `tests/common/speed_parts.rs`.

| call | 25-run medians | criterion |
|---|---|---|
| boolean cut 198-gon prism (200 faces) − 100-gon prism, default `BoolOpts`, before the inverted-check cache | 89.4, 91.5, 90.6 ms | |
| same, with the cache (the body's check cached, the tool's still run) | 85.9, 87.7, 84.1 ms | |
| same, `check_inverted: false` (both checks skipped) | 79.7–82.0 ms | |
| same, serial (`parallel: Some(false)`) | 120.0–128.5 ms | |
| same, `use_obb: true` | 82.0–89.2 ms | |
| `unify_same_domain` of the 300-face cut result, plain / skip | 21.6–22.4 / 11.1–11.6 ms | |
| boolean + unify, fast paths off / on (`kh20/cut-198-100-{plain,fast}`) | — / 91.7–99.6 ms | 111.8, 112.6 / 95.8, 93.3 ms |
| shell of the star prism (100 faces), top removed, −0.3 mm, intersection join | 308–316 ms | 318.7, 312.3 ms |
| shell of the 94-hole plate (100 faces), same | 636–648 ms | 642.7, 637.2 ms |

### Where the time goes (`AMG_OCCT_PROFILE=1`)

Per-call averages over the 26 runs of `budget/sol-extrude-cut`, from OCCT's progress scopes and
the shim's own steps (tracing on, so about 10 % above the plain timings).

**Boolean cut (`BRepAlgoAPI_BooleanOperation`, CUT, parallel), ~95 ms traced:**

| phase | ms |
|---|---|
| shim: detach operands, set up, DS init before the first progress step | 4.8 |
| intersection (`BOPAlgo_PaveFiller`): init 0.9, VF 0.6, VE 0.7, EF 7.0, FF 6.6, repeat 0.6, section edges 11.0, section edge intersection 1.5, projections 0.8, the rest between named steps 8.4 | 38.1 |
| result: splits of edges 1.2, splits of containers 7.8, splits of faces 13.1, **splits of solids 22.6** (incl. the inverted-solid checks), history 4.3, post-treatment 0.5, rest 0.8 | 50.3 |
| shim: status 0.4, history recording 1.2 | 1.6 |

**`unify_same_domain` (SOL-01 `simplify`), plain run ~22 ms:** `BRepBuilderAPI_Copy` 6.8 ms,
`ShapeUpgrade_UnifySameDomain::Build` 11–18 ms, history 2.4 ms. With the skip: copy, the
candidate scan and the history (~9–11 ms in all).

**Shell of the star prism (`BRepOffset_MakeOffset::MakeThickSolid`), ~362 ms traced:**

| phase | ms |
|---|---|
| shim: copy 2.9, curvature check 1.1, set-up and input check 3.1 | 7.1 |
| edge analysis 0.5, offset faces 2.5 | 3.0 |
| **`BRepOffset_Inter3d::ConnexIntByInt`** ("Intersecting offset faces") | **91.1** |
| **`BRepOffset_Inter3d::ContextIntByInt`** (the removed top face against every offset face; printed under "Creating unique intersection", the scope before it) | **207.1** |
| edge intersections (3D 8.6, from vertices 6.4, on faces 20.2), loops 2.3, history 2.4, shells 2.2, solid 2.4 | 44.5 |
| shim: canonical order 6.1, regularity 0.5, history 3.2 | 9.8 |

**Reading.** For an inward offset, `ContextIntByInt` intersects the removed face *as it is*
(bounded, 98 edges) with every enlarged offset face. `BRepOffset_Tool::Inter3D` then cannot
use its analytic plane–plane path (both faces must be unbounded) and runs a full
`BOPAlgo_PaveFiller` per pair, so the cost grows with (offset faces × edges of the removed
face). Outward (`offset > 0`) OCCT enlarges the removed face first (`ExtentContext`), and the
same shell takes 73 ms. `ConnexIntByInt` also runs one PaveFiller per adjacent pair of enlarged
offset faces. Together they are ~300 of ~313 ms, and none of OCCT's options reach them.
