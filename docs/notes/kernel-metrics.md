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
