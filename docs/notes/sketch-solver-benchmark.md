# Sketch solver benchmark — SK-13 report for ADR-007

| | |
|---|---|
| **Spec** | [S03](../specs/S03-sketch.md) SK-13 |
| **Plan** | [`sketch-solver.md`](sketch-solver.md) §5 (cases §5.2, metrics §5.3, decision rule §5.4) |
| **Date** | 2026-10-21 |
| **Status** | Measured. Recommendation: **option A (own Rust solver) as primary**, libslvs stays a test-only oracle (proposal in `docs/QUESTIONS.md` #50) |

## 1. What was measured

| Option | Code | Feature | Notes |
|---|---|---|---|
| **A. native prototype** | `amg_sketch::solve::proto` | always | Substitution (union-find on coincident/H/V/concentric/equal-radius and fixed params) → connected components → damped weighted min-norm Gauss–Newton (LM; first step pure GN), dense, hand-written Cholesky. Rank/DOF per component, dependency groups on the unsubstituted Jacobian. SK-02 residuals (signed, chirality-preserving). |
| A variants | `native-fast` (no rank test/diagnosis), `native-dogleg` (Powell dogleg), `native-sqp` (PlaneGCS-style feasible SQP dragging) | always | Same solver core; the variants answer the note's "dogleg vs LM" and "SQP vs weighted min-norm" questions. |
| **B. libslvs** | `amg_sketch::corpus::compare::slvs`, `slvs` 0.6.0 crate | `slvs` (GPL-3.0, dev/test only) | Our sketch mapped to libslvs entities on one workplane, fixed params in group 1. |
| **C. ezpz** | `amg_sketch::corpus::compare::ezpz`, `kcl-ezpz` 0.2.18 | `ezpz-bench` (MIT, dev-only) | Stateless; convergence tolerance set to 1e-10 (default 1e-8) for a fair residual comparison. |

All options run through one interface (`corpus::compare::{Contender, Loaded}`), and every result is
judged by the same SK-02 residuals on the values the option returns. "Solved" is the corpus runner
verdict: accepted status, the expected root (`Start`/`Target` values within 1e-6 mm, so D3 slot
chirality is checked) and the DOF when the option reports one.

**Timing** (µs, median / p95 of 200 runs, release build, `examples/sk13_report.rs`):
- *Cold*: map the start configuration into the option's model and solve (building our `Sketch` from the
  case is excluded for all).
- *Warm*: the first driving dimension alternates between +1 % and its value; each run solves from the
  previous solution. The native prototype has **no structure cache yet**: its warm solve rebuilds the
  equation system, substitution and components every time (SK-03 adds the cache).
- *Drag*: per-frame time over 3 passes of the case's drag script; jitter, branch flips and converged
  frames from the first pass. Jitter = max over frames of (largest movement of a non-dragged point) /
  (cursor step). Flips = sign changes of `cross(a − o1, b − o1)` (D7 assembly branch), ignoring frames
  at the toggle where `|sin| < 1e-6`.

**Machine:** the owner's Windows 11 workstation, MSVC 14.44, Rust 1.97.1, while about ten other
agents were compiling (so absolute numbers are pessimistic and p95 is noisy; ratios between options
are stable across the two runs made).

## 2. Results on D1–D10

Columns: *Solved* = runner verdict; *Status* as reported (libslvs `INCONSISTENT` with satisfied residuals is shown as `ConvergedRedundant`); max‖F‖ = SK-02 residual of the returned values; *Failing sets* = reported dependency / failed / unsatisfied constraint sets as case constraint indices (`none` = empty, `-` = not reported). D5 constraints: 0 fix, 1/3 horizontal, 2/4 vertical, 5/6 lengths, 7 parallel, 8 equal; D6: 0 fix, 1 horizontal, 2–4 lengths 10, 10, 30.

| Case | Option | Solved (runner) | Status | max‖F‖ (mm) | DOF | Cold µs med / p95 | Warm µs med / p95 | Drag µs/frame med / p95 | Jitter | Flips | Drag frames ok | Failing sets |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| D1 | native | yes | Converged | 5.4e-13 | 0 | 5.7 / 5.8 | 4.9 / 5.1 | - | - | - | - | none |
| D1 | native-fast | yes | Converged | 5.4e-13 | - | 5.3 / 5.5 | 4.7 / 4.7 | - | - | - | - | - |
| D1 | native-dogleg | yes | Converged | 5.4e-13 | 0 | 6.2 / 7.9 | 6.8 / 7.0 | - | - | - | - | none |
| D1 | native-sqp | yes | Converged | 5.4e-13 | 0 | 5.7 / 5.8 | 4.9 / 5.1 | - | - | - | - | none |
| D1 | libslvs | yes | Converged | 5.4e-13 | 0 | 72.7 / 122.3 | 69.2 / 115.5 | - | - | - | - | none |
| D1 | ezpz | yes | Converged | 7.2e-13 | - | 17.5 / 17.9 | 15.6 / 15.9 | - | - | - | - | none |
| D2 | native | yes | Converged | 0.0e0 | 0 | 4.9 / 5.0 | 4.3 / 4.6 | - | - | - | - | none |
| D2 | native-fast | yes | Converged | 0.0e0 | - | 4.6 / 7.7 | 4.0 / 6.2 | - | - | - | - | - |
| D2 | native-dogleg | yes | Converged | 0.0e0 | 0 | 4.9 / 5.0 | 4.4 / 4.6 | - | - | - | - | none |
| D2 | native-sqp | yes | Converged | 0.0e0 | 0 | 4.8 / 6.3 | 4.2 / 5.5 | - | - | - | - | none |
| D2 | libslvs | yes | Converged | 0.0e0 | 0 | 49.7 / 84.0 | 75.3 / 77.5 | - | - | - | - | none |
| D2 | ezpz | yes | Converged | 4.7e-11 | - | 15.4 / 19.4 | 13.2 / 13.4 | - | - | - | - | none |
| D3 | native | yes | Converged | 0.0e0 | 0 | 16.6 / 17.6 | 10.6 / 11.2 | - | - | - | - | none |
| D3 | native-fast | yes | Converged | 0.0e0 | - | 14.5 / 21.0 | 9.1 / 11.7 | - | - | - | - | - |
| D3 | native-dogleg | yes | Converged | 0.0e0 | 0 | 16.9 / 23.1 | 10.6 / 17.5 | - | - | - | - | none |
| D3 | native-sqp | yes | Converged | 0.0e0 | 0 | 15.7 / 16.5 | 10.4 / 10.9 | - | - | - | - | none |
| D3 | libslvs | yes | Converged | 0.0e0 | 0 | 5380.3 / 7205.7 | 5335.3 / 7875.8 | - | - | - | - | none |
| D3 | ezpz | yes | Converged | 3.1e-18 | - | 47.1 / 55.0 | 27.3 / 34.4 | - | - | - | - | none |
| D4 | native | yes | Converged | 0.0e0 | 102 | 11.2 / 11.9 | - | 0.7 / 1.3 | 0.000 | - | 100/100 | none |
| D4 | native-fast | yes | Converged | 0.0e0 | - | 10.4 / 11.2 | - | 0.7 / 0.8 | 0.000 | - | 100/100 | - |
| D4 | native-dogleg | yes | Converged | 0.0e0 | 102 | 10.1 / 10.5 | - | 0.8 / 1.3 | 0.000 | - | 100/100 | none |
| D4 | native-sqp | yes | Converged | 0.0e0 | 102 | 10.2 / 13.2 | - | 0.7 / 0.8 | 0.000 | - | 100/100 | none |
| D4 | libslvs | yes | Converged | 0.0e0 | 102 | 35.2 / 38.6 | - | 21.4 / 27.0 | 0.000 | - | 100/100 | none |
| D4 | ezpz | yes | Converged | 0.0e0 | - | 7.2 / 7.7 | - | 12.1 / 12.6 | 0.000 | - | 100/100 | none |
| D5 | native | yes | ConvergedRedundant | 0.0e0 | 0 | 10.2 / 10.4 | 9.7 / 11.4 | - | - | - | - | Redundant{1,3,7} Redundant{1,3,8} |
| D5 | native-fast | **no**: status Converged, expected ConvergedRedundant | Converged | 0.0e0 | - | 5.7 / 5.9 | 5.2 / 5.3 | - | - | - | - | - |
| D5 | native-dogleg | yes | ConvergedRedundant | 0.0e0 | 0 | 10.3 / 10.8 | 9.8 / 10.1 | - | - | - | - | Redundant{1,3,7} Redundant{1,3,8} |
| D5 | native-sqp | yes | ConvergedRedundant | 0.0e0 | 0 | 10.2 / 16.0 | 9.7 / 16.9 | - | - | - | - | Redundant{1,3,7} Redundant{1,3,8} |
| D5 | libslvs | yes | ConvergedRedundant | 0.0e0 | 0 | 1004.0 / 1073.6 | 1000.5 / 1059.7 | - | - | - | - | none |
| D5 | ezpz | **no**: status Converged, expected ConvergedRedundant | Converged | 8.9e-13 | - | 23.7 / 48.8 | 16.5 / 28.8 | - | - | - | - | none |
| D6 | native | yes | DidNotConverge | 3.3e0 | 0 | 18.6 / 35.2 | 14.1 / 14.3 (0/200 conv.) | - | - | - | - | Conflicting{2,3,4} |
| D6 | native-fast | yes | DidNotConverge | 3.3e0 | - | 16.2 / 16.6 | 12.6 / 23.1 (0/200 conv.) | - | - | - | - | - |
| D6 | native-dogleg | yes | DidNotConverge | 3.3e0 | 0 | 37.8 / 45.0 | 22.5 / 46.7 (0/200 conv.) | - | - | - | - | Conflicting{2,3,4} |
| D6 | native-sqp | yes | DidNotConverge | 3.3e0 | 0 | 18.7 / 24.0 | 14.2 / 14.6 (0/200 conv.) | - | - | - | - | Conflicting{2,3,4} |
| D6 | libslvs | yes | DidNotConverge | 2.3e1 | 0 | 223.5 / 475.6 | 216.9 / 307.6 (0/200 conv.) | - | - | - | - | Conflicting{3,4} |
| D6 | ezpz | yes | DidNotConverge | 2.3e1 | - | 343.6 / 551.9 | 340.3 / 674.8 (0/200 conv.) | - | - | - | - | - |
| D7 | native | yes | Converged | 2.6e-13 | 1 | 5.3 / 6.7 | 4.8 / 4.9 | 20.7 / 27.1 | 66.154 | 68 | 200/200 | none |
| D7 | native-fast | yes | Converged | 2.6e-13 | - | 5.0 / 9.7 | 4.5 / 4.6 | 20.3 / 25.0 | 66.154 | 68 | 200/200 | - |
| D7 | native-dogleg | yes | Converged | 2.6e-13 | 1 | 5.9 / 6.0 | 5.4 / 5.5 | 20.7 / 26.3 | 46.983 | 60 | 200/200 | none |
| D7 | native-sqp | yes | Converged | 2.6e-13 | 1 | 5.3 / 6.4 | 4.8 / 5.0 | 18.2 / 18.9 | 0.847 | 1 | 200/200 | none |
| D7 | libslvs | yes | Converged | 2.6e-13 | 1 | 82.5 / 169.7 | 76.0 / 119.5 | 229.5 / 392.9 | 105.566 | 43 | 47/200 | none |
| D7 | ezpz | yes | Converged | 2.6e-13 | - | 19.9 / 25.8 | 18.1 / 18.4 | 453.9 / 772.5 | 65.950 | 58 | 200/200 | none |
| D8 | native | yes | Converged | 1.3e-13 | 0 | 210.2 / 283.1 | 116.0 / 155.5 | - | - | - | - | none |
| D8 | native-fast | yes | Converged | 1.3e-13 | - | 166.4 / 221.1 | 76.2 / 107.2 | - | - | - | - | - |
| D8 | native-dogleg | yes | Converged | 1.3e-13 | 0 | 221.2 / 338.9 | 117.2 / 182.5 | - | - | - | - | none |
| D8 | native-sqp | yes | Converged | 1.3e-13 | 0 | 209.8 / 320.4 | 116.3 / 152.4 | - | - | - | - | none |
| D8 | libslvs | yes | Converged | 3.0e-11 | 0 | 202558.5 / 260748.0 | 199015.3 / 256821.6 | - | - | - | - | none |
| D8 | ezpz | yes | Converged | 4.2e-11 | - | 255.2 / 482.1 | 148.3 / 231.5 | - | - | - | - | none |
| D9 | native | yes | Converged | 0.0e0 | 0 | 161.8 / 225.4 | 92.0 / 117.5 | - | - | - | - | none |
| D9 | native-fast | yes | Converged | 0.0e0 | - | 153.0 / 275.1 | 87.4 / 101.4 | - | - | - | - | - |
| D9 | native-dogleg | yes | Converged | 0.0e0 | 0 | 162.4 / 317.5 | 90.1 / 106.5 | - | - | - | - | none |
| D9 | native-sqp | yes | Converged | 0.0e0 | 0 | 113.9 / 179.4 | 91.2 / 118.2 | - | - | - | - | none |
| D9 | libslvs | yes | Converged | 0.0e0 | 0 | 810.0 / 1493.1 | 650.8 / 1261.6 | - | - | - | - | none |
| D9 | ezpz | yes | Converged | 1.4e-14 | - | 324.0 / 459.8 | 163.6 / 232.0 | - | - | - | - | none |
| D10 | native | yes | Converged | 0.0e0 | 0 | 3.9 / 4.2 | 3.5 / 3.6 | - | - | - | - | none |
| D10 | native-fast | yes | Converged | 0.0e0 | - | 4.6 / 4.7 | 3.3 / 4.2 | - | - | - | - | - |
| D10 | native-dogleg | yes | Converged | 0.0e0 | 0 | 4.1 / 4.2 | 3.7 / 3.8 | - | - | - | - | none |
| D10 | native-sqp | yes | Converged | 0.0e0 | 0 | 4.0 / 4.1 | 3.5 / 3.6 | - | - | - | - | none |
| D10 | libslvs | yes | Converged | 0.0e0 | 0 | 43.7 / 55.0 | 74.4 / 81.8 | - | - | - | - | none |
| D10 | ezpz | yes | Converged | 1.1e-18 | - | 16.7 / 31.8 | 14.7 / 14.9 | - | - | - | - | none |

Decision cases solved: {"ezpz": 9, "libslvs": 10, "native": 10, "native-dogleg": 10, "native-fast": 9, "native-sqp": 10}

## 3. Whole SK-14 corpus (135 cases)

| Option | Pass | Fail | Skipped (unsupported) | Pass rate of run cases |
|---|---|---|---|---|
| native | 135 | 0 | 0 | 100.0 % |
| native-fast | 129 | 6 | 0 | 95.6 % |
| native-dogleg | 134 | 1 | 0 | 99.3 % |
| native-sqp | 135 | 0 | 0 | 100.0 % |
| libslvs | 118 | 7 | 10 | 94.4 % |
| ezpz | 121 | 7 | 7 | 94.5 % |

libslvs passes, native does not: **0** of 135 cases.

native passes, libslvs does not: 17 cases

#### ezpz failures

- ezpz/nonsquare: status Converged, expected ConvergedRedundant
- ezpz/parallelogram: status Converged, expected ConvergedRedundant
- ezpz/square: status Converged, expected ConvergedRedundant
- own/d05_rectangle_redundant: status Converged, expected ConvergedRedundant
- own/duplicate_coincident: status Converged, expected ConvergedRedundant
- planegcs/rectangle_four_right_angles: status Converged, expected ConvergedRedundant
- planegcs/symmetric_mirrored_redundant: status Converged, expected ConvergedRedundant

#### libslvs failures

- ezpz/angle_parallel_manual: status ConvergedRedundant, expected Converged; dof 1, expected 0
- ezpz/square: status Inconsistent, expected ConvergedRedundant; dof 2, expected 0
- own/duplicate_coincident: status Converged, expected ConvergedRedundant
- planegcs/rectangle_four_right_angles: status Inconsistent, expected ConvergedRedundant
- solvespace/large_dimensions/perpendicular_4m: status DidNotConverge, expected Converged
- solvespace/pt_on_line/normal: solution off by 2.206e-2 mm (tolerance 1.0e-6)
- solvespace/pt_on_line/normal_v22: solution off by 2.206e-2 mm (tolerance 1.0e-6)

#### native-dogleg failures

- ezpz/square: status Inconsistent, expected ConvergedRedundant

#### native-fast failures

- ezpz/nonsquare: status Converged, expected ConvergedRedundant
- ezpz/parallelogram: status Converged, expected ConvergedRedundant
- ezpz/square: status Converged, expected ConvergedRedundant
- own/d05_rectangle_redundant: status Converged, expected ConvergedRedundant
- planegcs/rectangle_four_right_angles: status Converged, expected ConvergedRedundant
- planegcs/symmetric_mirrored_redundant: status Converged, expected ConvergedRedundant

## 4. Decision rule (§5.4) check

| Rule | Threshold | Option A (native) | B (libslvs) | C (ezpz) | Met |
|---|---|---|---|---|---|
| 1. Decision cases solved | ≥ 9 / 10 | **10 / 10** | 10 / 10 | 9 / 10 (D5: no redundancy report) | yes |
| 2. D9 warm solve, median | < 5 ms | **0.092 ms** (no structure cache yet) | 0.65 ms | 0.16 ms | yes (54× margin) |
| 3a. Drag p95 per frame, D4 / D7 | < 2 ms | **0.0013 / 0.027 ms** (SQP: 0.0006 / 0.019) | 0.023 / 0.39 ms | 0.025 / 0.77 ms | yes |
| 3b. Drag jitter vs libslvs, D4 / D7 | ≤ 1.5× | **0 / 66.2** = 0× / 0.63× (SQP: 0 / 0.85) | 0 / 105.6 | 0 / 66.0 | yes |
| 4. Diagnosis D5 / D6 at least as specific as libslvs | — | D5: both minimal redundant sets `{1,3,7}`, `{1,3,8}`; D6: the minimal conflicting set `{2,3,4}` | D5: "redundant", empty failed list; D6: `{3,4}` (not a conflicting set on its own) | D5: not detected; D6: no set | yes |

**All four conditions hold, so the accepted rule selects option A as primary.** Option A misses
neither (1) nor (2), so option C is not evaluated as primary.

**Corpus gap (S03 OQ 4 / QUESTIONS #18):** libslvs passes **0** cases that the native prototype
fails (native 135/135; libslvs 118 of the 125 it can express). There is no > 2 % gap, so libslvs
stays test-only.

## 5. Findings

1. **Correctness.** The prototype passes the whole SK-14 corpus (135/135) with LM, and 134/135 with
   dogleg. It keeps every `Start` solution (minimum movement) and every `Target` root, including the
   D3 slot and the PlaneGCS left/right-turn cases.
2. **Speed.** The prototype is 3–1000× faster than libslvs on every case: D3 slot 17 µs vs 5.4 ms,
   D8 bolt circle 0.21 ms vs 200 ms (libslvs's dense Newton + rank test on all unknowns), D9 grid
   0.16 ms vs 0.81 ms cold. Substitution removes most rows on the CAD-like cases (D9: 220 of 240
   rows). Even without SK-03's structure cache, warm solves stay far below the 5 ms budget.
3. **Diagnosis.** Row-wise orthogonalisation of the unsubstituted Jacobian gives each dependency
   circuit separately. libslvs's leave-one-out search (`FindWhichToRemoveToFixJacobian`) cannot name
   anything when there are two independent redundancies (D5) and returned a non-minimal pair on D6.
   ezpz has neither DOF nor redundancy reporting (it fails all 7 redundant corpus cases). The rank
   test is needed for `ConvergedRedundant` (`native-fast` without it misses 6 corpus cases); it costs
   ~1–5 µs on the decision cases.
4. **Dragging near singularities (D7).** The D7 cursor path stays outside the linkage's reach
   (`|b − o1|` up to 16 mm, reach 12 mm), so a correct drag holds the linkage at the toggle.
   Restarting each frame from the cursor (weighted min-norm: native, libslvs, and ezpz's guess +
   low-priority `Fixed`) lands on arbitrary points of the singular set: 43–68 branch flips and jitter
   66–106. libslvs converged on only 47 of 200 frames. The **feasible SQP** variant (KKT step on
   `½‖p − cursor‖² + ½μ‖Δx‖²`, then min-norm restoration, step halving) has jitter 0.85, one flip (on
   the first frame, into the toggle), all frames converged, and is no slower (p95 19 µs). The
   weighted min-norm solve also slips the dragged point by ~W⁻¹ of the correction (≈ 1e-5 mm per
   frame on the rocker circle). **Recommendation for SK-05:** keep weighted min-norm for the common
   case and fall back to feasible SQP when the min-norm frame does not converge, moves the
   non-dragged geometry more than the cursor moved, or the Jacobian is near-singular; or make SQP the
   default. SK-05 decides with its own 200-entity scripted drags.
5. **Dogleg vs LM.** Identical on D1–D10 and the corpus except `ezpz/square` (from bad guesses
   dogleg stops at a non-solution, LM finds the redundant root). `Algorithm::Auto` = LM.
6. **libslvs limits seen.** Not expressible: line–circle and circle–circle tangency, arc length,
   distances to a circle edge, line–line distance (10 corpus cases). Its tangencies and angles are
   unsigned, so chirality depends on the start point. `PT_ON_LINE` adds a hidden curve parameter, and
   the solved under-constrained `solvespace/pt_on_line` cases move by 0.022 mm. It reports a
   duplicate coincident as `OKAY` (redundancy hidden by its substitution) and did not converge on
   `large_dimensions/perpendicular_4m`.
7. **ezpz limits seen.** No DOF count, no redundancy detection, no endpoint tangency (emulated with
   perpendicular/parallel radius lines, unsigned), no equal line–arc, symmetric arcs or edge
   distances (7 corpus cases skipped). Fast on solves (2–3× slower than the prototype); drags cost
   ~0.45 ms because the priority mechanism solves twice per frame.

## 6. Engineering

| | A. native | B. libslvs | C. ezpz |
|---|---|---|---|
| Code | prototype 1,081 lines + tests 253; corpus adapter 189 | adapter 618 lines + `build.rs` (advapi32) | adapter 483 lines |
| Build | pure Rust, no new crates | C++ (8 SolveSpace sources + mimalloc) via `cc`; **bindgen 0.64 needs libclang ≤ 15** (`LIBCLANG_PATH`; libclang 18 generates empty bindings for `slvs.h` and the crate fails to compile). Built here with the PyPI `libclang==15.0.6.1` wheel in the VS 2022 x64 developer environment | pure Rust, pulls `faer` 0.23 and ~20 small crates (all MIT / Apache-2.0) |
| Windows friction | none | yes: libclang version, MSVC environment, `advapi32` link | none |
| Data-model friction | none | two models kept in sync (handles, groups, sign conventions; `PROJ_PT_DISTANCE` and `PT_LINE_DISTANCE` have the opposite sign of ours) | value vector + request list rebuilt per solve; several of our constraints emulated |
| Licence | ours | GPL-3.0: feature-gated, never default (ADR-012) | MIT |

Build time added could not be measured reliably on the shared machine; the libslvs C++ compile and
bindgen are the largest addition (minutes, once per clean build), ezpz adds faer.

## 7. Proposed ADR-007 text (for the owner; see QUESTIONS #50)

> **ADR-007 — Sketch constraint solver: own Rust solver; libslvs as test oracle (Accepted <date>)**
> - The native Rust solver in `amg-sketch` is the only shipped solver: substitution, connected
>   components + Dulmage–Mendelsohn blocks, damped weighted min-norm Gauss–Newton/LM
>   (`Algorithm::Auto` = LM; dogleg optional), rank-revealing DOF analysis and per-circuit
>   dependency diagnosis on the unsubstituted Jacobian.
> - SolveSpace `libslvs` (GPL-3.0) stays behind cargo feature `slvs` as a differential-test oracle
>   only. SK-13 found no corpus case that libslvs solves and the native prototype does not (0 of 135),
>   so there is no runtime fallback.
> - ezpz (MIT) stays a test-case source and benchmark reference behind `ezpz-bench`; it is not a
>   dependency.
> - Evidence: `docs/notes/sketch-solver-benchmark.md` (SK-13, 2026-10-21): the native prototype
>   solved 10/10 decision cases and 135/135 corpus cases, D9 warm solve 0.09 ms (budget 5 ms), drag
>   p95 ≤ 0.03 ms, D7 jitter 0.63× libslvs, minimal dependency sets on D5/D6.
> - Consequence for SK-05: dragging near singular configurations needs a feasible-SQP fallback (or
>   SQP default); SK-05 decides.

## 8. Reproduce

```text
# options A + C (pure Rust)
cargo run --release -p amg-sketch --example sk13_report --features ezpz-bench -- 200 <out-dir>
# with libslvs: C++ compiler + libclang <= 15, e.g. in a VS x64 developer prompt:
#   pip download libclang==15.0.6.1 --no-deps  (unzip; LIBCLANG_PATH=<...>/clang/native)
cargo run --release -p amg-sketch --example sk13_report --features slvs,ezpz-bench -- 200 <out-dir>
# criterion: sk13/<cold|warm|drag>/<Dn>/<option>
cargo bench -p amg-sketch --bench solver_compare [--features slvs,ezpz-bench]
cargo nextest run -p amg-sketch -E 'binary(sk13_compare)'
```

## Appendix A — raw CSV (decision cases)

```csv
case,option,skipped,pass,status,max_residual,dof,cold_med_us,cold_p95_us,warm_med_us,warm_p95_us,drag_med_us,drag_p95_us,jitter,flips,drag_ok,drag_frames
D1,native,false,true,Converged,5.417888360170764e-13,0,5.700,5.800,4.900,5.100,,,,,,
D1,native-fast,false,true,Converged,5.417888360170764e-13,,5.300,5.500,4.700,4.700,,,,,,
D1,native-dogleg,false,true,Converged,5.417888360170764e-13,0,6.200,7.900,6.800,7.000,,,,,,
D1,native-sqp,false,true,Converged,5.417888360170764e-13,0,5.700,5.800,4.900,5.100,,,,,,
D1,libslvs,false,true,Converged,5.417888360170764e-13,0,72.700,122.300,69.200,115.500,,,,,,
D1,ezpz,false,true,Converged,7.247535904753022e-13,,17.500,17.900,15.600,15.900,,,,,,
D2,native,false,true,Converged,0e0,0,4.900,5.000,4.300,4.600,,,,,,
D2,native-fast,false,true,Converged,0e0,,4.600,7.700,4.000,6.200,,,,,,
D2,native-dogleg,false,true,Converged,0e0,0,4.900,5.000,4.400,4.600,,,,,,
D2,native-sqp,false,true,Converged,0e0,0,4.800,6.300,4.200,5.500,,,,,,
D2,libslvs,false,true,Converged,0e0,0,49.700,84.000,75.300,77.500,,,,,,
D2,ezpz,false,true,Converged,4.704195197402422e-11,,15.400,19.400,13.200,13.400,,,,,,
D3,native,false,true,Converged,0e0,0,16.600,17.600,10.600,11.200,,,,,,
D3,native-fast,false,true,Converged,0e0,,14.500,21.000,9.100,11.700,,,,,,
D3,native-dogleg,false,true,Converged,0e0,0,16.900,23.100,10.600,17.500,,,,,,
D3,native-sqp,false,true,Converged,0e0,0,15.700,16.500,10.400,10.900,,,,,,
D3,libslvs,false,true,Converged,0e0,0,5380.300,7205.700,5335.300,7875.800,,,,,,
D3,ezpz,false,true,Converged,3.1179003633654285e-18,,47.100,55.000,27.300,34.400,,,,,,
D4,native,false,true,Converged,0e0,102,11.200,11.900,,,0.700,1.300,0.000,,100,100
D4,native-fast,false,true,Converged,0e0,,10.400,11.200,,,0.700,0.800,0.000,,100,100
D4,native-dogleg,false,true,Converged,0e0,102,10.100,10.500,,,0.800,1.300,0.000,,100,100
D4,native-sqp,false,true,Converged,0e0,102,10.200,13.200,,,0.700,0.800,0.000,,100,100
D4,libslvs,false,true,Converged,0e0,102,35.200,38.600,,,21.400,27.000,0.000,,100,100
D4,ezpz,false,true,Converged,0e0,,7.200,7.700,,,12.100,12.600,0.000,,100,100
D5,native,false,true,ConvergedRedundant,0e0,0,10.200,10.400,9.700,11.400,,,,,,
D5,native-fast,false,false,Converged,0e0,,5.700,5.900,5.200,5.300,,,,,,
D5,native-dogleg,false,true,ConvergedRedundant,0e0,0,10.300,10.800,9.800,10.100,,,,,,
D5,native-sqp,false,true,ConvergedRedundant,0e0,0,10.200,16.000,9.700,16.900,,,,,,
D5,libslvs,false,true,ConvergedRedundant,0e0,0,1004.000,1073.600,1000.500,1059.700,,,,,,
D5,ezpz,false,false,Converged,8.899616460755022e-13,,23.700,48.800,16.500,28.800,,,,,,
D6,native,false,true,DidNotConverge,3.3333333423518816e0,0,18.600,35.200,14.100,14.300,,,,,,
D6,native-fast,false,true,DidNotConverge,3.3333333423518816e0,,16.200,16.600,12.600,23.100,,,,,,
D6,native-dogleg,false,true,DidNotConverge,3.336752884352858e0,0,37.800,45.000,22.500,46.700,,,,,,
D6,native-sqp,false,true,DidNotConverge,3.3333333423518816e0,0,18.700,24.000,14.200,14.600,,,,,,
D6,libslvs,false,true,DidNotConverge,2.2928932188134524e1,0,223.500,475.600,216.900,307.600,,,,,,
D6,ezpz,false,true,DidNotConverge,2.2928932188134524e1,,343.600,551.900,340.300,674.800,,,,,,
D7,native,false,true,Converged,2.5579538487363607e-13,1,5.300,6.700,4.800,4.900,20.700,27.100,66.154,68,200,200
D7,native-fast,false,true,Converged,2.5579538487363607e-13,,5.000,9.700,4.500,4.600,20.300,25.000,66.154,68,200,200
D7,native-dogleg,false,true,Converged,2.5579538487363607e-13,1,5.900,6.000,5.400,5.500,20.700,26.300,46.983,60,200,200
D7,native-sqp,false,true,Converged,2.5579538487363607e-13,1,5.300,6.400,4.800,5.000,18.200,18.900,0.847,1,200,200
D7,libslvs,false,true,Converged,2.5579538487363607e-13,1,82.500,169.700,76.000,119.500,229.500,392.900,105.566,43,47,200
D7,ezpz,false,true,Converged,2.6201263381153694e-13,,19.900,25.800,18.100,18.400,453.900,772.500,65.950,58,200,200
D8,native,false,true,Converged,1.3314384049638411e-13,0,210.200,283.100,116.000,155.500,,,,,,
D8,native-fast,false,true,Converged,1.3314384049638411e-13,,166.400,221.100,76.200,107.200,,,,,,
D8,native-dogleg,false,true,Converged,1.3314384049638411e-13,0,221.200,338.900,117.200,182.500,,,,,,
D8,native-sqp,false,true,Converged,1.3314384049638411e-13,0,209.800,320.400,116.300,152.400,,,,,,
D8,libslvs,false,true,Converged,3.03567956331771e-11,0,202558.500,260748.000,199015.300,256821.600,,,,,,
D8,ezpz,false,true,Converged,4.2472885118346534e-11,,255.200,482.100,148.300,231.500,,,,,,
D9,native,false,true,Converged,0e0,0,161.800,225.400,92.000,117.500,,,,,,
D9,native-fast,false,true,Converged,0e0,,153.000,275.100,87.400,101.400,,,,,,
D9,native-dogleg,false,true,Converged,0e0,0,162.400,317.500,90.100,106.500,,,,,,
D9,native-sqp,false,true,Converged,0e0,0,113.900,179.400,91.200,118.200,,,,,,
D9,libslvs,false,true,Converged,0e0,0,810.000,1493.100,650.800,1261.600,,,,,,
D9,ezpz,false,true,Converged,1.4210854715202004e-14,,324.000,459.800,163.600,232.000,,,,,,
D10,native,false,true,Converged,0e0,0,3.900,4.200,3.500,3.600,,,,,,
D10,native-fast,false,true,Converged,0e0,,4.600,4.700,3.300,4.200,,,,,,
D10,native-dogleg,false,true,Converged,0e0,0,4.100,4.200,3.700,3.800,,,,,,
D10,native-sqp,false,true,Converged,0e0,0,4.000,4.100,3.500,3.600,,,,,,
D10,libslvs,false,true,Converged,0e0,0,43.700,55.000,74.400,81.800,,,,,,
D10,ezpz,false,true,Converged,1.106013865413368e-18,,16.700,31.800,14.700,14.900,,,,,,
```

The per-case corpus verdicts (`sk13_corpus.csv`) are written by the same command.
