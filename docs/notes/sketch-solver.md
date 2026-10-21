# Sketch constraint solver — design note

| | |
|---|---|
| **Spec** | [S03](../specs/S03-sketch.md) |
| **Decides** | ADR-007 (own Rust solver vs `libslvs` FFI) |
| **Produced by** | S03 refinement (2026-10-19); the benchmark it plans is SK-13 |
| **Needed before** | SK-03 (decision gate in roadmap §Decision gates) |
| **Status** | SK-13 measured (2026-10-21, [`sketch-solver-benchmark.md`](sketch-solver-benchmark.md)): the §5.4 rule selects option A. ADR-007 waits for the owner (QUESTIONS #50) |

## 1. Problem

A 2D sketch is a set of real parameters `x ∈ ℝⁿ` (point coordinates, radii, curve parameters)
and a set of equations `F(x) = 0` (geometric constraints plus driving dimensions). The solver must:

1. **Solve.** From the current configuration `x₀`, find a nearby `x` with `‖F(x)‖ ≈ 0`.
   "Nearby" matters: users expect the sketch not to flip or jump (minimum movement).
2. **Report DOF.** Give the number of remaining degrees of freedom and a per-entity status:
   under (blue), fully (black) or over/conflicting (red).
3. **Diagnose.** For redundant (consistent) or conflicting (inconsistent) constraint sets,
   name the smallest group of constraints involved, so the UI can offer "delete / make driven".
4. **Drag.** At 120 Hz, move a dragged point toward the cursor while other geometry moves as
   little as possible and constraints stay satisfied.
5. **Stay fast and deterministic.** A 500-entity sketch must solve in < 5 ms after a small
   edit, and the same input must give bit-identical output.

## 2. Prior art

### 2.1 SolveSpace (`libslvs`), GPL-3.0-or-later
- **Source:** `src/system.cpp`, `src/constrainteq.cpp` and `src/slvs.h` / `include/slvs.h`
  (C API) in github.com/solvespace/solvespace.
- **Model:** parameters, entities built on them (point-in-2D, line segment, circle, arc of
  circle, cubic, workplane, normal…) and constraints that expand into symbolic expressions
  (`Expr`). Jacobians are computed **symbolically** by differentiating the expression trees.
- **Pipeline:**
  1. **Substitution.** Equations of the form `a − b = 0` (coincident points, horizontal,
     vertical) are removed by aliasing parameters. This shrinks typical systems a lot.
  2. **Newton with a least-squares minimum-norm step.** For a non-square `J`, the step is
     `dx = Jᵀ (J Jᵀ)⁻¹ F`. This is the minimum-norm Gauss–Newton step, so under-constrained
     sketches move as little as possible. Columns are scaled first, and dragged parameters get
     a different weight so they are the ones kept near the cursor.
     Current sources solve with Eigen `SparseQR` + COLAMD; older versions used dense arrays
     with a `MAX_UNKNOWNS` cap.
  3. **Rank test.** After convergence, the Jacobian rank is compared with the equation count.
     If it is short (redundant), SolveSpace finds the "failed" constraints by removing each
     constraint in turn and re-testing the rank (`calculateFaileds` in the C API).
  4. **Groups.** Sketch "groups" are solved in sequence, and earlier groups are constants
     for later ones. This is a user-level decomposition, not an automatic one.
- **C API result codes:** `SLVS_RESULT_OKAY`, `INCONSISTENT`, `DIDNT_CONVERGE`,
  `TOO_MANY_UNKNOWNS` (plus `REDUNDANT_OKAY` in newer sources), and a `dof` output.
- **Tests:** `test/constraint/<name>/` has 36 directories (angle, arc_line_tangent,
  cubic_line_tangent, curve_curve_tangent, diameter, equal_angle, equal_length_lines,
  equal_line_arc_len, equal_radius, eq_len_pt_line_d, eq_pt_ln_distances, horizontal,
  large_dimensions, length_difference, length_ratio, parallel, perpendicular, points_coincident,
  proj_pt_distance, pt_face_distance, pt_in_plane, pt_line_distance, pt_on_circle, pt_on_face,
  pt_on_line, pt_plane_distance, pt_pt_distance, same_orientation, symmetric, symmetric_horiz,
  symmetric_line, symmetric_vert, vertical, where_dragged, at_midpoint, comment). Each directory
  holds `.slvs` cases (text format; for example `normal.slvs`, `free_in_3d.slvs`,
  `reference.slvs`, `extended.slvs` with `_v20` / `_v22` variants), reference PNGs and a
  `test.cpp`.
- **Rust binding:** the crate `slvs` (thekakkun/rust_slvs, GPL-3.0, last release 0.6.0 in
  June 2023). It builds SolveSpace with `cc` and generates bindings with `bindgen`, so it
  needs libclang (`LIBCLANG_PATH` on Windows). Usage is small and maintenance is unclear.

### 2.2 FreeCAD PlaneGCS, LGPL-2.1-or-later
- **Source:** `src/Mod/Sketcher/App/planegcs/` (`GCS.cpp`, `Constraints.cpp`, `Geo.cpp`,
  `SubSystem.cpp`, `qp_eq.cpp`).
- **Model:** geometry types are point, line, circle, arc (with "arc rules" tying the endpoints
  to centre, radius and angles), ellipse (centre, focus, minor radius), arc of ellipse,
  hyperbola, parabola, B-spline (partial). Constraints are C++ classes with hand-written
  `error()` and `grad()`, which give **analytic gradients**.
- **Algorithms:** DogLeg (Powell trust region; the default), Levenberg–Marquardt and BFGS.
  When there are *temporary* constraints (the drag target), it switches to SQP (`qp_eq`): the
  temporary constraint is minimised as an objective while the real constraints are kept as
  equalities. That is how FreeCAD drags.
- **Decomposition:** the system is split into independent subsystems (connected components).
  Equality constraints are reduced by parameter grouping (similar to SolveSpace substitution).
- **DOF and diagnosis:** a QR decomposition of the Jacobian (Eigen dense
  `FullPivHouseholderQR` or `SparseQR`, selectable) gives the rank. Dependent rows are then
  mapped back to constraint groups, which are reported as **redundant**, **conflicting** or
  **partially redundant**. The FreeCAD UI shows these as clickable lists, which is a good model
  for our "sketch doctor".
- **Tests:** the C++ gtests in `tests/src/Mod/Sketcher/App/planegcs/` (`GCS.cpp`,
  `Constraints.cpp`) and the Python Sketcher solver tests. Also useful: the WebAssembly port
  `@salusoft89/planegcs`, whose README documents the primitive and constraint vocabulary.

### 2.3 Zoo `ezpz` (KittyCAD), MIT
- A pure-Rust 2D solver written for Zoo Design Studio's sketch mode (2025). It has a text
  problem format (`# constraints` / `# guesses`), `test_cases/` and a CLI benchmark. The
  README shows a 2000-variable problem solved in a mean of about 2.9 ms.
- **Relevance:** it is the only permissively licensed Rust prior art. Its test cases can be
  ported freely. It is young and tied to KCL, and its DOF and conflict diagnosis is unclear
  from public docs. SK-13 evaluates it as a third option (a dependency or a reference), not as
  a replacement plan.

### 2.4 Commercial
Siemens D-Cubed 2D DCM is licensed by most commercial MCAD systems (for example SolidWorks).
It is not available to us, but it is the behavioural reference that Fusion-style UX copies:
minimum-movement solving, chirality preservation (tangent arcs don't flip) and
"make dimension driven" on over-constraint.

### 2.5 Literature (decomposition and DOF analysis)
- **Structural decomposition.** Treat the system as a bipartite graph (equations × unknowns).
  A maximum matching (Hopcroft–Karp 1973) plus the **Dulmage–Mendelsohn** decomposition splits
  it into an over-determined part, a sequence of square well-determined blocks (the strongly
  connected components of the matched digraph, in topological order) and an under-determined
  part. The block triangular form follows Pothen & Fan, *Computing the block triangular form
  of a sparse matrix*, ACM TOMS 16(4), 1990. CAD use: Ait-Aoudia, Jegou & Michelucci,
  *Reduction of constraint systems*, Compugraphics 1993; Latham & Middleditch,
  *Connectivity analysis: a tool for processing geometric constraints*, CAD 28(11), 1996.
- **DR-planning.** Hoffmann, Lomonosov & Sitharam, *Decomposition plans for geometric
  constraint systems, Parts I & II*, J. Symbolic Computation 31, 2001. These are geometric
  rigid-cluster plans (Frontier algorithm). They are more powerful than DM on cyclic rigid
  structures, where DM gives one big block, but much more complex. Survey: Jermann,
  Trombettoni, Neveu & Mathis, *Decomposition of geometric constraint systems: a survey*,
  IJCGA 16(5–6), 2006.
- **Generic vs. numeric rank (witness configurations).** Structural methods assume generic
  positions. Numeric rank at a degenerate configuration (collinear construction, zero-length
  line) can be lower than the generic rank. Michelucci & Foufou, *Geometric constraint
  solving: the witness configuration method*, CAD 38(4), 2006, propose evaluating the
  Jacobian rank at a random "witness" configuration that satisfies the incidence structure.
  We use a lightweight version as a tie-breaker in diagnosis (§3.4).
- **Nonlinear least squares.** Nocedal & Wright, *Numerical Optimization* (2nd ed.), ch. 10–11:
  Gauss–Newton, Levenberg–Marquardt (Moré 1978) and Powell dogleg. Trust-region steps (LM or
  dogleg) are the standard fix for Newton overshoot near singular Jacobians.
- **Review.** Bettig & Hoffmann, *Geometric constraint solving in parametric computer-aided
  design*, JCISE 11(2), 2011.

## 3. Proposed native solver (option A)

### 3.1 Pipeline
```
Sketch ──build──▶ EquationSystem (params, residual fns, sparse analytic Jacobian)
       ──substitute──▶ aliased params (union-find on x_i = x_j and x_i = const)
       ──decompose──▶ connected components → Dulmage–Mendelsohn blocks per component
       ──solve blocks in topological order──▶ (well-determined blocks: Newton/LM;
                                               under-determined part: min-norm Gauss–Newton;
                                               over-determined part: least squares + flag)
       ──rank/DOF (optional)──▶ DofReport
       ──on failure (optional)──▶ Diagnosis (minimal conflict groups)
```
- **Substitution** handles coincident, horizontal and vertical (two points or a line), fix,
  concentric (via the centre points) and equal radius. Each substitution removes one equation
  and one unknown, so the DOF count is unchanged. A substitution that would alias two *locked*
  parameters with different values becomes a conflict, which goes to diagnosis.
- **Decomposition** first splits connected components (always valid), then applies DM inside
  each component. DM blocks are solved in order, and solved blocks become constants for later
  ones. DR-planning is **not** implemented in SK-03. It is revisited only if SK-03 benchmarks
  show a single DM block > 300 unknowns dominating time on real sketches.
- **The decomposition is cached** per structure revision (`Sketch::structure_rev`). Value-only
  edits (dimension change, drag) reuse the matching, block order and symbolic sparsity
  pattern.

### 3.2 Numerical method
- **Residuals.** Distances are in mm. Angular residuals are multiplied by a characteristic
  length `L_c` (sketch bbox diagonal, clamped to ≥ 1 mm) so all residuals share a unit and
  one tolerance applies.
- **Step.** Damped, weighted minimum-norm Gauss–Newton. Solve
  `(J W⁻¹ Jᵀ + λI) z = F`, then step `dx = −W⁻¹ Jᵀ z`, where `W` is a diagonal
  movement-cost matrix (1 for ordinary parameters, large for dragged parameters, §3.5), and
  `λ` is a Levenberg–Marquardt damping updated by the gain ratio (Nielsen's rule, start
  `λ = 0`, so the first step is pure Gauss–Newton as in SolveSpace).
- **Linear algebra.** Blocks with ≤ 64 unknowns use dense `nalgebra` (Cholesky of
  `J W⁻¹ Jᵀ + λI`, falling back to column-pivoted QR if it is not positive definite). Larger
  blocks use sparse `faer` (sparse Cholesky / QR with fill-reducing ordering). The threshold
  is tuned in SK-03.
- **Convergence.** `‖F‖∞ < 1e-10 mm` (scaled), with at most 50 iterations per block. If
  `‖F‖` stagnates (< 1 % decrease over 5 iterations), the block is reported as
  `DidNotConverge`. Default tolerances come from `amg_core::Tolerance` and are overridable in
  `SolveOptions`.
- **Chirality (no flips).** Tangency and angle equations are signed, using an orientation
  stored at constraint creation (`TangentKind::{External, Internal}`, line direction, angle
  branch). That way a solution with a mirrored arc is not a root of the system.
- **Determinism.** Parameters are ordered by `(EntityId, slot)`, equations by
  `(ConstraintId, row)`. There are no hash-map iterations, and no threading inside one solve.
- **Dogleg** (PlaneGCS default) is implemented as an alternative in SK-13 and compared. The
  winner becomes `Algorithm::Auto`.

### 3.3 DOF analysis (SK-04)
- At the solution, compute the rank `r` of the full (unsubstituted) Jacobian of the
  well/under-determined parts. Use rank-revealing column-pivoted QR (dense, `nalgebra`
  `ColPivQR`) per component. For components > 400 unknowns, use `faer` sparse QR.
  The rank tolerance is `τ = 1e-9 · max(m, n) · ‖J‖_max`.
- `DOF = n_free_params − r`.
- **Per-entity status.** The null space `N` of `J` (from the QR of `Jᵀ`, or SVD for small
  blocks) shows which parameters can still move. Parameter `i` is *free* iff
  `‖N[i, :]‖ > τ_N`. An entity is **Under** if any of its parameters is free, **Over** if it
  takes part in a redundancy or conflict group, and **Full** otherwise.
- **Cost.** DOF analysis runs only when asked for (`DofAnalysis::PerEntity`). The UI calls it
  after a solve settles, not on every drag step.

### 3.4 Redundancy and conflict diagnosis (SK-04)
1. A row of `J` (equation) is *dependent* iff it lies in the span of the other rows. A
   rank-revealing QR of `Jᵀ` with column pivoting gives the independent rows (the pivots).
   Each non-pivot row `k` is expressed as `J_k = Σ c_i J_i` by solving on the pivot rows.
   The equations with `|c_i| > τ` plus `k` form a **dependency group**.
2. Rows are mapped back to `ConstraintId`s (one constraint can own several rows). Groups are
   merged when they share constraints.
3. A group is **redundant** (consistent) if the residuals of its constraints are all
   satisfied, and **conflicting** otherwise.
4. **Witness tie-break.** If a group appears only because of a degenerate configuration
   (zero-length entity, three or more coincident-ish points), re-evaluate rank at a random
   perturbation of the free parameters (seeded, so deterministic). Groups that vanish there
   are reported as `DegenerateConfiguration`, not as over-constrained.
5. **Fallback.** If QR-based grouping gives a group larger than 12 constraints, refine it
   SolveSpace-style: try removing each member and keep those whose removal restores full row
   rank. That is O(group) rank tests and runs only on failure.
6. **Pre-check for new constraints** (`check_constraint`, used for auto-constraints and to
   give Fusion-style "would over-constrain" prompts): append the candidate's rows to the cached
   factorization and test whether the rank increases.

### 3.5 Dragging (SK-05)
- The drag target is applied by **setting the dragged parameters to the cursor position as
  the initial guess** and giving them a large movement cost in `W` (§3.2). The min-norm step
  then prefers to move every other parameter. This is SolveSpace's "dragged params" approach
  and needs no extra equations, so DOF is unchanged and no QP solver is needed.
- If the dragged point is fully constrained, the solve pulls it back to the constrained
  position (correct behaviour). If it is partly constrained (for example on a circle), it
  slides along the free direction, because only the free component of the cursor offset
  survives.
- **Warm start.** Each frame starts from the previous frame's solution. The structure cache
  is kept for the whole drag session.
- **Comparison.** PlaneGCS uses SQP with the cursor as a temporary objective. That is more
  general (it handles dragging an edge) but costs a QP per frame. SK-13 measures both on the
  4-bar and polyline cases. The weighted-min-norm approach is the default unless it is worse
  on smoothness (see §5.3 for the jitter metric).

### 3.6 Extensibility for S06 (3D joints, ASM-04)
The equation-system layer (`amg_sketch::solve::system`) is dimension-agnostic: parameters,
residual closures and sparse Jacobians. ASM-04 can reuse it with SE(3) parameterisations by
depending on `amg-sketch`, or by copying the module, which S06 already allows. SK-12 (3D
sketch) uses the same layer.

## 4. Options for ADR-007

| Criterion | A. Own Rust solver (§3) | B. `libslvs` FFI (feature `slvs`) | C. `ezpz` dependency |
|---|---|---|---|
| Licence / ADR-012 | Ours (permissive) | **GPL-3.0**: must be feature-gated, so a permissive build still needs A or C | MIT |
| Maturity | New code, so risk is high early | 15+ years in production, well tested | Young (2025), in production at Zoo |
| Entity coverage we need | Exactly what S03 defines | Points, lines, circles, arcs, cubics. **No ellipses, conics or general B-splines** | Points, lines, arcs, circles (check). Ellipse and spline support unclear |
| DOF per entity / conflict groups | Designed in (§3.3–3.4) | Total DOF + "failed" constraint list; no per-entity status | Unclear; likely needs to be added |
| Drag | Weighted min-norm (§3.5) | Built in (`dragged[]`) | Unclear |
| Analytic Jacobians | Hand-written + FD-checked | Symbolic (expression trees) | Yes (per its docs) |
| Build on Windows | Pure Rust | C++ via `cc` + bindgen/libclang; extra toolchain | Pure Rust |
| Data-model friction | None | Two models (ours ↔ slvs handles) kept in sync each solve | Mapping to its constraint vocabulary |
| Determinism control | Full | Good, but outside our control | Unknown |
| Reuse for 3D (S06, SK-12) | Same system layer | libslvs is natively 3D | 2D only |
| Effort to production | SK-03 + SK-04 + SK-05 ≈ 7–9 agent-days | ≈ 3–4 days for the binding + mapping, plus wrappers for per-entity DOF | ≈ 3 days + upstream gaps |

**Structural point.** ADR-012 requires GPL dependencies to be optional. So option B can never
be the *only* solver: a permissive build would have no sketcher. B is therefore either a test
oracle or an optional runtime fallback, never a replacement for A or C. That leaves the real
choice as **A vs C as the primary solver**, with B kept for differential testing.

**Provisional recommendation (confirmed by SK-13 on 2026-10-21, §5.5):** **A as primary.** Add **B behind
the `slvs` cargo feature, dev/test only**, as a differential oracle in the corpus runner
(`cargo nextest --features slvs`). Do not ship B as a runtime fallback unless SK-13 shows A
failing > 2 % of corpus cases that B solves. C is used as a source of MIT-licensed test cases
and a performance reference. Revisit C as a dependency only if A misses SK-13's targets by a
wide margin.

## 5. SK-13 prototype and benchmark plan (gives the owner the data to decide)

### 5.1 Deliverables
1. `amg-sketch` prototype native solver behind the internal module `solve::proto`. Scope:
   substitution + connected components + damped min-norm Gauss–Newton, dense `nalgebra`
   only, P0 entities only. This is a throwaway-quality but correct core. SK-03 hardens it
   (DM, sparse, caching).
2. **libslvs adapter** behind cargo feature `slvs` (dev-dependency `slvs = "0.6"`, GPL-3.0).
   It maps the prototype corpus format to libslvs handles and back. It is not part of default
   features or CI default jobs. If the `slvs` crate fails to build on MSVC, the fallback is a
   ~150-line `cc` build of `src/slvs/lib.cpp` + `system.cpp` + `expr.cpp` +
   `constrainteq.cpp` with a hand-written `extern "C"` declaration of `Slvs_Solve`.
3. **ezpz adapter** behind feature `ezpz-bench` (dev-dependency, MIT), only for cases its
   vocabulary covers.
4. A criterion bench `benches/solver_compare.rs` and a report
   `docs/notes/sketch-solver-benchmark.md` (tables below, raw CSV in
   `target/criterion` copied to the report appendix).
5. A proposed ADR-007 wording based on the results (delivered in the PR description for the
   coordinator; ADR edits stay with the owner).

### 5.2 The 10 decision cases (subset of the SK-14 corpus)
| # | Case | Size | What it stresses |
|---|---|---|---|
| D1 | Triangle with three distance dims + one fixed vertex + horizontal base | 3 lines | Rigid cycle (one DM block), basic Newton |
| D2 | Fully constrained rectangle (H/V, 2 dims, fixed corner) | 4 lines | Substitution, DOF = 0 |
| D3 | Slot: 2 lines + 2 tangent arcs, width + length dims | 4 entities | Tangency chirality |
| D4 | 50-segment polyline, endpoints coincident, nothing else, drag the middle vertex 100 frames | 50 lines | Under-constrained min-norm, drag smoothness |
| D5 | Rectangle + redundant parallel + redundant equal | 4 lines | Redundancy detection, consistent case |
| D6 | Triangle with dims 10, 10, 30 (violates the triangle inequality) | 3 lines | Conflict detection, non-convergence handling |
| D7 | Four-bar linkage (ground fixed, 3 lengths), drag the coupler point 200 frames through near-singular toggle positions | 4 lines | Drag near singularity, no flips |
| D8 | Circular pattern: 24 slots on a bolt circle with equal + angle + concentric | ~200 entities | Many identical blocks, decomposition payoff |
| D9 | Generated grid of 10×10 dimensioned rectangles sharing edges | ~500 entities | Scale; incremental edit of one dimension |
| D10 | SolveSpace `large_dimensions` + 1e-3 mm feature on a 1e5 mm part | small | Conditioning, scaling, tolerance |

### 5.3 Metrics per option per case
- **Correctness:** converged (`‖F‖∞ < 1e-10`), and the result is the "expected" root (D3/D7
  chirality preserved: compare with the reference configuration).
- **Time:** cold solve and warm (one dimension changed by 1 %) solve. Report median and p95
  over 200 runs on the reference machine.
- **Drag:** per-frame time and **jitter** = max over frames of
  `‖x_k − x_{k−1}‖ / ‖cursor_k − cursor_{k−1}‖` for non-dragged points (lower is smoother).
- **Diagnosis:** DOF count correct; size of the reported failing set vs the true minimal
  set (D5, D6).
- **Engineering:** lines of glue code, build time added, Windows build friction (yes/no
  notes).

### 5.4 Decision rule (proposed; owner signs off)
Choose **A as primary** if all of these hold:
1. A solves ≥ 9 of the 10 decision cases.
2. Warm solve on D9 is < 5 ms median.
3. Drag on D4/D7 is < 2 ms/frame p95 with jitter no worse than 1.5× libslvs.
4. Diagnosis on D5/D6 is at least as specific as libslvs.

If A misses (1) or (2) by more than 2×, evaluate C as primary. Keep B as the `slvs` dev/test
oracle in either case. A runtime libslvs fallback is added only by a separate owner decision.

### 5.5 SK-13 results (2026-10-21)
Full tables, method and raw CSV: [`sketch-solver-benchmark.md`](sketch-solver-benchmark.md).

| Rule | Threshold | A. native prototype | B. libslvs | C. ezpz |
|---|---|---|---|---|
| Decision cases solved | ≥ 9/10 | **10/10** | 10/10 | 9/10 |
| D9 warm solve, median | < 5 ms | **0.09 ms** | 0.65 ms | 0.16 ms |
| Drag p95, D4 / D7 | < 2 ms | **1.3 / 27 µs** | 23 / 390 µs | 25 / 770 µs |
| Drag jitter D7 (≤ 1.5× libslvs) | ≤ 158 | **66** (feasible SQP: 0.85) | 106 | 66 |
| Diagnosis D5 / D6 | ≥ libslvs | **both minimal circuits / minimal conflict set** | empty list / non-minimal pair | none |
| SK-14 corpus | — | **135/135** | 118/125 (10 inexpressible) | 121/128 (7 inexpressible) |

**Recommendation for ADR-007: option A (own Rust solver) as the only shipped solver; libslvs
stays a test-only oracle behind `slvs` (no corpus gap: 0 cases where libslvs beats A); ezpz stays a
test-case source and benchmark reference.** Proposed text: benchmark note §7 and QUESTIONS #50.

Consequences for later WPs:
- SK-03: `Algorithm::Auto` = LM (dogleg was equal or worse); the rank test is needed for
  `ConvergedRedundant`; per-circuit diagnosis (no merging of groups that share members) is more
  specific than libslvs and should be kept in SK-04.
- SK-05: weighted min-norm drag flips the D7 linkage at the toggle (68 flips, as do libslvs and
  ezpz); the feasible-SQP variant does not. SK-05 needs SQP (default or fallback).
- Building libslvs needs libclang ≤ 15 for the `slvs` crate's bindgen 0.64 (see the benchmark note
  §6); CI's `--all-features` jobs need that or must exclude `slvs`.

## 6. Risks
- **Hand-written Jacobians drift from residuals.** Mitigation: a mandatory
  finite-difference check per constraint (SK-02) plus a proptest over random configurations.
- **DM gives one huge block on cyclic sketches.** Mitigation: the sparse path; DR-planning
  only if measured.
- **Spurious redundancy at degenerate configurations.** Mitigation: the witness tie-break
  (§3.4.4).
- **libslvs GPL test data in the repo.** Ported `.slvs` cases are GPL-3 data. Keep them in
  `testdata/sketch/solvespace/` with a `NOTICE` and the upstream licence. They are dev-only,
  so they never enter a shipped build (consistent with ADR-012).
