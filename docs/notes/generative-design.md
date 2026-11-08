# Generative design: voxel FE, SIMP pipeline, AM filter and outcome conversion

Design note for [S14](../specs/S14-generative-design.md). Written during S14 refinement
(2026-11-08). It covers:
- how Fusion's generative design behaves and what we copy;
- the pipeline from a study record to an outcome body, and which crate owns each step;
- the design-space grid and how bodies become voxels;
- the matrix-free H8 solver on the GPU, its multigrid preconditioner and its precision;
- how S13 load cases become voxel boundary conditions;
- the optimisation formulation (SIMP, filters, projection, OC/MMA, stress, displacement);
- the additive overhang filter;
- outcome conversion to a watertight body and the validation FEA;
- the study record, outcome storage and jobs;
- verification targets and budgets.

Status: **Written** (refinement output; revisit after GEN-03 and GEN-04 land).

---

## 1. Units and conventions

| Thing | Convention |
|---|---|
| Units | The `amg-core` canonical system (mm, t, s → N, MPa, mJ), as in S13 (`docs/notes/simulation-fem.md` §1). |
| Grid frame | Axis-aligned with the world (design) frame. Element `(i, j, k)` spans `origin + (i, j, k) ⊙ h` to `origin + (i+1, j+1, k+1) ⊙ h`; x is fastest in every flat array. Nodes are `(i, j, k)` with `0 ≤ i ≤ nx`. |
| Voxel size | `h = [hx, hy, hz]`. `hx = hy = hz` except for additive studies with α ≠ 45° (§7.2). |
| Density | `x` = design variable per element, `x̃` = filtered, `x̄` = projected ("physical") density. Outcomes store `x̄`. |
| Overhang angle | S16 convention (`docs/notes/am-build-prep.md` §1.1): slope from horizontal, a downward face needs support when `−n·d > cos α`. Fusion's 45° default is the same in both conventions. |
| Build direction | One of ±X, ±Y, ±Z (Fusion offers the same six; owner decision 2026-11-09, QUESTIONS #91: six now, arbitrary directions later via a rotated grid frame, GEN-20). |
| Precision | `R: GpuReal` on the GPU: f64 where the device has `SHADER_F64` (in practice Vulkan), else f32 (ADR-004 note, S15 OQ 1); f64 on the CPU reference and in the optimiser (MMA/OC). |

## 2. Fusion behaviour (prior art) and our choices

| Fusion | Ours |
|---|---|
| Design space: preserve geometry, obstacle geometry, starting shape, symmetry planes; obstacle and preserve offsets. Loads and constraints may only sit on preserve geometry. | Same. The precheck refuses targets that are not on a preserve body. Offsets are per body. Symmetry planes must be axis-aligned (§3.4). |
| Design bounds come from preserves, obstacles and the starting shape. | Same default (bbox of preserves, starting shape and load targets, padded 10 % per side, clipped to obstacles), plus an optional explicit box. |
| Load cases: structural constraints and loads, several cases. | The S13 load-case model (`LoadCaseDef`, FEM-13), reused unchanged. Gravity is a design-dependent load (§6.5). |
| Objectives: minimise mass, or maximise stiffness with a mass target. Limits: safety factor (2 in Autodesk tutorials), displacement, modal frequency (P2). | `Objective::{MinMass, MaxStiffness { mass_target }}`, `Limits { safety_factor, max_displacement, min_frequency }`. Default safety factor 2.0. |
| Manufacturing: unrestricted, additive (build direction ±X/±Y/±Z or all, overhang angle default 45°, minimum thickness), 2.5/3/5-axis milling, 2-axis cutting, die casting. | Unrestricted and additive in P1; minimum thickness maps to the filter radius in P1 and is guaranteed by the robust formulation in P2 (GEN-09). Milling, 2-axis and casting are P3 (GEN-12). "All directions" is one outcome per direction (six outcomes). |
| Several materials × methods → several outcomes, solved in the cloud; outcome explorer with thumbnails, properties table, scatter plot, filters, compare, iteration slider. | Same set of outcomes, solved locally one at a time on the GPU (§10.3). Explorer in GEN-08 (P2). |
| Resolution slider (100 steps, default 40 of 100). | `Resolution::{Voxels(n), Size(h)}`. Default target 1 M voxels for the final run, with a 125 k "quick preview" button (owner, QUESTIONS #92). |
| Outcome → "Create design" (T-spline / B-rep / mesh), "Export mesh". | Outcome → mesh body (default) or implicit body through S08 (§8), SubD/B-rep later via S09. The body is a baked feature, not linked live to the study (§8.4). |
| Fusion's solver is level-set based (Autodesk research). | SIMP baseline (best documented, simplest to verify against the classics); level set is GEN-11 (P3). |

Sources: Autodesk help "Additive manufacturing parameters" (GD-ADDITIVE-REF) and the Fusion blog
"Achieving better additive manufacturing outcomes in generative design" (build direction ±X/Y/Z
or all, overhang angle 45° default, minimum thickness); Autodesk University "Advanced Generative
Design Manufacturing 2021" (resolution slider, obstacle offsets); Autodesk forum threads on preserve
bodies thinner than a voxel. Details in the S14 refinement log.

## 3. Pipeline and crate ownership

```
Design + Evaluated + gen.study record
   │ amg-features::sim::generative (document side)
   ├─ bodies → triangle meshes (BREP-08 tessellation / mesh bodies) → MeshSpatial → MeshSdf (IMP-03)
   ├─ study_geometry (FEM-13) + resolve_load_case → amg_fem::model::LoadCase per case
   ├─ target_triangles (FEM-13) per BC/load target
   ▼
amg_topopt (document-agnostic)
   ├─ grid::build_grid       DesignSpace → DesignGrid (cell states, symmetry)       GEN-02
   ├─ bc::voxel_case         LoadCase + triangles → VoxelCase (node constraints, loads) GEN-15
   ├─ fe::VoxelFe            matrix-free H8 + MG-PCG, CPU and GPU backends         GEN-14, GEN-03
   ├─ opt::optimize          SIMP + filter + projection + OC/MMA (+ AM filter)     GEN-04/05/06/16
   ├─ store                  outcome blob AMGGENO1                                 GEN-18
   └─ convert                density → SDF graph (S08) → mesh (IMP-04)             GEN-07
   ▼
amg-features::sim::generative
   ├─ RunOutcome job → SetExt(outcome header) + blob                              GEN-18
   ├─ gen.outcome_body feature (baked mesh or implicit body)                       GEN-07
   └─ validate_outcome: TET10 FEA of the converted body (amg-fem)                  GEN-17
amg-ui::generative: workspace, dialogs, explorer                                  GEN-13, GEN-08
```

`amg-topopt` depends on `amg-core`, `amg-jobs`, `amg-compute`, `amg-fem` (material and load-case
types), `amg-implicit` (conversion) and `amg-mesh` (spatial queries), which is exactly the
architecture's allow-list and the existing `Cargo.toml`. It never sees `amg-doc`. The document side
lives in `amg-features::sim::generative`, a submodule of lane F's `amg-features::sim`, so no lane
or crate change is needed.

### 3.1 Why voxels and not the S13 tet mesh
A regular grid gives one element stiffness matrix for the whole model, a matrix-free operator
with no assembly, perfect memory access for the GPU and a geometric multigrid hierarchy for free
(Wu, Dick, Westermann 2016; Aage et al. 2017). Tets would need assembly per iteration and an
algebraic or p-multigrid. The cost is a staircase boundary, which is why every outcome is converted
and re-checked with the S13 tet solver (§9).

## 4. Design-space grid (GEN-02)

### 4.1 Inputs
Every body arrives as a closed triangle mesh in world coordinates: solid bodies tessellated at a
deflection of `h/4` (BREP-08), mesh bodies as they are, implicit bodies meshed first (IMP-04 at
`h/2`). Each becomes a `MeshSpatial` (MESH-03) and a `MeshSdf` (IMP-03) with band `max(offset) + 2h`.

### 4.2 Grid placement
- Bounds: the explicit box if given; otherwise the bbox of preserves, the starting shape and every
  load target, padded by 10 % of the largest side per side.
- `h` from the resolution: `Size(h)` directly, or `Voxels(n)` → `h = (V_box / n)^(1/3)`.
- The origin is shifted (bounds grow by less than one cell) so every symmetry plane lies on a
  node plane, and dims are made even across each symmetry plane.
- dims are rounded up so each MG level halves cleanly: each dim is a multiple of `2^(L−1)` where
  `L` is the level count (§5.3). The extra cells are `Void`.

### 4.3 Cell classification (element centres, in this order)
| State | Rule | Optimised |
|---|---|---|
| `Preserve` | `d_preserve(c) ≤ offset + 0.5·h_min` (dilated half a cell so thin plates still get cells) | no, `x̄ = 1` |
| `Obstacle` | `d_obstacle(c) ≤ offset` | no, `x̄ = 0` |
| `Design` | inside the bounds, not above, and (if a starting shape is given and "restrict to starting shape" is set) inside it | yes |
| `Void` | everything else (padding) | no, `x̄ = 0` |

A preserve cell that is also inside an obstacle is a precheck error naming both bodies. A preserve
thinner than `2h` (measured by the share of its cells with a non-preserve neighbour on both sides)
gives a warning: Fusion fails silently in that case (forum reports), we say why.

Initial densities: the volume target for `Design` cells, or 1 inside the starting shape and the
volume target elsewhere.

### 4.4 Symmetry
Fusion lets the user pick symmetry planes. We accept up to three axis-aligned planes (origin planes
or offset construction planes parallel to them). The optimiser works on the reduced variable set:
mirrored elements share one variable and their sensitivities are summed. Loads need not be
symmetric (Fusion behaves the same). A tilted plane is a precheck error; a rotated grid frame is
left for later as GEN-20 (it also serves arbitrary build directions, QUESTIONS #91).

## 5. Voxel FE solver (GEN-14 CPU reference, GEN-03 GPU)

### 5.1 Element
Trilinear 8-node hexahedron, 3 DOF per node, 2×2×2 Gauss integration, isotropic material. One
24×24 `Ke` for a unit-modulus box `hx × hy × hz` (computed once in f64, stored f32 for the GPU).
SIMP: `E_e = E_min + x̄_e^p (E_0 − E_min)`, `p = 3`. Full integration locks slightly in bending;
with ≥ 4 elements through a member the error is a few per cent, which the validation FEA covers.

### 5.2 Operator
Matrix-free, node-wise (Dick/Wu "stencil" form): each node gathers the 27 neighbouring nodes'
displacements through the 8 adjacent elements' `E_e · Ke` blocks. No atomics and no scatter, so
each output DOF is written by one thread and the result is deterministic. Nodes whose 8 adjacent
cells are all `Void`/`Obstacle` are inactive (u = 0, excluded from dot products); this keeps large
obstacle regions free.

### 5.3 Preconditioner and Krylov method
PCG (CMP-05 driver) preconditioned with one geometric multigrid V-cycle (Amir, Aage, Lazarov 2014,
"On multigrid-CG for efficient topology optimization"):
- levels: halve the grid until the coarse grid has ≤ 40 k DOF (≈ 4–6 levels for 1–8 M voxels);
- coarse operators by re-discretisation: each coarse element's modulus is the mean of its 8 fine
  children's `E_e` (Wu 2016). Galerkin coarse operators are better but need stored matrices;
  re-discretisation keeps every level matrix-free;
- transfer: trilinear prolongation, restriction = its transpose;
- smoother: 2 pre and 2 post Chebyshev–Jacobi sweeps (degree 2, eigenvalue bound from 10 power
  iterations on the Jacobi-scaled operator, recomputed every 10 design iterations);
- coarsest level: CPU sparse Cholesky (CMP-03), symbolic once, numeric refactor per design
  iteration (≤ 40 k DOF: milliseconds).

### 5.4 Constraints
Every linear constraint is a per-node projector `P_n` (3×3, symmetric, idempotent): fixed axes zero
their components, frictionless and symmetry remove the normal component, pin keeps only the axial
(and optionally tangential) component in the cylinder frame. The solver works on `P K P + (I − P)`,
which is SPD; loads are projected too. Prescribed non-zero displacement is not supported by the
voxel solver (precheck error; Fusion's generative design does not offer it either). Coarse levels
get a constraint mask: a coarse node is fixed in an axis if any of its fine children is (cheap and
good enough for a preconditioner).

### 5.5 Precision
GPU precision is `R: GpuReal` (f64 where `SHADER_F64`, else f32; both paths tested, the f64 path through S15's local gate). `E_min = 1e-6 E_0` (the classic 1e-9 gives contrasts f32 PCG cannot resolve, and
the difference in compliance is below 0.1 %). Stop at relative residual `1e-4` in the optimisation
loop (warm-started from the previous design iteration's `u`; Amir 2014 shows loose tolerances do not
change the optimum), `1e-6` for the final metrics. The CPU reference uses f64 with the same `E_min`
so GPU and CPU results compare directly.

### 5.6 Memory (1 M voxels ≈ 1.03 M nodes, 3.1 M DOF)
Per f32 DOF vector 12.4 MB; PCG needs 5 (u, r, z, p, Kp) plus 2 per MG level for the smoother and
transfer (+15 % total). With densities, sensitivities and filter buffers: ≈ 150 MB at 1 M voxels,
≈ 1.2 GB at 8 M voxels. The study precheck estimates this and refuses when it exceeds 70 % of the
adapter's reported memory.

### 5.7 Backend
`fe::Backend::{Cpu, Gpu(Arc<Gpu>)}` behind a small internal trait (ADR-004 amendment: a CUDA path
can be added later without touching callers). The CPU backend is the f64 reference used by every
test and the fallback when no adapter is available (rayon, ≈ 30–60× slower).

## 6. Optimisation (GEN-04, GEN-05, GEN-16)

### 6.1 Chain per iteration
`x → (symmetry expand) → density filter → x̃ → Heaviside projection → x̄ → (AM filter, §7) →
FE solves → objective, constraints → adjoint sensitivities back through the chain → OC or MMA →
new x`.

### 6.2 Filter and projection
- Density filter (Bruns & Tortorelli 2001; Bourdin 2001): linear cone weights `max(0, r − ‖c_i − c_j‖)`.
  Explicit stencil on the GPU for `r ≤ 4h` (≤ 257 neighbours); a Helmholtz PDE filter (Lazarov &
  Sigmund 2011) solved with the same MG on one scalar field for larger radii. Default
  `r = max(1.5 h, t_min / 2)` where `t_min` is the additive minimum thickness.
- Heaviside projection (Guest 2004; Wang, Lazarov, Sigmund 2011 tanh form) with threshold
  `η = 0.5` and `β` continuation 1, 2, 4, 8, 16, doubling every 20 iterations or when the design
  change drops below 0.01, whichever is later. The default iteration limit is 100 (the vision
  budget's count).
- Grey measure `M_nd = 4/n Σ x̄(1 − x̄)` reported per iteration.

### 6.3 Optimisers
- **OC** (optimality criteria, bisection on the Lagrange multiplier, move 0.2, damping 0.5) for
  the single-constraint case (max stiffness with a mass target). Cheapest.
- **MMA** (Svanberg 1987, with the 2007 notes' asymptote rules and the dual solved by a
  primal-dual interior point on the `m`-dimensional dual). Own implementation in `amg_topopt::mma`
  (CPU, f64, rayon over the `n` design variables); no maintained Rust crate covers MMA. Used for
  everything with more than one constraint, stress or displacement.

### 6.4 Objectives and constraints
- **Max stiffness:** minimise `Σ w_i c_i` (compliance per load case, equal weights by default) s.t.
  mass ≤ target. Fusion's mass target can be absolute or a fraction; both map to a volume fraction
  of the `Design` cells after subtracting the preserve mass.
- **Min mass:** minimise mass s.t. for every load case: stress constraint (§6.6) and, if set, the
  displacement limit. All load cases must satisfy all limits (Fusion's behaviour).
- **Displacement limit:** p-norm (`P = 16`) of `‖u_n‖` over the nodes of preserve cells and loaded
  nodes. That is where the maximum sits for a well-supported part; the validation FEA reports the
  true maximum. One adjoint solve per case.
- **Frequency** (P2, GEN-19): first eigenfrequency ≥ limit, LOBPCG with the MG preconditioner,
  the Pedersen (2000) low-density mass modification against localised modes.

### 6.5 Gravity
Design-dependent body force `f_e = x̄_e ρ m_e g / 8` per node. It adds a term to the compliance
sensitivity (Bruyneel & Duysinx 2005). With SIMP the ratio of stiffness to self-weight goes to
zero at low density, so gravity-loaded designs use a RAMP-like low-density correction for the
load term only. Tested by finite differences (GEN-05).

### 6.6 Stress (GEN-16)
Element von Mises at the centroid, `qp`-relaxed (`σ_e^rel = x̄_e^q σ_e`, `q = 0.5`; Le et al.
2010). One p-norm (`P = 8`) per load case with adaptive normalisation (`σ_max ≈ c σ_PN`, `c`
updated each iteration from the previous ratio, Le 2010). Allowable `σ_allow = σ_y / SF` from the
material (`SolidMaterial::yield_strength`; a material without yield is a precheck error for min
mass). Preserve cells are included (bolt bosses are where stress peaks), void and obstacle cells
are not. One adjoint solve per case per iteration, so stress studies cost about twice a compliance
study. The vision budgets accordingly are 5 min (max stiffness), 10 min (stress-constrained) and
15 min (robust), decided by the owner on 2026-11-09 (QUESTIONS #93).

### 6.7 Termination and status
`Converged`: final β reached, design change < 0.01 for 3 iterations and all constraints satisfied
within 1 %. `IterationLimit`: limit reached (the outcome is still usable; Fusion shows "Completed"
for this case). `Infeasible`: constraints still violated by > 5 % at the limit (for example a
safety factor no design in the space reaches); the outcome is kept and marked.

### 6.8 Determinism
Fixed-order tree reductions (CMP-01), no float atomics, a fixed iteration schedule. Two runs on the
same machine and driver give bit-identical densities. Across GPUs only the metrics tolerances apply.

## 7. Additive overhang filter (GEN-06)

### 7.1 Method
Langelaar's AM filter (Langelaar 2016, *Additive Manufacturing* 12; 2017, SMO 55): layer by layer
along the build direction, a cell's printed density is `min(x̃, smax(support))` where the support
is the cells directly below it and their 4 in-plane face neighbours (the 5-cell "plus" stencil).
Smooth min/max with the paper's `P = 40`, `ε = 1e-4`, `Q = P + log(5)/log(0.5)`. The bottom layer
(the build plate side of the bounds) is supported. Sensitivities by the reverse layer sweep. Each
layer is parallel; layers are sequential (`n_build` dispatches, ≈ 100–200).

The plus stencil is conservative: along the grid axes it allows exactly `α`, along diagonals it
needs a slightly steeper wall, so it never lets through an overhang a slicer would support.

### 7.2 Angle
The stencil fixes the self-supporting slope at `atan(h_build / h_plane)`. For `α ≠ 45°` the grid is
built with `h_build = h · tan α` (the angle "can only be tuned by the element aspect ratio",
van de Ven et al. 2018). Allowed range 30°–60° (aspect 0.58–1.73; outside it the H8 aspect ratio
hurts accuracy). Arbitrary angles on cubic cells need the front-propagation filter (van de Ven et
al. 2018 SMO 57, 2020 CMAME 369), which is the documented upgrade path if the owner needs angles
outside the range.

### 7.3 Interplay
Preserves are given; an overhanging preserve still needs support and is reported (not the
optimiser's fault). Obstacles count as empty. Symmetry planes perpendicular to the build axis are a
precheck error (a mirrored part cannot be self-supporting in both directions).

### 7.4 Check
The filter's promise is checked on the converted mesh with S16's `classify_faces` (AM-02) at the
same `α` and build direction: overhang area outside preserve faces and outside the contact layer.
Smoothing (§8) rounds the staircase, so the test allows 2 % of the downward area.

## 8. Outcome → body (GEN-07)

### 8.1 Density to signed distance
1. Resample `x̄` (cell centres) to a cubic grid of spacing `h_min / 2` by trilinear interpolation
   (also removes the anisotropy of additive grids, since `SampledGrid::from_dense` takes one cell
   size).
2. `φ0 = (0.5 − x̄) · s` near the 0.5 crossing, where `s` is the local gradient scale, then
   fast-sweeping redistancing (Zhao 2005) in a band of 4 cells, so `φ` is a true signed distance
   (S08 assumes distances, its Lipschitz bounds depend on it).
3. Optional Gaussian smoothing of `φ` (σ from the "smoothing" slider, 0 to 1.5 h) in the band.
4. `SampledGrid::from_dense(origin, h_min/2, dims, φ)` → `Node::Grid`.

### 8.2 Composition with exact preserves and obstacles
The voxelised preserve is only accurate to a cell; bolt holes and mating faces must be exact. So:
`body = ((grid ∪_b preserve_sdfs) − (obstacle_sdfs ⊕ offset))` as an S08 `SdfGraph`: smooth union
with blend `b = h` (fillets at the junctions, like Fusion's smooth outcomes) and a sharp
difference with the obstacles. Preserve and obstacle SDFs are the same `MeshSdf`s used for
voxelisation.

### 8.3 Meshing
`amg_implicit::to_mesh` (IMP-04) at cell `h_min / 2`, sharp features on (keeps preserve edges),
decimation tolerance `0.25 h`. IMP-04 guarantees closed, oriented, manifold output.

### 8.4 Document side
"Create body from outcome" adds a `gen.outcome_body` feature with the result as a baked payload
(the S12 "feature + baked payload in one undo entry" mechanism). The feature records the study key,
outcome id, the outcome blob hash and the conversion options, so it can be re-run with other
options (`EditFeature` + new payload) but it does not follow later study edits, because rerunning
an optimisation is not a recompute. Output kind: mesh body (default) or implicit body
(`BodyGeom::Implicit` holding the graph). Owner decision 2026-11-09 (QUESTIONS #90): mesh by default,
implicit one click away.

## 9. Validation FEA (GEN-17)

The converted body is meshed and solved with S13 (FEM-02 tet mesh, FEM-14 TET10, FEM-04 static),
built directly with `amg-fem` types:
- patches: each output triangle whose centroid lies within `0.25 h` of a load-case target triangle
  (`target_triangles`, FEM-13) and whose normal agrees within 30° gets that target's `PatchId`;
  the rest get a single "free" patch;
- the load case's `Constraint`/`Load` items are re-targeted to those patches;
- material: the outcome's material (`SolidMaterial::from_material`).

The report (max von Mises, min safety factor, max displacement, mass, per case) is stored in the
outcome header next to the voxel model's prediction, with the relative difference. Validation runs
on demand and as part of "Create body", not automatically when an outcome finishes (owner, QUESTIONS #94,
2026-11-09).

## 10. Study record, outcomes and jobs (GEN-01, GEN-18, GEN-08)

### 10.1 Record
`ExtRecord` of kind `gen.study`, version 1, whole-record `Command::SetExt`, exactly like
`fem.study` (S13) and the S16 print setup. It holds the design space (`BodyId`s with offsets),
symmetry, load cases (`Vec<LoadCaseDef>` from FEM-13), objective, limits, manufacturing methods,
materials, resolution, solver settings and outcome headers.

### 10.2 Outcomes
One outcome per (material, method). Outcome header: id, material, method, status, metrics,
iteration count, `inputs_hash`, blob ref, optional validation report. The blob (`AMGGENO1`):
magic, version, grid frame, cell states (u8), final `x̄` (f32), iteration history (objective,
volume, max constraint, β, grey per iteration), and in P2 snapshots every 10 iterations quantised to u8 (no new dependency) for
the explorer's iteration slider. ≈ 5 MB per 1 M-voxel outcome without snapshots.

`inputs_hash` = blake3 over the study JSON (minus outcomes and UI state), the geometry key of every
preserve/obstacle/starting-shape body (S01 request `Evaluated::body_key`, already made by S13) and
the material content hashes. A changed input marks outcomes `OutOfDate` (kept, badged), like S13.

### 10.3 Jobs
`RunOutcome` (an `amg_jobs::Job`) builds the problem, optimises and returns a `SetExt` with the new
header plus the blob bytes, exactly like S13's `RunStudy`. Phases: voxelise (5 %), map BCs (2 %),
optimise (90 %, per iteration), store (3 %). A live preview callback hands the UI a density
snapshot every 5 iterations (the UI meshes it at `2h` for display). Outcomes run one at a time
(one GPU; a queue in `amg-features::sim::generative`), each its own job so cancelling one keeps
the others.

## 11. Verification targets (`tests/golden/topopt/`)

| Case | Quantity | Target | Tol | WP |
|---|---|---|---|---|
| H8 patch test (uniform strain) | stress | exact | 1e-10 | GEN-14 |
| Cantilever 200×20×20 voxels, tip load | tip deflection | Timoshenko beam | 3 % | GEN-14 |
| Same, GPU vs CPU f64 | displacement, energy norm | CPU reference | 1e-4 rel. | GEN-03 |
| 3D cantilever 60×20×4, vf 0.3, p 3, r 1.5 (top3d defaults, Liu & Tovar 2014) | compliance, volume | GNU Octave run of `top3d.m` with our `E_min` and filter | 2 %, 0.1 % | GEN-04 |
| 3D MBB half beam 96×32×16 with symmetry BC, vf 0.5 | compliance | Octave run of `top3D125.m` (Ferrari & Sigmund 2020) with matching settings | 2 % | GEN-04 |
| Two-case cantilever (down + sideways) | worst-case compliance | ≤ each single-case design's worst case | — | GEN-05 |
| L-bracket (Le et al. 2010 geometry, extruded), min mass s.t. σ ≤ σ_allow | max σ (voxel) | ≤ 1.05 σ_allow, below the compliance design's at equal mass | — | GEN-16 |
| Cantilever, build +Z, α = 45° and 55° | overhang area on the converted mesh (AM-02) | ≤ 2 % of downward area outside preserves | — | GEN-06 |
| Cantilever outcome converted | closed, manifold, volume vs Σ x̄ V | closed, 0 non-manifold edges, 3 % | — | GEN-07 |
| Validation FEA of the converted cantilever | max displacement | voxel prediction | 10 % | GEN-17 |

Reference compliances are not copied from papers (they depend on `E_min`, filter type and
iteration count); the GEN-04 implementer runs the published MATLAB codes in GNU Octave with our
settings and commits the script and the numbers to `testdata/topopt/ref/`.

## 12. Budgets

| Scenario | Budget | Bench | WP |
|---|---|---|---|
| One static solve, 1 M voxels, from zero to 1e-4 | < 2 s | `budget/topopt-solve-1m` | GEN-03 |
| Max stiffness (min compliance), 1 M voxels, 100 iterations (vision) | < 5 min | `budget/topopt-1m-100it` | GEN-04 |
| Stress-constrained, 1 M voxels, 100 iterations (vision) | < 10 min | `budget/topopt-stress-1m` | GEN-16 |
| Robust min-size, 1 M voxels, 100 iterations (vision) | < 15 min | `budget/topopt-robust-1m` | GEN-09 |
| Filter + projection + sensitivities + OC per iteration, 1 M | < 0.2 s | part of the above | GEN-04 |
| AM filter forward + adjoint, 1 M | < 0.1 s | `topopt/am-filter-1m` | GEN-06 |
| Voxelisation of a 1 M-triangle preserve set at 1 M voxels | < 5 s | `topopt/voxelize-1m` | GEN-02 |
| Outcome conversion, 1 M voxels → mesh | < 20 s | `topopt/convert-1m` | GEN-07 |
| UI during a run | no frame > 16 ms | trace | GEN-13 |

Rough check of the solve budget: a matrix-free H8 product on 3.1 M DOF reads ≈ 100 MB and does
≈ 0.5 GFLOP, a few ms on the reference GPU; one MG V-cycle ≈ 6 products; 15–30 PCG iterations
→ 0.2–1 s. Wu 2016 report comparable figures on older hardware.

## 13. Alternatives considered

- **Assembled sparse matrix + CMP-05 Jacobi PCG:** memory 1–2 GB at 1 M voxels and iteration
  counts that grow with resolution; rejected for the multigrid route.
- **Level set (closer to Fusion):** crisp boundaries, but needs reinitialisation and hole
  nucleation, and verification against published numbers is harder. GEN-11 (P3).
- **Front-propagation overhang filter:** arbitrary angles on cubic cells and unstructured meshes;
  more complex than the AM filter. Upgrade path if 30°–60° is not enough.
- **Narrow-band sparse grids (Liu et al. 2018):** billions of voxels on a desktop. Not needed for
  the 8 M-voxel target; the dense grid keeps the code simple.
- **CUDA (`cudarc`):** ADR-004 says it is the first optimisation after the wgpu baseline is
  benchmarked; the backend trait (§5.7) leaves room for it.

## References
- E. Andreassen et al., "Efficient topology optimization in MATLAB using 88 lines of code",
  SMO 43 (2011); code at topopt.mek.dtu.dk.
- K. Liu, A. Tovar, "An efficient 3D topology optimization code written in Matlab", SMO 50 (2014).
- F. Ferrari, O. Sigmund, "A new generation 99 line Matlab code for compliance topology
  optimization and its extension to 3D", SMO 62 (2020).
- N. Aage et al., "Giga-voxel computational morphogenesis for structural design", Nature 550 (2017).
- J. Wu, C. Dick, R. Westermann, "A system for high-resolution topology optimization", IEEE TVCG
  22(3) (2016).
- O. Amir, N. Aage, B. Lazarov, "On multigrid-CG for efficient topology optimization", SMO 49 (2014).
- F. Wang, B. Lazarov, O. Sigmund, "On projection methods, convergence and robust formulations in
  topology optimization", SMO 43 (2011).
- J. Guest, J. Prévost, T. Belytschko, "Achieving minimum length scale in topology optimization
  using nodal design variables and projection functions", IJNME 61 (2004).
- B. Lazarov, O. Sigmund, "Filters in topology optimization based on Helmholtz-type differential
  equations", IJNME 86 (2011).
- K. Svanberg, "The method of moving asymptotes", IJNME 24 (1987); "MMA and GCMMA – two methods
  for nonlinear optimization" (2007 notes).
- C. Le et al., "Stress-based topology optimization for continua", SMO 41 (2010).
- M. Langelaar, "Topology optimization of 3D self-supporting structures for additive
  manufacturing", Additive Manufacturing 12 (2016); "An additive manufacturing filter for topology
  optimization of print-ready designs", SMO 55 (2017).
- E. van de Ven et al., "Continuous front propagation-based overhang control for topology
  optimization with additive manufacturing", SMO 57 (2018); "Overhang control based on front
  propagation in 3D topology optimization for additive manufacturing", CMAME 369 (2020).
- N. L. Pedersen, "Maximization of eigenvalues using topology optimization", SMO 20 (2000)
  (low-density mass modification).
- M. Bruyneel, P. Duysinx, "Note on topology optimization of continuum structures including
  self-weight", SMO 29 (2005).
- H. Zhao, "A fast sweeping method for Eikonal equations", Math. Comp. 74 (2005).
- H. Liu et al., "Narrow-band topology optimization on a sparsely populated grid", ACM TOG 37(6)
  (2018).
