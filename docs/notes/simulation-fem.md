# Simulation (FEA): pipeline, discretisation, constraints, solvers and verification

Design note for [S13](../specs/S13-simulation.md). Written during S13 refinement
(2026-11-08). It covers:
- the study workflow and how Fusion behaves (what we copy, what we change);
- the pipeline from document bodies to a solved model, and which crate owns each step;
- tet meshing with fTetWild, surface tags and TET10 promotion;
- how boundary conditions, loads and bonded contacts become equations;
- element formulation, assembly, solvers and stress recovery;
- the result file, out-of-date tracking and the field export to S08/S14;
- the verification suite with target values.

Status: **Written** (refinement output; revisit after FEM-02 and FEM-04 land).

---

## 1. Units and conventions

| Thing | Convention |
|---|---|
| Unit system | The `amg-core` canonical system (`docs/notes/units-and-quantities.md`): mm, t (tonne), s. So force N, stress MPa (N/mm²), energy mJ, density t/mm³, acceleration mm/s². Material values arrive as `amg_core::units` quantities and are read with `.get()` (canonical). No conversion inside `amg-fem`. |
| Frequency | Hz in results; the eigen solver works with ω² in rad²/s². |
| Temperature | K. Thermal loads ΔT in K (or `delta_degC`, S00). |
| Voigt order | `(11, 22, 33, 23, 13, 12)`, engineering shear strains, = `amg_materials::VOIGT_ORDER`. Never the CalculiX order `(11,22,33,12,13,23)`; the CalculiX export (FEM-25) maps explicitly. |
| Material axes | Axis 3 = build direction for printed materials (S17, S16 `d_local = R⁻¹·Z`). P1 is isotropic, so axes do not matter until FEM-22. |
| Stress sign | Tension positive. Principal stresses sorted σ1 ≥ σ2 ≥ σ3. |
| Precision | f64 everywhere on the CPU. The GPU path (FEM-08) runs f64 kernels where the device has `SHADER_F64`, else f32 inner iterations with f64 outer residuals (§8.3; ADR-004 note). Results are stored in f64. |

## 2. Fusion behaviour (prior art) and our choices

Sources: Autodesk Fusion help and tutorials (static stress, mesh settings, contacts, safety
factor; see the Refinement log for links), the AU class "Become a Fusion Simulation Expert in
60 Minutes", and a published Fusion study report (mesh settings block).

| Fusion | Amalgam |
|---|---|
| Study types: static stress, modal, thermal, thermal stress, structural buckling, nonlinear static, event, shape optimisation (S14). | Same list. Static and modal are P1; thermal, thermal stress, buckling, nonlinear are P2; event is P3. |
| A study is a node in the Simulation browser, not in the timeline. It copies the design into a "simplified" model you can edit without touching the design. | A study is an extension record (`fem.study`, S01 §11). P1 has no separate simplified model: the study picks bodies and can exclude some. P2 adds defeaturing inside the study (FEM-24). |
| Study materials default to "same as model", with per-body overrides. | Same. An unassigned body is an error (QUESTIONS #28, S17). |
| Constraints: fixed (per axis), pin (radial/axial/tangential), frictionless, prescribed displacement. | Same. Symmetry is frictionless on a planar face (the owner asked for it by name, QUESTIONS #3). |
| Loads: force, pressure, moment, bearing, remote force, hydrostatic, gravity, linear/angular acceleration, temperature. | Same minus hydrostatic (P2 later, not in a WP). UI priority from QUESTIONS #3: fixed/pin, force, pressure, bearing, gravity, symmetry first; remote force, moment, thermal next. |
| Load cases: several per study, each with its own loads and constraints. | Same. Load cases that share constraints share one factorisation (§8.1). |
| Contacts: automatic detection with a tolerance; default type bonded; separation/sliding are nonlinear. | Same. Bonded is P1 (tied constraints, §6.4). Separation, sliding and friction are P2 (FEM-21). |
| Mesh: "model-based size" 10 % by default, parabolic (quadratic) elements, min size 20 % of average, adjacent size ratio 1.5, curvature-based refinement; local mesh control on faces. | Same defaults: average size = 10 % of the **largest bbox side** of the study bodies (we use the side, not the diagonal, to match Fusion's slider numbers; verified by FEM-14 against an owner-exported Fusion mesh if available), min size 20 %, growth 1.5, TET10. |
| Adaptive mesh refinement: repeat solves on finer meshes until results converge. | P2 (FEM-23), driven by a Zienkiewicz–Zhu error estimate. |
| Safety factor from yield (ductile) or ultimate (brittle) strength. | Same (§9.2). |
| Pre-check: an icon showing "ready to solve" or which input is missing. | Same, as a pure function `Study::precheck` (FEM-01) that the UI and the CLI share. |
| Results: von Mises, principal stresses, displacement, strain, reaction forces, safety factor, contact pressure, mode shapes; probes; deformation animation; report. | Same (FEM-06, FEM-19). |
| Solved in the cloud (cost tokens, minutes of queueing). | Local, as an `amg-jobs` job with progress and cancel. |

## 3. Pipeline and ownership

```
 amg-features::sim (sees amg-doc, amg-brep, amg-mesh, amg-fem)          amg-fem (doc-agnostic)
 ──────────────────────────────────────────────────────────          ─────────────────────────
 Study (ExtRecord) ──► snapshot bodies (Evaluated) ──► SurfaceInput ──► TetMesher (fTetWild)
                         solid: BREP-08 tessellate,      per body:        │  TetMesh (TET4)
                                face id per triangle     positions,       ▼
                         mesh:  face group per tri       tris, patch ids  promote_tet10 + snap
                                                                          │  TetMesh (TET10)
 LoadCaseDef (EntityRef) ──► resolve (DOC-06 / S07) ──► GeomPatch ─────► map_patches → MeshRegion
                                                         (used by S14)    │
 materials (MAT-06) ─────────────────────────────────► MaterialTable ───► FeModel
                                                                          │
                                                                          ▼
                                                               solve_static / solve_modal …
                                                                          │  Solution
 store results (blob in ext/) ◄── StudyResults ◄──────────────────────────┘
 amg-ui::sim: fringe plots via RND-08, probes, animation, report
```

Rules:
- `amg-fem` never sees `amg-doc`, `amg-brep` or `EntityRef`. It works on surface meshes with
  **patch ids** (`PatchId(u32)`), which `amg-features::sim` assigns per (body, B-rep face) or
  (body, mesh face group). This keeps `amg-fem` usable from S14 and the CLI, and keeps the
  layering rule (architecture: `amg-fem` may depend on `amg-mesh`, `amg-materials`, plus
  core/jobs/compute).
- Geometry projection for mid-node snapping (§4.4) needs the exact surface. `amg-fem` takes a
  `SurfaceProjector` trait object; `amg-features::sim` implements it with
  `BrepKernel::project_point_on_face` (BREP-09) for solids and with `MeshSpatial::closest_point`
  (S07) for mesh bodies.
- The fTetWild FFI lives in a new `-sys` crate, `amg-tetwild-sys` (lane F), allowed by the
  architecture's FFI row ("fTetWild, etc. via *-sys crates") and ADR-008. It must be added to
  the roadmap lane F crate list and to the S00 layering allow-list (`amg-fem → amg-tetwild-sys`).

## 4. Tet meshing

### 4.1 Mesher choice
ADR-008 (accepted): fTetWild via FFI is the default; TetGen (AGPL) is the fallback for
quality-constrained meshing. Both sit behind cargo features (`tetwild`, `tetgen`).
Decided (owner, 2026-11-09, QUESTIONS #71): `MesherChoice::Auto` uses TetGen for clean closed input
(`tetgen` on in the owner's build, personal use per ADR-012) and fTetWild otherwise; FEM-02
measures both on the corpus and reports.

- **fTetWild** (Hu et al., SIGGRAPH 2020, MPL-2.0) works on triangle soups with
  self-intersections and gaps. Its output boundary lies within an envelope ε of the input
  (default ε = bbox diagonal / 1000), so the boundary is **not** the input triangulation. It
  needs GMP (on Windows, MPIR; LGPL-3.0, dynamic link). Its CMake fetches geogram, libigl and
  optionally TBB; FEM-02 builds it through a vcpkg overlay port with the vcpkg `tbb` (the same
  oneTBB as Manifold, ADR-003), so there is one TBB in the process.
- **TetGen** (AGPL-3.0) is a constrained Delaunay mesher. It keeps the input triangles exactly
  (with Steiner points) and is fast on clean, closed input. It fails on self-intersecting input.
- Gmsh/HXT (GPL-2.0+) was considered (parallel, very fast, conforms to the surface). It would
  need a new ADR. Owner decision 2026-11-09 (QUESTIONS #72): not now; decide on FEM-02's measurements.

`TetMesher` is a small trait so the backends are swappable and testable:

```rust
pub trait TetMesher: Send + Sync {
    fn name(&self) -> &'static str;
    fn mesh(&self, input: &SurfaceInput, size: &SizeField, ctx: &JobCtx) -> Result<TetMesh, FemError>;
}
```

Default choice per input (`MesherChoice::Auto`): TetGen when the `tetgen` feature is on and the
input is closed, manifold and free of self-intersections (S07 `AnalysisReport`); fTetWild
otherwise. Without the `tetgen` feature, always fTetWild. Measured times on the corpus decide
whether `Auto` stays this way (FEM-02 records them).

### 4.2 Surface input and patch tags
`SurfaceInput` is one closed triangle surface per body with a `PatchId` per triangle. For a
B-rep solid it comes from BREP-08 `TessMesh::to_indexed_parts()` (welded, watertight, a face id
per triangle) with the `TessQuality` chosen from the mesh size (linear deflection ≤ 0.1 × min
element size, angular ≤ 15°). For a mesh body it is the `IndexedMesh` with `FaceGroupId` per
face.

Tag transfer to the tet boundary is **backend-independent**: after meshing, every boundary
triangle of the tet mesh gets the patch of the input triangle nearest to its centroid (BVH, S07
`MeshSpatial::closest_point`). A boundary triangle whose three vertices are nearest to
different patches lies on a patch edge; it takes the patch of the centroid but its vertices are
marked "edge nodes" for edge/vertex BCs (§6.1). TetGen keeps the input triangles, so the
transfer is exact there; with fTetWild it is exact except on triangles straddling a feature edge
that fTetWild rounded inside its envelope, which the quality report counts (`ambiguous_tags`).

Sharp edges matter for BCs: fTetWild preserves them within ε but may cut them. FEM-02 sets
ε = min(1e-3 × diag, 0.05 × min element size) and the Done-when measures tag ambiguity.

### 4.3 Sizing
`SizeField` = global average size h, min size h_min = 0.2 h, growth 1.5, plus local controls:
- **Per patch** (Fusion "local mesh control"): target size on selected faces.
- **Curvature**: on a curved patch, h ≤ r·θ with θ = 30° (r = local radius from the B-rep or
  from mesh curvature). Small features: h ≤ half the smallest edge length of a patch, clamped
  to h_min.

fTetWild takes a target edge length (`-l`, relative) and a background sizing mesh
(`--bg-mesh`). The sizing field is evaluated on a background grid (an octree of cubes) and
passed as the background mesh. TetGen takes a size per input vertex (`-m` with `.mtr`). FEM-02
does global size only; FEM-14 adds local controls.

### 4.4 TET10 promotion and snapping
Quadratic tets are needed: linear TET4 locks in bending and needs ~10× more nodes for the same
error. Promotion adds one mid-node per edge (edge map keyed by sorted vertex pair, so shared
edges share nodes). Boundary mid-nodes are projected onto the exact surface of their patch
(`SurfaceProjector::project(patch, p)`). After snapping, each element is checked: the Jacobian
determinant at the 4 Gauss points and the 10 nodes must be > 0 and the Jacobian ratio
(min/max) > 0.05. Failing elements get their snapped mid-nodes moved back toward the straight
position by halving (up to 4 times), then left straight. The quality report counts curved,
partially straightened and straight-fallback elements.

### 4.5 Quality report
Per mesh: element and node counts, DOFs, min/median radius ratio (3 r_in / r_circ, 1 = regular),
min dihedral angle, max aspect ratio, Jacobian-ratio histogram, `ambiguous_tags`, and the
number of elements below the "poor" thresholds (radius ratio < 0.1, dihedral < 5°). The UI
shows it after meshing; the solver refuses to run if any element has det J ≤ 0.

## 5. Elements

- **TET10 solid** (isoparametric, quadratic). Stiffness with a 4-point degree-2 Keast rule
  (exact for straight-sided TET10). Consistent mass with an 11-point degree-4 rule. Body loads
  with the 4-point rule. Element routines are generic over the node count so TET4 (1-point rule)
  exists for tests and for the p-multigrid coarse level (§8.3).
- **TRI6 surface** (boundary faces of TET10) for pressure, traction and convection: 6-point
  degree-4 Dunavant rule (curved faces).
- **Thermal**: the same TET10 shape functions with one DOF per node (FEM-09).
- Quadrature tables are written out in `amg-fem` with a test that each rule integrates all
  monomials up to its degree exactly (1e-14). `fenris-quadrature` (MIT/Apache-2.0) is a
  reference for the tables; not a dependency.

## 6. Constraints, loads and contacts

### 6.1 Regions
`map_patches` turns each load-case `GeomPatch` (a set of patch ids, or an edge or vertex of a
patch boundary) into a `MeshRegion`: boundary facets (TRI6) and their nodes. Edge regions are
the mesh edges whose two end nodes are edge nodes of the two adjacent patches; vertex regions
are the single nearest edge node. Loads on edges and vertices are allowed with a warning
(singular stress), as in Fusion.

### 6.2 One constraint mechanism: u = T·û + g
All linear constraints (fixed, per-axis, pin, frictionless, symmetry, prescribed displacement,
bonded tie, rigid remote connections) are expressed as a sparse map from the free (reduced)
DOFs û to all DOFs u: `u = T û + g`. The reduced system is

    K̂ = Tᵀ K T,   f̂ = Tᵀ (f − K g)

which stays symmetric positive definite when the model is properly constrained. Each node gets
at most one **local frame** (identity, cylindrical for pins, face normal for frictionless). A
constrained local component is dropped from û (with its prescribed value in g); a tied node's
DOFs are replaced by a combination of master DOFs. Conflicts (a node fixed in two different
local frames, a tied node that is also fixed) are resolved by fixed > prescribed > pin >
frictionless > tie, and each resolution is reported as a warning with the region names.
Building T row-by-row is cheap; `TᵀKT` is done during assembly by transforming each element
matrix (most rows of T are identity), so K is never formed twice.

| Constraint | Local frame and dropped components |
|---|---|
| Fixed (all or per axis X/Y/Z in a chosen frame) | identity or user frame; the selected components |
| Pin (cylindrical face) | cylinder frame (radial, tangential, axial) from the fitted axis (B-rep `GeomClass::Cylinder` or mesh `Primitive::Cylinder`); any subset of radial / tangential / axial |
| Frictionless / symmetry | node normal (area-weighted average of adjacent facet normals of that region); normal component |
| Prescribed displacement | identity or user frame; value in g |

### 6.3 Loads
All loads become consistent nodal forces (integrated with the TRI6 or TET10 rules):

| Load | Formulation |
|---|---|
| Force on faces | uniform traction F / A over the region (A = curved area), direction fixed or normal. |
| Pressure | p · n on each facet (n outward, positive pressure pushes in). Not a follower load (linear). |
| Bearing | Fusion's bearing load on a cylindrical face: radial pressure p(θ) = p₀ cos θ over the half-cylinder facing the load direction, p₀ scaled so the resultant equals F (p₀ = 2F / (π r L) for a full half-cylinder). |
| Remote force / moment | applied at a point; distributed to the region like a distributing coupling (RBE3): forces f_i = w_i (F/W + (M_c × r_i) solved through the region's area inertia tensor), w_i = nodal tributary area. This adds no stiffness. A "rigid" variant (RBE2) is a tie in §6.2 to a 6-DOF reference node and is P2. |
| Gravity / linear acceleration | body force −ρ a over every element. |
| Angular velocity / acceleration | body force ρ (ω × (ω × r) + α × r) about an axis. P1 in the model, P2 in the UI. |
| Temperature (thermal stress) | thermal strain α ΔT per node (FEM-20). |

### 6.4 Bonded contacts
Bodies are meshed separately (each keeps its own material; meshes do not need to match).
Contact pairs come from `find_contacts`: patch pairs of different bodies whose facets are within
tolerance (default 0.1 × min element size, at least 0.01 mm) and whose normals are opposed
(cos ≤ −0.9), found with an AABB tree. For each pair, the slave side (the finer mesh) nodes are
projected onto the master TRI6 facets; each slave DOF becomes Σ N_k(ξ) · master DOF in T.
Unprojected slave nodes (outside the master within tolerance) stay free and are counted. This is
the classic node-to-surface tie (CalculiX `*TIE`, Abaqus `*TIE`); it passes the patch test for
matching planes and is accurate to the mesh resolution for curved interfaces.

### 6.5 Rigid-body detection
An unconstrained or under-constrained body shows up as a zero (or tiny) pivot in the Cholesky
factorisation. Before solving, `precheck` flags bodies with no constraint and no tie to a
constrained body (graph check, cheap). During the solve, a pivot below 1e-12 × max diagonal stops
with `FemError::Unconstrained { body, dof }`, mapped to the body name in the UI ("Body 2 can
move freely in Z; add a constraint"). Modal analysis allows free bodies (rigid modes are
reported as ~0 Hz).

## 7. Assembly

- Node numbering: reverse Cuthill–McKee per body on the node graph (cache-friendly; the direct
  solver reorders again with AMD/METIS-style ordering inside faer).
- Element matrices are computed in parallel (rayon, chunks of 4096 elements), transformed by the
  local T blocks, and scattered into a CSR pattern built once from the reduced-DOF graph (upper
  triangle only; symmetric). The pattern build is a two-pass count + fill with no locks;
  scattering uses per-chunk row ownership (colouring by row range) to avoid atomics.
- Memory estimate is computed before assembly (`FeModel::estimate`), so the UI can warn ("about
  6 GB needed") and pick the iterative path.

## 8. Solvers

### 8.1 Direct (P1 default, CMP-03)
Sparse Cholesky (faer, supernodal) on K̂. One factorisation serves every load case with the
same constraint set (multiple right-hand sides). Practical limit on the reference machine: about
1.5 M DOF for TET10 (fill grows superlinearly; the estimate decides). Budget: linear static
~500 k DOF < 30 s (vision), split: assembly ≤ 3 s, factor + solve ≤ 20 s, recovery ≤ 3 s, with
the mesh taken from the cache. Meshing has its own budget, < 30 s for the 120 k-tet model
(owner decision 2026-11-09, QUESTIONS #70), measured by FEM-02/FEM-14.

### 8.2 Eigen (modal, buckling; CMP-04)
Generalised problem K φ = ω² M φ. Shift-invert Lanczos with a shift σ < 0 (σ = −(2π · 1 Hz)²
by default) so free-free models factor and rigid modes come out near 0 Hz. Default 8 modes
(Fusion's default). Mass-normalised modes, participation factors and effective mass per
direction. Buckling solves K φ = −λ K_G(σ₀) φ with the stress from a static solve (FEM-10).

### 8.3 Iterative GPU (FEM-08, CMP-05)
For models past the direct limit. Precision follows the ADR-004 note (S15 OQ 1): everything below is generic over `R: GpuReal`,
f64 where the device reports `SHADER_F64` (in practice Vulkan), else f32 plus refinement. Both paths are tested.
- PCG on the GPU (`R = f32` on DX12/WARP, `R = f64` on Vulkan) with a **p-multigrid** preconditioner: smoother on TET10 (Chebyshev-
  Jacobi, 2 sweeps), coarse level = the TET4 mesh of the same vertices (≈ 1/7 the DOFs), solved
  by a CPU Cholesky in f64 (or, if the TET4 level is still too big, by smoothed-aggregation AMG
  with the 6 rigid-body modes as near-null space, in a later step).
- On the f32 path, mixed-precision iterative refinement outside: residual r = f − K u in f64 on the
  CPU, correction solved on the GPU in f32, until ‖r‖/‖f‖ < 1e-8. The f64 path converges to
  1e-8 directly (normally one outer step).
- The matrix stays in CSR (`R` values, u32 columns) on the GPU. Matrix-free element-by-element
  products are a later optimisation.
Target: 2 M DOF static < 60 s; result matches the direct solve within 1e-6 (relative energy
norm) on models where both fit.

### 8.4 Nonlinear (P2, FEM-11, FEM-21)
Newton–Raphson with load stepping and automatic cut-back; total Lagrangian for large
displacement; J2 plasticity with bilinear isotropic hardening (S17 `Plasticity::Bilinear`) and
radial-return mapping, consistent tangent; contact (FEM-21) with node-to-surface augmented
Lagrangian and Coulomb friction (regularised). Each step reuses the direct or iterative solver.

### 8.5 Explicit (P3, FEM-12)
Central differences with lumped mass (row-sum for TET10 is not positive; use the HRZ lumping),
critical time step from the smallest element (Courant), optional mass scaling to a target
time step, GPU kernels for internal forces.

## 9. Results

### 9.1 Fields
Per load case (static) or per mode: nodal displacement (3), and stress and strain at Gauss
points. Nodal stress = average of element values extrapolated from the 4 Gauss points to the
corner nodes (TET4 extrapolation matrix) and interpolated to mid-nodes; averaging is per body
(never across bodies with different materials). Derived on demand: von Mises, principal stresses
and directions, max shear, safety factor, strain energy density. Reactions = K u − f on
constrained DOFs, summed per constraint region (force and moment about the region centroid).

### 9.2 Safety factor
- `VonMisesYield` (default, ductile): SF = σ_y / σ_vM. If the material has no yield strength,
  falls back to ultimate tensile with a warning.
- `MaxPrincipalUltimate` (brittle, resins): SF = σ_ult / max(σ1, 0) in tension and
  σ_ult,c / |min(σ3, 0)| in compression if a compressive value exists.
- Directional strengths (printed materials) in P1: `Directional::min()` (conservative) with the
  S17 `IsoPolicy` warning. FEM-22 adds Tsai-Wu with the build direction.
SF is capped at 15 for display (Fusion shows "15+").

### 9.3 Storage
Results are binary blobs in the `.amg` `ext/` directory (DOC-07), referenced from the study
record's `blobs`. Format `AMGFEMR1`: header (version, counts, blake3 of the payload), mesh
(nodes f64, TET10 connectivity u32, body and patch ids), per load case or mode the nodal
displacement (f64) and nodal stress tensors (f64 × 6), plus Gauss-point stress (f32, for
re-averaging) and reactions. Compressed by the zip container (deflate). A 500 k DOF static
result is about 40 MB raw. Results can be dropped on save ("save without results") and are
recomputed on demand.

### 9.4 Display
`FieldSurface` = the boundary TRI6 facets split into 4 linear triangles each (corner +
mid-nodes), per-vertex scalar values and displacement vectors in f32. This is what RND-08
`upload_scalar_field` / `upload_vector_field` and `Shading::Field` take (S13 OQ 3, from S10).
Deformation scale "auto" = 5 % of the bbox diagonal / max displacement (Fusion's "adjusted").

### 9.5 Field export for S08 and S14
`TetFieldSampler` locates points in the tet mesh (BVH over tets, barycentric test, then the
quadratic shape functions) and returns the nodal field value; outside the mesh it returns
`None`. `resample_grid` fills a regular grid (origin, spacing, dims) in parallel. S08 IMP-07
(stress-driven lattice density) calls it through `amg-features`, which sees both crates; S14
uses the same for validation of outcomes. The field is in the study bodies' world frame.

### 9.6 Out-of-date tracking
A study stores `inputs_hash` at solve time: blake3 over the study record's canonical JSON
(minus results and UI state), the geometry key of each study body (the producing feature's
`CacheKey` from `Evaluated`, or `IndexedMesh::content_hash` for baked/mesh bodies) and the
`Material::content_hash` of each used material. `Study::status(design, eval)` recomputes the hash
cheaply and returns `OutOfDate` when it differs; the browser shows the badge. Mesh cache: the
mesh is keyed by (geometry keys, mesh settings, mesher name and version) and reused across
solves and load-case edits.

## 10. Verification suite (`tests/golden/fem/`)

Every case is a small `.amg` or programmatic model plus an expected value and tolerance. The
golden harness runs in release mode in CI (nightly for the large ones).

| Case | Quantity | Target | Tol | WP |
|---|---|---|---|---|
| Patch test (TET10, arbitrary mesh of a cube, linear displacement field) | stress | exact constant | 1e-10 rel | FEM-04 |
| Cantilever 100×10×10 mm, steel, tip load 100 N | tip deflection | Timoshenko beam FL³/(3EI) + FL/(κGA) | 1 % | FEM-04 |
| NAFEMS LE1 elliptic membrane (solid, thickness 0.1 m in the benchmark units) | σyy at D | 92.7 MPa | 2 % | FEM-04 |
| NAFEMS LE10 thick plate, 1 MPa on top | σyy at D | −5.38 MPa | 2 % | FEM-04 |
| Plate with hole in tension, d/W = 0.1, t = d/10 | Kt (gross) | 3.02 (Pilkey Chart 4.1 fit: Kt,n = 3 − 3.14 r + 3.667 r² − 1.527 r³, r = d/W) | 3 % | FEM-04 |
| Two bonded blocks vs one block (same geometry) | tip deflection | equal | 0.5 % | FEM-16 |
| Free-free steel bar 200×10×10 mm | first bending, first axial | β₁L = 4.730; f = n/(2L) √(E/ρ) | 1 % | FEM-07 |
| NAFEMS FV32 cantilevered tapered membrane | modes 1–6 | 44.623, 130.03, 162.70, 246.05, 379.90, 391.44 Hz | 3 % | FEM-07 |
| 1D slab, fixed T both ends | T(x) | linear | 1e-8 abs | FEM-09 |
| Pin fin with convection, adiabatic tip | tip T | cosh(m(L−x))/cosh(mL) | 1 % | FEM-09 |
| NAFEMS LE11 solid cylinder/taper, thermal load | σzz at A | −105 MPa | 3 % | FEM-20 |
| Fixed-free Euler column, L/r = 100 | first λ | π²EI/(4L²) | 2 % | FEM-10 |
| Large-deflection cantilever (elastica), PL²/EI = 2 | tip vertical deflection / L | 0.4934 (Bisshopp–Drucker) | 2 % | FEM-11 |
| Thick cylinder, internal pressure, elastic–perfectly plastic | pressure at full yield | (2σ_y/√3) ln(b/a) | 2 % | FEM-11 |
| Hertz sphere on flat (frictionless) | contact radius, peak pressure | Hertz | 5 % | FEM-21 |
| Elastic wave in a bar (explicit) | wave speed | √(E/ρ) | 2 %; energy drift < 1 % | FEM-12 |

Where possible, the owner also exports a Fusion study result (max displacement, max von Mises)
for 3 corpus parts; these are recorded as reference values with a 10 % tolerance (Fusion's mesh
differs) and are informative, not gating.

## 11. Corpus (`testdata/fem/`)

`testdata/fem/corpus/` holds ≥ 40 bodies: 20 STEP parts from the S02 fixture set (brackets,
housings, thin walls, fillets, threads cut as helical sweeps), 10 meshes from the MESH-28 corpus
(scans, generative outcomes, lattices excluded), and 10 printed-part STLs with small features.
"Meshes without user intervention" means: default settings, no error, every element det J > 0,
`ambiguous_tags` < 1 % of boundary facets, and the volume within 1 % of the input volume.

## 12. Open points for the implementing WPs
- FEM-02 confirms the fTetWild build on Windows (MPIR DLL, vcpkg overlay, TBB unification) and
  whether fTetWild's library API (`floatTetWild::tetrahedralization`) can be called in-process
  without its CLI, including `--bg-mesh` sizing. If in-process calls are not possible, the
  fallback is spawning the fTetWild binary with `.msh` I/O (slower, but isolates crashes).
- FEM-04 measures the faer supernodal Cholesky on a 500 k DOF TET10 model and records fill and
  time in this note.
- FEM-08 records the p-multigrid iteration counts and the GPU/CPU time split.
