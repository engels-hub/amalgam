# S14 — Generative design & shape optimization

| | |
|---|---|
| **Status** | Refined |
| **Priority** | P1 (core) / P2 (outcome explorer, extra constraints) |
| **Crates** | `amg-topopt`, `amg-features::sim::generative` (study record, jobs, outcome body), `amg-ui::generative` (Generative workspace) |
| **Lane** | F |
| **Depends on** | S13 (FEM-01 study pattern, FEM-13 load cases, FEM-04/FEM-14 validation FEA); S15 (CMP-01 kernels, CMP-03 coarse Cholesky, CMP-05 PCG driver, CMP-04 for GEN-19); S08 (IMP-03 `MeshSdf`, IMP-04 `to_mesh`, IMP-12 `SampledGrid`); S07 (MESH-03 `MeshSpatial`); S02 (BREP-08 tessellation); S01 (DOC-05 `SetExt`, DOC-07 blobs, DOC-08 baked payloads); S16 (AM-02, tests of GEN-06); S17 (MAT-06); S10/S11 (overlays, workspace) |
| **Parity** | `03-fusion-parity.md` §Generative design — Fusion's flagship cloud feature, local here |

Design note: [`docs/notes/generative-design.md`](../notes/generative-design.md) (Fusion behaviour,
grid, voxel FE and multigrid, BC mapping, optimisation, AM filter, conversion, verification,
budgets).

## Summary
Local, GPU-accelerated topology optimization wrapped in Fusion's generative design workflow:
define design space (preserve geometry, obstacle geometry, starting shape, symmetry planes),
load cases, objectives & limits, manufacturing constraints (**additive with build direction
and overhang angle** first), materials; generate multiple outcomes in the background;
explore/compare; convert an outcome into a smooth, watertight, editable body.

## Scope
**In:**
- Generative study as an extension record (`gen.study`), precheck, out-of-date tracking.
- Design-space voxel grid from preserves, obstacles (with offsets), starting shape, bounds,
  axis-aligned symmetry planes.
- S13 load cases (FEM-13) mapped to voxel node constraints and loads; gravity.
- Matrix-free H8 elasticity with geometric-multigrid PCG, GPU (wgpu) and CPU reference.
- SIMP + density filter + Heaviside projection; OC and MMA; max stiffness with a mass target,
  min mass with safety factor and displacement limits; several load cases.
- Additive overhang filter (build direction ±X/±Y/±Z, angle 30°–60°), minimum thickness.
- Outcome storage, background run jobs, live preview; outcome → watertight mesh or implicit body;
  validation FEA of the converted body.
- P2: outcome batches and explorer, robust min/max member size, frequency limit, shape
  optimisation of an existing body. P3: level set, milling/2-axis/casting constraints.

**Out:**
- The FE solver for tets, meshing, result display → S13 (reused for validation).
- Generic GPU kernels, PCG driver, sparse direct solver → S15.
- Implicit modelling, meshing of SDFs → S08 (used for conversion).
- Build orientation optimisation and supports → S16 (S14 only constrains overhang).
- Outcome → SubD/B-rep → S09 later. Arbitrary (non-axis) build directions and tilted symmetry
  planes: QUESTIONS #91.
- Fusion's "remove rigid body modes" (inertia relief): later, no WP.

## Design
Full detail and the reasoning are in the design note. Summary:

- **Layering.** `amg-topopt` is document-agnostic, like `amg-fem`: it takes triangle meshes,
  `amg_fem::model::LoadCase`s and target triangles, and returns outcomes. The document side
  (record, BC resolution, jobs, outcome body feature, validation) is
  `amg-features::sim::generative`, a submodule of lane F's `sim` module. `amg-ui::generative` is
  the workspace. No crate or lane change.
- **Discretization:** regular voxel grid, axis-aligned with the world, over the design-space
  bounds (default: bbox of preserves, starting shape and load targets, padded 10 %). Element
  centres classify as `Preserve` (dilated half a cell), `Obstacle` (with offset), `Design` or
  `Void` using `MeshSdf` (IMP-03). Symmetry planes lie on node planes; mirrored elements share a
  variable. Resolution by voxel count (default 1 M, QUESTIONS #92) or size; up to 8 M voxels.
- **Solver:** trilinear H8, one `Ke`, SIMP moduli (`p = 3`, `E_min = 1e-6 E_0`), matrix-free
  node-wise operator (deterministic, no atomics), PCG preconditioned by a geometric-multigrid
  V-cycle (re-discretised coarse levels, Chebyshev–Jacobi smoother, CPU Cholesky at ≤ 40 k DOF).
  f32 on the GPU, f64 on the CPU reference. Constraints are per-node projectors (fixed, pin,
  frictionless, symmetry); prescribed non-zero displacement is refused.
- **Method:** density filter (cone up to 4 h, Helmholtz PDE filter beyond), Heaviside projection
  with β continuation 1→16, 100 iterations by default. OC for max stiffness with one mass
  constraint; own MMA (Svanberg) for everything else. Level set is GEN-11 (P3).
- **Objectives / constraints:** max stiffness s.t. mass target (weighted compliance over load
  cases); min mass s.t. per-case stress p-norm (`qp`-relaxed von Mises, `σ_y / SF`) and an
  optional displacement p-norm limit; gravity as a design-dependent load; frequency limit P2.
- **Manufacturing constraints:** Langelaar AM filter (5-cell plus stencil, conservative) along the
  chosen build axis; angles other than 45° by voxel aspect `h_build = h tan α`; minimum thickness
  → filter radius (P1), robust eroded/dilated formulation for guaranteed min/max size (P2);
  milling/casting P3.
- **Outcomes:** one per (material × method), queued one at a time on the GPU, each a cancellable
  job storing a header in the record and a density blob (`AMGGENO1`). Live preview every 5
  iterations. Explorer (thumbnails, scatter, filters, compare, iteration slider) is P2.
- **Post-processing:** density → cubic grid → redistanced SDF → optional smoothing →
  `SampledGrid::from_dense` → smooth union with the exact preserve SDFs, minus offset obstacles →
  `to_mesh` (IMP-04, watertight) → baked `gen.outcome_body` feature (mesh body by default,
  implicit optional, QUESTIONS #90) → validation FEA with S13 (TET10) on the converted body.
- **Shape optimization (Fusion simulation feature):** the same machinery with the design space
  restricted to an existing body (remove material only), keep regions as preserves and a mass
  target; result shown as a keep/remove overlay (S10 ghost layer), convertible like an outcome.

## Interfaces
Provisional Rust signatures. Names may change during implementation, but these shapes are what
`amg-features`, `amg-ui`, `amg-script` and tests code against. Units are canonical (mm, t, s →
N, MPa). Types from S13 (`amg_fem::model::*`, `amg_features::sim::*`) and S08
(`amg_implicit::*`) are used as declared in those specs.

```rust
// ───────────────────────────── amg_topopt (crate root) ─────────────────────────────
// Deps (existing Cargo.toml): amg-core, amg-jobs, amg-compute, amg-fem, amg-implicit, amg-mesh.
use amg_core::{Aabb3, Point3, Vec3};
use amg_jobs::{Gpu, JobCtx};
use std::sync::Arc;

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum Axis { X, Y, Z }
/// A world axis with a sign: build directions and symmetry normals.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub struct AxisDir { pub axis: Axis, pub positive: bool }

pub mod grid {                                   // GEN-02
    use super::*;
    /// Axis-aligned element grid. Element (i,j,k) spans origin + (i,j,k)⊙h .. +1; x fastest.
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct GridFrame { pub origin: Point3, pub h: [f64; 3], pub dims: [u32; 3] }
    impl GridFrame {
        pub fn elem_count(&self) -> usize;
        pub fn node_count(&self) -> usize;
        pub fn elem_index(&self, ijk: [u32; 3]) -> usize;
        pub fn elem_center(&self, e: usize) -> Point3;
        pub fn elem_at(&self, p: Point3) -> Option<[u32; 3]>;
        pub fn bounds(&self) -> Aabb3;
    }
    #[repr(u8)]
    #[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
    pub enum Cell { Void = 0, Design = 1, Preserve = 2, Obstacle = 3 }
    /// Symmetry plane on a node plane of the grid (normal = axis).
    #[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
    pub struct SymmetryPlane { pub axis: Axis, pub node_plane: u32 }
    /// One closed body of the design space (world frame) with its offset.
    pub struct SpaceBody { pub sdf: Arc<amg_implicit::MeshSdf>, pub bounds: Aabb3, pub offset: f64, pub label: String }
    #[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub enum Resolution { Voxels(u64), Size(f64) }
    pub struct DesignSpace {
        pub preserves: Vec<SpaceBody>,
        pub obstacles: Vec<SpaceBody>,
        pub starting_shape: Option<SpaceBody>,
        pub restrict_to_starting_shape: bool,
        pub bounds: Option<Aabb3>,                // None = bbox of preserves, start, extra, +10 %
        pub extra_bounds: Vec<Aabb3>,             // load-target boxes from the caller
        /// World planes `x_axis = at`; must be axis-aligned (the caller checks tilt).
        pub symmetry: Vec<(Axis, f64)>,
        pub resolution: Resolution,
        /// Additive studies: build axis and overhang angle set the voxel aspect (note §7.2).
        pub build: Option<(AxisDir, f64 /* α, rad */)>,
        pub mg_levels: u32,                       // dims rounded to multiples of 2^(levels−1)
    }
    pub struct DesignGrid {
        pub frame: GridFrame,
        pub cells: Vec<Cell>,                     // len == elem_count
        pub initial: Vec<f32>,                    // initial densities (Design cells)
        pub symmetry: Vec<SymmetryPlane>,
        pub warnings: Vec<amg_fem::FemDiagnostic>, // thin preserves, clipped bounds
    }
    impl DesignGrid {
        pub fn count(&self, c: Cell) -> usize;
        /// Element → design variable (after symmetry reduction); `u32::MAX` for fixed cells.
        pub fn variable_map(&self) -> (Vec<u32>, usize);
    }
    pub fn build_grid(space: &DesignSpace, initial_fraction: f64, ctx: &JobCtx) -> Result<DesignGrid, TopoptError>;
}

pub mod bc {                                     // GEN-15
    use super::*;
    /// Per-node linear constraint as a projector onto the free subspace.
    #[derive(Copy, Clone, Debug, PartialEq)]
    pub enum NodeConstraint {
        Fixed { axes: u8 /* amg_fem::model::Axes bits, global frame */ },
        /// Remove the component along `n` (frictionless, symmetry).
        Normal { n: Vec3 },
        /// Keep only the component along `t` (pin with radial + tangential fixed).
        Line { t: Vec3 },
    }
    #[derive(Clone, Debug, Default)]
    pub struct VoxelCase {
        pub name: String,
        pub constraints: Vec<(u32 /* node */, NodeConstraint)>,
        pub loads: Vec<(u32 /* node */, [f64; 3] /* N */)>,
        pub gravity: Option<Vec3>,                // mm/s², design-dependent body load
        pub weight: f64,                          // compliance weight (1.0)
    }
    impl VoxelCase {
        pub fn resultant(&self, frame: &grid::GridFrame) -> (Vec3, Vec3 /* moment about origin */);
        /// Hash of the constraint set (cases with equal keys share MG coarse factorisations).
        pub fn constraint_key(&self) -> [u8; 32];
    }
    /// Maps an S13 load case onto the grid. `tris(target)` returns the world triangles of a
    /// target (FEM-13 `target_triangles`); `patch_info` supplies cylinder axes for pins.
    /// Only nodes of `Preserve` cells receive BCs; anything else is a diagnostic.
    pub fn voxel_case(g: &grid::DesignGrid, case: &amg_fem::model::LoadCase,
                      tris: &dyn Fn(&amg_fem::model::Target) -> Vec<[Point3; 3]>,
                      patch_info: &dyn Fn(&amg_fem::model::Target) -> Option<amg_fem::mesh::PatchInfo>)
        -> (VoxelCase, Vec<amg_fem::FemDiagnostic>);
}

pub mod fe {                                     // GEN-14 (CPU, Ke), GEN-03 (GPU, MG)
    use super::*;
    #[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct VoxelMaterial { pub e: f64, pub nu: f64, pub density: f64 /* t/mm³ */, pub yield_strength: Option<f64> }
    impl VoxelMaterial { pub fn from_solid(m: &amg_fem::model::SolidMaterial) -> Result<VoxelMaterial, TopoptError>; }
    /// 24×24 stiffness of a unit-modulus hx×hy×hz H8 (2×2×2 Gauss), node order (000,100,110,010,001,101,111,011).
    pub fn h8_stiffness(h: [f64; 3], nu: f64) -> [[f64; 24]; 24];
    #[derive(Clone)]
    pub enum Backend { Cpu, Gpu(Arc<Gpu>) }
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct FeSettings { pub rel_tol: f64 /* 1e-4 in loop */, pub final_tol: f64 /* 1e-6 */, pub max_iter: u32 /* 200 */,
                            pub e_min_ratio: f64 /* 1e-6 */, pub coarse_dofs: u32 /* 40_000 */, pub smoother_sweeps: u32 /* 2 */ }
    impl Default for FeSettings { fn default() -> Self; }
    #[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct SolveInfo { pub iterations: u32, pub rel_residual: f64, pub seconds: f64 }
    /// Linear elastic voxel model; displacements stay on the backend between calls.
    pub struct VoxelFe { /* grid, Ke, levels, per-case u, backend state */ }
    impl VoxelFe {
        pub fn new(g: &grid::DesignGrid, m: VoxelMaterial, cases: &[bc::VoxelCase], backend: Backend,
                   s: &FeSettings, ctx: &JobCtx) -> Result<VoxelFe, TopoptError>;
        /// Estimated device (or host) bytes, for the precheck.
        pub fn estimate_bytes(g: &grid::GridFrame, cases: usize, s: &FeSettings) -> u64;
        /// Physical densities x̄ per element (fixed cells included); rebuilds coarse levels.
        pub fn set_densities(&mut self, xbar: &[f32], penal: f64) -> Result<(), TopoptError>;
        /// Solves case `c`, warm-started from its last solution.
        pub fn solve(&mut self, c: usize, tol: f64, ctx: &JobCtx) -> Result<SolveInfo, TopoptError>;
        /// Solves K λ = rhs for an adjoint (stress, displacement); same operator as case `c`.
        pub fn solve_adjoint(&mut self, c: usize, rhs: &[f32], tol: f64, ctx: &JobCtx) -> Result<Vec<f32>, TopoptError>;
        /// Element strain energy uₑᵀ Ke uₑ (unit modulus) for case `c`.
        pub fn element_energy(&self, c: usize, out: &mut [f32]) -> Result<(), TopoptError>;
        pub fn compliance(&self, c: usize) -> f64;
        /// Centroid von Mises per element (unit relaxation; caller applies x̄^q).
        pub fn element_von_mises(&self, c: usize, out: &mut [f32]) -> Result<(), TopoptError>;
        pub fn displacement(&self, c: usize) -> Result<Vec<[f32; 3]>, TopoptError>;
    }
}

pub mod mma {                                    // GEN-04
    /// Svanberg's MMA for min f0(x) s.t. fi(x) ≤ 0, lo ≤ x ≤ hi; CPU, f64.
    #[derive(Clone, Debug, PartialEq)]
    pub struct MmaSettings { pub move_limit: f64 /* 0.2 */, pub asy_init: f64 /* 0.5 */, pub asy_incr: f64 /* 1.2 */, pub asy_decr: f64 /* 0.7 */ }
    pub struct Mma { /* n, m, asymptotes, history */ }
    impl Mma {
        pub fn new(n: usize, m: usize, s: MmaSettings) -> Mma;
        /// One outer iteration; `x` is updated in place. `dg[i]` is the gradient of constraint i.
        pub fn update(&mut self, x: &mut [f64], f0: f64, df0: &[f64], g: &[f64], dg: &[&[f64]], lo: &[f64], hi: &[f64]);
    }
}

pub mod opt {                                    // GEN-04, GEN-05, GEN-06, GEN-09, GEN-16, GEN-19
    use super::*;
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub enum MassTarget { Absolute(f64 /* t */), Fraction(f64 /* of design-space mass */) }
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub enum Objective { MaxStiffness { mass_target: MassTarget }, MinMass }
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct Limits {
        pub safety_factor: Option<f64>,           // 2.0 default; required for MinMass
        pub max_displacement: Option<f64>,        // mm, p-norm over preserve and loaded nodes
        pub min_frequency_hz: Option<f64>,        // GEN-19 (P2)
    }
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub enum Manufacturing {
        Unrestricted { min_thickness: Option<f64> },
        /// Overhang α from horizontal (S16 convention), 30°..60°.
        Additive { build: AxisDir, overhang: f64 /* rad */, min_thickness: f64 /* mm */ },
        // GEN-12 (P3): Milling { .. }, TwoAxis { .. }, DieCasting { .. }
    }
    #[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
    pub enum Optimizer { Auto, Oc, Mma }
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct OptSettings {
        pub penal: f64,                           // 3.0
        pub filter_radius: Option<f64>,           // mm; None = max(1.5 h, t_min / 2)
        pub beta_max: f64,                        // 16
        pub beta_every: u32,                      // 20
        pub max_iter: u32,                        // 100
        pub change_tol: f64,                      // 0.01
        pub optimizer: Optimizer,
        pub robust: Option<Robust>,               // GEN-09 (P2)
        pub fe: fe::FeSettings,
        pub preview_every: u32,                   // 5; 0 = no previews
    }
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct Robust { pub eta_delta: f64 /* 0.1 */, pub max_member: Option<f64> /* mm */ }
    impl Default for OptSettings { fn default() -> Self; }
    pub struct Problem {
        pub grid: Arc<grid::DesignGrid>,
        pub material: fe::VoxelMaterial,
        pub cases: Vec<bc::VoxelCase>,
        pub objective: Objective,
        pub limits: Limits,
        pub manufacturing: Manufacturing,
        pub settings: OptSettings,
    }
    impl Problem {
        /// Static checks: BCs present, MinMass has a yield strength, angle in range,
        /// symmetry not perpendicular to the build axis, memory estimate.
        pub fn validate(&self, available_bytes: Option<u64>) -> Vec<amg_fem::FemDiagnostic>;
    }
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct IterationSummary { pub iter: u32, pub objective: f64, pub mass: f64, pub max_constraint: f64,
                                  pub change: f64, pub beta: f64, pub grey: f64, pub seconds: f64 }
    pub struct Preview { pub iter: u32, pub xbar: Arc<[f32]> }
    #[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
    pub enum OutcomeStatus { Converged, IterationLimit, Infeasible { worst: String } }
    #[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct OutcomeMetrics {
        pub mass: f64, pub volume: f64,
        pub compliance: Vec<f64>,                 // per case, mJ
        pub max_von_mises: Vec<f64>,              // per case, voxel model, MPa
        pub min_safety_factor: f64,
        pub max_displacement: Vec<f64>,           // per case, mm
        pub first_frequency_hz: Option<f64>,      // GEN-19
    }
    pub struct Outcome {
        pub frame: grid::GridFrame,
        pub cells: Arc<[grid::Cell]>,
        pub xbar: Arc<[f32]>,                     // physical density per element
        pub status: OutcomeStatus,
        pub metrics: OutcomeMetrics,
        pub history: Vec<IterationSummary>,
        pub snapshots: Vec<(u32, Arc<[u8]>)>,      // GEN-08: every 10 iterations, x̄ quantised to u8
    }
    pub fn optimize(p: &Problem, backend: fe::Backend, preview: &mut dyn FnMut(Preview), ctx: &JobCtx)
        -> Result<Outcome, TopoptError>;
}

pub mod am {                                     // GEN-06 (used inside opt; public for tests)
    use super::*;
    /// Langelaar AM filter along `build` (plus stencil); `x` and `out` per element.
    pub fn am_filter(f: &grid::GridFrame, cells: &[grid::Cell], build: AxisDir, x: &[f32], out: &mut [f32]);
    /// Adjoint: maps ∂f/∂(printed) to ∂f/∂x.
    pub fn am_filter_adjoint(f: &grid::GridFrame, cells: &[grid::Cell], build: AxisDir, x: &[f32], d_out: &[f32], d_x: &mut [f32]);
}

pub mod convert {                                // GEN-07
    use super::*;
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct ConvertOpts {
        pub iso: f32,                             // 0.5
        pub smoothing: f64,                       // 0..1 → σ = 0..1.5 h
        pub cell_size: Option<f64>,               // None = h_min / 2
        pub blend: Option<f64>,                   // preserve smooth-union radius; None = h_min
    }
    impl Default for ConvertOpts { fn default() -> Self; }
    /// Density → redistanced SampledGrid → (∪_blend preserves) − (obstacles ⊕ offset).
    pub fn outcome_sdf(o: &opt::Outcome, preserves: &[grid::SpaceBody], obstacles: &[grid::SpaceBody],
                       opts: &ConvertOpts, ctx: &JobCtx) -> Result<amg_implicit::SdfGraph, TopoptError>;
    /// `to_mesh` (IMP-04) at the conversion cell size; closed, oriented, manifold.
    pub fn outcome_mesh(g: &amg_implicit::SdfGraph, opts: &ConvertOpts, gpu: Option<&amg_implicit::GpuEvaluator>,
                        ctx: &JobCtx) -> Result<(amg_mesh::IndexedMesh, amg_implicit::MeshReport), TopoptError>;
}

pub mod store {                                  // GEN-18: blob `AMGGENO1` (note §10.2)
    use super::*;
    pub const OUTCOME_FORMAT: u32 = 1;
    pub fn write<W: std::io::Write>(o: &opt::Outcome, w: W) -> Result<(), TopoptError>;
    pub fn read<R: std::io::Read>(r: R) -> Result<opt::Outcome, TopoptError>;
}

#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum TopoptError {
    #[error("design space is empty: {0}")] EmptySpace(String),
    #[error("preserve `{preserve}` overlaps obstacle `{obstacle}`")] PreserveInObstacle { preserve: String, obstacle: String },
    #[error("load case `{0}` has no constraint on the grid")] Unconstrained(String),
    #[error("overhang angle {0:.1}° outside 30°..60°")] AngleRange(f64),
    #[error("model needs about {need} bytes, {have} available")] OutOfMemory { need: u64, have: u64 },
    #[error("solver did not converge: {0}")] NoConvergence(String),
    #[error("GPU: {0}")] Gpu(String),               // device loss, wraps amg_compute errors
    #[error("conversion: {0}")] Convert(String),    // wraps amg_implicit errors
    #[error("outcome file: {0}")] Store(String),
    #[error("operation cancelled")] Cancelled,
}
impl From<amg_core::Cancelled> for TopoptError { fn from(_: amg_core::Cancelled) -> Self { TopoptError::Cancelled } }
```

```rust
// ─────────── amg_features::sim::generative (GEN-01, GEN-07, GEN-10, GEN-17, GEN-18) ───────────
use amg_doc::{BodyId, Command, Design, Evaluated, ExtKey, ExtRecord, ExtRegistration, MigrationError, FeatureId};
use amg_features::sim::{LoadCaseDef, StudyGeometry};
use amg_materials::MaterialId;

pub const GEN_STUDY_KIND: &str = "gen.study";
pub const GEN_STUDY_VERSION: u32 = 1;

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SpaceBodyDef { pub body: BodyId, pub offset: f64 /* mm, ≥ 0 */ }
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SymmetryDef { pub axis: amg_topopt::Axis, pub at: f64 /* world coordinate */, pub source: Option<amg_doc::EntityRef> }
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum GenKind {
    Generative,
    /// GEN-10: design space = this body, keep regions = preserves, remove material only.
    ShapeOptimization { body: BodyId },
}
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct GenStudy {
    pub name: String,
    pub kind: GenKind,
    pub preserves: Vec<SpaceBodyDef>,
    pub obstacles: Vec<SpaceBodyDef>,
    pub starting_shape: Option<BodyId>,
    pub restrict_to_starting_shape: bool,
    pub bounds: Option<amg_core::Aabb3>,
    pub symmetry: Vec<SymmetryDef>,
    pub cases: Vec<LoadCaseDef>,                 // FEM-13; targets must be on preserve bodies
    pub objective: amg_topopt::opt::Objective,
    pub limits: amg_topopt::opt::Limits,
    pub methods: Vec<amg_topopt::opt::Manufacturing>,  // ≥ 1
    pub materials: Vec<MaterialId>,              // ≥ 1; outcomes = materials × methods
    pub resolution: amg_topopt::grid::Resolution,
    pub settings: amg_topopt::opt::OptSettings,
    pub outcomes: Vec<OutcomeHeader>,
}
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct OutcomeHeader {
    pub id: u32,
    pub material: MaterialId,
    pub method: u32,                             // index into `methods`
    pub status: amg_topopt::opt::OutcomeStatus,
    pub metrics: amg_topopt::opt::OutcomeMetrics,
    pub iterations: u32,
    pub inputs_hash: String,                     // hex blake3 (note §10.2)
    pub blob: amg_doc::BlobRef,
    pub blob_hash: String,
    pub validation: Option<ValidationReport>,    // GEN-17
}
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ValidationReport {
    pub max_von_mises: Vec<f64>, pub min_safety_factor: f64, pub max_displacement: Vec<f64>, pub mass: f64,
    /// (validated − voxel) / voxel per metric, for the explorer.
    pub rel_diff_displacement: Vec<f64>, pub rel_diff_stress: Vec<f64>,
    pub diagnostics: Vec<amg_fem::FemDiagnostic>,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum OutcomeState { NotGenerated, Running, Done, OutOfDate, Failed(String) }

impl GenStudy {
    pub fn new(name: &str) -> GenStudy;                           // Fusion defaults (SF 2, additive +Z 45°, one case)
    pub fn to_record(&self) -> ExtRecord;                         // deterministic JSON
    pub fn from_record(r: &ExtRecord) -> Result<GenStudy, GenError>;
    pub fn command(&self, key: ExtKey) -> Command;                // whole-record SetExt
    /// Fusion-style precheck shared by UI and CLI (preserves, targets on preserves, materials
    /// with yield for MinMass, symmetry axis-aligned, angle range, memory estimate).
    pub fn precheck(&self, d: &Design, e: &Evaluated, gpu: Option<&amg_jobs::Gpu>) -> Vec<amg_fem::FemDiagnostic>;
    pub fn inputs_hash(&self, d: &Design, e: &Evaluated) -> String;
    pub fn outcome_state(&self, id: u32, d: &Design, e: &Evaluated) -> OutcomeState;
    /// (material, method) pairs that have no up-to-date outcome yet.
    pub fn pending(&self, d: &Design, e: &Evaluated) -> Vec<(MaterialId, u32)>;
}
pub fn new_gen_study_command(d: &Design, s: &GenStudy) -> (ExtKey, Command);   // AddExt
pub fn gen_studies(d: &Design) -> Vec<(ExtKey, Result<GenStudy, GenError>)>;

/// GEN-15 glue: document → amg_topopt problem for one (material, method).
pub fn build_problem(s: &GenStudy, material: MaterialId, method: u32, d: &Design, e: &Evaluated, ctx: &amg_jobs::JobCtx)
    -> Result<(amg_topopt::opt::Problem, ProblemBodies, Vec<amg_fem::FemDiagnostic>), GenError>;
/// Preserve/obstacle SDFs kept for conversion, plus the S13 geometry for validation.
pub struct ProblemBodies { pub preserves: Vec<amg_topopt::grid::SpaceBody>, pub obstacles: Vec<amg_topopt::grid::SpaceBody>,
                           pub geometry: Arc<StudyGeometry> }

/// GEN-18: one outcome as a background job.
pub struct RunOutcome { pub key: ExtKey, pub material: MaterialId, pub method: u32,
                        pub design: amg_doc::DesignSnapshot, pub eval: Arc<Evaluated>,
                        pub backend: amg_topopt::fe::Backend,
                        pub preview: Option<std::sync::mpsc::Sender<amg_topopt::opt::Preview>> }
impl amg_jobs::Job for RunOutcome {
    type Output = OutcomeOutput; type Error = GenError;
    fn label(&self) -> std::borrow::Cow<'static, str>;
    fn run(self, ctx: &amg_jobs::JobCtx) -> Result<OutcomeOutput, GenError>;
}
pub struct OutcomeOutput { pub command: Command /* SetExt with the new header */, pub blob: (amg_doc::BlobRef, Vec<u8>),
                           pub outcome: Arc<amg_topopt::opt::Outcome> }
/// GEN-08: runs `pending()` outcomes one after another; each is its own job.
pub struct OutcomeQueue { /* job handles, order */ }
impl OutcomeQueue {
    pub fn start(key: ExtKey, s: &GenStudy, design: amg_doc::DesignSnapshot, eval: Arc<Evaluated>, jobs: &amg_jobs::JobSystem,
                 backend: amg_topopt::fe::Backend) -> OutcomeQueue;
    pub fn cancel(&self, outcome: (MaterialId, u32));
    pub fn cancel_all(&self);
}
pub fn load_outcome(d: &Design, key: &ExtKey, id: u32, blobs: &dyn amg_doc::BlobSource, ctx: &amg_jobs::JobCtx)
    -> Result<amg_topopt::opt::Outcome, GenError>;

/// GEN-07: `gen.outcome_body` (v1). The body is a baked payload in slot 0; the feature records
/// where it came from so it can be re-converted with other options.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct OutcomeBodyFeature {
    pub study: ExtKey,
    pub outcome: u32,
    pub blob_hash: String,
    pub opts: amg_topopt::convert::ConvertOpts,
    pub output: OutcomeBodyKind,
    pub build: Option<amg_topopt::AxisDir>,      // additive outcomes: for S16 pre-orientation
}
#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum OutcomeBodyKind { Mesh, Implicit }
/// Job: load outcome → convert → `AddFeature` with the baked body (one undo entry).
pub struct CreateOutcomeBody { pub key: ExtKey, pub outcome: u32, pub opts: amg_topopt::convert::ConvertOpts,
                               pub output: OutcomeBodyKind, pub design: amg_doc::DesignSnapshot, pub eval: Arc<Evaluated> }
impl amg_jobs::Job for CreateOutcomeBody { type Output = Command; type Error = GenError;
    fn label(&self) -> std::borrow::Cow<'static, str>; fn run(self, ctx: &amg_jobs::JobCtx) -> Result<Command, GenError>; }

/// GEN-17: TET10 static FEA of a converted body with the outcome's load cases (note §9).
pub struct ValidateOutcome { pub key: ExtKey, pub outcome: u32, pub body: BodyId,
                             pub design: amg_doc::DesignSnapshot, pub eval: Arc<Evaluated> }
impl amg_jobs::Job for ValidateOutcome { type Output = Command /* SetExt with validation */; type Error = GenError;
    fn label(&self) -> std::borrow::Cow<'static, str>; fn run(self, ctx: &amg_jobs::JobCtx) -> Result<Command, GenError>; }

#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum GenError {
    #[error("generative study record is invalid: {0}")] Decode(String),
    #[error("generative study v{0} is newer than supported")] TooNew(u32),
    #[error("study has no preserve geometry")] NoPreserve,
    #[error("load target is not on a preserve body: {0}")] TargetNotPreserve(String),
    #[error("material {0:?} has no yield strength (needed for the safety factor)")] NoYield(MaterialId),
    #[error("symmetry plane is not parallel to an origin plane")] TiltedSymmetry,
    #[error("outcome {0} not found")] NoOutcome(u32),
    #[error(transparent)] Sim(#[from] amg_features::sim::SimError),
    #[error(transparent)] Topopt(#[from] amg_topopt::TopoptError),
    #[error("operation cancelled")] Cancelled,
}
inventory::submit! { ExtRegistration { kind: GEN_STUDY_KIND, version: GEN_STUDY_VERSION, migrate: migrate_gen_study } }
fn migrate_gen_study(from: u32, v: serde_json::Value) -> Result<serde_json::Value, MigrationError>;
// register_feature!(OutcomeBodyFeature, "gen.outcome_body", version = 1);
```

`amg-ui::generative` exposes no API to other crates. It registers (UI-02) the Generative workspace
and commands `gen.new_study`, `gen.preserve`, `gen.obstacle`, `gen.starting_shape`,
`gen.symmetry`, `gen.bounds`, `gen.objectives`, `gen.manufacturing`, `gen.materials`,
`gen.settings`, `gen.precheck`, `gen.generate`, `gen.cancel`, `gen.explore` (GEN-08),
`gen.create_body`, `gen.validate`; the structural constraint and load commands reuse the S13
dialogs (same lane) on the study's `LoadCaseDef`s. It adds a `BrowserProvider` (study tree:
design space, load cases, objectives, manufacturing, materials, outcomes) and an
`OverlayProvider` (preserve green, obstacle red with offset shell, starting shape yellow,
symmetry planes, BC glyphs, live-preview density surface, GEN-10 keep/remove ghosts). Shape
optimisation (GEN-10) is started from the Simulation workspace (`sim.shape_optimization`,
`amg-ui::sim`, same lane).

**Needed from other specs** (requests, not edits; listed in the refinement log):
- **S13 (same lane):** none new beyond what S13 already lists for S14 (`LoadCaseDef`,
  `study_geometry`, `resolve_load_case`, `target_triangles`, FEM-13; FEM-02/FEM-14/FEM-04 for
  validation). Additive: the FEM-05 BC dialogs take a `&mut LoadCaseDef` plus a target filter
  ("preserve bodies only"), so `amg-ui::generative` can reuse them; GEN-10 adds a
  `sim.shape_optimization` command to `amg-ui::sim`. Noted under S13 Interface changes.
- **S15** (being refined): CMP-05 PCG driver over a caller-supplied matrix-free operator and
  preconditioner on GPU f32 buffers with warm start and a relative-residual stop; CMP-01 typed
  buffers, dispatch and deterministic (fixed-order) reductions; CMP-03 Cholesky with separate
  symbolic and numeric phases (MG coarse level refactored every design iteration). GEN-02 does
  not need CMP-02 (dense arrays only). Noted under S15 Interface changes.
- **S08:** `SampledGrid::from_dense` receives signed distances in mm (negative inside) on a cubic
  grid; it must produce a `GridKind::Distance` grid (or take a `kind` argument). `Node::Grid`,
  smooth `Bool` union with `MeshSdf` nodes, `to_mesh`. Noted under S08 Interface changes.
- **S01:** same requests as S13 (`exts_of_kind`, `Evaluated::body_key`, `BlobSource` and pending
  blobs with `SetExt`, `ExtRegistration`), plus the S12 request "add a feature together with its
  baked payload in one undo entry" (DOC-08 baked bodies). No new item.
- **S16:** none new. GEN-06 tests call `amg_am::overhang::classify_faces` (AM-02) from
  `amg-features` tests; `OutcomeBodyFeature::build` lets AM-01 pre-orient a print setup later
  (optional). Noted under S16 Interface changes.
- **S17:** `Material` yield/density through `SolidMaterial::from_material` (S13). None new.
- **S10/S11:** overlay meshes and the ghost layer (RND-07, viewport-rendering note §ghost), the
  generic selection filter (UI-04), workspace registration (UI-02). None new.
- **S18 (SCR-12):** the Python study API should also wrap `GenStudy`, `OutcomeQueue`,
  `CreateOutcomeBody` (request, not edited; see the refinement log).

## Work packages
WPs are ≈ 1–3 days each. GEN-01..12 keep their IDs and meaning (GEN-01, GEN-05, GEN-07 and
GEN-09 re-scoped; see the refinement log). GEN-13..19 are new. Times are on the reference
machine (RTX 5060 Ti, 8+ cores), release build. "Golden" = `tests/golden/topopt/` with the
targets in note §11.

| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| GEN-01 | Generative study data model: `GenStudy` record (`gen.study`, v1), `to_record`/`from_record`, `command`, `new_gen_study_command`, `gen_studies`, `inputs_hash`, `outcome_state` (out-of-date), `pending`, `precheck`; `ExtRegistration` + migration stub (workspace UI moved to GEN-13) | FEM-01, FEM-13, DOC-05, DOC-07, MAT-06 | P1 | Todo | 1. A study with 2 preserves, 1 obstacle, 1 symmetry plane, 2 load cases, 2 materials and 2 methods round-trips `to_record`/`from_record` and a `.amg` save→load→save with byte-identical `design.json`. 2. `SetExt` edits undo/redo to equal records. 3. Editing a preserve body's upstream feature, a material or a BC flips existing outcomes to `OutOfDate`; renaming the study does not. 4. `precheck` table test: no preserve, a load on a non-preserve face, a material without yield with `MinMass`, a tilted symmetry plane, overhang 70°, a memory estimate above the budget, each a named diagnostic. 5. A v2 record gives `TooNew`, a garbled one `Decode` (no panic). |
| GEN-02 | Design-space voxelisation: `GridFrame`, `build_grid` (bounds default and padding, resolution, symmetry alignment, MG-friendly dims, additive aspect), cell classification from `MeshSdf` with offsets, thin-preserve and overlap diagnostics | IMP-03, MESH-03, BREP-08 | P1 | Todo | 1. Box preserve 20 mm and sphere obstacle r = 10 mm with offset 2 at h = 1: every cell state equals the analytic rule of note §4.3 (`d ≤ offset + 0.5 h` for preserves, `d ≤ 12` for the sphere), evaluated in f64 at the cell centres. 2. Symmetry plane x = 0 lies on a node plane and the cell states are mirror-identical. 3. A 0.5 mm plate preserve at h = 1 still gets one cell layer and a thin-preserve warning. 4. Overlapping preserve/obstacle → `PreserveInObstacle` naming both. 5. With `build = (+Z, 55°)`, `hz / hx = tan 55°` within 1e-12. 6. Bench `topopt/voxelize-1m`: 1 M-triangle preserve set into 1 M voxels < 5 s. |
| GEN-14 | Voxel FE CPU reference: `h8_stiffness` (any box aspect), matrix-free node-wise operator (f64, rayon), projector constraints, MG hierarchy (re-discretised coarse levels, trilinear transfers, Chebyshev–Jacobi, CMP-03 coarse Cholesky), `VoxelFe` with `Backend::Cpu`, compliance, element energy, centroid von Mises, adjoint solve | CMP-03 | P1 | Todo | 1. `Ke` is symmetric, PSD with exactly 6 zero eigenvalues (1e-10), and equals a 3×3×3-Gauss integration within 1e-12. 2. Patch test (uniform strain on a 4×4×4 block) gives exact stress (1e-10). 3. Cantilever 200×20×20 voxels, tip load: tip deflection within 3 % of Timoshenko. 4. MG-PCG iteration count to 1e-8 grows by < 50 % from 32³ to 128³ (mesh independence). 5. Pin/frictionless/symmetry projectors: a half model with symmetry gives the full model's compliance within 1e-6 relative. 6. Same inputs give bit-identical results across 1/4/16 threads. |
| GEN-03 | GPU voxel FE: WGSL matrix-free H8 operator (f32, node-wise, no atomics), GPU MG V-cycle (smoother, transfers, coarse level via CPU Cholesky), CMP-05 PCG driver, warm start, `Backend::Gpu`, memory estimate, device-loss handling | GEN-14, CMP-05, CMP-01 | P1 | Todo | 1. On 64³ random-density fields (x̄ ∈ [0.001, 1]) GPU displacements match the CPU f64 reference within 1e-4 in the relative energy norm. 2. Bench `budget/topopt-solve-1m`: 1 M voxels (100³ cantilever, SIMP field) from zero to rel. residual 1e-4 < 2 s; iteration counts recorded in note §5.3. 3. 8 M voxels: measured device memory ≤ 1.5 GB and one solve < 20 s. 4. Two runs are bit-identical. 5. A cancelled solve returns `Cancelled` within 1 s; a simulated device loss returns `TopoptError::Gpu`, not a hang. |
| GEN-15 | Voxel BCs from S13 load cases: `bc::voxel_case` (constraints → node projectors on preserve cells; force, pressure, bearing, moment, remote force as area-weighted nodal loads; gravity as design-dependent load), `build_problem` glue in `amg-features::sim::generative` (bodies → `SpaceBody`, `resolve_load_case`, `target_triangles`) | GEN-02, GEN-14, FEM-13, FEM-03 | P1 | Todo | 1. For each load variant on the box, cylinder-with-hole and L-bracket fixtures, the nodal resultant force and moment equal the requested ones within 1e-9 relative. 2. All constrained and loaded nodes belong to `Preserve` cells; a target on a non-preserve face gives a diagnostic naming the BC. 3. Pin on a hole: the shaft-like preserve can rotate about the hole axis (zero reaction torque on the CPU solve). 4. A GEN-14 cantilever built through `build_problem` from a `.amg` fixture gives the programmatic model's compliance within 1 %. |
| GEN-04 | SIMP core: variable map with symmetry, density filter (cone, PDE beyond 4 h), Heaviside projection with β continuation, OC, own MMA (`amg_topopt::mma`), `opt::optimize` for max stiffness with a mass target (one load case), iteration history, previews, termination/status | GEN-03, GEN-14 | P1 | Todo | 1. MMA reproduces Svanberg's documented toy problems to 1e-4 of the published optimum. 2. Golden: 3D cantilever 60×20×4 (vf 0.3, p 3, r 1.5) compliance within 2 % and volume within 0.1 % of the committed GNU Octave `top3d.m` reference with matching settings; 3D MBB half beam 96×32×16 with symmetry within 2 % of `top3D125.m`. 3. OC and MMA agree within 2 % on case 2's cantilever. 4. With projection, final grey measure < 5 %. 5. Bench `budget/topopt-1m-100it`: 1 M voxels, 100 iterations < 5 min (vision). 6. Symmetric study: mirrored densities bit-identical; two runs bit-identical. 7. Filter and projection sensitivities pass a finite-difference check (1e-5 relative, CPU f64). |
| GEN-05 | Multiple load cases and min mass: weighted compliance over cases, min mass s.t. per-case displacement p-norm limit (MMA, one adjoint per case), gravity sensitivities, mass targets (absolute and fraction) | GEN-04, GEN-15 | P1 | Todo | 1. Two-case cantilever (tip down, tip sideways): the combined design's worst-case compliance ≤ the worst case of each single-case design evaluated on both cases. 2. Min mass s.t. tip displacement ≤ δ: final displacement within 2 % of δ (active) and below it. 3. Gravity-only and gravity + tip load sensitivities pass finite differences (1e-4 relative) on 20 random elements. 4. `MassTarget::Absolute` hits the mass within 0.5 %. |
| GEN-16 | Stress-constrained min mass: `qp`-relaxed centroid von Mises, per-case p-norm with adaptive normalisation, `σ_y / SF`, adjoint sensitivities, preserve cells included; reported safety factor | GEN-05 | P1 | Todo | 1. Stress p-norm sensitivities pass finite differences (1e-4 relative). 2. L-bracket (Le et al. 2010 geometry, extruded), SF 2: max voxel von Mises ≤ 1.05 σ_allow, and below the max of the compliance design with the same mass. 3. The reported `min_safety_factor` equals σ_y / max σ_vM of the final design within 1e-9. 4. 1 M-voxel stress study, 100 iterations < 10 min (recorded for QUESTIONS #93). |
| GEN-06 | Additive overhang filter: `am_filter` and adjoint (plus stencil, smooth min/max, layer sweep on GPU and CPU), build axis ±X/±Y/±Z, angle via voxel aspect (30°–60°), minimum thickness → filter radius, checks against AM-02 | GEN-04, GEN-07, AM-02 | P1 | Todo | 1. Filter sensitivities pass finite differences (1e-5, CPU f64); GPU matches CPU within 1e-6. 2. Cantilever, build +Z, α = 45° and α = 55°: on the GEN-07 mesh, `classify_faces` (AM-02, same α and direction) finds overhang area outside preserve faces and the contact layer ≤ 2 % of the downward area; the same study unrestricted finds ≥ 10 %. 3. Build −X gives the mirrored-direction result of +X on a symmetric case. 4. Bench `topopt/am-filter-1m`: forward + adjoint < 0.1 s. 5. Owner check (manual, recorded): the 45° outcome exported as 3MF slices in OrcaSlicer at a 45° threshold with no support generated outside preserves. |
| GEN-07 | Outcome → body: `convert::outcome_sdf` (cubic resample, redistancing, smoothing, `SampledGrid::from_dense`, smooth union with exact preserves, minus offset obstacles), `outcome_mesh` (IMP-04), `gen.outcome_body` feature with baked payload, `CreateOutcomeBody` job (mesh or implicit body) | GEN-04, GEN-02, GEN-01, IMP-04, IMP-12, IMP-03, DOC-08 | P1 | Todo | 1. Cantilever outcome → mesh: closed, oriented, 0 non-manifold edges, 0 self-intersections (MESH-02); volume within 3 % of Σ x̄ V. 2. Preserve faces farther than the blend radius from a junction are reproduced within 0.05 mm (one-sided Hausdorff). 3. No output vertex lies inside an obstacle's offset by more than half a conversion cell. 4. Bench `topopt/convert-1m`: 1 M-voxel outcome → mesh < 20 s. 5. `gen.outcome_body` round-trips save/load and undo as one entry; re-converting with other smoothing replaces the payload. 6. Golden mesh stats (volume, area, bbox) in `tests/golden/topopt/`. |
| GEN-17 | Validation FEA: patch assignment of the converted body to load-case targets, re-targeted `LoadCase`, TET10 mesh (FEM-02/FEM-14), `solve_static` (FEM-04), `ValidationReport` in the outcome header, `ValidateOutcome` job | GEN-07, GEN-15, FEM-04, FEM-14 | P1 | Todo | 1. Converted cantilever: the tet mesher succeeds and the validation max displacement is within 10 % of the voxel prediction. 2. Reaction resultant equals the applied load within 1 %. 3. Every constrained or loaded target maps to ≥ 1 patch; a target that maps to none gives a diagnostic, not a panic. 4. The report is stored with the outcome and survives save/load. 5. Cancel in each phase returns within 1 s and leaves the record unchanged. |
| GEN-18 | Run job and storage: `RunOutcome` (voxelise → map BCs → optimise → store), `store::{write, read}` (`AMGGENO1`), progress phases, cancel, live previews, `load_outcome`, out-of-date on arrival | GEN-01, GEN-15, GEN-04 | P1 | Todo | 1. A cantilever study generates from a `Document` as a job; progress reaches 1.0 through 4 named phases; cancel in each phase returns `Cancelled` within 1 s and leaves the record unchanged. 2. `store` round-trip is bit-identical; a 1 M-voxel outcome blob ≤ 6 MB. 3. Save→load keeps outcomes; `load_outcome` equals the in-memory outcome. 4. An edit during the run marks the arriving outcome `OutOfDate`. 5. Previews arrive every `preview_every` iterations; undo snapshots hold no blob bytes. |
| GEN-13 | Generative workspace UI: workspace + toolbar, study browser, design-space tools (preserve, obstacle with offset, starting shape, symmetry, bounds) with colour overlays, BC dialogs reused from S13 with a preserve-only filter, objectives/limits, manufacturing, materials and settings dialogs, precheck panel, Generate/Cancel, live preview, outcome list with Create body / Validate | GEN-01, GEN-18, UI-02, UI-04, UI-05, RND-07, FEM-05 | P1 | Todo | 1. Kittest: create a study, mark a preserve and an obstacle, fix one preserve face, load another, pick a material → the record has exactly those items (one undo entry each). 2. Generate stays disabled while precheck has errors and enables when they are fixed. 3. Overlays: preserve green, obstacle red with its offset shell, follow upstream edits (snapshot). 4. During a 1 M-voxel run no frame exceeds 16 ms (trace) and the preview surface updates. 5. Dialogs accept expressions with units (`2 mm`, `45 deg`, `1.5 kg`). |
| GEN-08 | Outcome batches and explorer: `OutcomeQueue` over materials × methods ("all directions" = six outcomes), per-outcome cancel/retry, snapshots every 10 iterations (x̄ quantised to u8) and iteration slider, explorer panel (thumbnail grid, properties table, scatter plot of any two metrics, filters, side-by-side compare of 2–4) | GEN-13, GEN-17, GEN-18 | P2 | Todo | 1. 3 materials × 2 methods = 6 outcomes finish unattended; cancelling one leaves the other 5. 2. Scatter plot points equal the stored metrics (kittest readback). 3. Filters on mass and safety factor show exactly the matching outcomes. 4. The iteration slider shows stored snapshots and the blob stays ≤ 3× the no-snapshot size. 5. Explorer state is UI-only (no undo entries). |
| GEN-09 | Min/max member size: robust formulation (eroded, intermediate, dilated projections; worst case over the three; η ± Δη from the target size), max member size by a local volume constraint (p-norm) | GEN-05 | P2 | Todo | 1. Cantilever with min size 3 mm at h = 0.5: the measured minimum wall thickness (MESH-11 on the GEN-07 mesh) ≥ 3 mm − 1 voxel. 2. Max size 4 mm: no member thicker than 4 mm + 1 voxel (distance-transform measure). 3. Robust compliance within 10 % of the non-robust design at the same mass. 4. Iteration cost ≤ 3.5× GEN-04's (three solves). |
| GEN-19 | Frequency limit: first eigenfrequency ≥ limit, LOBPCG on the voxel operator with the MG preconditioner, Pedersen low-density mass modification, eigen sensitivities (simple and repeated-mode guard) | GEN-05, CMP-04 | P2 | Todo | 1. LOBPCG on a voxel cantilever matches CMP-04 shift-invert Lanczos on the assembled CPU matrix within 1e-4 (first 3 modes). 2. Design with f₁ ≥ limit: final f₁ ≥ 0.98 × limit and no localised mode (≥ 90 % of modal strain energy in cells with x̄ > 0.5). 3. GEN-17-style S13 modal validation of the converted body within 10 % of the voxel f₁. |
| GEN-10 | Shape optimisation of an existing body: `GenKind::ShapeOptimization`, design space = the body (remove only), keep regions as preserves, mass-target slider, keep/remove ghost overlay (S10 ghost layer), convert like an outcome; `sim.shape_optimization` entry in the Simulation workspace | GEN-04, GEN-15, GEN-18, GEN-07, FEM-05 | P2 | Todo | 1. Bracket fixture, 60 % mass target: final mass within 1 % of the target and no cell outside the original body is solid. 2. Keep regions are never removed. 3. The overlay shows removed regions as ghosts and updates within 100 ms when the slider moves (iso-level change, no re-solve). 4. The result converts to a watertight body (GEN-07 checks). |
| GEN-11 | Level-set method alternative (P3) | GEN-04 | P3 | Todo | MBB and cantilever compliance within 3 % of the SIMP baseline at equal volume, with a crisp boundary (no grey cells) and the same budget class. |
| GEN-12 | Milling / 2-axis / casting constraints (P3) | GEN-04 | P3 | Todo | 2-axis cutting gives a prismatic design (every column along the cut axis uniform within 1e-6); 3-axis milling and die casting designs have no undercut along their tool/draw directions (ray check on the converted mesh). |

## Testing
- **Golden** (`tests/golden/topopt/`, note §11): H8 patch test and cantilever; 3D cantilever and
  MBB against committed GNU Octave runs of `top3d.m` / `top3D125.m` with our settings; two-case
  cantilever; L-bracket stress design; additive cantilever at 45° and 55° checked with AM-02;
  conversion stats; validation FEA. Reference scripts and values in `testdata/topopt/ref/`.
- **Differential:** GPU vs CPU f64 (solver, filter, AM filter, whole optimisation on small grids);
  MMA vs OC; voxel metrics vs S13 TET10 validation.
- **Sensitivities:** finite-difference checks for filter, projection, AM filter, compliance,
  gravity, stress and displacement p-norms (CPU f64).
- **Determinism:** bit-identical outcomes across two runs and across 1/4/16 CPU threads.
- **UI:** kittest flows (GEN-13, GEN-08), frame-time trace during a run.

## Performance targets
| Scenario | Target | WP |
|---|---|---|
| One static solve, 1 M voxels, to 1e-4 | < 2 s | GEN-03 |
| Topology optimization, 1 M voxels, 100 iterations (vision, min compliance) | < 5 min | GEN-04 |
| Stress-constrained, 1 M voxels, 100 iterations | < 10 min (QUESTIONS #93) | GEN-16 |
| AM filter forward + adjoint, 1 M voxels | < 0.1 s | GEN-06 |
| Voxelisation, 1 M-triangle preserves → 1 M voxels | < 5 s | GEN-02 |
| Outcome conversion, 1 M voxels → mesh | < 20 s | GEN-07 |
| Device memory at 8 M voxels | ≤ 1.5 GB | GEN-03 |
| UI during a run | no frame > 16 ms (vision) | GEN-13 |

## Open questions
1. ~~(from S16, 2026-10-20) GEN-06 should use the S16 overhang convention (`docs/notes/am-build-prep.md` §1.1: slope from horizontal, strict `−n·d > cos α`), so "prints without supports at α" is checkable with AM-02.~~ **Resolved 2026-11-08:** adopted (note §1, §7.4); GEN-06's done-when checks with `classify_faces`.
2. ~~(from S08, 2026-11-08) GEN-07 hands the density field to S08 as `SampledGrid::from_dense` (IMP-12), offsets it to the 0.5 iso level and meshes with `amg_implicit::to_mesh` (IMP-04). GEN-02's voxelisation can use `MeshSdf` (IMP-03).~~ **Resolved 2026-11-08:** adopted with two refinements: the density is resampled to a cubic grid and redistanced to a true signed distance before `from_dense` (S08's bounds assume distances), and exact preserve SDFs are smooth-unioned in (note §8). GEN-02 uses `MeshSdf`.
3. Default outcome body kind: mesh or implicit? (QUESTIONS #90.)
4. Arbitrary build directions and tilted symmetry planes (rotated grid frame)? (QUESTIONS #91.)
5. Default resolution: 1 M voxels or coarser for speed? (QUESTIONS #92.)
6. Budget scope for stress-constrained and robust studies. (QUESTIONS #93.)
7. Validation FEA: automatic after every outcome, or on demand? (QUESTIONS #94.)

8. (from S15, 2026-11-08) GEN-03 implements `amg_compute::mg::GridLevelOperator` for H8 elasticity (apply, diagonal, fixed mask, density-coarsened `coarsen`, coarsest `assemble`) and uses `Gmg` + `pcg_gpu` (CMP-05, which now builds on the new CMP-08). CMP-05's bar is a 17 M-DOF scalar Q1 MGPCG solve < 2 s, as headroom for GEN-03's 1 M voxels < 2 s. Density fields can use `amg_compute::grid::DenseGrid`; `testprob::simp_field` gives SIMP-like test coefficients.

## Refinement log
### 2026-11-08 - S14-refine (lane F refinement)
- **Read:** AGENTS.md, vision (topopt budget 1 M voxels / 100 iterations < 5 min), architecture
  (`amg-topopt` deps: fem, implicit, mesh), ADRs (ADR-004 wgpu + CUDA later behind a trait,
  ADR-006, ADR-012, ADR-014), roadmap, QUESTIONS, S13 (Interfaces, FEM-01/13/17 patterns,
  2026-11-08 log), simulation-fem note (§9.5 field export, §9.6 out-of-date), S08 (Interfaces,
  `MeshSdf`, `SampledGrid::from_dense`, `to_mesh`, 2026-11-08 log), implicit-modeling note, S15
  (Draft, open questions), S16 (overhang convention, AM-02 API, AM-09 voxel FE), S01 (§11 extension
  records, baked bodies, S12/S13 requests), S11 (workspace, `Tool`, providers), S17, fusion parity.
  Code: `amg-topopt` is an empty crate with deps core, jobs, compute, fem, implicit, mesh;
  `amg-compute` and `amg-fem` are empty; `amg-features` has `mesh` and `sketch` modules with the
  `register_feature!`/`link()` pattern and depends on `amg-topopt`; `amg-doc` has `ExtRecord`,
  `ExtKey`, `Command::{AddExt, SetExt}`, `BlobRef`, `BakedBody`, `BodyGeom::{Mesh, Implicit}`;
  `amg-jobs` has `Job`, `JobCtx::{checkpoint, progress, sub}`, `Gpu`.
- **Research:**
  - Fusion behaviour: additive constraint with build direction ±X/±Y/±Z or all directions,
    overhang angle (45° default) and minimum thickness
    ([Autodesk help: additive manufacturing parameters](https://help.autodesk.com/cloudhelp/ENU/Fusion-GenerativeDesign/files/GD-ADDITIVE-REF.htm),
    [Fusion blog: better AM outcomes](https://www.autodesk.com/products/fusion-360/blog/unlocking-better-additive-manufacturing-outcomes-generative-design/));
    resolution slider of 100 steps with default 40, obstacle offsets, rounded obstacles
    ([AU: Advanced Generative Design Manufacturing 2021](https://www.autodesk.com/autodesk-university/article/Advanced-Generative-Design-Manufacturing-2021),
    [Autodesk help: study settings](https://help.autodesk.com/cloudhelp/ENU/Fusion-GenerativeDesign/files/GD-STUDY-SETTINGS.htm));
    preserves thinner than a voxel make the space unsolvable (Autodesk forum); loads only on
    preserves; safety factor 2 in tutorials (no documented default found, so 2.0 is our choice).
  - Solvers: matrix-free GMG on the GPU (Wu, Dick, Westermann, IEEE TVCG 2016,
    [preprint](https://www.cs.cit.tum.de/fileadmin/w00cfj/cg/Research/Publications/2015/TopOpt/preprint.pdf));
    MG-preconditioned CG with loose tolerances (Amir, Aage, Lazarov 2014); giga-voxel scale
    (Aage et al., Nature 2017); narrow-band sparse grids (Liu et al. 2018) noted, not needed.
  - Method: SIMP, density filter, Heaviside/robust projection (Wang, Lazarov, Sigmund 2011),
    MMA (Svanberg 1987/2007), stress p-norm with `qp` relaxation and adaptive normalisation (Le et
    al. 2010). Reference codes `top88`, `top3d`, `top3D125`
    ([DTU topopt software](https://www.topopt.mek.dtu.dk/apps-and-software/efficient-topology-optimization-in-matlab)).
    Published compliance values depend on `E_min`, filter and iteration count, so the golden
    references are generated by running those codes in GNU Octave with our settings (note §11),
    not copied from papers.
  - AM filter: Langelaar 2016/2017 works on structured grids and tunes the angle only through
    element aspect; front propagation (van de Ven et al. 2018, 2020) allows arbitrary angles and
    unstructured meshes
    ([TU Delft record](https://repository.tudelft.nl/record/uuid:4af37279-0711-4cb4-b53a-5340b6ec37a4),
    [CMAME 2020](https://research.tudelft.nl/en/publications/overhang-control-based-on-front-propagation-in-3d-topology-optimi)).
    Chosen: Langelaar with aspect, 30°–60°; front propagation is the upgrade path.
  - Rust: no maintained MMA or topology-optimisation crate; MMA is small, so it is our own.
- **Decided:**
  - `amg-topopt` is document-agnostic; the document side is `amg-features::sim::generative`
    (lane F's `sim` module), so no lane or crate change and no roadmap ownership edit.
  - Regular axis-aligned voxel grid; build directions ±X/±Y/±Z like Fusion; symmetry planes
    axis-aligned (rotated frames: QUESTIONS #91).
  - S14 owns the H8 operator and the MG hierarchy; S15 provides the PCG driver, kernels and the
    coarse Cholesky. A CPU f64 reference backend is part of the plan (tests, no-GPU fallback).
  - f32 on the GPU with `E_min = 1e-6 E_0`, loose inner tolerance with warm start.
  - Constraints as per-node projectors; prescribed non-zero displacement is refused.
  - OC for single-constraint max stiffness, own MMA for the rest.
  - Outcome → baked `gen.outcome_body` feature (not live-linked to the study; rerunning an
    optimisation is not a recompute). Validation FEA with S13 on the converted body.
  - One outcome at a time on the GPU, each its own job; record + `AMGGENO1` blob like S13.
- **WPs:** GEN-01..12 keep their IDs and meaning. Re-scoped: GEN-01 → study data model only (the
  workspace UI moves to new GEN-13; dep UI-05 moves with it, FEM-13/DOC-05/DOC-07/MAT-06 added);
  GEN-05 → multiple load cases, min mass with displacement, gravity (stress moves to new GEN-16);
  GEN-07 → conversion and the outcome body feature (validation FEA moves to new GEN-17, so the
  FEM-04 dep moves there; GEN-07 gains IMP-12, IMP-03, GEN-01, GEN-02, DOC-08); GEN-09 → min/max
  member size only (frequency moves to new GEN-19). GEN-02 deps: IMP-03 + MESH-03, BREP-08.
  GEN-03 now builds on the new CPU reference GEN-14 and adds CMP-01. GEN-06 gains GEN-07 and AM-02
  for its check. GEN-08 gains GEN-13, GEN-17, GEN-18. GEN-10 gains GEN-15, GEN-18, GEN-07, FEM-05.
  New: GEN-13 (workspace UI), GEN-14 (CPU reference FE + MG), GEN-15 (voxel BCs from load cases),
  GEN-16 (stress), GEN-17 (validation FEA), GEN-18 (run job, storage), GEN-19 (frequency).
  External references checked: S13 log names GEN-01 (FEM-01), GEN-05 (FEM-13), GEN-07 (FEM-04);
  GEN-01 and GEN-05 keep those deps; GEN-07's FEM-04 dep moved to GEN-17 (validation), which
  GEN-07 originally covered. S08 names GEN-07 as composing IMP-04 and IMP-12 (kept). S15 CMP-02
  names GEN-02 as a user; GEN-02 does not need CMP-02 (note under S15 Interface changes). S16
  names GEN-06 for the shared α convention (kept).
- **Questions:** #90 (outcome body kind), #91 (arbitrary build directions / tilted symmetry), #92
  (default resolution), #93 (budget scope for stress and robust studies), #94 (validation FEA
  automatic or on demand). Resolved here: OQ 1 and 2.
- **Roadmap:** Phase 4 F gains GEN-13..18; Phase 5 F gains GEN-19. Suggestion (not applied): move
  GEN-14 and GEN-03 to Phase 3 F to de-risk roadmap risk 4 (GPU solver performance) earlier,
  if lane F has room after CMP-05.
- **Leftovers:** S18 SCR-12 could wrap `GenStudy`/`OutcomeQueue` (request only, S18 not edited).
  The S10 thumbnail path for the explorer (GEN-08) is assumed from the offscreen render used for
  `thumbnail.png`; confirm when GEN-08 starts.

## Interface changes
None yet (spec not implemented). Requests this spec makes of other specs are listed under
Interfaces → "Needed from other specs" and as short notes in S08, S13, S15 and S16.
