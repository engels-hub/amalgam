# S15 — GPU compute & numerics library

| | |
|---|---|
| **Status** | Refined |
| **Priority** | P1 |
| **Crates** | `amg-compute` (foundation layer) |
| **Lane** | F |
| **Depends on** | S00 (FND-03 core types, FND-04 `JobCtx`, FND-09 `Gpu`, FND-08 bench harness) |
| **Parity** | — (enabler for S08, S13, S14, S16) |

## Summary
Shared numeric building blocks so FEA, topology optimization, implicit modeling and AM
analyses don't each reinvent them: GPU buffer/kernel helpers, reductions/scans, sparse
matrices, iterative and direct solvers, eigen solvers, voxel/brick grids.

The design note [`docs/notes/compute-numerics.md`](../notes/compute-numerics.md) has the GPU
constraints, the precision and determinism policy, the grid layout and the solver choices, with
sources.

## Scope
**In:**
- A thin layer over `amg_jobs::gpu`: typed GPU vectors, a dispatch context, a WGSL prelude,
  timestamps (CMP-07).
- GPU primitives with CPU references: reduce, dot, scan, compaction, segmented reduce, stable
  radix sort, atomic counters (CMP-01).
- Dense voxel grids and sparse brick grids, CPU and GPU, with generated WGSL accessors (CMP-02).
- Symmetric sparse matrices (upper CSR), CPU SpMV, faer Cholesky / LDLᵀ / LBLᵀ with symbolic
  reuse, memory estimate, pivot report and inertia (CMP-03); nested-dissection ordering (CMP-10).
- Shift-invert Lanczos for vibration and buckling pencils (CMP-04); LOBPCG (CMP-09).
- Operator and preconditioner traits, CPU and GPU PCG, Jacobi / block-Jacobi / Chebyshev,
  mixed-precision refinement (CMP-08); geometric multigrid on structured grids (CMP-05);
  smoothed-aggregation AMG (CMP-11); optional CUDA backend (CMP-06).
- Structured test problems for benchmarks (`testprob`).

**Out:**
- Element formulations, assembly, constraints (S13 `amg-fem`; S14 owns the H8 voxel operator).
- SDF evaluation and its WGSL interpreter (S08); the brick grid is only its storage.
- Rendering kernels (S10).
- Non-symmetric sparse LU (no P1 consumer; faer has it, add when needed — note §5).
- f64 or emulated-double GPU kernels (note §2.1).

## Design
- **GPU constraints** (note §1): DX12 with FXC by default, so no subgroups, no `f16` shader type
  and no f64. Kernels use shared memory and barriers only. f16 data is packed two per `u32`
  (`pack2x16float`/`unpack2x16float`). Bindings stay under the device's storage-binding limit;
  1-D launches fold into 2-D grids past 65 535 workgroups.
- **Precision** (note §2.1, resolves open question 1): GPU kernels are f32. Accuracy comes from
  mixed-precision iterative refinement with f64 residuals on the CPU. Every GPU routine has an
  f64 (or exact integer) CPU reference with the same semantics.
- **Determinism** (note §2.2): CPU results are bit-identical across runs and thread counts; GPU
  results are bit-identical across runs on one device. No float atomics; deterministic allocation
  by mark → scan → scatter.
- **Primitives** (note §3): reduce-then-scan (portable; no decoupled look-back), LSD radix sort
  with 8-bit digits, fixed-tree reductions.
- **Grids** (note §4): `BrickGrid<T>` is a fixed two-level tree (dense root over 64³-cell tiles,
  512-slot tile nodes, brick pool), bricks of 8³ cells with an optional apron, two fill codes for
  empty space. GPU mirror with generated WGSL accessors. `DenseGrid<T>` for voxel fields.
- **Direct** (note §5): faer `=0.23.2` (the workspace pin). Upper-triangle CSR is passed to faer
  as lower CSC without a copy. Symbolic analysis is reusable across numeric factorisations
  (shifts, Newton steps).
- **Eigen** (note §6): thick-restart shift-invert Lanczos, vibration and buckling transforms, the
  factorisation as a callback, Sturm-count check for band searches.
- **Iterative** (note §7): PCG generic over a crate-private `VectorBackend` (ADR-004 amendment),
  full-storage f32 `GpuCsr`, Chebyshev–Jacobi smoothers shared with S13's p-multigrid, matrix-free
  geometric multigrid for voxel grids.

## Interfaces
Signatures other crates rely on (`amg-implicit`, `amg-fem`, `amg-topopt`, `amg-am`). Types from
`amg-core`: `Cancelled`, `Diagnostic`, `Severity`, `Point3`. From `amg-jobs`: `JobCtx`, and from
`amg_jobs::gpu`: `Gpu`, `GpuError`, `Readback`, `ComputePipelineDesc`, `wgpu` (re-exported).
New dependencies: `half` (MIT/Apache-2.0, feature `bytemuck`), `faer =0.23.2`
(`std`, `sparse-linalg`), `rayon`, `bytemuck`, `blake3`; `cudarc` behind `cuda`.
Cargo features: `gpu` (default, needs `amg-jobs/gpu`), `cuda` (CMP-06), `metis` (CMP-10, only
if chosen there).

```rust
// ───────────── amg_compute (crate root) ─────────────
pub use half::f16;

#[derive(Debug, thiserror::Error)]
pub enum ComputeError {
    #[error(transparent)] Gpu(#[from] amg_jobs::gpu::GpuError),
    #[error(transparent)] Cancelled(#[from] amg_core::Cancelled),
    #[error("buffer `{label}` needs {bytes} bytes; the device binding limit is {limit}")]
    TooLarge { label: String, bytes: u64, limit: u64 },
    #[error("invalid matrix: {0}")] InvalidMatrix(String),
    #[error("matrix is not positive definite: pivot {pivot:e} at row {row}")]
    NotPositiveDefinite { row: usize, pivot: f64 },
    #[error("singular matrix at row {row}")] Singular { row: usize },
    #[error("not converged after {iterations} iterations (relative residual {residual:e})")]
    NotConverged { iterations: u32, residual: f64 },
    #[error("invalid shape: {0}")] Shape(String),
}
impl amg_core::Diagnostic for ComputeError { /* user_message, hint, severity, is_cancelled */ }
pub type ComputeResult<T> = Result<T, ComputeError>;

/// Where a solver runs. `Cuda` exists only with feature `cuda` (CMP-06).
#[derive(Clone, Debug)]
pub enum SolverDevice {
    Cpu,
    Wgpu(std::sync::Arc<amg_jobs::gpu::Gpu>),
    #[cfg(feature = "cuda")] Cuda(std::sync::Arc<cuda::CudaDevice>),
}

// ───────────── amg_compute::gpu (CMP-07) ─────────────
/// Typed storage buffer. Usage always includes STORAGE | COPY_SRC | COPY_DST.
pub struct GpuVec<T: bytemuck::Pod> { /* wgpu::Buffer, len, PhantomData<T> */ }
impl<T: bytemuck::Pod> GpuVec<T> {
    /// Errors with `TooLarge` when `len * size_of::<T>()` exceeds the binding limit.
    pub fn zeroed(gpu: &Gpu, label: &str, len: u64) -> ComputeResult<Self>;
    pub fn from_slice(gpu: &Gpu, label: &str, data: &[T]) -> ComputeResult<Self>;
    pub fn len(&self) -> u64;
    pub fn is_empty(&self) -> bool;
    pub fn buffer(&self) -> &wgpu::Buffer;
    pub fn binding(&self) -> wgpu::BindingResource<'_>;
    pub fn write(&self, gpu: &Gpu, offset: u64, data: &[T]);          // queue.write_buffer
    pub fn read(&self, gpu: &Gpu) -> Readback<T>;                      // whole vector
    pub fn read_range(&self, gpu: &Gpu, range: std::ops::Range<u64>) -> Readback<T>;
}

/// A compiled kernel: pipeline + workgroup size (from the `WG` override, default 256).
#[derive(Clone)]
pub struct Kernel { /* Arc<wgpu::ComputePipeline>, workgroup: [u32; 3], label */ }
impl Kernel {
    /// Prepends `PRELUDE_WGSL` to `wgsl`; cached through `Gpu::compute_pipeline`.
    pub fn new(gpu: &Gpu, label: &str, wgsl: &str, entry: &str, constants: &[(&str, f64)]) -> ComputeResult<Self>;
    pub fn workgroup(&self) -> [u32; 3];
    /// `entries[i]` is bound at `@binding(i)` of `@group(group)`.
    pub fn bind_group(&self, gpu: &Gpu, group: u32, entries: &[wgpu::BindingResource<'_>]) -> wgpu::BindGroup;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct UniformSlot { pub offset: u64, pub size: u64 }
/// Records dispatches into one encoder; submits in batches. Not Sync; one per job.
pub struct ComputeCtx { /* Arc<Gpu>, Option<wgpu::CommandEncoder>, uniform arena, timestamps */ }
impl ComputeCtx {
    pub fn new(gpu: std::sync::Arc<Gpu>, label: &str) -> Self;
    pub fn gpu(&self) -> &Gpu;
    /// Copies `params` into the uniform arena (256-byte slots); valid until `submit`.
    pub fn uniform<P: bytemuck::Pod>(&mut self, params: &P) -> UniformSlot;
    pub fn uniform_binding(&self, slot: UniformSlot) -> wgpu::BindingResource<'_>;
    pub fn dispatch(&mut self, k: &Kernel, groups: &[&wgpu::BindGroup], workgroups: [u32; 3]);
    /// One thread per element for `n` elements; folds into 2-D past 65 535 workgroups.
    pub fn dispatch_1d(&mut self, k: &Kernel, groups: &[&wgpu::BindGroup], n: u64);
    pub fn copy<T: bytemuck::Pod>(&mut self, src: &GpuVec<T>, dst: &GpuVec<T>);
    pub fn submit(&mut self);
    /// Submits and blocks until the GPU is idle. Job threads only (never the UI thread).
    pub fn finish(&mut self, ctx: &JobCtx) -> ComputeResult<()>;
    /// GPU time per labelled dispatch since the last call; empty without TIMESTAMP_QUERY.
    pub fn take_timings(&mut self) -> Vec<(String, std::time::Duration)>;
}
/// `amg_linear_id(gid, nwg) -> u32`, `WG` override, shared constants.
pub const PRELUDE_WGSL: &str;

// ───────────── amg_compute::prim (CMP-01) ─────────────
#[derive(Clone, Copy, Debug, PartialEq, Eq)] pub enum ReduceOp { Sum, Min, Max }
#[derive(Clone, Copy, Debug, PartialEq, Eq)] pub enum ScanKind { Exclusive, Inclusive }
/// Element types the reductions accept: f32, u32, i32.
pub trait ReduceScalar: bytemuck::Pod + sealed::Sealed {}

/// Cached pipelines for all primitives plus reusable scratch buffers.
pub struct Primitives { /* kernels, scratch pool */ }
impl Primitives {
    pub fn new(gpu: &Gpu) -> ComputeResult<Self>;
    /// Writes the result to `out[out_index]` (stays on the GPU; no readback).
    pub fn reduce<T: ReduceScalar>(&self, cx: &mut ComputeCtx, op: ReduceOp, input: &GpuVec<T>,
                                   out: &GpuVec<T>, out_index: u64) -> ComputeResult<()>;
    pub fn dot(&self, cx: &mut ComputeCtx, a: &GpuVec<f32>, b: &GpuVec<f32>,
               out: &GpuVec<f32>, out_index: u64) -> ComputeResult<()>;
    /// `total` (optional) receives the sum of all inputs (the allocation size).
    pub fn scan(&self, cx: &mut ComputeCtx, kind: ScanKind, input: &GpuVec<u32>, output: &GpuVec<u32>,
                total: Option<(&GpuVec<u32>, u64)>) -> ComputeResult<()>;
    /// Keeps `values[i]` where `flags[i] != 0`, in order; `count[0]` = kept count.
    pub fn compact(&self, cx: &mut ComputeCtx, flags: &GpuVec<u32>, values: &GpuVec<u32>,
                   out: &GpuVec<u32>, count: &GpuVec<u32>) -> ComputeResult<()>;
    /// Segment s covers `values[offsets[s]..offsets[s+1]]`; empty segments give the identity.
    pub fn segmented_reduce(&self, cx: &mut ComputeCtx, op: ReduceOp, values: &GpuVec<f32>,
                            offsets: &GpuVec<u32>, out: &GpuVec<f32>) -> ComputeResult<()>;
    /// Stable LSD radix sort of the low `key_bits` bits (1..=32). `values` move with keys.
    pub fn sort(&self, cx: &mut ComputeCtx, keys: &mut GpuVec<u32>, values: Option<&mut GpuVec<u32>>,
                key_bits: u32) -> ComputeResult<()>;
}
/// One-element `u32` counter for atomic slot allocation in user kernels (S16 A-buffer).
pub struct AtomicCounter { /* GpuVec<u32> */ }
impl AtomicCounter {
    pub fn new(gpu: &Gpu, label: &str) -> ComputeResult<Self>;
    pub fn reset(&self, cx: &mut ComputeCtx);
    pub fn binding(&self) -> wgpu::BindingResource<'_>;
    pub fn read(&self, gpu: &Gpu) -> Readback<u32>;
}
/// CPU references with identical semantics (the f32 sum uses the GPU's tree order).
pub mod cpu {
    pub fn reduce<T: super::ReduceScalar>(op: super::ReduceOp, v: &[T]) -> T;
    pub fn dot(a: &[f32], b: &[f32]) -> f32;
    pub fn scan(kind: super::ScanKind, v: &[u32]) -> (Vec<u32>, u32);
    pub fn compact(flags: &[u32], values: &[u32]) -> Vec<u32>;
    pub fn segmented_reduce(op: super::ReduceOp, values: &[f32], offsets: &[u32]) -> Vec<f32>;
    pub fn sort(keys: &mut [u32], values: Option<&mut [u32]>, key_bits: u32);
}

// ───────────── amg_compute::grid (CMP-02) ─────────────
/// Cells per brick edge (fixed).
pub const BRICK_EDGE: u32 = 8;
/// Bricks per tile edge (fixed): a tile is 64³ cells.
pub const TILE_EDGE: u32 = 8;

/// Sample values a grid can hold: f16, f32, u8.
pub trait GridValue: bytemuck::Pod + Send + Sync + sealed::Sealed {
    fn to_f32(self) -> f32;
    fn from_f32(v: f32) -> Self;
}
/// Brick coordinate (cell coordinate / 8, floor).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct BrickCoord(pub [i32; 3]);
impl BrickCoord {
    pub fn of_cell(ijk: [i32; 3]) -> BrickCoord;
    pub fn morton(self) -> u64;                       // relative to the grid's tile origin
}
/// Samples stored per brick: `(BRICK_EDGE + apron)³`. Apron 1 = 9³ corner samples (S08).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct BrickLayout { pub apron: u32 /* 0 or 1 */ }
impl BrickLayout {
    pub const CELLS: BrickLayout = BrickLayout { apron: 0 };      // 512 cell values
    pub const CORNERS: BrickLayout = BrickLayout { apron: 1 };    // 729 corner samples
    pub fn samples_per_axis(self) -> u32;
    pub fn samples(self) -> usize;
}
/// Constant value of an empty region (S08: Fill0 = outside, Fill1 = inside).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)] pub enum Fill { Fill0, Fill1 }
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)] pub struct BrickId(pub u32);
pub enum Slot<'a, T> { Brick(BrickId, &'a [T]), Fill(Fill) }

/// Sparse grid in index space: dense root over tiles, 512-slot tile nodes, brick pool.
/// World mapping (origin, cell size) belongs to the caller.
#[derive(Clone)]
pub struct BrickGrid<T: GridValue> { /* layout, fill: [T; 2], tile_origin, tile_dims, root, tiles, data */ }
impl<T: GridValue> BrickGrid<T> {
    /// `cell_box` = inclusive-exclusive cell range the grid may cover; errors with `Shape`
    /// when the root would exceed 2²⁴ tiles.
    pub fn new(layout: BrickLayout, fill: [T; 2], cell_box: ([i32; 3], [i32; 3])) -> ComputeResult<Self>;
    pub fn layout(&self) -> BrickLayout;
    pub fn fill_value(&self, f: Fill) -> T;
    pub fn brick_count(&self) -> usize;
    /// `data.len() == layout.samples()`, x fastest. Replaces an existing brick.
    pub fn insert(&mut self, b: BrickCoord, data: &[T]) -> ComputeResult<BrickId>;
    /// Marks an empty slot (or a whole tile with `set_tile_fill`) as Fill0 or Fill1.
    pub fn set_fill(&mut self, b: BrickCoord, f: Fill) -> ComputeResult<()>;
    pub fn set_tile_fill(&mut self, tile: [i32; 3], f: Fill) -> ComputeResult<()>;
    pub fn remove(&mut self, b: BrickCoord, f: Fill);
    pub fn lookup(&self, b: BrickCoord) -> Slot<'_, T>;
    pub fn brick_mut(&mut self, b: BrickCoord) -> Option<&mut [T]>;
    /// Value at a sample point (cell corner with apron 1, cell centre with apron 0).
    pub fn sample(&self, ijk: [i32; 3]) -> T;
    /// Trilinear in sample units (apron 1 only; one brick per lookup).
    pub fn trilinear(&self, p: [f64; 3]) -> f64;
    /// Copies apron samples from the neighbouring bricks (or the fill value).
    pub fn fill_aprons(&mut self);
    /// Reorders bricks by Morton code. Grids built in any order compare equal afterwards.
    pub fn canonicalize(&mut self);
    pub fn bricks(&self) -> impl Iterator<Item = (BrickCoord, &[T])> + '_;
    pub fn map<U: GridValue>(&self, f: impl Fn(T) -> U + Sync, fill: [U; 2]) -> BrickGrid<U>;
    pub fn bytes(&self) -> usize;
    /// blake3 over layout, fill, and the canonical brick list.
    pub fn content_hash(&self) -> [u8; 32];
}

/// Dense grid, x fastest.
#[derive(Clone, Debug, PartialEq)]
pub struct DenseGrid<T> { pub dims: [u32; 3], pub data: Vec<T> }
impl<T: Copy + Default> DenseGrid<T> {
    pub fn new(dims: [u32; 3]) -> Self;
    pub fn index(&self, ijk: [u32; 3]) -> usize;
    pub fn get(&self, ijk: [u32; 3]) -> T;
    pub fn set(&mut self, ijk: [u32; 3], v: T);
}
impl DenseGrid<f32> { pub fn trilinear(&self, p: [f64; 3]) -> f64; }

/// GPU mirror. Bindings: meta (uniform), root, tiles, data (storage, read-only in user kernels).
pub struct GpuBrickGrid { /* buffers, layout, encoding, tile box, brick_count */ }
#[derive(Clone, Copy, Debug, PartialEq, Eq)] pub enum GpuEncoding { F16Packed, F32, U8Packed }
impl GpuBrickGrid {
    pub fn upload<T: GridValue>(gpu: &Gpu, g: &BrickGrid<T>) -> ComputeResult<Self>;
    pub fn download<T: GridValue>(&self, gpu: &Gpu, ctx: &JobCtx) -> ComputeResult<BrickGrid<T>>;
    /// GPU-side build: `mask[i] != 0` marks candidate brick i of `cell_box` in Morton order.
    /// Allocates bricks by scan (deterministic, same order as `canonicalize`); payloads are
    /// zero until the caller's kernel writes them; unmarked slots get `fill_of_unmarked`.
    pub fn allocate(cx: &mut ComputeCtx, prim: &Primitives, layout: BrickLayout, enc: GpuEncoding,
                    cell_box: ([i32; 3], [i32; 3]), mask: &GpuVec<u32>, fill: [f32; 2],
                    fill_of_unmarked: &GpuVec<u32> /* per tile: 0 = Fill0, 1 = Fill1 */) -> ComputeResult<Self>;
    pub fn brick_count(&self) -> u32;                 // known after `allocate`'s readback
    pub fn layout(&self) -> BrickLayout;
    pub fn encoding(&self) -> GpuEncoding;
    pub fn bindings(&self) -> [wgpu::BindingResource<'_>; 4];
    /// Read-write variant of `data` for the producing kernel.
    pub fn data_mut_binding(&self) -> wgpu::BindingResource<'_>;
}
/// WGSL declarations for the 4 bindings at `@group(group) @binding(first..first+3)` plus
/// `bg_slot(brick: vec3<i32>) -> u32`, `bg_sample(ijk: vec3<i32>) -> f32`,
/// `bg_trilinear(p: vec3<f32>) -> f32`, `bg_store(brick_index: u32, local: u32, v: f32)` (`writable`).
pub fn brick_grid_wgsl(group: u32, first_binding: u32, enc: GpuEncoding, writable: bool) -> String;

// ───────────── amg_compute::sparse (CMP-03) ─────────────
/// Symmetric matrix, upper triangle (diagonal included) in CSR, columns sorted per row.
#[derive(Clone, Debug, PartialEq)]
pub struct SymCsr { /* n, row_ptr: Vec<usize>, cols: Vec<u32>, vals: Vec<f64> */ }
impl SymCsr {
    /// Validates: row_ptr monotone, cols sorted, unique, `col >= row`, `< n`.
    pub fn from_raw(n: usize, row_ptr: Vec<usize>, cols: Vec<u32>, vals: Vec<f64>) -> ComputeResult<Self>;
    pub fn n(&self) -> usize;
    pub fn nnz(&self) -> usize;                       // stored (upper) entries
    pub fn row_ptr(&self) -> &[usize];
    pub fn cols(&self) -> &[u32];
    pub fn vals(&self) -> &[f64];
    pub fn vals_mut(&mut self) -> &mut [f64];         // refill values on a fixed pattern
    pub fn diagonal(&self) -> Vec<f64>;
    /// y = A·x (rayon; deterministic).
    pub fn spmv(&self, x: &[f64], y: &mut [f64]);
    /// self + alpha·other on the union pattern (K − σM).
    pub fn add_scaled(&self, alpha: f64, other: &SymCsr) -> ComputeResult<SymCsr>;
    pub fn same_pattern(&self, other: &SymCsr) -> bool;
}
/// Small/test assembly: sums duplicates, ignores (i, j) with j < i after swapping to upper.
#[derive(Default)]
pub struct TripletBuilder { /* n, entries */ }
impl TripletBuilder {
    pub fn new(n: usize) -> Self;
    pub fn push(&mut self, i: usize, j: usize, v: f64);
    pub fn build(self) -> ComputeResult<SymCsr>;
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Ordering { Amd, Identity, Custom(Vec<u32>) /* , NestedDissection { coords } with CMP-10 */ }
#[derive(Clone, Debug)]
pub struct FactorOpts {
    pub ordering: Ordering,                           // Amd
    /// Report the row with the smallest d_ii / a_ii below this (relative) threshold.
    pub pivot_warn: f64,                              // 1e-12
    pub threads: Option<usize>,                       // None = rayon pool
}
/// Exact after symbolic analysis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FactorEstimate { pub factor_nnz: u64, pub bytes: u64, pub flops: f64 }
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PivotReport { pub min_ratio: f64, pub row: usize, pub below_threshold: bool }
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Inertia { pub negative: usize, pub zero: usize, pub positive: usize }

/// Symbolic analysis of a pattern; reuse it for every numeric factorisation on that pattern.
pub struct SymbolicFactor { /* faer symbolic, permutation, n */ }
impl SymbolicFactor {
    pub fn analyze(a: &SymCsr, opts: &FactorOpts) -> ComputeResult<std::sync::Arc<SymbolicFactor>>;
    pub fn estimate(&self) -> FactorEstimate;
    pub fn permutation(&self) -> &[u32];
}
/// `L·Lᵀ`. Errors with `NotPositiveDefinite { row }` (original numbering).
pub struct Cholesky { /* Arc<SymbolicFactor>, faer numeric */ }
impl Cholesky {
    pub fn factor(sym: &std::sync::Arc<SymbolicFactor>, a: &SymCsr, ctx: &JobCtx) -> ComputeResult<Cholesky>;
    pub fn pivots(&self) -> PivotReport;
    pub fn solve_in_place(&self, b: &mut [f64]);
    /// `b` is n × nrhs, column-major.
    pub fn solve_many(&self, b: &mut [f64], nrhs: usize);
}
/// `L·D·Lᵀ` with Bunch–Kaufman pivoting inside supernodes (faer LBLᵀ): indefinite matrices.
pub struct Ldlt { /* Arc<SymbolicFactor>, faer numeric */ }
impl Ldlt {
    pub fn factor(sym: &std::sync::Arc<SymbolicFactor>, a: &SymCsr, ctx: &JobCtx) -> ComputeResult<Ldlt>;
    pub fn inertia(&self) -> Inertia;
    pub fn solve_in_place(&self, b: &mut [f64]);
    pub fn solve_many(&self, b: &mut [f64], nrhs: usize);
}

// ───────────── amg_compute::testprob (CMP-03; used by S13/S14 benches) ─────────────
#[derive(Clone, Copy, Debug)] pub enum Support { Clamped /* x = 0 face */, Free }
pub fn q1_poisson(nodes: u32, coeff: Option<&DenseGrid<f32>> /* per cell */) -> (SymCsr, Vec<f64>);
pub fn q1_elasticity(nodes: u32, support: Support, nu: f64) -> (SymCsr, SymCsr /* mass */, Vec<f64> /* load */);
pub fn simp_field(cells: u32, contrast: f32, seed: u64) -> DenseGrid<f32>;

// ───────────── amg_compute::iter (CMP-08) ─────────────
/// CPU operator in f64.
pub trait LinearOperator: Send + Sync {
    fn dim(&self) -> usize;
    fn apply(&self, x: &[f64], y: &mut [f64]);
}
impl LinearOperator for SymCsr { /* spmv */ }
pub trait CpuPreconditioner: Send + Sync { fn apply(&self, r: &[f64], z: &mut [f64]); }

/// GPU operator in f32. `apply` records work into `cx`; it does not submit.
pub trait GpuOperator: Send + Sync {
    fn dim(&self) -> u64;
    fn apply(&self, cx: &mut ComputeCtx, x: &GpuVec<f32>, y: &GpuVec<f32>) -> ComputeResult<()>;
    /// Diagonal (dim) or 3×3 node blocks (9·dim/3, row-major) for block Jacobi.
    fn diagonal(&self, cx: &mut ComputeCtx, out: &GpuVec<f32>) -> ComputeResult<()>;
    fn block_diagonal(&self, _cx: &mut ComputeCtx, _out: &GpuVec<f32>) -> ComputeResult<bool> { Ok(false) }
}
/// z ≈ A⁻¹ r. S13 plugs its p-multigrid in here; CMP-05 `Gmg` implements it.
pub trait GpuPreconditioner: Send + Sync {
    fn apply(&self, cx: &mut ComputeCtx, r: &GpuVec<f32>, z: &GpuVec<f32>) -> ComputeResult<()>;
}
/// Full (both triangles) storage: f32 values, u32 columns.
pub struct GpuCsr { /* row_ptr: GpuVec<u32>, cols, vals, n, nnz */ }
impl GpuCsr {
    pub fn upload(gpu: &Gpu, a: &SymCsr) -> ComputeResult<GpuCsr>;
    /// Overwrites values on the same pattern (Newton steps, SIMP updates).
    pub fn update_values(&self, gpu: &Gpu, a: &SymCsr) -> ComputeResult<()>;
    pub fn bytes(&self) -> u64;
}
impl GpuOperator for GpuCsr { /* ... */ }
pub struct Jacobi { /* inverse diagonal */ }
pub struct BlockJacobi3 { /* inverted 3×3 blocks */ }
/// Chebyshev polynomial of degree `degree` in D⁻¹A on [λmax/ratio, 1.1·λmax];
/// λmax from `estimate_steps` Lanczos steps. Used as smoother (S13 p-MG, CMP-05) or preconditioner.
pub struct Chebyshev { /* degree, bounds, inv diag */ }
impl Jacobi { pub fn new(cx: &mut ComputeCtx, a: &dyn GpuOperator) -> ComputeResult<Self>; }
impl BlockJacobi3 { pub fn new(cx: &mut ComputeCtx, a: &dyn GpuOperator) -> ComputeResult<Self>; }
#[derive(Clone, Copy, Debug)]
pub struct ChebyshevOpts { pub degree: u32 /* 2 */, pub ratio: f64 /* 30.0 */, pub estimate_steps: u32 /* 10 */ }
impl Chebyshev {
    pub fn new(cx: &mut ComputeCtx, prim: &Primitives, a: std::sync::Arc<dyn GpuOperator>,
               opts: ChebyshevOpts, ctx: &JobCtx) -> ComputeResult<Self>;
    pub fn bounds(&self) -> (f64, f64);
    /// `sweeps` applications as a smoother on x (A·x = b), x updated in place.
    pub fn smooth(&self, cx: &mut ComputeCtx, b: &GpuVec<f32>, x: &GpuVec<f32>, sweeps: u32) -> ComputeResult<()>;
}
impl GpuPreconditioner for Jacobi {} impl GpuPreconditioner for BlockJacobi3 {} impl GpuPreconditioner for Chebyshev {}

#[derive(Clone, Copy, Debug)]
pub struct PcgOpts { pub rel_tol: f64 /* 1e-6 */, pub max_iter: u32 /* 10_000 */, pub check_every: u32 /* 8 */ }
#[derive(Clone, Debug)]
pub struct SolveStats { pub iterations: u32, pub rel_residual: f64, pub converged: bool, pub history: Vec<(u32, f64)> }
/// `x` holds the initial guess on entry. Returns `converged: false` (not an error) at max_iter.
pub fn pcg_cpu(a: &dyn LinearOperator, m: &dyn CpuPreconditioner, b: &[f64], x: &mut [f64],
               opts: &PcgOpts, ctx: &JobCtx) -> ComputeResult<SolveStats>;
pub fn pcg_gpu(cx: &mut ComputeCtx, prim: &Primitives, a: &dyn GpuOperator, m: &dyn GpuPreconditioner,
               b: &GpuVec<f32>, x: &GpuVec<f32>, opts: &PcgOpts, ctx: &JobCtx) -> ComputeResult<SolveStats>;

#[derive(Clone, Copy, Debug)]
pub struct RefineOpts { pub rel_tol: f64 /* 1e-8 */, pub max_outer: u32 /* 20 */, pub inner_rel_tol: f64 /* 1e-4 */ }
#[derive(Clone, Debug)]
pub struct RefineStats { pub outer: u32, pub inner_iterations: u32, pub rel_residual: f64, pub converged: bool }
/// Mixed-precision iterative refinement: r = b − A·x in f64 (CPU), `inner(r) ≈ A⁻¹ r` (any
/// precision, typically `pcg_gpu`), x += d in f64. Errors with `NotConverged` when the
/// residual stops decreasing.
pub fn refine(a: &dyn LinearOperator, b: &[f64], x: &mut [f64], opts: &RefineOpts, ctx: &JobCtx,
              inner: &mut dyn FnMut(&[f64], &mut [f64], f64 /* inner tol */) -> ComputeResult<SolveStats>)
              -> ComputeResult<RefineStats>;

// ───────────── amg_compute::mg (CMP-05) ─────────────
/// One level of a structured (vertex-centred) grid operator, matrix-free on the GPU.
pub trait GridLevelOperator: GpuOperator {
    fn nodes(&self) -> [u32; 3];
    fn dofs_per_node(&self) -> u32;                   // 1 (scalar) or 3 (elasticity)
    /// Fixed (Dirichlet) DOF mask, one u32 per DOF (0/1).
    fn fixed(&self) -> &GpuVec<u32>;
    /// Next coarser level (nodes (n−1)/2+1 per axis), re-discretised from coarsened coefficients.
    fn coarsen(&self, cx: &mut ComputeCtx) -> ComputeResult<Box<dyn GridLevelOperator>>;
    /// Assembled f64 matrix with fixed DOFs eliminated (coarsest level only).
    fn assemble(&self, cx: &mut ComputeCtx, ctx: &JobCtx) -> ComputeResult<SymCsr>;
}
#[derive(Clone, Copy, Debug)]
pub struct GmgOpts {
    pub max_coarse_dofs: u64,                         // 50_000
    pub pre_smooth: u32, pub post_smooth: u32,        // 2, 2
    pub smoother: ChebyshevOpts,
}
/// V-cycle preconditioner; coarsest level solved by CMP-03 Cholesky (f64, CPU).
pub struct Gmg { /* levels, smoothers, transfer kernels, coarse Cholesky */ }
impl Gmg {
    pub fn build(cx: &mut ComputeCtx, prim: &Primitives, fine: std::sync::Arc<dyn GridLevelOperator>,
                 opts: &GmgOpts, ctx: &JobCtx) -> ComputeResult<Gmg>;
    pub fn levels(&self) -> usize;
    /// Re-coarsens after the fine coefficients changed (each SIMP iteration); keeps buffers.
    pub fn update(&mut self, cx: &mut ComputeCtx, ctx: &JobCtx) -> ComputeResult<()>;
}
impl GpuPreconditioner for Gmg {}
/// Reference operator: variable-coefficient Q1 Poisson (per-cell coefficient), 27-point.
pub struct Q1Poisson { /* nodes, coeff: GpuVec<f32>, fixed */ }
impl Q1Poisson {
    pub fn new(gpu: &Gpu, nodes: [u32; 3], coeff: &DenseGrid<f32>, fixed: &DenseGrid<u8>) -> ComputeResult<Self>;
}
impl GridLevelOperator for Q1Poisson {}

// ───────────── amg_compute::eigen (CMP-04; CMP-09) ─────────────
/// Solves with (K − σB). Implemented by S13 (own constraints) or by `CholeskyShift`/`LdltShift`.
pub trait ShiftInvert {
    fn shift(&self) -> f64;
    fn solve(&mut self, rhs: &mut [f64]) -> ComputeResult<()>;
    /// Sturm count (negative pivots); `None` when the factorisation cannot report it.
    fn inertia(&self) -> Option<Inertia>;
}
/// Factors K − σB with CMP-03 (Cholesky for σ < 0 in vibration mode, else LBLᵀ).
pub struct CholeskyShift { /* ... */ }
pub struct LdltShift { /* ... */ }
impl CholeskyShift { pub fn new(k: &SymCsr, b: &SymCsr, sigma: f64, opts: &FactorOpts, ctx: &JobCtx) -> ComputeResult<Self>; }
impl LdltShift { pub fn new(k: &SymCsr, b: &SymCsr, sigma: f64, opts: &FactorOpts, ctx: &JobCtx) -> ComputeResult<Self>; }
impl ShiftInvert for CholeskyShift {} impl ShiftInvert for LdltShift {}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EigenMode {
    /// K φ = λ M φ, M SPD; vectors M-orthonormal.
    Vibration,
    /// K φ = λ K_G φ, K SPD, K_G indefinite; vectors K-orthonormal.
    Buckling,
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Which { Nearest { count: u32 } /* nearest σ */, Band { lo: f64, hi: f64, max: u32 } }
#[derive(Clone, Copy, Debug)]
pub struct LanczosOpts {
    pub mode: EigenMode, pub which: Which,
    pub tol: f64,                                     // 1e-10 relative residual
    pub subspace: Option<u32>,                        // default max(2·count + 10, 30)
    pub max_restarts: u32,                            // 50
    pub seed: u64,                                    // start vector (deterministic)
}
#[derive(Clone, Debug)]
pub struct EigenResult {
    pub values: Vec<f64>,                             // ascending
    pub vectors: Vec<f64>,                            // n × values.len(), column-major
    pub residuals: Vec<f64>,                          // ‖Kφ − λBφ‖ / (|λ|·‖Bφ‖ + ‖Kφ‖)
    pub restarts: u32, pub solves: u32,
    /// Band mode: Sturm count in [lo, hi] matched the number found.
    pub sturm_checked: bool,
}
impl EigenResult { pub fn vector(&self, i: usize) -> &[f64]; }
/// `b` is M (vibration) or K_G (buckling). Band mode needs `inertia()` and may call
/// `reshift` to factor at a new σ inside the band.
pub fn lanczos(k: &dyn LinearOperator, b: &dyn LinearOperator, op: &mut dyn ShiftInvert,
               reshift: Option<&mut dyn FnMut(f64) -> ComputeResult<Box<dyn ShiftInvert>>>,
               opts: &LanczosOpts, ctx: &JobCtx) -> ComputeResult<EigenResult>;
/// LOBPCG (CMP-09): smallest `count` eigenpairs of K φ = λ M φ with a GPU preconditioner.
pub fn lobpcg(cx: &mut ComputeCtx, prim: &Primitives, k: &dyn GpuOperator, m: &dyn GpuOperator,
              precond: &dyn GpuPreconditioner, count: u32, opts: &LobpcgOpts, ctx: &JobCtx) -> ComputeResult<EigenResult>;
#[derive(Clone, Copy, Debug)]
pub struct LobpcgOpts { pub tol: f64 /* 1e-6 */, pub max_iter: u32 /* 500 */, pub seed: u64 }
```

**Expected from other specs** (owned there; listed so the dependency is explicit):
- **S00 / lane E (`amg-jobs::gpu`, FND-09, Done):** no change. `Gpu::compute_pipeline`,
  `upload`, `storage`, `readback`, `has`, `GpuError` are used as they are.
- **S08:** wraps `BrickGrid<f16>` / `GpuBrickGrid` in `SdfBricks` and `SampledGrid`; S08's
  per-tile classification maps to `set_tile_fill` (Fill0 = outside, Fill1 = inside). Its
  `amg-implicit` Cargo.toml gains `amg-compute` (allowed by the layering test).
- **S13:** uses `SymCsr` (builds the upper pattern itself, then `from_raw`), `SymbolicFactor`,
  `Cholesky`, `Ldlt`, `FactorEstimate`, `PivotReport`, `lanczos` with its own `ShiftInvert`,
  `GpuCsr`, `pcg_gpu`, `Chebyshev`, `refine`, and implements `GpuPreconditioner` for its
  p-multigrid.
- **S14:** implements `GridLevelOperator` for H8 elasticity (density-coarsened) and uses `Gmg`,
  `pcg_gpu`, `DenseGrid`.
- **S16:** `Primitives::{sort, scan, segmented_reduce, reduce}`, `AtomicCounter` (AM-13).

## Work packages
WPs are ≈ 1–3 days each. CMP-01..06 keep their IDs and meaning; CMP-07..11 are new. Times are
on the reference machine (note §1: Ryzen 7 5700X3D, 32 GB, RTX 5060 Ti 16 GB), release build,
GPU times from timestamps excluding upload and readback. "WARP" = the CI software adapter.
Benchmarks are criterion benches under `crates/amg-compute/benches/` registered with the FND-08
harness; measured numbers go into the Refinement log.

| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| CMP-07 | GPU layer: `GpuVec<T>` (binding-limit checks), `Kernel`, `ComputeCtx` (batched encoder, uniform arena, `dispatch_1d` 2-D folding, timestamps), `PRELUDE_WGSL`, `ComputeError` + `Diagnostic`, WARP test harness; crate deps (`half`, `bytemuck`, `rayon`) and THIRD_PARTY rows | FND-09, FND-05 | P1 | Todo | 1. On WARP and the reference GPU, `dispatch_1d` with n ∈ {0, 1, 255, 256, 257, 65 535·256 + 1, 2²⁶ + 3} writes every index exactly once (counter test). 2. `GpuVec::zeroed` past the binding limit returns `TooLarge`, no panic. 3. A destroyed device makes `finish` return `Gpu(DeviceLost)` within 1 s (no hang). 4. Bench: 1 000 empty dispatches recorded and submitted in one `ComputeCtx` cost ≤ 20 µs CPU each. 5. `take_timings` is non-empty on the reference GPU and empty without `TIMESTAMP_QUERY`. 6. `cargo deny` passes; `half` listed in THIRD_PARTY.md. |
| CMP-01 | GPU primitives: reduce (sum/min/max on f32/u32/i32), dot, exclusive/inclusive scan, compaction, segmented reduce, stable radix sort (keys + optional values, `key_bits`), `AtomicCounter`, `cpu` references; benchmarks vs CPU | CMP-07 | P1 | Todo | 1. Every primitive equals its `cpu` reference on sizes {0, 1, 255, 256, 257, 65 537, 2²⁰ + 7, 2²⁴} with random data, on WARP and the reference GPU: integers exactly, f32 sums bit-equal to the tree-order reference on the reference GPU and within n·ε₃₂·Σ\|x\| on WARP. 2. Same input → bit-identical output over 3 runs. 3. Sort is stable (equal keys keep value order; property test vs `slice::sort_by_key`). 4. Benches (reference GPU, 2²⁶ elements): reduce f32 ≥ 60 % of the measured device-to-device copy bandwidth; scan u32 ≥ 30 %; sort of 2²⁴ key-value pairs ≥ 1 G pairs/s and ≥ 3× rayon `par_sort_by_key` on the CPU; numbers recorded. |
| CMP-02 | Grids: `DenseGrid<T>`, `BrickGrid<T>` (two-level tree, `BrickLayout::{CELLS, CORNERS}`, fills, aprons, Morton `canonicalize`, `content_hash`), `GpuBrickGrid` (upload/download, deterministic scan-based `allocate`), `brick_grid_wgsl` | CMP-01 | P1 | Todo | 1. Sphere r = 50 at cell 0.5 baked as `d / cell` into `BrickGrid<f16>` (`CORNERS`, band ±4 cells, inside tiles Fill1): CPU `trilinear` error vs the analytic distance ≤ 0.01 cell inside the band; sign correct everywhere in the box (fills). 2. A GPU kernel using `bg_trilinear` at 1 M random points equals CPU `trilinear` within 1e-3 cell (WARP and reference GPU). 3. GPU `allocate` from a mask gives buffers bit-identical over 3 runs and the same brick order as CPU `canonicalize`; upload → download round-trips bit-exactly for f16, f32, u8. 4. Grids built in two insertion orders have equal `content_hash` after `canonicalize`. 5. Sphere r = 100 at cell 0.1: index (root + tiles) ≤ 10 % of brick bytes; bytes recorded. 6. Bench: ≥ 0.5 G `bg_trilinear` samples/s at random in-band points on the reference GPU. 7. A box needing > 2²⁴ root tiles returns `Shape`. |
| CMP-03 | Sparse direct: `SymCsr` (upper CSR, validation, deterministic `spmv`, `add_scaled`), `TripletBuilder`, `SymbolicFactor` (reusable, `estimate`), `Cholesky` and `Ldlt` (faer, LBLᵀ) with multi-RHS, `PivotReport`, `Inertia`; `testprob` generators | FND-03, FND-04 | P1 | Todo | 1. `q1_elasticity(55, Clamped)` (499 125 DOF): analyse + factor + solve ≤ 20 s (S13 note §8.1 split of the vision 30 s budget), relative residual ≤ 1e-10. 2. `q1_elasticity(70, Clamped)` (1 029 000 DOF): completes on 32 GB; time and peak RSS recorded; `estimate().bytes` within 10 % of the measured factor storage. 3. `solve_many` with 6 RHS takes ≤ 2× one `solve_in_place`. 4. `q1_elasticity(_, Free)` gives `NotPositiveDefinite` with a row, or a `PivotReport` with `below_threshold`; never a panic. 5. `Ldlt::inertia` on random diagonal-plus-low-rank matrices equals the dense eigenvalue sign count (proptest). 6. Re-factoring with new values on the same pattern skips analysis (counter). 7. Factor and solve are bit-identical over 2 runs; `spmv` bit-identical across 1, 4, 16 threads. 8. `from_raw` rejects unsorted, lower or out-of-range input with `InvalidMatrix`. |
| CMP-04 | Eigen: thick-restart shift-invert Lanczos (`lanczos`) with full B-reorthogonalisation, `EigenMode::{Vibration, Buckling}`, `Which::{Nearest, Band}` with Sturm check and re-shift, `ShiftInvert` callback, `CholeskyShift`, `LdltShift` | CMP-03 | P1 | Todo | 1. 50 random sparse SPD pencils (n ≤ 3 000): the 10 eigenvalues nearest σ match faer's dense generalised solver within 1e-10 relative; ‖ΦᵀBΦ − I‖∞ ≤ 1e-10. 2. Free-free `q1_elasticity(20, Free)` with σ < 0: exactly 6 values with \|λ\| ≤ 1e-8·λ₇, then the next 4 within 1e-9 of dense. 3. Band mode finds every eigenvalue in [lo, hi] on 20 pencils (vs dense), `sturm_checked` true; a band with more than `max` values returns the `max` lowest and says so. 4. Buckling mode on a pencil with known eigenvalues (indefinite K_G) within 1e-9. 5. Bench: `q1_elasticity(55, Clamped)`, 8 modes, including the factorisation, ≤ 40 s (FEM-07 budget 60 s). 6. Cancel during iterations returns `Cancelled` within 1 s after the factorisation ends. |
| CMP-08 | Iterative core: `LinearOperator`, `CpuPreconditioner`, `GpuOperator`, `GpuPreconditioner`; `GpuCsr` (full storage, `update_values`); `Jacobi`, `BlockJacobi3`, `Chebyshev` (Lanczos bounds, `smooth`); `pcg_cpu`, `pcg_gpu` (async convergence checks) on a crate-private `VectorBackend`; `refine` | CMP-01, CMP-03 | P1 | Todo | 1. `pcg_gpu` and `pcg_cpu` on `q1_poisson(65)` and `q1_elasticity(40)` reach 1e-5 relative residual with iteration counts within 2 % of each other (WARP and reference GPU). 2. `refine` + `pcg_gpu` (block Jacobi) on `q1_elasticity(70)` reaches ‖r‖/‖b‖ ≤ 1e-8, and the solution matches CMP-03 Cholesky within 1e-7 in relative energy norm. 3. Bench: `q1_elasticity(70)` with `BlockJacobi3` ≥ 250 PCG iterations/s; GPU idle between submissions ≤ 10 % (timestamps). 4. Iteration counts and results bit-identical over 3 runs. 5. `Chebyshev` λmax estimate within 5 % of the dense value on a 3 000-DOF problem. 6. Cancel returns `Cancelled` within 100 ms; device loss returns an error. |
| CMP-05 | Geometric multigrid for structured grids: `GridLevelOperator`, `Gmg` (Chebyshev–Jacobi smoothing, trilinear transfer, re-discretised coarse levels, padding to `2^k·m + 1`, fixed masks, coarse CPU Cholesky, `update`), `Q1Poisson` reference operator; MGPCG | CMP-08, CMP-02 | P1 | Todo | 1. `Q1Poisson` with a `simp_field` coefficient (contrast 1e3), clamped on one face: MGPCG reaches 1e-6 relative residual in ≤ 25 iterations at 129³ nodes, and the 257³ count is ≤ 1.2× the 129³ count (h-independence). 2. Bench: 257³ nodes (16 974 593 DOF) solve < 2 s on the reference GPU (headroom for GEN-03's 1 M-voxel elasticity < 2 s). 3. Contrast 1e6: iteration counts recorded (no limit). 4. Non-`2^k·m + 1` dims (e.g. 100³) converge within 1.3× the padded power-of-two case. 5. `update` after a coefficient change reuses all buffers (allocation counter) and gives the same iterations as a fresh `build`. 6. Matches `pcg_cpu` + Cholesky reference within 1e-5 relative on 33³. |
| CMP-06 | Optional CUDA backend (`cudarc`, NVRTC at runtime) behind feature `cuda`: `VectorBackend` impl, CSR SpMV, Chebyshev, `SolverDevice::Cuda`; ships only if benchmarks warrant (ADR-004) | CMP-05, CMP-08 | P3 | Todo | 1. Builds and tests pass with and without `cuda`; without a CUDA driver `SolverDevice::Cuda` creation returns an error, not a crash. 2. CMP-08 test 1 and CMP-05 test 1 pass on CUDA. 3. Bench table CUDA vs wgpu for CMP-08 bench 3 and CMP-05 bench 2 recorded; made the default only if ≥ 1.5× faster (owner decides, QUESTIONS #80). |
| CMP-09 | LOBPCG (`lobpcg`): block, preconditioned, GPU operators, f64 Rayleigh–Ritz on the CPU, soft locking | CMP-08, CMP-04 | P2 | Todo | 1. On `q1_elasticity(40, Clamped)`, 8 smallest eigenvalues within 1e-6 relative of CMP-04 Lanczos. 2. `q1_elasticity(88, Clamped)` (2 044 416 DOF), 8 modes with `BlockJacobi3`, < 120 s on the reference GPU; iterations recorded. 3. Deterministic over 2 runs. |
| CMP-10 | Nested-dissection ordering: geometric ND from node coordinates (`Ordering::NestedDissection`), or METIS behind `metis` if geometric ND is not enough | CMP-03 | P2 | Todo | 1. On `q1_elasticity(55)` and the S13 500 k DOF bracket, factor nnz ≥ 25 % lower than AMD. 2. Becomes the default only if factor time drops ≥ 20 %; numbers recorded either way. Only scheduled if CMP-03 test 1 misses its budget or S13 FEM-04 bench 5 does. |
| CMP-11 | Smoothed-aggregation AMG preconditioner: CPU setup (strength of connection, aggregation, rigid-body near-null space for 3-DOF nodes), GPU V-cycle with `Chebyshev` smoothing, `GpuPreconditioner` | CMP-08 | P2 | Todo | 1. Unstructured test (`q1_elasticity` with randomly perturbed nodes and a random symmetric renumbering): iterations to 1e-6 ≤ 2× `Gmg` on the unperturbed grid of equal size. 2. Setup for 1 M DOF < 5 s CPU. 3. Iterations grow ≤ 1.5× from 125 k to 1 M DOF. |

Status values: `Todo`, `Refining`, `Ready`, `In progress (agent/branch)`, `Review`, `Done`, `Blocked (reason)`.

## Testing
- Every GPU routine has a CPU reference in the crate, and every GPU test runs on WARP in CI
  (FND-05) and on the reference GPU locally. Timing assertions run only on the reference GPU
  (`AMG_BENCH_REFERENCE=1`); WARP checks correctness only.
- Property tests (proptest): primitives vs CPU references; `SymCsr` validation; sort stability;
  `Ldlt::inertia` vs dense.
- Solver tests on `testprob` problems with known answers (manufactured solutions for Poisson,
  dense eigen references for n ≤ 3 000).
- Determinism tests: repeated runs bit-identical (CPU across thread counts; GPU on one device).
- Golden: CMP-02's sphere grid (error and byte counts) under `tests/golden/compute/`. No other
  WP produces geometry.

## Performance targets
| Target | Value | Tied to |
|---|---|---|
| Direct solve, 499 k DOF 3-D elasticity (CMP-03) | ≤ 20 s analyse + factor + solve | vision FEA 500 k DOF < 30 s (S13 note §8.1 split) |
| Modal, 499 k DOF, 8 modes (CMP-04) | ≤ 40 s | FEM-07 < 60 s |
| GPU PCG throughput, 1.03 M DOF elasticity (CMP-08) | ≥ 250 iterations/s | FEM-08 2 M DOF < 60 s |
| MGPCG, 17 M DOF scalar Q1 (CMP-05) | < 2 s, ≤ 25 iterations at 129³ | GEN-03 1 M voxels < 2 s; vision topopt 1 M voxels × 100 iterations < 5 min |
| Primitives (CMP-01) | reduce ≥ 60 %, scan ≥ 30 % of copy bandwidth; sort ≥ 1 G pairs/s | AM-13, IMP-02 |
| Brick lookups (CMP-02) | ≥ 0.5 G trilinear samples/s | IMP-02, IMP-05 |
| UI thread | never blocks: all waits are on job threads (`ComputeCtx::finish`, `Readback::wait`) | vision 16 ms |

## Open questions
1. ~~(from S10, 2026-10-20) `SHADER_F64` is Vulkan-only in wgpu 30, so the default DX12 device never has it; plan f32 or emulated-double kernels.~~
   **Resolved 2026-11-08:** f32 kernels only, accuracy through f64 iterative refinement on the
   CPU; no emulated double (WGSL `fma` is not guaranteed fused on DX12, FXC may reorder, and the
   RTX 5060 Ti runs FP64 at 1/64 rate anyway). Note §2.1.
2. ~~(from S16, 2026-10-20) AM-13 (GPU orientation scoring, Phase 4) depends on CMP-01 and keeps its backend behind `ComputeBackend` with a CPU reference.~~
   **Resolved 2026-11-08:** CMP-01 ships a `cpu` reference for every primitive, so AM-13's CPU
   path can use the same semantics. `ComputeBackend` stays S07/S16's enum; `amg-compute` does
   not define a competing one (it has `SolverDevice` for solvers only).
3. ~~(integration, 2026-10-20) Agreed earlier-wave repoint applied: CMP-01 now depends on FND-09 (`amg-jobs::gpu`) instead of FND-04.~~
   **Resolved 2026-11-08:** the FND-09 dependency now sits on CMP-07 (the new GPU layer); CMP-01
   depends on CMP-07.
4. ~~(from S08, 2026-11-08) IMP-02 and IMP-12 need from CMP-02 a sparse brick grid with brick edge 8, `half::f16` (and f32) storage, insert/lookup by brick coordinate, and a GPU layout (brick table + hash or dense index) that WGSL can read. S08 wraps it as `SdfBricks` / `SampledGrid`, so the CMP-02 type name can still change.~~
   **Resolved 2026-11-08:** `BrickGrid<T>` / `GpuBrickGrid` (Interfaces): edge 8, f16/f32/u8,
   `insert`/`lookup` by `BrickCoord`, `CORNERS` layout for S08's 9³ samples, two fill codes for
   inside/outside, a two-level dense index (not a hash, for deterministic GPU allocation) and
   generated WGSL accessors. Note §4.
5. ~~(from S13, 2026-11-08) S13 lists what FEM needs from CMP-03 (symbolic/numeric Cholesky, multi-RHS, pivot report, memory estimate, LDLᵀ), CMP-04 (shift-invert Lanczos with a factorisation callback) and CMP-05 (f32 CSR PCG with a `Preconditioner` trait for p-multigrid) under Interfaces → "Needed from other specs"; see `docs/notes/simulation-fem.md` §8.~~
   **Resolved 2026-11-08:** all in Interfaces. CMP-03: `SymCsr` (upper), `SymbolicFactor` +
   `Cholesky`/`Ldlt`, `solve_many`, `PivotReport`, `FactorEstimate`, `Inertia`. CMP-04:
   `lanczos` with a `ShiftInvert` callback, `Which::{Nearest, Band}`, B-orthonormal vectors. The
   f32 CSR PCG with the `GpuPreconditioner` trait and Chebyshev–Jacobi is in **CMP-08** (new),
   which CMP-05 now depends on; FEM-08 can depend on CMP-08 instead of CMP-05 if S13 wants it
   earlier (note in S13 open questions).
6. CUDA timing: ADR-004's amendment calls CUDA the first optimisation once wgpu solvers are
   benchmarked, but the roadmap keeps CMP-06 at P3 in Phase 6. (QUESTIONS #80.)
7. Accept the no-f64-on-GPU policy (note §2.1) as an ADR-004 note? (QUESTIONS #81.)

## Refinement log
### 2026-11-08 - S15-refine (lane F refinement)
- **Read:** AGENTS.md, vision, architecture, ADRs (ADR-004, ADR-006, ADR-012, ADR-014),
  roadmap, QUESTIONS, S00 (FND-09 log), S08 (Interfaces, Interface-changes note for S15, the
  2026-11-08 log), S13 (refined 2026-11-08: Needs from S15, `docs/notes/simulation-fem.md` §7–8),
  S14, S16 (AM-13, AM-09, `am-build-prep.md` §5.4), S10 (`SHADER_F64`), the implicit-modeling
  note §3–5, and the code: `crates/amg-compute` (empty: Cargo.toml depends on core and jobs),
  `crates/amg-jobs/src/gpu` (`Gpu`, `GpuConfig` defaults DX12 + FXC + optional `SHADER_F64 |
  TIMESTAMP_QUERY`, `upload`, `storage`, `readback`, `compute_pipeline` cache with override
  constants, `GpuError` + `Diagnostic`), `JobCtx`, the workspace pins (`wgpu =30.0.1`,
  `faer =0.23.2` in `amg-mesh`/`amg-sketch`, `blake3`, `rayon`), and S07's `ComputeBackend`.
- **Reference machine checked** (owner's machine): Ryzen 7 5700X3D 8 cores, 32 GB RAM, RTX 5060
  Ti 16 GB (driver 617.14). Used for the direct-solver size limits and VRAM budgets.
- **Researched:** wgpu 30 feature support on DX12/FXC (no subgroups, no `f16`, no f64), WGSL
  atomics and packing built-ins, portable scan (Levien 2021; decoupled look-back hazards;
  decoupled fallback 2025), Onesweep and `wgpu_sort`, OpenVDB/NanoVDB, faer's sparse
  factorisations (`=0.23.2` pinned; 0.24.4 on crates.io), CHOLMOD licensing, mixed-precision
  refinement (Carson–Higham), spectral-transformation and thick-restart Lanczos, buckling mode
  (Grimes–Lewis–Simon), LOBPCG, Chebyshev smoothing (Adams et al.), GPU topology-optimisation
  MGPCG (Wu–Dick–Westermann 2016, Aage et al. 2017), smoothed aggregation, AMGCL. Fusion has no
  user-visible behaviour for this spec (its solvers are internal); the parity targets come from
  S13/S14 budgets. Details and sources in `docs/notes/compute-numerics.md`.
- **Decided:**
  - f32-only GPU kernels with f64 CPU refinement; no df64 (open question 1).
  - Kernels target FXC/SM 5.1 (no subgroups); f16 packed in `u32`.
  - Deterministic GPU results on one device: tree reductions, scan-based allocation, no float
    atomics.
  - Reduce-then-scan scan and radix sort (no look-back) for portability.
  - Brick grid: fixed two-level tree with a dense root over 64³-cell tiles (matches S08's
    tiles), not a hash (open question 4).
  - faer only, at the workspace pin; no CHOLMOD unless CMP-03 misses budget by > 2×; LU dropped
    (no consumer); LBLᵀ for indefinite shifts; inertia for Sturm checks.
  - IC(0) dropped (sequential triangular solves; S13 uses p-multigrid). Block Jacobi and
    Chebyshev instead.
  - GPU CSR uses full storage (no float atomics for symmetric SpMV).
  - PCG generic over a crate-private `VectorBackend` (ADR-004 amendment).
  - `amg-compute` defines no `ComputeBackend`; `SolverDevice` covers solvers (open question 2).
- **WP changes** (CMP-01..06 keep their IDs and meaning; CMP-01, CMP-02, CMP-03, CMP-04, CMP-05
  are referenced by S08, S13, S14, S16, the AM note and the roadmap):
  - New CMP-07 (GPU layer) split out of CMP-01; takes the FND-09 dep. CMP-01 keeps reductions,
    scans, sort (and adds compaction, segmented reduce, atomic counters for S16); deps CMP-07.
  - CMP-02: unchanged meaning; Done-when made concrete (was "used by IMP-02, GEN-02", which is
    not checkable inside S15).
  - CMP-03: LU dropped; symbolic reuse, estimate, pivot report, LDLᵀ/LBLᵀ, inertia, `testprob`
    added; adds dep FND-04 (`JobCtx`). The "1 M DOF benchmark" stays (test 2) with the 500 k
    budget as the hard bar.
  - CMP-04: Lanczos with vibration and buckling modes and band search. LOBPCG split out to new
    CMP-09 (P2).
  - New CMP-08 (operator/preconditioner traits, GPU CSR, PCG, Chebyshev, refinement) split out of
    CMP-05. CMP-05 keeps its meaning as the full GPU PCG + matrix-free + multigrid stack by
    depending on CMP-08; its own work is the geometric multigrid. IC(0) dropped. Adds dep CMP-02
    (`DenseGrid`).
  - CMP-06: deps CMP-05 and CMP-08; Done-when added; timing question to the owner (#80).
  - New CMP-10 (nested-dissection ordering, P2, conditional) and CMP-11 (smoothed-aggregation
    AMG, P2; S13 note §8.3 "later step").
- **Roadmap:** Phase 2 F gains CMP-07 (before CMP-01). Phase 3 F gains CMP-08. Phase 5 F gains
  CMP-09, CMP-10, CMP-11. Every WP is scheduled no earlier than its deps: CMP-07 needs FND-09
  and FND-05 (Phase 0), CMP-05 needs CMP-02 (Phase 2) and CMP-08 (Phase 3). Consumers stay
  behind their deps: IMP-02 (Phase 3) after CMP-02, FEM-04 (Phase 4) after CMP-03, AM-13
  (Phase 4) after CMP-01, FEM-08 and GEN-03 (Phase 4) after CMP-05.
- **Questions for the owner:** QUESTIONS #80 (CUDA timing), #81 (no-f64-on-GPU policy as an
  ADR-004 note).
- **Requests to other specs:** S13 (FEM-08 may depend on CMP-08 instead of CMP-05; S13's
  Interfaces name `amg_compute` types), S14 (GEN-03 implements `GridLevelOperator`), S08 (CMP-02
  shape delivered; fill mapping). Each is a short dated note in that spec.
- Status set to `Refined`.

## Interface changes
None yet (the spec has not reached `Ready`).
