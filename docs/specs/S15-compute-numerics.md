# S15 — GPU compute & numerics library

| | |
|---|---|
| **Status** | Ready |
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
- Runtime precision selection (CMP-07): numeric kernels in f64 where the device has wgpu
  `SHADER_F64`, in f32 with f64 iterative refinement on the CPU where it does not (note §2.1,
  ADR-004 note of 2026-11-09). Both paths are tested.
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
- Emulated-double (df64) GPU kernels (note §2.1). Native f64 kernels are in scope where the
  device supports them.

## Design
- **GPU constraints** (note §1): DX12 with FXC by default, so no subgroups and no `f16` shader
  type. f64 exists only where wgpu reports `SHADER_F64`, which in wgpu 30 is Vulkan only (not
  DX12, not WARP, not Metal). Kernels use shared memory and barriers only. f16 data is packed two per `u32`
  (`pack2x16float`/`unpack2x16float`). Bindings stay under the device's storage-binding limit;
  1-D launches fold into 2-D grids past 65 535 workgroups.
- **Precision** (note §2.1; open question 1 as re-resolved on 2026-11-09; ADR-004 note): numeric
  kernels are written once over a WGSL alias `real` and compiled as f64 where the device has
  `SHADER_F64`, as f32 otherwise. The precision is a runtime value (`Precision`), resolved once
  per device from a `PrecisionPolicy` (default `Auto`; `AMG_COMPUTE_PRECISION=auto|f32|f64`
  overrides it for tests and diagnostics). The f64 path solves to the requested tolerance
  directly. The f32 path (the default DX12 device, WARP) gets its accuracy from mixed-precision
  iterative refinement with f64 residuals on the CPU. `solve_gpu` is the one place that picks
  the path; callers build their operator in the resolved precision. Both paths are tested
  (Testing). Every GPU routine has an f64 (or exact integer) CPU reference with the same
  semantics. Integer and storage kernels (scan, compaction, sort, brick grids) do not depend on
  the precision.
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
  full-storage `GpuCsr<R>` (f32 or f64 values), Chebyshev–Jacobi smoothers shared with S13's p-multigrid, matrix-free
  geometric multigrid for voxel grids.

## Interfaces
Signatures other crates rely on (`amg-implicit`, `amg-fem`, `amg-topopt`, `amg-am`). Types from
`amg-core`: `Cancelled`, `Diagnostic`, `Severity`, `Point3`. From `amg-jobs`: `JobCtx`, and from
`amg_jobs::gpu`: `Gpu`, `GpuError`, `Readback`, `ComputePipelineDesc`, `wgpu` (re-exported).
New dependencies: `half` (MIT/Apache-2.0, feature `bytemuck`), `faer =0.23.2`
(`std`, `sparse-linalg`), `rayon`, `bytemuck`, `blake3`; `cudarc` behind `cuda`.
Cargo features: `gpu` (default, needs `amg-jobs/gpu`), `cuda` (CMP-06), `metis` (CMP-10, only
if chosen there). Precision is never a Cargo feature: it is chosen at runtime per device
(`amg_compute::precision`). GPU numeric types are generic over `R: GpuReal` (f32 or f64) with
`f32` as the default type parameter, so `dyn GpuOperator` still means the f32 operator.

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
    /// E.g. an f64 kernel or `PrecisionPolicy::RequireF64` on a device without `SHADER_F64`.
    #[error("not supported on this device: {0}")] Unsupported(String),
    /// E.g. an unknown `AMG_COMPUTE_PRECISION` value.
    #[error("invalid setting: {0}")] Config(String),
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
impl SolverDevice {
    /// `Cpu` → F64; `Wgpu` → `policy.resolve(gpu)`; `Cuda` → F64 unless `ForceF32`.
    pub fn precision(&self, policy: precision::PrecisionPolicy) -> ComputeResult<precision::Precision>;
}

// ───────────── amg_compute::precision (CMP-07) ─────────────
/// Floating-point precision of GPU numeric kernels.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Precision { F32, F64 }
/// How the precision is chosen for a device.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PrecisionPolicy {
    /// F64 iff the device has `SHADER_F64`, else F32 (the owner's policy; the default).
    #[default] Auto,
    /// The f32 + refinement path on any device (tests both paths on one f64-capable device).
    ForceF32,
    /// Errors with `Unsupported` on a device without `SHADER_F64` (the f64 test gate).
    RequireF64,
}
/// `auto` | `f32` | `f64`, case-insensitive; unset or empty = `Auto`.
pub const PRECISION_ENV: &str = "AMG_COMPUTE_PRECISION";
impl PrecisionPolicy {
    /// Errors with `Config` on any other value.
    pub fn parse(value: Option<&str>) -> ComputeResult<PrecisionPolicy>;
    pub fn from_env() -> ComputeResult<PrecisionPolicy>;
    pub fn resolve(self, gpu: &Gpu) -> ComputeResult<Precision>;
}
/// `Precision::F64` iff `gpu.has(wgpu::Features::SHADER_F64)`.
pub fn device_precision(gpu: &Gpu) -> Precision;
/// Scalars GPU numeric kernels are instantiated for: f32 and f64.
pub trait GpuReal: bytemuck::Pod + std::fmt::Debug + Send + Sync + sealed::Sealed + 'static {
    const PRECISION: Precision;
    /// `"f32"` or `"f64"`: the WGSL type the prelude binds to `real`.
    const WGSL: &'static str;
    fn from_f64(v: f64) -> Self;
    fn to_f64(self) -> f64;
}
impl GpuReal for f32 {} impl GpuReal for f64 {}

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
    /// Prepends `prelude_wgsl(Precision::F32)` to `wgsl`; cached through `Gpu::compute_pipeline`.
    pub fn new(gpu: &Gpu, label: &str, wgsl: &str, entry: &str, constants: &[(&str, f64)]) -> ComputeResult<Self>;
    /// Same with the prelude for `precision` (`alias real = f64;` for F64). Errors with
    /// `Unsupported` for F64 on a device without `SHADER_F64`. The pipeline cache keys on the
    /// full source, so both variants of one kernel are cached side by side.
    pub fn with_precision(gpu: &Gpu, label: &str, wgsl: &str, entry: &str, constants: &[(&str, f64)],
                          precision: Precision) -> ComputeResult<Self>;
    pub fn precision(&self) -> Precision;
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
/// `amg_linear_id(gid, nwg) -> u32`, `WG` override, `alias real = f32;` (or `f64`), shared constants.
pub fn prelude_wgsl(precision: Precision) -> &'static str;

// ───────────── amg_compute::prim (CMP-01) ─────────────
#[derive(Clone, Copy, Debug, PartialEq, Eq)] pub enum ReduceOp { Sum, Min, Max }
#[derive(Clone, Copy, Debug, PartialEq, Eq)] pub enum ScanKind { Exclusive, Inclusive }
/// Element types the reductions accept: f32, f64 (needs `SHADER_F64`), u32, i32.
pub trait ReduceScalar: bytemuck::Pod + sealed::Sealed {}

/// Cached pipelines for all primitives plus reusable scratch buffers.
pub struct Primitives { /* kernels, scratch pool */ }
impl Primitives {
    /// f64 pipelines are built lazily and only on a device with `SHADER_F64`; an f64 call on
    /// any other device returns `Unsupported`.
    pub fn new(gpu: &Gpu) -> ComputeResult<Self>;
    /// Writes the result to `out[out_index]` (stays on the GPU; no readback).
    pub fn reduce<T: ReduceScalar>(&self, cx: &mut ComputeCtx, op: ReduceOp, input: &GpuVec<T>,
                                   out: &GpuVec<T>, out_index: u64) -> ComputeResult<()>;
    pub fn dot<R: GpuReal>(&self, cx: &mut ComputeCtx, a: &GpuVec<R>, b: &GpuVec<R>,
                           out: &GpuVec<R>, out_index: u64) -> ComputeResult<()>;
    /// `total` (optional) receives the sum of all inputs (the allocation size).
    pub fn scan(&self, cx: &mut ComputeCtx, kind: ScanKind, input: &GpuVec<u32>, output: &GpuVec<u32>,
                total: Option<(&GpuVec<u32>, u64)>) -> ComputeResult<()>;
    /// Keeps `values[i]` where `flags[i] != 0`, in order; `count[0]` = kept count.
    pub fn compact(&self, cx: &mut ComputeCtx, flags: &GpuVec<u32>, values: &GpuVec<u32>,
                   out: &GpuVec<u32>, count: &GpuVec<u32>) -> ComputeResult<()>;
    /// Segment s covers `values[offsets[s]..offsets[s+1]]`; empty segments give the identity.
    pub fn segmented_reduce<R: GpuReal>(&self, cx: &mut ComputeCtx, op: ReduceOp, values: &GpuVec<R>,
                                        offsets: &GpuVec<u32>, out: &GpuVec<R>) -> ComputeResult<()>;
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
/// CPU references with identical semantics (f32 and f64 sums use the GPU's tree order).
pub mod cpu {
    pub fn reduce<T: super::ReduceScalar>(op: super::ReduceOp, v: &[T]) -> T;
    pub fn dot<R: GpuReal>(a: &[R], b: &[R]) -> R;
    pub fn scan(kind: super::ScanKind, v: &[u32]) -> (Vec<u32>, u32);
    pub fn compact(flags: &[u32], values: &[u32]) -> Vec<u32>;
    pub fn segmented_reduce<R: GpuReal>(op: super::ReduceOp, values: &[R], offsets: &[u32]) -> Vec<R>;
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

/// GPU operator in precision `R` (f32 or f64). `apply` records work into `cx`; it does not submit.
pub trait GpuOperator<R: GpuReal = f32>: Send + Sync {
    fn dim(&self) -> u64;
    fn apply(&self, cx: &mut ComputeCtx, x: &GpuVec<R>, y: &GpuVec<R>) -> ComputeResult<()>;
    /// Diagonal (dim) or 3×3 node blocks (9·dim/3, row-major) for block Jacobi.
    fn diagonal(&self, cx: &mut ComputeCtx, out: &GpuVec<R>) -> ComputeResult<()>;
    fn block_diagonal(&self, _cx: &mut ComputeCtx, _out: &GpuVec<R>) -> ComputeResult<bool> { Ok(false) }
}
/// z ≈ A⁻¹ r. S13 plugs its p-multigrid in here; CMP-05 `Gmg` implements it.
pub trait GpuPreconditioner<R: GpuReal = f32>: Send + Sync {
    fn apply(&self, cx: &mut ComputeCtx, r: &GpuVec<R>, z: &GpuVec<R>) -> ComputeResult<()>;
}
/// Full (both triangles) storage: `R` values (rounded from the f64 `SymCsr` for f32), u32 columns.
pub struct GpuCsr<R: GpuReal = f32> { /* row_ptr: GpuVec<u32>, cols, vals: GpuVec<R>, n, nnz */ }
impl<R: GpuReal> GpuCsr<R> {
    /// `Unsupported` for f64 on a device without `SHADER_F64`.
    pub fn upload(gpu: &Gpu, a: &SymCsr) -> ComputeResult<GpuCsr<R>>;
    /// Overwrites values on the same pattern (Newton steps, SIMP updates).
    pub fn update_values(&self, gpu: &Gpu, a: &SymCsr) -> ComputeResult<()>;
    pub fn bytes(&self) -> u64;
}
impl<R: GpuReal> GpuOperator<R> for GpuCsr<R> { /* ... */ }
pub struct Jacobi<R: GpuReal = f32> { /* inverse diagonal */ }
pub struct BlockJacobi3<R: GpuReal = f32> { /* inverted 3×3 blocks */ }
/// Chebyshev polynomial of degree `degree` in D⁻¹A on [λmax/ratio, 1.1·λmax];
/// λmax from `estimate_steps` Lanczos steps. Used as smoother (S13 p-MG, CMP-05) or preconditioner.
pub struct Chebyshev<R: GpuReal = f32> { /* degree, bounds, inv diag */ }
impl<R: GpuReal> Jacobi<R> { pub fn new(cx: &mut ComputeCtx, a: &dyn GpuOperator<R>) -> ComputeResult<Self>; }
impl<R: GpuReal> BlockJacobi3<R> { pub fn new(cx: &mut ComputeCtx, a: &dyn GpuOperator<R>) -> ComputeResult<Self>; }
#[derive(Clone, Copy, Debug)]
pub struct ChebyshevOpts { pub degree: u32 /* 2 */, pub ratio: f64 /* 30.0 */, pub estimate_steps: u32 /* 10 */ }
impl<R: GpuReal> Chebyshev<R> {
    pub fn new(cx: &mut ComputeCtx, prim: &Primitives, a: std::sync::Arc<dyn GpuOperator<R>>,
               opts: ChebyshevOpts, ctx: &JobCtx) -> ComputeResult<Self>;
    pub fn bounds(&self) -> (f64, f64);
    /// `sweeps` applications as a smoother on x (A·x = b), x updated in place.
    pub fn smooth(&self, cx: &mut ComputeCtx, b: &GpuVec<R>, x: &GpuVec<R>, sweeps: u32) -> ComputeResult<()>;
}
impl<R: GpuReal> GpuPreconditioner<R> for Jacobi<R> {}
impl<R: GpuReal> GpuPreconditioner<R> for BlockJacobi3<R> {}
impl<R: GpuReal> GpuPreconditioner<R> for Chebyshev<R> {}

#[derive(Clone, Copy, Debug)]
pub struct PcgOpts { pub rel_tol: f64 /* 1e-6 */, pub max_iter: u32 /* 10_000 */, pub check_every: u32 /* 8 */ }
#[derive(Clone, Debug)]
pub struct SolveStats { pub iterations: u32, pub rel_residual: f64, pub converged: bool, pub history: Vec<(u32, f64)> }
/// `x` holds the initial guess on entry. Returns `converged: false` (not an error) at max_iter.
pub fn pcg_cpu(a: &dyn LinearOperator, m: &dyn CpuPreconditioner, b: &[f64], x: &mut [f64],
               opts: &PcgOpts, ctx: &JobCtx) -> ComputeResult<SolveStats>;
pub fn pcg_gpu<R: GpuReal>(cx: &mut ComputeCtx, prim: &Primitives, a: &dyn GpuOperator<R>,
                           m: &dyn GpuPreconditioner<R>, b: &GpuVec<R>, x: &GpuVec<R>, opts: &PcgOpts,
                           ctx: &JobCtx) -> ComputeResult<SolveStats>;

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

/// An operator and preconditioner in the precision the device resolved to. Callers build it
/// with `match policy.resolve(gpu)? { Precision::F32 => GpuSystem::F32 { .. }, Precision::F64 =>
/// GpuSystem::F64 { .. } }`, instantiating their generic operator once per arm.
pub enum GpuSystem {
    F32 { a: std::sync::Arc<dyn GpuOperator<f32>>, m: std::sync::Arc<dyn GpuPreconditioner<f32>> },
    F64 { a: std::sync::Arc<dyn GpuOperator<f64>>, m: std::sync::Arc<dyn GpuPreconditioner<f64>> },
}
impl GpuSystem { pub fn precision(&self) -> Precision; }
#[derive(Clone, Copy, Debug)]
pub struct GpuSolveOpts {
    pub rel_tol: f64,                                 // 1e-8; on the f64 CPU residual when `cpu_a` is given
    pub pcg: PcgOpts,                                 // max_iter, check_every (its rel_tol is set per path)
    pub refine: RefineOpts,                           // max_outer; inner_rel_tol used on the F32 path only
}
#[derive(Clone, Debug)]
pub struct GpuSolveStats {
    pub precision: Precision,
    pub outer: u32, pub inner_iterations: u32,
    pub rel_residual: f64,                            // f64 CPU residual when `cpu_a` is given
    pub converged: bool,
}
/// The one place that picks the precision path. `b`, `x` are f64 on the CPU (`x` = initial guess).
/// With `cpu_a`: `refine` over `cpu_a` with `pcg_gpu::<R>` as the inner solve; the inner tolerance
/// is `refine.inner_rel_tol` on F32 (several outer steps) and `rel_tol` on F64 (normally one outer
/// step). Without `cpu_a` (loose solves, e.g. S14 topopt at ≥ 1e-5): one `pcg_gpu::<R>` to `rel_tol`.
pub fn solve_gpu(cx: &mut ComputeCtx, prim: &Primitives, sys: &GpuSystem, cpu_a: Option<&dyn LinearOperator>,
                 b: &[f64], x: &mut [f64], opts: &GpuSolveOpts, ctx: &JobCtx) -> ComputeResult<GpuSolveStats>;

// ───────────── amg_compute::mg (CMP-05) ─────────────
/// One level of a structured (vertex-centred) grid operator, matrix-free on the GPU.
pub trait GridLevelOperator<R: GpuReal = f32>: GpuOperator<R> {
    fn nodes(&self) -> [u32; 3];
    fn dofs_per_node(&self) -> u32;                   // 1 (scalar) or 3 (elasticity)
    /// Fixed (Dirichlet) DOF mask, one u32 per DOF (0/1).
    fn fixed(&self) -> &GpuVec<u32>;
    /// Next coarser level (nodes (n−1)/2+1 per axis), re-discretised from coarsened coefficients.
    fn coarsen(&self, cx: &mut ComputeCtx) -> ComputeResult<Box<dyn GridLevelOperator<R>>>;
    /// Assembled f64 matrix with fixed DOFs eliminated (coarsest level only).
    fn assemble(&self, cx: &mut ComputeCtx, ctx: &JobCtx) -> ComputeResult<SymCsr>;
}
#[derive(Clone, Copy, Debug)]
pub struct GmgOpts {
    pub max_coarse_dofs: u64,                         // 50_000
    pub pre_smooth: u32, pub post_smooth: u32,        // 2, 2
    pub smoother: ChebyshevOpts,
}
/// V-cycle preconditioner in precision `R`; coarsest level solved by CMP-03 Cholesky (f64, CPU).
pub struct Gmg<R: GpuReal = f32> { /* levels, smoothers, transfer kernels, coarse Cholesky */ }
impl<R: GpuReal> Gmg<R> {
    pub fn build(cx: &mut ComputeCtx, prim: &Primitives, fine: std::sync::Arc<dyn GridLevelOperator<R>>,
                 opts: &GmgOpts, ctx: &JobCtx) -> ComputeResult<Gmg>;
    pub fn levels(&self) -> usize;
    /// Re-coarsens after the fine coefficients changed (each SIMP iteration); keeps buffers.
    pub fn update(&mut self, cx: &mut ComputeCtx, ctx: &JobCtx) -> ComputeResult<()>;
}
impl<R: GpuReal> GpuPreconditioner<R> for Gmg<R> {}
/// Reference operator: variable-coefficient Q1 Poisson (per-cell coefficient), 27-point.
pub struct Q1Poisson<R: GpuReal = f32> { /* nodes, coeff: GpuVec<R>, fixed */ }
impl<R: GpuReal> Q1Poisson<R> {
    pub fn new(gpu: &Gpu, nodes: [u32; 3], coeff: &DenseGrid<f32>, fixed: &DenseGrid<u8>) -> ComputeResult<Self>;
}
impl<R: GpuReal> GridLevelOperator<R> for Q1Poisson<R> {}

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
pub fn lobpcg<R: GpuReal>(cx: &mut ComputeCtx, prim: &Primitives, k: &dyn GpuOperator<R>, m: &dyn GpuOperator<R>,
                          precond: &dyn GpuPreconditioner<R>, count: u32, opts: &LobpcgOpts,
                          ctx: &JobCtx) -> ComputeResult<EigenResult>;
#[derive(Clone, Copy, Debug)]
pub struct LobpcgOpts { pub tol: f64 /* 1e-6 */, pub max_iter: u32 /* 500 */, pub seed: u64 }
```

**Expected from other specs** (owned there; listed so the dependency is explicit):
- **S00 / lane E (`amg-jobs::gpu`, FND-09, Done):** no change. `Gpu::compute_pipeline`,
  `upload`, `storage`, `readback`, `has`, `GpuError` are used as they are. `GpuConfig` already
  requests `SHADER_F64` as an optional feature, so a Vulkan device gets it without changes. CI
  request (optional, 2026-11-09): a Vulkan software adapter job for the f64 path (Testing).
- **S08:** wraps `BrickGrid<f16>` / `GpuBrickGrid` in `SdfBricks` and `SampledGrid`; S08's
  per-tile classification maps to `set_tile_fill` (Fill0 = outside, Fill1 = inside). Its
  `amg-implicit` Cargo.toml gains `amg-compute` (allowed by the layering test).
- **S13:** uses `SymCsr` (builds the upper pattern itself, then `from_raw`), `SymbolicFactor`,
  `Cholesky`, `Ldlt`, `FactorEstimate`, `PivotReport`, `lanczos` with its own `ShiftInvert`,
  `GpuCsr`, `pcg_gpu`, `Chebyshev`, `refine` (or `solve_gpu`), and implements `GpuPreconditioner`
  for its p-multigrid, generic over `R: GpuReal` so it runs on both precision paths.
- **S14:** implements `GridLevelOperator<R>` for H8 elasticity (density-coarsened), generic over
  `R: GpuReal`, and uses `Gmg`, `pcg_gpu` (or `solve_gpu` without `cpu_a`), `DenseGrid`.
- **S16:** `Primitives::{sort, scan, segmented_reduce, reduce}`, `AtomicCounter` (AM-13).

## Work packages
WPs are ≈ 1–3 days each. CMP-01..06 keep their IDs and meaning; CMP-07..11 are new. Times are
on the reference machine (note §1: Ryzen 7 5700X3D, 32 GB, RTX 5060 Ti 16 GB), release build,
GPU times from timestamps excluding upload and readback. "WARP" = the CI software adapter.
"F32 path" / "F64 path" = the two precision paths (Design, note §2.1). "f64 gate" = the
reference GPU on Vulkan with `AMG_REQUIRE_F64=1` (Testing), the only place the F64 path runs
until CI has a Vulkan adapter. Budget bars apply to the F32 path on the default (DX12) device,
which is what the default Windows build runs; F64-path numbers are recorded beside them.
Benchmarks are criterion benches under `crates/amg-compute/benches/` registered with the FND-08
harness; measured numbers go into the Refinement log.

| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| CMP-07 | GPU layer: `GpuVec<T>` (binding-limit checks), `Kernel` (incl. `with_precision`), `ComputeCtx` (batched encoder, uniform arena, `dispatch_1d` 2-D folding, timestamps), `prelude_wgsl` (`real` alias), `ComputeError` + `Diagnostic`; precision selection (`Precision`, `PrecisionPolicy`, `AMG_COMPUTE_PRECISION`, `GpuReal`, `device_precision`, `SolverDevice::precision`); WARP test harness with a per-precision test matrix and the f64 skip/require rule; crate deps (`half`, `bytemuck`, `rayon`) and THIRD_PARTY rows | FND-09, FND-05 | P1 | Done | 1. On WARP and the reference GPU, `dispatch_1d` with n ∈ {0, 1, 255, 256, 257, 65 535·256 + 1, 2²⁶ + 3} writes every index exactly once (counter test). 2. `GpuVec::zeroed` past the binding limit returns `TooLarge`, no panic. 3. A destroyed device makes `finish` return `Gpu(DeviceLost)` within 1 s (no hang). 4. Bench: 1 000 empty dispatches recorded and submitted in one `ComputeCtx` cost ≤ 20 µs CPU each. 5. `take_timings` is non-empty on the reference GPU and empty without `TIMESTAMP_QUERY`. 6. `cargo deny` passes; `half` listed in THIRD_PARTY.md. 7. `PrecisionPolicy::resolve`: `Auto` gives F64 exactly when `Gpu::has(SHADER_F64)` (WARP and the reference GPU on DX12 → F32; the reference GPU on Vulkan → F64); `ForceF32` gives F32 on every device; `RequireF64` on WARP returns `Unsupported`, no panic; `parse` accepts `auto`/`f32`/`f64` (any case, empty = `Auto`) and rejects anything else with `Config`. 8. The harness runs each numeric GPU test once per precision. On a device without `SHADER_F64` the f64 case is reported as skipped with the reason (`f64: adapter has no SHADER_F64`), and fails instead when `AMG_REQUIRE_F64=1`. CI (WARP) runs the f32 cases and reports the f64 skips. 9. f64 gate (Testing): `AMG_WGPU_BACKEND=vulkan AMG_REQUIRE_F64=1 cargo nextest run -p amg-compute` on the reference GPU passes, including an f64 kernel (`real` = f64) for test 1 and a bit-exact f64 upload → kernel copy → readback; the run is recorded in the Refinement log before Review. |
| CMP-01 | GPU primitives: reduce (sum/min/max on f32/f64/u32/i32), dot and segmented reduce (f32/f64), exclusive/inclusive scan, compaction, stable radix sort (keys + optional values, `key_bits`), `AtomicCounter`, `cpu` references; benchmarks vs CPU | CMP-07 | P1 | Done | 1. Every primitive equals its `cpu` reference on sizes {0, 1, 255, 256, 257, 65 537, 2²⁰ + 7, 2²⁴} with random data, on WARP and the reference GPU: integers exactly, f32 sums bit-equal to the tree-order reference on the reference GPU and within n·ε₃₂·Σ\|x\| on WARP. The f64 variants (reduce, dot, segmented reduce) pass the same sizes on the f64 gate, bit-equal to the f64 tree-order reference; on WARP they are skipped by the CMP-07 rule, and an f64 call there returns `Unsupported`. 2. Same input → bit-identical output over 3 runs (both precisions). 3. Sort is stable (equal keys keep value order; property test vs `slice::sort_by_key`). 4. Benches (reference GPU, 2²⁶ elements): reduce f32 ≥ 60 % of the measured device-to-device copy bandwidth; scan u32 ≥ 30 %; sort of 2²⁴ key-value pairs ≥ 1 G pairs/s and ≥ 3× rayon `par_sort_by_key` on the CPU; f64 reduce and dot on Vulkan recorded (no bar); numbers recorded. |
| CMP-02 | Grids: `DenseGrid<T>`, `BrickGrid<T>` (two-level tree, `BrickLayout::{CELLS, CORNERS}`, fills, aprons, Morton `canonicalize`, `content_hash`), `GpuBrickGrid` (upload/download, deterministic scan-based `allocate`), `brick_grid_wgsl` | CMP-01 | P1 | Todo | 1. Sphere r = 50 at cell 0.5 baked as `d / cell` into `BrickGrid<f16>` (`CORNERS`, band ±4 cells, inside tiles Fill1): CPU `trilinear` error vs the analytic distance ≤ 0.01 cell inside the band; sign correct everywhere in the box (fills). 2. A GPU kernel using `bg_trilinear` at 1 M random points equals CPU `trilinear` within 1e-3 cell (WARP and reference GPU). 3. GPU `allocate` from a mask gives buffers bit-identical over 3 runs and the same brick order as CPU `canonicalize`; upload → download round-trips bit-exactly for f16, f32, u8. 4. Grids built in two insertion orders have equal `content_hash` after `canonicalize`. 5. Sphere r = 100 at cell 0.1: index (root + tiles) ≤ 10 % of brick bytes; bytes recorded. 6. Bench: ≥ 0.5 G `bg_trilinear` samples/s at random in-band points on the reference GPU. 7. A box needing > 2²⁴ root tiles returns `Shape`. |
| CMP-03 | Sparse direct: `SymCsr` (upper CSR, validation, deterministic `spmv`, `add_scaled`), `TripletBuilder`, `SymbolicFactor` (reusable, `estimate`), `Cholesky` and `Ldlt` (faer, LBLᵀ) with multi-RHS, `PivotReport`, `Inertia`; `testprob` generators | FND-03, FND-04 | P1 | Done | (1–3 narrowed by the owner, 2026-11-11: measurements recorded, no bar; the 20 s / 1 M targets move to CMP-10 + FEM-08, see Refinement log.) 1. `q1_elasticity(55, Clamped)` (499 125 DOF): analyse + factor + solve time recorded for AMD and for a geometric nested-dissection ordering, relative residual ≤ 1e-10. 2. `q1_elasticity(70, Clamped)` (1 029 000 DOF): `SymbolicFactor::estimate()` recorded from the analysis alone (AMD and ND); factored only where the estimate fits in RAM. 3. `solve_many` with 6 RHS vs one `solve_in_place`: ratio recorded. 4. `q1_elasticity(_, Free)` gives `NotPositiveDefinite` with a row, or a `PivotReport` with `below_threshold`; never a panic. 5. `Ldlt::inertia` on random diagonal-plus-low-rank matrices equals the dense eigenvalue sign count (proptest). 6. Re-factoring with new values on the same pattern skips analysis (counter). 7. Factor and solve are bit-identical over 2 runs; `spmv` bit-identical across 1, 4, 16 threads. 8. `from_raw` rejects unsorted, lower or out-of-range input with `InvalidMatrix`. |
| CMP-04 | Eigen: thick-restart shift-invert Lanczos (`lanczos`) with full B-reorthogonalisation, `EigenMode::{Vibration, Buckling}`, `Which::{Nearest, Band}` with Sturm check and re-shift, `ShiftInvert` callback, `CholeskyShift`, `LdltShift` | CMP-03 | P1 | Todo | 1. 50 random sparse SPD pencils (n ≤ 3 000): the 10 eigenvalues nearest σ match faer's dense generalised solver within 1e-10 relative; ‖ΦᵀBΦ − I‖∞ ≤ 1e-10. 2. Free-free `q1_elasticity(20, Free)` with σ < 0: exactly 6 values with \|λ\| ≤ 1e-8·λ₇, then the next 4 within 1e-9 of dense. 3. Band mode finds every eigenvalue in [lo, hi] on 20 pencils (vs dense), `sturm_checked` true; a band with more than `max` values returns the `max` lowest and says so. 4. Buckling mode on a pencil with known eigenvalues (indefinite K_G) within 1e-9. 5. Bench: `q1_elasticity(55, Clamped)`, 8 modes, including the factorisation, ≤ 40 s (FEM-07 budget 60 s). 6. Cancel during iterations returns `Cancelled` within 1 s after the factorisation ends. |
| CMP-08 | Iterative core: `LinearOperator`, `CpuPreconditioner`, `GpuOperator<R>`, `GpuPreconditioner<R>`; `GpuCsr<R>` (full storage, `update_values`); `Jacobi`, `BlockJacobi3`, `Chebyshev` (Lanczos bounds, `smooth`), all generic over f32/f64; `pcg_cpu`, `pcg_gpu<R>` (async convergence checks) on a crate-private `VectorBackend`; `refine`; `GpuSystem` + `solve_gpu` (precision-path selection) | CMP-01, CMP-03 | P1 | Todo | 1. `pcg_gpu` and `pcg_cpu` on `q1_poisson(65)` and `q1_elasticity(40)` reach 1e-5 relative residual with iteration counts within 2 % of each other: f32 on WARP and the reference GPU (DX12), f64 on the f64 gate. 2. `solve_gpu` with `BlockJacobi3` on `q1_elasticity(70)` reaches ‖r‖/‖b‖ ≤ 1e-8 (f64 CPU residual) on both paths, and both solutions match CMP-03 Cholesky within 1e-7 in relative energy norm: the F32 path on the reference GPU (DX12) and the F64 path on the f64 gate, where it needs exactly one outer step; on the same Vulkan device with `ForceF32` the F32 path meets the same bounds (both paths on one device). 3. Bench: `q1_elasticity(70)` with `BlockJacobi3` ≥ 250 PCG iterations/s on the F32 path on the default (DX12) reference device (the budget bar); GPU idle between submissions ≤ 10 % (timestamps). F64-path iterations/s and total `solve_gpu` time on Vulkan recorded next to the F32 numbers (no bar; input to open question 8). 4. Iteration counts and results bit-identical over 3 runs, on each path. 5. `Chebyshev` λmax estimate within 5 % of the dense value on a 3 000-DOF problem (both precisions). 6. Cancel returns `Cancelled` within 100 ms; device loss returns an error. 7. A `GpuSystem` built from `PrecisionPolicy::Auto` takes the F32 path on WARP and the F64 path on the f64 gate (`GpuSolveStats::precision`); building an f64 `GpuCsr` on WARP returns `Unsupported`. |
| CMP-05 | Geometric multigrid for structured grids: `GridLevelOperator<R>`, `Gmg<R>` (Chebyshev–Jacobi smoothing, trilinear transfer, re-discretised coarse levels, padding to `2^k·m + 1`, fixed masks, coarse CPU Cholesky, `update`), `Q1Poisson<R>` reference operator, all generic over f32/f64; MGPCG | CMP-08, CMP-02 | P1 | Todo | 1. `Q1Poisson` with a `simp_field` coefficient (contrast 1e3), clamped on one face: MGPCG reaches 1e-6 relative residual in ≤ 25 iterations at 129³ nodes, and the 257³ count is ≤ 1.2× the 129³ count (h-independence); on the F32 path (reference GPU, DX12) and the F64 path (f64 gate). 2. Bench: 257³ nodes (16 974 593 DOF) solve < 2 s on the reference GPU on the F32 path (default DX12 device; headroom for GEN-03's 1 M-voxel elasticity < 2 s). F64-path time on Vulkan recorded (no bar; input to open question 8). 3. Contrast 1e6: iteration counts recorded for both paths (no limit). 4. Non-`2^k·m + 1` dims (e.g. 100³) converge within 1.3× the padded power-of-two case. 5. `update` after a coefficient change reuses all buffers (allocation counter) and gives the same iterations as a fresh `build`. 6. Matches `pcg_cpu` + Cholesky reference within 1e-5 relative on 33³, on both paths (f32 also on WARP). |
| CMP-06 | Optional CUDA backend (`cudarc`, NVRTC at runtime) behind feature `cuda`: `VectorBackend` impl, CSR SpMV, Chebyshev, `SolverDevice::Cuda`, f32 and f64 (precision chosen by `SolverDevice::precision`); Phase 5, started only if the wgpu solver benchmarks miss their budgets (QUESTIONS #80 (b), ADR-004) | CMP-05, CMP-08 | P2 | Todo | Gate: scheduled only if CMP-08 bench 3 or CMP-05 bench 2, or the FEM-08 / GEN-03 budget benches built on them, miss their budgets on wgpu; otherwise it stays Todo. 1. Builds and tests pass with and without `cuda`; without a CUDA driver `SolverDevice::Cuda` creation returns an error, not a crash. 2. CMP-08 test 1 and CMP-05 test 1 pass on CUDA in f32 and f64. 3. Bench table CUDA vs wgpu for CMP-08 bench 3 and CMP-05 bench 2, per precision path, recorded; CUDA becomes the default only if ≥ 1.5× faster than wgpu on the same path (owner decides). |
| CMP-09 | LOBPCG (`lobpcg<R>`): block, preconditioned, GPU operators (f32 or f64), f64 Rayleigh–Ritz on the CPU, soft locking | CMP-08, CMP-04 | P2 | Todo | 1. On `q1_elasticity(40, Clamped)`, 8 smallest eigenvalues within 1e-6 relative of CMP-04 Lanczos, on the F32 path (reference GPU, DX12) and the F64 path (f64 gate). 2. `q1_elasticity(88, Clamped)` (2 044 416 DOF), 8 modes with `BlockJacobi3`, < 120 s on the reference GPU on the F32 path; iterations recorded for both paths. 3. Deterministic over 2 runs. |
| CMP-10 | Nested-dissection ordering: geometric ND from node coordinates (`Ordering::NestedDissection`), or METIS behind `metis` if geometric ND is not enough | CMP-03 | P2 | Todo | 1. On `q1_elasticity(55)` and the S13 500 k DOF bracket, factor nnz ≥ 25 % lower than AMD. 2. Becomes the default only if factor time drops ≥ 20 %; numbers recorded either way. Only scheduled if CMP-03 test 1 misses its budget or S13 FEM-04 bench 5 does (CMP-03 missed it; owner, 2026-11-11: CMP-10 goes ahead and takes over the direct-solve targets that fit on 32 GB, with S13 500 k+ cases routed to the GPU iterative solver, FEM-08). |
| CMP-11 | Smoothed-aggregation AMG preconditioner: CPU setup (strength of connection, aggregation, rigid-body near-null space for 3-DOF nodes), GPU V-cycle with `Chebyshev` smoothing, `GpuPreconditioner<R>` (f32 and f64) | CMP-08 | P2 | Todo | 1. Unstructured test (`q1_elasticity` with randomly perturbed nodes and a random symmetric renumbering): iterations to 1e-6 ≤ 2× `Gmg` on the unperturbed grid of equal size, on both precision paths. 2. Setup for 1 M DOF < 5 s CPU. 3. Iterations grow ≤ 1.5× from 125 k to 1 M DOF. |

Status values: `Todo`, `Refining`, `Ready`, `In progress (agent/branch)`, `Review`, `Done`, `Blocked (reason)`.

## Testing
- Every GPU routine has a CPU reference in the crate, and every GPU test runs on WARP in CI
  (FND-05) and on the reference GPU locally. Timing assertions run only on the reference GPU
  (`AMG_BENCH_REFERENCE=1`); WARP checks correctness only.
- **Both precision paths** (owner policy, QUESTIONS #81). Numeric GPU tests are written once,
  generic over `R: GpuReal`, and the CMP-07 harness runs them per precision:
  - **F32 path:** CI on WARP (DX12) and the reference GPU on DX12 (the default device). This
    path is always exercised by CI.
  - **F64 path:** needs a device with `SHADER_F64`, which in wgpu 30 means Vulkan. CI's Windows
    runners have only WARP (DX12), so the F64 path is tested by the **f64 gate**: on the
    reference machine, `AMG_WGPU_BACKEND=vulkan AMG_REQUIRE_F64=1 cargo nextest run -p
    amg-compute`, run by the implementing agent before setting a WP touching numeric kernels to
    `Review`, with the result recorded in the Refinement log. The same Vulkan device also runs
    the F32 path with `AMG_COMPUTE_PRECISION=f32`, so both paths are compared on one device.
  - **Explicit skip:** on a device without `SHADER_F64` every f64 case is reported as skipped
    with its reason (never silently passed), and fails instead when `AMG_REQUIRE_F64=1`.
  - **CI later:** a Mesa lavapipe (software Vulkan) job, installed the way `install-warp` is,
    would run the F64 path in CI if lavapipe reports `shaderFloat64` on the runner (to verify;
    lane E request, open question 9). When it exists it sets `AMG_REQUIRE_F64=1`.
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
| Direct solve, 499 k DOF 3-D elasticity (CMP-03 measured; target moved to CMP-10 + FEM-08, owner 2026-11-11) | ≤ 20 s analyse + factor + solve (CMP-03: AMD 413 s under load, ND 42.6 s idle) | vision FEA 500 k DOF < 30 s (S13 note §8.1 split) |
| Modal, 499 k DOF, 8 modes (CMP-04) | ≤ 40 s | FEM-07 < 60 s |
| GPU PCG throughput, 1.03 M DOF elasticity (CMP-08) | ≥ 250 iterations/s | FEM-08 2 M DOF < 60 s |
| MGPCG, 17 M DOF scalar Q1 (CMP-05) | < 2 s, ≤ 25 iterations at 129³ | GEN-03 1 M voxels < 2 s; vision topopt 1 M voxels × 100 iterations < 5 min |
| Primitives (CMP-01) | reduce ≥ 60 %, scan ≥ 30 % of copy bandwidth; sort ≥ 1 G pairs/s | AM-13, IMP-02 |
| Brick lookups (CMP-02) | ≥ 0.5 G trilinear samples/s | IMP-02, IMP-05 |
| UI thread | never blocks: all waits are on job threads (`ComputeCtx::finish`, `Readback::wait`) | vision 16 ms |

GPU solver bars (CMP-08, CMP-05, CMP-09) apply to the F32 path on the default DX12 device; the
F64 path's numbers (Vulkan) are recorded beside them without a bar until open question 8 is
answered.

## Open questions
1. ~~(from S10, 2026-10-20) `SHADER_F64` is Vulkan-only in wgpu 30, so the default DX12 device never has it; plan f32 or emulated-double kernels.~~
   ~~**Resolved 2026-11-08:** f32 kernels only, accuracy through f64 iterative refinement on the
   CPU; no emulated double (WGSL `fma` is not guaranteed fused on DX12, FXC may reorder, and the
   RTX 5060 Ti runs FP64 at 1/64 rate anyway). Note §2.1.~~
   **Re-resolved 2026-11-09 (owner, QUESTIONS #81, the f32-only proposal not accepted):** f64
   kernels where the device has `SHADER_F64`, f32 kernels with f64 CPU iterative refinement as
   the fallback, chosen at runtime; both paths tested. Still no emulated double. Design, note
   §2.1, ADR-004 note.
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
6. ~~CUDA timing: ADR-004's amendment calls CUDA the first optimisation once wgpu solvers are
   benchmarked, but the roadmap keeps CMP-06 at P3 in Phase 6. (QUESTIONS #80.)~~
   **Resolved 2026-11-09 (owner, QUESTIONS #80 (b)):** CMP-06 moves to Phase 5 at P2 and is
   started only if the wgpu solver benchmarks miss their budgets; CUDA becomes the default only
   if ≥ 1.5× faster. CMP-06 row updated.
7. ~~Accept the no-f64-on-GPU policy (note §2.1) as an ADR-004 note? (QUESTIONS #81.)~~
   **Resolved 2026-11-09 (owner, QUESTIONS #81):** not accepted; the ADR-004 note states the
   owner's policy instead (open question 1).
8. ~~(2026-11-09, QUESTIONS #103) **Default backend and the f64 path.** In
   wgpu 30 only Vulkan reports `SHADER_F64`, and the Windows default device is DX12 (FND-09), so
   with default settings no Windows user ever gets the F64 path. Options: (a) keep DX12 as the
   default; the F64 path runs only with `AMG_WGPU_BACKEND=vulkan` (or a later setting), and the
   budgets bind on the F32 path; (b) make Vulkan the Windows default (an S00/S10 decision that
   also moves rendering); (c) a second, compute-only Vulkan `Gpu` for solvers when the render
   device is DX12 (needs an ADR-004 change: "one device"). Related: the RTX 5060 Ti runs FP64 at
   1/64 of FP32, so compute-bound kernels (matrix-free H8, Chebyshev) may miss GEN-03 / FEM-08
   budgets on the F64 path, while bandwidth-bound CSR SpMV should be about 1.5× slower per
   iteration. Should `Auto` stay "F64 whenever available" for every solver, or may a solver
   pick F32 + refinement on F64 devices when its F64 bench misses budget? *Recommended:* (a) now,
   and decide the second part from the CMP-08 / CMP-05 bench tables (both paths recorded).
   Refinement log 2026-11-09.~~
   **Resolved 2026-11-10 (owner, QUESTIONS #103 (a)):** DX12 stays the Windows default; the F64
   path runs only with `AMG_WGPU_BACKEND=vulkan`, and the budgets bind on the F32 path. Still
   open, decided from the CMP-08 / CMP-05 bench tables: whether a solver may pick F32 +
   refinement on an F64 device when its F64 bench misses budget.
9. (2026-11-09, request to lane E / S00) Optional CI job with a Vulkan software adapter (Mesa
   lavapipe) so the F64 path runs in CI; until then the f64 gate is local (Testing).

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

### 2026-11-09 - owner answers QUESTIONS #80, #81 (lane F refinement)
- **Read:** QUESTIONS #80 and #81 (owner answers), ADR-004, this spec, note §1–2 and §7–8, S00
  FND-09 (log: `SHADER_F64` absent on DX12 with the RTX 5060 Ti), `crates/amg-jobs/src/gpu`
  (`GpuConfig::optional_features` = `SHADER_F64 | TIMESTAMP_QUERY`; `backend.rs`: DX12 default
  on Windows, `AMG_WGPU_BACKEND=dx12|vulkan|warp`), `.github/workflows/ci.yml` (all jobs on
  `windows-2025`, GPU tests on WARP with `AMG_REQUIRE_GPU=1`), and the wgpu 30.0.1 / naga 30.0.1
  sources in the cargo registry.
- **#81, owner did not accept f32-only.** Policy: f64 kernels where the device has
  `SHADER_F64`, f32 + f64 CPU iterative refinement as the fallback; both paths tested. Applied:
  - Interfaces: new `amg_compute::precision` (`Precision`, `PrecisionPolicy` with `Auto` /
    `ForceF32` / `RequireF64`, `AMG_COMPUTE_PRECISION`, `GpuReal` for f32/f64,
    `device_precision`), `SolverDevice::precision`, `Kernel::with_precision`, `prelude_wgsl`
    (replaces `PRELUDE_WGSL`; binds `alias real`), `ComputeError::{Unsupported, Config}`. Numeric
    GPU types are generic over `R: GpuReal` with default `f32` (`GpuOperator<R>`,
    `GpuPreconditioner<R>`, `GpuCsr<R>`, `Jacobi<R>`, `BlockJacobi3<R>`, `Chebyshev<R>`,
    `pcg_gpu<R>`, `GridLevelOperator<R>`, `Gmg<R>`, `Q1Poisson<R>`, `lobpcg<R>`); `dot`,
    `segmented_reduce` and the reductions accept f64. New `GpuSystem` + `solve_gpu`: the one
    place that picks the path (F64: PCG to the final tolerance, normally one outer step; F32:
    `refine` with a loose inner f32 PCG).
  - **Generic, not dual, kernels.** WGSL has no generics, so each numeric kernel is one source
    over `alias real`, and the prelude binds it to f32 or f64. The pipeline cache keys on the
    full source, so both variants coexist. Checked in naga 30: f64 needs no `enable` directive
    in WGSL (only the `FLOAT64` capability, which wgpu grants with `SHADER_F64`); f64 literals
    use the `lf` suffix, so kernels write constants as `real(…)` conversions. Atomics stay
    u32/i32 (no float atomics on either path).
  - **How the fallback is chosen:** at runtime, per device, from `Gpu::has(SHADER_F64)` (the
    feature is already requested as optional by FND-09's `GpuConfig`); never a Cargo feature.
    `ForceF32` lets one f64-capable device run both paths.
  - Grids, scan, compaction and sort are integer or storage formats and are unchanged; S08 / S16
    kernels may use `GpuReal` / `Precision` for their own numerics.
  - WP Done-when: CMP-07 (precision tests 7–9, skip/require rule), CMP-01 (f64 reductions),
    CMP-08 (tests on both paths, `solve_gpu` selection test 7), CMP-05, CMP-09, CMP-11 (both
    paths). Budget bars stay on the F32 path on the default device; F64 numbers recorded.
    CMP-02, CMP-03, CMP-04, CMP-10 unchanged (CPU or storage only). No new WP: the precision
    layer fits CMP-07 and the generic kernels are written once inside the existing WPs.
  - **Testing the f64 path:** CI has only WARP (DX12), which never has `SHADER_F64`. So: the
    "f64 gate" on the reference machine (Vulkan, `AMG_REQUIRE_F64=1`), recorded in this log
    before Review; on every other device the f64 cases are skipped with a reason. A lavapipe
    CI job would close the gap (open question 9; lavapipe's `shaderFloat64` on the runner not
    verified here).
- **Does the default backend matter? Yes.** wgpu-hal 30.0.1 sets `SHADER_F64` only in
  `vulkan/adapter.rs` (from `VkPhysicalDeviceFeatures::shaderFloat64`). The DX12 HAL never
  reports it (although D3D12 hardware has `DoublePrecisionFloatShaderOps` and SM 5.1 has
  `double`), and `wgpu-types` documents `SHADER_F64` as "Supported Platforms: Vulkan". Metal never
  reports it: MSL has no double type and naga's MSL backend rejects f64. So the owner's
  "Vulkan/Metal" is in practice Vulkan only; macOS would always run the F32 path. On Windows the
  default device is DX12 (`BackendSelection::platform_default`), so with default settings every
  Windows user runs the F32 path, and the F64 path needs `AMG_WGPU_BACKEND=vulkan`. The
  backend default therefore decides which path users get; S15's code is the same either way
  (precision is per `Gpu`). Options and the FP64-rate concern (RTX 5060 Ti: FP64 at 1/64 of
  ~23.7 TFLOPS FP32 ≈ 0.37 TFLOPS; 448 GB/s; f64 CSR SpMV at ~0.17 flop/byte stays
  bandwidth-bound, about 1.5× slower per iteration than f32 (12 vs 8 bytes per non-zero), while
  compute-bound matrix-free kernels may be much slower) are open question 8 for the owner. Not
  changed here: the DX12 default is S00/S10's, and the kernels must stay FXC-safe for the
  fallback anyway. If a later wgpu reports `SHADER_F64` on DX12, `Auto` picks it up with no
  code change.
- **#80 (b):** CMP-06 moves to Phase 5 at P2, gated on the wgpu solver benchmarks missing their
  budgets (CMP-08 bench 3, CMP-05 bench 2, FEM-08 / GEN-03 budget benches); default only if
  ≥ 1.5× faster on the same precision path. CMP-06 also covers f64 (CUDA devices always have
  it). Note §8 updated.
- **Roadmap (for the coordinator; this refinement does not edit it):** CMP-06 from Phase 6 to
  Phase 5 lane F, with the gate above.
- **Requests to other specs:** S13 (FEM-08 wording "f32 PCG", "wgpu DX12 has no f64"; the
  p-multigrid should be generic over `R: GpuReal`), S14 (GEN-03 "f32" H8 operator; generic over
  `R`), lane E (lavapipe CI job). Listed for the coordinator; not edited here.
- Status stays `Refined`.

### 2026-11-09 - CMP-07 GPU layer and precision selection (lane F implementation)
**Landed** in `crates/amg-compute`: `ComputeError` + `Diagnostic`, `SolverDevice`,
`precision` (`Precision`, `PrecisionPolicy`, `PRECISION_ENV`, `GpuReal`, `device_precision`),
`gpu` (`GpuVec`, `Kernel`, `ComputeCtx`, `UniformSlot`, `prelude_wgsl`); the per-precision harness
in `tests/common` (`gpu_test_per_precision!`, the f64 skip/require rule); `tests/gpu_layer.rs`;
`benches/gpu_layer.rs`. Deps: `half` 2.7.1 (MIT/Apache-2.0, new THIRD_PARTY row), `bytemuck`,
`rayon`, `thiserror`.

**Measured on the reference machine** (RTX 5060 Ti, driver 32.0.16.1714 / 617.14; WARP is
Microsoft Basic Render Driver 10.0.26100.9711). All runs `--no-capture`, `AMG_REQUIRE_GPU=1`,
`AMG_STRICT_TIMING=1`.

| Done-when | DX12 (reference) | WARP | Vulkan (reference) |
|---|---|---|---|
| 1. `dispatch_1d`, n up to 2²⁶ + 3, exactly once | pass, F32; largest n 745 ms | pass, F32; 1.50 s | pass, F32 411 ms and F64 610 ms |
| 2. `zeroed` past the limit (2 147 483 644 bytes) | `TooLarge` | `TooLarge` | `TooLarge` |
| 3. `finish` after `destroy` | `DeviceLost` in 295 µs | 307 µs | 150 µs |
| 5. `take_timings` | 3 entries (1.2–3.0 µs) | 3 entries | 3 entries (1.9 µs) |
| 7. `Auto` resolves to | F32 | F32 | F64 |
| 8. f64 cases | skipped, `f64: adapter has no SHADER_F64` | skipped, same reason | run |

- Done-when 4 (release, DX12, timestamps on): **14.4 µs CPU per empty dispatch** recorded and
  submitted (budget 20 µs; 655 batches of 1 000; criterion 14.5–16.3 ms per 1 000).
- Done-when 5, without `TIMESTAMP_QUERY`: empty, on every device.
- Done-when 6: `cargo deny check` and `cargo deny --all-features check` pass.
- Done-when 7: `RequireF64` on DX12 and WARP returns `Unsupported`, no panic; `parse` unit tests.
- Done-when 8: under `AMG_REQUIRE_F64=1` an f64 case that cannot run fails (`f64_skip_and_require_rule`).
- Done-when 9, f64 gate: `AMG_WGPU_BACKEND=vulkan AMG_REQUIRE_F64=1 cargo nextest run -p
  amg-compute` (also with `AMG_REQUIRE_GPU=1 AMG_STRICT_TIMING=1 AMG_BENCH_REFERENCE=1`):
  ```
  adapter: NVIDIA GeForce RTX 5060 Ti (Vulkan, DiscreteGpu, driver NVIDIA 617.14), SHADER_F64: true, TIMESTAMP_QUERY: true
  n = 16776961: every index written exactly once (F64, 127.8645ms on NVIDIA GeForce RTX 5060 Ti)
  n = 67108867: every index written exactly once (F64, 610.0639ms on NVIDIA GeForce RTX 5060 Ti)
  bit-exact: 1048576 of 1048576 F64 values
  Auto resolved to F64
  Summary [  15.473s] 40 tests run: 40 passed, 0 skipped
  ```
  The same device with `AMG_COMPUTE_PRECISION=f32`: 40 passed; both f64 cases report
  `SKIP: f64: AMG_COMPUTE_PRECISION=f32`.

**Decisions and deviations:**
- Each dispatch is its own compute pass, so timestamps work with plain `TIMESTAMP_QUERY`.
- A batch is submitted automatically at 1 024 dispatches (two timestamps each must fit one query
  set). An automatic submission keeps uniform slots alive; only `submit` releases them.
- `take_timings` keeps the last 256 batches, so a solver that never calls it stays bounded. It
  waits for the timestamps, so it is job-thread only.
- `AMG_COMPUTE_PRECISION=f32` maps to `ForceF32` and `f64` to `RequireF64`. In the harness `f32`
  skips the f64 cases and `f64` requires them, like `AMG_REQUIRE_F64=1`.
- `amg-compute` depends on `amg-jobs` by path with `default-features = false`, so the `gpu`
  feature is real; `--no-default-features` builds.
- `SolverDevice::Cuda` and the `cuda` feature are not added; they arrive with CMP-06.

**Leftovers:**
- f64 skips are printed to stderr, and nextest hides the output of passing tests, so CI needs
  output capture to show them (Interface changes below).
- `GpuVec::read` panics on an empty vector because `Readback` has no zero-length form. CMP-01's
  size-0 cases must not read back, or need the additive `amg-jobs` change below.
- GPU out-of-memory below the binding limit still reaches wgpu's error handler (a panic):
  `ComputeError` has no variant for it. Only the binding limit is checked (Done-when 2).

### 2026-11-10 - CMP-01 GPU primitives (lane F implementation)
Branch `engels-hub/wp-CMP-01-gpu-primitives`; status → `Review`. New module `amg_compute::prim`
(`gpu` feature for the GPU parts, `prim::cpu` always available).

**What landed:**
- `Primitives` (kernel cache + methods on a `ComputeCtx`): `reduce` (sum/min/max on f32/f64/u32/i32),
  `dot`, `segmented_reduce` (f32/f64), `scan` (exclusive/inclusive, optional total), `compact`,
  `sort` (stable LSD radix, keys + optional values, `key_bits` 1..=32), `AtomicCounter`;
  `prim::cpu` has a bit-exact reference of each.
- Determinism: reductions are a fixed tree (256 threads × 8 elements per tile, halving strides,
  one pass per level), mirrored by `cpu::reduce`; no float atomics. `dot` multiplies in its own
  kernel (products rounded to `real`) and then reduces, so there is no FMA contraction and the
  CPU reference is exact. Float min/max use identities ±finite MAX; NaN behaviour is unspecified.
- Scan is reduce-then-scan with recursion on the tile totals; compaction is a flags scan plus a
  scatter; sort is 5-bit-digit LSD passes (7 for 32 bits): per-tile histogram, scan of the
  digit-major histogram, in-tile stable sort (4-way splits of two digit bits on packed
  `digit << 11 | index` words) and a scatter to `offset + rank`.

**Results (reference GPU RTX 5060 Ti, 2²⁶ elements; DX12 unless noted, `AMG_STRICT_TIMING=1` passes):**

| Measure | Result | Bar |
|---|---|---|
| copy (read + write) | 1.446 ms, 371 GB/s | - |
| reduce f32 sum | 0.917 ms, 293 GB/s = 79 % of copy | ≥ 60 % |
| scan u32 exclusive | 2.547 ms, 211 GB/s = 57 % of copy | ≥ 30 % |
| sort 2²⁴ pairs | 13.4 ms = 1.25 G pairs/s (Vulkan 13.4 ms, 1.25 G pairs/s) | ≥ 1 G pairs/s |
| rayon `par_sort_by_key` 2²⁴ pairs | 160 ms (0.10 G pairs/s): GPU 12.0× | ≥ 3× |
| f64 reduce 2²⁵ (Vulkan) | 0.930 ms, 289 GB/s | recorded |
| f64 dot 2²⁵ (Vulkan) | 12.7 ms | recorded |
| f32 dot 2²⁵ (Vulkan) | 7.7 ms | - |

Bandwidth counts the ideal bytes (copy and scan 2 N, reduce N); the copy of the unsorted input
is subtracted from the sort time. Sort tuning history (2²⁴ pairs): 8-bit digits 18-25 ms,
4-bit 19 ms, 6-bit 17.5 ms, 5-bit 13.4 ms; 4 elements per thread beat 2 and 8.

**Test runs:** `cargo nextest run -p amg-compute` passes 63/63 on DX12 hardware, on WARP
(`AMG_WGPU_BACKEND=warp`, 8.5 min: the 2²⁴-element cases are slow there; f32 sums within
n·ε₃₂·Σ|x|) and on the f64 gate (`AMG_WGPU_BACKEND=vulkan AMG_REQUIRE_F64=1`, f64 bit-equal to
the tree-order reference). Sizes {0, 1, 255, 256, 257, 65 537, 2²⁰+7, 2²⁴}; 3-run bit identity;
stability property test vs `sort_by_key`. `amg-jobs` 61/61.

**Findings and deviations:**
- DX12/FXC zero-initialises workgroup memory by unrolling, and the compile time grows about
  quadratically with the array size (13 s for the scan and the sort scatter). Every `prim`
  kernel writes a workgroup word before reading it, so they opt out (Interface changes).
- Scratch buffers (dot products, scan partials, sort temporaries and histograms) are allocated per
  call, not pooled. This makes `dot` slow on large inputs (zero-filling a 2²⁵-element temporary
  per call) and is the first thing to fix if a consumer calls it in a loop.
- `GpuReal` gained the supertrait `ReduceScalar` (sealed) so the generic methods can pick the
  kernel type; `u32`/`i32` implement `Sealed`.
- `GpuVec::zeroed` now allocates at least one element (was: at least 4 bytes). Binding an empty
  f64 vector failed validation (minimum binding size 8).
- The CMP-07 leftover is avoided: size-0 cases never read back.

**Leftovers:**
- Pool the scratch buffers; fuse `dot` into the first reduce pass without FMA contraction.
- Sort supports 32-bit keys with 32-bit values only; 64-bit keys and payload structs need a
  new WP if a consumer wants them (AM-13 needs neither).
- The WARP run takes 8.5 min; the 2²⁴ cases may want a smaller size or an `slow` filter in CI.

### 2026-11-10 - CMP-03 sparse direct solvers (lane F implementation)
- **Landed** (`crates/amg-compute`): `sparse` (`SymCsr` with validated `from_raw`, `diagonal`,
  `get`, deterministic row-parallel `spmv` through a lazily built lower-triangle index,
  `add_scaled`, `same_pattern`; `TripletBuilder`; `Ordering`, `FactorOpts`, `FactorEstimate`,
  `PivotReport`, `Inertia`; `SymbolicFactor` (reusable `Arc`, exact `estimate`, `permutation`);
  `Cholesky` (faer supernodal LLᵀ, `pivots`, `solve_in_place`, `solve_many`); `Ldlt` (faer
  intranode Bunch–Kaufman LBLᵀ, `inertia`, solves)); `testprob` (`q1_poisson`, `q1_elasticity`
  with stiffness + consistent mass + body load, `simp_field`). Tests: 18 in
  `tests/sparse_direct.rs` (2 proptests, 1 ignored large analysis) + unit tests; bench `benches/sparse_direct.rs`.
- **faer details** (0.23.2, workspace pin, now with its `rayon` feature): the analysis always asks
  for the supernodal factorisation (`SupernodalThreshold::FORCE_SUPERNODAL`) so `Ldlt` always
  pivots; faer still picks simplicial when `L` has no off-diagonal entries (diagonal matrices),
  which is handled. faer's `SymmetricOrdering::Identity` ignores `Side::Lower` in the symbolic
  phase, so `Ordering::Identity` is passed as an explicit identity permutation. Indices are
  `usize` (a 1 M DOF factor overflows u32 value offsets), so the column indices are copied once
  per analysis; values are passed without a copy. faer does not report the failing pivot's value:
  `NotPositiveDefinite::pivot` is NaN (row is in the original numbering).
- **Additions beyond the Interfaces block** (additive): `SymCsr::get`, `SymbolicFactor::n`,
  `SymbolicFactor::numeric_factorizations`, `Cholesky::symbolic`, `Ldlt::symbolic`,
  `sparse::analyses_on_this_thread` (Done-when 6 counter; thread-local so parallel tests do not
  interfere), `Support: PartialEq + Eq`.
- **Deviation:** `q1_poisson(nodes, coeff: Option<&[f32]>)` and `simp_field(..) -> Vec<f32>`
  take/return a per-cell slice instead of CMP-02's `DenseGrid<f32>`, which does not exist yet.
  CMP-02 (or its first consumer) switches the signature when `DenseGrid` lands.
- **Dirichlet handling in `testprob`:** constrained DOFs stay in the system (3·55³ = 499 125
  rows) with zero off-diagonals and the assembled diagonal, so Clamped and Free share a pattern.
- **Done-when results** (reference machine, shared with 5 other build workers during the run):
  1. **Not met.** `q1_elasticity(55, Clamped)`, AMD, 16 threads, release: analyse 3.61 s +
     factor 372.7 s + solve 36.9 s (first solve; best of 5 later: 1.56 s) = 413 s vs the 20 s
     bar; relative residual 4.15e-12 (≤ 1e-10 met). Factor: 1.79e9 non-zeros, 17.11 GiB,
     2.53e13 flops (≈ 68 GFLOP/s achieved under load). Peak working set 17.3 GiB. The design
     note's "few hundred million non-zeros at 1 M DOF" was off by ~10× for AMD on a 3-D grid.
     Geometric nested dissection (test helper, estimate only): 8.66e8 non-zeros, 7.71 GiB,
     4.89e12 flops (5.2× fewer) — still ≈ 70 s at the measured rate, so ND alone (CMP-10) does
     not reach 20 s with faer on this machine.
  2. **Not met (not run).** Analysis only: `q1_elasticity(70, Clamped)` AMD estimate 4.29e9
     non-zeros, 40.43 GiB, 8.88e13 flops; above ~28 GB of usable RAM, so per the coordinator the
     1 M case was not factored. ND estimate: 2.32e9 non-zeros, 20.62 GiB, 2.10e13 flops (would
     fit). `estimate().bytes` vs measured storage: not measured at 1 M; at 500 k the estimate
     (17.11 GiB) is 6 % below the whole-process peak private memory sampled from outside
     (18.18 GiB, which also holds K and M, ≈ 0.44 GiB, the vectors and faer's transient
     workspace); the in-process before/after-factor reading needs a bench run.
  3. **Not met** at 500 k: `solve_many(6)` 7.61 s vs `solve_in_place` 1.56 s, ratio 4.87 (bar
     2). Correctness of multi-RHS checked (6 columns, residual ≤ 1e-10; small-problem test).
     faer's supernodal solve is memory-bound per column at this size; a blocked solve would need
     changes in faer.
  4. Met: `q1_elasticity(n, Free)` for n = 2..12 gives `NotPositiveDefinite` with a row or a
     `PivotReport` with `below_threshold`; the clamped matrix on the same analysis does not flag.
  5. Met: proptest (96 cases, n ≤ 59, diagonal + up to 3 signed rank-1 updates) — `inertia`
     equals faer's dense eigenvalue sign counts.
  6. Met: `analyses_on_this_thread` unchanged and `numeric_factorizations` = 3 after three
     re-factorisations (`Cholesky` ×2, `Ldlt`) on one `Arc<SymbolicFactor>`.
  7. Met: factor + solve (Cholesky and LBLᵀ) bit-identical over 2 runs for threads None / 1 / 4;
     `spmv` bit-identical across 1, 4, 16 threads.
  8. Met: unit test + proptest — unsorted, duplicate, lower, out-of-range and malformed
     `row_ptr` input all give `InvalidMatrix`.
- **Gates:** `cargo fmt --all --check`, `cargo clippy -p amg-compute --all-targets
  --all-features -- -D warnings`, `cargo clippy --workspace --all-targets -- -D warnings`,
  `cargo nextest run -p amg-compute` (89 passed, 1 skipped: the `#[ignore]` large analysis-only
  test `large_analysis_estimates`, which produced the estimates above) all pass. No numeric GPU
  kernel was touched, so the f64 gate does not apply.
- **Leftovers / decisions for the owner:** the 20 s budget for 500 k DOF is out of reach with
  faer + AMD here (≈ 20× over). Options: (a) CMP-10 ND (≈ 5× fewer flops, ≈ 2.2× less memory;
  makes the 1 M case fit), (b) reconsider CHOLMOD/MUMPS-class supernodal (design note: only if
  CMP-03 misses by > 2× — it does), (c) move the S13 500 k path to the iterative solver
  (CMP-08 / FEM-08) and keep direct for smaller models. Re-measure on an idle machine before
  deciding (`AMG_BENCH_1M=1` only with ND).

### 2026-11-11 - CMP-03 owner decision on the missed budgets; ND 500 k measurement
- **Measurement** (`benches/sparse_direct.rs` with `AMG_BENCH_ORDERING=nd`, release, 16 rayon
  threads, idle machine, run once; criterion repeats skipped): `q1_elasticity(55, Clamped)` with
  the geometric nested dissection of the node grid: analyse 1.45 s + factor 40.23 s + solve
  0.93 s = **42.6 s**, residual 3.74e-12. Factor 8.66e8 non-zeros, `estimate().bytes` 7.71 GiB,
  4.89e12 flops (≈ 122 GFLOP/s). Peak working set 8.42 GiB, peak private 8.50 GiB (sampled from
  outside; includes K, f and vectors). `solve_many(6)` 1.97 s vs `solve_in_place` 0.84 s, ratio
  **2.34**. The AMD case was not re-run: its ≈ 18 GiB peak did not fit in the 14 GiB then free.
- **Recorded numbers for CMP-03 Done-when 1–3:** AMD 500 k 413 s on the loaded machine
  (residual 4.15e-12, 17.11 GiB est.); ND 500 k 42.6 s idle; 1 M analysis-only estimate AMD
  40.43 GiB / ND 20.62 GiB (not factored); `solve_many(6)` ratio 4.87 (AMD, loaded) and 2.34
  (ND, idle).
- **Owner decision (2026-11-11):** keep faer for small and medium problems; add nested
  dissection (CMP-10); route S13 cases of 500 k DOF and more to the GPU iterative solver
  (FEM-08). CMP-03 Done-when 1–3 are narrowed to the recorded measurements above. The 20 s
  (500 k) and 1 M-on-32 GB targets move to CMP-10 (ordering, direct path) and FEM-08 (iterative
  path for large models). No CHOLMOD/MUMPS.
- `docs/notes/compute-numerics.md` §5 corrected: the 1 M factor is ≈ 4.3e9 (AMD) / 2.3e9 (ND)
  non-zeros, not "a few hundred million".

## Interface changes
### 2026-11-10: from CMP-03 (lane F; applied, additive)
- **faer feature unification (S03 `amg-sketch`, S07 `amg-mesh`):** `amg-compute` enables faer's
  `rayon` feature (same `=0.23.2` pin). In any build that contains `amg-compute`, faer's global
  parallelism defaults to the rayon pool, so `amg-mesh`'s hole-fill fairing (`sp_cholesky`, which
  uses the global setting) runs in parallel; it stays deterministic for a fixed thread count.
  `amg-sketch` passes `Par::Seq` explicitly and is unaffected. Owners of S07 to review.
### 2026-11-10: from CMP-01 (lane F; applied, additive)
- **`amg-jobs::gpu` (S00):** `ComputePipelineDesc` gains `pub zero_init_workgroup_memory: bool`
  (`new()` sets `true`, the previous behaviour), part of the pipeline cache key and passed to wgpu's
  `zero_initialize_workgroup_memory`. Reason: FXC compile time of large workgroup arrays (see the
  CMP-01 log). Lane owning S00 to review.
- **`amg-compute` (CMP-07 code, same lane):** `ComputeCtx::gpu_arc()`; `Kernel::without_workgroup_zeroing`
  (crate-private); `GpuVec::zeroed` allocates at least one element.
### 2026-11-09: requests from CMP-07 (lane F; not applied)
- **Lane E (`.config/nextest.toml`):** to show the f64 skips in CI (Done-when 8), add an override
  `filter = 'test(/gpu_f64/)'` with `success-output = "immediate"` to `[profile.ci]`, or store
  success output in the JUnit report.
- **`amg-jobs::gpu` (S00), additive, only if CMP-01 wants it:** a zero-length `Readback` so
  `GpuVec::read` can return an empty result for an empty vector.
- **`amg-jobs::gpu` (S00):** `Readback::start` can panic between `Device::destroy` and the
  lost flag being set (`map_async` validation error). `ComputeCtx` guards its own readback with an
  error scope; a user readback in that window is still exposed. Only explicit `destroy` opens the
  window.

### 2026-11-08: from the S14 refinement (lane F; inputs to the S15 refinement)
See `docs/specs/S14-generative-design.md` Interfaces and `docs/notes/generative-design.md` §5.
- CMP-05: a PCG driver over a caller-supplied matrix-free operator and preconditioner on GPU f32
  buffers (S14 owns the H8 operator and the multigrid V-cycle), with warm start, a relative-residual
  stop and cancellation; kept behind the ADR-004 backend trait.
- CMP-01: typed buffers, dispatch, and deterministic fixed-order reductions (dot, norm, max); no
  float atomics, so topology-optimisation runs are bit-identical.
- CMP-03: Cholesky with separate symbolic and numeric phases (the multigrid coarse level, ≤ 40 k
  DOF, is refactored every design iteration). Same request as S13.
- CMP-02: GEN-02 does not need it (dense per-element arrays only); its done-when can drop GEN-02.
- Coordinator, 2026-11-08: this request was written while S15 was being refined and merged after it. S15's Interfaces already cover the CMP-05 point (`GridLevelOperator`, `Gmg`, `pcg_gpu` on CMP-08), the CMP-01 point (fixed-order reductions, no float atomics) and the CMP-03 point (`SymbolicFactor` + `Cholesky`). The CMP-02 point (drop GEN-02 from its Done-when) is for the CMP-02 implementer to check.
