# Compute and numerics (S15, `amg-compute`)

Written during the S15 refinement (2026-11-08); §1, §2.1, §7.1 and §8 updated on 2026-11-09
for the owner's answers to QUESTIONS #80 and #81. It holds the evidence and reasoning behind the
S15 Interfaces and work packages. Specs that use `amg-compute` (S08, S13, S14, S16) should read
§1 (GPU constraints) and §2 (precision and determinism) first.

## 1. What the GPU path can rely on

The shared device is `amg_jobs::gpu::Gpu` (FND-09, Done). Its defaults set the limits for
every kernel in `amg-compute`.

| Fact | Source | Consequence |
|---|---|---|
| Backend is DX12 on Windows; Vulkan only with `AMG_WGPU_BACKEND=vulkan` | `GpuConfig::default`, S00 FND-09 | Kernels must run on DX12. |
| DX12 shader compiler is FXC (Shader Model 5.1) | `GpuConfig::dx12_compiler` default, S00 | No subgroup (wave) operations: wgpu exposes `SUBGROUP` on DX12 only with DXC (SM 6.0+). No `f16` shader type: `SHADER_F16` needs DXC (SM 6.2). |
| `SHADER_F64` is Vulkan-only in wgpu 30 | S00 FND-09 log (RTX 5060 Ti, driver 32.0.16.1714), S10; wgpu-hal 30.0.1 sets it only in `vulkan/adapter.rs` (from `shaderFloat64`); naga's MSL backend rejects f64 | No f64 on the default DX12 device, on WARP or on Metal. f64 kernels run only on a Vulkan device with `shaderFloat64`; everywhere else the f32 fallback runs (§2.1). |
| WGSL atomics are `atomic<u32>` / `atomic<i32>` only | WGSL spec | No float atomics. Sums use tree reductions, never atomic adds. |
| `max_compute_workgroups_per_dimension` = 65 535 | WebGPU default, also the DX12 value | Large 1-D launches fold into a 2-D grid (§3.1). |
| `max_storage_buffer_binding_size` is adapter-specific (WARP is smaller) | wgpu `Limits` | One binding never exceeds the device limit; `GpuVec` checks it and returns `ComputeError::TooLarge` instead of a wgpu validation panic. |
| CI runs on WARP (software DX12) | S00 FND-05/FND-09 | Every kernel test runs on WARP; timings only on the reference GPU. |

Reference machine (checked on the owner's machine, 2026-11-08): AMD Ryzen 7 5700X3D (8 cores,
16 threads, AVX2, no AVX-512), 32 GB RAM, RTX 5060 Ti with 16 GB VRAM (driver 617.14 /
32.0.16.1714). GeForce Blackwell runs FP64 at 1/64 of the FP32 rate (≈ 0.37 of ≈ 23.7
TFLOPS), so with `SHADER_F64` (Vulkan) compute-bound f64 kernels are slow, while
bandwidth-bound ones (CSR SpMV at ≈ 0.17 flop/byte against 448 GB/s) mainly pay for the larger
values (12 instead of 8 bytes per non-zero, about 1.5×). See §2.1.

Decisions:
- Kernels are written for FXC/SM 5.1 semantics: workgroup shared memory and barriers only, no
  subgroups. A subgroup fast path may be added later behind `Gpu::has(SUBGROUP)`, but every
  kernel keeps the portable path, and tests compare both.
- f16 data is stored packed, two values per `u32`, and read with `unpack2x16float` (core WGSL;
  naga emits HLSL `f16tof32`, which SM 5 has). Writes use `pack2x16float`.
- Workgroup size 256 by default, set through a WGSL `override` constant so a later tuning pass
  needs no source change (`ComputePipelineDesc::constants`).

## 2. Precision and determinism

### 2.1 Precision policy (S15 open question 1; owner policy 2026-11-09, ADR-004 note)
The 2026-11-08 refinement proposed f32-only kernels (QUESTIONS #81). The owner did not accept
that. The policy is:
- **f64 kernels where the device supports `SHADER_F64`.** In wgpu 30 that is a Vulkan device
  whose driver reports `shaderFloat64` (most desktop GPUs, including the reference RTX 5060 Ti).
  DX12 (the Windows default), WARP and Metal never report it (§1).
- **f32 kernels plus f64 iterative refinement on the CPU everywhere else** (the default DX12
  device, WARP in CI). This is mixed-precision iterative refinement (Carson and Higham 2018,
  three-precision refinement; the classic form is Wilkinson's). The residual `r = b − A·x` is
  computed in f64 on the CPU, the correction `A·d ≈ r` is solved on the GPU in f32 to a loose
  tolerance (1e-4 relative), and `x += d` is applied in f64. Each outer step gains about
  `−log10(inner tolerance)` digits while `κ(A)·ε₃₂ < 1` holds (ε₃₂ ≈ 6e-8). That covers
  elasticity with condition numbers up to ~1e7. Topology optimisation only needs about 1e-5
  relative accuracy, so it can skip the outer loop.
- **Chosen at runtime, per device.** `PrecisionPolicy::Auto` resolves to F64 exactly when
  `Gpu::has(SHADER_F64)` (FND-09's `GpuConfig` already requests it as optional). `ForceF32`
  runs the fallback on an f64-capable device (tests compare both paths on one device);
  `RequireF64` errors on devices without it (the f64 test gate). `AMG_COMPUTE_PRECISION`
  overrides the policy. Precision is never a Cargo feature.
- **One kernel source, two instantiations.** WGSL has no generics, so numeric kernels are
  written over `alias real`, which the prelude binds to f32 or f64 (naga 30 needs no `enable`
  directive for f64; literals take the `lf` suffix, so kernels write `real(…)`). Rust types are
  generic over `R: GpuReal`. `solve_gpu` is the single place that picks the path: on F64, PCG
  runs to the final tolerance (with a CPU f64 residual check, normally one outer step); on F32,
  `refine` wraps a loose f32 PCG.
- **Both paths are tested.** CI (WARP, DX12) runs the F32 path; the F64 path runs on the
  reference machine with Vulkan (the "f64 gate", S15 Testing) and is reported as skipped, with
  a reason, on devices without `SHADER_F64`. A Mesa lavapipe job could bring it into CI (S15
  open question 9).
- **Still no emulated double** ("double-float", pairs of f32 with error-free transforms).
  Error-free transforms need either a fused multiply-add or strict IEEE evaluation order. WGSL
  `fma` lowers to HLSL `mad`, which DX12 does not guarantee to be fused, and FXC may reorder
  float expressions (naga does not emit HLSL `precise`). A df64 kernel that silently loses its
  extra bits is worse than an honest f32 one. df64 costs 10–20 f32 operations per add or
  multiply, so it would also be slow.
- **The backend default decides which path users get.** With the Windows default (DX12), no
  Windows user gets the F64 path unless they select Vulkan. And because GeForce GPUs run FP64 at
  1/64 rate (§1), the F64 path may be slower than F32 + refinement for compute-bound kernels.
  Budgets are therefore measured on the F32 path on the default device, and F64 numbers are
  recorded beside them; whether to change the default or let a solver prefer F32 on f64 devices
  is S15 open question 8.
- The CPU is the f64 reference everywhere: every GPU primitive and solver has a CPU
  implementation with the same semantics, and tests compare them.

### 2.2 Determinism (vision: bit-stable within a build)
- On the CPU, every result is bit-identical across runs and thread counts. Parallel sums use a
  fixed chunking (`par_chunks` of a constant size, then a sequential sum of the partials), never
  `rayon::sum` over a variable split.
- On the GPU, every result is bit-identical across runs on the same device and driver:
  reductions use a fixed tree that depends only on the input length and the workgroup size, and
  nothing depends on the order in which atomics land. Where an atomic counter allocates slots
  (S16 A-buffer), the consumer sorts afterwards. Results differ across devices (WARP vs NVIDIA)
  in the last bits, so cross-device tests use tolerances.
- GPU allocation that must be deterministic (brick grids, compaction) uses mark → exclusive scan
  → scatter, never `atomicAdd` on a counter.
- PCG and Lanczos iteration counts are therefore reproducible, which keeps the S13 results and
  the recompute cache stable.

## 3. GPU layer and primitives (CMP-07, CMP-01)

### 3.1 Buffers and dispatch (CMP-07)
`amg-jobs::gpu` already has upload, storage allocation, readback and a pipeline cache. CMP-07
adds what every kernel needs on top, without wrapping wgpu away:
- `GpuVec<T>`: a typed storage buffer with its length, created from a slice or zeroed, with
  binding-size checks.
- `ComputeCtx`: records dispatches into one command encoder and submits in batches. Small kernel
  parameters go into a per-context uniform arena (256-byte aligned slots), because core wgpu has
  no push constants on all backends.
- `dispatch_1d(n)`: launches `ceil(n / 256)` workgroups folded into `x ≤ 65 535` by `y`. The
  shared WGSL prelude (`PRELUDE_WGSL`) has `amg_linear_id(gid, nwg)` so kernels get a flat index
  and a bound check.
- Optional GPU timestamps (`TIMESTAMP_QUERY`) per labelled dispatch, used by the benchmarks.
- Cancellation: long loops call `JobCtx::checkpoint` between submissions; `Readback::wait`
  already returns `DeviceLost` instead of hanging.

### 3.2 Primitives (CMP-01)
Requested by S16 (AM-13: sort, prefix scan, segmented reduce, atomic counter buffers), S08
(IMP-02: compaction and allocation), S13/S14 (dot products, norms).

| Primitive | Algorithm | Why |
|---|---|---|
| Reduce (sum, min, max; f32, f64 where supported, u32, i32) | Two-level tree: each workgroup reduces 256 × 8 elements in shared memory, a second pass reduces the partials | Fixed order, no atomics, ~memory bandwidth. |
| Dot product, squared norm (f32; f64 where supported) | Same tree, fused multiply in the first level | PCG needs 2 dots per iteration. |
| Exclusive / inclusive scan (u32) | Reduce-then-scan (3 passes: tile sums, scan of sums, tile scan with offset) | Portable. Single-pass decoupled look-back (Merrill and Garland 2016) needs forward-progress guarantees that WebGPU does not give; it can hang on some GPUs (Levien, "Prefix sum on portable compute shaders", 2021). The fallback variant (Smith, Levien, Owens, "Decoupled fallback", 2025) is a later optimisation. |
| Stream compaction | Mark → scan → scatter | Deterministic order. |
| Segmented reduce (f32 or f64 sum/min/max) | Segments given by an offsets array; one workgroup per segment for long segments, a flag-scan for short ones | AM-13 per-direction scores. |
| Radix sort (u32 keys, optional u32 values), stable | LSD, 4 passes of 8 bits; each pass = per-tile histogram, scan of histograms, stable scatter with per-tile ranking in shared memory | Onesweep (Adinets and Merrill 2022) and `wgpu_sort` use look-back and subgroups (see the scan row). Reduce-then-scan radix sort works on FXC. `key_bits` skips unneeded passes. |
| Atomic counter buffer | `GpuVec<u32>` of length 1 + reset | S16 A-buffer fragment allocation. |

Prior art read: `wgpu_sort` (MIT, used by web-splat; Onesweep-style, needs subgroups), Brush's
sort (Apache-2.0; CubeCL/burn based), CUB (BSD-3), Raph Levien's portable scan notes. We own the
kernels because of the FXC and determinism constraints; the crates are references only.

## 4. Grids (CMP-02)

### 4.1 Needs
- S08 (IMP-02, IMP-12): narrow-band SDF bricks, edge 8 cells, f16 (and f32) samples, 9³ corner
  samples per brick (one apron layer so trilinear lookups stay inside one brick), insert and
  lookup by brick coordinate, a WGSL-readable layout, inside/outside flags for empty regions.
  Bounds come from the part (e.g. 200 mm at 0.05 mm cells = 4 000 cells per axis).
- S14 (GEN-02..04), S16 (AM-09): dense voxel fields (density, flags) on regular boxes, x fastest.

### 4.2 Layout decision: fixed two-level tree with a dense root
A flat dense index over bricks is too big: 4 000³ cells = 500³ bricks = 1.25e8 slots, 500 MB of
`u32`. An open-addressing hash works but GPU insertion order is non-deterministic and lookups
need probe loops. OpenVDB (5-4-3 tree with a hash root; MPL-2.0) and NanoVDB (its GPU-friendly
linear form) show the standard answer: a shallow tree. We use a fixed two-level variant that fits
bounded CAD parts:

```
root  : dense u32 array over tiles   (tile = 8³ bricks = 64³ cells; matches S08's 64³ tiles)
tiles : 512 u32 per allocated tile   (brick index, or a fill code)
data  : brick payloads               (samples_per_brick values each)
codes : 0xFFFF_FFFF = FILL0 (e.g. outside), 0xFFFF_FFFE = FILL1 (e.g. inside)
```

For 4 000³ cells the root has 63³ = 250 k entries (1 MB); each allocated tile node is 2 KB. A
lookup is two dependent loads plus the sample load. The root is limited to 2²⁴ entries
(256³ tiles = 16 384³ cells); larger boxes return `ComputeError::Shape`.

- A brick with apron 1 stores 9³ = 729 samples. As f16 that is 365 `u32` words (the last half
  word is padding); as f32, 729 words. The stride is stored in the grid metadata.
- Empty slots carry one of two fill values (`fill[0]`, `fill[1]`), so S08's per-tile
  inside/outside classification is stored in the grid itself. Values are in index units; world
  mapping (origin, cell size) belongs to the owner (S08 `SdfBricks`, `SampledGrid`).
- **Determinism:** CPU grids keep bricks in Morton order of the brick coordinate after
  `canonicalize()`. GPU builds allocate with mark → scan → scatter over candidate bricks in
  Morton order, so the GPU and CPU builds give the same brick order.
- Memory example: a 100 mm-radius sphere at 0.1 mm cells, band ±4 cells: about 3e5 bricks ×
  1 460 B ≈ 440 MB of f16 data. That fits the 16 GB GPU; the index adds well under 10 %.

### 4.3 WGSL access
Bind-group indices cannot be WGSL `override`s, so the grid's WGSL is generated:
`brick_grid_wgsl(group, first_binding, encoding)` returns declarations for the 4 bindings (meta
uniform, root, tiles, data) and the functions `bg_slot(brick) -> u32`, `bg_sample(ijk) -> f32`
and `bg_trilinear(p) -> f32` (p in cell units). Callers prepend it to their shader source.

### 4.4 Dense grids
`DenseGrid<T>`: dims `[u32; 3]`, x fastest, cell- or node-centred by the caller's convention,
uploaded as a plain `GpuVec<T>` plus a 16-byte dims uniform. No tree, no apron.

## 5. Sparse direct solvers (CMP-03, CMP-10)

- **faer** (MIT) is the backend (ADR-006). The workspace pins `faer =0.23.2` (`amg-mesh`,
  `amg-sketch`); `amg-compute` uses the same pin. faer 0.24.4 exists (2026-06); moving the
  workspace is a separate WP for all three crates.
- faer provides supernodal and simplicial sparse Cholesky with a separate symbolic phase, sparse
  LDLᵀ, Bunch–Kaufman pivoting inside supernodes (LBLᵀ) and LU, with AMD/COLAMD orderings. It is
  CSC-based. Our symmetric matrix stores the **upper triangle in CSR** (S13 request), which is
  exactly the lower triangle in CSC, so it is passed to faer without copying (`Side::Lower`).
- **No CHOLMOD** for now: SuiteSparse's supernodal module is GPL and needs a C/Fortran build on
  Windows; faer's own benchmarks put it close to CHOLMOD. CMP-03 measures faer; CHOLMOD is
  reconsidered only if CMP-03 misses the S13 budget by more than 2×.
- **LU dropped from CMP-03:** no P1 consumer needs a non-symmetric sparse solve (S13 thermal and
  elasticity are symmetric; radiation is linearised by Picard). Add it when a consumer appears;
  faer has it.
- **Memory estimate before factoring** (S13 request): after the symbolic phase the factor's
  non-zero count is known exactly, so `FactorEstimate { factor_nnz, bytes, flops }` is exact for
  storage and a good proxy for time. S13 compares `bytes` with free RAM (60 % rule, FEM-08).
- **Pivot report** (S13 request): faer stops at the first non-positive pivot and reports its
  (permuted) index; we map it back to the original row. After a successful factorisation we also
  report the smallest `d_ii / a_ii` ratio and its row, so S13 can flag nearly unconstrained
  bodies.
- **Inertia** (negative/zero/positive pivot counts of LDLᵀ / LBLᵀ) gives a Sturm count: the
  number of eigenvalues of `K − σB` below 0. CMP-04 uses it to prove that a band search found
  every mode.
- **Cancellation:** faer's numeric factorisation cannot be interrupted. Cancel latency is one
  factorisation (≈ 10 s at 500 k DOF). Checkpoints sit between symbolic, numeric and solve phases.
- **Ordering:** AMD is faer's default. For 3-D meshes nested dissection usually gives less fill.
  CMP-10 (P2) adds a geometric nested dissection (node coordinates are always known in FE) or
  METIS (Apache-2.0, vcpkg) behind a feature, and is only needed if CMP-03's 500 k DOF benchmark
  misses budget.
- Rough size on the reference machine: 3-D elasticity at 1 M DOF has a factor of a few hundred
  million non-zeros (several GB) and needs ~10¹² flops, so roughly 10–30 s on 8 cores. S13's
  practical direct limit of about 1.5 M DOF is consistent with 32 GB RAM.

## 6. Eigen solvers (CMP-04, CMP-09)

- **Shift-invert Lanczos** (Ericsson and Ruhe 1980) with thick restart (Wu and Simon 2000) and
  full B-orthogonalisation (classical Gram–Schmidt twice, DGKS). The small projected problem is
  solved with faer's dense self-adjoint eigen solver.
  - Vibration mode: `K φ = λ M φ`, operator `(K − σM)⁻¹ M`, M-inner product. σ < 0 keeps
    `K − σM` positive definite even for free-free models (S13: σ = −(2π·1 Hz)²), so Cholesky
    works and rigid modes come out near 0.
  - Buckling mode (Grimes, Lewis and Simon 1994): `K φ = λ K_G φ` with K positive definite and
    K_G indefinite; operator `(K − σK_G)⁻¹ K`, K-inner product. With σ ≠ 0 the shifted matrix is
    indefinite, so it uses LBLᵀ.
  - The factorisation is a callback (`ShiftInvert` trait), as S13 asked, so S13 can reuse its own
    factorisation and constraint handling. A convenience path builds and factors `K − σB` from
    two `SymCsr`.
  - Band search: all λ in `[lo, hi]`. After convergence, the Sturm counts at `lo` and `hi`
    (inertia of `K − lo·B` and `K − hi·B`) must equal the number found; otherwise the solver
    adds a shift inside the band and continues.
- ARPACK (BSD-3) was rejected as a dependency: Fortran toolchain on Windows. Spectra (MPL-2.0,
  C++) and SLEPc are references only.
- **LOBPCG** (Knyazev 2001), CMP-09, P2: block, preconditioned, matrix-free; runs with the GPU
  operator and preconditioner (f32 or f64 products by §2.1, f64 Rayleigh–Ritz on the CPU). For
  models past the direct limit and for GEN-09 frequency constraints. `linfa-linalg`'s LOBPCG
  (MIT/Apache) is a reference.

## 7. Iterative solvers (CMP-08, CMP-05, CMP-11)

### 7.1 PCG and the backend trait (CMP-08)
- Public traits: `LinearOperator` (CPU, f64), `GpuOperator<R>` and `GpuPreconditioner<R>` (on
  the GPU, generic over `R: GpuReal` = f32 or f64, §2.1). S13 plugs its p-multigrid in as a
  `GpuPreconditioner`; S14 implements its H8 elasticity as a `GpuOperator`.
- GPU matrices: `GpuCsr<R>` with f32 or f64 values and u32 columns, **full** (both triangles)
  storage. Symmetric SpMV from upper storage would need float atomics, which WGSL does not have.
  2 M DOF TET10 (≈ 160 M non-zeros full) is ≈ 1.3 GB in f32 and ≈ 1.9 GB in f64, fine on 16 GB.
- Preconditioners shipped: Jacobi, 3×3 block Jacobi (one node's DOFs; much better for
  elasticity), and Chebyshev–Jacobi (polynomial smoother; eigen bounds from a few Lanczos steps;
  Adams et al. 2003 for its use as an MG smoother). The same Chebyshev type is the smoother for
  S13's p-multigrid and for CMP-05.
- **IC(0) dropped:** triangular solves are sequential; level scheduling on 3-D FE graphs gives
  little parallelism, and S13 chose p-multigrid instead. Recorded so it is not re-proposed
  without a benchmark.
- Convergence checks need a readback. PCG reads the residual norm every `check_every`
  iterations (default 8), asynchronously, so the GPU is not idle while the CPU waits.
- ADR-004 amendment: the PCG loop is generic over a crate-private `VectorBackend` trait (alloc,
  copy, axpy, dot, scale) with a wgpu implementation per precision; CMP-06 adds a CUDA one.
  Callers only see `SolverDevice`, `GpuSystem` and `solve_gpu`.

### 7.2 Geometric multigrid on structured grids (CMP-05)
For voxel FE (S14 topology optimisation, S16 AM-09): vertex-centred grids, coarsening by 2 per
axis, trilinear prolongation, restriction = its transpose, Chebyshev–Jacobi smoothing (2 pre, 2
post), V-cycle as a PCG preconditioner (MGPCG). Coarse operators are re-discretised from
coarsened coefficients (child-cell average, as in Wu, Dick and Westermann 2016), which keeps
every level matrix-free. Galerkin coarse operators (RAP) are optional on the coarsest levels.
The coarsest level (≤ 50 k DOF) is assembled and solved with CMP-03 Cholesky in f64 on the CPU.
Prior art: Wu, Dick and Westermann, "A system for high-resolution topology optimization" (TVCG
2016, GPU MGPCG, millions of voxels in seconds); Aage et al., "Giga-voxel computational
morphogenesis" (Nature 2017, MG-preconditioned Krylov on CPUs); Liu et al., narrow-band topology
optimisation (SIGGRAPH Asia 2018).
- Dims that are not `2^k·m + 1` are padded with zero-coefficient cells (void) and Dirichlet
  nodes stay fixed through a mask on every level.
- `GridLevelOperator` is the trait an owner implements (apply, diagonal, coarsen, fixed mask).
  CMP-05 ships a variable-coefficient Q1 Poisson stencil as the test operator and as a template
  for GEN-03's H8 elasticity.

### 7.3 Algebraic multigrid (CMP-11, P2)
Smoothed aggregation (Vaněk, Mandel, Brezina 1996) with the 6 rigid-body modes as near-null
space, setup on the CPU, V-cycle on the GPU. Needed when S13's TET4 coarse level is still too
large for a direct solve (S13 note §8.3). AMGCL (MIT, C++) is the reference for the design and
the backend split.

## 8. CUDA backend (CMP-06)
`cudarc` (MIT/Apache-2.0) behind feature `cuda`, never required. It implements the crate-private
`VectorBackend` and the CSR SpMV / Chebyshev kernels, with CUDA kernels compiled at runtime by
NVRTC so no CUDA toolkit is needed to build. ADR-004's amendment makes CUDA the first
optimisation to pursue once the wgpu solvers are benchmarked. **Timing (owner, 2026-11-09,
QUESTIONS #80 (b)):** CMP-06 is P2 in Phase 5 and starts only if the wgpu solver benchmarks
(CMP-08 bench 3, CMP-05 bench 2, or the FEM-08 / GEN-03 budget benches) miss their budgets; it
becomes the default only if ≥ 1.5× faster than wgpu on the same precision path. CUDA devices
always have f64, so CMP-06 implements both precisions and follows §2.1's policy through
`SolverDevice::precision`.

## 9. Test problems
`amg_compute::testprob` (public, used by S13/S14 benchmarks too) generates structured problems
with known properties:
- 3-D Q1 Poisson on an `n³` node grid, optionally with a per-cell coefficient field (random,
  lognormal, or SIMP-like 0/1 with a contrast factor).
- 3-D Q1 linear elasticity on an `n³` node grid (3 DOF per node, E = 1, ν = 0.3), clamped on one
  face, or free-free.
- Diagonal and 1-D Laplacian pencils with analytic eigenvalues.
Sizes used by the Done-when items: 500 k DOF elasticity is the 55³-node grid (499 125 DOF),
1 M DOF is 70³ (1 029 000 DOF), 2 M DOF is 88³ (2 044 416 DOF).

## Sources
- WGSL and WebGPU specs (atomics, limits, `pack2x16float`); wgpu 30 `Features` docs (`SUBGROUP`,
  `SHADER_F16`, `SHADER_F64` backend notes).
- wgpu-hal 30.0.1 source (`vulkan/adapter.rs` is the only backend that sets `SHADER_F64`) and
  naga 30.0.1 source (WGSL `lf` literals, no `enable` needed for f64; the MSL backend rejects
  f64), read from the cargo registry on 2026-11-09.
- E. Carson, N. Higham, "Accelerating the solution of linear systems by iterative refinement in
  three precisions", SIAM J. Sci. Comput. 2018.
- D. Merrill, M. Garland, "Single-pass parallel prefix scan with decoupled look-back", NVIDIA
  2016. A. Adinets, D. Merrill, "Onesweep: a faster least significant digit radix sort for GPUs",
  2022. R. Levien, "Prefix sum on portable compute shaders", 2021. T. Smith, R. Levien,
  J. Owens, "Decoupled fallback: a portable single-pass GPU scan", 2025.
- K. Museth, "VDB: high-resolution sparse volumes with dynamic topology", ACM TOG 2013;
  NanoVDB (OpenVDB project, MPL-2.0).
- faer documentation (sparse Cholesky, LDLᵀ, LBLᵀ, orderings).
- T. Ericsson, A. Ruhe, "The spectral transformation Lanczos method", Math. Comp. 1980.
  K. Wu, H. Simon, "Thick-restart Lanczos method for large symmetric eigenvalue problems",
  SIMAX 2000. R. Grimes, J. Lewis, H. Simon, "A shifted block Lanczos algorithm for solving
  sparse symmetric generalized eigenproblems", SIMAX 1994. A. Knyazev, "Toward the optimal
  preconditioned eigensolver: LOBPCG", SISC 2001.
- M. Adams, M. Brezina, J. Hu, R. Tuminaro, "Parallel multigrid smoothing: polynomial versus
  Gauss–Seidel", J. Comput. Phys. 2003. P. Vaněk, J. Mandel, M. Brezina, "Algebraic multigrid by
  smoothed aggregation", Computing 1996.
- J. Wu, C. Dick, R. Westermann, "A system for high-resolution topology optimization", IEEE TVCG
  2016. N. Aage et al., "Giga-voxel computational morphogenesis for structural design", Nature
  2017. H. Liu et al., "Narrow-band topology optimization on a sparsely populated grid", ACM TOG
  2018.
