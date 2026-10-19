# S15 — GPU compute & numerics library

| | |
|---|---|
| **Status** | Draft |
| **Priority** | P1 |
| **Crates** | `amg-compute` (foundation layer) |
| **Lane** | F |
| **Depends on** | S00 (FND-04 `Gpu`) |
| **Parity** | — (enabler for S08, S13, S14, S16) |

## Summary
Shared numeric building blocks so FEA, topology optimization, implicit modeling and AM
analyses don't each reinvent them: GPU buffer/kernel helpers, reductions/scans, sparse
matrices, iterative and direct solvers, eigen solvers, voxel/brick grids.

## Work packages
| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| CMP-01 | WGSL kernel helpers: typed buffers, dispatch, reductions, prefix scan, sort | FND-04 | P1 | Todo | benchmarks vs CPU |
| CMP-02 | Dense/sparse voxel & brick grids (CPU + GPU) | CMP-01 | P1 | Todo | used by IMP-02, GEN-02 |
| CMP-03 | Sparse matrices (CSR) + direct solve (faer Cholesky/LU) | FND-03 | P1 | Todo | 1 M DOF SPD solve benchmark |
| CMP-04 | Eigen solvers (Lanczos / LOBPCG, shift-invert) | CMP-03 | P1 | Todo | |
| CMP-05 | GPU PCG (Jacobi, IC0) + matrix-free operator interface + geometric multigrid | CMP-01 | P1 | Todo | |
| CMP-06 | Optional CUDA backend (`cudarc`) behind feature flag, only if benchmarks warrant (ADR-004) | CMP-05 | P3 | Todo | |
