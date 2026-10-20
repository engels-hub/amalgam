# Third-party dependencies

Required by ADR-012. Every non-trivial dependency is listed here with its licence. Copyleft
(GPL/AGPL) dependencies must sit behind a cargo feature so a permissive build stays possible.

Status: **Planned** = named in a spec or ADR but not yet added to `Cargo.toml` / `vcpkg.json`.
Agents update the row to **In use** (with version) when they add the dependency.

| Dependency | Used by | Licence | Copyleft? | Feature-gated? | Status |
|---|---|---|---|---|---|
| OpenCASCADE (OCCT) 7.8+ | `amg-occt-sys` (ADR-002) | LGPL-2.1 with exception | weak | no (core) | Planned |
| Manifold | `amg-mesh` booleans (ADR-003) | Apache-2.0 | no | no | Planned |
| fTetWild | `amg-fem` tet meshing (ADR-008) | MPL-2.0 | weak | yes | Planned |
| TetGen | `amg-fem` fallback mesher (ADR-008) | AGPL-3.0 | **yes** | yes | Planned |
| SolveSpace `libslvs` + `slvs` 0.6 crate | `amg-sketch` differential-test oracle behind feature `slvs` (S03, SK-13/SK-14); runtime fallback only if the owner decides (S03 OQ 4) | GPL-3.0 | **yes** | yes | Planned (test-only) |
| bindgen | build dep of `slvs` (needs libclang) | BSD-3-Clause | no | yes (with `slvs`) | Planned (test-only) |
| fidget | `amg-implicit` CPU evaluator (S08, under evaluation) | MPL-2.0 | weak | — | Planned (undecided) |
| wgpu | `amg-jobs`, `amg-render`, `amg-compute` (ADR-004); `amg-testkit::gpu_available` adapter probe | MIT / Apache-2.0 | no | no | In use (30.0.1) |
| egui, egui_dock, egui-wgpu | `amg-ui` (ADR-005) | MIT / Apache-2.0 | no | no | Planned |
| glam | geometry math (ADR-006) | MIT / Apache-2.0 | no | no | In use (0.34.1) |
| nalgebra | small dense linear algebra, sketch solver (ADR-006) | Apache-2.0 | no | no | Planned |
| faer | sparse/large linear algebra: FEA, sketch solver blocks, mesh fairing (ADR-006; S03, S07) | MIT | no | no | Planned |
| pyo3 | `amg-script` (ADR-011) | MIT / Apache-2.0 | no | no | Planned |
| cudarc | optional CUDA solver backend (ADR-004) | MIT / Apache-2.0 | no | yes | Planned (only if needed) |
| cxx | `amg-occt-sys` bridge (ADR-002) | MIT / Apache-2.0 | no | no | Planned |
| rayon | `amg-jobs` and domain crates (S00); `amg-testkit` BVH build and Hausdorff | MIT / Apache-2.0 | no | no | In use (1.12.0) |
| slotmap | generational ids, `amg-core` (S00) | Zlib | no | no | In use (1.1.1) |
| uuid | persistent ids, `amg-core` (S00) | MIT / Apache-2.0 | no | no | In use (1.28.0) |
| thiserror, anyhow | errors (AGENTS.md conventions) | MIT / Apache-2.0 | no | no | thiserror in use (2.0.21); anyhow Planned |
| tracing, tracing-subscriber | logging (S00) | MIT | no | no | Planned |
| serde, serde_json, zip | `.amg` file format (ADR-010); serde_json also in the `amg-testkit` layering test | MIT / Apache-2.0 (zip: MIT) | no | no | serde in use (1.0.229); serde_json in use (1.0.154, dev-only); zip Planned |
| inventory | feature registry, `amg-doc` (S01) | MIT / Apache-2.0 | no | no | Planned |
| imbl (fallback: rpds) | persistent collections, `amg-doc` (S01; DOC-01 bench confirms; `im` is unmaintained) | MPL-2.0 | weak | no | Planned |
| ttf-parser | sketch text (S03) | MIT / Apache-2.0 | no | no | Planned |
| dxf | DXF import (S03) | MIT | no | no | Planned |
| usvg | SVG import (S03) | MIT / Apache-2.0 (≥ 0.38; older releases MPL-2.0) | no | no | Planned |
| criterion, proptest | benches and property tests (dev-only) | MIT / Apache-2.0 | no | no | proptest in use (1.12.0, dev-only); criterion in use (0.8.2, dev-only) |
| blake3 | cache keys and manifest hashes, `amg-doc` (S01) | CC0-1.0 OR Apache-2.0 OR Apache-2.0 WITH LLVM-exception | no | no | Planned |
| serde_path_to_error | `.amg` load diagnostics, `amg-doc` (S01) | MIT / Apache-2.0 | no | no | Planned |
| crossbeam-channel | job results, `amg-jobs` (S00) | MIT / Apache-2.0 | no | no | Planned |
| parking_lot | locks, `amg-jobs` (S00) | MIT / Apache-2.0 | no | no | Planned |
| bytemuck | GPU buffer casts, `amg-jobs::gpu` (S00) | Zlib / MIT / Apache-2.0 | no | no | Planned |
| pollster | `Gpu::new_blocking`, `amg-jobs::gpu` (S00); `amg-testkit::gpu_available` | MIT / Apache-2.0 | no | no | In use (1.0.1) |
| tracing-appender | rolling log files (S00) | MIT | no | no | Planned |
| tracing-tracy | optional profiler integration (S00) | MIT / Apache-2.0 | no | yes (`tracy`) | Planned |
| clap | `amg-cli` (S00) | MIT / Apache-2.0 | no | no | In use (4.6.7) |
| approx | float assertions, glam `approx` feature (S00) | Apache-2.0 | no | no | In use (0.5.1) |
| toml | golden files (`amg-testkit`), mesh corpus manifest (S00, S07) | MIT / Apache-2.0 | no | no | In use (1.1.8) |
| crash-handler, minidumper | native crash minidumps (S00, FND-12) | MIT / Apache-2.0 | no | no | Planned |
| smallvec | small inline vectors, `amg-sketch`, `amg-mesh` (S03, S07) | MIT / Apache-2.0 | no | no | Planned |
| fontdb | system font discovery for sketch text (S03, SK-10) | MIT | no | no | Planned |
| kcl-ezpz | sketch solver comparison, dev-only (S03, SK-13) | MIT | no | yes (`ezpz-bench`) | Planned (dev-only) |
| robust | exact orientation predicates, `amg-mesh` (S07) | MIT / Apache-2.0 | no | no | Planned |
| spade | 2D constrained Delaunay for caps and planar holes, `amg-mesh` (S07) | MIT / Apache-2.0 | no | no | Planned |
| rand, rand_chacha | seeded RANSAC and corpus generators, `amg-mesh` (S07) | MIT / Apache-2.0 | no | no | Planned |
| oneTBB (vcpkg `tbb`) | Manifold parallel backend, `amg-manifold-sys` (S07, ADR-003) | Apache-2.0 | no | yes (`manifold`) | Planned |
| manifold-csg-sys | fallback bindings if own `amg-manifold-sys` fails (S07) | MIT / Apache-2.0 | no | yes (`manifold`) | Planned (fallback only) |
| parry3d-f64 | geometry test oracle, dev-only (S07) | Apache-2.0 | no | — | Planned (dev-only) |
| obvhs | BVH benchmark reference, dev-only (S07) | MIT / Apache-2.0 | no | — | Planned (dev-only) |
| cargo-nextest, cargo-deny | dev tools, not linked (S00) | MIT / Apache-2.0 | no | — | Planned |
