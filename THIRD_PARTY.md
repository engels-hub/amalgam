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
| SolveSpace `libslvs` | `amg-sketch` fallback solver (ADR-007, only if chosen) | GPL-3.0 | **yes** | yes | Planned (undecided) |
| fidget | `amg-implicit` CPU evaluator (S08, under evaluation) | MPL-2.0 | weak | — | Planned (undecided) |
| wgpu | `amg-jobs`, `amg-render`, `amg-compute` (ADR-004) | MIT / Apache-2.0 | no | no | Planned |
| egui, egui_dock, egui-wgpu | `amg-ui` (ADR-005) | MIT / Apache-2.0 | no | no | Planned |
| glam | geometry math (ADR-006) | MIT / Apache-2.0 | no | no | Planned |
| nalgebra | small dense linear algebra, sketch solver (ADR-006) | Apache-2.0 | no | no | Planned |
| faer | sparse/large linear algebra, FEA (ADR-006) | MIT | no | no | Planned |
| pyo3 | `amg-script` (ADR-011) | MIT / Apache-2.0 | no | no | Planned |
| cudarc | optional CUDA solver backend (ADR-004) | MIT / Apache-2.0 | no | yes | Planned (only if needed) |
| cxx | `amg-occt-sys` bridge (ADR-002) | MIT / Apache-2.0 | no | no | Planned |
| rayon | `amg-jobs` and domain crates (S00) | MIT / Apache-2.0 | no | no | Planned |
| slotmap | generational ids, `amg-core` (S00) | Zlib | no | no | Planned |
| uuid | persistent ids, `amg-core` (S00) | MIT / Apache-2.0 | no | no | Planned |
| thiserror, anyhow | errors (AGENTS.md conventions) | MIT / Apache-2.0 | no | no | Planned |
| tracing, tracing-subscriber | logging (S00) | MIT | no | no | Planned |
| serde, serde_json, zip | `.amg` file format (ADR-010) | MIT / Apache-2.0 (zip: MIT) | no | no | Planned |
| inventory | feature registry, `amg-doc` (S01) | MIT / Apache-2.0 | no | no | Planned |
| im or rpds | persistent collections, `amg-doc` (S01, choice in DOC-01) | MPL-2.0 | weak | no | Planned (undecided) |
| ttf-parser | sketch text (S03) | MIT / Apache-2.0 | no | no | Planned |
| dxf | DXF import (S03) | MIT | no | no | Planned |
| usvg | SVG import (S03) | MIT / Apache-2.0 (≥ 0.38; older releases MPL-2.0) | no | no | Planned |
| criterion, proptest | benches and property tests (dev-only) | MIT / Apache-2.0 | no | no | Planned |
