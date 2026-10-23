# Third-party dependencies

Required by ADR-012. Every non-trivial dependency is listed here with its licence. Copyleft
(GPL/AGPL) dependencies must sit behind a cargo feature so a permissive build stays possible.

Status: **Planned** = named in a spec or ADR but not yet added to `Cargo.toml` / `vcpkg.json`.
Agents update the row to **In use** (with version) when they add the dependency.

| Dependency | Used by | Licence | Copyleft? | Feature-gated? | Status |
|---|---|---|---|---|---|
| OpenCASCADE (OCCT) 8.0.1 (vcpkg `opencascade` 8.0.1 at baseline `2026.10.16`, `default-features: false`, no TBB; fallback 7.9.3#1; FND-13, QUESTIONS #27) | `amg-occt-sys` (ADR-002, S02); also S20 strategies (`GeomFill_Gordon`, `GeomPlate`) | LGPL-2.1-only with the OCCT exception (port installs LICENSE_LGPL_21 and OCCT_LGPL_EXCEPTION). Linked statically; relinking possible from the object files | weak | yes (`occt`, default on) | Planned |
| Manifold | `amg-mesh` booleans (ADR-003) | Apache-2.0 | no | no | Planned |
| fTetWild | `amg-fem` tet meshing (ADR-008) | MPL-2.0 | weak | yes | Planned |
| TetGen | `amg-fem` fallback mesher (ADR-008) | AGPL-3.0 | **yes** | yes | Planned |
| SolveSpace `libslvs` + `slvs` 0.6 crate | `amg-sketch` differential-test oracle behind feature `slvs` (S03, SK-13/SK-14); runtime fallback only if the owner decides (S03 OQ 4) | GPL-3.0 | **yes** | yes | In use (0.6.0, test-only, feature `slvs`, never default; needs libclang ≤ 15) |
| bindgen | build dep of `slvs` (needs libclang ≤ 15) | BSD-3-Clause | no | yes (with `slvs`) | In use (0.64.0, test-only) |
| SolveSpace `test/constraint/*.slvs` files (commit 5bb74ad2) | **test data only**: `testdata/sketch/corpus/solvespace/`, read at test time by `amg-sketch` (S03, SK-14); never compiled, linked or shipped; licence text and `NOTICE` kept beside the files | GPL-3.0-or-later | **yes** (data) | no (not a dependency) | In use (test data) |
| ezpz `test_cases/*/problem.md` (commit 915882cc) | **test data only**: `testdata/sketch/corpus/ezpz/` (S03, SK-14) | MIT | no | no | In use (test data) |
| FreeCAD Sketcher test scenarios (commit 33adf618) | **test data only**: hand-ported scenarios, `testdata/sketch/corpus/planegcs/` (S03, SK-14); no FreeCAD file copied verbatim | LGPL-2.1-or-later | weak (data) | no | In use (test data) |
| fidget | `amg-implicit` CPU evaluator (S08, under evaluation) | MPL-2.0 | weak | — | Planned (undecided) |
| wgpu 30.0.1 | `amg-jobs`, `amg-render`, `amg-compute` (ADR-004) | MIT / Apache-2.0 | no | no | In use (30.0.1; `amg-jobs::gpu` and the `amg-testkit` GPU probe, both behind feature `gpu`, default on) |
| egui, eframe, egui-wgpu 0.37.0 (or 0.36.2, QUESTIONS #23) | `amg-ui`, `amg-app`, `amg-render` (feature `egui`; eframe as dev-dep for the spin example) (ADR-005, S10, S11) | MIT / Apache-2.0 | no | no | Planned |
| egui_extras 0.37.0 (features `svg`, `serde`) | `amg-ui` tables and SVG icons (S11) | MIT / Apache-2.0 | no | no | Planned |
| resvg 0.45.x (via egui_extras `svg`) | SVG icon rasterisation (S11) | Apache-2.0 / MIT | no | no | Planned |
| egui_kittest 0.37.0 (+ transitive kittest 0.5.0) | `amg-ui` UI tests, dev-only (S11) | MIT / Apache-2.0 | no | no | Planned (dev-only) |
| egui_dock 0.21.1 (needs egui ^0.36; 0.22.x expected for 0.37) | `amg-ui::dock` behind `DockHost` (ADR-005, S11) | MIT | no | no | Planned |
| egui_tiles 0.18.0 | `amg-ui::dock` fallback if egui_dock lags egui (S11, QUESTIONS #23) | MIT / Apache-2.0 | no | no | Planned (fallback only) |
| rfd 0.17.2 | `amg-ui` native file dialogs (S11) | MIT | no | no | Planned |
| directories 6.0.0 | `amg-ui` prefs and layout paths (S11) | MIT / Apache-2.0 | no | no | Planned |
| bitflags 2 | `amg-render` `PickFilter`, `amg-features::dialog` `EntityFilter` (S10, S11; already transitive via wgpu) | MIT / Apache-2.0 | no | no | Planned |
| png 0.18.1 | `amg-render` headless capture, goldens, thumbnails; `amg-io` 3MF thumbnails (S10, S12) | MIT / Apache-2.0 | no | no | Planned |
| meshopt 0.7.0 (meshoptimizer) | `amg-render` clusters/LOD (S10 RND-14) | MIT / Apache-2.0 (meshoptimizer: MIT) | no | no | Planned (P2) |
| Viridis colormap table (data) | `amg-render` (S10 RND-08) | CC0-1.0 (matplotlib colormaps) | no | no | Planned |
| Turbo colormap table (data) | `amg-render` (S10 RND-08) | Apache-2.0 (Google) | no | no | Planned |
| mach-dxcompiler-rs (via wgpu `static-dxc`) | DX12 shader compiler, only if RND-01 adopts StaticDxc (S10) | MIT wrapper; bundled DXC under University of Illinois/NCSA (verify in RND-01) | no | yes (`static-dxc`) | Planned (conditional) |
| glam | geometry math (ADR-006) | MIT / Apache-2.0 | no | no | In use (0.34.1) |
| nalgebra | small dense linear algebra, sketch solver (ADR-006); `amg-nurbs` internal dense solves (S20) | Apache-2.0 | no | no | Planned |
| faer | sparse/large linear algebra: FEA, sketch solver blocks, mesh fairing (ADR-006; S03, S07) | MIT | no | no | In use (=0.23.2, `std` + `sparse-linalg`, `amg-mesh` hole-fill fairing, MESH-20) |
| pyo3 | `amg-script` (ADR-011) | MIT / Apache-2.0 | no | no | Planned |
| cudarc | optional CUDA solver backend (ADR-004) | MIT / Apache-2.0 | no | yes | Planned (only if needed) |
| cxx | `amg-occt-sys` bridge (ADR-002) | MIT / Apache-2.0 | no | no | Planned |
| rayon | `amg-jobs` and domain crates (S00); `amg-mesh` (S07) | MIT / Apache-2.0 | no | no | In use (1.12.0) |
| slotmap | generational ids, `amg-core` (S00) | Zlib | no | no | In use (1.1.1) |
| uuid | persistent ids, `amg-core` (S00) | MIT / Apache-2.0 | no | no | In use (1.28.0) |
| thiserror, anyhow | errors (AGENTS.md conventions) | MIT / Apache-2.0 | no | no | thiserror in use (2.0.21); anyhow Planned |
| tracing, tracing-subscriber | logging (S00), `amg-jobs::diag`, `amg-cli` | MIT | no | no | In use (tracing 0.1.44, tracing-subscriber 0.3.23 with `env-filter`) |
| serde, serde_json | `.amg` file format (ADR-010) | MIT / Apache-2.0 | no | no | In use (serde 1.0.229; serde_json 1.0.154) |
| zip (zip-rs/zip2) 9.0.5, `default-features = false`, features `deflate-flate2-zlib-rs` | `.amg` container (ADR-010, S01), 3MF OPC in `amg-io` (S12) | MIT | no | no | In use (9.0.5; `amg-doc` .amg container, DOC-07) |
| quick-xml 0.42.0 | `amg-io` 3MF XML (S12) | MIT | no | no | Planned |
| memchr 2.8.3 | `amg-io` parsers (S12; already transitive) | Unlicense / MIT | no | no | Planned |
| image 0.25.10 (png + jpeg features) | `amg-io` canvas images (S12 IO-10) | MIT / Apache-2.0 | no | no | Planned |
| stl_io 0.11.0, tobj 4.0.5, ply-rs-bw 4.0.1 | `amg-io` parser test oracles, dev-only (S12) | MIT | no | no | Planned (dev-only) |
| dhat 0.3.3 | `amg-io` peak-heap tests, dev-only (S12) | MIT / Apache-2.0 | no | no | Planned (dev-only) |
| arbitrary 1.5.0, libfuzzer-sys 0.4.13, cargo-fuzz 0.13.2 | `fuzz/` crate outside the workspace (S12 IO-16) | arbitrary, cargo-fuzz: MIT / Apache-2.0; libfuzzer-sys: (MIT OR Apache-2.0) AND NCSA | no | n/a (outside workspace) | Planned (fuzz only) |
| lib3mf 2.5.0 (PyPI, 3MF Consortium), xmlschema 4.3.2 (PyPI) | CI validation of written 3MF, not linked (S12) | lib3mf: BSD-2-Clause; xmlschema: MIT | no | n/a (tool) | Planned (CI tool) |
| 3MF Consortium test_suites (pinned commit) | `amg-io` test data, fetch-only (optional ≤ 50 MB LFS subset) (S12) | BSD-2-Clause | no | n/a (data) | Planned (test data) |
| winreg 0.56 | `amg-am` slicer detection, Windows-only (S16) | MIT | no | no | Planned |
| inventory | feature registry, `amg-doc` (S01) | MIT / Apache-2.0 | no | no | In use (0.3.25) |
| imbl | persistent collections, `amg-doc` (S01; DOC-01 bench confirmed; `im` is unmaintained). Pulls in `imbl-sized-chunks` (MPL-2.0+), `archery` (MIT), `wide`/`safe_arch` (Zlib/Apache-2.0/MIT), `rand_xoshiro`, `rand_core` 0.9 (MIT/Apache-2.0) | MPL-2.0 (crates.io: `MPL-2.0+`) | weak | no | In use (7.0.2) |
| rpds | fallback persistent collections; DOC-01 bench comparison, `amg-doc` dev-only (S01) | MIT (1.x; older releases MPL-2.0) | no | — | In use (1.2.1, dev-only) |
| skrifa 0.49.0 (`=0.49.0`, `default-features = false`, `std`; replaces unmaintained `ttf-parser`, RUSTSEC-2026-0192) | `amg-sketch::text` glyph outlines and metrics (S03, SK-10); Google Fonts "fontations". Pulls `read-fonts` 0.46.0, `font-types` 0.12.7 (both MIT OR Apache-2.0) and `bytemuck` (already in the tree) | MIT / Apache-2.0 | no | no | In use (0.49.0) |
| dxf | DXF import, `amg-sketch::import` (S03 SK-11); pulls `image`, `chrono`, `uuid`, `num`, `encoding_rs` ((Apache-2.0 OR MIT) AND BSD-3-Clause), all permissive | MIT | no | no | In use (0.6.2) |
| usvg | SVG import, `amg-sketch::import` (S03 SK-11); `default-features = false` (no text/font/raster stack); pulls `roxmltree`, `svgtypes`, `kurbo`, `simplecss`, `tiny-skia-path` (BSD-3-Clause), all permissive | MIT / Apache-2.0 (>= 0.38; older releases MPL-2.0) | no | no | In use (0.48.1) |
| criterion, proptest | benches and property tests (dev-only) | MIT / Apache-2.0 | no | no | proptest in use (1.12.0, dev-only); criterion in use (0.8.2, dev-only) |
| blake3 | cache keys and manifest hashes, `amg-doc` (S01) | CC0-1.0 OR Apache-2.0 OR Apache-2.0 WITH LLVM-exception | no | no | In use (1.8.7) |
| serde_path_to_error | `.amg` load diagnostics, `amg-doc` (S01) | MIT / Apache-2.0 | no | no | In use (0.1.20) |
| crossbeam-channel | job results, `amg-jobs` (S00) | MIT / Apache-2.0 | no | no | In use (0.5.17) |
| parking_lot | locks, `amg-jobs` (S00) | MIT / Apache-2.0 | no | no | In use (0.12.5) |
| bytemuck | GPU buffer casts, `amg-jobs::gpu` (S00); bulk FFI buffer casts, `amg-brep` (S02) | Zlib / MIT / Apache-2.0 | no | no | In use (1.25.2; `amg-jobs::gpu`, feature `gpu`) |
| pollster | `Gpu::new_blocking`, `amg-jobs::gpu` (S00) | MIT / Apache-2.0 | no | no | In use (1.0.1; `Gpu::new_blocking` in `amg-jobs::gpu` and the `amg-testkit` GPU probe) |
| tracing-appender | rolling log files, `amg-jobs::diag` (S00) | MIT | no | no | In use (0.2.5) |
| tracing-tracy (+ tracy-client 0.19.0, tracy-client-sys 0.30.1 which builds the bundled Tracy C++ client) | optional profiler layer, `amg-jobs::diag` (S00); its default features minus `crash-handler` (the diag panic hook and FND-12 own crash handling) | tracing-tracy, tracy-client: MIT / Apache-2.0; tracy-client-sys: (MIT OR Apache-2.0) AND BSD-3-Clause | no | yes (`tracy` on `amg-jobs` / `amg-cli`) | In use (0.12.0, feature-gated) |
| clap | `amg-cli` (S00) | MIT / Apache-2.0 | no | no | In use (4.6.7) |
| approx | float assertions, glam `approx` feature (S00) | Apache-2.0 | no | no | In use (0.5.1, dev-only) |
| toml | golden files (`amg-testkit`), mesh corpus manifest (S00, S07); machine library files, `amg-am` runtime (S16); material libraries, `amg-materials` (S17) | MIT / Apache-2.0 | no | no | In use (1.1.8; `amg-testkit` golden files) |
| crash-handler, minidumper | native crash minidumps (S00, FND-12) | MIT / Apache-2.0 | no | no | Planned |
| smallvec | small inline vectors, `amg-sketch`, `amg-mesh`, `amg-brep` history, `amg-nurbs` (S03, S07, S02, S20) | MIT / Apache-2.0 | no | no | In use (1.16.3, `amg-sketch`) |
| fontdb | system font discovery for sketch text (S03, SK-10); not added in SK-10, which takes parsed font bytes; the UI/document layer resolves families | MIT | no | no | Planned |
| kcl-ezpz (+ faer 0.23, winnow, indexmap, libm, mutants: MIT / Apache-2.0) | sketch solver comparison, dev-only (S03, SK-13) | MIT | no | yes (`ezpz-bench`) | In use (0.2.18, dev-only, feature `ezpz-bench`) |
| robust =1.2.0 | exact orientation predicates (`orient2d`, `orient3d`), `amg-mesh::selfint` (S07, MESH-19) | MIT / Apache-2.0 | no | no | In use (1.2.0) |
| spade | 2D constrained Delaunay for caps and planar holes, `amg-mesh` (S07); section caps, `amg-features::inspect` (S05) | MIT / Apache-2.0 | no | no | In use (=2.15.1, `amg-mesh` planar hole fill, MESH-20) |
| rand, rand_chacha | seeded RANSAC and corpus generators, `amg-mesh` (S07) | MIT / Apache-2.0 | no | no | `rand_chacha` 0.9.0 In use (MESH-28 generators, `amg-mesh` feature `corpus`; `rand_core` only, no `rand`); `rand` Planned |
| sha2 | sha256 of corpus models, `amg-mesh` feature `corpus` (S07, MESH-28) | MIT / Apache-2.0 | no | yes (`corpus`, test support) | In use (0.10.9) |
| Thingi10K models (Zhou & Jacobson, arXiv:1605.04797), 165 unmodified STL files from `raw_meshes/` | **test data only**: `testdata/mesh/real/thingi10k/`, git-LFS (S07, MESH-28); only models licensed CC0 / public domain / CC-BY / CC-BY-SA, per-model licence, author, title and Thingiverse URL in `corpus.toml` and `real/thingi10k/NOTICE`; never compiled, linked or shipped | CC0-1.0, LicenseRef-PublicDomain, CC-BY-3.0, CC-BY-SA-3.0 (versions assumed, see the folder README) | share-alike (data) | no (not a dependency) | In use (test data) |
| Thingi10K NC-licensed models (8) and Stanford 3D Scanning Repository bunny, dragon, armadillo | **fetch-only test data**: downloaded by `testdata/mesh/fetch.sh` / `fetch.ps1` into the git-ignored `testdata/mesh/fetch/` (pinned URL + sha256), never committed or redistributed (S07, MESH-28) | CC-BY-NC(-SA)-3.0 (Thingi10K); Stanford's research-use terms (`LicenseRef-Stanford-3DScanRep`) | restricted (data) | no | In use (fetch-only test data) |
| oneTBB (vcpkg `tbb`) | Manifold parallel backend, `amg-manifold-sys` (S07, ADR-003) | Apache-2.0 | no | yes (`manifold`) | Planned |
| manifold-csg-sys | fallback bindings if own `amg-manifold-sys` fails (S07) | MIT / Apache-2.0 | no | yes (`manifold`) | Planned (fallback only) |
| parry3d-f64 =0.31.1 | ray cast / closest point test oracle, `amg-mesh` dev-dependency only (S07 MESH-03); never linked into shipped binaries | Apache-2.0 | no | n/a (dev-only) | In use (dev/test-only; cargo deny clean) |
| obvhs =0.4.0 (`parallel`) | PLOC BVH build benchmark reference, `amg-mesh` dev-dependency only (S07 MESH-03); never linked into shipped binaries | MIT / Apache-2.0 | no | n/a (bench-only) | In use (dev/bench-only; cargo deny clean) |
| cargo-nextest, cargo-deny | dev tools, not linked (S00) | MIT / Apache-2.0 | no | — | In use (dev tools; nextest 0.9.148, deny 0.20.2; CI) |
| curvo =0.3.3 | `amg-nurbs` differential test oracle, dev-only (S20 KH-07/12/13) | MIT | no | n/a (dev-only) | Planned (dev-only) |
| TiGL `tests/TestData/CurveNetworks` (data only, no code) | Gordon corpus under `testdata/kernel/loft/tigl/` with attribution (S20 KH-15/16) | Apache-2.0 | no | n/a (data) | Planned (test data) |
| Microsoft.Direct3D.WARP 1.0.20 (NuGet) | CI only: software D3D12 adapter for GPU tests (FND-05), never shipped | Microsoft EULA (redistributable binary) | no | n/a (CI-only download) | In use (CI) |
| Noto Sans (Regular, Bold, Italic; ASCII subsets) | **test data only**: `crates/amg-sketch/tests/fixtures/fonts/` (S03, SK-10); plain files, not LFS; licence text kept beside them; never shipped | SIL OFL 1.1 | no | n/a (data) | In use (test data) |
