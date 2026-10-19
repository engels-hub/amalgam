# S08 — Implicit (SDF) modeling & volumetric lattices

| | |
|---|---|
| **Status** | Draft |
| **Priority** | P1 (lattice is a key AM differentiator) |
| **Crates** | `amg-implicit`, `amg-features::implicit` |
| **Lane** | C |
| **Depends on** | S00 (GPU), S07 (mesh output, mesh SDF), S15 (GPU compute helpers) |
| **Parity** | Volumetric lattice (Design Extension), beyond-Fusion field-driven design |

## Summary
nTop-style implicit modeling: bodies defined as signed distance fields built from an
expression graph (primitives, mesh/B-rep SDFs, booleans, smooth blends, offsets, shells,
lattices, field-driven variation), evaluated on the GPU, rendered directly (ray marching
preview) and meshed robustly for export. Also the backbone for robust shelling, "wrap" repair
and generative-design outputs.

## Design
- **Expression graph:** nodes = primitives (sphere, box, cylinder, plane, torus), imports
  (mesh SDF, B-rep via tessellation), ops (union/intersect/subtract, smooth variants, offset,
  shell, blend), transforms, lattice nodes, field nodes (distance-to-surface, scalar fields from
  FEA results / images / expressions), remap/ramp. Graph is a DAG, hashable, serializable.
- **Evaluation:** compile graph → WGSL (or interpret via bytecode in a compute shader); interval
  arithmetic pass to prune empty regions (sparse octree/brick map). Study `fidget` (Rust, JIT +
  interval arithmetic) as CPU evaluator / reference — may adopt directly for CPU path.
- **Mesh SDF:** narrow-band exact distance via BVH (S07) + fast winding number for sign; cached
  as sparse voxel grid (brick map, f16).
- **Lattices:** TPMS (gyroid, Schwarz P/D, diamond, Neovius, Lidinoid), strut lattices (BCC,
  FCC, octet, Kelvin, cubic, diamond), stochastic (Voronoi foam); cell size, thickness, and
  orientation; conformal (UVW-mapped) lattices later. Thickness/density driven by fields.
  Skin option (keep outer shell of thickness t).
- **Meshing:** dual contouring / manifold dual contouring on sparse grid, GPU-accelerated;
  output always watertight and manifold; target resolution or tolerance. Sharp features
  preserved for imported B-rep sources.
- **Preview:** direct GPU ray marching (sphere tracing with Lipschitz bound) in the viewport
  for instant feedback; mesh only when needed (export, booleans with meshes, FEA).

## Work packages
| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| IMP-01 | Expression graph types, serialization, hashing, CPU evaluator | FND-03 | P1 | Todo | eval matches analytic SDFs |
| IMP-02 | GPU evaluator (graph → WGSL) + interval pruning, sparse brick grid | IMP-01, CMP-02 | P1 | Todo | 512³ effective eval < 1 s |
| IMP-03 | Mesh SDF (narrow band, winding-number sign) | MESH-03 | P0 | Todo | needed by MESH-10 robust shell |
| IMP-04 | Dual contouring mesher (watertight, manifold, sharp features) | IMP-02, MESH-01 | P1 | Todo | always manifold on fuzz corpus |
| IMP-05 | Viewport ray-marching preview | IMP-02, RND-03 | P1 | Todo | interactive at 1080p |
| IMP-06 | TPMS + strut + stochastic lattices with skin | IMP-02 | P1 | Todo | lattice of 100 mm part meshes < 30 s |
| IMP-07 | Field-driven variation (distance, FEA stress, expressions, images) | IMP-06, FEM-06 | P2 | Todo | density graded by stress field |
| IMP-08 | Smooth booleans, variable shells, offsets as features | IMP-04 | P1 | Todo | |
| IMP-09 | Conformal lattices (UVW-mapped to body) | IMP-06 | P2 | Todo | |
| IMP-10 | Automated modeling (connect bodies, avoid obstacles) | IMP-08 | P2 | Todo | |

## Testing
Analytic SDF tests, meshing manifold guarantees (fuzz), lattice volume fraction vs analytic,
benchmarks at multiple resolutions.

## Open questions
1. Adopt `fidget` vs own evaluator — evaluate during refinement (licence MPL-2.0, fine).
