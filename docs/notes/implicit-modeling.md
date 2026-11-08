# Implicit modeling: SDF graph, evaluation, meshing and lattices

Design note for S08 (`docs/specs/S08-implicit-lattice.md`). Written by the S08 refinement
(2026-11-08). Needed before IMP-01 (§1–§3), IMP-02 (§3–§4), IMP-04 (§6) and IMP-06 (§7).

## 0. Prior art

| Source | What it does | What we take | What we reject |
|---|---|---|---|
| Autodesk Fusion, Volumetric Lattice (Design Extension; Autodesk forum preview thread 2022, AU 2022 class CP502032) | Turns a body into a "volumetric model". A library of cell types (beam and surface cells, plus custom cells), cell size, orientation, beam thickness or solidity, a skin (shell) on the outer faces, solid "offset regions" near chosen faces, and variable density (gradients between faces or along a path). The result is shown as a preview and becomes a mesh for export. | The feature shape: one command on a body, the cell library, cell size and rotation, thickness, skin with open faces, solid regions, grading. The lattice stays implicit until something needs a mesh. | Making the lattice a separate document type. In Amalgam it is an ordinary implicit body in the timeline. |
| nTop (nTopology) | Field-driven design: everything is a field. Walled TPMS, graph (beam) lattices from unit cells, conformal lattices, ramp/remap blocks, fields from FEA and images. | Field nodes plus ramp/remap that drive thickness. Strut lattices as explicit beam graphs. Conformal lattices as a later step. | A visual node editor. The timeline features build the graph; scripts (S18) can build any graph. |
| `fidget` (Rust, MPL-2.0, Matt Keeter; 0.4.x) | Closed-form implicit expressions. Interval-based tape simplification, an x86_64/aarch64 JIT, point, interval, SIMD and gradient evaluation, its own Manifold Dual Contouring mesher (its README says the mesher is "much less battle-tested than libfive"). f32 only. | Its ideas: tapes, interval pruning, tape shortening. A differential-test oracle and benchmark for the closed-form subset of our graph (dev-dependency behind `fidget-bench`). | A runtime dependency (§2, decision). |
| `libfive` (C++, MPL-2.0 kernel, Keeter) | Same model as fidget, plus an octree dual contouring mesher with QEF vertex placement and a "simplify" pass. | DC with QEF and a mass-point bias. | FFI to another C++ library. |
| Keeter, "Massively Parallel Rendering of Complex Closed-Form Implicit Surfaces" (SIGGRAPH 2020) | GPU interpreter over a tape, interval evaluation on tiles, per-tile tape shortening; real-time on large expressions. | Evidence that a WGSL *interpreter* (no per-graph shader compile) is fast enough. Tile → brick hierarchy (§4). | — |
| Ju et al., "Dual Contouring of Hermite Data" (2002); Schaefer, Ju, Warren, "Manifold Dual Contouring" (2007); Nielson, "Dual Marching Cubes" (2004) | DC keeps sharp features via QEF. Plain DC can be non-manifold. MDC and DMC put one vertex per surface patch in a cell, which makes the dual surface manifold. | One vertex per marching-cubes patch per cell, on a uniform sparse grid (no octree collapse in v1), QEF placement clamped to the cell (§6). | Octree simplification in v1. Decimation (MESH-07) does that job afterwards. |
| Hart, "Sphere Tracing" (1996) | Ray marching with step = distance / Lipschitz bound. | Preview ray marching (§8), with a per-graph Lipschitz bound. | — |
| Barill et al., "Fast Winding Numbers for Soups and Clouds" (2018) | Robust inside/outside for dirty meshes. | Already in MESH-03 (`MeshSpatial::winding_number`); IMP-03 uses it for the sign. | — |
| 3MF Beam Lattice extension | Beams `v1 v2 r1 r2 cap1 cap2` (cap = sphere, hemisphere, butt), optional balls, optional clipping mesh. | Our explicit strut representation is the same tapered-beam graph (§7.3), so IO-18 is a direct mapping. | — |

## 1. Authored vs evaluated

- Timeline features (`amg-features::implicit`) are authored state: serde structs with `Expr`
  fields and `BodyId` / `EntityRef` inputs, like the mesh features.
- `SdfGraph` is **evaluated** state. A feature builds it in `compute()`, and it lives in
  `BodyGeom::Implicit(Arc<SdfGraph>)` (ADR-014). It never holds `Expr`s: all values are already
  evaluated to f64.
- Heavy payloads (a `MeshSdf`, a sampled grid, a beam graph) are `Arc`-shared. A graph that
  composes two bodies copies the small node arenas and shares the payloads.
- The graph hash covers the canonical binary encoding. For an `Arc` payload it covers the
  payload's content hash (for example `IndexedMesh::content_hash`), not the pointer, so equal
  inputs give equal hashes across runs (S01 determinism).

## 2. Own evaluator; fidget is a test oracle

Decision (resolves S08 open question 1):
- The graph has non-closed-form nodes: mesh SDFs (a BVH query), sampled grids (FEA results,
  images), explicit beam graphs. fidget's `Context`/`Tree` only holds closed-form math, so these
  would be opaque callbacks, which defeats its JIT and interval simplification.
- fidget is f32. Our CPU path must be f64 (architecture: f32 only on the GPU).
- fidget's API changes between 0.x releases, and it has no GPU backend. We need a WGSL path
  anyway (ADR-004), so we need our own tape format. Two tape formats would be double work.
- So: own tape and evaluators. fidget is a dev-dependency behind the `fidget-bench` feature:
  IMP-01 cross-checks values and intervals of random closed-form graphs against it, and IMP-02
  benchmarks against its JIT on the CPU. This mirrors how ADR-007 treats ezpz.
- fidget is MPL-2.0. That is fine for a dev-dependency (ADR-012); list it in `THIRD_PARTY.md`.

## 3. Graph, tape and numerics

### 3.1 Nodes
An arena `Vec<Node>`; a node refers only to earlier nodes, so the graph is a DAG by
construction. Node kinds (full list in the spec Interfaces):
- **Primitives:** sphere, box (with corner radius), cylinder, capsule, cone, torus, half-space
  (plane). Exact Euclidean SDFs (Quilez's formulas), 1-Lipschitz.
- **Transforms:** rigid plus uniform scale only. Non-uniform scale breaks the distance property;
  it is refused (`ImplicitError::NonUniformScale`).
- **Booleans:** union/intersect/subtract as min/max, and smooth variants with blend radius `k`
  (polynomial smooth min, Quilez; it stays 1-Lipschitz and the blend region is bounded by `k`).
- **Offset** `d − r` and **shell** `|d + t/2| − t/2` (inside), `|d − t/2| − t/2` (outside),
  `|d| − t/2` (centred). `r` and `t` are `Scalar`s, so they can be field-driven (§9).
- **Lattices** (§7), **mesh SDF** (§5), **sampled grid** (§5), **fields** (§9).

### 3.2 Bounds and Lipschitz bound
- Every finished graph has a finite conservative AABB that contains the region `d ≤ 0`. Lattices
  and half-spaces are unbounded and must end up intersected with something bounded, otherwise
  `GraphBuilder::finish` returns `ImplicitError::Unbounded`.
- Every node carries an upper bound `L` on its Lipschitz constant. Exact SDFs have `L = 1`.
  min/max keep `max(L_a, L_b)`. A field-driven offset `d − r(p)` has `L_d + L_r`. TPMS nodes use
  the tabulated constants of §7.1. Ray marching steps by `d / L`. `may_cross` uses intervals.

### 3.3 Tape
`SdfGraph` lowers once (cached in a `OnceLock`) to a `Tape`: a register machine with SSA ops.
- Small ops: `x, y, z, const, add, sub, mul, div, neg, abs, min, max, sqrt, sin, cos, floor,
  fract, smooth_min(k)`.
- "Fat" ops keep tapes short: `primitive(kind, params)`, `tpms(kind, ...)`, `strut_periodic(cell)`,
  `beam_graph(buffer)`, `grid(buffer)`, `mesh_sdf(buffer)`, `xform(affine)`.
- Parameters live in a separate constant buffer, so editing a value (not the structure) reuses
  the tape layout and the GPU never recompiles a shader.
- Register allocation (linear scan) caps live registers at 32. Graphs that need more are split
  into passes (rare; the reference lattice graph needs fewer than 20).

### 3.4 Precision
- CPU: f64 everywhere. The CPU evaluator is the reference.
- GPU: f32, with coordinates rebased to the graph's bounds centre (like the RTE camera in S10),
  so a 200 mm part keeps about 1e-5 mm resolution. GPU vs CPU agreement is a test (IMP-02).
- Sampled bricks store `d / cell_size` as f16, clamped to ±`band / cell_size` (default ±4). That
  gives about 0.002 cell of resolution near the surface.

## 4. Evaluation pipeline (IMP-02)

1. **Tiles.** Split the bounds into 64³-cell tiles. Evaluate the tape on intervals per tile
   (GPU, one workgroup per tile). Drop tiles whose interval excludes `[-band, band]`; mark
   tiles fully inside or outside.
2. **Bricks.** For surviving tiles, interval-evaluate the 8³ bricks inside. Keep bricks that may
   cross the band.
3. **Samples.** Evaluate brick corners (9³ samples per brick, shared faces duplicated) with the
   point interpreter. Write the f16 bricks into a CMP-02 brick grid.
4. Optional (only if the budget is missed): per-tile tape shortening (MPR) — record which
   min/max branch dominates in the tile and run a shorter tape for its bricks.

The CPU fallback runs the same steps with rayon and the f64 interpreter (used with no GPU, and as
the reference). Results are bit-identical across thread counts on the CPU path.

## 5. Mesh SDF and sampled grids (IMP-03, IMP-12)

- `MeshSdf` (IMP-03, CPU): `MeshSpatial::closest_point` within the band plus the fast winding
  number for the sign (both MESH-03). Outside the band it returns `±band` with the correct sign,
  so the field stays 1-Lipschitz and `may_cross` can reject boxes farther than `band`. Sign from
  the winding number (> 0.5 inside) is robust to small holes and self-overlaps.
- The GPU cannot run BVH queries cheaply, so IMP-12 bakes a `MeshSdf` into a `SampledGrid`
  (narrow-band f16 bricks, trilinear) that becomes a graph node. The grid's interpolation error
  is bounded by the cell size; the CPU path can keep the exact `MeshSdf` for meshing.
- Solid bodies enter through the BREP-08 tessellation, then the same path.
- `MeshSdf::closest_group` returns the face group of the closest triangle. Skins with open faces
  and solid regions near faces use it (§7.4).

## 6. Meshing (IMP-04)

- **Topology.** Uniform sparse grid from §4. For each cell with a sign change, run the marching
  cubes case with one consistent face-ambiguity rule (the same "values equal to iso count as
  outside" rule as MESH-17). Put one vertex per MC patch (Nielson DMC / MDC). For each
  sign-changing edge, emit a quad over the patch vertices of its 4 cells; split it into two
  triangles along the shorter diagonal. This is the dual of a closed manifold MC surface, so it
  is closed and manifold.
- **Vertices.** Edge crossings are refined on the CPU with 2 secant steps on the f64 tape; normals
  come from the tape's forward-mode gradient. Each patch vertex minimises the QEF of its
  crossings (SVD with truncation 0.1, Lindstrom), biased to the mass point, and is clamped into
  its cell. With `sharp_features = false` the vertex is the mass point.
- **Checks and fallback.**
  - Every output passes the MESH-02 closed/manifold/oriented check.
  - If the check fails, the whole extraction reruns with MESH-17 (marching tetrahedra, guaranteed
    manifold) and `MeshReport::fallback` says why. That keeps "always watertight and manifold"
    a guarantee, not a hope.
  - Then MESH-19 counts self-intersections. Vertices of intersecting triangles are moved to their
    mass points and the check reruns once. The remaining count is reported, never hidden.
- Output goes into `IndexedMesh`. Optional MESH-07 decimation to a tolerance (default
  0.5 × cell size), because lattices produce many triangles.
- The S07 robust shell (MESH-10) and wrap (MESH-18) keep MESH-17. IMP-04 may replace it there
  later (S07 already allows that) without any `amg-mesh` change.

## 7. Lattices (IMP-06, IMP-14)

### 7.1 TPMS
With `k = 2π / a` (`a` = cell size, per axis allowed) and `(x, y, z) = k · p` in the lattice
frame:

| Kind | f |
|---|---|
| Gyroid | sin x cos y + sin y cos z + sin z cos x |
| Schwarz P | cos x + cos y + cos z |
| Diamond (Schwarz D) | sin x sin y sin z + sin x cos y cos z + cos x sin y cos z + cos x cos y sin z |
| Neovius | 3 (cos x + cos y + cos z) + 4 cos x cos y cos z |
| Lidinoid | ½ (sin 2x cos y sin z + sin 2y cos z sin x + sin 2z cos x sin y) − ½ (cos 2x cos 2y + cos 2y cos 2z + cos 2z cos 2x) + 0.15 |

(The original spec listed "Schwarz D" and "diamond" separately. They are the same surface.)

- `f` is not a distance. We use the first-order distance `d̃ = f / (k · max(|∇f|, g_floor))`,
  with the analytic gradient. `g_floor` is per kind (stops blow-up at critical points).
- **Modes:** `Sheet` (wall of thickness `t` centred on the surface: `|d̃| − t/2`), `Network`
  (solid on one side: `d̃ − c`, with `c` from a target thickness), and `NetworkInverse`.
- Each kind's Lipschitz bound of `d̃` is computed once by IMP-06 on a 256³ grid over one cell
  and stored as a constant with 10 % margin.
- Accuracy: wall thickness measured on the mesh (MESH-11) within ±5 % of `t` or ±1 cell,
  whichever is larger (IMP-06 test).

### 7.2 Periodic strut lattices
- A `UnitCell` is nodes in `[0, 1]³` plus beams. Built-ins: cubic, BCC, FCC, octet, Kelvin
  (truncated-octahedron edges), diamond. Beams lying on cell faces or edges are listed in every
  cell that touches them, so each cell's list covers its closure.
- Evaluation: cell index `floor(p / a)`, then the min over that cell's beams of the exact capsule
  (or round-cone, for tapered beams) distance. If the result minus the radius is larger than the
  distance from `p` to the cell boundary, the 26 neighbours are evaluated too. This keeps the
  value exact.
- Graded radius: a beam's end radii are the thickness field sampled at its two end nodes (global
  positions), so a graded lattice is still an exact round-cone SDF and still a 3MF beam.

### 7.3 Beam graphs (explicit struts)
- `BeamGraph { nodes, beams: [a, b, r_a, r_b], ball_radius }` is the explicit form. It is what
  IO-18 writes as a 3MF beam lattice (cap mode `sphere`; balls when `ball_radius` is set, ball
  mode `all`). This resolves S08 open question 3.
- `BeamGraph::from_periodic` instantiates a periodic lattice inside a region: beams with both ends
  inside are kept, beams crossing the boundary are cut at the crossing (bisection on the region
  SDF, 1e-4 mm), the rest are dropped. Beams shorter than 1e-3 mm are removed.
- A `BeamGraph` node evaluates through a uniform grid of beam bounding boxes (CPU and GPU).
  Stochastic (IMP-14) and conformal (IMP-09) lattices only exist as beam graphs.
- 3MF also allows unclipped beams plus a clipping mesh. IO-18 can choose that for smaller files;
  slicer support decides (S12).

### 7.4 Lattice in a body, skin, solid regions
With `s` the body SDF (negative inside), `L` the lattice and `t` the skin thickness:
- infill = `max(L, s)` (lattice ∩ body);
- skin = `max(s, −(s + t))` (inward shell of the body);
- result = `min(infill, skin, solid regions)`.
- **Open faces** (no skin on chosen faces, for powder removal or drainage): the skin term is
  dropped where `MeshSdf::closest_group` belongs to an open face.
- **Solid regions** (Fusion's offset regions, for example around bolt holes): solid where the
  distance to the chosen faces is below `r`, intersected with the body.

### 7.5 Stochastic (Voronoi) lattice (IMP-14)
- Seeds: a jittered grid at the mean cell size, seeded RNG (deterministic). A jittered grid
  bounds every Delaunay neighbourhood, which keeps the triangulation cost linear.
- Own 3D Bowyer–Watson Delaunay with `robust` predicates (already a dependency of `amg-mesh`).
  No permissive Rust 3D Delaunay crate covers it, and TetGen is AGPL.
- Voronoi edges (the duals of Delaunay faces, joining circumcentres of adjacent tets) become
  beams, clipped to the body with §7.3.

## 8. Preview ray marching (IMP-05)

- Rendering goes through the S10 `ExternalLayer` contract (`docs/notes/viewport-rendering.md`
  §14): Rgba16Float premultiplied colour, R32Float reversed-Z depth, the copied `Camera` WGSL
  layout. `amg-ui::implicit` wires the layer (resolves S08 open question 2: no change needed).
- The ray marcher reads the IMP-02 brick grid, not the tape. Empty-space skipping is a DDA over
  the brick occupancy. Inside bricks it sphere-traces trilinear samples with step `d / L`, then
  bisects 4 times. Normals come from central differences of the grid. Lighting copies S10's
  fixed studio lights (constants duplicated; `amg-implicit` cannot depend on `amg-render`).
- Two resolutions: a coarse grid (bounds diagonal / 256) is rebuilt after each edit; a fine grid
  (diagonal / 768) builds in the background and replaces it. While orbiting, the pass may run at
  half resolution and upsample.

## 9. Fields and grading (IMP-16, IMP-07)

- A `Scalar` is `Const(f64)` or `Field { field, ramp }`. Lattice thickness, shell thickness and
  offset distance are `Scalar`s.
- Field nodes:
  - distance to a body or to selected faces (from a `MeshSdf` or an SDF node);
  - an expression in `x, y, z` (mm, world frame), parsed by a small `FieldExpr` grammar
    (`+ − * / ^`, `sin cos tan sqrt abs min max clamp exp`, numbers). The feature substitutes
    parameter values before parsing, so `amg-doc` expressions are not needed per point;
  - an image projected along an axis (grey value 0..1, bilinear, clamped);
  - a sampled grid: FEA results (S13) or topology-optimisation density (S14), resampled by the
    producer onto a `SampledGrid`.
- `Ramp { in: [lo, hi], out: [lo, hi], curve: Linear | Smoothstep }` maps field values to mm.
  The ramp output is clamped, so the Lipschitz bound is finite: `L_field · |Δout / Δin|`.
- Printability: the lattice feature warns when the graded thickness goes below
  `min_feature` (default 0.4 mm, a typical FDM nozzle).

## 10. Cross-type booleans
- The architecture resolves cross-type booleans to the "lowest common" representation, usually
  mesh. For implicit bodies we propose: solid or mesh combined with implicit **stays implicit**
  (the other operand enters as a `MeshSdf`). Meshing happens once, at export or when a mesh-only
  feature reads it. A lattice would lose most of its value if every boolean meshed it at once.
  Owner decision 2026-11-09 (QUESTIONS #63); architecture §Geometry representations carries the
  exception.
- `ImplicitCombineFeature` accepts implicit, solid and mesh operands; MESH-27 combine keeps its
  "converted to mesh" warning when the target is a mesh body.

## 11. Later: conformal lattices and automated modeling
- **Conformal (IMP-09).** A `UvwMap` trait maps world points to lattice coordinates. First maps:
  cylindrical, spherical and sweep-along-a-curve. The map's Jacobian scales the Lipschitz bound.
  Struts are instantiated in UVW space and mapped back as a `BeamGraph`, so 3MF export still
  works. Body-conformal maps (volumetric parameterisation of an arbitrary body) are left for a
  later note.
- **Automated modeling (IMP-10).** Fusion generates connecting shapes between chosen faces while
  avoiding obstacle bodies. An implicit heuristic: smooth-union capsules or swept tubes between
  the connect faces, subtract obstacles with a clearance offset, then smooth. The topology
  optimiser (S14) is the heavier alternative. Decided (owner, QUESTIONS #62, 2026-11-09): the implicit
  heuristic ships first; a small topology-optimisation mode (connect faces as preserves, obstacles
  as keep-outs) is added later as an option.

## References
- fidget: https://github.com/mkeeter/fidget (README, docs.rs 0.4.x)
- libfive: https://libfive.com
- M. Keeter, "Massively Parallel Rendering of Complex Closed-Form Implicit Surfaces", ACM TOG
  39(4), 2020.
- T. Ju, F. Losasso, S. Schaefer, J. Warren, "Dual Contouring of Hermite Data", SIGGRAPH 2002.
- S. Schaefer, T. Ju, J. Warren, "Manifold Dual Contouring", IEEE TVCG 13(3), 2007.
- G. Nielson, "Dual Marching Cubes", IEEE Visualization 2004.
- J. Hart, "Sphere Tracing", The Visual Computer 12, 1996.
- G. Barill, N. Dickson, R. Schmidt, D. Levin, A. Jacobson, "Fast Winding Numbers for Soups and
  Clouds", SIGGRAPH 2018.
- I. Quilez, distance functions and smooth minimum articles, https://iquilezles.org
- 3MF Consortium, Beam Lattice Extension specification.
- Autodesk forum, "Looking for feedback on the Volumetric Lattice tool" (2022); AU 2022 class
  handout CP502032.
