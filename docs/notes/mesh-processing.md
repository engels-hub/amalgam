# Mesh processing — data layout, precision, libraries, algorithms, corpus

Design note for [S07 — Mesh workspace](../specs/S07-mesh.md). Written during S07 refinement
(2026-10-19). The spec holds the decisions and the API; this note holds the evidence and the
algorithm-level detail that implementing agents need.

## 1. Precision: f64 positions (no exception to the architecture)

**Decision:** vertex positions are `glam::DVec3` (f64, mm) in every CPU-side mesh type. The only
f32 data in `amg-mesh` is (a) BVH node bounds, which are rounded *outward* so they stay
conservative, with every exact test done in f64 against the original triangle, and (b) the
render upload buffers produced for `amg-render`, which are rebased to a body-local origin
before the f64→f32 cast. Neither stores model geometry, so ADR-006 ("f64 everywhere in the
model, f32 only on the GPU side") holds; (a) is spelled out as a clarification for ADR-006 in
the S07 refinement report.

**Evidence.**

*Precision.* f32 has a 24-bit significand. At 1000 mm from the origin (a large-format build plate
corner) one ulp is 1000·2⁻²⁴ ≈ 6·10⁻⁵ mm. `amg-core`'s default linear tolerance is 10⁻⁶ mm (S00).
With f32 storage the tolerance cannot even be represented, so weld/snap thresholds, the plane
classification in plane cut and the zero tests in self-intersection detection would give
different answers for the same part depending on where it sits on the plate. That breaks the
determinism principle in the vision doc. Iterative operations (remesh relaxation, smoothing,
repeated booleans) also add a rounding error at every write-back. Manifold itself moved to a
double-precision interchange type (`MeshGL64`) for the same reason, and the Rust bindings use
f64 by default.

*Input data is f32 anyway.* STL and most 3MF producers write f32. f32→f64 conversion is exact,
so f64 storage loses nothing on import, and export rounds once at the end.

*Memory is not the bottleneck.* For 10 M triangles (≈ 5 M vertices for a closed mesh, V ≈ F/2):

| Structure | f64 positions | f32 positions |
|---|---|---|
| positions (V × 24 B / 12 B) | 120 MB | 60 MB |
| triangles `[u32;3]` (F × 12 B) | 120 MB | 120 MB |
| face groups `u32` per face | 40 MB | 40 MB |
| **`IndexedMesh` total** | **≈ 280 MB** | **≈ 220 MB** |
| `HalfEdgeMesh` extra: twin `u32` per half-edge (3F × 4 B) + vertex→half-edge (V × 4 B) + flags | ≈ 145 MB | ≈ 145 MB |
| BVH, 4 tris/leaf, 32 B nodes (f32 bounds) + `u32` permutation | ≈ 200 MB | ≈ 200 MB |
| Fast-winding-number expansions (order 2, f64, per inner node, built lazily) | ≈ 250 MB | — |

f64 adds 60 MB (≈ 10 %) to the ≈ 625 MB working set of a 10 M-triangle mesh with half-edges
and BVH. The reference machine has ≥ 32 GB of RAM in practice. Peak memory is set by the
operations (repair keeps input + output + half-edge), and is budgeted at ≤ 2 GB for 10 M
triangles (spec §Performance targets).

*Throughput.* The hot loops (BVH traversal, closest point, QEM) are memory-bound on node and
index fetches, which f32 bounds already keep compact. Triangle-level math in f64 on AVX2 costs
about 2× peak FLOPs but is not the limiter. We accept this.

## 2. Data structures

### `IndexedMesh` (storage / interchange)
Flat arrays (`positions: Vec<DVec3>`, `tris: Vec<[u32; 3]>`) plus `MeshAttrs` (optional,
domain-tagged attribute arrays) plus `FaceGroups` (a per-face `FaceGroupId` and a group table).
This is the type that crosses crate boundaries: `amg-io` reads and writes it, `amg-doc` stores
it in `MeshBody`, `amg-implicit` meshes into it, and Manifold converts to and from it. It can
hold anything a file can hold: non-manifold edges, bow-tie vertices, duplicate and degenerate
triangles, unreferenced vertices, and triangles with repeated indices. Only `validate()`
rejects things that are *structurally* broken (an index out of range, or an attribute array of
the wrong length).

### `HalfEdgeMesh` (topology editing)
A compact **directed-edges** structure (Campagna, Kobbelt & Seidel 1998). Face `f` owns
half-edges `3f, 3f+1, 3f+2`; `next`/`prev` are index arithmetic, so the only stored per-half-edge
datum is `twin: u32` (`INVALID` on a boundary). Per vertex, one outgoing half-edge (a boundary
one if the vertex is on the boundary, so one-ring walks start correctly). Deletions are
tombstones (`alive` flags) followed by `garbage_collect()`. That is the standard approach in
OpenMesh and PMP, and it makes split/collapse/flip O(1).

The structure needs a manifold-with-boundary input. `from_indexed` takes a
`NonManifoldPolicy`:
- `Fail` returns `MeshError::NonManifold`;
- `SplitVertices` duplicates the vertices at non-manifold edges and bow-tie vertices so the
  result is manifold. Edges with more than two incident faces are cut into separate boundary
  edges, and the counts are reported. Repair (MESH-04) later re-welds what it can.

Rejected alternatives: a full pointer-style half-edge (next/prev/twin/vertex/face per
half-edge, ≈ 20 B/half-edge, 600 MB at 10 M tris), and a corner table (equivalent to directed
edges; directed edges are what our ops are written against in the literature).

### Attributes
- *Vertex domain:* normals (`DVec3`) and colours (`Rgba8`).
- *Corner domain* (per triangle corner): UVs (`[DVec2; 3]`), because OBJ/3MF texture seams are
  per corner.
- *Face domain:* 3MF property references (`TriProperty { group, index: [u32; 3] }`) and face
  groups.

Every op documents its attribute policy:
- vertex attributes are interpolated barycentrically at new vertices (split, remesh,
  iso-extraction re-projection);
- corner attributes are interpolated inside the source face;
- face attributes and face groups are inherited from the parent face. Faces created by hole
  fill or cap get a fresh group with `GroupOrigin::Fill` / `Cap`.

Ops that cannot preserve an attribute (for example decimation with `corner_uvs` and
`preserve_attributes = false`) drop it and say so in their report.

### Face groups and persistent naming
`FaceGroupId(u32)` is allocated from a monotonically increasing counter stored in `FaceGroups`
and is never reused within one body lineage. S01 says mesh `EntityRef`s are "face-group ids +
fingerprints". `GroupFingerprint` (area, centroid, mean normal, AABB, fitted primitive kind) is
the fallback match when a group id disappears; for example, a remesh that merges groups.

Ops propagate group ids from parent faces, so a fillet-like downstream feature that references
"group 17" still finds it after repair, remesh, decimate or smooth.

Booleans map groups through Manifold's `runOriginalID` / `faceID` provenance arrays: operand A's
group ids are kept, and operand B's groups are re-allocated into A's id space with
`GroupOrigin::Boolean { operand: 1, source }`.

## 3. Library evaluation (BVH and geometry processing)

Licences per ADR-012.

| Crate | Licence | What it gives | Verdict |
|---|---|---|---|
| `obvhs` 0.4 (2026-10) | MIT / Apache-2.0 | Very fast PLOC BVH2 + CWBVH (BVH8) builders, parallel reinsertion, rayon `parallel` feature, SIMD CPU traversal; glam 0.33–0.34 | **Benchmark reference only.** f32-only, ray-tracing-oriented: no closest-point, no node payloads for winding-number expansions, no tri–tri pair traversal. Its PLOC builder is the bar MESH-03's binned-SAH builder must come within 2× of. |
| `parry3d-f64` 0.31 (2026-09) | Apache-2.0 | `TriMesh` with BVH, ray cast, point projection, pseudo-normals; `parallel` feature; now on `glamx` | **Test oracle (dev-dependency) only.** Pulls a whole physics-geometry stack and its own math types, has no fast winding number, and we need control over memory layout and determinism. Useful to cross-check ray cast and closest point in tests. |
| `bvh` (svenstaro) | MIT | Generic SAH BVH over nalgebra | Rejected: nalgebra types, no closest point / FWN, slower builds than obvhs. |
| `baby_shark` 0.3.12 (2025-09) | MIT | Pure-Rust corner table, isotropic remesh, edge-collapse decimation, voxel booleans/offset | **Reference only.** nalgebra 0.34 + nalgebra-glm (marked obsolete), no attribute/face-group propagation, no hole filling, release cadence has stalled. Reading its incremental remesher alongside Botsch–Kobbelt is useful. |
| `remesh` 0.0.x | MIT / Apache-2.0 | Isotropic remeshing | Rejected: pre-release, no feature preservation. |
| `meshopt` (meshoptimizer bindings) | MIT | Very fast simplification, vertex cache optimisation | **Not for modeling** (f32, no f64 error metric, no face-group borders). Recommended to S10 for render LODs instead. |
| `lin-alg` | MIT | Small f32/f64 vector/quaternion library | Not needed: `glam` (ADR-006) covers it. |
| **`robust`** (georust) | MIT / Apache-2.0 | Shewchuk's adaptive exact `orient2d/orient3d/incircle/insphere` | **Adopt.** Exact predicates for self-intersection detection, plane-cut classification and hole-fill validity. |
| **`spade`** | MIT / Apache-2.0 | Robust 2D constrained Delaunay triangulation with exact predicates | **Adopt** for plane-cut caps (polygons with holes) and planar hole fills. |
| **`smallvec`** | MIT / Apache-2.0 | Inline small vectors | **Adopt** for one-rings and candidate lists. |
| **`rand` + `rand_chacha`** | MIT / Apache-2.0 | Seeded, platform-stable RNG | **Adopt** for RANSAC (MESH-24) and synthetic corpus generation. ChaCha gives the same sequence on every platform, which determinism needs. |
| `faer` | MIT | Sparse Cholesky / LU | Already planned (ADR-006). Used for Liepa fairing and implicit Laplacian smoothing. |
| `nalgebra` | Apache-2.0 | Small dense LSQ | Already planned. Used for primitive-fit Gauss–Newton (MESH-24). |

**Conclusion:** write our own BVH, half-edge, repair, remesh, decimation and smoothing on top of
`glam` + `rayon` + `robust` + `spade`, as ADR-003 already says for everything except booleans.
The algorithms are well documented, and owning them is what lets us guarantee f64, determinism,
cancellation, attribute and face-group propagation, and the memory layout above. The amount of
code is moderate: BVH + queries + FWN ≈ 2 k lines, half-edge ≈ 1.5 k, the rest ≈ 1–2 k per op.

## 4. Manifold integration (MESH-05, gated by ADR-003)

*Upstream.* Manifold (Apache-2.0) ships an optional C binding (`MANIFOLD_CBIND=ON`,
`manifoldc`), an optional TBB parallel backend (`MANIFOLD_PAR=ON`; the bundled TBB is
discouraged because a second statically linked TBB causes crashes), a vcpkg port (externally
maintained), `MeshGL64` (double precision) and `LevelSet` (SDF meshing).

*Existing Rust bindings.*
- `manifold3d` 0.4.1 (2026-09) is a facade over `manifold-csg` / `manifold-csg-sys`,
  MIT/Apache-2.0. It provides raw FFI to the C API, f64 `MeshGL64` by default, TBB `parallel`
  on by default, and "Windows tested". Its build script **clones Manifold with git and builds
  it with CMake on first build**, unless `MANIFOLD_CSG_LIB_DIR` points at a prebuilt
  `libmanifoldc` / `libmanifold`.
- `manifold-rs` 0.6 (WilstonOreo) wraps the C++ API through its own glue and says it is a work
  in progress.

*Decision.* Own thin `amg-manifold-sys` crate (hand-written `extern "C"` declarations for the
≈ 30 `manifoldc` functions we call), with Manifold + oneTBB built by **vcpkg in manifest mode**
like OCCT (S00 FND-01). If the upstream vcpkg port does not enable `MANIFOLD_CBIND`, add an
overlay port in `vcpkg/ports/manifold`.

Reasons:
1. No network fetch or CMake run hidden in `build.rs`. CI caches vcpkg binaries once (FND-05),
   the same story as OCCT.
2. The Manifold version is pinned in `vcpkg.json` next to OCCT. A third-party sys crate pins
   its own upstream tag, and a mismatch between that tag and a prebuilt library is an ABI
   hazard with raw FFI.
3. It matches ADR-002's "own the bindings" stance. The C surface is small and stable.

`manifold-csg-sys` stays the reference implementation for the declarations and for the
`catch_unwind` pattern around the level-set callback. The MESH-05 spike (first half-day)
re-checks this: if the vcpkg route fails on MSVC, the fallback is `manifold-csg-sys` with
`MANIFOLD_CSG_LIB_DIR` pointing at a CMake build of a pinned tag.

*Build flags.* `MANIFOLD_CBIND=ON`, `MANIFOLD_PAR=ON` (vcpkg `tbb`), `MANIFOLD_CROSS_SECTION=OFF`
(avoids Clipper2; we do 2D in `amg-sketch`), Python/JS bindings off, static CRT matched to the
Rust MSVC target (`/MD`). Licences: Manifold Apache-2.0, oneTBB Apache-2.0.

*Data path.* `IndexedMesh` → `MeshGL64` (interleaved `vertProperties` with 3 position channels,
plus optional normal channels so Manifold interpolates them), with `faceID` = face index and
`runOriginalID` per operand. Manifold `Status` is checked, and any non-`NoError` status maps to
`MeshError::Boolean { status }`. The result is `MeshGL64` → `IndexedMesh`, and groups are
mapped back through `faceID`.

Manifold rejects non-manifold input (`NotManifold`), so `boolean()` first runs `analyze`. If an
operand is not closed and manifold, it runs `repair` when `BoolOpts::repair_inputs` is set,
otherwise it returns `MeshError::NotClosed` / `NonManifold`.

*Cancellation.* Manifold ops are not interruptible. We check `ctx` before conversion, before the
boolean and after. A 1 M × 1 M boolean is budgeted at < 1 s, so this is acceptable. Booleans of
10 M-triangle inputs are documented as "cancel takes effect when the current boolean finishes".

*Determinism.* MESH-05 adds a test that runs the same boolean 10× with TBB enabled and asserts
bit-identical `MeshGL64` output. If it is not bit-identical, the fallback is to sort the output
canonically (by face group, then lexicographic triangle). The result is recorded in the spec.

*Feature flag.* `amg-mesh/manifold` (default on). With it off, `boolean()` returns
`MeshError::BackendUnavailable`. That keeps the crate building before ADR-003 is accepted and on
machines without vcpkg.

*If ADR-003 is rejected.* The fallback is an SDF boolean through MESH-17 (approximate, always
manifold, loses sharp edges unless the result is remeshed with feature snapping), plus a later
exact-arrangement boolean in the style of Zhou et al. 2016 (*Mesh Arrangements for Solid
Geometry*, SIGGRAPH) as its own WP. The SDF boolean cannot meet the 1 M × 1 M < 1 s budget at
fine resolution.

## 5. Algorithms

### BVH (MESH-03)
- **Build:** binned SAH (Wald 2007, *On fast construction of SAH-based BVHs*), 16 bins, top
  levels split in parallel with `rayon::join`, leaves ≤ 4 triangles. Nodes are flattened
  depth-first, 32 B each (`min: [f32;3], left_or_first: u32, max: [f32;3], count: u32`), and
  bounds are rounded outward with `next_after`.
- **Ray traversal:** robust slab test with ulp padding (Ize 2013, *Robust BVH ray traversal*);
  watertight ray–triangle test in f64 (Woop, Benthin & Wald 2013).
- **Closest point:** best-first traversal using squared distance to the node box; point–triangle
  closest point per Ericson (*Real-Time Collision Detection* §5.1.5). It returns the closest
  sub-feature (vertex/edge/face) so the sign can use angle-weighted pseudo-normals (Bærentzen &
  Aanæs 2005) on clean meshes.
- **Fast winding number:** Barill, Dickson, Schmidt, Levin & Jacobson 2018, *Fast Winding Numbers
  for Soups and Clouds* (ACM TOG 37(4)). Per inner node it stores a dipole expansion up to order
  2 about the area-weighted centroid. The far-field test is ‖p − c‖ > β·r with β = 2 by default
  (the paper's accuracy/speed sweet spot). Near-field uses exact solid angles (Van Oosterom &
  Strackee 1983). Generalized winding number semantics follow Jacobson, Kavan & Sorkine-Hornung
  2013. The tree is built lazily (`OnceLock`) the first time a winding number is asked for.

### Self-intersection detection (MESH-19)
BVH self-traversal yields candidate triangle pairs whose f32 boxes overlap. Each pair gets an
exact tri–tri test using `robust::orient3d` sign tests (Guigue & Devillers 2003 structure,
Möller 1997 as reference). Pairs that share an edge or a vertex are tested for the
"non-trivial" contact only:
- coplanar overlap beyond the shared edge;
- a fold-through at a shared vertex.

Parallel over the top-level node pairs. Output is sorted for determinism.

### Hole filling (MESH-20) — Liepa 2003, *Filling Holes in Meshes* (SGP)
1. Triangulate each boundary loop by dynamic programming over the min-weight triangulation, with
   weight = (max dihedral angle, area) compared lexicographically. Cost is O(n³), so loops longer
   than `max_dp_edges` (default 250) are first split by a shortest geodesic-ish chord, or planar
   loops are handed to `spade` CDT.
2. Refine: split triangles and relax edges until edge lengths match the boundary's average
   ("density factor" √2 per Liepa).
3. Fair: solve the harmonic or bi-harmonic (umbrella²) system for the interior patch vertices
   with `faer` sparse Cholesky. Boundary rings are fixed for C¹-ish continuity.

New faces get a fresh `GroupOrigin::Fill` group.

### Self-intersection removal (MESH-21)
Iterative local repair in the style of Attene 2010 (*A lightweight approach to repairing
digitized polygon meshes*, The Visual Computer; MeshFix is GPL, so it is used as a reference
only):
1. Remove intersecting triangles plus a k-ring (k grows 1→3 across rounds).
2. Fill the resulting holes (MESH-20).
3. Re-detect.

Stop after `max_rounds`. Anything left is reported as unresolved, and the user is pointed to
the Wrap repair (MESH-18).

### Repair pipeline (MESH-04)
Fixed order, every step reported:
1. weld (quantize-and-sort on positions; parallel radix sort of 64-bit cell keys; then exact
   distance check against neighbours within `weld_tolerance`);
2. drop degenerate triangles (repeated index, or area < `degenerate_area`) and duplicate
   triangles (same vertex set, either orientation);
3. resolve non-manifold edges and vertices (`SplitVertices`, then keep the two faces with the
   most consistent orientation on each over-full edge);
4. drop tiny shells (faces/volume below threshold);
5. orient consistently per shell (BFS flip), then outward per closed shell (signed volume > 0);
   open shells use the winding number of an offset sample point;
6. fill holes (MESH-20);
7. remove self-intersections (MESH-21);
8. re-weld and re-analyse.

Repeat steps 3–8 up to `max_rounds` (default 3) while defects decrease. Steps 1–5 are the PMP /
MeshLab "clean" set. The order follows MeshFix.

### Isotropic remesh (MESH-06) — Botsch & Kobbelt 2004, *A Remeshing Approach to Multiresolution Modeling* (SGP)
Per iteration:
1. split edges > 4/3·L;
2. collapse edges < 4/5·L (link condition, and no new edge > 4/3·L);
3. flip to make valence closer to 6 (4 on the boundary);
4. tangential relaxation (area-weighted centroid projected to the tangent plane);
5. project onto the *input* surface via `MeshSpatial::closest_point`.

Default is 5 iterations.

*Feature edges* are edges with dihedral angle > `feature_angle` (default 45°), plus group borders
and boundaries when requested. They may be split; they collapse only along the feature; they
never flip; feature vertices relax only along their feature curve; corners (≠ 2 feature edges)
stay fixed.

### Adaptive remesh (MESH-22)
The sizing field comes from max principal curvature κ (Meyer et al. 2003 discrete operators,
smoothed). Target length is L(v) = √(6ε/κ − 3ε²), clamped to [L_min, L_max], following Dunyach,
Vanderhaeghe, Barthe & Botsch 2013 (*Adaptive Remeshing for Real-Time Mesh Deformation*,
Eurographics short). Split/collapse thresholds use the mean of the two endpoint sizes.

### QEM decimation (MESH-07) — Garland & Heckbert 1997, *Surface Simplification Using Quadric Error Metrics* (SIGGRAPH)
- Edge-collapse with an optimal placement (3×3 solve, falling back to the endpoint or midpoint
  when the solve is ill-conditioned).
- Boundary and group-border preservation through penalty quadrics on perpendicular planes, with
  weight `boundary_weight`. Feature edges get the same penalty.
- Reject a collapse if any adjacent face normal flips (> 90°), if triangle quality drops below
  `min_triangle_quality`, or if the link condition fails.
- Priority queue with deterministic tie-break (cost, then min vertex id, then max vertex id).
  It is sequential; meshoptimizer shows single-threaded QEM at ≥ 5 M tris/s is achievable.
- Optional attribute-aware quadrics (Hoppe 1999, *New quadric metric for simplifying meshes with
  appearance attributes*) only when `corner_uvs`/colours exist.

### Smoothing (MESH-08)
- Laplacian with uniform or cotangent weights (Desbrun et al. 1999), explicit, step λ.
- Taubin λ|μ (Taubin 1995, *A Signal Processing Approach to Fair Surface Design*, SIGGRAPH),
  defaults λ = 0.5, μ = −0.53 for ≈ zero shrinkage.
- Feature-preserving smoothing: bilateral normal filtering (Zheng, Fu, Au & Tai 2011, *Bilateral
  Normal Filtering for Mesh Denoising*, IEEE TVCG), then vertex update from the filtered normals.
  Fleishman, Drori & Cohen-Or 2003 is the simpler alternative and the fallback.

### Isosurface extraction (MESH-17)
**Marching tetrahedra on a sparse block grid.** Each cube cell is split into 6 tetrahedra with
the same diagonal orientation everywhere (Freudenthal / Kuhn subdivision), so shared faces are
cut consistently. Values exactly equal to the iso level are treated as "outside" (symbolic
perturbation). That makes every output vertex lie on a unique grid edge, and the output is a
closed, oriented 2-manifold for **any** input field. Samples outside `bounds` are forced outside,
which closes the surface. This is the same guarantee Manifold's `LevelSet` gets from marching
tetrahedra on a BCC lattice; plain marching cubes can produce non-manifold vertices.

*Sparse.* An octree over blocks of 16³ cells, refined only where
`ScalarField::may_cross(aabb, iso)` is true. For a 1-Lipschitz SDF that test is
`|f(centre)| ≤ half-diagonal`. Blocks are evaluated in parallel. Output vertices are deduplicated
by grid-edge key, so the output is deterministic regardless of thread count.

Marching tets produce slivers, so callers usually follow with `decimate(MaxError)` and/or
`remesh`. Vertices can optionally be projected onto the zero level with 2–3 secant steps.

`amg-implicit`'s dual contouring (IMP-04, sharp features, GPU) is a separate, higher-quality
mesher. MESH-17 exists so Phase 1 has a guaranteed-manifold CPU mesher without waiting for
IMP-02 / IMP-04.

### Shell / offset (MESH-10)
- **Quick:** move vertices along angle-weighted vertex normals by `thickness`, duplicate with
  reversed orientation, and stitch the open boundaries with a strip. It is fast, but can
  self-intersect when |d| exceeds the local feature size. The report counts self-intersections
  (MESH-19), and the UI suggests "robust".
- **Robust:** let s(p) be the signed distance of the input (`amg_implicit::MeshSdf`, IMP-03;
  sign from the fast winding number, so it works on slightly dirty input). For an inward shell
  of thickness t the field is f(p) = max(s(p), −(s(p) + t)). Outward and both-sided shells are
  analogous. Extract with MESH-17 at `voxel_size`, then decimate to `voxel_size/4` tolerance.
  The output is always closed and manifold, with no self-intersections, because a single
  marching-tets surface of a field cannot self-intersect.

### Wrap repair (MESH-18)
Wrap is a morphological closing (dilate, then erode) done in two steps:
1. **Dilate.** Use the field f₁(p) = u(p) − r, where u is the *unsigned* distance to the input
   (needs only closest point, so any soup works) and r = `wrap_offset` (default 1.5 × voxel
   size). Extract it with MESH-17 and keep the outermost component S₁ (largest |volume|), which
   discards inner offset surfaces. S₁ is clean and closed by construction.
2. **Erode.** Use f₂(p) = s₁(p) + r, where s₁ is the signed distance to S₁ (reliable, because
   S₁ is clean). Extract with MESH-17, then decimate.

The result is a watertight, manifold envelope for any soup, including hopeless scans. Detail
below about r (gaps, slots, small holes) closes over.
Reference: Portaneri, Rouxel-Labbé, Hemmer, Cohen-Steiner & Alliez 2022, *Alpha Wrapping with
an Offset* (ACM TOG / CGAL 3D Alpha Wrapping, GPL). The algorithm is used as a reference only;
our version is SDF-based.

### Face groups and primitives (MESH-12, MESH-24)
- **Region growing:** seeds in order of lowest local curvature. Grow while the neighbour's normal
  deviates < `angle_threshold` from the region's running fitted normal, and curvature varies
  < `curvature_threshold`. Then merge small regions into their best neighbour.
- **Primitive fitting:** Efficient RANSAC (Schnabel, Wahl & Klein 2007, *Efficient RANSAC for
  Point-Cloud Shape Detection*, CGF) per group over vertex samples with normals. Types are
  plane, sphere, cylinder, cone and torus. Refine with Gauss–Newton geometric least squares
  (Lukács, Martin & Marshall 1998, *Faithful least-squares fitting of spheres, cylinders, cones
  and tori*). Seeded `ChaCha8Rng` keeps it deterministic.

### Wall thickness (MESH-11)
Per vertex, cast a cone of rays (default 1 + 8 rays, half-angle 15°) along the inward normal.
Thickness is the minimum hit distance, ignoring hits on the vertex's own 1-ring. The GPU path
uploads the flattened BVH and triangles (f32, rebased to the body origin) and traverses in WGSL
(stack-based); the CPU path uses `MeshSpatial::ray_cast`. The two must agree within
1e-3 · bbox diagonal. Reference: the shape-diameter function (Shapira, Shamir & Cohen-Or 2008)
for the cone idea.

### Plane cut and section
Classify vertices with a signed distance to the plane in f64. Vertices with
|d| ≤ `snap_tolerance` (default `Tolerance::linear`) snap onto the plane, so cuts through
existing vertices and edges create no slivers. Split edges, then extract the boundary loops on
the plane (closed polylines). Each set of nested loops (classified by 2D winding) is capped
with a `spade` CDT. The same intersection walk gives `section()` polylines, which MESH-14 and
INS-02 use.

## 6. Broken-mesh test corpus (MESH-28)

**Layout.**
```
testdata/mesh/
  corpus.toml               # manifest: one [[mesh]] entry per file (see below)
  real/<source>/<id>.<stl|ply|3mf|obj>   # committed via git-LFS (redistributable only)
  fetch/                    # non-redistributable sets: not committed; fetched by script
  fetch.ps1, fetch.sh       # download + sha256 verify into testdata/mesh/fetch/ (git-ignored)
```
LFS patterns: `testdata/**/*.stl`, `*.ply`, `*.3mf`, `*.obj` (FND-06 owns `.gitattributes`;
MESH-28 adds patterns additively). Budget is ≤ 300 MB committed and ≤ 20 MB per file.
Benchmark meshes of 1 M and 10 M triangles are **generated** deterministically, not stored.

**Manifest entry.**
```toml
[[mesh]]
id = "thingi10k-37266"
file = "real/thingi10k/37266.stl"     # or fetch/… for fetch-only entries
source_url = "https://ten-thousand-models.appspot.com/detail.html?file_id=37266"
license = "CC-BY-4.0"                 # SPDX id; attribution fields below when required
author = "…"
sha256 = "…"
faces = 12345
defects = ["holes", "non_manifold_edges", "self_intersections"]  # ground truth from MESH-02 + manual check
expect_repairable = true              # false ⇒ excluded from the 95 % watertight target (wrap only)
```

**Sources.**
1. **Thingi10K** (Zhou & Jacobson 2016, arXiv:1605.04797): 10 000 Thingiverse models with
   per-model licence and defect metadata (manifold, self-intersecting, components, degenerate
   faces), so it can be filtered by defect class. Select ≈ 150 models stratified across defect
   categories, restricted to CC0 / public-domain / CC-BY / CC-BY-SA, with attribution recorded.
   NC/ND-licensed models are fetch-only. The `thingi10k` Python package can download by id, so
   `fetch.ps1` can be generated.
2. **Stanford 3D Scanning Repository** (bunny raw scans with real holes, dragon, armadillo;
   Lucy as an optional large mesh): research-use terms, so **fetch-only**, never committed.
3. **Owner's own failing STLs/3MFs** (see spec open question). Committed if the owner agrees.
4. **Synthetic defect generator** (`amg-mesh` test utility, `ChaCha8Rng`-seeded). Base shapes:
   icosphere, torus, box with through-holes, torus knot, displaced sphere. Injected defects, each
   with known ground truth:
   - holes (remove k faces; boundary-loop sizes 3…2000);
   - flipped faces;
   - duplicate faces;
   - zero-area and needle slivers;
   - unwelded seams with jitter up to 1e-4 mm (T-junction cracks);
   - non-manifold fins and bow-tie vertices;
   - overlapping shells (self-intersection);
   - inverted shells;
   - tiny floating shells;
   - f32-rounded coordinates at ±1000 mm.

   Generator output is not stored. It is reproducible from `(shape, defects, seed)`.
5. **Benchmark generator:** displaced subdivided icosphere at 100 k / 1 M / 10 M triangles, and
   two offset spheres for 1 M × 1 M booleans (the vision-doc scenario).

Defect categories used for stratification: holes, non-manifold edges, non-manifold vertices,
inconsistent orientation, inverted, degenerate, duplicate, self-intersections, multiple shells,
unwelded, "hopeless" (scan soups → wrap only).

## 7. References (consolidated)
- Barill, Dickson, Schmidt, Levin, Jacobson. *Fast Winding Numbers for Soups and Clouds.* ACM TOG 37(4), 2018.
- Bærentzen, Aanæs. *Signed distance computation using the angle weighted pseudonormal.* IEEE TVCG 11(3), 2005.
- Botsch, Kobbelt. *A Remeshing Approach to Multiresolution Modeling.* SGP 2004.
- Botsch, Kobbelt, Pauly, Alliez, Lévy. *Polygon Mesh Processing.* AK Peters 2010 (and PMP library, MIT).
- Campagna, Kobbelt, Seidel. *Directed Edges — A Scalable Representation for Triangle Meshes.* JGT 3(4), 1998.
- Dunyach, Vanderhaeghe, Barthe, Botsch. *Adaptive Remeshing for Real-Time Mesh Deformation.* Eurographics 2013 short papers.
- Garland, Heckbert. *Surface Simplification Using Quadric Error Metrics.* SIGGRAPH 1997.
- Guigue, Devillers. *Fast and robust triangle-triangle overlap test using orientation predicates.* JGT 8(1), 2003.
- Hoppe. *New quadric metric for simplifying meshes with appearance attributes.* IEEE Vis 1999.
- Ize. *Robust BVH Ray Traversal.* JCGT 2(2), 2013.
- Jacobson, Kavan, Sorkine-Hornung. *Robust Inside-Outside Segmentation using Generalized Winding Numbers.* SIGGRAPH 2013.
- Liepa. *Filling Holes in Meshes.* SGP 2003.
- Attene. *A lightweight approach to repairing digitized polygon meshes.* The Visual Computer 26, 2010 (MeshFix).
- Portaneri et al. *Alpha Wrapping with an Offset.* ACM TOG 41(4), 2022.
- Schnabel, Wahl, Klein. *Efficient RANSAC for Point-Cloud Shape Detection.* CGF 26(2), 2007.
- Shewchuk. *Adaptive Precision Floating-Point Arithmetic and Fast Robust Geometric Predicates.* DCG 18, 1997.
- Taubin. *A Signal Processing Approach to Fair Surface Design.* SIGGRAPH 1995.
- Wald. *On fast Construction of SAH-based Bounding Volume Hierarchies.* IEEE RT 2007.
- Woop, Benthin, Wald. *Watertight Ray/Triangle Intersection.* JCGT 2(1), 2013.
- Zheng, Fu, Au, Tai. *Bilateral Normal Filtering for Mesh Denoising.* IEEE TVCG 17(10), 2011.
- Zhou, Grinspun, Zorin, Jacobson. *Mesh Arrangements for Solid Geometry.* SIGGRAPH 2016.
- Zhou, Jacobson. *Thingi10K: A Dataset of 10,000 3D-Printing Models.* arXiv:1605.04797, 2016.
- Manifold: github.com/elalish/manifold (Apache-2.0); Rust bindings `manifold3d`/`manifold-csg` (MIT/Apache-2.0), `manifold-rs`.
- Crates: obvhs, parry3d-f64, baby_shark, robust, spade (docs.rs / lib.rs pages, checked 2026-10-19).
