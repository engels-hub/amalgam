# S07 — Mesh workspace

| | |
|---|---|
| **Status** | Draft |
| **Priority** | P0 (core of the owner's AM workflow) |
| **Crates** | `amg-mesh`, `amg-features::mesh`, `amg-ui::mesh` |
| **Lane** | C |
| **Depends on** | S00; S01 for features; S02 for mesh→B-rep |
| **Parity** | Mesh workspace section |

## Summary
High-performance triangle mesh processing in Rust: data structures, spatial queries,
analysis, repair, remeshing, decimation, smoothing, booleans (Manifold), cutting, shelling,
face groups, and conversion to B-rep. Mesh bodies are first-class bodies in the design and
every operation is a timeline feature (or direct edit).

## Design
- **Data structures:** `IndexedMesh` (positions f64 or f32? → f64 positions, u32 indices,
  optional per-vertex normals/colors/uv, per-face group ids) as the storage/interchange format;
  `HalfEdgeMesh` built on demand for topology-changing ops. Non-manifold input must load
  (repair operates on `IndexedMesh` + soup).
- **Spatial:** BVH (SAH, flattened, parallel build) for ray casts, closest point, self-intersection,
  inside/outside (winding number, fast generalized winding number for dirty meshes).
- **Analysis:** watertight/manifold check, boundary loops, self-intersections, inverted normals,
  disconnected shells, wall thickness map (ray-based, GPU), volume/area.
- **Repair:** weld vertices, remove degenerate/duplicate faces, orient consistently, fill holes
  (minimal-area + fairing, Liepa), remove self-intersections (via Manifold / local remesh),
  "wrap" repair (SDF/voxel shrinkwrap through S08 when topology is hopeless).
- **Remesh:** isotropic explicit remeshing (Botsch–Kobbelt: split/collapse/flip/relax), adaptive
  variant (curvature-driven target length), feature-edge preservation.
- **Reduce:** QEM decimation (Garland–Heckbert) with boundary/feature preservation, target by
  count/ratio/tolerance.
- **Smooth:** Laplacian, Taubin, bilateral/feature-preserving.
- **Booleans (ADR-003):** Manifold; input must be manifold → auto-repair attempt otherwise.
- **Plane cut** (with cap fill), **shell** (offset via SDF for robustness, or vertex-normal offset
  for thin quick shells), **separate shells**, **erase & fill**, **reverse normals**.
- **Face groups:** region growing by normal deviation / curvature / fitted primitive (plane,
  cylinder, sphere, cone via RANSAC + least squares refine).
- **Mesh→B-rep:** faceted (each triangle a face, merged coplanar), prismatic (face groups →
  fitted analytic surfaces → trimmed B-rep via OCCT), organic (later: quad remesh → SubD → NURBS, S09).
- **Mesh section sketch:** plane ∩ mesh → polylines → fitted lines/arcs → sketch (S03).
- Parallelism with `rayon` everywhere; large meshes (≥ 10 M tris) supported.

## Interfaces
```rust
pub struct IndexedMesh { pub positions: Vec<DVec3>, pub tris: Vec<[u32;3]>, pub attrs: MeshAttrs }
pub fn repair(m: &IndexedMesh, opts: &RepairOpts, ctx: &JobCtx) -> Result<(IndexedMesh, RepairReport)>;
pub fn remesh(m: &IndexedMesh, opts: &RemeshOpts, ctx: &JobCtx) -> Result<IndexedMesh>;
pub fn boolean(op: BoolOp, a: &IndexedMesh, b: &IndexedMesh) -> Result<IndexedMesh>;
```

## Work packages
| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| MESH-01 | `IndexedMesh`, `HalfEdgeMesh`, conversions, attributes | FND-03 | P0 | Todo | round-trip, non-manifold input tolerated |
| MESH-02 | Analysis: manifold/watertight, boundaries, shells, volume/area, normals | MESH-01 | P0 | Todo | report on corpus of broken STLs |
| MESH-03 | BVH + queries (ray, closest point, winding number, self-intersection) | MESH-01 | P0 | Todo | 1 M-tri BVH build < 150 ms |
| MESH-04 | Repair pipeline (weld, degenerates, orient, holes, self-intersections) | MESH-02, MESH-03 | P0 | Todo | ≥ 95% of test corpus becomes watertight |
| MESH-05 | Booleans via Manifold (+ `-sys` crate) | MESH-01 | P0 | Todo | 1 M × 1 M < 1 s |
| MESH-06 | Isotropic + adaptive remesh with feature preservation | MESH-01 | P0 | Todo | quality metrics (angle distribution) |
| MESH-07 | QEM decimation | MESH-01 | P0 | Todo | 1 M → 100 k in < 2 s, Hausdorff bounded |
| MESH-08 | Smoothing (Laplacian/Taubin/bilateral) | MESH-01 | P0 | Todo | |
| MESH-09 | Plane cut with capping, separate, erase & fill, reverse normals | MESH-04 | P0 | Todo | |
| MESH-10 | Shell / offset (quick normal offset + robust SDF offset via S08) | MESH-04, IMP-03 | P0 | Todo | shell output always watertight |
| MESH-11 | Wall thickness analysis (GPU ray-based) | MESH-03, FND-04 | P1 | Todo | heat map overlay in viewport |
| MESH-12 | Face groups + primitive fitting (RANSAC) | MESH-03 | P1 | Todo | mechanical scan parts segment correctly |
| MESH-13 | Mesh→B-rep: faceted + prismatic | MESH-12, BREP-10 | P1 | Todo | prismatic output editable with fillet/extrude |
| MESH-14 | Mesh section sketch | MESH-03, SK-08 | P1 | Todo | |
| MESH-15 | Direct edit (move face groups, soft selection) | MESH-12 | P1 | Todo | |
| MESH-16 | Mesh features in timeline (wrappers for all ops above) | DOC-02 | P0 | Todo | each op is undoable/parametric |

## Testing
Corpus of real broken meshes (Thingiverse-style STLs, scans) in `testdata/mesh/`; metrics:
watertightness, Hausdorff, triangle quality; benchmarks for every op on 100 k / 1 M / 10 M tris.
