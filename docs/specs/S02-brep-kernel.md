# S02 — B-rep kernel (OpenCASCADE integration)

| | |
|---|---|
| **Status** | Draft |
| **Priority** | P0 |
| **Crates** | `amg-occt-sys`, `amg-brep` |
| **Lane** | A |
| **Depends on** | S00 (FND-01/02) |
| **Parity** | Underpins all solid modeling, STEP/IGES |

## Summary
Safe Rust wrapper around OCCT exposing exactly what Amalgam needs through a `BrepKernel`
trait: shape construction, booleans, fillets/chamfers, offsets/shells, drafts, sweeps/lofts,
topology queries, tessellation, mass properties, history (for persistent naming), STEP/IGES I/O.

## Scope
**In:** C++ build via vcpkg, `cxx` bridge, handle/lifetime management, error mapping,
the trait + OCCT backend, tessellation, history maps, healing (`ShapeFix`), serialization (`.brep`).
**Out:** feature semantics (S04), naming policy (S01) — this crate only *reports* history.

## Design
- `amg-occt-sys`: thin `cxx` bridge, C++ shim files grouping OCCT calls into coarse functions
  (`make_prism(face, vec) -> Shape`, `fillet(shape, edges, radii) -> Result`). Coarse = fewer FFI
  crossings + exceptions caught in C++ (`Standard_Failure` → error code + message).
- `amg-brep`:
  - `Shape` (owned handle, `Clone` = refcount), `Face/Edge/Vertex` views with stable in-shape
    indices (`TopTools_IndexedMapOfShape`).
  - `trait BrepKernel` with methods grouped: primitives, sweeps (prism, revol, pipe, loft),
    booleans (fuse/cut/common/split, with fuzzy tolerance), local ops (fillet, chamfer, draft,
    shell/thick solid, offset face, defeature/replace face), transforms, healing, queries
    (surface/curve type & params, adjacency, closest point, ray hit), mass props,
    tessellation (`BRepMesh_IncrementalMesh` with deflection/angle → indexed mesh with
    per-face ranges and edge polylines), `history()` returning generated/modified/deleted.
  - Thread safety: OCCT objects are `Send` but not `Sync`; operations on independent shapes may
    run in parallel; parallel tessellation enabled (`BRepMesh` has `isInParallel`).
- Geometry sanity: always check `BRepCheck_Analyzer` after ops in debug builds; `ShapeFix` on imports.

## Interfaces
```rust
pub trait BrepKernel {
    fn extrude(&self, profile: &Face, dir: DVec3, opts: &ExtrudeOpts) -> KResult<OpResult>;
    fn boolean(&self, op: BoolOp, a: &Shape, tools: &[Shape], fuzzy: f64) -> KResult<OpResult>;
    fn fillet(&self, s: &Shape, edges: &[(EdgeId, FilletSpec)]) -> KResult<OpResult>;
    fn tessellate(&self, s: &Shape, q: &TessQuality) -> KResult<TessMesh>;
    // …
}
pub struct OpResult { pub shape: Shape, pub history: ShapeHistory }
```

## Work packages
| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| BREP-01 | Build OCCT 7.8 via vcpkg, `cxx` bridge skeleton, exception mapping | FND-02 | P0 | Todo | box created and volume read from Rust; C++ exception becomes `Err` |
| BREP-02 | Shape handles, topology exploration, indexed sub-shapes, queries | BREP-01 | P0 | Todo | enumerate faces/edges of a STEP part with surface types |
| BREP-03 | Primitives, prism, revol, pipe (+guide), loft (+rails), thicken | BREP-02 | P0 | Todo | golden volumes for each |
| BREP-04 | Booleans (fuse/cut/common/split, fuzzy, parallel) | BREP-02 | P0 | Todo | 100-case boolean corpus passes BRepCheck |
| BREP-05 | History extraction (generated/modified/deleted) for every op | BREP-03 | P0 | Todo | API used by DOC-06 tests |
| BREP-06 | Fillet (const, variable, chord), chamfer (equal, two-dist, dist-angle) | BREP-02 | P0 | Todo | corpus incl. failure reporting with offending edge |
| BREP-07 | Shell, draft, offset face, replace face, split face/body, defeature | BREP-02 | P0 | Todo | golden tests |
| BREP-08 | Tessellation with per-face ranges, edge polylines, normals; quality presets | BREP-02 | P0 | Todo | 1000-face part tessellates < 100 ms |
| BREP-09 | Mass properties, closest point, ray cast, distance, interference | BREP-02 | P0 | Todo | matches analytic values |
| BREP-10 | Healing/sewing, `.brep` serialization, STEP/IGES read/write (with names/colors via XCAF) | BREP-02 | P0 | Todo | round-trip corpus; consumed by S12 |
| BREP-11 | Thread geometry (helical sweep) support | BREP-03 | P1 | Todo | M3–M20 threads model correctly |

## Testing
Golden volume/area/topology-count tests; BRepCheck validity on every result; fuzz booleans
with random primitives (expect `Ok(valid)` or clean `Err`, never crash).

## Open questions
1. OCCT version pin (7.8.x vs 7.9) and vcpkg triplet (`x64-windows` dynamic vs static).
   **Partly answered (2026-10-20, ADR-002 accepted with amendment):** the version is 7.9.x or
   8.0.x via vcpkg, and BREP-01 picks and pins one. The triplet is still open.
2. Do we expose OCCT's `BOPAlgo` parallel mode by default? Benchmark.
