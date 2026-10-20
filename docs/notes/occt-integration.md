# OCCT integration: version, bindings, build, thread safety, history

Design note for [S02 — B-rep kernel](../specs/S02-brep-kernel.md). Written during S02
refinement (2026-10-20). The spec holds the decisions and the API; this note holds the
evidence and the detail that BREP-01, BREP-02, BREP-05 and BREP-12 implementers need.

## 1. Version: OCCT 8.0.x (pin 8.0.0, port-version 1)

**Decision.** BREP-01 takes the OCCT that the vcpkg baseline `2026.07.29` ships:
`opencascade` **8.0.0, port-version 1**. No `overrides` entry is needed. 7.9.3 (port-version 1)
is the documented fallback if BREP-01 hits a blocking 8.0 regression (see §1.3).

### 1.1 Evidence
- **vcpkg versions database** (`versions/o-/opencascade.json`, master, 2026-10): 8.0.1#0,
  8.0.0#1, 8.0.0#0, 7.9.3#1, 7.9.3#0, 7.9.2, 7.9.1, 7.9.0, 7.8.1#1, … The port at tag
  `2026.07.29` is 8.0.0#1. 8.0.1 came later and needs a newer vcpkg baseline. An `overrides`
  entry can only select a version the checked-out registry knows, so 8.0.1 means a baseline
  bump (an S00 pin change), not an override.
- **OCCT 8.0.0 release** (2026-05-07, after 5 RCs and 3 betas, 500+ changes since 7.9.0):
  - C++17 baseline; CMake ≥ 3.16. Our MSVC v143 and CMake 4.3 (S00) are fine.
  - **`Standard_Failure` derives from `std::exception`** (`what()`, `ExceptionType()`).
    `Raise()`/`Throw()`/`Instance()` are removed in favour of plain `throw`. The cxx default
    `trycatch` (catches `std::exception`) therefore maps every OCCT exception to `Err` with no
    custom handler. We still install one (§3.3) to also catch `...` and to keep the exception
    type name.
  - `Standard_Mutex` and `TopTools_MutexForShapeProvider` are replaced by `std::mutex`. The
    error-handler stack is `thread_local`.
  - **STEP read and write are thread-safe with one reader/writer instance per thread. IGES read
    is still not thread-safe.** This drives the I/O locking rule in §4.4.
  - `BRepGraph`: a new graph view of topology with history. We do not use it in Phase 1–3,
    but it is the natural base for KH-11 later.
  - BRepMesh plugin macros are replaced by `BRepMesh_DiscretAlgoFactory`. We use
    `BRepMesh_IncrementalMesh` directly, so this does not affect us.
  - Typedef-only packages `TColGeom`, `TColGeom2d`, `Geom2dLProp`, `LProp3d` are removed.
    Shim code uses `NCollection_Array1<Handle(Geom_Curve)>` etc. directly.
  - `TopoDS_TShape` packs type and flags into a `uint16_t`, so `ShapeType()` is non-virtual.
    Geometry adaptors are devirtualised through `std::variant`, and the `IncAllocator` fast path
    is lock-free. These are free performance gains for exploration and tessellation.
  - Sources are re-laid out as `src/Module/Toolkit/Package`. Installed headers stay flat
    (`include/opencascade/*.hxx`), so this does not affect the shim.
- **8.0.0p1 / 8.0.1** fixes (hot patch notes) relevant to us: `Standard_Strtod` Bigint pool
  made thread-local ("random numeric mis-parses in parallel parsing workloads such as
  parallel BRep reads"), a `BRepFill_CompatibleWires` crash in ThruSections, a
  `BOPTools_AlgoTools::GetFaceOff` hardening for near-coplanar faces, and an infinite-loop guard
  in `IntWalk_IWalking`. **The Strtod fix matters**: text `.brep` reads in parallel on 8.0.0 can
  mis-parse numbers. Mitigation on 8.0.0: our caches use the binary format (`BinTools`, no
  `strtod`), and text `.brep` reads take the global I/O lock (§4.4). Recommend the owner bumps the
  baseline to a vcpkg release that ships 8.0.1 at the next S00 pin bump (report, not decided here).

### 1.2 Why not 7.9.x
- 7.9 is the last 7.x line. Fixes now land on 8.0.x, and 8.0.0p1 is "the foundation for
  future 8.0.x maintenance releases" (quarterly cadence).
- 7.9 lacks the thread-safe STEP I/O and the `std::exception` base. Both remove work from us.
- The S20 hardening work (KH-02..KH-09) files upstream bugs (KH-09). Upstream only accepts
  fixes against the current line.

### 1.3 Fallback rule
If BREP-01 finds a blocking regression in 8.0.0 (crash or wrong result in the smoke corpus that
7.9.3 does not have), it pins `"overrides": [{ "name": "opencascade", "version": "7.9.3",
"port-version": 1 }]` and records the reason in the S02 Refinement log. The shim must then also
compile against 7.9.3, which means the custom `trycatch` (§3.3) catches `Standard_Failure`
explicitly (in 7.9 it does not derive from `std::exception`). We write the handler that way
from day one, so this costs nothing.

## 2. Build: vcpkg manifest mode, static release libs

- **Triplet**: S00's overlay `x64-windows-amg` (x64, `VCPKG_CRT_LINKAGE dynamic`,
  `VCPKG_LIBRARY_LINKAGE static`, `VCPKG_BUILD_TYPE release`). Kept, with this evidence:
  - The port maps `VCPKG_LIBRARY_LINKAGE static` to `-DBUILD_LIBRARY_TYPE=Static` and patches
    `Standard_Macro.hxx` so `OCCT_STATIC_BUILD` is always defined. Our shim therefore needs no
    `Standard_EXPORT` defines.
  - The portfile does not touch the CRT, so the triplet's `/MD` holds. That matches rustc,
    which always links the release CRT, even in debug builds.
  - Static linking of LGPL-2.1 code is allowed when the user can relink. For a personal-use
    build this is satisfied trivially; it is recorded in `THIRD_PARTY.md` (ADR-012).
  - Static libs mean no DLLs next to each test binary, so `cargo nextest` just works.
    Link time is the cost; BREP-01 records it.
- **Features**: `"default-features": false`. The only default is `freetype`, which serves
  text rendering in the Visualization module, which we do not link. `tbb` stays **off**:
  OCCT's own `OSD_ThreadPool` is used for its internal parallelism, and S07's note warns that a
  second statically linked TBB next to Manifold's causes crashes. `vtk`, `freeimage`, `rapidjson`
  stay off. The port always builds the Visualization toolkits (needs `opengl`, which is a
  system library on Windows); we just don't link them.
- **Toolkits linked** (static, in dependency order; BREP-01 verifies the exact names against
  `lib/cmake/opencascade/OpenCASCADE*Targets.cmake`):
  `TKernel TKMath TKG2d TKG3d TKGeomBase TKBRep TKGeomAlgo TKTopAlgo TKPrim TKBO TKBool
  TKShHealing TKFillet TKOffset TKFeat TKMesh TKHelix TKXSBase TKDE TKDESTEP TKDEIGES TKCDF TKLCAF
  TKCAF TKXCAF TKBin TKBinL`. System libs: `advapi32 user32 ws2_32` (BREP-01 trims the list to
  what the linker asks for).
- **Finding the install**: vcpkg manifest mode installs into
  `<workspace>/vcpkg_installed/x64-windows-amg/`. `amg-occt-sys/build.rs` reads
  `AMG_OCCT_DIR` (set in `.cargo/config.toml [env]` to that path, relative to the workspace;
  overridable for Linux/system OCCT). It does **not** run vcpkg or CMake itself (same rule as
  `amg-manifold-sys`: no hidden network fetch or C++ build in `build.rs`). If the directory is
  missing it fails with a message that says to run `vcpkg install --triplet x64-windows-amg
  --overlay-triplets=triplets` at the workspace root.
- **Shim compile flags** (`cxx_build`): `/std:c++17 /EHsc /MD /utf-8 /bigobj /permissive-`,
  `-I $AMG_OCCT_DIR/include/opencascade`. Release optimisation even in dev
  (`[profile.dev.package."*"] opt-level = 2` already covers `cc`'s `OPT_LEVEL`).
- **Linux (best effort)**: `AMG_OCCT_DIR` points at a vcpkg `x64-linux` install or a system
  OCCT 8.0 prefix. The same toolkit list applies, linked statically if `.a` files exist, else
  dynamically.
- **Cold build cost**: the OCCT port is the largest C++ build in the project. CI relies on the
  S00 `files` binary cache; BREP-01 records the cold and cached times.

## 3. Bindings: hand-written cxx bridge over a coarse C++ shim

### 3.1 Options considered
| Option | Pros | Cons | Verdict |
|---|---|---|---|
| **cxx** + hand-written shim (`occt_shim.hxx/.cpp`) | Type-checked both sides; `UniquePtr`/`SharedPtr` ownership; exceptions → `Result` built in (`rust::behavior::trycatch`); Rust callbacks (`extern "Rust"`) for progress/cancel/logging; used by `opencascade-rs` | We write every function by hand | **Chosen** (ADR-002) |
| autocxx (bindgen → cxx) | Generates from headers | Needs libclang; OCCT's `Handle(T)`, NCollection templates, macros and overloaded virtuals defeat it; slow builds; its own docs recommend plain cxx when you control the C++ side; would expose fine-grained OCCT calls (many FFI crossings, exceptions everywhere) | Rejected |
| `extern "C"` shim + opaque pointers | No cxx dependency; simplest ABI | Manual `Drop`, manual error-out-params, manual string passing, manual callback trampolines; no type checking of the boundary | Rejected (cxx gives all of this for free) |

### 3.2 Shape of the shim
- **Coarse functions.** One shim function per kernel operation (`make_prism`, `boolean`,
  `fillet`, `tessellate`, …), not per OCCT class. Each call does all its OCCT work in C++ and
  returns either an owned result (`UniquePtr<OpOutput>`) or throws. This keeps FFI crossings to
  O(1) per operation and keeps OCCT types out of Rust entirely.
- **Opaque types** in `amg-occt-sys`: `OcShape` (wraps `TopoDS_Shape`), `OcIndex` (wraps the
  three `TopTools_IndexedMapOfShape`s for faces/edges/vertices plus face→edge/edge→face
  adjacency), `OcOpOutput` (result shape + `BRepTools_History` + role lists), `OcTess`
  (tessellation buffers before copy-out). Plain data crosses as cxx shared structs
  (`struct V3 { x: f64, y: f64, z: f64 }`, `struct SubRef { kind: u8, index: u32 }`, vectors of
  these as `rust::Vec`/`rust::Slice`).
- **Inputs as plain data.** Profiles, curves and surfaces cross as shared structs (see S02
  Interfaces, `geom` module); the shim builds `Geom_*` objects from them. No OCCT geometry handle
  ever crosses into Rust.
- **Bulk outputs copied once** into `rust::Vec<f64>`/`rust::Vec<u32>` (tessellation, polylines),
  which `amg-brep` reinterprets as `Vec<[f64; 3]>` via `bytemuck` (no per-element FFI).

### 3.3 Exceptions
`occt_shim.hxx` defines `rust::behavior::trycatch` (the documented cxx customisation point):

```cpp
namespace rust::behavior {
template <typename Try, typename Fail>
static void trycatch(Try &&func, Fail &&fail) noexcept try {
  func();
} catch (const Standard_Failure &e) {          // 8.0: also a std::exception; 7.9: not
  fail(amg::format_failure(e));               // "<ExceptionType>: <message>"
} catch (const std::exception &e) {
  fail(e.what());
} catch (...) {
  fail("unknown C++ exception");
}
}
```
Every shim function is declared `-> Result<…>` in the bridge, so no exception can reach
`std::terminate`. Algorithm-level failures that are not exceptions (`IsDone() == false`,
`BOPAlgo` alerts, `BRepFilletAPI_MakeFillet::NbFaultyContours() > 0`) are returned as a
structured `OcStatus` (code + message + offending sub-shapes) inside the output, not thrown, so
`KernelError` can carry the offending edges (S20 needs them).

**Access violations are not caught.** OCCT can translate SEH into exceptions
(`OSD::SetThreadLocalSignal`), but that needs `/EHa` in every TU and leaves process state
undefined after the fault. We do not enable it. Native crashes go to FND-12 minidumps, and S20
KH-01's `write_ahead` capture mode (inputs written *before* the call, see
`kernel-robustness.md` §1.3) keeps crashing inputs reproducible.

### 3.4 Messages, progress, cancellation
- **Logging**: BREP-12 replaces `Message::DefaultMessenger()`'s printers with one
  `Message_Printer` subclass that forwards to an `extern "Rust" fn occt_log(level, msg)`,
  which emits `tracing` events under target `occt`. No OCCT output reaches stdout.
- **Progress/cancel**: a `Message_ProgressIndicator` subclass (`AmgProgress`) wraps a Rust
  `JobCtx` reference passed as an opaque `extern "Rust"` type. `Show()` calls
  `ctx.progress(fraction)` (rate-limited by `amg-jobs` itself); `UserBreak()` returns
  `ctx.is_cancelled()`. Algorithms that accept a `Message_ProgressRange` (booleans
  `BRepAlgoAPI_*::Build`, `BRepMesh_IncrementalMesh`, `BRepOffsetAPI_ThruSections` and
  `MakePipeShell::Build`, `BRepBuilderAPI_Sewing::Perform`, STEP/IGES transfer) cancel
  mid-operation. Others (fillet, chamfer, draft, offset) cancel only before/after the call;
  the spec documents which.
- After a user break the shim returns `OcStatus::Cancelled`, which maps to
  `KernelError::Cancelled` (`From<Cancelled>` holds).

## 4. Thread safety and the `Shape: Sync` requirement (ADR-014)

### 4.1 The facts
1. **Reference counting is atomic.** `Standard_Transient`'s counter is atomic, so copying
   `TopoDS_Shape` values (handles to `TopoDS_TShape` + `TopLoc_Location`) and `Handle(Geom_*)`
   across threads is safe. This is what makes an `Arc`-like shared shape possible at all.
2. **Topology and geometry objects are plain data with no internal caches** once built.
   `Geom_BSplineSurface` lost its internal evaluation cache in OCCT 7.0; caches now live in
   per-call adaptors (`GeomAdaptor_Surface`, `BRepAdaptor_*`) that each algorithm owns.
   Read-only algorithms (exploration, `BRep_Tool`, `BRepGProp`, `BRepExtrema`,
   `BRepClass3d_SolidClassifier`, `BRepBndLib`, `BRepCheck_Analyzer`) build their own
   adaptors, so concurrent reads of one shape from several threads are data-race free.
3. **Some algorithms write into their *input* TShapes.** This is where the "not Sync" claim in
   the draft came from:
   - **`BRepMesh_IncrementalMesh`** stores `Poly_Triangulation` on each `BRep_TFace` and
     `Poly_PolygonOnTriangulation` on each `BRep_TEdge` of the shape it meshes. Kirill Gavrilov
     (OCCT): meshing works "with the triangulation cache stored directly within TopoDS_Shape
     structures", so shapes that share sub-shapes "lead to data races on modifications of
     TopoDS_Shape tessellation cache" when meshed from parallel threads. OCCT bug #22850 was an
     instance (`BRepMesh_FastDiscretFace::AddInShape` → `BRep_Builder::UpdateEdge` on shared
     edges).
   - **Booleans** (`BOPAlgo`) by default may enlarge the tolerances of input vertices/edges
     in place. `BOPAlgo_Options::SetNonDestructive(true)` makes them copy any sub-shape that
     needs updating instead ("argument shapes are not modified; a copy of a sub-shape is
     created in the result if it needs to be updated"). OCCT developers point at exactly this
     flag when users report crashes from concurrent booleans on shared inputs.
   - **Shape healing** (`ShapeFix_*`), `BRepLib::BuildCurves3d`, `BRepLib::SameParameter`,
     `BRepLib::UpdateTolerances`, `ShapeUpgrade_UnifySameDomain` (in-place mode) and
     `BRepTools::Clean` modify their argument.
   - `TopLoc`-located copies (`Shape::transformed`) share TShapes with the original, so a
     mutation through one is visible through the other.
4. Some global state exists: `Message::DefaultMessenger()`, `Interface_Static` parameters used
   by data exchange, `OSD_ThreadPool::DefaultPool()`. IGES read is not thread-safe even in 8.0.

### 4.2 Decision: immutable shapes, mutate only private copies
`amg_brep::Shape` is an **immutable, `Send + Sync`** handle, so ADR-014 holds as written and no
mutex is needed in `amg-doc` (S01's interim `BodyGeom::Solid` mutex goes away when BREP-02
lands):

```rust
#[derive(Clone)]
pub struct Shape(Arc<ShapeInner>);
struct ShapeInner {
    occ: cxx::UniquePtr<ffi::OcShape>,      // never mutated after construction
    index: OnceLock<SubShapeIndex>,         // face/edge/vertex maps + adjacency, built lazily
    stats: OnceLock<ShapeStats>,            // bbox, counts
}
// SAFETY (amg-occt-sys): `OcShape` is only ever passed to shim functions that take
// `const OcShape&` and are on the read-only allow-list (§4.3). Functions that would write into
// the TShape graph operate on a private copy made inside the shim.
unsafe impl Send for ffi::OcShape {}
unsafe impl Sync for ffi::OcShape {}
```

The rule, enforced in `amg-occt-sys`:

| Category | Rule | Examples |
|---|---|---|
| **R — read-only** | Take `const OcShape&`; may run concurrently on the same shape | exploration, indexing, `BRep_Tool` queries, mass props, distance, classification, bbox, BRepCheck, BinTools write, STEP write |
| **N — non-destructive by flag** | Set the OCCT flag that forbids input mutation | all `BOPAlgo`/`BRepAlgoAPI` ops: `SetNonDestructive(true)`; `ShapeUpgrade_UnifySameDomain` with a fresh result (not in-place) |
| **C — copy first** | `BRepBuilderAPI_Copy(input, /*copyGeom*/ false, /*copyMesh*/ false)` inside the shim, run the algorithm on the copy, compose the copy's history into the result's history | **tessellation**, `ShapeFix_*`, sewing of inputs, any op that BREP-12's stress test shows mutating its input |
| **G — global lock** | Process-wide mutex in `amg-brep` | IGES read/write; on 8.0.0 text `.brep` **and STEP** reads (`Standard_Strtod` race, §1.1; STEP parsing goes through the same `strtod`); any op the stress test shows racing on *global* state |

Topology-only copies (`copyGeom = false`) share the `Geom_*` objects, which are not mutated by
category-C algorithms (they write tolerances, pcurves and triangulations, which live in TShapes).
Healing ops that can replace geometry (`ShapeFix_Shape` with `FixFaceMode`) use
`copyGeom = true`.

**Tessellation in detail.** `tessellate()` copies the topology (`BRepBuilderAPI_Copy`, no
geometry, no mesh), runs `BRepMesh_IncrementalMesh` on the copy, copies the buffers out to Rust
and drops the copy. Nobody ever writes a triangulation into a shared TShape, so tessellating the
same `Shape` from two threads, or tessellating while another thread runs a boolean on it, is
safe. Cost: a topology copy is linear in sub-shape count with no geometry work; BREP-08 measures
it against the 100 ms / 1000-face budget (expected < 5 %). If it ever shows up in profiles, the
alternative is a per-`Shape` `Mutex<()>` around in-place meshing plus `BRepTools::Clean` on
output shapes. That is recorded, not built.

**Why not `Send`-only plus a mutex (the draft's model).** It would serialise all readers of a
body (the UI picking a face while recompute fillets it), put a lock in every `amg-doc` access,
and still not protect against the real hazard: two *different* `Shape`s that share TShapes
(after `transformed()`, patterns, or an unchanged face passed through a boolean) being meshed
in parallel. The copy rule is the only one that covers sharing.

**Output shapes and sharing.** Results of booleans and local ops share untouched sub-shapes
with their inputs (that is how OCCT history works). That is fine under the rule above, because
nobody writes to any of them.

### 4.3 Proving it: the allow-list and the stress test (BREP-12)
- Every shim function carries a `// CATEGORY: R|N|C|G` comment and a one-line reason;
  BREP-12 adds a test that greps the shim and fails if a function has no category.
- **Concurrency stress test** (BREP-12, re-run by every op WP): 8 threads × 2 000 iterations on
  one shared `Shape` (a 200-face filleted part from the fixture set) doing a random mix of
  category-R queries, `tessellate`, `boolean(cut, shared, small box)` and `fillet` on copies.
  It asserts (a) no crash, (b) every query result equals the single-threaded result
  bit-for-bit, (c) the shared input's BinTools serialisation hash is unchanged afterwards (proves
  no input mutation). On Linux CI (best effort) the same test runs under ThreadSanitizer
  (`-Zsanitizer=thread` nightly job, allowed to fail until the OCCT build is instrumented).
- Any op that fails (c) moves to category C.

### 4.4 Global state rules
- **Init once** (`amg_brep::init()`, idempotent, `OnceLock`): install the message printer
  (§3.4); set `OSD_ThreadPool::DefaultPool(n)` to `KernelConfig::occt_threads` (default:
  `available_parallelism() / 2`, min 1, see §5); read nothing from environment variables such
  as `CSF_*` (we never rely on resource files; data-exchange parameters are set
  explicitly per reader).
- **STEP**: one `STEPCAFControl_Reader`/`Writer` per call (thread-safe per instance in 8.0).
  On 8.0.0, reads still take the global lock until the baseline moves to 8.0.1 (§1.1).
  Parameters are passed through the 8.0 `DE_*` parameter objects, not `Interface_Static`
  globals.
- **IGES**: `G` category, global lock.

## 5. OCCT internal parallelism vs `amg-jobs`

- OCCT parallelises inside `BOPAlgo` (`SetRunParallel`), `BRepMesh` (`InParallel`) and a few
  others through `OSD_ThreadPool`, separate from our rayon pools.
- **Policy**:
  - Booleans: `SetRunParallel(true)` by default *if* BREP-04's determinism test passes
    (10 runs of each of 20 boolean cases with parallel on give byte-identical BinTools output).
    Otherwise default off, recorded in the spec. Answers draft open question 2.
  - Tessellation of one body: `InParallel = true` (parallel over faces).
    Tessellation of many bodies (`tessellate_many`): rayon over bodies, `InParallel = false`
    per body, to avoid oversubscription.
  - `occt_threads` default = half the cores, so OCCT's pool plus the Interactive rayon pool
    don't starve the Background pool. BREP-04 benchmarks 1/half/all and may change the default.
- OCCT pool threads run at normal OS priority even when the calling job is Background.
  Acceptable: Background OCCT work (batch STEP import) is short-lived compared with FEA.

## 6. Determinism
- Same inputs + same build → same output bits (vision principle). Sources of nondeterminism in
  OCCT: parallel mode (§5), and iteration over pointer-keyed maps (`TopTools_MapOfShape` hashes
  by TShape address). OCCT's own algorithms are written to be address-independent in their
  results, but sub-shape *order* in some outputs has historically depended on map iteration.
- **Our safeguard**: our in-shape indices come from `TopExp::MapShapes` on the result
  (deterministic DFS of the topology structure, not address-dependent). BREP-02 tests
  index stability across processes (build a shape in two processes, compare the index-ordered
  fingerprints). BREP-04/06 run each op 10× and compare BinTools bytes.
- If a case is found that is not deterministic, the fallback is to canonicalise the result
  (rebuild compounds in fingerprint order). The case goes into the S20 corpus.

## 7. History mapping (feeds S01 `TopoName`)

### 7.1 What OCCT gives
- `BRepBuilderAPI_MakeShape` (base of nearly every API class): `Generated(S)`, `Modified(S)`,
  `IsDeleted(S)` per input sub-shape, plus op-specific accessors:
  `BRepPrimAPI_MakePrism/Revol::FirstShape()/LastShape()` (caps, and per-edge
  `FirstShape(edge)`), `BRepOffsetAPI_ThruSections::GeneratedFace(edge)`,
  `BRepFilletAPI_MakeFillet::NewFaces(contour)`, `BRepOffsetAPI_MakeThickSolid` faces,
  `BRepAlgoAPI_BooleanOperation::SectionEdges()`.
- `BRepTools_History` (since 7.3): a mergeable history object; `BOPAlgo_Builder::History()`
  returns one; `BRepTools_History::Merge` composes two steps. We use it to compose the internal
  copy step (§4.2 category C) and multi-step ops (prism followed by `UnifySameDomain`).

### 7.2 What `amg-brep` returns
`ShapeHistory` (S02 Interfaces) is a flat, index-based, serialisable view:
- For each operand `i` (input shape), for each of its faces/edges/vertices `j` (by the operand's
  in-shape index): `Modified → [output indices]`, `Generated → [output indices]` (generation
  can cross kinds: edge → face for prism laterals, edge → face for fillets, vertex → edge for
  prism lateral edges), `Deleted`.
- Output sub-shapes not mentioned anywhere are either **unchanged** pass-throughs (the same
  TShape as an input sub-shape; detected with `IsSame` and reported as `Kept`) or op-created
  without a source (reported with a role, e.g. a fillet corner patch).
- **Roles**: op-semantic labels that OCCT's generic history does not express:
  `StartCap`, `EndCap`, `Lateral`, `FilletBlend { contour }`, `ChamferFace`, `ShellInner`,
  `SectionEdge`, `LoftSection { index }`, `SplitPiece { index }`. Roles are attached to output
  sub-shapes. DOC-11/DOC-06 turn (role, sources) into `NameSeg::Generated { role, from }`.
- **Tags** for constructed faces: face/wire builders take a caller-chosen `Tag(u64)` per input
  curve (S04 puts the sketch entity key there) and return `(EdgeId, Tag)` for every created
  edge. A later prism's `Generated(edge)` then links lateral faces to that tag.
- **Split ordering**: when one input face is modified into several output faces, the output
  list is sorted by (area descending, centroid lexicographic) so `NameSeg::Split { index }` has
  a deterministic meaning. DOC-11 may refine the criterion; the sort lives in one function.

### 7.3 Guarantees (tested in BREP-05)
- Every output face appears in the history either as Modified/Generated from some input element,
  as `Kept`, or carries a role. No face is "unexplained" (test over the op corpus).
- History of a composite op equals the composition of its steps (`Merge`), including the
  internal topology copy.
- `ShapeHistory` round-trips through serde (it is cached by S01 as part of `FeatureOutput`).

## 8. Sources
- vcpkg port and versions DB: `github.com/microsoft/vcpkg` `ports/opencascade/{vcpkg.json,
  portfile.cmake}` (tag `2026.07.29`: 8.0.0#1; master: 8.0.1), `versions/o-/opencascade.json`.
- OCCT 8.0.0 release announcement (2026-05-07):
  `occt3d.com/dev/content/open-cascade-technology-800-release` (formerly
  `dev.opencascade.org/content/open-cascade-technology-800-release`); RC3/RC5 notes; migration
  guide `dev.opencascade.org/doc/overview/html/occt__upgrade.html`.
- OCCT 8.0.0p1 notes (newreleases.io mirror of the GitHub release `V8_0_0_p1`).
- OCCT forum: "Thread safety of OCC?", "parallel applications with Open CASCADE",
  "Multithreaded BRepMesh::Mesh", "BRepAlgo_Common is not thread-safe", "Crash using
  BRepAlgoAPI_Common multithreading" (2024-05); tracker issue #22850.
- OCCT reference: `BOPAlgo_Options::SetNonDestructive` (7.4–7.9 refman), `BRepTools_History`.
- cxx book, "Result<T>" chapter (custom `rust::behavior::trycatch`).
- autocxx docs (google.github.io/autocxx): recommends cxx directly when you control the C++.
- opencascade-rs (`github.com/bschwind/opencascade-rs`): cxx + `wrapper.hxx`, build reference.
