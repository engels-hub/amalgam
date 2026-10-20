# Amalgam — Architecture

This is the shared map every spec builds on. Changing anything here (crate boundaries,
core data model, threading model) needs an ADR in [02-decisions.md](02-decisions.md).

## Layering

```
                 ┌───────────────────────────────────────────────┐
  App            │ amg-app (binary)        amg-cli (headless)    │
                 ├───────────────────────────────────────────────┤
  Interaction    │ amg-ui (egui shell, commands, tools, gizmos)  │
                 │ amg-render (wgpu viewport, picking)           │
                 ├───────────────────────────────────────────────┤
  Document       │ amg-doc (design tree, timeline, params,       │
                 │          persistent naming, undo, file format)│
                 │ amg-features (feature implementations)        │
                 ├───────────────────────────────────────────────┤
  Domains        │ amg-sketch  amg-brep  amg-mesh  amg-implicit  │
                 │ amg-subd    amg-fem   amg-topopt  amg-am      │
                 │ amg-io      amg-materials     amg-nurbs       │
                 ├───────────────────────────────────────────────┤
  Foundation     │ amg-core (math, units, ids, tolerance, errors)│
                 │ amg-jobs (task pool, GPU context, progress)   │
                 │ amg-compute (GPU kernels, solvers, grids)     │
                 ├───────────────────────────────────────────────┤
  FFI            │ amg-occt-sys (cxx → OpenCASCADE)              │
                 │ manifold3d, fTetWild, etc. via *-sys crates   │
                 └───────────────────────────────────────────────┘
  Scripting: amg-script (pyo3) sits beside amg-ui and drives amg-doc commands.
```

**Rule:** a crate may depend only on crates in its own layer or below. Domain crates do not
depend on each other except where listed in the table below (keeps parallel work conflict-free).

## Crates and ownership

| Crate | Purpose | May depend on (besides core/jobs) | Spec |
|---|---|---|---|
| `amg-core` | f64 math (`glam::DVec3` etc.), units, tolerances, ids, `Error`, bounding boxes, transforms, 2D/3D primitive curves | — | S00 |
| `amg-jobs` | rayon pool, cancellable jobs, progress reporting, shared `wgpu` device for compute | — | S00 |
| `amg-compute` | GPU kernel helpers, voxel/brick grids, sparse matrices, iterative/direct/eigen solvers | — | S15 |
| `amg-occt-sys` | `cxx` bridge to OpenCASCADE 8.0.x (7.9.x fallback) | — | S02 |
| `amg-brep` | `BrepKernel` trait, OCCT backend, shape handles, tessellation, topology queries, history, robustness strategies (S20) | occt-sys, nurbs | S02, S20 |
| `amg-nurbs` | pure-Rust NURBS curves/surfaces: evaluation, knot ops, fitting, skinning, Gordon surfaces (ADR-013); converted to OCCT in `amg-brep` | — | S20 |
| `amg-sketch` | 2D sketch entities, constraint solver, profile/region detection | — | S03 |
| `amg-mesh` | half-edge/indexed meshes, repair, remesh, decimate, booleans (Manifold), analysis | — | S07 |
| `amg-implicit` | SDF expression graph, GPU evaluation, lattices, meshing (dual contouring) | mesh | S08 |
| `amg-subd` | Catmull–Clark subdivision modeling with creases, SubD→NURBS | brep, mesh, nurbs | S09 |
| `amg-materials` | physical + appearance material library | — | S17 |
| `amg-io` | STEP/IGES (via brep), STL, 3MF, OBJ, PLY, DXF, SVG | brep, mesh, sketch, materials | S12 |
| `amg-fem` | tet meshing, FE assembly, solvers, result fields | mesh, materials | S13 |
| `amg-topopt` | topology/shape optimization (generative design) | fem, implicit, mesh | S14 |
| `amg-am` | build orientation, overhang, supports, packing, slicer hand-off | mesh, implicit, io | S16 |
| `amg-doc` | design tree, components, bodies, timeline DAG, parameters & expressions, persistent naming, undo/redo, `.amg` format | all domain crates (as data types) | S01 |
| `amg-features` | every timeline feature (`Extrude`, `Fillet`, `MeshRemesh`, `Lattice`, …) implementing `Feature` | doc + domains | S04/S05/S06 + domain specs |
| `amg-render` | wgpu renderer, scene graph mirror, picking, analysis shaders, result plots | core, jobs (no doc dependency: fed via `RenderScene`) | S10 |
| `amg-ui` | egui app shell, command system, tools, dialogs, browser tree, timeline UI. Framework owned by S11; workspace modules (`inspect`, `mesh`, `form`, `sim`, `generative`, `am`, `io`, `materials`) owned by their domain specs | everything above | S11 + domain specs |
| `amg-script` | Python API (pyo3), headless batch | doc, features | S18 |
| `amg-cli` / `amg-app` | binaries | — | S00 |

## Core data model (summary — full detail in S01)

```
Document
 └─ Design
     ├─ Parameters (user + model params, expressions with units)
     ├─ Timeline: Vec<FeatureNode>          (ordered; rollback marker; groups; suppression)
     └─ Root Component
         ├─ Occurrences of child Components (transform, joints)
         ├─ Origin + Construction geometry
         ├─ Sketches
         └─ Bodies: struct Body { name, material, appearance, visible, kind: BodyKind }
                    enum BodyKind { Solid(BrepShape), Mesh(MeshBody), Implicit(SdfBody), Form(SubdBody) }
```

- **Features** are pure-ish functions: `fn compute(&self, ctx: &mut RecomputeCtx) -> Result<FeatureOutput>`.
  They read inputs via `EntityRef`s and write bodies/sketches/construction geometry.
- **Recompute** is incremental: the timeline is a DAG of feature dependencies; editing a feature
  marks it and its dependents dirty; results of clean features are cached.
- **Persistent naming**: features never hold raw face/edge indices. They hold `EntityRef`s
  (generating feature + topological role + geometric fingerprint fallback). Resolution happens at
  recompute. This is the single hardest problem in the project — see S01 §Persistent naming.
- **Direct mode**: a design can turn history off (Fusion's "Do not capture design history");
  features then apply once and are baked.
- **Commands vs features**: UI tools produce `Command`s (`AddFeature`, `EditFeature`,
  `SetParameter`, `TransformOccurrence`…). Commands are the only way to mutate a document; they
  are recorded for undo/redo and exposed 1:1 to the Python API.

## Threading & compute model

- **UI thread** owns the egui context and the `Document` handle; never runs geometry.
- **Recompute** runs on the job system against an immutable snapshot (`Arc` structural sharing);
  results are swapped in atomically. Long recomputes show progress and are cancellable.
- **Heavy studies** (FEA, topopt, remesh of huge meshes, lattice meshing) are `Job`s with
  progress, cancel, and persisted results attached to the document.
- **GPU**: one `wgpu::Device` shared by render and compute (`amg-jobs::Gpu`). Compute workloads
  use WGSL compute shaders. CUDA (via `cudarc`) is an allowed optional backend for solvers only
  if wgpu proves too slow — behind a cargo feature, never required.
- **B-rep shapes** (amended 2026-10-20, owner accepted, QUESTIONS #42): `amg_brep::Shape` is
  immutable and `Send + Sync`. Kernel ops never mutate their inputs: non-destructive booleans,
  and private topology copies for meshing and healing. Only IGES I/O (and, on OCCT 8.0.0, text
  `.brep` and STEP reads) take a process-wide lock. Independent operations, including different
  ops on the same shape, may run in parallel.

## Geometry representations and conversions

| From \ To | Solid (B-rep) | Mesh | Implicit (SDF) | Form (SubD) |
|---|---|---|---|---|
| Solid | — | tessellate (S02) | voxelize / exact SDF of mesh (S08) | — |
| Mesh | faceted, prismatic (planes/cyl fit), organic (S07) | — | mesh SDF (S08) | quad-remesh → SubD cage (S09, later) |
| Implicit | via mesh | dual contouring (S08) | — | — |
| Form | SubD→NURBS (S09) | subdivide (S09) | via mesh | — |

Booleans across types are resolved by converting to the "lowest common" representation
(usually mesh) with a user-visible warning; solid∘solid stays B-rep.

## Rendering pipeline (summary — full detail in S10)

wgpu (DX12 on Windows, Vulkan elsewhere). forward shading (2–3 fixed studio lights), MSAA, GPU picking via ID buffer,
edge/silhouette rendering, section planes via fragment discard (DX12 has no clip distances) with one-pass back-face caps, transparent bodies via weighted OIT,
analysis overlays (zebra, curvature, draft, overhang, FEA fringe plots) as shader variants.
On-canvas text is painted by egui. (Rendering wording amended 2026-10-20, owner accepted,
QUESTIONS #42.)
Render scene is a flat list of GPU meshes + instances derived from the document; `amg-render`
does not know about features.

## File format (`.amg`)

Zip container:
- `design.json` — document tree, timeline, parameters (versioned schema, serde).
- `cache/` — optional recompute caches (OCCT `.brep`, binary meshes) keyed by feature hash, so
  opening a file doesn't require full recompute.
- `bodies/` — baked bodies for direct-mode features and imported meshes (binary, compressed).
- `studies/` — simulation / generative setups and results.
- `thumbnail.png`.

## Testing strategy

- Unit tests per crate; **golden geometry tests** (volume, area, bbox, topology counts, Hausdorff
  distance against reference meshes) in `tests/golden/`.
- Property tests (`proptest`) for mesh repair, booleans, solver robustness.
- Benchmarks (`criterion`) for every budget in the vision doc.
- Corpus of real-world test models in `testdata/` (git-LFS) — STEP files, broken STLs, lattices.
- Simulation verified against analytic solutions (cantilever beam, plate with hole, modal of a bar)
  and, where possible, against Fusion results exported by the owner.
