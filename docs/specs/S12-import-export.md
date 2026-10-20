# S12 — Import / export

| | |
|---|---|
| **Status** | Refined |
| **Priority** | P0 |
| **Crates** | `amg-io`, `amg-features::io` (import features, document ↔ scene adapter; lane G, see Refinement log), `amg-ui::io` (import/export dialogs, drag-drop) |
| **Lane** | G |
| **Depends on** | S00 (FND-04 jobs, FND-06 testkit, FND-10 units), S07 (MESH-01 `IndexedMesh`, MESH-02 analysis, MESH-16 mesh bodies), S01 (DOC-05 commands, DOC-07 container), S02 (BREP-08 tessellation, BREP-13 STEP/IGES `ForeignModel`, BREP-17 curve extraction), S03 (SK-01 curves for DXF/SVG export), S06 (ASM-01 components for STEP assemblies), S11 (UI-01/02/10), S16 (AM-01 print setup, consumer), S17 (MAT-01 material names, optional) |
| **Parity** | Data section (import/export) |

## Summary
Reliable interchange:
- STL/3MF/OBJ/PLY for meshes;
- STEP/IGES for B-rep, with assembly structure, names and colours;
- DXF/SVG export for 2D.

**3MF is the primary AM export.** It must open correctly in OrcaSlicer (the owner's FFF
slicer, multi-material via MMU, QUESTIONS #2), Bambu Studio, PrusaSlicer and Cura. It carries
multiple objects, build transforms, units, names, and per-object and per-face filament
assignment. Huge and broken files must import fast, never crash, and say exactly what was
wrong. Exports either produce valid output or fail loudly (vision principle "Robust over
clever").

Format research, slicer source findings, the paint encoding, the library evaluation and the
robustness/fuzzing plan are in the design note
[docs/notes/import-export-formats.md](../notes/import-export-formats.md).

## Scope
**In:**
- **`amg-io`.** Everything here is pure: no document types. It uses `amg-mesh`, `amg-brep`
  and `amg-sketch` data types only.
  - format registry and content sniffing;
  - `Scene` (the format-neutral import/export model);
  - unit policy and unit guess;
  - STL, 3MF, OBJ (+MTL) and PLY readers and writers;
  - 3MF slicer flavours (Orca/Bambu, Prusa);
  - slicer paint encode/decode;
  - STEP/IGES scene mapping over `amg-brep` XCAF I/O (cargo feature `brep`);
  - DXF/SVG export of 2D curves;
  - read limits;
  - fuzz targets.
- **`amg-features::io`:**
  - `ImportMeshFeature` (timeline feature holding the baked import);
  - `ImportBrepFeature` (Phase 2);
  - `scene_from_document` (Document → `Scene` for export, including occurrence transforms,
    body names and materials);
  - `import_commands` (`Scene` → `Command`s);
  - canvas extension record (IO-10).
- **`amg-ui::io`:**
  - Insert/Import dialog with unit preview;
  - drag-and-drop onto the viewport;
  - Export dialog (format, selection, unit, 3MF flavour, validity report with "Repair first");
  - progress and cancel through jobs.

**Out:**
- DXF/SVG **import** into sketches → S03 SK-11. This spec recommends the `dxf` and `usvg`
  crates to it.
- B-rep STEP/IGES reading/writing and XCAF → S02 BREP-13 (`amg_brep::io`, `ForeignModel`). S12
  maps the result to and from components and bodies.
- Tessellation algorithms → S02 BREP-08. S12 only exposes quality presets (IO-09).
- Mesh repair → S07 MESH-04. Export offers it but doesn't implement it.
- Plate layout, printer/filament model and the "Send to slicer" button → S16 (AM-01, AM-06).
  S12 writes what `Scene::plates`/`filaments` say.
- Headless thumbnail rendering → S10 RND-09. S12 accepts RGBA pixels.
- CLI `amalgam convert/export` → S18 SCR-05, calling `amg_io`/`amg_features::io`.
- Native `.amg` files → S01. Fusion `.f3d` → non-goal (vision).

## Design

### 1. Layering and data flow
```
file ──sniff──► amg_io::import(Source, Format?, ImportOptions, &JobCtx) ──► Scene (+ ImportReport)
                                                                              │
              amg_features::io::import_commands(scene, choices) ─────────────► Command(s)
                                                                              │ Document::apply
                                    ImportMeshFeature (baked raw meshes, unit, up-axis, placement)

Document ──amg_features::io::scene_from_document(selection, quality)──► Scene
Scene ──amg_io::check_export(scene, format, opts)──► ExportCheck (validity per object)
Scene ──amg_io::export(scene, Sink, format, ExportOptions, &JobCtx)──► file (atomic tmp+rename)
```
- `amg-io` is in the domain layer. It never sees `amg-doc`, so the CLI, Python and tests use
  it directly.
- The document adapter lives in `amg-features::io`, the only crate that sees both sides.
- Every entry point takes `&JobCtx`:
  - it checks cancellation at least every ~10 ms of work, per chunk;
  - it reports progress as bytes processed / total;
  - it returns `IoError::Cancelled`.

  `amg-ui::io` spawns `Priority::Interactive` jobs for files under 50 MB and `Background` jobs
  otherwise, and polls `JobHandle::try_take`. The UI thread never parses (vision: 16 ms rule).

### 2. `Scene`: one model for import and export
- **`objects`.** Geometry prototypes, each holding:
  - `Arc<IndexedMesh>` in **object space, in mm**;
  - name, `ObjectKind` (3MF `model | solidsupport | support | surface | other`);
  - an optional default material slot;
  - optional per-face material slots (`Arc<[u16]>`; `NO_SLOT` = object default).
- **`roots`.** Top-level `SceneNode`s: a transform plus `Mesh(object)` or `Group(children)`.
  3MF build items map to roots, components to groups, STEP assemblies to groups, and STL/OBJ/PLY
  to one root per object with identity transforms.
- **`materials`.** `MaterialSlot { name, color: Rgba8, filament: Option<u8> }`. This is
  3MF base materials / colour groups, OBJ MTL entries, and slicer filament slots.
- **`metadata`.** Ordered `BTreeMap`: title, designer, description, licence, application,
  plus unknown names kept verbatim with their namespace prefix.
- **`plates`.** Optional plate list, slicer flavours only.
- **`thumbnail`.** Optional, RGBA8 or encoded PNG.
- **Units.** `Scene` is always mm. `ImportReport` records the declared unit, the applied unit
  and the scale.
- **Per-face materials in the document.** `amg-features::io` stores the import as raw
  `IndexedMesh` + slot table inside the feature's baked payload. Per-face slots go into
  `MeshAttrs::tri_properties` as `TriProperty { group: <slot table id>, index: [slot; 3] }`, so
  every S07 op propagates them (face attributes inherit from the parent face). The slot table
  itself needs a home on the mesh. See **Needed from other specs, S07** (an additive
  `MeshAttrs::property_groups`). Until that lands, per-face slots survive only on unmodified
  imported bodies, and IO-14 says so in a health warning.

### 3. Format registry and sniffing (IO-01)
- `Format::sniff(head, len)` looks at content and never trusts the extension:
  - **3MF:** zip magic `PK\x03\x04` plus a `[Content_Types].xml` or `3D/` entry found when the
    central directory is read. A zip with `design.json` + `manifest.json` is `.amg`, which is
    not ours: `UnknownFormat`, with a hint to use File > Open.
  - **STL:** the §5.1 rules of the note.
  - **OBJ:** first non-comment token in `v`/`vn`/`vt`/`f`/`o`/`g`/`mtllib`.
  - **PLY:** `ply\n` or `ply\r\n`.
  - **STEP:** `ISO-10303-21;`.
  - **IGES:** column 73 is `S` on line 1, and lines are 80 characters.
  - **DXF:** `0\nSECTION`; **SVG:** `<svg` / `<?xml … <svg`.
- An extension/content mismatch gives `IoWarning::ExtensionMismatch`, and the content wins.
- `registry()` returns the formats available in this build. STEP/IGES need cargo feature
  `brep`, which is on in `amg-app`/`amg-cli`. When a format is unavailable,
  `IoError::Unsupported` says why.

### 4. Units, up-axis, placement (IO-01, IO-14)
- Policy is in note §7. In short:
  - declared units are used;
  - unitless files import as mm;
  - `guess_unit` *suggests* m/µm by extent;
  - the dialog shows dimensions under mm and inch interpretations, plus a build-volume fit when
    a print setup exists (AM-01);
  - **nothing is ever rescaled silently.**
- `ImportMeshFeature` stores **raw file numbers** (as read, no scale) and the parameters
  `unit: LengthUnit`, `up_axis: UpAxis` and `placement: Transform`. `compute()` applies
  `scale · up_axis · placement`. Changing the unit after import is an exact recompute with no
  re-read. This is Fusion's "Insert Mesh" behaviour. Scale factors are exact (note §7).
- `UpAxis::{Z, Y}` defaults to Z for every format. The UI remembers the last choice per format.
- Export writes mm by default, or the chosen `ExportOptions::unit`:
  - 3MF writes the `unit` attribute and keeps the numbers in that unit;
  - STL/OBJ/PLY scale the numbers.

### 5. STL (IO-02)
Detection, quirks and the streaming exact-weld pipeline are in note §5.
- Read produces one object, or one per ASCII `solid` block with `split_solids`.
- **Weld modes** (`StlWeld`):
  - `Exact` (default): bitwise f32 equality with `-0 = +0`; lossless and deterministic.
  - `Tolerance(mm)`: delegates to `IndexedMesh::weld`.
  - `None`: soup; 3 vertices per triangle.
- Degenerate triangles are kept and counted. NaN/inf triangles are dropped and counted.
- Optional 15-bit face colours (VisCAM/Magics) become per-face slots.
- **Write:**
  - binary by default, ASCII optional;
  - one solid; several objects are merged in world space (`merge = true`, default), or one
    file per object with `<name>_<n>.stl` (`merge = false`);
  - the header is `Amalgam <version> STL` and never starts with `solid`;
  - the facet normal is computed from winding (zero for degenerate faces);
  - the attribute word is 0 unless face colours are requested.

### 6. 3MF (IO-03 read, IO-11 write, IO-04/IO-12 flavours)
- **Read.**
  - OPC with ZIP64 and data descriptors; relationships resolved relative to their part;
    case-insensitive part names.
  - Core 1.4: units, objects of all types, components (acyclic, depth ≤ 32), build items,
    metadata, thumbnails.
  - **Production extension** `p:path` multi-part models, which Bambu/Orca projects need.
  - Base materials and **Materials extension colour groups**, object- and triangle-level.
  - Texture groups become UVs into `MeshAttrs::corner_uvs`, with a warning that the image is
    dropped until IO-19. Composites and multiproperties are flattened, with a warning.
  - Slicer extras, behind `read_slicer_extras` (default on):
    - Orca `paint_color` and Prusa `slic3rpe:mmu_segmentation` decode to per-face filament
      slots (note §4);
    - Orca `Metadata/model_settings.config` gives object/part names and extruder ids;
    - Prusa `Metadata/Slic3r_PE_model.config` gives names and extruder ids.
  - `requiredextensions` that we don't support give `UnsupportedRequiredExtension`. Encrypted
    parts give `Encrypted`.
  - With `strict = true` (used by the conformance test), every spec MUST is checked, including
    manifoldness of `model` meshes. In normal mode non-manifold `model` meshes load, with a
    warning, because repair happens downstream.
- **Write (IO-11, Generic flavour).**
  - One `/3D/3dmodel.model`, `unit="millimeter"` (or the chosen unit), `xml:lang="en-US"`.
  - `Application = Amalgam-<version>`. We never write another producer's name (note §3.1).
  - `basematerials` for slots, object `pid` for the default slot, and triangle `pid/p1` only
    for faces whose slot differs.
  - Components when the scene has groups; otherwise one object per mesh plus build items.
  - `Metadata/thumbnail.png` with the package thumbnail relationship when a thumbnail is
    given.
  - **Deterministic bytes:** fixed entry order (`[Content_Types].xml`, `_rels/.rels`, model,
    metadata), zip timestamps fixed at 1980-01-01, fixed Deflate level (default 1), ordered
    maps only.
  - Plain zip unless ZIP64 is needed.
  - Coordinates use `CoordPrecision::Decimals(6)` by default (1e-6 mm = `Tolerance` linear
    default), or `Roundtrip` (shortest round-trip f64). `-0` is normalised and non-finite
    values are rejected before writing.
- **Validity gate (`check_export`).** Note §2.3: `model`/`solidsupport` meshes must be
  manifold, consistently oriented and outward-facing. Before writing, `check_export` runs
  MESH-02 analysis per object (cached in `MeshBody`). With `ValidityPolicy::Require` (default)
  any object with boundary or non-manifold edges, inconsistent orientation or negative volume
  fails with `IoError::InvalidForFormat`, listing the objects and counts. The dialog offers
  "Repair first" (MESH-04) and "Export anyway". "Export anyway" uses
  `ValidityPolicy::AllowInvalid`, which still writes `type="model"` but adds an `ExportReport`
  warning. STL/OBJ/PLY use the same check but default to `Warn`.
- **Flavours** (note §3.5):
  - **`Orca`** (IO-04, also Bambu Studio):
    - per-filament single-colour `m:colorgroup` in slot order plus object `pid`, so Orca
      assigns extruders by colour;
    - `paint_color` whole-triangle states for per-face filaments;
    - `Metadata/model_settings.config` with object names, per-object/per-part `extruder`
      and plate membership.
  - **`Prusa`** (IO-12, P1):
    - `slic3rpe:mmu_segmentation` plus `Metadata/Slic3r_PE_model.config`;
    - re-checked against the then-current PrusaSlicer release (the facets JSON migration risk,
      note §3.2).
  - Generic output must pass lib3mf strict + XSD. Orca output must pass them after the
    `paint_color` attributes and `Metadata/model_settings.config` are removed (the test does
    exactly that).

### 7. OBJ, PLY (IO-05, IO-13)
- Own parsers, with the oracle crates as dev-deps only (note §6, §8).
- **OBJ:** `o`/`g` give objects (default) or face groups (`FaceGroupInfo { name, origin:
  Import { object } }`). `usemtl` gives per-face slots with the MTL `Kd`. Vertex colours become
  `MeshAttrs::vertex_colors`. `vt` becomes `corner_uvs`, and `vn` is dropped (normals are
  recomputed by S07).
- **OBJ write:** `o` per object, an optional `.mtl` next to it (`Kd` per slot, `usemtl`
  runs), optional `vn` (area-weighted), and the same precision options as 3MF.
- **PLY read:** ASCII and both binary endiannesses, with generic elements. Vertex/face colours
  and UVs are read. A point cloud gives `PointCloud`.
- **PLY write:** `binary_little_endian` (default) or ASCII, `float` or `double` positions
  (default `float`, warning if precision is lost beyond 1e-6 mm), and `uchar` RGBA vertex
  colours when present.

### 8. STEP / IGES (IO-06, IO-07; Phase 2/3)
- **Read.** S02 BREP-13 reads an XCAF document into a plain-data `amg_brep::ForeignModel` (a
  tree of `ForeignNode`s: name, transform, linear RGBA colour, optional `Shape`, per-face
  colours, shared-prototype index; units already mm). `amg_io::brep::scene_from_foreign` maps
  it to a `Scene`-like B-rep scene (`BrepScene`: nodes with `amg_brep::Shape` prototypes,
  names, sRGB colours and per-face colours). Shared prototypes stay shared.
  `amg-features::io` turns the result into:
  - components and occurrences (ASM-01) when the file has an assembly structure; otherwise
    bodies in the active component;
  - one `ImportBrepFeature` holding the baked shapes;
  - body names, and colours as appearance overrides once MAT-02 exists.
- **Units.** The STEP file's units always apply (OCCT converts to mm via
  `xstep.cascade.unit = MM`).
- **Write.** Selection → `ForeignModel` (occurrence tree, names, body colours). The schema is
  AP242 by default (AP203/AP214 selectable), in mm.
- **Healing:** `ShapeFix` on import (`ImportOpts::heal`, BREP-13). The health report lists fixed and failed
  shapes.
- IGES uses the same path with `read_iges`/`write_iges`. It has no assembly semantics
  (bodies only).

### 9. Tessellated export quality (IO-09; Phase 2)
- Solid bodies are exported to mesh formats through BREP-08 `tessellate` with S02's
  `TessQuality { linear, angular, relative, min_size }`. IO-09 owns the user-facing
  `ExportRefinement` presets, modelled on Fusion's refinement settings, and maps them onto
  those fields (`relative = false`):

  | Preset | Surface deviation → `linear` | Normal deviation → `angular` |
  |---|---|---|
  | Low | 0.1 mm | 30° |
  | Medium (default) | 0.025 mm | 15° |
  | High | 0.005 mm | 10° |
  | Custom | any | any |

  Presets can be set per body. Fusion's extra "max edge length" and "aspect ratio" controls
  need S02 support (P1 request). Until then, the dialog doesn't show them.
- The adapter caches tessellations by (body cache key, quality), so repeated exports are
  instant.
- S02's `TessMesh::to_indexed_parts` is welded and watertight for closed solids (S02 §8). That
  guarantee is what lets closed solids pass the 3MF validity gate, and IO-09 tests it.

### 10. 2D export (IO-08; Phase 3)
- `Drawing2d { unit, layers: [{ name, color, construction, curves: [Curve2] }] }`.
- Sources:
  - a sketch (all entities, construction curves on their own layer);
  - a planar B-rep face projected into its plane: its edges via S02 BREP-17 `project_edges` →
    `geom::CurveSeg`, converted to `Curve2`.
- **DXF** via the `dxf` crate, R2000 by default (R12 optional), with `$INSUNITS` set. Lines,
  circles and arcs stay native. Ellipses become `ELLIPSE` (R2000+) or polylines (R12).
  B-splines become `SPLINE` (R2000+) or polylines (R12).
- **SVG** is hand-written XML:
  - `width`/`height` in mm and a `viewBox`, Y flipped;
  - paths with `A` arcs and cubic Béziers (B-spline → Bézier decomposition for degree ≤ 3,
    otherwise sampled within 1e-3 mm).

### 11. Canvas (IO-10; Phase 3)
A Fusion-style calibrated reference image:
- **Stored as** an `ExtRecord` of kind `io.canvas` (S01 §11). It holds the PNG/JPEG blob,
  the plane `EntityRef` (or a construction plane), a placement transform, opacity, the
  display-through flag, and a 2-point calibration (two image pixels + real distance → scale).
- **Decoding:** `image` crate (PNG, JPEG).
- **Rendering:** needs a textured quad in `RenderScene` (request to S10).

### 12. Robustness and fuzzing (all readers; IO-16)
- Limits, the allocation rule, the degenerate/hostile input table and the fuzz plan are in note
  §9. Every reader WP lists its quirk cases in its tests.
- IO-16 adds the `fuzz/` project, the nightly CI job and the regression replay.

### 13. Determinism
- The same input file plus the same options give a bit-identical `Scene`:
  - first-seen vertex order;
  - chunk-ordered commits;
  - ordered maps;
  - independent of the rayon thread count.
- The same `Scene` plus options give byte-identical output files.
- Both properties are tested (1 vs 16 threads, two runs). This keeps the S01 cache keys of
  `ImportMeshFeature` stable.

## Interfaces
Signatures other crates rely on (`amg-features::io`, `amg-ui::io`, `amg-am`, `amg-script`,
`amg-cli`). Types from S00: `Transform` (`DAffine3`), `Aabb3`, `LengthUnit`, `JobCtx`,
`Cancelled`, `Diagnostic`, `Severity`. Types from S07: `IndexedMesh`, `Rgba8`, `MeshError`.

```rust
// ───────────────────────── amg_io (lib.rs re-exports) ─────────────────────────
use std::{collections::BTreeMap, path::{Path, PathBuf}, sync::Arc, time::Duration};
use amg_core::{units::LengthUnit, Aabb3, Diagnostic, Severity, Transform};
use amg_jobs::JobCtx;
use amg_mesh::{IndexedMesh, MeshError, Rgba8};

// ── formats ──
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
#[non_exhaustive]
pub enum Format { Stl, ThreeMf, Obj, Ply, Step, Iges, Dxf, Svg }

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum GeometryKind { Mesh, Brep, Curves2d }
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum UnitDeclaration { Always, Sometimes, Never }

#[derive(Copy, Clone, Debug)]
pub struct FormatInfo {
    pub format: Format,
    pub name: &'static str,                 // "3MF (3D Manufacturing Format)"
    pub extensions: &'static [&'static str], // lower-case, no dot
    pub kind: GeometryKind,
    pub units: UnitDeclaration,
    pub can_import: bool,                   // in this build (features, implemented WPs)
    pub can_export: bool,
    pub multi_object: bool,
    pub colors: bool,
}
impl Format {
    pub fn info(self) -> &'static FormatInfo;
    pub fn from_extension(ext: &str) -> Option<Format>;   // case-insensitive, "stl" or ".stl"
    /// Content sniffing (note §5.1, spec §3). `head` = first ≤ 4 KiB; zip formats need `probe`.
    pub fn sniff(head: &[u8], file_len: u64) -> Option<Format>;
}
/// Formats available in this build, for dialogs.
pub fn registry() -> &'static [FormatInfo];

// ── sources / sinks ──
pub enum Source<'a> { Path(&'a Path), Bytes(&'a [u8]) }
pub enum Sink<'a> {
    /// Written to `<path>.tmp` in the same directory, fsynced, renamed over `path`.
    Path(&'a Path),
    Memory(&'a mut Vec<u8>),
}

// ── scene ──
pub const NO_SLOT: u16 = u16::MAX;
pub type SlotIndex = u16;

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct MaterialSlot {
    pub name: String,                 // "PLA Red", "Material 2", 3MF base name, MTL name
    pub color: Rgba8,                 // sRGB + alpha
    pub filament: Option<u8>,         // 1-based slicer filament/extruder; None = by slot order
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Default, serde::Serialize, serde::Deserialize)]
pub enum ObjectKind { #[default] Model, SolidSupport, Support, Surface, Other }

#[derive(Clone, Debug)]
pub struct SceneObject {
    pub name: Option<String>,
    pub mesh: Arc<IndexedMesh>,               // object space, mm
    pub kind: ObjectKind,
    pub material: Option<SlotIndex>,          // object default
    pub filament: Option<u8>,                 // 1-based slicer extruder; overrides materials[material].filament
    pub face_materials: Option<Arc<[SlotIndex]>>, // len == mesh.face_count(); NO_SLOT = default
    pub part_subtype: PartSubtype,            // slicer part role (Orca/Prusa), default NormalPart
    pub source_id: Option<String>,            // 3MF object id / p:UUID / OBJ object name (diagnostics)
}
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum PartSubtype { #[default] NormalPart, NegativePart, Modifier, SupportEnforcer, SupportBlocker }

#[derive(Clone, Debug)]
pub struct SceneNode {
    pub name: Option<String>,
    pub transform: Transform,                 // parent ← node, mm
    pub content: NodeContent,
}
#[derive(Clone, Debug)]
pub enum NodeContent { Mesh(u32 /* index into objects */), Group(Vec<SceneNode>) }

#[derive(Clone, Debug, Default)]
pub struct Plate { pub name: Option<String>, pub roots: Vec<u32 /* index into Scene::roots */> }

#[derive(Clone, Debug)]
pub enum Thumbnail { Png(Arc<[u8]>), Rgba8 { width: u32, height: u32, pixels: Arc<[u8]> } }

#[derive(Clone, Debug, Default)]
pub struct Scene {
    pub objects: Vec<SceneObject>,
    pub roots: Vec<SceneNode>,                // 3MF build items, STEP top-level occurrences
    pub materials: Vec<MaterialSlot>,
    pub metadata: BTreeMap<String, String>,   // keys: core names or "ns:name"
    pub plates: Vec<Plate>,                   // empty = everything on plate 1
    pub thumbnail: Option<Thumbnail>,
}
impl Scene {
    /// Every (object, world transform) pair, depth-first, deterministic order.
    pub fn instances(&self) -> Vec<(u32, Transform)>;
    pub fn world_aabb(&self) -> Aabb3;
    pub fn triangle_count(&self) -> u64;      // over instances
    /// Merge all instances into one mesh in world space (STL write, `merge_objects`).
    pub fn flatten(&self, ctx: &JobCtx) -> Result<(IndexedMesh, Option<Vec<SlotIndex>>), IoError>;
    /// Exact uniform rescale of every mesh and translation (unit change).
    pub fn rescale(&mut self, from: LengthUnit, to: LengthUnit);
    pub fn validate(&self) -> Result<(), IoError>;  // indices, slot ranges, acyclic, lengths
}

// ── units ──
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum UpAxis { #[default] Z, Y }
impl UpAxis { pub fn to_z_up(self) -> Transform; }   // Y: +90° about X; Z: identity

#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum UnitChoice {
    /// Use the file's declared unit; unitless files use `fallback` (default mm).
    FromFile { fallback: LengthUnit },
    /// Ignore the declaration and interpret numbers in this unit.
    Assume(LengthUnit),
}
impl Default for UnitChoice { fn default() -> Self { UnitChoice::FromFile { fallback: LengthUnit::Millimetre } } }

#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Confidence { Low, Medium, High }
#[derive(Clone, Debug, PartialEq)]
pub struct UnitGuess {
    pub declared: Option<LengthUnit>,
    pub suggested: LengthUnit,
    pub confidence: Confidence,
    /// Extents (x, y, z) in mm under each plausible interpretation, for the dialog.
    pub candidates: Vec<(LengthUnit, [f64; 3])>,
}
/// `raw_extents` = bbox size in file numbers (before any scale). Rules in note §7.
pub fn guess_unit(raw_extents: [f64; 3], declared: Option<LengthUnit>) -> UnitGuess;
/// Exact factor file-unit → mm (µm handled by division; note §7).
pub fn scale_to_mm(v: f64, unit: LengthUnit) -> f64;

// ── options ──
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ReadLimits {
    pub max_triangles: u64,               // 250_000_000
    pub max_vertices: u64,                // 250_000_000
    pub max_entry_uncompressed: u64,      // 8 GiB
    pub max_total_uncompressed: u64,      // 16 GiB
    pub max_compression_ratio: u32,       // 500 (entries > 64 MiB)
    pub max_zip_entries: u32,             // 100_000
    pub max_xml_depth: u32,               // 64
    pub max_attr_value_len: u32,          // 1 MiB
    pub max_component_depth: u32,         // 32
}
impl Default for ReadLimits { fn default() -> Self; }

#[derive(Copy, Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum StlWeld { Exact, Tolerance(f64 /* mm */), None }

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct StlReadOptions { pub weld: StlWeld, pub split_solids: bool, pub face_colors: bool }
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ThreeMfReadOptions {
    pub strict: bool,                     // every spec MUST checked (conformance tests)
    pub read_slicer_extras: bool,         // paint_color / mmu_segmentation / model configs
    pub keep_components: bool,            // false = flatten groups into one node per instance
}
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ObjReadOptions { pub groups_as: ObjGroups, pub load_mtl: bool }
#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ObjGroups { Objects, FaceGroups, Ignore }

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ImportOptions {
    pub unit: UnitChoice,
    pub up_axis: UpAxis,
    pub merge_objects: bool,              // one object for the whole file
    pub limits: ReadLimits,
    pub stl: StlReadOptions,
    pub threemf: ThreeMfReadOptions,
    pub obj: ObjReadOptions,
}
impl Default for ImportOptions { fn default() -> Self; }

#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum CoordPrecision { Decimals(u8), Roundtrip }
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum ValidityPolicy { #[default] Require, Warn, AllowInvalid }
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum SlicerFlavor { #[default] Generic, Orca, Prusa }
#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Compression { Stored, Deflate { level: u8 /* 1..=9 */ } }

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct StlWriteOptions { pub ascii: bool, pub merge: bool, pub face_colors: bool }
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ThreeMfWriteOptions {
    pub flavor: SlicerFlavor,
    pub compression: Compression,         // Deflate { level: 1 }
    pub write_materials: bool,            // true
    pub thumbnail_max_px: u32,            // 512 (longest side; larger RGBA input is downscaled)
}
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ObjWriteOptions { pub mtl: bool, pub normals: bool }
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PlyWriteOptions { pub binary: bool, pub double: bool }

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ExportOptions {
    pub unit: LengthUnit,                 // default mm
    pub precision: CoordPrecision,        // Decimals(6)
    pub validity: Option<ValidityPolicy>, // None = per-format default (3MF Require, others Warn)
    pub stl: StlWriteOptions,
    pub threemf: ThreeMfWriteOptions,
    pub obj: ObjWriteOptions,
    pub ply: PlyWriteOptions,
}
impl Default for ExportOptions { fn default() -> Self; }

// ── reports, warnings, errors ──
#[derive(Clone, Debug, PartialEq)]
pub enum Location {
    Byte(u64),
    Line(u64),
    Xml { part: String, line: u64, column: u64 },
    ZipEntry(String),
}

#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum IoWarning {
    ExtensionMismatch { extension: String, detected: Format },
    HeaderCountMismatch { header: u64, actual: u64 },
    Truncated { expected: u64, read: u64 },
    TrailingBytes(u64),
    NonFiniteDropped(u64),
    InvalidIndexDropped(u64),
    PolygonTriangulated { polygons: u64, fallback_fan: u64 },
    UnitAssumed(LengthUnit),
    UnknownExtensionIgnored { namespace: String },
    UnsupportedContentIgnored { what: String, count: u64 },   // textures, composites, beams…
    PaintApproximated { triangles: u64 },
    NotManifold { object: String, boundary_edges: u64, non_manifold_edges: u64 },
    PrecisionLoss { max_error_mm: f64 },
    MissingReference { what: String },                          // mtllib, texture path
}
impl IoWarning { pub fn message(&self) -> String; }

#[derive(Clone, Debug, Default)]
pub struct ImportReport {
    pub format: Option<Format>,
    pub bytes: u64,
    pub declared_unit: Option<LengthUnit>,
    pub applied_unit: Option<LengthUnit>,
    pub triangles_read: u64,
    pub vertices: u64,                    // after weld
    pub degenerate_triangles: u64,
    pub warnings: Vec<IoWarning>,
    pub duration: Duration,
}
#[derive(Clone, Debug)]
pub struct Imported { pub scene: Scene, pub report: ImportReport }

#[derive(Clone, Debug, PartialEq)]
pub struct ObjectCheck {
    pub object: u32, pub name: Option<String>,
    pub boundary_edges: u64, pub non_manifold_edges: u64,
    pub inconsistent_orientation: bool, pub negative_volume: bool,
}
#[derive(Clone, Debug, Default)]
pub struct ExportCheck { pub objects: Vec<ObjectCheck>, pub warnings: Vec<IoWarning> }
impl ExportCheck { pub fn is_valid(&self) -> bool; }

#[derive(Clone, Debug, Default)]
pub struct ExportReport { pub bytes: u64, pub triangles: u64, pub files: Vec<PathBuf>, pub warnings: Vec<IoWarning>, pub duration: Duration }

#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum IoError {
    #[error("cannot read {path}: {source}")]
    Read { path: PathBuf, #[source] source: std::io::Error },
    #[error("cannot write {path}: {source}")]
    Write { path: PathBuf, #[source] source: std::io::Error },
    #[error("unknown or unsupported file format")]
    UnknownFormat,
    #[error("{format:?} {direction} is not available: {reason}")]
    Unsupported { format: Format, direction: &'static str, reason: &'static str },
    #[error("{format:?} file is malformed at {location:?}: {message}")]
    Malformed { format: Format, location: Location, message: String },
    #[error("file exceeds the {what} limit ({actual} > {limit})")]
    LimitExceeded { what: &'static str, limit: u64, actual: u64 },
    #[error("3MF file requires the unsupported extension {namespace}")]
    UnsupportedRequiredExtension { namespace: String },
    #[error("3MF file is encrypted (Secure Content extension)")]
    Encrypted,
    #[error("file contains no usable triangles")]
    Empty,
    #[error("PLY file has vertices but no faces (point clouds are not supported)")]
    PointCloud,
    #[error("{count} object(s) are not valid closed solids for {format:?}")]
    InvalidForFormat { format: Format, count: usize, check: Box<ExportCheck> },
    #[error(transparent)]
    Mesh(#[from] MeshError),
    #[error("B-rep exchange failed: {0}")]
    Brep(String),                                   // from amg_brep::KernelError (S02 owns it)
    #[error("operation cancelled")]
    Cancelled,
}
impl From<amg_core::Cancelled> for IoError { fn from(_: amg_core::Cancelled) -> Self { IoError::Cancelled } }
impl Diagnostic for IoError { /* hints: "Repair the mesh (Mesh > Repair) or choose Export anyway",
                                 "Re-export from the slicer without encryption", "Check the unit", … */ }

// ── entry points ──
/// Header-only inspection (no geometry): format, declared unit, object/triangle estimate.
pub fn probe(src: Source<'_>, limits: &ReadLimits) -> Result<ProbeInfo, IoError>;
#[derive(Clone, Debug)]
pub struct ProbeInfo { pub format: Format, pub declared_unit: Option<LengthUnit>, pub objects: Option<u32>, pub triangles_estimate: Option<u64>, pub has_slicer_data: bool }

/// Reads any mesh format. `format = None` sniffs. Runs inside a job.
pub fn import(src: Source<'_>, format: Option<Format>, opts: &ImportOptions, ctx: &JobCtx) -> Result<Imported, IoError>;
/// Validity per object for `format` (MESH-02 analysis). Cheap when MeshBody analysis is cached.
pub fn check_export(scene: &Scene, format: Format, ctx: &JobCtx) -> Result<ExportCheck, IoError>;
/// Writes `scene`. Runs `check_export` per `opts.validity` first. Atomic for `Sink::Path`.
pub fn export(scene: &Scene, dst: Sink<'_>, format: Format, opts: &ExportOptions, ctx: &JobCtx) -> Result<ExportReport, IoError>;

// ── per-format modules (public for tests, CLI and fuzzing) ──
pub mod stl {
    pub enum StlKind { Binary { declared: u32 }, Ascii }
    pub fn detect(head: &[u8], file_len: u64) -> Result<StlKind, super::IoError>;
}
pub mod paint {
    /// Whole-triangle state (0 = object default, 1.. = filament) → slicer string ("" for 0).
    pub fn encode_state(state: u8) -> String;
    /// Decodes one `paint_color` / `mmu_segmentation` string. Returns the dominant leaf state
    /// by area (unit triangle) and whether the triangle was subdivided.
    pub fn decode_dominant(s: &str) -> Result<(u8, bool), PaintError>;
    #[derive(Debug, thiserror::Error)] #[error("invalid paint string at nibble {pos}")]
    pub struct PaintError { pub pos: usize }
}

// ── 2D export (IO-08) ──
pub mod drawing {
    use amg_sketch::geom::Curve2;
    #[derive(Clone, Debug)]
    pub struct Layer { pub name: String, pub color: amg_mesh::Rgba8, pub construction: bool, pub curves: Vec<Curve2> }
    #[derive(Clone, Debug)]
    pub struct Drawing2d { pub unit: amg_core::units::LengthUnit, pub layers: Vec<Layer> }
    #[derive(Copy, Clone, Debug, PartialEq, Eq)] pub enum DxfVersion { R12, R2000 }
    pub fn write_dxf(d: &Drawing2d, version: DxfVersion, dst: super::Sink<'_>) -> Result<(), super::IoError>;
    pub fn write_svg(d: &Drawing2d, dst: super::Sink<'_>) -> Result<(), super::IoError>;
}

// ── B-rep exchange (IO-06/IO-07; cargo feature "brep") ──
#[cfg(feature = "brep")]
pub mod brep {
    use amg_brep::{ForeignModel, Shape};
    #[derive(Clone, Debug)]
    pub struct BrepObject { pub name: Option<String>, pub shape: Shape, pub color: Option<amg_mesh::Rgba8>, pub face_colors: Vec<(u32 /* face index */, amg_mesh::Rgba8)> }
    #[derive(Clone, Debug)]
    pub struct BrepNode { pub name: Option<String>, pub transform: amg_core::Transform, pub content: BrepContent }
    #[derive(Clone, Debug)]
    pub enum BrepContent { Shape(u32), Group(Vec<BrepNode>) }
    #[derive(Clone, Debug, Default)]
    pub struct BrepScene { pub objects: Vec<BrepObject>, pub roots: Vec<BrepNode>, pub metadata: std::collections::BTreeMap<String, String> }
    #[derive(Copy, Clone, Debug, PartialEq, Eq, Default)] pub enum StepSchema { Ap203, Ap214, #[default] Ap242 }
    pub fn import_step(src: &std::path::Path, ctx: &amg_jobs::JobCtx) -> Result<(BrepScene, super::ImportReport), super::IoError>;
    pub fn export_step(scene: &BrepScene, schema: StepSchema, dst: &std::path::Path, ctx: &amg_jobs::JobCtx) -> Result<super::ExportReport, super::IoError>;
    pub fn import_iges(src: &std::path::Path, ctx: &amg_jobs::JobCtx) -> Result<(BrepScene, super::ImportReport), super::IoError>;
    pub fn export_iges(scene: &BrepScene, dst: &std::path::Path, ctx: &amg_jobs::JobCtx) -> Result<super::ExportReport, super::IoError>;
    /// Linear RGBA (S02) → sRGB `Rgba8`; shared prototypes (`instance_of`) stay shared objects.
    pub fn scene_from_foreign(m: &ForeignModel) -> BrepScene;
    pub fn foreign_from_scene(s: &BrepScene) -> ForeignModel;
}
```

```rust
// ───────────────────────── amg_features::io (lane G) ─────────────────────────
use amg_doc::{BodyId, Command, ComponentId, Design, Evaluated, FeatureError};

/// Timeline feature "io.import_mesh", v1. Holds the raw imported meshes as baked bodies
/// (slot i = object i of the import). `compute` = scale(unit) · up_axis · placement.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ImportMeshFeature {
    pub source: ImportSource,
    pub unit: amg_core::units::LengthUnit,        // editable after import
    pub up_axis: amg_io::UpAxis,                  // editable after import
    pub placement: amg_core::Transform,           // editable (S06/UI move gizmo writes it)
    pub bodies: Vec<ImportedBodySpec>,
    pub materials: Vec<amg_io::MaterialSlot>,     // slot table (until S07 property_groups lands)
}
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ImportSource {
    pub file_name: String,            // display only
    pub path_hint: Option<String>,    // last known absolute path (for "Reload", P2); not a cache input
    pub format: amg_io::Format,
    pub blake3: String,               // hex of the source bytes
    pub declared_unit: Option<amg_core::units::LengthUnit>,
}
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ImportedBodySpec { pub slot: u32, pub name: String, pub transform: amg_core::Transform /* build-item transform, file units */, pub material: Option<u16> }

/// Phase 2: "io.import_brep", v1 (baked `amg_brep::Shape`s; units already mm).
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ImportBrepFeature { pub source: ImportSource, pub placement: amg_core::Transform, pub bodies: Vec<ImportedBodySpec> }

#[derive(Clone, Debug)]
pub struct ImportChoices {
    pub unit: amg_core::units::LengthUnit,
    pub up_axis: amg_io::UpAxis,
    pub target: ComponentId,
    pub as_components: bool,          // Phase 2 (ASM-01): one component per scene root
    pub name: Option<String>,         // feature name in the timeline
}
/// Builds the commands for one undoable "Insert <file>" transaction.
pub fn import_commands(imported: amg_io::Imported, source: ImportSource, choices: &ImportChoices) -> Result<Vec<Command>, ImportPrepError>;

#[derive(Clone, Debug)]
pub enum ExportSelection { AllVisible, Bodies(Vec<BodyId>), Component(ComponentId) }
#[derive(Clone, Debug, Default)]
pub struct ExportPrep {
    pub refinement: Option<ExportRefinement>,             // IO-09; required if solids are selected
    pub per_body_refinement: std::collections::BTreeMap<BodyId, ExportRefinement>,
    pub filaments: Option<Vec<amg_io::MaterialSlot>>,     // from S16 print setup (AM-01), if any
    pub body_filament: std::collections::BTreeMap<BodyId, u8>,
}
/// Document → Scene in world space (occurrence transforms applied to roots, not to meshes).
pub fn scene_from_document(design: &Design, eval: &Evaluated, sel: &ExportSelection, prep: &ExportPrep, ctx: &amg_jobs::JobCtx) -> Result<amg_io::Scene, ImportPrepError>;

/// IO-09 presets (spec §9); `to_tess()` maps onto S02 `TessQuality { linear, angular, relative: false, .. }`.
#[derive(Copy, Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum ExportRefinement { Low, Medium, High, Custom { surface_deviation_mm: f64, normal_deviation_rad: f64 } }
impl ExportRefinement { pub fn to_tess(self) -> amg_brep::TessQuality; }

#[derive(Debug, thiserror::Error)]
pub enum ImportPrepError {
    #[error(transparent)] Io(#[from] amg_io::IoError),
    #[error("{0} solid bodies need a tessellation quality (IO-09)")] NeedsTessellation(usize),
    #[error("body {0:?} is an implicit/form body; convert it to a mesh first")] UnsupportedBody(BodyId),
    #[error("nothing selected to export")] EmptySelection,
    #[error(transparent)] Feature(#[from] FeatureError),
}
```

**Needed from other specs (requests, not edits; also in the S12 refinement report):**
- **S00:**
  - IO-01 depends on FND-10 (`LengthUnit`), the agreed repoint. No new API.
  - The `zip` workspace dep should be `default-features = false, features =
    ["deflate-flate2-zlib-rs"]` (note §8), shared with DOC-07.
- **S01:**
  1. `RecomputeCtx::baked(&self, slot: u32) -> Result<&BodyGeom, FeatureError>`, returning the
     baked payload stored for `BodyId { origin: this feature, slot }`.
  2. A way to add a feature *with* baked payloads in one undo entry, for example
     `Command::AddFeature { …, baked: Vec<(u32, BodyGeom)> }` or
     `Command::SetBaked { body, geom }` inside a `Transaction`.
  3. The baked map is persisted in `bodies/` by DOC-07 in Phase 1, not first in DOC-08 (P1,
     Phase 3).

  S01's file table already lists "imported meshes" under `bodies/`, and MESH-16 assumes baked
  mesh bodies in Phase 1.
- **S07 (additive):**
  - `MeshAttrs::property_groups: Option<Arc<PropertyGroups>>` with
    `PropertyGroups { groups: BTreeMap<u32, PropertyGroup> }` and
    `PropertyGroup::{Materials(Vec<(String, Rgba8)>), Colors(Vec<Rgba8>)}`.
  - `IndexedMesh::append` remaps group ids.
  - This gives `tri_properties` (already in S07) its table, so per-face materials survive mesh
    ops.
  - Also: `MeshBody::analysis` must be callable from `amg-io` export checks (it already is,
    via `IndexedMesh`).
- **S02** (matched to the parallel S02 refinement draft of 2026-10-20):
  1. BREP-13 `amg_brep::io::{read_step, write_step, read_iges, write_iges}` with
     `ForeignModel`/`ForeignNode` is what IO-06/IO-07 consume. Please keep:
     - `instance_of` for shared prototypes;
     - per-face colours keyed by `FaceId`;
     - `source_unit_mm` (S12 reports it as the declared unit);
     - progress/cancel through `JobCtx`.
  2. BREP-08 `TessMesh::to_indexed_parts` is welded and watertight for closed solids. IO-09's
     validity test depends on this.
     - P1: optional `max_edge_length` / `max_aspect_ratio` fields on `TessQuality`, for
       Fusion-style export refinement.
  3. BREP-17 `project_edges` / `plane_section` → `geom::CurveSeg` feed IO-08 face export.
     IO-08 needs the loops of a planar face as ordered, closed `CurveSeg` chains (outer
     first). Please confirm `project_edges(face_edges, face_plane)` returns them ordered, or add
     `planar_face_loops(&Shape, FaceId) -> KResult<(Plane, Vec<Vec<geom::CurveSeg>>)>`.
  4. `Shape: Send + Sync` (ADR-014). The S02 draft already does this.
- **S16:**
  - AM-06 should depend on **IO-11 + IO-04** (writer and Orca flavour), not IO-03 (now the
    reader). S16's requested `Model3mf`/`write_3mf` sketch maps 1:1 onto `Scene` +
    `amg_io::export(…, Format::ThreeMf, …)`:
    - `Object3mf.extruder` → `SceneObject::filament`;
    - `tri_extruder` → `face_materials` slots with `MaterialSlot::filament`;
    - `BuildItem3mf` → `Scene::roots`, and `plate` → `Scene::plates`;
    - `SlicerFlavor::Core` is named `Generic` here;
    - `W: Write + Seek` is replaced by `Sink::Path` (atomic) or `Sink::Memory`;
    - binary STL through the same `export`.
  - The print setup (AM-01) supplies `filaments` and per-body filament for
    `ExportPrep`.
  - Resin hand-off uses binary STL (note §3.4) until the Chitubox question is answered.
- **S17:**
  - `scene_from_document` uses the body's material/appearance name and colour for
    `MaterialSlot` when no print-setup filament is set. The S17 draft's `Appearance::to_3mf()`
    / `from_3mf()` (`ThreeMfDisplay`) and `Srgba8` are what IO-14 and IO-19 consume. S12
    converts `Srgba8` ↔ `amg_mesh::Rgba8` (both are sRGB + alpha).
  - MAT-01 depends on FND-10 (the agreed repoint).
- **S10:**
  - IO-10 needs a textured-quad primitive in `RenderScene`.
  - IO-15 would like an RGBA viewport capture (`capture_rgba(size) -> JobHandle<Vec<u8>>`) for
    3MF thumbnails. Until it exists, no thumbnail is written (slicers render their own).
- **S03:** SK-11 should use `dxf` 0.6.2 and `usvg` 0.48.1 (licences in note §8). It should map
  `$INSUNITS` with the same table as IO-08.

## Work packages
WPs are ≈ 1–3 days each. IO-01..10 keep their IDs (re-scoped); IO-11..19 are new. Times are
on the reference machine (vision doc), release build. "Corpus" = `testdata/io/corpus.toml`
(note §11) plus the MESH-28 mesh corpus. Every WP ends with `cargo fmt`,
`cargo clippy --workspace -- -D warnings` and `cargo nextest run -p amg-io` (plus the touched
crates) passing.

| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| IO-01 | `amg-io` foundation: `Format`/`FormatInfo`/`registry`, `sniff`/`probe` skeleton, `Source`/`Sink` (atomic tmp+rename), `Scene` + `validate`/`instances`/`flatten`/`rescale`, `UpAxis`, `UnitChoice`, `guess_unit`, `scale_to_mm`, `ReadLimits`, options structs, `IoWarning`/`ImportReport`/`IoError` + `Diagnostic`, `testdata/io/corpus.toml` schema + harness | FND-04, FND-10, MESH-01 | P0 | Todo | 1. Sniff table test: ≥ 30 fixture heads give the right `Format`, including binary STL starting with `solid`, ASCII STL with BOM, `.amg` zip (→ `UnknownFormat` with hint), STEP, IGES, DXF and SVG. 2. `guess_unit` table test covers ≥ 12 cases from note §7. `scale_to_mm` is exact for in/ft/cm/m (bit-equal to the literal products) and correctly rounded for µm. 3. `Scene::validate` rejects out-of-range objects, slots and cycles (1 test each). `flatten` of 3 instances equals the hand-built mesh. 4. `Sink::Path` leaves the original file intact when the writer errors mid-way (test hook). 5. Every `IoError` variant has a non-empty `user_message`, and every variant that has a remedy has a hint (enumerating test). 6. The corpus harness iterates the manifest under nextest and skips absent fetch-only entries. |
| IO-02 | STL read/write: detection (note §5.1), binary streaming exact weld (§5.3), ASCII parallel parse, multi-solid, face colours, write binary/ASCII, merge/split | IO-01, MESH-28 | P0 | Todo | 1. All STL entries of the MESH-28 corpus parse with the manifest triangle count. 2. 20 synthetic quirk cases (note §5.1–5.2: `solid`-headed binary, count 0, truncated, trailing bytes, NaN, CRLF/uppercase/tabs, no `endsolid`, multi-solid, BOM, quads in `outer loop`, empty) each give the expected result and warning. 3. Exact weld's vertex count equals the number of distinct normalised f32 positions (proptest vs a `BTreeSet` oracle), and the result is identical for 1 and 16 threads. 4. Binary write→read is bit-exact for f32-representable input. 5. 1 GB binary (20 M tris, MESH-28 generator) imports in **< 5 s** with peak extra heap ≤ 2.5 GB (`dhat` in the bench). 1 GB ASCII < 6 s. 20 M-tri binary write < 2 s. Criterion benches `io/stl_read_bin_1gb`, `io/stl_read_ascii_1gb`, `io/stl_write_20m`. 6. Cancel during the 1 GB import returns `Cancelled` within 100 ms. 7. Agrees with `stl_io` (dev-dep) on positions and triangle count for the corpus. |
| IO-03 | 3MF reader: OPC/zip (ZIP64, data descriptors, limits), core 1.4 (units, object types, components, build items, metadata, thumbnail), Production `p:path` multi-part, base materials + colour groups (object/triangle), texture-group UVs with warning, slicer extras read (`paint_color`, `mmu_segmentation` via `paint::decode_dominant`, Orca/Prusa model configs), `strict` mode, parallel vertex/triangle scanner | IO-01, MESH-28 | P0 | Todo | 1. 3MF Consortium suites 2/3/5/6 (pinned, fetched): every `P_*` case in scope loads in strict mode, and every `N_*` case fails with a specific `IoError`. An allow-list lists out-of-scope cases with reasons, and their count is reported. 2. Slicer fixtures (OrcaSlicer 2.4.x, Bambu Studio 2.8.x, PrusaSlicer 2.9.x CLI exports of CC0 models; `scripts/io/make_slicer_fixtures.ps1`) load with the right object count, names, build transforms and per-face filament (painted fixtures; whole-triangle states exact). 3. All 3MF entries of the MESH-28 corpus load. 4. The scanner matches pure quick-xml parsing on all fixtures (differential test). 5. A 10 M-tri 3MF reads in < 6 s (criterion `io/3mf_read_10m`). 6. Zip bomb, 100-deep XML, component cycle and an encrypted fixture each give the expected error, in < 1 s and < 100 MB RSS. 7. Cancel < 100 ms. |
| IO-11 | 3MF writer, Generic flavour: OPC parts, base materials/pids, components or flat build items, metadata (`Application = Amalgam-<ver>`), thumbnail (PNG encode + downscale), `CoordPrecision`, deterministic zip, `check_export` + `ValidityPolicy`; validation scripts `scripts/io/validate_3mf.py` (PyPI `lib3mf` strict + `xmlschema` against the Consortium consolidated schema) | IO-03, MESH-02 | P0 | Todo | 1. 50 generated scenes (proptest: objects, groups, materials, per-face slots, transforms) round-trip write→read: positions within 1e-6 mm (`Decimals(6)`) or bit-exact (`Roundtrip`), with identical indices, names, slots and transforms (within 1e-12). 2. Every written fixture passes `validate_3mf.py` (lib3mf strict with no warnings, plus XSD) in CI. 3. Exporting the same scene twice, with 1 and 16 threads, gives byte-identical files. 4. A non-watertight object fails with `InvalidForFormat` listing the object and edge counts. `AllowInvalid` writes it with a warning. 5. Manual check, recorded in the PR with screenshots: a 3-object, 2-material file opens in OrcaSlicer 2.4.x, Bambu Studio 2.8.x, PrusaSlicer 2.9.x and Cura 5.13 with correct positions, scale and names. 6. A 10 M-tri write takes < 8 s (criterion `io/3mf_write_10m`). |
| IO-04 | 3MF Orca/Bambu flavour: per-filament single-colour `m:colorgroup` + object `pid`, `paint_color` whole-triangle encode (`paint::encode_state`), `Metadata/model_settings.config` (object/part names, `extruder`, part subtypes, plates from `Scene::plates`), Orca-honoured-keys probe recorded in the note | IO-11 | P0 | Todo | 1. `paint::encode_state`/`decode_dominant` round-trip for states 0–32 and match the `CONST_FILAMENTS` table (note §4). 2. Automated Orca check (skipped if `orca-slicer` is not on PATH; required on the owner's machine): for 5 scenes (1–4 filaments; per-object and per-face; 2 plates), `orca-slicer --info` lists the expected objects. In the `--export-3mf` re-export, `model_settings.config` shows the expected `extruder` per object, and the `paint_color` histogram of the model parts matches the painted-face filament histogram. 3. Manual check: the 4-filament per-face test part opens in OrcaSlicer 2.4.x and Bambu Studio 2.8.x with the painted faces on the right filaments (screenshots in PR). 4. With `paint_color` and `model_settings.config` removed, the output passes `validate_3mf.py`. |
| IO-05 | OBJ (+MTL) read/write: own parser (two-pass parallel, negative indices, polygons via ear clipping, vertex colours, `vt`, `o`/`g` options, `usemtl` slots, MTL `Kd`/`d`), writer with optional MTL and normals | IO-01, MESH-28 | P0 | Todo | 1. All OBJ entries of the MESH-28 corpus parse with the manifest face count. 2. 15 synthetic quirk cases (negative indices, `v/vt/vn` mixes, concave polygons, degenerate polygons, missing `mtllib`, line continuation, CRLF, vertex colours, `o` without faces) each give the expected result and warning. 3. Concave polygon triangulation preserves the polygon area within 1e-12 relative and has no flipped triangles (proptest, 1000 random simple polygons). 4. Write→read round-trips positions (`Roundtrip` bit-exact), indices, objects and slots. 5. Agrees with `tobj` on triangle count and positions on convex corpus files. 6. 1 GB OBJ reads in < 8 s (criterion `io/obj_read_1gb`). |
| IO-13 | PLY read/write: header (64 KiB limit), ASCII + binary LE/BE, generic elements/properties, vertex/face colours, UVs, point-cloud error; writer binary LE/ASCII, float/double | IO-01, MESH-28 | P0 | Todo | 1. All PLY entries of the MESH-28 corpus parse with the manifest face count. 2. 12 synthetic quirk cases (BE, ASCII, uchar/ushort/uint list counts, int/uint indices, extra elements, colours as uchar and float, no faces → `PointCloud`, truncated body, header without `end_header`) behave as specified. 3. Write→read round-trip (double: bit-exact; float: ≤ 1 f32 ulp, with `PrecisionLoss` reported). 4. Agrees with `ply-rs-bw` on the corpus. 5. 1 GB binary PLY reads in < 3 s. |
| IO-14 | `amg-features::io`: `ImportMeshFeature` (`io.import_mesh` v1, raw baked meshes, unit/up-axis/placement params), `import_commands` (one transaction: feature + baked payloads + body names), `scene_from_document` (selection, occurrence transforms, `ExportPrep` filaments and materials; mesh bodies), `link()` | IO-01, DOC-05, DOC-07, MESH-16 | P0 | Todo | 1. Importing a corpus STL, 3MF (3 objects) and OBJ creates one timeline feature and the expected bodies. Undo removes them all, and redo restores them with an identical `content_hash`. 2. Editing `unit` mm→in rescales exactly (bbox ×25.4 bit-exact), and only dependants recompute (counter test). 3. Save→load of the `.amg` (DOC-07) restores bit-identical meshes from `bodies/`. 4. `scene_from_document` of an imported 3MF re-exports (IO-11) to a file whose scene equals the original import (objects, transforms within 1e-12, slots). 5. The feature's cache key is identical across two processes. *Needs the S01 baked-payload API (Interfaces requests).* |
| IO-15 | `amg-ui::io`: File > Insert Mesh / Import, drag-and-drop onto the viewport, import dialog (format, declared/assumed unit dropdown with live mm/in extents and print-bed fit, up-axis, merge/separate), export dialog (format, selection, unit, 3MF flavour, validity report with "Repair first" (MESH-04) / "Export anyway"), job progress + cancel, remembered per-format choices | IO-14, IO-11, MESH-04, UI-01, UI-02, UI-10 | P0 | Todo | 1. Dropping each of STL/3MF/OBJ/PLY onto the viewport imports it. Dropping an unsupported file shows the `IoError` message and hint. 2. While a 1 GB STL imports, no frame exceeds 16 ms (tracing profile) and Cancel stops it within 200 ms. 3. Changing the unit in the dialog updates the shown extents without re-reading the file. 4. Exporting a body with holes shows the validity report, and "Repair first" runs MESH-04 and then exports successfully. 5. A manual Phase 1 script (drag STL → analyse → repair → export 3MF Orca flavour → open in Orca) passes on 3 corpus models. |
| IO-16 | Robustness: `fuzz/` cargo-fuzz project (targets `stl`, `obj`, `mtl`, `ply`, `threemf_package`, `threemf_model`, `paint_decode`, `roundtrip_scene`), seed corpora, scheduled Linux nightly CI job (5 min/target, `-rss_limit_mb=2048 -timeout=2`), minimised crash artifacts in `testdata/io/fuzz/<target>/`, stable regression-replay test | IO-02, IO-03, IO-05, IO-13 | P0 | Todo | 1. Each target runs ≥ 1 h locally (or ≥ 10 M execs) with 0 crashes, 0 OOMs and 0 timeouts. The exec counts are in the PR. 2. `roundtrip_scene` holds for every generated scene (export→import equality as in IO-11 #1). 3. The replay test runs under `cargo nextest` on Windows stable and replays every artifact. 4. The CI job is green for 3 consecutive nights. 5. Every bug found has a fix plus a minimised artifact. |
| IO-06 | STEP AP203/214/242 import/export with assemblies, names, colours: `amg_io::brep` (`scene_from_foreign`/`foreign_from_scene`, `import_step`/`export_step`), `ImportBrepFeature`, components via ASM-01, health report of healed shapes | BREP-13, ASM-01, IO-14 | P0 | Todo | 1. The NIST MBE PMI STEP models (public, fetch-only) import with the right component tree, part names and colours (compared with the `ForeignModel` tree), and every body passes BRepCheck. 2. Round trip import → export AP242 → import keeps the component tree, names, colours, volumes (within 1e-9 relative) and bboxes (within 1e-6 mm). 3. The owner's Fusion STEP exports under `$AMG_TESTDATA_DIR/fusion/` (QUESTIONS #4) round-trip with the same checks (skipped when absent). 4. A 50 MB STEP imports with progress and cancels within 500 ms (OCCT progress range). |
| IO-09 | Tessellated export quality: `ExportRefinement` presets Low/Medium/High/Custom per body (→ S02 `TessQuality`), tessellation cache by (body key, quality), solid bodies in `scene_from_document`, deviation report | BREP-08, IO-14 | P0 | Todo | 1. For 10 golden solids (S02 corpus), the measured max deviation (`amg-testkit` Hausdorff against a High+ reference) is ≤ the preset's surface deviation. 2. Every closed solid's tessellation passes `check_export` (watertight) and IO-11 validation. 3. A second export of an unchanged body hits the cache (no tessellation call, counter test). 4. A 1000-face part exports to 3MF Medium in < 300 ms end-to-end. |
| IO-07 | IGES import/export (bodies only, no assemblies): `import_iges`/`export_iges` | BREP-13, IO-06 | P1 | Todo | 1. 5 public IGES samples import, with bodies passing BRepCheck after healing (failures listed in the health report). 2. Export→import round trip preserves volume within 1e-6 relative for 10 golden solids. |
| IO-08 | DXF/SVG export of sketches and planar faces: `amg_io::drawing` (`write_dxf` R12/R2000 via `dxf`, `write_svg`), sketch and face sources (S02 BREP-17 `project_edges`) | SK-01, IO-01, BREP-17 | P1 | Todo | 1. 10 test sketches (lines, arcs, circles, ellipses, splines; construction on a separate layer) export to DXF R2000 and R12. Re-reading them with the `dxf` crate gives the same entities within 1e-9 mm (R12 polylines within 1e-3 mm). 2. SVG output parsed by `usvg` has a bbox within 1e-6 mm of the sketch bbox and the correct Y orientation. 3. A planar face with 2 holes exports 3 closed loops. 4. `$INSUNITS` is correct for mm and inch. |
| IO-10 | Canvas: `io.canvas` extension record (image blob, plane, placement, opacity, 2-point calibration), insert/calibrate UI in `amg-ui::io`, PNG/JPEG decode (`image`) | RND-03, DOC-05, UI-04 | P1 | Todo | 1. Inserting a PNG on the XY plane shows it at the expected size. After 2-point calibration (image distance → 50 mm), a measured distance on the canvas is within 0.1 % of 50 mm. 2. Opacity and calibration are undoable and survive save/load. 3. A 20 MP JPEG decodes in a job without frames > 16 ms. *Needs S10's textured quad.* |
| IO-12 | 3MF Prusa flavour + per-object slicer settings: `slic3rpe:mmu_segmentation`, `Metadata/Slic3r_PE_model.config` (names, extruder per volume), re-checked against the then-current PrusaSlicer release (facets-JSON migration, note §3.2); optional per-object layer height / infill overrides for Orca and Prusa from S16 | IO-04 | P1 | Todo | 1. `prusa-slicer-console --info` on 5 scenes reports the expected object and facet counts. Manual check in the current PrusaSlicer release: per-object and per-face extruders are correct (screenshots). 2. Prusa-flavour output passes `validate_3mf.py`. 3. Per-object overrides appear in Orca/Prusa's object settings list (manual check). |
| IO-18 | 3MF Beam Lattice: read (beams/balls → capsule mesh, with a warning) and write S08 strut lattices as beams | IO-11, IMP-06 | P2 | Todo | 1. Consortium suite 7 (beam lattice) `P_*` cases load and `N_*` cases fail. 2. A 10 k-beam lattice writes and validates with lib3mf, and its beam count and radii round-trip exactly. |
| IO-19 | 3MF full Materials extension: texture2d images, composites, multiproperties, PBR display properties → S17 appearances | IO-03, MAT-02 | P2 | Todo | 1. Consortium suites 2/6 cases that were allow-listed in IO-03 for materials now pass. 2. A textured fixture round-trips with byte-identical texture images and UVs within 1e-6. |

Status values: `Todo`, `Refining`, `Ready`, `In progress (agent/branch)`, `Review`, `Done`,
`Blocked (reason)`.

Proposed phase placement (the roadmap is not edited here; see the Refinement log):
- **Phase 1:** IO-01, IO-02, IO-03, IO-11, IO-04, IO-05, IO-13, IO-14, IO-15, IO-16.
- **Phase 2:** IO-06, IO-09.
- **Phase 3:** IO-07, IO-08, IO-10, IO-12.
- **Phase 5:** IO-18, IO-19.

## Testing
- **Unit tests** per reader/writer, with each quirk table from the note §5, §6 and §9.2 as
  individual cases.
- **Round-trip property tests** (`proptest`): Scene → each writer → reader equals the original
  within the format's precision (STL: f32; 3MF/OBJ: `Decimals(6)` or `Roundtrip`; PLY:
  float/double).
- **Conformance:** 3MF Consortium test suites (positive and negative). Exported Generic files
  are validated with PyPI `lib3mf` (strict) and `xmlschema` against the consolidated schema in
  CI (`scripts/io/validate_3mf.py`; Python is a dev dependency).
- **Differential oracles** (dev-deps only): `stl_io`, `tobj`, `ply-rs-bw`; the 3MF scanner vs
  quick-xml.
- **Corpus:**
  - `testdata/io/corpus.toml` (synthetic, Consortium, slicer fixtures) and the MESH-28 mesh
    corpus;
  - golden metrics in `tests/golden/io/<format>/<case>.toml` (volume, area, bbox, counts via
    `amg-testkit`).
- **Slicer checks:**
  - automated where a CLI exists: `orca-slicer --info` / `--export-3mf` (IO-04),
    `prusa-slicer-console --info` (IO-12), skipped when the binary is absent;
  - manual screenshots for Bambu Studio, Cura and Chitubox.
- **Determinism:** 1 vs 16 rayon threads and two runs give a bit-identical `Scene` and
  byte-identical output.
- **Fuzzing (IO-16):** nightly Linux libFuzzer, plus stable regression replay on Windows.
- **Benchmarks** (criterion, `amg-io/benches/`): `io/stl_read_bin_1gb`,
  `io/stl_read_ascii_1gb`, `io/stl_write_20m`, `io/3mf_read_10m`, `io/3mf_write_10m`,
  `io/obj_read_1gb`, `io/ply_read_1gb`. Input files are generated on first run into
  `target/bench-data/` (not committed).

## Performance targets
Reference machine (vision doc), release build. Import time for large meshes isn't a vision
budget row yet; the refinement report proposes adding "1 GB binary STL import < 5 s".

| Scenario | Target |
|---|---|
| Binary STL 1 GB (20 M tris) → `IndexedMesh`, exact weld | < 5 s, peak extra heap ≤ 2.5 GB |
| ASCII STL 1 GB | < 6 s |
| Binary STL write 20 M tris | < 2 s |
| 3MF read 10 M tris | < 6 s |
| 3MF write 10 M tris (Deflate 1) | < 8 s |
| OBJ read 1 GB | < 8 s |
| PLY binary read 1 GB | < 3 s |
| 100 k-tri file → body in document (excluding first render) | < 300 ms |
| Cancel latency, any import/export | < 100 ms (UI shows it within 200 ms) |
| UI thread during any import/export | no frame > 16 ms (vision) |

## Open questions
1. **(owner)** Default 3MF flavour for File > Export: **Generic** (strictly valid, opens
   everywhere; per-object colours, but Orca doesn't map them to extruders) or **Orca/Bambu**
   (multi-material extruders in Orca, but not strictly valid 3MF because of `paint_color`)?
   *Recommended:* Generic for File > Export. Orca is selected automatically by AM-06 "Send to
   OrcaSlicer", and the dialog remembers the last choice.
2. **(owner)** Non-watertight 3MF export: block by default with "Repair first" / "Export
   anyway" (*recommended*, matching the vision's "fail loudly"), or warn only?
3. **(owner)** Which Chitubox edition/version do you use, and does it open 3MF? This decides
   whether resin hand-off is STL (default) or 3MF.
4. **(owner)** Add a vision budget row "Import 1 GB binary STL < 5 s"? FND-08's `BUDGETS` and
   its row-count test would then need one more entry.
5. **(resolved, see log)** Import as a parametric timeline feature (Fusion "Insert Mesh"), or
   as plain baked bodies? → feature.
6. **(resolved, see log)** Own parsers vs crates for STL/OBJ/PLY/3MF → own, with crates as
   oracles.

## Refinement log
- **2026-10-20 (S12 refinement, lane G agent).**
  - **Read:** AGENTS.md, the vision, architecture, ADRs, the roadmap, QUESTIONS (binding
    answers: #2 Orca + Chitubox with MMU multi-material, #4 private Fusion STEPs later, #19
    licensing of corpora, #20 no owner STLs), the Ready specs S00/S01/S03/S07, and the notes
    units-and-quantities and mesh-processing.
  - **Research:**
    - 3MF Consortium specs: core 1.4.0, Materials 1.2.1, Production 1.1.2, Beam Lattice
      1.2.0.
    - Conformance test suites (BSD-2-Clause, P/N naming), lib3mf 2.5.0 (BSD-2-Clause, PyPI).
    - OrcaSlicer `bbs_3mf.cpp` / `TriangleSelector.cpp` / `Model.cpp` / `PrintConfig.cpp`
      and the PrusaSlicer `Model3mf.cpp` / import matrix.
    - Slicer release versions.
    - crates.io versions and licences.

    Everything is recorded in `docs/notes/import-export-formats.md`.
  - **Key findings driving the design:**
    1. **Orca for 3rd-party 3MFs.**
       - It maps object-level `m:colorgroup` colours to extruders.
       - It ignores triangle `pid/p1`.
       - It reads per-face filaments only from the unprefixed `paint_color` attribute.

       Hence separate Generic (strictly valid) and Orca flavours. Amalgam never writes
       another producer's `Application` name to trigger Orca's project path.
    2. **Bambu/Orca projects use Production-extension multi-part models**, so `p:path` reading
       is Phase 1, not optional.
    3. **3MF `model` meshes MUST be manifold and outward-oriented**, so there is an export
       validity gate with "Repair first".
    4. **`memmap2` needs `unsafe`**, which AGENTS.md forbids here, so STL streaming uses
       chunked reads.
    5. **`threemf` is 0BSD** (not in the `deny.toml` allow-list) and covers core only. The
       Rust `lib3mf` is 0.1.x. So we write our own 3MF on `zip` + `quick-xml`, and the
       official lib3mf is a Python validation tool.
  - **Decisions:**
    - **Own parsers** for STL/OBJ/PLY/3MF, for f64, streaming, cancel/progress, bounded
      allocation, line/byte-located errors and determinism. `stl_io`/`tobj`/`ply-rs-bw` are
      dev-dep oracles.
    - **Import is a timeline feature** (`ImportMeshFeature`) with raw baked meshes and
      editable unit/up-axis/placement, as in Fusion's Insert Mesh. This needs the S01
      baked-payload API (request).
    - **Format-neutral `Scene`** for both directions. The document adapter goes in
      `amg-features::io`, because `amg-io` sits below `amg-doc`.
    - **Units:** declared units are honoured; unitless files import as mm with a suggestion
      and are never silently rescaled.
  - **WP changes:**
    - IO-01 re-scoped to the `amg-io` foundation, with deps **FND-04, FND-10, MESH-01** (the
      FND-10 repoint as agreed; MAT-01 likewise depends on FND-10, requested to S17). The UI
      part moved to the new IO-15.
    - IO-03 split into a reader (IO-03) and a Generic writer (new IO-11).
    - **IO-03 no longer depends on MAT-01.** Materials enter through `MaterialSlot` in the
      adapter, and MAT-01 is optional for IO-14.
    - **IO-04 re-scoped to the Orca/Bambu flavour and raised P1 → P0**, because the owner's
      MMU workflow (QUESTIONS #2) needs it in Phase 1. Its dep on AM-06 was removed: AM-06
      should depend on IO-04, not the reverse.
    - Prusa flavour and per-object settings split out as IO-12 (P1).
    - IO-05 limited to OBJ. PLY is the new IO-13.
    - New: IO-14 (document adapter and import feature), IO-15 (UI), IO-16 (fuzzing), IO-18
      (beam lattice, P2), IO-19 (full materials, P2).
    - **STEP/IGES repointed from BREP-10 to BREP-13**, and IO-08 to BREP-17, following the
      parallel S02 refinement draft. That draft splits STEP/IGES (`ForeignModel`) out of
      healing.
    - IO-06 dep IO-14 added; IO-09 dep IO-14 added; IO-08 dep IO-01 added; IO-10 deps DOC-05
      and UI-04 added.
    - The reader WPs (IO-02/03/05/13) depend on MESH-28 for the corpus and the benchmark
      generator.
    - The S16 draft's requested `write_3mf` API and the S17 draft's `ThreeMfDisplay`/`Srgba8`
      were matched (see Interfaces requests).
    - IO-17 is not used.
  - **Ownership:** `amg-features::io` is a new lane-G module. The roadmap §Lanes should list it
    (report).
  - **Phase proposal (for the coordinator):**
    - Phase 1 G: IO-01, IO-02, IO-03, IO-11, IO-04, IO-05, IO-13, IO-14, IO-15, IO-16.
    - Phase 2 G: IO-06, IO-09 (unchanged).
    - Phase 3 G: IO-07, IO-08, IO-10, IO-12.
    - Phase 5 G: IO-18, IO-19.

    Every WP is scheduled no earlier than its deps: IO-14 needs DOC-05/DOC-07/MESH-16
    (Phase 1). IO-15 needs UI-01/02/10 (Phase 1). IO-10 needs RND-03/UI-04 (Phase 1). IO-18
    needs IMP-06 (Phase 3).
  - Status set to `Refined`.

## Interface changes
_None yet (spec not Ready)._
