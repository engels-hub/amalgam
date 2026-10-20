# Import / export formats: 3MF, slicer interop, mesh readers, robustness

Design note for S12 (`docs/specs/S12-import-export.md`). Written 2026-10-20 during S12
refinement. Evidence was gathered from the 3MF Consortium specifications and repositories, the
OrcaSlicer and PrusaSlicer source trees (cited by file), and crates.io metadata (queried
2026-10-20). Sections:

1. Formats in scope and what each declares
2. 3MF: package, core 1.4, extensions
3. Slicer interop (OrcaSlicer, Bambu Studio, PrusaSlicer, Cura, Chitubox)
4. Slicer paint encoding (`paint_color`, `slic3rpe:mmu_segmentation`)
5. STL: detection, quirks, streaming
6. OBJ and PLY
7. Units and up-axis
8. Library evaluation
9. Robustness: limits, degenerate input, fuzzing
10. Performance plan
11. Test corpora
12. References

---

## 1. Formats in scope and what each declares

| Format | Geometry | Units declared | Multi-object | Colour / material | Phase |
|---|---|---|---|---|---|
| STL binary | triangle soup, f32 | never | no (one solid) | non-standard 15-bit face colour (VisCAM/SolidView, Magics) | 1 |
| STL ASCII | triangle soup, decimal text | never | several `solid` blocks | none | 1 |
| 3MF | indexed mesh per object, components, build items | always (`unit`, default mm) | yes | base materials (core), colour groups and textures (Materials ext.) | 1 |
| OBJ (+ MTL) | polygons, `v/vt/vn` | never | `o` / `g` | `usemtl` → MTL `Kd`; non-standard `v x y z r g b` | 1 |
| PLY | vertices + face lists, ASCII or binary | never (a comment sometimes) | no | per-vertex / per-face `red green blue [alpha]` | 1 |
| STEP AP203/214/242 | B-rep via OCCT XCAF | always | assemblies | XCAF colours, layers, names | 2 |
| IGES | B-rep / surfaces via OCCT | always (global section) | weak | entity colours | 3 |
| DXF | 2D curves (export here; import is S03 SK-11) | `$INSUNITS` (often 0 = unitless) | layers | layer colour | 3 |
| SVG | 2D paths (export here; import is S03 SK-11) | via `width`/`height` + `viewBox` | groups | stroke | 3 |

Consequence: only 3MF, STEP and IGES carry trustworthy units. Everything else goes through the
unit policy in §7.

## 2. 3MF: package, core 1.4, extensions

### 2.1 Versions (3mf.io/spec, page updated 2026-09-17)

| Specification | Published | Latest revision | Namespace |
|---|---|---|---|
| Core | 1.3.0 | **1.4.0** (Feb 2025: removed deprecated `mirror`, clarified shapes and composition) | `http://schemas.microsoft.com/3dmanufacturing/core/2015/02` |
| Triangle sets (part of core since 1.3) | — | — | `http://schemas.microsoft.com/3dmanufacturing/trianglesets/2021/07` |
| Materials and Properties | 1.2.1 | 1.2.1 | `http://schemas.microsoft.com/3dmanufacturing/material/2015/02` |
| Production | 1.1.2 | 1.1.2 | `http://schemas.microsoft.com/3dmanufacturing/production/2015/06` |
| Beam Lattice | 1.2.0 | 1.2.0 | `http://schemas.microsoft.com/3dmanufacturing/beamlattice/2017/02` |
| Boolean Operations | 1.1.0 | 1.1.1 | — (not supported) |
| Secure Content | 1.0.2 | 1.0.2 | — (detected and rejected) |
| Volumetric | 0.8.0 | — | — (not supported) |
| Toolpath | 1.0.0 (2026-09-17) | — | — (not supported) |

3MF is also ISO/IEC 25422:2025. All Consortium repositories (`spec_*`, `lib3mf`,
`test_suites`) are BSD-2-Clause (GitHub API, 2026-10-20).

### 2.2 Package (OPC) rules that matter
- Zip entries are Deflate or Stored. **Consumers must support ZIP64 and streaming (data
  descriptor) entries.** Producers should write plain zip unless ZIP64 is needed (core §2).
- Required parts: `[Content_Types].xml`, `_rels/.rels` with a StartPart relationship of type
  `http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel` pointing at the root model part
  (conventionally `/3D/3dmodel.model`; Orca's source notes this exact path is the only one Cura
  accepts, `bbs_3mf.cpp` line 212).
- Package thumbnail: relationship type
  `http://schemas.openxmlformats.org/package/2006/relationships/metadata/thumbnail`, PNG or JPEG,
  conventionally `/Metadata/thumbnail.png`.
- Part names are case-insensitive URIs. Readers must resolve relationship targets relative to
  the source part, and percent-decode.

### 2.3 Model rules that matter
- `<model unit="…">`: `micron | millimeter | centimeter | inch | foot | meter`, default
  millimeter. It applies to all vertices and transform translations.
- `requiredextensions="p m …"` lists prefixes. A consumer **must fail** on a required extension
  it doesn't support. Non-required extensions are ignored. Attributes at `anyAttribute`
  extension points must be namespace-prefixed (core §3.4).
- Objects have `type`: `model | solidsupport | support | surface | other`. Build items must not
  reference `other`.
- **Meshes under `model` and `solidsupport` MUST be manifold, consistently oriented and
  outward-facing** (core §4.1). Self-intersections are allowed but should not be produced.
  Consumers apply the positive fill rule. `support` and `surface` objects may be open.
- Vertex arrays must have < 2³¹ entries. Resource ids are positive integers < 2³¹.
- Numbers follow `ST_Number`: `[+-]?(digits(.digits)?|.digits)([eE][+-]?digits)?`. There is no
  `inf`/`nan`, no `1.` and no hex. Writers must normalise `-0` and never emit non-finite values.
- Transforms are 3×4, written row-major as `m00 m01 m02 m10 m11 m12 m20 m21 m22 m30 m31 m32`, with
  the translation in `m3x` (row-vector convention: `p' = p · M`). Converting to
  `glam::DAffine3` (column vectors) means transposing the 3×3 part. A unit test pins this.
- Components: an object either has a mesh or `<components>` of `objectid` + `transform`
  (recursive, acyclic). Build `<item objectid transform partnumber>` entries are the
  printable instances.
- Triangle properties: `pid` + `p1 p2 p3` (per corner) or `pid` + `p1` (uniform). The object
  `pid`/`pindex` gives the default. Base materials (`<basematerials><base name
  displaycolor="#RRGGBB[AA]"/>`) are **core**. Colour groups, textures, composites and
  multiproperties are the Materials extension.
- Metadata names defined by core: `Title, Designer, Description, Copyright, LicenseTerms,
  Rating, CreationDate, ModificationDate, Application`. Other names must be prefixed.

### 2.4 Extensions we read or write

| Extension | Read | Write | Why |
|---|---|---|---|
| Production | **Phase 1**: `p:path` components into other model parts (`/3D/Objects/*.model`), `p:UUID` kept as metadata | no (single model part is the most compatible) | Bambu Studio and OrcaSlicer projects store every object in its own part. Without this, every MakerWorld/Orca 3MF fails to load. |
| Materials | **Phase 1**: colour groups (object- and triangle-level). Texture groups → UVs + a warning that the image is dropped. Composites/multiproperties → flattened to the first base colour, with a warning. | **Phase 1**: colour groups (Orca flavour) | Owner's multi-material workflow (QUESTIONS #2). |
| Materials (full) | P2 (IO-19): texture images, composites, multiproperties, PBR display properties | P2 | needs S17 appearances and S10 textures |
| Triangle sets | read and ignore (not required) | no | harmless |
| Beam Lattice | P2 (IO-18): beams → capsule mesh | P2: S08 strut lattices as beams (tiny files) | Slicers in scope don't read it. Netfabb/lib3mf tools do. |
| Slice, Volumetric, Boolean, Displacement, Toolpath | fail with `UnsupportedRequiredExtension` if required, else ignore | no | out of scope |
| Secure Content | detect encrypted parts → `IoError::Encrypted` | no | out of scope |

## 3. Slicer interop

Versions current on 2026-10-20 (GitHub releases): OrcaSlicer **2.4.2** (2026-07-07; 2.5.0-alpha
2026-10-16), Bambu Studio **02.08.02.61** (2026-08-21), PrusaSlicer **2.9.6** (2026-06-25), Cura
**5.13.0** (2026-05-28). The owner uses OrcaSlicer for FFF (Ender 3 V3 KE + MMU mod) and
Chitubox for MSLA resin (QUESTIONS #2).

### 3.1 OrcaSlicer / Bambu Studio (shared `src/libslic3r/Format/bbs_3mf.cpp`)
Findings from the OrcaSlicer `main` source (fetched 2026-10-20):
- **Producer detection.** A file counts as a "BBL 3MF" only if `<metadata name="Application">`
  starts with `BambuStudio-` or `OrcaSlicer-`, or it has an `OrcaSlicer` version metadata entry.
  **Amalgam never writes those values** (that would impersonate another producer). We write
  `Application = "Amalgam-<version>"`, so Orca takes its "3rd-party 3MF" path.
- **3rd-party path behaviour.**
  - `unit` is honoured (`bbs_get_unit_factor`).
  - Build item transforms are applied. An object with several build items is split into one
    Orca object per instance.
  - With one object, the object is renamed to the file name.
- **Colour → extruder mapping (key for multi-material).** For 3rd-party files, Orca collects
  every `m:colorgroup` and its colour (the last `m:color` in the group wins). It numbers the
  distinct colours 1, 2, 3… in ascending group-id order and sets each object's `extruder` from
  the object-level `pid`. **Triangle-level `pid/p1/p2/p3` are ignored** (code comment: "we are
  ignoring the following attributes: p1 p2 p3 pid"). `basematerials` is not mapped to
  extruders.
  - So: one single-colour `m:colorgroup` per filament, in filament order, plus an object `pid`
    gives per-object filament assignment, using only spec-conformant constructs.
- **Per-face filament** is read from the unprefixed triangle attribute **`paint_color`**
  (`MMU_SEGMENTATION_ATTR`), whatever the producer. The encoding is in §4. It is not
  namespace-prefixed, so files that carry it fail strict 3MF validation. It belongs only in the
  Orca flavour.
- **`Metadata/model_settings.config`** (XML `<config><object id><metadata key value/>
  <part id subtype>…</part></object><plate>…</plate></config>`) is parsed even for 3rd-party
  files. It carries object/part names, a per-object/per-part `extruder`, part subtypes
  (`normal_part`, `negative_part`, `modifier_part`, `support_enforcer`, `support_blocker`) and
  plate membership. IO-04 starts with a half-day probe that records exactly which keys Orca
  2.4.x honours for 3rd-party files (`_handle_start_config_xml_element`), and the Done-when
  checks the result through Orca's CLI.
- Other Orca parts (`project_settings.config`, `slice_info.config`, `plate_N.png`, `plate_N.gcode`,
  `filament_sequence.json`) are project state. We neither write nor need them, and on import we
  keep only geometry, names, extruder ids and paint.
- **CLI** (`PrintConfig.cpp` CLI options): `--info`, `--slice <plate>`, `--export-3mf <file>`,
  `--outputdir`, `--arrange`, `--load-settings`, `--load-filaments`. IO-04 uses
  `orca-slicer --info` and `--export-3mf` as an automated "opens in Orca" check.

### 3.2 PrusaSlicer
- **2.9.x (released).** It reads core geometry and `unit`. Multi-material comes from the
  triangle attribute `slic3rpe:mmu_segmentation` (namespace `http://schemas.slic3r.org/3mf/2017/06`,
  same encoding as §4) and per-volume extruders from `Metadata/Slic3r_PE_model.config`.
  Colour groups and base materials are ignored.
- **`master` (unreleased; the 3MF code moved to `src/slic3r-shared/.../Biz/Format/3mf/`).**
  Painting moves to `Metadata/Slic3r_facets_annotation.json`. The writer dual-writes the
  legacy attribute, and the reader ignores the legacy attribute in favour of the JSON. **Risk:**
  once that ships, a Prusa flavour must write the JSON too. IO-12 is P1/Phase 3, so it decides
  against whatever has shipped by then.
- 3MFs from non-Prusa producers load as "shapes only" (`doc/3mf-import-matrix.md`). That is
  fine for us.
- CLI: `prusa-slicer-console --info <file>` prints size, facet count and manifoldness.

### 3.3 Cura 5.13
Cura reads core 3MF through Savitar (LGPL; we only exchange files with it). It reads geometry,
build transforms and objects, and ignores colours. It needs the root model at
`/3D/3dmodel.model` (§2.2). Check: manual smoke test only, because Cura has no practical
slicing CLI for 3MF.

### 3.4 Chitubox (resin)
Chitubox reads STL and OBJ. Whether the owner's edition accepts 3MF is unverified (owner
question in S12). Resin hand-off defaults to binary STL, one file per plate, in mm.

### 3.5 Flavours (what IO-11/IO-04/IO-12 write)

| | Generic (IO-11) | Orca/Bambu (IO-04) | Prusa (IO-12) |
|---|---|---|---|
| Model part | single `/3D/3dmodel.model` | same | same |
| Per-object material | `basematerials` (name + `displaycolor`) + object `pid` | one single-colour `m:colorgroup` per filament slot + object `pid` | none in XML; extruder in `Slic3r_PE_model.config` |
| Per-face material | triangle `pid/p1` into the same group | `paint_color` (§4) + triangle `pid/p1` into the colour group | `slic3rpe:mmu_segmentation` (+ JSON if shipped) |
| Extra parts | `Metadata/thumbnail.png` | + `Metadata/model_settings.config` (names, extruder, plates) | + `Metadata/Slic3r_PE_model.config` |
| Strictly valid (lib3mf strict + XSD) | **yes** (gated in CI) | no (unprefixed `paint_color`); valid once those attributes are removed | yes (prefixed) |

## 4. Slicer paint encoding (`paint_color` / `slic3rpe:mmu_segmentation`)
Source: OrcaSlicer `TriangleSelector::serialize()` / `deserialize()`, and the
`CONST_FILAMENTS` table in `Model.cpp`. PrusaSlicer 2.3.1+ uses the same scheme.

Each original triangle has a bitstream describing a recursive subdivision tree:
- **Node.** 2 bits `split_sides` (0–3).
  - If `split_sides > 0`: 2 bits `special_side`, then `split_sides + 1` children, serialised in
    **reverse** child order.
  - If it is a leaf: the state. States 0–2 are written as 2 bits. State ≥ 3 is written as `11`
    followed by a nibble `n − 3`. A nibble of `1111` means a second nibble follows holding
    `n − 18`.
- **String form.** Bits are packed LSB-first into nibbles. Nibbles are written as uppercase hex
  **in reverse order** (the last nibble is the first character).
- **Whole-triangle states** (no split), from `CONST_FILAMENTS`:

  | Filament | 0 (object default) | 1 | 2 | 3 | 4 | … | 16 | 17 | 18 |
  |---|---|---|---|---|---|---|---|---|---|
  | String | (attribute omitted) | `4` | `8` | `0C` | `1C` | … | `DC` | `EC` | `0FC` |

Phase 1 policy:
- **Writing.** Only whole-triangle states (no splits). Amalgam's per-face materials are
  per-triangle, so this is exact.
- **Reading.** Whole-triangle states map exactly. Split triangles are reduced to the state of
  the largest-area leaf, and the count is reported as `IoWarning::PaintApproximated`. An exact
  import that splits the geometry is a later option (P2; it changes the triangle count).
- Out-of-range states are clamped to "object default" with a warning.
- The decoder is a pure function and gets its own fuzz target (IO-16).

## 5. STL

### 5.1 ASCII/binary detection (`amg_io::stl::detect`)
`solid` at the start of a **binary** header is common (SolidWorks and other CAD exporters), so
the leading keyword alone proves nothing. Rules, in order:
1. If `len ≥ 84` and `84 + 50·n == len`, where `n` is the u32 at offset 80, the file is
   **binary**. This test is exact and almost never matches by accident for ASCII files.
2. Otherwise, if the file, after skipping a UTF-8 BOM and ASCII whitespace, starts with
   `solid` (any case), and `facet` (any case) appears in the first 4 KiB, it is **ASCII**.
3. Otherwise, if `len ≥ 84` and `(len − 84) % 50 == 0`, it is **binary with a wrong count**:
   use `(len − 84)/50` and warn with `HeaderCountMismatch`. This covers writers that leave
   `n = 0`.
4. Otherwise, if `len ≥ 84` it is **binary, truncated or with trailing bytes**:
   - Read `min(n, (len − 84)/50)` complete triangles.
   - Warn with `Truncated` if fewer than `n`, or `TrailingBytes` otherwise.
   - A 0-triangle result is `IoError::Empty`.
5. Anything else is `Malformed` (`location: Byte(0)`).

### 5.2 Quirks handled
- **ASCII.**
  - Mixed case, tabs, CRLF, a missing `endsolid`, and names with spaces.
  - Several `solid … endsolid` blocks. Option `split_solids` gives one object per block;
    otherwise they are merged.
  - `outer loop` with ≠ 3 vertices: ≥ 4 is fan-triangulated with a warning, < 3 is dropped
    with a warning.
- **Binary.**
  - The stored facet normal is ignored. It is unreliable, and orientation comes from the
    winding order.
  - The 2-byte attribute word is read as a VisCAM/SolidView colour (bit 15 = valid, 5:5:5
    BGR) only when `StlReadOptions::face_colors` is set. Magics (`COLOR=` in the header) uses
    the inverse bit-15 sense, so the header is checked.
- **Non-finite coordinates** (NaN/inf): the triangle is dropped and counted
  (`NonFiniteDropped`). Never an error unless every triangle is dropped.
- **Degenerate triangles** (repeated vertex after weld, zero area) are **kept**, because
  `IndexedMesh` tolerates them and repair is MESH-04's job. They are counted in `ImportReport`.

### 5.3 Streaming read of huge binary STL
The target is a 1 GB binary file (~20 M triangles) in < 5 s, with peak extra heap ≤ 2.5 GB.
`unsafe` is not allowed in `amg-io` (AGENTS.md), so there is **no mmap** (`memmap2::Mmap::map`
is `unsafe`). The pipeline:
1. A reader thread fills 16 MiB chunks from a `BufReader`/`File::read` (sequential NVMe reads
   run at ≥ 2 GB/s, so this is not the limiter) and sends them over a bounded channel (depth 4).
2. Rayon workers parse each chunk into `[f32; 9]` triangles. Corner keys are the f32 bit
   patterns with `-0.0` normalised to `+0.0`.
3. **Exact weld:** a sharded hash map with 64 shards, chosen by the high bits of an FxHash-style
   key hash. Each shard maps `[u32; 3]` → vertex index and is locked per shard. Chunks are
   committed in chunk order, so indices are deterministic.
   - An alternative of equal standing is a global parallel radix sort of `(key, corner)` pairs.
     The WP picks one by benchmark and records the choice in the Refinement log.
   - Vertex ids are assigned in first-seen order, which makes the output independent of the
     thread count.
4. f32 → f64 conversion is lossless (S07 decision 1).
5. `ctx.checkpoint()` runs per chunk (< 10 ms of work). Progress is bytes read / file length.

ASCII STL is parsed in parallel by splitting chunks at `endfacet` boundaries.
`str::parse::<f64>` (Eisel–Lemire in std) is fast enough, so no float-parsing crate is needed.

## 6. OBJ and PLY

**OBJ (own parser; `tobj` as test oracle).**
- **Supported:** `v x y z [w]`, `v x y z r g b` (vertex colours as 0–1 floats; MeshLab/ZBrush
  convention), `vt`, `vn`, and `f` with `v`, `v/vt`, `v//vn`, `v/vt/vn` and **negative
  (relative) indices**. `o`/`g` become objects or face groups (option). `usemtl` becomes
  per-face material slots, and `mtllib` loads MTL `Kd` (and `d`) as the slot colour. Line
  continuation `\` is handled.
- **Polygons with > 3 vertices** are triangulated by ear clipping in the plane of the Newell
  normal, with fan fallback for degenerate polygons (and a warning). `tobj` only fan-triangulates,
  which is wrong for concave faces. That, plus f64, cancellation and line-numbered errors, is
  why we parse ourselves.
- **Large files** use two passes. Pass 1 counts `v/vt/vn` per chunk in parallel (prefix sums
  resolve negative indices). Pass 2 parses the chunks in parallel.
- **Ignored with a counted warning:** `l`, `p`, curves/surfaces (`cstype`, `curv`, `surf`) and
  `s` (smoothing groups).

**PLY (own parser; `ply-rs-bw` as test oracle).**
- **Encodings:** `ascii`, `binary_little_endian` and `binary_big_endian`.
- **Header limit 64 KiB.** Elements are read generically from their declared properties.
  - Known vertex properties: `x y z` (float/double), `nx ny nz`, `red green blue [alpha]`
    (uchar or float), and `s t`/`u v`/`texture_u texture_v`.
  - Faces: `vertex_indices`/`vertex_index` lists with uchar/ushort/uint count and
    int/uint indices. Polygons go through the same ear clipper.
  - Face `red green blue`.
  - Unknown elements and properties are skipped by size.
- **A file with no faces** (point cloud) is `IoError::PointCloud`. There is no point-cloud body
  kind.

## 7. Units and up-axis
- **Internal unit is mm** (vision, S00 FND-10). Imported coordinates are scaled exactly by
  `LengthUnit` → mm factors: µm 1e-3, cm 10, m 1000, in 25.4, ft 304.8. All are exact in
  f64 except µm (1e-3 is not exactly representable, so we divide by 1000 instead of
  multiplying).
- **Declared units** (3MF, STEP, IGES, DXF `$INSUNITS` ≠ 0) are used unless the user overrides
  them.
- **Unitless files** (STL, OBJ, PLY, DXF with `$INSUNITS = 0`) are imported as mm, Fusion-style.
  The import dialog then shows the extents under the chosen unit, plus a **suggestion** from
  `guess_unit`:
  - largest extent `E` < 1.0 (in file numbers) → suggest **m** (likely a metre-scale export
    from Blender or a scanner), confidence Medium;
  - `E` > 5000 → suggest **µm**, confidence Low;
  - otherwise → mm, confidence High.

  Inches cannot be told apart from mm by geometry. The dialog therefore shows the part
  dimensions in both mm and inch interpretations, and whether each fits the active printer's
  build volume (AM-01, when present). **Nothing is ever scaled silently.**
- **The import feature stores the raw file numbers and the chosen unit as a parameter.**
  Changing the unit after import is an exact rescale on recompute (IO-14), with no re-read.
- **Up-axis.** 3MF and STL are Z-up (slicer convention). OBJ and PLY are often Y-up (the Blender
  default export is `-Z forward, Y up`). The option `UpAxis::{Z, Y}` defaults to Z for every
  format. The UI remembers the last choice per format. Y-up applies a +90° rotation about X.
- **Export** writes mm by default. 3MF writes `unit="millimeter"`. STL/OBJ/PLY scale the
  coordinates if the user picks another unit.

## 8. Library evaluation (crates.io, 2026-10-20)

| Crate | Version (date) | Licence | Verdict |
|---|---|---|---|
| `zip` (zip-rs/zip2) | 9.0.5 (2026-10-17) | MIT | **Adopt** as a workspace dep shared with S01 DOC-07. Use `default-features = false, features = ["deflate-flate2-zlib-rs"]`; the defaults pull aes/bzip2/lzma/zstd/zopfli/xz, which we don't want. Supports ZIP64 and data descriptors. 9.0.x was released days ago, so pin the exact patch in FND-02/DOC-07 and accept a later 9.0.x. |
| `quick-xml` | 0.42.0 (2026-08-22) | MIT | **Adopt** (`default-features = false`). Streaming pull parser for every 3MF part and for MTL/config XML. Hot vertex/triangle loops use a hand-written attribute scanner on the inflated buffer, validated against quick-xml in a test (§10). |
| `png` | 0.18.1 | MIT OR Apache-2.0 | **Adopt** for thumbnail encode/decode. |
| `blake3` | 1.8.7 | CC0-1.0 OR Apache-2.0 OR Apache-2.0 WITH LLVM-exception | **Adopt** (already used by S01) for the import source hash. |
| `memchr` | 2.8.3 | Unlicense OR MIT | **Adopt** for chunk boundary search (`endfacet`, `\n`, `<triangle`). Already transitive via zip. |
| `rayon`, `crossbeam-channel`, `thiserror` | workspace | MIT/Apache | workspace deps |
| `dxf` (ixmilia) | 0.6.2 (2026-10-03) | MIT | **Adopt** for IO-08 DXF writing (R12 and R2000+). Also recommended to S03 SK-11 for reading, so there is one DXF stack. |
| `usvg` | 0.48.1 | Apache-2.0 OR MIT | Recommended to S03 SK-11 (SVG import). Not needed by S12: SVG export is a small hand-written writer. |
| `stl_io` | 0.11.0 | MIT | **Test oracle only** (dev-dep). f32 vertices, no progress/cancel, no streaming weld. |
| `tobj` | 4.0.5 | MIT | **Test oracle only.** Fan triangulation only, no cancel (§6). |
| `ply-rs-bw` | 4.0.1 | MIT | **Test oracle only** (the maintained fork; `ply-rs` 0.1.3 is unmaintained since 2020). |
| `threemf` | 0.8.0 | **0BSD** (not in `deny.toml` allow-list) | **Reject.** 249 lines, core mesh only (no materials, production, components), pins zip 8 / quick-xml 0.39 (duplicate versions). |
| `lib3mf` (Rust, telecos) | 0.1.6 | MIT | **Reject** for now: 0.1.x, ~3 k downloads, young. Re-evaluate in IO-19. |
| lib3mf (official C++; PyPI `lib3mf` 2.5.0) | 2.5.0 | BSD-2-Clause | **Validation tool** (Python, dev only): strict-mode read of every exported Generic 3MF in CI (`scripts/io/validate_3mf.py`). Not linked into Amalgam. |
| `xmlschema` (PyPI) | 4.3.2 | MIT | **Validation tool:** XSD check of exported model parts against the Consortium `Consolidated_Schema`. |
| `dhat` | 0.3.3 | MIT OR Apache-2.0 | **Dev-dep** for peak-heap assertions in IO-02/IO-03 benches (safe API, no `unsafe` in our code). |
| `libfuzzer-sys` | 0.4.13 | (MIT OR Apache-2.0) AND NCSA | **Fuzz crate only** (outside the workspace). NCSA is permissive (LLVM) but not in the allow-list, which is fine because `fuzz/` isn't a workspace member. |
| `cargo-fuzz` | 0.13.2 | MIT OR Apache-2.0 | tool (nightly) |
| `arbitrary` | 1.5.0 | MIT OR Apache-2.0 | fuzz crate + structured round-trip fuzzing |
| `bolero` | 0.13.7 | MIT | Considered (runs fuzz targets as stable tests). Rejected to keep one fuzz stack. Regression replay (IO-16) covers stable. |
| `memmap2` | 0.9.11 | MIT OR Apache-2.0 | **Reject:** `Mmap::map` is `unsafe`, which AGENTS.md disallows outside `-sys`/FFI crates. Chunked reads meet the budget. |
| `image` | 0.25.10 | MIT OR Apache-2.0 | IO-10 only (PNG/JPEG canvas import), `default-features = false, features = ["png", "jpeg"]`. |

## 9. Robustness

### 9.1 Limits (`ReadLimits`, all configurable; exceeding one gives `IoError::LimitExceeded`)

| Limit | Default | Why |
|---|---|---|
| `max_triangles` | 250 M | u32 indices (S07) leave headroom. Above 10 M we warn about performance. |
| `max_vertices` | 250 M | as above |
| `max_entry_uncompressed` | 8 GiB | 3MF XML of a 250 M-tri model |
| `max_total_uncompressed` | 16 GiB | zip bombs |
| `max_compression_ratio` | 500 : 1 per entry (applied once the entry exceeds 64 MiB) | 3MF XML compresses ~5–15 : 1, and a bomb is ≥ 1000 : 1 |
| `max_zip_entries` | 100 000 | |
| `max_xml_depth` | 64 | |
| `max_attr_value_len` | 1 MiB | `paint_color` strings can be long |
| `max_component_depth` | 32, and cycles are an error | 3MF components |
| `max_header_bytes` (PLY) | 64 KiB | |

**Allocation rule:** no reader allocates from a count declared in the file beyond what the
remaining file bytes can actually hold. For example, STL `n ≤ (len − 84)/50`, and a PLY element
count × minimum record size ≤ remaining bytes. Vectors grow geometrically otherwise.

### 9.2 Degenerate and hostile input (expected results, tested in each reader WP)
| Input | Result |
|---|---|
| Empty or 0-byte file | `IoError::Empty` |
| No triangles after dropping invalid ones | `IoError::Empty` (report says why) |
| Triangle index out of range (OBJ/PLY/3MF) | 3MF: `Malformed` with the XML line. OBJ/PLY: drop the face, `IoWarning::InvalidIndexDropped`. |
| Repeated indices / zero-area | kept, counted |
| NaN/inf | dropped, counted |
| 3MF missing StartPart / missing referenced part | `Malformed` |
| 3MF component cycle | `Malformed` |
| Unsupported required extension | `UnsupportedRequiredExtension { namespace }` |
| Encrypted 3MF | `Encrypted` |
| Zip bomb / deep XML | `LimitExceeded` |
| A wrong extension (e.g. `.stl` that is a zip) | sniffing decides. The extension is only a hint, and the mismatch is reported as a warning. |

### 9.3 Fuzzing (IO-16)
- **`fuzz/` cargo-fuzz project** (nightly), excluded from the workspace. Targets:
  - `stl` (detection + both parsers),
  - `obj`, `mtl`, `ply`,
  - `threemf_package` (whole zip),
  - `threemf_model` (model XML with a stub package),
  - `paint_decode`,
  - `roundtrip_scene`: `arbitrary` scene → export → import → compare.
- **Seeds:** synthetic fixtures + 3MF Consortium P/N files.
- **CI:** a scheduled Linux nightly job (libFuzzer runs most reliably there; Windows is
  best-effort), 5 min per target per night, with `-rss_limit_mb=2048 -timeout=2`.
- **Regression replay:** every crash artifact is minimised into `testdata/io/fuzz/<target>/`
  and replayed by a normal `cargo nextest` test on Windows stable. Fixed bugs therefore stay
  fixed without nightly.

## 10. Performance plan
Reference machine from the vision doc, release build. These are spec targets; a proposed
vision row is in the S12 report.

| Scenario | Target | Notes |
|---|---|---|
| Binary STL 1 GB (20 M tris) → `IndexedMesh` (exact weld) | **< 5 s**, peak extra heap ≤ 2.5 GB | §5.3 |
| ASCII STL 1 GB (~4.5 M tris) | < 6 s | parallel chunks |
| Binary STL write, 20 M tris | < 2 s | |
| 3MF read, 10 M tris (~0.6 GB XML) | < 6 s | inflate (zlib-rs ~1 GB/s) + parallel vertex/triangle scanning of the inflated buffer |
| 3MF write, 10 M tris, Deflate level 1 | < 8 s | single entry is single-threaded |
| OBJ 1 GB | < 8 s | two-pass parallel |
| PLY binary 1 GB | < 3 s | |
| Typical part (100 k tris), any mesh format: file → body in document | < 300 ms | excludes first render |
| Cancel latency during any import/export | < 100 ms | checkpoint per chunk |

**3MF hot loop.**
- Inflate the model part into one buffer (bounded by limits).
- quick-xml streams the structure. On `<vertices>` and `<triangles>`, the reader switches to a
  scanner that splits the byte range at `<vertex`/`<triangle` boundaries (`memchr`) into
  ~4 MiB chunks, which rayon parses in parallel.
- A differential test checks the scanner against pure quick-xml parsing on the Consortium
  suite and on fuzz seeds.

## 11. Test corpora (`testdata/io/`)
- **Manifest:** `testdata/io/corpus.toml`, in the same style as MESH-28. Each entry has `id`,
  `file`, `format`, SPDX `license`, `source_url`, `sha256`, and `expect = "ok" | "<IoError
  variant>"`. Optional fields: `triangles`, `objects`, `unit`, `warnings`.
- **Synthetic** (generated by `amg-io` test utilities, not stored): every §5–§6 quirk and every
  §9.2 row, plus 1 GB benchmark files built from the MESH-28 benchmark generator (displaced
  icosphere).
- **3MF Consortium conformance suites** (BSD-2-Clause, about 1.6 GB repository), suites 2/3/5/6
  (core, production, materials). They are fetched by `testdata/io/fetch.ps1` at a pinned
  commit with sha256s. A ≤ 50 MB subset of the core and materials positive/negative cases is
  committed to LFS if it fits; otherwise everything is fetch-only.
  - **Expected results:** every `P_*` case in our supported scope loads, and every `N_*` case
    fails with a specific error. Cases outside our scope are listed in an allow-list with the
    reason.
- **Slicer-produced fixtures:** CC0 inputs exported from OrcaSlicer 2.4.x, Bambu Studio 2.8.x and
  PrusaSlicer 2.9.x via their CLIs (`--export-3mf`) by `scripts/io/make_slicer_fixtures.ps1`.
  - Covered: Production-extension multi-part files, `paint_color` and `mmu_segmentation`
    painting, and multiple plates.
  - Our own outputs from CC0 inputs, so they can be committed (CC0, a few MB).
- **MESH-28 corpus** (S07): every STL/OBJ/PLY/3MF entry must parse, with the manifest's face
  count.
- **STEP (IO-06):** NIST MBE PMI CAD models (US-government work; confirm the terms in IO-06),
  plus the owner's Fusion exports under `$AMG_TESTDATA_DIR/fusion/` (QUESTIONS #4; private, not
  committed).
- No owner-supplied failing STLs (QUESTIONS #20).

## 12. References
- 3MF Consortium, *3MF Core Specification* v1.4.0 (2025-02-06), github.com/3MFConsortium/spec_core.
- 3MF Consortium, *Materials and Properties Extension* v1.2.1; *Production Extension* v1.1.2;
  *Beam Lattice Extension* v1.2.0 (3mf.io/spec, page updated 2026-09-17).
- 3MF Consortium, *Conformance Test Suites* and *3MF Test Specification* v2.4.1,
  github.com/3MFConsortium/test_suites (BSD-2-Clause).
- 3MF Consortium, lib3mf v2.5.0 (BSD-2-Clause), github.com/3MFConsortium/lib3mf; PyPI `lib3mf`.
- ISO/IEC 25422:2025, *3D Manufacturing Format (3MF) specification suite*.
- OrcaSlicer source, `src/libslic3r/Format/bbs_3mf.cpp`, `src/libslic3r/TriangleSelector.cpp`,
  `src/libslic3r/Model.cpp` (`CONST_FILAMENTS`), `src/libslic3r/PrintConfig.cpp` (CLI),
  github.com/SoftFever/OrcaSlicer `main`, fetched 2026-10-20 (AGPL-3.0; read for interop
  facts only, no code copied).
- PrusaSlicer source, `src/slic3r-shared/src/Slic3r/Biz/Format/3mf/Model3mf.cpp`,
  `doc/3mf-import-matrix.md`, github.com/prusa3d/PrusaSlicer `master`, fetched 2026-10-20
  (AGPL-3.0; interop facts only).
- ECMA-376 Part 2, *Open Packaging Conventions*.
- PKWARE, *APPNOTE.TXT* (ZIP64, data descriptors).
- Burkardt / Fabbers, *STL format* (binary layout, VisCAM/Magics colour conventions).
- Paul Bourke, *PLY — Polygon File Format*; Wavefront *OBJ* and *MTL* specifications.
- Autodesk, *DXF Reference* (`$INSUNITS` codes).
- Crates: zip, quick-xml, png, blake3, memchr, dxf, usvg, stl_io, tobj, ply-rs-bw, threemf,
  lib3mf, dhat, libfuzzer-sys, cargo-fuzz, arbitrary, bolero, memmap2, image (crates.io API,
  2026-10-20).
