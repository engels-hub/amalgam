# S16 — Additive manufacturing prep

| | |
|---|---|
| **Status** | Refined |
| **Priority** | P0 (machine profiles, plate placement, overhang analysis, 3MF hand-off) / P1 (orientation, arrange, printability, resin checks, resin support preview) / P3 (SLS packing, metal PBF) |
| **Crates** | `amg-am`, `amg-features::am` (print-setup extension record), `amg-ui::am` (Manufacture workspace) |
| **Lane** | G |
| **Depends on** | S00 (FND-03/04/06/08/09); S01 (DOC-05 `SetExt`, DOC-07 `ext/` persistence, DOC-09 `ExtChanged`); S07 (MESH-01..03, MESH-11, MESH-16); S10 (RND-03, RND-07, RND-08); S11 (UI-02..04, UI-10); S12 (IO-03, IO-04); S15 (CMP-01); S17 (MAT-01) |
| **Parity** | `03-fusion-parity.md` §Additive manufacturing |

## Summary
This spec covers everything between "design done" and "slicer":
- machine profiles and build volumes, imported from the owner's OrcaSlicer and Chitubox
  installs where present;
- print setups stored in the document, with plates and placed items;
- placement tools (drop, face down, rotate, centre, fit check);
- overhang and support analysis that updates live while rotating;
- automatic orientation;
- plate arrangement;
- printability checks (FFF and resin);
- support preview;
- a one-click hand-off that writes a 3MF and opens it in OrcaSlicer (FFF, multi-material) or
  Chitubox (MSLA).

Conventions, the math and the prior-art comparison are in the design note
[`docs/notes/am-build-prep.md`](../notes/am-build-prep.md).

## Scope
**In:**
- `amg-am` (pure algorithms over `amg-mesh` types, no `amg-doc`):
  - machine model and library: built-in TOML plus a user TOML directory;
  - profile import from Orca, Chitubox and Prusa;
  - placement maths and fit checks;
  - build-volume display geometry;
  - overhang classification and metrics;
  - support-volume estimate;
  - orientation search (CPU, then GPU);
  - arrange;
  - printability checks;
  - support preview;
  - slicer detection, plate export (through `amg-io`) and launch.
- `amg-features::am`:
  - `PrintSetup` extension record (`am.print_setup`) with registration and migration;
  - item → body resolution against `Evaluated`;
  - building the hand-off plate from a setup.
- `amg-ui::am`: the Manufacture workspace, including:
  - setup browser, machine picker, plate scene and placement tools;
  - overhang overlay and metrics panel;
  - auto-orient and arrange dialogs;
  - "Send to slicer".

**Out:**
- 3MF / STL writers and readers, and the Orca/Prusa 3MF metadata → S12 (`amg-io`). AM-06
  calls them.
- Shader implementation of the overhang overlay, helper geometry and line rendering → S10.
  This spec defines the classification they must match.
- Wall-thickness ray casting → S07 MESH-11. AM-07 consumes it.
- Material property values (filament/resin) → S17.
- AM overhang constraint inside topology optimisation → S14 GEN-06. It uses the same α
  convention.
- Slicing and toolpaths → the slicer (vision non-goal).
- Metal PBF inherent-strain solver internals → S13/S15. AM-09 only sets up and drives it.

## Design

### Decisions
1. **Frames and angles** are fixed in the note §1:
   - plate frame: origin at the front-left corner of the printable area, +Z = build direction;
   - item transform: rigid, `T = translate(t)·R`;
   - overhang test: `c = −n·d > cos α` (strict). This is the OrcaSlicer/PrusaSlicer
     "slope below threshold" convention; Cura's angle converts as `α = 90° − α_cura`.
   - Default α is 45° for FFF and resin, editable per setup. Orca's own default is 30°; see
     Open questions.
2. **Placement is stored relative, transforms are derived.** `Placement { rotation, xy, z }`
   stores the rotation and the plate position of the footprint centre. The translation is
   recomputed from the current geometry (note §4). An upstream edit therefore keeps the item
   on the plate and in place, with no re-placement command.
3. **Print setups are S01 extension records**, not timeline features (ADR-014, S01 §11).
   - Kind `am.print_setup`, version 1. The data is JSON of `amg_features::am::PrintSetup`.
   - Every edit is one `Command::SetExt` with the whole record. Records are small (≈ 1 KB per
     plate), so undo, save/load, migration and `ExtChanged` events come from S01 for free.
   - The setup embeds a **snapshot** of the resolved `Machine`, so a document opens the same on
     a machine without the profile. "Update from library" is an explicit action.
4. **Analysis results are not stored.** They are pure functions of
   (mesh `content_hash`, placement, settings). `amg-ui::am` caches them by that key. Nothing
   in the document depends on them.
5. **Live overlay = shader, numbers = CPU job.**
   - Colours while rotating come from the S10 RND-08 overhang shader mode, from uniforms only
     (`d_local`, `cos α`, contact plane). They are recomputed every frame, with no upload.
   - The CPU `analyze_overhang` job produces the exact per-face classes and the metrics:
     - coarse while dragging (cell ×2, ≤ 128² rays), with cancel-and-supersede;
     - fine on release.
   - A shared golden file of classification cases keeps the two in agreement (note §1.3).
6. **Support volume** uses the column method (note §3.1). It is exact per grid column, counts
   only gaps under `Overhang` faces, and splits on-plate and on-part support. Open meshes fall
   back to the per-face centroid ray estimate, flagged `approximate`.
7. **Orientation search is 2-DOF** (build direction on S²):
   - candidates: stable convex-hull poses, normal clusters and a Fibonacci sphere;
   - deterministic pattern-search refinement;
   - score: a normalised, explainable weighted sum with presets (note §5).
   - The CPU path (AM-04) is the reference; the GPU A-buffer path (AM-13) must agree within 1 %.
8. **Hand-off by executable path, not by file association.** On the owner's machine `.3mf`
   opens Creality Print 7.1. Slicers are detected from:
   - the Windows Uninstall registry keys, handling the owner's malformed Orca `DisplayIcon`
     (note §6.1);
   - well-known folders and `PATH`;
   - an explicit user path, which always wins.

   The file is written to a hand-off directory and the slicer is spawned detached.
9. **Owner's equipment drives the defaults** (QUESTIONS #2):
   - built-in profiles for the Creality Ender-3 V3 KE (Orca system values 220 × 220 × 245, plus
     a "KCM 4-slot" variant at 240 high with 4 filament slots, from the owner's Orca user
     preset);
   - the ELEGOO Mars 3 Pro (143.43 × 89.60 × 175 mm, 4098 × 2560 px), found in the owner's
     Chitubox config and pending confirmation;
   - generic FFF and MSLA templates.
10. **Multi-material.** Each plate item has `filament: Option<u8>` (1-based slot), and the
    hand-off writes it as the per-object extruder.
    - Per-face painting (one body, several filaments) is P1: it maps per-face-group slots to
      3MF per-triangle extruder data (S12 request).
    - Several bodies with different slots on one plate is the Phase 1 path, and covers the
      owner's MMU use.
11. **Body kinds.**
    - Phase 1 items must resolve to `BodyGeom::Mesh`.
    - Solid bodies resolve through the S02 tessellation at export quality (BREP-08) once
      available. Until then the item shows "convert to mesh first".
    - Implicit bodies need their meshed result (S08).
    - Unresolvable items stay in the setup with a warning. They are never dropped silently.
12. **Licences.** Orca, PrusaSlicer and Bambu sources are AGPL-3.0. They were read for
    behaviour only; no code is ported. New runtime dependencies:
    - `winreg` (MIT, Windows-only, slicer detection);
    - `toml` (MIT/Apache-2.0, machine library; already a dev-dependency in S07).

    `serde_json`, `glam`, `rayon` and `thiserror` are already listed.

### Machine library (AM-01, AM-10)
- `Machine`:
  - key, name, vendor/model, technology;
  - `BuildVolume`: bed shape (rect / polygon / circle), height, exclude polygons,
    `slicer_origin`;
  - process params: FFF nozzles, filament slots, layer and first-layer heights; resin pixel
    grid, layer height, default elevation; powder layer height;
  - `min_feature`, `default_critical_angle`, preferred slicer, source.
- **Built-in** profiles are embedded TOML (`include_str!`). **User** profiles are
  `<config>/machines/*.toml`; `amg-ui` passes the directory, and the library never hard-codes
  app paths.
- **Import (AM-10):**
  - Orca JSON with `inherits` resolution (note §6.2);
  - Chitubox `.cfgx` JSON;
  - PrusaSlicer INI.

  Imported profiles are listed with their source and can be "saved as user profile".

### Print setup (AM-01)
```
PrintSetup { name, machine: Machine (snapshot), settings: AnalysisSettings,
             plates: [Plate { id, name, items: [PlateItem { id, body: BodyId, placement, filament }] }],
             active_plate, next_item_id }
```
A design can hold several setups (`ExtKey { kind: "am.print_setup", id }`). `PlateId` and
`ItemId` are allocated from counters inside the record, which is deterministic and never reuses
an id. Cosmetic face groups for orientation are added in AM-04 as record **v2**, with a
`migrate` v1→v2 that exercises the S01 migration path.

### Placement and build volume (AM-03)
Algorithms are in note §4:
- effective transform;
- face-down rotation (shortest arc, keeps yaw);
- rotate about Z in place;
- centre on plate;
- fit check: height, convex-hull footprint vs bed polygon and exclude areas, max outside
  distance;
- SAT overlap between items;
- display geometry: plate mesh, 10 mm grid polylines, volume edges, exclude meshes.

### Overhang analysis (AM-02)
- Face classes are `Degenerate`, `Contact`, `Overhang` and `Supported`.
- Per-face results: class and down angle ψ (rad).
- Per-vertex results:
  - area-weighted severity, for the per-vertex overlay path;
  - island vertices (local minima above the contact slab).
- Metrics:
  - overhang area and projected area;
  - contact area and footprint hull area;
  - height and island count;
  - stability (centroid over the contact hull, with a margin);
  - support estimate: volume, on-plate, on-part, footprint, the cell size used, ray count.

Definitions and golden values are in note §§1–3. Rays run in body-local space on the body's
`MeshSpatial` (MESH-03), so rotating an item never rebuilds the BVH.

### Orientation (AM-04, AM-13)
- Candidates:
  - ≤ 64 stable hull poses (on-plate technologies);
  - 10 normal clusters;
  - a Fibonacci sphere (1 024 default, 4 096 "thorough").
- Refinement: top-8 pattern search, 4° → 0.25°.
- Score: weighted support volume, projected overhang, height, contact, cosmetic-face support and
  stair-stepping, plus guards.
- Presets: Min supports, Min height, Surface quality, Resin.
- Output: the 5 best distinct results with metrics, which the user picks from.
- AM-13 adds the GPU A-buffer support volume (note §5.4).

### Arrange (AM-05, AM-14)
- **2D (AM-05, FFF/MSLA):** footprints are convex hulls, optionally the exact projected outline
  via `section` unions, inflated by the spacing.
  - Placement: bottom-left-fill over no-fit polygons of convex hulls (Burke et al. 2007),
    trying yaw in 15° steps. Convex NFPs are Minkowski sums, which needs no polygon library.
  - Objects that don't fit go to a new plate.
  - Deterministic: input order is sorted by area, descending, with ties broken by `ItemId`.
- **3D packing (AM-14, SLS, P3):** voxel-based, first-fit decreasing in a height-field.
  Deferred, because the owner has no SLS printer.

### Printability (AM-07, AM-15)
- **FFF / general (AM-07):**
  - thin walls: MESH-11 field below `k × nozzle` (k = 2 default) or `min_feature`;
  - small features: connected components of the section at each layer with area below
    `min_feature²`;
  - layer islands: a new region at layer i not supported by layer i−1, from a sliced preview
    built on `amg_mesh::section` every layer height;
  - max bridge length: horizontal `Overhang` regions whose two ends are supported, measured on
    the section.
- **Resin (AM-15):**
  - trapped volumes ("cups"): enclosed air pockets open only downwards in the build direction,
    found with a flood fill on a voxel grid from the top;
  - drain-hole suggestion at the lowest point of each enclosed cavity of a hollowed part;
  - per-layer cross-section area plot (peel-force proxy) and its maximum;
  - minimum supported island size at the pixel grid.

### Supports preview (AM-08)
For resin and FFF: tree supports from the AM-02 support points (overhang faces sampled at
`support_spacing` + islands), in the style of Vaněk et al. 2014 and Schmidt–Umetani 2014.
- Branches merge downward within a cone of α.
- Contact tips and a base raft are added for resin.
- The output is a preview `IndexedMesh`, plus support volume and contact count, to compare
  orientations.
- It is not exported by default: the slicer makes the final supports (the Chitubox supports
  for resin). An "export with supports" option exists for resin.

### Hand-off (AM-06)
Note §6.3:
- resolve the plate;
- build the `amg-io` 3MF model in slicer coordinates, with per-object filament slot, name and
  colour;
- write to `%LOCALAPPDATA%\Amalgam\handoff` (temp file + rename, keep the 20 newest);
- spawn the configured slicer detached.

Orca and Prusa get 3MF. Chitubox gets 3MF or STL, decided by the AM-06 check. "Export 3MF…"
(IO-01 dialog) uses the same model builder.

### UI (`amg-ui::am`, AM-11, AM-12)
The Manufacture workspace (Fusion's "Manufacture → Additive"):
- **Setup browser:** setups → plates → items, with the machine shown.
- **Toolbar:**
  - New Setup (machine picker), Add Bodies, Remove;
  - Place on Plate (face pick), Drop, Rotate / Move dialog (numeric x, y, rotZ, tilt), Centre;
  - Overhang Analysis (toggle);
  - Auto-Orient (P1), Arrange (P1);
  - Send to Slicer, Export 3MF.
- **Plate scene:** the plate frame with the build volume, grid and exclude areas. Items are
  drawn with their effective transform. Out-of-volume items are tinted red, with a message
  such as "3.2 mm too tall".
- **Metrics panel:** the AM-02 metrics for the selected item or the whole plate, refreshed
  during drag (coarse) and on release (fine).
- Dialogs are hand-built on the framework API, like MESH-23 (UI-05 arrives in Phase 2). All
  geometry runs as jobs, and the UI only polls.

### Process simulation (AM-09, P3)
Metal PBF distortion via the inherent strain method on the S15 voxel/FE stack:
- layer-wise activation;
- calibrated strain tensor per material;
- distortion field and recoater-collision warning.

This needs FEM-04 and CMP-05. The owner has no metal printer, so it stays P3.

## Interfaces
Types from `amg-core` (S00): `Point3`/`Vec3` (= `DVec3`), `Quat` (= `DQuat`), `Transform`
(= `DAffine3`), `Aabb3`, `Ray3`, `Tolerance`, `Angle`, `Diagnostic`, `Cancelled`. From
`amg-jobs`: `JobCtx`, `Gpu`. From `amg-mesh` (S07): `IndexedMesh`, `MeshSpatial`, `FaceId`,
`VertexId`, `Polyline3`, `Rgba8`, `ComputeBackend`. All angles are in radians, lengths in mm.

```rust
// ───────────────────────── amg_am::machine (AM-01) ─────────────────────────
use std::path::{Path, PathBuf};
use glam::{DVec2, DVec3, DQuat};
use amg_core::{Aabb3, Transform};

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum Technology { Fff, Msla, Sla, Sls, Mpbf }
impl Technology { pub fn is_on_plate(self) -> bool; /* Fff */ pub fn is_resin(self) -> bool; }

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum BedShape {
    Rect { size: DVec2 },                    // origin (0,0) .. size
    Polygon { points: Vec<DVec2> },          // CCW, plate frame; may be non-convex
    Circle { diameter: f64 },                // centred at (d/2, d/2) in the plate frame
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct BuildVolume {
    pub bed: BedShape,
    pub height: f64,
    pub exclude: Vec<Vec<DVec2>>,            // keep-out polygons on the bed (Orca bed_exclude_area)
    pub slicer_origin: DVec2,                // plate-frame position of the slicer's (0,0)
}
impl BuildVolume {
    pub fn bed_polygon(&self) -> Vec<DVec2>;           // circle → 128-gon
    pub fn bed_rect(&self) -> (DVec2, DVec2);          // min, max
    pub fn centre(&self) -> DVec2;
    pub fn aabb(&self) -> Aabb3;
    pub fn contains_xy(&self, p: DVec2, tol: f64) -> bool;  // inside bed and outside all excludes
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FffParams {
    pub nozzle_diameters: Vec<f64>,          // one per physical extruder
    pub filament_slots: u8,                  // ≥ nozzle count (MMU / KCM: 4 slots, 1 nozzle)
    pub layer_height: f64,                   // default process value, mm
    pub first_layer_height: f64,
    pub layer_height_range: (f64, f64),
}
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ResinParams { pub resolution: [u32; 2], pub pixel_size: DVec2, pub layer_height: f64, pub default_elevation: f64 }
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum ProcessParams { Fff(FffParams), Resin(ResinParams), Powder { layer_height: f64 } }

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum SlicerKind { OrcaSlicer, PrusaSlicer, BambuStudio, Chitubox, Custom }

/// Stable library key: "builtin:<vendor>/<model>", "user:<file stem>", "orca:<preset name>",
/// "chitubox:<machinetype>", "prusa:<preset name>".
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize)]
pub struct MachineKey(pub String);

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum MachineSource {
    Builtin,
    User { path: PathBuf },
    Orca { preset: String, file: PathBuf, chain: Vec<String> /* inherits, child → root */ },
    Chitubox { file: PathBuf },
    Prusa { preset: String, file: PathBuf },
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Machine {
    pub key: MachineKey,
    pub name: String,
    pub vendor: Option<String>,
    pub model: Option<String>,
    pub technology: Technology,
    pub volume: BuildVolume,
    pub process: ProcessParams,
    pub min_feature: f64,                    // mm; FFF default = nozzle, resin = 2 px
    pub default_critical_angle: f64,         // rad; 45° default
    pub slicer: Option<SlicerKind>,          // preferred hand-off target
    pub source: MachineSource,
}
impl Machine {
    /// Half the first-layer height (FFF) / layer height (resin), else 0.1 mm.
    pub fn default_contact_tolerance(&self) -> f64;
    pub fn validate(&self) -> Result<(), AmError>;     // positive sizes, ≥ 3 bed points, slots ≥ nozzles …
}

#[derive(Clone, Debug)]
pub struct LoadWarning { pub file: PathBuf, pub message: String }

#[derive(Clone, Debug, Default)]
pub struct MachineLibrary { /* BTreeMap<MachineKey, Machine> — deterministic order */ }
impl MachineLibrary {
    pub fn builtin() -> Self;                                         // embedded TOML
    pub fn load_user_dir(&mut self, dir: &Path) -> Vec<LoadWarning>;  // *.toml; bad files → warnings
    pub fn insert(&mut self, m: Machine) -> Result<Option<Machine>, AmError>; // validates; returns replaced
    pub fn get(&self, key: &MachineKey) -> Option<&Machine>;
    pub fn iter(&self) -> impl Iterator<Item = &Machine> + '_;
    pub fn save_user(&self, key: &MachineKey, dir: &Path) -> Result<PathBuf, AmError>;  // → "user:" copy
}
pub fn machine_from_toml(s: &str) -> Result<Machine, AmError>;
pub fn machine_to_toml(m: &Machine) -> String;

// ───────────────────────── amg_am::import (AM-10) ─────────────────────────
#[derive(Clone, Debug, Default)]
pub struct SlicerDataDirs { pub orca: Vec<PathBuf>, pub prusa: Vec<PathBuf>, pub chitubox: Vec<PathBuf> }
impl SlicerDataDirs {
    /// %APPDATA%\OrcaSlicer, %APPDATA%\PrusaSlicer, %LOCALAPPDATA%\chitubox2_0\Conf\MachineConfig
    /// (+ Linux XDG equivalents); only existing dirs.
    pub fn discover() -> Self;
}
pub struct ImportResult { pub machines: Vec<Machine>, pub warnings: Vec<LoadWarning> }
pub fn import_orca(data_dirs: &[PathBuf], install_dirs: &[PathBuf]) -> ImportResult;
pub fn import_chitubox(dirs: &[PathBuf]) -> ImportResult;
pub fn import_prusa(dirs: &[PathBuf]) -> ImportResult;
/// Orca process preset → critical angle, if set and non-zero (support_threshold_angle).
pub fn orca_process_critical_angle(json: &serde_json::Value) -> Option<f64>;

// ───────────────────────── amg_am::place (AM-03) ─────────────────────────
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum ZMode { OnPlate { elevation: f64 }, Fixed { z: f64 } }

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Placement {
    pub rotation: DQuat,                     // unit; body-local → plate orientation
    pub xy: DVec2,                           // plate position of the rotated footprint-AABB centre
    pub z: ZMode,
}
impl Placement {
    pub fn on_plate(rotation: DQuat, xy: DVec2) -> Self;
    pub fn normalized(self) -> Self;         // re-normalises rotation (call after composing)
}

/// Body-local → plate. One parallel pass over vertices; deterministic.
pub fn effective_transform(m: &IndexedMesh, p: &Placement) -> Transform;
/// Rotation that puts body-local normal `n` facing −Z, composed with the current rotation
/// by shortest arc (keeps unrelated yaw). Anti-parallel case: 180° about plate X.
pub fn face_down_rotation(current: DQuat, n_local: DVec3) -> DQuat;
pub fn rotate_about_build_axis(p: &Placement, angle: f64) -> Placement;
pub fn centre_on_plate(p: &Placement, v: &BuildVolume) -> Placement;
/// Normal used for "place on plate" when the user picks triangle `f` (note §4).
pub fn pick_plane_normal(m: &IndexedMesh, f: FaceId) -> DVec3;

#[derive(Clone, Debug, PartialEq)]
pub struct FitReport {
    pub fits: bool,
    pub height_excess: f64,                  // mm above volume.height (0 if ok)
    pub max_outside_xy: f64,                 // mm the footprint hull leaves the bed (0 if ok)
    pub hits_exclude: bool,
    pub footprint_hull: Vec<DVec2>,          // CCW, plate frame
    pub aabb: Aabb3,                         // plate frame
}
pub fn check_fit(m: &IndexedMesh, t: &Transform, v: &BuildVolume, ctx: &JobCtx) -> Result<FitReport, AmError>;
pub fn footprint_hull(m: &IndexedMesh, t: &Transform) -> Vec<DVec2>;
/// SAT on convex hulls; true if closer than `gap`.
pub fn hulls_overlap(a: &[DVec2], b: &[DVec2], gap: f64) -> bool;

pub struct BuildVolumeGeometry {
    pub plate: IndexedMesh,                  // thin slab, top at z = 0
    pub grid: Vec<Polyline3>,                // 10 mm grid on z = 0
    pub outline: Vec<Polyline3>,             // volume edges
    pub exclude: Vec<IndexedMesh>,
}
pub fn build_volume_geometry(v: &BuildVolume, grid_step: f64) -> BuildVolumeGeometry;

// ───────────────────────── amg_am::overhang (AM-02) ─────────────────────────
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct AnalysisSettings {
    pub critical_angle: f64,                 // α, rad
    pub contact_tolerance: Option<f64>,      // None → Machine::default_contact_tolerance
    pub support_cell: f64,                   // mm, default 0.5
    pub max_rays: u32,                       // default 262_144
}
impl AnalysisSettings { pub fn for_machine(m: &Machine) -> Self; }

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum FaceClass { Supported = 0, Overhang = 1, Contact = 2, Degenerate = 3 }

#[derive(Clone, Debug)]
pub struct OverhangOpts {
    pub critical_angle: f64,
    pub contact_tolerance: f64,              // resolved (mm)
    pub support: Option<SupportOpts>,        // None = skip rays (classification + area metrics only)
}
#[derive(Clone, Copy, Debug)]
pub struct SupportOpts { pub cell: f64, pub max_rays: u32 }

#[derive(Clone, Debug, PartialEq)]
pub struct SupportEstimate {
    pub volume: f64, pub on_plate_volume: f64, pub on_part_volume: f64,
    pub footprint_area: f64,                 // area of grid cells with any counted gap
    pub cell: f64,                           // cell actually used (≥ requested)
    pub rays: u32,
    pub method: SupportMethod,
}
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SupportMethod { Columns, FaceCentroids }

#[derive(Clone, Debug, PartialEq)]
pub struct OverhangMetrics {
    pub overhang_area: f64,
    pub overhang_projected_area: f64,
    pub contact_area: f64,
    pub footprint_area: f64,
    pub height: f64,                         // z extent of the part (excl. elevation)
    pub island_count: usize,
    pub stable: Option<bool>,
    pub com_margin: Option<f64>,             // mm, signed (negative = tips over)
    pub support: Option<SupportEstimate>,
    pub approximate: bool,
}

#[derive(Clone, Debug)]
pub struct OverhangReport {
    pub transform: Transform,                // the T analysed
    pub face_class: Vec<FaceClass>,          // len == faces
    pub down_angle: Vec<f64>,                // ψ per face, rad ∈ [0, π]
    pub islands: Vec<VertexId>,              // sorted
    pub metrics: OverhangMetrics,
}

/// Classification only: O(F), parallel, no rays. Used for coarse live updates.
pub fn classify_faces(m: &IndexedMesh, t: &Transform, opts: &OverhangOpts, ctx: &JobCtx) -> Result<(Vec<FaceClass>, Vec<f64>), AmError>;
/// Full analysis; `s` is the body's MeshSpatial (body-local, MESH-03).
pub fn analyze_overhang(s: &MeshSpatial, t: &Transform, opts: &OverhangOpts, ctx: &JobCtx) -> Result<OverhangReport, AmError>;
/// Area-weighted per-vertex severity ∈ [0,1] (0 = fine, 1 = horizontal ceiling) for RND-08 per-vertex overlays.
pub fn vertex_severity(m: &IndexedMesh, r: &OverhangReport, critical_angle: f64) -> Vec<f64>;
/// Severity of one face from its down angle: clamp((cos ψ − cos α)/(1 − cos α), 0, 1).
pub fn severity(down_angle: f64, critical_angle: f64) -> f64;
/// Uniforms for the S10 overhang shader, in body-local space (f32 conversion happens in amg-ui).
pub struct OverhangShaderParams { pub build_dir_local: DVec3, pub cos_critical: f64, pub contact_plane: (DVec3, f64), pub contact_tolerance: f64 }
pub fn shader_params(t: &Transform, z_min: f64, opts: &OverhangOpts) -> OverhangShaderParams;

// ───────────────────────── amg_am::orient (AM-04, AM-13) ─────────────────────────
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct OrientWeights { pub support_volume: f64, pub overhang_area: f64, pub height: f64, pub contact: f64, pub cosmetic: f64, pub staircase: f64 }
#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum OrientPreset { MinSupport, MinHeight, SurfaceQuality, Resin }
impl OrientPreset { pub fn weights(self) -> OrientWeights; }

#[derive(Clone, Debug)]
pub struct OrientOpts {
    pub weights: OrientWeights,
    pub analysis: OverhangOpts,              // support: Some(..) required
    pub sphere_samples: u32,                 // 1024
    pub hull_poses: bool,                    // true for on-plate technologies
    pub normal_clusters: u32,                // 10
    pub refine_top_k: u32,                   // 8
    pub refine_min_step: f64,                // 0.25° in rad
    pub n_results: u32,                      // 5
    pub cosmetic_faces: Vec<FaceId>,
    pub volume: Option<BuildVolume>,         // fit guard
    pub elevated: bool,                      // resin: no stable-pose restriction
    pub backend: ComputeBackend,             // Cpu until AM-13
}
#[derive(Clone, Debug, PartialEq)]
pub struct OrientTerms { pub support: f64, pub overhang: f64, pub height: f64, pub contact: f64, pub cosmetic: f64, pub staircase: f64, pub guards: f64 }
#[derive(Clone, Debug)]
pub struct OrientResult { pub rotation: DQuat, pub score: f64, pub terms: OrientTerms, pub metrics: OverhangMetrics }
/// Results sorted by score (best first), distinct by ≥ 2°. Deterministic.
pub fn optimize_orientation(s: &MeshSpatial, opts: &OrientOpts, gpu: Option<&Gpu>, ctx: &JobCtx) -> Result<Vec<OrientResult>, AmError>;
pub fn score_orientation(s: &MeshSpatial, rotation: DQuat, opts: &OrientOpts, ctx: &JobCtx) -> Result<OrientResult, AmError>;

// ───────────────────────── amg_am::handoff (AM-06) ─────────────────────────
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum DetectSource { Registry, KnownDir, Path, User }
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SlicerInstall { pub kind: SlicerKind, pub exe: PathBuf, pub version: Option<String>, pub source: DetectSource }
/// Registry (HKLM/HKCU Uninstall, cfg(windows) via `winreg`), well-known dirs, PATH. Sorted, de-duplicated.
pub fn detect_slicers() -> Vec<SlicerInstall>;
/// Extract an existing .exe path from a registry DisplayIcon/InstallLocation value; handles
/// `"D:\X\D:\X\app.exe"`, trailing `,0` icon indices and quotes.
pub fn parse_registry_exe(value: &str) -> Option<PathBuf>;

#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum HandoffFormat { ThreeMf, Stl }
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SlicerConfig { pub kind: SlicerKind, pub exe: PathBuf, pub args: Vec<String> /* "{file}" placeholder; default ["{file}"] */, pub format: HandoffFormat }

pub struct HandoffItem { pub name: String, pub mesh: std::sync::Arc<IndexedMesh>, pub transform: Transform /* body-local → plate */, pub filament: Option<u8>, pub color: Option<Rgba8> }
pub struct HandoffPlate<'a> { pub design_name: String, pub plate_name: String, pub machine: &'a Machine, pub items: Vec<HandoffItem>, pub thumbnail_png: Option<Vec<u8>> }
#[derive(Clone, Debug)]
pub struct HandoffReport { pub path: PathBuf, pub objects: usize, pub triangles: usize, pub bytes: u64, pub millis: u32 }

pub fn handoff_path(dir: &Path, design: &str, plate: &str, now: std::time::SystemTime) -> PathBuf; // sanitised, timestamped
/// Writes atomically (tmp + rename) via amg_io; plate → slicer coords with `machine.volume.slicer_origin`.
pub fn export_plate(p: &HandoffPlate<'_>, format: HandoffFormat, flavor_for: SlicerKind, path: &Path, ctx: &JobCtx) -> Result<HandoffReport, AmError>;
/// Spawns detached; never waits. Returns the child pid.
pub fn launch(cfg: &SlicerConfig, file: &Path) -> Result<u32, AmError>;
/// Deletes all but the newest `keep` *.3mf/*.stl in `dir`.
pub fn prune_handoff_dir(dir: &Path, keep: usize) -> std::io::Result<usize>;

// ───────────────────────── errors ─────────────────────────
#[derive(Debug, thiserror::Error)]
pub enum AmError {
    #[error(transparent)] Mesh(#[from] amg_mesh::MeshError),
    #[error("export failed: {0}")] Io(String),                         // from amg_io error (S12 owns the type)
    #[error("invalid machine profile `{key}`: {reason}")] Machine { key: String, reason: String },
    #[error("cannot read profile {file}: {reason}")] Profile { file: PathBuf, reason: String },
    #[error("profile inheritance of `{preset}` is broken: {reason}")] Inherits { preset: String, reason: String },
    #[error("item transform is not rigid")] NotRigid,
    #[error("nothing to export: plate is empty")] EmptyPlate,
    #[error("slicer executable not found: {0}")] SlicerNotFound(PathBuf),
    #[error("could not start the slicer: {0}")] Launch(String),
    #[error("operation cancelled")] Cancelled,
}
impl From<amg_core::Cancelled> for AmError { fn from(_: amg_core::Cancelled) -> Self { AmError::Cancelled } }
impl amg_core::Diagnostic for AmError { /* hints: "Set the slicer path in Preferences → Manufacture" … */ }
```

```rust
// ───────────────────────── amg_features::am (AM-01, AM-06 glue) ─────────────────────────
use amg_doc::{BodyId, Command, Design, Evaluated, ExtKey, ExtRecord, ExtRegistration, MigrationError};
use amg_am::{machine::Machine, place::Placement, overhang::AnalysisSettings};

pub const PRINT_SETUP_KIND: &str = "am.print_setup";
pub const PRINT_SETUP_VERSION: u32 = 1;                     // 2 after AM-04 (cosmetic groups)

#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize)]
pub struct PlateId(pub u32);
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize)]
pub struct ItemId(pub u32);

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PlateItem { pub id: ItemId, pub body: BodyId, pub placement: Placement, pub filament: Option<u8> }
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Plate { pub id: PlateId, pub name: String, pub items: Vec<PlateItem> }
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PrintSetup {
    pub name: String,
    pub machine: Machine,                                   // snapshot
    pub settings: AnalysisSettings,
    pub plates: Vec<Plate>,                                 // ≥ 1
    pub active_plate: PlateId,
    pub next_plate_id: u32,
    pub next_item_id: u32,
}
impl PrintSetup {
    pub fn new(name: &str, machine: Machine) -> Self;       // one empty plate
    pub fn add_body(&mut self, plate: PlateId, body: BodyId, placement: Placement) -> ItemId;
    pub fn remove_item(&mut self, id: ItemId) -> bool;
    pub fn item_mut(&mut self, id: ItemId) -> Option<&mut PlateItem>;
    pub fn add_plate(&mut self, name: &str) -> PlateId;
    pub fn to_record(&self) -> ExtRecord;                   // deterministic JSON (BTreeMap order)
    pub fn from_record(r: &ExtRecord) -> Result<Self, AmFeatureError>;
    /// The one mutation path: whole-record replace.
    pub fn command(&self, key: ExtKey) -> Command { Command::SetExt { key, record: Some(self.to_record()) } }
}
pub fn delete_setup_command(key: ExtKey) -> Command;        // SetExt { record: None }
/// Lowest unused id among `am.print_setup` keys (needs S01 `Design::exts_of_kind`).
pub fn next_setup_key(d: &Design) -> ExtKey;
pub fn setups(d: &Design) -> Vec<(ExtKey, Result<PrintSetup, AmFeatureError>)>;

#[derive(Clone, Debug)]
pub enum ItemGeom { Mesh(amg_mesh::MeshBody), Unresolved(UnresolvedReason) }
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum UnresolvedReason { BodyMissing, NotAMesh /* solid without tessellation (Phase 1), implicit */, Hidden }
pub struct ResolvedItem<'s> { pub item: &'s PlateItem, pub geom: ItemGeom, pub name: String, pub color: Option<amg_mesh::Rgba8> }
pub fn resolve_plate<'s>(s: &'s PrintSetup, plate: PlateId, d: &Design, e: &Evaluated) -> Vec<ResolvedItem<'s>>;
/// Effective transforms + meshes → hand-off input (skips unresolved, reports them).
pub fn handoff_plate<'s>(s: &'s PrintSetup, plate: PlateId, d: &Design, e: &Evaluated) -> (amg_am::handoff::HandoffPlate<'s>, Vec<(ItemId, UnresolvedReason)>);

#[derive(Debug, thiserror::Error)]
pub enum AmFeatureError {
    #[error("print setup record is invalid: {0}")] Decode(String),
    #[error("print setup v{0} is newer than supported")] TooNew(u32),
    #[error(transparent)] Am(#[from] amg_am::AmError),
}
inventory::submit! { ExtRegistration { kind: PRINT_SETUP_KIND, version: PRINT_SETUP_VERSION, migrate: migrate_print_setup } }
fn migrate_print_setup(from: u32, v: serde_json::Value) -> Result<serde_json::Value, MigrationError>;
```

`amg-ui::am` exposes no API to other crates. It registers commands `am.new_setup`,
`am.add_bodies`, `am.place_face_down`, `am.drop`, `am.move_rotate`, `am.centre`,
`am.overhang_toggle`, `am.auto_orient`, `am.arrange`, `am.send_to_slicer` and `am.export_3mf`
with the UI-02 registry.

**Expected from other specs** (requests, not edits; listed in the refinement report):
- **S01:** `Design::exts_of_kind(&self, kind: &str) -> impl Iterator<Item = (&ExtKey, &ExtRecord)>`
  (additive). `ExtChanged` on `SetExt` (already in DOC-09).
- **S07:** none new. Uses `IndexedMesh::{face_normal, face_area, aabb, signed_volume,
  content_hash}`, `MeshBody::{spatial, analysis}`, `MeshSpatial::{ray_cast, ray_cast_all}`,
  `AnalysisReport::is_watertight`, `wall_thickness` (AM-07), `section` (AM-05/07), and
  `FaceGroups` + `fit_primitive` info for `pick_plane_normal`.
- **S10 (RND-08):** an "overhang" overlay mode on a body instance. Uniforms (f32, body-local):
  `build_dir`, `cos_critical`, `contact_plane: vec4`, `contact_tolerance`, and three colours
  (supported / overhang ramp / contact). It uses the flat face normal (`primitive_index` →
  normal buffer, or screen-space derivative). It must pass the AM-02 golden classification file.
  Also: an optional **per-face scalar or class buffer** indexed by `primitive_index` (S07
  guarantees face order), so the exact CPU classes can be shown after the job.
- **S10 (RND-03/07):** non-pickable helper scene items (a mesh with flat colour and alpha, and
  polyline sets with colour and width) for the plate, grid, volume box and exclude areas; plus
  a per-instance tint (red for out-of-volume).
- **S12 (IO-03/IO-04):** a 3MF writer API that takes several objects with build-item transforms,
  object names, a per-object extruder (filament slot), an optional per-triangle extruder
  (painting, P1), object colour, unit = mm and a plate thumbnail. It also needs
  `SlicerFlavor::{Core, Orca, Prusa}`, which adds the metadata OrcaSlicer 2.4 reads for
  per-object extruder and keeps positions. Phase 1 needs per-object extruders and positions;
  plate assignment and project settings are IO-04. Sketch:
  ```rust
  pub struct Model3mf { pub objects: Vec<Object3mf>, pub items: Vec<BuildItem3mf>, pub metadata: Vec<(String, String)>, pub thumbnail_png: Option<Vec<u8>> }
  pub struct Object3mf { pub name: String, pub mesh: Arc<IndexedMesh>, pub extruder: Option<u8>, pub tri_extruder: Option<Vec<u8>>, pub color: Option<Rgba8> }
  pub struct BuildItem3mf { pub object: usize, pub transform: Transform, pub plate: Option<u32> }
  pub enum SlicerFlavor { Core, Orca, Prusa }
  pub fn write_3mf<W: Write + Seek>(m: &Model3mf, flavor: SlicerFlavor, w: W, ctx: &JobCtx) -> Result<(), IoError>;
  pub fn write_stl<W: Write>(meshes: &[(&IndexedMesh, Transform)], binary: bool, w: W, ctx: &JobCtx) -> Result<(), IoError>;
  ```
- **S15 (CMP-01):** sort, prefix scan, segmented reduce and atomic counter buffers (AM-13).
- **S17 (MAT-01):** optionally, `PhysicalMaterial::am: Option<AmProps { technology, critical_overhang_angle: Option<Angle>, min_wall: Option<f64> }>`
  to default α per material (PLA vs PETG vs resin). P1, not blocking.
- **S11:** a Manufacture workspace slot in the workspace switcher (UI-01), and the
  preferences page "Manufacture" (UI-11, P1) for slicer paths and the hand-off directory.
  Phase 1 keeps these in an `amg-ui::am` settings dialog.

## Work packages
WPs are ≈ 1–3 days each. Times are on the reference machine, release build, 8+ cores. AM-01..09
keep their IDs (several re-scoped). AM-10..16 are new. "Test set" = `testdata/am/` (see
Testing).

| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| AM-01 | `amg-am::machine` (`Machine`, `BuildVolume`, `MachineLibrary`, TOML (de)serialisation, built-ins: Ender-3 V3 KE, Ender-3 V3 KE KCM 4-slot, ELEGOO Mars 3 Pro, generic FFF 220³, generic MSLA) + `amg-features::am` `PrintSetup` record, registration, `next_setup_key`, `setups` | DOC-05, DOC-07, FND-03 | P0 | Todo | 1. Every built-in profile validates, and TOML round-trip is lossless (proptest over random valid `Machine`s). 2. An invalid user TOML gives a `LoadWarning` naming file and field, and the library still loads. 3. A `PrintSetup` with 2 plates and 3 items round-trips `to_record`/`from_record` and a `.amg` save→load→save with byte-identical `design.json`. 4. `SetExt` edits undo/redo to equal records. 5. A v0 fixture record is rejected with `TooNew`/`Decode` (no panic). 6. `next_setup_key` never reuses a deleted key within a session. |
| AM-03 | `amg-am::place`: `Placement`/`ZMode`, `effective_transform`, `face_down_rotation`, `rotate_about_build_axis`, `centre_on_plate`, `pick_plane_normal`, `check_fit` (hull vs bed polygon, excludes, height), `hulls_overlap`, `build_volume_geometry` | AM-01, MESH-01 | P0 | Todo | 1. For 1 000 random rotations of corpus meshes: min z after `effective_transform` = elevation ± 1e-9 mm, and the footprint-AABB centre = `xy` ± 1e-9 mm. 2. Face-down on each face of a cube and of a tessellated 12-gon prism puts that face's normal at −Z within 1e-12 rad, with the anti-parallel case covered. 3. `check_fit` matches brute-force vertex containment on 200 random placements over rect, non-convex polygon and circle beds with excludes, and reports `max_outside_xy` within 1e-6 mm. 4. Modifying the mesh (scale 1.1 about an off-centre point) keeps the item on the plate at the same `xy`. 5. `effective_transform` + `check_fit` on 1 M tris < 15 ms; `footprint_hull` 1 M < 40 ms. |
| AM-02 | `amg-am::overhang`: `classify_faces`, `analyze_overhang` (column support estimate + centroid fallback, islands, contact, stability), `vertex_severity`, `shader_params`, golden classification file for S10 | MESH-02, MESH-03, AM-03 | P0 | Todo | 1. Golden tests in `tests/golden/am/` reproduce note §3.3: T-shape and shelf volumes within 0.5 % at 0.25 mm cells; the sphere cap area, projected area and support volume converge (error at least halves per 4× triangles / ½ cell, < 1 % at 320 k tris and 0.1 mm); the cube cases are exact (1e-9). 2. Strict-threshold behaviour at α = 45° vs 46° on the edge-balanced cube. 3. Rotation about Z by 37° changes area metrics by < 1e-9 relative and support volume by ≤ the documented cell bound. 4. An open mesh gives `approximate = true` with `FaceCentroids`. 5. `overhang_classes.toml` (5 000 cases) is generated and checked. 6. Results are identical across 1/4/16 threads. 7. Benchmarks: `classify_faces` 1 M tris < 15 ms; `analyze_overhang` 1 M tris with default support (≤ 512² rays) < 300 ms, coarse (≤ 128² rays) < 40 ms. |
| AM-06 | Slicer hand-off library: `detect_slicers` (registry via `winreg`, known dirs, PATH), `parse_registry_exe`, `SlicerConfig`, `export_plate` (via IO-03, 3MF/STL, slicer-origin offset, atomic write), `launch` (detached), `prune_handoff_dir`; `amg-features::am::handoff_plate` | AM-03, IO-11, IO-04 | P0 | Todo | 1. `parse_registry_exe` handles the owner's `D:\OrcaSlicer\D:\OrcaSlicer\orca-slicer.exe`, quoted paths and `,0` suffixes (unit fixtures). 2. Exported 3MF of a 3-item plate (two filament slots) re-imported through IO-03 gives the same plate-frame vertex positions within 1e-6 mm, names and extruders. 3. **Manual check (documented in PR):** on the owner's machine, one click opens the file in OrcaSlicer 2.4.0 with all items at the Amalgam plate positions (± 0.1 mm, read off the Orca object list), assigned to the right filament slots on the KCM preset, without a re-arrange prompt. The same file opens in Chitubox, and the chosen Chitubox format (3MF or STL) is recorded in the Refinement log. 4. A missing exe gives `SlicerNotFound` with a hint. 5. Export + spawn of a 1 M-tri plate < 3 s, and the UI is not blocked (runs as a job). |
| AM-11 | `amg-ui::am` workspace (Phase 1): Manufacture workspace, setup browser, machine picker (library + user dir), add/remove bodies, plate scene with build-volume helper geometry, Place-on-plate face pick, Drop, Move/Rotate numeric dialog, Centre, out-of-volume tint + message, settings dialog (slicer path, hand-off dir), Send to Slicer / Export 3MF buttons | AM-01, AM-03, AM-06, UI-02, UI-03, UI-04, UI-10, RND-03, RND-07 | P0 | Todo | 1. Every command listed under Interfaces is reachable from the toolbar and S-key search, and each edit is one undo entry. 2. Picking a face of an imported STL puts it on the plate. 3. An item moved out of the volume turns red with a size message. 4. No frame > 16 ms in the tracing profile while placing a 1 M-tri body. 5. A manual Phase 1 script (open STL → new setup on Ender-3 V3 KE KCM → place face down → Send to Slicer) passes on 3 corpus models. |
| AM-12 | `amg-ui::am` overhang overlay + metrics panel: RND-08 overhang mode driven by `shader_params`, coarse/fine CPU analysis jobs with cancel-and-supersede, per-face class buffer after the fine job, island markers, per-item/plate metrics, α/contact settings in the setup | AM-02, AM-11, RND-08 | P0 | Todo | 1. Rotating a 1 M-tri item (numeric dialog with live preview) keeps the overlay in sync every frame. Coarse metrics update ≥ 10 Hz, fine metrics arrive < 500 ms after release, and no frame > 16 ms. 2. The overlay and the CPU class buffer agree on ≥ 99.9 % of faces of the test set (headless comparison once RND-09 exists; until then a screenshot in the PR). 3. With the overlay on, a 5 M-tri mesh orbits at ≥ 60 fps (vision budget, S10 bench). 4. Changing α updates colours immediately and metrics within 500 ms. |
| AM-10 | Profile import: `SlicerDataDirs::discover`, Orca JSON with `inherits` chain (user/system/install roots), Chitubox `.cfgx`, PrusaSlicer INI; `orca_process_critical_angle`; "save as user profile" in the AM-11 machine picker | AM-01, AM-11 | P1 | Todo | 1. Fixture copies of the Orca system + user KE/KCM presets (committed under `testdata/am/profiles/`, values only, no personal G-code) resolve to 220 × 220 × 240 (user override) and 4 filament slots. 2. Abstract presets (`instantiation: false`) are not listed; a cycle and a missing parent each give an `Inherits` warning. 3. The Chitubox fixture gives the Mars 3 Pro values (143.43 × 89.60 × 175, 4098 × 2560). 4. A Prusa INI fixture parses `bed_shape`/`max_print_height`. 5. On the owner's machine `discover()` finds all three dirs (manual, PR). |
| AM-04 | Orientation optimisation, CPU reference: stable hull poses, normal clusters, Fibonacci candidates, pattern-search refine, score terms + presets, fit guard, cosmetic face groups (setup record v2 + migration), Auto-Orient dialog in `amg-ui::am` showing the top 5 | AM-02, AM-12, MESH-16 | P1 | Todo | On the test set (≥ 25 models: synthetic brackets/bridges/mushrooms/T/L/overhang ladders + corpus models): 1. The chosen score is ≤ the best of the 6 axis-aligned orientations ("manual" baseline) on 100 % of models and ≥ 10 % lower on ≥ 40 %. 2. Within 2 % of a brute-force 16 384-direction optimum on ≥ 90 %. 3. Deterministic (same output across runs and thread counts). 4. 100 k tris < 2 s, 1 M tris < 20 s (CPU, Min-support preset). 5. v1 setup records migrate to v2 on load. |
| AM-13 | Orientation GPU scoring: per-direction classification dispatch, A-buffer support-volume pass, CMP-01 reductions; `ComputeBackend::Gpu/Auto` | AM-04, CMP-01, FND-09 | P1 | Todo | GPU support volume within 1 % of CPU at equal cell size on the test set, and the same best orientation on ≥ 95 % (ties within 0.5 % excepted). 1 M tris, 1 024 candidates + refine < 5 s on the RTX 5060 Ti. CPU fallback is used when there is no adapter (WARP test in CI). |
| AM-05 | Arrange (2D): convex-hull footprints + spacing, convex NFP bottom-left-fill with 15° yaw steps, overflow to new plates, "Arrange" command | AM-03, AM-11 | P1 | Todo | 1. 50 random corpus items on the KE bed: no overlaps (SAT, spacing ≥ requested − 1e-6), all inside the volume, and plate utilisation within 15 % of an offline best-of-1000-random-orders reference. 2. Deterministic. 3. 100 items < 2 s. |
| AM-07 | Printability FFF/general: thin walls (MESH-11 vs nozzle/min feature), small features, layer islands and max bridge from a sliced preview (`section` per layer), report + overlay | AM-02, MESH-11, MESH-09 | P1 | Todo | Synthetic cases: a 0.6 mm wall on a 0.4 mm nozzle profile (k = 2) is flagged and a 1 mm wall is not; a floating island at 10 mm is reported at the right layer ± 1; a 30 mm bridge measures 30 ± layer width. A 1 M-tri model at 0.2 mm layers < 10 s as a cancellable job. |
| AM-15 | Printability resin: trapped volumes / suction cups (voxel flood fill along −d), drain-hole suggestions for enclosed cavities, per-layer cross-section area plot + peak, minimum island size vs pixel grid | AM-02, MESH-09 | P1 | Todo | An upside-down cup (open towards the plate) is reported with its volume within 2 %. A hollow sphere has one enclosed cavity and a suggested hole at its lowest point. The cross-section area of a 20 mm cylinder equals π·10² ± 0.5 % per layer. Results feed the Resin orientation preset. 1 M tris at 0.05 mm layers < 15 s (job). |
| AM-08 | Support preview (tree): support points from AM-02 (overhang sampling + islands), downward merging within the α cone, resin tips + raft, preview mesh + volume/contact count, optional "export with supports" for resin | AM-02, AM-15 | P1 | Todo | On the test set every support point is connected to the plate or part, no branch exceeds α from vertical, and preview supports do not intersect the part (MESH-19 check). Support volume is within 2× of the AM-02 estimate. 100 k-tri model < 3 s. |
| AM-14 | 3D packing for SLS (voxel height-field first-fit decreasing) | AM-05 | P3 | Todo | 30 random corpus parts in a 150³ volume: no voxel overlap at a 1 mm grid plus spacing, packing density ≥ 15 %, deterministic. |
| AM-09 | Metal PBF distortion simulation (inherent strain): layer activation on voxel FE, strain calibration per material, distortion field overlay, recoater collision warning | FEM-04, CMP-05, AM-02 | P3 | Todo | Cantilever benchmark (NIST AM-Bench-style geometry) distortion within 15 % of the published value; 1 M voxels < 5 min (vision topopt budget class). |

Phase placement for the new and re-scoped WPs is proposed in the Refinement log. The roadmap
is not edited here.

## Testing
- **Unit tests** per module, including all the analytic cases in note §3.3 and §4.
- **Golden geometry** in `tests/golden/am/` (via `amg-testkit`):
  - effective transforms;
  - overhang metrics (T, shelf, sphere convergence, cubes);
  - fit reports;
  - the `overhang_classes.toml` cross-check consumed by S10.
- **Property tests (`proptest`):**
  - placement invariants (on plate, anchor, rigidity);
  - yaw invariance of metrics;
  - `check_fit` vs brute force;
  - `hulls_overlap` vs polygon clipping brute force;
  - `Machine` TOML round-trip;
  - `PrintSetup` record round-trip.
- **Determinism:** analysis and orientation run on 1 and 16 rayon threads, and twice in a row,
  must give identical outputs.
- **Test set** `testdata/am/manifest.toml`, reusing the MESH-28 harness:
  - synthetic generators (T, L, shelf, bridge, mushroom, overhang ladder 10°–80°, cup,
    hollow sphere, brackets) with seed and parameters;
  - ≥ 15 corpus models;
  - profile fixtures (Orca / Chitubox / Prusa JSON/INI, with personal G-code stripped).
- **Manual hand-off check** (AM-06, AM-11): scripted steps, results recorded in the PR.
  Automating the slicer GUI is out of scope.
- **Benchmarks (`criterion`, FND-08):**
  - `classify_faces`, `analyze_overhang` (coarse/fine), `effective_transform`, `check_fit`;
  - orientation at 100 k / 1 M;
  - export + spawn.

## Performance targets
Reference machine, release build. Inputs are displaced icospheres from the MESH-28 benchmark
generator. Anything above 16 ms runs as a job.

| Operation | 100 k tris | 1 M tris | Note |
|---|---|---|---|
| `classify_faces` | 2 ms | 15 ms | per-frame capable, but the overlay uses the shader |
| `analyze_overhang`, coarse (≤ 128² rays) | 8 ms | 40 ms | ≥ 10 Hz during drag |
| `analyze_overhang`, fine (≤ 512² rays) | 60 ms | 300 ms | on release |
| `effective_transform` + `check_fit` | 2 ms | 15 ms | |
| Overlay orbit, 5 M tris | ≥ 60 fps | | vision viewport budget (S10) |
| Orientation, CPU, Min-support | 2 s | 20 s | AM-04 |
| Orientation, GPU | 0.5 s | 5 s | AM-13 |
| Export plate + spawn slicer | 0.5 s | 3 s | AM-06; dominated by IO-03 |
| UI frame while any of the above runs | ≤ 16 ms | | vision budget |

## Open questions
1. **(owner, answered 2026-10-20)** Printers and slicers: FFF on a Creality Ender 3 V3 KE with
   an MMU mod (multi-material), and an MSLA resin printer. Slicers: OrcaSlicer (FFF) and Chitubox
   (resin).
   - AM-06 targets OrcaSlicer (multi-material 3MF) and Chitubox (STL/3MF + launch).
   - Multi-material 3MF export (per-body, later per-face filament) is a must-have for IO-03.
   - Resin checks and the resin support preview gain priority: AM-15 is new at P1, and AM-08
     moves from P2 to P1.
2. **(owner, new)** Please confirm the resin printer. The owner's Chitubox config contains an
   **ELEGOO Mars 3 Pro** (143.43 × 89.60 × 175 mm, 4098 × 2560 px). Built-in AM-01 profile:
   use it as the default resin machine? Is the MMU mod the **CoPrint KCM** (4 filament slots,
   one nozzle), as the owner's Orca preset "… - Connected KCM" suggests?
   *Default:* yes to both.
3. **(owner, new)** Default critical overhang angle: 45° (DfAM rule, Fusion default; flags more
   faces than Orca will actually support) or 30° (OrcaSlicer's default `support_threshold_angle`,
   so "red" ≈ "Orca will add supports")?
   *Default:* 45° for analysis, with the imported Orca process value offered once AM-10 lands.
4. Resolved: the angle convention follows Orca/Prusa ("slope from horizontal below α"), and the
   Cura conversion is documented (note §1.1).
5. Resolved: print setups are extension records with an embedded machine snapshot, not
   timeline features (S01 §11, ADR-014).
6. Resolved: Phase 1 hand-off is per-object filament slots. Per-face painting waits for IO-04
   and the S12 per-triangle extruder support (P1).
7. Resolved at implementation (AM-06 check, not an owner decision): whether Chitubox gets 3MF or
   STL, and which Orca 3MF metadata keeps positions and extruders. The result is recorded in
   the Refinement log.

## Refinement log
**2026-10-20 — S16 refinement (lane G agent).**

*Researched:*
- OrcaSlicer sources (AGPL-3.0, behaviour only):
  - `src/libslic3r/Orient.cpp` / `Orient.hpp`, the Tweaker-3 derivative: area-cumulation and
    hull candidates, appearance-face ×3 penalty, `ASCENT = cos(π − α)`, bottom/contour terms,
    `BOTTOM_MIN` guard;
  - `src/libslic3r/SLA/Rotfinder.cpp` (PrusaSlicer heritage): convex-hull rotations when on the
    floor, else a √N×√N grid over X/Y; per-face score `√A·(1 − ψ/π)³`; misalignment and
    Z-fit finders;
  - `PrintConfig.cpp`: `support_threshold_angle` default 30°, slope-from-horizontal convention.
- Tweaker-3 `MeshTweaker.py`: unprintability formula and evolutionary-fitted constants.
- Papers:
  - orientation and support volume: Ezair–Massarwi–Elber 2015 (GPU support volume,
    non-smooth); Zhang et al. 2015 (perceptual direction preference); Delfs et al. 2016
    (surface quality vs orientation); Pandey 2003 / Thrimurthulu 2004 (cusp height);
  - supports: Vaněk et al. 2014 and Schmidt–Umetani 2014 (tree supports);
  - arrange: Burke et al. 2007 (NFP);
  - Langelaar (AM filter, convention shared with S14).
- Slicer hand-off: OrcaSlicer CLI (`orca-slicer [input files…] [options…]`, official wiki).
- **Owner's machine, read-only inspection:**
  - OrcaSlicer 2.4.0, with its malformed registry entry;
  - the Orca data dir layout and `inherits` chain;
  - the user KCM preset (4 slots, 240 mm height);
  - Chitubox v1.2.0 / Basic 2.x `.cfgx` JSON with an ELEGOO Mars 3 Pro;
  - `.3mf` associated with Creality Print 7.1;
  - PrusaSlicer also present.
- Licences checked on crates.io: `winreg` 0.56.0 MIT; `toml` 1.1.8 MIT/Apache-2.0.

*Decided:*
- frames, anchor-based placement and the strict overhang rule (note §1);
- face classes, island detection and the shader parity file;
- the column support-volume method with on-part split, and the open-mesh fallback;
- 2-DOF orientation search with stable poses, clusters, Fibonacci sampling and pattern search;
- normalised score with presets;
- print setup as an extension record with a machine snapshot;
- the doc-facing types live in `amg-features::am`, so `amg-am` stays free of `amg-doc`;
- hand-off by executable path, detached spawn, hand-off directory;
- built-in profiles for the owner's printers.

*Changed WPs:*
- Re-scoped:
  - AM-01: machine library + setup record; deps +DOC-05, +FND-03;
  - AM-02: algorithm only; deps MESH-02 + MESH-03 + AM-03 (RND-08 moved to AM-12);
  - AM-03: algorithm only; deps AM-01 + MESH-01 (RND-03 moved to AM-11);
  - AM-04: CPU reference + UI; deps AM-02, AM-12, MESH-16 (CMP-01 moved to AM-13);
  - AM-05: 2D only; deps +AM-11;
  - AM-06: library + glue; deps +AM-03;
  - AM-07: FFF/general; deps +AM-02, +MESH-09;
  - AM-08: tree preview; P2 → **P1** (owner); deps +AM-15;
  - AM-09: deps +AM-02.
- New:
  - AM-10 profile import (P1);
  - AM-11 Phase 1 workspace UI (P0);
  - AM-12 overlay + metrics UI (P0);
  - AM-13 GPU orientation scoring (P1);
  - AM-14 SLS 3D packing (P3, split from AM-05);
  - AM-15 resin printability (P1, owner).
- Every WP has a measurable Done-when.
- Added Scope, Interfaces (concrete), Testing, Performance targets and Open questions.

*Roadmap phase-order notes* (roadmap not edited; proposals are in the report):
- Phase 1, lane G: AM-01, AM-02, AM-03, AM-06, **AM-11, AM-12** (new). All their deps are in
  Phase 0/1: DOC-05/07, MESH-01..03, IO-03, UI-02..04/10, RND-03/07/08.
- AM-10: Phase 1 if lane G has capacity, else Phase 3.
- Phase 3: AM-07, AM-15.
- Phase 4: AM-04, AM-13, AM-05; AM-08 moves Phase 5 → 4 (P1).
- Phase 6: AM-14 next to AM-09.
- Check: AM-04 needs MESH-16 (Phase 1) and AM-13 needs CMP-01 (Phase 2), so nothing is
  scheduled before its deps.

*Design note created:* `docs/notes/am-build-prep.md`.

## Interface changes

### 2026-10-20: requests from the wave-2 refinements (S02, S05, S10, S11, S12, S16, S17, S20)

Additive requests recorded by the integration pass; the owning lane confirms or adjusts them when it implements the affected WP.

- **From S12 (import/export).** AM-06 now depends on IO-11 (Generic writer) + IO-04
  (Orca/Bambu flavour) instead of IO-03 (now the reader); applied in the WP table. The
  `Model3mf`/`write_3mf` sketch maps 1:1 onto `amg_io::Scene` + `export` (see S12 Interface
  changes). Per-face painting can be Phase 1, because IO-04 is P0 in Phase 1.
- **From S10 (rendering).** AM-02/AM-12 and S10 RND-08 must use the same overhang rule (S10 note
  §11 vs `docs/notes/am-build-prep.md` §1); the AM-02 golden file is the arbiter. S10's request
  that AM-03 depend on RND-07 is satisfied by AM-11, which already depends on RND-07.
- **From S05 (inspection).** `amg-ui::am` may reuse `amg_features::inspect::fields::draft_angle`
  for overhang (the same field with d = −build direction).
- **From S17 (materials).** The print setup refers to materials by `MaterialKey` and reads
  `AmInfo`: `process`, `slicer_type` (the Orca `filament_type` string written to the Orca
  project by AM-06), temperatures, `shrinkage`, `cost`. The build direction (plate +Z,
  `d_local = R⁻¹·Z`) is what S13 maps to material axis 3; the material never stores an
  orientation. Infill and print settings stay in S16.
