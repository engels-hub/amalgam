# S18 — Scripting API, add-ins & CLI

| | |
|---|---|
| **Status** | Refined |
| **Priority** | P1 |
| **Crates** | `amg-script`, `amg-cli`, `amg-ui::script` (proposed, QUESTIONS #56) |
| **Lane** | E |
| **Depends on** | S01 (commands, recompute, `.amg`), S04 (`amg_features::dialog`), S11 (dynamic commands, panels), S12 (export), S05 (measure, properties), S20 (kernel CLI) |
| **Parity** | `03-fusion-parity.md` §Data & other: "Python API & add-ins, text commands" (P1) |

## Summary
Python API (pyo3, ADR-011) mirroring the command set: open/create designs, create sketches
and features, set parameters, run studies, export. Embedded interpreter in the app (script
manager + console) and a pip-installable module for headless use. CLI for batch tasks.
Also the main vehicle for parity regression tests (rebuild reference models by script).
Design reasoning: [`docs/notes/scripting-api.md`](../notes/scripting-api.md).

## Scope
**In:**
- `amg_script::ops`: a headless Rust facade (open, save, recompute report, export, convert,
  properties, JSON reports) shared by the CLI and Python.
- `amg_script::py`: the `amalgam` Python module (pyo3, `abi3-py312`): documents, parameters,
  timeline, raw commands, transactions, bodies, selectors, sketches, generated feature API,
  export, measure and properties.
- A wheel (`maturin`) for headless use from any CPython ≥ 3.12.
- `amg_script::runtime`: CPython embedded in the app on a script thread, with output capture,
  cancel, script manager and add-ins.
- `amg-ui::script`: console panel, script manager dialog, add-in commands (dynamic registration).
- `amg-cli` subcommands: `recompute`, `export`, `convert`, `props`, `run`, `kernel …`,
  `study run`.
- Type stubs and generated API reference.

**Out:**
- A compatibility layer for Fusion's `adsk.*` API (QUESTIONS #57, proposed: no).
- Bundling a Python distribution with the app (QUESTIONS #58, proposed: use the installed one).
- Free-threaded CPython builds (no `abi3` for them yet).
- Publishing the wheel to PyPI (personal-use project; local `maturin build` only).
- Python-defined dialogs for add-ins in the first cut (SCR-16, P3).
- Sandboxing scripts. Scripts and add-ins are trusted local code, as in Fusion.

## Design
Full reasoning, prior art (Fusion API, FreeCAD, Onshape FeatureScript, CadQuery/build123d,
Blender `bpy`, pyo3/maturin) and the CLI reference are in the note. Decisions:

1. **Layers** (note §2). `ops` (plain Rust, no Python) ← `py` (pyo3 conversions only) ←
   `runtime` (embedding). Cargo feature `python` turns on pyo3; without it `amg-script` builds
   with no Python at all, so `amg-cli` needs Python only for `run`.
2. **Python version** (note §3.1). `abi3-py312`: one build for CPython ≥ 3.12. The app and
   `amg-cli` delay-load `python3.dll` and find the interpreter through `AMG_PYTHON_HOME`, PEP 514
   registry keys or `PATH`. Without Python, the app runs and scripting says why it is off.
3. **One script thread** (note §3.2). The interpreter lives on its own thread (its main thread).
   Runs, console lines and add-in callbacks are queued to it. Rust work releases the GIL.
4. **Shadow document + mirror** (note §4). A script run gets its own `Document` that shares
   the UI document's `GeomCache`. Commands apply locally (ids at once, no frame round-trip) and
   are mirrored to the UI document, which checks that created ids match. While a script runs,
   document edits in the UI are disabled ("Script running"); navigation and Stop still work.
5. **Reads never see stale geometry.** Evaluated reads recompute first if the design is ahead.
6. **Undo** (note §5). Headless: one entry per `apply`, `with doc.transaction()` groups. App:
   one entry per script run or add-in command. Needs `History::begin_group`/`end_group`
   (additive `amg-doc`, lane E; see Interface changes).
7. **Selectors** (note §6). Scripts pick entities with method-chain selectors (`faces()
   .normal("+z").largest()`), modelled on CadQuery and Onshape queries. Handles turn into
   `TopoRef`s through `topo_ref_at` when passed to a feature. A handle used after the
   document changed raises `StaleEntityError`.
8. **Generated feature API** (note §7). Built at runtime from `amg_features::dialog`
   registrations: keyword arguments = `FieldKey`s, values converted per `FieldKind`. Numbers for
   `Expr` fields are written as literals with the internal unit (`30` → `"30 mm"`). Kinds without
   a dialog are reachable through `add_raw(kind, dict)`, so every registered kind is scriptable.
9. **Units**: floats are mm, mm², mm³, rad, g, s (internal units). Fusion's API uses cm; we
   do not.
10. **Errors**: one Python exception hierarchy rooted at `amalgam.AmalgamError`, mapped from
    `ScriptError`.
11. **CLI** (note §10): thin clap layer over `ops`. Exit codes 0/1/2/3/130, `--json` with a
    `schema` field, progress on stderr only.
12. **Add-ins** (note §9): folders with `manifest.toml` and `run(ctx)`/`stop(ctx)`, disabled
    until enabled, commands registered as `script.<addin>.<id>` through
    `UiRegistry::register_dynamic`.

## Interfaces
Public API other crates use. `amg-ui::script` uses `runtime` and `host`; `amg-cli` uses `ops`
(and `runtime` for `run`). Python-facing names are in the note. Types from S01 are the code in
`crates/amg-doc` today; types from S04/S11/S12/S05/S20 follow their Interfaces sections.

```rust
// ===== amg_script (always built) ===========================================================
/// Forces feature registrations to be linked (S01 `link()` convention) and checks the
/// registry. Every entry point (`Session::new`, `py` module init, `runtime::start`) calls it.
pub fn link() -> Result<(), &'static amg_doc::RegistryError>;

#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum ScriptError {
    #[error(transparent)] Format(#[from] amg_doc::FormatError),
    #[error(transparent)] Command(#[from] amg_doc::CommandError),
    #[error(transparent)] Recompute(#[from] amg_doc::RecomputeError),
    #[error(transparent)] Io(#[from] amg_io::IoError),
    #[error(transparent)] Export(#[from] amg_features::io::ImportPrepError),
    #[error(transparent)] Inspect(#[from] amg_features::inspect::InspectError),
    #[error("field `{field}`: {msg}")] Field { field: String, msg: String },
    #[error("no {what} named `{name}`")] NotFound { what: &'static str, name: String },
    #[error("entity handle is from revision {held:?}, the document is at {now:?}; select again")]
    StaleEntity { held: amg_doc::Revision, now: amg_doc::Revision },
    #[error("the app document diverged from the script document at batch {seq}")]
    Diverged { seq: u64 },
    #[error("Python 3.12+ not found")] PythonNotFound { searched: Vec<String> },
    #[error("{message}")] Python { message: String, traceback: String },
    #[error("cancelled")] Cancelled,
}
impl amg_core::Diagnostic for ScriptError {}            // hint() for PythonNotFound / StaleEntity

// ===== amg_script::host — the document a script works on ===================================
use amg_doc::{Command, CommandOutcome, CreatedId, Document, Evaluated, Revision};
use std::{path::{Path, PathBuf}, sync::Arc};

/// A script's document: a real `amg_doc::Document`, optionally mirrored to the UI's.
pub struct ScriptDoc { /* doc: Document, path: Option<PathBuf>, mirror: Option<Mirror>, jobs */ }
impl ScriptDoc {
    pub fn new(name: &str, jobs: Arc<amg_jobs::JobSystem>) -> ScriptDoc;
    pub fn open(path: &Path, jobs: Arc<amg_jobs::JobSystem>) -> Result<ScriptDoc, ScriptError>;
    /// In-app run: a shadow of the UI document sharing its cache; batches go to `mirror`.
    pub fn shadow(ui_doc: &Document, jobs: Arc<amg_jobs::JobSystem>, mirror: Mirror) -> ScriptDoc;
    pub fn document(&self) -> &Document;
    pub fn path(&self) -> Option<&Path>;
    pub fn apply(&mut self, label: &str, cmds: Vec<Command>) -> Result<Vec<CommandOutcome>, ScriptError>;
    pub fn undo(&mut self) -> bool;
    pub fn redo(&mut self) -> bool;
    /// Evaluated state for the current revision; recomputes (blocking, off the UI thread) if stale.
    pub fn evaluated(&mut self, cancel: &amg_jobs::CancelToken) -> Result<Arc<Evaluated>, ScriptError>;
    pub fn save(&mut self, path: Option<&Path>, opts: &amg_doc::SaveOptions) -> Result<PathBuf, ScriptError>;
}

/// Sending half: owned by the script's `ScriptDoc`.
pub struct Mirror { /* crossbeam Sender<MirrorMsg>, waker */ }
/// Receiving half: owned by `amg-ui::script`, drained once per frame.
pub struct MirrorQueue { /* Receiver<MirrorMsg> */ }
pub fn mirror_channel(waker: Arc<dyn Fn() + Send + Sync>) -> (Mirror, MirrorQueue);

#[derive(Debug)]
pub enum MirrorMsg {
    BeginGroup { label: String },
    Batch(MirrorBatch),
    Undo { seq: u64 },
    Redo { seq: u64 },
    EndGroup,
}
#[derive(Debug)]
pub struct MirrorBatch {
    pub seq: u64,
    pub label: String,
    pub base: Revision,                      // script-side revision before the batch
    pub commands: Vec<Command>,
    pub created: Vec<Vec<CreatedId>>,        // per command, to check on the UI side
    /// Contains `SetDesignMode { Direct }`: apply only once the UI evaluation reached `base`.
    pub needs_evaluated: bool,
}
impl MirrorQueue {
    pub fn try_recv(&self) -> Option<MirrorMsg>;
    /// Applies one batch to the UI document and checks the ids (`Diverged` on mismatch).
    pub fn apply_batch(doc: &mut Document, b: &MirrorBatch) -> Result<(), ScriptError>;
}

// ===== amg_script::ops — headless facade (CLI + Python) ====================================
pub const REPORT_SCHEMA: u32 = 1;

#[derive(Clone, Debug, serde::Serialize)]
pub struct FeatureHealthRow {
    pub id: amg_doc::FeatureId, pub kind: String, pub name: String,
    pub health: &'static str,                // "ok" | "warning" | "error" | "suppressed" | "rolled_back"
    pub messages: Vec<String>, pub millis: f64,
}
#[derive(Clone, Debug, serde::Serialize)]
pub struct RecomputeReport {
    pub schema: u32, pub revision: u64, pub computed: usize, pub cached: usize,
    pub millis: f64, pub features: Vec<FeatureHealthRow>,
}
impl RecomputeReport { pub fn has_errors(&self) -> bool; }
pub fn recompute(doc: &mut ScriptDoc, cancel: &amg_jobs::CancelToken) -> Result<RecomputeReport, ScriptError>;

#[derive(Clone, Debug)]
pub enum BodyFilter { AllVisible, Names(Vec<String>), Ids(Vec<amg_doc::BodyId>) }
#[derive(Clone, Debug)]
pub struct ExportRequest {
    pub path: PathBuf,
    pub format: Option<amg_io::Format>,      // None: from the extension
    pub bodies: BodyFilter,
    pub refinement: Option<amg_features::io::ExportRefinement>,   // default Medium if solids
    pub options: amg_io::ExportOptions,
}
#[derive(Clone, Debug, serde::Serialize)]
pub struct ExportSummary { pub schema: u32, pub path: PathBuf, pub format: String, pub bodies: usize,
                           pub triangles: u64, pub bytes: u64, pub warnings: Vec<String>, pub millis: f64 }
pub fn export(doc: &mut ScriptDoc, req: &ExportRequest, cancel: &amg_jobs::CancelToken) -> Result<ExportSummary, ScriptError>;

#[derive(Clone, Debug)]
pub struct ConvertRequest { pub src: PathBuf, pub dst: PathBuf, pub src_format: Option<amg_io::Format>,
                            pub dst_format: Option<amg_io::Format>, pub unit: Option<amg_core::units::LengthUnit>,
                            pub import: amg_io::ImportOptions, pub export: amg_io::ExportOptions }
pub fn convert(req: &ConvertRequest, jobs: &amg_jobs::JobSystem, cancel: &amg_jobs::CancelToken) -> Result<ExportSummary, ScriptError>;

#[derive(Clone, Debug, serde::Serialize)]
pub struct BodyProps { pub id: amg_doc::BodyId, pub name: String, pub kind: &'static str,
                       pub volume_mm3: f64, pub area_mm2: f64, pub mass_g: Option<f64>,
                       pub centroid: [f64; 3], pub bbox_min: [f64; 3], pub bbox_max: [f64; 3],
                       pub watertight: Option<bool> }
#[derive(Clone, Debug, serde::Serialize)]
pub struct PropsReport { pub schema: u32, pub bodies: Vec<BodyProps>, pub total: Option<BodyProps> }
/// Mesh bodies via S05 INS-03 `props::mesh_mass_props`; solids via INS-10 once Done
/// (until then solids report `NotFound { what: "solid properties" }`).
pub fn properties(doc: &mut ScriptDoc, bodies: &BodyFilter, cancel: &amg_jobs::CancelToken) -> Result<PropsReport, ScriptError>;

/// S05 request: measure between two script handles (wraps `inspect::measure`).
pub fn measure(doc: &mut ScriptDoc, a: &select::Handle, b: Option<&select::Handle>,
               cancel: &amg_jobs::CancelToken) -> Result<amg_features::inspect::MeasureReport, ScriptError>;

// ===== amg_script::select — selectors (SCR-09) ============================================
pub mod select {
    use amg_doc::{BodyId, EntityRef, Revision, TopoKind};
    #[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
    pub struct Handle { pub body: BodyId, pub kind: TopoKind, pub index: u32, pub revision: Revision }
    #[derive(Clone, Debug, Default)]
    pub struct Selection(pub Vec<Handle>);
    #[derive(Clone, Debug)]
    pub enum Filter {
        Class(amg_doc::GeomClass),
        Normal { dir: amg_core::Vec3, tol_rad: f64 },
        Parallel(amg_core::Vec3), Perpendicular(amg_core::Vec3),
        Radius { r: f64, tol: f64 },
        At { p: amg_core::Point3, tol: f64 },
        CreatedBy { feature: amg_doc::FeatureId, role: Option<String> },
        Adjacent(Handle),
    }
    #[derive(Copy, Clone, Debug)]
    pub enum Order { Largest, Smallest, Nearest(amg_core::Point3), Axis(usize /* 0..3 */), Size }
    pub fn entities(view: &SelectView<'_>, body: BodyId, kind: TopoKind) -> Result<Selection, super::ScriptError>;
    pub fn filter(view: &SelectView<'_>, s: &Selection, f: &Filter) -> Result<Selection, super::ScriptError>;
    pub fn order(view: &SelectView<'_>, s: &Selection, o: Order) -> Result<Selection, super::ScriptError>;
    /// Sub-entities of a selection (faces → edges, edges → vertices, …), deduplicated.
    pub fn children(view: &SelectView<'_>, s: &Selection, kind: TopoKind) -> Result<Selection, super::ScriptError>;
    /// Persistent reference for a feature field (naming note §10 `topo_ref_at`).
    pub fn to_entity_ref(view: &SelectView<'_>, h: &Handle) -> Result<EntityRef, super::ScriptError>;
    /// Evaluated state + kernel at one revision.
    pub struct SelectView<'a> { /* &'a Evaluated, &'a dyn amg_brep::BrepKernel, revision */ _p: std::marker::PhantomData<&'a ()> }
}

// ===== amg_script::features — generated binding core (SCR-02), Python-free ================
pub mod features {
    use amg_features::dialog::{DialogRegistration, FieldValues};
    /// Python-side value before conversion (built by `py`, unit-tested without Python).
    #[derive(Clone, Debug)]
    pub enum ArgValue { Str(String), Float(f64), Int(i64), Bool(bool),
                        Handles(Vec<super::select::Handle>), Refs(Vec<amg_doc::EntityRef>), Param(amg_doc::ParamId) }
    /// Overlays `args` on `base` per the schema (§7 table); `Field` error names the bad key.
    pub fn to_field_values(reg: &DialogRegistration, base: FieldValues,
                           args: &[(String, ArgValue)], view: &super::select::SelectView<'_>)
                           -> Result<FieldValues, super::ScriptError>;
    /// `AddFeature` for a dialog kind (defaults + args → `build`).
    pub fn add_command(reg: &DialogRegistration, doc: &mut super::ScriptDoc, args: &[(String, ArgValue)],
                       context: amg_doc::ComponentId, name: Option<String>) -> Result<amg_doc::Command, super::ScriptError>;
    /// `EditFeature` (`read` → overlay → `build(values, Some(old))`).
    pub fn edit_command(reg: &DialogRegistration, doc: &mut super::ScriptDoc, id: amg_doc::FeatureId,
                        args: &[(String, ArgValue)]) -> Result<amg_doc::Command, super::ScriptError>;
    /// Any registered kind from its serde data (`FeatureRegistration::deserialize` + `validate`).
    pub fn raw_feature(kind: &str, data: serde_json::Value) -> Result<amg_doc::FeatureBox, super::ScriptError>;
    /// Python attribute path of a kind: "solid.extrude" → ["solid", "extrude"]; '-' → '_'.
    pub fn attr_path(kind: &str) -> Vec<String>;
}

// ===== amg_script::sketch — SCR-10 helpers over lane B's SketchEdit =======================
pub mod sketch {
    use amg_features::sketch::{PlaneInput, SketchEdit};
    /// Rectangle, slot, polygon and arc helpers expand into one `SketchEdit::Batch`.
    pub fn rectangle(a: amg_core::Point2, b: amg_core::Point2, constrained: bool) -> SketchEdit;
    pub fn center_rectangle(c: amg_core::Point2, w: f64, h: f64) -> SketchEdit;
    pub fn arc_3pt(a: amg_core::Point2, m: amg_core::Point2, b: amg_core::Point2) -> SketchEdit;
    pub fn polygon(c: amg_core::Point2, r: f64, sides: u32, inscribed: bool) -> SketchEdit;
    /// Applies the edits to a clone of the sketch feature → one `EditFeature`.
    pub fn edit_command(doc: &super::ScriptDoc, sketch: amg_doc::FeatureId, edits: Vec<SketchEdit>)
                        -> Result<amg_doc::Command, super::ScriptError>;
    pub fn new_command(plane: PlaneInput, context: amg_doc::ComponentId, name: Option<String>) -> amg_doc::Command;
}

// ===== amg_script::runtime — embedded CPython (feature "python") ==========================
#[cfg(feature = "python")]
pub mod runtime {
    use std::path::{Path, PathBuf};
    #[derive(Clone, Debug)]
    pub struct PythonInstall { pub home: PathBuf, pub version: (u8, u8, u8), pub source: &'static str /* "env" | "registry" | "path" | "prefs" */ }
    /// AMG_PYTHON_HOME → PEP 514 (HKCU, HKLM) → PATH; `prefs_home` first when set.
    pub fn discover(prefs_home: Option<&Path>) -> Result<PythonInstall, super::ScriptError>;

    #[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)] pub struct RunId(pub u64);
    #[derive(Clone, Debug)]
    pub enum RunSource { File { path: PathBuf, args: Vec<String> }, Console(String), AddinCommand { id: String } }
    #[derive(Clone, Debug)]
    pub struct ScriptFailure { pub message: String, pub traceback: String, pub location: Option<(PathBuf, u32)> }
    #[derive(Clone, Debug)]
    pub struct AddinCommand { pub id: String /* "script.<addin>.<cmd>" */, pub label: String, pub tooltip: String,
                              pub icon_svg: Option<Vec<u8>>, pub placement: Option<String> /* "SOLID/CREATE"; None: search only */,
                              pub shortcut: Option<String> }
    #[derive(Clone, Debug)]
    pub struct AddinInfo { pub name: String, pub version: String, pub description: String, pub dir: PathBuf,
                           pub running: bool, pub run_on_startup: bool, pub error: Option<String> }
    #[derive(Debug)]
    pub enum ScriptEvent {
        Output { run: RunId, text: String, stderr: bool },
        Started { run: RunId, label: String },
        Finished { run: RunId, result: Result<(), ScriptFailure>, millis: f64 },
        RegisterCommand(AddinCommand),
        UnregisterCommand(String),
        AddinsChanged,
    }
    /// What a run needs from the app: a fresh shadow document per run.
    pub trait AppDocs: Send + Sync {
        /// Called on the UI thread when a run starts.
        fn begin_run(&self, label: &str) -> super::host::ScriptDoc;
    }
    pub struct ScriptRuntime { /* script thread, queues, cancel tokens */ }
    impl ScriptRuntime {
        /// Loads python3.dll from `install`, starts the interpreter on its own thread.
        pub fn start(install: &PythonInstall, addin_dirs: Vec<PathBuf>) -> Result<ScriptRuntime, super::ScriptError>;
        pub fn run(&self, src: RunSource, doc: super::host::ScriptDoc, selection: Vec<amg_doc::EntityRef>) -> RunId;
        pub fn cancel(&self, run: RunId);
        pub fn busy(&self) -> Option<RunId>;
        pub fn complete(&self, line: &str, cursor: usize) -> Vec<String>;      // console tab completion, ≤ 10 ms
        pub fn drain_events(&self) -> Vec<ScriptEvent>;
        pub fn addins(&self) -> Vec<AddinInfo>;
        pub fn set_addin_enabled(&self, name: &str, on: bool);
        pub fn shutdown(self, timeout: std::time::Duration);                    // stop() add-ins, then finalize
    }
}

// ===== amg_script::py (feature "python") — the `amalgam._native` module ====================
#[cfg(feature = "python")]
pub mod py {
    /// Module init used by the wheel (`#[pymodule]`) and by `runtime` (`append_to_inittab!`).
    pub fn init_module(m: &pyo3::Bound<'_, pyo3::types::PyModule>) -> pyo3::PyResult<()>;
    pub const API_LEVEL: u32 = 1;
}
```

`amg-cli` exposes no library API. Its subcommands are listed in the note §10.

## Work packages
All lane E. SCR-01..06 keep their original meaning; SCR-05 lost `study run` (now SCR-12).

| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| SCR-07 | `amg_script::ops` + `host::ScriptDoc` (no mirror): `link`, `ScriptError`, `open`/`new`/`save`, `evaluated()` auto-recompute, `RecomputeReport`, `export` (via `scene_from_document` + `amg_io::export`), `convert`, `properties` (mesh bodies), JSON reports with `schema: 1` | DOC-14, IO-14, IO-02, IO-11, INS-03 | P1 | Todo | 1. Integration tests on 3 fixture designs (mesh-only, mixed with a test feature, one with a failing feature): `recompute` reports the right computed/cached counts and the failing feature as `error` with its message; `has_errors()` is true only for the third. 2. `export` to STL and 3MF re-imports with the same triangle count and volume within 1e-9 relative; `BodyFilter::Names` with an unknown name gives `NotFound`. 3. `convert` STL→3MF→STL keeps triangle count and bbox exactly. 4. `properties` on a 10 mm mesh cube gives volume 1000 ± 1e-9 mm³ and area 600. 5. Report JSON validated against checked-in `tests/schema/*.json` snapshots. 6. Cancelling a slow (sleeping) test feature returns `Cancelled` within 100 ms. |
| SCR-05 | CLI: `recompute`, `export`, `convert`, `props` (S05 request) with `--json`, exit codes 0/1/2/3/130, progress on stderr | SCR-07, FND-07 | P1 | Todo | 1. `assert_cmd` tests per subcommand on the SCR-07 fixtures: stdout JSON parses and matches the schema snapshot; nothing but JSON on stdout with `--json`. 2. Exit code 3 for the failing-feature design, 1 for a missing file and a corrupt `.amg` (message names the `FormatError`), 2 for bad usage. 3. `recompute --save` writes a file that loads with 0 feature computes (DOC-14 cache). 4. Ctrl+C (a test sends `CTRL_BREAK` to a child running a slow export) exits 130 within 500 ms and leaves no partial output file (atomic sink). 5. `amg-cli convert` of a 1 GB / 20 M-triangle binary STL to 3MF finishes in < 15 s on the reference machine (import budget 5 s + write), measured in the log. |
| SCR-08 | CLI `kernel replay`, `kernel promote`, `kernel harness` (S20 request) over `amg_brep::capture::{replay, promote}` and the KH-02 harness entry point; harness worker processes are `amg-cli kernel harness-worker` (hidden) | KH-01, KH-02, SCR-05 | P1 | Todo | 1. `kernel replay` on the KH-01 test dumps prints one line per dump and `--json` matches KH-01's `ReplayReport` serialisation; exit 0 if all `Reproduced`/`NowSucceeds`, 1 otherwise. 2. `kernel promote` produces the same corpus case as KH-01's `promote` test. 3. `kernel harness` on the KH-02 smoke subset writes the same report as the `kernel-harness` example (diff of the JSON ignoring timings is empty), and an injected access violation counts as `Crash` without ending the run. 4. S20's examples are kept or deleted per S20's choice; the spec log says which. |
| SCR-01 | pyo3 module: `python` feature (`abi3-py312`), `amalgam._native` + pure-Python `amalgam` package, maturin `pyproject.toml`, wheel with OCCT/vcpkg DLLs (delvewheel); `Document` (open/new/save/undo/redo/transaction), `params` (get/set/add/rename/delete, values with units), timeline (features list, suppress, rename, delete, move, marker), raw `apply(dict)` (externally tagged `Command` JSON), `bodies` (name, kind, visible, volume/area/bbox), `export`/`properties` via `ops`, exception hierarchy, GIL released in Rust work, Ctrl+C polling | SCR-07, DOC-05 | P1 | Todo | 1. In a clean venv on Windows, `pip install <wheel>` then `python -c "import amalgam; print(amalgam.__version__)"` works with no other env vars; the same with CPython 3.12 and the newest 3.x (CI). 2. pytest suite (`tests/py/test_doc.py`, ≥ 40 tests) covers every class/method listed, including: set a parameter → recompute → body volume changes as expected; `with doc.transaction()` makes one undo entry; an invalid expression raises `CommandError` with the `ParamError` text. 3. `doc.apply({"AddParam": {...}})` round-trips: the same JSON from `Command` serde. 4. Ctrl+C during a slow recompute raises `KeyboardInterrupt` within 200 ms and the document stays usable. 5. `import amalgam` cold < 500 ms on the reference machine (measured with `-X importtime`). 6. Wheel licence check: `THIRD_PARTY.md` lists pyo3, maturin, delvewheel. |
| SCR-09 | Selectors: `amg_script::select` (`entities`, `filter`, `order`, `children`, `to_entity_ref`) and the Python `Selection`/`Face`/`Edge`/`Vertex` handles, `StaleEntityError`; mesh face groups; `feature.resolutions` and `doc.accept_matches(feature)` (naming note §7) | SCR-01, DOC-06, BREP-02, BREP-09 | P1 | Todo | 1. On a box with a through-hole and filleted top edges: `faces().normal("+z").largest()` is the top face; `faces().kind("cylinder").radius(2.5)` has exactly the hole face; `edges().created_by(fillet)` count equals the fillet's face count; `nearest(p)` is checked against brute force on 1 000 random points. 2. A handle turned into a feature input via `to_entity_ref` survives the DOC-11 corpus edits used by SOL-05 (same entity resolved, `MatchQuality::Exact`). 3. Using a handle after an edit raises `StaleEntityError`. 4. Ordering ties are broken by index (determinism test over 2 runs and 1 vs 16 threads). 5. Selector on a 10 k-face body < 20 ms (criterion). |
| SCR-10 | Sketch scripting: `doc.sketches.add(plane)`, `with sketch.edit() as e:` (`point`, `line`, `circle`, `arc_3pt`, `rectangle`, `center_rectangle`, `polygon`, `constraint(kind, ...)`, `dimension(entity, expr)`, `construction`), `sketch.profiles`, `sketch.dimensions`; one `EditFeature` per `edit()` block via `SketchFeature::apply` | SCR-01, SK-19 | P1 | Todo | 1. A script draws a fully constrained 40×20 rectangle with a centred Ø8 circle dimensioned by the parameter `d`; `sketch.profiles` has 2 regions with areas 800 − 16π and 16π (1e-9 relative); changing `d` re-solves. 2. A failing edit (conflicting constraint) raises with the solver diagnosis and leaves the sketch unchanged (undo stack unchanged). 3. One `edit()` block is one undo entry. 4. 1 000-line sketch built in one block in < 200 ms (excluding solve). |
| SCR-02 | Generated feature API: `amg_script::features` (`to_field_values`, `add_command`, `edit_command`, `raw_feature`, `attr_path`), `doc.features.add(kind, **kw)`, generated namespaces (`doc.features.solid.extrude(...)`), `feature.edit(**kw)`, `feature.values`, `add_raw(kind, dict)` for kinds without a dialog | SOL-01, SCR-09, SCR-10 | P1 | Todo | 1. A test enumerates `amg_doc::registry().kinds()`: every kind is creatable either through `add` (dialog) or `add_raw` (a serde fixture per kind), and none needs hand-written glue. 2. For every `DialogRegistration`: `add(kind)` with defaults on a fixture input gives the same `FeatureBox` JSON as `build(defaults(..))`, and `feature.edit()` with no args leaves the feature byte-identical. 3. Every `FieldKey` is a valid non-keyword Python identifier. 4. Wrong types, unknown keys, unknown choice ids and out-of-range ints raise `FieldError` naming the key and the valid values. 5. `distance=30` and `distance="30 mm"` give identical features. 6. Python rebuild of the `sol01_*` golden extrudes matches their golden volumes. |
| SCR-11 | Python parity regression: `amg-cli run <script.py> [-- args]` (embedded interpreter, `python` feature), `tests/scripts/` rebuilding the S04 golden designs and the owner's Fusion references (SOL-29) in Python, CI `golden` job | SCR-02, SCR-05, SOL-29 | P1 | Todo | 1. Every SOL-29 Rust parity script has a Python twin; both give the same `design.json` hash and volumes within 1e-12 relative. 2. `amg-cli run` exits 1 with the traceback on stderr for a raising script and 130 on Ctrl+C. 3. The CI `golden` job runs the scripts (skipping owner data when `AMG_TESTDATA_DIR` is unset) in < 3 min. 4. The 50-feature bench part built from Python recomputes after a first-sketch edit within the < 500 ms vision budget (same as SOL-29; script overhead excluded and reported). |
| SCR-03 | Embedded interpreter + console: `runtime` (discovery, delay-load, `embed` FFI module, script thread, `RunSource`, output capture, async-exception cancel, completion), `host::{Mirror, MirrorQueue}` + undo groups (`History::begin_group`/`end_group`, journal records), `amg-ui::script` console panel (`PanelDef`, history, multi-line input, tab completion, Stop), "Script running" enablement | UI-01, UI-02, SCR-01, QUESTIONS #56 | P1 | Todo | 1. Kittest: typing `doc.params.add("w", "10 mm")` in the console adds the parameter and Ctrl+Z removes it in one step. 2. With no Python found, the app starts normally and the console shows the not-found message with the searched locations (test overrides discovery). 3. A script with an infinite pure-Python loop stops within 200 ms of Stop; the interpreter is still usable after. 4. Mirror property test: 200 random command scripts through a shadow `ScriptDoc` + `MirrorQueue` into a second `Document` leave both `design.json` byte-identical; a forced id mismatch gives `Diverged` and stops the run. 5. During a 2 s script the UI frame time stays < 16 ms (tracing spans) and document commands show "Script running". 6. A script of 300 `AddParam` commands finishes in < 300 ms in the app (no per-command frame wait). 7. Undo after a run that created 5 features removes all 5 in one step; a crash mid-run recovers the applied part (DOC-14 recovery test). |
| SCR-13 | Script manager dialog (`amg-ui::script`): scripts and add-ins folders, list with run/stop/edit (opens the file in the system editor)/open folder, new-script template, recent runs with status and duration, Python path in prefs, `amalgam.debug.listen()` (debugpy if installed) | SCR-03 | P2 | Todo | 1. Kittest: a script dropped into the scripts folder appears without restart, runs from the dialog, and its failure shows the line number. 2. New-script template runs unchanged and prints the design name. 3. Re-running after editing a helper module in the script's folder uses the new code. 4. `debug.listen()` without debugpy raises a clear `ImportError` message; with debugpy, a VS Code attach hits a breakpoint (manual check, recorded in the log). |
| SCR-04 | Add-ins: discovery (`manifest.toml`), enable/disable, run-on-startup (prefs), `run(ctx)`/`stop(ctx)`, `ctx.add_command` → `UiRegistry::register_dynamic` (`script.<addin>.<id>`, leaked ids reused), `CmdAction::Dyn` → `RunSource::AddinCommand` with the selection, automatic unregister on stop | SCR-03, SCR-13, UI-02 | P2 | Todo | 1. Kittest: an add-in registering `script.demo.hello` in `SOLID/CREATE` shows a toolbar button; clicking it with a face selected runs `on_execute` with that face handle; the run is one undo entry. 2. Disabling removes the button even when `stop` raises. 3. A new add-in folder is listed but not started until enabled; run-on-startup survives a restart. 4. An add-in whose manifest asks for a higher `api_level` is listed with an error and not started. 5. 100 enable/disable cycles leak ≤ one id string per distinct id (counter). |
| SCR-06 | Type stubs (`.pyi`) + API reference: generated `features/*.pyi` from the dialog registrations (fields, types, choices, defaults, help), hand-written stubs for core classes, Markdown reference `docs/api/python.md` generated by a test-guarded generator | SCR-02 | P2 | Todo | 1. `mypy --strict` passes on `tests/scripts/` against the stubs. 2. A test fails when a `#[pymethods]` name (from `dir()` of each class) is missing from the stubs, or a registered kind has no generated stub. 3. The generated reference is up to date (the test regenerates and diffs). |
| SCR-14 | numpy interop (`numpy` crate, feature `numpy`): `body.mesh()` → `(vertices: float64[N,3], triangles: uint32[M,3])`, `doc.add_mesh(v, t)` (baked mesh body), study result fields later; lazy import so `import amalgam` works without numpy | SCR-01 | P2 | Todo | 1. Round trip mesh → numpy → `add_mesh` keeps vertices bit-exact. 2. A 1 M-triangle mesh converts in < 50 ms (zero-copy or one memcpy, measured). 3. Without numpy installed, only these calls raise `ImportError`. 4. `THIRD_PARTY.md` lists `numpy` (BSD-2-Clause). |
| SCR-12 | Studies from script and CLI: `doc.studies`, `study.run()` (job with progress), results summary; `amg-cli study run` | FEM-01, SCR-07, SCR-01 | P2 | Blocked (S13 not refined; the study API is not defined yet) | Refine when S13 is `Refined`: running the S13 cantilever study from Python and from the CLI gives the same max displacement as the in-app run (1e-9 relative). |
| SCR-15 | "Copy as script": turn undo entries (`UndoEntry::commands`) of the current session into a Python script using the generated API (dialog kinds) and `apply` (other commands) | SCR-02 | P3 | Todo | Replaying the generated script on the starting design gives a byte-identical `design.json` for the SOL-29 bench part built interactively in a kittest. |
| SCR-16 | Python-defined dialogs for add-in commands (Fusion `CommandInputs`): runtime-built schema rendered by UI-05 | SCR-04, UI-05 | P3 | Todo | Needs the S11 change in Interface changes (owned schema). An add-in dialog with an expression field, a face selection and a choice runs `on_execute` with typed values and a live preview. |

## Testing
- **Rust unit/integration**: `ops`, `host`, `select`, `features` and `sketch` are Python-free and
  tested with plain `cargo nextest` on test features and fixtures.
- **Python**: pytest files in `crates/amg-script/tests/py/`, run by `tests/python.rs` with the
  module registered in-process (`append_to_inittab!`). Skips without Python ≥ 3.12; fails if
  `AMG_REQUIRE_PYTHON=1` (set in CI, with `actions/setup-python` 3.12 and the newest 3.x).
- **CLI**: `assert_cmd` tests per subcommand; JSON snapshots in `crates/amg-cli/tests/schema/`.
- **UI**: kittest via `amg_ui::testing::AppHarness` (S11) for the console, script manager and
  add-ins.
- **Mirror invariant**: proptest of random command scripts through shadow + mirror (SCR-03).
- **Golden**: SCR-11 rebuilds S04 golden designs from Python; geometry is compared with the
  existing `tests/golden/solid/*.toml` values, not new goldens.
- **Wheel**: CI job builds the wheel with maturin, installs it in a fresh venv and runs
  `tests/py/test_smoke.py`.

## Performance targets
| Target | Budget | WP |
|---|---|---|
| `import amalgam` (wheel, cold) | < 500 ms | SCR-01 |
| Python → Rust call overhead for a property read | < 20 µs median | SCR-01 |
| 300 commands from an in-app script | < 300 ms total | SCR-03 |
| UI frame during a running script | < 16 ms (vision: UI never blocks) | SCR-03 |
| Console tab completion | < 10 ms | SCR-03 |
| Selector on a 10 k-face body | < 20 ms | SCR-09 |
| 1 M-triangle mesh to numpy | < 50 ms | SCR-14 |
| `amg-cli convert` 1 GB binary STL → 3MF | < 15 s (import < 5 s per vision) | SCR-05 |
| Recompute of the 50-feature part from a script edit | < 500 ms (vision) | SCR-11 |

## Open questions
1. (from S05, 2026-10-20) Python/CLI wrappers `measure(a, b)`, `physical_properties(body)`,
   `section_stats(body, plane)`; suggested CLI `amg-cli props <file>`.
   **Resolved 2026-11-08:** `ops::measure` and `ops::properties` (SCR-07), Python
   `am.measure(a, b)` / `body.physical_properties()` (SCR-01, SCR-09), `amg-cli props` (SCR-05).
   `section_stats` wraps S05 `section_mesh` and lands with SCR-09 (it needs a plane handle).
2. (from S11, 2026-10-20) SCR-03 and SCR-04 use `UiRegistry::register_dynamic` (`script.*` ids)
   and `PanelDef`. **Resolved 2026-11-08:** adopted as is. Ids are `script.<addin>.<cmd>`; the
   `&'static str` in `CmdId` is met by leaking each distinct id once (note §9). The console is a
   `PanelDef` in `amg-ui::script` (placement needs QUESTIONS #56).
3. (from S20, 2026-10-20) Thin `amg-cli` subcommands `kernel replay|promote|harness`.
   **Resolved 2026-11-08:** accepted as SCR-08 (P1, Phase 2). Harness workers are the same
   binary (`kernel harness-worker`, hidden).
4. Where do the console panel and add-in commands live, given `amg-ui` may not depend on
   `amg-script` today? → QUESTIONS #56 (proposed: new edge `amg-ui → amg-script`, module
   `amg-ui::script` owned by lane E).
5. Compatibility shim for Fusion's `adsk.*` API? → QUESTIONS #57 (proposed: no).
6. Bundle a Python with the app, or use the installed one? → QUESTIONS #58 (proposed: installed,
   delay-loaded).

## Refinement log

### 2026-11-08 - S18-refine (lane E refinement)
- **Read:** AGENTS.md, vision, architecture, ADRs (ADR-011, ADR-012, ADR-014), roadmap, QUESTIONS;
  S00 (`amg-cli` skeleton, layering test, Python row), S01 (commands, history, recompute,
  `.amg`, linker `link()` convention), S04 (`amg_features::dialog`, SOL-01, SOL-29), S05
  (inspect API), S11 (§A registry, `register_dynamic`, `PanelDef`, §B schema), S12 (`amg_io`
  import/export, `amg-features::io`), S13 (study model, Draft), S20 (capture/replay, harness);
  naming note §7 and §10, kernel-robustness note §5–6.
- **Code checked:** `crates/amg-script` and `crates/amg-brep` are stubs; `crates/amg-cli` has
  `info` only (deps core/jobs/clap/tracing); `amg-doc` has the final `Command`, `Document`,
  `History`/`Transaction`, `registry()`, `SaveOptions`; `amg-features::{sketch, mesh}` register
  with `register_feature!` and export `link()`; `SketchFeature::apply(SketchEdit)` exists.
  Workspace edition 2024 (so `env::set_var` is `unsafe`), `panic = "unwind"` already set for
  pyo3.
- **Prior art** (note §1): Fusion API (scripts vs add-ins, manifest, run/stop, run on startup,
  command definitions, text commands, cm units, blocking UI thread), FreeCAD (console, macro
  recording), Onshape FeatureScript queries, CadQuery/build123d selectors, Blender `bpy`
  (generated API, add-on register/unregister), pyo3 `abi3` + maturin. Web search for the current
  pyo3 release and Fusion's bundled Python version was inconclusive; SCR-01 pins the newest pyo3
  at its start and only relies on `abi3-py312`.
- **Decided:** layers `ops`/`py`/`runtime` with a `python` cargo feature; `abi3-py312`;
  installed CPython found by env/PEP 514/PATH and delay-loaded; one script thread; shadow
  document sharing the `GeomCache` plus a mirror to the UI document instead of per-command UI
  round-trips; UI edits disabled while a script runs; one undo entry per run; selectors →
  `topo_ref_at`; feature API generated from dialog registrations, with `add_raw` for kinds
  without a dialog; mm units; CLI exit codes and JSON schema.
- **WPs:** SCR-01..06 keep their meaning. SCR-01 now depends on SCR-07 (the shared facade).
  SCR-02 gains SCR-09/SCR-10 as deps (selection and sketch values). SCR-03 depends on SCR-01
  and QUESTIONS #56. SCR-05 drops `study run` (moved to SCR-12, Blocked until S13 is refined) and
  gains `props`. New: SCR-07 (ops facade), SCR-08 (kernel CLI, S20), SCR-09 (selectors),
  SCR-10 (sketch API), SCR-11 (Python parity + `amg-cli run`), SCR-12 (studies), SCR-13
  (script manager), SCR-14 (numpy), SCR-15 (copy as script), SCR-16 (add-in dialogs).
- **Roadmap:** Phase 2 E gains SCR-05, SCR-07, SCR-08 (their deps are Phase 1/2). Phase 3 E:
  SCR-01..03, SCR-09..11. Phase 4 E: SCR-12. Phase 5 E: SCR-04, SCR-06, SCR-13, SCR-14.
  Phase 6: SCR-15, SCR-16. The lanes table is not edited (QUESTIONS #56).
- **New third-party (for `THIRD_PARTY.md` when the WPs land):** pyo3 (MIT/Apache-2.0, already
  planned), `numpy` crate (BSD-2-Clause, SCR-14), maturin (MIT/Apache-2.0, build tool),
  delvewheel (MIT, build tool), `assert_cmd` (MIT/Apache-2.0, dev), debugpy (MIT, optional,
  user-installed, never shipped).

## Interface changes
Requests from the 2026-11-08 refinement (additive; owners must confirm):
- **S01 (lane E, same lane):** `History::begin_group(&mut self, label: &str) -> GroupToken` and
  `end_group(&mut self, GroupToken)` (and the same on `Document`): commands applied while a group
  is open merge into one `UndoEntry`; the crash journal records `GroupBegin`/`GroupEnd`. Lands
  with SCR-03; recorded in S01 Interface changes by SCR-03.
- **S01 (lane E, same lane):** `Document::recompute_blocking_with(&mut self, jobs: &JobSystem,
  cancel: &CancelToken) -> Result<(), RecomputeError>`. Today `recompute_blocking` waits on a
  private job handle and cannot be cancelled from outside, which SCR-07's Done-when 6 and Ctrl+C
  need. Lands with SCR-07.
- **S00 / architecture (ADR-level, QUESTIONS #56):** allow the edge `amg-ui → amg-script` in the
  layering allow-list; add `amg-ui::script` to lane E in the roadmap lanes table.
- **S11 (lane D):** `amg-app` links `python3.dll` delay-loaded when its `python` feature is on
  (`/DELAYLOAD:python3.dll` + `delayimp.lib` in `build.rs`), and starts `ScriptRuntime` lazily on
  first console/script use. For SCR-16 (P3): an owned `DialogSchema` variant (`Arc<DialogSchema>`
  instead of `fn() -> &'static DialogSchema`) so UI-05 can render runtime-built dialogs.
- **S04 (lane A):** `FieldKey`s are snake_case Python identifiers (checked by SCR-02's test).
