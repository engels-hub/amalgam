# Scripting API, add-ins and CLI — design note (S18)

Written during the S18 refinement (2026-11-08). Linked from `docs/specs/S18-scripting-cli.md`.
It covers the questions that shape every SCR work package: how Python is embedded, how a
script reaches a document that the UI thread owns, how the feature API is generated, how
scripts pick faces and edges, and how the CLI shares the same code.

## 1. Prior art

| Source | What it does | What we take | What we reject |
|---|---|---|---|
| **Fusion API** (Autodesk docs: "Creating Scripts and Add-Ins", Python add-in template, `Application.executeTextCommand`) | Bundled CPython. Scripts (`run(context)`) and add-ins (`run`/`stop`, "Run on Startup") in folders with a `.manifest`. Object model `app.activeProduct.rootComponent.sketches.add(plane)`, `extrudeFeatures.createInput(profile, op)` + `setDistanceExtent` + `add`. Command definitions with toolbar controls and `CommandInputs` dialogs. Text commands window. VS Code debugging. Internal unit is the **centimetre**. Scripts run on the UI thread and block the app. | Scripts vs add-ins split, `run`/`stop` entry points, run-on-startup flag, folder layout with a manifest, command definitions placed in toolbar panels, text console, debugger hookup | The `createInput` + setter object dance (we take keyword arguments generated from the dialog schema). Centimetres (we use mm, the internal unit, so values never need a factor). A blocking UI thread. An `adsk.*` compatibility layer (QUESTIONS #57) |
| **FreeCAD** (`FreeCAD`/`FreeCADGui` modules, macro recorder, Python console) | Every GUI action echoes the Python line it ran; macros are recorded Python. Headless via `FreeCADCmd`. | The console and "copy as script" from recorded commands (SCR-15). Headless and GUI share one module. | Echoing every UI action into the console (noise); App/Gui module split by object type |
| **Onshape FeatureScript** (`qCreatedBy`, `qGeometry`, `qParallelPlanes`, `qLargest`, `qClosestTo`) | Entity references are queries evaluated at regeneration. | Query vocabulary for script selectors (§6): created-by + role, geometry class, direction, largest, closest | Storing queries in features: S04 features store resolved `TopoRef`s (naming note, Onshape row) |
| **CadQuery / build123d** (Apache-2.0) | Fluent selectors on faces/edges: `faces(">Z")` (top face), edges parallel to Z, `%PLANE`, sorting by area/length/position. | Method-chain selectors over a body's faces/edges, sorted/filtered by axis, type, position, size | String selector mini-language as the main API (kept out; plain methods are discoverable and typed). Their kernel-level model (no timeline) |
| **Blender `bpy`** | Embedded CPython on the main thread, operators exposed 1:1 from the C registry, generated API docs, add-ons with `register()`/`unregister()`. | Generating the API from the registry (our `DialogRegistration`); generated reference docs; `register`/`unregister` idea | Running scripts on the UI thread |
| **pyo3 + maturin** (MIT/Apache-2.0) | Rust ↔ CPython bindings; `abi3` stable-ABI wheels; maturin builds mixed Rust/Python wheels. | Both (ADR-011). `abi3-py312`: one wheel and one embedded build for every CPython ≥ 3.12 | Free-threaded CPython (3.13t/3.14t): no `abi3` there yet; revisit when an `abi3t` exists |

## 2. Shape of the solution

```
 amg-cli ─┐                          ┌─ amg-ui::script (console, script manager, add-in commands)
          ├── amg_script::ops ───────┤        │
 Python ──┘   (headless Rust facade) │        ▼
 (wheel)        ▲                    └─ amg_script::runtime (feature "python": embedded CPython,
                │                             script thread, cancel, output, add-ins)
          amg_script::py (pyo3 classes, feature "python")
                │
          amg_script::host::ScriptDoc  (a real amg_doc::Document, optional Mirror to the UI)
```

- **`amg_script::ops`** is plain Rust, built without Python: open/save, recompute report,
  export, convert, properties, JSON reports. The CLI calls it directly, so `amg-cli` never
  needs Python except for `amg-cli run`.
- **`amg_script::py`** is the pyo3 layer. It only converts values and calls `ops` and
  `host`. No geometry logic lives in it.
- **`amg_script::runtime`** embeds CPython in the app: it starts the interpreter on its own
  thread, runs files and console lines, captures output, interrupts, and hosts add-ins.
- **`amg-ui::script`** is the egui side (console panel, script manager, dynamic commands).
  It needs a new layering edge `amg-ui → amg-script` (QUESTIONS #56).

## 3. Embedding CPython

### 3.1 Which Python
- **Stable ABI**: pyo3 with `abi3-py312`. One build runs on any CPython ≥ 3.12, the version in
  `dev-setup.md`. CI tests 3.12 and the newest release.
- **Not bundled (default)**: the app uses an installed CPython (QUESTIONS #58). Discovery order:
  1. `AMG_PYTHON_HOME` (a directory holding `python3.dll`);
  2. PEP 514 registry keys `HKCU` then `HKLM\Software\Python\PythonCore\<ver>\InstallPath`,
     highest version ≥ 3.12;
  3. `python.exe` on `PATH` (`sys.base_prefix`).
  Prefs (UI-11) can override the path.
- **Delay-loaded**: `amg-app` and `amg-cli` link `python3.dll` with `/DELAYLOAD:python3.dll`
  (+ `delayimp.lib`). The app starts without Python. The first scripting call loads
  `<home>\python3.dll` with `LoadLibraryExW(.., LOAD_WITH_ALTERED_SEARCH_PATH)`; the delay-load
  stub then binds to the loaded module. Without Python the console shows
  "Python 3.12+ not found" with the discovery hint, and the rest of the app works.
- **Home**: the interpreter is started with `Py_InitializeFromConfig` and `config.home = home`
  (pyo3 has no safe wrapper). This and the `LoadLibraryExW` call are the only `unsafe` in
  `amg-script`: one documented FFI module `runtime::embed` with `#[allow(unsafe_code)]` and a
  `SAFETY` note per call (AGENTS.md conventions). `std::env::set_var("PYTHONHOME")` is not used
  (it is `unsafe` in edition 2024 and racy with other threads).
- **Packages**: scripts see the chosen interpreter's `site-packages` (numpy etc. are the user's
  choice). The `amalgam` module itself is built into the app binary and registered with
  `PyImport_AppendInittab`-equivalent (`pyo3::append_to_inittab!`) before start, so the app
  never imports a wheel from `site-packages` and cannot pick up a mismatched version.

### 3.2 One script thread
- The interpreter is created on a dedicated **script thread** and lives until exit. That thread
  is Python's main thread, so signal-style interrupts and `threading.main_thread()` behave.
- Script runs, console lines and add-in callbacks are queued to that thread and run one at a
  time. Python code may start its own threads; API calls from them are allowed (the GIL
  serialises them).
- Every blocking Rust call (recompute, export, import, measure) releases the GIL
  (`Python::allow_threads` / `detach` in newer pyo3) and polls cancellation.

### 3.3 Cancel
- **App**: the Stop button sets the run's `CancelToken` and calls
  `PyThreadState_SetAsyncExc(script_thread_id, amalgam.Cancelled)` (part of `runtime::embed`).
  Pure-Python loops stop at the next bytecode; Rust calls see the token through `JobCtx` and
  return `Cancelled`. A second Stop within 2 s after a run ignored the first one shows
  "script not responding" with the option to keep waiting (no thread kill: killing a thread
  holding the GIL corrupts the interpreter).
- **Wheel / `amg-cli run`**: Ctrl+C. While the GIL is released, the waiting thread wakes every
  50 ms, briefly takes the GIL and calls `Python::check_signals()`; a `KeyboardInterrupt`
  cancels the running job and is re-raised.

### 3.4 Output and errors
- `sys.stdout`/`sys.stderr` are replaced by writer objects that forward text to the run's
  output channel (console) or the real stdout (headless).
- An uncaught exception ends the run with `ScriptFailure { message, traceback, location }`.
  `location` is the innermost frame inside the script file, so the console can show
  "line 12 in bracket.py".
- Re-running a script from its folder first removes `sys.modules` entries whose `__file__` is
  under that folder, so edits to helper modules take effect (a known Fusion annoyance).

## 4. How a script reaches the document

The UI thread owns `Document` (architecture §Threading). Sending each command to the UI and
waiting for an answer costs a frame per call, and eframe frames are vsync-limited (~16 ms).
A 300-command script would take 5 s. Instead:

**Shadow document + mirror.** When a run starts, the script thread builds its own
`Document::with_cache(snapshot_design, ui_doc.cache().clone())`:
- The design is an `Arc` structural-sharing copy, so it is cheap.
- The `GeomCache` is shared, so a feature computed by either side is a cache hit for the other.
- The script applies commands to its own document and gets `CreatedId`s at once. It
  recomputes with `recompute_blocking`, which is allowed off the UI thread.
- Every successful `apply` is also sent to the UI as a `MirrorBatch { seq, label, commands,
  created }` (fire-and-forget). The UI applies forwarded batches at the end of the next frame,
  inside an open undo group (§5), and checks that its `created` ids equal the script's. A
  mismatch stops the run with an error ("document diverged"). It means a non-deterministic
  command, which is a bug.
- `Command::apply` is deterministic given the same design (S01: ids come from design
  counters), so the two documents stay equal. One exception: `SetDesignMode { Direct }` bakes
  *evaluated* bodies. A batch containing it is applied by the UI only once its own evaluation
  has caught up to the batch's base revision. The UI's queue waits for that; the script does not.
- **While a script runs, the user cannot edit the document**: document commands are disabled
  with the reason "Script running" (viewport navigation, selection and the Stop button still
  work). This matches Fusion, where a script blocks the app, without freezing the window.
- The UI wakes on a mirror batch through the `JobSystem` notifier → `request_repaint`, so an
  idle app (UI-01 Done-when 4) still applies it.

Headless (wheel, `amg-cli run`) uses the same `ScriptDoc` without a mirror.

```rust
pub struct ScriptDoc { doc: amg_doc::Document, path: Option<PathBuf>, mirror: Option<Mirror> }
```

Reads (`body.volume`, `feature.health`, selectors) need the evaluated state. They call
`ScriptDoc::evaluated()`, which runs `recompute_blocking` first if the design revision is ahead
of the evaluated one. So scripts never see stale geometry and never call `recompute()` by hand
(they may, to get the report).

## 5. Undo

- **Headless**: each `apply` is one undo entry. `with doc.transaction("label"):` groups.
- **App**: a whole script run is one undo entry "Script: <file stem>". An add-in command
  invocation is one entry labelled with the command. `doc.transaction()` inside a run nests
  and is flattened into the outer entry.
- This needs an additive `amg-doc` API (lane E, recorded in S01 Interface changes by SCR-03):
  `History::begin_group(label) -> GroupToken` / `end_group(token)`. Commands applied while a
  group is open merge into one `UndoEntry` (first `before`, last `after`, concatenated
  `commands`). The crash journal writes `GroupBegin`/`GroupEnd` records, so recovery replays a
  half-finished run as applied-so-far (the user sees the partial result, as after any crash).
- `doc.undo()` / `doc.redo()` in a script are mirrored like commands.

## 6. Selecting faces and edges from a script

Scripts cannot click. They get entities from **selectors** over evaluated bodies, then the API
converts the chosen entity into a persistent `TopoRef` with `amg_doc::naming::topo_ref_at`
(naming note §10) when it is passed to a feature.

```python
top   = box.faces().normal("+z").largest()          # one Face
holes = body.faces().kind("cylinder").radius(2.5)    # Selection of faces
rim   = ext.faces(role="end").edges()                # edges of the extrude's end cap
near  = body.edges().nearest((10, 0, 5))
```

| Method | Meaning |
|---|---|
| `faces()` / `edges()` / `vertices()` | All of a body, feature output or selection |
| `kind(k)` | `GeomClass`: `"plane"`, `"cylinder"`, `"cone"`, `"sphere"`, `"torus"`, `"bspline"`; edges `"line"`, `"circle"`, … |
| `normal(dir, tol_deg=1.0)` | Planar faces whose outward normal is within `tol_deg` of `dir` (`"+z"`, `(0,0,1)`) |
| `parallel(dir)` / `perpendicular(dir)` | Edges/axes (lines, cylinder axes) |
| `radius(r, tol=1e-6)` | Circles, cylinders, spheres |
| `at(point, tol)` / `nearest(point)` | Containing / closest by distance |
| `largest()` / `smallest()` | By area (faces), length (edges); ties broken by index for determinism |
| `sort(key)` | `"x"`, `"y"`, `"z"` (centroid), `"area"`, `"length"` |
| `created_by(feature, role=None)` | Entities whose `TopoName` has `Gen { feature, role }` (naming note §3) |
| `adjacent(face)` / `edges()` of faces / `faces()` of edges | Topology walks |
| `face_groups()` | Mesh bodies: face groups (`EntityFilter::MESH_FACE_GROUP`) |

A selector result is a list-like `Selection` of handles `(body, kind, index)` valid for the
evaluated revision it came from. Using a handle after the document changed raises
`StaleEntityError` with a hint to re-select. Persistent refs are what features store, so
the *feature* survives later edits even though the script's handle does not (the same rule as
interactive picks).

## 7. Generated feature API (SCR-02)

The binding is generated at runtime from `amg_features::dialog::registrations()` (S04, S11
§B). No per-feature glue.

| `FieldKind` | Python value accepted | Stored as |
|---|---|---|
| `Selection { filter, min, max }` | handle, `Selection`, list of handles; `Sketch`/`Profile`/`Body`/construction handles | `FieldValue::Selection(Vec<EntityRef>)` (filter and count checked) |
| `Expr { dim, .. }` | `str` expression (`"30 mm"`, `"width / 2"`), `Param` handle, or `float`/`int` | `FieldValue::Expr { src }`. A number is written as a literal *with the internal unit* (`30` → `"30 mm"`, angle `0.5` → `"0.5 rad"`) so the default-unit rule never changes its meaning |
| `Choice { options }` | option id `str` | `FieldValue::Choice` (unknown id → `FieldError` listing valid ids) |
| `Bool`, `FlipDirection` | `bool` | `Bool` / `Flip` |
| `Int { min, max }` | `int` | `Int` (range checked) |
| `Text` | `str` | `Text` |

- **Create**: `doc.features.add(kind, *, name=None, at=None, **fields)` and the generated
  namespace `doc.features.solid.extrude(**fields)` (kind segments become attributes; `-`
  becomes `_`). Missing fields come from `defaults(DialogInit { preselection: [] })`. Then
  `build(values, None)` → `Command::AddFeature`. `FieldErrors` raise `FieldError(field, msg)`.
- **Edit**: `feature.edit(**fields)`: `read(feature, design)` → overlay → `build(values,
  Some(old))` → `EditFeature`. Expression fields bound to model parameters keep their `param`.
- **Keyword names** are the `FieldKey` strings. SCR-02 tests that every registered `FieldKey`
  is a valid, non-keyword Python identifier (request to S04/S11: snake_case keys).
- **Kinds without a dialog** (e.g. `sketch.sketch`, mesh features before their dialogs exist):
  `doc.features.add_raw(kind, data: dict)` deserializes through the registry
  (`FeatureRegistration::deserialize` + `validate`). So every registered kind is scriptable
  (SCR-02 Done-when), with nicer keywords where a dialog exists.
- **Sketches** are edited through `SketchEdit` (lane B's API), not dialogs: SCR-10.

## 8. Python package layout

```
amalgam/                 # pure-Python part of the wheel (also embedded in the app as frozen modules)
  __init__.py            # re-exports from amalgam._native; __version__; api_level
  _native.pyd            # amg_script::py (pyo3, abi3)
  features/__init__.pyi  # generated (SCR-06)
  ui.py                  # add-in helpers (SCR-04): add_command, remove_command
  debug.py               # listen(port=5678): starts debugpy if installed
  py.typed
```

- `amalgam.api_level` is an integer bumped on any breaking change; scripts can assert it.
- Errors: `AmalgamError` (base) → `CommandError`, `FieldError`, `FeatureFailed`
  (`.feature`, `.diagnostics`), `RecomputeError`, `IoError`, `StaleEntityError`, `Cancelled`.
- Units: floats in mm, mm², mm³, radians, grams, seconds (internal units, vision §Principles);
  strings are expressions with explicit units.

## 9. Add-ins (SCR-04)

```
%APPDATA%\Amalgam\addins\<name>\manifest.toml     # name, version, description, entry = "main.py",
                                                  # run_on_startup = false, api_level = 1
%APPDATA%\Amalgam\addins\<name>\main.py           # def run(ctx): ...   def stop(ctx): ...
%APPDATA%\Amalgam\scripts\<name>.py | <name>\main.py   # scripts: top-level code or def run(ctx)
```

- Newly found add-ins are listed but **not started** until the user enables them in the script
  manager. "Run on startup" is a per-add-in toggle stored in prefs (not in the manifest
  after first load), as in Fusion.
- `ctx.add_command(id, label, on_execute, *, tooltip="", icon=None, placement="SOLID/CREATE",
  shortcut=None, enabled=None)`. The id becomes `script.<addin>.<id>` and goes through
  `UiRegistry::register_dynamic` (S11). `on_execute(cmd_ctx)` runs on the script thread with
  `cmd_ctx.selection` (handles) and `cmd_ctx.doc`.
- S11's `CmdId` holds `&'static str`. Dynamic ids are leaked (`Box::leak`) once per distinct
  id per process and reused on re-registration. That is a bounded leak (ids are short and few).
- `stop(ctx)` runs on disable and at exit; commands an add-in registered are removed
  automatically even if `stop` raises.
- Add-in dialogs (Fusion `CommandInputs`) need an owned, runtime-built `DialogSchema`; S11
  renders only `&'static` schemas today. Deferred to SCR-16 with an S11 request.

## 10. CLI (SCR-05, SCR-08, SCR-11, SCR-12)

```
amg-cli info
amg-cli recompute <file.amg> [--save] [--json]
amg-cli export <file.amg> -o <out> [--format F] [--bodies NAME,...] [--refinement low|medium|high] [--json]
amg-cli convert <in> <out> [--unit mm|cm|m|in|ft] [--json]       # mesh/STEP formats, no document
amg-cli props <file.amg> [--bodies NAME,...] [--json]
amg-cli run <script.py> [-- args...]                               # needs Python (SCR-11)
amg-cli kernel replay <dir>... [--robust] [--json]                 # S20 (SCR-08)
amg-cli kernel promote <dump> --corpus <dir> --id <name>
amg-cli kernel harness --corpus <dir>[,...] --strategy <name|all> --jobs N --report out.json
amg-cli study run <file.amg> --study <name> [--json]               # SCR-12, after S13 is refined
```

- **Exit codes**: 0 success; 1 error (I/O, format, script exception); 2 usage (clap);
  3 recompute finished but at least one feature has `Health::Error`; 130 cancelled (Ctrl+C).
- **Output**: human text on stdout; with `--json` one JSON document on stdout with
  `"schema": 1`. Progress and logs go to stderr only (with `AMG_LOG` per FND-07).
- `convert` runs `amg_io::import` → `amg_io::export` without a document; `export` uses
  `amg_features::io::scene_from_document`.

## 11. What we test
- Python tests are `pytest` files under `crates/amg-script/tests/py/`, run by a Rust
  integration test (`tests/python.rs`) that builds the module in-process
  (`append_to_inittab!`) and runs pytest when Python ≥ 3.12 is found; it skips otherwise and
  fails if `AMG_REQUIRE_PYTHON=1` (CI sets it).
- The parity scripts (SCR-11) live in `tests/scripts/` and rebuild S04 golden designs.
- The mirror invariant (§4) is a property test: random command scripts applied through a
  `ScriptDoc` with a mirror into a second `Document` leave both `design.json` byte-identical.
