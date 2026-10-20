# UI framework design (S11)

Status: **Written** (S11 refinement, 2026-10-20). Linked from
[`docs/specs/S11-ui-shell.md`](../specs/S11-ui-shell.md). This note gives the reasons behind the
S11 design. The binding signatures are in the spec's Interfaces section.

## 1. Library survey (crates.io, checked 2026-10-20)

| Crate | Version | Needs egui | Licence | Use |
|---|---|---|---|---|
| `egui`, `eframe`, `egui-wgpu`, `egui_extras` | 0.37.0 (2026-10-15) | — | MIT OR Apache-2.0 | Pinned. `egui-wgpu` 0.37.0 needs `wgpu ^30.0`, so wgpu is 30.0.1. |
| `egui_kittest` | 0.37.0 | 0.37 | MIT OR Apache-2.0 | UI tests (dev-dep). Features: `snapshot` (dify image diff), `wgpu` (render via egui-wgpu/wgpu 30), `eframe` (`Harness::new_eframe`). |
| `kittest` | 0.5.0 | — | MIT OR Apache-2.0 | Transitive (AccessKit-tree queries). |
| `egui_dock` | 0.21.1 (2026-08-06) | **0.36** | MIT | ADR-005 docking crate. **No release for egui 0.37 yet.** The `release-0.22` branch is still on 0.36. |
| `egui_tiles` | 0.18.0 (2026-10-19) | 0.37 | MIT OR Apache-2.0 | rerun's tiling/tab crate. Released in lockstep with egui. The fallback. |
| `egui-phosphor` | 0.14.0 | 0.36 | MIT OR Apache-2.0 | Not used: it lags egui, and CAD icons are custom anyway. |
| `egui_ltreeview` | 0.9.0 | 0.36 | MIT | Not used: lags egui. The browser tree is built in-house on `ScrollArea::show_rows`. |
| `egui-notify` / `egui-toast` | 0.23 / 0.22 | 0.36 | MIT | Not used: lag egui. Toasts are about 150 lines of our own code. |
| `rfd` | 0.17.2 | none | MIT | Native file dialogs (Windows `IFileDialog`). |
| `directories` | 6.0.0 | none | MIT OR Apache-2.0 | `%APPDATA%\Amalgam` for prefs and shortcut files. |
| `resvg` | 0.45.x (via `egui_extras` `svg` feature) | — | Apache-2.0 OR MIT | SVG icon rasterisation. |
| `nucleo-matcher` | 0.3.1 | none | MPL-2.0 | Considered for fuzzy search but **rejected**: it is unmaintained since 2024, and our command list (about 1 000 entries) needs only a 100-line subsequence scorer. |

**Lesson:** third-party egui widgets usually ship 0 to 22 days after an egui minor release.
egui_dock's lag was 0 days for 0.36, 2 for 0.35, 3 for 0.34 and 22 for 0.33. The framework
therefore takes only a few such dependencies, and puts each one behind a thin facade
(`amg_ui::dock`).

**Docking decision.** ADR-005 names egui_dock, and that stays the preferred crate. It supports
tabs, splits and detached windows, and Fusion-like layouts need nothing more.
- The facade `amg_ui::dock::DockHost` exposes only what the shell needs: add a panel into a
  slot, show/hide/focus it, serialise the layout, and render it.
- If egui_dock has no egui-0.37 release when UI-01 starts, pin the egui family to **0.36.2**
  until it ships. `egui-wgpu` 0.36.2 also needs `wgpu ^30.0`, so FND-09 and S10 are
  unaffected.
- Switching to `egui_tiles` 0.18 is the second fallback. It has no floating windows, which
  we do not need at P0.
- This is S11 open question 1 (owner).

## 2. Shell layout (Fusion parity)

```
┌ App bar: [doc tabs …] ─────────────────────────────── [undo/redo] [save] [search] ┐
├ Workspace switcher ▾ │ Toolbar tabs: SOLID SURFACE MESH INSPECT … (contextual SKETCH) ┤
│ Toolbar: groups (CREATE ▾ MODIFY ▾ …) with pinned buttons                           │
├──────────┬──────────────────────────────────────────────────────┬─────────────────┤
│ Browser  │ Dock area: viewport tab(s) + docked panels           │ (feature dialog │
│ (left)   │   view cube, nav bar, selection-filter bar overlays  │  floats here,   │
│          │                                                      │  anchored right)│
├──────────┴──────────────────────────────────────────────────────┴─────────────────┤
│ Timeline (bottom, Fusion style)                                     │ Jobs ▸ 2     │
├ Status bar: hint text │ recompute status │ selection summary │ units │ fps ───────┤
```

- The fixed regions use egui 0.37 `Panel` (top, bottom, left, right). The centre is the dock
  area. Feature dialogs are `egui::Window`s anchored top-right of the viewport, movable, with
  the position remembered, like Fusion's floating dialogs. Only one dialog is open at a time,
  because only one tool is active at a time.
- **Workspaces.** The registry decides the grouping. The default proposal follows current
  Fusion:
  - The **Design** workspace has the tabs Solid, Surface, Mesh, Form (contextual), Inspect and
    Utilities.
  - **Simulation**, **Generative** and **Manufacture-AM** are separate workspaces.
  - The S11 Summary lists Mesh and Form as their own workspaces. Owner open question 3 decides
    this. Either way it only changes `TabDef::workspace` values in lane C's registrations, not
    framework code.
- **Document tabs.** Several open documents, each with its own `amg_doc::Document`,
  `RecoverySession`, selection, camera and undo. Only the active one is polled at full rate.
  Background documents still get `poll` every frame, which is cheap and lets their recompute
  finish. This is open question 2.

## 3. Frame loop (eframe 0.37)

eframe 0.37 splits the frame into `App::logic(&Context, &mut Frame)` and
`App::ui(&mut Ui, &mut Frame)`. The shell uses them as follows.

`logic` (no layout):
1. `doc.poll(&jobs)`. This starts, cancels or collects recompute and never blocks (S01 DOC-12).
2. `doc.drain_events()`. The events go to the `SceneAdapter` (RenderScene updates and
   tessellation jobs), the browser/timeline caches, selection revalidation and the
   `on_doc_events` of every panel.
3. `JobTracker::poll`. Finished tracked jobs run their completion callbacks, which get
   `&mut AppCx`.
4. `PickService::poll`. The latest hover/region pick results land in `PickState`.

`ui`:
5. Layout, in this order: app bar, toolbar, browser, timeline, status bar, then the dock area,
   whose viewport tab is `ViewportPanel`. The viewport widget turns egui input over its rect
   into `ViewportEvent`s and routes them (§4).
6. Then the active tool's dialog window, popups (S-key search, marking menu, context menus) and
   toasts.
7. **End of frame.** The `CommandQueue` is applied: each `CommitBatch` becomes one
   `Document::transaction(label)`. Errors become notifications (UI-10). Applying at the end
   means no widget ever holds `&mut Document` while another reads it, and one frame's input
   gives one consistent snapshot.

**Repaint.** egui repaints on input. The `JobSystemConfig::notifier` calls
`Context::request_repaint` when job state changes (S00). Picks and previews request a repaint
while they are in flight. When nothing changes the app is idle at 0 % CPU.

## 4. Viewport input routing

`ViewportEvent`s are routed in priority order. The first handler that returns `Consumed` wins.

1. **Text focus.** If an egui text edit has focus, keys go to egui. The only exceptions are
   `Esc` (closes the edit) and shortcuts with Ctrl or Alt.
2. **Open popup** (marking menu, S-key box).
3. **Manipulator** under the cursor (UI-06), by CPU hit test on the projected gizmo geometry.
4. **Active tool** (`Tool::input`).
5. **Navigation.** The active `NavPreset` maps buttons and modifiers to orbit, pan or zoom.
   It runs only if the tool ignored the event, so a tool can take, for example, left-drag
   while middle-drag still orbits.
6. **Default Select tool.** Click selects, drag does a window/crossing box, a long press
   cycles through overlapping entities (select-through), and right-click opens the marking
   menu.
7. **Shortcuts.** The `ShortcutMap` is checked for the current scopes (Sketch, Viewport,
   Global).

**Keys.**
- `Esc` cancels the active tool. With no tool, it clears the selection.
- `Enter` commits the active tool (= OK).
- Starting a new command while a tool is active cancels the current tool. Fusion does this
  too, unless the tool marks itself `commit_on_replace`.

**Marking menu gesture.**
- A right press and release within 4 px and 300 ms opens the radial menu at the cursor.
- A right press followed by a drag of more than 12 px selects the wedge in the drag direction
  on release ("gesture mode", as in Fusion).
- Holding without moving for more than 300 ms opens the menu and waits.

**Picking is asynchronous.** Hover picks are requested when the cursor moved and no pick is in
flight (latest-wins). Results arrive 1–2 frames later through S10's non-blocking readback.
`ToolCx::hover()` returns the latest completed hit, together with the cursor position it was
taken at, so a tool can reject a stale hit.

## 5. Extension points (how a domain lane adds a module)

UI-01 creates stub modules for every domain lane: `amg_ui::{inspect, mesh, form, sim,
generative, am, io, materials}`. Each stub has an empty `register` function and an
`inventory::submit!`. After that, no lane edits `lib.rs` or other framework files.

A module registers everything through `Registrar` (spec Interfaces):

| Need | API | Example |
|---|---|---|
| A workspace or toolbar tab | `Registrar::workspace`, `toolbar_tab`, `toolbar_group` | lane F: Simulation workspace |
| A button, menu item or search entry | `Registrar::command(CommandDef)` | lane C: `mesh.remesh` |
| An interactive operation | `CmdAction::StartTool` + `impl Tool` | lane C: Plane Cut tool with its own dialog |
| A dockable panel | `Registrar::panel(PanelDef)` + `impl Panel` | lane C: Mesh Analysis panel |
| Browser items | `Registrar::browser_provider` | lane F: studies; lane G: print setups |
| Marking menu / context menu entries | `Registrar::marking_menu`, `context_menu` | lane C: Repair on a selected mesh body |
| How a timeline node looks and is edited | `Registrar::feature_ui(FeatureUiInfo)` | lane C: `mesh.remesh` opens its hand-built dialog |
| How a body kind is displayed | `Registrar::tessellator` | lane C: implicit → mesh |
| Persistent viewport overlays | `Registrar::overlay_provider` | lane G: build plate |
| Preferences page | `Registrar::prefs_page` | lane G: printer profiles |
| Icons | `Registrar::icon(id, svg_bytes)` | every lane |

Features with a dialog schema (S04 SOL-01 and later domain features) need **no** amg-ui module
at all:
- UI-05 turns every `DialogRegistration` into a command, a toolbar entry and a timeline entry
  automatically.
- That is why lane A has no `amg-ui::solid` module.

Determinism and conflicts:
- Ids are strings with a module prefix (`mesh.remesh`, `sim.study.new`). Duplicate ids are a
  startup panic in debug and a logged error in release, like S01's duplicate feature kinds.
- Ordering inside a group uses an `order: i32` key, then the id, so the toolbar layout does not
  depend on link order.

## 6. Generic feature dialogs (UI-05) and the schema request to S04

- S04 says each feature exposes a declarative dialog schema. `amg-features` cannot depend on
  `amg-ui`, so the schema types must live in `amg-features` (proposed:
  `amg_features::dialog`, owned by lane A through SOL-01).
- S11 states the minimum shape it renders (spec Interfaces, "requested from S04 SOL-01").
  Key points:
  - **Pure data + four functions.**
    - `schema()`: the field list, each with label, kind, conditions and an optional
      manipulator.
    - `defaults(init)`: initial values, including the pre-selection.
    - `read(&feature)`: the values of an existing feature, for edit mode.
    - `build(values, prev)`: a new `FeatureBox`, or per-field errors.
  - Also `anchors(feature, design, eval)`: where manipulators sit. It is evaluated on the
    *preview* result, so arrows follow the preview.
  - **Expression fields** carry `ExprSrc` text plus, in edit mode, the model `ParamId`.
    Editing such a field emits `SetParamExpr { id, expr }` instead of a new inline `Expr`.
    This is Fusion behaviour: the timeline dimension `d3` stays `d3`. The renderer puts
    `EditFeature` and the `SetParamExpr`s in one transaction.
  - **Selection fields** carry an `EntityFilter` (bit set shared with the UI selection system)
    and min/max counts. The first empty selection field takes focus automatically.
  - **Conditions** (`visible_if` and `enabled_if`) are a small closed expression type, not
    closures, so SCR-02 (Python bindings from the schema) can read them too.
- **Live preview.** After the last valid change the renderer waits for the debounce (prefs,
  default 150 ms), then calls `Document::preview(AddFeature | EditFeature)`.
  - The result is latest-wins: a new change cancels the in-flight preview handle.
  - The preview result is drawn on the preview layer. Its health is shown in the dialog
    banner.
  - S04 sets a preview budget of < 100 ms for typical inputs.
- **Edit mode.** Double-clicking a timeline node opens the dialog with `read()` values. The
  view shows the model as it was just before that feature. This uses a *preview* of
  `SetMarker` on a scratch snapshot, not an undoable `SetMarker`. OK emits `EditFeature`.

## 7. Selection

- **Items.** A selection item is a resolved display target plus the authored reference:
  `SelItem { target: PickTarget, entity: EntityRef }`. `PickTarget` is
  `{ body/sketch/construction, sub-kind, sub-index }` in the current `Evaluated`.
  - **Picked items:** S10 hit → `PickMap` → `PickTarget` → `EntityRef`. For topology this goes
    through `BodyNaming` at that index, plus a fingerprint (S01 request R1).
  - **Stored refs** (for example a dialog in edit mode): `EntityRef` → display target through
    S01 request R2 (`resolve_for_display`).
- **Filters.** `EntityFilter` combines the user's selection-filter bar with the active
  selection field's filter. Hits that fail the filter are skipped, and the next depth layer is
  tried.
- **Window and crossing.** Dragging left to right is a window (fully inside). Dragging right
  to left is a crossing (touching). This is Fusion's rule and needs S10 region picking with
  two modes.
- **Select-through.** A long press (≥ 500 ms) without movement opens a list of every hit along
  the pick ray within 3 px, ordered by depth (S10 `AllAlongRay`).
- **Priority.** "Select priority" (bodies / faces / edges / components) changes which sub-kind
  a click on a face resolves to.
- **Selection sets** are saved in the document as an `ExtRecord` of kind `ui.selection_sets`
  (S01 extension records). They are undoable through `SetExt` and listed in the browser. No S01
  change is needed.
- **Lifecycle.** The selection is per document. It is cleared on `DocumentReplaced`. On
  `BodyRemoved` and `BodyGeometryChanged` it is revalidated: items whose `EntityRef` no longer
  resolves are dropped silently.

## 8. Sketch mode (UI-09, UI-13, UI-14, UI-16, UI-17, UI-18)

- **Session.** `SketchSession { feature: FeatureId, working: SketchFeature (clone), solver
  worker, dof: Option<DofReport>, palette state }`.
  - Entering (Create Sketch → pick a plane, or Edit Sketch from the timeline or browser)
    animates the camera to "look at" the sketch plane (S10).
  - The contextual SKETCH tab and the Sketch Palette panel appear.
  - Finish Sketch leaves the session.
- **Edits.** Every user edit is a `SketchEdit` applied to the working copy and then issued as
  `Command::EditFeature`, so there is one undo entry per edit (S03 contract).
  - Downstream features recompute in the background through the normal incremental recompute.
    Unchanged dependants are cache hits.
  - The session draws from the **working copy** (solved locally), so the user never waits for
    document recompute.
- **Solver worker.** S03 promises synchronous, `Send` solver entry points that must not run on
  the UI thread.
  - The session owns `Option<SolverState { solver: NativeSolver, sketch: Sketch, drag:
    Option<Box<dyn DragSession>> }>`. A job takes it by value and gives it back in its result.
    At most one job is in flight.
  - Cursor targets that arrive while a job runs are coalesced: only the latest is kept, so
    old targets are dropped (latest-wins).
  - SK-05 gives p95 < 2 ms per drag step, so a drag lags at most one frame.
  - After each structural edit an `analyze` job (SK-04, < 10 ms for 500 entities) refreshes
    DOF colouring.
- **Queries on the UI thread.** `amg_sketch::query::{pick, snap_candidates}` are pure,
  read-only functions budgeted at < 0.1 ms for 500 entities (SK-20). Running them on the UI
  thread every hover is inside the 16 ms budget and avoids a frame of lag in the snap glyphs.
  The S03 "never on the UI thread" contract covers solver entry points only. The spec writes
  this down.
- **Inference.**
  - `snap_candidates(sketch, p, radius = 8 px in sketch units, from = rubber-band anchor)`
    gives candidates. The highest-priority candidate draws its glyph and dashed alignment
    lines.
  - On commit its `suggested` constraint is added in the same `SketchEdit::Batch`
    (auto-constraint).
  - Holding Ctrl suppresses inference (Fusion).
  - Grid snapping is a UI rule applied after inference.
  - Candidates S03 already filtered with `check_constraint` (redundant ones) are never shown.
- **Over-constraint UX** (QUESTIONS #16: refuse and offer driven).
  - Before adding a dimension, the tool runs `check_constraint` in a job.
  - `Conflicting` or `Redundant` opens a popup: "This dimension would over-constrain the
    sketch. Create as driven dimension?" Yes adds it with `driving: false`; No cancels.
- **DOF colours** (theme tokens): `Under` = accent blue, `Full` = foreground (black on light,
  white on dark), `Over` = error red. Fixed or projected geometry is drawn in the "locked"
  colour.
- **Inline dimensions.** The dimension text is a world-anchored overlay label at `label_pos`.
  - Double-click opens an `ExprInput` popup at the label position. Enter emits
    `SketchEdit::SetDimension`.
  - Dragging the label moves `label_pos`. This needs S03 request R5 (`SetDimensionLabel`) and
    S01/S03 request R6 (UI-only fields excluded from cache keys).
- **Typed values while drawing.** In the Line and Rectangle tools, typing a number shows a
  length/angle field next to the cursor (Tab switches fields), like Fusion's heads-up input.
  The value becomes a driving dimension on commit.

## 9. Scene adapter (DocEvent → RenderScene, UI-20)

- The adapter lives in `amg-ui` (S01 §9, S10). It maps `BodyId`, `SketchId` and
  `ConstructionId` to S10 `RenderEntityKey(u32)` and keeps the reverse `PickMap`.
- **Tessellation.** It goes through a `BodyTessellator` registry keyed by the `BodyGeom`
  variant:
  - The framework provides Mesh (direct upload; UI-20) and Solid (BREP-08 tessellation as a
    job; UI-22).
  - Lane C registers Implicit and Form.
  - Jobs are latest-wins per body. While a body re-tessellates, the old GPU mesh stays visible.
- **Preview layer.** Each frame `PreviewDriver` hands the adapter a preview `Evaluated` diff
  (only bodies whose `Arc` differs from the committed state). These go to S10's preview layer
  (ghost style, R-S10-6).
- **Sketches and construction geometry** are drawn through the overlay layer (RND-07), not as
  scene meshes. This keeps sketch-mode updates per frame cheap.

## 10. Testing approach

- **`amg_ui::testing::AppHarness`** wraps `egui_kittest::Harness`. It builds the real
  `AmalgamApp` with:
  - a headless `JobSystem`;
  - a `CpuPickService`, which ray-casts the scene adapter's triangle soups with `amg_core`
    intersectors, so picking works without a GPU;
  - a `NullViewport`, which draws nothing.
  It adds `drive_until_idle()`: it repeats `harness.run()` until no job, pick or preview is
  pending, with a 10 s timeout.
- **Widget queries go through AccessKit labels** (`get_by_label`, `get_by_role`). Every
  custom-painted framework widget (toolbar buttons, timeline nodes, browser rows, marking
  menu wedges) must set `Response::widget_info` with its label, or kittest cannot see it.
  This is a review rule.
- **Snapshot tests** (kittest `snapshot` + `wgpu`, rendered on WARP in CI) cover theming and
  layout regressions only. Each tolerance is set per test (`SnapshotOptions::threshold`).
  Logic is asserted on state, not pixels.
- **Scripted scenarios.** Sketch tutorial acceptance (UI-16) and the Phase 1 mesh flow (MESH-23)
  are kittest scripts: click, drag at world points through `AppHarness::world_to_screen`, type
  text, then assert document state (DOF, profile count, feature health, volumes).
- **Frame budget.** A criterion bench `shell_frame` runs one full frame (logic + ui) on a
  synthetic document: 200 features, 2 000 browser rows and 1 000 commands. The target is
  p95 < 4 ms. A debug-build tracing layer warns about any UI-thread span over 16 ms.
