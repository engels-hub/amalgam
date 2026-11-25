# S11 — UI shell & interaction

| | |
|---|---|
| **Status** | Ready |
| **Priority** | P0 |
| **Crates** | `amg-ui` framework modules (`app`, `module`, `command`, `shortcut`, `search`, `marking`, `tool`, `input`, `nav`, `pick`, `select`, `widgets`, `dialog`, `manip`, `browser`, `timeline`, `sketch`, `params`, `scene`, `jobs`, `notify`, `prefs`, `theme`, `icons`, `dock`, `testing`), `amg-app` |
| **Lane** | D |
| **Depends on** | S00 (FND-04 jobs + notifier + UI-thread guard, FND-07 diagnostics, FND-09 `Gpu`, FND-10 units); S01 (DOC-04 expressions, DOC-05 commands/undo, DOC-07 file I/O, DOC-09 events, DOC-12 recompute/preview, DOC-14 recovery); S03 (SK-04, SK-05, SK-07..11, SK-16..22 for sketch mode); S04 (SOL-01 dialog schema, for UI-05); S10 (RND-01..04, RND-07) |
| **Parity** | Fusion UI layout and interaction model; [03-fusion-parity.md](../03-fusion-parity.md) §Sketch (sketch tools), §Data & other (Parameters) |
| **Design note** | [docs/notes/ui-framework.md](../notes/ui-framework.md) |

## Summary
The application shell is laid out close to Fusion so that muscle memory transfers. It has:
- document tabs and a workspace switcher;
- a toolbar with tabs and groups, a browser tree on the left and a timeline at the bottom;
- floating feature dialogs on the right;
- a right-click marking menu, an S-key command search and selection filters;
- on-canvas manipulators and the full sketch-mode UX.

S11 also owns the **framework** that every domain lane builds its `amg-ui::<module>` on: the
command registry, tool state machine, viewport input routing, selection, generic and
hand-built dialog widgets, browser/timeline providers, the scene adapter and the test harness.
After UI-01, domain lanes add modules without touching framework files.

## Scope
**In:**
- The `amg-app` binary: start-up, `Gpu` sharing with eframe and egui-wgpu, diagnostics, and
  shut-down.
- The shell layout: document tabs, workspaces, toolbar, menus, docking, the status bar, and
  dark and light themes.
- The command registry with shortcuts, the S-key search and the marking menu.
- The tool trait, viewport input routing, the Fusion navigation preset and asynchronous pick
  integration.
- Selection: filters, priority, window/crossing, select-through and selection sets.
- Dialog widgets (expression input, selection field, dialog frame) and the generic
  feature-dialog renderer with live preview.
- Manipulators (distance, angle, triad, plane), bound to dialog fields in both directions.
- The browser tree and the timeline panel.
- Sketch-mode UX: session, palette, inference, creation, constraint, dimension and modify
  tools, glyphs, DOF colouring and inline dimensions.
- The job/progress panel, notifications and error display.
- File operations: new, open, save, save-as, recent files, close prompt and crash recovery.
- Preferences, the other navigation presets and the shortcut editor.
- The Parameters dialog.
- The `DocEvent` → `RenderScene` scene adapter and pick map.
- `amg_ui::testing`.

**Out:**
- Workspace and tool modules of the domain lanes: `amg-ui::{inspect}` (lane A),
  `{mesh, form}` (C), `{sim, generative}` (F) and `{am, io, materials}` (G).
- Dialog **schemas** of features (S04 SOL-01 and the domain specs; S11 renders them).
- Rendering, camera maths, the GPU ID buffer, overlay and text rendering (S10).
- The Python console and add-in commands (S18 SCR-03/04). They use the dynamic registration
  API defined here.
- The material browser (S17 MAT-03) and import/export dialogs (S12 IO-01).

## Design
The full reasoning, library survey and layout diagram are in the
[UI framework note](../notes/ui-framework.md). The binding decisions follow.

### 1. Stack and versions
- `egui`, `eframe` (features `wgpu` and `accesskit`; `persistence` off, because
  layout and prefs are our own files), `egui-wgpu` and
  `egui_extras` (features `svg`, `serde`): all **0.36.2** (owner decision 2026-10-20, QUESTIONS
  #23, ADR-005 amendment; move to 0.37 when egui_dock supports it). Where this spec names
  0.37 APIs (`App::logic`/`App::ui`, `Panel`, `egui_kittest` 0.37), UI-01 uses the 0.36.2
  equivalents; the facades hide the difference. **wgpu 30.0.1** (egui-wgpu 0.37
  requires `wgpu ^30.0`). `egui_kittest` 0.37.0 (`snapshot`, `wgpu`, `eframe`) is a
  dev-dependency.
- **Docking (ADR-005):** `egui_dock`, behind the `amg_ui::dock::DockHost` facade.
  - As of 2026-10-20 egui_dock 0.21.1 still requires egui 0.36, and no 0.37 release exists.
  - Decided 2026-10-20 (owner, QUESTIONS #23): the egui family is pinned to **0.36.2** until
    egui_dock supports 0.37 (0.22 expected). egui-wgpu 0.36.2 also uses wgpu 30, so FND-09 and S10 are not
    affected.
  - Second fallback: `egui_tiles` 0.18 (already on egui 0.37).
  - Open question 1 (resolved 2026-10-20).
- Only egui-family crates that ship together with egui are allowed as dependencies, and every
  further egui widget crate needs a facade. Icons, toasts and the tree are built in-house
  (note §1).
- `rfd` 0.17 for native file dialogs, `directories` 6 for config paths, and `inventory`
  (already used by S01) for module registration.

### 2. App start-up and frame loop (`amg-app`, `amg_ui::app`)
1. `amg_jobs::diag::init` and `install_panic_hook` (FND-07), then `amg_jobs::mark_ui_thread()`.
2. `Gpu::new_blocking(&GpuConfig::from_env())` (FND-09), allowed before the event loop. The
   `wgpu_options.wgpu_setup` of `eframe::NativeOptions` is
   `WgpuSetup::Existing(WgpuSetupExisting { instance, adapter, device, queue })` cloned from
   `Gpu`, so render and compute share one device.
3. `JobSystem::new` with `notifier` = a closure that calls `request_repaint` on an
   `Arc<OnceLock<egui::Context>>`. The context is filled in eframe's creation callback.
4. `amg_features::link()` (S01 linker caveat), then `UiRegistry::build()`, which collects
   every `inventory`-submitted `UiModule` and every `DialogRegistration`.
5. The recovery check (UI-19), then the first frame. The target is the vision's < 2 s cold
   start.

Frame (eframe 0.37 `App::logic` then `App::ui`; note §3):
- `logic`: `doc.poll` → `drain_events` → scene adapter / panels / selection → `JobTracker::poll`
  → `PickService::poll`.
- `ui`: layout and widgets, then the active tool's dialog, popups and toasts.
- **End of frame:** the `CommandQueue` is applied. Each `CommitBatch` becomes one
  `Document::transaction`. Errors go to notifications.
- No widget holds `&mut Document`. Everything mutates through `AppCx::submit`.
- **UI-thread rule.** No blocking call happens in a frame. Blocking calls are file dialogs,
  `wait_blocking`, OCCT, the solver, and tessellation. They run in jobs, or on a dedicated
  thread for `rfd`.

### 3. Layout, workspaces, docking, theme (UI-01)
- **Fixed regions** use egui 0.37 `Panel`s: app bar (document tabs, quick access), toolbar
  (workspace switcher, tabs, groups), browser (left), timeline (bottom), and status bar.
- **The centre is the `DockHost`.** It holds the viewport tab or tabs and module panels. The
  layout is saved per workspace in `%APPDATA%\Amalgam\layout.json`.
- **Feature dialogs** are movable `egui::Window`s anchored to the top-right of the viewport,
  like Fusion's floating dialogs.
- **Workspaces** come from the registry. The framework registers **Design** with the tabs
  Solid, Surface, Inspect and Utilities, plus the contextual tab **Sketch**. The other lanes
  register the rest.
  - The default proposal follows current Fusion: Mesh and Form are tabs of Design, and
    Simulation, Generative and Manufacture-AM are separate workspaces.
  - Open question 3 decides the Mesh/Form placement.
  - Switching workspace changes the toolbar tabs, the default panels and the available
    marking-menu entries. It never changes the document.
- **Contextual tabs** (Sketch, Form edit) are shown while their `ContextId` is active and take
  focus on entry.
- **Theme.** `Theme::{Light, Dark, System}` (default: System) maps to `ThemeTokens` (accent,
  hover/pre-select, selected, error, warning, DOF colours, preview, grid).
  - The tokens are applied to `egui::Visuals` and handed to S10 for highlight and overlay
    colours, so the panels and the viewport always match.
- **Document tabs.** There are N documents. Each `DocTab` owns its `Document`,
  `RecoverySession`, `Selection`, camera state, active tool and `JobTracker` scope. Open
  question 2.

### 4. Command registry, shortcuts, search (UI-02)
- **Name collision.** `amg_doc::Command` (a document mutation) is imported as `DocCommand`.
  UI commands are `CommandDef`s with a `CmdId(&'static str)`. Ids are module-prefixed
  (`sketch.line`, `mesh.remesh`) and unique: a duplicate panics in debug and is logged and
  dropped in release.
- **A `CommandDef` feeds:**
  - the toolbar and menus (`Placement`);
  - S-key search (`label`, `keywords`, `id`);
  - shortcuts;
  - the marking menu (through providers);
  - the timeline/browser context menus;
  - S18 (SCR-04 registers commands at runtime through `UiRegistry::register_dynamic`).
- **Enablement.** `enabled(&CmdEnv) -> Enablement::{Enabled, Disabled(reason)}`. It is
  evaluated lazily, only for visible buttons and search results. Disabled buttons show the
  reason as a tooltip.
- **Shortcuts.**
  - `ShortcutMap` = the registered defaults overlaid with the user's `shortcuts.toml`.
  - Scopes: `Sketch` > `Viewport` > `Global`. Conflicts within a scope are reported in
    preferences (UI-11) and the later binding is ignored.
  - Single-letter shortcuts fire only when no text field has focus.
  - Defaults follow Fusion: `S` search, `L` line, `R` rectangle, `C` circle, `D` dimension,
    `X` construction, `T` trim, `O` offset, `P` project, `E` extrude, `F` fillet,
    `Q` press-pull, `M` move, `I` measure, `Ctrl+Z`/`Ctrl+Y` undo/redo, `Ctrl+S` save.
    Domain lanes register theirs.
- **S-key search.**
  - A popup at the cursor with a text box and a result list of up to 12 rows. Each row shows
    the icon, label, its path ("Mesh ▸ Modify"), the shortcut and the disabled reason.
  - Scoring: our own subsequence matcher with bonuses for word starts and adjacency, plus a
    recency bonus (last 20 commands). Run as you type, it takes < 1 ms over 1 000 commands.
  - Enter runs the command. Esc closes the popup.
  - Searching also matches `DialogRegistration` features and parameters by name. Picking a
    parameter opens the Parameters dialog on it.

### 5. Marking menu (UI-21)
- The radial menu has 8 wedges (N, NE, E, SE, S, SW, W, NW), plus an overflow list below it
  and "Repeat <last command>" at the top centre (Fusion).
- Contents come from `MarkingMenuProvider`s, ordered by priority. Each provider gets a
  `CmdEnv` (workspace, context, selection, hovered entity) and fills slots. The framework
  provider fills the defaults: undo, redo, press-pull/extrude, move, sketch, delete and
  measure.
- The gesture rules (click, drag-gesture, hold) are in note §4.

### 6. Tools and viewport input (UI-03)
- **One active tool per document.** The default is `SelectTool`. A `Tool` gets
  `ViewportEvent`s, draws overlays, can render its dialog in the right window and finishes
  with `FinishReason::{Commit, Cancel, Replaced}`.
  - A commit returns a `CommitBatch { label, commands }`. It is applied at the end of the
    frame as one undo entry.
- **Routing priority:** text focus → popup → manipulator → active tool → navigation →
  `SelectTool` → shortcuts (note §4). `Esc` cancels the tool. With no tool, `Esc` clears the
  selection. `Enter` commits the tool.
- **Navigation presets.** `NavPreset::{Fusion, SolidWorks, Blender}` are data tables that
  map (button, modifiers) to `Orbit | Pan | Zoom`, plus wheel direction.
  - Defaults: orbit about the cursor pick point, zoom to the cursor.
  - UI-03 ships the Fusion preset (orbit `Shift+MMB`, pan `MMB`,
    zoom with the wheel). UI-11 adds the others and the editor.
  - The view cube and nav bar are S10 widgets. The shell forwards their events.
- **Picking.** `PickService` is a trait; S10's implementation is used in the app, and
  `CpuPickService` in tests.
  - Hover picks are latest-wins and never block. `ToolCx::hover()` returns the newest result
    and the cursor position it was taken at.
  - The effective filter is `tool.pick_filter() ∩ selection-filter bar`.
- **Previews.** `PreviewDriver` wraps `Document::preview` with a debounce (prefs, default
  150 ms), latest-wins cancellation and preview-layer upload through the scene adapter.
  Tools and dialogs use it. It never touches the undo stack (S01 §6).
- **Tracked jobs.** `AppCx::spawn_tracked(job, on_done)` spawns on `JobSystem`. The callback
  runs on the UI thread with `&mut AppCx` once the handle yields a result, which is how
  domain tools get results back without polling code of their own.

### 7. Scene adapter and pick map (UI-20, UI-22)
- The adapter turns `DocEvent`s into `RenderScene` updates (S10). It assigns
  `RenderEntityKey`s and keeps `PickMap` (render key + sub-id ↔ `PickTarget`).
- `BodyGeom` is converted for display through `BodyTessellator`s:
  - Mesh is a direct upload (UI-20).
  - Solid is BREP-08 tessellation in a job (UI-22).
  - Implicit and Form are registered by lane C.
- Re-tessellation is latest-wins per body, and the old mesh stays visible meanwhile.
  `DocumentReplaced` rebuilds everything.
- Sketches and construction geometry are drawn as overlays (RND-07), not as scene meshes.
- Hover, selection, error and preview highlights go to S10 as id sets per highlight layer.

### 8. Selection (UI-04)
- `Selection` is an ordered set of `SelItem { target: PickTarget, entity: EntityRef }` per
  document.
  - Picked items map hit → `PickTarget` → `EntityRef` through `BodyNaming` (S01 request R1).
  - Stored refs map back through S01 request R2.
- **Filters.** The `EntityFilter` bit set is shared with the dialog schema (§9): body, face,
  edge, vertex, mesh face group, sketch, sketch curve, sketch point, profile, construction
  plane/axis/point, occurrence/component. The selection-filter bar sits in the viewport's
  top-right. "Select priority" offers body / face / edge / component.
- **Click rules.** Click replaces the selection. `Ctrl`/`Shift`+click toggles an item. A click
  on empty space clears the selection.
- **Box selection.** Dragging left to right is a window (fully inside). Dragging right to left
  is a crossing (touching). This uses S10 region picking.
- **Select-through.** A long press (≥ 500 ms, < 4 px movement) lists every hit along the ray
  within 3 px, ordered by depth.
- **Selection sets** are `ExtRecord` kind `ui.selection_sets`, written through
  `DocCommand::SetExt`. They are undoable and saved. The browser lists them under "Selection
  Sets".
- **Revalidation.** On `BodyRemoved` and `BodyGeometryChanged`, items whose `EntityRef` no
  longer resolves are dropped. `DocumentReplaced` clears the selection.

### 9. Dialog widgets and generic feature dialogs (UI-15, UI-05)
- **UI-15 widgets** are public framework API, used by the Phase 1 hand-built dialogs (MESH-23)
  and by UI-05:
  - **`ExprInput`** shows the expression text and the evaluated value with its unit
    (`format_quantity` per the document `UnitPrefs`).
    - It parses on every change with `ParamTable::try_eval(src, Some(dim))`. An error
      underlines the `ExprError` span and shows the message as a tooltip.
    - Autocomplete covers parameter names, functions and unit names (from
      `expr::references` and the unit table).
    - `Enter` or focus loss commits. `Esc` reverts. Up/Down nudges by the preferences
      increment.
    - A field bound to a model parameter shows its name (`d3 = 20 mm`).
  - **`SelectionField`** shows "N selected", a clear button and whether it is active. Only
    one field is active at a time, and it owns the pick filter while active.
  - **`ChoiceField`** is a dropdown or a row of icon buttons. There are also a toggle, an
    integer field and a direction flip.
  - **`DialogFrame`** gives the title, help link, collapsible groups, an error/warning banner
    (preview health) and OK / Cancel. OK is disabled with a reason while any field is
    invalid.
- **UI-05 generic renderer.** `GenericFeatureTool` is built from a `DialogRegistration`
  (requested schema, Interfaces §B).
  - It evaluates `visible_if` and `enabled_if`, auto-focuses the first empty selection field
    and runs the live preview through `PreviewDriver`.
  - On OK it emits `AddFeature`. In edit mode it emits `EditFeature` + `SetParamExpr` for
    fields bound to model parameters, all in one transaction (note §6).
  - Every registration gets a command `feature.<kind>`, a toolbar placement and a
    `FeatureUiInfo` automatically.

### 10. Manipulators (UI-06)
- Manipulator kinds:
  - `Distance`: an arrow along an axis that edits a length.
  - `Angle`: a ring about an axis that edits an angle.
  - `Triad`: move/rotate with 3 axes, 3 planes and a free centre.
  - `Plane`: a 2D handle.
- **Placement and drawing.** Placement comes from `anchors()` evaluated on the latest preview.
  Gizmos are drawn through S10 overlays at a constant screen size (90 px). Hit tests run on
  the CPU against the projected gizmo geometry, with a 6 px tolerance.
- **Binding.** A drag writes `format_quantity(value)` into the bound field's `ExprInput`, which
  replaces any expression, as Fusion does. Editing the field moves the manipulator.
  - `Ctrl` while dragging snaps to the preferences increment (1 mm / 5°).
  - A heads-up `ExprInput` follows the cursor during a drag.

### 11. Browser (UI-07)
- The tree is rebuilt only when the relevant `DocEvent`s arrive, and rendered with
  `ScrollArea::show_rows` (virtualised, so 10 000 rows are cheap).
- **Framework nodes:**
  - Document Settings (units → S01 request R4).
  - Named Views.
  - Origin (planes, axes and point).
  - Components → Bodies / Sketches / Construction / child occurrences.
  - Selection Sets.
- **Other nodes** come from `BrowserProvider`s: lane F adds studies, lane G adds print setups,
  lane A adds analyses.
- **Row actions:**
  - The eye toggle sends `SetBodyProps { visible }` for bodies or `SetOccurrenceVisible` for
    occurrences. Sketches and construction geometry have no S01 visibility field, so their
    visibility is the `ExtRecord` `ui.visibility` (`SetExt`).
  - Rename sends `RenameFeature`, `SetBodyProps.name` or `RenameComponent` (S01 request R3).
  - Rows have a context menu (providers).
  - Hovering a row highlights its geometry. Clicking selects the item. Double-clicking edits
    it (sketch → sketch mode, feature → dialog).

### 12. Timeline (UI-08)
- The timeline is a horizontal strip of node icons from `FeatureUiInfo` (kind → icon, display
  name, edit action). Unknown kinds use a generic icon. Groups show as a bracket and can be
  collapsed. Rows are virtualised at 500+ nodes. The timeline is hidden in Direct mode.
- **Rollback marker.** Dragging it sends `SetMarker`.
- **Reordering.** Dragging a node sends `MoveFeature`. Invalid drop positions are shaded red
  while dragging, using `Timeline::valid_move_range` (S01 request R7).
- **Context menu:** Edit feature, Edit sketch, Roll to here, Suppress/Unsuppress, Rename,
  Delete, Group/Ungroup, Find in browser.
- **Badges** come from `Evaluated::node(id).health`:
  - Warning and error badges show the message in a tooltip.
  - `Health::Error` nodes are tinted red. Hovering one highlights its `Diagnostic.entities`
    in the error colour.
  - Hovering any node highlights its output bodies (`BodyId.origin == id`).
  - The node being recomputed shows a progress ring (`recompute_status`).

### 13. Sketch mode (UI-09, UI-13, UI-14, UI-16, UI-17, UI-18)
- **Session.** It is created by Create Sketch (pick a plane: origin plane, construction plane
  or planar face) → `AddFeature(SketchFeature)`, or by Edit Sketch.
  - The session holds a working-copy `SketchFeature`, the solver worker state, the DOF report
    and the palette.
  - On entry the camera animates to "look at" the plane and the Sketch tab plus the Sketch
    Palette panel appear. Finish Sketch, Esc with no tool active, or the Finish button leaves
    the session.
- **Edits.** Each `SketchEdit` is applied to the working copy, then issued as
  `DocCommand::EditFeature` (one undo entry per edit; S03 contract).
  - A drag issues one `CommitDrag` on release.
  - Undo/redo while in the session reloads the working copy from the document's feature.
- **Threading.**
  - Solver calls (`solve`, `analyze`, `check_constraint`, drag steps) run in `amg-jobs` jobs
    that take the `SolverState` by value and return it, so at most one is in flight.
    Cursor targets are coalesced, latest-wins.
  - `query::pick` and `query::snap_candidates` run **on the UI thread** (pure, < 0.1 ms,
    SK-20). The S03 "not on the UI thread" contract covers solver entry points only.
- **Rendering.** All sketch geometry is drawn as S10 overlays:
  - curves, with construction curves dashed;
  - points;
  - constraint glyphs, as icons offset from the entity, with hover highlighting the related
    entities;
  - dimensions, as world-anchored labels;
  - profile shading, when "Show profile" is on.
- **DOF colouring** comes from `DofReport` (`Under` blue, `Full` foreground, `Over` red), and
  the status bar shows "DOF: n".
- **Inference.** The best `SnapCandidate` draws its glyph and alignment lines. Its
  `suggested` constraint is added in the same `SketchEdit::Batch` on commit. Ctrl suppresses
  inference. Grid snap applies when it is enabled in the palette.
- **Over-constraint.** `check_constraint` runs first. A redundant or conflicting dimension
  opens "Create as driven dimension?" (QUESTIONS #16).
- **Heads-up input.** Typing a number while drawing opens a length/angle field at the cursor
  (Tab switches fields). The typed value becomes a driving dimension.
- **Tool set** (Fusion parity §Sketch, P0 unless noted):

  | WP | Tools |
  |---|---|
  | UI-09 | Line (chained, with tangent-arc drag-back) |
  | UI-13 | Rectangle 2pt/3pt/centre, circle centre/2pt/3pt/2-tangent, arc 3pt/centre/tangent, polygon, slots ×4, point, ellipse, construction toggle |
  | UI-14 | The 12 geometric constraints, sketch dimension (linear, aligned, angular, radial, diameter), fix/unfix |
  | UI-16 | Trim, extend, break, fillet, chamfer, offset, mirror, rect/circ pattern, move/copy/rotate/scale |
  | UI-17 | Project, include, intersect (P0) |
  | UI-18 (P1) | Spline (fit/control), conic, text, insert DXF/SVG, canvas image, intersection curve, project to surface |

### 14. Jobs, notifications, errors (UI-10)
- **Job panel** (bottom-right, expandable): `JobSystem::jobs()` grouped by parent, with a
  progress bar, the message, elapsed time and a cancel button (`JobSystem::cancel`). The
  status bar shows the count of running jobs and the recompute status.
- **Toasts** (`Notification`) carry severity, title, body, hint and a `details` section. They
  are made from any `amg_core::Diagnostic` with `user_message`, `hint`, `severity` and
  `amg_core::report` for the details.
  - Results with `is_cancelled()` are never shown as errors.
  - Errors stay until dismissed. Info and warnings auto-hide after 6 s.
  - There is a "Copy details" button and an "Open log folder" button.
- **GPU device lost** (`Gpu::is_lost`): a red banner "GPU device lost — save your work and
  restart Amalgam". Saving still works (it needs no GPU). Re-creating the device is out of
  scope.

### 15. File operations and recovery (UI-19)
- New, Open, Open Recent (10), Save, Save As and Save Compact (S01) go in the File menu.
  - `rfd::FileDialog` runs on a dedicated thread (COM STA, blocking), parented to the main
    window through `raw-window-handle`. The result comes back via a channel plus
    `request_repaint`.
  - `amg_doc::save` and `load` run as jobs with progress.
- Import/Export menu entries are placeholders that lane G fills (IO-01) through
  `Placement::Menu`.
- **Close prompt.** On close with a dirty document, `ViewportCommand::CancelClose` is sent
  and a Save / Don't save / Cancel `Modal` opens.
- **Recovery.** At start-up `find_recoverable` (DOC-14) feeds a recovery dialog (open
  recovered / discard / later). Each open document runs a `RecoverySession`, and
  `on_commit` is called after each applied batch.

### 16. Preferences (UI-11)
- The preferences are stored in `%APPDATA%\Amalgam\prefs.toml` (`directories`), with a schema
  `version` field. Unknown keys are kept on save.
- Pages:
  - General: theme, default units for new documents, preview debounce, autosave interval.
  - Navigation: preset, orbit about cursor, zoom direction, invert.
  - Shortcuts: an editor with conflict detection.
  - Graphics: backend override, MSAA. These apply after restart.
  - Pages added by modules (`Registrar::prefs_page`).

### 17. Parameters dialog (UI-12)
- A table built with `egui_extras::TableBuilder`. Columns: favourite ★, name, unit, expression
  (`ExprInput`), value and comment.
- Rows are grouped as User Parameters, then Model Parameters per feature. Favourites are
  pinned on top, and a text filter narrows the list.
- Actions: add or delete a user parameter (`AddParam` / `DeleteParam`), rename
  (`RenameParam`, which rewrites references; S01), edit (`SetParamExpr`, with cycle and unit
  errors shown inline) and favourite (`SetParamMeta`).
- The dialog is non-modal. Each edit is one undo entry.

## Interfaces
Rust signatures that domain lanes code against. A (framework, owned here) is binding once the
spec is `Ready`. B is a **request to S04 SOL-01**: the minimum schema shape S11 renders. C
(requests to S10) is listed under §Cross-spec requests. `egui` types are `egui::*`. `DocCommand`
is `amg_doc::Command`.

### A. `amg-ui` framework API

```rust
// ===== amg_ui::module — how a domain module plugs in =====
pub struct UiModule {
    pub id: &'static str,                 // "mesh", "sim", …
    pub register: fn(&mut Registrar<'_>),
}
inventory::collect!(UiModule);
/// In each domain module (stubs created by UI-01):
/// `inventory::submit! { amg_ui::module::UiModule { id: "mesh", register: crate::mesh::register } }`

pub struct Registrar<'a> { /* &'a mut UiRegistry, current module id */ }
impl Registrar<'_> {
    pub fn workspace(&mut self, def: WorkspaceDef);
    pub fn toolbar_tab(&mut self, def: TabDef);
    pub fn toolbar_group(&mut self, def: GroupDef);
    pub fn command(&mut self, def: CommandDef);
    pub fn panel(&mut self, def: PanelDef);
    pub fn browser_provider(&mut self, p: Box<dyn BrowserProvider>);
    pub fn marking_menu(&mut self, p: Box<dyn MarkingMenuProvider>);
    pub fn context_menu(&mut self, p: Box<dyn ContextMenuProvider>);
    pub fn feature_ui(&mut self, info: FeatureUiInfo);
    pub fn tessellator(&mut self, t: Box<dyn BodyTessellator>);
    pub fn overlay_provider(&mut self, p: Box<dyn OverlayProvider>);
    pub fn prefs_page(&mut self, def: PrefsPageDef);
    pub fn icon(&mut self, id: IconId, svg: &'static [u8]);   // include_bytes!("icons/x.svg")
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)] pub struct WorkspaceId(pub &'static str);
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)] pub struct TabId(pub &'static str);
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)] pub struct GroupId(pub &'static str);
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)] pub struct PanelId(pub &'static str);
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)] pub struct ContextId(pub &'static str); // "sketch", "form.edit"
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)] pub struct IconId(pub &'static str);

pub struct WorkspaceDef { pub id: WorkspaceId, pub label: &'static str, pub icon: IconId, pub order: i32,
                          pub default_panels: Vec<PanelId>,
                          pub on_enter: Option<fn(&mut AppCx<'_>)>, pub on_exit: Option<fn(&mut AppCx<'_>)> }
pub struct TabDef { pub id: TabId, pub workspace: WorkspaceId, pub label: &'static str, pub order: i32,
                    /// Shown only while this context is active (e.g. SKETCH tab).
                    pub contextual: Option<ContextId> }
pub struct GroupDef { pub id: GroupId, pub tab: TabId, pub label: &'static str, pub order: i32 }  // "CREATE ▾"

// ===== amg_ui::command =====
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct CmdId(pub &'static str);                      // "mesh.remesh"; module-prefixed, unique
pub struct CommandDef {
    pub id: CmdId,
    pub label: Cow<'static, str>,
    pub tooltip: Cow<'static, str>,
    pub icon: IconId,
    pub placements: Vec<Placement>,
    pub shortcut: Option<Shortcut>,
    pub keywords: &'static [&'static str],             // extra S-key search terms
    pub enabled: Enabler,
    pub action: CmdAction,
}
pub enum Placement {
    Toolbar { group: GroupId, pinned: bool, order: i32 }, // pinned = also a top-level button
    Menu { path: &'static str, order: i32 },             // "File/Export", "Edit"
    SearchOnly,
}
pub enum Enabler { Always, When(fn(&CmdEnv<'_>) -> Enablement), Dyn(Arc<dyn Fn(&CmdEnv<'_>) -> Enablement + Send + Sync>) }
#[derive(Clone, Debug, PartialEq)] pub enum Enablement { Enabled, Disabled(Cow<'static, str>) }
pub enum CmdAction {
    StartTool(fn(&ToolInit<'_>) -> Box<dyn Tool>),
    Run(fn(&mut AppCx<'_>)),
    Toggle { get: fn(&CmdEnv<'_>) -> bool, set: fn(&mut AppCx<'_>, bool) },
    Dyn(Arc<dyn Fn(&mut AppCx<'_>) + Send + Sync>),      // scripting (SCR-04)
}
impl UiRegistry {
    /// Runtime registration (Python add-ins). Id must start with "script.".
    pub fn register_dynamic(&mut self, def: CommandDef) -> Result<(), RegistryError>;
    pub fn unregister_dynamic(&mut self, id: CmdId) -> bool;
    pub fn command(&self, id: CmdId) -> Option<&CommandDef>;
    pub fn commands(&self) -> impl Iterator<Item = &CommandDef> + '_;
}
#[derive(Debug, thiserror::Error)]
pub enum RegistryError {
    #[error("command id `{0}` already registered")] Duplicate(&'static str),
    #[error("dynamic command ids must start with `script.`: `{0}`")] BadPrefix(&'static str),
}

// ===== amg_ui::shortcut =====
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub struct Shortcut { pub mods: egui::Modifiers, pub key: egui::Key, pub scope: ShortcutScope }
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
pub enum ShortcutScope { Global, Viewport, Sketch }
impl Shortcut { pub fn parse(s: &str) -> Result<Shortcut, ShortcutParseError>; /* "Ctrl+Shift+S", "viewport:E" */ }

// ===== amg_ui::app — contexts handed to modules =====
/// Read-only view for enablement, providers, panels.
pub struct CmdEnv<'a> {
    pub workspace: WorkspaceId,
    pub contexts: &'a [ContextId],
    pub design: &'a amg_doc::Design,
    pub evaluated: &'a Arc<amg_doc::Evaluated>,
    pub selection: &'a Selection,
    pub hovered: Option<&'a SelItem>,
    pub active_tool: Option<CmdId>,
    pub prefs: &'a Prefs,
    pub jobs: &'a amg_jobs::JobSystem,
}
/// Mutable access; all document mutation is deferred to end of frame.
pub struct AppCx<'a> { /* &mut DocTab, &UiRegistry, &mut Notifications, &mut CommandQueue … */ _p: PhantomData<&'a ()> }
impl<'a> AppCx<'a> {
    pub fn env(&self) -> CmdEnv<'_>;
    pub fn submit(&mut self, batch: CommitBatch);
    pub fn start_tool(&mut self, id: CmdId);
    pub fn run_command(&mut self, id: CmdId);
    pub fn selection_mut(&mut self) -> &mut Selection;
    pub fn notify(&mut self, n: Notification);
    pub fn notify_error(&mut self, e: &dyn amg_core::Diagnostic);
    pub fn open_panel(&mut self, id: PanelId);
    pub fn enter_context(&mut self, id: ContextId);
    pub fn exit_context(&mut self, id: ContextId);
    pub fn camera(&mut self) -> &mut dyn CameraControl;          // implemented over S10 camera
    pub fn jobs(&self) -> &amg_jobs::JobSystem;
    /// Spawn a job; `on_done` runs on the UI thread with the result.
    pub fn spawn_tracked<J: amg_jobs::Job>(&mut self, job: J, opts: amg_jobs::SpawnOptions,
        on_done: impl FnOnce(Result<J::Output, amg_jobs::JobError<J::Error>>, &mut AppCx<'_>) + 'static)
        -> amg_jobs::JobId;
    pub fn request_repaint(&self);
}
#[derive(Clone, Debug)]
pub struct CommitBatch { pub label: String, pub commands: Vec<amg_doc::Command> }

// ===== amg_ui::tool =====
pub struct ToolInit<'a> { pub env: CmdEnv<'a>, pub preselection: &'a [SelItem], pub edit: Option<amg_doc::FeatureId> }
pub trait Tool: 'static {
    fn id(&self) -> CmdId;
    fn label(&self) -> Cow<'static, str>;
    fn start(&mut self, _cx: &mut ToolCx<'_>) {}
    fn input(&mut self, ev: &ViewportEvent, cx: &mut ToolCx<'_>) -> InputResult;
    /// Once per frame (poll jobs/previews, update state).
    fn update(&mut self, _cx: &mut ToolCx<'_>) {}
    /// Contents of the floating dialog. Return false for tools without a dialog.
    fn dialog_ui(&mut self, _ui: &mut egui::Ui, _cx: &mut ToolCx<'_>) -> bool { false }
    fn overlays(&self, _cx: &OverlayCx<'_>, _out: &mut OverlayList) {}
    fn pick_filter(&self) -> EntityFilter { EntityFilter::ALL }
    fn status_hint(&self) -> Option<Cow<'static, str>> { None }
    /// If true, starting another command commits this tool instead of cancelling it.
    fn commit_on_replace(&self) -> bool { false }
    /// `Commit` → Some(batch) to apply (or None if nothing to do); `Cancel`/`Replaced` → None.
    fn finish(&mut self, reason: FinishReason, cx: &mut ToolCx<'_>) -> Option<CommitBatch>;
}
#[derive(Copy, Clone, Debug, PartialEq, Eq)] pub enum InputResult { Consumed, Ignored }
#[derive(Copy, Clone, Debug, PartialEq, Eq)] pub enum FinishReason { Commit, Cancel, Replaced }
pub struct ToolCx<'a> { /* AppCx + viewport state */ _p: PhantomData<&'a ()> }
impl<'a> ToolCx<'a> {
    pub fn app(&mut self) -> &mut AppCx<'a>;
    pub fn env(&self) -> CmdEnv<'_>;
    /// Ask the router to finish the tool at end of frame (e.g. after the last click).
    pub fn request_finish(&mut self, reason: FinishReason);
    pub fn hover(&self) -> Option<&HoverHit>;
    pub fn camera(&self) -> &CameraView;
    pub fn preview(&mut self) -> &mut PreviewDriver;
    pub fn selection(&self) -> &Selection;
    pub fn selection_mut(&mut self) -> &mut Selection;
    pub fn set_cursor(&mut self, icon: egui::CursorIcon);
}
#[derive(Clone, Debug)]
pub struct HoverHit { pub at: egui::Pos2, pub item: SelItem, pub world: amg_core::Point3, pub depth: f32 }

// ===== amg_ui::input =====
#[derive(Clone, Debug)]
pub enum ViewportEvent {
    Hover { pos: egui::Pos2, ray: amg_core::Ray3 },
    Press { button: egui::PointerButton, pos: egui::Pos2, ray: amg_core::Ray3, mods: egui::Modifiers },
    Release { button: egui::PointerButton, pos: egui::Pos2, ray: amg_core::Ray3, mods: egui::Modifiers },
    Click { button: egui::PointerButton, pos: egui::Pos2, ray: amg_core::Ray3, mods: egui::Modifiers, count: u8 },
    DragStart { button: egui::PointerButton, from: egui::Pos2, mods: egui::Modifiers },
    Drag { button: egui::PointerButton, pos: egui::Pos2, delta: egui::Vec2, ray: amg_core::Ray3, mods: egui::Modifiers },
    DragEnd { button: egui::PointerButton, pos: egui::Pos2, ray: amg_core::Ray3, mods: egui::Modifiers },
    Scroll { delta: egui::Vec2, pos: egui::Pos2, mods: egui::Modifiers },
    Key { key: egui::Key, pressed: bool, mods: egui::Modifiers },
    Text(String),                                    // heads-up numeric input
}
/// Camera snapshot for the frame (from S10). Points are egui logical points.
#[derive(Clone, Debug)]
pub struct CameraView { pub rect: egui::Rect, pub view: amg_core::Mat4 /* DMat4 */, pub proj: amg_core::Mat4, pub ortho: bool }
impl CameraView {
    pub fn project(&self, p: amg_core::Point3) -> Option<egui::Pos2>;
    pub fn ray(&self, pos: egui::Pos2) -> amg_core::Ray3;
    /// World length of one screen point at `p` (for constant-screen-size gizmos, pick radii).
    pub fn world_per_point(&self, p: amg_core::Point3) -> f64;
}
pub trait CameraControl {
    fn orbit(&mut self, delta: egui::Vec2, pivot: Option<amg_core::Point3>);
    fn pan(&mut self, delta: egui::Vec2);
    fn zoom(&mut self, factor: f64, at: egui::Pos2);
    fn fit(&mut self, bounds: Option<amg_core::Aabb3>);           // None = whole scene
    fn look_at_plane(&mut self, plane: &amg_core::Plane, animate: bool);
    fn set_ortho(&mut self, ortho: bool);
}

// ===== amg_ui::nav =====
#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum NavPreset { Fusion, SolidWorks, Blender, Custom }
#[derive(Copy, Clone, Debug, PartialEq, Eq)] pub enum NavAction { Orbit, Pan, Zoom }
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct NavBindings { pub drags: Vec<(egui::PointerButton, egui::Modifiers, NavAction)>,
                         pub wheel_zoom_in_is_up: bool, pub orbit_about_cursor: bool, pub zoom_to_cursor: bool }
impl NavPreset { pub fn bindings(self) -> NavBindings; }

// ===== amg_ui::pick =====
/// Display-side identity of a pickable thing in the current Evaluated.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum PickTarget {
    Body { body: amg_doc::BodyId, sub: Option<(amg_doc::TopoKind, u32)> },  // None = whole body
    SketchEntity { sketch: amg_doc::SketchId, entity: amg_sketch::EntityId },
    Profile { sketch: amg_doc::SketchId, profile: amg_sketch::ProfileId },
    Construction(amg_doc::ConstructionId),
    Occurrence(amg_doc::OccurrenceId),
}
#[derive(Copy, Clone, Debug, PartialEq, Eq)] pub enum PickMode { Nearest, AllAlongRay }
#[derive(Copy, Clone, Debug, PartialEq, Eq)] pub enum RegionMode { Window, Crossing }
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)] pub struct PickTicket(pub u64);
#[derive(Clone, Debug)] pub struct PickHitUi { pub target: PickTarget, pub world: amg_core::Point3, pub depth: f32 }
/// Implemented over S10 (app) and by `testing::CpuPickService` (tests). Never blocks.
pub trait PickService {
    fn request_point(&mut self, pos: egui::Pos2, radius_pt: f32, filter: EntityFilter, mode: PickMode) -> PickTicket;
    fn request_region(&mut self, rect: egui::Rect, filter: EntityFilter, mode: RegionMode) -> PickTicket;
    fn poll(&mut self, t: PickTicket) -> Option<Vec<PickHitUi>>;  // Some exactly once
}

// ===== amg_ui::select =====
pub use amg_features::dialog::EntityFilter;        // shared bit set (Interfaces §B)
#[derive(Clone, Debug, PartialEq)]
pub struct SelItem { pub target: PickTarget, pub entity: amg_doc::EntityRef }
#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum SelectPriority { Default, Body, Face, Edge, Component }
#[derive(Clone, Debug, Default)]
pub struct Selection { /* ordered, de-duplicated by target */ }
impl Selection {
    pub fn items(&self) -> &[SelItem];
    pub fn contains(&self, t: &PickTarget) -> bool;
    pub fn set(&mut self, items: Vec<SelItem>);
    pub fn add(&mut self, item: SelItem) -> bool;
    pub fn toggle(&mut self, item: SelItem);
    pub fn remove(&mut self, t: &PickTarget) -> bool;
    pub fn clear(&mut self);
    pub fn filter(&self) -> EntityFilter;                    // user filter bar
    pub fn set_filter(&mut self, f: EntityFilter);
    pub fn revision(&self) -> u64;                           // bumps on change (cheap cache checks)
}

// ===== amg_ui::widgets (UI-15) =====
pub struct ExprInput<'a> { /* src: &'a mut String, dim, params, unit prefs, id_salt */ _p: PhantomData<&'a ()> }
impl<'a> ExprInput<'a> {
    pub fn new(src: &'a mut String, expected: amg_doc::Dim, params: &'a amg_doc::ParamTable, prefs: &'a amg_doc::UnitPrefs) -> Self;
    pub fn bound_param(self, id: Option<amg_doc::ParamId>) -> Self;
    pub fn nudge_step(self, step: f64) -> Self;
    pub fn show(self, ui: &mut egui::Ui) -> ExprResponse;
}
pub struct ExprResponse { pub response: egui::Response, pub value: Result<amg_doc::Quantity, amg_doc::ExprError>, pub committed: bool }
pub struct SelectionField<'a> { _p: PhantomData<&'a ()> }
impl<'a> SelectionField<'a> {
    pub fn new(label: &'a str, items: &'a mut Vec<SelItem>, filter: EntityFilter, active: &'a mut bool) -> Self;
    pub fn limits(self, min: u32, max: Option<u32>) -> Self;
    pub fn show(self, ui: &mut egui::Ui) -> egui::Response;
}
pub struct DialogFrame<'a> { _p: PhantomData<&'a ()> }
pub enum DialogAction { None, Ok, Cancel }
impl<'a> DialogFrame<'a> {
    pub fn new(title: &'a str) -> Self;
    pub fn banner(self, health: Option<&'a amg_doc::Health>) -> Self;   // preview health
    pub fn ok_enabled(self, e: Enablement) -> Self;
    pub fn show(self, ctx: &egui::Context, body: impl FnOnce(&mut egui::Ui)) -> DialogAction;
}

// ===== amg_ui::preview =====
pub struct PreviewDriver { /* debounce timer, in-flight JobHandle<Arc<Evaluated>, _>, last result */ }
#[derive(Clone, Debug)] pub enum PreviewState { Idle, Pending, Computing, Ready(Arc<amg_doc::Evaluated>), Failed(String) }
impl PreviewDriver {
    /// Debounced (prefs.preview_debounce); a newer call cancels the in-flight preview.
    pub fn set(&mut self, cmd: amg_doc::Command);
    pub fn clear(&mut self);
    pub fn state(&self) -> &PreviewState;
}

// ===== amg_ui::overlay (framework-side list, translated to S10 RND-07 primitives) =====
#[derive(Default)] pub struct OverlayList { /* Vec<OverlayPrim> */ }
pub struct OverlayCx<'a> { pub camera: &'a CameraView, pub theme: &'a ThemeTokens }
impl OverlayList {
    pub fn line(&mut self, a: amg_core::Point3, b: amg_core::Point3, style: LineStyle);
    pub fn polyline(&mut self, pts: &[amg_core::Point3], style: LineStyle);
    pub fn point(&mut self, p: amg_core::Point3, size_pt: f32, color: egui::Color32);
    pub fn glyph(&mut self, p: amg_core::Point3, offset_pt: egui::Vec2, icon: IconId);
    pub fn label(&mut self, p: amg_core::Point3, text: &str, style: LabelStyle);
    pub fn fill(&mut self, loop_: &[amg_core::Point3], color: egui::Color32);           // profile shading
    pub fn gizmo(&mut self, g: GizmoPrim);                                              // manipulators
}
#[derive(Copy, Clone, Debug)] pub struct LineStyle { pub color: egui::Color32, pub width_pt: f32, pub dashed: bool, pub on_top: bool }
#[derive(Copy, Clone, Debug)] pub struct LabelStyle { pub color: egui::Color32, pub size_pt: f32, pub background: Option<egui::Color32> }
pub trait OverlayProvider { fn overlays(&self, env: &CmdEnv<'_>, cx: &OverlayCx<'_>, out: &mut OverlayList); }

// ===== amg_ui::manip (UI-06) =====
#[derive(Clone, Debug)]
pub enum GizmoPrim {
    Arrow { origin: amg_core::Point3, dir: amg_core::Dir3, id: u32, hot: bool },
    Ring  { center: amg_core::Point3, axis: amg_core::Dir3, ref_dir: amg_core::Dir3, id: u32, hot: bool },
    Triad { origin: amg_core::Point3, frame: amg_core::Mat3, id: u32, hot_part: Option<TriadPart> },
    PlaneHandle { origin: amg_core::Point3, normal: amg_core::Dir3, id: u32, hot: bool },
}
#[derive(Copy, Clone, Debug, PartialEq, Eq)] pub enum TriadPart { AxisX, AxisY, AxisZ, PlaneXY, PlaneYZ, PlaneZX, RotX, RotY, RotZ, Free }

// ===== amg_ui::browser (UI-07) =====
#[derive(Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct NodeKey(pub Cow<'static, str>);               // "body/12:0", "study/3"; provider-prefixed
pub struct BrowserNode {
    pub key: NodeKey, pub label: String, pub icon: IconId,
    pub visible: Option<bool>,                           // Some = eye toggle shown
    pub target: Option<SelItem>,                         // hover highlight / click select
    pub badge: Option<amg_core::Severity>,
    pub renamable: bool,
    pub children: Vec<BrowserNode>,
}
pub enum BrowserAnchor { Root, Component(amg_doc::ComponentId) }
pub trait BrowserProvider {
    fn id(&self) -> &'static str;
    /// Rebuild only when this returns true for some event of the frame.
    fn affected_by(&self, ev: &amg_doc::DocEvent) -> bool;
    fn build(&self, env: &CmdEnv<'_>, anchor: &BrowserAnchor, out: &mut Vec<BrowserNode>);
    fn set_visible(&self, _node: &NodeKey, _visible: bool, _cx: &mut AppCx<'_>) {}
    fn rename(&self, _node: &NodeKey, _name: &str, _cx: &mut AppCx<'_>) {}
    fn double_click(&self, _node: &NodeKey, _cx: &mut AppCx<'_>) {}
    fn context_menu(&self, _node: &NodeKey, _ui: &mut egui::Ui, _cx: &mut AppCx<'_>) {}
}

// ===== amg_ui::marking / context menus (UI-21) =====
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Wedge { N, NE, E, SE, S, SW, W, NW }
pub struct MarkingMenuBuilder { /* slots + overflow */ }
impl MarkingMenuBuilder {
    /// First provider (by priority) to claim a wedge wins; later ones go to overflow.
    pub fn wedge(&mut self, w: Wedge, cmd: CmdId);
    pub fn overflow(&mut self, cmd: CmdId);
}
pub trait MarkingMenuProvider { fn priority(&self) -> i32; fn fill(&self, env: &CmdEnv<'_>, out: &mut MarkingMenuBuilder); }
pub enum ContextTarget<'a> { Viewport(Option<&'a SelItem>), Browser(&'a NodeKey), Timeline(amg_doc::FeatureId) }
pub trait ContextMenuProvider { fn fill(&self, env: &CmdEnv<'_>, target: &ContextTarget<'_>, out: &mut Vec<CmdId>); }

// ===== amg_ui::timeline (UI-08) =====
pub struct FeatureUiInfo {
    pub kind: &'static str,                     // S01 Feature::kind
    pub display_name: &'static str,
    pub icon: IconId,
    pub edit: EditAction,
}
pub enum EditAction {
    GenericDialog,                                            // via DialogRegistration (UI-05)
    StartTool(fn(&ToolInit<'_>) -> Box<dyn Tool>),            // hand-built (MESH-23 etc.); ToolInit.edit = Some(id)
    SketchMode,                                               // sketch.sketch
    NotEditable,
}

// ===== amg_ui::scene (UI-20) =====
pub trait BodyTessellator: Send + Sync {
    /// Which BodyGeom variants this handles.
    fn handles(&self, geom: &amg_doc::BodyGeom) -> bool;
    /// Runs inside a job. Output is the S10 upload type.
    fn tessellate(&self, geom: &amg_doc::BodyGeom, quality: DisplayQuality, ctx: &amg_jobs::JobCtx)
        -> Result<amg_render::GpuMeshData /* S10 upload type; name provisional (R-S10-4) */, TessellateError>;
}
#[derive(Copy, Clone, Debug, PartialEq, Eq)] pub enum DisplayQuality { Preview, Normal, Fine }
#[derive(Debug, thiserror::Error)]
pub enum TessellateError { #[error("{0}")] Failed(String), #[error(transparent)] Cancelled(#[from] amg_core::Cancelled) }

// ===== amg_ui::panel =====
#[derive(Copy, Clone, Debug, PartialEq, Eq)] pub enum DockSlot { Left, Right, Bottom, CenterTab, Floating }
pub struct PanelDef { pub id: PanelId, pub title: &'static str, pub icon: IconId, pub slot: DockSlot,
                      pub workspaces: Option<Vec<WorkspaceId>>, pub create: fn() -> Box<dyn Panel> }
pub trait Panel: 'static {
    fn ui(&mut self, ui: &mut egui::Ui, cx: &mut AppCx<'_>);
    fn on_doc_events(&mut self, _events: &[amg_doc::DocEvent], _env: &CmdEnv<'_>) {}
}

// ===== amg_ui::notify (UI-10) =====
#[derive(Clone, Debug)]
pub struct Notification { pub severity: amg_core::Severity, pub title: String, pub body: String,
                          pub hint: Option<String>, pub details: Option<String>, pub sticky: bool }
impl Notification { pub fn from_diagnostic(e: &dyn amg_core::Diagnostic) -> Option<Notification>; } // None if cancelled

// ===== amg_ui::prefs (UI-11) / theme =====
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Prefs { pub version: u32, pub theme: Theme, pub nav: NavPreset, pub nav_custom: Option<NavBindings>,
                   pub preview_debounce_ms: u32, pub autosave_s: u32, pub default_units: amg_doc::UnitPrefs,
                   pub snap_length_mm: f64, pub snap_angle_rad: f64,
                   #[serde(flatten)] pub modules: std::collections::BTreeMap<String, toml::Value> }
pub struct PrefsPageDef { pub id: &'static str, pub title: &'static str, pub ui: fn(&mut egui::Ui, &mut toml::Value) }
#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)] pub enum Theme { Light, Dark, System }
#[derive(Clone, Debug)]
pub struct ThemeTokens { pub accent: egui::Color32, pub hover: egui::Color32, pub selected: egui::Color32,
                         pub preview: egui::Color32, pub error: egui::Color32, pub warning: egui::Color32,
                         pub dof_under: egui::Color32, pub dof_full: egui::Color32, pub dof_over: egui::Color32,
                         pub construction: egui::Color32, pub grid: egui::Color32, pub background_top: egui::Color32,
                         pub background_bottom: egui::Color32 }

// ===== amg_ui::testing (dev / test only; feature "testing") =====
pub struct AppHarness { /* egui_kittest::Harness<'static, AmalgamApp> + CpuPickService */ }
impl AppHarness {
    pub fn new() -> AppHarness;                               // empty document, NullViewport
    pub fn with_document(doc: amg_doc::Document) -> AppHarness;
    pub fn kittest(&mut self) -> &mut egui_kittest::Harness<'static, AmalgamApp>;
    /// Steps frames until no job / pick / preview is pending (timeout 10 s → panic).
    pub fn drive_until_idle(&mut self);
    pub fn run_command(&mut self, id: CmdId);
    pub fn world_to_screen(&self, p: amg_core::Point3) -> egui::Pos2;
    pub fn click_world(&mut self, p: amg_core::Point3);
    pub fn drag_world(&mut self, from: amg_core::Point3, to: amg_core::Point3);
    pub fn document(&self) -> &amg_doc::Document;
    pub fn selection(&self) -> &Selection;
}
```

### B. Requested from S04 SOL-01: `amg_features::dialog` (minimum shape S11 renders)

The schema types must live below `amg-ui`. `amg-features` is the natural home, owned by lane A
through SOL-01. S11 codes UI-05 against this shape. SOL-01 may extend it additively.

```rust
use amg_doc::{Design, Evaluated, EntityRef, Dim, ParamId, Feature, FeatureBox};

bitflags::bitflags! {
    #[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
    pub struct EntityFilter: u32 {
        const BODY = 1; const FACE = 1<<1; const EDGE = 1<<2; const VERTEX = 1<<3;
        const MESH_FACE_GROUP = 1<<4; const SKETCH = 1<<5; const SKETCH_CURVE = 1<<6;
        const SKETCH_POINT = 1<<7; const PROFILE = 1<<8; const CONSTRUCTION_PLANE = 1<<9;
        const CONSTRUCTION_AXIS = 1<<10; const CONSTRUCTION_POINT = 1<<11; const OCCURRENCE = 1<<12;
        const PLANAR_FACE = 1<<13;  // narrows FACE
        const ALL = 0x3fff;
    }
}
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct FieldKey(pub &'static str);
pub struct DialogSchema { pub kind: &'static str, pub title: String, pub help: Option<&'static str>, pub fields: Vec<FieldSpec> }
pub struct FieldSpec {
    pub key: FieldKey, pub label: String, pub tooltip: Option<String>, pub group: Option<String>,
    pub kind: FieldKind,
    pub visible_if: Option<Condition>, pub enabled_if: Option<Condition>,
    pub manipulator: Option<ManipKind>,
}
pub enum FieldKind {
    Selection { filter: EntityFilter, min: u32, max: Option<u32> },
    Expr { dim: Dim, default: &'static str },
    Choice { options: Vec<ChoiceOption>, style: ChoiceStyle },
    Bool, Int { min: i64, max: i64 }, Text, FlipDirection,
}
pub struct ChoiceOption { pub id: &'static str, pub label: String, pub icon_svg: Option<&'static [u8]> }
#[derive(Copy, Clone, Debug, PartialEq, Eq)] pub enum ChoiceStyle { Dropdown, IconRow }
#[derive(Clone, Debug, PartialEq)]
pub enum Condition { Is(FieldKey, FieldValue), NonEmpty(FieldKey), Not(Box<Condition>), All(Vec<Condition>), Any(Vec<Condition>) }
#[derive(Clone, Debug, PartialEq)]
pub enum FieldValue {
    Selection(Vec<EntityRef>),
    Expr { src: String, param: Option<ParamId> },   // param = bound model parameter in edit mode
    Choice(&'static str), Bool(bool), Int(i64), Text(String), Flip(bool),
}
#[derive(Clone, Debug, Default, PartialEq)] pub struct FieldValues(pub std::collections::BTreeMap<FieldKey, FieldValue>);
#[derive(Clone, Debug, Default)] pub struct FieldErrors(pub Vec<(FieldKey, String)>);
#[derive(Copy, Clone, Debug, PartialEq, Eq)] pub enum ManipKind { Distance, Angle, Triad, Plane }
#[derive(Clone, Debug)]
pub struct ManipAnchor { pub field: FieldKey, pub origin: amg_core::Point3, pub axis: amg_core::Dir3,
                         pub ref_dir: Option<amg_core::Dir3>, /** value shown at origin */ pub base: f64 }
pub struct DialogInit<'a> { pub preselection: &'a [EntityRef], pub design: &'a Design, pub evaluated: &'a Evaluated }
pub struct CommandMeta { pub workspace: &'static str, pub tab: &'static str, pub group: &'static str,
                         pub order: i32, pub pinned: bool, pub icon_svg: &'static [u8],
                         pub shortcut: Option<&'static str>, pub keywords: &'static [&'static str] }
pub struct DialogRegistration {
    pub kind: &'static str,
    pub schema: fn() -> &'static DialogSchema,
    pub defaults: fn(&DialogInit<'_>) -> FieldValues,
    pub read: fn(&dyn Feature, &Design) -> Option<FieldValues>,
    pub build: fn(&FieldValues, Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors>,
    pub anchors: fn(&dyn Feature, &Design, &Evaluated) -> Vec<ManipAnchor>,
    pub command: CommandMeta,
}
inventory::collect!(DialogRegistration);
```

## Work packages
All lane D. Phases are proposals (roadmap not edited here; see the Refinement log).

| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| UI-01 | App shell: `amg-app` start-up (diag, UI-thread mark, `Gpu` → `WgpuSetup::Existing`, `JobSystem` notifier → repaint), eframe 0.37 frame loop (`logic`/`ui`, end-of-frame `CommandQueue`), fixed panels + `DockHost` facade (egui_dock), theme tokens (light/dark/system), workspace/tab registry + `UiModule` inventory + stub modules for every domain lane, document tabs, undo/redo buttons, status bar, `amg_ui::testing::AppHarness` (kittest, `NullViewport`, `CpuPickService` stub) | RND-01, FND-07 | P0 | Done | 1. `amg-app` opens on the owner's machine showing the Fusion layout (app bar, toolbar, browser, timeline, status bar, viewport tab with the RND-01 mesh). Cold start to first interactive frame < 2 s (tracing span `startup`). 2. Kittest: switching workspace via the switcher changes the visible tabs (`get_by_label`). A test module registering a workspace + tab through `inventory` appears without editing framework files. 3. Light/dark snapshot tests pass on WARP. 4. With no input and no jobs, the app does not repaint (frame counter stays constant over 1 s). 5. A test job's notifier wakes the UI within one frame. 6. Two document tabs keep independent undo stacks (kittest). |
| UI-02 | Command registry (`CommandDef`, `Placement`, `Enabler`, dynamic registration), toolbar tabs/groups/dropdowns, menus, `ShortcutMap` with scopes and `shortcuts.toml` overrides, S-key search (own fuzzy scorer, recency) | UI-01 | P0 | Done | 1. Kittest: pressing `S`, typing "remsh" lists a registered `test.remesh` first, and Enter runs it. 2. Disabled commands show their reason in the tooltip and are skipped by Enter. 3. A duplicate `CmdId` panics in debug (unit test). 4. Shortcut scope test: `L` in sketch scope and `L` in global scope resolve correctly. Single letters are ignored while a text field has focus. 5. Search over 1 000 synthetic commands < 1 ms per keystroke (criterion). 6. `register_dynamic` rejects ids without the `script.` prefix. |
| UI-03 | `Tool` trait + router (priority chain, Esc/Enter, replace semantics), `ViewportEvent` synthesis from egui input, `PickService` trait + S10 implementation + latest-wins hover, Fusion navigation preset over S10 camera, `PreviewDriver`, `spawn_tracked`, `SelectTool` (click only) | UI-02, RND-04, RND-11 | P0 | Done | 1. Kittest with a scripted test tool: events arrive in order, Esc cancels (`FinishReason::Cancel`), Enter commits one undo entry, and starting another command gives `Replaced`. 2. Middle-drag orbits even while a tool is active that ignores the event. 3. Hover pick → highlight within ≤ 2 frames on the owner's GPU and with `CpuPickService`. 4. `PreviewDriver`: 20 rapid `set` calls → exactly one preview computed (test counter feature), and none enters the undo stack. 5. No UI-thread frame > 16 ms while a 2 s test job runs (tracing assertion). |
| UI-20 | Scene adapter: `DocEvent` → `RenderScene` (RND-03), `RenderEntityKey` allocation, `PickMap`, `BodyTessellator` registry with Mesh tessellator, highlight layers, preview layer feed, `CpuPickService` real implementation | UI-01, RND-03, DOC-09 | P0 | Done | 1. Scripted test: add/edit/delete 3 mesh bodies → the RenderScene contains exactly the live bodies, and an unchanged body is never re-uploaded (upload counter). 2. `DocumentReplaced` rebuilds the scene, and keys are not reused within a session. 3. `PickMap` round-trips every face index of a 10 k-triangle body. 4. A registered test tessellator for a stub `BodyGeom` variant is used. 5. Updating 1 changed body in a 100-body scene < 1 ms of UI-thread time. |
| UI-04 | Selection system: `Selection`, `SelItem`, `EntityFilter` bar, select priority, click/Ctrl/Shift rules, window/crossing box (S10 region pick), select-through list, selection sets (`ExtRecord ui.selection_sets`), revalidation on events | UI-03, UI-20 | P0 | Done | 1. Kittest on a fixture with 3 mesh bodies: window select left→right picks only fully enclosed bodies, and right→left also picks touching ones. 2. With the filter set to faces only, a click on an edge selects the adjacent face. 3. Select-through lists all 3 stacked bodies in depth order. 4. Save a selection set → reload the file → restore gives the same `EntityRef`s, and undo removes the set. 5. Deleting a selected body drops it from the selection without error. |
| UI-15 | Dialog widgets: `ExprInput` (live parse via `ParamTable::try_eval`, error spans, autocomplete of params/functions/units, nudge, bound-param display), `SelectionField`, `ChoiceField`, toggle/int/flip, `DialogFrame` (banner, OK enablement), quantity formatting per `UnitPrefs` | UI-04, DOC-04 | P0 | Done | 1. Kittest: typing `d1 + 5` in a length field with `d1 = 10 mm` shows `15.00 mm`. `12 kg` shows a unit-mismatch error underlined at the right span, and OK is disabled. 2. Typing `d` offers every parameter starting with `d`, and Tab completes. 3. `12,5` shows the `DecimalComma` hint (QUESTIONS #11). 4. Only one `SelectionField` is active at a time, and the active one sets the pick filter. 5. All widgets are queryable by label (AccessKit). |
| UI-05 | Generic feature dialog renderer: `GenericFeatureTool` from `DialogRegistration` (conditions, auto-focus, live preview with debounce, preview-health banner), edit mode (preview rollback, `EditFeature` + `SetParamExpr` in one transaction), automatic `feature.<kind>` commands + toolbar placement + `FeatureUiInfo` | UI-15, SOL-01, DOC-12 | P0 | Todo | 1. The Extrude dialog is generated entirely from SOL-01's schema (no extrude-specific UI code; grep test). Changing extent type shows/hides the right fields. 2. Kittest: select a profile, type a distance → the preview appears ≤ debounce + 100 ms on a typical part. OK yields one undo entry with `AddFeature`. 3. Edit mode: changing the distance of an existing extrude keeps its model parameter `dN` and changes its expression. 4. An invalid input disables OK with the field error. Cancel leaves the document unchanged (revision equal). |
| UI-06 | Manipulators: Distance, Angle, Triad, Plane gizmos via overlays (constant 90 px), CPU hit test, drag ↔ field binding both ways, Ctrl snapping, heads-up value | UI-05, RND-07 | P0 | Todo | 1. Dragging the extrude arrow by a scripted world delta of 12.5 mm writes `12.5 mm` into the field (prefs precision), and editing the field moves the arrow (kittest via `world_to_screen`). 2. Ctrl-drag snaps to 1 mm / 5°. 3. Triad: each of 10 parts hit-tests correctly in a 6-view matrix (unit test on `CameraView`). 4. Gizmo screen size stays 90 ± 2 px across a 1000× zoom range. |
| UI-07 | Browser tree: framework nodes (settings, named views, origin, components/bodies/sketches/construction, selection sets), `BrowserProvider` API, virtualised rows, eye toggles (`SetBodyProps`, `SetOccurrenceVisible`, `ui.visibility` ExtRecord), rename, context menu, hover highlight, click select, double-click edit | UI-04, DOC-09 | P0 | Done | 1. Kittest on a fixture with 2 components × 3 bodies: hiding a body hides it in the scene, and undo shows it again. Renaming a body persists through save/load. 2. A test provider adds a node under Root without framework changes. 3. 10 000 synthetic bodies: browser frame cost < 1 ms (criterion), and the tree is rebuilt only on affecting events (counter). 4. Hovering a row highlights the body (highlight layer contains it). |
| UI-08 | Timeline panel: node icons via `FeatureUiInfo`, groups, rollback marker drag (`SetMarker`), drag reorder with valid-range shading (`MoveFeature`), context menu, health badges + tooltips, hover highlight of outputs and error entities, recompute progress ring, hidden in Direct mode | UI-04, DOC-05, DOC-12 | P0 | Done | 1. Kittest on a 20-node test timeline: dragging the marker to index 5 sends `SetMarker{5}`, and nodes ≥ 5 render as rolled back. 2. Dragging a node before its input is refused (no command sent, red shading shown). A valid move sends one `MoveFeature`. 3. An erroring test feature shows an error badge with the diagnostic message as tooltip. 4. Suppress/rename/delete/group via the context menu each give one undo entry. 5. 500 nodes render < 1 ms (criterion). |
| UI-10 | Job/progress panel (`JobSystem::jobs()`, grouped, cancel), status-bar recompute/job indicators, toasts from `Diagnostic` (severity, hint, details via `report`, copy, open log folder), cancelled-is-not-error rule, GPU device-lost banner | UI-01, FND-04, FND-07 | P0 | Done | 1. Kittest: a 2 s test job shows progress; pressing Cancel resolves it as `Cancelled` within 200 ms and no error toast appears. 2. A failing job with an `amg_core::Error::InvalidArgument` shows the user message and hint, and "Details" contains the `report` chain. 3. Info toasts auto-hide after 6 s, and errors stay. 4. Simulated `is_lost()` shows the banner and Save stays enabled. |
| UI-19 | File operations & recovery: New/Open/Open Recent/Save/Save As/Save Compact, `rfd` on a dedicated thread parented to the window, save/load as jobs with progress, dirty-close prompt (`CancelClose` + `Modal`), recovery dialog from `find_recoverable`, `RecoverySession::on_commit` per batch, `Placement::Menu` slots for Import/Export | UI-10, DOC-07, DOC-14 | P0 | Done | 1. Kittest with a fake file-dialog backend (trait): save → reopen gives an equal `design.json`. 2. Closing a dirty document shows the prompt, and Cancel keeps the app open. 3. Kill test: commit 5 commands, abort the process, restart → the recovery dialog offers the document and "Open recovered" restores it. 4. Opening a 50 MB file never blocks a frame > 16 ms (tracing assertion). |
| UI-21 | Marking menu: radial 8 wedges + overflow + repeat-last, click/gesture/hold modes, `MarkingMenuProvider` + `ContextMenuProvider` APIs, framework default provider | UI-03, UI-04 | P0 | Done | 1. Kittest: right-click opens the menu, clicking the N wedge runs its command. A right-drag gesture toward E runs E's command without the menu staying open. 2. "Repeat <last>" runs the previous command. 3. A test provider adds an entry for a selected mesh body only (context check). 4. Every wedge is queryable by label. |
| UI-12 | Parameters dialog: table (favourite, name, unit, expression, value, comment), user/model grouping, filter, add/delete/rename/edit/favourite via S01 commands, inline cycle/unit errors | UI-15, DOC-05 | P0 | Review | 1. Kittest: add user param `w = 20 mm`, set a model param to `w * 2` → value `40.00 mm`. Renaming `w` → `width` rewrites the model param text. 2. `a = b`, `b = a` shows a cycle error on the second edit, and the document is unchanged. 3. Each edit is one undo entry. 4. 1 000 params render < 2 ms (criterion). |
| UI-22 | Solid body display: `BodyTessellator` for `BodyGeom::Solid` via BREP-08 (per-face ranges → face sub-ids, edge polylines → edge sub-ids), quality presets, latest-wins re-tessellation | UI-20, BREP-08 | P0 | Done (app check with a real solid pending owner) | 1. A STEP test part (≥ 1 000 faces) displays with every face and edge pickable (PickMap round-trip). 2. Editing an upstream parameter re-tessellates only changed bodies (counter). 3. Tessellation runs on a job (no frame > 16 ms). |
| UI-23 | SVG icon registry and icons: `Registrar::icon` (id → embedded SVG), rasterised per DPI scale via egui_extras `svg` (resvg) with a texture cache, one MIT-licensed open icon set (Lucide or Tabler, chosen with an ADR-012 check and a THIRD_PARTY.md row, SVGs vendored with their licence), icons for the built-in commands and the Phase 1 domain modules, drawn in the toolbar, marking-menu wedges, browser nodes and timeline nodes; missing icons fall back to the label | UI-02, UI-07, UI-08, UI-21 | P2 | Done (100/150 % visual check pending owner) | 1. Every Phase 1 toolbar command, marking-menu wedge and browser/timeline node kind with an `IconId` draws its icon (kittest checks the texture is used; a test lists commands without an icon). 2. Icons are crisp at 100/150/200 % scale (rasterised per scale, cached: re-rasterising happens only on a scale change, counter test). 3. An unknown or `NONE` icon falls back to the text label without a panic. 4. Cold start adds < 20 ms for all icons (measured, timing assert only under AMG_STRICT_TIMING=1). 5. The licence file of the icon set is in the repo and listed in THIRD_PARTY.md. |
| UI-09 | Sketch mode core: Create/Edit/Finish Sketch, plane pick, look-at, SKETCH contextual tab + Sketch Palette (grid, snap, show profile/points/dimensions/constraints, slice), working copy + `SolverState` job (latest-wins), `analyze` → DOF colouring + status DOF, overlay rendering of curves/points/profiles, select & drag (`DragSession` → `CommitDrag`), inference engine on `query::snap_candidates` (glyphs, alignment lines, auto-constraints, Ctrl suppress), heads-up numeric input, Line tool | UI-03, UI-04, UI-15, SK-04, SK-05, SK-19, SK-20, RND-07 | P0 | Done (GUI sketch check pending owner) | 1. Kittest: create a sketch on XY, draw a closed 4-segment line chain using inference → 4 coincident + 2 horizontal + 2 vertical constraints auto-added, 1 profile detected. 2. DOF shown = 4 for that rectangle, entities coloured `Under`. 3. Dragging a corner for 1 000 frames: p95 UI-thread time < 2 ms per frame and no frame > 16 ms. Release = exactly one undo entry. 4. Undo inside the session reloads the working copy. 5. Finish Sketch leaves sketch mode and the timeline shows the sketch node with health Ok. |
| UI-13 | Sketch creation tools: rectangles (2pt/3pt/centre), circles (centre/2pt/3pt/2-tangent), arcs (3pt/centre/tangent), polygon, slots ×4, point, ellipse, construction toggle (`X`), all with inference + heads-up dimensions | UI-09, SK-21, SK-22 | P0 | Todo | 1. One kittest per tool: the created entities and constraint set equal the SK-21 builder output for the same picks. Typed heads-up values create driving dimensions with those values. 2. Each tool is reachable via toolbar, S-key and (where Fusion has one) its shortcut. 3. Esc mid-tool leaves the sketch unchanged. |
| UI-14 | Constraint & dimension UX: 12 constraint tools, smart sketch dimension (linear/aligned/angular/radial/diameter by pick), glyph rendering with hover-related highlight and delete, inline dimension editing (`ExprInput` at label), label drag (needs R5/R6), over-constraint "create as driven?" flow, fix/unfix | UI-09 | P0 | Todo | 1. Kittest: dimensioning a fully constrained rectangle's side offers "driven" and creates a driven dimension on Yes. 2. Double-click a dimension → typing `w/2` (with user param `w`) sets the expression, the sketch re-solves, and DOF is unchanged. 3. Each constraint tool applies to a valid selection and refuses invalid kinds with a status hint. 4. Glyph hover highlights exactly the constrained entities. |
| UI-16 | Sketch modify tools: trim, extend, break, fillet, chamfer, offset, mirror, rect/circ pattern, move/copy/rotate/scale; **sketch tutorial acceptance script** | UI-13, UI-14, SK-07, SK-16, SK-17, SK-18 | P0 | Todo | 1. One kittest per tool matching the S03 edit op's fixture outcome. 2. The scripted "Fusion sketch basics" scenario passes: a bracket profile with outer rectangle, filleted corners, two concentric hole pairs, a slot, mirror about a centreline and dimensions driven by 3 user parameters reaches **DOF 0**, with all entities `Full`, 3 profiles and the expected areas within 1e-9 relative. Changing a parameter keeps DOF 0. ≤ 60 scripted UI actions; script in `crates/amg-ui/tests/sketch_tutorial.rs`. |
| UI-17 | Project / include / intersect tool: pick edges, faces or bodies → `ProjectionLink`s, locked projected entities in the "locked" colour, refresh on upstream edits | UI-09, SK-09 | P0 | Todo | 1. Kittest: project the edges of an extruded box face into a sketch, dimension to them, edit the extrude → the projected edges and dependent geometry move. 2. Projected entities cannot be dragged. |
| UI-11 | Preferences: `prefs.toml` (versioned, unknown keys kept), pages (general, navigation presets SolidWorks/Blender/custom, shortcuts editor with conflicts, graphics), module pages | UI-02, UI-03 | P1 | Todo | 1. Every preference round-trips through `prefs.toml`, and an unknown module key survives a save. 2. Switching to the Blender preset makes middle-drag orbit and Shift+middle pan (kittest on camera deltas). 3. Assigning an in-use shortcut shows the conflict and keeps the old binding until confirmed. |
| UI-18 | Advanced sketch tools: spline (fit/control), conic, text, insert DXF/SVG, canvas image, intersection curve, project to surface | UI-13, SK-08, SK-10, SK-11 | P1 | Todo | 1. One kittest per tool on the S03 fixtures. 2. A text block "AgB" creates 3 outer + 4 hole profiles. 3. DXF insert of a sample file shows the expected entity count. |

**Dependency changes from the Draft:**
- UI-03: + UI-02.
- UI-04: + UI-20.
- UI-05: UI-03, SOL-01, DOC-04 → UI-15, SOL-01, DOC-12.
- UI-07: UI-01 → UI-04.
- UI-08: UI-01, DOC-03 → UI-04, DOC-05, DOC-12.
- UI-09: UI-03, SK-05 → + SK-04, SK-20 (agreed cross-spec repoint), SK-19, UI-04, UI-15,
  RND-07.
- UI-12: UI-05 → UI-15, DOC-05.
- UI-01: + FND-07.
- UI-11: UI-01 → UI-02, UI-03.

**Proposed phases:**
- Phase 1: UI-01..04, UI-07, UI-08, UI-10, UI-15, UI-19, UI-20, UI-21.
- Phase 2: UI-05, UI-06, UI-09, UI-12, UI-13, UI-14, UI-16, UI-17, UI-22.
- Phase 3: UI-11, UI-18.

## Testing
- **UI tests** use `egui_kittest` 0.37 through `amg_ui::testing::AppHarness`.
  - The harness runs a headless `JobSystem`, `CpuPickService` and `NullViewport`, so no GPU
    is needed for logic tests.
  - Widgets are found through AccessKit labels. Every custom-painted widget must set
    `widget_info` (review rule).
  - Tests assert document state (commands, DOF, health, selection), not pixels.
- **Snapshot tests** (kittest `snapshot` + `wgpu`) cover only theme/layout regressions: shell
  in light and dark, toolbar, timeline with badges, sketch glyphs.
  - They run on WARP in CI (FND-05/09). Each test sets its own threshold.
  - Bless with `UPDATE_SNAPSHOTS=1`, the kittest convention.
- **Scenario scripts:** the sketch tutorial (UI-16) and, owned by lane C, the Phase 1 mesh flow
  (MESH-23).
- **Benchmarks** (criterion, `crates/amg-ui/benches`): `shell_frame` (200 features, 2 000
  browser rows, 1 000 commands), `search_1000`, `timeline_500`, `browser_10k`, `params_1000`,
  `scene_update_1of100`.
- **Frame-budget guard:** a debug-build tracing layer records UI-thread spans. A test helper
  `assert_no_frame_over(16ms)` is used in the job/preview/drag tests.

## Performance targets
| Item | Target | Vision budget |
|---|---|---|
| Cold start to interactive viewport | < 2 s (UI-01) | < 2 s |
| Any UI-thread frame | ≤ 16 ms, always | 16 ms |
| Shell frame CPU (logic + ui) on the `shell_frame` fixture | p95 < 4 ms | headroom inside 16 ms |
| Hover pick → highlight | ≤ 2 frames | interactive feel |
| S-key search per keystroke, 1 000 commands | < 1 ms | — |
| Feature dialog input → preview visible | debounce (150 ms) + preview compute (S04: < 100 ms typical) | instant feel |
| Sketch drag, 200-entity sketch | UI-thread p95 < 2 ms/frame; ≤ 1 frame solver lag | instant feel |
| Esc → tool cancelled | same frame | — |
| Idle CPU (no input, no jobs) | no repaints | — |

## Open questions
1. **(owner)** egui_dock has no release for the pinned egui 0.37 (0.21.1 needs 0.36; checked
   2026-10-20). If that is still true when UI-01 starts:
   (a) temporarily pin egui/eframe/egui-wgpu to **0.36.2** (same wgpu 30.0.1, so no impact on
   FND-09/S10) until egui_dock 0.22 ships (*default*), or
   (b) switch the docking facade to `egui_tiles` 0.18 (already on 0.37, but no floating
   windows), or (c) use egui_dock from git.
   **Resolved 2026-10-20 (owner, QUESTIONS #23):** (a). The egui family stays on 0.36.2 until
   egui_dock supports 0.37; the ADR-005 amendment records it.
2. **(owner)** Several documents as tabs in one window, as in Fusion (*default*), or one
   document per window?
   **Resolved 2026-10-20 (owner, QUESTIONS #24):** tabs in one window.
3. **(owner)** Workspace grouping. Follow current Fusion: Mesh and Form are tabs in Design;
   Simulation, Generative and Manufacture-AM are workspaces (*default*). Or keep Mesh and Form
   as separate workspaces as the Draft summary lists? Only lane C's registration data
   changes.
   **Resolved 2026-10-20 (owner, QUESTIONS #25):** follow current Fusion (Mesh and Form are
   tabs inside Design).
4. Feature dialogs float (anchored top-right, movable) instead of docking. Decided: Fusion
   parity. Revisit after Phase 1 use.
5. Phase 1 mesh dialogs: MESH-23 hand-builds them (QUESTIONS #22). UI-15 widgets are in Phase 1
   so they can be used. MESH-23 adding the dependency UI-15 is a request to S07.

## Cross-spec requests
Requests to other specs. The owning spec decides; nothing is edited here.

**S00:**
- **R-S00-1:** `[workspace.dependencies]` uses `wgpu = "30.0.1"` (S00 lists `"29"`).
- **R-S00-2:** also add `egui`, `eframe`, `egui-wgpu`, `egui_extras` = `0.36.2` (owner pin,
  QUESTIONS #23; was 0.37.0), and the matching `egui_kittest` 0.36.x as a dev-dependency.
- **R-S00-3:** FND-09's doctest for `WgpuSetup::Existing` targets egui-wgpu 0.37
  (`WgpuSetupExisting { instance, adapter, device, queue }`, checked).

**S01:**
- **R1:** `amg_doc::naming::topo_ref_at(eval: &Evaluated, body: BodyId, kind: TopoKind, index:
  u32) -> Option<TopoRef>`. It builds the `TopoRef` (name + fingerprint) for a picked
  sub-entity, outside recompute.
- **R2:** `Evaluated::resolve_for_display(&self, design: &Design, r: &EntityRef) ->
  Result<Resolved, ResolveError>`. It shows stored refs (dialog edit mode, selection sets).
- **R3:** `Command::RenameComponent { id: ComponentId, name: String }` (additive).
- **R4:** `Command::SetDesignMeta { name: Option<String>, default_units: Option<UnitPrefs> }`
  for Document Settings → Units.
- **R6:** a UI-only data rule. Data that does not affect geometry must not change the cache
  key, such as a dimension's `label_pos`. Otherwise moving a dimension label recomputes every
  downstream feature, because keys include upstream keys. Proposal: `Feature::key_data(&self)
  -> serde_json::Value` (default `to_json()`), which `SketchFeature` overrides to drop
  `label_pos`.
- **R7:** `Timeline::valid_move_range(&self, id: FeatureId) -> std::ops::RangeInclusive<usize>`
  for drag shading, consistent with `MoveFeature` validation.
- **Minor:** `Document::preview` returns `JobHandle<Arc<Evaluated>, FeatureError>` (the second
  type parameter is missing in S01). Consider renaming `amg_doc::Diagnostic` (struct) to avoid
  confusion with the `amg_core::Diagnostic` trait (for example `HealthNote`).

**S03:**
- **R5:** `SketchEdit::SetDimensionLabel { id: ConstraintId, pos: Point2 }`.
- **Note** in the "Threading" paragraph: `query::*` may run on the UI thread, while solver
  entry points may not.

**S04 SOL-01:** adopt Interfaces §B (`amg_features::dialog`) as the dialog schema shape, or
extend it additively.

**S07:** MESH-23 may add the dependency UI-15 (shared dialog widgets in Phase 1). MESH-23
registers its timeline nodes with `EditAction::StartTool`.

**S10**, for the RND-01/03/04/07 refiner:
- **R-S10-1, viewport widget.** It renders the scene into an offscreen MSAA target, resolved
  into a texture registered with `egui_wgpu::Renderer::register_native_texture` (or a paint
  callback) inside a given `egui::Rect`. It exposes `CameraView` data (view, proj, rect,
  ortho), and the view cube and nav bar emit camera actions.
- **R-S10-2, camera.** A controller implementing `CameraControl`: orbit about a pivot, pan,
  zoom-to-cursor, fit, animated `look_at_plane`, ortho/perspective, named views.
- **R-S10-3, picking without blocking.**
  - Point queries with radius, filter mask and `Nearest | AllAlongRay` return
    (`RenderEntityKey`, sub-kind face/edge/vertex/curve/point, sub-index, depth, world
    point).
  - Region queries return `Window | Crossing` sets.
  - Requests return a ticket and are polled. Results arrive in ≤ 2 frames.
- **R-S10-4, ids.** The adapter allocates `RenderEntityKey(u32)`. Sub-ids come from per-face
  ranges and per-edge polylines in `GpuMeshData`.
- **R-S10-5, highlight layers.** Hover, Selected, Error and Preview, each set as
  `(key, Option<sub>)` lists with theme colours.
- **R-S10-6, preview layer.** Temporary bodies drawn in ghost style, replaced as a whole each
  frame.
- **R-S10-7, overlay primitives (RND-07).** Lines (dashed, width in px, on-top or
  depth-tested), points, icon glyphs from an atlas, world-anchored MSDF labels with
  background, filled polygons, and gizmo meshes (arrow, ring, triad, plane handle) at
  constant screen size.
- **R-S10-8:** theme tokens drive the background gradient, highlights and grid.
- **R-S10-9:** device-lost signalling and frame-time stats for the status bar.
- **R-S10-10:** offscreen rendering for kittest snapshots and thumbnails (RND-09).

**S18:** SCR-03/04 use `UiRegistry::register_dynamic` (ids `script.*`) and `PanelDef` for the
console.

## Refinement log
### 2026-10-20 — S11 refinement (lane D agent)
**Research** (crates.io and docs.rs, 2026-10-20):
- `egui`, `eframe`, `egui-wgpu`, `egui_extras` and `egui_kittest` are 0.37.0, released
  2026-10-15, MIT OR Apache-2.0. `egui-wgpu` 0.37 → `wgpu ^30.0`, and wgpu 30.0.1 exists.
- eframe 0.37's `App` is split into `logic(&Context, &mut Frame)` and
  `ui(&mut Ui, &mut Frame)`. The frame loop (§2) is built on that split.
- `egui_wgpu::WgpuSetup::{CreateNew, Existing}` and
  `WgpuSetupExisting { instance, adapter, device, queue }` are confirmed in 0.37, so S00's
  device-sharing plan holds.
- egui 0.37 containers include `Panel`, `Modal`, `Popup` and `Scene`. The `InputState` methods
  `consume_shortcut` and `consume_key` exist.
- `egui_kittest` 0.37 `Harness` offers `new_ui`, `new_eframe`, `get_by_label`, `get_by_role`,
  `drag_at`, `hover_at`, `key_press`, `key_combination`, `snapshot` and `run`/`step`.
  Features: `snapshot`, `wgpu` (wgpu 30) and `eframe`. It queries through AccessKit
  (`kittest` 0.5).
- **egui_dock 0.21.1 requires egui 0.36. No 0.37 release exists yet**, and the `release-0.22`
  branch is still on 0.36.
  - Its past lag behind egui minors was 0–22 days.
  - `egui_tiles` 0.18.0 (2026-10-19) already targets 0.37.
  - `egui-wgpu` 0.36.2 also targets wgpu 30, so pinning 0.36.2 is a safe fallback.
  - This became owner question 1.
- egui widget crates `egui-phosphor`, `egui_ltreeview`, `egui-notify` and `egui-toast` are all
  still on 0.36. They are rejected to avoid lockstep risk; icons, tree and toasts are built
  in-house.
- `rfd` 0.17.2 (MIT, `raw-window-handle` 0.6) and `directories` 6.0.0 (MIT OR Apache-2.0) are
  adopted. `resvg` 0.45 (Apache-2.0 OR MIT) comes through `egui_extras/svg`.
- `nucleo-matcher` (MPL-2.0, last release 2024) is rejected in favour of a small scorer of our
  own.

**Decisions:**
- **Framework / module split.**
  - `UiModule` uses `inventory`, and UI-01 creates stub modules for every lane, so domain lanes
    never edit framework files.
  - All extension points (commands, workspaces, tabs, panels, browser, marking/context menus,
    timeline info, tessellators, overlays, prefs pages, icons) go through `Registrar`.
  - Features with a dialog schema need no UI module (automatic commands), which is why lane A
    has no `amg-ui::solid`.
- **Command naming.** UI `CommandDef`/`CmdId` vs `amg_doc::Command` (imported as
  `DocCommand`). All mutations are deferred to an end-of-frame `CommandQueue`, applied as one
  transaction per `CommitBatch`.
- **Dialog schema** types must live below `amg-ui`. The minimum shape is proposed as
  `amg_features::dialog` for S04 SOL-01 (Interfaces §B). Expression fields bound to model
  parameters edit through `SetParamExpr` (Fusion behaviour).
- **Picking** is a `PickService` trait (S10 implementation and CPU fake), so UI logic tests run
  without a GPU.
- **Sketch threading.** Solver calls run in single-flight jobs that move the `SolverState` in
  and out (latest-wins). `query::*` runs on the UI thread, inside the SK-20 < 0.1 ms budget.
- **Selection sets and sketch/construction visibility** live in `ExtRecord`s
  (`ui.selection_sets`, `ui.visibility`), so no S01 command is needed for them.

**WP restructuring** (IDs kept; new IDs from UI-13):
- UI-09 (too big) is split into UI-09 (session core + inference + Line), UI-13 (creation
  tools), UI-14 (constraints/dimensions), UI-16 (modify tools + tutorial acceptance), UI-17
  (project/include) and UI-18 (P1 advanced tools).
- New UI-15 (dialog widgets, Phase 1, usable by MESH-23).
- New UI-19 (file operations & recovery, split out of UI-01).
- New UI-20 (scene adapter + pick map) and UI-22 (solid display via BREP-08).
- New UI-21 (marking menu, split out of UI-02).
- Every WP now has a measurable Done-when, and UI-09's "Fusion sketch tutorial reproducible"
  is now the scripted DOF-0 bracket scenario in UI-16.
- Dependency changes are listed under the WP table.

**Cross-cutting finding (R6):** S01 cache keys include upstream keys. Moving a sketch
dimension label (`label_pos`, UI-only) would therefore invalidate every downstream feature.
This was raised with S01 and S03.

**Roadmap (proposal, not edited):**
- Phase 1 lane D adds UI-15, UI-19, UI-20 and UI-21.
- Phase 2 lane D adds UI-13, UI-14, UI-16, UI-17 and UI-22.
- Phase 3 lane D adds UI-18.

### 2026-11-10 - UI-01 app shell

*Landed (status `Review`):*
- **`amg-app`** (`src/main.rs`, `src/demo.rs`, `build.rs` for the git SHA): `diag::init` +
  `install_panic_hook`, `mark_ui_thread`, `Gpu::new_blocking` with `amg_render::OPTIONAL_FEATURES`
  handed to eframe as `WgpuSetup::Existing` (`HIGH_THROUGHPUT`, multisampling 0, no depth), the
  `JobSystem` whose notifier is `amg_ui::app::repaint_notifier` (an `Arc<OnceLock<egui::Context>>`
  filled in the creation callback), `amg_features::{mesh,sketch}::link()`, `UiRegistry::build()`,
  `egui_glue::install` + one view, then `AmalgamApp`. The RND-01 displaced icosphere (1 M tris,
  `--tris N`, `--no-demo`) is generated and uploaded on a job and inserted when it finishes.
  The whole start-up is the span `startup`; its event `first interactive frame` carries
  `elapsed_ms` (`--exit-after-first-frame` for repeat measurements). The dock layout is saved
  on exit.
- **`amg-ui` framework:** `module` (`UiModule` + `inventory::collect!`, `Registrar` with
  `workspace`/`toolbar_tab`/`toolbar_group`/`panel`, the id newtypes, `WorkspaceDef`, `TabDef`,
  `GroupDef`), `registry::UiRegistry` (`build`, `from_modules`, sorted queries, contextual-tab
  filtering; duplicate ids panic in debug and are logged and dropped in release), `panel`
  (`PanelDef`, `DockSlot`, `Panel`), `dock::DockHost` (the only file naming `egui_dock`: open in a
  slot, focus, close, retain, JSON round trip with unknown panels dropped), `theme` (`Theme`,
  `ThemeTokens` light/dark, applied to `egui::Visuals` only when the resolved theme changes, and
  to the S10 view background), `viewport` (`ViewportBackend`, `NullViewport`, `RenderViewport`
  over `egui_glue::Viewport`), `app` (`AmalgamApp` with `logic` / `ui` / `end_frame`, `DocTab`,
  `CommitBatch` applied as one `Document::transaction` each and rolled back as a whole on error,
  `CmdEnv`, `AppCx`, per-workspace dock layouts persisted to `%APPDATA%\Amalgam\layout.json`),
  `shell` (app bar: File menu, Undo/Redo with the step labels as tooltips, document tabs, `+`,
  Theme menu; toolbar: workspace switcher menu, tabs, group menus; browser; timeline strip;
  status bar with status/error text, recompute status, `Jobs: N`, units and the device-lost
  banner), `design` (the framework's Design workspace: Solid, Surface, Inspect, Utilities,
  contextual Sketch, and their groups) and `testing` (`AppHarness`, `AppHarnessBuilder` with an
  optional wgpu setup, `drive_until_idle`, `CpuPickService` stub).
- **Lane stubs** `amg_ui::{inspect, mesh, form, sim, generative, am, io, materials}`, each with
  `register` + `inventory::submit!`. Following QUESTIONS #25, `mesh` registers the MESH tab of
  Design, `form` the contextual FORM tab (`form.edit`), and `sim`, `generative` and `am` register
  the Simulation, Generative Design and Manufacture workspaces with one tab each; `inspect`, `io`
  and `materials` are empty. The owning lanes edit these files from now on. `amg_ui::link()`
  keeps every module's submit in the binary (called by `UiRegistry::build`).

*Evidence (Done-when):*
1. `amg-app` (debug build, RTX 5060 Ti / DX12) opens with the Fusion layout and the 1 003 520-tri
   RND-01 mesh in the viewport tab (screenshot for the owner's confirmation). `startup` →
   `first interactive frame` with the final binary: 721, 674, 642, 640 and 649 ms over five
   launches (earlier builds: 663-777 ms), all while other workers were compiling (budget 2 s). One launch while the disk was saturated took 4 812 ms,
   and the very first launch spent 90 s in `Gpu::new_blocking` (first driver use under heavy
   I/O) before the measuring script's timeout killed it. The mesh arrives about 60-110 ms after
   the first frame.
2. `tests/shell.rs`: `switching_workspace_changes_the_visible_tabs` clicks `DESIGN ⏷` →
   `Test Workspace`; its `TEST TAB` appears and `SOLID` disappears, then Simulation shows
   `SETUP`. The test module is an `inventory::submit!` in the test file
   (`inventory_module_is_collected_without_framework_edits`).
3. `tests/snapshots.rs`: `shell_dark` and `shell_light` rendered by kittest's wgpu renderer on a
   WARP device (`force_fallback_adapter`), threshold 0.6, at most 64 differing pixels; images in
   `crates/amg-ui/tests/snapshots/`.
4. `tests/idle.rs`: the real app on a raw `egui::Context`; a frame runs only when a requested
   repaint is due (as in eframe). After settling, the frame counter is unchanged over 1 s and no
   repaint is scheduled; a finished job wakes it, and it settles again.
5. `job_notifier_wakes_the_ui_within_one_frame`: a gated job shows `Jobs: 1`; after its release
   the notifier requests a repaint and the very next `step` shows `Jobs: 0`.
6. `two_document_tabs_keep_independent_undo_stacks`: with two tabs, undo/redo through the
   toolbar buttons only ever changes the active document.

*Deviations:*
- egui/eframe/egui-wgpu 0.36.2, egui_dock 0.21.1 and egui_kittest 0.36.2 (QUESTIONS #23).
  egui 0.36.2 already has `Panel`, `CentralPanel::show(ui)` and eframe's `App::logic`/`App::ui`,
  so the frame loop is as specified.
- Interfaces §A is implemented as the UI-01 subset: `Registrar` has no `command`,
  `browser_provider`, `marking_menu`, `context_menu`, `feature_ui`, `tessellator`,
  `overlay_provider`, `prefs_page` or `icon` yet; `CmdEnv` lacks `selection`, `hovered`,
  `active_tool` and `prefs`; `AppCx` lacks `start_tool`, `run_command`, `selection_mut`, `notify`,
  `notify_error`, `camera` and `spawn_tracked`. Each arrives additively with its WP (UI-02, -03,
  -04, -07, -10, -11, -20, -21). Additive extras: `AppCx::{registry, doc_tab}`,
  `AmalgamApp::{frame, with_cx, submit, request_undo, request_redo, layouts_json,
  restore_layouts, save_layout, status_message}`, `UiRegistry::{from_modules, panel_by_str,
  workspace_owner, tab_owner}`.
- `AppHarness` adds `builder()`, `app()`, `app_mut()` and `picks()`; `run_command`,
  `world_to_screen`, `click_world`, `drag_world` and `selection` come with UI-02/03/04.
  `CpuPickService` is a stub with a pending counter until UI-03 defines `PickService`.
- Errors of a commit batch are shown in the status bar until UI-10's notifications exist.
- `end_frame` and the UI-state setters (workspace, tab, document, theme) request one more
  frame, because the frame that made the change drew its widgets before it; without this an
  error or the new undo state only appeared on the next input.
- Selected-label text is white on the accent colour in both themes (egui's light default was
  unreadable on it).
- Undo/redo also react to Ctrl+Z / Ctrl+Y / Ctrl+Shift+Z outside text fields; UI-02 moves them
  into the `ShortcutMap`.
- Group and workspace dropdowns are labelled `CREATE ⏷`: egui's default fonts have no `▾`.
- egui_dock writes never-laid-out rects as ±∞, which JSON cannot hold; `DockHost::to_json`
  stores those coordinates as 0 (egui_dock recomputes rects on the next layout).
- Navigation is not part of UI-01 (UI-03/RND-11): the demo mesh is shown from a fixed home view.

*Leftovers:*
- The owner confirms the layout on the screenshot (`D:/coding/amalgam-handoff/UI-01.png`).
- Start-up was measured on a debug build on a busy machine; a release build on an idle machine
  would show the real cold start.
- The S10 background is the midpoint of the theme gradient (`Background` has only `Solid`);
  RND-02 adds the gradient.

### 2026-11-12 - UI-02 command registry, shortcuts, search
Landed in `amg-ui`: `command` (`CmdId`, `CommandDef` + builder methods, `Placement`, `Enabler`,
`Enablement`, `CmdAction::{Run, Toggle, Dyn}`, `RegistryError`), `shortcut` (`Shortcut::parse`,
`ShortcutScope`, `ShortcutMap` with scopes and conflicts, `ShortcutOverrides` = `shortcuts.toml`),
`search` (own scorer, `SearchIndex`, `Recency`, popup), `builtin` (module `app`: `app.search` =
`S`, `edit.undo` = Ctrl+Z, `edit.redo` = Ctrl+Y / Ctrl+Shift+Z). `Registrar::command`,
`UiRegistry::{register_dynamic, unregister_dynamic, command, commands, group_commands,
menu_commands, menu_children, generation}`. The shell draws registered commands in toolbar group
dropdowns (plus pinned buttons) and in menus (`File/Export` submenus, extra top-level menus);
UI-01's hand-drawn Undo/Redo/File items are unchanged, so the UI-01 snapshots are unchanged.

Done-when: 1. `tests/commands.rs::s_remsh_lists_remesh_first_and_enter_runs_it`. 2. disabled reason
as toolbar tooltip and inline in search; Enter starts on the first enabled row and never runs a
disabled one (`disabled_*` tests). 3. `registry::tests::duplicate_cmd_id_panics_in_debug`.
4. `sketch_scope_beats_global_for_the_same_key`, `single_letters_are_ignored_while_a_text_field_has_focus`,
`shortcut_map_scope_resolution_without_the_app`. 5. `search::tests::search_1000_commands_budget`
(asserted under `AMG_STRICT_TIMING=1`) and criterion bench `benches/search.rs`.
6. `registry::tests::register_dynamic_rejects_ids_without_script_prefix`, plus the
dynamic register/search/run/unregister kittest.

*Deviations:*
- `CmdAction::StartTool` and `AppCx::start_tool` wait for the `Tool` trait (UI-03). `CmdEnv` keeps
  its UI-01 fields. `AppCx::run_command` is deferred to the end of the frame (nested requests up
  to depth 16); `AmalgamApp::request_command` is the host-side equivalent.
- Additive: `CommandDef::alt_shortcuts` (redo has two bindings), `with_*` builders,
  `AppCx::{undo, redo, open_search}`, `AmalgamApp::{request_command, registry_mut, shortcuts,
  set_shortcut_overrides, load_shortcuts_file, search, search_mut, recent_commands}`,
  `AppHarness::run_command`. `AppConfig` is unchanged.
- While a text field has focus only Ctrl/Alt chords fire, and not the editing ones
  (Ctrl+Z/Y/A/C/V/X). The `Viewport` scope is active while the pointer was over the viewport last frame.
- A `shortcuts.toml` with a bad binding is rejected whole (bindings stay as they were); a missing
  file is fine. User overrides bind before defaults, so they win a conflict.
- Search tests use a fixed number of frames: `drive_until_idle` never settles while the search
  text field is focused (cursor blink repaints).
- `toml` (already in `THIRD_PARTY.md`) is now also used by `amg-ui`.

*Leftovers:*
- Search of `DialogRegistration` features (UI-05 registers `feature.<kind>` commands, which search
  already lists) and of document parameters ("opens the Parameters dialog") are not done: the
  Parameters dialog does not exist yet.
- Icons are not drawn (the SVG icon registry is still open); recency is not persisted (UI-11).
- The criterion numbers are in the handoff report.

### 2026-11-12 - UI-10 job panel, toasts, device-lost banner
Landed in `amg-ui`: `notify` (`Notification`, `Notification::from_diagnostic` = `None` for a
cancellation, `Notifications` queue with a skewable clock, `AUTO_HIDE` = 6 s, toast drawing),
`jobs` (private: `JobTracker` for `spawn_tracked`, and the job panel). `AppCx::{notify,
notify_error, spawn_tracked}` and the host-side `AmalgamApp::{notify, notify_error, notifications,
dismiss_notification, advance_notification_clock, spawn_tracked, jobs_panel_open,
set_jobs_panel_open, device_lost, log_dir, set_log_dir, set_folder_opener, open_log_folder}`.
`AmalgamApp::logic` polls the tracker without blocking (before the document poll), so a tracked job's
callback runs on the UI thread with `&mut AppCx`. The status bar's `Jobs: N` is now a toggle for
the panel (bottom right, above the timeline): per root job the label, a progress bar, the latest
message, elapsed time and a Cancel button (`JobSystem::cancel`), children indented under their
parent. While it is open and jobs run it repaints every 100 ms (progress has no notifier); closed,
the idle behaviour is unchanged. Toasts are drawn at the top right: title in the severity colour,
body, `Hint: ...`, a collapsible "Details" (the `amg_core::report` chain), "Copy details"
(`copy_text`), "Open log folder" and "Dismiss". Errors are sticky; info and warnings hide after 6 s
(the queue asks for `request_repaint_after` so an idle app still wakes). The device-lost banner is
now a full-width red panel under the toolbar; nothing is disabled by it. `amg-app` hands the
`DiagGuard` log directory to `set_log_dir`.

Done-when: 1. `tests/jobs_panel.rs::job_panel_shows_progress_and_cancel_resolves_without_an_error_toast`
(a 2 s tracked job: progress and message in the panel, Cancel resolves it as `JobError::Cancelled`,
no toast; click to callback 12 ms here, asserted < 200 ms only under `AMG_STRICT_TIMING=1`) and
`jobs_are_grouped_under_their_parent_and_cancel_reaches_children`. 2.
`failing_job_shows_message_hint_and_report_chain` (`Error::InvalidArgument`: body = user message,
details = `report`) and `toast_shows_hint_and_details_with_the_source_chain` (hint on screen, "Details"
opens the chain; `InvalidArgument` itself has no hint). 3.
`info_toasts_hide_after_six_seconds_errors_stay_until_dismissed` (5.9 s still shown, 6.1 s gone,
the error survives an hour and goes on Dismiss) plus `notify::tests`. 4.
`device_lost_shows_the_banner_and_save_stays_enabled` (a viewport that reports a lost device).
Also `open_log_folder_and_copy_details_buttons`.

*Deviations:*
- `spawn_tracked` needs `J::Error: amg_core::Diagnostic` (every library error has it by
  convention): the tracker turns a failure into a toast before `on_done` runs, and never for a
  cancellation. `on_done` still receives the full `Result`.
- Commit-batch failures are a toast ("Edit failed", the label and the doc error, `report` as
  details) in addition to the status-bar text, which stays because the UI-01 test checks it. The
  toast body is `<label>: <error>`, not `<label> failed: ...`, so the two never match the same
  text query.
- There is no Save command yet (UI-19), so the "Save stays enabled" test registers a `Save` menu
  command of its own. Nothing in the banner path disables commands.
- Additive test support: `AppHarnessBuilder::viewport`. The harness's `drive_until_idle` waits for
  the delayed repaint of a pending auto-hide toast, so tests with info/warning toasts step a fixed
  number of frames instead (as the search tests do).
- A freshly opened panel (an `egui::Area`) is laid out invisibly in its first frame; kittest tests
  settle a few frames before clicking Cancel.

*Leftovers:*
- The "Open log folder" default opens `explorer` / `open` / `xdg-open` on the log directory; only
  the injected opener is tested.
- No toast stacking limit or grouping of repeated messages; no toast hover-pause.
- Re-creating a lost device and a Save command are outside this WP (UI-19 owns Save).

### 2026-11-13 - UI-20 scene adapter
Landed in `amg-ui` (status `Review`):
- **`scene`** (`scene/mod.rs`): the `SceneAdapter`.
  - Methods: `handle_events(events, design, evaluated, jobs)`, `rebuild`, `clear`, `poll`,
    `set_preview`, `set_placement_overlay` (S06 request), `set_highlight` / `highlight`,
    `set_theme`, `set_quality`, and `register_handles` / `release_key` (overlay owners: handle id ->
    `PickTarget`), `raycast`.
  - Queries: `pick_map`, `occurrence_path`, `bodies`, `display_mesh`, `mesh_slot`,
    `instance_slots`, `uploads_of`, `pending`, `stats`, `backend`, `last_key`, `take_failures`.
  - **Keys.** `RenderEntityKey(u32)` is S10's `PickId` (`pick_id()`). Keys are allocated densely
    from 1 and never reused in a session, also not across `DocumentReplaced`. There is one key per
    (occurrence path, body). Sketches and construction geometry get keys too; they are drawn later
    as RND-07 overlays and have no scene mesh.
  - **`PickMap`** maps `(key, Option<RenderSub>)` to `PickTarget` and back (`target`, `render`,
    `body_key`, `body_keys`, `keys`, `generation`).
  - **Events** are coalesced per drain. Each touched body is reconciled against the current
    `Evaluated`: present means upsert, absent means remove. So an add plus a remove (or the
    reverse) in one drain comes out right.
  - **Geometry** is compared with `BodyGeom::same_as`, so an unchanged body is never tessellated
    again.
  - **Instance-only changes:** props and visibility, `ComponentTreeChanged` and
    `OccurrenceTransform/VisibilityChanged` only rewrite instances.
    - Transforms come from `Evaluated::placements()`, or from the placement overlay.
    - A body is visible when it is visible and every occurrence on its path is visible.
    - Opacity goes to `base_color.a`.
  - **Tessellation and upload** run in one job per body (`spawn_fn`, Interactive, label "Display
    body"). It is latest-wins: a newer geometry cancels the job in flight, and the old mesh stays
    drawn meanwhile. When the new upload arrives, `poll` inserts the mesh, repoints the instances
    and removes the old mesh in one op batch, so no frame lacks the body.
  - **Preview:** a body goes on `Layer::Preview` when its `Arc<EvalBody>` in the preview
    `Evaluated` differs from the committed one. It is drawn in the theme's preview colour at
    alpha 0.4 and is not in the pick map. Its upload is kept while the preview `Arc` is unchanged;
    `None` clears the layer.
  - **Highlights:** for each `HighlightLayer` (Hover, Selected, Error, Preview) the adapter keeps
    the targets. It resolves them to `HighlightItem { key, sub: Option<(RenderSubKind, Range32)> }`
    again whenever the pick map changes, so a deleted body drops out. Colours come from the theme
    tokens.
- **`scene::tess`:**
  - `BodyTessellator` (`handles`, `tessellate(geom, quality, ctx) -> DisplayMesh`),
    `DisplayQuality`, `TessellateError`.
  - `DisplayMesh { data: MeshData, face_ranges }`, with `face_of_triangle` and `face_count`.
  - `MeshTessellator` calls `IndexedMesh::render_buffers`, rebased to the centre of the bounds. It
    is flat shaded unless the mesh has vertex normals.
  - Registration: `Registrar::tessellator` and `UiRegistry::{tessellators, tessellator_owners}`.
    The last registered tessellator that handles a variant wins. `builtin` (module `app`, which
    sorts first) registers the Mesh one.
- **`scene::backend`:**
  - Types: `SceneBackend` (`uploader`, `apply(Vec<SceneOp>)`, `as_any`), `MeshUploader`,
    `UploadedMesh`, `SceneOp`, `InstanceDesc`, `MeshSlot` / `InstanceSlot`, `HighlightLayer`,
    `HighlightItem`, `RenderSubKind`, `Range32`.
  - `GpuSceneBackend` drives S10's `RenderScene`, built with `egui(rs)` (over
    `egui_glue::with_renderer`), `shared(gpu, Arc<Mutex<RenderScene>>)` or `new(gpu, access)`.
  - `CpuSceneBackend` needs no GPU. It records meshes, instances and highlights, and counts uploads.
- **`pick`:** `PickTarget`, `PickMode`, `PickTicket`, `PickHitUi`.
- **App:**
  - `AmalgamApp` owns one `SceneAdapter` for the active document. `logic` rebuilds it when the
    active tab changed (`add_document`, `activate`, ...); otherwise it feeds in the drained events.
    It then `poll`s the adapter, which never blocks, and turns failures into an error toast.
  - Additive: `AmalgamApp::{scene, scene_mut, set_scene_backend}`. The default backend is
    `CpuSceneBackend`.
  - `amg-app` installs `GpuSceneBackend::egui`. The RND-01 demo mesh is unchanged and sits next to
    the adapter's instances.
- **`testing`:**
  - `CpuPickService` is real: `request_ray` returns a `PickTicket`, `poll(scene, ticket)` answers
    it once, and `pick(scene, ray, mode)` picks synchronously.
  - It ray-casts the adapter's display meshes: a lazy median-split BVH per mesh and amg-core's
    watertight `ray_triangle`. Each instance gives its nearest hit, front to back; `AllAlongRay`
    keeps them all.
  - New `AppHarness::{picks_mut, poll_pick, pick_ray, scene}`. `drive_until_idle` also waits for
    tessellations in flight.

*Evidence (Done-when; `tests/scene.rs` has 12 tests, plus `benches/scene.rs`):*
1. **Add, edit, delete.**
   `add_edit_delete_three_mesh_bodies_keeps_the_scene_exact_and_skips_unchanged_bodies` runs
   through `AppHarness` with a real recompute. After every step the scene's bodies equal the
   evaluated bodies, each live body has exactly one instance and one mesh, and every instance key
   maps to its body.
   - The upload counter is 3 after adding.
   - It is 4 after editing one feature. The other two bodies keep 1 upload each, the old mesh is
     gone and the key survives.
   - It stays at 4 after the opacity and visibility edits, and after deleting one body.
   - Undo brings the body back as upload 5, with a new key.

   `gpu_scene_adapter_drives_a_real_render_scene` repeats this on a real `RenderScene`, which
   passed on the owner GPU. `instance_count` / `mesh_count` are 3/3, still 3/3 after the edit and
   2/2 after the delete; `gpu_bytes` is 0 after `clear`.
2. **`DocumentReplaced`.** In `document_replaced_rebuilds_the_scene_with_fresh_keys`, re-opening
   the same design (`Document::from_design`) rebuilds the scene: `rebuilds` +1 and 6 uploads. The
   new keys are disjoint from the old ones and all above the old maximum. See also
   `document_replaced_event_rebuilds_without_the_app` and
   `switching_document_tabs_shows_the_active_document_only`.
3. **`PickMap` round trip.** `pick_map_round_trips_every_face_of_a_10k_triangle_body` uses a
   100x50 grid (10 000 triangles). For every triangle, a CPU pick through its centroid returns
   `Body { sub: (Face, i) }`, and `render` / `target` round-trip to `(key, Face i)`. The 10 000
   picks took 12.5 ms in total. See also
   `cpu_pick_service_is_asynchronous_and_select_through_is_depth_ordered` (3 stacked bodies).
4. **Registered tessellator.** In `registered_test_tessellator_displays_a_stub_variant`, a test
   file's `inventory` module registers a tessellator for `BodyGeom::Implicit` (the `SdfGraph`
   stub). It is called once, and the body gets a 12-triangle mesh and is pickable. The mesh body
   next to it still uses `MeshTessellator`.
5. **1 of 100 bodies.** `timing_update_one_changed_body_of_100_under_1ms_ui_thread` uses 100
   bodies and 20 rounds in the debug-opt test profile. One changed body spawns exactly one
   tessellation. UI-thread time (the event frame plus the insert frame) is p50 11.6 us and worst
   18.6 us. The budget is 1 ms, asserted under `AMG_STRICT_TIMING=1`. The criterion bench
   `scene_update_1of100` (release, the same two UI-thread steps) gives a mean of 11.2 us
   [11.0, 11.4].

Further tests:
- `latest_wins_keeps_the_old_mesh_until_the_new_one_arrives`: 3 edits during a gated
  tessellation. 2 are superseded, the old mesh stays drawn throughout, and only the latest is
  uploaded.
- `highlights_resolve_through_the_pick_map_and_follow_the_theme`.
- `preview_bodies_go_to_the_preview_layer_and_are_replaced_as_a_whole`.

*Deviations:*
- `BodyTessellator::tessellate` returns `DisplayMesh` (`MeshData` + `face_ranges`), not the
  provisional `GpuMeshData`. The same job uploads it through the backend's `MeshUploader`.
- **`PickTarget`:**
  - `PickTarget::Body` has `instance: u32` (S06 request).
  - `PickTarget` derives `Hash` but not `Ord`, because `amg_doc::TopoKind` is not `Ord`.
  - The sketch variants are a single `Sketch(SketchId)` until UI-09 draws sketch overlays.
    `SketchEntity` / `Profile` need `amg-sketch` ids, and `amg-ui` does not depend on
    `amg-sketch` yet.
  - A mesh triangle is `(TopoKind::Face, triangle)`. `TopoKind::FaceGroup` has no render sub-id;
    the mesh lane maps groups to triangles.
- **RND-04 gaps:**
  - `RenderSubKind`, `Range32`, `HighlightLayer` and `HighlightItem` live in `amg_ui::scene` until
    RND-04 adds S10's `SubKind` and highlight API.
  - `GpuSceneBackend` stores the highlight lists and colours but cannot draw them yet.
  - The pick id is not written to the instance record (its `w` lanes are reserved for RND-04).
- Each body keeps a CPU copy of its display mesh (`Arc<DisplayMesh>`) for the CPU ray cast
  (select-through). A mesh's BVH is built on its first ray cast, on the calling thread.
- Mesh bodies are uploaded with `closed = false` (double-sided). `IndexedMesh` has no cheap
  watertight flag; section caps (RND-06) will need one.
- Switching document tabs rebuilds the scene and re-uploads everything; there is no per-document
  scene cache.
- `AppConfig` is unchanged, because `idle.rs` builds it as a struct literal. The backend is set
  with `AmalgamApp::set_scene_backend` instead.

*Leftovers:*
- **RND-04:**
  - write `RenderEntityKey` into the instance record;
  - draw the highlight layers;
  - back UI-03's `PickService` with GPU picks. `CpuPickService` and `raycast` stay for
    `AllAlongRay`.
- **UI-22:** the Solid tessellator. Placeholder `Shape`s and `Form` bodies have no tessellator
  yet (counted in `stats().undisplayable`).
- The preview keeps the committed version of a previewed body visible, and does not hide bodies
  that the preview removes.
- No camera fit when bodies arrive: navigation is UI-03/RND-11. In `amg-app`, document bodies can
  therefore be outside the fixed home view.

### 2026-11-15 - UI-19 file operations and recovery
Landed in `amg-ui` (status `Review`):
- **`file`** (new framework module, `file.rs`):
  - `FileDialogBackend` (`show(&FileDialogRequest) -> Option<PathBuf>`, blocking) with
    `FileDialogRequest { kind: FileDialogKind::{Open, Save}, title, filters: Vec<FileFilter>,
    directory, file_name }`. `RfdDialogs` is the `rfd` 0.17.2 backend; `RfdDialogs::with_parent(&W)`
    (`W: HasWindowHandle + HasDisplayHandle`, e.g. eframe's `CreationContext`) keeps rfd's parent
    handles, so every dialog is modal to the main window. Each dialog runs on a fresh thread
    `amg-file-dialog` (its own COM apartment); the answer comes back over an `mpsc` channel plus
    `request_repaint` and is polled in `logic`. One dialog at a time.
  - `FileOp::{New, Open, OpenPath, Save, SaveAs, SaveCompact, Close}`, queued with the additive
    `AppCx::file(op)` (end of frame, `ShellRequest::File`), or run by the host with
    `AmalgamApp::file_op(doc, op)`.
  - **Save** (`AmalgamApp::save_document(id, save_as, compact)`): a document without a path (or
    Save As) asks first; the path gets `.amg` appended unless it ends in it. The save is an
    `amg_jobs` job ("Save <name>", progress and Cancel in the job panel) running
    `amg_doc::io::save_in_job` on `Document::snapshot()`; normal saves pass
    `Document::cache_export()`, Save Compact passes `cache: None`. When the job succeeds the tab
    gets the path and the file stem as name, the file goes to Open Recent, and
    `History::mark_saved` runs only if the design revision is still the one that was saved
    (edits made during the job keep the document dirty). Failures are error toasts with the
    `report` chain; a cancellation shows nothing.
  - **Open** (`open_path`): a job runs `load_in_job` + `LoadedDocument::into_document` (lazy cache
    seed) + `mark_saved`, so the UI thread only adds the tab. An already open path activates its
    tab. Load warnings become warning toasts; a bad file is an error toast and opens nothing.
  - **Open Recent**: `RecentFiles` (10, most recent first, deduplicated), persisted as
    `%APPDATA%\Amalgam\recent.json` (`default_recent_path`, `AmalgamApp::set_recent_path`;
    in memory only by default).
  - **Close prompt**: `logic` checks `viewport().close_requested()`. With dirty documents it sends
    `ViewportCommand::CancelClose` and a `Modal` ("Unsaved changes", Save / Don't save / Cancel,
    Esc or backdrop = Cancel) asks for each dirty document in tab order, activating it. Save
    saves (via the dialog if needed) and continues when the job succeeded; a cancelled dialog or
    a failed save stops the close. When all are resolved the app sends `ViewportCommand::Close`
    and lets the next close request through (`CloseState::{Idle, Prompting, Saving, Closing}`,
    `AmalgamApp::close_state`). `file.close` (Close document) uses the same flow for one tab;
    the app always keeps one document.
  - **Recovery** (`set_recovery(Some(RecoveryConfig))`): every open and later added document gets
    `RecoverySession::start(cfg, design uuid)` + `attach`, so each applied `CommitBatch` (one
    `Document::transaction`) is one journal record through `HistoryObserver::on_commit`; undo and
    redo are journaled by DOC-14 itself. `logic` calls `tick` for documents that are dirty or
    already have a snapshot. One FND-07 recovery hook (`amg-ui.recovery`) flushes all live
    sessions (weak list, `try_lock` only) and reports the recovery folder.
    `check_recovery()` lists `find_recoverable` (skipping the app's own documents), runs
    `recover_with_report` per candidate on a job and opens the "Recover unsaved work" `Modal`:
    per document its name, age and replayed edit count, Open recovered / Discard (a background
    job calls `RecoveryCandidate::discard`), and Later (keeps the folders; offered again next
    start). Open recovered adds "<name> (recovered)" (dirty), whose new session takes over the
    folder. Closing a tab finishes its session on a helper thread; `AmalgamApp::shutdown` (the
    host's `on_exit`) finishes all of them, so a clean exit leaves nothing to recover.
    `check_recovery` also sweeps, on a background job, unlocked folders that never got a
    snapshot (a crash before the first edit), by taking them over with `RecoverySession::start`
    and `finish`; folders with a snapshot or a live lock are left alone.
  - Window title `"<name>[ *] — Amalgam"` (`ViewportCommand::Title`, sent on change).
  - Defaults: `default_recovery_dir()` = `%LOCALAPPDATA%\Amalgam\recovery` on Windows
    (`$XDG_STATE_HOME/amalgam/recovery` on Linux).
- **`builtin`** registers `file.new` "New document" (Ctrl+N), `file.open` "Open…" (Ctrl+O),
  `file.save` "Save document" (Ctrl+S), `file.save_as` "Save As…" (Ctrl+Shift+S),
  `file.save_compact` "Save Compact", `file.close` "Close document" (Ctrl+W), all
  `Placement::Menu { path: "File" }`.
- **`shell`**: the File menu draws the registered File commands (replacing UI-01's hand-drawn
  "New document"), the Open Recent submenu, and the **Import** / **Export** slots: each is a
  submenu that shows "No formats registered yet" until a module registers a command under
  `File/Import` or `File/Export` (IO-01 then fills it with no framework change). Other `File/…`
  submenus are drawn as before. The close prompt and the recovery dialog are drawn after the
  toasts.
- **`app`**: `DocTab::{path, recovery}`; `new_document` marks the empty document saved, so an
  untouched Untitled closes without a prompt (`Document::new` has no saved marker). Tracing spans
  `amg_ui.frame` (`AmalgamApp::frame`), `amg_ui.logic` and `amg_ui.ui` (debug level).
- **`testing`**: `FakeFileDialog` (queued answers, an empty queue cancels; records requests).
  `drive_until_idle` also waits while `AmalgamApp::file_busy()` (dialog, save, open or recovery
  load pending).
- **`amg-app`**: installs `RfdDialogs::with_parent(cc)`, `set_recent_path(default_recent_path())`,
  `set_recovery` + `check_recovery` before the first frame (`--recovery-dir DIR`,
  `--no-recovery`), opens a `FILE.amg` argument, and calls `shutdown()` in `on_exit`.

*Evidence (Done-when; `tests/file_ops.rs`, 12 tests):*
1. **Save → reopen.** `save_then_reopen_gives_an_equal_design_json`: two `AddParam` batches, then
   File ▸ Save document (clicked in the menu). The fake save dialog is asked once (kind `Save`,
   filter `amg`) and answers `bracket` with no extension; `bracket.amg` is written, the tab is
   renamed "bracket", gets the path, and the document is clean. A later Save with a path shows no
   dialog. Close document (clean, no prompt) leaves a fresh Untitled; File ▸ Open… (fake answers
   the path) opens a clean "bracket" tab with the three parameters, and opening the same path again
   only activates it. Save As `copy.amg` of the reopened document gives a `design.json` that is
   **byte-equal** to the first file's, and so does Save Compact. Open Recent then lists
   `copy.amg` first. Also `cancelled_dialog_and_failed_open_change_nothing` (a cancelled save keeps
   the document dirty and pathless; a non-Amalgam file opens no tab and shows "Cannot open the
   file"), `open_recent_persists_and_reopens` (recent.json survives a new app; File ▸ Open Recent ▸
   r.amg opens it) and `import_and_export_slots_are_in_the_file_menu`.
2. **Dirty close prompt.** `closing_a_dirty_document_prompts_and_cancel_keeps_the_app_open`: a
   `ViewportEvent::Close` in the kittest input makes that frame send `ViewportCommand::CancelClose`
   (and no `Close`); the "Unsaved changes" modal asks `Save changes to “Untitled 1” before
   closing?` (`CloseState::Prompting`). Cancel: no `Close` in the next 5 frames, the state is
   `Idle`, the modal is gone, the document is still open and dirty. Asking again and choosing
   Don't save sends `Close` (`CloseState::Closing`), after which a close request is let through.
   Also `save_in_the_close_prompt_saves_then_closes` (two dirty tabs: Save writes `one.amg` through
   the dialog and moves on to the second; cancelling the second's dialog stops the close; Don't
   save then closes), `clean_documents_close_without_a_prompt` and
   `closing_a_dirty_tab_prompts_and_dont_save_removes_it` (Close document on a dirty tab).
3. **Kill test.** `kill_test_recovery_dialog_offers_and_restores_the_document` re-runs the test
   binary as a child (`AMG_UI19_KILL_CHILD`): an `AppHarness` with recovery on adds a "Bracket"
   document, commits 5 `CommitBatch`es through the app, saves the expected state to
   `expected.amg`, flushes the journal and calls `process::abort()`. The parent asserts the child
   failed, starts a new app on the same recovery folder: `check_recovery()` returns 1, the "Recover
   unsaved work" dialog names "Bracket"; "Open recovered" opens "Bracket (recovered)", dirty, with
   p0..p4, a new recovery session, and its `design.json` is **byte-equal** to `expected.amg`'s. A
   live session is not offered (`find_recoverable` empty), and after `shutdown()` the recovery
   folder is empty (the child's untouched Untitled folder, which never had a snapshot, is swept).
   Also `recovery_discard_deletes_and_later_keeps_the_data` ("1 edit(s)" shown; Later keeps the
   folder and it is offered again; Discard deletes it) and `every_commit_batch_is_journaled` (a
   two-command batch is one journal record; the last record is `"label":"Add c"`).
4. **50 MB open.** `timing_opening_a_50mb_file_never_blocks_a_frame`: a 55 284 170-byte `.amg`
   (one feature with 70 M base64 characters in `design.json`), opened through File ▸ Open… with
   recovery on, stepping frames like a continuously repainting event loop. Every `amg_ui.frame`
   span is timed by a test `tracing_subscriber::Layer`. Three runs with `AMG_STRICT_TIMING=1`
   (debug test profile, opt-level 1, machine shared with 3 other workers): the open took
   585-598 ms over 227-232 frames; UI-thread frame p50 191-195 µs, **max 2.1-2.8 ms** (budget
   16 ms, asserted only under `AMG_STRICT_TIMING=1`).

Unit tests: `file::tests::recent_files_dedupe_cap_and_round_trip`,
`file::tests::save_paths_get_the_amg_extension`.

*Deviations:*
- The Save command is labelled "Save document", not "Save": the close prompt's button is "Save",
  and UI-10's test registers its own `Save` menu command that `get_by_label("Save")` must find
  uniquely (existing tests are unchanged).
- Save, open and recovery jobs are tracked by the file module itself (`JobHandle::try_take` in
  `logic`), not through `AppCx::spawn_tracked`: their callbacks need the whole app (any tab, the
  close flow), and `spawn_tracked` callbacks only see the active document's `AppCx`. They still
  appear in the job panel with progress and Cancel.
- The kill test runs the real `AmalgamApp` through `AppHarness` in a child process (the test binary
  re-run with `AMG_UI19_KILL_CHILD`), not the `amg-app` binary: `amg-app` needs a window and a GPU.
  The child calls `flush_recovery()` before `process::abort()` so the test does not depend on the
  journal writer thread's timing (the panic hook does the same flush on a crash).
- "50 MB file" = a `design.json` feature payload (70 M pseudo-random base64 characters, which
  deflates to ~53 MB on disk); the frame time comes from the `amg_ui.frame` tracing span through a
  test `tracing_subscriber::Layer`, asserted < 16 ms only under `AMG_STRICT_TIMING=1`.
- A successful save does not reset the recovery folder (DOC-14 has no "saved" call), so a crash
  right after a save still offers the document; the recovered copy equals the saved one.
- kittest clicks in a just-opened `Modal` wait a few frames: its first frame is a layout pass at
  another position, so an immediate click lands on the backdrop (= Cancel), as UI-10 saw for
  `Area`s.
- `AppConfig` is unchanged (`idle.rs` builds it as a struct literal); the backend, recent path and
  recovery are setters.

*Leftovers:*
- Drag-and-drop of `.amg` files, a document-tab close button (only File ▸ Close document / Ctrl+W),
  and a dirty marker on the tab label (only in the window title) are not done.
- Open Recent keeps paths that no longer exist (opening one shows the error toast).
- The `rfd` dialog itself (and the `amg-app` close path through eframe) was not exercised by an
  automated test; `RfdDialogs` is covered only by compiling. The owner should try File ▸ Open… /
  Save As… and closing a dirty window once.
- The recovery dialog shows a document's age, not its file path (DOC-14 candidates carry no path).

### 2026-11-17 - UI-03 tools, input routing, picking, navigation, previews

*Landed in `amg-ui` (status `Review`):*
- **`tool`**: `Tool` (as §A, without `overlays`), `ToolInit`, `InputResult`, `FinishReason`,
  `HoverHit`, `ToolCx` (`app`, `env`, `request_finish`, `hover`, `camera`, `preview`,
  `selection(_mut)`, `set_cursor`, plus `pick_filter`, `pick_point`, `poll_pick`), the
  per-document router and the click-only `SelectTool` (`app.select`).
  - Priority: key and text events go to the active tool before the `ShortcutMap`
    (`AmalgamApp::route_keys`, skipped while a text field has focus or search is open). An
    ignored `Esc` cancels the tool (no tool: clears the selection), an ignored `Enter` commits.
    Consumed presses are removed from the frame input, so shortcuts do not see them.
  - Pointer events: active tool → `Navigator` → `SelectTool` (only without an active tool). A
    drag goes to whoever takes its `DragStart` and stays there until release. A press counts
    only inside the viewport rect, outside the backend's own widgets (view cube, nav bar) and
    where no other egui layer covers it.
  - Replace: `CmdAction::StartTool(fn(&ToolInit) -> Box<dyn Tool>)` (new variant) finishes the
    active tool with `Replaced`, or `Commit` if `commit_on_replace`. A commit's `CommitBatch` is
    submitted and becomes one undo entry at the end of the frame. `AppCx::start_tool(id)` queues
    the command.
  - The tool's dialog is an `egui::Window` at the viewport's top right (skipped once
    `dialog_ui` returns false). The status bar shows `status_hint` (or the label).
  - Highlights: the hovered body goes to `HighlightLayer::Hover`, the selection to `Selected`
    (whole bodies; face-level priority is UI-04).
- **`input`**: `ViewportEvent` (as §A), `InputSynth` (egui raw events → Hover / Press /
  Release / Click with double-click count / DragStart past 3 pt / Drag / DragEnd / Scroll in
  points, a line = 50 pt), `key_events`, `CameraView` (`project`, `ray`, `world_per_point`,
  plus `camera`, `size_px`, `pixels_per_point`, `to_px`, `from_px`).
- **`nav`**: `NavPreset::{Fusion, SolidWorks, Blender, Custom}`, `NavAction`, `NavBindings`
  (`action_for`), `Navigator` (drives RND-11's `NavController`: `begin`/`drag`/`end`, `zoom`,
  `tick`; view-cube / nav-bar actions through `egui_glue::apply_camera_action`; `StartMode`
  arms the next primary drag), `CameraControl` implemented over the document camera
  (`AppCx::camera`), `home_camera`. No camera maths of its own. Fusion: Shift+MMB orbit about
  the hover pick point (else the scene centre), MMB pan with the grabbed point, wheel zoom to the
  cursor. SolidWorks and Blender tables exist; UI-11 adds the page and the editor.
- **`pick`**: `PickService` (`request_point`, `request_region`, `poll`, plus `begin_frame`,
  `set_view`, `pending`), `RegionMode`, `PickFrame`; `CpuPickService` moved here from `testing`
  (re-exported there, UI-20 API unchanged) and implements the trait: rays from the frame camera,
  answered in the next frame's `begin_frame`; rectangles by projected display-mesh vertices.
  `GpuPickService` over `Renderer::request_pick` / `poll_picks` (`new(RendererAccess, ViewId)`,
  `egui(rs, view)`), mapping hits through the `PickMap`; `AllAlongRay` goes to its inner CPU
  service. Hover: one point pick per frame with pointer motion, the newest answer wins.
- **`preview`**: `PreviewDriver` (`set`, `clear`, `state`, `update`, `spawned`, `is_busy`,
  `shown`, `set_debounce`, `set_clock`), `PreviewState`, `DEFAULT_PREVIEW_DEBOUNCE` = 150 ms. One
  per document (`ToolCx::preview`, `AmalgamApp::preview_driver`); results go to
  `SceneAdapter::set_preview`. Never touches the undo stack (it wraps `Document::preview`).
- **`select`**: `EntityFilter` (local bit set, see deviations), `SelItem`
  (`from_target`), `Selection` (as §A, plus `len`, `is_empty`).
- **App**: `DocTab` gains `camera`, `selection` and crate-private router / navigator state;
  `CmdEnv` gains `selection`, `hovered`, `active_tool`; `AppCx::{selection, selection_mut,
  camera, start_tool}`; `AmalgamApp::{active_tool, finish_tool, hover, preview_driver,
  input_busy, pick_service, set_pick_service, navigator, set_nav_preset, camera_view}`.
  `spawn_tracked` was already there (UI-10). `logic` runs the router's pick / hover / update /
  preview / transition / highlight step after the scene poll.
- **Viewport**: `ViewportFrame::camera`; `ViewportBackend` gains defaulted
  `take_camera_actions`, `input_exclusions`, `pick_service` and `after_frame`.
  `RenderViewport` writes the document camera into its S10 view every frame, turns on the view
  cube (`ViewCube::for_scale(ppp)`), paints its labels, forwards `egui_glue::view_cube` and
  `nav_bar` actions (visual style applied to the view directly) and supplies a
  `GpuPickService`. `NullViewport` is unchanged (UI-01 snapshots unchanged).
- **`scene`**: `SceneAdapter::bounds` (cached per mesh) and `region` (CPU rectangle query).
- **`testing`**: `AppHarness::{camera_view, world_to_screen, hover_at, button, click_at,
  click_world, drag, key}`; `drive_until_idle` also waits for `input_busy`.

*Evidence (Done-when; `tests/tools.rs`, 7 tests; RTX 5060 Ti / DX12):*
1. `scripted_tool_sees_events_in_order_and_esc_enter_and_replace_finish_it`: hover, press,
   drag start, drag, release, drag end, press, release, click, key down, key up arrive in that
   order. Esc gives `Cancel` and no undo entry. Enter gives `Commit` and exactly one undo entry
   labelled "Script commit". Starting another command gives `Replaced` (nothing committed), and
   a `commit_on_replace` tool commits instead.
2. `middle_drag_navigates_while_a_tool_that_ignores_it_is_active`: the tool is offered the
   `DragStart` and ignores it. Navigation then owns the drag (the tool sees no further drag
   events). Shift+MMB orbits: target fixed, distance kept, view turned. MMB pans: eye and
   target move together. Two wheel lines zoom by exactly 0.9². A primary drag leaves the camera
   unchanged. Fusion maps plain MMB to pan; the orbit is Shift+MMB (spec §6).
3. `hover_highlights_within_two_frames_and_click_selects_with_cpu_picking` (CpuPickService) and
   `gpu::gpu_hover_highlights_within_two_frames_with_s10_picking` (S10 picking through
   `GpuPickService`, a test viewport that encodes and submits after each frame as egui's paint
   callback does). Both put the highlight on in the frame after the event frame, so 2 frames in
   total (5 and 10 rounds, on and off the body). The GPU run took 20 encodes. Clicks select the
   body and set the Selected layer, and a click on empty space or Esc clears it.
4. `twenty_rapid_preview_sets_compute_one_preview_outside_the_undo_stack`: 20 `set` calls with a
   frame each inside the debounce compute exactly one preview (the test feature's counter is 1).
   It is shown on the preview layer; there is no undo entry and no timeline node; `clear`
   removes it. `a_newer_preview_cancels_the_one_in_flight`: the newest command's result wins.
5. `timing_no_ui_frame_over_16ms_while_a_two_second_job_runs`: a 2 s tracked job, a tool
   active, hover sweeps, Shift+MMB orbits and wheel zooms. 791 frames in 2.006 s; the
   `amg_ui.frame` span was p50 0.20 ms, p99 0.34 ms, max 0.41 ms. The 16 ms limit is asserted
   only under `AMG_STRICT_TIMING=1`.

*Deviations:*
- `EntityFilter` lives in `amg_ui::select`: `amg_features::dialog` (S04 SOL-01) does not exist
  yet. It becomes a re-export when it lands.
- `Tool::overlays` / `OverlayCx` / `OverlayList` are not in the trait yet. There is no
  framework overlay module; UI-06 / UI-09 add the method with a default (additive).
- `ToolCx::camera` returns `Option<&CameraView>` (`None` before the first viewport frame).
  `AppCx::camera` returns `impl CameraControl + '_`, not `&mut dyn CameraControl`.
- `NavBindings::drags` uses `NavButton` / `NavMods`, because egui's types are not serde-enabled
  in the workspace. Modifiers match exactly (Shift, Ctrl/Cmd, Alt).
- The pick radius is a hint. S10 uses `RendererConfig::pick_radius_px`; the CPU service casts
  the ray through the cursor (faces only). Region hits carry no world point. The CPU region
  test is by projected vertices and ignores occlusion.
- `SelectTool` selects whole bodies only (`SelItem::from_target` maps whole bodies and
  construction geometry). Sub-entity refs need `BodyNaming` (UI-04).
- `CmdEnv::hovered` / `active_tool` read as `None` inside tool callbacks: the router holds the
  state then. Tools use `ToolCx::hover`.
- Manipulators (priority slot between popup and tool) come with UI-06.

*Leftovers:*
- The owner should try navigation in `amg-app` once: the view cube, nav bar and Fusion
  bindings on the RND-01 demo mesh. These were tested headless and through a test viewport,
  not through eframe.
- No pivot marker during orbit (RND-11 leftover). View-cube hover (`ViewCube::hover`) is still
  not set.
- The preview debounce is not a preference yet (UI-11).

### 2026-11-17 - UI-04 selection system

*Landed in `amg-ui` (status `Review`):*
- **`select`** (now `select.rs` + `select/`):
  - `EntityFilter` gains `without`, `of_topo`; `FILTER_GROUPS` (the bar's seven groups:
    Bodies, Faces (+ mesh face groups), Edges, Vertices, Sketches (all four sketch kinds),
    Construction, Components). `HANDLE` is never switched off by the bar.
  - `SelectPriority::{Default, Body, Face, Edge, Component}` (serde) with `label` and
    `pick_filter` (Face leaves edges/vertices out of the pick, Edge leaves bodies/faces/vertices
    out). `Selection` gains `priority` / `set_priority`, `extend` and `revalidate`.
  - `item_for_hit(hit, filter, priority, eval, scene) -> Option<SelItem>`: Default = whole body
    when the filter has bodies, else the sub-entity the filter admits, else (mesh edge) its
    triangle; Body = whole body; Face = the face (a mesh edge `e` lifts to triangle `e / 3`);
    Edge = edges only; Component = `PickTarget::Occurrence(first occurrence of the instance's
    path)` (root bodies fall back to Body).
  - `SelItem::resolve(target, eval)` builds `EntityRef::Topo` for sub-entities.
    `target_for_entity(design, eval, scene, &EntityRef)` maps a stored ref back (first displayed
    instance).
  - `select::naming` (provisional S01 R1/R2, see deviations): `topo_ref_at`, `resolve_topo`,
    `mesh_fingerprint`, `mesh_edge`, role tags `ui.mesh.{triangle,edge,vertex}`.
  - `select::sets`: `SELECTION_SETS_KIND = "ui.selection_sets"` (version 1, `data = {"sets":
    [{name, items: [EntityRef]}]}`, one record per design), `SelectionSet`, `selection_sets`,
    `selection_set`, `sets_key`, `next_set_name` ("Selection Set N"), `save_set_command`
    (`AddExt` for the first set, `SetExt` after), `remove_set_command`, `save_selection(cx,
    name)`. `AmalgamApp::restore_selection_set(name) -> Option<usize /*skipped*/>` and
    `AppCx::restore_selection_set(name)` (end of frame) for UI-07.
  - `select::bar`: the filter bar (group toggles, "Reset filter", priority Auto / Body / Face /
    Edge / Component), drawn from `shell::draw`.
  - `SelectTool` moved here (`tool::SelectTool` / `tool::SELECT_TOOL` re-export it): click /
    Ctrl / Shift / Cmd rules as before but through `item_for_hit`; primary drag = box (left to
    right `RegionMode::Window`, right to left `Crossing`, through `PickService::request_region`,
    i.e. S10's RND-04 rectangles in the app; plain replaces, Ctrl/Shift/Cmd adds); long press
    (`LONG_PRESS` = 500 ms, `LONG_PRESS_SLOP_PT` = 4) = select-through (`AllAlongRay`, deduped,
    depth order) shown as a popup list; its release does not click. `SelectThrough`,
    `SelectThroughEntry`, `AmalgamApp::{select_through, choose_select_through}`. Esc closes the
    list before it clears the selection.
- **`tool`** (additive): `ToolCx::{pick_region, item_for, scene}`; the effective filter for the
  select tool also intersects the priority's filter; hover items go through `item_for_hit`
  (face hover under a faces-only filter); an `Occurrence` target highlights every body seen
  through it; the box is painted on a foreground layer (window solid, crossing dashed).
- **`app`**: every drained event batch revalidates that document's selection (`DocumentReplaced`
  clears it; body / construction / sketch / component-tree events drop refs that no longer
  resolve, and re-index a moved named sub-entity). `AppCx::request_repaint_after` (additive).
  `tool_parts` also returns the scene.
- **`pick`**: `CpuPickService` point picks report a mesh triangle edge (`3 × tri + corner`) or
  mesh vertex within the pick radius of the hit point, ahead of its face, when the filter has
  `EDGE` / `VERTEX`. **`scene`**: `PickMap::instances_of`, `SceneAdapter::occurrence_bodies`,
  crate-private `mesh_triangle`.
- **Commands** (`app` module): `select.save_set` "Save selection set" (disabled with an empty
  selection) and `select.clear` "Clear selection"; no placement (search only).

*Evidence (Done-when; `tests/selection.rs`, 5 kittest tests on CpuPickService, plus 4 unit tests
in `select` / `select::naming`):*
1. `window_box_picks_enclosed_bodies_and_crossing_box_also_touching_ones`: 3 mesh bodies
   (4 × 4 mm at x = 0, 6, 12). A box over x ∈ [−1, 8] dragged left → right selects only body 0;
   the same box right → left selects bodies 0 and 1; the Selected layer has 1 then 2 items.
   Shift+box adds body 2, a plain box replaces; Ctrl+click toggles on/off, click replaces, click
   on empty space clears.
2. `faces_only_filter_turns_a_click_on_an_edge_into_the_adjacent_face`: the filter is set by
   clicking the bar's toggles. With Edges only, a click on the midpoint of a grid diagonal
   selects that edge (`TopoRef` kind Edge, endpoints checked). With Faces only, the same click
   selects a face whose triangle contains both edge endpoints (`TopoRef` kind Face, highlighted).
   Face priority with every kind does the same, Body priority selects the body, "Reset filter"
   restores ALL / Auto.
3. `long_press_lists_all_three_stacked_bodies_in_depth_order`: 3 stacked bodies (z = 0, 4, 8).
   A press held 500 ms opens the list with the bodies z = 8, 4, 0, depths strictly increasing;
   the release does not change the selection; every entry is found by label and clicking the
   middle one selects it and closes the list; Esc closes a reopened list and keeps the
   selection.
4. `selection_set_round_trips_through_save_and_reload_and_undo_removes_it`: a whole body plus a
   face (face priority) saved by `select.save_set` is one undo entry; undo removes the set and
   its `ui.selection_sets` record, redo restores it. Save As → close → open the `.amg`: restoring
   "Selection Set 1" skips nothing, the `EntityRef`s equal the saved ones and both are
   highlighted.
5. `deleting_a_selected_body_drops_it_from_the_selection_without_error`: of 3 boxed bodies plus a
   face of body 1, deleting body 1's feature leaves exactly bodies 0 and 2 (and 2 highlights), no
   error toast; undo brings the body back without reselecting it; replacing the document clears
   the selection.

All 41 `amg-ui` lib + selection tests pass; the full `amg-ui` suite (existing tests unchanged)
runs 105 tests, 105 passed, in 12.6 s.

*Deviations:*
- **S01 R1/R2 are provisional, in `amg-ui`** (`select::naming`), since DOC-06 is not done: a
  sub-entity uses the body's `BodyNaming` name when it has one (exact name match on resolve);
  otherwise, for mesh bodies only, a `NameSeg::Generated { feature: body.origin, role:
  ui.mesh.triangle|edge|vertex + index }` name plus a `GeomFingerprint` of the element. Such a
  ref resolves only while the element at that index has the same fingerprint (never a silent
  wrong match). B-rep sub-entities without naming are not selectable yet. To be replaced by
  `naming::topo_ref_at` / `Evaluated::resolve_for_display` when DOC-06 lands.
- **Component selection:** `EntityRef` has no occurrence variant, so a component pick is stored
  as `EntityRef::InContext { path: [occurrence], target: Body(picked body) }`. S06's
  `InContext` wrapping of sub-entities outside the active component is not done.
- **Filter bar position:** top centre of the viewport, not top right (the view cube and the tool
  dialog occupy the top-right corner). It shows while the pointer is over the viewport, or
  always while the filter / priority is not the default; so the UI-01 snapshots are unchanged.
- **CPU edges / vertices** exist only for mesh bodies (triangle edges; a shared edge has one id
  per adjacent triangle). On the GPU path S10 draws edge ids only for edge polylines, so mesh
  triangle edges are not GPU-pickable; B-rep edges come with UI-22.
- **Box selection with a faces-only filter** selects nothing on the CPU path (its region pick is
  per body); S10's rectangle pick returns sub-ids, which `item_for_hit` lifts per priority.
- The select-through list is egui-drawn at the press point; "within 3 px" is the pick radius
  hint (the CPU service casts the ray through the point).

*Leftovers:*
- UI-07 lists the sets under "Selection Sets" (rename / delete through `remove_set_command`,
  restore through `AppCx::restore_selection_set`).
- S05 request "a hook so a tool can expand a pick (smart region, edge chain)" and the surface-point
  kind are not done; mesh face groups have no render sub-id (filter bit only).
- Owner check in `amg-app`: box selection and the filter bar with GPU picking (tested headless on
  CpuPickService only).

### 2026-11-18 - UI-15 dialog widgets

*Landed in `amg-ui` (status `Review`):*
- **`widgets`** (new module, `widgets.rs` + `widgets/{expr,selection,fields,frame}.rs`):
  - `format_quantity(q, &UnitPrefs)` and `display_unit(dim, &UnitPrefs)`: `DISPLAY_PRECISION` =
    2 decimals, untrimmed (`15.00 mm`), in the document length unit (its square / cube for
    areas / volumes), degrees for angles, kg for masses, else the dimension's canonical table
    unit; dimensionless values print the bare number.
  - `ExprInput` (as §A: `new(src, expected, params, prefs)`, `bound_param`, `nudge_step`,
    `show`; plus `label`, `id_salt`, `desired_width`). Every frame it runs
    `check_expr(src, dim, params, prefs)`, which wraps `ParamTable::try_eval`: the error span is
    drawn underlined in the error colour (a `TextEdit` layouter), the message is shown under
    the field and as the field's tooltip together with the `Diagnostic` hint. Errors without a
    span (`WrongDimension`, `Cycle`) underline the whole trimmed expression. A `,` between two
    digits outside parentheses is reported as amg-core's `ParseQuantityError::DecimalComma`
    (message "use `.` as decimal separator", hint "Write 12.5, not 12,5.", span = the comma;
    QUESTIONS #11). The value is shown next to the field (`d3 = 20.00 mm` when bound).
  - Autocomplete (`completions(src, cursor, params)`): the word before the cursor (`12mm`
    completes `mm`); every parameter starting with it (by name) first, then up to 12 functions,
    constants and unit names (`expr::reserved_names`). A list under the field; Up/Down picks,
    Tab (or a click) replaces the whole word (functions get `(`), Esc closes the list.
  - Keys: `Enter` / focus loss commit (`ExprResponse::committed`, reported once: the focus
    change is tracked in the field state, not with `Response::lost_focus`, which a repeated pass
    reports twice); `Esc` restores the text from
    focus time without committing; Up/Down without a list nudge a single literal by the step
    in the display unit (`12 mm` → `13 mm`). `ExprResponse` gains `changed`, `error_span`,
    `message`, `hint` and `is_valid()`.
  - `SelectionField` (as §A, plus `mirror(&mut Selection)`): label (clicking it activates the
    field), "N selected" button (shown pressed while active), "Clear <label>" button, a "needs …" warning outside the limits
    (`selection_count_valid`). Only one field is active app-wide: the active field (id, filter,
    pass) is recorded in egui memory; a click or a rising `active` flag (auto-focus) claims it,
    and the other fields turn their flag off on their next draw. With `mirror`, activation sets
    the document `Selection` to the field's items and, while active, the field takes the
    selected items its filter admits (newest `max`), so UI-04's select rules feed it (the tool
    forwards its events to a `SelectTool`).
  - `ChoiceField` (dropdown, or `.buttons()` for a row of icon buttons), `toggle`,
    `IntField` (`range`), `flip` (`⇄ label`, pressed while flipped).
  - `DialogFrame` (as §A: `new`, `banner(Option<&Health>)`, `ok_enabled(Enablement)`,
    `show(ctx, body)`; plus `help(url)` and `show_inside(ui, body)` for tool dialogs, which the
    router already puts in a titled window) and `group(ui, title, body)` (collapsible, open by
    default). A warning / error health is a bordered banner; a disabled OK shows its reason
    next to the buttons and as its tooltip. `DialogAction::{None, Ok, Cancel}`.
  - Labels reach AccessKit through `labelled_by` (text, combo and drag fields, the selection
    button) or the widget's own text / `widget_info`.
- **`tool`** (lane D, additive): after the tool dialog is drawn, `active_selection_filter(ctx)`
  is stored in the router frame and intersected into the active tool's effective pick filter
  (`tool.pick_filter() ∩ filter bar ∩ active selection field`), so hover, clicks and boxes only
  pick what the active field accepts. Cleared when the tool finishes.

*Evidence (Done-when; `tests/widgets.rs`, 6 kittest tests, plus unit tests in `widgets` /
`widgets::{expr, selection}`):*
1. `length_field_shows_the_value_and_a_unit_mismatch_disables_ok`: with `d1 = 10 mm`, typing
   `d1 + 5` shows `15.00 mm` and OK is enabled. `12 kg` gives `WrongDimension`, underlines
   0..5, shows "unit mismatch: expected mm, got t", and OK is disabled (AccessKit) with the
   reason shown; clicking it does nothing. `2 * (d1 + 3 kg)` underlines exactly `(d1 + 3 kg)` (amg-doc's span of the
   parenthesised operation).
2. `typing_d_offers_every_d_parameter_and_tab_completes`: `d` lists `d1` and `depth` (not
   `width`) plus units such as `deg`; Down + Tab completes `depth` (focus kept, `3.00 mm`
   shown, list closed); `depth * 2 + wi` + Tab gives `depth * 2 + width` = `46.00 mm`.
3. `decimal_comma_shows_the_hint`: `12,5` underlines the comma (2..3), shows "Write 12.5, not
   12,5." and the decimal-separator message, OK disabled.
4. `only_one_selection_field_is_active_and_it_sets_the_pick_filter` (real app, `AppHarness`,
   CpuPickService): a tool with "Target faces" (auto-focused) and "Target body" (limit 1)
   fields. The tool's effective filter is FACE; clicking "Target body" makes it the only active
   field and the filter BODY; a viewport click (forwarded to UI-04's `SelectTool`) puts the
   whole body in it ("1 selected"); clicking "Target faces" again empties the selection, keeps
   the body field's item, and the next click selects a mesh face into the faces field.
5. `every_widget_is_queryable_by_label`: Distance, Profile, Clear Profile, Extent, Direction
   buttons, Chain faces, Count, Flip, Options, Help, OK, Cancel, the banner, the OK reason and
   "0 selected" are found by label, and clicking the label nodes drives the widgets (a click on
   the "Profile" label activates the field).
- Also `enter_commits_esc_reverts_and_arrows_nudge` (bound display `d1 = 12.00 mm`, Up/Down,
  Esc revert without commit, Enter commits once).

*Deviations:*
- `WrongDimension` has no span in S01, so the whole expression is underlined (the result as a
  whole has the wrong unit). No `amg-doc` change.
- The DecimalComma check is UI-side: the expression parser reads `12,5` as an argument-list
  comma (a parse error), and `check_expr` reports amg-core's `DecimalComma` instead.
- Nudge applies to single literals only; on an expression Up/Down does nothing (rewriting it
  as a number would lose the parameter references). The step is a constant
  (`DEFAULT_NUDGE_STEP` = 1 display unit) until UI-11 adds the preference.
- `UnitPrefs` has only the length unit (S01 TODO FND-10), so the precision is fixed at 2 and
  angles are always degrees.
- The single active field is app-wide (egui memory), not per dialog: there is one tool dialog
  at a time.

*Leftovers:*
- UI-05 uses `SelectionField::mirror` + a forwarded `SelectTool` for its selection inputs and
  `check_expr` for OK enablement; UI-12 reuses `ExprInput` in its table.
- The nudge step and display precision become preferences in UI-11.
- Owner check in `amg-app`: the completion list and the underline look (tested headless).

### 2026-11-18 - UI-07 browser tree

*Landed in `amg-ui` (status `Review`):*
- **`browser`** (new module, `browser.rs` + `browser/{framework, visibility}.rs`):
  `NodeKey` (as §A, plus `new`, `owned`, `as_str`), `BrowserNode` (as §A, plus builder methods
  `new`, `visible`, `target`, `badge`, `renamable`, `icon`, `children`), `BrowserAnchor`
  (`Root`, `Component`), `BrowserProvider` (as §A, `'static`), `BrowserState` (per document:
  cached pre-order tree, expansion set, inline rename, hover; `rebuilds`, `node_count`,
  `row_count`, `row_labels`, `node`, `is_expanded`, `set_expanded`, `reveal`, `invalidate`),
  `ROW_HEIGHT`, `FRAMEWORK_PROVIDER`.
  - **Framework nodes:** the root (design name, renamable) → Document Settings (units),
    Named Views (Home / Top / Front / Right; double-click orients the camera), Origin (O, XY,
    XZ, YZ, X, Y, Z; one eye), then the root component's contents, Selection Sets (when there
    are any; rename, double-click restores, context menu Restore / Delete), then the providers'
    root nodes. A component's contents: Bodies / Sketches / Construction folders (only when
    non-empty; a folder's eye shows / hides everything in it in one undo entry), the providers'
    nodes for `BrowserAnchor::Component(c)`, then child occurrences (recursively). Default
    names are "Body<n>" (body order), "Sketch<n>", "Plane<n>" / "Axis<n>" / "Point<n>";
    feature names and `BodyProps.name` win. Badges come from the creating feature's `Health`.
  - **Eye:** bodies `SetBodyProps { visible }`, occurrences `SetOccurrenceVisible`, sketches /
    construction / origin the `ExtRecord` `ui.visibility` (`browser::visibility`: version 1,
    `data = {"hidden": [keys]}`, one record per design, `AddExt` / `SetExt`, removed when
    empty; `hidden`, `is_hidden`, `sketch_key`, `construction_key`, `ORIGIN_KEY`,
    `set_hidden_command` for the RND-07 / UI-09 overlays).
  - **Rename** (context menu "Rename", or F2 on a hovered row; an inline field with the name
    selected; Enter / focus loss commits, Esc cancels): bodies `SetBodyProps.name`,
    occurrences and the root `RenameComponent` (R3), sketches / construction `RenameFeature`,
    selection sets `select::sets::rename_set_command` (new).
  - **Hover** feeds the router's Hover highlight when nothing in the viewport is hovered
    (occurrence rows highlight all of their bodies). **Click** selects the node's target
    (Ctrl / Shift / Cmd toggles); targets are re-resolved through `target_for_entity`, because
    body instance indices are scene-side. **Double-click** goes to the provider; the framework
    edits the creating feature (`AppCx::edit_feature`).
  - **Rebuilds** happen only when a provider's `affected_by` accepts one of the frame's events
    (the framework: body add / remove / props, component tree, occurrence visibility, sketch,
    construction, timeline, health, `ExtChanged` of `ui.selection_sets` / `ui.visibility`,
    `DocumentReplaced`), or when the workspace, the document identity or its units changed.
    Expand / collapse only re-flattens the visible rows. Rows are drawn with
    `ScrollArea::show_rows` at a fixed 18 pt row height.
  - Accessibility names: the row label is the node label; the eye is a button "Hide <label>" /
    "Show <label>"; the arrow is "Expand <label>" / "Collapse <label>".
- **`module` / `registry`:** `Registrar::browser_provider`, `UiRegistry::browser_providers`
  (a duplicate provider id is a registration conflict, like every other id).
- **`app`:** `DocTab` gains the browser state and `sketch_edit: Option<FeatureId>`.
  `AppCx::edit_feature(id)` / `AmalgamApp::edit_feature(id)`: a `sketch.sketch` feature enters
  the sketch context and records `sketch_edit`; another kind starts the tool of its
  `feature.<kind>` command (the UI-05 convention) with `ToolInit::edit = Some(id)`; without one
  an info toast says so. `AmalgamApp::{browser_ui, browser, browser_mut}`,
  `app::SKETCH_FEATURE_KIND`. The shell's left panel calls `browser_ui` (the UI-01 placeholder
  is gone). **`tool`:** the Hover layer falls back to the browser's hover; tool factories get
  `ToolInit::edit` through the new crate-private `start_tool_editing`.

*Evidence (Done-when; `tests/browser.rs`, 7 kittest tests on the real app, plus unit tests in
`browser::{framework, visibility}`):*
1. Fixture: two components ("Bracket A", "Bracket B", via `NewComponent`) × three mesh bodies
   each, plus a root construction plane.
   - `eye_toggle_hides_a_body_in_the_scene_and_undo_shows_it_again`: clicking "Hide Body2" is
     one undo entry, sets `visible = false`, and the body's scene instance is no longer drawn
     (the other two are); the eye now reads "Show Body2"; undo draws it again. "Hide Bracket B"
     hides all three of its bodies (`SetOccurrenceVisible`); the Bodies folder's eye hides its
     three in one undo entry; "Hide Plane1" writes `ui.visibility`, and undo removes it.
   - `renaming_a_body_persists_through_save_and_load`: right-click → Rename → "Base Plate" →
     Enter sets `BodyProps.name`; Esc cancels a second rename; renaming the occurrence renames
     its component; after Save As → close → open the names are back in the document and in
     the browser.
   - `click_selects_and_a_selection_set_restores_on_double_click`: a click selects Body1 (the
     Selected layer has it), Ctrl+click adds Body3, `select.save_set` adds "Selection Sets" /
     "Selection Set 1", and after clearing, a double-click on the set restores both bodies.
2. `a_registered_provider_adds_nodes_under_root_without_framework_changes`: an
   `inventory::submit!` module in the test file registers a provider. Its "Test Study" node
   (with a child) appears under the root after the framework nodes, its `build` is called for
   every component anchor, and its eye click reaches `set_visible`.
3. `benches/browser.rs` (criterion): 10 000 one-triangle bodies with the Bodies folder expanded
   (10 000+ rows). A steady frame takes 167 µs (criterion 162.7-172.2 µs); a full rebuild plus
   frame, which only follows an affecting event, takes 2.88 ms (2.85-2.91 ms). The < 1 ms
   budget is asserted on the steady frame's median under `AMG_STRICT_TIMING=1`; it passed
   (bench profile, on a machine shared with another worker's build).
   `the_tree_is_rebuilt_only_on_affecting_events`: expand / collapse, hovering rows, idle
   frames, `AddParam`, `SetOccurrenceTransform` and clearing the selection leave
   `BrowserState::rebuilds` unchanged; one `SetBodyProps` adds exactly one rebuild.
4. `hovering_a_row_highlights_the_body`: hovering "Body5" puts exactly that body's render key
   on the Hover layer; hovering "Bracket B" puts its three bodies there; moving to the viewport
   clears it.

The full `amg-ui` suite runs 125 tests, and all pass (the UI-01 shell snapshots were
re-blessed, see deviations). `cargo clippy -p amg-ui --all-targets -- -D warnings` and
`cargo fmt --all --check` are clean.

*Deviations:*
- **Shell snapshots re-blessed** (`tests/snapshots/shell_{dark,light}.png`): the browser
  panel now shows the real tree (expand arrows, the Origin eye, no empty Bodies / Sketches
  folders) instead of the UI-01 placeholder labels. The test code is unchanged; the owner
  should glance at the new images.
- `BrowserNode::icon` is carried but not drawn (no icon atlas until UI-21); the eye is the 👁
  glyph at small size.
- Occurrence rows target `PickTarget::Occurrence` with `EntityRef::InContext { path: [occ],
  target: Body(first body) }` (the UI-04 encoding); components without bodies of their own
  have no target. The bodies of a component placed several times appear under each
  occurrence (node keys carry an `@occ.path` suffix), and a click selects the first displayed
  instance.
- Sketches have no hover / select target (sketch picking comes with RND-07 / UI-09), so
  `ui.visibility` is stored, but no overlay reads it yet.
- Registry `ContextMenuProvider`s (UI-21) are not consulted; the row menu is the framework's
  Rename / Show / Hide plus `BrowserProvider::context_menu`.
- "Activate component" and the Analysis folder (lane A request, Interface changes 2026-10-20)
  are not part of this WP.
- In kittest a double-click needs a wider `max_double_click_delay`: kittest runs one frame per
  queued pointer event, and egui's clock advanced 0.75 s between the two clicks. The test sets
  2 s; the app is unchanged.

*Leftovers:*
- UI-09 takes over `DocTab::sketch_edit` / the sketch context for the real sketch session;
  UI-05 registers `feature.<kind>` commands, so that a double-click on bodies and construction
  opens their dialogs (until then an info toast).
- Owner check in `amg-app`: the browser with GPU picking and real fonts.

### 2026-11-18 - UI-08 timeline panel

*Landed in `amg-ui` (status `Review`):*
- **`timeline`** (new module, `timeline.rs`): `FeatureUiInfo` and `EditAction` (as §Interfaces),
  `TimelineState` (per document: a node cache rebuilt only when the design revision or the
  evaluated state changes; selection, drag, rename and hover state; the last frame's drawn
  nodes for tests), `NodeHealth`, `DrawnNode`, `DragView`, `EDIT_TOOL_CMD`, `SLOT_W`,
  `computing_node`, `output_targets`, `entity_targets`.
  - **Strip:** playback buttons ⏮ ⏴ ⏵ ⏭ (each a `SetMarker`; disabled when it would not move
    the marker), then one 28 pt slot per node in a horizontal `ScrollArea::show_viewport`. Only
    visible slots are drawn, plus the node being dragged. Node labels are the feature name,
    else `<display_name><n>` (the ordinal within the kind). The icon is drawn as the initials
    of the `FeatureUiInfo` display name. A kind without info gets the generic "◇" and the last
    dotted segment of its kind as name (`tl_anonymous1`). An empty timeline shows "Timeline is
    empty".
  - **Groups:** an expanded group draws a bracket over its nodes (a button "Collapse <name>");
    a collapsed group is one slot (a button "Expand <name>", context menu Expand / Ungroup).
    Both use `SetGroup`.
  - **Rollback marker:** a handle at the marker boundary (accessible Slider "Rollback
    marker"). Dragging it previews the rolled-back nodes (dimmed) and sends one `SetMarker` on
    release.
  - **Reorder:** dragging a node uses `Timeline::valid_move_range`. Positions outside it are
    shaded red for the whole drag, and a drop line marks the target. Release on a valid
    position sends one `MoveFeature { to }` (the final index); an invalid drop sends nothing.
  - **Context menu:** Edit Feature (disabled for `NotEditable`), Edit Sketch (when the
    feature reads a sketch), Roll to Here, Suppress / Unsuppress, Rename (an inline popup
    field; Enter commits, Esc cancels), Delete, Group / Ungroup, Find in Browser
    (`BrowserState::reveal_first`). Actions apply to the timeline selection, in timeline
    order, when the right-clicked node is part of it. Click selects, Ctrl / Shift toggles,
    double-click edits. Each menu action is one `CommitBatch`, so one undo entry.
  - **Health:** error nodes are tinted red and get an "Error" badge, warnings a "Warning"
    badge. The badge's tooltip is the diagnostic message; the node's tooltip is the label plus
    the messages. Suppressed nodes are struck through, rolled-back ones dimmed.
  - **Hover:** a hovered node's output bodies go on the Hover layer (after the viewport and the
    browser hover). A hovered error node's `Diagnostic::entities` go on the Error layer.
  - **Progress ring:** while recompute is `Running`, the node at `progress × len` (below the
    marker) gets an arc ring, and the strip repaints every 50 ms.
  - Hidden in Direct mode (the shell draws the bottom panel only in parametric mode).
- **`registry` / `module`:** `Registrar::feature_ui`, `UiRegistry::{feature_ui, feature_uis}`
  (a duplicate kind is a registration conflict).
- **`app`:** `DocTab::timeline`; `AmalgamApp::{timeline_ui, timeline, timeline_mut}`.
  `edit_feature` consults `FeatureUiInfo::edit` first: `StartTool` starts that tool with
  `ToolInit::edit` (command id `feature.<kind>` if registered, else `EDIT_TOOL_CMD`),
  `SketchMode` enters the sketch context, `NotEditable` shows an info toast, `GenericDialog`
  falls through to the UI-07 behaviour. **`shell`:** the UI-01 timeline placeholder is gone.
  **`tool`:** the router's hover state is a `Vec<PickTarget>` (a node can have several
  bodies), and the Error layer is set from the timeline each frame.

*Evidence (Done-when; `tests/timeline.rs`, 6 kittest tests on the real app, plus 5 unit tests in
`timeline::tests`):*
1. `dragging_the_marker_to_index_5_rolls_back_the_rest`: 20 test blocks (labels Block1..Block20
   from `FeatureUiInfo`). Dragging the marker handle to the boundary before node 5 previews
   nodes ≥ 5 as rolled back with no command sent yet. Release gives one undo entry "Move
   marker", the marker is 5, the drawn nodes ≥ 5 are rolled back, and `Evaluated` agrees
   (node 5 `RolledBack`, node 4 `Ok`). ⏵ and ⏭ move it to 6 and 20; "Roll to Here" on Block3
   puts it at 3.
2. `moving_a_node_before_its_input_is_refused_and_a_valid_move_sends_one_command`: A, Chain(A),
   B, C. Dragging Chain before A shows `to = 0`, `valid = false` and exactly position 0 shaded
   red. Release sends nothing: the undo depth and the order are unchanged. Dragging it to the
   end sends one "Move feature" entry, and the order becomes A, B, C, Chain.
3. `an_erroring_feature_shows_an_error_badge_with_the_message_as_tooltip`: the Broken test
   feature has `NodeHealth::Error` "invalid value for depth: the depth must be positive", an
   Error badge and a red tint (the OK node has neither). Hovering the "Error" badge shows a
   tooltip whose text is exactly that message. "Edit Feature" is disabled because Broken is
   `NotEditable`.
4. `context_menu_edits_are_one_undo_entry_each`: Suppress, Unsuppress, Rename (to "Base"),
   Group (Block1 + Ctrl-click Block2 → "Group1"), Ungroup and Delete each add exactly one undo
   entry, and the document reflects each one (collapse / expand of the group also works).
5. `benches/timeline.rs` (criterion, 500 nodes in a 1280 × 34 strip): a steady frame takes
   48.0 µs (criterion 46.8-49.3 µs), and a node-cache rebuild plus frame takes 211 µs
   (209.4-213.9 µs). Under `AMG_STRICT_TIMING=1` the bench asserts a median under 1 ms over
   200 frames; it passed (bench profile, on a machine shared with other workers' builds).
   The bench also asserts that steady frames don't rebuild and that only the visible nodes
   are drawn.

Also tested: `hovering_a_node_highlights_its_output_bodies` (Block2 → its one body on the
Hover layer; a node without outputs → empty; leaving clears it; `entity_targets` resolves a body
entity) and `unknown_kinds_get_a_generic_node_and_direct_mode_hides_the_timeline`.

The full `amg-ui` suite runs 136 tests, and all pass. The shell snapshots are unchanged (the
empty timeline keeps its height and text). `cargo clippy -p amg-ui --all-targets -- -D warnings`
and `cargo fmt --all --check` are clean.

*Deviations:*
- Icons: `IconId` is not drawn; the node shows the display name's initials (no icon atlas until
  UI-21).
- The computing node is derived from `RecomputeStatus::Running::progress` (`i / len` when node
  i starts), because the running message carries the feature's name, not its id.
- Recompute errors carry no entities today (`Diagnostic::entities` is empty for
  `FeatureError`s), so the Error-layer hover is wired up, but only `entity_targets` is tested
  directly.
- Groups get default names `Group<n>`; renaming a group has no UI yet (`SetGroup` supports
  it).

*Leftovers:*
- UI-21: real icons and the registry `ContextMenuProvider`s in the node menu.
- Features that report error entities (lane-owned), so that the Error-layer hover shows
  something in practice.
- Owner check in `amg-app`: the strip with real fonts, and the progress ring during a slow
  recompute.

### 2026-11-19 - UI-21 marking menu and context-menu providers

*Landed in `amg-ui` (status `Review`):*
- **`marking`** (new module): `Wedge` (as §A, plus `ALL`, `index`, `dir`, `from_dir`, `name`),
  `MarkingMenuBuilder` (`wedge`, `overflow` as §A, plus `wedge_any` = first registered of
  several ids, `new`, `len`, `is_empty`), `MarkingMenuProvider` (as §A, `'static`),
  `ContextTarget` (as §A), `ContextMenuProvider` (as §A, `'static`), `MarkingMenu` (resolved
  `wedges`, `overflow`, `repeat`; `build`, `resolve`, `wedge`, `contains`), `OpenMarkingMenu`
  (`center`, `menu`, `held`, `hot_wedge`), `context_menu_commands`, `context_menu_ui`,
  `FrameworkMarkingMenu` + `marking::defaults`, constants `GESTURE_PT` (12), `CLICK_SLOP_PT`
  (4), `HOLD` (300 ms), `RADIUS_PT`, `DEAD_ZONE_PT`, `OUTER_PT`.
  - **Contents:** providers fill highest priority first (ties in registration order); the
    first claim of a wedge wins, a later claim of a taken wedge goes to the overflow list;
    claims of unregistered commands are skipped (so a later claim can take the wedge) and a
    command appears once. Then the `ContextMenuProvider`s' entries for
    `ContextTarget::Viewport(hovered)` go to the overflow list. "Repeat" is the most recent
    command in the search recency list that is registered, except undo / redo / search.
  - **Framework provider** (priority 0, registered by the `app` module): W `edit.undo`, E
    `edit.redo`, N `sketch.sketch` | `feature.sketch.sketch`, NE `solid.press_pull` |
    `feature.solid.press_pull` | `feature.solid.extrude`, SW `solid.move` | `feature.solid.move`,
    S `edit.delete`, SE `inspect.measure`. Only undo / redo exist today; the others appear once
    their lanes register those ids (see Interface changes).
  - **Gestures** (right button, no modifier, when the active tool ignores the press; the
    menu is built at the press, so it reflects that frame's selection and hover): release
    within 12 pt before 300 ms opens the menu (it stays open); a drag beyond 12 pt runs the
    wedge in the drag direction on release without opening anything (a trail with the
    target's label follows the pointer); holding 300 ms with < 4 pt of movement opens it while
    held, and the release over a wedge runs it (in the centre it stays open). While open:
    clicking a wedge button, "Repeat <label>" or an overflow row runs it; a viewport press in
    a wedge's sector (20..260 pt from the centre) runs that wedge; any other press or `Esc`
    closes it. The press that closed the menu and the release that ended a gesture are
    swallowed up to their click / drag end, so they never select. Commands run through
    `request_command` (end of frame, enablement checked, recorded for recency). Disabled
    entries show their reason as tooltip.
  - The router (`tool`): the marking state lives in the per-document `ToolState`; it is the
    popup step before the tool for pointer events, and `Esc` closes it before cancelling a
    tool.
- **Context providers wired** into the browser row menu (`ContextTarget::Browser(key)`, after
  Rename / Show / Hide and `BrowserProvider::context_menu`) and the timeline node menu
  (`ContextTarget::Timeline(id)`, after Find in Browser), behind a separator.
- `Registrar::{marking_menu, context_menu}`, `UiRegistry::{marking_menu_providers,
  context_menu_providers, marking_menu_owners}`, `AmalgamApp::{marking_menu,
  open_marking_menu, close_marking_menu}`.

*Evidence (Done-when; `tests/marking.rs`, 6 kittest tests on the real app, plus 3 unit tests in
`marking::tests`):*
1. `right_click_opens_the_menu_and_clicking_n_runs_it_and_a_gesture_runs_e`: a right click at
   the viewport centre opens the menu there (not held, still open two frames later); clicking
   the "Wedge N" button runs N once and closes it. A right drag of 80 pt to the right runs E
   once on release, and `marking_menu()` is `None` after every one of the 8 drag steps and
   after the release. A click in the S sector (not on the button) runs S; a click in the
   centre and `Esc` only close it.
   `holding_the_right_button_opens_the_menu_and_releasing_over_a_wedge_runs_it` covers hold
   mode (held, release over W runs W; a release in the centre keeps it open).
2. `repeat_runs_the_previous_command`: no "Repeat" after only an undo; after NE ran from the
   menu (and an undo in between), "Repeat Wedge NE" is offered and runs NE again; a command
   run another way (`run_command`) becomes the repeat entry.
3. `a_test_provider_adds_an_entry_only_for_a_selected_mesh_body`: a priority-50 provider adds
   "Mesh Body Only" to the overflow list only when a whole mesh body is selected; with nothing
   or a solid body selected it is neither in `MarkingMenu::overflow` nor drawn. Its button runs
   the command.
4. `every_wedge_and_the_defaults_are_queryable_by_label`: each of the 8 wedges is found by
   role Button and label, holds the provider's command, and sits in its own direction from the
   centre (`Wedge::from_dir` of the button centre). The framework's Undo / Redo, displaced by
   the test provider's wedges, are drawn in the overflow list.

Also `context_providers_extend_the_browser_and_timeline_menus`: a provider's command in the
Body1 row menu and in the first timeline node's menu runs; its unregistered id is skipped. The
existing browser and timeline tests pass unchanged.

*Deviations:*
- A press / release that moved between 4 and 12 pt before 300 ms opens the menu (the note's
  rules leave that band open; it is treated as a click). A hold needs < 4 pt.
- `MarkingMenuBuilder::wedge_any` (additive) lets the framework provider name "press-pull,
  else extrude" without a registry handle; claims are resolved by `MarkingMenu::resolve`.
- Viewport context entries (`ContextTarget::Viewport`) are shown in the marking menu's overflow
  list; there is no separate viewport context menu (as in Fusion).
- Wedge buttons are labelled with the command label only, so a label that also exists in the
  shell (Undo in the app bar) is found twice by kittest; the test counts it.

*Leftovers:*
- Icons: `IconId` is still not drawn (wedges, browser, timeline). Proposed **UI-23** "SVG icon
  registry (`Registrar::icon`, egui_extras `svg`, licence row already planned in
  THIRD_PARTY.md) and icons in toolbar, marking menu, browser and timeline".
- The default wedges sketch / press-pull / move / delete / measure appear only when UI-09,
  UI-05 / S04, a delete command and INS-08 register those ids; `edit.delete` has no owner yet
  (proposed for UI-05 or a small framework WP).
- Owner check in `amg-app`: the menu with real fonts and a real mouse (tested headless only).

### 2026-11-25 - UI-23 added (owner decision)

The owner approved the UI-21 leftover proposal: new WP UI-23 (SVG icon registry and icons, lane D, P2).
Icon set: an MIT-licensed open set (Lucide or Tabler), decided by the implementing agent with the
ADR-012 check. Not added (owner, 2026-11-25): a framework `edit.delete` WP and a per-face GPU class
buffer (S10 request from AM-12) stay as recorded leftovers.

### 2026-11-25 - UI-23 SVG icon registry and icons (lane D; status Review)

What landed (all in `amg-ui`, no change to other crates):
- **Set**: Tabler Icons 3.53.0 outline (MIT; ADR-012 check: permissive, no mixed licence text unlike Lucide's ISC + Feather MIT; 5 000+ icons incl. CAD-like ones). Only the 56 used SVGs are vendored in `crates/amg-ui/assets/icons/` with `LICENSE-tabler.txt`; no build-time download. THIRD_PARTY.md: egui_extras and resvg flipped to In use (egui_extras **0.36.2**, resvg 0.45.1, not the planned 0.37.0, to match the egui 0.36 pin), Tabler row added. Workspace `Cargo.toml` gets `egui_extras` (`default-features = false`, `svg`).
- **`amg_ui::icons`**: `ids::*` consts + `BUILTIN` table (`include_bytes!`), `Registrar::icon(IconId, &'static [u8])` -> `UiRegistry::{icons, has_icon}` (duplicate id: debug panic / release log, like the other registries), registered for the framework by `builtin::register`. `IconCache` (in the egui context, installed each frame by `AmalgamApp::logic`) rasterises `currentColor` strokes as white through egui_extras `load_svg_bytes_with_size` at `size_pt * pixels_per_point` and keeps one texture per (icon, pixel size); a scale change drops the textures and re-rasterises lazily, `raster_count()` counts rasterisations. Draw helpers `icons::button` (icon + label button, tinted with the text colour, weak when disabled), `icons::image`, `icons::paint`. `IconId::NONE`, an unknown id or a broken SVG draw nothing and the caller keeps its label.
- **Drawn in**: toolbar and menus (`shell::command_button`, Undo / Redo of the title bar), marking-menu wedges, repeat and overflow entries and context menus (`marking::command_button`), browser rows (`BrowserNode::icon`, 14 pt before the label), timeline nodes (`FeatureUiInfo::icon`; unknown kinds use `ids::GENERIC`; a node whose icon cannot be drawn keeps the initials). Labels stay the accessible names and tooltips, so every existing kittest query still works.
- **Assigned** (one `.with_icon(..)` / `.icon(..)` per command or node, no behaviour change): framework (`app.search`, undo, redo, selection, file), `amg-ui::mesh` (12 ops via the new `MeshOp::icon()` + analysis), `inspect` (measure, properties, section, section stats, Analysis folder and rows), `io` (insert mesh, export), `am` (14 commands via a new private `Cmd::icon` field; setup / plate / item nodes), framework browser nodes (root, settings, units, views, origin, bodies, sketches, construction, components, selection sets).
- `TimelineState` got a parallel `icon_ids` vector instead of changing `Item`, so the existing timeline unit tests compile unchanged.
- Snapshots `tests/snapshots/shell_{dark,light}.png` regenerated (toolbar / browser now show icons; 921 pixels differed); that is the only change to an existing test artefact.

Done-when evidence (`crates/amg-ui/tests/icons.rs`, 10 tests): 1. `every_registered_command_has_an_icon_that_exists` lists commands without an icon (empty), plus `toolbar_icons_are_drawn_with_their_texture`, `marking_menu_wedges_draw_their_icon`, `browser_nodes_draw_their_icon`, `timeline_nodes_draw_registered_and_generic_icons` check that the painted shapes contain a quad with the icon's texture. 2. `rasterising_happens_only_on_a_scale_change` (1.0 -> 1.5 -> 2.0: counter constant while frames repeat, grows on each scale change) and `rasters_have_the_pixel_size_of_the_scale` (16 / 24 / 32 px). 3. `unknown_and_none_icons_fall_back_to_the_label` (no icon / unknown id: labelled button, no panic, `texture` returns `None`). 4. `rasterising_every_icon_cold_is_fast`: 56 icons in 4.6 ms (dev profile, one machine), asserted < 20 ms only with `AMG_STRICT_TIMING=1`. 5. `icon_set_licence_is_vendored_and_listed`.

Leftovers / notes: workspace-wide clippy cannot run in this worktree (no vcpkg Manifold install); `cargo clippy -p amg-ui --all-targets -- -D warnings` is clean. A second usvg (0.45 via resvg) sits beside amg-sketch's 0.48.1; harmless, to be unified when egui moves to 0.37. Workspace / panel / sketch constraint icons are not drawn yet (workspace switcher and panel tabs still text-only; the sketch module registers no commands yet). Domain modules that add commands later should pass `.with_icon(..)`; the test `every_registered_command_has_an_icon_that_exists` fails for any new command without one (test-module commands with the `uitest.` prefix are exempt). Owner check: run the app (`cargo app` or `cargo run -p amg-app`) and look at the toolbar, marking menu (right-click in the viewport), browser and timeline at 100 % and 150 % scale.

### 2026-11-25 - UI-09 sketch mode core (lane D; status Review)

*Landed in `amg-ui` (new module `sketch`: `sketch.rs` + `sketch/{session,solver,infer,view,tools,palette}.rs`; no change to `amg-sketch`, `amg-features`, `amg-render`, `amg-doc`):*
- **Commands:** `sketch.sketch` "Create Sketch" (SOLID ▸ Create and SURFACE ▸ Create, not pinned, see deviations; the marking menu's N wedge now resolves), `sketch.finish` "Finish Sketch" (SKETCH ▸ Finish Sketch, pinned), `sketch.line` "Line" (SKETCH ▸ Create, pinned), `sketch.select` (search only; the session's default tool). Enablers: Create only outside, the others only inside the `sketch` context. `FeatureUiInfo` for `sketch.sketch` (display name "Sketch", pencil icon, `EditAction::SketchMode`). Icons `line` and `circle-check` vendored (Tabler 3.53.0).
- **Session (`sketch::session`):** `SketchLayer` is a `DocLayer` (`sketch.session`) that owns the `SketchSession`. It opens a session from `DocTab::sketch_edit` (browser / timeline Edit Sketch, taken with the new `AppCx::take_sketch_edit`) or from Create Sketch (the feature id comes back from an `apply_document` closure), animates `look_at_plane`, enters the `sketch` context (SKETCH tab) and opens the Sketch Palette. It ends on Finish Sketch, the palette's Finish button, Esc in the select tool with nothing selected, the context going away, or the feature disappearing (deleted, or its creation undone). When no tool is active it restarts `sketch.select` (through the new `LayerOut::active_tool`).
  - Working copy: a clone of the `SketchFeature`; every edit is `SketchFeature::apply` on it plus one `EditFeature` (one undo entry). The design's feature is watched by `Arc` pointer; any other change (undo / redo, the command layer promoting dimensions to `d<n>` parameters) reloads the working copy. `commit` / `commit_op(op, then)` let a tool add follow-up edits that need the created ids (the line's end points) in the same undo entry, and name constraints that must be checked first.
  - Solver worker (`sketch::solver`): `SolverState { solver: NativeSolver, sketch, drag }` moves into a hidden interactive `SolverJob` and back, so at most one solver call is in flight. Ops: `Solve` (with `DofAnalysis::PerEntity`, and SK-06 profile detection on the job), `Check` (SK-04 `check_constraint`, accepted ones added in order), `DragStep { begin, target }` (SK-05 `begin_drag` / `step`), `DragFinish`. Edits fold into the next solve (latest wins; results of older generations are dropped); drag targets are coalesced (only the newest is solved). Solved values are copied back into the working copy as its warm start.
  - Drag: `begin_drag` (point, line, circle, arc), `drag_to`, `end_drag`; the release finishes the drag in a job and becomes one `CommitDrag` (undo label "Drag sketch geometry").
- **Overlays (`sketch::view`):** drawn every frame from the newest solved sketch: curves (construction dashed), points, profile shading (ear clipping of the outer loop), constraint glyphs (text: H, V, //, _|_, T, =, Mid, Fix, ...), dimension labels at `label_pos`, and the adaptive RND-07 grid, all `AlwaysOnTop`. DOF colours come from `DofReport` (`Under` blue, `Full` foreground, `Over` red; selection and hover override). Visible sketches that are not being edited are drawn thin (depth tested) outside sketch mode too; sketches hidden in `ui.visibility` are skipped.
- **Palette (`sketch::palette`):** `SketchPaletteDialog` (an `AppDialog`, bottom right): Sketch grid, Snap, Slice (an RND-06 section plane at the sketch plane through `LayerOut::sections`), Show profile / points / dimensions / constraints, DOF, Finish Sketch.
- **Inference (`sketch::infer`):** `infer` on `query::snap_candidates_with` (UI thread).
  - Point snaps win over alignments; their suggestion is added on commit.
  - Alignment with the rubber-band anchor makes the line `Horizontal(Line)` / `Vertical(Line)`. Alignment with other points snaps the position (both axes combine) and adds `Horizontal/Vertical(Points)` only when the line gets no direction constraint.
  - Direction constraints of a segment whose end also snapped (for example the closing segment) go through the `Check` job first.
  - Ctrl (held now, `ToolCx::modifiers`) suppresses inference. Grid snap (palette) applies when nothing else snapped (step 1/2/5 × 10ⁿ mm, at least 20 pt).
  - A glyph is drawn next to the cursor, with dashed alignment guides.
- **Tools (`sketch::tools`):**
  - `CreateSketchTool`: origin planes drawn as quads at 90 pt, ray-tested on the CPU (nearest wins); construction planes through the hover pick; dialog buttons XY / XZ / YZ.
  - `SketchSelectTool`: hover, click and Shift/Ctrl toggle via `query::pick`; drag through the session; Delete removes the selection; Esc clears it, then leaves the sketch.
  - `LineTool`: chained click-click lines (consecutive segments joined by `Coincident`; clicking the chain's first point closes and ends the chain). Heads-up input: type a number for the length field, Tab for the angle field (from the previous line, or the x axis), Enter places the point; the typed values become driving `Length` / `Angle` dimensions with their expressions. Tangent-arc drag-back: press on the chain end and drag to get an arc tangent to the previous curve (`Coincident` + oriented `Tangent { AtEndpoint }`).
  - The status hint starts with "DOF: n".

*Evidence (Done-when; `crates/amg-ui/tests/sketch_mode.rs`, 10 kittest tests, plus 7 unit tests in `sketch::{infer,view,tools}`):*
1. `rectangle_with_inference_gets_coincident_horizontal_vertical_and_one_profile`: Create Sketch on XY, Line tool, clicks (0,0) → (40,0) → (40,30) → (0,30) → (0,0). Result: exactly 4 `Coincident`, 2 `Horizontal`, 2 `Vertical` (all four on the lines), 8 constraints in total; 1 profile of area 1200 mm²; 4 undo entries (the closing one after its check job). The document's recompute has the same profile.
2. `rectangle_has_four_dof_and_every_entity_is_under_constrained`: DOF 4; all 12 entities (4 lines, 8 points) are `Under`; "DOF: 4 ·" is in the status bar and "DOF: 4" in the palette.
3. `timing_dragging_a_corner_for_1000_frames_is_one_undo_entry`: 1000 pointer moves drag corner (40,30) to (55,45). 1006 frames; `amg_ui.frame` p50 0.32-0.37 ms, **p95 0.61-0.64 ms, max 0.94-1.14 ms** (three runs with `AMG_STRICT_TIMING=1`, dev profile, this machine with another worker building); 973 drag solves. Exactly one undo entry ("Drag sketch geometry"); both coincident corner points end at (55,45), and the profile is 55 × 45. p95 < 2 ms and max < 16 ms are asserted only under `AMG_STRICT_TIMING=1`.
4. `undo_and_redo_inside_the_session_reload_the_working_copy`: undo → 3 lines in the working copy and the display, open chain (no profile), DOF 5; undo again → 2; redo twice → 4 lines, DOF 4, 1 profile.
5. `finish_sketch_leaves_sketch_mode_and_the_timeline_shows_a_healthy_sketch`: Finish Sketch → no `sketch` context, no session, no active tool, palette closed, the finished sketch still drawn. `Evaluated::node(id).health == Ok`, the node is `sketch.sketch` and "Sketch1" is shown. Edit Sketch re-opens it with its 4 lines.

Also tested:
- `clicking_the_xy_plane_in_the_viewport_creates_the_sketch_and_looks_at_it`: plane pick in the viewport; the SKETCH tab and the palette show; the camera looks along −Z.
- `esc_with_no_sketch_tool_leaves_the_session_and_ctrl_suppresses_inference`: a Ctrl click adds no `Horizontal`; hovering shows the "H" glyph; Esc ends the chain, then the Line tool, then the session.
- `heads_up_length_becomes_a_driving_dimension`: type 25, Enter → a 25 mm line toward the cursor with a driving `Length` dimension.
- `dragging_back_from_the_chain_end_draws_a_tangent_arc`: 1 arc, 1 `Tangent`, the solve converges.
- `palette_toggles_change_the_view`: Slice adds one section plane; grid off removes its overlay.

The full `amg-ui` suite runs 270 tests: 270 passed, 2 skipped. Existing tests and the UI-01 shell snapshots are unchanged. In one full run, `browser::a_replaced_document_rebuilds_the_tree` failed once inside `AppHarness::with_document` under load; it passed in 3 targeted reruns and in the next full run. `cargo clippy -p amg-ui --all-targets -- -D warnings` and `cargo fmt --all --check` are clean.

*Deviations:*
- **Create Sketch is not pinned** in SOLID ▸ Create (it is in the dropdown): a pinned button changes the UI-01 shell snapshots, which this WP had to keep unchanged. Pinning it is a one-line change plus re-blessing the snapshots (owner decision).
- **No default `L` shortcut for Line:** `tests/commands.rs` registers `test.sketch_line` with `sketch:L`, and the module registered first (`sketch` sorts before `zz_cmds`) would win the binding, which changes that test's result. Give `sketch.line` its `sketch:L` default once that test uses another key.
- **Constraint and inference glyphs are text labels** (there is no glyph icon atlas yet); UI-14 owns the real glyph rendering with hover-related highlight.
- **Heads-up angle:** it is measured from the previous line (straight on = 0°) and becomes a driving `Angle` dimension only when the previous chain curve is a line. On the first segment the angle sets the direction from the sketch x axis, without a dimension.
- **Profile shading** fills the outer loop only (holes are not cut out).
- **Selection inside the sketch** is session-local (`SketchSession::selected`), not `amg_ui::Selection`, because `PickTarget` has no sketch-entity variant yet. A primary drag on empty space does nothing (no box selection in sketch mode yet).
- Planar faces as sketch planes need BREP-17 (`PlaneInput::Ref(Topo)` fails in SK-19), so only origin and construction planes are offered.
- The full-foreground colour is a fixed near-black (as in Fusion), not a theme token (the layer has no theme access).

*Leftovers:*
- Owner check in the app (`cargo app`), tested headless only:
  1. Create Sketch (SOLID ▸ Create ▸ Create Sketch, or the marking menu's N wedge), then click the XY plane.
  2. Line (SKETCH tab): draw a rectangle with inference (watch the H / V glyphs and the dashed guides; close on the first point).
  3. Press Esc and drag a corner.
  4. Finish Sketch.
  Check the look-at animation, the palette and the overlay colours with the real renderer.
- Suggestions are not pre-checked on hover with `query::suppress_redundant` (SK-20 suggests a job on hover dwell); they are checked at commit, and only for segments whose end snapped.
- The overlays are rebuilt every frame from the display sketch. This is fine at this size; for large sketches, cache the sampled polylines per `values_rev`. Dragging several entities at once (multi-select) is not wired.

### 2026-11-25 - UI-22 solid body display (lane D; status Review)

*Landed in `amg-ui` (new `scene/solid.rs`; changes in `scene/{mod,tess,backend}.rs`, `pick.rs`, `builtin.rs`; no change to `amg-brep`, `amg-render`, `amg-doc` or `amg-features`):*
- **`scene::SolidTessellator`** (registered by module `app` next to the Mesh one): handles a `BodyGeom::Solid` that holds a real `amg_brep::Shape` and calls the public BREP-08 `BrepKernel::tessellate` inside the adapter's job.
  - `quality_preset`: `Preview` -> `TessQuality::preview()`, `Normal` -> `display()`, `Fine` -> `export()`.
  - Positions and edge points are rebased to the centre of their bounds. `TessMesh::face_ranges` become `DisplayMesh::face_ranges` (face sub-id = B-rep face index). Each edge polyline becomes an `amg_render::Polyline` whose id is the B-rep edge index; degenerated edges have no points and are left out.
  - `MeshData::closed` is set when the welded triangles form a closed, consistently oriented surface, so RND-06 caps solids but not open surface bodies.
- **Additive trait API:** `DisplayGeometry { mesh, edges }`; `BodyTessellator::{tessellate_display, depends_on_quality}` with defaults, so existing tessellators and `DisplayMesh` literals compile unchanged.
- **Upload:** the GPU backend uploads B-rep meshes with `upload_mesh_with_faces` (GPU picks report the face index) and the edges with `upload_edges` (RND-12) on the same job. The edge set is inserted, repointed and removed together with its mesh, and the instance gets `Instance::edges` and `mesh_wireframe = false`. `UploadedMesh::{with_edges, edge_polylines, into_gpu_parts}`, `MeshUploader::upload_edges` (default: count only), `CpuSceneBackend::mesh_edges`, `GpuSceneBackend::edges_key`, `SceneAdapter::display_edges`.
- **Latest-wins and quality:** a geometry change works as for meshes (old mesh drawn until the new one arrives). New `SceneAdapter::change_quality(q, jobs)` re-tessellates only the committed bodies whose tessellator depends on quality; `set_quality` is unchanged. Preview-layer bodies are always tessellated at `Preview`.
- **CPU picks:** `CpuPickService` point picks now report the B-rep edge within the pick radius of the face hit point (select-through and headless tests; the app's front-most picks come from S10).
- **Without OCCT** (and for placeholder solids) no tessellator handles the body: it shows nothing, is counted in `stats().undisplayable`, and the adapter warns once.
- AM-11 body views, AM-12 shading, INS-09 section style and INS-10 tints act on instances, so they apply to solids without further code (tints are asserted on a solid).
- `amg-ui` has a new feature `occt` (`amg-features/occt`, `amg-brep/occt`) and a normal dependency on the workspace crate `amg-brep`. No new third-party dependency.

*Evidence (Done-when; `tests/solid_display.rs`, 6 tests of which 5 need `--features occt`, plus 3 unit tests in `scene::solid`; dev profile, owner machine with another worker building):*
1. `a_1030_face_part_displays_with_every_face_and_edge_pickable`: a 32 x 32 hole plate (BREP-14 box minus 1024 cylinders in one BREP-04 cut) shows 1030 faces and 3084 edges (151 564 triangles), one polyline per edge with ids 0..3084. `PickMap` round-trips every face and edge. A ray at each face returns that face (1030 picks, 100 ms) and a `CpuPickService` edge pick next to each edge returns that edge (3084 picks, 2.6 s). Recompute plus display took 7.6 s. `gpu_picks_report_brep_face_and_edge_indices` checks the real renderer on a box: its 3 visible faces and 9 visible edges are picked by index, and nothing is left on the GPU after `clear`.
2. `editing_an_upstream_parameter_tessellates_only_the_changed_bodies`: three solids, two reading parameter `w`. Setting `w` takes `stats().tessellations` from 3 to 5 and uploads per body to 2 / 1 / 2; the third body keeps its mesh slot.
3. `timing_tessellating_a_large_solid_runs_on_a_job_without_slow_frames` (967-face plate): the tessellator runs off the UI thread; the job took 4.4 s while 1709 frames ran, `amg_ui.frame` p50 0.26 ms, max 0.96 ms. With `AMG_STRICT_TIMING=1` (asserts max < 16 ms): 4.7 s, 1796 frames, p50 0.33 ms, max 1.8 ms.

Also: `quality_presets_retessellate_solids_latest_wins` (cylinder: 100 / 212 / 672 triangles at Preview / Normal / Fine; a superseded job keeps the old mesh drawn; previews use `Preview`), `solids_without_a_shape_show_nothing_and_are_logged_once`.

`cargo nextest run -p amg-ui`: 279 passed, 2 skipped; with `--features occt`: 284 passed, 2 skipped. `cargo clippy -p amg-ui --all-targets -- -D warnings` is clean with and without `occt`. Existing tests pass unchanged.

*Deviations:*
- The Done-when 1 part is not a STEP file: BREP-13 (the reader) does not exist yet.
- "Every face and edge pickable" is proven for all of them on the CPU path; the GPU path is checked on the box only.
- The frame timing uses the CPU scene backend, so the UI-thread insert into a real `RenderScene` is not in those numbers.

*Leftovers:*
- Owner check: `cargo app` with a solid part. No command or importer creates a solid yet (S04 features, BREP-13), so this waits for the first one. Then check shading, edges, the visual styles, hover and selection of faces and edges, and a section cap.
- B-rep vertices are not drawn or picked (no `PointData`).
- A hit B-rep edge does not stand for a face under the face selection priority (`select::face_of` needs edge-to-face adjacency); selecting solid sub-entities also needs the feature to provide naming.
- Large planar faces mesh slowly (BREP-18), and OCCT cancels coarsely, so a superseded job on such a part runs on for a while.
- A CPU ray cast (select-through) builds a mesh's BVH on the calling thread (UI-20 behaviour); on a large solid this and the linear nearest-edge search may need a BVH built on the job.
- CI: no job runs `amg-ui` with `--features occt` yet (S00).

### 2026-11-25 - UI-12 parameters dialog (lane D; status Review)

*Landed in `amg-ui` (new module `params`; no change to `amg-doc`, `amg-features` or `amg-ui::sketch`):*
- **Command** `params.show` "Change Parameters" (SOLID ▸ Modify and SURFACE ▸ Modify dropdowns, not pinned, and S-key search; icon `adjustments`; no shortcut). It opens one non-modal `params::ParametersDialog` per document (an `AppDialog`; the view flag `params.open` keeps it single).
- **`params::ParametersView`** (the content, usable without the app): filter (name, expression or comment, any case), the add row (name, unit, expression, comment, preview, "Add parameter" / Enter) and an `egui_extras::TableBuilder` table with virtualised rows: ★, name, unit, expression (`ExprInput`), value, comment, Delete. Groups: "Favourites" (pinned; a favourite leaves its own group), "User Parameters" (id order), "Model Parameters: <feature label>" in timeline order. The row list is rebuilt only when the document revision or the filter changes.
- **Edits**: one S01 command per `CommitBatch`, so one undo entry each (`AddParam`, `RenameParam`, `SetParamExpr`, `SetParamMeta` for comment and favourite, `DeleteParam`). A text cell commits when it loses the focus (Enter, a click elsewhere, Tab in name and comment cells) and when the dialog closes; Esc reverts; unchanged text submits nothing.
- **Inline errors**: every command is first tried with `amg_doc::apply_command` on an O(1) copy of the design. A rejected edit is not submitted (no toast, document unchanged): the typed text stays in the cell and the message is in the row's value column, with the hint as tooltip ("circular reference: b → a → b", "unit mismatch: expected mm, got t", "name `w` already in use", "in use by v"). Expression cells show the error, with the underline, while typing; so the check applies the parameter's own unit and finds cycles, unlike `check_expr`.
- **Additive framework changes**: `ExprInput::{id, show_value, accessible_name, checker}`; `timeline::node_label` (crate-internal, now shared by the timeline and the dialog's group titles).

*Evidence (Done-when; `tests/params.rs`, 8 kittest / headless egui tests):*
1. `user_parameter_drives_a_model_parameter_and_rename_rewrites_it`: add `w = 20 mm` in the add row, set `d1` to `w * 2` → value cell `40.00 mm`; rename `w` → `width` → `d1` reads `width * 2`, still `40.00 mm`.
2. `a_cycle_is_shown_inline_and_leaves_the_document_unchanged`: `a = b`, then `b = a`: "circular reference: b → a → b" while typing and after Enter; serialized design and history revision unchanged; Esc restores the text. Unit, unknown-name, decimal-comma, name-taken and in-use errors: `unit_name_and_in_use_errors_are_shown_inline`.
3. `each_edit_is_one_undo_entry`: add, expression, rename, comment, favourite, delete each raise the history revision by exactly 1 with their own undo label; unchanged re-entry adds nothing; six undos walk back through the same parameter states and leave nothing to undo.
4. `benches/params.rs` (`params_1000/{steady_frame, rebuild}`) and `timing_a_thousand_parameters_render_only_the_visible_rows`; the 2 ms budget is asserted only under `AMG_STRICT_TIMING=1`. Measured on the owner machine (880 × 440 pt dialog content, layout without GPU tessellation or painting): criterion `steady_frame` **298 µs**, `rebuild` (new revision each frame) **338 µs**; the test (dev profile, optimised) 447 µs / 420 µs median.
- Also `groups_favourites_and_filter`, `new_parameter_row_previews_and_refuses`, `the_command_opens_one_dialog_and_it_can_be_reopened`.

*Deviations:*
- The unit column is read-only: S01 has no command that changes a parameter's unit (see Interface changes).
- Model parameters have no Delete button (they go with their feature; a deleted feature's parameters become user parameters, which can be deleted).

*Leftovers:*
- Owner check in `cargo app` (tested headless only): SOLID ▸ Modify ▸ Change Parameters (or `S`, "param"), add `w = 20 mm`, set another parameter to `w * 2`, rename `w`. Look at the table layout, the ★ and → glyphs and the completion list under a table cell.
- S-key search by parameter name that opens the dialog on it (spec §4) is still not wired.
- UI-15 `ExprInput` captures `Tab` for completion, so in an expression cell it does not move the focus to the next cell.

## Interface changes

### 2026-10-20: requests from the wave-2 refinements (S02, S05, S10, S11, S12, S16, S17, S20)

Additive requests recorded by the integration pass; the owning lane confirms or adjusts them when it implements the affected WP.

- **From S10 (rendering).**
  - UI-03 now depends on RND-11 and drives `NavController` from `NavPreset`; applied in the WP
    table.
  - `amg-app` uses `SurfaceConfig::HIGH_THROUGHPUT`, egui multisampling 0, no depth buffer.
  - UI-20/UI-22 follow S10's scene-adapter contract; UI-06/UI-09 overlays map onto `OverlayList`.
  - Two deviations to absorb: labels are egui-painted (`project_labels` + `paint_labels`), not
    MSDF; `PickMode::AllAlongRay` is served by a CPU ray cast in `PickService` (S10 returns
    visible hits only).
  - Device-lost UX: save + restart.
- **From S05 (inspection).**
  - UI-03: non-committing tools (Measure) that own overlays and close with Esc.
  - UI-04: mesh sub-element selection kinds (triangle, triangle edge, vertex, surface point) in
    the filters, plus a hook so a tool can expand a pick (smart region, edge chain).
  - UI-07: an "Analysis" folder populated from registered `ExtRecord` kinds (icon, label,
    visibility toggle, context actions supplied by the domain module), plus "Construction" and
    "Origin" folders.
  - UI-05: dialog schemas for the construct features (CON-04).
  - A clipboard helper for copying measured values.
- **From S16 (AM).** Manufacture workspace entry in the UI-01 switcher (covered by the
  Manufacture-AM workspace); later a "Manufacture" preferences page (UI-11) for slicer paths and
  the hand-off directory (Phase 1 uses an `amg-ui::am` settings dialog).
- **From S02 (B-rep kernel).** UI-22 maps a picked triangle to a face with
  `TessMesh::face_of_triangle` (face order = `face_ranges`); edge lines come from
  `TessMesh::edges`.

### 2026-11-08: from the S04 refinement (lane A)

S04 adopts Interfaces §B (`amg_features::dialog`) as is (S04 Open question 3), with two
additive pieces UI-05 should render:
- `DialogRegistration::new_component: Option<fn(&FieldValues) -> Option<String>>`. When it
  returns `Some(name)`, UI-05 commits `AddComponent { name }` and `AddFeature` with the new
  component as the context in one transaction (Fusion's "New component" operation). `None`
  for every registration until S04 SOL-24.
- `DispatchRegistration { command_id, dispatch, command }` (inventory): a command that starts
  another kind's generic dialog chosen from the current selection. Press-pull
  (`solid.press_pull`, S04 SOL-09) maps a face to `solid.offset_face`, an edge to
  `solid.fillet` and a profile to `solid.extrude`.

### 2026-11-08: requests from the S06 refinement (lane E)
- Generic dialogs and tools take `AddFeature::context` from `Document::active_component()`
  (S06 ASM-01).
- UI-20 takes instance transforms from `Evaluated::instances` / `placements` (solved, ASM-01/04)
  and also updates them on `OccurrenceTransformChanged` from an evaluated swap.
- `SceneAdapter::set_placement_overlay(Option<Arc<amg_doc::Placements>>)`: transient placements
  for drag frames (S06 ASM-12), so dragged instances move without a recompute.
- Picking through instances: `PickTarget::Body` gains an instance index (`u32` into the scene's
  occurrence-path table; `PickTarget` stays `Copy`), and `SelItem::entity` wraps sub-entities
  picked outside the active component in `EntityRef::InContext { path, .. }`.
- The assembly module would be `amg-ui::assembly`, owned by lane E if the owner agrees
  (QUESTIONS #60).

### 2026-11-10: UI-01 (lane D)
- Workspace `Cargo.toml` (shared): new `[workspace.dependencies]` `egui_dock` 0.21.1 (`serde`),
  `egui_kittest` 0.36.2 (`snapshot`, `wgpu`) and `directories` 6.0.0, listed in
  `THIRD_PARTY.md`. `amg-app` enables eframe's `accesskit` feature for itself only.
- No change to `amg-render`, `amg-jobs`, `amg-doc` or `amg-features`: `amg-app` calls the
  existing `amg_features::{mesh, sketch}::link()` (there is no crate-level `link()` yet).

### 2026-11-12: UI-02 (lane D)
- `amg-ui` gains the `toml` workspace dependency (already in `THIRD_PARTY.md`) and a `criterion`
  dev-dependency with bench `search`. No change outside lane D except the `amg-app` start-up
  loading `shortcuts.toml`.

### 2026-11-12: UI-10 (lane D)
- No change outside lane D. `amg-ui` has no new dependency (`egui`'s own `ProgressBar`, `Area` and
  `copy_text`). `amg-app` reads `DiagGuard::log_dir()` and calls `AmalgamApp::set_log_dir`.

### 2026-11-13: UI-20 (lane D)
- No change outside lane D and no new dependency. `amg-ui` calls the inherent `render_buffers` / `aabb` of
  `amg_doc::stub::IndexedMesh` (S07's `RenderBuffers` contract), with no direct `amg-mesh` dependency.
  `amg-render` is used through its public API only (`RenderScene`, `upload_mesh`,
  `egui_glue::with_renderer`, `Renderer::{gpu, scene_mut}`). No `amg-doc` type reaches `amg-render`.
- For RND-04 (S10, lane D): `amg_ui::scene::{RenderSubKind, Range32, HighlightLayer, HighlightItem}`
  mirror S10 §Interfaces. They become re-exports when `SubKind` / `set_highlight_layer` land.
  `InstanceDesc::key` is the `PickId` to write into the instance record.
- `amg-app` installs `GpuSceneBackend::egui` after `AmalgamApp::new`.

### 2026-11-15: UI-19 (lane D)
- Workspace `Cargo.toml` (shared): new `[workspace.dependencies]` `rfd` 0.17.2
  (`default-features = false`, `xdg-portal`) and `raw-window-handle` 0.6.2, both used by `amg-ui`
  only; `zip` and `tracing-subscriber` become `amg-ui` dev-dependencies. `THIRD_PARTY.md` rows
  updated (rfd MIT, its transitive crates MIT / Apache-2.0, raw-window-handle MIT / Apache-2.0 /
  Zlib; the lock file gains `rfd` and, for Linux only, `pollster` 0.4.0 (MIT / Apache-2.0)).
- No change to `amg-doc`, `amg-jobs` or `amg-core`: UI-19 uses the public DOC-07 / DOC-14 API
  (`io::{save_in_job, load_in_job, SaveOptions, StubBodyCodec}`, `Document::{snapshot,
  cache_export}`, `History::mark_saved`, `RecoverySession`, `find_recoverable`,
  `recover_with_report`, `RecoveryCandidate::discard`) and `amg_jobs::diag::{register,
  unregister}_recovery_hook`.
- For IO-01 (lane G): register Import/Export commands with `Placement::Menu { path:
  "File/Import" | "File/Export", .. }`; the placeholder submenu disappears once one exists.
- `amg-app`: `RfdDialogs::with_parent(cc)`, `set_recent_path`, `set_recovery` + `check_recovery`
  before the first frame, `--recovery-dir` / `--no-recovery` / a `FILE.amg` argument, and
  `shutdown()` in `on_exit`.

### 2026-11-15: RND-04 (S10, lane D) in `amg_ui::scene::backend`
- `GpuSceneBackend` now writes `InstanceDesc::key.pick_id()` into `amg_render::Instance::pick` (on
  insert and on every `SetInstance`), so the GPU pick pass reports the adapter's `RenderEntityKey`.
- `SceneOp::Highlight` / `SceneOp::HighlightColor` are forwarded to
  `RenderScene::set_highlight_layer` / `set_highlight_color` (mapped item by item; the backend still
  keeps the last layers for `highlight` / `highlight_color`), so hover and selection highlights are
  drawn. The `amg_ui::scene` mirror types stay (no re-export yet), to keep this change minimal.
- Not done (leftover for UI-03): a GPU `PickService`. S10's `Renderer::request_pick` / `poll_picks`
  are ready for it (point picks with radius and filter, window/crossing rectangles); `PickService`
  does not exist yet (UI-03 defines it), and `CpuPickService` stays the CPU reference and the
  select-through path.

### 2026-11-17: UI-03 (lane D)
- No change to `amg-render`, `amg-doc`, `amg-jobs` or `amg-core`, and no new dependency. UI-03
  uses S10's public API only: `nav::NavController`, `egui_glue::{apply_camera_action,
  view_cube, view_cube_rect, nav_bar, paint_labels}`, `Renderer::{request_pick, poll_picks,
  project_labels, view_mut}`, `View::{camera, show_view_cube, view_cube}`.
- `amg-ui` additive API: `CmdAction::StartTool` is a new variant (exhaustive matches outside
  `amg-ui` need an arm; there were none). `DocTab` and `CmdEnv` gain fields, and `ViewportFrame`
  gains `camera`. Struct literals of these outside the crate need the new fields (there were
  none). `ViewportBackend`'s new methods have defaults.
- `amg-app`: the demo mesh's home view is set on the active document's camera instead of the
  renderer view, because `RenderViewport` now writes the document camera into the view every
  frame. The now-unused `Shell::view` field was removed.

### 2026-11-17: UI-04 (lane D)
- No change to `amg-render`, `amg-doc`, `amg-jobs` or `amg-core`, and no new dependency. UI-04
  uses RND-04's rectangle picks through the existing `GpuPickService::request_region` and the
  existing highlight layers; selection sets use the public `Command::{AddExt, SetExt}` and
  `Design::{exts, ext}`.
- **For S01 (DOC-06):** `amg_ui::select::naming::{topo_ref_at, resolve_topo}` are UI-side
  stand-ins for R1 / R2 (mesh triangle / edge / vertex names with role tags
  `ui.mesh.triangle|edge|vertex` plus a fingerprint). When DOC-06 lands they should delegate to
  S01 and the mesh lane should name mesh elements; stored selection sets with the provisional
  names then need a migration (ExtRecord version 1 → 2) or are dropped on resolve.
- **For S01:** an occurrence reference (`EntityRef::Occurrence` or similar) would replace the
  `InContext { path: [occ], target: Body }` encoding of component selections.
- `amg-ui` additive API: `ToolCx::{pick_region, item_for, scene}`, `AppCx::{request_repaint_after,
  restore_selection_set}`, `AmalgamApp::{select_through, choose_select_through,
  restore_selection_set}`, `PickMap::instances_of`, `SceneAdapter::occurrence_bodies`, the
  `select::{bar, naming, sets}` modules, and two commands (`select.save_set`, `select.clear`).
  `tool::SelectTool` / `tool::SELECT_TOOL` remain as re-exports.

### 2026-11-18: UI-15 (lane D)
- No change to `amg-doc`, `amg-core`, `amg-render` or `amg-jobs`, and no new dependency.
  `ExprInput` uses the public `ParamTable::{try_eval, iter, get}`, `expr::{reserved_names,
  format_value}`, `ExprError::span` and amg-core's `ParseQuantityError::DecimalComma`.
- `amg-ui` additive API: the `widgets` module; `ExprInput::{label, id_salt, desired_width}`,
  `ExprResponse::{changed, error_span, message, hint, is_valid}`, `SelectionField::mirror`,
  `DialogFrame::{help, show_inside}`; the router intersects the active tool's pick filter with
  `widgets::active_selection_filter`.

### 2026-11-18: MESH-23 (lane C, flagged for lane D)
- **`AppCx::set_highlight(layer, targets)`** (additive): queues a `ShellRequest::Highlight` that
  replaces a highlight layer at the end of the frame while the requesting document is active.
  Body targets are expanded to every displayed instance of their body
  (`tool::all_instances`, crate-private; an undisplayed body's target is kept as given). The
  router keeps owning `Hover` and `Selected`; the Mesh Analysis panel (`amg_ui::mesh`) uses
  `Error` for defective faces. About 25 added lines in `app.rs` / `tool.rs`, nothing else in
  the framework changed. A later per-module highlight-layer API (or RND-08 overlays) can
  replace it.
- `amg-ui` gains a normal dependency on `amg-mesh` (workspace crate; the panel runs the MESH-02
  analysis) and dev-dependencies `amg-mesh` (`corpus`) and `amg-io` (MESH-23 tests).
- The mesh commands' labels end in `…` (dialog commands): a plain "Remesh" would tie with the
  `test.remesh` fixture of `tests/commands.rs` in S-key search.
- Merge with UI-08 (coordinator, 2026-11-19): the router now rewrites the `Error` layer from the
  hovered timeline error node only when that set changes (`ToolState::errors_shown`), so a
  `set_highlight(Error, ..)` from the Mesh Analysis panel is not cleared every frame.

### 2026-11-18: UI-07 (lane D)
- No change to `amg-doc`, `amg-render`, `amg-core` or `amg-jobs`, and no new dependency. The
  browser uses the public `Command::{SetBodyProps, SetOccurrenceVisible, RenameComponent,
  RenameFeature, AddExt, SetExt}`, `Design::{components, occurrences, body_props, exts}` and
  `Evaluated::{bodies, sketches, constructions, node}`.
- New `ExtRecord` kind `ui.visibility` (version 1, owned by `amg_ui::browser::visibility`).
- `amg-ui` additive API: the `browser` module; `Registrar::browser_provider`,
  `UiRegistry::browser_providers`; `AppCx::edit_feature`; `AmalgamApp::{browser_ui, browser,
  browser_mut, edit_feature}`; `DocTab::sketch_edit`; `app::SKETCH_FEATURE_KIND`;
  `select::sets::rename_set_command`.

### 2026-11-18: UI-08 (lane D)
- No change to `amg-doc`, `amg-render`, `amg-core` or `amg-jobs`, and no new dependency. The
  timeline uses the public `Timeline::{nodes, marker, len, node, group, groups,
  valid_move_range}`, `Command::{SetMarker, MoveFeature, SetSuppressed, RenameFeature,
  DeleteFeatures, GroupFeatures, Ungroup, SetGroup}`, `Health`, `RecomputeStatus` and
  `DesignMode`.
- `amg-ui` additive API: the `timeline` module (`FeatureUiInfo`, `EditAction`,
  `TimelineState`, `NodeHealth`, `DrawnNode`, `DragView`, `EDIT_TOOL_CMD`, `SLOT_W`,
  `computing_node`, `output_targets`, `entity_targets`); `Registrar::feature_ui`,
  `UiRegistry::{feature_ui, feature_uis}` (a duplicate kind is a registration conflict);
  `AmalgamApp::{timeline_ui, timeline, timeline_mut}`; `DocTab::timeline`;
  `BrowserState::reveal_first`. `AmalgamApp::edit_feature` consults the kind's
  `FeatureUiInfo::edit` first (then the UI-07 fallback). **`tool`** (small, focused edit next
  to MESH-23's): the router's hover state is now a `Vec<PickTarget>`, the Hover layer falls back
  to the browser hover and then to the hovered timeline node's output bodies, and the Error
  layer shows the hovered error node's diagnostic entities.

### 2026-11-19: INS-08 (lane A, S05 Measure tool) — small additive framework hooks
Flagged for lane D review. No change to `amg-render`, `amg-doc`, `amg-core` or `amg-jobs`, and no
new dependency. The Measure tool lives in `amg-ui::inspect` (lane A's module).
- **`tool`:** `Tool::overlay(&mut self, out: &mut amg_render::OverlayList, cx: &ToolCx)` (default
  no-op). The router calls it once per frame after `update` (active tools only) into a per-document
  `ToolState::overlay`, cleared when the tool finishes; `AmalgamApp::tool_overlays()` reads it
  (tests). This is the UI-03 "non-committing tools that own overlays" request of 2026-10-20.
- **`viewport`:** `ViewportFrame::overlays: &OverlayList` (the shell passes the active document's
  list). `RenderViewport` copies it into `RenderScene::begin_overlays()` inside its existing
  `with_renderer` lock each frame; labels were already projected and egui-painted there. The
  `NullViewport` ignores it. Struct literals of `ViewportFrame` outside the crate need the new
  field (there are none).
- **`scene`:** `SceneAdapter::instance_world(body, instance) -> Option<Transform>`, the body →
  world transform of a displayed instance (measurements need the rigid `to_world` of the picked
  instance).

### 2026-11-23: INS-03 (lane A, S05 Properties panel) - one additive framework hook
Flagged for lane D review. No change to `amg-render`, `amg-doc`, `amg-core` or `amg-jobs`;
`amg-ui` gains the dependency `amg-materials` (a workspace crate below it; body material and
density come from `Design::mass_material`).
- **`panel`:** `Panel::overlay(&mut self, out: &mut amg_render::OverlayList, env: &CmdEnv)`
  (default no-op). The router calls it once per frame for every instantiated panel of the active
  document, after the active tool's `overlay`, into the same per-document `ToolState::overlay`
  list that the viewport draws (`AmalgamApp::tool_overlays()` reads it in tests). The Properties
  panel draws its centre-of-mass glyph with it.

### 2026-11-23: INS-09 (lane A, S05 Section Analysis) - additive framework hooks
Flagged for lane D review. No change to `amg-render`, `amg-doc`, `amg-core` or `amg-jobs`, and no
new dependency. The tool, layer, browser folder and panel live in `amg-ui::inspect::{section,
section_tool}` (lane A's modules).
- **`layer` (new module):** per-document view layers, for module state that must act on the view
  whatever tool or panel is open (persistent section analyses). `DocLayer` trait (`update(&mut
  self, cx: &mut AppCx, out: &mut LayerOut)`), `DocLayerDef { id, create }`,
  `LayerOut { overlays, sections }`, `ViewSections { planes: Vec<amg_render::SectionPlane>, style:
  SectionStyle }`. `Registrar::doc_layer(def)` (duplicate id = registration conflict),
  `UiRegistry::doc_layers()`. Each `DocTab` holds its layers (`pub(crate) layers`, created the
  first time it is active); the router runs them once per frame for the active document, after
  the tool's and the panels' overlays, in registration order. `AppCx::doc_layer::<T>()`
  (tools, panels, browser providers) and `AmalgamApp::{doc_layer::<T>(), view_sections()}` (tests)
  reach them.
- **`viewport`:** `ViewportFrame::sections: &[amg_render::SectionPlane]` (the shell passes the
  active document's planes). `RenderViewport` keeps the `SectionKey`s it added and rewrites them in
  place each frame (`section_mut`, add / remove only when the count changes). `NullViewport`
  ignores it.
- **`scene`:** `SectionStyle { clipped: Option<BTreeSet<BodyId>>, cap_colors: BTreeMap<BodyId,
  [f32; 3]> }` and `SceneAdapter::{set_section_style, section_of}`: per committed body, RND-06
  `Instance::sectioned` and `Material::cap_color` (instances are rewritten only when their
  description changes; no upload). `InstanceDesc::sectioned: bool` (new field; the GPU backend
  forwards it; struct literals outside the crate: none). The router calls `set_section_style`
  when the layers' style changes.
- **`scene::tess`:** `MeshTessellator` sets `MeshData::closed` for meshes that are closed and
  consistently oriented (`amg_features::inspect::section::is_closed_oriented`, cached per mesh,
  on the tessellation job), so RND-06 caps them; it was always `false`, so no mesh body ever got a
  GPU cap before.

### 2026-11-19: UI-21 (lane D)
- No change outside `amg-ui` and no new dependency.
- `amg-ui` additive API: the `marking` module (see the UI-21 log); `Registrar::{marking_menu,
  context_menu}` as §A; `UiRegistry::{marking_menu_providers, context_menu_providers,
  marking_menu_owners}`; `AmalgamApp::{marking_menu, open_marking_menu, close_marking_menu}`.
  Small focused edits: `tool` (marking state in `ToolState`, popup step in `route_pointer`,
  `Esc` closes the menu first), `shell` (draws the menu), `builtin` (registers the default
  provider), `browser` / `timeline` (append registry context providers to their menus).
- **For other lanes:** the framework provider places these ids once they are registered:
  `sketch.sketch` (UI-09), `solid.press_pull` / `feature.solid.press_pull` /
  `feature.solid.extrude` (S04 / UI-05), `solid.move` / `feature.solid.move`, `edit.delete`,
  `inspect.measure` (S05 INS-08). A lane that wants its own wedges registers a
  `MarkingMenuProvider` with priority > 0; context items go through `ContextMenuProvider`.

### 2026-11-23: IO-15 (lane G, S12 `amg-ui::io`) — small additive framework hooks
Flagged for lane D review. No change outside `amg-ui` (plus `ImportSource::from_path` in
`amg-features::io`, see S12) and one new normal dependency edge: `amg-ui` → `amg-io` (it was a
dev-dependency; already in the workspace). Existing tests pass unchanged.
- **New module `dialog`:** `AppDialog` (`title`, `show(&mut self, &egui::Context, &mut AppCx) -> bool`,
  `as_any`, `as_any_mut`) and `DropHandler = fn(&mut AppCx, &Path) -> bool`. A module dialog that
  is not a tool (a tool's dialog lives and dies with the tool): opened with `AppCx::open_dialog`,
  drawn by the shell once per frame for the active document after the tool dialog
  (`AmalgamApp::draw_dialogs`), shown again when its document becomes active, dropped when that
  document closes. `AmalgamApp::{dialog::<T>, dialog_mut::<T>, dialog_titles}` for tests.
- **`AppCx` (additive):**
  - `open_dialog(Box<dyn AppDialog>)` (end-of-frame `ShellRequest::OpenDialog`).
  - `apply_document(label, FnOnce(&mut Document) -> Result<(), CommandError>)`: a direct document
    mutation at the end of the frame, for commits `CommitBatch` cannot carry
    (`Document::add_feature_baked`, with baked mesh payloads). An `Err` is a toast like a failed
    batch. New private `DocAction::Apply`.
  - `pick_file(FileDialogRequest) -> Receiver<Option<PathBuf>>`: the native Open/Save dialog on the
    existing dialog thread for a module (`DialogPurpose::Reply`); `None` if the user cancels or a
    file dialog is already open.
- **`Registrar::file_drop(DropHandler)`** and `UiRegistry::drop_handlers()`. `AmalgamApp::logic` calls
  `poll_dropped_files`: every path in egui's `raw.dropped_files` that ends in `.amg` opens as a
  document (`open_path`), the others are offered to the handlers in module id order; if none takes
  the file a warning toast says so. The handler runs with an `AppCx` on the active document.
  The window is the drop target (egui reports no drop position); `amg-ui::io` takes every other file.
- **Small focused edits:** `app.rs` (the above plus `AmalgamApp::dialogs`), `file.rs`
  (`DialogPurpose::Reply`, `pick_file_for`), `shell.rs` (one `draw_dialogs` call), `registry.rs`,
  `module.rs`, `lib.rs` (`pub mod dialog`).
- **MESH-23 (`amg-ui::mesh`) not changed.** It had no export entry in `src`; the File ▸ Export entry
  is `io.export` (`amg-ui::io`).

### 2026-11-23: AM-11 (lane G, S16 `amg-ui::am`) - one additive framework hook
Flagged for lane D review. No change to `amg-render`, `amg-doc`, `amg-core` or `amg-jobs`. New
normal dependency edge `amg-ui` → `amg-am` (workspace crate; its third-party dependencies are
already listed). Existing tests pass unchanged.
- **Per-body view overrides (`scene`):** `BodyView { transform: Transform, tint: Option<[f32; 4]> }`
  and `BodyViews { bodies: BTreeMap<BodyId, BodyView> }`; `SceneAdapter::{set_body_views(Option<
  Arc<BodyViews>>, &Design, &Evaluated), body_views()}`. While set, each listed committed body is
  drawn with `transform` as the world transform of every instance (so picking, bounds,
  `instance_world` and highlights follow it) and `tint` as its base colour; every other committed
  body is hidden and not pickable. Preview bodies are not affected. Only instances whose
  description changes are rewritten (no upload). Internally: `Core::views`, `Entry::view_tint`,
  three lines in `sync_instances` / `send_instances`.
- **`layer::LayerOut::body_views: &mut Option<Arc<BodyViews>>`** (new field; `LayerOut` is built
  only by the router): `None` each frame, the last layer that sets it wins. After the layers ran,
  the router compares it with `SceneAdapter::body_views()` (pointer, then value) and calls
  `set_body_views` on a change, so switching documents or workspaces never keeps stale overrides.
- Used by the Manufacture workspace: plate items at their plate transforms (`effective_transform`),
  red when out of the build volume, every other body hidden while the workspace is active.
### 2026-11-23: AM-12 (lane G, S16 `amg-ui::am`) - additive framework hooks
Flagged for lane D review. No change to `amg-render`, `amg-doc`, `amg-core` or `amg-jobs`; no new
dependency. Existing tests pass unchanged.
- **`scene::BodyView::shading: Option<amg_render::Shading>`** (new field next to AM-11's `tint`):
  the RND-08 shading of every instance of the listed body; `None` draws the material. Internally
  `Entry::view_shading`, one line in `sync_instances`, and `send_instances` puts it into the
  description (preview bodies always draw `Shading::Material`).
- **`scene::InstanceDesc::shading: amg_render::Shading`** (new field): the GPU backend copies it
  into `Instance::shading` when it adds or rewrites an instance; the CPU backend stores it (tests
  read it). Changing it rewrites the instance only (RND-08: no geometry upload).
- **`testing::AppHarnessBuilder::job_threads(interactive, background)`**: pool sizes for the
  harness's `JobSystem` instead of the fixed 2 / 1 (0 = the machine's parallelism, like the app's
  `JobSystemConfig::default`). Used by AM-12's timing tests so they measure the app's pool, not a
  2-thread one.
- Used by the Manufacture workspace's Overhang Analysis: each plate item's view carries
  `Shading::Analysis(Analysis::Overhang { .. })` built from AM-02 `shader_params` at the item's
  drawn transform.

### 2026-11-25: UI-09 (lane D)
- No change outside `amg-ui`. New normal dependency edge `amg-ui` → `amg-sketch` (a workspace crate; no new third-party dependency). Two more vendored Tabler icons (`line`, `circle-check`).
- `amg-ui` additive API:
  - the `sketch` module (see the UI-09 log);
  - `AppCx::take_sketch_edit()`;
  - `ToolCx::modifiers()` (keyboard modifiers of the newest routed frame, `ToolFrame::mods`);
  - `layer::LayerOut::active_tool: Option<CmdId>` (new field; `LayerOut` is built only by the router);
  - `icons::ids::{LINE, SKETCH_FINISH}`.
- Only public APIs of `amg-sketch` are used (`solve::{NativeSolver, SolverBackend, DragSession, DofAnalysis, orient_constraint}`, `query::{pick, snap_candidates_with}`, `detect_profiles`), and of `amg-features::sketch` (`SketchFeature::{apply, edit_sketch}`, `SketchEdit`, `PlaneInput`). Nothing is requested from S03 or S10.

### 2026-11-25: INS-10 (lane A, S05 properties and component colours) - additive framework hooks
Flagged for lane D review. All in `amg-ui`; no change to `amg-render` (component colours replace
the base colour of RND-03 instance materials in the scene adapter), `amg-doc`, `amg-core` or
`amg-jobs`; no new dependency. Existing tests pass unchanged.
- **`CmdEnv::view_flags: &BTreeSet<&'static str>`** and **`AppCx::set_view_flag(flag, on)`**:
  per-document view state that is neither saved nor undoable. Needed because a
  `CmdAction::Toggle` getter only sees the `CmdEnv`, which cannot read a document layer.
- **`CmdEnv::instance_paths: &[OccPath]`**: the scene adapter's occurrence-path table
  (`SceneAdapter::occurrence_paths`, new), copied to the document tab each frame, so a panel can
  tell through which occurrence a `PickTarget::Body::instance` was picked.
- **`scene::BodyTints`** (`BTreeMap<BodyId, [f32; 3]>`, linear RGB),
  **`SceneAdapter::{set_body_tints, body_tints}`**: per-body base colours of the committed bodies,
  opacity kept, instances rewritten only (no upload). A `BodyView::tint` (AM-11) wins over them.
- **`layer::LayerOut::body_tints`**: how a `DocLayer` sets them (`None` each frame, the last
  layer that sets it wins), applied like `body_views`.
- Used by S05 INS-10: "Display Component Colours" and the Properties panel. Solid bodies show the
  colours once UI-22 displays them.

### 2026-11-25: UI-22 (lane D)
- No change to `amg-brep` (only `BrepKernel::tessellate`, `TessQuality`, `TessMesh`), `amg-render` (only the existing RND-04 / RND-12 API: `upload_mesh_with_faces`, `upload_edges`, `Instance::edges`, `RenderScene::{insert_edges, remove_edges}`), `amg-doc` or `amg-features`. Nothing to record under S10.
- Workspace: new normal dependency edge `amg-ui` -> `amg-brep` (`Cargo.lock` changes by that line) and `amg-ui` feature `occt`. `amg-app --features occt` already enables the backend through feature unification.
- Additive `amg-ui` API: `scene::{SolidTessellator, DisplayGeometry, solid::quality_preset}`, `BodyTessellator::{tessellate_display, depends_on_quality}`, `MeshUploader::upload_edges`, `UploadedMesh::{with_edges, edge_polylines, into_gpu_parts}`, `CpuSceneBackend::mesh_edges`, `GpuSceneBackend::edges_key`, `SceneAdapter::{change_quality, display_edges}`.
- For S00: the CI `occt` job could also run `cargo nextest run -p amg-ui --features occt`.

### 2026-11-25: UI-12 (lane D)
- No change outside `amg-ui`; `amg-doc` is used through `apply_command`, the `Command` parameter variants, `ParamTable` and `History::revision`. `egui_extras` (already a dependency) now also provides the dialog's table.
- Additive `amg-ui` API: the `params` module, `ExprInput::{id, show_value, accessible_name, checker}`.
- Possible S01 request (not needed by the UI-12 row, nothing implemented): `Command::SetParamUnit { id, unit }` so the dialog's unit column can be edited.

