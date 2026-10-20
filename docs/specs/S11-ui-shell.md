# S11 — UI shell & interaction

| | |
|---|---|
| **Status** | Refined |
| **Priority** | P0 |
| **Crates** | `amg-ui` framework modules (`app`, `module`, `command`, `shortcut`, `search`, `marking`, `tool`, `input`, `nav`, `pick`, `select`, `widgets`, `dialog`, `manip`, `browser`, `timeline`, `sketch`, `scene`, `jobs`, `notify`, `prefs`, `theme`, `icons`, `dock`, `testing`), `amg-app` |
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
  `egui_extras` (features `svg`, `serde`): all **0.37.0**. **wgpu 30.0.1** (egui-wgpu 0.37
  requires `wgpu ^30.0`). `egui_kittest` 0.37.0 (`snapshot`, `wgpu`, `eframe`) is a
  dev-dependency.
- **Docking (ADR-005):** `egui_dock`, behind the `amg_ui::dock::DockHost` facade.
  - As of 2026-10-20 egui_dock 0.21.1 still requires egui 0.36, and no 0.37 release exists.
  - If that is still true when UI-01 starts, the egui family is pinned to **0.36.2** until
    egui_dock 0.22 ships. egui-wgpu 0.36.2 also uses wgpu 30, so FND-09 and S10 are not
    affected.
  - Second fallback: `egui_tiles` 0.18 (already on egui 0.37).
  - Open question 1 (owner).
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
| UI-01 | App shell: `amg-app` start-up (diag, UI-thread mark, `Gpu` → `WgpuSetup::Existing`, `JobSystem` notifier → repaint), eframe 0.37 frame loop (`logic`/`ui`, end-of-frame `CommandQueue`), fixed panels + `DockHost` facade (egui_dock), theme tokens (light/dark/system), workspace/tab registry + `UiModule` inventory + stub modules for every domain lane, document tabs, undo/redo buttons, status bar, `amg_ui::testing::AppHarness` (kittest, `NullViewport`, `CpuPickService` stub) | RND-01, FND-07 | P0 | Todo | 1. `amg-app` opens on the owner's machine showing the Fusion layout (app bar, toolbar, browser, timeline, status bar, viewport tab with the RND-01 mesh). Cold start to first interactive frame < 2 s (tracing span `startup`). 2. Kittest: switching workspace via the switcher changes the visible tabs (`get_by_label`). A test module registering a workspace + tab through `inventory` appears without editing framework files. 3. Light/dark snapshot tests pass on WARP. 4. With no input and no jobs, the app does not repaint (frame counter stays constant over 1 s). 5. A test job's notifier wakes the UI within one frame. 6. Two document tabs keep independent undo stacks (kittest). |
| UI-02 | Command registry (`CommandDef`, `Placement`, `Enabler`, dynamic registration), toolbar tabs/groups/dropdowns, menus, `ShortcutMap` with scopes and `shortcuts.toml` overrides, S-key search (own fuzzy scorer, recency) | UI-01 | P0 | Todo | 1. Kittest: pressing `S`, typing "remsh" lists a registered `test.remesh` first, and Enter runs it. 2. Disabled commands show their reason in the tooltip and are skipped by Enter. 3. A duplicate `CmdId` panics in debug (unit test). 4. Shortcut scope test: `L` in sketch scope and `L` in global scope resolve correctly. Single letters are ignored while a text field has focus. 5. Search over 1 000 synthetic commands < 1 ms per keystroke (criterion). 6. `register_dynamic` rejects ids without the `script.` prefix. |
| UI-03 | `Tool` trait + router (priority chain, Esc/Enter, replace semantics), `ViewportEvent` synthesis from egui input, `PickService` trait + S10 implementation + latest-wins hover, Fusion navigation preset over S10 camera, `PreviewDriver`, `spawn_tracked`, `SelectTool` (click only) | UI-02, RND-04, RND-11 | P0 | Todo | 1. Kittest with a scripted test tool: events arrive in order, Esc cancels (`FinishReason::Cancel`), Enter commits one undo entry, and starting another command gives `Replaced`. 2. Middle-drag orbits even while a tool is active that ignores the event. 3. Hover pick → highlight within ≤ 2 frames on the owner's GPU and with `CpuPickService`. 4. `PreviewDriver`: 20 rapid `set` calls → exactly one preview computed (test counter feature), and none enters the undo stack. 5. No UI-thread frame > 16 ms while a 2 s test job runs (tracing assertion). |
| UI-20 | Scene adapter: `DocEvent` → `RenderScene` (RND-03), `RenderEntityKey` allocation, `PickMap`, `BodyTessellator` registry with Mesh tessellator, highlight layers, preview layer feed, `CpuPickService` real implementation | UI-01, RND-03, DOC-09 | P0 | Todo | 1. Scripted test: add/edit/delete 3 mesh bodies → the RenderScene contains exactly the live bodies, and an unchanged body is never re-uploaded (upload counter). 2. `DocumentReplaced` rebuilds the scene, and keys are not reused within a session. 3. `PickMap` round-trips every face index of a 10 k-triangle body. 4. A registered test tessellator for a stub `BodyGeom` variant is used. 5. Updating 1 changed body in a 100-body scene < 1 ms of UI-thread time. |
| UI-04 | Selection system: `Selection`, `SelItem`, `EntityFilter` bar, select priority, click/Ctrl/Shift rules, window/crossing box (S10 region pick), select-through list, selection sets (`ExtRecord ui.selection_sets`), revalidation on events | UI-03, UI-20 | P0 | Todo | 1. Kittest on a fixture with 3 mesh bodies: window select left→right picks only fully enclosed bodies, and right→left also picks touching ones. 2. With the filter set to faces only, a click on an edge selects the adjacent face. 3. Select-through lists all 3 stacked bodies in depth order. 4. Save a selection set → reload the file → restore gives the same `EntityRef`s, and undo removes the set. 5. Deleting a selected body drops it from the selection without error. |
| UI-15 | Dialog widgets: `ExprInput` (live parse via `ParamTable::try_eval`, error spans, autocomplete of params/functions/units, nudge, bound-param display), `SelectionField`, `ChoiceField`, toggle/int/flip, `DialogFrame` (banner, OK enablement), quantity formatting per `UnitPrefs` | UI-04, DOC-04 | P0 | Todo | 1. Kittest: typing `d1 + 5` in a length field with `d1 = 10 mm` shows `15.00 mm`. `12 kg` shows a unit-mismatch error underlined at the right span, and OK is disabled. 2. Typing `d` offers every parameter starting with `d`, and Tab completes. 3. `12,5` shows the `DecimalComma` hint (QUESTIONS #11). 4. Only one `SelectionField` is active at a time, and the active one sets the pick filter. 5. All widgets are queryable by label (AccessKit). |
| UI-05 | Generic feature dialog renderer: `GenericFeatureTool` from `DialogRegistration` (conditions, auto-focus, live preview with debounce, preview-health banner), edit mode (preview rollback, `EditFeature` + `SetParamExpr` in one transaction), automatic `feature.<kind>` commands + toolbar placement + `FeatureUiInfo` | UI-15, SOL-01, DOC-12 | P0 | Todo | 1. The Extrude dialog is generated entirely from SOL-01's schema (no extrude-specific UI code; grep test). Changing extent type shows/hides the right fields. 2. Kittest: select a profile, type a distance → the preview appears ≤ debounce + 100 ms on a typical part. OK yields one undo entry with `AddFeature`. 3. Edit mode: changing the distance of an existing extrude keeps its model parameter `dN` and changes its expression. 4. An invalid input disables OK with the field error. Cancel leaves the document unchanged (revision equal). |
| UI-06 | Manipulators: Distance, Angle, Triad, Plane gizmos via overlays (constant 90 px), CPU hit test, drag ↔ field binding both ways, Ctrl snapping, heads-up value | UI-05, RND-07 | P0 | Todo | 1. Dragging the extrude arrow by a scripted world delta of 12.5 mm writes `12.5 mm` into the field (prefs precision), and editing the field moves the arrow (kittest via `world_to_screen`). 2. Ctrl-drag snaps to 1 mm / 5°. 3. Triad: each of 10 parts hit-tests correctly in a 6-view matrix (unit test on `CameraView`). 4. Gizmo screen size stays 90 ± 2 px across a 1000× zoom range. |
| UI-07 | Browser tree: framework nodes (settings, named views, origin, components/bodies/sketches/construction, selection sets), `BrowserProvider` API, virtualised rows, eye toggles (`SetBodyProps`, `SetOccurrenceVisible`, `ui.visibility` ExtRecord), rename, context menu, hover highlight, click select, double-click edit | UI-04, DOC-09 | P0 | Todo | 1. Kittest on a fixture with 2 components × 3 bodies: hiding a body hides it in the scene, and undo shows it again. Renaming a body persists through save/load. 2. A test provider adds a node under Root without framework changes. 3. 10 000 synthetic bodies: browser frame cost < 1 ms (criterion), and the tree is rebuilt only on affecting events (counter). 4. Hovering a row highlights the body (highlight layer contains it). |
| UI-08 | Timeline panel: node icons via `FeatureUiInfo`, groups, rollback marker drag (`SetMarker`), drag reorder with valid-range shading (`MoveFeature`), context menu, health badges + tooltips, hover highlight of outputs and error entities, recompute progress ring, hidden in Direct mode | UI-04, DOC-05, DOC-12 | P0 | Todo | 1. Kittest on a 20-node test timeline: dragging the marker to index 5 sends `SetMarker{5}`, and nodes ≥ 5 render as rolled back. 2. Dragging a node before its input is refused (no command sent, red shading shown). A valid move sends one `MoveFeature`. 3. An erroring test feature shows an error badge with the diagnostic message as tooltip. 4. Suppress/rename/delete/group via the context menu each give one undo entry. 5. 500 nodes render < 1 ms (criterion). |
| UI-10 | Job/progress panel (`JobSystem::jobs()`, grouped, cancel), status-bar recompute/job indicators, toasts from `Diagnostic` (severity, hint, details via `report`, copy, open log folder), cancelled-is-not-error rule, GPU device-lost banner | UI-01, FND-04, FND-07 | P0 | Todo | 1. Kittest: a 2 s test job shows progress; pressing Cancel resolves it as `Cancelled` within 200 ms and no error toast appears. 2. A failing job with an `amg_core::Error::InvalidArgument` shows the user message and hint, and "Details" contains the `report` chain. 3. Info toasts auto-hide after 6 s, and errors stay. 4. Simulated `is_lost()` shows the banner and Save stays enabled. |
| UI-19 | File operations & recovery: New/Open/Open Recent/Save/Save As/Save Compact, `rfd` on a dedicated thread parented to the window, save/load as jobs with progress, dirty-close prompt (`CancelClose` + `Modal`), recovery dialog from `find_recoverable`, `RecoverySession::on_commit` per batch, `Placement::Menu` slots for Import/Export | UI-10, DOC-07, DOC-14 | P0 | Todo | 1. Kittest with a fake file-dialog backend (trait): save → reopen gives an equal `design.json`. 2. Closing a dirty document shows the prompt, and Cancel keeps the app open. 3. Kill test: commit 5 commands, abort the process, restart → the recovery dialog offers the document and "Open recovered" restores it. 4. Opening a 50 MB file never blocks a frame > 16 ms (tracing assertion). |
| UI-21 | Marking menu: radial 8 wedges + overflow + repeat-last, click/gesture/hold modes, `MarkingMenuProvider` + `ContextMenuProvider` APIs, framework default provider | UI-03, UI-04 | P0 | Todo | 1. Kittest: right-click opens the menu, clicking the N wedge runs its command. A right-drag gesture toward E runs E's command without the menu staying open. 2. "Repeat <last>" runs the previous command. 3. A test provider adds an entry for a selected mesh body only (context check). 4. Every wedge is queryable by label. |
| UI-12 | Parameters dialog: table (favourite, name, unit, expression, value, comment), user/model grouping, filter, add/delete/rename/edit/favourite via S01 commands, inline cycle/unit errors | UI-15, DOC-05 | P0 | Todo | 1. Kittest: add user param `w = 20 mm`, set a model param to `w * 2` → value `40.00 mm`. Renaming `w` → `width` rewrites the model param text. 2. `a = b`, `b = a` shows a cycle error on the second edit, and the document is unchanged. 3. Each edit is one undo entry. 4. 1 000 params render < 2 ms (criterion). |
| UI-22 | Solid body display: `BodyTessellator` for `BodyGeom::Solid` via BREP-08 (per-face ranges → face sub-ids, edge polylines → edge sub-ids), quality presets, latest-wins re-tessellation | UI-20, BREP-08 | P0 | Todo | 1. A STEP test part (≥ 1 000 faces) displays with every face and edge pickable (PickMap round-trip). 2. Editing an upstream parameter re-tessellates only changed bodies (counter). 3. Tessellation runs on a job (no frame > 16 ms). |
| UI-09 | Sketch mode core: Create/Edit/Finish Sketch, plane pick, look-at, SKETCH contextual tab + Sketch Palette (grid, snap, show profile/points/dimensions/constraints, slice), working copy + `SolverState` job (latest-wins), `analyze` → DOF colouring + status DOF, overlay rendering of curves/points/profiles, select & drag (`DragSession` → `CommitDrag`), inference engine on `query::snap_candidates` (glyphs, alignment lines, auto-constraints, Ctrl suppress), heads-up numeric input, Line tool | UI-03, UI-04, UI-15, SK-04, SK-05, SK-19, SK-20, RND-07 | P0 | Todo | 1. Kittest: create a sketch on XY, draw a closed 4-segment line chain using inference → 4 coincident + 2 horizontal + 2 vertical constraints auto-added, 1 profile detected. 2. DOF shown = 4 for that rectangle, entities coloured `Under`. 3. Dragging a corner for 1 000 frames: p95 UI-thread time < 2 ms per frame and no frame > 16 ms. Release = exactly one undo entry. 4. Undo inside the session reloads the working copy. 5. Finish Sketch leaves sketch mode and the timeline shows the sketch node with health Ok. |
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
2. **(owner)** Several documents as tabs in one window, as in Fusion (*default*), or one
   document per window?
3. **(owner)** Workspace grouping. Follow current Fusion: Mesh and Form are tabs in Design;
   Simulation, Generative and Manufacture-AM are workspaces (*default*). Or keep Mesh and Form
   as separate workspaces as the Draft summary lists? Only lane C's registration data
   changes.
4. Feature dialogs float (anchored top-right, movable) instead of docking. Decided: Fusion
   parity. Revisit after Phase 1 use.
5. Phase 1 mesh dialogs: MESH-23 hand-builds them (QUESTIONS #22). UI-15 widgets are in Phase 1
   so they can be used. MESH-23 adding the dependency UI-15 is a request to S07.

## Cross-spec requests
Requests to other specs. The owning spec decides; nothing is edited here.

**S00:**
- **R-S00-1:** `[workspace.dependencies]` uses `wgpu = "30.0.1"` (S00 lists `"29"`).
- **R-S00-2:** also add `egui`, `eframe`, `egui-wgpu`, `egui_extras` = `0.37.0`, and
  `egui_kittest` = `0.37.0` as a dev-dependency.
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
