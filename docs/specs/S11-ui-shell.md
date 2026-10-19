# S11 — UI shell & interaction

| | |
|---|---|
| **Status** | Draft |
| **Priority** | P0 |
| **Crates** | `amg-ui` framework (shell, widgets, generic dialogs, viewport input, browser, timeline, sketch-mode UX), `amg-app` |
| **Lane** | D |
| **Depends on** | S00, S01 (commands), S10 (viewport) |
| **Parity** | Fusion UI layout and interaction model |

## Summary
The application shell, deliberately close to Fusion's layout so muscle memory transfers:
workspace switcher (Design / Mesh / Form / Simulation / Generative / Manufacture-AM),
toolbar with tabs, browser tree, timeline at the bottom, feature dialogs on the right,
marking menu (right-click radial), S-key shortcut box, text command search, selection filters,
on-canvas manipulators.

## Design
- egui + `egui_dock` (ADR-005). Dark and light theme.
- **Command registry:** every tool registered with id, name, icon, workspace/tab, shortcut,
  enabled-predicate. Feeds toolbar, marking menu, search box, keyboard shortcuts, Python API.
- **Tool state machine:** active tool receives viewport input events (hover, click, drag,
  keys) and owns its preview; Esc cancels; Enter/OK commits a `Command`.
- **Generic feature dialogs:** rendered from the feature dialog schema (S04 SOL-01): selection
  fields with filters, expression inputs (with units and parameter autocomplete), dropdowns,
  toggles; live preview with debounce.
- **Manipulators:** distance arrows, angle rings, move/rotate triad, plane drag — linked to
  dialog fields both ways.
- **Selection:** filters (bodies, faces, edges, vertices, sketches, components), window/crossing
  select, selection sets, select-through (cycle overlapping), selection priority.
- **Browser tree:** components, bodies, sketches, construction, origin, analyses, studies;
  visibility toggles, rename, context menus.
- **Timeline:** feature icons, drag to reorder (with dependency validation), rollback marker,
  edit/suppress/group, error/warning badges, hover highlights geometry.
- **Sketch mode UX:** grid/snap, inference lines, auto-constraints, constraint glyphs,
  dimension editing inline, DOF coloring, sketch palette.
- **Preferences:** units, mouse navigation preset (Fusion/SolidWorks/Blender), shortcuts, GPU.
- **Notifications/job panel:** progress and cancel for background jobs.

## Work packages
| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| UI-01 | App shell: window, docking, theme, workspaces, menu, file open/save dialogs | RND-01 | P0 | Todo | |
| UI-02 | Command registry, shortcuts, S-key search, marking menu | UI-01 | P0 | Todo | |
| UI-03 | Tool state machine + viewport input routing | UI-01, RND-04 | P0 | Todo | |
| UI-04 | Selection system + filters + selection sets | UI-03 | P0 | Todo | |
| UI-05 | Generic feature dialog renderer + expression input widget | UI-03, SOL-01, DOC-04 | P0 | Todo | Extrude dialog fully generated |
| UI-06 | Manipulators (distance, angle, triad, plane) linked to dialog fields | UI-05, RND-07 | P0 | Todo | |
| UI-07 | Browser tree | UI-01, DOC-09 | P0 | Todo | |
| UI-08 | Timeline panel | UI-01, DOC-03 | P0 | Todo | |
| UI-09 | Sketch mode UX (tools, inference, constraint glyphs, inline dims) | UI-03, SK-05 | P0 | Todo | Fusion sketch tutorial reproducible |
| UI-10 | Job/progress panel, notifications, error reporting | UI-01, FND-04 | P0 | Todo | |
| UI-11 | Preferences + navigation presets | UI-01 | P1 | Todo | |
| UI-12 | Parameters dialog (table, favorites, expressions) | UI-05 | P0 | Todo | |
