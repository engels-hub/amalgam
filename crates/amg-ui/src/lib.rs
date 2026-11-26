//! egui app shell, command system, tools, dialogs, browser tree and timeline UI. Lane D owns the framework; domain lanes own their modules.
//!
//! Spec: S11 + domain specs. Lane: D (framework).
//!
//! UI-01 provides the shell: [`AmalgamApp`] with its frame loop (`logic` → `ui` → end-of-frame
//! command queue), the fixed regions (app bar with document tabs and undo/redo, toolbar with
//! workspace switcher, tabs and groups, browser, timeline, status bar), the [`dock::DockHost`]
//! facade over `egui_dock`, [`theme`] tokens, the [`UiRegistry`] fed by `inventory`-submitted
//! [`module::UiModule`]s and the stub modules of every domain lane. The `testing` feature
//! adds [`testing::AppHarness`].
//!
//! UI-02 adds the command registry ([`command`]), the [`shortcut::ShortcutMap`] with scopes and
//! `shortcuts.toml` overrides, and the S-key [`search`].
//!
//! UI-20 adds the [`scene`] adapter (`DocEvent` → render scene, pick keys, [`scene::PickMap`],
//! body tessellators, highlight and preview layers) and the [`pick`] target types.
//!
//! UI-22 displays solid bodies ([`scene::solid`]): BREP-08 tessellation in the adapter's jobs,
//! with face and edge sub-ids. Real solids need the `occt` feature.
//!
//! UI-19 adds [`file`]: New/Open/Open Recent/Save/Save As/Save Compact/Close as jobs, native
//! dialogs on a dedicated thread behind [`file::FileDialogBackend`], the dirty-close prompt and
//! crash recovery (per-document `RecoverySession`, the recovery dialog).
//!
//! UI-03 adds the [`tool`] router ([`tool::Tool`], Esc/Enter, replace semantics, the click-only
//! [`tool::SelectTool`]), [`input`] (`ViewportEvent` synthesis, `CameraView`), [`nav`] (the
//! Fusion navigation preset over S10's `NavController`), the [`pick::PickService`] trait with
//! CPU and GPU implementations and latest-wins hover, [`preview::PreviewDriver`] and the
//! [`select`] types.
//!
//! UI-15 adds the dialog [`widgets`]: `ExprInput`, `SelectionField` (the active one narrows the
//! tool's pick filter), `ChoiceField`, toggle / integer / flip, `DialogFrame` and quantity
//! formatting per the document `UnitPrefs`.
//!
//! UI-08 adds the [`timeline`] strip: node icons from [`timeline::FeatureUiInfo`], groups,
//! rollback-marker drag, drag reorder with valid-range shading, context menu, health badges,
//! hover highlights and the recompute progress ring.
//!
//! UI-21 adds the right-click [`marking`] menu (8 wedges, overflow, "Repeat", click / gesture /
//! hold) with `MarkingMenuProvider`s and `ContextMenuProvider`s for the browser and timeline
//! menus.
//!
//! UI-09 adds [`sketch`] mode: the sketch session (working copy, solver worker, DOF), the Sketch
//! Palette, inference, overlays, select & drag and the Line tool.
//!
//! UI-12 adds the [`params`] dialog ("Change Parameters"): the parameter table with favourites,
//! user and model groups, a filter and inline cycle / unit errors.
//!
//! UI-05 adds [`feature_dialog`]: the generic feature dialog (`GenericFeatureTool`) rendered
//! from every `amg_features::dialog::DialogRegistration`, with its `feature.<kind>` command,
//! toolbar placement and timeline info.
//!
//! UI-06 adds [`manip`]: Distance / Angle / Triad / Plane gizmos drawn as constant-size (90 pt)
//! overlays, CPU hit tests, drags with `Ctrl` snapping; the generic feature dialog binds them to
//! the schema fields that declare a manipulator (both ways) and shows a heads-up value.
//!
//! ASM-10 adds [`assembly`] (lane E): the active component, grounding, pending positions and the
//! assembly menus in the browser.
//!
//! egui is 0.36.2 (QUESTIONS #23); its `Panel` and the eframe `logic`/`ui` split already exist
//! there, so the spec's 0.37 names apply unchanged.

#![warn(missing_docs)]

pub mod app;
pub mod browser;
mod builtin;
pub mod command;
pub mod design;
pub mod dialog;
pub mod dock;
pub mod feature_dialog;
pub mod file;
pub mod guard;
pub mod icons;
pub mod input;
mod jobs;
pub mod layer;
pub mod manip;
pub mod marking;
pub mod module;
pub mod nav;
pub mod notify;
pub mod panel;
pub mod params;
pub mod pick;
pub mod preview;
pub mod registry;
pub mod scene;
pub mod search;
pub mod select;
mod shell;
pub mod shortcut;
pub mod sketch;
pub mod theme;
pub mod timeline;
pub mod tool;
pub mod viewport;
pub mod widgets;

// Domain-lane modules (stubs from UI-01; each lane owns its file).
pub mod am;
pub mod assembly;
pub mod form;
pub mod generative;
pub mod inspect;
pub mod io;
pub mod materials;
pub mod mesh;
pub mod sim;

#[cfg(feature = "testing")]
pub mod testing;

pub use app::{AmalgamApp, AppConfig, AppCx, CmdEnv, CommitBatch, DocTab, DocTabId};
pub use command::{CmdAction, CmdId, CommandDef, Enablement, Enabler, Placement, RegistryError};
pub use notify::{Notification, Notifications};
pub use pick::{PickService, PickTarget};
pub use registry::UiRegistry;
pub use scene::SceneAdapter;
pub use select::{EntityFilter, SelItem, Selection};
pub use shortcut::{Shortcut, ShortcutMap, ShortcutScope};
pub use tool::{FinishReason, InputResult, Tool, ToolCx, ToolInit};

/// Keeps every module's `inventory::submit!` in the final binary (the S01 linker caveat: an
/// object file nothing references may be dropped). [`UiRegistry::build`] calls it.
pub fn link() {
    let registers: [fn(&mut module::Registrar<'_>); 13] = [
        builtin::register,
        design::register,
        am::register,
        assembly::register,
        form::register,
        generative::register,
        inspect::register,
        io::register,
        materials::register,
        mesh::register,
        params::register,
        sim::register,
        sketch::register,
    ];
    std::hint::black_box(registers);
}
