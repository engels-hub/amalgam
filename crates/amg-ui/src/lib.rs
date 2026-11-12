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
//! egui is 0.36.2 (QUESTIONS #23); its `Panel` and the eframe `logic`/`ui` split already exist
//! there, so the spec's 0.37 names apply unchanged.

#![warn(missing_docs)]

pub mod app;
mod builtin;
pub mod command;
pub mod design;
pub mod dock;
mod jobs;
pub mod module;
pub mod notify;
pub mod panel;
pub mod registry;
pub mod search;
mod shell;
pub mod shortcut;
pub mod theme;
pub mod viewport;

// Domain-lane modules (stubs from UI-01; each lane owns its file).
pub mod am;
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
pub use registry::UiRegistry;
pub use shortcut::{Shortcut, ShortcutMap, ShortcutScope};

/// Keeps every module's `inventory::submit!` in the final binary (the S01 linker caveat: an
/// object file nothing references may be dropped). [`UiRegistry::build`] calls it.
pub fn link() {
    let registers: [fn(&mut module::Registrar<'_>); 10] = [
        builtin::register,
        design::register,
        am::register,
        form::register,
        generative::register,
        inspect::register,
        io::register,
        materials::register,
        mesh::register,
        sim::register,
    ];
    std::hint::black_box(registers);
}
