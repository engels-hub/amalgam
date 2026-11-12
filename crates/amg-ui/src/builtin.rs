//! The framework's own commands (UI-02): search, undo/redo and new document.
//!
//! They are search-and-shortcut only: the app bar already draws Undo/Redo buttons and the File
//! menu (UI-01), and the toolbar groups are filled by the domain lanes.

use crate::command::{CmdAction, CommandDef};
use crate::module::{Registrar, UiModule};

inventory::submit! { UiModule { id: "app", register } }

/// Registers the framework commands.
pub fn register(r: &mut Registrar<'_>) {
    r.command(
        CommandDef::new(
            "app.search",
            "Search commands",
            CmdAction::Run(|cx| cx.open_search()),
        )
        .with_tooltip("Find a command by name")
        .with_shortcut("S")
        .with_keywords(&["find", "command palette", "action"]),
    );
    r.command(
        CommandDef::new("edit.undo", "Undo", CmdAction::Run(|cx| cx.undo()))
            .with_shortcut("Ctrl+Z")
            .with_keywords(&["revert"]),
    );
    r.command(
        CommandDef::new("edit.redo", "Redo", CmdAction::Run(|cx| cx.redo()))
            .with_shortcut("Ctrl+Y")
            .with_alt_shortcut("Ctrl+Shift+Z"),
    );
}
