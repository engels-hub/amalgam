//! The framework's own commands: search, undo/redo (UI-02), the File menu commands (UI-19),
//! the selection commands (UI-04); and the Mesh (UI-20) and Solid
//! (UI-22) body tessellators.
//!
//! Search and undo/redo are search-and-shortcut only (the app bar draws Undo/Redo buttons).
//! The File commands are in the File menu; the toolbar groups are filled by the domain lanes.

use crate::command::{CmdAction, CommandDef, Enablement, Enabler};
use crate::file::FileOp;
use crate::module::{Registrar, UiModule};

inventory::submit! { UiModule { id: "app", register } }

/// Registers the framework commands.
pub fn register(r: &mut Registrar<'_>) {
    crate::icons::register_builtin(r);
    r.tessellator(crate::scene::MeshTessellator);
    r.tessellator(crate::scene::SolidTessellator);
    r.command(
        CommandDef::new(
            "app.search",
            "Search commands",
            CmdAction::Run(|cx| cx.open_search()),
        )
        .with_icon(crate::icons::ids::SEARCH)
        .with_tooltip("Find a command by name")
        .with_shortcut("S")
        .with_keywords(&["find", "command palette", "action"]),
    );
    r.command(
        CommandDef::new("edit.undo", "Undo", CmdAction::Run(|cx| cx.undo()))
            .with_icon(crate::icons::ids::UNDO)
            .with_shortcut("Ctrl+Z")
            .with_keywords(&["revert"]),
    );
    r.command(
        CommandDef::new("edit.redo", "Redo", CmdAction::Run(|cx| cx.redo()))
            .with_icon(crate::icons::ids::REDO)
            .with_shortcut("Ctrl+Y")
            .with_alt_shortcut("Ctrl+Shift+Z"),
    );
    register_file(r);
    register_select(r);
    // UI-21: undo, redo, press-pull / extrude, move, sketch, delete and measure wedges.
    r.marking_menu(Box::new(crate::marking::FrameworkMarkingMenu));
}

/// Selection commands (UI-04, spec §8).
fn register_select(r: &mut Registrar<'_>) {
    r.command(
        CommandDef::new(
            "select.save_set",
            "Save selection set",
            CmdAction::Run(|cx| {
                crate::select::sets::save_selection(cx, None);
            }),
        )
        .with_icon(crate::icons::ids::BOOKMARK)
        .with_tooltip("Keep the current selection as a named set in the design")
        .with_keywords(&["selection set", "group", "remember"])
        .with_enabler(Enabler::When(|env| {
            if env.selection.is_empty() {
                Enablement::disabled("Select something first")
            } else {
                Enablement::Enabled
            }
        })),
    );
    r.command(
        CommandDef::new(
            "select.clear",
            "Clear selection",
            CmdAction::Run(|cx| cx.selection_mut().clear()),
        )
        .with_icon(crate::icons::ids::CLEAR_SELECTION)
        .with_keywords(&["deselect", "none"]),
    );
}

/// File menu commands (spec §15). Labels avoid a bare "Save": that one belongs to the
/// Save / Don't save / Cancel prompt.
fn register_file(r: &mut Registrar<'_>) {
    r.command(
        CommandDef::new(
            "file.new",
            "New document",
            CmdAction::Run(|cx| cx.file(FileOp::New)),
        )
        .with_icon(crate::icons::ids::FILE_NEW)
        .with_shortcut("Ctrl+N")
        .with_keywords(&["create", "design"])
        .in_menu("File", 10),
    );
    r.command(
        CommandDef::new(
            "file.open",
            "Open…",
            CmdAction::Run(|cx| cx.file(FileOp::Open)),
        )
        .with_icon(crate::icons::ids::FOLDER_OPEN)
        .with_tooltip("Open an Amalgam design (.amg)")
        .with_shortcut("Ctrl+O")
        .with_keywords(&["load", "file"])
        .in_menu("File", 20),
    );
    r.command(
        CommandDef::new(
            "file.save",
            "Save document",
            CmdAction::Run(|cx| cx.file(FileOp::Save)),
        )
        .with_icon(crate::icons::ids::SAVE)
        .with_tooltip("Save to the document's file")
        .with_shortcut("Ctrl+S")
        .with_keywords(&["save", "write"])
        .in_menu("File", 30),
    );
    r.command(
        CommandDef::new(
            "file.save_as",
            "Save As…",
            CmdAction::Run(|cx| cx.file(FileOp::SaveAs)),
        )
        .with_icon(crate::icons::ids::SAVE_AS)
        .with_tooltip("Save to a new file")
        .with_shortcut("Ctrl+Shift+S")
        .in_menu("File", 40),
    );
    r.command(
        CommandDef::new(
            "file.save_compact",
            "Save Compact",
            CmdAction::Run(|cx| cx.file(FileOp::SaveCompact)),
        )
        .with_icon(crate::icons::ids::SAVE_COMPACT)
        .with_tooltip("Save without the geometry cache (smaller file, recomputes on open)")
        .with_keywords(&["small", "no cache"])
        .in_menu("File", 50),
    );
    r.command(
        CommandDef::new(
            "file.close",
            "Close document",
            CmdAction::Run(|cx| cx.file(FileOp::Close)),
        )
        .with_icon(crate::icons::ids::CLOSE)
        .with_shortcut("Ctrl+W")
        .in_menu("File", 60),
    );
}
