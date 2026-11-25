//! Import/export UI (lane G, S12 IO-15): File ▸ Import ▸ Insert Mesh…, File ▸ Export ▸
//! Export…, drag and drop onto the window, the import and export dialogs.
//!
//! - **Insert Mesh / Import** ([`ImportDialog`]). The file is read once on a job (progress and
//!   Cancel in the dialog and in the jobs panel, UI-10) with the raw options of
//!   [`amg_features::io::raw_options`]; the dialog shows format, a unit dropdown (declared /
//!   assumed), the extents in mm and inches under the chosen unit and up axis, the build-volume
//!   fit (the design's print setup, else the remembered default bed), merge or separate, and
//!   the reader's warnings. A unit change recomputes the shown extents from the raw bounding
//!   box: the file is not read again. OK builds the "Insert <file>" transaction on a second
//!   job and applies it as one undo entry ([`amg_features::io::ImportPlan`]).
//! - **Drag and drop.** Every dropped file that is not an `.amg` design opens the import dialog
//!   (a registered [`crate::dialog::DropHandler`]); the reader sniffs the format, and a file it
//!   cannot read ends in the `IoError` toast with its hint.
//! - **Export** ([`ExportDialog`]): format, selection (all visible / selected bodies), unit,
//!   3MF flavour (Generic, Orca/Bambu), the refinement preset when a solid body exists
//!   (IO-09: Low, Medium, High, Custom), the validity report with "Repair first" (adds MESH-04
//!   `mesh.repair` features, then exports) and "Export anyway", progress and Cancel.
//! - **Remembered choices** per format in `io-prefs.json` ([`prefs`]).
//!
//! The dialogs are [`crate::dialog::AppDialog`]s; no file is read or written on the UI thread.

pub mod export;
pub mod import;
pub mod prefs;

use crate::command::{CmdAction, CmdId, CommandDef, Enablement, Enabler};
use crate::module::{Registrar, UiModule};

pub use export::{ExportDialog, Outcome, export_formats};
pub use import::{
    BACKGROUND_THRESHOLD, ImportDialog, IoJobError, Loaded, UnitSel, bed_fit, fmt_size,
    up_axis_extents,
};
pub use prefs::{ExportPref, ImportPref, IoPrefs, set_prefs_path};

inventory::submit! { UiModule { id: "io", register } }

/// File ▸ Import ▸ Insert Mesh…
pub const INSERT_MESH_CMD: CmdId = CmdId("io.insert_mesh");
/// File ▸ Export ▸ Export…
pub const EXPORT_CMD: CmdId = CmdId("io.export");

fn has_body(env: &crate::app::CmdEnv<'_>) -> Enablement {
    if env.evaluated.bodies().next().is_some() {
        Enablement::Enabled
    } else {
        Enablement::disabled("The design has no body to export")
    }
}

/// Registers the module's commands and the file-drop handler.
pub fn register(r: &mut Registrar<'_>) {
    r.command(
        CommandDef::new(
            INSERT_MESH_CMD.0,
            "Insert Mesh…",
            CmdAction::Run(|cx| {
                let dialog = ImportDialog::pick(cx);
                cx.open_dialog(Box::new(dialog));
            }),
        )
        .with_icon(crate::icons::ids::IMPORT)
        .with_tooltip("Insert an STL, 3MF, OBJ or PLY file as a mesh body")
        .with_keywords(&["import", "stl", "3mf", "obj", "ply", "open mesh", "load"])
        .in_menu("File/Import", 0),
    );
    r.command(
        CommandDef::new(
            EXPORT_CMD.0,
            "Export…",
            CmdAction::Run(|cx| {
                let dialog = ExportDialog::new(cx);
                cx.open_dialog(Box::new(dialog));
            }),
        )
        .with_icon(crate::icons::ids::EXPORT)
        .with_tooltip("Export the bodies as STL, 3MF, OBJ or PLY")
        .with_keywords(&[
            "save", "stl", "3mf", "obj", "ply", "slicer", "orca", "print",
        ])
        .with_enabler(Enabler::When(has_body))
        .in_menu("File/Export", 0),
    );
    r.file_drop(import::on_drop);
}
