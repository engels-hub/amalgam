//! Simulation UI (lane F, S13/S14): the Simulation workspace.
//!
//! Stub created by UI-01 (lane D). The owning lane fills in `register`; the framework never
//! needs to change for it (note §5).

use crate::module::{Registrar, UiModule};
use crate::module::{TabDef, TabId, WorkspaceDef, WorkspaceId};

inventory::submit! { UiModule { id: "sim", register } }

/// Registers the module's workspaces, tabs, groups and panels.
pub fn register(r: &mut Registrar<'_>) {
    r.workspace(WorkspaceDef::new("sim", "Simulation", 10));
    r.toolbar_tab(TabDef {
        id: TabId("sim.setup"),
        workspace: WorkspaceId("sim"),
        label: "Setup",
        order: 0,
        contextual: None,
    });
}
