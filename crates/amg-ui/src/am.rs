//! Additive manufacturing UI (lane G, S16): the Manufacture workspace.
//!
//! Stub created by UI-01 (lane D). The owning lane fills in `register`; the framework never
//! needs to change for it (note §5).

use crate::module::{Registrar, UiModule};
use crate::module::{TabDef, TabId, WorkspaceDef, WorkspaceId};

inventory::submit! { UiModule { id: "am", register } }

/// Registers the module's workspaces, tabs, groups and panels.
pub fn register(r: &mut Registrar<'_>) {
    r.workspace(WorkspaceDef::new("am", "Manufacture", 30));
    r.toolbar_tab(TabDef {
        id: TabId("am.additive"),
        workspace: WorkspaceId("am"),
        label: "Additive",
        order: 0,
        contextual: None,
    });
}
