//! Generative design / topology optimisation UI (lane F, S15): the Generative workspace.
//!
//! Stub created by UI-01 (lane D). The owning lane fills in `register`; the framework never
//! needs to change for it (note §5).

use crate::module::{Registrar, UiModule};
use crate::module::{TabDef, TabId, WorkspaceDef, WorkspaceId};

inventory::submit! { UiModule { id: "generative", register } }

/// Registers the module's workspaces, tabs, groups and panels.
pub fn register(r: &mut Registrar<'_>) {
    r.workspace(WorkspaceDef::new("generative", "Generative Design", 20));
    r.toolbar_tab(TabDef {
        id: TabId("generative.define"),
        workspace: WorkspaceId("generative"),
        label: "Define",
        order: 0,
        contextual: None,
    });
}
