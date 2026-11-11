//! Form (SubD) UI (lane C, S09): the contextual FORM tab of Design, shown in the `form.edit` context.
//!
//! Stub created by UI-01 (lane D). The owning lane fills in `register`; the framework never
//! needs to change for it (note §5).

use crate::design::DESIGN;
use crate::module::{ContextId, TabDef, TabId};
use crate::module::{Registrar, UiModule};

inventory::submit! { UiModule { id: "form", register } }

/// Registers the module's workspaces, tabs, groups and panels.
pub fn register(r: &mut Registrar<'_>) {
    r.toolbar_tab(TabDef {
        id: TabId("form"),
        workspace: DESIGN,
        label: "Form",
        order: -5,
        contextual: Some(ContextId("form.edit")),
    });
}
