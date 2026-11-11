//! Mesh tools UI (lane C, S07): the MESH tab of the Design workspace (QUESTIONS #25).
//!
//! Stub created by UI-01 (lane D). The owning lane fills in `register`; the framework never
//! needs to change for it (note §5).

use crate::design::DESIGN;
use crate::module::{GroupDef, GroupId, TabDef, TabId};
use crate::module::{Registrar, UiModule};

inventory::submit! { UiModule { id: "mesh", register } }

/// Registers the module's workspaces, tabs, groups and panels.
pub fn register(r: &mut Registrar<'_>) {
    r.toolbar_tab(TabDef {
        id: TabId("mesh"),
        workspace: DESIGN,
        label: "Mesh",
        order: 20,
        contextual: None,
    });
    for (order, (id, label)) in [
        ("mesh.create", "Create"),
        ("mesh.prepare", "Prepare"),
        ("mesh.modify", "Modify"),
        ("mesh.inspect", "Inspect"),
    ]
    .into_iter()
    .enumerate()
    {
        r.toolbar_group(GroupDef {
            id: GroupId(id),
            tab: TabId("mesh"),
            label,
            order: order as i32 * 10,
        });
    }
}
