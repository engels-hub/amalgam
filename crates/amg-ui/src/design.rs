//! The framework's own UI module: the **Design** workspace with the tabs Solid, Surface,
//! Inspect and Utilities plus the contextual **Sketch** tab (spec §3). Mesh and Form are tabs
//! of Design registered by lane C (QUESTIONS #25).

use crate::module::{
    ContextId, GroupDef, GroupId, Registrar, TabDef, TabId, UiModule, WorkspaceDef, WorkspaceId,
};

/// The Design workspace.
pub const DESIGN: WorkspaceId = WorkspaceId("design");
/// The sketch context (contextual SKETCH tab).
pub const SKETCH_CONTEXT: ContextId = ContextId("sketch");
/// Tab ids of the Design workspace registered here.
pub mod tabs {
    use crate::module::TabId;
    /// SOLID.
    pub const SOLID: TabId = TabId("design.solid");
    /// SURFACE.
    pub const SURFACE: TabId = TabId("design.surface");
    /// INSPECT.
    pub const INSPECT: TabId = TabId("design.inspect");
    /// UTILITIES.
    pub const UTILITIES: TabId = TabId("design.utilities");
    /// SKETCH (contextual).
    pub const SKETCH: TabId = TabId("design.sketch");
}

inventory::submit! { UiModule { id: "design", register } }

/// Registers the Design workspace.
pub fn register(r: &mut Registrar<'_>) {
    r.workspace(WorkspaceDef::new(DESIGN.0, "Design", 0));
    let tab = |r: &mut Registrar<'_>, id: TabId, label, order, contextual| {
        r.toolbar_tab(TabDef {
            id,
            workspace: DESIGN,
            label,
            order,
            contextual,
        });
    };
    tab(r, tabs::SKETCH, "Sketch", -10, Some(SKETCH_CONTEXT));
    tab(r, tabs::SOLID, "Solid", 0, None);
    tab(r, tabs::SURFACE, "Surface", 10, None);
    tab(r, tabs::INSPECT, "Inspect", 40, None);
    tab(r, tabs::UTILITIES, "Utilities", 50, None);

    let groups: [(TabId, &[(&'static str, &'static str)]); 5] = [
        (
            tabs::SOLID,
            &[
                ("design.solid.create", "Create"),
                ("design.solid.modify", "Modify"),
                ("design.solid.assemble", "Assemble"),
                ("design.solid.construct", "Construct"),
                ("design.solid.inspect", "Inspect"),
                ("design.solid.insert", "Insert"),
                ("design.solid.select", "Select"),
            ],
        ),
        (
            tabs::SURFACE,
            &[
                ("design.surface.create", "Create"),
                ("design.surface.modify", "Modify"),
                ("design.surface.construct", "Construct"),
                ("design.surface.inspect", "Inspect"),
            ],
        ),
        (
            tabs::INSPECT,
            &[
                ("design.inspect.inspect", "Inspect"),
                ("design.inspect.analysis", "Analysis"),
            ],
        ),
        (
            tabs::UTILITIES,
            &[
                ("design.utilities.utility", "Utility"),
                ("design.utilities.addins", "Add-ins"),
            ],
        ),
        (
            tabs::SKETCH,
            &[
                ("design.sketch.create", "Create"),
                ("design.sketch.modify", "Modify"),
                ("design.sketch.constraints", "Constraints"),
                ("design.sketch.finish", "Finish Sketch"),
            ],
        ),
    ];
    for (tab, list) in groups {
        for (order, (id, label)) in list.iter().enumerate() {
            r.toolbar_group(GroupDef {
                id: GroupId(id),
                tab,
                label,
                order: order as i32 * 10,
            });
        }
    }
}
