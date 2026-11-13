//! How a domain module plugs into the shell (spec S11 Interfaces §A, note §5).
//!
//! Each domain lane owns one `amg_ui::<module>` with a `register` function and an
//! `inventory::submit!` of a [`UiModule`]. [`crate::UiRegistry::build`] collects them all, so a
//! lane never edits framework files:
//!
//! ```ignore
//! inventory::submit! { amg_ui::module::UiModule { id: "mesh", register: crate::mesh::register } }
//! ```

use crate::app::AppCx;
use crate::command::CommandDef;
use crate::panel::PanelDef;
use crate::registry::UiRegistry;

/// A UI module: an id plus the function that registers its workspaces, tabs and panels.
#[derive(Clone, Copy, Debug)]
pub struct UiModule {
    /// Module id, e.g. `"mesh"` or `"sim"`. Registration runs in id order.
    pub id: &'static str,
    /// Registers everything the module contributes.
    pub register: fn(&mut Registrar<'_>),
}

inventory::collect!(UiModule);

macro_rules! str_id {
    ($(#[$m:meta])* $name:ident) => {
        $(#[$m])*
        #[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
        pub struct $name(pub &'static str);

        impl $name {
            /// The id string.
            pub fn as_str(self) -> &'static str {
                self.0
            }
        }

        impl std::fmt::Display for $name {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                f.write_str(self.0)
            }
        }
    };
}

str_id!(
    /// A workspace (`"design"`, `"sim"`, …).
    WorkspaceId
);
str_id!(
    /// A toolbar tab (`"design.solid"`, …).
    TabId
);
str_id!(
    /// A toolbar group (`"design.solid.create"`, …).
    GroupId
);
str_id!(
    /// A dockable panel.
    PanelId
);
str_id!(
    /// A UI context such as `"sketch"` or `"form.edit"`; contextual tabs show while it is active.
    ContextId
);
str_id!(
    /// An icon in the icon registry (UI-02 adds the SVG registry; UI-01 shows text only).
    IconId
);

impl IconId {
    /// No icon.
    pub const NONE: IconId = IconId("");
}

/// A workspace in the switcher.
#[derive(Clone, Debug)]
pub struct WorkspaceDef {
    /// Unique id.
    pub id: WorkspaceId,
    /// Switcher label ("Design").
    pub label: &'static str,
    /// Icon.
    pub icon: IconId,
    /// Sort key in the switcher (then the id).
    pub order: i32,
    /// Panels opened in the dock area when the workspace is first shown.
    pub default_panels: Vec<PanelId>,
    /// Called after the workspace became active.
    pub on_enter: Option<fn(&mut AppCx<'_>)>,
    /// Called before another workspace becomes active.
    pub on_exit: Option<fn(&mut AppCx<'_>)>,
}

impl WorkspaceDef {
    /// A workspace with no panels and no hooks.
    pub fn new(id: &'static str, label: &'static str, order: i32) -> WorkspaceDef {
        WorkspaceDef {
            id: WorkspaceId(id),
            label,
            icon: IconId::NONE,
            order,
            default_panels: Vec::new(),
            on_enter: None,
            on_exit: None,
        }
    }
}

/// A toolbar tab of a workspace.
#[derive(Clone, Debug)]
pub struct TabDef {
    /// Unique id.
    pub id: TabId,
    /// The workspace the tab belongs to.
    pub workspace: WorkspaceId,
    /// Label ("Solid"; shown upper-case like Fusion).
    pub label: &'static str,
    /// Sort key inside the workspace (then the id).
    pub order: i32,
    /// Shown only while this context is active (e.g. the SKETCH tab).
    pub contextual: Option<ContextId>,
}

/// A group ("CREATE ⏷") of a toolbar tab.
#[derive(Clone, Debug)]
pub struct GroupDef {
    /// Unique id.
    pub id: GroupId,
    /// The tab the group belongs to.
    pub tab: TabId,
    /// Label ("Create"; shown upper-case with a ⏷).
    pub label: &'static str,
    /// Sort key inside the tab (then the id).
    pub order: i32,
}

/// Handed to [`UiModule::register`]; records what the module contributes.
#[derive(Debug)]
pub struct Registrar<'a> {
    pub(crate) reg: &'a mut UiRegistry,
    pub(crate) module: &'static str,
}

impl Registrar<'_> {
    /// The id of the module being registered.
    pub fn module(&self) -> &'static str {
        self.module
    }

    /// Adds a workspace.
    pub fn workspace(&mut self, def: WorkspaceDef) {
        self.reg.add_workspace(self.module, def);
    }

    /// Adds a toolbar tab (to any workspace, also another module's).
    pub fn toolbar_tab(&mut self, def: TabDef) {
        self.reg.add_tab(self.module, def);
    }

    /// Adds a toolbar group.
    pub fn toolbar_group(&mut self, def: GroupDef) {
        self.reg.add_group(self.module, def);
    }

    /// Adds a command (toolbar, menu, search and shortcut entry). A duplicate id panics in
    /// debug builds and is logged and dropped in release builds.
    pub fn command(&mut self, def: CommandDef) {
        self.reg.add_command(self.module, def);
    }

    /// Adds a dockable panel.
    pub fn panel(&mut self, def: PanelDef) {
        self.reg.add_panel(self.module, def);
    }

    /// Adds a body tessellator (UI-20). The most recently registered one that handles a
    /// `BodyGeom` variant is used.
    pub fn tessellator(&mut self, t: impl crate::scene::BodyTessellator) {
        self.reg
            .add_tessellator(self.module, std::sync::Arc::new(t));
    }
}
