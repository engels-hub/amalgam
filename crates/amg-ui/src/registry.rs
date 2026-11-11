//! The registry of everything modules contribute (workspaces, tabs, groups, panels).

use std::collections::BTreeMap;
use std::fmt;

use crate::module::{
    ContextId, GroupDef, GroupId, PanelId, Registrar, TabDef, TabId, UiModule, WorkspaceDef,
    WorkspaceId,
};
use crate::panel::PanelDef;

/// Everything registered by the [`UiModule`]s, sorted for display.
///
/// Ids are unique per kind. A duplicate panics in debug builds and is logged and dropped in
/// release builds (spec §4, note §5), so a broken module cannot take the shell down.
#[derive(Default)]
pub struct UiRegistry {
    modules: Vec<&'static str>,
    workspaces: BTreeMap<WorkspaceId, (&'static str, WorkspaceDef)>,
    tabs: BTreeMap<TabId, (&'static str, TabDef)>,
    groups: BTreeMap<GroupId, (&'static str, GroupDef)>,
    panels: BTreeMap<PanelId, (&'static str, PanelDef)>,
}

impl fmt::Debug for UiRegistry {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("UiRegistry")
            .field("modules", &self.modules)
            .field("workspaces", &self.workspaces.keys().collect::<Vec<_>>())
            .field("tabs", &self.tabs.keys().collect::<Vec<_>>())
            .field("groups", &self.groups.len())
            .field("panels", &self.panels.keys().collect::<Vec<_>>())
            .finish()
    }
}

impl UiRegistry {
    /// Collects every `inventory`-submitted [`UiModule`] (framework and domain modules) and
    /// runs their `register` functions in module-id order.
    pub fn build() -> UiRegistry {
        crate::link();
        let mut modules: Vec<&UiModule> = inventory::iter::<UiModule>.into_iter().collect();
        modules.sort_by_key(|m| m.id);
        Self::from_modules(modules)
    }

    /// Builds a registry from an explicit module list (tests), in the given order.
    pub fn from_modules<'m>(modules: impl IntoIterator<Item = &'m UiModule>) -> UiRegistry {
        let mut reg = UiRegistry::default();
        for m in modules {
            if reg.modules.contains(&m.id) {
                conflict("module", m.id, m.id, m.id);
                continue;
            }
            reg.modules.push(m.id);
            (m.register)(&mut Registrar {
                reg: &mut reg,
                module: m.id,
            });
        }
        reg
    }

    /// Ids of the registered modules, in registration order.
    pub fn modules(&self) -> &[&'static str] {
        &self.modules
    }

    pub(crate) fn add_workspace(&mut self, module: &'static str, def: WorkspaceDef) {
        if let Some((first, _)) = self.workspaces.get(&def.id) {
            conflict("workspace", def.id.0, first, module);
            return;
        }
        self.workspaces.insert(def.id, (module, def));
    }

    pub(crate) fn add_tab(&mut self, module: &'static str, def: TabDef) {
        if let Some((first, _)) = self.tabs.get(&def.id) {
            conflict("toolbar tab", def.id.0, first, module);
            return;
        }
        self.tabs.insert(def.id, (module, def));
    }

    pub(crate) fn add_group(&mut self, module: &'static str, def: GroupDef) {
        if let Some((first, _)) = self.groups.get(&def.id) {
            conflict("toolbar group", def.id.0, first, module);
            return;
        }
        self.groups.insert(def.id, (module, def));
    }

    pub(crate) fn add_panel(&mut self, module: &'static str, def: PanelDef) {
        if let Some((first, _)) = self.panels.get(&def.id) {
            conflict("panel", def.id.0, first, module);
            return;
        }
        self.panels.insert(def.id, (module, def));
    }

    /// Workspaces in switcher order (`order`, then id).
    pub fn workspaces(&self) -> Vec<&WorkspaceDef> {
        let mut v: Vec<&WorkspaceDef> = self.workspaces.values().map(|(_, d)| d).collect();
        v.sort_by_key(|d| (d.order, d.id));
        v
    }

    /// One workspace.
    pub fn workspace(&self, id: WorkspaceId) -> Option<&WorkspaceDef> {
        self.workspaces.get(&id).map(|(_, d)| d)
    }

    /// The first workspace in switcher order.
    pub fn default_workspace(&self) -> Option<WorkspaceId> {
        self.workspaces().first().map(|d| d.id)
    }

    /// Every tab of a workspace (contextual ones included), in display order.
    pub fn all_tabs(&self, ws: WorkspaceId) -> Vec<&TabDef> {
        let mut v: Vec<&TabDef> = self
            .tabs
            .values()
            .map(|(_, d)| d)
            .filter(|t| t.workspace == ws)
            .collect();
        v.sort_by_key(|d| (d.order, d.id));
        v
    }

    /// The tabs visible in `ws` while `contexts` are active: every non-contextual tab plus the
    /// contextual tabs whose context is active.
    pub fn visible_tabs(&self, ws: WorkspaceId, contexts: &[ContextId]) -> Vec<&TabDef> {
        self.all_tabs(ws)
            .into_iter()
            .filter(|t| t.contextual.is_none_or(|c| contexts.contains(&c)))
            .collect()
    }

    /// One tab.
    pub fn tab(&self, id: TabId) -> Option<&TabDef> {
        self.tabs.get(&id).map(|(_, d)| d)
    }

    /// Groups of a tab, in display order.
    pub fn groups(&self, tab: TabId) -> Vec<&GroupDef> {
        let mut v: Vec<&GroupDef> = self
            .groups
            .values()
            .map(|(_, d)| d)
            .filter(|g| g.tab == tab)
            .collect();
        v.sort_by_key(|d| (d.order, d.id));
        v
    }

    /// One panel.
    pub fn panel(&self, id: PanelId) -> Option<&PanelDef> {
        self.panels.get(&id).map(|(_, d)| d)
    }

    /// Every panel, by id.
    pub fn panels(&self) -> impl Iterator<Item = &PanelDef> + '_ {
        self.panels.values().map(|(_, d)| d)
    }

    /// The registered panel whose id string is `s` (layout files store ids as strings).
    pub fn panel_by_str(&self, s: &str) -> Option<PanelId> {
        self.panels.keys().copied().find(|p| p.0 == s)
    }

    /// The module that registered a workspace.
    pub fn workspace_owner(&self, id: WorkspaceId) -> Option<&'static str> {
        self.workspaces.get(&id).map(|(m, _)| *m)
    }

    /// The module that registered a tab.
    pub fn tab_owner(&self, id: TabId) -> Option<&'static str> {
        self.tabs.get(&id).map(|(m, _)| *m)
    }
}

/// A duplicate id: panic in debug, log and drop in release.
fn conflict(kind: &str, id: &str, first: &str, second: &str) {
    let msg = format!("duplicate {kind} id `{id}`: registered by `{first}`, again by `{second}`");
    if cfg!(debug_assertions) {
        panic!("{msg}");
    }
    tracing::error!("{msg}; the second registration is ignored");
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ws_a(r: &mut Registrar<'_>) {
        r.workspace(WorkspaceDef::new("a", "A", 5));
        r.toolbar_tab(TabDef {
            id: TabId("a.two"),
            workspace: WorkspaceId("a"),
            label: "Two",
            order: 2,
            contextual: None,
        });
        r.toolbar_tab(TabDef {
            id: TabId("a.one"),
            workspace: WorkspaceId("a"),
            label: "One",
            order: 1,
            contextual: None,
        });
        r.toolbar_tab(TabDef {
            id: TabId("a.ctx"),
            workspace: WorkspaceId("a"),
            label: "Ctx",
            order: 0,
            contextual: Some(ContextId("ctx")),
        });
        r.toolbar_group(GroupDef {
            id: GroupId("a.one.z"),
            tab: TabId("a.one"),
            label: "Z",
            order: 0,
        });
        r.toolbar_group(GroupDef {
            id: GroupId("a.one.y"),
            tab: TabId("a.one"),
            label: "Y",
            order: 0,
        });
    }

    fn ws_b(r: &mut Registrar<'_>) {
        r.workspace(WorkspaceDef::new("b", "B", 1));
    }

    fn dup(r: &mut Registrar<'_>) {
        r.workspace(WorkspaceDef::new("a", "Again", 0));
    }

    #[test]
    fn ordering_is_by_order_then_id() {
        let ma = UiModule {
            id: "ma",
            register: ws_a,
        };
        let mb = UiModule {
            id: "mb",
            register: ws_b,
        };
        let reg = UiRegistry::from_modules([&ma, &mb]);
        let ids: Vec<_> = reg.workspaces().iter().map(|w| w.id.0).collect();
        assert_eq!(ids, ["b", "a"]);
        assert_eq!(reg.default_workspace(), Some(WorkspaceId("b")));
        let tabs: Vec<_> = reg
            .visible_tabs(WorkspaceId("a"), &[])
            .iter()
            .map(|t| t.id.0)
            .collect();
        assert_eq!(tabs, ["a.one", "a.two"]);
        let with_ctx: Vec<_> = reg
            .visible_tabs(WorkspaceId("a"), &[ContextId("ctx")])
            .iter()
            .map(|t| t.id.0)
            .collect();
        assert_eq!(with_ctx, ["a.ctx", "a.one", "a.two"]);
        let groups: Vec<_> = reg.groups(TabId("a.one")).iter().map(|g| g.id.0).collect();
        assert_eq!(groups, ["a.one.y", "a.one.z"]);
        assert_eq!(reg.workspace_owner(WorkspaceId("a")), Some("ma"));
    }

    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "duplicate workspace id `a`")]
    fn duplicate_workspace_panics_in_debug() {
        let ma = UiModule {
            id: "ma",
            register: ws_a,
        };
        let md = UiModule {
            id: "md",
            register: dup,
        };
        let _ = UiRegistry::from_modules([&ma, &md]);
    }

    #[test]
    fn inventory_collects_framework_and_every_lane_stub() {
        let reg = UiRegistry::build();
        for id in [
            "design",
            "inspect",
            "mesh",
            "form",
            "sim",
            "generative",
            "am",
            "io",
            "materials",
        ] {
            assert!(reg.modules().contains(&id), "module `{id}` not collected");
        }
        assert_eq!(reg.default_workspace(), Some(WorkspaceId("design")));
        let design: Vec<_> = reg
            .visible_tabs(WorkspaceId("design"), &[])
            .iter()
            .map(|t| t.label)
            .collect();
        for t in ["Solid", "Surface", "Mesh", "Inspect", "Utilities"] {
            assert!(design.contains(&t), "Design lacks tab {t}: {design:?}");
        }
        assert!(!design.contains(&"Sketch"), "Sketch is contextual");
    }
}
