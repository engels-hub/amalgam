//! The registry of everything modules contribute (workspaces, tabs, groups, panels).

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

use crate::command::{CmdId, CommandDef, DYNAMIC_PREFIX, Placement, RegistryError};
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
    commands: BTreeMap<CmdId, (&'static str, CommandDef)>,
    /// Ids registered at runtime (`script.` namespace); only these can be unregistered.
    dynamic: BTreeSet<CmdId>,
    /// Toolbar index: group → (order, pinned, id), sorted.
    by_group: BTreeMap<GroupId, Vec<(i32, bool, CmdId)>>,
    /// Menu index: path → (order, id), sorted.
    by_menu: BTreeMap<&'static str, Vec<(i32, CmdId)>>,
    /// Bumped by every command change; search and shortcut caches compare it.
    generation: u64,
}

impl fmt::Debug for UiRegistry {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("UiRegistry")
            .field("modules", &self.modules)
            .field("workspaces", &self.workspaces.keys().collect::<Vec<_>>())
            .field("tabs", &self.tabs.keys().collect::<Vec<_>>())
            .field("groups", &self.groups.len())
            .field("panels", &self.panels.keys().collect::<Vec<_>>())
            .field("commands", &self.commands.len())
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

    pub(crate) fn add_command(&mut self, module: &'static str, def: CommandDef) {
        if let Some((first, _)) = self.commands.get(&def.id) {
            conflict("command", def.id.0, first, module);
            return;
        }
        self.insert_command(module, def);
    }

    fn insert_command(&mut self, module: &'static str, def: CommandDef) {
        let id = def.id;
        for p in &def.placements {
            match p {
                Placement::Toolbar {
                    group,
                    pinned,
                    order,
                } => {
                    let v = self.by_group.entry(*group).or_default();
                    v.push((*order, *pinned, id));
                    v.sort();
                }
                Placement::Menu { path, order } => {
                    let v = self.by_menu.entry(path).or_default();
                    v.push((*order, id));
                    v.sort();
                }
                Placement::SearchOnly => {}
            }
        }
        self.commands.insert(id, (module, def));
        self.generation += 1;
    }

    /// Registers a command at runtime (Python add-ins, SCR-04).
    ///
    /// # Errors
    /// [`RegistryError::BadPrefix`] unless the id starts with `script.`;
    /// [`RegistryError::Duplicate`] if the id is taken. Neither panics: add-ins are user code.
    pub fn register_dynamic(&mut self, def: CommandDef) -> Result<(), RegistryError> {
        if !def.id.0.starts_with(DYNAMIC_PREFIX) {
            return Err(RegistryError::BadPrefix(def.id.0));
        }
        if self.commands.contains_key(&def.id) {
            return Err(RegistryError::Duplicate(def.id.0));
        }
        self.dynamic.insert(def.id);
        self.insert_command("script", def);
        Ok(())
    }

    /// Removes a command registered with [`Self::register_dynamic`]. Returns false for an
    /// unknown id or a statically registered command (those cannot be removed).
    pub fn unregister_dynamic(&mut self, id: CmdId) -> bool {
        if !self.dynamic.remove(&id) {
            return false;
        }
        self.commands.remove(&id);
        for v in self.by_group.values_mut() {
            v.retain(|(_, _, c)| *c != id);
        }
        for v in self.by_menu.values_mut() {
            v.retain(|(_, c)| *c != id);
        }
        self.generation += 1;
        true
    }

    /// One command.
    pub fn command(&self, id: CmdId) -> Option<&CommandDef> {
        self.commands.get(&id).map(|(_, d)| d)
    }

    /// Every command, by id.
    pub fn commands(&self) -> impl Iterator<Item = &CommandDef> + '_ {
        self.commands.values().map(|(_, d)| d)
    }

    /// The module that registered a command (`"script"` for runtime registrations).
    pub fn command_owner(&self, id: CmdId) -> Option<&'static str> {
        self.commands.get(&id).map(|(m, _)| *m)
    }

    /// Changes whenever a command is added or removed.
    pub fn generation(&self) -> u64 {
        self.generation
    }

    /// The commands placed in a toolbar group, in display order: `(pinned, command)`.
    pub fn group_commands(&self, group: GroupId) -> impl Iterator<Item = (bool, &CommandDef)> + '_ {
        self.by_group
            .get(&group)
            .into_iter()
            .flatten()
            .filter_map(|(_, pinned, id)| self.command(*id).map(|d| (*pinned, d)))
    }

    /// The commands placed exactly at menu `path`, in display order.
    pub fn menu_commands(&self, path: &str) -> impl Iterator<Item = &CommandDef> + '_ {
        self.by_menu
            .get(path)
            .into_iter()
            .flatten()
            .filter_map(|(_, id)| self.command(*id))
    }

    /// The names of the menus directly below `prefix` (`""` = the top-level menus), sorted,
    /// that contain a command at or below them.
    pub fn menu_children(&self, prefix: &str) -> Vec<&'static str> {
        let mut out: Vec<&'static str> = Vec::new();
        for path in self.by_menu.keys() {
            let rest = if prefix.is_empty() {
                *path
            } else {
                match path.strip_prefix(prefix).and_then(|r| r.strip_prefix('/')) {
                    Some(r) => r,
                    None => continue,
                }
            };
            let first = rest.split('/').next().unwrap_or_default();
            if !first.is_empty() && !out.contains(&first) {
                out.push(first);
            }
        }
        out.sort_unstable();
        out
    }

    /// One toolbar group.
    pub fn group(&self, id: GroupId) -> Option<&GroupDef> {
        self.groups.get(&id).map(|(_, d)| d)
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

    fn cmd_a(r: &mut Registrar<'_>) {
        r.command(CommandDef::new(
            "dup.cmd",
            "First",
            crate::command::CmdAction::Run(|_| {}),
        ));
    }

    fn cmd_b(r: &mut Registrar<'_>) {
        r.command(CommandDef::new(
            "dup.cmd",
            "Second",
            crate::command::CmdAction::Run(|_| {}),
        ));
    }

    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "duplicate command id `dup.cmd`")]
    fn duplicate_cmd_id_panics_in_debug() {
        let a = UiModule {
            id: "ca",
            register: cmd_a,
        };
        let b = UiModule {
            id: "cb",
            register: cmd_b,
        };
        let _ = UiRegistry::from_modules([&a, &b]);
    }

    #[test]
    fn register_dynamic_rejects_ids_without_script_prefix() {
        use crate::command::{CmdAction, CmdId, CommandDef, RegistryError};
        let mut reg = UiRegistry::default();
        let mk = |id| CommandDef::new(id, "X", CmdAction::Run(|_| {}));
        assert_eq!(
            reg.register_dynamic(mk("mesh.remesh")),
            Err(RegistryError::BadPrefix("mesh.remesh"))
        );
        assert_eq!(reg.commands().count(), 0);
        assert_eq!(reg.register_dynamic(mk("script.a")), Ok(()));
        assert_eq!(
            reg.register_dynamic(mk("script.a")),
            Err(RegistryError::Duplicate("script.a"))
        );
        let g = reg.generation();
        assert!(reg.unregister_dynamic(CmdId("script.a")));
        assert!(reg.generation() > g);
        assert!(reg.command(CmdId("script.a")).is_none());
    }
}
