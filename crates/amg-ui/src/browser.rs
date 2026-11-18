//! The browser tree (spec §11, UI-07).
//!
//! - **Framework nodes** ([`framework`]): the root component with Document Settings, Named
//!   Views, Origin, then per component its Bodies / Sketches / Construction folders and child
//!   occurrences, and Selection Sets.
//! - **Other nodes** come from [`BrowserProvider`]s registered with
//!   [`Registrar::browser_provider`](crate::module::Registrar::browser_provider). Each is asked
//!   once per [`BrowserAnchor`] (the root, and every component) and its nodes are appended
//!   there; row actions on them go back to the provider that built them.
//! - **Rebuilds are event-driven.** The tree is rebuilt only when some provider's
//!   [`BrowserProvider::affected_by`] accepts one of the frame's `DocEvent`s (or the document
//!   was replaced / the workspace changed). Expanding and collapsing only re-flattens the
//!   visible rows. [`BrowserState::rebuilds`] counts rebuilds.
//! - **Rows are virtualised** with `ScrollArea::show_rows`: only the rows in view are laid out,
//!   so 10 000 bodies cost the same per frame as 30.
//! - **Row actions:** eye toggle → [`BrowserProvider::set_visible`]; rename (context menu or
//!   F2, inline text field, Enter commits, Esc cancels) → [`BrowserProvider::rename`]; context
//!   menu → [`BrowserProvider::context_menu`]; hover → hover highlight of the node's target;
//!   click → select it (Ctrl/Shift toggles); double-click → [`BrowserProvider::double_click`].
//!
//! Accessibility names (what kittest queries): the row label is the node label; the eye is a
//! button "Hide <label>" / "Show <label>"; the arrow is "Expand <label>" / "Collapse <label>".

pub mod framework;
pub mod visibility;

use std::borrow::Cow;
use std::collections::HashSet;

use amg_core::Severity;
use amg_doc::{ComponentId, DocEvent, EntityRef};

use crate::app::{AppCx, CmdEnv};
use crate::module::{IconId, WorkspaceId};
use crate::registry::UiRegistry;
use crate::select::SelItem;

pub use framework::FRAMEWORK_PROVIDER;

/// Key of a browser node, unique in the tree; provider-prefixed (`"body/12:0"`, `"study/3"`).
#[derive(Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct NodeKey(pub Cow<'static, str>);

impl NodeKey {
    /// A key from a static string.
    pub const fn new(s: &'static str) -> NodeKey {
        NodeKey(Cow::Borrowed(s))
    }

    /// A key from a formatted string.
    pub fn owned(s: String) -> NodeKey {
        NodeKey(Cow::Owned(s))
    }

    /// The key text.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl std::fmt::Display for NodeKey {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

/// One node a provider builds.
#[derive(Clone, Debug)]
pub struct BrowserNode {
    /// Unique key (provider-prefixed).
    pub key: NodeKey,
    /// Row text.
    pub label: String,
    /// Icon (drawn once the icon atlas exists, UI-21; unused until then).
    pub icon: IconId,
    /// `Some`: the eye toggle is shown with this state.
    pub visible: Option<bool>,
    /// Hover highlight and click selection.
    pub target: Option<SelItem>,
    /// Warning / error marker.
    pub badge: Option<Severity>,
    /// Offers "Rename" (context menu, F2).
    pub renamable: bool,
    /// Child nodes, in display order.
    pub children: Vec<BrowserNode>,
}

impl BrowserNode {
    /// A plain node: no eye, no target, no badge, not renamable, no children.
    pub fn new(key: NodeKey, label: impl Into<String>) -> BrowserNode {
        BrowserNode {
            key,
            label: label.into(),
            icon: IconId::NONE,
            visible: None,
            target: None,
            badge: None,
            renamable: false,
            children: Vec::new(),
        }
    }

    /// With an eye toggle in state `visible`.
    #[must_use]
    pub fn visible(mut self, visible: bool) -> BrowserNode {
        self.visible = Some(visible);
        self
    }

    /// With a hover / select target.
    #[must_use]
    pub fn target(mut self, target: Option<SelItem>) -> BrowserNode {
        self.target = target;
        self
    }

    /// With a badge.
    #[must_use]
    pub fn badge(mut self, badge: Option<Severity>) -> BrowserNode {
        self.badge = badge;
        self
    }

    /// Renamable.
    #[must_use]
    pub fn renamable(mut self) -> BrowserNode {
        self.renamable = true;
        self
    }

    /// With an icon.
    #[must_use]
    pub fn icon(mut self, icon: IconId) -> BrowserNode {
        self.icon = icon;
        self
    }

    /// With children.
    #[must_use]
    pub fn children(mut self, children: Vec<BrowserNode>) -> BrowserNode {
        self.children = children;
        self
    }
}

/// Where a provider's nodes go.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum BrowserAnchor {
    /// Top level, after the framework nodes of the root component.
    Root,
    /// Inside a component, after its Bodies / Sketches / Construction folders and before its
    /// child occurrences. Called for every component shown (the root too).
    Component(ComponentId),
}

/// Contributes browser nodes (lane F studies, lane G print setups, lane A analyses).
pub trait BrowserProvider: 'static {
    /// Unique id (the module id, or `<module>.<name>`).
    fn id(&self) -> &'static str;
    /// Rebuild only when this returns true for some event of the frame.
    fn affected_by(&self, ev: &DocEvent) -> bool;
    /// Appends this provider's nodes at `anchor`.
    fn build(&self, env: &CmdEnv<'_>, anchor: &BrowserAnchor, out: &mut Vec<BrowserNode>);
    /// The eye toggle of `node` was clicked.
    fn set_visible(&self, _node: &NodeKey, _visible: bool, _cx: &mut AppCx<'_>) {}
    /// `node` was renamed to `name` (trimmed, non-empty).
    fn rename(&self, _node: &NodeKey, _name: &str, _cx: &mut AppCx<'_>) {}
    /// `node` was double-clicked (edit).
    fn double_click(&self, _node: &NodeKey, _cx: &mut AppCx<'_>) {}
    /// Extra context-menu entries for `node` (after the framework's Rename / Show / Hide).
    fn context_menu(&self, _node: &NodeKey, _ui: &mut egui::Ui, _cx: &mut AppCx<'_>) {}
}

/// Row height, points (uniform: `show_rows` needs it).
pub const ROW_HEIGHT: f32 = 18.0;
/// Indent per level, points.
const INDENT: f32 = 12.0;
/// Width of the expand arrow / eye columns, points.
const ICON_W: f32 = 16.0;

/// One node of the flattened tree (pre-order).
#[derive(Debug)]
struct Entry {
    /// The node (its `children` moved into the following entries).
    node: BrowserNode,
    /// Index into the provider list (0: the framework).
    provider: usize,
    depth: u16,
    /// One past the last entry of this node's subtree.
    end: u32,
}

/// What the user did in the browser this frame.
#[derive(Debug)]
enum Action {
    Expand(usize, bool),
    Eye(usize),
    Select { entry: usize, toggle: bool },
    DoubleClick(usize),
    StartRename(usize),
    Rename(usize, String),
}

/// An inline rename in progress.
#[derive(Debug)]
struct Renaming {
    key: NodeKey,
    text: String,
    focus: bool,
}

/// The browser of one document: the cached tree, expansion, rename and hover state.
#[derive(Debug)]
pub struct BrowserState {
    entries: Vec<Entry>,
    /// Visible rows: indices into `entries`.
    rows: Vec<u32>,
    expanded: HashSet<NodeKey>,
    dirty: bool,
    rows_dirty: bool,
    /// What the tree was built for (workspace, document identity, units).
    built_for: Option<(WorkspaceId, amg_core::PersistentId, amg_doc::UnitPrefs)>,
    rebuilds: u64,
    renaming: Option<Renaming>,
    /// The hovered row's target (hover highlight, applied by the tool router).
    pub(crate) hover: Option<SelItem>,
    /// Rows to scroll to (after `reveal`).
    scroll_to: Option<usize>,
}

impl Default for BrowserState {
    fn default() -> BrowserState {
        let mut expanded = HashSet::new();
        expanded.insert(framework::ROOT_KEY);
        BrowserState {
            entries: Vec::new(),
            rows: Vec::new(),
            expanded,
            dirty: true,
            rows_dirty: true,
            built_for: None,
            rebuilds: 0,
            renaming: None,
            hover: None,
            scroll_to: None,
        }
    }
}

/// The framework provider followed by the registered ones.
fn providers(registry: &UiRegistry) -> Vec<&dyn BrowserProvider> {
    std::iter::once(&FRAMEWORK_PROVIDER as &dyn BrowserProvider)
        .chain(registry.browser_providers())
        .collect()
}

impl BrowserState {
    /// How many times the tree was rebuilt.
    pub fn rebuilds(&self) -> u64 {
        self.rebuilds
    }

    /// Number of nodes in the tree (after the last rebuild).
    pub fn node_count(&self) -> usize {
        self.entries.len()
    }

    /// Number of visible rows (expanded subtrees only).
    pub fn row_count(&self) -> usize {
        self.rows.len()
    }

    /// The labels of the visible rows, in order (tests).
    pub fn row_labels(&self) -> Vec<&str> {
        self.rows
            .iter()
            .map(|&i| self.entries[i as usize].node.label.as_str())
            .collect()
    }

    /// The node with `key`, if it is in the tree.
    pub fn node(&self, key: &NodeKey) -> Option<&BrowserNode> {
        self.entries
            .iter()
            .find(|e| &e.node.key == key)
            .map(|e| &e.node)
    }

    /// Whether `key` is expanded.
    pub fn is_expanded(&self, key: &NodeKey) -> bool {
        self.expanded.contains(key)
    }

    /// Expands or collapses `key` (a cheap re-flatten, never a rebuild).
    pub fn set_expanded(&mut self, key: &NodeKey, open: bool) {
        let changed = if open {
            self.expanded.insert(key.clone())
        } else {
            self.expanded.remove(key)
        };
        self.rows_dirty |= changed;
    }

    /// Expands `key` and its ancestors and scrolls it into view on the next draw.
    pub fn reveal(&mut self, key: &NodeKey) {
        let Some(i) = self.entries.iter().position(|e| &e.node.key == key) else {
            return;
        };
        for (j, e) in self.entries.iter().enumerate().take(i) {
            if (e.end as usize) > i {
                self.rows_dirty |= self.expanded.insert(self.entries[j].node.key.clone());
            }
        }
        self.scroll_to = Some(i);
    }

    /// Marks the tree stale (it is rebuilt on the next draw).
    pub fn invalidate(&mut self) {
        self.dirty = true;
    }

    /// Marks the tree stale if a provider is affected by one of `events`.
    pub(crate) fn on_events(&mut self, events: &[DocEvent], registry: &UiRegistry) {
        if self.dirty {
            return;
        }
        let ps = providers(registry);
        self.dirty = events.iter().any(|ev| {
            matches!(ev, DocEvent::DocumentReplaced) || ps.iter().any(|p| p.affected_by(ev))
        });
    }

    /// Rebuilds the tree if it is stale.
    fn ensure_built(&mut self, env: &CmdEnv<'_>, registry: &UiRegistry) {
        // Not covered by events: the workspace, another document put in place, the units
        // shown under Document Settings.
        let meta = env.design.meta();
        let stamp = (env.workspace, meta.uuid, meta.default_units.clone());
        if self.built_for.as_ref() != Some(&stamp) {
            self.dirty = true;
        }
        if !self.dirty {
            return;
        }
        let _span = tracing::debug_span!("amg_ui.browser.rebuild").entered();
        let ps = providers(registry);
        self.entries.clear();
        framework::build_tree(
            env,
            &ps,
            &mut Builder {
                out: &mut self.entries,
            },
        );
        self.dirty = false;
        self.rows_dirty = true;
        self.built_for = Some(stamp);
        self.rebuilds += 1;
        if let Some(r) = &self.renaming
            && !self.entries.iter().any(|e| e.node.key == r.key)
        {
            self.renaming = None;
        }
        if let Some(h) = &self.hover
            && !self
                .entries
                .iter()
                .any(|e| e.node.target.as_ref().is_some_and(|t| t.entity == h.entity))
        {
            self.hover = None;
        }
    }

    /// Recomputes the visible rows from the tree and the expansion set.
    fn ensure_rows(&mut self) {
        if !self.rows_dirty {
            return;
        }
        self.rows.clear();
        let mut i = 0;
        while i < self.entries.len() {
            self.rows.push(i as u32);
            let e = &self.entries[i];
            let has_children = (e.end as usize) > i + 1;
            i = if has_children && self.expanded.contains(&e.node.key) {
                i + 1
            } else {
                e.end as usize
            };
        }
        self.rows_dirty = false;
    }

    /// Draws the tree; returns the actions to apply after the frame's layout. Provider context
    /// menus run here (they need `cx`).
    fn show(&mut self, ui: &mut egui::Ui, cx: &mut AppCx<'_>) -> Vec<Action> {
        let registry = cx.registry_ref();
        {
            let env = cx.env();
            self.ensure_built(&env, registry);
        }
        self.ensure_rows();
        let ps = providers(registry);
        let selected: Vec<EntityRef> = cx
            .selection()
            .items()
            .iter()
            .map(|i| i.entity.clone())
            .collect();
        let mut actions = Vec::new();
        let mut hover = None;
        let mut area = egui::ScrollArea::vertical()
            .auto_shrink([false, false])
            .id_salt("amg_browser_rows");
        if let Some(i) = self.scroll_to.take()
            && let Some(row) = self.rows.iter().position(|&r| r as usize == i)
        {
            let spacing = ui.spacing().item_spacing.y;
            area = area.vertical_scroll_offset(row as f32 * (ROW_HEIGHT + spacing));
        }
        area.show_rows(ui, ROW_HEIGHT, self.rows.len(), |ui, range| {
            for row in range {
                let i = self.rows[row] as usize;
                self.row(ui, i, &selected, &ps, cx, &mut actions, &mut hover);
            }
        });
        if hover.as_ref().map(|h: &SelItem| &h.entity) != self.hover.as_ref().map(|h| &h.entity) {
            self.hover = hover;
            cx.request_repaint();
        }
        actions
    }

    #[allow(
        clippy::too_many_arguments,
        reason = "one row of the draw loop over disjoint borrows"
    )]
    fn row(
        &mut self,
        ui: &mut egui::Ui,
        i: usize,
        selected: &[EntityRef],
        ps: &[&dyn BrowserProvider],
        cx: &mut AppCx<'_>,
        actions: &mut Vec<Action>,
        hover: &mut Option<SelItem>,
    ) {
        let e = &self.entries[i];
        let node = &e.node;
        let has_children = (e.end as usize) > i + 1;
        let is_selected = node
            .target
            .as_ref()
            .is_some_and(|t| selected.contains(&t.entity));
        let (rect, _) = ui.allocate_exact_size(
            egui::vec2(ui.available_width(), ROW_HEIGHT),
            egui::Sense::hover(),
        );
        if is_selected {
            ui.painter()
                .rect_filled(rect, 2.0, ui.visuals().selection.bg_fill);
        }
        let mut row = ui.new_child(
            egui::UiBuilder::new()
                .max_rect(rect)
                .layout(egui::Layout::left_to_right(egui::Align::Center)),
        );
        row.spacing_mut().item_spacing.x = 2.0;
        row.add_space(f32::from(e.depth) * INDENT);
        if has_children {
            let open = self.expanded.contains(&node.key);
            let (text, name) = if open {
                ("⏷", "Collapse")
            } else {
                ("⏵", "Expand")
            };
            let r = row.add_sized(
                [ICON_W, ROW_HEIGHT],
                egui::Button::new(egui::RichText::new(text).small()).frame(false),
            );
            let label = format!("{name} {}", node.label);
            r.widget_info(|| {
                egui::WidgetInfo::labeled(egui::WidgetType::Button, true, label.clone())
            });
            if r.clicked() {
                actions.push(Action::Expand(i, !open));
            }
        } else {
            row.add_space(ICON_W);
        }
        if let Some(visible) = node.visible {
            let text = egui::RichText::new("👁").small();
            let text = if visible { text } else { text.weak() };
            let r = row.add_sized([ICON_W, ROW_HEIGHT], egui::Button::new(text).frame(false));
            let label = format!("{} {}", if visible { "Hide" } else { "Show" }, node.label);
            r.widget_info(|| {
                egui::WidgetInfo::labeled(egui::WidgetType::Button, true, label.clone())
            });
            if r.clicked() {
                actions.push(Action::Eye(i));
            }
        }
        let renaming = self.renaming.as_mut().filter(|r| r.key == node.key);
        if let Some(r) = renaming {
            let out = egui::TextEdit::singleline(&mut r.text)
                .desired_width(row.available_width() - 4.0)
                .id_salt(("amg_browser_rename", node.key.as_str()))
                .show(&mut row);
            let edit = out.response;
            if r.focus {
                // Focus with the whole name selected, so typing replaces it.
                edit.request_focus();
                let mut state = out.state;
                let n = r.text.chars().count();
                state
                    .cursor
                    .set_char_range(Some(egui::text::CCursorRange::two(
                        egui::text::CCursor::new(0),
                        egui::text::CCursor::new(n),
                    )));
                state.store(row.ctx(), edit.id);
                r.focus = false;
            }
            let (enter, esc) = row.input(|inp| {
                (
                    inp.key_pressed(egui::Key::Enter),
                    inp.key_pressed(egui::Key::Escape),
                )
            });
            if esc {
                self.renaming = None;
            } else if edit.lost_focus() || enter {
                let text = r.text.trim().to_owned();
                self.renaming = None;
                actions.push(Action::Rename(i, text));
            }
            return;
        }
        let mut text = egui::RichText::new(&node.label);
        if node.visible == Some(false) {
            text = text.weak();
        }
        if is_selected {
            text = text.color(ui.visuals().selection.stroke.color);
        }
        let label = row.add(
            egui::Label::new(text)
                .sense(egui::Sense::click())
                .selectable(false)
                .truncate(),
        );
        match node.badge {
            Some(Severity::Error) => {
                row.colored_label(ui.visuals().error_fg_color, "⚠")
                    .on_hover_text("Error");
            }
            Some(Severity::Warning) => {
                row.colored_label(ui.visuals().warn_fg_color, "⚠")
                    .on_hover_text("Warning");
            }
            Some(Severity::Info) | None => {}
        }
        if label.hovered() {
            hover.clone_from(&node.target);
        }
        if label.double_clicked() {
            actions.push(Action::DoubleClick(i));
        } else if label.clicked() {
            let toggle = row.input(|inp| inp.modifiers.command || inp.modifiers.shift);
            actions.push(Action::Select { entry: i, toggle });
        }
        if label.hovered() && node.renamable && row.input(|inp| inp.key_pressed(egui::Key::F2)) {
            actions.push(Action::StartRename(i));
        }
        let provider = ps[e.provider];
        let key = node.key.clone();
        let renamable = node.renamable;
        let visible = node.visible;
        label.context_menu(|ui| {
            if renamable && ui.button("Rename").clicked() {
                actions.push(Action::StartRename(i));
                ui.close();
            }
            if let Some(v) = visible
                && ui.button(if v { "Hide" } else { "Show" }).clicked()
            {
                actions.push(Action::Eye(i));
                ui.close();
            }
            provider.context_menu(&key, ui, cx);
        });
    }
}

/// Appends entries in pre-order.
pub(crate) struct Builder<'a> {
    out: &'a mut Vec<Entry>,
}

impl Builder<'_> {
    /// Opens a node whose children follow until [`Builder::close`]. Returns its index.
    pub(crate) fn open(&mut self, node: BrowserNode, provider: usize, depth: u16) -> usize {
        debug_assert!(node.children.is_empty());
        self.out.push(Entry {
            node,
            provider,
            depth,
            end: 0,
        });
        self.out.len() - 1
    }

    /// Closes the node opened at `index`.
    pub(crate) fn close(&mut self, index: usize) {
        self.out[index].end = self.out.len() as u32;
    }

    /// Appends a node and its children (recursively), tagged with `provider`.
    pub(crate) fn push_tree(&mut self, mut node: BrowserNode, provider: usize, depth: u16) {
        let children = std::mem::take(&mut node.children);
        let i = self.open(node, provider, depth);
        for c in children {
            self.push_tree(c, provider, depth + 1);
        }
        self.close(i);
    }

    /// Number of entries so far.
    pub(crate) fn len(&self) -> usize {
        self.out.len()
    }
}

// ---- app integration ---------------------------------------------------------------------

impl crate::app::AmalgamApp {
    /// Draws the browser of the active document into `ui` (the shell's left panel calls it;
    /// benches call it alone) and applies what the user did.
    pub fn browser_ui(&mut self, ui: &mut egui::Ui) {
        let _span = tracing::debug_span!("amg_ui.browser").entered();
        let active = self.active;
        let mut state = std::mem::take(&mut self.docs[active].browser);
        let actions = self.with_cx(|cx| state.show(ui, cx));
        for a in actions {
            self.apply_browser_action(&mut state, a);
        }
        self.docs[active].browser = state;
    }

    /// The browser of the active document.
    pub fn browser(&self) -> &BrowserState {
        &self.docs[self.active].browser
    }

    /// The browser of the active document, mutable (expand, reveal).
    pub fn browser_mut(&mut self) -> &mut BrowserState {
        self.repaint_soon();
        &mut self.docs[self.active].browser
    }

    fn apply_browser_action(&mut self, state: &mut BrowserState, a: Action) {
        let provider_of = |state: &BrowserState, i: usize| state.entries[i].provider;
        match a {
            Action::Expand(i, open) => {
                let key = state.entries[i].node.key.clone();
                state.set_expanded(&key, open);
                self.repaint_soon();
            }
            Action::Eye(i) => {
                let key = state.entries[i].node.key.clone();
                let visible = state.entries[i].node.visible.unwrap_or(true);
                let p = provider_of(state, i);
                self.with_provider(p, |p, cx| p.set_visible(&key, !visible, cx));
            }
            Action::Select { entry, toggle } => {
                let Some(item) = state.entries[entry].node.target.clone() else {
                    return;
                };
                let item = self.resolve_item(item);
                let sel = &mut self.docs[self.active].selection;
                if toggle {
                    sel.toggle(item);
                } else {
                    sel.set(vec![item]);
                }
                self.repaint_soon();
            }
            Action::DoubleClick(i) => {
                let key = state.entries[i].node.key.clone();
                let p = provider_of(state, i);
                self.with_provider(p, |p, cx| p.double_click(&key, cx));
            }
            Action::StartRename(i) => {
                state.renaming = Some(Renaming {
                    key: state.entries[i].node.key.clone(),
                    text: state.entries[i].node.label.clone(),
                    focus: true,
                });
                self.repaint_soon();
            }
            Action::Rename(i, name) => {
                if name.is_empty() || name == state.entries[i].node.label {
                    return;
                }
                let key = state.entries[i].node.key.clone();
                let p = provider_of(state, i);
                self.with_provider(p, |p, cx| p.rename(&key, &name, cx));
            }
        }
    }

    /// Runs `f` with provider `index` (0: the framework) and an [`AppCx`].
    fn with_provider(
        &mut self,
        index: usize,
        f: impl FnOnce(&dyn BrowserProvider, &mut AppCx<'_>),
    ) {
        self.with_cx(|cx| {
            let registry = cx.registry_ref();
            let ps = providers(registry);
            if let Some(p) = ps.get(index) {
                f(*p, cx);
            }
        });
    }

    /// `item` with its target mapped to what the scene shows now (bodies inside components
    /// have instance indices only the scene knows).
    pub(crate) fn resolve_item(&self, item: SelItem) -> SelItem {
        let doc = &self.docs[self.active];
        match crate::select::target_for_entity(
            doc.document.design(),
            doc.document.evaluated(),
            &self.scene,
            &item.entity,
        ) {
            Some(target) => SelItem {
                target,
                entity: item.entity,
            },
            None => item,
        }
    }

    /// The browser's hover target, resolved against the scene (the tool router highlights it
    /// when nothing in the viewport is hovered).
    pub(crate) fn browser_hover_target(&self) -> Option<crate::pick::PickTarget> {
        let item = self.docs[self.active].browser.hover.clone()?;
        Some(self.resolve_item(item).target)
    }

    fn repaint_soon(&self) {
        if let Some(ctx) = &self.ctx {
            ctx.request_repaint();
        }
    }
}
