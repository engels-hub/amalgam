//! The timeline strip (spec §12, UI-08).
//!
//! - **Nodes** are drawn left to right from the document's `Timeline`, each a fixed-width
//!   slot with an icon from its kind's [`FeatureUiInfo`] (registered with
//!   [`Registrar::feature_ui`](crate::module::Registrar::feature_ui)). Unknown kinds get a
//!   generic icon. Labels are the node name, else `<display name><n>` (`Extrude2`).
//! - **Groups** show as a bracket under their nodes; clicking the bracket collapses the group
//!   into one slot (`SetGroup { collapsed }`), clicking that slot expands it again.
//! - **Virtualised:** the strip is a horizontal `ScrollArea::show_viewport`; only the slots in
//!   view are laid out (the node being dragged is always interacted with).
//! - **Rollback marker:** a handle between two slots; dragging it previews the rolled-back
//!   nodes and sends one `SetMarker` on release. The ⏮ ⏴ ⏵ ⏭ buttons step it.
//! - **Position guard** (ASM-02): while occurrence moves are pending, marker moves, reorders
//!   and edits are handed to the registered [`crate::guard`] first (Capture / Revert / Cancel).
//! - **Reordering:** dragging a node shades the positions it can't take (outside
//!   `Timeline::valid_move_range`) red and shows the drop line; a release on a valid position
//!   sends one `MoveFeature`, a release on an invalid one sends nothing.
//! - **Context menu:** Edit Feature, Edit Sketch, Roll to Here, Suppress / Unsuppress,
//!   Rename, Delete, Group / Ungroup, Find in Browser. Each edit is one undo entry.
//! - **Health:** warning / error badges with the diagnostic messages as tooltip; error nodes
//!   are tinted red. Hovering a node highlights its output bodies (Hover layer); hovering an
//!   error node highlights its `Diagnostic.entities` (Error layer). The node being recomputed
//!   shows a progress ring.
//! - The shell hides the timeline in Direct mode.
//!
//! Accessibility names (what kittest queries): a node is a button named by its label; a
//! collapsed group is a button "Expand <group>"; an expanded group's bracket is a button
//! "Collapse <group>"; the marker is a slider "Rollback marker"; badges are labels "Error" /
//! "Warning" with the messages as hover text.

use std::ops::RangeInclusive;
use std::sync::Arc;

use amg_core::Severity;
use amg_doc::{
    Command as DocCommand, Dependency, Design, EntityRef, Evaluated, FeatureId, GroupId, Health,
    RecomputeStatus, Revision,
};

use crate::app::{AmalgamApp, AppCx, CommitBatch};
use crate::guard::GuardedAction;
use crate::module::IconId;
use crate::pick::PickTarget;
use crate::tool::{Tool, ToolInit};

/// How a timeline node's kind is shown and edited (registered per S01 `Feature::kind`).
#[derive(Clone, Copy, Debug)]
pub struct FeatureUiInfo {
    /// S01 `Feature::kind`.
    pub kind: &'static str,
    /// Display name ("Extrude"); default node labels are `<display_name><n>`.
    pub display_name: &'static str,
    /// Icon (UI-23; a node without a drawable icon shows the display name's initials).
    pub icon: IconId,
    /// What "Edit Feature" / double-click does.
    pub edit: EditAction,
}

/// What editing a feature does.
#[derive(Clone, Copy, Debug)]
pub enum EditAction {
    /// The generic dialog of its `feature.<kind>` command (UI-05).
    GenericDialog,
    /// A hand-built tool (MESH-23 etc.), started with `ToolInit::edit = Some(id)`.
    StartTool(fn(&ToolInit<'_>) -> Box<dyn Tool>),
    /// Sketch mode (`sketch.sketch`).
    SketchMode,
    /// Not editable ("Edit Feature" is disabled).
    NotEditable,
}

/// The command id an [`EditAction::StartTool`] tool runs under when its kind has no
/// `feature.<kind>` command.
pub const EDIT_TOOL_CMD: crate::command::CmdId = crate::command::CmdId("timeline.edit_feature");

/// Width of one node slot, points.
pub const SLOT_W: f32 = 28.0;
/// Height of a node box, points.
const NODE_H: f32 = 20.0;
/// Top of the node boxes inside the strip, points.
const NODE_TOP: f32 = 3.0;
/// Gap between slots (each side), points.
const PAD: f32 = 2.0;
/// Width of the marker handle, points.
const MARKER_W: f32 = 8.0;
/// Generic icon text for kinds without a [`FeatureUiInfo`].
const GENERIC_ICON: &str = "◇";

/// A node's health as the timeline shows it.
#[derive(Clone, Debug, PartialEq)]
pub enum NodeHealth {
    /// Computed fine (or not evaluated yet).
    Ok,
    /// Warnings, with their messages.
    Warning(Vec<String>),
    /// Failed, with the message and the entities to highlight.
    Error(String, Vec<EntityRef>),
}

/// One cached timeline node.
#[derive(Clone, Debug)]
struct Item {
    id: FeatureId,
    label: String,
    /// Icon text (initials or the generic icon).
    icon: String,
    known: bool,
    edit: Option<EditAction>,
    group: Option<GroupId>,
    suppressed: bool,
    health: NodeHealth,
    /// The sketch this feature reads (Edit Sketch).
    sketch: Option<FeatureId>,
}

/// One horizontal slot: a node, or a collapsed group of nodes.
#[derive(Clone, Debug, PartialEq, Eq)]
enum Slot {
    Node(usize),
    Collapsed(GroupId, RangeInclusive<usize>),
}

impl Slot {
    fn first(&self) -> usize {
        match self {
            Slot::Node(i) => *i,
            Slot::Collapsed(_, r) => *r.start(),
        }
    }
}

/// An expanded group's bracket.
#[derive(Clone, Debug)]
struct Bracket {
    group: GroupId,
    name: String,
    /// First and last slot.
    slots: RangeInclusive<usize>,
}

/// A drag in progress.
#[derive(Clone, Debug, PartialEq, Eq)]
enum Drag {
    Marker {
        to: usize,
    },
    Node {
        id: FeatureId,
        from: usize,
        valid: Option<RangeInclusive<usize>>,
        /// Final index under the pointer (`None` before the pointer left the start slot).
        to: Option<usize>,
    },
}

/// What the user did in the timeline this frame.
#[derive(Debug)]
enum Action {
    Marker(usize),
    Move(FeatureId, usize),
    Select {
        id: FeatureId,
        toggle: bool,
    },
    Edit(FeatureId),
    Suppress(FeatureId, bool),
    StartRename(FeatureId),
    Rename(FeatureId, String),
    Delete(FeatureId),
    Group(FeatureId),
    Ungroup(GroupId),
    Collapse(GroupId, bool),
    FindInBrowser(FeatureId),
    /// A context provider's command (UI-21).
    Run(crate::command::CmdId),
}

/// A node as drawn in the last frame (tests, benches and the hover highlight read this).
#[derive(Clone, Debug, PartialEq)]
pub struct DrawnNode {
    /// The feature.
    pub id: FeatureId,
    /// Timeline index.
    pub index: usize,
    /// The node's rectangle, screen points.
    pub rect: egui::Rect,
    /// Drawn as rolled back (after the marker, or after the marker being dragged).
    pub rolled_back: bool,
    /// Drawn as suppressed.
    pub suppressed: bool,
    /// Badge shown.
    pub badge: Option<Severity>,
    /// Tinted red (error).
    pub error_tint: bool,
    /// Shaded red as an invalid drop position of the node being dragged.
    pub invalid_drop: bool,
    /// The recompute progress ring is drawn on it.
    pub computing: bool,
}

/// The drag of a node as drawn in the last frame.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DragView {
    /// The node being dragged.
    pub id: FeatureId,
    /// Final index under the pointer.
    pub to: Option<usize>,
    /// Whether dropping there is allowed.
    pub valid: bool,
}

/// An inline rename in progress.
#[derive(Debug)]
struct Renaming {
    id: FeatureId,
    text: String,
    focus: bool,
}

/// The timeline of one document: the cached nodes, timeline selection, drag, rename, hover.
#[derive(Debug, Default)]
pub struct TimelineState {
    items: Vec<Item>,
    /// Icon of each item (parallel to `items`; missing entries draw the text icon), UI-23.
    icon_ids: Vec<IconId>,
    slots: Vec<Slot>,
    /// Node index → slot index.
    slot_of: Vec<usize>,
    brackets: Vec<Bracket>,
    /// Names of the groups shown.
    group_names: Vec<(GroupId, String)>,
    marker: usize,
    /// What the cache was built for: design revision, evaluated pointer.
    built_for: Option<(Revision, usize)>,
    rebuilds: u64,
    selected: Vec<FeatureId>,
    drag: Option<Drag>,
    renaming: Option<Renaming>,
    /// The hovered node (hover highlight, applied by the tool router).
    pub(crate) hover: Option<FeatureId>,
    drawn: Vec<DrawnNode>,
    drag_view: Option<DragView>,
    marker_rect: Option<egui::Rect>,
    bracket_rects: Vec<(GroupId, egui::Rect)>,
}

impl TimelineState {
    /// How many times the node cache was rebuilt.
    pub fn rebuilds(&self) -> u64 {
        self.rebuilds
    }

    /// Number of nodes.
    pub fn len(&self) -> usize {
        self.items.len()
    }

    /// True if the timeline has no nodes.
    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }

    /// Node labels in timeline order.
    pub fn labels(&self) -> Vec<&str> {
        self.items.iter().map(|i| i.label.as_str()).collect()
    }

    /// The label of `id`.
    pub fn label(&self, id: FeatureId) -> Option<&str> {
        self.items
            .iter()
            .find(|i| i.id == id)
            .map(|i| i.label.as_str())
    }

    /// The health shown for `id`.
    pub fn health(&self, id: FeatureId) -> Option<&NodeHealth> {
        self.items.iter().find(|i| i.id == id).map(|i| &i.health)
    }

    /// Nodes drawn in the last frame (only the ones in view).
    pub fn drawn(&self) -> &[DrawnNode] {
        &self.drawn
    }

    /// The node `id` as drawn in the last frame.
    pub fn drawn_node(&self, id: FeatureId) -> Option<&DrawnNode> {
        self.drawn.iter().find(|d| d.id == id)
    }

    /// The marker handle's rectangle in the last frame.
    pub fn marker_rect(&self) -> Option<egui::Rect> {
        self.marker_rect
    }

    /// An expanded group's bracket rectangle in the last frame.
    pub fn bracket_rect(&self, g: GroupId) -> Option<egui::Rect> {
        self.bracket_rects
            .iter()
            .find(|(id, _)| *id == g)
            .map(|(_, r)| *r)
    }

    /// The node drag as drawn in the last frame.
    pub fn drag_view(&self) -> Option<&DragView> {
        self.drag_view.as_ref()
    }

    /// The marker position being dragged to, if the marker is being dragged.
    pub fn marker_drag(&self) -> Option<usize> {
        match self.drag {
            Some(Drag::Marker { to }) => Some(to),
            _ => None,
        }
    }

    /// The timeline selection (for Group, Delete).
    pub fn selected(&self) -> &[FeatureId] {
        &self.selected
    }

    /// The hovered node.
    pub fn hovered(&self) -> Option<FeatureId> {
        self.hover
    }

    /// Marks the cache stale (rebuilt on the next draw).
    pub fn invalidate(&mut self) {
        self.built_for = None;
    }

    /// Rebuilds the node cache if the document or its evaluation changed.
    fn ensure_built(
        &mut self,
        design: &Design,
        eval: &Arc<Evaluated>,
        registry: &crate::UiRegistry,
    ) {
        let stamp = (design.revision(), Arc::as_ptr(eval) as usize);
        if self.built_for == Some(stamp) {
            return;
        }
        let _span = tracing::debug_span!("amg_ui.timeline.rebuild").entered();
        let tl = design.timeline();
        self.items.clear();
        self.icon_ids.clear();
        let mut ordinals: std::collections::HashMap<String, u32> = Default::default();
        for node in tl.nodes() {
            let info = registry.feature_ui(node.feature.kind());
            let known = info.is_some() && !node.feature.is_unknown();
            let (base, label) = node_label(node, registry, &mut ordinals);
            self.icon_ids.push(match info {
                Some(i) if known => i.icon,
                _ => crate::icons::ids::GENERIC,
            });
            let icon = if known {
                initials(&base)
            } else {
                GENERIC_ICON.to_owned()
            };
            let health = match eval.node(node.id).map(|e| &e.health) {
                Some(Health::Warning(ds)) => {
                    NodeHealth::Warning(ds.iter().map(|d| d.message.clone()).collect())
                }
                Some(Health::Error(d)) => NodeHealth::Error(d.message.clone(), d.entities.clone()),
                _ => NodeHealth::Ok,
            };
            let sketch = node.feature.inputs().iter().find_map(|d| match d {
                Dependency::Sketch(s) => Some(s.0),
                _ => None,
            });
            self.items.push(Item {
                id: node.id,
                label,
                icon,
                known,
                edit: info.map(|i| i.edit),
                group: node.group,
                suppressed: node.suppressed,
                health,
                sketch,
            });
        }
        self.marker = tl.marker();
        // Slots and brackets.
        self.slots.clear();
        self.slot_of.clear();
        self.brackets.clear();
        self.group_names.clear();
        let mut i = 0;
        while i < self.items.len() {
            let Some(g) = self.items[i].group else {
                self.slot_of.push(self.slots.len());
                self.slots.push(Slot::Node(i));
                i += 1;
                continue;
            };
            let mut end = i;
            while end + 1 < self.items.len() && self.items[end + 1].group == Some(g) {
                end += 1;
            }
            let group = tl.group(g);
            let name = group.map_or_else(|| format!("{g}"), |g| g.name.clone());
            self.group_names.push((g, name.clone()));
            if group.is_some_and(|g| g.collapsed) {
                for _ in i..=end {
                    self.slot_of.push(self.slots.len());
                }
                self.slots.push(Slot::Collapsed(g, i..=end));
            } else {
                let first = self.slots.len();
                for j in i..=end {
                    self.slot_of.push(self.slots.len());
                    self.slots.push(Slot::Node(j));
                }
                self.brackets.push(Bracket {
                    group: g,
                    name,
                    slots: first..=self.slots.len() - 1,
                });
            }
            i = end + 1;
        }
        let ids: Vec<FeatureId> = self.items.iter().map(|i| i.id).collect();
        self.selected.retain(|id| ids.contains(id));
        if self.hover.is_some_and(|h| !ids.contains(&h)) {
            self.hover = None;
        }
        if self.renaming.as_ref().is_some_and(|r| !ids.contains(&r.id)) {
            self.renaming = None;
        }
        if let Some(Drag::Node { id, .. }) = &self.drag
            && !ids.contains(id)
        {
            self.drag = None;
        }
        self.built_for = Some(stamp);
        self.rebuilds += 1;
    }

    /// Slot boundary `b` (0..=slots) → node index (0..=len).
    fn boundary_node(&self, b: usize) -> usize {
        self.slots.get(b).map_or(self.items.len(), Slot::first)
    }

    /// Node index (0..=len) → slot boundary.
    fn node_boundary(&self, i: usize) -> usize {
        match self.slot_of.get(i) {
            None => self.slots.len(),
            Some(&s) if self.slots[s].first() == i => s,
            // Inside a collapsed group: after it.
            Some(&s) => s + 1,
        }
    }

    /// Nearest slot boundary to content x.
    fn boundary_at(&self, x: f32) -> usize {
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            reason = "clamped, small"
        )]
        let b = (x / SLOT_W).round().max(0.0) as usize;
        b.min(self.slots.len())
    }

    /// Slot under content x.
    fn slot_at(&self, x: f32) -> usize {
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            reason = "clamped, small"
        )]
        let s = (x / SLOT_W).floor().max(0.0) as usize;
        s.min(self.slots.len().saturating_sub(1))
    }

    /// Final index of node `from` dropped at slot boundary `b`.
    fn drop_index(&self, from: usize, b: usize) -> usize {
        let at = self.boundary_node(b);
        if at > from { at - 1 } else { at }
    }

    /// Draws the strip; returns the actions to apply after the frame's layout.
    fn show(
        &mut self,
        ui: &mut egui::Ui,
        cx: &mut AppCx<'_>,
        tokens: &crate::theme::ThemeTokens,
    ) -> Vec<Action> {
        let registry = cx.registry_ref();
        {
            let env = cx.env();
            self.ensure_built(env.design, env.evaluated, registry);
        }
        let status = cx.doc_tab().document.recompute_status();
        let valid_range = |id| cx.env().design.timeline().valid_move_range(id);
        // Registry context providers below the node menu's own entries (UI-21).
        let provider_menu = |ui: &mut egui::Ui, id: FeatureId| {
            crate::marking::context_menu_ui(
                ui,
                registry,
                &cx.env(),
                &crate::marking::ContextTarget::Timeline(id),
            )
        };
        let mut actions = Vec::new();
        self.drawn.clear();
        self.drag_view = None;
        self.marker_rect = None;
        self.bracket_rects.clear();
        let len = self.items.len();
        ui.horizontal_centered(|ui| {
            let m = self.marker;
            for (label, tip, to) in [
                ("⏮", "Go to start", 0),
                ("⏴", "Step back", m.saturating_sub(1)),
                ("⏵", "Step forward", (m + 1).min(len)),
                ("⏭", "Go to end", len),
            ] {
                let r = ui
                    .add_enabled(to != m, egui::Button::new(label))
                    .on_disabled_hover_text(tip)
                    .on_hover_text(tip);
                if r.clicked() {
                    actions.push(Action::Marker(to));
                }
            }
            ui.separator();
            if self.items.is_empty() {
                ui.weak("Timeline is empty");
                return;
            }
            egui::ScrollArea::horizontal()
                .auto_shrink([false, false])
                .id_salt("amg_timeline_strip")
                .show_viewport(ui, |ui, viewport| {
                    self.strip(
                        ui,
                        viewport,
                        &status,
                        tokens,
                        &valid_range,
                        &provider_menu,
                        &mut actions,
                    );
                });
        });
        if let Some(r) = self.renaming.as_mut() {
            let anchor = self
                .drawn
                .iter()
                .find(|d| d.id == r.id)
                .map_or_else(|| ui.max_rect().left_top(), |d| d.rect.left_top());
            let mut done = None;
            egui::Area::new(egui::Id::new("amg_timeline_rename"))
                .order(egui::Order::Foreground)
                .pivot(egui::Align2::LEFT_BOTTOM)
                .fixed_pos(anchor)
                .show(ui.ctx(), |ui| {
                    egui::Frame::popup(ui.style()).show(ui, |ui| {
                        let out = egui::TextEdit::singleline(&mut r.text)
                            .desired_width(160.0)
                            .id_salt("amg_timeline_rename_field")
                            .show(ui);
                        let edit = out.response;
                        if r.focus {
                            edit.request_focus();
                            let mut state = out.state;
                            let n = r.text.chars().count();
                            state
                                .cursor
                                .set_char_range(Some(egui::text::CCursorRange::two(
                                    egui::text::CCursor::new(0),
                                    egui::text::CCursor::new(n),
                                )));
                            state.store(ui.ctx(), edit.id);
                            r.focus = false;
                        }
                        let (enter, esc) = ui.input(|i| {
                            (
                                i.key_pressed(egui::Key::Enter),
                                i.key_pressed(egui::Key::Escape),
                            )
                        });
                        if esc {
                            done = Some(None);
                        } else if enter || edit.lost_focus() {
                            done = Some(Some(r.text.trim().to_owned()));
                        }
                    });
                });
            match done {
                Some(Some(name)) => {
                    let id = r.id;
                    self.renaming = None;
                    actions.push(Action::Rename(id, name));
                }
                Some(None) => self.renaming = None,
                None => {}
            }
        }
        if matches!(status, RecomputeStatus::Running { .. }) {
            cx.request_repaint_after(std::time::Duration::from_millis(50));
        }
        actions
    }

    /// The scrolled strip of slots, brackets and the marker.
    #[allow(
        clippy::too_many_lines,
        reason = "one draw pass over the visible slots"
    )]
    #[allow(
        clippy::too_many_arguments,
        reason = "private drawing helper over disjoint borrows of the caller's state"
    )]
    fn strip(
        &mut self,
        ui: &mut egui::Ui,
        viewport: egui::Rect,
        status: &RecomputeStatus,
        tokens: &crate::theme::ThemeTokens,
        valid_range: &dyn Fn(FeatureId) -> Option<RangeInclusive<usize>>,
        provider_menu: &dyn Fn(&mut egui::Ui, FeatureId) -> Option<crate::command::CmdId>,
        actions: &mut Vec<Action>,
    ) {
        #[allow(clippy::cast_precision_loss, reason = "slot counts are small")]
        let total_w = (self.slots.len() as f32 + 0.5) * SLOT_W;
        let height = ui.available_height().max(NODE_TOP + NODE_H + 6.0);
        let (content, _) =
            ui.allocate_exact_size(egui::vec2(total_w, height), egui::Sense::hover());
        let origin = content.min;
        let painter = ui.painter().clone();
        let pointer_x = ui.ctx().pointer_interact_pos().map(|p| p.x - origin.x);
        let visuals = ui.visuals().clone();
        let len = self.items.len();
        // Marker position shown (drag preview or the document's).
        let shown_marker = self.marker_drag().unwrap_or(self.marker);
        let computing = computing_node(status, len, self.marker);
        // Drag state of a node.
        let node_drag = match &self.drag {
            Some(Drag::Node {
                id,
                from,
                valid,
                to,
            }) => Some((*id, *from, valid.clone(), *to)),
            _ => None,
        };
        // Visible slots (plus the dragged node's slot).
        let first = self.slot_at(viewport.min.x.max(0.0));
        let last = self.slot_at(viewport.max.x);
        let mut slots: Vec<usize> = (first..=last.min(self.slots.len() - 1)).collect();
        if let Some((_, from, ..)) = &node_drag
            && let Some(&s) = self.slot_of.get(*from)
            && !slots.contains(&s)
        {
            slots.push(s);
        }
        let mut hover = None;
        for s in slots {
            #[allow(clippy::cast_precision_loss, reason = "slot counts are small")]
            let x = origin.x + s as f32 * SLOT_W;
            let rect = egui::Rect::from_min_size(
                egui::pos2(x + PAD, origin.y + NODE_TOP),
                egui::vec2(SLOT_W - 2.0 * PAD, NODE_H),
            );
            match self.slots[s].clone() {
                Slot::Collapsed(g, range) => {
                    let name = self
                        .group_name(g)
                        .unwrap_or_else(|| format!("Group ({} features)", range.clone().count()));
                    let id = ui.id().with(("amg_timeline_group", g));
                    let r = ui.interact(rect, id, egui::Sense::click());
                    let rolled = *range.start() >= shown_marker;
                    let fill = slot_fill(&visuals, rolled, false, false, tokens);
                    painter.rect_filled(rect, 3.0, fill);
                    painter.rect_stroke(
                        rect,
                        3.0,
                        egui::Stroke::new(1.0, visuals.widgets.noninteractive.fg_stroke.color),
                        egui::StrokeKind::Inside,
                    );
                    painter.text(
                        rect.center(),
                        egui::Align2::CENTER_CENTER,
                        "▣",
                        egui::FontId::proportional(12.0),
                        visuals.text_color(),
                    );
                    let label = format!("Expand {name}");
                    r.widget_info(|| {
                        egui::WidgetInfo::labeled(egui::WidgetType::Button, true, label.clone())
                    });
                    let r = r.on_hover_text(&name);
                    if r.clicked() {
                        actions.push(Action::Collapse(g, false));
                    }
                    r.context_menu(|ui| {
                        if ui.button("Expand").clicked() {
                            actions.push(Action::Collapse(g, false));
                            ui.close();
                        }
                        if ui.button("Ungroup").clicked() {
                            actions.push(Action::Ungroup(g));
                            ui.close();
                        }
                    });
                }
                Slot::Node(i) => {
                    let item = self.items[i].clone();
                    let id = ui.id().with(("amg_timeline_node", item.id));
                    let r = ui.interact(rect, id, egui::Sense::click_and_drag());
                    let rolled = i >= shown_marker;
                    let is_error = matches!(item.health, NodeHealth::Error(..));
                    let dragged_self = node_drag.as_ref().is_some_and(|(d, ..)| *d == item.id);
                    let invalid_drop = node_drag.as_ref().is_some_and(|(_, _, valid, _)| {
                        valid.as_ref().is_none_or(|v| !v.contains(&i))
                    });
                    let selected = self.selected.contains(&item.id);
                    let mut fill = slot_fill(&visuals, rolled, is_error, item.suppressed, tokens);
                    if dragged_self {
                        fill = fill.gamma_multiply(0.5);
                    }
                    painter.rect_filled(rect, 3.0, fill);
                    if selected || r.hovered() {
                        let c = if selected {
                            visuals.selection.stroke.color
                        } else {
                            visuals.widgets.hovered.fg_stroke.color
                        };
                        painter.rect_stroke(
                            rect,
                            3.0,
                            egui::Stroke::new(1.5, c),
                            egui::StrokeKind::Inside,
                        );
                    }
                    let mut text_color = visuals.text_color();
                    if rolled || item.suppressed {
                        text_color = text_color.gamma_multiply(0.45);
                    }
                    let icon_id = self.icon_ids.get(i).copied().unwrap_or(IconId::NONE);
                    let drawn = crate::icons::paint(
                        ui.ctx(),
                        &painter,
                        rect.shrink(4.0),
                        icon_id,
                        text_color,
                    );
                    if !drawn {
                        painter.text(
                            rect.center(),
                            egui::Align2::CENTER_CENTER,
                            &item.icon,
                            egui::FontId::proportional(11.0),
                            text_color,
                        );
                    }
                    if item.suppressed {
                        painter.line_segment(
                            [rect.left_bottom(), rect.right_top()],
                            egui::Stroke::new(1.0, text_color),
                        );
                    }
                    if invalid_drop {
                        painter.rect_filled(
                            rect.expand(PAD),
                            0.0,
                            tokens.error.gamma_multiply(0.35),
                        );
                    }
                    let is_computing = computing.is_some_and(|(c, _)| c == i);
                    if let Some((c, frac)) = computing
                        && c == i
                    {
                        progress_ring(&painter, rect, frac, tokens.accent);
                    }
                    let label = item.label.clone();
                    r.widget_info(|| {
                        egui::WidgetInfo::labeled(egui::WidgetType::Button, true, label.clone())
                    });
                    // Badge.
                    let badge = match &item.health {
                        NodeHealth::Warning(_) => Some(Severity::Warning),
                        NodeHealth::Error(..) => Some(Severity::Error),
                        NodeHealth::Ok => None,
                    };
                    if let Some(sev) = badge {
                        let c = egui::pos2(rect.right() - 3.0, rect.top() + 3.0);
                        let brect = egui::Rect::from_center_size(c, egui::vec2(8.0, 8.0));
                        let color = if sev == Severity::Error {
                            tokens.error
                        } else {
                            tokens.warning
                        };
                        painter.circle_filled(c, 3.5, color);
                        let bid = ui.id().with(("amg_timeline_badge", item.id));
                        let br = ui.interact(brect, bid, egui::Sense::hover());
                        let name = if sev == Severity::Error {
                            "Error"
                        } else {
                            "Warning"
                        };
                        br.widget_info(|| {
                            egui::WidgetInfo::labeled(egui::WidgetType::Label, true, name)
                        });
                        let _ = br.on_hover_text(health_text(&item.health));
                    }
                    let tip = match &item.health {
                        NodeHealth::Ok => item.label.clone(),
                        h => format!("{}\n{}", item.label, health_text(h)),
                    };
                    let r = r.on_hover_text(tip);
                    if r.hovered() && self.drag.is_none() {
                        hover = Some(item.id);
                    }
                    self.drawn.push(DrawnNode {
                        id: item.id,
                        index: i,
                        rect,
                        rolled_back: rolled,
                        suppressed: item.suppressed,
                        badge,
                        error_tint: is_error,
                        invalid_drop,
                        computing: is_computing,
                    });
                    // Interaction.
                    if r.drag_started() && item.known {
                        let valid = valid_range(item.id);
                        self.drag = Some(Drag::Node {
                            id: item.id,
                            from: i,
                            valid,
                            to: None,
                        });
                    }
                    if r.double_clicked() {
                        actions.push(Action::Edit(item.id));
                    } else if r.clicked() {
                        let toggle = ui.input(|i| i.modifiers.command || i.modifiers.shift);
                        actions.push(Action::Select {
                            id: item.id,
                            toggle,
                        });
                    }
                    let fid = item.id;
                    let editable = !matches!(item.edit, Some(EditAction::NotEditable));
                    let sketch = item.sketch;
                    let suppressed = item.suppressed;
                    let group = item.group;
                    r.context_menu(|ui| {
                        let mut entry =
                            |ui: &mut egui::Ui, enabled: bool, text: &str, a: Action| {
                                if ui.add_enabled(enabled, egui::Button::new(text)).clicked() {
                                    actions.push(a);
                                    ui.close();
                                }
                            };
                        entry(ui, editable, "Edit Feature", Action::Edit(fid));
                        if let Some(s) = sketch {
                            entry(ui, true, "Edit Sketch", Action::Edit(s));
                        }
                        ui.separator();
                        entry(ui, true, "Roll to Here", Action::Marker(i + 1));
                        let (text, to) = if suppressed {
                            ("Unsuppress", false)
                        } else {
                            ("Suppress", true)
                        };
                        entry(ui, true, text, Action::Suppress(fid, to));
                        entry(ui, true, "Rename", Action::StartRename(fid));
                        entry(ui, true, "Delete", Action::Delete(fid));
                        ui.separator();
                        match group {
                            Some(g) => entry(ui, true, "Ungroup", Action::Ungroup(g)),
                            None => entry(ui, true, "Group", Action::Group(fid)),
                        }
                        entry(ui, true, "Find in Browser", Action::FindInBrowser(fid));
                        if let Some(cmd) = provider_menu(ui, fid) {
                            actions.push(Action::Run(cmd));
                        }
                    });
                }
            }
        }
        // Group brackets.
        for b in &self.brackets {
            #[allow(clippy::cast_precision_loss, reason = "slot counts are small")]
            let (x0, x1) = (
                origin.x + *b.slots.start() as f32 * SLOT_W + PAD,
                origin.x + (*b.slots.end() + 1) as f32 * SLOT_W - PAD,
            );
            if x1 < origin.x + viewport.min.x || x0 > origin.x + viewport.max.x {
                continue;
            }
            let y = origin.y + NODE_TOP + NODE_H + 2.0;
            let rect = egui::Rect::from_min_max(egui::pos2(x0, y), egui::pos2(x1, y + 5.0));
            let id = ui.id().with(("amg_timeline_bracket", b.group));
            let r = ui.interact(rect, id, egui::Sense::click());
            let color = if r.hovered() {
                visuals.widgets.hovered.fg_stroke.color
            } else {
                visuals.widgets.noninteractive.fg_stroke.color
            };
            let stroke = egui::Stroke::new(1.5, color);
            painter.line_segment([egui::pos2(x0, y + 3.0), egui::pos2(x1, y + 3.0)], stroke);
            painter.line_segment([egui::pos2(x0, y), egui::pos2(x0, y + 3.0)], stroke);
            painter.line_segment([egui::pos2(x1, y), egui::pos2(x1, y + 3.0)], stroke);
            let label = format!("Collapse {}", b.name);
            r.widget_info(|| {
                egui::WidgetInfo::labeled(egui::WidgetType::Button, true, label.clone())
            });
            let r = r.on_hover_text(&b.name);
            if r.clicked() {
                actions.push(Action::Collapse(b.group, true));
            }
            let g = b.group;
            r.context_menu(|ui| {
                if ui.button("Ungroup").clicked() {
                    actions.push(Action::Ungroup(g));
                    ui.close();
                }
            });
            self.bracket_rects.push((b.group, rect));
        }
        // Node drag: drop position and line.
        if let Some((id, from, valid, _)) = node_drag {
            let released = ui.input(|i| i.pointer.any_released());
            let to = pointer_x.map(|x| self.drop_index(from, self.boundary_at(x)));
            let ok = to.is_some_and(|t| valid.as_ref().is_some_and(|v| v.contains(&t)));
            if let (Some(t), Some(x)) = (to, pointer_x) {
                #[allow(clippy::cast_precision_loss, reason = "slot counts are small")]
                let bx = origin.x + self.boundary_at(x) as f32 * SLOT_W;
                let color = if ok { tokens.accent } else { tokens.error };
                painter.line_segment(
                    [
                        egui::pos2(bx, origin.y + NODE_TOP - 1.0),
                        egui::pos2(bx, origin.y + NODE_TOP + NODE_H + 1.0),
                    ],
                    egui::Stroke::new(2.0, color),
                );
                let _ = t;
            }
            self.drag_view = Some(DragView { id, to, valid: ok });
            if released {
                self.drag = None;
                if let Some(t) = to
                    && ok
                    && t != from
                {
                    actions.push(Action::Move(id, t));
                }
            } else {
                self.drag = Some(Drag::Node {
                    id,
                    from,
                    valid,
                    to,
                });
            }
        }
        // Rollback marker.
        let mb = self.node_boundary(shown_marker);
        #[allow(clippy::cast_precision_loss, reason = "slot counts are small")]
        let mx = origin.x + mb as f32 * SLOT_W;
        let mrect = egui::Rect::from_center_size(
            egui::pos2(mx, origin.y + NODE_TOP + NODE_H / 2.0),
            egui::vec2(MARKER_W, NODE_H + 4.0),
        );
        let mid = ui.id().with("amg_timeline_marker");
        let mr = ui.interact(mrect, mid, egui::Sense::drag());
        painter.rect_filled(
            egui::Rect::from_center_size(mrect.center(), egui::vec2(3.0, mrect.height())),
            1.0,
            tokens.accent,
        );
        painter.circle_filled(egui::pos2(mx, mrect.top() + 1.0), 3.0, tokens.accent);
        mr.widget_info(|| {
            egui::WidgetInfo::labeled(egui::WidgetType::Slider, true, "Rollback marker")
        });
        let _ = mr.clone().on_hover_text("Rollback marker (drag)");
        self.marker_rect = Some(mrect);
        if mr.drag_started() || (mr.dragged() && !matches!(self.drag, Some(Drag::Marker { .. }))) {
            self.drag = Some(Drag::Marker { to: self.marker });
        }
        if matches!(self.drag, Some(Drag::Marker { .. })) {
            let to = pointer_x.map_or(self.marker, |x| self.boundary_node(self.boundary_at(x)));
            if mr.drag_stopped() || ui.input(|i| i.pointer.any_released()) {
                self.drag = None;
                if to != self.marker {
                    actions.push(Action::Marker(to));
                }
            } else {
                self.drag = Some(Drag::Marker { to });
                ui.ctx().request_repaint();
            }
        }
        if hover != self.hover {
            self.hover = hover;
            ui.ctx().request_repaint();
        }
    }

    fn group_name(&self, g: GroupId) -> Option<String> {
        self.group_names
            .iter()
            .find(|(id, _)| *id == g)
            .map(|(_, n)| n.clone())
    }
}

/// The display-name base and the label of a timeline node: its name, else
/// `<display name><n>`, counting the nodes of that base seen so far in `ordinals` (call it for
/// every node in timeline order). The parameters dialog (UI-12) labels its groups the same way.
pub(crate) fn node_label(
    node: &amg_doc::FeatureNode,
    registry: &crate::UiRegistry,
    ordinals: &mut std::collections::HashMap<String, u32>,
) -> (String, String) {
    let base = registry.feature_ui(node.feature.kind()).map_or_else(
        || fallback_name(node.feature.kind()),
        |i| i.display_name.to_owned(),
    );
    let n = ordinals.entry(base.clone()).or_insert(0);
    *n += 1;
    let label = node.name.clone().unwrap_or_else(|| format!("{base}{n}"));
    (base, label)
}

/// The default label base of a kind without a [`FeatureUiInfo`]: its last dotted segment.
fn fallback_name(kind: &str) -> String {
    kind.rsplit('.').next().unwrap_or(kind).to_owned()
}

/// Up to two initials of a display name ("Extrude" → "Ex", "Shell Body" → "SB").
fn initials(name: &str) -> String {
    let words: Vec<&str> = name.split_whitespace().collect();
    if words.len() >= 2 {
        words
            .iter()
            .take(2)
            .filter_map(|w| w.chars().next())
            .collect()
    } else {
        name.chars().take(2).collect()
    }
}

/// Tooltip text of a health.
fn health_text(h: &NodeHealth) -> String {
    match h {
        NodeHealth::Ok => String::new(),
        NodeHealth::Warning(ms) => ms.join("\n"),
        NodeHealth::Error(m, _) => m.clone(),
    }
}

/// The fill of a slot.
fn slot_fill(
    visuals: &egui::Visuals,
    rolled: bool,
    error: bool,
    suppressed: bool,
    tokens: &crate::theme::ThemeTokens,
) -> egui::Color32 {
    let base = visuals.widgets.inactive.bg_fill;
    let c = if error {
        lerp_color(base, tokens.error, 0.55)
    } else {
        base
    };
    if rolled || suppressed {
        c.gamma_multiply(0.4)
    } else {
        c
    }
}

fn lerp_color(a: egui::Color32, b: egui::Color32, t: f32) -> egui::Color32 {
    let l = |x: u8, y: u8| {
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            reason = "in 0..=255"
        )]
        let v = (f32::from(x) + (f32::from(y) - f32::from(x)) * t).round() as u8;
        v
    };
    egui::Color32::from_rgb(l(a.r(), b.r()), l(a.g(), b.g()), l(a.b(), b.b()))
}

/// The node a running recompute is computing and the fraction of it done: the recompute
/// reports `index / len` when it starts node `index` (S01 DOC-12), so the node is the integer
/// part of `progress · len`. `None` when idle or past the marker.
pub fn computing_node(status: &RecomputeStatus, len: usize, marker: usize) -> Option<(usize, f32)> {
    let RecomputeStatus::Running { progress, .. } = status else {
        return None;
    };
    if len == 0 {
        return None;
    }
    #[allow(clippy::cast_precision_loss, reason = "node counts are small")]
    let x = progress.clamp(0.0, 1.0) * len as f32;
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "clamped to 0..=len"
    )]
    let i = (x.floor() as usize).min(len - 1);
    #[allow(clippy::cast_precision_loss, reason = "node counts are small")]
    let frac = (x - i as f32).clamp(0.0, 1.0);
    (i < marker).then_some((i, frac))
}

/// A progress ring around the centre of `rect`: a faint full circle and an arc for `frac`.
fn progress_ring(painter: &egui::Painter, rect: egui::Rect, frac: f32, color: egui::Color32) {
    let c = rect.center();
    let r = rect.height().min(rect.width()) / 2.0 - 1.0;
    painter.circle_stroke(c, r, egui::Stroke::new(1.0, color.gamma_multiply(0.3)));
    let n = 24;
    let arc = frac.max(0.05) * std::f32::consts::TAU;
    let pts: Vec<egui::Pos2> = (0..=n)
        .map(|k| {
            #[allow(clippy::cast_precision_loss, reason = "small")]
            let a = -std::f32::consts::FRAC_PI_2 + arc * k as f32 / n as f32;
            c + egui::vec2(a.cos(), a.sin()) * r
        })
        .collect();
    painter.add(egui::Shape::line(pts, egui::Stroke::new(2.0, color)));
}

// ---- app integration ---------------------------------------------------------------------

impl AmalgamApp {
    /// Draws the timeline of the active document into `ui` (the shell's bottom strip calls
    /// it; benches call it alone) and applies what the user did.
    pub fn timeline_ui(&mut self, ui: &mut egui::Ui) {
        let _span = tracing::debug_span!("amg_ui.timeline").entered();
        let active = self.active;
        let tokens = self.tokens.clone();
        let mut state = std::mem::take(&mut self.docs[active].timeline);
        let actions = self.with_cx(|cx| state.show(ui, cx, &tokens));
        for a in actions {
            self.apply_timeline_action(&mut state, a);
        }
        self.docs[active].timeline = state;
    }

    /// The timeline of the active document.
    pub fn timeline(&self) -> &TimelineState {
        &self.docs[self.active].timeline
    }

    /// The timeline of the active document, mutable.
    pub fn timeline_mut(&mut self) -> &mut TimelineState {
        &mut self.docs[self.active].timeline
    }

    fn apply_timeline_action(&mut self, state: &mut TimelineState, a: Action) {
        let submit = |app: &mut AmalgamApp, cmd: DocCommand| {
            app.submit(CommitBatch {
                label: cmd.label().to_owned(),
                commands: vec![cmd],
            });
        };
        match a {
            Action::Marker(index) => {
                if let Some(index) = self.guard_timeline(GuardedAction::SetMarker { index }) {
                    submit(self, DocCommand::SetMarker { index });
                }
            }
            Action::Move(id, to) => {
                if self
                    .guard_timeline(GuardedAction::MoveFeature { id, to })
                    .is_some()
                {
                    submit(self, DocCommand::MoveFeature { id, to });
                }
            }
            Action::Select { id, toggle } => {
                if toggle {
                    if let Some(p) = state.selected.iter().position(|s| *s == id) {
                        state.selected.remove(p);
                    } else {
                        state.selected.push(id);
                    }
                } else {
                    state.selected = vec![id];
                }
                self.timeline_repaint();
            }
            Action::Edit(id) => {
                if self
                    .guard_timeline(GuardedAction::EditFeature(id))
                    .is_some()
                {
                    self.edit_feature(id);
                }
            }
            Action::Suppress(id, suppressed) => {
                let ids = state.targets(id);
                submit(self, DocCommand::SetSuppressed { ids, suppressed });
            }
            Action::StartRename(id) => {
                let text = state.label(id).unwrap_or_default().to_owned();
                state.renaming = Some(Renaming {
                    id,
                    text,
                    focus: true,
                });
                self.timeline_repaint();
            }
            Action::Rename(id, name) => {
                if name.is_empty() || state.label(id) == Some(name.as_str()) {
                    return;
                }
                submit(
                    self,
                    DocCommand::RenameFeature {
                        id,
                        name: Some(name),
                    },
                );
            }
            Action::Delete(id) => {
                let ids = state.targets(id);
                submit(self, DocCommand::DeleteFeatures { ids });
            }
            Action::Group(id) => {
                let ids = state.targets(id);
                let tl = self.docs[self.active].document.design().timeline();
                let n = tl.groups().count() + 1;
                submit(
                    self,
                    DocCommand::GroupFeatures {
                        ids,
                        name: format!("Group{n}"),
                    },
                );
            }
            Action::Ungroup(group) => submit(self, DocCommand::Ungroup { group }),
            Action::Collapse(group, collapsed) => submit(
                self,
                DocCommand::SetGroup {
                    group,
                    name: None,
                    collapsed: Some(collapsed),
                },
            ),
            Action::FindInBrowser(id) => {
                let n = id.0;
                let found = self.docs[self.active].browser.reveal_first(|k| {
                    let k = k.as_str();
                    [
                        format!("body/{n}:"),
                        format!("sketch/{n}"),
                        format!("construction/{n}"),
                    ]
                    .iter()
                    .any(|p| {
                        k.strip_prefix(p.as_str()).is_some_and(|rest| {
                            p.ends_with(':') || rest.is_empty() || rest.starts_with('@')
                        })
                    })
                });
                if !found {
                    self.notify(crate::notify::Notification::info(
                        "Not in the browser",
                        "This feature has no body, sketch or construction in the browser.",
                    ));
                }
                self.timeline_repaint();
            }
            Action::Run(cmd) => self.request_command(cmd),
        }
    }

    /// Asks the position guard (ASM-02, [`crate::guard`]) about a timeline action while moves
    /// are pending. `Some(marker index or 0)` means: go ahead as before; `None`: the guard
    /// kept the action. Without pending moves or a guard nothing changes.
    fn guard_timeline(&mut self, action: GuardedAction) -> Option<usize> {
        let index = match action {
            GuardedAction::SetMarker { index } => index,
            _ => 0,
        };
        let pending = self.docs[self.active]
            .document
            .design()
            .has_pending_positions();
        let guard = self.registry.position_guard();
        match guard {
            Some(g) if pending => {
                let back = self.with_cx(|cx| g(cx, action));
                back.map(|_| index)
            }
            _ => Some(index),
        }
    }

    fn timeline_repaint(&self) {
        if let Some(ctx) = &self.ctx {
            ctx.request_repaint();
        }
    }

    /// Targets of the timeline hover: every displayed instance of the hovered node's output
    /// bodies (`BodyId.origin == id`). Empty when nothing is hovered.
    pub(crate) fn timeline_hover_targets(&self) -> Vec<PickTarget> {
        let doc = &self.docs[self.active];
        let Some(id) = doc.timeline.hover else {
            return Vec::new();
        };
        output_targets(doc.document.evaluated(), &self.scene, id)
    }

    /// Targets of the hovered error node's `Diagnostic.entities` (the Error highlight layer).
    pub(crate) fn timeline_error_targets(&self) -> Vec<PickTarget> {
        let doc = &self.docs[self.active];
        let Some(id) = doc.timeline.hover else {
            return Vec::new();
        };
        let Some(NodeHealth::Error(_, entities)) = doc.timeline.health(id) else {
            return Vec::new();
        };
        entity_targets(
            doc.document.design(),
            doc.document.evaluated(),
            &self.scene,
            entities,
        )
    }
}

impl TimelineState {
    /// The nodes an action on `id` applies to: the timeline selection if `id` is in it, else
    /// just `id`.
    fn targets(&self, id: FeatureId) -> Vec<FeatureId> {
        if self.selected.contains(&id) && self.selected.len() > 1 {
            let mut ids = self.selected.clone();
            let index = |f: &FeatureId| self.items.iter().position(|i| i.id == *f);
            ids.sort_by_key(index);
            ids
        } else {
            vec![id]
        }
    }
}

/// Every displayed instance of the bodies feature `id` created.
pub fn output_targets(
    eval: &Evaluated,
    scene: &crate::SceneAdapter,
    id: FeatureId,
) -> Vec<PickTarget> {
    let mut out = Vec::new();
    for (body, _) in eval.bodies().filter(|(b, _)| b.origin == id) {
        for instance in scene.pick_map().instances_of(body) {
            out.push(PickTarget::Body {
                body,
                instance,
                sub: None,
            });
        }
    }
    out
}

/// Pick targets of diagnostic entities (those that resolve in the current evaluation).
pub fn entity_targets(
    design: &Design,
    eval: &Evaluated,
    scene: &crate::SceneAdapter,
    entities: &[EntityRef],
) -> Vec<PickTarget> {
    entities
        .iter()
        .filter_map(|e| crate::select::target_for_entity(design, eval, scene, e))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn computing_node_follows_the_recompute_progress() {
        let running = |p| RecomputeStatus::Running {
            progress: p,
            message: None,
        };
        assert_eq!(computing_node(&RecomputeStatus::Idle, 10, 10), None);
        assert_eq!(computing_node(&running(0.0), 10, 10), Some((0, 0.0)));
        let (i, f) = computing_node(&running(0.35), 10, 10).unwrap();
        assert_eq!(i, 3);
        assert!((f - 0.5).abs() < 1e-4);
        assert_eq!(computing_node(&running(1.0), 10, 10).map(|c| c.0), Some(9));
        // Past the marker nothing is computed.
        assert_eq!(computing_node(&running(0.75), 10, 5), None);
        assert_eq!(computing_node(&running(0.5), 0, 0), None);
    }

    #[test]
    fn initials_and_fallback_names() {
        assert_eq!(initials("Extrude"), "Ex");
        assert_eq!(initials("Shell Body"), "SB");
        assert_eq!(fallback_name("uitest.box"), "box");
        assert_eq!(fallback_name("plain"), "plain");
    }

    fn state_with(groups: &[Option<u64>], collapsed: &[u64]) -> TimelineState {
        let mut s = TimelineState::default();
        for (n, g) in groups.iter().enumerate() {
            s.items.push(Item {
                id: FeatureId(n as u64 + 100),
                label: format!("F{n}"),
                icon: String::new(),
                known: true,
                edit: None,
                group: g.map(GroupId),
                suppressed: false,
                health: NodeHealth::Ok,
                sketch: None,
            });
        }
        let mut i = 0;
        while i < s.items.len() {
            match s.items[i].group {
                Some(g) if collapsed.contains(&g.0) => {
                    let mut end = i;
                    while end + 1 < s.items.len() && s.items[end + 1].group == Some(g) {
                        end += 1;
                    }
                    for _ in i..=end {
                        s.slot_of.push(s.slots.len());
                    }
                    s.slots.push(Slot::Collapsed(g, i..=end));
                    i = end + 1;
                }
                _ => {
                    s.slot_of.push(s.slots.len());
                    s.slots.push(Slot::Node(i));
                    i += 1;
                }
            }
        }
        s
    }

    #[test]
    fn boundaries_map_between_slots_and_nodes() {
        // Nodes 0, [1 2 3] collapsed, 4.
        let s = state_with(&[None, Some(7), Some(7), Some(7), None], &[7]);
        assert_eq!(s.slots.len(), 3);
        assert_eq!(s.boundary_node(0), 0);
        assert_eq!(s.boundary_node(1), 1);
        assert_eq!(s.boundary_node(2), 4);
        assert_eq!(s.boundary_node(3), 5);
        assert_eq!(s.node_boundary(0), 0);
        assert_eq!(s.node_boundary(1), 1);
        assert_eq!(s.node_boundary(2), 2, "inside a collapsed group: after it");
        assert_eq!(s.node_boundary(5), 3);
        assert_eq!(s.boundary_at(-5.0), 0);
        assert_eq!(s.boundary_at(SLOT_W * 1.4), 1);
        assert_eq!(s.boundary_at(SLOT_W * 99.0), 3);
    }

    #[test]
    fn drop_index_is_the_final_index() {
        let s = state_with(&[None; 6], &[]);
        // Node 4 dropped at boundary 1 (between 0 and 1) ends at index 1.
        assert_eq!(s.drop_index(4, 1), 1);
        // Node 1 dropped at boundary 4 (between 3 and 4) ends at index 3.
        assert_eq!(s.drop_index(1, 4), 3);
        // Dropped on its own boundaries: unchanged.
        assert_eq!(s.drop_index(2, 2), 2);
        assert_eq!(s.drop_index(2, 3), 2);
        assert_eq!(s.drop_index(0, 6), 5);
    }

    #[test]
    fn actions_on_a_selected_node_apply_to_the_whole_selection_in_order() {
        let mut s = state_with(&[None; 4], &[]);
        s.selected = vec![FeatureId(102), FeatureId(101)];
        assert_eq!(
            s.targets(FeatureId(101)),
            vec![FeatureId(101), FeatureId(102)]
        );
        assert_eq!(s.targets(FeatureId(103)), vec![FeatureId(103)]);
    }
}
