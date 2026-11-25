//! The assembly decoration and row menus of the browser (S06 §7).
//!
//! The rows are the framework's; this provider reads their keys (see
//! `amg_ui::browser::framework`: `root`, `occ/<occurrence>`, `body/<feature>:<slot>`,
//! `bodies/<component>`, each with an optional `@<path>` suffix).

use amg_doc::{BodyId, ComponentId, DocEvent, FeatureId, OccurrenceId};

use super::actions::{self, Destination};
use super::state::AssemblyLayer;
use super::{icon_ids, labels};
use crate::app::{AppCx, CmdEnv};
use crate::browser::{
    BrowserAnchor, BrowserNode, BrowserProvider, NodeKey, RowDecoration, RowItem, RowItemKind,
};

/// Row item ids.
const ACTIVATE: &str = "activate";
const CAPTURE: &str = "capture";

/// A framework row this module acts on.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Row {
    Root,
    Occurrence(OccurrenceId),
    Body(BodyId),
    Bodies(ComponentId),
}

fn parse(key: &NodeKey) -> Option<Row> {
    let base = key.as_str().split('@').next()?;
    if base == "root" {
        return Some(Row::Root);
    }
    let (kind, rest) = base.split_once('/')?;
    match kind {
        "occ" => Some(Row::Occurrence(OccurrenceId(rest.parse().ok()?))),
        "bodies" => Some(Row::Bodies(ComponentId(rest.parse().ok()?))),
        "body" => {
            let (f, s) = rest.split_once(':')?;
            Some(Row::Body(BodyId {
                origin: FeatureId(f.parse().ok()?),
                slot: s.parse().ok()?,
            }))
        }
        _ => None,
    }
}

/// The component a row stands for (the root and occurrence rows).
fn component_of(env: &CmdEnv<'_>, row: Row) -> Option<ComponentId> {
    match row {
        Row::Root => Some(env.design.root()),
        Row::Occurrence(o) => env.design.occurrence(o).map(|o| o.component),
        Row::Body(_) | Row::Bodies(_) => None,
    }
}

/// The assembly provider: no nodes of its own.
#[derive(Debug)]
pub(super) struct AssemblyRows;

impl BrowserProvider for AssemblyRows {
    fn id(&self) -> &'static str {
        "assembly"
    }

    fn affected_by(&self, _ev: &DocEvent) -> bool {
        false
    }

    fn build(&self, _env: &CmdEnv<'_>, _anchor: &BrowserAnchor, _out: &mut Vec<BrowserNode>) {}

    fn decorate(&self, env: &CmdEnv<'_>, node: &BrowserNode, out: &mut RowDecoration) {
        let Some(row) = parse(&node.key) else {
            return;
        };
        let mark = |id, icon, fallback, label: String| RowItem {
            id,
            icon,
            fallback,
            label,
            kind: RowItemKind::Mark,
        };
        if let Row::Occurrence(o) = row {
            if env.design.occurrence(o).is_some_and(|d| d.grounded) {
                out.items.push(mark(
                    "grounded",
                    icon_ids::GROUNDED,
                    "G",
                    format!("{} {}", labels::GROUNDED, node.label),
                ));
            }
            if env.design.pending_positions().get(o).is_some() {
                out.items.push(mark(
                    "pending",
                    icon_ids::PENDING,
                    "*",
                    format!("{} {}", labels::PENDING, node.label),
                ));
            }
        }
        // A design without occurrences has nothing to switch between.
        if env.design.occurrences().next().is_some()
            && let Some(component) = component_of(env, row)
        {
            let selected = component == env.active_component;
            out.emphasized = selected;
            out.items.push(RowItem {
                id: ACTIVATE,
                icon: if selected {
                    icon_ids::ACTIVE
                } else {
                    icon_ids::INACTIVE
                },
                fallback: if selected { "◉" } else { "○" },
                label: format!("{} {}", labels::ACTIVATE, node.label),
                kind: RowItemKind::Radio { selected },
            });
        }
        if row == Row::Root
            && env
                .doc_layer::<AssemblyLayer>()
                .is_some_and(|l| !l.out_of_date().is_empty())
        {
            out.items.push(RowItem {
                id: CAPTURE,
                icon: icon_ids::OUT_OF_DATE,
                fallback: "⚠",
                label: labels::OUT_OF_DATE.to_owned(),
                kind: RowItemKind::Button,
            });
        }
    }

    fn item_clicked(&self, node: &NodeKey, item: &'static str, cx: &mut AppCx<'_>) {
        match item {
            ACTIVATE => {
                let component = parse(node).and_then(|row| component_of(&cx.env(), row));
                if let Some(c) = component {
                    actions::activate(cx, c);
                }
            }
            CAPTURE => actions::capture(cx),
            _ => {}
        }
    }

    fn row_menu(&self, node: &NodeKey, ui: &mut egui::Ui, cx: &mut AppCx<'_>) {
        let Some(row) = parse(node) else {
            return;
        };
        match row {
            Row::Root => {
                ui.separator();
                let root = cx.env().design.root();
                component_entries(ui, cx, root);
                paste_entry(ui, cx, root);
            }
            Row::Occurrence(o) => occurrence_menu(ui, cx, o),
            Row::Body(b) => {
                let bodies = {
                    let env = cx.env();
                    if env.evaluated.body(b).is_none() {
                        return;
                    }
                    actions::targets(actions::selected_bodies(&env), b)
                };
                ui.separator();
                create_entry(ui, cx, &bodies);
                if actions::bodies_blocked(&cx.env()).is_none() {
                    move_menu(
                        ui,
                        cx,
                        |env| actions::body_destinations(env, &bodies),
                        |cx, to| actions::move_bodies_to(cx, &bodies, to),
                    );
                }
            }
            Row::Bodies(c) => {
                let bodies: Vec<BodyId> = cx
                    .env()
                    .evaluated
                    .bodies()
                    .filter(|(_, b)| b.component == c)
                    .map(|(id, _)| id)
                    .collect();
                if !bodies.is_empty() {
                    ui.separator();
                    create_entry(ui, cx, &bodies);
                }
            }
        }
    }
}

/// A menu entry; true if it was clicked (the menu is then closed).
fn entry(ui: &mut egui::Ui, label: &str) -> bool {
    let clicked = ui.button(label).clicked();
    if clicked {
        ui.close();
    }
    clicked
}

/// New Component and, unless it is the active one, Activate.
fn component_entries(ui: &mut egui::Ui, cx: &mut AppCx<'_>, component: ComponentId) {
    if entry(ui, labels::NEW_COMPONENT) {
        actions::new_component(cx, component);
    }
    if cx.env().active_component != component && entry(ui, labels::ACTIVATE) {
        actions::activate(cx, component);
    }
}

fn paste_entry(ui: &mut egui::Ui, cx: &mut AppCx<'_>, target: ComponentId) {
    if !actions::pastable(&cx.env(), target).is_empty() && entry(ui, labels::PASTE) {
        actions::paste(cx, target);
    }
}

fn create_entry(ui: &mut egui::Ui, cx: &mut AppCx<'_>, bodies: &[BodyId]) {
    let blocked = actions::bodies_blocked(&cx.env());
    let mut r = ui.add_enabled(blocked.is_none(), egui::Button::new(labels::FROM_BODIES));
    if let Some(reason) = blocked {
        r = r.on_disabled_hover_text(reason);
    }
    if r.clicked() {
        actions::create_components(cx, bodies);
        ui.close();
    }
}

fn occurrence_menu(ui: &mut egui::Ui, cx: &mut AppCx<'_>, occ: OccurrenceId) {
    let (component, occs, any_free, any_grounded) = {
        let env = cx.env();
        let Some(def) = env.design.occurrence(occ) else {
            return;
        };
        let occs = actions::targets(actions::selected_occurrences(&env), occ);
        let grounded = |o: &OccurrenceId| env.design.occurrence(*o).map(|d| d.grounded);
        let any_free = occs.iter().any(|o| grounded(o) == Some(false));
        let any_grounded = occs.iter().any(|o| grounded(o) == Some(true));
        (def.component, occs, any_free, any_grounded)
    };
    ui.separator();
    component_entries(ui, cx, component);
    if any_free && entry(ui, labels::GROUND) {
        actions::set_grounded(cx, &occs, true);
    }
    if any_grounded && entry(ui, labels::UNGROUND) {
        actions::set_grounded(cx, &occs, false);
    }
    if entry(ui, labels::COPY) {
        actions::copy(cx, &occs);
    }
    paste_entry(ui, cx, component);
    move_menu(
        ui,
        cx,
        |env| actions::occurrence_destinations(env, &occs),
        |cx, to| actions::move_occurrences_to(cx, &occs, to),
    );
    if entry(ui, labels::DELETE) {
        actions::delete(cx, occs.clone());
    }
}

/// The "Move to Component" submenu; not shown in a design with only the root. The
/// destinations are found only while the submenu is open.
fn move_menu(
    ui: &mut egui::Ui,
    cx: &mut AppCx<'_>,
    destinations: impl Fn(&CmdEnv<'_>) -> Vec<Destination>,
    apply: impl Fn(&mut AppCx<'_>, ComponentId),
) {
    if cx.env().design.components().nth(1).is_none() {
        return;
    }
    ui.menu_button(labels::MOVE_TO, |ui| {
        let list: Vec<(Destination, String)> = {
            let env = cx.env();
            destinations(&env)
                .into_iter()
                .map(|d| {
                    let name = actions::component_name(env.design, d.component);
                    (d, name)
                })
                .collect()
        };
        if list.is_empty() {
            ui.weak("No other component");
        }
        for (d, name) in list {
            let mut r = ui.add_enabled(d.refused.is_none(), egui::Button::new(name));
            if let Some(why) = &d.refused {
                r = r.on_disabled_hover_text(why);
            }
            if r.clicked() {
                apply(cx, d.component);
                ui.close();
            }
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn framework_keys_parse_with_and_without_an_occurrence_path() {
        let p = |s: &str| parse(&NodeKey::owned(s.to_owned()));
        assert_eq!(p("root"), Some(Row::Root));
        assert_eq!(p("occ/7@3.4"), Some(Row::Occurrence(OccurrenceId(7))));
        assert_eq!(p("bodies/2@3"), Some(Row::Bodies(ComponentId(2))));
        assert_eq!(
            p("body/12:3@4"),
            Some(Row::Body(BodyId {
                origin: FeatureId(12),
                slot: 3
            }))
        );
        assert_eq!(p("origin"), None);
        assert_eq!(p("set/occ/1"), None);
    }
}
