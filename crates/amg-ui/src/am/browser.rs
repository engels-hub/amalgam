//! The setup browser (Print Setups → setup → plates → items) and the Plate panel.

use amg_doc::{DocEvent, ExtKey};
use amg_features::am::{PRINT_SETUP_KIND, PlateId, PrintSetup, delete_setup_command, setups};

use super::layer::AmLayer;
use super::{
    ADD_BODIES, CENTRE, DROP, EXPORT_3MF, MOVE_ROTATE, NEW_SETUP, PLACE_FACE_DOWN, SEND_TO_SLICER,
    SETTINGS, active_setup, dialogs::machine_summary, submit_setup,
};
use crate::app::{AppCx, CmdEnv, CommitBatch};
use crate::browser::{BrowserAnchor, BrowserNode, BrowserProvider, NodeKey};
use crate::panel::Panel;
use crate::pick::PickTarget;
use crate::select::SelItem;
use amg_core::Severity;

const ROOT: &str = "am.setups";

fn setup_key(k: &ExtKey) -> NodeKey {
    NodeKey::owned(format!("am.setup.{}", k.id))
}

fn parse_setup(node: &NodeKey) -> Option<ExtKey> {
    let id = node.as_str().strip_prefix("am.setup.")?.parse().ok()?;
    Some(ExtKey {
        kind: PRINT_SETUP_KIND.to_owned(),
        id,
    })
}

/// The browser provider of print setups.
#[derive(Debug)]
pub struct SetupBrowser;

impl BrowserProvider for SetupBrowser {
    fn id(&self) -> &'static str {
        "am"
    }

    fn affected_by(&self, ev: &DocEvent) -> bool {
        match ev {
            DocEvent::ExtChanged(k) => k.kind == PRINT_SETUP_KIND,
            DocEvent::BodyAdded(_)
            | DocEvent::BodyRemoved(_)
            | DocEvent::BodyPropsChanged(_)
            | DocEvent::DocumentReplaced => true,
            _ => false,
        }
    }

    fn build(&self, env: &CmdEnv<'_>, anchor: &BrowserAnchor, out: &mut Vec<BrowserNode>) {
        if !matches!(anchor, BrowserAnchor::Root) {
            return;
        }
        let all = setups(env.design);
        if all.is_empty() {
            return;
        }
        let mut root = BrowserNode::new(NodeKey::new(ROOT), "Print Setups");
        for (k, s) in all {
            let node = match s {
                Ok(s) => setup_node(env, &k, &s),
                Err(e) => {
                    let mut n =
                        BrowserNode::new(setup_key(&k), format!("Setup {} (unreadable)", k.id));
                    n.badge = Some(Severity::Error);
                    tracing::debug!("am: setup {}: {e}", k.id);
                    n
                }
            };
            root.children.push(node);
        }
        out.push(root);
    }

    fn rename(&self, node: &NodeKey, name: &str, cx: &mut AppCx<'_>) {
        let Some(key) = parse_setup(node) else { return };
        let Some(Ok(mut s)) = setups(cx.env().design)
            .into_iter()
            .find(|(k, _)| *k == key)
            .map(|(_, s)| s)
        else {
            return;
        };
        name.clone_into(&mut s.name);
        submit_setup(cx, "Rename Print Setup", key, &s);
    }

    fn double_click(&self, node: &NodeKey, cx: &mut AppCx<'_>) {
        if let Some(key) = parse_setup(node)
            && let Some(l) = cx.doc_layer::<AmLayer>()
        {
            l.set_active(Some(key));
        }
    }

    fn context_menu(&self, node: &NodeKey, ui: &mut egui::Ui, cx: &mut AppCx<'_>) {
        let Some(key) = parse_setup(node) else { return };
        if ui.button("Activate").clicked() {
            if let Some(l) = cx.doc_layer::<AmLayer>() {
                l.set_active(Some(key.clone()));
            }
            ui.close();
        }
        if ui.button("Delete Setup").clicked() {
            cx.submit(CommitBatch {
                label: "Delete Print Setup".to_owned(),
                commands: vec![delete_setup_command(key)],
            });
            ui.close();
        }
    }
}

fn setup_node(env: &CmdEnv<'_>, k: &ExtKey, s: &PrintSetup) -> BrowserNode {
    let mut n = BrowserNode::new(setup_key(k), format!("{} — {}", s.name, s.machine.name));
    n.renamable = true;
    for p in &s.plates {
        n.children.push(plate_node(env, k, s, p.id));
    }
    n
}

fn plate_node(env: &CmdEnv<'_>, k: &ExtKey, s: &PrintSetup, plate: PlateId) -> BrowserNode {
    let Some(p) = s.plates.iter().find(|p| p.id == plate) else {
        return BrowserNode::new(
            NodeKey::owned(format!("am.plate.{}.{}", k.id, plate.0)),
            "Plate",
        );
    };
    let active = if p.id == s.active_plate {
        " (active)"
    } else {
        ""
    };
    let mut n = BrowserNode::new(
        NodeKey::owned(format!("am.plate.{}.{}", k.id, p.id.0)),
        format!("{}{active}", p.name),
    );
    for it in &p.items {
        let props = env.design.body_props(it.body);
        let name = props
            .name
            .clone()
            .unwrap_or_else(|| format!("Body {}.{}", it.body.origin.0, it.body.slot));
        let label = match it.filament {
            Some(f) => format!("{name} [filament {f}]"),
            None => name,
        };
        let mut c = BrowserNode::new(
            NodeKey::owned(format!("am.item.{}.{}", k.id, it.id.0)),
            label,
        );
        c.target = SelItem::from_target(PickTarget::Body {
            body: it.body,
            instance: 0,
            sub: None,
        });
        let printable = env
            .evaluated
            .body(it.body)
            .is_some_and(|b| matches!(b.geom, amg_doc::BodyGeom::Mesh(_)));
        if !printable {
            c.badge = Some(Severity::Warning);
        }
        n.children.push(c);
    }
    n
}

// ---- Plate panel --------------------------------------------------------------------------

/// The Plate panel: the active setup and machine, the items with their fit, and the main
/// commands.
#[derive(Debug, Default)]
pub struct PlatePanel;

pub(crate) fn create_panel() -> Box<dyn Panel> {
    Box::new(PlatePanel)
}

impl Panel for PlatePanel {
    fn ui(&mut self, ui: &mut egui::Ui, cx: &mut AppCx<'_>) {
        let chosen = cx.doc_layer::<AmLayer>().and_then(|l| l.active.clone());
        let all = setups(cx.env().design);
        let current = active_setup(cx.env().design, chosen.as_ref());
        let Some((key, s)) = current else {
            ui.label("No print setup yet.");
            if ui.button("New Setup…").clicked() {
                cx.run_command(NEW_SETUP);
            }
            return;
        };
        let mut pick = key.clone();
        egui::ComboBox::from_label("Setup")
            .selected_text(s.name.clone())
            .show_ui(ui, |ui| {
                for (k, r) in &all {
                    if let Ok(o) = r {
                        ui.selectable_value(&mut pick, k.clone(), o.name.clone());
                    }
                }
            });
        if pick != key
            && let Some(l) = cx.doc_layer::<AmLayer>()
        {
            l.set_active(Some(pick));
        }
        ui.label(format!("Machine: {}", s.machine.name));
        ui.weak(machine_summary(&s.machine));
        ui.separator();
        let status = cx
            .doc_layer::<AmLayer>()
            .map(|l| l.status().to_vec())
            .unwrap_or_default();
        if status.is_empty() {
            ui.weak("The plate is empty: select bodies and Add Bodies.");
        }
        for st in &status {
            ui.horizontal(|ui| {
                let sel = ui.selectable_label(false, &st.name);
                if sel.clicked()
                    && let Some(item) = SelItem::from_target(PickTarget::Body {
                        body: st.body,
                        instance: 0,
                        sub: None,
                    })
                {
                    let s = cx.selection_mut();
                    s.clear();
                    s.add(item);
                }
                match (&st.message, st.pending) {
                    (Some(m), _) => {
                        ui.colored_label(ui.visuals().error_fg_color, m);
                    }
                    (None, true) => {
                        ui.spinner();
                    }
                    (None, false) => {
                        ui.weak("fits");
                    }
                }
            });
        }
        ui.separator();
        ui.horizontal_wrapped(|ui| {
            for (id, label) in [
                (ADD_BODIES, "Add Bodies"),
                (PLACE_FACE_DOWN, "Place on Plate"),
                (DROP, "Drop"),
                (MOVE_ROTATE, "Move/Rotate…"),
                (CENTRE, "Centre"),
            ] {
                if ui.button(label).clicked() {
                    cx.run_command(id);
                }
            }
        });
        ui.horizontal_wrapped(|ui| {
            if ui.button("Send to Slicer").clicked() {
                cx.run_command(SEND_TO_SLICER);
            }
            if ui.button("Export 3MF…").clicked() {
                cx.run_command(EXPORT_3MF);
            }
            if ui.button("Settings…").clicked() {
                cx.run_command(SETTINGS);
            }
        });
        if let Some(o) = cx
            .doc_layer::<AmLayer>()
            .and_then(|l| l.last_output().cloned())
        {
            ui.weak(format!("Last hand-off: {}", o.path.display()));
        }
    }
}
