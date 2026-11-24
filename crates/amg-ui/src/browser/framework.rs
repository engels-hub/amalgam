//! The framework's browser nodes (spec §11) and their row actions.
//!
//! Node keys (an occurrence path suffix `@o1.o2` keeps nodes seen through several
//! occurrences of one component unique):
//! - `root`; `settings`, `settings/units`; `views`, `views/{home,top,front,right}`;
//!   `origin`, `origin/{o,x,y,z,xy,xz,yz}`; `sets`, `set/<name>`;
//! - `bodies/<component>`, `body/<feature>:<slot>`; `sketches/<component>`,
//!   `sketch/<feature>`; `constructions/<component>`, `construction/<feature>`;
//!   `occ/<occurrence>`.
//!
//! Actions:
//! - **Eye:** bodies → `SetBodyProps { visible }`, occurrences → `SetOccurrenceVisible`,
//!   sketches / construction / origin → the [`ui.visibility`](super::visibility) record. A
//!   folder's eye shows or hides everything in it (one undo entry).
//! - **Rename:** bodies → `SetBodyProps.name`, occurrences and the root → `RenameComponent`
//!   (S01 R3), sketches / construction → `RenameFeature`, selection sets → the
//!   `ui.selection_sets` record.
//! - **Double-click:** sketches, construction and bodies → edit the creating feature
//!   ([`AppCx::edit_feature`]); selection sets → restore; named views → orient the camera.

use std::collections::BTreeMap;

use amg_core::{Plane, Severity};
use amg_doc::{
    BodyId, BodyProps, Command, ComponentId, ConstructionGeom, ConstructionId, DocEvent, EntityRef,
    FeatureId, Health, OccurrenceId, SketchId,
};

use super::visibility::{self, ORIGIN_KEY, VISIBILITY_KIND};
use super::{BrowserAnchor, BrowserNode, BrowserProvider, Builder, NodeKey};
use crate::app::{AppCx, CmdEnv, CommitBatch};
use crate::nav::CameraControl;
use crate::pick::PickTarget;
use crate::select::SelItem;
use crate::select::sets::{SELECTION_SETS_KIND, rename_set_command, selection_sets};

/// Key of the root node.
pub const ROOT_KEY: NodeKey = NodeKey::new("root");

/// The framework provider (index 0 of the provider list).
pub static FRAMEWORK_PROVIDER: FrameworkNodes = FrameworkNodes;

/// The framework's nodes: settings, named views, origin, the component tree, selection sets.
#[derive(Debug)]
pub struct FrameworkNodes;

/// Per-component contents, collected once per rebuild.
#[derive(Default)]
struct Contents {
    bodies: BTreeMap<ComponentId, Vec<BodyId>>,
    sketches: BTreeMap<ComponentId, Vec<SketchId>>,
    constructions: BTreeMap<ComponentId, Vec<ConstructionId>>,
    /// Default body names ("Body<n>", by body order).
    body_number: BTreeMap<BodyId, usize>,
    hidden: std::collections::BTreeSet<String>,
}

fn badge(health: Option<&Health>) -> Option<Severity> {
    match health? {
        Health::Error(_) => Some(Severity::Error),
        Health::Warning(_) => Some(Severity::Warning),
        Health::Ok | Health::Suppressed | Health::RolledBack => None,
    }
}

fn suffix(path: &[OccurrenceId]) -> String {
    if path.is_empty() {
        return String::new();
    }
    let parts: Vec<String> = path.iter().map(|o| o.0.to_string()).collect();
    format!("@{}", parts.join("."))
}

fn feature_name(env: &CmdEnv<'_>, f: FeatureId) -> Option<String> {
    env.design
        .timeline()
        .node(f)
        .and_then(|(_, n)| n.name.clone())
}

/// Builds the whole tree: the framework nodes and every provider's nodes at its anchors.
pub(super) fn build_tree(env: &CmdEnv<'_>, ps: &[&dyn BrowserProvider], b: &mut Builder<'_>) {
    let design = env.design;
    let eval = env.evaluated;
    let mut c = Contents {
        hidden: visibility::hidden(design),
        ..Contents::default()
    };
    for (n, (id, body)) in eval.bodies().enumerate() {
        c.bodies.entry(body.component).or_default().push(id);
        c.body_number.insert(id, n + 1);
    }
    for (id, _) in eval.sketches() {
        if let Some((_, node)) = design.timeline().node(id.0) {
            c.sketches.entry(node.context).or_default().push(id);
        }
    }
    for (id, _) in eval.constructions() {
        if let Some((_, node)) = design.timeline().node(id.0) {
            c.constructions.entry(node.context).or_default().push(id);
        }
    }

    let root_id = design.root();
    let root = b.open(
        BrowserNode::new(ROOT_KEY, design.meta().name.clone())
            .icon(crate::icons::ids::COMPONENT)
            .renamable(),
        0,
        0,
    );
    // Document Settings.
    let s = b.open(
        BrowserNode::new(NodeKey::new("settings"), "Document Settings")
            .icon(crate::icons::ids::SETTINGS),
        0,
        1,
    );
    let len: amg_core::units::Unit = design.meta().default_units.length.into();
    let unit = format!("Units: {}", len.symbol());
    b.push_tree(
        BrowserNode::new(NodeKey::new("settings/units"), unit).icon(crate::icons::ids::RULER),
        0,
        2,
    );
    b.close(s);
    // Named Views.
    let v = b.open(
        BrowserNode::new(NodeKey::new("views"), "Named Views").icon(crate::icons::ids::CAMERA),
        0,
        1,
    );
    for (key, label) in NAMED_VIEWS {
        b.push_tree(
            BrowserNode::new(NodeKey::new(key), *label).icon(crate::icons::ids::CAMERA),
            0,
            2,
        );
    }
    b.close(v);
    // Origin.
    let o = b.open(
        BrowserNode::new(NodeKey::new("origin"), "Origin")
            .icon(crate::icons::ids::ORIGIN)
            .visible(!c.hidden.contains(ORIGIN_KEY)),
        0,
        1,
    );
    for (key, label) in ORIGIN_ITEMS {
        b.push_tree(BrowserNode::new(NodeKey::new(key), *label), 0, 2);
    }
    b.close(o);
    // The root component's contents.
    component(env, ps, b, &c, root_id, &[], 1);
    // Selection Sets.
    let sets = selection_sets(design);
    if !sets.is_empty() {
        let s = b.open(
            BrowserNode::new(NodeKey::new("sets"), "Selection Sets")
                .icon(crate::icons::ids::BOOKMARKS),
            0,
            1,
        );
        for set in sets {
            let key = NodeKey::owned(format!("set/{}", set.name));
            b.push_tree(
                BrowserNode::new(key, set.name)
                    .icon(crate::icons::ids::BOOKMARK)
                    .renamable(),
                0,
                2,
            );
        }
        b.close(s);
    }
    // Providers at the root.
    for (pi, p) in ps.iter().enumerate().skip(1) {
        let mut out = Vec::new();
        p.build(env, &BrowserAnchor::Root, &mut out);
        for n in out {
            b.push_tree(n, pi, 1);
        }
    }
    b.close(root);
}

const NAMED_VIEWS: &[(&str, &str)] = &[
    ("views/home", "Home"),
    ("views/top", "Top"),
    ("views/front", "Front"),
    ("views/right", "Right"),
];

const ORIGIN_ITEMS: &[(&str, &str)] = &[
    ("origin/o", "O"),
    ("origin/xy", "XY"),
    ("origin/xz", "XZ"),
    ("origin/yz", "YZ"),
    ("origin/x", "X"),
    ("origin/y", "Y"),
    ("origin/z", "Z"),
];

/// The contents of `comp` seen through `path`, at `depth`.
fn component(
    env: &CmdEnv<'_>,
    ps: &[&dyn BrowserProvider],
    b: &mut Builder<'_>,
    c: &Contents,
    comp: ComponentId,
    path: &[OccurrenceId],
    depth: u16,
) {
    let design = env.design;
    let eval = env.evaluated;
    let sfx = suffix(path);
    // Bodies.
    if let Some(bodies) = c.bodies.get(&comp) {
        let any = bodies.iter().any(|id| design.body_props(*id).visible);
        let f = b.open(
            BrowserNode::new(NodeKey::owned(format!("bodies/{}{sfx}", comp.0)), "Bodies")
                .icon(crate::icons::ids::FOLDER)
                .visible(any),
            0,
            depth,
        );
        for id in bodies {
            let props = design.body_props(*id);
            let label = props
                .name
                .clone()
                .unwrap_or_else(|| format!("Body{}", c.body_number.get(id).copied().unwrap_or(0)));
            let node = BrowserNode::new(
                NodeKey::owned(format!("body/{}:{}{sfx}", id.origin.0, id.slot)),
                label,
            )
            .icon(crate::icons::ids::BODY)
            .visible(props.visible)
            .renamable()
            .badge(badge(eval.node(id.origin).map(|n| &n.health)))
            .target(Some(SelItem {
                target: PickTarget::Body {
                    body: *id,
                    instance: 0,
                    sub: None,
                },
                entity: EntityRef::Body(*id),
            }));
            b.push_tree(node, 0, depth + 1);
        }
        b.close(f);
    }
    // Sketches.
    if let Some(sketches) = c.sketches.get(&comp) {
        let any = sketches
            .iter()
            .any(|s| !c.hidden.contains(&visibility::sketch_key(*s)));
        let f = b.open(
            BrowserNode::new(
                NodeKey::owned(format!("sketches/{}{sfx}", comp.0)),
                "Sketches",
            )
            .icon(crate::icons::ids::FOLDER)
            .visible(any),
            0,
            depth,
        );
        for (n, s) in sketches.iter().enumerate() {
            let label = feature_name(env, s.0).unwrap_or_else(|| format!("Sketch{}", n + 1));
            let node = BrowserNode::new(NodeKey::owned(format!("sketch/{}{sfx}", s.0.0)), label)
                .icon(crate::icons::ids::SKETCH)
                .visible(!c.hidden.contains(&visibility::sketch_key(*s)))
                .renamable()
                .badge(badge(eval.node(s.0).map(|n| &n.health)));
            b.push_tree(node, 0, depth + 1);
        }
        b.close(f);
    }
    // Construction.
    if let Some(cons) = c.constructions.get(&comp) {
        let any = cons
            .iter()
            .any(|k| !c.hidden.contains(&visibility::construction_key(*k)));
        let f = b.open(
            BrowserNode::new(
                NodeKey::owned(format!("constructions/{}{sfx}", comp.0)),
                "Construction",
            )
            .icon(crate::icons::ids::FOLDER)
            .visible(any),
            0,
            depth,
        );
        let mut counts = [0_usize; 3];
        for k in cons {
            let (slot, kind) = match eval.construction(*k) {
                Some(ConstructionGeom::Plane(_)) | None => (0, "Plane"),
                Some(ConstructionGeom::Axis(_)) => (1, "Axis"),
                Some(ConstructionGeom::Point(_)) => (2, "Point"),
            };
            counts[slot] += 1;
            let label = feature_name(env, k.0).unwrap_or_else(|| format!("{kind}{}", counts[slot]));
            let node = BrowserNode::new(
                NodeKey::owned(format!("construction/{}{sfx}", k.0.0)),
                label,
            )
            .icon(crate::icons::ids::ORIGIN)
            .visible(!c.hidden.contains(&visibility::construction_key(*k)))
            .renamable()
            .badge(badge(eval.node(k.0).map(|n| &n.health)))
            .target(SelItem::from_target(PickTarget::Construction(*k)));
            b.push_tree(node, 0, depth + 1);
        }
        b.close(f);
    }
    // Providers inside this component.
    for (pi, p) in ps.iter().enumerate().skip(1) {
        let mut out = Vec::new();
        p.build(env, &BrowserAnchor::Component(comp), &mut out);
        for n in out {
            b.push_tree(n, pi, depth);
        }
    }
    // Child occurrences.
    let Some(def) = design.component(comp) else {
        return;
    };
    for occ_id in &def.children {
        let Some(occ) = design.occurrence(*occ_id) else {
            continue;
        };
        let name = occ.name.clone().unwrap_or_else(|| {
            design
                .component(occ.component)
                .map(|k| k.name.clone())
                .unwrap_or_default()
        });
        let first_body = c.bodies.get(&occ.component).and_then(|v| v.first());
        let target = first_body.map(|b| SelItem {
            target: PickTarget::Occurrence(*occ_id),
            entity: EntityRef::InContext {
                path: vec![*occ_id],
                target: Box::new(EntityRef::Body(*b)),
            },
        });
        let mut sub = path.to_vec();
        sub.push(*occ_id);
        let start = b.len();
        let o = b.open(
            BrowserNode::new(NodeKey::owned(format!("occ/{}{sfx}", occ_id.0)), name)
                .icon(crate::icons::ids::COMPONENT)
                .visible(occ.visible)
                .renamable()
                .target(target),
            0,
            depth,
        );
        debug_assert_eq!(start, o);
        // A component cannot contain itself (validated by S01), so this terminates.
        component(env, ps, b, c, occ.component, &sub, depth + 1);
        b.close(o);
    }
}

// ---- key parsing ----------------------------------------------------------------------------

/// `"body/12:0@3.4"` → (`"body"`, `"12:0"`).
fn split(key: &NodeKey) -> (&str, &str) {
    // Set names may contain `@`; sets only exist at the root, so they carry no path.
    if let Some(name) = key.as_str().strip_prefix("set/") {
        return ("set", name);
    }
    let base = key.as_str().split('@').next().unwrap_or_default();
    base.split_once('/').unwrap_or((base, ""))
}

fn parse_body(rest: &str) -> Option<BodyId> {
    let (f, s) = rest.split_once(':')?;
    Some(BodyId {
        origin: FeatureId(f.parse().ok()?),
        slot: s.parse().ok()?,
    })
}

fn parse_u64(rest: &str) -> Option<u64> {
    rest.parse().ok()
}

/// Bodies of component `comp`.
fn bodies_of(env: &CmdEnv<'_>, comp: ComponentId) -> Vec<BodyId> {
    env.evaluated
        .bodies()
        .filter(|(_, b)| b.component == comp)
        .map(|(id, _)| id)
        .collect()
}

fn features_in(env: &CmdEnv<'_>, comp: ComponentId, sketches: bool) -> Vec<String> {
    let in_comp = |f: FeatureId| {
        env.design
            .timeline()
            .node(f)
            .is_some_and(|(_, n)| n.context == comp)
    };
    if sketches {
        env.evaluated
            .sketches()
            .filter(|(s, _)| in_comp(s.0))
            .map(|(s, _)| visibility::sketch_key(s))
            .collect()
    } else {
        env.evaluated
            .constructions()
            .filter(|(k, _)| in_comp(k.0))
            .map(|(k, _)| visibility::construction_key(k))
            .collect()
    }
}

fn body_visibility(env: &CmdEnv<'_>, bodies: &[BodyId], visible: bool) -> Vec<Command> {
    bodies
        .iter()
        .filter_map(|b| {
            let props = env.design.body_props(*b);
            (props.visible != visible).then(|| Command::SetBodyProps {
                body: *b,
                props: BodyProps { visible, ..props },
            })
        })
        .collect()
}

fn submit(cx: &mut AppCx<'_>, label: &str, commands: Vec<Command>) {
    if !commands.is_empty() {
        cx.submit(CommitBatch {
            label: label.to_owned(),
            commands,
        });
    }
}

impl BrowserProvider for FrameworkNodes {
    fn id(&self) -> &'static str {
        "app"
    }

    fn affected_by(&self, ev: &DocEvent) -> bool {
        match ev {
            DocEvent::BodyAdded(_)
            | DocEvent::BodyRemoved(_)
            | DocEvent::BodyPropsChanged(_)
            | DocEvent::ComponentTreeChanged
            | DocEvent::OccurrenceVisibilityChanged(_)
            | DocEvent::SketchChanged(_)
            | DocEvent::ConstructionChanged(_)
            | DocEvent::TimelineChanged
            | DocEvent::HealthChanged(_)
            | DocEvent::DocumentReplaced => true,
            DocEvent::ExtChanged(k) => k.kind == SELECTION_SETS_KIND || k.kind == VISIBILITY_KIND,
            _ => false,
        }
    }

    fn build(&self, _env: &CmdEnv<'_>, _anchor: &BrowserAnchor, _out: &mut Vec<BrowserNode>) {
        // The framework's nodes are built by `build_tree` directly.
    }

    fn set_visible(&self, node: &NodeKey, visible: bool, cx: &mut AppCx<'_>) {
        let label = if visible { "Show" } else { "Hide" };
        let (kind, rest) = split(node);
        let commands = {
            let env = cx.env();
            match kind {
                "body" => parse_body(rest)
                    .map(|b| body_visibility(&env, &[b], visible))
                    .unwrap_or_default(),
                "bodies" => parse_u64(rest)
                    .map(|c| body_visibility(&env, &bodies_of(&env, ComponentId(c)), visible))
                    .unwrap_or_default(),
                "occ" => parse_u64(rest)
                    .map(|o| {
                        vec![Command::SetOccurrenceVisible {
                            occ: OccurrenceId(o),
                            visible,
                        }]
                    })
                    .unwrap_or_default(),
                "sketch" | "construction" => {
                    let item = format!("{kind}/{rest}");
                    visibility::set_hidden_command(env.design, &[item], !visible)
                        .into_iter()
                        .collect()
                }
                "origin" => {
                    visibility::set_hidden_command(env.design, &[ORIGIN_KEY.to_owned()], !visible)
                        .into_iter()
                        .collect()
                }
                "sketches" | "constructions" => parse_u64(rest)
                    .and_then(|c| {
                        let items = features_in(&env, ComponentId(c), kind == "sketches");
                        visibility::set_hidden_command(env.design, &items, !visible)
                    })
                    .into_iter()
                    .collect(),
                _ => Vec::new(),
            }
        };
        submit(cx, label, commands);
    }

    fn rename(&self, node: &NodeKey, name: &str, cx: &mut AppCx<'_>) {
        let (kind, rest) = split(node);
        let command = {
            let env = cx.env();
            match kind {
                "root" => Some(Command::RenameComponent {
                    id: env.design.root(),
                    name: name.to_owned(),
                }),
                "body" => parse_body(rest).map(|b| Command::SetBodyProps {
                    body: b,
                    props: BodyProps {
                        name: Some(name.to_owned()),
                        ..env.design.body_props(b)
                    },
                }),
                "occ" => parse_u64(rest)
                    .and_then(|o| env.design.occurrence(OccurrenceId(o)))
                    .map(|occ| Command::RenameComponent {
                        id: occ.component,
                        name: name.to_owned(),
                    }),
                "sketch" | "construction" => parse_u64(rest).map(|f| Command::RenameFeature {
                    id: FeatureId(f),
                    name: Some(name.to_owned()),
                }),
                "set" => match rename_set_command(env.design, rest, name) {
                    Some(Ok(cmd)) => Some(cmd),
                    Some(Err(e)) => {
                        tracing::warn!("cannot rename selection set `{rest}`: {e}");
                        None
                    }
                    None => None,
                },
                _ => None,
            }
        };
        if let Some(cmd) = command {
            submit(cx, "Rename", vec![cmd]);
        }
    }

    fn double_click(&self, node: &NodeKey, cx: &mut AppCx<'_>) {
        let (kind, rest) = split(node);
        match kind {
            "sketch" | "construction" => {
                if let Some(f) = parse_u64(rest) {
                    cx.edit_feature(FeatureId(f));
                }
            }
            "body" => {
                if let Some(b) = parse_body(rest)
                    && cx.env().design.timeline().contains(b.origin)
                {
                    cx.edit_feature(b.origin);
                }
            }
            "set" => cx.restore_selection_set(rest),
            "views" => named_view(rest, cx),
            _ => {}
        }
    }

    fn context_menu(&self, node: &NodeKey, ui: &mut egui::Ui, cx: &mut AppCx<'_>) {
        let (kind, rest) = split(node);
        match kind {
            "set" => {
                if ui.button("Restore selection").clicked() {
                    cx.restore_selection_set(rest);
                    ui.close();
                }
                if ui.button("Delete selection set").clicked() {
                    let cmd = crate::select::sets::remove_set_command(cx.env().design, rest);
                    if let Some(Ok(cmd)) = cmd {
                        submit(cx, "Delete selection set", vec![cmd]);
                    }
                    ui.close();
                }
            }
            "sketch" | "body" | "construction" | "views" => {
                let label = match kind {
                    "sketch" => "Edit Sketch",
                    "views" => "Go to view",
                    _ => "Edit Feature",
                };
                if ui.button(label).clicked() {
                    self.double_click(node, cx);
                    ui.close();
                }
            }
            _ => {}
        }
    }
}

/// Orients the camera to a named view (`home` fits the scene).
fn named_view(which: &str, cx: &mut AppCx<'_>) {
    let mut cam = cx.camera();
    match which {
        "top" => cam.look_at_plane(&Plane::XY, true),
        "front" => cam.look_at_plane(&Plane::XZ, true),
        "right" => cam.look_at_plane(&Plane::YZ, true),
        "home" => cam.fit(None),
        _ => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keys_parse_with_and_without_an_occurrence_path() {
        let k = NodeKey::owned("body/12:3@4.5".into());
        assert_eq!(split(&k), ("body", "12:3"));
        assert_eq!(
            parse_body("12:3"),
            Some(BodyId {
                origin: FeatureId(12),
                slot: 3
            })
        );
        assert_eq!(split(&NodeKey::new("origin")), ("origin", ""));
        assert_eq!(split(&NodeKey::new("set/A@b")), ("set", "A@b"));
        assert_eq!(suffix(&[OccurrenceId(4), OccurrenceId(5)]), "@4.5");
        assert_eq!(suffix(&[]), "");
    }
}
