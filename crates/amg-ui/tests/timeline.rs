//! UI-08 tests: the timeline strip on the real app in `AppHarness` (kittest).
//!
//! Done-when 1: on a 20-node timeline, dragging the marker to index 5 sends `SetMarker{5}`
//! and nodes ≥ 5 render as rolled back. Done-when 2: dragging a node before its input is
//! refused (no command, red shading) and a valid move sends one `MoveFeature`. Done-when 3:
//! an erroring feature shows an error badge with the diagnostic message as tooltip.
//! Done-when 4: suppress / rename / delete / group via the context menu are one undo entry
//! each. Done-when 5 (timing) is `benches/timeline.rs`. Also: hover highlights the outputs,
//! groups collapse, the timeline is hidden in Direct mode, `FeatureUiInfo` drives labels.

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::sync::Arc;

use amg_core::Point3;
use amg_doc::stub::IndexedMesh;
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, Command, Dependency, DesignMode, EntityRef,
    FeatureBox, FeatureError, FeatureId, FeatureOutput, RecomputeCtx,
};
use amg_ui::CommitBatch;
use amg_ui::module::{IconId, Registrar, UiModule};
use amg_ui::scene::HighlightLayer;
use amg_ui::testing::AppHarness;
use amg_ui::testing::egui_kittest::kittest::{NodeT, Queryable};
use amg_ui::timeline::{EditAction, FeatureUiInfo, NodeHealth, entity_targets};
use egui::accesskit::Role;
use serde::{Deserialize, Serialize};

// ---- test features ---------------------------------------------------------------------------

/// One flat 2 × 2 mm square at `x0`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Block {
    x0: f64,
}

impl amg_doc::Feature for Block {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let x = self.x0;
        let mesh = IndexedMesh::new(
            vec![
                Point3::new(x, 0.0, 0.0),
                Point3::new(x + 2.0, 0.0, 0.0),
                Point3::new(x + 2.0, 2.0, 0.0),
                Point3::new(x, 2.0, 0.0),
            ],
            vec![[0, 1, 2], [0, 2, 3]],
        )
        .expect("valid square");
        Ok(FeatureOutput {
            bodies: vec![BodyChange {
                id: ctx.new_body_id(0),
                change: BodyChangeKind::Created {
                    geom: BodyGeom::Mesh(Arc::new(mesh)),
                    naming: None,
                },
            }],
            ..FeatureOutput::default()
        })
    }
}

amg_doc::register_feature!(Block, "uitest.tl_block", version = 1);

/// Reads an earlier feature; outputs nothing.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Chain {
    input: FeatureId,
}

impl amg_doc::Feature for Chain {
    fn inputs(&self) -> Vec<Dependency> {
        vec![Dependency::Feature(self.input)]
    }

    fn compute(&self, _ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Ok(FeatureOutput::default())
    }
}

amg_doc::register_feature!(Chain, "uitest.tl_chain", version = 1);

/// Always fails.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Broken;

impl amg_doc::Feature for Broken {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, _ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Err(FeatureError::InvalidValue {
            field: "depth",
            msg: "the depth must be positive".into(),
        })
    }
}

amg_doc::register_feature!(Broken, "uitest.tl_broken", version = 1);

/// No `FeatureUiInfo`: generic icon, label from the kind.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Anonymous;

impl amg_doc::Feature for Anonymous {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, _ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Ok(FeatureOutput::default())
    }
}

amg_doc::register_feature!(Anonymous, "uitest.tl_anonymous", version = 1);

inventory::submit! { UiModule { id: "zz_timeline_test", register: register_test_module } }

fn register_test_module(r: &mut Registrar<'_>) {
    for (kind, display_name) in [
        ("uitest.tl_block", "Block"),
        ("uitest.tl_chain", "Chain"),
        ("uitest.tl_broken", "Broken"),
    ] {
        r.feature_ui(FeatureUiInfo {
            kind,
            display_name,
            icon: IconId("feature.test"),
            edit: if kind == "uitest.tl_broken" {
                EditAction::NotEditable
            } else {
                EditAction::GenericDialog
            },
        });
    }
}

// ---- helpers -------------------------------------------------------------------------------

fn commit(h: &mut AppHarness, cmd: Command) {
    h.app_mut().submit(CommitBatch {
        label: "Setup".into(),
        commands: vec![cmd],
    });
    h.drive_until_idle();
}

fn add(h: &mut AppHarness, feature: FeatureBox) -> FeatureId {
    let root = h.document().design().root();
    commit(
        h,
        Command::AddFeature {
            at: None,
            feature,
            context: root,
            name: None,
        },
    );
    h.document()
        .design()
        .timeline()
        .nodes()
        .last()
        .expect("added")
        .id
}

fn order(h: &AppHarness) -> Vec<FeatureId> {
    h.document()
        .design()
        .timeline()
        .nodes()
        .map(|n| n.id)
        .collect()
}

fn depth(h: &AppHarness) -> usize {
    h.document().history().stack().applied()
}

fn rect(h: &AppHarness, id: FeatureId) -> egui::Rect {
    h.app()
        .timeline()
        .drawn_node(id)
        .unwrap_or_else(|| panic!("{id:?} is drawn"))
        .rect
}

/// The x of the slot boundary before node `id` (between it and its left neighbour).
fn boundary_before(h: &AppHarness, id: FeatureId) -> f32 {
    rect(h, id).left() - 2.0
}

fn click_button(h: &mut AppHarness, label: &str) {
    h.kittest()
        .get_by_role_and_label(Role::Button, label)
        .click();
    h.drive_until_idle();
}

fn context_menu(h: &mut AppHarness, node: &str, entry: &str) {
    h.kittest()
        .get_by_role_and_label(Role::Button, node)
        .click_secondary();
    h.drive_until_idle();
    click_button(h, entry);
}

fn has_button(h: &mut AppHarness, label: &str) -> bool {
    h.kittest()
        .query_all_by_role_and_label(Role::Button, label)
        .next()
        .is_some()
}

fn blocks(h: &mut AppHarness, n: usize) -> Vec<FeatureId> {
    (0..n)
        .map(|i| add(h, FeatureBox::new(Block { x0: 3.0 * i as f64 })))
        .collect()
}

fn harness() -> AppHarness {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    h
}

/// Presses at `from`, moves to `to` in steps, then checks `mid` before releasing.
fn drag_with(h: &mut AppHarness, from: egui::Pos2, to: egui::Pos2, mid: impl FnOnce(&AppHarness)) {
    let none = egui::Modifiers::NONE;
    h.hover_at(from);
    h.button(from, egui::PointerButton::Primary, true, none);
    for i in 1..=6 {
        let t = i as f32 / 6.0;
        h.hover_at(from + (to - from) * t);
    }
    h.hover_at(to);
    mid(h);
    h.button(to, egui::PointerButton::Primary, false, none);
    h.drive_until_idle();
}

// ---- Done-when 1 -------------------------------------------------------------------------------

#[test]
fn dragging_the_marker_to_index_5_rolls_back_the_rest() {
    let mut h = harness();
    let ids = blocks(&mut h, 20);
    assert_eq!(h.document().design().timeline().marker(), 20);
    let labels = h.app().timeline().labels();
    assert_eq!(labels.len(), 20);
    assert_eq!(labels[0], "Block1", "default names from FeatureUiInfo");
    assert_eq!(labels[19], "Block20");
    assert_eq!(h.app().timeline().drawn().len(), 20, "all 20 nodes in view");
    assert!(h.app().timeline().drawn().iter().all(|d| !d.rolled_back));

    let marker = h.app().timeline().marker_rect().expect("marker drawn");
    let to = egui::pos2(boundary_before(&h, ids[5]), marker.center().y);
    let before = depth(&h);
    drag_with(&mut h, marker.center(), to, |h| {
        // While dragging the nodes after the new position preview as rolled back, and no
        // command was sent yet.
        assert_eq!(h.app().timeline().marker_drag(), Some(5));
        for d in h.app().timeline().drawn() {
            assert_eq!(d.rolled_back, d.index >= 5, "preview of node {}", d.index);
        }
        assert_eq!(h.document().design().timeline().marker(), 20);
    });
    assert_eq!(depth(&h), before + 1, "one undo entry");
    assert_eq!(
        h.document().design().timeline().marker(),
        5,
        "SetMarker{{5}}"
    );
    assert_eq!(
        h.document().history().undo_label(),
        Some("Move marker"),
        "the entry is the marker move"
    );
    for d in h.app().timeline().drawn() {
        assert_eq!(d.rolled_back, d.index >= 5, "node {}", d.index);
    }
    // The evaluated state agrees: nodes ≥ 5 are rolled back.
    let eval = h.document().evaluated();
    assert!(matches!(
        eval.node(ids[5]).unwrap().health,
        amg_doc::Health::RolledBack
    ));
    assert!(matches!(
        eval.node(ids[4]).unwrap().health,
        amg_doc::Health::Ok
    ));

    // The step buttons move it by one; "Go to end" rolls everything forward.
    click_button(&mut h, "⏵");
    assert_eq!(h.document().design().timeline().marker(), 6);
    click_button(&mut h, "⏭");
    assert_eq!(h.document().design().timeline().marker(), 20);
    // "Roll to Here" puts the marker after the node.
    context_menu(&mut h, "Block3", "Roll to Here");
    assert_eq!(h.document().design().timeline().marker(), 3);
}

// ---- Done-when 2 -------------------------------------------------------------------------------

#[test]
fn moving_a_node_before_its_input_is_refused_and_a_valid_move_sends_one_command() {
    let mut h = harness();
    let a = add(&mut h, FeatureBox::new(Block { x0: 0.0 }));
    let chain = add(&mut h, FeatureBox::new(Chain { input: a }));
    let b = add(&mut h, FeatureBox::new(Block { x0: 3.0 }));
    let c = add(&mut h, FeatureBox::new(Block { x0: 6.0 }));
    assert_eq!(order(&h), vec![a, chain, b, c]);

    // Before its input A: refused.
    let from = rect(&h, chain).center();
    let to = egui::pos2(boundary_before(&h, a) + 1.0, from.y);
    let before = depth(&h);
    drag_with(&mut h, from, to, |h| {
        let tl = h.app().timeline();
        let drag = tl.drag_view().expect("dragging");
        assert_eq!(drag.id, chain);
        assert_eq!(drag.to, Some(0));
        assert!(!drag.valid, "index 0 is before the input");
        let shaded: Vec<usize> = tl
            .drawn()
            .iter()
            .filter(|d| d.invalid_drop)
            .map(|d| d.index)
            .collect();
        assert_eq!(shaded, vec![0], "the position before A is shaded red");
    });
    assert_eq!(depth(&h), before, "no command was sent");
    assert_eq!(order(&h), vec![a, chain, b, c]);
    assert!(h.app().timeline().drag_view().is_none(), "the drag ended");

    // To the end: valid, one MoveFeature.
    let from = rect(&h, chain).center();
    let to = egui::pos2(rect(&h, c).right() + 2.0, from.y);
    drag_with(&mut h, from, to, |h| {
        let drag = h.app().timeline().drag_view().expect("dragging");
        assert_eq!(drag.to, Some(3));
        assert!(drag.valid);
        // The invalid range stays shaded during the whole drag; the drop target is not in it.
        let shaded: Vec<usize> = h
            .app()
            .timeline()
            .drawn()
            .iter()
            .filter(|d| d.invalid_drop)
            .map(|d| d.index)
            .collect();
        assert_eq!(shaded, vec![0]);
    });
    assert_eq!(depth(&h), before + 1, "one undo entry");
    assert_eq!(h.document().history().undo_label(), Some("Move feature"));
    assert_eq!(order(&h), vec![a, b, c, chain]);
}

// ---- Done-when 3 -------------------------------------------------------------------------------

#[test]
fn an_erroring_feature_shows_an_error_badge_with_the_message_as_tooltip() {
    let mut h = harness();
    let ok = add(&mut h, FeatureBox::new(Block { x0: 0.0 }));
    let bad = add(&mut h, FeatureBox::new(Broken));
    let tl = h.app().timeline();
    let NodeHealth::Error(msg, _) = tl.health(bad).expect("health").clone() else {
        panic!("Broken is an error");
    };
    assert!(msg.contains("the depth must be positive"), "{msg}");
    let d = tl.drawn_node(bad).unwrap();
    assert_eq!(d.badge, Some(amg_core::Severity::Error));
    assert!(d.error_tint, "error nodes are tinted red");
    let d = tl.drawn_node(ok).unwrap();
    assert_eq!(d.badge, None);
    assert!(!d.error_tint);

    h.kittest()
        .ctx
        .global_style_mut(|s| s.interaction.tooltip_delay = 0.0);
    h.kittest().get_by_label("Error").hover();
    for _ in 0..4 {
        h.kittest().step();
    }
    // The badge's own tooltip is exactly the message (the node's tooltip, also shown because the
    // badge sits on the node, is "<label>, newline, <message>").
    assert!(
        h.kittest().query_by_label(&msg).is_some(),
        "the badge's tooltip is the diagnostic message"
    );
    // A non-editable kind disables "Edit Feature".
    h.kittest()
        .get_by_role_and_label(Role::Button, "Broken1")
        .click_secondary();
    h.drive_until_idle();
    let edit = h
        .kittest()
        .get_by_role_and_label(Role::Button, "Edit Feature");
    assert!(edit.accesskit_node().is_disabled());
}

// ---- Done-when 4 -------------------------------------------------------------------------------

#[test]
fn context_menu_edits_are_one_undo_entry_each() {
    let mut h = harness();
    let ids = blocks(&mut h, 4);

    // Suppress / Unsuppress.
    let d0 = depth(&h);
    context_menu(&mut h, "Block2", "Suppress");
    assert_eq!(depth(&h), d0 + 1);
    let node = |h: &AppHarness, id| {
        h.document()
            .design()
            .timeline()
            .node(id)
            .map(|(_, n)| n.clone())
            .unwrap()
    };
    assert!(node(&h, ids[1]).suppressed);
    assert!(h.app().timeline().drawn_node(ids[1]).unwrap().suppressed);
    context_menu(&mut h, "Block2", "Unsuppress");
    assert_eq!(depth(&h), d0 + 2);
    assert!(!node(&h, ids[1]).suppressed);

    // Rename. A focused text field blinks its cursor (a repaint request every frame), so this
    // steps a fixed number of frames instead of waiting for idle.
    h.kittest()
        .get_by_role_and_label(Role::Button, "Block3")
        .click_secondary();
    h.drive_until_idle();
    h.kittest()
        .get_by_role_and_label(Role::Button, "Rename")
        .click();
    h.kittest().run_steps(3);
    let field = h.kittest().get_by_role(Role::TextInput);
    assert!(field.is_focused(), "the rename field takes focus");
    field.type_text("Base");
    h.kittest().run_steps(2);
    h.kittest().key_press(egui::Key::Enter);
    h.drive_until_idle();
    assert_eq!(depth(&h), d0 + 3);
    assert_eq!(node(&h, ids[2]).name.as_deref(), Some("Base"));
    assert!(has_button(&mut h, "Base"));

    // Group two selected nodes (click, Ctrl+click), then collapse, expand and ungroup.
    h.kittest()
        .get_by_role_and_label(Role::Button, "Block1")
        .click();
    h.drive_until_idle();
    h.kittest()
        .get_by_role_and_label(Role::Button, "Block2")
        .click_modifiers(egui::Modifiers::COMMAND);
    h.drive_until_idle();
    assert_eq!(h.app().timeline().selected(), &[ids[0], ids[1]]);
    context_menu(&mut h, "Block2", "Group");
    assert_eq!(depth(&h), d0 + 4);
    let g = node(&h, ids[0]).group.expect("grouped");
    assert_eq!(node(&h, ids[1]).group, Some(g));
    assert_eq!(node(&h, ids[2]).group, None);
    click_button(&mut h, "Collapse Group1");
    assert!(h.document().design().timeline().group(g).unwrap().collapsed);
    assert!(
        !has_button(&mut h, "Block1"),
        "collapsed nodes are not drawn"
    );
    click_button(&mut h, "Expand Group1");
    assert!(!h.document().design().timeline().group(g).unwrap().collapsed);
    let d1 = depth(&h);
    context_menu(&mut h, "Block1", "Ungroup");
    assert_eq!(depth(&h), d1 + 1);
    assert_eq!(node(&h, ids[0]).group, None);

    // Delete.
    let d2 = depth(&h);
    context_menu(&mut h, "Block4", "Delete");
    assert_eq!(depth(&h), d2 + 1);
    assert!(!h.document().design().timeline().contains(ids[3]));
    assert!(!has_button(&mut h, "Block4"));
}

// ---- hover, Direct mode, unknown kinds ---------------------------------------------------------

#[test]
fn hovering_a_node_highlights_its_output_bodies() {
    let mut h = harness();
    let ids = blocks(&mut h, 3);
    let chain = add(&mut h, FeatureBox::new(Chain { input: ids[0] }));
    assert!(h.scene().highlight(HighlightLayer::Hover).is_empty());
    h.kittest()
        .get_by_role_and_label(Role::Button, "Block2")
        .hover();
    h.drive_until_idle();
    let body = BodyId {
        origin: ids[1],
        slot: 0,
    };
    let keys = h.scene().pick_map().body_keys(body);
    let hl: Vec<_> = h
        .scene()
        .highlight(HighlightLayer::Hover)
        .iter()
        .map(|i| i.key)
        .collect();
    assert_eq!(hl.len(), 1);
    assert!(keys.contains(&hl[0]), "the hover layer holds Block2's body");
    // A node without outputs highlights nothing.
    h.kittest()
        .get_by_role_and_label(Role::Button, "Chain1")
        .hover();
    h.drive_until_idle();
    assert_eq!(h.app().timeline().hovered(), Some(chain));
    assert!(h.scene().highlight(HighlightLayer::Hover).is_empty());
    // Leaving the timeline clears it.
    h.hover_at(egui::pos2(700.0, 400.0));
    h.drive_until_idle();
    assert_eq!(h.app().timeline().hovered(), None);

    // Error entities resolve to pick targets (the Error layer of a hovered error node).
    let doc = h.document();
    let targets = entity_targets(
        doc.design(),
        doc.evaluated(),
        h.scene(),
        &[EntityRef::Body(body)],
    );
    assert_eq!(targets.len(), 1);
    assert_eq!(targets[0].body(), Some(body));
}

#[test]
fn unknown_kinds_get_a_generic_node_and_direct_mode_hides_the_timeline() {
    let mut h = harness();
    assert!(
        h.kittest()
            .query_all_by_label("Timeline is empty")
            .next()
            .is_some()
    );
    add(&mut h, FeatureBox::new(Block { x0: 0.0 }));
    add(&mut h, FeatureBox::new(Anonymous));
    assert_eq!(h.app().timeline().labels(), vec!["Block1", "tl_anonymous1"]);
    assert!(has_button(&mut h, "tl_anonymous1"));
    assert!(has_button(&mut h, "Go to start") || has_button(&mut h, "⏮"));

    let r = h
        .app_mut()
        .active_document_mut()
        .document
        .apply(Command::SetDesignMode {
            mode: DesignMode::Direct,
        });
    assert!(r.is_ok(), "{r:?}");
    h.drive_until_idle();
    assert!(!has_button(&mut h, "Block1"), "hidden in Direct mode");
    assert!(!has_button(&mut h, "⏮"));
}
