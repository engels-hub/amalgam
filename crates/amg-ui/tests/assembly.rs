//! ASM-10: the assembly UI (`amg_ui::assembly`) in the real app (`AppHarness`, kittest, CPU
//! scene backend): activate radio, row menus and their commands, grounded and pending markers,
//! and the "positions out of date" badge.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    missing_docs,
    reason = "test code"
)]

use std::sync::Arc;

use amg_core::{Point3, Transform, Vec3};
use amg_doc::positions::CapturePositionFeature;
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, Command, ComponentId, Dependency, Design,
    Evaluated, Feature, FeatureBox, FeatureError, FeatureId, FeatureOutput, Health, OccurrenceId,
    RecomputeCtx, UndoEntry, register_feature,
};
use amg_mesh::IndexedMesh;
use amg_ui::CommitBatch;
use amg_ui::assembly::{AssemblyLayer, cmd, labels};
use amg_ui::command::Enablement;
use amg_ui::marking::{ContextTarget, context_menu_commands};
use amg_ui::mesh::MeshOp;
use amg_ui::pick::PickTarget;
use amg_ui::scene::CpuSceneBackend;
use amg_ui::select::SelItem;
use amg_ui::testing::AppHarness;
use amg_ui::testing::egui_kittest::kittest::{NodeT, Queryable};
use egui::accesskit::{Role, Toggled};
use serde::{Deserialize, Serialize};

// ---- fixtures ---------------------------------------------------------------------------------

/// A 10 mm tetrahedron mesh body, `serial` tells two of them apart.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Tet {
    serial: u32,
}

impl Feature for Tet {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let x = 20.0 * f64::from(self.serial);
        let positions = vec![
            Point3::new(x, 0.0, 0.0),
            Point3::new(x + 10.0, 0.0, 0.0),
            Point3::new(x, 10.0, 0.0),
            Point3::new(x, 0.0, 10.0),
        ];
        let tris = vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]];
        let mesh = IndexedMesh::new(positions, tris).unwrap();
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

register_feature!(Tet, "uitest.asm10_tet", version = 1);

fn harness() -> AppHarness {
    amg_features::mesh::link();
    let mut h = AppHarness::new();
    h.drive_until_idle();
    h
}

fn commit(h: &mut AppHarness, command: Command) {
    h.app_mut().submit(CommitBatch {
        label: "Fixture".into(),
        commands: vec![command],
    });
    h.drive_until_idle();
}

fn design(h: &AppHarness) -> &Design {
    h.document().design()
}

fn root(h: &AppHarness) -> ComponentId {
    design(h).root()
}

/// Adds a component named `name` in `parent`; returns it and its occurrence.
fn component(h: &mut AppHarness, parent: ComponentId, name: &str) -> (ComponentId, OccurrenceId) {
    commit(
        h,
        Command::NewComponent {
            parent,
            name: name.into(),
            transform: Transform::IDENTITY,
        },
    );
    design(h)
        .occurrences()
        .find(|(_, o)| {
            design(h)
                .component(o.component)
                .is_some_and(|c| c.name == name)
        })
        .map(|(id, o)| (o.component, id))
        .expect("the new component")
}

fn tet(h: &mut AppHarness, context: ComponentId, serial: u32) -> BodyId {
    commit(
        h,
        Command::AddFeature {
            at: None,
            feature: FeatureBox::new(Tet { serial }),
            context,
            name: None,
        },
    );
    let ev: &Evaluated = h.document().evaluated();
    ev.bodies().map(|(id, _)| id).max().expect("a body")
}

fn undo_depth(h: &AppHarness) -> usize {
    h.document().history().stack().applied()
}

fn last_entry(h: &AppHarness) -> &UndoEntry {
    let stack = h.document().history().stack();
    stack
        .entries()
        .nth(stack.applied() - 1)
        .expect("an undo entry")
}

fn undo(h: &mut AppHarness) {
    let id = h.app().active_document().id;
    h.app_mut().request_undo(id);
    h.drive_until_idle();
}

fn select(h: &mut AppHarness, targets: &[PickTarget]) {
    let items: Vec<SelItem> = targets
        .iter()
        .map(|t| match *t {
            PickTarget::Occurrence(o) => SelItem {
                target: *t,
                entity: amg_doc::EntityRef::InContext {
                    path: vec![o],
                    target: Box::new(amg_doc::EntityRef::Body(any_body(h))),
                },
            },
            _ => SelItem::from_target(*t).unwrap(),
        })
        .collect();
    let sel = &mut h.app_mut().active_document_mut().selection;
    sel.set(items);
    h.drive_until_idle();
}

fn any_body(h: &AppHarness) -> BodyId {
    h.document().evaluated().bodies().next().expect("a body").0
}

fn body_target(body: BodyId) -> PickTarget {
    PickTarget::Body {
        body,
        instance: 0,
        sub: None,
    }
}

// ---- kittest helpers --------------------------------------------------------------------------

fn count(h: &mut AppHarness, role: Role, label: &str) -> usize {
    h.kittest().query_all_by_role_and_label(role, label).count()
}

fn has_label(h: &mut AppHarness, label: &str) -> bool {
    h.kittest().query_all_by_label(label).next().is_some()
}

/// Right-clicks the first browser row labelled `row` (the document tab shows the root's name
/// too, as a button).
fn open_menu(h: &mut AppHarness, row: &str) {
    h.kittest()
        .query_all_by_label(row)
        .find(|n| n.accesskit_node().role() == Role::Label)
        .unwrap_or_else(|| panic!("no row {row}"))
        .click_secondary();
    h.drive_until_idle();
}

/// Closes the open menu with a click in the empty corner of the status bar. (Escape also
/// clears the selection and leaves the menu open.)
fn close_menu(h: &mut AppHarness) {
    h.click_at(
        egui::pos2(1275.0, 795.0),
        egui::PointerButton::Primary,
        egui::Modifiers::NONE,
    );
    h.drive_until_idle();
    assert_eq!(count(h, Role::Button, labels::NEW_COMPONENT), 0);
    assert_eq!(count(h, Role::Button, labels::FROM_BODIES), 0);
}

fn press(h: &mut AppHarness, label: &str) {
    h.kittest()
        .get_by_role_and_label(Role::Button, label)
        .click();
    h.drive_until_idle();
}

/// Right-clicks `row` and clicks `entry` in its menu.
fn menu(h: &mut AppHarness, row: &str, entry: &str) {
    open_menu(h, row);
    press(h, entry);
}

/// The "Move to Component" submenu button (egui appends an arrow to its name).
fn move_buttons(h: &mut AppHarness) -> usize {
    h.kittest()
        .query_all_by_role(Role::Button)
        .filter(|n| {
            n.accesskit_node()
                .label()
                .is_some_and(|l| l.starts_with(labels::MOVE_TO))
        })
        .count()
}

fn open_move_menu(h: &mut AppHarness) {
    h.kittest()
        .query_all_by_role(Role::Button)
        .find(|n| {
            n.accesskit_node()
                .label()
                .is_some_and(|l| l.starts_with(labels::MOVE_TO))
        })
        .expect("the Move to Component submenu")
        .click();
    h.drive_until_idle();
}

fn radio_selected(h: &mut AppHarness, row: &str) -> bool {
    let label = format!("{} {row}", labels::ACTIVATE);
    h.kittest()
        .get_by_role_and_label(Role::RadioButton, &label)
        .accesskit_node()
        .toggled()
        == Some(Toggled::True)
}

fn click_radio(h: &mut AppHarness, row: &str) {
    let label = format!("{} {row}", labels::ACTIVATE);
    h.kittest()
        .get_by_role_and_label(Role::RadioButton, &label)
        .click();
    h.drive_until_idle();
}

// ---- Done-when 1: the active component is the context of new features --------------------------

#[test]
fn a_feature_run_with_a_sub_component_active_has_that_context() {
    let mut h = harness();
    let root_name = design(&h).meta().name.clone();
    // A design without occurrences offers no radio buttons.
    assert_eq!(h.kittest().query_all_by_role(Role::RadioButton).count(), 0);
    let r = root(&h);
    let (bracket, _) = component(&mut h, r, "Bracket");
    let body = tet(&mut h, bracket, 0);
    assert!(radio_selected(&mut h, &root_name));
    assert!(!radio_selected(&mut h, "Bracket"));

    let depth = undo_depth(&h);
    click_radio(&mut h, "Bracket");
    assert_eq!(h.document().active_component(), bracket);
    assert!(radio_selected(&mut h, "Bracket"));
    assert!(!radio_selected(&mut h, &root_name));
    assert_eq!(undo_depth(&h), depth, "activating is not an edit");

    // The Mesh Repair dialog (the hand-built stand-in of the generic dialog, MESH-23).
    select(&mut h, &[body_target(body)]);
    h.run_command(MeshOp::Repair.cmd());
    press(&mut h, "OK");
    assert_eq!(undo_depth(&h), depth + 1);
    let (_, node) = design(&h)
        .timeline()
        .nodes()
        .enumerate()
        .last()
        .expect("the repair node");
    assert_eq!(node.feature.kind(), "mesh.repair");
    assert_eq!(node.context, bracket);

    // A body of another component stays where it is; the feature still gets the context.
    let other = tet(&mut h, r, 1);
    select(&mut h, &[body_target(other)]);
    h.run_command(MeshOp::Repair.cmd());
    press(&mut h, "OK");
    let node = design(&h).timeline().nodes().last().expect("a node");
    assert_eq!(
        (node.feature.kind(), node.context),
        ("mesh.repair", bracket)
    );
    let health = &h.document().evaluated().node(node.id).unwrap().health;
    assert!(
        matches!(health, Health::Ok | Health::Warning(_)),
        "{health:?}"
    );
    assert_eq!(h.document().evaluated().body(other).unwrap().component, r);

    // Back to the root through the row menu.
    menu(&mut h, &root_name, labels::ACTIVATE);
    assert_eq!(h.document().active_component(), r);
    assert!(radio_selected(&mut h, &root_name));
}

// ---- Done-when 2: copy / paste ----------------------------------------------------------------

fn backend(h: &AppHarness) -> &CpuSceneBackend {
    h.scene()
        .backend()
        .as_any()
        .downcast_ref::<CpuSceneBackend>()
        .expect("default backend")
}

#[test]
fn copy_and_paste_of_an_occurrence_shares_one_uploaded_mesh() {
    let mut h = harness();
    let r = root(&h);
    let root_name = design(&h).meta().name.clone();
    tet(&mut h, r, 0);
    assert_eq!(backend(&h).uploads(), 1);
    // Create Components from Bodies, from the Bodies folder.
    let depth = undo_depth(&h);
    menu(&mut h, "Bodies", labels::FROM_BODIES);
    assert_eq!(undo_depth(&h), depth + 1, "one undo entry");
    let (occ, comp) = design(&h)
        .occurrences()
        .map(|(id, o)| (id, o.component))
        .next()
        .expect("an occurrence");
    let name = design(&h).component(comp).unwrap().name.clone();
    assert_eq!(h.document().evaluated().bodies().count(), 1);
    assert!(
        h.document()
            .evaluated()
            .bodies()
            .all(|(_, b)| b.component == comp)
    );

    // Nothing to paste before a copy.
    open_menu(&mut h, &root_name);
    assert_eq!(count(&mut h, Role::Button, labels::PASTE), 0);
    close_menu(&mut h);
    menu(&mut h, &name, labels::COPY);
    // The component cannot be pasted into itself.
    open_menu(&mut h, &name);
    assert_eq!(count(&mut h, Role::Button, labels::PASTE), 0);
    close_menu(&mut h);
    let depth = undo_depth(&h);
    menu(&mut h, &root_name, labels::PASTE);
    assert_eq!(undo_depth(&h), depth + 1);
    assert!(matches!(
        last_entry(&h).commands.as_slice(),
        [Command::AddOccurrence { parent, component, .. }]
            if *parent == r && *component == comp
    ));
    assert_eq!(design(&h).occurrences().count(), 2);
    assert!(design(&h).occurrence(occ).is_some());
    assert_eq!(backend(&h).instance_count(), 2, "two render instances");
    assert_eq!(backend(&h).mesh_count(), 1, "one mesh");
    assert_eq!(backend(&h).uploads(), 1, "uploaded once");
    // The scene follows undo and redo of the paste.
    undo(&mut h);
    assert_eq!(backend(&h).instance_count(), 1);
    let id = h.app().active_document().id;
    h.app_mut().request_redo(id);
    h.drive_until_idle();
    assert_eq!(backend(&h).instance_count(), 2);
    assert_eq!(backend(&h).uploads(), 1);

    // The commands do the same for the selection (paste goes into the active component).
    select(&mut h, &[PickTarget::Occurrence(occ)]);
    h.run_command(cmd::COPY);
    h.run_command(cmd::PASTE);
    assert_eq!(design(&h).occurrences().count(), 3);
    assert_eq!(backend(&h).instance_count(), 3);
    assert_eq!(backend(&h).uploads(), 1);
}

// ---- Done-when 3: ground ----------------------------------------------------------------------

#[test]
fn ground_sends_set_occurrence_grounded_and_undo_restores_it() {
    let mut h = harness();
    let r = root(&h);
    let (_, occ) = component(&mut h, r, "Bracket");
    let grounded = |h: &AppHarness| design(h).occurrence(occ).unwrap().grounded;
    let marker = format!("{} Bracket", labels::GROUNDED);
    assert!(!grounded(&h) && !has_label(&mut h, &marker));

    let depth = undo_depth(&h);
    menu(&mut h, "Bracket", labels::GROUND);
    assert_eq!(undo_depth(&h), depth + 1);
    assert!(matches!(
        last_entry(&h).commands.as_slice(),
        [Command::SetOccurrenceGrounded { occ: o, grounded: true }] if *o == occ
    ));
    assert!(grounded(&h) && has_label(&mut h, &marker));

    menu(&mut h, "Bracket", labels::UNGROUND);
    assert!(matches!(
        last_entry(&h).commands.as_slice(),
        [Command::SetOccurrenceGrounded {
            grounded: false,
            ..
        }]
    ));
    assert!(!grounded(&h) && !has_label(&mut h, &marker));
    undo(&mut h);
    assert!(grounded(&h) && has_label(&mut h, &marker));
    undo(&mut h);
    assert!(!grounded(&h) && !has_label(&mut h, &marker));
}

// ---- Done-when 4: delete cascade --------------------------------------------------------------

#[test]
fn deleting_a_component_with_features_is_one_undo_entry() {
    let mut h = harness();
    let r = root(&h);
    tet(&mut h, r, 0);
    let (bracket, occ) = component(&mut h, r, "Bracket");
    tet(&mut h, bracket, 1);
    tet(&mut h, bracket, 2);
    let (pin, _) = component(&mut h, bracket, "Pin");
    tet(&mut h, pin, 3);
    // Deleting the active component falls back to the root.
    click_radio(&mut h, "Bracket");
    let before = h.document().snapshot();
    assert_eq!(before.timeline().len(), 4);

    assert_eq!(backend(&h).instance_count(), 4);
    let depth = undo_depth(&h);
    menu(&mut h, "Bracket", labels::DELETE);
    assert_eq!(undo_depth(&h), depth + 1, "one undo entry");
    assert!(matches!(
        last_entry(&h).commands.as_slice(),
        [Command::DeleteOccurrences { occs }] if occs == &[occ]
    ));
    assert!(design(&h).component(bracket).is_none() && design(&h).component(pin).is_none());
    assert_eq!(design(&h).timeline().len(), 1);
    assert_eq!(h.document().evaluated().bodies().count(), 1);
    assert_eq!(h.document().active_component(), r);
    assert_eq!(backend(&h).instance_count(), 1);

    undo(&mut h);
    assert_eq!(backend(&h).instance_count(), 4);
    let json = |d: &Design| serde_json::to_value(d).unwrap();
    assert_eq!(json(design(&h)), json(&before), "undo restores the design");
    assert_eq!(h.document().evaluated().bodies().count(), 4);
}

// ---- Done-when 5: menu entries per selection --------------------------------------------------

const ENTRIES: [&str; 8] = [
    labels::NEW_COMPONENT,
    labels::ACTIVATE,
    labels::GROUND,
    labels::UNGROUND,
    labels::COPY,
    labels::PASTE,
    labels::DELETE,
    labels::FROM_BODIES,
];

/// Checks that the menu of `row` has exactly `expected` of the assembly entries.
fn assert_menu(h: &mut AppHarness, row: &str, expected: &[&str], move_to: bool) {
    open_menu(h, row);
    for entry in ENTRIES {
        let want = usize::from(expected.contains(&entry));
        assert_eq!(count(h, Role::Button, entry), want, "{row}: {entry}");
    }
    assert_eq!(move_buttons(h), usize::from(move_to), "{row}: Move to");
    close_menu(h);
}

#[test]
fn every_menu_entry_is_reachable_and_only_shown_for_the_right_rows() {
    let mut h = harness();
    let r = root(&h);
    let root_name = design(&h).meta().name.clone();
    tet(&mut h, r, 0);
    press(&mut h, "Expand Bodies");
    // Only the root: no Move to Component, no Activate (it is active).
    assert_menu(&mut h, &root_name, &[labels::NEW_COMPONENT], false);
    assert_menu(&mut h, "Body1", &[labels::FROM_BODIES], false);
    assert_menu(&mut h, "Bodies", &[labels::FROM_BODIES], false);
    assert_menu(&mut h, "Origin", &[], false);
    assert_menu(&mut h, "Named Views", &[], false);

    // New Component from the root row.
    menu(&mut h, &root_name, labels::NEW_COMPONENT);
    assert!(has_label(&mut h, "Component1"));
    let (bracket, occ) = component(&mut h, r, "Bracket");
    let occurrence = [
        labels::NEW_COMPONENT,
        labels::ACTIVATE,
        labels::GROUND,
        labels::COPY,
        labels::DELETE,
    ];
    assert_menu(&mut h, "Bracket", &occurrence, true);
    assert_menu(&mut h, "Body1", &[labels::FROM_BODIES], true);
    assert_menu(&mut h, "Bodies", &[labels::FROM_BODIES], false);
    assert_menu(&mut h, &root_name, &[labels::NEW_COMPONENT], false);
    assert_menu(&mut h, "Origin", &[], false);

    // New Component under an occurrence, then Activate from its menu.
    menu(&mut h, "Bracket", labels::NEW_COMPONENT);
    assert!(
        design(&h)
            .component(bracket)
            .is_some_and(|c| c.children.len() == 1)
    );
    menu(&mut h, "Bracket", labels::ACTIVATE);
    assert_eq!(h.document().active_component(), bracket);
    assert_menu(
        &mut h,
        "Bracket",
        &[
            labels::NEW_COMPONENT,
            labels::GROUND,
            labels::COPY,
            labels::DELETE,
        ],
        true,
    );
    assert_menu(
        &mut h,
        &root_name,
        &[labels::NEW_COMPONENT, labels::ACTIVATE],
        false,
    );

    // Grounded: Unground instead of Ground. After a copy: Paste where it fits.
    menu(&mut h, "Bracket", labels::GROUND);
    menu(&mut h, "Component1", labels::COPY);
    assert_menu(
        &mut h,
        "Bracket",
        &[
            labels::NEW_COMPONENT,
            labels::UNGROUND,
            labels::COPY,
            labels::PASTE,
            labels::DELETE,
        ],
        true,
    );
    assert_menu(
        &mut h,
        &root_name,
        &[labels::NEW_COMPONENT, labels::ACTIVATE, labels::PASTE],
        false,
    );

    // A menu on a selected row acts on the whole selection.
    let other = design(&h)
        .occurrences()
        .map(|(id, _)| id)
        .find(|id| *id != occ && design(&h).occurrence(*id).unwrap().parent == r)
        .expect("Component1");
    let both = [PickTarget::Occurrence(occ), PickTarget::Occurrence(other)];
    select(&mut h, &both);
    assert_menu(
        &mut h,
        "Bracket",
        &[
            labels::NEW_COMPONENT,
            labels::GROUND,
            labels::UNGROUND,
            labels::COPY,
            labels::PASTE,
            labels::DELETE,
        ],
        true,
    );
    menu(&mut h, "Bracket", labels::GROUND);
    assert!(
        design(&h)
            .occurrences()
            .filter(|(_, o)| o.parent == r)
            .all(|(_, o)| o.grounded)
    );
}

#[test]
fn move_to_component_moves_occurrences_and_bodies() {
    let mut h = harness();
    let r = root(&h);
    let body = tet(&mut h, r, 0);
    let (bracket, _) = component(&mut h, r, "Bracket");
    let (_, pin) = component(&mut h, r, "Pin");

    // Occurrence: Pin under Bracket. Neither its parent nor itself is offered.
    open_menu(&mut h, "Pin");
    open_move_menu(&mut h);
    assert_eq!(count(&mut h, Role::Button, "Pin"), 0);
    let depth = undo_depth(&h);
    press(&mut h, "Bracket");
    assert_eq!(undo_depth(&h), depth + 1);
    assert_eq!(design(&h).occurrence(pin).unwrap().parent, bracket);

    // Body: from the root into Bracket.
    press(&mut h, "Expand Bodies");
    open_menu(&mut h, "Body1");
    open_move_menu(&mut h);
    press(&mut h, "Bracket");
    assert_eq!(
        h.document().evaluated().body(body).unwrap().component,
        bracket
    );
    assert_eq!(undo_depth(&h), depth + 2);
}

#[test]
fn commands_are_enabled_for_the_right_selection() {
    let mut h = harness();
    let r = root(&h);
    let body = tet(&mut h, r, 0);
    let (bracket, occ) = component(&mut h, r, "Bracket");
    let enabled = |h: &mut AppHarness| -> Vec<_> {
        h.app_mut().with_cx(|cx| {
            let env = cx.env();
            cmd::ALL
                .into_iter()
                .filter(|id| {
                    cx.registry().command(*id).unwrap().enablement(&env) == Enablement::Enabled
                })
                .collect()
        })
    };
    assert_eq!(enabled(&mut h), [cmd::NEW_COMPONENT]);
    select(&mut h, &[body_target(body)]);
    assert_eq!(enabled(&mut h), [cmd::NEW_COMPONENT, cmd::FROM_BODIES]);
    select(&mut h, &[PickTarget::Occurrence(occ)]);
    assert_eq!(
        enabled(&mut h),
        [
            cmd::NEW_COMPONENT,
            cmd::ACTIVATE,
            cmd::GROUND,
            cmd::COPY,
            cmd::DELETE
        ]
    );
    // The marking menu's overflow list offers them for the viewport.
    let offered = h.app_mut().with_cx(|cx| {
        context_menu_commands(cx.registry(), &cx.env(), &ContextTarget::Viewport(None))
    });
    for id in [cmd::ACTIVATE, cmd::GROUND, cmd::COPY, cmd::DELETE] {
        assert!(offered.contains(&id), "{id}");
    }
    assert!(!offered.contains(&cmd::UNGROUND));

    h.run_command(cmd::ACTIVATE);
    assert_eq!(h.document().active_component(), bracket);
    h.run_command(cmd::GROUND);
    h.run_command(cmd::COPY);
    // Paste goes into the active component, which is the copied one: disabled.
    assert!(!enabled(&mut h).contains(&cmd::PASTE));
    assert!(enabled(&mut h).contains(&cmd::UNGROUND));
    h.run_command(cmd::UNGROUND);
    assert!(!design(&h).occurrence(occ).unwrap().grounded);
    h.run_command(cmd::DELETE);
    assert!(design(&h).occurrence(occ).is_none());
}

// ---- Done-when 6: pending marker and badge ----------------------------------------------------

/// Stands in for the joint solver's out-of-date occurrences (none exist before ASM-04):
/// reports the occurrences with a pending move.
fn pending_are_out_of_date(design: &Design, _eval: &Evaluated) -> Vec<OccurrenceId> {
    design.pending_positions().moves().map(|(o, _)| o).collect()
}

fn capture_nodes(h: &AppHarness) -> Vec<FeatureId> {
    design(h)
        .timeline()
        .nodes()
        .filter(|n| n.feature.downcast_ref::<CapturePositionFeature>().is_some())
        .map(|n| n.id)
        .collect()
}

fn move_to(h: &mut AppHarness, occ: OccurrenceId, x: f64) {
    commit(
        h,
        Command::SetPendingPositions {
            moves: vec![(occ, Transform::from_translation(Vec3::new(x, 0.0, 0.0)))],
            joint_states: Vec::new(),
        },
    );
}

#[test]
fn a_pending_move_shows_its_marker_and_the_badge_captures_once() {
    let mut h = harness();
    let r = root(&h);
    let (_, occ) = component(&mut h, r, "Bracket");
    let (_, _) = component(&mut h, r, "Pin");
    let marker = format!("{} Bracket", labels::PENDING);
    let still = format!("{} Pin", labels::PENDING);
    assert!(!has_label(&mut h, &marker));

    move_to(&mut h, occ, 12.5);
    assert!(has_label(&mut h, &marker));
    assert!(!has_label(&mut h, &still), "only the moved occurrence");
    // `joints::out_of_date` is empty (solved placements equal the seeds): no badge.
    assert!(amg_doc::joints::out_of_date(design(&h), h.document().evaluated()).is_empty());
    assert!(!has_label(&mut h, labels::OUT_OF_DATE));

    h.app_mut().with_cx(|cx| {
        cx.doc_layer::<AssemblyLayer>()
            .expect("the module's layer")
            .set_out_of_date_source(pending_are_out_of_date);
    });
    h.drive_until_idle();
    assert_eq!(
        count(&mut h, Role::Button, labels::OUT_OF_DATE),
        1,
        "the badge"
    );

    let depth = undo_depth(&h);
    press(&mut h, labels::OUT_OF_DATE);
    assert_eq!(undo_depth(&h), depth + 1, "one undo entry");
    assert!(matches!(
        last_entry(&h).commands.as_slice(),
        [Command::CapturePositions { moves, .. }]
            if moves.len() == 1 && moves[0].0 == occ && (moves[0].1.translation.x - 12.5).abs() < 1e-12
    ));
    assert_eq!(capture_nodes(&h).len(), 1);
    assert!(!design(&h).has_pending_positions());
    assert!(!has_label(&mut h, &marker));
    assert!(!has_label(&mut h, labels::OUT_OF_DATE));

    // Undo restores the pending move, its marker and the badge.
    undo(&mut h);
    assert!(has_label(&mut h, &marker));
    assert!(has_label(&mut h, labels::OUT_OF_DATE));
    assert!(capture_nodes(&h).is_empty());
    // The command does the same as the badge.
    h.run_command(cmd::CAPTURE);
    assert_eq!(capture_nodes(&h).len(), 1);
}

#[test]
fn bodies_can_be_turned_into_components_from_the_selection() {
    let mut h = harness();
    let r = root(&h);
    let a = tet(&mut h, r, 0);
    let b = tet(&mut h, r, 1);
    let c = tet(&mut h, r, 2);
    select(&mut h, &[body_target(a), body_target(b)]);
    press(&mut h, "Expand Bodies");
    let depth = undo_depth(&h);
    menu(&mut h, "Body1", labels::FROM_BODIES);
    assert_eq!(undo_depth(&h), depth + 1);
    assert_eq!(design(&h).occurrences().count(), 2, "the two selected");
    assert_eq!(h.document().evaluated().body(c).unwrap().component, r);
    let created = last_entry(&h)
        .commands
        .iter()
        .filter(|c| matches!(c, Command::NewComponent { .. }))
        .count();
    assert_eq!(created, 2);
    let info = h
        .app()
        .registry()
        .feature_ui(amg_features::assembly::BODIES_TO_COMPONENTS_KIND);
    assert!(
        info.is_some(),
        "the timeline has a name and icon for the node"
    );
}

#[test]
fn row_items_do_not_cover_the_arrow_eye_or_label() {
    let mut h = harness();
    let r = root(&h);
    let long = "A component with a name far too long for the browser panel to show";
    let (_, short) = component(&mut h, r, "Pin");
    component(&mut h, r, long);
    move_to(&mut h, short, 1.0);
    commit(
        &mut h,
        Command::SetOccurrenceGrounded {
            occ: short,
            grounded: true,
        },
    );
    for name in ["Pin", long] {
        let mut rects = vec![
            h.kittest().get_by_label(&format!("Hide {name}")).rect(),
            h.kittest()
                .query_all_by_label(name)
                .find(|n| n.accesskit_node().role() == Role::Label)
                .unwrap()
                .rect(),
        ];
        let items: &[&str] = if name == "Pin" {
            &[labels::GROUNDED, labels::PENDING, labels::ACTIVATE]
        } else {
            &[labels::ACTIVATE]
        };
        for item in items {
            rects.push(h.kittest().get_by_label(&format!("{item} {name}")).rect());
        }
        for pair in rects.windows(2) {
            assert!(pair[0].right() <= pair[1].left() + 0.5, "{name}: {rects:?}");
        }
        let panel = h.camera_view().rect.left();
        assert!(rects.last().unwrap().right() <= panel, "{name}: {rects:?}");
    }
}
