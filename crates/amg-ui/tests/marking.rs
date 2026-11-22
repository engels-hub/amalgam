//! UI-21 tests: the right-click marking menu (click / gesture / hold), "Repeat", context
//! checks of providers, labels of every wedge, and registry context providers in the browser
//! and timeline menus.
//!
//! Everything runs the real app in `AppHarness` (kittest). The module below registers test
//! commands, a marking-menu provider that fills all eight wedges, a provider that adds an entry
//! only for a selected mesh body, and a context provider.

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};

use amg_core::Point3;
use amg_doc::stub::{IndexedMesh, Shape};
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, Command, Dependency, EntityRef, FeatureBox,
    FeatureError, FeatureOutput, RecomputeCtx,
};
use amg_ui::command::{CmdAction, CmdId, CommandDef};
use amg_ui::marking::{
    ContextMenuProvider, ContextTarget, HOLD, MarkingMenuBuilder, MarkingMenuProvider, Wedge,
};
use amg_ui::module::{Registrar, UiModule};
use amg_ui::pick::PickTarget;
use amg_ui::testing::AppHarness;
use amg_ui::testing::egui_kittest::kittest::Queryable;
use amg_ui::{CmdEnv, CommitBatch, SelItem};
use egui::accesskit::Role;
use serde::{Deserialize, Serialize};

// ---- test commands and providers -------------------------------------------------------------

static RUNS: [AtomicUsize; 10] = [const { AtomicUsize::new(0) }; 10];

/// The wedge commands, by `Wedge::index`; labels "Wedge N", "Wedge NE", ….
const WEDGE_CMDS: [CmdId; 8] = [
    CmdId("uitest.wedge_n"),
    CmdId("uitest.wedge_ne"),
    CmdId("uitest.wedge_e"),
    CmdId("uitest.wedge_se"),
    CmdId("uitest.wedge_s"),
    CmdId("uitest.wedge_sw"),
    CmdId("uitest.wedge_w"),
    CmdId("uitest.wedge_nw"),
];
const MESH_ONLY: CmdId = CmdId("uitest.mesh_only");
const MESH_ONLY_LABEL: &str = "Mesh Body Only";
const CONTEXT_CMD: CmdId = CmdId("uitest.context");
const CONTEXT_LABEL: &str = "Context Test";

fn runs(i: usize) -> usize {
    RUNS[i].load(Ordering::SeqCst)
}

macro_rules! counter_actions {
    ($($i:literal),*) => {
        [$(CmdAction::Run(|_| { RUNS[$i].fetch_add(1, Ordering::SeqCst); })),*]
    };
}

/// Fills every wedge (priority above the framework's 0).
struct AllWedges;

impl MarkingMenuProvider for AllWedges {
    fn priority(&self) -> i32 {
        100
    }

    fn fill(&self, _env: &CmdEnv<'_>, out: &mut MarkingMenuBuilder) {
        for w in Wedge::ALL {
            out.wedge(w, WEDGE_CMDS[w.index()]);
        }
    }
}

/// Adds "Mesh Body Only" to the overflow list while a whole mesh body is selected.
struct MeshOnly;

impl MarkingMenuProvider for MeshOnly {
    fn priority(&self) -> i32 {
        50
    }

    fn fill(&self, env: &CmdEnv<'_>, out: &mut MarkingMenuBuilder) {
        let mesh_selected = env.selection.items().iter().any(|i| match i.entity {
            EntityRef::Body(b) => env
                .evaluated
                .bodies()
                .any(|(id, body)| id == b && matches!(body.geom, BodyGeom::Mesh(_))),
            _ => false,
        });
        if mesh_selected {
            out.overflow(MESH_ONLY);
        }
    }
}

/// Adds "Context Test" to browser and timeline menus.
struct ContextTest;

impl ContextMenuProvider for ContextTest {
    fn fill(&self, _env: &CmdEnv<'_>, target: &ContextTarget<'_>, out: &mut Vec<CmdId>) {
        if matches!(
            target,
            ContextTarget::Browser(_) | ContextTarget::Timeline(_)
        ) {
            out.push(CONTEXT_CMD);
            out.push(CmdId("uitest.not_registered"));
        }
    }
}

fn register(r: &mut Registrar<'_>) {
    let actions = counter_actions!(0, 1, 2, 3, 4, 5, 6, 7, 8, 9);
    let mut actions = actions.into_iter();
    for w in Wedge::ALL {
        let label = format!("Wedge {}", w.name());
        r.command(CommandDef::new(
            WEDGE_CMDS[w.index()].0,
            label,
            actions.next().unwrap(),
        ));
    }
    r.command(CommandDef::new(
        MESH_ONLY.0,
        MESH_ONLY_LABEL,
        actions.next().unwrap(),
    ));
    r.command(CommandDef::new(
        CONTEXT_CMD.0,
        CONTEXT_LABEL,
        actions.next().unwrap(),
    ));
    r.marking_menu(Box::new(AllWedges));
    r.marking_menu(Box::new(MeshOnly));
    r.context_menu(Box::new(ContextTest));
}

inventory::submit! { UiModule { id: "uitest_marking", register } }

const MESH_ONLY_RUNS: usize = 8;
const CONTEXT_RUNS: usize = 9;

// ---- test features: one mesh body, one solid body ----------------------------------------------

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct MkBody {
    mesh: bool,
}

impl amg_doc::Feature for MkBody {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let geom = if self.mesh {
            let pos = vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(4.0, 0.0, 0.0),
                Point3::new(0.0, 4.0, 0.0),
            ];
            BodyGeom::Mesh(Arc::new(IndexedMesh::new(pos, vec![[0, 1, 2]]).unwrap()))
        } else {
            BodyGeom::Solid(Shape::new())
        };
        Ok(FeatureOutput {
            bodies: vec![BodyChange {
                id: ctx.new_body_id(0),
                change: BodyChangeKind::Created { geom, naming: None },
            }],
            ..FeatureOutput::default()
        })
    }
}

amg_doc::register_feature!(MkBody, "uitest.mk_body", version = 1);

// ---- helpers -----------------------------------------------------------------------------------

fn harness() -> AppHarness {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    h
}

fn center(h: &AppHarness) -> egui::Pos2 {
    h.camera_view().rect.center()
}

const RIGHT: egui::PointerButton = egui::PointerButton::Secondary;

/// Right-clicks at `at` (press and release in place) and draws the result.
fn right_click(h: &mut AppHarness, at: egui::Pos2) {
    h.click_at(at, RIGHT, egui::Modifiers::NONE);
    h.drive_until_idle();
}

fn label_count(h: &mut AppHarness, label: &str) -> usize {
    h.kittest().query_all_by_label(label).count()
}

fn add_bodies(h: &mut AppHarness) -> Vec<BodyId> {
    for mesh in [true, false] {
        let cmd = Command::AddFeature {
            at: None,
            feature: FeatureBox::new(MkBody { mesh }),
            context: h.document().design().root(),
            name: None,
        };
        h.app_mut().submit(CommitBatch {
            label: "Body".into(),
            commands: vec![cmd],
        });
        h.drive_until_idle();
    }
    let ev = h.document().evaluated().clone();
    let mut bodies: Vec<(bool, BodyId)> = ev
        .bodies()
        .map(|(id, b)| (matches!(b.geom, BodyGeom::Mesh(_)), id))
        .collect();
    // Mesh first.
    bodies.sort_by_key(|(m, _)| !*m);
    bodies.into_iter().map(|(_, id)| id).collect()
}

fn select_body(h: &mut AppHarness, body: Option<BodyId>) {
    let items = body
        .map(|b| SelItem {
            target: PickTarget::Body {
                body: b,
                instance: 0,
                sub: None,
            },
            entity: EntityRef::Body(b),
        })
        .into_iter()
        .collect();
    h.app_mut().active_document_mut().selection.set(items);
    h.drive_until_idle();
}

// ---- Done-when 1: click opens, N wedge runs; gesture to E runs E without the menu -------------

#[test]
fn right_click_opens_the_menu_and_clicking_n_runs_it_and_a_gesture_runs_e() {
    let mut h = harness();
    let c = center(&h);
    assert!(h.app().marking_menu().is_none());

    // Click mode: opens at the cursor and stays open.
    right_click(&mut h, c);
    let open = h
        .app()
        .marking_menu()
        .expect("a right click opens the menu")
        .clone();
    assert_eq!(open.center, c);
    assert!(!open.held);
    assert_eq!(open.menu.wedge(Wedge::N), Some(WEDGE_CMDS[0]));
    h.kittest().run_steps(2);
    assert!(h.app().marking_menu().is_some(), "the menu stays open");

    // Clicking the N wedge's button runs its command and closes the menu.
    let n0 = runs(Wedge::N.index());
    h.kittest()
        .get_by_role_and_label(Role::Button, "Wedge N")
        .click();
    h.drive_until_idle();
    assert_eq!(runs(Wedge::N.index()), n0 + 1, "N ran once");
    assert!(
        h.app().marking_menu().is_none(),
        "running a wedge closes it"
    );
    assert_eq!(label_count(&mut h, "Wedge N"), 0, "the wedges are gone");

    // Gesture mode: a right drag of 80 pt to the right runs E on release; the menu never
    // opens (checked after every step).
    let e0 = runs(Wedge::E.index());
    h.hover_at(c);
    h.button(c, RIGHT, true, egui::Modifiers::NONE);
    for i in 1..=8 {
        #[allow(clippy::cast_precision_loss, reason = "small")]
        let p = c + egui::vec2(10.0 * i as f32, -2.0);
        h.kittest().event(egui::Event::PointerMoved(p));
        h.kittest().step();
        assert!(h.app().marking_menu().is_none(), "gesture: no menu");
    }
    let end = c + egui::vec2(80.0, -2.0);
    h.button(end, RIGHT, false, egui::Modifiers::NONE);
    h.drive_until_idle();
    assert_eq!(runs(Wedge::E.index()), e0 + 1, "the gesture ran E");
    assert!(
        h.app().marking_menu().is_none(),
        "the menu did not stay open"
    );
    assert_eq!(label_count(&mut h, "Wedge E"), 0);

    // Clicking in a wedge's sector of the viewport (not on its button) runs it too; a press
    // elsewhere (the centre) and Esc only close the menu.
    let s0 = runs(Wedge::S.index());
    right_click(&mut h, c);
    h.click_at(
        c + egui::vec2(0.0, 50.0),
        egui::PointerButton::Primary,
        egui::Modifiers::NONE,
    );
    h.drive_until_idle();
    assert_eq!(
        runs(Wedge::S.index()),
        s0 + 1,
        "a click in the S sector runs S"
    );
    assert!(h.app().marking_menu().is_none());
    right_click(&mut h, c);
    h.click_at(c, egui::PointerButton::Primary, egui::Modifiers::NONE);
    h.drive_until_idle();
    assert!(
        h.app().marking_menu().is_none(),
        "a click in the centre closes it"
    );
    right_click(&mut h, c);
    h.key(egui::Key::Escape);
    h.drive_until_idle();
    assert!(h.app().marking_menu().is_none(), "Esc closes it");
    assert_eq!(runs(Wedge::S.index()), s0 + 1, "closing ran nothing");
}

#[test]
fn holding_the_right_button_opens_the_menu_and_releasing_over_a_wedge_runs_it() {
    let mut h = harness();
    let c = center(&h);
    h.hover_at(c);
    h.button(c, RIGHT, true, egui::Modifiers::NONE);
    std::thread::sleep(HOLD + std::time::Duration::from_millis(50));
    h.kittest().step();
    let open = h
        .app()
        .marking_menu()
        .expect("held: the menu opens")
        .clone();
    assert!(open.held);
    // Move to the W wedge and release there.
    let w0 = runs(Wedge::W.index());
    let at = c + egui::vec2(-70.0, 0.0);
    h.kittest().event(egui::Event::PointerMoved(at));
    h.kittest().step();
    h.button(at, RIGHT, false, egui::Modifiers::NONE);
    h.drive_until_idle();
    assert_eq!(runs(Wedge::W.index()), w0 + 1, "release over W ran it");
    assert!(h.app().marking_menu().is_none());

    // A hold released in the centre keeps the menu open (click mode).
    h.button(c, RIGHT, true, egui::Modifiers::NONE);
    std::thread::sleep(HOLD + std::time::Duration::from_millis(50));
    h.kittest().step();
    h.button(c, RIGHT, false, egui::Modifiers::NONE);
    h.drive_until_idle();
    let open = h.app().marking_menu().expect("still open");
    assert!(!open.held);
    assert_eq!(runs(Wedge::W.index()), w0 + 1);
}

// ---- Done-when 2: "Repeat <last>" ---------------------------------------------------------------

#[test]
fn repeat_runs_the_previous_command() {
    let mut h = harness();
    let c = center(&h);
    // Nothing repeatable yet (undo / redo / search are never offered).
    h.run_command(CmdId("edit.undo"));
    h.drive_until_idle();
    right_click(&mut h, c);
    assert_eq!(h.app().marking_menu().unwrap().menu.repeat, None);
    h.key(egui::Key::Escape);
    h.drive_until_idle();

    // Run NE through the menu, then "Repeat Wedge NE" runs it again.
    let i = Wedge::NE.index();
    let n0 = runs(i);
    right_click(&mut h, c);
    h.kittest()
        .get_by_role_and_label(Role::Button, "Wedge NE")
        .click();
    h.drive_until_idle();
    assert_eq!(runs(i), n0 + 1);
    // Undo in between does not change what "Repeat" offers.
    h.run_command(CmdId("edit.undo"));
    h.drive_until_idle();
    right_click(&mut h, c);
    assert_eq!(
        h.app().marking_menu().unwrap().menu.repeat,
        Some(WEDGE_CMDS[i])
    );
    h.kittest()
        .get_by_role_and_label(Role::Button, "Repeat Wedge NE")
        .click();
    h.drive_until_idle();
    assert_eq!(runs(i), n0 + 2, "Repeat ran the previous command");
    assert!(h.app().marking_menu().is_none());

    // A command run another way (search, shortcut, toolbar: here `run_command`) is repeated.
    h.run_command(WEDGE_CMDS[Wedge::SW.index()]);
    h.drive_until_idle();
    right_click(&mut h, c);
    assert!(h.kittest().query_by_label("Repeat Wedge SW").is_some());
}

// ---- Done-when 3: a provider entry only for a selected mesh body ---------------------------------

#[test]
fn a_test_provider_adds_an_entry_only_for_a_selected_mesh_body() {
    let mut h = harness();
    let bodies = add_bodies(&mut h);
    let (mesh, solid) = (bodies[0], bodies[1]);
    let c = center(&h);
    let has_entry = |h: &mut AppHarness| {
        let in_menu = h
            .app()
            .marking_menu()
            .unwrap()
            .menu
            .overflow
            .contains(&MESH_ONLY);
        let drawn = label_count(h, MESH_ONLY_LABEL) == 1;
        assert_eq!(
            in_menu, drawn,
            "the overflow list draws what the menu holds"
        );
        in_menu
    };

    for (sel, want) in [(None, false), (Some(solid), false), (Some(mesh), true)] {
        select_body(&mut h, sel);
        right_click(&mut h, c);
        assert_eq!(has_entry(&mut h), want, "selection {sel:?}");
        h.key(egui::Key::Escape);
        h.drive_until_idle();
    }

    // The entry runs its command; switching back to the solid drops it again.
    select_body(&mut h, Some(mesh));
    right_click(&mut h, c);
    let r0 = runs(MESH_ONLY_RUNS);
    h.kittest()
        .get_by_role_and_label(Role::Button, MESH_ONLY_LABEL)
        .click();
    h.drive_until_idle();
    assert_eq!(runs(MESH_ONLY_RUNS), r0 + 1);
    select_body(&mut h, Some(solid));
    right_click(&mut h, c);
    assert!(!has_entry(&mut h));
}

// ---- Done-when 4: every wedge is queryable by label ----------------------------------------------

#[test]
fn every_wedge_and_the_defaults_are_queryable_by_label() {
    let mut h = harness();
    let c = center(&h);
    let undo_before = label_count(&mut h, "Undo");
    right_click(&mut h, c);
    let menu = h.app().marking_menu().unwrap().menu.clone();
    for w in Wedge::ALL {
        let label = format!("Wedge {}", w.name());
        assert_eq!(menu.wedge(w), Some(WEDGE_CMDS[w.index()]), "{label}");
        let node = h.kittest().get_by_role_and_label(Role::Button, &label);
        // Each wedge sits in its direction from the centre.
        let r = node.rect();
        let d = r.center() - c;
        assert_eq!(Wedge::from_dir(d), Some(w), "{label} at {d:?}");
    }
    // The framework defaults that lost their wedge to the test provider are in the overflow
    // list (claims of unregistered commands are skipped), and drawn there.
    assert!(menu.overflow.contains(&CmdId("edit.undo")));
    assert!(menu.overflow.contains(&CmdId("edit.redo")));
    assert_eq!(
        label_count(&mut h, "Undo"),
        undo_before + 1,
        "Undo is in the overflow list"
    );
    assert!(h.app().registry().marking_menu_owners().contains(&"app"));
}

// ---- context providers in the browser and timeline menus -----------------------------------------

#[test]
fn context_providers_extend_the_browser_and_timeline_menus() {
    let mut h = harness();
    add_bodies(&mut h);
    if h.kittest().query_by_label("Expand Bodies").is_some() {
        h.kittest()
            .get_by_role_and_label(Role::Button, "Expand Bodies")
            .click();
        h.drive_until_idle();
    }
    let r0 = runs(CONTEXT_RUNS);
    h.kittest().get_by_label("Body1").click_secondary();
    h.drive_until_idle();
    assert_eq!(label_count(&mut h, CONTEXT_LABEL), 1, "browser row menu");
    h.kittest()
        .get_by_role_and_label(Role::Button, CONTEXT_LABEL)
        .click();
    h.drive_until_idle();
    assert_eq!(runs(CONTEXT_RUNS), r0 + 1);

    let first = h.app().timeline().labels()[0].to_owned();
    h.kittest()
        .get_by_role_and_label(Role::Button, &first)
        .click_secondary();
    h.drive_until_idle();
    assert!(
        h.kittest()
            .query_by_role_and_label(Role::Button, "Find in Browser")
            .is_some()
    );
    h.kittest()
        .get_by_role_and_label(Role::Button, CONTEXT_LABEL)
        .click();
    h.drive_until_idle();
    assert_eq!(runs(CONTEXT_RUNS), r0 + 2, "timeline node menu");
}
