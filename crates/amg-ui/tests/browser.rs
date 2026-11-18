//! UI-07 tests: the browser tree on the real app in `AppHarness` (kittest).
//!
//! Done-when 1: two components × three mesh bodies; hiding a body through its eye hides it
//! in the scene and undo shows it again; a rename persists through save / load. Done-when 2:
//! a provider registered by an `inventory::submit!` module in this file adds nodes without
//! framework changes. Done-when 3 (counter half; the timing half is `benches/browser.rs`):
//! the tree is rebuilt only on affecting events. Done-when 4: hovering a row highlights the
//! body.

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::path::PathBuf;
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::{AtomicUsize, Ordering};

use amg_core::{Point3, Transform};
use amg_doc::stub::IndexedMesh;
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, Command, ComponentId, ConstructionGeom,
    Dependency, DocEvent, Document, FeatureBox, FeatureError, FeatureOutput, RecomputeCtx,
};
use amg_ui::app::CmdEnv;
use amg_ui::browser::visibility::{VISIBILITY_KIND, hidden};
use amg_ui::browser::{BrowserAnchor, BrowserNode, BrowserProvider, NodeKey};
use amg_ui::command::CmdId;
use amg_ui::file::FileOp;
use amg_ui::module::{Registrar, UiModule};
use amg_ui::scene::{CpuSceneBackend, HighlightLayer};
use amg_ui::testing::egui_kittest::kittest::Queryable;
use amg_ui::testing::{AppHarness, FakeFileDialog};
use amg_ui::{AppCx, CommitBatch};
use egui::accesskit::Role;
use serde::{Deserialize, Serialize};

// ---- test features ---------------------------------------------------------------------------

/// `count` flat 2 × 2 mm squares side by side from `x0`, one body each.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Squares {
    count: u32,
    x0: f64,
}

fn square(x: f64) -> IndexedMesh {
    let p = vec![
        Point3::new(x, 0.0, 0.0),
        Point3::new(x + 2.0, 0.0, 0.0),
        Point3::new(x + 2.0, 2.0, 0.0),
        Point3::new(x, 2.0, 0.0),
    ];
    IndexedMesh::new(p, vec![[0, 1, 2], [0, 2, 3]]).expect("valid square")
}

impl amg_doc::Feature for Squares {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let bodies = (0..self.count)
            .map(|i| BodyChange {
                id: ctx.new_body_id(i),
                change: BodyChangeKind::Created {
                    geom: BodyGeom::Mesh(Arc::new(square(self.x0 + 3.0 * f64::from(i)))),
                    naming: None,
                },
            })
            .collect();
        Ok(FeatureOutput {
            bodies,
            ..FeatureOutput::default()
        })
    }
}

amg_doc::register_feature!(Squares, "uitest.browser_squares", version = 1);

/// A construction plane (XY).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct TestPlane;

impl amg_doc::Feature for TestPlane {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, _ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Ok(FeatureOutput {
            construction: Some(ConstructionGeom::Plane(amg_core::Plane::XY)),
            ..FeatureOutput::default()
        })
    }
}

amg_doc::register_feature!(TestPlane, "uitest.browser_plane", version = 1);

// ---- a provider registered the way a domain lane does it ---------------------------------------

inventory::submit! { UiModule { id: "zz_browser_test", register: register_test_module } }

fn register_test_module(r: &mut Registrar<'_>) {
    r.browser_provider(Box::new(StudyProvider));
}

/// `(node, visible)` calls of `StudyProvider::set_visible`.
static STUDY_TOGGLES: Mutex<Vec<(String, bool)>> = Mutex::new(Vec::new());
/// `build` calls with a component anchor.
static STUDY_COMPONENT_BUILDS: AtomicUsize = AtomicUsize::new(0);

struct StudyProvider;

impl BrowserProvider for StudyProvider {
    fn id(&self) -> &'static str {
        "test.studies"
    }

    fn affected_by(&self, ev: &DocEvent) -> bool {
        matches!(ev, DocEvent::ExtChanged(k) if k.kind == "test.study")
    }

    fn build(&self, _env: &CmdEnv<'_>, anchor: &BrowserAnchor, out: &mut Vec<BrowserNode>) {
        match anchor {
            BrowserAnchor::Root => out.push(
                BrowserNode::new(NodeKey::new("test.study/1"), "Test Study")
                    .visible(true)
                    .children(vec![BrowserNode::new(
                        NodeKey::new("test.study/1/loads"),
                        "Loads",
                    )]),
            ),
            BrowserAnchor::Component(_) => {
                STUDY_COMPONENT_BUILDS.fetch_add(1, Ordering::SeqCst);
            }
        }
    }

    fn set_visible(&self, node: &NodeKey, visible: bool, _cx: &mut AppCx<'_>) {
        STUDY_TOGGLES
            .lock()
            .unwrap()
            .push((node.as_str().to_owned(), visible));
    }
}

// ---- fixture -----------------------------------------------------------------------------------

struct Fixture {
    h: AppHarness,
    comps: [ComponentId; 2],
    /// Bodies of component 0 then 1, three each.
    bodies: Vec<BodyId>,
}

fn commit(h: &mut AppHarness, cmd: Command) {
    h.app_mut().submit(CommitBatch {
        label: "Setup".into(),
        commands: vec![cmd],
    });
    h.drive_until_idle();
}

/// Two components ("Bracket A", "Bracket B") with three bodies each, plus one root plane.
fn fixture() -> Fixture {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    let root = h.document().design().root();
    let mut comps = Vec::new();
    for (i, name) in ["Bracket A", "Bracket B"].into_iter().enumerate() {
        commit(
            &mut h,
            Command::NewComponent {
                parent: root,
                name: name.into(),
                transform: Transform::IDENTITY,
            },
        );
        let c = h
            .document()
            .design()
            .components()
            .map(|(id, _)| id)
            .filter(|id| *id != root && !comps.contains(id))
            .max()
            .expect("new component");
        comps.push(c);
        commit(
            &mut h,
            Command::AddFeature {
                at: None,
                feature: FeatureBox::new(Squares {
                    count: 3,
                    x0: 10.0 * i as f64,
                }),
                context: c,
                name: None,
            },
        );
    }
    commit(
        &mut h,
        Command::AddFeature {
            at: None,
            feature: FeatureBox::new(TestPlane),
            context: root,
            name: None,
        },
    );
    let comps = [comps[0], comps[1]];
    let mut bodies: Vec<BodyId> = h
        .document()
        .evaluated()
        .bodies()
        .map(|(id, _)| id)
        .collect();
    bodies.sort();
    assert_eq!(bodies.len(), 6);
    for (n, b) in bodies.iter().enumerate() {
        let comp = h.document().evaluated().body(*b).unwrap().component;
        assert_eq!(comp, comps[n / 3]);
    }
    Fixture { h, comps, bodies }
}

fn click(h: &mut AppHarness, label: &str) {
    h.kittest().get_by_label(label).click();
    h.drive_until_idle();
}

fn click_button(h: &mut AppHarness, label: &str) {
    h.kittest()
        .get_by_role_and_label(Role::Button, label)
        .click();
    h.drive_until_idle();
}

/// Clicks "Rename" in the open context menu. A focused text field blinks its cursor (a repaint
/// request every frame), so this steps a fixed number of frames instead of waiting for idle.
fn start_rename(h: &mut AppHarness) {
    h.kittest()
        .get_by_role_and_label(Role::Button, "Rename")
        .click();
    h.kittest().run_steps(3);
}

fn has_label(h: &mut AppHarness, label: &str) -> bool {
    h.kittest().query_all_by_label(label).next().is_some()
}

/// Whether the scene draws `body` (its instance in the CPU backend is visible).
fn drawn(h: &AppHarness, body: BodyId) -> bool {
    let scene = h.scene();
    let cpu = scene
        .backend()
        .as_any()
        .downcast_ref::<CpuSceneBackend>()
        .expect("default backend");
    let keys = scene.pick_map().body_keys(body);
    assert!(!keys.is_empty(), "{body:?} is in the scene");
    cpu.instances()
        .filter(|(_, i)| keys.contains(&i.key))
        .any(|(_, i)| i.visible)
}

fn temp_dir(tag: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!(
        "amg-ui-browser-{tag}-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

// ---- Done-when 1 -------------------------------------------------------------------------------

#[test]
fn eye_toggle_hides_a_body_in_the_scene_and_undo_shows_it_again() {
    let Fixture {
        mut h,
        comps,
        bodies,
    } = fixture();
    // The framework nodes, and both components (collapsed).
    for label in [
        "Untitled 1",
        "Document Settings",
        "Named Views",
        "Origin",
        "Construction",
        "Bracket A",
        "Bracket B",
    ] {
        assert!(has_label(&mut h, label), "missing `{label}`");
    }
    assert!(!has_label(&mut h, "Body2"), "components start collapsed");
    click_button(&mut h, "Expand Bracket A");
    click_button(&mut h, "Expand Bodies");
    for label in ["Body1", "Body2", "Body3"] {
        assert!(has_label(&mut h, label), "missing `{label}`");
    }
    assert!(!has_label(&mut h, "Body4"), "Bracket B stays collapsed");

    // Hide Body2: one undo entry, hidden in the scene; the others stay drawn.
    let b = bodies[1];
    assert!(drawn(&h, b));
    let depth = h.document().history().stack().applied();
    click_button(&mut h, "Hide Body2");
    assert_eq!(h.document().history().stack().applied(), depth + 1);
    assert!(!h.document().design().body_props(b).visible);
    assert!(!drawn(&h, b), "hidden in the scene");
    assert!(drawn(&h, bodies[0]) && drawn(&h, bodies[2]));
    assert!(
        has_label(&mut h, "Show Body2"),
        "the eye shows the new state"
    );

    // Undo shows it again.
    h.app_mut().with_cx(|cx| cx.undo());
    h.drive_until_idle();
    assert!(h.document().design().body_props(b).visible);
    assert!(drawn(&h, b), "undo shows it in the scene again");
    assert!(has_label(&mut h, "Hide Body2"));

    // An occurrence's eye hides every body of its component (SetOccurrenceVisible).
    click_button(&mut h, "Hide Bracket B");
    let occ = h
        .document()
        .design()
        .occurrences()
        .find(|(_, o)| o.component == comps[1])
        .map(|(id, o)| (id, o.visible))
        .unwrap();
    assert!(!occ.1);
    for b in &bodies[3..] {
        assert!(!drawn(&h, *b), "{b:?} hidden with its occurrence");
    }
    assert!(drawn(&h, bodies[0]));

    // A folder's eye hides all of its bodies in one undo entry.
    let depth = h.document().history().stack().applied();
    click_button(&mut h, "Hide Bodies");
    assert_eq!(h.document().history().stack().applied(), depth + 1);
    for b in &bodies[..3] {
        assert!(!drawn(&h, *b));
    }

    // Construction geometry has no S01 visibility: its eye writes `ui.visibility`.
    click_button(&mut h, "Expand Construction");
    click_button(&mut h, "Hide Plane1");
    let hid = hidden(h.document().design());
    assert_eq!(hid.len(), 1);
    assert!(hid.iter().next().unwrap().starts_with("construction/"));
    assert!(
        h.document()
            .design()
            .exts()
            .any(|(k, _)| k.kind == VISIBILITY_KIND)
    );
    assert!(has_label(&mut h, "Show Plane1"));
    h.app_mut().with_cx(|cx| cx.undo());
    h.drive_until_idle();
    assert!(
        hidden(h.document().design()).is_empty(),
        "undo shows it again"
    );
}

#[test]
fn renaming_a_body_persists_through_save_and_load() {
    let Fixture { mut h, bodies, .. } = fixture();
    let fake = FakeFileDialog::new();
    h.app_mut().set_file_dialog_backend(fake.clone());
    click_button(&mut h, "Expand Bracket A");
    click_button(&mut h, "Expand Bodies");

    // Right-click → Rename → type → Enter.
    h.kittest().get_by_label("Body1").click_secondary();
    h.drive_until_idle();
    start_rename(&mut h);
    let field = h.kittest().get_by_role(Role::TextInput);
    assert!(field.is_focused(), "the rename field takes focus");
    field.type_text("Base Plate");
    h.kittest().run_steps(2);
    h.kittest().key_press(egui::Key::Enter);
    h.drive_until_idle();
    assert_eq!(
        h.document().design().body_props(bodies[0]).name.as_deref(),
        Some("Base Plate")
    );
    assert!(has_label(&mut h, "Base Plate"));
    assert!(!has_label(&mut h, "Body1"));

    // Esc cancels a rename.
    h.kittest().get_by_label("Body2").click_secondary();
    h.drive_until_idle();
    start_rename(&mut h);
    h.kittest().get_by_role(Role::TextInput).type_text("Never");
    h.kittest().run_steps(2);
    h.kittest().key_press(egui::Key::Escape);
    h.drive_until_idle();
    assert!(h.document().design().body_props(bodies[1]).name.is_none());
    assert!(has_label(&mut h, "Body2"));

    // Renaming an occurrence renames its component (RenameComponent).
    h.kittest().get_by_label("Bracket B").click_secondary();
    h.drive_until_idle();
    start_rename(&mut h);
    h.kittest()
        .get_by_role(Role::TextInput)
        .type_text("Bracket C");
    h.kittest().run_steps(2);
    h.kittest().key_press(egui::Key::Enter);
    h.drive_until_idle();
    assert!(
        h.document()
            .design()
            .components()
            .any(|(_, c)| c.name == "Bracket C")
    );

    // Save As → close → open: the names are back, and the browser shows them.
    let dir = temp_dir("rename");
    let path = dir.join("rename.amg");
    fake.answer(Some(path.clone()));
    h.run_command(CmdId("file.save_as"));
    h.drive_until_idle();
    assert!(path.exists());
    h.run_command(CmdId("file.close"));
    h.drive_until_idle();
    h.app_mut().file_op(None, FileOp::OpenPath(path.clone()));
    h.drive_until_idle();
    assert_eq!(
        h.app().active_document().path.as_deref(),
        Some(path.as_path())
    );
    assert_eq!(
        h.document().design().body_props(bodies[0]).name.as_deref(),
        Some("Base Plate")
    );
    assert!(has_label(&mut h, "Bracket C"));
    click_button(&mut h, "Expand Bracket A");
    click_button(&mut h, "Expand Bodies");
    assert!(has_label(&mut h, "Base Plate"));
    let _ = std::fs::remove_dir_all(dir);
}

#[test]
fn click_selects_and_a_selection_set_restores_on_double_click() {
    let Fixture { mut h, bodies, .. } = fixture();
    click_button(&mut h, "Expand Bracket A");
    click_button(&mut h, "Expand Bodies");
    click(&mut h, "Body1");
    let sel: Vec<_> = h
        .app()
        .active_document()
        .selection
        .items()
        .iter()
        .map(|i| i.target.body())
        .collect();
    assert_eq!(sel, vec![Some(bodies[0])]);
    assert_eq!(
        h.scene().highlight(HighlightLayer::Selected).len(),
        1,
        "the selected body is highlighted (its instance resolved through the scene)"
    );
    // Ctrl+click adds.
    h.kittest()
        .get_by_label("Body3")
        .click_modifiers(egui::Modifiers::COMMAND);
    h.drive_until_idle();
    assert_eq!(h.app().active_document().selection.len(), 2);

    // Save as a set; it appears under "Selection Sets".
    h.run_command(CmdId("select.save_set"));
    h.drive_until_idle();
    assert!(has_label(&mut h, "Selection Sets"));
    click_button(&mut h, "Expand Selection Sets");
    assert!(has_label(&mut h, "Selection Set 1"));
    h.run_command(CmdId("select.clear"));
    h.drive_until_idle();
    assert!(h.app().active_document().selection.is_empty());

    // Double-click restores it. kittest runs one frame per queued pointer event and egui's
    // clock advances with every pass, so widen the double-click window for the test.
    h.kittest().run_steps(60);
    h.kittest()
        .ctx
        .options_mut(|o| o.input_options.max_double_click_delay = 2.0);
    let node = h.kittest().get_by_label("Selection Set 1");
    node.click();
    node.click();
    h.drive_until_idle();
    let mut got: Vec<_> = h
        .app()
        .active_document()
        .selection
        .items()
        .iter()
        .filter_map(|i| i.target.body())
        .collect();
    got.sort();
    assert_eq!(got, vec![bodies[0], bodies[2]]);
}

// ---- Done-when 2 -------------------------------------------------------------------------------

#[test]
fn a_registered_provider_adds_nodes_under_root_without_framework_changes() {
    let Fixture { mut h, .. } = fixture();
    assert!(
        h.app()
            .registry()
            .browser_providers()
            .any(|p| p.id() == "test.studies")
    );
    assert!(has_label(&mut h, "Test Study"));
    assert!(
        !has_label(&mut h, "Loads"),
        "provider nodes start collapsed"
    );
    click_button(&mut h, "Expand Test Study");
    assert!(has_label(&mut h, "Loads"));
    // Asked once per component anchor (root + 2 components) on every rebuild.
    assert!(STUDY_COMPONENT_BUILDS.load(Ordering::SeqCst) >= 3);
    // Row actions come back to the provider.
    click_button(&mut h, "Hide Test Study");
    let toggles = STUDY_TOGGLES.lock().unwrap().clone();
    assert!(toggles.contains(&("test.study/1".to_owned(), false)));
    let labels = h.app().browser().row_labels();
    let study = labels.iter().position(|l| *l == "Test Study").unwrap();
    let origin = labels.iter().position(|l| *l == "Origin").unwrap();
    assert!(study > origin, "provider nodes follow the framework nodes");
}

// ---- Done-when 3 (counter) ---------------------------------------------------------------------

#[test]
fn the_tree_is_rebuilt_only_on_affecting_events() {
    let Fixture {
        mut h,
        comps,
        bodies,
    } = fixture();
    h.drive_until_idle();
    let r0 = h.app().browser().rebuilds();
    // Expanding, hovering, scrolling and idle frames re-flatten at most.
    click_button(&mut h, "Expand Bracket A");
    click_button(&mut h, "Expand Bodies");
    h.kittest().get_by_label("Body2").hover();
    h.drive_until_idle();
    h.kittest().get_by_label("Body3").hover();
    h.drive_until_idle();
    click_button(&mut h, "Collapse Bodies");
    for _ in 0..5 {
        h.kittest().run();
    }
    assert_eq!(
        h.app().browser().rebuilds(),
        r0,
        "UI-only changes do not rebuild"
    );

    // Events the browser does not show: a parameter, an occurrence move, selection.
    commit(
        &mut h,
        Command::AddParam {
            name: "w".into(),
            expr: "5 mm".into(),
            unit: None,
            comment: String::new(),
        },
    );
    let occ = h
        .document()
        .design()
        .occurrences()
        .find(|(_, o)| o.component == comps[0])
        .map(|(id, _)| id)
        .unwrap();
    commit(
        &mut h,
        Command::SetOccurrenceTransform {
            occ,
            transform: Transform::from_translation(amg_core::Vec3::new(1.0, 0.0, 0.0)),
        },
    );
    h.run_command(CmdId("select.clear"));
    h.drive_until_idle();
    assert_eq!(
        h.app().browser().rebuilds(),
        r0,
        "unaffecting events do not rebuild"
    );

    // A body property change rebuilds exactly once.
    let props = h.document().design().body_props(bodies[4]);
    commit(
        &mut h,
        Command::SetBodyProps {
            body: bodies[4],
            props: amg_doc::BodyProps {
                opacity: 0.5,
                ..props
            },
        },
    );
    assert_eq!(h.app().browser().rebuilds(), r0 + 1);
}

// ---- Done-when 4 -------------------------------------------------------------------------------

#[test]
fn hovering_a_row_highlights_the_body() {
    let Fixture { mut h, bodies, .. } = fixture();
    click_button(&mut h, "Expand Bracket B");
    click_button(&mut h, "Expand Bodies");
    assert!(h.scene().highlight(HighlightLayer::Hover).is_empty());
    h.kittest().get_by_label("Body5").hover();
    h.drive_until_idle();
    let keys = h.scene().pick_map().body_keys(bodies[4]);
    let hl: Vec<_> = h
        .scene()
        .highlight(HighlightLayer::Hover)
        .iter()
        .map(|i| i.key)
        .collect();
    assert_eq!(hl.len(), 1);
    assert!(keys.contains(&hl[0]), "the hover layer holds Body5");
    // An occurrence row highlights all of its bodies.
    h.kittest().get_by_label("Bracket B").hover();
    h.drive_until_idle();
    assert_eq!(h.scene().highlight(HighlightLayer::Hover).len(), 3);
    // Leaving the browser clears it.
    h.hover_at(egui::pos2(700.0, 400.0));
    h.drive_until_idle();
    assert!(h.scene().highlight(HighlightLayer::Hover).is_empty());
}

// ---- documents and fixtures without the harness ---------------------------------------------

#[test]
fn a_replaced_document_rebuilds_the_tree() {
    let mut doc = Document::new("Other");
    doc.history_mut().mark_saved();
    let mut h = AppHarness::with_document(doc);
    h.drive_until_idle();
    assert!(has_label(&mut h, "Other"), "the root shows the design name");
    assert!(!has_label(&mut h, "Bodies"), "empty folders are not shown");
}
