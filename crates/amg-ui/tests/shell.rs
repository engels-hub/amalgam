//! UI-01 shell tests through `AppHarness` (kittest, no GPU).

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use amg_doc::Command as DocCommand;
use amg_jobs::{Cancelled, SpawnOptions};
use amg_ui::CommitBatch;
use amg_ui::design::{SKETCH_CONTEXT, tabs};
use amg_ui::module::{Registrar, TabDef, TabId, UiModule, WorkspaceDef, WorkspaceId};
use amg_ui::testing::AppHarness;
use amg_ui::testing::egui_kittest::kittest::Queryable;

// A module registered the way a domain lane does it: no framework file names it.
inventory::submit! { UiModule { id: "zz_test", register: register_test_module } }

fn register_test_module(r: &mut Registrar<'_>) {
    r.workspace(WorkspaceDef::new("test.ws", "Test Workspace", 100));
    r.toolbar_tab(TabDef {
        id: TabId("test.ws.tab"),
        workspace: WorkspaceId("test.ws"),
        label: "Test Tab",
        order: 0,
        contextual: None,
    });
}

#[test]
fn switching_workspace_changes_the_visible_tabs() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    {
        let k = h.kittest();
        assert!(k.query_by_label("SOLID").is_some());
        assert!(
            k.query_by_label("MESH").is_some(),
            "lane C's Mesh tab sits in Design"
        );
        assert!(k.query_by_label("TEST TAB").is_none());
        k.get_by_label("DESIGN ⏷").click();
    }
    h.drive_until_idle();
    h.kittest().get_by_label("Test Workspace").click();
    h.drive_until_idle();
    assert_eq!(h.app().workspace(), WorkspaceId("test.ws"));
    {
        let k = h.kittest();
        assert!(k.query_by_label("TEST TAB").is_some());
        assert!(k.query_by_label("SOLID").is_none());
        assert!(k.query_by_label("TEST WORKSPACE ⏷").is_some());
        k.get_by_label("TEST WORKSPACE ⏷").click();
    }
    h.drive_until_idle();
    h.kittest().get_by_label("Simulation").click();
    h.drive_until_idle();
    let k = h.kittest();
    assert!(k.query_by_label("SETUP").is_some());
    assert!(k.query_by_label("TEST TAB").is_none());
}

#[test]
fn inventory_module_is_collected_without_framework_edits() {
    let h = AppHarness::new();
    let reg = h.app().registry();
    assert!(reg.modules().contains(&"zz_test"));
    assert_eq!(reg.workspace_owner(WorkspaceId("test.ws")), Some("zz_test"));
    assert_eq!(reg.tab_owner(TabId("test.ws.tab")), Some("zz_test"));
}

#[test]
fn contextual_tab_shows_and_takes_focus_while_its_context_is_active() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    assert!(h.kittest().query_by_label("SKETCH").is_none());
    h.app_mut().with_cx(|cx| cx.enter_context(SKETCH_CONTEXT));
    h.drive_until_idle();
    assert!(h.kittest().query_by_label("SKETCH").is_some());
    assert_eq!(h.app().active_tab(), Some(tabs::SKETCH));
    assert!(h.kittest().query_by_label("FINISH SKETCH ⏷").is_some());
    h.app_mut().with_cx(|cx| cx.exit_context(SKETCH_CONTEXT));
    h.drive_until_idle();
    assert!(h.kittest().query_by_label("SKETCH").is_none());
    assert_eq!(h.app().active_tab(), Some(tabs::SOLID));
}

fn add_param(name: &str) -> CommitBatch {
    CommitBatch {
        label: format!("Add {name}"),
        commands: vec![DocCommand::AddParam {
            name: name.to_owned(),
            expr: "10".to_owned(),
            unit: None,
            comment: String::new(),
        }],
    }
}

fn params(h: &AppHarness, doc: usize) -> Vec<String> {
    let mut v: Vec<String> = h.app().documents()[doc]
        .document
        .design()
        .params()
        .iter()
        .map(|(_, p)| p.name.clone())
        .collect();
    v.sort();
    v
}

/// Clicks a document tab in the app bar (the browser root shows the same name lower down).
fn click_doc_tab(h: &mut AppHarness, name: &str) {
    let k = h.kittest();
    let node = k
        .get_all_by_label(name)
        .min_by(|a, b| a.rect().min.y.total_cmp(&b.rect().min.y))
        .expect("document tab");
    node.click();
}

#[test]
fn two_document_tabs_keep_independent_undo_stacks() {
    let mut h = AppHarness::new();
    h.app_mut().submit(add_param("a"));
    h.drive_until_idle();
    h.kittest().get_by_label("+").click();
    h.drive_until_idle();
    assert_eq!(h.app().documents().len(), 2);
    assert_eq!(h.app().active_document().name, "Untitled 2");

    h.app_mut().submit(add_param("b"));
    h.drive_until_idle();
    h.app_mut().submit(add_param("c"));
    h.drive_until_idle();
    assert_eq!(params(&h, 0), ["a"]);
    assert_eq!(params(&h, 1), ["b", "c"]);

    // Undo in document 2 leaves document 1 alone.
    h.kittest().get_by_label("Undo").click();
    h.drive_until_idle();
    assert_eq!(params(&h, 1), ["b"]);
    assert_eq!(params(&h, 0), ["a"]);

    // Document 1 still has its own single step.
    click_doc_tab(&mut h, "Untitled 1");
    h.drive_until_idle();
    assert_eq!(h.app().active_document().name, "Untitled 1");
    h.kittest().get_by_label("Undo").click();
    h.drive_until_idle();
    assert_eq!(params(&h, 0), Vec::<String>::new());
    assert_eq!(params(&h, 1), ["b"]);
    assert!(!h.app().documents()[0].document.history().can_undo());
    assert!(h.app().documents()[1].document.history().can_undo());

    h.kittest().get_by_label("Redo").click();
    h.drive_until_idle();
    assert_eq!(params(&h, 0), ["a"]);
    assert_eq!(params(&h, 1), ["b"]);
}

#[test]
fn a_failing_batch_rolls_back_and_reports_in_the_status_bar() {
    let mut h = AppHarness::new();
    h.app_mut().submit(CommitBatch {
        label: "Two params".to_owned(),
        commands: vec![
            add_param("x").commands[0].clone(),
            add_param("x").commands[0].clone(),
        ],
    });
    h.drive_until_idle();
    assert_eq!(
        params(&h, 0),
        Vec::<String>::new(),
        "whole batch rolled back"
    );
    assert!(!h.app().active_document().document.history().can_undo());
    let msg = h.app().status_message().expect("error shown").to_owned();
    assert!(msg.starts_with("Two params failed"), "{msg}");
    assert!(
        h.kittest()
            .query_by_label_contains("Two params failed")
            .is_some(),
        "status bar does not show: {msg}"
    );
}

#[test]
fn job_notifier_wakes_the_ui_within_one_frame() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    let (tx, rx) = std::sync::mpsc::channel::<()>();
    let handle = h
        .app()
        .jobs()
        .spawn_fn("gate", SpawnOptions::default(), move |_ctx| {
            let _ = rx.recv();
            Ok::<_, Cancelled>(())
        });
    // The spawn itself notifies; one frame shows the running job.
    assert!(h.kittest().ctx.has_requested_repaint());
    h.kittest().step();
    assert!(h.kittest().query_by_label("Jobs: 1").is_some());
    // Settle: nothing else repaints while the job waits.
    h.kittest().run();
    assert!(!h.kittest().ctx.has_requested_repaint());

    tx.send(()).unwrap();
    let t0 = std::time::Instant::now();
    while !handle.is_finished() {
        assert!(t0.elapsed() < std::time::Duration::from_secs(5));
        std::thread::yield_now();
    }
    // The finish woke the UI ...
    let t1 = std::time::Instant::now();
    while !h.kittest().ctx.has_requested_repaint() {
        assert!(
            t1.elapsed() < std::time::Duration::from_secs(1),
            "no repaint after the job finished"
        );
        std::thread::yield_now();
    }
    // ... and a single frame sees it.
    h.kittest().step();
    assert!(h.kittest().query_by_label("Jobs: 0").is_some());
}

#[test]
fn theme_menu_switches_between_light_and_dark() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    assert!(h.kittest().ctx.global_style().visuals.dark_mode);
    h.kittest().get_by_label("Theme").click();
    h.drive_until_idle();
    h.kittest().get_by_label("Light").click();
    h.drive_until_idle();
    assert_eq!(h.app().theme(), amg_ui::theme::Theme::Light);
    assert!(!h.kittest().ctx.global_style().visuals.dark_mode);
    assert_eq!(h.app().tokens(), &amg_ui::theme::ThemeTokens::light());
}

#[test]
fn fusion_layout_regions_are_present() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    let k = h.kittest();
    for label in [
        "File",
        "Undo",
        "Redo",
        "Untitled 1",
        "DESIGN ⏷",
        "SOLID",
        "SURFACE",
        "MESH",
        "INSPECT",
        "UTILITIES",
        "CREATE ⏷",
        "MODIFY ⏷",
        "BROWSER",
        "Document Settings",
        "Origin",
        "Timeline is empty",
        "Ready",
        "Jobs: 0",
        "Viewport",
    ] {
        assert!(
            k.query_all_by_label(label).next().is_some(),
            "missing `{label}`"
        );
    }
}
