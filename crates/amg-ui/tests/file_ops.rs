//! UI-19 tests: file operations through the fake file-dialog backend, the dirty-close prompt,
//! crash recovery (including a kill test in a child process) and the "opening a 50 MB file
//! never blocks a frame" check.
//!
//! `timing_*` tests print their measurement and assert the budget only under
//! `AMG_STRICT_TIMING=1`.

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::io::Read as _;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use amg_doc::io::SaveOptions;
use amg_doc::{
    Command as DocCommand, Dependency, Document, FeatureBox, FeatureError, FeatureOutput,
    RecomputeCtx, RecoveryConfig, RecoverySession, find_recoverable,
};
use amg_ui::CmdId;
use amg_ui::CommitBatch;
use amg_ui::file::{CloseState, FileDialogKind};
use amg_ui::testing::egui_kittest::kittest::Queryable;
use amg_ui::testing::{AppHarness, FakeFileDialog};
use serde::{Deserialize, Serialize};

// ---- helpers ------------------------------------------------------------------------------

/// A fresh folder under the system temp dir.
fn temp_dir(tag: &str) -> PathBuf {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let dir = std::env::temp_dir().join(format!("amg-ui19-{tag}-{}-{nanos}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn add_param(name: &str) -> CommitBatch {
    CommitBatch {
        label: format!("Add {name}"),
        commands: vec![DocCommand::AddParam {
            name: name.to_owned(),
            expr: "10 mm".to_owned(),
            unit: None,
            comment: String::new(),
        }],
    }
}

fn params(doc: &Document) -> Vec<String> {
    let mut v: Vec<String> = doc
        .design()
        .params()
        .iter()
        .map(|(_, p)| p.name.clone())
        .collect();
    v.sort();
    v
}

/// `design.json` of a saved `.amg`.
fn design_json(path: &Path) -> Vec<u8> {
    let file = std::fs::File::open(path).unwrap();
    let mut zip = zip::ZipArchive::new(file).unwrap();
    let mut entry = zip.by_name("design.json").unwrap();
    let mut out = Vec::new();
    entry.read_to_end(&mut out).unwrap();
    out
}

fn harness_with_fake() -> (AppHarness, Arc<FakeFileDialog>) {
    let mut h = AppHarness::new();
    let fake = FakeFileDialog::new();
    h.app_mut().set_file_dialog_backend(fake.clone());
    h.drive_until_idle();
    (h, fake)
}

/// A few frames: a freshly opened `Modal` or menu is laid out invisibly in its first frame
/// (UI-10 note), so clicks wait until it has a real position.
fn settle(h: &mut AppHarness) {
    for _ in 0..3 {
        h.kittest().step();
    }
}

/// The window manager asks the window to close (eframe sets `close_requested`). Returns the
/// viewport commands of that frame.
fn request_window_close(h: &mut AppHarness) -> Vec<egui::ViewportCommand> {
    h.kittest()
        .input_mut()
        .viewports
        .entry(egui::ViewportId::ROOT)
        .or_default()
        .events
        .push(egui::ViewportEvent::Close);
    h.kittest().step();
    let cmds = sent_commands(h);
    settle(h);
    cmds
}

/// The viewport commands the last frame sent.
fn sent_commands(h: &mut AppHarness) -> Vec<egui::ViewportCommand> {
    h.kittest()
        .output()
        .viewport_output
        .get(&egui::ViewportId::ROOT)
        .map(|v| v.commands.clone())
        .unwrap_or_default()
}

fn sent_close(cmds: &[egui::ViewportCommand]) -> bool {
    cmds.iter()
        .any(|c| matches!(c, egui::ViewportCommand::Close))
}

fn sent_cancel_close(cmds: &[egui::ViewportCommand]) -> bool {
    cmds.iter()
        .any(|c| matches!(c, egui::ViewportCommand::CancelClose))
}

// ---- Done-when 1: save → reopen gives an equal design.json --------------------------------

#[test]
fn save_then_reopen_gives_an_equal_design_json() {
    let dir = temp_dir("save");
    let (mut h, fake) = harness_with_fake();
    for name in ["width", "height"] {
        h.app_mut().submit(add_param(name));
        h.drive_until_idle();
    }
    assert!(h.document().is_dirty());

    // Save through the File menu: no path yet, so the save dialog asks. No extension given.
    fake.answer(Some(dir.join("bracket")));
    h.kittest().get_by_label("File").click();
    h.drive_until_idle();
    h.kittest().get_by_label_contains("Save document").click();
    h.drive_until_idle();
    let first = dir.join("bracket.amg");
    assert!(first.exists(), "saved with the .amg extension added");
    let reqs = fake.requests();
    assert_eq!(reqs.len(), 1);
    assert_eq!(reqs[0].kind, FileDialogKind::Save);
    assert_eq!(reqs[0].filters[0].extensions, ["amg"]);
    let tab = h.app().active_document();
    assert_eq!(tab.name, "bracket");
    assert_eq!(tab.path.as_deref(), Some(first.as_path()));
    assert!(
        !h.document().is_dirty(),
        "a finished save marks the document clean"
    );
    assert!(
        h.kittest().query_all_by_label("bracket").next().is_some(),
        "tab renamed"
    );

    // Save again: the document has a file, so no dialog.
    h.app_mut().submit(add_param("depth"));
    h.drive_until_idle();
    h.run_command(CmdId("file.save"));
    assert_eq!(fake.requests().len(), 1, "Save with a path shows no dialog");
    assert!(!h.document().is_dirty());

    // Close the clean tab (no prompt) and open the file again.
    h.run_command(CmdId("file.close"));
    assert_eq!(h.app().close_state(), CloseState::Idle);
    assert_eq!(h.app().documents().len(), 1);
    assert!(
        h.app().documents()[0].path.is_none(),
        "a fresh Untitled replaced it"
    );
    fake.answer(Some(first.clone()));
    h.run_command(CmdId("file.open"));
    assert_eq!(fake.requests()[1].kind, FileDialogKind::Open);
    assert_eq!(h.app().documents().len(), 2);
    let tab = h.app().active_document();
    assert_eq!(tab.name, "bracket");
    assert_eq!(tab.path.as_deref(), Some(first.as_path()));
    assert!(!tab.document.is_dirty(), "an opened file is clean");
    assert_eq!(params(&tab.document), ["depth", "height", "width"]);

    // Opening an open file activates its tab instead of a second copy.
    let first_tab = h.app().documents()[0].id;
    h.app_mut().activate(first_tab);
    fake.answer(Some(first.clone()));
    h.run_command(CmdId("file.open"));
    assert_eq!(h.app().documents().len(), 2);
    assert_eq!(h.app().active_document().name, "bracket");

    // Save As the reopened document: its design.json equals the first file's.
    let second = dir.join("copy.amg");
    fake.answer(Some(second.clone()));
    h.run_command(CmdId("file.save_as"));
    assert!(second.exists());
    assert_eq!(design_json(&first), design_json(&second));
    assert_eq!(h.app().active_document().name, "copy");

    // Save Compact leaves the cache out; the design is the same.
    h.run_command(CmdId("file.save_compact"));
    assert_eq!(design_json(&first), design_json(&second));

    let recent = h.app().recent_files().files().to_vec();
    assert_eq!(recent[0], second);
    assert!(recent.contains(&first));

    drop(h);
    let _ = std::fs::remove_dir_all(dir);
}

#[test]
fn cancelled_dialog_and_failed_open_change_nothing() {
    let dir = temp_dir("cancel");
    let (mut h, fake) = harness_with_fake();
    h.app_mut().submit(add_param("a"));
    h.drive_until_idle();
    // Empty answer queue: the user cancels.
    h.run_command(CmdId("file.save"));
    assert!(h.document().is_dirty());
    assert!(h.app().active_document().path.is_none());

    let bogus = dir.join("not-amalgam.amg");
    std::fs::write(&bogus, b"hello").unwrap();
    fake.answer(Some(bogus));
    h.run_command(CmdId("file.open"));
    assert_eq!(h.app().documents().len(), 1, "a broken file opens no tab");
    let shown: Vec<String> = h
        .app()
        .notifications()
        .items()
        .iter()
        .map(|t| t.notification.title.clone())
        .collect();
    assert!(
        shown.iter().any(|t| t == "Cannot open the file"),
        "{shown:?}"
    );
    drop(h);
    let _ = std::fs::remove_dir_all(dir);
}

#[test]
fn open_recent_persists_and_reopens() {
    let dir = temp_dir("recent");
    let recent = dir.join("recent.json");
    let file = dir.join("r.amg");
    let mut doc = Document::new("r");
    doc.apply(add_param("x").commands.remove(0)).unwrap();
    amg_doc::save(&file, &doc.snapshot(), &SaveOptions::default()).unwrap();
    {
        let (mut h, fake) = harness_with_fake();
        h.app_mut().set_recent_path(Some(recent.clone()));
        fake.answer(Some(file.clone()));
        h.run_command(CmdId("file.open"));
        assert!(recent.exists());
    }
    let (mut h, _fake) = harness_with_fake();
    h.app_mut().set_recent_path(Some(recent));
    assert_eq!(h.app().recent_files().files(), std::slice::from_ref(&file));
    h.kittest().get_by_label("File").click();
    h.drive_until_idle();
    h.kittest().get_by_label_contains("Open Recent").click();
    h.drive_until_idle();
    h.kittest().get_by_label("r.amg").click();
    h.drive_until_idle();
    assert_eq!(h.app().documents().len(), 2);
    assert_eq!(
        h.app().active_document().path.as_deref(),
        Some(file.as_path())
    );
    drop(h);
    let _ = std::fs::remove_dir_all(dir);
}

#[test]
fn import_and_export_slots_are_in_the_file_menu() {
    let (mut h, _fake) = harness_with_fake();
    h.kittest().get_by_label("File").click();
    h.drive_until_idle();
    for label in [
        "New document",
        "Open…",
        "Save document",
        "Save As…",
        "Save Compact",
        "Close document",
        "Open Recent",
        "Import",
    ] {
        assert!(
            h.kittest()
                .query_all_by_label_contains(label)
                .next()
                .is_some(),
            "missing `{label}`"
        );
    }
}

// ---- Done-when 2: dirty close prompt; Cancel keeps the app open ----------------------------

#[test]
fn closing_a_dirty_document_prompts_and_cancel_keeps_the_app_open() {
    let (mut h, _fake) = harness_with_fake();
    h.app_mut().submit(add_param("a"));
    h.drive_until_idle();

    let cmds = request_window_close(&mut h);
    assert!(
        sent_cancel_close(&cmds),
        "the close was cancelled: {cmds:?}"
    );
    assert!(!sent_close(&cmds));
    let doc = h.app().active_document().id;
    assert_eq!(h.app().close_state(), CloseState::Prompting(doc));
    assert!(h.kittest().query_by_label("Unsaved changes").is_some());
    assert!(
        h.kittest()
            .query_by_label_contains("Save changes to “Untitled 1”")
            .is_some()
    );

    h.kittest().get_by_label("Cancel").click();
    for _ in 0..5 {
        h.kittest().step();
        assert!(!sent_close(&sent_commands(&mut h)), "Cancel never closes");
    }
    h.drive_until_idle();
    assert_eq!(h.app().close_state(), CloseState::Idle);
    assert!(h.kittest().query_by_label("Unsaved changes").is_none());
    assert_eq!(h.app().documents().len(), 1);
    assert!(h.document().is_dirty(), "nothing was saved or discarded");

    // Asking again and choosing "Don't save" lets the window close.
    assert!(sent_cancel_close(&request_window_close(&mut h)));
    h.kittest().get_by_label("Don't save").click();
    h.kittest().step();
    assert!(
        sent_close(&sent_commands(&mut h)),
        "Don't save closes the window"
    );
    assert_eq!(h.app().close_state(), CloseState::Closing);
    // The window's own close request is now let through.
    assert!(!sent_cancel_close(&request_window_close(&mut h)));
}

#[test]
fn save_in_the_close_prompt_saves_then_closes() {
    let dir = temp_dir("prompt-save");
    let (mut h, fake) = harness_with_fake();
    h.app_mut().new_document();
    let ids: Vec<_> = h.app().documents().iter().map(|d| d.id).collect();
    for id in ids {
        h.app_mut().activate(id);
        h.app_mut().submit(add_param("a"));
        h.drive_until_idle();
    }
    request_window_close(&mut h);
    let first = h.app().documents()[0].id;
    assert_eq!(h.app().close_state(), CloseState::Prompting(first));

    fake.answer(Some(dir.join("one.amg")));
    h.kittest().get_by_label("Save").click();
    h.drive_until_idle();
    assert!(dir.join("one.amg").exists());
    settle(&mut h);
    let second = h.app().documents()[1].id;
    assert_eq!(
        h.app().close_state(),
        CloseState::Prompting(second),
        "then the next dirty document"
    );
    // Cancelling its save dialog stops the whole close.
    h.kittest().get_by_label("Save").click();
    h.drive_until_idle();
    assert_eq!(h.app().close_state(), CloseState::Idle);
    assert_eq!(fake.requests().len(), 2, "the second save asked for a path");
    assert!(h.app().documents()[1].document.is_dirty());

    request_window_close(&mut h);
    assert_eq!(h.app().close_state(), CloseState::Prompting(second));
    h.kittest().get_by_label("Don't save").click();
    h.kittest().step();
    assert!(sent_close(&sent_commands(&mut h)));
    drop(h);
    let _ = std::fs::remove_dir_all(dir);
}

#[test]
fn clean_documents_close_without_a_prompt() {
    let (mut h, _fake) = harness_with_fake();
    assert!(!sent_cancel_close(&request_window_close(&mut h)));
    assert_eq!(h.app().close_state(), CloseState::Closing);
}

#[test]
fn closing_a_dirty_tab_prompts_and_dont_save_removes_it() {
    let (mut h, _fake) = harness_with_fake();
    h.app_mut().new_document();
    h.app_mut().submit(add_param("a"));
    h.drive_until_idle();
    h.run_command(CmdId("file.close"));
    let id = h.app().active_document().id;
    assert_eq!(h.app().close_state(), CloseState::Prompting(id));
    settle(&mut h);
    h.kittest().get_by_label("Don't save").click();
    h.drive_until_idle();
    assert_eq!(h.app().documents().len(), 1);
    assert!(h.app().documents().iter().all(|d| d.id != id));
    assert!(
        !sent_close(&sent_commands(&mut h)),
        "closing a tab never closes the window"
    );
}

// ---- Done-when 3: kill test ---------------------------------------------------------------

const KILL_CHILD_ENV: &str = "AMG_UI19_KILL_CHILD";
const KILL_TEST: &str = "kill_test_recovery_dialog_offers_and_restores_the_document";

/// The child: commits 5 batches in a recovery-enabled app, writes what it should recover to,
/// then dies without any clean-up.
fn kill_child(dir: &Path) -> ! {
    let mut h = AppHarness::new();
    h.app_mut()
        .set_recovery(Some(RecoveryConfig::new(dir.join("recovery"))));
    h.app_mut()
        .add_document("Bracket".to_owned(), Document::new("Bracket"));
    h.drive_until_idle();
    for i in 0..5 {
        h.app_mut().submit(add_param(&format!("p{i}")));
        h.drive_until_idle();
    }
    let session = h.app().active_document().recovery.clone().expect("session");
    assert!(
        session.has_snapshot(),
        "the first dirty frame wrote a snapshot"
    );
    amg_doc::save(
        &dir.join("expected.amg"),
        &h.document().snapshot(),
        &SaveOptions::default(),
    )
    .unwrap();
    // The journal lines are written as they are committed; flushing makes the test
    // independent of the writer thread's timing (abort itself loses nothing the OS has).
    h.app().flush_recovery();
    std::process::abort();
}

#[test]
fn kill_test_recovery_dialog_offers_and_restores_the_document() {
    if let Ok(dir) = std::env::var(KILL_CHILD_ENV) {
        kill_child(Path::new(&dir));
    }
    let dir = temp_dir("kill");
    let status = std::process::Command::new(std::env::current_exe().unwrap())
        .args([KILL_TEST, "--exact", "--nocapture", "--test-threads", "1"])
        .env(KILL_CHILD_ENV, &dir)
        .status()
        .unwrap();
    assert!(!status.success(), "the child aborted: {status:?}");
    let expected = dir.join("expected.amg");
    assert!(expected.exists(), "the child got as far as the abort");

    let cfg = RecoveryConfig::new(dir.join("recovery"));
    let (mut h, _fake) = harness_with_fake();
    h.app_mut().set_recovery(Some(cfg.clone()));
    assert_eq!(h.app_mut().check_recovery(), 1, "one dead session");
    h.drive_until_idle();
    assert!(h.app().recovery_dialog_open());
    assert!(h.kittest().query_by_label("Recover unsaved work").is_some());
    assert!(
        h.kittest().query_by_label("Bracket").is_some(),
        "the dialog names the document"
    );

    h.kittest().get_by_label("Open recovered").click();
    h.drive_until_idle();
    assert!(!h.app().recovery_dialog_open());
    let tab = h.app().active_document();
    assert_eq!(tab.name, "Bracket (recovered)");
    assert!(tab.document.is_dirty(), "recovered work is unsaved");
    assert_eq!(params(&tab.document), ["p0", "p1", "p2", "p3", "p4"]);
    assert!(
        tab.recovery.is_some(),
        "the recovered document is protected again"
    );
    let recovered = dir.join("recovered.amg");
    amg_doc::save(
        &recovered,
        &tab.document.snapshot(),
        &SaveOptions::default(),
    )
    .unwrap();
    assert_eq!(design_json(&expected), design_json(&recovered));
    assert!(
        find_recoverable(&cfg).is_empty(),
        "live sessions are not offered"
    );

    // A clean shutdown leaves nothing to recover.
    h.app_mut().shutdown();
    drop(h);
    assert!(find_recoverable(&cfg).is_empty());
    let left: Vec<_> = std::fs::read_dir(&cfg.dir).unwrap().collect();
    assert!(left.is_empty(), "clean close deletes the folders: {left:?}");
    let _ = std::fs::remove_dir_all(dir);
}

/// A dead session in-process: a snapshot and journal, then the session is dropped unfinished.
fn leave_crashed_session(cfg: &RecoveryConfig, name: &str) {
    let mut doc = Document::new(name);
    let session = RecoverySession::start(cfg, doc.design().meta().uuid).unwrap();
    session.attach(&mut doc);
    session.snapshot_now(&doc).unwrap();
    doc.apply(add_param("q").commands.remove(0)).unwrap();
    session.flush();
    drop(session);
}

#[test]
fn recovery_discard_deletes_and_later_keeps_the_data() {
    let dir = temp_dir("discard");
    let cfg = RecoveryConfig::new(dir.join("recovery"));
    leave_crashed_session(&cfg, "Old");
    assert_eq!(find_recoverable(&cfg).len(), 1);

    let (mut h, _fake) = harness_with_fake();
    h.app_mut().set_recovery(Some(cfg.clone()));
    assert_eq!(h.app_mut().check_recovery(), 1);
    h.drive_until_idle();
    assert!(h.kittest().query_by_label_contains("1 edit(s)").is_some());
    h.kittest().get_by_label("Later").click();
    h.drive_until_idle();
    assert!(!h.app().recovery_dialog_open());
    assert_eq!(find_recoverable(&cfg).len(), 1, "Later keeps the data");

    assert_eq!(h.app_mut().check_recovery(), 1, "offered again");
    h.drive_until_idle();
    h.kittest().get_by_label("Discard").click();
    h.drive_until_idle();
    assert!(!h.app().recovery_dialog_open());
    assert!(find_recoverable(&cfg).is_empty(), "Discard deletes it");
    assert_eq!(h.app().documents().len(), 1);
    h.app_mut().shutdown();
    drop(h);
    let _ = std::fs::remove_dir_all(dir);
}

#[test]
fn every_commit_batch_is_journaled() {
    let dir = temp_dir("journal");
    let cfg = RecoveryConfig::new(dir.join("recovery"));
    let (mut h, _fake) = harness_with_fake();
    h.app_mut().set_recovery(Some(cfg));
    let session = h.app().active_document().recovery.clone().expect("session");
    h.app_mut().submit(CommitBatch {
        label: "Two params".to_owned(),
        commands: [add_param("a"), add_param("b")]
            .into_iter()
            .flat_map(|b| b.commands)
            .collect(),
    });
    h.drive_until_idle();
    h.app_mut().submit(add_param("c"));
    h.drive_until_idle();
    h.app().flush_recovery();
    let journal = std::fs::read_to_string(session.dir().join("journal.jsonl")).unwrap();
    // The first snapshot covers what came before it; at most the two batches remain, and no
    // batch is split into its commands.
    let lines: Vec<&str> = journal.lines().collect();
    assert!((1..=2).contains(&lines.len()), "{journal}");
    assert!(
        lines.last().unwrap().contains("\"label\":\"Add c\""),
        "{journal}"
    );
    assert!(
        !lines.iter().any(|l| l.contains("\"label\":\"Add a\"")),
        "a batch is one record, not one per command: {journal}"
    );
    h.app_mut().shutdown();
    drop(h);
    let _ = std::fs::remove_dir_all(dir);
}

// ---- Done-when 4: a 50 MB file never blocks a frame ---------------------------------------

/// A feature that carries a large payload in `design.json` and computes nothing.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct UiPayload {
    data: String,
}

impl amg_doc::Feature for UiPayload {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, _ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Ok(FeatureOutput::default())
    }
}

amg_doc::register_feature!(UiPayload, "uitest.payload", version = 1);

/// `n` pseudo-random base64 characters (6 bits each, so deflate keeps about 3/4 of them).
fn noise(n: usize) -> String {
    const ALPHABET: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut x: u64 = 0x9E37_79B9_7F4A_7C15;
    let mut out = Vec::with_capacity(n);
    for _ in 0..n {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        out.push(ALPHABET[(x >> 58) as usize]);
    }
    String::from_utf8(out).unwrap()
}

/// Records how long each `amg_ui.frame` span was entered (the UI thread's frame time).
#[derive(Clone, Default)]
struct FrameTimes(Arc<Mutex<Vec<Duration>>>);

struct Entered(Instant);

impl<S> tracing_subscriber::Layer<S> for FrameTimes
where
    S: tracing::Subscriber + for<'a> tracing_subscriber::registry::LookupSpan<'a>,
{
    fn on_enter(&self, id: &tracing::span::Id, ctx: tracing_subscriber::layer::Context<'_, S>) {
        if let Some(span) = ctx.span(id)
            && span.name() == "amg_ui.frame"
        {
            span.extensions_mut().replace(Entered(Instant::now()));
        }
    }

    fn on_exit(&self, id: &tracing::span::Id, ctx: tracing_subscriber::layer::Context<'_, S>) {
        if let Some(span) = ctx.span(id)
            && let Some(Entered(t)) = span.extensions().get::<Entered>()
        {
            self.0.lock().unwrap().push(t.elapsed());
        }
    }
}

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").as_deref() == Ok("1")
}

#[test]
fn timing_opening_a_50mb_file_never_blocks_a_frame() {
    use tracing_subscriber::layer::SubscriberExt as _;

    let dir = temp_dir("big");
    let file = dir.join("big.amg");
    let mut doc = Document::new("Big");
    let root = doc.design().root();
    doc.apply(DocCommand::AddFeature {
        at: None,
        feature: FeatureBox::new(UiPayload {
            data: noise(70 << 20),
        }),
        context: root,
        name: Some("Payload".to_owned()),
    })
    .unwrap();
    amg_doc::save(&file, &doc.snapshot(), &SaveOptions::default()).unwrap();
    drop(doc);
    let size = std::fs::metadata(&file).unwrap().len();
    assert!(size >= 50 << 20, "the file is {size} bytes");

    let times = FrameTimes::default();
    let subscriber = tracing_subscriber::registry().with(times.clone());
    let _guard = tracing::subscriber::set_default(subscriber);

    let (mut h, fake) = harness_with_fake();
    h.app_mut()
        .set_recovery(Some(RecoveryConfig::new(dir.join("recovery"))));
    times.0.lock().unwrap().clear();
    fake.answer(Some(file.clone()));
    let start = Instant::now();
    h.app_mut().request_command(CmdId("file.open"));
    // Step like an event loop that repaints continuously, until the document is open and
    // evaluated and nothing is pending.
    let mut frames = 0;
    loop {
        h.kittest().step();
        frames += 1;
        let app = h.app();
        let open = app.documents().len() == 2;
        let idle = !app.file_busy()
            && app.jobs().jobs().is_empty()
            && app.documents().iter().all(|d| {
                matches!(
                    d.document.recompute_status(),
                    amg_doc::RecomputeStatus::Idle
                )
            });
        if open && idle {
            break;
        }
        assert!(
            start.elapsed() < Duration::from_secs(120),
            "open took too long"
        );
        std::thread::sleep(Duration::from_millis(2));
    }
    let total = start.elapsed();
    assert_eq!(h.app().active_document().name, "big");
    assert_eq!(h.document().design().timeline().nodes().count(), 1);

    let mut t = times.0.lock().unwrap().clone();
    t.sort();
    let max = *t.last().unwrap();
    let p50 = t[t.len() / 2];
    println!(
        "open {size} B in {total:?}: {frames} frames, UI-thread frame p50 {p50:?}, max {max:?}"
    );
    assert!(
        frames > 3,
        "the open took several frames, not one blocking one"
    );
    if strict_timing() {
        assert!(
            max < Duration::from_millis(16),
            "a frame took {max:?} while opening"
        );
    }
    h.app_mut().shutdown();
    drop(h);
    let _ = std::fs::remove_dir_all(dir);
}
