//! DOC-14: autosave journal, recovery snapshots, lock, kill-and-recover.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    missing_docs,
    reason = "tests"
)]

use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command as Process;
use std::sync::atomic::{AtomicU32, Ordering};
use std::time::{Duration, Instant};

use amg_doc::testing::TestCounter;
use amg_doc::{
    Command, Dependency, Document, FeatureBox, RecoveryConfig, RecoverySession, find_recoverable,
    recover, recover_with_report,
};
use amg_jobs::{JobSystem, JobSystemConfig};

struct TempDir(PathBuf);

impl TempDir {
    fn new() -> TempDir {
        static N: AtomicU32 = AtomicU32::new(0);
        let p = std::env::temp_dir().join(format!(
            "amg-doc-recovery-{}-{}",
            std::process::id(),
            N.fetch_add(1, Ordering::Relaxed)
        ));
        fs::create_dir_all(&p).unwrap();
        TempDir(p)
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}

fn cfg(dir: &Path) -> RecoveryConfig {
    RecoveryConfig {
        dir: dir.to_path_buf(),
        snapshot_interval: Duration::ZERO,
    }
}

fn design_json(doc: &Document) -> Vec<u8> {
    serde_json::to_vec_pretty(doc.design()).unwrap()
}

fn add_param(doc: &mut Document, i: usize) {
    doc.apply(Command::AddParam {
        name: format!("p{i}"),
        expr: format!("{i}"),
        unit: None,
        comment: String::new(),
    })
    .unwrap();
}

fn add_counter(doc: &mut Document, i: i64) {
    let root = doc.design().root();
    doc.apply(Command::AddFeature {
        at: None,
        feature: FeatureBox::new(TestCounter {
            deps: Vec::<Dependency>::new(),
            value: i,
            fail: false,
        }),
        context: root,
        name: None,
    })
    .unwrap();
}

/// The commits of the kill test: `n` commands incl. a two-command transaction, an undo and a redo.
fn script(doc: &mut Document, range: std::ops::Range<usize>) {
    for i in range {
        match i % 5 {
            0 => add_counter(doc, i as i64),
            3 => {
                let mut tx = doc.transaction("two params");
                for k in 0..2 {
                    tx.apply(Command::AddParam {
                        name: format!("t{i}_{k}"),
                        expr: "1".to_owned(),
                        unit: None,
                        comment: String::new(),
                    })
                    .unwrap();
                }
                tx.commit();
            }
            4 => {
                assert!(doc.undo());
                assert!(doc.redo());
            }
            _ => add_param(doc, i),
        }
    }
}

fn snapshot_files(dir: &Path) -> Vec<String> {
    let mut v: Vec<String> = fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
        .filter(|n| n.starts_with("snapshot-") && n.ends_with(".amg"))
        .collect();
    v.sort();
    v
}

fn journal_lines(dir: &Path) -> usize {
    fs::read_to_string(dir.join("journal.jsonl"))
        .unwrap()
        .lines()
        .count()
}

// ---- Done-when 2: kill test ----------------------------------------------------------------

/// Child half of [`killed_process_is_recovered_to_the_precrash_design`].
#[test]
fn kill_child() {
    let (Some(dir), Some(out)) = (
        std::env::var_os("AMG_REC_DIR"),
        std::env::var_os("AMG_REC_OUT"),
    ) else {
        return; // not the child
    };
    let mut doc = Document::new("Dying");
    let session = RecoverySession::start(&cfg(Path::new(&dir)), doc.design().meta().uuid).unwrap();
    session.attach(&mut doc);
    // A snapshot in the middle, so recovery combines snapshot and journal tail.
    script(&mut doc, 0..10);
    session.snapshot_now(&doc).unwrap();
    script(&mut doc, 10..20);
    session.flush();
    fs::write(out, design_json(&doc)).unwrap();
    std::process::abort();
}

#[test]
fn killed_process_is_recovered_to_the_precrash_design() {
    let tmp = TempDir::new();
    let rec = tmp.0.join("recovery");
    let out = tmp.0.join("expected.json");
    let status = Process::new(std::env::current_exe().unwrap())
        .args(["--exact", "kill_child", "--nocapture", "--test-threads=1"])
        .env("AMG_REC_DIR", &rec)
        .env("AMG_REC_OUT", &out)
        .status()
        .unwrap();
    assert!(!status.success(), "the child must have died");
    let expected = fs::read(&out).unwrap();

    let found = find_recoverable(&cfg(&rec));
    assert_eq!(found.len(), 1, "the dead session's folder is offered");
    let c = &found[0];
    assert!(c.snapshot_seq > 0);
    let r = recover_with_report(c).unwrap();
    assert!(r.stopped.is_none(), "{:?}", r.stopped);
    assert!(r.replayed > 0);
    assert_eq!(design_json(&r.document), expected);
    assert!(r.document.is_dirty(), "a recovered document is unsaved");
    // The journal alone (no snapshot yet in the second half) also works for `recover`.
    assert_eq!(design_json(&recover(c).unwrap()), expected);
    c.discard().unwrap();
    assert!(find_recoverable(&cfg(&rec)).is_empty());
}

// ---- Done-when 3: lock ---------------------------------------------------------------------

#[test]
fn a_held_lock_is_not_offered_and_a_released_one_is() {
    let tmp = TempDir::new();
    let config = cfg(&tmp.0);
    let mut doc = Document::new("Locked");
    let uuid = doc.design().meta().uuid;
    let session = RecoverySession::start(&config, uuid).unwrap();
    session.attach(&mut doc);
    add_param(&mut doc, 1);
    session.snapshot_now(&doc).unwrap();
    session.flush();

    assert!(find_recoverable(&config).is_empty(), "live session");
    let second = RecoverySession::start(&config, uuid).unwrap_err();
    assert_eq!(second.kind(), std::io::ErrorKind::WouldBlock);
    // Starting a second session must not have damaged the first one's files.
    assert_eq!(snapshot_files(session.dir()).len(), 1);

    drop(session); // a crash looks like this to the next start: lock gone, folder kept
    let found = find_recoverable(&config);
    assert_eq!(found.len(), 1);
    assert_eq!(found[0].doc_uuid, Some(uuid));
    assert_eq!(design_json(&recover(&found[0]).unwrap()), design_json(&doc));
}

#[test]
fn a_candidate_cannot_be_discarded_while_its_session_lives() {
    let tmp = TempDir::new();
    let config = cfg(&tmp.0);
    let mut doc = Document::new("Locked");
    let session = RecoverySession::start(&config, doc.design().meta().uuid).unwrap();
    session.attach(&mut doc);
    session.snapshot_now(&doc).unwrap();
    let dir = session.dir().to_path_buf();
    drop(session);
    let c = find_recoverable(&config).remove(0);
    let again = RecoverySession::start(&config, doc.design().meta().uuid).unwrap();
    assert_eq!(
        c.discard().unwrap_err().kind(),
        std::io::ErrorKind::WouldBlock
    );
    assert!(dir.exists());
    again.finish().unwrap();
}

// ---- journal, snapshots, lifecycle ---------------------------------------------------------

#[test]
fn journal_has_one_line_per_commit_undo_and_redo_and_flush_waits_for_it() {
    let tmp = TempDir::new();
    let mut doc = Document::new("J");
    let session = RecoverySession::start(&cfg(&tmp.0), doc.design().meta().uuid).unwrap();
    session.attach(&mut doc);
    add_param(&mut doc, 1);
    add_param(&mut doc, 2);
    assert!(doc.undo());
    assert!(doc.redo());
    let mut tx = doc.transaction("two");
    tx.apply(Command::AddParam {
        name: "a".into(),
        expr: "1".into(),
        unit: None,
        comment: String::new(),
    })
    .unwrap();
    tx.apply(Command::AddParam {
        name: "b".into(),
        expr: "2".into(),
        unit: None,
        comment: String::new(),
    })
    .unwrap();
    tx.commit();
    session.flush();
    assert_eq!(
        journal_lines(session.dir()),
        5,
        "2 applies, undo, redo, 1 tx"
    );
}

#[test]
fn snapshot_drops_the_journal_it_contains_and_older_snapshots() {
    let tmp = TempDir::new();
    let mut doc = Document::new("S");
    let session = RecoverySession::start(&cfg(&tmp.0), doc.design().meta().uuid).unwrap();
    session.attach(&mut doc);
    for i in 0..4 {
        add_param(&mut doc, i);
    }
    session.snapshot_now(&doc).unwrap();
    session.flush(); // queued after the compaction request
    assert_eq!(journal_lines(session.dir()), 0);
    add_param(&mut doc, 9);
    session.snapshot_now(&doc).unwrap();
    add_param(&mut doc, 10);
    session.flush();
    assert_eq!(
        snapshot_files(session.dir()).len(),
        1,
        "older snapshot removed"
    );
    assert_eq!(journal_lines(session.dir()), 1);
    drop(session);
    let c = find_recoverable(&cfg(&tmp.0)).remove(0);
    assert_eq!(design_json(&recover(&c).unwrap()), design_json(&doc));
}

#[test]
fn tick_writes_the_first_snapshot_then_periodic_ones_as_jobs() {
    let tmp = TempDir::new();
    let jobs = JobSystem::new(JobSystemConfig::default()).unwrap();
    let mut doc = Document::new("T");
    let session = RecoverySession::start(&cfg(&tmp.0), doc.design().meta().uuid).unwrap();
    session.attach(&mut doc);
    assert!(!session.has_snapshot());
    let wait = |doc: &Document, want: usize| {
        let t = Instant::now();
        loop {
            session.tick(doc, &jobs);
            if session.has_snapshot() && snapshot_files(session.dir()).len() == 1 {
                // The newest snapshot must be the current state.
                let name = snapshot_files(session.dir()).remove(0);
                let l = amg_doc::load(&session.dir().join(name)).unwrap();
                if serde_json::to_vec_pretty(&l.design).unwrap() == design_json(doc) {
                    return;
                }
            }
            assert!(t.elapsed() < Duration::from_secs(10), "snapshot {want}");
            std::thread::sleep(Duration::from_millis(5));
        }
    };
    wait(&doc, 1);
    add_param(&mut doc, 1);
    wait(&doc, 2);
}

#[test]
fn undo_past_the_snapshot_forces_a_new_snapshot_and_recovery_matches() {
    let tmp = TempDir::new();
    let jobs = JobSystem::new(JobSystemConfig::default()).unwrap();
    let mut doc = Document::new("U");
    let session = RecoverySession::start(&cfg(&tmp.0), doc.design().meta().uuid).unwrap();
    session.attach(&mut doc);
    for i in 0..3 {
        add_param(&mut doc, i);
    }
    session.snapshot_now(&doc).unwrap();
    // These undos reach entries older than the snapshot: they cannot be journaled.
    assert!(doc.undo());
    assert!(doc.undo());
    let t = Instant::now();
    loop {
        session.tick(&doc, &jobs);
        let name = snapshot_files(session.dir()).pop().unwrap();
        let l = amg_doc::load(&session.dir().join(name)).unwrap();
        if serde_json::to_vec_pretty(&l.design).unwrap() == design_json(&doc) {
            break;
        }
        assert!(t.elapsed() < Duration::from_secs(10));
        std::thread::sleep(Duration::from_millis(5));
    }
    session.flush();
    drop(session);
    let c = find_recoverable(&cfg(&tmp.0)).remove(0);
    assert_eq!(design_json(&recover(&c).unwrap()), design_json(&doc));
}

#[test]
fn a_torn_last_journal_line_stops_replay_without_failing() {
    let tmp = TempDir::new();
    let mut doc = Document::new("Torn");
    let session = RecoverySession::start(&cfg(&tmp.0), doc.design().meta().uuid).unwrap();
    session.attach(&mut doc);
    session.snapshot_now(&doc).unwrap();
    add_param(&mut doc, 1);
    add_param(&mut doc, 2);
    let expected_two = design_json(&doc);
    session.flush();
    let journal = session.dir().join("journal.jsonl");
    drop(session);
    let mut text = fs::read_to_string(&journal).unwrap();
    text.push_str("{\"seq\":3,\"op\":\"apply\",\"label\":\"Add param"); // killed mid-write
    fs::write(&journal, text).unwrap();

    let c = find_recoverable(&cfg(&tmp.0)).remove(0);
    let r = recover_with_report(&c).unwrap();
    assert_eq!(r.replayed, 2);
    assert!(r.stopped.is_some());
    assert_eq!(design_json(&r.document), expected_two);
}

#[test]
fn clean_close_deletes_the_folder_and_ignores_later_edits() {
    let tmp = TempDir::new();
    let mut doc = Document::new("Close");
    let session = RecoverySession::start(&cfg(&tmp.0), doc.design().meta().uuid).unwrap();
    session.attach(&mut doc);
    session.snapshot_now(&doc).unwrap();
    add_param(&mut doc, 1);
    let dir = session.dir().to_path_buf();
    session.finish().unwrap();
    assert!(!dir.exists());
    add_param(&mut doc, 2); // observer still attached; must not recreate anything
    assert!(find_recoverable(&cfg(&tmp.0)).is_empty());
    assert!(!dir.exists());
}

#[test]
fn panic_hook_registration_flushes_and_reports_the_folder() {
    let tmp = TempDir::new();
    let mut doc = Document::new("Hook");
    let session = RecoverySession::start(&cfg(&tmp.0), doc.design().meta().uuid).unwrap();
    session.attach(&mut doc);
    add_param(&mut doc, 1);
    session.register_panic_hook("amg-doc-test");
    assert!(amg_jobs::diag::unregister_recovery_hook("amg-doc-test"));
    session.flush();
    assert_eq!(journal_lines(session.dir()), 1);
}

#[test]
fn folders_without_a_snapshot_are_not_offered() {
    let tmp = TempDir::new();
    let mut doc = Document::new("NoSnap");
    let session = RecoverySession::start(&cfg(&tmp.0), doc.design().meta().uuid).unwrap();
    session.attach(&mut doc);
    add_param(&mut doc, 1);
    drop(session);
    assert!(find_recoverable(&cfg(&tmp.0)).is_empty());
}

#[test]
fn entering_direct_mode_snapshots_instead_of_journaling() {
    let tmp = TempDir::new();
    let jobs = JobSystem::new(JobSystemConfig::default()).unwrap();
    let mut doc = Document::new("D");
    let session = RecoverySession::start(&cfg(&tmp.0), doc.design().meta().uuid).unwrap();
    session.attach(&mut doc);
    add_counter(&mut doc, 1);
    add_counter(&mut doc, 2);
    doc.recompute_blocking(&jobs).unwrap();
    session.flush();
    let lines = journal_lines(session.dir());
    // The bake needs evaluated geometry, so replay could not reproduce it from the journal.
    doc.apply(Command::SetDesignMode {
        mode: amg_doc::DesignMode::Direct,
    })
    .unwrap();
    session.flush();
    assert_eq!(
        journal_lines(session.dir()),
        lines,
        "bake must not be journaled"
    );
    let t = Instant::now();
    loop {
        session.tick(&doc, &jobs);
        if let Some(name) = snapshot_files(session.dir()).pop() {
            let l = amg_doc::load(&session.dir().join(name)).unwrap();
            if serde_json::to_vec_pretty(&l.design).unwrap() == design_json(&doc) {
                break;
            }
        }
        assert!(t.elapsed() < Duration::from_secs(10));
        std::thread::sleep(Duration::from_millis(5));
    }
    // Undoing the bake reaches behind the new snapshot: it must snapshot too, not journal.
    assert!(doc.undo());
    session.flush();
    assert_eq!(
        journal_lines(session.dir()),
        0,
        "snapshot dropped the journal"
    );
    session.snapshot_now(&doc).unwrap();
    session.flush();
    drop(session);
    let c = find_recoverable(&cfg(&tmp.0)).remove(0);
    assert_eq!(design_json(&recover(&c).unwrap()), design_json(&doc));
}
