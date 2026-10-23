//! Autosave and crash recovery (S01 §8, DOC-14).
//!
//! # Layout
//! One folder per open document, `<recovery dir>/<design uuid>/`:
//!
//! | File | Content |
//! |---|---|
//! | `lock` | Held open with `File::try_lock` while the session lives. |
//! | `journal.jsonl` | One JSON line per committed undo entry, undo and redo: `{seq, op, ...}`. |
//! | `snapshot-<seq>.amg` | Compact `.amg` of the design after record `seq` (no caches). |
//!
//! The snapshot's sequence number is in its name (the spec says `snapshot.amg`): a snapshot and
//! "which journal records it already contains" are therefore one atomic fact, and a crash between
//! writing the snapshot and truncating the journal cannot replay records twice.
//!
//! # Flow
//! - [`RecoverySession::start`] locks the folder and starts the writer thread.
//! - [`RecoverySession::attach`] makes a [`Document`]'s history report to the session
//!   ([`HistoryObserver`]); each commit serializes its commands on the UI thread and hands the
//!   line to the writer thread, which appends it (no fsync per line).
//! - [`RecoverySession::tick`] (once per frame, next to `Document::poll`) starts a snapshot job
//!   when none exists yet, when an undo or redo reached back past the last snapshot (the journal
//!   cannot replay that), or every `snapshot_interval` when records were added. A finished
//!   snapshot deletes older ones and drops the journal records it contains.
//! - [`find_recoverable`] lists folders whose lock can be taken (dead sessions); [`recover`] loads
//!   the newest snapshot and replays the journal after it.
//! - [`RecoverySession::finish`] (clean close) deletes the folder.
//! - [`RecoverySession::register_panic_hook`] installs the FND-07 hook that syncs the journal.
//!
//! `amg-doc` never picks the directory; the app passes it in.

use std::fs::{self, File, OpenOptions};
use std::io::Write as _;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant, SystemTime};

use amg_core::PersistentId;
use amg_jobs::{JobHandle, JobSystem, SpawnOptions};
use serde::{Deserialize, Serialize};

use crate::command::Command;
use crate::document::Document;
use crate::history::HistoryObserver;
use crate::io::{FormatError, SaveOptions, load, save};
use crate::model::DesignSnapshot;

/// Default snapshot interval (S01 §8: 5 minutes, a preference).
pub const DEFAULT_SNAPSHOT_INTERVAL: Duration = Duration::from_secs(5 * 60);

const LOCK_FILE: &str = "lock";
const JOURNAL_FILE: &str = "journal.jsonl";
const SNAPSHOT_PREFIX: &str = "snapshot-";
const SNAPSHOT_SUFFIX: &str = ".amg";

/// Where recovery data lives and how often snapshots are taken.
#[derive(Clone, Debug)]
pub struct RecoveryConfig {
    /// Parent of the per-document folders.
    pub dir: PathBuf,
    /// Time between snapshots of a document with new journal records.
    pub snapshot_interval: Duration,
}

impl RecoveryConfig {
    /// `dir` with the default interval.
    pub fn new(dir: impl Into<PathBuf>) -> RecoveryConfig {
        RecoveryConfig {
            dir: dir.into(),
            snapshot_interval: DEFAULT_SNAPSHOT_INTERVAL,
        }
    }
}

/// One journal line.
#[derive(Serialize, Deserialize)]
struct Record {
    seq: u64,
    #[serde(flatten)]
    op: Op,
}

#[derive(Serialize, Deserialize)]
#[serde(tag = "op", rename_all = "lowercase")]
enum Op {
    Apply { label: String, cmds: Vec<Command> },
    Undo,
    Redo,
}

/// Only the sequence number of a line (compaction does not need the rest).
#[derive(Deserialize)]
struct SeqOnly {
    seq: u64,
}

enum Msg {
    Append(String),
    Flush(Sender<()>),
    /// Drop records with `seq <= n`.
    Compact(u64),
    Shutdown,
}

struct State {
    /// Sequence number of the last journaled record.
    seq: u64,
    /// Entries a replay from the latest snapshot could undo / redo.
    undo_depth: usize,
    redo_depth: usize,
    /// A snapshot is needed now (undo/redo reached past the last one, or a job failed).
    force: bool,
    /// A snapshot job finished successfully at least once.
    has_snapshot: bool,
    /// `seq` the latest started snapshot covers.
    snapshot_seq: u64,
    last_snapshot: Instant,
    job: Option<JobHandle<(), FormatError>>,
    last_error: Option<String>,
    closed: bool,
}

/// The recovery state of one open document: lock, journal writer and snapshot scheduling.
pub struct RecoverySession {
    dir: PathBuf,
    interval: Duration,
    state: Mutex<State>,
    tx: Mutex<Option<Sender<Msg>>>,
    writer: Mutex<Option<JoinHandle<()>>>,
    lock: Mutex<Option<File>>,
    journal_failed: Arc<AtomicBool>,
}

impl std::fmt::Debug for RecoverySession {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RecoverySession")
            .field("dir", &self.dir)
            .finish_non_exhaustive()
    }
}

fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

fn snapshot_name(seq: u64) -> String {
    format!("{SNAPSHOT_PREFIX}{seq:020}{SNAPSHOT_SUFFIX}")
}

fn parse_snapshot_seq(name: &str) -> Option<u64> {
    name.strip_prefix(SNAPSHOT_PREFIX)?
        .strip_suffix(SNAPSHOT_SUFFIX)?
        .parse()
        .ok()
}

/// Snapshot files in `dir`, as `(seq, path)`, oldest first.
fn snapshots(dir: &Path) -> Vec<(u64, PathBuf)> {
    let mut v: Vec<_> = fs::read_dir(dir)
        .into_iter()
        .flatten()
        .flatten()
        .filter_map(|e| {
            let seq = parse_snapshot_seq(e.file_name().to_str()?)?;
            Some((seq, e.path()))
        })
        .collect();
    v.sort();
    v
}

impl RecoverySession {
    /// Creates `<cfg.dir>/<doc_uuid>/`, takes its lock and starts the journal writer.
    ///
    /// Leftovers of an earlier session of the same document are discarded: an app lists
    /// [`find_recoverable`] first and lets the user decide. Fails with
    /// `ErrorKind::WouldBlock` if another live session holds the folder.
    pub fn start(cfg: &RecoveryConfig, doc_uuid: PersistentId) -> std::io::Result<Arc<Self>> {
        let dir = cfg.dir.join(doc_uuid.to_string());
        fs::create_dir_all(&dir)?;
        let lock_file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .open(dir.join(LOCK_FILE))?;
        lock_file.try_lock().map_err(|e| match e {
            fs::TryLockError::WouldBlock => std::io::Error::new(
                std::io::ErrorKind::WouldBlock,
                "recovery folder is in use by another session",
            ),
            fs::TryLockError::Error(e) => e,
        })?;
        for (_, p) in snapshots(&dir) {
            let _ = fs::remove_file(p);
        }
        let journal_path = dir.join(JOURNAL_FILE);
        let journal = OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(true)
            .open(&journal_path)?;

        let (tx, rx) = mpsc::channel();
        let failed = Arc::new(AtomicBool::new(false));
        let writer = {
            let failed = Arc::clone(&failed);
            std::thread::Builder::new()
                .name("amg-journal".to_owned())
                .spawn(move || writer_loop(&rx, journal, &journal_path, &failed))?
        };
        Ok(Arc::new(RecoverySession {
            dir,
            interval: cfg.snapshot_interval,
            state: Mutex::new(State {
                seq: 0,
                undo_depth: 0,
                redo_depth: 0,
                force: false,
                has_snapshot: false,
                snapshot_seq: 0,
                last_snapshot: Instant::now(),
                job: None,
                last_error: None,
                closed: false,
            }),
            tx: Mutex::new(Some(tx)),
            writer: Mutex::new(Some(writer)),
            lock: Mutex::new(Some(lock_file)),
            journal_failed: failed,
        }))
    }

    /// The folder holding this document's recovery data.
    pub fn dir(&self) -> &Path {
        &self.dir
    }

    /// Makes `doc`'s undo history report commits, undo and redo to this session. Call
    /// [`RecoverySession::tick`] each frame so the first snapshot gets written.
    pub fn attach(self: &Arc<Self>, doc: &mut Document) {
        doc.history_mut()
            .set_observer(Some(Arc::new(WeakObserver(Arc::downgrade(self)))));
    }

    fn send(&self, msg: Msg) {
        if let Some(tx) = lock(&self.tx).as_ref() {
            let _ = tx.send(msg);
        }
    }

    fn journal(&self, op: Op, st: &mut State) {
        st.seq += 1;
        let rec = Record { seq: st.seq, op };
        if let Ok(line) = serde_json::to_string(&rec) {
            self.send(Msg::Append(line));
        } else {
            st.force = true; // cannot journal it; a snapshot will cover it
        }
    }

    /// Per-frame housekeeping: collects a finished snapshot job and starts the next one when due
    /// (see the module docs). Never blocks.
    pub fn tick(&self, doc: &Document, jobs: &JobSystem) {
        let mut st = lock(&self.state);
        if st.closed {
            return;
        }
        self.collect(&mut st);
        if st.job.is_some() {
            return;
        }
        let due = !st.has_snapshot
            || st.force
            || (st.seq > st.snapshot_seq && st.last_snapshot.elapsed() >= self.interval);
        if !due {
            return;
        }
        let snap = doc.snapshot();
        let seq = st.seq;
        let (dir, tx) = (self.dir.clone(), lock(&self.tx).clone());
        st.job = Some(jobs.spawn_fn(
            "Autosave",
            SpawnOptions::default(),
            move |_ctx: &amg_jobs::JobCtx| write_snapshot(&dir, seq, &snap, tx.as_ref()),
        ));
        st.snapshot_seq = seq;
        st.undo_depth = 0;
        st.redo_depth = 0;
        st.force = false;
        st.last_snapshot = Instant::now();
    }

    fn collect(&self, st: &mut State) {
        let Some(job) = st.job.as_mut() else { return };
        let Some(result) = job.try_take() else { return };
        st.job = None;
        match result {
            Ok(()) => st.has_snapshot = true,
            Err(e) => {
                st.force = true;
                st.last_error = Some(e.to_string());
            }
        }
    }

    /// Writes a snapshot of `doc` now and waits for it (tests, and "save before risky op").
    pub fn snapshot_now(&self, doc: &Document) -> Result<(), FormatError> {
        let (seq, tx) = {
            let mut st = lock(&self.state);
            st.snapshot_seq = st.seq;
            st.undo_depth = 0;
            st.redo_depth = 0;
            st.force = false;
            st.last_snapshot = Instant::now();
            (st.seq, lock(&self.tx).clone())
        };
        write_snapshot(&self.dir, seq, &doc.snapshot(), tx.as_ref())?;
        lock(&self.state).has_snapshot = true;
        Ok(())
    }

    /// True once a snapshot exists, so a crash is recoverable.
    pub fn has_snapshot(&self) -> bool {
        let mut st = lock(&self.state);
        self.collect(&mut st);
        st.has_snapshot
    }

    /// The last snapshot or journal error, for the status bar.
    pub fn last_error(&self) -> Option<String> {
        let st = lock(&self.state);
        st.last_error.clone().or_else(|| {
            self.journal_failed
                .load(Ordering::Relaxed)
                .then(|| "writing the recovery journal failed".to_owned())
        })
    }

    /// Waits (up to 5 s) until every queued record is on disk (`fsync`). The FND-07 panic hook
    /// calls this; the journal is already written, so it only forces it out of the OS cache.
    pub fn flush(&self) {
        let Some(tx) = self.tx.try_lock().ok().and_then(|g| g.clone()) else {
            return;
        };
        let (ack, done) = mpsc::channel();
        if tx.send(Msg::Flush(ack)).is_ok() {
            let _ = done.recv_timeout(Duration::from_secs(5));
        }
    }

    /// Registers this session as recovery hook `name` of the FND-07 panic hook. The hook reports
    /// the recovery folder as "the recovery file".
    pub fn register_panic_hook(self: &Arc<Self>, name: &'static str) {
        let s = Arc::clone(self);
        amg_jobs::diag::register_recovery_hook(
            name,
            Box::new(move || {
                s.flush();
                Some(s.dir.clone())
            }),
        );
    }

    /// Clean close: stops the writer, releases the lock and deletes the folder.
    pub fn finish(&self) -> std::io::Result<()> {
        {
            let mut st = lock(&self.state);
            st.closed = true;
            if let Some(j) = st.job.take() {
                j.cancel();
                let _ = j.wait_blocking();
            }
        }
        self.stop_writer();
        drop(lock(&self.lock).take());
        match fs::remove_dir_all(&self.dir) {
            Err(e) if e.kind() != std::io::ErrorKind::NotFound => Err(e),
            _ => Ok(()),
        }
    }

    fn stop_writer(&self) {
        if let Some(tx) = lock(&self.tx).take() {
            let _ = tx.send(Msg::Shutdown);
        }
        if let Some(h) = lock(&self.writer).take() {
            let _ = h.join();
        }
    }
}

impl Drop for RecoverySession {
    /// Stops the writer and releases the lock but keeps the folder: dropping without
    /// [`RecoverySession::finish`] is what a crash looks like to the next start.
    fn drop(&mut self) {
        self.stop_writer();
    }
}

/// What the history holds: a weak handle, so dropping the session (a crash, in tests) releases
/// the lock even while the document lives on.
struct WeakObserver(std::sync::Weak<RecoverySession>);

impl HistoryObserver for WeakObserver {
    fn on_commit(&self, label: &str, commands: &[Command]) {
        if let Some(s) = self.0.upgrade() {
            s.on_commit(label, commands);
        }
    }
    fn on_undo(&self) {
        if let Some(s) = self.0.upgrade() {
            s.on_undo();
        }
    }
    fn on_redo(&self) {
        if let Some(s) = self.0.upgrade() {
            s.on_redo();
        }
    }
}

impl RecoverySession {
    fn on_commit(&self, label: &str, commands: &[Command]) {
        let mut st = lock(&self.state);
        if st.closed {
            return;
        }
        st.undo_depth += 1;
        st.redo_depth = 0;
        self.journal(
            Op::Apply {
                label: label.to_owned(),
                cmds: commands.to_vec(),
            },
            &mut st,
        );
    }

    fn on_undo(&self) {
        let mut st = lock(&self.state);
        if st.closed {
            return;
        }
        if st.undo_depth > 0 {
            st.undo_depth -= 1;
            st.redo_depth += 1;
            self.journal(Op::Undo, &mut st);
        } else {
            // The entry undone predates the snapshot: replay could not follow. Snapshot instead.
            st.force = true;
        }
    }

    fn on_redo(&self) {
        let mut st = lock(&self.state);
        if st.closed {
            return;
        }
        if st.redo_depth > 0 {
            st.redo_depth -= 1;
            st.undo_depth += 1;
            self.journal(Op::Redo, &mut st);
        } else {
            st.force = true;
        }
    }
}

/// Writes `snapshot-<seq>.amg` (compact), deletes older ones and asks the writer to drop the
/// journal records it contains.
fn write_snapshot(
    dir: &Path,
    seq: u64,
    snap: &DesignSnapshot,
    tx: Option<&Sender<Msg>>,
) -> Result<(), FormatError> {
    let path = dir.join(snapshot_name(seq));
    let opts = SaveOptions {
        keep_backup: false,
        ..SaveOptions::default()
    };
    save(&path, snap, &opts)?;
    for (s, p) in snapshots(dir) {
        if s < seq {
            let _ = fs::remove_file(p);
        }
    }
    if let Some(tx) = tx {
        let _ = tx.send(Msg::Compact(seq));
    }
    Ok(())
}

fn writer_loop(rx: &Receiver<Msg>, mut file: File, path: &Path, failed: &AtomicBool) {
    while let Ok(msg) = rx.recv() {
        match msg {
            Msg::Append(mut line) => {
                line.push('\n');
                if file.write_all(line.as_bytes()).is_err() {
                    failed.store(true, Ordering::Relaxed);
                }
            }
            Msg::Flush(ack) => {
                let _ = file.sync_data();
                let _ = ack.send(());
            }
            Msg::Compact(upto) => match compact(path, upto) {
                Ok(f) => file = f,
                Err(_) => failed.store(true, Ordering::Relaxed),
            },
            Msg::Shutdown => {
                let _ = file.sync_data();
                return;
            }
        }
    }
}

/// Rewrites the journal without records `<= upto` (tmp + rename) and returns a new append handle.
fn compact(path: &Path, upto: u64) -> std::io::Result<File> {
    let text = fs::read(path)?;
    let tmp = path.with_extension("jsonl.tmp");
    {
        let mut out = File::create(&tmp)?;
        for line in text.split(|b| *b == b'\n').filter(|l| !l.is_empty()) {
            let keep = serde_json::from_slice::<SeqOnly>(line).is_ok_and(|r| r.seq > upto);
            if keep {
                out.write_all(line)?;
                out.write_all(b"\n")?;
            }
        }
        out.sync_data()?;
    }
    fs::rename(&tmp, path)?;
    OpenOptions::new().append(true).open(path)
}

/// A recovery folder left by a session that is no longer running.
#[derive(Clone, Debug)]
pub struct RecoveryCandidate {
    /// The folder.
    pub dir: PathBuf,
    /// Document identity (the folder name), if it parses.
    pub doc_uuid: Option<PersistentId>,
    /// Newest snapshot.
    pub snapshot: PathBuf,
    /// Journal records with `seq <=` this are already in the snapshot.
    pub snapshot_seq: u64,
    /// When the folder was last written.
    pub modified: SystemTime,
}

impl RecoveryCandidate {
    /// Deletes the folder (the user declined the restore, or it was restored and saved).
    /// Fails with `WouldBlock` if a live session holds it.
    pub fn discard(&self) -> std::io::Result<()> {
        let f = OpenOptions::new()
            .read(true)
            .write(true)
            .open(self.dir.join(LOCK_FILE))?;
        f.try_lock().map_err(|_| {
            std::io::Error::new(std::io::ErrorKind::WouldBlock, "recovery folder in use")
        })?;
        drop(f);
        fs::remove_dir_all(&self.dir)
    }
}

/// Recovery folders under `cfg.dir` that belong to dead sessions (their lock can be taken) and
/// hold a snapshot, newest first.
pub fn find_recoverable(cfg: &RecoveryConfig) -> Vec<RecoveryCandidate> {
    let mut found: Vec<RecoveryCandidate> = fs::read_dir(&cfg.dir)
        .into_iter()
        .flatten()
        .flatten()
        .filter_map(|e| candidate(&e.path()))
        .collect();
    found.sort_by_key(|c| std::cmp::Reverse(c.modified));
    found
}

fn candidate(dir: &Path) -> Option<RecoveryCandidate> {
    let lock_file = OpenOptions::new()
        .read(true)
        .write(true)
        .open(dir.join(LOCK_FILE))
        .ok()?;
    // Held: a live session. (Dropping the handle releases our probe lock.)
    lock_file.try_lock().ok()?;
    drop(lock_file);
    let (snapshot_seq, snapshot) = snapshots(dir).pop()?;
    let modified = [dir.join(JOURNAL_FILE), snapshot.clone()]
        .iter()
        .filter_map(|p| fs::metadata(p).ok()?.modified().ok())
        .max()?;
    Some(RecoveryCandidate {
        dir: dir.to_path_buf(),
        doc_uuid: dir
            .file_name()
            .and_then(|n| n.to_str())
            .and_then(|n| n.parse().ok()),
        snapshot,
        snapshot_seq,
        modified,
    })
}

/// A recovered document and what happened while replaying.
#[derive(Debug)]
pub struct Recovered {
    /// The document: snapshot plus replayed journal, marked unsaved.
    pub document: Document,
    /// Journal records applied.
    pub replayed: usize,
    /// Why replay stopped before the end of the journal (torn last line, a command that no
    /// longer applies), if it did.
    pub stopped: Option<String>,
}

/// Loads the newest snapshot of `c` and replays the journal after it through the normal
/// `Document` paths. The folder is left alone; call [`RecoveryCandidate::discard`] when done.
pub fn recover(c: &RecoveryCandidate) -> Result<Document, FormatError> {
    recover_with_report(c).map(|r| r.document)
}

/// [`recover`] with the replay report.
pub fn recover_with_report(c: &RecoveryCandidate) -> Result<Recovered, FormatError> {
    let loaded = load(&c.snapshot)?;
    let mut doc = Document::from_design(loaded.design);
    let journal = match fs::read(c.dir.join(JOURNAL_FILE)) {
        Ok(b) => b,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Vec::new(),
        Err(e) => return Err(e.into()),
    };
    let mut replayed = 0;
    let mut stopped = None;
    for line in journal.split(|b| *b == b'\n').filter(|l| !l.is_empty()) {
        let rec: Record = match serde_json::from_slice(line) {
            Ok(r) => r,
            Err(e) => {
                // A torn last line is the normal sign of a kill; anything after it is untrusted.
                stopped = Some(format!("journal line unreadable: {e}"));
                break;
            }
        };
        if rec.seq <= c.snapshot_seq {
            continue;
        }
        if let Err(msg) = replay(&mut doc, rec.op) {
            stopped = Some(format!("journal record {}: {msg}", rec.seq));
            break;
        }
        replayed += 1;
    }
    doc.history_mut().mark_dirty();
    Ok(Recovered {
        document: doc,
        replayed,
        stopped,
    })
}

fn replay(doc: &mut Document, op: Op) -> Result<(), String> {
    match op {
        Op::Apply { label, mut cmds } => {
            if cmds.len() == 1 {
                if let Some(cmd) = cmds.pop() {
                    doc.apply(cmd).map_err(|e| e.to_string())?;
                }
            } else {
                let mut tx = doc.transaction(&label);
                for cmd in cmds {
                    tx.apply(cmd).map_err(|e| e.to_string())?;
                }
                tx.commit();
            }
            Ok(())
        }
        Op::Undo => doc
            .undo()
            .then_some(())
            .ok_or_else(|| "nothing to undo".to_owned()),
        Op::Redo => doc
            .redo()
            .then_some(())
            .ok_or_else(|| "nothing to redo".to_owned()),
    }
}
