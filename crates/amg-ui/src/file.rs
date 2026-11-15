//! File operations and crash recovery (spec §15, UI-19).
//!
//! - **Commands** (`builtin`): New, Open…, Save, Save As…, Save Compact and Close, in the File
//!   menu, plus the Open Recent submenu (10 entries, `recent.json`) and the Import/Export menu
//!   slots that lane G fills through `Placement::Menu { path: "File/Import" | "File/Export" }`.
//! - **Dialogs.** A [`FileDialogBackend`] shows the native dialog. It blocks, so it runs on a
//!   dedicated thread (`amg-file-dialog`, its own COM apartment on Windows); the result comes
//!   back through a channel plus `request_repaint`. [`RfdDialogs`] is the `rfd` backend, parented
//!   to the main window; tests use `testing::FakeFileDialog`.
//! - **Save and open** run as jobs (`amg_doc::io::save_in_job` / `load_in_job`, with progress
//!   and cancel in the job panel). A save marks the document clean only if it did not change
//!   while the job ran.
//! - **Close prompt.** A close request with dirty documents is cancelled
//!   (`ViewportCommand::CancelClose`) and a Save / Don't save / Cancel `Modal` asks for each
//!   dirty document in turn; the window closes once every one is resolved.
//! - **Recovery** (DOC-14). With a [`RecoveryConfig`], every open document runs a
//!   `RecoverySession` attached to its history, so each applied commit batch is journaled
//!   (`HistoryObserver::on_commit`); `tick` runs every frame. At start-up
//!   [`AmalgamApp::check_recovery`] lists `find_recoverable`, recovers each candidate on a job
//!   and shows the recovery dialog (Open recovered / Discard / Later).

use std::path::{Path, PathBuf};
use std::sync::mpsc::{self, Receiver, TryRecvError};
use std::sync::{Arc, Mutex, Weak};
use std::time::SystemTime;

use amg_doc::io::{FormatError, SaveOptions, StubBodyCodec, load_in_job, save_in_job};
use amg_doc::{
    Document, Recovered, RecoveryCandidate, RecoveryConfig, RecoverySession, Revision,
    find_recoverable, recover_with_report,
};
use amg_jobs::{JobHandle, SpawnOptions};
use egui::{RichText, ViewportCommand};

use crate::app::{AmalgamApp, DocTabId};
use crate::notify::Notification;

/// File extension of Amalgam documents.
pub const AMG_EXTENSION: &str = "amg";
/// Entries kept in Open Recent.
pub const MAX_RECENT: usize = 10;
/// Name of the FND-07 recovery hook the app registers ([`AmalgamApp::set_recovery`]).
pub const PANIC_HOOK_NAME: &str = "amg-ui.recovery";

// ---- file dialogs --------------------------------------------------------------------------

/// Open or save dialog.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FileDialogKind {
    /// Pick an existing file.
    Open,
    /// Pick a target path (the dialog asks before overwriting).
    Save,
}

/// A file-type filter ("Amalgam design", `["amg"]`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FileFilter {
    /// Shown name.
    pub name: String,
    /// Extensions without the dot.
    pub extensions: Vec<String>,
}

/// What a file dialog shows.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FileDialogRequest {
    /// Open or save.
    pub kind: FileDialogKind,
    /// Window title.
    pub title: String,
    /// File-type filters.
    pub filters: Vec<FileFilter>,
    /// Starting folder.
    pub directory: Option<PathBuf>,
    /// Pre-filled file name (save dialogs).
    pub file_name: Option<String>,
}

impl FileDialogRequest {
    fn amg(kind: FileDialogKind, title: &str) -> FileDialogRequest {
        FileDialogRequest {
            kind,
            title: title.to_owned(),
            filters: vec![FileFilter {
                name: "Amalgam design".to_owned(),
                extensions: vec![AMG_EXTENSION.to_owned()],
            }],
            directory: None,
            file_name: None,
        }
    }
}

/// Shows native file dialogs. [`show`](Self::show) blocks until the user answers, so the app
/// only ever calls it on its dialog thread, never on the UI thread.
pub trait FileDialogBackend: Send + Sync + 'static {
    /// The chosen path, or `None` if the user cancelled.
    fn show(&self, req: &FileDialogRequest) -> Option<PathBuf>;
}

/// `rfd` dialogs (Win32 `IFileDialog`, GTK/portal, NSSavePanel), modal to the main window when
/// built with [`RfdDialogs::with_parent`].
#[derive(Clone, Debug, Default)]
pub struct RfdDialogs {
    template: rfd::FileDialog,
}

impl RfdDialogs {
    /// Dialogs without a parent window.
    pub fn new() -> RfdDialogs {
        RfdDialogs::default()
    }

    /// Dialogs parented to `window` (eframe's `CreationContext` or `Frame`). Only the raw
    /// handles are kept; they stay valid while the window lives.
    pub fn with_parent<W>(window: &W) -> RfdDialogs
    where
        W: raw_window_handle::HasWindowHandle + raw_window_handle::HasDisplayHandle + ?Sized,
    {
        RfdDialogs {
            template: rfd::FileDialog::new().set_parent(window),
        }
    }
}

impl FileDialogBackend for RfdDialogs {
    fn show(&self, req: &FileDialogRequest) -> Option<PathBuf> {
        let mut d = self.template.clone().set_title(req.title.as_str());
        for f in &req.filters {
            d = d.add_filter(f.name.as_str(), &f.extensions);
        }
        if let Some(dir) = &req.directory {
            d = d.set_directory(dir);
        }
        if let Some(name) = &req.file_name {
            d = d.set_file_name(name.as_str());
        }
        match req.kind {
            FileDialogKind::Open => d.pick_file(),
            FileDialogKind::Save => d.save_file(),
        }
    }
}

/// Why a dialog was opened.
#[derive(Clone, Debug)]
enum DialogPurpose {
    Open,
    SaveTo { doc: DocTabId, compact: bool },
}

struct PendingDialog {
    purpose: DialogPurpose,
    rx: Receiver<Option<PathBuf>>,
}

// ---- requests ------------------------------------------------------------------------------

/// A file operation on a document (the File menu commands; [`crate::AppCx::file`]).
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FileOp {
    /// New empty document.
    New,
    /// Open dialog, then open the file.
    Open,
    /// Open this file (Open Recent, drag and drop, command line).
    OpenPath(PathBuf),
    /// Save to the document's file (Save As if it has none).
    Save,
    /// Save dialog, then save there.
    SaveAs,
    /// Save without the geometry cache (S01 "save compact"); asks for a path if there is none.
    SaveCompact,
    /// Close the document (asks first if it is dirty).
    Close,
}

/// Where the close flow stands ([`AmalgamApp::close_state`]).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CloseState {
    /// No close in progress.
    Idle,
    /// The Save / Don't save / Cancel prompt is shown for this document.
    Prompting(DocTabId),
    /// Waiting for this document's save (dialog or job) before closing.
    Saving(DocTabId),
    /// Every document is resolved; the window was asked to close.
    Closing,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum CloseScope {
    App,
    Doc(DocTabId),
}

#[derive(Debug)]
struct CloseFlow {
    scope: CloseScope,
    /// Dirty documents still to ask about, in tab order.
    queue: Vec<DocTabId>,
    /// The document the prompt is shown for.
    prompt: Option<DocTabId>,
    /// The document whose save the flow waits for.
    saving: Option<DocTabId>,
}

// ---- jobs ----------------------------------------------------------------------------------

struct SaveJob {
    doc: DocTabId,
    path: PathBuf,
    /// Design revision the snapshot was taken at.
    revision: Revision,
    handle: JobHandle<(), FormatError>,
}

/// An opened file, built off the UI thread.
struct Opened {
    document: Document,
    warnings: Vec<String>,
}

struct OpenJob {
    path: PathBuf,
    handle: JobHandle<Opened, FormatError>,
}

// ---- recovery ------------------------------------------------------------------------------

enum EntryStatus {
    Loading(JobHandle<Recovered, FormatError>),
    Ready(Box<Recovered>),
    Failed(String),
}

struct RecoveryEntry {
    candidate: RecoveryCandidate,
    status: EntryStatus,
}

/// What the user picked in the recovery dialog this frame.
enum RecoveryAction {
    Open(usize),
    Discard(usize),
    Later,
}

/// What the user picked in the close prompt this frame.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum PromptAction {
    Save,
    DontSave,
    Cancel,
}

// ---- recent files --------------------------------------------------------------------------

/// The Open Recent list, most recent first, persisted as `recent.json`.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct RecentFiles {
    files: Vec<PathBuf>,
}

impl RecentFiles {
    /// The paths, most recent first.
    pub fn files(&self) -> &[PathBuf] {
        &self.files
    }

    /// Moves `path` to the front; keeps at most [`MAX_RECENT`].
    pub fn add(&mut self, path: &Path) {
        self.files.retain(|p| p != path);
        self.files.insert(0, path.to_path_buf());
        self.files.truncate(MAX_RECENT);
    }

    /// Removes `path`; true if it was listed.
    pub fn remove(&mut self, path: &Path) -> bool {
        let before = self.files.len();
        self.files.retain(|p| p != path);
        before != self.files.len()
    }

    /// `{"version":1,"files":[…]}`.
    pub fn to_json(&self) -> serde_json::Value {
        serde_json::json!({ "version": 1, "files": self.files })
    }

    /// Reads [`Self::to_json`]; anything else gives an empty list.
    pub fn from_json(v: &serde_json::Value) -> RecentFiles {
        let files = v
            .get("files")
            .and_then(serde_json::Value::as_array)
            .map(|a| {
                a.iter()
                    .filter_map(serde_json::Value::as_str)
                    .map(PathBuf::from)
                    .take(MAX_RECENT)
                    .collect()
            })
            .unwrap_or_default();
        RecentFiles { files }
    }
}

/// The default `recent.json` path: `%APPDATA%\Amalgam\recent.json` on Windows.
pub fn default_recent_path() -> Option<PathBuf> {
    directories::BaseDirs::new().map(|d| d.config_dir().join("Amalgam").join("recent.json"))
}

/// The default recovery folder (S01 §8): `%LOCALAPPDATA%\Amalgam\recovery` on Windows,
/// `$XDG_STATE_HOME/amalgam/recovery` on Linux.
pub fn default_recovery_dir() -> Option<PathBuf> {
    directories::BaseDirs::new().map(|d| match d.state_dir() {
        Some(state) => state.join("amalgam").join("recovery"),
        None => d.data_local_dir().join("Amalgam").join("recovery"),
    })
}

// ---- state ---------------------------------------------------------------------------------

/// The app's file-operation state.
pub(crate) struct FileState {
    backend: Arc<dyn FileDialogBackend>,
    dialog: Option<PendingDialog>,
    saves: Vec<SaveJob>,
    opens: Vec<OpenJob>,
    pub(crate) recent: RecentFiles,
    recent_path: Option<PathBuf>,
    recovery: Option<RecoveryConfig>,
    /// Sessions the panic hook flushes (weak: a closed document drops out).
    hook_sessions: Arc<Mutex<Vec<Weak<RecoverySession>>>>,
    entries: Vec<RecoveryEntry>,
    recovery_open: bool,
    close: Option<CloseFlow>,
    /// The close flow finished: the next close request is let through.
    allow_close: bool,
    /// `finish` of closed documents' sessions (joined by [`AmalgamApp::shutdown`]).
    finishing: Vec<std::thread::JoinHandle<()>>,
    title: String,
}

impl Default for FileState {
    fn default() -> Self {
        FileState {
            backend: Arc::new(RfdDialogs::new()),
            dialog: None,
            saves: Vec::new(),
            opens: Vec::new(),
            recent: RecentFiles::default(),
            recent_path: None,
            recovery: None,
            hook_sessions: Arc::new(Mutex::new(Vec::new())),
            entries: Vec::new(),
            recovery_open: false,
            close: None,
            allow_close: false,
            finishing: Vec::new(),
            title: String::new(),
        }
    }
}

impl std::fmt::Debug for FileState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("FileState")
            .field("dialog", &self.dialog.as_ref().map(|d| &d.purpose))
            .field("saves", &self.saves.len())
            .field("opens", &self.opens.len())
            .field("recovery", &self.recovery)
            .field("close", &self.close)
            .finish_non_exhaustive()
    }
}

/// `path` ending in `.amg` (appended unless it already ends so, so `part.v2` → `part.v2.amg`).
fn with_amg_extension(path: PathBuf) -> PathBuf {
    if path
        .extension()
        .is_some_and(|e| e.eq_ignore_ascii_case(AMG_EXTENSION))
    {
        return path;
    }
    let mut s = path.into_os_string();
    s.push(".");
    s.push(AMG_EXTENSION);
    PathBuf::from(s)
}

/// Tab name for a file: its stem.
fn tab_name(path: &Path) -> String {
    path.file_stem().map_or_else(
        || path.display().to_string(),
        |s| s.to_string_lossy().into_owned(),
    )
}

/// Deletes per-document folders under `cfg.dir` that hold no snapshot and whose lock is free
/// (sessions that died before their first snapshot). A folder in `keep`, one with a snapshot,
/// and one still locked by a live session (another Amalgam) are left alone. Taking the lock is
/// `RecoverySession::start`; `finish` then deletes the folder.
fn sweep_stale_folders(cfg: &RecoveryConfig, keep: &[PathBuf]) {
    let Ok(entries) = std::fs::read_dir(&cfg.dir) else {
        return;
    };
    for entry in entries.flatten() {
        let dir = entry.path();
        if !dir.is_dir() || keep.contains(&dir) {
            continue;
        }
        let Some(uuid) = dir
            .file_name()
            .and_then(|n| n.to_str())
            .and_then(|n| n.parse::<amg_core::PersistentId>().ok())
        else {
            continue;
        };
        let has_snapshot = std::fs::read_dir(&dir).is_ok_and(|files| {
            files
                .flatten()
                .any(|f| f.file_name().to_string_lossy().starts_with("snapshot-"))
        });
        if has_snapshot {
            continue;
        }
        if let Ok(session) = RecoverySession::start(cfg, uuid)
            && let Err(e) = session.finish()
        {
            tracing::debug!("could not delete {}: {e}", dir.display());
        }
    }
}

/// "3 min ago" style age of a recovery folder.
fn age(t: SystemTime) -> String {
    let secs = SystemTime::now()
        .duration_since(t)
        .map_or(0, |d| d.as_secs());
    match secs {
        0..60 => "just now".to_owned(),
        60..3600 => format!("{} min ago", secs / 60),
        3600..86_400 => format!("{} h ago", secs / 3600),
        _ => format!("{} days ago", secs / 86_400),
    }
}

impl AmalgamApp {
    // ---- configuration ----------------------------------------------------------------------

    /// Replaces the file-dialog backend (default: [`RfdDialogs`] without a parent; `amg-app`
    /// passes one parented to the window, tests a fake).
    pub fn set_file_dialog_backend(&mut self, backend: Arc<dyn FileDialogBackend>) {
        self.files.backend = backend;
    }

    /// Where Open Recent is persisted (`None`: in memory only). Loads the list.
    pub fn set_recent_path(&mut self, path: Option<PathBuf>) {
        if let Some(p) = &path
            && let Ok(text) = std::fs::read_to_string(p)
        {
            match serde_json::from_str::<serde_json::Value>(&text) {
                Ok(v) => self.files.recent = RecentFiles::from_json(&v),
                Err(e) => tracing::warn!("ignoring {}: {e}", p.display()),
            }
        }
        self.files.recent_path = path;
    }

    /// The Open Recent list.
    pub fn recent_files(&self) -> &RecentFiles {
        &self.files.recent
    }

    fn save_recent(&self) {
        let Some(path) = &self.files.recent_path else {
            return;
        };
        // A few hundred bytes; the UI thread writes layout.json the same way.
        let write = || -> std::io::Result<()> {
            if let Some(dir) = path.parent() {
                std::fs::create_dir_all(dir)?;
            }
            let text = serde_json::to_string_pretty(&self.files.recent.to_json())
                .map_err(std::io::Error::other)?;
            std::fs::write(path, text)
        };
        if let Err(e) = write() {
            tracing::warn!("could not write {}: {e}", path.display());
        }
    }

    fn add_recent(&mut self, path: &Path) {
        self.files.recent.add(path);
        self.save_recent();
    }

    /// Enables crash recovery: every open (and later opened) document gets a
    /// `RecoverySession` in `cfg.dir`, and the FND-07 panic hook flushes their journals.
    /// `None` stops new sessions (running ones continue).
    pub fn set_recovery(&mut self, cfg: Option<RecoveryConfig>) {
        self.files.recovery = cfg;
        let Some(cfg) = &self.files.recovery else {
            return;
        };
        let sessions = Arc::clone(&self.files.hook_sessions);
        let dir = cfg.dir.clone();
        amg_jobs::diag::register_recovery_hook(
            PANIC_HOOK_NAME,
            Box::new(move || {
                // Never wait for a lock the panicking thread may hold.
                if let Ok(list) = sessions.try_lock() {
                    for s in list.iter().filter_map(Weak::upgrade) {
                        s.flush();
                    }
                }
                Some(dir.clone())
            }),
        );
        let ids: Vec<DocTabId> = self
            .docs
            .iter()
            .filter(|d| d.recovery.is_none())
            .map(|d| d.id)
            .collect();
        for id in ids {
            self.start_recovery_session(id);
        }
    }

    /// The recovery configuration, if enabled.
    pub fn recovery_config(&self) -> Option<&RecoveryConfig> {
        self.files.recovery.as_ref()
    }

    /// Starts (and attaches) the recovery session of a document, if recovery is enabled.
    pub(crate) fn start_recovery_session(&mut self, id: DocTabId) {
        let Some(cfg) = self.files.recovery.clone() else {
            return;
        };
        let Some(doc) = self.docs.iter_mut().find(|d| d.id == id) else {
            return;
        };
        let uuid = doc.document.design().meta().uuid;
        match RecoverySession::start(&cfg, uuid) {
            Ok(session) => {
                session.attach(&mut doc.document);
                if let Ok(mut list) = self.files.hook_sessions.lock() {
                    list.retain(|w| w.strong_count() > 0);
                    list.push(Arc::downgrade(&session));
                }
                doc.recovery = Some(session);
            }
            Err(e) => tracing::warn!("no crash recovery for `{}`: {e}", doc.name),
        }
    }

    /// Syncs every document's recovery journal to disk (the panic hook does the same).
    pub fn flush_recovery(&self) {
        for d in &self.docs {
            if let Some(s) = &d.recovery {
                s.flush();
            }
        }
    }

    // ---- per frame --------------------------------------------------------------------------

    /// File-related work of `logic`: dialog results, finished save/open/recovery jobs, the
    /// window's close request, recovery ticks and the window title. Never blocks.
    pub(crate) fn poll_files(&mut self, ctx: &egui::Context) {
        self.poll_dialog();
        self.poll_saves();
        self.poll_opens();
        self.poll_recovery_entries();
        self.tick_recovery();
        self.handle_close_request(ctx);
        self.update_title(ctx);
    }

    fn tick_recovery(&self) {
        for d in &self.docs {
            if let Some(s) = &d.recovery
                && (d.document.is_dirty() || s.has_snapshot())
            {
                s.tick(&d.document, &self.jobs);
            }
        }
    }

    fn update_title(&mut self, ctx: &egui::Context) {
        let doc = &self.docs[self.active];
        let dirty = if doc.document.is_dirty() { " *" } else { "" };
        let title = format!("{}{dirty} — Amalgam", doc.name);
        if title != self.files.title {
            ctx.send_viewport_cmd(ViewportCommand::Title(title.clone()));
            self.files.title = title;
        }
    }

    /// True while a file dialog, save, open or recovery job is pending.
    pub fn file_busy(&self) -> bool {
        self.files.dialog.is_some()
            || !self.files.saves.is_empty()
            || !self.files.opens.is_empty()
            || self
                .files
                .entries
                .iter()
                .any(|e| matches!(e.status, EntryStatus::Loading(_)))
    }

    // ---- operations -------------------------------------------------------------------------

    /// Runs a file operation on a document (`None`: the active one).
    pub fn file_op(&mut self, doc: Option<DocTabId>, op: FileOp) {
        let id = doc.unwrap_or(self.docs[self.active].id);
        match op {
            FileOp::New => {
                self.new_document();
            }
            FileOp::Open => {
                let _ = self.show_dialog(
                    FileDialogRequest::amg(FileDialogKind::Open, "Open design"),
                    DialogPurpose::Open,
                );
            }
            FileOp::OpenPath(p) => self.open_path(p),
            FileOp::Save => self.save_document(id, false, false),
            FileOp::SaveAs => self.save_document(id, true, false),
            FileOp::SaveCompact => self.save_document(id, false, true),
            FileOp::Close => self.close_document(id),
        }
        self.repaint_now();
    }

    fn repaint_now(&self) {
        if let Some(ctx) = &self.ctx {
            ctx.request_repaint();
        }
    }

    /// Shows a dialog on the dialog thread; false if none could be shown (one is open).
    fn show_dialog(&mut self, req: FileDialogRequest, purpose: DialogPurpose) -> bool {
        if self.files.dialog.is_some() {
            tracing::debug!("a file dialog is already open");
            return false;
        }
        let (tx, rx) = mpsc::channel();
        let backend = Arc::clone(&self.files.backend);
        let ctx = self.ctx.clone();
        let spawned = std::thread::Builder::new()
            .name("amg-file-dialog".to_owned())
            .spawn(move || {
                let answer = backend.show(&req);
                let _ = tx.send(answer);
                if let Some(ctx) = ctx {
                    ctx.request_repaint();
                }
            });
        match spawned {
            Ok(_) => {
                self.files.dialog = Some(PendingDialog { purpose, rx });
                true
            }
            Err(e) => {
                self.notify(Notification::error(
                    "Cannot show the file dialog",
                    e.to_string(),
                ));
                false
            }
        }
    }

    fn poll_dialog(&mut self) {
        let Some(pending) = &self.files.dialog else {
            return;
        };
        let answer = match pending.rx.try_recv() {
            Ok(a) => a,
            Err(TryRecvError::Empty) => return,
            Err(TryRecvError::Disconnected) => None, // the dialog thread died: a cancel
        };
        let Some(PendingDialog { purpose, .. }) = self.files.dialog.take() else {
            return;
        };
        match (purpose, answer) {
            (DialogPurpose::Open, Some(path)) => self.open_path(path),
            (DialogPurpose::SaveTo { doc, compact }, Some(path)) => {
                self.start_save(doc, with_amg_extension(path), compact);
            }
            (DialogPurpose::SaveTo { doc, .. }, None) => self.save_finished(doc, false),
            (DialogPurpose::Open, None) => {}
        }
        self.repaint_now();
    }

    /// Opens `path` as a new tab on a job (or activates it if it is open already).
    pub fn open_path(&mut self, path: PathBuf) {
        if let Some(d) = self.docs.iter().find(|d| d.path.as_deref() == Some(&path)) {
            let id = d.id;
            self.activate(id);
            return;
        }
        if self.files.opens.iter().any(|o| o.path == path) {
            return;
        }
        let job_path = path.clone();
        let label = format!("Open {}", tab_name(&path));
        let handle = self
            .jobs
            .spawn_fn(label, SpawnOptions::default(), move |ctx| {
                let loaded = load_in_job(&job_path, &StubBodyCodec, ctx)?;
                let warnings = loaded.warnings.clone();
                let mut document = loaded.into_document();
                document.history_mut().mark_saved();
                Ok(Opened { document, warnings })
            });
        self.files.opens.push(OpenJob { path, handle });
    }

    fn poll_opens(&mut self) {
        let mut i = 0;
        while i < self.files.opens.len() {
            let Some(result) = self.files.opens[i].handle.try_take() else {
                i += 1;
                continue;
            };
            let job = self.files.opens.swap_remove(i);
            match result {
                Ok(opened) => {
                    let id = self.add_document(tab_name(&job.path), opened.document);
                    if let Some(d) = self.docs.iter_mut().find(|d| d.id == id) {
                        d.path = Some(job.path.clone());
                    }
                    self.add_recent(&job.path);
                    for w in opened.warnings {
                        self.notify(Notification::warning("Opened with warnings", w));
                    }
                }
                Err(e) => {
                    if !matches!(
                        e,
                        amg_jobs::JobError::Cancelled
                            | amg_jobs::JobError::Failed(FormatError::Cancelled)
                    ) {
                        self.notify(
                            Notification::error(
                                "Cannot open the file",
                                format!("{}: {e}", job.path.display()),
                            )
                            .with_details(amg_core::report(&e)),
                        );
                    }
                }
            }
        }
    }

    /// Saves a document: to its file, or through the save dialog for Save As and documents
    /// without one. `compact` leaves the geometry cache out.
    pub fn save_document(&mut self, id: DocTabId, save_as: bool, compact: bool) {
        let Some(doc) = self.docs.iter().find(|d| d.id == id) else {
            return;
        };
        if self.files.saves.iter().any(|s| s.doc == id) {
            self.status = Some(format!("{} is being saved", doc.name));
            return;
        }
        match (&doc.path, save_as) {
            (Some(path), false) => {
                let path = path.clone();
                self.start_save(id, path, compact);
            }
            (path, _) => {
                let mut req = FileDialogRequest::amg(
                    FileDialogKind::Save,
                    if compact {
                        "Save compact"
                    } else {
                        "Save design"
                    },
                );
                req.directory = path
                    .as_ref()
                    .and_then(|p| p.parent())
                    .map(Path::to_path_buf);
                req.file_name = Some(format!("{}.{AMG_EXTENSION}", doc.name));
                if !self.show_dialog(req, DialogPurpose::SaveTo { doc: id, compact }) {
                    self.save_finished(id, false);
                }
            }
        }
    }

    fn start_save(&mut self, id: DocTabId, path: PathBuf, compact: bool) {
        let Some(doc) = self.docs.iter().find(|d| d.id == id) else {
            return;
        };
        let snap = doc.document.snapshot();
        let revision = doc.document.design().revision();
        let opts = SaveOptions {
            cache: (!compact).then(|| doc.document.cache_export()),
            ..SaveOptions::default()
        };
        let job_path = path.clone();
        let label = format!("Save {}", tab_name(&path));
        let handle = self
            .jobs
            .spawn_fn(label, SpawnOptions::default(), move |ctx| {
                save_in_job(&job_path, &snap, &opts, ctx)
            });
        self.files.saves.push(SaveJob {
            doc: id,
            path,
            revision,
            handle,
        });
    }

    fn poll_saves(&mut self) {
        let mut i = 0;
        while i < self.files.saves.len() {
            let Some(result) = self.files.saves[i].handle.try_take() else {
                i += 1;
                continue;
            };
            let job = self.files.saves.swap_remove(i);
            let ok = match result {
                Ok(()) => {
                    if let Some(d) = self.docs.iter_mut().find(|d| d.id == job.doc) {
                        d.path = Some(job.path.clone());
                        d.name = tab_name(&job.path);
                        // Edits made while the job ran are not in the file.
                        if d.document.design().revision() == job.revision {
                            d.document.history_mut().mark_saved();
                        }
                    }
                    self.status = None;
                    self.add_recent(&job.path);
                    true
                }
                Err(e) => {
                    if !matches!(
                        e,
                        amg_jobs::JobError::Cancelled
                            | amg_jobs::JobError::Failed(FormatError::Cancelled)
                    ) {
                        self.notify(
                            Notification::error(
                                "Save failed",
                                format!("{}: {e}", job.path.display()),
                            )
                            .with_details(amg_core::report(&e)),
                        );
                    }
                    false
                }
            };
            self.save_finished(job.doc, ok);
        }
    }

    /// A save of `doc` ended (`ok`), or its dialog was cancelled (`!ok`): continues or stops
    /// the close flow that waits for it.
    fn save_finished(&mut self, doc: DocTabId, ok: bool) {
        let Some(flow) = &mut self.files.close else {
            return;
        };
        if flow.saving != Some(doc) {
            return;
        }
        flow.saving = None;
        if ok {
            self.advance_close();
        } else {
            self.files.close = None;
        }
    }

    // ---- closing ----------------------------------------------------------------------------

    /// Where the close flow stands.
    pub fn close_state(&self) -> CloseState {
        match &self.files.close {
            _ if self.files.allow_close => CloseState::Closing,
            None => CloseState::Idle,
            Some(f) => match (f.prompt, f.saving) {
                (Some(d), _) => CloseState::Prompting(d),
                (None, Some(d)) => CloseState::Saving(d),
                (None, None) => CloseState::Idle,
            },
        }
    }

    /// Closes a document tab, asking first if it is dirty.
    pub fn close_document(&mut self, id: DocTabId) {
        if self.files.close.is_some() {
            return;
        }
        self.files.close = Some(CloseFlow {
            scope: CloseScope::Doc(id),
            queue: vec![id],
            prompt: None,
            saving: None,
        });
        self.advance_close();
    }

    /// The window was asked to close: dirty documents cancel the close and start the prompt.
    fn handle_close_request(&mut self, ctx: &egui::Context) {
        if !ctx.input(|i| i.viewport().close_requested()) || self.files.allow_close {
            return;
        }
        let dirty: Vec<DocTabId> = self
            .docs
            .iter()
            .filter(|d| d.document.is_dirty())
            .map(|d| d.id)
            .collect();
        if dirty.is_empty() && self.files.close.is_none() {
            self.files.allow_close = true;
            return;
        }
        ctx.send_viewport_cmd(ViewportCommand::CancelClose);
        if self.files.close.is_none() {
            self.files.close = Some(CloseFlow {
                scope: CloseScope::App,
                queue: dirty,
                prompt: None,
                saving: None,
            });
            self.advance_close();
        }
        ctx.request_repaint();
    }

    /// Prompts for the next dirty document of the flow, or completes it.
    fn advance_close(&mut self) {
        loop {
            let Some(flow) = &mut self.files.close else {
                return;
            };
            if flow.prompt.is_some() || flow.saving.is_some() {
                return;
            }
            if flow.queue.is_empty() {
                let scope = flow.scope;
                self.files.close = None;
                self.complete_close(scope);
                return;
            }
            let next = flow.queue.remove(0);
            let dirty = self
                .docs
                .iter()
                .find(|d| d.id == next)
                .is_some_and(|d| d.document.is_dirty());
            if dirty {
                flow.prompt = Some(next);
                self.activate(next);
                return;
            }
            // Clean (or gone): nothing to ask. A single clean tab closes at once.
            if flow.scope == CloseScope::Doc(next) {
                self.remove_document(next);
            }
        }
    }

    fn complete_close(&mut self, scope: CloseScope) {
        match scope {
            CloseScope::App => {
                self.files.allow_close = true;
                if let Some(ctx) = &self.ctx {
                    ctx.send_viewport_cmd(ViewportCommand::Close);
                    ctx.request_repaint();
                }
            }
            CloseScope::Doc(id) => self.remove_document(id),
        }
    }

    fn prompt_answer(&mut self, action: PromptAction) {
        let Some(flow) = &mut self.files.close else {
            return;
        };
        let Some(doc) = flow.prompt.take() else {
            return;
        };
        match action {
            PromptAction::Cancel => self.files.close = None,
            PromptAction::DontSave => {
                if flow.scope == CloseScope::Doc(doc) {
                    self.remove_document(doc);
                }
                self.advance_close();
            }
            PromptAction::Save => {
                flow.saving = Some(doc);
                if flow.scope == CloseScope::Doc(doc) {
                    flow.queue.push(doc); // closes once the save made it clean
                }
                self.save_document(doc, false, false);
            }
        }
        self.repaint_now();
    }

    /// Removes a tab without asking; its recovery session is finished (folder deleted) off
    /// the UI thread. The app always keeps one document.
    pub(crate) fn remove_document(&mut self, id: DocTabId) {
        let Some(i) = self.docs.iter().position(|d| d.id == id) else {
            return;
        };
        let active_id = self.docs[self.active].id;
        let doc = self.docs.remove(i);
        if let Some(s) = doc.recovery {
            let spawned = std::thread::Builder::new()
                .name("amg-recovery-finish".to_owned())
                .spawn(move || {
                    if let Err(e) = s.finish() {
                        tracing::warn!("could not delete the recovery folder: {e}");
                    }
                });
            if let Ok(h) = spawned {
                self.files.finishing.push(h);
            }
        }
        if self.docs.is_empty() {
            self.new_document();
        } else {
            self.active = self
                .docs
                .iter()
                .position(|d| d.id == active_id)
                .unwrap_or(i.min(self.docs.len() - 1));
        }
        self.invalidate_scene();
        self.repaint_now();
    }

    /// Clean shutdown (the host's `on_exit`): finishes every recovery session, so nothing is
    /// offered for recovery on the next start. Blocks briefly; the window is already gone.
    pub fn shutdown(&mut self) {
        for d in &mut self.docs {
            if let Some(s) = d.recovery.take()
                && let Err(e) = s.finish()
            {
                tracing::warn!("could not delete the recovery folder: {e}");
            }
        }
        for h in self.files.finishing.drain(..) {
            let _ = h.join();
        }
        amg_jobs::diag::unregister_recovery_hook(PANIC_HOOK_NAME);
    }

    // ---- recovery ---------------------------------------------------------------------------

    /// Lists recoverable documents (`find_recoverable`), recovers each on a job and opens the
    /// recovery dialog. Returns the number of candidates. Call once at start-up, after
    /// [`Self::set_recovery`].
    pub fn check_recovery(&mut self) -> usize {
        let Some(cfg) = &self.files.recovery else {
            return 0;
        };
        let own: Vec<_> = self
            .docs
            .iter()
            .map(|d| d.document.design().meta().uuid)
            .collect();
        let candidates: Vec<RecoveryCandidate> = find_recoverable(cfg)
            .into_iter()
            .filter(|c| c.doc_uuid.is_none_or(|u| !own.contains(&u)))
            .filter(|c| !self.files.entries.iter().any(|e| e.candidate.dir == c.dir))
            .collect();
        let n = candidates.len();
        // Dead sessions that never wrote a snapshot (a crash before the first edit) are not
        // recoverable; delete their folders in the background so they do not pile up.
        let mut keep: Vec<PathBuf> = candidates.iter().map(|c| c.dir.clone()).collect();
        keep.extend(self.files.entries.iter().map(|e| e.candidate.dir.clone()));
        keep.extend(
            self.docs
                .iter()
                .filter_map(|d| d.recovery.as_ref().map(|s| s.dir().to_path_buf())),
        );
        let sweep_cfg = cfg.clone();
        let _ = self.jobs.spawn_fn(
            "Clean up recovery folders",
            SpawnOptions {
                priority: amg_jobs::Priority::Background,
                hidden: false,
            },
            move |_ctx| {
                sweep_stale_folders(&sweep_cfg, &keep);
                Ok::<(), FormatError>(())
            },
        );
        for candidate in candidates {
            let c = candidate.clone();
            let handle =
                self.jobs
                    .spawn_fn("Recover document", SpawnOptions::default(), move |ctx| {
                        ctx.checkpoint()?;
                        recover_with_report(&c)
                    });
            self.files.entries.push(RecoveryEntry {
                candidate,
                status: EntryStatus::Loading(handle),
            });
        }
        if !self.files.entries.is_empty() {
            self.files.recovery_open = true;
            self.repaint_now();
        }
        n
    }

    /// Whether the recovery dialog is shown.
    pub fn recovery_dialog_open(&self) -> bool {
        self.files.recovery_open
    }

    fn poll_recovery_entries(&mut self) {
        for e in &mut self.files.entries {
            if let EntryStatus::Loading(h) = &mut e.status
                && let Some(result) = h.try_take()
            {
                e.status = match result {
                    Ok(r) => EntryStatus::Ready(Box::new(r)),
                    Err(err) => EntryStatus::Failed(err.to_string()),
                };
            }
        }
    }

    fn recovery_action(&mut self, action: RecoveryAction) {
        match action {
            RecoveryAction::Later => {
                for e in self.files.entries.drain(..) {
                    if let EntryStatus::Loading(h) = e.status {
                        h.cancel();
                    }
                }
                self.files.recovery_open = false;
            }
            RecoveryAction::Discard(i) if i < self.files.entries.len() => {
                let e = self.files.entries.remove(i);
                if let EntryStatus::Loading(h) = &e.status {
                    h.cancel();
                }
                let candidate = e.candidate;
                let _ = self.jobs.spawn_fn(
                    "Discard recovery data",
                    SpawnOptions {
                        priority: amg_jobs::Priority::Background,
                        hidden: false,
                    },
                    move |_ctx| candidate.discard().map_err(FormatError::from),
                );
            }
            RecoveryAction::Open(i) if i < self.files.entries.len() => {
                if !matches!(self.files.entries[i].status, EntryStatus::Ready(_)) {
                    return;
                }
                let e = self.files.entries.remove(i);
                if let EntryStatus::Ready(rec) = e.status {
                    let name = format!("{} (recovered)", rec.document.design().meta().name);
                    if let Some(why) = &rec.stopped {
                        self.notify(Notification::warning(
                            "Recovered with gaps",
                            format!("Replay stopped early: {why}"),
                        ));
                    }
                    // Starting the new session takes over (and resets) the old folder.
                    self.add_document(name, rec.document);
                }
            }
            RecoveryAction::Open(_) | RecoveryAction::Discard(_) => {}
        }
        if self.files.entries.is_empty() {
            self.files.recovery_open = false;
        }
        self.repaint_now();
    }

    // ---- drawing ----------------------------------------------------------------------------

    /// The close prompt and the recovery dialog (both modal).
    pub(crate) fn draw_file_dialogs(&mut self, ctx: &egui::Context) {
        if let Some(action) = self.close_prompt_ui(ctx) {
            self.prompt_answer(action);
        }
        if let Some(action) = self.recovery_ui(ctx) {
            self.recovery_action(action);
        }
    }

    fn close_prompt_ui(&self, ctx: &egui::Context) -> Option<PromptAction> {
        let doc = self.files.close.as_ref()?.prompt?;
        let name = self.docs.iter().find(|d| d.id == doc)?.name.clone();
        let mut action = None;
        let modal = egui::Modal::new(egui::Id::new("amg_close_prompt")).show(ctx, |ui| {
            ui.set_max_width(360.0);
            ui.heading("Unsaved changes");
            ui.label(format!("Save changes to “{name}” before closing?"));
            ui.add_space(8.0);
            ui.horizontal(|ui| {
                if ui.button("Save").clicked() {
                    action = Some(PromptAction::Save);
                }
                if ui.button("Don't save").clicked() {
                    action = Some(PromptAction::DontSave);
                }
                if ui.button("Cancel").clicked() {
                    action = Some(PromptAction::Cancel);
                }
            });
        });
        if action.is_none() && modal.should_close() {
            action = Some(PromptAction::Cancel);
        }
        action
    }

    fn recovery_ui(&self, ctx: &egui::Context) -> Option<RecoveryAction> {
        if !self.files.recovery_open || self.files.close.is_some() {
            return None;
        }
        let mut action = None;
        egui::Modal::new(egui::Id::new("amg_recovery")).show(ctx, |ui| {
            ui.set_max_width(480.0);
            ui.heading("Recover unsaved work");
            ui.label("Amalgam did not shut down cleanly. These documents have unsaved changes:");
            ui.add_space(6.0);
            for (i, e) in self.files.entries.iter().enumerate() {
                ui.push_id(("amg_recovery_row", i), |ui| {
                    ui.separator();
                    match &e.status {
                        EntryStatus::Loading(_) => {
                            ui.horizontal(|ui| {
                                ui.spinner();
                                ui.label("Reading recovery data…");
                            });
                        }
                        EntryStatus::Ready(rec) => {
                            ui.label(
                                RichText::new(rec.document.design().meta().name.as_str()).strong(),
                            );
                            ui.weak(format!(
                                "{} · {} edit(s) after the last snapshot",
                                age(e.candidate.modified),
                                rec.replayed
                            ));
                            ui.horizontal(|ui| {
                                if ui.button("Open recovered").clicked() {
                                    action = Some(RecoveryAction::Open(i));
                                }
                                if ui.button("Discard").clicked() {
                                    action = Some(RecoveryAction::Discard(i));
                                }
                            });
                        }
                        EntryStatus::Failed(err) => {
                            ui.colored_label(self.tokens.error, format!("Cannot recover: {err}"));
                            if ui.button("Discard").clicked() {
                                action = Some(RecoveryAction::Discard(i));
                            }
                        }
                    }
                });
            }
            ui.separator();
            if ui
                .button("Later")
                .on_hover_text("Keep the recovery data and ask again at the next start")
                .clicked()
            {
                action = Some(RecoveryAction::Later);
            }
        });
        action
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recent_files_dedupe_cap_and_round_trip() {
        let mut r = RecentFiles::default();
        for i in 0..12 {
            r.add(Path::new(&format!("f{i}.amg")));
        }
        assert_eq!(r.files().len(), MAX_RECENT);
        assert_eq!(r.files()[0], Path::new("f11.amg"));
        r.add(Path::new("f5.amg"));
        assert_eq!(r.files()[0], Path::new("f5.amg"));
        assert_eq!(r.files().len(), MAX_RECENT, "no duplicate");
        assert!(r.remove(Path::new("f5.amg")));
        assert!(!r.remove(Path::new("f5.amg")));
        let back = RecentFiles::from_json(&r.to_json());
        assert_eq!(back, r);
        assert_eq!(
            RecentFiles::from_json(&serde_json::json!("junk")),
            RecentFiles::default()
        );
    }

    #[test]
    fn save_paths_get_the_amg_extension() {
        assert_eq!(
            with_amg_extension(PathBuf::from("a/part")),
            Path::new("a/part.amg")
        );
        assert_eq!(
            with_amg_extension(PathBuf::from("a/part.amg")),
            Path::new("a/part.amg")
        );
        assert_eq!(
            with_amg_extension(PathBuf::from("part.v2")),
            Path::new("part.v2.amg")
        );
        assert_eq!(tab_name(Path::new("dir/Bracket v2.amg")), "Bracket v2");
    }
}
