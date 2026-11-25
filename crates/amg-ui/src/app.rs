//! The application state, the frame loop and the contexts handed to modules (spec §2, note §3).
//!
//! A frame is [`AmalgamApp::logic`] (document polling and events, no layout), then
//! [`AmalgamApp::ui`] (layout and widgets), which ends with [`AmalgamApp::end_frame`]: the
//! queued [`CommitBatch`]es become one `Document::transaction` each. No widget holds
//! `&mut Document`; everything mutates through [`AppCx::submit`].
//!
//! Repaint policy: the app never asks for a repaint itself. egui repaints on input, and the
//! [`JobSystem`] notifier ([`repaint_notifier`]) repaints when a job changes state, so an idle
//! app runs no frames at all (Done-when 4).

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::{Arc, OnceLock};

use amg_doc::{Command as DocCommand, Design, Document, Evaluated};
use amg_jobs::JobSystem;

use crate::command::{CmdAction, CmdId, Enablement};
use crate::dock::DockHost;
use crate::input::CameraView;
use crate::jobs::JobTracker;
use crate::module::{ContextId, PanelId, TabId, WorkspaceId};
use crate::nav::{CameraControl, DocCamera, Navigator};
use crate::notify::{Notification, Notifications};
use crate::panel::Panel;
use crate::pick::{CpuPickService, PickService};
use crate::registry::UiRegistry;
use crate::scene::{CpuSceneBackend, SceneAdapter, SceneBackend};
use crate::search::{Recency, SearchState};
use crate::select::{SelItem, Selection};
use crate::shortcut::{ShortcutMap, ShortcutOverrides};
use crate::theme::{Theme, ThemeState, ThemeTokens};
use crate::tool::ToolState;
use crate::viewport::ViewportBackend;

/// Identifies an open document tab for the lifetime of the app (never reused).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct DocTabId(pub u64);

/// One open document and its per-document UI state: camera, navigation, selection and the
/// tool router (UI-03).
#[derive(Debug)]
pub struct DocTab {
    /// Stable id.
    pub id: DocTabId,
    /// Tab label.
    pub name: String,
    /// The document; mutated only at the end of a frame.
    pub document: Document,
    /// Active UI contexts ("sketch", …), outermost first.
    pub contexts: Vec<ContextId>,
    /// The file it was opened from or last saved to (UI-19).
    pub path: Option<PathBuf>,
    /// Crash-recovery journal and snapshots, when recovery is enabled (UI-19).
    pub recovery: Option<Arc<amg_doc::RecoverySession>>,
    /// The view's camera (S10), moved by the navigator (UI-03).
    pub camera: amg_render::Camera,
    /// The selection (UI-03 click selection; UI-04 the rest).
    pub selection: Selection,
    /// Navigation preset state over S10's `NavController`.
    pub(crate) nav: Navigator,
    /// Active tool, hover, preview, input state.
    pub(crate) tools: ToolState,
    /// The newest camera snapshot of the viewport (None before it was drawn).
    pub(crate) view: Option<CameraView>,
    /// Bounds of the displayed scene (navigation pivot and fit).
    pub(crate) bounds: amg_core::Aabb3,
    /// The browser tree, expansion and rename state (UI-07).
    pub(crate) browser: crate::browser::BrowserState,
    /// The timeline cache, selection, drag and rename state (UI-08).
    pub(crate) timeline: crate::timeline::TimelineState,
    /// The sketch opened for editing from the browser or timeline (UI-07 enters the sketch
    /// context; UI-09's sketch session takes it from here).
    pub sketch_edit: Option<amg_doc::FeatureId>,
    /// Module state attached to this document (additive, INS-09; [`crate::layer`]).
    pub(crate) layers: crate::layer::DocLayers,
    /// View flags that are on (INS-10).
    pub(crate) view_flags: std::collections::BTreeSet<&'static str>,
    /// The scene's occurrence-path table (INS-10).
    pub(crate) instance_paths: Vec<amg_doc::OccPath>,
}

/// A group of document commands applied as one undo entry at the end of the frame.
#[derive(Clone, Debug)]
pub struct CommitBatch {
    /// Undo label.
    pub label: String,
    /// The commands, in order.
    pub commands: Vec<DocCommand>,
}

/// The closure of [`ApplyFn`].
pub(crate) type ApplyClosure = Box<dyn FnOnce(&mut Document) -> Result<(), amg_doc::CommandError>>;

/// A closure that mutates the document directly (IO-15: baked-payload commits).
pub(crate) struct ApplyFn(pub(crate) ApplyClosure, pub(crate) String);

impl std::fmt::Debug for ApplyFn {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_tuple("ApplyFn").field(&self.1).finish()
    }
}

#[derive(Debug)]
enum DocAction {
    Commit(CommitBatch),
    Apply(ApplyFn),
    Undo,
    Redo,
}

/// Document mutations requested during a frame, applied by [`AmalgamApp::end_frame`].
#[derive(Debug, Default)]
pub(crate) struct CommandQueue {
    items: Vec<(DocTabId, DocAction)>,
}

impl CommandQueue {
    fn push(&mut self, doc: DocTabId, action: DocAction) {
        self.items.push((doc, action));
    }
}

/// Shell requests from modules, applied after the frame's layout.
#[derive(Debug)]
pub(crate) enum ShellRequest {
    OpenPanel(PanelId),
    FocusContextTab(ContextId),
    RunCommand(CmdId),
    OpenSearch,
    File(DocTabId, crate::file::FileOp),
    RestoreSelectionSet(DocTabId, String),
    /// Replace a highlight layer (additive, MESH-23).
    Highlight(
        DocTabId,
        crate::scene::HighlightLayer,
        Vec<crate::pick::PickTarget>,
    ),

    EditFeature(DocTabId, amg_doc::FeatureId),
    /// Open a module dialog on a document (additive, IO-15).
    OpenDialog(DocTabId, crate::dialog::BoxedDialog),
    /// Show a native file dialog; the answer goes to the sender (additive, IO-15).
    PickFile(
        crate::file::FileDialogRequest,
        std::sync::mpsc::Sender<Option<PathBuf>>,
    ),
}

/// Read-only view for enablement, providers and panels.
///
/// UI-03 adds `selection`, `hovered` and `active_tool`; UI-11 adds `prefs` (additive).
#[derive(Debug)]
pub struct CmdEnv<'a> {
    /// Active workspace.
    pub workspace: WorkspaceId,
    /// Active contexts of the document.
    pub contexts: &'a [ContextId],
    /// Authored state.
    pub design: &'a Design,
    /// Latest evaluated state.
    pub evaluated: &'a Arc<Evaluated>,
    /// The job system.
    pub jobs: &'a JobSystem,
    /// The document's selection.
    pub selection: &'a Selection,
    /// The newest hover pick.
    pub hovered: Option<&'a SelItem>,
    /// The command of the active tool (`None`: the select tool).
    pub active_tool: Option<CmdId>,
    /// The document's view flags that are on ([`AppCx::set_view_flag`]; additive, INS-10).
    pub view_flags: &'a std::collections::BTreeSet<&'static str>,
    /// Occurrence paths by `PickTarget::Body::instance` index, as of the last scene update
    /// (empty before it; additive, INS-10).
    pub instance_paths: &'a [amg_doc::OccPath],
}

/// Mutable access for modules; all document mutation is deferred to the end of the frame.
///
/// UI-02..UI-10 add `start_tool`, `run_command`, `selection_mut`, `notify`, `camera` and
/// `spawn_tracked` (additive).
pub struct AppCx<'a> {
    doc: &'a mut DocTab,
    queue: &'a mut CommandQueue,
    requests: &'a mut Vec<ShellRequest>,
    registry: &'a UiRegistry,
    jobs: &'a JobSystem,
    notifications: &'a mut Notifications,
    tracker: &'a mut JobTracker,
    workspace: WorkspaceId,
    ctx: Option<&'a egui::Context>,
}

impl std::fmt::Debug for AppCx<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AppCx")
            .field("doc", &self.doc.id)
            .field("workspace", &self.workspace)
            .finish_non_exhaustive()
    }
}

impl<'a> AppCx<'a> {
    /// The registry with the context's full lifetime (the browser draws providers while it
    /// holds `&mut self`).
    pub(crate) fn registry_ref(&self) -> &'a UiRegistry {
        self.registry
    }

    #[allow(
        clippy::too_many_arguments,
        reason = "crate-internal constructor over the app's disjoint fields"
    )]
    pub(crate) fn new(
        doc: &'a mut DocTab,
        queue: &'a mut CommandQueue,
        requests: &'a mut Vec<ShellRequest>,
        registry: &'a UiRegistry,
        jobs: &'a JobSystem,
        notifications: &'a mut Notifications,
        tracker: &'a mut JobTracker,
        workspace: WorkspaceId,
        ctx: Option<&'a egui::Context>,
    ) -> AppCx<'a> {
        AppCx {
            doc,
            queue,
            requests,
            registry,
            jobs,
            notifications,
            tracker,
            workspace,
            ctx,
        }
    }
}

impl AppCx<'_> {
    /// The read-only view.
    pub fn env(&self) -> CmdEnv<'_> {
        env_of_doc(self.workspace, self.doc, self.jobs)
    }

    /// The document's view layer of type `T` ([`crate::layer::DocLayer`]; additive, INS-09).
    /// `None` if no such layer is registered, the document has not been active yet, or the
    /// layer itself is running.
    pub fn doc_layer<T: crate::layer::DocLayer>(&mut self) -> Option<&mut T> {
        self.doc.layers.get_mut::<T>()
    }

    /// Turns a per-document view flag (`<module>.<name>`) on or off: view state that is not
    /// saved, read through [`CmdEnv::view_flags`] by toggle commands, panels and layers
    /// (additive, INS-10).
    pub fn set_view_flag(&mut self, flag: &'static str, on: bool) {
        if on {
            self.doc.view_flags.insert(flag);
        } else {
            self.doc.view_flags.remove(flag);
        }
        self.request_repaint();
    }

    /// The document's selection.
    pub fn selection(&self) -> &Selection {
        &self.doc.selection
    }

    /// The document's selection, mutable.
    pub fn selection_mut(&mut self) -> &mut Selection {
        &mut self.doc.selection
    }

    /// Camera control over the document's camera (S10 `NavController`; animated where the
    /// controller animates).
    pub fn camera(&mut self) -> impl CameraControl + '_ {
        if let Some(ctx) = self.ctx {
            ctx.request_repaint();
        }
        let doc = &mut *self.doc;
        let view = doc.view.clone().unwrap_or_else(|| {
            CameraView::new(
                doc.camera,
                egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(800.0, 600.0)),
                1.0,
            )
        });
        DocCamera {
            camera: &mut doc.camera,
            nav: &mut doc.nav,
            view,
            bounds: doc.bounds,
        }
    }

    /// Starts the tool of command `id` at the end of the frame (same as running the command).
    pub fn start_tool(&mut self, id: CmdId) {
        self.run_command(id);
    }

    /// Queues a batch for this document; applied at the end of the frame as one undo entry.
    pub fn submit(&mut self, batch: CommitBatch) {
        self.queue.push(self.doc.id, DocAction::Commit(batch));
    }

    /// Queues a registered command; it runs at the end of the frame if it is enabled then.
    pub fn run_command(&mut self, id: CmdId) {
        self.requests.push(ShellRequest::RunCommand(id));
    }

    /// Opens the S-key command search at the cursor.
    pub fn open_search(&mut self) {
        self.requests.push(ShellRequest::OpenSearch);
    }

    /// Queues restoring the selection set `name` of this document (end of frame, when it is
    /// the active document; additive, UI-04).
    pub fn restore_selection_set(&mut self, name: &str) {
        self.requests.push(ShellRequest::RestoreSelectionSet(
            self.doc.id,
            name.to_owned(),
        ));
    }

    /// Replaces highlight layer `layer` with `targets` at the end of the frame, while this
    /// document is the active one (additive, MESH-23: mesh defect highlighting on the `Error`
    /// layer). Body targets are shown on every displayed instance of their body (their
    /// `instance` is ignored). The router owns the `Hover` and `Selected` layers.
    pub fn set_highlight(
        &mut self,
        layer: crate::scene::HighlightLayer,
        targets: Vec<crate::pick::PickTarget>,
    ) {
        self.requests
            .push(ShellRequest::Highlight(self.doc.id, layer, targets));
    }

    /// Queues editing feature `id` of this document (end of frame): a sketch enters sketch
    /// mode; another feature starts the tool of its `feature.<kind>` command (UI-05) with
    /// `ToolInit::edit = Some(id)`; without one an info toast says so (additive, UI-07).
    pub fn edit_feature(&mut self, id: amg_doc::FeatureId) {
        self.requests
            .push(ShellRequest::EditFeature(self.doc.id, id));
    }

    /// Queues a file operation (New, Open, Save, …) on this document; it runs at the end of
    /// the frame (UI-19).
    pub fn file(&mut self, op: crate::file::FileOp) {
        self.requests.push(ShellRequest::File(self.doc.id, op));
    }

    /// Opens a module dialog on this document (see [`crate::dialog::AppDialog`]); it is drawn
    /// from the next frame on (additive, IO-15).
    pub fn open_dialog(&mut self, dialog: Box<dyn crate::dialog::AppDialog>) {
        self.requests.push(ShellRequest::OpenDialog(
            self.doc.id,
            crate::dialog::BoxedDialog(dialog),
        ));
    }

    /// Queues a direct mutation of this document, applied at the end of the frame as one
    /// undo entry labelled `label` (additive, IO-15). For commits [`CommitBatch`] cannot
    /// express, such as `Document::add_feature_baked`. An `Err` is shown as a toast.
    pub fn apply_document(
        &mut self,
        label: impl Into<String>,
        f: impl FnOnce(&mut Document) -> Result<(), amg_doc::CommandError> + 'static,
    ) {
        self.queue.push(
            self.doc.id,
            DocAction::Apply(ApplyFn(Box::new(f), label.into())),
        );
    }

    /// Shows a native file dialog on the dialog thread; the path (or `None` for a cancel) comes
    /// back through the receiver. If another file dialog is open the answer is `None`
    /// (additive, IO-15).
    pub fn pick_file(
        &mut self,
        req: crate::file::FileDialogRequest,
    ) -> std::sync::mpsc::Receiver<Option<PathBuf>> {
        let (tx, rx) = std::sync::mpsc::channel();
        self.requests.push(ShellRequest::PickFile(req, tx));
        rx
    }

    /// Queues undo for this document (applied at the end of the frame).
    pub fn undo(&mut self) {
        self.queue.push(self.doc.id, DocAction::Undo);
    }

    /// Queues redo for this document (applied at the end of the frame).
    pub fn redo(&mut self) {
        self.queue.push(self.doc.id, DocAction::Redo);
    }

    /// Opens (or focuses) a registered panel in the current workspace's dock area.
    pub fn open_panel(&mut self, id: PanelId) {
        self.requests.push(ShellRequest::OpenPanel(id));
    }

    /// Activates a context; its contextual tab is shown and takes focus.
    pub fn enter_context(&mut self, id: ContextId) {
        if !self.doc.contexts.contains(&id) {
            self.doc.contexts.push(id);
        }
        self.requests.push(ShellRequest::FocusContextTab(id));
    }

    /// Deactivates a context.
    pub fn exit_context(&mut self, id: ContextId) {
        self.doc.contexts.retain(|c| *c != id);
    }

    /// The UI registry.
    pub fn registry(&self) -> &UiRegistry {
        self.registry
    }

    /// The job system.
    pub fn jobs(&self) -> &JobSystem {
        self.jobs
    }

    /// The document tab (read-only).
    pub fn doc_tab(&self) -> &DocTab {
        self.doc
    }

    /// Shows a toast.
    pub fn notify(&mut self, n: Notification) {
        self.notifications.push(n);
        self.request_repaint();
    }

    /// Shows a toast for a diagnostic. A cancellation shows nothing (cancelled is not an
    /// error).
    pub fn notify_error(&mut self, e: &dyn amg_core::Diagnostic) {
        if let Some(n) = Notification::from_diagnostic(e) {
            self.notify(n);
        }
    }

    /// Spawns a job whose result comes back on the UI thread: `on_done` runs with `&mut AppCx`
    /// in a later frame's `logic`, once the job finished. A failure (not a cancellation)
    /// is shown as a toast before `on_done` runs, so tools need no error handling of
    /// their own.
    pub fn spawn_tracked<J: amg_jobs::Job>(
        &mut self,
        job: J,
        opts: amg_jobs::SpawnOptions,
        on_done: impl FnOnce(Result<J::Output, amg_jobs::JobError<J::Error>>, &mut AppCx<'_>) + 'static,
    ) -> amg_jobs::JobId
    where
        J::Error: amg_core::Diagnostic,
    {
        self.tracker.spawn(self.jobs, job, opts, on_done)
    }

    /// Asks for another frame (only for work the job notifier does not cover).
    pub fn request_repaint(&self) {
        if let Some(ctx) = self.ctx {
            ctx.request_repaint();
        }
    }

    /// Asks for a frame after `after` (timers such as the select-through long press;
    /// additive, UI-04).
    pub fn request_repaint_after(&self, after: std::time::Duration) {
        if let Some(ctx) = self.ctx {
            ctx.request_repaint_after(after);
        }
    }
}

/// The read-only view of one document (`hovered` and `active_tool` from its router state).
pub(crate) fn env_of_doc<'a>(
    workspace: WorkspaceId,
    doc: &'a DocTab,
    jobs: &'a JobSystem,
) -> CmdEnv<'a> {
    CmdEnv {
        workspace,
        contexts: &doc.contexts,
        design: doc.document.design(),
        evaluated: doc.document.evaluated(),
        jobs,
        selection: &doc.selection,
        hovered: doc.tools.frame.hover.as_ref().map(|h| &h.item),
        active_tool: doc.tools.active.as_ref().map(|a| a.id),
        view_flags: &doc.view_flags,
        instance_paths: &doc.instance_paths,
    }
}

/// A `JobSystemConfig::notifier` that repaints the egui context once it is known.
///
/// The context is filled in by eframe's creation callback (or the test harness) and by the
/// first [`AmalgamApp::logic`]. Cheap and callable from any thread.
pub fn repaint_notifier(slot: &Arc<OnceLock<egui::Context>>) -> Arc<dyn Fn() + Send + Sync> {
    let slot = Arc::clone(slot);
    Arc::new(move || {
        if let Some(ctx) = slot.get() {
            ctx.request_repaint();
        }
    })
}

/// How [`AmalgamApp::new`] is set up.
pub struct AppConfig {
    /// The job system; its notifier should be [`repaint_notifier`] of `repaint`.
    pub jobs: Arc<JobSystem>,
    /// The context slot the notifier repaints.
    pub repaint: Arc<OnceLock<egui::Context>>,
    /// Draws the viewport tab.
    pub viewport: Box<dyn ViewportBackend>,
    /// Everything the modules registered.
    pub registry: UiRegistry,
    /// Initial theme.
    pub theme: Theme,
    /// Where dock layouts are loaded from and saved to (`None`: not persisted).
    pub layout_path: Option<PathBuf>,
}

impl std::fmt::Debug for AppConfig {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AppConfig")
            .field("registry", &self.registry)
            .field("theme", &self.theme)
            .field("layout_path", &self.layout_path)
            .finish_non_exhaustive()
    }
}

/// The application: open documents, workspace, theme, dock layouts and panels.
pub struct AmalgamApp {
    pub(crate) registry: UiRegistry,
    pub(crate) jobs: Arc<JobSystem>,
    repaint: Arc<OnceLock<egui::Context>>,
    pub(crate) ctx: Option<egui::Context>,
    pub(crate) docs: Vec<DocTab>,
    pub(crate) active: usize,
    next_doc: u64,
    untitled: u32,
    pub(crate) workspace: WorkspaceId,
    pub(crate) active_tab: BTreeMap<WorkspaceId, TabId>,
    pub(crate) theme: Theme,
    theme_state: ThemeState,
    pub(crate) tokens: ThemeTokens,
    pub(crate) layouts: BTreeMap<WorkspaceId, DockHost>,
    pub(crate) panels: BTreeMap<PanelId, Box<dyn Panel>>,
    pub(crate) viewport: Box<dyn ViewportBackend>,
    pub(crate) queue: CommandQueue,
    pub(crate) requests: Vec<ShellRequest>,
    pub(crate) status: Option<String>,
    layout_path: Option<PathBuf>,
    frames: u64,
    shortcut_overrides: ShortcutOverrides,
    pub(crate) shortcuts: ShortcutMap,
    pub(crate) search: SearchState,
    pub(crate) recency: Recency,
    /// The pointer was over the viewport last frame (enables the `Viewport` shortcut scope).
    pub(crate) viewport_hovered: bool,
    pub(crate) notifications: Notifications,
    pub(crate) tracker: JobTracker,
    /// The job panel is expanded (toggled by the status bar's `Jobs: N`).
    pub(crate) jobs_panel_open: bool,
    log_dir: PathBuf,
    folder_opener: Box<dyn FnMut(&std::path::Path)>,
    /// `DocEvent` → render scene (UI-20); shows the active document.
    pub(crate) scene: SceneAdapter,
    /// The document the scene shows (`None`: rebuild on the next `logic`).
    scene_doc: Option<DocTabId>,
    /// File dialogs, save/open jobs, close prompt and crash recovery (UI-19).
    pub(crate) files: crate::file::FileState,
    /// Screen-space picking (UI-03): the viewport's GPU service, else the CPU one.
    pub(crate) picks: Box<dyn PickService>,
    /// Open module dialogs with the document they belong to (additive, IO-15).
    pub(crate) dialogs: Vec<(DocTabId, Box<dyn crate::dialog::AppDialog>)>,
}

impl std::fmt::Debug for AmalgamApp {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AmalgamApp")
            .field(
                "docs",
                &self.docs.iter().map(|d| &d.name).collect::<Vec<_>>(),
            )
            .field("active", &self.active)
            .field("workspace", &self.workspace)
            .field("theme", &self.theme)
            .field("frames", &self.frames)
            .finish_non_exhaustive()
    }
}

/// Feature kind of sketches (S03); double-click opens sketch mode instead of a dialog.
pub const SKETCH_FEATURE_KIND: &str = "sketch.sketch";

/// Fallback workspace id when no module registered any workspace.
const NO_WORKSPACE: WorkspaceId = WorkspaceId("design");

impl AmalgamApp {
    /// An app with one empty document in the first workspace.
    pub fn new(cfg: AppConfig) -> AmalgamApp {
        let workspace = cfg.registry.default_workspace().unwrap_or(NO_WORKSPACE);
        let mut viewport = cfg.viewport;
        let picks = viewport
            .pick_service()
            .unwrap_or_else(|| Box::new(CpuPickService::new()));
        let mut app = AmalgamApp {
            registry: cfg.registry,
            jobs: cfg.jobs,
            repaint: cfg.repaint,
            ctx: None,
            docs: Vec::new(),
            active: 0,
            next_doc: 1,
            untitled: 0,
            workspace,
            active_tab: BTreeMap::new(),
            theme: cfg.theme,
            theme_state: ThemeState::default(),
            tokens: ThemeTokens::for_theme(cfg.theme.resolve(None)),
            layouts: BTreeMap::new(),
            panels: BTreeMap::new(),
            viewport,
            queue: CommandQueue::default(),
            requests: Vec::new(),
            status: None,
            layout_path: cfg.layout_path,
            frames: 0,
            shortcut_overrides: ShortcutOverrides::none(),
            shortcuts: ShortcutMap::default(),
            search: SearchState::default(),
            recency: Recency::new(),
            viewport_hovered: false,
            notifications: Notifications::new(),
            tracker: JobTracker::default(),
            jobs_panel_open: false,
            log_dir: amg_jobs::diag::default_log_dir(),
            folder_opener: Box::new(open_in_file_manager),
            scene: SceneAdapter::new(Box::new(CpuSceneBackend::new()), Vec::new()),
            scene_doc: None,
            files: crate::file::FileState::default(),
            picks,
            dialogs: Vec::new(),
        };
        app.scene = SceneAdapter::new(
            Box::new(CpuSceneBackend::new()),
            app.registry.tessellators(),
        );
        app.shortcuts = ShortcutMap::build(&app.registry, &app.shortcut_overrides);
        app.load_layouts();
        app.new_document();
        app.ensure_dock(workspace);
        app
    }

    // ---- frame loop ---------------------------------------------------------------------

    /// One whole frame: [`Self::logic`] then [`Self::ui`] (which ends with
    /// [`Self::end_frame`]). For hosts without the eframe split (kittest).
    pub fn frame(&mut self, ui: &mut egui::Ui) {
        let _span = tracing::debug_span!("amg_ui.frame").entered();
        self.logic(ui.ctx());
        self.ui(ui);
    }

    /// Per-frame work without layout: theme, document polling and events.
    pub fn logic(&mut self, ctx: &egui::Context) {
        let _span = tracing::debug_span!("amg_ui.logic").entered();
        if self.ctx.is_none() {
            let _ = self.repaint.set(ctx.clone());
            self.ctx = Some(ctx.clone());
        }
        crate::icons::install(ctx, &self.registry);
        if let Some(tokens) = self.theme_state.apply(ctx, self.theme) {
            self.tokens = tokens;
        }
        if self.shortcuts.generation() != self.registry.generation() {
            self.shortcuts = ShortcutMap::build(&self.registry, &self.shortcut_overrides);
        }
        self.poll_tracked_jobs();
        let jobs = Arc::clone(&self.jobs);
        let scene_stale = self.scene_doc != self.docs.get(self.active).map(|d| d.id);
        for (i, doc) in self.docs.iter_mut().enumerate() {
            doc.document.poll(&jobs);
            let events = doc.document.drain_events();
            if !events.is_empty() {
                doc.selection
                    .revalidate(&events, doc.document.design(), doc.document.evaluated());
                doc.browser.on_events(&events, &self.registry);
            }
            if i == self.active {
                if scene_stale {
                    self.scene
                        .rebuild(doc.document.design(), doc.document.evaluated(), &jobs);
                    self.scene_doc = Some(doc.id);
                } else {
                    self.scene.handle_events(
                        &events,
                        doc.document.design(),
                        doc.document.evaluated(),
                        &jobs,
                    );
                }
            }
            if i == self.active && !events.is_empty() {
                let env = env_of_doc(self.workspace, doc, &jobs);
                for panel in self.panels.values_mut() {
                    panel.on_doc_events(&events, &env);
                }
            }
        }
        self.poll_scene();
        self.route_logic();
        self.poll_files(ctx);
        self.poll_dropped_files(ctx);
    }

    /// Inserts finished tessellations and keeps the highlight colours on the theme. Never
    /// blocks.
    fn poll_scene(&mut self) {
        self.scene.set_theme(&self.tokens);
        self.scene.poll();
        for (body, err) in self.scene.take_failures() {
            self.notifications.push(Notification::error(
                "Cannot display a body",
                format!("{body:?}: {err}"),
            ));
        }
    }

    /// Runs the callbacks of tracked jobs that finished. Never blocks.
    fn poll_tracked_jobs(&mut self) {
        if self.tracker.entries.is_empty() || self.docs.is_empty() {
            return;
        }
        let mut entries = std::mem::take(&mut self.tracker.entries);
        self.with_cx(|cx| entries.retain_mut(|poll| !poll(cx)));
        // Jobs spawned by the callbacks went to the (emptied) tracker.
        entries.append(&mut self.tracker.entries);
        self.tracker.entries = entries;
    }

    /// Layout and widgets, then [`Self::end_frame`].
    pub fn ui(&mut self, ui: &mut egui::Ui) {
        let _span = tracing::debug_span!("amg_ui.ui").entered();
        crate::shell::draw(self, ui);
        self.end_frame();
        self.viewport.after_frame();
    }

    /// Applies the frame's queued document mutations and shell requests.
    ///
    /// This frame's widgets were drawn before the changes, so a frame that applied anything
    /// asks for one more to show the result (undo state, errors, new tabs).
    pub fn end_frame(&mut self) {
        self.run_queued_commands();
        let changed = !self.queue.items.is_empty() || !self.requests.is_empty();
        if changed && let Some(ctx) = &self.ctx {
            ctx.request_repaint();
        }
        for (doc_id, action) in std::mem::take(&mut self.queue.items) {
            let Some(doc) = self.docs.iter_mut().find(|d| d.id == doc_id) else {
                continue;
            };
            match action {
                DocAction::Commit(batch) => {
                    if let Err(f) = apply_batch(&mut doc.document, batch) {
                        let msg = format!("{} failed: {}", f.label, f.error);
                        tracing::warn!("{msg}");
                        self.status = Some(msg);
                        self.notifications.push(
                            Notification::error("Edit failed", format!("{}: {}", f.label, f.error))
                                .with_details(amg_core::report(&f.error)),
                        );
                    }
                }
                DocAction::Apply(ApplyFn(f, label)) => {
                    if let Err(e) = f(&mut doc.document) {
                        let msg = format!("{label} failed: {e}");
                        tracing::warn!("{msg}");
                        self.status = Some(msg);
                        self.notifications.push(
                            Notification::error("Edit failed", format!("{label}: {e}"))
                                .with_details(amg_core::report(&e)),
                        );
                    }
                }
                DocAction::Undo => {
                    doc.document.undo();
                }
                DocAction::Redo => {
                    doc.document.redo();
                }
            }
        }
        for req in std::mem::take(&mut self.requests) {
            match req {
                ShellRequest::OpenPanel(id) => self.open_panel(id),
                ShellRequest::RunCommand(_) => {}
                ShellRequest::File(doc, op) => self.file_op(Some(doc), op),
                ShellRequest::EditFeature(doc, feature) => {
                    if self.docs.get(self.active).is_some_and(|d| d.id == doc) {
                        self.edit_feature(feature);
                    }
                }
                ShellRequest::RestoreSelectionSet(doc, name) => {
                    if self.docs.get(self.active).is_some_and(|d| d.id == doc) {
                        self.restore_selection_set(&name);
                    }
                }
                ShellRequest::Highlight(doc, layer, targets) => {
                    if self.docs.get(self.active).is_some_and(|d| d.id == doc) {
                        let list = crate::tool::all_instances(&self.scene, &targets);
                        self.scene.set_highlight(layer, &list);
                    }
                }
                ShellRequest::OpenDialog(doc, d) => self.dialogs.push((doc, d.0)),
                ShellRequest::PickFile(req, tx) => self.pick_file_for(req, tx),
                ShellRequest::OpenSearch => {
                    let at = self
                        .ctx
                        .as_ref()
                        .and_then(|c| c.input(|i| i.pointer.latest_pos()));
                    self.search.open(at);
                }
                ShellRequest::FocusContextTab(c) => {
                    let tab = self
                        .registry
                        .all_tabs(self.workspace)
                        .into_iter()
                        .find(|t| t.contextual == Some(c))
                        .map(|t| t.id);
                    if let Some(tab) = tab {
                        self.active_tab.insert(self.workspace, tab);
                    }
                }
            }
        }
        self.frames += 1;
    }

    /// Runs the commands requested so far (and those they request, up to a depth of 16).
    fn run_queued_commands(&mut self) {
        for _ in 0..16 {
            let mut cmds = Vec::new();
            self.requests.retain(|r| match r {
                ShellRequest::RunCommand(id) => {
                    cmds.push(*id);
                    false
                }
                _ => true,
            });
            if cmds.is_empty() {
                return;
            }
            for id in cmds {
                self.execute_command(id);
            }
        }
        tracing::warn!("commands keep requesting commands; the rest is dropped");
        self.requests
            .retain(|r| !matches!(r, ShellRequest::RunCommand(_)));
    }

    /// Runs a command now: checks its enablement, records it for search recency and applies its
    /// action. Returns false if the command is unknown or disabled.
    fn execute_command(&mut self, id: CmdId) -> bool {
        let Some(def) = self.registry.command(id) else {
            tracing::warn!("run_command: unknown command `{id}`");
            return false;
        };
        let action = def.action.clone();
        let doc = &self.docs[self.active];
        let env = env_of_doc(self.workspace, doc, &self.jobs);
        if let Enablement::Disabled(reason) = def.enablement(&env) {
            tracing::debug!("command `{id}` is disabled: {reason}");
            return false;
        }
        let checked = match &action {
            CmdAction::Toggle { get, .. } => get(&env),
            _ => false,
        };
        self.recency.record(id);
        match action {
            CmdAction::StartTool(f) => self.start_tool_with(id, f),
            CmdAction::Run(f) => self.with_cx(f),
            CmdAction::Toggle { set, .. } => self.with_cx(|cx| set(cx, !checked)),
            CmdAction::Dyn(f) => self.with_cx(|cx| f(cx)),
        }
        true
    }

    /// Edits feature `id` of the active document now (see [`AppCx::edit_feature`]).
    pub fn edit_feature(&mut self, id: amg_doc::FeatureId) {
        let kind = {
            let doc = &self.docs[self.active];
            match doc.document.design().timeline().node(id) {
                Some((_, node)) => node.feature.kind().to_owned(),
                None => return,
            }
        };
        // The kind's timeline info decides first (UI-08); without one, the UI-07 convention.
        match self.registry.feature_ui(&kind).map(|i| i.edit) {
            Some(crate::timeline::EditAction::StartTool(f)) => {
                let cmd = format!("feature.{kind}");
                let cid = self
                    .registry
                    .commands()
                    .find(|c| c.id.0 == cmd)
                    .map_or(crate::timeline::EDIT_TOOL_CMD, |c| c.id);
                self.start_tool_editing(cid, f, Some(id));
                return;
            }
            Some(crate::timeline::EditAction::NotEditable) => {
                self.notify(Notification::info(
                    "This feature can't be edited",
                    format!("`{kind}` has no editor."),
                ));
                return;
            }
            Some(crate::timeline::EditAction::SketchMode) => {
                self.docs[self.active].sketch_edit = Some(id);
                self.with_cx(|cx| cx.enter_context(crate::design::SKETCH_CONTEXT));
                return;
            }
            Some(crate::timeline::EditAction::GenericDialog) | None => {}
        }
        if kind == SKETCH_FEATURE_KIND {
            self.docs[self.active].sketch_edit = Some(id);
            self.with_cx(|cx| cx.enter_context(crate::design::SKETCH_CONTEXT));
            return;
        }
        let cmd = format!("feature.{kind}");
        let tool = self
            .registry
            .commands()
            .find(|c| c.id.0 == cmd)
            .and_then(|c| match c.action {
                CmdAction::StartTool(f) => Some((c.id, f)),
                _ => None,
            });
        match tool {
            Some((cid, f)) => self.start_tool_editing(cid, f, Some(id)),
            None => self.notify(Notification::info(
                "Cannot edit this feature yet",
                format!("No editor is registered for `{kind}`."),
            )),
        }
    }

    /// Asks for a registered command to run at the end of this (or the next) frame. This is
    /// what toolbar buttons, menus, shortcuts and search use.
    pub fn request_command(&mut self, id: CmdId) {
        self.requests.push(ShellRequest::RunCommand(id));
        self.repaint();
    }

    /// The registry, mutable: runtime registration of add-in commands
    /// ([`UiRegistry::register_dynamic`]). Shortcuts and search follow on the next frame.
    pub fn registry_mut(&mut self) -> &mut UiRegistry {
        self.repaint();
        &mut self.registry
    }

    /// The effective shortcut bindings.
    pub fn shortcuts(&self) -> &ShortcutMap {
        &self.shortcuts
    }

    /// Replaces the user's shortcut overrides and rebuilds the map.
    pub fn set_shortcut_overrides(&mut self, overrides: ShortcutOverrides) {
        self.shortcut_overrides = overrides;
        self.shortcuts = ShortcutMap::build(&self.registry, &self.shortcut_overrides);
        self.repaint();
    }

    /// Loads `shortcuts.toml`. A missing file is not an error; a malformed one is reported
    /// and leaves the current bindings alone.
    ///
    /// # Errors
    /// The parse error of a malformed file.
    pub fn load_shortcuts_file(
        &mut self,
        path: &std::path::Path,
    ) -> Result<(), crate::shortcut::ShortcutFileError> {
        let Ok(text) = std::fs::read_to_string(path) else {
            return Ok(());
        };
        let o = ShortcutOverrides::parse(&text)?;
        self.set_shortcut_overrides(o);
        Ok(())
    }

    /// The S-key search state.
    pub fn search(&self) -> &SearchState {
        &self.search
    }

    /// The S-key search state, mutable.
    pub fn search_mut(&mut self) -> &mut SearchState {
        &mut self.search
    }

    /// The most recently run commands (search recency).
    pub fn recent_commands(&self) -> &Recency {
        &self.recency
    }

    /// The scene adapter (UI-20).
    pub fn scene(&self) -> &SceneAdapter {
        &self.scene
    }

    /// The scene adapter, mutable (highlights, previews, placement overlays).
    pub fn scene_mut(&mut self) -> &mut SceneAdapter {
        &mut self.scene
    }

    /// Replaces the render backend (the app passes S10's `GpuSceneBackend`; the default is a
    /// `CpuSceneBackend`). The scene is rebuilt from the active document on the next frame.
    pub fn set_scene_backend(&mut self, backend: Box<dyn SceneBackend>) {
        self.scene.clear();
        self.scene = SceneAdapter::new(backend, self.registry.tessellators());
        self.scene_doc = None;
        self.repaint();
    }

    /// The scene shows the active document again from scratch on the next `logic`.
    pub(crate) fn invalidate_scene(&mut self) {
        self.scene_doc = None;
    }

    /// Number of frames run so far.
    pub fn frame_count(&self) -> u64 {
        self.frames
    }

    // ---- documents ----------------------------------------------------------------------

    /// Opens a new empty document ("Untitled N") and makes it active.
    pub fn new_document(&mut self) -> DocTabId {
        self.untitled += 1;
        let name = format!("Untitled {}", self.untitled);
        let mut document = Document::new(&name);
        // Nothing to save yet: an untouched new document closes without a prompt (UI-19).
        document.history_mut().mark_saved();
        self.add_document(name, document)
    }

    /// Adds a document as a new tab and makes it active.
    pub fn add_document(&mut self, name: String, document: Document) -> DocTabId {
        let id = DocTabId(self.next_doc);
        self.next_doc += 1;
        self.docs.push(DocTab {
            id,
            name,
            document,
            contexts: Vec::new(),
            path: None,
            recovery: None,
            camera: crate::nav::home_camera(),
            selection: Selection::default(),
            nav: Navigator::default(),
            tools: ToolState::default(),
            view: None,
            bounds: amg_core::Aabb3::EMPTY,
            browser: crate::browser::BrowserState::default(),
            timeline: crate::timeline::TimelineState::default(),
            sketch_edit: None,
            layers: crate::layer::DocLayers::default(),
            view_flags: std::collections::BTreeSet::new(),
            instance_paths: Vec::new(),
        });
        self.active = self.docs.len() - 1;
        self.start_recovery_session(id);
        self.repaint();
        id
    }

    /// Makes a document tab active. Returns false if it does not exist.
    pub fn activate(&mut self, id: DocTabId) -> bool {
        match self.docs.iter().position(|d| d.id == id) {
            Some(i) => {
                self.active = i;
                self.repaint();
                true
            }
            None => false,
        }
    }

    /// Open document tabs, in tab order.
    pub fn documents(&self) -> &[DocTab] {
        &self.docs
    }

    /// The active document tab.
    pub fn active_document(&self) -> &DocTab {
        &self.docs[self.active]
    }

    /// The active document tab, mutable (tests and hosts; widgets use [`AppCx`]).
    pub fn active_document_mut(&mut self) -> &mut DocTab {
        &mut self.docs[self.active]
    }

    /// Queues a batch for the active document (applied at the end of the next frame).
    pub fn submit(&mut self, batch: CommitBatch) {
        let id = self.docs[self.active].id;
        self.queue.push(id, DocAction::Commit(batch));
    }

    /// Queues undo for a document.
    pub fn request_undo(&mut self, doc: DocTabId) {
        self.queue.push(doc, DocAction::Undo);
    }

    /// Queues redo for a document.
    pub fn request_redo(&mut self, doc: DocTabId) {
        self.queue.push(doc, DocAction::Redo);
    }

    /// Runs `f` with an [`AppCx`] on the active document.
    pub fn with_cx<R>(&mut self, f: impl FnOnce(&mut AppCx<'_>) -> R) -> R {
        let (mut cx, _, _) = self.tool_parts();
        f(&mut cx)
    }

    /// An [`AppCx`] on the active document, the pick service and the scene (disjoint
    /// borrows).
    pub(crate) fn tool_parts(&mut self) -> (AppCx<'_>, &mut dyn PickService, &SceneAdapter) {
        let cx = AppCx::new(
            &mut self.docs[self.active],
            &mut self.queue,
            &mut self.requests,
            &self.registry,
            &self.jobs,
            &mut self.notifications,
            &mut self.tracker,
            self.workspace,
            self.ctx.as_ref(),
        );
        (cx, self.picks.as_mut(), &self.scene)
    }

    /// The read-only view of `doc` with an explicit hover item and active tool (the router
    /// holds the document's tool state while it calls a tool factory).
    pub(crate) fn env_for<'a>(
        &'a self,
        doc: &'a DocTab,
        hovered: Option<&'a SelItem>,
        active_tool: Option<CmdId>,
    ) -> CmdEnv<'a> {
        CmdEnv {
            hovered,
            active_tool,
            ..env_of_doc(self.workspace, doc, &self.jobs)
        }
    }

    /// The pick service (the viewport's GPU one in the app, `CpuPickService` without one).
    pub fn pick_service(&mut self) -> &mut dyn PickService {
        self.picks.as_mut()
    }

    /// Replaces the pick service.
    pub fn set_pick_service(&mut self, picks: Box<dyn PickService>) {
        self.picks = picks;
    }

    /// The active document's navigator (preset bindings, S10 controller).
    pub fn navigator(&self) -> &Navigator {
        &self.docs[self.active].nav
    }

    /// Sets the navigation preset of every open document (UI-11 persists it).
    pub fn set_nav_preset(&mut self, preset: crate::nav::NavPreset) {
        for d in &mut self.docs {
            d.nav.set_bindings(preset.bindings());
        }
    }

    /// The newest camera snapshot of the active document's viewport.
    pub fn camera_view(&self) -> Option<&CameraView> {
        self.docs[self.active].view.as_ref()
    }

    /// The last commit-batch error shown in the status bar (it is also a toast).
    pub fn status_message(&self) -> Option<&str> {
        self.status.as_deref()
    }

    // ---- notifications and jobs ---------------------------------------------------------

    /// Shows a toast.
    pub fn notify(&mut self, n: Notification) {
        self.notifications.push(n);
        self.repaint();
    }

    /// Shows a toast for a diagnostic; a cancellation shows nothing.
    pub fn notify_error(&mut self, e: &dyn amg_core::Diagnostic) {
        if let Some(n) = Notification::from_diagnostic(e) {
            self.notify(n);
        }
    }

    /// The toasts currently shown.
    pub fn notifications(&self) -> &Notifications {
        &self.notifications
    }

    /// Dismisses a toast.
    pub fn dismiss_notification(&mut self, id: u64) {
        self.notifications.dismiss(id);
        self.repaint();
    }

    /// Moves the toast clock forward (tests: the 6 s auto-hide without sleeping).
    pub fn advance_notification_clock(&mut self, by: std::time::Duration) {
        self.notifications.advance_clock(by);
        self.repaint();
    }

    /// Like [`AppCx::spawn_tracked`], for the host and tests.
    pub fn spawn_tracked<J: amg_jobs::Job>(
        &mut self,
        job: J,
        opts: amg_jobs::SpawnOptions,
        on_done: impl FnOnce(Result<J::Output, amg_jobs::JobError<J::Error>>, &mut AppCx<'_>) + 'static,
    ) -> amg_jobs::JobId
    where
        J::Error: amg_core::Diagnostic,
    {
        self.tracker.spawn(&self.jobs, job, opts, on_done)
    }

    /// Whether the job panel is expanded.
    pub fn jobs_panel_open(&self) -> bool {
        self.jobs_panel_open
    }

    /// Expands or collapses the job panel.
    pub fn set_jobs_panel_open(&mut self, open: bool) {
        self.jobs_panel_open = open;
        self.repaint();
    }

    /// True if the GPU device was lost (the banner is shown; nothing else is disabled).
    pub fn device_lost(&self) -> bool {
        self.viewport.device_lost()
    }

    /// The folder "Open log folder" opens.
    pub fn log_dir(&self) -> &std::path::Path {
        &self.log_dir
    }

    /// Sets the log folder (default: `amg_jobs::diag::default_log_dir()`).
    pub fn set_log_dir(&mut self, dir: PathBuf) {
        self.log_dir = dir;
    }

    /// Replaces what "Open log folder" does (tests record the path instead of opening a
    /// file manager).
    pub fn set_folder_opener(&mut self, opener: impl FnMut(&std::path::Path) + 'static) {
        self.folder_opener = Box::new(opener);
    }

    /// Opens the log folder.
    pub fn open_log_folder(&mut self) {
        let dir = self.log_dir.clone();
        (self.folder_opener)(&dir);
    }

    // ---- workspaces, tabs, panels -------------------------------------------------------

    /// The registry.
    pub fn registry(&self) -> &UiRegistry {
        &self.registry
    }

    /// The job system.
    pub fn jobs(&self) -> &Arc<JobSystem> {
        &self.jobs
    }

    /// The active workspace.
    pub fn workspace(&self) -> WorkspaceId {
        self.workspace
    }

    /// Switches workspace: runs the exit/enter hooks and shows that workspace's tabs and dock
    /// layout. Never touches the documents.
    pub fn set_workspace(&mut self, id: WorkspaceId) {
        if id == self.workspace || self.registry.workspace(id).is_none() {
            return;
        }
        if let Some(exit) = self
            .registry
            .workspace(self.workspace)
            .and_then(|w| w.on_exit)
        {
            self.with_cx(exit);
        }
        self.workspace = id;
        self.ensure_dock(id);
        self.repaint();
        if let Some(enter) = self.registry.workspace(id).and_then(|w| w.on_enter) {
            self.with_cx(enter);
        }
    }

    /// The selected toolbar tab of the active workspace (first visible tab by default).
    pub fn active_tab(&self) -> Option<TabId> {
        let visible = self
            .registry
            .visible_tabs(self.workspace, &self.docs[self.active].contexts);
        let chosen = self.active_tab.get(&self.workspace).copied();
        match chosen {
            Some(t) if visible.iter().any(|v| v.id == t) => Some(t),
            _ => visible.first().map(|t| t.id),
        }
    }

    /// Selects a toolbar tab of the active workspace.
    pub fn select_tab(&mut self, id: TabId) {
        if self
            .registry
            .tab(id)
            .is_some_and(|t| t.workspace == self.workspace)
        {
            self.active_tab.insert(self.workspace, id);
            self.repaint();
        }
    }

    /// The dock area of the active workspace.
    pub fn dock(&self) -> Option<&DockHost> {
        self.layouts.get(&self.workspace)
    }

    /// Opens (or focuses) a panel in the active workspace's dock area.
    pub fn open_panel(&mut self, id: PanelId) {
        let Some(def) = self.registry.panel(id) else {
            tracing::warn!("open_panel: unknown panel `{id}`");
            return;
        };
        if def
            .workspaces
            .as_ref()
            .is_some_and(|ws| !ws.contains(&self.workspace))
        {
            return;
        }
        let slot = def.slot;
        self.panels.entry(id).or_insert_with(def.create);
        self.ensure_dock(self.workspace);
        if let Some(dock) = self.layouts.get_mut(&self.workspace) {
            dock.open(id, slot);
        }
    }

    fn ensure_dock(&mut self, ws: WorkspaceId) {
        let fresh = match self.layouts.entry(ws) {
            std::collections::btree_map::Entry::Vacant(e) => {
                e.insert(DockHost::new());
                true
            }
            std::collections::btree_map::Entry::Occupied(_) => false,
        };
        if fresh {
            let defaults = self
                .registry
                .workspace(ws)
                .map(|w| w.default_panels.clone())
                .unwrap_or_default();
            let prev = self.workspace;
            self.workspace = ws;
            for p in defaults {
                self.open_panel(p);
            }
            self.workspace = prev;
        }
        // Instantiate panels restored from a saved layout.
        let reg = &self.registry;
        if let Some(dock) = self.layouts.get(&ws) {
            for tab in dock.tabs() {
                if let crate::dock::DockTab::Panel(p) = tab
                    && let Some(def) = reg.panel(p)
                {
                    self.panels.entry(p).or_insert_with(def.create);
                }
            }
        }
    }

    // ---- theme --------------------------------------------------------------------------

    /// The theme choice.
    pub fn theme(&self) -> Theme {
        self.theme
    }

    /// Sets the theme; applied at the start of the next frame.
    pub fn set_theme(&mut self, theme: Theme) {
        self.theme = theme;
        self.repaint();
    }

    /// One more frame: state changed after this frame's widgets were drawn.
    fn repaint(&self) {
        if let Some(ctx) = &self.ctx {
            ctx.request_repaint();
        }
    }

    /// Current theme colours.
    pub fn tokens(&self) -> &ThemeTokens {
        &self.tokens
    }

    // ---- layout persistence -------------------------------------------------------------

    /// All workspace layouts as JSON (`{"version":1,"workspaces":{id: layout}}`).
    pub fn layouts_json(&self) -> serde_json::Value {
        let ws: serde_json::Map<String, serde_json::Value> = self
            .layouts
            .iter()
            .map(|(id, dock)| (id.0.to_owned(), dock.to_json()))
            .collect();
        serde_json::json!({ "version": LAYOUT_VERSION, "workspaces": ws })
    }

    /// Restores workspace layouts saved by [`Self::layouts_json`]. Unknown workspaces and
    /// panels are dropped.
    pub fn restore_layouts(&mut self, json: &serde_json::Value) {
        if json.get("version").and_then(serde_json::Value::as_u64) != Some(LAYOUT_VERSION) {
            return;
        }
        let Some(map) = json
            .get("workspaces")
            .and_then(serde_json::Value::as_object)
        else {
            return;
        };
        for (key, value) in map {
            let Some(ws) = self
                .registry
                .workspaces()
                .iter()
                .map(|w| w.id)
                .find(|w| w.0 == key)
            else {
                continue;
            };
            match DockHost::from_json(value, &self.registry) {
                Ok(dock) => {
                    self.layouts.insert(ws, dock);
                }
                Err(e) => tracing::warn!("ignoring the saved layout of `{key}`: {e}"),
            }
        }
        let current = self.workspace;
        let keys: Vec<WorkspaceId> = self.layouts.keys().copied().collect();
        for ws in keys {
            self.ensure_dock(ws);
        }
        self.workspace = current;
    }

    fn load_layouts(&mut self) {
        let Some(path) = &self.layout_path else {
            return;
        };
        let Ok(text) = std::fs::read_to_string(path) else {
            return;
        };
        match serde_json::from_str::<serde_json::Value>(&text) {
            Ok(json) => self.restore_layouts(&json),
            Err(e) => tracing::warn!("ignoring {}: {e}", path.display()),
        }
    }

    /// Writes the layouts to the configured `layout.json` (no-op without a path).
    ///
    /// # Errors
    /// The I/O error if the file cannot be written.
    pub fn save_layout(&self) -> std::io::Result<()> {
        let Some(path) = &self.layout_path else {
            return Ok(());
        };
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir)?;
        }
        let text =
            serde_json::to_string_pretty(&self.layouts_json()).map_err(std::io::Error::other)?;
        std::fs::write(path, text)
    }
}

/// Version of the `layout.json` format.
const LAYOUT_VERSION: u64 = 1;

/// The default `layout.json` path: `%APPDATA%\Amalgam\layout.json` on Windows (spec §3).
pub fn default_layout_path() -> Option<PathBuf> {
    directories::BaseDirs::new().map(|d| d.config_dir().join("Amalgam").join("layout.json"))
}

/// A batch that failed and was rolled back.
struct BatchFailure {
    label: String,
    error: amg_doc::CommandError,
}

/// One transaction per batch; a failing command rolls the whole batch back.
fn apply_batch(doc: &mut Document, batch: CommitBatch) -> Result<(), BatchFailure> {
    let mut tx = doc.transaction(&batch.label);
    for cmd in batch.commands {
        if let Err(error) = tx.apply(cmd) {
            tx.rollback();
            return Err(BatchFailure {
                label: batch.label,
                error,
            });
        }
    }
    tx.commit();
    Ok(())
}

/// Opens a folder in the platform's file manager without blocking.
fn open_in_file_manager(dir: &std::path::Path) {
    let _ = std::fs::create_dir_all(dir);
    let program = if cfg!(windows) {
        "explorer"
    } else if cfg!(target_os = "macos") {
        "open"
    } else {
        "xdg-open"
    };
    match std::process::Command::new(program).arg(dir).spawn() {
        Ok(mut child) => {
            // Reap the child so it does not linger as a zombie.
            std::thread::spawn(move || {
                let _ = child.wait();
            });
        }
        Err(e) => tracing::warn!("cannot open {}: {e}", dir.display()),
    }
}
