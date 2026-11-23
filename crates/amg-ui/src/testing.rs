//! Test support (feature `testing`): [`AppHarness`] runs the real [`AmalgamApp`] in
//! `egui_kittest` with a headless [`JobSystem`], a [`NullViewport`], the default
//! [`CpuSceneBackend`](crate::scene::CpuSceneBackend) and a [`CpuPickService`].
//!
//! Widgets are found through AccessKit labels (`kittest().get_by_label(..)`). Snapshot tests
//! pass a wgpu setup ([`AppHarnessBuilder::wgpu`]); logic tests need no GPU.

use std::sync::{Arc, OnceLock};
use std::time::{Duration, Instant};

pub use egui_kittest;
use egui_kittest::Harness;

use amg_core::Ray3;
use amg_doc::Document;
use amg_jobs::{JobSystem, JobSystemConfig};

use crate::app::{AmalgamApp, AppConfig, repaint_notifier};
use crate::pick::{PickHitUi, PickMode, PickTicket};
use crate::registry::UiRegistry;
use crate::scene::SceneAdapter;
use crate::theme::Theme;
use crate::viewport::{NullViewport, ViewportBackend};

/// How long [`AppHarness::drive_until_idle`] waits before it panics.
pub const IDLE_TIMEOUT: Duration = Duration::from_secs(10);

pub use crate::pick::CpuPickService;

/// A [`FileDialogBackend`](crate::file::FileDialogBackend) for tests: answers dialogs from a
/// queue (an empty queue cancels) and records every request. Install it with
/// `app.set_file_dialog_backend(fake.clone())`.
#[derive(Debug, Default)]
pub struct FakeFileDialog {
    answers: std::sync::Mutex<std::collections::VecDeque<Option<std::path::PathBuf>>>,
    requests: std::sync::Mutex<Vec<crate::file::FileDialogRequest>>,
}

impl FakeFileDialog {
    /// A fake with no queued answers.
    pub fn new() -> Arc<FakeFileDialog> {
        Arc::new(FakeFileDialog::default())
    }

    /// Queues the answer to the next dialog (`None`: the user cancels).
    pub fn answer(&self, path: Option<std::path::PathBuf>) {
        lock(&self.answers).push_back(path);
    }

    /// The dialogs shown so far.
    pub fn requests(&self) -> Vec<crate::file::FileDialogRequest> {
        lock(&self.requests).clone()
    }
}

fn lock<T>(m: &std::sync::Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    m.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
}

impl crate::file::FileDialogBackend for FakeFileDialog {
    fn show(&self, req: &crate::file::FileDialogRequest) -> Option<std::path::PathBuf> {
        lock(&self.requests).push(req.clone());
        lock(&self.answers).pop_front().flatten()
    }
}

/// Options for [`AppHarness`].
pub struct AppHarnessBuilder {
    size: egui::Vec2,
    theme: Theme,
    registry: Option<UiRegistry>,
    wgpu: Option<egui_wgpu::WgpuSetup>,
    viewport: Option<Box<dyn ViewportBackend>>,
    job_threads: Option<(usize, usize)>,
}

impl std::fmt::Debug for AppHarnessBuilder {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AppHarnessBuilder")
            .field("size", &self.size)
            .field("theme", &self.theme)
            .finish_non_exhaustive()
    }
}

impl Default for AppHarnessBuilder {
    fn default() -> Self {
        AppHarnessBuilder {
            size: egui::vec2(1280.0, 800.0),
            theme: Theme::Dark,
            registry: None,
            wgpu: None,
            viewport: None,
            job_threads: None,
        }
    }
}

impl AppHarnessBuilder {
    /// Window size in points (default 1280×800).
    pub fn size(mut self, size: egui::Vec2) -> Self {
        self.size = size;
        self
    }

    /// Theme (default dark; `System` would depend on the machine).
    pub fn theme(mut self, theme: Theme) -> Self {
        self.theme = theme;
        self
    }

    /// A registry instead of [`UiRegistry::build`].
    pub fn registry(mut self, reg: UiRegistry) -> Self {
        self.registry = Some(reg);
        self
    }

    /// Job pool sizes (interactive, background) instead of the default 2 / 1, e.g. the app's
    /// own sizing for timing tests (additive, AM-12). Zero means the machine's parallelism.
    pub fn job_threads(mut self, interactive: usize, background: usize) -> Self {
        self.job_threads = Some((interactive, background));
        self
    }

    /// Render with wgpu (snapshot tests), e.g. a WARP device.
    pub fn wgpu(mut self, setup: egui_wgpu::WgpuSetup) -> Self {
        self.wgpu = Some(setup);
        self
    }

    /// A viewport backend instead of [`NullViewport`] (e.g. one that reports a lost device).
    pub fn viewport(mut self, viewport: Box<dyn ViewportBackend>) -> Self {
        self.viewport = Some(viewport);
        self
    }

    /// Builds the harness with one empty document.
    ///
    /// # Panics
    /// If the job system's threads cannot be created.
    pub fn build(self) -> AppHarness {
        let slot = Arc::new(OnceLock::new());
        let (interactive, background) = self.job_threads.unwrap_or((2, 1));
        #[allow(clippy::expect_used, reason = "test support; no threads means no test")]
        let jobs = JobSystem::new(JobSystemConfig {
            interactive_threads: std::num::NonZeroUsize::new(interactive),
            background_threads: std::num::NonZeroUsize::new(background),
            notifier: Some(repaint_notifier(&slot)),
        })
        .expect("job system");
        let app = AmalgamApp::new(AppConfig {
            jobs: Arc::new(jobs),
            repaint: Arc::clone(&slot),
            viewport: self.viewport.unwrap_or_else(|| Box::new(NullViewport)),
            registry: self.registry.unwrap_or_else(UiRegistry::build),
            theme: self.theme,
            layout_path: None,
        });
        let mut builder = Harness::builder().with_size(self.size);
        if let Some(setup) = self.wgpu {
            builder = builder.wgpu_setup(setup);
        }
        let harness = builder.build_ui_state(|ui, app: &mut AmalgamApp| app.frame(ui), app);
        let _ = slot.set(harness.ctx.clone());
        AppHarness {
            harness,
            pick: CpuPickService::new(),
        }
    }
}

/// The real app in a kittest harness.
pub struct AppHarness {
    harness: Harness<'static, AmalgamApp>,
    pick: CpuPickService,
}

impl std::fmt::Debug for AppHarness {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AppHarness")
            .field("app", self.harness.state())
            .finish_non_exhaustive()
    }
}

impl Default for AppHarness {
    fn default() -> Self {
        AppHarness::new()
    }
}

impl AppHarness {
    /// Empty document, dark theme, [`NullViewport`], every inventory module.
    pub fn new() -> AppHarness {
        AppHarnessBuilder::default().build()
    }

    /// Options.
    pub fn builder() -> AppHarnessBuilder {
        AppHarnessBuilder::default()
    }

    /// Like [`AppHarness::new`], with `doc` as the open document.
    pub fn with_document(doc: Document) -> AppHarness {
        let mut h = AppHarness::new();
        h.app_mut().active_document_mut().document = doc;
        h.harness.run();
        h
    }

    /// The kittest harness (queries, clicks, keys, snapshots).
    pub fn kittest(&mut self) -> &mut Harness<'static, AmalgamApp> {
        &mut self.harness
    }

    /// The app.
    pub fn app(&self) -> &AmalgamApp {
        self.harness.state()
    }

    /// The app, mutable.
    pub fn app_mut(&mut self) -> &mut AmalgamApp {
        self.harness.state_mut()
    }

    /// The pick service.
    pub fn picks(&self) -> &CpuPickService {
        &self.pick
    }

    /// The pick service, mutable (queue requests, then [`Self::poll_pick`]).
    pub fn picks_mut(&mut self) -> &mut CpuPickService {
        &mut self.pick
    }

    /// Answers a queued pick against the current scene.
    pub fn poll_pick(&mut self, t: PickTicket) -> Option<Vec<PickHitUi>> {
        let scene = self.harness.state().scene();
        self.pick.poll(scene, t)
    }

    /// Picks along a world ray now.
    pub fn pick_ray(&self, ray: Ray3, mode: PickMode) -> Vec<PickHitUi> {
        CpuPickService::pick(self.app().scene(), &ray, mode)
    }

    /// The scene adapter.
    pub fn scene(&self) -> &SceneAdapter {
        self.app().scene()
    }

    /// Runs a registered command as a toolbar click would (queued, applied at the end of the
    /// next frame), then steps until idle. A disabled command does nothing.
    pub fn run_command(&mut self, id: crate::command::CmdId) {
        self.app_mut().request_command(id);
        self.drive_until_idle();
    }

    /// The active document.
    pub fn document(&self) -> &Document {
        &self.app().active_document().document
    }

    // ---- viewport input (UI-03) ------------------------------------------------------------

    /// The camera snapshot of the last frame's viewport.
    ///
    /// # Panics
    /// Before the first frame drew the viewport.
    pub fn camera_view(&self) -> crate::input::CameraView {
        #[allow(clippy::expect_used, reason = "test support")]
        self.app()
            .camera_view()
            .cloned()
            .expect("the viewport was drawn")
    }

    /// Where a world point is on screen (last frame's camera).
    ///
    /// # Panics
    /// If the point is behind the camera.
    pub fn world_to_screen(&self, p: amg_core::Point3) -> egui::Pos2 {
        #[allow(clippy::expect_used, reason = "test support")]
        self.camera_view()
            .project(p)
            .expect("in front of the camera")
    }

    /// Moves the pointer to `pos` and runs one frame.
    pub fn hover_at(&mut self, pos: egui::Pos2) {
        self.harness.event(egui::Event::PointerMoved(pos));
        self.harness.step();
    }

    /// Presses or releases `button` at `pos` (one frame per event).
    pub fn button(
        &mut self,
        pos: egui::Pos2,
        button: egui::PointerButton,
        pressed: bool,
        mods: egui::Modifiers,
    ) {
        self.harness.event(egui::Event::ModifiersChanged(mods));
        self.harness.event(egui::Event::PointerButton {
            pos,
            button,
            pressed,
            modifiers: mods,
        });
        self.harness.step();
        if !pressed {
            self.harness
                .event(egui::Event::ModifiersChanged(egui::Modifiers::NONE));
            self.harness.step();
        }
    }

    /// Clicks `button` at `pos` (move, press, release).
    pub fn click_at(
        &mut self,
        pos: egui::Pos2,
        button: egui::PointerButton,
        mods: egui::Modifiers,
    ) {
        self.hover_at(pos);
        self.button(pos, button, true, mods);
        self.button(pos, button, false, mods);
    }

    /// Clicks the primary button on a world point.
    pub fn click_world(&mut self, p: amg_core::Point3) {
        let pos = self.world_to_screen(p);
        self.click_at(pos, egui::PointerButton::Primary, egui::Modifiers::NONE);
    }

    /// Drags `button` with `mods` from `from` to `to` in `steps` pointer moves (one frame each).
    pub fn drag(
        &mut self,
        button: egui::PointerButton,
        mods: egui::Modifiers,
        from: egui::Pos2,
        to: egui::Pos2,
        steps: u32,
    ) {
        self.hover_at(from);
        self.button(from, button, true, mods);
        let steps = steps.max(1);
        for i in 1..=steps {
            #[allow(clippy::cast_precision_loss, reason = "small step counts")]
            let t = i as f32 / steps as f32;
            self.harness
                .event(egui::Event::PointerMoved(from + (to - from) * t));
            self.harness.step();
        }
        self.button(to, button, false, mods);
    }

    /// One frame with a key press and release.
    pub fn key(&mut self, key: egui::Key) {
        self.harness.key_press(key);
        self.harness.step();
    }

    /// Steps frames until nothing is pending: no repaint requested, no unfinished job, every
    /// document's recompute idle, no tessellation in flight, no pick queued and no file
    /// dialog, save, open or recovery load pending.
    ///
    /// # Panics
    /// After [`IDLE_TIMEOUT`].
    pub fn drive_until_idle(&mut self) {
        let start = Instant::now();
        loop {
            self.harness.step();
            let app = self.harness.state();
            let busy_jobs = !app.jobs().jobs().is_empty();
            let busy_docs = app.documents().iter().any(|d| {
                !matches!(
                    d.document.recompute_status(),
                    amg_doc::RecomputeStatus::Idle | amg_doc::RecomputeStatus::Failed(_)
                )
            });
            let busy_scene = app.scene().pending() > 0;
            let busy = busy_jobs
                || busy_docs
                || busy_scene
                || app.file_busy()
                || app.input_busy()
                || self.pick.pending() > 0;
            if !busy && !self.harness.ctx.has_requested_repaint() {
                return;
            }
            assert!(
                start.elapsed() < IDLE_TIMEOUT,
                "drive_until_idle: still busy after {IDLE_TIMEOUT:?} (jobs {busy_jobs}, recompute {busy_docs})"
            );
            if busy {
                std::thread::sleep(Duration::from_millis(1));
            }
        }
    }
}
