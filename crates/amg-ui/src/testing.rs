//! Test support (feature `testing`): [`AppHarness`] runs the real [`AmalgamApp`] in
//! `egui_kittest` with a headless [`JobSystem`], a [`NullViewport`], the default
//! [`CpuSceneBackend`](crate::scene::CpuSceneBackend) and a [`CpuPickService`].
//!
//! Widgets are found through AccessKit labels (`kittest().get_by_label(..)`). Snapshot tests
//! pass a wgpu setup ([`AppHarnessBuilder::wgpu`]); logic tests need no GPU.

use std::collections::BTreeMap;
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

/// CPU picking for tests: ray casts over the scene adapter's display meshes
/// ([`SceneAdapter::raycast`]), mapped to document targets through its [`PickMap`].
///
/// Requests are asynchronous like S10's: [`request_ray`](Self::request_ray) returns a ticket
/// and [`poll`](Self::poll) answers it once. UI-03 puts the `PickService` trait (screen
/// positions, radius, filter) over this.
///
/// [`PickMap`]: crate::scene::PickMap
#[derive(Debug, Default)]
pub struct CpuPickService {
    next: u64,
    requests: BTreeMap<PickTicket, (Ray3, PickMode)>,
}

impl CpuPickService {
    /// A service with no pending requests.
    pub fn new() -> CpuPickService {
        CpuPickService::default()
    }

    /// Number of requests not yet polled.
    pub fn pending(&self) -> usize {
        self.requests.len()
    }

    /// Queues a pick along a world ray.
    pub fn request_ray(&mut self, ray: Ray3, mode: PickMode) -> PickTicket {
        self.next += 1;
        let t = PickTicket(self.next);
        self.requests.insert(t, (ray, mode));
        t
    }

    /// The result of a request, exactly once (`None` for an unknown or already polled
    /// ticket).
    pub fn poll(&mut self, scene: &SceneAdapter, t: PickTicket) -> Option<Vec<PickHitUi>> {
        let (ray, mode) = self.requests.remove(&t)?;
        Some(Self::pick(scene, &ray, mode))
    }

    /// Picks now: the hits along `ray` in document terms, front to back.
    pub fn pick(scene: &SceneAdapter, ray: &Ray3, mode: PickMode) -> Vec<PickHitUi> {
        scene
            .raycast(ray, mode)
            .into_iter()
            .filter_map(|h| {
                Some(PickHitUi {
                    target: scene.pick_map().target(h.key, Some(h.sub))?,
                    world: h.world,
                    #[allow(clippy::cast_possible_truncation, reason = "S10 depth is f32")]
                    depth: h.t as f32,
                })
            })
            .collect()
    }
}

/// Options for [`AppHarness`].
pub struct AppHarnessBuilder {
    size: egui::Vec2,
    theme: Theme,
    registry: Option<UiRegistry>,
    wgpu: Option<egui_wgpu::WgpuSetup>,
    viewport: Option<Box<dyn ViewportBackend>>,
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
        #[allow(clippy::expect_used, reason = "test support; no threads means no test")]
        let jobs = JobSystem::new(JobSystemConfig {
            interactive_threads: std::num::NonZeroUsize::new(2),
            background_threads: std::num::NonZeroUsize::new(1),
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

    /// Steps frames until nothing is pending: no repaint requested, no unfinished job, every
    /// document's recompute idle, no tessellation in flight and no pick queued.
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
            let busy = busy_jobs || busy_docs || busy_scene || self.pick.pending() > 0;
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
