//! The Amalgam desktop application.
//!
//! Spec: S11 §2 (start-up and frame loop). Lane: D.
//!
//! Start-up: diagnostics and the panic hook (FND-07), the UI-thread mark, the shared `Gpu`
//! (FND-09) handed to eframe as `WgpuSetup::Existing`, the `JobSystem` whose notifier repaints
//! egui, the feature and UI registries, then the first frame. The whole start-up is the
//! tracing span `startup`; its event `first interactive frame` carries `elapsed_ms`.
//!
//! ```text
//! amg-app [--tris N] [--no-demo] [--theme light|dark|system] [--exit-after-first-frame]
//! ```

mod demo;

use std::sync::{Arc, OnceLock};
use std::time::{Duration, Instant};

use amg_core::{Aabb3, Dir3, Transform, Vec3};
use amg_jobs::diag::{self, DiagConfig};
use amg_jobs::gpu::{Gpu, GpuConfig};
use amg_jobs::{JobHandle, JobSystem, JobSystemConfig, SpawnOptions};
use amg_render::egui_glue;
use amg_render::{Camera, GpuMesh, Instance, Projection, RenderError, RendererConfig, ViewId};
use amg_ui::app::{default_layout_path, repaint_notifier};
use amg_ui::theme::Theme;
use amg_ui::viewport::RenderViewport;
use amg_ui::{AmalgamApp, AppConfig, UiRegistry};
use anyhow::{Context as _, anyhow, bail};
use egui_wgpu::RenderState;

/// Triangles of the demo mesh (the RND-01 spike size).
const DEFAULT_DEMO_TRIS: usize = 1_000_000;

#[derive(Clone, Debug)]
struct Args {
    tris: usize,
    demo: bool,
    theme: Theme,
    exit_after_first_frame: bool,
}

fn parse_args() -> anyhow::Result<Args> {
    let mut out = Args {
        tris: DEFAULT_DEMO_TRIS,
        demo: true,
        theme: Theme::System,
        exit_after_first_frame: false,
    };
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "--tris" => {
                out.tris = it
                    .next()
                    .context("--tris needs a number")?
                    .parse()
                    .context("--tris needs a number")?;
            }
            "--no-demo" => out.demo = false,
            "--theme" => {
                out.theme = match it.next().as_deref() {
                    Some("light") => Theme::Light,
                    Some("dark") => Theme::Dark,
                    Some("system") => Theme::System,
                    other => bail!("--theme light|dark|system, got {other:?}"),
                };
            }
            "--exit-after-first-frame" => out.exit_after_first_frame = true,
            other => bail!(
                "unknown argument `{other}`; usage: amg-app [--tris N] [--no-demo] \
                 [--theme light|dark|system] [--exit-after-first-frame]"
            ),
        }
    }
    Ok(out)
}

fn main() -> anyhow::Result<()> {
    let start = Instant::now();
    let args = parse_args()?;
    let diag_guard = diag::init(&DiagConfig {
        app_name: "amalgam",
        version: env!("CARGO_PKG_VERSION"),
        git_sha: env!("AMG_GIT_SHA"),
        ..DiagConfig::default()
    })?;
    diag::install_panic_hook();
    amg_jobs::mark_ui_thread();

    let startup = tracing::info_span!("startup");
    let (gpu, jobs, slot, registry) = startup.in_scope(|| -> anyhow::Result<_> {
        let mut cfg = GpuConfig::from_env();
        cfg.optional_features |= amg_render::OPTIONAL_FEATURES;
        // Allowed before the event loop (spec §2 step 2).
        let gpu = Gpu::new_blocking(&cfg).context("open the GPU device")?;
        let info = gpu.info();
        tracing::info!(adapter = %info.name, backend = ?info.backend, "GPU device open");

        let slot: Arc<OnceLock<egui::Context>> = Arc::new(OnceLock::new());
        let jobs = Arc::new(JobSystem::new(JobSystemConfig {
            notifier: Some(repaint_notifier(&slot)),
            ..JobSystemConfig::default()
        })?);
        amg_features::mesh::link();
        amg_features::sketch::link();
        let registry = UiRegistry::build();
        tracing::info!(modules = ?registry.modules(), "UI registry built");
        Ok((gpu, jobs, slot, registry))
    })?;
    // UI-19: the recovery check (`find_recoverable`) runs here, before the first frame.

    let native = eframe::NativeOptions {
        renderer: eframe::Renderer::Wgpu,
        wgpu_options: egui_wgpu::WgpuConfiguration {
            surface: egui_wgpu::SurfaceConfig::HIGH_THROUGHPUT,
            wgpu_setup: egui_wgpu::WgpuSetup::Existing(egui_wgpu::WgpuSetupExisting {
                instance: gpu.instance().clone(),
                adapter: gpu.adapter().clone(),
                device: gpu.device().clone(),
                queue: gpu.queue().clone(),
            }),
            ..Default::default()
        },
        multisampling: 0,
        depth_buffer: 0,
        stencil_buffer: 0,
        viewport: egui::ViewportBuilder::default()
            .with_title("Amalgam")
            .with_inner_size([1600.0, 960.0])
            .with_min_inner_size([800.0, 500.0]),
        ..Default::default()
    };

    let app_jobs = Arc::clone(&jobs);
    let run = eframe::run_native(
        "Amalgam",
        native,
        Box::new(move |cc| {
            let _ = slot.set(cc.egui_ctx.clone());
            let rs = cc
                .wgpu_render_state
                .clone()
                .context("eframe must run on the wgpu renderer")?;
            if !egui_glue::shares_device(&rs, &gpu) {
                return Err(anyhow!("egui does not draw with the shared Gpu device").into());
            }
            egui_glue::install(&rs, Arc::clone(&gpu), &RendererConfig::default(), 1)?;
            let view = egui_glue::with_renderer(&rs, |r| r.create_view([1, 1]))
                .context("renderer installed")?;
            let mut app = AmalgamApp::new(AppConfig {
                jobs: Arc::clone(&app_jobs),
                repaint: slot,
                viewport: Box::new(RenderViewport::new(rs.clone(), view)),
                registry,
                theme: args.theme,
                layout_path: default_layout_path(),
            });
            if let Some(path) = amg_ui::shortcut::default_shortcuts_path()
                && let Err(e) = app.load_shortcuts_file(&path)
            {
                tracing::warn!("ignoring {}: {e}", path.display());
            }
            let demo = args
                .demo
                .then(|| spawn_demo(&app_jobs, Arc::clone(&gpu), args.tris));
            Ok(Box::new(Shell {
                app,
                rs,
                view,
                demo,
                startup: Some((startup, start)),
                exit_after_first_frame: args.exit_after_first_frame,
            }))
        }),
    );
    let report = jobs.shutdown(Duration::from_secs(2));
    if !report.still_running.is_empty() {
        tracing::warn!(
            jobs = report.still_running.len(),
            "jobs still running at exit"
        );
    }
    drop(diag_guard);
    run.map_err(|e| anyhow!("the window failed: {e}"))
}

/// Generates and uploads the demo mesh on a job (never on the UI thread).
fn spawn_demo(
    jobs: &JobSystem,
    gpu: Arc<Gpu>,
    tris: usize,
) -> JobHandle<(GpuMesh, Aabb3), RenderError> {
    jobs.spawn_fn("demo mesh", SpawnOptions::default(), move |ctx| {
        let data = demo::displaced_icosphere(tris);
        ctx.checkpoint()?;
        let bounds =
            Aabb3::from_points(data.positions.iter().map(|p| {
                data.origin + Vec3::new(f64::from(p[0]), f64::from(p[1]), f64::from(p[2]))
            }));
        let mesh = amg_render::upload_mesh(&gpu, &data, ctx)?;
        Ok((mesh, bounds))
    })
}

/// eframe glue around [`AmalgamApp`]: start-up timing and the demo mesh.
struct Shell {
    app: AmalgamApp,
    rs: RenderState,
    view: ViewId,
    demo: Option<JobHandle<(GpuMesh, Aabb3), RenderError>>,
    startup: Option<(tracing::Span, Instant)>,
    exit_after_first_frame: bool,
}

impl Shell {
    fn take_demo(&mut self, ctx: &egui::Context) {
        let Some(handle) = &mut self.demo else {
            return;
        };
        let Some(result) = handle.try_take() else {
            return;
        };
        self.demo = None;
        match result {
            Ok((mesh, bounds)) => {
                let triangles = mesh.triangle_count();
                let view = self.view;
                egui_glue::with_renderer(&self.rs, |r| {
                    let key = r.scene_mut().insert_mesh(mesh);
                    r.scene_mut()
                        .add_instance(Instance::new(key, Transform::IDENTITY));
                    if let Some(v) = r.view_mut(view) {
                        v.camera = home_camera(&bounds);
                    }
                });
                tracing::info!(triangles, "demo mesh in the scene");
                ctx.request_repaint();
            }
            Err(e) => tracing::warn!("demo mesh failed: {e}"),
        }
    }
}

/// Fusion-like home view: from front-right-top, Z up.
fn home_camera(bounds: &Aabb3) -> Camera {
    let target = bounds.center();
    let distance = bounds.diagonal() * 1.3;
    let dir = Vec3::new(1.0, -1.0, 0.8).normalize();
    Camera {
        eye: target + dir * distance,
        target,
        up: Dir3::Z,
        projection: Projection::Perspective {
            fov_y: 30f64.to_radians(),
        },
    }
}

impl eframe::App for Shell {
    fn logic(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.app.logic(ctx);
        self.take_demo(ctx);
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.app.ui(ui);
        if let Some((span, t0)) = self.startup.take() {
            let elapsed_ms = t0.elapsed().as_secs_f64() * 1e3;
            span.in_scope(|| tracing::info!(elapsed_ms, "first interactive frame"));
            if self.exit_after_first_frame {
                ui.ctx().send_viewport_cmd(egui::ViewportCommand::Close);
            }
        }
    }

    fn on_exit(&mut self) {
        if let Err(e) = self.app.save_layout() {
            tracing::warn!("could not save the dock layout: {e}");
        }
    }
}
