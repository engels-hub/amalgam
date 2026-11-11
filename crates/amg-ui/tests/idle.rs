//! UI-01 Done-when 4: with no input and no jobs the app does not repaint.
//!
//! eframe runs a frame only when egui asks for one (immediately or after a delay). This test
//! drives the real `AmalgamApp` on a raw `egui::Context` the same way: a frame runs when a
//! requested repaint is due, and the frame counter must stay constant over 1 s of idling.

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

use amg_jobs::{JobSystem, JobSystemConfig};
use amg_ui::app::repaint_notifier;
use amg_ui::theme::Theme;
use amg_ui::viewport::NullViewport;
use amg_ui::{AmalgamApp, AppConfig, UiRegistry};

struct Loop {
    ctx: egui::Context,
    app: AmalgamApp,
    start: Instant,
    due: Arc<Mutex<Option<Instant>>>,
}

impl Loop {
    fn new() -> Loop {
        let ctx = egui::Context::default();
        let due: Arc<Mutex<Option<Instant>>> = Arc::new(Mutex::new(None));
        let d = Arc::clone(&due);
        ctx.set_request_repaint_callback(move |info| {
            let at = Instant::now() + info.delay;
            let mut slot = d.lock().unwrap();
            *slot = Some(slot.map_or(at, |s| s.min(at)));
        });
        let slot = Arc::new(OnceLock::new());
        let _ = slot.set(ctx.clone());
        let jobs = JobSystem::new(JobSystemConfig {
            notifier: Some(repaint_notifier(&slot)),
            ..JobSystemConfig::default()
        })
        .unwrap();
        let app = AmalgamApp::new(AppConfig {
            jobs: Arc::new(jobs),
            repaint: slot,
            viewport: Box::new(NullViewport),
            registry: UiRegistry::build(),
            theme: Theme::Dark,
            layout_path: None,
        });
        Loop {
            ctx,
            app,
            start: Instant::now(),
            due,
        }
    }

    fn frame(&mut self) {
        let input = egui::RawInput {
            time: Some(self.start.elapsed().as_secs_f64()),
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(1280.0, 800.0),
            )),
            ..Default::default()
        };
        let app = &mut self.app;
        let mut out = self.ctx.run_ui(input, |ui| app.frame(ui));
        // No renderer: drop the texture uploads the way a backend would after applying them.
        out.textures_delta.clear();
    }

    /// Runs a frame if a repaint is due. Returns true if it ran one.
    fn pump(&mut self) -> bool {
        let now = Instant::now();
        let ready = {
            let mut slot = self.due.lock().unwrap();
            match *slot {
                Some(at) if at <= now => {
                    *slot = None;
                    true
                }
                _ => false,
            }
        };
        if ready {
            self.frame();
        }
        ready
    }
}

#[test]
fn idle_app_runs_no_frames() {
    let mut l = Loop::new();
    l.frame();
    // Settle: egui's own follow-up passes (layout, fonts) within 2 s.
    let settle = Instant::now();
    let mut quiet_since = Instant::now();
    while quiet_since.elapsed() < Duration::from_millis(300) {
        assert!(
            settle.elapsed() < Duration::from_secs(5),
            "never settled: {} frames",
            l.app.frame_count()
        );
        if l.pump() {
            quiet_since = Instant::now();
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    let settled = l.app.frame_count();
    let t = Instant::now();
    while t.elapsed() < Duration::from_secs(1) {
        l.pump();
        std::thread::sleep(Duration::from_millis(5));
    }
    assert_eq!(
        l.app.frame_count(),
        settled,
        "frames ran while idle: causes {:?}",
        l.ctx.repaint_causes()
    );
    assert!(
        l.due.lock().unwrap().is_none(),
        "a repaint is still scheduled"
    );
}

#[test]
fn input_runs_a_frame_and_then_the_app_settles_again() {
    let mut l = Loop::new();
    l.frame();
    let t = Instant::now();
    while t.elapsed() < Duration::from_millis(500) {
        l.pump();
        std::thread::sleep(Duration::from_millis(2));
    }
    let before = l.app.frame_count();
    // A job (spawn and finish) wakes the UI through the notifier.
    let h = l
        .app
        .jobs()
        .spawn_fn("noop", amg_jobs::SpawnOptions::default(), |_| {
            Ok::<_, amg_jobs::Cancelled>(())
        });
    while !h.is_finished() {
        std::thread::yield_now();
    }
    let t = Instant::now();
    while t.elapsed() < Duration::from_millis(500) {
        l.pump();
        std::thread::sleep(Duration::from_millis(2));
    }
    let after_job = l.app.frame_count();
    assert!(after_job > before, "the notifier did not wake the UI");
    let t = Instant::now();
    while t.elapsed() < Duration::from_secs(1) {
        l.pump();
        std::thread::sleep(Duration::from_millis(5));
    }
    assert_eq!(
        l.app.frame_count(),
        after_job,
        "kept repainting after the job"
    );
}
