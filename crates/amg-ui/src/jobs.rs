//! Tracked jobs and the job panel (spec §6 "Tracked jobs", §14, UI-10).
//!
//! [`JobTracker`] owns the handles of jobs started with `spawn_tracked`. Once per frame
//! (`AmalgamApp::logic`) it polls them without blocking; a finished job first becomes a
//! notification if it failed (never if it was cancelled), then its callback runs on the UI
//! thread with `&mut AppCx`.
//!
//! The panel lists `JobSystem::jobs()` grouped by parent, with a progress bar, the latest
//! message, the elapsed time and a Cancel button.

use std::time::Duration;

use amg_jobs::{Job, JobError, JobId, JobInfo, JobState, JobSystem, SpawnOptions};

use crate::app::AppCx;

/// Repaint period while the panel shows running jobs (progress has no notifier).
const PANEL_REFRESH: Duration = Duration::from_millis(100);

/// Polls one tracked job; true once it finished and its callback ran.
type Poller = Box<dyn FnMut(&mut AppCx<'_>) -> bool>;

/// The tracked jobs of the app.
#[derive(Default)]
pub(crate) struct JobTracker {
    pub(crate) entries: Vec<Poller>,
}

impl std::fmt::Debug for JobTracker {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("JobTracker")
            .field("pending", &self.entries.len())
            .finish()
    }
}

impl JobTracker {
    pub(crate) fn spawn<J: Job>(
        &mut self,
        jobs: &JobSystem,
        job: J,
        opts: SpawnOptions,
        on_done: impl FnOnce(Result<J::Output, JobError<J::Error>>, &mut AppCx<'_>) + 'static,
    ) -> JobId
    where
        J::Error: amg_core::Diagnostic,
    {
        let mut handle = jobs.spawn_with(job, opts);
        let id = handle.id();
        let mut on_done = Some(on_done);
        self.entries.push(Box::new(move |cx| {
            let Some(result) = handle.try_take() else {
                return false;
            };
            if let Err(e) = &result {
                cx.notify_error(e);
            }
            if let Some(f) = on_done.take() {
                f(result, cx);
            }
            true
        }));
        id
    }
}

/// Parent → children grouping of the unfinished jobs; roots are jobs without a listed parent.
fn roots(infos: &[JobInfo]) -> Vec<&JobInfo> {
    infos
        .iter()
        .filter(|i| i.parent.is_none_or(|p| !infos.iter().any(|o| o.id == p)))
        .collect()
}

fn job_row(ui: &mut egui::Ui, jobs: &JobSystem, infos: &[JobInfo], info: &JobInfo) {
    ui.push_id(info.id.0, |ui| {
        ui.horizontal(|ui| {
            ui.strong(info.label.as_ref());
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let r = ui.button("Cancel");
                if r.clicked() {
                    jobs.cancel(info.id);
                }
            });
        });
        let (fraction, message) = match &info.state {
            JobState::Running(p) => (p.fraction, p.message.as_deref().unwrap_or("")),
            _ => (0.0, ""),
        };
        let text = match &info.state {
            JobState::Queued => "queued".to_owned(),
            _ => format!("{:.0} %", fraction * 100.0),
        };
        ui.add(
            egui::ProgressBar::new(fraction.clamp(0.0, 1.0))
                .desired_width(240.0)
                .text(text),
        );
        let elapsed = info.started.map(|s| s.elapsed().as_secs_f32());
        let line = match (message.is_empty(), elapsed) {
            (false, Some(e)) => format!("{message} · {e:.1} s"),
            (true, Some(e)) => format!("{e:.1} s"),
            (false, None) => message.to_owned(),
            (true, None) => String::new(),
        };
        if !line.is_empty() {
            ui.weak(line);
        }
    });
    let children: Vec<&JobInfo> = infos.iter().filter(|c| c.parent == Some(info.id)).collect();
    if !children.is_empty() {
        ui.indent(("amg_job_children", info.id.0), |ui| {
            for c in children {
                job_row(ui, jobs, infos, c);
            }
        });
    }
}

/// The expandable job panel at the bottom right, above the status bar.
pub(crate) fn panel(ctx: &egui::Context, jobs: &JobSystem, bottom_offset: f32) {
    let infos = jobs.jobs();
    egui::Area::new(egui::Id::new("amg_job_panel"))
        .anchor(
            egui::Align2::RIGHT_BOTTOM,
            egui::vec2(-12.0, -bottom_offset),
        )
        .order(egui::Order::Foreground)
        .show(ctx, |ui| {
            egui::Frame::popup(ui.style()).show(ui, |ui| {
                ui.set_min_width(280.0);
                ui.strong("Jobs");
                ui.separator();
                if infos.is_empty() {
                    ui.weak("No running jobs");
                }
                for root in roots(&infos) {
                    job_row(ui, jobs, &infos, root);
                }
            });
        });
    if !infos.is_empty() {
        ctx.request_repaint_after(PANEL_REFRESH);
    }
}
