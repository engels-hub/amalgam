//! UI-10 tests: job panel, tracked jobs, toasts from `Diagnostic`, the cancelled-is-not-error
//! rule and the GPU device-lost banner, through `AppHarness` (kittest, no GPU).
//!
//! Timing assertions run only under `AMG_STRICT_TIMING=1`.

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::borrow::Cow;
use std::cell::{Cell, RefCell};
use std::path::PathBuf;
use std::rc::Rc;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::time::{Duration, Instant};

use amg_core::{Cancelled, Diagnostic};
use amg_jobs::{Job, JobCtx, JobError, SpawnOptions};
use amg_ui::Notification;
use amg_ui::command::{CmdAction, CommandDef};
use amg_ui::module::{Registrar, UiModule};
use amg_ui::testing::AppHarness;
use amg_ui::testing::egui_kittest::kittest::{NodeT, Queryable};
use amg_ui::viewport::{NullViewport, ViewportBackend, ViewportFrame};

static SAVES: AtomicUsize = AtomicUsize::new(0);

inventory::submit! { UiModule { id: "zz_jobs_panel", register } }

fn register(r: &mut Registrar<'_>) {
    r.command(
        CommandDef::new(
            "test.save",
            "Save",
            CmdAction::Run(|_| {
                SAVES.fetch_add(1, Ordering::SeqCst);
            }),
        )
        .in_menu("File", 5),
    );
}

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").as_deref() == Ok("1")
}

/// Steps frames (sleeping a little between them) until `done` holds.
fn pump(h: &mut AppHarness, what: &str, mut done: impl FnMut(&mut AppHarness) -> bool) {
    let t0 = Instant::now();
    loop {
        h.kittest().step();
        if done(h) {
            return;
        }
        assert!(t0.elapsed() < Duration::from_secs(8), "timed out: {what}");
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// A few frames, for tests with auto-hide toasts: `drive_until_idle` would wait for the
/// delayed repaint of the toast timer.
fn settle(h: &mut AppHarness) {
    for _ in 0..3 {
        h.kittest().step();
    }
}

/// Runs for about 2 s, reporting progress with a message, and stops at a checkpoint.
struct Slow;

impl Job for Slow {
    type Output = ();
    type Error = amg_core::Error;
    fn label(&self) -> Cow<'static, str> {
        "Slow test job".into()
    }
    fn run(self, ctx: &JobCtx) -> Result<(), amg_core::Error> {
        let t0 = Instant::now();
        while t0.elapsed() < Duration::from_secs(2) {
            ctx.checkpoint()?;
            ctx.progress_msg(t0.elapsed().as_secs_f32() / 2.0, "halfway there");
            std::thread::sleep(Duration::from_millis(10));
        }
        Ok(())
    }
}

/// Fails at once with a given error.
struct Fail<E: std::error::Error + From<Cancelled> + Send + Sync + 'static>(fn() -> E);

impl<E: std::error::Error + From<Cancelled> + Send + Sync + 'static> Job for Fail<E> {
    type Output = ();
    type Error = E;
    fn label(&self) -> Cow<'static, str> {
        "Failing job".into()
    }
    fn run(self, _ctx: &JobCtx) -> Result<(), E> {
        Err((self.0)())
    }
}

/// An error with a hint and a `source()` chain.
#[derive(Debug, thiserror::Error)]
#[error("build failed")]
struct Chained {
    #[source]
    inner: amg_core::Error,
}

impl From<Cancelled> for Chained {
    fn from(c: Cancelled) -> Chained {
        Chained { inner: c.into() }
    }
}

impl Diagnostic for Chained {
    fn hint(&self) -> Option<String> {
        Some("Check that the profile is closed".to_owned())
    }
}

/// A parent that waits for one child.
struct Parent;
struct Child;

impl Job for Child {
    type Output = ();
    type Error = amg_core::Error;
    fn label(&self) -> Cow<'static, str> {
        "Child job".into()
    }
    fn run(self, ctx: &JobCtx) -> Result<(), amg_core::Error> {
        let t0 = Instant::now();
        while t0.elapsed() < Duration::from_secs(2) {
            ctx.checkpoint()?;
            std::thread::sleep(Duration::from_millis(10));
        }
        Ok(())
    }
}

impl Job for Parent {
    type Output = ();
    type Error = amg_core::Error;
    fn label(&self) -> Cow<'static, str> {
        "Parent job".into()
    }
    fn run(self, ctx: &JobCtx) -> Result<(), amg_core::Error> {
        let child = ctx.spawn_child(Child);
        match ctx.wait(child) {
            Ok(()) => Ok(()),
            Err(JobError::Cancelled) => Err(Cancelled.into()),
            Err(JobError::Failed(e)) => Err(e),
            Err(JobError::Panicked(m)) => Err(amg_core::Error::InvalidArgument {
                name: "child",
                reason: m,
            }),
        }
    }
}

#[test]
fn job_panel_shows_progress_and_cancel_resolves_without_an_error_toast() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    let outcome: Rc<Cell<Option<bool>>> = Rc::new(Cell::new(None));
    let o = Rc::clone(&outcome);
    h.app_mut()
        .spawn_tracked(Slow, SpawnOptions::default(), move |res, _cx| {
            o.set(Some(matches!(res, Err(JobError::Cancelled))));
        });
    h.kittest().step();
    assert!(
        h.kittest().query_by_label("Cancel").is_none(),
        "the panel is collapsed by default"
    );
    h.kittest().get_by_label("Jobs: 1").click();
    pump(&mut h, "panel shows the progress message", |h| {
        h.kittest()
            .query_by_label_contains("halfway there")
            .is_some()
    });
    assert!(h.kittest().query_by_label("Slow test job").is_some());
    let jobs = h.app().jobs().jobs();
    assert_eq!(jobs.len(), 1);
    assert!(
        matches!(&jobs[0].state, amg_jobs::JobState::Running(p) if p.fraction > 0.0),
        "{:?}",
        jobs[0].state
    );

    // The area is laid out invisibly in its first frame; let it settle before clicking.
    settle(&mut h);
    h.kittest().get_by_label("Cancel").click();
    let t0 = Instant::now();
    pump(&mut h, "tracked job resolved after Cancel", |_| {
        outcome.get().is_some()
    });
    let took = t0.elapsed();
    eprintln!("cancel -> resolved: {took:?}");
    if strict_timing() {
        assert!(took < Duration::from_millis(200), "{took:?}");
    }
    assert_eq!(outcome.get(), Some(true), "resolved as Cancelled");
    h.drive_until_idle();
    assert!(
        h.app().notifications().is_empty(),
        "no toast for a cancel: {:?}",
        h.app().notifications().items()
    );
    assert!(h.kittest().query_by_label("Jobs: 0").is_some());
}

#[test]
fn failing_job_shows_message_hint_and_report_chain() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    h.app_mut().spawn_tracked(
        Fail(|| amg_core::Error::InvalidArgument {
            name: "width",
            reason: "must be positive".to_owned(),
        }),
        SpawnOptions::default(),
        |res, _| assert!(matches!(res, Err(JobError::Failed(_)))),
    );
    h.drive_until_idle();
    let toasts = h.app().notifications().items();
    assert_eq!(toasts.len(), 1);
    let n = &toasts[0].notification;
    let err = amg_core::Error::InvalidArgument {
        name: "width",
        reason: "must be positive".to_owned(),
    };
    assert_eq!(n.severity, amg_core::Severity::Error);
    assert_eq!(n.body, err.user_message());
    assert_eq!(n.details.as_deref(), Some(amg_core::report(&err).as_str()));
    assert!(n.sticky);
    assert!(
        h.kittest()
            .query_by_label("invalid argument `width`: must be positive")
            .is_some(),
        "the message is on screen"
    );
}

#[test]
fn toast_shows_hint_and_details_with_the_source_chain() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    h.app_mut().spawn_tracked(
        Fail(|| Chained {
            inner: amg_core::Error::Degenerate { what: "profile" },
        }),
        SpawnOptions::default(),
        |_, _| {},
    );
    h.drive_until_idle();
    assert!(h.kittest().query_by_label("build failed").is_some());
    assert!(
        h.kittest()
            .query_by_label("Hint: Check that the profile is closed")
            .is_some()
    );
    assert!(
        h.kittest().query_by_label_contains("caused by").is_none(),
        "details are collapsed"
    );
    h.kittest().get_by_label("Details").click();
    h.drive_until_idle();
    let shown = h
        .kittest()
        .query_by_label_contains("caused by: degenerate geometry: profile");
    assert!(shown.is_some(), "details show the report chain");
    let n = &h.app().notifications().items()[0].notification;
    assert_eq!(
        n.details.as_deref(),
        Some("build failed\n  caused by: degenerate geometry: profile")
    );
}

#[test]
fn info_toasts_hide_after_six_seconds_errors_stay_until_dismissed() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    h.app_mut()
        .notify(Notification::info("Saved", "Part saved"));
    h.app_mut()
        .notify(Notification::warning("Careful", "Thin wall"));
    h.app_mut()
        .notify(Notification::error("Broke", "Disk full"));
    settle(&mut h);
    for text in ["Part saved", "Thin wall", "Disk full"] {
        assert!(h.kittest().query_by_label(text).is_some(), "{text}");
    }
    h.app_mut()
        .advance_notification_clock(Duration::from_millis(5900));
    settle(&mut h);
    assert!(h.kittest().query_by_label("Part saved").is_some(), "5.9 s");
    h.app_mut()
        .advance_notification_clock(Duration::from_millis(200));
    settle(&mut h);
    assert!(h.kittest().query_by_label("Part saved").is_none(), "6.1 s");
    assert!(h.kittest().query_by_label("Thin wall").is_none());
    assert!(
        h.kittest().query_by_label("Disk full").is_some(),
        "error stays"
    );
    h.app_mut()
        .advance_notification_clock(Duration::from_secs(3600));
    settle(&mut h);
    assert!(
        h.kittest().query_by_label("Disk full").is_some(),
        "still there"
    );
    h.kittest().get_by_label("Dismiss").click();
    settle(&mut h);
    assert!(h.kittest().query_by_label("Disk full").is_none());
    assert!(h.app().notifications().is_empty());
}

#[test]
fn open_log_folder_and_copy_details_buttons() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    let opened: Rc<RefCell<Vec<PathBuf>>> = Rc::default();
    let rec = Rc::clone(&opened);
    h.app_mut()
        .set_folder_opener(move |p| rec.borrow_mut().push(p.to_path_buf()));
    h.app_mut().set_log_dir(PathBuf::from("logs-dir"));
    h.app_mut().notify(
        Notification::error("Broke", "Disk full").with_details("disk full\n  caused by: io"),
    );
    h.drive_until_idle();
    h.kittest().get_by_label("Open log folder").click();
    h.drive_until_idle();
    assert_eq!(*opened.borrow(), vec![PathBuf::from("logs-dir")]);
    h.kittest().get_by_label("Copy details").click();
    h.kittest().step();
    let copied = h.kittest().output().platform_output.commands.iter().any(
        |c| matches!(c, egui::OutputCommand::CopyText(t) if t == "disk full\n  caused by: io"),
    );
    assert!(copied, "details were copied");
    assert!(
        !h.app().notifications().is_empty(),
        "copy and open leave the toast"
    );
}

#[test]
fn jobs_are_grouped_under_their_parent_and_cancel_reaches_children() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    let outcome: Rc<Cell<Option<bool>>> = Rc::default();
    let o = Rc::clone(&outcome);
    h.app_mut()
        .spawn_tracked(Parent, SpawnOptions::default(), move |res, _| {
            o.set(Some(matches!(res, Err(JobError::Cancelled))));
        });
    pump(&mut h, "child spawned", |h| {
        h.app().jobs().jobs().len() == 2
    });
    let infos = h.app().jobs().jobs();
    assert_eq!(infos[1].parent, Some(infos[0].id));
    h.kittest().get_by_label("Jobs: 2").click();
    pump(&mut h, "both rows shown", |h| {
        h.kittest().query_by_label("Parent job").is_some()
            && h.kittest().query_by_label("Child job").is_some()
    });
    settle(&mut h);
    h.kittest()
        .query_all_by_label("Cancel")
        .next()
        .unwrap()
        .click();
    pump(&mut h, "parent resolved", |_| outcome.get().is_some());
    assert_eq!(outcome.get(), Some(true));
    h.drive_until_idle();
    assert!(h.app().notifications().is_empty());
}

struct LostViewport(Arc<AtomicBool>);

impl ViewportBackend for LostViewport {
    fn show(&mut self, ui: &mut egui::Ui, frame: &ViewportFrame<'_>) -> egui::Response {
        NullViewport.show(ui, frame)
    }
    fn device_lost(&self) -> bool {
        self.0.load(Ordering::SeqCst)
    }
}

#[test]
fn device_lost_shows_the_banner_and_save_stays_enabled() {
    let lost = Arc::new(AtomicBool::new(false));
    let mut h = AppHarness::builder()
        .viewport(Box::new(LostViewport(Arc::clone(&lost))))
        .build();
    h.drive_until_idle();
    let banner = "GPU device lost — save your work and restart Amalgam";
    assert!(h.kittest().query_by_label(banner).is_none());

    lost.store(true, Ordering::SeqCst);
    h.kittest().step();
    assert!(h.kittest().query_by_label(banner).is_some(), "banner shown");
    assert!(h.app().device_lost());

    h.kittest().get_by_label("File").click();
    h.drive_until_idle();
    let save = h.kittest().get_by_label("Save");
    assert!(!save.accesskit_node().is_disabled(), "Save is enabled");
    let before = SAVES.load(Ordering::SeqCst);
    save.click();
    h.drive_until_idle();
    assert_eq!(SAVES.load(Ordering::SeqCst), before + 1, "Save ran");
}
