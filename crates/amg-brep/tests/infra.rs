//! BREP-12 Done-when 1 and 2: OCCT messages go to `tracing` (target `occt`) and never to stdout;
//! the progress/cancel bridge reports monotone progress and cancels within 50 ms.
//!
//! Timing is asserted only with `AMG_STRICT_TIMING=1`; the measured numbers are printed always.
#![cfg(feature = "occt")]
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::fmt::Write as _;
use std::process::Command;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use amg_brep::{KernelConfig, KernelError, OcctKernel};
use amg_jobs::{JobError, JobSystem, JobSystemConfig, SpawnOptions};
use amg_occt_sys::{LOG_TARGET, ProgressSink, ProgressTarget, ffi};
use tracing::field::{Field, Visit};
use tracing_subscriber::layer::{Context, SubscriberExt};

const CHILD_MODE: &str = "AMG_BREP_LOG_CHILD";
const MARKER: &str = "amg-brep-12 occt message probe";

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}

// ---- Done-when 1 ----

#[derive(Clone, Debug)]
struct Captured {
    target: String,
    level: tracing::Level,
    message: String,
}

#[derive(Clone, Default)]
struct Capture(Arc<Mutex<Vec<Captured>>>);

struct MessageVisitor(String);

impl Visit for MessageVisitor {
    fn record_debug(&mut self, field: &Field, value: &dyn std::fmt::Debug) {
        if field.name() == "message" {
            write!(self.0, "{value:?}").unwrap();
        }
    }
}

impl<S: tracing::Subscriber> tracing_subscriber::Layer<S> for Capture {
    fn on_event(&self, event: &tracing::Event<'_>, _ctx: Context<'_, S>) {
        let mut visitor = MessageVisitor(String::new());
        event.record(&mut visitor);
        self.0.lock().unwrap().push(Captured {
            target: event.metadata().target().to_owned(),
            level: *event.metadata().level(),
            message: visitor.0,
        });
    }
}

#[test]
fn occt_messages_become_tracing_events() {
    amg_brep::init(&KernelConfig::default());
    let capture = Capture::default();
    let subscriber = tracing_subscriber::registry().with(capture.clone());
    tracing::subscriber::with_default(subscriber, || {
        for gravity in 0..=4u8 {
            ffi::emit_message(gravity, &format!("{MARKER} {gravity}")).unwrap();
        }
    });
    let events = capture.0.lock().unwrap().clone();
    let ours: Vec<&Captured> = events
        .iter()
        .filter(|e| e.message.contains(MARKER))
        .collect();
    assert_eq!(ours.len(), 5, "{events:?}");
    assert!(ours.iter().all(|e| e.target == LOG_TARGET), "{ours:?}");
    use tracing::Level;
    let levels: Vec<Level> = ours.iter().map(|e| e.level).collect();
    assert_eq!(
        levels,
        [
            Level::TRACE,
            Level::INFO,
            Level::WARN,
            Level::ERROR,
            Level::ERROR
        ]
    );
    let warning = ours[2];
    assert_eq!(warning.message, format!("{MARKER} 2"));
}

/// Child half of `occt_messages_never_reach_stdout`; does nothing unless spawned by it.
#[test]
fn log_child() {
    let Ok(mode) = std::env::var(CHILD_MODE) else {
        return;
    };
    if mode == "init" {
        amg_brep::init(&KernelConfig::default());
    }
    for gravity in 1..=4u8 {
        ffi::emit_message(gravity, MARKER).unwrap();
    }
}

fn child_stdout(mode: &str) -> String {
    let output = Command::new(std::env::current_exe().unwrap())
        .args(["--exact", "log_child", "--nocapture", "--test-threads=1"])
        .env(CHILD_MODE, mode)
        .output()
        .unwrap();
    assert!(output.status.success(), "child ({mode}) failed: {output:?}");
    String::from_utf8_lossy(&output.stdout).into_owned()
}

#[test]
fn occt_messages_never_reach_stdout() {
    // Control: OCCT's default printer writes to stdout, so the probe sees the marker.
    let raw = child_stdout("raw");
    assert!(
        raw.contains(MARKER),
        "control child printed no marker:\n{raw}"
    );
    // With `init`, the same messages reach only `tracing`.
    let routed = child_stdout("init");
    assert!(!routed.contains(MARKER), "OCCT wrote to stdout:\n{routed}");
}

// ---- Done-when 2 ----

/// Records every `Show` fraction.
#[derive(Clone, Default)]
struct Recorder {
    shown: Arc<Mutex<Vec<f64>>>,
}

impl ProgressTarget for Recorder {
    fn show(&self, fraction: f64) {
        self.shown.lock().unwrap().push(fraction);
    }

    fn user_break(&self) -> bool {
        false
    }
}

fn assert_monotone(values: &[f64]) {
    for pair in values.windows(2) {
        assert!(pair[1] >= pair[0], "progress went back: {pair:?}");
    }
}

#[test]
fn bridge_reports_monotone_progress() {
    let recorder = Recorder::default();
    let sink = ProgressSink::new(Box::new(recorder.clone()));
    assert!(ffi::progress_test_op(&sink, 500, 0).unwrap());
    let shown = recorder.shown.lock().unwrap().clone();
    assert!(shown.len() >= 500, "only {} updates", shown.len());
    assert_monotone(&shown);
    assert!(shown.iter().all(|f| (0.0..=1.0).contains(f)));
    assert!((shown.last().unwrap() - 1.0).abs() <= 1e-9, "{shown:?}");
}

fn job_system() -> JobSystem {
    JobSystem::new(JobSystemConfig::default()).unwrap()
}

#[test]
fn job_progress_is_monotone() {
    let kernel = OcctKernel::new(KernelConfig::default());
    let js = job_system();
    let mut handle = js.spawn_fn("progress probe", SpawnOptions::default(), move |ctx| {
        kernel.progress_probe(300, Duration::from_millis(1), ctx)
    });
    let mut samples = Vec::new();
    let result = loop {
        samples.push(f64::from(handle.progress().fraction));
        if let Some(result) = handle.try_take() {
            break result;
        }
        std::thread::sleep(Duration::from_micros(500));
    };
    result.unwrap();
    assert_monotone(&samples);
    let intermediate = samples.iter().filter(|&&f| f > 0.0 && f < 1.0).count();
    assert!(
        intermediate >= 5,
        "too few intermediate samples: {samples:?}"
    );
    println!(
        "job progress: {} samples, {intermediate} intermediate",
        samples.len()
    );
}

#[test]
fn cancel_returns_cancelled_within_50_ms() {
    let kernel = OcctKernel::new(KernelConfig::default());
    let js = job_system();
    let returned: Arc<Mutex<Option<(Instant, bool)>>> = Arc::default();
    let returned_in_job = Arc::clone(&returned);
    // 20 s of work in 1 ms steps; it must stop long before.
    let mut handle = js.spawn_fn("cancel probe", SpawnOptions::default(), move |ctx| {
        let result = kernel.progress_probe(20_000, Duration::from_millis(1), ctx);
        let cancelled = matches!(result, Err(KernelError::Cancelled));
        *returned_in_job.lock().unwrap() = Some((Instant::now(), cancelled));
        result
    });
    let started = Instant::now();
    while handle.progress().fraction < 0.005 {
        assert!(
            started.elapsed() < Duration::from_secs(10),
            "probe never ran"
        );
        std::thread::sleep(Duration::from_millis(1));
    }
    let cancelled_at = Instant::now();
    handle.cancel();
    let result = loop {
        if let Some(result) = handle.try_take() {
            break result;
        }
        assert!(cancelled_at.elapsed() < Duration::from_secs(10), "no stop");
        std::thread::sleep(Duration::from_micros(200));
    };
    assert!(matches!(result, Err(JobError::Cancelled)), "{result:?}");
    let (at, was_cancelled) = returned.lock().unwrap().expect("job returned");
    assert!(
        was_cancelled,
        "the probe did not return KernelError::Cancelled"
    );
    let latency = at.duration_since(cancelled_at);
    println!("cancel latency: {:.3} ms", latency.as_secs_f64() * 1e3);
    if strict_timing() {
        assert!(latency < Duration::from_millis(50), "{latency:?}");
    }
}

#[test]
fn cancelled_before_start_is_cancelled() {
    let kernel = OcctKernel::new(KernelConfig::default());
    let token = amg_jobs::CancelToken::new();
    token.cancel();
    let ctx = amg_jobs::JobCtx::with_token(token);
    let result = kernel.progress_probe(10, Duration::from_millis(1), &ctx);
    assert!(matches!(result, Err(KernelError::Cancelled)), "{result:?}");
}
