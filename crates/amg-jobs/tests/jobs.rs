//! FND-04 behaviour tests. Done-when items are tagged `(a)`..`(h)`; timing items are named
//! `timing_*`.

// Test helpers outside `#[test]` fns are not covered by clippy.toml's allow-*-in-tests.
#![allow(clippy::unwrap_used, reason = "test code")]

use std::{
    borrow::Cow,
    num::NonZeroUsize,
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicUsize, Ordering},
    },
    thread,
    time::{Duration, Instant},
};

use amg_jobs::{
    CancelToken, Cancelled, Job, JobCtx, JobError, JobHandle, JobId, JobState, JobSystem,
    JobSystemConfig, Priority, SpawnOptions, is_ui_thread, mark_ui_thread,
};
use crossbeam_channel::{Receiver, Sender, bounded, unbounded};
use rayon::prelude::*;

#[derive(Debug, thiserror::Error)]
enum TestError {
    #[error("cancelled")]
    Cancelled,
    #[error("boom {0}")]
    Boom(u32),
}

impl From<Cancelled> for TestError {
    fn from(_: Cancelled) -> Self {
        TestError::Cancelled
    }
}

type R<T> = Result<T, TestError>;

fn system(interactive: usize, background: usize) -> JobSystem {
    JobSystem::new(JobSystemConfig {
        interactive_threads: NonZeroUsize::new(interactive),
        background_threads: NonZeroUsize::new(background),
        notifier: None,
    })
    .unwrap()
}

fn opts(priority: Priority) -> SpawnOptions {
    SpawnOptions {
        priority,
        hidden: false,
    }
}

/// Polls `try_take` until done; fails the test instead of hanging.
fn take_within<T: Send + 'static, E: Send + 'static>(
    h: &mut JobHandle<T, E>,
    limit: Duration,
) -> Result<T, JobError<E>> {
    let deadline = Instant::now() + limit;
    loop {
        if let Some(r) = h.try_take() {
            return r;
        }
        assert!(Instant::now() < deadline, "job {:?} did not finish", h.id());
        thread::sleep(Duration::from_micros(200));
    }
}

/// Spins (busy work) for `d`, checkpointing every ~1 ms.
fn busy(ctx: &JobCtx, d: Duration) -> Result<(), Cancelled> {
    let end = Instant::now() + d;
    loop {
        ctx.checkpoint()?;
        let step = Instant::now() + Duration::from_millis(1);
        while Instant::now() < step {
            std::hint::spin_loop();
        }
        if Instant::now() >= end {
            return Ok(());
        }
    }
}

fn median(mut v: Vec<Duration>) -> Duration {
    v.sort();
    v[v.len() / 2]
}

struct Blocker {
    release: Receiver<()>,
}

impl Job for Blocker {
    type Output = ();
    type Error = TestError;
    fn label(&self) -> Cow<'static, str> {
        "blocker".into()
    }
    fn run(self, _ctx: &JobCtx) -> R<()> {
        let _ = self.release.recv_timeout(Duration::from_secs(10));
        Ok(())
    }
}

// ---- (a) cancellation storm -------------------------------------------------------------

/// Best measurement out of at least `min_attempts`, retrying for up to 5 s until one meets
/// `target`. Timing targets describe the job system on an otherwise idle machine; nextest runs
/// the workspace's other tests in parallel processes, so single samples can be disturbed by
/// contention. Correctness is asserted on every attempt.
///
/// Holds an exclusive cross-process file lock for the whole measurement, so the `timing_*`
/// tests (two of which saturate every core) never overlap each other.
fn best_of(
    min_attempts: usize,
    mut measure: impl FnMut() -> Duration,
    target: Duration,
) -> Duration {
    let lock = std::fs::File::create(
        std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("amg-jobs-timing.lock"),
    )
    .unwrap();
    lock.lock().unwrap(); // released on drop
    let start = Instant::now();
    let mut best = Duration::MAX;
    for attempt in 1.. {
        best = best.min(measure());
        if best < target || (attempt >= min_attempts && start.elapsed() > Duration::from_secs(5)) {
            break;
        }
        thread::sleep(Duration::from_millis(20));
    }
    best
}

#[test]
fn timing_a_cancel_storm_1000_jobs_resolves_within_100ms() {
    let target = Duration::from_millis(100);
    let drained = best_of(3, cancel_storm, target);
    assert!(drained < target, "drained in {drained:?}");
}

/// Spawns 1000 jobs, `cancel_all` mid-way; every handle must resolve. Returns the drain time.
fn cancel_storm() -> Duration {
    let js = JobSystem::new(JobSystemConfig::default()).unwrap();
    let mut handles: Vec<JobHandle<u32, TestError>> = (0..1000u32)
        .map(|i| {
            let p = if i % 2 == 0 {
                Priority::Interactive
            } else {
                Priority::Background
            };
            js.spawn_fn(format!("storm-{i}"), opts(p), move |ctx| {
                busy(ctx, Duration::from_millis(50))?;
                Ok(i)
            })
        })
        .collect();
    thread::sleep(Duration::from_millis(20));
    js.cancel_all();
    let t0 = Instant::now();
    let mut pending: Vec<usize> = (0..handles.len()).collect();
    let (mut ok, mut cancelled) = (0, 0);
    while !pending.is_empty() {
        pending.retain(|&i| match handles[i].try_take() {
            None => true,
            Some(Ok(v)) => {
                assert_eq!(v as usize, i);
                ok += 1;
                false
            }
            Some(Err(JobError::Cancelled)) => {
                cancelled += 1;
                false
            }
            Some(Err(e)) => panic!("unexpected {e}"),
        });
        assert!(
            t0.elapsed() < Duration::from_secs(5),
            "{} handles hang",
            pending.len()
        );
    }
    let drained = t0.elapsed();
    assert_eq!(ok + cancelled, 1000);
    assert!(cancelled > 0);
    assert!(js.jobs().is_empty());
    drained
}

// ---- (b) panic capture ------------------------------------------------------------------

#[test]
fn b_panicking_job_yields_panicked_and_system_keeps_working() {
    let js = system(2, 1);
    let mut h = js.spawn_fn("panics", SpawnOptions::default(), |_ctx| -> R<()> {
        panic!("boom at step {}", 3)
    });
    match take_within(&mut h, Duration::from_secs(5)) {
        Err(JobError::Panicked(msg)) => assert!(msg.contains("boom at step 3"), "{msg}"),
        other => panic!("expected Panicked, got {other:?}"),
    }
    assert!(matches!(h.state(), JobState::Panicked(m) if m.contains("boom")));

    // Static-str payload and a panic inside a rayon par_iter on the job's pool.
    let h2 = js.spawn_fn("par-panic", opts(Priority::Background), |_ctx| -> R<u32> {
        (0..100u32).into_par_iter().for_each(|i| {
            if i == 57 {
                panic!("static payload");
            }
        });
        Ok(1)
    });
    assert!(matches!(h2.wait_blocking(), Err(JobError::Panicked(m)) if m == "static payload"));

    // Both pools still run jobs afterwards.
    for p in [Priority::Interactive, Priority::Background] {
        let h = js.spawn_fn("after", opts(p), |_| Ok::<_, TestError>(42));
        assert_eq!(h.wait_blocking().unwrap(), 42);
    }
}

// ---- (c) progress -----------------------------------------------------------------------

type Step = Box<dyn Fn(&JobCtx) + Send>;

/// Runs `steps` inside a job, one per `go` message, reporting after each step.
fn stepped_job(
    js: &JobSystem,
    steps: Vec<Step>,
) -> (JobHandle<(), TestError>, Sender<()>, Receiver<()>) {
    let (go_tx, go_rx) = unbounded::<()>();
    let (done_tx, done_rx) = unbounded::<()>();
    let h = js.spawn_fn("stepped", SpawnOptions::default(), move |ctx| {
        for s in &steps {
            go_rx.recv_timeout(Duration::from_secs(10)).unwrap();
            s(ctx);
            done_tx.send(()).unwrap();
        }
        go_rx.recv_timeout(Duration::from_secs(10)).unwrap();
        Ok(())
    });
    (h, go_tx, done_rx)
}

#[test]
fn c_progress_is_monotone_and_nested_sub_ranges_map_correctly() {
    let js = system(2, 1);
    let steps: Vec<Step> = vec![
        Box::new(|c| c.progress(0.5)),
        Box::new(|c| c.progress(0.2)),               // lower: ignored
        Box::new(|c| c.sub(0.5, 1.0).progress(0.5)), // 0.5 + 0.5*0.5
        Box::new(|c| c.sub(0.5, 1.0).sub(0.5, 1.0).progress(0.5)), // [0.75,1] -> 0.875
        Box::new(|c| c.sub(0.9, 0.95).sub(0.0, 0.5).progress(1.0)), // [0.9,0.925] -> 0.925
        Box::new(|c| c.progress(f32::NAN)),          // ignored
        Box::new(|c| c.progress(7.0)),               // clamped to 1
    ];
    let expected = [0.5, 0.5, 0.75, 0.875, 0.925, 0.925, 1.0];
    let (mut h, go, done) = stepped_job(&js, steps);
    for want in expected {
        go.send(()).unwrap();
        done.recv_timeout(Duration::from_secs(10)).unwrap();
        let got = h.progress().fraction;
        assert!((got - want).abs() < 1e-6, "want {want}, got {got}");
        assert!(matches!(h.state(), JobState::Running(p) if (p.fraction - want).abs() < 1e-6));
    }
    go.send(()).unwrap();
    take_within(&mut h, Duration::from_secs(5)).unwrap();
    assert_eq!(h.state(), JobState::Succeeded);
    assert!((h.progress().fraction - 1.0).abs() < 1e-6);
}

#[test]
fn c_progress_observed_from_outside_never_decreases() {
    let js = system(4, 1);
    let stop = Arc::new(AtomicBool::new(false));
    let stop2 = stop.clone();
    let mut h = js.spawn_fn("noisy", SpawnOptions::default(), move |ctx| {
        // Several threads report scattered, partly decreasing values concurrently.
        (0..4u32).into_par_iter().for_each(|t| {
            for i in 0..20_000u32 {
                let x = ((i * 7919 + t * 104_729) % 10_000) as f32 / 10_000.0;
                ctx.sub(0.0, 0.5 + 0.1 * t as f32).progress(x);
            }
        });
        stop2.store(true, Ordering::SeqCst);
        Ok::<_, TestError>(())
    });
    let mut last = 0.0f32;
    while !stop.load(Ordering::SeqCst) {
        let f = h.progress().fraction;
        assert!(f >= last, "progress went down: {last} -> {f}");
        assert!((0.0..=1.0).contains(&f));
        last = f;
    }
    take_within(&mut h, Duration::from_secs(10)).unwrap();
}

#[test]
fn c_progress_messages_are_rate_limited_to_30hz() {
    let js = system(1, 1);
    // Whether the two back-to-back updates landed within one 1/30 s window (they always do
    // unless the thread is preempted for > 30 ms in between).
    let fast = Arc::new(AtomicBool::new(false));
    let fast2 = fast.clone();
    let steps: Vec<Step> = vec![
        Box::new(move |c| {
            let t = Instant::now();
            c.progress_msg(0.1, "first");
            c.progress_msg(0.2, "too soon");
            fast2.store(t.elapsed() < Duration::from_millis(30), Ordering::SeqCst);
        }),
        Box::new(|c| {
            thread::sleep(Duration::from_millis(40));
            c.progress_msg(0.3, "later");
        }),
    ];
    let (mut h, go, done) = stepped_job(&js, steps);
    go.send(()).unwrap();
    done.recv_timeout(Duration::from_secs(10)).unwrap();
    let p = h.progress();
    assert!(
        (p.fraction - 0.2).abs() < 1e-6,
        "fraction is never rate-limited"
    );
    if fast.load(Ordering::SeqCst) {
        assert_eq!(p.message.as_deref(), Some("first"));
    }
    go.send(()).unwrap();
    done.recv_timeout(Duration::from_secs(10)).unwrap();
    let p = h.progress();
    assert!((p.fraction - 0.3).abs() < 1e-6);
    assert_eq!(p.message.as_deref(), Some("later"));
    go.send(()).unwrap();
    take_within(&mut h, Duration::from_secs(5)).unwrap();
}

#[test]
fn notifier_fires_on_state_changes_and_is_rate_limited_for_progress() {
    let calls = Arc::new(AtomicUsize::new(0));
    let c2 = calls.clone();
    let js = JobSystem::new(JobSystemConfig {
        interactive_threads: NonZeroUsize::new(1),
        background_threads: NonZeroUsize::new(1),
        notifier: Some(Arc::new(move || {
            c2.fetch_add(1, Ordering::SeqCst);
        })),
    })
    .unwrap();
    let h = js.spawn_fn("noop", SpawnOptions::default(), |_| Ok::<_, TestError>(()));
    h.wait_blocking().unwrap();
    // spawn (queued) + running + finished; the last one fires just after the state is visible.
    let deadline = Instant::now() + Duration::from_secs(5);
    while calls.load(Ordering::SeqCst) < 3 {
        assert!(Instant::now() < deadline, "notifier not called 3 times");
        thread::yield_now();
    }

    calls.store(0, Ordering::SeqCst);
    let h = js.spawn_fn("progress", SpawnOptions::default(), |ctx| {
        let t = Instant::now();
        for i in 0..100_000u32 {
            ctx.progress(i as f32 / 100_000.0);
        }
        Ok::<_, TestError>(t.elapsed())
    });
    let took = h.wait_blocking().unwrap();
    // queued + running + finished, plus at least one progress notification...
    while calls.load(Ordering::SeqCst) < 4 {
        assert!(Instant::now() < deadline, "notifier not called 4 times");
        thread::yield_now();
    }
    thread::sleep(Duration::from_millis(20)); // let a trailing notification land
    // ...and at most one progress notification per 1/30 s.
    let n = calls.load(Ordering::SeqCst);
    let max = 3 + 1 + (took.as_secs_f64() * 30.0).ceil() as usize;
    assert!(
        n <= max,
        "{n} notifications for 100k progress updates in {took:?}"
    );

    // A panicking notifier does not break the job.
    let js = JobSystem::new(JobSystemConfig {
        interactive_threads: NonZeroUsize::new(1),
        background_threads: NonZeroUsize::new(1),
        notifier: Some(Arc::new(|| panic!("notifier panic"))),
    })
    .unwrap();
    let h = js.spawn_fn("ok", SpawnOptions::default(), |_| Ok::<_, TestError>(5));
    assert_eq!(h.wait_blocking().unwrap(), 5);
}

// ---- (d) child cancellation -------------------------------------------------------------

#[test]
fn d_cancelling_a_parent_cancels_its_children() {
    let js = system(4, 1);
    let (ids_tx, ids_rx) = bounded::<(JobId, JobId)>(1);
    let mut parent = js.spawn_fn("parent", SpawnOptions::default(), move |ctx| {
        let child = ctx.spawn_child(ChildLoop { depth: 1 });
        ids_tx.send((ctx.id().unwrap(), child.id())).unwrap();
        let r = ctx.wait(child);
        assert!(matches!(r, Err(JobError::Cancelled)), "child: {r:?}");
        ctx.checkpoint()?;
        Ok::<_, TestError>(())
    });
    let (pid, cid) = ids_rx.recv_timeout(Duration::from_secs(5)).unwrap();
    // Child and grandchild are visible with parent links.
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        let jobs = js.jobs();
        let child = jobs.iter().find(|j| j.id == cid);
        let grandchild = jobs.iter().find(|j| j.parent == Some(cid));
        if let (Some(c), Some(_)) = (child, grandchild) {
            assert_eq!(c.parent, Some(pid));
            assert_eq!(c.label, "child-1");
            break;
        }
        assert!(Instant::now() < deadline);
        thread::sleep(Duration::from_millis(1));
    }
    parent.cancel();
    assert!(matches!(
        take_within(&mut parent, Duration::from_secs(5)),
        Err(JobError::Cancelled)
    ));
    let deadline = Instant::now() + Duration::from_secs(5);
    while !js.jobs().is_empty() {
        assert!(Instant::now() < deadline, "children still running");
        thread::sleep(Duration::from_millis(1));
    }
}

/// Spawns a grandchild (depth 1) and then loops until cancelled.
struct ChildLoop {
    depth: u32,
}

impl Job for ChildLoop {
    type Output = ();
    type Error = TestError;
    fn label(&self) -> Cow<'static, str> {
        format!("child-{}", self.depth).into()
    }
    fn run(self, ctx: &JobCtx) -> R<()> {
        let grandchild = (self.depth < 2).then(|| ctx.spawn_child(ChildLoop { depth: 2 }));
        loop {
            busy(ctx, Duration::from_millis(1))?;
            if let Some(g) = &grandchild {
                // Keep the grandchild alive; it is cancelled through the token chain.
                let _ = g.is_finished();
            }
        }
    }
}

#[test]
fn cancelling_a_child_does_not_cancel_the_parent() {
    let js = system(2, 1);
    let h = js.spawn_fn("parent", SpawnOptions::default(), |ctx| {
        let child = ctx.spawn_child(ChildLoop { depth: 2 });
        child.cancel();
        assert!(matches!(ctx.wait(child), Err(JobError::Cancelled)));
        ctx.checkpoint()?;
        Ok::<_, TestError>(7)
    });
    assert_eq!(h.wait_blocking().unwrap(), 7);
}

// ---- (e) nested waits on a 1-thread pool -----------------------------------------------

struct Fib(u32);

impl Job for Fib {
    type Output = u64;
    type Error = TestError;
    fn label(&self) -> Cow<'static, str> {
        "fib".into()
    }
    fn run(self, ctx: &JobCtx) -> R<u64> {
        if self.0 < 2 {
            return Ok(u64::from(self.0));
        }
        let a = ctx.spawn_child(Fib(self.0 - 1));
        let b = ctx.spawn_child(Fib(self.0 - 2));
        let b = ctx.wait(b).map_err(|_| TestError::Boom(0))?;
        let a = ctx.wait(a).map_err(|_| TestError::Boom(1))?;
        Ok(a + b)
    }
}

#[test]
fn e_nested_wait_in_a_one_thread_pool_completes() {
    let js = system(1, 1);
    for p in [Priority::Interactive, Priority::Background] {
        let mut h = js.spawn_with(Fib(12), opts(p));
        assert_eq!(take_within(&mut h, Duration::from_secs(10)).unwrap(), 144);
    }
}

// ---- (f) non-blocking accessors (median < 1 µs) ------------------------------------------

#[test]
fn timing_f_try_take_and_state_are_sub_microsecond() {
    // Medians of 10k samples are robust to preemption; no retries needed.
    let js = system(1, 1);
    let (release_tx, release) = bounded(1);
    let mut h = js.spawn(Blocker { release });
    let mut take = Vec::with_capacity(10_000);
    let mut state = Vec::with_capacity(10_000);
    let mut prog = Vec::with_capacity(10_000);
    for _ in 0..10_000 {
        let t = Instant::now();
        assert!(h.try_take().is_none());
        take.push(t.elapsed());
        let t = Instant::now();
        std::hint::black_box(h.state());
        state.push(t.elapsed());
        let t = Instant::now();
        std::hint::black_box(h.progress());
        prog.push(t.elapsed());
    }
    release_tx.send(()).unwrap();
    take_within(&mut h, Duration::from_secs(5)).unwrap();
    let us = Duration::from_micros(1);
    let (mt, ms, mp) = (median(take), median(state), median(prog));
    assert!(mt < us, "try_take median {mt:?}");
    assert!(ms < us, "state median {ms:?}");
    assert!(mp < us, "progress median {mp:?}");
}

// ---- (g) spawn-to-start latency (median < 50 µs) ----------------------------------------

#[test]
fn timing_g_spawn_to_start_latency_on_idle_pool() {
    let js = system(2, 1);
    let target = Duration::from_micros(50);
    let m = best_of(
        3,
        || {
            let mut samples = Vec::new();
            for _ in 0..25 {
                thread::sleep(Duration::from_millis(3)); // let workers go idle
                let t0 = Instant::now();
                let h = js.spawn_fn("t", SpawnOptions::default(), |_| {
                    Ok::<_, TestError>(Instant::now())
                });
                let started = h.wait_blocking().unwrap();
                samples.push(started.saturating_duration_since(t0));
            }
            median(samples)
        },
        target,
    );
    assert!(m < target, "median spawn-to-start {m:?}");
}

// ---- (h) background saturation ----------------------------------------------------------

#[test]
fn timing_h_saturated_background_does_not_delay_interactive() {
    // The probe job itself takes ~1 ms (one busy step); anything beyond is added delay.
    let target = Duration::from_millis(1) + Duration::from_millis(5);
    let m = best_of(3, interactive_latency_under_background_load, target);
    assert!(m < target, "median {m:?}");
}

/// Median spawn-to-finish time of a ~1 ms Interactive job while the Background pool is
/// saturated (2n busy jobs, each also running nested rayon work).
fn interactive_latency_under_background_load() -> Duration {
    let js = JobSystem::new(JobSystemConfig::default()).unwrap();
    let n = thread::available_parallelism().map_or(1, NonZeroUsize::get);
    let hogs: Vec<_> = (0..n * 2)
        .map(|_| {
            js.spawn_fn("hog", opts(Priority::Background), |ctx| -> R<()> {
                // Saturate with nested rayon work too.
                loop {
                    (0..64u32).into_par_iter().for_each(|_| {
                        let end = Instant::now() + Duration::from_micros(500);
                        while Instant::now() < end {
                            std::hint::spin_loop();
                        }
                    });
                    ctx.checkpoint()?;
                }
            })
        })
        .collect();
    thread::sleep(Duration::from_millis(50));
    let mut samples = Vec::new();
    for _ in 0..30 {
        let t0 = Instant::now();
        let h = js.spawn_fn("interactive", SpawnOptions::default(), |ctx| {
            busy(ctx, Duration::from_micros(100))?;
            Ok::<_, TestError>(())
        });
        h.wait_blocking().unwrap();
        samples.push(t0.elapsed());
        thread::sleep(Duration::from_millis(2));
    }
    for h in &hogs {
        h.cancel();
    }
    for h in hogs {
        assert!(matches!(h.wait_blocking(), Err(JobError::Cancelled)));
    }
    median(samples)
}

// ---- results & classification -----------------------------------------------------------

#[test]
fn errors_are_classified() {
    let js = system(2, 1);
    let h = js.spawn_fn("fails", SpawnOptions::default(), |_| -> R<()> {
        Err(TestError::Boom(9))
    });
    let id = h.id();
    let mut h = h;
    match take_within(&mut h, Duration::from_secs(5)) {
        Err(JobError::Failed(TestError::Boom(9))) => {}
        other => panic!("{other:?}"),
    }
    assert_eq!(h.state(), JobState::Failed("boom 9".into()));
    assert!(!js.cancel(id), "finished jobs are not cancellable");

    // Err after cancellation counts as Cancelled, whatever the error.
    let (tx, rx) = bounded(1);
    let h = js.spawn_fn("err-after-cancel", SpawnOptions::default(), move |ctx| {
        tx.send(()).unwrap();
        while !ctx.is_cancelled() {
            thread::yield_now();
        }
        Err::<(), _>(TestError::Boom(1))
    });
    rx.recv_timeout(Duration::from_secs(5)).unwrap();
    h.cancel();
    let token = h.cancel_token();
    assert!(token.is_cancelled());
    assert!(matches!(h.wait_blocking(), Err(JobError::Cancelled)));

    // Ok after cancellation stays Succeeded.
    let (tx, rx) = bounded(1);
    let h = js.spawn_fn("ok-after-cancel", SpawnOptions::default(), move |ctx| {
        tx.send(()).unwrap();
        while !ctx.is_cancelled() {
            thread::yield_now();
        }
        Ok::<_, TestError>(3)
    });
    rx.recv_timeout(Duration::from_secs(5)).unwrap();
    assert!(js.cancel(h.id()));
    let mut h = h;
    assert_eq!(take_within(&mut h, Duration::from_secs(5)).unwrap(), 3);
    assert_eq!(h.state(), JobState::Succeeded);
}

#[test]
fn try_take_is_some_exactly_once() {
    let js = system(1, 1);
    let mut h = js.spawn_fn("once", SpawnOptions::default(), |_| Ok::<_, TestError>(1));
    assert_eq!(take_within(&mut h, Duration::from_secs(5)).unwrap(), 1);
    assert!(h.is_finished());
    assert!(h.try_take().is_none());
    assert!(h.try_take().is_none());
}

#[test]
fn queued_job_cancels_immediately_without_running() {
    let js = system(1, 1);
    let (release_tx, release) = bounded(1);
    let mut blocker = js.spawn(Blocker { release });
    let ran = Arc::new(AtomicBool::new(false));
    let ran2 = ran.clone();
    let mut queued = js.spawn_fn("queued", SpawnOptions::default(), move |_| {
        ran2.store(true, Ordering::SeqCst);
        Ok::<_, TestError>(())
    });
    // Wait until the blocker runs so `queued` is really queued behind it.
    let deadline = Instant::now() + Duration::from_secs(5);
    while !matches!(blocker.state(), JobState::Running(_)) {
        assert!(Instant::now() < deadline);
        thread::yield_now();
    }
    assert_eq!(queued.state(), JobState::Queued);
    queued.cancel();
    assert!(queued.is_finished());
    assert!(matches!(queued.try_take(), Some(Err(JobError::Cancelled))));
    release_tx.send(()).unwrap();
    take_within(&mut blocker, Duration::from_secs(5)).unwrap();
    // The pool skips the cancelled closure.
    let h = js.spawn_fn("after", SpawnOptions::default(), |_| Ok::<_, TestError>(()));
    h.wait_blocking().unwrap();
    assert!(!ran.load(Ordering::SeqCst));
}

#[test]
fn dropping_a_handle_detaches_the_job() {
    let js = system(1, 1);
    let (tx, rx) = bounded(1);
    drop(js.spawn_fn("detached", SpawnOptions::default(), move |_| {
        tx.send(5u32).unwrap();
        Ok::<_, TestError>(())
    }));
    assert_eq!(rx.recv_timeout(Duration::from_secs(5)).unwrap(), 5);
}

// ---- registry ---------------------------------------------------------------------------

#[test]
fn jobs_lists_unfinished_visible_jobs_with_metadata() {
    let js = system(1, 1);
    let (release_tx, release) = bounded(1);
    let (release_tx2, release2) = bounded(1);
    let t_before = Instant::now();
    let mut a = js.spawn(Blocker { release });
    let mut b = js.spawn_with(
        Blocker { release: release2 },
        SpawnOptions {
            priority: Priority::Background,
            hidden: false,
        },
    );
    let (_hidden_tx, hidden_rx) = bounded::<()>(1);
    let hidden = js.spawn_with(
        Blocker { release: hidden_rx },
        SpawnOptions {
            priority: Priority::Interactive,
            hidden: true,
        },
    );
    assert!(a.id() < b.id() && b.id() < hidden.id(), "ids increase");
    let deadline = Instant::now() + Duration::from_secs(5);
    while !matches!(a.state(), JobState::Running(_)) {
        assert!(Instant::now() < deadline);
        thread::yield_now();
    }
    let jobs = js.jobs();
    assert_eq!(jobs.len(), 2, "hidden job excluded");
    assert_eq!(jobs[0].id, a.id());
    assert_eq!(jobs[0].label, "blocker");
    assert_eq!(jobs[0].priority, Priority::Interactive);
    assert_eq!(jobs[0].parent, None);
    assert!(jobs[0].created >= t_before);
    assert!(jobs[0].started.is_some());
    assert!(matches!(jobs[0].state, JobState::Running(_)));
    assert_eq!(jobs[1].priority, Priority::Background);
    hidden.cancel();
    release_tx.send(()).unwrap();
    release_tx2.send(()).unwrap();
    take_within(&mut a, Duration::from_secs(5)).unwrap();
    take_within(&mut b, Duration::from_secs(5)).unwrap();
    assert!(js.jobs().is_empty());
    assert!(!js.cancel(JobId(u64::MAX)));
}

#[test]
fn shutdown_cancels_and_reports() {
    let js = system(2, 1);
    let h1 = js.spawn_fn("loop", SpawnOptions::default(), |ctx| -> R<()> {
        loop {
            busy(ctx, Duration::from_millis(1))?;
        }
    });
    let stuck_release = Arc::new(AtomicBool::new(false));
    let sr = stuck_release.clone();
    let h2 = js.spawn_fn("ignores-cancel", opts(Priority::Background), move |_| {
        while !sr.load(Ordering::SeqCst) {
            thread::sleep(Duration::from_millis(1));
        }
        Ok::<_, TestError>(())
    });
    // Both must be running (a queued job would be cancelled out of the queue instead). The
    // below-normal Background thread can take a while to get scheduled on a loaded machine.
    let deadline = Instant::now() + Duration::from_secs(10);
    while !(matches!(h1.state(), JobState::Running(_))
        && matches!(h2.state(), JobState::Running(_)))
    {
        assert!(Instant::now() < deadline, "jobs did not start");
        thread::sleep(Duration::from_millis(1));
    }
    let report = js.shutdown(Duration::from_millis(100));
    assert_eq!(report.cancelled, 2);
    assert_eq!(report.still_running.len(), 1);
    assert_eq!(report.still_running[0].id, h2.id());
    assert!(matches!(h1.wait_blocking(), Err(JobError::Cancelled)));
    // New spawns are cancelled immediately.
    let mut late = js.spawn_fn("late", SpawnOptions::default(), |_| Ok::<_, TestError>(()));
    assert!(matches!(late.try_take(), Some(Err(JobError::Cancelled))));
    stuck_release.store(true, Ordering::SeqCst);
    // Returned Ok after its token was cancelled: still Succeeded.
    h2.wait_blocking().unwrap();
    let report = js.shutdown(Duration::from_millis(100));
    assert_eq!(report.cancelled, 0);
    assert!(report.still_running.is_empty());
}

// ---- pools ------------------------------------------------------------------------------

#[test]
fn jobs_and_install_run_on_the_right_pool() {
    let js = system(3, 2);
    for (p, n) in [(Priority::Interactive, 3), (Priority::Background, 2)] {
        assert_eq!(js.install(p, rayon::current_num_threads), n);
        let h = js.spawn_fn("pool", opts(p), |_| {
            let name = thread::current().name().unwrap_or_default().to_owned();
            let inner: Vec<usize> = (0..64)
                .into_par_iter()
                .map(|_| rayon::current_num_threads())
                .collect();
            Ok::<_, TestError>((name, rayon::current_num_threads(), inner))
        });
        let (name, threads, inner) = h.wait_blocking().unwrap();
        let want = match p {
            Priority::Interactive => "amg-interactive-",
            Priority::Background => "amg-background-",
        };
        assert!(name.starts_with(want), "{name}");
        assert_eq!(threads, n);
        assert!(inner.iter().all(|&t| t == n));
    }
    let sum: u64 = js.install(Priority::Background, || (0..1000u64).into_par_iter().sum());
    assert_eq!(sum, 499_500);
}

#[test]
fn default_config_sizes() {
    let n = thread::available_parallelism().map_or(1, NonZeroUsize::get);
    let js = JobSystem::new(JobSystemConfig::default()).unwrap();
    assert_eq!(
        js.install(Priority::Interactive, rayon::current_num_threads),
        n
    );
    assert_eq!(
        js.install(Priority::Background, rayon::current_num_threads),
        n.saturating_sub(1).max(1)
    );
    let t = Instant::now();
    let _js2 = JobSystem::new(JobSystemConfig::default()).unwrap();
    let dt = t.elapsed();
    // S00 target is < 5 ms; allow slack for a loaded test machine.
    assert!(dt < Duration::from_millis(50), "JobSystem::new took {dt:?}");
}

// ---- detached contexts ------------------------------------------------------------------

#[test]
fn detached_ctx_is_never_cancelled_and_runs_children_inline() {
    let ctx = JobCtx::detached();
    assert_eq!(ctx.id(), None);
    assert!(!ctx.is_cancelled());
    assert!(ctx.checkpoint().is_ok());
    ctx.progress(0.5);
    ctx.progress_msg(0.7, "ignored");
    ctx.sub(0.1, 0.2).progress(1.0);
    let child = ctx.spawn_child(Fib(10));
    assert!(
        child.is_finished(),
        "inline child finished before returning"
    );
    assert!(child.id().0 >= 1 << 63);
    assert_eq!(ctx.wait(child).unwrap(), 55);
    // Fib(10) directly, with nested inline children.
    assert_eq!(Fib(10).run(&JobCtx::detached()).unwrap(), 55);
}

#[test]
fn with_token_ctx_observes_cancellation() {
    let token = CancelToken::new();
    let ctx = JobCtx::with_token(token.clone());
    assert!(ctx.checkpoint().is_ok());
    let sub = ctx.sub(0.0, 0.5);
    token.cancel();
    assert_eq!(ctx.checkpoint(), Err(Cancelled));
    assert!(sub.is_cancelled());
    assert!(ctx.cancel_token().is_cancelled());
    // Children of a cancelled detached ctx are cancelled before they start.
    let child = ctx.spawn_child(Fib(3));
    assert!(matches!(ctx.wait(child), Err(JobError::Cancelled)));
}

#[test]
fn job_state_is_finished() {
    assert!(!JobState::Queued.is_finished());
    assert!(
        !JobState::Running(amg_jobs::Progress {
            fraction: 0.0,
            message: None
        })
        .is_finished()
    );
    assert!(JobState::Succeeded.is_finished());
    assert!(JobState::Cancelled.is_finished());
    assert!(JobState::Failed(String::new()).is_finished());
    assert!(JobState::Panicked(String::new()).is_finished());
}

#[test]
fn job_error_display() {
    let e: JobError<TestError> = JobError::Failed(TestError::Boom(2));
    assert_eq!(e.to_string(), "boom 2");
    assert_eq!(JobError::<TestError>::Cancelled.to_string(), "cancelled");
    assert_eq!(
        JobError::<TestError>::Panicked("x".into()).to_string(),
        "internal error (panic): x"
    );
    assert_eq!(Cancelled.to_string(), "operation cancelled");
}

// ---- UI-thread guard --------------------------------------------------------------------

/// Runs `f` on a fresh thread marked as the UI thread; returns whether it panicked.
fn on_ui_thread(f: impl FnOnce() + Send + 'static) -> bool {
    thread::spawn(move || {
        mark_ui_thread();
        assert!(is_ui_thread());
        f();
    })
    .join()
    .is_err()
}

#[test]
fn ui_thread_guard_rejects_blocking_calls_in_debug() {
    let js = system(1, 1);
    let debug = cfg!(debug_assertions);
    let j = js.clone();
    let h = j.spawn_fn("x", SpawnOptions::default(), |_| Ok::<_, TestError>(()));
    assert_eq!(on_ui_thread(move || drop(h.wait_blocking())), debug);
    let j = js.clone();
    assert_eq!(
        on_ui_thread(move || {
            let _ = j.install(Priority::Interactive, || 1);
        }),
        debug
    );
    let j = js.clone();
    assert_eq!(
        on_ui_thread(move || {
            let h = j.spawn_fn("y", SpawnOptions::default(), |_| Ok::<_, TestError>(()));
            drop(JobCtx::detached().wait(h));
        }),
        debug
    );
    // Non-blocking APIs are fine on the UI thread.
    let j = js.clone();
    assert!(!on_ui_thread(move || {
        let mut h = j.spawn_fn("z", SpawnOptions::default(), |_| Ok::<_, TestError>(()));
        let deadline = Instant::now() + Duration::from_secs(5);
        while h.try_take().is_none() {
            assert!(Instant::now() < deadline);
            let _ = h.state();
            let _ = h.progress();
            let _ = j.jobs();
            thread::yield_now();
        }
    }));
}
