//! The job system: pools, registry, spawning and execution.

use std::{
    borrow::Cow,
    collections::HashMap,
    fmt,
    marker::PhantomData,
    num::NonZeroUsize,
    panic::AssertUnwindSafe,
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicU64, Ordering},
    },
    time::{Duration, Instant},
};

use crossbeam_channel::Sender;
use parking_lot::{Condvar, Mutex};
use rayon::{ThreadPool, ThreadPoolBuilder};

use crate::{
    CancelToken, Cancelled, Job, JobCtx, JobError, JobHandle, JobId, JobInfo, Priority,
    core::{CANCELLED, FAILED, JobCore, PANICKED, SUCCEEDED},
    handle::JobResult,
    os,
    ui::is_ui_thread,
};

pub(crate) type Notifier = Arc<dyn Fn() + Send + Sync>;

/// Configuration for [`JobSystem::new`]. `Default` leaves every field unset.
#[derive(Clone, Default)]
pub struct JobSystemConfig {
    /// Interactive pool size. Default: `available_parallelism`.
    pub interactive_threads: Option<NonZeroUsize>,
    /// Background pool size. Default: `max(1, n - 1)`. Its threads run at below-normal OS
    /// priority.
    pub background_threads: Option<NonZeroUsize>,
    /// Called (from worker threads) after state changes and rate-limited (30 Hz) progress
    /// updates. The app passes `egui::Context::request_repaint`. Must be cheap; panics in it
    /// are swallowed.
    pub notifier: Option<Arc<dyn Fn() + Send + Sync>>,
}

impl fmt::Debug for JobSystemConfig {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("JobSystemConfig")
            .field("interactive_threads", &self.interactive_threads)
            .field("background_threads", &self.background_threads)
            .field("notifier", &self.notifier.as_ref().map(|_| "Fn()"))
            .finish()
    }
}

/// Error from [`JobSystem::new`].
#[derive(Debug, thiserror::Error)]
pub enum JobSystemError {
    /// A rayon pool could not be built (thread creation failed).
    #[error("failed to build thread pool: {0}")]
    Pool(#[from] rayon::ThreadPoolBuildError),
}

/// Per-spawn options.
#[derive(Clone, Debug, Default)]
pub struct SpawnOptions {
    /// Pool to run on.
    pub priority: Priority,
    /// Hidden jobs (e.g. hover picking helpers) don't appear in `jobs()`.
    pub hidden: bool,
}

/// Result of [`JobSystem::shutdown`].
#[derive(Clone, Debug)]
pub struct ShutdownReport {
    /// Number of unfinished jobs that were cancelled.
    pub cancelled: usize,
    /// Jobs still unfinished when the timeout expired.
    pub still_running: Vec<JobInfo>,
}

pub(crate) struct Inner {
    interactive: ThreadPool,
    background: ThreadPool,
    next_id: AtomicU64,
    /// Unfinished jobs. Finished jobs remove themselves.
    registry: Mutex<HashMap<JobId, Arc<JobCore>>>,
    registry_cv: Condvar,
    notifier: Option<Notifier>,
    shutting_down: AtomicBool,
}

impl Inner {
    fn pool(&self, p: Priority) -> &ThreadPool {
        match p {
            Priority::Interactive => &self.interactive,
            Priority::Background => &self.background,
        }
    }

    pub(crate) fn unregister(&self, id: JobId) {
        let mut reg = self.registry.lock();
        reg.remove(&id);
        if reg.is_empty() {
            self.registry_cv.notify_all();
        }
    }

    fn unfinished(&self) -> Vec<Arc<JobCore>> {
        self.registry.lock().values().cloned().collect()
    }
}

/// The CPU job system: two rayon pools plus a registry of unfinished jobs.
/// Cheap to clone (`Arc` inside); `Send + Sync`.
#[derive(Clone)]
pub struct JobSystem {
    inner: Arc<Inner>,
}

impl fmt::Debug for JobSystem {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("JobSystem")
            .field(
                "interactive_threads",
                &self.inner.interactive.current_num_threads(),
            )
            .field(
                "background_threads",
                &self.inner.background.current_num_threads(),
            )
            .field("unfinished", &self.inner.registry.lock().len())
            .finish()
    }
}

impl JobSystem {
    /// Builds both pools. Takes a few milliseconds (thread creation).
    pub fn new(cfg: JobSystemConfig) -> Result<JobSystem, JobSystemError> {
        let n = std::thread::available_parallelism().map_or(1, NonZeroUsize::get);
        let interactive_n = cfg.interactive_threads.map_or(n, NonZeroUsize::get);
        let background_n = cfg
            .background_threads
            .map_or(n.saturating_sub(1).max(1), NonZeroUsize::get);
        // Job panics are caught in `execute`; this only guards against aborts from anything
        // that escapes (e.g. a panicking `Drop` of a discarded result).
        let interactive = ThreadPoolBuilder::new()
            .num_threads(interactive_n)
            .thread_name(|i| format!("amg-interactive-{i}"))
            .panic_handler(|_| {})
            .build()?;
        let background = ThreadPoolBuilder::new()
            .num_threads(background_n)
            .thread_name(|i| format!("amg-background-{i}"))
            .start_handler(|_| {
                let _ = os::lower_current_thread_priority();
            })
            .panic_handler(|_| {})
            .build()?;
        Ok(JobSystem {
            inner: Arc::new(Inner {
                interactive,
                background,
                next_id: AtomicU64::new(1),
                registry: Mutex::new(HashMap::new()),
                registry_cv: Condvar::new(),
                notifier: cfg.notifier,
                shutting_down: AtomicBool::new(false),
            }),
        })
    }

    /// Spawns `job` on the Interactive pool.
    pub fn spawn<J: Job>(&self, job: J) -> JobHandle<J::Output, J::Error> {
        self.spawn_with(job, SpawnOptions::default())
    }

    /// Spawns `job` with explicit options.
    pub fn spawn_with<J: Job>(&self, job: J, opts: SpawnOptions) -> JobHandle<J::Output, J::Error> {
        spawn_job(
            &self.inner,
            job,
            opts.priority,
            opts.hidden,
            None,
            CancelToken::new(),
        )
    }

    /// Closure convenience for [`JobSystem::spawn_with`].
    pub fn spawn_fn<T, E, F>(
        &self,
        label: impl Into<Cow<'static, str>>,
        opts: SpawnOptions,
        f: F,
    ) -> JobHandle<T, E>
    where
        T: Send + 'static,
        E: std::error::Error + From<Cancelled> + Send + Sync + 'static,
        F: FnOnce(&JobCtx) -> Result<T, E> + Send + 'static,
    {
        self.spawn_with(
            FnJob {
                label: label.into(),
                f,
                _out: PhantomData,
            },
            opts,
        )
    }

    /// Snapshot of unfinished, non-hidden jobs, sorted by id, for the job panel (UI-10).
    /// Non-blocking (short lock).
    pub fn jobs(&self) -> Vec<JobInfo> {
        let mut v: Vec<JobInfo> = self
            .inner
            .unfinished()
            .iter()
            .filter(|c| !c.hidden)
            .map(|c| c.info())
            .collect();
        v.sort_by_key(|i| i.id);
        v
    }

    /// Cancels an unfinished job (and, through its token, its children). Returns `false` if no
    /// unfinished job has this id.
    pub fn cancel(&self, id: JobId) -> bool {
        let core = self.inner.registry.lock().get(&id).cloned();
        match core {
            Some(c) => {
                c.cancel();
                true
            }
            None => false,
        }
    }

    /// Cancels every unfinished job. Queued jobs resolve as `Cancelled` immediately.
    pub fn cancel_all(&self) {
        for c in self.inner.unfinished() {
            c.cancel();
        }
    }

    /// Runs `f` inside a pool so its rayon iterators use that pool. Blocking: not on the UI
    /// thread (`debug_assert!`).
    pub fn install<R: Send>(&self, priority: Priority, f: impl FnOnce() -> R + Send) -> R {
        debug_assert!(
            !is_ui_thread(),
            "JobSystem::install called on the UI thread"
        );
        self.inner.pool(priority).install(f)
    }

    /// Cancels everything and waits up to `timeout` for workers to finish. Jobs spawned
    /// afterwards are cancelled immediately.
    pub fn shutdown(&self, timeout: Duration) -> ShutdownReport {
        self.inner.shutting_down.store(true, Ordering::SeqCst);
        let unfinished = self.inner.unfinished();
        let cancelled = unfinished.len();
        for c in &unfinished {
            c.cancel();
        }
        drop(unfinished);
        let deadline = Instant::now() + timeout;
        let mut reg = self.inner.registry.lock();
        while !reg.is_empty() {
            if self
                .inner
                .registry_cv
                .wait_until(&mut reg, deadline)
                .timed_out()
            {
                break;
            }
        }
        let cores: Vec<Arc<JobCore>> = reg.values().cloned().collect();
        drop(reg);
        let mut still_running: Vec<JobInfo> = cores.iter().map(|c| c.info()).collect();
        still_running.sort_by_key(|i| i.id);
        ShutdownReport {
            cancelled,
            still_running,
        }
    }
}

struct FnJob<F, T, E> {
    label: Cow<'static, str>,
    f: F,
    _out: PhantomData<fn() -> (T, E)>,
}

impl<T, E, F> Job for FnJob<F, T, E>
where
    T: Send + 'static,
    E: std::error::Error + From<Cancelled> + Send + Sync + 'static,
    F: FnOnce(&JobCtx) -> Result<T, E> + Send + 'static,
{
    type Output = T;
    type Error = E;
    fn label(&self) -> Cow<'static, str> {
        self.label.clone()
    }
    fn run(self, ctx: &JobCtx) -> Result<T, E> {
        (self.f)(ctx)
    }
}

/// Registers and queues a job on `inner`'s pool for `priority`.
pub(crate) fn spawn_job<J: Job>(
    inner: &Arc<Inner>,
    job: J,
    priority: Priority,
    hidden: bool,
    parent: Option<JobId>,
    token: CancelToken,
) -> JobHandle<J::Output, J::Error> {
    let id = JobId(inner.next_id.fetch_add(1, Ordering::Relaxed));
    let core = Arc::new(JobCore::new(
        id,
        job.label(),
        priority,
        parent,
        hidden,
        token,
        inner.notifier.clone(),
        Arc::downgrade(inner),
    ));
    let (tx, rx) = crossbeam_channel::bounded(1);
    let handle = JobHandle::new(core.clone(), rx);
    inner.registry.lock().insert(id, core.clone());
    if inner.shutting_down.load(Ordering::SeqCst) {
        core.token.cancel();
    }
    if core.token.is_cancelled() {
        core.cancel_if_queued();
        return handle;
    }
    let ctx = JobCtx::for_job(core.clone(), Some(inner.clone()));
    inner.pool(priority).spawn(move || execute(job, ctx, tx));
    core.notify();
    handle
}

/// Runs a job on the current thread: start transition, `catch_unwind`, classification, result
/// delivery, then the terminal state.
pub(crate) fn execute<J: Job>(job: J, ctx: JobCtx, tx: Sender<JobResult<J::Output, J::Error>>) {
    let Some(core) = ctx.core.clone() else {
        return;
    };
    if core.token.is_cancelled() {
        core.cancel_if_queued();
        return;
    }
    if !core.try_start() {
        return; // cancelled out of the queue
    }
    let outcome = crate::diag::catch_panic(AssertUnwindSafe(|| job.run(&ctx)));
    drop(ctx);
    let (result, code, detail) = match outcome {
        Ok(Ok(v)) => (Ok(v), SUCCEEDED, None),
        Ok(Err(_)) if core.token.is_cancelled() => (Err(JobError::Cancelled), CANCELLED, None),
        Ok(Err(e)) => {
            let msg = e.to_string();
            (Err(JobError::Failed(e)), FAILED, Some(msg))
        }
        Err(payload) => {
            let msg = panic_message(payload.as_ref());
            (Err(JobError::Panicked(msg.clone())), PANICKED, Some(msg))
        }
    };
    // Send before publishing the terminal state: `try_take` relies on this order.
    let _ = tx.send(result);
    core.finish(code, detail);
}

fn panic_message(payload: &(dyn std::any::Any + Send)) -> String {
    if let Some(s) = payload.downcast_ref::<&'static str>() {
        (*s).to_owned()
    } else if let Some(s) = payload.downcast_ref::<String>() {
        s.clone()
    } else {
        "non-string panic payload".to_owned()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn background_threads_run_below_normal_priority() {
        let js = JobSystem::new(JobSystemConfig {
            interactive_threads: NonZeroUsize::new(2),
            background_threads: NonZeroUsize::new(2),
            notifier: None,
        })
        .unwrap();
        let bg = js.install(Priority::Background, os::current_thread_priority);
        let fg = js.install(Priority::Interactive, os::current_thread_priority);
        if cfg!(windows) {
            assert_eq!(bg, Some(os::BELOW_NORMAL));
            assert_eq!(fg, Some(0)); // THREAD_PRIORITY_NORMAL
        }
        let h = js.spawn_fn(
            "prio",
            SpawnOptions {
                priority: Priority::Background,
                hidden: false,
            },
            |_| Ok::<_, Cancelled>(os::current_thread_priority()),
        );
        assert_eq!(h.wait_blocking().unwrap(), bg);
    }

    #[test]
    fn panic_message_handles_payload_kinds() {
        assert_eq!(panic_message(&"a"), "a");
        assert_eq!(panic_message(&String::from("b")), "b");
        assert_eq!(panic_message(&5u32), "non-string panic payload");
    }
}
