//! The context passed to running work.

use std::{
    fmt,
    sync::{
        Arc, Weak,
        atomic::{AtomicU64, Ordering},
    },
};

use crate::{
    CancelToken, Cancelled, Job, JobError, JobHandle, JobId,
    core::{JobCore, unit},
    handle::IDLE_WAIT,
    system::{self, Inner},
    ui::is_ui_thread,
};

/// Ids for children of detached contexts; disjoint from per-system ids.
static DETACHED_IDS: AtomicU64 = AtomicU64::new(1 << 63);

/// Passed to running work. `Send + Sync + Clone`: may be shared into rayon `par_iter` closures.
///
/// Each context covers a sub-range `[lo, hi]` of the root job's progress `[0, 1]`;
/// [`JobCtx::sub`] narrows it so nested steps report progress without knowing about each
/// other.
#[derive(Clone)]
pub struct JobCtx {
    pub(crate) core: Option<Arc<JobCore>>,
    pub(crate) system: Option<Arc<Inner>>,
    pub(crate) token: CancelToken,
    lo: f32,
    hi: f32,
}

impl fmt::Debug for JobCtx {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("JobCtx")
            .field("id", &self.id())
            .field("range", &(self.lo, self.hi))
            .field("cancelled", &self.is_cancelled())
            .finish()
    }
}

impl JobCtx {
    pub(crate) fn for_job(core: Arc<JobCore>, system: Option<Arc<Inner>>) -> JobCtx {
        let token = core.token.clone();
        JobCtx {
            core: Some(core),
            system,
            token,
            lo: 0.0,
            hi: 1.0,
        }
    }

    /// For synchronous callers (tests, CLI, direct calls): never cancelled, progress discarded.
    /// Children spawned from it run inline on the calling thread.
    pub fn detached() -> JobCtx {
        JobCtx::with_token(CancelToken::new())
    }

    /// Like [`JobCtx::detached`] but cancellable via `token` (tests of cancel paths).
    pub fn with_token(token: CancelToken) -> JobCtx {
        JobCtx {
            core: None,
            system: None,
            token,
            lo: 0.0,
            hi: 1.0,
        }
    }

    /// The running job's id; `None` for detached contexts.
    pub fn id(&self) -> Option<JobId> {
        self.core.as_ref().map(|c| c.id)
    }

    /// True once this job (or an ancestor) was cancelled.
    pub fn is_cancelled(&self) -> bool {
        self.token.is_cancelled()
    }

    /// `Err(Cancelled)` once cancelled. Call at least every ~10 ms of work.
    pub fn checkpoint(&self) -> Result<(), Cancelled> {
        if self.is_cancelled() {
            Err(Cancelled)
        } else {
            Ok(())
        }
    }

    /// Reports progress. `fraction` in `[0, 1]` is relative to this context's range; it is
    /// clamped (NaN counts as 0) and the job's overall fraction never decreases. One atomic op.
    pub fn progress(&self, fraction: f32) {
        if let Some(core) = &self.core {
            core.raise_progress(self.map(fraction));
        }
    }

    /// Like [`JobCtx::progress`], also setting the progress message. Message updates are
    /// rate-limited to 30 Hz per job; updates arriving faster are dropped.
    pub fn progress_msg(&self, fraction: f32, message: &str) {
        if let Some(core) = &self.core {
            core.raise_progress(self.map(fraction));
            core.set_message(message);
        }
    }

    /// Sub-range `[start, end]` (relative to this context's range, clamped to `[0, 1]`; `end`
    /// below `start` is raised to `start`) with the same cancellation.
    pub fn sub(&self, start: f32, end: f32) -> JobCtx {
        let (s, e) = (unit(start), unit(end));
        let e = e.max(s);
        JobCtx {
            core: self.core.clone(),
            system: self.system.clone(),
            token: self.token.clone(),
            lo: self.map(s),
            hi: self.map(e),
        }
    }

    fn map(&self, f: f32) -> f32 {
        self.lo + unit(f) * (self.hi - self.lo)
    }

    /// Spawns a child job on this job's pool. The child's cancel token is a child of this
    /// context's token, and its [`crate::JobInfo::parent`] is this job.
    ///
    /// On a detached context the child runs inline, synchronously, before this returns.
    pub fn spawn_child<J: Job>(&self, job: J) -> JobHandle<J::Output, J::Error> {
        match (&self.system, &self.core) {
            (Some(sys), Some(core)) => system::spawn_job(
                sys,
                job,
                core.priority,
                core.hidden,
                Some(core.id),
                self.token.child(),
            ),
            _ => {
                let id = JobId(DETACHED_IDS.fetch_add(1, Ordering::Relaxed));
                let core = Arc::new(JobCore::new(
                    id,
                    job.label(),
                    self.core
                        .as_ref()
                        .map_or(Default::default(), |c| c.priority),
                    self.id(),
                    false,
                    self.token.child(),
                    None,
                    Weak::new(),
                ));
                let (tx, rx) = crossbeam_channel::bounded(1);
                let handle = JobHandle::new(core.clone(), rx);
                system::execute(job, JobCtx::for_job(core, None), tx);
                handle
            }
        }
    }

    /// Waits for a child by work-stealing (`rayon::yield_now`): while the child is unfinished
    /// this thread runs other pending jobs of its pool, so a 1-thread pool cannot deadlock.
    /// Off a pool thread it blocks. Panics in debug on the UI thread.
    ///
    /// # Panics
    /// If the handle's result was already taken with [`JobHandle::try_take`].
    pub fn wait<T, E>(&self, handle: JobHandle<T, E>) -> Result<T, JobError<E>>
    where
        T: Send + 'static,
        E: Send + 'static,
    {
        debug_assert!(!is_ui_thread(), "JobCtx::wait called on the UI thread");
        let mut handle = handle;
        handle.take_waiting(|core| match rayon::yield_now() {
            Some(rayon::Yield::Executed) => {}
            Some(rayon::Yield::Idle) => core.wait_done(Some(IDLE_WAIT)),
            None => core.wait_done(None),
        })
    }

    /// This context's cancellation token (shared with the job and its sub-contexts).
    pub fn cancel_token(&self) -> CancelToken {
        self.token.clone()
    }
}
