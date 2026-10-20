//! The spawner's handle to a job.

use std::{fmt, sync::Arc, time::Duration};

use crossbeam_channel::Receiver;

use crate::{CancelToken, JobError, JobId, JobState, Progress, core::JobCore, ui::is_ui_thread};

pub(crate) type JobResult<T, E> = Result<T, JobError<E>>;

/// Owned by the spawner. Dropping it detaches (job keeps running, result discarded);
/// call [`JobHandle::cancel`] first to stop it. All methods except
/// [`JobHandle::wait_blocking`] are non-blocking (atomics plus a channel `try_recv`).
pub struct JobHandle<T, E> {
    pub(crate) core: Arc<JobCore>,
    rx: Receiver<JobResult<T, E>>,
    taken: bool,
}

impl<T, E> fmt::Debug for JobHandle<T, E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("JobHandle")
            .field("id", &self.core.id)
            .field("label", &self.core.label)
            .field("state", &self.core.state())
            .field("taken", &self.taken)
            .finish()
    }
}

impl<T: Send + 'static, E: Send + 'static> JobHandle<T, E> {
    pub(crate) fn new(core: Arc<JobCore>, rx: Receiver<JobResult<T, E>>) -> Self {
        JobHandle {
            core,
            rx,
            taken: false,
        }
    }

    /// The job's id.
    pub fn id(&self) -> JobId {
        self.core.id
    }

    /// Current state. Non-blocking.
    pub fn state(&self) -> JobState {
        self.core.state()
    }

    /// Current progress (fraction of the whole job, latest message). Non-blocking.
    pub fn progress(&self) -> Progress {
        self.core.progress()
    }

    /// True once the job reached a terminal state (its result can be taken).
    pub fn is_finished(&self) -> bool {
        self.core.is_finished()
    }

    /// `Some` exactly once, when finished; `None` before that and after the result was taken.
    /// Non-blocking.
    pub fn try_take(&mut self) -> Option<Result<T, JobError<E>>> {
        if self.taken || !self.core.is_finished() {
            return None;
        }
        self.taken = true;
        // The executor sends the result before publishing the terminal state, so after the
        // Acquire load above it is visible. No result means the job was cancelled before it
        // started (or its closure was dropped unrun), which is a cancellation.
        Some(self.rx.try_recv().unwrap_or(Err(JobError::Cancelled)))
    }

    /// Requests cancellation. A queued job finishes as `Cancelled` immediately; a running job
    /// stops at its next checkpoint. Children are cancelled through the token hierarchy.
    pub fn cancel(&self) {
        self.core.cancel();
    }

    /// The job's cancellation token.
    pub fn cancel_token(&self) -> CancelToken {
        self.core.token.clone()
    }

    /// Blocks the calling thread until the job finishes. `debug_assert!(!is_ui_thread())`.
    /// For the CLI and tests; inside a job use [`crate::JobCtx::wait`] instead.
    ///
    /// # Panics
    /// If the result was already taken with [`JobHandle::try_take`].
    pub fn wait_blocking(mut self) -> Result<T, JobError<E>> {
        debug_assert!(
            !is_ui_thread(),
            "JobHandle::wait_blocking called on the UI thread"
        );
        self.take_waiting(|core| core.wait_done(None))
    }

    /// Loops `try_take`, calling `block` while the job is unfinished.
    pub(crate) fn take_waiting(
        &mut self,
        mut block: impl FnMut(&JobCore),
    ) -> Result<T, JobError<E>> {
        assert!(!self.taken, "JobHandle result was already taken");
        loop {
            if let Some(r) = self.try_take() {
                return r;
            }
            block(&self.core);
        }
    }
}

/// Idle-wait slice for `JobCtx::wait` when there is no local work to steal.
pub(crate) const IDLE_WAIT: Duration = Duration::from_micros(200);
