//! Job system: CPU pools, cancellable jobs with progress, the shared GPU device and diagnostics.
//!
//! Spec: S00 §4. Lane: E. This crate holds the CPU job system (FND-04) and diagnostics
//! ([`diag`], FND-07); the GPU module (`gpu`, FND-09) lands later.
//!
//! # Model
//! - A [`JobSystem`] owns two rayon pools: [`Priority::Interactive`] (recompute, previews) and
//!   [`Priority::Background`] (studies, big meshing; below-normal OS thread priority), so
//!   interactive work never queues behind a long study.
//! - Work implements [`Job`] (or is a closure passed to [`JobSystem::spawn_fn`]). It receives a
//!   [`JobCtx`] for cancellation checkpoints, progress and child jobs, and returns
//!   `Result<Output, Error>` where `Error: From<Cancelled>`, so `ctx.checkpoint()?` works.
//! - The spawner gets a [`JobHandle`]. [`JobHandle::try_take`], [`JobHandle::state`] and
//!   [`JobHandle::progress`] never block; the UI polls them when the configured notifier fires.
//! - Panics inside a job are caught and reported as [`JobError::Panicked`].
//!
//! ```
//! use amg_jobs::{Cancelled, JobSystem, JobSystemConfig, SpawnOptions};
//!
//! let jobs = JobSystem::new(JobSystemConfig::default()).unwrap();
//! let handle = jobs.spawn_fn("sum", SpawnOptions::default(), |ctx| {
//!     let mut sum = 0u64;
//!     for i in 0..1000u64 {
//!         if i % 100 == 0 {
//!             ctx.checkpoint()?;
//!             ctx.progress(i as f32 / 1000.0);
//!         }
//!         sum += i;
//!     }
//!     Ok::<_, Cancelled>(sum)
//! });
//! assert_eq!(handle.wait_blocking().unwrap(), 499_500);
//! ```

#![warn(missing_docs)]

mod cancel;
mod core;
mod ctx;
pub mod diag;
mod handle;
mod os;
mod system;
mod ui;

use std::borrow::Cow;

pub use cancel::CancelToken;
pub use ctx::JobCtx;
pub use handle::JobHandle;
pub use system::{JobSystem, JobSystemConfig, JobSystemError, ShutdownReport, SpawnOptions};
pub use ui::{is_ui_thread, mark_ui_thread};

pub use amg_core::Cancelled;

/// Identifies a job. Monotonically increasing per [`JobSystem`], starting at 1.
///
/// Children spawned from a detached [`JobCtx`] (no job system) get ids from a separate
/// process-wide counter starting at `1 << 63`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct JobId(pub u64);

/// Which pool a job runs on.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Priority {
    /// Recompute, previews and picking-adjacent work. Normal OS priority.
    #[default]
    Interactive,
    /// Studies, large remeshes, lattice meshing. Below-normal OS priority.
    Background,
}

/// A unit of work run on a [`JobSystem`] pool.
///
/// `run` consumes the job. It should call [`JobCtx::checkpoint`] at least every ~10 ms of work
/// and may use rayon parallel iterators, which execute on the job's own pool.
pub trait Job: Send + 'static {
    /// Value produced on success.
    type Output: Send + 'static;
    /// Error type. `From<Cancelled>` lets `ctx.checkpoint()?` propagate cancellation.
    type Error: std::error::Error + From<Cancelled> + Send + Sync + 'static;
    /// Human-readable label shown in the job panel. Called once, at spawn.
    fn label(&self) -> Cow<'static, str>;
    /// Does the work.
    fn run(self, ctx: &JobCtx) -> Result<Self::Output, Self::Error>;
}

/// Snapshot of a job's progress.
#[derive(Clone, Debug, PartialEq)]
pub struct Progress {
    /// Fraction of the whole job in `[0, 1]`; monotone.
    pub fraction: f32,
    /// Latest progress message, if any (rate-limited to 30 Hz).
    pub message: Option<std::sync::Arc<str>>,
}

/// Lifecycle state of a job: `Queued → Running → Succeeded | Failed | Cancelled | Panicked`.
#[derive(Clone, Debug, PartialEq)]
pub enum JobState {
    /// Waiting for a pool thread.
    Queued,
    /// Executing, with its current progress.
    Running(Progress),
    /// Finished with `Ok`.
    Succeeded,
    /// Finished with `Err`; the error's `Display` text.
    Failed(String),
    /// Cancelled before it started, or returned `Err` after its token was cancelled.
    Cancelled,
    /// The job panicked; the panic message.
    Panicked(String),
}

impl JobState {
    /// True for `Succeeded`, `Failed`, `Cancelled` and `Panicked`.
    pub fn is_finished(&self) -> bool {
        !matches!(self, JobState::Queued | JobState::Running(_))
    }
}

/// Snapshot of one job for the job panel (UI-10). See [`JobSystem::jobs`].
#[derive(Clone, Debug)]
pub struct JobInfo {
    /// The job's id.
    pub id: JobId,
    /// The label from [`Job::label`].
    pub label: Cow<'static, str>,
    /// Pool the job runs on.
    pub priority: Priority,
    /// The spawning job, for jobs created with [`JobCtx::spawn_child`].
    pub parent: Option<JobId>,
    /// Current state.
    pub state: JobState,
    /// When the job was spawned.
    pub created: std::time::Instant,
    /// When a pool thread picked it up (always `Some` once `Running`).
    pub started: Option<std::time::Instant>,
}

/// Failure of a job as seen by the spawner.
#[derive(Debug, thiserror::Error)]
pub enum JobError<E> {
    /// The job was cancelled (before starting, or it returned `Err` after cancellation).
    #[error("cancelled")]
    Cancelled,
    /// The job panicked; the panic message.
    #[error("internal error (panic): {0}")]
    Panicked(String),
    /// The job returned this error.
    #[error(transparent)]
    Failed(E),
}

/// `Failed` delegates to `E`; `Cancelled` is an `Info` that reports `is_cancelled()`; `Panicked`
/// is an internal error whose panic text stays in `Display` (for logs), not in the user message.
impl<E: amg_core::Diagnostic> amg_core::Diagnostic for JobError<E> {
    fn user_message(&self) -> String {
        match self {
            JobError::Cancelled => "The operation was cancelled.".to_owned(),
            JobError::Panicked(_) => "An internal error occurred.".to_owned(),
            JobError::Failed(e) => e.user_message(),
        }
    }

    fn hint(&self) -> Option<String> {
        match self {
            JobError::Cancelled => None,
            JobError::Panicked(_) => {
                Some("This is a bug in Amalgam; the log has the details.".to_owned())
            }
            JobError::Failed(e) => e.hint(),
        }
    }

    fn severity(&self) -> amg_core::Severity {
        match self {
            JobError::Cancelled => amg_core::Severity::Info,
            JobError::Panicked(_) => amg_core::Severity::Error,
            JobError::Failed(e) => e.severity(),
        }
    }

    fn is_cancelled(&self) -> bool {
        match self {
            JobError::Cancelled => true,
            JobError::Panicked(_) => false,
            JobError::Failed(e) => e.is_cancelled(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use amg_core::{Diagnostic, Severity};

    #[derive(Debug, thiserror::Error)]
    #[error("profile open")]
    struct Open;
    impl Diagnostic for Open {
        fn hint(&self) -> Option<String> {
            Some("close it".into())
        }
        fn severity(&self) -> Severity {
            Severity::Warning
        }
    }

    #[test]
    fn job_error_diagnostic_delegates_and_classifies() {
        let f: JobError<Open> = JobError::Failed(Open);
        assert_eq!(f.user_message(), "profile open");
        assert_eq!(f.hint().as_deref(), Some("close it"));
        assert_eq!(f.severity(), Severity::Warning);
        assert!(!f.is_cancelled());

        let c: JobError<Open> = JobError::Cancelled;
        assert!(c.is_cancelled());
        assert_eq!(c.severity(), Severity::Info);
        assert_eq!(c.hint(), None);

        let p: JobError<Open> = JobError::Panicked("index out of bounds".into());
        assert!(!p.is_cancelled());
        assert_eq!(p.severity(), Severity::Error);
        assert!(!p.user_message().contains("index"));
        assert!(p.hint().is_some());
        assert!(amg_core::report(&p).contains("index out of bounds"));

        // A failure that is itself a cancellation stays a cancellation.
        let fc: JobError<amg_core::Error> = JobError::Failed(amg_core::Error::Cancelled(Cancelled));
        assert!(fc.is_cancelled());
    }
}
