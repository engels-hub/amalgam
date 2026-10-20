//! Job system: CPU pools, cancellable jobs with progress, the shared GPU device and diagnostics.
//!
//! Spec: S00 §4. Lane: E. This crate currently holds the CPU job system (FND-04); the GPU
//! (`gpu`, FND-09) and diagnostics (`diag`, FND-07) modules land later.
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

// TODO(FND-03): replace this definition with `pub use amg_core::Cancelled;` once FND-03 lands.
// It is declared exactly as in the S00 interface, so the swap is source-compatible.
/// Shared cancellation marker; every crate's error implements `From<Cancelled>`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
#[error("operation cancelled")]
pub struct Cancelled;

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

// TODO(FND-03): add `impl<E: amg_core::Diagnostic> amg_core::Diagnostic for JobError<E>`
// (delegating to `E` for `Failed`, `is_cancelled() == true` for `Cancelled`, an "internal error"
// message for `Panicked`) once `amg_core::Diagnostic` exists.
