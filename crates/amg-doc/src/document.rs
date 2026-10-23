//! The [`Document`]: authored state with undo ([`History`]), the evaluated state, the geometry
//! cache and the recompute jobs (S01 §3 "Job integration", DOC-12).
//!
//! # Recompute lifecycle
//! The UI calls [`Document::poll`] once per frame. It never blocks:
//! 1. A finished job is collected. Its result is swapped in (one `Arc` store) only if its
//!    revision is still the current one; otherwise it is dropped.
//! 2. A running job for an older revision is cancelled (cooperatively; outputs of features
//!    that finished stay in the [`GeomCache`] and are reused by the next job).
//! 3. If the evaluated state is older than the authored state and no job runs, a
//!    [`RecomputeJob`] is spawned on the interactive pool against an `Arc` snapshot.
//!
//! "Revision" here is [`History::revision`], which also moves on undo and redo (unlike
//! `Design::revision`). Headless callers use [`Document::recompute_blocking`]. Previews
//! ([`Document::preview`]) run in [`ComputeMode::Preview`] against a scratch design and a
//! cache of their own, so they never reach the persistent cache or the undo stack.

use std::borrow::Cow;
use std::sync::Arc;

use amg_core::Cancelled;
use amg_jobs::{Job, JobCtx, JobError, JobHandle, JobSystem, SpawnOptions, is_ui_thread};

use crate::cache::GeomCache;
use crate::command::{Command, CommandError, CommandOutcome, apply_command};
use crate::eval::Evaluated;
use crate::event::{DocEvent, EventState};
use crate::feature::ComputeMode;
use crate::history::{History, Transaction};
use crate::id::{FeatureId, Revision};
use crate::model::{Design, DesignSnapshot};
use crate::recompute::{RecomputeResult, recompute};

/// Default budget of the persistent geometry cache (S01 §3: 2 GiB).
pub const DEFAULT_CACHE_BUDGET: u64 = 2 << 30;
/// Budget of the preview cache.
const PREVIEW_CACHE_BUDGET: u64 = 256 << 20;

/// Why a recompute job produced no result.
#[derive(Debug, thiserror::Error)]
pub enum RecomputeError {
    /// The job was cancelled (superseded, or the job system shut down).
    #[error("recompute was cancelled")]
    Cancelled,
    /// The job panicked (a bug in a feature); the panic message.
    #[error("recompute failed with an internal error: {0}")]
    Panicked(String),
}

impl From<Cancelled> for RecomputeError {
    fn from(_: Cancelled) -> RecomputeError {
        RecomputeError::Cancelled
    }
}

impl From<JobError<RecomputeError>> for RecomputeError {
    fn from(e: JobError<RecomputeError>) -> RecomputeError {
        match e {
            JobError::Cancelled => RecomputeError::Cancelled,
            JobError::Panicked(m) => RecomputeError::Panicked(m),
            JobError::Failed(e) => e,
        }
    }
}

/// Where the evaluated state stands relative to the authored state.
#[derive(Clone, Debug, PartialEq)]
pub enum RecomputeStatus {
    /// `evaluated()` matches the authored state.
    Idle,
    /// A job for the current revision is running.
    Running {
        /// Fraction of the whole recompute in `[0, 1]`.
        progress: f32,
        /// Name of the feature being computed (and whatever it reports itself).
        message: Option<Arc<str>>,
    },
    /// The authored state changed and no job for it runs yet (the next `poll` starts one).
    Stale,
    /// The job for the current revision failed (panic or external cancellation); `poll` does
    /// not retry until the state changes. The message is for logs.
    Failed(String),
}

/// Which features the last swapped-in recompute computed and which it took from the cache.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct RecomputeSummary {
    /// Features whose `compute` ran, in timeline order.
    pub computed: Vec<FeatureId>,
    /// Features served from the cache.
    pub cached: Vec<FeatureId>,
}

/// The recompute work: runs the engine over an immutable snapshot.
#[derive(Debug)]
pub struct RecomputeJob {
    snapshot: DesignSnapshot,
    revision: Revision,
    cache: Arc<GeomCache>,
    mode: ComputeMode,
}

impl RecomputeJob {
    /// The revision (of [`History::revision`]) this job computes.
    pub fn revision(&self) -> Revision {
        self.revision
    }
}

impl Job for RecomputeJob {
    type Output = RecomputeResult;
    type Error = RecomputeError;

    fn label(&self) -> Cow<'static, str> {
        match self.mode {
            ComputeMode::Full => Cow::Borrowed("Recompute"),
            ComputeMode::Preview => Cow::Borrowed("Preview"),
        }
    }

    fn run(self, ctx: &JobCtx) -> Result<RecomputeResult, RecomputeError> {
        Ok(recompute(&self.snapshot, &self.cache, self.mode, ctx)?)
    }
}

struct Running {
    handle: JobHandle<RecomputeResult, RecomputeError>,
    revision: Revision,
}

/// A design with its undo history, evaluated state, geometry cache and recompute jobs.
/// Owned by the UI thread.
pub struct Document {
    history: History,
    cache: Arc<GeomCache>,
    preview_cache: Arc<GeomCache>,
    evaluated: Arc<Evaluated>,
    /// History revision `evaluated` was computed for.
    evaluated_for: Option<Revision>,
    running: Option<Running>,
    failed: Option<(Revision, String)>,
    last_run: Option<RecomputeSummary>,
    events: EventState,
}

impl std::fmt::Debug for Document {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Document")
            .field("revision", &self.history.revision())
            .field("evaluated_for", &self.evaluated_for)
            .field("running", &self.running.as_ref().map(|r| r.revision))
            .finish_non_exhaustive()
    }
}

impl Drop for Document {
    fn drop(&mut self) {
        if let Some(r) = &self.running {
            r.handle.cancel();
        }
    }
}

impl Document {
    /// An empty document.
    pub fn new(name: &str) -> Document {
        Document::from_design(Design::new(name))
    }

    /// A document around `design`; nothing is evaluated yet (`recompute_status` is `Stale`).
    pub fn from_design(design: Design) -> Document {
        Document::with_cache(design, GeomCache::new(DEFAULT_CACHE_BUDGET))
    }

    /// Like [`Document::from_design`] with a given (e.g. seeded) cache.
    pub fn with_cache(design: Design, cache: Arc<GeomCache>) -> Document {
        let evaluated = Arc::new(Evaluated::new(design.revision()));
        let history = History::new(design);
        let events = EventState::new(history.snapshot(), Arc::clone(&evaluated));
        Document {
            history,
            cache,
            preview_cache: GeomCache::new(PREVIEW_CACHE_BUDGET),
            evaluated,
            evaluated_for: None,
            running: None,
            failed: None,
            last_run: None,
            events,
        }
    }

    // ---- change events (DOC-09) -----------------------------------------------------------

    /// Takes the change events since the last call (once per UI frame). Authored changes are
    /// diffed against what the previous call saw, so several edits between two calls yield
    /// their net effect. Events carry ids only; read the state from `design()`/`evaluated()`.
    pub fn drain_events(&mut self) -> Vec<DocEvent> {
        self.events.sync_authored(&self.history.snapshot());
        self.events.take()
    }

    // ---- authored state (delegates to History) ----------------------------------------

    /// The current authored state, O(1).
    pub fn snapshot(&self) -> DesignSnapshot {
        self.history.snapshot()
    }

    /// The current design.
    pub fn design(&self) -> &Design {
        self.history.design()
    }

    /// The undo history.
    pub fn history(&self) -> &History {
        &self.history
    }

    /// Mutable access to the history (depth limit, saved marker, clearing).
    pub fn history_mut(&mut self) -> &mut History {
        &mut self.history
    }

    /// Applies one command as one undo entry. The next `poll` starts the recompute.
    pub fn apply(&mut self, cmd: Command) -> Result<CommandOutcome, CommandError> {
        self.history.apply(cmd)
    }

    /// Starts a transaction (several commands, one undo entry).
    pub fn transaction(&mut self, label: &str) -> Transaction<'_> {
        self.history.transaction(label)
    }

    /// Steps back. Returns false if there is nothing to undo.
    pub fn undo(&mut self) -> bool {
        self.history.undo()
    }

    /// Steps forward. Returns false if there is nothing to redo.
    pub fn redo(&mut self) -> bool {
        self.history.redo()
    }

    /// Label of the step `undo` would revert.
    pub fn undo_label(&self) -> Option<&str> {
        self.history.undo_label()
    }

    /// Label of the step `redo` would reapply.
    pub fn redo_label(&self) -> Option<&str> {
        self.history.redo_label()
    }

    /// True if the design differs from the last saved state.
    pub fn is_dirty(&self) -> bool {
        self.history.is_dirty()
    }

    // ---- evaluated state ----------------------------------------------------------------

    /// The latest evaluated state. May lag the authored state while a recompute runs; its
    /// `revision` says which `Design::revision` it belongs to.
    pub fn evaluated(&self) -> &Arc<Evaluated> {
        &self.evaluated
    }

    /// The persistent geometry cache (what DOC-14 saves).
    pub fn cache(&self) -> &Arc<GeomCache> {
        &self.cache
    }

    /// The scratch cache previews use; never persisted.
    pub fn preview_cache(&self) -> &Arc<GeomCache> {
        &self.preview_cache
    }

    /// What the last swapped-in recompute computed and what it took from the cache.
    pub fn last_run(&self) -> Option<&RecomputeSummary> {
        self.last_run.as_ref()
    }

    /// True if `evaluated()` is for the current authored state.
    pub fn is_evaluated(&self) -> bool {
        self.evaluated_for == Some(self.history.revision())
    }

    /// Progress and staleness of the recompute, for the timeline's progress ring.
    pub fn recompute_status(&self) -> RecomputeStatus {
        let cur = self.history.revision();
        if let Some(r) = self.running.as_ref().filter(|r| r.revision == cur) {
            let p = r.handle.progress();
            return RecomputeStatus::Running {
                progress: p.fraction,
                message: p.message,
            };
        }
        if self.evaluated_for == Some(cur) {
            RecomputeStatus::Idle
        } else if let Some((_, m)) = self.failed.as_ref().filter(|(r, _)| *r == cur) {
            RecomputeStatus::Failed(m.clone())
        } else {
            RecomputeStatus::Stale
        }
    }

    // ---- recompute ----------------------------------------------------------------------

    /// Call once per UI frame. Never blocks: collects a finished recompute, cancels one made
    /// stale by newer edits, and starts the one the current state needs.
    pub fn poll(&mut self, jobs: &JobSystem) {
        self.collect();
        let cur = self.history.revision();
        self.cancel_if_stale(cur);
        if self.running.is_none() && self.needs_recompute(cur) {
            self.spawn(jobs, cur);
        }
    }

    /// Recomputes until `evaluated()` matches the authored state, waiting for the job.
    /// For the CLI, Python and tests.
    ///
    /// # Panics
    /// In debug builds on the UI thread (`amg_jobs::mark_ui_thread`): use [`Document::poll`].
    pub fn recompute_blocking(&mut self, jobs: &JobSystem) -> Result<(), RecomputeError> {
        debug_assert!(
            !is_ui_thread(),
            "Document::recompute_blocking called on the UI thread"
        );
        self.failed = None;
        let cur = self.history.revision();
        self.cancel_if_stale(cur);
        if self.running.is_none() {
            if !self.needs_recompute(cur) {
                return Ok(());
            }
            self.spawn(jobs, cur);
        }
        let Some(r) = self.running.take() else {
            return Ok(());
        };
        match r.handle.wait_blocking() {
            Ok(result) => {
                self.swap_in(result, r.revision);
                Ok(())
            }
            Err(e) => {
                let e = RecomputeError::from(e);
                self.failed = Some((r.revision, e.to_string()));
                Err(e)
            }
        }
    }

    /// Computes what the document would look like after `cmd`, in [`ComputeMode::Preview`]
    /// against a scratch design and the preview cache. The document, its undo stack and the
    /// persistent cache are untouched. Cancel the handle when the preview is superseded.
    pub fn preview(
        &self,
        jobs: &JobSystem,
        cmd: &Command,
    ) -> Result<JobHandle<Arc<Evaluated>, RecomputeError>, CommandError> {
        let mut work = (*self.history.snapshot()).clone();
        apply_command(&mut work, cmd.clone())?;
        let cache = Arc::clone(&self.preview_cache);
        Ok(
            jobs.spawn_fn("Preview", SpawnOptions::default(), move |ctx: &JobCtx| {
                let r = recompute(&work, &cache, ComputeMode::Preview, ctx)?;
                Ok::<_, RecomputeError>(Arc::new(r.evaluated))
            }),
        )
    }

    fn cancel_if_stale(&mut self, cur: Revision) {
        if self.running.as_ref().is_some_and(|r| r.revision != cur)
            && let Some(r) = self.running.take()
        {
            r.handle.cancel();
        }
    }

    fn needs_recompute(&self, cur: Revision) -> bool {
        self.evaluated_for != Some(cur) && self.failed.as_ref().map(|f| f.0) != Some(cur)
    }

    fn spawn(&mut self, jobs: &JobSystem, revision: Revision) {
        self.events.sync_authored(&self.history.snapshot());
        self.events.recompute_started(revision);
        let job = RecomputeJob {
            snapshot: self.history.snapshot(),
            revision,
            cache: Arc::clone(&self.cache),
            mode: ComputeMode::Full,
        };
        self.running = Some(Running {
            handle: jobs.spawn(job),
            revision,
        });
    }

    /// Takes a finished job's result: swap it in if current, drop it if stale.
    fn collect(&mut self) {
        let Some(r) = self.running.as_mut() else {
            return;
        };
        let Some(result) = r.handle.try_take() else {
            return;
        };
        let revision = r.revision;
        self.running = None;
        match result {
            Ok(res) if revision == self.history.revision() => self.swap_in(res, revision),
            // Stale: newer edits exist; the next job reuses what this one cached.
            Ok(_) => {}
            Err(e) => {
                // Cancelled by someone else (shutdown) or panicked: don't respawn every frame.
                let e = RecomputeError::from(e);
                self.failed = Some((revision, e.to_string()));
            }
        }
    }

    fn swap_in(&mut self, result: RecomputeResult, revision: Revision) {
        self.evaluated = Arc::new(result.evaluated);
        self.events.sync_authored(&self.history.snapshot());
        self.events.swap_evaluated(&self.evaluated, revision);
        self.evaluated_for = Some(revision);
        self.failed = None;
        self.last_run = Some(RecomputeSummary {
            computed: result.computed,
            cached: result.cached,
        });
    }
}
