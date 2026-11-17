//! Live previews (spec §6 "Previews", Interfaces §A `amg_ui::preview`).
//!
//! [`PreviewDriver`] wraps `Document::preview` (S01 §6, DOC-12) with a debounce, latest-wins
//! cancellation and the hand-off to the scene adapter's preview layer. It never touches the
//! undo stack: a preview recomputes a scratch copy of the design.
//!
//! [`PreviewDriver::set`] only records the command and a deadline. The app calls
//! [`PreviewDriver::update`] once per frame: after the debounce it cancels the preview in
//! flight and spawns the newest one; a finished preview becomes [`PreviewState::Ready`] and is
//! shown on the preview layer.

use std::sync::Arc;
use std::time::{Duration, Instant};

use amg_doc::{Command as DocCommand, Document, Evaluated, RecomputeError};
use amg_jobs::{JobError, JobHandle, JobSystem};

/// Default debounce (spec: preferences, default 150 ms; UI-11 makes it a preference).
pub const DEFAULT_PREVIEW_DEBOUNCE: Duration = Duration::from_millis(150);

/// Where the preview is.
#[derive(Clone, Debug)]
pub enum PreviewState {
    /// Nothing to preview.
    Idle,
    /// A command waits for the debounce.
    Pending,
    /// The preview job runs.
    Computing,
    /// The preview of the newest command.
    Ready(Arc<Evaluated>),
    /// The newest command failed (message for the dialog banner).
    Failed(String),
}

/// What [`PreviewDriver::update`] did.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct PreviewUpdate {
    /// The shown preview changed (`Ready` arrived or the preview was cleared).
    pub changed: bool,
    /// Wake the UI after this long (the debounce deadline).
    pub wake_after: Option<Duration>,
}

/// Debounced, latest-wins previews for tools and dialogs.
pub struct PreviewDriver {
    debounce: Duration,
    pending: Option<(DocCommand, Instant)>,
    in_flight: Option<JobHandle<Arc<Evaluated>, RecomputeError>>,
    state: PreviewState,
    /// The preview layer must be updated (cleared or replaced).
    dirty: bool,
    spawned: u64,
    now: Option<Instant>,
}

impl std::fmt::Debug for PreviewDriver {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PreviewDriver")
            .field("debounce", &self.debounce)
            .field("pending", &self.pending.is_some())
            .field("in_flight", &self.in_flight.is_some())
            .field("state", &self.state)
            .field("spawned", &self.spawned)
            .finish()
    }
}

impl Default for PreviewDriver {
    fn default() -> Self {
        PreviewDriver::new(DEFAULT_PREVIEW_DEBOUNCE)
    }
}

impl PreviewDriver {
    /// A driver with the given debounce.
    pub fn new(debounce: Duration) -> PreviewDriver {
        PreviewDriver {
            debounce,
            pending: None,
            in_flight: None,
            state: PreviewState::Idle,
            dirty: false,
            spawned: 0,
            now: None,
        }
    }

    /// The debounce.
    pub fn debounce(&self) -> Duration {
        self.debounce
    }

    /// Changes the debounce (preferences).
    pub fn set_debounce(&mut self, d: Duration) {
        self.debounce = d;
    }

    /// Debounced (prefs.preview_debounce); a newer call cancels the in-flight preview.
    pub fn set(&mut self, cmd: DocCommand) {
        if let Some(h) = self.in_flight.take() {
            h.cancel();
        }
        let now = self.clock();
        self.pending = Some((cmd, now + self.debounce));
        self.state = PreviewState::Pending;
    }

    /// Drops the pending and in-flight previews and clears the preview layer.
    pub fn clear(&mut self) {
        if let Some(h) = self.in_flight.take() {
            h.cancel();
        }
        let had = !matches!(self.state, PreviewState::Idle) || self.pending.is_some();
        self.pending = None;
        self.state = PreviewState::Idle;
        if had {
            self.dirty = true;
        }
    }

    /// Where the preview is.
    pub fn state(&self) -> &PreviewState {
        &self.state
    }

    /// Previews spawned so far (tests: "20 rapid `set`s → one preview").
    pub fn spawned(&self) -> u64 {
        self.spawned
    }

    /// True while a command waits or a preview job runs.
    pub fn is_busy(&self) -> bool {
        self.pending.is_some() || self.in_flight.is_some()
    }

    /// The preview to show, if ready.
    pub fn shown(&self) -> Option<&Arc<Evaluated>> {
        match &self.state {
            PreviewState::Ready(e) => Some(e),
            _ => None,
        }
    }

    /// Takes the "preview layer changed" flag.
    pub(crate) fn take_dirty(&mut self) -> bool {
        std::mem::take(&mut self.dirty)
    }

    /// Fixes the clock (tests); `None` returns to `Instant::now`.
    pub fn set_clock(&mut self, now: Option<Instant>) {
        self.now = now;
    }

    fn clock(&self) -> Instant {
        self.now.unwrap_or_else(Instant::now)
    }

    /// Once per frame: spawns the debounced preview, collects a finished one. Never blocks.
    pub fn update(&mut self, doc: &Document, jobs: &JobSystem) -> PreviewUpdate {
        let now = self.clock();
        let mut out = PreviewUpdate::default();
        if let Some((_, due)) = &self.pending {
            if *due <= now {
                if let Some((cmd, _)) = self.pending.take() {
                    match doc.preview(jobs, &cmd) {
                        Ok(h) => {
                            self.spawned += 1;
                            self.in_flight = Some(h);
                            self.state = PreviewState::Computing;
                        }
                        Err(e) => {
                            self.state = PreviewState::Failed(e.to_string());
                            self.dirty = true;
                        }
                    }
                }
            } else {
                out.wake_after = Some(*due - now);
            }
        }
        if let Some(h) = &mut self.in_flight
            && let Some(result) = h.try_take()
        {
            self.in_flight = None;
            match result {
                Ok(ev) => self.state = PreviewState::Ready(ev),
                Err(JobError::Cancelled) => {}
                Err(e) => self.state = PreviewState::Failed(e.to_string()),
            }
            self.dirty = true;
        }
        out.changed = self.dirty;
        out
    }
}
