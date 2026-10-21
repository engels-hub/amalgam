//! Untyped per-job state shared by the handle, the context and the system registry.

use std::{
    borrow::Cow,
    sync::{
        Arc, OnceLock, Weak,
        atomic::{AtomicU8, AtomicU32, AtomicU64, Ordering},
    },
    time::{Duration, Instant},
};

use parking_lot::{Condvar, Mutex};

use crate::{
    CancelToken, JobId, JobInfo, JobState, Priority, Progress,
    system::{Inner, Notifier},
};

pub(crate) const QUEUED: u8 = 0;
pub(crate) const RUNNING: u8 = 1;
pub(crate) const SUCCEEDED: u8 = 2;
pub(crate) const FAILED: u8 = 3;
pub(crate) const CANCELLED: u8 = 4;
pub(crate) const PANICKED: u8 = 5;

/// Minimum interval between progress messages and between progress notifications (30 Hz).
pub(crate) const RATE_LIMIT: Duration = Duration::from_nanos(1_000_000_000 / 30);
const NEVER: u64 = u64::MAX;

fn now_ns() -> u64 {
    static EPOCH: OnceLock<Instant> = OnceLock::new();
    let epoch = *EPOCH.get_or_init(Instant::now);
    u64::try_from(epoch.elapsed().as_nanos()).unwrap_or(NEVER - 1)
}

fn rate_limit_ns() -> u64 {
    u64::try_from(RATE_LIMIT.as_nanos()).unwrap_or(NEVER)
}

/// Normalises to `[0, 1]`, mapping NaN and `-0.0` to `+0.0` so f32 bit order equals value order.
pub(crate) fn unit(x: f32) -> f32 {
    if x > 0.0 { x.min(1.0) } else { 0.0 }
}

pub(crate) struct JobCore {
    pub(crate) id: JobId,
    pub(crate) label: Cow<'static, str>,
    pub(crate) priority: Priority,
    pub(crate) parent: Option<JobId>,
    pub(crate) hidden: bool,
    pub(crate) created: Instant,
    pub(crate) started: OnceLock<Instant>,
    pub(crate) token: CancelToken,
    state: AtomicU8,
    /// Error / panic text; set before the terminal state is published.
    detail: OnceLock<String>,
    /// f32 bits of the root fraction in `[0, 1]`. Non-negative f32 bit patterns order like the
    /// values, so `fetch_max` keeps it monotone with a single atomic op.
    fraction: AtomicU32,
    message: Mutex<Option<Arc<str>>>,
    last_msg_ns: AtomicU64,
    last_notify_ns: AtomicU64,
    notifier: Option<Notifier>,
    done_lock: Mutex<()>,
    done_cv: Condvar,
    system: Weak<Inner>,
}

impl JobCore {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        id: JobId,
        label: Cow<'static, str>,
        priority: Priority,
        parent: Option<JobId>,
        hidden: bool,
        token: CancelToken,
        notifier: Option<Notifier>,
        system: Weak<Inner>,
    ) -> JobCore {
        JobCore {
            id,
            label,
            priority,
            parent,
            hidden,
            created: Instant::now(),
            started: OnceLock::new(),
            token,
            state: AtomicU8::new(QUEUED),
            detail: OnceLock::new(),
            fraction: AtomicU32::new(0.0f32.to_bits()),
            message: Mutex::new(None),
            last_msg_ns: AtomicU64::new(NEVER),
            last_notify_ns: AtomicU64::new(NEVER),
            notifier,
            done_lock: Mutex::new(()),
            done_cv: Condvar::new(),
            system,
        }
    }

    pub(crate) fn raw_state(&self) -> u8 {
        self.state.load(Ordering::Acquire)
    }

    pub(crate) fn is_finished(&self) -> bool {
        self.raw_state() >= SUCCEEDED
    }

    /// `Queued → Running`. False if the job was already cancelled out of the queue.
    pub(crate) fn try_start(&self) -> bool {
        // Set before publishing `Running`, so a running job always has `started`. If a cancel
        // wins the race, `started` still records when a pool thread picked the job up.
        let _ = self.started.set(Instant::now());
        let ok = self
            .state
            .compare_exchange(QUEUED, RUNNING, Ordering::AcqRel, Ordering::Acquire)
            .is_ok();
        if ok {
            self.notify();
        }
        ok
    }

    /// Cancels the token and, if the job has not started, finishes it as `Cancelled` right away
    /// (so queued jobs resolve without waiting for a pool thread).
    pub(crate) fn cancel(&self) {
        self.token.cancel();
        self.cancel_if_queued();
    }

    pub(crate) fn cancel_if_queued(&self) {
        if self
            .state
            .compare_exchange(QUEUED, CANCELLED, Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
        {
            self.published_terminal();
        }
    }

    /// Publishes a terminal state. The result (if any) must already be in the channel.
    pub(crate) fn finish(&self, code: u8, detail: Option<String>) {
        debug_assert!(code >= SUCCEEDED);
        if let Some(d) = detail {
            let _ = self.detail.set(d);
        }
        if code == SUCCEEDED {
            self.fraction.fetch_max(1.0f32.to_bits(), Ordering::Relaxed);
        }
        // Leave the registry first, so a handle that reports "finished" is never still listed.
        self.unregister();
        self.state.store(code, Ordering::Release);
        self.wake_waiters_and_notify();
    }

    fn unregister(&self) {
        if let Some(sys) = self.system.upgrade() {
            sys.unregister(self.id);
        }
    }

    /// After a `Queued → Cancelled` transition.
    fn published_terminal(&self) {
        self.unregister();
        self.wake_waiters_and_notify();
    }

    fn wake_waiters_and_notify(&self) {
        {
            let _g = self.done_lock.lock();
            self.done_cv.notify_all();
        }
        self.notify();
    }

    /// Blocks until finished, or until `timeout` elapses if given.
    pub(crate) fn wait_done(&self, timeout: Option<Duration>) {
        let mut g = self.done_lock.lock();
        if self.is_finished() {
            return;
        }
        match timeout {
            Some(t) => {
                let _ = self.done_cv.wait_for(&mut g, t);
            }
            None => {
                while !self.is_finished() {
                    self.done_cv.wait(&mut g);
                }
            }
        }
    }

    /// Raises the root fraction to `fraction` (already mapped to the root range).
    pub(crate) fn raise_progress(&self, fraction: f32) {
        let bits = unit(fraction).to_bits();
        let prev = self.fraction.fetch_max(bits, Ordering::Relaxed);
        if bits > prev {
            self.notify_rate_limited();
        }
    }

    pub(crate) fn set_message(&self, message: &str) {
        let now = now_ns();
        let last = self.last_msg_ns.load(Ordering::Relaxed);
        if last != NEVER && now.saturating_sub(last) < rate_limit_ns() {
            return;
        }
        if self
            .last_msg_ns
            .compare_exchange(last, now, Ordering::Relaxed, Ordering::Relaxed)
            .is_err()
        {
            return; // another thread just set one
        }
        *self.message.lock() = Some(Arc::from(message));
        self.notify_rate_limited();
    }

    pub(crate) fn progress(&self) -> Progress {
        Progress {
            fraction: f32::from_bits(self.fraction.load(Ordering::Relaxed)),
            message: self.message.lock().clone(),
        }
    }

    pub(crate) fn state(&self) -> JobState {
        match self.raw_state() {
            QUEUED => JobState::Queued,
            RUNNING => JobState::Running(self.progress()),
            SUCCEEDED => JobState::Succeeded,
            FAILED => JobState::Failed(self.detail.get().cloned().unwrap_or_default()),
            PANICKED => JobState::Panicked(self.detail.get().cloned().unwrap_or_default()),
            _ => JobState::Cancelled,
        }
    }

    pub(crate) fn info(&self) -> JobInfo {
        JobInfo {
            id: self.id,
            label: self.label.clone(),
            priority: self.priority,
            parent: self.parent,
            state: self.state(),
            created: self.created,
            started: self.started.get().copied(),
        }
    }

    /// Calls the notifier (state changes). Panics in the notifier are swallowed.
    pub(crate) fn notify(&self) {
        if let Some(n) = &self.notifier {
            let _ = crate::diag::catch_panic(std::panic::AssertUnwindSafe(|| n()));
        }
    }

    fn notify_rate_limited(&self) {
        if self.notifier.is_none() {
            return;
        }
        let now = now_ns();
        let last = self.last_notify_ns.load(Ordering::Relaxed);
        if last != NEVER && now.saturating_sub(last) < rate_limit_ns() {
            return;
        }
        if self
            .last_notify_ns
            .compare_exchange(last, now, Ordering::Relaxed, Ordering::Relaxed)
            .is_ok()
        {
            self.notify();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::unit;

    #[test]
    fn unit_normalises() {
        assert_eq!(unit(f32::NAN).to_bits(), 0);
        assert_eq!(unit(-0.0).to_bits(), 0);
        assert_eq!(unit(-3.0).to_bits(), 0);
        assert_eq!(unit(2.0).to_bits(), 1.0f32.to_bits());
        assert_eq!(unit(0.25).to_bits(), 0.25f32.to_bits());
    }
}
