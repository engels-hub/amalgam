//! The deterministic chain runner (design note §2.1): strategies in a fixed order, each result
//! through the validation gate, and a per-attempt deadline whose expiry aborts the chain.
//!
//! [`Chain`] is public so that the fillet (KH-03) and loft (KH-08) chains, and tests with
//! injected strategies, run through the same rules as [`super::robust_boolean`].

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{self, RecvTimeoutError};
use std::sync::{Arc, OnceLock};
use std::time::{Duration, Instant};

use amg_jobs::{CancelToken, JobCtx};

use super::{
    Attempt, AttemptOutcome, RetryPolicy, RobustCtx, RobustError, RobustOutput, RobustReport,
    StrategyId, Validation, is_final, validate,
};
use crate::capture::{FailureClass, FailureInfo, KernelOp};
use crate::error::{KResult, KernelError};
use crate::history::OpResult;
use crate::kernel::BrepKernel;
use crate::shape::Shape;

/// Deadlines longer than this are not armed.
const UNARMED: Duration = Duration::from_secs(24 * 3600);

/// What a strategy runs: the op under the attempt's context.
pub type RunFn<'s> = dyn FnOnce(&JobCtx) -> KResult<OpResult> + 's;

/// One strategy of a chain.
pub struct Strategy<'s> {
    /// Its id, logged in the report.
    pub id: StrategyId,
    /// The gate this strategy's result must pass.
    pub validation: Validation,
    /// What the acceptance warning says it did, e.g. `fuzzy 1e-4 mm`.
    pub detail: String,
    /// Further health warnings added when it is accepted.
    pub warnings: Vec<String>,
    /// Runs the strategy. The context carries the caller's cancellation plus the attempt's
    /// deadline (its token is cancelled when the deadline expires), but no progress.
    pub run: Box<RunFn<'s>>,
}

impl std::fmt::Debug for Strategy<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Strategy")
            .field("id", &self.id)
            .field("validation", &self.validation)
            .field("detail", &self.detail)
            .finish_non_exhaustive()
    }
}

impl<'s> Strategy<'s> {
    /// A strategy without extra warnings.
    pub fn new(
        id: StrategyId,
        validation: Validation,
        detail: impl Into<String>,
        run: impl FnOnce(&JobCtx) -> KResult<OpResult> + 's,
    ) -> Self {
        Strategy {
            id,
            validation,
            detail: detail.into(),
            warnings: Vec::new(),
            run: Box::new(run),
        }
    }
}

/// The process-wide deadline thread: one long-lived thread serves every armed attempt, so an
/// attempt costs two channel messages instead of a thread spawn.
enum TimerMsg {
    Arm {
        id: u64,
        at: Instant,
        token: CancelToken,
        fired: Arc<AtomicBool>,
    },
    Disarm(u64),
}

fn timer() -> Option<&'static mpsc::Sender<TimerMsg>> {
    static TIMER: OnceLock<Option<mpsc::Sender<TimerMsg>>> = OnceLock::new();
    TIMER
        .get_or_init(|| {
            let (tx, rx) = mpsc::channel::<TimerMsg>();
            std::thread::Builder::new()
                .name("amg-robust-deadline".into())
                .spawn(move || timer_loop(&rx))
                .ok()
                .map(|_| tx)
        })
        .as_ref()
}

fn timer_loop(rx: &mpsc::Receiver<TimerMsg>) {
    let mut pending: Vec<(u64, Instant, CancelToken, Arc<AtomicBool>)> = Vec::new();
    loop {
        let now = Instant::now();
        pending.retain(|(_, at, token, fired)| {
            if *at <= now {
                fired.store(true, Ordering::Release);
                token.cancel();
                false
            } else {
                true
            }
        });
        let msg = match pending.iter().map(|p| p.1).min() {
            None => match rx.recv() {
                Ok(m) => m,
                Err(_) => return,
            },
            Some(at) => match rx.recv_timeout(at.saturating_duration_since(now)) {
                Ok(m) => m,
                Err(RecvTimeoutError::Timeout) => continue,
                Err(RecvTimeoutError::Disconnected) => return,
            },
        };
        match msg {
            TimerMsg::Arm {
                id,
                at,
                token,
                fired,
            } => pending.push((id, at, token, fired)),
            TimerMsg::Disarm(id) => pending.retain(|p| p.0 != id),
        }
    }
}

/// Cancels `token` when `deadline` passes before it is dropped.
struct Watchdog {
    id: Option<u64>,
    fired: Arc<AtomicBool>,
}

impl Watchdog {
    fn arm(token: CancelToken, deadline: Duration) -> Watchdog {
        static NEXT: AtomicU64 = AtomicU64::new(0);
        let fired = Arc::new(AtomicBool::new(false));
        let mut id = None;
        if deadline < UNARMED
            && let Some(tx) = timer()
        {
            let n = NEXT.fetch_add(1, Ordering::Relaxed);
            let msg = TimerMsg::Arm {
                id: n,
                at: Instant::now() + deadline,
                token,
                fired: Arc::clone(&fired),
            };
            if tx.send(msg).is_ok() {
                id = Some(n);
            }
        }
        Watchdog { id, fired }
    }

    fn fired(&self) -> bool {
        self.fired.load(Ordering::Acquire)
    }
}

impl Drop for Watchdog {
    fn drop(&mut self) {
        if let (Some(id), Some(tx)) = (self.id, timer()) {
            let _ = tx.send(TimerMsg::Disarm(id));
        }
    }
}

/// A running chain. Call [`Chain::attempt`] for each strategy in the fixed order until it
/// returns `Some`, then [`Chain::fail`] if every strategy failed.
pub struct Chain<'c> {
    k: &'c dyn BrepKernel,
    op: &'static str,
    policy: &'c RetryPolicy,
    cx: &'c RobustCtx<'c>,
    report: RobustReport,
    /// The direct attempt's failure: class, message, kernel error.
    first: Option<(FailureClass, String, Option<KernelError>)>,
}

impl std::fmt::Debug for Chain<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Chain")
            .field("op", &self.op)
            .field("report", &self.report)
            .finish_non_exhaustive()
    }
}

impl<'c> Chain<'c> {
    /// A chain for op `op` (for messages) under `policy`.
    pub fn new(
        k: &'c dyn BrepKernel,
        op: &'static str,
        policy: &'c RetryPolicy,
        cx: &'c RobustCtx<'c>,
    ) -> Self {
        Chain {
            k,
            op,
            policy,
            cx,
            report: RobustReport::default(),
            first: None,
        }
    }

    /// The report so far.
    pub fn report(&self) -> &RobustReport {
        &self.report
    }

    /// Whether the policy's attempt budget is used up.
    pub fn exhausted(&self) -> bool {
        self.report.attempts.len() >= usize::from(self.policy.max_attempts)
    }

    /// Runs `s`. `None`: it failed or was rejected, go on with the next strategy. `Some(Ok)`:
    /// accepted, the chain is done. `Some(Err)`: the chain is aborted (deadline, cancellation,
    /// an argument error no strategy can fix, or the attempt budget is used up).
    pub fn attempt(&mut self, s: Strategy<'_>) -> Option<Result<RobustOutput, RobustError>> {
        if self.cx.job.is_cancelled() {
            return Some(Err(self.abort(
                FailureClass::Cancelled,
                "cancelled".into(),
                Some(KernelError::Cancelled),
            )));
        }
        if self.exhausted() {
            return Some(Err(self.fail()));
        }
        let n = self.report.attempts.len();
        let max = f32::from(self.policy.max_attempts.max(1));
        #[allow(clippy::cast_precision_loss, reason = "n < 256")]
        self.cx.job.progress(n as f32 / max);

        let token = self.cx.job.cancel_token().child();
        let ctx = JobCtx::with_token(token.clone());
        let deadline = self.policy.attempt_deadline;
        let watchdog = Watchdog::arm(token, deadline);
        let start = Instant::now();
        let result = (s.run)(&ctx);
        let elapsed = start.elapsed();
        let timed_out = watchdog.fired() || elapsed > deadline;
        drop(watchdog);
        let millis = u32::try_from(elapsed.as_millis()).unwrap_or(u32::MAX);

        if self.cx.job.is_cancelled() {
            self.log(s.id, AttemptOutcome::Failed(cancelled_info()), millis);
            return Some(Err(self.abort(
                FailureClass::Cancelled,
                "cancelled".into(),
                Some(KernelError::Cancelled),
            )));
        }
        if timed_out {
            let message = format!(
                "{} attempt {} exceeded its {} ms deadline",
                self.op,
                s.id,
                deadline.as_millis()
            );
            self.log(
                s.id,
                AttemptOutcome::Failed(FailureInfo::new(FailureClass::Timeout, message.clone())),
                millis,
            );
            return Some(Err(self.abort(FailureClass::Timeout, message, None)));
        }
        match result {
            Err(e) => {
                let info = FailureInfo::from_kernel_error(&e, None);
                let stop = is_final(&e);
                if self.first.is_none() {
                    self.first = Some((info.class, info.message.clone(), Some(e.clone())));
                }
                self.log(s.id, AttemptOutcome::Failed(info.clone()), millis);
                if stop {
                    return Some(Err(self.abort(info.class, info.message, Some(e))));
                }
                None
            }
            Ok(r) => match validate(self.k, &r.shape, &s.validation) {
                Err(rejection) => {
                    if self.first.is_none() {
                        self.first = Some((
                            rejection.class(),
                            format!("{} result rejected: {rejection}", self.op),
                            None,
                        ));
                    }
                    self.log(s.id, AttemptOutcome::Rejected(rejection), millis);
                    None
                }
                Ok(max_tol) => {
                    self.log(s.id, AttemptOutcome::Accepted, millis);
                    let mut report = std::mem::take(&mut self.report);
                    report.accepted = Some(s.id);
                    report.max_tolerance = max_tol;
                    if s.id != StrategyId::Direct {
                        report.warnings.push(format!(
                            "{} built with {} ({}) after {} failed attempt(s)",
                            self.op,
                            s.detail,
                            s.id,
                            report.attempts.len() - 1
                        ));
                        report.warnings.extend(s.warnings);
                    }
                    Some(Ok(RobustOutput { result: r, report }))
                }
            },
        }
    }

    /// Runs `strategies` in order until one is accepted or the chain aborts; the iterator is
    /// lazy, so strategies after an acceptance or abort are never even built.
    ///
    /// # Errors
    /// As [`Chain::attempt`], or [`Chain::fail`] when every strategy failed.
    pub fn run<'s>(
        mut self,
        strategies: impl IntoIterator<Item = Strategy<'s>>,
    ) -> Result<RobustOutput, RobustError> {
        for s in strategies {
            if let Some(done) = self.attempt(s) {
                return done;
            }
        }
        Err(self.fail())
    }

    /// The error after every strategy failed: the direct attempt's class and message.
    pub fn fail(&self) -> RobustError {
        let (class, message, error) = self.first.clone().unwrap_or((
            FailureClass::Other,
            format!("{}: no strategy ran", self.op),
            None,
        ));
        RobustError {
            class,
            message: format!(
                "{message} ({} attempt(s) failed)",
                self.report.attempts.len()
            ),
            report: self.report.clone(),
            error: error.map(Box::new),
        }
    }

    fn abort(
        &self,
        class: FailureClass,
        message: String,
        error: Option<KernelError>,
    ) -> RobustError {
        RobustError {
            class,
            message,
            report: self.report.clone(),
            error: error.map(Box::new),
        }
    }

    fn log(&mut self, strategy: StrategyId, outcome: AttemptOutcome, millis: u32) {
        self.report.attempts.push(Attempt {
            strategy,
            outcome,
            millis,
        });
    }
}

/// Runs `f` under a child of `job`'s cancellation that is also cancelled when `deadline` passes
/// (the chain's attempt rule, for calls that are not attempts: KH-03's diagnostic probes).
/// Returns `f`'s value and whether the deadline expired.
pub(crate) fn with_deadline<T>(
    job: &JobCtx,
    deadline: Duration,
    f: impl FnOnce(&JobCtx) -> T,
) -> (T, bool) {
    let token = job.cancel_token().child();
    let ctx = JobCtx::with_token(token.clone());
    let watchdog = Watchdog::arm(token, deadline);
    let start = Instant::now();
    let value = f(&ctx);
    let timed_out = watchdog.fired() || start.elapsed() > deadline;
    (value, timed_out)
}

fn cancelled_info() -> FailureInfo {
    FailureInfo::new(FailureClass::Cancelled, "cancelled")
}

/// Writes a dump for a final failure through `cx.recorder` (not for `Cancelled`), with the
/// chain's attempts. Recording errors are logged, never returned.
pub fn record_failure(cx: &RobustCtx<'_>, op: &KernelOp, inputs: &[&Shape], e: &RobustError) {
    let Some(recorder) = cx.recorder else {
        return;
    };
    if matches!(e.class, FailureClass::Cancelled)
        || matches!(e.error.as_deref(), Some(KernelError::BackendUnavailable))
    {
        return;
    }
    let info = match e.error.as_deref() {
        Some(err) => {
            let mut info = FailureInfo::from_kernel_error(err, inputs.first().copied());
            info.class = e.class;
            info.message.clone_from(&e.message);
            info
        }
        None => FailureInfo::new(e.class, e.message.clone()),
    };
    match recorder.record(op, inputs, &info, &e.report.dump_attempts()) {
        Ok(id) => tracing::info!("robust failure captured: {id}"),
        Err(err) => tracing::warn!("robust failure capture failed: {err}"),
    }
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::AtomicUsize;

    use super::*;
    use crate::{KernelConfig, OcctKernel};

    fn not_done() -> KernelError {
        KernelError::OpFailed {
            op: "test",
            message: "injected".into(),
            offending: Vec::new(),
        }
    }

    /// A strategy that works for `busy` and polls its context like an OCCT op does.
    fn slow(busy: Duration, ctx: &JobCtx) -> KResult<OpResult> {
        let start = Instant::now();
        while start.elapsed() < busy {
            ctx.checkpoint()?;
            std::thread::sleep(Duration::from_millis(2));
        }
        Err(not_done())
    }

    fn policy(deadline_ms: u64) -> RetryPolicy {
        RetryPolicy {
            attempt_deadline: Duration::from_millis(deadline_ms),
            ..RetryPolicy::recompute()
        }
    }

    #[test]
    fn a_deadline_aborts_the_chain_with_timeout() {
        let k = OcctKernel::new(KernelConfig::default());
        let job = JobCtx::detached();
        let cx = RobustCtx::new(&job);
        let p = policy(50);
        let later = AtomicUsize::new(0);
        let chain = Chain::new(&k, "test", &p, &cx);
        let start = Instant::now();
        let strategies = [
            Strategy::new(
                StrategyId::Custom(0),
                Validation::default(),
                "slow",
                |ctx| slow(Duration::from_secs(5), ctx),
            ),
            Strategy::new(StrategyId::Custom(1), Validation::default(), "fast", |_| {
                later.fetch_add(1, Ordering::Relaxed);
                Err(not_done())
            }),
        ];
        let e = chain
            .run(strategies)
            .expect_err("the slow strategy must abort the chain");
        let waited = start.elapsed();
        assert_eq!(e.class, FailureClass::Timeout);
        assert!(
            waited < Duration::from_secs(2),
            "cancelled late: {waited:?}"
        );
        assert_eq!(e.report.attempts.len(), 1);
        assert!(e.report.accepted.is_none());
        // The later strategy never ran, so its result can never be returned.
        assert_eq!(later.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn failures_fall_through_in_order_and_the_budget_holds() {
        let k = OcctKernel::new(KernelConfig::default());
        let job = JobCtx::detached();
        let cx = RobustCtx::new(&job);
        let p = RetryPolicy {
            max_attempts: 2,
            ..policy(1000)
        };
        let mut chain = Chain::new(&k, "test", &p, &cx);
        for i in 0..2 {
            let r = chain.attempt(Strategy::new(
                StrategyId::Custom(i),
                Validation::default(),
                "x",
                |_| Err(not_done()),
            ));
            assert!(r.is_none());
        }
        let Some(Err(e)) = chain.attempt(Strategy::new(
            StrategyId::Custom(9),
            Validation::default(),
            "x",
            |_| panic!("over budget"),
        )) else {
            panic!("budget");
        };
        assert_eq!(e.class, FailureClass::NotDone);
        let ids: Vec<_> = e.report.attempts.iter().map(|a| a.strategy).collect();
        assert_eq!(ids, vec![StrategyId::Custom(0), StrategyId::Custom(1)]);
        assert!(matches!(
            e.error.as_deref(),
            Some(KernelError::OpFailed { .. })
        ));
    }

    #[test]
    fn argument_errors_and_cancellation_stop_the_chain() {
        let k = OcctKernel::new(KernelConfig::default());
        let job = JobCtx::detached();
        let cx = RobustCtx::new(&job);
        let p = policy(1000);
        let mut chain = Chain::new(&k, "test", &p, &cx);
        let Some(Err(e)) = chain.attempt(Strategy::new(
            StrategyId::Direct,
            Validation::default(),
            "x",
            |_| Err(KernelError::BackendUnavailable),
        )) else {
            panic!("final error");
        };
        assert!(matches!(
            e.error.as_deref(),
            Some(KernelError::BackendUnavailable)
        ));

        let token = CancelToken::new();
        let job = JobCtx::with_token(token.clone());
        let cx = RobustCtx::new(&job);
        let mut chain = Chain::new(&k, "test", &p, &cx);
        let Some(Err(e)) = chain.attempt(Strategy::new(
            StrategyId::Direct,
            Validation::default(),
            "x",
            |ctx| {
                token.cancel();
                ctx.checkpoint()?;
                Err(not_done())
            },
        )) else {
            panic!("cancel");
        };
        assert_eq!(e.class, FailureClass::Cancelled);
        assert!(matches!(e.error.as_deref(), Some(KernelError::Cancelled)));
    }
}
