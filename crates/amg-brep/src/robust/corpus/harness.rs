//! The process-isolated corpus harness (KH-02, design note §6).
//!
//! [`run_isolated`] runs every `(case, strategy)` in **worker processes**: one per job slot, a
//! fresh one after [`HarnessConfig::cases_per_worker`] cases. The parent sends one JSON
//! [`Request`] per line on the worker's stdin; the worker ([`serve`]) prepares the case, runs the
//! op, judges the result and prints one [`RECORD_PREFIX`] line. A worker that dies inside a
//! case (an OCCT access violation, an abort, a stack overflow) gives a `Crash` record for that
//! case, one that does not answer within [`HarnessConfig::timeout`] is killed and gives a
//! `Timeout` record; either way the slot starts a new worker and the run goes on.
//!
//! A worker is any program that calls [`serve_stdio`]: the `kernel-harness` example, `amg-cli
//! kernel harness-worker`, or a test binary re-running itself (`tests/kernel_corpus.rs`).
//!
//! **Judging** (robustness note §6). An `Ok` is *valid* when BRepCheck passes, every face is in
//! a closed solid of positive volume, a one-solid input gives one solid, and the volume and
//! face count match the case's oracle where it has one; otherwise it is an *invalid Ok*. An
//! `Err` is *classified* when its [`FailureClass`] is anything but `Other` or `Panic`. A case
//! *passes* when it expects success and gets a valid `Ok`, or expects a clean failure and gets
//! a classified `Err`. [`Report`] (JSON, schema [`REPORT_SCHEMA`]) aggregates per strategy and
//! tag; [`Baseline`] stores the passing ids per strategy and [`Baseline::regressions`] is the
//! regression gate.

use std::collections::{BTreeMap, BTreeSet};
use std::io::{BufRead, BufReader, Write};
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::path::Path;
use std::process::{Child, ChildStdin, Command, Stdio};
use std::sync::Mutex;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::mpsc::{self, RecvTimeoutError};
use std::time::{Duration, Instant};

use amg_jobs::JobCtx;

use super::case::{Case, CaseError, ExpectOutcome, Inject, Prepared};
use crate::capture::FailureClass;
use crate::config::{KernelConfig, Validate};
use crate::error::KernelError;
use crate::kernel::{BrepKernel, OcctKernel};
use crate::robust::{HardenedKernel, Rejection, RetryPolicy, Validation, validate};
use crate::shape::{Shape, SubKind, SubShape};

/// The report JSON schema.
pub const REPORT_SCHEMA: u32 = 1;
/// The baseline TOML schema.
pub const BASELINE_SCHEMA: u32 = 1;
/// The prefix of a worker's answer line on stdout.
pub const RECORD_PREFIX: &str = "AMG_HARNESS_RECORD ";
/// The prefix of a worker's progress line: `<index> prepare` before it builds or reads the inputs,
/// `<index> op` before it runs the op, so a crash record says where the worker died.
pub const PHASE_PREFIX: &str = "AMG_HARNESS_PHASE ";
/// The default per-case timeout (design note §6).
pub const DEFAULT_TIMEOUT: Duration = Duration::from_secs(60);
/// Cases per worker process before it is recycled (design note §6).
pub const CASES_PER_WORKER: usize = 50;
/// Valid results with a sub-shape tolerance above this are counted as "above the cap" (the
/// robust gate's default `tol_cap`, mm).
pub const TOL_CAP: f64 = 1e-3;
/// The exit status of an unhandled access violation on Windows (`STATUS_ACCESS_VIOLATION`).
#[allow(
    clippy::cast_possible_wrap,
    reason = "the NTSTATUS bit pattern as an exit code"
)]
pub const STATUS_ACCESS_VIOLATION: i32 = 0xC000_0005_u32 as i32;

/// How an op is run.
#[derive(
    Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize,
)]
#[serde(rename_all = "snake_case")]
pub enum Strategy {
    /// The raw OCCT op (`OcctKernel` with the validation policy off, so OCCT's own invalid
    /// results stay visible): the baseline.
    Raw,
    /// Through `HardenedKernel` with `RetryPolicy::recompute()`: fillets run the KH-03 strategy
    /// chain (`robust::fillet::fillet`).
    Hardened,
}

impl Strategy {
    /// Every strategy.
    pub const ALL: [Strategy; 2] = [Strategy::Raw, Strategy::Hardened];

    /// Its name.
    pub fn name(self) -> &'static str {
        match self {
            Strategy::Raw => "raw",
            Strategy::Hardened => "hardened",
        }
    }

    /// The strategy called `name`.
    pub fn from_name(name: &str) -> Option<Strategy> {
        Strategy::ALL.into_iter().find(|s| s.name() == name)
    }
}

/// What the op did.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Outcome {
    /// It returned a result.
    Ok,
    /// It returned an error (or panicked, or the case could not be prepared).
    Err,
    /// The worker process died inside the case.
    Crash,
    /// The worker did not answer in time and was killed.
    Timeout,
}

/// The judged outcome.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Verdict {
    /// A valid result.
    ValidOk,
    /// A result that fails the independent check (target 0).
    InvalidOk,
    /// An error with a meaningful failure class.
    ClassifiedErr,
    /// An error of class `Other` or a panic.
    UnclassifiedErr,
    /// The case's inputs could not be built or read (a corpus or setup problem, not an op
    /// failure; for example `BackendUnavailable` without OCCT).
    SetupError,
    /// The worker crashed.
    Crash,
    /// The worker timed out.
    Timeout,
}

/// One `(case, strategy)` result.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct CaseRecord {
    /// The case id.
    pub id: String,
    /// The strategy.
    pub strategy: Strategy,
    /// The case's tags.
    pub tags: Vec<String>,
    /// What the case expects.
    pub expect: ExpectOutcome,
    /// What happened.
    pub outcome: Outcome,
    /// The judged outcome.
    pub verdict: Verdict,
    /// Whether the verdict is what the case expects.
    pub passed: bool,
    /// The error's class (`Crash`/`Timeout` for those outcomes).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub failure_class: Option<FailureClass>,
    /// Whether `failure_class` is the expected one, when the case names one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub class_matches: Option<bool>,
    /// The error message.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub message: Option<String>,
    /// The sub-shapes the error names (`KernelError::offending`: the failing edges of a fillet).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub offending: Vec<SubShape>,
    /// Why an `Ok` is invalid.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub problems: Vec<String>,
    /// The result volume, mm³.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub volume: Option<f64>,
    /// `(volume − oracle) / oracle` where the case has an exact volume.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub volume_dev: Option<f64>,
    /// The result's largest sub-shape tolerance, mm.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_tolerance: Option<f64>,
    /// The result's face count.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub faces: Option<u32>,
    /// Wall time of the op alone (not the input build or the judging), ms; for crashes and
    /// timeouts the time until the parent noticed.
    pub millis: f64,
}

impl CaseRecord {
    fn new(case: &Case, strategy: Strategy, expect: ExpectOutcome) -> CaseRecord {
        CaseRecord {
            id: case.id.clone(),
            strategy,
            tags: case.tags.clone(),
            expect,
            outcome: Outcome::Err,
            verdict: Verdict::SetupError,
            passed: false,
            failure_class: None,
            class_matches: None,
            message: None,
            offending: Vec::new(),
            problems: Vec::new(),
            volume: None,
            volume_dev: None,
            max_tolerance: None,
            faces: None,
            millis: 0.0,
        }
    }

    /// The record of a worker that crashed or timed out in `case`.
    pub fn abnormal(
        case: &Case,
        strategy: Strategy,
        outcome: Outcome,
        message: String,
        millis: f64,
    ) -> CaseRecord {
        let (verdict, class) = match outcome {
            Outcome::Timeout => (Verdict::Timeout, FailureClass::Timeout),
            _ => (Verdict::Crash, FailureClass::Crash),
        };
        CaseRecord {
            outcome,
            verdict,
            failure_class: Some(class),
            message: Some(message),
            millis,
            ..CaseRecord::new(case, strategy, case.expect.outcome)
        }
    }

    /// Whether the record has `tag`.
    pub fn has_tag(&self, tag: &str) -> bool {
        self.tags.iter().any(|t| t == tag)
    }
}

/// Whether `class` is a meaningful classification.
pub fn is_classified(class: FailureClass) -> bool {
    !matches!(
        class,
        FailureClass::Other | FailureClass::Panic | FailureClass::Crash | FailureClass::Timeout
    )
}

fn passes(expect: ExpectOutcome, verdict: Verdict) -> bool {
    matches!(
        (expect, verdict),
        (ExpectOutcome::Success, Verdict::ValidOk)
            | (ExpectOutcome::CleanFailure, Verdict::ClassifiedErr)
    )
}

fn millis(d: Duration) -> f64 {
    d.as_secs_f64() * 1e3
}

/// Runs `case` with `k` (the kernel of `strategy`) in this process and judges the result. A
/// panic in the op is caught; an injected [`Inject::AccessViolation`], [`Inject::Abort`] or
/// [`Inject::Hang`] ends or blocks the process, so those belong in a worker only.
pub fn run_case(k: &dyn BrepKernel, case: &Case, strategy: Strategy) -> CaseRecord {
    run_case_phased(k, case, strategy, &mut |_| {})
}

/// [`run_case`], calling `phase("prepare")` before the inputs are built or read and
/// `phase("op")` before the op runs.
pub fn run_case_phased(
    k: &dyn BrepKernel,
    case: &Case,
    strategy: Strategy,
    phase: &mut dyn FnMut(&'static str),
) -> CaseRecord {
    let mut rec = CaseRecord::new(case, strategy, case.expect.outcome);
    if let Some(inject) = case.inject {
        // An injected fault replaces the whole case, so it fires with or without a backend.
        phase("op");
        let start = Instant::now();
        let result = catch_unwind(AssertUnwindSafe(|| injected(inject)));
        rec.millis = millis(start.elapsed());
        match result {
            Err(payload) => panicked(&mut rec, payload.as_ref()),
            Ok(e) => failed(&mut rec, &e, case.expect.failure_class),
        }
        rec.passed = passes(rec.expect, rec.verdict);
        return rec;
    }
    phase("prepare");
    let prepared = match catch_unwind(AssertUnwindSafe(|| case.prepare(k))) {
        Ok(Ok(p)) => p,
        Ok(Err(e)) => {
            rec.message = Some(e.to_string());
            if let CaseError::Kernel(ke) = &e {
                rec.failure_class = Some(FailureClass::of(ke));
            }
            return rec;
        }
        Err(_) => {
            rec.message = Some("preparing the case panicked".to_owned());
            rec.failure_class = Some(FailureClass::Panic);
            return rec;
        }
    };
    rec.expect = prepared.expect.outcome;
    phase("op");
    let ctx = JobCtx::detached();
    let start = Instant::now();
    let result = catch_unwind(AssertUnwindSafe(|| {
        prepared.op.run(k, &prepared.inputs, &ctx)
    }));
    rec.millis = millis(start.elapsed());
    match result {
        Err(payload) => panicked(&mut rec, payload.as_ref()),
        Ok(Err(e)) => failed(&mut rec, &e, prepared.expect.failure_class),
        Ok(Ok(shapes)) => {
            rec.outcome = Outcome::Ok;
            judge(k, &prepared, shapes.first(), &mut rec);
        }
    }
    rec.passed = passes(rec.expect, rec.verdict);
    rec
}

/// Runs an injected fault: ends or blocks the process, panics, or returns the injected error.
fn injected(inject: Inject) -> KernelError {
    match inject {
        Inject::AccessViolation => std::process::exit(STATUS_ACCESS_VIOLATION),
        Inject::Abort => std::process::abort(),
        Inject::Hang => loop {
            std::thread::sleep(Duration::from_secs(1));
        },
        Inject::Panic => panic!("injected panic"),
        Inject::NotDone => KernelError::OpFailed {
            op: "harness",
            message: "injected NotDone".to_owned(),
            offending: Vec::new(),
        },
    }
}

fn panicked(rec: &mut CaseRecord, payload: &(dyn std::any::Any + Send)) {
    let msg = payload
        .downcast_ref::<&str>()
        .map(|s| (*s).to_owned())
        .or_else(|| payload.downcast_ref::<String>().cloned())
        .unwrap_or_default();
    rec.outcome = Outcome::Err;
    rec.verdict = Verdict::UnclassifiedErr;
    rec.failure_class = Some(FailureClass::Panic);
    rec.message = Some(format!("panic: {msg}"));
}

fn failed(rec: &mut CaseRecord, e: &KernelError, expected: Option<FailureClass>) {
    let class = FailureClass::of(e);
    rec.outcome = Outcome::Err;
    rec.verdict = if is_classified(class) {
        Verdict::ClassifiedErr
    } else {
        Verdict::UnclassifiedErr
    };
    rec.failure_class = Some(class);
    rec.class_matches = expected.map(|c| c == class);
    rec.offending = e.offending().to_vec();
    rec.message = Some(e.to_string());
}

/// The independent check of an `Ok` (module docs).
fn judge(k: &dyn BrepKernel, p: &Prepared, shape: Option<&Shape>, rec: &mut CaseRecord) {
    let Some(s) = shape else {
        rec.verdict = Verdict::InvalidOk;
        rec.problems.push("no result shape".to_owned());
        return;
    };
    let gate = Validation {
        brepcheck: true,
        require_solid: true,
        max_tolerance: f64::INFINITY,
        volume_bounds: None,
        self_interference: false,
        disjoint_solids: false,
    };
    match validate(k, s, &gate) {
        Ok(_) => {}
        Err(Rejection::Check(kinds)) => rec.problems.extend(kinds),
        Err(r) => rec.problems.push(r.to_string()),
    }
    let solids = s.solids().len();
    let input_solids = p.inputs.first().map_or(0, |i| i.solids().len());
    if input_solids == 1 && solids != 1 {
        rec.problems
            .push(format!("{solids} solids from a one-solid input"));
    }
    let faces = s.count(SubKind::Face);
    rec.faces = Some(faces);
    if let Some(want) = p.expect.faces
        && want != faces
    {
        rec.problems.push(format!("{faces} faces, expected {want}"));
    }
    match s.volume() {
        Ok(v) => {
            rec.volume = Some(v);
            if let Some(want) = p.expect.volume {
                rec.volume_dev = Some((v - want) / want.abs().max(1e-12));
            }
            if let Some((lo, hi)) = p.expect.volume_range()
                && !(lo..=hi).contains(&v)
            {
                rec.problems
                    .push(format!("volume {v} outside [{lo}, {hi}]"));
            }
        }
        Err(e) => rec.problems.push(format!("volume: {e}")),
    }
    rec.max_tolerance = Some(s.max_tolerance());
    rec.verdict = if rec.problems.is_empty() {
        Verdict::ValidOk
    } else {
        Verdict::InvalidOk
    };
}

/// The kernels a worker runs the strategies on.
#[allow(missing_debug_implementations, reason = "boxed trait objects")]
pub struct Kernels {
    /// [`Strategy::Raw`].
    pub raw: Box<dyn BrepKernel>,
    /// [`Strategy::Hardened`].
    pub hardened: Box<dyn BrepKernel>,
}

impl Kernels {
    /// The OCCT kernels: raw with the validation policy off, hardened with
    /// `RetryPolicy::recompute()` around it.
    pub fn occt() -> Kernels {
        let cfg = KernelConfig {
            validate: Validate::Off,
            ..KernelConfig::default()
        };
        Kernels {
            raw: Box::new(OcctKernel::new(cfg.clone())),
            hardened: Box::new(HardenedKernel::new(
                OcctKernel::new(cfg),
                RetryPolicy::recompute(),
                None,
            )),
        }
    }

    /// The kernel of `s`.
    pub fn get(&self, s: Strategy) -> &dyn BrepKernel {
        match s {
            Strategy::Raw => self.raw.as_ref(),
            Strategy::Hardened => self.hardened.as_ref(),
        }
    }
}

/// One task sent to a worker.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Request {
    /// The task index, echoed in the answer.
    pub index: usize,
    /// The strategy.
    pub strategy: Strategy,
    /// The case.
    pub case: Case,
}

/// A worker's answer.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Response {
    /// The task index.
    pub index: usize,
    /// The record.
    pub record: CaseRecord,
}

/// The worker loop: one [`Request`] per input line, one [`RECORD_PREFIX`] line per answer,
/// until the input ends. Returns the number of cases served.
///
/// # Errors
/// I/O errors on `input` or `out`, and malformed request lines (`InvalidData`).
pub fn serve(
    kernels: &Kernels,
    input: impl BufRead,
    mut out: impl Write,
) -> std::io::Result<usize> {
    let mut served = 0;
    for line in input.lines() {
        let line = line?;
        if line.trim().is_empty() {
            continue;
        }
        let req: Request = serde_json::from_str(&line)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
        let mut phase_err = None;
        let record = run_case_phased(
            kernels.get(req.strategy),
            &req.case,
            req.strategy,
            &mut |phase| {
                if let Err(e) =
                    writeln!(out, "{PHASE_PREFIX}{} {phase}", req.index).and_then(|()| out.flush())
                {
                    phase_err.get_or_insert(e);
                }
            },
        );
        if let Some(e) = phase_err {
            return Err(e);
        }
        let json = serde_json::to_string(&Response {
            index: req.index,
            record,
        })
        .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
        writeln!(out, "{RECORD_PREFIX}{json}")?;
        out.flush()?;
        served += 1;
    }
    Ok(served)
}

/// [`serve`] on stdin/stdout with [`Kernels::occt`].
///
/// # Errors
/// As [`serve`].
pub fn serve_stdio() -> std::io::Result<usize> {
    let kernels = Kernels::occt();
    let stdin = std::io::stdin();
    serve(&kernels, stdin.lock(), std::io::stdout().lock())
}

/// How [`run_isolated`] runs.
#[derive(Clone, Debug, PartialEq)]
pub struct HarnessConfig {
    /// Worker processes running at once.
    pub jobs: usize,
    /// Cases per worker before it is replaced.
    pub cases_per_worker: usize,
    /// Per-case timeout.
    pub timeout: Duration,
    /// The strategies each case runs with.
    pub strategies: Vec<Strategy>,
}

impl Default for HarnessConfig {
    /// 4 jobs (fewer on smaller machines), [`CASES_PER_WORKER`], [`DEFAULT_TIMEOUT`], raw only.
    fn default() -> Self {
        let cores = std::thread::available_parallelism().map_or(1, std::num::NonZeroUsize::get);
        HarnessConfig {
            jobs: cores.clamp(1, 4),
            cases_per_worker: CASES_PER_WORKER,
            timeout: DEFAULT_TIMEOUT,
            strategies: vec![Strategy::Raw],
        }
    }
}

/// Why the harness itself failed.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum HarnessError {
    /// A worker could not be started.
    #[error("starting a worker: {0}")]
    Spawn(std::io::Error),
    /// A worker answered something unreadable or for another task.
    #[error("worker protocol: {0}")]
    Protocol(String),
}

enum Reply {
    Line(String),
    Timeout,
    Died,
}

enum Msg {
    Record(String),
    Phase(String),
}

/// A running worker process.
struct Worker {
    child: Child,
    stdin: Option<ChildStdin>,
    rx: mpsc::Receiver<Msg>,
    served: usize,
    /// The last progress line (`<index> <phase>`).
    phase: Option<String>,
}

impl Worker {
    fn spawn(mut cmd: Command) -> Result<Worker, HarnessError> {
        cmd.stdin(Stdio::piped()).stdout(Stdio::piped());
        let mut child = cmd.spawn().map_err(HarnessError::Spawn)?;
        let stdin = child.stdin.take();
        let stdout = child.stdout.take();
        let (tx, rx) = mpsc::channel();
        if let Some(stdout) = stdout {
            std::thread::spawn(move || {
                for line in BufReader::new(stdout).lines() {
                    let Ok(line) = line else { break };
                    // Anywhere in the line: a test harness may print its own text first.
                    let msg = if let Some(i) = line.find(RECORD_PREFIX) {
                        Msg::Record(line[i + RECORD_PREFIX.len()..].to_owned())
                    } else if let Some(i) = line.find(PHASE_PREFIX) {
                        Msg::Phase(line[i + PHASE_PREFIX.len()..].trim_end().to_owned())
                    } else {
                        continue;
                    };
                    if tx.send(msg).is_err() {
                        break;
                    }
                }
            });
        }
        Ok(Worker {
            child,
            stdin,
            rx,
            served: 0,
            phase: None,
        })
    }

    fn send(&mut self, line: &str) -> std::io::Result<()> {
        let stdin = self
            .stdin
            .as_mut()
            .ok_or_else(|| std::io::Error::from(std::io::ErrorKind::BrokenPipe))?;
        writeln!(stdin, "{line}")?;
        stdin.flush()
    }

    /// The next record within `timeout`, remembering the progress lines on the way.
    fn recv(&mut self, timeout: Duration) -> Reply {
        let deadline = Instant::now() + timeout;
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            match self.rx.recv_timeout(left) {
                Ok(Msg::Record(line)) => return Reply::Line(line),
                Ok(Msg::Phase(p)) => self.phase = Some(p),
                Err(RecvTimeoutError::Timeout) => return Reply::Timeout,
                Err(RecvTimeoutError::Disconnected) => return Reply::Died,
            }
        }
    }

    /// `" during <phase>"` of the last progress line, if any.
    fn during(&self) -> String {
        self.phase
            .as_deref()
            .and_then(|p| p.split_once(' '))
            .map(|(_, p)| format!(" during {p}"))
            .unwrap_or_default()
    }

    /// Kills this worker (its own PID) and reaps it.
    fn kill(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }

    /// The exit status of a worker whose stdout closed.
    fn status(&mut self) -> String {
        // The stdout pipe closes as the process ends; give it a moment to be reaped.
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            match self.child.try_wait() {
                Ok(Some(status)) => return status.to_string(),
                Ok(None) if Instant::now() < deadline => {
                    std::thread::sleep(Duration::from_millis(10));
                }
                _ => {
                    self.kill();
                    return "closed its output and was killed".to_owned();
                }
            }
        }
    }

    /// Ends the worker: EOF on stdin, then a short wait before killing it.
    fn close(mut self) {
        self.stdin = None;
        let _ = self.status();
    }
}

impl Drop for Worker {
    fn drop(&mut self) {
        if matches!(self.child.try_wait(), Ok(None)) {
            self.kill();
        }
    }
}

/// Runs every case with every strategy of `cfg` in worker processes started by `worker`
/// (module docs). Records come back in case order, strategies in `cfg` order within a case.
///
/// # Errors
/// `Spawn` when a worker cannot be started, `Protocol` for an unreadable answer.
pub fn run_isolated(
    cases: &[Case],
    cfg: &HarnessConfig,
    worker: &(dyn Fn() -> Command + Sync),
) -> Result<Vec<CaseRecord>, HarnessError> {
    let tasks: Vec<(usize, Strategy)> = (0..cases.len())
        .flat_map(|c| cfg.strategies.iter().map(move |&s| (c, s)))
        .collect();
    let next = AtomicUsize::new(0);
    let results: Mutex<Vec<Option<CaseRecord>>> = Mutex::new(vec![None; tasks.len()]);
    let jobs = cfg.jobs.clamp(1, tasks.len().max(1));
    let errors: Vec<HarnessError> = std::thread::scope(|scope| {
        let handles: Vec<_> = (0..jobs)
            .map(|_| {
                scope.spawn(|| -> Result<(), HarnessError> {
                    let mut current: Option<Worker> = None;
                    loop {
                        let i = next.fetch_add(1, Ordering::Relaxed);
                        let Some(&(c, strategy)) = tasks.get(i) else {
                            break;
                        };
                        let case = &cases[c];
                        let line = serde_json::to_string(&Request {
                            index: i,
                            strategy,
                            case: case.clone(),
                        })
                        .map_err(|e| HarnessError::Protocol(e.to_string()))?;
                        let rec = run_task(&mut current, worker, &line, i, case, strategy, cfg)?;
                        if let Ok(mut r) = results.lock() {
                            r[i] = Some(rec);
                        }
                        if current
                            .as_ref()
                            .is_some_and(|w| w.served >= cfg.cases_per_worker)
                            && let Some(w) = current.take()
                        {
                            w.close();
                        }
                    }
                    if let Some(w) = current.take() {
                        w.close();
                    }
                    Ok(())
                })
            })
            .collect();
        handles
            .into_iter()
            .filter_map(|h| match h.join() {
                Ok(Ok(())) => None,
                Ok(Err(e)) => Some(e),
                Err(_) => Some(HarnessError::Protocol("a job slot panicked".to_owned())),
            })
            .collect()
    });
    if let Some(e) = errors.into_iter().next() {
        return Err(e);
    }
    let results = results
        .into_inner()
        .map_err(|_| HarnessError::Protocol("result lock poisoned".to_owned()))?;
    results
        .into_iter()
        .map(|r| r.ok_or_else(|| HarnessError::Protocol("a task has no record".to_owned())))
        .collect()
}

#[allow(clippy::too_many_arguments, reason = "one task of the slot loop")]
fn run_task(
    current: &mut Option<Worker>,
    worker: &(dyn Fn() -> Command + Sync),
    line: &str,
    index: usize,
    case: &Case,
    strategy: Strategy,
    cfg: &HarnessConfig,
) -> Result<CaseRecord, HarnessError> {
    let mut resent = false;
    loop {
        if current.is_none() {
            *current = Some(Worker::spawn(worker())?);
        }
        let Some(w) = current.as_mut() else {
            continue;
        };
        if let Err(e) = w.send(line) {
            // The worker died between cases: start a new one and send the task again, once.
            w.kill();
            *current = None;
            if resent {
                return Ok(CaseRecord::abnormal(
                    case,
                    strategy,
                    Outcome::Crash,
                    format!("no worker would take the case: {e}"),
                    0.0,
                ));
            }
            resent = true;
            continue;
        }
        let start = Instant::now();
        let reply = w.recv(cfg.timeout);
        let ms = millis(start.elapsed());
        return match reply {
            Reply::Line(json) => {
                w.served += 1;
                let resp: Response = serde_json::from_str(&json)
                    .map_err(|e| HarnessError::Protocol(format!("{e}: {json}")))?;
                if resp.index != index {
                    return Err(HarnessError::Protocol(format!(
                        "answer for task {} while waiting for {index}",
                        resp.index
                    )));
                }
                Ok(resp.record)
            }
            Reply::Timeout => {
                let during = w.during();
                w.kill();
                *current = None;
                Ok(CaseRecord::abnormal(
                    case,
                    strategy,
                    Outcome::Timeout,
                    format!(
                        "no answer within {:?}{during}; the worker was killed",
                        cfg.timeout
                    ),
                    ms,
                ))
            }
            Reply::Died => {
                let status = w.status();
                let during = w.during();
                *current = None;
                Ok(CaseRecord::abnormal(
                    case,
                    strategy,
                    Outcome::Crash,
                    format!("the worker process ended inside the case ({status}){during}"),
                    ms,
                ))
            }
        };
    }
}

/// The cases having every tag of `tags` (all cases for none), the first `limit` of them.
pub fn select(cases: Vec<Case>, tags: &[String], limit: Option<usize>) -> Vec<Case> {
    cases
        .into_iter()
        .filter(|c| tags.iter().all(|t| c.has_tag(t)))
        .take(limit.unwrap_or(usize::MAX))
        .collect()
}

/// The `corpus` lines of a report: the source's roots and seed lists and the tag filter.
pub fn describe(source: &super::case::CorpusSource, tags: &[String]) -> Vec<String> {
    let mut out: Vec<String> = source
        .seeds
        .iter()
        .map(|p| format!("seeds: {}", p.display()))
        .chain(source.dirs.iter().map(|p| format!("dir: {}", p.display())))
        .collect();
    if !tags.is_empty() {
        out.push(format!("tags: {}", tags.join(", ")));
    }
    out
}

/// Runs every case with every strategy in this process (no isolation; a crash ends the run).
pub fn run_in_process(
    kernels: &Kernels,
    cases: &[Case],
    strategies: &[Strategy],
) -> Vec<CaseRecord> {
    cases
        .iter()
        .flat_map(|c| {
            strategies
                .iter()
                .map(move |&s| run_case(kernels.get(s), c, s))
        })
        .collect()
}

/// The nearest-rank percentile `q` ∈ (0, 1] of `sorted` (ascending); `None` when empty.
pub fn percentile(sorted: &[f64], q: f64) -> Option<f64> {
    if sorted.is_empty() {
        return None;
    }
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::cast_precision_loss,
        reason = "a rank within the slice length"
    )]
    let rank = ((q * sorted.len() as f64).ceil() as usize).clamp(1, sorted.len());
    Some(sorted[rank - 1])
}

/// Counts and rates over a set of records (robustness note §6).
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Aggregate {
    /// Records.
    pub cases: usize,
    /// Records expecting success.
    pub expect_success: usize,
    /// Valid `Ok`s among them.
    pub valid_ok: usize,
    /// `valid_ok / expect_success` (1 when there are none).
    pub success_rate: f64,
    /// Records expecting a clean failure.
    pub expect_clean_failure: usize,
    /// Classified `Err`s among them.
    pub clean_failures: usize,
    /// `clean_failures / expect_clean_failure` (1 when there are none).
    pub clean_failure_rate: f64,
    /// Clean failures with the expected failure class.
    pub class_matches: usize,
    /// Invalid `Ok`s (any expectation; target 0).
    pub invalid_ok: usize,
    /// Errors (classified or not) on cases expecting success.
    pub failed: usize,
    /// Valid `Ok`s on cases expecting a clean failure.
    pub unexpected_ok: usize,
    /// Setup errors.
    pub setup_errors: usize,
    /// Crashes.
    pub crashes: usize,
    /// Timeouts.
    pub timeouts: usize,
    /// Valid `Ok`s with a tolerance above [`TOL_CAP`].
    pub above_tol_cap: usize,
    /// Median of `|volume_dev|` over valid `Ok`s with an exact oracle.
    pub vol_dev_p50: Option<f64>,
    /// Largest `|volume_dev|` over valid `Ok`s with an exact oracle.
    pub vol_dev_max: Option<f64>,
    /// Median op time over `Ok`/`Err` records, ms.
    pub p50_ms: Option<f64>,
    /// 95th percentile op time, ms.
    pub p95_ms: Option<f64>,
}

impl Aggregate {
    /// The aggregate of `records`.
    pub fn of<'a>(records: impl IntoIterator<Item = &'a CaseRecord>) -> Aggregate {
        let mut a = Aggregate::default();
        let mut times = Vec::new();
        let mut devs = Vec::new();
        for r in records {
            a.cases += 1;
            let success = r.expect == ExpectOutcome::Success;
            if success {
                a.expect_success += 1;
            } else {
                a.expect_clean_failure += 1;
            }
            match r.verdict {
                Verdict::ValidOk if success => a.valid_ok += 1,
                Verdict::ValidOk => a.unexpected_ok += 1,
                Verdict::InvalidOk => a.invalid_ok += 1,
                Verdict::ClassifiedErr if !success => {
                    a.clean_failures += 1;
                    if r.class_matches == Some(true) {
                        a.class_matches += 1;
                    }
                }
                Verdict::ClassifiedErr | Verdict::UnclassifiedErr => {
                    if success {
                        a.failed += 1;
                    }
                }
                Verdict::SetupError => a.setup_errors += 1,
                Verdict::Crash => a.crashes += 1,
                Verdict::Timeout => a.timeouts += 1,
            }
            if r.verdict == Verdict::ValidOk {
                if r.max_tolerance.is_some_and(|t| t > TOL_CAP) {
                    a.above_tol_cap += 1;
                }
                if let Some(d) = r.volume_dev {
                    devs.push(d.abs());
                }
            }
            if matches!(r.outcome, Outcome::Ok | Outcome::Err) && r.verdict != Verdict::SetupError {
                times.push(r.millis);
            }
        }
        let rate = |n: usize, d: usize| {
            #[allow(clippy::cast_precision_loss, reason = "case counts")]
            if d == 0 { 1.0 } else { n as f64 / d as f64 }
        };
        a.success_rate = rate(a.valid_ok, a.expect_success);
        a.clean_failure_rate = rate(a.clean_failures, a.expect_clean_failure);
        times.sort_by(f64::total_cmp);
        devs.sort_by(f64::total_cmp);
        a.p50_ms = percentile(&times, 0.5);
        a.p95_ms = percentile(&times, 0.95);
        a.vol_dev_p50 = percentile(&devs, 0.5);
        a.vol_dev_max = devs.last().copied();
        a
    }

    /// A markdown table row: `| label | cases | valid Ok / expecting success (rate) | clean
    /// failures / expecting one | invalid Ok | crashes | timeouts | p50 ms | p95 ms |`.
    pub fn markdown_row(&self, label: &str) -> String {
        let ms = |v: Option<f64>| v.map_or_else(|| "–".to_owned(), |v| format!("{v:.2}"));
        format!(
            "| {label} | {} | {} / {} ({:.1} %) | {} / {} | {} | {} | {} | {} | {} |",
            self.cases,
            self.valid_ok,
            self.expect_success,
            100.0 * self.success_rate,
            self.clean_failures,
            self.expect_clean_failure,
            self.invalid_ok,
            self.crashes,
            self.timeouts,
            ms(self.p50_ms),
            ms(self.p95_ms),
        )
    }

    /// The header of [`Aggregate::markdown_row`] tables.
    pub const MARKDOWN_HEADER: &'static str = "| set | cases | valid Ok / expecting success | clean failures / expecting one | invalid Ok | crashes | timeouts | p50 ms | p95 ms |\n|---|---|---|---|---|---|---|---|---|";
}

/// The aggregates of one strategy.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct StrategySummary {
    /// The strategy.
    pub strategy: Strategy,
    /// Over every record.
    pub all: Aggregate,
    /// Per tag.
    pub by_tag: BTreeMap<String, Aggregate>,
}

/// The harness report (`--report out.json`, schema [`REPORT_SCHEMA`]).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Report {
    /// [`REPORT_SCHEMA`].
    pub schema: u32,
    /// When the report was made, RFC 3339 UTC.
    pub created_utc: String,
    /// `amg_brep::kernel_version()` of the parent.
    pub kernel_version: String,
    /// `robust::POLICY_VERSION`.
    pub policy_version: u32,
    /// What was run (corpus roots, seed lists, filters).
    pub corpus: Vec<String>,
    /// Per strategy.
    pub strategies: Vec<StrategySummary>,
    /// Every record.
    pub cases: Vec<CaseRecord>,
}

impl Report {
    /// The report of `records`.
    pub fn new(records: Vec<CaseRecord>, corpus: Vec<String>) -> Report {
        let mut strategies: Vec<Strategy> = records.iter().map(|r| r.strategy).collect();
        strategies.sort_unstable();
        strategies.dedup();
        let summaries = strategies
            .into_iter()
            .map(|s| {
                let mine: Vec<&CaseRecord> = records.iter().filter(|r| r.strategy == s).collect();
                let tags: BTreeSet<&str> = mine
                    .iter()
                    .flat_map(|r| r.tags.iter().map(String::as_str))
                    .collect();
                let by_tag = tags
                    .into_iter()
                    .map(|t| {
                        (
                            t.to_owned(),
                            Aggregate::of(mine.iter().copied().filter(|r| r.has_tag(t))),
                        )
                    })
                    .collect();
                StrategySummary {
                    strategy: s,
                    all: Aggregate::of(mine.iter().copied()),
                    by_tag,
                }
            })
            .collect();
        Report {
            schema: REPORT_SCHEMA,
            created_utc: crate::capture::time_rfc3339_now(),
            kernel_version: crate::kernel_version().to_owned(),
            policy_version: crate::robust::POLICY_VERSION,
            corpus,
            strategies: summaries,
            cases: records,
        }
    }

    /// The summary of `s`.
    pub fn summary(&self, s: Strategy) -> Option<&StrategySummary> {
        self.strategies.iter().find(|x| x.strategy == s)
    }

    /// A markdown table: one row per strategy, then one per strategy and tag of `tags`.
    pub fn markdown(&self, tags: &[&str]) -> String {
        let mut out = String::from(Aggregate::MARKDOWN_HEADER);
        for s in &self.strategies {
            out.push('\n');
            out.push_str(&s.all.markdown_row(&format!("all, {}", s.strategy.name())));
        }
        for s in &self.strategies {
            for t in tags {
                if let Some(a) = s.by_tag.get(*t) {
                    out.push('\n');
                    out.push_str(&a.markdown_row(&format!("`{t}`, {}", s.strategy.name())));
                }
            }
        }
        out
    }

    /// Writes the report as pretty JSON.
    ///
    /// # Errors
    /// I/O errors.
    pub fn write(&self, path: &Path) -> std::io::Result<()> {
        let json = serde_json::to_string_pretty(self)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
        std::fs::write(path, json)
    }
}

/// The passing case ids of one strategy.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BaselineEntry {
    /// How many cases the baseline run had.
    pub cases: usize,
    /// The ids that passed, sorted.
    pub passing: Vec<String>,
}

/// `tests/golden/kernel/<corpus>-baseline.toml`: the regression gate (design note §6).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Baseline {
    /// [`BASELINE_SCHEMA`].
    pub schema: u32,
    /// The kernel version of the baseline run (informational).
    #[serde(default)]
    pub kernel_version: String,
    /// Per strategy name.
    #[serde(default)]
    pub strategy: BTreeMap<String, BaselineEntry>,
}

/// A baseline-passing case that no longer passes.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Regression {
    /// The case id.
    pub id: String,
    /// The strategy.
    pub strategy: Strategy,
    /// What it gave now.
    pub verdict: Verdict,
    /// The error or problems.
    pub message: String,
}

impl Baseline {
    /// The baseline of `records`: per strategy, the ids that passed.
    pub fn from_records(records: &[CaseRecord]) -> Baseline {
        let mut strategy: BTreeMap<String, BaselineEntry> = BTreeMap::new();
        for r in records {
            let e = strategy.entry(r.strategy.name().to_owned()).or_default();
            e.cases += 1;
            if r.passed {
                e.passing.push(r.id.clone());
            }
        }
        for e in strategy.values_mut() {
            e.passing.sort();
        }
        Baseline {
            schema: BASELINE_SCHEMA,
            kernel_version: crate::kernel_version().to_owned(),
            strategy,
        }
    }

    /// Reads a baseline file.
    ///
    /// # Errors
    /// `Io` or `Schema`.
    pub fn load(path: &Path) -> Result<Baseline, CaseError> {
        let text = std::fs::read_to_string(path).map_err(|source| CaseError::Io {
            path: path.to_path_buf(),
            source,
        })?;
        let b: Baseline = toml::from_str(&text).map_err(|e| CaseError::Schema {
            path: path.to_path_buf(),
            message: e.to_string(),
        })?;
        if b.schema != BASELINE_SCHEMA {
            return Err(CaseError::Schema {
                path: path.to_path_buf(),
                message: format!("schema {} (this build reads {BASELINE_SCHEMA})", b.schema),
            });
        }
        Ok(b)
    }

    /// The TOML text, with a comment header.
    ///
    /// # Errors
    /// Serialisation errors (none for a well-formed baseline).
    pub fn to_toml(&self) -> Result<String, toml::ser::Error> {
        let quote = |s: &str| toml::Value::String(s.to_owned()).to_string();
        let mut out = String::from(
            "# KH-02 regression baseline: the case ids each strategy passes (valid Ok where success\n\
             # is expected, a classified Err where a clean failure is). Regenerate after a run on the\n\
             # full corpus with\n\
             # `cargo run -p amg-brep --features occt --example kernel-harness -- run --write-baseline <this file>`\n\
             # (or the ignored test `kernel_corpus::full_fillet_corpus` with AMG_HARNESS_WRITE_BASELINE=1).\n\
             # Never drop an id to make a regression pass.\n",
        );
        out.push_str(&format!(
            "schema = {}\nkernel_version = {}\n",
            self.schema,
            quote(&self.kernel_version)
        ));
        for (name, e) in &self.strategy {
            out.push_str(&format!(
                "\n[strategy.{}]\ncases = {}\npassing = [\n",
                name, e.cases
            ));
            for id in &e.passing {
                out.push_str(&format!("    {},\n", quote(id)));
            }
            out.push_str("]\n");
        }
        // Self-check: the text parses back to this baseline.
        let back: Baseline = toml::from_str(&out).map_err(|e| {
            <toml::ser::Error as serde::ser::Error>::custom(format!("baseline round trip: {e}"))
        })?;
        if back != *self {
            return Err(<toml::ser::Error as serde::ser::Error>::custom(
                "baseline round trip changed the content",
            ));
        }
        Ok(out)
    }

    /// The ids `s` passes in the baseline.
    pub fn passing(&self, s: Strategy) -> BTreeSet<&str> {
        self.strategy
            .get(s.name())
            .map(|e| e.passing.iter().map(String::as_str).collect())
            .unwrap_or_default()
    }

    /// Every record whose case passed in the baseline for its strategy and does not pass now.
    /// Cases the run did not include are not regressions (subset runs).
    pub fn regressions(&self, records: &[CaseRecord]) -> Vec<Regression> {
        records
            .iter()
            .filter(|r| !r.passed && self.passing(r.strategy).contains(r.id.as_str()))
            .map(|r| Regression {
                id: r.id.clone(),
                strategy: r.strategy,
                verdict: r.verdict,
                message: r.message.clone().unwrap_or_else(|| r.problems.join("; ")),
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robust::corpus::case::{CaseInput, Expect};
    use crate::robust::corpus::fillet::{FilletGenCase, FilletGenerator};

    fn case(id: &str) -> Case {
        Case {
            id: id.to_owned(),
            source: "synthetic".to_owned(),
            tags: vec!["t".to_owned()],
            expect: Expect::success(),
            input: CaseInput::Fillet(FilletGenCase {
                generator: FilletGenerator::PrismVerticalEdges,
                seed: 0,
            }),
            inject: None,
        }
    }

    fn rec(id: &str, verdict: Verdict, expect: ExpectOutcome, ms: f64) -> CaseRecord {
        let mut r = CaseRecord::new(&case(id), Strategy::Raw, expect);
        r.verdict = verdict;
        r.outcome = match verdict {
            Verdict::ValidOk | Verdict::InvalidOk => Outcome::Ok,
            Verdict::Crash => Outcome::Crash,
            Verdict::Timeout => Outcome::Timeout,
            _ => Outcome::Err,
        };
        r.passed = passes(expect, verdict);
        r.millis = ms;
        r
    }

    #[test]
    fn percentiles_are_nearest_rank() {
        let v: Vec<f64> = (1..=20).map(f64::from).collect();
        assert_eq!(percentile(&v, 0.5), Some(10.0));
        assert_eq!(percentile(&v, 0.95), Some(19.0));
        assert_eq!(percentile(&v[..1], 0.95), Some(1.0));
        assert_eq!(percentile(&[], 0.5), None);
    }

    #[test]
    fn aggregates_count_each_verdict() {
        let s = ExpectOutcome::Success;
        let f = ExpectOutcome::CleanFailure;
        let records = vec![
            rec("a", Verdict::ValidOk, s, 1.0),
            rec("b", Verdict::InvalidOk, s, 2.0),
            rec("c", Verdict::ClassifiedErr, s, 3.0),
            rec("d", Verdict::ClassifiedErr, f, 4.0),
            rec("e", Verdict::Crash, s, 100.0),
            rec("f", Verdict::Timeout, f, 100.0),
            rec("g", Verdict::ValidOk, f, 5.0),
        ];
        let a = Aggregate::of(&records);
        assert_eq!(a.cases, 7);
        assert_eq!((a.expect_success, a.valid_ok), (4, 1));
        assert_eq!((a.expect_clean_failure, a.clean_failures), (3, 1));
        assert_eq!((a.invalid_ok, a.failed, a.unexpected_ok), (1, 1, 1));
        assert_eq!((a.crashes, a.timeouts), (1, 1));
        assert!((a.success_rate - 0.25).abs() < 1e-12);
        // Crash and timeout times are not op times.
        assert_eq!(a.p50_ms, Some(3.0));
        assert_eq!(a.p95_ms, Some(5.0));
        assert!(
            a.markdown_row("x")
                .starts_with("| x | 7 | 1 / 4 (25.0 %) | 1 / 3 | 1 | 1 | 1 |")
        );
    }

    #[test]
    fn baselines_find_regressions_only_for_passing_cases() {
        let s = ExpectOutcome::Success;
        let before = vec![
            rec("a", Verdict::ValidOk, s, 1.0),
            rec("b", Verdict::ClassifiedErr, s, 1.0),
        ];
        let base = Baseline::from_records(&before);
        assert_eq!(
            base.passing(Strategy::Raw).into_iter().collect::<Vec<_>>(),
            ["a"]
        );
        let text = base.to_toml().unwrap();
        let back: Baseline = toml::from_str(&text).unwrap();
        assert_eq!(back, base);
        let after = vec![
            rec("a", Verdict::Crash, s, 1.0),
            rec("b", Verdict::ClassifiedErr, s, 1.0),
            rec("z", Verdict::ClassifiedErr, s, 1.0),
        ];
        let reg = base.regressions(&after);
        assert_eq!(reg.len(), 1);
        assert_eq!((reg[0].id.as_str(), reg[0].verdict), ("a", Verdict::Crash));
        assert!(base.regressions(&before).is_empty());
    }

    #[test]
    fn reports_round_trip_through_json() {
        let records = vec![
            rec("a", Verdict::ValidOk, ExpectOutcome::Success, 1.0),
            rec(
                "b",
                Verdict::ClassifiedErr,
                ExpectOutcome::CleanFailure,
                2.0,
            ),
        ];
        let r = Report::new(records, vec!["test".to_owned()]);
        assert_eq!(r.schema, REPORT_SCHEMA);
        let s = r.summary(Strategy::Raw).unwrap();
        assert_eq!(s.all.cases, 2);
        assert_eq!(s.by_tag["t"].cases, 2);
        let json = serde_json::to_string(&r).unwrap();
        assert_eq!(serde_json::from_str::<Report>(&json).unwrap(), r);
        assert!(r.markdown(&["t"]).lines().count() == 4);
    }

    #[test]
    fn the_worker_loop_answers_each_request() {
        // Without OCCT every case is a setup error; with it, a real fillet. Either way one
        // record per request, with the index echoed.
        let kernels = Kernels::occt();
        let mut input = String::new();
        for (i, inject) in [None, Some(Inject::NotDone), Some(Inject::Panic)]
            .into_iter()
            .enumerate()
        {
            let mut c = case(&format!("c{i}"));
            c.inject = inject;
            let req = Request {
                index: i,
                strategy: Strategy::Raw,
                case: c,
            };
            input.push_str(&serde_json::to_string(&req).unwrap());
            input.push('\n');
        }
        let mut out = Vec::new();
        assert_eq!(serve(&kernels, input.as_bytes(), &mut out).unwrap(), 3);
        let text = String::from_utf8(out).unwrap();
        let resp: Vec<Response> = text
            .lines()
            .filter_map(|l| l.strip_prefix(RECORD_PREFIX))
            .map(|l| serde_json::from_str(l).unwrap())
            .collect();
        assert_eq!(resp.iter().map(|r| r.index).collect::<Vec<_>>(), [0, 1, 2]);
        // Injected faults replace the case: they fire with or without a backend.
        assert_eq!(resp[1].record.verdict, Verdict::ClassifiedErr);
        assert_eq!(resp[1].record.failure_class, Some(FailureClass::NotDone));
        assert_eq!(resp[2].record.verdict, Verdict::UnclassifiedErr);
        assert_eq!(resp[2].record.failure_class, Some(FailureClass::Panic));
        // `prepare` + `op` for the plain case (only `prepare` without OCCT, whose build fails),
        // `op` for each injected one.
        let phases = if crate::OCCT_BACKEND { 4 } else { 3 };
        assert_eq!(text.matches(PHASE_PREFIX).count(), phases, "{text}");
        let plain = if crate::OCCT_BACKEND {
            Verdict::ValidOk
        } else {
            Verdict::SetupError
        };
        assert_eq!(resp[0].record.verdict, plain, "{:?}", resp[0].record);
    }
}
