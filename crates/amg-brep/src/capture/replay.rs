//! Replaying dumps (in process or in a child process) and promoting them to corpus cases.

use std::fs;
use std::io::BufReader;
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::Instant;

use amg_jobs::JobCtx;

use crate::capture::info::{FailureClass, FailureInfo};
use crate::capture::recorder::{Manifest, read_manifest};
use crate::capture::{CaptureError, op::SubShapeGeom};
use crate::check::CheckOpts;
use crate::kernel::BrepKernel;
use crate::shape::{Shape, SubShape};

/// How [`replay`] runs a dump.
///
/// KH-17 adds `robust: Option<robust::RetryPolicy>` (replay through the strategy chain).
#[derive(Clone, Debug, Default)]
#[non_exhaustive]
pub struct ReplayOpts {
    /// Count a result that fails `check` as an `InvalidResult` failure (as the validation
    /// gate does) instead of `NowSucceeds { valid: false }`.
    pub validate: bool,
}

impl ReplayOpts {
    /// Options with `validate` set.
    pub fn validating() -> ReplayOpts {
        ReplayOpts { validate: true }
    }
}

/// What a replay found.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum ReplayOutcome {
    /// The op failed again with the recorded class.
    Reproduced(FailureInfo),
    /// The op failed with another class.
    DifferentFailure(FailureInfo),
    /// The op returned a result; `valid` is its `check` verdict.
    NowSucceeds {
        /// Whether every result shape passes `check`.
        valid: bool,
    },
}

/// The result of replaying one dump.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ReplayReport {
    /// The dump directory.
    pub dump: PathBuf,
    /// The outcome.
    pub outcome: ReplayOutcome,
    /// Wall time of the replayed op, ms.
    pub millis: u64,
    /// Whether an edge argument was found again by its fingerprint because its index named
    /// another edge in the reloaded input.
    pub reindexed_by_fingerprint: bool,
}

/// Loads the dump's inputs.
fn load_inputs(dump_dir: &Path, m: &Manifest) -> Result<Vec<Shape>, CaptureError> {
    m.inputs
        .iter()
        .map(|file| {
            if Path::new(file).is_absolute() || file.contains("..") {
                return Err(CaptureError::Schema(format!("input path `{file}`")));
            }
            let f = fs::File::open(dump_dir.join(file))?;
            Ok(Shape::read_text(BufReader::new(f))?)
        })
        .collect()
}

fn classify(recorded: FailureClass, info: FailureInfo) -> ReplayOutcome {
    if info.class == recorded {
        ReplayOutcome::Reproduced(info)
    } else {
        ReplayOutcome::DifferentFailure(info)
    }
}

/// Re-runs the dump in `dump_dir` on `k` and classifies the result against the recorded
/// failure class. A panic in the op is caught and reported as a `Panic` failure. A hard crash
/// ends the process; use [`replay_in_child`] for dumps that may crash.
///
/// # Errors
/// `Io`, `Schema` or `Version` for an unreadable dump, `Kernel` when an input does not load,
/// `Unmatched` when an edge argument cannot be found again.
pub fn replay(
    dump_dir: &Path,
    k: &dyn BrepKernel,
    o: &ReplayOpts,
) -> Result<ReplayReport, CaptureError> {
    let m = read_manifest(dump_dir)?;
    let inputs = load_inputs(dump_dir, &m)?;
    let mut op = m.op.clone();
    let reindexed = op.reindex(&inputs)?;
    let ctx = JobCtx::detached();
    let start = Instant::now();
    let result = catch_unwind(AssertUnwindSafe(|| op.run(k, &inputs, &ctx)));
    let millis = u64::try_from(start.elapsed().as_millis()).unwrap_or(u64::MAX);
    let recorded = m.error.class;
    let outcome = match result {
        Err(payload) => {
            let msg = payload
                .downcast_ref::<&str>()
                .map(|s| (*s).to_owned())
                .or_else(|| payload.downcast_ref::<String>().cloned())
                .unwrap_or_default();
            classify(recorded, FailureInfo::new(FailureClass::Panic, msg))
        }
        Ok(Err(e)) => classify(recorded, FailureInfo::from_kernel_error(&e, inputs.first())),
        Ok(Ok(shapes)) => {
            let reports: Vec<_> = shapes
                .iter()
                .map(|s| k.check(s, &CheckOpts::default()))
                .collect();
            let valid = reports.iter().all(|r| r.valid);
            if valid || !o.validate {
                ReplayOutcome::NowSucceeds { valid }
            } else {
                let mut info = FailureInfo::new(
                    FailureClass::InvalidResult,
                    format!(
                        "{} produced an invalid shape: {:?}",
                        m.op.kind(),
                        reports
                            .iter()
                            .flat_map(|r| r.problem_kinds())
                            .collect::<Vec<_>>()
                    ),
                );
                if let (Some(s), Some(r)) = (shapes.first(), reports.first()) {
                    info.faulty = r
                        .problems
                        .iter()
                        .filter_map(|(sub, _)| *sub)
                        .map(|sub: SubShape| (sub, SubShapeGeom::of(s, sub)))
                        .collect();
                }
                classify(recorded, info)
            }
        }
    };
    Ok(ReplayReport {
        dump: dump_dir.to_path_buf(),
        outcome,
        millis,
        reindexed_by_fingerprint: reindexed,
    })
}

/// The line prefix of a report printed by a replay child process.
pub const REPORT_PREFIX: &str = "AMG_KERNEL_REPLAY_REPORT ";

/// The line a replay child prints on stdout for [`replay_in_child`] to read.
pub fn child_report_line(r: &ReplayReport) -> String {
    let json = serde_json::to_string(r).unwrap_or_else(|_| String::from("{}"));
    format!("{REPORT_PREFIX}{json}")
}

/// Runs `child`, a process that replays `dump_dir` and prints [`child_report_line`] on
/// stdout, and returns its report. A child that dies without a report (an access violation,
/// an abort, a stack overflow) gives a `Crash` failure, `Reproduced` when the dump records a
/// crash. This is how a harness survives a crashing dump (design note §6).
///
/// # Errors
/// `Io` when the child cannot start, `Child` when it exits normally without a report, and the
/// errors of [`read_manifest`] for the dump.
pub fn replay_in_child(dump_dir: &Path, mut child: Command) -> Result<ReplayReport, CaptureError> {
    let m = read_manifest(dump_dir)?;
    let start = Instant::now();
    let out = child.output()?;
    let millis = u64::try_from(start.elapsed().as_millis()).unwrap_or(u64::MAX);
    let stdout = String::from_utf8_lossy(&out.stdout);
    if let Some(json) = stdout
        .lines()
        .rev()
        // Anywhere in the line: a test harness may print its own text before the report.
        .find_map(|l| l.find(REPORT_PREFIX).map(|i| l[i + REPORT_PREFIX.len()..].trim_end()))
    {
        return serde_json::from_str(json).map_err(|e| CaptureError::Child(e.to_string()));
    }
    if out.status.success() {
        return Err(CaptureError::Child(format!(
            "exited normally without a report; stdout: {}; stderr: {}",
            stdout.trim(),
            String::from_utf8_lossy(&out.stderr).trim()
        )));
    }
    let info = FailureInfo::new(
        FailureClass::Crash,
        format!("the replay process ended without a report ({})", out.status),
    );
    Ok(ReplayReport {
        dump: dump_dir.to_path_buf(),
        outcome: classify(m.error.class, info),
        millis,
        reindexed_by_fingerprint: false,
    })
}

/// Whether `id` is `component(/component)*` with components of `[A-Za-z0-9._-]+`, none of them
/// `.` or `..`.
fn valid_id(id: &str) -> bool {
    !id.is_empty()
        && id.split('/').all(|c| {
            !c.is_empty()
                && c != "."
                && c != ".."
                && c.chars()
                    .all(|ch| ch.is_ascii_alphanumeric() || matches!(ch, '.' | '_' | '-'))
        })
}

fn class_name(class: FailureClass) -> String {
    serde_json::to_string(&class)
        .unwrap_or_default()
        .trim_matches('"')
        .to_owned()
}

/// Turns a dump into a corpus case directory `<corpus_dir>/<id>/` with `case.toml` (design
/// note §5.2), `op.json` (the `KernelOp`, `InputRef(n)` = `inputs/<n>.brep`) and `inputs/`.
/// `expect` comes from a replay on `k` with validation: `clean_failure` with the failure
/// class when the failure reproduces or changes, `success` when the op now succeeds.
/// Returns the case directory.
///
/// # Errors
/// `InvalidId` for a malformed `id`; `Io` when the case directory exists or cannot be written;
/// the errors of [`replay`].
pub fn promote(
    dump_dir: &Path,
    corpus_dir: &Path,
    id: &str,
    k: &dyn BrepKernel,
) -> Result<PathBuf, CaptureError> {
    if !valid_id(id) {
        return Err(CaptureError::InvalidId(id.to_owned()));
    }
    let m = read_manifest(dump_dir)?;
    let report = replay(dump_dir, k, &ReplayOpts::validating())?;
    let case = id
        .split('/')
        .fold(corpus_dir.to_path_buf(), |p, c| p.join(c));
    if case.exists() {
        return Err(CaptureError::Io(std::io::Error::new(
            std::io::ErrorKind::AlreadyExists,
            format!("{} exists", case.display()),
        )));
    }
    fs::create_dir_all(case.join("inputs"))?;
    for file in &m.inputs {
        fs::copy(dump_dir.join(file), case.join(file))?;
    }
    let op =
        serde_json::to_string_pretty(&m.op).map_err(|e| CaptureError::Schema(e.to_string()))?;
    fs::write(case.join("op.json"), op)?;
    let expect = match &report.outcome {
        ReplayOutcome::Reproduced(info) | ReplayOutcome::DifferentFailure(info) => format!(
            "outcome = \"clean_failure\"\nfailure_class = \"{}\"\n",
            class_name(info.class)
        ),
        ReplayOutcome::NowSucceeds { .. } => "outcome = \"success\"\n".to_owned(),
    };
    let toml = format!(
        "schema = 1\n\
         id = \"{id}\"\n\
         op = \"op.json\"\n\
         source = \"promoted:{hash}\"\n\
         licence = \"LicenseRef-Amalgam-Personal\"\n\
         tags = [\"promoted\", \"{kind}\"]\n\
         recorded_class = \"{recorded}\"\n\
         kernel_version = \"{kv}\"\n\
         \n[expect]\n{expect}",
        hash = m.hash,
        kind = m.op.kind(),
        recorded = class_name(m.error.class),
        kv = m.kernel_version.replace('"', "'"),
    );
    fs::write(case.join("case.toml"), toml)?;
    Ok(case)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn corpus_ids() {
        assert!(valid_id("fillet/promoted/bracket-07"));
        assert!(valid_id("a.b_c"));
        for bad in [
            "", "/a", "a/", "a//b", "../a", "a/./b", "a b", "a\\b", "c:x",
        ] {
            assert!(!valid_id(bad), "{bad}");
        }
    }

    #[test]
    fn class_names_are_serde_names() {
        assert_eq!(class_name(FailureClass::NotDone), "NotDone");
        assert_eq!(class_name(FailureClass::InvalidInput), "InvalidInput");
    }
}
