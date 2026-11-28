//! `amg-cli kernel harness`: a thin wrapper over `amg_brep::robust::corpus::harness` (S20 KH-02,
//! design note `kernel-robustness.md` §6). The `kernel-harness` example of `amg-brep` offers
//! the same run.

use std::path::PathBuf;
use std::process::{Command, ExitCode};
use std::time::Duration;

use amg_brep::robust::corpus::case::CorpusSource;
use amg_brep::robust::corpus::harness::{
    Baseline, HarnessConfig, Report, Strategy, describe, run_isolated, select, serve_stdio,
};
use clap::{Args, Subcommand};

#[derive(Debug, Subcommand)]
pub(crate) enum KernelCommand {
    /// Run a kernel corpus in worker processes (an OCCT crash is a `Crash` record, not the end
    /// of the run) and print the summary; with no `--corpus`/`--seeds`, the committed fillet
    /// corpus. Needs a build with `--features occt`.
    Harness(Box<HarnessArgs>),
    /// The worker side of `harness` (reads requests on stdin).
    #[command(hide = true)]
    HarnessWorker,
}

#[derive(Debug, Args)]
pub(crate) struct HarnessArgs {
    /// A directory searched for case directories (`case.toml`); repeatable.
    #[arg(long)]
    corpus: Vec<PathBuf>,
    /// A `seeds.toml` seed list; repeatable.
    #[arg(long)]
    seeds: Vec<PathBuf>,
    /// Only cases with this tag (all given tags); `smoke` is the CI subset.
    #[arg(long)]
    tag: Vec<String>,
    /// Only the first N selected cases.
    #[arg(long)]
    limit: Option<usize>,
    /// `raw`, `hardened` or `all`.
    #[arg(long, default_value = "raw")]
    strategy: String,
    /// Worker processes at once (default: up to 4).
    #[arg(long)]
    jobs: Option<usize>,
    /// Per-case timeout, seconds.
    #[arg(long, default_value_t = 60.0)]
    timeout_s: f64,
    /// Write the report JSON (schema 1) here.
    #[arg(long)]
    report: Option<PathBuf>,
    /// Compare with this baseline; exit code 1 on a regression.
    #[arg(long)]
    baseline: Option<PathBuf>,
    /// Write the passing ids of this run as a new baseline here.
    #[arg(long)]
    write_baseline: Option<PathBuf>,
}

/// `kernel harness-worker`.
pub(crate) fn worker() -> ExitCode {
    match serve_stdio() {
        Ok(_) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("amg-cli kernel harness-worker: {e}");
            ExitCode::from(2)
        }
    }
}

pub(crate) fn run(cmd: KernelCommand) -> ExitCode {
    match cmd {
        KernelCommand::HarnessWorker => worker(),
        KernelCommand::Harness(args) => match harness(&args) {
            Ok(code) => code,
            Err(e) => {
                eprintln!("amg-cli kernel harness: {e}");
                ExitCode::from(2)
            }
        },
    }
}

fn harness(a: &HarnessArgs) -> Result<ExitCode, String> {
    let mut source = CorpusSource {
        dirs: a.corpus.clone(),
        seeds: a.seeds.clone(),
    };
    if source.dirs.is_empty() && source.seeds.is_empty() {
        source = CorpusSource::fillet();
    }
    let strategies = if a.strategy == "all" {
        Strategy::ALL.to_vec()
    } else {
        vec![
            Strategy::from_name(&a.strategy)
                .ok_or_else(|| format!("unknown strategy {}", a.strategy))?,
        ]
    };
    let mut cfg = HarnessConfig {
        timeout: Duration::from_secs_f64(a.timeout_s),
        strategies,
        ..HarnessConfig::default()
    };
    if let Some(j) = a.jobs {
        cfg.jobs = j;
    }
    let cases = select(source.load().map_err(|e| e.to_string())?, &a.tag, a.limit);
    let exe = std::env::current_exe().map_err(|e| e.to_string())?;
    let worker = move || {
        let mut c = Command::new(&exe);
        c.args(["kernel", "harness-worker"]);
        c
    };
    let records = run_isolated(&cases, &cfg, &worker).map_err(|e| e.to_string())?;
    let report = Report::new(records, describe(&source, &a.tag));
    println!("{}", report.markdown(&["synthetic", "handmade", "smoke"]));
    if let Some(path) = &a.report {
        report.write(path).map_err(|e| e.to_string())?;
    }
    if let Some(path) = &a.write_baseline {
        let text = Baseline::from_records(&report.cases)
            .to_toml()
            .map_err(|e| e.to_string())?;
        std::fs::write(path, text).map_err(|e| e.to_string())?;
    }
    if let Some(path) = &a.baseline {
        let regressions = Baseline::load(path)
            .map_err(|e| e.to_string())?
            .regressions(&report.cases);
        for r in &regressions {
            eprintln!(
                "REGRESSION {} ({}): {:?} {}",
                r.id,
                r.strategy.name(),
                r.verdict,
                r.message
            );
        }
        if !regressions.is_empty() {
            return Ok(ExitCode::from(1));
        }
    }
    Ok(ExitCode::SUCCESS)
}
