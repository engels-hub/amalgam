//! `kernel-harness`: runs a kernel corpus in worker processes and reports pass rates (S20
//! KH-02, design note `kernel-robustness.md` §6).
//!
//! Stands in for `amg-cli kernel harness` (which wraps the same `robust::corpus::harness`
//! calls) so the harness runs from the `amg-brep` crate alone.
//!
//! ```text
//! kernel-harness run [--corpus <dir>]... [--seeds <seeds.toml>]... [--tag <tag>]...
//!                    [--limit N] [--strategy raw|hardened|all] [--jobs N] [--timeout-s S]
//!                    [--report out.json] [--baseline <file>] [--write-baseline <file>]
//! kernel-harness list [--corpus <dir>]... [--seeds <seeds.toml>]... [--tag <tag>]... [--id <s>]...
//! kernel-harness show (the same filters: the case JSON and, for synthetic cases, the drawn part)
//! ```
//!
//! Without `--corpus`/`--seeds` it runs the committed fillet corpus
//! (`tests/golden/kernel/fillet/{seeds.toml, handmade/}` plus
//! `$AMG_TESTDATA_DIR/kernel/fillet/{owner,harvested}` when set). Every case runs in a child
//! process (this program with the hidden `worker` command), so an OCCT access violation is a
//! `Crash` record and the run goes on. Prints the summary table (markdown, as in
//! `docs/notes/kernel-metrics.md`) on stdout; `--baseline` exits with 1 when a baseline-passing
//! case fails.
//!
//! The full fillet run:
//! `cargo run -p amg-brep --features occt --release --example kernel-harness -- run --report fillet.json`.
#![allow(
    clippy::print_stdout,
    clippy::print_stderr,
    reason = "command-line tool"
)]

use std::path::PathBuf;
use std::process::{Command, ExitCode};
use std::time::Duration;

use amg_brep::robust::corpus::case::{CaseInput, CorpusSource};
use amg_brep::robust::corpus::harness::{
    Baseline, HarnessConfig, Report, Strategy, describe, run_isolated, select, serve_stdio,
};

const USAGE: &str = "usage:
  kernel-harness run  [--corpus <dir>]... [--seeds <file>]... [--tag <tag>]... [--id <substring>]... [--limit N]
                      [--strategy raw|hardened|all] [--jobs N] [--timeout-s S]
                      [--report out.json] [--baseline <file>] [--write-baseline <file>]
  kernel-harness list [--corpus <dir>]... [--seeds <file>]... [--tag <tag>]... [--id <substring>]...
  kernel-harness show [--corpus <dir>]... [--seeds <file>]... [--tag <tag>]... [--id <substring>]...";

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match run(&args) {
        Ok(code) => code,
        Err(e) => {
            eprintln!("kernel-harness: {e}");
            ExitCode::from(2)
        }
    }
}

#[derive(Default)]
struct Opts {
    source: CorpusSource,
    tags: Vec<String>,
    ids: Vec<String>,
    limit: Option<usize>,
    cfg: HarnessConfig,
    report: Option<PathBuf>,
    baseline: Option<PathBuf>,
    write_baseline: Option<PathBuf>,
}

fn parse(args: &[String]) -> Result<Opts, String> {
    let mut o = Opts::default();
    let mut it = args.iter();
    while let Some(a) = it.next() {
        let mut value = || {
            it.next()
                .cloned()
                .ok_or_else(|| format!("{a} needs a value"))
        };
        match a.as_str() {
            "--corpus" => o.source.dirs.push(PathBuf::from(value()?)),
            "--seeds" => o.source.seeds.push(PathBuf::from(value()?)),
            "--tag" => o.tags.push(value()?),
            "--id" => o.ids.push(value()?),
            "--limit" => o.limit = Some(value()?.parse().map_err(|e| format!("--limit: {e}"))?),
            "--jobs" => o.cfg.jobs = value()?.parse().map_err(|e| format!("--jobs: {e}"))?,
            "--timeout-s" => {
                let s: f64 = value()?.parse().map_err(|e| format!("--timeout-s: {e}"))?;
                o.cfg.timeout = Duration::from_secs_f64(s);
            }
            "--strategy" => {
                let v = value()?;
                o.cfg.strategies = if v == "all" {
                    Strategy::ALL.to_vec()
                } else {
                    vec![Strategy::from_name(&v).ok_or_else(|| format!("unknown strategy {v}"))?]
                };
            }
            "--report" => o.report = Some(PathBuf::from(value()?)),
            "--baseline" => o.baseline = Some(PathBuf::from(value()?)),
            "--write-baseline" => o.write_baseline = Some(PathBuf::from(value()?)),
            other => return Err(format!("unknown argument {other}\n{USAGE}")),
        }
    }
    if o.source.dirs.is_empty() && o.source.seeds.is_empty() {
        o.source = CorpusSource::fillet();
    }
    Ok(o)
}

fn run(args: &[String]) -> Result<ExitCode, String> {
    let Some((cmd, rest)) = args.split_first() else {
        eprintln!("{USAGE}");
        return Ok(ExitCode::from(2));
    };
    match cmd.as_str() {
        "worker" => {
            serve_stdio().map_err(|e| e.to_string())?;
            Ok(ExitCode::SUCCESS)
        }
        "list" => {
            let o = parse(rest)?;
            let cases = cases(&o)?;
            for c in &cases {
                println!("{}\t{}", c.id, c.tags.join(","));
            }
            eprintln!("{} cases", cases.len());
            Ok(ExitCode::SUCCESS)
        }
        "show" => {
            // The case as the harness sends it, plus the generator's drawn part (to reproduce a
            // synthetic case outside the harness, for example for an upstream report).
            let o = parse(rest)?;
            for c in &cases(&o)? {
                println!(
                    "{}",
                    serde_json::to_string_pretty(c).map_err(|e| e.to_string())?
                );
                if let CaseInput::Fillet(g) = &c.input {
                    println!("{:#?}", g.part());
                }
            }
            Ok(ExitCode::SUCCESS)
        }
        "run" => run_cmd(&parse(rest)?),
        _ => {
            eprintln!("{USAGE}");
            Ok(ExitCode::from(2))
        }
    }
}

fn run_cmd(o: &Opts) -> Result<ExitCode, String> {
    let cases = cases(o)?;
    let exe = std::env::current_exe().map_err(|e| e.to_string())?;
    let worker = move || {
        let mut c = Command::new(&exe);
        c.arg("worker");
        c
    };
    eprintln!(
        "running {} cases × {} strategies in {} worker slots",
        cases.len(),
        o.cfg.strategies.len(),
        o.cfg.jobs
    );
    let records = run_isolated(&cases, &o.cfg, &worker).map_err(|e| e.to_string())?;
    let report = Report::new(records, describe(&o.source, &o.tags));
    println!(
        "{}",
        report.markdown(&[
            "synthetic",
            "handmade",
            "smoke",
            "prism_vertical_edges",
            "prism_mixed_edges",
            "boolean_stack",
            "radius_too_large",
            "corner3",
            "concave",
        ])
    );
    if let Some(path) = &o.report {
        report.write(path).map_err(|e| e.to_string())?;
        eprintln!("report: {}", path.display());
    }
    if let Some(path) = &o.write_baseline {
        let text = Baseline::from_records(&report.cases)
            .to_toml()
            .map_err(|e| e.to_string())?;
        std::fs::write(path, text).map_err(|e| e.to_string())?;
        eprintln!("baseline: {}", path.display());
    }
    if let Some(path) = &o.baseline {
        let base = Baseline::load(path).map_err(|e| e.to_string())?;
        let regressions = base.regressions(&report.cases);
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
        eprintln!("no regressions against {}", path.display());
    }
    Ok(ExitCode::SUCCESS)
}

/// The selected cases: tags, then `--id` substrings (any), then the limit.
fn cases(o: &Opts) -> Result<Vec<amg_brep::robust::corpus::case::Case>, String> {
    let all = select(o.source.load().map_err(|e| e.to_string())?, &o.tags, None);
    let by_id = all
        .into_iter()
        .filter(|c| o.ids.is_empty() || o.ids.iter().any(|i| c.id.contains(i.as_str())));
    Ok(by_id.take(o.limit.unwrap_or(usize::MAX)).collect())
}
