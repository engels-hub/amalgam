//! `kernel-replay`: replay, list, recover and promote kernel failure dumps (S20 KH-01).
//!
//! Stands in for `amg-cli kernel replay|promote` until lane E adds those subcommands; they
//! should be thin wrappers over the same `amg_brep::capture` calls.
//!
//! ```text
//! kernel-replay replay [--validate] [--isolated] <dump-or-capture-dir>...
//! kernel-replay list    [capture-dir]
//! kernel-replay recover [capture-dir]
//! kernel-replay promote <dump> <corpus-dir> <id>
//! ```
//!
//! `replay` takes dump directories or a capture directory (every dump in it) and prints one
//! JSON `ReplayReport` per line, then a summary on stderr. `--isolated` runs each dump in a
//! child process (this program again), so a crashing dump is reported as `Crash` instead of
//! ending the run. The capture directory defaults to `CaptureConfig::from_env()`
//! (`AMG_KERNEL_CAPTURE_DIR`, else `%LOCALAPPDATA%\Amalgam\kernel-failures`).
//!
//! Build with `--features occt`; without OCCT every replay reports `BackendUnavailable`.
#![allow(
    clippy::print_stdout,
    clippy::print_stderr,
    reason = "command-line tool"
)]

use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode};

use amg_brep::capture::{
    CaptureConfig, FailureRecorder, ReplayOpts, ReplayOutcome, ReplayReport, child_report_line,
    promote, read_manifest, replay, replay_in_child,
};
use amg_brep::{KernelConfig, OcctKernel};

const USAGE: &str = "usage:
  kernel-replay replay [--validate] [--isolated] <dump-or-capture-dir>...
  kernel-replay list    [capture-dir]
  kernel-replay recover [capture-dir]
  kernel-replay promote <dump> <corpus-dir> <id>";

/// Hidden mode used by `--isolated`: replay one dump and print the report line.
const CHILD: &str = "--child";

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match run(&args) {
        Ok(code) => code,
        Err(e) => {
            eprintln!("kernel-replay: {e}");
            ExitCode::from(2)
        }
    }
}

fn kernel() -> OcctKernel {
    OcctKernel::new(KernelConfig::default())
}

fn capture_dir(arg: Option<&String>) -> PathBuf {
    arg.map_or_else(|| CaptureConfig::from_env().dir, PathBuf::from)
}

fn run(args: &[String]) -> Result<ExitCode, String> {
    let Some((cmd, rest)) = args.split_first() else {
        eprintln!("{USAGE}");
        return Ok(ExitCode::from(2));
    };
    match cmd.as_str() {
        CHILD => {
            let (validate, dump) = match rest {
                [flag, dump] if flag == "--validate" => (true, dump),
                [dump] => (false, dump),
                _ => return Err(format!("{CHILD} takes [--validate] <dump>")),
            };
            let report =
                replay(Path::new(dump), &kernel(), &opts(validate)).map_err(|e| e.to_string())?;
            println!("{}", child_report_line(&report));
            Ok(ExitCode::SUCCESS)
        }
        "replay" => replay_cmd(rest),
        "list" => {
            let dir = capture_dir(rest.first());
            let rec = recorder(&dir)?;
            for d in rec.dumps().map_err(|e| e.to_string())? {
                match read_manifest(&d) {
                    Ok(m) => println!(
                        "{}\t{}\t{:?}\tcount={}\t{}",
                        d.file_name().unwrap_or_default().to_string_lossy(),
                        m.op.kind(),
                        m.error.class,
                        m.count,
                        m.error.message
                    ),
                    Err(e) => println!("{}\tunreadable: {e}", d.display()),
                }
            }
            Ok(ExitCode::SUCCESS)
        }
        "recover" => {
            let dir = capture_dir(rest.first());
            let n = recorder(&dir)?
                .recover_pending()
                .map_err(|e| e.to_string())?;
            println!("recovered {n} pending dump(s) in {}", dir.display());
            Ok(ExitCode::SUCCESS)
        }
        "promote" => {
            let [dump, corpus, id] = rest else {
                return Err(format!("promote takes <dump> <corpus-dir> <id>\n{USAGE}"));
            };
            let case = promote(Path::new(dump), Path::new(corpus), id, &kernel())
                .map_err(|e| e.to_string())?;
            println!("{}", case.display());
            Ok(ExitCode::SUCCESS)
        }
        "-h" | "--help" | "help" => {
            println!("{USAGE}");
            Ok(ExitCode::SUCCESS)
        }
        other => Err(format!("unknown command `{other}`\n{USAGE}")),
    }
}

fn opts(validate: bool) -> ReplayOpts {
    if validate {
        ReplayOpts::validating()
    } else {
        ReplayOpts::default()
    }
}

fn recorder(dir: &Path) -> Result<FailureRecorder, String> {
    FailureRecorder::new(CaptureConfig::in_dir(dir)).map_err(|e| e.to_string())
}

/// Expands each argument: a dump (has `manifest.json`) or a capture directory (its dumps).
fn dumps_of(paths: &[String]) -> Result<Vec<PathBuf>, String> {
    let mut out = Vec::new();
    for p in paths {
        let p = PathBuf::from(p);
        if p.join("manifest.json").is_file() {
            out.push(p);
        } else {
            out.extend(recorder(&p)?.dumps().map_err(|e| e.to_string())?);
        }
    }
    Ok(out)
}

fn replay_cmd(rest: &[String]) -> Result<ExitCode, String> {
    let validate = rest.iter().any(|a| a == "--validate");
    let isolated = rest.iter().any(|a| a == "--isolated");
    let paths: Vec<String> = rest
        .iter()
        .filter(|a| !a.starts_with("--"))
        .cloned()
        .collect();
    if paths.is_empty() {
        return Err(format!("replay needs at least one path\n{USAGE}"));
    }
    let dumps = dumps_of(&paths)?;
    let k = (!isolated).then(kernel);
    let exe = std::env::current_exe().map_err(|e| e.to_string())?;
    let (mut reproduced, mut different, mut succeeds, mut errors) = (0, 0, 0, 0);
    for d in &dumps {
        let report: Result<ReplayReport, _> = match &k {
            Some(k) => replay(d, k, &opts(validate)),
            None => {
                let mut child = Command::new(&exe);
                child.arg(CHILD);
                if validate {
                    child.arg("--validate");
                }
                child.arg(d);
                replay_in_child(d, child)
            }
        };
        match report {
            Ok(r) => {
                match r.outcome {
                    ReplayOutcome::Reproduced(_) => reproduced += 1,
                    ReplayOutcome::DifferentFailure(_) => different += 1,
                    ReplayOutcome::NowSucceeds { .. } => succeeds += 1,
                }
                println!("{}", serde_json::to_string(&r).map_err(|e| e.to_string())?);
            }
            Err(e) => {
                errors += 1;
                eprintln!("{}: {e}", d.display());
            }
        }
    }
    eprintln!(
        "{} dump(s): {reproduced} reproduced, {different} different failure, {succeeds} now \
         succeed, {errors} unreadable",
        dumps.len()
    );
    Ok(if errors == 0 {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    })
}
