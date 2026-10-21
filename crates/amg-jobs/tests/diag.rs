//! FND-07 process-level test: a child process (this test binary, re-run with an env switch)
//! initialises diagnostics, survives a job panic, then panics on its own thread. The parent
//! checks the log file, the crash report and the recovery hook.

#![allow(clippy::unwrap_used, clippy::expect_used, reason = "test code")]

use std::{
    path::{Path, PathBuf},
    process::Command,
    time::{SystemTime, UNIX_EPOCH},
};

use amg_jobs::{
    Cancelled, JobError, JobSystem, JobSystemConfig, SpawnOptions,
    diag::{self, DiagConfig, LOG_DIR_ENV, LOG_ENV},
};

const CHILD_ENV: &str = "AMG_DIAG_TEST_CHILD";
const VERSION: &str = "9.8.7-diagtest";
const SHA: &str = "feedc0ffee12";

/// The child half. A no-op unless spawned by [`panic_writes_log_crash_report_and_recovery`].
#[test]
fn diag_child() {
    if std::env::var_os(CHILD_ENV).is_none() {
        return;
    }
    let dir = PathBuf::from(std::env::var_os(LOG_DIR_ENV).unwrap());
    let guard = diag::init(&DiagConfig {
        app_name: "diag-test",
        version: VERSION,
        git_sha: SHA,
        log_dir: None,
        default_filter: "info",
        stderr: true,
    })
    .unwrap();
    assert_eq!(guard.log_dir(), dir);
    assert!(matches!(
        diag::init(&DiagConfig::default()),
        Err(diag::DiagError::AlreadyInitialised)
    ));

    let recovery = dir.join("recovery.amg");
    diag::register_recovery_hook(
        "test-autosave",
        Box::new(move || {
            std::fs::write(&recovery, b"recovered").ok()?;
            Some(recovery.clone())
        }),
    );
    diag::register_crash_info("adapter", Box::new(|| "Test Adapter (WARP)".to_owned()));

    // A panic inside a job is caught: the process keeps going and no crash report is written.
    let jobs = JobSystem::new(JobSystemConfig::default()).unwrap();
    let h = jobs.spawn_fn(
        "boom",
        SpawnOptions::default(),
        |_ctx| -> Result<(), Cancelled> { panic!("job panic payload") },
    );
    match h.wait_blocking() {
        Err(JobError::Panicked(m)) => assert_eq!(m, "job panic payload"),
        other => panic!("unexpected job result {other:?}"),
    }
    assert_eq!(crash_files(&dir).len(), 0);
    tracing::info!("context line before crash: job panic survived");
    eprintln!("CHILD: job panic survived");

    let _guard = guard; // flushed while unwinding
    panic!("deliberate diag test panic");
}

#[test]
fn panic_writes_log_crash_report_and_recovery() {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let dir = std::env::temp_dir().join(format!("amg-diag-it-{}-{nanos}", std::process::id()));
    let out = Command::new(std::env::current_exe().unwrap())
        .args(["diag_child", "--exact", "--nocapture"])
        .env(CHILD_ENV, "1")
        .env(LOG_DIR_ENV, &dir)
        .env_remove(LOG_ENV)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&out.stderr);
    let ctx = format!(
        "status {:?}\n--- stdout\n{}\n--- stderr\n{stderr}",
        out.status,
        String::from_utf8_lossy(&out.stdout)
    );
    assert!(!out.status.success(), "child should fail\n{ctx}");
    assert!(stderr.contains("CHILD: job panic survived"), "{ctx}");

    // Recovery hook ran and its path is printed.
    let rec_line = stderr
        .lines()
        .find_map(|l| l.trim().strip_prefix("Recovery file: "))
        .unwrap_or_else(|| panic!("no recovery line\n{ctx}"));
    let rec = PathBuf::from(rec_line.trim());
    assert_eq!(rec, dir.join("recovery.amg"));
    assert_eq!(std::fs::read(&rec).unwrap(), b"recovered");

    // The rolling log has both panics, with backtraces.
    let log: String = files_with(&dir, "diag-test", ".log")
        .iter()
        .map(|p| std::fs::read_to_string(p).unwrap())
        .collect();
    assert!(!log.is_empty(), "no log file in {}\n{ctx}", dir.display());
    for needle in [
        "diagnostics initialised",
        "caught panic",
        "job panic payload",
        "deliberate diag test panic",
        "backtrace:",
        "diag_child",
        "Recovery file:",
    ] {
        assert!(log.contains(needle), "log lacks {needle:?}:\n{log}");
    }

    // Exactly one crash report (the job panic made none), with the expected content.
    let crashes = crash_files(&dir);
    assert_eq!(crashes.len(), 1, "{crashes:?}\n{ctx}");
    let report = std::fs::read_to_string(&crashes[0]).unwrap();
    for needle in [
        &format!("version:    {VERSION}") as &str,
        &format!("git sha:    {SHA}"),
        "adapter: Test Adapter (WARP)",
        "message:    deliberate diag test panic",
        "test-autosave: saved",
        "[backtrace]",
        "context line before crash: job panic survived",
    ] {
        assert!(
            report.contains(needle),
            "report lacks {needle:?}:\n{report}"
        );
    }

    let _ = std::fs::remove_dir_all(&dir);
}

fn files_with(dir: &Path, prefix: &str, suffix: &str) -> Vec<PathBuf> {
    let Ok(rd) = std::fs::read_dir(dir) else {
        return Vec::new();
    };
    let mut v: Vec<PathBuf> = rd
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| {
            let n = p.file_name().unwrap().to_string_lossy();
            n.starts_with(prefix) && n.ends_with(suffix)
        })
        .collect();
    v.sort();
    v
}

fn crash_files(dir: &Path) -> Vec<PathBuf> {
    files_with(dir, "crash-", ".txt")
}
