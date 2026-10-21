//! The panic hook, crash reports, recovery hooks and crash-info providers.

use std::{
    backtrace::Backtrace,
    cell::Cell,
    fmt::Write as _,
    io::Write as _,
    panic::{PanicHookInfo, UnwindSafe},
    path::{Path, PathBuf},
    sync::{Arc, Once},
    time::Duration,
};

use parking_lot::Mutex;

use super::{ring, time::Utc};

/// Called by the panic hook on a crash. Saves what it can (S01 autosave flushes the recovery
/// file) and returns the path of what it saved, or `None` if there was nothing to save.
///
/// Runs on a helper thread while the panicking thread waits, with a [`HOOK_TIMEOUT`] limit, so
/// it must not need locks the panicking thread may hold. A panic inside it is logged and
/// ignored.
pub type RecoveryHook = Box<dyn Fn() -> Option<PathBuf> + Send + Sync>;

/// Supplies one section of the crash report, e.g. the GPU adapter (`"adapter"`, registered by
/// FND-09 / the app once the device exists). Same execution rules as [`RecoveryHook`].
pub type CrashInfoProvider = Box<dyn Fn() -> String + Send + Sync>;

/// How long the panic hook waits for one recovery hook or crash-info provider.
pub const HOOK_TIMEOUT: Duration = Duration::from_secs(10);

type SharedRecovery = Arc<dyn Fn() -> Option<PathBuf> + Send + Sync>;
type SharedInfo = Arc<dyn Fn() -> String + Send + Sync>;

static RECOVERY: Mutex<Vec<(&'static str, SharedRecovery)>> = Mutex::new(Vec::new());
static CRASH_INFO: Mutex<Vec<(&'static str, SharedInfo)>> = Mutex::new(Vec::new());

thread_local! {
    /// > 0 while inside [`catch_panic`] on this thread.
    static CATCH_DEPTH: Cell<u32> = const { Cell::new(0) };
    /// True while this thread runs the panic hook.
    static IN_HOOK: Cell<bool> = const { Cell::new(false) };
}

/// Registers (or replaces, by `name`) a hook the panic hook calls on a crash.
pub fn register_recovery_hook(name: &'static str, hook: RecoveryHook) {
    upsert(&RECOVERY, name, Arc::from(hook));
}

/// Removes the recovery hook registered under `name`; returns whether one existed.
pub fn unregister_recovery_hook(name: &'static str) -> bool {
    let mut v = RECOVERY.lock();
    let before = v.len();
    v.retain(|(n, _)| *n != name);
    v.len() != before
}

/// Registers (or replaces, by `name`) a section of the crash report. The report always has an
/// `adapter` line; it says `unknown` until a provider named `"adapter"` is registered.
pub fn register_crash_info(name: &'static str, provider: CrashInfoProvider) {
    upsert(&CRASH_INFO, name, Arc::from(provider));
}

fn upsert<T: ?Sized>(reg: &Mutex<Vec<(&'static str, Arc<T>)>>, name: &'static str, v: Arc<T>) {
    let mut reg = reg.lock();
    match reg.iter_mut().find(|(n, _)| *n == name) {
        Some(slot) => slot.1 = v,
        None => reg.push((name, v)),
    }
}

/// `std::panic::catch_unwind` that tells the panic hook the panic is handled: the hook then only
/// logs it (with a backtrace) instead of writing a crash report and running recovery hooks.
/// The job system runs every job (and the notifier) through this.
pub fn catch_panic<R>(f: impl FnOnce() -> R + UnwindSafe) -> std::thread::Result<R> {
    struct Depth;
    impl Drop for Depth {
        fn drop(&mut self) {
            CATCH_DEPTH.with(|d| d.set(d.get().saturating_sub(1)));
        }
    }
    CATCH_DEPTH.with(|d| d.set(d.get() + 1));
    let _depth = Depth;
    std::panic::catch_unwind(f)
}

/// Installs the panic hook (see the [module docs](super)). Idempotent; [`super::init`] calls it.
///
/// Before [`super::init`] has run, the previous hook (normally the default stderr message) is
/// still called, and crash reports go to the default log directory.
pub fn install_panic_hook() {
    static ONCE: Once = Once::new();
    ONCE.call_once(|| {
        let prev = std::panic::take_hook();
        std::panic::set_hook(Box::new(move |info| on_panic(info, &*prev)));
    });
}

type PrevHook = dyn Fn(&PanicHookInfo<'_>) + Send + Sync;

fn on_panic(info: &PanicHookInfo<'_>, prev: &PrevHook) {
    if IN_HOOK.with(|h| h.replace(true)) {
        return; // re-entered on this thread; std aborts right after
    }
    struct Reset;
    impl Drop for Reset {
        fn drop(&mut self) {
            IN_HOOK.with(|h| h.set(false));
        }
    }
    let _reset = Reset;

    let message = payload_message(info);
    let location = info
        .location()
        .map_or_else(|| "<unknown>".to_owned(), ToString::to_string);
    let current = std::thread::current();
    let thread = current.name().unwrap_or("<unnamed>").to_owned();
    let backtrace = Backtrace::force_capture();
    let installed = super::installed();

    if CATCH_DEPTH.with(Cell::get) > 0 {
        tracing::error!(
            target: "amg::panic",
            "caught panic in thread '{thread}' at {location}: {message}\nbacktrace:\n{backtrace}"
        );
        if installed.is_none() {
            prev(info);
        }
        return;
    }

    // Snapshot before logging the panic, so the backtrace does not push context out.
    let last_lines = ring::snapshot();
    tracing::error!(
        target: "amg::panic",
        "panic in thread '{thread}' at {location}: {message}\nbacktrace:\n{backtrace}"
    );
    match installed {
        None => prev(info),
        Some(i) if !i.stderr => {
            eprintln!(
                "{} panicked in thread '{thread}' at {location}: {message}",
                i.app_name
            );
        }
        Some(_) => {}
    }

    let recovery = run_recovery_hooks();
    for (_, outcome) in &recovery {
        if let Outcome::Done(Some(path)) = outcome {
            eprintln!("Recovery file: {}", path.display());
            tracing::warn!(target: "amg::panic", "Recovery file: {}", path.display());
        }
    }

    let now = Utc::now();
    let report = Report {
        app: installed.map_or("amalgam", |i| i.app_name),
        version: installed.map_or(env!("CARGO_PKG_VERSION"), |i| i.version),
        git_sha: installed.map_or("unknown", |i| i.git_sha),
        time: now.rfc3339(),
        thread: &thread,
        location: &location,
        message: &message,
        info: collect_crash_info(),
        recovery: &recovery,
        backtrace: &backtrace.to_string(),
        last_lines: last_lines.as_deref(),
    }
    .render();
    let dir = installed.map_or_else(
        || super::DiagConfig::default().resolved_log_dir(),
        |i| i.log_dir.clone(),
    );
    match write_crash_file(&dir, &now, &report) {
        Ok(path) => {
            eprintln!("Crash report: {}", path.display());
            tracing::error!(target: "amg::panic", "Crash report: {}", path.display());
        }
        Err(e) => eprintln!("could not write a crash report to {}: {e}", dir.display()),
    }
}

fn payload_message(info: &PanicHookInfo<'_>) -> String {
    let p = info.payload();
    if let Some(s) = p.downcast_ref::<&'static str>() {
        (*s).to_owned()
    } else if let Some(s) = p.downcast_ref::<String>() {
        s.clone()
    } else {
        "non-string panic payload".to_owned()
    }
}

/// Result of one recovery hook or crash-info provider.
#[derive(Debug)]
enum Outcome<T> {
    Done(T),
    Panicked,
    TimedOut,
}

/// Runs `f` on a helper thread inside [`catch_panic`] and waits up to [`HOOK_TIMEOUT`]. A panic
/// inside the panic hook itself would abort the process; on another thread it is just caught.
fn run_isolated<T: Send + 'static>(name: &str, f: Arc<dyn Fn() -> T + Send + Sync>) -> Outcome<T> {
    let (tx, rx) = crossbeam_channel::bounded(1);
    let spawned = std::thread::Builder::new()
        .name(format!("amg-crash-{name}"))
        .spawn(move || {
            let r = catch_panic(std::panic::AssertUnwindSafe(|| f()));
            let _ = tx.send(r.ok());
        });
    if spawned.is_err() {
        return Outcome::Panicked;
    }
    match rx.recv_timeout(HOOK_TIMEOUT) {
        Ok(Some(v)) => Outcome::Done(v),
        Ok(None) => Outcome::Panicked,
        Err(_) => Outcome::TimedOut,
    }
}

fn run_recovery_hooks() -> Vec<(&'static str, Outcome<Option<PathBuf>>)> {
    let hooks = RECOVERY
        .try_lock_for(Duration::from_secs(1))
        .map(|v| v.clone())
        .unwrap_or_default();
    hooks
        .into_iter()
        .map(|(name, hook)| (name, run_isolated(name, hook)))
        .collect()
}

fn collect_crash_info() -> Vec<(&'static str, String)> {
    let providers = CRASH_INFO
        .try_lock_for(Duration::from_secs(1))
        .map(|v| v.clone())
        .unwrap_or_default();
    let mut out: Vec<(&'static str, String)> = providers
        .into_iter()
        .map(|(name, p)| {
            let text = match run_isolated(name, p) {
                Outcome::Done(s) => s,
                Outcome::Panicked => "<provider panicked>".to_owned(),
                Outcome::TimedOut => "<provider timed out>".to_owned(),
            };
            (name, text)
        })
        .collect();
    if !out.iter().any(|(n, _)| *n == "adapter") {
        out.insert(
            0,
            ("adapter", "unknown (no GPU info registered)".to_owned()),
        );
    }
    out
}

struct Report<'a> {
    app: &'a str,
    version: &'a str,
    git_sha: &'a str,
    time: String,
    thread: &'a str,
    location: &'a str,
    message: &'a str,
    info: Vec<(&'static str, String)>,
    recovery: &'a [(&'static str, Outcome<Option<PathBuf>>)],
    backtrace: &'a str,
    last_lines: Option<&'a [String]>,
}

impl Report<'_> {
    fn render(&self) -> String {
        let mut s = String::new();
        let _ = writeln!(s, "Amalgam crash report");
        let _ = writeln!(s, "app:        {}", self.app);
        let _ = writeln!(s, "version:    {}", self.version);
        let _ = writeln!(s, "git sha:    {}", self.git_sha);
        let _ = writeln!(s, "time (UTC): {}", self.time);
        let _ = writeln!(
            s,
            "os:         {} {}",
            std::env::consts::OS,
            std::env::consts::ARCH
        );
        let _ = writeln!(s, "thread:     {}", self.thread);
        let _ = writeln!(s, "location:   {}", self.location);
        let _ = writeln!(s, "message:    {}", self.message);
        let _ = writeln!(s, "\n[crash info]");
        for (name, text) in &self.info {
            let _ = writeln!(s, "{name}: {text}");
        }
        let _ = writeln!(s, "\n[recovery]");
        if self.recovery.is_empty() {
            let _ = writeln!(s, "(no recovery hooks registered)");
        }
        for (name, outcome) in self.recovery {
            let _ = match outcome {
                Outcome::Done(Some(p)) => writeln!(s, "{name}: saved {}", p.display()),
                Outcome::Done(None) => writeln!(s, "{name}: nothing to save"),
                Outcome::Panicked => writeln!(s, "{name}: panicked"),
                Outcome::TimedOut => writeln!(s, "{name}: timed out"),
            };
        }
        let _ = writeln!(s, "\n[backtrace]\n{}", self.backtrace.trim_end());
        match self.last_lines {
            Some(lines) => {
                let _ = writeln!(s, "\n[last {} log lines]", lines.len());
                for l in lines {
                    let _ = writeln!(s, "{l}");
                }
            }
            None => {
                let _ = writeln!(s, "\n[last log lines]\n(log buffer was locked)");
            }
        }
        s
    }
}

/// Writes `crash-<utc>.txt` (or `crash-<utc>-<n>.txt` if taken) into `dir`.
fn write_crash_file(dir: &Path, now: &Utc, report: &str) -> std::io::Result<PathBuf> {
    std::fs::create_dir_all(dir)?;
    let stamp = now.compact();
    for n in 0..100u32 {
        let name = if n == 0 {
            format!("crash-{stamp}.txt")
        } else {
            format!("crash-{stamp}-{n}.txt")
        };
        let path = dir.join(name);
        match std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&path)
        {
            Ok(mut f) => {
                f.write_all(report.as_bytes())?;
                f.sync_all()?;
                return Ok(path);
            }
            Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => {}
            Err(e) => return Err(e),
        }
    }
    Err(std::io::Error::other(
        "too many crash reports with the same timestamp",
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registries_replace_and_remove_by_name() {
        register_recovery_hook("unit-a", Box::new(|| None));
        register_recovery_hook("unit-a", Box::new(|| Some(PathBuf::from("x"))));
        assert_eq!(
            RECOVERY
                .lock()
                .iter()
                .filter(|(n, _)| *n == "unit-a")
                .count(),
            1
        );
        assert!(unregister_recovery_hook("unit-a"));
        assert!(!unregister_recovery_hook("unit-a"));
    }

    #[test]
    fn isolated_runner_reports_panics_and_values() {
        let ok = run_isolated("t-ok", Arc::new(|| 7));
        assert!(matches!(ok, Outcome::Done(7)));
        // The panic hook may be the default one here; either way the panic stays on the helper.
        let bad = run_isolated::<u32>("t-bad", Arc::new(|| panic!("provider bug")));
        assert!(matches!(bad, Outcome::Panicked));
    }

    #[test]
    fn catch_panic_tracks_depth() {
        assert_eq!(CATCH_DEPTH.with(Cell::get), 0);
        let r = catch_panic(|| CATCH_DEPTH.with(Cell::get));
        assert_eq!(r.unwrap(), 1);
        assert!(catch_panic(|| panic!("x")).is_err());
        assert_eq!(CATCH_DEPTH.with(Cell::get), 0);
    }

    #[test]
    fn report_has_all_sections_and_unique_files() {
        let recovery = [
            ("doc", Outcome::Done(Some(PathBuf::from("r.amg")))),
            ("other", Outcome::TimedOut),
        ];
        let lines = vec!["line one".to_owned()];
        let report = Report {
            app: "t",
            version: "1.2.3",
            git_sha: "abc123",
            time: "2026-10-20T00:00:00.000Z".to_owned(),
            thread: "main",
            location: "src/x.rs:1:1",
            message: "boom",
            info: vec![("adapter", "RTX".to_owned())],
            recovery: &recovery,
            backtrace: "0: frame",
            last_lines: Some(&lines),
        }
        .render();
        for needle in [
            "version:    1.2.3",
            "git sha:    abc123",
            "adapter: RTX",
            "doc: saved r.amg",
            "other: timed out",
            "[backtrace]\n0: frame",
            "[last 1 log lines]\nline one",
        ] {
            assert!(report.contains(needle), "missing {needle:?} in\n{report}");
        }

        let dir = std::env::temp_dir().join(format!("amg-diag-unit-{}", std::process::id()));
        let now = Utc::now();
        let a = write_crash_file(&dir, &now, "a").unwrap();
        let b = write_crash_file(&dir, &now, "b").unwrap();
        assert_ne!(a, b);
        assert!(
            a.file_name()
                .unwrap()
                .to_string_lossy()
                .starts_with("crash-")
        );
        assert_eq!(std::fs::read_to_string(&b).unwrap(), "b");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn adapter_line_defaults_to_unknown() {
        let info = collect_crash_info();
        assert_eq!(info[0].0, "adapter");
    }
}
