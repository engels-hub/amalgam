//! Diagnostics: tracing setup, rolling log files, the panic hook, crash reports and recovery
//! hooks (S00 §4, FND-07).
//!
//! A binary calls [`init`] once at startup and keeps the returned [`DiagGuard`] alive until it
//! exits:
//!
//! ```no_run
//! use amg_jobs::diag::{self, DiagConfig};
//!
//! let _diag = diag::init(&DiagConfig {
//!     app_name: "amalgam",
//!     version: env!("CARGO_PKG_VERSION"),
//!     ..DiagConfig::default()
//! })
//! .expect("diagnostics");
//! tracing::info!("started");
//! ```
//!
//! # What gets installed
//! - A global `tracing` subscriber filtered by [`LOG_ENV`] (`AMG_LOG`, env-filter syntax) or,
//!   when unset or invalid, [`DiagConfig::default_filter`]. Layers: stderr (optional), a daily
//!   rolling file `<app_name>.<yyyy-mm-dd>.log` in the log directory (7 files kept), an
//!   in-memory ring of the last [`RING_LINES`] lines for crash reports, and with the `tracy`
//!   cargo feature a `tracing-tracy` layer.
//! - The panic hook ([`install_panic_hook`]).
//!
//! # Panics
//! A panic inside [`catch_panic`] (job panics, notifier panics) is *caught*: the hook logs it with
//! a backtrace and nothing else. Any other panic is treated as a crash: the hook logs it, runs the
//! registered [`RecoveryHook`]s (printing `Recovery file: <path>` to stderr for each saved file),
//! and writes `crash-<utc>.txt` into the log directory with the version, git SHA, the registered
//! [`register_crash_info`] sections (e.g. the GPU adapter), the panic, a backtrace and the last
//! log lines. The previous hook is not called once [`init`] has run (the log has everything).

mod hook;
mod ring;
mod time;

use std::{
    fmt,
    io::IsTerminal,
    path::{Path, PathBuf},
    sync::OnceLock,
};

use parking_lot::Mutex;
use tracing_subscriber::{
    EnvFilter, Layer, Registry, filter::LevelFilter, layer::SubscriberExt, util::SubscriberInitExt,
};

pub use hook::{
    CrashInfoProvider, HOOK_TIMEOUT, RecoveryHook, catch_panic, install_panic_hook,
    register_crash_info, register_recovery_hook, unregister_recovery_hook,
};

/// Environment variable with the log filter (`tracing-subscriber` env-filter syntax), e.g.
/// `AMG_LOG=debug` or `AMG_LOG=info,amg_jobs=trace`. Overrides [`DiagConfig::default_filter`].
pub const LOG_ENV: &str = "AMG_LOG";
/// Environment variable with the log directory. Overrides [`DiagConfig::log_dir`] and the
/// platform default (tests point it at a temporary directory).
pub const LOG_DIR_ENV: &str = "AMG_LOG_DIR";
/// Number of rolling daily log files kept per app.
pub const MAX_LOG_FILES: usize = 7;
/// Number of most recent log lines kept in memory for crash reports.
pub const RING_LINES: usize = 200;

/// Configuration for [`init`].
#[derive(Clone, Debug)]
pub struct DiagConfig {
    /// File-name prefix of the logs, e.g. `"amalgam"` or `"amalgam-cli"`.
    pub app_name: &'static str,
    /// Version written into crash reports (the binary's `CARGO_PKG_VERSION`).
    pub version: &'static str,
    /// Git commit written into crash reports (the binary embeds it at build time).
    pub git_sha: &'static str,
    /// Log directory. `None` means the platform default (`%LOCALAPPDATA%\Amalgam\logs` on
    /// Windows). [`LOG_DIR_ENV`] overrides both.
    pub log_dir: Option<PathBuf>,
    /// Filter used when [`LOG_ENV`] is unset or invalid.
    pub default_filter: &'static str,
    /// Also log to stderr.
    pub stderr: bool,
}

impl Default for DiagConfig {
    /// `amalgam`, this crate's version, git SHA `unknown`, default log dir, filter `info`,
    /// stderr on.
    fn default() -> Self {
        Self {
            app_name: "amalgam",
            version: env!("CARGO_PKG_VERSION"),
            git_sha: "unknown",
            log_dir: None,
            default_filter: "info",
            stderr: true,
        }
    }
}

impl DiagConfig {
    /// The log directory [`init`] uses: [`LOG_DIR_ENV`], else [`DiagConfig::log_dir`], else
    /// [`default_log_dir`].
    pub fn resolved_log_dir(&self) -> PathBuf {
        match std::env::var_os(LOG_DIR_ENV) {
            Some(dir) if !dir.is_empty() => PathBuf::from(dir),
            _ => self.log_dir.clone().unwrap_or_else(default_log_dir),
        }
    }
}

/// The platform default log directory: `%LOCALAPPDATA%\Amalgam\logs` on Windows; elsewhere
/// `$XDG_STATE_HOME/amalgam/logs`, `$HOME/.local/state/amalgam/logs` or the temp dir.
pub fn default_log_dir() -> PathBuf {
    let var = |name: &str| std::env::var_os(name).filter(|v| !v.is_empty());
    if cfg!(windows)
        && let Some(local) = var("LOCALAPPDATA")
    {
        return PathBuf::from(local).join("Amalgam").join("logs");
    }
    if let Some(state) = var("XDG_STATE_HOME") {
        return PathBuf::from(state).join("amalgam").join("logs");
    }
    if let Some(home) = var("HOME") {
        return PathBuf::from(home).join(".local/state/amalgam/logs");
    }
    std::env::temp_dir().join("amalgam").join("logs")
}

/// Keeps file logging alive. Hold it for the whole process: dropping it flushes the log file
/// and stops file logging.
#[must_use = "dropping the guard stops file logging"]
pub struct DiagGuard {
    _worker: tracing_appender::non_blocking::WorkerGuard,
    log_dir: PathBuf,
}

impl DiagGuard {
    /// The directory holding the log files and crash reports.
    pub fn log_dir(&self) -> &Path {
        &self.log_dir
    }
}

impl fmt::Debug for DiagGuard {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("DiagGuard")
            .field("log_dir", &self.log_dir)
            .finish_non_exhaustive()
    }
}

/// Failure of [`init`].
#[derive(Debug, thiserror::Error)]
pub enum DiagError {
    /// The log directory could not be created or the log file could not be opened.
    #[error("cannot create log directory {path}: {source}")]
    LogDir {
        /// The directory.
        path: PathBuf,
        /// The underlying error.
        #[source]
        source: std::io::Error,
    },
    /// Another global `tracing` subscriber (or `log` logger) was installed first.
    #[error("a global tracing subscriber is already installed")]
    AlreadyInitialised,
}

impl amg_core::Diagnostic for DiagError {
    fn hint(&self) -> Option<String> {
        match self {
            DiagError::LogDir { .. } => Some(format!(
                "check the directory is writable, or set {LOG_DIR_ENV} to another directory"
            )),
            DiagError::AlreadyInitialised => None,
        }
    }
}

/// Process-wide settings the panic hook reads; set once by [`init`].
#[derive(Debug)]
pub(crate) struct Installed {
    pub(crate) app_name: &'static str,
    pub(crate) version: &'static str,
    pub(crate) git_sha: &'static str,
    pub(crate) log_dir: PathBuf,
    pub(crate) stderr: bool,
}

static INSTALLED: OnceLock<Installed> = OnceLock::new();
/// Serialises [`init`] so two racing calls cannot both pass the "already initialised" check.
static INIT_LOCK: Mutex<()> = Mutex::new(());

pub(crate) fn installed() -> Option<&'static Installed> {
    INSTALLED.get()
}

/// Installs the global `tracing` subscriber and the panic hook (see the module docs).
///
/// Errors with [`DiagError::AlreadyInitialised`] if called twice or if another global
/// subscriber exists, and with [`DiagError::LogDir`] if the log directory is unusable (nothing is
/// installed then, so the caller may retry with another directory).
pub fn init(cfg: &DiagConfig) -> Result<DiagGuard, DiagError> {
    let _lock = INIT_LOCK.lock();
    if INSTALLED.get().is_some() {
        return Err(DiagError::AlreadyInitialised);
    }
    let log_dir = cfg.resolved_log_dir();
    let log_dir_err = |source| DiagError::LogDir {
        path: log_dir.clone(),
        source,
    };
    std::fs::create_dir_all(&log_dir).map_err(log_dir_err)?;
    let appender = tracing_appender::rolling::Builder::new()
        .rotation(tracing_appender::rolling::Rotation::DAILY)
        .filename_prefix(cfg.app_name)
        .filename_suffix("log")
        .max_log_files(MAX_LOG_FILES)
        .build(&log_dir)
        .map_err(|e| log_dir_err(std::io::Error::other(e)))?;
    let (file_writer, worker) = tracing_appender::non_blocking::NonBlockingBuilder::default()
        .lossy(false)
        .thread_name("amg-log-writer")
        .finish(appender);

    let (filter, bad_env) = build_filter(cfg.default_filter);

    let mut layers: Vec<Box<dyn Layer<Registry> + Send + Sync>> = Vec::new();
    if cfg.stderr {
        layers.push(
            tracing_subscriber::fmt::layer()
                .with_writer(std::io::stderr)
                .with_ansi(std::io::stderr().is_terminal())
                .boxed(),
        );
    }
    layers.push(
        tracing_subscriber::fmt::layer()
            .with_writer(file_writer)
            .with_ansi(false)
            .with_thread_names(true)
            .boxed(),
    );
    layers.push(
        tracing_subscriber::fmt::layer()
            .with_writer(ring::RingWriter::make)
            .with_ansi(false)
            .with_thread_names(true)
            .boxed(),
    );
    #[cfg(feature = "tracy")]
    layers.push(tracing_tracy::TracyLayer::default().boxed());

    tracing_subscriber::registry()
        .with(layers.with_filter(filter))
        .try_init()
        .map_err(|_| DiagError::AlreadyInitialised)?;

    let _ = INSTALLED.set(Installed {
        app_name: cfg.app_name,
        version: cfg.version,
        git_sha: cfg.git_sha,
        log_dir: log_dir.clone(),
        stderr: cfg.stderr,
    });
    install_panic_hook();
    if let Some(e) = bad_env {
        tracing::warn!("ignoring invalid {LOG_ENV}: {e}");
    }
    tracing::info!(
        app = cfg.app_name,
        version = cfg.version,
        git_sha = cfg.git_sha,
        log_dir = %log_dir.display(),
        "diagnostics initialised"
    );
    Ok(DiagGuard {
        _worker: worker,
        log_dir,
    })
}

/// The [`LOG_ENV`] filter, else `default` (invalid directives in `default` are ignored). Also
/// returns the parse error of an invalid [`LOG_ENV`].
fn build_filter(default: &str) -> (EnvFilter, Option<String>) {
    let builder = || EnvFilter::builder().with_default_directive(LevelFilter::INFO.into());
    match std::env::var(LOG_ENV) {
        Ok(env) if !env.trim().is_empty() => match builder().parse(&env) {
            Ok(f) => (f, None),
            Err(e) => (builder().parse_lossy(default), Some(e.to_string())),
        },
        _ => (builder().parse_lossy(default), None),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_config_matches_spec() {
        let c = DiagConfig::default();
        assert_eq!(c.default_filter, "info");
        assert!(c.stderr);
        assert!(c.log_dir.is_none());
    }

    #[test]
    fn explicit_log_dir_is_used_without_env() {
        // AMG_LOG_DIR is not set by the test runner; skip if a developer set it.
        if std::env::var_os(LOG_DIR_ENV).is_some() {
            return;
        }
        let c = DiagConfig {
            log_dir: Some(PathBuf::from("x/y")),
            ..DiagConfig::default()
        };
        assert_eq!(c.resolved_log_dir(), PathBuf::from("x/y"));
        let d = DiagConfig::default().resolved_log_dir();
        assert_eq!(d, default_log_dir());
        assert!(d.ends_with("logs"));
    }

    #[test]
    fn invalid_default_filter_is_lossy() {
        // Only the default branch is exercised (the env var is process-global).
        if std::env::var_os(LOG_ENV).is_some() {
            return;
        }
        let (_f, bad) = build_filter("info,=[");
        assert!(bad.is_none());
    }

    #[test]
    fn diag_error_has_hint_for_log_dir() {
        use amg_core::Diagnostic;
        let e = DiagError::LogDir {
            path: PathBuf::from("z"),
            source: std::io::Error::other("denied"),
        };
        assert!(e.hint().unwrap().contains(LOG_DIR_ENV));
        assert!(e.to_string().contains('z'));
        assert!(DiagError::AlreadyInitialised.hint().is_none());
    }
}
