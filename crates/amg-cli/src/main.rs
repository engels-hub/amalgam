//! Headless Amalgam command-line tool.
//!
//! Spec: S00 (skeleton), S18 (subcommands). Lane: E.

#![allow(clippy::print_stdout)]

use amg_jobs::diag::{self, DiagConfig, DiagGuard, LOG_ENV};
use clap::{Parser, Subcommand};

/// Crate version plus the git commit it was built from.
const VERSION: &str = concat!(env!("CARGO_PKG_VERSION"), " (", env!("AMG_GIT_SHA"), ")");

#[derive(Debug, Parser)]
#[command(name = "amg-cli", version = VERSION, about = "Headless Amalgam tool")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Print build and runtime information.
    Info,
}

fn main() {
    let cli = Cli::parse();
    let diag = init_diagnostics();
    match cli.command {
        Command::Info => info(diag.as_ref()),
    }
}

/// Logs go to the rolling file, and to stderr only when `AMG_LOG` is set, so normal output
/// stays clean. Diagnostics failing to start is reported but never fatal.
fn init_diagnostics() -> Option<DiagGuard> {
    let cfg = DiagConfig {
        app_name: "amalgam-cli",
        version: env!("CARGO_PKG_VERSION"),
        git_sha: env!("AMG_GIT_SHA"),
        log_dir: None,
        default_filter: "info",
        stderr: std::env::var_os(LOG_ENV).is_some_and(|v| !v.is_empty()),
    };
    match diag::init(&cfg) {
        Ok(guard) => Some(guard),
        Err(e) => {
            eprintln!("warning: {}", amg_core::report_diagnostic(&e));
            diag::install_panic_hook();
            None
        }
    }
}

fn info(diag: Option<&DiagGuard>) {
    let profile = if cfg!(debug_assertions) {
        "debug"
    } else {
        "release"
    };
    let threads = std::thread::available_parallelism().map_or(1, |n| n.get());
    let background = threads.saturating_sub(1).max(1);
    let log_dir = diag.map_or_else(
        || "unavailable".to_owned(),
        |d| d.log_dir().display().to_string(),
    );
    tracing::info!(
        version = VERSION,
        profile,
        interactive_threads = threads,
        background_threads = background,
        "amg-cli info"
    );
    println!("amg-cli {VERSION}");
    println!("profile:            {profile}");
    println!("interactive pool:   {threads} threads");
    println!("background pool:    {background} threads");
    // FND-09 replaces this with the adapter from amg_jobs::gpu.
    println!("gpu:                not initialised (FND-09)");
    println!("log dir:            {log_dir}");
}
