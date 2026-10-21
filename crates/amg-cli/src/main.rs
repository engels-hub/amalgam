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
    println!("log dir:            {log_dir}");
    gpu_info();
}

/// Opens the shared device the app would use (`AMG_WGPU_BACKEND` honoured) and prints it.
fn gpu_info() {
    use amg_jobs::gpu::{Gpu, GpuConfig, wgpu};

    let cfg = match GpuConfig::try_from_env() {
        Ok(cfg) => cfg,
        Err(e) => {
            println!("gpu:                error: {e}");
            return;
        }
    };
    let start = std::time::Instant::now();
    match Gpu::new_blocking(&cfg) {
        Ok(gpu) => {
            let ms = start.elapsed().as_secs_f64() * 1e3;
            let info = gpu.info();
            println!("gpu:                {}", info.name);
            println!(
                "gpu backend:        {:?} ({:?})",
                info.backend, info.device_type
            );
            println!("gpu driver:         {} {}", info.driver, info.driver_info);
            println!(
                "gpu features:       SHADER_F64={} TIMESTAMP_QUERY={}",
                gpu.has(wgpu::Features::SHADER_F64),
                gpu.has(wgpu::Features::TIMESTAMP_QUERY)
            );
            println!("gpu init:           {ms:.0} ms");
            tracing::info!(adapter = %info.name, backend = ?info.backend, init_ms = ms, "gpu");
        }
        Err(e) => {
            use amg_core::Diagnostic as _;
            println!("gpu:                unavailable: {e}");
            tracing::warn!(error = %e, "gpu unavailable");
            if let Some(hint) = e.hint() {
                println!("                    {hint}");
            }
        }
    }
}
