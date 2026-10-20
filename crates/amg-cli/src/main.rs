//! Headless Amalgam command-line tool.
//!
//! Spec: S00 (skeleton), S18 (subcommands). Lane: E.

#![allow(clippy::print_stdout)]

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
    match Cli::parse().command {
        Command::Info => info(),
    }
}

fn info() {
    let profile = if cfg!(debug_assertions) {
        "debug"
    } else {
        "release"
    };
    let threads = std::thread::available_parallelism().map_or(1, |n| n.get());
    println!("amg-cli {VERSION}");
    println!("profile:            {profile}");
    println!("interactive pool:   {threads} threads");
    println!(
        "background pool:    {} threads",
        threads.saturating_sub(1).max(1)
    );
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
        }
        Err(e) => {
            use amg_core::Diagnostic as _;
            println!("gpu:                unavailable: {e}");
            if let Some(hint) = e.hint() {
                println!("                    {hint}");
            }
        }
    }
}
