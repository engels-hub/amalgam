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
    // FND-09 replaces this with the adapter from amg_jobs::gpu.
    println!("gpu:                not initialised (FND-09)");
}
