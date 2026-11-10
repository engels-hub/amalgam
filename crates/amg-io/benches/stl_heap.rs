//! IO-02 Done-when 5, peak extra heap of the 1 GB binary import (at most 2.5 GB) measured
//! with `dhat`. Its own binary because the `dhat` allocator would distort the timed benches.
//! The ASCII import is measured too (same bound).
#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::print_stdout,
    reason = "bench code"
)]

mod common;

use amg_io::{Format, ImportOptions, Source, import};
use amg_jobs::JobCtx;

#[global_allocator]
static ALLOC: dhat::Alloc = dhat::Alloc;

const LIMIT: u64 = 2_500_000_000;

fn measure(name: &str, path: &std::path::Path) {
    let profiler = dhat::Profiler::builder().testing().build();
    let imported = import(
        Source::Path(path),
        Some(Format::Stl),
        &ImportOptions::default(),
        &JobCtx::detached(),
    )
    .expect("import");
    let stats = dhat::HeapStats::get();
    println!(
        "{name}: {} triangles; peak heap {:.0} MB ({} blocks at peak), live after import {:.0} MB",
        imported.report.triangles_read,
        stats.max_bytes as f64 / 1e6,
        stats.max_blocks,
        stats.curr_bytes as f64 / 1e6,
    );
    assert!(
        (stats.max_bytes as u64) <= LIMIT,
        "{name}: peak heap {} exceeds {LIMIT}",
        stats.max_bytes
    );
    drop(imported);
    drop(profiler);
}

fn main() {
    let bin = common::bin_1gb();
    measure("stl_read_bin_1gb", &bin);
    let ascii = common::ascii_1gb();
    measure("stl_read_ascii_1gb", &ascii);
}
