//! MAT-01 Done-when (h): a 1000-material library parses in < 20 ms. The timing is asserted
//! only under `AMG_STRICT_TIMING=1` (use `--release`); the criterion bench is
//! `benches/library.rs`.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    reason = "test and bench code"
)]

mod common;

use std::time::Instant;

use amg_materials::Library;

fn median_ms(mut f: impl FnMut()) -> f64 {
    let mut runs: Vec<f64> = (0..15)
        .map(|_| {
            let t = Instant::now();
            f();
            t.elapsed().as_secs_f64() * 1e3
        })
        .collect();
    runs.sort_by(f64::total_cmp);
    runs[runs.len() / 2]
}

#[test]
fn parse_1000_materials_budget() {
    let text = common::library(1000).to_toml_string();
    let lib = Library::from_toml_str(&text, "gen").expect("generated library loads");
    assert_eq!(lib.materials().count(), 1000);
    let ms = median_ms(|| {
        std::hint::black_box(Library::from_toml_str(std::hint::black_box(&text), "gen").unwrap());
    });
    println!(
        "parse 1000 materials ({} KiB): {ms:.2} ms median",
        text.len() / 1024
    );
    if std::env::var_os("AMG_STRICT_TIMING").is_some_and(|v| v == "1") {
        assert!(ms < 20.0, "parse took {ms:.2} ms, budget 20 ms");
    }
}

#[test]
fn content_hash_budget() {
    let lib = common::library(10);
    let m = lib.materials().next().unwrap().clone();
    let ms = median_ms(|| {
        for _ in 0..1000 {
            std::hint::black_box(m.content_hash());
        }
    });
    let us = ms * 1e3 / 1000.0;
    println!("content_hash: {us:.2} us median");
    if std::env::var_os("AMG_STRICT_TIMING").is_some_and(|v| v == "1") {
        assert!(us < 10.0, "content_hash took {us:.2} us, budget 10 us");
    }
}
