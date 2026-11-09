//! MAT-01 budgets (docs/specs/S17-materials.md): parse a 1000-material library < 20 ms,
//! `Material::content_hash` < 10 µs.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    reason = "test and bench code"
)]

#[path = "../tests/common/mod.rs"]
mod common;

use std::hint::black_box;

use amg_materials::{Library, builtin};
use criterion::{Criterion, criterion_group, criterion_main};

fn benches(c: &mut Criterion) {
    let text = common::library(1000).to_toml_string();
    c.bench_function("parse_library_1000", |b| {
        b.iter(|| Library::from_toml_str(black_box(&text), "bench").unwrap());
    });
    c.bench_function("toml_value_only_1000", |b| {
        b.iter(|| black_box(&text).parse::<toml::Table>().unwrap());
    });
    c.bench_function("toml_detable_only_1000", |b| {
        b.iter(|| toml::de::DeTable::parse(black_box(&text)).unwrap());
    });
    let lib = common::library(1000);
    c.bench_function("write_library_1000", |b| {
        b.iter(|| black_box(&lib).to_toml_string())
    });
    let m = common::material(1);
    c.bench_function("content_hash", |b| b.iter(|| black_box(&m).content_hash()));
    c.bench_function("builtin_parse_cold", |b| {
        let src = builtin().to_toml_string();
        b.iter(|| Library::from_toml_str(black_box(&src), "builtin").unwrap());
    });
}

criterion_group!(group, benches);
criterion_main!(group);
