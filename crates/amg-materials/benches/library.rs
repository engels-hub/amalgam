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

use amg_materials::{Library, LibrarySet, MaterialId, MaterialQuery, builtin};
use criterion::{Criterion, criterion_group, criterion_main};

/// MAT-07 (f): `LibrarySet::search` over 5000 materials < 1 ms median.
fn search_bench(c: &mut Criterion) {
    let mut set = LibrarySet::builtin_only();
    let dir = std::env::temp_dir().join("amg-materials-bench-search");
    let _ = std::fs::remove_dir_all(&dir);
    let lib = set.create_user_library(&dir, "user.gen", "Gen").unwrap();
    for i in 0..5000 {
        let mut m = common::material(i);
        m.id = MaterialId::new("user.gen", &format!("m{i}")).unwrap();
        lib.upsert_material(m).unwrap();
    }
    let text = MaterialQuery {
        text: Some("material 4242".into()),
        ..Default::default()
    };
    let all = MaterialQuery::default();
    c.bench_function("search_5000_text", |b| {
        b.iter(|| set.search(black_box(&text)))
    });
    c.bench_function("search_5000_all", |b| {
        b.iter(|| set.search(black_box(&all)))
    });
    let _ = std::fs::remove_dir_all(&dir);
}

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
    search_bench(c);
    c.bench_function("builtin_parse_cold", |b| {
        let src = builtin().to_toml_string();
        b.iter(|| Library::from_toml_str(black_box(&src), "builtin").unwrap());
    });
}

criterion_group!(group, benches);
criterion_main!(group);
