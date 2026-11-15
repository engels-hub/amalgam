//! MAT-06 (S17): `Design` clone with a 100-entry material table (target < 1 µs: the table is
//! a persistent map, so a clone shares it).

#![allow(clippy::unwrap_used, missing_docs, reason = "benchmark harness")]

use std::hint::black_box;
use std::sync::Arc;

use amg_doc::{Command, Design, apply_command};
use amg_materials::{MaterialId, builtin, default_material_id};
use criterion::{Criterion, criterion_group, criterion_main};

fn design_with_materials(n: usize) -> Design {
    let mut d = Design::new("bench");
    let base = builtin().material(&default_material_id()).unwrap();
    for i in 0..n {
        let mut m = (**base).clone();
        m.id = MaterialId::new("doc", &format!("m{i:04}")).unwrap();
        apply_command(
            &mut d,
            Command::SetMaterial {
                material: Arc::new(m),
            },
        )
        .unwrap();
    }
    d
}

fn bench_clone(c: &mut Criterion) {
    let d = design_with_materials(100);
    c.bench_function("design_clone_100_materials", |b| {
        b.iter(|| black_box(black_box(&d).clone()));
    });
}

criterion_group!(benches, bench_clone);
criterion_main!(benches);
