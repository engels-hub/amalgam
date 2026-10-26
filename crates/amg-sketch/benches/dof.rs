//! SK-04 DOF analysis (per-entity status + diagnosis) on generated sketches of about 50, 200,
//! 500 and 2000 entities.
//!
//! `cargo bench -p amg-sketch --bench dof`. Names: `sk04/dof_analysis/<family>/<entities>`; the
//! solver's structure cache is warm (as after a solve), the analysis cache is cold. Done-when 3
//! is every `*/500` median < 10 ms. Families:
//! - `rects`: a grid of fully dimensioned rectangles (DOF 0);
//! - `rects-free`: the same rectangles with H/V only (DOF 4 each, every entity `Under`);
//! - `stairs`: one connected staircase polyline, H/V + lengths from a fixed origin (DOF 0);
//! - `stairs-free`: the staircase with lengths only (one under-determined block);
//! - `redundant`: `rects` plus a redundant parallel per rectangle (diagnosis + witness).
//!
//! `sk04/check/<family>/500` is one `check_constraint` of a redundant parallel on a warm
//! analysis cache.

#![allow(clippy::unwrap_used, reason = "bench harness")]

use amg_core::{Plane, Point2};
use amg_sketch::model::HvTarget;
use amg_sketch::solve::{NativeSolver, SolveOptions, SolverBackend};
use amg_sketch::{ConstraintKind as K, Dimension, DimensionKind, EntityId, Sketch};
use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};

fn len(s: &mut Sketch, line: EntityId, v: f64) {
    s.add_constraint(K::Dimension(Dimension {
        kind: DimensionKind::Length { line },
        value: v,
        driving: true,
        label_pos: Point2::ZERO,
    }))
    .unwrap();
}

/// `n` rectangles (8 entities each) on a grid: `dims` adds a fix and two lengths, `extra` a
/// redundant parallel.
fn rects(entities: usize, dims: bool, extra: bool) -> Sketch {
    let mut s = Sketch::new(Plane::XY);
    for i in 0..entities.div_ceil(8) {
        let (x, y) = ((i % 16) as f64 * 30.0, (i / 16) as f64 * 20.0);
        let p: Vec<EntityId> = [(0.0, 0.0), (20.0, 0.0), (20.0, 10.0), (0.0, 10.0)]
            .iter()
            .map(|&(dx, dy)| s.add_point(Point2::new(x + dx, y + dy)).unwrap())
            .collect();
        let l: Vec<EntityId> = (0..4)
            .map(|k| s.add_line_between(p[k], p[(k + 1) % 4]).unwrap())
            .collect();
        for (k, &li) in l.iter().enumerate() {
            let t = HvTarget::Line(li);
            s.add_constraint(if k % 2 == 0 {
                K::Horizontal(t)
            } else {
                K::Vertical(t)
            })
            .unwrap();
        }
        if dims {
            s.add_constraint(K::Fix { entity: p[0] }).unwrap();
            len(&mut s, l[0], 20.0);
            len(&mut s, l[1], 10.0);
        }
        if extra {
            s.add_constraint(K::Parallel { a: l[0], b: l[2] }).unwrap();
        }
    }
    s
}

/// One staircase polyline from a fixed origin (2 entities per segment).
fn stairs(entities: usize, hv: bool) -> Sketch {
    let mut s = Sketch::new(Plane::XY);
    let mut prev = s.add_point(Point2::ZERO).unwrap();
    s.add_constraint(K::Fix { entity: prev }).unwrap();
    let (mut x, mut y) = (0.0, 0.0);
    for i in 0..entities / 2 {
        if i % 2 == 0 {
            x += 5.0;
        } else {
            y += 5.0;
        }
        let q = s.add_point(Point2::new(x, y)).unwrap();
        let l = s.add_line_between(prev, q).unwrap();
        if hv {
            let t = HvTarget::Line(l);
            s.add_constraint(if i % 2 == 0 {
                K::Horizontal(t)
            } else {
                K::Vertical(t)
            })
            .unwrap();
        }
        len(&mut s, l, 5.0);
        prev = q;
    }
    s
}

/// A named sketch generator (entity count → sketch).
type Family = (&'static str, fn(usize) -> Sketch);

fn bench(c: &mut Criterion) {
    let never = || false;
    let mut g = c.benchmark_group("sk04");
    g.sample_size(30);
    let families: [Family; 5] = [
        ("rects", |n| rects(n, true, false)),
        ("rects-free", |n| rects(n, false, false)),
        ("stairs", |n| stairs(n, true)),
        ("stairs-free", |n| stairs(n, false)),
        ("redundant", |n| rects(n, true, true)),
    ];
    for (name, make) in families {
        for n in [50, 200, 500, 2000] {
            let mut s = make(n);
            let mut warm = NativeSolver::new();
            warm.solve(&mut s, &SolveOptions::fast(), &never);
            g.bench_function(BenchmarkId::new(format!("dof_analysis/{name}"), n), |b| {
                b.iter_batched(
                    || warm.clone(),
                    |mut solver| solver.analyze(&s),
                    criterion::BatchSize::SmallInput,
                );
            });
            if n == 500 && name.starts_with("rects") {
                // A redundant parallel on the first rectangle's horizontal sides.
                let lines: Vec<EntityId> = s
                    .entities()
                    .filter(|(_, e)| matches!(e.kind, amg_sketch::EntityKind::Line { .. }))
                    .map(|(id, _)| id)
                    .take(3)
                    .collect();
                let (a, b) = (lines[0], lines[2]);
                let mut solver = warm.clone();
                solver.analyze(&s);
                g.bench_function(BenchmarkId::new(format!("check/{name}"), n), |bch| {
                    bch.iter(|| solver.check_constraint(&s, &K::Parallel { a, b }).unwrap());
                });
            }
        }
    }
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
