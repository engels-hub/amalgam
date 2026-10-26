//! SK-20 `pick` + `snap_candidates` on a 500-entity sketch.
//!
//! `cargo bench -p amg-sketch --bench query`. Done-when 3 is `sk20/pick_snap/*` < 0.1 ms per
//! query pair. Names: `sk20/pick_snap/held` (a `QueryIndex` kept by the caller),
//! `sk20/pick_snap/cached` (the free functions, per-thread index cache), `sk20/index_build`
//! (one rebuild after an edit) and `sk20/suppress_redundant` (the `check_constraint` filter on
//! a dragged corner of a fully constrained rectangle; outside the 0.1 ms budget).

#![allow(clippy::unwrap_used, reason = "bench harness")]

use amg_core::{Plane, Point2};
use amg_sketch::model::HvTarget;
use amg_sketch::query::{
    PickFilter, QueryIndex, SnapOptions, pick, snap_candidates_with, suppress_redundant,
};
use amg_sketch::solve::{NativeSolver, SolverBackend};
use amg_sketch::{ConstraintKind as K, Dimension, DimensionKind, EntityId, Sketch};
use criterion::{Criterion, criterion_group, criterion_main};
use std::hint::black_box;

fn pt(x: f64, y: f64) -> Point2 {
    Point2::new(x, y)
}

/// About 520 entities: rectangles, circles, arcs, ellipses and splines on a grid.
fn big_sketch() -> Sketch {
    let mut s = Sketch::new(Plane::XY);
    let mut cell = 0usize;
    let mut at = || {
        let c = cell;
        cell += 1;
        pt((c % 20) as f64 * 40.0, (c / 20) as f64 * 40.0)
    };
    for _ in 0..40 {
        let o = at();
        let p: Vec<EntityId> = [(0.0, 0.0), (20.0, 0.0), (20.0, 10.0), (0.0, 10.0)]
            .iter()
            .map(|&(x, y)| s.add_point(o + pt(x, y)).unwrap())
            .collect();
        for i in 0..4 {
            s.add_line_between(p[i], p[(i + 1) % 4]).unwrap();
        }
    }
    for _ in 0..20 {
        s.add_circle(at() + pt(10.0, 10.0), 8.0).unwrap();
    }
    for _ in 0..20 {
        let o = at() + pt(10.0, 10.0);
        s.add_arc(o, o + pt(8.0, 0.0), o + pt(0.0, 8.0)).unwrap();
    }
    for _ in 0..10 {
        let o = at() + pt(15.0, 15.0);
        s.add_ellipse(o, o + pt(10.0, 0.0), 5.0).unwrap();
    }
    for _ in 0..10 {
        let o = at();
        let poles = [o, o + pt(5.0, 20.0), o + pt(25.0, -5.0), o + pt(30.0, 20.0)];
        s.add_spline(3, &poles, None, None).unwrap();
    }
    s
}

fn probes() -> Vec<Point2> {
    (0..256)
        .map(|i| {
            let k = f64::from(i);
            pt((k * 7.31) % 800.0, (k * 3.17) % 400.0)
        })
        .collect()
}

fn rectangle_with_neighbours() -> (Sketch, EntityId) {
    let mut s = big_sketch();
    let p: Vec<EntityId> = [(0.0, -100.0), (20.0, -100.0), (20.0, -90.0), (0.0, -90.0)]
        .iter()
        .map(|&(x, y)| s.add_point(pt(x, y)).unwrap())
        .collect();
    let l: Vec<EntityId> = (0..4)
        .map(|i| s.add_line_between(p[i], p[(i + 1) % 4]).unwrap())
        .collect();
    for (i, &line) in l.iter().enumerate() {
        let t = HvTarget::Line(line);
        s.add_constraint(if i % 2 == 0 {
            K::Horizontal(t)
        } else {
            K::Vertical(t)
        })
        .unwrap();
    }
    s.add_constraint(K::Fix { entity: p[0] }).unwrap();
    for (line, v) in [(l[0], 20.0), (l[1], 10.0)] {
        s.add_constraint(K::Dimension(Dimension {
            kind: DimensionKind::Length { line },
            value: v,
            driving: true,
            label_pos: Point2::ZERO,
        }))
        .unwrap();
    }
    (s, p[2])
}

fn bench(c: &mut Criterion) {
    let s = big_sketch();
    let ix = QueryIndex::new(&s);
    let ps = probes();
    let opts = SnapOptions {
        from: Some(pt(100.0, 100.0)),
        ..SnapOptions::default()
    };
    let mut g = c.benchmark_group("sk20");
    // One iteration is 256 query pairs; criterion reports the time per pair via throughput.
    g.throughput(criterion::Throughput::Elements(ps.len() as u64));
    g.bench_function("pick_snap/held", |b| {
        b.iter(|| {
            for &p in &ps {
                black_box(ix.pick(p, 2.0, PickFilter::ALL));
                black_box(ix.snap_candidates(p, 2.0, &opts));
            }
        });
    });
    g.bench_function("pick_snap/cached", |b| {
        b.iter(|| {
            for &p in &ps {
                black_box(pick(&s, p, 2.0, PickFilter::ALL));
                black_box(snap_candidates_with(&s, p, 2.0, &opts));
            }
        });
    });
    g.throughput(criterion::Throughput::Elements(1));
    g.bench_function("index_build", |b| b.iter(|| black_box(QueryIndex::new(&s))));
    let (rs, corner) = rectangle_with_neighbours();
    let mut solver = NativeSolver::new();
    let _ = solver.analyze(&rs);
    let ropts = SnapOptions {
        moving: Some(corner),
        ..SnapOptions::default()
    };
    let cands = snap_candidates_with(&rs, pt(25.0, -90.2), 1.0, &ropts);
    g.bench_function("suppress_redundant", |b| {
        b.iter(|| black_box(suppress_redundant(&rs, &mut solver, cands.clone())));
    });
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
