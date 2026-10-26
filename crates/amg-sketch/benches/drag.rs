//! SK-05 drag solving: per-frame cost of `DragSession::step` on D4, D7 and the 200-entity truss.
//!
//! `cargo bench -p amg-sketch --bench drag`. Names:
//! - `sk05/p95/<case>/<mode>`: the 95th percentile of the step time over a 1000-frame scripted
//!   drag (Done-when 1, budget 2 ms). Criterion's time is the p95 of the best of three passes
//!   (`iter_custom` reports `p95 × iters`).
//! - `sk05/frame/<case>/<mode>`: mean step time, the session driven back and forth along the
//!   script.
//!
//! `<mode>` is `auto` (minimum norm with SQP fallback, the default), `minnorm` or `sqp`.

#![allow(clippy::unwrap_used, reason = "bench harness")]

use std::path::PathBuf;
use std::time::Duration;

use amg_core::Point2;
use amg_sketch::corpus::drag::{resample, run_script, truss_case};
use amg_sketch::corpus::{self, Case};
use amg_sketch::solve::{
    DragMode, DragSession, DragSpec, NativeSolver, SolveOptions, SolverBackend,
};
use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};

const FRAMES: usize = 1000;

fn decision(n: u32) -> Case {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../testdata/sketch/corpus");
    let tag = format!("decision:D{n}");
    corpus::load_corpus(&root)
        .unwrap()
        .cases
        .into_iter()
        .find(|c| c.tags.contains(&tag))
        .unwrap()
}

fn cases() -> Vec<(&'static str, Case, Vec<[f64; 2]>)> {
    let mut v = Vec::new();
    for (name, case) in [("D4", decision(4)), ("D7", decision(7))] {
        let path = resample(&case.drag.as_ref().unwrap().path, FRAMES);
        v.push((name, case, path));
    }
    let truss = truss_case(FRAMES);
    let path = truss.drag.as_ref().unwrap().path.clone();
    v.push(("truss200", truss, path));
    v
}

fn bench(c: &mut Criterion) {
    let modes = [
        ("auto", DragMode::Auto),
        ("minnorm", DragMode::MinNorm),
        ("sqp", DragMode::Sqp),
    ];
    let mut g = c.benchmark_group("sk05");
    for (name, case, path) in cases() {
        for (label, mode) in modes {
            g.bench_function(BenchmarkId::new(format!("p95/{name}"), label), |b| {
                b.iter_custom(|iters| {
                    let p95 = (0..3)
                        .map(|_| run_script(&case, Some(&path), mode).unwrap().p95_us())
                        .fold(f64::INFINITY, f64::min);
                    Duration::from_secs_f64(p95 * 1e-6 * iters as f64)
                });
            });
            g.bench_function(BenchmarkId::new(format!("frame/{name}"), label), |b| {
                let mut built = case.build().unwrap();
                let mut solver = NativeSolver::new().with_drag_mode(mode);
                solver.solve(&mut built.sketch, &SolveOptions::default(), &|| false);
                let ent = built.entities[case.drag.as_ref().unwrap().entity as usize];
                let grab = built.sketch.point(ent).unwrap();
                let mut session = solver
                    .drag(&built.sketch, DragSpec { entity: ent, grab })
                    .unwrap();
                // Forward then backward along the script, so the cursor never jumps.
                let mut k = 0usize;
                b.iter(|| {
                    let i = k % (2 * path.len() - 2);
                    let i = if i < path.len() {
                        i
                    } else {
                        2 * path.len() - 2 - i
                    };
                    k += 1;
                    let t = path[i];
                    session.step(&mut built.sketch, Point2::new(t[0], t[1]), &|| false)
                });
            });
        }
    }
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
