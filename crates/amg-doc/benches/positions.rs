//! ASM-13 budgets (S06): `SetPendingPositions` of 100 occurrences < 1 ms;
//! `captured_transform_at` with 50 capture nodes < 2 µs. Timing assertions only under
//! `AMG_STRICT_TIMING=1`.

#![allow(clippy::unwrap_used, missing_docs, reason = "benchmark harness")]

use std::hint::black_box;
use std::time::Instant;

use amg_core::{Quat, Transform, Vec3};
use amg_doc::{CapturePositionFeature, Command, Design, FeatureBox, OccurrenceId, apply_command};
use criterion::{Criterion, criterion_group, criterion_main};

fn strict() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}

fn pose(i: usize) -> Transform {
    Transform::from_rotation_translation(
        Quat::from_rotation_z(i as f64 * 0.01),
        Vec3::new(i as f64, 1.0, 2.0),
    )
}

/// 200 occurrences of one component at the root.
fn design() -> (Design, Vec<OccurrenceId>) {
    let mut d = Design::new("asm13-bench");
    let root = d.root();
    let (leaf, first) = d.add_component(root, "leaf", Transform::IDENTITY).unwrap();
    let mut occs = vec![first];
    for i in 1..200 {
        occs.push(d.add_occurrence(root, leaf, pose(i)).unwrap());
    }
    (d, occs)
}

/// Adds 50 capture nodes, each listing 4 occurrences; the first lists `occs[0]` only there.
fn with_captures(d: &mut Design, occs: &[OccurrenceId]) {
    let root = d.root();
    for k in 0..50 {
        let mut moves: Vec<_> = (0..4)
            .map(|j| (occs[1 + (k * 3 + j) % 150], pose(k + j)))
            .collect();
        if k == 0 {
            moves.push((occs[0], pose(7)));
        }
        moves.sort_by_key(|(o, _)| *o);
        let f = CapturePositionFeature {
            moves,
            joint_states: Vec::new(),
        };
        d.add_feature(None, FeatureBox::new(f), root, None).unwrap();
    }
}

fn bench(c: &mut Criterion) {
    let (mut d, occs) = design();
    let moves: Vec<_> = occs.iter().take(100).map(|o| (*o, pose(3))).collect();
    let cmd = Command::SetPendingPositions {
        moves,
        joint_states: Vec::new(),
    };
    let apply = |d: &Design, cmd: &Command| {
        let mut w = d.clone();
        apply_command(&mut w, cmd.clone()).unwrap();
        w
    };
    c.bench_function("asm13/set_pending_positions_100", |b| {
        b.iter(|| black_box(apply(black_box(&d), black_box(&cmd))));
    });

    with_captures(&mut d, &occs);
    let end = d.timeline().len();
    assert_eq!(end, 50);
    // `occs[0]` is listed by the first node only: the worst case for a scan.
    assert!(d.captured_transform_at(occs[0], end).is_some());
    c.bench_function("asm13/captured_transform_at_50", |b| {
        b.iter(|| black_box(d.captured_transform_at(black_box(occs[0]), black_box(end))));
    });

    if strict() {
        let reps = 100u32;
        let mut times: Vec<_> = (0..50)
            .map(|_| {
                let t0 = Instant::now();
                for _ in 0..reps {
                    black_box(apply(black_box(&d), black_box(&cmd)));
                }
                t0.elapsed() / reps
            })
            .collect();
        times.sort();
        let median = times[times.len() / 2];
        assert!(
            median.as_micros() < 1_000,
            "SetPendingPositions of 100: {median:?} (budget 1 ms)"
        );

        let reps = 100_000u32;
        let t0 = Instant::now();
        for _ in 0..reps {
            black_box(d.captured_transform_at(black_box(occs[0]), black_box(end)));
        }
        let per = t0.elapsed() / reps;
        assert!(
            per.as_nanos() < 2_000,
            "captured_transform_at with 50 captures: {per:?} (budget 2 µs)"
        );
    }
}

criterion_group!(benches, bench);
criterion_main!(benches);
