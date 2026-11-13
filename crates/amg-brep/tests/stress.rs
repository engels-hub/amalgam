//! BREP-12 Done-when 5: 8 threads × 2 000 iterations of category-R queries on one shared
//! shape. No crash, every result bit-identical to the single-threaded one, and the input's
//! `BinTools` hash unchanged (S02 §2).
//!
//! Every later op WP adds its op to `common/stress.rs` and runs it here (S02 §Work packages).
#![cfg(feature = "occt")]
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

#[path = "common/stress.rs"]
mod stress;

use std::sync::atomic::{AtomicUsize, Ordering};

use amg_brep::{BrepKernel, KernelConfig, OcctKernel, Shape, SubKind, Validate};
use stress::{StressOp, eval, read_queries, run, run_with, sweep_queries, tess_queries};

const THREADS: usize = 8;
const ITERATIONS: usize = 2_000;

fn load(name: &str) -> Option<Shape> {
    let path = amg_testkit::testdata(&format!("brep/{name}.brep"))?;
    let file = std::fs::File::open(path).expect("open fixture");
    Some(Shape::read_text(std::io::BufReader::new(file)).expect("read fixture"))
}

fn kernel() -> OcctKernel {
    OcctKernel::new(KernelConfig {
        validate: Validate::Always,
        ..KernelConfig::default()
    })
}

/// Stresses `name`: the reference comes from one copy, the threads share a second copy whose
/// lazy index and bbox they build concurrently.
fn stress_fixture(name: &str, seed: u64) {
    let (Some(reference_shape), Some(shared)) = (load(name), load(name)) else {
        return;
    };
    let kernel = kernel();
    let ops: Vec<StressOp> = read_queries(&reference_shape);
    let reference: Vec<String> = ops
        .iter()
        .map(|&op| eval(&kernel, &reference_shape, op))
        .collect();
    let report = run(
        &kernel, &shared, &ops, &reference, THREADS, ITERATIONS, seed,
    );
    eprintln!(
        "{name}: {} threads x {} iterations over {} ops in {:.2} s; BinTools hash {:016x} -> {:016x}",
        report.threads,
        report.iterations,
        report.ops,
        report.elapsed.as_secs_f64(),
        report.hash_before,
        report.hash_after
    );
    assert!(
        report.mismatches.is_empty(),
        "{name}: {} mismatches, first: {}",
        report.mismatches.len(),
        report.mismatches[0]
    );
    assert!(report.input_unchanged, "{name}: the shared input changed");
    assert_eq!(report.hash_before, report.hash_after);
    // Single-threaded again after the storm: still identical.
    for (k, &op) in ops.iter().enumerate().step_by(97) {
        assert_eq!(eval(&kernel, &shared, op), reference[k], "{op:?}");
    }
}

#[test]
fn shared_plate_survives_concurrent_queries() {
    stress_fixture("plate_with_holes", 0x5EED_0001);
}

#[test]
fn shared_filleted_box_survives_concurrent_queries() {
    stress_fixture("filleted_box", 0x5EED_0002);
}

#[test]
fn the_check_is_deterministic_and_valid_on_every_fixture() {
    let kernel = kernel();
    for name in [
        "box",
        "cylinder",
        "cone",
        "sphere",
        "torus",
        "filleted_box",
        "plate_with_holes",
        "located_compound",
        "bspline_patch",
        "revolved_profile",
        "cylinder_mirrored",
    ] {
        let Some(shape) = load(name) else { return };
        let first = eval(&kernel, &shape, StressOp::Check);
        assert!(first.contains("valid: true"), "{name}: {first}");
        assert_eq!(eval(&kernel, &shape, StressOp::Check), first);
        kernel.validate_result(name, &shape).unwrap();
        assert!(shape.max_tolerance() >= 1e-7, "{name}");
    }
}

/// BREP-08 Done-when 4: 8 threads tessellate one shared shape (whole-shape meshes at two
/// presets, edge samples, a few reads) while another thread runs non-destructive booleans on
/// it. No crash, every mesh bit-identical to the single-threaded one, every boolean result the
/// same, and the input's `BinTools` hash unchanged.
fn stress_tessellation(name: &str, iterations: usize, seed: u64) {
    let (Some(reference_shape), Some(shared)) = (load(name), load(name)) else {
        return;
    };
    let kernel = kernel();
    let mut ops = tess_queries(&reference_shape);
    // Reads too (no `Check`: the harness would start every worker with it instead).
    ops.extend([StressOp::Bbox, StressOp::Counts, StressOp::MaxTolerance]);
    let reference: Vec<String> = ops
        .iter()
        .map(|&op| eval(&kernel, &reference_shape, op))
        .collect();
    // The tool: a 10 x 20 x 30 box overlapping the part's lower corner.
    let corner = reference_shape.bbox().min - amg_core::Vec3::splat(3.0);
    let frame = amg_core::Plane {
        origin: corner,
        ..amg_core::Plane::XY
    };
    let tool = kernel
        .make_box(&frame, amg_core::Vec3::new(10.0, 20.0, 30.0))
        .unwrap()
        .shape;
    let cut_reference = kernel.boolean_cut_probe(&reference_shape, &tool).unwrap();
    let cut_volume = cut_reference.volume().unwrap();
    let cut_faces = cut_reference.count(SubKind::Face);
    assert!(cut_volume < reference_shape.volume().unwrap());
    let bad_cuts = AtomicUsize::new(0);
    let (report, cuts) = run_with(
        &kernel,
        &shared,
        &ops,
        &reference,
        THREADS,
        iterations,
        seed,
        || {
            let cut = kernel.boolean_cut_probe(&shared, &tool).expect("cut");
            let same = cut.count(SubKind::Face) == cut_faces
                && cut.volume().unwrap().to_bits() == cut_volume.to_bits();
            if !same {
                bad_cuts.fetch_add(1, Ordering::Relaxed);
            }
        },
    );
    eprintln!(
        "{name}: {} threads x {} iterations over {} ops, {cuts} booleans alongside, {:.2} s; \
         BinTools hash {:016x} -> {:016x}",
        report.threads,
        report.iterations,
        report.ops,
        report.elapsed.as_secs_f64(),
        report.hash_before,
        report.hash_after
    );
    assert!(
        report.mismatches.is_empty(),
        "{name}: {} mismatches, first: {}",
        report.mismatches.len(),
        report.mismatches[0]
    );
    assert!(cuts >= 1);
    assert_eq!(bad_cuts.into_inner(), 0, "{name}: a boolean result changed");
    assert!(report.input_unchanged, "{name}: the shared input changed");
    assert_eq!(report.hash_before, report.hash_after);
    // Single-threaded again afterwards: still the reference (no triangulation was left on the
    // shared input to be reused).
    for (k, &op) in ops.iter().enumerate() {
        assert_eq!(eval(&kernel, &shared, op), reference[k], "{op:?}");
    }
}

#[test]
fn shared_filleted_box_tessellates_during_booleans() {
    stress_tessellation("filleted_box", 40, 0x5EED_0801);
}

#[test]
fn shared_plate_tessellates_during_booleans() {
    stress_tessellation("plate_with_holes", 16, 0x5EED_0802);
}

/// BREP-14 Done-when 4: 8 threads run prisms (straight and tapered) and revols of the faces of
/// one shared shape, alongside reads. The sweeps share the input's TShapes through `sub_shape`,
/// so this also checks that they sweep a private copy: the input's `BinTools` hash must not
/// change, and every result (volume bits, counts, history) must equal the single-threaded one.
fn stress_sweeps(name: &str, limit: usize, iterations: usize, seed: u64) {
    let (Some(reference_shape), Some(shared)) = (load(name), load(name)) else {
        return;
    };
    let kernel = kernel();
    let mut ops = sweep_queries(&reference_shape, limit);
    assert!(!ops.is_empty(), "{name}: no planar faces");
    ops.extend([StressOp::Bbox, StressOp::Counts, StressOp::Check]);
    let reference: Vec<String> = ops
        .iter()
        .map(|&op| eval(&kernel, &reference_shape, op))
        .collect();
    let failed = reference.iter().filter(|r| r.starts_with("Err")).count();
    eprintln!(
        "{name}: {} sweep ops, {failed} of them errors",
        ops.len() - 3
    );
    let report = run(
        &kernel, &shared, &ops, &reference, THREADS, iterations, seed,
    );
    eprintln!(
        "{name}: {} threads x {} iterations over {} ops in {:.2} s; BinTools hash {:016x} -> {:016x}",
        report.threads,
        report.iterations,
        report.ops,
        report.elapsed.as_secs_f64(),
        report.hash_before,
        report.hash_after
    );
    assert!(
        report.mismatches.is_empty(),
        "{name}: {} mismatches, first: {}",
        report.mismatches.len(),
        report.mismatches[0]
    );
    assert!(report.input_unchanged, "{name}: the shared input changed");
    assert_eq!(report.hash_before, report.hash_after);
    for (k, &op) in ops.iter().enumerate() {
        assert_eq!(eval(&kernel, &shared, op), reference[k], "{op:?}");
    }
}

#[test]
fn shared_box_survives_concurrent_sweeps() {
    stress_sweeps("box", 6, 200, 0x5EED_1401);
}

#[test]
fn shared_filleted_box_survives_concurrent_sweeps() {
    stress_sweeps("filleted_box", 6, 60, 0x5EED_1402);
}

#[test]
fn shared_plate_survives_concurrent_sweeps() {
    stress_sweeps("plate_with_holes", 6, 6, 0x5EED_1403);
}
