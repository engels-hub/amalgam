//! BREP-12 Done-when 5: 8 threads × 2 000 iterations of category-R queries on one shared
//! shape. No crash, every result bit-identical to the single-threaded one, and the input's
//! `BinTools` hash unchanged (S02 §2).
//!
//! Every later op WP adds its op to `common/stress.rs` and runs it here (S02 §Work packages).
#![cfg(feature = "occt")]
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

#[path = "common/stress.rs"]
mod stress;

use amg_brep::{KernelConfig, OcctKernel, Shape, Validate};
use stress::{StressOp, eval, read_queries, run};

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
