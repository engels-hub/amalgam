//! S20 KH-20: the kernel fast path gives the results of the plain OCCT calls, deterministically.
//!
//! - `unify_same_domain` skips `ShapeUpgrade_UnifySameDomain` when no face or edge pair could
//!   merge (`KernelConfig::fast_paths`): byte-identical to the plain run, history and warnings
//!   included, on shapes with and without something to merge; the skip is taken on the
//!   `budget/sol-extrude-cut` result and not on shapes with coplanar neighbours.
//! - Offsets and shells have no fast path (KH-20 log): byte-identical with the flag on and off.
//! - The extrude-cut boolean and the unify are byte-identical over two runs on 1- and 8-thread
//!   rayon pools, and the boolean with OCCT's parallel mode on and off.
#![cfg(feature = "occt")]
#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::print_stderr,
    reason = "test code"
)]

#[path = "common/offset_corpus.rs"]
mod corpus;
#[path = "common/speed_parts.rs"]
mod speed;

use amg_brep::{
    BoolOp, BoolOpts, BrepKernel, JobCtx, KernelConfig, OcctKernel, OffsetJoin, Point2, Shape,
    SubKind, UnifyOpts, Validate,
};
use corpus::shell_opts;

fn kernel(fast: bool) -> OcctKernel {
    OcctKernel::new(KernelConfig {
        validate: Validate::Always,
        fast_paths: fast,
        ..KernelConfig::default()
    })
}

fn bin(s: &Shape) -> Vec<u8> {
    let mut bytes = Vec::new();
    s.write_bin(&mut bytes).unwrap();
    bytes
}

fn boolean(k: &OcctKernel, op: BoolOp, a: &Shape, b: &Shape) -> Shape {
    k.boolean(
        op,
        std::slice::from_ref(a),
        std::slice::from_ref(b),
        &BoolOpts::default(),
        &JobCtx::detached(),
    )
    .unwrap()
    .shape
}

/// Shapes for the unify skip, and whether a face or edge pair could merge.
fn unify_inputs(k: &OcctKernel) -> Vec<(&'static str, Shape, bool)> {
    let (body, tool) = speed::extrude_cut_pair(k);
    let block = corpus::block(k);
    let shifted = speed::prism_of(
        k,
        &speed::polygon(Point2::new(30.0, 8.0), 14.0, 4),
        &[],
        corpus::C,
    );
    let small = speed::prism_of(
        k,
        &speed::polygon(Point2::new(10.0, 8.0), 3.0, 6),
        &[],
        corpus::C + 5.0,
    );
    let star = speed::star_prism(k);
    let star_top = speed::cap_at(k, &star, 20.0);
    let star_shell = k
        .shell(
            &star,
            &[star_top],
            &shell_opts(-0.3, OffsetJoin::Intersection),
            &JobCtx::detached(),
        )
        .unwrap()
        .shape;
    vec![
        // The `budget/sol-extrude-cut` result (300 faces).
        (
            "extrude_cut_198_100",
            boolean(k, BoolOp::Cut, &body, &tool),
            false,
        ),
        ("star_prism", star, false),
        ("star_prism_shelled", star_shell, false),
        (
            "box_minus_hexagon",
            boolean(k, BoolOp::Cut, &block, &small),
            false,
        ),
        // A seam vertex has exactly two edges.
        ("cylinder", corpus::cylinder(k), true),
        ("holed_plate", speed::holed_plate(k), true),
        // Fillet cylinders meet planes and other cylinders only; every vertex has three edges.
        ("filleted_box", corpus::filleted_block(k), false),
        // Coplanar faces to merge.
        (
            "box_fused_with_a_shifted_box",
            boolean(k, BoolOp::Fuse, &block, &shifted),
            true,
        ),
        (
            "box_fused_with_a_hexagon",
            boolean(k, BoolOp::Fuse, &block, &small),
            true,
        ),
    ]
}

#[test]
fn unify_skip_is_byte_identical_to_the_plain_run() {
    let (plain_k, fast_k) = (kernel(false), kernel(true));
    for (name, s, candidates) in unify_inputs(&fast_k) {
        for opts in [
            UnifyOpts::default(),
            UnifyOpts {
                edges: false,
                ..UnifyOpts::default()
            },
        ] {
            let t0 = std::time::Instant::now();
            let p = plain_k.unify_same_domain(&s, &opts).unwrap();
            let t1 = std::time::Instant::now();
            let f = fast_k.unify_same_domain(&s, &opts).unwrap();
            let t2 = std::time::Instant::now();
            let could = fast_k.unify_candidates(&s, &opts).unwrap();
            eprintln!(
                "{name} (edges {}): {} -> {} faces, candidates {could}, plain {:.2} ms, fast {:.2} ms",
                opts.edges,
                s.count(SubKind::Face),
                p.shape.count(SubKind::Face),
                (t1 - t0).as_secs_f64() * 1e3,
                (t2 - t1).as_secs_f64() * 1e3
            );
            assert_eq!(bin(&p.shape), bin(&f.shape), "{name}: bytes");
            assert_eq!(p.history, f.history, "{name}: history");
            assert_eq!(p.warnings, f.warnings, "{name}: warnings");
            if opts.edges {
                assert_eq!(could, candidates, "{name}: candidates");
            }
            if !could {
                // Nothing could merge, so the plain run merged nothing either.
                assert_eq!(
                    p.shape.count(SubKind::Face),
                    s.count(SubKind::Face),
                    "{name}"
                );
                assert_eq!(
                    p.shape.count(SubKind::Edge),
                    s.count(SubKind::Edge),
                    "{name}"
                );
            }
        }
    }
}

#[test]
fn offsets_and_shells_are_unchanged_by_the_flag() {
    let (plain_k, fast_k) = (kernel(false), kernel(true));
    for case in corpus::corpus(&fast_k) {
        let p = case.run(&plain_k).unwrap();
        let f = case.run(&fast_k).unwrap();
        assert_eq!(bin(&p.shape), bin(&f.shape), "{}", case.name);
        assert_eq!(p.history, f.history, "{}", case.name);
    }
}

fn in_pool<T: Send>(threads: usize, f: impl FnOnce() -> T + Send) -> T {
    rayon::ThreadPoolBuilder::new()
        .num_threads(threads)
        .build()
        .unwrap()
        .install(f)
}

#[test]
fn extrude_cut_and_unify_are_deterministic() {
    let k = kernel(true);
    let (body, tool) = speed::extrude_cut_pair(&k);
    let run = || {
        let cut = k
            .boolean(
                BoolOp::Cut,
                std::slice::from_ref(&body),
                std::slice::from_ref(&tool),
                &BoolOpts::default(),
                &JobCtx::detached(),
            )
            .unwrap();
        let u = k
            .unify_same_domain(&cut.shape, &UnifyOpts::default())
            .unwrap();
        (
            bin(&cut.shape),
            bin(&u.shape),
            format!("{:?} {:?}", cut.history, u.history),
        )
    };
    let first = in_pool(1, run);
    for threads in [1, 8] {
        for round in 0..2 {
            assert!(
                in_pool(threads, run) == first,
                "{threads} threads, round {round}: results differ"
            );
        }
    }
}

#[test]
fn the_extrude_cut_boolean_is_the_same_serial_and_parallel() {
    let k = kernel(true);
    let (body, tool) = speed::extrude_cut_pair(&k);
    let cut = |parallel| {
        k.boolean(
            BoolOp::Cut,
            std::slice::from_ref(&body),
            std::slice::from_ref(&tool),
            &BoolOpts {
                parallel: Some(parallel),
                ..BoolOpts::default()
            },
            &JobCtx::detached(),
        )
        .unwrap()
    };
    let (serial, parallel) = (cut(false), cut(true));
    assert_eq!(bin(&serial.shape), bin(&parallel.shape));
    assert_eq!(serial.history, parallel.history);
    assert_eq!(body.count(SubKind::Face), 200);
    assert_eq!(serial.shape.count(SubKind::Face), 300);
}
