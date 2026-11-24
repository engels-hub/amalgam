//! BREP-04: `boolean` (fuse/cut/common, N objects × M tools), `split` and `section`.
//!
//! Done-when 1 (the corpus `testdata/brep/bool/cases.toml` gives valid results with volumes
//! within 1e-6 relative of the reference, or the expected `Err`), 2 (proptest: 500 random
//! primitive pairs give `Ok(valid)` or a clean `Err`), 3 (the inputs' `BinTools` hashes are
//! unchanged after every op), 4 (20 cases × 10 runs with parallel on are byte-identical) and 6
//! (cancelling a long boolean returns `Cancelled` within 200 ms; asserted only with
//! `AMG_STRICT_TIMING=1`). Done-when 5 is the criterion bench `benches/boolean.rs`.
#![cfg(feature = "occt")]
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

#[path = "common/bool_corpus.rs"]
mod bool_corpus;
mod common;
#[path = "common/parts.rs"]
mod parts;

use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};
use std::path::PathBuf;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::{Duration, Instant};

use amg_brep::{
    BoolOp, BoolOpts, BrepKernel, InputSub, JobCtx, KernelConfig, KernelError, KernelWarning,
    OcctKernel, OpResult, OpRole, Plane, Point3, Relation, Shape, ShapeKind, SubKind, SubShape,
    Tolerance, Validate, Vec3,
};
use amg_jobs::CancelToken;
use proptest::prelude::*;

use bool_corpus::{Case, corpus, frame};

const TOL: Tolerance = Tolerance::DEFAULT;

/// Done-when 1 bound.
const REL: f64 = 1e-6;

fn kernel() -> OcctKernel {
    OcctKernel::new(KernelConfig {
        validate: Validate::Always,
        ..KernelConfig::default()
    })
}

fn ctx() -> JobCtx {
    JobCtx::detached()
}

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}

fn bin(s: &Shape) -> Vec<u8> {
    let mut bytes = Vec::new();
    s.write_bin(&mut bytes).unwrap();
    bytes
}

fn bin_hash(s: &Shape) -> u64 {
    let mut h = DefaultHasher::new();
    bin(s).hash(&mut h);
    h.finish()
}

fn counts(s: &Shape) -> [u32; 3] {
    [
        s.count(SubKind::Face),
        s.count(SubKind::Edge),
        s.count(SubKind::Vertex),
    ]
}

/// The volume by adaptive integration (`Shape::volume`'s fixed-order quadrature is off by up to
/// 1e-4 relative on obliquely trimmed spheres, see the Refinement log).
fn volume(s: &Shape) -> f64 {
    if s.count(SubKind::Face) == 0 {
        0.0
    } else {
        s.volume_adaptive(1e-10).unwrap()
    }
}

fn face(index: u32) -> SubShape {
    SubShape {
        kind: SubKind::Face,
        index,
    }
}

fn error_name(e: &KernelError) -> &'static str {
    match e {
        KernelError::InvalidInput { .. } => "InvalidInput",
        KernelError::OpFailed { .. } => "OpFailed",
        KernelError::InvalidResult { .. } => "InvalidResult",
        KernelError::Occt { .. } => "Occt",
        KernelError::Cancelled => "Cancelled",
        _ => "other",
    }
}

/// Every output index in `h` is in range for `out`, and there is one operand count per operand.
fn check_history_ranges(
    h: &amg_brep::ShapeHistory,
    operands: &[&Shape],
    out: &Shape,
) -> Result<(), String> {
    if h.operand_counts.len() != operands.len() {
        return Err(format!(
            "{} operand counts for {} operands",
            h.operand_counts.len(),
            operands.len()
        ));
    }
    let limit = |s: SubShape| out.count(s.kind);
    for (input, relation) in &h.entries {
        let operand = usize::from(input.operand);
        if operand >= operands.len() || input.sub.index >= operands[operand].count(input.sub.kind) {
            return Err(format!("input {input:?} out of range"));
        }
        let outs: Vec<SubShape> = match relation {
            Relation::Modified(o) | Relation::Generated(o) => o.to_vec(),
            Relation::Kept(o) => vec![*o],
            Relation::Deleted => vec![],
        };
        if let Some(bad) = outs.iter().find(|o| o.index >= limit(**o)) {
            return Err(format!("output {bad:?} of {input:?} out of range"));
        }
    }
    Ok(())
}

/// Done-when 1 and 3 over the whole corpus.
#[test]
fn corpus_gives_reference_volumes_or_expected_errors() {
    let Some(cases) = corpus() else { return };
    assert!(cases.len() >= 100, "{} cases", cases.len());
    let k = kernel();
    let mut failures = Vec::new();
    let (mut ok, mut errs, mut worst, mut worst_plain) = (0, 0, 0.0_f64, 0.0_f64);
    for case in &cases {
        assert!(!case.source.is_empty(), "{}: no source", case.name);
        let (objects, tools) = case.operands(&k);
        let before: Vec<u64> = objects.iter().chain(&tools).map(bin_hash).collect();
        let result = case.run(&k, &objects, &tools, None);
        let after: Vec<u64> = objects.iter().chain(&tools).map(bin_hash).collect();
        if before != after {
            failures.push(format!("{}: an input's BinTools hash changed", case.name));
        }
        match (&case.error, result) {
            (Some(want), Err(e)) if error_name(&e) == want => {
                errs += 1;
                eprintln!("{}: expected {want}: {e}", case.name);
            }
            (Some(want), other) => {
                failures.push(format!(
                    "{}: expected {want}, got {:?}",
                    case.name,
                    other.map(|r| volume(&r.shape))
                ));
            }
            (None, Err(e)) => failures.push(format!("{}: {e} ({e:?})", case.name)),
            (None, Ok(r)) => {
                let got = volume(&r.shape);
                let want = case.volume.expect("a case without error has a volume");
                let err = if want == 0.0 {
                    got.abs()
                } else {
                    (got - want).abs() / want.abs()
                };
                let bound = if want == 0.0 { 1e-9 } else { REL };
                worst = worst.max(if want == 0.0 { 0.0 } else { err });
                if err > bound {
                    failures.push(format!(
                        "{}: volume {got} vs {want} (error {err:e})",
                        case.name
                    ));
                }
                let solids = r.shape.solids().len();
                if let Some(n) = case.solids
                    && solids != n
                {
                    failures.push(format!("{}: {solids} solids, want {n}", case.name));
                }
                let operands: Vec<&Shape> = objects.iter().chain(&tools).collect();
                if let Err(e) = check_history_ranges(&r.history, &operands, &r.shape) {
                    failures.push(format!("{}: history {e}", case.name));
                }
                if !k.check(&r.shape, &Default::default()).valid {
                    failures.push(format!("{}: invalid result", case.name));
                }
                ok += 1;
                let plain = if want == 0.0 {
                    0.0
                } else {
                    (r.shape.volume().unwrap() - want).abs() / want.abs()
                };
                worst_plain = worst_plain.max(plain);
                eprintln!(
                    "{}: volume {got} vs {want}, error {err:.2e} (fixed-order {plain:.2e}), {solids} solids",
                    case.name
                );
            }
        }
    }
    eprintln!(
        "corpus: {} cases, {ok} valid results (largest relative volume error {worst:.2e}, {worst_plain:.2e} with Shape::volume), {errs} expected errors",
        cases.len()
    );
    assert!(
        failures.is_empty(),
        "{} failures:\n{}",
        failures.len(),
        failures.join("\n")
    );
}

/// Done-when 4: the 20 `determinism` cases, 10 runs each with OCCT parallel mode on, give
/// byte-identical `BinTools` output.
#[test]
fn parallel_mode_is_deterministic() {
    let Some(cases) = corpus() else { return };
    let chosen: Vec<&Case> = cases.iter().filter(|c| c.determinism).collect();
    assert_eq!(chosen.len(), 20);
    let k = kernel();
    let pool = amg_occt_sys::ffi::default_thread_pool_size().unwrap();
    let mut differing = Vec::new();
    for case in &chosen {
        let (objects, tools) = case.operands(&k);
        let first = bin(&case.run(&k, &objects, &tools, Some(true)).unwrap().shape);
        let serial = bin(&case.run(&k, &objects, &tools, Some(false)).unwrap().shape);
        for run in 1..10 {
            let again = bin(&case.run(&k, &objects, &tools, Some(true)).unwrap().shape);
            if again != first {
                differing.push(format!("{} (run {run})", case.name));
                break;
            }
        }
        eprintln!(
            "{}: {} bytes, parallel {} serial",
            case.name,
            first.len(),
            if first == serial { "==" } else { "!=" }
        );
    }
    eprintln!(
        "determinism: {} cases x 10 runs, OCCT pool {pool} threads, {} differ",
        chosen.len(),
        differing.len()
    );
    assert!(differing.is_empty(), "not byte-identical: {differing:?}");
}

// ---- proptest (Done-when 2) ---------------------------------------------------------------

#[derive(Clone, Debug)]
struct RandomPrim {
    kind: u8,
    origin: [f64; 3],
    axis: [f64; 3],
    a: f64,
    b: f64,
    c: f64,
}

fn random_prim() -> impl Strategy<Value = RandomPrim> {
    (
        0u8..5,
        prop::array::uniform3(-4.0f64..4.0),
        prop::array::uniform3(-1.0f64..1.0),
        1.0f64..8.0,
        0.2f64..1.0,
        1.0f64..8.0,
    )
        .prop_map(|(kind, origin, axis, a, b, c)| RandomPrim {
            kind,
            origin,
            axis,
            a,
            b,
            c,
        })
}

fn build_random(k: &OcctKernel, p: &RandomPrim) -> Shape {
    use std::f64::consts::TAU;
    let axis = if Vec3::from(p.axis).length() < 0.1 {
        [0.0, 0.0, 1.0]
    } else {
        p.axis
    };
    let f = frame(p.origin, Some(axis), None);
    let made = match p.kind {
        0 => k.make_box(&f, Vec3::new(p.a, p.c, p.a * p.b + 0.5)),
        1 => k.make_cylinder(&f, p.a * 0.5, p.c, TAU),
        2 => k.make_cone(&f, p.a * 0.5, p.a * 0.5 * p.b * 0.9, p.c, TAU),
        3 => k.make_sphere(&f, p.a * 0.6),
        _ => k.make_torus(&f, p.a * 0.5 + 0.6, (p.a * 0.5 + 0.6) * p.b * 0.5, TAU),
    };
    made.unwrap().shape
}

static PROP_OK: AtomicUsize = AtomicUsize::new(0);
static PROP_ERR: AtomicUsize = AtomicUsize::new(0);

proptest! {
    #![proptest_config(ProptestConfig { cases: 500, failure_persistence: None, ..ProptestConfig::default() })]

    #[test]
    fn random_primitive_pairs_give_valid_results_or_clean_errors(
        a in random_prim(),
        b in random_prim(),
        op in 0u8..4,
    ) {
        let k = kernel();
        let (sa, sb) = (build_random(&k, &a), build_random(&k, &b));
        let (ha, hb) = (bin_hash(&sa), bin_hash(&sb));
        let objects = [sa];
        let tools = [sb];
        let result = match op {
            0 => k.boolean(BoolOp::Fuse, &objects, &tools, &BoolOpts::default(), &ctx()),
            1 => k.boolean(BoolOp::Cut, &objects, &tools, &BoolOpts::default(), &ctx()),
            2 => k.boolean(BoolOp::Common, &objects, &tools, &BoolOpts::default(), &ctx()),
            _ => k.split(&objects, &tools, &BoolOpts::default(), &ctx()),
        };
        match &result {
            Ok(r) => {
                // `Validate::Always` already rejected invalid results; check again directly.
                prop_assert!(k.check(&r.shape, &Default::default()).valid);
                PROP_OK.fetch_add(1, Ordering::Relaxed);
            }
            Err(e) => {
                prop_assert!(matches!(
                    e,
                    KernelError::OpFailed { .. } | KernelError::InvalidResult { .. } | KernelError::Occt { .. }
                ), "unexpected error {e:?}");
                PROP_ERR.fetch_add(1, Ordering::Relaxed);
            }
        }
        prop_assert_eq!(bin_hash(&objects[0]), ha);
        prop_assert_eq!(bin_hash(&tools[0]), hb);
        let (ok, err) = (PROP_OK.load(Ordering::Relaxed), PROP_ERR.load(Ordering::Relaxed));
        if (ok + err) % 100 == 0 {
            eprintln!("proptest: {} pairs, {ok} Ok(valid), {err} Err", ok + err);
        }
    }
}

// ---- history, options, errors ---------------------------------------------------------------

/// A 10 × 10 × 10 box with a blind hole r = 2 from the top (cylinder from z = 6 to 12).
fn blind_hole(k: &OcctKernel) -> (Shape, Shape, OpResult) {
    let block = k.make_box(&Plane::XY, Vec3::splat(10.0)).unwrap().shape;
    let drill = k
        .make_cylinder(
            &frame([5.0, 5.0, 6.0], None, None),
            2.0,
            6.0,
            std::f64::consts::TAU,
        )
        .unwrap()
        .shape;
    let r = k
        .boolean(
            BoolOp::Cut,
            std::slice::from_ref(&block),
            std::slice::from_ref(&drill),
            &BoolOpts::default(),
            &ctx(),
        )
        .unwrap();
    (block, drill, r)
}

#[test]
fn cut_history_keeps_modifies_and_covers_every_face() {
    let k = kernel();
    let (block, drill, r) = blind_hole(&k);
    let want = 1000.0 - std::f64::consts::PI * 4.0 * 4.0;
    assert!((r.shape.volume().unwrap() - want).abs() < 1e-9 * want);
    let h = &r.history;
    assert_eq!(h.operand_counts, vec![counts(&block), counts(&drill)]);
    // The box's top face (z = 10) is modified into the face with the hole; the five others are
    // kept as they are.
    let top = block
        .faces()
        .find(|&f| {
            let bb = block.sub_shape(face(f.0)).unwrap().bbox();
            (bb.min.z - 10.0).abs() < 1e-9 && (bb.max.z - 10.0).abs() < 1e-9
        })
        .unwrap();
    let (mut kept, mut modified) = (0, 0);
    for f in block.faces() {
        let input = InputSub {
            operand: 0,
            sub: face(f.0),
        };
        // Besides its face relation, the top face also `Generated` the hole's rim edge and seam
        // vertex (the section of the drill with it).
        let relations: Vec<&Relation> = h
            .of_input(input)
            .filter(|r| !matches!(r, Relation::Generated(_)))
            .collect();
        assert_eq!(relations.len(), 1, "{f:?}: {relations:?}");
        match relations[0] {
            Relation::Kept(_) => kept += 1,
            Relation::Modified(outs) => {
                assert_eq!(f, top);
                assert_eq!(outs.len(), 1);
                modified += 1;
                let generated: Vec<&Relation> = h
                    .of_input(input)
                    .filter(|r| matches!(r, Relation::Generated(_)))
                    .collect();
                assert_eq!(generated.len(), 1, "{generated:?}");
            }
            other => panic!("{f:?}: {other:?}"),
        }
    }
    assert_eq!((kept, modified), (5, 1));
    // The drill's lateral face becomes the hole wall, its top cap is gone, its bottom cap is the
    // hole's floor.
    let mut deleted = 0;
    for f in drill.faces() {
        let input = InputSub {
            operand: 1,
            sub: face(f.0),
        };
        if h.of_input(input).any(|r| *r == Relation::Deleted) {
            deleted += 1;
        }
    }
    assert_eq!(deleted, 1);
    // Completeness (BREP-05 rule): every result face comes from some input.
    for f in r.shape.faces() {
        assert!(!h.sources_of(face(f.0)).is_empty(), "{f:?} has no source");
    }
    assert_eq!(r.shape.count(SubKind::Face), 8);
    assert!(r.warnings.is_empty(), "{:?}", r.warnings);
}

#[test]
fn n_objects_by_m_tools_number_operands_objects_first() {
    let k = kernel();
    let tau = std::f64::consts::TAU;
    let objects = [
        k.make_box(&Plane::XY, Vec3::new(10.0, 10.0, 5.0))
            .unwrap()
            .shape,
        k.make_box(
            &frame([20.0, 0.0, 0.0], None, None),
            Vec3::new(10.0, 10.0, 5.0),
        )
        .unwrap()
        .shape,
    ];
    let tools = [
        k.make_cylinder(&frame([5.0, 5.0, -1.0], None, None), 2.0, 7.0, tau)
            .unwrap()
            .shape,
        k.make_cylinder(&frame([25.0, 5.0, -1.0], None, None), 3.0, 7.0, tau)
            .unwrap()
            .shape,
    ];
    let r = k
        .boolean(BoolOp::Cut, &objects, &tools, &BoolOpts::default(), &ctx())
        .unwrap();
    assert_eq!(r.history.operand_counts.len(), 4);
    let solids = r.shape.solids();
    assert_eq!(solids.len(), 2);
    // Each tool's lateral face becomes a hole wall in its own box only: the wall faces from
    // operand 3 are cylinders of radius 3.
    for (input, relation) in &r.history.entries {
        if input.operand == 3
            && input.sub.kind == SubKind::Face
            && let Relation::Modified(outs) = relation
        {
            for o in outs {
                match r.shape.surface(amg_brep::FaceId(o.index)) {
                    amg_brep::geom::Surface::Cylinder { radius, .. } => {
                        assert!((radius - 3.0).abs() < 1e-12, "{radius}");
                    }
                    other => panic!("{other:?}"),
                }
            }
        }
    }
    // A fuse without tools fuses the objects with each other; operands stay in order.
    let fused = k
        .boolean(BoolOp::Fuse, &objects, &[], &BoolOpts::default(), &ctx())
        .unwrap();
    assert_eq!(fused.history.operand_counts.len(), 2);
    assert_eq!(fused.shape.solids().len(), 2);
}

#[test]
fn split_keeps_every_piece() {
    let k = kernel();
    let block = k.make_box(&Plane::XY, Vec3::splat(10.0)).unwrap().shape;
    let pts = [
        [-1.0, -1.0, 3.0],
        [11.0, -1.0, 3.0],
        [11.0, 11.0, 3.0],
        [-1.0, 11.0, 3.0],
    ]
    .map(Point3::from);
    let blade = k.make_planar_polygon(&pts, &[], &TOL).unwrap();
    let r = k
        .split(
            std::slice::from_ref(&block),
            std::slice::from_ref(&blade),
            &BoolOpts::default(),
            &ctx(),
        )
        .unwrap();
    let mut volumes: Vec<f64> = r
        .shape
        .solids()
        .iter()
        .map(|s| s.volume().unwrap())
        .collect();
    volumes.sort_by(f64::total_cmp);
    assert_eq!(volumes.len(), 2);
    assert!(
        (volumes[0] - 300.0).abs() < 1e-9 && (volumes[1] - 700.0).abs() < 1e-9,
        "{volumes:?}"
    );
    // The four side faces are each split in two.
    let split_sides = block
        .faces()
        .filter(|f| {
            let input = InputSub {
                operand: 0,
                sub: face(f.0),
            };
            r.history
                .of_input(input)
                .any(|rel| matches!(rel, Relation::Modified(o) if o.len() == 2))
        })
        .count();
    assert_eq!(split_sides, 4);
}

#[test]
fn section_gives_tagged_edges_with_face_history() {
    let k = kernel();
    let block = k.make_box(&Plane::XY, Vec3::splat(10.0)).unwrap().shape;
    let rod = k
        .make_cylinder(
            &frame([5.0, 5.0, -5.0], None, None),
            2.0,
            20.0,
            std::f64::consts::TAU,
        )
        .unwrap()
        .shape;
    let r = k.section(&block, &rod, &ctx()).unwrap();
    // Two circles where the rod crosses the top and bottom faces.
    let edges = r.shape.count(SubKind::Edge);
    assert!(edges >= 2, "{edges} edges");
    let mut length = 0.0;
    for e in r.shape.edges() {
        let sub = SubShape {
            kind: SubKind::Edge,
            index: e.0,
        };
        assert_eq!(r.history.role_of(sub), Some(&OpRole::SectionEdge));
        match r.shape.curve(e) {
            Some((amg_brep::geom::Curve3::Circle { radius, .. }, t0, t1)) => {
                assert!((radius - 2.0).abs() < 1e-9);
                length += radius * (t1 - t0);
            }
            other => panic!("{other:?}"),
        }
        assert!(
            !r.history.sources_of(sub).is_empty(),
            "{sub:?} has no source"
        );
    }
    // Two full circles of radius 2.
    assert!(
        (length - 2.0 * 2.0 * std::f64::consts::TAU).abs() < 1e-9,
        "{length}"
    );
    assert_eq!(r.history.operand_counts.len(), 2);
    // Disjoint shapes: an empty section.
    let far = k
        .make_box(&frame([50.0, 0.0, 0.0], None, None), Vec3::ONE)
        .unwrap()
        .shape;
    let none = k.section(&block, &far, &ctx()).unwrap();
    assert_eq!(none.shape.count(SubKind::Edge), 0);
}

#[test]
fn failures_name_the_alert_and_inputs_stay_untouched() {
    let k = kernel();
    let block = k.make_box(&Plane::XY, Vec3::splat(10.0)).unwrap().shape;
    let pts = [
        [-1.0, -1.0, 5.0],
        [11.0, -1.0, 5.0],
        [11.0, 11.0, 5.0],
        [-1.0, 11.0, 5.0],
    ]
    .map(Point3::from);
    let sheet = k.make_planar_polygon(&pts, &[], &TOL).unwrap();
    match k.boolean(
        BoolOp::Fuse,
        std::slice::from_ref(&block),
        std::slice::from_ref(&sheet),
        &BoolOpts::default(),
        &ctx(),
    ) {
        Err(KernelError::OpFailed { op, message, .. }) => {
            assert_eq!(op, "boolean");
            assert!(message.contains("BOPAlgo_Alert"), "{message}");
            eprintln!("fuse of a solid and a face: {message}");
        }
        other => panic!("expected OpFailed, got {other:?}"),
    }
    let bad = BoolOpts {
        fuzzy: f64::NAN,
        ..BoolOpts::default()
    };
    assert!(matches!(
        k.boolean(
            BoolOp::Cut,
            std::slice::from_ref(&block),
            std::slice::from_ref(&block),
            &bad,
            &ctx()
        ),
        Err(KernelError::InvalidInput { what: "fuzzy", .. })
    ));
    assert!(matches!(
        k.boolean(
            BoolOp::Common,
            std::slice::from_ref(&block),
            &[],
            &BoolOpts::default(),
            &ctx()
        ),
        Err(KernelError::InvalidInput { what: "tools", .. })
    ));
    assert!(matches!(
        k.split(
            &[],
            std::slice::from_ref(&block),
            &BoolOpts::default(),
            &ctx()
        ),
        Err(KernelError::InvalidInput {
            what: "objects",
            ..
        })
    ));
    // A cancelled context stops before the run.
    let token = CancelToken::new();
    token.cancel();
    assert!(matches!(
        k.boolean(
            BoolOp::Cut,
            std::slice::from_ref(&block),
            std::slice::from_ref(&sheet),
            &BoolOpts::default(),
            &JobCtx::with_token(token)
        ),
        Err(KernelError::Cancelled)
    ));
}

#[test]
fn fuzzy_joins_a_gap_and_says_so() {
    let k = kernel();
    let a = k.make_box(&Plane::XY, Vec3::splat(10.0)).unwrap().shape;
    let b = k
        .make_box(
            &frame([10.0 + 1e-5, 0.0, 0.0], None, None),
            Vec3::splat(10.0),
        )
        .unwrap()
        .shape;
    let exact = k
        .boolean(
            BoolOp::Fuse,
            std::slice::from_ref(&a),
            std::slice::from_ref(&b),
            &BoolOpts::default(),
            &ctx(),
        )
        .unwrap();
    assert_eq!(exact.shape.solids().len(), 2);
    assert!(exact.warnings.is_empty());
    let opts = BoolOpts {
        fuzzy: 1e-4,
        ..BoolOpts::default()
    };
    let fuzzy = k
        .boolean(
            BoolOp::Fuse,
            std::slice::from_ref(&a),
            std::slice::from_ref(&b),
            &opts,
            &ctx(),
        )
        .unwrap();
    assert_eq!(fuzzy.shape.solids().len(), 1);
    assert!(
        fuzzy.warnings.contains(&KernelWarning::FuzzyUsed(1e-4)),
        "{:?}",
        fuzzy.warnings
    );
    assert!(
        fuzzy
            .warnings
            .iter()
            .any(|w| matches!(w, KernelWarning::ToleranceGrown { max } if *max >= 5e-6)),
        "{:?}",
        fuzzy.warnings
    );
    // OBB, check-inverted off and serial mode change nothing for a plain case.
    let (_, _, reference) = blind_hole(&k);
    let block = k.make_box(&Plane::XY, Vec3::splat(10.0)).unwrap().shape;
    let drill = k
        .make_cylinder(
            &frame([5.0, 5.0, 6.0], None, None),
            2.0,
            6.0,
            std::f64::consts::TAU,
        )
        .unwrap()
        .shape;
    for opts in [
        BoolOpts {
            use_obb: true,
            ..BoolOpts::default()
        },
        BoolOpts {
            check_inverted: false,
            ..BoolOpts::default()
        },
        BoolOpts {
            parallel: Some(false),
            ..BoolOpts::default()
        },
        BoolOpts {
            parallel: Some(true),
            ..BoolOpts::default()
        },
    ] {
        let r = k
            .boolean(
                BoolOp::Cut,
                std::slice::from_ref(&block),
                std::slice::from_ref(&drill),
                &opts,
                &ctx(),
            )
            .unwrap();
        assert_eq!(bin(&r.shape), bin(&reference.shape), "{opts:?}");
        assert_eq!(r.history, reference.history, "{opts:?}");
    }
}

#[test]
fn compound_results_are_compounds() {
    let k = kernel();
    let (_, _, r) = blind_hole(&k);
    assert!(matches!(
        r.shape.kind(),
        ShapeKind::Compound | ShapeKind::Solid
    ));
}

// ---- cancellation (Done-when 6) ---------------------------------------------------------------

/// A cut of two 1030-face plates runs for seconds; cancelling it 300 ms in returns `Cancelled`.
#[test]
fn cancelling_a_long_boolean_returns_cancelled_quickly() {
    let k = OcctKernel::new(KernelConfig {
        validate: Validate::Off,
        ..KernelConfig::default()
    });
    let (object, tool) = parts::plate_pair(&k, 32);
    assert_eq!(object.count(SubKind::Face), 1030);
    let token = CancelToken::new();
    let job = JobCtx::with_token(token.clone());
    let started = Instant::now();
    let latency = std::thread::scope(|scope| {
        let worker = scope.spawn(|| {
            let r = k.boolean(
                BoolOp::Cut,
                std::slice::from_ref(&object),
                std::slice::from_ref(&tool),
                &BoolOpts::default(),
                &job,
            );
            (r.map(|r| r.shape.count(SubKind::Face)), Instant::now())
        });
        std::thread::sleep(Duration::from_millis(300));
        let cancelled_at = Instant::now();
        token.cancel();
        let (result, ended) = worker.join().unwrap();
        assert!(matches!(result, Err(KernelError::Cancelled)), "{result:?}");
        ended.duration_since(cancelled_at)
    });
    eprintln!(
        "cancel: Cancelled {:.1} ms after cancel() ({:.0} ms after the start)",
        latency.as_secs_f64() * 1e3,
        started.elapsed().as_secs_f64() * 1e3
    );
    if strict_timing() {
        assert!(latency < Duration::from_millis(200), "{latency:?}");
    }
}

/// The same cut, run to the end once: how long the cancel test's boolean takes, and that it is
/// valid. Ignored by default (it takes seconds); the numbers are in the S02 Refinement log.
#[test]
#[ignore = "slow: run with --ignored to time the 1030-face cut"]
fn thousand_face_cut_runs_to_completion() {
    let k = kernel();
    let (object, tool) = parts::plate_pair(&k, 32);
    let started = Instant::now();
    let r = k
        .boolean(
            BoolOp::Cut,
            std::slice::from_ref(&object),
            std::slice::from_ref(&tool),
            &BoolOpts::default(),
            &ctx(),
        )
        .unwrap();
    eprintln!(
        "1030-face cut: {:.0} ms, {} faces, {} solids",
        started.elapsed().as_secs_f64() * 1e3,
        r.shape.count(SubKind::Face),
        r.shape.solids().len()
    );
}

// ---- goldens ----------------------------------------------------------------------------------

#[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
struct BrepGolden {
    tolerances: GoldenTolerances,
    metrics: BrepMetrics,
}

#[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
struct GoldenTolerances {
    rel: f64,
}

#[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
struct BrepMetrics {
    volume: f64,
    area: f64,
    solids: u32,
    faces: u32,
    edges: u32,
    vertices: u32,
    classes: String,
}

fn golden_path(case: &str) -> PathBuf {
    amg_testkit::golden_dir()
        .join("brep")
        .join(format!("bool_{case}.toml"))
}

/// Compares `s` with `tests/golden/brep/bool_<case>.toml`; `AMG_BLESS=1` (never under `CI`)
/// writes it instead.
#[track_caller]
fn check_golden(case: &str, s: &Shape) {
    let [faces, edges, vertices] = counts(s);
    let metrics = BrepMetrics {
        volume: volume(s),
        area: s.faces().map(|f| s.face_area(f)).sum(),
        solids: u32::try_from(s.solids().len()).unwrap(),
        faces,
        edges,
        vertices,
        classes: common::face_classes(s),
    };
    let path = golden_path(case);
    if std::env::var("AMG_BLESS").is_ok_and(|v| v == "1") {
        assert!(
            std::env::var_os("CI").is_none(),
            "AMG_BLESS is refused under CI"
        );
        let golden = BrepGolden {
            tolerances: GoldenTolerances { rel: 1e-9 },
            metrics,
        };
        let text = format!(
            "# Exact B-rep metrics of brep/bool_{case} (S02 BREP-04). Written by `AMG_BLESS=1`.\n\n{}",
            toml::to_string(&golden).unwrap()
        );
        std::fs::write(&path, text).unwrap();
        return;
    }
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    let golden: BrepGolden = toml::from_str(&text).unwrap();
    let rel = golden.tolerances.rel;
    let close = |a: f64, b: f64| (a - b).abs() <= rel * b.abs().max(1.0);
    let want = &golden.metrics;
    assert!(
        close(metrics.volume, want.volume),
        "{case} volume {} vs {}",
        metrics.volume,
        want.volume
    );
    assert!(
        close(metrics.area, want.area),
        "{case} area {} vs {}",
        metrics.area,
        want.area
    );
    assert_eq!(
        (
            metrics.solids,
            metrics.faces,
            metrics.edges,
            metrics.vertices,
            &metrics.classes
        ),
        (
            want.solids,
            want.faces,
            want.edges,
            want.vertices,
            &want.classes
        ),
        "{case} topology"
    );
}

#[test]
fn goldens() {
    let k = kernel();
    let (_, _, hole) = blind_hole(&k);
    check_golden("blind_hole", &hole.shape);
    let tau = std::f64::consts::TAU;
    let a = k
        .make_cylinder(&frame([0.0, 0.0, -10.0], None, None), 3.0, 20.0, tau)
        .unwrap()
        .shape;
    let b = k
        .make_cylinder(
            &frame([-10.0, 0.0, 0.0], Some([1.0, 0.0, 0.0]), None),
            3.0,
            20.0,
            tau,
        )
        .unwrap()
        .shape;
    let steinmetz = k
        .boolean(
            BoolOp::Common,
            std::slice::from_ref(&a),
            std::slice::from_ref(&b),
            &BoolOpts::default(),
            &ctx(),
        )
        .unwrap();
    check_golden("steinmetz", &steinmetz.shape);
    let cross = k
        .boolean(
            BoolOp::Fuse,
            std::slice::from_ref(&a),
            std::slice::from_ref(&b),
            &BoolOpts::default(),
            &ctx(),
        )
        .unwrap();
    check_golden("cylinder_cross_fuse", &cross.shape);
    let s1 = k.make_sphere(&Plane::XY, 5.0).unwrap().shape;
    let s2 = k
        .make_sphere(&frame([6.0, 0.0, 0.0], None, None), 5.0)
        .unwrap()
        .shape;
    let split = k
        .split(&[s1, s2], &[], &BoolOpts::default(), &ctx())
        .unwrap();
    check_golden("spheres_split", &split.shape);
}
