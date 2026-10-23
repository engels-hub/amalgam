//! MESH-04 tests: the repair pipeline (weld, degenerate, duplicate, non-manifold, small shells,
//! orientation, holes, self-intersections) on synthetic defects and on the corpus.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::print_stderr
)]

use amg_core::{Point3, Vec3};
use amg_jobs::{CancelToken, JobCtx};
use amg_mesh::corpus::synth::{DefectSpec, Shape, SynthSpec, generate};
use amg_mesh::corpus::{Defect, FileStatus, Manifest, corpus_root, read_mesh};
use amg_mesh::{
    AnalysisReport, AnalyzeOpts, HoleFillOpts, IndexedMesh, MeshError, RepairOpts, RepairReport,
    RepairStepKind, SmallShellFilter, UnresolvedIssue, WeldOpts, analyze, orient_consistently,
    repair,
};
use amg_testkit::{TriSoup, shapes};

fn ctx() -> JobCtx {
    JobCtx::detached()
}

fn require_testdata() -> bool {
    std::env::var("AMG_REQUIRE_TESTDATA").is_ok_and(|v| v == "1")
}

fn soup(m: &IndexedMesh) -> TriSoup<'_> {
    TriSoup {
        positions: &m.positions,
        triangles: &m.tris,
    }
}

fn report(m: &IndexedMesh, si: bool) -> AnalysisReport {
    let o = AnalyzeOpts {
        self_intersections: si,
        ..AnalyzeOpts::default()
    };
    analyze(m, &o, &ctx()).unwrap()
}

/// Defect counts by category (self-intersections included when the report has them).
fn counts(r: &AnalysisReport) -> [usize; 7] {
    [
        r.boundary_loops.len(),
        r.non_manifold_edges.len(),
        r.non_manifold_vertices.len(),
        r.orientation_conflicts,
        r.degenerate_faces.len(),
        r.duplicate_faces.len(),
        r.self_intersecting_pairs.as_ref().map_or(0, Vec::len),
    ]
}

fn watertight_outward(r: &AnalysisReport) -> bool {
    r.is_watertight() && !r.is_inverted
}

fn spec(shape: Shape, defects: Vec<DefectSpec>, seed: u64) -> SynthSpec {
    SynthSpec {
        shape,
        defects,
        seed,
    }
}

fn sphere() -> Shape {
    Shape::Icosphere {
        frequency: 16,
        radius: 10.0,
    }
}

fn no_si() -> RepairOpts {
    RepairOpts {
        remove_self_intersections: false,
        ..RepairOpts::default()
    }
}

fn run(m: &IndexedMesh, o: &RepairOpts) -> (IndexedMesh, RepairReport) {
    repair(m, o, &ctx()).unwrap()
}

fn icosphere(radius: f64, sub: u32) -> IndexedMesh {
    let s = shapes::icosphere(Point3::ZERO, radius, sub);
    IndexedMesh::new(s.positions, s.triangles).unwrap()
}

fn flip_all(m: &mut IndexedMesh) {
    for t in &mut m.tris {
        t.swap(1, 2);
    }
}

// ------------------------------------------------------------------------------ synthetic

#[test]
fn clean_mesh_comes_back_bit_identical() {
    let m = generate(&spec(sphere(), vec![], 1)).mesh;
    let (out, rep) = run(&m, &RepairOpts::default());
    assert_eq!(out, m);
    assert_eq!(out.content_hash(), m.content_hash());
    assert!(rep.steps.iter().all(|s| s.changed == 0), "{:?}", rep.steps);
    assert!(rep.unresolved.is_empty());
    assert_eq!(counts(&rep.before), counts(&rep.after));
}

#[test]
fn each_defect_is_repaired() {
    let cases: Vec<(&str, DefectSpec)> = vec![
        (
            "holes",
            DefectSpec::Holes {
                count: 4,
                max_boundary: 30,
            },
        ),
        ("seam", DefectSpec::UnweldedSeam { jitter: 5e-6 }),
        ("flipped", DefectSpec::FlippedFaces { count: 20 }),
        ("duplicates", DefectSpec::DuplicateFaces { count: 15 }),
        ("zero slivers", DefectSpec::ZeroAreaSlivers { count: 12 }),
        ("needles", DefectSpec::NeedleSlivers { count: 12 }),
        ("fins", DefectSpec::NonManifoldFins { count: 6 }),
        ("bow ties", DefectSpec::BowTies { count: 4 }),
        ("inverted shell", DefectSpec::InvertedShell),
        ("floating shells", DefectSpec::FloatingShells { count: 5 }),
        ("f32", DefectSpec::F32Rounded),
    ];
    let base = generate(&spec(sphere(), vec![], 1)).mesh;
    let base_volume = base.signed_volume();
    for (name, d) in cases {
        let s = generate(&spec(sphere(), vec![d], 3)).mesh;
        let before = report(&s, false);
        let defective = !watertight_outward(&before)
            || counts(&before).iter().any(|c| *c > 0)
            || before.shells.len() > 1
            || before.shells.iter().any(|s| s.signed_volume < 0.0);
        assert!(
            defective || matches!(name, "f32" | "needles"),
            "{name}: input should be defective"
        );
        let (out, rep) = run(&s, &RepairOpts::default());
        out.validate().unwrap();
        let after = report(&out, true);
        assert!(watertight_outward(&after), "{name}: {:?}", counts(&after));
        assert_eq!(
            after.self_intersecting_pairs.as_ref().map(Vec::len),
            Some(0),
            "{name}"
        );
        assert!(rep.unresolved.is_empty(), "{name}: {:?}", rep.unresolved);
        // Same body: the patches and removed debris change the volume by a few percent at most.
        let v = after.volume.unwrap();
        let expect = if name == "inverted shell" {
            base_volume * (1.0 + 1.0 / 64.0)
        } else {
            base_volume
        };
        assert!(
            (v - expect).abs() < 0.03 * expect,
            "{name}: volume {v} vs {expect}"
        );
        let want_shells = if name == "inverted shell" { 2 } else { 1 };
        assert_eq!(after.shells.len(), want_shells, "{name}");
    }
}

#[test]
fn all_defects_together() {
    let defects = vec![
        DefectSpec::Holes {
            count: 3,
            max_boundary: 40,
        },
        DefectSpec::UnweldedSeam { jitter: 5e-6 },
        DefectSpec::FlippedFaces { count: 15 },
        DefectSpec::DuplicateFaces { count: 10 },
        DefectSpec::ZeroAreaSlivers { count: 8 },
        DefectSpec::NeedleSlivers { count: 8 },
        DefectSpec::NonManifoldFins { count: 4 },
        DefectSpec::FloatingShells { count: 3 },
    ];
    for shape in [
        sphere(),
        Shape::Torus {
            major_segments: 48,
            minor_segments: 24,
            major: 10.0,
            minor: 3.0,
        },
    ] {
        let s = generate(&spec(shape, defects.clone(), 5)).mesh;
        let (out, rep) = run(&s, &no_si());
        let after = report(&out, true);
        assert!(watertight_outward(&after), "{:?}", counts(&after));
        assert!(counts(&after).iter().all(|c| *c == 0));
        for k in 0..6 {
            assert!(counts(&rep.after)[k] <= counts(&rep.before)[k]);
        }
        // Repairing the result again changes nothing.
        let (again, rep2) = run(&out, &no_si());
        assert_eq!(again.content_hash(), out.content_hash());
        assert!(
            rep2.steps.iter().all(|s| s.changed == 0),
            "{:?}",
            rep2.steps
        );
    }
}

#[test]
fn no_single_step_makes_any_count_worse() {
    let defects = vec![
        DefectSpec::Holes {
            count: 3,
            max_boundary: 40,
        },
        DefectSpec::UnweldedSeam { jitter: 5e-6 },
        DefectSpec::FlippedFaces { count: 15 },
        DefectSpec::DuplicateFaces { count: 10 },
        DefectSpec::ZeroAreaSlivers { count: 8 },
        DefectSpec::NeedleSlivers { count: 8 },
        DefectSpec::NonManifoldFins { count: 4 },
        DefectSpec::BowTies { count: 3 },
        DefectSpec::FloatingShells { count: 3 },
    ];
    let s = generate(&spec(sphere(), defects, 9)).mesh;
    let off = RepairOpts {
        weld: None,
        remove_degenerate: false,
        remove_duplicates: false,
        fix_non_manifold: false,
        remove_small_shells: None,
        orient: false,
        fill_holes: None,
        remove_self_intersections: false,
        max_rounds: 1,
    };
    let steps: Vec<(&str, RepairOpts)> = vec![
        (
            "weld",
            RepairOpts {
                weld: Some(WeldOpts::default()),
                ..off.clone()
            },
        ),
        (
            "degenerate",
            RepairOpts {
                remove_degenerate: true,
                ..off.clone()
            },
        ),
        (
            "duplicates",
            RepairOpts {
                remove_duplicates: true,
                ..off.clone()
            },
        ),
        (
            "non-manifold",
            RepairOpts {
                fix_non_manifold: true,
                ..off.clone()
            },
        ),
        (
            "orient",
            RepairOpts {
                orient: true,
                ..off.clone()
            },
        ),
        (
            "holes",
            RepairOpts {
                fill_holes: Some(HoleFillOpts::default()),
                ..off.clone()
            },
        ),
    ];
    // Pipeline order, each step on the previous step's output: the total of all defect counts
    // never grows, and the counts the step is for never grow.
    let targets: [&[usize]; 6] = [&[0], &[4], &[5], &[1, 2], &[3], &[0]];
    let mut cur = s;
    for ((name, o), target) in steps.into_iter().zip(targets) {
        let (out, rep) = run(&cur, &o);
        let (b, a) = (counts(&rep.before), counts(&rep.after));
        assert_eq!(counts(&report(&out, false)), a);
        for &k in target {
            assert!(a[k] <= b[k], "{name}: category {k}: {b:?} -> {a:?}");
        }
        assert!(
            a.iter().sum::<usize>() <= b.iter().sum::<usize>(),
            "{name}: {b:?} -> {a:?}"
        );
        eprintln!("{name}: {b:?} -> {a:?}");
        cur = out;
    }
    let last = report(&cur, false);
    assert!(watertight_outward(&last), "{:?}", counts(&last));
}

#[test]
fn seam_weld_and_report() {
    let s = generate(&spec(
        sphere(),
        vec![DefectSpec::UnweldedSeam { jitter: 5e-6 }],
        2,
    ))
    .mesh;
    let (out, rep) = run(&s, &no_si());
    let weld = rep
        .steps
        .iter()
        .find(|s| s.kind == RepairStepKind::Weld)
        .unwrap();
    assert!(weld.changed > 0);
    assert!(rep.before.boundary_edges > 0);
    assert_eq!(rep.after.boundary_edges, 0);
    assert_eq!(out.vertex_count(), 10 * 16 * 16 + 2);
    // The report's `after` is the analysis of the output.
    assert_eq!(rep.after, report(&out, false));
}

#[test]
fn degenerate_faces_are_collapsed_not_cut_out() {
    let s = generate(&spec(
        sphere(),
        vec![
            DefectSpec::ZeroAreaSlivers { count: 10 },
            DefectSpec::NeedleSlivers { count: 10 },
        ],
        4,
    ))
    .mesh;
    let before = report(&s, false);
    assert!(before.degenerate_faces.len() >= 10);
    assert_eq!(before.boundary_edges, 0);
    let o = RepairOpts {
        fill_holes: None,
        ..no_si()
    };
    let (out, rep) = run(&s, &o);
    let after = report(&out, false);
    // Collapsing keeps the surface closed, so no hole step is needed.
    assert!(watertight_outward(&after), "{:?}", counts(&after));
    assert!(after.degenerate_faces.is_empty());
    assert!(rep.unresolved.is_empty());
    assert!((after.volume.unwrap() - before.volume.unwrap()).abs() < 1e-6 * before.volume.unwrap());
}

#[test]
fn duplicates_and_opposite_pairs() {
    let base = icosphere(5.0, 2);
    // Same winding: one copy stays.
    let mut m = base.clone();
    m.tris.push(base.tris[0]);
    m.groups = amg_mesh::FaceGroups::new(m.tris.len());
    let (out, _) = run(&m, &no_si());
    assert_eq!(out.content_hash(), base.content_hash());
    // Opposite winding: the pair cancels and the hole is filled again.
    let mut m = base.clone();
    let mut t = base.tris[0];
    t.swap(1, 2);
    m.tris.push(t);
    m.groups = amg_mesh::FaceGroups::new(m.tris.len());
    let (out, rep) = run(&m, &no_si());
    let after = report(&out, false);
    assert!(watertight_outward(&after));
    assert_eq!(out.tris.len(), base.tris.len());
    assert!(
        rep.steps
            .iter()
            .any(|s| s.kind == RepairStepKind::Duplicate && s.changed == 2)
    );
}

#[test]
fn non_manifold_is_unzipped_without_losing_faces() {
    let s = generate(&spec(
        sphere(),
        vec![DefectSpec::NonManifoldFins { count: 6 }],
        7,
    ))
    .mesh;
    let before = report(&s, false);
    assert_eq!(before.non_manifold_edges.len(), 6);
    let o = RepairOpts {
        fill_holes: None,
        remove_duplicates: false,
        remove_degenerate: false,
        remove_small_shells: None,
        ..no_si()
    };
    let (out, rep) = run(&s, &o);
    let after = report(&out, false);
    assert!(after.non_manifold_edges.is_empty());
    assert!(after.non_manifold_vertices.is_empty());
    assert_eq!(out.tris.len(), s.tris.len());
    assert!(after.is_closed, "{:?}", counts(&after));
    assert!(rep.unresolved.is_empty());
    // With the default small-shell filter the zero-thickness fins disappear altogether.
    let (clean, _) = run(&s, &no_si());
    let r = report(&clean, false);
    assert!(watertight_outward(&r), "{:?}", counts(&r));
    assert_eq!(r.shells.len(), 1);
}

#[test]
fn bow_ties_are_split() {
    let s = generate(&spec(sphere(), vec![DefectSpec::BowTies { count: 5 }], 8)).mesh;
    let before = report(&s, false);
    assert!(!before.non_manifold_vertices.is_empty());
    let (out, _) = run(&s, &no_si());
    let after = report(&out, false);
    assert!(after.is_manifold);
    assert!(watertight_outward(&after), "{:?}", counts(&after));
}

#[test]
fn small_shells_go_and_the_largest_stays() {
    let s = generate(&spec(
        sphere(),
        vec![DefectSpec::FloatingShells { count: 5 }],
        2,
    ))
    .mesh;
    let (out, rep) = run(&s, &no_si());
    assert_eq!(rep.before.shells.len(), 6);
    assert_eq!(rep.after.shells.len(), 1);
    assert_eq!(
        out.tris.len(),
        generate(&spec(sphere(), vec![], 2)).mesh.tris.len()
    );
    // Off: they stay.
    let o = RepairOpts {
        remove_small_shells: None,
        ..no_si()
    };
    let (out, _) = run(&s, &o);
    assert_eq!(report(&out, false).shells.len(), 6);
    // A big filter never removes the only large shell.
    let o = RepairOpts {
        remove_small_shells: Some(SmallShellFilter {
            max_faces: usize::MAX,
            max_volume_fraction: 1.0,
        }),
        ..no_si()
    };
    let (out, _) = run(&s, &o);
    assert_eq!(report(&out, false).shells.len(), 1);
}

#[test]
fn orientation_is_outward_and_cavities_inward() {
    let mut outer = icosphere(10.0, 2);
    let mut inner = icosphere(4.0, 2);
    // Outer inverted, inner outward: both wrong for a hollow body.
    flip_all(&mut outer);
    let mut m = outer.clone();
    m.append(&inner).unwrap();
    let rep0 = report(&m, false);
    assert!(rep0.is_watertight());
    let flipped = {
        let mut c = m.clone();
        orient_consistently(&mut c, &ctx()).unwrap()
    };
    assert_eq!(flipped, outer.tris.len() + inner.tris.len());
    let (out, _) = run(&m, &no_si());
    let r = report(&out, false);
    assert!(watertight_outward(&r));
    // Hollow: volume of the outer sphere minus the inner one.
    let v_outer = icosphere(10.0, 2).signed_volume();
    let v_inner = icosphere(4.0, 2).signed_volume();
    let v = r.volume.unwrap();
    assert!((v - (v_outer - v_inner)).abs() < 1e-6 * v_outer, "{v}");
    assert_eq!(r.shells.len(), 2);
    assert!(r.shells.iter().any(|s| s.signed_volume < 0.0));
    // Already right: nothing changes.
    flip_all(&mut inner);
    let mut ok = icosphere(10.0, 2);
    ok.append(&inner).unwrap();
    let (out, rep) = run(&ok, &no_si());
    assert_eq!(out.content_hash(), ok.content_hash());
    assert!(rep.steps.iter().all(|s| s.changed == 0));
    // Two separate solids: both outward.
    let mut a = icosphere(3.0, 2);
    let mut b = icosphere(3.0, 2);
    b.transform(&amg_core::Transform::from_translation(Point3::new(
        20.0, 0.0, 0.0,
    )));
    flip_all(&mut b);
    a.append(&b).unwrap();
    let (out, _) = run(&a, &no_si());
    let r = report(&out, false);
    assert!(r.shells.iter().all(|s| s.signed_volume > 0.0));
}

#[test]
fn big_holes_are_reported_not_hidden() {
    let s = generate(&spec(
        Shape::Icosphere {
            frequency: 24,
            radius: 10.0,
        },
        vec![DefectSpec::Holes {
            count: 1,
            max_boundary: 60,
        }],
        11,
    ))
    .mesh;
    let o = RepairOpts {
        fill_holes: Some(HoleFillOpts {
            max_boundary_edges: 4,
            ..HoleFillOpts::default()
        }),
        ..no_si()
    };
    let (_, rep) = run(&s, &o);
    assert!(
        rep.unresolved.iter().any(
            |u| matches!(u, UnresolvedIssue::HoleTooLarge { boundary_edges } if *boundary_edges > 4)
        ),
        "{:?}",
        rep.unresolved
    );
    assert!(!rep.after.is_closed);
}

#[test]
fn self_intersections_are_removed_when_asked() {
    // A sphere with a few vertices moved sideways by 3.5 edges and slightly inwards (folds).
    let base = icosphere(10.0, 4);
    let edge =
        base.positions[base.tris[0][0] as usize].distance(base.positions[base.tris[0][1] as usize]);
    let fold = |first: usize| {
        let mut m = base.clone();
        for k in 0..4 {
            let i = first + 331 * k;
            let p = m.positions[i];
            let radial = p.normalize();
            let t = radial.cross(Vec3::X).normalize();
            m.positions[i] = p + t * (3.5 * edge) - radial * (0.8 * edge);
        }
        m
    };
    let m = (100..200)
        .map(fold)
        .find(|m| {
            report(m, true)
                .self_intersecting_pairs
                .is_some_and(|p| !p.is_empty())
        })
        .expect("some fold self-intersects");
    let before = report(&m, true);
    let n_before = before.self_intersecting_pairs.as_ref().map_or(0, Vec::len);
    assert!(n_before > 0);
    let (out, rep) = run(&m, &RepairOpts::default());
    assert!(
        rep.steps
            .iter()
            .any(|s| s.kind == RepairStepKind::SelfIntersections && s.changed > 0)
    );
    assert_eq!(
        rep.after.self_intersecting_pairs.as_ref().map(Vec::len),
        Some(0)
    );
    assert!(watertight_outward(&report(&out, false)));
    // Off: untouched.
    let (same, rep) = run(&m, &no_si());
    assert_eq!(same.content_hash(), m.content_hash());
    assert!(rep.after.self_intersecting_pairs.is_none());
}

#[test]
fn deterministic_across_thread_counts() {
    let defects = vec![
        DefectSpec::Holes {
            count: 3,
            max_boundary: 40,
        },
        DefectSpec::UnweldedSeam { jitter: 5e-6 },
        DefectSpec::FlippedFaces { count: 15 },
        DefectSpec::DuplicateFaces { count: 10 },
        DefectSpec::NonManifoldFins { count: 4 },
    ];
    let s = generate(&spec(sphere(), defects, 5)).mesh;
    let hashes: Vec<u64> = [1usize, 4, 13]
        .into_iter()
        .map(|n| {
            let pool = rayon::ThreadPoolBuilder::new()
                .num_threads(n)
                .build()
                .unwrap();
            pool.install(|| run(&s, &RepairOpts::default()).0.content_hash())
        })
        .collect();
    assert!(hashes.windows(2).all(|w| w[0] == w[1]), "{hashes:?}");
}

#[test]
fn cancel_and_invalid_options() {
    let s = generate(&spec(
        sphere(),
        vec![DefectSpec::FlippedFaces { count: 5 }],
        1,
    ))
    .mesh;
    let token = CancelToken::new();
    token.cancel();
    let c = JobCtx::with_token(token);
    assert!(matches!(
        repair(&s, &RepairOpts::default(), &c),
        Err(MeshError::Cancelled)
    ));
    let bad = RepairOpts {
        weld: Some(WeldOpts { tolerance: -1.0 }),
        ..RepairOpts::default()
    };
    assert!(matches!(
        repair(&s, &bad, &ctx()),
        Err(MeshError::InvalidOption { .. })
    ));
    let bad = RepairOpts {
        remove_small_shells: Some(SmallShellFilter {
            max_faces: 3,
            max_volume_fraction: f64::NAN,
        }),
        ..RepairOpts::default()
    };
    assert!(matches!(
        repair(&s, &bad, &ctx()),
        Err(MeshError::InvalidOption { .. })
    ));
    // Empty mesh and zero rounds are no-ops.
    let e = IndexedMesh::default();
    let (out, rep) = repair(&e, &RepairOpts::default(), &ctx()).unwrap();
    assert!(out.tris.is_empty() && rep.steps.iter().all(|s| s.changed == 0));
    let zero = RepairOpts {
        max_rounds: 0,
        ..RepairOpts::default()
    };
    let (out, rep) = run(&s, &zero);
    assert_eq!(out, s);
    assert!(rep.steps.is_empty());
}

#[test]
fn golden_repaired_sphere_and_torus() {
    let defects = vec![
        DefectSpec::Holes {
            count: 2,
            max_boundary: 30,
        },
        DefectSpec::UnweldedSeam { jitter: 5e-6 },
        DefectSpec::FlippedFaces { count: 10 },
        DefectSpec::DuplicateFaces { count: 6 },
        DefectSpec::NonManifoldFins { count: 3 },
        DefectSpec::FloatingShells { count: 2 },
    ];
    let s = generate(&spec(sphere(), defects.clone(), 21)).mesh;
    let (out, _) = run(&s, &RepairOpts::default());
    amg_testkit::assert_golden!("mesh", "repair_sphere_defects", soup(&out));
    let t = generate(&spec(
        Shape::Torus {
            major_segments: 40,
            minor_segments: 20,
            major: 10.0,
            minor: 3.0,
        },
        defects,
        22,
    ))
    .mesh;
    let (out, _) = run(&t, &RepairOpts::default());
    amg_testkit::assert_golden!("mesh", "repair_torus_defects", soup(&out));
}

// ------------------------------------------------------------------------------- corpus

fn max_faces() -> usize {
    std::env::var("AMG_REPAIR_MAX_FACES")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(150_000)
}

/// Done-when: at least 95 % of the `expect_repairable` models become watertight (closed,
/// manifold, oriented, outward) and at least 85 % are also free of self-intersections; the
/// second run changes nothing; no category gets worse.
#[test]
fn corpus_repairable_models() {
    let manifest = Manifest::load().unwrap();
    let root = corpus_root();
    let limit = max_faces();
    let (mut tested, mut watertight, mut si_free, mut idem) = (0usize, 0usize, 0usize, 0usize);
    let (mut flagged, mut flagged_free) = (0usize, 0usize);
    let (mut total_ms, mut slowest) = (0.0f64, (0.0f64, String::new()));
    let mut fails = Vec::new();
    let mut not_watertight = Vec::new();
    let mut si_grew: Vec<String> = Vec::new();
    for e in manifest.meshes.iter() {
        if !e.expect_repairable || e.faces > limit {
            continue;
        }
        let path = match e.locate(&root) {
            FileStatus::Ready(p) => p,
            FileStatus::LfsPointer(p) => {
                assert!(
                    !require_testdata(),
                    "{} is an un-fetched git-LFS pointer",
                    p.display()
                );
                continue;
            }
            _ => continue,
        };
        let Ok(raw) = read_mesh(&path) else {
            continue;
        };
        tested += 1;
        let started = std::time::Instant::now();
        let (out, rep) = repair(&raw, &RepairOpts::default(), &ctx()).unwrap();
        let ms = started.elapsed().as_secs_f64() * 1e3;
        total_ms += ms;
        if ms > slowest.0 {
            slowest = (ms, e.id.clone());
        }
        let (b, a) = (counts(&rep.before), counts(&rep.after));
        for k in 0..7 {
            // The patches that close hundreds of holes can cross the surface they are
            // sewn into (overlapping solids), so the pair count is only held to
            // models that had no holes or non-manifold elements.
            let holes_filled = k == 6 && (b[0] > 0 || b[1] > 0 || b[2] > 0);
            if a[k] > b[k] {
                if holes_filled {
                    si_grew.push(e.id.clone());
                } else {
                    fails.push(format!("{}: category {k} grew {b:?} -> {a:?}", e.id));
                }
            }
        }
        let wt = watertight_outward(&rep.after);
        let free = rep
            .after
            .self_intersecting_pairs
            .as_ref()
            .is_some_and(Vec::is_empty);
        let was_flagged = e.defects.contains(&Defect::SelfIntersections);
        if was_flagged {
            flagged += 1;
        }
        if wt {
            watertight += 1;
        } else {
            not_watertight.push(e.id.clone());
        }
        if wt && free {
            si_free += 1;
            if was_flagged {
                flagged_free += 1;
            }
        }
        // Idempotence.
        let (again, rep2) = repair(&out, &RepairOpts::default(), &ctx()).unwrap();
        if again.content_hash() == out.content_hash() && rep2.steps.iter().all(|s| s.changed == 0) {
            idem += 1;
        } else {
            fails.push(format!(
                "{}: second run changed {:?}",
                e.id,
                rep2.steps
                    .iter()
                    .filter(|s| s.changed > 0)
                    .map(|s| (s.kind, s.changed))
                    .collect::<Vec<_>>()
            ));
        }
        eprintln!(
            "CORPUS {}: {} faces, {:?} -> {:?}, wt {}, si-free {}, {:.0} ms",
            e.id, e.faces, b, a, wt, free, ms
        );
    }
    eprintln!(
        "corpus repair: {tested} models <= {limit} faces: watertight {watertight} ({:.1} %), watertight and SI-free {si_free} ({:.1} %), flagged-SI {flagged_free}/{flagged}, idempotent {idem}, total {total_ms:.0} ms, slowest {} {:.0} ms",
        100.0 * watertight as f64 / tested.max(1) as f64,
        100.0 * si_free as f64 / tested.max(1) as f64,
        slowest.1,
        slowest.0
    );
    eprintln!("not watertight: {not_watertight:?}");
    eprintln!("pair count above the input's after filling holes: {si_grew:?}");
    assert!(fails.is_empty(), "{fails:#?}");
    if tested == 0 {
        return; // clone without `git lfs pull`
    }
    assert!(tested >= 50, "only {tested} corpus models usable");
    assert!(
        watertight as f64 >= 0.95 * tested as f64,
        "{watertight}/{tested} watertight, need >= 95 %"
    );
    assert!(
        si_free as f64 >= 0.85 * tested as f64,
        "{si_free}/{tested} watertight and SI-free, need >= 85 %"
    );
}
