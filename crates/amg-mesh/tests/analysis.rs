//! MESH-02: `analyze` against the corpus manifest's defect flags, the synthetic generator's
//! ground truth, the independent `quick_stats` counts, and thread-count independence.

#![allow(clippy::expect_used)] // test helpers panic on bad fixtures

use amg_jobs::JobCtx;
use amg_mesh::corpus::bench::{BenchSize, benchmark_sphere};
use amg_mesh::corpus::synth::{DefectSpec, Shape, SynthSpec, generate};
use amg_mesh::corpus::{Defect, FileStatus, Manifest, corpus_root, quick_stats, read_mesh};
use amg_mesh::{AnalysisReport, AnalyzeOpts, FaceId, IndexedMesh, WeldOpts, analyze};

fn run(m: &IndexedMesh) -> AnalysisReport {
    analyze(m, &AnalyzeOpts::default(), &JobCtx::detached()).expect("analyze")
}

fn require_testdata() -> bool {
    std::env::var("AMG_REQUIRE_TESTDATA").is_ok_and(|v| v == "1")
}

/// Cross-checks every count against `quick_stats` (independent implementation).
fn cross_check(id: &str, m: &IndexedMesh, r: &AnalysisReport, bad: &mut Vec<String>) {
    let q = quick_stats(m);
    let mut chk = |name: &str, got: usize, want: usize| {
        if got != want {
            bad.push(format!("{id}: {name}: analysis {got}, quick_stats {want}"));
        }
    };
    chk("boundary edges", r.boundary_edges, q.boundary_edges);
    chk("nm edges", r.non_manifold_edges.len(), q.non_manifold_edges);
    chk("conflicts", r.orientation_conflicts, q.inconsistent_edges);
    chk("degenerate", r.degenerate_faces.len(), q.degenerate_faces);
    chk("duplicate", r.duplicate_faces.len(), q.duplicate_faces);
    chk("shells", r.shells.len(), q.components);
    chk(
        "nm vertices",
        r.non_manifold_vertices.len(),
        q.non_manifold_vertices,
    );
    if let (Some(mut lens), 0) = (q.boundary_loops, r.unclosed_boundary_loops) {
        let mut got: Vec<usize> = r.boundary_loops.iter().map(Vec::len).collect();
        got.sort_unstable();
        lens.sort_unstable();
        if got != lens {
            bad.push(format!("{id}: loop lengths differ"));
        }
    }
    let total_vol: f64 = r.shells.iter().map(|s| s.signed_volume).sum();
    let q_vol = m.signed_volume();
    let tol = 1e-9 * (1.0 + q_vol.abs() + r.aabb.diagonal().powi(3));
    if (total_vol - q_vol).abs() > tol {
        bad.push(format!(
            "{id}: volume {total_vol} vs {q_vol} (quick_stats {})",
            q.signed_volume
        ));
    }
    let area = m.area();
    if (r.area - area).abs() > 1e-9 * (1.0 + area) {
        bad.push(format!("{id}: area {} vs {area}", r.area));
    }
}

#[test]
fn analysis_matches_every_corpus_manifest_entry() {
    let man = Manifest::load().expect("corpus.toml");
    let root = corpus_root();
    let (mut checked, mut bad) = (0usize, Vec::new());
    for e in &man.meshes {
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
            FileStatus::NotFetched(_) => continue,
            FileStatus::Missing(p) => {
                bad.push(format!("{}: {} missing", e.id, p.display()));
                continue;
            }
        };
        let m = read_mesh(&path).expect("read");
        let r = run(&m);
        let has = |d: Defect| e.defects.contains(&d);
        let mut eq = |name: &str, flag: bool, got: bool| {
            if flag != got {
                bad.push(format!("{}: {name}: manifest {flag}, analysis {got}", e.id));
            }
        };
        eq("holes", has(Defect::Holes), !r.boundary_loops.is_empty());
        eq(
            "non_manifold_edges",
            has(Defect::NonManifoldEdges),
            !r.non_manifold_edges.is_empty(),
        );
        eq(
            "non_manifold_vertices",
            has(Defect::NonManifoldVertices),
            !r.non_manifold_vertices.is_empty(),
        );
        eq(
            "inconsistent_orientation",
            has(Defect::InconsistentOrientation),
            !r.is_oriented,
        );
        eq(
            "degenerate",
            has(Defect::Degenerate),
            !r.degenerate_faces.is_empty(),
        );
        eq(
            "duplicate",
            has(Defect::Duplicate),
            !r.duplicate_faces.is_empty(),
        );
        eq(
            "multiple_shells",
            has(Defect::MultipleShells),
            r.shells.len() > 1,
        );
        eq("inverted", has(Defect::Inverted), r.is_inverted);
        eq(
            "unwelded",
            has(Defect::Unwelded),
            r.unwelded_boundary_edges > 0,
        );
        // self_intersections: MESH-19. hopeless: a repair verdict, not a measurement.
        cross_check(&e.id, &m, &r, &mut bad);
        assert_eq!(r.shell_of_face.len(), m.face_count());
        checked += 1;
    }
    eprintln!("analysis: {checked} corpus entries checked");
    assert!(
        bad.is_empty(),
        "{} mismatches:\n{}",
        bad.len(),
        bad.join("\n")
    );
    if require_testdata() {
        assert!(checked >= 150, "only {checked} entries checked");
    }
}

fn sphere() -> Shape {
    Shape::Icosphere {
        frequency: 24,
        radius: 10.0,
    }
}

fn synth(shape: Shape, defects: Vec<DefectSpec>, seed: u64) -> amg_mesh::corpus::synth::Synth {
    generate(&SynthSpec {
        shape,
        defects,
        seed,
    })
}

#[test]
fn clean_shapes_are_watertight() {
    for shape in [
        sphere(),
        Shape::Torus {
            major_segments: 40,
            minor_segments: 20,
            major: 10.0,
            minor: 3.0,
        },
        Shape::BoxWithHoles {
            cells: 16,
            holes_per_side: 4,
            size: 40.0,
        },
        Shape::TorusKnot {
            segments: 240,
            sides: 12,
        },
    ] {
        let s = synth(shape.clone(), vec![], 1);
        let r = run(&s.mesh);
        assert!(r.is_watertight(), "{shape:?}");
        assert!(!r.is_inverted);
        assert_eq!(r.shells.len(), 1);
        assert!(r.shells[0].closed);
        assert!(r.shell_of_face.iter().all(|&x| x == 0));
        assert!(r.volume.expect("volume") > 0.0);
        assert_eq!(r.unreferenced_vertices, 0);
        assert!((r.area - s.mesh.area()).abs() < 1e-9 * r.area);
        assert!((r.volume.unwrap_or(0.0) - s.mesh.signed_volume()).abs() < 1e-9 * r.area);
    }
}

#[test]
fn single_defects_match_ground_truth() {
    let one = |d: DefectSpec, seed: u64| synth(sphere(), vec![d], seed);

    let s = one(
        DefectSpec::Holes {
            count: 6,
            max_boundary: 80,
        },
        7,
    );
    let r = run(&s.mesh);
    let mut lens: Vec<usize> = r.boundary_loops.iter().map(Vec::len).collect();
    lens.sort_unstable();
    assert_eq!(lens, s.truth.hole_loops);
    assert!(!r.is_closed && r.volume.is_none());
    assert!(!r.shells[0].closed);
    // Every loop is closed: the last vertex has an edge back to the first.
    for lp in &r.boundary_loops {
        assert!(lp.len() >= 3);
    }

    let s = one(DefectSpec::FlippedFaces { count: 20 }, 3);
    let r = run(&s.mesh);
    assert_eq!(r.orientation_conflicts, 60);
    assert!(!r.is_oriented && r.is_closed && r.is_manifold);

    let s = one(DefectSpec::DuplicateFaces { count: 15 }, 3);
    let r = run(&s.mesh);
    assert_eq!(r.duplicate_faces.len(), 15);
    assert_eq!(r.non_manifold_edges.len(), s.truth.non_manifold_edges);
    // The reported face is the later copy.
    for &(orig, dup) in &s.truth.duplicate_faces {
        let want = FaceId(orig.max(dup));
        assert!(r.duplicate_faces.contains(&want));
    }

    let s = one(DefectSpec::ZeroAreaSlivers { count: 10 }, 3);
    let r = run(&s.mesh);
    let got: Vec<u32> = r.degenerate_faces.iter().map(|f| f.0).collect();
    let mut want = s.truth.zero_area_faces.clone();
    want.sort_unstable();
    assert_eq!(got, want);
    assert!(r.is_watertight(), "zero-area slivers keep the topology");

    let s = one(DefectSpec::NeedleSlivers { count: 10 }, 3);
    assert!(
        run(&s.mesh).degenerate_faces.is_empty(),
        "needles have area"
    );

    let s = one(DefectSpec::NonManifoldFins { count: 8 }, 3);
    let r = run(&s.mesh);
    assert_eq!(r.non_manifold_edges.len(), 8);
    assert!(r.non_manifold_vertices.is_empty());
    assert!(!r.is_manifold && !r.is_closed);

    let s = one(DefectSpec::BowTies { count: 4 }, 3);
    let r = run(&s.mesh);
    assert_eq!(r.non_manifold_vertices.len(), 4);
    assert!(r.non_manifold_edges.is_empty() && r.boundary_loops.is_empty());
    assert_eq!(r.shells.len(), 1);

    let s = one(DefectSpec::FloatingShells { count: 3 }, 5);
    let r = run(&s.mesh);
    assert_eq!(r.shells.len(), s.truth.shells);
    assert!(r.shells.iter().all(|sh| sh.closed));
    assert_eq!(
        r.shells.iter().map(|sh| sh.faces).sum::<usize>(),
        s.mesh.face_count()
    );

    let s = one(DefectSpec::OverlappingShell, 5);
    let r = run(&s.mesh);
    assert_eq!(r.shells.len(), 2);
    assert!(r.is_watertight(), "SI is MESH-19's job");
    assert!(r.self_intersecting_pairs.is_none());

    let s = one(DefectSpec::InvertedShell, 5);
    let r = run(&s.mesh);
    assert_eq!(r.shells.len(), 2);
    assert!(r.shells.iter().any(|sh| sh.signed_volume < 0.0));
    assert!(!r.is_inverted, "total volume is still positive");
}

#[test]
fn unwelded_seam_is_found_and_disappears_after_welding() {
    let s = synth(sphere(), vec![DefectSpec::UnweldedSeam { jitter: 1e-4 }], 9);
    let r = run(&s.mesh);
    assert!(!r.boundary_loops.is_empty());
    assert!(r.unwelded_boundary_edges > 0);
    assert_eq!(r.unwelded_boundary_edges, r.boundary_edges);
    let mut m = s.mesh.clone();
    m.weld(WeldOpts { tolerance: 1e-3 }).expect("weld");
    m.compact();
    let r = run(&m);
    assert!(r.is_watertight());
    assert_eq!(r.unwelded_boundary_edges, 0);
    assert_eq!(r.vertices, s.truth.base_vertices);
}

#[test]
fn combined_defects_match_ground_truth() {
    let defects = vec![
        DefectSpec::Holes {
            count: 4,
            max_boundary: 40,
        },
        DefectSpec::FlippedFaces { count: 12 },
        DefectSpec::DuplicateFaces { count: 6 },
        DefectSpec::ZeroAreaSlivers { count: 5 },
        DefectSpec::NonManifoldFins { count: 4 },
        DefectSpec::BowTies { count: 3 },
        DefectSpec::FloatingShells { count: 2 },
    ];
    let s = synth(
        Shape::DisplacedSphere {
            frequency: 40,
            radius: 10.0,
            amplitude: 0.05,
        },
        defects,
        11,
    );
    let r = run(&s.mesh);
    let t = &s.truth;
    assert_eq!(r.non_manifold_edges.len(), t.non_manifold_edges);
    assert_eq!(r.non_manifold_vertices.len(), t.non_manifold_vertices);
    assert_eq!(r.shells.len(), t.shells);
    assert_eq!(r.duplicate_faces.len(), t.duplicate_faces.len() + t.fins);
    assert_eq!(r.degenerate_faces.len(), t.zero_area_faces.len());
    assert!(r.boundary_loops.len() >= t.hole_loops.len());
    assert!(r.orientation_conflicts >= 3 * t.flipped_faces.len());
    let mut bad = Vec::new();
    cross_check("combined", &s.mesh, &r, &mut bad);
    assert!(bad.is_empty(), "{bad:?}");
}

#[test]
fn result_is_identical_across_thread_counts() {
    let s = synth(
        Shape::DisplacedSphere {
            frequency: 60,
            radius: 10.0,
            amplitude: 0.05,
        },
        vec![
            DefectSpec::Holes {
                count: 5,
                max_boundary: 60,
            },
            DefectSpec::DuplicateFaces { count: 30 },
            DefectSpec::NonManifoldFins { count: 9 },
            DefectSpec::BowTies { count: 5 },
            DefectSpec::FloatingShells { count: 4 },
            DefectSpec::FlippedFaces { count: 40 },
        ],
        4,
    );
    let base = run(&s.mesh);
    for threads in [1, 3, 8] {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .expect("pool");
        assert_eq!(pool.install(|| run(&s.mesh)), base, "{threads} threads");
    }
}

#[test]
fn benchmark_sphere_is_clean_and_cancel_works() {
    let m = benchmark_sphere(BenchSize::K100, 1);
    let r = run(&m);
    assert!(r.is_watertight());
    assert_eq!(r.faces, 100_820);
    let tok = amg_jobs::CancelToken::new();
    tok.cancel();
    let err = analyze(&m, &AnalyzeOpts::default(), &JobCtx::with_token(tok));
    assert!(matches!(err, Err(amg_mesh::MeshError::Cancelled)));
}

/// Boundary edge count after snapping vertices to a grid of cell `cell`.
fn snapped_boundary(m: &IndexedMesh, cell: f64) -> usize {
    use std::collections::HashMap;
    let key = |v: u32| {
        let p = m.positions[v as usize] / cell;
        [p.x.round() as i64, p.y.round() as i64, p.z.round() as i64]
    };
    let mut ids: HashMap<[i64; 3], u32> = HashMap::new();
    let mut edges: HashMap<(u32, u32), u32> = HashMap::new();
    for t in &m.tris {
        let n = t.map(|v| {
            let l = ids.len() as u32;
            *ids.entry(key(v)).or_insert(l)
        });
        if n[0] == n[1] || n[1] == n[2] || n[0] == n[2] {
            continue;
        }
        for k in 0..3 {
            let (a, b) = (n[k], n[(k + 1) % 3]);
            *edges.entry((a.min(b), a.max(b))).or_default() += 1;
        }
    }
    edges.values().filter(|&&c| c == 1).count()
}

#[test]
#[ignore = "diagnostic dump for manifest disagreements (writes AMG_DUMP)"]
fn dump_manifest_disagreements() {
    use std::fmt::Write as _;
    let man = Manifest::load().expect("corpus.toml");
    let root = corpus_root();
    let mut out = String::new();
    for e in &man.meshes {
        let FileStatus::Ready(path) = e.locate(&root) else {
            continue;
        };
        let m = read_mesh(&path).expect("read");
        let r = run(&m);
        let has = |d: Defect| e.defects.contains(&d);
        let unw = r.unwelded_boundary_edges > 0;
        let nmv = !r.non_manifold_vertices.is_empty();
        let corrected = matches!(e.id.as_str(), "thingi10k-41928" | "thingi10k-180689");
        if !corrected && unw == has(Defect::Unwelded) && nmv == has(Defect::NonManifoldVertices) {
            continue;
        }
        let diag = r.aabb.diagonal();
        if corrected {
            // Every boundary edge (single live face) with every other edge, of any face count,
            // whose endpoints lie within 1e-3 x diagonal of its endpoints.
            use std::collections::HashMap;
            let mut uses: HashMap<(u32, u32), u32> = HashMap::new();
            for t in m
                .tris
                .iter()
                .filter(|t| t[0] != t[1] && t[1] != t[2] && t[0] != t[2])
            {
                for k in 0..3 {
                    let (a, b) = (t[k], t[(k + 1) % 3]);
                    *uses.entry((a.min(b), a.max(b))).or_default() += 1;
                }
            }
            let tol = 1e-4 * diag;
            let d = |a: u32, b: u32| (m.positions[a as usize] - m.positions[b as usize]).length();
            for (&(u, v), _) in uses.iter().filter(|&(_, &c)| c == 1) {
                let _ = writeln!(
                    out,
                    "{} boundary edge {u}-{v} len {:.3e} (1e-4*diag = {:.3e})",
                    e.id,
                    d(u, v),
                    1e-4 * diag
                );
                for (&(a, b), &c) in &uses {
                    if (a, b) == (u, v) {
                        continue;
                    }
                    let near =
                        (d(a, u) <= tol && d(b, v) <= tol) || (d(a, v) <= tol && d(b, u) <= tol);
                    if near {
                        let gap = (d(a, u).max(d(b, v))).min(d(a, v).max(d(b, u)));
                        let _ = writeln!(
                            out,
                            "    near edge {a}-{b} faces {c} max endpoint gap {gap:.3e}"
                        );
                    }
                }
            }
        }
        let mut sweep = String::new();
        for tol in [1e-6, 1e-5, 3e-5, 1e-4, 3e-4, 1e-3, 1e-2] {
            let o = AnalyzeOpts {
                unwelded_tolerance: tol,
                ..AnalyzeOpts::default()
            };
            let rr = analyze(&m, &o, &JobCtx::detached()).expect("analyze");
            let _ = write!(sweep, " {tol:e}:{}", rr.unwelded_boundary_edges);
        }
        let _ = writeln!(
            out,
            "{} faces={} manifest[nmv={} unw={}] mine[nmv={} ({}) unw={}] nme={} bnd={} dup={} degen={} diag={diag:.4}\n   sweep{sweep}\n   snap(1e-4*diag): before {} after {}",
            e.id,
            m.face_count(),
            has(Defect::NonManifoldVertices),
            has(Defect::Unwelded),
            nmv,
            r.non_manifold_vertices.len(),
            unw,
            r.non_manifold_edges.len(),
            r.boundary_edges,
            r.duplicate_faces.len(),
            r.degenerate_faces.len(),
            snapped_boundary(&m, 1e-9 * diag),
            snapped_boundary(&m, 1e-4 * diag),
        );
    }
    let path = std::env::var("AMG_DUMP").expect("AMG_DUMP");
    std::fs::write(path, out).expect("write");
}

#[test]
fn bad_input_is_an_error_not_a_panic() {
    let mut m = IndexedMesh::new(vec![amg_core::Point3::ZERO; 3], vec![[0, 1, 2]]).expect("mesh");
    m.tris[0] = [0, 1, 9];
    assert!(matches!(
        analyze(&m, &AnalyzeOpts::default(), &JobCtx::detached()),
        Err(amg_mesh::MeshError::IndexOutOfRange { .. })
    ));
    let m = IndexedMesh::new(vec![], vec![]).expect("empty");
    let r = run(&m);
    assert!(r.shells.is_empty() && !r.is_closed);
    let opts = AnalyzeOpts {
        self_intersections: true,
        ..AnalyzeOpts::default()
    };
    assert!(analyze(&m, &opts, &JobCtx::detached()).is_err());
}
