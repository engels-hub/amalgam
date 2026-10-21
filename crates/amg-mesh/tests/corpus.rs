//! MESH-28: the broken-mesh corpus. The manifest is checked and every model iterated; the
//! synthetic defect generator is checked against an independent topology count; the benchmark
//! generator against its closed-form sizes.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

use amg_core::{Point3, Vec3};
use amg_mesh::corpus::bench::{BenchSize, benchmark_sphere, boolean_pair, geodesic_sphere};
use amg_mesh::corpus::synth::{DefectSpec, Shape, SynthSpec, generate};
use amg_mesh::corpus::{
    Defect, FileStatus, MAX_COMMITTED_FILE_BYTES, MAX_COMMITTED_TOTAL_BYTES, Manifest, QuickStats,
    corpus_root, quick_stats, read_mesh, read_ply, read_stl, sha256_hex,
};
use amg_mesh::{IndexedMesh, WeldOpts};

fn require_testdata() -> bool {
    std::env::var("AMG_REQUIRE_TESTDATA").is_ok_and(|v| v == "1")
}

fn walk(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(rd) = std::fs::read_dir(dir) else {
        return;
    };
    for e in rd.flatten() {
        let p = e.path();
        if p.is_dir() {
            walk(&p, out);
        } else {
            out.push(p);
        }
    }
}

// ------------------------------------------------------------------------------- manifest

#[test]
fn manifest_is_valid_and_complete() {
    let m = Manifest::load().expect("corpus.toml");
    assert!(m.committed().count() >= 150, "need >= 150 committed models");
    // Every committed `real/` model file is listed exactly once and nothing else is there.
    let root = corpus_root();
    let mut files = Vec::new();
    walk(&root.join("real"), &mut files);
    let on_disk: BTreeSet<String> = files
        .iter()
        .filter(|p| {
            p.extension()
                .and_then(|e| e.to_str())
                .is_some_and(|e| matches!(e, "stl" | "ply" | "obj" | "3mf"))
        })
        .map(|p| {
            p.strip_prefix(&root)
                .expect("under root")
                .to_string_lossy()
                .replace('\\', "/")
        })
        .collect();
    let listed: BTreeSet<String> = m.committed().map(|e| e.file.clone()).collect();
    assert_eq!(on_disk, listed, "real/ files and manifest entries differ");
    // Every real/ source folder carries its licence notice.
    let folders: BTreeSet<&str> = listed.iter().filter_map(|f| f.split('/').nth(1)).collect();
    for f in folders {
        let d = root.join("real").join(f);
        assert!(
            d.join("NOTICE").is_file() && d.join("README.md").is_file(),
            "real/{f} needs NOTICE and README.md"
        );
    }
    // Budget: <= 300 MB total, <= 20 MB per file.
    let mut total = 0u64;
    for e in m.committed() {
        let len = std::fs::metadata(root.join(&e.file)).expect("file").len();
        assert!(len <= MAX_COMMITTED_FILE_BYTES, "{} is {len} bytes", e.file);
        total += len;
    }
    assert!(
        total <= MAX_COMMITTED_TOTAL_BYTES,
        "{total} bytes committed"
    );
}

#[test]
fn fetch_list_matches_manifest() {
    let m = Manifest::load().expect("corpus.toml");
    let text = std::fs::read_to_string(corpus_root().join("fetch.tsv")).expect("fetch.tsv");
    let rows: Vec<Vec<&str>> = text
        .lines()
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .map(|l| l.split('\t').collect())
        .collect();
    let want: Vec<Vec<String>> = m
        .fetch_only()
        .map(|e| {
            vec![
                e.id.clone(),
                e.file.clone(),
                e.fetch_url.clone().expect("url"),
                e.fetch_sha256.clone().expect("sha"),
                e.fetch_archive.clone().expect("archive"),
                e.fetch_member.clone().unwrap_or_else(|| "-".into()),
                e.sha256.clone(),
            ]
        })
        .collect();
    let got: Vec<Vec<String>> = rows
        .iter()
        .map(|r| r.iter().map(|s| (*s).to_owned()).collect())
        .collect();
    assert_eq!(
        got, want,
        "fetch.tsv must list exactly the fetch-only entries"
    );
}

#[test]
fn manifest_rejects_bad_entries() {
    let text = std::fs::read_to_string(corpus_root().join("corpus.toml")).expect("corpus.toml");
    // A committed model with a non-redistributable licence is refused.
    let bad = text.replacen("license = \"CC-BY-3.0\"", "license = \"CC-BY-NC-3.0\"", 1);
    assert!(Manifest::parse(&bad).is_err());
    // A malformed hash is refused.
    let first = text.find("sha256 = \"").expect("sha") + 10;
    let mut bad = text.clone();
    bad.replace_range(first..first + 1, "G");
    assert!(Manifest::parse(&bad).is_err());
    // Unknown keys are refused.
    assert!(Manifest::parse(&text.replacen("faces = ", "facez = ", 1)).is_err());
}

/// Flags that exact-weld topology alone decides must match the manifest in both directions.
fn check_flags(e: &amg_mesh::corpus::Entry, s: &QuickStats) -> Vec<String> {
    let has = |d: Defect| e.defects.contains(&d);
    let mut bad = Vec::new();
    let mut eq = |name: &str, flag: bool, actual: bool| {
        if flag != actual {
            bad.push(format!("{name}: manifest {flag}, measured {actual}"));
        }
    };
    eq("holes", has(Defect::Holes), s.boundary_edges > 0);
    eq(
        "non_manifold_edges",
        has(Defect::NonManifoldEdges),
        s.non_manifold_edges > 0,
    );
    eq(
        "inconsistent_orientation",
        has(Defect::InconsistentOrientation),
        s.inconsistent_edges > 0,
    );
    eq(
        "degenerate",
        has(Defect::Degenerate),
        s.degenerate_faces > 0,
    );
    eq("duplicate", has(Defect::Duplicate), s.duplicate_faces > 0);
    eq(
        "multiple_shells",
        has(Defect::MultipleShells),
        s.components > 1,
    );
    let closed_oriented =
        s.boundary_edges == 0 && s.non_manifold_edges == 0 && s.inconsistent_edges == 0;
    eq(
        "inverted",
        has(Defect::Inverted),
        closed_oriented && s.signed_volume < 0.0,
    );
    if s.non_manifold_edges == 0 && s.faces < 40_000 {
        eq(
            "non_manifold_vertices",
            has(Defect::NonManifoldVertices),
            s.non_manifold_vertices > 0,
        );
    }
    bad
}

#[test]
fn every_corpus_model_matches_its_manifest_entry() {
    let m = Manifest::load().expect("corpus.toml");
    let root = corpus_root();
    let (mut checked, mut skipped, mut lfs_skipped) = (0usize, Vec::new(), Vec::new());
    let mut failures = Vec::new();
    eprintln!(
        "{:<22} {:>8} {:>7} {:>6} {:>5} {:>5} {:>5} {:>6}",
        "id", "faces", "bnd", "nm-e", "orient", "degen", "dup", "shells"
    );
    for e in &m.meshes {
        let path = match e.locate(&root) {
            FileStatus::Ready(p) => p,
            FileStatus::NotFetched(p) => {
                skipped.push(e.id.clone());
                eprintln!(
                    "SKIP: {} not fetched ({}); run testdata/mesh/fetch.sh",
                    e.id,
                    p.display()
                );
                continue;
            }
            FileStatus::LfsPointer(p) => {
                assert!(
                    !require_testdata(),
                    "{} is an un-fetched git-LFS pointer and AMG_REQUIRE_TESTDATA=1; run `git lfs pull`",
                    p.display()
                );
                lfs_skipped.push(e.id.clone());
                continue;
            }
            FileStatus::Missing(p) => {
                failures.push(format!(
                    "{}: committed file {} is missing",
                    e.id,
                    p.display()
                ));
                continue;
            }
        };
        let hash = sha256_hex(&path).expect("hash");
        if hash != e.sha256 {
            failures.push(format!("{}: sha256 {hash} != manifest {}", e.id, e.sha256));
            continue;
        }
        let mesh = match read_mesh(&path) {
            Ok(mesh) => mesh,
            Err(err) => {
                failures.push(format!("{}: {err}", e.id));
                continue;
            }
        };
        if mesh.face_count() != e.faces {
            failures.push(format!(
                "{}: {} faces, manifest {}",
                e.id,
                mesh.face_count(),
                e.faces
            ));
        }
        if let Err(err) = mesh.validate() {
            failures.push(format!("{}: invalid mesh: {err}", e.id));
        }
        let s = quick_stats(&mesh);
        eprintln!(
            "{:<22} {:>8} {:>7} {:>6} {:>5} {:>5} {:>5} {:>6}",
            e.id,
            s.faces,
            s.boundary_edges,
            s.non_manifold_edges,
            s.inconsistent_edges,
            s.degenerate_faces,
            s.duplicate_faces,
            s.components
        );
        for b in check_flags(e, &s) {
            failures.push(format!("{}: {b}", e.id));
        }
        checked += 1;
    }
    eprintln!(
        "corpus: {checked} checked, {} fetch-only skipped, {} LFS pointers skipped",
        skipped.len(),
        lfs_skipped.len()
    );
    assert!(
        failures.is_empty(),
        "{} failures:\n{}",
        failures.len(),
        failures.join("\n")
    );
    if !require_testdata() && checked == 0 {
        eprintln!("SKIP: no corpus file is available (git lfs pull)");
    }
}

// ------------------------------------------------------------------------------- readers

#[test]
fn readers_handle_stl_and_ply_variants() {
    // ASCII STL
    let ascii = b"solid t\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid";
    let soup = read_stl(ascii).expect("ascii stl");
    assert_eq!(soup, vec![[Point3::ZERO, Point3::X, Point3::Y]]);
    // Binary STL whose header starts with "solid"
    let mut bin = vec![0u8; 84];
    bin[..5].copy_from_slice(b"solid");
    bin[80] = 1;
    bin.extend_from_slice(&[0u8; 12]);
    for v in [[0.0f32, 0.0, 0.0], [0.5, 0.0, 0.0], [0.0, 0.25, 0.0]] {
        for c in v {
            bin.extend_from_slice(&c.to_le_bytes());
        }
    }
    bin.extend_from_slice(&[0, 0]);
    let soup = read_stl(&bin).expect("binary stl");
    assert_eq!(soup[0][1], Point3::new(0.5, 0.0, 0.0));
    assert_eq!(soup[0][2], Point3::new(0.0, 0.25, 0.0));
    // ASCII PLY with a quad (fan-triangulated) and extra properties
    let ply = b"ply\nformat ascii 1.0\nelement vertex 4\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nelement face 1\nproperty list uchar int vertex_indices\nend_header\n0 0 0 9\n1 0 0 9\n1 1 0 9\n0 1 0 9\n4 0 1 2 3\n";
    assert_eq!(read_ply(ply).expect("ascii ply").len(), 2);
    // Big-endian binary PLY with a per-face property before the list (the Armadillo layout)
    let mut be = b"ply\nformat binary_big_endian 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\nelement face 1\nproperty uchar intensity\nproperty list uchar int vertex_indices\nend_header\n".to_vec();
    for v in [[0.0f32, 0.0, 0.0], [2.0, 0.0, 0.0], [0.0, 3.0, 0.0]] {
        for c in v {
            be.extend_from_slice(&c.to_be_bytes());
        }
    }
    be.extend_from_slice(&[7, 3]);
    for i in [0i32, 1, 2] {
        be.extend_from_slice(&i.to_be_bytes());
    }
    let soup = read_ply(&be).expect("binary ply");
    assert_eq!(
        soup,
        vec![[
            Point3::ZERO,
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(0.0, 3.0, 0.0)
        ]]
    );
    // Garbage is an error, not a panic.
    assert!(read_stl(b"hello").is_err());
    assert!(read_ply(b"ply\nformat ascii 1.0\nend_header\n").is_err());
}

// ------------------------------------------------------------------------------- synthetic

fn spec(shape: Shape, defects: Vec<DefectSpec>, seed: u64) -> SynthSpec {
    SynthSpec {
        shape,
        defects,
        seed,
    }
}

fn sphere() -> Shape {
    Shape::Icosphere {
        frequency: 24,
        radius: 10.0,
    }
}

fn assert_clean(s: &QuickStats) {
    assert_eq!(s.boundary_edges, 0);
    assert_eq!(s.non_manifold_edges, 0);
    assert_eq!(s.inconsistent_edges, 0);
    assert_eq!(s.non_manifold_vertices, 0);
    assert_eq!(s.degenerate_faces, 0);
    assert_eq!(s.duplicate_faces, 0);
    assert_eq!(s.components, 1);
    assert!(s.signed_volume > 0.0);
}

#[test]
fn base_shapes_are_clean_closed_outward() {
    let shapes = [
        sphere(),
        Shape::DisplacedSphere {
            frequency: 16,
            radius: 10.0,
            amplitude: 0.06,
        },
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
    ];
    for shape in shapes {
        let m = generate(&spec(shape.clone(), vec![], 1)).mesh;
        let s = quick_stats(&m);
        assert_clean(&s);
        assert_eq!(s.boundary_loops, Some(vec![]), "{shape:?}");
    }
    // Euler characteristic of the plate: 2 - 2 * genus, genus = holes^2.
    let m = generate(&spec(
        Shape::BoxWithHoles {
            cells: 16,
            holes_per_side: 4,
            size: 40.0,
        },
        vec![],
        1,
    ))
    .mesh;
    let (v, f) = (m.vertex_count() as i64, m.face_count() as i64);
    assert_eq!(v - 3 * f / 2 + f, 2 - 2 * 16);
}

#[test]
fn holes_match_ground_truth() {
    let s = generate(&spec(
        sphere(),
        vec![DefectSpec::Holes {
            count: 6,
            max_boundary: 80,
        }],
        7,
    ));
    let q = quick_stats(&s.mesh);
    assert_eq!(s.truth.hole_loops.len(), 6);
    assert_eq!(q.boundary_loops.as_ref(), Some(&s.truth.hole_loops));
    assert_eq!(q.boundary_edges, s.truth.hole_loops.iter().sum::<usize>());
    assert_eq!(
        (
            q.non_manifold_edges,
            q.inconsistent_edges,
            q.non_manifold_vertices
        ),
        (0, 0, 0)
    );
    assert_eq!(q.components, 1);
    assert!(s.truth.hole_loops.iter().all(|&l| l >= 3));
    assert!(
        s.truth.hole_loops.iter().any(|&l| l > 3),
        "sizes should vary"
    );
}

#[test]
fn single_defect_classes_match_ground_truth() {
    let n = |s: &amg_mesh::corpus::synth::Synth| quick_stats(&s.mesh);

    let s = generate(&spec(
        sphere(),
        vec![DefectSpec::FlippedFaces { count: 20 }],
        3,
    ));
    let q = n(&s);
    assert_eq!(s.truth.flipped_faces.len(), 20);
    assert_eq!(q.inconsistent_edges, 60);
    assert_eq!((q.boundary_edges, q.non_manifold_edges), (0, 0));

    let s = generate(&spec(
        sphere(),
        vec![DefectSpec::DuplicateFaces { count: 15 }],
        3,
    ));
    let q = n(&s);
    assert_eq!(s.truth.duplicate_faces.len(), 15);
    assert_eq!(q.duplicate_faces, 15);
    assert_eq!(q.non_manifold_edges, s.truth.non_manifold_edges);
    assert_eq!(s.truth.non_manifold_edges, 45);

    let s = generate(&spec(
        sphere(),
        vec![DefectSpec::ZeroAreaSlivers { count: 10 }],
        3,
    ));
    let q = n(&s);
    assert_eq!(s.truth.zero_area_faces.len(), 10);
    assert_eq!(q.degenerate_faces, 10);
    assert_eq!(
        (q.boundary_edges, q.non_manifold_edges, q.inconsistent_edges),
        (0, 0, 0)
    );
    assert_eq!(q.components, 1);

    let s = generate(&spec(
        sphere(),
        vec![DefectSpec::NeedleSlivers { count: 10 }],
        3,
    ));
    let q = n(&s);
    assert_eq!(s.truth.needle_faces.len(), 10);
    assert_eq!(q.degenerate_faces, 0, "needles have positive area");
    assert_eq!(
        (q.boundary_edges, q.non_manifold_edges, q.inconsistent_edges),
        (0, 0, 0)
    );
    for &f in &s.truth.needle_faces {
        let [a, b, c] = s.mesh.triangle(amg_mesh::FaceId(f));
        let longest = (b - a).length().max((c - b).length()).max((a - c).length());
        let height = 2.0 * s.mesh.face_area(amg_mesh::FaceId(f)) / longest;
        assert!(
            height / longest < 1e-5,
            "needle aspect {}",
            height / longest
        );
    }

    let s = generate(&spec(
        sphere(),
        vec![DefectSpec::NonManifoldFins { count: 8 }],
        3,
    ));
    let q = n(&s);
    assert_eq!(s.truth.fins, 8);
    assert_eq!(q.non_manifold_edges, 8);
    assert_eq!((q.boundary_edges, q.non_manifold_vertices), (0, 0));
    assert_eq!(q.components, 1);

    let s = generate(&spec(sphere(), vec![DefectSpec::BowTies { count: 4 }], 3));
    let q = n(&s);
    assert_eq!(s.truth.bow_tie_vertices.len(), 4);
    assert_eq!(q.non_manifold_vertices, 4);
    assert_eq!(
        (q.boundary_edges, q.non_manifold_edges, q.inconsistent_edges),
        (0, 0, 0)
    );
    assert_eq!(q.components, 1);
}

#[test]
fn shell_defects_match_ground_truth() {
    let base_volume = quick_stats(&generate(&spec(sphere(), vec![], 5)).mesh).signed_volume;

    let s = generate(&spec(sphere(), vec![DefectSpec::OverlappingShell], 5));
    let q = quick_stats(&s.mesh);
    assert_eq!(
        (s.truth.overlapping_shells, s.truth.shells, q.components),
        (1, 2, 2)
    );
    assert!(s.truth.has_self_intersections());
    assert!((q.signed_volume - 2.0 * base_volume).abs() < 1e-6 * base_volume);

    let s = generate(&spec(sphere(), vec![DefectSpec::InvertedShell], 5));
    let q = quick_stats(&s.mesh);
    assert_eq!((s.truth.inverted_shells, q.components), (1, 2));
    let expected = base_volume * (1.0 - 1.0 / 64.0);
    assert!(
        (q.signed_volume - expected).abs() < 1e-6 * base_volume,
        "{} vs {expected}",
        q.signed_volume
    );

    let s = generate(&spec(
        sphere(),
        vec![DefectSpec::FloatingShells { count: 3 }],
        5,
    ));
    let q = quick_stats(&s.mesh);
    assert_eq!(
        (s.truth.floating_shells, s.truth.shells, q.components),
        (3, 4, 4)
    );
    assert!(q.signed_volume > base_volume, "tetrahedra are outward");
}

#[test]
fn unwelded_seam_welds_back_to_the_base() {
    let jitter = 1e-4;
    let s = generate(&spec(
        sphere(),
        vec![DefectSpec::UnweldedSeam { jitter }],
        9,
    ));
    assert!(!s.truth.seam_pairs.is_empty());
    let before = quick_stats(&s.mesh);
    assert!(before.boundary_edges > 0, "the seam is open before welding");
    for &(a, b) in &s.truth.seam_pairs {
        let d = s.mesh.positions[a as usize] - s.mesh.positions[b as usize];
        assert!(d.abs().max_element() <= jitter, "jitter {d:?}");
    }
    let mut m = s.mesh.clone();
    m.weld(WeldOpts { tolerance: 1e-3 }).expect("weld");
    m.compact();
    assert_clean(&quick_stats(&m));
    assert_eq!(m.vertex_count(), s.truth.base_vertices);
}

#[test]
fn f32_rounding_is_exact_at_1000_mm() {
    let s = generate(&spec(sphere(), vec![DefectSpec::F32Rounded], 2));
    assert_eq!(s.truth.f32_offset, Some(Vec3::new(1000.0, -1000.0, 1000.0)));
    for p in &s.mesh.positions {
        for c in p.to_array() {
            assert_eq!(
                f64::from(c as f32).to_bits(),
                c.to_bits(),
                "{c} is not an f32"
            );
        }
    }
    assert!(s.mesh.positions.iter().all(|p| p.x > 980.0));
}

fn all_defects() -> Vec<DefectSpec> {
    vec![
        DefectSpec::Holes {
            count: 4,
            max_boundary: 40,
        },
        DefectSpec::UnweldedSeam { jitter: 5e-5 },
        DefectSpec::FlippedFaces { count: 12 },
        DefectSpec::DuplicateFaces { count: 6 },
        DefectSpec::ZeroAreaSlivers { count: 5 },
        DefectSpec::NeedleSlivers { count: 5 },
        DefectSpec::NonManifoldFins { count: 4 },
        DefectSpec::BowTies { count: 3 },
        DefectSpec::OverlappingShell,
        DefectSpec::InvertedShell,
        DefectSpec::FloatingShells { count: 2 },
    ]
}

#[test]
fn combined_defects_are_additive_and_reproducible() {
    let sp = spec(
        Shape::DisplacedSphere {
            frequency: 40,
            radius: 10.0,
            amplitude: 0.05,
        },
        all_defects(),
        11,
    );
    let a = generate(&sp);
    let b = generate(&sp);
    assert_eq!(
        a.mesh.content_hash(),
        b.mesh.content_hash(),
        "same spec, same bits"
    );
    assert_eq!(a.mesh, b.mesh);
    assert_eq!(a.truth, b.truth);
    let other = generate(&SynthSpec {
        seed: 12,
        ..sp.clone()
    });
    assert_ne!(a.mesh.content_hash(), other.mesh.content_hash());

    // Defect order in the spec does not matter.
    let mut rev = sp.clone();
    rev.defects.reverse();
    assert_eq!(generate(&rev).mesh.content_hash(), a.mesh.content_hash());

    let mut m = a.mesh.clone();
    m.weld(WeldOpts { tolerance: 1e-3 }).expect("weld");
    let q = quick_stats(&m);
    let t = &a.truth;
    assert_eq!(q.non_manifold_edges, t.non_manifold_edges);
    assert_eq!(q.non_manifold_vertices, t.non_manifold_vertices);
    assert_eq!(q.components, t.shells);
    // A fin's two flaps share one vertex set, so each fin also counts as a duplicate pair.
    assert_eq!(q.duplicate_faces, t.duplicate_faces.len() + t.fins);
    assert_eq!(q.degenerate_faces, t.zero_area_faces.len());
    assert_eq!(t.hole_loops.len(), 4);
    assert!(q.boundary_edges >= t.hole_loops.iter().sum::<usize>());
    assert!(q.inconsistent_edges >= 3 * t.flipped_faces.len());
}

// ------------------------------------------------------------------------------- benchmark

#[test]
fn geodesic_spheres_are_closed_and_exact_in_size() {
    for k in [1u32, 2, 3, 7] {
        let m = geodesic_sphere(k, 5.0, None);
        let k = k as usize;
        assert_eq!(m.face_count(), 20 * k * k);
        assert_eq!(m.vertex_count(), 10 * k * k + 2);
        assert_clean(&quick_stats(&m));
        let r = 5.0;
        let ball = 4.0 / 3.0 * std::f64::consts::PI * r * r * r;
        assert!(m.signed_volume() < ball);
    }
    assert_eq!(BenchSize::K100.triangles(), 100_820);
    assert_eq!(BenchSize::M1.triangles(), 1_003_520);
    assert_eq!(BenchSize::M10.triangles(), 9_996_980);
}

#[test]
fn benchmark_sphere_100k_is_manifold_and_bit_identical_across_threads() {
    let a = benchmark_sphere(BenchSize::K100, 1);
    assert_eq!(a.face_count(), 100_820);
    assert_clean(&quick_stats(&a));
    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(1)
        .build()
        .expect("pool");
    let b = pool.install(|| benchmark_sphere(BenchSize::K100, 1));
    assert_eq!(a.content_hash(), b.content_hash());
    assert_ne!(
        a.content_hash(),
        benchmark_sphere(BenchSize::K100, 2).content_hash()
    );
}

#[test]
fn one_million_triangle_benchmark_pair_has_the_documented_size() {
    let (a, b) = boolean_pair(BenchSize::M1, 4);
    assert_eq!((a.face_count(), b.face_count()), (1_003_520, 1_003_520));
    let (ba, bb) = (a.aabb(), b.aabb());
    assert!(ba.max.x > bb.min.x, "the two spheres overlap");
    assert!(a.signed_volume() > 0.0 && b.signed_volume() > 0.0);
}

#[test]
#[ignore = "10 M triangles: about 1 GB of RAM; run with --ignored"]
fn ten_million_triangle_benchmark_sphere() {
    let m = benchmark_sphere(BenchSize::M10, 1);
    assert_eq!(m.face_count(), 9_996_980);
    assert_eq!(m.vertex_count(), 4_998_492);
    let _: &IndexedMesh = &m;
}
