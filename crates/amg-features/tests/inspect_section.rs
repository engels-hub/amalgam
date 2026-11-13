//! INS-02 section caps: watertightness on the MESH-28 corpus (note §3.4), thread determinism
//! and the one-plane timing budget.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::print_stderr,
    missing_docs
)] // tests

use std::time::Instant;

use amg_core::{Dir3, Plane, Point3, Tolerance, Transform, Vec3};
use amg_features::inspect::section::{CapOpts, SectionPlanes, section_mesh};
use amg_jobs::JobCtx;
use amg_mesh::corpus::bench::geodesic_sphere;
use amg_mesh::corpus::{FileStatus, Manifest, corpus_root, read_mesh};
use amg_mesh::{
    AnalyzeOpts, IndexedMesh, KeepSide, MeshBody, PlaneCutOpts, RepairOpts, WeldOpts, analyze,
    plane_cut, repair,
};

fn ctx() -> JobCtx {
    JobCtx::detached()
}

fn require_testdata() -> bool {
    std::env::var("AMG_REQUIRE_TESTDATA").is_ok_and(|v| v == "1")
}

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }

    fn dir(&mut self) -> Vec3 {
        loop {
            let v = Vec3::new(self.next(), self.next(), self.next()) * 2.0 - Vec3::ONE;
            let l = v.length();
            if l > 0.1 && l <= 1.0 {
                return v / l;
            }
        }
    }
}

/// Analysis of a kept half + cap after welding.
struct Capped {
    closed: bool,
    manifold: bool,
    oriented: bool,
    boundary_edges: usize,
    non_manifold_edges: usize,
    non_manifold_vertices: usize,
    open_curves: usize,
}

impl Capped {
    fn ok(&self) -> bool {
        self.closed && self.manifold && self.oriented && self.open_curves == 0
    }
}

impl std::fmt::Display for Capped {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "closed {} manifold {} oriented {} (boundary edges {}, non-manifold edges {}, \
             non-manifold vertices {}, open curves {})",
            self.closed,
            self.manifold,
            self.oriented,
            self.boundary_edges,
            self.non_manifold_edges,
            self.non_manifold_vertices,
            self.open_curves
        )
    }
}

fn snap() -> f64 {
    Tolerance::mesh().linear
}

/// `plane_cut(keep = Negative, cap = false)` + the INS-02 cap, welded at the snap tolerance
/// (note §3.4). `None` when nothing is kept.
fn capped_half(m: &IndexedMesh, body: &MeshBody, p: Plane) -> Option<Capped> {
    let cut = plane_cut(
        m,
        &PlaneCutOpts {
            plane: p,
            keep: KeepSide::Negative,
            cap: false,
            snap_tolerance: snap(),
        },
        &ctx(),
    )
    .unwrap();
    let half = cut.negative?;
    let s = section_mesh(
        body,
        &Transform::IDENTITY,
        &SectionPlanes(vec![p]),
        &CapOpts::default(),
        &ctx(),
    )
    .unwrap();
    let cap = &s.caps[0];
    let mut positions = half.positions.clone();
    let mut tris = half.tris.clone();
    let off = positions.len() as u32;
    positions.extend_from_slice(&cap.positions);
    tris.extend(cap.tris.iter().map(|t| t.map(|i| i + off)));
    let mut joined = IndexedMesh::new(positions, tris).unwrap();
    joined.weld(WeldOpts { tolerance: snap() }).unwrap();
    let rep = analyze(&joined, &AnalyzeOpts::default(), &ctx()).unwrap();
    Some(Capped {
        closed: rep.is_closed,
        manifold: rep.is_manifold,
        oriented: rep.is_oriented,
        boundary_edges: rep.boundary_edges,
        non_manifold_edges: rep.non_manifold_edges.len(),
        non_manifold_vertices: rep.non_manifold_vertices.len(),
        open_curves: cap.open.len(),
    })
}

fn assert_capped_closed(m: &IndexedMesh, body: &MeshBody, p: Plane) {
    if let Some(c) = capped_half(m, body, p) {
        assert!(c.ok(), "{c}");
    }
}

/// MESH-09's own capped half for the same plane: `(closed and manifold, description)`.
fn mesh09_capped(m: &IndexedMesh, p: Plane) -> (bool, String) {
    let cut = plane_cut(
        m,
        &PlaneCutOpts {
            plane: p,
            keep: KeepSide::Negative,
            cap: true,
            snap_tolerance: snap(),
        },
        &ctx(),
    )
    .unwrap();
    let Some(h) = cut.negative else {
        return (true, "empty".into());
    };
    let r = analyze(&h, &AnalyzeOpts::default(), &ctx()).unwrap();
    (
        r.is_closed && r.is_manifold,
        format!(
            "open loops {}, closed {} manifold {} (non-manifold vertices {})",
            cut.open_loops,
            r.is_closed,
            r.is_manifold,
            r.non_manifold_vertices.len()
        ),
    )
}

/// Done-when 1: for every watertight corpus model (as committed, or after MESH-07 repair for
/// models marked repairable; ≤ 200 k faces) and 3 seeded random planes plus a plane exactly
/// through a vertex and one containing an edge, the kept half plus the cap welds into a closed,
/// manifold, oriented mesh.
///
/// Exceptions, each one asserted to fail for MESH-09's own capped cut of the same plane too:
/// - a plane through a vertex where two section loops touch (a pinch): any planar cap of both
///   loops makes that vertex non-manifold (closed, no non-manifold edge, one non-manifold
///   vertex);
/// - loops that cross in the plane (vertices snapped onto it from both sides): no planar cap
///   matches the boundary edge for edge, and MESH-09 leaves them open.
///
/// Self-intersecting models are skipped for the same reason as the second exception.
#[test]
fn corpus_caps_are_watertight() {
    let manifest = Manifest::load().unwrap();
    let root = corpus_root();
    let (mut models, mut cuts, mut skipped_si) = (0usize, 0usize, 0usize);
    let (mut failures, mut pinches, mut crossings) = (Vec::new(), Vec::new(), Vec::new());
    for e in &manifest.meshes {
        if e.faces > 200_000 {
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
        let Ok(raw) = read_mesh(&path) else { continue };
        let o = AnalyzeOpts {
            self_intersections: true,
            ..AnalyzeOpts::default()
        };
        let watertight = |m: &IndexedMesh| analyze(m, &o, &ctx()).ok();
        let mut chosen = None;
        if let Some(rep) = watertight(&raw)
            && rep.is_watertight()
            && !rep.is_inverted
        {
            chosen = Some((raw.clone(), rep));
        } else if e.expect_repairable
            && let Ok((m, _)) = repair(&raw, &RepairOpts::default(), &ctx())
            && let Some(rep) = watertight(&m)
            && rep.is_watertight()
            && !rep.is_inverted
        {
            chosen = Some((m, rep));
        }
        let Some((m, rep)) = chosen else { continue };
        if rep
            .self_intersecting_pairs
            .as_ref()
            .is_none_or(|p| !p.is_empty())
        {
            skipped_si += 1;
            continue;
        }
        models += 1;
        let body = MeshBody::new(m.clone());
        let bb = m.aabb();
        let mut rng = Rng(0x5eed_0000 ^ e.faces as u64 ^ (e.id.len() as u64) << 40);
        let mut planes = Vec::new();
        for _ in 0..3 {
            let o = bb.min + (bb.max - bb.min) * Vec3::new(rng.next(), rng.next(), rng.next());
            planes.push(Plane::from_normal(o, Dir3::new_unchecked(rng.dir())));
        }
        // Exactly through a vertex, and containing an edge.
        let t = m.tris[(rng.next() * m.tris.len() as f64) as usize % m.tris.len()];
        let (a, b) = (m.positions[t[0] as usize], m.positions[t[1] as usize]);
        planes.push(Plane::from_normal(a, Dir3::new_unchecked(rng.dir())));
        let d = (b - a).normalize();
        let n = d.cross(rng.dir()).normalize();
        planes.push(Plane::from_normal(a, Dir3::new_unchecked(n)));
        for (k, p) in planes.into_iter().enumerate() {
            cuts += 1;
            let Some(c) = capped_half(&m, &body, p) else {
                continue;
            };
            if c.ok() {
                continue;
            }
            let kind = ["random", "random", "random", "vertex", "edge"][k];
            let (m09_ok, m09) = mesh09_capped(&m, p);
            let what = format!("{} ({kind}): {c}; MESH-09 cap: {m09}", e.id);
            let pinch = c.closed && c.oriented && c.non_manifold_edges == 0 && c.open_curves == 0;
            if m09_ok {
                failures.push(what);
            } else if pinch {
                pinches.push(what);
            } else {
                crossings.push(what);
            }
        }
    }
    eprintln!(
        "corpus section caps: {models} models, {cuts} cuts: {} closed and manifold, {} pinch \
         vertices, {} crossing loops (both also fail in MESH-09), {} failures; {skipped_si} \
         self-intersecting models skipped",
        cuts - pinches.len() - crossings.len() - failures.len(),
        pinches.len(),
        crossings.len(),
        failures.len()
    );
    for x in pinches.iter().chain(&crossings) {
        eprintln!("  exception: {x}");
    }
    assert!(failures.is_empty(), "{failures:#?}");
    if require_testdata() {
        assert!(models >= 20, "only {models} corpus models usable");
    }
}

/// Done-when 1 on synthetic solids with planes through vertices and edges.
#[test]
fn sphere_and_box_caps_are_watertight() {
    let s = geodesic_sphere(20, 10.0, None);
    let sb = MeshBody::new(s.clone());
    let v = s.positions[17];
    let mut rng = Rng(7);
    for p in [
        Plane::from_normal(Point3::ZERO, Dir3::Z),
        Plane::from_normal(v, Dir3::new_unchecked(rng.dir())),
        Plane::from_normal(Point3::new(0.3, 0.1, -0.2), Dir3::new_unchecked(rng.dir())),
    ] {
        assert_capped_closed(&s, &sb, p);
    }
    let c = amg_testkit::shapes::cube(Point3::ZERO, 10.0);
    let cm = IndexedMesh::new(c.positions, c.triangles).unwrap();
    let cb = MeshBody::new(cm.clone());
    for (o, n) in [
        (Point3::new(0.0, 0.0, 5.0), Vec3::Z),
        // Diagonal through two edges.
        (Point3::ZERO, Vec3::new(1.0, 1.0, 0.0).normalize()),
        // Through a corner.
        (
            Point3::new(10.0, 0.0, 0.0),
            Vec3::new(1.0, 1.0, 1.0).normalize(),
        ),
        // Through vertices.
        (
            Point3::new(5.0, 5.0, 5.0),
            Vec3::new(1.0, -1.0, 1.0).normalize(),
        ),
    ] {
        assert_capped_closed(&cm, &cb, Plane::from_normal(o, Dir3::new_unchecked(n)));
    }
}

/// Bit-identical results with 1 and 16 threads, each run twice, on several planes.
#[test]
fn deterministic_across_thread_counts() {
    let body = MeshBody::new(geodesic_sphere(60, 10.0, None));
    let mut rng = Rng(11);
    let planes = SectionPlanes(
        (0..4)
            .map(|_| {
                Plane::from_normal(
                    Point3::new(rng.next(), rng.next(), rng.next()) * 4.0,
                    Dir3::new_unchecked(rng.dir()),
                )
            })
            .collect(),
    );
    let run = |threads: usize| {
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap()
            .install(|| {
                section_mesh(
                    &body,
                    &Transform::IDENTITY,
                    &planes,
                    &CapOpts::default(),
                    &ctx(),
                )
                .unwrap()
            })
    };
    let a = run(1);
    assert!(a.caps.iter().any(|c| !c.tris.is_empty()));
    for t in [1, 16, 16] {
        let b = run(t);
        assert_eq!(a, b, "{t} threads");
        for (x, y) in a.caps.iter().zip(&b.caps) {
            let bits = |c: &amg_features::inspect::section::Cap| -> Vec<u64> {
                c.positions
                    .iter()
                    .flat_map(|p| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()])
                    .collect()
            };
            assert_eq!(bits(x), bits(y));
        }
    }
}

/// Done-when 7 (test profile; asserted only with `AMG_STRICT_TIMING=1`, the release numbers come
/// from `benches/inspect_section.rs`).
#[test]
fn timing_section_1m_one_plane() {
    let body = MeshBody::new(geodesic_sphere(224, 10.0, None));
    let planes = SectionPlanes(vec![Plane::from_normal(
        Point3::new(0.3, -0.2, 0.1),
        Dir3::new_unchecked(Vec3::new(0.3, 0.5, 0.8).normalize()),
    )]);
    let o = CapOpts::default();
    // Warm the per-mesh orientation flag.
    section_mesh(&body, &Transform::IDENTITY, &planes, &o, &ctx()).unwrap();
    let mut best = f64::INFINITY;
    for _ in 0..3 {
        let t = Instant::now();
        let r = section_mesh(&body, &Transform::IDENTITY, &planes, &o, &ctx()).unwrap();
        best = best.min(t.elapsed().as_secs_f64());
        assert!(r.stats[0].area.get() > 0.0);
    }
    eprintln!("section 1M one plane: {:.1} ms", best * 1e3);
    if strict_timing() {
        assert!(best < 0.05, "{best} s");
    }
}
