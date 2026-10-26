//! MESH-12 tests: face-group segmentation on 20 synthetic tessellated CAD parts.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::print_stderr
)]

#[path = "segment_common/mod.rs"]
mod parts;

use amg_jobs::JobCtx;
use amg_mesh::{
    AnalyzeOpts, FaceGroupId, GroupOrigin, IndexedMesh, SegmentOpts, analyze, segment_face_groups,
};
use amg_testkit::TriSoup;
use parts::{PART_COUNT, Part, matched_fraction};

fn segmented(p: &Part, opts: &SegmentOpts) -> (IndexedMesh, usize) {
    let mut m = p.mesh.clone();
    let n = segment_face_groups(&mut m, opts, &JobCtx::detached()).unwrap();
    (m, n)
}

fn labels(m: &IndexedMesh) -> Vec<u32> {
    m.groups.per_face().iter().map(|g| g.0).collect()
}

/// Labels as dense ids (the groups of the mesh are numbered after the default group).
fn dense(m: &IndexedMesh) -> Vec<u32> {
    labels(m).iter().map(|l| l - 1).collect()
}

fn true_faces(p: &Part) -> usize {
    p.truth.iter().max().map_or(0, |&m| m as usize + 1)
}

#[test]
fn fixtures_are_closed_oriented_parts_with_contiguous_true_faces() {
    for noise in [false, true] {
        for p in parts::all_parts(noise) {
            p.mesh.validate().unwrap();
            let r = analyze(&p.mesh, &AnalyzeOpts::default(), &JobCtx::detached()).unwrap();
            assert!(r.is_watertight(), "{}: not watertight", p.name);
            assert!(r.volume.unwrap() > 0.0, "{}: not outward", p.name);
            assert_eq!(p.truth.len(), p.mesh.face_count());
            let mut seen = vec![0usize; true_faces(&p)];
            for &t in &p.truth {
                seen[t as usize] += 1;
            }
            assert!(
                seen.iter().all(|&c| c >= 32),
                "{}: a true face has fewer than 32 triangles: {seen:?}",
                p.name
            );
        }
    }
    assert_eq!(parts::all_parts(false).len(), 20);
}

/// Done-when: on 20 synthetic parts, with and without 0.02 mm noise, at least 95 % of the faces
/// get the ground-truth label after optimal matching.
#[test]
fn twenty_parts_match_ground_truth() {
    let (clean, noisy) = (parts::build(0, false), parts::build(0, true));
    let moved = clean
        .mesh
        .positions
        .iter()
        .zip(&noisy.mesh.positions)
        .map(|(a, b)| a.distance(*b))
        .fold(0.0f64, f64::max);
    assert!((0.01..0.04).contains(&moved), "noise displacement {moved}");
    for noise in [false, true] {
        let mut worst = (1.0f64, "");
        let mut total = 0.0;
        for i in 0..PART_COUNT {
            let p = parts::build(i, noise);
            let (m, n) = segmented(&p, &SegmentOpts::default());
            let frac = matched_fraction(&p.truth, &dense(&m));
            eprintln!(
                "noise={noise} {:>22}: {:>6} tris, {:>2} true faces, {:>3} groups, {:.4}",
                p.name,
                p.mesh.face_count(),
                true_faces(&p),
                n,
                frac
            );
            assert!(frac >= 0.95, "{} (noise={noise}): {frac:.4}", p.name);
            if frac < worst.0 {
                worst = (frac, p.name);
            }
            total += frac;
        }
        eprintln!(
            "noise={noise}: mean {:.4}, worst {:.4} ({})",
            total / PART_COUNT as f64,
            worst.0,
            worst.1
        );
    }
}

/// Without noise the true faces are found exactly: same count, every face matched.
#[test]
fn noise_free_parts_are_segmented_exactly() {
    for i in 0..PART_COUNT {
        let p = parts::build(i, false);
        let (m, n) = segmented(&p, &SegmentOpts::default());
        assert_eq!(n, true_faces(&p), "{}", p.name);
        assert!(
            matched_fraction(&p.truth, &dense(&m)) > 1.0 - 1e-12,
            "{}",
            p.name
        );
    }
}

#[test]
fn groups_are_recorded_as_segmentation_groups() {
    let p = parts::build(0, false);
    let (m, n) = segmented(&p, &SegmentOpts::default());
    m.validate().unwrap();
    let used: std::collections::BTreeSet<FaceGroupId> =
        m.groups.per_face().iter().copied().collect();
    assert_eq!(used.len(), n);
    for g in used {
        let info = m.groups.info(g).unwrap();
        assert_eq!(info.origin, GroupOrigin::Segmentation);
        assert!(m.groups.faces_of(g).next().is_some());
    }
    // The pre-existing default group stays in the table, empty.
    assert!(m.groups.faces_of(amg_mesh::DEFAULT_GROUP).next().is_none());
}

/// The result is identical on repeated runs and for 1, 4 and 13 threads.
#[test]
fn deterministic_across_runs_and_thread_counts() {
    for i in [0, 3, 11, 15, 18] {
        let p = parts::build(i, true);
        let reference = segmented(&p, &SegmentOpts::default()).0;
        assert_eq!(
            reference.content_hash(),
            segmented(&p, &SegmentOpts::default()).0.content_hash()
        );
        for threads in [1, 4, 13] {
            let pool = rayon::ThreadPoolBuilder::new()
                .num_threads(threads)
                .build()
                .unwrap();
            let m = pool.install(|| segmented(&p, &SegmentOpts::default()).0);
            assert_eq!(m, reference, "{} with {threads} threads", p.name);
        }
    }
}

/// Faces are numbered by their smallest face: group ids ascend along the first face of each.
#[test]
fn groups_are_numbered_by_first_face() {
    let p = parts::build(10, true);
    let (m, _) = segmented(&p, &SegmentOpts::default());
    let mut next = m.groups.per_face()[0].0;
    for g in m.groups.per_face() {
        assert!(g.0 <= next, "id {} seen before {}", g.0, next);
        if g.0 == next {
            next += 1;
        }
    }
}

#[test]
fn curvature_gating_keeps_the_exact_result_on_clean_parts() {
    let opts = SegmentOpts {
        curvature_threshold: Some(1.0),
        ..SegmentOpts::default()
    };
    for i in [1, 4, 8, 14, 17] {
        let p = parts::build(i, false);
        let (m, _) = segmented(&p, &opts);
        assert!(matched_fraction(&p.truth, &dense(&m)) >= 0.95, "{}", p.name);
    }
}

#[test]
fn a_tight_angle_threshold_splits_curved_faces() {
    let p = parts::build(0, false);
    let coarse = segmented(&p, &SegmentOpts::default()).1;
    let tight = segmented(
        &p,
        &SegmentOpts {
            angle_threshold: 1f64.to_radians(),
            min_group_faces: 0,
            ..SegmentOpts::default()
        },
    )
    .1;
    assert!(tight > coarse);
}

#[test]
fn golden_fixture_geometry() {
    for (i, name) in [(0, "segment_stepped_shaft"), (14, "segment_l_bracket")] {
        let p = parts::build(i, false);
        amg_testkit::assert_golden!(
            "mesh",
            name,
            TriSoup {
                positions: &p.mesh.positions,
                triangles: &p.mesh.tris,
            }
        );
    }
}

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}

/// Done-when: 1 M triangles in under 2 s. The bound is asserted only with
/// `AMG_STRICT_TIMING=1` (measured in `benches/mesh_segment.rs`).
#[test]
fn timing_one_million_triangles() {
    for noise in [false, true] {
        let p = parts::l_bracket_fine(15, noise);
        assert!(p.mesh.face_count() >= 1_000_000, "{}", p.mesh.face_count());
        let mut m = p.mesh.clone();
        let t = std::time::Instant::now();
        let n = segment_face_groups(&mut m, &SegmentOpts::default(), &JobCtx::detached()).unwrap();
        let secs = t.elapsed().as_secs_f64();
        eprintln!(
            "segment {} tris (noise={noise}): {n} groups in {secs:.2} s",
            m.face_count()
        );
        // The 0.02 mm noise is about a third of the 0.067 mm edge here, so only the clean mesh
        // has to come out right.
        if !noise {
            let frac = matched_fraction(&p.truth, &dense(&m));
            assert!(frac >= 0.95, "{frac}");
        }
        if strict_timing() {
            assert!(secs < 2.0, "segmentation took {secs:.2} s");
        }
    }
}
