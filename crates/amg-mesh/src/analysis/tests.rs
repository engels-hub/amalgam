//! Hand-built meshes with known answers.

use amg_core::Point3;
use amg_jobs::JobCtx;

use super::*;

fn mesh(p: &[[f64; 3]], t: &[[u32; 3]]) -> IndexedMesh {
    IndexedMesh::new(
        p.iter().map(|&[x, y, z]| Point3::new(x, y, z)).collect(),
        t.to_vec(),
    )
    .expect("mesh")
}

fn run(m: &IndexedMesh) -> AnalysisReport {
    analyze(m, &AnalyzeOpts::default(), &JobCtx::detached()).expect("analyze")
}

/// Unit-edge tetrahedron corner at the origin, outward faces.
fn tet() -> IndexedMesh {
    mesh(
        &[[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
        &[[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]],
    )
}

#[test]
fn tetrahedron_is_watertight_with_exact_volume() {
    let r = run(&tet());
    assert!(r.is_watertight() && !r.is_inverted);
    assert_eq!(r.shells.len(), 1);
    assert!((r.volume.expect("volume") - 1.0 / 6.0).abs() < 1e-15);
    assert!((r.area - (1.5 + 3f64.sqrt() / 2.0)).abs() < 1e-12);
    assert_eq!(r.shell_of_face, vec![0; 4]);
}

#[test]
fn flipped_tetrahedron_is_inverted() {
    let mut m = tet();
    for t in &mut m.tris {
        t.swap(1, 2);
    }
    let r = run(&m);
    assert!(r.is_watertight() && r.is_inverted);
    assert!((r.volume.expect("volume") + 1.0 / 6.0).abs() < 1e-15);
}

#[test]
fn open_triangle_has_one_loop() {
    let m = mesh(&[[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]], &[[0, 1, 2]]);
    let r = run(&m);
    assert_eq!(r.boundary_loops, vec![vec![0, 1, 2]]);
    assert!(!r.is_closed && r.is_manifold && r.is_oriented);
    assert!(r.volume.is_none());
    assert!(!r.shells[0].closed);
}

#[test]
fn bow_tie_is_a_non_manifold_vertex_in_one_shell() {
    let m = mesh(
        &[
            [0., 0., 0.],
            [1., 0., 0.],
            [0., 1., 0.],
            [-1., 0., 0.],
            [0., -1., 0.],
        ],
        &[[0, 1, 2], [0, 3, 4]],
    );
    let r = run(&m);
    assert_eq!(r.non_manifold_vertices, vec![0]);
    assert!(r.non_manifold_edges.is_empty());
    assert_eq!(r.shells.len(), 1);
    assert_eq!(r.boundary_edges, 6);
    assert_eq!(r.boundary_loops, vec![vec![0, 1, 2], vec![0, 3, 4]]);
    assert_eq!(r.unclosed_boundary_loops, 0);
}

#[test]
fn fin_duplicate_degenerate_and_unreferenced() {
    // Three faces on edge 0-1 (a fin), a duplicate of face 0, a repeated-index face,
    // a zero-area face and an unused vertex.
    let m = mesh(
        &[
            [0., 0., 0.],
            [1., 0., 0.],
            [0., 1., 0.],
            [0., -1., 0.],
            [0., 0., 1.],
            [5., 5., 5.],
            [2., 0., 0.],
        ],
        &[
            [0, 1, 2],
            [1, 0, 3],
            [0, 1, 4],
            [2, 1, 0],
            [3, 3, 4],
            [0, 1, 6],
        ],
    );
    let r = run(&m);
    assert_eq!(r.non_manifold_edges, vec![[0, 1]]);
    assert_eq!(r.duplicate_faces, vec![FaceId(3)]);
    assert_eq!(r.degenerate_faces, vec![FaceId(4), FaceId(5)]);
    assert_eq!(r.shell_of_face[4], INVALID);
    assert_eq!(r.unreferenced_vertices, 1);
    assert!(!r.is_manifold);
}

#[test]
fn two_separate_tetrahedra_are_two_shells_in_vertex_order() {
    let mut m = tet();
    let mut far = tet();
    for p in &mut far.positions {
        p.x += 10.0;
    }
    m.append(&far).expect("append");
    let r = run(&m);
    assert_eq!(r.shells.len(), 2);
    assert_eq!(r.shell_of_face, vec![0, 0, 0, 0, 1, 1, 1, 1]);
    assert!(r.shells.iter().all(|s| s.closed && s.faces == 4));
    assert!((r.shells[1].aabb.min.x - 10.0).abs() < 1e-12);
    assert!(r.is_watertight());
}

#[test]
fn orientation_conflict_is_counted_per_edge() {
    let m = mesh(
        &[[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [1., 1., 0.]],
        &[[0, 1, 2], [0, 1, 3]],
    );
    let r = run(&m);
    assert_eq!(r.orientation_conflicts, 1);
    assert!(!r.is_oriented);
}
