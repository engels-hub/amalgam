use amg_core::{Point2, Point3, Vec3};

use super::*;
use crate::attrs::{Rgba8, TriProperty};
use crate::error::{MeshError, TopoError};
use crate::groups::{FaceGroupInfo, GroupOrigin};

fn mesh(p: &[[f64; 3]], t: &[[u32; 3]]) -> IndexedMesh {
    IndexedMesh::new(
        p.iter().map(|&a| Point3::from_array(a)).collect(),
        t.to_vec(),
    )
    .unwrap()
}

fn tetra() -> IndexedMesh {
    mesh(
        &[
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        &[[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]],
    )
}

/// 3 × 3 vertex grid (2 × 2 cells, 8 triangles), centre vertex 4.
fn grid2() -> IndexedMesh {
    let mut p = Vec::new();
    for y in 0..3 {
        for x in 0..3 {
            p.push([f64::from(x), f64::from(y), 0.0]);
        }
    }
    let mut t = Vec::new();
    for y in 0..2u32 {
        for x in 0..2u32 {
            let a = y * 3 + x;
            t.push([a, a + 1, a + 4]);
            t.push([a, a + 4, a + 3]);
        }
    }
    mesh(&p, &t)
}

fn build(m: &IndexedMesh) -> HalfEdgeMesh {
    let (h, r) = HalfEdgeMesh::from_indexed(m, NonManifoldPolicy::Fail).unwrap();
    assert_eq!(r, ConversionReport::default());
    h.check_invariants().unwrap();
    h
}

fn area(h: &HalfEdgeMesh) -> f64 {
    h.to_indexed().area()
}

#[test]
fn closed_tetra() {
    let h = build(&tetra());
    assert!(h.boundary_loops().is_empty());
    for v in h.vertices() {
        assert!(!h.is_boundary_vertex(v));
        assert_eq!(h.valence(v), 3);
        assert_eq!(h.neighbors(v).len(), 3);
    }
    for e in 0..12 {
        let e = HalfEdgeId(e);
        let t = h.twin(e).unwrap();
        assert_eq!(h.origin(t), h.target(e));
        assert_eq!(h.next(h.prev(e)), e);
    }
    assert_eq!(h.to_indexed(), tetra());
}

#[test]
fn open_grid_boundary() {
    let h = build(&grid2());
    let loops = h.boundary_loops();
    assert_eq!(loops.len(), 1);
    assert_eq!(loops[0].len(), 8);
    for w in loops[0].windows(2) {
        assert_eq!(h.target(w[0]), h.origin(w[1]));
    }
    assert!(!h.is_boundary_vertex(VertexId(4)));
    assert_eq!(h.valence(VertexId(4)), 6);
    assert!(h.is_boundary_vertex(VertexId(0)));
    assert_eq!(h.valence(VertexId(0)), 3);
    assert_eq!(h.valence(VertexId(2)), 2);
    // The stored half-edge of a boundary vertex is twin-less, and the CCW walk covers the fan.
    let o: Vec<_> = h.outgoing(VertexId(1)).collect();
    assert_eq!(o.len(), 3);
    assert!(h.twin(o[0]).is_none());
    assert_eq!(h.neighbors(VertexId(1)).len(), 4);
}

#[test]
fn bow_tie_splits_one_vertex() {
    // Two triangles touching at vertex 0.
    let m = mesh(
        &[
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
            [-1.0, -1.0, 0.0],
        ],
        &[[0, 1, 2], [0, 3, 4]],
    );
    assert!(matches!(
        HalfEdgeMesh::from_indexed(&m, NonManifoldPolicy::Fail),
        Err(MeshError::NonManifold {
            edges: 0,
            vertices: 1
        })
    ));
    let (h, r) = HalfEdgeMesh::from_indexed(&m, NonManifoldPolicy::SplitVertices).unwrap();
    assert_eq!(r.split_vertices, 1);
    assert_eq!(r.cut_edges, 0);
    h.check_invariants().unwrap();
    let out = h.to_indexed();
    assert_eq!(out.vertex_count(), 6);
    assert_eq!(out.tris[1][0], 5);
    assert_eq!(out.positions[5], Point3::ZERO);
}

#[test]
fn fin_cuts_edge_and_splits_endpoints() {
    // Three triangles on edge 0–1.
    let m = mesh(
        &[
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.5, 1.0, 0.0],
            [0.5, -1.0, 0.0],
            [0.5, 0.0, 1.0],
        ],
        &[[0, 1, 2], [1, 0, 3], [0, 1, 4]],
    );
    assert!(matches!(
        HalfEdgeMesh::from_indexed(&m, NonManifoldPolicy::Fail),
        Err(MeshError::NonManifold {
            edges: 1,
            vertices: 2
        })
    ));
    let (h, r) = HalfEdgeMesh::from_indexed(&m, NonManifoldPolicy::SplitVertices).unwrap();
    assert_eq!(r.cut_edges, 1);
    assert_eq!(r.split_vertices, 4);
    h.check_invariants().unwrap();
    assert_eq!(h.boundary_loops().len(), 3);
}

#[test]
fn orientation_conflict_is_cut_and_degenerates_dropped() {
    let m = mesh(
        &[
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.5, 1.0, 0.0],
            [0.5, -1.0, 0.0],
        ],
        &[[0, 1, 2], [0, 1, 3], [2, 2, 3]],
    );
    let (h, r) = HalfEdgeMesh::from_indexed(&m, NonManifoldPolicy::SplitVertices).unwrap();
    assert_eq!(r.cut_edges, 1);
    assert_eq!(r.split_vertices, 2);
    assert_eq!(r.dropped_degenerate, 1);
    assert_eq!(h.face_count(), 2);
    h.check_invariants().unwrap();
}

#[test]
fn split_interior_and_boundary_edges() {
    let mut h = build(&tetra());
    let e = h.find_halfedge(VertexId(0), VertexId(1)).unwrap();
    let m = h.split_edge(e, Point3::new(0.5, 0.0, 0.0)).unwrap();
    h.check_invariants().unwrap();
    assert_eq!(h.face_count(), 6);
    assert_eq!(h.valence(m), 4);
    let ix = h.to_indexed();
    assert!((ix.signed_volume() - tetra().signed_volume()).abs() < 1e-15);

    let mut g = build(&grid2());
    let a0 = area(&g);
    let b = h_boundary(&g);
    let p = (g.position(g.origin(b)) + g.position(g.target(b))) * 0.5;
    let m = g.split_edge(b, p).unwrap();
    g.check_invariants().unwrap();
    assert!(g.is_boundary_vertex(m));
    assert_eq!(g.valence(m), 3);
    assert_eq!(g.boundary_loops()[0].len(), 9);
    assert!((area(&g) - a0).abs() < 1e-12);
}

fn h_boundary(h: &HalfEdgeMesh) -> HalfEdgeId {
    (0..h.halfedge_slots() as u32)
        .map(HalfEdgeId)
        .find(|&e| h.twin(e).is_none())
        .unwrap()
}

#[test]
fn split_interpolates_attributes() {
    let mut m = mesh(
        &[[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [0.0, 2.0, 0.0]],
        &[[0, 1, 2]],
    );
    m.attrs.vertex_colors = Some(vec![Rgba8([0; 4]), Rgba8([200; 4]), Rgba8([0; 4])]);
    m.attrs.vertex_normals = Some(vec![Vec3::Z, Vec3::X, Vec3::Z]);
    m.attrs.corner_uvs = Some(vec![[Point2::ZERO, Point2::X, Point2::Y]]);
    m.attrs.tri_properties = Some(vec![TriProperty {
        group: 4,
        index: [10, 11, 12],
    }]);
    let mut h = build(&m);
    let v = h
        .split_edge(HalfEdgeId(0), Point3::new(0.5, 0.0, 0.0))
        .unwrap();
    h.check_invariants().unwrap();
    let out = h.to_indexed();
    assert_eq!(
        out.attrs.vertex_colors.as_ref().unwrap()[v.0 as usize],
        Rgba8([50; 4])
    );
    let n = out.attrs.vertex_normals.as_ref().unwrap()[v.0 as usize];
    assert!((n.length() - 1.0).abs() < 1e-12 && n.x > 0.0 && n.z > n.x);
    let uv = out.attrs.corner_uvs.as_ref().unwrap();
    assert_eq!(uv[0][1], Point2::new(0.25, 0.0));
    assert_eq!(uv[1][0], Point2::new(0.25, 0.0));
    let tp = out.attrs.tri_properties.as_ref().unwrap();
    assert_eq!(tp[1].group, 4);
    assert_eq!(tp[1].index, [10, 11, 12]);
    assert_eq!(out.groups.per_face().len(), 2);
}

#[test]
fn flip_rules() {
    // A quad of two triangles.
    let q = mesh(
        &[
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
        ],
        &[[0, 1, 2], [0, 2, 3]],
    );
    let mut h = build(&q);
    let d = h.find_halfedge(VertexId(0), VertexId(2)).unwrap();
    h.flip_edge(d).unwrap();
    h.check_invariants().unwrap();
    assert!(h.find_halfedge(VertexId(0), VertexId(2)).is_none());
    assert!(
        h.find_halfedge(VertexId(1), VertexId(3)).is_some()
            || h.find_halfedge(VertexId(3), VertexId(1)).is_some()
    );
    assert!((area(&h) - 1.0).abs() < 1e-15);
    let b = h.find_halfedge(VertexId(0), VertexId(1)).unwrap();
    assert_eq!(h.flip_edge(b), Err(TopoError::BoundaryEdge));

    let mut t = build(&tetra());
    let before = t.clone();
    assert_eq!(t.flip_edge(HalfEdgeId(0)), Err(TopoError::DuplicateEdge));
    assert_eq!(t, before);
}

#[test]
fn collapse_rules() {
    let mut g = build(&grid2());
    // Interior edge 4→1 (centre to boundary): both boundary? 4 is interior, so allowed.
    let e = g.find_halfedge(VertexId(4), VertexId(5)).unwrap();
    assert!(g.can_collapse(e));
    let keep = g.collapse_edge(e, Point3::new(1.5, 1.0, 0.0)).unwrap();
    assert_eq!(keep, VertexId(5));
    g.check_invariants().unwrap();
    assert_eq!(g.face_count(), 6);
    assert_eq!(g.vertex_count(), 8);
    assert!(!g.is_vertex_alive(VertexId(4)));
    // An interior edge between two boundary vertices is rejected.
    let e = g.find_halfedge(VertexId(1), VertexId(5)).unwrap();
    assert!(g.twin(e).is_some());
    assert!(!g.can_collapse(e));
    let before = g.clone();
    assert_eq!(
        g.collapse_edge(e, Point3::ZERO),
        Err(TopoError::LinkCondition)
    );
    assert_eq!(g, before);
    let remap = g.garbage_collect();
    assert_eq!(remap.vertices[4], INVALID);
    assert_eq!(remap.vertices[5], 4);
    g.check_invariants().unwrap();
    assert_eq!(g.vertex_slots(), 8);

    // A lone triangle cannot lose an edge.
    let mut one = build(&mesh(
        &[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
        &[[0, 1, 2]],
    ));
    assert!(!one.can_collapse(HalfEdgeId(0)));
    assert_eq!(
        one.collapse_edge(HalfEdgeId(0), Point3::ZERO),
        Err(TopoError::LinkCondition)
    );

    // Collapsing a tetrahedron edge leaves a closed two-face shell.
    let mut t = build(&tetra());
    let e = HalfEdgeId(0);
    assert!(t.can_collapse(e));
    t.collapse_edge(e, Point3::ZERO).unwrap();
    t.check_invariants().unwrap();
    assert_eq!(t.face_count(), 2);
    assert!(t.boundary_loops().is_empty());
}

#[test]
fn delete_and_add_faces() {
    let mut g = build(&grid2());
    // Face 0 = (0, 1, 4): deleting it opens a notch.
    g.delete_face(FaceId(0));
    g.check_invariants().unwrap();
    assert_eq!(g.face_count(), 7);
    let f = g
        .add_face(
            [VertexId(0), VertexId(1), VertexId(4)],
            crate::DEFAULT_GROUP,
        )
        .unwrap();
    g.check_invariants().unwrap();
    assert_eq!(f, FaceId(8));
    assert_eq!(g.boundary_loops()[0].len(), 8);
    // A duplicate directed edge is rejected; so is a face touching only at a vertex.
    assert_eq!(
        g.add_face(
            [VertexId(0), VertexId(1), VertexId(2)],
            crate::DEFAULT_GROUP
        ),
        Err(TopoError::DuplicateEdge)
    );
    let far = g.add_vertex(Point3::new(-1.0, -1.0, 0.0));
    let far2 = g.add_vertex(Point3::new(-2.0, -1.0, 0.0));
    let before = g.clone();
    assert_eq!(
        g.add_face([VertexId(0), far2, far], crate::DEFAULT_GROUP),
        Err(TopoError::NonManifold)
    );
    assert_eq!(g, before);
    // Interior vertex 4 cannot take another face.
    assert_eq!(
        g.add_face([VertexId(4), far, far2], crate::DEFAULT_GROUP),
        Err(TopoError::NonManifold)
    );
    // A face in the middle of a boundary vertex's fan: deleting it splits the vertex.
    let mut h = build(&grid2());
    // Face 3 (1,5,4) sits in the middle of the fans of boundary vertices 1 and 5.
    h.delete_face(FaceId(3));
    h.check_invariants().unwrap();
    assert_eq!(h.vertex_count(), 11);
    h.delete_vertex(VertexId(4));
    h.check_invariants().unwrap();
    let ix = h.to_indexed();
    assert!(ix.validate().is_ok());
    // Isolated vertices survive to_indexed (only deleted ones are dropped).
    assert_eq!(ix.vertex_count(), h.vertex_count());
}

#[test]
fn add_face_groups_and_gc_keep_attrs() {
    let mut m = grid2();
    m.attrs.tri_properties = Some(vec![TriProperty::NONE; 8]);
    let mut h = build(&m);
    let g = h.groups_mut().alloc(FaceGroupInfo::new(GroupOrigin::Fill));
    h.delete_face(FaceId(7));
    let f = h
        .add_face([VertexId(4), VertexId(8), VertexId(7)], g)
        .unwrap();
    assert_eq!(h.groups().group_of(f), g);
    let r = h.garbage_collect();
    assert_eq!(r.faces[7], INVALID);
    assert_eq!(r.faces[8], 7);
    h.check_invariants().unwrap();
    let out = h.to_indexed();
    assert!(out.validate().is_ok());
    assert_eq!(out.groups.group_of(FaceId(7)), g);
}

/// A vertex of valence 100 000: edge pairing must not scan per-vertex lists quadratically.
#[test]
fn high_valence_fan_converts() {
    const N: u32 = 100_000;
    let mut p = vec![Point3::ZERO];
    p.extend((0..=N).map(|i| {
        let a = f64::from(i) * 1e-5;
        Point3::new(a.cos(), a.sin(), 0.0)
    }));
    let t: Vec<[u32; 3]> = (1..=N).map(|i| [0, i, i + 1]).collect();
    let m = IndexedMesh::new(p, t).unwrap();
    let (h, rep) = HalfEdgeMesh::from_indexed(&m, NonManifoldPolicy::Fail).unwrap();
    assert_eq!(rep, ConversionReport::default());
    assert_eq!(h.face_count(), N as usize);
    assert_eq!(h.valence(VertexId(0)), N as usize + 1);
    assert_eq!(h.boundary_loops().len(), 1);
    assert_eq!(h.to_indexed(), m);
}

/// 100 000 faces on one edge: one cut edge, every face ends up on its own vertex copies, and
/// the result is independent of the thread count.
#[test]
fn high_valence_fin_cuts_once_and_is_deterministic() {
    const N: u32 = 100_000;
    let mut p = vec![Point3::ZERO, Point3::X];
    p.extend((0..N).map(|i| Point3::new(0.5, f64::from(i) + 1.0, 1.0)));
    let t: Vec<[u32; 3]> = (0..N).map(|i| [0, 1, i + 2]).collect();
    let m = IndexedMesh::new(p, t).unwrap();
    let run = |threads: usize| {
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap()
            .install(|| {
                let (h, rep) =
                    HalfEdgeMesh::from_indexed(&m, NonManifoldPolicy::SplitVertices).unwrap();
                h.check_invariants().unwrap();
                (h.to_indexed(), rep)
            })
    };
    let (a, rep) = run(1);
    assert_eq!(rep.cut_edges, 1);
    assert_eq!(a.face_count(), N as usize);
    let (b, rep_b) = run(8);
    assert_eq!(a, b);
    assert_eq!(rep, rep_b);
    assert!(matches!(
        HalfEdgeMesh::from_indexed(&m, NonManifoldPolicy::Fail),
        Err(MeshError::NonManifold { edges: 1, .. })
    ));
}
