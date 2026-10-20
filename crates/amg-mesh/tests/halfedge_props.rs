//! MESH-01 property tests: Indexed → HalfEdge → Indexed round trip, and half-edge invariants
//! under random topology edits.

#![allow(clippy::unwrap_used, clippy::float_cmp)] // test helpers; bit-exact comparisons are the point

mod common;

use amg_core::{Point2, Point3, Vec3};
use amg_mesh::{
    FaceGroupId, FaceGroupInfo, FaceId, GroupOrigin, HalfEdgeId, HalfEdgeMesh, IndexedMesh,
    MeshBody, NonManifoldPolicy, Rgba8, TopoError, TriProperty, VertexId,
};
use amg_testkit::TriSoup;
use common::{Rng, grid, torus};
use proptest::prelude::*;

/// Adds every attribute kind and a few face groups, derived from the indices.
fn decorate(mut m: IndexedMesh, salt: u32) -> IndexedMesh {
    let nv = m.vertex_count();
    let nf = m.face_count();
    m.attrs.vertex_normals = Some(
        (0..nv)
            .map(|i| Vec3::new(i as f64, 1.0, f64::from(salt)).normalize())
            .collect(),
    );
    m.attrs.vertex_colors = Some(
        (0..nv)
            .map(|i| Rgba8([i as u8, salt as u8, 7, 255]))
            .collect(),
    );
    m.attrs.corner_uvs = Some(
        (0..nf)
            .map(|f| {
                let b = f as f64;
                [
                    Point2::new(b, 0.0),
                    Point2::new(b, 1.0),
                    Point2::new(b, 2.0),
                ]
            })
            .collect(),
    );
    m.attrs.tri_properties = Some(
        (0..nf as u32)
            .map(|f| TriProperty {
                group: f % 3,
                index: [f, f + 1, f + 2],
            })
            .collect(),
    );
    let k = 1 + salt % 4;
    let ids: Vec<FaceGroupId> = (0..k)
        .map(|i| {
            m.groups
                .alloc(FaceGroupInfo::new(GroupOrigin::Import { object: i }))
        })
        .collect();
    for f in 0..nf {
        if f % 2 == 1 {
            m.groups.set(FaceId(f as u32), ids[f % ids.len()]);
        }
    }
    m.validate().unwrap();
    m
}

/// Checks the round trip of `m` under `SplitVertices`, and that `Fail` agrees with the report.
fn check_round_trip(m: &IndexedMesh) -> Result<(), TestCaseError> {
    let (he, rep) = HalfEdgeMesh::from_indexed(m, NonManifoldPolicy::SplitVertices).unwrap();
    he.check_invariants().map_err(TestCaseError::fail)?;
    let out = he.to_indexed();
    out.validate().unwrap();

    let kept: Vec<usize> = (0..m.face_count())
        .filter(|&f| {
            let t = m.tris[f];
            t[0] != t[1] && t[1] != t[2] && t[0] != t[2]
        })
        .collect();
    prop_assert_eq!(rep.dropped_degenerate, m.face_count() - kept.len());
    prop_assert_eq!(out.face_count(), kept.len());
    let nv = m.vertex_count();
    prop_assert_eq!(out.vertex_count(), nv + rep.split_vertices);
    for i in 0..nv {
        prop_assert_eq!(out.positions[i].to_array(), m.positions[i].to_array());
    }
    let (ia, oa) = (&m.attrs, &out.attrs);
    for (o, &f) in kept.iter().enumerate() {
        for k in 0..3 {
            let vi = m.tris[f][k] as usize;
            let vo = out.tris[o][k] as usize;
            prop_assert!(vo == vi || vo >= nv, "corner moved to another input vertex");
            prop_assert_eq!(out.positions[vo].to_array(), m.positions[vi].to_array());
            if let (Some(a), Some(b)) = (&ia.vertex_normals, &oa.vertex_normals) {
                prop_assert_eq!(a[vi], b[vo]);
            }
            if let (Some(a), Some(b)) = (&ia.vertex_colors, &oa.vertex_colors) {
                prop_assert_eq!(a[vi], b[vo]);
            }
        }
        if let (Some(a), Some(b)) = (&ia.corner_uvs, &oa.corner_uvs) {
            prop_assert_eq!(a[f], b[o]);
        }
        if let (Some(a), Some(b)) = (&ia.tri_properties, &oa.tri_properties) {
            prop_assert_eq!(a[f], b[o]);
        }
        prop_assert_eq!(
            m.groups.group_of(FaceId(f as u32)),
            out.groups.group_of(FaceId(o as u32))
        );
    }
    prop_assert_eq!(
        m.groups.ids().collect::<Vec<_>>(),
        out.groups.ids().collect::<Vec<_>>()
    );
    for g in m.groups.ids() {
        prop_assert_eq!(m.groups.info(g), out.groups.info(g));
    }
    if rep.split_vertices == 0 && rep.cut_edges == 0 && rep.dropped_degenerate == 0 {
        prop_assert_eq!(&out, m);
    }
    let fail = HalfEdgeMesh::from_indexed(m, NonManifoldPolicy::Fail);
    prop_assert_eq!(fail.is_ok(), rep.split_vertices == 0 && rep.cut_edges == 0);
    // The split output is manifold: converting it again changes nothing.
    let (again, rep2) = HalfEdgeMesh::from_indexed(&out, NonManifoldPolicy::Fail)
        .map_err(|e| TestCaseError::fail(format!("split output not manifold: {e}")))?;
    prop_assert_eq!(
        rep2.split_vertices + rep2.cut_edges + rep2.dropped_degenerate,
        0
    );
    prop_assert_eq!(again.to_indexed(), out);
    Ok(())
}

/// Random soup over few vertices: lots of fins, bow-ties, duplicates and degenerates.
fn soup_strategy() -> impl Strategy<Value = IndexedMesh> {
    (3u32..12, any::<u32>()).prop_flat_map(|(n, salt)| {
        prop::collection::vec([0..n, 0..n, 0..n], 1..40).prop_map(move |tris| {
            let mut r = Rng(u64::from(salt));
            let p = (0..n).map(|_| r.vec3()).collect();
            decorate(IndexedMesh::new(p, tris).unwrap(), salt)
        })
    })
}

/// Grids with random diagonals, deleted faces (bow-ties) and flipped faces (conflicts).
fn grid_strategy() -> impl Strategy<Value = IndexedMesh> {
    (1u32..7, 1u32..7, any::<u64>(), 0u32..4).prop_map(|(w, h, seed, mode)| {
        let mut r = Rng(seed);
        let diag: Vec<bool> = (0..w * h).map(|_| r.below(2) == 1).collect();
        let mut m = grid(w, h, |i| diag[i]);
        if mode & 1 == 1 {
            let keep: Vec<[u32; 3]> = m.tris.iter().copied().filter(|_| r.below(4) != 0).collect();
            if !keep.is_empty() {
                m = IndexedMesh::new(m.positions.clone(), keep).unwrap();
            }
        }
        if mode & 2 == 2 {
            for t in &mut m.tris {
                if r.below(8) == 0 {
                    t.swap(1, 2);
                }
            }
        }
        decorate(m, seed as u32)
    })
}

proptest! {
    #![proptest_config(ProptestConfig { cases: 1000, ..ProptestConfig::default() })]

    #[test]
    fn round_trip_random_soups(m in soup_strategy()) {
        check_round_trip(&m)?;
    }

    #[test]
    fn round_trip_random_grids(m in grid_strategy()) {
        check_round_trip(&m)?;
    }
}

// ---- topology edits ---------------------------------------------------------------------

fn euler(h: &HalfEdgeMesh) -> i64 {
    let ix = h.to_indexed();
    let mut edges = std::collections::BTreeSet::new();
    for t in &ix.tris {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            edges.insert((a.min(b), a.max(b)));
        }
    }
    let used: std::collections::BTreeSet<u32> = ix.tris.iter().flatten().copied().collect();
    used.len() as i64 - edges.len() as i64 + ix.face_count() as i64
}

#[derive(Debug, Clone)]
enum Op {
    Split(u32, f64),
    Collapse(u32, f64),
    Flip(u32),
    DeleteFace(u32),
    DeleteVertex(u32),
    AddFace([u32; 3]),
    Move(u32, [f64; 3]),
    Gc,
}

fn op_strategy() -> impl Strategy<Value = Op> {
    prop_oneof![
        4 => (any::<u32>(), 0.0..1.0).prop_map(|(h, t)| Op::Split(h, t)),
        4 => (any::<u32>(), 0.0..1.0).prop_map(|(h, t)| Op::Collapse(h, t)),
        4 => any::<u32>().prop_map(Op::Flip),
        1 => any::<u32>().prop_map(Op::DeleteFace),
        1 => any::<u32>().prop_map(Op::DeleteVertex),
        2 => any::<[u32; 3]>().prop_map(Op::AddFace),
        1 => (any::<u32>(), any::<[f64; 3]>()).prop_map(|(v, d)| Op::Move(v, d)),
        1 => Just(Op::Gc),
    ]
}

fn apply(h: &mut HalfEdgeMesh, op: &Op) -> Result<(), TestCaseError> {
    let n_he = h.halfedge_slots() as u32;
    let before = h.clone();
    let chi = euler(h);
    let faces = h.face_count();
    let needs_faces = !matches!(
        op,
        Op::AddFace(_) | Op::Move(..) | Op::Gc | Op::DeleteVertex(_)
    );
    if (needs_faces && n_he == 0) || h.vertex_slots() == 0 {
        return Ok(());
    }
    match *op {
        Op::Split(e, t) => {
            let e = HalfEdgeId(e % n_he);
            let a = h.position(h.origin(e));
            let b = h.position(h.target(e));
            let boundary = h.twin(e).is_none();
            match h.split_edge(e, a.lerp(b, t)) {
                Ok(_) => {
                    prop_assert_eq!(h.face_count(), faces + if boundary { 1 } else { 2 });
                    prop_assert_eq!(euler(h), chi);
                }
                Err(err) => {
                    prop_assert!(matches!(err, TopoError::Dead | TopoError::NonManifold));
                    prop_assert_eq!(&*h, &before);
                }
            }
        }
        Op::Collapse(e, t) => {
            let e = HalfEdgeId(e % n_he);
            let ok = h.can_collapse(e);
            let p = if h.is_face_alive(h.face(e)) {
                h.position(h.origin(e)).lerp(h.position(h.target(e)), t)
            } else {
                Point3::ZERO
            };
            let boundary = h.twin(e).is_none();
            match h.collapse_edge(e, p) {
                Ok(v) => {
                    prop_assert!(ok);
                    prop_assert!(h.is_vertex_alive(v));
                    prop_assert_eq!(h.face_count(), faces - if boundary { 1 } else { 2 });
                    prop_assert_eq!(euler(h), chi);
                }
                Err(_) => {
                    prop_assert!(!ok);
                    prop_assert_eq!(&*h, &before);
                }
            }
        }
        Op::Flip(e) => {
            let e = HalfEdgeId(e % n_he);
            match h.flip_edge(e) {
                Ok(()) => {
                    prop_assert_eq!(h.face_count(), faces);
                    prop_assert_eq!(euler(h), chi);
                }
                Err(_) => prop_assert_eq!(&*h, &before),
            }
        }
        Op::DeleteFace(f) => {
            let f = FaceId(f % h.face_slots() as u32);
            let alive = h.is_face_alive(f);
            h.delete_face(f);
            prop_assert_eq!(h.face_count(), faces - usize::from(alive));
        }
        Op::DeleteVertex(v) => {
            let v = VertexId(v % h.vertex_slots() as u32);
            let alive = h.is_vertex_alive(v);
            let incident = h
                .faces()
                .filter(|&f| h.face_vertices(f).contains(&v))
                .count();
            h.delete_vertex(v);
            prop_assert!(!h.is_vertex_alive(v));
            if alive {
                prop_assert_eq!(h.face_count(), faces - incident);
            } else {
                prop_assert_eq!(&*h, &before);
            }
        }
        Op::AddFace(v) => {
            let n = h.vertex_slots() as u32;
            let v = v.map(|x| VertexId(x % n));
            match h.add_face(v, amg_mesh::DEFAULT_GROUP) {
                Ok(f) => prop_assert_eq!(h.face_vertices(f), v),
                Err(_) => prop_assert_eq!(&*h, &before),
            }
        }
        Op::Move(v, d) => {
            let v = VertexId(v % h.vertex_slots() as u32);
            if h.is_vertex_alive(v) && d.iter().all(|x| x.is_finite()) {
                let p = h.position(v) + Vec3::from_array(d).clamp_length_max(0.1);
                h.set_position(v, p);
            }
        }
        Op::Gc => {
            let r = h.garbage_collect();
            prop_assert_eq!(r.faces.len(), before.face_slots());
            prop_assert_eq!(h.face_count(), faces);
            prop_assert_eq!(h.face_slots(), faces);
        }
    }
    h.check_invariants()
        .map_err(|e| TestCaseError::fail(format!("after {op:?}: {e}")))
}

fn start_mesh(kind: u8, seed: u64) -> HalfEdgeMesh {
    let mut r = Rng(seed);
    let m = match kind % 3 {
        0 => torus(5 + r.below(4) as u32, 4 + r.below(4) as u32, 10.0, 3.0),
        1 => {
            let diag: Vec<bool> = (0..64).map(|_| r.below(2) == 1).collect();
            grid(1 + r.below(6) as u32, 1 + r.below(6) as u32, |i| diag[i])
        }
        _ => {
            // Closed icosahedron-like shape: an octahedron.
            IndexedMesh::new(
                vec![
                    Point3::X,
                    Point3::NEG_X,
                    Point3::Y,
                    Point3::NEG_Y,
                    Point3::Z,
                    Point3::NEG_Z,
                ],
                vec![
                    [0, 2, 4],
                    [2, 1, 4],
                    [1, 3, 4],
                    [3, 0, 4],
                    [2, 0, 5],
                    [1, 2, 5],
                    [3, 1, 5],
                    [0, 3, 5],
                ],
            )
            .unwrap()
        }
    };
    let m = decorate(m, seed as u32);
    let (h, rep) = HalfEdgeMesh::from_indexed(&m, NonManifoldPolicy::Fail).unwrap();
    assert_eq!(rep.split_vertices + rep.cut_edges, 0);
    h
}

proptest! {
    #![proptest_config(ProptestConfig { cases: 500, ..ProptestConfig::default() })]

    #[test]
    fn topology_edits_keep_invariants(
        kind in any::<u8>(),
        seed in any::<u64>(),
        ops in prop::collection::vec(op_strategy(), 1..80),
    ) {
        let mut h = start_mesh(kind, seed);
        h.check_invariants().map_err(TestCaseError::fail)?;
        for op in &ops {
            apply(&mut h, op)?;
        }
        // The edited mesh is still a manifold: it converts back without any splits, and
        // survives the binary round trip bit-exactly.
        let ix = h.to_indexed();
        let (again, rep) = HalfEdgeMesh::from_indexed(&ix, NonManifoldPolicy::Fail)
            .map_err(|e| TestCaseError::fail(format!("edited mesh not manifold: {e}")))?;
        prop_assert_eq!(rep.split_vertices + rep.cut_edges + rep.dropped_degenerate, 0);
        again.check_invariants().map_err(TestCaseError::fail)?;
        let mut buf = Vec::new();
        MeshBody::new(ix.clone()).write_binary(&mut buf).unwrap();
        let back = MeshBody::read_binary(buf.as_slice()).unwrap();
        prop_assert_eq!(&**back.mesh(), &ix);
    }
}

#[test]
fn torus_conversion_is_closed_and_exact() {
    let m = decorate(torus(24, 16, 10.0, 3.0), 3);
    let (h, rep) = HalfEdgeMesh::from_indexed(&m, NonManifoldPolicy::Fail).unwrap();
    assert_eq!(
        rep.split_vertices + rep.cut_edges + rep.dropped_degenerate,
        0
    );
    h.check_invariants().unwrap();
    assert!(h.boundary_loops().is_empty());
    assert!(h.vertices().all(|v| h.valence(v) == 6));
    assert_eq!(h.to_indexed(), m);
    let vol = amg_testkit::signed_volume(TriSoup {
        positions: &m.positions,
        triangles: &m.tris,
    });
    assert!((vol - m.signed_volume()).abs() <= 1e-9 * vol.abs());
    let exact = 2.0 * std::f64::consts::PI.powi(2) * 10.0 * 9.0;
    assert!(
        vol > 0.0 && (vol - exact).abs() / exact < 0.05,
        "{vol} vs {exact}"
    );
}

/// Deterministic long edit run: every operation kind must actually succeed many times (so the
/// property test above is not vacuous), with invariants checked after each step.
#[test]
fn long_edit_run_exercises_every_op() {
    let mut h = start_mesh(0, 7);
    let mut r = Rng(99);
    let mut ok = [0usize; 5];
    for step in 0..4000 {
        let n_he = h.halfedge_slots() as u64;
        let e = HalfEdgeId(r.below(n_he) as u32);
        let alive = h.is_face_alive(h.face(e));
        let res = match step % 5 {
            0 if alive => {
                let p = h.position(h.origin(e)).lerp(h.position(h.target(e)), 0.5);
                h.split_edge(e, p).is_ok()
            }
            1 if alive => {
                let p = h.position(h.target(e));
                h.collapse_edge(e, p).is_ok()
            }
            2 => h.flip_edge(e).is_ok(),
            3 if alive && r.below(4) == 0 => {
                let f = h.face(e);
                let v = h.face_vertices(f);
                let g = h.groups().group_of(f);
                h.delete_face(f);
                h.add_face(v, g).is_ok()
            }
            4 if step % 500 == 4 => {
                h.garbage_collect();
                true
            }
            _ => false,
        };
        if res {
            ok[step % 5] += 1;
        }
        h.check_invariants()
            .unwrap_or_else(|err| panic!("step {step}: {err}"));
    }
    assert!(
        ok.iter().take(4).all(|&n| n >= 50),
        "successful ops per kind: {ok:?}"
    );
    assert_eq!(euler(&h), 0, "torus genus changed");
}

#[test]
fn conversion_and_weld_are_independent_of_thread_count() {
    let mut r = Rng(5);
    // A soup of a torus plus random non-manifold triangles over its vertices.
    let base = torus(40, 30, 10.0, 3.0);
    let mut tris = base.tris.clone();
    let n = base.vertex_count() as u64;
    for _ in 0..500 {
        tris.push([r.below(n) as u32, r.below(n) as u32, r.below(n) as u32]);
    }
    let m = decorate(IndexedMesh::new(base.positions.clone(), tris).unwrap(), 1);
    let soup = common::soup(&m);
    let run = |threads: usize| {
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap()
            .install(|| {
                let (h, rep) =
                    HalfEdgeMesh::from_indexed(&m, NonManifoldPolicy::SplitVertices).unwrap();
                let w = IndexedMesh::from_soup(&soup, amg_mesh::WeldOpts::default()).unwrap();
                (h.to_indexed().content_hash(), rep, w.content_hash())
            })
    };
    let a = run(1);
    assert!(a.1.cut_edges > 0 && a.1.split_vertices > 0);
    assert_eq!(a, run(4));
    assert_eq!(a, run(16));
}
