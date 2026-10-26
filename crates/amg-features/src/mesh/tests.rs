//! MESH-16 tests: features in a real `Document`, references, save/load.

#![allow(clippy::unwrap_used, clippy::expect_used, reason = "tests")]

use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};

use amg_core::Point3;
use amg_doc::{
    BodyGeom, BodyId, Command, CreatedId, Dependency, Design, DesignMode, Document, FeatureBox,
    FeatureError, FeatureId, FeatureOutput, Health, MatchQuality, RecomputeCtx, SaveOptions, load,
    save,
};
use amg_jobs::{JobSystem, JobSystemConfig};
use amg_mesh::{
    FaceGroupId, FaceGroupInfo, FaceId, GroupOrigin, IndexedMesh, MeshBody, RepairOpts,
};
use serde::{Deserialize, Serialize};

use super::*;

/// Test-only mesh source (stands in for the importers of S12): creates body slot 0.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct TestMeshSource {
    mesh: IndexedMesh,
}

impl amg_doc::Feature for TestMeshSource {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Ok(FeatureOutput {
            bodies: vec![amg_doc::BodyChange {
                id: ctx.new_body_id(0),
                change: amg_doc::BodyChangeKind::Created {
                    geom: BodyGeom::Mesh(Arc::new(self.mesh.clone())),
                    naming: None,
                },
            }],
            ..FeatureOutput::default()
        })
    }
}

amg_doc::register_feature!(TestMeshSource, "test.mesh_source", version = 1);

fn jobs() -> JobSystem {
    JobSystem::new(JobSystemConfig::default()).unwrap()
}

/// Outward-oriented UV sphere (radius 10), `lat` rings by `lon` segments, as a welded mesh.
fn uv_sphere(lat: u32, lon: u32) -> IndexedMesh {
    let mut pos = vec![Point3::new(0.0, 0.0, 10.0)];
    for i in 1..lat {
        let th = std::f64::consts::PI * f64::from(i) / f64::from(lat);
        for j in 0..lon {
            let ph = std::f64::consts::TAU * f64::from(j) / f64::from(lon);
            pos.push(Point3::new(
                10.0 * th.sin() * ph.cos(),
                10.0 * th.sin() * ph.sin(),
                10.0 * th.cos(),
            ));
        }
    }
    pos.push(Point3::new(0.0, 0.0, -10.0));
    let south = (pos.len() - 1) as u32;
    let ring = |i: u32, j: u32| 1 + (i - 1) * lon + j % lon;
    let mut tris = Vec::new();
    for j in 0..lon {
        tris.push([0, ring(1, j), ring(1, j + 1)]);
    }
    for i in 1..lat - 1 {
        for j in 0..lon {
            let (a, b, c, d) = (
                ring(i, j),
                ring(i + 1, j),
                ring(i + 1, j + 1),
                ring(i, j + 1),
            );
            tris.push([a, b, c]);
            tris.push([a, c, d]);
        }
    }
    for j in 0..lon {
        tris.push([south, ring(lat - 1, j + 1), ring(lat - 1, j)]);
    }
    let mut m = IndexedMesh::new(pos, tris).unwrap();
    if m.signed_volume() < 0.0 {
        m = reverse_normals(&m, None);
    }
    assert!(m.signed_volume() > 0.0);
    m
}

/// A sphere with two groups (`top`: z > 0, group 0 below), one missing face and one
/// duplicated face: repair has work to do.
fn defective_sphere() -> (IndexedMesh, FaceGroupId) {
    let mut m = uv_sphere(10, 16);
    let top = m.groups.alloc(FaceGroupInfo::new(GroupOrigin::User));
    let faces = m.face_count();
    for f in 0..faces {
        let id = FaceId(f as u32);
        let c = m.triangle(id).iter().map(|p| p.z).sum::<f64>() / 3.0;
        if c > 0.0 {
            m.groups.set(id, top);
        }
    }
    // Remove a bottom face, duplicate a top face.
    let drop = (0..faces)
        .find(|f| m.groups.group_of(FaceId(*f as u32)) != top)
        .unwrap();
    let dup = (0..faces)
        .find(|f| m.groups.group_of(FaceId(*f as u32)) == top)
        .unwrap();
    let mut tris = m.tris.clone();
    let mut per_face: Vec<_> = m.groups.per_face().to_vec();
    tris.push(tris[dup]);
    per_face.push(top);
    tris.remove(drop);
    per_face.remove(drop);
    let mut out = IndexedMesh::new(m.positions.clone(), tris).unwrap();
    for (i, g) in per_face.iter().enumerate() {
        // The table is shared by id: allocate until `top` exists, then assign.
        while out.groups.next_id() <= g.0 {
            out.groups.alloc(FaceGroupInfo::new(GroupOrigin::User));
        }
        out.groups.set(FaceId(i as u32), *g);
    }
    (out, top)
}

fn feature<F: amg_doc::Feature>(f: F) -> FeatureBox {
    FeatureBox::new(f)
}

struct Fixture {
    doc: Document,
    jobs: JobSystem,
}

impl Fixture {
    fn new() -> Fixture {
        link();
        Fixture {
            doc: Document::new("mesh"),
            jobs: jobs(),
        }
    }

    fn add(&mut self, f: FeatureBox) -> FeatureId {
        let root = self.doc.design().root();
        let out = self
            .doc
            .apply(Command::AddFeature {
                at: None,
                feature: f,
                context: root,
                name: None,
            })
            .unwrap();
        match out.created[0] {
            CreatedId::Feature(id) => id,
            ref other => panic!("{other:?}"),
        }
    }

    fn edit(&mut self, id: FeatureId, f: FeatureBox) {
        self.doc
            .apply(Command::EditFeature { id, feature: f })
            .unwrap();
    }

    fn recompute(&mut self) {
        self.doc.recompute_blocking(&self.jobs).unwrap();
    }

    fn mesh(&self, id: BodyId) -> Arc<IndexedMesh> {
        match &self.doc.evaluated().body(id).unwrap().geom {
            BodyGeom::Mesh(m) => Arc::clone(m),
            other => panic!("{other:?}"),
        }
    }

    fn computed(&self) -> Vec<FeatureId> {
        self.doc.last_run().unwrap().computed.clone()
    }

    fn cached(&self) -> Vec<FeatureId> {
        self.doc.last_run().unwrap().cached.clone()
    }
}

fn body(origin: FeatureId) -> BodyId {
    BodyId { origin, slot: 0 }
}

fn source(m: &IndexedMesh) -> FeatureBox {
    feature(TestMeshSource { mesh: m.clone() })
}

#[test]
fn repair_runs_as_a_feature_and_undo_redo_restore_the_exact_hash() {
    let mut fx = Fixture::new();
    let (mesh, _) = defective_sphere();
    let src = fx.add(source(&mesh));
    fx.recompute();
    let h0 = fx.mesh(body(src)).content_hash();
    assert_eq!(h0, mesh.content_hash());
    assert!(
        !amg_mesh::analyze(&mesh, &Default::default(), &amg_jobs::JobCtx::detached())
            .unwrap()
            .is_watertight()
    );

    let rep = fx.add(feature(MeshRepairFeature::new(body(src))));
    fx.recompute();
    assert_eq!(fx.doc.evaluated().node(rep).unwrap().health, Health::Ok);
    let repaired = fx.mesh(body(src));
    let h1 = repaired.content_hash();
    assert_ne!(h1, h0);
    assert!(
        amg_mesh::analyze(
            &repaired,
            &Default::default(),
            &amg_jobs::JobCtx::detached()
        )
        .unwrap()
        .is_watertight()
    );
    assert!(repaired.signed_volume() > 0.0);

    assert!(fx.doc.undo());
    fx.recompute();
    assert_eq!(fx.mesh(body(src)).content_hash(), h0, "undo: exact hash");
    assert!(fx.doc.redo());
    fx.recompute();
    assert_eq!(fx.mesh(body(src)).content_hash(), h1, "redo: exact hash");
}

#[test]
fn clean_mesh_is_returned_as_the_same_allocation() {
    let mut fx = Fixture::new();
    let src = fx.add(source(&uv_sphere(8, 12)));
    fx.add(feature(MeshRepairFeature::new(body(src))));
    fx.recompute();
    let a = fx.mesh(body(src));
    let src_out = fx.doc.evaluated().body(body(src)).unwrap();
    assert!(matches!(&src_out.geom, BodyGeom::Mesh(m) if Arc::ptr_eq(m, &a)));
    // Same allocation as the source output: look it up through a second document run.
    let mut fx2 = Fixture::new();
    let s2 = fx2.add(source(&uv_sphere(8, 12)));
    fx2.recompute();
    let before = fx2.mesh(body(s2));
    fx2.add(feature(MeshRepairFeature::new(body(s2))));
    fx2.recompute();
    assert!(Arc::ptr_eq(&before, &fx2.mesh(body(s2))));
}

#[test]
fn new_body_mode_keeps_the_input() {
    let mut fx = Fixture::new();
    let (mesh, _) = defective_sphere();
    let src = fx.add(source(&mesh));
    let mut r = MeshRepairFeature::new(body(src));
    r.output = OutputMode::NewBody;
    let rep = fx.add(feature(r));
    fx.recompute();
    assert_eq!(fx.mesh(body(src)).content_hash(), mesh.content_hash());
    assert_ne!(fx.mesh(body(rep)).content_hash(), mesh.content_hash());
}

#[test]
fn reverse_normal_negates_volume_and_twice_is_identity() {
    let mut fx = Fixture::new();
    let m = uv_sphere(8, 12);
    let src = fx.add(source(&m));
    let r1 = fx.add(feature(MeshReverseNormalFeature {
        body: body(src),
        groups: None,
    }));
    fx.recompute();
    assert_eq!(fx.doc.evaluated().node(r1).unwrap().health, Health::Ok);
    let flipped = fx.mesh(body(src));
    assert!((flipped.signed_volume() + m.signed_volume()).abs() <= 1e-9 * m.signed_volume());
    fx.add(feature(MeshReverseNormalFeature {
        body: body(src),
        groups: None,
    }));
    fx.recompute();
    assert_eq!(fx.mesh(body(src)).content_hash(), m.content_hash());
}

#[test]
fn reverse_normal_of_a_group_flips_only_that_group() {
    let (mesh, top) = defective_sphere();
    let r = MeshEntityRef::new(&mesh, top).unwrap();
    let flipped = reverse_normals(&mesh, Some(&mesh.groups.faces_of(top).collect::<Vec<_>>()));
    for f in 0..mesh.face_count() {
        let id = FaceId(f as u32);
        let same = mesh.tris[f] == flipped.tris[f];
        assert_eq!(same, mesh.groups.group_of(id) != top, "face {f}");
    }
    assert_eq!(flipped.groups, mesh.groups);
    assert!(r.fingerprint.area > 0.0);
}

#[test]
fn reedit_recomputes_only_dependents_and_unchanged_inputs_hit_the_cache() {
    let mut fx = Fixture::new();
    let (mesh, _) = defective_sphere();
    let a = fx.add(source(&mesh));
    let ra = fx.add(feature(MeshRepairFeature::new(body(a))));
    let xa = fx.add(feature(MeshReverseNormalFeature {
        body: body(a),
        groups: None,
    }));
    // An independent chain.
    let b = fx.add(source(&uv_sphere(6, 8)));
    let xb = fx.add(feature(MeshReverseNormalFeature {
        body: body(b),
        groups: None,
    }));
    fx.recompute();
    assert_eq!(fx.computed(), vec![a, ra, xa, b, xb]);

    // Edit the repair: it and the reverse after it recompute; both sources and the other chain
    // come from the cache.
    let mut r = MeshRepairFeature::new(body(a));
    r.opts.remove_self_intersections = false;
    fx.edit(ra, feature(r));
    fx.recompute();
    assert_eq!(fx.computed(), vec![ra, xa]);
    assert_eq!(fx.cached(), vec![a, b, xb]);

    // Editing back to the earlier definition hits the cache for everything.
    fx.edit(ra, feature(MeshRepairFeature::new(body(a))));
    fx.recompute();
    assert!(fx.computed().is_empty(), "{:?}", fx.computed());
    assert_eq!(fx.cached().len(), 5);
}

#[test]
fn weld_tolerance_is_an_expression_and_a_parameter_edit_recomputes() {
    let mut fx = Fixture::new();
    let (mesh, _) = defective_sphere();
    let src = fx.add(source(&mesh));
    let mut r = MeshRepairFeature::new(body(src));
    r.opts.weld_tolerance = amg_doc::Expr::Inline("2 um".into());
    let rep = fx.add(feature(r.clone()));
    fx.recompute();
    assert_eq!(fx.doc.evaluated().node(rep).unwrap().health, Health::Ok);
    assert_eq!(
        amg_doc::Feature::exprs(&r).len(),
        1,
        "the tolerance joins the cache key"
    );
    r.opts.weld_tolerance = amg_doc::Expr::Inline("-1 mm".into());
    fx.edit(rep, feature(r));
    fx.recompute();
    assert!(matches!(
        fx.doc.evaluated().node(rep).unwrap().health,
        Health::Error(_)
    ));
}

#[test]
fn a_downstream_face_group_reference_resolves_after_an_upstream_repair() {
    let mut fx = Fixture::new();
    let (mesh, top) = defective_sphere();
    let r = MeshEntityRef::new(&mesh, top).unwrap();
    let src = fx.add(source(&mesh));
    let rep = fx.add(feature(MeshRepairFeature::new(body(src))));
    let rev = fx.add(feature(MeshReverseNormalFeature {
        body: body(src),
        groups: Some(vec![r.clone()]),
    }));
    fx.recompute();
    assert_eq!(fx.doc.evaluated().node(rep).unwrap().health, Health::Ok);
    assert_eq!(fx.doc.evaluated().node(rev).unwrap().health, Health::Ok);
    let after = fx.mesh(body(src));
    // The group id survived the repair; the duplicate face is gone.
    let resolved = resolve_group(&after, &r).unwrap();
    assert_eq!(resolved.group, top);
    assert_eq!(resolved.quality, MatchQuality::Exact);
    // The reverse flipped exactly the top group of the repaired mesh.
    let repaired_only = {
        let (m, _) =
            amg_mesh::repair(&mesh, &RepairOpts::default(), &amg_jobs::JobCtx::detached()).unwrap();
        m
    };
    let top_faces: Vec<_> = repaired_only.groups.faces_of(top).collect();
    assert!(
        top_faces.len() < mesh.groups.faces_of(top).count(),
        "duplicate removed"
    );
    for f in &top_faces {
        assert_ne!(repaired_only.tris[f.0 as usize], after.tris[f.0 as usize]);
    }
    assert_eq!(
        fx.doc.evaluated().node(rev).unwrap().health,
        Health::Ok,
        "no fingerprint warning needed"
    );
}

#[test]
fn a_reference_falls_back_to_the_fingerprint_when_the_id_is_gone() {
    let (mesh, top) = defective_sphere();
    let r = MeshEntityRef::new(&mesh, top).unwrap();

    // Same geometry, groups renumbered (as an import / boolean might).
    let mut other = IndexedMesh::new(mesh.positions.clone(), mesh.tris.clone()).unwrap();
    let g = other.groups.alloc(FaceGroupInfo::new(GroupOrigin::User));
    let g2 = other.groups.alloc(FaceGroupInfo::new(GroupOrigin::User));
    for f in 0..other.face_count() {
        let id = FaceId(f as u32);
        other.groups.set(
            id,
            if mesh.groups.group_of(id) == top {
                g2
            } else {
                g
            },
        );
    }
    // Drop the old ids from the table by making a mesh that never had them.
    let gone = MeshEntityRef {
        group: FaceGroupId(99),
        fingerprint: r.fingerprint.clone(),
    };
    let res = resolve_group(&other, &gone).unwrap();
    assert_eq!(res.group, g2);
    assert_eq!(res.quality, MatchQuality::FingerprintOnly);

    // A reference whose shape matches nothing fails.
    let nowhere = MeshEntityRef {
        group: FaceGroupId(99),
        fingerprint: amg_mesh::GroupFingerprint {
            centroid: Point3::new(500.0, 0.0, 0.0),
            ..r.fingerprint.clone()
        },
    };
    assert!(matches!(
        resolve_group(&other, &nowhere),
        Err(MeshFeatureError::UnresolvedGroup(FaceGroupId(99)))
    ));
}

#[test]
fn a_feature_with_an_unresolvable_group_fails_with_a_reason() {
    let mut fx = Fixture::new();
    let (mesh, top) = defective_sphere();
    let mut r = MeshEntityRef::new(&mesh, top).unwrap();
    r.group = FaceGroupId(77);
    r.fingerprint.centroid = Point3::new(900.0, 0.0, 0.0);
    let src = fx.add(source(&mesh));
    let rev = fx.add(feature(MeshReverseNormalFeature {
        body: body(src),
        groups: Some(vec![r]),
    }));
    fx.recompute();
    assert!(matches!(
        fx.doc.evaluated().node(rev).unwrap().health,
        Health::Error(_)
    ));
}

#[test]
fn feature_json_round_trips() {
    link();
    let (mesh, top) = defective_sphere();
    let features = [
        feature(MeshRepairFeature::new(BodyId {
            origin: FeatureId(3),
            slot: 0,
        })),
        feature(MeshReverseNormalFeature {
            body: BodyId {
                origin: FeatureId(3),
                slot: 0,
            },
            groups: Some(vec![MeshEntityRef::new(&mesh, top).unwrap()]),
        }),
    ];
    for f in features {
        let s = serde_json::to_string(&f).unwrap();
        let back: FeatureBox = serde_json::from_str(&s).unwrap();
        assert!(!back.is_unknown());
        assert_eq!(back, f);
        assert_eq!(serde_json::to_string(&back).unwrap(), s);
    }
}

struct TempFile(PathBuf);

impl TempFile {
    fn new() -> TempFile {
        static N: AtomicU32 = AtomicU32::new(0);
        TempFile(std::env::temp_dir().join(format!(
            "amg-features-mesh16-{}-{}.amg",
            std::process::id(),
            N.fetch_add(1, Ordering::Relaxed)
        )))
    }
}

impl Drop for TempFile {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.0);
        let _ = std::fs::remove_file(self.0.with_extension("amg.bak"));
    }
}

fn bits(m: &IndexedMesh) -> Vec<u8> {
    let mut v = Vec::new();
    MeshBody::new(m.clone()).write_binary(&mut v).unwrap();
    v
}

#[test]
fn baked_mesh_body_survives_save_and_load_bit_exact() {
    let mut fx = Fixture::new();
    let (mesh, top) = defective_sphere();
    let src = fx.add(source(&mesh));
    fx.add(feature(MeshRepairFeature::new(body(src))));
    fx.recompute();
    let repaired = fx.mesh(body(src));

    // Bake: History -> Direct.
    fx.doc
        .apply(Command::SetDesignMode {
            mode: DesignMode::Direct,
        })
        .unwrap();
    let f = TempFile::new();
    save(&f.0, &fx.doc.snapshot(), &SaveOptions::default()).unwrap();
    let loaded = load(&f.0).unwrap();
    let (_, baked) = loaded.design.baked_bodies().next().unwrap();
    let Some(BodyGeom::Mesh(back)) = &baked.geom else {
        panic!("mesh geometry was not read back");
    };
    assert_eq!(**back, *repaired, "equal");
    assert_eq!(back.content_hash(), repaired.content_hash());
    assert_eq!(bits(back), bits(&repaired), "bit-exact");
    assert!(back.groups.faces_of(top).next().is_some(), "group ids kept");

    // Saving the loaded design again gives the identical archive.
    let g = TempFile::new();
    save(
        &g.0,
        &Arc::new(loaded.design.clone()),
        &SaveOptions::default(),
    )
    .unwrap();
    assert_eq!(std::fs::read(&f.0).unwrap(), std::fs::read(&g.0).unwrap());

    // The loaded document evaluates to the same mesh.
    let mut doc = loaded.into_document();
    doc.recompute_blocking(&fx.jobs).unwrap();
    let (_, eb) = doc.evaluated().bodies().next().unwrap();
    assert!(matches!(&eb.geom, BodyGeom::Mesh(m) if m.content_hash() == repaired.content_hash()));
}

#[test]
fn a_history_document_with_mesh_features_round_trips_with_its_cache() {
    let mut fx = Fixture::new();
    let (mesh, _) = defective_sphere();
    let src = fx.add(source(&mesh));
    let rep = fx.add(feature(MeshRepairFeature::new(body(src))));
    fx.recompute();
    let h = fx.mesh(body(src)).content_hash();
    let f = TempFile::new();
    let opts = SaveOptions {
        cache: Some(fx.doc.cache_export()),
        ..SaveOptions::default()
    };
    save(&f.0, &fx.doc.snapshot(), &opts).unwrap();
    let mut doc = load(&f.0).unwrap().into_document();
    doc.recompute_blocking(&fx.jobs).unwrap();
    assert_eq!(
        doc.last_run().unwrap().computed,
        Vec::<FeatureId>::new(),
        "everything served from the persisted cache"
    );
    let (_, b) = doc.evaluated().bodies().next().unwrap();
    assert!(matches!(&b.geom, BodyGeom::Mesh(m) if m.content_hash() == h));
    assert_eq!(doc.evaluated().node(rep).unwrap().health, Health::Ok);
    let _ = Design::new("unused");
}

// ----------------------------------------------------------------- MESH-12 face groups

/// Closed box `[0, n]^3`, each side an `n x n` lattice of unit cells, outward triangles, one
/// group.
fn lattice_box(n: u32) -> IndexedMesh {
    let mut ids = std::collections::HashMap::new();
    let mut positions = Vec::new();
    let mut tris = Vec::new();
    for axis in 0..3usize {
        let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
        for side in [0, n] {
            for i in 0..n {
                for j in 0..n {
                    let mut quad = [(0, 0), (1, 0), (1, 1), (0, 1)].map(|(di, dj)| {
                        let mut p = [0u32; 3];
                        p[axis] = side;
                        p[u] = i + di;
                        p[v] = j + dj;
                        *ids.entry(p).or_insert_with(|| {
                            positions.push(Point3::new(
                                f64::from(p[0]),
                                f64::from(p[1]),
                                f64::from(p[2]),
                            ));
                            (positions.len() - 1) as u32
                        })
                    });
                    if side == 0 {
                        quad.reverse();
                    }
                    tris.push([quad[0], quad[1], quad[2]]);
                    tris.push([quad[0], quad[2], quad[3]]);
                }
            }
        }
    }
    IndexedMesh::new(positions, tris).unwrap()
}

fn used_groups(m: &IndexedMesh) -> std::collections::BTreeSet<FaceGroupId> {
    m.groups.per_face().iter().copied().collect()
}

#[test]
fn face_groups_run_as_a_feature_with_undo_redo_and_caching() {
    let mut fx = Fixture::new();
    let src = fx.add(source(&lattice_box(6)));
    fx.recompute();
    let h0 = fx.mesh(body(src)).content_hash();
    assert_eq!(used_groups(&fx.mesh(body(src))).len(), 1);

    let seg = fx.add(feature(MeshFaceGroupsFeature::new(body(src))));
    let rev = fx.add(feature(MeshReverseNormalFeature {
        body: body(src),
        groups: None,
    }));
    fx.recompute();
    assert_eq!(fx.doc.evaluated().node(seg).unwrap().health, Health::Ok);
    let segmented = fx.mesh(body(src));
    let h1 = segmented.content_hash();
    assert_ne!(h1, h0);
    // A box has six sides, 72 triangles each.
    let groups = used_groups(&segmented);
    assert_eq!(groups.len(), 6);
    for g in &groups {
        assert_eq!(segmented.groups.faces_of(*g).count(), 72);
    }

    assert!(fx.doc.undo());
    assert!(fx.doc.undo());
    fx.recompute();
    assert_eq!(fx.mesh(body(src)).content_hash(), h0, "undo: exact hash");
    assert!(fx.doc.redo());
    assert!(fx.doc.redo());
    fx.recompute();
    assert_eq!(fx.mesh(body(src)).content_hash(), h1, "redo: exact hash");

    // Editing a parameter recomputes the feature and the reverse after it only; the source
    // comes from the cache. Editing back hits the cache.
    let mut f = MeshFaceGroupsFeature::new(body(src));
    f.opts.angle_threshold = amg_doc::Expr::Inline("20 deg".into());
    fx.edit(seg, feature(f));
    fx.recompute();
    assert_eq!(fx.computed(), vec![seg, rev]);
    assert_eq!(fx.cached(), vec![src]);
    fx.edit(seg, feature(MeshFaceGroupsFeature::new(body(src))));
    fx.recompute();
    assert!(fx.computed().is_empty(), "{:?}", fx.computed());
}

#[test]
fn a_downstream_reference_to_a_segmented_group_survives_a_parameter_edit() {
    let mut fx = Fixture::new();
    let src = fx.add(source(&lattice_box(6)));
    let seg = fx.add(feature(MeshFaceGroupsFeature::new(body(src))));
    fx.recompute();
    let segmented = fx.mesh(body(src));
    let g = segmented.groups.group_of(FaceId(0));
    let r = MeshEntityRef::new(&segmented, g).unwrap();
    let rev = fx.add(feature(MeshReverseNormalFeature {
        body: body(src),
        groups: Some(vec![r.clone()]),
    }));
    fx.recompute();
    assert_eq!(fx.doc.evaluated().node(rev).unwrap().health, Health::Ok);
    let flipped = fx.mesh(body(src));
    let n = flipped
        .tris
        .iter()
        .zip(&segmented.tris)
        .filter(|(a, b)| a != b)
        .count();
    assert_eq!(n, 72, "only that side flipped");

    // Same grouping after a parameter change: the ids are the same, so no fingerprint fallback.
    let mut f = MeshFaceGroupsFeature::new(body(src));
    f.opts.min_group_faces = 4;
    fx.edit(seg, feature(f));
    fx.recompute();
    let health = &fx.doc.evaluated().node(rev).unwrap().health;
    assert_eq!(*health, Health::Ok, "{health:?}");
    assert_eq!(
        resolve_group(&fx.mesh(body(src)), &r).unwrap().quality,
        MatchQuality::Exact
    );
}

#[test]
fn replacing_existing_groups_warns_and_new_body_mode_keeps_the_input() {
    let mut fx = Fixture::new();
    let (mesh, _) = defective_sphere();
    let src = fx.add(source(&mesh));
    let mut f = MeshFaceGroupsFeature::new(body(src));
    f.output = OutputMode::NewBody;
    let seg = fx.add(feature(f));
    fx.recompute();
    let node = fx.doc.evaluated().node(seg).unwrap();
    assert!(
        matches!(node.health, Health::Warning(_)),
        "{:?}",
        node.health
    );
    assert_eq!(fx.mesh(body(src)).content_hash(), mesh.content_hash());
    let new = fx.mesh(body(seg));
    assert!(!used_groups(&new).is_empty());
}

#[test]
fn bad_thresholds_fail_the_feature_and_the_json_round_trips() {
    let mut fx = Fixture::new();
    let src = fx.add(source(&lattice_box(3)));
    let mut f = MeshFaceGroupsFeature::new(body(src));
    f.opts.angle_threshold = amg_doc::Expr::Inline("0 deg".into());
    let seg = fx.add(feature(f));
    fx.recompute();
    assert!(matches!(
        fx.doc.evaluated().node(seg).unwrap().health,
        Health::Error(_)
    ));
    let mut f = MeshFaceGroupsFeature::new(body(src));
    f.opts.curvature_threshold = Some(amg_doc::Expr::Inline("-1".into()));
    fx.edit(seg, feature(f));
    fx.recompute();
    assert!(matches!(
        fx.doc.evaluated().node(seg).unwrap().health,
        Health::Error(_)
    ));

    link();
    let mut f = MeshFaceGroupsFeature::new(body(src));
    f.opts.curvature_threshold = Some(amg_doc::Expr::Inline("0.5".into()));
    let f = feature(f);
    let s = serde_json::to_string(&f).unwrap();
    let back: FeatureBox = serde_json::from_str(&s).unwrap();
    assert!(!back.is_unknown());
    assert_eq!(back, f);
}
