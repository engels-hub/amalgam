//! MESH-27: the `mesh.combine` feature in a real `Document`.
//!
//! - combine of two corpus bodies: undoable, parametric (a parameter edit upstream of a tool
//!   recomputes the combine and its dependents only), serialises, saves and reloads;
//! - a non-manifold tool is repaired first, with a health warning;
//! - an implicit operand (target or tool) stays implicit: nothing is meshed, the feature fails
//!   with the reason and every body is left as it was; no "converted to mesh" warning;
//! - a solid tool into a mesh target is converted with that warning (needs OCCT: feature
//!   `occt`);
//! - golden volumes of combine results (`tests/golden/mesh/feature_combine_*.toml`).
//!
//! The Manifold backend is the `manifold` feature (off by default, it needs the vcpkg-built
//! library). Without it the boolean tests check that the feature fails cleanly with
//! "boolean backend unavailable" instead. Corpus files are git-LFS: un-fetched pointers are
//! skipped unless `AMG_REQUIRE_TESTDATA=1`.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "tests"
)]

use std::sync::{Arc, OnceLock};

use amg_core::{Dimension, Point3, Transform, Vec3};
use amg_doc::{
    BodyGeom, BodyId, Command, CreatedId, Dependency, Document, Expr, Feature, FeatureBox,
    FeatureError, FeatureId, FeatureOutput, Health, RecomputeCtx,
};
use amg_features::mesh::{MeshCombineFeature, OutputMode};
use amg_jobs::{JobSystem, JobSystemConfig};
use amg_mesh::{BoolOp, IndexedMesh};
use amg_testkit::shapes;
use serde::{Deserialize, Serialize};

// ----------------------------------------------------------------------------- fixtures

/// Test-only body source: a mesh, a data-less implicit body or a placeholder solid in slot 0.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
enum CombineSource {
    Mesh(IndexedMesh),
    Implicit,
    Solid,
}

impl Feature for CombineSource {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let geom = match self {
            CombineSource::Mesh(m) => BodyGeom::Mesh(Arc::new(m.clone())),
            CombineSource::Implicit => {
                BodyGeom::Implicit(Arc::new(amg_doc::stub::SdfGraph::default()))
            }
            CombineSource::Solid => BodyGeom::Solid(amg_doc::stub::Shape::new()),
        };
        Ok(FeatureOutput {
            bodies: vec![amg_doc::BodyChange {
                id: ctx.new_body_id(0),
                change: amg_doc::BodyChangeKind::Created { geom, naming: None },
            }],
            ..FeatureOutput::default()
        })
    }
}

amg_doc::register_feature!(CombineSource, "test.mesh_combine_source", version = 1);

/// Test-only move of a mesh body by `base + (dx, 0, 0)`; `dx` is a length parameter.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct CombineMove {
    body: BodyId,
    base: [f64; 3],
    dx: Expr,
}

impl Feature for CombineMove {
    fn inputs(&self) -> Vec<Dependency> {
        vec![Dependency::Body(self.body)]
    }

    fn exprs(&self) -> Vec<&Expr> {
        vec![&self.dx]
    }

    fn exprs_mut(&mut self) -> Vec<(&mut Expr, Dimension)> {
        vec![(&mut self.dx, Dimension::LENGTH)]
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let dx = ctx.eval_length(&self.dx)?;
        let BodyGeom::Mesh(m) = ctx.body(self.body)? else {
            return Err(FeatureError::Kernel("not a mesh".into()));
        };
        let mut m = (**m).clone();
        m.transform(&Transform::from_translation(
            Vec3::from_array(self.base) + Vec3::new(dx, 0.0, 0.0),
        ));
        Ok(FeatureOutput {
            bodies: vec![amg_doc::BodyChange {
                id: self.body,
                change: amg_doc::BodyChangeKind::Modified {
                    geom: BodyGeom::Mesh(Arc::new(m)),
                    naming: None,
                },
            }],
            ..FeatureOutput::default()
        })
    }
}

amg_doc::register_feature!(CombineMove, "test.mesh_combine_move", version = 1);

fn jobs() -> &'static JobSystem {
    static JOBS: OnceLock<JobSystem> = OnceLock::new();
    JOBS.get_or_init(|| JobSystem::new(JobSystemConfig::default()).unwrap())
}

fn link() {
    amg_features::mesh::link();
}

fn body(origin: FeatureId) -> BodyId {
    BodyId { origin, slot: 0 }
}

fn add_feature(doc: &mut Document, f: FeatureBox) -> FeatureId {
    let root = doc.design().root();
    let out = doc
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

fn source(m: &IndexedMesh) -> FeatureBox {
    FeatureBox::new(CombineSource::Mesh(m.clone()))
}

fn combine(target: BodyId, tools: &[BodyId], op: BoolOp) -> FeatureBox {
    FeatureBox::new(MeshCombineFeature::new(target, tools.to_vec(), op))
}

fn recompute(doc: &mut Document) {
    doc.recompute_blocking(jobs()).unwrap();
}

fn try_mesh(doc: &Document, id: BodyId) -> Option<Arc<IndexedMesh>> {
    match &doc.evaluated().body(id)?.geom {
        BodyGeom::Mesh(m) => Some(Arc::clone(m)),
        _ => None,
    }
}

fn mesh(doc: &Document, id: BodyId) -> Arc<IndexedMesh> {
    try_mesh(doc, id).unwrap_or_else(|| panic!("{id:?} is a mesh body"))
}

fn health(doc: &Document, id: FeatureId) -> Health {
    doc.evaluated().node(id).unwrap().health.clone()
}

fn messages(h: &Health) -> Vec<String> {
    match h {
        Health::Warning(d) => d.iter().map(|d| d.message.clone()).collect(),
        Health::Error(d) => vec![d.message.clone()],
        _ => Vec::new(),
    }
}

fn icosphere(center: Point3, radius: f64, sub: u32) -> IndexedMesh {
    let s = shapes::icosphere(center, radius, sub);
    IndexedMesh::new(s.positions, s.triangles).unwrap()
}

fn cube(min: Point3, size: f64) -> IndexedMesh {
    let s = shapes::cube(min, size);
    IndexedMesh::new(s.positions, s.triangles).unwrap()
}

// ----------------------------------------------------------------------------- registration

#[test]
fn combine_is_registered_and_serialises() {
    link();
    let reg = amg_doc::try_registry().unwrap();
    assert_eq!(reg.get("mesh.combine").unwrap().version, 1);
    let f = FeatureBox::new(MeshCombineFeature {
        keep_tools: true,
        output: OutputMode::NewBody,
        ..MeshCombineFeature::new(
            BodyId {
                origin: FeatureId(3),
                slot: 0,
            },
            vec![BodyId {
                origin: FeatureId(4),
                slot: 1,
            }],
            BoolOp::Intersection,
        )
    });
    let s = serde_json::to_string(&f).unwrap();
    let back: FeatureBox = serde_json::from_str(&s).unwrap();
    assert!(!back.is_unknown());
    assert_eq!(back, f);
}

#[test]
fn bad_tool_lists_are_refused_by_validate() {
    let t = BodyId {
        origin: FeatureId(1),
        slot: 0,
    };
    let u = BodyId {
        origin: FeatureId(2),
        slot: 0,
    };
    for tools in [vec![], vec![t], vec![u, u]] {
        let f = MeshCombineFeature::new(t, tools.clone(), BoolOp::Union);
        assert!(
            matches!(
                f.validate(),
                Err(FeatureError::InvalidValue { field: "tools", .. })
            ),
            "{tools:?}"
        );
    }
    assert!(
        MeshCombineFeature::new(t, vec![u], BoolOp::Union)
            .validate()
            .is_ok()
    );
}

// ----------------------------------------------------------------------------- implicit operands

/// Done-when: a boolean with an implicit operand stays implicit (QUESTIONS #63). The mesh
/// combine never meshes it: the feature fails with the reason, the implicit body is still the
/// same `BodyGeom::Implicit`, the other operand is untouched (and not deleted), and there is no
/// "converted to mesh" warning. Independent of the boolean backend.
#[test]
fn an_implicit_operand_stays_implicit_and_nothing_is_meshed() {
    link();
    for implicit_is_target in [true, false] {
        let mut doc = Document::new("implicit");
        let imp = add_feature(&mut doc, FeatureBox::new(CombineSource::Implicit));
        let m = add_feature(&mut doc, source(&icosphere(Point3::ZERO, 5.0, 2)));
        recompute(&mut doc);
        let imp_before = match &doc.evaluated().body(body(imp)).unwrap().geom {
            BodyGeom::Implicit(g) => Arc::clone(g),
            other => panic!("{other:?}"),
        };
        let mesh_before = mesh(&doc, body(m));
        let (target, tool) = if implicit_is_target {
            (body(imp), body(m))
        } else {
            (body(m), body(imp))
        };
        let c = add_feature(&mut doc, combine(target, &[tool], BoolOp::Difference));
        recompute(&mut doc);
        let h = health(&doc, c);
        let Health::Error(d) = &h else {
            panic!("implicit operand must fail the mesh combine: {h:?}")
        };
        assert!(d.message.contains("stays implicit"), "{}", d.message);
        assert!(d.message.contains("Implicit Combine"), "{}", d.message);
        assert!(!d.message.contains("converted to mesh"), "{}", d.message);
        // Still implicit, the very same graph; no mesh body appeared for it.
        match &doc.evaluated().body(body(imp)).unwrap().geom {
            BodyGeom::Implicit(g) => assert!(Arc::ptr_eq(g, &imp_before)),
            other => panic!("the implicit body was converted: {other:?}"),
        }
        assert!(Arc::ptr_eq(&mesh(&doc, body(m)), &mesh_before));
        assert!(doc.evaluated().body(body(c)).is_none(), "no new body");
        assert_eq!(doc.evaluated().bodies().count(), 2);
    }
}

/// The solid-tool path in a build without a real B-rep (the stub placeholder solid): a clear
/// error, not a silent mesh, and no "converted" warning for a failed conversion.
#[test]
fn a_placeholder_solid_tool_fails_with_a_reason() {
    link();
    let mut doc = Document::new("solid");
    let a = add_feature(&mut doc, source(&cube(Point3::ZERO, 10.0)));
    let s = add_feature(&mut doc, FeatureBox::new(CombineSource::Solid));
    let c = add_feature(&mut doc, combine(body(a), &[body(s)], BoolOp::Union));
    recompute(&mut doc);
    let h = health(&doc, c);
    assert!(
        matches!(&h, Health::Error(d) if d.message.contains("B-rep")),
        "{h:?}"
    );
    assert!(matches!(
        doc.evaluated().body(body(s)).unwrap().geom,
        BodyGeom::Solid(_)
    ));
}

/// A solid target is not a mesh combine (the B-rep combine is S02's), and no conversion
/// warning is given: the warning applies only to a mesh target.
#[test]
fn a_solid_target_is_not_a_mesh_combine() {
    link();
    let mut doc = Document::new("solid target");
    let s = add_feature(&mut doc, FeatureBox::new(CombineSource::Solid));
    let a = add_feature(&mut doc, source(&cube(Point3::ZERO, 10.0)));
    let c = add_feature(&mut doc, combine(body(s), &[body(a)], BoolOp::Union));
    recompute(&mut doc);
    let h = health(&doc, c);
    assert!(
        matches!(&h, Health::Error(d) if d.message.contains("not a mesh")),
        "{h:?}"
    );
    assert!(messages(&h).iter().all(|m| !m.contains("converted")));
    assert!(try_mesh(&doc, body(a)).is_some(), "tool kept");
}

// ----------------------------------------------------------------------------- without backend

/// Default build (no `manifold`): the combine fails cleanly and leaves the bodies alone.
#[cfg(not(feature = "manifold"))]
#[test]
fn without_the_backend_combine_fails_cleanly() {
    link();
    let mut doc = Document::new("no backend");
    let a = add_feature(&mut doc, source(&cube(Point3::ZERO, 10.0)));
    let b = add_feature(&mut doc, source(&cube(Point3::splat(5.0), 10.0)));
    recompute(&mut doc);
    let (ha, hb) = (
        mesh(&doc, body(a)).content_hash(),
        mesh(&doc, body(b)).content_hash(),
    );
    let c = add_feature(&mut doc, combine(body(a), &[body(b)], BoolOp::Union));
    recompute(&mut doc);
    let h = health(&doc, c);
    assert!(
        matches!(&h, Health::Error(d) if d.message.contains("boolean backend unavailable")),
        "{h:?}"
    );
    assert_eq!(mesh(&doc, body(a)).content_hash(), ha);
    assert_eq!(mesh(&doc, body(b)).content_hash(), hb, "tool not deleted");
}

// ----------------------------------------------------------------------------- with backend

#[cfg(feature = "manifold")]
mod with_backend {
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicU32, Ordering};

    use amg_doc::{ExprSrc, ParamId, SaveOptions, load, save};
    use amg_features::mesh::MeshReverseNormalFeature;
    use amg_jobs::JobCtx;
    use amg_mesh::corpus::{FileStatus, Manifest, corpus_root, quick_stats, read_mesh};
    use amg_mesh::{BoolOpts, boolean_n};
    use amg_testkit::TriSoup;

    use super::*;

    /// An icosphere with one face duplicated: its three edges have three faces (non-manifold).
    fn non_manifold_sphere(center: Point3) -> IndexedMesh {
        let mut m = icosphere(center, 6.0, 2);
        let t = m.tris[0];
        m.tris.push(t);
        m.groups = amg_mesh::FaceGroups::new(m.tris.len());
        m
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

    fn watertight(m: &IndexedMesh) -> bool {
        amg_mesh::analyze(m, &Default::default(), &JobCtx::detached())
            .unwrap()
            .is_watertight()
    }

    fn computed(doc: &Document) -> Vec<FeatureId> {
        doc.last_run().unwrap().computed.clone()
    }

    fn cached(doc: &Document) -> Vec<FeatureId> {
        doc.last_run().unwrap().cached.clone()
    }

    fn first_param(doc: &Document, id: FeatureId) -> ParamId {
        let (_, node) = doc.design().timeline().node(id).unwrap();
        match node.feature.get().exprs().first() {
            Some(Expr::Param(p)) => *p,
            other => panic!("inline expressions should have become parameters: {other:?}"),
        }
    }

    struct TempFile(PathBuf);

    impl TempFile {
        fn new() -> TempFile {
            static N: AtomicU32 = AtomicU32::new(0);
            TempFile(std::env::temp_dir().join(format!(
                "amg-features-mesh27-{}-{}.amg",
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

    /// Two clean corpus models (committed, no recorded defect, 200 to 3000 faces, closed,
    /// manifold, consistently oriented, one component), smallest ids first; `None` if the
    /// files are un-fetched LFS pointers (an error with `AMG_REQUIRE_TESTDATA=1`).
    fn two_corpus_bodies() -> Option<[(String, IndexedMesh); 2]> {
        let manifest = Manifest::load().expect("corpus.toml");
        let root = corpus_root();
        let mut entries: Vec<_> = manifest
            .committed()
            .filter(|e| e.defects.is_empty() && (200..=3000).contains(&e.faces))
            .collect();
        entries.sort_by(|a, b| a.id.cmp(&b.id));
        let mut found = Vec::new();
        for e in entries {
            let path = match e.locate(&root) {
                FileStatus::Ready(p) => p,
                FileStatus::LfsPointer(p) => {
                    assert!(
                        !require_testdata(),
                        "{} is an un-fetched git-LFS pointer and AMG_REQUIRE_TESTDATA=1",
                        p.display()
                    );
                    continue;
                }
                FileStatus::NotFetched(_) | FileStatus::Missing(_) => continue,
            };
            let Ok(m) = read_mesh(&path) else { continue };
            let s = quick_stats(&m);
            if s.boundary_edges == 0
                && s.non_manifold_edges == 0
                && s.non_manifold_vertices == 0
                && s.inconsistent_edges == 0
                && s.degenerate_faces == 0
                && s.duplicate_faces == 0
                && s.components == 1
                && s.signed_volume > 0.0
                && watertight(&m)
            {
                found.push((e.id.clone(), m));
                if found.len() == 2 {
                    let b = found.pop().unwrap();
                    let a = found.pop().unwrap();
                    return Some([a, b]);
                }
            }
        }
        assert!(!require_testdata(), "fewer than two clean corpus models");
        eprintln!("SKIP: fewer than two clean corpus models available (git lfs pull)");
        None
    }

    fn centre(m: &IndexedMesh) -> Vec3 {
        let b = m.aabb();
        (b.min + b.max) * 0.5
    }

    /// The direct MESH-05 boolean of the same operands (the reference result).
    fn direct(op: BoolOp, a: &IndexedMesh, b: &IndexedMesh) -> IndexedMesh {
        boolean_n(
            op,
            &[a, b],
            &BoolOpts {
                repair_inputs: true,
            },
            &JobCtx::detached(),
        )
        .unwrap()
        .0
    }

    /// Done-when: a combine of two corpus bodies is undoable and parametric.
    #[test]
    fn corpus_combine_is_undoable_and_parametric() {
        link();
        let Some([(ida, ma), (idb, mb)]) = two_corpus_bodies() else {
            return;
        };
        eprintln!(
            "corpus bodies {ida} ({} faces) and {idb} ({} faces)",
            ma.face_count(),
            mb.face_count()
        );
        // Tool centred on the target's centre, then moved by `dx` (a parameter) along x by
        // a quarter of the target's size.
        let size = ma.aabb().size().length();
        let base = centre(&ma) - centre(&mb);
        let dx0 = 0.25 * size;
        let mut doc = Document::new("corpus combine");
        let a = add_feature(&mut doc, source(&ma));
        let b = add_feature(&mut doc, source(&mb));
        let mv = add_feature(
            &mut doc,
            FeatureBox::new(CombineMove {
                body: body(b),
                base: base.to_array(),
                dx: Expr::Inline(ExprSrc(format!("{dx0} mm"))),
            }),
        );
        recompute(&mut doc);
        let h0 = mesh(&doc, body(a)).content_hash();
        let tool0 = mesh(&doc, body(b));

        // Combine: health Ok, the result is the direct boolean, the tool is gone.
        let c = add_feature(&mut doc, combine(body(a), &[body(b)], BoolOp::Union));
        recompute(&mut doc);
        assert_eq!(health(&doc, c), Health::Ok);
        let out1 = mesh(&doc, body(a));
        let reference = direct(BoolOp::Union, &ma, &tool0);
        assert_eq!(out1.content_hash(), reference.content_hash());
        assert!(watertight(&out1));
        let (va, vb, vu) = (
            ma.signed_volume(),
            tool0.signed_volume(),
            out1.signed_volume(),
        );
        assert!(vu >= va.max(vb) * (1.0 - 1e-9) && vu <= (va + vb) * (1.0 + 1e-9));
        assert!(
            doc.evaluated().body(body(b)).is_none(),
            "tool body deleted (keep tools off)"
        );
        let h1 = out1.content_hash();
        assert_ne!(h1, h0);

        // Undo / redo of the add.
        assert!(doc.undo());
        recompute(&mut doc);
        assert_eq!(mesh(&doc, body(a)).content_hash(), h0, "undo");
        assert_eq!(
            mesh(&doc, body(b)).content_hash(),
            tool0.content_hash(),
            "undo brings the tool back"
        );
        assert!(doc.redo());
        recompute(&mut doc);
        assert_eq!(mesh(&doc, body(a)).content_hash(), h1, "redo");
        assert!(doc.evaluated().body(body(b)).is_none());

        // A dependent of the combine and an independent chain.
        let rev = add_feature(
            &mut doc,
            FeatureBox::new(MeshReverseNormalFeature {
                body: body(a),
                groups: None,
            }),
        );
        let x = add_feature(&mut doc, source(&icosphere(Point3::ZERO, 3.0, 2)));
        let rev_x = add_feature(
            &mut doc,
            FeatureBox::new(MeshReverseNormalFeature {
                body: body(x),
                groups: None,
            }),
        );
        recompute(&mut doc);
        assert_eq!(computed(&doc), vec![rev, x, rev_x]);
        let after_rev = mesh(&doc, body(a)).content_hash();

        // Parametric: moving the tool (its parameter) recomputes the move, the combine and
        // its dependent only.
        let p = first_param(&doc, mv);
        doc.apply(Command::SetParamExpr {
            id: p,
            expr: format!("{} mm", 0.4 * size),
        })
        .unwrap();
        recompute(&mut doc);
        assert_eq!(computed(&doc), vec![mv, c, rev], "dependents only");
        assert_eq!(cached(&doc), vec![a, b, x, rev_x], "the rest is cached");
        assert_eq!(health(&doc, c), Health::Ok);
        let h2 = mesh(&doc, body(a)).content_hash();
        assert_ne!(h2, after_rev, "the edit changed the result");
        assert!(doc.undo());
        recompute(&mut doc);
        assert_eq!(mesh(&doc, body(a)).content_hash(), after_rev);
        assert!(doc.redo());
        recompute(&mut doc);
        assert_eq!(mesh(&doc, body(a)).content_hash(), h2);

        // Editing the operation (cut) and keeping the tools.
        doc.apply(Command::EditFeature {
            id: c,
            feature: FeatureBox::new(MeshCombineFeature {
                keep_tools: true,
                ..MeshCombineFeature::new(body(a), vec![body(b)], BoolOp::Difference)
            }),
        })
        .unwrap();
        recompute(&mut doc);
        assert_eq!(health(&doc, c), Health::Ok);
        let cut = mesh(&doc, body(a));
        assert!(watertight(&cut));
        assert!(cut.signed_volume() < va, "a cut removes material");
        assert!(try_mesh(&doc, body(b)).is_some(), "keep tools");

        // JSON round-trip and save / load to identical meshes.
        let (_, node) = doc.design().timeline().node(c).unwrap();
        let f = node.feature.clone();
        let s = serde_json::to_string(&f).unwrap();
        let back: FeatureBox = serde_json::from_str(&s).unwrap();
        assert_eq!(back, f);
        let all: Vec<(BodyId, u64)> = doc
            .evaluated()
            .bodies()
            .filter_map(|(id, b)| match &b.geom {
                BodyGeom::Mesh(m) => Some((id, m.content_hash())),
                _ => None,
            })
            .collect();
        let file = TempFile::new();
        save(&file.0, &doc.snapshot(), &SaveOptions::default()).unwrap();
        let mut reopened = load(&file.0).unwrap().into_document();
        reopened.recompute_blocking(jobs()).unwrap();
        for (id, h) in &all {
            assert_eq!(mesh(&reopened, *id).content_hash(), *h, "{id:?}");
        }
        assert_eq!(health(&reopened, c), Health::Ok);
    }

    /// Done-when: a non-manifold tool triggers auto-repair with a warning.
    #[test]
    fn a_non_manifold_tool_is_repaired_with_a_warning() {
        link();
        let tool = non_manifold_sphere(Point3::new(5.0, 5.0, 5.0));
        let stats = quick_stats(&tool);
        assert!(stats.non_manifold_edges > 0, "{stats:?}");
        let mut doc = Document::new("repair");
        let a = add_feature(&mut doc, source(&cube(Point3::ZERO, 10.0)));
        let b = add_feature(&mut doc, source(&tool));
        let c = add_feature(&mut doc, combine(body(a), &[body(b)], BoolOp::Union));
        recompute(&mut doc);
        let h = health(&doc, c);
        let msgs = messages(&h);
        assert!(matches!(h, Health::Warning(_)), "{h:?}");
        assert!(
            msgs.iter()
                .any(|m| m.contains("tool body 1") && m.contains("repaired")),
            "{msgs:?}"
        );
        assert!(msgs.iter().all(|m| !m.contains("converted")), "{msgs:?}");
        let out = mesh(&doc, body(a));
        assert!(watertight(&out));
        assert!(out.signed_volume() > 1000.0);
    }

    /// Several tools, keep tools, new body.
    #[test]
    fn several_tools_new_body_and_keep_tools() {
        link();
        let mut doc = Document::new("tools");
        let a = add_feature(&mut doc, source(&cube(Point3::ZERO, 10.0)));
        let b = add_feature(&mut doc, source(&cube(Point3::new(5.0, 0.0, 0.0), 10.0)));
        let d = add_feature(&mut doc, source(&cube(Point3::new(0.0, 5.0, 0.0), 10.0)));
        recompute(&mut doc);
        let ha = mesh(&doc, body(a)).content_hash();
        let c = add_feature(
            &mut doc,
            FeatureBox::new(MeshCombineFeature {
                keep_tools: true,
                output: OutputMode::NewBody,
                ..MeshCombineFeature::new(body(a), vec![body(b), body(d)], BoolOp::Union)
            }),
        );
        recompute(&mut doc);
        assert_eq!(health(&doc, c), Health::Ok);
        // Union of three 10 mm cubes, the tools offset by 5 mm along x and along y: a, plus
        // 500 of b and 500 of d outside a (b and d overlap only inside a) = 2000 mm³.
        let out = mesh(&doc, body(c));
        assert!(
            (out.signed_volume() - 2000.0).abs() < 1e-9,
            "{}",
            out.signed_volume()
        );
        assert_eq!(
            mesh(&doc, body(a)).content_hash(),
            ha,
            "target kept (new body)"
        );
        assert!(try_mesh(&doc, body(b)).is_some() && try_mesh(&doc, body(d)).is_some());

        // Disjoint intersection: an empty result is a warning, not an error.
        let e = add_feature(&mut doc, source(&cube(Point3::splat(50.0), 1.0)));
        let i = add_feature(&mut doc, combine(body(b), &[body(e)], BoolOp::Intersection));
        recompute(&mut doc);
        let msgs = messages(&health(&doc, i));
        assert!(msgs.iter().any(|m| m.contains("empty")), "{msgs:?}");
    }

    /// Done-when: golden volume test (volume, area, bbox, counts of the combine results).
    #[test]
    fn golden_combine_volumes() {
        link();
        let cases: [(&str, BoolOp, IndexedMesh, IndexedMesh, Option<f64>); 3] = [
            (
                "feature_combine_cubes_union",
                BoolOp::Union,
                cube(Point3::ZERO, 10.0),
                cube(Point3::new(5.0, 0.0, 0.0), 10.0),
                Some(1500.0),
            ),
            (
                "feature_combine_box_minus_sphere",
                BoolOp::Difference,
                cube(Point3::ZERO, 20.0),
                icosphere(Point3::new(20.0, 20.0, 20.0), 10.0, 4),
                None,
            ),
            (
                "feature_combine_sphere_intersection",
                BoolOp::Intersection,
                icosphere(Point3::ZERO, 10.0, 4),
                icosphere(Point3::new(8.0, 0.0, 0.0), 10.0, 4),
                None,
            ),
        ];
        for (name, op, ma, mb, exact) in cases {
            let mut doc = Document::new(name);
            let a = add_feature(&mut doc, source(&ma));
            let b = add_feature(&mut doc, source(&mb));
            let c = add_feature(&mut doc, combine(body(a), &[body(b)], op));
            recompute(&mut doc);
            assert_eq!(health(&doc, c), Health::Ok, "{name}");
            let out = mesh(&doc, body(a));
            assert!(watertight(&out), "{name}");
            if let Some(v) = exact {
                assert!((out.signed_volume() - v).abs() < 1e-9, "{name}");
            }
            // Volume identity against the direct booleans of the same operands.
            let v = |op| direct(op, &ma, &mb).signed_volume();
            let (va, vb) = (ma.signed_volume(), mb.signed_volume());
            let (vu, vd, vi) = (
                v(BoolOp::Union),
                v(BoolOp::Difference),
                v(BoolOp::Intersection),
            );
            assert!(((vu + vi) - (va + vb)).abs() <= 1e-6 * (va + vb), "{name}");
            assert!(((vd + vi) - va).abs() <= 1e-6 * va, "{name}");
            amg_testkit::assert_golden!("mesh", name, soup(&out));
        }
    }
}

// ----------------------------------------------------------------------------- solid tool (OCCT)

/// A solid tool into a mesh target is tessellated, combined and warned about ("converted to
/// mesh"); the warning names the tool. Needs both native backends.
#[cfg(all(feature = "occt", feature = "manifold"))]
#[test]
fn a_solid_tool_into_a_mesh_target_is_converted_with_a_warning() {
    use amg_brep::BrepKernel;

    /// A real B-rep box tool (S02 `make_box`).
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    struct BoxSolid;

    impl Feature for BoxSolid {
        fn inputs(&self) -> Vec<Dependency> {
            Vec::new()
        }

        fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
            let k = amg_brep::OcctKernel::new(amg_brep::KernelConfig::default());
            let mut frame = amg_core::Plane::XY;
            frame.origin = Point3::new(5.0, 0.0, 0.0);
            let r = k
                .make_box(&frame, Vec3::splat(10.0))
                .map_err(|e| FeatureError::Kernel(e.to_string()))?;
            Ok(FeatureOutput {
                bodies: vec![amg_doc::BodyChange {
                    id: ctx.new_body_id(0),
                    change: amg_doc::BodyChangeKind::Created {
                        geom: BodyGeom::Solid(amg_doc::stub::Shape::from_brep(r.shape)),
                        naming: None,
                    },
                }],
                ..FeatureOutput::default()
            })
        }
    }

    amg_doc::register_feature!(BoxSolid, "test.mesh_combine_box_solid", version = 1);

    link();
    let mut doc = Document::new("solid tool");
    let a = add_feature(&mut doc, source(&cube(Point3::ZERO, 10.0)));
    let s = add_feature(&mut doc, FeatureBox::new(BoxSolid));
    let c = add_feature(&mut doc, combine(body(a), &[body(s)], BoolOp::Union));
    recompute(&mut doc);
    let h = health(&doc, c);
    let msgs = messages(&h);
    assert!(
        msgs.iter()
            .any(|m| m.contains("tool body 1") && m.contains("converted to mesh")),
        "{h:?}"
    );
    let out = mesh(&doc, body(a));
    assert!(
        (out.signed_volume() - 1500.0).abs() < 1e-6,
        "{}",
        out.signed_volume()
    );
    assert!(
        doc.evaluated().body(body(s)).is_none(),
        "solid tool consumed"
    );
}
