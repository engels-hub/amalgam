//! S01 Interface changes from S12 (IO-14): `RecomputeCtx::baked` and
//! `Document::add_feature_baked` (a feature with its baked payloads in one undo entry).

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    missing_docs,
    reason = "tests"
)]

use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};

use amg_core::Point3;
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, BodyProps, Command, CommandError, CreatedId,
    Dependency, DesignMode, Document, Feature, FeatureBox, FeatureError, FeatureId, FeatureOutput,
    Health, RecomputeCtx, SaveOptions, load, save,
};
use amg_jobs::{JobSystem, JobSystemConfig};
use amg_mesh::IndexedMesh;
use serde::{Deserialize, Serialize};

/// Emits its baked payloads `0..slots` unchanged.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct BakedReader {
    slots: u32,
}

impl Feature for BakedReader {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let mut out = FeatureOutput::default();
        for slot in 0..self.slots {
            let geom = ctx.baked(slot)?.clone();
            out.bodies.push(BodyChange {
                id: ctx.new_body_id(slot),
                change: BodyChangeKind::Created { geom, naming: None },
            });
        }
        Ok(out)
    }
}

amg_doc::register_feature!(BakedReader, "test.baked_reader", version = 1);

fn jobs() -> JobSystem {
    JobSystem::new(JobSystemConfig::default()).unwrap()
}

fn tri(x: f64) -> BodyGeom {
    let m = IndexedMesh::new(
        vec![
            Point3::new(x, 0.0, 0.0),
            Point3::new(x + 1.0, 0.0, 0.0),
            Point3::new(x, 1.0, 0.0),
        ],
        vec![[0, 1, 2]],
    )
    .unwrap();
    BodyGeom::Mesh(Arc::new(m))
}

fn add(doc: &Document, slots: u32) -> Command {
    Command::AddFeature {
        at: None,
        feature: FeatureBox::new(BakedReader { slots }),
        context: doc.design().root(),
        name: None,
    }
}

fn named(n: &str) -> BodyProps {
    BodyProps {
        name: Some(n.to_owned()),
        ..BodyProps::default()
    }
}

fn mesh_of(doc: &Document, id: BodyId) -> Arc<IndexedMesh> {
    match &doc.evaluated().body(id).unwrap().geom {
        BodyGeom::Mesh(m) => Arc::clone(m),
        other => panic!("{other:?}"),
    }
}

#[test]
fn feature_with_baked_payloads_is_one_undo_entry() {
    let jobs = jobs();
    let mut doc = Document::new("baked");
    let a = tri(0.0);
    let b = tri(5.0);
    let applied = doc.history().stack().applied();
    let out = doc
        .add_feature_baked(
            "Insert test",
            add(&doc, 2),
            vec![(0, a.clone()), (1, b.clone())],
            vec![(0, named("A")), (1, named("B"))],
        )
        .unwrap();
    let CreatedId::Feature(fid) = out.created[0] else {
        panic!()
    };
    assert_eq!(doc.history().stack().applied(), applied + 1);
    assert_eq!(doc.history().undo_label(), Some("Insert test"));
    let id0 = BodyId {
        origin: fid,
        slot: 0,
    };
    let id1 = BodyId {
        origin: fid,
        slot: 1,
    };
    assert_eq!(doc.design().body_props(id1).name.as_deref(), Some("B"));
    doc.recompute_blocking(&jobs).unwrap();
    assert_eq!(doc.evaluated().node(fid).unwrap().health, Health::Ok);
    assert!(doc.evaluated().body(id0).unwrap().geom.same_as(&a));
    assert!(doc.evaluated().body(id1).unwrap().geom.same_as(&b));

    assert!(doc.undo());
    assert_eq!(doc.design().timeline().len(), 0);
    assert_eq!(doc.design().baked_bodies().count(), 0);
    assert_eq!(doc.design().body_props(id0), BodyProps::default());
    assert!(doc.redo());
    doc.recompute_blocking(&jobs).unwrap();
    assert!(doc.evaluated().body(id0).unwrap().geom.same_as(&a));

    // Deleting the feature drops its payloads.
    doc.apply(Command::DeleteFeatures { ids: vec![fid] })
        .unwrap();
    assert_eq!(doc.design().baked_bodies().count(), 0);
}

#[test]
fn baked_payloads_survive_save_and_load() {
    static N: AtomicU32 = AtomicU32::new(0);
    let path = std::env::temp_dir().join(format!(
        "amg-doc-baked-{}-{}.amg",
        std::process::id(),
        N.fetch_add(1, Ordering::Relaxed)
    ));
    let jobs = jobs();
    let mut doc = Document::new("baked");
    let out = doc
        .add_feature_baked("Insert", add(&doc, 1), vec![(0, tri(2.5))], Vec::new())
        .unwrap();
    let CreatedId::Feature(fid) = out.created[0] else {
        panic!()
    };
    doc.recompute_blocking(&jobs).unwrap();
    save(&path, &doc.snapshot(), &SaveOptions::default()).unwrap();
    let mut back = load(&path).unwrap().into_document();
    back.recompute_blocking(&jobs).unwrap();
    let id = BodyId {
        origin: fid,
        slot: 0,
    };
    assert_eq!(mesh_of(&doc, id).positions, mesh_of(&back, id).positions);
    let _ = std::fs::remove_file(&path);
    let _ = std::fs::remove_file(path.with_extension("amg.bak"));
}

#[test]
fn missing_payload_is_a_missing_input() {
    let jobs = jobs();
    let mut doc = Document::new("baked");
    // Two slots read, one baked.
    let out = doc
        .add_feature_baked("Insert", add(&doc, 2), vec![(0, tri(0.0))], Vec::new())
        .unwrap();
    let CreatedId::Feature(fid) = out.created[0] else {
        panic!()
    };
    doc.recompute_blocking(&jobs).unwrap();
    assert!(matches!(
        doc.evaluated().node(fid).unwrap().health,
        Health::Error(_)
    ));
}

#[test]
fn bad_requests_change_nothing() {
    let mut doc = Document::new("baked");
    let applied = doc.history().stack().applied();
    let not_add = Command::RenameFeature {
        id: FeatureId(1),
        name: None,
    };
    assert!(matches!(
        doc.add_feature_baked("x", not_add, Vec::new(), Vec::new()),
        Err(CommandError::Invalid(_))
    ));
    let twice = vec![(0, tri(0.0)), (0, tri(1.0))];
    assert!(matches!(
        doc.add_feature_baked("x", add(&doc, 1), twice, Vec::new()),
        Err(CommandError::Invalid(_))
    ));
    assert_eq!(doc.history().stack().applied(), applied);
    assert_eq!(doc.design().timeline().len(), 0);

    doc.recompute_blocking(&jobs()).unwrap();
    doc.apply(Command::SetDesignMode {
        mode: DesignMode::Direct,
    })
    .unwrap_or_else(|e| panic!("{e}"));
    assert!(matches!(
        doc.add_feature_baked("x", add(&doc, 1), vec![(0, tri(0.0))], Vec::new()),
        Err(CommandError::WrongMode(_))
    ));
}
