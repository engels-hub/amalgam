//! DOC-08: direct mode, bake, base feature.
//!
//! Geometry is still the `amg-doc::stub` placeholder, which has no volume, area or bounding
//! box. The "volume/area/bbox within 1e-9" check is therefore the stronger identity check:
//! every body's geometry handle after History -> Direct -> History is the same allocation
//! (`BodyGeom::same_as`) as before, so any measure of it is bit-identical. Topology counts
//! likewise cannot differ.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    missing_docs,
    reason = "tests"
)]

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicU32, Ordering};

use amg_core::Transform;
use amg_doc::testing::{TestBox, TestCounter};
use amg_doc::{
    BASE_FEATURE_KIND, BaseFeature, BodyGeom, BodyId, BodyProps, Command, CommandError,
    ComponentId, Dependency, Design, DesignMode, Document, FeatureBox, FeatureId, Health,
    SaveOptions, load, save,
};
use amg_jobs::{JobSystem, JobSystemConfig};

fn jobs() -> JobSystem {
    JobSystem::new(JobSystemConfig::default()).unwrap()
}

fn body(origin: FeatureId, slot: u32) -> BodyId {
    BodyId { origin, slot }
}

fn boxf(x: f64) -> FeatureBox {
    FeatureBox::new(TestBox {
        size: [x, 2.0, 3.0],
    })
}

fn counter(deps: Vec<Dependency>, value: i64, fail: bool) -> FeatureBox {
    FeatureBox::new(TestCounter { deps, value, fail })
}

fn add_mesh(n: u32) -> FeatureBox {
    FeatureBox::new(amg_doc::testing::TestAddMesh {
        bodies: n,
        label: "m".into(),
    })
}

/// Two solids.
fn solid_design() -> Design {
    amg_doc::testing::link();
    let mut d = Design::new("solid");
    let root = d.root();
    d.add_feature(None, boxf(1.0), root, Some("Box 1".into()))
        .unwrap();
    d.add_feature(None, boxf(5.0), root, None).unwrap();
    d
}

/// Three mesh bodies.
fn mesh_design() -> Design {
    amg_doc::testing::link();
    let mut d = Design::new("mesh");
    let root = d.root();
    d.add_feature(None, add_mesh(3), root, None).unwrap();
    d
}

/// A solid in a sub-component, imported meshes, a feature that modifies the solid and adds a
/// mesh body, a failed-looking-but-suppressed feature, a parameter and body properties.
fn mixed_design() -> Design {
    amg_doc::testing::link();
    let mut d = Design::new("mixed");
    let root = d.root();
    let (arm, _) = d.add_component(root, "Arm", Transform::IDENTITY).unwrap();
    let b = d.add_feature(None, boxf(4.0), arm, None).unwrap();
    d.add_feature(None, add_mesh(2), root, None).unwrap();
    d.add_feature(
        None,
        counter(vec![Dependency::Body(body(b, 0))], 1, false),
        arm,
        None,
    )
    .unwrap();
    let sup = d
        .add_feature(None, counter(vec![], 2, false), root, None)
        .unwrap();
    d.set_suppressed(sup, true).unwrap();
    d.add_param("width", "12 mm", None, "").unwrap();
    d.set_body_props(
        body(b, 0),
        BodyProps {
            name: Some("Plate".into()),
            opacity: 0.5,
            ..BodyProps::default()
        },
    )
    .unwrap();
    d
}

type Bodies = BTreeMap<BodyId, (ComponentId, BodyGeom)>;

fn bodies(doc: &Document) -> Bodies {
    doc.evaluated()
        .bodies()
        .map(|(id, b)| (id, (b.component, b.geom.clone())))
        .collect()
}

fn assert_same(a: &Bodies, b: &Bodies) {
    assert_eq!(
        a.keys().collect::<Vec<_>>(),
        b.keys().collect::<Vec<_>>(),
        "same BodyIds"
    );
    for (id, (ca, ga)) in a {
        let (cb, gb) = &b[id];
        assert_eq!(ca, cb, "component of {id:?}");
        assert!(ga.same_as(gb), "geometry of {id:?} is the same allocation");
    }
}

fn json(d: &Design) -> String {
    serde_json::to_string_pretty(d).unwrap()
}

fn mode(doc: &Document, m: DesignMode) -> Command {
    assert_ne!(doc.design().meta().mode, m);
    Command::SetDesignMode { mode: m }
}

fn props_of(d: &Design) -> Vec<(BodyId, BodyProps)> {
    d.explicit_body_props()
        .map(|(i, p)| (i, p.clone()))
        .collect()
}

fn round_trip(d: Design) {
    let jobs = jobs();
    let mut doc = Document::from_design(d);
    doc.recompute_blocking(&jobs).unwrap();
    let before = bodies(&doc);
    assert!(!before.is_empty());
    let (json0, props0) = (json(doc.design()), props_of(doc.design()));

    // History -> Direct.
    doc.apply(mode(&doc, DesignMode::Direct)).unwrap();
    let json1 = json(doc.design());
    assert_eq!(doc.design().meta().mode, DesignMode::Direct);
    assert_eq!(doc.design().timeline().len(), 0);
    assert_eq!(doc.design().baked_bodies().count(), before.len());
    assert_eq!(props_of(doc.design()), props0, "body props preserved");
    doc.recompute_blocking(&jobs).unwrap();
    assert_same(&before, &bodies(&doc));

    // Direct -> History.
    let out = doc.apply(mode(&doc, DesignMode::History)).unwrap();
    assert_eq!(out.created.len(), 1, "one base feature");
    assert_eq!(doc.design().meta().mode, DesignMode::History);
    assert_eq!(doc.design().timeline().len(), 1);
    let (base_id, base_kind) = {
        let n = doc.design().timeline().nodes().next().unwrap();
        (n.id, n.feature.kind().to_owned())
    };
    assert_eq!(base_kind, BASE_FEATURE_KIND);
    assert_eq!(props_of(doc.design()), props0, "body props preserved");
    doc.recompute_blocking(&jobs).unwrap();
    assert_same(&before, &bodies(&doc));
    assert_eq!(doc.evaluated().node(base_id).unwrap().health, Health::Ok);

    // Undo restores the full timeline, step by step; redo reapplies.
    let json2 = json(doc.design());
    assert!(doc.undo());
    assert_eq!(json(doc.design()), json1);
    assert!(doc.undo());
    assert_eq!(json(doc.design()), json0, "the original timeline is back");
    assert_eq!(doc.design().timeline().len(), before_len(&json0));
    doc.recompute_blocking(&jobs).unwrap();
    assert_same(&before, &bodies(&doc));
    assert!(doc.redo() && doc.redo());
    assert_eq!(json(doc.design()), json2);
}

fn before_len(design_json: &str) -> usize {
    let v: serde_json::Value = serde_json::from_str(design_json).unwrap();
    v["timeline"]["nodes"].as_array().unwrap().len()
}

#[test]
fn solid_design_round_trips() {
    round_trip(solid_design());
}

#[test]
fn mesh_design_round_trips() {
    round_trip(mesh_design());
}

#[test]
fn mixed_design_round_trips() {
    round_trip(mixed_design());
}

#[test]
fn bodies_keep_their_components_and_unsuppressed_state() {
    let jobs = jobs();
    let mut doc = Document::from_design(mixed_design());
    doc.recompute_blocking(&jobs).unwrap();
    let arm = doc
        .design()
        .components()
        .find(|(_, c)| c.name == "Arm")
        .unwrap()
        .0;
    assert!(bodies(&doc).values().any(|(c, _)| *c == arm));
    doc.apply(Command::SetDesignMode {
        mode: DesignMode::Direct,
    })
    .unwrap();
    for (_, b) in doc.design().baked_bodies() {
        assert!(doc.design().component(b.component).is_some());
    }
    assert!(doc.design().baked_bodies().any(|(_, b)| b.component == arm));
    // The model parameter / user parameter survives the timeline's removal.
    assert!(doc.design().params().by_name("width").is_some());
    doc.design().validate().unwrap();
}

#[test]
fn switching_needs_an_evaluated_error_free_document() {
    let jobs = jobs();
    let mut d = solid_design();
    let root = d.root();
    d.add_feature(None, counter(vec![], 0, true), root, None)
        .unwrap();
    let mut doc = Document::from_design(d);
    let to_direct = Command::SetDesignMode {
        mode: DesignMode::Direct,
    };
    // Not evaluated yet.
    assert!(matches!(
        doc.apply(to_direct.clone()),
        Err(CommandError::Invalid(_))
    ));
    doc.recompute_blocking(&jobs).unwrap();
    // A failing feature would be lost silently: refused.
    let err = doc.apply(to_direct.clone()).unwrap_err();
    assert!(
        matches!(err, CommandError::Invalid(ref m) if m.contains("error")),
        "{err}"
    );
    assert_eq!(doc.design().meta().mode, DesignMode::History);
    assert!(!doc.can_undo_anything());
    // History::apply / apply_command have no geometry to bake with.
    let mut plain = solid_design();
    assert!(matches!(
        amg_doc::apply_command(&mut plain, to_direct),
        Err(CommandError::Invalid(_))
    ));
}

trait NoUndo {
    fn can_undo_anything(&self) -> bool;
}

impl NoUndo for Document {
    fn can_undo_anything(&self) -> bool {
        self.history().can_undo()
    }
}

#[test]
fn setting_the_current_mode_is_a_no_op() {
    let jobs = jobs();
    let mut doc = Document::from_design(solid_design());
    doc.recompute_blocking(&jobs).unwrap();
    doc.apply(Command::SetDesignMode {
        mode: DesignMode::History,
    })
    .unwrap();
    assert!(!doc.history().can_undo());
    doc.apply(Command::SetDesignMode {
        mode: DesignMode::Direct,
    })
    .unwrap();
    let n = doc.history().stack().applied();
    doc.apply(Command::SetDesignMode {
        mode: DesignMode::Direct,
    })
    .unwrap();
    assert_eq!(doc.history().stack().applied(), n);
}

#[test]
fn timeline_commands_are_wrong_mode_in_direct() {
    let jobs = jobs();
    let mut doc = Document::from_design(solid_design());
    doc.recompute_blocking(&jobs).unwrap();
    let root = doc.design().root();
    let f = doc.design().timeline().nodes().next().unwrap().id;
    doc.apply(Command::SetDesignMode {
        mode: DesignMode::Direct,
    })
    .unwrap();
    for cmd in [
        Command::AddFeature {
            at: None,
            feature: boxf(1.0),
            context: root,
            name: None,
        },
        Command::DeleteFeatures { ids: vec![f] },
        Command::SetMarker { index: 0 },
        Command::RenameFeature { id: f, name: None },
    ] {
        assert!(
            matches!(
                doc.apply(cmd),
                Err(CommandError::WrongMode(DesignMode::Direct))
            ),
            "timeline command in direct mode"
        );
    }
    // Non-timeline commands still work.
    let (id, _) = doc.design().baked_bodies().next().unwrap();
    doc.apply(Command::SetBodyProps {
        body: id,
        props: BodyProps {
            name: Some("X".into()),
            ..BodyProps::default()
        },
    })
    .unwrap();
}

#[test]
fn base_feature_bodies_can_be_read_by_new_features() {
    let jobs = jobs();
    let mut doc = Document::from_design(solid_design());
    doc.recompute_blocking(&jobs).unwrap();
    let before = bodies(&doc);
    let target = *before.keys().next().unwrap();
    doc.apply(Command::SetDesignMode {
        mode: DesignMode::Direct,
    })
    .unwrap();
    doc.apply(Command::SetDesignMode {
        mode: DesignMode::History,
    })
    .unwrap();
    let root = doc.design().root();
    let base = doc.design().timeline().nodes().next().unwrap().id;
    let out = doc
        .apply(Command::AddFeature {
            at: None,
            feature: counter(vec![Dependency::Body(target)], 9, false),
            context: root,
            name: None,
        })
        .unwrap();
    let amg_doc::CreatedId::Feature(f) = out.created[0] else {
        panic!()
    };
    doc.recompute_blocking(&jobs).unwrap();
    let ev = doc.evaluated();
    assert_eq!(ev.node(f).unwrap().health, Health::Ok);
    assert!(ev.body(body(f, 0)).is_some(), "the reader created its body");
    // The reader can't be moved before the base feature that provides its input.
    assert!(
        doc.apply(Command::MoveFeature { id: f, to: 0 }).is_err(),
        "ordering rule resolves the producer to the base feature"
    );
    // The base feature can't be edited, but deleting it removes its bodies and properties.
    let again = FeatureBox::new(BaseFeature { bodies: vec![] });
    assert!(matches!(
        doc.apply(Command::EditFeature {
            id: base,
            feature: again
        }),
        Err(CommandError::Invalid(_))
    ));
    doc.apply(Command::DeleteFeatures { ids: vec![base] })
        .unwrap();
    assert_eq!(doc.design().baked_bodies().count(), 0);
    doc.design().validate().unwrap();
    doc.recompute_blocking(&jobs).unwrap();
    assert!(matches!(
        doc.evaluated().node(f).unwrap().health,
        Health::Error(_)
    ));
}

#[test]
fn a_feature_is_baked_in_direct_mode() {
    let jobs = jobs();
    let mut doc = Document::from_design(mixed_design());
    doc.recompute_blocking(&jobs).unwrap();
    doc.apply(Command::SetDesignMode {
        mode: DesignMode::Direct,
    })
    .unwrap();
    doc.recompute_blocking(&jobs).unwrap();
    let before = bodies(&doc);
    let before_json = json(doc.design());
    let root = doc.design().root();
    let target = *before.keys().next().unwrap();

    // A failing feature changes nothing.
    let err = doc
        .bake_feature_blocking(
            &jobs,
            Command::AddFeature {
                at: None,
                feature: counter(vec![], 0, true),
                context: root,
                name: None,
            },
        )
        .unwrap_err();
    assert!(matches!(err, amg_doc::DirectError::Feature(_)), "{err}");
    assert_eq!(json(doc.design()), before_json);

    // A box and a feature that reads a baked body (passes it through, adds a mesh).
    let n = doc.history().stack().applied();
    let out = doc
        .bake_feature_blocking(
            &jobs,
            Command::AddFeature {
                at: None,
                feature: boxf(7.0),
                context: root,
                name: None,
            },
        )
        .unwrap();
    let amg_doc::CreatedId::Feature(boxed) = out.created[0] else {
        panic!()
    };
    let out = doc
        .bake_feature_blocking(
            &jobs,
            Command::AddFeature {
                at: None,
                feature: counter(vec![Dependency::Body(target)], 3, false),
                context: root,
                name: None,
            },
        )
        .unwrap();
    let amg_doc::CreatedId::Feature(read) = out.created[0] else {
        panic!()
    };
    assert_eq!(doc.history().stack().applied(), n + 2, "one undo step each");
    assert_eq!(doc.design().timeline().len(), 0, "the nodes are dropped");
    assert_eq!(doc.design().meta().mode, DesignMode::Direct);
    assert_eq!(doc.design().baked_bodies().count(), before.len() + 2);
    assert!(doc.design().baked(body(boxed, 0)).is_some());
    assert!(doc.design().baked(body(read, 0)).is_some());
    doc.design().validate().unwrap();
    doc.recompute_blocking(&jobs).unwrap();
    let after = bodies(&doc);
    for (id, (c, g)) in &before {
        assert!(after[id].1.same_as(g), "{id:?} untouched");
        assert_eq!(after[id].0, *c);
    }
    assert_eq!(after.len(), before.len() + 2);

    // Undo each; the document is exactly as before.
    assert!(doc.undo() && doc.undo());
    assert_eq!(json(doc.design()), before_json);
    assert!(doc.redo());
}

#[test]
fn a_stale_bake_is_refused() {
    let jobs = jobs();
    let mut doc = Document::from_design(solid_design());
    doc.recompute_blocking(&jobs).unwrap();
    doc.apply(Command::SetDesignMode {
        mode: DesignMode::Direct,
    })
    .unwrap();
    let root = doc.design().root();
    let bake = doc
        .bake_feature(
            &jobs,
            Command::AddFeature {
                at: None,
                feature: boxf(2.0),
                context: root,
                name: None,
            },
        )
        .unwrap()
        .wait_blocking()
        .unwrap();
    let (id, _) = doc.design().baked_bodies().next().unwrap();
    doc.apply(Command::SetBodyProps {
        body: id,
        props: BodyProps {
            visible: false,
            ..BodyProps::default()
        },
    })
    .unwrap();
    assert!(matches!(
        doc.commit_bake(bake),
        Err(CommandError::Invalid(_))
    ));
    // Not in history mode either.
    let mut hist = Document::from_design(solid_design());
    assert!(matches!(
        hist.bake_feature(
            &jobs,
            Command::AddFeature {
                at: None,
                feature: boxf(2.0),
                context: root,
                name: None
            }
        ),
        Err(CommandError::WrongMode(DesignMode::History))
    ));
    hist.recompute_blocking(&jobs).unwrap();
}

struct TempFile(std::path::PathBuf);

impl TempFile {
    fn new() -> TempFile {
        static N: AtomicU32 = AtomicU32::new(0);
        TempFile(std::env::temp_dir().join(format!(
            "amg-doc-direct-{}-{}.amg",
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

#[test]
fn direct_mode_saves_and_loads() {
    let jobs = jobs();
    let mut doc = Document::from_design(mixed_design());
    doc.recompute_blocking(&jobs).unwrap();
    doc.apply(Command::SetDesignMode {
        mode: DesignMode::Direct,
    })
    .unwrap();
    let root = doc.design().root();
    doc.bake_feature_blocking(
        &jobs,
        Command::AddFeature {
            at: None,
            feature: boxf(9.0),
            context: root,
            name: None,
        },
    )
    .unwrap();
    doc.recompute_blocking(&jobs).unwrap();
    let before = bodies(&doc);

    let f = TempFile::new();
    save(&f.0, &doc.snapshot(), &SaveOptions::default()).unwrap();
    // Every baked body has its own entry in bodies/.
    let names: Vec<String> = {
        let zip = std::fs::File::open(&f.0).unwrap();
        let z = zip::ZipArchive::new(zip).unwrap();
        z.file_names().map(|n| n.unwrap().into_owned()).collect()
    };
    for (_, b) in doc.design().baked_bodies() {
        assert!(names.contains(&b.blob.0), "{} in {names:?}", b.blob.0);
    }

    let loaded = load(&f.0).unwrap();
    assert_eq!(loaded.design.meta().mode, DesignMode::Direct);
    assert_eq!(json(&loaded.design), json(doc.design()));
    assert!(
        loaded.design.baked_bodies().all(|(_, b)| b.geom.is_some()),
        "geometry is read back"
    );
    // Re-save is byte-identical.
    let g = TempFile::new();
    save(
        &g.0,
        &std::sync::Arc::new(loaded.design.clone()),
        &SaveOptions::default(),
    )
    .unwrap();
    assert_eq!(
        std::fs::read(&f.0).unwrap(),
        std::fs::read(&g.0).unwrap(),
        "save -> load -> save is byte-identical"
    );

    // The loaded document evaluates to the same bodies (kinds; the stub geometry is recreated).
    let mut doc2 = Document::from_design(loaded.design);
    doc2.recompute_blocking(&jobs).unwrap();
    let after = bodies(&doc2);
    assert_eq!(
        before.keys().collect::<Vec<_>>(),
        after.keys().collect::<Vec<_>>()
    );
    for (id, (c, g)) in &before {
        assert_eq!(after[id].0, *c);
        assert_eq!(
            std::mem::discriminant(&after[id].1),
            std::mem::discriminant(g)
        );
    }

    // And it can go back to history mode and forth again.
    doc2.apply(Command::SetDesignMode {
        mode: DesignMode::History,
    })
    .unwrap();
    doc2.recompute_blocking(&jobs).unwrap();
    assert_eq!(bodies(&doc2).len(), before.len());
}
