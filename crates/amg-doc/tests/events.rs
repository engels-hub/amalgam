//! DOC-09: change events (S01 §9). Exact event sets per command type and for undo/redo.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    missing_docs,
    reason = "tests"
)]

use amg_core::{Transform, Vec3};
use amg_doc::DocEvent::{
    BodyAdded, BodyGeometryChanged, BodyPropsChanged, BodyRemoved, ComponentTreeChanged,
    DocumentReplaced, ExtChanged, HealthChanged, OccurrenceTransformChanged,
    OccurrenceVisibilityChanged, ParamsChanged, RecomputeFinished, RecomputeStarted,
    TimelineChanged,
};
use amg_doc::testing::{TestBox, TestCounter};
use amg_doc::{
    BodyId, BodyProps, Command, CreatedId, Design, DocEvent, Document, ExtRecord, FeatureBox,
    FeatureId, Revision,
};
use amg_jobs::{JobSystem, JobSystemConfig};

fn jobs() -> JobSystem {
    JobSystem::new(JobSystemConfig::default()).unwrap()
}

fn bx(x: f64) -> FeatureBox {
    FeatureBox::new(TestBox {
        size: [x, 1.0, 1.0],
    })
}

struct Fixture {
    doc: Document,
    jobs: JobSystem,
    f: [FeatureId; 3],
    b: [BodyId; 3],
}

impl Fixture {
    /// Three boxes (three bodies), evaluated, events drained.
    fn new() -> Fixture {
        let mut d = Design::new("doc09");
        let root = d.root();
        let f = [1.0, 2.0, 3.0].map(|x| d.add_feature(None, bx(x), root, None).unwrap());
        let mut doc = Document::from_design(d);
        let jobs = jobs();
        doc.recompute_blocking(&jobs).unwrap();
        let ids: Vec<BodyId> = doc.evaluated().bodies().map(|(id, _)| id).collect();
        let b = [ids[0], ids[1], ids[2]];
        doc.drain_events();
        Fixture { doc, jobs, f, b }
    }

    fn apply(&mut self, cmd: Command) -> Vec<CreatedId> {
        self.doc.apply(cmd).unwrap().created
    }

    fn rev(&self) -> Revision {
        self.doc.history().revision()
    }

    /// Events from the authored change alone (before any recompute).
    fn authored(&mut self) -> Vec<DocEvent> {
        self.doc.drain_events()
    }

    /// Recomputes and returns the events of the recompute.
    fn recompute(&mut self) -> Vec<DocEvent> {
        self.doc.recompute_blocking(&self.jobs).unwrap();
        self.doc.drain_events()
    }
}

#[test]
fn new_document_and_first_recompute() {
    let mut d = Design::new("doc09");
    let root = d.root();
    let f: Vec<FeatureId> = (1..=3)
        .map(|x| d.add_feature(None, bx(f64::from(x)), root, None).unwrap())
        .collect();
    let mut doc = Document::from_design(d);
    assert_eq!(doc.drain_events(), vec![DocumentReplaced]);
    assert_eq!(doc.drain_events(), vec![]);

    doc.recompute_blocking(&jobs()).unwrap();
    let b: Vec<BodyId> = doc.evaluated().bodies().map(|(id, _)| id).collect();
    let rev = doc.history().revision();
    assert_eq!(
        doc.drain_events(),
        vec![
            RecomputeStarted(rev),
            BodyAdded(b[0]),
            BodyAdded(b[1]),
            BodyAdded(b[2]),
            HealthChanged(f[0]),
            HealthChanged(f[1]),
            HealthChanged(f[2]),
            RecomputeFinished {
                revision: rev,
                failed: 0
            },
        ]
    );
    assert_eq!(doc.drain_events(), vec![]);
}

#[test]
fn editing_one_feature_changes_only_its_body() {
    let mut fx = Fixture::new();
    fx.apply(Command::EditFeature {
        id: fx.f[1],
        feature: bx(9.0),
    });
    assert_eq!(fx.authored(), vec![TimelineChanged]);
    let rev = fx.rev();
    assert_eq!(
        fx.recompute(),
        vec![
            RecomputeStarted(rev),
            BodyGeometryChanged(fx.b[1]),
            RecomputeFinished {
                revision: rev,
                failed: 0
            },
        ]
    );
}

#[test]
fn props_only_change_emits_no_geometry_event() {
    let mut fx = Fixture::new();
    fx.apply(Command::SetBodyProps {
        body: fx.b[0],
        props: BodyProps {
            name: Some("Lid".into()),
            ..BodyProps::default()
        },
    });
    assert_eq!(fx.authored(), vec![BodyPropsChanged(fx.b[0])]);
    let rev = fx.rev();
    assert_eq!(
        fx.recompute(),
        vec![
            RecomputeStarted(rev),
            RecomputeFinished {
                revision: rev,
                failed: 0
            }
        ]
    );
}

#[test]
fn timeline_commands() {
    let mut fx = Fixture::new();
    let root = fx.doc.design().root();
    let cmds = vec![
        Command::AddFeature {
            at: None,
            feature: bx(5.0),
            context: root,
            name: None,
        },
        Command::RenameFeature {
            id: fx.f[0],
            name: Some("Base".into()),
        },
        Command::SetSuppressed {
            ids: vec![fx.f[2]],
            suppressed: true,
        },
        Command::MoveFeature { id: fx.f[0], to: 2 },
        Command::SetMarker { index: 1 },
        Command::GroupFeatures {
            ids: vec![fx.f[0]],
            name: "g".into(),
        },
        Command::DeleteFeatures { ids: vec![fx.f[1]] },
    ];
    for c in cmds {
        let label = c.label();
        fx.apply(c);
        assert_eq!(fx.authored(), vec![TimelineChanged], "{label}");
    }
}

#[test]
fn rolling_back_removes_the_body_and_changes_health() {
    let mut fx = Fixture::new();
    fx.apply(Command::SetMarker { index: 2 });
    assert_eq!(fx.authored(), vec![TimelineChanged]);
    let rev = fx.rev();
    assert_eq!(
        fx.recompute(),
        vec![
            RecomputeStarted(rev),
            BodyRemoved(fx.b[2]),
            HealthChanged(fx.f[2]),
            RecomputeFinished {
                revision: rev,
                failed: 0
            },
        ]
    );
}

#[test]
fn parameter_commands() {
    let mut fx = Fixture::new();
    let created = fx.apply(Command::AddParam {
        name: "w".into(),
        expr: "3".into(),
        unit: None,
        comment: String::new(),
    });
    let CreatedId::Param(p) = created[0].clone() else {
        panic!()
    };
    assert_eq!(fx.authored(), vec![ParamsChanged]);
    fx.apply(Command::SetParamExpr {
        id: p,
        expr: "4".into(),
    });
    assert_eq!(fx.authored(), vec![ParamsChanged]);
    fx.apply(Command::DeleteParam { id: p });
    assert_eq!(fx.authored(), vec![ParamsChanged]);
}

#[test]
fn component_and_occurrence_commands() {
    let mut fx = Fixture::new();
    let root = fx.doc.design().root();
    let created = fx.apply(Command::NewComponent {
        parent: root,
        name: "Arm".into(),
        transform: Transform::IDENTITY,
    });
    assert_eq!(fx.authored(), vec![ComponentTreeChanged]);
    let (CreatedId::Component(c), CreatedId::Occurrence(occ)) =
        (created[0].clone(), created[1].clone())
    else {
        panic!("{created:?}")
    };

    fx.apply(Command::SetOccurrenceTransform {
        occ,
        transform: Transform::from_translation(Vec3::X),
    });
    assert_eq!(fx.authored(), vec![OccurrenceTransformChanged(occ)]);

    fx.apply(Command::SetOccurrenceVisible {
        occ,
        visible: false,
    });
    assert_eq!(fx.authored(), vec![OccurrenceVisibilityChanged(occ)]);

    fx.apply(Command::RenameComponent {
        id: c,
        name: "Arm2".into(),
    });
    assert_eq!(fx.authored(), vec![ComponentTreeChanged]);

    // The design name is the root component's name.
    fx.apply(Command::SetDesignMeta {
        name: Some("renamed".into()),
        default_units: None,
    });
    assert_eq!(fx.authored(), vec![ComponentTreeChanged]);
}

#[test]
fn extension_commands() {
    let mut fx = Fixture::new();
    let rec = ExtRecord {
        kind: "test.ext".into(),
        version: 1,
        data: serde_json::json!({"a": 1}),
        blobs: vec![],
    };
    let created = fx.apply(Command::AddExt {
        record: rec.clone(),
    });
    let CreatedId::Ext(key) = created[0].clone() else {
        panic!()
    };
    assert_eq!(fx.authored(), vec![ExtChanged(key.clone())]);
    fx.apply(Command::SetExt {
        key: key.clone(),
        record: Some(ExtRecord {
            data: serde_json::json!({"a": 2}),
            ..rec
        }),
    });
    assert_eq!(fx.authored(), vec![ExtChanged(key.clone())]);
    fx.apply(Command::SetExt {
        key: key.clone(),
        record: None,
    });
    assert_eq!(fx.authored(), vec![ExtChanged(key)]);
}

#[test]
fn transaction_emits_the_union_once() {
    let mut fx = Fixture::new();
    let mut tx = fx.doc.transaction("both");
    tx.apply(Command::AddParam {
        name: "w".into(),
        expr: "3".into(),
        unit: None,
        comment: String::new(),
    })
    .unwrap();
    tx.apply(Command::SetBodyProps {
        body: fx.b[2],
        props: BodyProps {
            visible: false,
            ..BodyProps::default()
        },
    })
    .unwrap();
    tx.apply(Command::RenameFeature {
        id: fx.f[0],
        name: Some("x".into()),
    })
    .unwrap();
    tx.commit();
    assert_eq!(
        fx.authored(),
        vec![TimelineChanged, ParamsChanged, BodyPropsChanged(fx.b[2])]
    );
}

#[test]
fn rolled_back_transaction_and_failed_command_emit_nothing() {
    let mut fx = Fixture::new();
    let mut tx = fx.doc.transaction("nope");
    tx.apply(Command::RenameFeature {
        id: fx.f[0],
        name: Some("x".into()),
    })
    .unwrap();
    tx.rollback();
    assert!(
        fx.doc
            .apply(Command::DeleteFeatures {
                ids: vec![FeatureId(999_999)]
            })
            .is_err()
    );
    assert_eq!(fx.authored(), vec![]);
}

#[test]
fn undo_and_redo() {
    let mut fx = Fixture::new();
    fx.apply(Command::EditFeature {
        id: fx.f[1],
        feature: bx(9.0),
    });
    fx.apply(Command::SetBodyProps {
        body: fx.b[0],
        props: BodyProps {
            visible: false,
            ..BodyProps::default()
        },
    });
    fx.authored();
    fx.recompute();

    // Undo the props edit: authored event only, geometry untouched.
    assert!(fx.doc.undo());
    assert_eq!(fx.authored(), vec![BodyPropsChanged(fx.b[0])]);
    let rev = fx.rev();
    assert_eq!(
        fx.recompute(),
        vec![
            RecomputeStarted(rev),
            RecomputeFinished {
                revision: rev,
                failed: 0
            }
        ]
    );

    // Undo the feature edit: the old geometry comes back out of the cache.
    assert!(fx.doc.undo());
    assert_eq!(fx.authored(), vec![TimelineChanged]);
    let rev = fx.rev();
    assert_eq!(
        fx.recompute(),
        vec![
            RecomputeStarted(rev),
            BodyGeometryChanged(fx.b[1]),
            RecomputeFinished {
                revision: rev,
                failed: 0
            }
        ]
    );

    // Redo it.
    assert!(fx.doc.redo());
    assert_eq!(fx.authored(), vec![TimelineChanged]);
    let rev = fx.rev();
    assert_eq!(
        fx.recompute(),
        vec![
            RecomputeStarted(rev),
            BodyGeometryChanged(fx.b[1]),
            RecomputeFinished {
                revision: rev,
                failed: 0
            }
        ]
    );
    assert!(fx.doc.redo());
    assert_eq!(fx.authored(), vec![BodyPropsChanged(fx.b[0])]);
}

#[test]
fn edit_undone_before_the_next_drain_nets_out() {
    let mut fx = Fixture::new();
    fx.apply(Command::RenameFeature {
        id: fx.f[0],
        name: Some("x".into()),
    });
    assert!(fx.doc.undo());
    assert_eq!(fx.authored(), vec![]);
    // The authored state is back to what the evaluated state was computed from: the cache
    // serves everything and no body changes.
    let rev = fx.rev();
    assert_eq!(
        fx.recompute(),
        vec![
            RecomputeStarted(rev),
            RecomputeFinished {
                revision: rev,
                failed: 0
            }
        ]
    );
}

#[test]
fn failing_feature_reports_health_and_failed_count() {
    let mut d = Design::new("doc09-fail");
    let root = d.root();
    let counter = |fail| {
        FeatureBox::new(TestCounter {
            deps: vec![],
            value: 1,
            fail,
        })
    };
    let f = d.add_feature(None, counter(false), root, None).unwrap();
    let mut doc = Document::from_design(d);
    let jobs = jobs();
    doc.recompute_blocking(&jobs).unwrap();
    doc.drain_events();

    doc.apply(Command::EditFeature {
        id: f,
        feature: counter(true),
    })
    .unwrap();
    let rev = doc.history().revision();
    doc.recompute_blocking(&jobs).unwrap();
    let ev = doc.drain_events();
    assert!(ev.contains(&HealthChanged(f)), "{ev:?}");
    assert_eq!(
        ev.last(),
        Some(&RecomputeFinished {
            revision: rev,
            failed: 1
        })
    );
}
