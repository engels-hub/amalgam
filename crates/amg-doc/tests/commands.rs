//! DOC-05: commands, snapshot undo/redo, transactions.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    missing_docs,
    reason = "tests"
)]

use amg_core::{Transform, Vec3};
use amg_doc::testing::{TestAddMesh, TestBox, TestCounter};
use amg_doc::{
    BodyId, BodyProps, Command, CommandError, CreatedId, Dependency, Design, Expr, ExtRecord,
    Feature, FeatureBox, FeatureError, FeatureId, FeatureOutput, History, ParamKind, RecomputeCtx,
    UnitPrefs,
};
use proptest::prelude::*;
use serde::{Deserialize, Serialize};

/// A feature that reads one parameter (exercises `Feature::exprs`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ParamUser {
    pub len: Expr,
}

impl Feature for ParamUser {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }
    fn exprs(&self) -> Vec<&Expr> {
        vec![&self.len]
    }
    fn compute(&self, _ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Ok(FeatureOutput::default())
    }
}

amg_doc::register_feature!(ParamUser, "test.param_user", version = 1);

fn json(d: &Design) -> String {
    serde_json::to_string(d).unwrap()
}

fn boxf(w: f64) -> FeatureBox {
    FeatureBox::new(TestBox {
        size: [w, 1.0, 1.0],
    })
}

fn add_box(h: &History, w: f64) -> Command {
    Command::AddFeature {
        at: None,
        feature: boxf(w),
        context: h.design().root(),
        name: None,
    }
}

fn feature_id(out: &amg_doc::CommandOutcome) -> FeatureId {
    match out.created[0] {
        CreatedId::Feature(f) => f,
        ref o => panic!("expected a feature, got {o:?}"),
    }
}

#[test]
fn undo_redo_basics_and_labels() {
    amg_doc::testing::link();
    let mut h = History::new(Design::new("t"));
    let initial = json(h.design());
    assert!(!h.can_undo() && !h.undo());
    let a = h.apply(add_box(&h, 2.0)).unwrap();
    let fid = feature_id(&a);
    h.apply(Command::RenameFeature {
        id: fid,
        name: Some("Base".into()),
    })
    .unwrap();
    assert_eq!(h.undo_label(), Some("Rename feature"));
    let end = json(h.design());
    assert!(h.undo());
    assert_eq!(h.redo_label(), Some("Rename feature"));
    assert_eq!(h.undo_label(), Some("Add feature"));
    assert!(h.undo());
    assert_eq!(json(h.design()), initial);
    assert!(h.redo() && h.redo() && !h.redo());
    assert_eq!(json(h.design()), end);
    // A new command after an undo drops the redo tail.
    assert!(h.undo());
    h.apply(add_box(&h, 3.0)).unwrap();
    assert!(!h.can_redo());
}

#[test]
fn failed_and_noop_commands_leave_no_entry() {
    let mut h = History::new(Design::new("t"));
    let before = (json(h.design()), h.revision());
    let err = h
        .apply(Command::DeleteFeatures {
            ids: vec![FeatureId(999)],
        })
        .unwrap_err();
    assert_eq!(
        err,
        CommandError::NotFound {
            what: "feature",
            id: 999
        }
    );
    // Same name again is a no-op.
    h.apply(Command::SetDesignMeta {
        name: Some("t".into()),
        default_units: None,
    })
    .unwrap();
    assert_eq!((json(h.design()), h.revision()), before);
    assert!(!h.can_undo());
}

#[test]
fn transaction_commit_is_one_entry_and_rollback_restores() {
    amg_doc::testing::link();
    let mut h = History::new(Design::new("t"));
    let initial = json(h.design());
    let mut tx = h.transaction("Two boxes");
    tx.apply(add_box_tx(&tx, 1.0)).unwrap();
    tx.apply(add_box_tx(&tx, 2.0)).unwrap();
    assert_eq!(tx.design().timeline().len(), 2);
    tx.rollback();
    assert_eq!(json(h.design()), initial);
    assert!(!h.can_undo());

    let mut tx = h.transaction("Two boxes");
    tx.apply(add_box_tx(&tx, 1.0)).unwrap();
    // A failing command inside changes nothing; the caller decides.
    assert!(
        tx.apply(Command::MoveFeature {
            id: FeatureId(12345),
            to: 0
        })
        .is_err()
    );
    tx.apply(add_box_tx(&tx, 2.0)).unwrap();
    let rev = tx.commit();
    assert_eq!(rev, h.revision());
    assert_eq!(h.design().timeline().len(), 2);
    assert_eq!(h.stack().applied(), 1);
    assert_eq!(h.undo_label(), Some("Two boxes"));
    assert_eq!(h.stack().entries().next().unwrap().commands.len(), 2);
    assert!(h.undo());
    assert_eq!(json(h.design()), initial);

    // Dropping (e.g. via `?`) rolls back; an empty transaction adds nothing.
    {
        let mut tx = h.transaction("dropped");
        tx.apply(add_box_tx(&tx, 1.0)).unwrap();
    }
    h.transaction("empty").commit();
    assert_eq!(json(h.design()), initial);
    assert_eq!(h.stack().applied(), 0);
}

fn add_box_tx(tx: &amg_doc::Transaction<'_>, w: f64) -> Command {
    Command::AddFeature {
        at: None,
        feature: boxf(w),
        context: tx.design().root(),
        name: None,
    }
}

#[test]
fn depth_limit_drops_oldest_and_dirty_tracking() {
    amg_doc::testing::link();
    let mut h = History::with_limit(Design::new("t"), 3);
    assert!(!h.is_dirty());
    for i in 0..5 {
        h.apply(add_box(&h, 1.0 + f64::from(i))).unwrap();
    }
    assert_eq!(h.stack().applied(), 3);
    assert!(h.is_dirty());
    h.mark_saved();
    assert!(!h.is_dirty());
    assert!(h.undo());
    assert!(h.is_dirty());
    assert!(h.redo());
    assert!(!h.is_dirty());
    while h.undo() {}
    assert_eq!(
        h.design().timeline().len(),
        2,
        "two entries fell off the front"
    );
    h.set_limit(1);
    assert_eq!(h.stack().entries().count(), 1);
}

#[test]
fn delete_feature_converts_model_params_and_param_in_use_is_refused() {
    amg_doc::testing::link();
    let mut d = Design::new("t");
    let root = d.root();
    let mut len = Expr::Inline("12 mm".into());
    let f = d
        .add_feature(
            None,
            FeatureBox::new(ParamUser { len: len.clone() }),
            root,
            None,
        )
        .unwrap();
    // Wire a model parameter into the feature the way the command layer will (DOC-04).
    let ids = d
        .add_model_params(f, vec![(&mut len, amg_core::Dimension::LENGTH)])
        .unwrap();
    d.replace_feature(f, FeatureBox::new(ParamUser { len }))
        .unwrap();
    let p = ids[0];
    let mut h = History::new(d);

    let err = h.apply(Command::DeleteParam { id: p }).unwrap_err();
    assert!(
        matches!(err, CommandError::InUse { what: "parameter", id, ref by } if id == p.0 && by == &[f.0]),
        "{err:?}"
    );
    // Editing a feature to point at a missing parameter is refused.
    let bad = Command::EditFeature {
        id: f,
        feature: FeatureBox::new(ParamUser {
            len: Expr::Param(amg_doc::ParamId(9999)),
        }),
    };
    assert!(matches!(
        h.apply(bad).unwrap_err(),
        CommandError::NotFound {
            what: "parameter",
            ..
        }
    ));

    let before = json(h.design());
    h.apply(Command::DeleteFeatures { ids: vec![f] }).unwrap();
    let param = h.design().params().get(p).unwrap();
    assert_eq!(param.kind, ParamKind::User, "converted, not deleted");
    assert!(h.undo());
    assert_eq!(json(h.design()), before);
    assert!(matches!(
        h.design().params().get(p).unwrap().kind,
        ParamKind::Model { owner } if owner == f
    ));
    // After the feature is gone the parameter can be deleted.
    h.apply(Command::DeleteFeatures { ids: vec![f] }).unwrap();
    h.apply(Command::DeleteParam { id: p }).unwrap();
}

#[test]
fn param_commands_and_default_units() {
    let mut h = History::new(Design::new("t"));
    let a = h
        .apply(Command::AddParam {
            name: "w".into(),
            expr: "10 mm".into(),
            unit: None,
            comment: String::new(),
        })
        .unwrap();
    let CreatedId::Param(w) = a.created[0] else {
        panic!()
    };
    assert_eq!(
        h.apply(Command::AddParam {
            name: "w".into(),
            expr: "1".into(),
            unit: None,
            comment: String::new(),
        })
        .unwrap_err(),
        CommandError::NameTaken("w".into())
    );
    h.apply(Command::RenameParam {
        id: w,
        name: "width".into(),
    })
    .unwrap();
    assert!(matches!(
        h.apply(Command::SetParamExpr {
            id: w,
            expr: "nope + 1".into()
        })
        .unwrap_err(),
        CommandError::Expr(_)
    ));
    h.apply(Command::SetParamMeta {
        id: w,
        comment: Some("x".into()),
        favorite: Some(true),
    })
    .unwrap();
    h.apply(Command::SetDesignMeta {
        name: Some("renamed".into()),
        default_units: Some(UnitPrefs {
            length: amg_core::LengthUnit::Inch,
        }),
    })
    .unwrap();
    assert_eq!(h.design().meta().name, "renamed");
    assert!(h.undo());
    assert_eq!(h.design().meta().name, "t");
}

#[test]
fn components_ext_and_mode() {
    let mut h = History::new(Design::new("t"));
    let root = h.design().root();
    let o = h
        .apply(Command::NewComponent {
            parent: root,
            name: "Arm".into(),
            transform: Transform::from_translation(Vec3::X),
        })
        .unwrap();
    let (CreatedId::Component(c), CreatedId::Occurrence(occ)) =
        (o.created[0].clone(), o.created[1].clone())
    else {
        panic!()
    };
    h.apply(Command::RenameComponent {
        id: c,
        name: "Arm2".into(),
    })
    .unwrap();
    h.apply(Command::SetOccurrenceVisible {
        occ,
        visible: false,
    })
    .unwrap();
    h.apply(Command::SetOccurrenceTransform {
        occ,
        transform: Transform::from_translation(Vec3::Y),
    })
    .unwrap();
    assert!(!h.design().occurrence(occ).unwrap().visible);
    assert_eq!(h.design().component(c).unwrap().name, "Arm2");

    let rec = ExtRecord {
        kind: "test.ext".into(),
        version: 1,
        data: serde_json::json!({"a": 1}),
        blobs: vec![],
    };
    let e = h
        .apply(Command::AddExt {
            record: rec.clone(),
        })
        .unwrap();
    let CreatedId::Ext(key) = e.created[0].clone() else {
        panic!()
    };
    assert_eq!(h.design().ext(&key), Some(&rec));
    h.apply(Command::SetExt {
        key: key.clone(),
        record: None,
    })
    .unwrap();
    assert!(h.design().ext(&key).is_none());

    assert!(matches!(
        h.apply(Command::SetDesignMode {
            mode: amg_doc::DesignMode::Direct
        }),
        Err(CommandError::Invalid(_))
    ));
    h.apply(Command::SetDesignMode {
        mode: amg_doc::DesignMode::History,
    })
    .unwrap();
}

#[test]
fn commands_round_trip_through_serde() {
    amg_doc::testing::link();
    let root = Design::new("t").root();
    let rec = ExtRecord {
        kind: "k".into(),
        version: 1,
        data: serde_json::json!([1, 2]),
        blobs: vec![],
    };
    let cmds = vec![
        Command::AddFeature {
            at: Some(0),
            feature: boxf(2.0),
            context: root,
            name: Some("n".into()),
        },
        Command::EditFeature {
            id: FeatureId(3),
            feature: FeatureBox::new(TestAddMesh {
                bodies: 2,
                label: "x".into(),
            }),
        },
        Command::DeleteFeatures {
            ids: vec![FeatureId(2), FeatureId(3)],
        },
        Command::MoveFeature {
            id: FeatureId(2),
            to: 4,
        },
        Command::SetMarker { index: 1 },
        Command::SetSuppressed {
            ids: vec![FeatureId(2)],
            suppressed: true,
        },
        Command::RenameFeature {
            id: FeatureId(2),
            name: None,
        },
        Command::GroupFeatures {
            ids: vec![FeatureId(2)],
            name: "g".into(),
        },
        Command::Ungroup {
            group: amg_doc::GroupId(5),
        },
        Command::SetGroup {
            group: amg_doc::GroupId(5),
            name: Some("h".into()),
            collapsed: Some(true),
        },
        Command::AddParam {
            name: "a".into(),
            expr: "1 mm".into(),
            unit: None,
            comment: "c".into(),
        },
        Command::SetParamExpr {
            id: amg_doc::ParamId(6),
            expr: "2".into(),
        },
        Command::RenameParam {
            id: amg_doc::ParamId(6),
            name: "b".into(),
        },
        Command::SetParamMeta {
            id: amg_doc::ParamId(6),
            comment: None,
            favorite: Some(true),
        },
        Command::DeleteParam {
            id: amg_doc::ParamId(6),
        },
        Command::SetBodyProps {
            body: BodyId {
                origin: FeatureId(2),
                slot: 0,
            },
            props: BodyProps {
                name: Some("p".into()),
                ..BodyProps::default()
            },
        },
        Command::NewComponent {
            parent: root,
            name: "c".into(),
            transform: Transform::from_translation(Vec3::new(1.0, 2.5, -3.0)),
        },
        Command::RenameComponent {
            id: root,
            name: "r".into(),
        },
        Command::SetOccurrenceTransform {
            occ: amg_doc::OccurrenceId(7),
            transform: Transform::IDENTITY,
        },
        Command::SetOccurrenceVisible {
            occ: amg_doc::OccurrenceId(7),
            visible: false,
        },
        Command::SetDesignMeta {
            name: Some("n".into()),
            default_units: Some(UnitPrefs::default()),
        },
        Command::SetDesignMode {
            mode: amg_doc::DesignMode::History,
        },
        Command::SetExt {
            key: amg_doc::ExtKey {
                kind: "k".into(),
                id: 9,
            },
            record: Some(rec.clone()),
        },
        Command::AddExt { record: rec },
    ];
    for c in cmds {
        let s = serde_json::to_string(&c).unwrap();
        let back: Command = serde_json::from_str(&s).unwrap();
        assert_eq!(serde_json::to_string(&back).unwrap(), s, "{c:?}");
    }
}

// ---- property test ------------------------------------------------------------------------

#[derive(Clone, Debug)]
struct Op {
    kind: u8,
    a: u32,
    b: u32,
    c: u32,
}

fn op() -> impl Strategy<Value = Op> {
    (0u8..22, any::<u32>(), any::<u32>(), 1u32..1000).prop_map(|(kind, a, b, c)| Op {
        kind,
        a,
        b,
        c,
    })
}

fn pick<T: Copy>(v: &[T], i: u32) -> Option<T> {
    (!v.is_empty()).then(|| v[i as usize % v.len()])
}

/// Turns a random op into a command that is usually valid against `d`.
fn build(d: &Design, o: &Op) -> Option<Command> {
    let tl = d.timeline();
    let fids: Vec<FeatureId> = tl.nodes().map(|n| n.id).collect();
    let params: Vec<_> = d.params().iter().map(|(id, _)| id).collect();
    let root = d.root();
    let occs: Vec<_> = d.occurrences().map(|(id, _)| id).collect();
    let comps: Vec<_> = d.components().map(|(id, _)| id).collect();
    let groups: Vec<_> = tl.groups().map(|(id, _)| id).collect();
    let w = f64::from(o.c) / 10.0;
    Some(match o.kind {
        0 | 1 => Command::AddFeature {
            at: o.b.is_multiple_of(2).then(|| o.a as usize % (tl.len() + 1)),
            feature: boxf(w),
            context: root,
            name: None,
        },
        2 => Command::AddFeature {
            at: None,
            feature: FeatureBox::new(TestCounter {
                deps: pick(&fids, o.a)
                    .map(|f| vec![Dependency::Feature(f)])
                    .unwrap_or_default(),
                value: i64::from(o.c),
                fail: false,
            }),
            context: root,
            name: Some(format!("c{}", o.c)),
        },
        3 => {
            let id = tl
                .nodes()
                .filter(|n| n.feature.downcast_ref::<TestBox>().is_some())
                .map(|n| n.id)
                .nth(o.a as usize % 7)?;
            Command::EditFeature {
                id,
                feature: boxf(w),
            }
        }
        4 => Command::DeleteFeatures {
            ids: vec![pick(&fids, o.a)?],
        },
        5 => Command::MoveFeature {
            id: pick(&fids, o.a)?,
            to: o.b as usize % (tl.len() + 1),
        },
        6 => Command::SetMarker {
            index: o.a as usize % (tl.len() + 1),
        },
        7 => Command::SetSuppressed {
            ids: vec![pick(&fids, o.a)?],
            suppressed: o.b.is_multiple_of(2),
        },
        8 => Command::RenameFeature {
            id: pick(&fids, o.a)?,
            name: (!o.b.is_multiple_of(3)).then(|| format!("n{}", o.c)),
        },
        9 => {
            let last = fids.len().checked_sub(1)?;
            let i = o.a as usize % fids.len();
            Command::GroupFeatures {
                ids: fids.get(i..=(i + (o.b as usize % 2)).min(last))?.to_vec(),
                name: "grp".into(),
            }
        }
        10 => Command::Ungroup {
            group: pick(&groups, o.a)?,
        },
        11 => Command::AddParam {
            name: format!("p{}", o.c),
            expr: format!("{w} mm"),
            unit: None,
            comment: String::new(),
        },
        12 => {
            let id = pick(&params, o.a)?;
            Command::SetParamExpr {
                id,
                expr: format!("{} mm", o.c),
            }
        }
        13 => Command::RenameParam {
            id: pick(&params, o.a)?,
            name: format!("q{}", o.c),
        },
        14 => Command::DeleteParam {
            id: pick(&params, o.a)?,
        },
        15 => Command::SetBodyProps {
            body: BodyId {
                origin: pick(&fids, o.a)?,
                slot: 0,
            },
            props: BodyProps {
                name: Some(format!("b{}", o.c)),
                visible: o.b.is_multiple_of(2),
                ..BodyProps::default()
            },
        },
        16 => Command::NewComponent {
            parent: pick(&comps, o.a)?,
            name: format!("comp{}", o.c),
            transform: Transform::from_translation(Vec3::new(w, 0.0, 0.0)),
        },
        17 => Command::SetOccurrenceTransform {
            occ: pick(&occs, o.a)?,
            transform: Transform::from_translation(Vec3::new(0.0, w, 1.0)),
        },
        18 => Command::SetOccurrenceVisible {
            occ: pick(&occs, o.a)?,
            visible: o.b.is_multiple_of(2),
        },
        19 => Command::AddExt {
            record: ExtRecord {
                kind: "test.ext".into(),
                version: 1,
                data: serde_json::json!({ "v": o.c }),
                blobs: vec![],
            },
        },
        20 => Command::SetDesignMeta {
            name: Some(format!("design{}", o.c)),
            default_units: o.b.is_multiple_of(4).then_some(UnitPrefs {
                length: if o.b.is_multiple_of(8) {
                    amg_core::LengthUnit::Inch
                } else {
                    amg_core::LengthUnit::Millimetre
                },
            }),
        },
        _ => Command::SetGroup {
            group: pick(&groups, o.a)?,
            name: Some(format!("g{}", o.c)),
            collapsed: Some(o.b.is_multiple_of(2)),
        },
    })
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    #[test]
    fn undo_all_then_redo_all(ops in proptest::collection::vec(op(), 1..200)) {
        amg_doc::testing::link();
        let mut h = History::new(Design::new("prop"));
        let initial = json(h.design());
        let mut applied = 0usize;
        for o in &ops {
            let Some(cmd) = build(h.design(), o) else { continue };
            let before = json(h.design());
            match h.apply(cmd) {
                Ok(_) => applied += 1,
                Err(_) => prop_assert_eq!(json(h.design()), before, "failed command changed the design"),
            }
            // Every state reachable by commands must save and reload.
            prop_assert!(h.design().validate().is_ok());
        }
        let end = json(h.design());
        let entries = h.stack().applied();
        prop_assert!(entries <= applied);
        let mut undone = 0;
        while h.undo() { undone += 1; }
        prop_assert_eq!(undone, entries);
        prop_assert_eq!(json(h.design()), initial);
        while h.redo() {}
        prop_assert_eq!(json(h.design()), end.clone());
        let back: Design = serde_json::from_str(&end).unwrap();
        prop_assert_eq!(json(&back), end);
    }
}
