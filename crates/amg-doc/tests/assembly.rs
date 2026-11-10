//! ASM-01: components and occurrences (S06): tree commands, cascade delete, placements and
//! world queries, rigid-transform validation, events, and "moving recomputes nothing".

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "tests"
)]

use amg_core::{Quat, Transform, Vec3};
use amg_doc::DocEvent::{
    BodyRemoved, ComponentTreeChanged, HealthChanged, OccurrenceTransformChanged,
    RecomputeFinished, RecomputeStarted, TimelineChanged,
};
use amg_doc::testing::{TestBox, TestCounter};
use amg_doc::{
    BodyId, Command, CommandError, ComponentId, CreatedId, Dependency, Design, DesignMode,
    DocEvent, Document, ErrorCause, FeatureBox, FeatureId, Health, History, ModelError,
    OccurrenceId,
};
use amg_jobs::{JobSystem, JobSystemConfig};
use proptest::prelude::*;

fn jobs() -> JobSystem {
    JobSystem::new(JobSystemConfig::default()).unwrap()
}

fn json(d: &Design) -> String {
    serde_json::to_string(d).unwrap()
}

fn tr(x: f64, y: f64, z: f64) -> Transform {
    Transform::from_translation(Vec3::new(x, y, z))
}

fn rot(axis: Vec3, angle: f64, t: Vec3) -> Transform {
    Transform::from_rotation_translation(Quat::from_axis_angle(axis.normalize(), angle), t)
}

fn bx() -> FeatureBox {
    FeatureBox::new(TestBox {
        size: [1.0, 2.0, 3.0],
    })
}

fn max_diff(a: &Transform, b: &Transform) -> f64 {
    a.to_cols_array()
        .iter()
        .zip(b.to_cols_array())
        .map(|(x, y)| (x - y).abs())
        .fold(0.0, f64::max)
}

fn comp_occ(out: &amg_doc::CommandOutcome) -> (ComponentId, OccurrenceId) {
    let mut c = None;
    let mut o = None;
    for id in &out.created {
        match id {
            CreatedId::Component(x) => c = Some(*x),
            CreatedId::Occurrence(x) => o = Some(*x),
            _ => {}
        }
    }
    (c.unwrap(), o.unwrap())
}

// ---- 1. property test ---------------------------------------------------------------------

#[derive(Clone, Debug)]
struct Op {
    kind: u8,
    a: u32,
    b: u32,
    c: u32,
    ang: f64,
}

fn op() -> impl Strategy<Value = Op> {
    (
        0u8..9,
        any::<u32>(),
        any::<u32>(),
        any::<u32>(),
        -3.0f64..3.0,
    )
        .prop_map(|(kind, a, b, c, ang)| Op { kind, a, b, c, ang })
}

fn random_rigid(o: &Op) -> Transform {
    let axis = Vec3::new(
        1.0 + f64::from(o.a % 5),
        f64::from(o.b % 7),
        f64::from(o.c % 3) - 1.0,
    );
    let t = rot(
        axis,
        o.ang,
        Vec3::new(f64::from(o.a % 100), f64::from(o.b % 50), 0.0),
    );
    if o.c.is_multiple_of(4) {
        // A mirror: reflect x before the rotation.
        t * Transform::from_scale(Vec3::new(-1.0, 1.0, 1.0))
    } else {
        t
    }
}

fn pick<T: Copy>(v: &[T], i: u32) -> Option<T> {
    (!v.is_empty()).then(|| v[i as usize % v.len()])
}

fn build(d: &Design, o: &Op) -> Option<Command> {
    let comps: Vec<ComponentId> = d.components().map(|(c, _)| c).collect();
    let occs: Vec<OccurrenceId> = d.occurrences().map(|(o, _)| o).collect();
    Some(match o.kind {
        0 => Command::NewComponent {
            parent: pick(&comps, o.a)?,
            name: format!("c{}", o.c),
            transform: random_rigid(o),
        },
        1 => Command::AddOccurrence {
            parent: pick(&comps, o.a)?,
            component: pick(&comps, o.b)?,
            transform: random_rigid(o),
            name: o.c.is_multiple_of(2).then(|| format!("o{}", o.c)),
        },
        2 => Command::DeleteOccurrences {
            occs: vec![pick(&occs, o.a)?],
        },
        3 => Command::MoveOccurrence {
            occ: pick(&occs, o.a)?,
            parent: pick(&comps, o.b)?,
            keep_world: o.c.is_multiple_of(2),
        },
        4 => Command::SetOccurrenceGrounded {
            occ: pick(&occs, o.a)?,
            grounded: o.b.is_multiple_of(2),
        },
        5 => Command::SetOccurrenceName {
            occ: pick(&occs, o.a)?,
            name: (!o.b.is_multiple_of(3)).then(|| format!("n{}", o.c)),
        },
        6 => Command::SetOccurrenceTransforms {
            moves: vec![
                (pick(&occs, o.a)?, random_rigid(o)),
                (
                    pick(&occs, o.a.wrapping_add(1))?,
                    tr(1.0, f64::from(o.c % 9), 0.0),
                ),
            ],
        },
        7 => Command::AddFeature {
            at: None,
            feature: bx(),
            context: pick(&comps, o.a)?,
            name: None,
        },
        _ => Command::SetOccurrenceVisible {
            occ: pick(&occs, o.a)?,
            visible: o.b.is_multiple_of(2),
        },
    })
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    /// ASM-01 Done-when 1.
    #[test]
    fn tree_commands_undo_all_redo_all(ops in proptest::collection::vec(op(), 1..100)) {
        amg_doc::testing::link();
        let mut h = History::new(Design::new("asm01"));
        let initial = json(h.design());
        for o in &ops {
            let Some(cmd) = build(h.design(), o) else { continue };
            let before = json(h.design());
            if h.apply(cmd).is_err() {
                prop_assert_eq!(json(h.design()), before, "failed command changed the design");
            }
            prop_assert!(h.design().validate().is_ok());
        }
        let end = json(h.design());
        while h.undo() {}
        prop_assert_eq!(json(h.design()), initial);
        while h.redo() {}
        prop_assert_eq!(json(h.design()), end.clone());
        let back: Design = serde_json::from_str(&end).unwrap();
        prop_assert_eq!(json(&back), end);
    }
}

// ---- 2. cascade delete --------------------------------------------------------------------

#[test]
fn deleting_last_occurrence_cascades_in_one_undo_entry() {
    amg_doc::testing::link();
    let mut doc = Document::new("cascade");
    let root = doc.design().root();
    let (c, oc) = comp_occ(
        &doc.apply(Command::NewComponent {
            parent: root,
            name: "C".into(),
            transform: tr(5.0, 0.0, 0.0),
        })
        .unwrap(),
    );
    let (d_comp, _od) = comp_occ(
        &doc.apply(Command::NewComponent {
            parent: c,
            name: "D".into(),
            transform: tr(0.0, 1.0, 0.0),
        })
        .unwrap(),
    );
    // A second component that only lives in the root: must survive.
    let (keep, _ok) = comp_occ(
        &doc.apply(Command::NewComponent {
            parent: root,
            name: "Keep".into(),
            transform: Transform::IDENTITY,
        })
        .unwrap(),
    );
    let add = |doc: &mut Document, context| {
        let out = doc
            .apply(Command::AddFeature {
                at: None,
                feature: bx(),
                context,
                name: None,
            })
            .unwrap();
        let CreatedId::Feature(f) = out.created[0] else {
            panic!()
        };
        f
    };
    let in_c = [add(&mut doc, c), add(&mut doc, c), add(&mut doc, c)];
    let in_d = add(&mut doc, d_comp);
    let in_keep = add(&mut doc, keep);
    // A root feature that reads a body of C.
    let reader = doc
        .apply(Command::AddFeature {
            at: None,
            feature: FeatureBox::new(TestCounter {
                deps: vec![Dependency::Body(BodyId {
                    origin: in_c[0],
                    slot: 0,
                })],
                value: 0,
                fail: false,
            }),
            context: root,
            name: None,
        })
        .unwrap();
    let CreatedId::Feature(reader) = reader.created[0] else {
        panic!()
    };
    let js = jobs();
    doc.recompute_blocking(&js).unwrap();
    assert!(matches!(
        doc.evaluated().node(reader).unwrap().health,
        Health::Ok
    ));

    let entries = doc.history().stack().applied();
    let before = json(doc.design());
    doc.apply(Command::DeleteOccurrences { occs: vec![oc] })
        .unwrap();
    assert_eq!(doc.history().stack().applied(), entries + 1);
    let d = doc.design();
    d.validate().unwrap();
    assert!(d.component(c).is_none());
    assert!(d.component(d_comp).is_none());
    assert!(d.component(keep).is_some());
    for f in in_c.iter().chain([&in_d]) {
        assert!(!d.timeline().contains(*f), "{f} should be gone");
    }
    assert!(d.timeline().contains(in_keep));
    assert!(d.timeline().contains(reader));

    doc.recompute_blocking(&js).unwrap();
    let Health::Error(diag) = &doc.evaluated().node(reader).unwrap().health else {
        panic!("reader should fail")
    };
    assert!(matches!(diag.cause, Some(ErrorCause::Upstream(_))));

    assert!(doc.undo());
    assert_eq!(json(doc.design()), before);
    doc.recompute_blocking(&js).unwrap();
    assert!(matches!(
        doc.evaluated().node(reader).unwrap().health,
        Health::Ok
    ));
}

#[test]
fn delete_keeps_components_with_other_occurrences() {
    let mut h = History::new(Design::new("shared"));
    let root = h.design().root();
    let (a, oa1) = comp_occ(
        &h.apply(Command::NewComponent {
            parent: root,
            name: "A".into(),
            transform: Transform::IDENTITY,
        })
        .unwrap(),
    );
    let CreatedId::Occurrence(oa2) = h
        .apply(Command::AddOccurrence {
            parent: root,
            component: a,
            transform: tr(10.0, 0.0, 0.0),
            name: Some("second".into()),
        })
        .unwrap()
        .created[0]
    else {
        panic!()
    };
    h.apply(Command::DeleteOccurrences { occs: vec![oa1] })
        .unwrap();
    assert!(h.design().component(a).is_some());
    assert_eq!(
        h.design().occurrence(oa2).unwrap().name.as_deref(),
        Some("second")
    );
    h.apply(Command::DeleteOccurrences { occs: vec![oa2] })
        .unwrap();
    assert!(h.design().component(a).is_none());
    assert!(matches!(
        h.apply(Command::DeleteOccurrences { occs: vec![] }),
        Err(CommandError::Model(_))
    ));
    assert!(matches!(
        h.apply(Command::DeleteOccurrences { occs: vec![oa2] }),
        Err(CommandError::NotFound { .. })
    ));
}

#[test]
fn cascade_removes_baked_bodies_in_direct_mode() {
    amg_doc::testing::link();
    let mut doc = Document::new("direct");
    let root = doc.design().root();
    let (c, oc) = comp_occ(
        &doc.apply(Command::NewComponent {
            parent: root,
            name: "C".into(),
            transform: Transform::IDENTITY,
        })
        .unwrap(),
    );
    doc.apply(Command::AddFeature {
        at: None,
        feature: bx(),
        context: c,
        name: None,
    })
    .unwrap();
    doc.apply(Command::AddFeature {
        at: None,
        feature: bx(),
        context: root,
        name: None,
    })
    .unwrap();
    let js = jobs();
    doc.recompute_blocking(&js).unwrap();
    doc.apply(Command::SetDesignMode {
        mode: DesignMode::Direct,
    })
    .unwrap();
    assert_eq!(doc.design().baked_bodies().count(), 2);
    doc.apply(Command::DeleteOccurrences { occs: vec![oc] })
        .unwrap();
    assert_eq!(doc.design().baked_bodies().count(), 1);
    doc.design().validate().unwrap();
    // Leaving direct mode adopts only what is left.
    doc.apply(Command::SetDesignMode {
        mode: DesignMode::History,
    })
    .unwrap();
    doc.design().validate().unwrap();
    doc.recompute_blocking(&js).unwrap();
    assert_eq!(doc.evaluated().bodies().count(), 1);
}

// ---- 3. world transforms and instances ----------------------------------------------------

/// root ⊃ A×2 ⊃ B ⊃ C; the body lives in C.
struct Tree {
    doc: Document,
    body: BodyId,
    a_occs: [OccurrenceId; 2],
    b_occ: OccurrenceId,
    c_occ: OccurrenceId,
    ts: [Transform; 4],
}

fn tree() -> Tree {
    amg_doc::testing::link();
    let mut doc = Document::new("tree");
    let root = doc.design().root();
    let ta1 = rot(Vec3::Z, 0.7, Vec3::new(10.0, 0.0, 0.0));
    let ta2 = rot(Vec3::X, -1.1, Vec3::new(0.0, 20.0, 0.0));
    let tb = rot(Vec3::new(1.0, 1.0, 0.0), 0.3, Vec3::new(1.0, 2.0, 3.0));
    let tc = rot(Vec3::Y, 2.0, Vec3::new(-4.0, 0.5, 9.0));
    let (a, oa1) = comp_occ(
        &doc.apply(Command::NewComponent {
            parent: root,
            name: "A".into(),
            transform: ta1,
        })
        .unwrap(),
    );
    let CreatedId::Occurrence(oa2) = doc
        .apply(Command::AddOccurrence {
            parent: root,
            component: a,
            transform: ta2,
            name: None,
        })
        .unwrap()
        .created[0]
    else {
        panic!()
    };
    let (b, ob) = comp_occ(
        &doc.apply(Command::NewComponent {
            parent: a,
            name: "B".into(),
            transform: tb,
        })
        .unwrap(),
    );
    let (c, oc) = comp_occ(
        &doc.apply(Command::NewComponent {
            parent: b,
            name: "C".into(),
            transform: tc,
        })
        .unwrap(),
    );
    let f = doc
        .apply(Command::AddFeature {
            at: None,
            feature: bx(),
            context: c,
            name: None,
        })
        .unwrap();
    let CreatedId::Feature(f) = f.created[0] else {
        panic!()
    };
    doc.recompute_blocking(&jobs()).unwrap();
    Tree {
        doc,
        body: BodyId { origin: f, slot: 0 },
        a_occs: [oa1, oa2],
        b_occ: ob,
        c_occ: oc,
        ts: [ta1, ta2, tb, tc],
    }
}

#[test]
fn world_transform_and_instances() {
    let t = tree();
    let ev = t.doc.evaluated();
    let [ta1, ta2, tb, tc] = t.ts;
    let p1 = [t.a_occs[0], t.b_occ, t.c_occ];
    let p2 = [t.a_occs[1], t.b_occ, t.c_occ];
    let w1 = ta1 * tb * tc;
    let w2 = ta2 * tb * tc;
    assert!(max_diff(&ev.world_transform(&p1).unwrap(), &w1) < 1e-12);
    assert!(max_diff(&ev.world_transform(&p2).unwrap(), &w2) < 1e-12);
    assert!(max_diff(&ev.body_to_world(t.body, &p1).unwrap(), &w1) < 1e-12);
    assert!(max_diff(&ev.body_to_world(t.body, &p2).unwrap(), &w2) < 1e-12);
    // The empty path is the root; it is not the body's component.
    assert_eq!(ev.world_transform(&[]), Some(Transform::IDENTITY));
    assert!(ev.body_to_world(t.body, &[]).is_none());
    // A prefix ends in the wrong component; an invalid order is refused.
    assert!(ev.body_to_world(t.body, &p1[..2]).is_none());
    assert!(ev.world_transform(&[t.b_occ, t.a_occs[0]]).is_none());
    assert!(ev.world_transform(&[t.b_occ]).is_none());

    let inst = ev.instances(t.body);
    assert_eq!(inst.len(), 2);
    assert_eq!(inst[0].path, p1);
    assert_eq!(inst[1].path, p2);
    assert!(max_diff(&inst[0].world, &w1) < 1e-12);
    assert!(max_diff(&inst[1].world, &w2) < 1e-12);
    assert!(
        ev.instances(BodyId {
            origin: FeatureId(9999),
            slot: 0
        })
        .is_empty()
    );
    assert_eq!(ev.placements().len(), 4);

    // The design's own path queries agree.
    let d = t.doc.design();
    assert!(d.is_valid_path(d.root(), &p1));
    assert!(!d.is_valid_path(d.root(), &[t.b_occ]));
    let want = d.occurrence_paths(d.occurrence(t.c_occ).unwrap().component);
    assert_eq!(want, vec![p1.to_vec(), p2.to_vec()]);
    assert!(max_diff(&d.authored_world(&p1).unwrap(), &w1) < 1e-12);
    assert_eq!(
        d.features_in(d.occurrence(t.c_occ).unwrap().component)
            .count(),
        1
    );
}

// ---- 4. rigid / mirror validation ---------------------------------------------------------

#[test]
fn scale_is_refused_and_mirror_accepted() {
    let mut h = History::new(Design::new("rigid"));
    let root = h.design().root();
    let scaled = Transform::from_scale(Vec3::splat(2.0));
    let sheared = Transform::from_mat3(amg_core::Mat3::from_cols(
        Vec3::X,
        Vec3::new(0.5, 1.0, 0.0),
        Vec3::Z,
    ));
    let mirror = Transform::from_scale(Vec3::new(1.0, -1.0, 1.0));
    for bad in [scaled, sheared] {
        let before = json(h.design());
        let e = h
            .apply(Command::NewComponent {
                parent: root,
                name: "X".into(),
                transform: bad,
            })
            .unwrap_err();
        assert!(
            matches!(
                e,
                CommandError::Model(ModelError::InvalidValue {
                    field: "transform",
                    ..
                })
            ),
            "{e:?}"
        );
        assert_eq!(json(h.design()), before);
    }
    let (c, o) = comp_occ(
        &h.apply(Command::NewComponent {
            parent: root,
            name: "M".into(),
            transform: mirror,
        })
        .unwrap(),
    );
    for cmd in [
        Command::SetOccurrenceTransform {
            occ: o,
            transform: scaled,
        },
        Command::SetOccurrenceTransforms {
            moves: vec![(o, tr(1.0, 0.0, 0.0)), (o, scaled)],
        },
        Command::AddOccurrence {
            parent: root,
            component: c,
            transform: scaled,
            name: None,
        },
    ] {
        assert!(matches!(
            h.apply(cmd),
            Err(CommandError::Model(ModelError::InvalidValue { .. }))
        ));
    }
    // A failed batch changes nothing.
    assert_eq!(h.design().occurrence(o).unwrap().transform, mirror);
    h.apply(Command::SetOccurrenceTransforms {
        moves: vec![(o, mirror * tr(3.0, 0.0, 0.0))],
    })
    .unwrap();
}

// ---- 5. MoveOccurrence --------------------------------------------------------------------

#[test]
fn move_occurrence_keeps_world() {
    let mut h = History::new(Design::new("move"));
    let root = h.design().root();
    let (a, oa) = comp_occ(
        &h.apply(Command::NewComponent {
            parent: root,
            name: "A".into(),
            transform: rot(Vec3::Z, 0.4, Vec3::new(5.0, 1.0, 0.0)),
        })
        .unwrap(),
    );
    let (b, ob) = comp_occ(
        &h.apply(Command::NewComponent {
            parent: root,
            name: "B".into(),
            transform: rot(Vec3::X, 1.2, Vec3::new(-3.0, 4.0, 2.0)),
        })
        .unwrap(),
    );
    let (c, oc) = comp_occ(
        &h.apply(Command::NewComponent {
            parent: a,
            name: "C".into(),
            transform: rot(Vec3::Y, -0.8, Vec3::new(1.0, 2.0, 3.0)),
        })
        .unwrap(),
    );
    let world_before = h.design().authored_world(&[oa, oc]).unwrap();
    h.apply(Command::MoveOccurrence {
        occ: oc,
        parent: b,
        keep_world: true,
    })
    .unwrap();
    let d = h.design();
    d.validate().unwrap();
    assert_eq!(d.occurrence(oc).unwrap().parent, b);
    assert!(d.component(a).unwrap().children.is_empty());
    assert_eq!(d.component(b).unwrap().children, vec![oc]);
    assert!(max_diff(&d.authored_world(&[ob, oc]).unwrap(), &world_before) < 1e-12);

    // Without keep_world the local transform stays.
    let local = d.occurrence(oc).unwrap().transform;
    h.apply(Command::MoveOccurrence {
        occ: oc,
        parent: root,
        keep_world: false,
    })
    .unwrap();
    assert_eq!(h.design().occurrence(oc).unwrap().transform, local);

    // Cycles are refused, also into the component's own subtree.
    h.apply(Command::AddOccurrence {
        parent: a,
        component: c,
        transform: Transform::IDENTITY,
        name: None,
    })
    .unwrap();
    let e = h
        .apply(Command::MoveOccurrence {
            occ: oa,
            parent: c,
            keep_world: false,
        })
        .unwrap_err();
    assert!(
        matches!(e, CommandError::Model(ModelError::WouldCycle { .. })),
        "{e:?}"
    );

    // A parent placed twice makes keep_world ambiguous.
    h.apply(Command::AddOccurrence {
        parent: root,
        component: b,
        transform: Transform::IDENTITY,
        name: None,
    })
    .unwrap();
    let e = h
        .apply(Command::MoveOccurrence {
            occ: oc,
            parent: b,
            keep_world: true,
        })
        .unwrap_err();
    assert!(
        matches!(
            e,
            CommandError::Model(ModelError::InvalidValue {
                field: "keep_world",
                ..
            })
        ),
        "{e:?}"
    );
    h.apply(Command::MoveOccurrence {
        occ: oc,
        parent: b,
        keep_world: false,
    })
    .unwrap();
}

// ---- 6. events ----------------------------------------------------------------------------

struct Fx {
    doc: Document,
    js: JobSystem,
}

impl Fx {
    fn new() -> Fx {
        amg_doc::testing::link();
        let mut doc = Document::new("events");
        let root = doc.design().root();
        doc.apply(Command::AddFeature {
            at: None,
            feature: bx(),
            context: root,
            name: None,
        })
        .unwrap();
        let js = jobs();
        doc.recompute_blocking(&js).unwrap();
        doc.drain_events();
        Fx { doc, js }
    }
    fn authored(&mut self) -> Vec<DocEvent> {
        self.doc.drain_events()
    }
    fn recompute(&mut self) -> Vec<DocEvent> {
        self.doc.recompute_blocking(&self.js).unwrap();
        self.doc.drain_events()
    }
    fn rev(&self) -> amg_doc::Revision {
        self.doc.history().revision()
    }
}

#[test]
fn exact_event_sets() {
    let mut fx = Fx::new();
    let root = fx.doc.design().root();
    let (c, o1) = comp_occ(
        &fx.doc
            .apply(Command::NewComponent {
                parent: root,
                name: "C".into(),
                transform: Transform::IDENTITY,
            })
            .unwrap(),
    );
    assert_eq!(fx.authored(), vec![ComponentTreeChanged]);
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

    // AddOccurrence: tree change only.
    let CreatedId::Occurrence(o2) = fx
        .doc
        .apply(Command::AddOccurrence {
            parent: root,
            component: c,
            transform: tr(1.0, 0.0, 0.0),
            name: None,
        })
        .unwrap()
        .created[0]
    else {
        panic!()
    };
    assert_eq!(fx.authored(), vec![ComponentTreeChanged]);
    fx.recompute();

    // Grounding and naming are tree changes.
    fx.doc
        .apply(Command::SetOccurrenceGrounded {
            occ: o2,
            grounded: true,
        })
        .unwrap();
    assert_eq!(fx.authored(), vec![ComponentTreeChanged]);
    fx.doc
        .apply(Command::SetOccurrenceName {
            occ: o2,
            name: Some("pin".into()),
        })
        .unwrap();
    assert_eq!(fx.authored(), vec![ComponentTreeChanged]);
    fx.recompute();

    // A batch placement: one transform event per changed occurrence (authored), then again
    // when the new placements are swapped in.
    fx.doc
        .apply(Command::SetOccurrenceTransforms {
            moves: vec![(o2, tr(7.0, 0.0, 0.0)), (o1, tr(0.0, 0.0, 0.0))],
        })
        .unwrap();
    assert_eq!(fx.authored(), vec![OccurrenceTransformChanged(o2)]);
    let rev = fx.rev();
    assert_eq!(
        fx.recompute(),
        vec![
            RecomputeStarted(rev),
            OccurrenceTransformChanged(o2),
            RecomputeFinished {
                revision: rev,
                failed: 0
            }
        ]
    );

    // Without draining in between, the swap does not repeat the authored event.
    fx.doc
        .apply(Command::SetOccurrenceTransforms {
            moves: vec![(o2, tr(8.0, 0.0, 0.0))],
        })
        .unwrap();
    let rev = fx.rev();
    assert_eq!(
        fx.recompute(),
        vec![
            OccurrenceTransformChanged(o2),
            RecomputeStarted(rev),
            RecomputeFinished {
                revision: rev,
                failed: 0
            }
        ]
    );

    // MoveOccurrence: tree change (and a transform event when the local changes).
    let (c2, _o3) = comp_occ(
        &fx.doc
            .apply(Command::NewComponent {
                parent: root,
                name: "D".into(),
                transform: tr(0.0, 5.0, 0.0),
            })
            .unwrap(),
    );
    fx.authored();
    fx.recompute();
    fx.doc
        .apply(Command::MoveOccurrence {
            occ: o2,
            parent: c2,
            keep_world: false,
        })
        .unwrap();
    assert_eq!(fx.authored(), vec![ComponentTreeChanged]);
    fx.recompute();

    // Undo of the move: same authored set.
    assert!(fx.doc.undo());
    assert_eq!(fx.authored(), vec![ComponentTreeChanged]);
    fx.recompute();

    // DeleteOccurrences with cascade: tree + timeline + the removed body + health.
    let f = fx
        .doc
        .apply(Command::AddFeature {
            at: None,
            feature: bx(),
            context: c,
            name: None,
        })
        .unwrap();
    let CreatedId::Feature(f) = f.created[0] else {
        panic!()
    };
    fx.authored();
    fx.recompute();
    fx.doc
        .apply(Command::DeleteOccurrences { occs: vec![o1, o2] })
        .unwrap();
    assert_eq!(fx.authored(), vec![TimelineChanged, ComponentTreeChanged]);
    let rev = fx.rev();
    assert_eq!(
        fx.recompute(),
        vec![
            RecomputeStarted(rev),
            BodyRemoved(BodyId { origin: f, slot: 0 }),
            HealthChanged(f),
            RecomputeFinished {
                revision: rev,
                failed: 0
            }
        ]
    );
    // Undo restores both.
    assert!(fx.doc.undo());
    assert_eq!(fx.authored(), vec![TimelineChanged, ComponentTreeChanged]);
}

#[test]
fn active_component_is_session_state() {
    let mut fx = Fx::new();
    let root = fx.doc.design().root();
    assert_eq!(fx.doc.active_component(), root);
    let (c, o) = comp_occ(
        &fx.doc
            .apply(Command::NewComponent {
                parent: root,
                name: "C".into(),
                transform: Transform::IDENTITY,
            })
            .unwrap(),
    );
    fx.authored();
    fx.doc.set_active_component(c).unwrap();
    assert_eq!(fx.doc.active_component(), c);
    assert_eq!(fx.authored(), vec![DocEvent::ActiveComponentChanged(c)]);
    // Setting it again is silent; unknown components are refused.
    fx.doc.set_active_component(c).unwrap();
    assert_eq!(fx.authored(), vec![]);
    assert!(fx.doc.set_active_component(ComponentId(99_999)).is_err());
    assert_eq!(fx.doc.active_component(), c);
    // It is not undoable: undoing unrelated history keeps it, deleting it resets to the root.
    fx.doc
        .apply(Command::DeleteOccurrences { occs: vec![o] })
        .unwrap();
    assert_eq!(fx.doc.active_component(), root);
    assert_eq!(
        fx.authored(),
        vec![ComponentTreeChanged, DocEvent::ActiveComponentChanged(root)]
    );
    // Undo brings the component back but does not re-activate it.
    assert!(fx.doc.undo());
    assert_eq!(fx.doc.active_component(), root);
}

// ---- 7. moving an occurrence recomputes nothing -------------------------------------------

#[test]
fn moving_an_occurrence_recomputes_no_feature() {
    let mut t = tree();
    let js = jobs();
    let total = t.doc.design().timeline().len();
    assert!(total > 0);
    for cmd in [
        Command::SetOccurrenceTransform {
            occ: t.b_occ,
            transform: rot(Vec3::Z, 1.0, Vec3::new(3.0, 3.0, 3.0)),
        },
        Command::SetOccurrenceTransforms {
            moves: vec![
                (t.a_occs[0], tr(1.0, 2.0, 3.0)),
                (t.c_occ, tr(0.0, 0.0, 1.0)),
            ],
        },
        Command::SetOccurrenceGrounded {
            occ: t.a_occs[1],
            grounded: true,
        },
    ] {
        t.doc.apply(cmd).unwrap();
        t.doc.recompute_blocking(&js).unwrap();
        let run = t.doc.last_run().unwrap();
        assert!(run.computed.is_empty(), "computed {:?}", run.computed);
        assert_eq!(run.cached.len(), total);
    }
    // The solved placements follow.
    let p = [t.a_occs[0], t.b_occ, t.c_occ];
    let want = t.doc.design().authored_world(&p).unwrap();
    assert!(max_diff(&t.doc.evaluated().world_transform(&p).unwrap(), &want) < 1e-12);
}
