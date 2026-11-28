//! SOL-24: component operations. "New component" from the extrude dialog (one component, one
//! body, one undo entry), component patterns and mirrors of occurrences (`AddOccurrence` per
//! instance, one undo entry) and body operations whose participant is in a sub-component
//! (assembly context). One test (or test group) per Done-when item, see the test names. The
//! geometry checks need the `occt` feature (AMG_OCCT_DIR); the command, transform and undo
//! checks run everywhere.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "tests"
)]

mod solid_common;

use std::f64::consts::PI;

use amg_core::{Transform, Vec3};
use amg_doc::{Command, ComponentId, CreatedId, Document, EntityRef, Expr, ExprSrc, OccurrenceId};
use amg_features::construct::{OriginEntity, RefInput};
use amg_features::dialog::{self, DialogInit, FieldValue, FieldValues};
use amg_features::solid::component::{
    ComponentOpError, mirror_occurrences, occurrence_pattern_transforms, pattern_occurrences,
};
use amg_features::solid::{
    AngleSpan, BodyOp, ComputeType, Mirror, Pattern, PatternDir, PatternKind, PatternObjects,
    Spacing,
};
#[allow(unused_imports, reason = "the occt tests use it through `super::*`")]
use solid_common::*;

fn ex(s: &str) -> Expr {
    Expr::Inline(ExprSrc(s.to_owned()))
}

#[track_caller]
fn assert_transform(got: &Transform, want: &Transform, tol: f64) -> f64 {
    let worst = got
        .to_cols_array()
        .iter()
        .zip(want.to_cols_array())
        .map(|(a, b)| (a - b).abs())
        .fold(0.0, f64::max);
    assert!(worst <= tol, "got {got:?}, want {want:?} (worst {worst:e})");
    worst
}

/// A new component in `parent` placed at `t`, and its occurrence.
fn new_component(
    doc: &mut Document,
    parent: ComponentId,
    name: &str,
    t: Transform,
) -> (ComponentId, OccurrenceId) {
    let o = doc
        .apply(Command::NewComponent {
            parent,
            name: name.into(),
            transform: t,
        })
        .unwrap();
    let c = o
        .created
        .iter()
        .find_map(|c| match c {
            CreatedId::Component(c) => Some(*c),
            _ => None,
        })
        .unwrap();
    let occ = o
        .created
        .iter()
        .find_map(|c| match c {
            CreatedId::Occurrence(o) => Some(*o),
            _ => None,
        })
        .unwrap();
    (c, occ)
}

/// Applies `cmds` in one transaction (one undo entry).
fn commit(doc: &mut Document, label: &str, cmds: Vec<Command>) -> Vec<CreatedId> {
    let mut created = Vec::new();
    let mut t = doc.transaction(label);
    for c in cmds {
        created.extend(t.apply(c).unwrap().created);
    }
    t.commit();
    created
}

fn occurrences_of(doc: &Document, c: ComponentId) -> Vec<(OccurrenceId, Transform)> {
    doc.design()
        .occurrences()
        .filter(|(_, o)| o.component == c)
        .map(|(id, o)| (id, o.transform))
        .collect()
}

/// A rotation of `deg` about Z after a translation (the seed placement of the pattern tests).
fn seed_placement() -> Transform {
    Transform::from_translation(Vec3::new(40.0, 0.0, 5.0))
        * Transform::from_rotation_z(15f64.to_radians())
}

fn circular_of(occ: OccurrenceId, count: &str) -> Pattern {
    Pattern {
        objects: PatternObjects::Occurrences(vec![occ]),
        kind: PatternKind::Circular {
            axis: RefInput::Origin(OriginEntity::ZAxis),
            count: ex(count),
            span: AngleSpan::Full,
            symmetric: false,
        },
        suppressed: Vec::new(),
        compute: ComputeType::Identical,
        target: None,
    }
}

// ---- Done-when 1: "New component" (commands and undo; geometry in occt.rs) ----------------------

fn extrude_values(doc: &Document, sketch_profiles: &[EntityRef], op: &'static str) -> FieldValues {
    let reg = dialog::registration("solid.extrude").unwrap();
    (reg.defaults)(&DialogInit {
        preselection: sketch_profiles,
        design: doc.design(),
        evaluated: doc.evaluated(),
    })
    .with("operation", FieldValue::Choice(op))
}

#[test]
fn dw1_new_component_is_an_operation_option_that_hides_the_objects() {
    for kind in ["solid.extrude", "solid.revolve"] {
        let reg = dialog::registration(kind).unwrap();
        let schema = (reg.schema)();
        let op = schema.field(dialog::FieldKey("operation")).unwrap();
        let dialog::FieldKind::Choice { options, .. } = &op.kind else {
            panic!("{kind}: operation is a choice");
        };
        assert!(
            options.iter().any(|o| o.id == "new_component"),
            "{kind}: has New component"
        );
        let bodies = schema.field(dialog::FieldKey("bodies")).unwrap();
        let cond = bodies.visible_if.as_ref().unwrap();
        let v = |op| FieldValues::default().with("operation", FieldValue::Choice(op));
        assert!(!cond.holds(&v("new_component")), "{kind}");
        assert!(!cond.holds(&v("new_body")), "{kind}");
        assert!(cond.holds(&v("cut")), "{kind}");
        let f = reg.new_component.expect("registered");
        assert_eq!(f(&v("new_component")), Some(String::new()), "{kind}");
        assert_eq!(f(&v("new_body")), None, "{kind}");
        assert_eq!(f(&v("join")), None, "{kind}");
    }
}

#[test]
fn dw1_extrude_with_new_component_commits_one_component_in_one_undo_entry() {
    let mut doc = Document::new("sol24-dw1");
    let root = doc.design().root();
    let s = add(
        &mut doc,
        amg_doc::FeatureBox::new(rect_sketch(&[[0.0, 0.0, 10.0, 20.0]])),
    );
    let profs = profiles(&doc, s);
    let reg = dialog::registration("solid.extrude").unwrap();
    let v = extrude_values(&doc, &profs, "new_component");
    let cmds = dialog::new_feature_commands(reg, &v, doc.design(), root).unwrap();
    assert_eq!(cmds.len(), 2);
    assert!(
        matches!(&cmds[0], Command::NewComponent { parent, name, transform }
        if *parent == root && name == "Component" && *transform == Transform::IDENTITY)
    );
    let components = doc.design().components().count();
    let undo_before = doc.history().stack().applied();
    let created = commit(&mut doc, "Extrude", cmds);
    doc.recompute_blocking(jobs()).unwrap();
    assert_eq!(
        doc.history().stack().applied(),
        undo_before + 1,
        "one undo entry"
    );
    assert_eq!(doc.design().components().count(), components + 1);
    let c = created
        .iter()
        .find_map(|x| match x {
            CreatedId::Component(c) => Some(*c),
            _ => None,
        })
        .unwrap();
    let f = created
        .iter()
        .find_map(|x| match x {
            CreatedId::Feature(f) => Some(*f),
            _ => None,
        })
        .unwrap();
    assert_eq!(doc.design().timeline().node(f).unwrap().1.context, c);
    // A plain new body stays one AddFeature in the active component.
    let plain = dialog::new_feature_commands(
        reg,
        &extrude_values(&doc, &profs, "new_body"),
        doc.design(),
        root,
    )
    .unwrap();
    assert!(matches!(&plain[..], [Command::AddFeature { context, .. }] if *context == root));
    // A second new component gets the next free name.
    let again = dialog::new_feature_commands(reg, &v, doc.design(), root).unwrap();
    assert!(matches!(&again[0], Command::NewComponent { name, .. } if name == "Component1"));
    // One undo removes the component and the feature.
    assert!(doc.undo());
    doc.recompute_blocking(jobs()).unwrap();
    assert_eq!(doc.design().components().count(), components);
    assert!(doc.design().component(c).is_none());
    assert!(doc.design().timeline().node(f).is_none());
}

// ---- Done-when 2: component pattern and mirror --------------------------------------------------

#[test]
fn dw2_circular_pattern_of_an_occurrence_creates_six_occurrences() {
    let mut doc = Document::new("sol24-dw2");
    let root = doc.design().root();
    let (c, occ) = new_component(&mut doc, root, "Blade", seed_placement());
    doc.recompute_blocking(jobs()).unwrap();
    let p = circular_of(occ, "6");
    let cmds = pattern_occurrences(doc.design(), doc.evaluated(), &p, root).unwrap();
    assert_eq!(cmds.len(), 5, "count 6 includes the seed");
    let undo_before = doc.history().stack().applied();
    commit(&mut doc, "Pattern", cmds);
    doc.recompute_blocking(jobs()).unwrap();
    assert_eq!(
        doc.history().stack().applied(),
        undo_before + 1,
        "one undo entry"
    );
    let occs = occurrences_of(&doc, c);
    assert_eq!(occs.len(), 6, "six occurrences of the component");
    let mut worst: f64 = 0.0;
    for k in 0..6u32 {
        let want = Transform::from_rotation_z(f64::from(k) * PI / 3.0) * seed_placement();
        let (id, got) = occs
            .iter()
            .find(|(_, t)| {
                t.to_cols_array()
                    .iter()
                    .zip(want.to_cols_array())
                    .all(|(a, b)| (a - b).abs() < 1e-9)
            })
            .unwrap_or_else(|| panic!("instance {k} at {want:?}"));
        worst = worst.max(assert_transform(got, &want, 1e-12));
        // The evaluated world placement agrees.
        let w = doc.evaluated().world_transform(&[*id]).unwrap();
        assert_transform(&w, &want, 1e-12);
        let occ = doc.design().occurrence(*id).unwrap();
        assert_eq!(occ.parent, root);
    }
    println!("dw2 circular: 6 occurrences, worst transform deviation {worst:e}");
    // Undo removes the five instances in one step.
    assert!(doc.undo());
    assert_eq!(occurrences_of(&doc, c).len(), 1);
}

#[test]
fn dw2_rectangular_pattern_with_a_suppressed_instance() {
    let mut doc = Document::new("sol24-dw2-rect");
    let root = doc.design().root();
    let (c, occ) = new_component(&mut doc, root, "Peg", seed_placement());
    doc.recompute_blocking(jobs()).unwrap();
    let d = |axis, n: &str, s: &str| PatternDir {
        dir: RefInput::Origin(axis),
        count: ex(n),
        spacing: Spacing::Spacing(ex(s)),
        flip: false,
        symmetric: false,
    };
    let mut p = Pattern {
        objects: PatternObjects::Occurrences(vec![occ]),
        kind: PatternKind::Rectangular {
            d1: d(OriginEntity::XAxis, "3", "20 mm"),
            d2: Some(d(OriginEntity::YAxis, "2", "15 mm")),
        },
        suppressed: vec![1 | (1 << 16)],
        compute: ComputeType::Identical,
        target: None,
    };
    let inst = occurrence_pattern_transforms(doc.design(), doc.evaluated(), &p, root).unwrap();
    assert_eq!(inst.len(), 4, "3×2 minus the seed and one suppressed");
    let cmds = pattern_occurrences(doc.design(), doc.evaluated(), &p, root).unwrap();
    commit(&mut doc, "Pattern", cmds);
    let occs = occurrences_of(&doc, c);
    assert_eq!(occs.len(), 5);
    for (k, t) in &inst {
        let (i, j) = (f64::from(k & 0xffff), f64::from(k >> 16));
        let want =
            Transform::from_translation(Vec3::new(20.0 * i, 15.0 * j, 0.0)) * seed_placement();
        assert_transform(
            t,
            &Transform::from_translation(Vec3::new(20.0 * i, 15.0 * j, 0.0)),
            1e-12,
        );
        assert!(
            occs.iter().any(|(_, g)| g.abs_diff_eq(want, 1e-12)),
            "instance {k}"
        );
    }
    // A pattern with no instance besides the seed builds no commands.
    p.kind = PatternKind::Rectangular {
        d1: d(OriginEntity::XAxis, "1", "20 mm"),
        d2: None,
    };
    assert!(
        pattern_occurrences(doc.design(), doc.evaluated(), &p, root)
            .unwrap()
            .is_empty()
    );
}

#[test]
fn dw2_mirror_creates_a_mirrored_occurrence() {
    let mut doc = Document::new("sol24-dw2-mirror");
    let root = doc.design().root();
    let (c, occ) = new_component(&mut doc, root, "Bracket", seed_placement());
    doc.recompute_blocking(jobs()).unwrap();
    let m = Mirror {
        objects: PatternObjects::Occurrences(vec![occ]),
        plane: RefInput::Origin(OriginEntity::YZPlane),
        op: BodyOp::NewBody,
        target: None,
    };
    let cmds = mirror_occurrences(doc.design(), doc.evaluated(), &m, root).unwrap();
    assert_eq!(cmds.len(), 1);
    let undo_before = doc.history().stack().applied();
    let created = commit(&mut doc, "Mirror", cmds);
    assert_eq!(doc.history().stack().applied(), undo_before + 1);
    let new = created
        .iter()
        .find_map(|x| match x {
            CreatedId::Occurrence(o) => Some(*o),
            _ => None,
        })
        .unwrap();
    let t = doc.design().occurrence(new).unwrap().transform;
    let flip_x = Transform::from_mat3(amg_core::Mat3::from_diagonal(Vec3::new(-1.0, 1.0, 1.0)));
    let worst = assert_transform(&t, &(flip_x * seed_placement()), 1e-12);
    assert!(
        (t.matrix3.determinant() + 1.0).abs() < 1e-12,
        "a mirrored placement"
    );
    assert_eq!(doc.design().occurrence(new).unwrap().component, c);
    println!(
        "dw2 mirror: det {}, worst deviation {worst:e}",
        t.matrix3.determinant()
    );
    assert_eq!(occurrences_of(&doc, c).len(), 2);
}

#[test]
fn dw2_component_operations_reject_bad_selections() {
    let mut doc = Document::new("sol24-dw2-errors");
    let root = doc.design().root();
    let (outer, outer_occ) = new_component(&mut doc, root, "Outer", Transform::IDENTITY);
    let (_, inner_occ) = new_component(&mut doc, outer, "Inner", Transform::IDENTITY);
    doc.recompute_blocking(jobs()).unwrap();
    let (d, e) = (doc.design(), doc.evaluated());
    // Not placed in the active component.
    assert!(matches!(
        pattern_occurrences(d, e, &circular_of(inner_occ, "4"), root),
        Err(ComponentOpError::NotInContext { .. })
    ));
    // ... but fine where it is placed.
    assert_eq!(
        pattern_occurrences(d, e, &circular_of(inner_occ, "4"), outer)
            .unwrap()
            .len(),
        3
    );
    let mut p = circular_of(outer_occ, "4");
    p.objects = PatternObjects::Occurrences(Vec::new());
    assert!(matches!(
        pattern_occurrences(d, e, &p, root),
        Err(ComponentOpError::NoOccurrences)
    ));
    p.objects = PatternObjects::Bodies(Vec::new());
    assert!(matches!(
        pattern_occurrences(d, e, &p, root),
        Err(ComponentOpError::NotOccurrences)
    ));
    p.objects = PatternObjects::Occurrences(vec![OccurrenceId(9999)]);
    assert!(matches!(
        pattern_occurrences(d, e, &p, root),
        Err(ComponentOpError::NoOccurrence(_))
    ));
    let bad = circular_of(outer_occ, "2.5");
    assert!(matches!(
        pattern_occurrences(d, e, &bad, root),
        Err(ComponentOpError::Feature(_))
    ));
    // Stored as a timeline feature, a component pattern is a clean error, not a panic.
    let f = add(
        &mut doc,
        amg_doc::FeatureBox::new(circular_of(outer_occ, "3")),
    );
    assert!(matches!(health(&doc, f), amg_doc::Health::Error(_)));
}

#[cfg(feature = "occt")]
#[path = "solid_component/occt.rs"]
mod occt;
