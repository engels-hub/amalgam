//! SOL-09: `solid.move` (free, translate, rotate, point to point; copy), `solid.align`,
//! `solid.offset_face`, `solid.replace_face`, `solid.delete_face` (heal / remove) and the
//! press-pull dispatch; one test (or test group) per Done-when item, see the test names.
//! Geometry needs the `occt` feature (AMG_OCCT_DIR); the dialog, dispatch and data tests run
//! everywhere.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "tests"
)]

mod solid_common;

use amg_doc::stub::ProfileKey;
use amg_doc::{
    BodyId, Dependency, Document, EntityRef, Feature, FeatureBox, FeatureId, SketchId, TopoKind,
    TopoRef,
};
use amg_features::construct::{OriginEntity, RefInput};
use amg_features::dialog::{self, DialogInit};
use amg_features::solid::{Align, DeleteFace, Move, MoveSpec, OffsetFace, ReplaceFace};
#[allow(unused_imports, reason = "the occt tests use it through `super::*`")]
use solid_common::*;

const KINDS: [&str; 5] = [
    "solid.move",
    "solid.align",
    "solid.offset_face",
    "solid.replace_face",
    "solid.delete_face",
];

fn body(origin: u64) -> BodyId {
    BodyId {
        origin: FeatureId(origin),
        slot: 0,
    }
}

fn topo(kind: TopoKind, key: u32) -> TopoRef {
    TopoRef {
        body: body(1),
        kind,
        name: amg_doc::naming::TopoName::default(),
        fingerprint: None,
        key,
    }
}

// ---- dialog and data (every build) ------------------------------------------------------------

#[test]
fn the_five_kinds_register_dialogs_that_round_trip() {
    let doc = Document::new("sol09");
    for pre in [
        vec![],
        vec![EntityRef::Body(body(3)), EntityRef::Body(body(4))],
        vec![EntityRef::Topo(topo(TopoKind::Face, 1))],
    ] {
        let init = DialogInit {
            preselection: &pre,
            design: doc.design(),
            evaluated: doc.evaluated(),
        };
        for kind in KINDS {
            let reg = dialog::registration(kind).unwrap_or_else(|| panic!("{kind}"));
            let v = (reg.defaults)(&init);
            let f = (reg.build)(&v, None).unwrap();
            f.validate().unwrap();
            assert_eq!((reg.read)(f.get(), doc.design()).unwrap(), v, "{kind}");
            assert_eq!(reg.command.group, "modify");
            assert_eq!((reg.schema)().kind, kind);
            // Unique labels; option labels distinct from field labels (the UI-05 harness finds
            // widgets by label).
            let fields = &(reg.schema)().fields;
            let mut labels: Vec<&str> = fields.iter().map(|f| f.label.as_str()).collect();
            labels.sort_unstable();
            labels.dedup();
            assert_eq!(labels.len(), fields.len(), "{kind}: duplicate label");
            for field in fields {
                if let dialog::FieldKind::Choice { options, .. } = &field.kind {
                    for o in options {
                        assert!(!labels.contains(&o.label.as_str()), "{kind}: {}", o.label);
                    }
                }
            }
        }
    }
}

#[test]
fn preselections_fill_bodies_and_faces() {
    let doc = Document::new("pre");
    let pre = [
        EntityRef::Body(body(3)),
        EntityRef::Topo(topo(TopoKind::Face, 7)),
    ];
    let init = DialogInit {
        preselection: &pre,
        design: doc.design(),
        evaluated: doc.evaluated(),
    };
    let reg = dialog::registration("solid.move").unwrap();
    let f = (reg.build)(&(reg.defaults)(&init), None).unwrap();
    let m = f.get().as_any().downcast_ref::<Move>().unwrap();
    assert_eq!(m.bodies, vec![body(3)]);
    assert!(matches!(m.spec, MoveSpec::Translate(_)));
    let reg = dialog::registration("solid.align").unwrap();
    let f = (reg.build)(&(reg.defaults)(&init), None).unwrap();
    let a = f.get().as_any().downcast_ref::<Align>().unwrap();
    assert_eq!((a.bodies.clone(), a.from.clone()), (vec![body(3)], None));
    for kind in [
        "solid.offset_face",
        "solid.replace_face",
        "solid.delete_face",
    ] {
        let reg = dialog::registration(kind).unwrap();
        let f = (reg.build)(&(reg.defaults)(&init), None).unwrap();
        let faces = match f.get().as_any() {
            x if x.is::<OffsetFace>() => x.downcast_ref::<OffsetFace>().unwrap().faces.clone(),
            x if x.is::<ReplaceFace>() => x.downcast_ref::<ReplaceFace>().unwrap().faces.clone(),
            x => x.downcast_ref::<DeleteFace>().unwrap().faces.clone(),
        };
        assert_eq!(faces, vec![topo(TopoKind::Face, 7)], "{kind}");
    }
}

/// Done-when 6 (registration level): the press-pull dispatch maps a face, an edge and a profile
/// selection to Offset Face, Fillet and Extrude, whose dialogs exist.
#[test]
fn dw6_press_pull_dispatches_face_edge_and_profile() {
    let doc = Document::new("pp");
    let d = dialog::dispatches()
        .find(|d| d.command_id == "solid.press_pull")
        .expect("press-pull is registered");
    assert!(d.command.pinned);
    let profile = EntityRef::Profile {
        sketch: SketchId(FeatureId(2)),
        profile: ProfileKey(0),
        fingerprint: None,
    };
    for (sel, want) in [
        (
            vec![EntityRef::Topo(topo(TopoKind::Face, 1))],
            Some("solid.offset_face"),
        ),
        (
            vec![EntityRef::Topo(topo(TopoKind::Edge, 1))],
            Some("solid.fillet"),
        ),
        (
            vec![profile.clone(), profile.clone()],
            Some("solid.extrude"),
        ),
        (
            vec![EntityRef::Topo(topo(TopoKind::Face, 1)), profile],
            None,
        ),
        (vec![EntityRef::Body(body(1))], None),
        (vec![], None),
    ] {
        let got = (d.dispatch)(&sel, doc.design());
        assert_eq!(got, want, "{sel:?}");
        if let Some(k) = got {
            assert!(dialog::registration(k).is_some(), "{k} has a dialog");
        }
    }
}

#[test]
fn inputs_and_expressions_are_listed() {
    let axis = RefInput::Ref(EntityRef::Topo(topo(TopoKind::Edge, 2)));
    let m = Move::rotate(vec![body(1), body(2)], axis.clone(), "30 deg");
    assert_eq!(
        m.inputs(),
        vec![
            Dependency::Body(body(1)),
            Dependency::Body(body(2)),
            Dependency::Entity(EntityRef::Topo(topo(TopoKind::Edge, 2))),
        ]
    );
    let mut m2 = m.clone();
    assert_eq!(m2.exprs().len(), 1);
    assert_eq!(m2.exprs_mut()[0].1, amg_doc::Dim::ANGLE);
    let t = Move::translate(vec![body(1)], ["1 mm", "2 mm", "3 mm"]);
    assert_eq!(t.exprs().len(), 3);
    let a = Align::new(
        vec![body(1)],
        RefInput::Ref(EntityRef::Topo(topo(TopoKind::Face, 1))),
        RefInput::Origin(OriginEntity::XYPlane),
    );
    assert_eq!(
        a.inputs().len(),
        2,
        "body and from face; origin planes have none"
    );
    assert_eq!(a.exprs().len(), 1);
    let o = OffsetFace::new(vec![topo(TopoKind::Face, 1)], "2 mm");
    assert_eq!(o.exprs().len(), 1);
    assert_eq!(o.inputs()[0], Dependency::Body(body(1)));
    // Unpicked selections validate (the dialog's `min: 1` keeps OK off) and fail in compute.
    Move::translate(vec![], ["0 mm", "0 mm", "0 mm"])
        .validate()
        .unwrap();
    DeleteFace::new(vec![], false).validate().unwrap();
    // A non-affine free matrix is refused by validate.
    let mut bad = amg_features::solid::moves::IDENTITY_MATRIX;
    bad[3][0] = 2.0;
    assert!(
        Move {
            bodies: vec![body(1)],
            spec: MoveSpec::Free { matrix: bad },
            copy: false,
        }
        .validate()
        .is_err()
    );
    // JSON round trip.
    let fb = FeatureBox::new(m);
    let back: FeatureBox = serde_json::from_str(&serde_json::to_string(&fb).unwrap()).unwrap();
    assert_eq!(back, fb);
}

#[cfg(feature = "occt")]
#[path = "solid_moves/occt.rs"]
mod occt;
