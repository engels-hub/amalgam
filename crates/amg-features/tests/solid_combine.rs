//! SOL-07: `solid.combine` (join / cut / intersect, keep tools), `solid.split_body` (plane,
//! face, body; extend tool) and `solid.split_face` (surface tool; sketch curves along a
//! vector); one test (or test group) per Done-when item, see the test names. Geometry needs the
//! `occt` feature (AMG_OCCT_DIR); the dialog and data tests run everywhere.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "tests"
)]

mod solid_common;

use amg_doc::{BodyId, Dependency, Document, EntityRef, Feature, FeatureBox, FeatureId};
use amg_features::construct::{OriginEntity, RefInput};
use amg_features::dialog::{self, DialogInit};
use amg_features::solid::{BodyOp, Combine, SplitBody, SplitFace, SplitFaceTool};
#[allow(unused_imports, reason = "the occt tests use it through `super::*`")]
use solid_common::*;

fn body(origin: u64) -> BodyId {
    BodyId {
        origin: FeatureId(origin),
        slot: 0,
    }
}

// ---- dialog and data (every build) ------------------------------------------------------------

#[test]
fn the_three_kinds_register_dialogs_that_round_trip() {
    let doc = Document::new("sol07");
    for pre in [
        vec![],
        vec![EntityRef::Body(body(3)), EntityRef::Body(body(4))],
    ] {
        let init = DialogInit {
            preselection: &pre,
            design: doc.design(),
            evaluated: doc.evaluated(),
        };
        for kind in ["solid.combine", "solid.split_body", "solid.split_face"] {
            let reg = dialog::registration(kind).unwrap_or_else(|| panic!("{kind}"));
            let v = (reg.defaults)(&init);
            let f = (reg.build)(&v, None).unwrap();
            f.validate().unwrap();
            assert_eq!((reg.read)(f.get(), doc.design()).unwrap(), v, "{kind}");
            assert_eq!(reg.command.group, "modify");
            // Conditional fields are hidden for the defaults (UI-05 harness rule), except the
            // surface-tool fields of a split face, whose default tool type is "surface".
            for field in &(reg.schema)().fields {
                if let Some(c) = &field.visible_if {
                    let shown = kind == "solid.split_face" && field.label == "Splitting tool";
                    assert_eq!(c.holds(&v), shown, "{kind}: `{}`", field.label);
                }
            }
        }
    }
}

#[test]
fn preselected_bodies_fill_target_tools_and_split_body() {
    let doc = Document::new("pre");
    let pre = [EntityRef::Body(body(3)), EntityRef::Body(body(4))];
    let init = DialogInit {
        preselection: &pre,
        design: doc.design(),
        evaluated: doc.evaluated(),
    };
    let reg = dialog::registration("solid.combine").unwrap();
    let f = (reg.build)(&(reg.defaults)(&init), None).unwrap();
    let c = f.get().as_any().downcast_ref::<Combine>().unwrap();
    assert_eq!(c, &Combine::new(body(3), vec![body(4)], BodyOp::Join));
    let reg = dialog::registration("solid.split_body").unwrap();
    let f = (reg.build)(&(reg.defaults)(&init), None).unwrap();
    let s = f.get().as_any().downcast_ref::<SplitBody>().unwrap();
    assert_eq!(s.body, Some(body(3)));
}

#[test]
fn inputs_list_every_body_the_features_read() {
    let c = Combine::new(body(1), vec![body(2), body(3)], BodyOp::Intersect);
    assert_eq!(
        c.inputs(),
        [1, 2, 3].map(|o| Dependency::Body(body(o))).to_vec()
    );
    let s = SplitBody::new(body(1), RefInput::Ref(EntityRef::Body(body(2))));
    assert_eq!(
        s.inputs(),
        vec![Dependency::Body(body(1)), Dependency::Body(body(2))]
    );
    let f = SplitFace::by_surface(
        Vec::new(),
        RefInput::Ref(EntityRef::Construction(amg_doc::ConstructionId(FeatureId(
            9,
        )))),
    );
    assert_eq!(
        f.inputs(),
        vec![Dependency::Construction(amg_doc::ConstructionId(
            FeatureId(9)
        ))]
    );
}

#[test]
fn corpus_json_of_tnp21_and_tnp23_deserializes() {
    // TNP-23's combine and TNP-21's split body, with the `$` bindings filled in.
    let comb: FeatureBox = serde_json::from_str(
        r#"{"kind":"solid.combine","v":1,"data":{"target":{"origin":2,"slot":0},"tools":[{"origin":4,"slot":0}],"op":"Join","keep_tools":false}}"#,
    )
    .unwrap();
    let c = comb.get().as_any().downcast_ref::<Combine>().unwrap();
    assert_eq!(c, &Combine::new(body(2), vec![body(4)], BodyOp::Join));
    let split: FeatureBox = serde_json::from_str(
        r#"{"kind":"solid.split_body","v":1,"data":{"body":{"origin":2,"slot":0},"tool":{"Origin":"YZPlane"}}}"#,
    )
    .unwrap();
    let s = split.get().as_any().downcast_ref::<SplitBody>().unwrap();
    assert_eq!(
        s,
        &SplitBody::new(body(2), RefInput::Origin(OriginEntity::YZPlane))
    );
    let back: FeatureBox = serde_json::from_str(&serde_json::to_string(&split).unwrap()).unwrap();
    assert_eq!(back, split);
}

#[test]
fn unpicked_and_invalid_selections_validate_or_fail_cleanly() {
    let c = Combine {
        target: None,
        tools: Vec::new(),
        op: BodyOp::Cut,
        keep_tools: false,
    };
    c.validate().unwrap();
    let mut bad = c.clone();
    bad.op = BodyOp::NewBody;
    assert!(bad.validate().is_err());
    assert!(
        SplitBody::new(body(1), RefInput::Ref(EntityRef::Body(body(1))))
            .validate()
            .is_err()
    );
    let f = SplitFace {
        faces: Vec::new(),
        tool: SplitFaceTool::CurvesAlong {
            curves: Vec::new(),
            dir: None,
        },
        extend_tool: false,
    };
    f.validate().unwrap();
}

#[cfg(feature = "occt")]
#[path = "solid_combine/occt.rs"]
mod occt;
