//! SOL-06: `solid.shell` (remove faces, hollow, inside / outside / both, tangent chain),
//! `solid.draft` (one side, two sides, symmetric), `solid.scale` (uniform, non-uniform) and
//! `solid.thicken`; one test (or test group) per Done-when item, see the test names. Geometry
//! needs the `occt` feature (AMG_OCCT_DIR); the dialog and data tests run everywhere.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "tests"
)]

mod solid_common;

use amg_doc::{
    BodyId, ConstructionId, Dependency, Document, EntityRef, Feature, FeatureBox, FeatureId,
    TopoKind, TopoName, TopoRef,
};
use amg_features::construct::{OriginEntity, RefInput};
use amg_features::dialog::{self, DialogInit};
use amg_features::solid::{
    BodyOp, Draft, DraftSides, OpTarget, Scale, ScaleKind, Shell, ShellDirection, Thicken,
};
#[allow(unused_imports, reason = "the occt tests use it through `super::*`")]
use solid_common::*;

const KINDS: [&str; 4] = ["solid.shell", "solid.draft", "solid.scale", "solid.thicken"];

fn body(origin: u64) -> BodyId {
    BodyId {
        origin: FeatureId(origin),
        slot: 0,
    }
}

fn face(origin: u64, key: u32) -> TopoRef {
    TopoRef {
        body: body(origin),
        kind: TopoKind::Face,
        name: TopoName::new(Vec::new()),
        fingerprint: None,
        key,
    }
}

// ---- dialog and data (every build) ------------------------------------------------------------

#[test]
fn the_four_kinds_register_dialogs_that_round_trip() {
    let doc = Document::new("sol06");
    for pre in [
        vec![],
        vec![EntityRef::Topo(face(3, 1)), EntityRef::Body(body(4))],
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
            assert_eq!(f.kind(), kind);
            // Labels are unique within a dialog.
            let mut labels: Vec<&str> = (reg.schema)()
                .fields
                .iter()
                .map(|f| f.label.as_str())
                .collect();
            let n = labels.len();
            labels.sort_unstable();
            labels.dedup();
            assert_eq!(labels.len(), n, "{kind}: duplicate label");
        }
    }
}

#[test]
fn preselection_fills_faces_and_bodies() {
    let doc = Document::new("pre");
    let pre = [EntityRef::Topo(face(3, 1)), EntityRef::Body(body(4))];
    let init = DialogInit {
        preselection: &pre,
        design: doc.design(),
        evaluated: doc.evaluated(),
    };
    let build = |kind: &str| {
        let reg = dialog::registration(kind).unwrap();
        (reg.build)(&(reg.defaults)(&init), None).unwrap()
    };
    let s = build("solid.shell");
    let s = s.get().as_any().downcast_ref::<Shell>().unwrap();
    assert_eq!(
        (s.remove.clone(), s.bodies.clone()),
        (vec![face(3, 1)], vec![body(4)])
    );
    let d = build("solid.draft");
    assert_eq!(
        d.get().as_any().downcast_ref::<Draft>().unwrap().faces,
        vec![face(3, 1)]
    );
    let sc = build("solid.scale");
    assert_eq!(
        sc.get().as_any().downcast_ref::<Scale>().unwrap().bodies,
        vec![body(4)]
    );
    let t = build("solid.thicken");
    assert_eq!(
        t.get().as_any().downcast_ref::<Thicken>().unwrap().faces,
        vec![face(3, 1)]
    );
}

#[test]
fn inputs_list_every_body_the_features_read() {
    let mut s = Shell::inside(vec![face(1, 1), face(2, 2)], "1 mm");
    s.bodies = vec![body(3), body(1)];
    let d = s.inputs();
    for b in [1, 2, 3] {
        assert_eq!(
            d.iter()
                .filter(|x| **x == Dependency::Body(body(b)))
                .count(),
            1,
            "body {b}"
        );
    }
    assert_eq!(d.len(), 5, "3 bodies + 2 faces");
    let plane = RefInput::Ref(EntityRef::Construction(ConstructionId(FeatureId(9))));
    let dr = Draft::one_side(plane, vec![face(1, 1)], "3 deg");
    assert_eq!(
        dr.inputs(),
        vec![
            Dependency::Body(body(1)),
            Dependency::Entity(EntityRef::Topo(face(1, 1))),
            Dependency::Construction(ConstructionId(FeatureId(9))),
        ]
    );
    let sc = Scale::uniform(vec![body(1), body(2), body(1)], "2");
    assert_eq!(
        sc.inputs(),
        vec![Dependency::Body(body(1)), Dependency::Body(body(2))]
    );
    let mut t = Thicken::new_body(vec![face(1, 1)], "1 mm");
    t.target = OpTarget {
        op: BodyOp::Cut,
        participants: vec![body(5)],
    };
    let d = t.inputs();
    assert!(d.contains(&Dependency::Body(body(1))) && d.contains(&Dependency::Body(body(5))));
}

#[test]
fn feature_json_round_trips_and_exprs_have_dimensions() {
    let d = Draft {
        sides: DraftSides::TwoSides {
            angle2: amg_doc::Expr::Inline(amg_doc::ExprSrc("5 deg".into())),
        },
        ..Draft::default()
    };
    let mut s = Shell::inside(vec![face(1, 1)], "1 mm");
    s.direction = ShellDirection::Both;
    let all: Vec<FeatureBox> = vec![
        FeatureBox::new(s),
        FeatureBox::new(d),
        FeatureBox::new(Scale::non_uniform(vec![body(1)], ["1", "2", "3"])),
        FeatureBox::new(Thicken::new_body(vec![face(1, 1)], "1 mm")),
    ];
    let dims = [2, 2, 3, 1];
    for (f, n) in all.into_iter().zip(dims) {
        let json = serde_json::to_string(&f).unwrap();
        let back: FeatureBox = serde_json::from_str(&json).unwrap();
        assert_eq!(back, f);
        assert_eq!(f.get().exprs().len(), n, "{}", f.kind());
    }
    let sc = Scale::uniform(vec![body(1)], "2");
    assert!(matches!(sc.kind, ScaleKind::Uniform(_)));
    assert_eq!(sc.point, RefInput::Origin(OriginEntity::Point));
}

#[test]
fn edge_picks_are_refused_by_validate() {
    let mut e = face(1, 1);
    e.kind = TopoKind::Edge;
    assert!(Shell::inside(vec![e.clone()], "1 mm").validate().is_err());
    assert!(
        Draft::one_side(
            RefInput::Origin(OriginEntity::XYPlane),
            vec![e.clone()],
            "3 deg"
        )
        .validate()
        .is_err()
    );
    assert!(Thicken::new_body(vec![e], "1 mm").validate().is_err());
}

#[cfg(feature = "occt")]
#[path = "solid_shell/occt.rs"]
mod occt;
