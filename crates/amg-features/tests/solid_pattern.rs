//! SOL-08: `solid.pattern` (rectangular and circular; bodies and features; suppressed
//! instances; identical compute) and `solid.mirror` (bodies new/join, features); one test (or
//! test group) per Done-when item, see the test names. Geometry needs the `occt` feature
//! (AMG_OCCT_DIR); the dialog and data tests run everywhere.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "tests"
)]

mod solid_common;

#[allow(
    unused_imports,
    reason = "the occt tests use FeatureBox through `super::*`"
)]
use amg_doc::FeatureBox;
use amg_doc::{BodyId, Dependency, Document, EntityRef, Expr, ExprSrc, Feature, FeatureId};
use amg_features::construct::{OriginEntity, RefInput};
use amg_features::dialog::{self, DialogInit, FieldValue, FieldValues};
use amg_features::solid::{
    AngleSpan, BodyOp, ComputeType, Mirror, OpTarget, Pattern, PatternDir, PatternKind,
    PatternObjects, Spacing,
};
#[allow(unused_imports, reason = "the occt tests use it through `super::*`")]
use solid_common::*;

fn ex(s: &str) -> Expr {
    Expr::Inline(ExprSrc(s.to_owned()))
}

fn dir(axis: OriginEntity, count: &str, spacing: &str) -> PatternDir {
    PatternDir {
        dir: RefInput::Origin(axis),
        count: ex(count),
        spacing: Spacing::Spacing(ex(spacing)),
        flip: false,
        symmetric: false,
    }
}

fn rect(objects: PatternObjects, d1: PatternDir, d2: Option<PatternDir>) -> Pattern {
    Pattern {
        objects,
        kind: PatternKind::Rectangular { d1, d2 },
        suppressed: Vec::new(),
        compute: ComputeType::Identical,
        target: None,
    }
}

fn circular(objects: PatternObjects, count: &str, span: AngleSpan, symmetric: bool) -> Pattern {
    Pattern {
        objects,
        kind: PatternKind::Circular {
            axis: RefInput::Origin(OriginEntity::ZAxis),
            count: ex(count),
            span,
            symmetric,
        },
        suppressed: Vec::new(),
        compute: ComputeType::Identical,
        target: None,
    }
}

fn a_body() -> BodyId {
    BodyId {
        origin: FeatureId(3),
        slot: 0,
    }
}

// ---- dialog and data (every build) ------------------------------------------------------------

fn defaults(kind: &str) -> FieldValues {
    let doc = Document::new("defaults");
    let reg = dialog::registration(kind).unwrap();
    let pre = [EntityRef::Body(a_body())];
    (reg.defaults)(&DialogInit {
        preselection: &pre,
        design: doc.design(),
        evaluated: doc.evaluated(),
    })
}

fn round_trip(kind: &str, v: &FieldValues) {
    let doc = Document::new("rt");
    let reg = dialog::registration(kind).unwrap();
    let f = (reg.build)(v, None).unwrap_or_else(|e| panic!("{kind}: {e:?}"));
    f.validate().unwrap();
    let back = (reg.read)(f.get(), doc.design()).unwrap();
    assert_eq!(&back, v, "{kind}: read(build(v)) == v");
}

#[test]
fn dialog_defaults_build_a_valid_feature_and_round_trip() {
    for kind in ["solid.pattern", "solid.mirror"] {
        let v = defaults(kind);
        assert_eq!(
            v.get("bodies"),
            Some(&FieldValue::Selection(vec![EntityRef::Body(a_body())])),
            "{kind}: preselected bodies"
        );
        round_trip(kind, &v);
        let schema = (dialog::registration(kind).unwrap().schema)();
        for field in &schema.fields {
            assert!(
                v.get(field.key.0).is_some(),
                "{kind}: default for {}",
                field.key.0
            );
        }
    }
}

#[test]
fn dialog_round_trips_every_pattern_layout() {
    let base = defaults("solid.pattern");
    let two_dirs = base
        .clone()
        .with("d1_spacing_type", FieldValue::Choice("extent"))
        .with("d1_count", FieldValue::expr("4"))
        .with("d1_flip", FieldValue::Flip(true))
        .with("d2_enabled", FieldValue::Bool(true))
        .with("d2_symmetric", FieldValue::Bool(true))
        .with("suppressed", FieldValue::Text("3, 65537".into()));
    round_trip("solid.pattern", &two_dirs);
    for span in ["full", "extent", "spacing"] {
        let v = base
            .clone()
            .with("pattern_type", FieldValue::Choice("circular"))
            .with("span", FieldValue::Choice(span))
            .with("count", FieldValue::expr("6"));
        round_trip("solid.pattern", &v);
    }
    let features = base
        .clone()
        .with("object_type", FieldValue::Choice("features"))
        .with("bodies", FieldValue::Selection(Vec::new()))
        .with("features", FieldValue::Text("12, 15".into()));
    round_trip("solid.pattern", &features);
    let picked = base.with("d1_axis", FieldValue::Choice("picked")).with(
        "d1_ref",
        FieldValue::Selection(vec![EntityRef::Construction(amg_doc::ConstructionId(
            FeatureId(8),
        ))]),
    );
    round_trip("solid.pattern", &picked);
}

#[test]
fn dialog_round_trips_mirror_options() {
    let base = defaults("solid.mirror");
    round_trip(
        "solid.mirror",
        &base.clone().with("operation", FieldValue::Choice("join")),
    );
    let features = base
        .with("object_type", FieldValue::Choice("features"))
        .with("bodies", FieldValue::Selection(Vec::new()))
        .with("features", FieldValue::Text("4".into()))
        .with("plane", FieldValue::Choice("picked"))
        .with(
            "plane_ref",
            FieldValue::Selection(vec![EntityRef::Construction(amg_doc::ConstructionId(
                FeatureId(8),
            ))]),
        );
    round_trip("solid.mirror", &features);
}

#[test]
fn dialog_reports_bad_ids() {
    let reg = dialog::registration("solid.pattern").unwrap();
    let v = defaults("solid.pattern")
        .with("object_type", FieldValue::Choice("features"))
        .with("features", FieldValue::Text("12, x".into()));
    let errs = (reg.build)(&v, None).unwrap_err();
    assert_eq!(errs.0[0].0.0, "features");
    let v = defaults("solid.pattern").with("suppressed", FieldValue::Text("-1".into()));
    assert!((reg.build)(&v, None).is_err());
}

#[test]
fn data_round_trips_through_json_and_lists_inputs() {
    let mut p = rect(
        PatternObjects::Features(vec![FeatureId(5)]),
        dir(OriginEntity::XAxis, "3", "10 mm"),
        Some(dir(OriginEntity::YAxis, "4", "5 mm")),
    );
    p.suppressed = vec![1, 65537];
    p.target = Some(OpTarget {
        op: BodyOp::Cut,
        participants: vec![a_body()],
    });
    let json = serde_json::to_string(&p).unwrap();
    let back: Pattern = serde_json::from_str(&json).unwrap();
    assert_eq!(back, p);
    assert_eq!(
        p.inputs(),
        vec![
            Dependency::Feature(FeatureId(5)),
            Dependency::Body(a_body())
        ]
    );
    assert_eq!(p.exprs().len(), 4);
    let m = Mirror {
        objects: PatternObjects::Bodies(vec![a_body()]),
        plane: RefInput::Ref(EntityRef::Construction(amg_doc::ConstructionId(FeatureId(
            9,
        )))),
        op: BodyOp::Join,
        target: None,
    };
    let back: Mirror = serde_json::from_str(&serde_json::to_string(&m).unwrap()).unwrap();
    assert_eq!(back, m);
    assert_eq!(
        m.inputs(),
        vec![
            Dependency::Body(a_body()),
            Dependency::Construction(amg_doc::ConstructionId(FeatureId(9)))
        ]
    );
}

#[test]
fn validate_refuses_empty_and_inconsistent_inputs() {
    let empty = rect(
        PatternObjects::Bodies(Vec::new()),
        dir(OriginEntity::XAxis, "2", "1 mm"),
        None,
    );
    // An empty selection builds and validates (the dialog's `min: 1` keeps OK off); it fails
    // in compute (`a_bad_count_is_a_clean_error` and friends cover compute errors).
    empty.validate().unwrap();
    let mut with_target = rect(
        PatternObjects::Bodies(vec![a_body()]),
        dir(OriginEntity::XAxis, "2", "1 mm"),
        None,
    );
    with_target.target = Some(OpTarget::default());
    assert!(with_target.validate().is_err());
    let cut_mirror = Mirror {
        objects: PatternObjects::Bodies(vec![a_body()]),
        plane: RefInput::Origin(OriginEntity::YZPlane),
        op: BodyOp::Cut,
        target: None,
    };
    assert!(cut_mirror.validate().is_err());
    let ok = circular(
        PatternObjects::Bodies(vec![a_body()]),
        "6",
        AngleSpan::Full,
        false,
    );
    ok.validate().unwrap();
}

#[cfg(feature = "occt")]
#[path = "solid_pattern/occt.rs"]
mod occt;
