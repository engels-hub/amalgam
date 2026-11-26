//! SOL-02: `solid.extrude` with every start, direction and extent, tapers, planar-face and
//! text profiles; one test (or test group) per Done-when item, see the test names. Geometry
//! needs the `occt` feature (AMG_OCCT_DIR); the dialog and data tests run everywhere.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "tests"
)]

mod solid_common;

use amg_doc::{Dependency, Document, EntityRef, Expr, ExprSrc, Feature, FeatureBox, FeatureId};
use amg_features::dialog::{self, DialogInit, FieldValue, FieldValues};
use amg_features::solid::{Direction, Extent, ExtentSide, Extrude, OpTarget, Start};
#[allow(unused_imports, reason = "the occt tests use it through `super::*`")]
use solid_common::*;

fn ex(s: &str) -> Expr {
    Expr::Inline(ExprSrc(s.to_owned()))
}

fn a_profile_ref() -> EntityRef {
    EntityRef::Profile {
        sketch: amg_doc::SketchId(FeatureId(7)),
        profile: amg_doc::stub::ProfileKey(42),
        fingerprint: None,
    }
}

fn a_construction() -> EntityRef {
    EntityRef::Construction(amg_doc::ConstructionId(FeatureId(9)))
}

// ---- dialog and data (every build) ------------------------------------------------------------

fn defaults() -> FieldValues {
    let doc = Document::new("defaults");
    let reg = dialog::registration("solid.extrude").unwrap();
    let pre = [a_profile_ref()];
    (reg.defaults)(&DialogInit {
        preselection: &pre,
        design: doc.design(),
        evaluated: doc.evaluated(),
    })
}

#[test]
fn dialog_conditional_fields_are_hidden_for_the_defaults() {
    // UI-05 shows a field iff its `visible_if` holds; the UI harness expects every field with a
    // condition to be hidden for the defaults (one side, profile plane, fixed distance).
    let reg = dialog::registration("solid.extrude").unwrap();
    let schema = (reg.schema)();
    let v = defaults();
    let mut hidden = Vec::new();
    for f in &schema.fields {
        if let Some(c) = &f.visible_if {
            assert!(!c.holds(&v), "{} is shown for the defaults", f.key.0);
            hidden.push(f.key.0);
        }
        assert!(v.0.contains_key(&f.key), "{} has a default", f.key.0);
    }
    for k in [
        "start_offset",
        "start_object",
        "whole_length",
        "object",
        "side2_extent",
        "side2_taper",
    ] {
        assert!(hidden.contains(&k), "{k} is conditional");
    }
    // Labels are unique, and no option label equals a field label (the UI finds by label).
    let labels: Vec<&str> = schema.fields.iter().map(|f| f.label.as_str()).collect();
    let mut sorted = labels.clone();
    sorted.sort_unstable();
    sorted.dedup();
    assert_eq!(sorted.len(), labels.len(), "unique labels");
    for f in &schema.fields {
        if let dialog::FieldKind::Choice { options, .. } = &f.kind {
            for o in options {
                assert!(
                    !labels.contains(&o.label.as_str()),
                    "option label {} equals a field label",
                    o.label
                );
            }
        }
    }
    // Showing the second side reveals its fields.
    let two = v.clone().with("direction", FieldValue::Choice("two_sides"));
    let side2 = schema
        .fields
        .iter()
        .find(|f| f.key.0 == "side2_extent")
        .unwrap();
    assert!(side2.visible_if.as_ref().unwrap().holds(&two));
}

/// `read(build(v)) == v` and the built feature, for one dialog state.
fn round_trip(v: &FieldValues) -> Extrude {
    let doc = Document::new("rt");
    let reg = dialog::registration("solid.extrude").unwrap();
    let f = (reg.build)(v, None).unwrap_or_else(|e| panic!("{e:?}"));
    f.validate().unwrap();
    assert_eq!(&(reg.read)(f.get(), doc.design()).unwrap(), v);
    f.downcast_ref::<Extrude>().unwrap().clone()
}

#[test]
fn dialog_round_trips_every_start_direction_and_extent() {
    let base = defaults();
    let target = FieldValue::Selection(vec![a_construction()]);
    // Starts.
    let e = round_trip(
        &base
            .clone()
            .with("start", FieldValue::Choice("offset"))
            .with("start_offset", FieldValue::expr("5 mm")),
    );
    assert_eq!(e.start, Start::Offset(ex("5 mm")));
    let e = round_trip(
        &base
            .clone()
            .with("start", FieldValue::Choice("object"))
            .with("start_object", target.clone())
            .with("start_offset", FieldValue::expr("2 mm")),
    );
    assert_eq!(
        e.start,
        Start::Object {
            target: a_construction(),
            offset: ex("2 mm")
        }
    );
    // Extents of side 1, with taper.
    let e = round_trip(
        &base
            .clone()
            .with("extent", FieldValue::Choice("to_object"))
            .with("object", target.clone())
            .with("object_offset", FieldValue::expr("-1 mm"))
            .with("extend", FieldValue::Bool(true))
            .with("taper", FieldValue::expr("3 deg")),
    );
    assert_eq!(
        e.side1,
        ExtentSide {
            extent: Extent::ToObject {
                target: a_construction(),
                offset: ex("-1 mm"),
                extend: true
            },
            taper: ex("3 deg")
        }
    );
    let e = round_trip(&base.clone().with("extent", FieldValue::Choice("all")));
    assert_eq!(e.side1.extent, Extent::All);
    // Two sides with every second-side extent.
    for (ext, want) in [
        ("distance", Extent::Distance(ex("4 mm"))),
        ("all", Extent::All),
        (
            "to_object",
            Extent::ToObject {
                target: a_construction(),
                offset: ex("0 mm"),
                extend: false,
            },
        ),
    ] {
        let mut v = base
            .clone()
            .with("direction", FieldValue::Choice("two_sides"))
            .with("side2_extent", FieldValue::Choice(ext))
            .with("side2_taper", FieldValue::expr("-2 deg"));
        if ext == "distance" {
            v = v.with("side2_distance", FieldValue::expr("4 mm"));
        }
        if ext == "to_object" {
            v = v.with("side2_object", target.clone());
        }
        let e = round_trip(&v);
        assert_eq!(e.direction, Direction::TwoSides);
        assert_eq!(e.side2.as_ref().unwrap().extent, want);
        assert_eq!(e.side2.as_ref().unwrap().taper, ex("-2 deg"));
    }
    // Symmetric, half and whole.
    for whole in [false, true] {
        let e = round_trip(
            &base
                .clone()
                .with("direction", FieldValue::Choice("symmetric"))
                .with("whole_length", FieldValue::Bool(whole)),
        );
        assert_eq!(
            e.direction,
            Direction::Symmetric {
                whole_length: whole
            }
        );
        assert!(e.side2.is_none());
    }
    // A to-object side without an object is a field error.
    let reg = dialog::registration("solid.extrude").unwrap();
    let errs =
        (reg.build)(&base.with("extent", FieldValue::Choice("to_object")), None).unwrap_err();
    assert_eq!(errs.0[0].0.0, "object");
}

#[test]
fn feature_data_lists_targets_exprs_and_validates_sides() {
    let mut e = Extrude::distance(vec![a_profile_ref()], "10 mm", OpTarget::default());
    e.start = Start::Object {
        target: a_construction(),
        offset: ex("1 mm"),
    };
    e.direction = Direction::TwoSides;
    let face = EntityRef::Body(amg_doc::BodyId {
        origin: FeatureId(3),
        slot: 0,
    });
    e.side2 = Some(ExtentSide {
        extent: Extent::ToObject {
            target: face.clone(),
            offset: ex("2 mm"),
            extend: false,
        },
        taper: ex("1 deg"),
    });
    e.validate().unwrap();
    let d = e.inputs();
    assert!(d.contains(&Dependency::Entity(a_construction())));
    assert!(d.contains(&Dependency::Entity(face)));
    assert_eq!(
        e.exprs().len(),
        5,
        "start offset, distance, taper, offset 2, taper 2"
    );
    assert_eq!(e.clone().exprs_mut().len(), 5);
    let json = serde_json::to_string(&FeatureBox::new(e.clone())).unwrap();
    let back: FeatureBox = serde_json::from_str(&json).unwrap();
    assert_eq!(back.downcast_ref::<Extrude>(), Some(&e));
    // Symmetric needs a distance.
    let mut s = Extrude::distance(vec![a_profile_ref()], "10 mm", OpTarget::default());
    s.direction = Direction::Symmetric {
        whole_length: false,
    };
    s.validate().unwrap();
    s.side1.extent = Extent::All;
    assert!(s.validate().is_err());
}

#[cfg(feature = "occt")]
#[path = "solid_extrude_sol02/occt.rs"]
mod occt;
