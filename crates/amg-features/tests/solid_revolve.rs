//! SOL-03: `solid.revolve` (profiles, axis kinds, angle, full, two sides, symmetric, to
//! object; profile/axis validation), one test (or test group) per Done-when item, see the test
//! names. Geometry needs the `occt` feature (AMG_OCCT_DIR); the dialog and data tests run
//! everywhere.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "tests"
)]

mod solid_common;

use amg_doc::{
    Dependency, Dim, Document, EntityRef, Expr, ExprSrc, Feature, FeatureBox, FeatureId,
};
use amg_features::construct::{OriginEntity, RefInput};
use amg_features::dialog::{self, DialogInit, FieldValue, FieldValues};
use amg_features::solid::{Direction, Extent, ExtentSide, OpTarget, Revolve};
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

fn z_axis() -> RefInput {
    RefInput::Origin(OriginEntity::ZAxis)
}

// ---- dialog and data (every build) ------------------------------------------------------------

fn defaults_with(pre: &[EntityRef]) -> FieldValues {
    let doc = Document::new("defaults");
    let reg = dialog::registration("solid.revolve").unwrap();
    (reg.defaults)(&DialogInit {
        preselection: pre,
        design: doc.design(),
        evaluated: doc.evaluated(),
    })
}

fn defaults() -> FieldValues {
    defaults_with(&[a_profile_ref()])
}

#[test]
fn dialog_exists_and_conditional_fields_are_hidden_for_the_defaults() {
    let reg = dialog::registration("solid.revolve").unwrap();
    assert_eq!(reg.command.group, "create");
    let schema = (reg.schema)();
    assert_eq!(schema.kind, "solid.revolve");
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
        "axis_ref",
        "whole_length",
        "object",
        "side2_extent",
        "side2_angle",
        "bodies",
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
    // The defaults build a full turn about Z.
    let f = (reg.build)(&v, None).unwrap();
    let r = f.downcast_ref::<Revolve>().unwrap();
    assert_eq!(r.side1.extent, Extent::Full);
    assert_eq!(r.axis, z_axis());
    assert_eq!(r.profiles, vec![a_profile_ref()]);
}

#[test]
fn a_preselected_line_becomes_the_axis() {
    let line = EntityRef::SketchCurve {
        sketch: amg_doc::SketchId(FeatureId(7)),
        curve: amg_doc::stub::SketchEntityKey(3),
    };
    let v = defaults_with(&[a_profile_ref(), line.clone()]);
    assert_eq!(v.choice("axis", "z").unwrap(), "picked");
    let reg = dialog::registration("solid.revolve").unwrap();
    let f = (reg.build)(&v, None).unwrap();
    let r = f.downcast_ref::<Revolve>().unwrap();
    assert_eq!(r.axis, RefInput::Ref(line));
    assert_eq!(r.profiles, vec![a_profile_ref()]);
}

/// `read(build(v)) == v` and the built feature, for one dialog state.
fn round_trip(v: &FieldValues) -> Revolve {
    let doc = Document::new("rt");
    let reg = dialog::registration("solid.revolve").unwrap();
    let f = (reg.build)(v, None).unwrap_or_else(|e| panic!("{e:?}"));
    f.validate().unwrap();
    assert_eq!(&(reg.read)(f.get(), doc.design()).unwrap(), v);
    f.downcast_ref::<Revolve>().unwrap().clone()
}

#[test]
fn dialog_round_trips_every_axis_direction_and_extent() {
    let base = defaults();
    let target = FieldValue::Selection(vec![a_construction()]);
    // Axes.
    for (id, want) in [
        ("x", RefInput::Origin(OriginEntity::XAxis)),
        ("y", RefInput::Origin(OriginEntity::YAxis)),
        ("z", z_axis()),
    ] {
        assert_eq!(
            round_trip(&base.clone().with("axis", FieldValue::Choice(id))).axis,
            want
        );
    }
    let r = round_trip(
        &base
            .clone()
            .with("axis", FieldValue::Choice("picked"))
            .with("axis_ref", target.clone()),
    );
    assert_eq!(r.axis, RefInput::Ref(a_construction()));
    // Extents of side 1.
    let r = round_trip(
        &base
            .clone()
            .with("extent", FieldValue::Choice("angle"))
            .with("angle", FieldValue::expr("75 deg")),
    );
    assert_eq!(r.side1.extent, Extent::Distance(ex("75 deg")));
    let r = round_trip(
        &base
            .clone()
            .with("extent", FieldValue::Choice("to_object"))
            .with("object", target.clone())
            .with("object_offset", FieldValue::expr("1 mm"))
            .with("extend", FieldValue::Bool(true)),
    );
    assert_eq!(
        r.side1.extent,
        Extent::ToObject {
            target: a_construction(),
            offset: ex("1 mm"),
            extend: true
        }
    );
    // Two sides with each second-side extent.
    for (ext, want) in [
        ("angle", Extent::Distance(ex("60 deg"))),
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
            .with("extent", FieldValue::Choice("angle"))
            .with("angle", FieldValue::expr("30 deg"))
            .with("side2_extent", FieldValue::Choice(ext));
        if ext == "angle" {
            v = v.with("side2_angle", FieldValue::expr("60 deg"));
        }
        if ext == "to_object" {
            v = v.with("side2_object", target.clone());
        }
        let r = round_trip(&v);
        assert_eq!(r.direction, Direction::TwoSides);
        assert_eq!(r.side1.extent, Extent::Distance(ex("30 deg")));
        assert_eq!(r.side2.as_ref().unwrap().extent, want);
    }
    // Symmetric, half and whole.
    for whole in [false, true] {
        let r = round_trip(
            &base
                .clone()
                .with("direction", FieldValue::Choice("symmetric"))
                .with("extent", FieldValue::Choice("angle"))
                .with("angle", FieldValue::expr("90 deg"))
                .with("whole_length", FieldValue::Bool(whole)),
        );
        assert_eq!(
            r.direction,
            Direction::Symmetric {
                whole_length: whole
            }
        );
        assert!(r.side2.is_none());
    }
    // A full turn on two sides and a picked axis without a pick are field errors.
    let reg = dialog::registration("solid.revolve").unwrap();
    let errs = (reg.build)(
        &base
            .clone()
            .with("direction", FieldValue::Choice("two_sides")),
        None,
    )
    .unwrap_err();
    assert_eq!(errs.0[0].0.0, "extent");
    let errs = (reg.build)(&base.with("axis", FieldValue::Choice("picked")), None).unwrap_err();
    assert_eq!(errs.0[0].0.0, "axis_ref");
}

#[test]
fn feature_data_lists_inputs_exprs_and_validates_sides() {
    let body = amg_doc::BodyId {
        origin: FeatureId(3),
        slot: 0,
    };
    let mut r = Revolve::angle(
        vec![a_profile_ref()],
        RefInput::Ref(a_construction()),
        "30 deg",
        OpTarget::default(),
    );
    r.direction = Direction::TwoSides;
    r.side2 = Some(ExtentSide {
        extent: Extent::ToObject {
            target: EntityRef::Body(body),
            offset: ex("2 mm"),
            extend: false,
        },
        taper: ex("0 deg"),
    });
    r.validate().unwrap();
    let d = r.inputs();
    assert!(d.contains(&Dependency::Entity(a_profile_ref())));
    assert!(d.contains(&Dependency::Sketch(amg_doc::SketchId(FeatureId(7)))));
    assert!(
        d.contains(&Dependency::Construction(amg_doc::ConstructionId(
            FeatureId(9)
        )))
    );
    // A body target is read whole, so the body itself is an input (cache key).
    assert!(d.contains(&Dependency::Body(body)));
    let mut m = r.clone();
    let dims: Vec<Dim> = m.exprs_mut().into_iter().map(|(_, d)| d).collect();
    assert_eq!(dims, vec![Dim::ANGLE, Dim::ANGLE, Dim::LENGTH, Dim::ANGLE]);
    assert_eq!(r.exprs().len(), 4, "angle, taper, offset 2, taper 2");
    let json = serde_json::to_string(&FeatureBox::new(r.clone())).unwrap();
    assert!(json.contains("solid.revolve"));
    let back: FeatureBox = serde_json::from_str(&json).unwrap();
    assert_eq!(back.downcast_ref::<Revolve>(), Some(&r));
    // Static checks.
    let full = Revolve::full(vec![a_profile_ref()], z_axis(), OpTarget::default());
    full.validate().unwrap();
    let mut bad = full.clone();
    bad.direction = Direction::Symmetric {
        whole_length: false,
    };
    assert!(bad.validate().is_err(), "symmetric needs an angle");
    let mut bad = full.clone();
    bad.side1.extent = Extent::All;
    assert!(bad.validate().is_err(), "no All on a revolve");
    let mut bad = full.clone();
    bad.profiles.clear();
    assert!(bad.validate().is_err());
    let mut bad = r;
    bad.side2 = None;
    assert!(bad.validate().is_err(), "two sides need side 2");
}

#[test]
fn every_registration_still_round_trips_with_the_revolve() {
    let doc = Document::new("all");
    let pre = [a_profile_ref()];
    let init = DialogInit {
        preselection: &pre,
        design: doc.design(),
        evaluated: doc.evaluated(),
    };
    assert!(dialog::registrations().any(|r| r.kind == "solid.revolve"));
    let reg = dialog::registration("solid.revolve").unwrap();
    let v = (reg.defaults)(&init);
    let f = (reg.build)(&v, None).unwrap();
    assert_eq!((reg.read)(f.get(), doc.design()).unwrap(), v);
}

#[cfg(feature = "occt")]
#[path = "solid_revolve/occt.rs"]
mod occt;
