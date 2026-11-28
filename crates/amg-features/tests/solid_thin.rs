//! SOL-18: thin extrude and thin revolve (open and closed chains, side 1/2/center, names
//! `ThinInner` / `ThinOuter` / `ThinEnd`), one test (or test group) per Done-when item, see
//! the test names. Geometry needs the `occt` feature (AMG_OCCT_DIR); the dialog and data tests
//! run everywhere.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "tests"
)]

mod solid_common;

use amg_doc::stub::SketchEntityKey;
use amg_doc::{
    Dependency, Dim, Document, EntityRef, Expr, ExprSrc, Feature, FeatureBox, FeatureId, SketchId,
};
use amg_features::construct::{OriginEntity, RefInput};
use amg_features::dialog::{self, DialogInit, FieldKey, FieldValue, FieldValues};
use amg_features::solid::{Extrude, OpTarget, Revolve, ThinSide, ThinSpec};
use amg_sketch::EntityId;
#[allow(unused_imports, reason = "the occt tests use it through `super::*`")]
use solid_common::*;

fn ex(s: &str) -> Expr {
    Expr::Inline(ExprSrc(s.to_owned()))
}

fn thin(thickness: &str, side: ThinSide) -> Option<ThinSpec> {
    Some(ThinSpec {
        thickness: ex(thickness),
        side,
    })
}

fn curve_ref(sketch: FeatureId, curve: EntityId) -> EntityRef {
    EntityRef::SketchCurve {
        sketch: SketchId(sketch),
        curve: SketchEntityKey(u64::from(curve.0)),
    }
}

fn values(kind: &str, preselection: &[EntityRef]) -> FieldValues {
    let doc = Document::new("dialog");
    let reg = dialog::registration(kind).unwrap();
    (reg.defaults)(&DialogInit {
        design: doc.design(),
        evaluated: doc.evaluated(),
        preselection,
    })
}

#[test]
fn the_dialogs_hide_the_thin_options_until_thin_is_on() {
    for kind in ["solid.extrude", "solid.revolve"] {
        let reg = dialog::registration(kind).unwrap();
        let fields = &(reg.schema)().fields;
        let field = |key: &str| fields.iter().find(|f| f.key.0 == key).unwrap();
        assert!(field("thin").visible_if.is_none(), "{kind}");
        let off = values(kind, &[]);
        let on = off.clone().with("thin", FieldValue::Bool(true));
        for key in ["thickness", "thin_side"] {
            let condition = field(key).visible_if.as_ref().expect(key);
            assert!(!condition.holds(&off), "{kind} {key}");
            assert!(condition.holds(&on), "{kind} {key}");
        }
        let mut labels: Vec<&str> = fields.iter().map(|f| f.label.as_str()).collect();
        labels.sort_unstable();
        let n = labels.len();
        labels.dedup();
        assert_eq!(labels.len(), n, "{kind}: unique labels");
    }
}

#[test]
fn thin_values_round_trip_through_both_dialogs() {
    let curve = curve_ref(FeatureId(7), EntityId(3));
    for kind in ["solid.extrude", "solid.revolve"] {
        let reg = dialog::registration(kind).unwrap();
        for (id, side) in [
            ("side1", ThinSide::Side1),
            ("side2", ThinSide::Side2),
            ("center", ThinSide::Center),
        ] {
            let v = values(kind, &[])
                .with("profiles", FieldValue::Selection(vec![curve.clone()]))
                .with("thin", FieldValue::Bool(true))
                .with("thickness", FieldValue::expr("2.5 mm"))
                .with("thin_side", FieldValue::Choice(id));
            let f = (reg.build)(&v, None).unwrap();
            let spec = match kind {
                "solid.extrude" => f.as_any().downcast_ref::<Extrude>().unwrap().thin.clone(),
                _ => f.as_any().downcast_ref::<Revolve>().unwrap().thin.clone(),
            };
            assert_eq!(spec, thin("2.5 mm", side), "{kind} {id}");
            let doc = Document::new("read");
            assert_eq!((reg.read)(&*f, doc.design()).unwrap(), v, "{kind} {id}");
            // Turning it off removes the options.
            let off = v.with("thin", FieldValue::Bool(false)).with(
                "profiles",
                FieldValue::Selection(vec![EntityRef::Profile {
                    sketch: SketchId(FeatureId(7)),
                    profile: amg_doc::stub::ProfileKey(1),
                    fingerprint: None,
                }]),
            );
            let g = (reg.build)(&off, Some(&*f)).unwrap();
            assert!(g.exprs().iter().all(|e| **e != ex("2.5 mm")), "{kind}");
        }
        // Sketch curves without Thin are a field error on the toggle.
        let v = values(kind, &[]).with("profiles", FieldValue::Selection(vec![curve.clone()]));
        let errs = (reg.build)(&v, None).unwrap_err();
        assert_eq!(errs.0.len(), 1, "{errs:?}");
        assert_eq!(errs.0[0].0, FieldKey("thin"));
    }
}

#[test]
fn preselected_curves_start_a_thin_extrude() {
    let curves = [
        curve_ref(FeatureId(7), EntityId(3)),
        curve_ref(FeatureId(7), EntityId(4)),
    ];
    let v = values("solid.extrude", &curves);
    assert_eq!(v.get("thin"), Some(&FieldValue::Bool(true)));
    assert_eq!(
        v.get("profiles"),
        Some(&FieldValue::Selection(curves.to_vec()))
    );
    // In a revolve a preselected curve stays the axis.
    let v = values("solid.revolve", &curves[..1]);
    assert_eq!(v.get("thin"), Some(&FieldValue::Bool(false)));
}

#[test]
fn thin_inputs_and_expressions() {
    let sketch = FeatureId(7);
    let picks = vec![curve_ref(sketch, EntityId(3))];
    let mut e = Extrude::distance(picks.clone(), "10 mm", OpTarget::default());
    e.thin = thin("2 mm", ThinSide::Center);
    let mut r = Revolve::full(
        picks.clone(),
        RefInput::Origin(OriginEntity::ZAxis),
        OpTarget::default(),
    );
    r.thin = thin("2 mm", ThinSide::Center);
    for f in [&mut e as &mut dyn Feature, &mut r] {
        let inputs = f.inputs();
        assert!(inputs.contains(&Dependency::Sketch(SketchId(sketch))));
        assert!(inputs.contains(&Dependency::Entity(picks[0].clone())));
        assert!(f.exprs().contains(&&ex("2 mm")));
        assert!(
            f.exprs_mut()
                .iter()
                .any(|(x, d)| **x == ex("2 mm") && *d == Dim::LENGTH)
        );
    }
    let json = serde_json::to_string(&e).unwrap();
    assert_eq!(serde_json::from_str::<Extrude>(&json).unwrap(), e);
    let b = FeatureBox::new(e);
    assert!(b.validate().is_ok());
}

#[cfg(feature = "occt")]
#[path = "solid_thin/occt.rs"]
mod occt;
