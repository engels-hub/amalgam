//! SOL-04: the primitives `solid.box`, `solid.cylinder`, `solid.sphere` and `solid.torus`
//! (`dw1_*` … `dw4_*` name the Done-when items). Geometry needs the `occt` feature
//! (AMG_OCCT_DIR); the dialog and data tests run everywhere.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "tests"
)]

mod solid_common;

use amg_core::{Point3, Vec3};
use amg_doc::{
    BodyId, ConstructionId, Dependency, Dim, Document, EntityRef, Expr, ExprSrc, Feature,
    FeatureBox, FeatureId, TopoKind, TopoName, TopoRef,
};
use amg_features::construct::{ConstructPlane, OriginEntity, PlaneDef, RefInput};
use amg_features::dialog::{self, DialogInit, FieldValue, FieldValues, ManipKind};
use amg_features::solid::{
    BodyOp, BoxPrim, CylinderPrim, OpTarget, PrimitiveFrame, SpherePrim, TorusPrim,
};
#[allow(unused_imports, reason = "the occt tests use it through `super::*`")]
use solid_common::*;

const KINDS: [&str; 4] = ["solid.box", "solid.cylinder", "solid.sphere", "solid.torus"];

fn size_keys(kind: &str) -> &'static [&'static str] {
    match kind {
        "solid.box" => &["length", "width", "height"],
        "solid.cylinder" => &["radius", "height"],
        "solid.sphere" => &["radius"],
        "solid.torus" => &["major_radius", "minor_radius"],
        other => panic!("{other}"),
    }
}

fn ex(s: &str) -> Expr {
    Expr::Inline(ExprSrc(s.to_owned()))
}

fn body_id(n: u64) -> BodyId {
    BodyId {
        origin: FeatureId(n),
        slot: 0,
    }
}

fn a_face() -> EntityRef {
    EntityRef::Topo(TopoRef {
        body: body_id(3),
        kind: TopoKind::Face,
        name: TopoName::default(),
        fingerprint: None,
        key: 0,
    })
}

/// A frame on `plane` with offsets and a rotation.
fn frame(plane: RefInput, u: &str, v: &str, angle: &str) -> PrimitiveFrame {
    PrimitiveFrame {
        plane,
        u: ex(u),
        v: ex(v),
        angle: ex(angle),
    }
}

/// One feature of each kind on `frame`.
fn one_of_each(frame: &PrimitiveFrame, target: &OpTarget) -> [FeatureBox; 4] {
    let (f, t) = (frame.clone(), target.clone());
    [
        FeatureBox::new(BoxPrim::new(
            f.clone(),
            ["10 mm", "20 mm", "30 mm"],
            t.clone(),
        )),
        FeatureBox::new(CylinderPrim::new(f.clone(), "5 mm", "10 mm", t.clone())),
        FeatureBox::new(SpherePrim::new(f.clone(), "5 mm", t.clone())),
        FeatureBox::new(TorusPrim::new(f, "10 mm", "2 mm", t)),
    ]
}

fn defaults_with(kind: &str, doc: &Document, pre: &[EntityRef]) -> FieldValues {
    let reg = dialog::registration(kind).unwrap();
    (reg.defaults)(&DialogInit {
        preselection: pre,
        design: doc.design(),
        evaluated: doc.evaluated(),
    })
}

// ---- data (every build) -----------------------------------------------------------------------

#[test]
fn features_list_their_inputs_and_expressions_and_round_trip_as_json() {
    let plane = RefInput::Ref(EntityRef::Construction(ConstructionId(FeatureId(4))));
    let target = OpTarget {
        op: BodyOp::Join,
        participants: vec![body_id(2)],
    };
    let features = one_of_each(&frame(plane, "1 mm", "2 mm", "3 deg"), &target);
    for (mut f, kind) in features.into_iter().zip(KINDS) {
        assert_eq!(f.kind(), kind);
        assert_eq!(
            f.get().inputs(),
            vec![
                Dependency::Construction(ConstructionId(FeatureId(4))),
                Dependency::Body(body_id(2)),
            ]
        );
        f.get().validate().unwrap();
        let json = serde_json::to_string(&f).unwrap();
        let back: FeatureBox = serde_json::from_str(&json).unwrap();
        assert_eq!(back, f);
        // u, v (length), angle, then one length per size; both lists in the same order.
        let n = size_keys(kind).len();
        let read: Vec<Expr> = f.get().exprs().into_iter().cloned().collect();
        assert_eq!(read.len(), 3 + n);
        let mut_list = std::sync::Arc::get_mut(&mut f.0)
            .expect("unshared")
            .exprs_mut();
        let dims: Vec<Dim> = mut_list.iter().map(|(_, d)| *d).collect();
        let written: Vec<Expr> = mut_list.into_iter().map(|(e, _)| e.clone()).collect();
        assert_eq!(read, written, "{kind}");
        let mut want = vec![Dim::LENGTH, Dim::LENGTH, Dim::ANGLE];
        want.extend(std::iter::repeat_n(Dim::LENGTH, n));
        assert_eq!(dims, want, "{kind}");
    }
    // A new body ignores its participants; an origin plane is no dependency.
    let f = SpherePrim::new(PrimitiveFrame::xy(), "1 mm", OpTarget::default());
    assert!(f.inputs().is_empty());
}

#[cfg(not(feature = "occt"))]
#[test]
fn without_a_kernel_a_primitive_fails_cleanly() {
    let mut doc = Document::new("no-kernel");
    for f in one_of_each(&PrimitiveFrame::xy(), &OpTarget::default()) {
        let id = add(&mut doc, f);
        let h = format!("{:?}", health(&doc, id));
        assert!(h.contains("not available"), "{h}");
    }
}

// ---- Done-when 4: dialogs and manipulator anchors ---------------------------------------------

#[test]
fn dw4_every_primitive_dialog_has_a_distance_manipulator_per_size_field() {
    let doc = Document::new("dialogs");
    for kind in KINDS {
        let reg = dialog::registration(kind).expect(kind);
        let schema = (reg.schema)();
        assert_eq!(schema.kind, kind);
        assert_eq!(reg.command.group, "create");
        assert!(!reg.command.pinned);
        let manipulated: Vec<&str> = schema
            .fields
            .iter()
            .filter(|f| f.manipulator.is_some())
            .map(|f| {
                assert_eq!(f.manipulator, Some(ManipKind::Distance), "{}", f.key.0);
                f.key.0
            })
            .collect();
        assert_eq!(manipulated, size_keys(kind), "{kind}");

        let v = defaults_with(kind, &doc, &[]);
        for f in &schema.fields {
            assert!(v.0.contains_key(&f.key), "{kind}: {} default", f.key.0);
            if let Some(c) = &f.visible_if {
                assert!(!c.holds(&v), "{kind}: {} shown by default", f.key.0);
            }
        }
        // Labels are unique, and no option label equals a field label (the UI finds by
        // label).
        let labels: Vec<&str> = schema.fields.iter().map(|f| f.label.as_str()).collect();
        let mut sorted = labels.clone();
        sorted.sort_unstable();
        sorted.dedup();
        assert_eq!(sorted.len(), labels.len(), "{kind}: unique labels");
        for f in &schema.fields {
            if let dialog::FieldKind::Choice { options, .. } = &f.kind {
                for o in options {
                    assert!(!labels.contains(&o.label.as_str()), "{kind}: {}", o.label);
                }
            }
        }
        let built = (reg.build)(&v, None).unwrap();
        built.validate().unwrap();
        assert_eq!(built.kind(), kind);
        assert_eq!((reg.read)(built.get(), doc.design()).unwrap(), v);
    }
}

#[test]
fn dialog_values_round_trip_for_every_plane_and_operation() {
    let doc = Document::new("round-trip");
    let target = OpTarget {
        op: BodyOp::Cut,
        participants: vec![body_id(2)],
    };
    let planes = [
        RefInput::Origin(OriginEntity::XYPlane),
        RefInput::Origin(OriginEntity::YZPlane),
        RefInput::Origin(OriginEntity::XZPlane),
        RefInput::Ref(a_face()),
    ];
    for plane in planes {
        let features = one_of_each(&frame(plane, "1 mm", "-2 mm", "30 deg"), &target);
        for f in features {
            let reg = dialog::registration(f.kind()).unwrap();
            let v = (reg.read)(f.get(), doc.design()).unwrap();
            let back = (reg.build)(&v, Some(f.get())).unwrap();
            assert_eq!(back, f);
        }
    }
    let mut b = BoxPrim::new(PrimitiveFrame::xy(), ["1", "2", "3"], OpTarget::default());
    b.centered = true;
    let reg = dialog::registration("solid.box").unwrap();
    let v = (reg.read)(&b, doc.design()).unwrap();
    let back = (reg.build)(&v, None).unwrap();
    assert_eq!(back.downcast_ref::<BoxPrim>().unwrap(), &b);
}

#[test]
fn dialog_preselection_and_field_errors() {
    let mut doc = Document::new("pre");
    let plane = add(
        &mut doc,
        FeatureBox::new(ConstructPlane {
            def: PlaneDef::Offset {
                base: RefInput::Origin(OriginEntity::XYPlane),
                distance: ex("5 mm"),
            },
        }),
    );
    let construction = EntityRef::Construction(ConstructionId(plane));
    for kind in KINDS {
        for pre in [construction.clone(), a_face()] {
            let v = defaults_with(kind, &doc, &[pre.clone(), EntityRef::Body(body_id(2))]);
            assert_eq!(v.choice("plane", "xy").unwrap(), "picked");
            assert_eq!(v.selection("plane_ref").unwrap(), [pre]);
            assert_eq!(
                v.selection("bodies").unwrap(),
                [EntityRef::Body(body_id(2))]
            );
        }
        let reg = dialog::registration(kind).unwrap();
        let mut v = defaults_with(kind, &doc, &[]);
        v = v.with("plane", FieldValue::Choice("picked"));
        v = v.with(size_keys(kind)[0], FieldValue::expr(" "));
        let errs = (reg.build)(&v, None).unwrap_err();
        let keys: Vec<&str> = errs.0.iter().map(|(k, _)| k.0).collect();
        assert!(keys.contains(&"plane_ref"), "{kind}: {keys:?}");
        assert!(keys.contains(&size_keys(kind)[0]), "{kind}: {keys:?}");
    }
}

#[track_caller]
fn assert_point(got: Point3, want: [f64; 3]) {
    assert!(
        (got - Point3::from_array(want)).length() < 1e-9,
        "got {got:?}, want {want:?}"
    );
}

/// The anchors as `(field, origin, axis)` of a feature on the evaluated `doc`.
fn anchors(doc: &Document, f: &FeatureBox) -> Vec<(&'static str, Point3, Vec3)> {
    let reg = dialog::registration(f.kind()).unwrap();
    (reg.anchors)(f.get(), doc.design(), doc.evaluated())
        .into_iter()
        .map(|a| {
            assert_eq!(a.base, 0.0);
            assert!(a.ref_dir.is_none());
            (a.field.0, a.origin, a.axis.get())
        })
        .collect()
}

#[test]
fn dw4_anchors_sit_on_the_placed_frame_one_per_size_field() {
    let mut doc = Document::new("anchors");
    let plane = add(
        &mut doc,
        FeatureBox::new(ConstructPlane {
            def: PlaneDef::Offset {
                base: RefInput::Origin(OriginEntity::XYPlane),
                distance: ex("5 mm"),
            },
        }),
    );
    // On the plane z = 5, origin moved to (1, 2) and turned 90°: x is +Y, y is −X.
    let on = frame(
        RefInput::Ref(EntityRef::Construction(ConstructionId(plane))),
        "1 mm",
        "2 mm",
        "90 deg",
    );
    let [bx, cyl, sph, tor] = one_of_each(&on, &OpTarget::default());
    for f in [&bx, &cyl, &sph, &tor] {
        let fields: Vec<&str> = anchors(&doc, f).iter().map(|a| a.0).collect();
        assert_eq!(fields, size_keys(f.kind()), "{}", f.kind());
    }

    // Box 10 × 20 × 30 with a corner at (1, 2, 5): footprint centre (1 − 10, 2 + 5, 5).
    let a = anchors(&doc, &bx);
    assert_point(a[0].1, [-9.0, 2.0, 5.0]);
    assert_point(a[0].2, [0.0, 1.0, 0.0]);
    assert_point(a[1].1, [1.0, 7.0, 5.0]);
    assert_point(a[1].2, [-1.0, 0.0, 0.0]);
    assert_point(a[2].1, [-9.0, 7.0, 5.0]);
    assert_point(a[2].2, [0.0, 0.0, 1.0]);
    // Each arrow's tip (origin + size × axis) is on the opposite face.
    assert_point(a[0].1 + a[0].2 * 10.0, [-9.0, 12.0, 5.0]);
    assert_point(a[1].1 + a[1].2 * 20.0, [-19.0, 7.0, 5.0]);

    // Centred: the footprint centre is the frame origin.
    let mut centred = BoxPrim::new(on, ["10 mm", "20 mm", "30 mm"], OpTarget::default());
    centred.centered = true;
    let a = anchors(&doc, &FeatureBox::new(centred));
    assert_point(a[0].1, [1.0, -3.0, 5.0]);
    assert_point(a[1].1, [11.0, 2.0, 5.0]);
    assert_point(a[2].1, [1.0, 2.0, 5.0]);

    let a = anchors(&doc, &cyl);
    assert_point(a[0].1, [1.0, 2.0, 5.0]);
    assert_point(a[0].2, [0.0, 1.0, 0.0]);
    assert_point(a[1].1, [1.0, 2.0, 5.0]);
    assert_point(a[1].2, [0.0, 0.0, 1.0]);

    let a = anchors(&doc, &sph);
    assert_point(a[0].1, [1.0, 2.0, 5.0]);

    // The minor arrow starts at the centre of the tube, major radius 10 along x.
    let a = anchors(&doc, &tor);
    assert_point(a[0].1, [1.0, 2.0, 5.0]);
    assert_point(a[1].1, [1.0, 12.0, 5.0]);
    assert_point(a[1].2, [0.0, 0.0, 1.0]);

    // A reference that doesn't resolve has no manipulators.
    let [bad, ..] = one_of_each(
        &PrimitiveFrame::on(RefInput::Ref(a_face())),
        &OpTarget::default(),
    );
    assert!(anchors(&doc, &bad).is_empty());
}

#[cfg(feature = "occt")]
#[path = "solid_primitive/occt.rs"]
mod occt;
