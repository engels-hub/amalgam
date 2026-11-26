//! CON-04: the dialog schemas of the construction features, without a UI.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    missing_docs,
    reason = "tests"
)]

use amg_core::{Point3, Vec3};
use amg_doc::naming::{NameSeg, Role, TopoKind, TopoName, TopoRef};
use amg_doc::stub::SketchEntityKey;
use amg_doc::{
    BodyId, Command, ConstructionId, CreatedId, Document, EntityRef, FeatureBox, FeatureId,
    SketchId,
};
use amg_features::construct::dialog::{
    self as construct, AXIS_KIND, Accepts, PICK, PLANE_KIND, POINT_KIND, TYPE, Unfilled, VARIANTS,
    origin_id, variant_values,
};
use amg_features::construct::{ConstructAxis, ConstructPlane, ConstructPoint, RefInput};
use amg_features::dialog::{
    DialogInit, DialogRegistration, FieldKey, FieldKind, FieldValue, FieldValues, ManipKind,
    registration,
};

fn topo(kind: TopoKind, n: u32) -> EntityRef {
    EntityRef::Topo(TopoRef {
        body: BodyId {
            origin: FeatureId(40),
            slot: 0,
        },
        kind,
        name: TopoName::new(vec![NameSeg::Gen {
            feature: FeatureId(40),
            role: Role::new("Test", n),
            from: None,
        }]),
        fingerprint: None,
        key: n,
    })
}

fn curve(n: u64) -> EntityRef {
    EntityRef::SketchCurve {
        sketch: SketchId(FeatureId(41)),
        curve: SketchEntityKey(n),
    }
}

fn point(n: u64) -> EntityRef {
    EntityRef::SketchPoint {
        sketch: SketchId(FeatureId(41)),
        point: SketchEntityKey(n),
    }
}

/// An entity that fills input number `n` of kind `accepts`.
fn entity(accepts: Accepts, n: u32) -> EntityRef {
    match accepts {
        Accepts::Plane | Accepts::Face | Accepts::PlaneOrPoint => topo(TopoKind::Face, n),
        Accepts::Line | Accepts::Revolved | Accepts::Center | Accepts::Path => curve(n.into()),
        Accepts::Point => point(n.into()),
    }
}

fn reg(kind: &str) -> &'static DialogRegistration {
    registration(kind).unwrap_or_else(|| panic!("no registration for {kind}"))
}

fn init<'a>(doc: &'a Document, preselection: &'a [EntityRef]) -> DialogInit<'a> {
    DialogInit {
        preselection,
        design: doc.design(),
        evaluated: doc.evaluated(),
    }
}

fn refs(f: &FeatureBox) -> Vec<RefInput> {
    let a = f.get().as_any();
    let v: Vec<&RefInput> = if let Some(p) = a.downcast_ref::<ConstructPlane>() {
        p.def.refs()
    } else if let Some(x) = a.downcast_ref::<ConstructAxis>() {
        x.def.refs()
    } else {
        a.downcast_ref::<ConstructPoint>().unwrap().def.refs()
    };
    v.into_iter().cloned().collect()
}

/// The keys of the fields shown for `v`.
fn shown(kind: &str, v: &FieldValues) -> Vec<&'static str> {
    (reg(kind).schema)()
        .fields
        .iter()
        .filter(|f| f.visible_if.as_ref().is_none_or(|c| c.holds(v)))
        .map(|f| f.key.0)
        .collect()
}

#[test]
fn there_are_8_planes_6_axes_and_6_points_each_a_type_option() {
    for (kind, n) in [(PLANE_KIND, 8), (AXIS_KIND, 6), (POINT_KIND, 6)] {
        assert_eq!(construct::variants_of(kind).count(), n);
        let schema = (reg(kind).schema)();
        assert_eq!(schema.kind, kind);
        let FieldKind::Choice { options, .. } = &schema.field(FieldKey(TYPE)).unwrap().kind else {
            panic!("Type is a choice");
        };
        let ids: Vec<&str> = options.iter().map(|o| o.id).collect();
        let want: Vec<&str> = construct::variants_of(kind).map(|v| v.id).collect();
        assert_eq!(ids, want);
        // Keys are unique in a schema.
        let mut keys: Vec<&str> = schema.fields.iter().map(|f| f.key.0).collect();
        let len = keys.len();
        keys.sort_unstable();
        keys.dedup();
        assert_eq!(keys.len(), len, "{kind}");
    }
}

#[test]
fn every_variant_shows_only_its_own_fields_and_round_trips() {
    let doc = Document::new("dialogs");
    for var in &VARIANTS {
        let picks: Vec<EntityRef> = (0..)
            .zip(var.inputs)
            .map(|(n, i)| entity(i.accepts, n))
            .collect();
        let v = variant_values(var.kind, var.id, &init(&doc, &picks), Unfilled::Pick).unwrap();
        // Shown: the type, per input its chooser (if any) and selection, and the expressions.
        let mut want = vec![TYPE];
        for i in var.inputs {
            if i.optional || !i.accepts.origins().is_empty() {
                want.push(i.source);
            }
            want.push(i.key);
        }
        want.extend::<&[&str]>(match (var.kind, var.id) {
            (PLANE_KIND, "offset") => &[construct::OFFSET_DISTANCE],
            (PLANE_KIND, "at_angle") => &[construct::AT_ANGLE_ANGLE],
            (PLANE_KIND, "tangent") => &[construct::TANGENT_ANGLE],
            (_, "along_path") => &[construct::POSITION_BY, construct::FRACTION],
            _ => &[],
        });
        assert_eq!(shown(var.kind, &v), want, "{}", var.label);
        // Every input took its preselected entity, in order.
        let built =
            (reg(var.kind).build)(&v, None).unwrap_or_else(|e| panic!("{}: {e:?}", var.label));
        let expected: Vec<RefInput> = picks.iter().cloned().map(RefInput::Ref).collect();
        assert_eq!(refs(&built), expected, "{}", var.label);
        let read = (reg(var.kind).read)(built.get(), doc.design()).unwrap();
        let again = (reg(var.kind).build)(&read, Some(built.get())).unwrap();
        assert_eq!(again, built, "{}", var.label);
        assert_eq!(read.get(TYPE), Some(&FieldValue::Choice(var.id)));
    }
}

#[test]
fn every_origin_option_builds_an_origin_input() {
    let doc = Document::new("origins");
    for var in &VARIANTS {
        for (n, input) in var.inputs.iter().enumerate() {
            for &o in input.accepts.origins() {
                let picks: Vec<EntityRef> = (0..)
                    .zip(var.inputs)
                    .map(|(k, i)| entity(i.accepts, k))
                    .collect();
                let v = variant_values(var.kind, var.id, &init(&doc, &picks), Unfilled::Pick)
                    .unwrap()
                    .with(input.source, FieldValue::Choice(origin_id(o)));
                assert!(!shown(var.kind, &v).contains(&input.key), "hidden");
                let built = (reg(var.kind).build)(&v, None).unwrap();
                assert_eq!(refs(&built)[n], RefInput::Origin(o), "{}", var.label);
                // Reading it back chooses the same origin.
                let read = (reg(var.kind).read)(built.get(), doc.design()).unwrap();
                assert_eq!(
                    read.get(input.source),
                    Some(&FieldValue::Choice(origin_id(o)))
                );
            }
        }
    }
}

#[test]
fn an_empty_pick_is_a_field_error_and_defaults_are_complete() {
    let doc = Document::new("empty");
    for var in &VARIANTS {
        let v = variant_values(var.kind, var.id, &init(&doc, &[]), Unfilled::Pick).unwrap();
        let errors = (reg(var.kind).build)(&v, None).unwrap_err();
        let keys: Vec<&str> = errors.0.iter().map(|(k, _)| k.0).collect();
        let required: Vec<&str> = var
            .inputs
            .iter()
            .filter(|i| !i.optional)
            .map(|i| i.key)
            .collect();
        assert_eq!(keys, required, "{}", var.label);
        for i in var.inputs.iter().filter(|i| !i.optional) {
            assert_eq!(v.get(i.source), Some(&FieldValue::Choice(PICK)));
        }
    }
    // The generic dialogs open complete, on origin geometry.
    for kind in [PLANE_KIND, AXIS_KIND, POINT_KIND] {
        let v = (reg(kind).defaults)(&init(&doc, &[]));
        let built = (reg(kind).build)(&v, None).unwrap();
        assert_eq!((reg(kind).read)(built.get(), doc.design()).unwrap(), v);
    }
    // Another kind's feature is not read.
    let v = (reg(PLANE_KIND).defaults)(&init(&doc, &[]));
    let plane = (reg(PLANE_KIND).build)(&v, None).unwrap();
    assert!((reg(AXIS_KIND).read)(plane.get(), doc.design()).is_none());
}

#[test]
fn preselection_fills_matching_inputs_in_any_order() {
    let doc = Document::new("order");
    // Point first, then the face: each goes where it fits.
    let picks = [point(1), topo(TopoKind::Face, 2)];
    let v = variant_values(
        PLANE_KIND,
        "tangent_at_point",
        &init(&doc, &picks),
        Unfilled::Pick,
    )
    .unwrap();
    assert_eq!(
        v.get("tap_face"),
        Some(&FieldValue::Selection(vec![picks[1].clone()]))
    );
    assert_eq!(
        v.get("tap_point"),
        Some(&FieldValue::Selection(vec![picks[0].clone()]))
    );
    // A second line is not forced into a point input.
    let v = variant_values(
        AXIS_KIND,
        "two_points",
        &init(&doc, &[curve(1)]),
        Unfilled::Pick,
    )
    .unwrap();
    assert_eq!(v.get("line_a"), None);
    assert!(variant_values(AXIS_KIND, "offset", &init(&doc, &[]), Unfilled::Pick).is_none());
}

#[test]
fn construction_preselections_fit_by_their_geometry() {
    amg_features::construct::link();
    let mut doc = Document::new("fits");
    let root = doc.design().root();
    let mut add = |kind: &str| {
        let doc_ref = &mut doc;
        let init = DialogInit {
            preselection: &[],
            design: doc_ref.design(),
            evaluated: doc_ref.evaluated(),
        };
        let v = (reg(kind).defaults)(&init);
        let feature = (reg(kind).build)(&v, None).unwrap();
        let out = doc_ref
            .apply(Command::AddFeature {
                at: None,
                feature,
                context: root,
                name: None,
            })
            .unwrap();
        match out.created.first() {
            Some(CreatedId::Feature(id)) => EntityRef::Construction(ConstructionId(*id)),
            other => panic!("{other:?}"),
        }
    };
    let (plane, axis, pt) = (add(PLANE_KIND), add(AXIS_KIND), add(POINT_KIND));
    let jobs = amg_jobs::JobSystem::new(amg_jobs::JobSystemConfig::default()).unwrap();
    doc.recompute_blocking(&jobs).unwrap();
    let eval = doc.evaluated();
    for (accepts, want) in [
        (Accepts::Plane, [true, false, false]),
        (Accepts::Line, [false, true, false]),
        (Accepts::Point, [false, false, true]),
        (Accepts::PlaneOrPoint, [true, false, true]),
        (Accepts::Face, [false, false, false]),
        (Accepts::Path, [false, false, false]),
    ] {
        let got = [&plane, &axis, &pt].map(|e| accepts.fits(e, eval));
        assert_eq!(got, want, "{accepts:?}");
    }

    // Offset arrow: on a construction base, along its normal; the angle ring about the line.
    let init = DialogInit {
        preselection: std::slice::from_ref(&plane),
        design: doc.design(),
        evaluated: doc.evaluated(),
    };
    let r = reg(PLANE_KIND);
    let v = variant_values(PLANE_KIND, "offset", &init, Unfilled::Pick).unwrap();
    let f = (r.build)(&v, None).unwrap();
    let anchors = (r.anchors)(f.get(), doc.design(), eval);
    assert_eq!(anchors.len(), 1);
    assert_eq!(anchors[0].field, FieldKey(construct::OFFSET_DISTANCE));
    // The default plane is XY + 10 mm.
    assert!((anchors[0].origin - Point3::new(0.0, 0.0, 10.0)).length() < 1e-9);
    assert!((anchors[0].axis.get() - Vec3::Z).length() < 1e-12);
    let spec = (r.schema)().field(anchors[0].field).unwrap();
    assert_eq!(spec.manipulator, Some(ManipKind::Distance));

    let init = DialogInit {
        preselection: std::slice::from_ref(&axis),
        design: doc.design(),
        evaluated: eval,
    };
    let v = variant_values(PLANE_KIND, "at_angle", &init, Unfilled::Pick).unwrap();
    let f = (r.build)(&v, None).unwrap();
    let anchors = (r.anchors)(f.get(), doc.design(), eval);
    assert_eq!(anchors.len(), 1);
    assert_eq!(anchors[0].field, FieldKey(construct::AT_ANGLE_ANGLE));
    assert!((anchors[0].axis.get() - Vec3::X).length() < 1e-12);
    assert!(anchors[0].ref_dir.unwrap().get().dot(Vec3::X).abs() < 1e-12);

    // A face base needs a kernel query: no manipulator.
    let face = [topo(TopoKind::Face, 1)];
    let init = DialogInit {
        preselection: &face,
        design: doc.design(),
        evaluated: eval,
    };
    let v = variant_values(PLANE_KIND, "offset", &init, Unfilled::Pick).unwrap();
    let f = (r.build)(&v, None).unwrap();
    assert!((r.anchors)(f.get(), doc.design(), eval).is_empty());
}
