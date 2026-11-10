//! CON-01: `construct.plane` in a real `Document`: registration, compute from origin and
//! construction inputs, health on bad inputs, parameter edits through the DOC-03 recompute
//! counter, serialisation and recompute from the persisted cache.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "tests"
)]

use std::sync::OnceLock;

use amg_core::{Plane, Point3, Vec3};
use amg_doc::{
    Command, CommandOutcome, ConstructionGeom, ConstructionId, CreatedId, Document, EntityRef,
    Expr, ExprSrc, FeatureBox, FeatureId, Health, ParamId, SaveOptions,
};
use amg_features::construct::{ConstructPlane, OriginEntity, PlaneDef, RefInput};
use amg_jobs::{JobSystem, JobSystemConfig};

fn jobs() -> &'static JobSystem {
    static JOBS: OnceLock<JobSystem> = OnceLock::new();
    JOBS.get_or_init(|| JobSystem::new(JobSystemConfig::default()).unwrap())
}

fn inline(s: &str) -> Expr {
    Expr::Inline(ExprSrc(s.to_owned()))
}

fn construction(id: FeatureId) -> RefInput {
    RefInput::Ref(EntityRef::Construction(ConstructionId(id)))
}

fn offset(base: RefInput, d: &str) -> ConstructPlane {
    ConstructPlane {
        def: PlaneDef::Offset {
            base,
            distance: inline(d),
        },
    }
}

fn add(doc: &mut Document, f: ConstructPlane) -> FeatureId {
    let root = doc.design().root();
    let out = doc
        .apply(Command::AddFeature {
            at: None,
            feature: FeatureBox::new(f),
            context: root,
            name: None,
        })
        .unwrap();
    match out.created[0] {
        CreatedId::Feature(id) => id,
        ref other => panic!("{other:?}"),
    }
}

fn created_param(out: CommandOutcome) -> ParamId {
    match out.created[0] {
        CreatedId::Param(id) => id,
        ref other => panic!("{other:?}"),
    }
}

fn plane(doc: &Document, id: FeatureId) -> Plane {
    match doc.evaluated().construction(ConstructionId(id)) {
        Some(ConstructionGeom::Plane(p)) => *p,
        other => panic!("{other:?}"),
    }
}

fn health(doc: &Document, id: FeatureId) -> Health {
    doc.evaluated().node(id).unwrap().health.clone()
}

fn computed(doc: &Document) -> Vec<FeatureId> {
    doc.last_run().unwrap().computed.clone()
}

#[test]
fn the_kind_is_registered() {
    amg_features::construct::link();
    let reg = amg_doc::try_registry().unwrap();
    assert_eq!(reg.get("construct.plane").unwrap().version, 1);
}

/// Builds: height = 5 mm; A = XY offset by height; B = A offset by 2 mm; C = midplane(XY, A);
/// D = YZ offset by 1 mm (independent).
fn chain() -> (Document, ParamId, [FeatureId; 4]) {
    amg_features::construct::link();
    let mut doc = Document::new("con01");
    let h = created_param(
        doc.apply(Command::AddParam {
            name: "height".to_owned(),
            expr: "5 mm".to_owned(),
            unit: None,
            comment: String::new(),
        })
        .unwrap(),
    );
    let a = add(
        &mut doc,
        offset(RefInput::Origin(OriginEntity::XYPlane), "height"),
    );
    let b = add(&mut doc, offset(construction(a), "2 mm"));
    let c = add(
        &mut doc,
        ConstructPlane {
            def: PlaneDef::Midplane {
                a: RefInput::Origin(OriginEntity::XYPlane),
                b: construction(a),
            },
        },
    );
    let d = add(
        &mut doc,
        offset(RefInput::Origin(OriginEntity::YZPlane), "1 mm"),
    );
    doc.recompute_blocking(jobs()).unwrap();
    (doc, h, [a, b, c, d])
}

#[test]
fn offset_and_midplane_compute_from_origin_and_construction_inputs() {
    let (doc, _, [a, b, c, d]) = chain();
    for id in [a, b, c, d] {
        assert_eq!(health(&doc, id), Health::Ok, "{id:?}");
    }
    assert_eq!(plane(&doc, a).origin, Point3::new(0.0, 0.0, 5.0));
    assert_eq!(plane(&doc, b).origin, Point3::new(0.0, 0.0, 7.0));
    assert_eq!(plane(&doc, c).origin, Point3::new(0.0, 0.0, 2.5));
    assert_eq!(plane(&doc, c).normal.get(), Vec3::Z);
    assert_eq!(plane(&doc, d).origin, Point3::new(1.0, 0.0, 0.0));
    assert_eq!(plane(&doc, d).normal.get(), Vec3::X);
}

#[test]
fn a_parameter_edit_recomputes_only_the_plane_and_its_dependants() {
    let (mut doc, h, [a, b, c, d]) = chain();
    assert_eq!(computed(&doc), vec![a, b, c, d]);
    // The inline distance became a model parameter (exprs_mut).
    let ConstructPlane {
        def: PlaneDef::Offset { distance, .. },
    } = doc
        .design()
        .timeline()
        .node(a)
        .unwrap()
        .1
        .feature
        .downcast_ref::<ConstructPlane>()
        .unwrap()
        .clone()
    else {
        panic!()
    };
    assert!(matches!(distance, Expr::Param(_)), "{distance:?}");

    doc.apply(Command::SetParamExpr {
        id: h,
        expr: "8 mm".to_owned(),
    })
    .unwrap();
    doc.recompute_blocking(jobs()).unwrap();
    assert_eq!(computed(&doc), vec![a, b, c]);
    assert_eq!(doc.last_run().unwrap().cached, vec![d]);
    assert_eq!(plane(&doc, a).origin.z, 8.0);
    assert_eq!(plane(&doc, b).origin.z, 10.0);
    assert_eq!(plane(&doc, c).origin.z, 4.0);
}

#[test]
fn a_degenerate_input_is_a_feature_error() {
    let (mut doc, _, [a, ..]) = chain();
    // The X axis does not lie in plane A (z = 5).
    let bad = add(
        &mut doc,
        ConstructPlane {
            def: PlaneDef::AtAngle {
                line: RefInput::Origin(OriginEntity::XAxis),
                zero: Some(construction(a)),
                angle: inline("30 deg"),
            },
        },
    );
    // Midplane of a plane with itself.
    let same = add(
        &mut doc,
        ConstructPlane {
            def: PlaneDef::Midplane {
                a: construction(a),
                b: construction(a),
            },
        },
    );
    doc.recompute_blocking(jobs()).unwrap();
    for id in [bad, same] {
        let Health::Error(d) = health(&doc, id) else {
            panic!("{:?}", health(&doc, id))
        };
        assert!(!d.message.is_empty());
        assert!(doc.evaluated().construction(ConstructionId(id)).is_none());
    }
    // From the XY plane the same at-angle plane works.
    let good = add(
        &mut doc,
        ConstructPlane {
            def: PlaneDef::AtAngle {
                line: RefInput::Origin(OriginEntity::XAxis),
                zero: Some(RefInput::Origin(OriginEntity::XYPlane)),
                angle: inline("30 deg"),
            },
        },
    );
    doc.recompute_blocking(jobs()).unwrap();
    let (s, c) = 30f64.to_radians().sin_cos();
    let n = plane(&doc, good).normal.get();
    assert!(
        (n - Vec3::new(0.0, -s, c)).abs().max_element() <= 1e-12,
        "{n:?}"
    );
}

#[test]
fn the_feature_round_trips_and_recomputes_from_the_cache_bit_identically() {
    let (doc, _, ids) = chain();
    // Feature data round-trip through the registry.
    for id in ids {
        let (_, node) = doc.design().timeline().node(id).unwrap();
        let json = node.feature.get().to_json().unwrap();
        let back = FeatureBox::from_parts("construct.plane", 1, json).unwrap();
        assert_eq!(
            back.downcast_ref::<ConstructPlane>(),
            node.feature.downcast_ref::<ConstructPlane>()
        );
    }

    let dir = std::env::temp_dir().join(format!("amg-con01-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("planes.amg");
    let opts = SaveOptions {
        cache: Some(doc.cache_export()),
        ..SaveOptions::default()
    };
    amg_doc::save(&path, &doc.snapshot(), &opts).unwrap();
    let loaded = amg_doc::load(&path).unwrap();
    assert!(loaded.warnings.is_empty(), "{:?}", loaded.warnings);
    assert_eq!(
        serde_json::to_value(&loaded.design).unwrap(),
        serde_json::to_value(doc.design()).unwrap()
    );

    // Opening with the cache computes nothing and gives the same planes.
    let mut cached = loaded.into_document();
    cached.recompute_blocking(jobs()).unwrap();
    assert!(computed(&cached).is_empty(), "{:?}", computed(&cached));
    // Opening without it recomputes everything, bit-identically.
    let mut fresh = Document::from_design(amg_doc::load(&path).unwrap().design);
    fresh.recompute_blocking(jobs()).unwrap();
    assert_eq!(computed(&fresh).len(), ids.len());
    for id in ids {
        let want = plane(&doc, id);
        for other in [&cached, &fresh] {
            let got = plane(other, id);
            let bits = |p: &Plane| {
                [p.origin, p.x.get(), p.y.get(), p.normal.get()]
                    .map(|v| v.to_array().map(f64::to_bits))
            };
            assert_eq!(bits(&got), bits(&want), "{id:?}");
        }
    }
    std::fs::remove_dir_all(&dir).ok();
}
