//! CON-02: `construct.axis` and `construct.point` in a real `Document`: registration, compute
//! from origin and construction inputs (including other axes and points), health on bad
//! inputs, parameter edits through the DOC-03 recompute counter, serialisation and recompute
//! from the persisted cache.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "tests"
)]

use std::sync::OnceLock;

use amg_core::{Line3, Point3, Vec3};
use amg_doc::{
    Command, CommandOutcome, ConstructionGeom, ConstructionId, CreatedId, Document, EntityRef,
    Expr, ExprSrc, Feature, FeatureBox, FeatureId, Health, ParamId, SaveOptions,
};
use amg_features::construct::{
    AxisDef, ConstructAxis, ConstructPlane, ConstructPoint, OriginEntity, PathPosition, PlaneDef,
    PointDef, RefInput,
};
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

const XY: RefInput = RefInput::Origin(OriginEntity::XYPlane);
const YZ: RefInput = RefInput::Origin(OriginEntity::YZPlane);
const XZ: RefInput = RefInput::Origin(OriginEntity::XZPlane);

fn add<F: Feature>(doc: &mut Document, f: F) -> FeatureId {
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

fn axis(def: AxisDef) -> ConstructAxis {
    ConstructAxis { def }
}

fn point(def: PointDef) -> ConstructPoint {
    ConstructPoint { def }
}

fn created_param(out: CommandOutcome) -> ParamId {
    match out.created[0] {
        CreatedId::Param(id) => id,
        ref other => panic!("{other:?}"),
    }
}

fn axis_of(doc: &Document, id: FeatureId) -> Line3 {
    match doc.evaluated().construction(ConstructionId(id)) {
        Some(ConstructionGeom::Axis(l)) => *l,
        other => panic!("{other:?}"),
    }
}

fn point_of(doc: &Document, id: FeatureId) -> Point3 {
    match doc.evaluated().construction(ConstructionId(id)) {
        Some(ConstructionGeom::Point(p)) => *p,
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
fn the_kinds_are_registered() {
    amg_features::construct::link();
    let reg = amg_doc::try_registry().unwrap();
    assert_eq!(reg.get("construct.axis").unwrap().version, 1);
    assert_eq!(reg.get("construct.point").unwrap().version, 1);
}

/// The chain built by [`chain`].
struct Chain {
    height: ParamId,
    /// XY offset by `height`.
    plane: FeatureId,
    /// `plane` ∩ YZ (direction +Y).
    axis: FeatureId,
    /// `plane` ∩ YZ ∩ XZ = (0, 0, h).
    corner: FeatureId,
    /// Z axis × `plane` = (0, 0, h).
    pierce: FeatureId,
    /// Through `corner` towards `far`.
    diagonal: FeatureId,
    /// `axis` × `diagonal` = `corner`.
    meet: FeatureId,
    /// Independent, and an error: 3 mm along the (infinite) X axis.
    along: FeatureId,
    /// Independent: the X axis × (YZ offset by 2 mm) = (2, 0, 0).
    far: FeatureId,
    /// Independent helper: YZ offset by 2 mm.
    yz2: FeatureId,
}

impl Chain {
    fn all(&self) -> Vec<FeatureId> {
        let mut v = vec![
            self.plane,
            self.axis,
            self.corner,
            self.pierce,
            self.diagonal,
            self.meet,
            self.along,
            self.far,
            self.yz2,
        ];
        v.sort();
        v
    }
}

fn chain() -> (Document, Chain) {
    amg_features::construct::link();
    let mut doc = Document::new("con02");
    let height = created_param(
        doc.apply(Command::AddParam {
            name: "height".to_owned(),
            expr: "5 mm".to_owned(),
            unit: None,
            comment: String::new(),
        })
        .unwrap(),
    );
    let plane = add(
        &mut doc,
        ConstructPlane {
            def: PlaneDef::Offset {
                base: XY,
                distance: inline("height"),
            },
        },
    );
    let axis_id = add(
        &mut doc,
        axis(AxisDef::ThroughTwoPlanes {
            a: construction(plane),
            b: YZ,
        }),
    );
    let corner = add(
        &mut doc,
        point(PointDef::ThroughThreePlanes {
            a: construction(plane),
            b: YZ,
            c: XZ,
        }),
    );
    let pierce = add(
        &mut doc,
        point(PointDef::EdgeAndPlane {
            edge: RefInput::Origin(OriginEntity::ZAxis),
            plane: construction(plane),
        }),
    );
    let yz2 = add(
        &mut doc,
        ConstructPlane {
            def: PlaneDef::Offset {
                base: YZ,
                distance: inline("2 mm"),
            },
        },
    );
    let far = add(
        &mut doc,
        point(PointDef::EdgeAndPlane {
            edge: RefInput::Origin(OriginEntity::XAxis),
            plane: construction(yz2),
        }),
    );
    let diagonal = add(
        &mut doc,
        axis(AxisDef::ThroughTwoPoints {
            a: construction(corner),
            b: construction(far),
        }),
    );
    let meet = add(
        &mut doc,
        point(PointDef::ThroughTwoEdges {
            a: construction(axis_id),
            b: construction(diagonal),
        }),
    );
    let along = add(
        &mut doc,
        point(PointDef::AlongPath {
            path: RefInput::Origin(OriginEntity::XAxis),
            at: PathPosition::Distance(inline("3 mm")),
        }),
    );
    doc.recompute_blocking(jobs()).unwrap();
    let c = Chain {
        height,
        plane,
        axis: axis_id,
        corner,
        pierce,
        diagonal,
        meet,
        along,
        far,
        yz2,
    };
    (doc, c)
}

#[test]
fn axes_and_points_compute_from_origin_and_construction_inputs() {
    let (doc, c) = chain();
    for id in [
        c.plane, c.axis, c.corner, c.pierce, c.diagonal, c.meet, c.far,
    ] {
        assert_eq!(health(&doc, id), Health::Ok, "{id:?}");
    }
    let a = axis_of(&doc, c.axis);
    assert_eq!(a.dir.get(), Vec3::Y);
    assert_eq!(a.point, Point3::new(0.0, 0.0, 5.0));
    assert_eq!(point_of(&doc, c.corner), Point3::new(0.0, 0.0, 5.0));
    assert_eq!(point_of(&doc, c.pierce), Point3::new(0.0, 0.0, 5.0));
    assert_eq!(point_of(&doc, c.far), Point3::new(2.0, 0.0, 0.0));
    let d = axis_of(&doc, c.diagonal);
    assert_eq!(d.point, Point3::new(0.0, 0.0, 5.0));
    let want = Vec3::new(2.0, 0.0, -5.0).normalize();
    assert!((d.dir.get() - want).abs().max_element() <= 1e-12);
    let m = point_of(&doc, c.meet);
    assert!(
        (m - Point3::new(0.0, 0.0, 5.0)).abs().max_element() <= 1e-12,
        "{m:?}"
    );
    // An infinite origin axis is not a path.
    let Health::Error(diag) = health(&doc, c.along) else {
        panic!("{:?}", health(&doc, c.along))
    };
    assert!(
        diag.message.contains("edge or sketch curve"),
        "{}",
        diag.message
    );
}

#[test]
fn a_parameter_edit_recomputes_only_the_dependants() {
    let (mut doc, c) = chain();
    let mut first = computed(&doc);
    first.sort();
    // A feature whose compute failed is not reported as computed.
    let mut ok = c.all();
    ok.retain(|id| *id != c.along);
    assert_eq!(first, ok);

    doc.apply(Command::SetParamExpr {
        id: c.height,
        expr: "8 mm".to_owned(),
    })
    .unwrap();
    doc.recompute_blocking(jobs()).unwrap();
    let mut again = computed(&doc);
    again.sort();
    let mut want = vec![c.plane, c.axis, c.corner, c.pierce, c.diagonal, c.meet];
    want.sort();
    assert_eq!(again, want);
    let cached = &doc.last_run().unwrap().cached;
    for id in [c.far, c.yz2] {
        assert!(cached.contains(&id), "{id:?} not cached: {cached:?}");
    }
    assert_eq!(axis_of(&doc, c.axis).point.z, 8.0);
    assert_eq!(point_of(&doc, c.corner), Point3::new(0.0, 0.0, 8.0));
    assert_eq!(point_of(&doc, c.pierce), Point3::new(0.0, 0.0, 8.0));
    let m = point_of(&doc, c.meet);
    assert!(
        (m - Point3::new(0.0, 0.0, 8.0)).abs().max_element() <= 1e-12,
        "{m:?}"
    );
}

#[test]
fn degenerate_inputs_are_feature_errors() {
    let (mut doc, c) = chain();
    // Parallel planes have no common line.
    let parallel = add(
        &mut doc,
        axis(AxisDef::ThroughTwoPlanes {
            a: XY,
            b: construction(c.plane),
        }),
    );
    // The same point twice.
    let same = add(
        &mut doc,
        axis(AxisDef::ThroughTwoPoints {
            a: construction(c.corner),
            b: construction(c.pierce),
        }),
    );
    // XY, the offset plane and YZ: two are parallel.
    let degenerate = add(
        &mut doc,
        point(PointDef::ThroughThreePlanes {
            a: XY,
            b: construction(c.plane),
            c: YZ,
        }),
    );
    doc.recompute_blocking(jobs()).unwrap();
    for id in [parallel, same, degenerate] {
        let Health::Error(d) = health(&doc, id) else {
            panic!("{:?}", health(&doc, id))
        };
        assert!(!d.message.is_empty());
        assert!(doc.evaluated().construction(ConstructionId(id)).is_none());
    }
}

#[test]
fn the_features_round_trip_and_recompute_from_the_cache_bit_identically() {
    let (doc, c) = chain();
    let ok = [c.axis, c.corner, c.pierce, c.diagonal, c.meet, c.far];
    for id in c.all() {
        let (_, node) = doc.design().timeline().node(id).unwrap();
        let f = node.feature.get();
        let json = f.to_json().unwrap();
        let kind = node.feature.kind();
        let back = FeatureBox::from_parts(kind, 1, json).unwrap();
        if let Some(a) = node.feature.downcast_ref::<ConstructAxis>() {
            assert_eq!(back.downcast_ref::<ConstructAxis>(), Some(a));
        } else if let Some(p) = node.feature.downcast_ref::<ConstructPoint>() {
            assert_eq!(back.downcast_ref::<ConstructPoint>(), Some(p));
        } else {
            assert_eq!(kind, "construct.plane");
        }
    }

    let dir = std::env::temp_dir().join(format!("amg-con02-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("axes_points.amg");
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

    let mut cached = loaded.into_document();
    cached.recompute_blocking(jobs()).unwrap();
    let mut fresh = Document::from_design(amg_doc::load(&path).unwrap().design);
    fresh.recompute_blocking(jobs()).unwrap();
    assert!(!computed(&fresh).is_empty());
    let bits = |doc: &Document, id: FeatureId| -> Vec<u64> {
        match doc.evaluated().construction(ConstructionId(id)) {
            Some(ConstructionGeom::Axis(l)) => [l.point, l.dir.get()]
                .iter()
                .flat_map(|v| v.to_array())
                .map(f64::to_bits)
                .collect(),
            Some(ConstructionGeom::Point(p)) => p.to_array().map(f64::to_bits).to_vec(),
            other => panic!("{other:?}"),
        }
    };
    for id in ok {
        let want = bits(&doc, id);
        assert_eq!(bits(&cached, id), want, "cached {id:?}");
        assert_eq!(bits(&fresh, id), want, "fresh {id:?}");
    }
    std::fs::remove_dir_all(&dir).ok();
}
