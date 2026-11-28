//! SOL-23: `solid.pattern` along a path (spacing/extent, start, orientation), face patterns
//! (boss/pocket on a planar support) and "adjust" compute; one test (or test group) per
//! Done-when item, see the test names. Geometry needs the `occt` feature (AMG_OCCT_DIR); the
//! dialog and data tests run everywhere.

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
    BodyId, Dependency, Document, EntityRef, Expr, ExprSrc, Feature, FeatureId, SketchId, TopoKind,
    TopoRef,
};
use amg_features::dialog::{self, DialogInit, FieldValue, FieldValues};
use amg_features::solid::{
    ComputeType, PathOrientation, Pattern, PatternKind, PatternObjects, Spacing,
};
#[allow(unused_imports, reason = "the occt tests use it through `super::*`")]
use solid_common::*;

fn ex(s: &str) -> Expr {
    Expr::Inline(ExprSrc(s.to_owned()))
}

fn path_kind(path: Vec<EntityRef>, count: &str, spacing: Spacing, start: &str) -> PatternKind {
    PatternKind::Path {
        path,
        count: ex(count),
        spacing,
        start: ex(start),
        orientation: PathOrientation::PathDirection,
        flip: false,
    }
}

fn curve_ref(sketch: FeatureId, curve: u64) -> EntityRef {
    EntityRef::SketchCurve {
        sketch: SketchId(sketch),
        curve: SketchEntityKey(curve),
    }
}

fn a_body() -> BodyId {
    BodyId {
        origin: FeatureId(3),
        slot: 0,
    }
}

fn a_face() -> TopoRef {
    TopoRef {
        body: a_body(),
        kind: TopoKind::Face,
        name: amg_doc::naming::TopoName::new(Vec::new()),
        fingerprint: None,
        key: 1,
    }
}

// ---- dialog and data (every build) ------------------------------------------------------------

fn defaults() -> FieldValues {
    let doc = Document::new("defaults");
    let reg = dialog::registration("solid.pattern").unwrap();
    (reg.defaults)(&DialogInit {
        preselection: &[],
        design: doc.design(),
        evaluated: doc.evaluated(),
    })
}

fn round_trip(v: &FieldValues) -> Pattern {
    let doc = Document::new("rt");
    let reg = dialog::registration("solid.pattern").unwrap();
    let f = (reg.build)(v, None).unwrap_or_else(|e| panic!("{e:?}"));
    f.validate().unwrap();
    let back = (reg.read)(f.get(), doc.design()).unwrap();
    assert_eq!(&back, v, "read(build(v)) == v");
    f.get().as_any().downcast_ref::<Pattern>().unwrap().clone()
}

#[test]
fn dialog_round_trips_path_patterns() {
    let path = vec![curve_ref(FeatureId(4), 7), curve_ref(FeatureId(4), 9)];
    let v = defaults()
        .with("pattern_type", FieldValue::Choice("path"))
        .with("path", FieldValue::Selection(path.clone()))
        .with("path_count", FieldValue::expr("8"))
        .with("path_spacing_type", FieldValue::Choice("extent"))
        .with("path_distance", FieldValue::expr("70 mm"))
        .with("path_start", FieldValue::expr("0.25"))
        .with("path_orientation", FieldValue::Choice("path_direction"))
        .with("path_flip", FieldValue::Flip(true));
    let p = round_trip(&v);
    assert_eq!(
        p.kind,
        PatternKind::Path {
            path,
            count: ex("8"),
            spacing: Spacing::Extent(ex("70 mm")),
            start: ex("0.25"),
            orientation: PathOrientation::PathDirection,
            flip: true,
        }
    );
    // Defaults with the path type: identical orientation, spacing, start 0.
    let p = round_trip(&defaults().with("pattern_type", FieldValue::Choice("path")));
    let PatternKind::Path {
        orientation, flip, ..
    } = p.kind
    else {
        panic!("{:?}", p.kind)
    };
    assert_eq!((orientation, flip), (PathOrientation::Identical, false));
}

#[test]
fn dialog_round_trips_faces_and_adjust_compute() {
    let faces = defaults()
        .with("object_type", FieldValue::Choice("faces"))
        .with(
            "faces",
            FieldValue::Selection(vec![EntityRef::Topo(a_face())]),
        );
    let p = round_trip(&faces);
    assert_eq!(p.objects, PatternObjects::Faces(vec![a_face()]));
    let adjust = defaults()
        .with("object_type", FieldValue::Choice("features"))
        .with("features", FieldValue::Text("12".into()))
        .with("compute", FieldValue::Choice("adjust"));
    let p = round_trip(&adjust);
    assert_eq!(p.compute, ComputeType::Adjust);
    // The mirror dialog doesn't offer faces.
    let schema = (dialog::registration("solid.mirror").unwrap().schema)();
    assert!(schema.fields.iter().all(|f| f.key.0 != "faces"));
}

#[test]
fn path_patterns_list_their_curves_and_exprs() {
    let p = Pattern {
        objects: PatternObjects::Bodies(vec![a_body()]),
        kind: path_kind(
            vec![curve_ref(FeatureId(4), 7)],
            "8",
            Spacing::Spacing(ex("15 mm")),
            "0",
        ),
        suppressed: Vec::new(),
        compute: ComputeType::Identical,
        target: None,
    };
    assert_eq!(
        p.inputs(),
        vec![
            Dependency::Body(a_body()),
            Dependency::Entity(curve_ref(FeatureId(4), 7))
        ]
    );
    assert_eq!(p.exprs().len(), 3);
    let json = serde_json::to_string(&p).unwrap();
    assert_eq!(serde_json::from_str::<Pattern>(&json).unwrap(), p);
    // A face pattern lists its faces and reads no context bodies.
    let f = Pattern {
        objects: PatternObjects::Faces(vec![a_face()]),
        ..p
    };
    assert_eq!(f.inputs()[0], Dependency::Entity(EntityRef::Topo(a_face())));
    assert_eq!(f.context_reads(), None);
    f.validate().unwrap();
}

#[cfg(feature = "occt")]
#[path = "solid_pattern_sol23/occt.rs"]
mod occt;
