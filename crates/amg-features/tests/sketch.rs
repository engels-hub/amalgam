//! SK-19: the sketch feature in the timeline (`amg_features::sketch`), one test per Done-when
//! item of the work package (see the test names).

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "tests"
)]

use std::sync::{Arc, OnceLock};

use amg_core::Point2;
use amg_doc::stub::SketchGeom;
use amg_doc::{
    Command, CommandError, CommandOutcome, CreatedId, Design, Document, Expr, ExprSrc, FeatureBox,
    FeatureId, Health, ParamId, ParamKind, SaveOptions, SketchId,
};
use amg_features::sketch::{PlaneInput, SketchEdit, SketchFeature};
use amg_jobs::{JobSystem, JobSystemConfig};
use amg_sketch::solve::SolveStatus;
use amg_sketch::{
    AngleBranch, ConstraintId, ConstraintKind, Dimension, DimensionKind, EntityId, EntityKind,
    build,
};
use proptest::prelude::*;

fn jobs() -> &'static JobSystem {
    static JOBS: OnceLock<JobSystem> = OnceLock::new();
    JOBS.get_or_init(|| JobSystem::new(JobSystemConfig::default()).unwrap())
}

fn inline(s: &str) -> Expr {
    Expr::Inline(ExprSrc(s.to_owned()))
}

fn len_dim(line: EntityId, value: f64) -> ConstraintKind {
    ConstraintKind::Dimension(Dimension {
        kind: DimensionKind::Length { line },
        value,
        driving: true,
        label_pos: Point2::new(0.0, 0.0),
    })
}

/// A 40 × 20 rectangle at the origin whose width and height are the given expressions.
fn rect_feature(w: &str, h: &str) -> SketchFeature {
    let mut f = SketchFeature::new(PlaneInput::OriginXY);
    let built = f
        .edit_sketch(|s| build::rectangle_2pt(s, Point2::new(0.0, 0.0), Point2::new(40.0, 20.0)))
        .unwrap();
    // entities: 4 points, then the lines bottom, right, top, left.
    let (bottom, right) = (built.entities[4], built.entities[5]);
    f.apply(SketchEdit::AddConstraint {
        kind: len_dim(bottom, 40.0),
        expr: Some(inline(w)),
    })
    .unwrap();
    f.apply(SketchEdit::AddConstraint {
        kind: len_dim(right, 20.0),
        expr: Some(inline(h)),
    })
    .unwrap();
    f
}

fn created_feature(o: CommandOutcome) -> FeatureId {
    o.created
        .iter()
        .find_map(|c| match c {
            CreatedId::Feature(f) => Some(*f),
            _ => None,
        })
        .expect("a feature was created")
}

fn created_param(o: CommandOutcome) -> ParamId {
    o.created
        .iter()
        .find_map(|c| match c {
            CreatedId::Param(p) => Some(*p),
            _ => None,
        })
        .expect("a parameter was created")
}

fn add_param(doc: &mut Document, name: &str, expr: &str) -> ParamId {
    created_param(
        doc.apply(Command::AddParam {
            name: name.to_owned(),
            expr: expr.to_owned(),
            unit: None,
            comment: String::new(),
        })
        .unwrap(),
    )
}

fn add_sketch(doc: &mut Document, f: SketchFeature) -> Result<FeatureId, CommandError> {
    let root = doc.design().root();
    doc.apply(Command::AddFeature {
        at: None,
        feature: FeatureBox::new(f),
        context: root,
        name: None,
    })
    .map(created_feature)
}

fn edit_feature(doc: &mut Document, id: FeatureId, f: SketchFeature) {
    doc.apply(Command::EditFeature {
        id,
        feature: FeatureBox::new(f),
    })
    .unwrap();
}

fn current(doc: &Document, id: FeatureId) -> SketchFeature {
    let (_, node) = doc.design().timeline().node(id).unwrap();
    node.feature
        .downcast_ref::<SketchFeature>()
        .unwrap()
        .clone()
}

fn geom(doc: &Document, id: FeatureId) -> Arc<SketchGeom> {
    doc.evaluated().sketch(SketchId(id)).cloned().unwrap()
}

fn health(doc: &Document, id: FeatureId) -> Health {
    doc.evaluated().node(id).unwrap().health.clone()
}

/// Area of the single profile of the sketch.
fn area(g: &SketchGeom) -> f64 {
    assert_eq!(g.profiles.profiles.len(), 1, "{:?}", g.profiles.issues);
    g.profiles.profiles[0].fingerprint.area
}

fn design_json(doc: &Document) -> serde_json::Value {
    serde_json::to_value(doc.design()).unwrap()
}

// Done-when 1.
#[test]
fn changing_a_user_parameter_recomputes_the_sketch_and_its_profiles() {
    let mut doc = Document::new("sk19");
    let w = add_param(&mut doc, "w", "40 mm");
    let id = add_sketch(&mut doc, rect_feature("w", "20 mm")).unwrap();
    doc.recompute_blocking(jobs()).unwrap();
    assert_eq!(health(&doc, id), Health::Ok);
    assert!((area(&geom(&doc, id)) - 800.0).abs() < 1e-6);

    doc.apply(Command::SetParamExpr {
        id: w,
        expr: "50 mm".to_owned(),
    })
    .unwrap();
    doc.recompute_blocking(jobs()).unwrap();
    assert_eq!(health(&doc, id), Health::Ok);
    let g = geom(&doc, id);
    assert!((area(&g) - 1000.0).abs() < 1e-6, "area {}", area(&g));
    assert!(g.report.status.is_converged());

    // A parameter the sketch does not read leaves it cached: same allocation.
    add_param(&mut doc, "unrelated", "1 mm");
    doc.recompute_blocking(jobs()).unwrap();
    assert!(Arc::ptr_eq(&g, &geom(&doc, id)));
}

// Done-when 2.
#[test]
fn a_wrong_unit_expression_is_a_feature_error() {
    // Through the command layer the model parameter is refused up front ...
    let mut doc = Document::new("sk19");
    let err = add_sketch(&mut doc, rect_feature("30 deg", "20 mm")).unwrap_err();
    assert!(matches!(err, CommandError::Expr(_)), "{err:?}");
    assert_eq!(
        doc.design().timeline().len(),
        0,
        "the refused add left no feature"
    );

    // ... and a feature that got into a design anyway fails its compute with a unit error.
    let mut d = Design::new("sk19");
    let root = d.root();
    let bad = d
        .add_feature(
            None,
            FeatureBox::new(rect_feature("30 deg", "20 mm")),
            root,
            None,
        )
        .unwrap();
    let mut f = rect_feature("40 mm", "20 mm");
    let lines: Vec<EntityId> = f
        .sketch
        .entities()
        .filter(|(_, e)| matches!(e.kind, EntityKind::Line { .. }))
        .map(|(id, _)| id)
        .collect();
    f.apply(SketchEdit::AddConstraint {
        kind: ConstraintKind::Dimension(Dimension {
            kind: DimensionKind::Angle {
                a: lines[0],
                b: lines[1],
                branch: AngleBranch {
                    reverse_a: false,
                    reverse_b: false,
                },
            },
            value: std::f64::consts::FRAC_PI_2,
            driving: true,
            label_pos: Point2::new(0.0, 0.0),
        }),
        expr: Some(inline("5 mm")),
    })
    .unwrap();
    let bad_angle = d.add_feature(None, FeatureBox::new(f), root, None).unwrap();
    let mut doc = Document::from_design(d);
    doc.recompute_blocking(jobs()).unwrap();
    for id in [bad, bad_angle] {
        match health(&doc, id) {
            Health::Error(d) => assert!(!d.message.is_empty()),
            other => panic!("expected an error, got {other:?}"),
        }
        assert!(doc.evaluated().sketch(SketchId(id)).is_none());
    }
}

#[test]
fn a_bare_angle_is_degrees_and_a_bare_length_is_millimetres() {
    let mut f = rect_feature("40", "20");
    let lines: Vec<EntityId> = f
        .sketch
        .entities()
        .filter(|(_, e)| matches!(e.kind, EntityKind::Line { .. }))
        .map(|(id, _)| id)
        .collect();
    // A right angle between the bottom and the right side, written in degrees.
    let r = f
        .apply(SketchEdit::AddConstraint {
            kind: ConstraintKind::Dimension(Dimension {
                kind: DimensionKind::Angle {
                    a: lines[0],
                    b: lines[1],
                    branch: AngleBranch {
                        reverse_a: false,
                        reverse_b: false,
                    },
                },
                value: 0.0,
                driving: true,
                label_pos: Point2::new(0.0, 0.0),
            }),
            expr: Some(inline("90 deg")),
        })
        .unwrap();
    assert_eq!(r.constraints_added.len(), 1);
    let mut doc = Document::new("sk19");
    let id = add_sketch(&mut doc, f).unwrap();
    doc.recompute_blocking(jobs()).unwrap();
    assert!((area(&geom(&doc, id)) - 800.0).abs() < 1e-6);
}

// Done-when 3.
#[test]
fn driving_dimensions_appear_as_model_parameters() {
    let mut doc = Document::new("sk19");
    let w = add_param(&mut doc, "w", "40 mm");
    let id = add_sketch(&mut doc, rect_feature("w", "2 * 10 mm")).unwrap();

    let f = current(&doc, id);
    assert_eq!(f.dims.len(), 2);
    let mut names = Vec::new();
    for e in f.dims.values() {
        let Expr::Param(p) = e else {
            panic!("dimension still inline: {e:?}")
        };
        let param = doc.design().params().get(*p).unwrap();
        assert_eq!(param.kind, ParamKind::Model { owner: id });
        names.push(param.name.clone());
    }
    assert_eq!(names, ["d1", "d2"]);

    // Editing a model parameter drives the sketch like any other parameter.
    let d1 = doc.design().params().by_name("d1").unwrap();
    let d2 = doc.design().params().by_name("d2").unwrap();
    doc.apply(Command::SetParamExpr {
        id: d2,
        expr: "30 mm".to_owned(),
    })
    .unwrap();
    doc.recompute_blocking(jobs()).unwrap();
    assert!((area(&geom(&doc, id)) - 1200.0).abs() < 1e-6);
    // d1 reads `w`; the user parameter is still in use by it.
    assert_eq!(doc.design().params().get(d1).unwrap().expr.0, "w");
    assert!(matches!(
        doc.apply(Command::DeleteParam { id: w }).unwrap_err(),
        CommandError::Expr(_) | CommandError::InUse { .. } | CommandError::Invalid(_)
    ));

    // A new dimension added by a later edit gets the next `d<n>`.
    let mut f = current(&doc, id);
    let top = f
        .sketch
        .entities()
        .filter(|(_, e)| matches!(e.kind, EntityKind::Line { .. }))
        .map(|(id, _)| id)
        .nth(2)
        .unwrap();
    f.apply(SketchEdit::AddConstraint {
        kind: len_dim(top, 40.0),
        expr: Some(inline("40 mm")),
    })
    .unwrap();
    edit_feature(&mut doc, id, f);
    assert!(doc.design().params().by_name("d3").is_some());

    // Undo takes the parameters back with the edit.
    assert!(doc.undo());
    assert!(doc.design().params().by_name("d3").is_none());
    assert!(doc.undo()); // SetParamExpr
    assert!(doc.undo()); // AddFeature
    assert!(doc.design().params().by_name("d1").is_none());
}

// Done-when 4.
#[test]
fn an_inconsistent_edit_warns_with_a_diagnosis_and_keeps_the_previous_geometry() {
    let mut doc = Document::new("sk19");
    let id = add_sketch(&mut doc, rect_feature("40 mm", "20 mm")).unwrap();
    doc.recompute_blocking(jobs()).unwrap();
    assert_eq!(health(&doc, id), Health::Ok);

    // Widen it and store the solved values as the sketch's last good geometry.
    let mut f = current(&doc, id);
    let w = *f.dims.keys().next().unwrap();
    f.apply(SketchEdit::SetDimension {
        id: w,
        expr: inline("50 mm"),
    })
    .unwrap();
    edit_feature(&mut doc, id, f);
    doc.recompute_blocking(jobs()).unwrap();
    let good = geom(&doc, id);
    assert!((area(&good) - 1000.0).abs() < 1e-6);
    let mut f = current(&doc, id);
    let commit = f.commit_solved(&good);
    f.apply(commit).unwrap();
    edit_feature(&mut doc, id, f);

    // A second, contradicting width on the same line.
    let mut f = current(&doc, id);
    let (bottom, _) = match f.sketch.constraint(w).unwrap().kind {
        ConstraintKind::Dimension(Dimension {
            kind: DimensionKind::Length { line },
            ..
        }) => (line, ()),
        _ => unreachable!(),
    };
    let r = f
        .apply(SketchEdit::AddConstraint {
            kind: len_dim(bottom, 30.0),
            expr: Some(inline("30 mm")),
        })
        .unwrap();
    let bad = r.constraints_added[0];
    edit_feature(&mut doc, id, f);
    doc.recompute_blocking(jobs()).unwrap();

    let Health::Warning(w_list) = health(&doc, id) else {
        panic!("expected a warning, got {:?}", health(&doc, id))
    };
    let diag = &w_list[0];
    assert!(diag.message.contains("contradict"), "{}", diag.message);
    assert!(
        diag.message.contains(&format!("c{}", bad.0)),
        "{}",
        diag.message
    );
    assert!(
        !diag.entities.is_empty(),
        "the conflicting entities are named"
    );
    let g = geom(&doc, id);
    assert_eq!(g.report.status, SolveStatus::Inconsistent);
    assert!(
        (area(&g) - 1000.0).abs() < 1e-6,
        "previous geometry kept, area {}",
        area(&g)
    );

    // Dropping the offending dimension heals the sketch.
    let mut f = current(&doc, id);
    f.apply(SketchEdit::RemoveConstraint(bad)).unwrap();
    edit_feature(&mut doc, id, f);
    doc.recompute_blocking(jobs()).unwrap();
    assert_eq!(health(&doc, id), Health::Ok);
}

#[test]
fn a_failed_edit_changes_nothing_and_batches_are_atomic() {
    let mut f = rect_feature("40 mm", "20 mm");
    let before = serde_json::to_value(&f).unwrap();
    let err = f
        .apply(SketchEdit::Batch(vec![
            SketchEdit::AddPoint {
                at: Point2::new(1.0, 2.0),
            },
            SketchEdit::RemoveEntities(vec![EntityId(9999)]),
        ]))
        .unwrap_err();
    assert!(matches!(err, amg_sketch::SketchError::UnknownEntity(_)));
    assert_eq!(serde_json::to_value(&f).unwrap(), before);
}

// Done-when 5.
#[derive(Clone, Debug)]
enum Op {
    Point(f64, f64),
    Line(f64, f64, f64, f64),
    Circle(f64, f64, f64),
    Remove(usize),
    LengthDim(usize, f64),
    RadiusDim(usize, f64),
    SetDim(usize, f64),
    Drive(usize, bool),
    Construction(usize, bool),
    RemoveConstraint(usize),
    Label(usize, f64, f64),
    Plane(u8),
    Commit(usize, f64),
    Batch(Vec<Op>),
}

fn coord() -> impl Strategy<Value = f64> {
    -50.0..50.0f64
}

fn leaf_op() -> impl Strategy<Value = Op> {
    prop_oneof![
        (coord(), coord()).prop_map(|(x, y)| Op::Point(x, y)),
        (coord(), coord(), coord(), coord()).prop_map(|(a, b, c, d)| Op::Line(a, b, c, d)),
        (coord(), coord(), 0.5..30.0f64).prop_map(|(x, y, r)| Op::Circle(x, y, r)),
        any::<usize>().prop_map(Op::Remove),
        (any::<usize>(), 1.0..80.0f64).prop_map(|(i, v)| Op::LengthDim(i, v)),
        (any::<usize>(), 1.0..30.0f64).prop_map(|(i, v)| Op::RadiusDim(i, v)),
        (any::<usize>(), 1.0..80.0f64).prop_map(|(i, v)| Op::SetDim(i, v)),
        (any::<usize>(), any::<bool>()).prop_map(|(i, b)| Op::Drive(i, b)),
        (any::<usize>(), any::<bool>()).prop_map(|(i, b)| Op::Construction(i, b)),
        any::<usize>().prop_map(Op::RemoveConstraint),
        (any::<usize>(), coord(), coord()).prop_map(|(i, x, y)| Op::Label(i, x, y)),
        (0u8..3).prop_map(Op::Plane),
        (any::<usize>(), coord()).prop_map(|(i, v)| Op::Commit(i, v)),
    ]
}

fn op() -> impl Strategy<Value = Op> {
    prop_oneof![
        8 => leaf_op(),
        1 => proptest::collection::vec(leaf_op(), 1..4).prop_map(Op::Batch),
    ]
}

fn pick<T: Copy>(items: &[T], i: usize) -> Option<T> {
    (!items.is_empty()).then(|| items[i % items.len()])
}

/// Turns `op` into a concrete edit of the current state; `None` if there is nothing to act on.
fn to_edit(f: &SketchFeature, op: &Op) -> Option<SketchEdit> {
    let lines: Vec<EntityId> = f
        .sketch
        .entities()
        .filter(|(_, e)| matches!(e.kind, EntityKind::Line { .. }))
        .map(|(id, _)| id)
        .collect();
    let circles: Vec<EntityId> = f
        .sketch
        .entities()
        .filter(|(_, e)| matches!(e.kind, EntityKind::Circle { .. }))
        .map(|(id, _)| id)
        .collect();
    let entities: Vec<EntityId> = f.sketch.entities().map(|(id, _)| id).collect();
    let dims: Vec<ConstraintId> = f
        .sketch
        .constraints()
        .filter(|(_, c)| matches!(c.kind, ConstraintKind::Dimension(_)))
        .map(|(id, _)| id)
        .collect();
    let constraints: Vec<ConstraintId> = f.sketch.constraints().map(|(id, _)| id).collect();
    Some(match op {
        Op::Point(x, y) => SketchEdit::AddPoint {
            at: Point2::new(*x, *y),
        },
        Op::Line(a, b, c, d) => SketchEdit::AddLine {
            a: Point2::new(*a, *b),
            b: Point2::new(*c, *d),
        },
        Op::Circle(x, y, r) => SketchEdit::AddCircle {
            center: Point2::new(*x, *y),
            radius: *r,
        },
        Op::Remove(i) => SketchEdit::RemoveEntities(vec![pick(&entities, *i)?]),
        Op::LengthDim(i, v) => SketchEdit::AddConstraint {
            kind: len_dim(pick(&lines, *i)?, *v),
            expr: None,
        },
        Op::RadiusDim(i, v) => SketchEdit::AddConstraint {
            kind: ConstraintKind::Dimension(Dimension {
                kind: DimensionKind::Radius {
                    curve: pick(&circles, *i)?,
                },
                value: *v,
                driving: true,
                label_pos: Point2::new(0.0, 0.0),
            }),
            expr: Some(inline(&format!("{v} mm"))),
        },
        Op::SetDim(i, v) => SketchEdit::SetDimension {
            id: pick(&dims, *i)?,
            expr: inline(&format!("{v} mm")),
        },
        Op::Drive(i, b) => SketchEdit::SetDriving {
            id: pick(&dims, *i)?,
            driving: *b,
        },
        Op::Construction(i, b) => SketchEdit::SetConstruction {
            id: pick(&entities, *i)?,
            on: *b,
        },
        Op::RemoveConstraint(i) => SketchEdit::RemoveConstraint(pick(&constraints, *i)?),
        Op::Label(i, x, y) => SketchEdit::SetDimensionLabel {
            id: pick(&dims, *i)?,
            pos: Point2::new(*x, *y),
        },
        Op::Plane(p) => SketchEdit::SetPlane(match p {
            0 => PlaneInput::OriginXY,
            1 => PlaneInput::OriginYZ,
            _ => PlaneInput::OriginXZ,
        }),
        Op::Commit(i, v) => {
            let n = f.sketch.params().len();
            if n == 0 {
                return None;
            }
            SketchEdit::CommitDrag {
                values: vec![(amg_sketch::ParamId(u32::try_from(i % n).ok()?), *v)],
            }
        }
        Op::Batch(ops) => {
            // Each step sees the state before the batch, like a UI building it up front.
            SketchEdit::Batch(ops.iter().filter_map(|o| to_edit(f, o)).collect())
        }
    })
}

/// `dims` holds exactly the driving dimensions, each backed by a model parameter of `id`.
fn assert_dims_consistent(doc: &Document, id: FeatureId) {
    let f = current(doc, id);
    let driving: Vec<ConstraintId> = f
        .sketch
        .constraints()
        .filter(|(_, c)| matches!(&c.kind, ConstraintKind::Dimension(d) if d.driving))
        .map(|(id, _)| id)
        .collect();
    let keys: Vec<ConstraintId> = f.dims.keys().copied().collect();
    assert_eq!(keys, driving);
    for e in f.dims.values() {
        let Expr::Param(p) = e else {
            panic!("dimension not promoted: {e:?}")
        };
        let param = doc.design().params().get(*p).unwrap();
        assert_eq!(param.kind, ParamKind::Model { owner: id });
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(24))]

    #[test]
    fn random_edit_sequences_undo_to_the_initial_state(ops in proptest::collection::vec(op(), 1..25)) {
        let mut doc = Document::new("sk19-prop");
        let initial = design_json(&doc);
        let id = add_sketch(&mut doc, SketchFeature::new(PlaneInput::OriginXY)).unwrap();
        for op in &ops {
            let f = current(&doc, id);
            let Some(edit) = to_edit(&f, op) else { continue };
            let mut edited = f.clone();
            if edited.apply(edit).is_err() {
                continue;
            }
            doc.apply(Command::EditFeature { id, feature: FeatureBox::new(edited) }).unwrap();
            assert_dims_consistent(&doc, id);
        }
        let last = design_json(&doc);

        // Whatever the edits made, the sketch recomputes without failing.
        doc.recompute_blocking(jobs()).unwrap();
        prop_assert!(!matches!(health(&doc, id), Health::Error(_)), "{:?}", health(&doc, id));

        // A no-op edit leaves no undo entry, so count the entries instead of the edits.
        let mut undone = 0;
        while doc.undo() {
            undone += 1;
        }
        prop_assert!(undone >= 1);
        prop_assert_eq!(design_json(&doc), initial);
        for _ in 0..undone {
            prop_assert!(doc.redo());
        }
        prop_assert!(!doc.redo());
        prop_assert_eq!(design_json(&doc), last);
    }
}

// Done-when 6.
#[test]
fn save_and_load_round_trip_through_the_amg_format() {
    let mut doc = Document::new("sk19-io");
    add_param(&mut doc, "w", "40 mm");
    let id = add_sketch(&mut doc, rect_feature("w", "20 mm")).unwrap();
    // A circle, a construction line and a driven dimension exercise the rest of the data.
    let mut f = current(&doc, id);
    f.apply(SketchEdit::Batch(vec![
        SketchEdit::AddCircle {
            center: Point2::new(100.0, 0.0),
            radius: 5.0,
        },
        SketchEdit::AddLine {
            a: Point2::new(0.0, -10.0),
            b: Point2::new(40.0, -10.0),
        },
    ]))
    .unwrap();
    let dim = *f.dims.keys().next().unwrap();
    f.apply(SketchEdit::SetDimensionLabel {
        id: dim,
        pos: Point2::new(12.0, -3.0),
    })
    .unwrap();
    edit_feature(&mut doc, id, f);
    doc.recompute_blocking(jobs()).unwrap();
    let before = geom(&doc, id);

    let dir = std::env::temp_dir().join(format!("amg-sk19-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("sketch.amg");
    let opts = SaveOptions {
        cache: Some(doc.cache_export()),
        ..SaveOptions::default()
    };
    amg_doc::save(&path, &doc.snapshot(), &opts).unwrap();
    let loaded = amg_doc::load(&path).unwrap();
    assert!(loaded.warnings.is_empty(), "{:?}", loaded.warnings);
    assert_eq!(
        serde_json::to_value(&loaded.design).unwrap(),
        design_json(&doc)
    );

    let (_, node) = loaded.design.timeline().node(id).unwrap();
    assert_eq!(node.feature, FeatureBox::new(current(&doc, id)));
    let restored = node.feature.downcast_ref::<SketchFeature>().unwrap();
    assert_eq!(restored.dims.len(), 2);
    assert_eq!(
        restored.sketch.constraints().count(),
        current(&doc, id).sketch.constraints().count()
    );

    let mut reopened = Document::from_design(loaded.design);
    reopened.recompute_blocking(jobs()).unwrap();
    assert_eq!(health(&reopened, id), Health::Ok);
    let after = geom(&reopened, id);
    let total =
        |g: &SketchGeom| -> f64 { g.profiles.profiles.iter().map(|p| p.fingerprint.area).sum() };
    assert!((total(&after) - total(&before)).abs() < 1e-9);
    assert_eq!(before.profiles.profiles.len(), 2, "rectangle and circle");
    assert_eq!(after.profiles.profiles.len(), 2);

    // Saving the loaded design again gives the same document.
    let again = dir.join("again.amg");
    amg_doc::save(&again, &reopened.snapshot(), &SaveOptions::default()).unwrap();
    let l2 = amg_doc::load(&again).unwrap();
    assert_eq!(serde_json::to_value(&l2.design).unwrap(), design_json(&doc));
    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn the_kind_is_registered() {
    amg_features::sketch::link();
    let reg = amg_doc::try_registry().unwrap();
    assert_eq!(reg.get("sketch.sketch").unwrap().version, 1);
}
