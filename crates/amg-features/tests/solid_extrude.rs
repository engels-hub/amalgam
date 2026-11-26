//! SOL-01: the solid-feature framework and the minimal `solid.extrude`, one test per Done-when
//! item (see the test names). Geometry needs the `occt` feature (AMG_OCCT_DIR); the dialog,
//! registry and engine tests run everywhere, and without OCCT an extrude fails with the
//! backend-unavailable error.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "tests"
)]

mod solid_common;

use amg_doc::{
    Dependency, Diagnostic, Document, EntityRef, Feature, FeatureBox, FeatureError, FeatureId,
    FeatureOutput, Health, RecomputeCtx, register_feature,
};
use amg_features::dialog::{self, DialogInit, EntityFilter, FieldKey, FieldKind, FieldValue};
use amg_features::solid::{BodyOp, Extrude, OpTarget, SolidError, tool_of};
use serde::{Deserialize, Serialize};
use solid_common::*;

fn a_profile_ref() -> EntityRef {
    EntityRef::Profile {
        sketch: amg_doc::SketchId(FeatureId(7)),
        profile: amg_doc::stub::ProfileKey(42),
        fingerprint: None,
    }
}

// ---- Done-when 7: dialog registration -------------------------------------------------------

#[test]
fn dw7_extrude_dialog_build_of_defaults_round_trips_through_read() {
    let doc = Document::new("dw7");
    let reg = dialog::registration("solid.extrude").expect("solid.extrude registers a dialog");
    assert!(reg.new_component.is_none(), "None until SOL-24");
    let schema = (reg.schema)();
    assert_eq!(schema.kind, "solid.extrude");
    let profiles = schema.field(FieldKey("profiles")).unwrap();
    assert!(matches!(
        profiles.kind,
        FieldKind::Selection { filter, min: 1, .. } if filter.contains(EntityFilter::PROFILE)
    ));
    assert!(
        schema
            .field(FieldKey("distance"))
            .unwrap()
            .manipulator
            .is_some()
    );

    let pre = [a_profile_ref(), EntityRef::Body(body0(FeatureId(3)))];
    let init = DialogInit {
        preselection: &pre,
        design: doc.design(),
        evaluated: doc.evaluated(),
    };
    let v = (reg.defaults)(&init);
    assert_eq!(
        v.get("profiles"),
        Some(&FieldValue::Selection(vec![a_profile_ref()])),
        "only profiles of the preselection"
    );
    let f = (reg.build)(&v, None).expect("defaults build a feature");
    assert_eq!(f.kind(), "solid.extrude");
    f.validate().unwrap();
    let back = (reg.read)(f.get(), doc.design()).unwrap();
    assert_eq!(back, v, "read(build(defaults)) == defaults");

    // Edited values round-trip too, and editing keeps fields the dialog doesn't show.
    let edited = v
        .clone()
        .with("distance", FieldValue::expr("30 mm"))
        .with("operation", FieldValue::Choice("cut"))
        .with("flip", FieldValue::Flip(true))
        .with(
            "bodies",
            FieldValue::Selection(vec![EntityRef::Body(body0(FeatureId(3)))]),
        );
    let g = (reg.build)(&edited, Some(f.get())).unwrap();
    assert_eq!((reg.read)(g.get(), doc.design()).unwrap(), edited);
    let e = g.downcast_ref::<Extrude>().unwrap();
    assert_eq!(e.target.op, BodyOp::Cut);
    assert_eq!(e.target.participants, vec![body0(FeatureId(3))]);

    // An empty profile selection is a field error, not a feature.
    let empty = v.with("profiles", FieldValue::Selection(Vec::new()));
    let errs = (reg.build)(&empty, None).unwrap_err();
    assert_eq!(errs.0[0].0, FieldKey("profiles"));
}

#[test]
fn every_dialog_registration_round_trips() {
    let doc = Document::new("all");
    let pre = [a_profile_ref()];
    let init = DialogInit {
        preselection: &pre,
        design: doc.design(),
        evaluated: doc.evaluated(),
    };
    let mut n = 0;
    for reg in dialog::registrations() {
        n += 1;
        let v = (reg.defaults)(&init);
        let f = (reg.build)(&v, None).unwrap_or_else(|e| panic!("{}: {e:?}", reg.kind));
        f.validate().unwrap();
        assert_eq!(
            (reg.read)(f.get(), doc.design()).unwrap(),
            v,
            "{}",
            reg.kind
        );
        assert_eq!((reg.schema)().kind, reg.kind);
    }
    assert!(n >= 1);
    let mut kinds: Vec<&str> = dialog::registrations().map(|r| r.kind).collect();
    kinds.sort_unstable();
    let len = kinds.len();
    kinds.dedup();
    assert_eq!(kinds.len(), len, "one registration per kind");
}

#[test]
fn feature_json_round_trips_and_validate_checks_sides() {
    let mut e = Extrude::distance(vec![a_profile_ref()], "30 mm", OpTarget::default());
    let fb = FeatureBox::new(e.clone());
    let json = serde_json::to_string(&fb).unwrap();
    let back: FeatureBox = serde_json::from_str(&json).unwrap();
    assert_eq!(back, fb);
    assert_eq!(e.exprs().len(), 2, "distance and taper");
    e.direction = amg_features::solid::Direction::TwoSides;
    assert!(e.validate().is_err(), "two sides need side2");
    e.direction = amg_features::solid::Direction::OneSide;
    e.profiles.clear();
    assert!(e.validate().is_err(), "no profiles");
}

// ---- tools and errors -----------------------------------------------------------------------

#[test]
fn tool_of_finds_registered_kinds_and_refuses_others() {
    let doc = Document::new("tools");
    let ctx = RecomputeCtx::new(
        doc.design(),
        doc.evaluated(),
        FeatureId(99),
        doc.design().root(),
        amg_doc::ComputeMode::Full,
    );
    let sketch =
        amg_features::sketch::SketchFeature::new(amg_features::sketch::PlaneInput::OriginXY);
    let err = tool_of(&sketch, &ctx).unwrap_err();
    assert_eq!(
        err,
        FeatureError::from(SolidError::NotPatternable("sketch.sketch".into()))
    );
    // The extrude's tool is found (and fails here only because its profile doesn't exist).
    let e = Extrude::distance(vec![a_profile_ref()], "10 mm", OpTarget::default());
    let err = tool_of(&e, &ctx).unwrap_err();
    assert!(
        !matches!(
            err,
            FeatureError::InvalidValue {
                field: "objects",
                ..
            }
        ),
        "{err:?}"
    );
}

#[test]
fn solid_errors_map_to_feature_errors() {
    assert!(matches!(
        FeatureError::from(SolidError::WrongBodyKind),
        FeatureError::InvalidValue {
            field: "participants",
            ..
        }
    ));
    assert!(matches!(
        FeatureError::from(SolidError::NothingToCut),
        FeatureError::Kernel(m) if m.contains("nothing to cut")
    ));
    use amg_core::Diagnostic as _;
    assert!(
        SolidError::TargetMissed
            .hint()
            .unwrap()
            .contains("Extend faces")
    );
}

/// Fails after marking an entity for highlight (the engine side of Done-when 5).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct HighlightFail;

impl Feature for HighlightFail {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }
    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        ctx.highlight([EntityRef::Body(body0(FeatureId(5)))]);
        ctx.warn(Diagnostic {
            message: "lost on failure".into(),
            cause: None,
            entities: Vec::new(),
        });
        Err(FeatureError::Kernel("boom".into()))
    }
}

register_feature!(HighlightFail, "test.sol01_highlight_fail", version = 1);

#[test]
fn the_engine_puts_highlights_into_the_error_diagnostic() {
    let mut doc = Document::new("hl");
    let f = add(&mut doc, FeatureBox::new(HighlightFail));
    let Health::Error(d) = health(&doc, f) else {
        panic!("{:?}", health(&doc, f));
    };
    assert_eq!(d.entities, vec![EntityRef::Body(body0(FeatureId(5)))]);
    assert!(d.message.contains("boom"));
}

#[cfg(not(feature = "occt"))]
#[test]
fn without_occt_extrude_fails_with_backend_unavailable() {
    let mut doc = Document::new("no-occt");
    let (_, e) = extrude_rects(
        &mut doc,
        &[[0.0, 0.0, 10.0, 20.0]],
        30.0,
        BodyOp::NewBody,
        vec![],
    );
    let Health::Error(d) = health(&doc, e) else {
        panic!("{:?}", health(&doc, e));
    };
    assert!(
        d.message.contains("not available in this build"),
        "{}",
        d.message
    );
}

#[test]
fn unsupported_options_fail_with_not_yet() {
    let mut doc = Document::new("not-yet");
    let s = add(
        &mut doc,
        FeatureBox::new(rect_sketch(&[[0.0, 0.0, 10.0, 20.0]])),
    );
    let mut e = Extrude::distance(profiles(&doc, s), "30 mm", OpTarget::default());
    // Symmetric extrudes landed with SOL-02; thin extrudes are SOL-18.
    e.thin = Some(amg_features::solid::ThinSpec {
        thickness: amg_doc::Expr::Inline(amg_doc::ExprSrc("1 mm".into())),
        side: amg_features::solid::ThinSide::Side1,
    });
    let id = add(&mut doc, FeatureBox::new(e));
    let Health::Error(d) = health(&doc, id) else {
        panic!("{:?}", health(&doc, id));
    };
    assert!(
        d.message.contains("not available in this build yet"),
        "{}",
        d.message
    );
}

#[cfg(feature = "occt")]
mod occt {
    use amg_brep::{KernelConfig, OcctKernel, SubKind, SubShape};
    use amg_core::Point2;
    use amg_doc::naming::{KernelGeom, NamingView, default_kernel, topo_ref_at};
    use amg_doc::testing::brep::{TestBrepFillet, TestProbe};
    use amg_doc::{
        BodyId, ComputeMode, Evaluated, GeomFingerprint, MatchQuality, SetPolicy, TopoKind, TopoRef,
    };
    use amg_features::sketch::SketchEdit;

    use super::*;

    // ---- Done-when 1: one-sided distance extrude, each body op, goldens ----------------------

    const RECT: [f64; 4] = [0.0, 0.0, 10.0, 20.0];
    /// The 10 × 20 rectangle inside the 40 × 40 × 10 base plate.
    const INNER: [f64; 4] = [5.0, 5.0, 15.0, 25.0];
    const PLATE: [f64; 4] = [0.0, 0.0, 40.0, 40.0];

    #[test]
    fn dw1_new_body_rectangle_10x20_h30_has_volume_6000() {
        let mut doc = Document::new("dw1-new");
        let (_, e) = extrude_rects(&mut doc, &[RECT], 30.0, BodyOp::NewBody, vec![]);
        assert_eq!(health(&doc, e), Health::Ok);
        let s = brep(&doc, body0(e));
        assert_rel(volume(&s), 6000.0, 1e-9);
        assert_eq!(faces(&s), 6);
        check_golden("new_body", &s);
        // Caps and laterals carry the naming-note names.
        let n = doc.evaluated().naming(body0(e)).unwrap();
        let roles: Vec<String> = n
            .list(TopoKind::Face)
            .unwrap()
            .iter()
            .map(|t| format!("{:?}", t.segments()[0]))
            .collect();
        assert_eq!(roles.iter().filter(|r| r.contains("Lateral")).count(), 4);
        assert_eq!(roles.iter().filter(|r| r.contains("StartCap")).count(), 1);
        assert_eq!(roles.iter().filter(|r| r.contains("EndCap")).count(), 1);
        assert!(
            roles
                .iter()
                .all(|r| r.contains("SketchCurve") || r.contains("Profile"))
        );
    }

    fn plate(doc: &mut Document) -> BodyId {
        let (_, p) = extrude_rects(doc, &[PLATE], 10.0, BodyOp::NewBody, vec![]);
        body0(p)
    }

    #[test]
    fn dw1_join_onto_a_box() {
        let mut doc = Document::new("dw1-join");
        let b = plate(&mut doc);
        let (_, e) = extrude_rects(&mut doc, &[INNER], 30.0, BodyOp::Join, vec![b]);
        assert_eq!(health(&doc, e), Health::Ok);
        assert!(
            doc.evaluated().body(body0(e)).is_none(),
            "joined, no new body"
        );
        let s = brep(&doc, b);
        assert_rel(volume(&s), 16000.0 + 6000.0 - 2000.0, 1e-9);
        // Plate 6 (top with a hole loop, bottom merged with the tool's start cap) + 4 laterals
        // above the plate + the end cap.
        assert_eq!(faces(&s), 11);
        check_golden("join_box", &s);
    }

    #[test]
    fn dw1_cut_through_a_box() {
        let mut doc = Document::new("dw1-cut");
        let b = plate(&mut doc);
        let (_, e) = extrude_rects(&mut doc, &[INNER], 30.0, BodyOp::Cut, vec![b]);
        assert_eq!(health(&doc, e), Health::Ok);
        let s = brep(&doc, b);
        assert_rel(volume(&s), 16000.0 - 2000.0, 1e-9);
        assert_eq!(faces(&s), 10, "6 plate faces + 4 hole walls");
        check_golden("cut_box", &s);
    }

    #[test]
    fn dw1_intersect_with_a_box() {
        let mut doc = Document::new("dw1-int");
        let b = plate(&mut doc);
        let (_, e) = extrude_rects(&mut doc, &[INNER], 30.0, BodyOp::Intersect, vec![b]);
        assert_eq!(health(&doc, e), Health::Ok);
        let s = brep(&doc, b);
        assert_rel(volume(&s), 2000.0, 1e-9);
        assert_eq!(faces(&s), 6);
        check_golden("intersect_box", &s);
    }

    #[test]
    fn flip_and_negative_distance_go_the_other_way() {
        let mut doc = Document::new("flip");
        let s = add(&mut doc, FeatureBox::new(rect_sketch(&[RECT])));
        let mut e = Extrude::distance(profiles(&doc, s), "30 mm", OpTarget::default());
        e.flip = true;
        let a = add(&mut doc, FeatureBox::new(e));
        let neg = Extrude::distance(profiles(&doc, s), "-30 mm", OpTarget::default());
        let b = add(&mut doc, FeatureBox::new(neg));
        for id in [a, b] {
            let sh = brep(&doc, body0(id));
            assert_rel(volume(&sh), 6000.0, 1e-9);
            assert!((sh.bbox().min.z + 30.0).abs() < 1e-6 && sh.bbox().max.z.abs() < 1e-6);
        }
    }

    // ---- Done-when 2: BodyOp rules -----------------------------------------------------------

    #[test]
    fn dw2_join_touching_two_keeps_the_first_and_consumes_the_second() {
        let mut doc = Document::new("dw2-join2");
        let (_, a) = extrude_rects(
            &mut doc,
            &[[0.0, 0.0, 10.0, 10.0]],
            10.0,
            BodyOp::NewBody,
            vec![],
        );
        let (_, b) = extrude_rects(
            &mut doc,
            &[[20.0, 0.0, 30.0, 10.0]],
            10.0,
            BodyOp::NewBody,
            vec![],
        );
        let (a, b) = (body0(a), body0(b));
        let (_, e) = extrude_rects(
            &mut doc,
            &[[5.0, 2.0, 25.0, 8.0]],
            5.0,
            BodyOp::Join,
            vec![a, b],
        );
        assert_eq!(health(&doc, e), Health::Ok);
        let ev = doc.evaluated();
        assert!(ev.body(b).is_none(), "the second participant is consumed");
        assert_eq!(ev.successors(b), &[a], "Consumed {{ into: first }}");
        assert_rel(volume(&brep(&doc, a)), 2000.0 + 600.0 - 300.0, 1e-9);
        assert_eq!(ev.bodies().count(), 1);
    }

    #[test]
    fn dw2_cut_splitting_a_bar_keeps_piece_0_and_creates_slot_1() {
        let mut doc = Document::new("dw2-split");
        let (_, bar) = extrude_rects(
            &mut doc,
            &[[0.0, 0.0, 100.0, 10.0]],
            10.0,
            BodyOp::NewBody,
            vec![],
        );
        let bar = body0(bar);
        let (_, e) = extrude_rects(
            &mut doc,
            &[[60.0, -5.0, 70.0, 15.0]],
            20.0,
            BodyOp::Cut,
            vec![bar],
        );
        assert_eq!(health(&doc, e), Health::Ok);
        let piece1 = BodyId { origin: e, slot: 1 };
        let p0 = brep(&doc, bar);
        let p1 = brep(&doc, piece1);
        assert_rel(volume(&p0), 6000.0, 1e-9);
        assert_rel(volume(&p1), 3000.0, 1e-9);
        assert!(
            p0.bbox().max.x < 60.0 + 1e-6,
            "piece 0 is the larger, left piece"
        );
        let ev: &Evaluated = doc.evaluated();
        assert_eq!(
            NamingView::derived_from(ev, piece1),
            Some(bar),
            "derived_from"
        );
        check_golden("cut_split_piece0", &p0);
        check_golden("cut_split_piece1", &p1);
    }

    #[test]
    fn dw2_cut_touching_nothing_fails_nothing_to_cut() {
        let mut doc = Document::new("dw2-none");
        let b = plate(&mut doc);
        let (_, e) = extrude_rects(
            &mut doc,
            &[[100.0, 100.0, 110.0, 110.0]],
            5.0,
            BodyOp::Cut,
            vec![b],
        );
        let Health::Error(d) = health(&doc, e) else {
            panic!("{:?}", health(&doc, e));
        };
        assert!(d.message.contains("nothing to cut"), "{}", d.message);
    }

    #[test]
    fn dw2_mesh_participant_fails_wrong_body_kind() {
        let mut doc = Document::new("dw2-mesh");
        let m = add(
            &mut doc,
            FeatureBox::new(amg_doc::testing::TestAddMesh {
                bodies: 1,
                label: "mesh".into(),
            }),
        );
        let (_, e) = extrude_rects(&mut doc, &[RECT], 5.0, BodyOp::Cut, vec![body0(m)]);
        let Health::Error(d) = health(&doc, e) else {
            panic!("{:?}", health(&doc, e));
        };
        assert!(d.message.contains("body is not a solid"), "{}", d.message);
    }

    #[test]
    fn dw2_join_touching_nothing_creates_a_body_and_warns() {
        let mut doc = Document::new("dw2-join0");
        let b = plate(&mut doc);
        let (_, e) = extrude_rects(
            &mut doc,
            &[[100.0, 100.0, 110.0, 120.0]],
            30.0,
            BodyOp::Join,
            vec![b],
        );
        let Health::Warning(w) = health(&doc, e) else {
            panic!("{:?}", health(&doc, e));
        };
        assert!(
            w.iter().any(|d| d.message.contains("Nothing to join")),
            "{w:?}"
        );
        assert_rel(volume(&brep(&doc, body0(e))), 6000.0, 1e-9);
        assert_rel(volume(&brep(&doc, b)), 16000.0, 1e-9);
    }

    // ---- Done-when 3: regions sharing curves fuse into one body -----------------------------

    #[test]
    fn dw3_rectangle_split_by_a_line_extrudes_to_one_body_with_6_faces() {
        let mut doc = Document::new("dw3");
        let mut sk = rect_sketch(&[RECT]);
        sk.apply(SketchEdit::AddLine {
            a: Point2::new(5.0, 0.0),
            b: Point2::new(5.0, 20.0),
        })
        .unwrap();
        let s = add(&mut doc, FeatureBox::new(sk));
        let refs = profiles(&doc, s);
        assert_eq!(
            refs.len(),
            2,
            "the line splits the rectangle into two regions"
        );
        let e = add(
            &mut doc,
            FeatureBox::new(Extrude::distance(refs, "30 mm", OpTarget::default())),
        );
        assert_eq!(health(&doc, e), Health::Ok);
        assert_eq!(doc.evaluated().bodies().count(), 1);
        let sh = brep(&doc, body0(e));
        assert_eq!(faces(&sh), 6);
        assert_rel(volume(&sh), 6000.0, 1e-9);
        check_golden("two_regions", &sh);
    }

    #[test]
    fn disjoint_regions_give_one_body_each_with_their_own_names() {
        let mut doc = Document::new("disjoint");
        let (_, e) = extrude_rects(
            &mut doc,
            &[[0.0, 0.0, 10.0, 10.0], [20.0, 0.0, 25.0, 10.0]],
            10.0,
            BodyOp::NewBody,
            vec![],
        );
        assert_eq!(health(&doc, e), Health::Ok);
        let ev = doc.evaluated();
        assert_eq!(ev.bodies().count(), 2);
        let v: Vec<f64> = (0..2)
            .map(|k| volume(&brep(&doc, BodyId { origin: e, slot: k })))
            .collect();
        assert_rel(v[0] + v[1], 1500.0, 1e-9);
        // Every face of each body is named, and the names of the two bodies differ.
        let n0 = ev.naming(BodyId { origin: e, slot: 0 }).unwrap();
        let n1 = ev.naming(BodyId { origin: e, slot: 1 }).unwrap();
        assert_eq!(n0.list(TopoKind::Face).unwrap().len(), 6);
        assert_eq!(n1.list(TopoKind::Face).unwrap().len(), 6);
        for t in n0.list(TopoKind::Face).unwrap() {
            assert!(!n1.list(TopoKind::Face).unwrap().contains(t));
        }
    }

    #[test]
    fn preview_mode_gives_the_full_result() {
        let mut doc = Document::new("preview");
        let b = plate(&mut doc);
        let s = add(&mut doc, FeatureBox::new(rect_sketch(&[INNER])));
        let e = Extrude::distance(
            profiles(&doc, s),
            "30 mm",
            OpTarget {
                op: BodyOp::Cut,
                participants: vec![b],
            },
        );
        let mut vols = Vec::new();
        for mode in [ComputeMode::Preview, ComputeMode::Full] {
            let mut ctx = RecomputeCtx::new(
                doc.design(),
                doc.evaluated(),
                FeatureId(9_999),
                doc.design().root(),
                mode,
            );
            let out = e.compute(&mut ctx).unwrap();
            let amg_doc::BodyChangeKind::Modified { geom, naming } = &out.bodies[0].change else {
                panic!("{:?}", out.bodies);
            };
            assert!(naming.is_some(), "preview still names");
            let amg_doc::BodyGeom::Solid(s) = geom else {
                panic!();
            };
            vols.push(volume(s.brep().unwrap()));
        }
        assert_rel(vols[0], 14000.0, 1e-9);
        assert_eq!(vols[0], vols[1]);
    }

    // ---- Done-when 4: TNP-01, TNP-02, TNP-03 on solid.extrude -------------------------------

    /// `$pick`: the nearest entity of `kind` with a line/plane class to `near` (note §11.2).
    fn pick(ev: &Evaluated, body: BodyId, kind: TopoKind, near: [f64; 3]) -> TopoRef {
        let geom = KernelGeom::new(ev, default_kernel());
        let near = amg_core::Point3::from_array(near);
        let mut found: Vec<(f64, u32)> = amg_doc::naming::GeomSource::all(&geom, body, kind)
            .into_iter()
            .map(|(i, fp)| ((fp.centroid - near).length(), i))
            .collect();
        found.sort_by(|a, b| a.0.total_cmp(&b.0));
        assert!(
            found.len() > 1 && found[1].0 - found[0].0 > 1e-6,
            "tied pick"
        );
        let mut r = topo_ref_at(ev, default_kernel(), body, kind, found[0].1).unwrap();
        r.key = 1;
        r
    }

    /// The fillet's (only) resolution after the edit: quality and the entity's fingerprint.
    fn fillet_resolution(doc: &Document, fil: FeatureId) -> (MatchQuality, GeomFingerprint) {
        let node = doc.evaluated().node(fil).unwrap();
        let rec = &node.resolutions[0];
        let q = rec.result.as_ref().expect("resolved")[0].quality;
        let Some(EntityRef::Topo(t)) = rec.current.first() else {
            panic!("{rec:?}");
        };
        (q, t.fingerprint.clone().unwrap())
    }

    #[track_caller]
    fn expect_edge(fp: &GeomFingerprint, centroid: [f64; 3], measure: f64) {
        let c = amg_core::Point3::from_array(centroid);
        assert!((fp.centroid - c).length() < 1e-6, "{fp:?} vs {centroid:?}");
        assert!((fp.measure - measure).abs() < 1e-6, "{fp:?} vs {measure}");
    }

    fn fillet(doc: &mut Document, body: BodyId, near: [f64; 3]) -> FeatureId {
        let r = pick(doc.evaluated(), body, TopoKind::Edge, near);
        add(
            doc,
            FeatureBox::new(TestBrepFillet {
                body,
                edges: vec![r],
                radius: 5.0,
                policy: SetPolicy::DropMissing,
            }),
        )
    }

    #[test]
    fn dw4_tnp01_distance_edit_keeps_the_top_edge() {
        // B1 as an extrude: 100 × 60 rectangle, 40 high; edit to 80.
        let mut doc = Document::new("tnp01");
        let (s, e) = extrude_rects(
            &mut doc,
            &[[0.0, 0.0, 100.0, 60.0]],
            40.0,
            BodyOp::NewBody,
            vec![],
        );
        let fil = fillet(&mut doc, body0(e), [50.0, 0.0, 40.0]);
        assert_eq!(health(&doc, fil), Health::Ok);
        let edited = Extrude::distance(profiles(&doc, s), "80 mm", OpTarget::default());
        edit(&mut doc, e, FeatureBox::new(edited));
        assert_eq!(health(&doc, fil), Health::Ok);
        let (q, fp) = fillet_resolution(&doc, fil);
        assert_eq!(q, MatchQuality::Exact);
        expect_edge(&fp, [50.0, 0.0, 80.0], 100.0);
    }

    const B2: [[f64; 2]; 4] = [[0.0, 0.0], [100.0, 0.0], [100.0, 60.0], [0.0, 60.0]];

    /// B2 as a sketch polygon + extrude 40; returns (doc, sketch, extrude, sketch feature,
    /// points, lines).
    fn b2() -> (
        Document,
        FeatureId,
        FeatureId,
        amg_features::sketch::SketchFeature,
        Vec<amg_sketch::EntityId>,
        Vec<amg_sketch::EntityId>,
    ) {
        let mut doc = Document::new("b2");
        let (sk, pts, lines) = polygon_sketch(&B2);
        let s = add(&mut doc, FeatureBox::new(sk.clone()));
        let ext = Extrude::distance(profiles(&doc, s), "40 mm", OpTarget::default());
        let e = add(&mut doc, FeatureBox::new(ext));
        (doc, s, e, sk, pts, lines)
    }

    fn reextrude(
        doc: &mut Document,
        s: FeatureId,
        e: FeatureId,
        sk: amg_features::sketch::SketchFeature,
    ) {
        edit(doc, s, FeatureBox::new(sk));
        let ext = Extrude::distance(profiles(doc, s), "40 mm", OpTarget::default());
        edit(doc, e, FeatureBox::new(ext));
    }

    #[test]
    fn dw4_tnp02_inserting_a_point_keeps_the_far_vertical_edge() {
        let (mut doc, s, e, mut sk, pts, lines) = b2();
        let fil = fillet(&mut doc, body0(e), [100.0, 0.0, 20.0]);
        assert_eq!(health(&doc, fil), Health::Ok);
        // Point 5 at (-20, 30) between points 4 and 1: line 4→1 becomes 4→5→1.
        sk.edit_sketch(|s| {
            s.remove_entities(&[lines[3]])?;
            let p5 = s.add_point(Point2::new(-20.0, 30.0))?;
            s.add_line_between(pts[3], p5)?;
            s.add_line_between(p5, pts[0])?;
            Ok(())
        })
        .unwrap();
        reextrude(&mut doc, s, e, sk);
        assert_eq!(health(&doc, fil), Health::Ok);
        let (q, fp) = fillet_resolution(&doc, fil);
        assert_eq!(q, MatchQuality::Exact);
        expect_edge(&fp, [100.0, 0.0, 20.0], 40.0);
    }

    #[test]
    fn a_profile_lost_by_id_is_found_by_fingerprint_with_a_warning() {
        let (mut doc, s, e, mut sk, pts, lines) = b2();
        sk.edit_sketch(|s| {
            s.remove_entities(&[lines[3]])?;
            let p5 = s.add_point(Point2::new(-20.0, 30.0))?;
            s.add_line_between(pts[3], p5)?;
            s.add_line_between(p5, pts[0])?;
            Ok(())
        })
        .unwrap();
        edit(&mut doc, s, FeatureBox::new(sk));
        let Health::Warning(w) = health(&doc, e) else {
            panic!("{:?}", health(&doc, e));
        };
        assert!(
            w.iter().any(|d| d.message.contains("FingerprintOnly")),
            "{w:?}"
        );
        assert_rel(volume(&brep(&doc, body0(e))), 40.0 * (6000.0 + 600.0), 1e-9);
    }

    #[test]
    fn dw4_tnp03_deleting_a_point_keeps_the_far_vertical_edge() {
        let (mut doc, s, e, mut sk, pts, lines) = b2();
        let fil = fillet(&mut doc, body0(e), [0.0, 60.0, 20.0]);
        assert_eq!(health(&doc, fil), Health::Ok);
        // Delete point 2: the outline becomes the triangle 1, 3, 4.
        sk.edit_sketch(|s| {
            s.remove_entities(&[lines[0], lines[1]])?;
            s.add_line_between(pts[0], pts[2])?;
            Ok(())
        })
        .unwrap();
        reextrude(&mut doc, s, e, sk);
        assert_eq!(health(&doc, fil), Health::Ok);
        let (q, fp) = fillet_resolution(&doc, fil);
        assert_eq!(q, MatchQuality::Exact);
        expect_edge(&fp, [0.0, 60.0, 20.0], 40.0);
    }

    #[test]
    fn edited_cut_keeps_names_of_the_target_and_the_tool_walls() {
        // A probe on a hole wall (tool lateral) and on the plate top resolves Exact after the
        // cut depth changes from through to blind.
        let mut doc = Document::new("names");
        let b = plate(&mut doc);
        let s = add(&mut doc, FeatureBox::new(rect_sketch(&[INNER])));
        let cut = |d: &str, doc: &Document| {
            Extrude::distance(
                profiles(doc, s),
                d,
                OpTarget {
                    op: BodyOp::Cut,
                    participants: vec![b],
                },
            )
        };
        let through = cut("30 mm", &doc);
        let e = add(&mut doc, FeatureBox::new(through));
        let wall = pick(doc.evaluated(), b, TopoKind::Face, [5.0, 15.0, 5.0]);
        let top = pick(doc.evaluated(), b, TopoKind::Face, [20.0, 20.0, 10.0]);
        let probe = add(
            &mut doc,
            FeatureBox::new(TestProbe {
                body: b,
                refs: vec![wall, top],
                policy: SetPolicy::AllOrError,
            }),
        );
        // From z = 0 upwards: a blind pocket 5 deep.
        let blind = cut("5 mm", &doc);
        edit(&mut doc, e, FeatureBox::new(blind));
        assert_eq!(health(&doc, probe), Health::Ok, "{:?}", health(&doc, probe));
        let node = doc.evaluated().node(probe).unwrap();
        for rec in node.resolutions.iter() {
            assert_eq!(rec.result.as_ref().unwrap()[0].quality, MatchQuality::Exact);
        }
        assert_rel(volume(&brep(&doc, b)), 16000.0 - 1000.0, 1e-9);
    }

    // ---- Done-when 5: kernel failure highlights the offending edges -------------------------

    #[test]
    fn dw5_injected_kernel_failure_highlights_the_offending_edges() {
        let mut doc = Document::new("dw5");
        let b = plate(&mut doc);
        let s = add(&mut doc, FeatureBox::new(rect_sketch(&[INNER])));
        let e = Extrude::distance(
            profiles(&doc, s),
            "30 mm",
            OpTarget {
                op: BodyOp::Cut,
                participants: vec![b],
            },
        );
        let offending = vec![
            SubShape {
                kind: SubKind::Edge,
                index: 0,
            },
            SubShape {
                kind: SubKind::Edge,
                index: 3,
            },
        ];
        let failing = failing::Failing {
            inner: OcctKernel::new(KernelConfig::default()),
            offending,
        };
        let mut ctx = RecomputeCtx::new(
            doc.design(),
            doc.evaluated(),
            FeatureId(9_999),
            doc.design().root(),
            ComputeMode::Full,
        )
        .with_kernel(&failing);
        let err = e.compute(&mut ctx).unwrap_err();
        assert!(
            matches!(&err, FeatureError::Kernel(m) if m.contains("injected")),
            "{err:?}"
        );
        let hl = ctx.highlights();
        assert_eq!(hl.len(), 2);
        for (h, want) in hl.iter().zip([0u32, 3]) {
            let EntityRef::Topo(t) = h else {
                panic!("{h:?}");
            };
            assert_eq!((t.body, t.kind), (b, TopoKind::Edge));
            let r = doc.evaluated().resolve(t).unwrap();
            assert_eq!(
                r.index, want,
                "the highlight resolves to the offending edge"
            );
            assert_eq!(r.quality, MatchQuality::Exact);
        }
    }
}
