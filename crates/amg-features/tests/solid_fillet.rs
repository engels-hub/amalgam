//! SOL-05: `solid.fillet` and `solid.chamfer` on raw BREP-06, one test (or golden) per
//! Done-when item (see the test names; Done-when 5 is `benches/sol_fillet.rs`). Geometry needs
//! the `occt` feature (AMG_OCCT_DIR); the dialog, data and option tests run everywhere.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "tests"
)]

mod solid_common;

use amg_doc::naming::TopoName;
use amg_doc::{
    ComputeMode, Dim, Document, EntityRef, Expr, ExprSrc, Feature, FeatureBox, FeatureError,
    FeatureId, RecomputeCtx, TopoKind, TopoRef,
};
use amg_features::dialog::{self, DialogInit, EntityFilter, FieldKey, FieldKind, FieldValue};
use amg_features::solid::{
    Chamfer, ChamferKind, ChamferSet, CornerType, Fillet, FilletRadius, FilletSet, SolidError,
    VarRadius,
};
use solid_common::*;

fn expr(s: &str) -> Expr {
    Expr::Inline(ExprSrc(s.to_owned()))
}

/// A syntactically valid edge reference (it resolves nowhere).
fn an_edge_ref(kind: TopoKind, key: u32) -> TopoRef {
    TopoRef {
        body: body0(FeatureId(3)),
        kind,
        name: TopoName::new(Vec::new()),
        fingerprint: None,
        key,
    }
}

fn a_face_ref() -> TopoRef {
    an_edge_ref(TopoKind::Face, 0)
}

// ---- dialogs and data (everywhere) -----------------------------------------------------------

#[test]
fn fillet_dialog_round_trips_every_radius_type() {
    let doc = Document::new("fillet-dialog");
    let reg = dialog::registration("solid.fillet").expect("solid.fillet registers a dialog");
    assert!(reg.new_component.is_none());
    let schema = (reg.schema)();
    assert_eq!(schema.kind, "solid.fillet");
    assert!(matches!(
        schema.field(FieldKey("edges")).unwrap().kind,
        FieldKind::Selection { filter, min: 1, .. } if filter == EntityFilter::EDGE
    ));
    assert!(
        schema
            .field(FieldKey("radius"))
            .unwrap()
            .manipulator
            .is_some()
    );
    let edge = EntityRef::Topo(an_edge_ref(TopoKind::Edge, 1));
    let pre = [edge.clone(), EntityRef::Body(body0(FeatureId(3)))];
    let init = DialogInit {
        preselection: &pre,
        design: doc.design(),
        evaluated: doc.evaluated(),
    };
    let v = (reg.defaults)(&init);
    assert_eq!(
        v.get("edges"),
        Some(&FieldValue::Selection(vec![edge.clone()])),
        "only edges of the preselection"
    );
    let f = (reg.build)(&v, None).unwrap();
    f.validate().unwrap();
    assert_eq!((reg.read)(f.get(), doc.design()).unwrap(), v);
    let fil = f.downcast_ref::<Fillet>().unwrap();
    assert!(fil.tangent_chain);
    assert_eq!(fil.sets[0].radius, FilletRadius::Constant(expr("1 mm")));

    let chord = v
        .clone()
        .with("radius_type", FieldValue::Choice("chord"))
        .with("radius", FieldValue::expr("3 mm"))
        .with("tangent_chain", FieldValue::Bool(false));
    let g = (reg.build)(&chord, Some(f.get())).unwrap();
    assert_eq!((reg.read)(g.get(), doc.design()).unwrap(), chord);
    let gf = g.downcast_ref::<Fillet>().unwrap();
    assert_eq!(gf.sets[0].radius, FilletRadius::Chord(expr("3 mm")));
    assert!(!gf.tangent_chain);

    let var = v
        .clone()
        .with("radius_type", FieldValue::Choice("variable"))
        .with("start_radius", FieldValue::expr("2 mm"))
        .with("end_radius", FieldValue::expr("4 mm"));
    let h = (reg.build)(&var, None).unwrap();
    assert_eq!((reg.read)(h.get(), doc.design()).unwrap(), var);
    let FilletRadius::Variable(pts) = &h.downcast_ref::<Fillet>().unwrap().sets[0].radius else {
        panic!("variable");
    };
    assert_eq!(pts.len(), 2);
    assert_eq!(
        (&pts[0].radius, &pts[1].radius),
        (&expr("2 mm"), &expr("4 mm"))
    );

    // Editing keeps further sets; a face in the edge field is a field error.
    let mut two = fil.clone();
    two.sets.push(FilletSet {
        edges: vec![an_edge_ref(TopoKind::Edge, 2)],
        radius: FilletRadius::Constant(expr("7 mm")),
    });
    let kept = (reg.build)(&chord, Some(&two)).unwrap();
    assert_eq!(kept.downcast_ref::<Fillet>().unwrap().sets[1], two.sets[1]);
    let bad = v.with(
        "edges",
        FieldValue::Selection(vec![EntityRef::Topo(a_face_ref())]),
    );
    assert_eq!(
        (reg.build)(&bad, None).unwrap_err().0[0].0,
        FieldKey("edges")
    );
}

#[test]
fn chamfer_dialog_round_trips_every_kind() {
    let doc = Document::new("chamfer-dialog");
    let reg = dialog::registration("solid.chamfer").expect("solid.chamfer registers a dialog");
    let schema = (reg.schema)();
    assert_eq!(schema.kind, "solid.chamfer");
    assert!(matches!(
        schema.field(FieldKey("face")).unwrap().kind,
        FieldKind::Selection { max: Some(1), .. }
    ));
    let edge = EntityRef::Topo(an_edge_ref(TopoKind::Edge, 1));
    let pre = [edge.clone()];
    let init = DialogInit {
        preselection: &pre,
        design: doc.design(),
        evaluated: doc.evaluated(),
    };
    let v = (reg.defaults)(&init);
    let f = (reg.build)(&v, None).unwrap();
    f.validate().unwrap();
    assert_eq!((reg.read)(f.get(), doc.design()).unwrap(), v);
    assert_eq!(
        f.downcast_ref::<Chamfer>().unwrap().sets[0].kind,
        ChamferKind::Equal(expr("1 mm"))
    );

    let face = FieldValue::Selection(vec![EntityRef::Topo(a_face_ref())]);
    let two = v
        .clone()
        .with("chamfer_type", FieldValue::Choice("two_distances"))
        .with("distance", FieldValue::expr("2 mm"))
        .with("distance2", FieldValue::expr("3 mm"))
        .with("face", face.clone());
    let g = (reg.build)(&two, None).unwrap();
    assert_eq!((reg.read)(g.get(), doc.design()).unwrap(), two);
    assert!(matches!(
        &g.downcast_ref::<Chamfer>().unwrap().sets[0].kind,
        ChamferKind::TwoDistances { d1, d2, .. } if *d1 == expr("2 mm") && *d2 == expr("3 mm")
    ));

    let da = v
        .clone()
        .with("chamfer_type", FieldValue::Choice("distance_angle"))
        .with("angle", FieldValue::expr("30 deg"))
        .with("face", face);
    let h = (reg.build)(&da, None).unwrap();
    assert_eq!((reg.read)(h.get(), doc.design()).unwrap(), da);

    // Two distances without a reference face is a field error on `face`.
    let no_face = two.with("face", FieldValue::Selection(Vec::new()));
    assert_eq!(
        (reg.build)(&no_face, None).unwrap_err().0[0].0,
        FieldKey("face")
    );
}

#[test]
fn feature_json_round_trips_and_exprs_have_their_dimensions() {
    let f = Fillet {
        sets: vec![
            FilletSet {
                edges: vec![an_edge_ref(TopoKind::Edge, 1)],
                radius: FilletRadius::Variable(vec![
                    VarRadius {
                        at: expr("0"),
                        radius: expr("2 mm"),
                    },
                    VarRadius {
                        at: expr("1"),
                        radius: expr("4 mm"),
                    },
                ]),
            },
            FilletSet {
                edges: vec![an_edge_ref(TopoKind::Edge, 2)],
                radius: FilletRadius::Chord(expr("3 mm")),
            },
        ],
        tangent_chain: false,
        corner: CornerType::RollingBall,
    };
    let fb = FeatureBox::new(f.clone());
    let back: FeatureBox = serde_json::from_str(&serde_json::to_string(&fb).unwrap()).unwrap();
    assert_eq!(back.downcast_ref::<Fillet>().unwrap(), &f);
    let mut g = f.clone();
    let dims: Vec<Dim> = g.exprs_mut().into_iter().map(|(_, d)| d).collect();
    assert_eq!(
        dims,
        vec![Dim::NONE, Dim::LENGTH, Dim::NONE, Dim::LENGTH, Dim::LENGTH]
    );
    assert_eq!(f.exprs().len(), 5);
    assert_eq!(f.inputs().len(), 3, "one body + two edge refs");

    // `tangent_chain` and `corner` default; unknown fields are refused.
    let json = r#"{"sets":[{"edges":[],"radius":{"Constant":{"Inline":"1 mm"}}}]}"#;
    let d: Fillet = serde_json::from_str(json).unwrap();
    assert!(d.tangent_chain);
    assert_eq!(d.corner, CornerType::RollingBall);
    assert!(serde_json::from_str::<Fillet>(r#"{"sets":[],"extra":1}"#).is_err());

    let c = Chamfer {
        sets: vec![ChamferSet {
            edges: vec![an_edge_ref(TopoKind::Edge, 1)],
            kind: ChamferKind::DistanceAngle {
                d: expr("2 mm"),
                angle: expr("30 deg"),
                face: a_face_ref(),
            },
        }],
        tangent_chain: true,
    };
    let mut c2 = c.clone();
    let dims: Vec<Dim> = c2.exprs_mut().into_iter().map(|(_, d)| d).collect();
    assert_eq!(dims, vec![Dim::LENGTH, Dim::ANGLE]);
    c.validate().unwrap();
    let mut bad = c;
    if let ChamferKind::DistanceAngle { face, .. } = &mut bad.sets[0].kind {
        face.kind = TopoKind::Edge;
    }
    assert!(matches!(
        bad.validate(),
        Err(FeatureError::InvalidValue { field: "face", .. })
    ));
}

#[test]
fn setback_corners_and_empty_selections_fail_cleanly() {
    let doc = Document::new("opts");
    let mut ctx = RecomputeCtx::new(
        doc.design(),
        doc.evaluated(),
        FeatureId(99),
        doc.design().root(),
        ComputeMode::Full,
    );
    let mut f = Fillet::constant(Vec::new(), "1 mm");
    f.corner = CornerType::Setback;
    let e = f.compute(&mut ctx).unwrap_err();
    assert_eq!(
        e,
        FeatureError::from(SolidError::NotYet {
            what: "setback fillet corners"
        })
    );
    f.corner = CornerType::RollingBall;
    assert!(matches!(
        f.compute(&mut ctx),
        Err(FeatureError::InvalidValue { field: "edges", .. })
    ));
    let bad_radius = Fillet {
        sets: vec![FilletSet {
            edges: Vec::new(),
            radius: FilletRadius::Variable(vec![
                VarRadius {
                    at: expr("0.5"),
                    radius: expr("2 mm"),
                },
                VarRadius {
                    at: expr("0.5"),
                    radius: expr("3 mm"),
                },
            ]),
        }],
        tangent_chain: true,
        corner: CornerType::RollingBall,
    };
    assert!(matches!(
        bad_radius.compute(&mut ctx),
        Err(FeatureError::InvalidValue {
            field: "radius",
            ..
        })
    ));
    let neg = Fillet::constant(Vec::new(), "-1 mm");
    assert!(matches!(
        neg.compute(&mut ctx),
        Err(FeatureError::InvalidValue {
            field: "radius",
            ..
        })
    ));
}

// ---- geometry (OCCT) -------------------------------------------------------------------------

#[cfg(feature = "occt")]
mod occt {
    use std::f64::consts::{FRAC_PI_4, FRAC_PI_6, PI};

    use amg_brep::{BrepKernel, EdgeId, FilletSpec, SubKind};
    use amg_core::{Plane, Point2, Vec3};
    use amg_doc::naming::{GeomClass, KernelGeom, default_kernel, topo_ref_at};
    use amg_doc::testing::brep::{TestBrepBox, TestProbe};
    use amg_doc::{
        BodyId, Command, Evaluated, GeomFingerprint, Health, MatchQuality, ResolveError, SetPolicy,
    };
    use amg_features::sketch::{PlaneInput, SketchFeature};
    use amg_features::solid::fillet::harness::{self, FilletCase};
    use amg_features::solid::{BodyOp, Extrude, OpTarget, tangent_chain_edges};

    use super::*;

    const REL: f64 = 1e-6;

    /// Cross-section area of a fillet of radius `r` between perpendicular faces.
    fn spandrel(r: f64) -> f64 {
        (1.0 - FRAC_PI_4) * r * r
    }

    /// Distance of the spandrel's centroid from either face.
    fn spandrel_c(r: f64) -> f64 {
        r * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI)
    }

    /// `$pick`: the entity of `kind` whose centroid is nearest to `near` (no ties).
    fn pick(ev: &Evaluated, body: BodyId, kind: TopoKind, near: [f64; 3], key: u32) -> TopoRef {
        let geom = KernelGeom::new(ev, default_kernel());
        let near = amg_core::Point3::from_array(near);
        let mut found: Vec<(f64, u32)> = amg_doc::naming::GeomSource::all(&geom, body, kind)
            .into_iter()
            .map(|(i, fp)| ((fp.centroid - near).length(), i))
            .collect();
        found.sort_by(|a, b| a.0.total_cmp(&b.0));
        assert!(
            found.len() > 1 && found[1].0 - found[0].0 > 1e-6,
            "tied pick near {near:?}"
        );
        let mut r = topo_ref_at(ev, default_kernel(), body, kind, found[0].1).unwrap();
        r.key = key;
        r
    }

    fn edges(doc: &Document, body: BodyId, near: &[[f64; 3]]) -> Vec<TopoRef> {
        near.iter()
            .zip(1..)
            .map(|(p, key)| pick(doc.evaluated(), body, TopoKind::Edge, *p, key))
            .collect()
    }

    fn cube(doc: &mut Document, x0: f64) -> BodyId {
        let (_, e) = extrude_rects(
            doc,
            &[[x0, 0.0, x0 + 20.0, 20.0]],
            20.0,
            BodyOp::NewBody,
            vec![],
        );
        body0(e)
    }

    /// Extrudes a sketch feature by `h` as a new body.
    fn extrude_sketch(doc: &mut Document, s: SketchFeature, h: f64) -> (FeatureId, BodyId) {
        let s = add(doc, FeatureBox::new(s));
        let e = Extrude::distance(profiles(doc, s), &format!("{h} mm"), OpTarget::default());
        let e = add(doc, FeatureBox::new(e));
        assert_eq!(health(doc, e), Health::Ok);
        (e, body0(e))
    }

    /// The slot 0..60 × −10..10 (end arcs R 10), 10 high: a 4-edge tangent top loop.
    fn slot(doc: &mut Document) -> BodyId {
        let mut f = SketchFeature::new(PlaneInput::OriginXY);
        f.edit_sketch(|s| {
            amg_sketch::build::slot_overall(s, Point2::new(0.0, 0.0), Point2::new(60.0, 0.0), 20.0)
        })
        .unwrap();
        extrude_sketch(doc, f, 10.0).1
    }

    /// The rounded rectangle 0..60 × 0..40 with corner radius 8, 10 high: an 8-edge tangent
    /// top loop.
    fn rounded_rect(doc: &mut Document) -> BodyId {
        let mut f = SketchFeature::new(PlaneInput::OriginXY);
        f.edit_sketch(|s| {
            let p = Point2::new;
            s.add_line(p(8.0, 0.0), p(52.0, 0.0))?;
            s.add_arc(p(52.0, 8.0), p(52.0, 0.0), p(60.0, 8.0))?;
            s.add_line(p(60.0, 8.0), p(60.0, 32.0))?;
            s.add_arc(p(52.0, 32.0), p(60.0, 32.0), p(52.0, 40.0))?;
            s.add_line(p(52.0, 40.0), p(8.0, 40.0))?;
            s.add_arc(p(8.0, 32.0), p(8.0, 40.0), p(0.0, 32.0))?;
            s.add_line(p(0.0, 32.0), p(0.0, 8.0))?;
            s.add_arc(p(8.0, 8.0), p(0.0, 8.0), p(8.0, 0.0))?;
            Ok(())
        })
        .unwrap();
        extrude_sketch(doc, f, 10.0).1
    }

    /// Rounded-rectangle area and the top-loop path: straight length and the corner radius.
    const RR_AREA: f64 = 60.0 * 40.0 - 4.0 * 64.0 + 64.0 * PI;
    const RR_STRAIGHT: f64 = 2.0 * 44.0 + 2.0 * 24.0;

    fn add_fillet(doc: &mut Document, f: Fillet) -> FeatureId {
        add(doc, FeatureBox::new(f))
    }

    #[track_caller]
    fn ok(doc: &Document, f: FeatureId) {
        assert_eq!(health(doc, f), Health::Ok, "feature {f:?}");
    }

    #[track_caller]
    fn golden(case: &str, s: &amg_brep::Shape) {
        assert!(
            kernel().check(s, &amg_brep::CheckOpts::default()).valid,
            "{case}: valid"
        );
        check_golden_as("sol05", REL, case, s);
    }

    // ---- Done-when 1: goldens -------------------------------------------------------------

    #[test]
    fn dw1_cube_edge() {
        let mut doc = Document::new("cube-edge");
        let b = cube(&mut doc, 0.0);
        let e = edges(&doc, b, &[[10.0, 0.0, 20.0]]);
        let f = add_fillet(&mut doc, Fillet::constant(e, "2 mm"));
        ok(&doc, f);
        let s = brep(&doc, b);
        assert_rel(volume(&s), 8000.0 - spandrel(2.0) * 20.0, REL);
        assert_eq!(faces(&s), 7);
        golden("cube_edge", &s);
    }

    #[test]
    fn dw1_three_edge_vertex_blend() {
        let mut doc = Document::new("vertex");
        let b = cube(&mut doc, 0.0);
        let e = edges(
            &doc,
            b,
            &[[10.0, 0.0, 20.0], [20.0, 10.0, 20.0], [20.0, 0.0, 10.0]],
        );
        let f = add_fillet(&mut doc, Fillet::constant(e, "3 mm"));
        ok(&doc, f);
        let s = brep(&doc, b);
        let r: f64 = 3.0;
        let removed = spandrel(r) * (60.0 - 3.0 * r) + r.powi(3) * (1.0 - FRAC_PI_6);
        assert_rel(volume(&s), 8000.0 - removed, REL);
        golden("vertex_blend", &s);
    }

    #[test]
    fn dw1_variable_2_to_4() {
        let mut doc = Document::new("variable");
        let b = cube(&mut doc, 0.0);
        let e = edges(&doc, b, &[[10.0, 0.0, 20.0]]);
        let f = Fillet {
            sets: vec![FilletSet {
                edges: e,
                radius: FilletRadius::Variable(vec![
                    VarRadius {
                        at: expr("0"),
                        radius: expr("2 mm"),
                    },
                    VarRadius {
                        at: expr("1"),
                        radius: expr("4 mm"),
                    },
                ]),
            }],
            tangent_chain: true,
            corner: CornerType::RollingBall,
        };
        let f = add_fillet(&mut doc, f);
        ok(&doc, f);
        let s = brep(&doc, b);
        let v = volume(&s);
        assert!(
            v < 8000.0 - spandrel(2.0) * 20.0 && v > 8000.0 - spandrel(4.0) * 20.0,
            "{v}"
        );
        golden("variable_2_4", &s);
    }

    #[test]
    fn dw1_chord_3() {
        let mut doc = Document::new("chord");
        let b = cube(&mut doc, 0.0);
        let e = edges(&doc, b, &[[10.0, 0.0, 20.0]]);
        let f = Fillet {
            sets: vec![FilletSet {
                edges: e,
                radius: FilletRadius::Chord(expr("3 mm")),
            }],
            tangent_chain: true,
            corner: CornerType::RollingBall,
        };
        let f = add_fillet(&mut doc, f);
        ok(&doc, f);
        let s = brep(&doc, b);
        let r = 3.0 / 2.0_f64.sqrt();
        assert_rel(volume(&s), 8000.0 - spandrel(r) * 20.0, REL);
        golden("chord_3", &s);
    }

    #[test]
    fn dw1_two_sets_with_their_own_radii() {
        let mut doc = Document::new("sets");
        let b = cube(&mut doc, 0.0);
        let e = edges(&doc, b, &[[10.0, 0.0, 20.0], [10.0, 20.0, 0.0]]);
        let f = Fillet {
            sets: vec![
                FilletSet {
                    edges: vec![e[0].clone()],
                    radius: FilletRadius::Constant(expr("2 mm")),
                },
                FilletSet {
                    edges: vec![e[1].clone()],
                    radius: FilletRadius::Constant(expr("3 mm")),
                },
            ],
            tangent_chain: true,
            corner: CornerType::RollingBall,
        };
        let f = add_fillet(&mut doc, f);
        ok(&doc, f);
        let s = brep(&doc, b);
        assert_rel(
            volume(&s),
            8000.0 - (spandrel(2.0) + spandrel(3.0)) * 20.0,
            REL,
        );
        golden("two_sets", &s);
    }

    #[test]
    fn dw1_two_bodies_in_one_feature() {
        let mut doc = Document::new("two-bodies");
        let a = cube(&mut doc, 0.0);
        let b = cube(&mut doc, 40.0);
        let mut e = edges(&doc, a, &[[10.0, 0.0, 20.0]]);
        let mut eb = edges(&doc, b, &[[50.0, 0.0, 20.0]]);
        eb[0].key = 2;
        e.append(&mut eb);
        let f = add_fillet(&mut doc, Fillet::constant(e, "2 mm"));
        ok(&doc, f);
        for (body, case) in [(a, "two_bodies_a"), (b, "two_bodies_b")] {
            let s = brep(&doc, body);
            assert_rel(volume(&s), 8000.0 - spandrel(2.0) * 20.0, REL);
            golden(case, &s);
        }
    }

    #[test]
    fn dw1_box_top_loop_mitred() {
        let mut doc = Document::new("top-loop");
        let b = cube(&mut doc, 0.0);
        let e = edges(
            &doc,
            b,
            &[
                [10.0, 0.0, 20.0],
                [20.0, 10.0, 20.0],
                [10.0, 20.0, 20.0],
                [0.0, 10.0, 20.0],
            ],
        );
        let f = add_fillet(&mut doc, Fillet::constant(e, "2 mm"));
        ok(&doc, f);
        let s = brep(&doc, b);
        // Pappus over the mitred loop: the spandrel's centroid runs on the square inset by c.
        let removed = spandrel(2.0) * (80.0 - 8.0 * spandrel_c(2.0));
        assert_rel(volume(&s), 8000.0 - removed, REL);
        golden("top_loop", &s);
    }

    #[test]
    fn dw1_slot_four_edge_tangent_loop_from_one_pick() {
        let mut doc = Document::new("slot");
        let b = slot(&mut doc);
        let e = edges(&doc, b, &[[30.0, -10.0, 10.0]]);
        let f = add_fillet(&mut doc, Fillet::constant(e, "2 mm"));
        ok(&doc, f);
        let s = brep(&doc, b);
        let v0 = (40.0 * 20.0 + PI * 100.0) * 10.0;
        let removed = spandrel(2.0) * (80.0 + 2.0 * PI * (10.0 - spandrel_c(2.0)));
        assert_rel(volume(&s), v0 - removed, REL);
        golden("slot_loop", &s);
    }

    #[test]
    fn dw1_rounded_rectangle_eight_edge_loop_from_one_pick() {
        let mut doc = Document::new("rr");
        let b = rounded_rect(&mut doc);
        let e = edges(&doc, b, &[[30.0, 0.0, 10.0]]);
        let f = add_fillet(&mut doc, Fillet::constant(e, "2 mm"));
        ok(&doc, f);
        let s = brep(&doc, b);
        let removed = spandrel(2.0) * (RR_STRAIGHT + 2.0 * PI * (8.0 - spandrel_c(2.0)));
        assert_rel(volume(&s), RR_AREA * 10.0 - removed, REL);
        golden("rounded_rect_loop", &s);
    }

    #[test]
    fn dw1_concave_edge_adds_material() {
        let mut doc = Document::new("concave");
        let (sk, _, _) = polygon_sketch(&[
            [0.0, 0.0],
            [40.0, 0.0],
            [40.0, 20.0],
            [20.0, 20.0],
            [20.0, 40.0],
            [0.0, 40.0],
        ]);
        let (_, b) = extrude_sketch(&mut doc, sk, 10.0);
        let e = edges(&doc, b, &[[20.0, 20.0, 5.0]]);
        let f = add_fillet(&mut doc, Fillet::constant(e, "3 mm"));
        ok(&doc, f);
        let s = brep(&doc, b);
        assert_rel(volume(&s), 12000.0 + spandrel(3.0) * 10.0, REL);
        golden("concave_edge", &s);
    }

    #[test]
    fn dw1_cylinder_rim() {
        let mut doc = Document::new("rim");
        let mut sk = SketchFeature::new(PlaneInput::OriginXY);
        sk.edit_sketch(|s| s.add_circle(Point2::new(0.0, 0.0), 10.0).map(|_| ()))
            .unwrap();
        let (_, b) = extrude_sketch(&mut doc, sk, 20.0);
        let e = edges(&doc, b, &[[0.0, 0.0, 20.0]]);
        let f = add_fillet(&mut doc, Fillet::constant(e, "2 mm"));
        ok(&doc, f);
        let s = brep(&doc, b);
        let removed = spandrel(2.0) * 2.0 * PI * (10.0 - spandrel_c(2.0));
        assert_rel(volume(&s), PI * 100.0 * 20.0 - removed, REL);
        golden("cylinder_rim", &s);
    }

    fn chamfer_one(doc: &mut Document, b: BodyId, kind: ChamferKind) -> FeatureId {
        let e = edges(doc, b, &[[10.0, 0.0, 20.0]]);
        let c = Chamfer {
            sets: vec![ChamferSet { edges: e, kind }],
            tangent_chain: true,
        };
        add(doc, FeatureBox::new(c))
    }

    /// The area of the (renamed) top face after a chamfer: `top` was picked before it.
    fn face_area(doc: &Document, top: &TopoRef) -> f64 {
        let r = doc.evaluated().resolve(top).unwrap();
        brep(doc, top.body).face_area(amg_brep::FaceId(r.index))
    }

    #[test]
    fn dw1_chamfer_equal() {
        let mut doc = Document::new("ch-eq");
        let b = cube(&mut doc, 0.0);
        let c = chamfer_one(&mut doc, b, ChamferKind::Equal(expr("2 mm")));
        ok(&doc, c);
        let s = brep(&doc, b);
        assert_rel(volume(&s), 8000.0 - 2.0 * 20.0, REL);
        golden("chamfer_equal", &s);
    }

    #[test]
    fn dw1_chamfer_two_distances_measures_d1_on_the_face() {
        let mut doc = Document::new("ch-two");
        let b = cube(&mut doc, 0.0);
        let top = pick(doc.evaluated(), b, TopoKind::Face, [10.0, 10.0, 20.0], 9);
        let kind = ChamferKind::TwoDistances {
            d1: expr("2 mm"),
            d2: expr("3 mm"),
            face: top.clone(),
        };
        let c = chamfer_one(&mut doc, b, kind);
        ok(&doc, c);
        let s = brep(&doc, b);
        assert_rel(volume(&s), 8000.0 - 2.0 * 3.0 / 2.0 * 20.0, REL);
        assert_rel(face_area(&doc, &top), 400.0 - 2.0 * 20.0, REL);
        golden("chamfer_two_distances", &s);
    }

    #[test]
    fn dw1_chamfer_distance_angle() {
        let mut doc = Document::new("ch-da");
        let b = cube(&mut doc, 0.0);
        let top = pick(doc.evaluated(), b, TopoKind::Face, [10.0, 10.0, 20.0], 9);
        let kind = ChamferKind::DistanceAngle {
            d: expr("2 mm"),
            angle: expr("30 deg"),
            face: top.clone(),
        };
        let c = chamfer_one(&mut doc, b, kind);
        ok(&doc, c);
        let s = brep(&doc, b);
        let other = 2.0 * FRAC_PI_6.tan();
        assert_rel(volume(&s), 8000.0 - 2.0 * other / 2.0 * 20.0, REL);
        assert_rel(face_area(&doc, &top), 400.0 - 2.0 * 20.0, REL);
        golden("chamfer_distance_angle", &s);
    }

    #[test]
    fn dw1_chamfer_slot_loop_from_one_pick() {
        let mut doc = Document::new("ch-slot");
        let b = slot(&mut doc);
        let e = edges(&doc, b, &[[30.0, -10.0, 10.0]]);
        let c = add(&mut doc, FeatureBox::new(Chamfer::equal(e, "2 mm")));
        ok(&doc, c);
        let s = brep(&doc, b);
        let v0 = (40.0 * 20.0 + PI * 100.0) * 10.0;
        let removed = 2.0 * (80.0 + 2.0 * PI * (10.0 - 2.0 / 3.0));
        assert_rel(volume(&s), v0 - removed, REL);
        golden("chamfer_slot_loop", &s);
    }

    #[test]
    fn dw1_chamfer_three_edge_vertex() {
        let mut doc = Document::new("ch-vertex");
        let b = cube(&mut doc, 0.0);
        let e = edges(
            &doc,
            b,
            &[[10.0, 0.0, 20.0], [20.0, 10.0, 20.0], [20.0, 0.0, 10.0]],
        );
        let c = add(&mut doc, FeatureBox::new(Chamfer::equal(e, "2 mm")));
        ok(&doc, c);
        let s = brep(&doc, b);
        // Three wedges along the edges, the corner trimmed (BREP-06: OCCT adds a corner face).
        let v = volume(&s);
        let wedges = 2.0 * 60.0;
        assert!(
            v < 8000.0 - wedges + 8.0 && v > 8000.0 - wedges - 8.0,
            "{v}"
        );
        golden("chamfer_vertex", &s);
    }

    // ---- Done-when 2: tangent chains --------------------------------------------------------

    #[test]
    fn dw2_one_pick_on_a_rounded_rectangle_selects_all_8_tangent_edges() {
        let mut doc = Document::new("dw2");
        let b = rounded_rect(&mut doc);
        let e = edges(&doc, b, &[[30.0, 0.0, 10.0]]);
        let ctx = RecomputeCtx::new(
            doc.design(),
            doc.evaluated(),
            FeatureId(9_999),
            doc.design().root(),
            ComputeMode::Full,
        );
        let mut f = Fillet::constant(e.clone(), "2 mm");
        let plan = f.plan(&ctx).unwrap();
        assert_eq!(plan.len(), 1);
        assert_eq!(plan[0].body, b);
        assert_eq!(plan[0].edges.len(), 8, "{plan:?}");
        assert_eq!(plan[0].edges[0].key, Some(1), "the pick comes first");
        assert!(plan[0].edges[1..].iter().all(|x| x.key.is_none()));
        let s = brep(&doc, b);
        for x in &plan[0].edges {
            let ends = kernel().edge_ends(&s, x.edge).unwrap();
            assert!((ends.p0.z - 10.0).abs() < 1e-9 && (ends.p1.z - 10.0).abs() < 1e-9);
        }
        // The helper agrees, from any edge of the loop.
        let all = tangent_chain_edges(kernel(), &s, &[plan[0].edges[3].edge], 1e-3).unwrap();
        assert_eq!(all.len(), 8);

        f.tangent_chain = false;
        let plan = f.plan(&ctx).unwrap();
        assert_eq!(plan[0].edges.len(), 1);
        assert_eq!(plan[0].edges[0].key, Some(1));
        drop(ctx);

        // A box has no tangent chains: a picked edge stays alone.
        let c = cube(&mut doc, 100.0);
        let ce = edges(&doc, c, &[[110.0, 0.0, 20.0]]);
        let ctx = RecomputeCtx::new(
            doc.design(),
            doc.evaluated(),
            FeatureId(9_999),
            doc.design().root(),
            ComputeMode::Full,
        );
        let plan = Fillet::constant(ce, "1 mm").plan(&ctx).unwrap();
        assert_eq!(plan[0].edges.len(), 1);
    }

    // ---- Done-when 3: failing-edge highlight --------------------------------------------------

    #[track_caller]
    fn error_entities(doc: &Document, f: FeatureId) -> Vec<EntityRef> {
        let Health::Error(d) = health(doc, f) else {
            panic!("{:?}", health(doc, f));
        };
        d.entities
    }

    #[test]
    fn dw3_a_radius_larger_than_the_face_highlights_exactly_that_edge() {
        let mut doc = Document::new("dw3");
        let (_, e) = extrude_rects(
            &mut doc,
            &[[0.0, 0.0, 40.0, 40.0]],
            5.0,
            BodyOp::NewBody,
            vec![],
        );
        let b = body0(e);
        // The top-front edge of a 5 mm plate with a 7 mm radius: wider than the front face.
        let picks = edges(&doc, b, &[[20.0, 0.0, 5.0]]);
        let edge_index = doc.evaluated().resolve(&picks[0]).unwrap().index;
        let f = add_fillet(&mut doc, Fillet::constant(picks, "7 mm"));
        let ents = error_entities(&doc, f);
        assert_eq!(ents.len(), 1, "{ents:?}");
        let EntityRef::Topo(t) = &ents[0] else {
            panic!("{ents:?}");
        };
        assert_eq!((t.body, t.kind), (b, TopoKind::Edge));
        let r = doc.evaluated().resolve(t).unwrap();
        assert_eq!((r.index, r.quality), (edge_index, MatchQuality::Exact));
    }

    #[test]
    fn dw3_with_two_sets_only_the_failing_set_is_highlighted() {
        let mut doc = Document::new("dw3b");
        let (_, e) = extrude_rects(
            &mut doc,
            &[[0.0, 0.0, 40.0, 40.0]],
            5.0,
            BodyOp::NewBody,
            vec![],
        );
        let b = body0(e);
        let picks = edges(&doc, b, &[[20.0, 0.0, 5.0], [20.0, 40.0, 0.0]]);
        let bad = doc.evaluated().resolve(&picks[0]).unwrap().index;
        let f = Fillet {
            sets: vec![
                FilletSet {
                    edges: vec![picks[0].clone()],
                    radius: FilletRadius::Constant(expr("7 mm")),
                },
                FilletSet {
                    edges: vec![picks[1].clone()],
                    radius: FilletRadius::Constant(expr("1 mm")),
                },
            ],
            tangent_chain: true,
            corner: CornerType::RollingBall,
        };
        let f = add_fillet(&mut doc, f);
        let ents = error_entities(&doc, f);
        let idx: Vec<u32> = ents
            .iter()
            .map(|x| match x {
                EntityRef::Topo(t) => doc.evaluated().resolve(t).unwrap().index,
                other => panic!("{other:?}"),
            })
            .collect();
        assert_eq!(idx, vec![bad]);
    }

    // ---- naming: edit-and-resolve, Done-when 4 (TNP-11, TNP-27) --------------------------------

    fn probe(doc: &mut Document, body: BodyId, r: TopoRef) -> FeatureId {
        add(
            doc,
            FeatureBox::new(TestProbe {
                body,
                refs: vec![r],
                policy: SetPolicy::AllOrError,
            }),
        )
    }

    fn resolution(doc: &Document, f: FeatureId) -> (MatchQuality, GeomFingerprint) {
        let node = doc.evaluated().node(f).unwrap();
        let rec = &node.resolutions[0];
        let q = rec.result.as_ref().expect("resolved")[0].quality;
        let Some(EntityRef::Topo(t)) = rec.current.first() else {
            panic!("{rec:?}");
        };
        (q, t.fingerprint.clone().unwrap())
    }

    #[test]
    fn a_blend_face_keeps_its_name_when_the_radius_is_edited() {
        let mut doc = Document::new("edit-radius");
        let b = cube(&mut doc, 0.0);
        let e = edges(&doc, b, &[[10.0, 0.0, 20.0]]);
        let fil = add_fillet(&mut doc, Fillet::constant(e.clone(), "2 mm"));
        let blend = pick(doc.evaluated(), b, TopoKind::Face, [10.0, 0.6, 19.4], 1);
        let p = probe(&mut doc, b, blend);
        ok(&doc, p);
        edit(&mut doc, fil, FeatureBox::new(Fillet::constant(e, "4 mm")));
        ok(&doc, p);
        let (q, fp) = resolution(&doc, p);
        assert_eq!(q, MatchQuality::Exact);
        assert_eq!(fp.class, GeomClass::Cylinder);
        assert!((fp.radius.unwrap() - 4.0).abs() < 1e-9);
        assert_rel(fp.measure, 0.5 * PI * 4.0 * 20.0, 1e-6);
    }

    fn b1(doc: &mut Document, height: f64) -> (FeatureId, BodyId) {
        let f = add(
            doc,
            FeatureBox::new(TestBrepBox {
                size: [100.0, 60.0, height],
            }),
        );
        (f, body0(f))
    }

    #[test]
    fn dw4_tnp11_an_inserted_fillet_on_the_same_edge_gives_not_found() {
        let mut doc = Document::new("tnp11");
        let (_, b) = b1(&mut doc, 40.0);
        let e = edges(&doc, b, &[[50.0, 0.0, 40.0]]);
        let fil = add_fillet(&mut doc, Fillet::constant(e.clone(), "5 mm"));
        ok(&doc, fil);
        let at = doc.design().timeline().node(fil).unwrap().0;
        let root = doc.design().root();
        doc.apply(Command::AddFeature {
            at: Some(at),
            feature: FeatureBox::new(Fillet::constant(e, "3 mm")),
            context: root,
            name: None,
        })
        .unwrap();
        doc.recompute_blocking(jobs()).unwrap();
        let node = doc.evaluated().node(fil).unwrap();
        assert!(
            matches!(
                node.resolutions[0].result,
                Err(ResolveError::NotFound {
                    kind: TopoKind::Edge,
                    ..
                })
            ),
            "{:?}",
            node.resolutions[0]
        );
        assert!(matches!(node.health, Health::Error(_)), "{:?}", node.health);
    }

    #[test]
    fn dw4_tnp27_the_corner_patch_survives_a_height_edit() {
        let mut doc = Document::new("tnp27");
        let (bx, b) = b1(&mut doc, 40.0);
        let e = edges(
            &doc,
            b,
            &[[50.0, 0.0, 40.0], [100.0, 30.0, 40.0], [100.0, 0.0, 20.0]],
        );
        let fil = add_fillet(&mut doc, Fillet::constant(e, "5 mm"));
        ok(&doc, fil);
        let corner = pick(doc.evaluated(), b, TopoKind::Face, [97.5, 2.5, 37.5], 1);
        let p = probe(&mut doc, b, corner);
        ok(&doc, p);
        edit(
            &mut doc,
            bx,
            FeatureBox::new(TestBrepBox {
                size: [100.0, 60.0, 50.0],
            }),
        );
        ok(&doc, fil);
        ok(&doc, p);
        let (q, fp) = resolution(&doc, p);
        assert_eq!(q, MatchQuality::Exact);
        assert_eq!(fp.class, GeomClass::Sphere);
        let want = amg_core::Point3::new(97.5, 2.5, 47.5);
        assert!((fp.centroid - want).length() < 1e-4, "{fp:?}");
        assert!((fp.measure - 39.26991).abs() < 1e-4, "{fp:?}");
    }

    // ---- Done-when 6: the corpus-harness entry point -----------------------------------------

    fn harness_box() -> amg_brep::Shape {
        kernel()
            .make_box(&Plane::XY, Vec3::new(20.0, 20.0, 20.0))
            .unwrap()
            .shape
    }

    /// The edge of `s` nearest to `p` (by its mid point).
    fn edge_near(s: &amg_brep::Shape, p: [f64; 3]) -> EdgeId {
        let p = amg_core::Point3::from_array(p);
        s.edges()
            .min_by(|a, b| {
                let m = |e: EdgeId| {
                    let ends = kernel().edge_ends(s, e).unwrap();
                    let mid = amg_core::Point3::new(
                        0.5 * (ends.p0.x + ends.p1.x),
                        0.5 * (ends.p0.y + ends.p1.y),
                        0.5 * (ends.p0.z + ends.p1.z),
                    );
                    (mid - p).length()
                };
                m(*a).total_cmp(&m(*b))
            })
            .unwrap()
    }

    #[test]
    fn dw6_the_harness_drives_solid_fillet_on_a_corpus_case() {
        let s = harness_box();
        let e = edge_near(&s, [10.0, 0.0, 20.0]);
        let specs = [(e, FilletSpec::Constant(2.0))];
        let run = harness::run_fillet(
            kernel(),
            &FilletCase {
                shape: &s,
                edges: &specs,
                tangent_chain: true,
                mode: ComputeMode::Full,
            },
        )
        .unwrap();
        assert_rel(volume(&run.shape), 8000.0 - spandrel(2.0) * 20.0, REL);
        assert_eq!(run.naming.list(TopoKind::Face).map(<[_]>::len), Some(7));

        // Variable and chord specs pass through, and so does preview mode.
        let specs = [
            (e, FilletSpec::Variable(vec![(0.0, 1.0), (1.0, 2.0)])),
            (edge_near(&s, [10.0, 20.0, 0.0]), FilletSpec::Chord(2.0)),
        ];
        let run = harness::run_fillet(
            kernel(),
            &FilletCase {
                shape: &s,
                edges: &specs,
                tangent_chain: false,
                mode: ComputeMode::Preview,
            },
        )
        .unwrap();
        assert!(volume(&run.shape) < 8000.0);

        // A clean failure names its failing edge in the case's index space.
        let big = [(e, FilletSpec::Constant(25.0))];
        let err = harness::run_fillet(
            kernel(),
            &FilletCase {
                shape: &s,
                edges: &big,
                tangent_chain: true,
                mode: ComputeMode::Full,
            },
        )
        .unwrap_err();
        assert!(matches!(err.error, FeatureError::Kernel(_)), "{err:?}");
        assert_eq!(err.failing, vec![e]);
        assert_eq!(s.count(SubKind::Edge), 12);
    }

    // ---- Testing §: random radii never panic -------------------------------------------------

    proptest::proptest! {
        #![proptest_config(proptest::prelude::ProptestConfig { cases: 16, ..Default::default() })]
        #[test]
        fn random_radii_give_a_valid_solid_or_a_clean_error(r in 0.01f64..30.0, edge in 0u32..12) {
            let s = harness_box();
            let specs = [(EdgeId(edge), FilletSpec::Constant(r))];
            match harness::run_fillet(kernel(), &FilletCase {
                shape: &s, edges: &specs, tangent_chain: true, mode: ComputeMode::Full,
            }) {
                Ok(run) => {
                    proptest::prop_assert!(kernel().check(&run.shape, &amg_brep::CheckOpts::default()).valid);
                    proptest::prop_assert!(volume(&run.shape) < 8000.0);
                }
                Err(e) => proptest::prop_assert!(matches!(e.error, FeatureError::Kernel(_)), "{e:?}"),
            }
        }
    }
}
