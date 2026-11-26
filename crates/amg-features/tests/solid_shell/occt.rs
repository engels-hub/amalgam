//! The OCCT half of the SOL-06 tests: goldens `tests/golden/solid/sol06_*.toml`, analytic
//! volumes, draft angles, the collapse highlight, TNP-X3 and TNP-X6.

use amg_brep::geom::{BSplineSurfaceData, FaceBounds, Surface};
use amg_brep::{BrepKernel, CheckOpts, FaceId, ShapeHistory, SubKind, SubShape};
use amg_core::{Plane, Point3};
use amg_doc::naming::{GeomSource, KernelGeom, Role, default_kernel, topo_ref_at};
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, ComputeMode, Expr, ExprSrc, FeatureError, FeatureOutput,
    Health, MatchQuality, RecomputeCtx, register_feature,
};
use amg_features::construct::{ConstructPlane, PlaneDef};
use amg_features::solid::{Extrude, Fillet};
use serde::{Deserialize, Serialize};

use super::*;

const REL: f64 = 1e-9;

fn ex(s: &str) -> Expr {
    Expr::Inline(ExprSrc(s.to_owned()))
}

#[track_caller]
fn golden(case: &str, s: &amg_brep::Shape) {
    check_golden_as("sol06", REL, case, s);
}

#[track_caller]
fn ok(doc: &Document, f: FeatureId) {
    assert_eq!(health(doc, f), Health::Ok, "feature {f:?}");
}

#[track_caller]
fn valid(s: &amg_brep::Shape) {
    assert!(kernel().check(s, &CheckOpts::default()).valid, "valid");
}

/// A box `[x0, y0, x1, y1]` × `0..h` as a new body (sketch + extrude).
fn block(doc: &mut Document, r: [f64; 4], h: f64) -> BodyId {
    body0(extrude_rects(doc, &[r], h, BodyOp::NewBody, vec![]).1)
}

/// `$pick`: the entity of `kind` whose centroid is nearest to `near` (no ties).
fn pick(
    ev: &amg_doc::Evaluated,
    body: BodyId,
    kind: TopoKind,
    near: [f64; 3],
    key: u32,
) -> TopoRef {
    let geom = KernelGeom::new(ev, default_kernel());
    let near = Point3::from_array(near);
    let mut found: Vec<(f64, u32)> = GeomSource::all(&geom, body, kind)
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

/// The first reference of `f`: its quality and current fingerprint.
fn first_resolution(doc: &Document, f: FeatureId) -> (MatchQuality, amg_doc::GeomFingerprint) {
    let node = doc.evaluated().node(f).unwrap();
    let rec = &node.resolutions[0];
    let res = &rec.result.as_ref().expect("resolved")[0];
    let Some(EntityRef::Topo(t)) = rec.current.first() else {
        panic!("{rec:?}");
    };
    (res.quality, t.fingerprint.clone().unwrap())
}

/// A context just after the last feature (for `plan`).
fn ctx_after(doc: &Document) -> RecomputeCtx<'_> {
    RecomputeCtx::new(
        doc.design(),
        doc.evaluated(),
        FeatureId(u64::MAX / 2),
        doc.design().root(),
        ComputeMode::Full,
    )
}

/// The outward normal of face `f` at its centroid.
fn normal_at_centroid(s: &amg_brep::Shape, f: FaceId) -> amg_core::Vec3 {
    let k = kernel();
    let g = k
        .sub_geom(
            s,
            SubShape {
                kind: SubKind::Face,
                index: f.0,
            },
        )
        .unwrap();
    let (uv, _) = k.project_point_on_face(s, f, g.centroid).unwrap();
    k.eval_surface(s, f, uv).unwrap().normal.get()
}

// ---- test bodies: a sphere and a B-spline sheet -----------------------------------------------

/// Creates body slot 0: a sphere of radius `sphere` at the origin, or (`None`) a bicubic
/// Bézier sheet over 40 × 40 mm with its 4 middle poles raised by 2 mm. Faces `Surface#i`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Sol06Shape {
    sphere: Option<f64>,
}

fn bspline_sheet(k: &dyn BrepKernel) -> Result<amg_brep::Shape, amg_brep::KernelError> {
    let mut poles = Vec::new();
    for i in 0..4 {
        for j in 0..4 {
            let z = if (1..3).contains(&i) && (1..3).contains(&j) {
                2.0
            } else {
                0.0
            };
            poles.push(Point3::new(
                40.0 * f64::from(i) / 3.0,
                40.0 * f64::from(j) / 3.0,
                z,
            ));
        }
    }
    let data = BSplineSurfaceData {
        u_degree: 3,
        v_degree: 3,
        u_knots: vec![0.0, 1.0],
        u_mults: vec![4, 4],
        v_knots: vec![0.0, 1.0],
        v_mults: vec![4, 4],
        poles,
        nu: 4,
        nv: 4,
        weights: None,
        u_periodic: false,
        v_periodic: false,
    };
    k.make_face(&Surface::BSpline(data), &FaceBounds::Natural, 1e-6)
        .map(|b| b.face)
}

impl Feature for Sol06Shape {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let k = ctx.kernel();
        let kerr = |e: amg_brep::KernelError| FeatureError::Kernel(e.to_string());
        let shape = match self.sphere {
            Some(r) => k.make_sphere(&Plane::XY, r).map_err(kerr)?.shape,
            None => bspline_sheet(k).map_err(kerr)?,
        };
        let mut b = ctx.naming_builder();
        b.from_history(&[], &ShapeHistory::default());
        for f in 0..shape.count(SubKind::Face) {
            b.generated(TopoKind::Face, f, Role::new("Surface", f), None);
        }
        let naming = b
            .finish(k, &shape)
            .map_err(|e| FeatureError::Kernel(e.to_string()))?;
        Ok(FeatureOutput {
            bodies: vec![BodyChange {
                id: ctx.new_body_id(0),
                change: BodyChangeKind::Created {
                    geom: BodyGeom::Solid(amg_doc::stub::Shape::from_brep(shape)),
                    naming: Some(naming),
                },
            }],
            ..FeatureOutput::default()
        })
    }
}

register_feature!(Sol06Shape, "test.sol06_shape", version = 1);

fn shape_body(doc: &mut Document, sphere: Option<f64>) -> BodyId {
    body0(add(doc, FeatureBox::new(Sol06Shape { sphere })))
}

// ---- Done-when 1: shelled and hollow boxes ----------------------------------------------------

const A: f64 = 40.0;
const B: f64 = 30.0;
const C: f64 = 20.0;

#[test]
fn dw1_shelled_and_hollow_boxes_match_analytic_volumes() {
    let abc = A * B * C;
    let open = [
        (
            "inside",
            ShellDirection::Inside,
            abc - (A - 2.0) * (B - 2.0) * (C - 1.0),
        ),
        (
            "outside",
            ShellDirection::Outside,
            (A + 2.0) * (B + 2.0) * (C + 1.0) - abc,
        ),
        (
            "both",
            ShellDirection::Both,
            (A + 2.0) * (B + 2.0) * (C + 1.0) - (A - 2.0) * (B - 2.0) * (C - 1.0),
        ),
    ];
    for (name, dir, want) in open {
        let mut doc = Document::new(name);
        let b = block(&mut doc, [0.0, 0.0, A, B], C);
        let top = pick(doc.evaluated(), b, TopoKind::Face, [20.0, 15.0, 20.0], 1);
        let mut s = Shell::inside(vec![top], "1 mm");
        s.direction = dir;
        let f = add(&mut doc, FeatureBox::new(s));
        ok(&doc, f);
        let sh = brep(&doc, b);
        valid(&sh);
        assert_rel(volume(&sh), want, REL);
        assert_eq!(sh.solids().len(), 1, "{name}");
        golden(&format!("box_top_{name}"), &sh);
    }
    // No face removed (R-S04-8): a closed void.
    let hollow = [
        (
            "inside",
            ShellDirection::Inside,
            abc - (A - 2.0) * (B - 2.0) * (C - 2.0),
        ),
        (
            "outside",
            ShellDirection::Outside,
            (A + 2.0) * (B + 2.0) * (C + 2.0) - abc,
        ),
        (
            "both",
            ShellDirection::Both,
            (A + 2.0) * (B + 2.0) * (C + 2.0) - (A - 2.0) * (B - 2.0) * (C - 2.0),
        ),
    ];
    for (name, dir, want) in hollow {
        let mut doc = Document::new(name);
        let b = block(&mut doc, [0.0, 0.0, A, B], C);
        let mut s = Shell::inside(Vec::new(), "1 mm");
        s.bodies = vec![b];
        s.direction = dir;
        let f = add(&mut doc, FeatureBox::new(s));
        ok(&doc, f);
        let sh = brep(&doc, b);
        valid(&sh);
        assert_rel(volume(&sh), want, REL);
        assert_eq!(faces(&sh), 12, "{name}: outer and inner box");
        golden(&format!("hollow_{name}"), &sh);
    }
}

#[test]
fn both_takes_separate_inside_and_outside_thicknesses() {
    let mut doc = Document::new("both");
    let b = block(&mut doc, [0.0, 0.0, A, B], C);
    let top = pick(doc.evaluated(), b, TopoKind::Face, [20.0, 15.0, 20.0], 1);
    let mut s = Shell::inside(vec![top], "1 mm");
    s.direction = ShellDirection::Both;
    s.outside = ex("2 mm");
    let f = add(&mut doc, FeatureBox::new(s));
    ok(&doc, f);
    let sh = brep(&doc, b);
    valid(&sh);
    let want = (A + 4.0) * (B + 4.0) * (C + 2.0) - (A - 2.0) * (B - 2.0) * (C - 1.0);
    assert_rel(volume(&sh), want, REL);
    let bb = sh.bbox();
    assert!(
        (bb.max.z - C).abs() < 1e-6,
        "the opening stays at the top plane"
    );
    assert!((bb.min.z + 2.0).abs() < 1e-6);
}

// ---- Done-when 2: tangent chain on a filleted box ---------------------------------------------

/// The box with its 4 top edges filleted (r 3): 10 faces.
fn filleted_box(doc: &mut Document) -> BodyId {
    let b = block(doc, [0.0, 0.0, A, B], C);
    let edges = [
        [20.0, 0.0, 20.0],
        [40.0, 15.0, 20.0],
        [20.0, 30.0, 20.0],
        [0.0, 15.0, 20.0],
    ]
    .iter()
    .zip(1..)
    .map(|(p, k)| pick(doc.evaluated(), b, TopoKind::Edge, *p, k))
    .collect();
    let fil = add(doc, FeatureBox::new(Fillet::constant(edges, "3 mm")));
    ok(doc, fil);
    assert_eq!(faces(&brep(doc, b)), 10);
    b
}

#[test]
fn dw2_shelling_a_filleted_box_with_tangent_chain_removes_the_fillet_ring() {
    let mut doc = Document::new("chain");
    let b = filleted_box(&mut doc);
    let top = pick(doc.evaluated(), b, TopoKind::Face, [20.0, 15.0, 20.0], 1);
    let mut s = Shell::inside(vec![top.clone()], "1 mm");
    let plan = s.plan(&ctx_after(&doc)).unwrap();
    assert_eq!(plan.len(), 1);
    assert_eq!(plan[0].faces.len(), 5, "top + 4 fillets");
    assert_eq!(plan[0].faces[0].key, Some(1));
    assert!(plan[0].faces[1..].iter().all(|f| f.key.is_none()));
    s.tangent_chain = false;
    assert_eq!(s.plan(&ctx_after(&doc)).unwrap()[0].faces.len(), 1);

    // With the chain: bottom + 4 sides outside, 5 inner faces, and the rim. OCCT closes an
    // opening on the removed faces' own surfaces, so the rim is 4 pieces of the fillet
    // cylinders (between the outer sides at z = 17 and the inner walls).
    let f = add(&mut doc, FeatureBox::new(Shell::inside(vec![top], "1 mm")));
    ok(&doc, f);
    let sh = brep(&doc, b);
    valid(&sh);
    let [fs, _, _] = kernel().sub_geom_all(&sh).unwrap();
    assert_eq!(fs.len(), 14, "5 outer + 5 inner + 4 rim faces");
    let cylinders = fs
        .iter()
        .filter(|g| g.class == amg_brep::geom::GeomClass::Cylinder)
        .count();
    assert_eq!(
        cylinders, 4,
        "only the rim pieces of the fillets are curved"
    );
    assert!(
        sh.bbox().max.z < C - 0.5,
        "the top and its fillet ring are open"
    );
    let naming = doc.evaluated().body(b).unwrap().naming.clone().unwrap();
    let rims = (0..sh.count(SubKind::Face))
        .filter_map(|i| naming.name_of(TopoKind::Face, i))
        .filter(|n| format!("{n:?}").contains("ShellRim"))
        .count();
    assert_eq!(rims, 4, "a rim face per removed fillet");
    golden("filleted_box_chain", &sh);

    // Without the chain the fillets stay tangent to the open top, which OCCT can't shell (it
    // returns the body unchanged): a clear error with the fillets highlighted.
    let mut doc = Document::new("no chain");
    let b = filleted_box(&mut doc);
    let before = brep(&doc, b);
    let top = pick(doc.evaluated(), b, TopoKind::Face, [20.0, 15.0, 20.0], 1);
    let mut s = Shell::inside(vec![top], "1 mm");
    s.tangent_chain = false;
    let f = add(&mut doc, FeatureBox::new(s));
    let Health::Error(d) = health(&doc, f) else {
        panic!("expected an error, got {:?}", health(&doc, f));
    };
    assert!(d.message.contains("Tangent chain"), "{}", d.message);
    assert_eq!(d.entities.len(), 4, "the 4 fillets");
    for ent in &d.entities {
        let EntityRef::Topo(t) = ent else {
            panic!("{ent:?}");
        };
        let r = doc.evaluated().resolve(t).unwrap();
        let g = kernel()
            .sub_geom(
                &before,
                SubShape {
                    kind: SubKind::Face,
                    index: r.index,
                },
            )
            .unwrap();
        assert_eq!(g.class, amg_brep::geom::GeomClass::Cylinder);
    }
}

// ---- Done-when 3: a wall thicker than a concave radius ----------------------------------------

#[test]
fn dw3_an_outward_wall_above_the_concave_radius_fails_with_the_faces_highlighted() {
    let mut doc = Document::new("collapse");
    // A plate with a boss on top; the boss's base edge on y = 10 gets a concave 1 mm fillet.
    let plate = block(&mut doc, [0.0, 0.0, 40.0, 30.0], 10.0);
    let (_, boss) = extrude_rects(
        &mut doc,
        &[[10.0, 10.0, 30.0, 20.0]],
        20.0,
        BodyOp::Join,
        vec![plate],
    );
    ok(&doc, boss);
    let e = pick(
        doc.evaluated(),
        plate,
        TopoKind::Edge,
        [20.0, 10.0, 10.0],
        1,
    );
    let mut fil = Fillet::constant(vec![e], "1 mm");
    fil.tangent_chain = false;
    let fil = add(&mut doc, FeatureBox::new(fil));
    ok(&doc, fil);
    let before = brep(&doc, plate);
    let bottom = pick(doc.evaluated(), plate, TopoKind::Face, [20.0, 15.0, 0.0], 1);
    let mut s = Shell::inside(vec![bottom], "2 mm");
    s.direction = ShellDirection::Outside;
    s.outside = ex("2 mm");
    let f = add(&mut doc, FeatureBox::new(s));
    let Health::Error(d) = health(&doc, f) else {
        panic!("expected an error, got {:?}", health(&doc, f));
    };
    assert!(d.message.contains("shell"), "{}", d.message);
    assert!(!d.entities.is_empty(), "the failing faces are highlighted");
    for ent in &d.entities {
        let EntityRef::Topo(t) = ent else {
            panic!("{ent:?}");
        };
        assert_eq!(t.kind, TopoKind::Face);
        let r = doc.evaluated().resolve(t).unwrap();
        let g = kernel()
            .sub_geom(
                &before,
                SubShape {
                    kind: SubKind::Face,
                    index: r.index,
                },
            )
            .unwrap();
        assert_eq!(
            g.class,
            amg_brep::geom::GeomClass::Cylinder,
            "the concave fillet: {g:?}"
        );
        assert!((g.centroid.y - 10.0).abs() < 1.0 && (g.centroid.z - 10.0).abs() < 1.0);
    }
    // A 0.5 mm wall stays below the radius and works.
    let mut s = Shell::inside(
        vec![pick(
            doc.evaluated(),
            plate,
            TopoKind::Face,
            [20.0, 15.0, 0.0],
            1,
        )],
        "0.5 mm",
    );
    s.direction = ShellDirection::Outside;
    s.outside = ex("0.5 mm");
    let f = add(&mut doc, FeatureBox::new(s));
    ok(&doc, f);
    valid(&brep(&doc, plate));
}

// ---- Done-when 4: draft ----------------------------------------------------------------------

/// Volume of an `a × b` box of height `h` whose sides narrow by `h tan θ` towards the top.
fn drafted(a: f64, b: f64, h: f64, theta: f64) -> f64 {
    let t = theta.tan();
    a * b * h - (a + b) * t * h * h + 4.0 * t * t * h.powi(3) / 3.0
}

fn side_faces(doc: &Document, b: BodyId) -> Vec<TopoRef> {
    [
        [20.0, 0.0, 10.0],
        [40.0, 15.0, 10.0],
        [20.0, 30.0, 10.0],
        [0.0, 15.0, 10.0],
    ]
    .iter()
    .zip(1..)
    .map(|(p, k)| pick(doc.evaluated(), b, TopoKind::Face, *p, k))
    .collect()
}

/// `asin(n · z)` of every face whose normal is within 45° of horizontal.
fn side_angles(s: &amg_brep::Shape) -> Vec<(f64, f64)> {
    let k = kernel();
    s.faces()
        .filter_map(|f| {
            let n = normal_at_centroid(s, f);
            let c = k
                .sub_geom(
                    s,
                    SubShape {
                        kind: SubKind::Face,
                        index: f.0,
                    },
                )
                .unwrap()
                .centroid;
            (n.z.abs() < 0.7).then(|| (n.z.asin(), c.z))
        })
        .collect()
}

#[test]
fn dw4_draft_of_four_box_sides_at_3_degrees_one_side_and_symmetric() {
    let deg3 = 3f64.to_radians();
    let mut doc = Document::new("draft");
    let b = block(&mut doc, [0.0, 0.0, A, B], C);
    let sides = side_faces(&doc, b);
    let f = add(
        &mut doc,
        FeatureBox::new(Draft::one_side(
            RefInput::Origin(OriginEntity::XYPlane),
            sides,
            "3 deg",
        )),
    );
    ok(&doc, f);
    let sh = brep(&doc, b);
    valid(&sh);
    assert_eq!(faces(&sh), 6);
    let angles = side_angles(&sh);
    assert_eq!(angles.len(), 4);
    for (a, _) in &angles {
        assert!((a - deg3).abs() < 1e-9, "angle {} deg", a.to_degrees());
    }
    assert_rel(volume(&sh), drafted(A, B, C, deg3), REL);
    golden("draft_one_side_3", &sh);

    // Symmetric about the mid plane z = 10: both halves narrow away from it.
    let mut doc = Document::new("draft sym");
    let b = block(&mut doc, [0.0, 0.0, A, B], C);
    let mid = add(
        &mut doc,
        FeatureBox::new(ConstructPlane {
            def: PlaneDef::Offset {
                base: RefInput::Origin(OriginEntity::XYPlane),
                distance: ex("10 mm"),
            },
        }),
    );
    let sides = side_faces(&doc, b);
    let mut d = Draft::one_side(
        RefInput::Ref(EntityRef::Construction(ConstructionId(mid))),
        sides,
        "3 deg",
    );
    d.sides = DraftSides::Symmetric;
    let f = add(&mut doc, FeatureBox::new(d));
    ok(&doc, f);
    let sh = brep(&doc, b);
    valid(&sh);
    assert_eq!(faces(&sh), 10, "4 sides split in two");
    let angles = side_angles(&sh);
    assert_eq!(angles.len(), 8);
    for (a, z) in &angles {
        let want = if *z > 10.0 { deg3 } else { -deg3 };
        assert!(
            (a - want).abs() < 1e-9,
            "angle {} deg at z {z}",
            a.to_degrees()
        );
    }
    assert_rel(volume(&sh), 2.0 * drafted(A, B, 10.0, deg3), REL);
    golden("draft_symmetric_3", &sh);

    // Two sides, 3° above and 5° below.
    let mut d = Draft::one_side(
        RefInput::Ref(EntityRef::Construction(ConstructionId(mid))),
        Vec::new(),
        "3 deg",
    );
    d.sides = DraftSides::TwoSides {
        angle2: ex("5 deg"),
    };
    let mut doc2 = Document::new("draft two");
    let b2 = block(&mut doc2, [0.0, 0.0, A, B], C);
    let mid2 = add(
        &mut doc2,
        FeatureBox::new(ConstructPlane {
            def: PlaneDef::Offset {
                base: RefInput::Origin(OriginEntity::XYPlane),
                distance: ex("10 mm"),
            },
        }),
    );
    d.plane = RefInput::Ref(EntityRef::Construction(ConstructionId(mid2)));
    d.faces = side_faces(&doc2, b2);
    let f = add(&mut doc2, FeatureBox::new(d));
    ok(&doc2, f);
    let sh = brep(&doc2, b2);
    valid(&sh);
    let want = drafted(A, B, 10.0, deg3) + drafted(A, B, 10.0, 5f64.to_radians());
    assert_rel(volume(&sh), want, REL);
    golden("draft_two_sides_3_5", &sh);
}

#[test]
fn a_flipped_draft_narrows_downwards() {
    let deg3 = 3f64.to_radians();
    let mut doc = Document::new("flip");
    let b = block(&mut doc, [0.0, 0.0, A, B], C);
    let top_plane = add(
        &mut doc,
        FeatureBox::new(ConstructPlane {
            def: PlaneDef::Offset {
                base: RefInput::Origin(OriginEntity::XYPlane),
                distance: ex("20 mm"),
            },
        }),
    );
    let mut d = Draft::one_side(
        RefInput::Ref(EntityRef::Construction(ConstructionId(top_plane))),
        side_faces(&doc, b),
        "3 deg",
    );
    d.flip = true;
    let f = add(&mut doc, FeatureBox::new(d));
    ok(&doc, f);
    let sh = brep(&doc, b);
    valid(&sh);
    assert_rel(volume(&sh), drafted(A, B, C, deg3), REL);
    for (a, _) in side_angles(&sh) {
        assert!((a + deg3).abs() < 1e-9);
    }
}

// ---- Done-when 5: scale -----------------------------------------------------------------------

#[test]
fn dw5_uniform_and_non_uniform_scale_volumes() {
    let mut doc = Document::new("scale");
    let b = block(&mut doc, [0.0, 0.0, A, B], C);
    let v0 = volume(&brep(&doc, b));
    let f = add(&mut doc, FeatureBox::new(Scale::uniform(vec![b], "2")));
    ok(&doc, f);
    let sh = brep(&doc, b);
    valid(&sh);
    assert_rel(volume(&sh), 8.0 * v0, 1e-12);
    assert!((sh.bbox().max - Point3::new(80.0, 60.0, 40.0)).length() < 1e-9);
    golden("scale_uniform_2", &sh);

    let mut doc = Document::new("ellipsoid");
    let s = shape_body(&mut doc, Some(10.0));
    let vs = volume(&brep(&doc, s));
    let ws = 4.0 / 3.0 * std::f64::consts::PI * 1000.0;
    println!("sphere volume {vs} rel {:e}", (vs - ws).abs() / ws);
    let f = add(
        &mut doc,
        FeatureBox::new(Scale::non_uniform(vec![s], ["1", "2", "3"])),
    );
    ok(&doc, f);
    let sh = brep(&doc, s);
    valid(&sh);
    let want = 4.0 / 3.0 * std::f64::consts::PI * 10.0 * 20.0 * 30.0;
    // The rational B-spline ellipsoid is exact; the default `mass_props` quadrature is good to
    // ~1e-9 on it, so the check integrates adaptively to 1e-14.
    let v = sh.volume_adaptive(1e-14).unwrap();
    println!(
        "ellipsoid volume {v} (mass_props {}), want {want} (rel {:e})",
        volume(&sh),
        (v - want).abs() / want
    );
    assert_rel(v, want, REL);
    let bb = sh.bbox();
    assert!((bb.max.z - 30.0).abs() < 1e-6 && (bb.max.y - 20.0).abs() < 1e-6);
}

#[test]
fn scale_about_a_vertex_keeps_it_fixed_and_refuses_bad_factors() {
    let mut doc = Document::new("about");
    let b = block(&mut doc, [0.0, 0.0, A, B], C);
    let v = pick(doc.evaluated(), b, TopoKind::Vertex, [40.0, 30.0, 20.0], 1);
    let mut s = Scale::uniform(vec![b], "0.5");
    s.point = RefInput::Ref(EntityRef::Topo(v));
    let f = add(&mut doc, FeatureBox::new(s));
    ok(&doc, f);
    let bb = brep(&doc, b).bbox();
    assert!((bb.max - Point3::new(40.0, 30.0, 20.0)).length() < 1e-9);
    assert!((bb.min - Point3::new(20.0, 15.0, 10.0)).length() < 1e-9);
    for bad in ["0", "-1"] {
        let f = add(&mut doc, FeatureBox::new(Scale::uniform(vec![b], bad)));
        assert!(matches!(health(&doc, f), Health::Error(_)), "{bad}");
    }
}

// ---- Done-when 6: thicken ---------------------------------------------------------------------

#[test]
fn dw6_thickening_a_bspline_sheet_by_1mm_is_valid_and_near_area_times_t() {
    let mut doc = Document::new("thicken");
    let sheet = shape_body(&mut doc, None);
    let area = kernel().mass_props(&brep(&doc, sheet)).unwrap().area;
    let face = topo_ref_at(
        &**doc.evaluated(),
        default_kernel(),
        sheet,
        TopoKind::Face,
        0,
    )
    .map(|mut r| {
        r.key = 1;
        r
    })
    .unwrap();
    let mut vols = Vec::new();
    for (side, name) in [
        (amg_features::solid::ThinSide::Side1, "side1"),
        (amg_features::solid::ThinSide::Side2, "side2"),
        (amg_features::solid::ThinSide::Center, "center"),
    ] {
        let mut t = Thicken::new_body(vec![face.clone()], "1 mm");
        t.side = side;
        let f = add(&mut doc, FeatureBox::new(t));
        ok(&doc, f);
        let s = brep(&doc, body0(f));
        valid(&s);
        assert_eq!(s.solids().len(), 1);
        let v = volume(&s);
        println!(
            "thicken {name}: volume {v}, area x t {area} (rel {:e})",
            (v - area).abs() / area
        );
        assert!((v - area).abs() <= 0.005 * area, "{name}: {v} vs {area}");
        vols.push(v);
        golden(&format!("thicken_bspline_{name}"), &s);
        // Names: the far face is `Offset#0`, the side walls `ThickenSide#0`.
        let naming = doc
            .evaluated()
            .body(body0(f))
            .unwrap()
            .naming
            .clone()
            .unwrap();
        let roles: Vec<String> = (0..s.count(SubKind::Face))
            .filter_map(|i| naming.name_of(TopoKind::Face, i))
            .map(|n| format!("{n:?}"))
            .collect();
        assert_eq!(
            roles.iter().filter(|r| r.contains("ThickenSide")).count(),
            4,
            "{roles:?}"
        );
        assert_eq!(
            roles.iter().filter(|r| r.contains("\"Offset\"")).count(),
            1,
            "{roles:?}"
        );
    }
    // The mean-curvature terms of the two sides cancel (Steiner): their mean and the centred
    // slab are area × t to second order.
    assert_rel(0.5 * (vols[0] + vols[1]), area, 1e-5);
    assert_rel(vols[2], area, 1e-5);
    // The sheet is kept.
    assert!(doc.evaluated().body(sheet).is_some());
}

#[test]
fn thicken_joins_a_box_face_outward() {
    let mut doc = Document::new("thicken join");
    let b = block(&mut doc, [0.0, 0.0, A, B], C);
    let top = pick(doc.evaluated(), b, TopoKind::Face, [20.0, 15.0, 20.0], 1);
    let mut t = Thicken::new_body(vec![top], "2 mm");
    t.target = OpTarget {
        op: BodyOp::Join,
        participants: vec![b],
    };
    let f = add(&mut doc, FeatureBox::new(t));
    ok(&doc, f);
    let s = brep(&doc, b);
    valid(&s);
    assert_rel(volume(&s), A * B * (C + 2.0), REL);
    assert_eq!(faces(&s), 6, "the slab is unified with the box");
}

// ---- Done-when 7: TNP-X3 and TNP-X6 -----------------------------------------------------------

#[test]
fn dw7_tnp_x3_fillet_on_the_inner_floor_edge_survives_a_box_height_edit() {
    let mut doc = Document::new("tnp-x3");
    let (sk, ext) = extrude_rects(&mut doc, &[[0.0, 0.0, A, B]], C, BodyOp::NewBody, vec![]);
    let b = body0(ext);
    let top = pick(doc.evaluated(), b, TopoKind::Face, [20.0, 15.0, 20.0], 1);
    let sh = add(&mut doc, FeatureBox::new(Shell::inside(vec![top], "1 mm")));
    ok(&doc, sh);
    // The inner floor edge along the x = 1 wall: (1, 15, 1), length 28.
    let e = pick(doc.evaluated(), b, TopoKind::Edge, [1.0, 15.0, 1.0], 1);
    let fil = add(
        &mut doc,
        FeatureBox::new(Fillet::constant(vec![e], "0.5 mm")),
    );
    ok(&doc, fil);
    let (q, fp) = first_resolution(&doc, fil);
    assert_eq!(q, MatchQuality::Exact);
    assert!((fp.centroid - Point3::new(1.0, 15.0, 1.0)).length() < 1e-6);
    // Box height 20 → 30.
    let p = profiles(&doc, sk);
    edit(
        &mut doc,
        ext,
        FeatureBox::new(Extrude::distance(p, "30 mm", OpTarget::default())),
    );
    ok(&doc, sh);
    ok(&doc, fil);
    let (q, fp) = first_resolution(&doc, fil);
    assert_eq!(q, MatchQuality::Exact, "TNP-X3 expects Ok Exact");
    assert!(
        (fp.centroid - Point3::new(1.0, 15.0, 1.0)).length() < 1e-6,
        "{fp:?}"
    );
    assert!((fp.measure - 28.0).abs() < 1e-6, "{fp:?}");
    // The inner faces are `ShellInner` from the outer faces' names (Topo).
    let naming = doc.evaluated().body(b).unwrap().naming.clone().unwrap();
    let inner = (0..brep(&doc, b).count(SubKind::Face))
        .filter_map(|i| naming.name_of(TopoKind::Face, i))
        .filter(|n| format!("{n:?}").contains("ShellInner"))
        .count();
    assert_eq!(inner, 5, "5 inner walls");
    let rim = (0..brep(&doc, b).count(SubKind::Face))
        .filter_map(|i| naming.name_of(TopoKind::Face, i))
        .filter(|n| format!("{n:?}").contains("ShellRim"))
        .count();
    assert_eq!(rim, 1, "one rim");
    valid(&brep(&doc, b));
}

#[test]
fn dw7_tnp_x6_fillet_on_a_drafted_vertical_edge_survives_an_angle_edit() {
    let mut doc = Document::new("tnp-x6");
    let b = block(&mut doc, [0.0, 0.0, A, B], C);
    let sides = side_faces(&doc, b);
    let draft = add(
        &mut doc,
        FeatureBox::new(Draft::one_side(
            RefInput::Origin(OriginEntity::XYPlane),
            sides.clone(),
            "3 deg",
        )),
    );
    ok(&doc, draft);
    // The vertical edge at the (0, 0) corner, leaning inward.
    let e = pick(doc.evaluated(), b, TopoKind::Edge, [0.5, 0.5, 10.0], 1);
    let fil = add(&mut doc, FeatureBox::new(Fillet::constant(vec![e], "2 mm")));
    ok(&doc, fil);
    assert_eq!(first_resolution(&doc, fil).0, MatchQuality::Exact);
    // Draft angle 3° → 5°.
    edit(
        &mut doc,
        draft,
        FeatureBox::new(Draft::one_side(
            RefInput::Origin(OriginEntity::XYPlane),
            sides,
            "5 deg",
        )),
    );
    ok(&doc, draft);
    ok(&doc, fil);
    let (q, fp) = first_resolution(&doc, fil);
    assert_eq!(q, MatchQuality::Exact, "TNP-X6 expects Ok Exact");
    // The corner edge runs from (0, 0, 0) to (t, t, 20)·(tan 5°) at the top.
    let t = 20.0 * 5f64.to_radians().tan();
    assert!(
        (fp.centroid - Point3::new(t / 2.0, t / 2.0, 10.0)).length() < 1e-6,
        "{fp:?}"
    );
    valid(&brep(&doc, b));
}

// ---- tangent_chain_faces ----------------------------------------------------------------------

#[test]
fn tangent_chain_faces_is_transitive_and_blend_faces_stops_at_planes() {
    let mut doc = Document::new("chain faces");
    let b = filleted_box(&mut doc);
    let s = brep(&doc, b);
    let top = FaceId(
        doc.evaluated()
            .resolve(&pick(
                doc.evaluated(),
                b,
                TopoKind::Face,
                [20.0, 15.0, 20.0],
                1,
            ))
            .unwrap()
            .index,
    );
    let k = kernel();
    let all = amg_features::solid::tangent_chain_faces(k, &s, &[top], 1e-3).unwrap();
    assert_eq!(all.len(), 9, "top, 4 fillets and the 4 sides");
    assert_eq!(all[0], top);
    let blends = amg_features::solid::tangent_blend_faces(k, &s, &[top], 1e-3).unwrap();
    assert_eq!(blends.len(), 5, "top and the fillet ring");
    let mut d = Document::new("plain");
    let pb = block(&mut d, [0.0, 0.0, 10.0, 10.0], 10.0);
    let plain = brep(&d, pb);
    assert_eq!(
        amg_features::solid::tangent_chain_faces(k, &plain, &[FaceId(0)], 1e-3)
            .unwrap()
            .len(),
        1
    );
}
