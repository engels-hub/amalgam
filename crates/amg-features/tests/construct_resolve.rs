//! CON-03: reference resolution. Sketch, construction, mesh face-group and occurrence-path
//! references run everywhere; the B-rep part (S04 `solid.extrude`) needs the `occt` feature
//! (AMG_OCCT_DIR).

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "tests"
)]

// Shared with the SOL-01 tests; this binary doesn't use their goldens.
#[allow(unused_imports, reason = "shared test module")]
mod solid_common;

use std::f64::consts::{FRAC_PI_2, TAU};
use std::sync::Arc;

use amg_core::{Plane, Point2, Point3, Transform, Vec3};
use amg_doc::naming::{self, NameSeg, ProfileFp, Role, TopoKind, TopoName, TopoRef};
use amg_doc::stub::SketchEntityKey;
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, BodyNaming, Command, ComponentId,
    ConstructionGeom, ConstructionId, CreatedId, Dependency, Diagnostic, Document, EntityRef, Expr,
    ExprSrc, Feature, FeatureBox, FeatureError, FeatureId, FeatureOutput, Health, OccurrenceId,
    RecomputeCtx, ResolveError, SketchId, register_feature,
};
use amg_features::construct::resolve::{InspectResolveError, resolve_in, resolve_in_at};
use amg_features::construct::{
    AxisDef, ConstructAxis, ConstructPlane, ConstructPoint, OriginEntity, PathPosition, PlaneDef,
    PointDef, RefGeom, RefInput, ResolvedRef, math,
};
use amg_features::sketch::{PlaneInput, SketchFeature};
use amg_mesh::{FaceGroupInfo, FaceId, GroupOrigin, IndexedMesh};
use amg_sketch::EntityId;
use serde::{Deserialize, Serialize};
use solid_common::*;

const EPS: f64 = 1e-9;

fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

#[track_caller]
fn close(a: Point3, b: Point3) {
    assert!(a.distance(b) < EPS, "{a} vs {b}");
}

fn expr(s: &str) -> Expr {
    Expr::Inline(ExprSrc(s.to_owned()))
}

fn doc(name: &str) -> Document {
    amg_features::construct::link();
    amg_features::sketch::link();
    Document::new(name)
}

fn resolve(doc: &Document, r: &RefInput) -> Result<ResolvedRef, InspectResolveError> {
    resolve_in(doc.evaluated(), doc.design(), r)
}

fn of(doc: &Document, r: &EntityRef) -> ResolvedRef {
    resolve(doc, &RefInput::Ref(r.clone())).unwrap()
}

fn plane_of(doc: &Document, id: FeatureId) -> Plane {
    match doc.evaluated().construction(ConstructionId(id)) {
        Some(ConstructionGeom::Plane(p)) => *p,
        other => panic!("no plane: {other:?} ({:?})", health(doc, id)),
    }
}

#[cfg(feature = "occt")]
fn axis_of(doc: &Document, id: FeatureId) -> amg_core::Line3 {
    match doc.evaluated().construction(ConstructionId(id)) {
        Some(ConstructionGeom::Axis(l)) => *l,
        other => panic!("no axis: {other:?} ({:?})", health(doc, id)),
    }
}

#[track_caller]
fn assert_ok(doc: &Document, id: FeatureId) {
    assert_eq!(health(doc, id), Health::Ok);
}

#[track_caller]
fn error(doc: &Document, id: FeatureId) -> Diagnostic {
    match health(doc, id) {
        Health::Error(d) => d,
        other => panic!("expected an error, got {other:?}"),
    }
}

fn offset(base: EntityRef, distance: &str) -> FeatureBox {
    FeatureBox::new(ConstructPlane {
        def: PlaneDef::Offset {
            base: RefInput::Ref(base),
            distance: expr(distance),
        },
    })
}

// ---- sketches -------------------------------------------------------------------------------

struct Sketched {
    id: FeatureId,
    feature: SketchFeature,
    line: EntityId,
    circle: EntityId,
    arc: EntityId,
    ellipse: EntityId,
    spline: EntityId,
    points: [EntityId; 3],
}

impl Sketched {
    fn curve(&self, e: EntityId) -> EntityRef {
        EntityRef::SketchCurve {
            sketch: SketchId(self.id),
            curve: SketchEntityKey(u64::from(e.0)),
        }
    }
    fn point(&self, e: EntityId) -> EntityRef {
        EntityRef::SketchPoint {
            sketch: SketchId(self.id),
            point: SketchEntityKey(u64::from(e.0)),
        }
    }
}

/// One of each curve kind on a plane 7 mm above XY. The three free points are collinear.
fn sketched(doc: &mut Document) -> Sketched {
    let base = add(
        doc,
        FeatureBox::new(ConstructPlane {
            def: PlaneDef::Offset {
                base: RefInput::Origin(OriginEntity::XYPlane),
                distance: expr("7 mm"),
            },
        }),
    );
    let mut f = SketchFeature::new(PlaneInput::Ref(EntityRef::Construction(ConstructionId(
        base,
    ))));
    let p2 = Point2::new;
    let (line, circle, arc, ellipse, spline, points) = f
        .edit_sketch(|s| {
            let line = s.add_line(p2(1.0, 2.0), p2(11.0, 2.0))?;
            let circle = s.add_circle(p2(20.0, 5.0), 3.0)?;
            // A quarter arc from +X to +Y about (0, 20).
            let arc = s.add_arc(p2(0.0, 20.0), p2(4.0, 20.0), p2(0.0, 24.0))?;
            let ellipse = s.add_ellipse(p2(40.0, 0.0), p2(46.0, 0.0), 2.0)?;
            let spline = s.add_spline_fit(
                &[p2(0.0, 40.0), p2(3.0, 45.0), p2(8.0, 38.0), p2(12.0, 42.0)],
                None,
                None,
            )?;
            let points = [
                s.add_point(p2(0.0, 60.0))?,
                s.add_point(p2(5.0, 60.0))?,
                s.add_point(p2(10.0, 60.0))?,
            ];
            Ok((line, circle, arc, ellipse, spline, points))
        })
        .unwrap();
    let id = add(doc, FeatureBox::new(f.clone()));
    assert_ok(doc, id);
    Sketched {
        id,
        feature: f,
        line,
        circle,
        arc,
        ellipse,
        spline,
        points,
    }
}

#[test]
fn a_sketch_line_resolves_to_a_line_with_the_sketch_plane_as_context() {
    let mut doc = doc("sketch-line");
    let s = sketched(&mut doc);
    let plane = doc.evaluated().sketch(SketchId(s.id)).unwrap().plane;
    assert!((plane.origin.z - 7.0).abs() < EPS);

    let r = of(&doc, &s.curve(s.line));
    // A sketch line is the straight, directed form of a line (`RefGeom::Segment`).
    let RefGeom::Segment(seg) = &r.geom else {
        panic!("not straight: {}", r.geom.kind_name());
    };
    close(seg.a, p(1.0, 2.0, 7.0));
    close(seg.b, p(11.0, 2.0, 7.0));
    let line = math::line_of(&r.geom, &doc.design().meta().tolerance).unwrap();
    close(line.point, p(1.0, 2.0, 7.0));
    close(line.dir.get(), Vec3::X);
    assert_eq!(r.context_plane, Some(plane));
    assert_eq!(r.quality, None);

    // The context plane is the zero plane of "plane at angle" when none is picked.
    let at_angle = add(
        &mut doc,
        FeatureBox::new(ConstructPlane {
            def: PlaneDef::AtAngle {
                line: RefInput::Ref(s.curve(s.line)),
                zero: None,
                angle: expr("90 deg"),
            },
        }),
    );
    assert_ok(&doc, at_angle);
    close(plane_of(&doc, at_angle).normal.get(), Vec3::NEG_Y);
}

#[test]
fn sketch_curves_and_points_map_to_world_space() {
    let mut doc = doc("sketch-curves");
    let s = sketched(&mut doc);

    let r = of(&doc, &s.curve(s.circle));
    let RefGeom::Circle {
        center,
        normal,
        radius,
        arc: None,
    } = r.geom
    else {
        panic!("not a full circle");
    };
    close(center, p(20.0, 5.0, 7.0));
    close(normal.get(), Vec3::Z);
    assert_eq!(radius, 3.0);
    assert!(r.context_plane.is_some());

    let r = of(&doc, &s.curve(s.arc));
    let RefGeom::Circle {
        center,
        normal,
        radius,
        arc: Some((a, b)),
    } = r.geom
    else {
        panic!("not an arc");
    };
    assert!((radius - 4.0).abs() < EPS);
    assert!((b - a - FRAC_PI_2).abs() < EPS);
    let frame = Plane::from_normal(center, normal);
    let at = |t: f64| frame.to_world(Point2::new(radius * t.cos(), radius * t.sin()));
    close(at(a), p(4.0, 20.0, 7.0));
    close(at(b), p(0.0, 24.0, 7.0));

    for (i, x) in [0.0, 5.0, 10.0].into_iter().enumerate() {
        let r = of(&doc, &s.point(s.points[i]));
        let RefGeom::Point(q) = r.geom else {
            panic!("not a point");
        };
        close(q, p(x, 60.0, 7.0));
        assert!(r.context_plane.is_some());
    }

    // Ellipse and spline keep their exact geometry behind `CurveEval`.
    let r = of(&doc, &s.curve(s.ellipse));
    let RefGeom::Curve(c) = &r.geom else {
        panic!("not a curve");
    };
    let (t0, t1) = c.range();
    assert_eq!((t0, t1), (0.0, TAU));
    close(c.eval(0.0), p(46.0, 0.0, 7.0));
    close(c.eval(FRAC_PI_2), p(40.0, 2.0, 7.0));
    // Perimeter of the 6 × 2 ellipse (Gauss-AGM formula).
    assert!(
        (c.length() - 26.729_786_441_110_5).abs() < 1e-9,
        "{}",
        c.length()
    );
    assert!((c.param_at_length(c.length() / 4.0) - FRAC_PI_2).abs() < EPS);
    assert!((c.closest_param(p(40.0, 9.0, 7.0)) - FRAC_PI_2).abs() < EPS);

    let r = of(&doc, &s.curve(s.spline));
    let RefGeom::Curve(c) = &r.geom else {
        panic!("not a curve");
    };
    let (t0, t1) = c.range();
    close(c.eval(t0), p(0.0, 40.0, 7.0));
    close(c.eval(t1), p(12.0, 42.0, 7.0));
    for i in 1..10 {
        let t = t0 + (t1 - t0) * f64::from(i) / 10.0;
        let h = 1e-5;
        let d1 = (c.eval(t + h) - c.eval(t - h)) / (2.0 * h);
        let d2 = (c.d1(t + h) - c.d1(t - h)) / (2.0 * h);
        assert!((c.d1(t) - d1).length() < 1e-5 * d1.length().max(1.0));
        assert!((c.d2(t) - d2).length() < 1e-3 * d2.length().max(1.0));
        // Arc length round trip against a fine polyline.
        let n = 20_000;
        let polyline: f64 = (0..n)
            .map(|k| {
                let u = |k: i32| t0 + (t - t0) * f64::from(k) / f64::from(n);
                c.eval(u(k)).distance(c.eval(u(k + 1)))
            })
            .sum();
        assert!((c.param_at_length(polyline) - t).abs() < 1e-5);
        assert!(c.eval(c.closest_param(c.eval(t))).distance(c.eval(t)) < EPS);
    }

    // A point halfway along the spline's length through the feature.
    let mid = add(
        &mut doc,
        FeatureBox::new(ConstructPoint {
            def: PointDef::AlongPath {
                path: RefInput::Ref(s.curve(s.spline)),
                at: PathPosition::Fraction(expr("0.5")),
            },
        }),
    );
    assert_ok(&doc, mid);
    let want = c.eval(c.param_at_length(c.length() / 2.0));
    assert_eq!(
        doc.evaluated().construction(ConstructionId(mid)),
        Some(&ConstructionGeom::Point(want))
    );
}

#[test]
fn a_deleted_sketch_curve_is_an_error_naming_it() {
    let mut doc = doc("sketch-delete");
    let s = sketched(&mut doc);
    let line = s.curve(s.line);
    let axis = add(
        &mut doc,
        FeatureBox::new(ConstructAxis {
            def: AxisDef::ThroughEdge {
                edge: RefInput::Ref(line.clone()),
            },
        }),
    );
    assert_ok(&doc, axis);

    let mut f = s.feature.clone();
    f.edit_sketch(|k| k.remove_entities(&[s.line]).map(|_| ()))
        .unwrap();
    edit(&mut doc, s.id, FeatureBox::new(f));
    assert_eq!(error(&doc, axis).entities, vec![line.clone()]);
    assert!(matches!(
        resolve(&doc, &RefInput::Ref(line)),
        Err(InspectResolveError::Missing(_))
    ));
}

#[test]
fn degenerate_inputs_highlight_every_reference() {
    let mut doc = doc("degenerate");
    let s = sketched(&mut doc);
    let refs = s.points.map(|e| s.point(e));
    let [a, b, c] = refs.clone().map(RefInput::Ref);
    let plane = add(
        &mut doc,
        FeatureBox::new(ConstructPlane {
            def: PlaneDef::ThreePoints { a, b, c },
        }),
    );
    let d = error(&doc, plane);
    assert!(d.message.contains("collinear"), "{}", d.message);
    assert_eq!(d.entities, refs.to_vec());
}

#[test]
fn construction_geometry_origin_entities_and_unsupported_kinds_resolve_in_tools() {
    let mut doc = doc("tools");
    let s = sketched(&mut doc);
    let plane = add(
        &mut doc,
        FeatureBox::new(ConstructPlane {
            def: PlaneDef::Offset {
                base: RefInput::Origin(OriginEntity::YZPlane),
                distance: expr("3 mm"),
            },
        }),
    );
    let r = of(&doc, &EntityRef::Construction(ConstructionId(plane)));
    let RefGeom::Plane(got) = r.geom else {
        panic!("not a plane");
    };
    assert_eq!(got, plane_of(&doc, plane));
    let r = resolve(&doc, &RefInput::Origin(OriginEntity::ZAxis)).unwrap();
    assert!(matches!(r.geom, RefGeom::Line(l) if l.dir.get() == Vec3::Z));

    for unsupported in [
        EntityRef::Body(body0(s.id)),
        EntityRef::Profile {
            sketch: SketchId(s.id),
            profile: amg_doc::stub::ProfileKey(0),
            fingerprint: None::<ProfileFp>,
        },
    ] {
        assert_eq!(
            resolve(&doc, &RefInput::Ref(unsupported)).unwrap_err(),
            InspectResolveError::Unsupported
        );
    }
    assert!(matches!(
        resolve(
            &doc,
            &RefInput::Ref(EntityRef::Construction(ConstructionId(FeatureId(999))))
        ),
        Err(InspectResolveError::Missing(_))
    ));
}

// ---- mesh face groups and occurrences -------------------------------------------------------

/// Creates a named mesh body (stands in for an importer).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Con03Mesh {
    mesh: IndexedMesh,
}

impl Feature for Con03Mesh {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let naming = BodyNaming::for_mesh(ctx.feature_id(), &self.mesh.groups, None);
        Ok(FeatureOutput {
            bodies: vec![BodyChange {
                id: ctx.new_body_id(0),
                change: BodyChangeKind::Created {
                    geom: BodyGeom::Mesh(Arc::new(self.mesh.clone())),
                    naming: Some(naming),
                },
            }],
            ..FeatureOutput::default()
        })
    }
}

register_feature!(Con03Mesh, "test.con03_mesh", version = 1);

/// The box `[0, 10] × [0, 20] × [0, 30]` with one face group per side (z−, z+, y−, y+, x−,
/// x+), plus the id of a group that spans two sides.
fn grouped_box() -> (IndexedMesh, [u32; 6]) {
    let corner = |i: u32| {
        p(
            f64::from(i & 1) * 10.0,
            f64::from((i >> 1) & 1) * 20.0,
            f64::from((i >> 2) & 1) * 30.0,
        )
    };
    let quads = [
        [0, 2, 3, 1],
        [4, 5, 7, 6],
        [0, 1, 5, 4],
        [2, 6, 7, 3],
        [0, 4, 6, 2],
        [1, 3, 7, 5],
    ];
    let tris = quads
        .iter()
        .flat_map(|&[a, b, c, d]| [[a, b, c], [a, c, d]])
        .collect();
    let mut mesh = IndexedMesh::new((0..8).map(corner).collect(), tris).unwrap();
    let ids = std::array::from_fn(|side| {
        let g = mesh.groups.alloc(FaceGroupInfo::new(GroupOrigin::User));
        let first = u32::try_from(side).unwrap() * 2;
        mesh.groups.set(FaceId(first), g);
        mesh.groups.set(FaceId(first + 1), g);
        g.0
    });
    (mesh, ids)
}

fn add_in(doc: &mut Document, f: FeatureBox, context: ComponentId) -> FeatureId {
    let o = doc
        .apply(Command::AddFeature {
            at: None,
            feature: f,
            context,
            name: None,
        })
        .unwrap();
    doc.recompute_blocking(jobs()).unwrap();
    created_feature(&o)
}

fn group_ref(doc: &Document, body: BodyId, group: u32) -> TopoRef {
    naming::topo_ref_at(
        &**doc.evaluated(),
        naming::default_kernel(),
        body,
        TopoKind::FaceGroup,
        group,
    )
    .expect("the group is named")
}

/// `r` with a name nothing has, so that only its fingerprint can match.
fn renamed(r: &TopoRef) -> TopoRef {
    let name = TopoName::new(vec![NameSeg::Gen {
        feature: FeatureId(9999),
        role: Role::new("Gone", 0),
        from: None,
    }]);
    TopoRef { name, ..r.clone() }
}

#[test]
fn a_planar_mesh_face_group_resolves_to_its_plane() {
    let mut doc = doc("mesh");
    let (mesh, ids) = grouped_box();
    let body = body0(add(&mut doc, FeatureBox::new(Con03Mesh { mesh })));
    // (outward normal, a point of the side)
    let sides = [
        (Vec3::NEG_Z, p(0.0, 0.0, 0.0)),
        (Vec3::Z, p(0.0, 0.0, 30.0)),
        (Vec3::NEG_Y, p(0.0, 0.0, 0.0)),
        (Vec3::Y, p(0.0, 20.0, 0.0)),
        (Vec3::NEG_X, p(0.0, 0.0, 0.0)),
        (Vec3::X, p(10.0, 0.0, 0.0)),
    ];
    for (group, (normal, on)) in ids.into_iter().zip(sides) {
        let r = of(&doc, &EntityRef::Topo(group_ref(&doc, body, group)));
        let RefGeom::Plane(plane) = r.geom else {
            panic!("not a plane");
        };
        close(plane.normal.get(), normal);
        assert!(plane.signed_distance(on).abs() < EPS);
        assert!(r.quality.is_some());
    }

    // An offset plane on the top group.
    let top = EntityRef::Topo(group_ref(&doc, body, ids[1]));
    let plane = add(&mut doc, offset(top, "5 mm"));
    assert_ok(&doc, plane);
    assert!((plane_of(&doc, plane).origin.z - 35.0).abs() < EPS);
}

#[test]
fn a_curved_face_group_is_an_error_naming_it() {
    let mut doc = doc("mesh-curved");
    let (mut mesh, ids) = grouped_box();
    // Merge the y− side into the top group.
    let top = amg_mesh::FaceGroupId(ids[1]);
    mesh.groups.set(FaceId(4), top);
    mesh.groups.set(FaceId(5), top);
    let body = body0(add(&mut doc, FeatureBox::new(Con03Mesh { mesh })));
    let r = EntityRef::Topo(group_ref(&doc, body, ids[1]));
    let plane = add(&mut doc, offset(r.clone(), "1 mm"));
    let d = error(&doc, plane);
    assert!(d.message.contains("not planar"), "{}", d.message);
    assert_eq!(d.entities, vec![r]);
}

#[test]
fn a_fingerprint_only_match_is_one_warning_and_a_lost_reference_an_error() {
    let mut doc = doc("health");
    let (mesh, ids) = grouped_box();
    let body = body0(add(&mut doc, FeatureBox::new(Con03Mesh { mesh })));
    let good = group_ref(&doc, body, ids[1]);

    let moved = EntityRef::Topo(renamed(&good));
    let warned = add(&mut doc, offset(moved.clone(), "5 mm"));
    let Health::Warning(w) = health(&doc, warned) else {
        panic!("expected a warning, got {:?}", health(&doc, warned));
    };
    assert_eq!(w.len(), 1, "{w:?}");
    assert_eq!(w[0].entities, vec![moved.clone()]);
    assert!((plane_of(&doc, warned).origin.z - 35.0).abs() < EPS);
    assert_eq!(
        of(&doc, &moved).quality,
        Some(amg_doc::MatchQuality::FingerprintOnly)
    );

    // Neither a name nor a fingerprint matches.
    let lost = EntityRef::Topo(TopoRef {
        fingerprint: None,
        ..renamed(&good)
    });
    let failed = add(&mut doc, offset(lost.clone(), "5 mm"));
    assert_eq!(error(&doc, failed).entities, vec![lost.clone()]);
    assert!(matches!(
        resolve(&doc, &RefInput::Ref(lost)),
        Err(InspectResolveError::Resolve(ResolveError::NotFound { .. }))
    ));
}

fn new_component(doc: &mut Document, transform: Transform) -> (ComponentId, OccurrenceId) {
    let o = doc
        .apply(Command::NewComponent {
            parent: doc.design().root(),
            name: "Part".to_owned(),
            transform,
        })
        .unwrap();
    let component = o
        .created
        .iter()
        .find_map(|c| match c {
            CreatedId::Component(c) => Some(*c),
            _ => None,
        })
        .unwrap();
    let occurrence = doc
        .design()
        .occurrence_paths(component)
        .first()
        .and_then(|path| path.last().copied())
        .unwrap();
    (component, occurrence)
}

#[test]
fn occurrence_paths_place_references_and_origin_entities() {
    let mut doc = doc("assembly");
    // A quarter turn about Z, then 100 mm along X.
    let turn = Transform::from_translation(Vec3::new(100.0, 0.0, 0.0))
        * Transform::from_rotation_z(FRAC_PI_2);
    let (child, occ) = new_component(&mut doc, turn);
    let (mesh, ids) = grouped_box();
    let body = body0(add_in(&mut doc, FeatureBox::new(Con03Mesh { mesh }), child));
    // The x+ side: normal +X at x = 10 in the component, +Y at y = 10 in the world.
    let side = EntityRef::Topo(group_ref(&doc, body, ids[5]));
    let inside = RefInput::Ref(side.clone());
    let through = RefInput::Ref(EntityRef::InContext {
        path: vec![occ],
        target: Box::new(side),
    });
    let eval = doc.evaluated();
    let design = doc.design();
    let plane = |r: Result<ResolvedRef, InspectResolveError>| match r.unwrap().geom {
        RefGeom::Plane(p) => p,
        other => panic!("not a plane: {}", other.kind_name()),
    };

    // Component space without a path; world space with one.
    let local = plane(resolve_in(eval, design, &inside));
    close(local.normal.get(), Vec3::X);
    for r in [
        resolve_in(eval, design, &through),
        resolve_in_at(eval, design, &[occ], &inside),
    ] {
        let world = plane(r);
        close(world.normal.get(), Vec3::Y);
        assert!(world.signed_distance(p(100.0, 10.0, 0.0)).abs() < EPS);
    }

    // Origin entities of the placed component.
    let yz = plane(resolve_in_at(
        eval,
        design,
        &[occ],
        &RefInput::Origin(OriginEntity::YZPlane),
    ));
    close(yz.origin, p(100.0, 0.0, 0.0));
    close(yz.normal.get(), Vec3::Y);
    let o = resolve_in_at(eval, design, &[occ], &RefInput::Origin(OriginEntity::Point)).unwrap();
    assert!(matches!(o.geom, RefGeom::Point(q) if q.distance(p(100.0, 0.0, 0.0)) < EPS));

    assert!(matches!(
        resolve_in_at(eval, design, &[OccurrenceId(FeatureId(777).0)], &inside),
        Err(InspectResolveError::Missing(_))
    ));

    // Inside a recompute in the root component, and following a moved occurrence.
    let feature = FeatureBox::new(ConstructPlane {
        def: PlaneDef::Offset {
            base: through,
            distance: expr("0 mm"),
        },
    });
    let id = add(&mut doc, feature);
    assert_ok(&doc, id);
    assert!(
        plane_of(&doc, id)
            .signed_distance(p(100.0, 10.0, 0.0))
            .abs()
            < EPS
    );
    doc.apply(Command::SetOccurrenceTransform {
        occ,
        transform: Transform::from_translation(Vec3::new(0.0, 0.0, 50.0)),
    })
    .unwrap();
    doc.recompute_blocking(jobs()).unwrap();
    assert_ok(&doc, id);
    let moved = plane_of(&doc, id);
    close(moved.normal.get(), Vec3::X);
    assert!(moved.signed_distance(p(10.0, 0.0, 0.0)).abs() < EPS);
}

// ---- the B-rep test part --------------------------------------------------------------------

#[cfg(feature = "occt")]
mod solid {
    use super::*;
    use amg_brep::geom::{Curve3, Surface};
    use amg_brep::{EdgeId, FaceId as BrepFace, Shape, VertexId};
    use amg_features::solid::{BodyOp, Extrude, OpTarget};

    const HOLE: (f64, f64, f64) = (15.0, 12.0, 6.0);

    struct Part {
        sketch: FeatureId,
        feature: SketchFeature,
        circle: EntityId,
        extrude: FeatureId,
        body: BodyId,
    }

    fn extrude_of(doc: &Document, sketch: FeatureId, height: &str) -> FeatureBox {
        let g = doc.evaluated().sketch(SketchId(sketch)).unwrap();
        // The rectangle region: with the hole while the circle exists, else the plain one.
        let region = g
            .profiles
            .profiles
            .iter()
            .max_by_key(|r| r.outer.edges.len())
            .unwrap();
        let profile = EntityRef::Profile {
            sketch: SketchId(sketch),
            profile: amg_doc::stub::ProfileKey(region.id.0),
            fingerprint: Some(ProfileFp(region.fingerprint.clone())),
        };
        FeatureBox::new(Extrude::distance(
            vec![profile],
            height,
            OpTarget {
                op: BodyOp::NewBody,
                participants: Vec::new(),
            },
        ))
    }

    /// A 40 × 30 × 10 box with a Ø12 through hole at (15, 12), from `solid.extrude`.
    fn part(doc: &mut Document) -> Part {
        let mut f = rect_sketch(&[[0.0, 0.0, 40.0, 30.0]]);
        let circle = f
            .edit_sketch(|s| s.add_circle(Point2::new(HOLE.0, HOLE.1), HOLE.2))
            .unwrap();
        let sketch = add(doc, FeatureBox::new(f.clone()));
        let feature = extrude_of(doc, sketch, "10 mm");
        let extrude = add(doc, feature);
        assert_ok(doc, extrude);
        Part {
            sketch,
            feature: f,
            circle,
            extrude,
            body: body0(extrude),
        }
    }

    fn topo(doc: &Document, body: BodyId, kind: TopoKind, index: u32) -> EntityRef {
        EntityRef::Topo(
            naming::topo_ref_at(&**doc.evaluated(), kernel(), body, kind, index)
                .expect("the sub-shape is named"),
        )
    }

    /// The face for which `pick` holds (exactly one).
    fn face(shape: &Shape, pick: impl Fn(&Shape, BrepFace) -> bool) -> u32 {
        let found: Vec<BrepFace> = shape.faces().filter(|&f| pick(shape, f)).collect();
        assert_eq!(found.len(), 1);
        found[0].0
    }

    fn corners(shape: &Shape, f: BrepFace) -> Vec<Point3> {
        shape
            .face_edges(f)
            .iter()
            .flat_map(|&e| shape.edge_vertices(e))
            .map(|v| shape.vertex_point(v))
            .collect()
    }

    /// The planar face whose vertices all satisfy `coordinate(v) == value`.
    fn side(shape: &Shape, axis: usize, value: f64) -> u32 {
        face(shape, |s, f| {
            matches!(s.surface(f), Surface::Plane(_))
                && corners(s, f).iter().all(|v| (v[axis] - value).abs() < EPS)
        })
    }

    fn hole(shape: &Shape) -> u32 {
        face(shape, |s, f| {
            matches!(s.surface(f), Surface::Cylinder { .. })
        })
    }

    #[test]
    fn planar_faces_resolve_to_their_planes_with_outward_normals() {
        let mut doc = doc("planes");
        let part = part(&mut doc);
        let shape = brep(&doc, part.body);
        for (axis, value, normal) in [
            (0, 0.0, Vec3::NEG_X),
            (0, 40.0, Vec3::X),
            (1, 0.0, Vec3::NEG_Y),
            (1, 30.0, Vec3::Y),
            (2, 0.0, Vec3::NEG_Z),
            (2, 10.0, Vec3::Z),
        ] {
            let f = side(&shape, axis, value);
            let r = of(&doc, &topo(&doc, part.body, TopoKind::Face, f));
            let RefGeom::Plane(plane) = r.geom else {
                panic!("not a plane");
            };
            close(plane.normal.get(), normal);
            for v in corners(&shape, BrepFace(f)) {
                assert!(plane.signed_distance(v).abs() < EPS);
            }
            assert_eq!(r.quality, Some(amg_doc::MatchQuality::Exact));
        }
    }

    #[test]
    fn the_hole_face_resolves_to_a_cylinder_with_the_exact_radius() {
        let mut doc = doc("hole");
        let part = part(&mut doc);
        let shape = brep(&doc, part.body);
        let face = topo(&doc, part.body, TopoKind::Face, hole(&shape));
        let r = of(&doc, &face);
        let RefGeom::Cylinder { axis, radius } = r.geom else {
            panic!("not a cylinder: {}", r.geom.kind_name());
        };
        assert_eq!(radius, HOLE.2);
        close(axis.dir.get().abs(), Vec3::Z);
        close(p(axis.point.x, axis.point.y, 0.0), p(HOLE.0, HOLE.1, 0.0));

        // The axis through the hole.
        let through = add(
            &mut doc,
            FeatureBox::new(ConstructAxis {
                def: AxisDef::ThroughRevolved {
                    face_or_edge: RefInput::Ref(face.clone()),
                },
            }),
        );
        assert_ok(&doc, through);
        close(axis_of(&doc, through).dir.get().abs(), Vec3::Z);

        // The hole's outward normal points towards its axis: from the origin's closest wall
        // point, back towards (15, 12).
        let origin = RefInput::Origin(OriginEntity::Point);
        let normal = add(
            &mut doc,
            FeatureBox::new(ConstructAxis {
                def: AxisDef::PerpendicularToFaceAtPoint {
                    face: RefInput::Ref(face.clone()),
                    point: origin.clone(),
                },
            }),
        );
        assert_ok(&doc, normal);
        let to_axis = Vec3::new(HOLE.0, HOLE.1, 0.0).normalize();
        close(axis_of(&doc, normal).dir.get(), to_axis);
        let tangent = add(
            &mut doc,
            FeatureBox::new(ConstructPlane {
                def: PlaneDef::TangentAtPoint {
                    face: RefInput::Ref(face),
                    point: origin,
                },
            }),
        );
        assert_ok(&doc, tangent);
        close(plane_of(&doc, tangent).normal.get(), to_axis);
    }

    #[test]
    fn a_boss_face_keeps_the_outward_normal_of_its_cylinder() {
        let mut doc = doc("boss");
        let mut f = SketchFeature::new(PlaneInput::OriginXY);
        f.edit_sketch(|s| s.add_circle(Point2::new(20.0, 0.0), 4.0))
            .unwrap();
        let sketch = add(&mut doc, FeatureBox::new(f));
        let feature = extrude_of(&doc, sketch, "5 mm");
        let extrude = add(&mut doc, feature);
        let body = body0(extrude);
        let face = topo(&doc, body, TopoKind::Face, hole(&brep(&doc, body)));
        let normal = add(
            &mut doc,
            FeatureBox::new(ConstructAxis {
                def: AxisDef::PerpendicularToFaceAtPoint {
                    face: RefInput::Ref(face),
                    point: RefInput::Origin(OriginEntity::Point),
                },
            }),
        );
        assert_ok(&doc, normal);
        close(axis_of(&doc, normal).dir.get(), Vec3::NEG_X);
    }

    #[test]
    fn circular_edges_straight_edges_and_vertices_resolve() {
        let mut doc = doc("edges");
        let part = part(&mut doc);
        let shape = brep(&doc, part.body);
        let (mut circles, mut lines) = (Vec::new(), 0);
        for e in shape.edges() {
            let r = of(&doc, &topo(&doc, part.body, TopoKind::Edge, e.0));
            let (curve, t0, t1) = shape.curve(e).unwrap();
            let [a, b] = shape.edge_vertices(e).map(|v| shape.vertex_point(v));
            // The first planar face next to the edge. Only the seam of the hole has none: the
            // cylinder is on both its sides.
            let seam = shape.edge_faces(e) == [BrepFace(hole(&shape))];
            assert_eq!(r.context_plane.is_none(), seam);
            match (curve, r.geom) {
                (
                    Curve3::Circle { .. },
                    RefGeom::Circle {
                        center,
                        normal,
                        radius,
                        arc,
                    },
                ) => {
                    assert_eq!(radius, HOLE.2);
                    close(p(center.x, center.y, 0.0), p(HOLE.0, HOLE.1, 0.0));
                    close(normal.get().abs(), Vec3::Z);
                    assert_eq!(arc, None);
                    // The adjacent planar face: the top or bottom of the box.
                    let context = r.context_plane.unwrap();
                    assert!(context.signed_distance(center).abs() < EPS);
                    close(context.normal.get().abs(), Vec3::Z);
                    circles.push((center.z, e));
                }
                (Curve3::Line { .. }, RefGeom::Segment(s)) => {
                    close(s.a, a);
                    close(s.b, b);
                    assert!((s.length() - (t1 - t0)).abs() < EPS);
                    if let Some(context) = r.context_plane {
                        assert!(context.signed_distance(s.a).abs() < EPS);
                        assert!(context.signed_distance(s.b).abs() < EPS);
                    }
                    lines += 1;
                }
                (curve, geom) => panic!("{curve:?} resolved to {}", geom.kind_name()),
            }
        }
        circles.sort_by(|a, b| a.0.total_cmp(&b.0));
        assert_eq!(circles.len(), 2);
        assert!(circles[0].0.abs() < EPS && (circles[1].0 - 10.0).abs() < EPS);
        assert!(lines >= 12);

        for v in 0..shape.count(amg_brep::SubKind::Vertex) {
            let r = of(&doc, &topo(&doc, part.body, TopoKind::Vertex, v));
            let RefGeom::Point(q) = r.geom else {
                panic!("not a point");
            };
            close(q, shape.vertex_point(VertexId(v)));
        }

        // The centre of the top circular edge, through the feature.
        let top: EdgeId = circles[1].1;
        let edge = topo(&doc, part.body, TopoKind::Edge, top.0);
        let center = add(
            &mut doc,
            FeatureBox::new(ConstructPoint {
                def: PointDef::AtCenter {
                    entity: RefInput::Ref(edge),
                },
            }),
        );
        assert_ok(&doc, center);
        let Some(ConstructionGeom::Point(c)) = doc.evaluated().construction(ConstructionId(center))
        else {
            panic!("no point");
        };
        close(*c, p(HOLE.0, HOLE.1, 10.0));
    }

    #[test]
    fn an_offset_plane_follows_an_upstream_extrude_edit() {
        let mut doc = doc("offset");
        let part = part(&mut doc);
        let top = side(&brep(&doc, part.body), 2, 10.0);
        let top = topo(&doc, part.body, TopoKind::Face, top);
        let plane = add(&mut doc, offset(top.clone(), "5 mm"));
        assert_ok(&doc, plane);
        let before = plane_of(&doc, plane);
        close(before.normal.get(), Vec3::Z);
        assert!((before.origin.z - 15.0).abs() < EPS);

        for (height, z) in [("25 mm", 30.0), ("3.5 mm", 8.5)] {
            let feature = extrude_of(&doc, part.sketch, height);
            edit(&mut doc, part.extrude, feature);
            assert_ok(&doc, plane);
            let after = plane_of(&doc, plane);
            close(after.normal.get(), Vec3::Z);
            assert!((after.origin.z - z).abs() < EPS, "{}", after.origin.z);
            let RefGeom::Plane(face) = of(&doc, &top).geom else {
                panic!("not a plane");
            };
            assert!((face.origin.z - (z - 5.0)).abs() < EPS);
        }
    }

    #[test]
    fn a_deleted_face_is_an_error_whose_diagnostic_names_the_reference() {
        let mut doc = doc("deleted");
        let part = part(&mut doc);
        let face = hole(&brep(&doc, part.body));
        let face = topo(&doc, part.body, TopoKind::Face, face);
        let axis = add(
            &mut doc,
            FeatureBox::new(ConstructAxis {
                def: AxisDef::ThroughRevolved {
                    face_or_edge: RefInput::Ref(face.clone()),
                },
            }),
        );
        assert_ok(&doc, axis);

        // Delete the circle, so the extrude makes a plain box.
        let mut f = part.feature.clone();
        f.edit_sketch(|s| s.remove_entities(&[part.circle]).map(|_| ()))
            .unwrap();
        edit(&mut doc, part.sketch, FeatureBox::new(f));
        let feature = extrude_of(&doc, part.sketch, "10 mm");
        edit(&mut doc, part.extrude, feature);
        assert_ok(&doc, part.extrude);
        assert_eq!(faces(&brep(&doc, part.body)), 6);

        let d = error(&doc, axis);
        assert_eq!(d.entities, vec![face.clone()], "{}", d.message);
        assert!(matches!(
            d.cause,
            Some(amg_doc::ErrorCause::Resolve(ResolveError::NotFound { .. }))
        ));
        assert!(doc.evaluated().construction(ConstructionId(axis)).is_none());
        assert!(matches!(
            resolve(&doc, &RefInput::Ref(face)),
            Err(InspectResolveError::Resolve(_))
        ));
    }

    #[test]
    fn a_reference_of_the_wrong_kind_is_highlighted() {
        let mut doc = doc("kind");
        let part = part(&mut doc);
        let shape = brep(&doc, part.body);
        let face = topo(&doc, part.body, TopoKind::Face, hole(&shape));
        // A cylinder is not a plane.
        let bad = add(&mut doc, offset(face.clone(), "1 mm"));
        let d = error(&doc, bad);
        assert!(d.message.contains("cylinder"), "{}", d.message);
        assert_eq!(d.entities, vec![face]);
    }
}
