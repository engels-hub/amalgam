//! BREP-05: the history framework applied to the BREP-03, BREP-14 and BREP-04 ops.
//!
//! Done-when 1 (completeness over the op corpora), 2 (prism of a tagged profile), 3 (a box cut
//! by a cylinder: split order, 10 runs), 4 (`compose` of prism + unify equals the end-to-end
//! history on 5 cases), 5 (serde round trip), 6 (`sub_geom_all` on a 1000-face part; the
//! < 20 ms assertion only with `AMG_STRICT_TIMING=1`), plus `sub_geom` values and
//! `OpResult::per_solid` (R-S04-2). The fixtures here (box, tagged prism, box cut by a
//! cylinder) are the ones S01 DOC-06 can build its naming tests on.
#![cfg(feature = "occt")]
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

#[path = "common/blend_corpus.rs"]
mod blend_corpus;
#[path = "common/bool_corpus.rs"]
mod bool_corpus;
#[path = "common/heal_corpus.rs"]
mod heal_corpus;
#[path = "common/history.rs"]
mod history_check;
#[path = "common/offset_corpus.rs"]
mod offset_corpus;
#[path = "common/parts.rs"]
mod parts;

use std::collections::BTreeMap;
use std::f64::consts::{FRAC_PI_2, PI, TAU};
use std::time::{Duration, Instant};

use amg_brep::geom::{
    Curve2d, Curve3, CurveSeg, GeomClass, Loop2, PlanarProfile, ProfileEdge2, Tag,
};
use amg_brep::{
    BoolOp, BoolOpts, BrepKernel, Dir3, EdgeId, FaceId, InputSub, JobCtx, KernelConfig, OcctKernel,
    OpResult, OpRole, Plane, Point2, Point3, PrismExtent, PrismOpts, Relation, Shape, ShapeHistory,
    SubKind, SubShape, SubShapeGeom, Tolerance, Transform, Validate, Vec2, Vec3, split_order_key,
};
use amg_core::{Line3, Mat3};

use bool_corpus::frame;
use history_check::{completeness_error, counts};

const TOL: Tolerance = Tolerance::DEFAULT;

fn kernel() -> OcctKernel {
    OcctKernel::new(KernelConfig {
        validate: Validate::Always,
        ..KernelConfig::default()
    })
}

fn ctx() -> JobCtx {
    JobCtx::detached()
}

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}

fn bin(s: &Shape) -> Vec<u8> {
    let mut bytes = Vec::new();
    s.write_bin(&mut bytes).unwrap();
    bytes
}

fn face_sub(index: u32) -> SubShape {
    SubShape {
        kind: SubKind::Face,
        index,
    }
}

/// A plane tilted off every axis, away from the origin.
fn tilted() -> Plane {
    frame(
        [3.0, -2.0, 5.0],
        Some([0.3, -0.5, 0.8]),
        Some([1.0, 0.2, 0.0]),
    )
}

// ---- profiles ------------------------------------------------------------------------------

fn line(a: (f64, f64), b: (f64, f64)) -> Curve2d {
    Curve2d::Line {
        a: Point2::new(a.0, a.1),
        b: Point2::new(b.0, b.1),
    }
}

fn polygon(points: &[(f64, f64)]) -> Vec<Curve2d> {
    (0..points.len())
        .map(|i| line(points[i], points[(i + 1) % points.len()]))
        .collect()
}

fn arc(center: (f64, f64), radius: f64, start_angle: f64, sweep: f64) -> Curve2d {
    Curve2d::Arc {
        center: Point2::new(center.0, center.1),
        radius,
        start_angle,
        sweep,
    }
}

/// A circle as `n` arcs, clockwise (a hole) or counter-clockwise.
fn arcs(center: (f64, f64), radius: f64, n: u32, clockwise: bool) -> Vec<Curve2d> {
    let sweep = if clockwise { -TAU } else { TAU } / f64::from(n);
    (0..n)
        .map(|i| arc(center, radius, f64::from(i) * sweep, sweep))
        .collect()
}

fn tagged(curves: Vec<Curve2d>, first_tag: u64) -> Loop2 {
    Loop2 {
        edges: curves
            .into_iter()
            .zip(first_tag..)
            .map(|(curve, t)| ProfileEdge2 { curve, tag: Tag(t) })
            .collect(),
    }
}

/// A planar face; outer edges are tagged 1000.., hole `h`'s edges `2000 + 100 h`.. Returns the
/// face and its `(edge, tag)` map.
fn profile(
    k: &OcctKernel,
    plane: Plane,
    outer: Vec<Curve2d>,
    holes: Vec<Vec<Curve2d>>,
) -> (Shape, Vec<(EdgeId, Tag)>) {
    let p = PlanarProfile {
        plane,
        outer: tagged(outer, 1000),
        holes: holes
            .into_iter()
            .zip(0u64..)
            .map(|(h, i)| tagged(h, 2000 + 100 * i))
            .collect(),
    };
    let built = k.make_planar_faces(&[p], &TOL).unwrap().remove(0);
    (built.face, built.edge_tags)
}

fn square_points(side: f64) -> Vec<(f64, f64)> {
    vec![(0.0, 0.0), (side, 0.0), (side, side), (0.0, side)]
}

fn ellipse_quarters(a: f64, b: f64) -> Vec<Curve2d> {
    (0..4)
        .map(|i| Curve2d::Ellipse {
            center: Point2::ZERO,
            major: Vec2::new(a, 0.0),
            minor_radius: b,
            t0: f64::from(i) * FRAC_PI_2,
            t1: f64::from(i + 1) * FRAC_PI_2,
        })
        .collect()
}

fn slot(len: f64, r: f64, split_right_arc: bool) -> Vec<Curve2d> {
    let mut out = vec![line((0.0, -r), (len, -r))];
    if split_right_arc {
        out.push(arc((len, 0.0), r, -FRAC_PI_2, FRAC_PI_2));
        out.push(arc((len, 0.0), r, 0.0, FRAC_PI_2));
    } else {
        out.push(arc((len, 0.0), r, -FRAC_PI_2, PI));
    }
    out.push(line((len, r), (0.0, r)));
    out.push(arc((0.0, 0.0), r, FRAC_PI_2, PI));
    out
}

fn opts(taper: f64, merge_coplanar: bool) -> PrismOpts {
    PrismOpts {
        taper,
        merge_coplanar,
    }
}

fn extrude(k: &OcctKernel, s: &Shape, v: Vec3, o: PrismOpts) -> OpResult {
    k.prism(s, &PrismExtent::Vector(v), &o, &ctx()).unwrap()
}

/// The XZ rectangle x in [5, 8], z in [0, 4] (a revol profile about Z).
fn xz_rectangle(k: &OcctKernel) -> Shape {
    let pts = [(5.0, 0.0), (8.0, 0.0), (8.0, 4.0), (5.0, 4.0)].map(|(x, z)| Point3::new(x, 0.0, z));
    k.make_planar_polygon(&pts, &[], &TOL).unwrap()
}

fn z_axis() -> Line3 {
    Line3 {
        point: Point3::ZERO,
        dir: Dir3::Z,
    }
}

fn line_seg(a: Point3, b: Point3, tag: u64) -> CurveSeg {
    let d = b - a;
    CurveSeg {
        curve: Curve3::Line {
            origin: a,
            dir: Dir3::new_unchecked(d.normalize()),
        },
        t0: 0.0,
        t1: d.length(),
        tag: Tag(tag),
    }
}

// ---- Done-when 1: completeness -------------------------------------------------------------

/// One op run: name, operands, result.
type Run = (String, Vec<Shape>, OpResult);

/// The BREP-03 and BREP-14 op corpus: `(name, operands, result)`.
fn op_corpus(k: &OcctKernel) -> Vec<Run> {
    let mut out: Vec<Run> = Vec::new();
    let mut add = |name: &str, operands: Vec<Shape>, r: OpResult| {
        out.push((name.to_owned(), operands, r));
    };
    let t = tilted();
    // BREP-03: wires (tags only).
    let pts = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(10.0, 0.0, 0.0),
        Point3::new(10.0, 5.0, 1.0),
        Point3::new(0.0, 5.0, 0.0),
    ];
    let open: Vec<CurveSeg> = (0..3)
        .map(|i| line_seg(pts[i], pts[i + 1], i as u64))
        .collect();
    add("wire_open", vec![], k.make_wire(&open, &TOL).unwrap());
    let closed: Vec<CurveSeg> = (0..4)
        .map(|i| line_seg(pts[i], pts[(i + 1) % 4], 10 + i as u64))
        .collect();
    add("wire_closed", vec![], k.make_wire(&closed, &TOL).unwrap());

    // BREP-14: primitives.
    add(
        "box",
        vec![],
        k.make_box(&t, Vec3::new(10.0, 20.0, 30.0)).unwrap(),
    );
    add(
        "cylinder",
        vec![],
        k.make_cylinder(&t, 5.0, 12.0, TAU).unwrap(),
    );
    add(
        "cylinder_90",
        vec![],
        k.make_cylinder(&t, 5.0, 12.0, FRAC_PI_2).unwrap(),
    );
    add(
        "cone_frustum",
        vec![],
        k.make_cone(&t, 5.0, 2.0, 10.0, TAU).unwrap(),
    );
    add(
        "cone_full",
        vec![],
        k.make_cone(&t, 5.0, 0.0, 10.0, TAU).unwrap(),
    );
    add(
        "cone_half",
        vec![],
        k.make_cone(&t, 5.0, 2.0, 10.0, PI).unwrap(),
    );
    add("sphere", vec![], k.make_sphere(&t, 7.0).unwrap());
    add("torus", vec![], k.make_torus(&t, 10.0, 3.0, TAU).unwrap());
    add(
        "torus_half",
        vec![],
        k.make_torus(&t, 10.0, 3.0, PI).unwrap(),
    );

    // BREP-14: prisms.
    let up = Vec3::new(0.0, 0.0, 10.0);
    let sq = profile(k, Plane::XY, polygon(&square_points(10.0)), vec![]).0;
    add(
        "prism_square",
        vec![sq.clone()],
        extrude(k, &sq, up, opts(0.0, false)),
    );
    let disc = profile(k, Plane::XY, arcs((0.0, 0.0), 5.0, 1, false), vec![]).0;
    add(
        "prism_circle",
        vec![disc.clone()],
        extrude(k, &disc, up, opts(0.0, false)),
    );
    let holed = profile(
        k,
        Plane::XY,
        polygon(&square_points(10.0)),
        vec![arcs((5.0, 5.0), 2.0, 1, true)],
    )
    .0;
    add(
        "prism_hole",
        vec![holed.clone()],
        extrude(k, &holed, up, opts(0.0, false)),
    );
    let ellipse = profile(k, t, ellipse_quarters(6.0, 3.0), vec![]).0;
    let along = 8.0 * t.normal.get();
    add(
        "prism_ellipse",
        vec![ellipse.clone()],
        extrude(k, &ellipse, along, opts(0.0, false)),
    );
    let wire = k
        .make_wire(
            &[
                line_seg(Point3::ZERO, Point3::new(10.0, 0.0, 0.0), 1),
                line_seg(Point3::new(10.0, 0.0, 0.0), Point3::new(10.0, 5.0, 0.0), 2),
            ],
            &TOL,
        )
        .unwrap()
        .shape;
    add(
        "prism_wire",
        vec![wire.clone()],
        extrude(k, &wire, Vec3::new(0.0, 0.0, 7.0), opts(0.0, false)),
    );
    let sl = profile(k, Plane::XY, slot(10.0, 2.0, false), vec![]).0;
    add(
        "prism_slot",
        vec![sl.clone()],
        extrude(k, &sl, up, opts(0.0, false)),
    );
    add(
        "prism_oblique",
        vec![sq.clone()],
        extrude(k, &sq, Vec3::new(3.0, -2.0, 10.0), opts(0.0, false)),
    );
    let split = profile(
        k,
        Plane::XY,
        polygon(&[
            (0.0, 0.0),
            (5.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0),
        ]),
        vec![],
    )
    .0;
    add(
        "prism_merge",
        vec![split.clone()],
        extrude(k, &split, up, opts(0.0, true)),
    );
    add(
        "prism_taper_out",
        vec![sq.clone()],
        extrude(k, &sq, up, opts(5f64.to_radians(), false)),
    );
    add(
        "prism_taper_in",
        vec![sq.clone()],
        extrude(k, &sq, up, opts(-5f64.to_radians(), false)),
    );
    add(
        "prism_taper_disc",
        vec![disc.clone()],
        extrude(k, &disc, up, opts(3f64.to_radians(), false)),
    );
    let one_way = PrismExtent::Infinite {
        dir: Dir3::Z,
        both: false,
    };
    add(
        "prism_semi_infinite",
        vec![sq.clone()],
        k.prism(&sq, &one_way, &opts(0.0, false), &ctx()).unwrap(),
    );
    let both = PrismExtent::Infinite {
        dir: Dir3::Z,
        both: true,
    };
    add(
        "prism_infinite",
        vec![sq.clone()],
        k.prism(&sq, &both, &opts(0.0, false), &ctx()).unwrap(),
    );

    // BREP-14: revols.
    let rect = xz_rectangle(k);
    add(
        "revol_360",
        vec![rect.clone()],
        k.revol(&rect, &z_axis(), TAU, &ctx()).unwrap(),
    );
    add(
        "revol_90",
        vec![rect.clone()],
        k.revol(&rect, &z_axis(), FRAC_PI_2, &ctx()).unwrap(),
    );
    let tri = k
        .make_planar_polygon(
            &[
                Point3::ZERO,
                Point3::new(4.0, 0.0, 0.0),
                Point3::new(0.0, 0.0, 6.0),
            ],
            &[],
            &TOL,
        )
        .unwrap();
    add(
        "revol_cone",
        vec![tri.clone()],
        k.revol(&tri, &z_axis(), TAU, &ctx()).unwrap(),
    );
    let rwire = k
        .make_wire(
            &[
                line_seg(Point3::new(5.0, 0.0, 0.0), Point3::new(8.0, 0.0, 0.0), 1),
                line_seg(Point3::new(8.0, 0.0, 0.0), Point3::new(8.0, 0.0, 4.0), 2),
            ],
            &TOL,
        )
        .unwrap()
        .shape;
    add(
        "revol_wire",
        vec![rwire.clone()],
        k.revol(&rwire, &z_axis(), PI, &ctx()).unwrap(),
    );

    // BREP-14: affine maps and mirror.
    let block = k.make_box(&t, Vec3::new(10.0, 20.0, 30.0)).unwrap().shape;
    let scale2 = Transform::from_mat3_translation(Mat3::from_diagonal(Vec3::splat(2.0)), Vec3::X);
    add(
        "gtransformed_uniform",
        vec![block.clone()],
        block.gtransformed(&scale2).unwrap(),
    );
    let ball = k.make_sphere(&Plane::XY, 10.0).unwrap().shape;
    let stretch =
        Transform::from_mat3_translation(Mat3::from_diagonal(Vec3::new(1.0, 2.0, 3.0)), Vec3::ZERO);
    add(
        "gtransformed_ellipsoid",
        vec![ball.clone()],
        ball.gtransformed(&stretch).unwrap(),
    );
    add(
        "mirrored",
        vec![block.clone()],
        k.mirrored(&block, &t).unwrap(),
    );
    out
}

#[test]
fn every_output_face_has_a_source_or_a_role() {
    let k = kernel();
    let mut failures = Vec::new();
    let mut faces = 0;
    // BREP-03 face builders return `FaceBuild`: their edge tags must name edges of the face.
    for (name, outer, holes) in [
        ("square", polygon(&square_points(10.0)), vec![]),
        (
            "holed",
            polygon(&square_points(10.0)),
            vec![arcs((5.0, 5.0), 2.0, 3, true)],
        ),
        ("slot", slot(10.0, 2.0, true), vec![]),
    ] {
        let (face, tags) = profile(&k, tilted(), outer, holes);
        if tags.iter().any(|(e, _)| e.0 >= face.count(SubKind::Edge)) {
            failures.push(format!("{name}: an edge tag is out of range"));
        }
    }
    let corpus = op_corpus(&k);
    for (name, operands, r) in &corpus {
        let operands: Vec<&Shape> = operands.iter().collect();
        if let Some(e) = completeness_error(r, &operands) {
            failures.push(format!("{name}: {e}"));
        }
        faces += r.shape.count(SubKind::Face);
    }
    let mut bool_ok = 0;
    if let Some(cases) = bool_corpus::corpus() {
        for case in &cases {
            let (objects, tools) = case.operands(&k);
            let Ok(r) = case.run(&k, &objects, &tools, None) else {
                continue;
            };
            let operands: Vec<&Shape> = objects.iter().chain(&tools).collect();
            if let Some(e) = completeness_error(&r, &operands) {
                failures.push(format!("{}: {e}", case.name));
            }
            faces += r.shape.count(SubKind::Face);
            bool_ok += 1;
        }
        assert!(bool_ok >= 130, "{bool_ok} boolean results");
    }
    // BREP-06: every fillet and chamfer of its corpus.
    let blends = blend_corpus::corpus(&k);
    for case in &blends {
        let r = case.run(&k).unwrap();
        if let Some(e) = completeness_error(&r, &[&case.input]) {
            failures.push(format!("{}: {e}", case.name));
        }
        faces += r.shape.count(SubKind::Face);
    }
    // BREP-07: every shell, offset and thicken of its corpus.
    let offsets = offset_corpus::corpus(&k);
    for case in &offsets {
        let r = case.run(&k).unwrap();
        if let Some(e) = completeness_error(&r, &[&case.input]) {
            failures.push(format!("{}: {e}", case.name));
        }
        faces += r.shape.count(SubKind::Face);
    }
    // BREP-10: heal (the defect corpus), unify_same_domain, sew and make_solid.
    let heals = heal_corpus::corpus(&k);
    let mut heal_ok = 0;
    for case in &heals {
        let Ok(r) = case.run(&k) else {
            continue;
        };
        let operands: Vec<&Shape> = case.inputs.iter().collect();
        if let Some(e) = completeness_error(&r, &operands) {
            failures.push(format!("{}: {e}", case.name));
        }
        faces += r.shape.count(SubKind::Face);
        heal_ok += 1;
    }
    assert!(heal_ok >= 60, "{heal_ok} BREP-10 results");
    eprintln!(
        "completeness: {} BREP-03/14 results, {bool_ok} BREP-04 results, {} BREP-06 results, \
         {} BREP-07 results, {heal_ok} BREP-10 results, {faces} output faces",
        corpus.len(),
        blends.len(),
        offsets.len()
    );
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn primitive_faces_carry_cap_and_lateral_roles() {
    let k = kernel();
    let t = tilted();
    let roles = |r: &OpResult| {
        let mut m: BTreeMap<String, u32> = BTreeMap::new();
        for (_, role) in &r.history.roles {
            *m.entry(format!("{role:?}")).or_default() += 1;
        }
        m
    };
    let expect = |r: &OpResult, start: u32, end: u32, lateral: u32| {
        let m = roles(r);
        assert_eq!(m.get("StartCap").copied().unwrap_or(0), start, "{m:?}");
        assert_eq!(m.get("EndCap").copied().unwrap_or(0), end, "{m:?}");
        assert_eq!(m.get("Lateral").copied().unwrap_or(0), lateral, "{m:?}");
    };
    let block = k.make_box(&t, Vec3::new(10.0, 20.0, 30.0)).unwrap();
    expect(&block, 1, 1, 4);
    // The bottom is the face at the frame origin facing -normal.
    let (bottom, _) = block
        .history
        .roles
        .iter()
        .find(|(_, r)| *r == OpRole::StartCap)
        .unwrap();
    let g = k.sub_geom(&block.shape, *bottom).unwrap();
    assert!((g.axis.unwrap() + t.normal.get()).length() < 1e-12);
    expect(&k.make_cylinder(&t, 5.0, 12.0, TAU).unwrap(), 1, 1, 1);
    expect(&k.make_cylinder(&t, 5.0, 12.0, FRAC_PI_2).unwrap(), 1, 1, 3);
    expect(&k.make_cone(&t, 5.0, 0.0, 10.0, TAU).unwrap(), 1, 0, 1);
    expect(&k.make_sphere(&t, 7.0).unwrap(), 0, 0, 1);
    expect(&k.make_torus(&t, 10.0, 3.0, PI).unwrap(), 0, 0, 3);
}

// ---- Done-when 2: tagged prism ---------------------------------------------------------------

#[test]
fn prism_laterals_come_from_tagged_profile_edges() {
    let k = kernel();
    // An L-shape with a hole made of two arcs, on a tilted plane: 6 + 2 tagged edges.
    let l_shape = polygon(&[
        (0.0, 0.0),
        (20.0, 0.0),
        (20.0, 8.0),
        (8.0, 8.0),
        (8.0, 20.0),
        (0.0, 20.0),
    ]);
    let (face, tags) = profile(&k, tilted(), l_shape, vec![arcs((4.0, 4.0), 2.0, 2, true)]);
    assert_eq!(tags.len(), 8);
    let tag_of: BTreeMap<u32, Tag> = tags.iter().map(|(e, t)| (e.0, *t)).collect();
    let r = extrude(&k, &face, 12.0 * tilted().normal.get(), opts(0.0, false));
    assert_eq!(completeness_error(&r, &[&face]), None);
    let h = &r.history;

    let mut lateral_tags = Vec::new();
    let (mut start, mut end) = (Vec::new(), Vec::new());
    for f in r.shape.faces() {
        let o = face_sub(f.0);
        let sources = h.sources_of(o);
        match h.role_of(o) {
            Some(OpRole::Lateral) => {
                // Exactly one source: Generated from one profile edge, which carries a tag.
                assert_eq!(sources.len(), 1, "face {f:?}: {sources:?}");
                let (input, relation) = sources[0];
                assert!(matches!(relation, Relation::Generated(_)), "{relation:?}");
                assert_eq!(input.operand, 0);
                assert_eq!(input.sub.kind, SubKind::Edge);
                lateral_tags.push(tag_of[&input.sub.index]);
            }
            Some(OpRole::StartCap) => start.push((f, sources)),
            Some(OpRole::EndCap) => end.push((f, sources)),
            other => panic!("face {f:?} has role {other:?}"),
        }
    }
    lateral_tags.sort();
    let mut want: Vec<Tag> = tags.iter().map(|(_, t)| *t).collect();
    want.sort();
    assert_eq!(
        lateral_tags, want,
        "each tagged edge sweeps one lateral face"
    );
    assert_eq!(start.len(), 1);
    assert_eq!(end.len(), 1);
    let profile_face = InputSub {
        operand: 0,
        sub: face_sub(0),
    };
    assert!(matches!(start[0].1[..], [(i, Relation::Modified(_))] if i == profile_face));
    assert!(matches!(end[0].1[..], [(i, Relation::Generated(_))] if i == profile_face));
}

// ---- Done-when 3: split order of a cut ---------------------------------------------------------

/// A 10 mm cube cut by a cylinder (r 1) along X at y = 3 on the top face: the top face splits
/// into strips y in [0, 2] (area 20) and [4, 10] (area 60).
fn grooved_cube(k: &OcctKernel) -> (Shape, Shape, OpResult) {
    let cube = k.make_box(&Plane::XY, Vec3::splat(10.0)).unwrap().shape;
    let axis = frame(
        [-1.0, 3.0, 10.0],
        Some([1.0, 0.0, 0.0]),
        Some([0.0, 1.0, 0.0]),
    );
    let tool = k.make_cylinder(&axis, 1.0, 12.0, TAU).unwrap().shape;
    let r = k
        .boolean(
            BoolOp::Cut,
            std::slice::from_ref(&cube),
            std::slice::from_ref(&tool),
            &BoolOpts::default(),
            &ctx(),
        )
        .unwrap();
    (cube, tool, r)
}

/// The cube face whose outward normal is +Z.
fn top_face(k: &OcctKernel, cube: &Shape) -> u32 {
    cube.faces()
        .find(|f| {
            let g = k.sub_geom(cube, face_sub(f.0)).unwrap();
            g.axis.is_some_and(|a| (a - Vec3::Z).length() < 1e-12)
        })
        .unwrap()
        .0
}

#[test]
fn a_split_face_lists_its_pieces_by_area_descending_every_run() {
    let k = kernel();
    let mut runs = Vec::new();
    for _ in 0..10 {
        let (cube, tool, r) = grooved_cube(&k);
        assert_eq!(completeness_error(&r, &[&cube, &tool]), None);
        let top = InputSub {
            operand: 0,
            sub: face_sub(top_face(&k, &cube)),
        };
        // The top face is Modified into its two strips (and Generates the section edges on it).
        let relations: Vec<&Relation> = r.history.of_input(top).collect();
        let modified: Vec<_> = relations
            .iter()
            .filter_map(|r| match r {
                Relation::Modified(o) => Some(o),
                _ => None,
            })
            .collect();
        let [pieces] = modified[..] else {
            panic!("top face: {relations:?}");
        };
        assert_eq!(pieces.len(), 2, "{pieces:?}");
        let areas: Vec<f64> = pieces
            .iter()
            .map(|p| k.sub_geom(&r.shape, *p).unwrap().measure)
            .collect();
        assert!((areas[0] - 60.0).abs() < 1e-9, "{areas:?}");
        assert!((areas[1] - 20.0).abs() < 1e-9, "{areas:?}");
        runs.push((bin(&r.shape), format!("{:?}", r.history), pieces.to_vec()));
    }
    for (i, run) in runs.iter().enumerate().skip(1) {
        assert!(run.0 == runs[0].0, "run {i}: result bytes differ");
        assert!(run.1 == runs[0].1, "run {i}: history differs");
        assert_eq!(run.2, runs[0].2, "run {i}: split order differs");
    }
}

#[test]
fn split_order_key_breaks_ties_by_centroid_then_index() {
    // Two congruent pieces: equal area, ordered by centroid x, then y.
    let g = |measure: f64, c: (f64, f64, f64)| SubShapeGeom {
        class: GeomClass::Plane,
        measure,
        centroid: Point3::new(c.0, c.1, c.2),
        axis: None,
        bbox: amg_brep::Aabb3::EMPTY,
    };
    let a = split_order_key(&g(20.0, (1.0, 0.0, 0.0)), 7);
    let b = split_order_key(&g(20.0 + 1e-8, (0.0, 5.0, 0.0)), 3);
    let c = split_order_key(&g(60.0, (9.0, 9.0, 9.0)), 9);
    // b's area is within the quantum of a's: the centroid decides.
    let mut keys = [a, b, c];
    keys.sort();
    assert_eq!(keys, [c, b, a]);
}

// ---- Done-when 4: compose -----------------------------------------------------------------------

#[test]
fn compose_of_prism_and_unify_equals_the_end_to_end_history() {
    let k = kernel();
    let t = tilted();
    type Case = (&'static str, Plane, Vec<Curve2d>, Vec<Vec<Curve2d>>);
    let cases: Vec<Case> = vec![
        (
            "square_split_side",
            Plane::XY,
            polygon(&[
                (0.0, 0.0),
                (5.0, 0.0),
                (10.0, 0.0),
                (10.0, 10.0),
                (0.0, 10.0),
            ]),
            vec![],
        ),
        (
            "l_shape_two_splits",
            t,
            polygon(&[
                (0.0, 0.0),
                (10.0, 0.0),
                (20.0, 0.0),
                (20.0, 8.0),
                (8.0, 8.0),
                (8.0, 14.0),
                (8.0, 20.0),
                (0.0, 20.0),
            ]),
            vec![],
        ),
        (
            "rectangle_all_sides_split",
            Plane::XY,
            polygon(&[
                (0.0, 0.0),
                (6.0, 0.0),
                (12.0, 0.0),
                (12.0, 4.0),
                (12.0, 8.0),
                (6.0, 8.0),
                (0.0, 8.0),
                (0.0, 4.0),
            ]),
            vec![],
        ),
        (
            "split_side_and_two_arc_hole",
            t,
            polygon(&[
                (0.0, 0.0),
                (5.0, 0.0),
                (10.0, 0.0),
                (10.0, 10.0),
                (0.0, 10.0),
            ]),
            vec![arcs((5.0, 5.0), 2.0, 2, true)],
        ),
        ("slot_split_arc", t, slot(12.0, 3.0, true), vec![]),
    ];
    for (name, plane, outer, holes) in cases {
        let (face, _) = profile(&k, plane, outer, holes);
        let v = 7.0 * plane.normal.get();
        let raw = extrude(&k, &face, v, opts(0.0, false));
        let unified = k.unify_coplanar(&raw.shape).unwrap();
        let end_to_end = extrude(&k, &face, v, opts(0.0, true));
        assert!(
            unified.shape.count(SubKind::Face) < raw.shape.count(SubKind::Face),
            "{name}: nothing merged"
        );
        // The same topology in the same index order: equal fingerprints of every sub-shape.
        assert_eq!(
            k.sub_geom_all(&unified.shape).unwrap(),
            k.sub_geom_all(&end_to_end.shape).unwrap(),
            "{name}: unify after prism and prism with merge_coplanar differ"
        );
        assert_eq!(completeness_error(&unified, &[&raw.shape]), None, "{name}");
        let mut composed = raw.history.compose(&unified.history);
        composed.order_splits(&unified.shape).unwrap();
        assert_eq!(composed, end_to_end.history, "{name}");
        assert_eq!(completeness_error(&end_to_end, &[&face]), None, "{name}");
        eprintln!(
            "{name}: {} → {} faces, {} entries",
            raw.shape.count(SubKind::Face),
            unified.shape.count(SubKind::Face),
            composed.entries.len()
        );
    }
}

// ---- Done-when 5: serde ---------------------------------------------------------------------------

#[test]
fn histories_round_trip_through_serde() {
    let k = kernel();
    let mut histories: Vec<ShapeHistory> = op_corpus(&k)
        .into_iter()
        .map(|(_, _, r)| r.history)
        .collect();
    histories.push(grooved_cube(&k).2.history);
    let tagged_wire = k
        .make_wire(
            &[line_seg(Point3::ZERO, Point3::new(1.0, 2.0, 3.0), 77)],
            &TOL,
        )
        .unwrap();
    histories.push(tagged_wire.history);
    let mut bytes = 0;
    for h in &histories {
        let json = serde_json::to_string(h).unwrap();
        bytes += json.len();
        let back: ShapeHistory = serde_json::from_str(&json).unwrap();
        assert_eq!(&back, h);
        let pretty = serde_json::to_string_pretty(h).unwrap();
        assert_eq!(&serde_json::from_str::<ShapeHistory>(&pretty).unwrap(), h);
    }
    assert!(histories.iter().any(|h| !h.tags.is_empty()));
    assert!(histories.iter().any(|h| h.operand_counts.len() == 2));
    eprintln!("{} histories, {bytes} bytes of JSON", histories.len());
}

// ---- Done-when 6: sub_geom ------------------------------------------------------------------------

#[test]
fn sub_geom_gives_analytic_box_and_cylinder_values() {
    let k = kernel();
    let t = tilted();
    let block = k.make_box(&t, Vec3::new(10.0, 20.0, 30.0)).unwrap().shape;
    let [faces, edges, vertices] = k.sub_geom_all(&block).unwrap();
    assert_eq!((faces.len(), edges.len(), vertices.len()), (6, 12, 8));
    let centre = t.origin + 5.0 * t.x.get() + 10.0 * t.y.get() + 15.0 * t.normal.get();
    let mut areas: Vec<f64> = faces.iter().map(|g| g.measure).collect();
    areas.sort_by(f64::total_cmp);
    for (got, want) in areas.iter().zip([200.0, 200.0, 300.0, 300.0, 600.0, 600.0]) {
        assert!((got - want).abs() < 1e-9 * want, "{areas:?}");
    }
    for g in &faces {
        assert_eq!(g.class, GeomClass::Plane);
        let n = g.axis.unwrap();
        // Outward: the normal points from the box centre to the face centroid.
        assert!((n.length() - 1.0).abs() < 1e-12);
        assert!(n.dot(g.centroid - centre) > 0.0);
        assert!(g.bbox.min.x <= g.centroid.x + 1e-9 && g.centroid.x <= g.bbox.max.x + 1e-9);
    }
    let total: f64 = edges.iter().map(|g| g.measure).sum();
    assert!((total - 240.0).abs() < 1e-9);
    assert!(
        edges
            .iter()
            .all(|g| g.class == GeomClass::Line && g.axis.is_some())
    );
    assert!(
        vertices
            .iter()
            .all(|g| g.class == GeomClass::Point && g.measure == 0.0)
    );
    for (i, g) in faces.iter().enumerate() {
        assert_eq!(&k.sub_geom(&block, face_sub(i as u32)).unwrap(), g);
    }

    let cyl = k.make_cylinder(&t, 5.0, 12.0, TAU).unwrap().shape;
    let [faces, edges, _] = k.sub_geom_all(&cyl).unwrap();
    let side = faces
        .iter()
        .find(|g| g.class == GeomClass::Cylinder)
        .unwrap();
    assert!((side.measure - TAU * 5.0 * 12.0).abs() < 1e-9 * side.measure);
    assert!(side.axis.unwrap().cross(t.normal.get()).length() < 1e-12);
    let mid = t.origin + 6.0 * t.normal.get();
    assert!((side.centroid - mid).length() < 1e-9);
    let rim = edges.iter().find(|g| g.class == GeomClass::Circle).unwrap();
    assert!((rim.measure - TAU * 5.0).abs() < 1e-9);
    assert!(k.sub_geom(&cyl, face_sub(99)).is_err());
}

#[test]
fn sub_geom_all_of_a_thousand_face_part_is_fast() {
    let k = OcctKernel::new(KernelConfig::default());
    let plate = parts::plate(&k, 32, Point3::ZERO);
    assert_eq!(plate.count(SubKind::Face), 1030);
    let first = k.sub_geom_all(&plate).unwrap();
    assert_eq!(first[0].len(), 1030);
    let mut times: Vec<Duration> = (0..7)
        .map(|_| {
            let start = Instant::now();
            let again = k.sub_geom_all(&plate).unwrap();
            let took = start.elapsed();
            assert!(again == first, "sub_geom_all is not deterministic");
            took
        })
        .collect();
    times.sort();
    let median = times[times.len() / 2];
    eprintln!(
        "sub_geom_all, 1030 faces / {} edges / {} vertices: median {median:?} of 7 (test profile)",
        first[1].len(),
        first[2].len()
    );
    if strict_timing() {
        assert!(median < Duration::from_millis(20), "{median:?}");
    }
}

// ---- R-S04-2: per_solid ---------------------------------------------------------------------------

#[test]
fn per_solid_restricts_and_reindexes_the_history() {
    let k = kernel();
    // A 30 × 10 × 10 bar cut by a 10 mm slab in its middle: two 10 mm cubes.
    let bar = k
        .make_box(&Plane::XY, Vec3::new(30.0, 10.0, 10.0))
        .unwrap()
        .shape;
    let slab_frame = frame([10.0, -1.0, -1.0], None, None);
    let slab = k
        .make_box(&slab_frame, Vec3::new(10.0, 12.0, 12.0))
        .unwrap()
        .shape;
    let cut = k
        .boolean(
            BoolOp::Cut,
            std::slice::from_ref(&bar),
            std::slice::from_ref(&slab),
            &BoolOpts::default(),
            &ctx(),
        )
        .unwrap();
    assert_eq!(cut.shape.solids().len(), 2);
    let pieces = cut.per_solid().unwrap();
    assert_eq!(pieces.len(), 2);
    let mut faces = 0;
    for (i, p) in pieces.iter().enumerate() {
        assert_eq!(completeness_error(p, &[&bar, &slab]), None, "piece {i}");
        assert_eq!(p.shape.count(SubKind::Face), 6);
        faces += p.shape.count(SubKind::Face);
        // The bar's 4 long side faces are split, one piece in each solid.
        let split: Vec<_> = p
            .history
            .entries
            .iter()
            .filter(|(input, r)| {
                input.operand == 0 && matches!(r, Relation::Modified(o) if o.len() > 1)
            })
            .collect();
        assert!(split.is_empty(), "piece {i}: {split:?}");
        // Face centroids of the piece lie inside it.
        let bb = p.shape.bbox();
        for f in p.shape.faces() {
            let c = k.sub_geom(&p.shape, face_sub(f.0)).unwrap().centroid;
            assert!(c.x >= bb.min.x - 1e-9 && c.x <= bb.max.x + 1e-9);
        }
    }
    assert_eq!(faces, cut.shape.count(SubKind::Face));
    // In the whole result each long side face has two Modified pieces.
    let whole_splits = cut
        .history
        .entries
        .iter()
        .filter(|(input, r)| {
            input.operand == 0
                && input.sub.kind == SubKind::Face
                && matches!(r, Relation::Modified(o) if o.len() == 2)
        })
        .count();
    assert_eq!(whole_splits, 4);

    // A split of a box by a face: SplitPiece roles name the solid of each face.
    let cube = k.make_box(&Plane::XY, Vec3::splat(10.0)).unwrap().shape;
    let cutter = k
        .make_planar_polygon(
            &[
                Point3::new(-1.0, -1.0, 4.0),
                Point3::new(11.0, -1.0, 4.0),
                Point3::new(11.0, 11.0, 4.0),
                Point3::new(-1.0, 11.0, 4.0),
            ],
            &[],
            &TOL,
        )
        .unwrap();
    let split = k
        .split(
            std::slice::from_ref(&cube),
            std::slice::from_ref(&cutter),
            &BoolOpts::default(),
            &ctx(),
        )
        .unwrap();
    assert_eq!(completeness_error(&split, &[&cube, &cutter]), None);
    let parts = split.per_solid().unwrap();
    assert_eq!(parts.len(), 2);
    for (i, part) in parts.iter().enumerate() {
        let index = u32::try_from(i).unwrap();
        assert_eq!(
            completeness_error(part, &[&cube, &cutter]),
            None,
            "part {i}"
        );
        let own = part
            .history
            .roles
            .iter()
            .filter(|(_, r)| *r == OpRole::SplitPiece { index })
            .count();
        // Every face of part 0 says piece 0; the shared cut face is piece 0's in part 1.
        assert!(own >= 5, "part {i}: {:?}", part.history.roles);
    }
    let volumes: Vec<f64> = parts.iter().map(|p| p.shape.volume().unwrap()).collect();
    assert!(
        (volumes.iter().sum::<f64>() - 1000.0).abs() < 1e-9,
        "{volumes:?}"
    );
}

#[test]
fn face_roles_and_sources_of_a_split_are_in_range_of_each_piece() {
    // per_solid of a single-solid result is the same history over the same indices.
    let k = kernel();
    let (_, _, r) = grooved_cube(&k);
    let parts = r.per_solid().unwrap();
    assert_eq!(parts.len(), 1);
    assert_eq!(counts(&parts[0].shape), counts(&r.shape));
    assert_eq!(parts[0].history, r.history);
    let none: Vec<FaceId> = parts[0]
        .history
        .unexplained_faces(parts[0].shape.count(SubKind::Face));
    assert!(none.is_empty());
}

// ---- sub_geom accuracy ----------------------------------------------------------------------------

const FIXTURES: [&str; 11] = [
    "box",
    "cylinder",
    "cone",
    "sphere",
    "torus",
    "filleted_box",
    "plate_with_holes",
    "located_compound",
    "bspline_patch",
    "revolved_profile",
    "cylinder_mirrored",
];

fn load(name: &str) -> Option<Shape> {
    let path = amg_testkit::testdata(&format!("brep/{name}.brep"))?;
    let file = std::fs::File::open(path).unwrap();
    Some(Shape::read_text(std::io::BufReader::new(file)).unwrap())
}

/// Face areas and centroids of `sub_geom` (Green's theorem on analytic surfaces, fixed-order
/// `BRepGProp` elsewhere) against the adaptive `mass_props` of each face (eps 1e-12).
#[test]
fn face_fingerprints_match_adaptive_integration() {
    let k = kernel();
    let mut shapes: Vec<(String, Shape)> = FIXTURES
        .iter()
        .filter_map(|n| load(n).map(|s| ((*n).to_owned(), s)))
        .collect();
    for (name, _, r) in op_corpus(&k) {
        shapes.push((name, r.shape));
    }
    shapes.push(("grooved_cube".into(), grooved_cube(&k).2.shape));
    if let Some(cases) = bool_corpus::corpus() {
        for case in cases.iter().step_by(4) {
            let (objects, tools) = case.operands(&k);
            if let Ok(r) = case.run(&k, &objects, &tools, None) {
                shapes.push((case.name.clone(), r.shape));
            }
        }
    }
    // [analytic surfaces (Green's theorem), others (fixed-order BRepGProp)]: worst relative area
    // error, worst centroid error over the face size, and the bound.
    // The analytic bound allows for fuzzy-glued faces, whose pcurves are within the 1e-5 mm gap.
    let mut worst = [(0.0_f64, 0.0_f64, 1e-5), (0.0_f64, 0.0_f64, 1e-2)];
    let mut faces = 0;
    let mut failures = Vec::new();
    for (name, s) in &shapes {
        let [geoms, _, _] = k.sub_geom_all(s).unwrap();
        for (i, g) in geoms.iter().enumerate() {
            let f = FaceId(i as u32);
            // Unbounded faces (infinite prisms) and faces with hundreds of loops (the plate caps,
            // 1.8 s each adaptively) are left out.
            if !g.measure.is_finite() || s.face_edges(f).len() > 64 || g.measure > 1e9 {
                continue;
            }
            let face = s.sub_shape(face_sub(f.0)).unwrap();
            let Ok(reference) = k.mass_props(&face) else {
                continue;
            };
            faces += 1;
            let analytic = matches!(
                g.class,
                GeomClass::Plane
                    | GeomClass::Cylinder
                    | GeomClass::Cone
                    | GeomClass::Sphere
                    | GeomClass::Torus
            );
            let slot = &mut worst[usize::from(!analytic)];
            let size = g.bbox.max.distance(g.bbox.min).max(1.0);
            let area_err = (g.measure - reference.area).abs() / reference.area.max(1e-12);
            let centroid_err = g.centroid.distance(reference.centroid) / size;
            slot.0 = slot.0.max(area_err);
            slot.1 = slot.1.max(centroid_err);
            if area_err > slot.2 || centroid_err > slot.2 {
                failures.push(format!(
                    "{name} face {i} ({:?}): area {} vs {} ({area_err:.1e}), centroid off {centroid_err:.1e}",
                    g.class, g.measure, reference.area
                ));
            }
        }
    }
    eprintln!(
        "{faces} faces of {} shapes; largest relative area / centroid errors: analytic {:.2e} / {:.2e}, B-spline and other {:.2e} / {:.2e}",
        shapes.len(),
        worst[0].0,
        worst[0].1,
        worst[1].0,
        worst[1].1
    );
    assert!(faces > 300, "{faces}");
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}
