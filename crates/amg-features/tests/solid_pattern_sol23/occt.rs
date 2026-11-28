//! The OCCT half of the SOL-23 tests: path patterns against `CurveEval` frames, face patterns
//! of a boss and a pocket, adjust compute across a sloped target; goldens
//! `tests/golden/solid/sol23_*.toml`.

use amg_brep::{BrepKernel, Shape as BShape, SubKind};
use amg_core::{Dir3, Plane, Point2, Point3, Tolerance, Transform, Vec3};
use amg_doc::naming::{GeomSource, KernelGeom, Role, default_kernel, topo_ref_at};
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, ComputeMode, FeatureBox, FeatureError, FeatureOutput,
    Health, RecomputeCtx,
};
use amg_features::construct::curves::curve_of;
use amg_features::construct::math::{path_param, plane_along_path};
use amg_features::construct::resolve::resolve_in;
use amg_features::construct::{OriginEntity, RefInput};
use amg_features::sketch::{PlaneInput, SketchFeature};
use amg_features::solid::{BodyOp, Extent, ExtentSide, Extrude, OpTarget, PatternDir};
use serde::{Deserialize, Serialize};

use super::*;

// ---- a box body for plates and targets ----------------------------------------------------------

/// A box on the frame (`origin`, `normal`, `x`) with edge lengths `size`, faces `F#i`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Sol23Box {
    pub origin: [f64; 3],
    pub normal: [f64; 3],
    pub x: [f64; 3],
    pub size: [f64; 3],
}

impl Feature for Sol23Box {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let k = ctx.kernel();
        let kerr = |e: amg_brep::KernelError| FeatureError::Kernel(e.to_string());
        let n = Dir3::new_unchecked(Vec3::from_array(self.normal).normalize());
        let frame = Plane::from_normal_x(
            Point3::from_array(self.origin),
            n,
            Vec3::from_array(self.x),
            &Tolerance::default(),
        )
        .unwrap();
        let shape = k
            .make_box(&frame, Vec3::from_array(self.size))
            .map_err(kerr)?
            .shape;
        let mut b = ctx.naming_builder();
        for f in 0..shape.count(SubKind::Face) {
            b.generated(TopoKind::Face, f, Role::new("F", f), None);
        }
        let naming = b
            .finish(k, &shape)
            .map_err(|e| FeatureError::Kernel(e.to_string()))?;
        let mut out = FeatureOutput::default();
        out.bodies.push(BodyChange {
            id: ctx.new_body_id(0),
            change: BodyChangeKind::Created {
                geom: BodyGeom::Solid(amg_doc::stub::Shape::from_brep(shape)),
                naming: Some(naming),
            },
        });
        Ok(out)
    }
}

amg_doc::register_feature!(Sol23Box, "sol23test.box", version = 1);

fn abox(doc: &mut Document, min: [f64; 3], size: [f64; 3]) -> BodyId {
    body0(add(
        doc,
        FeatureBox::new(Sol23Box {
            origin: min,
            normal: [0.0, 0.0, 1.0],
            x: [1.0, 0.0, 0.0],
            size,
        }),
    ))
}

// ---- helpers ----------------------------------------------------------------------------------

#[track_caller]
fn valid(s: &BShape) {
    let r = kernel().check(s, &amg_brep::CheckOpts::default());
    assert!(r.valid, "{r:?}");
}

#[track_caller]
fn golden(case: &str, s: &BShape, rel: f64) {
    valid(s);
    check_golden_as("sol23", rel, case, s);
}

#[track_caller]
fn ok(doc: &Document, f: FeatureId) {
    assert_eq!(health(doc, f), Health::Ok, "feature {f:?}");
}

/// Every live solid body produced by feature `f`, by slot.
fn bodies_of(doc: &Document, f: FeatureId) -> Vec<(u32, BShape)> {
    let mut v: Vec<(u32, BShape)> = doc
        .evaluated()
        .bodies()
        .filter(|(id, _)| id.origin == f)
        .filter_map(|(id, b)| match &b.geom {
            BodyGeom::Solid(s) => s.brep().map(|s| (id.slot, s.clone())),
            _ => None,
        })
        .collect();
    v.sort_by_key(|(slot, _)| *slot);
    v
}

fn centroid(s: &BShape) -> Point3 {
    kernel().mass_props(s).unwrap().centroid
}

fn ctx_for<'a>(doc: &'a Document, f: FeatureId) -> RecomputeCtx<'a> {
    RecomputeCtx::new(
        doc.design(),
        doc.evaluated(),
        f,
        doc.design().root(),
        ComputeMode::Full,
    )
}

/// Every face of `body` whose fingerprint centroid satisfies `keep`, with keys 1, 2, ….
fn faces_where(doc: &Document, body: BodyId, keep: impl Fn(Point3) -> bool) -> Vec<TopoRef> {
    let ev: &amg_doc::Evaluated = doc.evaluated();
    let geom = KernelGeom::new(ev, default_kernel());
    let mut v = Vec::new();
    for (i, fp) in GeomSource::all(&geom, body, TopoKind::Face) {
        if keep(fp.centroid) {
            let mut r = topo_ref_at(ev, default_kernel(), body, TopoKind::Face, i).unwrap();
            r.key = u32::try_from(v.len() + 1).unwrap();
            v.push(r);
        }
    }
    v
}

fn pattern(objects: PatternObjects, kind: PatternKind) -> Pattern {
    Pattern {
        objects,
        kind,
        suppressed: Vec::new(),
        compute: ComputeType::Identical,
        target: None,
    }
}

fn along_x(count: &str, spacing: &str) -> PatternKind {
    PatternKind::Rectangular {
        d1: PatternDir {
            dir: RefInput::Origin(OriginEntity::XAxis),
            count: ex(count),
            spacing: Spacing::Spacing(ex(spacing)),
            flip: false,
            symmetric: false,
        },
        d2: None,
    }
}

// ---- Done-when 1: a path pattern along a spline ---------------------------------------------------

const POLES: [[f64; 2]; 6] = [
    [0.0, 0.0],
    [30.0, 40.0],
    [60.0, -20.0],
    [100.0, 30.0],
    [130.0, -10.0],
    [160.0, 20.0],
];

/// A sketch on XY holding one cubic spline; returns the sketch feature and the path ref.
fn spline_path(doc: &mut Document) -> (FeatureId, EntityRef) {
    let mut sk = SketchFeature::new(PlaneInput::OriginXY);
    let id = sk
        .edit_sketch(|s| {
            let poles: Vec<Point2> = POLES.iter().map(|p| Point2::new(p[0], p[1])).collect();
            s.add_spline(3, &poles, None, None)
        })
        .unwrap();
    let f = add(doc, FeatureBox::new(sk));
    (f, curve_ref(f, u64::from(id.0)))
}

/// The reference frame of the path at arc length `s`: CON-03 resolution, `curve_of`,
/// `path_param` and `plane_along_path`, independent of the pattern code.
fn reference_frame(doc: &Document, path: &EntityRef, s: f64) -> Plane {
    let g = resolve_in(doc.evaluated(), doc.design(), &RefInput::Ref(path.clone()))
        .unwrap()
        .geom;
    let c = curve_of(&g).unwrap();
    let t = path_param(&*c, s, true).unwrap();
    plane_along_path(&*c, t, &Tolerance::DEFAULT).unwrap()
}

fn path_length(doc: &Document, path: &EntityRef) -> f64 {
    let g = resolve_in(doc.evaluated(), doc.design(), &RefInput::Ref(path.clone()))
        .unwrap()
        .geom;
    curve_of(&g).unwrap().length()
}

#[track_caller]
fn same_frame(a: &Plane, b: &Plane, tol: f64) {
    let d = [
        (a.origin - b.origin).length(),
        (a.x.get() - b.x.get()).length(),
        (a.y.get() - b.y.get()).length(),
        (a.normal.get() - b.normal.get()).length(),
    ];
    assert!(d.iter().all(|x| *x <= tol), "{a:?} vs {b:?}: {d:?}");
}

/// `frame` moved by `t` (rigid).
fn moved(t: &Transform, p: &Plane) -> Plane {
    Plane {
        origin: t.transform_point3(p.origin),
        x: Dir3::new_unchecked(t.transform_vector3(p.x.get())),
        y: Dir3::new_unchecked(t.transform_vector3(p.y.get())),
        normal: Dir3::new_unchecked(t.transform_vector3(p.normal.get())),
    }
}

#[test]
fn dw1_eight_instances_along_a_spline_follow_the_curve_frames() {
    let mut doc = Document::new("path");
    let (_, path) = spline_path(&mut doc);
    let len = path_length(&doc, &path);
    // A small block at the start of the path.
    let seed = abox(&mut doc, [-2.0, -2.0, -2.0], [4.0, 4.0, 4.0]);
    let (start, step) = (0.1, 0.1 * len);
    let p = pattern(
        PatternObjects::Bodies(vec![seed]),
        path_kind(
            vec![path.clone()],
            "8",
            Spacing::Spacing(ex(&format!("{step} mm"))),
            &start.to_string(),
        ),
    );
    let pid = add(&mut doc, FeatureBox::new(p.clone()));
    ok(&doc, pid);
    let ctx = ctx_for(&doc, pid);
    let inst = p.instances(&ctx).unwrap();
    assert_eq!(
        inst.iter().map(|x| x.0).collect::<Vec<_>>(),
        (1..8).collect::<Vec<_>>()
    );
    let f0 = reference_frame(&doc, &path, start * len);
    let seed_c = centroid(&brep(&doc, seed));
    let copies = bodies_of(&doc, pid);
    assert_eq!(copies.len(), 7);
    let mut worst_frame = 0.0f64;
    let mut worst_body = 0.0f64;
    for ((k, t), (slot, s)) in inst.iter().zip(&copies) {
        assert_eq!(k, slot);
        let fk = reference_frame(&doc, &path, start * len + step * f64::from(*k));
        let got = moved(t, &f0);
        same_frame(&got, &fk, 1e-9);
        worst_frame = worst_frame.max((got.origin - fk.origin).length());
        // The copy sits where the frame map puts the seed.
        let want = fk
            .to_transform()
            .transform_point3(f0.to_transform().inverse().transform_point3(seed_c));
        let d = (centroid(s) - want).length();
        worst_body = worst_body.max(d);
        assert!(d < 1e-9, "instance {k}: {d}");
        valid(s);
        assert_rel(volume(s), 64.0, 1e-12);
    }
    println!("path pattern: worst frame origin {worst_frame:e} mm, body {worst_body:e} mm");
    golden("path_body_k7", &copies[6].1, 1e-9);

    // Identical orientation: pure translations along the curve.
    let mut q = p.clone();
    let PatternKind::Path { orientation, .. } = &mut q.kind else {
        unreachable!()
    };
    *orientation = PathOrientation::Identical;
    for (k, t) in q.instances(&ctx).unwrap() {
        let fk = reference_frame(&doc, &path, start * len + step * f64::from(k));
        let d = t.transform_point3(f0.origin) - fk.origin;
        assert!(d.length() < 1e-9);
        assert!(t.matrix3.abs_diff_eq(Transform::IDENTITY.matrix3, 1e-15));
    }
}

#[test]
fn path_extent_flip_and_open_ends() {
    let mut doc = Document::new("path-extent");
    let (_, path) = spline_path(&mut doc);
    let len = path_length(&doc, &path);
    let seed = abox(&mut doc, [-2.0, -2.0, -2.0], [4.0, 4.0, 4.0]);
    // Extent: the last of 5 instances at the end of the path.
    let p = pattern(
        PatternObjects::Bodies(vec![seed]),
        path_kind(
            vec![path.clone()],
            "5",
            Spacing::Extent(ex(&format!("{len} mm"))),
            "0",
        ),
    );
    let ctx = ctx_for(&doc, FeatureId(9_999));
    let inst = p.instances(&ctx).unwrap();
    assert_eq!(inst.len(), 4);
    let end = reference_frame(&doc, &path, len);
    let f0 = reference_frame(&doc, &path, 0.0);
    same_frame(&moved(&inst[3].1, &f0), &end, 1e-9);
    // Flipped from the end with a spacing too large for all of them: two fall off the start.
    let mut q = pattern(
        PatternObjects::Bodies(vec![seed]),
        path_kind(
            vec![path],
            "5",
            Spacing::Spacing(ex(&format!("{} mm", 0.4 * len))),
            "1",
        ),
    );
    if let PatternKind::Path { flip, .. } = &mut q.kind {
        *flip = true;
    }
    let pid = add(&mut doc, FeatureBox::new(q));
    let Health::Warning(w) = health(&doc, pid) else {
        panic!("{:?}", health(&doc, pid))
    };
    assert!(
        w.iter()
            .any(|w| w.message.contains("beyond the end of the path")),
        "{w:?}"
    );
    assert_eq!(bodies_of(&doc, pid).len(), 2);
}

#[test]
fn a_disconnected_path_or_a_bad_start_is_a_clean_error() {
    let mut doc = Document::new("path-bad");
    let mut sk = SketchFeature::new(PlaneInput::OriginXY);
    let (a, b) = sk
        .edit_sketch(|s| {
            let a = s.add_line(Point2::new(0.0, 0.0), Point2::new(10.0, 0.0))?;
            let b = s.add_line(Point2::new(20.0, 0.0), Point2::new(30.0, 0.0))?;
            Ok((a, b))
        })
        .unwrap();
    let s = add(&mut doc, FeatureBox::new(sk));
    let seed = abox(&mut doc, [0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
    let bad = add(
        &mut doc,
        FeatureBox::new(pattern(
            PatternObjects::Bodies(vec![seed]),
            path_kind(
                vec![curve_ref(s, u64::from(a.0)), curve_ref(s, u64::from(b.0))],
                "2",
                Spacing::Spacing(ex("5 mm")),
                "0",
            ),
        )),
    );
    let Health::Error(d) = health(&doc, bad) else {
        panic!("{:?}", health(&doc, bad))
    };
    assert!(d.message.contains("doesn't start where"), "{d:?}");
    let start = add(
        &mut doc,
        FeatureBox::new(pattern(
            PatternObjects::Bodies(vec![seed]),
            path_kind(
                vec![curve_ref(s, u64::from(a.0))],
                "2",
                Spacing::Spacing(ex("5 mm")),
                "1.5",
            ),
        )),
    );
    assert!(matches!(health(&doc, start), Health::Error(_)));
}

// ---- Done-when 2: face patterns of a boss and a pocket --------------------------------------------

const PENTAGON: [[f64; 2]; 5] = [
    [0.0, 0.0],
    [16.0, 0.0],
    [20.0, 10.0],
    [8.0, 18.0],
    [-4.0, 10.0],
];

fn pentagon_area() -> f64 {
    let n = PENTAGON.len();
    (0..n)
        .map(|i| {
            let (a, b) = (PENTAGON[i], PENTAGON[(i + 1) % n]);
            a[0] * b[1] - b[0] * a[1]
        })
        .sum::<f64>()
        / 2.0
}

fn in_pentagon_box(p: Point3) -> bool {
    p.x > -5.0 && p.x < 21.0 && p.y > -1.0 && p.y < 19.0
}

/// A pentagon extrude by `h` (flipped when `down`) with `op` on `plate`.
fn pentagon(doc: &mut Document, h: f64, op: BodyOp, plate: BodyId) -> FeatureId {
    let (sk, _, _) = polygon_sketch(&PENTAGON);
    let s = add(doc, FeatureBox::new(sk));
    let e = Extrude::distance(
        profiles(doc, s),
        &format!("{h} mm"),
        OpTarget {
            op,
            participants: vec![plate],
        },
    );
    let e = add(doc, FeatureBox::new(e));
    ok(doc, e);
    e
}

#[test]
fn dw2_face_pattern_of_a_boss_gives_the_analytic_volume() {
    let mut doc = Document::new("boss");
    // Plate z ∈ [−10, 0]; a 5 mm pentagon boss on its top.
    let plate = abox(&mut doc, [-10.0, -10.0, -10.0], [120.0, 60.0, 10.0]);
    pentagon(&mut doc, 5.0, BodyOp::Join, plate);
    let boss = faces_where(&doc, plate, |c| c.z > 1e-3);
    assert_eq!(boss.len(), 6, "5 sides and the top");
    let p = add(
        &mut doc,
        FeatureBox::new(pattern(PatternObjects::Faces(boss), along_x("3", "35 mm"))),
    );
    ok(&doc, p);
    let s = brep(&doc, plate);
    let want = 120.0 * 60.0 * 10.0 + 3.0 * pentagon_area() * 5.0;
    let got = volume(&s);
    println!(
        "boss face pattern: volume {got} vs {want} (rel {:e})",
        (got - want).abs() / want
    );
    assert_rel(got, want, 1e-9);
    assert_eq!(faces(&s), 6 + 3 * 6);
    assert_eq!(s.solids().len(), 1);
    golden("face_pattern_boss", &s, 1e-9);
}

#[test]
fn dw2_face_pattern_of_a_pocket_gives_the_analytic_volume() {
    let mut doc = Document::new("pocket");
    // Plate z ∈ [0, 10]; a 3 mm pentagon pocket cut up from its bottom face.
    let plate = abox(&mut doc, [-10.0, -10.0, 0.0], [120.0, 60.0, 10.0]);
    pentagon(&mut doc, 3.0, BodyOp::Cut, plate);
    let pocket = faces_where(&doc, plate, |c| {
        in_pentagon_box(c) && c.z > 1e-3 && c.z < 10.0 - 1e-3
    });
    assert_eq!(pocket.len(), 6, "5 sides and the ceiling");
    let p = add(
        &mut doc,
        FeatureBox::new(pattern(
            PatternObjects::Faces(pocket),
            along_x("3", "35 mm"),
        )),
    );
    ok(&doc, p);
    let s = brep(&doc, plate);
    let want = 120.0 * 60.0 * 10.0 - 3.0 * pentagon_area() * 3.0;
    let got = volume(&s);
    println!(
        "pocket face pattern: volume {got} vs {want} (rel {:e})",
        (got - want).abs() / want
    );
    assert_rel(got, want, 1e-9);
    assert_eq!(faces(&s), 6 + 3 * 6);
    golden("face_pattern_pocket", &s, 1e-9);
}

#[test]
fn face_patterns_refuse_faces_without_a_planar_support() {
    let mut doc = Document::new("no-support");
    let plate = abox(&mut doc, [0.0, 0.0, 0.0], [20.0, 20.0, 10.0]);
    // Every face of the box: nothing left to sit on.
    let all = faces_where(&doc, plate, |_| true);
    let p = add(
        &mut doc,
        FeatureBox::new(pattern(PatternObjects::Faces(all), along_x("2", "30 mm"))),
    );
    let Health::Error(d) = health(&doc, p) else {
        panic!("{:?}", health(&doc, p))
    };
    assert!(d.message.contains("closed body"), "{d:?}");
    // Two adjacent box faces: the boundary runs over four other faces, not one plane.
    let two = faces_where(&doc, plate, |c| c.z > 9.999 || c.x > 19.999);
    assert_eq!(two.len(), 2);
    let p = add(
        &mut doc,
        FeatureBox::new(pattern(PatternObjects::Faces(two), along_x("2", "30 mm"))),
    );
    let Health::Error(d) = health(&doc, p) else {
        panic!("{:?}", health(&doc, p))
    };
    assert!(d.message.contains("planar"), "{d:?}");
}

// ---- Done-when 3: adjust compute of a to-object extrude --------------------------------------------

const ALPHA: f64 = 20.0;

/// A box tilted by `ALPHA` about Y whose bottom face is `z = 15 + tan α (x + 30)`.
fn tilted(doc: &mut Document) -> BodyId {
    let a = ALPHA.to_radians();
    body0(add(
        doc,
        FeatureBox::new(Sol23Box {
            origin: [-30.0, -30.0, 15.0],
            normal: [-a.sin(), 0.0, a.cos()],
            x: [a.cos(), 0.0, a.sin()],
            size: [90.0, 80.0, 20.0],
        }),
    ))
}

/// The mean height of a 10 × 20 rectangle `[x0, x0 + 10] × [0, 20]` under the tilted face.
fn mean_height(x0: f64) -> f64 {
    15.0 + ALPHA.to_radians().tan() * (x0 + 5.0 + 30.0)
}

/// A 10 × 20 rectangle extruded from XY up to the tilted face's bottom, new body.
fn to_slope(doc: &mut Document) -> FeatureId {
    let target = tilted(doc);
    let bottom = faces_where(doc, target, |c| {
        let a = ALPHA.to_radians();
        // The bottom face's centre: 45 mm along the tilted x from the origin corner.
        (c - Point3::new(-30.0 + 45.0 * a.cos(), 10.0, 15.0 + 45.0 * a.sin())).length() < 1e-6
    });
    assert_eq!(bottom.len(), 1);
    let s = add(doc, FeatureBox::new(rect_sketch(&[[0.0, 0.0, 10.0, 20.0]])));
    let mut e = Extrude::distance(profiles(doc, s), "1 mm", OpTarget::default());
    e.side1 = ExtentSide {
        extent: Extent::ToObject {
            target: EntityRef::Topo(bottom[0].clone()),
            offset: ex("0 mm"),
            extend: false,
        },
        taper: ex("0 deg"),
    };
    let e = add(doc, FeatureBox::new(e));
    ok(doc, e);
    e
}

/// The pattern's instance solids sorted by x, with their mean heights (volume / 200 mm²).
fn instance_heights(doc: &Document, p: FeatureId) -> Vec<(f64, BShape)> {
    let mut v: Vec<(f64, f64, BShape)> = bodies_of(doc, p)
        .into_iter()
        .map(|(_, s)| (s.bbox().min.x, volume(&s) / 200.0, s))
        .collect();
    v.sort_by(|a, b| a.0.total_cmp(&b.0));
    v.into_iter().map(|(_, h, s)| (h, s)).collect()
}

#[test]
fn dw3_adjust_recomputes_a_to_object_extent_per_instance() {
    let mut doc = Document::new("adjust");
    let e = to_slope(&mut doc);
    assert_rel(
        volume(&brep(&doc, body0(e))) / 200.0,
        mean_height(0.0),
        1e-9,
    );
    let mut p = pattern(PatternObjects::Features(vec![e]), along_x("4", "12 mm"));
    p.compute = ComputeType::Adjust;
    let adj = add(&mut doc, FeatureBox::new(p.clone()));
    ok(&doc, adj);
    let got = instance_heights(&doc, adj);
    assert_eq!(got.len(), 3);
    let mut worst = 0.0f64;
    for (i, (h, s)) in got.iter().enumerate() {
        valid(s);
        let x0 = 12.0 * (i as f64 + 1.0);
        let want = mean_height(x0);
        worst = worst.max((h - want).abs());
        assert!((h - want).abs() < 1e-6, "instance {}: {h} vs {want}", i + 1);
        // The top follows the slope: highest at the far edge.
        let top = 15.0 + ALPHA.to_radians().tan() * (x0 + 10.0 + 30.0);
        assert!(
            (s.bbox().max.z - top).abs() < 1e-6,
            "{} vs {top}",
            s.bbox().max.z
        );
    }
    println!(
        "adjust: heights {:?}, worst deviation {worst:e} mm",
        got.iter().map(|x| x.0).collect::<Vec<_>>()
    );
    golden("adjust_to_slope_k3", &got[2].1, 1e-9);

    // Identical compute moves the seed's tool: every instance keeps the seed's height.
    p.compute = ComputeType::Identical;
    let same = add(&mut doc, FeatureBox::new(p));
    ok(&doc, same);
    for (h, _) in instance_heights(&doc, same) {
        assert_rel(h, mean_height(0.0), 1e-9);
    }
}

#[test]
fn adjust_needs_a_sketch_based_seed() {
    let mut doc = Document::new("adjust-face");
    let plate = abox(&mut doc, [0.0, 0.0, 0.0], [20.0, 20.0, 5.0]);
    let top = faces_where(&doc, plate, |c| c.z > 4.999);
    let mut e = Extrude::distance(
        vec![EntityRef::Topo(top[0].clone())],
        "3 mm",
        OpTarget::default(),
    );
    e.target = OpTarget {
        op: BodyOp::NewBody,
        participants: Vec::new(),
    };
    let e = add(&mut doc, FeatureBox::new(e));
    ok(&doc, e);
    let mut p = pattern(PatternObjects::Features(vec![e]), along_x("2", "30 mm"));
    p.compute = ComputeType::Adjust;
    let p = add(&mut doc, FeatureBox::new(p));
    let Health::Error(d) = health(&doc, p) else {
        panic!("{:?}", health(&doc, p))
    };
    assert!(d.message.contains("has none"), "{d:?}");
}

#[test]
fn adjust_along_a_path_with_path_direction() {
    // A seed hole cut "All" through a plate, patterned along a straight path with adjust:
    // the same holes as identical compute.
    let mut doc = Document::new("adjust-path");
    let plate = abox(&mut doc, [-10.0, -10.0, 0.0], [120.0, 20.0, 5.0]);
    let mut sk = SketchFeature::new(PlaneInput::OriginXY);
    let line = sk
        .edit_sketch(|s| s.add_line(Point2::new(0.0, 0.0), Point2::new(100.0, 0.0)))
        .unwrap();
    let path_sketch = add(&mut doc, FeatureBox::new(sk));
    let mut c = SketchFeature::new(PlaneInput::OriginXY);
    c.edit_sketch(|s| s.add_circle(Point2::new(0.0, 0.0), 2.0).map(|_| ()))
        .unwrap();
    let c = add(&mut doc, FeatureBox::new(c));
    let mut e = Extrude::distance(
        profiles(&doc, c),
        "1 mm",
        OpTarget {
            op: BodyOp::Cut,
            participants: vec![plate],
        },
    );
    e.side1.extent = Extent::All;
    let e = add(&mut doc, FeatureBox::new(e));
    ok(&doc, e);
    let mut p = pattern(
        PatternObjects::Features(vec![e]),
        path_kind(
            vec![curve_ref(path_sketch, u64::from(line.0))],
            "5",
            Spacing::Extent(ex("100 mm")),
            "0",
        ),
    );
    p.compute = ComputeType::Adjust;
    let p = add(&mut doc, FeatureBox::new(p));
    ok(&doc, p);
    let want = 120.0 * 20.0 * 5.0 - 5.0 * std::f64::consts::PI * 4.0 * 5.0;
    assert_rel(volume(&brep(&doc, plate)), want, 1e-9);
}
