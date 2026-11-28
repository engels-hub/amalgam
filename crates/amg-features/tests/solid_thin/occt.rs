//! The OCCT half of the SOL-18 tests: goldens `tests/golden/solid/sol18_*.toml`, closed-form
//! volumes of the walls, naming across thickness and side edits, and failures.
//!
//! A wall's cross-section area is its thickness times the length of its mid line. Moving the
//! mid line `d` to the left of a smooth chain that turns by `θ` (left positive) shortens it by
//! `θ·d`.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use amg_brep::{BrepKernel, CheckOpts, SubKind};
use amg_core::{Point2, Point3};
use amg_doc::naming::{NameSeg, NameSource, default_kernel, topo_ref_at};
use amg_doc::{BodyId, Health, MatchQuality, TopoKind, TopoName};
use amg_features::sketch::{PlaneInput, SketchFeature};
use amg_features::solid::{BodyOp, Direction, ExtentSide, Fillet};

use super::*;

const REL: f64 = 1e-9;
const T: f64 = 2.0;
const H: f64 = 10.0;
const SIDES: [(&str, ThinSide); 3] = [
    ("side1", ThinSide::Side1),
    ("side2", ThinSide::Side2),
    ("center", ThinSide::Center),
];

/// The mid line's offset to the left of the chain.
fn shift(side: ThinSide, t: f64) -> f64 {
    match side {
        ThinSide::Side1 => 0.5 * t,
        ThinSide::Side2 => -0.5 * t,
        ThinSide::Center => 0.0,
    }
}

#[derive(Clone, Copy)]
enum Seg {
    Line([f64; 2], [f64; 2]),
    /// Centre, start, end (counter-clockwise).
    Arc([f64; 2], [f64; 2], [f64; 2]),
}

fn p(a: [f64; 2]) -> Point2 {
    Point2::new(a[0], a[1])
}

fn sketch_on(doc: &mut Document, plane: PlaneInput, segs: &[Seg]) -> (FeatureId, Vec<EntityId>) {
    let mut f = SketchFeature::new(plane);
    let ids = f
        .edit_sketch(|s| {
            segs.iter()
                .map(|seg| match *seg {
                    Seg::Line(a, b) => s.add_line(p(a), p(b)),
                    Seg::Arc(c, a, b) => s.add_arc(p(c), p(a), p(b)),
                })
                .collect::<Result<Vec<_>, _>>()
        })
        .unwrap();
    (add(doc, FeatureBox::new(f)), ids)
}

/// A line of 10 and a tangent quarter arc of radius 5 turning left: length `10 + 5π/2`.
const LINE_ARC: [Seg; 2] = [
    Seg::Line([0.0, 0.0], [10.0, 0.0]),
    Seg::Arc([10.0, 5.0], [10.0, 0.0], [15.0, 5.0]),
];
const LINE_ARC_L: f64 = 10.0 + 5.0 * FRAC_PI_2;

/// `LINE_ARC` followed by a quarter arc turning right (drawn against the chain), so the total
/// turn is zero: length `10 + 5π`.
const S_CHAIN: [Seg; 3] = [
    Seg::Line([0.0, 0.0], [10.0, 0.0]),
    Seg::Arc([10.0, 5.0], [10.0, 0.0], [15.0, 5.0]),
    Seg::Arc([20.0, 5.0], [20.0, 10.0], [15.0, 5.0]),
];
const S_CHAIN_L: f64 = 10.0 + 10.0 * FRAC_PI_2;

fn curves(sketch: FeatureId, ids: &[EntityId]) -> Vec<EntityRef> {
    ids.iter().map(|id| curve_ref(sketch, *id)).collect()
}

fn thin_extrude(refs: Vec<EntityRef>, t: f64, side: ThinSide) -> Extrude {
    let mut e = Extrude::distance(refs, &format!("{H} mm"), OpTarget::default());
    e.thin = thin(&format!("{t} mm"), side);
    e
}

#[track_caller]
fn solid(doc: &Document, f: FeatureId) -> amg_brep::Shape {
    assert_eq!(health(doc, f), Health::Ok, "feature {f:?}");
    let s = brep(doc, body0(f));
    assert!(kernel().check(&s, &CheckOpts::default()).valid, "valid");
    assert_eq!(s.solids().len(), 1, "one solid");
    s
}

#[track_caller]
fn built(doc: &mut Document, f: impl Feature + 'static) -> (FeatureId, amg_brep::Shape) {
    let f = add(doc, FeatureBox::new(f));
    let s = solid(doc, f);
    (f, s)
}

#[track_caller]
fn golden(case: &str, s: &amg_brep::Shape) {
    check_golden_as("sol18", REL, case, s);
}

#[track_caller]
fn error(doc: &Document, f: FeatureId) -> String {
    match health(doc, f) {
        Health::Error(d) => d.message,
        other => panic!("expected an error, got {other:?}"),
    }
}

/// `(role tag, role index, source)` of every face, sorted.
fn face_roles(doc: &Document, body: BodyId) -> Vec<(String, u32, NameSource)> {
    let b = doc.evaluated().body(body).unwrap();
    let naming = b.naming.as_ref().unwrap();
    let shape = brep(doc, body);
    let mut v: Vec<_> = (0..shape.count(SubKind::Face))
        .map(|i| {
            let name = naming.name_of(TopoKind::Face, i).unwrap();
            match name.origin() {
                Some(NameSeg::Gen {
                    role,
                    from: Some(from),
                    ..
                }) => (role.tag.to_string(), role.index, from.clone()),
                other => panic!("face {i}: {other:?}"),
            }
        })
        .collect();
    v.sort();
    v
}

fn count(roles: &[(String, u32, NameSource)], tag: &str, index: u32) -> usize {
    roles.iter().filter(|r| r.0 == tag && r.1 == index).count()
}

fn all_names(doc: &Document, body: BodyId) -> Vec<(TopoKind, TopoName)> {
    let b = doc.evaluated().body(body).unwrap();
    let naming = b.naming.as_ref().unwrap();
    let shape = brep(doc, body);
    let mut v = Vec::new();
    for (kind, sub) in [
        (TopoKind::Face, SubKind::Face),
        (TopoKind::Edge, SubKind::Edge),
        (TopoKind::Vertex, SubKind::Vertex),
    ] {
        for i in 0..shape.count(sub) {
            v.push((kind, naming.name_of(kind, i).unwrap().clone()));
        }
    }
    v.sort();
    v
}

/// A reference to every face, edge and vertex of `body`, with the index it names now.
fn all_refs(doc: &Document, body: BodyId) -> Vec<amg_doc::TopoRef> {
    let shape = brep(doc, body);
    let mut v = Vec::new();
    for (kind, sub) in [
        (TopoKind::Face, SubKind::Face),
        (TopoKind::Edge, SubKind::Edge),
        (TopoKind::Vertex, SubKind::Vertex),
    ] {
        for i in 0..shape.count(sub) {
            v.push(topo_ref_at(&**doc.evaluated(), default_kernel(), body, kind, i).unwrap());
        }
    }
    v
}

#[track_caller]
fn assert_all_exact(doc: &Document, refs: &[amg_doc::TopoRef], what: &str) {
    for r in refs {
        let res = doc
            .evaluated()
            .resolve(r)
            .unwrap_or_else(|e| panic!("{what}: {e:?} for {r:?}"));
        assert_eq!(res.quality, MatchQuality::Exact, "{what}: {r:?}");
    }
}

// ---- Done-when 1: open chains -----------------------------------------------------------------

#[test]
fn dw1_a_line_arc_chain_in_each_side_mode() {
    for (label, side) in SIDES {
        let mut doc = Document::new("dw1");
        let (s, ids) = sketch_on(&mut doc, PlaneInput::OriginXY, &LINE_ARC);
        let (e, shape) = built(&mut doc, thin_extrude(curves(s, &ids), T, side));
        // L·t·h in the centre mode; a side mode's mid line is t/2 off the sketched chain.
        let want = T * H * (LINE_ARC_L - FRAC_PI_2 * shift(side, T));
        let v = volume(&shape);
        println!(
            "line+arc {label}: volume {v:.12}, want {want:.12}, rel {:.2e}",
            (v - want).abs() / want
        );
        assert_rel(v, want, REL);
        assert_eq!(faces(&shape), 8, "{label}");
        golden(&format!("line_arc_{label}"), &shape);

        let roles = face_roles(&doc, body0(e));
        for (tag, index, n) in [
            ("ThinInner", 0, 2),
            ("ThinOuter", 0, 2),
            ("ThinEnd", 0, 1),
            ("ThinEnd", 1, 1),
            ("StartCap", 0, 1),
            ("EndCap", 0, 1),
        ] {
            assert_eq!(
                count(&roles, tag, index),
                n,
                "{label} {tag}#{index}: {roles:?}"
            );
        }
        for (tag, _, from) in &roles {
            let ok = match tag.as_str() {
                "ThinInner" | "ThinOuter" => matches!(from, NameSource::SketchCurve { .. }),
                "ThinEnd" => matches!(from, NameSource::SketchPoint { .. }),
                _ => matches!(from, NameSource::Profile { outer, .. } if outer.len() == 2),
            };
            assert!(ok, "{tag}: {from:?}");
        }
    }
}

#[test]
fn dw1_chains_with_no_net_turn_give_exactly_length_thickness_height() {
    let line = [Seg::Line([0.0, 0.0], [12.0, 5.0])];
    let chains: [(&str, &[Seg], f64); 2] = [("s", &S_CHAIN, S_CHAIN_L), ("line", &line, 13.0)];
    for (name, segs, length) in chains {
        for (label, side) in SIDES {
            let mut doc = Document::new("dw1-s");
            let (s, ids) = sketch_on(&mut doc, PlaneInput::OriginXY, segs);
            let (_, shape) = built(&mut doc, thin_extrude(curves(s, &ids), T, side));
            let want = length * T * H;
            let v = volume(&shape);
            println!(
                "{name} {label}: volume {v:.12}, L·t·h {want:.12}, rel {:.2e}",
                (v - want).abs() / want
            );
            assert_rel(v, want, REL);
            golden(&format!("{name}_{label}"), &shape);
        }
    }
}

#[test]
fn dw1_the_sides_lie_where_they_say() {
    // Side 1 is left of the chain (+y for a line along +x), Side 2 right of it.
    for (side, y0, y1) in [
        (ThinSide::Side1, 0.0, 2.0),
        (ThinSide::Side2, -2.0, 0.0),
        (ThinSide::Center, -1.0, 1.0),
    ] {
        let mut doc = Document::new("sides");
        let (s, ids) = sketch_on(
            &mut doc,
            PlaneInput::OriginXY,
            &[Seg::Line([0.0, 0.0], [10.0, 0.0])],
        );
        let (_, shape) = built(&mut doc, thin_extrude(curves(s, &ids), T, side));
        let b = shape.bbox();
        assert!(
            (b.min.y - y0).abs() < 1e-6 && (b.max.y - y1).abs() < 1e-6,
            "{side:?} {b:?}"
        );
        assert!(
            b.min.x.abs() < 1e-6 && (b.max.x - 10.0).abs() < 1e-6,
            "{b:?}"
        );
        assert!(b.min.z.abs() < 1e-6 && (b.max.z - H).abs() < 1e-6, "{b:?}");
    }
}

#[test]
fn the_names_do_not_depend_on_the_pick_order() {
    let mut doc = Document::new("order");
    let (s, ids) = sketch_on(&mut doc, PlaneInput::OriginXY, &S_CHAIN);
    let (a, _) = built(&mut doc, thin_extrude(curves(s, &ids), T, ThinSide::Side1));
    let before = all_names(&doc, body0(a));
    let mut reversed = ids.clone();
    reversed.reverse();
    edit(
        &mut doc,
        a,
        FeatureBox::new(thin_extrude(curves(s, &reversed), T, ThinSide::Side1)),
    );
    solid(&doc, a);
    assert_eq!(all_names(&doc, body0(a)), before);
}

// ---- Done-when 2: closed chains ---------------------------------------------------------------

/// `(outer area − inner area)` of the wall of a `w × d` rectangle.
fn rect_ring(w: f64, d: f64, side: ThinSide, t: f64) -> f64 {
    let (inner, outer) = match side {
        ThinSide::Side1 => (t, 0.0),
        ThinSide::Side2 => (0.0, t),
        ThinSide::Center => (0.5 * t, 0.5 * t),
    };
    (w + 2.0 * outer) * (d + 2.0 * outer) - (w - 2.0 * inner) * (d - 2.0 * inner)
}

#[test]
fn dw2_a_closed_rectangle_gives_a_ring() {
    for (label, side) in SIDES {
        // From the region, and from its four curves picked clockwise.
        let mut doc = Document::new("dw2");
        let s = add(
            &mut doc,
            FeatureBox::new(rect_sketch(&[[0.0, 0.0, 20.0, 10.0]])),
        );
        let region = profiles(&doc, s);
        let (e, shape) = built(&mut doc, thin_extrude(region, T, side));
        let want = rect_ring(20.0, 10.0, side, T) * H;
        let v = volume(&shape);
        println!(
            "rect {label}: volume {v:.12}, want {want:.12}, rel {:.2e}",
            (v - want).abs() / want
        );
        assert_rel(v, want, REL);
        assert_eq!(faces(&shape), 10, "{label}");
        golden(&format!("rect_{label}"), &shape);
        let roles = face_roles(&doc, body0(e));
        assert_eq!(count(&roles, "ThinInner", 0), 4, "{roles:?}");
        assert_eq!(count(&roles, "ThinOuter", 0), 4, "{roles:?}");
        assert!(roles.iter().all(|r| r.0 != "ThinEnd"));

        let mut doc = Document::new("dw2-curves");
        let (f, _, lines) = polygon_sketch(&[[0.0, 0.0], [0.0, 10.0], [20.0, 10.0], [20.0, 0.0]]);
        let s = add(&mut doc, FeatureBox::new(f));
        let (e, shape) = built(&mut doc, thin_extrude(curves(s, &lines), T, side));
        assert_rel(volume(&shape), want, REL);
        assert_eq!(faces(&shape), 10, "{label}");
        // Inside is inner whichever way the curves were drawn.
        let b = shape.bbox();
        let grow = if side == ThinSide::Side1 {
            0.0
        } else if side == ThinSide::Side2 {
            T
        } else {
            0.5 * T
        };
        assert!((b.min.x + grow).abs() < 1e-6, "{label} {b:?}");
        let roles = face_roles(&doc, body0(e));
        assert_eq!(count(&roles, "ThinInner", 0), 4, "{roles:?}");
    }
}

#[test]
fn dw2_circles_holes_and_shared_edges() {
    // A lone circle.
    for (label, side) in SIDES {
        let mut doc = Document::new("circle");
        let mut f = SketchFeature::new(PlaneInput::OriginXY);
        let c = f
            .edit_sketch(|s| s.add_circle(Point2::new(3.0, 4.0), 8.0))
            .unwrap();
        let s = add(&mut doc, FeatureBox::new(f));
        let ro = 8.0 - shift(side, T) + 0.5 * T;
        let want = PI * (ro * ro - (ro - T) * (ro - T)) * H;
        let from_curve = thin_extrude(vec![curve_ref(s, c)], T, side);
        let (_, shape) = built(&mut doc, from_curve);
        assert_rel(volume(&shape), want, REL);
        assert_eq!(faces(&shape), 4, "{label}");
        golden(&format!("circle_{label}"), &shape);
        let from_region = thin_extrude(profiles(&doc, s), T, side);
        let (_, shape) = built(&mut doc, from_region);
        assert_rel(volume(&shape), want, REL);
    }
    // A region with a hole: one ring per loop, each widened toward its own inside.
    let mut doc = Document::new("holes");
    let mut f = rect_sketch(&[[0.0, 0.0, 40.0, 30.0]]);
    f.edit_sketch(|s| s.add_circle(Point2::new(20.0, 15.0), 6.0))
        .unwrap();
    let s = add(&mut doc, FeatureBox::new(f));
    let outer_region = profiles(&doc, s)
        .into_iter()
        .find(|r| {
            let EntityRef::Profile { profile, .. } = r else {
                return false;
            };
            let g = doc.evaluated().sketch(SketchId(s)).unwrap();
            g.profiles
                .get(amg_sketch::ProfileId(profile.0))
                .is_some_and(|p| !p.holes.is_empty())
        })
        .unwrap();
    let e = add(
        &mut doc,
        FeatureBox::new(thin_extrude(vec![outer_region], T, ThinSide::Side1)),
    );
    assert_eq!(health(&doc, e), Health::Ok);
    let bodies: Vec<BodyId> = doc.evaluated().bodies().map(|(id, _)| id).collect();
    assert_eq!(bodies.len(), 2, "two separate rings");
    let total: f64 = bodies.iter().map(|b| volume(&brep(&doc, *b))).sum();
    let want = (rect_ring(40.0, 30.0, ThinSide::Side1, T) + PI * (36.0 - 16.0)) * H;
    assert_rel(total, want, REL);

    // Two regions sharing an edge: the walls fuse, the shared wall counted on both sides.
    let mut doc = Document::new("shared");
    let mut f = rect_sketch(&[[0.0, 0.0, 20.0, 10.0]]);
    f.edit_sketch(|s| s.add_line(Point2::new(8.0, 0.0), Point2::new(8.0, 10.0)))
        .unwrap();
    let s = add(&mut doc, FeatureBox::new(f));
    let two = profiles(&doc, s);
    assert_eq!(two.len(), 2);
    let (_, shape) = built(&mut doc, thin_extrude(two, T, ThinSide::Center));
    // The 22 × 12 outline minus the two holes 6 × 8 and 10 × 8.
    let want = (22.0 * 12.0 - 6.0 * 8.0 - 10.0 * 8.0) * H;
    assert_rel(volume(&shape), want, REL);
    golden("shared_edge", &shape);
}

// ---- Done-when 3: revolve ---------------------------------------------------------------------

/// A wall in XZ: a line up from (20, 0) to (20, 10) and a tangent quarter arc of radius 5
/// turning toward the axis.
const PROFILE_XZ: [Seg; 2] = [
    Seg::Line([20.0, 0.0], [20.0, 10.0]),
    Seg::Arc([15.0, 10.0], [20.0, 10.0], [15.0, 15.0]),
];

/// Area and first moment about the axis of the wall of `PROFILE_XZ` (left is toward the axis).
fn wall_moments(side: ThinSide, t: f64) -> (f64, f64) {
    let (l, r) = match side {
        ThinSide::Side1 => (t, 0.0),
        ThinSide::Side2 => (0.0, t),
        ThinSide::Center => (0.5 * t, 0.5 * t),
    };
    // The straight part: x in [20 − l, 20 + r], z in [0, 10].
    let line_area = (l + r) * 10.0;
    let line_moment = line_area * (20.0 + 0.5 * (r - l));
    // The quarter annulus about (15, 10) between radii 5 − l and 5 + r.
    let (ri, ro): (f64, f64) = (5.0 - l, 5.0 + r);
    let arc_area = 0.5 * FRAC_PI_2 * (ro * ro - ri * ri);
    let arc_moment = 15.0 * arc_area + (ro.powi(3) - ri.powi(3)) / 3.0;
    (line_area + arc_area, line_moment + arc_moment)
}

fn thin_revolve(refs: Vec<EntityRef>, t: f64, side: ThinSide, angle: &str) -> Revolve {
    let mut r = Revolve::angle(
        refs,
        RefInput::Origin(OriginEntity::ZAxis),
        angle,
        OpTarget::default(),
    );
    r.thin = thin(&format!("{t} mm"), side);
    r
}

#[test]
fn dw3_a_thin_revolve_of_an_open_chain_matches_pappus() {
    for (label, side) in SIDES {
        let (area, moment) = wall_moments(side, T);
        for (angle, theta) in [("90 deg", FRAC_PI_2), ("360 deg", TAU)] {
            let mut doc = Document::new("dw3");
            let (s, ids) = sketch_on(&mut doc, PlaneInput::OriginXZ, &PROFILE_XZ);
            let (r, shape) = built(&mut doc, thin_revolve(curves(s, &ids), T, side, angle));
            // Pappus: area × centroid radius × angle = first moment × angle.
            let want = moment * theta;
            let v = volume(&shape);
            println!(
                "revolve {label} {angle}: area {area:.9}, volume {v:.12}, want {want:.12}, \
                 rel {:.2e}",
                (v - want).abs() / want
            );
            assert_rel(v, want, REL);
            let full = theta == TAU;
            assert_eq!(faces(&shape), if full { 6 } else { 8 }, "{label} {angle}");
            golden(
                &format!("revolve_{label}_{}", if full { "full" } else { "90" }),
                &shape,
            );
            let roles = face_roles(&doc, body0(r));
            assert_eq!(count(&roles, "ThinInner", 0), 2, "{roles:?}");
            assert_eq!(count(&roles, "ThinOuter", 0), 2, "{roles:?}");
            assert_eq!(count(&roles, "ThinEnd", 0), 1, "{roles:?}");
            assert_eq!(count(&roles, "ThinEnd", 1), 1, "{roles:?}");
            assert_eq!(count(&roles, "StartCap", 0), usize::from(!full));
            assert_eq!(count(&roles, "EndCap", 0), usize::from(!full));
        }
    }
}

#[test]
fn two_sided_thin_features_number_their_second_side() {
    // Revolve 30° one way and 60° the other.
    let mut doc = Document::new("two-rev");
    let (s, ids) = sketch_on(&mut doc, PlaneInput::OriginXZ, &PROFILE_XZ);
    let mut r = thin_revolve(curves(s, &ids), T, ThinSide::Side2, "30 deg");
    r.direction = Direction::TwoSides;
    r.side2 = Some(ExtentSide::distance(ex("60 deg")));
    let (r, shape) = built(&mut doc, r);
    let (_, moment) = wall_moments(ThinSide::Side2, T);
    assert_rel(volume(&shape), moment * FRAC_PI_2, REL);
    // The coplanar side faces of the two sides merge under side 0; each side keeps its cap.
    let roles = face_roles(&doc, body0(r));
    assert_eq!(count(&roles, "EndCap", 0), 1, "{roles:?}");
    assert_eq!(count(&roles, "EndCap", 1), 1, "{roles:?}");
    assert_eq!(count(&roles, "ThinInner", 0), 2, "{roles:?}");

    // Extrude 10 up and 4 down.
    let mut doc = Document::new("two-ext");
    let (s, ids) = sketch_on(&mut doc, PlaneInput::OriginXY, &LINE_ARC);
    let mut e = thin_extrude(curves(s, &ids), T, ThinSide::Center);
    e.direction = Direction::TwoSides;
    e.side2 = Some(ExtentSide::distance(ex("4 mm")));
    let (e, shape) = built(&mut doc, e);
    assert_rel(volume(&shape), LINE_ARC_L * T * 14.0, REL);
    assert_eq!(faces(&shape), 8);
    let roles = face_roles(&doc, body0(e));
    assert_eq!(count(&roles, "ThinEnd", 0), 1, "{roles:?}");
    assert_eq!(count(&roles, "ThinEnd", 1), 1, "{roles:?}");
    golden("two_sides", &shape);

    // Symmetric.
    let mut sym = thin_extrude(curves(s, &ids), T, ThinSide::Center);
    sym.direction = Direction::Symmetric {
        whole_length: false,
    };
    let (_, shape) = built(&mut doc, sym);
    assert_rel(volume(&shape), LINE_ARC_L * T * H, REL);
    let b = shape.bbox();
    assert!(
        (b.min.z + 5.0).abs() < 1e-6 && (b.max.z - 5.0).abs() < 1e-6,
        "{b:?}"
    );
}

#[test]
fn taper_flip_and_cut_work_on_walls() {
    let mut doc = Document::new("cut");
    let (_, plate) = extrude_rects(
        &mut doc,
        &[[-5.0, -5.0, 25.0, 15.0]],
        H,
        BodyOp::NewBody,
        vec![],
    );
    let (s, ids) = sketch_on(&mut doc, PlaneInput::OriginXY, &LINE_ARC);
    let mut e = thin_extrude(curves(s, &ids), T, ThinSide::Center);
    e.target = OpTarget {
        op: BodyOp::Cut,
        participants: vec![body0(plate)],
    };
    let cut = add(&mut doc, FeatureBox::new(e));
    assert_eq!(health(&doc, cut), Health::Ok);
    let slot = brep(&doc, body0(plate));
    assert_rel(volume(&slot), 30.0 * 20.0 * H - LINE_ARC_L * T * H, REL);

    let mut flipped = thin_extrude(curves(s, &ids), T, ThinSide::Side1);
    flipped.flip = true;
    let (_, shape) = built(&mut doc, flipped);
    let b = shape.bbox();
    assert!((b.min.z + H).abs() < 1e-6 && b.max.z.abs() < 1e-6, "{b:?}");
    // Flipping the extrude doesn't move the wall to the other side.
    assert!(b.min.y.abs() < 1e-6, "{b:?}");

    let mut tapered = thin_extrude(vec![curve_ref(s, ids[0])], T, ThinSide::Center);
    tapered.side1.taper = ex("3 deg");
    let (_, shape) = built(&mut doc, tapered);
    // Every side face leans out: (10 + 2kz)(2 + 2kz) integrated over the height.
    let k = 3f64.to_radians().tan();
    let want = 20.0 * H + 12.0 * k * H * H + 4.0 / 3.0 * k * k * H.powi(3);
    assert_rel(volume(&shape), want, REL);
}

// ---- Done-when 4: names survive edits ---------------------------------------------------------

/// A thin feature at a thickness and side.
type Rebuild = Box<dyn Fn(f64, ThinSide) -> FeatureBox>;

#[test]
fn dw4_editing_the_thickness_keeps_all_names() {
    let mut doc = Document::new("dw4");
    let (s, ids) = sketch_on(&mut doc, PlaneInput::OriginXY, &S_CHAIN);
    let rect = add(
        &mut doc,
        FeatureBox::new(rect_sketch(&[[30.0, 0.0, 50.0, 10.0]])),
    );
    let (xz, xz_ids) = sketch_on(&mut doc, PlaneInput::OriginXZ, &PROFILE_XZ);
    let region = profiles(&doc, rect);
    let cases: [(&str, Rebuild); 3] = [
        (
            "open extrude",
            Box::new(move |t, side| FeatureBox::new(thin_extrude(curves(s, &ids), t, side))),
        ),
        (
            "closed extrude",
            Box::new(move |t, side| FeatureBox::new(thin_extrude(region.clone(), t, side))),
        ),
        (
            "open revolve",
            Box::new(move |t, side| {
                FeatureBox::new(thin_revolve(curves(xz, &xz_ids), t, side, "120 deg"))
            }),
        ),
    ];
    for (what, feature) in cases {
        let f = add(&mut doc, feature(T, ThinSide::Side1));
        solid(&doc, f);
        let names = all_names(&doc, body0(f));
        let refs = all_refs(&doc, body0(f));
        for (t, side) in [
            (3.0, ThinSide::Side1),
            (0.5, ThinSide::Side1),
            (T, ThinSide::Side2),
            (1.5, ThinSide::Center),
        ] {
            edit(&mut doc, f, feature(t, side));
            solid(&doc, f);
            assert_eq!(all_names(&doc, body0(f)), names, "{what} {t} {side:?}");
            assert_all_exact(&doc, &refs, what);
        }
        println!("{what}: {} references Exact after 4 edits", refs.len());
    }
}

#[test]
fn dw4_a_downstream_fillet_stays_exact() {
    let mut doc = Document::new("dw4-fillet");
    let (s, ids) = sketch_on(&mut doc, PlaneInput::OriginXY, &LINE_ARC);
    let (e, shape) = built(&mut doc, thin_extrude(curves(s, &ids), T, ThinSide::Side1));
    // The upright edge of the start end on the sketched chain.
    let at = Point3::new(0.0, 0.0, 0.5 * H);
    let edge = (0..shape.count(SubKind::Edge))
        .map(|i| {
            topo_ref_at(
                &**doc.evaluated(),
                default_kernel(),
                body0(e),
                TopoKind::Edge,
                i,
            )
            .unwrap()
        })
        .enumerate()
        .find(|(i, _)| {
            let g = kernel()
                .sub_geom(
                    &shape,
                    amg_brep::SubShape {
                        kind: SubKind::Edge,
                        index: u32::try_from(*i).unwrap(),
                    },
                )
                .unwrap();
            (g.centroid - at).length() < 1e-6
        })
        .map(|(_, mut r)| {
            r.key = 1;
            r
        })
        .unwrap();
    let fillet = add(
        &mut doc,
        FeatureBox::new(Fillet::constant(vec![edge], "0.2 mm")),
    );
    assert_eq!(health(&doc, fillet), Health::Ok);
    for t in [3.0, 1.0] {
        edit(
            &mut doc,
            e,
            FeatureBox::new(thin_extrude(curves(s, &ids), t, ThinSide::Side1)),
        );
        assert_eq!(health(&doc, fillet), Health::Ok, "{t}");
        let rec = &doc.evaluated().node(fillet).unwrap().resolutions[0];
        let res = rec.result.as_ref().expect("resolved");
        assert_eq!(res[0].quality, MatchQuality::Exact, "{t}: {rec:?}");
    }
}

/// The length of the referenced curves (for a region, of its outer loop), by chords.
fn chain_length(doc: &Document, refs: &[EntityRef]) -> f64 {
    const N: u32 = 200_000;
    let of = |c: &amg_sketch::Curve2| {
        let (t0, t1) = c.param_range();
        let at = |i: u32| c.eval(t0 + (t1 - t0) * f64::from(i) / f64::from(N));
        (0..N).map(|i| (at(i + 1) - at(i)).length()).sum::<f64>()
    };
    refs.iter()
        .map(|r| match r {
            EntityRef::SketchCurve { sketch, curve } => {
                let g = doc.evaluated().sketch(*sketch).unwrap();
                let id = EntityId(u32::try_from(curve.0).unwrap());
                of(&g.sketch.curve(id).unwrap())
            }
            EntityRef::Profile {
                sketch, profile, ..
            } => {
                let g = doc.evaluated().sketch(*sketch).unwrap();
                let p = g.profiles.get(amg_sketch::ProfileId(profile.0)).unwrap();
                p.outer.edges.iter().map(|e| of(&e.curve)).sum()
            }
            other => panic!("{other:?}"),
        })
        .sum()
}

#[test]
fn dw4_spline_and_ellipse_walls_keep_their_names() {
    let mut doc = Document::new("dw4-spline");
    // A line, then an S-shaped fit spline from its end.
    let mut f = SketchFeature::new(PlaneInput::OriginXY);
    let ids = f
        .edit_sketch(|s| {
            let line = s.add_line(p([0.0, 0.0]), p([10.0, 0.0]))?;
            let spline = s.add_spline_fit(
                &[
                    p([10.0, 0.0]),
                    p([20.0, 4.0]),
                    p([30.0, -4.0]),
                    p([40.0, 0.0]),
                ],
                Some(amg_core::Vec2::new(10.0, 0.0)),
                None,
            )?;
            Ok(vec![line, spline])
        })
        .unwrap();
    let open = add(&mut doc, FeatureBox::new(f));
    let mut f = SketchFeature::new(PlaneInput::OriginXY);
    f.edit_sketch(|s| s.add_ellipse(p([0.0, 30.0]), p([15.0, 30.0]), 9.0))
        .unwrap();
    let ellipse = add(&mut doc, FeatureBox::new(f));
    let open_refs = curves(open, &ids);
    let ellipse_refs = profiles(&doc, ellipse);
    {
        let g = doc.evaluated().sketch(SketchId(open)).unwrap();
        for id in &ids {
            let c = g.sketch.curve(*id).unwrap();
            let (t0, t1) = c.param_range();
            println!(
                "derivative {:?} {:?} at {:?} {:?}",
                c.derivative(t0),
                c.derivative(t1),
                c.eval(t0),
                c.eval(t1)
            );
        }
        let g = doc.evaluated().sketch(SketchId(ellipse)).unwrap();
        println!("ellipse {:?}", g.profiles.profiles[0].outer);
    }
    for (what, refs) in [("spline", open_refs), ("ellipse", ellipse_refs)] {
        let f = add(
            &mut doc,
            FeatureBox::new(thin_extrude(refs.clone(), 1.0, ThinSide::Center)),
        );
        let shape = solid(&doc, f);
        // The centred wall of a smooth chain has its thickness times the chain's length.
        // Checked on the caps' area: `mass_props` integrates the volume of solids with
        // free-form faces only roughly (3 % off on the closed ellipse wall).
        let length = chain_length(&doc, &refs);
        let caps: Vec<f64> = (0..faces(&shape))
            .filter(|i| {
                let g = kernel()
                    .sub_geom(
                        &shape,
                        amg_brep::SubShape {
                            kind: SubKind::Face,
                            index: *i,
                        },
                    )
                    .unwrap();
                g.bbox.max.z - g.bbox.min.z < 1e-6
            })
            .map(|i| shape.face_area(amg_brep::FaceId(i)))
            .collect();
        assert_eq!(caps.len(), 2, "{what}: flat caps");
        for area in caps {
            println!(
                "{what}: cap area {area:.9}, L·t {length:.9}, rel {:.2e}",
                (area - length).abs() / length
            );
            // The spline's cap integral is the looser of the two.
            assert_rel(area, length, 2e-4);
        }
        // One face per curve and boundary: the kernel rejoins the pieces of a spline offset.
        assert_eq!(
            faces(&shape),
            if what == "spline" { 8 } else { 4 },
            "{what}"
        );
        let b = shape.bbox();
        assert!(b.min.z.abs() < 1e-6 && (b.max.z - H).abs() < 1e-6, "{b:?}");
        let names = all_names(&doc, body0(f));
        let topo = all_refs(&doc, body0(f));
        for (t, side) in [
            (2.0, ThinSide::Center),
            (0.4, ThinSide::Side1),
            (1.5, ThinSide::Side2),
        ] {
            edit(
                &mut doc,
                f,
                FeatureBox::new(thin_extrude(refs.clone(), t, side)),
            );
            solid(&doc, f);
            assert_eq!(all_names(&doc, body0(f)), names, "{what} {t} {side:?}");
            assert_all_exact(&doc, &topo, what);
        }
    }
}

// ---- failures ---------------------------------------------------------------------------------

#[test]
fn bad_selections_and_thicknesses_fail_cleanly() {
    let mut doc = Document::new("errors");
    let (s, ids) = sketch_on(
        &mut doc,
        PlaneInput::OriginXY,
        &[
            Seg::Line([0.0, 0.0], [10.0, 0.0]),
            Seg::Line([0.0, 0.0], [0.0, 10.0]),
            Seg::Line([0.0, 0.0], [-10.0, 0.0]),
        ],
    );
    let check = |doc: &mut Document, f: Extrude, what: &str| {
        let f = add(doc, FeatureBox::new(f));
        let message = error(doc, f);
        assert!(message.contains(what), "{what}: {message}");
    };
    let one = vec![curve_ref(s, ids[0])];
    check(
        &mut doc,
        thin_extrude(curves(s, &ids), T, ThinSide::Side1),
        "branch",
    );
    for t in [0.0, -1.0] {
        check(
            &mut doc,
            thin_extrude(one.clone(), t, ThinSide::Side1),
            "thickness must be positive",
        );
    }
    check(
        &mut doc,
        thin_extrude(vec![curve_ref(s, EntityId(9999))], T, ThinSide::Side1),
        "no longer",
    );
    check(
        &mut doc,
        Extrude::distance(one, "10 mm", OpTarget::default()),
        "sketch curves need Thin",
    );
    // Wider than the circle it grows into.
    let mut f = SketchFeature::new(PlaneInput::OriginXY);
    let c = f
        .edit_sketch(|s| s.add_circle(Point2::new(0.0, 0.0), 1.0))
        .unwrap();
    let circle = add(&mut doc, FeatureBox::new(f));
    check(
        &mut doc,
        thin_extrude(vec![curve_ref(circle, c)], T, ThinSide::Side1),
        "radius",
    );
    // A planar face is not a thin profile.
    let (_, plate) = extrude_rects(
        &mut doc,
        &[[0.0, 0.0, 5.0, 5.0]],
        1.0,
        BodyOp::NewBody,
        vec![],
    );
    let face = topo_ref_at(
        &**doc.evaluated(),
        default_kernel(),
        body0(plate),
        TopoKind::Face,
        0,
    )
    .unwrap();
    check(
        &mut doc,
        thin_extrude(vec![EntityRef::Topo(face)], T, ThinSide::Side1),
        "sketch regions or sketch curves",
    );
}

#[test]
fn a_thin_wall_across_the_axis_fails_naming_it() {
    let mut doc = Document::new("axis");
    let (s, ids) = sketch_on(
        &mut doc,
        PlaneInput::OriginXZ,
        &[Seg::Line([-5.0, 0.0], [5.0, 10.0])],
    );
    let r = add(
        &mut doc,
        FeatureBox::new(thin_revolve(curves(s, &ids), T, ThinSide::Center, "90 deg")),
    );
    let message = error(&doc, r);
    assert!(message.contains("crosses the axis"), "{message}");
}
