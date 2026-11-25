//! UI-13 sketch creation tools, kittest on the real app with the `NullViewport`.
//!
//! - Done-when 1: for every tool, the committed sketch equals the SK-21 builder (or SK-22 /
//!   `Sketch::add_*`) output for the same picks, and typed heads-up values create driving
//!   dimensions with those values.
//! - Done-when 2: every tool runs from the toolbar, from the S-key search and from its
//!   shortcut where Fusion has one.
//! - Done-when 3: Esc mid-tool leaves the sketch unchanged.

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use amg_core::{Point2, Point3};
use amg_render::{Camera, Projection};
use amg_sketch::{
    ConstraintKind, DimensionKind, End, EntityId, EntityKind, HvTarget, Sketch, build,
};
use amg_ui::command::CmdId;
use amg_ui::design::SKETCH_CONTEXT;
use amg_ui::sketch::{
    CONSTRUCTION_CMD, CREATE_SKETCH_CMD, CreateKind, LINE_CMD, SKETCH_SELECT_CMD, SketchLayer,
    SketchSession,
};
use amg_ui::testing::AppHarness;
use amg_ui::testing::egui_kittest::kittest::Queryable as _;

const PRIM: egui::PointerButton = egui::PointerButton::Primary;
const NONE: egui::Modifiers = egui::Modifiers::NONE;
/// Screen picks come back through an f32 camera ray.
const EPS: f64 = 1e-3;

fn pt(x: f64, y: f64) -> Point2 {
    Point2::new(x, y)
}

fn session(h: &AppHarness) -> &SketchSession {
    h.app()
        .doc_layer::<SketchLayer>()
        .expect("sketch layer")
        .session()
        .expect("an open sketch session")
}

fn sketch(h: &AppHarness) -> Sketch {
    session(h).working().sketch.clone()
}

fn json(s: &Sketch) -> serde_json::Value {
    serde_json::to_value(s).expect("sketch json")
}

fn applied(h: &AppHarness) -> usize {
    h.document().history().stack().applied()
}

/// A new XY sketch seen from the top.
fn open() -> AppHarness {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    h.run_command(CREATE_SKETCH_CMD);
    h.kittest().get_by_label("XY").click();
    h.drive_until_idle();
    h.app_mut().active_document_mut().camera = Camera {
        eye: Point3::new(20.0, 15.0, 150.0),
        target: Point3::new(20.0, 15.0, 0.0),
        up: amg_core::Dir3::Y,
        projection: Projection::Perspective { fov_y: 0.8 },
    };
    h.drive_until_idle();
    h
}

fn screen(h: &AppHarness, p: Point2) -> egui::Pos2 {
    h.world_to_screen(Point3::new(p.x, p.y, 0.0))
}

fn hover(h: &mut AppHarness, p: Point2) {
    let s = screen(h, p);
    h.hover_at(s);
    h.drive_until_idle();
}

fn click_mods(h: &mut AppHarness, p: Point2, mods: egui::Modifiers) {
    let s = screen(h, p);
    h.click_at(s, PRIM, mods);
    h.drive_until_idle();
}

fn click(h: &mut AppHarness, p: Point2) {
    click_mods(h, p, NONE);
}

/// Hovers at `p`, types the heads-up fields (Tab between them) and presses Enter.
fn type_at(h: &mut AppHarness, p: Point2, fields: &[&str]) {
    hover(h, p);
    for (i, text) in fields.iter().enumerate() {
        if i > 0 {
            h.key(egui::Key::Tab);
        }
        h.kittest().event(egui::Event::Text((*text).to_owned()));
        h.kittest().step();
    }
    let hu = session(h).heads_up.clone().expect("heads-up input open");
    for (i, text) in fields.iter().enumerate() {
        assert_eq!(hu.fields[i], *text);
    }
    h.key(egui::Key::Enter);
    h.drive_until_idle();
}

fn start(h: &mut AppHarness, kind: CreateKind) {
    h.run_command(kind.def().id);
    assert_eq!(h.app().active_tool(), Some(kind.def().id), "{kind:?}");
}

/// Two lines from the origin: along x to (40, 0) and up to (0, 30).
fn two_lines(h: &mut AppHarness) -> (EntityId, EntityId) {
    for (a, b) in [(pt(0.0, 0.0), pt(40.0, 0.0)), (pt(0.0, 0.0), pt(0.0, 30.0))] {
        h.run_command(LINE_CMD);
        click(h, a);
        click(h, b);
        h.key(egui::Key::Escape);
        h.drive_until_idle();
    }
    let s = sketch(h);
    let mut l = s
        .entities()
        .filter(|(_, e)| matches!(e.kind, EntityKind::Line { .. }))
        .map(|(id, _)| id);
    (l.next().unwrap(), l.next().unwrap())
}

/// The status bar (the tool's hint) contains `text`.
fn status_shows(h: &mut AppHarness, text: &str) -> bool {
    h.kittest()
        .query_all_by_label_contains(text)
        .next()
        .is_some()
}

// ---- comparison with the builders -----------------------------------------------------------

/// The same entities (ids, kinds, construction), parameters and constraints.
fn assert_same(got: &Sketch, want: &Sketch, what: &str) {
    let ents = |s: &Sketch| {
        s.entities()
            .map(|(id, e)| (id, e.kind.clone(), e.construction))
            .collect::<Vec<_>>()
    };
    assert_eq!(ents(got), ents(want), "{what}: entities");
    let (a, b) = (got.params().as_slice(), want.params().as_slice());
    assert_eq!(a.len(), b.len(), "{what}: parameter count");
    for (i, (x, y)) in a.iter().zip(b).enumerate() {
        assert!((x - y).abs() < EPS, "{what}: parameter {i}: {x} vs {y}");
    }
    let cons = |s: &Sketch| {
        s.constraints()
            .map(|(id, c)| (id, c.kind.clone()))
            .collect::<Vec<_>>()
    };
    let (a, b) = (cons(got), cons(want));
    assert_eq!(a.len(), b.len(), "{what}: constraints {a:?} vs {b:?}");
    for ((ia, ka), (ib, kb)) in a.iter().zip(&b) {
        assert_eq!(ia, ib, "{what}");
        match (ka, kb) {
            (ConstraintKind::Dimension(x), ConstraintKind::Dimension(y)) => {
                assert_eq!((&x.kind, x.driving), (&y.kind, y.driving), "{what}");
                assert!((x.value - y.value).abs() < EPS, "{what}: {x:?} vs {y:?}");
            }
            _ => assert_eq!(ka, kb, "{what}"),
        }
    }
}

/// Runs `kind` with `picks` on the open sketch and compares with `builder` applied to the
/// sketch as it was.
fn check_builder(
    h: &mut AppHarness,
    kind: CreateKind,
    picks: &[Point2],
    builder: impl FnOnce(&mut Sketch),
) {
    let mut want = sketch(h);
    builder(&mut want);
    let before = applied(h);
    start(h, kind);
    assert_eq!(picks.len(), kind.picks(), "{kind:?}");
    for (i, p) in picks.iter().enumerate() {
        assert_eq!(applied(h), before, "{kind:?}: nothing before pick {i}");
        click(h, *p);
    }
    assert_eq!(applied(h), before + 1, "{kind:?}: one undo entry");
    assert_eq!(
        h.document().undo_label(),
        Some(kind.def().label),
        "{kind:?}"
    );
    assert_same(&sketch(h), &want, kind.def().label);
    assert_eq!(h.app().active_tool(), Some(kind.def().id), "stays active");
    let s = session(h);
    assert!(
        s.status().is_some_and(|s| s.is_converged()),
        "{kind:?}: {:?}",
        s.status()
    );
}

#[test]
fn rectangle_2pt_matches_the_builder() {
    let mut h = open();
    let (a, b) = (pt(2.0, 3.0), pt(32.0, 21.0));
    check_builder(&mut h, CreateKind::Rect2Point, &[a, b], |s| {
        build::rectangle_2pt(s, a, b).unwrap();
    });
}

#[test]
fn rectangle_3pt_matches_the_builder() {
    let mut h = open();
    let (a, b, c) = (pt(0.0, 0.0), pt(30.0, 10.0), pt(8.0, 24.0));
    check_builder(&mut h, CreateKind::Rect3Point, &[a, b, c], |s| {
        build::rectangle_3pt(s, a, b, c).unwrap();
    });
}

#[test]
fn rectangle_center_matches_the_builder() {
    let mut h = open();
    let (c, k) = (pt(15.0, 12.0), pt(31.0, 23.0));
    check_builder(&mut h, CreateKind::RectCenter, &[c, k], |s| {
        build::rectangle_center(s, c, k).unwrap();
    });
}

#[test]
fn circle_center_matches_the_sketch_output() {
    let mut h = open();
    let (c, p) = (pt(15.0, 12.0), pt(27.0, 21.0));
    check_builder(&mut h, CreateKind::CircleCenter, &[c, p], |s| {
        s.add_circle(c, c.distance(p)).unwrap();
    });
}

#[test]
fn circle_2pt_matches_the_sketch_output() {
    let mut h = open();
    let (a, b) = (pt(2.0, 3.0), pt(26.0, 17.0));
    check_builder(&mut h, CreateKind::Circle2Point, &[a, b], |s| {
        s.add_circle((a + b) * 0.5, a.distance(b) * 0.5).unwrap();
    });
}

#[test]
fn circle_3pt_matches_the_builder() {
    let mut h = open();
    let (a, b, c) = (pt(0.0, 0.0), pt(24.0, 4.0), pt(10.0, 21.0));
    check_builder(&mut h, CreateKind::Circle3Point, &[a, b, c], |s| {
        build::circle_3pt(s, a, b, c).unwrap();
    });
}

#[test]
fn circle_2_tangent_matches_the_builder() {
    let mut h = open();
    let (x, y) = two_lines(&mut h);
    let near = pt(8.0, 6.0);
    let picks = [pt(25.0, 0.0), pt(0.0, 19.0), near];
    // The radius is the mean distance of the third pick from the two lines.
    check_builder(&mut h, CreateKind::Circle2Tangent, &picks, |s| {
        build::circle_2_tangent(s, x, y, 7.0, near).unwrap();
    });
}

#[test]
fn arc_3pt_matches_the_builder() {
    let mut h = open();
    let (start, end, through) = (pt(0.0, 0.0), pt(30.0, 6.0), pt(13.0, 17.0));
    check_builder(&mut h, CreateKind::Arc3Point, &[start, end, through], |s| {
        build::arc_3pt(s, start, through, end).unwrap();
    });
}

#[test]
fn arc_center_matches_the_sketch_output_in_both_directions() {
    let (c, a, b) = (pt(12.0, 10.0), pt(30.0, 12.0), pt(14.0, 28.0));
    let mut h = open();
    check_builder(&mut h, CreateKind::ArcCenter, &[c, a, b], |s| {
        s.add_arc(c, a, b).unwrap();
    });
    // Turning clockwise: the arc runs from the end pick (on the start's radius) to the start.
    let mut h = open();
    let below = pt(14.0, -8.0);
    check_builder(&mut h, CreateKind::ArcCenter, &[c, a, below], |s| {
        let end = c + (below - c).normalize() * c.distance(a);
        s.add_arc(c, end, a).unwrap();
    });
}

#[test]
fn arc_tangent_matches_the_builder() {
    let mut h = open();
    let (x, _) = two_lines(&mut h);
    let (joint, end) = (pt(40.0, 0.0), pt(52.0, 14.0));
    check_builder(&mut h, CreateKind::ArcTangent, &[joint, end], |s| {
        build::arc_tangent(s, x, End::End, end).unwrap();
    });
}

#[test]
fn polygon_matches_the_builder_inscribed_and_circumscribed() {
    let (c, v) = (pt(15.0, 12.0), pt(31.0, 19.0));
    let mut h = open();
    check_builder(&mut h, CreateKind::Polygon, &[c, v], |s| {
        build::polygon(s, c, v, 6, true).unwrap();
    });
    let mut h = open();
    start(&mut h, CreateKind::Polygon);
    h.kittest().get_by_label("Circumscribed").click();
    h.drive_until_idle();
    let mut want = sketch(&h);
    build::polygon(&mut want, c, v, 6, false).unwrap();
    click(&mut h, c);
    click(&mut h, v);
    assert_same(&sketch(&h), &want, "circumscribed polygon");
}

#[test]
fn slot_center_to_center_matches_the_builder() {
    let mut h = open();
    let (a, b) = (pt(2.0, 4.0), pt(30.0, 14.0));
    let n = (b - a).normalize().perp();
    let w = b + n * 5.0;
    check_builder(&mut h, CreateKind::SlotCenterToCenter, &[a, b, w], |s| {
        build::slot_center_to_center(s, a, b, 10.0).unwrap();
    });
}

#[test]
fn slot_overall_matches_the_builder() {
    let mut h = open();
    let (a, b) = (pt(0.0, 2.0), pt(36.0, 16.0));
    let w = (a + b) * 0.5 + (b - a).normalize().perp() * 4.0;
    check_builder(&mut h, CreateKind::SlotOverall, &[a, b, w], |s| {
        build::slot_overall(s, a, b, 8.0).unwrap();
    });
}

#[test]
fn slot_center_point_matches_the_builder() {
    let mut h = open();
    let (c, e) = (pt(15.0, 12.0), pt(31.0, 19.0));
    let w = e + (e - c).normalize().perp() * 4.0;
    check_builder(&mut h, CreateKind::SlotCenterPoint, &[c, e, w], |s| {
        build::slot_center_point(s, c, e, 8.0).unwrap();
    });
}

#[test]
fn slot_arc_matches_the_builder() {
    let mut h = open();
    let (c, a, b) = (pt(4.0, 0.0), pt(34.0, 4.0), pt(10.0, 30.0));
    let w = c + amg_core::Vec2::from_angle(0.7) * (c.distance(a) + 4.0);
    check_builder(&mut h, CreateKind::SlotArc, &[c, a, b, w], |s| {
        build::slot_arc(s, c, a, b, 8.0).unwrap();
    });
}

#[test]
fn point_matches_the_sketch_output() {
    let mut h = open();
    let p = pt(7.0, 9.0);
    check_builder(&mut h, CreateKind::Point, &[p], |s| {
        s.add_point(p).unwrap();
    });
}

#[test]
fn ellipse_matches_the_sketch_output() {
    let mut h = open();
    let (c, m, p) = (pt(16.0, 12.0), pt(34.0, 18.0), pt(12.0, 22.0));
    check_builder(&mut h, CreateKind::Ellipse, &[c, m, p], |s| {
        // The minor radius that puts the third pick on the ellipse.
        let u = (m - c).normalize();
        let (x, y) = ((p - c).dot(u), (p - c).dot(u.perp()));
        let b = y.abs() / (1.0 - (x / c.distance(m)).powi(2)).sqrt();
        s.add_ellipse(c, m, b).unwrap();
    });
}

// ---- Done-when 1: typed values ------------------------------------------------------------

enum Step {
    Click(f64, f64),
    Type(f64, f64, &'static [&'static str]),
}
use Step::{Click as C, Type as T};

/// Runs `steps` of `kind` and returns the dimensions the shape created: (name, value).
fn typed(h: &mut AppHarness, kind: CreateKind, steps: &[Step]) -> Vec<(&'static str, f64)> {
    let old: Vec<_> = sketch(h).constraints().map(|(id, _)| id).collect();
    let before = applied(h);
    start(h, kind);
    for step in steps {
        match step {
            Step::Click(x, y) => click(h, pt(*x, *y)),
            Step::Type(x, y, f) => type_at(h, pt(*x, *y), f),
        }
    }
    assert_eq!(applied(h), before + 1, "{kind:?}: one undo entry");
    let s = session(h);
    assert!(
        s.status().is_some_and(|s| s.is_converged()),
        "{kind:?}: {:?}",
        s.status()
    );
    let mut dims = Vec::new();
    for (id, c) in s.working().sketch.constraints() {
        if let ConstraintKind::Dimension(d) = &c.kind
            && !old.contains(&id)
        {
            assert!(d.driving, "{kind:?}: {d:?}");
            assert!(s.working().dims.contains_key(&id), "{kind:?}: expression");
            dims.push((d.kind.name(), d.value));
        }
    }
    dims
}

fn expect(got: &[(&str, f64)], want: &[(&str, f64)], kind: CreateKind) {
    assert_eq!(got.len(), want.len(), "{kind:?}: {got:?}");
    for (g, w) in got.iter().zip(want) {
        assert_eq!(g.0, w.0, "{kind:?}: {got:?}");
        assert!((g.1 - w.1).abs() < 1e-9, "{kind:?}: {got:?}");
    }
}

fn typed_case(kind: CreateKind, steps: &[Step], want: &[(&str, f64)]) -> AppHarness {
    let mut h = open();
    if kind.needs_sketch() {
        two_lines(&mut h);
    }
    let got = typed(&mut h, kind, steps);
    expect(&got, want, kind);
    h
}

fn point_of(s: &Sketch, id: u32) -> Point2 {
    s.point(EntityId(id)).expect("a point")
}

#[test]
fn typed_rectangles() {
    let h = typed_case(
        CreateKind::Rect2Point,
        &[C(0.0, 0.0), T(10.0, 8.0, &["25", "12"])],
        &[("length", 25.0), ("length", 12.0)],
    );
    let s = sketch(&h);
    let far = s
        .entities()
        .filter_map(|(e, _)| s.point(e))
        .any(|p| p.distance(pt(25.0, 12.0)) < EPS);
    assert!(far, "the opposite corner is at the typed size");
    typed_case(
        CreateKind::Rect3Point,
        &[
            C(0.0, 0.0),
            T(20.0, 10.0, &["30", "20"]),
            T(2.0, 20.0, &["12"]),
        ],
        &[("length", 30.0), ("length", 12.0)],
    );
    typed_case(
        CreateKind::RectCenter,
        &[C(15.0, 10.0), T(20.0, 15.0, &["20", "10 mm"])],
        &[("length", 20.0), ("length", 10.0)],
    );
}

#[test]
fn typed_circles() {
    typed_case(
        CreateKind::CircleCenter,
        &[C(10.0, 10.0), T(20.0, 15.0, &["16"])],
        &[("diameter", 16.0)],
    );
    typed_case(
        CreateKind::Circle2Point,
        &[C(5.0, 5.0), T(20.0, 15.0, &["2*8"])],
        &[("diameter", 16.0)],
    );
    typed_case(
        CreateKind::Circle3Point,
        &[C(0.0, 0.0), C(20.0, 4.0), T(10.0, 18.0, &["30"])],
        &[("diameter", 30.0)],
    );
    // The builder's own radius dimension takes the typed value.
    typed_case(
        CreateKind::Circle2Tangent,
        &[C(25.0, 0.0), C(0.0, 19.0), T(8.0, 8.0, &["6"])],
        &[("radius", 6.0)],
    );
}

#[test]
fn typed_arcs() {
    typed_case(
        CreateKind::Arc3Point,
        &[
            C(0.0, 0.0),
            T(20.0, 10.0, &["25", "20"]),
            T(8.0, 15.0, &["18"]),
        ],
        &[("distance", 25.0), ("radius", 18.0)],
    );
    let h = typed_case(
        CreateKind::ArcCenter,
        &[
            C(10.0, 8.0),
            T(25.0, 10.0, &["20", "10"]),
            T(12.0, 28.0, &["25"]),
        ],
        &[("radius", 20.0), ("arc length", 25.0)],
    );
    let s = sketch(&h);
    let end = s
        .entities()
        .find_map(|(_, e)| match e.kind {
            EntityKind::Arc { end, .. } => s.point(end),
            _ => None,
        })
        .unwrap();
    let angle = 10f64.to_radians() + 25.0 / 20.0;
    assert!(
        end.distance(pt(10.0, 8.0) + amg_core::Vec2::from_angle(angle) * 20.0) < EPS,
        "{end:?}"
    );
    typed_case(
        CreateKind::ArcTangent,
        &[C(40.0, 0.0), T(50.0, 12.0, &["12"])],
        &[("radius", 12.0)],
    );
}

#[test]
fn typed_polygon_point_and_ellipse() {
    typed_case(
        CreateKind::Polygon,
        &[C(15.0, 12.0), T(30.0, 18.0, &["18", "10"])],
        &[("radius", 18.0)],
    );
    // A point has no size: typed values are its coordinates.
    let h = typed_case(CreateKind::Point, &[T(5.0, 5.0, &["12", "-7"])], &[]);
    assert!(point_of(&sketch(&h), 0).distance(pt(12.0, -7.0)) < 1e-9);
    typed_case(
        CreateKind::Ellipse,
        &[
            C(15.0, 10.0),
            T(30.0, 18.0, &["20", "25"]),
            T(10.0, 22.0, &["8"]),
        ],
        &[
            ("ellipse major radius", 20.0),
            ("ellipse minor radius", 8.0),
        ],
    );
}

#[test]
fn typed_slots() {
    typed_case(
        CreateKind::SlotCenterToCenter,
        &[
            C(2.0, 4.0),
            T(30.0, 14.0, &["30", "15"]),
            T(10.0, 20.0, &["8"]),
        ],
        &[("distance", 30.0), ("diameter", 8.0)],
    );
    let h = typed_case(
        CreateKind::SlotOverall,
        &[
            C(0.0, 2.0),
            T(30.0, 14.0, &["40", "15"]),
            T(10.0, 20.0, &["8"]),
        ],
        &[("length", 40.0), ("diameter", 8.0)],
    );
    // The overall length is carried by a construction line between the extreme points.
    let s = sketch(&h);
    let axis = s
        .entities()
        .filter(|(_, e)| e.construction && matches!(e.kind, EntityKind::Line { .. }))
        .count();
    assert_eq!(axis, 1);
    assert_eq!(
        session(&h).dof().map(|d| d.dof),
        Some(3),
        "5 - 2 dimensions"
    );
    typed_case(
        CreateKind::SlotCenterPoint,
        &[
            C(15.0, 10.0),
            T(30.0, 16.0, &["18", "15"]),
            T(10.0, 22.0, &["8"]),
        ],
        &[("distance", 18.0), ("diameter", 8.0)],
    );
    typed_case(
        CreateKind::SlotArc,
        &[
            C(4.0, 0.0),
            T(30.0, 4.0, &["25", "5"]),
            T(10.0, 28.0, &["30"]),
            T(25.0, 25.0, &["6"]),
        ],
        &[("radius", 25.0), ("arc length", 30.0), ("diameter", 6.0)],
    );
}

#[test]
fn a_value_without_a_shape_keeps_the_input_open() {
    let mut h = open();
    start(&mut h, CreateKind::Arc3Point);
    click(&mut h, pt(0.0, 0.0));
    click(&mut h, pt(30.0, 0.0));
    let before = applied(&h);
    // A radius below half the chord.
    type_at(&mut h, pt(15.0, 10.0), &["5"]);
    assert_eq!(applied(&h), before);
    assert_eq!(
        session(&h).heads_up.as_ref().map(|hu| hu.fields[0].clone()),
        Some("5".to_owned())
    );
    assert!(status_shows(&mut h, "no shape has these values"));
}

// ---- inference ------------------------------------------------------------------------------

fn count(s: &Sketch, f: impl Fn(&ConstraintKind) -> bool) -> usize {
    s.constraints().filter(|(_, c)| f(&c.kind)).count()
}

#[test]
fn picks_on_existing_geometry_are_constrained_and_ctrl_suppresses_it() {
    let mut h = open();
    two_lines(&mut h);
    let coincident = |h: &AppHarness| {
        count(&sketch(h), |k| {
            matches!(k, ConstraintKind::Coincident { .. })
        })
    };
    let before = coincident(&h);
    // A circle centred on a line end.
    start(&mut h, CreateKind::CircleCenter);
    click(&mut h, pt(40.2, 0.2));
    click(&mut h, pt(46.0, 5.0));
    assert_eq!(coincident(&h), before + 1);
    // The same with Ctrl adds nothing.
    click_mods(&mut h, pt(0.2, 30.2), egui::Modifiers::CTRL);
    click_mods(&mut h, pt(6.0, 34.0), egui::Modifiers::CTRL);
    assert_eq!(coincident(&h), before + 1);
    // A circle through an existing point: the point goes onto it.
    let on = |h: &AppHarness| {
        count(&sketch(h), |k| {
            matches!(k, ConstraintKind::PointOnCurve { .. })
        })
    };
    start(&mut h, CreateKind::Circle3Point);
    for p in [pt(40.0, 0.0), pt(30.0, 14.0), pt(18.0, 9.0)] {
        click(&mut h, p);
    }
    assert_eq!(on(&h), 1);
    assert!(session(&h).status().is_some_and(|s| s.is_converged()));
}

#[test]
fn a_horizontal_slot_axis_is_inferred() {
    let mut h = open();
    start(&mut h, CreateKind::SlotCenterToCenter);
    click(&mut h, pt(2.0, 8.0));
    hover(&mut h, pt(30.0, 8.3));
    assert_eq!(
        session(&h).inference.as_ref().and_then(|i| i.glyph()),
        Some("H")
    );
    click(&mut h, pt(30.0, 8.3));
    click(&mut h, pt(20.0, 14.0));
    let s = sketch(&h);
    assert_eq!(
        count(&s, |k| matches!(
            k,
            ConstraintKind::Horizontal(HvTarget::Points(..))
        )),
        1
    );
    assert_eq!(session(&h).dof().map(|d| d.dof), Some(4));
}

#[test]
fn the_dimension_kinds_of_a_typed_circle_measure_the_new_circle() {
    let h = typed_case(
        CreateKind::CircleCenter,
        &[C(10.0, 10.0), T(20.0, 15.0, &["16"])],
        &[("diameter", 16.0)],
    );
    let s = sketch(&h);
    let circle = s
        .entities()
        .find(|(_, e)| matches!(e.kind, EntityKind::Circle { .. }))
        .map(|(id, _)| id)
        .unwrap();
    assert_eq!(
        count(&s, |k| matches!(
            k,
            ConstraintKind::Dimension(d) if d.kind == DimensionKind::Diameter { curve: circle }
        )),
        1
    );
}

// ---- Done-when 2: reachability --------------------------------------------------------------

fn all_commands() -> Vec<(CmdId, &'static str)> {
    CreateKind::ALL
        .iter()
        .map(|k| (k.def().id, k.def().label))
        .collect()
}

fn settle(h: &mut AppHarness) {
    for _ in 0..4 {
        h.kittest().step();
    }
}

#[test]
fn every_tool_runs_from_the_toolbar() {
    let mut h = open();
    for (id, label) in all_commands() {
        h.run_command(SKETCH_SELECT_CMD);
        h.kittest().get_by_label("CREATE ⏷").click();
        settle(&mut h);
        // The dropdown item comes after a pinned button of the same command.
        let exact = h.kittest().query_all_by_label(label).last();
        match exact {
            Some(n) => n.click(),
            None => h
                .kittest()
                .query_all_by_label_contains(label)
                .last()
                .unwrap_or_else(|| panic!("{label} in the toolbar"))
                .click(),
        }
        h.drive_until_idle();
        assert_eq!(h.app().active_tool(), Some(id), "{label}");
    }
    // The construction toggle too.
    assert!(!session(&h).construction);
    h.kittest().get_by_label("CREATE ⏷").click();
    settle(&mut h);
    h.kittest()
        .get_by_label_contains("Normal / Construction")
        .click();
    h.drive_until_idle();
    assert!(session(&h).construction);
}

#[test]
fn every_tool_runs_from_the_s_key_search() {
    let mut h = open();
    let mut commands = all_commands();
    commands.push((CONSTRUCTION_CMD, "Normal / Construction"));
    for (id, label) in commands {
        h.run_command(SKETCH_SELECT_CMD);
        h.kittest().key_press(egui::Key::S);
        settle(&mut h);
        assert!(h.app().search().is_open(), "{label}");
        h.kittest().event(egui::Event::Text(label.to_owned()));
        settle(&mut h);
        assert_eq!(
            h.app().search().results().first().map(|r| r.id),
            Some(id),
            "{label}"
        );
        h.key(egui::Key::Enter);
        h.drive_until_idle();
        if id == CONSTRUCTION_CMD {
            assert!(session(&h).construction);
        } else {
            assert_eq!(h.app().active_tool(), Some(id), "{label}");
        }
    }
}

#[test]
fn fusion_shortcuts_r_c_and_x() {
    let mut h = open();
    let map = h.app().shortcuts();
    assert!(map.conflicts().is_empty(), "{:?}", map.conflicts());
    for (id, key) in [
        (CreateKind::Rect2Point.def().id, "R"),
        (CreateKind::CircleCenter.def().id, "C"),
        (CONSTRUCTION_CMD, "X"),
    ] {
        assert_eq!(map.display_of(id).as_deref(), Some(key));
    }
    h.key(egui::Key::R);
    h.drive_until_idle();
    assert_eq!(h.app().active_tool(), Some(CreateKind::Rect2Point.def().id));
    h.key(egui::Key::C);
    h.drive_until_idle();
    assert_eq!(
        h.app().active_tool(),
        Some(CreateKind::CircleCenter.def().id)
    );
    h.key(egui::Key::X);
    h.drive_until_idle();
    assert!(session(&h).construction);
    h.key(egui::Key::X);
    h.drive_until_idle();
    assert!(!session(&h).construction);
    // Outside a sketch the letters do nothing.
    h.run_command(amg_ui::sketch::FINISH_SKETCH_CMD);
    h.drive_until_idle();
    assert!(!h.app().active_document().contexts.contains(&SKETCH_CONTEXT));
    h.key(egui::Key::R);
    h.drive_until_idle();
    assert_eq!(h.app().active_tool(), None);
}

#[test]
fn construction_mode_draws_construction_geometry_and_x_toggles_the_selection() {
    let mut h = open();
    h.run_command(CONSTRUCTION_CMD);
    assert!(session(&h).construction);
    assert!(status_shows(&mut h, "· construction"));
    start(&mut h, CreateKind::Rect2Point);
    click(&mut h, pt(2.0, 3.0));
    click(&mut h, pt(22.0, 15.0));
    h.run_command(LINE_CMD);
    click(&mut h, pt(26.0, 3.0));
    click(&mut h, pt(36.0, 11.0));
    let curves = |h: &AppHarness| {
        sketch(h)
            .entities()
            .filter(|(_, e)| !matches!(e.kind, EntityKind::Point { .. }))
            .map(|(_, e)| e.construction)
            .collect::<Vec<_>>()
    };
    assert_eq!(curves(&h), vec![true; 5]);
    assert!(session(&h).profiles().profiles.is_empty());
    // Back to normal mode, then select the line (Esc twice: chain, tool) and press X.
    h.key(egui::Key::X);
    h.drive_until_idle();
    assert!(!session(&h).construction);
    for _ in 0..2 {
        h.key(egui::Key::Escape);
        h.drive_until_idle();
    }
    assert_eq!(h.app().active_tool(), Some(SKETCH_SELECT_CMD));
    click(&mut h, pt(31.0, 7.0));
    assert_eq!(session(&h).selected.len(), 1);
    let before = applied(&h);
    h.key(egui::Key::X);
    h.drive_until_idle();
    assert_eq!(applied(&h), before + 1);
    assert_eq!(curves(&h), vec![true, true, true, true, false]);
    h.key(egui::Key::X);
    h.drive_until_idle();
    assert_eq!(curves(&h), vec![true; 5]);
    assert!(!session(&h).construction, "the mode is unchanged");
}

// ---- Done-when 3: Esc -----------------------------------------------------------------------

#[test]
fn esc_mid_tool_leaves_the_sketch_unchanged() {
    let mut h = open();
    two_lines(&mut h);
    let reference = json(&sketch(&h));
    let before = applied(&h);
    for kind in CreateKind::ALL {
        start(&mut h, kind);
        let picks: [Point2; 3] = match kind {
            CreateKind::Circle2Tangent => [pt(25.0, 0.0), pt(0.0, 19.0), pt(8.0, 8.0)],
            CreateKind::ArcTangent => [pt(40.0, 0.0), pt(50.0, 12.0), pt(0.0, 0.0)],
            _ => [pt(8.0, 6.0), pt(28.0, 13.0), pt(14.0, 24.0)],
        };
        // Every pick but the last, then an open heads-up input.
        for p in picks.iter().take(kind.picks() - 1) {
            click(&mut h, *p);
        }
        hover(&mut h, pt(18.0, 20.0));
        if !kind.fields(kind.picks() - 1).is_empty() {
            h.kittest().event(egui::Event::Text("7".into()));
            h.kittest().step();
            assert!(session(&h).heads_up.is_some(), "{kind:?}");
            h.key(egui::Key::Escape);
            h.drive_until_idle();
            assert!(session(&h).heads_up.is_none(), "{kind:?}");
            assert_eq!(h.app().active_tool(), Some(kind.def().id), "{kind:?}");
        }
        if kind.picks() > 1 {
            h.key(egui::Key::Escape);
            h.drive_until_idle();
            assert_eq!(h.app().active_tool(), Some(kind.def().id), "{kind:?}");
            assert!(
                status_shows(&mut h, kind.step_hint(0)),
                "{kind:?}: the picks are gone"
            );
        }
        h.key(egui::Key::Escape);
        h.drive_until_idle();
        assert_eq!(h.app().active_tool(), Some(SKETCH_SELECT_CMD), "{kind:?}");
        assert_eq!(json(&sketch(&h)), reference, "{kind:?}");
        assert_eq!(applied(&h), before, "{kind:?}");
    }
}

#[test]
fn a_degenerate_pick_is_refused() {
    let mut h = open();
    start(&mut h, CreateKind::Rect2Point);
    let before = applied(&h);
    click(&mut h, pt(5.0, 5.0));
    // Zero height.
    click_mods(&mut h, pt(5.0, 5.0), egui::Modifiers::CTRL);
    assert_eq!(applied(&h), before);
    click(&mut h, pt(25.0, 17.0));
    assert_eq!(applied(&h), before + 1);
}

/// The framework's Tab fix also serves the Line tool: length, Tab, angle.
#[test]
fn tab_reaches_the_line_tools_angle_field() {
    let mut h = open();
    h.run_command(LINE_CMD);
    click(&mut h, pt(0.0, 0.0));
    type_at(&mut h, pt(10.0, 10.0), &["25", "30"]);
    let s = sketch(&h);
    let end = point_of(&s, 1);
    let want = amg_core::Vec2::from_angle(30f64.to_radians()) * 25.0;
    // The start point is a screen pick.
    assert!(end.distance(want) < EPS, "{end:?}");
}
