//! SK-10 acceptance: sketch text as text blocks of locked spline outlines. The fixture fonts
//! are ASCII subsets of Noto Sans (SIL OFL 1.1, `tests/fixtures/fonts/OFL.txt`), plain files
//! (not git-LFS), so the CI test job has them.

#![allow(
    clippy::unwrap_used,
    clippy::float_cmp,
    reason = "test helpers; a failure is a test failure"
)]

use std::f64::consts::{FRAC_PI_2, PI};
use std::path::PathBuf;

use amg_core::{Plane, Point2};
use amg_sketch::solve::build_system;
use amg_sketch::text::{Font, FontSet, PathPlacement, Placement, TextBlock, TextError, TextSpec};
use amg_sketch::{EntityKind, ProfileOptions, ProfileSet, Sketch, detect_profiles};

fn font(name: &str) -> Font {
    let path: PathBuf = [
        env!("CARGO_MANIFEST_DIR"),
        "tests",
        "fixtures",
        "fonts",
        name,
    ]
    .iter()
    .collect();
    Font::from_bytes(std::fs::read(path).unwrap()).unwrap()
}

fn family() -> FontSet {
    FontSet {
        regular: font("NotoSans-Regular-ascii.ttf"),
        bold: Some(font("NotoSans-Bold-ascii.ttf")),
        italic: Some(font("NotoSans-Italic-ascii.ttf")),
        bold_italic: None,
    }
}

fn free(x: f64, y: f64, angle: f64) -> Placement {
    Placement::Free {
        anchor: Point2::new(x, y),
        angle,
    }
}

fn profiles(s: &Sketch) -> ProfileSet {
    detect_profiles(s, &ProfileOptions::default())
}

/// Net area of the solid faces of `b`.
fn area(s: &Sketch, b: &TextBlock) -> f64 {
    let p = profiles(s);
    b.regions(&p)
        .solid
        .iter()
        .map(|&id| {
            let p = p.get(id).unwrap();
            p.outer.signed_area() + p.holes.iter().map(|h| h.signed_area()).sum::<f64>()
        })
        .sum()
}

/// Bounding box of the solid faces of `b`.
fn bbox(s: &Sketch, b: &TextBlock) -> (Point2, Point2) {
    let p = profiles(s);
    let mut lo = Point2::splat(f64::MAX);
    let mut hi = Point2::splat(f64::MIN);
    for pr in b.regions(&p).solid.iter().map(|&id| p.get(id).unwrap()) {
        let b = pr.outer.bbox();
        lo = lo.min(b.min);
        hi = hi.max(b.max);
    }
    (lo, hi)
}

/// (solid faces, holes of the solid faces, counter faces) of the block.
fn regions(s: &Sketch, b: &TextBlock) -> (usize, usize, usize) {
    let p = profiles(s);
    let r = b.regions(&p);
    let h = r
        .solid
        .iter()
        .map(|&id| p.get(id).unwrap().holes.len())
        .sum();
    (r.solid.len(), h, r.counters.len())
}

fn text(s: &mut Sketch, set: &FontSet, spec: TextSpec, at: &Placement) -> TextBlock {
    TextBlock::create(s, set, spec, at).unwrap()
}

#[test]
fn agb_has_three_outer_profiles_and_four_holes() {
    let set = family();
    let mut s = Sketch::new(Plane::XY);
    let b = text(
        &mut s,
        &set,
        TextSpec::new("AgB", 10.0, "Noto Sans"),
        &free(0.0, 0.0, 0.0),
    );
    let p = profiles(&s);
    // 3 glyph faces with 4 holes (A 1, g 1, B 2); the arrangement lists each hole's inside
    // as a face too.
    assert_eq!(regions(&s, &b), (3, 4, 4));
    assert_eq!(p.profiles.len(), 7);
    // Golden numbers (Noto Sans Regular subset, 10 mm cap height): net area of the three
    // glyph faces, outline entities.
    assert!(
        (area(&s, &b) - 97.654_143_357).abs() < 1e-6,
        "{}",
        area(&s, &b)
    );
    assert_eq!((b.curves.len(), b.points.len()), (74, 119));
    assert!(p.issues.is_empty(), "{:?}", p.issues);
    assert!(p.open_chains.is_empty());
    assert!(!b.curves.is_empty() && b.points.len() > b.curves.len());
}

#[test]
fn glyph_and_hole_counts_of_other_strings() {
    let set = family();
    for (t, outer, holes_) in [
        ("I", 1, 0),
        ("O", 1, 1),
        ("8", 1, 2),
        ("i", 2, 0),
        ("Hello", 5, 2), // H e l l o: e and o have a counter
        ("A B", 2, 3),   // the space adds no outline
        ("0123456789", 10, 6),
    ] {
        let mut s = Sketch::new(Plane::XY);
        let b = text(
            &mut s,
            &set,
            TextSpec::new(t, 10.0, "Noto Sans"),
            &free(0.0, 0.0, 0.0),
        );
        let (solid, h, counters) = regions(&s, &b);
        assert_eq!((solid, h, counters), (outer, holes_, holes_), "{t:?}");
    }
}

#[test]
fn height_is_the_cap_height() {
    let set = family();
    let mut s = Sketch::new(Plane::XY);
    let b = text(
        &mut s,
        &set,
        TextSpec::new("HEI", 12.5, "Noto Sans"),
        &free(3.0, -4.0, 0.0),
    );
    let (lo, hi) = bbox(&s, &b);
    assert!((lo.y - -4.0).abs() < 1e-9, "baseline {}", lo.y);
    assert!((hi.y - (-4.0 + 12.5)).abs() < 1e-9, "cap height {}", hi.y);
    assert!(lo.x > 3.0 && lo.x < 3.0 + 2.0, "left bearing {}", lo.x);
}

#[test]
fn outline_is_locked_and_invisible_to_the_solver() {
    let set = family();
    let mut s = Sketch::new(Plane::XY);
    let b = text(
        &mut s,
        &set,
        TextSpec::new("AgB", 10.0, "Noto Sans"),
        &free(0.0, 0.0, 0.0),
    );
    for id in &b.points {
        let EntityKind::Point { x, y } = s.entity(*id).unwrap().kind else {
            panic!("outline point expected");
        };
        assert!(s.params().flags(x).locked && s.params().flags(y).locked);
    }
    for id in &b.curves {
        let e = s.entity(*id).unwrap();
        assert!(!e.construction);
        assert_eq!(e.origin, amg_sketch::EntityOrigin::Imported);
    }
    // Anchor and direction stay free: 2 points = 4 unknowns.
    let sys = build_system(&s).unwrap();
    let free_params = (0..s.params().len())
        .filter(|&i| !s.params().flags(amg_sketch::ParamId(i as u32)).locked)
        .count();
    assert_eq!(free_params, 4);
    let _ = sys;
}

#[test]
fn moving_the_anchor_rotates_the_outline_rigidly() {
    let set = family();
    let mut s = Sketch::new(Plane::XY);
    let b = text(
        &mut s,
        &set,
        TextSpec::new("AgB", 10.0, "Noto Sans"),
        &free(0.0, 0.0, 0.0),
    );
    let (area0, (lo0, hi0)) = (area(&s, &b), bbox(&s, &b));
    let ids: Vec<_> = s.entities().map(|(id, _)| id).collect();

    // Rotate the baseline by 90 degrees about (5, 5) by moving the two free points.
    let set_pt = |s: &mut Sketch, id, p: Point2| {
        let EntityKind::Point { x, y } = s.entity(id).unwrap().kind else {
            panic!();
        };
        s.params_mut().set(x, p.x);
        s.params_mut().set(y, p.y);
    };
    set_pt(&mut s, b.anchor, Point2::new(5.0, 5.0));
    set_pt(&mut s, b.direction, Point2::new(5.0, 15.0));
    b.update(&mut s, &set).unwrap();

    assert_eq!(
        ids,
        s.entities().map(|(id, _)| id).collect::<Vec<_>>(),
        "ids are stable"
    );
    assert_eq!(regions(&s, &b), (3, 4, 4));
    assert!((area(&s, &b) - area0).abs() < 1e-9 * area0);
    let (lo1, hi1) = bbox(&s, &b);
    // (x, y) -> (5 - y, 5 + x): the width becomes the height.
    assert!(((hi1.y - lo1.y) - (hi0.x - lo0.x)).abs() < 1e-9);
    assert!(((hi1.x - lo1.x) - (hi0.y - lo0.y)).abs() < 1e-9);
    assert!((lo1.y - (5.0 + lo0.x)).abs() < 1e-9);

    // A new height rescales in place.
    let mut b2 = b.clone();
    b2.spec.height = 20.0;
    b2.update(&mut s, &set).unwrap();
    let (lo2, hi2) = bbox(&s, &b2);
    assert!(((hi2.y - lo2.y) - 2.0 * (hi0.x - lo0.x)).abs() < 1e-9);
}

#[test]
fn bold_italic_and_synthetic_italic() {
    let set = family();
    let build = |bold, italic, set: &FontSet| {
        let mut s = Sketch::new(Plane::XY);
        let mut spec = TextSpec::new("AgB", 10.0, "Noto Sans");
        spec.bold = bold;
        spec.italic = italic;
        TextBlock::create(&mut s, set, spec, &free(0.0, 0.0, 0.0)).map(|b| (s, b))
    };
    let (ps, pb) = build(false, false, &set).unwrap();
    let (plain_area, plain_x) = (area(&ps, &pb), bbox(&ps, &pb).1.x);

    let (s, b) = build(true, false, &set).unwrap();
    assert_eq!(regions(&s, &b), (3, 4, 4));
    assert!(area(&s, &b) > 1.1 * plain_area, "bold is heavier");

    let (s, b) = build(false, true, &set).unwrap();
    assert_eq!(regions(&s, &b), (3, 4, 4));

    // No italic face: the regular face sheared by tan 12 degrees; area is preserved.
    let regular_only = FontSet::new(font("NotoSans-Regular-ascii.ttf"));
    let (s, b) = build(false, true, &regular_only).unwrap();
    assert_eq!(regions(&s, &b), (3, 4, 4));
    assert!((area(&s, &b) - plain_area).abs() < 1e-9 * plain_area);
    assert!(bbox(&s, &b).1.x > plain_x + 1.0, "slanted to the right");
    // No bold face: an error.
    assert_eq!(
        build(true, false, &regular_only).unwrap_err(),
        TextError::StyleUnavailable("bold")
    );
    // Bold italic from the bold face, sheared.
    let (s, b) = build(true, true, &set).unwrap();
    assert_eq!(regions(&s, &b), (3, 4, 4));
}

#[test]
fn multi_line_text_stacks_downwards() {
    let set = family();
    let mut s = Sketch::new(Plane::XY);
    let mut spec = TextSpec::new("I\nI", 10.0, "Noto Sans");
    spec.line_spacing = 1.5;
    let b = text(&mut s, &set, spec, &free(0.0, 0.0, 0.0));
    let p = profiles(&s);
    let r = b.regions(&p);
    assert_eq!((r.solid.len(), r.counters.len()), (2, 0));
    let ys: Vec<f64> = r
        .solid
        .iter()
        .map(|&id| p.get(id).unwrap().outer.bbox().min.y)
        .collect();
    let pitch = (ys[0] - ys[1]).abs();
    let expect = 1.5 * set.regular.line_height() * 10.0 / set.regular.cap_height();
    assert!((pitch - expect).abs() < 1e-9, "{pitch} vs {expect}");
}

#[test]
fn text_along_a_circle() {
    let set = family();
    let mut s = Sketch::new(Plane::XY);
    let circle = s.add_circle(Point2::new(0.0, 0.0), 40.0).unwrap();
    s.set_construction(circle, true).unwrap();
    let place = PathPlacement {
        curve: circle,
        offset: 5.0,
        reversed: false,
    };
    let b = text(
        &mut s,
        &set,
        TextSpec::new("AgB", 10.0, "Noto Sans"),
        &Placement::AlongPath(place),
    );
    assert_eq!(regions(&s, &b), (3, 4, 4));
    // Up is to the left of the path direction, i.e. towards the centre of a CCW circle: the
    // caps reach in by about the cap height, the descender of g sticks out by about 3.5 mm.
    let mut r_min = f64::MAX;
    let mut r_max = 0.0_f64;
    for id in &b.points {
        let pt = s.point(*id).unwrap();
        r_min = r_min.min(pt.length());
        r_max = r_max.max(pt.length());
    }
    assert!(
        (29.0..31.0).contains(&r_min) && (40.0..44.5).contains(&r_max),
        "{r_min} {r_max}"
    );
    // The anchor sits at arc length 5 on the circle: angle 5 / 40 rad.
    let a = s.point(b.anchor).unwrap();
    assert!((a.x - 40.0 * (5.0_f64 / 40.0).cos()).abs() < 1e-6);
    assert!((a.y - 40.0 * (5.0_f64 / 40.0).sin()).abs() < 1e-6);
    // The path text is ruled by the path: the anchor is locked.
    let EntityKind::Point { x, .. } = s.entity(b.anchor).unwrap().kind else {
        panic!();
    };
    assert!(s.params().flags(x).locked);

    // Moving the path (here: a bigger radius) and updating re-lays the text.
    let EntityKind::Circle { radius, .. } = s.entity(circle).unwrap().kind else {
        panic!();
    };
    s.params_mut().set(radius, 60.0);
    b.update(&mut s, &set).unwrap();
    let r_min2 = b
        .points
        .iter()
        .map(|id| s.point(*id).unwrap().length())
        .fold(f64::MAX, f64::min);
    assert!((49.0..51.0).contains(&r_min2), "{r_min2}");
}

#[test]
fn text_along_a_line_matches_free_text() {
    let set = family();
    let mut s1 = Sketch::new(Plane::XY);
    let line = s1
        .add_line(Point2::new(0.0, 0.0), Point2::new(0.0, 100.0))
        .unwrap();
    s1.set_construction(line, true).unwrap();
    // Vertical line: text rotated by 90 degrees; a single glyph is unchanged by the
    // centring rule, so compare the bbox of "H" with free text.
    let mut s2 = Sketch::new(Plane::XY);
    let spec = TextSpec::new("H", 10.0, "Noto Sans");
    let b1 = text(
        &mut s1,
        &set,
        spec.clone(),
        &Placement::AlongPath(PathPlacement {
            curve: line,
            offset: 2.0,
            reversed: false,
        }),
    );
    let b2 = text(&mut s2, &set, spec, &free(0.0, 2.0, FRAC_PI_2));
    let (a, b) = (bbox(&s1, &b1), bbox(&s2, &b2));
    for (p, q) in [(a.0, b.0), (a.1, b.1)] {
        assert!(p.distance(q) < 1e-9, "{p:?} {q:?}");
    }
}

#[test]
fn errors_leave_the_sketch_untouched() {
    let set = family();
    let mut s = Sketch::new(Plane::XY);
    let line = s.add_line(Point2::ZERO, Point2::new(10.0, 0.0)).unwrap();
    let snapshot = s.clone();
    let spec = |t: &str, h: f64| TextSpec::new(t, h, "Noto Sans");
    let err = |s: &mut Sketch, sp, at: &Placement| TextBlock::create(s, &set, sp, at).unwrap_err();

    assert_eq!(
        err(&mut s, spec("Aé", 10.0), &free(0.0, 0.0, 0.0)),
        TextError::MissingGlyph('é')
    );
    assert_eq!(
        err(&mut s, spec("  ", 10.0), &free(0.0, 0.0, 0.0)),
        TextError::Empty
    );
    assert!(matches!(
        err(&mut s, spec("A", -1.0), &free(0.0, 0.0, 0.0)),
        TextError::Sketch(_)
    ));
    let on_line = |off: f64| {
        Placement::AlongPath(PathPlacement {
            curve: line,
            offset: off,
            reversed: false,
        })
    };
    assert!(matches!(
        err(&mut s, spec("AgB AgB", 10.0), &on_line(0.0)),
        TextError::PathTooShort { available, .. } if (available - 10.0).abs() < 1e-12
    ));
    assert!(matches!(
        err(&mut s, spec("A\nB", 1.0), &on_line(0.0)),
        TextError::BadPath(_)
    ));
    assert!(matches!(
        err(
            &mut s,
            spec("A", 1.0),
            &Placement::AlongPath(PathPlacement {
                curve: amg_sketch::EntityId(999),
                offset: 0.0,
                reversed: false
            })
        ),
        TextError::BadPath(_)
    ));
    assert_eq!(s.entities().count(), snapshot.entities().count());
    assert_eq!(s.params().len(), snapshot.params().len());
    let _ = PI;
}

#[test]
fn block_serialises_and_removes() {
    let set = family();
    let mut s = Sketch::new(Plane::XY);
    let keep = s.add_line(Point2::ZERO, Point2::new(1.0, 1.0)).unwrap();
    let n0 = s.entities().count();
    let b = text(
        &mut s,
        &set,
        TextSpec::new("AgB", 10.0, "Noto Sans"),
        &free(0.0, 0.0, 0.0),
    );
    let json = serde_json::to_string(&b).unwrap();
    let back: TextBlock = serde_json::from_str(&json).unwrap();
    assert_eq!(back, b);
    // The sketch with its text round-trips as well.
    let sk: Sketch = serde_json::from_str(&serde_json::to_string(&s).unwrap()).unwrap();
    assert_eq!(regions(&sk, &b), (3, 4, 4));

    b.remove(&mut s).unwrap();
    assert_eq!(s.entities().count(), n0);
    assert!(s.entity(keep).is_some());
}

#[test]
fn fixture_fonts_are_parsed() {
    let f = font("NotoSans-Bold-ascii.ttf");
    assert_eq!(f.family(), "Noto Sans");
    assert!(f.is_bold() && !f.is_italic());
    assert!(font("NotoSans-Italic-ascii.ttf").is_italic());
    assert!(Font::from_bytes(vec![0u8; 16]).is_err());
    assert!(f.glyph('g').unwrap().contours.len() == 2);
    assert!(f.glyph('\u{4e2d}').is_none());
}
