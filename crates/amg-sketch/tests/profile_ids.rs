//! SK-15: `ProfileId` stability under dimension edits, deterministic new ids and the intended
//! successor mapping of `ProfileSet::match_previous` after topology edits.
//!
//! Profiles are identified in fixtures by a probe point inside them, so the intent of every
//! case reads as "the region around P becomes the region around Q" (or is lost).

#![allow(
    clippy::unwrap_used,
    reason = "test helpers; a failure is a test failure"
)]

use amg_core::{Plane, Point2};
use amg_sketch::{
    EntityId, ProfileId, ProfileMatch, ProfileOptions, ProfileSet, Sketch, detect_profiles,
};

fn p(x: f64, y: f64) -> Point2 {
    Point2::new(x, y)
}

fn sk() -> Sketch {
    Sketch::new(Plane::XY)
}

fn rect(s: &mut Sketch, x0: f64, y0: f64, x1: f64, y1: f64) -> Vec<EntityId> {
    let pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)];
    let ids: Vec<EntityId> = pts
        .iter()
        .map(|&(x, y)| s.add_point(p(x, y)).unwrap())
        .collect();
    (0..4)
        .map(|i| s.add_line_between(ids[i], ids[(i + 1) % 4]).unwrap())
        .collect()
}

fn detect(s: &Sketch) -> ProfileSet {
    detect_profiles(s, &ProfileOptions::default())
}

fn id_at(set: &ProfileSet, x: f64, y: f64) -> ProfileId {
    set.at_point(p(x, y))
        .unwrap_or_else(|| panic!("no profile at ({x}, {y})"))
}

/// Rectangle `w x h` with a circular hole and a line crossing the rectangle and the hole.
fn scene(w: f64, h: f64, cx: f64, cy: f64, r: f64, dy: f64) -> Sketch {
    let mut s = sk();
    rect(&mut s, 0.0, 0.0, w, h);
    s.add_circle(p(cx, cy), r).unwrap();
    s.add_line(p(-5.0, cy + dy), p(w + 5.0, cy + dy)).unwrap();
    s
}

#[test]
fn twenty_dimension_edits_keep_every_id() {
    let base = detect(&scene(100.0, 60.0, 50.0, 25.0, 10.0, 4.0));
    assert!(base.profiles.len() >= 3, "{}", base.profiles.len());
    let ids: Vec<ProfileId> = base.profiles.iter().map(|p| p.id).collect();
    for k in 0..20 {
        let t = f64::from(k);
        let (w, h) = (100.0 + 3.0 * t, 60.0 + 2.0 * t);
        let (cx, cy) = (50.0 + t, 25.0 + 0.5 * t);
        let r = 10.0 + 0.2 * t;
        let dy = 4.0 + 0.1 * t;
        let set = detect(&scene(w, h, cx, cy, r, dy));
        let new_ids: Vec<ProfileId> = set.profiles.iter().map(|p| p.id).collect();
        assert_eq!(new_ids, ids, "edit {k}");
        let m = set.match_previous(&base);
        for id in &ids {
            assert_eq!(m[id], ProfileMatch::Same(*id), "edit {k}");
        }
    }
}

type Xy = (f64, f64);

/// One topology edit: the intent is a list of `(probe in old, expected probe in new)`;
/// `None` means the old region has no successor.
struct Case {
    name: &'static str,
    before: Sketch,
    after: Sketch,
    intent: Vec<(Xy, Option<Xy>)>,
    /// Old probes whose id must be unchanged (`Same`).
    same: Vec<Xy>,
}

fn cases() -> Vec<Case> {
    let mut v = Vec::new();

    // 1. A line crosses a rectangle: it becomes two pieces; the old region follows the larger.
    let mut b = sk();
    rect(&mut b, 0.0, 0.0, 100.0, 60.0);
    let mut a = b.clone();
    a.add_line(p(70.0, -5.0), p(70.0, 65.0)).unwrap();
    v.push(Case {
        name: "split by crossing line",
        before: b,
        after: a,
        intent: vec![((50.0, 30.0), Some((20.0, 30.0)))],
        same: vec![],
    });

    // 2. A hole is added.
    let mut b = sk();
    rect(&mut b, 0.0, 0.0, 100.0, 60.0);
    let mut a = b.clone();
    a.add_circle(p(50.0, 30.0), 10.0).unwrap();
    v.push(Case {
        name: "hole added",
        before: b,
        after: a,
        intent: vec![((10.0, 10.0), Some((10.0, 10.0)))],
        same: vec![],
    });

    // 3. A hole is removed; the inner disc is lost.
    let mut b = sk();
    rect(&mut b, 0.0, 0.0, 100.0, 60.0);
    b.add_circle(p(50.0, 30.0), 10.0).unwrap();
    let mut a = sk();
    rect(&mut a, 0.0, 0.0, 100.0, 60.0);
    v.push(Case {
        name: "hole removed",
        before: b,
        after: a,
        intent: vec![((10.0, 10.0), Some((10.0, 10.0))), ((50.0, 30.0), None)],
        same: vec![],
    });

    // 4. A disjoint rectangle is added: the old region keeps its id.
    let mut b = sk();
    rect(&mut b, 0.0, 0.0, 100.0, 60.0);
    let mut a = b.clone();
    rect(&mut a, 200.0, 0.0, 230.0, 20.0);
    v.push(Case {
        name: "disjoint curve added",
        before: b,
        after: a,
        intent: vec![((50.0, 30.0), Some((50.0, 30.0)))],
        same: vec![(50.0, 30.0)],
    });

    // 5. Outer curves deleted and redrawn (new entity ids, same place): fingerprint rescue.
    let mut b = sk();
    rect(&mut b, 0.0, 0.0, 100.0, 60.0);
    let mut a = sk();
    for _ in 0..8 {
        a.add_point(p(500.0, 500.0)).unwrap(); // burn ids so nothing is shared
    }
    rect(&mut a, 0.0, 0.0, 100.0, 60.0);
    v.push(Case {
        name: "redrawn outer loop",
        before: b,
        after: a,
        intent: vec![((50.0, 30.0), Some((50.0, 30.0)))],
        same: vec![],
    });

    // 6. Overlapping circles, then B moves away: crescent A becomes disc A, the lens is lost.
    let mk = |bx: f64| {
        let mut s = sk();
        s.add_circle(p(0.0, 0.0), 10.0).unwrap();
        s.add_circle(p(bx, 0.0), 10.0).unwrap();
        s
    };
    v.push(Case {
        name: "lens vanishes",
        before: mk(12.0),
        after: mk(21.0),
        intent: vec![
            ((-8.0, 0.0), Some((0.0, 0.0))),
            ((6.0, 0.0), None),
            ((20.0, 0.0), Some((21.0, 0.0))),
        ],
        same: vec![],
    });

    // 7. Two adjacent rectangles lose their divider: the larger old region takes the merge.
    let mut b = sk();
    rect(&mut b, 0.0, 0.0, 70.0, 60.0);
    rect(&mut b, 70.0, 0.0, 100.0, 60.0);
    let mut a = sk();
    rect(&mut a, 0.0, 0.0, 100.0, 60.0);
    v.push(Case {
        name: "divider deleted",
        before: b,
        after: a,
        intent: vec![((30.0, 30.0), Some((30.0, 30.0))), ((85.0, 30.0), None)],
        same: vec![],
    });

    // 8. A corner of a triangle is chopped by a new line: the old triangle follows the
    //    remaining large piece.
    let tri = |s: &mut Sketch| {
        let a = s.add_point(p(0.0, 0.0)).unwrap();
        let b = s.add_point(p(100.0, 0.0)).unwrap();
        let c = s.add_point(p(0.0, 100.0)).unwrap();
        s.add_line_between(a, b).unwrap();
        s.add_line_between(b, c).unwrap();
        s.add_line_between(c, a).unwrap();
    };
    let mut b = sk();
    tri(&mut b);
    let mut a = sk();
    tri(&mut a);
    a.add_line(p(90.0, -5.0), p(90.0, 15.0)).unwrap();
    v.push(Case {
        name: "corner chopped",
        before: b,
        after: a,
        intent: vec![((10.0, 10.0), Some((10.0, 10.0)))],
        same: vec![],
    });

    // 9. Annulus loses its inner circle: annulus -> disc, inner disc lost.
    let mut b = sk();
    b.add_circle(p(0.0, 0.0), 30.0).unwrap();
    b.add_circle(p(0.0, 0.0), 10.0).unwrap();
    let mut a = sk();
    a.add_circle(p(0.0, 0.0), 30.0).unwrap();
    v.push(Case {
        name: "inner circle deleted",
        before: b,
        after: a,
        intent: vec![((20.0, 0.0), Some((20.0, 0.0))), ((0.0, 0.0), None)],
        same: vec![],
    });

    // 10. A crossing line is deleted after a split: both halves merge back; the larger
    //     half's id continues, the smaller is lost.
    let mut b = sk();
    rect(&mut b, 0.0, 0.0, 100.0, 60.0);
    b.add_line(p(70.0, -5.0), p(70.0, 65.0)).unwrap();
    let mut a = sk();
    rect(&mut a, 0.0, 0.0, 100.0, 60.0);
    v.push(Case {
        name: "crossing line deleted",
        before: b,
        after: a,
        intent: vec![((20.0, 30.0), Some((20.0, 30.0))), ((85.0, 30.0), None)],
        same: vec![],
    });

    v
}

#[test]
fn ten_topology_edits_map_to_intended_successors() {
    let cs = cases();
    assert_eq!(cs.len(), 10);
    for c in &cs {
        let (old, new) = (detect(&c.before), detect(&c.after));
        // Deterministic: re-running yields identical ids and matches.
        let (old2, new2) = (detect(&c.before), detect(&c.after));
        let ids = |s: &ProfileSet| s.profiles.iter().map(|p| p.id).collect::<Vec<_>>();
        assert_eq!(ids(&old), ids(&old2), "{}", c.name);
        assert_eq!(ids(&new), ids(&new2), "{}", c.name);
        let m = new.match_previous(&old);
        assert_eq!(m, new2.match_previous(&old2), "{}", c.name);
        assert_eq!(
            m.len(),
            old.profiles.len(),
            "{}: one entry per old profile",
            c.name
        );

        for &(ox, oy) in &c.same {
            let id = id_at(&old, ox, oy);
            assert_eq!(m[&id], ProfileMatch::Same(id), "{}", c.name);
        }
        for &((ox, oy), want) in &c.intent {
            let oid = id_at(&old, ox, oy);
            match (want, m[&oid]) {
                (None, ProfileMatch::Lost) => {}
                (Some((nx, ny)), ProfileMatch::Same(n))
                | (Some((nx, ny)), ProfileMatch::Fingerprint { new: n, .. }) => {
                    assert_eq!(n, id_at(&new, nx, ny), "{}: ({ox}, {oy})", c.name);
                }
                (w, got) => panic!("{}: ({ox}, {oy}) expected {w:?}, got {got:?}", c.name),
            }
        }
        // One-to-one: no two old profiles share a successor.
        let mut seen = std::collections::BTreeSet::new();
        for v in m.values() {
            match v {
                ProfileMatch::Same(n) | ProfileMatch::Fingerprint { new: n, .. } => {
                    assert!(seen.insert(*n), "{}: successor claimed twice", c.name);
                    assert!(new.get(*n).is_some(), "{}", c.name);
                }
                ProfileMatch::Lost => {}
            }
        }
    }
}

#[test]
fn hole_changes_id_but_not_outer_sources() {
    // The cap-name key (outer source set) stays while the ProfileId changes.
    let mut b = sk();
    rect(&mut b, 0.0, 0.0, 100.0, 60.0);
    let mut a = b.clone();
    a.add_circle(p(50.0, 30.0), 10.0).unwrap();
    let (old, new) = (detect(&b), detect(&a));
    let o = old.get(id_at(&old, 10.0, 10.0)).unwrap();
    let n = new.get(id_at(&new, 10.0, 10.0)).unwrap();
    assert_ne!(o.id, n.id);
    assert_eq!(o.outer_sources(), n.outer_sources());
}

#[test]
fn match_fingerprint_is_pure_and_finds_the_region() {
    let mut s = sk();
    rect(&mut s, 0.0, 0.0, 100.0, 60.0);
    rect(&mut s, 200.0, 0.0, 230.0, 20.0);
    let set = detect(&s);
    for pr in &set.profiles {
        match set.match_fingerprint(&pr.fingerprint) {
            ProfileMatch::Fingerprint { new, score } => {
                assert_eq!(new, pr.id);
                assert!((score - 1.0).abs() < 1e-12);
            }
            other => panic!("{other:?}"),
        }
    }
    let mut fp = set.profiles[0].fingerprint.clone();
    fp.centroid = p(5000.0, 5000.0);
    fp.area *= 100.0;
    fp.sources.clear();
    fp.outer_sources.clear();
    assert_eq!(set.match_fingerprint(&fp), ProfileMatch::Lost);
}
