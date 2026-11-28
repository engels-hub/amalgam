//! DOC-15: extrude "All" (SOL-02) and feature patterns and mirrors (SOL-08) declare the
//! context bodies they read, so an upstream edit of one of them recomputes them. The
//! declaration tests run everywhere; the stale-cache regressions need the `occt` feature
//! (AMG_OCCT_DIR).

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "tests"
)]

mod solid_common;

use amg_doc::{BodyId, ContextBodies, Expr, ExprSrc, Feature, FeatureId};
use amg_features::construct::{OriginEntity, RefInput};
use amg_features::solid::{
    BodyOp, ComputeType, Direction, Extent, ExtentSide, Extrude, Mirror, OpTarget, Pattern,
    PatternDir, PatternKind, PatternObjects, Spacing,
};

fn ex(s: &str) -> Expr {
    Expr::Inline(ExprSrc(s.to_owned()))
}

fn some_body() -> BodyId {
    BodyId {
        origin: FeatureId(3),
        slot: 0,
    }
}

/// Three instances along X, 20 mm apart.
fn row(objects: PatternObjects, target: Option<OpTarget>) -> Pattern {
    Pattern {
        objects,
        kind: PatternKind::Rectangular {
            d1: PatternDir {
                dir: RefInput::Origin(OriginEntity::XAxis),
                count: ex("3"),
                spacing: Spacing::Spacing(ex("20 mm")),
                flip: false,
                symmetric: false,
            },
            d2: None,
        },
        suppressed: Vec::new(),
        compute: ComputeType::Identical,
        target,
    }
}

fn mirror(objects: PatternObjects, target: Option<OpTarget>) -> Mirror {
    Mirror {
        objects,
        plane: RefInput::Origin(OriginEntity::YZPlane),
        op: BodyOp::NewBody,
        target,
    }
}

#[test]
fn only_an_all_extent_makes_an_extrude_read_its_context() {
    let mut e = Extrude::distance(Vec::new(), "1 mm", OpTarget::default());
    assert_eq!(e.context_reads(), None);
    e.side1.extent = Extent::All;
    assert_eq!(e.context_reads(), Some(ContextBodies::Context));
    e.side1 = ExtentSide::distance(ex("1 mm"));
    e.direction = Direction::TwoSides;
    e.side2 = Some(ExtentSide::distance(ex("2 mm")));
    assert_eq!(e.context_reads(), None);
    e.side2.as_mut().unwrap().extent = Extent::All;
    assert_eq!(e.context_reads(), Some(ContextBodies::Context));
}

#[test]
fn feature_patterns_and_mirrors_read_their_context_body_ones_do_not() {
    let seeds = PatternObjects::Features(vec![FeatureId(2)]);
    let bodies = PatternObjects::Bodies(vec![some_body()]);
    let target = OpTarget {
        op: BodyOp::Cut,
        participants: vec![some_body()],
    };
    for t in [None, Some(target)] {
        let c = Some(ContextBodies::Context);
        assert_eq!(row(seeds.clone(), t.clone()).context_reads(), c);
        assert_eq!(mirror(seeds.clone(), t).context_reads(), c);
    }
    assert_eq!(row(bodies.clone(), None).context_reads(), None);
    assert_eq!(mirror(bodies, None).context_reads(), None);
}

#[cfg(feature = "occt")]
mod occt {
    use std::f64::consts::PI;
    use std::time::Duration;

    use amg_core::{Point2, Transform};
    use amg_doc::{
        BodyGeom, CacheKey, Command, ComponentId, CreatedId, Document, FeatureBox, Health,
    };
    use amg_features::sketch::{PlaneInput, SketchFeature};

    use super::solid_common::*;
    use super::*;

    /// Inserts `f` into the timeline at `at` (root component) and recomputes.
    fn insert(doc: &mut Document, at: usize, f: FeatureBox) -> FeatureId {
        let root = doc.design().root();
        insert_in(doc, Some(at), root, f)
    }

    fn insert_in(
        doc: &mut Document,
        at: Option<usize>,
        context: ComponentId,
        feature: FeatureBox,
    ) -> FeatureId {
        let o = doc
            .apply(Command::AddFeature {
                at,
                feature,
                context,
                name: None,
            })
            .unwrap();
        doc.recompute_blocking(jobs()).unwrap();
        created_feature(&o)
    }

    fn index(doc: &Document, f: FeatureId) -> usize {
        doc.design().timeline().node(f).unwrap().0
    }

    #[track_caller]
    fn key(doc: &Document, f: FeatureId) -> CacheKey {
        assert_eq!(health(doc, f), Health::Ok, "feature {f:?}");
        doc.evaluated().node(f).unwrap().key.unwrap()
    }

    /// `f` was served from the cache in the last pass, under the key `was`.
    #[track_caller]
    fn assert_hit(doc: &Document, f: FeatureId, was: CacheKey) {
        assert_eq!(key(doc, f), was, "{f:?}: the key changed");
        let node = doc.evaluated().node(f).unwrap();
        assert_eq!(node.duration, Duration::ZERO, "{f:?} was computed");
    }

    /// The total volume of the solid bodies `feature` created.
    fn created_volume(doc: &Document, feature: FeatureId) -> f64 {
        doc.evaluated()
            .bodies()
            .filter(|(id, _)| id.origin == feature)
            .map(|(_, b)| match &b.geom {
                BodyGeom::Solid(s) => volume(s.brep().expect("a B-rep")),
                other => panic!("not a solid: {other:?}"),
            })
            .sum()
    }

    /// A sketch of one rectangle and its extrude by `h`, inserted at `at`.
    fn insert_block(
        doc: &mut Document,
        at: usize,
        rect: [f64; 4],
        h: f64,
    ) -> (FeatureId, FeatureId) {
        let s = insert(doc, at, FeatureBox::new(rect_sketch(&[rect])));
        let e = Extrude::distance(profiles(doc, s), &format!("{h} mm"), OpTarget::default());
        let e = insert(doc, at + 1, FeatureBox::new(e));
        assert_eq!(health(doc, e), Health::Ok);
        (s, e)
    }

    /// Changes the height of the block extrude `e` of sketch `s`.
    fn set_height(doc: &mut Document, s: FeatureId, e: FeatureId, h: f64) {
        let f = Extrude::distance(profiles(doc, s), &format!("{h} mm"), OpTarget::default());
        edit(doc, e, FeatureBox::new(f));
    }

    /// A body in a new component and a sketch in the root: neither is in an "All" extrude's or
    /// a feature pattern's context bodies. Both go in at timeline index `at`.
    fn unrelated_edits(doc: &mut Document, at: usize) {
        let root = doc.design().root();
        let o = doc
            .apply(Command::NewComponent {
                parent: root,
                name: "elsewhere".into(),
                transform: Transform::IDENTITY,
            })
            .unwrap();
        let other = o
            .created
            .iter()
            .find_map(|c| match c {
                CreatedId::Component(c) => Some(*c),
                _ => None,
            })
            .unwrap();
        let s = insert_in(
            doc,
            Some(at),
            other,
            FeatureBox::new(rect_sketch(&[[200.0, 0.0, 210.0, 10.0]])),
        );
        let e = Extrude::distance(profiles(doc, s), "500 mm", OpTarget::default());
        let e = insert_in(doc, Some(at + 1), other, FeatureBox::new(e));
        assert_eq!(health(doc, e), Health::Ok);
        set_height(doc, s, e, 600.0);
        insert(
            doc,
            at,
            FeatureBox::new(rect_sketch(&[[-20.0, -20.0, -10.0, -10.0]])),
        );
    }

    /// Plate A (10 × 10 × 10) and, after it, a 2 × 2 sketch inside its footprint.
    fn plate_and_profile(doc: &mut Document) -> (BodyId, FeatureId) {
        let (_, a) = extrude_rects(
            doc,
            &[[0.0, 0.0, 10.0, 10.0]],
            10.0,
            BodyOp::NewBody,
            Vec::new(),
        );
        let s = add(doc, FeatureBox::new(rect_sketch(&[[2.0, 2.0, 4.0, 4.0]])));
        (body0(a), s)
    }

    fn all_extrude(doc: &mut Document, sketch: FeatureId, target: OpTarget) -> FeatureId {
        let mut e = Extrude::distance(profiles(doc, sketch), "1 mm", target);
        e.side1.extent = Extent::All;
        add(doc, FeatureBox::new(e))
    }

    #[test]
    fn all_new_body_follows_bodies_inserted_and_edited_before_it() {
        let mut doc = Document::new("all-new");
        let (_, s) = plate_and_profile(&mut doc);
        let all = all_extrude(&mut doc, s, OpTarget::default());
        // 2 × 2 × (10 + 1).
        assert_rel(created_volume(&doc, all), 44.0, 1e-9);
        let first = key(&doc, all);

        // A taller block between the seed sketch and the "All" extrude.
        let at = index(&doc, all);
        let (bs, be) = insert_block(&mut doc, at, [50.0, 0.0, 60.0, 10.0], 30.0);
        assert_rel(created_volume(&doc, all), 124.0, 1e-9);
        assert_ne!(key(&doc, all), first);

        set_height(&mut doc, bs, be, 50.0);
        assert_rel(created_volume(&doc, all), 204.0, 1e-9);

        let before = key(&doc, all);
        let at = index(&doc, all);
        unrelated_edits(&mut doc, at);
        assert_hit(&doc, all, before);
        assert_rel(created_volume(&doc, all), 204.0, 1e-9);

        // A body after the extrude is not one it reads.
        let end = doc.design().timeline().len();
        insert_block(&mut doc, end, [80.0, 0.0, 90.0, 10.0], 90.0);
        assert_hit(&doc, all, before);
    }

    #[test]
    fn all_join_follows_a_body_that_is_not_a_participant() {
        let mut doc = Document::new("all-join");
        let (a, s) = plate_and_profile(&mut doc);
        let (bs, be) = {
            let end = doc.design().timeline().len();
            insert_block(&mut doc, end, [50.0, 0.0, 60.0, 10.0], 10.0)
        };
        let all = all_extrude(
            &mut doc,
            s,
            OpTarget {
                op: BodyOp::Join,
                participants: vec![a],
            },
        );
        assert_eq!(health(&doc, all), Health::Ok);
        assert_rel(volume(&brep(&doc, a)), 1004.0, 1e-9);
        set_height(&mut doc, bs, be, 30.0);
        assert_rel(volume(&brep(&doc, a)), 1084.0, 1e-9);
    }

    #[test]
    fn all_cut_recomputes_when_a_body_before_it_changes() {
        let mut doc = Document::new("all-cut");
        let (a, s) = plate_and_profile(&mut doc);
        let cut = all_extrude(
            &mut doc,
            s,
            OpTarget {
                op: BodyOp::Cut,
                participants: vec![a],
            },
        );
        assert_rel(volume(&brep(&doc, a)), 960.0, 1e-9);
        let first = key(&doc, cut);
        let at = index(&doc, cut);
        let (bs, be) = insert_block(&mut doc, at, [50.0, 0.0, 60.0, 10.0], 30.0);
        let inserted = key(&doc, cut);
        assert_ne!(inserted, first);
        set_height(&mut doc, bs, be, 40.0);
        assert_ne!(key(&doc, cut), inserted);
        assert_rel(volume(&brep(&doc, a)), 960.0, 1e-9);
    }

    // ---- feature patterns and mirrors ---------------------------------------------------------

    const HOLE: f64 = 4.0 * PI; // r = 2, per mm of depth

    /// A 100 × 20 × 10 plate centred on X = 0 and a r 2 hole 5 mm deep at (10, 10), cut from it.
    fn plate_with_hole(doc: &mut Document) -> (BodyId, FeatureId) {
        let (_, e) = extrude_rects(
            doc,
            &[[-50.0, 0.0, 50.0, 20.0]],
            10.0,
            BodyOp::NewBody,
            Vec::new(),
        );
        let plate = body0(e);
        let mut sk = SketchFeature::new(PlaneInput::OriginXY);
        sk.edit_sketch(|s| s.add_circle(Point2::new(10.0, 10.0), 2.0).map(|_| ()))
            .unwrap();
        let s = add(doc, FeatureBox::new(sk));
        let hole = Extrude::distance(
            profiles(doc, s),
            "5 mm",
            OpTarget {
                op: BodyOp::Cut,
                participants: vec![plate],
            },
        );
        let hole = add(doc, FeatureBox::new(hole));
        assert_eq!(health(doc, hole), Health::Ok);
        (plate, hole)
    }

    /// Inserts before `dependent` a block joined onto the plate's far end (x 40..50, from the
    /// ground to `h`, so it adds `200 (h − 10)` mm³), edits it, and makes unrelated edits;
    /// `base` is the plate's volume after `dependent` without the boss.
    fn check_participant_edits(doc: &mut Document, plate: BodyId, dependent: FeatureId, base: f64) {
        assert_rel(volume(&brep(doc, plate)), base, 1e-9);
        let first = key(doc, dependent);

        let at = index(doc, dependent);
        let s = insert(
            doc,
            at,
            FeatureBox::new(rect_sketch(&[[40.0, 0.0, 50.0, 20.0]])),
        );
        let boss = |doc: &Document, h: &str| {
            FeatureBox::new(Extrude::distance(
                profiles(doc, s),
                h,
                OpTarget {
                    op: BodyOp::Join,
                    participants: vec![plate],
                },
            ))
        };
        let f = boss(doc, "20 mm");
        let b = insert(doc, at + 1, f);
        assert_eq!(health(doc, b), Health::Ok);
        // Stale before DOC-15: the dependent feature put its old plate back, without the boss.
        assert_ne!(key(doc, dependent), first);
        assert_rel(volume(&brep(doc, plate)), base + 2000.0, 1e-9);

        let f = boss(doc, "30 mm");
        edit(doc, b, f);
        assert_rel(volume(&brep(doc, plate)), base + 4000.0, 1e-9);

        let before = key(doc, dependent);
        let at = index(doc, dependent);
        unrelated_edits(doc, at);
        assert_hit(doc, dependent, before);
        assert_rel(volume(&brep(doc, plate)), base + 4000.0, 1e-9);
    }

    #[test]
    fn target_less_feature_pattern_follows_its_participant() {
        let mut doc = Document::new("pattern");
        let (plate, hole) = plate_with_hole(&mut doc);
        // Holes at x = 10, −10, −30 (the row steps backwards, away from the boss).
        let mut p = row(PatternObjects::Features(vec![hole]), None);
        let PatternKind::Rectangular { d1, .. } = &mut p.kind else {
            unreachable!()
        };
        d1.flip = true;
        let p = add(&mut doc, FeatureBox::new(p));
        check_participant_edits(&mut doc, plate, p, 20_000.0 - 3.0 * 5.0 * HOLE);
    }

    #[test]
    fn target_less_feature_mirror_follows_its_participant() {
        let mut doc = Document::new("mirror");
        let (plate, hole) = plate_with_hole(&mut doc);
        let m = mirror(PatternObjects::Features(vec![hole]), None);
        let m = add(&mut doc, FeatureBox::new(m));
        check_participant_edits(&mut doc, plate, m, 20_000.0 - 2.0 * 5.0 * HOLE);
    }

    /// With an explicit target the pattern still rebuilds an "All" seed's tool against the
    /// bodies just before itself.
    #[test]
    fn targeted_pattern_of_an_all_seed_follows_a_body_after_the_seed() {
        let mut doc = Document::new("pattern-all-seed");
        let (a, s) = plate_and_profile(&mut doc);
        let join = OpTarget {
            op: BodyOp::Join,
            participants: vec![a],
        };
        let seed = all_extrude(&mut doc, s, join.clone());
        // One more instance, 4 mm along X, still inside the plate's footprint.
        let mut p = row(PatternObjects::Features(vec![seed]), Some(join));
        let PatternKind::Rectangular { d1, .. } = &mut p.kind else {
            unreachable!()
        };
        d1.count = ex("2");
        d1.spacing = Spacing::Spacing(ex("4 mm"));
        let p = add(&mut doc, FeatureBox::new(p));
        assert_eq!(health(&doc, p), Health::Ok);
        // The seed stands 1 mm above the plate. Its tool, rebuilt at the pattern, measures
        // the seed's own tab too, so the copy stands 2 mm above it.
        assert_rel(volume(&brep(&doc, a)), 1000.0 + 4.0 + 8.0, 1e-9);
        let seed_key = key(&doc, seed);
        let at = index(&doc, p);
        insert_block(&mut doc, at, [50.0, 0.0, 60.0, 10.0], 30.0);
        assert_hit(&doc, seed, seed_key);
        // The seed still ends at 11; the copy now reaches 31.
        assert_rel(volume(&brep(&doc, a)), 1000.0 + 4.0 + 84.0, 1e-9);
    }
}
