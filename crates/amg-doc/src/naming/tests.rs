//! Unit tests of the naming framework on synthetic element maps and histories (DOC-06
//! Done-when 1). No OCCT needed: fingerprints come from a table, topology from
//! [`SyntheticTopology`].

use std::collections::{BTreeMap, HashMap};

use amg_brep::geom::Tag;
use amg_brep::{EdgeId, InputSub, OpRole, Relation, ShapeHistory, SubKind, SubShape};
use amg_core::{Aabb3, Point3, Tolerance, Vec3};

use super::*;
use crate::eval::BodyGeom;
use crate::id::FeatureId;
use crate::stub::{SdfGraph, Shape};

// ---- helpers ----------------------------------------------------------------------------------

const F: fn(u64) -> FeatureId = FeatureId;

fn body(o: u64) -> BodyId {
    BodyId {
        origin: F(o),
        slot: 0,
    }
}

fn gen_name(feature: u64, tag: &'static str) -> TopoName {
    TopoName::new(vec![NameSeg::Gen {
        feature: F(feature),
        role: Role::new(tag, 0),
        from: None,
    }])
}

fn split(n: &TopoName, feature: u64, index: u32, count: u32) -> TopoName {
    n.with(NameSeg::Split {
        feature: F(feature),
        index,
        count,
    })
}

fn bound(faces: &[&TopoName], index: u32, count: u32) -> TopoName {
    let mut f: Vec<TopoName> = faces.iter().map(|x| (*x).clone()).collect();
    f.sort();
    TopoName::new(vec![NameSeg::Bound {
        faces: f,
        index,
        count,
    }])
}

fn fp(class: GeomClass, c: [f64; 3], measure: f64, half: f64) -> GeomFingerprint {
    let c = Point3::from_array(c);
    GeomFingerprint {
        class,
        measure,
        centroid: c,
        axis: Some(Vec3::X),
        radius: None,
        bbox: Aabb3::new(c - Vec3::splat(half), c + Vec3::splat(half)),
    }
}

fn line(x: f64, len: f64) -> GeomFingerprint {
    let mut f = fp(GeomClass::Line, [x, 0.0, 40.0], len, 0.0);
    f.bbox = Aabb3::new(
        Point3::new(x - len / 2.0, 0.0, 40.0),
        Point3::new(x + len / 2.0, 0.0, 40.0),
    );
    f
}

fn plane(x: f64, area: f64) -> GeomFingerprint {
    fp(GeomClass::Plane, [x, 30.0, 40.0], area, 30.0)
}

#[derive(Default)]
struct View {
    bodies: BTreeMap<BodyId, (BodyGeom, BodyNaming)>,
    succ: BTreeMap<BodyId, Vec<BodyId>>,
    derived: BTreeMap<BodyId, BodyId>,
    fps: HashMap<(BodyId, TopoKind, u32), GeomFingerprint>,
}

impl View {
    fn with(naming: BodyNaming) -> View {
        let mut v = View::default();
        v.add(body(1), naming);
        v
    }

    fn add(&mut self, id: BodyId, naming: BodyNaming) {
        self.bodies
            .insert(id, (BodyGeom::Solid(Shape::new()), naming));
    }

    fn fp(mut self, kind: TopoKind, index: u32, f: GeomFingerprint) -> View {
        self.fps.insert((body(1), kind, index), f);
        self
    }

    fn resolve(&self, r: &TopoRef) -> Result<Resolved, ResolveError> {
        resolve_with(self, self, &Tolerance::default(), r)
            .result
            .map(|v| v[0])
    }
}

impl NamingView for View {
    fn body_geom(&self, id: BodyId) -> Option<&BodyGeom> {
        self.bodies.get(&id).map(|b| &b.0)
    }
    fn naming(&self, id: BodyId) -> Option<&BodyNaming> {
        self.bodies.get(&id).map(|b| &b.1)
    }
    fn successors(&self, id: BodyId) -> &[BodyId] {
        self.succ.get(&id).map_or(&[], Vec::as_slice)
    }
    fn derived_from(&self, id: BodyId) -> Option<BodyId> {
        self.derived.get(&id).copied()
    }
}

impl GeomSource for View {
    fn fingerprint(&self, b: BodyId, kind: TopoKind, index: u32) -> Option<GeomFingerprint> {
        self.fps.get(&(b, kind, index)).cloned()
    }
    fn all(&self, b: BodyId, kind: TopoKind) -> Vec<(u32, GeomFingerprint)> {
        let mut v: Vec<_> = self
            .fps
            .iter()
            .filter(|((bb, k, _), _)| *bb == b && *k == kind)
            .map(|((_, _, i), f)| (*i, f.clone()))
            .collect();
        v.sort_by_key(|x| x.0);
        v
    }
}

fn r(kind: TopoKind, name: TopoName, f: Option<GeomFingerprint>) -> TopoRef {
    TopoRef {
        body: body(1),
        kind,
        name,
        fingerprint: f,
        key: 1,
    }
}

/// Front, top (whole) and back faces of a box-like body.
fn faces() -> (TopoName, TopoName, TopoName) {
    (
        gen_name(1, "YMin"),
        gen_name(1, "ZMax"),
        gen_name(1, "YMax"),
    )
}

// ---- types, serde, metric ---------------------------------------------------------------------

#[test]
fn fingerprint_eq_is_quantised() {
    let f = |x: f64| plane(x, 100.0);
    assert_eq!(f(1.0), f(1.0 + 1e-9));
    assert_ne!(f(1.0), f(1.0 + 1e-3));
    let hash = |f: &GeomFingerprint| {
        let mut h = std::collections::hash_map::DefaultHasher::new();
        f.hash(&mut h);
        h.finish()
    };
    assert_eq!(hash(&f(2.0)), hash(&f(2.0 + 1e-9)));
    let mut rad = f(1.0);
    rad.radius = Some(3.0);
    assert_ne!(rad, f(1.0), "radius takes part in equality");
}

#[test]
fn entity_ref_round_trips_and_old_data_loads() {
    let name = split(
        &TopoName::new(vec![NameSeg::Gen {
            feature: F(3),
            role: Role::new("Lateral", 0),
            from: Some(NameSource::SketchCurve {
                sketch: SketchId(F(2)),
                curve: SketchEntityKey(7),
            }),
        }]),
        5,
        1,
        2,
    );
    let er = EntityRef::InContext {
        path: vec![OccurrenceId(5)],
        target: Box::new(EntityRef::Topo(TopoRef {
            key: 4,
            ..r(TopoKind::Face, name, Some(plane(1.0, 100.0)))
        })),
    };
    let s = serde_json::to_string(&er).unwrap();
    assert!(
        s.contains(r#"{"Gen":{"feature":3,"role":{"tag":"Lateral","index":0}"#),
        "{s}"
    );
    let back: EntityRef = serde_json::from_str(&s).unwrap();
    assert_eq!(back, er);
    assert_eq!(serde_json::to_string(&back).unwrap(), s);
    let old = s
        .replace(r#","key":4"#, "")
        .replace(r#","radius":null"#, "");
    let EntityRef::InContext { target, .. } = serde_json::from_str(&old).unwrap() else {
        panic!()
    };
    let EntityRef::Topo(t) = *target else {
        panic!()
    };
    assert_eq!(t.key, 0);
    // Profile refs gained an optional fingerprint.
    let p: EntityRef = serde_json::from_str(r#"{"Profile":{"sketch":2,"profile":9}}"#).unwrap();
    assert!(matches!(
        p,
        EntityRef::Profile {
            fingerprint: None,
            ..
        }
    ));
}

#[test]
fn distance_is_none_on_class_mismatch_never_infinite() {
    let tol = Tolerance::default();
    let a = line(50.0, 100.0);
    let b = plane(50.0, 100.0);
    assert_eq!(a.distance(&b, &tol), None);
    let d = a.distance(&line(65.0, 70.0), &tol).unwrap();
    assert!((d - 0.2598).abs() < 1e-3, "TNP-05 long piece: {d}");
    let d = a.distance(&line(10.0, 20.0), &tol).unwrap();
    assert!((d - 0.6928).abs() < 1e-3, "TNP-05 short piece: {d}");
    // A zero-size reference uses the 10·tol scale, not a division by zero.
    let p = fp(GeomClass::Point, [0.0; 3], 0.0, 0.0);
    let d = p.distance(&fp(GeomClass::Point, [1e-6, 0.0, 0.0], 0.0, 0.0), &tol);
    assert!(d.is_some_and(f64::is_finite));
}

#[test]
fn identity_ignores_events_and_bound_indices() {
    let (_, top, front) = faces();
    let a = split(&top, 5, 0, 2);
    assert_eq!(identity(&a), identity(&top));
    assert_eq!(
        identity(&bound(&[&a, &front], 1, 2)),
        identity(&bound(&[&top, &front], 0, 1))
    );
    let inst = top.with(NameSeg::Inst {
        feature: F(9),
        instance: 3,
    });
    assert_ne!(identity(&inst), identity(&top), "Inst is part of identity");
    assert_eq!(identity_digest(&a), identity_digest(&top));
}

// ---- resolver stages (Done-when 1) ------------------------------------------------------------

fn top_edge_body(pieces: &[(f64, f64)]) -> View {
    let (front, top, back) = faces();
    let n = u32::try_from(pieces.len()).unwrap();
    let edges: Vec<TopoName> = (0..n).map(|i| bound(&[&front, &top], i, n)).collect();
    let mut v = View::with(BodyNaming::new(vec![front, top, back], edges, Vec::new()));
    for (i, (x, len)) in (0u32..).zip(pieces) {
        v = v.fp(TopoKind::Edge, i, line(*x, *len));
    }
    v
}

fn top_edge_ref() -> TopoRef {
    let (front, top, _) = faces();
    r(
        TopoKind::Edge,
        bound(&[&front, &top], 0, 1),
        Some(line(50.0, 100.0)),
    )
}

#[test]
fn exact_when_the_name_is_unique_and_equal() {
    let v = top_edge_body(&[(50.0, 100.0)]);
    let x = v.resolve(&top_edge_ref()).unwrap();
    assert_eq!(
        (x.index, x.quality, x.warning),
        (0, MatchQuality::Exact, None)
    );
    // A dimension edit moves the edge far: still Exact, no sanity check on a same-name match.
    let v = top_edge_body(&[(500.0, 1000.0)]);
    assert_eq!(
        v.resolve(&top_edge_ref()).unwrap().quality,
        MatchQuality::Exact
    );
}

#[test]
fn split_disambiguated_by_fingerprint_with_new_split_warning() {
    // TNP-05: an off-centre notch leaves 20 and 70 mm pieces.
    let v = top_edge_body(&[(65.0, 70.0), (10.0, 20.0)]);
    let x = v.resolve(&top_edge_ref()).unwrap();
    assert_eq!(x.index, 0);
    assert_eq!(x.quality, MatchQuality::Disambiguated);
    assert_eq!(
        x.warning,
        Some(ResolveWarning::NewSplit {
            feature: None,
            pieces: 2
        })
    );
    // Face split by feature 5: the warning names it.
    let (front, top, back) = faces();
    let v = View::with(BodyNaming::new(
        vec![front, split(&top, 5, 0, 2), split(&top, 5, 1, 2), back],
        Vec::new(),
        Vec::new(),
    ))
    .fp(TopoKind::Face, 1, plane(30.0, 3600.0))
    .fp(TopoKind::Face, 2, plane(85.0, 1800.0));
    let x = v
        .resolve(&r(TopoKind::Face, faces().1, Some(plane(80.0, 1200.0))))
        .unwrap();
    assert_eq!(x.index, 2);
    assert!(matches!(
        x.warning,
        Some(ResolveWarning::NewSplit {
            feature: Some(FeatureId(5)),
            pieces: 2
        })
    ));
}

#[test]
fn symmetric_pieces_are_ambiguous() {
    // TNP-04: a centred notch.
    let v = top_edge_body(&[(22.5, 45.0), (77.5, 45.0)]);
    match v.resolve(&top_edge_ref()) {
        Err(ResolveError::Ambiguous { candidates }) => {
            assert_eq!(candidates, vec![(body(1), 0), (body(1), 1)]);
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn seen_split_follows_position_with_index_drift() {
    // TNP-07: the reference saw piece 1; the pieces swapped order.
    let (front, top, back) = faces();
    let a = split(&top, 6, 1, 2); // x 0..35 (now the smaller piece, index 1)
    let b = split(&top, 6, 0, 2); // x 45..100
    let edges = vec![bound(&[&front, &a], 0, 1), bound(&[&front, &b], 0, 1)];
    let v = View::with(BodyNaming::new(
        vec![front.clone(), a, b, back],
        edges,
        Vec::new(),
    ))
    .fp(TopoKind::Edge, 0, line(17.5, 35.0))
    .fp(TopoKind::Edge, 1, line(72.5, 55.0));
    let seen = r(
        TopoKind::Edge,
        bound(&[&front, &split(&top, 6, 1, 2)], 0, 1),
        Some(line(85.0, 30.0)),
    );
    let x = v.resolve(&seen).unwrap();
    assert_eq!(x.index, 1, "position wins over the index");
    assert_eq!(x.warning, Some(ResolveWarning::IndexDrift));
    assert_eq!(x.quality, MatchQuality::Disambiguated);
    // Same picture without the swap: name-equal and nearest agree → Exact.
    let seen0 = TopoRef {
        fingerprint: Some(line(17.0, 34.0)),
        ..seen
    };
    let x = v.resolve(&seen0).unwrap();
    assert_eq!(
        (x.index, x.quality, x.warning),
        (0, MatchQuality::Exact, None)
    );
}

#[test]
fn margins_low_and_far() {
    let v = top_edge_body(&[(40.0, 100.0), (75.0, 100.0)]);
    let x = v.resolve(&top_edge_ref()).unwrap();
    assert!(
        matches!(x.warning, Some(ResolveWarning::NewSplit { .. })),
        "new split outranks LowMargin"
    );
    // A seen split (same classes) with a ratio in (0.25, 0.5]: LowMargin.
    let (front, top, back) = faces();
    let (a, b) = (split(&top, 6, 0, 2), split(&top, 6, 1, 2));
    let mk = |xa: f64, xb: f64| {
        View::with(BodyNaming::new(
            vec![front.clone(), a.clone(), b.clone(), back.clone()],
            vec![bound(&[&front, &a], 0, 1), bound(&[&front, &b], 0, 1)],
            Vec::new(),
        ))
        .fp(TopoKind::Edge, 0, line(xa, 30.0))
        .fp(TopoKind::Edge, 1, line(xb, 30.0))
    };
    let rf = r(
        TopoKind::Edge,
        bound(&[&front, &a], 0, 1),
        Some(line(50.0, 30.0)),
    );
    let x = mk(60.0, 20.0).resolve(&rf).unwrap();
    assert!(
        matches!(x.warning, Some(ResolveWarning::LowMargin { .. })),
        "{x:?}"
    );
    let x = mk(140.0, 1000.0).resolve(&rf).unwrap();
    assert!(
        matches!(x.warning, Some(ResolveWarning::FarMatch { .. })),
        "{x:?}"
    );
}

#[test]
fn fingerprint_only_when_lineage_is_lost() {
    // TNP-13: the generator is gone; an identical edge sits where the old one was.
    let v = top_edge_body(&[(50.0, 100.0), (50.0, 100.0 * 0.6)]);
    let mut v = v;
    v.fps.insert((body(1), TopoKind::Edge, 1), {
        let mut f = line(50.0, 100.0);
        f.centroid.z = 0.0;
        f.bbox = Aabb3::new(Point3::new(0.0, 0.0, 0.0), Point3::new(100.0, 0.0, 0.0));
        f
    });
    let gone = gen_name(77, "Lateral");
    let lost = r(
        TopoKind::Edge,
        bound(&[&faces().0, &gone], 0, 1),
        Some(line(50.0, 100.0)),
    );
    let x = v.resolve(&lost).unwrap();
    assert_eq!(x.index, 0);
    assert_eq!(x.quality, MatchQuality::FingerprintOnly);
    assert!(matches!(
        x.warning,
        Some(ResolveWarning::FingerprintOnly { .. })
    ));
    // Not unique (two identical edges): NotFound, never a guess.
    let v = top_edge_body(&[(50.0, 100.0), (50.0, 100.0)]);
    assert!(matches!(
        v.resolve(&lost),
        Err(ResolveError::NotFound {
            removed_by: None,
            ..
        })
    ));
    // No fingerprint: NotFound.
    let nofp = TopoRef {
        fingerprint: None,
        ..lost
    };
    assert!(matches!(
        v.resolve(&nofp),
        Err(ResolveError::NotFound { .. })
    ));
}

#[test]
fn not_found_when_faces_are_no_longer_adjacent() {
    // TNP-11: front and top still exist but share no edge; an identical-looking edge elsewhere
    // is not a fallback.
    let (front, top, back) = faces();
    let blend = gen_name(9, "Blend");
    let edges = vec![bound(&[&front, &blend], 0, 1), bound(&[&top, &blend], 0, 1)];
    let v = View::with(BodyNaming::new(
        vec![front, top, back, blend],
        edges,
        Vec::new(),
    ))
    .fp(TopoKind::Edge, 0, line(50.0, 100.0));
    assert_eq!(
        v.resolve(&top_edge_ref()),
        Err(ResolveError::NotFound {
            kind: TopoKind::Edge,
            removed_by: None
        })
    );
}

#[test]
fn tombstones_are_found_by_lineage() {
    let (front, top, back) = faces();
    let mut n = BodyNaming::new(vec![front, back], Vec::new(), Vec::new());
    n.removed = Arc::new(BTreeMap::from([(split(&top, 5, 0, 1), F(7))]));
    let v = View::with(n).fp(TopoKind::Face, 0, plane(50.0, 6000.0));
    let removed = Err(ResolveError::NotFound {
        kind: TopoKind::Face,
        removed_by: Some(F(7)),
    });
    // The whole face hits the tombstone of its only remaining piece...
    assert_eq!(
        v.resolve(&r(TopoKind::Face, top.clone(), Some(plane(50.0, 6000.0)))),
        removed
    );
    // ...a piece of it too (indices are wildcards), and no fingerprint guess happens.
    assert_eq!(
        v.resolve(&r(
            TopoKind::Face,
            split(&top, 5, 3, 4),
            Some(plane(50.0, 6000.0))
        )),
        removed
    );
    // A different lineage (split by another feature) doesn't match the tombstone.
    let other = split(&top, 8, 0, 2);
    assert!(matches!(
        v.resolve(&r(TopoKind::Face, other, None)),
        Err(ResolveError::NotFound {
            removed_by: None,
            ..
        })
    ));
    // An edge whose face was removed reports the removing feature.
    let (front, _, _) = faces();
    assert_eq!(
        v.resolve(&r(TopoKind::Edge, bound(&[&front, &top], 0, 1), None)),
        Err(ResolveError::NotFound {
            kind: TopoKind::Edge,
            removed_by: Some(F(7))
        })
    );
}

#[test]
fn unmerged_halves_tie_or_warn() {
    // TNP-20: the reference saw cap1 + Merge{e2, [cap2]}; the merge no longer happens.
    let c1 = gen_name(2, "EndCap");
    let c2 = gen_name(3, "EndCap");
    let merged = c1.with(NameSeg::Merge {
        feature: F(3),
        with: vec![c2.clone()],
    });
    let mk = |a: GeomFingerprint, b: GeomFingerprint| {
        View::with(BodyNaming::new(
            vec![c1.clone(), c2.clone()],
            Vec::new(),
            Vec::new(),
        ))
        .fp(TopoKind::Face, 0, a)
        .fp(TopoKind::Face, 1, b)
    };
    let rf = r(TopoKind::Face, merged, Some(plane(50.0, 6000.0)));
    assert!(matches!(
        mk(plane(25.0, 3000.0), plane(75.0, 3000.0)).resolve(&rf),
        Err(ResolveError::Ambiguous { .. })
    ));
    let x = mk(plane(48.0, 5800.0), plane(120.0, 200.0))
        .resolve(&rf)
        .unwrap();
    assert_eq!(x.index, 0);
    assert_eq!(x.warning, Some(ResolveWarning::Unmerged));
}

#[test]
fn widened_when_a_split_goes_away_or_a_merge_absorbs_the_face() {
    // TNP-08: the reference saw piece 1 of a split that no longer happens.
    let (front, top, back) = faces();
    let v = View::with(BodyNaming::new(
        vec![front.clone(), top.clone(), back.clone()],
        Vec::new(),
        Vec::new(),
    ));
    let x = v
        .resolve(&r(TopoKind::Face, split(&top, 6, 1, 2), None))
        .unwrap();
    assert_eq!(x.quality, MatchQuality::Widened);
    assert_eq!(
        x.warning,
        Some(ResolveWarning::Widened {
            cause: WidenCause::SplitGone { feature: F(6) }
        })
    );
    // The referenced face was absorbed by another (source (b)).
    let absorbed = top.with(NameSeg::Merge {
        feature: F(4),
        with: vec![back.clone()],
    });
    let v = View::with(BodyNaming::new(
        vec![front, absorbed],
        Vec::new(),
        Vec::new(),
    ));
    let x = v.resolve(&r(TopoKind::Face, back, None)).unwrap();
    assert_eq!((x.index, x.quality), (1, MatchQuality::Widened));
    assert_eq!(
        x.warning,
        Some(ResolveWarning::Widened {
            cause: WidenCause::Merged { feature: F(4) }
        })
    );
}

#[test]
fn class_changes_are_never_quiet() {
    let (front, top, back) = faces();
    let (a, b) = (split(&top, 6, 0, 2), split(&top, 6, 1, 2));
    let names = vec![front, a.clone(), b, back];
    let rf = r(TopoKind::Face, a, Some(plane(30.0, 1800.0)));
    let mut cone = plane(30.0, 1800.0);
    cone.class = GeomClass::Cone;
    // One candidate changed class: the other is taken, with ClassChanged.
    let v = View::with(BodyNaming::new(names.clone(), Vec::new(), Vec::new()))
        .fp(TopoKind::Face, 1, cone.clone())
        .fp(TopoKind::Face, 2, plane(80.0, 1200.0));
    let x = v.resolve(&rf).unwrap();
    assert_eq!((x.index, x.quality), (2, MatchQuality::Disambiguated));
    // The name-equal candidate is the one that changed class: IndexDrift outranks ClassChanged.
    assert_eq!(x.warning, Some(ResolveWarning::IndexDrift));
    // No name-equal candidate (the split count changed): ClassChanged.
    let x = v
        .resolve(&TopoRef {
            name: split(&faces().1, 6, 4, 5),
            ..rf.clone()
        })
        .unwrap();
    assert_eq!((x.index, x.quality), (2, MatchQuality::Disambiguated));
    assert_eq!(x.warning, Some(ResolveWarning::ClassChanged));
    // Every candidate changed class: Ambiguous.
    let v = View::with(BodyNaming::new(names.clone(), Vec::new(), Vec::new()))
        .fp(TopoKind::Face, 1, cone.clone())
        .fp(TopoKind::Face, 2, cone.clone());
    assert!(matches!(
        v.resolve(&rf),
        Err(ResolveError::Ambiguous { .. })
    ));
    // A unique wildcard match whose class changed: sanity check warns.
    let (front, top, back) = faces();
    let v = View::with(BodyNaming::new(
        vec![front, top, back],
        Vec::new(),
        Vec::new(),
    ))
    .fp(TopoKind::Face, 1, cone);
    let x = v
        .resolve(&r(
            TopoKind::Face,
            split(&faces().1, 6, 0, 2),
            Some(plane(30.0, 1800.0)),
        ))
        .unwrap();
    assert!(x.warning.is_some(), "{x:?}");
    assert_ne!(x.quality, MatchQuality::Exact);
}

#[test]
fn several_candidates_without_fingerprint() {
    let (front, top, back) = faces();
    let (a, b) = (split(&top, 6, 0, 2), split(&top, 6, 1, 2));
    let v = View::with(BodyNaming::new(
        vec![front, a.clone(), b, back],
        Vec::new(),
        Vec::new(),
    ));
    let x = v.resolve(&r(TopoKind::Face, a, None)).unwrap();
    assert_eq!(
        (x.index, x.warning),
        (1, Some(ResolveWarning::NoFingerprint))
    );
    assert!(matches!(
        v.resolve(&r(TopoKind::Face, top, None)),
        Err(ResolveError::Ambiguous { .. })
    ));
}

#[test]
fn body_errors_and_kinds() {
    let v = top_edge_body(&[(50.0, 100.0)]);
    let mut missing = top_edge_ref();
    missing.body = body(42);
    assert_eq!(
        v.resolve(&missing),
        Err(ResolveError::BodyMissing(body(42)))
    );
    let mut v = View::default();
    v.bodies.insert(
        body(1),
        (
            BodyGeom::Implicit(Arc::new(SdfGraph::default())),
            BodyNaming::default(),
        ),
    );
    assert_eq!(v.resolve(&top_edge_ref()), Err(ResolveError::Unsupported));
    let v = top_edge_body(&[(50.0, 100.0)]);
    let group = r(TopoKind::FaceGroup, faces().0, None);
    assert!(matches!(
        v.resolve(&group),
        Err(ResolveError::NotFound { .. })
    ));
}

#[test]
fn body_lineage_consumed_is_quiet_split_off_warns() {
    let (front, top, back) = faces();
    let naming = BodyNaming::new(
        vec![front.clone(), top.clone(), back],
        vec![bound(&[&front, &top], 0, 1)],
        Vec::new(),
    );
    // Body 1 was consumed into body 2 (combine): Exact in body 2.
    let mut v = View::default();
    v.add(body(2), naming.clone());
    v.succ.insert(body(1), vec![body(2)]);
    let x = v.resolve(&top_edge_ref()).unwrap();
    assert_eq!(
        (x.body, x.quality, x.warning),
        (body(2), MatchQuality::Exact, None)
    );
    // Body 1 still exists but lost the face; body 3 was split off it: BodyChanged.
    let mut v = View::default();
    v.add(
        body(1),
        BodyNaming::new(vec![gen_name(1, "XMin")], Vec::new(), Vec::new()),
    );
    let piece = BodyId {
        origin: F(8),
        slot: 1,
    };
    v.add(piece, naming);
    v.succ.insert(body(1), vec![piece]);
    v.derived.insert(piece, body(1));
    let x = v.resolve(&top_edge_ref()).unwrap();
    assert_eq!(x.body, piece);
    assert_eq!(
        x.warning,
        Some(ResolveWarning::BodyChanged { now_in: piece })
    );
}

#[test]
fn set_policies() {
    let tol = Tolerance::default();
    let (front, top, back) = faces();
    let v = top_edge_body(&[(65.0, 70.0), (10.0, 20.0)]);
    let ok = r(TopoKind::Face, front.clone(), None);
    let lost = r(TopoKind::Face, gen_name(99, "Gone"), None);
    // DropMissing: the lost member is dropped and recorded; the other resolves.
    let (recs, out) = resolve_set(
        &v,
        &v,
        &tol,
        &[ok.clone(), lost.clone()],
        SetPolicy::DropMissing,
    );
    let out = out.unwrap();
    assert_eq!((out[0].len(), out[1].len()), (1, 0));
    assert!(recs[1].result.is_err());
    // AllOrError: the first failure fails the set.
    let (_, out) = resolve_set(&v, &v, &tol, &[lost.clone(), ok], SetPolicy::AllOrError);
    assert!(out.is_err());
    // DropMissing with nothing left: error.
    let (_, out) = resolve_set(&v, &v, &tol, &[lost], SetPolicy::DropMissing);
    assert!(out.is_err());
    // A member predating a symmetric split expands to every piece (owner question 2).
    let v = top_edge_body(&[(22.5, 45.0), (77.5, 45.0)]);
    let (recs, out) = resolve_set(&v, &v, &tol, &[top_edge_ref()], SetPolicy::DropMissing);
    let out = out.unwrap();
    assert_eq!(out[0].iter().map(|x| x.index).collect::<Vec<_>>(), [0, 1]);
    assert!(matches!(
        out[0][0].warning,
        Some(ResolveWarning::NewSplit { pieces: 2, .. })
    ));
    assert_eq!(recs[0].current.len(), 2);
    let _ = (top, back);
}

#[test]
fn resolution_is_deterministic() {
    let v = top_edge_body(&[(65.0, 70.0), (10.0, 20.0), (90.0, 5.0)]);
    let tol = Tolerance::default();
    let a = resolve_with(&v, &v, &tol, &top_edge_ref());
    for _ in 0..20 {
        let w = top_edge_body(&[(65.0, 70.0), (10.0, 20.0), (90.0, 5.0)]);
        assert_eq!(resolve_with(&w, &w, &tol, &top_edge_ref()), a);
    }
    assert_eq!(a.current.len(), 1);
}

// ---- builder ----------------------------------------------------------------------------------

/// A 3-face strip: faces 0-1 share edge 0, 1-2 edge 1; free edges 2, 3; vertices on 0/1.
fn strip() -> SyntheticTopology {
    SyntheticTopology {
        faces: 3,
        edge_faces: vec![vec![0, 1], vec![1, 2], vec![0], vec![2]],
        vertex_edges: vec![vec![0, 2], vec![1, 3]],
        fingerprints: HashMap::new(),
    }
}

fn kept(kind: SubKind, n: u32) -> Vec<(InputSub, Relation)> {
    (0..n)
        .map(|index| {
            let sub = SubShape { kind, index };
            (InputSub { operand: 0, sub }, Relation::Kept(sub))
        })
        .collect()
}

fn history(entries: Vec<(InputSub, Relation)>) -> ShapeHistory {
    let mut entries = entries;
    entries.sort_by_key(|(i, _)| *i);
    ShapeHistory {
        entries,
        ..ShapeHistory::default()
    }
}

fn face(i: u32) -> SubShape {
    SubShape {
        kind: SubKind::Face,
        index: i,
    }
}

fn input(op: u16, sub: SubShape) -> InputSub {
    InputSub { operand: op, sub }
}

fn named_strip() -> BodyNaming {
    let mut b = NamingBuilder::new(F(1));
    for (i, t) in ["A", "B", "C"].into_iter().enumerate() {
        b.generated(
            TopoKind::Face,
            u32::try_from(i).unwrap(),
            Role::new(t, 0),
            None,
        );
    }
    b.finish_topo(&strip()).unwrap().0
}

#[test]
fn kept_names_and_edge_arcs_are_reused() {
    let base = named_strip();
    assert_eq!(base.edges.len(), 4);
    assert!(matches!(
        base.edges[0].origin(),
        Some(NameSeg::Bound { .. })
    ));
    let mut e = kept(SubKind::Face, 3);
    e.extend(kept(SubKind::Edge, 4));
    e.extend(kept(SubKind::Vertex, 2));
    let mut b = NamingBuilder::new(F(2));
    b.from_history(&[(0, OperandNaming::Named(&base))], &history(e));
    let (n, report) = b.finish_topo(&strip()).unwrap();
    assert!(!report.partial_history());
    assert_eq!(n, base);
    assert!(n.faces.iter().zip(&base.faces).all(|(a, b)| a.ptr_eq(b)));
    assert!(n.edges.iter().zip(&base.edges).all(|(a, b)| a.ptr_eq(b)));
}

#[test]
fn split_merge_generated_and_role_only_rules() {
    let base = named_strip();
    // Output: face 0 and 1 are pieces of input face 0 (A); face 2 merges B and C.
    let topo = SyntheticTopology {
        faces: 4,
        edge_faces: vec![vec![0, 1], vec![1, 2], vec![2, 3], vec![0, 3]],
        vertex_edges: vec![vec![0, 1], vec![2, 3]],
        fingerprints: HashMap::from([
            ((TopoKind::Face, 0), plane(10.0, 100.0)),
            ((TopoKind::Face, 1), plane(20.0, 300.0)),
        ]),
    };
    let e = vec![
        (
            input(0, face(0)),
            Relation::Modified(vec![face(0), face(1)].into()),
        ),
        (input(0, face(1)), Relation::Modified(vec![face(2)].into())),
        (input(0, face(2)), Relation::Modified(vec![face(2)].into())),
    ];
    let mut h = history(e);
    h.roles = vec![(face(3), OpRole::CornerPatch)];
    let mut b = NamingBuilder::new(F(5));
    b.from_history(&[(0, OperandNaming::Named(&base))], &h);
    let (n, _) = b.finish_topo(&topo).unwrap();
    // Larger piece first.
    assert_eq!(n.faces[1], split(&base.faces[0], 5, 0, 2));
    assert_eq!(n.faces[0], split(&base.faces[0], 5, 1, 2));
    assert_eq!(
        n.faces[2],
        base.faces[1].with(NameSeg::Merge {
            feature: F(5),
            with: vec![base.faces[2].clone()]
        })
    );
    // Role-only corner: named from its neighbours.
    let NameSeg::Gen {
        role,
        from: Some(NameSource::Topo(t)),
        ..
    } = &n.faces[3].segments()[0]
    else {
        panic!("{:?}", n.faces[3])
    };
    assert_eq!(*role, Role::new("Corner", 0));
    let Some(NameSeg::Bound { faces, .. }) = t.origin() else {
        panic!()
    };
    assert_eq!(faces.len(), 2);
}

#[test]
fn generated_sources_tags_selections_dedup_and_unexplained() {
    let base = named_strip();
    let edge = |i: u32| SubShape {
        kind: SubKind::Edge,
        index: i,
    };
    let topo = SyntheticTopology {
        faces: 4,
        edge_faces: vec![vec![0, 1], vec![1, 2], vec![2, 3]],
        vertex_edges: Vec::new(),
        fingerprints: HashMap::from([
            ((TopoKind::Face, 0), plane(10.0, 100.0)),
            ((TopoKind::Face, 1), plane(20.0, 50.0)),
        ]),
    };
    // Tagged edge 5 of operand 1 generates faces 0 and 1 (dedup Split); edge 0 of the named
    // operand 0, picked with key 7, generates face 2; face 3 has no source and no role.
    let tags = [(EdgeId(5), Tag(42))];
    let mut h = history(vec![
        (
            input(1, edge(5)),
            Relation::Generated(vec![face(0), face(1)].into()),
        ),
        (input(0, edge(0)), Relation::Generated(vec![face(2)].into())),
    ]);
    h.roles = vec![
        (face(0), OpRole::Lateral),
        (face(1), OpRole::Lateral),
        (face(2), OpRole::FilletBlend { contour: 0 }),
    ];
    let mut b = NamingBuilder::new(F(6));
    b.from_history(
        &[
            (0, OperandNaming::Named(&base)),
            (1, OperandNaming::Tagged(&tags)),
        ],
        &h,
    );
    let curve = NameSource::SketchCurve {
        sketch: SketchId(F(6)),
        curve: SketchEntityKey(42),
    };
    b.tag(Tag(42), curve.clone());
    b.selection(0, edge(0), 7);
    let (n, report) = b.finish_topo(&topo).unwrap();
    let lateral = TopoName::new(vec![NameSeg::Gen {
        feature: F(6),
        role: Role::new("Lateral", 0),
        from: Some(curve),
    }]);
    assert_eq!(n.faces[0], split(&lateral, 6, 0, 2));
    assert_eq!(n.faces[1], split(&lateral, 6, 1, 2));
    assert_eq!(
        n.faces[2],
        TopoName::new(vec![NameSeg::Gen {
            feature: F(6),
            role: Role::new("Blend", 0),
            from: Some(NameSource::Selection { key: 7 }),
        }])
    );
    assert_eq!(report.unexplained, vec![3]);
    let NameSeg::Gen { role, .. } = &n.faces[3].segments()[0] else {
        panic!()
    };
    assert_eq!(*role, Role::new("Unexplained", 0));
    // The input faces of the named operand have no history entry: no tombstones.
    assert!(n.removed.is_empty());
}

#[test]
fn tombstones_record_deleted_and_consumed_faces() {
    let base = named_strip();
    let mut carried = base.clone();
    carried.removed = Arc::new(BTreeMap::from([(gen_name(0, "Old"), F(0))]));
    let topo = SyntheticTopology {
        faces: 2,
        edge_faces: vec![vec![0, 1]],
        vertex_edges: Vec::new(),
        fingerprints: HashMap::new(),
    };
    let e = vec![
        (input(0, face(0)), Relation::Kept(face(0))),
        (input(0, face(1)), Relation::Deleted),
        (input(0, face(2)), Relation::Generated(vec![face(1)].into())),
    ];
    let mut b = NamingBuilder::new(F(3));
    b.from_history(&[(0, OperandNaming::Named(&carried))], &history(e));
    let (n, _) = b.finish_topo(&topo).unwrap();
    assert_eq!(n.removed.get(&base.faces[1]), Some(&F(3)));
    assert_eq!(n.removed.get(&base.faces[2]), Some(&F(3)));
    assert_eq!(
        n.removed.get(&gen_name(0, "Old")),
        Some(&F(0)),
        "carried over"
    );
}

#[test]
fn builder_errors() {
    let e = vec![(input(3, face(0)), Relation::Kept(face(0)))];
    let mut b = NamingBuilder::new(F(3));
    b.from_history(&[], &history(e));
    assert_eq!(
        b.finish_topo(&strip()).unwrap_err(),
        NamingError::MissingOperand(3)
    );
    let base = named_strip();
    let e = vec![(input(0, face(0)), Relation::Kept(face(9)))];
    let mut b = NamingBuilder::new(F(3));
    b.from_history(&[(0, OperandNaming::Named(&base))], &history(e));
    assert!(matches!(
        b.finish_topo(&strip()),
        Err(NamingError::CountMismatch { .. })
    ));
}

#[test]
fn deep_nesting_is_capped_by_digest() {
    let mut name = gen_name(1, "Seed");
    for f in 2..20 {
        let src = if name.depth() + 1 > MAX_NAME_DEPTH {
            NameSource::Digest(identity_digest(&name))
        } else {
            NameSource::Topo(name.clone())
        };
        name = TopoName::new(vec![NameSeg::Gen {
            feature: F(f),
            role: Role::new("Blend", 0),
            from: Some(src),
        }]);
        assert!(name.depth() <= MAX_NAME_DEPTH, "{}", name.depth());
    }
    // The builder applies the same cap for Topo sources.
    let base = BodyNaming::new(vec![name.clone()], vec![name], Vec::new());
    let topo = SyntheticTopology {
        faces: 1,
        edge_faces: vec![vec![]],
        vertex_edges: Vec::new(),
        fingerprints: HashMap::new(),
    };
    let e = vec![(
        input(
            0,
            SubShape {
                kind: SubKind::Edge,
                index: 0,
            },
        ),
        Relation::Generated(vec![face(0)].into()),
    )];
    let mut b = NamingBuilder::new(F(30));
    b.from_history(&[(0, OperandNaming::Named(&base))], &history(e));
    let (n, _) = b.finish_topo(&topo).unwrap();
    assert!(n.faces[0].depth() <= MAX_NAME_DEPTH);
}

#[test]
fn instanced_names_are_distinct_and_rederived() {
    let base = named_strip();
    let copy = base.instanced_names(F(9), 3);
    assert_eq!(copy.faces.len(), base.faces.len());
    assert!(copy.faces.iter().all(|f| !base.faces.contains(f)));
    let Some(NameSeg::Bound { faces, .. }) = copy.edges[0].origin() else {
        panic!()
    };
    assert!(
        faces.iter().all(|f| copy.faces.contains(f)),
        "edges name the copy's faces"
    );
    // A ref into instance 3 resolves exactly in the copy and not in the seed.
    let v = View::with(copy.clone());
    let x = v
        .resolve(&r(TopoKind::Edge, copy.edges[0].clone(), None))
        .unwrap();
    assert_eq!(x.quality, MatchQuality::Exact);
    let seed = View::with(base.clone());
    assert!(
        seed.resolve(&r(TopoKind::Edge, copy.edges[0].clone(), None))
            .is_err()
    );
}

#[test]
fn mesh_groups_are_named_and_resolved() {
    let mut groups = amg_mesh::FaceGroups::new(2);
    let g2 = groups.alloc(amg_mesh::FaceGroupInfo::new(amg_mesh::GroupOrigin::User));
    groups.set(amg_mesh::FaceId(1), g2);
    let n = BodyNaming::for_mesh(F(4), &groups, None);
    assert_eq!(n.groups.len(), 2);
    let solid = named_strip();
    let map = |g: u32| (g == g2.0).then_some(2);
    let n2 = BodyNaming::for_mesh(F(4), &groups, Some((&solid, &map)));
    assert_eq!(
        n2.groups[&g2.0], solid.faces[2],
        "tessellated group keeps the face name"
    );
    let mut v = View::default();
    v.bodies.insert(
        body(1),
        (
            BodyGeom::Mesh(Arc::new(amg_mesh::IndexedMesh::default())),
            n2.clone(),
        ),
    );
    let x = v
        .resolve(&r(TopoKind::FaceGroup, solid.faces[2].clone(), None))
        .unwrap();
    assert_eq!((x.index, x.quality), (g2.0, MatchQuality::Exact));
}

#[test]
fn interned_wire_round_trips_and_shares_names() {
    let mut n = named_strip();
    n.removed = Arc::new(BTreeMap::from([(gen_name(0, "Old"), F(0))]));
    n.groups.insert(5, n.faces[0].clone());
    let w = WireNaming::encode(&n);
    let json = serde_json::to_vec(&w).unwrap();
    let back: WireNaming = serde_json::from_slice(&json).unwrap();
    let d = back.decode().unwrap();
    assert_eq!(d, n);
    // The face inside edge 0's Bound is the same allocation as the face itself.
    let Some(NameSeg::Bound { faces, .. }) = d.edges[0].origin() else {
        panic!()
    };
    assert!(
        faces
            .iter()
            .any(|f| f.ptr_eq(&d.faces[0]) || f.ptr_eq(&d.faces[1]))
    );
    let plain = serde_json::to_vec(&n).unwrap();
    assert!(
        json.len() < plain.len(),
        "interned {} < plain {}",
        json.len(),
        plain.len()
    );
}
