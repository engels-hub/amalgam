//! DOC-15: context-body inputs (`Feature::context_reads`) and the cache key that covers them.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    missing_docs,
    reason = "tests"
)]

use std::sync::Arc;

use amg_doc::stub::IndexedMesh;
use amg_doc::testing::TestCounter;
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, CacheKey, ComponentId, ComputeMode,
    ContextBodies, Dependency, Design, Feature, FeatureBox, FeatureError, FeatureId, FeatureOutput,
    GeomCache, RecomputeCtx, RecomputeResult, recompute,
};
use amg_jobs::JobCtx;
use serde::{Deserialize, Serialize};

/// Which scope a [`Reader`] declares (a component is named by its raw id).
#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
enum Scope {
    Nothing,
    Context,
    Component(u64),
    All,
}

/// Reads no listed input; creates one mesh body, so it can be read by others.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Reader {
    scope: Scope,
}

impl Feature for Reader {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn context_reads(&self) -> Option<ContextBodies> {
        match self.scope {
            Scope::Nothing => None,
            Scope::Context => Some(ContextBodies::Context),
            Scope::Component(c) => Some(ContextBodies::Component(ComponentId(c))),
            Scope::All => Some(ContextBodies::All),
        }
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Ok(FeatureOutput {
            bodies: vec![BodyChange {
                id: ctx.new_body_id(0),
                change: BodyChangeKind::Created {
                    geom: BodyGeom::Mesh(Arc::new(IndexedMesh::default())),
                    naming: None,
                },
            }],
            ..FeatureOutput::default()
        })
    }
}

amg_doc::register_feature!(Reader, "test.doc15_reader", version = 1);

/// Moves a body into another component.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Mover {
    body: BodyId,
    to: ComponentId,
}

impl Feature for Mover {
    fn inputs(&self) -> Vec<Dependency> {
        vec![Dependency::Body(self.body)]
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Ok(FeatureOutput {
            bodies: vec![BodyChange {
                id: self.body,
                change: BodyChangeKind::Moved {
                    component: self.to,
                    geom: ctx.body(self.body)?.clone(),
                    naming: None,
                },
            }],
            ..FeatureOutput::default()
        })
    }
}

amg_doc::register_feature!(Mover, "test.doc15_mover", version = 1);

/// A design with a second component, one body in each (`here` in the root, `there` in
/// `other`), and one reader per scope in the root.
struct Fixture {
    design: Design,
    cache: Arc<GeomCache>,
    other: ComponentId,
    here: FeatureId,
    there: FeatureId,
    nothing: FeatureId,
    context: FeatureId,
    of_other: FeatureId,
    all: FeatureId,
}

fn body(f: FeatureId) -> BodyId {
    BodyId { origin: f, slot: 0 }
}

fn counter(deps: &[BodyId], value: i64, fail: bool) -> FeatureBox {
    FeatureBox::new(TestCounter {
        deps: deps.iter().copied().map(Dependency::Body).collect(),
        value,
        fail,
    })
}

fn add_at(d: &mut Design, at: Option<usize>, c: ComponentId, f: FeatureBox) -> FeatureId {
    d.add_feature(at, f, c, None).unwrap()
}

impl Fixture {
    fn new() -> Fixture {
        let mut design = Design::new("context-reads");
        let root = design.root();
        let (other, _) = design
            .add_component(root, "other", amg_core::Transform::IDENTITY)
            .unwrap();
        let here = add_at(&mut design, None, root, counter(&[], 0, false));
        let there = add_at(&mut design, None, other, counter(&[], 0, false));
        let mut reader = |scope| add_at(&mut design, None, root, FeatureBox::new(Reader { scope }));
        let nothing = reader(Scope::Nothing);
        let context = reader(Scope::Context);
        let of_other = reader(Scope::Component(other.0));
        let all = reader(Scope::All);
        Fixture {
            design,
            cache: GeomCache::new(1 << 30),
            other,
            here,
            there,
            nothing,
            context,
            of_other,
            all,
        }
    }

    fn run(&self) -> RecomputeResult {
        recompute(
            &self.design,
            &self.cache,
            ComputeMode::Full,
            &JobCtx::detached(),
        )
        .unwrap()
    }

    /// Which of the four readers (nothing, context, other component, all) computed.
    fn readers_computed(&self, r: &RecomputeResult) -> [bool; 4] {
        [self.nothing, self.context, self.of_other, self.all].map(|f| r.computed.contains(&f))
    }

    /// The timeline index of the first reader: inserting there is "before the readers".
    fn before_readers(&self) -> usize {
        self.design.timeline().node(self.nothing).unwrap().0
    }

    fn key(r: &RecomputeResult, f: FeatureId) -> CacheKey {
        r.evaluated.node(f).unwrap().key.unwrap()
    }
}

const NONE: [bool; 4] = [false; 4];

#[test]
fn unchanged_design_hits_for_every_scope() {
    let f = Fixture::new();
    assert_eq!(f.readers_computed(&f.run()), [true; 4]);
    assert_eq!(f.readers_computed(&f.run()), NONE);
}

#[test]
fn editing_a_body_recomputes_the_scopes_that_hold_it() {
    let mut f = Fixture::new();
    let first = f.run();

    f.design
        .replace_feature(f.here, counter(&[], 1, false))
        .unwrap();
    // nothing, context, other, all
    assert_eq!(f.readers_computed(&f.run()), [false, true, false, true]);

    f.design
        .replace_feature(f.there, counter(&[], 1, false))
        .unwrap();
    assert_eq!(f.readers_computed(&f.run()), [false, false, true, true]);

    // Reverting both is served from the cache, with the first keys.
    f.design
        .replace_feature(f.here, counter(&[], 0, false))
        .unwrap();
    f.design
        .replace_feature(f.there, counter(&[], 0, false))
        .unwrap();
    let back = f.run();
    assert!(back.computed.is_empty(), "{:?}", back.computed);
    for r in [f.nothing, f.context, f.of_other, f.all] {
        assert_eq!(Fixture::key(&back, r), Fixture::key(&first, r));
    }
}

#[test]
fn a_modifier_inserted_before_the_reader_is_seen() {
    let mut f = Fixture::new();
    f.run();
    let root = f.design.root();
    let at = f.before_readers();
    let m = add_at(
        &mut f.design,
        Some(at),
        root,
        counter(&[body(f.here)], 5, false),
    );
    // The modifier rewrites `here` and adds a body in the root.
    assert_eq!(f.readers_computed(&f.run()), [false, true, false, true]);

    f.design
        .replace_feature(m, counter(&[body(f.here)], 6, false))
        .unwrap();
    assert_eq!(f.readers_computed(&f.run()), [false, true, false, true]);

    // Suppressing it restores the earlier state: served from the cache.
    f.design.set_suppressed(m, true).unwrap();
    assert_eq!(f.readers_computed(&f.run()), NONE);
}

#[test]
fn new_and_removed_bodies_change_the_key() {
    let mut f = Fixture::new();
    f.run();
    let at = f.before_readers();
    let other = f.other;
    let new = add_at(&mut f.design, Some(at), other, counter(&[], 9, false));
    assert_eq!(f.readers_computed(&f.run()), [false, false, true, true]);
    f.design.remove_feature(new).unwrap();
    assert_eq!(f.readers_computed(&f.run()), NONE);

    // Removing a body that was there before: a state not computed yet.
    f.design.remove_feature(f.here).unwrap();
    assert_eq!(f.readers_computed(&f.run()), [false, true, false, true]);
}

#[test]
fn a_body_moved_between_components_changes_both() {
    let mut f = Fixture::new();
    f.run();
    let at = f.before_readers();
    let mover = Mover {
        body: body(f.there),
        to: f.design.root(),
    };
    let root = f.design.root();
    add_at(&mut f.design, Some(at), root, FeatureBox::new(mover));
    assert_eq!(f.readers_computed(&f.run()), [false, true, true, true]);
}

#[test]
fn unrelated_edits_still_hit_the_cache() {
    let mut f = Fixture::new();
    f.run();
    let root = f.design.root();

    // A body created after the readers is not "before" them.
    let late = add_at(&mut f.design, None, root, counter(&[], 3, false));
    assert_eq!(f.readers_computed(&f.run()), NONE);
    f.design
        .replace_feature(late, counter(&[], 4, false))
        .unwrap();
    let r = f.run();
    assert_eq!(f.readers_computed(&r), NONE);
    assert_eq!(r.computed, vec![late]);

    // A failing feature before the readers has no effect, so it changes no body.
    let at = f.before_readers();
    let failing = add_at(
        &mut f.design,
        Some(at),
        root,
        counter(&[body(f.here)], 7, true),
    );
    let r = f.run();
    assert_eq!(r.computed, Vec::<FeatureId>::new());
    assert!(!r.cached.contains(&failing));
}

#[test]
fn a_declaration_adds_no_ordering_rule_or_missing_input() {
    let mut f = Fixture::new();
    // The reader of every body can move to the front, where there is none.
    f.design.move_feature(f.all, 0).unwrap();
    let r = f.run();
    assert_eq!(r.evaluated.node(f.all).unwrap().health, amg_doc::Health::Ok);
}
