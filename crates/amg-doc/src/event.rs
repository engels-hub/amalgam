//! Change events (S01 §9, DOC-09).
//!
//! [`DocEvent`]s carry ids only, never geometry. They come from two comparisons:
//! - **authored:** `OrdMap::diff` of the maps of the design the consumers last saw and the
//!   current one (shared subtrees are skipped, so the cost follows the size of the change);
//! - **evaluated:** `Arc::ptr_eq` of body/sketch allocations and health equality between the
//!   evaluated state last seen and the one swapped in.
//!
//! The authored diff is taken lazily, at [`Document::drain_events`] (and before a recompute
//! starts or is swapped in, to keep the order authored → recompute). Because [`History`] owns
//! `apply`, transactions, undo and redo, one diff against the last seen snapshot covers all of
//! them uniformly and coalesces several edits between two frames into their net effect (an
//! edit that is undone before the next drain emits nothing).
//!
//! [`Document::drain_events`]: crate::Document::drain_events
//! [`History`]: crate::History

use std::collections::BTreeSet;
use std::sync::Arc;

use crate::eval::{Evaluated, Health};
use crate::id::{BodyId, ConstructionId, ExtKey, FeatureId, OccurrenceId, Revision, SketchId};
use crate::model::{Design, DesignSnapshot};
use crate::pcoll::{PDiff, PMap};

/// A change notification for the UI and other consumers. Ids only; read the new state from
/// `Document::design()` / `Document::evaluated()`.
#[derive(Clone, Debug, PartialEq)]
pub enum DocEvent {
    /// A body appeared in the evaluated state.
    BodyAdded(BodyId),
    /// A body disappeared from the evaluated state.
    BodyRemoved(BodyId),
    /// A body's geometry allocation changed.
    BodyGeometryChanged(BodyId),
    /// A body's properties (name, material, appearance, visibility, opacity) changed.
    BodyPropsChanged(BodyId),
    /// Components or occurrences were added, removed, renamed or re-parented.
    ComponentTreeChanged,
    /// An occurrence's placement changed.
    OccurrenceTransformChanged(OccurrenceId),
    /// An occurrence was shown or hidden.
    OccurrenceVisibilityChanged(OccurrenceId),
    /// A solved sketch changed, appeared or disappeared.
    SketchChanged(SketchId),
    /// Construction geometry changed, appeared or disappeared.
    ConstructionChanged(ConstructionId),
    /// The timeline changed (features, order, marker, suppression, names, groups).
    TimelineChanged,
    /// A timeline node's health changed.
    HealthChanged(FeatureId),
    /// A parameter was added, removed or edited.
    ParamsChanged,
    /// An extension record changed.
    ExtChanged(ExtKey),
    /// The configuration table changed: a configuration was added, removed, renamed or
    /// edited, or another one was activated (DOC-10). Activating a configuration, or editing
    /// the active one, additionally raises `ParamsChanged` / `TimelineChanged` when the
    /// effective parameter expressions / suppression flags differ, and the following
    /// recompute raises the geometry and health events as usual.
    ConfigsChanged,
    /// A recompute job started for this revision.
    RecomputeStarted(Revision),
    /// A recompute result was swapped in; `failed` nodes have an error.
    RecomputeFinished {
        /// The revision the result belongs to.
        revision: Revision,
        /// Number of nodes whose health is an error.
        failed: u32,
    },
    /// The document was created, opened or recovered: consumers rebuild everything.
    DocumentReplaced,
}

/// What the consumers have seen so far, and the events they have not drained yet.
#[derive(Debug)]
pub(crate) struct EventState {
    seen_design: DesignSnapshot,
    seen_evaluated: Arc<Evaluated>,
    queue: Vec<DocEvent>,
}

impl EventState {
    /// A fresh document: consumers start by rebuilding everything.
    pub(crate) fn new(design: DesignSnapshot, evaluated: Arc<Evaluated>) -> EventState {
        EventState {
            seen_design: design,
            seen_evaluated: evaluated,
            queue: vec![DocEvent::DocumentReplaced],
        }
    }

    /// Queues the authored changes between the last seen design and `current`.
    pub(crate) fn sync_authored(&mut self, current: &DesignSnapshot) {
        if Arc::ptr_eq(&self.seen_design, current) {
            return;
        }
        authored_events(&self.seen_design, current, &mut self.queue);
        self.seen_design = Arc::clone(current);
    }

    /// Queues the geometry/health changes of a swapped-in evaluated state, then
    /// `RecomputeFinished`.
    pub(crate) fn swap_evaluated(&mut self, new: &Arc<Evaluated>, revision: Revision) {
        if !Arc::ptr_eq(&self.seen_evaluated, new) {
            evaluated_events(&self.seen_evaluated, new, &mut self.queue);
            self.seen_evaluated = Arc::clone(new);
        }
        let failed = new
            .nodes()
            .filter(|(_, n)| matches!(n.health, Health::Error(_)))
            .count();
        self.queue.push(DocEvent::RecomputeFinished {
            revision,
            failed: u32::try_from(failed).unwrap_or(u32::MAX),
        });
    }

    pub(crate) fn recompute_started(&mut self, revision: Revision) {
        self.queue.push(DocEvent::RecomputeStarted(revision));
    }

    pub(crate) fn take(&mut self) -> Vec<DocEvent> {
        std::mem::take(&mut self.queue)
    }
}

/// Keys whose value differs between `a` and `b` (added, removed or updated to an unequal
/// value), in key order, with the value on each side.
fn changed<'a, 'b, K, V>(
    a: &'a PMap<K, Arc<V>>,
    b: &'b PMap<K, Arc<V>>,
) -> Vec<(K, Option<&'a V>, Option<&'b V>)>
where
    K: Ord + Clone,
    V: PartialEq,
{
    let mut out = Vec::new();
    for item in a.diff(b) {
        match item {
            PDiff::Add(k, v) => out.push((k.clone(), None, Some(&**v))),
            PDiff::Remove(k, v) => out.push((k.clone(), Some(&**v), None)),
            PDiff::Update { old, new } => {
                if *old.1 != *new.1 {
                    out.push((old.0.clone(), Some(&**old.1), Some(&**new.1)));
                }
            }
        }
    }
    out
}

/// Events for the difference between two authored designs.
fn authored_events(old: &Design, new: &Design, out: &mut Vec<DocEvent>) {
    // Timeline: nodes (in order), marker, groups.
    let timeline_changed = old.timeline().marker() != new.timeline().marker()
        || old.timeline().len() != new.timeline().len()
        || old.timeline().nodes().ne(new.timeline().nodes())
        || old.timeline().groups().ne(new.timeline().groups());
    // Configurations (DOC-10): the table itself, and what the active one changes for the
    // consumers of parameters and the timeline.
    let (oc, nc) = (old.configs(), new.configs());
    if oc != nc {
        out.push(DocEvent::ConfigsChanged);
    }
    let (oa, na) = (old.active_config_data(), new.active_config_data());
    let eff_params = old.params != new.params || oa.map(|c| &c.params) != na.map(|c| &c.params);
    let eff_supp = oa.map(|c| &c.suppressed) != na.map(|c| &c.suppressed);

    if timeline_changed || eff_supp {
        out.push(DocEvent::TimelineChanged);
    }
    if eff_params {
        out.push(DocEvent::ParamsChanged);
    }

    // Component tree and occurrences.
    let mut tree = !changed(&old.components, &new.components).is_empty();
    let mut occ_events = Vec::new();
    for (id, o, n) in changed(&old.occurrences, &new.occurrences) {
        match (o, n) {
            (Some(o), Some(n)) => {
                if o.transform != n.transform {
                    occ_events.push(DocEvent::OccurrenceTransformChanged(id));
                }
                if o.visible != n.visible {
                    occ_events.push(DocEvent::OccurrenceVisibilityChanged(id));
                }
                let (mut a, b) = (o.clone(), n.clone());
                a.transform = b.transform;
                a.visible = b.visible;
                tree |= a != b;
            }
            _ => tree = true,
        }
    }
    if tree {
        out.push(DocEvent::ComponentTreeChanged);
    }
    out.append(&mut occ_events);

    // Body properties; a body whose props record appears or disappears also changed.
    for (id, _, _) in changed(&old.body_props, &new.body_props) {
        out.push(DocEvent::BodyPropsChanged(id));
    }
    for (key, _, _) in changed(&old.extensions, &new.extensions) {
        out.push(DocEvent::ExtChanged(key));
    }
}

/// Events for the difference between two evaluated states.
fn evaluated_events(old: &Evaluated, new: &Evaluated, out: &mut Vec<DocEvent>) {
    let mut ids: BTreeSet<BodyId> = old.bodies().map(|(id, _)| id).collect();
    ids.extend(new.bodies().map(|(id, _)| id));
    for id in ids {
        match (old.body(id), new.body(id)) {
            (None, Some(_)) => out.push(DocEvent::BodyAdded(id)),
            (Some(_), None) => out.push(DocEvent::BodyRemoved(id)),
            (Some(a), Some(b)) => {
                if Arc::ptr_eq(a, b) {
                    continue;
                }
                if !a.geom.same_as(&b.geom) {
                    out.push(DocEvent::BodyGeometryChanged(id));
                } else if a.component != b.component {
                    out.push(DocEvent::ComponentTreeChanged);
                }
            }
            (None, None) => {}
        }
    }

    let mut ids: BTreeSet<SketchId> = old.sketches().map(|(id, _)| id).collect();
    ids.extend(new.sketches().map(|(id, _)| id));
    for id in ids {
        let same = match (old.sketch(id), new.sketch(id)) {
            (Some(a), Some(b)) => Arc::ptr_eq(a, b),
            (None, None) => true,
            _ => false,
        };
        if !same {
            out.push(DocEvent::SketchChanged(id));
        }
    }

    let mut ids: BTreeSet<ConstructionId> = old.constructions().map(|(id, _)| id).collect();
    ids.extend(new.constructions().map(|(id, _)| id));
    for id in ids {
        if old.construction(id) != new.construction(id) {
            out.push(DocEvent::ConstructionChanged(id));
        }
    }

    let mut ids: BTreeSet<FeatureId> = old.nodes().map(|(id, _)| id).collect();
    ids.extend(new.nodes().map(|(id, _)| id));
    for id in ids {
        let health = |e: &Evaluated| e.node(id).map(|n| n.health.clone());
        if health(old) != health(new) {
            out.push(DocEvent::HealthChanged(id));
        }
    }
}
