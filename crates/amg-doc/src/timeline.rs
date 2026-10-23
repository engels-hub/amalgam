//! The parametric timeline (S01 §2): ordered feature nodes, rollback marker, suppression and
//! groups, plus the low-level timeline edits on [`Design`].
//!
//! Invariants (checked by every edit and by [`Design::validate`] on load):
//! - node ids are unique feature ids; `marker <= len`; each node's context is a component;
//! - **ordering rule:** every feature a node reads (`Feature::inputs`) is earlier in the
//!   timeline. A producer that is no longer in the timeline (deleted) is allowed: recompute
//!   reports the missing input. Adding or editing a feature requires its inputs to exist;
//! - groups are non-empty, contiguous runs of nodes, and a node is in at most one group;
//! - every feature is registered (or an [`UnknownFeature`](crate::UnknownFeature) from a
//!   file), at its current schema version, valid, and its JSON round-trips.

use std::collections::BTreeMap;
use std::ops::RangeInclusive;
use std::sync::Arc;

use serde::{Deserialize, Deserializer, Serialize, Serializer};

use crate::feature::{Dependency, FeatureBox};
use crate::id::{ComponentId, FeatureId, GroupId};
use crate::model::{Design, ModelError};
use crate::pcoll::{PMap, PSet, PVec, entries};
use crate::registry::registry;

/// Ordered feature nodes, the rollback marker and the groups.
#[derive(Clone, Debug, Default)]
pub struct Timeline {
    nodes: PVec<FeatureNode>,
    /// Number of rolled-forward nodes; nodes at index ≥ marker are not computed.
    marker: usize,
    groups: PMap<GroupId, Arc<Group>>,
    /// Node ids, for O(log n) membership (derived, not serialized).
    ids: PSet<FeatureId>,
}

/// One timeline entry. Health is evaluated state ([`NodeEval`](crate::NodeEval)), not here.
#[derive(Clone, Debug, PartialEq)]
pub struct FeatureNode {
    /// The feature's id (also names its bodies, sketch, construction).
    pub id: FeatureId,
    /// The feature itself.
    pub feature: FeatureBox,
    /// Suppressed features are skipped by recompute; their outputs pass through.
    pub suppressed: bool,
    /// The group this node belongs to.
    pub group: Option<GroupId>,
    /// User-given name (default: generated from the kind).
    pub name: Option<String>,
    /// Component the feature was created in.
    pub context: ComponentId,
}

/// A named, collapsible run of timeline nodes.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Group {
    /// Display name.
    pub name: String,
    /// Collapsed in the timeline UI.
    pub collapsed: bool,
}

/// (index, id) of the nearest producer before and the first dependent after a node.
type MoveBounds = (Option<(usize, FeatureId)>, Option<(usize, FeatureId)>);

impl Timeline {
    /// Number of nodes.
    pub fn len(&self) -> usize {
        self.nodes.len()
    }

    /// True if there are no nodes.
    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }

    /// Number of rolled-forward nodes (Fusion's marker position).
    pub fn marker(&self) -> usize {
        self.marker
    }

    /// All nodes in timeline order.
    pub fn nodes(&self) -> impl Iterator<Item = &FeatureNode> + '_ {
        self.nodes.iter()
    }

    /// The node at `index`.
    pub fn node_at(&self, index: usize) -> Option<&FeatureNode> {
        self.nodes.get(index)
    }

    /// A node and its index.
    pub fn node(&self, id: FeatureId) -> Option<(usize, &FeatureNode)> {
        if !self.ids.contains(&id) {
            return None;
        }
        self.nodes.iter().enumerate().find(|(_, n)| n.id == id)
    }

    /// True if `id` is a node of this timeline.
    pub fn contains(&self, id: FeatureId) -> bool {
        self.ids.contains(&id)
    }

    /// A group.
    pub fn group(&self, id: GroupId) -> Option<&Group> {
        self.groups.get(&id).map(|g| &**g)
    }

    /// All groups in id order.
    pub fn groups(&self) -> impl Iterator<Item = (GroupId, &Group)> + '_ {
        self.groups.iter().map(|(k, v)| (*k, &**v))
    }

    /// Index range of a group's nodes.
    pub fn group_range(&self, id: GroupId) -> Option<RangeInclusive<usize>> {
        let mut it = self
            .nodes
            .iter()
            .enumerate()
            .filter(|(_, n)| n.group == Some(id))
            .map(|(i, _)| i);
        let first = it.next()?;
        Some(first..=it.last().unwrap_or(first))
    }

    /// Nodes that read an output of `id`, in timeline order.
    pub fn dependents(&self, id: FeatureId) -> Vec<FeatureId> {
        self.nodes
            .iter()
            .filter(|n| self.reads(n, id))
            .map(|n| n.id)
            .collect()
    }

    /// Final indices `id` can be moved to without breaking the ordering rule: after its last
    /// input, before its first dependent (S11 R7). An unknown feature can't move (its inputs
    /// are unknown), so its range is its own index.
    pub fn valid_move_range(&self, id: FeatureId) -> Option<RangeInclusive<usize>> {
        let (i, node) = self.node(id)?;
        if node.feature.is_unknown() {
            return Some(i..=i);
        }
        let (lo, hi) = self.move_bounds(i, node);
        Some(lo.map_or(0, |(p, _)| p + 1)..=hi.map_or(self.len() - 1, |(d, _)| d - 1))
    }

    /// (index and id of the last producer, index and id of the first dependent) of node `i`.
    fn move_bounds(&self, i: usize, node: &FeatureNode) -> MoveBounds {
        let producers: Vec<FeatureId> = node
            .feature
            .inputs()
            .iter()
            .filter_map(Dependency::producer)
            .map(|p| self.effective_producer(p))
            .collect();
        let last_producer = self
            .nodes
            .iter()
            .take(i)
            .enumerate()
            .filter(|(_, n)| producers.contains(&n.id))
            .map(|(j, n)| (j, n.id))
            .last();
        let first_dependent = self
            .nodes
            .iter()
            .enumerate()
            .skip(i + 1)
            .find(|(_, n)| self.reads(n, node.id))
            .map(|(j, n)| (j, n.id));
        (last_producer, first_dependent)
    }

    /// The group a node inserted at `at` (in the list without it) joins: the group around it,
    /// else `own` if it stays adjacent to it.
    fn group_at(&self, at: usize, own: Option<GroupId>) -> Option<GroupId> {
        let left = at
            .checked_sub(1)
            .and_then(|i| self.nodes.get(i))
            .and_then(|n| n.group);
        let right = self.nodes.get(at).and_then(|n| n.group);
        if left.is_some() && left == right {
            left
        } else if own.is_some() && (own == left || own == right) {
            own
        } else {
            None
        }
    }

    /// Removes `group` if no node is in it any more.
    fn drop_group_if_empty(&mut self, group: Option<GroupId>) {
        if let Some(g) = group
            && !self.nodes.iter().any(|n| n.group == Some(g))
        {
            self.groups.remove(&g);
        }
    }

    /// The timeline node that provides what `producer` (a body's `origin`, ...) names: the
    /// node itself, or, for the origin of a body adopted by a base feature (DOC-08), that
    /// base feature. Ids that are neither are returned unchanged.
    pub(crate) fn effective_producer(&self, producer: FeatureId) -> FeatureId {
        if self.ids.contains(&producer) {
            return producer;
        }
        self.nodes
            .iter()
            .find(|n| {
                n.feature
                    .downcast_ref::<crate::direct::BaseFeature>()
                    .is_some_and(|b| b.owns(producer))
            })
            .map_or(producer, |n| n.id)
    }

    /// True if node `n` reads an output of `id`.
    fn reads(&self, n: &FeatureNode, id: FeatureId) -> bool {
        n.feature
            .inputs()
            .iter()
            .filter_map(Dependency::producer)
            .any(|p| self.effective_producer(p) == id)
    }

    fn index_of(&self, id: FeatureId) -> Result<usize, ModelError> {
        self.node(id).map(|(i, _)| i).ok_or(ModelError::NotFound {
            what: "feature",
            id: id.0,
        })
    }

    /// Sets a node's suppression flag without touching any revision (configurations apply
    /// their overrides to a scratch copy this way, DOC-10). Unknown ids are ignored.
    pub(crate) fn force_suppressed(&mut self, id: FeatureId, suppressed: bool) {
        if let Some((i, n)) = self.node(id)
            && n.suppressed != suppressed
        {
            self.update(i, |n| n.suppressed = suppressed);
        }
    }

    fn update(&mut self, i: usize, f: impl FnOnce(&mut FeatureNode)) {
        if let Some(n) = self.nodes.get_mut(i) {
            f(n);
        }
    }
}

fn invalid(field: &'static str, msg: impl Into<String>) -> ModelError {
    ModelError::InvalidValue {
        field,
        msg: msg.into(),
    }
}

/// Checks a feature that is about to enter a design: known, current version, valid, and its
/// JSON round-trips through the registry byte for byte (this rejects NaN/inf fields, which
/// JSON can't hold, and asymmetric serde impls).
fn check_new_feature(f: &FeatureBox) -> Result<(), ModelError> {
    if f.is_unknown() {
        return Err(invalid(
            "feature",
            format!(
                "unknown feature kind `{}` can only come from a file",
                f.kind()
            ),
        ));
    }
    let Some(reg) = registry().get(f.kind()) else {
        return Err(invalid(
            "feature",
            format!("kind `{}` is not registered", f.kind()),
        ));
    };
    if reg.version != f.version() {
        return Err(invalid(
            "feature",
            format!(
                "`{}` reports version {} but is registered as {}",
                f.kind(),
                f.version(),
                reg.version
            ),
        ));
    }
    f.validate()
        .map_err(|e| invalid("feature", e.to_string()))?;
    let json = serde_json::to_string(f).map_err(|e| invalid("feature", e.to_string()))?;
    let back: FeatureBox = serde_json::from_str(&json).map_err(|e| {
        invalid(
            "feature",
            format!("`{}` does not load back after saving: {e}", f.kind()),
        )
    })?;
    match serde_json::to_string(&back) {
        Ok(again) if again == json => Ok(()),
        _ => Err(invalid(
            "feature",
            format!("`{}` saves differently after a reload", f.kind()),
        )),
    }
}

impl Design {
    /// The parametric timeline.
    pub fn timeline(&self) -> &Timeline {
        &self.timeline
    }

    // ---- timeline edits (low level; DOC-05 commands build on these) ---------------------

    /// Inserts a feature at index `at` (`None`: at the marker) in component `context`.
    ///
    /// Inserting at or before the marker rolls the marker forward by one, so the new feature
    /// is computed. A node inserted inside a group joins it. Every input must be produced by an
    /// earlier node ([`ModelError::WouldBreakDependency`] otherwise, `NotFound` if the
    /// producer doesn't exist); every occurrence it reads must exist.
    pub fn add_feature(
        &mut self,
        at: Option<usize>,
        feature: FeatureBox,
        context: ComponentId,
        name: Option<String>,
    ) -> Result<FeatureId, ModelError> {
        let at = at.unwrap_or(self.timeline.marker);
        if at > self.timeline.len() {
            return Err(invalid(
                "index",
                format!("{at} is past the end ({})", self.timeline.len()),
            ));
        }
        self.require_component(context)?;
        check_new_feature(&feature)?;
        self.ensure_ids(1)?;
        // The id isn't allocated yet, so the feature can't reference itself.
        self.check_inputs(&feature, FeatureId(self.meta.next_id), at)?;
        let id = FeatureId(self.alloc()?);
        let group = self.timeline.group_at(at, None);
        let tl = &mut self.timeline;
        tl.nodes.insert(
            at,
            FeatureNode {
                id,
                feature,
                suppressed: false,
                group,
                name,
                context,
            },
        );
        tl.ids.insert(id);
        if at <= tl.marker {
            tl.marker += 1;
        }
        self.bump();
        Ok(id)
    }

    /// Replaces a feature's definition (an edit). The kind can't change; the new inputs must
    /// satisfy the ordering rule at the node's position. Unknown features can't be edited.
    pub fn replace_feature(
        &mut self,
        id: FeatureId,
        feature: FeatureBox,
    ) -> Result<(), ModelError> {
        let i = self.timeline.index_of(id)?;
        let old = &self.timeline.nodes[i].feature;
        if old.is_unknown() {
            return Err(invalid("feature", "an unknown feature can't be edited"));
        }
        if old.kind() != feature.kind() {
            return Err(invalid(
                "feature",
                format!(
                    "kind can't change (`{}` → `{}`)",
                    old.kind(),
                    feature.kind()
                ),
            ));
        }
        check_new_feature(&feature)?;
        self.check_inputs(&feature, id, i)?;
        if self.timeline.nodes[i].feature == feature {
            return Ok(());
        }
        self.timeline.update(i, |n| n.feature = feature);
        self.bump();
        Ok(())
    }

    /// Deletes a feature together with the properties and baked payloads of its bodies.
    /// Features that read it stay and fail at recompute with a missing input (DOC-05 decides
    /// the user-facing cascade).
    pub fn remove_feature(&mut self, id: FeatureId) -> Result<(), ModelError> {
        let i = self.timeline.index_of(id)?;
        let tl = &mut self.timeline;
        let node = tl.nodes.remove(i);
        tl.ids.remove(&id);
        if i < tl.marker {
            tl.marker -= 1;
        }
        tl.drop_group_if_empty(node.group);
        let range = crate::model::origin_range(id);
        let props: Vec<_> = self
            .body_props
            .range(range.clone())
            .map(|(k, _)| *k)
            .collect();
        for b in props {
            self.body_props.remove(&b);
        }
        let mut baked: Vec<_> = self.baked.range(range).map(|(k, _)| *k).collect();
        if let Some(base) = node.feature.downcast_ref::<crate::direct::BaseFeature>() {
            // A base feature owns the bodies it adopted (DOC-08).
            for b in &base.bodies {
                baked.push(b.id);
                self.body_props.remove(&b.id);
            }
        }
        for b in baked {
            self.baked.remove(&b);
        }
        self.bump();
        Ok(())
    }

    /// Moves a feature to final index `to`.
    ///
    /// Fails with [`ModelError::WouldBreakDependency`] if that puts it before one of its inputs
    /// or after one of its dependents. The marker stays between the same pair of other nodes;
    /// a node moved exactly onto the marker keeps its rolled-forward state. Moving into a group
    /// joins it; moving away from the own group leaves it.
    pub fn move_feature(&mut self, id: FeatureId, to: usize) -> Result<(), ModelError> {
        let (i, node) = self.timeline.node(id).ok_or(ModelError::NotFound {
            what: "feature",
            id: id.0,
        })?;
        if to >= self.timeline.len() {
            return Err(invalid(
                "index",
                format!("{to} is past the last node ({})", self.timeline.len() - 1),
            ));
        }
        if to == i {
            return Ok(());
        }
        if node.feature.is_unknown() {
            return Err(invalid(
                "feature",
                "an unknown feature can't be moved (its inputs are unknown)",
            ));
        }
        let (lo, hi) = self.timeline.move_bounds(i, node);
        if let Some((p, input)) = lo
            && to <= p
        {
            return Err(ModelError::WouldBreakDependency { feature: id, input });
        }
        if let Some((d, dependent)) = hi
            && to >= d
        {
            return Err(ModelError::WouldBreakDependency {
                feature: dependent,
                input: id,
            });
        }
        let tl = &mut self.timeline;
        let was_forward = i < tl.marker;
        let mut node = tl.nodes.remove(i);
        let mut marker = tl.marker - usize::from(was_forward);
        if to < marker || (to == marker && was_forward) {
            marker += 1;
        }
        let old_group = node.group;
        node.group = tl.group_at(to, old_group);
        tl.nodes.insert(to, node);
        tl.marker = marker;
        tl.drop_group_if_empty(old_group);
        self.bump();
        Ok(())
    }

    /// Rolls the timeline to `index` (0 = nothing computed, `len` = everything).
    pub fn set_marker(&mut self, index: usize) -> Result<(), ModelError> {
        if index > self.timeline.len() {
            return Err(invalid(
                "marker",
                format!("{index} is past the end ({})", self.timeline.len()),
            ));
        }
        if index != self.timeline.marker {
            self.timeline.marker = index;
            self.bump();
        }
        Ok(())
    }

    /// Suppresses or unsuppresses a feature.
    pub fn set_suppressed(&mut self, id: FeatureId, suppressed: bool) -> Result<(), ModelError> {
        let i = self.timeline.index_of(id)?;
        if self.timeline.nodes[i].suppressed != suppressed {
            self.timeline.update(i, |n| n.suppressed = suppressed);
            self.bump();
        }
        Ok(())
    }

    /// Sets or clears a feature's display name.
    pub fn rename_feature(
        &mut self,
        id: FeatureId,
        name: Option<String>,
    ) -> Result<(), ModelError> {
        let i = self.timeline.index_of(id)?;
        if self.timeline.nodes[i].name != name {
            self.timeline.update(i, |n| n.name = name);
            self.bump();
        }
        Ok(())
    }

    /// Groups features that form one contiguous run of ungrouped nodes (any order in `ids`).
    pub fn group_features(&mut self, ids: &[FeatureId], name: &str) -> Result<GroupId, ModelError> {
        let mut idx = Vec::with_capacity(ids.len());
        for &id in ids {
            let i = self.timeline.index_of(id)?;
            if let Some(g) = self.timeline.nodes[i].group {
                return Err(invalid("group", format!("{id} is already in {g}")));
            }
            idx.push(i);
        }
        idx.sort_unstable();
        idx.dedup();
        let (Some(&first), Some(&last)) = (idx.first(), idx.last()) else {
            return Err(invalid("group", "no features given"));
        };
        if last - first + 1 != idx.len() {
            return Err(invalid(
                "group",
                "features must be adjacent in the timeline",
            ));
        }
        self.ensure_ids(1)?;
        let gid = GroupId(self.alloc()?);
        let tl = &mut self.timeline;
        tl.groups.insert(
            gid,
            Arc::new(Group {
                name: name.to_owned(),
                collapsed: false,
            }),
        );
        for i in first..=last {
            tl.update(i, |n| n.group = Some(gid));
        }
        self.bump();
        Ok(gid)
    }

    /// Dissolves a group; its features stay in place.
    pub fn ungroup(&mut self, group: GroupId) -> Result<(), ModelError> {
        if self.timeline.groups.remove(&group).is_none() {
            return Err(ModelError::NotFound {
                what: "group",
                id: group.0,
            });
        }
        for i in 0..self.timeline.len() {
            if self.timeline.nodes[i].group == Some(group) {
                self.timeline.update(i, |n| n.group = None);
            }
        }
        self.bump();
        Ok(())
    }

    /// Renames or collapses/expands a group.
    pub fn set_group(&mut self, id: GroupId, group: Group) -> Result<(), ModelError> {
        let Some(g) = self.timeline.groups.get(&id) else {
            return Err(ModelError::NotFound {
                what: "group",
                id: id.0,
            });
        };
        if **g != group {
            self.timeline.groups.insert(id, Arc::new(group));
            self.bump();
        }
        Ok(())
    }

    // ---- checks ---------------------------------------------------------------------------

    /// Ordering rule for a feature placed at index `at` (in the list without `self_id`).
    /// Producers must be nodes before `at`; occurrences must exist.
    fn check_inputs(
        &self,
        f: &FeatureBox,
        self_id: FeatureId,
        at: usize,
    ) -> Result<(), ModelError> {
        for dep in f.inputs() {
            for o in dep.occurrences() {
                if self.occurrence(o).is_none() {
                    return Err(ModelError::NotFound {
                        what: "occurrence",
                        id: o.0,
                    });
                }
            }
            let Some(p) = dep.producer() else { continue };
            let p = self.timeline.effective_producer(p);
            // Baked bodies of direct mode exist from the start: no producer node to order after.
            if self.timeline.node(p).is_none()
                && self
                    .baked
                    .range(crate::model::origin_range(p))
                    .next()
                    .is_some()
            {
                continue;
            }
            match self.timeline.node(p) {
                Some((j, _)) if j < at && p != self_id => {}
                Some(_) => {
                    return Err(ModelError::WouldBreakDependency {
                        feature: self_id,
                        input: p,
                    });
                }
                None if p == self_id => {
                    return Err(ModelError::WouldBreakDependency {
                        feature: self_id,
                        input: p,
                    });
                }
                None => {
                    return Err(ModelError::NotFound {
                        what: "feature",
                        id: p.0,
                    });
                }
            }
        }
        Ok(())
    }

    /// Timeline invariants on load (ids are claimed by `Design::validate`).
    pub(crate) fn validate_timeline(&self) -> Result<(), ModelError> {
        let bad = |m: String| Err(ModelError::Inconsistent(m));
        let tl = &self.timeline;
        if tl.marker > tl.len() {
            return bad(format!(
                "marker {} is past the end ({})",
                tl.marker,
                tl.len()
            ));
        }
        let index: BTreeMap<FeatureId, usize> = tl
            .nodes
            .iter()
            .enumerate()
            .map(|(i, n)| (n.id, i))
            .collect();
        if index.len() != tl.len() || tl.ids.len() != tl.len() {
            return bad("duplicate feature id in the timeline".to_owned());
        }
        let mut last_seen: BTreeMap<GroupId, usize> = BTreeMap::new();
        for (i, n) in tl.nodes.iter().enumerate() {
            let id = n.id;
            if self.component(n.context).is_none() {
                return bad(format!("{id} lives in missing {}", n.context));
            }
            if let Some(g) = n.group {
                if !tl.groups.contains_key(&g) {
                    return bad(format!("{id} is in missing {g}"));
                }
                if let Some(prev) = last_seen.insert(g, i)
                    && prev + 1 != i
                {
                    return bad(format!("{g} is not contiguous"));
                }
            }
            let f = &n.feature;
            if !f.is_unknown() {
                match registry().get(f.kind()) {
                    Some(r) if r.version == f.version() => {}
                    _ => {
                        return bad(format!(
                            "{id}: `{}` v{} is not registered",
                            f.kind(),
                            f.version()
                        ));
                    }
                }
                if let Err(e) = f.validate() {
                    return bad(format!("{id} (`{}`): {e}", f.kind()));
                }
            }
            for dep in f.inputs() {
                for o in dep.occurrences() {
                    if self.occurrence(o).is_none() {
                        return bad(format!("{id} reads missing {o}"));
                    }
                }
                let Some(p) = dep.producer() else { continue };
                let p = tl.effective_producer(p);
                match index.get(&p) {
                    Some(&j) if j < i => {}
                    Some(_) => return bad(format!("{id} reads {p}, which comes later")),
                    // Deleted producer: allowed (missing input at recompute) unless the id
                    // was never a feature.
                    None => match self.owner_of(p.0) {
                        Some(owner) if owner != "feature" => {
                            return bad(format!("{id} reads {p}, which is a {owner}"));
                        }
                        _ if p.0 == 0 || p.0 >= self.meta.next_id => {
                            return bad(format!("{id} reads unallocated {p}"));
                        }
                        _ => {}
                    },
                }
            }
        }
        if let Some(g) = tl.groups.keys().find(|g| !last_seen.contains_key(g)) {
            return bad(format!("{g} is empty"));
        }
        Ok(())
    }
}

// ---- serde ------------------------------------------------------------------------------

/// Wire form of a node: the feature's `kind`/`v`/`data` inline, so load errors read
/// `timeline.nodes[12].data.extent`.
#[derive(Serialize)]
struct NodeOut<'a> {
    id: FeatureId,
    name: &'a Option<String>,
    context: ComponentId,
    suppressed: bool,
    group: Option<GroupId>,
    kind: &'a str,
    v: u32,
    data: serde_json::Value,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct NodeIn {
    id: FeatureId,
    name: Option<String>,
    context: ComponentId,
    suppressed: bool,
    group: Option<GroupId>,
    kind: String,
    v: u32,
    data: serde_json::Value,
}

#[derive(Serialize)]
struct TimelineOut<'a> {
    nodes: Vec<NodeOut<'a>>,
    marker: usize,
    #[serde(serialize_with = "entries::serialize")]
    groups: &'a PMap<GroupId, Arc<Group>>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct TimelineIn {
    nodes: Vec<NodeIn>,
    marker: usize,
    #[serde(deserialize_with = "entries::deserialize")]
    groups: PMap<GroupId, Arc<Group>>,
}

impl Serialize for Timeline {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        use serde::ser::Error as _;
        let nodes = self
            .nodes
            .iter()
            .map(|n| {
                Ok(NodeOut {
                    id: n.id,
                    name: &n.name,
                    context: n.context,
                    suppressed: n.suppressed,
                    group: n.group,
                    kind: n.feature.kind(),
                    v: n.feature.version(),
                    data: n.feature.to_json().map_err(S::Error::custom)?,
                })
            })
            .collect::<Result<Vec<_>, S::Error>>()?;
        TimelineOut {
            nodes,
            marker: self.marker,
            groups: &self.groups,
        }
        .serialize(s)
    }
}

impl<'de> Deserialize<'de> for Timeline {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Timeline, D::Error> {
        use serde::de::Error as _;
        let w = TimelineIn::deserialize(d)?;
        let mut nodes = PVec::new();
        let mut ids = PSet::new();
        for (i, n) in w.nodes.into_iter().enumerate() {
            let feature = FeatureBox::from_parts(&n.kind, n.v, n.data)
                .map_err(|e| D::Error::custom(format_args!("timeline node {i}: {e}")))?;
            ids.insert(n.id);
            nodes.push_back(FeatureNode {
                id: n.id,
                feature,
                suppressed: n.suppressed,
                group: n.group,
                name: n.name,
                context: n.context,
            });
        }
        Ok(Timeline {
            nodes,
            marker: w.marker,
            groups: w.groups,
            ids,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::id::BodyId;
    use crate::model::BodyProps;
    use crate::testing::{TestAddMesh, TestBox, TestCounter};
    use crate::{FeatureDataError, OccurrenceId};
    use serde_json::{Value, json};

    fn design() -> Design {
        Design::with_uuid("T", "0190f5a3-0000-7000-8000-000000000002".parse().unwrap())
    }

    fn boxf() -> FeatureBox {
        FeatureBox::new(TestBox {
            size: [1.0, 2.0, 3.0],
        })
    }

    fn counter(deps: &[Dependency], value: i64) -> FeatureBox {
        FeatureBox::new(TestCounter {
            deps: deps.to_vec(),
            value,
            fail: false,
        })
    }

    fn body_of(f: FeatureId, slot: u32) -> Dependency {
        Dependency::Body(BodyId { origin: f, slot })
    }

    fn add(d: &mut Design, f: FeatureBox) -> FeatureId {
        let root = d.root();
        d.add_feature(None, f, root, None).unwrap()
    }

    /// Four independent counters, marker at the end.
    fn four() -> (Design, Vec<FeatureId>) {
        let mut d = design();
        let ids = (0..4).map(|i| add(&mut d, counter(&[], i))).collect();
        (d, ids)
    }

    fn order(d: &Design) -> Vec<FeatureId> {
        d.timeline().nodes().map(|n| n.id).collect()
    }

    /// Validates, saves, reloads; asserts the reload is byte-identical.
    #[track_caller]
    fn rt(d: &Design) -> Design {
        d.validate().unwrap();
        let s = serde_json::to_string(d).unwrap();
        let back: Design =
            serde_json::from_str(&s).unwrap_or_else(|e| panic!("won't load: {e}\n{s}"));
        assert_eq!(serde_json::to_string(&back).unwrap(), s);
        assert_eq!(order(&back), order(d));
        assert_eq!(back.timeline().marker(), d.timeline().marker());
        back
    }

    /// Runs `edit` and asserts it failed and changed nothing (JSON and revision).
    #[track_caller]
    fn fails_unchanged<T: std::fmt::Debug>(
        d: &mut Design,
        edit: impl FnOnce(&mut Design) -> Result<T, ModelError>,
    ) -> ModelError {
        let (before, rev) = (serde_json::to_string(&*d).unwrap(), d.revision());
        let e = edit(d).unwrap_err();
        assert_eq!(serde_json::to_string(&*d).unwrap(), before, "{e}");
        assert_eq!(d.revision(), rev);
        e
    }

    #[test]
    fn timeline_of_test_features_round_trips() {
        let mut d = design();
        let a = add(&mut d, boxf());
        let b = add(
            &mut d,
            counter(&[body_of(a, 0), Dependency::Feature(a)], -7),
        );
        let c = add(
            &mut d,
            FeatureBox::new(TestAddMesh {
                bodies: 3,
                label: "ünï \"quoted\"".into(),
            }),
        );
        let e = add(&mut d, counter(&[body_of(c, 2)], 1));
        d.rename_feature(b, Some("Second".into())).unwrap();
        d.set_suppressed(c, true).unwrap();
        let g = d.group_features(&[c, e], "Tail").unwrap();
        d.set_group(
            g,
            Group {
                name: "Tail".into(),
                collapsed: true,
            },
        )
        .unwrap();
        d.set_marker(3).unwrap();
        let back = rt(&d);
        let tl = back.timeline();
        assert_eq!(tl.len(), 4);
        assert_eq!(tl.marker(), 3);
        assert_eq!(tl.node(b).unwrap().1.name.as_deref(), Some("Second"));
        assert!(tl.node(c).unwrap().1.suppressed);
        assert!(tl.group(g).unwrap().collapsed);
        assert_eq!(
            tl.node(a).unwrap().1.feature.downcast_ref::<TestBox>(),
            Some(&TestBox {
                size: [1.0, 2.0, 3.0]
            })
        );
        // Equal as JSON values too, not just as text.
        assert_eq!(
            serde_json::to_value(&d).unwrap()["timeline"],
            serde_json::to_value(&back).unwrap()["timeline"]
        );
        // Node wire shape.
        let v = serde_json::to_value(&d).unwrap();
        let n0 = &v["timeline"]["nodes"][0];
        assert_eq!(n0["kind"], "test.box");
        assert_eq!(n0["v"], 2);
        assert_eq!(n0["data"], json!({"size": [1.0, 2.0, 3.0]}));
    }

    /// `d` plus an extra node appended to its saved JSON.
    fn load_with_node(
        d: &Design,
        kind: &str,
        v: u32,
        data: Value,
    ) -> Result<(Design, FeatureId), String> {
        let mut j = serde_json::to_value(d).unwrap();
        let id = j["meta"]["next_id"].as_u64().unwrap();
        j["meta"]["next_id"] = (id + 1).into();
        let root = j["root"].clone();
        let n = j["timeline"]["nodes"].as_array_mut().unwrap();
        n.push(
            json!({"id": id, "name": null, "context": root, "suppressed": false,
            "group": null, "kind": kind, "v": v, "data": data}),
        );
        let m = n.len();
        j["timeline"]["marker"] = m.into();
        serde_json::from_value::<Design>(j)
            .map(|d| (d, FeatureId(id)))
            .map_err(|e| e.to_string())
    }

    #[test]
    fn unknown_kind_loads_as_unknown_and_resaves_byte_identical() {
        let text = r#"{"kind":"future.loft","v":4,"data":{"a":[0.1,1e-7,123456789.1234568,-0.0,5e-324,1.7976931348623157e+308],"b":{"x":null,"y":"s"},"z":true}}"#;
        let fb: FeatureBox = serde_json::from_str(text).unwrap();
        let u = fb.as_unknown().expect("unknown");
        assert_eq!((u.kind.as_str(), u.version), ("future.loft", 4));
        assert_eq!(serde_json::to_string(&fb).unwrap(), text);

        // Inside a design, via save -> load -> save.
        let mut d = design();
        add(&mut d, boxf());
        let data: Value = serde_json::from_str(r#"{"k":[1,2.5,{"q":0.3}],"w":"x"}"#).unwrap();
        let (d2, id) = load_with_node(&d, "future.loft", 4, data.clone()).unwrap();
        let back = rt(&d2);
        let (_, n) = back.timeline().node(id).unwrap();
        assert!(n.feature.is_unknown());
        assert_eq!(n.feature.to_json().unwrap(), data);
        assert_eq!(
            serde_json::to_value(&back).unwrap()["timeline"]["nodes"][1]["data"],
            data
        );
        // Unknown features can be suppressed, renamed and removed, nothing else.
        let mut d3 = back;
        d3.set_suppressed(id, true).unwrap();
        d3.rename_feature(id, Some("?".into())).unwrap();
        rt(&d3);
        let e = fails_unchanged(&mut d3, |d| d.replace_feature(id, boxf()));
        assert!(e.to_string().contains("unknown"), "{e}");
        fails_unchanged(&mut d3, |d| d.move_feature(id, 0));
        let root = d3.root();
        fails_unchanged(&mut d3, |d| d.add_feature(None, fb.clone(), root, None));
        d3.remove_feature(id).unwrap();
        rt(&d3);
    }

    #[test]
    fn float_data_of_unknown_features_survives_exactly() {
        // xorshift over random bit patterns.
        let mut s = 0x9E37_79B9_7F4A_7C15_u64;
        let mut vals = Vec::new();
        while vals.len() < 20_000 {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            let f = f64::from_bits(s);
            if f.is_finite() {
                vals.push(f);
            }
        }
        let text = format!(
            r#"{{"kind":"future.x","v":9,"data":{}}}"#,
            serde_json::to_string(&vals).unwrap()
        );
        let fb: FeatureBox = serde_json::from_str(&text).unwrap();
        assert_eq!(serde_json::to_string(&fb).unwrap(), text);
    }

    #[test]
    fn newer_version_of_a_known_kind_is_preserved_as_unknown() {
        let text = r#"{"kind":"test.box","v":3,"data":{"size":[1.0,2.0,3.0],"zeta":"new"}}"#;
        let fb: FeatureBox = serde_json::from_str(text).unwrap();
        assert!(fb.is_unknown());
        assert_eq!(serde_json::to_string(&fb).unwrap(), text);
    }

    #[test]
    fn older_versions_migrate_and_bad_data_is_an_error() {
        let fb = FeatureBox::from_parts("test.box", 1, json!({"edge": 2.0})).unwrap();
        assert_eq!(
            fb.downcast_ref::<TestBox>(),
            Some(&TestBox {
                size: [2.0, 2.0, 2.0]
            })
        );
        assert_eq!(fb.version(), 2, "saved at the current version");
        assert!(matches!(
            FeatureBox::from_parts("test.box", 1, json!({"wrong": 1})),
            Err(FeatureDataError::Migration { .. })
        ));
        assert!(matches!(
            FeatureBox::from_parts("test.box", 2, json!({"size": "x"})),
            Err(FeatureDataError::Invalid { .. })
        ));
        // A kind without a migration can't load an older version.
        assert!(matches!(
            FeatureBox::from_parts("test.counter", 0, json!({})),
            Err(FeatureDataError::Migration { .. })
        ));
        // In a design: loads, then saves as v2.
        let (d, id) = load_with_node(&design(), "test.box", 1, json!({"edge": 4.0})).unwrap();
        let v = serde_json::to_value(&d).unwrap();
        assert_eq!(v["timeline"]["nodes"][0]["v"], 2);
        assert_eq!(
            v["timeline"]["nodes"][0]["data"],
            json!({"size": [4.0, 4.0, 4.0]})
        );
        assert!(!d.timeline().node(id).unwrap().1.feature.is_unknown());
        rt(&d);
    }

    #[test]
    fn move_before_a_dependency_is_rejected() {
        let mut d = design();
        let a = add(&mut d, boxf());
        let b = add(&mut d, counter(&[body_of(a, 0)], 1));
        let c = add(&mut d, counter(&[], 2));
        assert_eq!(d.timeline().valid_move_range(b), Some(1..=2));
        assert_eq!(d.timeline().valid_move_range(a), Some(0..=0));
        assert_eq!(d.timeline().valid_move_range(c), Some(0..=2));
        assert_eq!(d.timeline().dependents(a), vec![b]);
        // b before its producer a.
        let e = fails_unchanged(&mut d, |d| d.move_feature(b, 0));
        assert_eq!(
            e,
            ModelError::WouldBreakDependency {
                feature: b,
                input: a
            }
        );
        // a after its dependent b.
        let e = fails_unchanged(&mut d, |d| d.move_feature(a, 1));
        assert_eq!(
            e,
            ModelError::WouldBreakDependency {
                feature: b,
                input: a
            }
        );
        fails_unchanged(&mut d, |d| d.move_feature(a, 2));
        fails_unchanged(&mut d, |d| d.move_feature(a, 3)); // past the end
        // Legal moves.
        d.move_feature(c, 0).unwrap();
        assert_eq!(order(&d), [c, a, b]);
        d.move_feature(c, 2).unwrap();
        assert_eq!(order(&d), [a, b, c]);
        let rev = d.revision();
        d.move_feature(c, 2).unwrap();
        assert_eq!(d.revision(), rev, "no-op doesn't bump");
        rt(&d);
    }

    #[test]
    fn ordering_rule_applies_to_add_and_replace() {
        let mut d = design();
        let a = add(&mut d, boxf());
        let b = add(&mut d, counter(&[], 1));
        let root = d.root();
        let missing = FeatureId(d.meta().next_id + 10);
        // Reads a later or missing producer, a missing occurrence, or sits past the end.
        fails_unchanged(&mut d, |d| {
            d.add_feature(Some(0), counter(&[body_of(a, 0)], 0), root, None)
        });
        fails_unchanged(&mut d, |d| {
            d.add_feature(None, counter(&[body_of(missing, 0)], 0), root, None)
        });
        fails_unchanged(&mut d, |d| d.add_feature(Some(9), boxf(), root, None));
        fails_unchanged(&mut d, |d| {
            d.add_feature(
                None,
                counter(&[Dependency::Occurrence(OccurrenceId(5000))], 0),
                root,
                None,
            )
        });
        // Self-reference through replace: b reads b.
        fails_unchanged(&mut d, |d| {
            d.replace_feature(b, counter(&[body_of(b, 0)], 1))
        });
        // Valid replace, no-op replace, kind change.
        d.replace_feature(b, counter(&[body_of(a, 0)], 2)).unwrap();
        let rev = d.revision();
        d.replace_feature(b, counter(&[body_of(a, 0)], 2)).unwrap();
        assert_eq!(d.revision(), rev);
        fails_unchanged(&mut d, |d| d.replace_feature(b, boxf()));
        rt(&d);
    }

    #[test]
    fn non_finite_and_invalid_values_are_rejected() {
        let mut d = design();
        let id = add(&mut d, boxf());
        let root = d.root();
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 0.0, -1.0] {
            let f = FeatureBox::new(TestBox {
                size: [1.0, bad, 1.0],
            });
            fails_unchanged(&mut d, |d| d.add_feature(None, f, root, None));
        }
        fails_unchanged(&mut d, |d| {
            d.replace_feature(
                id,
                FeatureBox::new(TestBox {
                    size: [f64::NAN; 3],
                }),
            )
        });
        rt(&d);
    }

    #[test]
    fn marker_follows_inserts_and_moves() {
        let (mut d, n) = four();
        assert_eq!(
            d.timeline().marker(),
            4,
            "appending at the marker keeps it at the end"
        );
        d.set_marker(2).unwrap();
        let root = d.root();
        // Insert at the marker: the new feature is computed (marker advances).
        let x = d.add_feature(None, counter(&[], 9), root, None).unwrap();
        assert_eq!(d.timeline().node(x).unwrap().0, 2);
        assert_eq!(d.timeline().marker(), 3);
        d.remove_feature(x).unwrap();
        assert_eq!(d.timeline().marker(), 2);
        // Insert after the marker: marker stays.
        let y = d.add_feature(Some(3), counter(&[], 9), root, None).unwrap();
        assert_eq!(d.timeline().marker(), 2);
        d.remove_feature(y).unwrap();
        // Moves keep the marker between the same neighbours.
        d.move_feature(n[3], 0).unwrap(); // from after the marker to before it
        assert_eq!(order(&d), [n[3], n[0], n[1], n[2]]);
        assert_eq!(d.timeline().marker(), 3);
        d.move_feature(n[0], 3).unwrap(); // from before the marker to the end
        assert_eq!(order(&d), [n[3], n[1], n[2], n[0]]);
        assert_eq!(d.timeline().marker(), 2);
        // Bounds.
        d.set_marker(0).unwrap();
        d.set_marker(4).unwrap();
        fails_unchanged(&mut d, |d| d.set_marker(5));
        rt(&d);
    }

    #[test]
    fn suppress_and_rename_are_idempotent_edits() {
        let (mut d, n) = four();
        d.set_suppressed(n[1], true).unwrap();
        let rev = d.revision();
        d.set_suppressed(n[1], true).unwrap();
        d.rename_feature(n[1], None).unwrap();
        assert_eq!(d.revision(), rev);
        fails_unchanged(&mut d, |d| d.set_suppressed(FeatureId(99_999), true));
        fails_unchanged(&mut d, |d| d.rename_feature(FeatureId(99_999), None));
        d.set_suppressed(n[1], false).unwrap();
        assert!(!d.timeline().node(n[1]).unwrap().1.suppressed);
    }

    #[test]
    fn groups_are_contiguous_and_follow_edits() {
        let (mut d, n) = four();
        fails_unchanged(&mut d, |d| d.group_features(&[n[0], n[2]], "gap"));
        fails_unchanged(&mut d, |d| d.group_features(&[], "none"));
        fails_unchanged(&mut d, |d| d.group_features(&[FeatureId(99_999)], "nf"));
        let g = d.group_features(&[n[2], n[1]], "mid").unwrap();
        assert_eq!(d.timeline().group_range(g), Some(1..=2));
        fails_unchanged(&mut d, |d| d.group_features(&[n[1], n[0]], "again"));
        // Insert inside the group joins it; at its edge doesn't.
        let root = d.root();
        let inside = d.add_feature(Some(2), counter(&[], 5), root, None).unwrap();
        assert_eq!(d.timeline().node(inside).unwrap().1.group, Some(g));
        let edge = d.add_feature(Some(1), counter(&[], 5), root, None).unwrap();
        assert_eq!(d.timeline().node(edge).unwrap().1.group, None);
        assert_eq!(d.timeline().group_range(g), Some(2..=4));
        rt(&d);
        // Order is now n0, edge, n1, inside, n2, n3. Move n3 into the group, then out.
        d.move_feature(n[3], 3).unwrap();
        assert_eq!(d.timeline().node(n[3]).unwrap().1.group, Some(g));
        // Moved to the group's edge it stays in; moved elsewhere it leaves.
        d.move_feature(n[3], 5).unwrap();
        assert_eq!(d.timeline().node(n[3]).unwrap().1.group, Some(g));
        d.move_feature(n[3], 0).unwrap();
        assert_eq!(d.timeline().node(n[3]).unwrap().1.group, None);
        // Removing every member drops the group.
        for id in [inside, n[1], n[2]] {
            d.remove_feature(id).unwrap();
        }
        assert!(d.timeline().group(g).is_none());
        rt(&d);
        let g2 = d.group_features(&[n[3], n[0]], "ends").unwrap();
        d.ungroup(g2).unwrap();
        assert!(d.timeline().nodes().all(|n| n.group.is_none()));
        fails_unchanged(&mut d, |d| d.ungroup(g2));
        fails_unchanged(&mut d, |d| {
            d.set_group(
                g2,
                Group {
                    name: "x".into(),
                    collapsed: false,
                },
            )
        });
        rt(&d);
    }

    #[test]
    fn removing_a_feature_drops_its_body_data_and_leaves_dangling_readers() {
        let mut d = design();
        let a = add(&mut d, boxf());
        let b = add(&mut d, counter(&[body_of(a, 0)], 1));
        let props = BodyProps {
            visible: false,
            ..BodyProps::default()
        };
        d.set_body_props(BodyId { origin: a, slot: 0 }, props.clone())
            .unwrap();
        d.set_body_props(BodyId { origin: b, slot: 0 }, props.clone())
            .unwrap();
        // Props need a timeline origin.
        let stray = BodyId {
            origin: FeatureId(d.meta().next_id + 3),
            slot: 0,
        };
        assert!(d.set_body_props(stray, props).is_err());
        d.remove_feature(a).unwrap();
        let left: Vec<_> = d.explicit_body_props().map(|(k, _)| k.origin).collect();
        assert_eq!(left, [b]);
        assert_eq!(d.timeline().dependents(a), vec![b]);
        let back = rt(&d); // a dangling producer is allowed
        assert_eq!(back.timeline().len(), 1);
        let root = d.root();
        fails_unchanged(&mut d, |d| {
            d.add_feature(None, counter(&[body_of(a, 0)], 0), root, None)
        });
        fails_unchanged(&mut d, |d| d.remove_feature(a));
    }

    #[test]
    fn load_rejects_inconsistent_timelines() {
        let mut d = design();
        let a = add(&mut d, boxf());
        add(&mut d, counter(&[body_of(a, 0)], 1));
        add(&mut d, counter(&[], 2));
        d.group_features(&[a], "g").unwrap();
        let good = serde_json::to_value(&d).unwrap();
        let dep = |x: Dependency| serde_json::to_value(vec![x]).unwrap();
        let next = good["meta"]["next_id"].as_u64().unwrap();
        let root = good["root"].as_u64().unwrap();
        type Edit = Box<dyn Fn(&mut Value)>;
        let set_deps = |x: Dependency| -> Edit {
            let x = dep(x);
            Box::new(move |v| v["timeline"]["nodes"][2]["data"]["deps"] = x.clone())
        };
        let cases: Vec<(&str, Edit)> = vec![
            (
                "comes later",
                Box::new(|v| v["timeline"]["nodes"].as_array_mut().unwrap().swap(0, 1)),
            ),
            ("marker", Box::new(|v| v["timeline"]["marker"] = 99.into())),
            (
                "unknown field",
                Box::new(|v| v["timeline"]["nodes"][0]["extra"] = 1.into()),
            ),
            (
                "unknown field",
                Box::new(|v| v["timeline"]["extra"] = 1.into()),
            ),
            (
                "timeline node 1",
                Box::new(|v| v["timeline"]["nodes"][1]["data"]["value"] = "x".into()),
            ),
            (
                "must be finite and positive",
                Box::new(|v| v["timeline"]["nodes"][0]["data"]["size"][1] = 0.0.into()),
            ),
            (
                "missing",
                Box::new(|v| v["timeline"]["nodes"][0]["context"] = 99_999.into()),
            ),
            (
                "missing",
                Box::new(|v| v["timeline"]["nodes"][2]["group"] = 99_999.into()),
            ),
            (
                "not contiguous",
                Box::new(|v| {
                    let g = v["timeline"]["nodes"][0]["group"].clone();
                    v["timeline"]["nodes"][2]["group"] = g;
                }),
            ),
            (
                "is empty",
                Box::new(|v| v["timeline"]["nodes"][0]["group"] = Value::Null),
            ),
            (
                "used twice",
                Box::new(|v| {
                    let id = v["timeline"]["nodes"][0]["id"].clone();
                    v["timeline"]["nodes"][2]["id"] = id;
                }),
            ),
            (
                "used twice",
                Box::new(move |v| v["timeline"]["nodes"][2]["id"] = root.into()),
            ),
            (
                "reads missing",
                set_deps(Dependency::Occurrence(OccurrenceId(7777))),
            ),
            (
                "unallocated",
                set_deps(Dependency::Feature(FeatureId(next + 5))),
            ),
            ("which is a", set_deps(Dependency::Feature(FeatureId(root)))),
            (
                "no migration",
                Box::new(|v| v["timeline"]["nodes"][2]["v"] = 0.into()),
            ),
        ];
        for (needle, edit) in cases {
            let mut v = good.clone();
            edit(&mut v);
            let e = serde_json::from_value::<Design>(v)
                .map(|_| "loaded".to_owned())
                .unwrap_or_else(|e| e.to_string());
            assert!(e.contains(needle), "expected `{needle}` in `{e}`");
        }
        serde_json::from_value::<Design>(good).unwrap();
    }
}
