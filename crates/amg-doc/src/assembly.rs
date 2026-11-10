//! Components, occurrences and placements (S06, ASM-01).
//!
//! - Low-level `Design` edits behind the assembly commands (`AddOccurrence`,
//!   `DeleteOccurrences` with its cascade, `MoveOccurrence`, grounding, naming, batch
//!   placement), plus the path queries other lanes use.
//! - [`Placements`]: the position of every occurrence in evaluated state. Until the joint
//!   solver (ASM-04) they equal the authored transforms; later they are the solved positions.
//!   World transforms of occurrence paths and body instances are answered from them.

use std::collections::BTreeSet;
use std::sync::Arc;

use amg_core::Transform;

use crate::direct::BaseFeature;
use crate::feature::FeatureBox;
use crate::id::{BodyId, ComponentId, FeatureId, OccurrenceId};
use crate::model::{Design, ModelError, check_transform};

/// A path of occurrences from a component downward: `[o1, .., on]` where `o1` is placed in the
/// start component, `o2` in the component `o1` places, and so on. Empty means the start
/// component itself. World queries take paths from the root.
pub type OccPath = Vec<OccurrenceId>;

/// What a cascading delete removed (besides the occurrences named by the caller).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Removed {
    /// Occurrences removed, ascending: the requested ones and those inside deleted components.
    pub occurrences: Vec<OccurrenceId>,
    /// Components deleted because their last occurrence went, ascending.
    pub components: Vec<ComponentId>,
    /// Timeline features deleted because their context component went, in timeline order.
    pub features: Vec<FeatureId>,
    /// Baked bodies deleted with their component, ascending.
    pub baked: Vec<BodyId>,
}

fn invalid(field: &'static str, msg: impl Into<String>) -> ModelError {
    ModelError::InvalidValue {
        field,
        msg: msg.into(),
    }
}

impl Design {
    /// True if `path` walks parent → child starting in `from`: the first occurrence is placed
    /// in `from`, each next one in the component the previous one places. Empty paths are
    /// valid.
    pub fn is_valid_path(&self, from: ComponentId, path: &[OccurrenceId]) -> bool {
        let mut at = from;
        for o in path {
            match self.occurrence(*o) {
                Some(occ) if occ.parent == at => at = occ.component,
                _ => return false,
            }
        }
        true
    }

    /// Features whose context is `c`, in timeline order.
    pub fn features_in(&self, c: ComponentId) -> impl Iterator<Item = FeatureId> + '_ {
        self.timeline()
            .nodes()
            .filter(move |n| n.context == c)
            .map(|n| n.id)
    }

    /// The authored transform of the occurrence path `path` from the root (product of the
    /// locals), or `None` if the path is not valid from the root.
    pub fn authored_world(&self, path: &[OccurrenceId]) -> Option<Transform> {
        if !self.is_valid_path(self.root(), path) {
            return None;
        }
        Some(
            path.iter()
                .filter_map(|o| self.occurrence(*o))
                .fold(Transform::IDENTITY, |acc, o| acc * o.transform),
        )
    }

    /// Sets or clears the name override of an occurrence. Empty names are refused.
    pub fn set_occurrence_name(
        &mut self,
        occ: OccurrenceId,
        name: Option<String>,
    ) -> Result<(), ModelError> {
        if name.as_deref().is_some_and(|n| n.trim().is_empty()) {
            return Err(invalid("name", "an occurrence name can't be empty"));
        }
        let o = self.occurrences.get_mut(&occ).ok_or(ModelError::NotFound {
            what: "occurrence",
            id: occ.0,
        })?;
        if o.name != name {
            Arc::make_mut(o).name = name;
            self.bump();
        }
        Ok(())
    }

    /// Grounds or unground an occurrence.
    pub fn set_occurrence_grounded(
        &mut self,
        occ: OccurrenceId,
        grounded: bool,
    ) -> Result<(), ModelError> {
        let o = self.occurrences.get_mut(&occ).ok_or(ModelError::NotFound {
            what: "occurrence",
            id: occ.0,
        })?;
        if o.grounded != grounded {
            Arc::make_mut(o).grounded = grounded;
            self.bump();
        }
        Ok(())
    }

    /// Sets several occurrence transforms at once, all or none. An occurrence may be listed
    /// only once.
    pub fn set_occurrence_transforms(
        &mut self,
        moves: &[(OccurrenceId, Transform)],
    ) -> Result<(), ModelError> {
        let mut seen = BTreeSet::new();
        for (o, t) in moves {
            if self.occurrence(*o).is_none() {
                return Err(ModelError::NotFound {
                    what: "occurrence",
                    id: o.0,
                });
            }
            if !seen.insert(*o) {
                return Err(invalid("moves", format!("{o} is listed twice")));
            }
            check_transform(t)?;
        }
        for (o, t) in moves {
            self.set_occurrence_transform(*o, *t)?;
        }
        Ok(())
    }

    /// Moves `occ` into `parent` (appended to its children). With `keep_world` the local
    /// transform is recomputed so the world transform stays the same; that is only defined
    /// when the old and the new parent are each placed exactly once (the root counts as once).
    pub fn move_occurrence(
        &mut self,
        occ: OccurrenceId,
        parent: ComponentId,
        keep_world: bool,
    ) -> Result<(), ModelError> {
        let o = self.occurrence(occ).ok_or(ModelError::NotFound {
            what: "occurrence",
            id: occ.0,
        })?;
        let (old_parent, component, local) = (o.parent, o.component, o.transform);
        self.require_component(parent)?;
        if parent == old_parent {
            return Ok(());
        }
        if self.contains(component, parent) {
            return Err(ModelError::WouldCycle { component, parent });
        }
        let transform = if keep_world {
            let world_of = |c: ComponentId| -> Result<Transform, ModelError> {
                let paths = self.occurrence_paths(c);
                match paths.as_slice() {
                    [p] => Ok(self.authored_world(p).unwrap_or(Transform::IDENTITY)),
                    _ => Err(invalid(
                        "keep_world",
                        format!(
                            "{c} is placed {} times, so the world transform is ambiguous",
                            paths.len()
                        ),
                    )),
                }
            };
            let t = world_of(parent)?.inverse() * world_of(old_parent)? * local;
            check_transform(&t)?;
            t
        } else {
            local
        };
        if let Some(p) = self.components.get_mut(&old_parent) {
            Arc::make_mut(p).children.retain(|c| *c != occ);
        }
        if let Some(p) = self.components.get_mut(&parent) {
            Arc::make_mut(p).children.push(occ);
        }
        if let Some(o) = self.occurrences.get_mut(&occ) {
            let o = Arc::make_mut(o);
            o.parent = parent;
            o.transform = transform;
        }
        self.bump();
        Ok(())
    }

    /// Removes occurrences with the S06 §1 cascade: a component left without occurrences is
    /// deleted with the occurrences placed inside it (which may orphan further components),
    /// every timeline feature whose context was deleted (model parameters they owned become
    /// user parameters), and baked bodies in deleted components. Atomic.
    pub fn remove_occurrences(&mut self, occs: &[OccurrenceId]) -> Result<Removed, ModelError> {
        let mut want: Vec<OccurrenceId> = occs.to_vec();
        want.sort_unstable();
        want.dedup();
        if want.is_empty() {
            return Err(invalid("occs", "no occurrences given"));
        }
        for o in &want {
            if self.occurrence(*o).is_none() {
                return Err(ModelError::NotFound {
                    what: "occurrence",
                    id: o.0,
                });
            }
        }
        let mut w = self.clone();
        let mut gone: BTreeSet<OccurrenceId> = BTreeSet::new();
        for o in &want {
            if let Some(occ) = w.occurrences.remove(o) {
                if let Some(p) = w.components.get_mut(&occ.parent) {
                    Arc::make_mut(p).children.retain(|c| c != o);
                }
                gone.insert(*o);
            }
        }
        let mut doomed: BTreeSet<ComponentId> = BTreeSet::new();
        loop {
            let placed: BTreeSet<ComponentId> =
                w.occurrences.values().map(|o| o.component).collect();
            let orphans: Vec<ComponentId> = w
                .components
                .keys()
                .copied()
                .filter(|c| *c != w.root() && !placed.contains(c))
                .collect();
            if orphans.is_empty() {
                break;
            }
            for c in orphans {
                if let Some(comp) = w.components.remove(&c) {
                    for child in &comp.children {
                        if w.occurrences.remove(child).is_some() {
                            gone.insert(*child);
                        }
                    }
                }
                doomed.insert(c);
            }
        }

        let features: Vec<FeatureId> = w
            .timeline()
            .nodes()
            .filter(|n| doomed.contains(&n.context))
            .map(|n| n.id)
            .collect();
        for f in &features {
            w.remove_feature(*f)?;
        }
        w.convert_model_params(&features);

        let baked: Vec<BodyId> = w
            .baked
            .iter()
            .filter(|(_, b)| doomed.contains(&b.component))
            .map(|(id, _)| *id)
            .collect();
        for b in &baked {
            w.baked.remove(b);
            w.body_props.remove(b);
        }
        if !baked.is_empty() {
            let bases: Vec<(FeatureId, BaseFeature)> = w
                .timeline()
                .nodes()
                .filter_map(|n| {
                    n.feature
                        .downcast_ref::<BaseFeature>()
                        .filter(|bf| bf.bodies.iter().any(|b| baked.contains(&b.id)))
                        .map(|bf| (n.id, bf.clone()))
                })
                .collect();
            for (id, mut bf) in bases {
                bf.bodies.retain(|b| !baked.contains(&b.id));
                w.replace_feature(id, FeatureBox::new(bf))?;
            }
        }
        w.bump();
        let out = Removed {
            occurrences: gone.into_iter().collect(),
            components: doomed.into_iter().collect(),
            features,
            baked,
        };
        *self = w;
        Ok(out)
    }
}

// ---- placements ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq)]
struct Slot {
    parent: ComponentId,
    component: ComponentId,
    local: Transform,
}

/// Placements of every occurrence after the assembly stage (= the authored transforms until
/// ASM-04 adds the joint solver). Part of [`Evaluated`](crate::Evaluated).
///
/// Lookups are binary searches over id-sorted vectors, so building is linear and a
/// `world` query is `O(depth · log n)` without allocation.
#[derive(Clone, Debug)]
pub struct Placements {
    root: ComponentId,
    /// Sorted by occurrence id.
    slots: Vec<(OccurrenceId, Slot)>,
    /// `(component, occurrence)` sorted: the occurrences of each component.
    by_component: Vec<(ComponentId, OccurrenceId)>,
}

impl Default for Placements {
    fn default() -> Placements {
        Placements {
            root: ComponentId(0),
            slots: Vec::new(),
            by_component: Vec::new(),
        }
    }
}

impl Placements {
    /// Placements equal to the authored transforms of `design`.
    pub fn from_design(design: &Design) -> Placements {
        let slots: Vec<(OccurrenceId, Slot)> = design
            .occurrences()
            .map(|(id, o)| {
                (
                    id,
                    Slot {
                        parent: o.parent,
                        component: o.component,
                        local: o.transform,
                    },
                )
            })
            .collect();
        let mut by_component: Vec<(ComponentId, OccurrenceId)> =
            slots.iter().map(|(id, s)| (s.component, *id)).collect();
        by_component.sort_unstable();
        Placements {
            root: design.root(),
            slots,
            by_component,
        }
    }

    fn slot(&self, o: OccurrenceId) -> Option<&Slot> {
        self.slots
            .binary_search_by_key(&o, |(id, _)| *id)
            .ok()
            .map(|i| &self.slots[i].1)
    }

    /// Placement of `o` relative to its parent component.
    pub fn local(&self, o: OccurrenceId) -> Option<Transform> {
        self.slot(o).map(|s| s.local)
    }

    /// Product of the locals along `path` (from the root). `None` if the path is not valid
    /// from the root. The empty path is the identity.
    pub fn world(&self, path: &[OccurrenceId]) -> Option<Transform> {
        let mut at = self.root;
        let mut acc = Transform::IDENTITY;
        for o in path {
            let s = self.slot(*o)?;
            if s.parent != at {
                return None;
            }
            acc *= s.local;
            at = s.component;
        }
        Some(acc)
    }

    /// Number of occurrences.
    pub fn len(&self) -> usize {
        self.slots.len()
    }

    /// True if there are no occurrences.
    pub fn is_empty(&self) -> bool {
        self.slots.is_empty()
    }

    /// The root component these placements were built for.
    pub fn root(&self) -> ComponentId {
        self.root
    }

    /// The component an occurrence path ends in (`from` for the empty path), or `None` if the
    /// path is not valid from the root.
    fn leaf_component(&self, path: &[OccurrenceId]) -> Option<ComponentId> {
        let mut at = self.root;
        for o in path {
            let s = self.slot(*o)?;
            if s.parent != at {
                return None;
            }
            at = s.component;
        }
        Some(at)
    }

    /// Every path from the root to a placement of `c`, in the order of
    /// [`Design::occurrence_paths`] (occurrence id order, diamonds give several).
    pub fn paths_of(&self, c: ComponentId) -> Vec<OccPath> {
        if c == self.root {
            return vec![Vec::new()];
        }
        let start = self.by_component.partition_point(|(k, _)| *k < c);
        let mut out = Vec::new();
        for (k, o) in &self.by_component[start..] {
            if *k != c {
                break;
            }
            let Some(s) = self.slot(*o) else { continue };
            for mut p in self.paths_of(s.parent) {
                p.push(*o);
                out.push(p);
            }
        }
        out
    }

    /// Occurrences whose placement differs between `old` and `self` (present in both).
    pub(crate) fn moved_since(&self, old: &Placements) -> Vec<OccurrenceId> {
        let mut out = Vec::new();
        let (mut i, mut j) = (0, 0);
        while i < self.slots.len() && j < old.slots.len() {
            let (a, b) = (&self.slots[i], &old.slots[j]);
            match a.0.cmp(&b.0) {
                std::cmp::Ordering::Less => i += 1,
                std::cmp::Ordering::Greater => j += 1,
                std::cmp::Ordering::Equal => {
                    if a.1.local != b.1.local {
                        out.push(a.0);
                    }
                    i += 1;
                    j += 1;
                }
            }
        }
        out
    }

    /// True if `body` seen through `path` is a valid placement: the path ends at the body's
    /// component.
    pub(crate) fn path_ends_at(&self, path: &[OccurrenceId], component: ComponentId) -> bool {
        self.leaf_component(path) == Some(component)
    }
}

/// One placement of a body: the occurrence path from the root and the body-to-world transform.
#[derive(Clone, Debug, PartialEq)]
pub struct BodyInstance {
    /// Path from the root to the body's component.
    pub path: OccPath,
    /// Transform from the body's (component) space to world space.
    pub world: Transform,
}
