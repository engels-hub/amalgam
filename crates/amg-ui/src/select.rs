//! Selection state (spec §8, Interfaces §A `amg_ui::select`).
//!
//! UI-03 ships what the click-only [`SelectTool`](crate::tool::SelectTool) needs: the
//! [`EntityFilter`] bit set, [`SelItem`] and an ordered, de-duplicated [`Selection`] with a
//! revision counter. UI-04 adds the filter bar, select priority, box selection, select-through,
//! selection sets and revalidation on document events.

use amg_doc::EntityRef;

use crate::pick::PickTarget;

/// Which kinds of entity a pick or a selection accepts (a bit set).
///
/// Spec: `pub use amg_features::dialog::EntityFilter` (Interfaces §B). `amg_features::dialog`
/// does not exist yet (S04 SOL-01), so the set lives here until then and becomes a re-export.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct EntityFilter(u32);

impl EntityFilter {
    /// Nothing.
    pub const NONE: EntityFilter = EntityFilter(0);
    /// Whole bodies.
    pub const BODY: EntityFilter = EntityFilter(1);
    /// Faces (B-rep faces, mesh triangles).
    pub const FACE: EntityFilter = EntityFilter(1 << 1);
    /// Edges.
    pub const EDGE: EntityFilter = EntityFilter(1 << 2);
    /// Vertices.
    pub const VERTEX: EntityFilter = EntityFilter(1 << 3);
    /// Mesh face groups.
    pub const MESH_FACE_GROUP: EntityFilter = EntityFilter(1 << 4);
    /// Whole sketches.
    pub const SKETCH: EntityFilter = EntityFilter(1 << 5);
    /// Sketch curves.
    pub const SKETCH_CURVE: EntityFilter = EntityFilter(1 << 6);
    /// Sketch points.
    pub const SKETCH_POINT: EntityFilter = EntityFilter(1 << 7);
    /// Sketch profiles.
    pub const PROFILE: EntityFilter = EntityFilter(1 << 8);
    /// Construction planes, axes and points.
    pub const CONSTRUCTION: EntityFilter = EntityFilter(1 << 9);
    /// Occurrences / components.
    pub const OCCURRENCE: EntityFilter = EntityFilter(1 << 10);
    /// Overlay handles (manipulators, UI-06).
    pub const HANDLE: EntityFilter = EntityFilter(1 << 11);
    /// Everything.
    pub const ALL: EntityFilter = EntityFilter((1 << 12) - 1);

    /// The raw bits.
    pub fn bits(self) -> u32 {
        self.0
    }

    /// Whether every kind of `other` is in the set.
    pub fn contains(self, other: EntityFilter) -> bool {
        self.0 & other.0 == other.0
    }

    /// Whether the sets share a kind.
    pub fn intersects(self, other: EntityFilter) -> bool {
        self.0 & other.0 != 0
    }

    /// Kinds in both sets.
    #[must_use]
    pub const fn intersection(self, other: EntityFilter) -> EntityFilter {
        EntityFilter(self.0 & other.0)
    }

    /// Kinds in either set.
    #[must_use]
    pub const fn union(self, other: EntityFilter) -> EntityFilter {
        EntityFilter(self.0 | other.0)
    }

    /// Whether a pick target passes the filter. A body sub-entity passes if its own kind is in
    /// the set or if whole bodies are (the click then selects the body).
    pub fn admits(self, t: &PickTarget) -> bool {
        match t {
            PickTarget::Body { sub: None, .. } => self.contains(EntityFilter::BODY),
            PickTarget::Body {
                sub: Some((kind, _)),
                ..
            } => {
                let own = match kind {
                    amg_doc::TopoKind::Face => EntityFilter::FACE,
                    amg_doc::TopoKind::Edge => EntityFilter::EDGE,
                    amg_doc::TopoKind::Vertex => EntityFilter::VERTEX,
                    amg_doc::TopoKind::FaceGroup => EntityFilter::MESH_FACE_GROUP,
                };
                self.intersects(own.union(EntityFilter::BODY))
            }
            PickTarget::Sketch(_) => self.intersects(
                EntityFilter::SKETCH
                    .union(EntityFilter::SKETCH_CURVE)
                    .union(EntityFilter::SKETCH_POINT)
                    .union(EntityFilter::PROFILE),
            ),
            PickTarget::Construction(_) => self.contains(EntityFilter::CONSTRUCTION),
            PickTarget::Occurrence(_) => self.contains(EntityFilter::OCCURRENCE),
        }
    }
}

impl Default for EntityFilter {
    fn default() -> Self {
        EntityFilter::ALL
    }
}

impl std::ops::BitAnd for EntityFilter {
    type Output = EntityFilter;
    fn bitand(self, rhs: EntityFilter) -> EntityFilter {
        self.intersection(rhs)
    }
}

impl std::ops::BitOr for EntityFilter {
    type Output = EntityFilter;
    fn bitor(self, rhs: EntityFilter) -> EntityFilter {
        self.union(rhs)
    }
}

/// One selected thing: its display identity and its document reference.
#[derive(Clone, Debug, PartialEq)]
pub struct SelItem {
    /// What was picked.
    pub target: PickTarget,
    /// The document entity it stands for.
    pub entity: EntityRef,
}

impl SelItem {
    /// The item for a picked target, when the target maps to a document entity without
    /// persistent naming: whole bodies, sketches and construction geometry. Body sub-entities
    /// need `BodyNaming` (S01 R1, UI-04) and return `None` here.
    pub fn from_target(target: PickTarget) -> Option<SelItem> {
        let entity = match target {
            PickTarget::Body {
                body, sub: None, ..
            } => EntityRef::Body(body),
            PickTarget::Construction(c) => EntityRef::Construction(c),
            PickTarget::Body { sub: Some(_), .. }
            | PickTarget::Sketch(_)
            | PickTarget::Occurrence(_) => return None,
        };
        Some(SelItem { target, entity })
    }
}

/// The selection of one document: ordered, de-duplicated by target.
#[derive(Clone, Debug)]
pub struct Selection {
    items: Vec<SelItem>,
    filter: EntityFilter,
    revision: u64,
}

impl Default for Selection {
    fn default() -> Self {
        Selection {
            items: Vec::new(),
            filter: EntityFilter::ALL,
            revision: 0,
        }
    }
}

impl Selection {
    /// The items, in selection order.
    pub fn items(&self) -> &[SelItem] {
        &self.items
    }

    /// Whether `t` is selected.
    pub fn contains(&self, t: &PickTarget) -> bool {
        self.items.iter().any(|i| i.target == *t)
    }

    /// Number of selected items.
    pub fn len(&self) -> usize {
        self.items.len()
    }

    /// Nothing is selected.
    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }

    /// Replaces the selection (duplicates are dropped, the first one stays).
    pub fn set(&mut self, items: Vec<SelItem>) {
        let mut out: Vec<SelItem> = Vec::with_capacity(items.len());
        for i in items {
            if !out.iter().any(|o| o.target == i.target) {
                out.push(i);
            }
        }
        if out != self.items {
            self.items = out;
            self.bump();
        }
    }

    /// Adds an item; false if it was already selected.
    pub fn add(&mut self, item: SelItem) -> bool {
        if self.contains(&item.target) {
            return false;
        }
        self.items.push(item);
        self.bump();
        true
    }

    /// Adds the item, or removes it if it is selected.
    pub fn toggle(&mut self, item: SelItem) {
        if !self.remove(&item.target) {
            self.add(item);
        }
    }

    /// Removes a target; false if it was not selected.
    pub fn remove(&mut self, t: &PickTarget) -> bool {
        let before = self.items.len();
        self.items.retain(|i| i.target != *t);
        let removed = self.items.len() != before;
        if removed {
            self.bump();
        }
        removed
    }

    /// Clears the selection.
    pub fn clear(&mut self) {
        if !self.items.is_empty() {
            self.items.clear();
            self.bump();
        }
    }

    /// The user's filter (the filter bar, UI-04).
    pub fn filter(&self) -> EntityFilter {
        self.filter
    }

    /// Sets the user's filter.
    pub fn set_filter(&mut self, f: EntityFilter) {
        if f != self.filter {
            self.filter = f;
            self.bump();
        }
    }

    /// Bumps on every change (cheap cache checks).
    pub fn revision(&self) -> u64 {
        self.revision
    }

    fn bump(&mut self) {
        self.revision += 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use amg_doc::BodyId;

    fn body(n: u64) -> SelItem {
        SelItem::from_target(PickTarget::Body {
            body: BodyId {
                origin: amg_doc::FeatureId(n),
                slot: 0,
            },
            instance: 0,
            sub: None,
        })
        .unwrap()
    }

    #[test]
    fn selection_is_ordered_deduplicated_and_counts_revisions() {
        let mut s = Selection::default();
        assert!(s.add(body(1)));
        assert!(!s.add(body(1)));
        s.toggle(body(2));
        assert_eq!(s.len(), 2);
        let r = s.revision();
        s.toggle(body(1));
        assert_eq!(s.items(), &[body(2)]);
        assert!(s.revision() > r);
        let r = s.revision();
        s.set(vec![body(2)]);
        assert_eq!(s.revision(), r, "no change, no bump");
        s.clear();
        assert!(s.is_empty());
    }

    #[test]
    fn filter_admits_sub_entities_through_their_kind_or_the_body() {
        let face = PickTarget::Body {
            body: BodyId {
                origin: amg_doc::FeatureId(1),
                slot: 0,
            },
            instance: 0,
            sub: Some((amg_doc::TopoKind::Face, 3)),
        };
        assert!(EntityFilter::FACE.admits(&face));
        assert!(EntityFilter::BODY.admits(&face));
        assert!(!EntityFilter::EDGE.admits(&face));
        assert!(!EntityFilter::FACE.admits(&face.whole()));
        assert_eq!(
            (EntityFilter::BODY | EntityFilter::FACE) & EntityFilter::FACE,
            EntityFilter::FACE
        );
    }
}
