//! Selection (spec §8, Interfaces §A `amg_ui::select`).
//!
//! - [`EntityFilter`]: the kinds a pick or a selection accepts. The user's filter bar
//!   ([`bar`]) sets [`Selection::filter`]; tools intersect it with their own filter.
//! - [`SelItem`]: a picked target and the document entity it stands for. Body sub-entities get
//!   a persistent [`TopoRef`](amg_doc::TopoRef) through [`naming`] (provisional S01 R1/R2).
//! - [`SelectPriority`] and [`item_for_hit`]: what a hit selects (the body, the face under an
//!   edge, the component, ...).
//! - [`Selection`]: ordered, de-duplicated by target, with a revision counter; items whose
//!   reference is gone are dropped on document events ([`Selection::revalidate`]).
//! - [`sets`]: named selection sets in the `ui.selection_sets` extension record.
//! - [`SelectTool`] (the default tool): click / Ctrl / Shift rules, window and crossing box
//!   selection over the pick service's rectangle picks, and the select-through list.

pub mod bar;
pub mod naming;
pub mod sets;
mod tool;

pub use tool::{
    LONG_PRESS, LONG_PRESS_SLOP_PT, SELECT_TOOL, SelectThrough, SelectThroughEntry, SelectTool,
};

use amg_doc::{BodyGeom, Design, DocEvent, EntityRef, Evaluated, TopoKind};
use serde::{Deserialize, Serialize};

use crate::pick::PickTarget;
use crate::scene::SceneAdapter;

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

    /// Kinds in `self` but not in `other`.
    #[must_use]
    pub const fn without(self, other: EntityFilter) -> EntityFilter {
        EntityFilter(self.0 & !other.0)
    }

    /// The filter bit of a body sub-entity kind.
    pub fn of_topo(kind: TopoKind) -> EntityFilter {
        match kind {
            TopoKind::Face => EntityFilter::FACE,
            TopoKind::Edge => EntityFilter::EDGE,
            TopoKind::Vertex => EntityFilter::VERTEX,
            TopoKind::FaceGroup => EntityFilter::MESH_FACE_GROUP,
        }
    }

    /// Whether a pick target passes the filter. A body sub-entity passes if its own kind is in
    /// the set or if whole bodies are (the click then selects the body).
    pub fn admits(self, t: &PickTarget) -> bool {
        match t {
            PickTarget::Body { sub: None, .. } => self.contains(EntityFilter::BODY),
            PickTarget::Body {
                sub: Some((kind, _)),
                ..
            } => self.intersects(EntityFilter::of_topo(*kind).union(EntityFilter::BODY)),
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

/// The groups of the selection-filter bar: label and the kinds it switches.
pub const FILTER_GROUPS: &[(&str, EntityFilter)] = &[
    ("Bodies", EntityFilter::BODY),
    (
        "Faces",
        EntityFilter::FACE.union(EntityFilter::MESH_FACE_GROUP),
    ),
    ("Edges", EntityFilter::EDGE),
    ("Vertices", EntityFilter::VERTEX),
    (
        "Sketches",
        EntityFilter::SKETCH
            .union(EntityFilter::SKETCH_CURVE)
            .union(EntityFilter::SKETCH_POINT)
            .union(EntityFilter::PROFILE),
    ),
    ("Construction", EntityFilter::CONSTRUCTION),
    ("Components", EntityFilter::OCCURRENCE),
];

/// What a click on a body selects (spec §8 "Select priority").
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SelectPriority {
    /// The whole body when the filter has bodies, otherwise the picked sub-entity the filter
    /// admits (with edges filtered out, the face under the cursor).
    #[default]
    Default,
    /// Always the whole body.
    Body,
    /// Faces: edges and vertices are not picked, so a click on an edge selects the face under
    /// the cursor.
    Face,
    /// Edges only.
    Edge,
    /// The top-level occurrence the body is seen through (the body itself in the root).
    Component,
}

impl SelectPriority {
    /// Every priority, in filter-bar order.
    pub const ALL: [SelectPriority; 5] = [
        SelectPriority::Default,
        SelectPriority::Body,
        SelectPriority::Face,
        SelectPriority::Edge,
        SelectPriority::Component,
    ];

    /// Label in the filter bar.
    pub fn label(self) -> &'static str {
        match self {
            SelectPriority::Default => "Auto",
            SelectPriority::Body => "Body",
            SelectPriority::Face => "Face",
            SelectPriority::Edge => "Edge",
            SelectPriority::Component => "Component",
        }
    }

    /// The kinds a pick requests under this priority (intersected with the other filters):
    /// `Face` leaves out edges and vertices, `Edge` leaves out bodies, faces and vertices.
    pub fn pick_filter(self) -> EntityFilter {
        match self {
            SelectPriority::Face => {
                EntityFilter::ALL.without(EntityFilter::EDGE | EntityFilter::VERTEX)
            }
            SelectPriority::Edge => EntityFilter::ALL.without(
                EntityFilter::BODY
                    | EntityFilter::FACE
                    | EntityFilter::MESH_FACE_GROUP
                    | EntityFilter::VERTEX,
            ),
            SelectPriority::Default | SelectPriority::Body | SelectPriority::Component => {
                EntityFilter::ALL
            }
        }
    }
}

/// The face a body sub-entity hit stands for: the face itself, or for a mesh triangle edge its
/// triangle (`edge / 3`). A B-rep edge → face needs adjacency (UI-22).
fn face_of(eval: &Evaluated, hit: &PickTarget) -> Option<u32> {
    let PickTarget::Body {
        body,
        sub: Some((kind, index)),
        ..
    } = hit
    else {
        return None;
    };
    match kind {
        TopoKind::Face => Some(*index),
        TopoKind::Edge => match &eval.body(*body)?.geom {
            BodyGeom::Mesh(_) => Some(*index / 3),
            _ => None,
        },
        TopoKind::Vertex | TopoKind::FaceGroup => None,
    }
}

/// What a hit selects under `filter` (tool ∩ filter bar) and `priority`, as a [`SelItem`] with
/// its document reference. `None`: the hit selects nothing (filtered out, or a sub-entity
/// without a persistent reference).
pub fn item_for_hit(
    hit: PickTarget,
    filter: EntityFilter,
    priority: SelectPriority,
    eval: &Evaluated,
    scene: &SceneAdapter,
) -> Option<SelItem> {
    let PickTarget::Body {
        body,
        instance,
        sub,
    } = hit
    else {
        return if filter.admits(&hit) {
            SelItem::from_target(hit)
        } else {
            None
        };
    };
    let whole = PickTarget::Body {
        body,
        instance,
        sub: None,
    };
    if priority == SelectPriority::Component
        && filter.contains(EntityFilter::OCCURRENCE)
        && let Some(path) = scene.occurrence_path(instance)
        && let Some(&top) = path.first()
    {
        return Some(SelItem {
            target: PickTarget::Occurrence(top),
            entity: EntityRef::InContext {
                path: vec![top],
                target: Box::new(EntityRef::Body(body)),
            },
        });
    }
    let face = |f: Option<u32>| {
        f.filter(|_| filter.contains(EntityFilter::FACE))
            .map(|i| PickTarget::Body {
                body,
                instance,
                sub: Some((TopoKind::Face, i)),
            })
    };
    let own = sub.is_some_and(|(k, _)| filter.contains(EntityFilter::of_topo(k)));
    let chosen = match priority {
        SelectPriority::Body | SelectPriority::Component => {
            filter.contains(EntityFilter::BODY).then_some(whole)
        }
        SelectPriority::Face => face(face_of(eval, &hit)),
        SelectPriority::Edge => {
            (own && sub.is_some_and(|(k, _)| k == TopoKind::Edge)).then_some(hit)
        }
        SelectPriority::Default => {
            if filter.contains(EntityFilter::BODY) {
                Some(whole)
            } else if own {
                Some(hit)
            } else {
                face(face_of(eval, &hit))
            }
        }
    }?;
    SelItem::resolve(chosen, eval)
}

/// The displayed target a stored reference stands for now (selection sets), or `None` when it
/// no longer resolves or is not displayed. Bodies use their first displayed instance.
pub fn target_for_entity(
    design: &Design,
    eval: &Evaluated,
    scene: &SceneAdapter,
    entity: &EntityRef,
) -> Option<PickTarget> {
    let instance = |b| scene.pick_map().instances_of(b).first().copied();
    match entity {
        EntityRef::Body(b) => {
            eval.body(*b)?;
            Some(PickTarget::Body {
                body: *b,
                instance: instance(*b)?,
                sub: None,
            })
        }
        EntityRef::Topo(r) => {
            let index = naming::resolve_topo(eval, r)?;
            Some(PickTarget::Body {
                body: r.body,
                instance: instance(r.body)?,
                sub: Some((r.kind, index)),
            })
        }
        EntityRef::Construction(c) => {
            eval.construction(*c)?;
            Some(PickTarget::Construction(*c))
        }
        EntityRef::InContext { path, target } => match (path.as_slice(), target.as_ref()) {
            ([occ], EntityRef::Body(b)) => {
                design.occurrence(*occ)?;
                eval.body(*b)?;
                Some(PickTarget::Occurrence(*occ))
            }
            _ => None,
        },
        EntityRef::SketchCurve { .. }
        | EntityRef::SketchPoint { .. }
        | EntityRef::Profile { .. } => None,
    }
}

/// Whether a stored reference still resolves.
fn alive(e: &EntityRef, design: &Design, eval: &Evaluated) -> bool {
    match e {
        EntityRef::Body(b) => eval.body(*b).is_some(),
        EntityRef::Topo(r) => naming::resolve_topo(eval, r).is_some(),
        EntityRef::Construction(c) => eval.construction(*c).is_some(),
        EntityRef::SketchCurve { sketch, .. }
        | EntityRef::SketchPoint { sketch, .. }
        | EntityRef::Profile { sketch, .. } => eval.sketch(*sketch).is_some(),
        EntityRef::InContext { path, target } => {
            path.iter().all(|o| design.occurrence(*o).is_some()) && alive(target, design, eval)
        }
    }
}

/// The target `item` has after a document change, or `None` if its reference is gone.
fn revalidated(item: &SelItem, design: &Design, eval: &Evaluated) -> Option<PickTarget> {
    match (&item.entity, item.target) {
        (
            EntityRef::Topo(r),
            PickTarget::Body {
                body,
                instance,
                sub: Some((kind, _)),
            },
        ) => {
            let index = naming::resolve_topo(eval, r)?;
            Some(PickTarget::Body {
                body,
                instance,
                sub: Some((kind, index)),
            })
        }
        (e, t) => alive(e, design, eval).then_some(t),
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
    /// need naming ([`SelItem::resolve`]) and return `None` here.
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

    /// The item for a picked target, with a persistent reference for body sub-entities
    /// ([`naming::topo_ref_at`]). `None` if the target has no document reference.
    pub fn resolve(target: PickTarget, eval: &Evaluated) -> Option<SelItem> {
        match target {
            PickTarget::Body {
                body,
                sub: Some((kind, index)),
                ..
            } => Some(SelItem {
                target,
                entity: EntityRef::Topo(naming::topo_ref_at(eval, body, kind, index)?),
            }),
            _ => SelItem::from_target(target),
        }
    }
}

/// The selection of one document: ordered, de-duplicated by target.
#[derive(Clone, Debug)]
pub struct Selection {
    items: Vec<SelItem>,
    filter: EntityFilter,
    priority: SelectPriority,
    revision: u64,
}

impl Default for Selection {
    fn default() -> Self {
        Selection {
            items: Vec::new(),
            filter: EntityFilter::ALL,
            priority: SelectPriority::Default,
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

    /// Adds every item not yet selected (box selection with Ctrl / Shift). True if any was new.
    pub fn extend(&mut self, items: impl IntoIterator<Item = SelItem>) -> bool {
        let before = self.items.len();
        for i in items {
            if !self.contains(&i.target) {
                self.items.push(i);
            }
        }
        let added = self.items.len() != before;
        if added {
            self.bump();
        }
        added
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

    /// The user's filter (the filter bar).
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

    /// The select priority (the filter bar).
    pub fn priority(&self) -> SelectPriority {
        self.priority
    }

    /// Sets the select priority.
    pub fn set_priority(&mut self, p: SelectPriority) {
        if p != self.priority {
            self.priority = p;
            self.bump();
        }
    }

    /// Brings the selection in line with a document change (spec §8 "Revalidation"):
    /// `DocumentReplaced` clears it; after body, construction, sketch or component-tree events
    /// every item whose reference no longer resolves is dropped, and a sub-entity whose index
    /// moved gets its new index. Returns the number of dropped items. Never fails.
    pub fn revalidate(&mut self, events: &[DocEvent], design: &Design, eval: &Evaluated) -> usize {
        if self.items.is_empty() {
            return 0;
        }
        if events
            .iter()
            .any(|e| matches!(e, DocEvent::DocumentReplaced))
        {
            let n = self.items.len();
            self.clear();
            return n;
        }
        let relevant = events.iter().any(|e| {
            matches!(
                e,
                DocEvent::BodyRemoved(_)
                    | DocEvent::BodyGeometryChanged(_)
                    | DocEvent::ConstructionChanged(_)
                    | DocEvent::SketchChanged(_)
                    | DocEvent::ComponentTreeChanged
            )
        });
        if !relevant {
            return 0;
        }
        let before = std::mem::take(&mut self.items);
        let n = before.len();
        let mut changed = false;
        for mut item in before {
            match revalidated(&item, design, eval) {
                Some(t) => {
                    changed |= t != item.target;
                    item.target = t;
                    if !self.items.iter().any(|o| o.target == item.target) {
                        self.items.push(item);
                    }
                }
                None => changed = true,
            }
        }
        if changed {
            self.bump();
        }
        n - self.items.len()
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
        assert!(s.extend([body(2), body(3)]));
        assert_eq!(s.items(), &[body(2), body(3)]);
        assert!(!s.extend([body(3)]));
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
            sub: Some((TopoKind::Face, 3)),
        };
        assert!(EntityFilter::FACE.admits(&face));
        assert!(EntityFilter::BODY.admits(&face));
        assert!(!EntityFilter::EDGE.admits(&face));
        assert!(!EntityFilter::FACE.admits(&face.whole()));
        assert_eq!(
            (EntityFilter::BODY | EntityFilter::FACE) & EntityFilter::FACE,
            EntityFilter::FACE
        );
        assert!(
            !SelectPriority::Face
                .pick_filter()
                .intersects(EntityFilter::EDGE)
        );
        assert!(
            !SelectPriority::Edge
                .pick_filter()
                .intersects(EntityFilter::FACE)
        );
    }

    #[test]
    fn revalidation_drops_missing_bodies_and_document_replaced_clears() {
        let design = Design::new("t");
        let eval = Evaluated::new(amg_doc::Revision(1));
        let mut s = Selection::default();
        s.set(vec![body(1), body(2)]);
        assert_eq!(
            s.revalidate(&[DocEvent::BodyAdded(body_id(3))], &design, &eval),
            0
        );
        assert_eq!(s.len(), 2, "unrelated events keep the items");
        assert_eq!(
            s.revalidate(&[DocEvent::BodyRemoved(body_id(1))], &design, &eval),
            2
        );
        assert!(s.is_empty());
        s.set(vec![body(1)]);
        assert_eq!(
            s.revalidate(&[DocEvent::DocumentReplaced], &design, &eval),
            1
        );
        assert!(s.is_empty());
    }

    fn body_id(n: u64) -> BodyId {
        BodyId {
            origin: amg_doc::FeatureId(n),
            slot: 0,
        }
    }
}
