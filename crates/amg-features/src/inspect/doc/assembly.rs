//! Assembly side of the physical properties (INS-10): which body instances a selection stands
//! for, and which component and occurrence totals each instance belongs to.

use std::collections::{BTreeMap, BTreeSet};

use amg_core::Transform;
use amg_doc::{BodyGeom, BodyId, ComponentId, Design, Evaluated, OccPath, OccurrenceId};

/// One selected thing.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PropsPick {
    /// One placement of a body. With `path: None`, or a path that does not place the body, its
    /// first placement is taken.
    Body {
        /// The body.
        body: BodyId,
        /// Occurrence path from the root.
        path: Option<OccPath>,
    },
    /// Every body instance seen through this occurrence.
    Occurrence(OccurrenceId),
}

/// One placed body.
#[derive(Clone, Debug, PartialEq)]
pub struct PlacedBody {
    /// The body.
    pub body: BodyId,
    /// Occurrence path from the root (empty for the root component's bodies).
    pub path: OccPath,
    /// Body to world.
    pub world: Transform,
}

/// Whether the body has properties to compute (mesh and solid bodies).
pub fn is_measurable(evaluated: &Evaluated, body: BodyId) -> bool {
    evaluated
        .body(body)
        .is_some_and(|b| matches!(b.geom, BodyGeom::Mesh(_) | BodyGeom::Solid(_)))
}

/// Every instance of every mesh and solid body.
fn all_instances(evaluated: &Evaluated) -> impl Iterator<Item = (BodyId, OccPath, Transform)> + '_ {
    evaluated
        .bodies()
        .map(|(id, _)| id)
        .filter(|id| is_measurable(evaluated, *id))
        .flat_map(|id| {
            evaluated
                .instances(id)
                .into_iter()
                .map(move |i| (id, i.path, i.world))
        })
}

/// The body instances `picks` stand for, each once, ordered by body then path. Body picks that
/// name no mesh or solid body are ignored. If no pick is left (an empty selection), the result
/// is every instance of every mesh and solid body: the whole assembly. A selected occurrence
/// without any such body gives nothing.
pub fn selected_instances(evaluated: &Evaluated, picks: &[PropsPick]) -> Vec<PlacedBody> {
    let mut out: BTreeMap<(BodyId, OccPath), Transform> = BTreeMap::new();
    let mut occurrences: BTreeSet<OccurrenceId> = BTreeSet::new();
    let mut picked = false;
    for pick in picks {
        match pick {
            PropsPick::Body { body, path } => {
                if !is_measurable(evaluated, *body) {
                    continue;
                }
                picked = true;
                let named = path.as_ref().and_then(|p| {
                    let w = evaluated.body_to_world(*body, p)?;
                    Some((p.clone(), w))
                });
                let first = || {
                    let i = evaluated.instances(*body).into_iter().next()?;
                    Some((i.path, i.world))
                };
                if let Some((p, w)) = named.or_else(first) {
                    out.insert((*body, p), w);
                }
            }
            PropsPick::Occurrence(o) => {
                picked = true;
                occurrences.insert(*o);
            }
        }
    }
    if !picked || !occurrences.is_empty() {
        for (id, path, world) in all_instances(evaluated) {
            if !picked || path.iter().any(|o| occurrences.contains(o)) {
                out.insert((id, path), world);
            }
        }
    }
    out.into_iter()
        .map(|((body, path), world)| PlacedBody { body, path, world })
        .collect()
}

/// A total an instance counts into.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum PropsScope {
    /// One occurrence in the tree, by its path from the root: its component's bodies and
    /// everything below.
    Occurrence(OccPath),
    /// A component, summed over all its placements (its own bodies and its children).
    Component(ComponentId),
}

/// The scopes of the instance placed through `path`: every occurrence on the path (as the
/// prefix ending there), the root component and the component of every occurrence on the path.
/// No component appears twice (a component cannot contain itself).
pub fn scopes_of(design: &Design, path: &[OccurrenceId]) -> Vec<PropsScope> {
    let mut out = vec![PropsScope::Component(design.root())];
    for (i, o) in path.iter().enumerate() {
        out.push(PropsScope::Occurrence(path[..=i].to_vec()));
        if let Some(occ) = design.occurrence(*o) {
            out.push(PropsScope::Component(occ.component));
        }
    }
    out
}

/// The display name of an occurrence: its name override, else `"<component>:<n>"` with `n`
/// counting the occurrences of that component in the same parent from 1.
pub fn occurrence_name(design: &Design, o: OccurrenceId) -> String {
    let fallback = || format!("Occurrence {}", o.0);
    let Some(occ) = design.occurrence(o) else {
        return fallback();
    };
    if let Some(name) = &occ.name {
        return name.clone();
    }
    let (Some(component), Some(parent)) = (
        design.component(occ.component),
        design.component(occ.parent),
    ) else {
        return fallback();
    };
    let n = parent
        .children
        .iter()
        .filter(|s| {
            design
                .occurrence(**s)
                .is_some_and(|s| s.component == occ.component)
        })
        .position(|s| *s == o)
        .map_or(1, |i| i + 1);
    format!("{}:{n}", component.name)
}
