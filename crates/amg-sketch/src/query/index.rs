//! The query index: evaluated curves and points of a sketch with their bounding boxes in flat
//! arrays, so a hover query rejects almost everything with four comparisons per entity.

use amg_core::{Point2, Tolerance};

use crate::geom::Curve2;
use crate::ids::EntityId;
use crate::model::{EntityKind, EntityOrigin, Sketch};

use super::{PickFilter, PickHit};

/// How a point entity is used by the curves.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(super) enum PointRole {
    /// End point of a curve, or a free point.
    Endpoint,
    /// Only the centre of circles, arcs or ellipses.
    Center,
}

#[derive(Clone, Debug)]
pub(super) enum Shape {
    Point(Point2, PointRole),
    Curve(Curve2),
}

#[derive(Clone, Debug)]
pub(super) struct Item {
    pub id: EntityId,
    pub shape: Shape,
    pub construction: bool,
    pub projected: bool,
}

/// Evaluated geometry of one sketch for [`pick`](super::pick) and
/// [`snap_candidates`](super::snap_candidates).
///
/// Building it evaluates every entity once (about 0.1 µs each); queries then touch only the
/// entities near the cursor. Keep one per sketch revision: [`matches`](QueryIndex::matches)
/// tells whether it is current and [`refresh`](QueryIndex::refresh) rebuilds it. The free
/// functions [`pick`](super::pick) and [`snap_candidates`](super::snap_candidates) keep a
/// per-thread cache of the last index, so repeated hover queries on an unchanged sketch do not
/// rebuild.
#[derive(Clone, Debug)]
pub struct QueryIndex {
    structure_rev: u64,
    values_rev: u64,
    pub(super) items: Vec<Item>,
    /// `[min_x, min_y, max_x, max_y]` per item.
    boxes: Vec<[f64; 4]>,
    /// Indices into `items` of the point items.
    pub(super) points: Vec<u32>,
}

impl QueryIndex {
    /// Evaluates `sketch`. Curves that cannot be evaluated (a degenerate spline) are left out.
    pub fn new(sketch: &Sketch) -> QueryIndex {
        let roles = roles(sketch);
        let mut items = Vec::new();
        let mut boxes = Vec::new();
        let mut points = Vec::new();
        for (id, e) in sketch.entities() {
            let shape = if matches!(e.kind, EntityKind::Point { .. }) {
                let Some(p) = sketch.point(id) else { continue };
                let role = roles.get(id.index()).copied().unwrap_or(0);
                let role = if role == CENTER {
                    PointRole::Center
                } else {
                    PointRole::Endpoint
                };
                Shape::Point(p, role)
            } else {
                match sketch.curve(id) {
                    Some(c) => Shape::Curve(c),
                    None => continue,
                }
            };
            let b = match &shape {
                Shape::Point(p, _) => [p.x, p.y, p.x, p.y],
                Shape::Curve(c) => {
                    let b = c.bbox();
                    [b.min.x, b.min.y, b.max.x, b.max.y]
                }
            };
            if !b.iter().all(|v| v.is_finite()) {
                continue;
            }
            if matches!(shape, Shape::Point(..)) {
                points.push(u32::try_from(items.len()).unwrap_or(u32::MAX));
            }
            boxes.push(b);
            items.push(Item {
                id,
                shape,
                construction: e.construction,
                projected: matches!(e.origin, EntityOrigin::Projected(_)),
            });
        }
        QueryIndex {
            structure_rev: sketch.structure_rev(),
            values_rev: sketch.values_rev(),
            items,
            boxes,
            points,
        }
    }

    /// True if the index was built from this revision of `sketch`.
    pub fn matches(&self, sketch: &Sketch) -> bool {
        self.structure_rev == sketch.structure_rev() && self.values_rev == sketch.values_rev()
    }

    /// Rebuilds the index if `sketch` changed since it was built.
    pub fn refresh(&mut self, sketch: &Sketch) {
        if !self.matches(sketch) {
            *self = QueryIndex::new(sketch);
        }
    }

    /// Number of indexed entities.
    pub fn len(&self) -> usize {
        self.items.len()
    }

    /// True if nothing is indexed.
    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }

    /// Indices of the items whose box is within `r` of `p` (a square test).
    pub(super) fn near(&self, p: Point2, r: f64) -> impl Iterator<Item = usize> + '_ {
        let (lx, ly, hx, hy) = (p.x - r, p.y - r, p.x + r, p.y + r);
        self.boxes
            .iter()
            .enumerate()
            .filter(move |(_, b)| b[0] <= hx && b[2] >= lx && b[1] <= hy && b[3] >= ly)
            .map(|(i, _)| i)
    }

    /// The point entity at `p` within a micrometre, if any.
    pub(super) fn point_at(&self, p: Point2) -> Option<EntityId> {
        let tol = Tolerance::DEFAULT.linear;
        self.points.iter().find_map(|&i| {
            let it = &self.items[i as usize];
            match it.shape {
                Shape::Point(q, _) if q.distance(p) <= tol => Some(it.id),
                _ => None,
            }
        })
    }

    /// Entities within `radius` of `p`, nearest first (ties: points before curves, then by
    /// id). See [`pick`](super::pick).
    pub fn pick(&self, p: Point2, radius: f64, filter: PickFilter) -> Vec<PickHit> {
        let mut hits: Vec<(i64, u8, PickHit)> = Vec::new();
        for i in self.near(p, radius) {
            let it = &self.items[i];
            if (it.construction && !filter.construction) || (it.projected && !filter.projected) {
                continue;
            }
            let (rank, distance, param) = match &it.shape {
                Shape::Point(q, _) if filter.points => (0, q.distance(p), 0.0),
                Shape::Curve(c) if filter.curves => {
                    let (t, q) = c.closest(p);
                    (1, q.distance(p), t)
                }
                _ => continue,
            };
            if distance <= radius {
                let key = (distance * 1e9).round() as i64;
                hits.push((
                    key,
                    rank,
                    PickHit {
                        entity: it.id,
                        distance,
                        param,
                    },
                ));
            }
        }
        hits.sort_by_key(|(key, rank, h)| (*key, *rank, h.entity));
        hits.into_iter().map(|(_, _, h)| h).collect()
    }
}

const CENTER: u8 = 1;
const END: u8 = 2;

/// Per point entity: `END` if some curve ends there (or it is a spline control point), else
/// `CENTER` if it is only a centre.
fn roles(sketch: &Sketch) -> Vec<u8> {
    let max = sketch.entities().map(|(id, _)| id.index()).max();
    let mut roles = vec![0u8; max.map_or(0, |m| m + 1)];
    let mut mark = |id: EntityId, flag: u8| {
        if let Some(r) = roles.get_mut(id.index()) {
            *r |= flag;
        }
    };
    for (_, e) in sketch.entities() {
        match e.kind {
            EntityKind::Point { .. } => {}
            EntityKind::Line { start, end } => {
                mark(start, END);
                mark(end, END);
            }
            EntityKind::Circle { center, .. } => mark(center, CENTER),
            EntityKind::Arc { center, start, end } => {
                mark(center, CENTER);
                mark(start, END);
                mark(end, END);
            }
            EntityKind::Ellipse { center, major, .. } => {
                mark(center, CENTER);
                mark(major, END);
            }
            EntityKind::EllipticArc {
                center,
                major,
                start,
                end,
                ..
            } => {
                mark(center, CENTER);
                mark(major, END);
                mark(start, END);
                mark(end, END);
            }
            _ => {
                for c in e.kind.children() {
                    mark(c, END);
                }
            }
        }
    }
    // A point that is only a centre keeps the CENTER role; any end use wins.
    for r in &mut roles {
        if *r & END != 0 {
            *r = END;
        }
    }
    roles
}
