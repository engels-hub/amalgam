//! The sketch inference engine (spec §13 *Inference*) on SK-20's `query::snap_candidates`.
//!
//! [`infer`] runs on the UI thread for every cursor move of a creation tool (SK-20: pure, far
//! below 0.1 ms). It returns where the point goes and why:
//!
//! - **Point snaps** (endpoint, centre, midpoint, intersection, quadrant, tangent,
//!   perpendicular, on-curve) win over alignments; the best one moves the point onto the
//!   snap and its suggested constraint is added on commit.
//! - **Alignment** with the rubber-band anchor makes the new line horizontal / vertical
//!   (`Horizontal(Line)` / `Vertical(Line)`). Alignment with another point snaps the
//!   position (both axes combine) and, when the line itself gets no direction constraint,
//!   adds `Horizontal(Points)` / `Vertical(Points)` with that point.
//! - **Ctrl** suppresses inference (no snap, no constraint). **Grid snap** (palette) rounds
//!   the point to the grid when nothing else snapped.
//!
//! [`Inference::constraints`] resolves the suggestions for the entities a tool created.

use amg_core::Point2;
use amg_sketch::query::{self, NEW_ENTITY, SnapCandidate, SnapKind, SnapOptions};
use amg_sketch::{ConstraintKind, EntityId, HvTarget, Sketch};

/// Snap radius around the cursor, points.
pub const SNAP_RADIUS_PT: f64 = 8.0;

/// Lines this close to axis-parallel (mm over their length) count as horizontal / vertical.
const AXIS_EPS: f64 = 1e-7;

/// The rubber-band anchor of a creation tool.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Anchor {
    /// Position, sketch coordinates.
    pub at: Point2,
    /// The point entity there, if it exists already.
    pub entity: Option<EntityId>,
}

/// What [`infer`] decided for one cursor position.
#[derive(Clone, Debug, Default)]
pub struct Inference {
    /// Where the point goes (sketch coordinates).
    pub point: Point2,
    /// The point snap used, if any (not an alignment).
    pub snap: Option<SnapCandidate>,
    /// Alignment with another point (not the anchor), horizontal and / or vertical.
    pub aligns: Vec<SnapCandidate>,
    /// The new line from the anchor is horizontal (`Some(true)`) or vertical (`Some(false)`).
    pub line_axis: Option<bool>,
    /// The position came from the grid.
    pub grid: bool,
}

impl Inference {
    /// No inference: the point stays where it is.
    pub fn plain(p: Point2) -> Inference {
        Inference {
            point: p,
            ..Inference::default()
        }
    }

    /// The glyph next to the cursor (a short text; `None` without inference).
    pub fn glyph(&self) -> Option<&'static str> {
        if let Some(s) = &self.snap {
            return Some(kind_glyph(s.kind));
        }
        match self.line_axis {
            Some(true) => return Some("H"),
            Some(false) => return Some("V"),
            None => {}
        }
        self.aligns.first().map(|a| kind_glyph(a.kind))
    }

    /// Alignment guide lines `(from, to)`, sketch coordinates: from each aligned point (and
    /// the anchor for an axis-parallel line) to the inferred point.
    pub fn guides(&self, sketch: &Sketch, anchor: Option<Anchor>) -> Vec<(Point2, Point2)> {
        let mut out = Vec::new();
        for a in &self.aligns {
            if let Some(p) = a.entities.first().and_then(|e| sketch.point(*e)) {
                out.push((p, self.point));
            }
        }
        if let (Some(_), Some(a)) = (self.line_axis, anchor) {
            out.push((a.at, self.point));
        }
        out
    }

    /// The constraints for the new point `point` (and the new line `line` from the anchor,
    /// when there is one): the snap's suggestion, the line direction and, without a line
    /// direction, the alignments with other points. Placeholders are resolved.
    pub fn constraints(&self, point: EntityId, line: Option<EntityId>) -> Vec<ConstraintKind> {
        let mut out = Vec::new();
        if let Some(s) = &self.snap {
            for k in s.constraints(point, line.unwrap_or(NEW_ENTITY)) {
                if !k.entities().contains(&NEW_ENTITY) {
                    out.push(k);
                }
            }
        }
        if let (Some(axis), Some(l)) = (self.line_axis, line) {
            out.push(if axis {
                ConstraintKind::Horizontal(HvTarget::Line(l))
            } else {
                ConstraintKind::Vertical(HvTarget::Line(l))
            });
        } else if self.snap.is_none() {
            for a in &self.aligns {
                if let Some(k) = &a.suggested {
                    let k = query_resolve(k, point);
                    if !k.entities().contains(&NEW_ENTITY) {
                        out.push(k);
                    }
                }
            }
        }
        out
    }
}

/// Resolves the new-point placeholder of an alignment suggestion.
fn query_resolve(k: &ConstraintKind, point: EntityId) -> ConstraintKind {
    let sub = |e: EntityId| if e == NEW_ENTITY { point } else { e };
    match *k {
        ConstraintKind::Horizontal(HvTarget::Points(a, b)) => {
            ConstraintKind::Horizontal(HvTarget::Points(sub(a), sub(b)))
        }
        ConstraintKind::Vertical(HvTarget::Points(a, b)) => {
            ConstraintKind::Vertical(HvTarget::Points(sub(a), sub(b)))
        }
        ref other => other.clone(),
    }
}

/// A short glyph per snap kind (drawn next to the cursor).
pub fn kind_glyph(k: SnapKind) -> &'static str {
    match k {
        SnapKind::Endpoint => "End",
        SnapKind::Midpoint => "Mid",
        SnapKind::Center => "Ctr",
        SnapKind::Quadrant => "Quad",
        SnapKind::OnCurve => "On",
        SnapKind::Intersection => "X",
        SnapKind::AlignHorizontal => "-",
        SnapKind::AlignVertical => "|",
        SnapKind::Tangent => "Tan",
        SnapKind::Perpendicular => "Perp",
    }
}

/// Context of one [`infer`] call.
#[derive(Clone, Debug, Default)]
pub struct InferInput<'a> {
    /// The rubber-band anchor.
    pub anchor: Option<Anchor>,
    /// Snap radius, mm.
    pub radius: f64,
    /// Ctrl held: no inference.
    pub suppress: bool,
    /// Grid step, mm, when grid snap is on.
    pub grid: Option<f64>,
    /// Entities that are never snap targets.
    pub exclude: &'a [EntityId],
}

/// Infers the point for cursor `p` (sketch coordinates) on `sketch`.
pub fn infer(sketch: &Sketch, p: Point2, input: &InferInput<'_>) -> Inference {
    let grid_point = |p: Point2| match input.grid {
        Some(step) if step > 0.0 => Some(Point2::new(
            (p.x / step).round() * step,
            (p.y / step).round() * step,
        )),
        _ => None,
    };
    if input.suppress {
        return match grid_point(p) {
            Some(g) => Inference {
                point: g,
                grid: true,
                ..Inference::default()
            },
            None => Inference::plain(p),
        };
    }
    let opts = SnapOptions {
        from: input.anchor.map(|a| a.at),
        from_entity: input.anchor.and_then(|a| a.entity),
        exclude: input.exclude,
        ..SnapOptions::default()
    };
    let cands = query::snap_candidates_with(sketch, p, input.radius, &opts);
    let is_align = |k: SnapKind| matches!(k, SnapKind::AlignHorizontal | SnapKind::AlignVertical);
    let mut inf = Inference::plain(p);
    if let Some(best) = cands.iter().find(|c| !is_align(c.kind)) {
        inf.point = best.point;
        inf.snap = Some(best.clone());
    } else {
        // Alignment: the anchor's own guides make the line axis-parallel; the others snap.
        let anchor_at = input.anchor.map(|a| a.at);
        let anchor_ent = input.anchor.and_then(|a| a.entity);
        let with_anchor = |c: &SnapCandidate| match anchor_at {
            Some(a) => {
                anchor_ent.is_some_and(|e| c.entities.contains(&e))
                    || match c.kind {
                        SnapKind::AlignHorizontal => (c.point.y - a.y).abs() <= AXIS_EPS,
                        _ => (c.point.x - a.x).abs() <= AXIS_EPS,
                    }
            }
            None => false,
        };
        let mut h: Option<&SnapCandidate> = None;
        let mut v: Option<&SnapCandidate> = None;
        for c in cands.iter().filter(|c| is_align(c.kind)) {
            let slot = if c.kind == SnapKind::AlignHorizontal {
                &mut h
            } else {
                &mut v
            };
            // Prefer the anchor's guide, then the nearest (the list is sorted by distance).
            match slot {
                None => *slot = Some(c),
                Some(old) if !with_anchor(old) && with_anchor(c) => *slot = Some(c),
                _ => {}
            }
        }
        let mut point = p;
        if let Some(c) = h {
            point.y = c.point.y;
        }
        if let Some(c) = v {
            point.x = c.point.x;
        }
        inf.point = point;
        for c in [h, v].into_iter().flatten() {
            if !with_anchor(c) {
                inf.aligns.push(c.clone());
            }
        }
        if inf.aligns.is_empty()
            && h.is_none()
            && v.is_none()
            && let Some(g) = grid_point(p)
        {
            inf.point = g;
            inf.grid = true;
        }
    }
    if let Some(a) = input.anchor {
        let d = inf.point - a.at;
        let len = d.length();
        if len > 1e-9 {
            let tol = AXIS_EPS * len.max(1.0);
            if d.y.abs() <= tol {
                inf.line_axis = Some(true);
            } else if d.x.abs() <= tol {
                inf.line_axis = Some(false);
            }
        }
    }
    inf
}

#[cfg(test)]
mod tests {
    use super::*;
    use amg_core::Plane;

    fn square() -> (Sketch, EntityId) {
        let mut s = Sketch::new(Plane::XY);
        let l = s
            .add_line(Point2::new(0.0, 0.0), Point2::new(10.0, 0.0))
            .unwrap();
        (s, l)
    }

    #[test]
    fn endpoint_wins_and_suggests_coincidence() {
        let (s, _) = square();
        let inf = infer(
            &s,
            Point2::new(10.2, 0.1),
            &InferInput {
                radius: 1.0,
                ..InferInput::default()
            },
        );
        assert_eq!(inf.snap.as_ref().map(|c| c.kind), Some(SnapKind::Endpoint));
        assert_eq!(inf.point, Point2::new(10.0, 0.0));
        let cs = inf.constraints(EntityId(99), None);
        assert!(matches!(cs[..], [ConstraintKind::Coincident { .. }]));
    }

    #[test]
    fn anchor_alignment_makes_the_line_horizontal_and_ctrl_suppresses_it() {
        let (s, _) = square();
        let anchor = Anchor {
            at: Point2::new(30.0, 20.0),
            entity: None,
        };
        let input = InferInput {
            anchor: Some(anchor),
            radius: 1.0,
            ..InferInput::default()
        };
        let inf = infer(&s, Point2::new(50.0, 20.3), &input);
        assert!((inf.point.y - 20.0).abs() < 1e-12);
        assert_eq!(inf.line_axis, Some(true));
        let cs = inf.constraints(EntityId(98), Some(EntityId(99)));
        assert_eq!(
            cs,
            vec![ConstraintKind::Horizontal(HvTarget::Line(EntityId(99)))]
        );
        let off = infer(
            &s,
            Point2::new(50.0, 20.3),
            &InferInput {
                suppress: true,
                ..input
            },
        );
        assert_eq!(off.point, Point2::new(50.0, 20.3));
        assert!(off.snap.is_none() && off.line_axis.is_none() && off.glyph().is_none());
    }

    #[test]
    fn grid_snap_applies_only_without_inference() {
        let (s, _) = square();
        let inf = infer(
            &s,
            Point2::new(23.3, 41.6),
            &InferInput {
                radius: 0.5,
                grid: Some(5.0),
                ..InferInput::default()
            },
        );
        assert!(inf.grid);
        assert_eq!(inf.point, Point2::new(25.0, 40.0));
    }
}
