//! Constraint glyphs and dimension labels as screen-space targets (spec §13 *Rendering*,
//! UI-14).
//!
//! - [`layout`] places one glyph per constrained entity: an icon from the UI-23 registry,
//!   offset from the entity (curves at their middle, points at the point) and stacked when an
//!   entity has several. Coincident and point-on-curve glyphs only appear for hovered or
//!   selected entities (they would sit on every corner).
//! - [`glyph_hit`] and [`dimension_hit`] find the glyph or dimension label under the cursor;
//!   the select tool uses them for hover (the constrained entities light up), click-select,
//!   Delete, double-click edit and label drag.
//! - [`GlyphPainter`] paints the glyphs over the viewport every frame (an [`AppDialog`] that
//!   lives as long as its session), with the hovered and selected ones highlighted.

use std::borrow::Cow;
use std::collections::BTreeSet;

use amg_core::{Plane, Point2};
use amg_sketch::{ConstraintId, ConstraintKind, Dimension, EntityId, HvTarget, Sketch};

use super::session::with_session;
use super::view;
use crate::app::AppCx;
use crate::dialog::AppDialog;
use crate::icons::ids;
use crate::input::CameraView;
use crate::module::IconId;

/// Glyph size, points.
pub const GLYPH_PT: f32 = 16.0;
/// Distance of the first glyph from its entity, points.
const OFFSET_PT: f32 = 16.0;
/// Spacing of stacked glyphs, points.
const STEP_PT: f32 = 19.0;

/// One glyph of a constraint, next to one of its entities.
#[derive(Clone, Debug, PartialEq)]
pub struct Glyph {
    /// The constraint it stands for.
    pub constraint: ConstraintId,
    /// The entity it is drawn next to.
    pub entity: EntityId,
    /// The icon (UI-23 registry).
    pub icon: IconId,
    /// Text fallback and accessible name ("H", "//", ...).
    pub text: &'static str,
    /// The entity's anchor in sketch coordinates.
    pub anchor: Point2,
    /// Screen offset from the anchor, points.
    pub offset: egui::Vec2,
}

impl Glyph {
    /// Where the glyph's centre appears in `view` (`None` behind the camera).
    pub fn screen(&self, view: &CameraView, plane: &Plane) -> Option<egui::Pos2> {
        Some(view.project(plane.to_world(self.anchor))? + self.offset)
    }
}

/// The icon and text of a constraint glyph; `None` for dimensions.
pub fn glyph_icon(k: &ConstraintKind) -> Option<(IconId, &'static str)> {
    Some(match k {
        ConstraintKind::Coincident { .. } => (ids::COINCIDENT, "Coi"),
        ConstraintKind::PointOnCurve { .. } => (ids::COINCIDENT, "On"),
        ConstraintKind::Horizontal(_) => (ids::HORIZONTAL, "H"),
        ConstraintKind::Vertical(_) => (ids::VERTICAL, "V"),
        ConstraintKind::Parallel { .. } => (ids::PARALLEL, "//"),
        ConstraintKind::Perpendicular { .. } => (ids::PERPENDICULAR, "_|_"),
        ConstraintKind::Tangent { .. } => (ids::TANGENT, "T"),
        ConstraintKind::Curvature { .. } => (ids::SMOOTH, "G2"),
        ConstraintKind::Equal { .. } => (ids::EQUAL, "="),
        ConstraintKind::Midpoint { .. } => (ids::MIDPOINT, "Mid"),
        ConstraintKind::Fix { .. } => (ids::FIX, "Fix"),
        ConstraintKind::Symmetric { .. } => (ids::SYMMETRIC, "Sym"),
        ConstraintKind::Concentric { .. } => (ids::CONCENTRIC, "(o)"),
        ConstraintKind::Collinear { .. } => (ids::COLLINEAR, "Col"),
        ConstraintKind::Dimension(_) => return None,
    })
}

/// The entities a constraint's glyphs are drawn next to.
fn marked(k: &ConstraintKind) -> Vec<EntityId> {
    match *k {
        ConstraintKind::Horizontal(HvTarget::Line(l))
        | ConstraintKind::Vertical(HvTarget::Line(l)) => {
            vec![l]
        }
        ConstraintKind::Horizontal(HvTarget::Points(a, _))
        | ConstraintKind::Vertical(HvTarget::Points(a, _)) => vec![a],
        ConstraintKind::Midpoint { point, .. } => vec![point],
        ConstraintKind::PointOnCurve { point, .. } => vec![point],
        ConstraintKind::Coincident { a, .. } => vec![a],
        ConstraintKind::Fix { entity } => vec![entity],
        ConstraintKind::Symmetric { a, b, .. } => vec![a, b],
        ConstraintKind::Parallel { a, b }
        | ConstraintKind::Perpendicular { a, b }
        | ConstraintKind::Collinear { a, b }
        | ConstraintKind::Tangent { a, b, .. }
        | ConstraintKind::Curvature { a, b, .. }
        | ConstraintKind::Concentric { a, b }
        | ConstraintKind::Equal { a, b } => vec![a, b],
        ConstraintKind::Dimension(_) => Vec::new(),
    }
}

/// A point that represents entity `e`: the point itself, or the middle of a curve.
pub fn anchor_of(sketch: &Sketch, e: EntityId) -> Option<Point2> {
    if let Some(p) = sketch.point(e) {
        return Some(p);
    }
    let c = sketch.curve(e)?;
    let (t0, t1) = c.param_range();
    Some(c.eval(0.5 * (t0 + t1)))
}

/// The glyphs of `sketch`. Coincident and point-on-curve glyphs are only laid out when one of
/// their entities is in `reveal` (hovered or selected geometry).
pub fn layout(sketch: &Sketch, reveal: &BTreeSet<EntityId>) -> Vec<Glyph> {
    let mut slots: std::collections::BTreeMap<EntityId, u16> = std::collections::BTreeMap::new();
    let mut out = Vec::new();
    for (id, c) in sketch.constraints() {
        let Some((icon, text)) = glyph_icon(&c.kind) else {
            continue;
        };
        if matches!(
            c.kind,
            ConstraintKind::Coincident { .. } | ConstraintKind::PointOnCurve { .. }
        ) && !c.kind.entities().iter().any(|e| reveal.contains(e))
        {
            continue;
        }
        for e in marked(&c.kind) {
            let Some(anchor) = anchor_of(sketch, e) else {
                continue;
            };
            let k = slots.entry(e).or_insert(0);
            let offset = egui::vec2(OFFSET_PT + STEP_PT * f32::from(*k), -OFFSET_PT);
            *k += 1;
            out.push(Glyph {
                constraint: id,
                entity: e,
                icon,
                text,
                anchor,
                offset,
            });
        }
    }
    out
}

/// The glyph under screen position `pos` (the nearest within its square).
pub fn glyph_hit<'g>(
    glyphs: &'g [Glyph],
    view: &CameraView,
    plane: &Plane,
    pos: egui::Pos2,
) -> Option<&'g Glyph> {
    let half = GLYPH_PT * 0.5 + 1.0;
    glyphs
        .iter()
        .filter_map(|g| {
            let c = g.screen(view, plane)?;
            let d = c - pos;
            (d.x.abs() <= half && d.y.abs() <= half).then_some((d.length_sq(), g))
        })
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .map(|(_, g)| g)
}

/// The screen rectangle of a dimension label showing `text` (labels are centred on their
/// anchor, see `amg_render::egui_glue::paint_labels`; the size is estimated from the text).
pub fn label_rect(view: &CameraView, plane: &Plane, at: Point2, text: &str) -> Option<egui::Rect> {
    let c = view.project(plane.to_world(at))?;
    #[allow(clippy::cast_precision_loss, reason = "short label texts")]
    let w = 6.8 * text.chars().count() as f32 + 10.0;
    Some(egui::Rect::from_center_size(c, egui::vec2(w, 20.0)))
}

/// Where dimension `d` (constraint `id`) shows its label: the dragged position while its label
/// is being dragged, else `label_pos`.
pub fn label_pos(d: &Dimension, id: ConstraintId, drag: Option<(ConstraintId, Point2)>) -> Point2 {
    match drag {
        Some((c, p)) if c == id => p,
        _ => d.label_pos,
    }
}

/// The dimension whose label is under screen position `pos`.
pub fn dimension_hit(
    sketch: &Sketch,
    view: &CameraView,
    plane: &Plane,
    pos: egui::Pos2,
    drag: Option<(ConstraintId, Point2)>,
) -> Option<ConstraintId> {
    sketch.constraints().find_map(|(id, c)| {
        let d = c.kind.as_dimension()?;
        let text = view::dimension_text(&d.kind, d.value);
        label_rect(view, plane, label_pos(d, id, drag), &text)?
            .contains(pos)
            .then_some(id)
    })
}

/// Paints the open session's constraint glyphs over the viewport (opened with the session,
/// closed when it ends; draws nothing while "Show constraints" is off).
#[derive(Debug, Default)]
pub struct GlyphPainter {
    /// The session it belongs to (`SketchSession::serial`).
    pub session: u32,
}

/// Linear RGBA → egui colour.
fn color32(c: [f32; 4]) -> egui::Color32 {
    let [r, g, b, a] = c.map(|v| (v.clamp(0.0, 1.0) * 255.0).round() as u8);
    egui::Color32::from_rgba_unmultiplied(r, g, b, a)
}

impl AppDialog for GlyphPainter {
    fn title(&self) -> Cow<'static, str> {
        "Sketch glyphs".into()
    }

    fn show(&mut self, ctx: &egui::Context, cx: &mut AppCx<'_>) -> bool {
        let view = cx.doc_tab().view.clone();
        let got = with_session(cx, |s, _| {
            if s.serial() != self.session {
                return None;
            }
            if !s.palette.show_constraints {
                return Some(Vec::new());
            }
            let reveal: BTreeSet<EntityId> = s.selected.iter().chain(&s.hovered).copied().collect();
            let hot: BTreeSet<ConstraintId> = s
                .hovered_constraint
                .iter()
                .chain(&s.selected_constraints)
                .copied()
                .collect();
            let plane = *s.plane();
            let glyphs = layout(s.display(), &reveal);
            Some(
                glyphs
                    .into_iter()
                    .map(|g| (hot.contains(&g.constraint), g, plane))
                    .collect::<Vec<_>>(),
            )
        })
        .flatten();
        let Some(glyphs) = got else {
            return false;
        };
        let Some(view) = view else {
            return true;
        };
        let painter = ctx
            .layer_painter(egui::LayerId::new(
                egui::Order::Middle,
                egui::Id::new("amg_sketch_glyphs"),
            ))
            .with_clip_rect(view.rect);
        for (hot, g, plane) in glyphs {
            let Some(c) = g.screen(&view, &plane) else {
                continue;
            };
            let rect = egui::Rect::from_center_size(c, egui::vec2(GLYPH_PT, GLYPH_PT));
            let (bg, fg) = if hot {
                (view::colors::HOVER, view::colors::FULL)
            } else {
                (view::colors::LABEL_BG, view::colors::INFER)
            };
            painter.rect_filled(rect.expand(1.5), 3.0, color32(bg));
            if !crate::icons::paint(ctx, &painter, rect.shrink(1.0), g.icon, color32(fg)) {
                painter.text(
                    c,
                    egui::Align2::CENTER_CENTER,
                    g.text,
                    egui::FontId::proportional(10.0),
                    color32(fg),
                );
            }
        }
        true
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn glyphs_stack_per_entity_and_hide_coincidence() {
        let mut s = Sketch::new(Plane::XY);
        let a = s
            .add_line(Point2::new(0.0, 0.0), Point2::new(10.0, 0.0))
            .unwrap();
        let b = s
            .add_line(Point2::new(0.0, 5.0), Point2::new(10.0, 6.0))
            .unwrap();
        s.add_constraint(ConstraintKind::Horizontal(HvTarget::Line(a)))
            .unwrap();
        s.add_constraint(ConstraintKind::Parallel { a, b }).unwrap();
        let none = BTreeSet::new();
        let g = layout(&s, &none);
        assert_eq!(g.len(), 3, "H on a, // on a and b");
        let on_a: Vec<_> = g.iter().filter(|g| g.entity == a).collect();
        assert_eq!(on_a.len(), 2);
        assert!(on_a[1].offset.x > on_a[0].offset.x, "stacked");
        let amg_sketch::EntityKind::Line { start, end } = s.entity(a).unwrap().kind else {
            unreachable!()
        };
        let _ = end;
        let p = s.add_point(Point2::new(3.0, 3.0)).unwrap();
        s.add_constraint(ConstraintKind::Coincident { a: start, b: p })
            .unwrap();
        assert_eq!(layout(&s, &none).len(), 3, "coincidence hidden");
        assert_eq!(
            layout(&s, &BTreeSet::from([p])).len(),
            4,
            "shown for hovered points"
        );
    }
}
