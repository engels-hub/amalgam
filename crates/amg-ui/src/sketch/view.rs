//! Sketch overlays (spec §13 *Rendering*) and the screen ↔ sketch-plane mapping.
//!
//! Everything is drawn as RND-07 overlays, always on top of the model while the session is
//! open: curves (construction dashed), points, shaded profiles, constraint glyphs and
//! dimension labels, coloured by the DOF report (`Under` blue, `Full` foreground, `Over` red;
//! selection and hover override).

use amg_core::{Plane, Point2, Point3, Ray3};
use amg_render::{DepthMode, LabelStyle, LineStyle, OverlayList, PointStyle};
use amg_sketch::solve::ConstraintState;
use amg_sketch::{ConstraintKind, Curve2, DimensionKind, EntityId, EntityKind, Quantity, Sketch};

use super::session::SketchSession;
use crate::input::CameraView;

/// Linear RGBA colours of the sketch overlays.
pub mod colors {
    /// Under-constrained geometry (blue).
    pub const UNDER: [f32; 4] = [0.05, 0.30, 0.95, 1.0];
    /// Fully constrained geometry (foreground).
    pub const FULL: [f32; 4] = [0.02, 0.02, 0.025, 1.0];
    /// Over-constrained geometry (red).
    pub const OVER: [f32; 4] = [0.90, 0.06, 0.05, 1.0];
    /// Selected geometry.
    pub const SELECTED: [f32; 4] = [0.0, 0.70, 1.0, 1.0];
    /// Geometry under the cursor.
    pub const HOVER: [f32; 4] = [0.45, 0.85, 1.0, 1.0];
    /// Construction geometry.
    pub const CONSTRUCTION: [f32; 4] = [0.95, 0.45, 0.05, 1.0];
    /// Profile shading.
    pub const PROFILE: [f32; 4] = [1.0, 0.72, 0.25, 0.28];
    /// Grid lines.
    pub const GRID: [f32; 4] = [0.35, 0.37, 0.40, 0.55];
    /// Inference glyphs and alignment guides.
    pub const INFER: [f32; 4] = [0.05, 0.65, 0.25, 1.0];
    /// Rubber band / preview geometry.
    pub const PREVIEW: [f32; 4] = [0.05, 0.30, 0.95, 0.85];
    /// Label background.
    pub const LABEL_BG: [f32; 4] = [1.0, 1.0, 1.0, 0.85];
}

/// Line width of sketch curves, px.
pub const CURVE_PX: f32 = 2.0;
/// Diameter of sketch points, px.
pub const POINT_PX: f32 = 7.0;
/// Segments per full turn when curves are sampled.
const SEGMENTS_PER_TURN: f64 = 96.0;

/// Where `ray` meets `plane`, in plane coordinates (`None` when it runs parallel to it).
pub fn ray_to_plane(plane: &Plane, ray: &Ray3) -> Option<Point2> {
    let n = plane.normal.get();
    let d = ray.dir.get();
    let den = d.dot(n);
    if den.abs() < 1e-12 {
        return None;
    }
    let t = (plane.origin - ray.origin).dot(n) / den;
    Some(plane.to_local(ray.origin + d * t))
}

/// Millimetres per screen point at sketch point `p` (1 before the first viewport frame).
pub fn mm_per_point(view: Option<&CameraView>, plane: &Plane, p: Point2) -> f64 {
    view.map_or(1.0, |v| v.world_per_point(plane.to_world(p)))
}

/// Samples `c` into a polyline (sketch coordinates).
pub fn sample(c: &Curve2) -> Vec<Point2> {
    let (t0, t1) = c.param_range();
    let n = match c {
        Curve2::Line(_) => 1,
        Curve2::Circle(_) => SEGMENTS_PER_TURN as usize,
        Curve2::Arc(a) => ((a.sweep.abs() / std::f64::consts::TAU) * SEGMENTS_PER_TURN)
            .ceil()
            .max(4.0) as usize,
        Curve2::Ellipse(_) | Curve2::BSpline(_) => SEGMENTS_PER_TURN as usize,
    };
    (0..=n)
        .map(|i| c.eval(t0 + (t1 - t0) * i as f64 / n as f64))
        .collect()
}

/// Ear-clipping triangulation of a simple polygon (either orientation).
pub fn triangulate(poly: &[Point2]) -> Vec<[u32; 3]> {
    let n = poly.len();
    if n < 3 {
        return Vec::new();
    }
    let area: f64 = (0..n)
        .map(|i| {
            let (a, b) = (poly[i], poly[(i + 1) % n]);
            a.x * b.y - b.x * a.y
        })
        .sum();
    let ccw = area >= 0.0;
    let mut idx: Vec<usize> = (0..n).collect();
    let cross = |o: Point2, a: Point2, b: Point2| (a - o).perp_dot(b - o);
    let mut out = Vec::with_capacity(n - 2);
    let mut guard = 0usize;
    while idx.len() > 3 && guard < n * n {
        guard += 1;
        let m = idx.len();
        let mut clipped = false;
        for i in 0..m {
            let (ia, ib, ic) = (idx[(i + m - 1) % m], idx[i], idx[(i + 1) % m]);
            let (a, b, c) = (poly[ia], poly[ib], poly[ic]);
            let turn = cross(a, b, c);
            if (ccw && turn <= 1e-12) || (!ccw && turn >= -1e-12) {
                continue;
            }
            let inside = idx.iter().any(|&j| {
                if j == ia || j == ib || j == ic {
                    return false;
                }
                let p = poly[j];
                let (d1, d2, d3) = (cross(a, b, p), cross(b, c, p), cross(c, a, p));
                if ccw {
                    d1 >= 0.0 && d2 >= 0.0 && d3 >= 0.0
                } else {
                    d1 <= 0.0 && d2 <= 0.0 && d3 <= 0.0
                }
            });
            if inside {
                continue;
            }
            out.push([ia as u32, ib as u32, ic as u32]);
            idx.remove(i);
            clipped = true;
            break;
        }
        if !clipped {
            break;
        }
    }
    if idx.len() == 3 {
        out.push([idx[0] as u32, idx[1] as u32, idx[2] as u32]);
    }
    out
}

fn line_style(color: [f32; 4], construction: bool) -> LineStyle {
    LineStyle {
        width_px: CURVE_PX,
        color,
        dash: construction.then_some((6.0, 4.0)),
        depth: DepthMode::AlwaysOnTop,
    }
}

/// Colour of entity `e` in `s`.
fn entity_color(s: &SketchSession, e: EntityId, construction: bool) -> [f32; 4] {
    if s.selected.contains(&e) {
        return colors::SELECTED;
    }
    if s.hovered == Some(e) {
        return colors::HOVER;
    }
    match s.entity_state(e) {
        Some(ConstraintState::Over) => colors::OVER,
        Some(ConstraintState::Full) if !construction => colors::FULL,
        _ if construction => colors::CONSTRUCTION,
        _ => colors::UNDER,
    }
}

/// Draws a polyline in sketch coordinates.
pub fn polyline(out: &mut OverlayList, plane: &Plane, pts: &[Point2], style: LineStyle) {
    let w: Vec<Point3> = pts.iter().map(|p| plane.to_world(*p)).collect();
    out.polyline(&w, style, None);
}

/// A label at sketch point `p`.
pub fn label(out: &mut OverlayList, plane: &Plane, p: Point2, text: String, color: [f32; 4]) {
    out.label(
        plane.to_world(p),
        text,
        LabelStyle {
            size_pt: 12.0,
            color,
            offset_px: [0.0, 0.0],
            background: Some(colors::LABEL_BG),
        },
    );
}

/// The glyph of a constraint (`None`: not drawn, e.g. coincidence).
pub fn constraint_glyph(k: &ConstraintKind) -> Option<&'static str> {
    Some(match k {
        ConstraintKind::Horizontal(_) => "H",
        ConstraintKind::Vertical(_) => "V",
        ConstraintKind::Parallel { .. } => "//",
        ConstraintKind::Perpendicular { .. } => "_|_",
        ConstraintKind::Tangent { .. } => "T",
        ConstraintKind::Equal { .. } => "=",
        ConstraintKind::Midpoint { .. } => "Mid",
        ConstraintKind::Fix { .. } => "Fix",
        ConstraintKind::Symmetric { .. } => "Sym",
        ConstraintKind::Concentric { .. } => "(o)",
        ConstraintKind::Collinear { .. } => "Col",
        _ => return None,
    })
}

/// A point that represents entity `e` (the middle of a curve, the point itself).
fn anchor_of(sketch: &Sketch, e: EntityId) -> Option<Point2> {
    if let Some(p) = sketch.point(e) {
        return Some(p);
    }
    let c = sketch.curve(e)?;
    let (t0, t1) = c.param_range();
    Some(c.eval(0.5 * (t0 + t1)))
}

/// Text of a dimension value.
pub fn dimension_text(kind: &DimensionKind, value: f64) -> String {
    match kind.quantity() {
        Quantity::Length => format!("{value:.2} mm"),
        Quantity::Angle => format!("{:.1}°", value.to_degrees()),
    }
}

/// Draws the session's sketch: grid, profiles, curves, points, glyphs and dimensions.
pub fn draw_session(s: &SketchSession, out: &mut OverlayList, view: Option<&CameraView>) {
    let plane = *s.plane();
    let sketch = s.display().clone();
    let pal = s.palette;
    if pal.grid {
        let ext = view.map_or(500.0, |v| {
            (v.world_per_point(plane.origin) * f64::from(v.rect.width().max(v.rect.height())))
                .max(10.0)
        });
        out.grid(&plane, [ext, ext], colors::GRID);
    }
    if pal.show_profile {
        for p in &s.profiles().profiles {
            let mut poly: Vec<Point2> = Vec::new();
            for e in &p.outer.edges {
                let pts = sample(&e.curve);
                poly.extend_from_slice(&pts[..pts.len().saturating_sub(1)]);
            }
            let tris = triangulate(&poly);
            let w: Vec<Point3> = poly.iter().map(|q| plane.to_world(*q)).collect();
            out.polygon(&w, &tris, colors::PROFILE, DepthMode::AlwaysOnTop, None);
        }
    }
    let mut points = Vec::new();
    let mut point_colors: Vec<([f32; 4], Point3)> = Vec::new();
    for (id, ent) in sketch.entities() {
        if let EntityKind::Point { .. } = ent.kind {
            if pal.show_points
                && let Some(p) = sketch.point(id)
            {
                let c = entity_color(s, id, ent.construction);
                if c == colors::UNDER {
                    points.push(plane.to_world(p));
                } else {
                    point_colors.push((c, plane.to_world(p)));
                }
            }
            continue;
        }
        let Some(curve) = sketch.curve(id) else {
            continue;
        };
        let color = entity_color(s, id, ent.construction);
        polyline(
            out,
            &plane,
            &sample(&curve),
            line_style(color, ent.construction),
        );
    }
    let pstyle = |color| PointStyle {
        size_px: POINT_PX,
        color,
        depth: DepthMode::AlwaysOnTop,
    };
    out.points(&points, pstyle(colors::UNDER), None);
    for (c, p) in point_colors {
        out.points(&[p], pstyle(c), None);
    }
    if pal.show_constraints || pal.show_dimensions {
        for (_, c) in sketch.constraints() {
            match &c.kind {
                ConstraintKind::Dimension(d) => {
                    if pal.show_dimensions {
                        let color = if d.driving {
                            colors::FULL
                        } else {
                            colors::UNDER
                        };
                        label(
                            out,
                            &plane,
                            d.label_pos,
                            dimension_text(&d.kind, d.value),
                            color,
                        );
                    }
                }
                k if pal.show_constraints => {
                    let Some(g) = constraint_glyph(k) else {
                        continue;
                    };
                    let Some(e) = k.entities().first().copied() else {
                        continue;
                    };
                    if let Some(at) = anchor_of(&sketch, e) {
                        let off = mm_per_point(view, &plane, at) * 12.0;
                        label(
                            out,
                            &plane,
                            at + Point2::new(off, off),
                            g.to_owned(),
                            colors::INFER,
                        );
                    }
                }
                _ => {}
            }
        }
    }
}

/// Draws the visible sketches of the design that are not being edited (thin, in the
/// foreground colour; construction dashed; no points or glyphs). Hidden sketches (browser
/// eye, `ui.visibility`) are skipped.
pub fn draw_finished(
    out: &mut OverlayList,
    design: &amg_doc::Design,
    eval: &amg_doc::Evaluated,
    editing: Option<amg_doc::FeatureId>,
) {
    let mut hidden = None;
    for (id, geom) in eval.sketches() {
        if Some(id.0) == editing {
            continue;
        }
        let hidden = hidden.get_or_insert_with(|| crate::browser::visibility::hidden(design));
        if hidden.contains(&crate::browser::visibility::sketch_key(id)) {
            continue;
        }
        let plane = geom.plane;
        for (e, ent) in geom.sketch.entities() {
            if matches!(ent.kind, EntityKind::Point { .. }) {
                continue;
            }
            if let Some(c) = geom.sketch.curve(e) {
                let mut style = line_style(colors::FULL, ent.construction);
                style.width_px = 1.0;
                style.depth = DepthMode::Test;
                polyline(out, &plane, &sample(&c), style);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ear_clipping_covers_a_concave_polygon() {
        // An L shape: area 3.
        let poly = [
            Point2::new(0.0, 0.0),
            Point2::new(2.0, 0.0),
            Point2::new(2.0, 1.0),
            Point2::new(1.0, 1.0),
            Point2::new(1.0, 2.0),
            Point2::new(0.0, 2.0),
        ];
        let tris = triangulate(&poly);
        assert_eq!(tris.len(), 4);
        let area: f64 = tris
            .iter()
            .map(|t| {
                let [a, b, c] = t.map(|i| poly[i as usize]);
                0.5 * (b - a).perp_dot(c - a).abs()
            })
            .sum();
        assert!((area - 3.0).abs() < 1e-12);
    }

    #[test]
    fn rays_meet_the_sketch_plane() {
        let ray = Ray3 {
            origin: Point3::new(3.0, 4.0, 10.0),
            dir: amg_core::Dir3::NEG_Z,
        };
        assert_eq!(ray_to_plane(&Plane::XY, &ray), Some(Point2::new(3.0, 4.0)));
        let flat = Ray3 {
            origin: Point3::new(0.0, 0.0, 1.0),
            dir: amg_core::Dir3::X,
        };
        assert_eq!(ray_to_plane(&Plane::XY, &flat), None);
    }
}
