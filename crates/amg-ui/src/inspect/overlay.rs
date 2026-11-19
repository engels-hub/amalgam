//! The Measure tool's RND-07 overlay (S05 §2): the witness line with its distance label, the Δ
//! components, the angle arc and fitted circles, plus pick markers and the hover snap.
//!
//! Everything is built per frame into an [`OverlayList`] (immediate mode). Sizes given in
//! pixels are converted with the camera's pixel size at the anchor, so arcs and markers keep
//! their screen size at any zoom.

use amg_core::{Dir3, Point3, Tolerance, Vec3};
use amg_features::inspect::measure::{Circle3, EntityMeasure, MeasureReport, MeasureRow};
use amg_render::{DepthMode, LabelStyle, LineStyle, OverlayList, PointStyle};

use crate::input::CameraView;

/// Accent colour of the witness line and markers (linear RGBA).
pub const ACCENT: [f32; 4] = [0.95, 0.55, 0.05, 1.0];
/// Δ X, Y, Z leg colours (linear RGBA).
pub const AXIS: [[f32; 4]; 3] = [
    [0.80, 0.10, 0.10, 1.0],
    [0.10, 0.65, 0.15, 1.0],
    [0.10, 0.25, 0.90, 1.0],
];
/// Radius of the angle arc, physical pixels.
pub const ARC_PX: f64 = 48.0;
/// Segments of an angle arc.
const ARC_SEGMENTS: usize = 32;
/// Segments of a fitted circle.
const CIRCLE_SEGMENTS: usize = 96;

/// What one measurement draws (kept for "Pin").
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MeasureOverlay {
    /// Witness points `a → b` and the distance text.
    pub witness: Option<(Point3, Point3, String)>,
    /// Δ components as text (x, y, z), drawn as axis legs from `a`.
    pub delta: Option<[String; 3]>,
    /// Angle arc: apex, the two unit directions and the text.
    pub angle: Option<(Point3, Vec3, Vec3, String)>,
    /// Circles (fits or 3-point) with their diameter text.
    pub circles: Vec<(Circle3, String)>,
    /// Pick markers (snapped points).
    pub markers: Vec<Point3>,
    /// Picked edges and chains (world polylines).
    pub chains: Vec<Vec<Point3>>,
}

fn line(color: [f32; 4], width_px: f32, dash: Option<(f32, f32)>) -> LineStyle {
    LineStyle {
        width_px,
        color,
        dash,
        depth: DepthMode::AlwaysOnTop,
    }
}

fn label_style(offset_px: [f32; 2]) -> LabelStyle {
    LabelStyle {
        size_pt: 13.0,
        color: [1.0, 1.0, 1.0, 1.0],
        offset_px,
        background: Some([0.02, 0.02, 0.025, 0.85]),
    }
}

/// The text of the row `key`, if present.
fn row<'a>(rows: &'a [MeasureRow], key: &str) -> Option<&'a str> {
    rows.iter().find(|r| r.key == key).map(|r| r.text.as_str())
}

/// The direction an entity contributes to an angle: a straight edge's direction, else its
/// plane normal.
fn direction(e: &EntityMeasure) -> Option<Vec3> {
    e.direction
        .map(Dir3::get)
        .or(e.plane.map(|p| p.plane.normal.get()))
}

impl MeasureOverlay {
    /// The overlay of `report`, with labels from its displayed `rows`.
    pub fn of_report(report: &MeasureReport, rows: &[MeasureRow]) -> MeasureOverlay {
        let mut o = MeasureOverlay::default();
        if let Some(p) = &report.pair {
            let (a, b) = (p.min_distance.a.point, p.min_distance.b.point);
            let text = row(rows, "distance").unwrap_or_default().to_owned();
            o.witness = Some((a, b, text));
            if (b - a).length() > 0.0 {
                o.delta = Some(["dx", "dy", "dz"].map(|k| row(rows, k).unwrap_or("").to_owned()));
            }
            if let (Some(_), Some(text)) = (p.angle, row(rows, "angle")) {
                let s = report.second.as_ref();
                if let (Some(u), Some(v)) = (direction(&report.first), s.and_then(direction)) {
                    // Apex: the midpoint of the witness points.
                    let apex = a.lerp(b, 0.5);
                    o.angle = Some((apex, u, v, text.to_owned()));
                }
            }
        }
        let diameter = |n: &str| row(rows, n).map(|t| format!("⌀ {t}"));
        for (e, key) in [
            (Some(&report.first), "first.diameter"),
            (report.second.as_ref(), "second.diameter"),
        ] {
            if let Some(c) = e.and_then(|e| e.circle) {
                o.circles
                    .push((c.circle, diameter(key).unwrap_or_default()));
            }
        }
        o
    }

    /// Nothing to draw.
    pub fn is_empty(&self) -> bool {
        *self == MeasureOverlay::default()
    }

    /// Draws into `out` for `view`.
    pub fn draw(&self, out: &mut OverlayList, view: &CameraView) {
        let px = |p: Point3| view.camera.pixel_size_at(p, view.size_px);
        for c in &self.chains {
            out.polyline(c, line(ACCENT, 3.0, None), None);
        }
        if !self.markers.is_empty() {
            out.points(
                &self.markers,
                PointStyle {
                    size_px: 8.0,
                    color: ACCENT,
                    depth: DepthMode::AlwaysOnTop,
                },
                None,
            );
        }
        if let Some((a, b, text)) = &self.witness {
            out.polyline(&[*a, *b], line(ACCENT, 2.0, None), None);
            out.points(
                &[*a, *b],
                PointStyle {
                    size_px: 7.0,
                    color: ACCENT,
                    depth: DepthMode::AlwaysOnTop,
                },
                None,
            );
            out.label(a.lerp(*b, 0.5), text.clone(), label_style([0.0, -14.0]));
            if let Some(d) = &self.delta {
                // Legs a → +ΔX → +ΔY → b, each in its axis colour.
                let delta = *b - *a;
                let mut from = *a;
                for (k, text) in d.iter().enumerate() {
                    let mut step = Vec3::ZERO;
                    step[k] = delta[k];
                    if step[k].abs() <= Tolerance::mesh().linear {
                        continue;
                    }
                    let to = from + step;
                    out.polyline(&[from, to], line(AXIS[k], 1.5, Some((6.0, 4.0))), None);
                    let name = ["ΔX", "ΔY", "ΔZ"][k];
                    let mut style = label_style([0.0, 12.0]);
                    style.size_pt = 11.0;
                    style.color = [1.0, 1.0, 1.0, 0.9];
                    out.label(from.lerp(to, 0.5), format!("{name} {text}"), style);
                    from = to;
                }
            }
        }
        if let Some((apex, u, v, text)) = &self.angle {
            let r = ARC_PX * px(*apex);
            let pts = arc(*apex, *u, *v, r);
            if pts.len() >= 2 {
                out.polyline(&[*apex, pts[0]], line(ACCENT, 1.5, Some((4.0, 3.0))), None);
                out.polyline(
                    &[*apex, pts[pts.len() - 1]],
                    line(ACCENT, 1.5, Some((4.0, 3.0))),
                    None,
                );
                out.polyline(&pts, line(ACCENT, 2.0, None), None);
                out.label(pts[pts.len() / 2], text.clone(), label_style([0.0, -12.0]));
            }
        }
        for (c, text) in &self.circles {
            let pts = circle_points(c);
            out.polyline(&pts, line(ACCENT, 2.0, None), None);
            out.points(
                &[c.center],
                PointStyle {
                    size_px: 6.0,
                    color: ACCENT,
                    depth: DepthMode::AlwaysOnTop,
                },
                None,
            );
            out.label(
                c.center + c.normal.any_orthogonal().get() * c.radius,
                text.clone(),
                label_style([0.0, -14.0]),
            );
        }
    }
}

/// The arc of radius `r` around `apex` from direction `u` to `v` (the smaller angle).
pub fn arc(apex: Point3, u: Vec3, v: Vec3, r: f64) -> Vec<Point3> {
    let (Some(u), Some(v)) = (u.try_normalize(), v.try_normalize()) else {
        return Vec::new();
    };
    let theta = u.dot(v).clamp(-1.0, 1.0).acos();
    // In-plane unit vector ⟂ u towards v (any ⟂ when u ∥ v).
    let w = (v - u * u.dot(v))
        .try_normalize()
        .unwrap_or_else(|| Dir3::new_unchecked(u).any_orthogonal().get());
    (0..=ARC_SEGMENTS)
        .map(|i| {
            #[allow(clippy::cast_precision_loss, reason = "small segment counts")]
            let t = theta * (i as f64 / ARC_SEGMENTS as f64);
            apex + (u * t.cos() + w * t.sin()) * r
        })
        .collect()
}

/// A closed polyline on `c`.
pub fn circle_points(c: &Circle3) -> Vec<Point3> {
    let n = c.normal.get();
    let x = c.normal.any_orthogonal().get();
    let y = n.cross(x);
    (0..=CIRCLE_SEGMENTS)
        .map(|i| {
            #[allow(clippy::cast_precision_loss, reason = "small segment counts")]
            let t = std::f64::consts::TAU * (i as f64 / CIRCLE_SEGMENTS as f64);
            c.center + (x * t.cos() + y * t.sin()) * c.radius
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arcs_span_the_angle_at_the_radius() {
        let pts = arc(Point3::ZERO, Vec3::X, Vec3::Y, 2.0);
        assert_eq!(pts.len(), ARC_SEGMENTS + 1);
        assert!((pts[0] - Point3::new(2.0, 0.0, 0.0)).length() < 1e-12);
        assert!((pts[ARC_SEGMENTS] - Point3::new(0.0, 2.0, 0.0)).length() < 1e-12);
        assert!(pts.iter().all(|p| (p.length() - 2.0).abs() < 1e-12));
        // Antiparallel directions still give a half circle.
        let half = arc(Point3::ZERO, Vec3::X, -Vec3::X, 1.0);
        assert!((half[ARC_SEGMENTS] + Point3::X).length() < 1e-12);
    }

    #[test]
    fn circles_close_on_their_radius() {
        let c = Circle3 {
            center: Point3::new(1.0, 2.0, 3.0),
            normal: Dir3::Z,
            radius: 5.0,
        };
        let pts = circle_points(&c);
        assert!((pts[0] - pts[CIRCLE_SEGMENTS]).length() < 1e-9);
        assert!(
            pts.iter()
                .all(|p| ((*p - c.center).length() - 5.0).abs() < 1e-9 && (p.z - 3.0).abs() < 1e-12)
        );
    }
}
