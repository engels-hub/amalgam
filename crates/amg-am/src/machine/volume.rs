//! Bed shapes and the build volume.

use amg_core::Aabb3;
use amg_core::glam::{DVec2, DVec3};
use serde::{Deserialize, Serialize};

/// Bed outline in the plate frame.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "shape", rename_all = "snake_case")]
pub enum BedShape {
    /// Rectangle from the origin to `size`.
    Rect {
        /// Width (x) and depth (y), mm.
        size: DVec2,
    },
    /// Counter-clockwise polygon, may be non-convex.
    Polygon {
        /// The corners.
        points: Vec<DVec2>,
    },
    /// Circle centred at `(d/2, d/2)`.
    Circle {
        /// Diameter, mm.
        diameter: f64,
    },
}

/// Number of edges used to approximate a circular bed.
const CIRCLE_SEGMENTS: usize = 128;

/// The printable region: bed outline, height, keep-out areas and slicer origin.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BuildVolume {
    /// Bed outline.
    pub bed: BedShape,
    /// Printable height, mm.
    pub height: f64,
    /// Keep-out polygons on the bed (Orca `bed_exclude_area`).
    #[serde(default)]
    pub exclude: Vec<Vec<DVec2>>,
    /// Plate-frame position of the slicer origin (0, 0).
    pub slicer_origin: DVec2,
}

fn point_in_polygon(p: DVec2, poly: &[DVec2]) -> bool {
    let mut inside = false;
    let mut j = poly.len() - 1;
    for i in 0..poly.len() {
        let (a, b) = (poly[i], poly[j]);
        if (a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x {
            inside = !inside;
        }
        j = i;
    }
    inside
}

fn dist_to_boundary(p: DVec2, poly: &[DVec2]) -> f64 {
    let mut best = f64::INFINITY;
    let mut j = poly.len() - 1;
    for i in 0..poly.len() {
        let (a, b) = (poly[j], poly[i]);
        let ab = b - a;
        let len2 = ab.length_squared();
        let t = if len2 > 0.0 {
            ((p - a).dot(ab) / len2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        best = best.min((p - (a + ab * t)).length());
        j = i;
    }
    best
}

fn signed_area(poly: &[DVec2]) -> f64 {
    let mut s = 0.0;
    let mut j = poly.len() - 1;
    for i in 0..poly.len() {
        s += poly[j].x * poly[i].y - poly[i].x * poly[j].y;
        j = i;
    }
    s * 0.5
}

impl BuildVolume {
    /// Bed outline as a polygon (a circle becomes a 128-gon).
    pub fn bed_polygon(&self) -> Vec<DVec2> {
        match &self.bed {
            BedShape::Rect { size } => vec![
                DVec2::ZERO,
                DVec2::new(size.x, 0.0),
                *size,
                DVec2::new(0.0, size.y),
            ],
            BedShape::Polygon { points } => points.clone(),
            BedShape::Circle { diameter } => {
                let r = diameter * 0.5;
                (0..CIRCLE_SEGMENTS)
                    .map(|i| {
                        let t = std::f64::consts::TAU * i as f64 / CIRCLE_SEGMENTS as f64;
                        DVec2::new(r + r * t.cos(), r + r * t.sin())
                    })
                    .collect()
            }
        }
    }

    /// Bounding rectangle (min, max) of the bed.
    pub fn bed_rect(&self) -> (DVec2, DVec2) {
        match &self.bed {
            BedShape::Rect { size } => (DVec2::ZERO, *size),
            BedShape::Circle { diameter } => (DVec2::ZERO, DVec2::splat(*diameter)),
            BedShape::Polygon { points } => points
                .iter()
                .fold((DVec2::INFINITY, DVec2::NEG_INFINITY), |(lo, hi), p| {
                    (lo.min(*p), hi.max(*p))
                }),
        }
    }

    /// Centre of the bed's bounding rectangle.
    pub fn centre(&self) -> DVec2 {
        let (lo, hi) = self.bed_rect();
        (lo + hi) * 0.5
    }

    /// The bed's bounding box over the full height.
    pub fn aabb(&self) -> Aabb3 {
        let (lo, hi) = self.bed_rect();
        Aabb3 {
            min: DVec3::new(lo.x, lo.y, 0.0),
            max: DVec3::new(hi.x, hi.y, self.height),
        }
    }

    /// True if `p` is on the bed (within `tol` of its outline counts as on) and not more than
    /// `tol` inside any exclude polygon.
    pub fn contains_xy(&self, p: DVec2, tol: f64) -> bool {
        let on_bed = match &self.bed {
            BedShape::Rect { size } => {
                p.x >= -tol && p.y >= -tol && p.x <= size.x + tol && p.y <= size.y + tol
            }
            BedShape::Circle { diameter } => {
                (p - DVec2::splat(diameter * 0.5)).length() <= diameter * 0.5 + tol
            }
            BedShape::Polygon { points } => {
                !points.is_empty()
                    && (point_in_polygon(p, points) || dist_to_boundary(p, points) <= tol)
            }
        };
        on_bed
            && !self
                .exclude
                .iter()
                .filter(|e| !e.is_empty())
                .any(|e| point_in_polygon(p, e) && dist_to_boundary(p, e) > tol)
    }

    /// Validation used by `Machine::validate`; reasons name the field.
    pub(crate) fn validate(&self) -> Result<(), String> {
        let finite2 = |p: &DVec2| p.x.is_finite() && p.y.is_finite();
        match &self.bed {
            BedShape::Rect { size } => {
                if !(finite2(size) && size.x > 0.0 && size.y > 0.0) {
                    return Err(format!(
                        "`volume.bed.size` must be positive, got [{}, {}]",
                        size.x, size.y
                    ));
                }
            }
            BedShape::Circle { diameter } => {
                if !(diameter.is_finite() && *diameter > 0.0) {
                    return Err(format!(
                        "`volume.bed.diameter` must be positive, got {diameter}"
                    ));
                }
            }
            BedShape::Polygon { points } => {
                if points.len() < 3 {
                    return Err("`volume.bed.points` needs at least 3 points".into());
                }
                if !points.iter().all(finite2) {
                    return Err("`volume.bed.points` must be finite".into());
                }
                if signed_area(points).abs() <= 0.0 {
                    return Err("`volume.bed.points` enclose no area".into());
                }
            }
        }
        if !(self.height.is_finite() && self.height > 0.0) {
            return Err(format!(
                "`volume.height` must be positive, got {}",
                self.height
            ));
        }
        for (i, e) in self.exclude.iter().enumerate() {
            if e.len() < 3 || !e.iter().all(finite2) {
                return Err(format!(
                    "`volume.exclude[{i}]` needs at least 3 finite points"
                ));
            }
        }
        if !finite2(&self.slicer_origin) {
            return Err("`volume.slicer_origin` must be finite".into());
        }
        Ok(())
    }
}
