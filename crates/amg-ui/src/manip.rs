//! Manipulators (spec §10, Interfaces §A `amg_ui::manip`, UI-06).
//!
//! On-canvas gizmos that edit a value by dragging:
//! - [`GizmoPrim::Arrow`] (`Distance`): an arrow along an axis that edits a length.
//! - [`GizmoPrim::Ring`] (`Angle`): a ring about an axis that edits an angle; a pointer shows the
//!   current angle.
//! - [`GizmoPrim::Triad`] (`Triad`): move / rotate with 3 axis arrows, 3 plane squares, 3
//!   rotation arcs and a free centre ([`TriadPart`]).
//! - [`GizmoPrim::PlaneHandle`] (`Plane`): a square handle that moves in its plane (2D).
//!
//! **Constant screen size.** Every gizmo is [`GIZMO_SIZE_PT`] (90 pt) on screen whatever the
//! zoom: its world size is `90 × CameraView::world_per_point(origin)`, computed per frame. The
//! size is the arrow length, the ring diameter, the triad arm length and the plane handle side.
//!
//! **Drawing** goes through RND-07 overlays ([`draw`], into `amg_render::OverlayList`), always on
//! top; the hovered or dragged part is drawn in the hot colour.
//!
//! **Hit tests run on the CPU** against the projected gizmo geometry ([`GizmoShape`]), with a
//! [`HIT_TOLERANCE_PT`] (6 pt) tolerance: no GPU pick readback. The nearest part wins; plane
//! squares and the free centre count as distance 0 inside.
//!
//! **Dragging** ([`GizmoDrag`]) maps the cursor ray to a value change against the geometry at
//! drag start: the closest point on the axis line (arrows, triad axes), the angle in the ring
//! plane (rings, triad arcs), or the point in a plane (plane squares, plane handles, the free
//! centre in the view plane). [`snap`] rounds to the preferences increment while `Ctrl` is held
//! ([`SNAP_LENGTH`] display unit, [`SNAP_ANGLE`]).
//!
//! [`Manipulators`] is the per-tool controller: it takes the tool's gizmos each frame, tracks
//! hover, owns the primary drag that starts on a gizmo and turns viewport events into
//! [`ManipEvent`]s. A tool calls it before its own input handling, which is the "manipulator"
//! slot of the routing priority (§6). The generic feature dialog (UI-05) binds it to the
//! schema fields that declare a manipulator.

use amg_core::{Dir3, Mat3, Plane, Point3, Ray3, Vec3};
use amg_render::{DepthMode, LineStyle, OverlayList, PointStyle};
use egui::Pos2;

use crate::input::{CameraView, ViewportEvent};
use crate::tool::InputResult;

/// Screen size of every gizmo, points.
pub const GIZMO_SIZE_PT: f32 = 90.0;

/// Hit-test tolerance around the projected gizmo geometry, points.
pub const HIT_TOLERANCE_PT: f32 = 6.0;

/// `Ctrl` snapping step of lengths, in the document's display length unit (1 mm in a mm
/// document). UI-11 makes it a preference.
pub const SNAP_LENGTH: f64 = 1.0;

/// `Ctrl` snapping step of angles, radians (5°). UI-11 makes it a preference.
pub const SNAP_ANGLE: f64 = 5.0 * std::f64::consts::PI / 180.0;

/// Triad proportions, fractions of the gizmo size.
const TRIAD_AXIS_START: f64 = 0.2;
const TRIAD_PLANE_LO: f64 = 0.3;
const TRIAD_PLANE_HI: f64 = 0.5;
const TRIAD_ARC_RADIUS: f64 = 0.75;
/// Radius of the triad's free centre, points.
const TRIAD_FREE_PT: f32 = 7.0;
/// Ring radius, fraction of the size (the size is the diameter).
const RING_RADIUS: f64 = 0.5;
/// Plane-handle half side, fraction of the size.
const PLANE_HALF: f64 = 0.5;
/// Polyline segments of a ring and of a triad arc.
const RING_SEGMENTS: usize = 64;
const ARC_SEGMENTS: usize = 16;
/// Line width of gizmos, points.
const LINE_PT: f32 = 2.5;

/// Linear RGBA of the hot (hovered / dragged) part.
const HOT: [f32; 4] = [1.0, 0.72, 0.05, 1.0];
/// Linear RGBA of arrows, rings and plane handles.
const BASE: [f32; 4] = [0.05, 0.35, 0.95, 1.0];
/// Linear RGBA of the triad axes X, Y, Z.
const AXIS_COLORS: [[f32; 4]; 3] = [
    [0.80, 0.08, 0.06, 1.0],
    [0.10, 0.62, 0.08, 1.0],
    [0.06, 0.20, 0.85, 1.0],
];

/// One gizmo as drawn and hit-tested this frame (Interfaces §A).
#[derive(Clone, Debug, PartialEq)]
pub enum GizmoPrim {
    /// An arrow from `origin` along `dir`.
    Arrow {
        /// Base of the arrow (world, mm).
        origin: Point3,
        /// Direction.
        dir: Dir3,
        /// Caller's id (reported in [`ManipEvent`]s).
        id: u32,
        /// Drawn in the hot colour.
        hot: bool,
    },
    /// A ring about `axis` through `center`; the pointer shows `ref_dir`.
    Ring {
        /// Centre (world, mm).
        center: Point3,
        /// Rotation axis.
        axis: Dir3,
        /// Current angle direction (perpendicular to `axis`).
        ref_dir: Dir3,
        /// Caller's id.
        id: u32,
        /// Drawn in the hot colour.
        hot: bool,
    },
    /// A move / rotate triad; `frame`'s columns are its x, y, z axes.
    Triad {
        /// Centre (world, mm).
        origin: Point3,
        /// Orthonormal axes as columns.
        frame: Mat3,
        /// Caller's id.
        id: u32,
        /// The part drawn in the hot colour.
        hot_part: Option<TriadPart>,
    },
    /// A square handle in the plane through `origin` normal to `normal`.
    PlaneHandle {
        /// Centre (world, mm).
        origin: Point3,
        /// Plane normal.
        normal: Dir3,
        /// Caller's id.
        id: u32,
        /// Drawn in the hot colour.
        hot: bool,
    },
}

/// The ten parts of a triad.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum TriadPart {
    /// Move along x.
    AxisX,
    /// Move along y.
    AxisY,
    /// Move along z.
    AxisZ,
    /// Move in the xy plane.
    PlaneXY,
    /// Move in the yz plane.
    PlaneYZ,
    /// Move in the zx plane.
    PlaneZX,
    /// Rotate about x.
    RotX,
    /// Rotate about y.
    RotY,
    /// Rotate about z.
    RotZ,
    /// Move in the view plane.
    Free,
}

impl TriadPart {
    /// All parts, in hit-test priority order (free centre, planes, axes, arcs).
    pub const ALL: [TriadPart; 10] = [
        TriadPart::Free,
        TriadPart::PlaneXY,
        TriadPart::PlaneYZ,
        TriadPart::PlaneZX,
        TriadPart::AxisX,
        TriadPart::AxisY,
        TriadPart::AxisZ,
        TriadPart::RotX,
        TriadPart::RotY,
        TriadPart::RotZ,
    ];

    /// A world point in the middle of the part, for a triad of world size `size` (mm): where a
    /// script or test grabs it.
    pub fn grab_point(self, origin: Point3, frame: &Mat3, size: f64) -> Point3 {
        let [x, y, z] = axes(frame);
        let mid = 0.5 * (TRIAD_PLANE_LO + TRIAD_PLANE_HI) * size;
        let axis_mid = 0.5 * (TRIAD_ARC_RADIUS + 1.0) * size;
        let arc = TRIAD_ARC_RADIUS * size * std::f64::consts::FRAC_1_SQRT_2;
        match self {
            TriadPart::AxisX => origin + x * axis_mid,
            TriadPart::AxisY => origin + y * axis_mid,
            TriadPart::AxisZ => origin + z * axis_mid,
            TriadPart::PlaneXY => origin + (x + y) * mid,
            TriadPart::PlaneYZ => origin + (y + z) * mid,
            TriadPart::PlaneZX => origin + (z + x) * mid,
            TriadPart::RotX => origin + (y + z) * arc,
            TriadPart::RotY => origin + (z + x) * arc,
            TriadPart::RotZ => origin + (x + y) * arc,
            TriadPart::Free => origin,
        }
    }

    /// The axis index (0..3) of an axis or rotation part.
    fn axis_index(self) -> Option<usize> {
        match self {
            TriadPart::AxisX | TriadPart::RotX => Some(0),
            TriadPart::AxisY | TriadPart::RotY => Some(1),
            TriadPart::AxisZ | TriadPart::RotZ => Some(2),
            _ => None,
        }
    }

    /// The two in-plane axis indices of a plane part.
    fn plane_axes(self) -> Option<(usize, usize)> {
        match self {
            TriadPart::PlaneXY => Some((0, 1)),
            TriadPart::PlaneYZ => Some((1, 2)),
            TriadPart::PlaneZX => Some((2, 0)),
            _ => None,
        }
    }
}

/// The part of a gizmo under the cursor or being dragged.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum GizmoPart {
    /// An arrow.
    Arrow,
    /// A ring.
    Ring,
    /// A plane handle.
    Plane,
    /// A triad part.
    Triad(TriadPart),
}

impl GizmoPrim {
    /// The caller's id.
    pub fn id(&self) -> u32 {
        match self {
            GizmoPrim::Arrow { id, .. }
            | GizmoPrim::Ring { id, .. }
            | GizmoPrim::Triad { id, .. }
            | GizmoPrim::PlaneHandle { id, .. } => *id,
        }
    }

    /// The anchor point the screen size is measured at.
    pub fn origin(&self) -> Point3 {
        match self {
            GizmoPrim::Arrow { origin, .. }
            | GizmoPrim::Triad { origin, .. }
            | GizmoPrim::PlaneHandle { origin, .. } => *origin,
            GizmoPrim::Ring { center, .. } => *center,
        }
    }

    /// The same gizmo with `part` hot (or nothing hot).
    pub fn with_hot(&self, part: Option<GizmoPart>) -> GizmoPrim {
        let mut g = self.clone();
        match &mut g {
            GizmoPrim::Arrow { hot, .. } => *hot = part == Some(GizmoPart::Arrow),
            GizmoPrim::Ring { hot, .. } => *hot = part == Some(GizmoPart::Ring),
            GizmoPrim::PlaneHandle { hot, .. } => *hot = part == Some(GizmoPart::Plane),
            GizmoPrim::Triad { hot_part, .. } => {
                *hot_part = match part {
                    Some(GizmoPart::Triad(p)) => Some(p),
                    _ => None,
                };
            }
        }
        g
    }
}

/// The columns of `frame`, normalised.
fn axes(frame: &Mat3) -> [Vec3; 3] {
    [
        frame.x_axis.normalize_or_zero(),
        frame.y_axis.normalize_or_zero(),
        frame.z_axis.normalize_or_zero(),
    ]
}

/// The triad frame as drawn in `cam`: each axis flipped, if needed, to point towards the
/// viewer, so the ten parts never overlap on screen (as long as no axis is seen end-on).
/// Moves are vectors and rotations are reported about the original axes, so the flip does not
/// change what a drag means.
pub fn facing_frame(frame: &Mat3, cam: &CameraView) -> Mat3 {
    let back = -cam.camera.forward().get();
    let f = |v: Vec3| if v.dot(back) < 0.0 { -v } else { v };
    let [x, y, z] = axes(frame);
    Mat3::from_cols(f(x), f(y), f(z))
}

/// A unit vector perpendicular to `n`.
pub fn perpendicular(n: Vec3) -> Vec3 {
    let a = if n.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    n.cross(a).normalize_or_zero()
}

/// An orthonormal frame with z = `axis` and x towards `ref_dir` (made perpendicular), or any
/// perpendicular when `ref_dir` is `None` or parallel to the axis.
pub fn frame_from(axis: Dir3, ref_dir: Option<Dir3>) -> Mat3 {
    let z = axis.get();
    let x = ref_dir
        .map(|r| (r.get() - z * r.get().dot(z)).normalize_or_zero())
        .filter(|v| v.length_squared() > 0.5)
        .unwrap_or_else(|| perpendicular(z));
    Mat3::from_cols(x, z.cross(x), z)
}

/// `v` rotated by `angle` (radians) about the unit `axis` (Rodrigues).
pub fn rotate(v: Vec3, axis: Vec3, angle: f64) -> Vec3 {
    let (s, c) = angle.sin_cos();
    v * c + axis.cross(v) * s + axis * axis.dot(v) * (1.0 - c)
}

/// Rounds `value` to a multiple of `step` (no-op for a non-positive step).
pub fn snap(value: f64, step: f64) -> f64 {
    if step > 0.0 {
        (value / step).round() * step
    } else {
        value
    }
}

/// World size (mm) of a gizmo at `p` drawn [`GIZMO_SIZE_PT`] on screen.
pub fn world_size(cam: &CameraView, p: Point3) -> f64 {
    f64::from(GIZMO_SIZE_PT) * cam.world_per_point(p)
}

/// The text a drag writes into a field: the value in its display unit with
/// [`DISPLAY_PRECISION`](crate::widgets::DISPLAY_PRECISION) decimals, trailing zeros trimmed
/// (`12.5 mm`, `22 mm`, `35 deg`), so it parses back to the shown value.
pub fn field_text(value: f64, dim: amg_core::Dimension, prefs: &amg_doc::UnitPrefs) -> String {
    let q = amg_core::Quantity::new(value, dim);
    let spec = amg_core::FormatSpec {
        style: amg_core::FormatStyle::Decimal,
        precision: crate::widgets::DISPLAY_PRECISION,
        trim_zeros: true,
        show_unit: true,
    };
    match crate::widgets::display_unit(dim, prefs) {
        Some(u) => amg_core::format_quantity(q, u, &spec)
            .unwrap_or_else(|_| crate::widgets::format_quantity(q, prefs)),
        None => crate::widgets::format_quantity(q, prefs),
    }
}

/// The `Ctrl` snapping step of a value of dimension `dim` (canonical units): [`SNAP_ANGLE`]
/// for angles, otherwise [`SNAP_LENGTH`] of the display unit (1 mm, 1 in, ...).
pub fn snap_step(dim: amg_core::Dimension, prefs: &amg_doc::UnitPrefs) -> f64 {
    if dim == amg_core::Dimension::ANGLE {
        return SNAP_ANGLE;
    }
    crate::widgets::display_unit(dim, prefs).map_or(SNAP_LENGTH, |u| u.to_canonical(SNAP_LENGTH))
}

// ---- screen shapes and hit tests ---------------------------------------------------------------

/// The screen form of one part.
#[derive(Clone, Debug, PartialEq)]
pub enum PartForm {
    /// An open polyline (points).
    Polyline(Vec<Pos2>),
    /// A closed convex polygon, hit inside.
    Polygon(Vec<Pos2>),
    /// A disc, hit inside.
    Disc {
        /// Centre.
        center: Pos2,
        /// Radius, points.
        radius: f32,
    },
}

impl PartForm {
    /// Screen distance from `p` (0 inside polygons and discs).
    pub fn distance(&self, p: Pos2) -> f32 {
        match self {
            PartForm::Polyline(pts) => polyline_distance(pts, p),
            PartForm::Polygon(pts) => {
                if inside_convex(pts, p) {
                    0.0
                } else {
                    let mut closed = pts.clone();
                    if let Some(first) = pts.first() {
                        closed.push(*first);
                    }
                    polyline_distance(&closed, p)
                }
            }
            PartForm::Disc { center, radius } => (center.distance(p) - radius).max(0.0),
        }
    }

    /// The points of the form (polyline / polygon vertices, or the disc centre).
    pub fn points(&self) -> Vec<Pos2> {
        match self {
            PartForm::Polyline(p) | PartForm::Polygon(p) => p.clone(),
            PartForm::Disc { center, .. } => vec![*center],
        }
    }
}

fn segment_distance(a: Pos2, b: Pos2, p: Pos2) -> f32 {
    let ab = b - a;
    let len2 = ab.length_sq();
    let t = if len2 > 0.0 {
        ((p - a).dot(ab) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (a + ab * t).distance(p)
}

fn polyline_distance(pts: &[Pos2], p: Pos2) -> f32 {
    match pts {
        [] => f32::INFINITY,
        [a] => a.distance(p),
        _ => pts
            .windows(2)
            .map(|w| segment_distance(w[0], w[1], p))
            .fold(f32::INFINITY, f32::min),
    }
}

fn inside_convex(pts: &[Pos2], p: Pos2) -> bool {
    if pts.len() < 3 {
        return false;
    }
    let (mut pos, mut neg) = (false, false);
    for i in 0..pts.len() {
        let a = pts[i];
        let b = pts[(i + 1) % pts.len()];
        let c = (b - a).x * (p - a).y - (b - a).y * (p - a).x;
        pos |= c > 0.0;
        neg |= c < 0.0;
    }
    !(pos && neg)
}

/// A gizmo projected into the view: its parts in hit-test priority order.
#[derive(Clone, Debug, PartialEq)]
pub struct GizmoShape {
    /// The gizmo's id.
    pub id: u32,
    /// The world size the gizmo has in this view (mm).
    pub world_size: f64,
    /// The parts' screen forms (parts behind the camera are left out).
    pub parts: Vec<(GizmoPart, PartForm)>,
}

impl GizmoShape {
    /// The nearest part within `tol` points of `p` and its distance (the first of equals).
    pub fn hit(&self, p: Pos2, tol: f32) -> Option<(GizmoPart, f32)> {
        let mut best: Option<(GizmoPart, f32)> = None;
        for (part, form) in &self.parts {
            let d = form.distance(p);
            if d <= tol && best.is_none_or(|(_, b)| d < b) {
                best = Some((*part, d));
            }
        }
        best
    }

    /// The form of `part`.
    pub fn part(&self, part: GizmoPart) -> Option<&PartForm> {
        self.parts.iter().find(|(p, _)| *p == part).map(|(_, f)| f)
    }
}

fn project_all(cam: &CameraView, pts: &[Point3]) -> Option<Vec<Pos2>> {
    pts.iter().map(|p| cam.project(*p)).collect()
}

fn ring_points(
    center: Point3,
    u: Vec3,
    v: Vec3,
    r: f64,
    from: f64,
    to: f64,
    n: usize,
) -> Vec<Point3> {
    (0..=n)
        .map(|i| {
            #[allow(clippy::cast_precision_loss, reason = "small segment counts")]
            let t = from + (to - from) * (i as f64 / n as f64);
            center + (u * t.cos() + v * t.sin()) * r
        })
        .collect()
}

/// The world points of a square centred on `c` with half sides `h` along `u`, `v`.
fn square(c: Point3, u: Vec3, v: Vec3, h: f64) -> [Point3; 4] {
    [
        c + (-u - v) * h,
        c + (u - v) * h,
        c + (u + v) * h,
        c + (-u + v) * h,
    ]
}

/// The plane squares of a triad: corner `origin + (a + b) * lo` to `origin + (a + b) * hi`.
fn triad_square(origin: Point3, a: Vec3, b: Vec3, size: f64) -> [Point3; 4] {
    let (lo, hi) = (TRIAD_PLANE_LO * size, TRIAD_PLANE_HI * size);
    [
        origin + a * lo + b * lo,
        origin + a * hi + b * lo,
        origin + a * hi + b * hi,
        origin + a * lo + b * hi,
    ]
}

/// The arc of rotation part `axis_i`: in the plane of the other two axes, a quarter circle.
fn triad_arc(origin: Point3, ax: &[Vec3; 3], axis_i: usize, size: f64) -> Vec<Point3> {
    let u = ax[(axis_i + 1) % 3];
    let v = ax[(axis_i + 2) % 3];
    ring_points(
        origin,
        u,
        v,
        TRIAD_ARC_RADIUS * size,
        0.0,
        std::f64::consts::FRAC_PI_2,
        ARC_SEGMENTS,
    )
}

/// Projects `g` into `cam` (the hit-test geometry).
pub fn shape(g: &GizmoPrim, cam: &CameraView) -> GizmoShape {
    let s = world_size(cam, g.origin());
    let mut parts = Vec::new();
    match g {
        GizmoPrim::Arrow { origin, dir, .. } => {
            if let Some(p) = project_all(cam, &[*origin, *origin + dir.get() * s]) {
                parts.push((GizmoPart::Arrow, PartForm::Polyline(p)));
            }
        }
        GizmoPrim::Ring { center, axis, .. } => {
            let f = frame_from(*axis, None);
            let pts = ring_points(
                *center,
                f.x_axis,
                f.y_axis,
                RING_RADIUS * s,
                0.0,
                std::f64::consts::TAU,
                RING_SEGMENTS,
            );
            if let Some(p) = project_all(cam, &pts) {
                parts.push((GizmoPart::Ring, PartForm::Polyline(p)));
            }
        }
        GizmoPrim::PlaneHandle { origin, normal, .. } => {
            let f = frame_from(*normal, None);
            let sq = square(*origin, f.x_axis, f.y_axis, PLANE_HALF * 0.5 * s);
            if let Some(p) = project_all(cam, &sq) {
                parts.push((GizmoPart::Plane, PartForm::Polygon(p)));
            }
        }
        GizmoPrim::Triad { origin, frame, .. } => {
            let ax = axes(&facing_frame(frame, cam));
            for part in TriadPart::ALL {
                let form = match part {
                    TriadPart::Free => cam.project(*origin).map(|c| PartForm::Disc {
                        center: c,
                        radius: TRIAD_FREE_PT,
                    }),
                    TriadPart::PlaneXY | TriadPart::PlaneYZ | TriadPart::PlaneZX => {
                        let Some((a, b)) = part.plane_axes() else {
                            continue;
                        };
                        project_all(cam, &triad_square(*origin, ax[a], ax[b], s))
                            .map(PartForm::Polygon)
                    }
                    TriadPart::AxisX | TriadPart::AxisY | TriadPart::AxisZ => {
                        let Some(i) = part.axis_index() else {
                            continue;
                        };
                        project_all(
                            cam,
                            &[
                                *origin + ax[i] * (TRIAD_AXIS_START * s),
                                *origin + ax[i] * s,
                            ],
                        )
                        .map(PartForm::Polyline)
                    }
                    TriadPart::RotX | TriadPart::RotY | TriadPart::RotZ => {
                        let Some(i) = part.axis_index() else {
                            continue;
                        };
                        project_all(cam, &triad_arc(*origin, &ax, i, s)).map(PartForm::Polyline)
                    }
                };
                if let Some(form) = form {
                    parts.push((GizmoPart::Triad(part), form));
                }
            }
        }
    }
    GizmoShape {
        id: g.id(),
        world_size: s,
        parts,
    }
}

/// The nearest gizmo part within [`HIT_TOLERANCE_PT`] of `pos` (the first gizmo of equals).
pub fn hit_test(gizmos: &[GizmoPrim], cam: &CameraView, pos: Pos2) -> Option<(u32, GizmoPart)> {
    let mut best: Option<(u32, GizmoPart, f32)> = None;
    for g in gizmos {
        if let Some((part, d)) = shape(g, cam).hit(pos, HIT_TOLERANCE_PT)
            && best.is_none_or(|(_, _, b)| d < b)
        {
            best = Some((g.id(), part, d));
        }
    }
    best.map(|(id, part, _)| (id, part))
}

// ---- drawing ------------------------------------------------------------------------------------

fn style(color: [f32; 4], cam: &CameraView) -> LineStyle {
    LineStyle {
        width_px: LINE_PT * cam.pixels_per_point,
        color,
        dash: None,
        depth: DepthMode::AlwaysOnTop,
    }
}

fn pick(hot: bool, color: [f32; 4]) -> [f32; 4] {
    if hot { HOT } else { color }
}

fn translucent(c: [f32; 4]) -> [f32; 4] {
    [c[0], c[1], c[2], 0.35]
}

/// Draws `g` into `out` (RND-07 overlays, always on top; sizes are physical pixels of `cam`).
pub fn draw(out: &mut OverlayList, g: &GizmoPrim, cam: &CameraView) {
    let s = world_size(cam, g.origin());
    let px = GIZMO_SIZE_PT * cam.pixels_per_point;
    match g {
        GizmoPrim::Arrow {
            origin, dir, hot, ..
        } => {
            out.arrow(*origin, *dir, px, style(pick(*hot, BASE), cam), None);
        }
        GizmoPrim::Ring {
            center,
            axis,
            ref_dir,
            hot,
            ..
        } => {
            let c = pick(*hot, BASE);
            #[allow(clippy::cast_possible_truncation, reason = "pixel sizes are f32")]
            out.ring(
                *center,
                *axis,
                (f64::from(px) * RING_RADIUS) as f32,
                style(c, cam),
                None,
            );
            out.polyline(
                &[*center, *center + ref_dir.get() * (RING_RADIUS * s)],
                style(c, cam),
                None,
            );
        }
        GizmoPrim::PlaneHandle {
            origin,
            normal,
            hot,
            ..
        } => {
            let f = frame_from(*normal, None);
            let c = pick(*hot, BASE);
            let h = PLANE_HALF * 0.5 * s;
            let plane = Plane {
                origin: *origin,
                x: Dir3::new_unchecked(f.x_axis),
                y: Dir3::new_unchecked(f.y_axis),
                normal: *normal,
            };
            out.plane_quad(&plane, [h, h], translucent(c), style(c, cam), None);
        }
        GizmoPrim::Triad {
            origin,
            frame,
            hot_part,
            ..
        } => {
            let ax = axes(&facing_frame(frame, cam));
            let hot = |p: TriadPart| *hot_part == Some(p);
            for (i, part) in [TriadPart::AxisX, TriadPart::AxisY, TriadPart::AxisZ]
                .into_iter()
                .enumerate()
            {
                #[allow(clippy::cast_possible_truncation, reason = "pixel sizes are f32")]
                let len = (f64::from(px) * (1.0 - TRIAD_AXIS_START)) as f32;
                out.arrow(
                    *origin + ax[i] * (TRIAD_AXIS_START * s),
                    Dir3::new_unchecked(ax[i]),
                    len,
                    style(pick(hot(part), AXIS_COLORS[i]), cam),
                    None,
                );
            }
            for (part, (a, b)) in [
                (TriadPart::PlaneXY, (0, 1)),
                (TriadPart::PlaneYZ, (1, 2)),
                (TriadPart::PlaneZX, (2, 0)),
            ] {
                let half = 0.5 * (TRIAD_PLANE_HI - TRIAD_PLANE_LO) * s;
                let mid = 0.5 * (TRIAD_PLANE_HI + TRIAD_PLANE_LO) * s;
                let n = ax[a].cross(ax[b]).normalize_or_zero();
                let plane = Plane {
                    origin: *origin + (ax[a] + ax[b]) * mid,
                    x: Dir3::new_unchecked(ax[a]),
                    y: Dir3::new_unchecked(ax[b]),
                    normal: Dir3::new_unchecked(n),
                };
                // The square takes the colour of the axis normal to it.
                let c = pick(hot(part), AXIS_COLORS[3 - a - b]);
                out.plane_quad(&plane, [half, half], translucent(c), style(c, cam), None);
            }
            for (i, part) in [TriadPart::RotX, TriadPart::RotY, TriadPart::RotZ]
                .into_iter()
                .enumerate()
            {
                out.polyline(
                    &triad_arc(*origin, &ax, i, s),
                    style(pick(hot(part), AXIS_COLORS[i]), cam),
                    None,
                );
            }
            out.points(
                &[*origin],
                PointStyle {
                    size_px: 2.0 * TRIAD_FREE_PT * cam.pixels_per_point,
                    color: pick(hot(TriadPart::Free), [0.9, 0.9, 0.9, 1.0]),
                    depth: DepthMode::AlwaysOnTop,
                },
                None,
            );
        }
    }
}

// ---- dragging -----------------------------------------------------------------------------------

/// What a drag changed since it started.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum DragValue {
    /// Distance moved along the dragged axis (mm, signed by the axis).
    Along(f64),
    /// Rotation about the dragged axis (radians, right-handed).
    Angle(f64),
    /// Translation (mm).
    Move(Vec3),
}

impl DragValue {
    /// The change as a length along `axis` (moves are projected onto it), or `None` for an
    /// angle.
    pub fn along(self, axis: Vec3) -> Option<f64> {
        match self {
            DragValue::Along(d) => Some(d),
            DragValue::Move(v) => Some(v.dot(axis)),
            DragValue::Angle(_) => None,
        }
    }
}

/// How the cursor ray is read during a drag.
#[derive(Copy, Clone, Debug, PartialEq)]
enum Grab {
    /// Closest point on the line `origin + t·dir`, `t0` at the start; `moves` reports a
    /// translation instead of a distance.
    Line {
        origin: Point3,
        dir: Vec3,
        t0: f64,
        moves: bool,
    },
    /// Angle about `axis` in the plane through `center`; `last` is the previous reading and
    /// `acc` the unwrapped angle since the start.
    Angle {
        center: Point3,
        axis: Vec3,
        u: Vec3,
        v: Vec3,
        last: f64,
        acc: f64,
    },
    /// The point in the plane through `p0` normal to `normal`.
    Plane { p0: Point3, normal: Vec3 },
}

/// Parameter on the line `o + t·d` (unit `d`) closest to `ray`; `None` when they are parallel.
fn line_param(o: Point3, d: Vec3, ray: &Ray3) -> Option<f64> {
    let rd = ray.dir.get();
    let w0 = o - ray.origin;
    let b = d.dot(rd);
    let denom = 1.0 - b * b;
    if denom < 1e-9 {
        return None;
    }
    Some((b * rd.dot(w0) - d.dot(w0)) / denom)
}

/// Where `ray` meets the plane through `p` normal to `n` (also behind the eye in orthographic
/// views); `None` when the ray is parallel to it.
fn plane_hit(p: Point3, n: Vec3, ray: &Ray3) -> Option<Point3> {
    let rd = ray.dir.get();
    let den = rd.dot(n);
    if den.abs() < 1e-9 {
        return None;
    }
    Some(ray.at((p - ray.origin).dot(n) / den))
}

fn angle_in(center: Point3, u: Vec3, v: Vec3, q: Point3) -> f64 {
    let w = q - center;
    w.dot(v).atan2(w.dot(u))
}

/// A drag of one gizmo part, read against the gizmo as it was when the drag started.
#[derive(Clone, Debug, PartialEq)]
pub struct GizmoDrag {
    /// The gizmo's id.
    pub id: u32,
    /// The dragged part.
    pub part: GizmoPart,
    grab: Grab,
}

impl GizmoDrag {
    /// Starts dragging `part` of `g` with the cursor at `pos`. `None` when the part cannot be
    /// dragged in this view (an axis seen end-on, a ring or plane seen edge-on).
    pub fn begin(g: &GizmoPrim, part: GizmoPart, cam: &CameraView, pos: Pos2) -> Option<GizmoDrag> {
        let ray = cam.ray(pos);
        let line = |origin: Point3, dir: Vec3, moves: bool| {
            line_param(origin, dir, &ray).map(|t0| Grab::Line {
                origin,
                dir,
                t0,
                moves,
            })
        };
        let angle = |center: Point3, axis: Vec3| {
            let f = frame_from(Dir3::new_unchecked(axis), None);
            let (u, v) = (f.x_axis, f.y_axis);
            let q = plane_hit(center, axis, &ray)?;
            let a0 = angle_in(center, u, v, q);
            Some(Grab::Angle {
                center,
                axis,
                u,
                v,
                last: a0,
                acc: 0.0,
            })
        };
        let plane = |p: Point3, normal: Vec3| {
            plane_hit(p, normal, &ray).map(|p0| Grab::Plane { p0, normal })
        };
        let grab = match (g, part) {
            (GizmoPrim::Arrow { origin, dir, .. }, GizmoPart::Arrow) => {
                line(*origin, dir.get(), false)?
            }
            (GizmoPrim::Ring { center, axis, .. }, GizmoPart::Ring) => angle(*center, axis.get())?,
            (GizmoPrim::PlaneHandle { origin, normal, .. }, GizmoPart::Plane) => {
                plane(*origin, normal.get())?
            }
            (GizmoPrim::Triad { origin, frame, .. }, GizmoPart::Triad(p)) => {
                let ax = axes(&facing_frame(frame, cam));
                match p {
                    TriadPart::AxisX | TriadPart::AxisY | TriadPart::AxisZ => {
                        line(*origin, ax[p.axis_index()?], true)?
                    }
                    TriadPart::RotX | TriadPart::RotY | TriadPart::RotZ => {
                        // About the original axis (the drawn one may be flipped).
                        angle(*origin, axes(frame)[p.axis_index()?])?
                    }
                    TriadPart::PlaneXY | TriadPart::PlaneYZ | TriadPart::PlaneZX => {
                        let (a, b) = p.plane_axes()?;
                        plane(*origin, ax[a].cross(ax[b]).normalize_or_zero())?
                    }
                    TriadPart::Free => plane(*origin, cam.camera.forward().get())?,
                }
            }
            _ => return None,
        };
        Some(GizmoDrag {
            id: g.id(),
            part,
            grab,
        })
    }

    /// The change since the start with the cursor at `pos` (`None` when the ray cannot be
    /// read, e.g. parallel to the plane; keep the last value then).
    pub fn update(&mut self, cam: &CameraView, pos: Pos2) -> Option<DragValue> {
        let ray = cam.ray(pos);
        match &mut self.grab {
            Grab::Line {
                origin,
                dir,
                t0,
                moves,
            } => {
                let d = line_param(*origin, *dir, &ray)? - *t0;
                Some(if *moves {
                    DragValue::Move(*dir * d)
                } else {
                    DragValue::Along(d)
                })
            }
            Grab::Angle {
                center,
                axis,
                u,
                v,
                last,
                acc,
            } => {
                let q = plane_hit(*center, *axis, &ray)?;
                let a = angle_in(*center, *u, *v, q);
                // Unwrap across ±π so a drag can go round more than half a turn.
                let mut step = a - *last;
                if step > std::f64::consts::PI {
                    step -= std::f64::consts::TAU;
                } else if step < -std::f64::consts::PI {
                    step += std::f64::consts::TAU;
                }
                *acc += step;
                *last = a;
                Some(DragValue::Angle(*acc))
            }
            Grab::Plane { p0, normal } => {
                let q = plane_hit(*p0, *normal, &ray)?;
                Some(DragValue::Move(q - *p0))
            }
        }
    }
}

// ---- the per-tool controller ---------------------------------------------------------------------

/// What the user did with a gizmo.
#[derive(Clone, Debug, PartialEq)]
pub enum ManipEvent {
    /// A drag started on `part` of gizmo `id`.
    Begin {
        /// The gizmo.
        id: u32,
        /// The part.
        part: GizmoPart,
    },
    /// The drag moved: the change since the start, whether `Ctrl` is held (snap), and the
    /// cursor position (for the heads-up value).
    Drag {
        /// The gizmo.
        id: u32,
        /// Change since the start.
        value: DragValue,
        /// `Ctrl` is held.
        snap: bool,
        /// Cursor, points.
        pos: Pos2,
    },
    /// The drag ended with this final change (`None`: unreadable last position, keep the
    /// previous value).
    End {
        /// The gizmo.
        id: u32,
        /// Final change since the start.
        value: Option<DragValue>,
        /// `Ctrl` is held.
        snap: bool,
    },
    /// `Esc` during the drag: restore the value from before it.
    Cancel {
        /// The gizmo.
        id: u32,
    },
}

/// What [`Manipulators::input`] did with an event.
#[derive(Clone, Debug, PartialEq)]
pub struct ManipResponse {
    /// `Consumed` when a gizmo took the event (the tool should not handle it).
    pub result: InputResult,
    /// The gizmo event, if any.
    pub event: Option<ManipEvent>,
    /// The hot part changed (repaint).
    pub changed: bool,
}

impl ManipResponse {
    fn ignored() -> ManipResponse {
        ManipResponse {
            result: InputResult::Ignored,
            event: None,
            changed: false,
        }
    }

    fn consumed(event: Option<ManipEvent>) -> ManipResponse {
        ManipResponse {
            result: InputResult::Consumed,
            event,
            changed: true,
        }
    }
}

/// The manipulators of a tool: this frame's gizmos, hover, and the drag in progress.
#[derive(Clone, Debug, Default)]
pub struct Manipulators {
    gizmos: Vec<GizmoPrim>,
    hot: Option<(u32, GizmoPart)>,
    /// A primary press landed on this part (a drag may follow).
    armed: Option<(u32, GizmoPart)>,
    drag: Option<GizmoDrag>,
    /// Newest cursor position over the viewport.
    cursor: Option<Pos2>,
}

impl Manipulators {
    /// No gizmos.
    pub fn new() -> Manipulators {
        Manipulators::default()
    }

    /// Replaces this frame's gizmos (immediate mode). A drag in progress keeps reading against
    /// its start geometry; its gizmo may move.
    pub fn set(&mut self, gizmos: Vec<GizmoPrim>) {
        self.gizmos = gizmos;
        if let Some((id, _)) = self.hot
            && !self.gizmos.iter().any(|g| g.id() == id)
        {
            self.hot = None;
        }
    }

    /// This frame's gizmos, with the hot part marked.
    pub fn gizmos(&self) -> Vec<GizmoPrim> {
        let hot = self.drag.as_ref().map(|d| (d.id, d.part)).or(self.hot);
        self.gizmos
            .iter()
            .map(|g| g.with_hot(hot.filter(|(id, _)| *id == g.id()).map(|(_, p)| p)))
            .collect()
    }

    /// The gizmo with `id`.
    pub fn gizmo(&self, id: u32) -> Option<&GizmoPrim> {
        self.gizmos.iter().find(|g| g.id() == id)
    }

    /// The hovered part (not updated during a drag).
    pub fn hot(&self) -> Option<(u32, GizmoPart)> {
        self.hot
    }

    /// The drag in progress.
    pub fn dragging(&self) -> Option<&GizmoDrag> {
        self.drag.as_ref()
    }

    /// The newest cursor position seen.
    pub fn cursor(&self) -> Option<Pos2> {
        self.cursor
    }

    /// The part under `pos`.
    pub fn hit(&self, cam: &CameraView, pos: Pos2) -> Option<(u32, GizmoPart)> {
        hit_test(&self.gizmos, cam, pos)
    }

    /// Draws every gizmo (hot part highlighted).
    pub fn draw(&self, out: &mut OverlayList, cam: &CameraView) {
        for g in self.gizmos() {
            draw(out, &g, cam);
        }
    }

    /// Ends any drag without an event (the tool finished).
    pub fn reset(&mut self) {
        self.drag = None;
        self.armed = None;
        self.hot = None;
    }

    /// Handles a viewport event before the tool: hover tracking, and the primary press / drag
    /// / release that start on a gizmo (consumed, so the tool and navigation never see them).
    pub fn input(&mut self, ev: &ViewportEvent, cam: &CameraView) -> ManipResponse {
        use egui::PointerButton::Primary;
        if let Some(p) = ev.pos() {
            self.cursor = Some(p);
        }
        match ev {
            ViewportEvent::Hover { pos, .. } if self.drag.is_none() => {
                let hot = self.hit(cam, *pos);
                let changed = hot != self.hot;
                self.hot = hot;
                ManipResponse {
                    result: InputResult::Ignored,
                    event: None,
                    changed,
                }
            }
            ViewportEvent::Press {
                button: Primary,
                pos,
                ..
            } => match self.hit(cam, *pos) {
                Some(h) => {
                    self.armed = Some(h);
                    self.hot = Some(h);
                    ManipResponse::consumed(None)
                }
                None => {
                    self.armed = None;
                    ManipResponse::ignored()
                }
            },
            ViewportEvent::DragStart {
                button: Primary,
                from,
                ..
            } => {
                let Some((id, part)) = self.armed.take() else {
                    return ManipResponse::ignored();
                };
                let Some(g) = self.gizmo(id).cloned() else {
                    return ManipResponse::ignored();
                };
                match GizmoDrag::begin(&g, part, cam, *from) {
                    Some(d) => {
                        self.drag = Some(d);
                        ManipResponse::consumed(Some(ManipEvent::Begin { id, part }))
                    }
                    // Not draggable in this view: still keep the press from selecting.
                    None => ManipResponse::consumed(None),
                }
            }
            ViewportEvent::Drag {
                button: Primary,
                pos,
                mods,
                ..
            } => match &mut self.drag {
                Some(d) => {
                    let event = d.update(cam, *pos).map(|value| ManipEvent::Drag {
                        id: d.id,
                        value,
                        snap: mods.ctrl,
                        pos: *pos,
                    });
                    ManipResponse::consumed(event)
                }
                None => ManipResponse::ignored(),
            },
            ViewportEvent::DragEnd {
                button: Primary,
                pos,
                mods,
                ..
            } => match self.drag.take() {
                Some(mut d) => {
                    let value = d.update(cam, *pos);
                    self.hot = self.hit(cam, *pos);
                    ManipResponse::consumed(Some(ManipEvent::End {
                        id: d.id,
                        value,
                        snap: mods.ctrl,
                    }))
                }
                None => ManipResponse::ignored(),
            },
            ViewportEvent::Release {
                button: Primary, ..
            } if self.armed.is_some() || self.drag.is_some() => ManipResponse::consumed(None),
            ViewportEvent::Click {
                button: Primary, ..
            } if self.armed.is_some() => {
                // A click on a gizmo does not select through it.
                self.armed = None;
                ManipResponse::consumed(None)
            }
            ViewportEvent::Key {
                key: egui::Key::Escape,
                pressed: true,
                ..
            } => match self.drag.take() {
                Some(d) => ManipResponse::consumed(Some(ManipEvent::Cancel { id: d.id })),
                None => ManipResponse::ignored(),
            },
            _ => ManipResponse::ignored(),
        }
    }
}

#[cfg(test)]
mod tests {
    use amg_render::{Camera, Projection};

    use super::*;

    fn view(eye: Point3) -> CameraView {
        CameraView::new(
            Camera {
                eye,
                target: Point3::ZERO,
                up: Dir3::Z,
                projection: Projection::Perspective { fov_y: 0.8 },
            },
            egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(800.0, 600.0)),
            1.0,
        )
    }

    #[test]
    fn snapping_rounds_to_the_step() {
        assert!((snap(22.3, 1.0) - 22.0).abs() < 1e-12);
        assert!((snap(-2.6, 1.0) + 3.0).abs() < 1e-12);
        let a = snap(37.0_f64.to_radians(), SNAP_ANGLE);
        assert!((a.to_degrees() - 35.0).abs() < 1e-9);
        assert!((snap(1.234, 0.0) - 1.234).abs() < 1e-12);
    }

    #[test]
    fn arrow_drag_reads_the_axis_under_the_cursor() {
        let cam = view(Point3::new(80.0, -120.0, 70.0));
        let g = GizmoPrim::Arrow {
            origin: Point3::ZERO,
            dir: Dir3::Z,
            id: 7,
            hot: false,
        };
        let s = world_size(&cam, Point3::ZERO);
        let grab = Point3::new(0.0, 0.0, 0.5 * s);
        let from = cam.project(grab).unwrap();
        assert_eq!(
            hit_test(std::slice::from_ref(&g), &cam, from),
            Some((7, GizmoPart::Arrow))
        );
        let mut d = GizmoDrag::begin(&g, GizmoPart::Arrow, &cam, from).unwrap();
        let to = cam.project(grab + Vec3::Z * 12.5).unwrap();
        match d.update(&cam, to) {
            Some(DragValue::Along(v)) => assert!((v - 12.5).abs() < 1e-3, "{v}"),
            other => panic!("{other:?}"),
        }
        // 10 pt beside the shaft is a miss.
        let side = from + egui::vec2(HIT_TOLERANCE_PT + 4.0, 0.0);
        assert_eq!(hit_test(&[g], &cam, side), None);
    }

    #[test]
    fn ring_drag_unwraps_the_angle() {
        let cam = view(Point3::new(10.0, -20.0, 200.0));
        let g = GizmoPrim::Ring {
            center: Point3::ZERO,
            axis: Dir3::Z,
            ref_dir: Dir3::X,
            id: 1,
            hot: false,
        };
        let s = world_size(&cam, Point3::ZERO);
        let r = RING_RADIUS * s;
        let at = |deg: f64| {
            let a = deg.to_radians();
            cam.project(Point3::new(r * a.cos(), r * a.sin(), 0.0))
                .unwrap()
        };
        assert_eq!(
            hit_test(std::slice::from_ref(&g), &cam, at(30.0)).map(|h| h.1),
            Some(GizmoPart::Ring)
        );
        let mut d = GizmoDrag::begin(&g, GizmoPart::Ring, &cam, at(0.0)).unwrap();
        let mut last = 0.0;
        for deg in (10..=270).step_by(10) {
            if let Some(DragValue::Angle(a)) = d.update(&cam, at(f64::from(deg))) {
                last = a;
            }
        }
        assert!(
            (last.to_degrees() - 270.0).abs() < 0.5,
            "{}",
            last.to_degrees()
        );
    }

    #[test]
    fn controller_consumes_a_drag_that_starts_on_a_gizmo() {
        let cam = view(Point3::new(80.0, -120.0, 70.0));
        let mut m = Manipulators::new();
        m.set(vec![GizmoPrim::Arrow {
            origin: Point3::ZERO,
            dir: Dir3::Z,
            id: 3,
            hot: false,
        }]);
        let s = world_size(&cam, Point3::ZERO);
        let p = cam.project(Point3::new(0.0, 0.0, 0.5 * s)).unwrap();
        let q = cam.project(Point3::new(0.0, 0.0, 0.5 * s + 4.0)).unwrap();
        let ray = cam.ray(p);
        let mods = egui::Modifiers::NONE;
        let r = m.input(&ViewportEvent::Hover { pos: p, ray }, &cam);
        assert!(r.changed && m.hot() == Some((3, GizmoPart::Arrow)));
        let r = m.input(
            &ViewportEvent::Press {
                button: egui::PointerButton::Primary,
                pos: p,
                ray,
                mods,
            },
            &cam,
        );
        assert_eq!(r.result, InputResult::Consumed);
        let r = m.input(
            &ViewportEvent::DragStart {
                button: egui::PointerButton::Primary,
                from: p,
                mods,
            },
            &cam,
        );
        assert_eq!(
            r.event,
            Some(ManipEvent::Begin {
                id: 3,
                part: GizmoPart::Arrow
            })
        );
        let r = m.input(
            &ViewportEvent::Drag {
                button: egui::PointerButton::Primary,
                pos: q,
                delta: q - p,
                ray: cam.ray(q),
                mods: egui::Modifiers::CTRL,
            },
            &cam,
        );
        match r.event {
            Some(ManipEvent::Drag {
                value: DragValue::Along(v),
                snap: true,
                ..
            }) => assert!((v - 4.0).abs() < 1e-3),
            other => panic!("{other:?}"),
        }
        let r = m.input(
            &ViewportEvent::Key {
                key: egui::Key::Escape,
                pressed: true,
                mods,
            },
            &cam,
        );
        assert_eq!(r.event, Some(ManipEvent::Cancel { id: 3 }));
        assert!(m.dragging().is_none());
        // A press away from the gizmo is not taken.
        let away = egui::pos2(5.0, 5.0);
        let r = m.input(
            &ViewportEvent::Press {
                button: egui::PointerButton::Primary,
                pos: away,
                ray: cam.ray(away),
                mods,
            },
            &cam,
        );
        assert_eq!(r.result, InputResult::Ignored);
    }
}
