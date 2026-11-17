//! Navigation and the view cube (RND-11, design note §13): pure f64 camera math, no GPU.
//!
//! - World up is +Z. The Front view looks along +Y with the camera at −Y; Home is the iso view
//!   from direction `(1, −1, 1)`.
//! - [`NavController`] orbits about a pivot (free, or constrained to ±89.9° pitch), zooms to the
//!   cursor, pans with the grabbed point under the cursor, fits bounds, sets standard views and
//!   sketch "look at" views, and switches projection keeping the apparent size at the target.
//!   Fit, standard views and look-at can animate over [`TRANSITION`] with smoothstep easing
//!   (orientation slerp, target lerp, log-distance / log-height lerp) driven by
//!   [`NavController::tick`].
//! - The view cube is a [`ViewCube`] inset (RND-07's [`OverlayInset`]): 54 shaded quads (the 26
//!   regions of the 6 faces), the 12 cube edges and egui face letters, drawn with an
//!   orthographic camera sharing the view's orientation. [`view_cube_hit`] is analytic (ray
//!   against `[−1, 1]³`; coordinates with `|c| > 0.6` mark the faces touched).
//! - Which mouse button does what is S11's `NavPreset` table; it calls [`NavController`].
//! - The near-plane policy lives with the camera ([`Camera::depth_range_for`]): perspective near
//!   = 1e-3 of the eye-to-target distance (≥ 1e-4 mm), which the controller keeps at the depth
//!   of what is being looked at (zoom scales the target with the eye); orthographic views fit
//!   the depth range to the visible scene.

use std::f64::consts::PI;
use std::time::Duration;

use amg_core::{Aabb3, Dir3, Mat3, Plane, Point3, Quat, Vec3};

use crate::camera::{Camera, Projection};
use crate::lines::{DepthMode, LineStyle};
use crate::overlay::{LabelStyle, OverlayInset, OverlayList};

/// Duration of animated transitions (fit, standard views, view cube, look-at).
pub const TRANSITION: Duration = Duration::from_millis(300);
/// Orbit angle per viewport height of drag, radians (180°).
pub const ORBIT_RAD_PER_HEIGHT: f64 = PI;
/// Zoom factor of one wheel step (`distance · 0.9^steps`).
pub const ZOOM_STEP: f64 = 0.9;
/// Drag distance of one zoom step in [`NavMode::Zoom`] drags, px.
pub const ZOOM_DRAG_PX_PER_STEP: f64 = 20.0;
/// Pitch limit of the constrained (turntable) orbit, radians (89.9°).
pub const CONSTRAINED_PITCH_LIMIT: f64 = 89.9 * PI / 180.0;
/// Field of view used when switching an orthographic view to perspective, radians (45°).
pub const DEFAULT_FOV_Y: f64 = PI / 4.0;
/// Fit: perspective distance `FIT_MARGIN · r / sin(min(fov_x, fov_y) / 2)`.
const FIT_MARGIN: f64 = 1.05;
/// Fit: orthographic height `ORTHO_FIT · r / min(1, aspect)`.
const ORTHO_FIT: f64 = 2.1;
/// Radius used to fit degenerate (point) bounds, mm.
const MIN_FIT_RADIUS: f64 = 1e-3;
/// View-cube coordinates beyond this mark a face touched (design note §13).
const CUBE_REGION_EDGE: f64 = 0.6;
/// Orthographic height of the view-cube camera in cube units (the cube is `[−1, 1]³`; its
/// longest projection, the space diagonal, is `2√3 ≈ 3.46`).
const CUBE_VIEW_HEIGHT: f64 = 3.6;
/// Distance of the view-cube camera from the cube centre, cube units.
const CUBE_EYE_DISTANCE: f64 = 6.0;
/// Face letters show when the face normal faces the viewer by more than this (`n · view`).
const CUBE_LABEL_MIN_FACING: f64 = 0.25;

/// A standard view direction.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum StandardView {
    /// Camera at −Y looking along +Y.
    Front,
    /// Camera at +Y.
    Back,
    /// Camera at −X.
    Left,
    /// Camera at +X.
    Right,
    /// Camera at +Z looking down, +Y up on screen.
    Top,
    /// Camera at −Z looking up, −Y up on screen.
    Bottom,
    /// The iso view from `(1, −1, 1)`.
    Home,
    /// The view from direction `(x, y, z)` (each −1, 0 or 1): cube edges and corners.
    Iso {
        /// X component of the eye direction.
        x: i8,
        /// Y component.
        y: i8,
        /// Z component.
        z: i8,
    },
}

/// The six faces in [`CubeRegion::Edge`] axis order: X faces, then Y, then Z.
const FACES: [StandardView; 6] = [
    StandardView::Left,
    StandardView::Right,
    StandardView::Front,
    StandardView::Back,
    StandardView::Bottom,
    StandardView::Top,
];

impl StandardView {
    /// Unit direction from the target toward the eye.
    pub fn eye_direction(self) -> Vec3 {
        match self {
            StandardView::Front => Vec3::NEG_Y,
            StandardView::Back => Vec3::Y,
            StandardView::Left => Vec3::NEG_X,
            StandardView::Right => Vec3::X,
            StandardView::Top => Vec3::Z,
            StandardView::Bottom => Vec3::NEG_Z,
            StandardView::Home => Vec3::new(1.0, -1.0, 1.0).normalize(),
            StandardView::Iso { x, y, z } => {
                let v = Vec3::new(
                    f64::from(x.signum()),
                    f64::from(y.signum()),
                    f64::from(z.signum()),
                );
                if v == Vec3::ZERO {
                    StandardView::Home.eye_direction()
                } else {
                    v.normalize()
                }
            }
        }
    }

    /// Screen-up direction of the view: +Z, except looking straight down (+Y up) or up (−Y up).
    pub fn up(self) -> Vec3 {
        let d = self.eye_direction();
        if d.x == 0.0 && d.y == 0.0 {
            if d.z > 0.0 { Vec3::Y } else { Vec3::NEG_Y }
        } else {
            Vec3::Z
        }
    }

    /// The face of the cube whose outward normal is `sign · axis` (0 = X, 1 = Y, 2 = Z).
    fn face(axis: usize, positive: bool) -> StandardView {
        FACES[axis * 2 + usize::from(positive)]
    }

    /// Axis and sign of a face view; `None` for Home and Iso.
    fn axis(self) -> Option<(usize, bool)> {
        FACES
            .iter()
            .position(|f| *f == self)
            .map(|i| (i / 2, i % 2 == 1))
    }
}

/// How orbiting treats the pole.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum OrbitMode {
    /// Yaw about world Z and pitch about the camera right axis, unlimited (the camera may go
    /// over the pole and turn upside down).
    #[default]
    Free,
    /// Turntable: world Z stays up and the pitch is clamped to ±89.9°.
    Constrained,
}

/// What a drag does.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NavMode {
    /// Rotate about the pivot.
    Orbit,
    /// Move with the grabbed point under the cursor.
    Pan,
    /// Zoom about the drag start point (drag up zooms in).
    Zoom,
}

/// One of the 26 regions of the view cube, or its home button.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum CubeRegion {
    /// A face (`Front`, `Back`, `Left`, `Right`, `Top`, `Bottom`).
    Face(StandardView),
    /// The edge between two faces; `a` is on the lower axis (X faces, then Y, then Z).
    Edge {
        /// Face on the lower axis.
        a: StandardView,
        /// Face on the higher axis.
        b: StandardView,
    },
    /// A corner; each component is −1 or 1.
    Corner {
        /// X sign.
        x: i8,
        /// Y sign.
        y: i8,
        /// Z sign.
        z: i8,
    },
    /// The home button.
    Home,
}

impl CubeRegion {
    /// The 26 regions of the cube (6 faces, 12 edges, 8 corners), in a fixed order.
    pub fn all() -> Vec<CubeRegion> {
        let mut out = Vec::with_capacity(26);
        for x in -1i8..=1 {
            for y in -1i8..=1 {
                for z in -1i8..=1 {
                    if let Some(r) = CubeRegion::from_signs([x, y, z]) {
                        out.push(r);
                    }
                }
            }
        }
        out
    }

    /// The region touching the faces with non-zero signs `s` (each −1, 0 or 1).
    fn from_signs(s: [i8; 3]) -> Option<CubeRegion> {
        let faces: Vec<StandardView> = (0..3)
            .filter(|&i| s[i] != 0)
            .map(|i| StandardView::face(i, s[i] > 0))
            .collect();
        match faces.as_slice() {
            [f] => Some(CubeRegion::Face(*f)),
            [a, b] => Some(CubeRegion::Edge { a: *a, b: *b }),
            [_, _, _] => Some(CubeRegion::Corner {
                x: s[0],
                y: s[1],
                z: s[2],
            }),
            _ => None,
        }
    }

    /// The signs `(x, y, z)` of the region's outward direction; `None` for Home or an invalid
    /// region (an edge between parallel faces).
    pub fn signs(self) -> Option<[i8; 3]> {
        let mut s = [0i8; 3];
        let mut add = |v: StandardView| -> Option<()> {
            let (axis, positive) = v.axis()?;
            if s[axis] != 0 {
                return None;
            }
            s[axis] = if positive { 1 } else { -1 };
            Some(())
        };
        match self {
            CubeRegion::Face(f) => add(f)?,
            CubeRegion::Edge { a, b } => {
                add(a)?;
                add(b)?;
            }
            CubeRegion::Corner { x, y, z } => s = [x.signum(), y.signum(), z.signum()],
            CubeRegion::Home => return None,
        }
        (s != [0; 3]).then_some(s)
    }
}

/// The view a click on `r` animates to.
pub fn view_for_region(r: CubeRegion) -> StandardView {
    match r {
        CubeRegion::Face(f) => f,
        CubeRegion::Home => StandardView::Home,
        r => match r.signs() {
            Some([x, y, z]) => StandardView::Iso { x, y, z },
            None => StandardView::Home,
        },
    }
}

/// The orthographic camera of the view-cube inset: looks at the cube `[−1, 1]³` (cube units)
/// with the orientation of `cam`.
pub fn view_cube_camera(cam: &Camera) -> Camera {
    let (_, up, forward) = cam.basis();
    Camera {
        eye: -forward * CUBE_EYE_DISTANCE,
        target: Point3::ZERO,
        up: Dir3::new_unchecked(up),
        projection: Projection::Orthographic {
            height: CUBE_VIEW_HEIGHT,
        },
    }
}

/// Hit test in the view-cube inset (`px` in physical pixels relative to the inset's top-left,
/// for a square inset `inset_px` wide): the region under `px`, `None` off the cube.
pub fn view_cube_hit(cam: &Camera, px: [f64; 2], inset_px: u32) -> Option<CubeRegion> {
    let cube = view_cube_camera(cam);
    let ray = cube.ray(px, [inset_px, inset_px]);
    let (o, d) = (ray.origin, ray.dir.get());
    let (mut t0, mut t1) = (f64::NEG_INFINITY, f64::INFINITY);
    for i in 0..3 {
        if d[i].abs() < 1e-300 {
            if o[i].abs() > 1.0 {
                return None;
            }
            continue;
        }
        let (a, b) = ((-1.0 - o[i]) / d[i], (1.0 - o[i]) / d[i]);
        t0 = t0.max(a.min(b));
        t1 = t1.min(a.max(b));
    }
    if t0 > t1 || t1 < 0.0 {
        return None;
    }
    let hit = o + d * t0.max(0.0);
    CubeRegion::from_signs(cube_signs(hit))
}

/// Signs of the faces a point of the cube surface touches.
fn cube_signs(p: Vec3) -> [i8; 3] {
    [p.x, p.y, p.z].map(|c| {
        if c > CUBE_REGION_EDGE {
            1
        } else if c < -CUBE_REGION_EDGE {
            -1
        } else {
            0
        }
    })
}

/// Layout and style of the view-cube inset (`View::view_cube`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViewCube {
    /// Side of the square inset, physical px (design note §13: 110 logical px).
    pub size_px: u32,
    /// Gap to the top and right view borders, physical px.
    pub margin_px: u32,
    /// Font size of the face letters, egui points.
    pub label_pt: f32,
    /// Region drawn in the hover colour.
    pub hover: Option<CubeRegion>,
}

impl Default for ViewCube {
    /// [`ViewCube::for_scale`] at 1 pixel per point.
    fn default() -> Self {
        ViewCube::for_scale(1.0)
    }
}

/// Linear RGB of a face centre, an edge or corner, the hovered region and the cube edges.
const CUBE_FACE_RGB: [f32; 3] = [0.62, 0.64, 0.68];
const CUBE_RIM_RGB: [f32; 3] = [0.48, 0.50, 0.54];
const CUBE_HOVER_RGB: [f32; 3] = [0.10, 0.36, 0.85];
const CUBE_EDGE_RGBA: [f32; 4] = [0.05, 0.06, 0.07, 1.0];
const CUBE_LABEL_RGBA: [f32; 4] = [0.02, 0.02, 0.03, 1.0];

impl ViewCube {
    /// The default cube for a display scale: 110 points wide, 10 points from the borders.
    pub fn for_scale(pixels_per_point: f32) -> ViewCube {
        let ppp = if pixels_per_point.is_finite() && pixels_per_point > 0.0 {
            pixels_per_point
        } else {
            1.0
        };
        ViewCube {
            size_px: (110.0 * ppp).round() as u32,
            margin_px: (10.0 * ppp).round() as u32,
            label_pt: 9.0,
            hover: None,
        }
    }

    /// `[x, y, width, height]` of the inset in a view of `view_size_px` (top-right corner).
    pub fn rect_px(&self, view_size_px: [u32; 2]) -> [u32; 4] {
        let n = self.size_px.min(view_size_px[0]).min(view_size_px[1]);
        let x = view_size_px[0].saturating_sub(n + self.margin_px);
        let y = self.margin_px.min(view_size_px[1].saturating_sub(n));
        [x, y, n, n]
    }

    /// The overlay inset of the cube for a view with camera `cam` and size `view_size_px`:
    /// shaded face, edge and corner quads, the cube edges and the face letters (labels shown when
    /// the face looks at the viewer, `n · view > 0.25`).
    pub fn inset(&self, cam: &Camera, view_size_px: [u32; 2]) -> OverlayInset {
        let camera = view_cube_camera(cam);
        let toward_eye = -camera.forward().get();
        let mut overlays = OverlayList::new();
        let bounds = [-1.0, -CUBE_REGION_EDGE, CUBE_REGION_EDGE, 1.0];
        for axis in 0..3 {
            let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
            for positive in [false, true] {
                let s = if positive { 1.0 } else { -1.0 };
                let mut n = Vec3::ZERO;
                n[axis] = s;
                let facing = n.dot(toward_eye);
                if facing <= 0.0 {
                    continue; // hidden by the cube itself
                }
                let light = (0.55 + 0.45 * facing) as f32;
                for iu in 0..3 {
                    for iv in 0..3 {
                        let corner = |a: usize, b: usize| {
                            let mut p = n;
                            p[u] = bounds[iu + a];
                            p[v] = bounds[iv + b];
                            p
                        };
                        let pts = [corner(0, 0), corner(1, 0), corner(1, 1), corner(0, 1)];
                        let centre = (pts[0] + pts[2]) * 0.5;
                        let region = CubeRegion::from_signs(cube_signs(centre));
                        let rgb = if region.is_some() && region == self.hover {
                            CUBE_HOVER_RGB
                        } else if iu == 1 && iv == 1 {
                            CUBE_FACE_RGB
                        } else {
                            CUBE_RIM_RGB
                        };
                        overlays.polygon(
                            &pts,
                            &[[0, 1, 2], [0, 2, 3]],
                            [rgb[0] * light, rgb[1] * light, rgb[2] * light, 1.0],
                            DepthMode::AlwaysOnTop,
                            None,
                        );
                    }
                }
                if facing > CUBE_LABEL_MIN_FACING {
                    let face = StandardView::face(axis, positive);
                    overlays.label(
                        n,
                        face_label(face),
                        LabelStyle {
                            size_pt: self.label_pt,
                            color: CUBE_LABEL_RGBA,
                            offset_px: [0.0, 0.0],
                            background: None,
                        },
                    );
                }
            }
        }
        let edge = LineStyle {
            width_px: 1.2,
            color: CUBE_EDGE_RGBA,
            dash: None,
            depth: DepthMode::AlwaysOnTop,
        };
        for axis in 0..3 {
            let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
            for (su, sv) in [(-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)] {
                let mut n = Vec3::ZERO;
                n[u] = su;
                n[v] = sv;
                // Only edges of a visible face.
                let (mut nu, mut nv) = (Vec3::ZERO, Vec3::ZERO);
                nu[u] = su;
                nv[v] = sv;
                if nu.dot(toward_eye) <= 0.0 && nv.dot(toward_eye) <= 0.0 {
                    continue;
                }
                let (mut a, mut b) = (n, n);
                a[axis] = -1.0;
                b[axis] = 1.0;
                overlays.polyline(&[a, b], edge, None);
            }
        }
        OverlayInset {
            rect_px: self.rect_px(view_size_px),
            camera,
            overlays,
        }
    }
}

/// The face letters (Fusion style).
fn face_label(v: StandardView) -> &'static str {
    match v {
        StandardView::Front => "FRONT",
        StandardView::Back => "BACK",
        StandardView::Left => "LEFT",
        StandardView::Right => "RIGHT",
        StandardView::Top => "TOP",
        StandardView::Bottom => "BOTTOM",
        StandardView::Home | StandardView::Iso { .. } => "",
    }
}

/// Holds drag state and the running transition. Inputs in physical pixels.
#[derive(Clone, Debug, Default)]
pub struct NavController {
    orbit_mode: OrbitMode,
    drag: Option<Drag>,
    transition: Option<Transition>,
}

#[derive(Clone, Copy, Debug)]
struct Drag {
    mode: NavMode,
    /// The camera at drag start; every drag step is computed from it (no drift).
    start: Camera,
    start_px: [f64; 2],
    /// Orbit pivot: the anchor, else the scene centre, else the target.
    pivot: Point3,
    /// The pick point under the cursor at drag start.
    anchor: Option<Point3>,
}

#[derive(Clone, Copy, Debug)]
struct Transition {
    from: Camera,
    to: Camera,
    elapsed: Duration,
}

impl NavController {
    /// A controller with free orbit and nothing running.
    pub fn new() -> NavController {
        NavController::default()
    }

    /// Starts a drag. `anchor` = world point under the cursor from a pick (`None` on
    /// background); `scene` = the visible bounds (the orbit pivot on background). Stops a
    /// running transition where it is.
    pub fn begin(
        &mut self,
        mode: NavMode,
        cam: &Camera,
        cursor_px: [f64; 2],
        anchor: Option<Point3>,
        scene: Aabb3,
    ) {
        self.transition = None;
        let pivot = anchor.unwrap_or(if scene.is_empty() {
            cam.target
        } else {
            scene.center()
        });
        self.drag = Some(Drag {
            mode,
            start: *cam,
            start_px: cursor_px,
            pivot,
            anchor,
        });
    }

    /// Moves the drag to `cursor_px` in a view of `size_px` (from the drag start, so the result
    /// depends only on the total cursor offset). Does nothing without [`NavController::begin`].
    pub fn drag(&mut self, cam: &mut Camera, cursor_px: [f64; 2], size_px: [u32; 2]) {
        let Some(d) = self.drag else {
            return;
        };
        let h = f64::from(size_px[1].max(1));
        let (dx, dy) = (cursor_px[0] - d.start_px[0], cursor_px[1] - d.start_px[1]);
        match d.mode {
            NavMode::Orbit => {
                let yaw = -ORBIT_RAD_PER_HEIGHT * dx / h;
                let pitch = -ORBIT_RAD_PER_HEIGHT * dy / h;
                *cam = orbit(&d.start, d.pivot, yaw, pitch, self.orbit_mode);
            }
            NavMode::Pan => {
                let grab = d
                    .anchor
                    .unwrap_or_else(|| target_plane_point(&d.start, d.start_px, size_px));
                let forward = d.start.forward().get();
                let depth = (grab - d.start.eye).dot(forward);
                let ray = d.start.ray(cursor_px, size_px);
                let dir = ray.dir.get();
                let t = (depth - (ray.origin - d.start.eye).dot(forward)) / dir.dot(forward);
                let delta = grab - (ray.origin + dir * t);
                *cam = Camera {
                    eye: d.start.eye + delta,
                    target: d.start.target + delta,
                    ..d.start
                };
            }
            NavMode::Zoom => {
                let at = d
                    .anchor
                    .unwrap_or_else(|| target_plane_point(&d.start, d.start_px, size_px));
                *cam = zoom_about(&d.start, at, -dy / ZOOM_DRAG_PX_PER_STEP);
            }
        }
    }

    /// Ends the drag.
    pub fn end(&mut self) {
        self.drag = None;
    }

    /// True between [`NavController::begin`] and [`NavController::end`].
    pub fn is_dragging(&self) -> bool {
        self.drag.is_some()
    }

    /// The orbit pivot of the running drag (for a pivot marker).
    pub fn pivot(&self) -> Option<Point3> {
        self.drag.map(|d| d.pivot)
    }

    /// Zooms by `steps` wheel steps (positive = in, factor `0.9^steps`) about the world point
    /// under the cursor: `anchor` from a pick, or the cursor ray on the plane through the target
    /// facing the camera. The anchor keeps its pixel.
    pub fn zoom(
        &mut self,
        cam: &mut Camera,
        steps: f64,
        cursor_px: [f64; 2],
        anchor: Option<Point3>,
        size_px: [u32; 2],
    ) {
        self.transition = None;
        let at = anchor.unwrap_or_else(|| target_plane_point(cam, cursor_px, size_px));
        *cam = zoom_about(cam, at, steps);
    }

    /// Fits `bounds` (bounding sphere spanning about 95 % of the smaller view dimension), keeping
    /// the orientation. Empty bounds do nothing.
    pub fn fit(&mut self, cam: &mut Camera, bounds: Aabb3, size_px: [u32; 2], animate: bool) {
        let to = fitted(cam, bounds, size_px);
        self.go(cam, to, animate);
    }

    /// Turns to a standard view and fits `bounds` (empty bounds keep the target and distance).
    pub fn set_view(
        &mut self,
        cam: &mut Camera,
        v: StandardView,
        bounds: Aabb3,
        size_px: [u32; 2],
        animate: bool,
    ) {
        let oriented = oriented(cam, v.eye_direction(), v.up(), cam.target);
        let to = fitted(&oriented, bounds, size_px);
        self.go(cam, to, animate);
    }

    /// Switches the lens and keeps the apparent size at the target: to orthographic, the height
    /// becomes `2 · |eye − target| · tan(fov_y / 2)` (the given height is ignored); to
    /// perspective with `fov_y`, the eye moves along the view axis to distance
    /// `height / (2 tan(fov_y / 2))`. Not animated.
    pub fn set_projection(&mut self, cam: &mut Camera, p: Projection) {
        self.transition = None;
        let size = apparent_height(cam);
        let forward = cam.forward().get();
        match p {
            Projection::Orthographic { .. } => {
                cam.projection = Projection::Orthographic { height: size };
            }
            Projection::Perspective { fov_y } => {
                let dist = size / (2.0 * (fov_y * 0.5).tan());
                cam.eye = cam.target - forward * dist;
                cam.projection = Projection::Perspective { fov_y };
            }
        }
    }

    /// Sketch "look at": views along −`plane.normal` (the normal toward the viewer) with
    /// `plane.x` to the right and `plane.y` up, fitting `bounds` (empty bounds: target at the
    /// plane origin, distance kept).
    pub fn look_at_plane(
        &mut self,
        cam: &mut Camera,
        plane: &Plane,
        bounds: Aabb3,
        size_px: [u32; 2],
        animate: bool,
    ) {
        let target = if bounds.is_empty() {
            plane.origin
        } else {
            cam.target
        };
        let oriented = oriented(cam, plane.normal.get(), plane.y.get(), target);
        let to = fitted(&oriented, bounds, size_px);
        self.go(cam, to, animate);
    }

    /// Sets the orbit mode for the next drags.
    pub fn set_orbit_mode(&mut self, m: OrbitMode) {
        self.orbit_mode = m;
    }

    /// The orbit mode.
    pub fn orbit_mode(&self) -> OrbitMode {
        self.orbit_mode
    }

    /// True while a transition runs.
    pub fn is_animating(&self) -> bool {
        self.transition.is_some()
    }

    /// Advances transitions by `dt` and writes the camera; returns true while animating
    /// (request a repaint). The last step lands exactly on the destination.
    pub fn tick(&mut self, cam: &mut Camera, dt: Duration) -> bool {
        let Some(t) = self.transition.as_mut() else {
            return false;
        };
        t.elapsed += dt;
        let s = (t.elapsed.as_secs_f64() / TRANSITION.as_secs_f64()).min(1.0);
        if s >= 1.0 {
            *cam = t.to;
            self.transition = None;
            return false;
        }
        *cam = interpolate(&t.from, &t.to, smoothstep(s));
        true
    }

    /// Jumps to `to` or animates there.
    fn go(&mut self, cam: &mut Camera, to: Camera, animate: bool) {
        self.drag = None;
        if animate && to != *cam {
            self.transition = Some(Transition {
                from: *cam,
                to,
                elapsed: Duration::ZERO,
            });
        } else {
            self.transition = None;
            *cam = to;
        }
    }
}

/// Rotates `start` rigidly about `pivot`: yaw about world Z, pitch about the camera right axis
/// (positive pitch turns the view direction up). Rigid, so `pivot` keeps its pixel.
fn orbit(start: &Camera, pivot: Point3, yaw: f64, pitch: f64, mode: OrbitMode) -> Camera {
    let (right, up, forward) = start.basis();
    let (rot, new_up) = match mode {
        OrbitMode::Free => {
            let rot = Quat::from_axis_angle(Vec3::Z, yaw) * Quat::from_axis_angle(right, pitch);
            (rot, rot * up)
        }
        OrbitMode::Constrained => {
            let horizontal = forward.cross(Vec3::Z);
            let axis = if horizontal.length_squared() > 1e-24 {
                horizontal.normalize()
            } else {
                right
            };
            let elevation = forward.z.clamp(-1.0, 1.0).asin();
            let to = (elevation + pitch).clamp(-CONSTRAINED_PITCH_LIMIT, CONSTRAINED_PITCH_LIMIT);
            let rot =
                Quat::from_axis_angle(Vec3::Z, yaw) * Quat::from_axis_angle(axis, to - elevation);
            (rot, Vec3::Z)
        }
    };
    Camera {
        eye: pivot + rot * (start.eye - pivot),
        target: pivot + rot * (start.target - pivot),
        up: Dir3::new_unchecked(new_up.normalize()),
        projection: start.projection,
    }
}

/// `cam` zoomed by `0.9^steps` about `at`, which keeps its pixel: perspective scales eye and
/// target toward `at`; orthographic scales the height and moves the target in the view plane.
fn zoom_about(cam: &Camera, at: Point3, steps: f64) -> Camera {
    let f = ZOOM_STEP.powf(steps);
    match cam.projection {
        Projection::Perspective { .. } => Camera {
            eye: at + (cam.eye - at) * f,
            target: at + (cam.target - at) * f,
            ..*cam
        },
        Projection::Orthographic { height } => {
            let forward = cam.forward().get();
            // In the view plane only: the depth of the target does not change the image.
            let lateral = |v: Vec3| v - forward * v.dot(forward);
            let shift = lateral(at - cam.target) * (1.0 - f);
            Camera {
                eye: cam.eye + shift,
                target: cam.target + shift,
                projection: Projection::Orthographic { height: height * f },
                ..*cam
            }
        }
    }
}

/// The cursor ray of `cam` on the plane through the target facing the camera.
fn target_plane_point(cam: &Camera, cursor_px: [f64; 2], size_px: [u32; 2]) -> Point3 {
    let ray = cam.ray(cursor_px, size_px);
    let forward = cam.forward().get();
    let denom = ray.dir.get().dot(forward);
    if denom.abs() < 1e-12 {
        return cam.target;
    }
    let t = (cam.target - ray.origin).dot(forward) / denom;
    ray.at(t)
}

/// World height of the view at the target, mm.
fn apparent_height(cam: &Camera) -> f64 {
    match cam.projection {
        Projection::Perspective { fov_y } => {
            2.0 * cam.eye.distance(cam.target) * (fov_y * 0.5).tan()
        }
        Projection::Orthographic { height } => height,
    }
}

/// `cam` looking at `target` from direction `eye_dir`, at the same distance, with `up`.
fn oriented(cam: &Camera, eye_dir: Vec3, up: Vec3, target: Point3) -> Camera {
    let dist = cam.eye.distance(cam.target).max(1e-9);
    Camera {
        eye: target + eye_dir.normalize() * dist,
        target,
        up: Dir3::new_unchecked(up.normalize()),
        projection: cam.projection,
    }
}

/// `cam` moved (orientation kept) so the bounding sphere of `bounds` fills the view (design note
/// §13). Empty bounds return `cam`.
fn fitted(cam: &Camera, bounds: Aabb3, size_px: [u32; 2]) -> Camera {
    if bounds.is_empty() {
        return *cam;
    }
    let centre = bounds.center();
    let r = (bounds.diagonal() * 0.5).max(MIN_FIT_RADIUS);
    let aspect = f64::from(size_px[0].max(1)) / f64::from(size_px[1].max(1));
    let forward = cam.forward().get();
    match cam.projection {
        Projection::Perspective { fov_y } => {
            let fov_x = 2.0 * ((fov_y * 0.5).tan() * aspect).atan();
            let dist = FIT_MARGIN * r / (fov_x.min(fov_y) * 0.5).sin();
            Camera {
                eye: centre - forward * dist,
                target: centre,
                ..*cam
            }
        }
        Projection::Orthographic { .. } => {
            let height = ORTHO_FIT * r / aspect.min(1.0);
            Camera {
                eye: centre - forward * (ORTHO_FIT * r),
                target: centre,
                projection: Projection::Orthographic { height },
                ..*cam
            }
        }
    }
}

fn smoothstep(t: f64) -> f64 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Camera orientation as a quaternion (columns right, up, back).
fn orientation(cam: &Camera) -> Quat {
    let (right, up, forward) = cam.basis();
    Quat::from_mat3(&Mat3::from_cols(right, up, -forward)).normalize()
}

/// The transition pose at `s` in `[0, 1]`: orientation slerp, target lerp, log-distance and
/// log-height lerp.
fn interpolate(from: &Camera, to: &Camera, s: f64) -> Camera {
    let rot = orientation(from).slerp(orientation(to), s);
    let target = from.target.lerp(to.target, s);
    let log_lerp = |a: f64, b: f64| (a.max(1e-300).ln() * (1.0 - s) + b.max(1e-300).ln() * s).exp();
    let dist = log_lerp(from.eye.distance(from.target), to.eye.distance(to.target));
    let projection = match (from.projection, to.projection) {
        (Projection::Perspective { fov_y: a }, Projection::Perspective { fov_y: b }) => {
            Projection::Perspective {
                fov_y: a + (b - a) * s,
            }
        }
        (Projection::Orthographic { height: a }, Projection::Orthographic { height: b }) => {
            Projection::Orthographic {
                height: log_lerp(a, b),
            }
        }
        (a, b) => {
            if s < 0.5 {
                a
            } else {
                b
            }
        }
    };
    Camera {
        eye: target + rot * Vec3::Z * dist,
        target,
        up: Dir3::new_unchecked((rot * Vec3::Y).normalize()),
        projection,
    }
}

#[cfg(test)]
mod tests;
