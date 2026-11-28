//! Overlays (RND-07, design note §12): gizmos, helpers, sketch geometry and labels drawn after the
//! scene into the final image, with their own depth buffer.
//!
//! - [`OverlayList`] is immediate mode: the active tool refills it every frame
//!   ([`crate::RenderScene::begin_overlays`]). It holds polylines, arrows, rings, triads, plane
//!   quads, an adaptive grid, image planes, points, sprites, filled polygons and labels.
//! - **Constant pixel size.** Arrows, rings, triads and sprites are tessellated on the CPU per
//!   view and frame with `size_px × Camera::pixel_size_at(anchor)` (f64), into eye-relative f32
//!   coordinates, so the size holds at any view distance.
//! - **Reuse.** Polylines (and the shafts, rings and borders built from the primitives) go
//!   through RND-12's segment builder and wide-line vertex and fragment stages
//!   (`line_vertex_slot`, `line_color` in `shaders/lines.wgsl`); points through its point-sprite
//!   stages. One instance record + line record per primitive carries its style; the per-segment
//!   id buffer holds the record slot, so all overlay lines are one draw.
//! - **Depth.** The overlay pass has its own depth target (cleared each frame) so gizmos occlude
//!   themselves; [`DepthMode`] against the scene is evaluated in the fragment shader from the
//!   scene depth target: `Test` hides, `XRay` dims to `hidden_alpha`, `AlwaysOnTop` ignores it.
//! - **Picking.** Primitives with a handle `(PickId, u32)` are drawn into RND-04's point-pick
//!   pass as [`crate::SubKind::Handle`] with that sub-id (always-on-top and x-ray handles win over
//!   the scene).
//! - **Labels** are not drawn by the GPU: [`crate::Renderer::project_labels`] projects their
//!   anchors and `egui_glue::paint_labels` paints them with egui (design note §12).
//! - **Insets** ([`OverlayInset`], `View::insets`) are overlay lists drawn in a sub-rectangle
//!   with their own camera, over everything: the rendering hook of the view cube (RND-11).

use std::sync::OnceLock;

use amg_core::{Dir3, Plane, Point3, Transform, Vec3};
use amg_jobs::JobCtx;
use amg_jobs::gpu::Gpu;

use crate::camera::Camera;
use crate::highlight::{HighlightItem, HighlightLayer};
use crate::lines::{
    DepthMode, EdgeData, LineRecord, LineStyle, PointStyle, Polyline, build_segments,
};
use crate::pick::SubKind;
use crate::renderer::RgbaImage;
use crate::scene::InstanceRecord;
use crate::{ImageKey, PickId, RenderError};

/// Depth-mode codes of the overlay shaders (`MODE_*` in `shaders/overlay.wgsl`).
const MODE_TEST: f32 = 0.0;
const MODE_ALWAYS: f32 = 1.0;
const MODE_XRAY: f32 = 2.0;

/// Triangle kinds (`KIND_*` in `shaders/overlay.wgsl`).
const KIND_FLAT: f32 = 0.0;
const KIND_IMAGE: f32 = 1.0;
const KIND_GRID: f32 = 2.0;

/// Segments of a ring (closed polyline).
const RING_SEGMENTS: usize = 64;
/// Facets of an arrow-head cone.
const CONE_SEGMENTS: usize = 16;
/// Arrow-head length as a fraction of the arrow length.
const HEAD_FRACTION: f64 = 0.25;
/// Arrow-head radius as a fraction of the head length.
const HEAD_RADIUS: f64 = 0.35;
/// Width of the triad arms, px.
const TRIAD_WIDTH_PX: f32 = 2.0;
/// Width of the grid lines, px.
const GRID_LINE_PX: f32 = 1.0;
/// Size of the grid's mesh cells at its anchor, in pixel sizes there; cells grow by
/// [`GRID_CELL_GROWTH`] per step away from it.
const GRID_CELL_PX: f64 = 64.0;
const GRID_CELL_GROWTH: f64 = 8.0;
/// The grid offset is snapped to a multiple of this many pixel sizes (at the plane point nearest
/// the camera target), so plane coordinates stay small in f32 near the camera.
const GRID_SNAP_PX: f64 = 1e4;

/// Linear RGB of the triad axes X, Y, Z.
const TRIAD_COLORS: [[f32; 4]; 3] = [
    [0.80, 0.08, 0.06, 1.0],
    [0.10, 0.62, 0.08, 1.0],
    [0.06, 0.20, 0.85, 1.0],
];

/// How a label is painted (egui, `egui_glue::paint_labels`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LabelStyle {
    /// Font size, egui points.
    pub size_pt: f32,
    /// Linear RGBA of the text.
    pub color: [f32; 4],
    /// Offset of the text centre from the projected anchor, physical px (x right, y down).
    pub offset_px: [f32; 2],
    /// Linear RGBA of a rounded background box; `None` = no box.
    pub background: Option<[f32; 4]>,
}

impl Default for LabelStyle {
    /// 13 pt, near-white, no offset, no background.
    fn default() -> Self {
        LabelStyle {
            size_pt: 13.0,
            color: [0.9, 0.9, 0.9, 1.0],
            offset_px: [0.0, 0.0],
            background: None,
        }
    }
}

/// A label anchor projected into a view (from [`crate::Renderer::project_labels`]).
#[derive(Clone, Debug, PartialEq)]
pub struct ProjectedLabel {
    /// Text centre in physical pixels of the view (origin top-left, y down): the projected
    /// anchor plus [`LabelStyle::offset_px`].
    pub pos_px: [f32; 2],
    /// Reversed-Z depth of the anchor (1 at the near plane).
    pub depth: f32,
    /// The text.
    pub text: String,
    /// Style.
    pub style: LabelStyle,
}

/// An overlay list drawn in a sub-rectangle of a view with its own camera, over the scene and
/// the view's own overlays (no scene depth test): the rendering hook of the view cube (RND-11).
#[derive(Clone, Debug)]
pub struct OverlayInset {
    /// `[x, y, width, height]` in physical pixels of the view (origin top-left).
    pub rect_px: [u32; 4],
    /// Camera of the inset (typically orthographic, sharing the view's orientation).
    pub camera: Camera,
    /// What to draw. Depth modes are ignored (always on top of the scene); the inset has its
    /// own depth so its geometry occludes itself.
    pub overlays: OverlayList,
}

/// One overlay primitive. Point runs index [`OverlayList::points`].
#[derive(Clone, Debug)]
enum Item {
    Polyline {
        pts: (u32, u32),
        style: LineStyle,
        handle: Option<(PickId, u32)>,
    },
    Arrow {
        origin: Point3,
        dir: Dir3,
        length_px: f32,
        style: LineStyle,
        handle: Option<(PickId, u32)>,
    },
    Ring {
        center: Point3,
        normal: Dir3,
        radius_px: f32,
        style: LineStyle,
        handle: Option<(PickId, u32)>,
    },
    Triad {
        frame: Transform,
        size_px: f32,
        handle_base: Option<PickId>,
    },
    PlaneQuad {
        plane: Plane,
        half_size: [f64; 2],
        fill: [f32; 4],
        border: LineStyle,
        handle: Option<(PickId, u32)>,
    },
    Grid {
        plane: Plane,
        extent: [f64; 2],
        color: [f32; 4],
    },
    Image {
        img: ImageKey,
        corners: [Point3; 4],
        opacity: f32,
    },
    Points {
        pts: (u32, u32),
        style: PointStyle,
        handle: Option<(PickId, u32)>,
    },
    Sprite {
        at: Point3,
        img: ImageKey,
        uv: [[f32; 2]; 2],
        size_px: f32,
        handle: Option<(PickId, u32)>,
    },
    Polygon {
        pts: (u32, u32),
        tris: (u32, u32),
        fill: [f32; 4],
        depth: DepthMode,
        handle: Option<(PickId, u32)>,
    },
}

#[derive(Clone, Debug)]
struct Label {
    anchor: Point3,
    text: String,
    style: LabelStyle,
}

/// Immediate-mode overlays: cleared by [`crate::RenderScene::begin_overlays`] and refilled by the
/// active tool every frame. Sizes in `_px` are physical pixels and stay constant on screen.
#[derive(Clone, Debug, Default)]
pub struct OverlayList {
    items: Vec<Item>,
    points: Vec<Point3>,
    tris: Vec<[u32; 3]>,
    labels: Vec<Label>,
}

impl OverlayList {
    /// An empty list.
    pub fn new() -> OverlayList {
        OverlayList::default()
    }

    /// Removes everything (keeps the allocations).
    pub fn clear(&mut self) {
        self.items.clear();
        self.points.clear();
        self.tris.clear();
        self.labels.clear();
    }

    /// No primitives and no labels.
    pub fn is_empty(&self) -> bool {
        self.items.is_empty() && self.labels.is_empty()
    }

    /// Number of primitives (labels not counted).
    pub fn primitive_count(&self) -> usize {
        self.items.len()
    }

    /// Number of labels.
    pub fn label_count(&self) -> usize {
        self.labels.len()
    }

    /// Extent `[lo, hi]` along `forward` (mm from `eye`) of the world anchors and corners of the
    /// primitives: what an orthographic view's fitted depth range must hold (RND-11). Sizes in
    /// pixels are not included; the fitted range adds a margin for them.
    pub(crate) fn depth_extent(&self, eye: Point3, forward: Vec3) -> Option<(f64, f64)> {
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        let mut add = |p: Point3| {
            let d = (p - eye).dot(forward);
            lo = lo.min(d);
            hi = hi.max(d);
        };
        let plane_corners = |plane: &Plane, h: [f64; 2]| {
            [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)].map(|(u, v)| {
                plane.origin + plane.x.get() * (u * h[0]) + plane.y.get() * (v * h[1])
            })
        };
        for item in &self.items {
            match item {
                Item::Polyline { .. } | Item::Points { .. } | Item::Polygon { .. } => {}
                Item::Arrow { origin, .. } => add(*origin),
                Item::Ring { center, .. } => add(*center),
                Item::Triad { frame, .. } => add(frame.translation),
                Item::Sprite { at, .. } => add(*at),
                Item::Image { corners, .. } => corners.iter().copied().for_each(&mut add),
                Item::PlaneQuad {
                    plane, half_size, ..
                } => plane_corners(plane, *half_size)
                    .into_iter()
                    .for_each(&mut add),
                Item::Grid { plane, extent, .. } => {
                    plane_corners(plane, *extent).into_iter().for_each(&mut add);
                }
            }
        }
        // Polylines, point sets and polygons keep their points in the shared list.
        self.points.iter().copied().for_each(&mut add);
        (lo <= hi).then_some((lo, hi))
    }

    fn push_points(&mut self, pts: &[Point3]) -> (u32, u32) {
        let first = self.points.len() as u32;
        self.points.extend_from_slice(pts);
        (first, pts.len() as u32)
    }

    /// An open polyline through `pts` (world, mm). `handle` makes it pickable.
    pub fn polyline(&mut self, pts: &[Point3], style: LineStyle, handle: Option<(PickId, u32)>) {
        if pts.len() < 2 {
            return;
        }
        let pts = self.push_points(pts);
        self.items.push(Item::Polyline { pts, style, handle });
    }

    /// An arrow from `origin` along `dir`, `length_px` long on screen (shaft in `style`, a cone
    /// head a quarter of the length).
    pub fn arrow(
        &mut self,
        origin: Point3,
        dir: Dir3,
        length_px: f32,
        style: LineStyle,
        handle: Option<(PickId, u32)>,
    ) {
        self.items.push(Item::Arrow {
            origin,
            dir,
            length_px,
            style,
            handle,
        });
    }

    /// A circle around `center` in the plane normal to `normal`, of radius `radius_px` on
    /// screen (at the centre's depth).
    pub fn ring(
        &mut self,
        center: Point3,
        normal: Dir3,
        radius_px: f32,
        style: LineStyle,
        handle: Option<(PickId, u32)>,
    ) {
        self.items.push(Item::Ring {
            center,
            normal,
            radius_px,
            style,
            handle,
        });
    }

    /// Three arrows along the frame's x, y, z axes (red, green, blue), `size_px` long, always on
    /// top. Handles: `(handle_base, 0 | 1 | 2)` for x, y, z.
    pub fn triad(&mut self, frame: &Transform, size_px: f32, handle_base: Option<PickId>) {
        self.items.push(Item::Triad {
            frame: *frame,
            size_px,
            handle_base,
        });
    }

    /// A filled rectangle `plane.origin ± half_size` along the plane axes, with a border. The
    /// fill uses the border's depth mode; it writes overlay depth when opaque.
    pub fn plane_quad(
        &mut self,
        plane: &Plane,
        half_size: [f64; 2],
        fill: [f32; 4],
        border: LineStyle,
        handle: Option<(PickId, u32)>,
    ) {
        self.items.push(Item::PlaneQuad {
            plane: *plane,
            half_size,
            fill,
            border,
            handle,
        });
    }

    /// An adaptive grid on `plane` over `plane.origin ± extent` (half sizes, mm): lines at
    /// powers of ten, chosen per pixel so minor lines are at least 8 px apart, fading in up to
    /// 40 px, with every tenth line as the major line. Depth tested against the scene.
    pub fn grid(&mut self, plane: &Plane, extent: [f64; 2], color: [f32; 4]) {
        self.items.push(Item::Grid {
            plane: *plane,
            extent,
            color,
        });
    }

    /// An image on the quad `corners` (top-left, top-right, bottom-right, bottom-left of the
    /// image: row 0 of the [`RgbaImage`] runs from `corners[0]` to `corners[1]`). Depth tested.
    pub fn image_plane(&mut self, img: ImageKey, corners: [Point3; 4], opacity: f32) {
        self.items.push(Item::Image {
            img,
            corners,
            opacity,
        });
    }

    /// Point sprites at `pts`.
    pub fn points(&mut self, pts: &[Point3], style: PointStyle, handle: Option<(PickId, u32)>) {
        if pts.is_empty() {
            return;
        }
        let pts = self.push_points(pts);
        self.items.push(Item::Points { pts, style, handle });
    }

    /// A screen-aligned square `size_px` wide centred on `at`, showing the sub-rectangle `uv`
    /// (`[min, max]`, texture coordinates with v down) of `img` (icon atlas glyphs, R-S10-7).
    /// Always on top.
    pub fn sprite(
        &mut self,
        at: Point3,
        img: ImageKey,
        uv: [[f32; 2]; 2],
        size_px: f32,
        handle: Option<(PickId, u32)>,
    ) {
        self.items.push(Item::Sprite {
            at,
            img,
            uv,
            size_px,
            handle,
        });
    }

    /// A filled planar polygon (profiles, regions) triangulated by the caller (`tris` index
    /// `pts`; out-of-range triangles are skipped). Writes overlay depth when opaque.
    pub fn polygon(
        &mut self,
        pts: &[Point3],
        tris: &[[u32; 3]],
        fill: [f32; 4],
        depth: DepthMode,
        handle: Option<(PickId, u32)>,
    ) {
        if pts.is_empty() || tris.is_empty() {
            return;
        }
        let pts = self.push_points(pts);
        let first = self.tris.len() as u32;
        self.tris.extend_from_slice(tris);
        self.items.push(Item::Polygon {
            pts,
            tris: (first, tris.len() as u32),
            fill,
            depth,
            handle,
        });
    }

    /// A screen-space text label at `anchor`, painted by egui (see
    /// [`crate::Renderer::project_labels`]).
    pub fn label(&mut self, anchor: Point3, text: impl Into<String>, style: LabelStyle) {
        self.labels.push(Label {
            anchor,
            text: text.into(),
            style,
        });
    }

    /// The labels projected with `cam` into a view of `size_px` (anchors behind the eye are
    /// dropped), shifted by `origin_px`.
    pub(crate) fn project_labels(
        &self,
        cam: &Camera,
        size_px: [u32; 2],
        origin_px: [f64; 2],
        out: &mut Vec<ProjectedLabel>,
    ) {
        for l in &self.labels {
            let Some([x, y, depth]) = cam.project(l.anchor, size_px) else {
                continue;
            };
            out.push(ProjectedLabel {
                pos_px: [
                    (x + origin_px[0] + f64::from(l.style.offset_px[0])) as f32,
                    (y + origin_px[1] + f64::from(l.style.offset_px[1])) as f32,
                ],
                depth: depth as f32,
                text: l.text.clone(),
                style: l.style,
            });
        }
    }
}

/// One overlay triangle vertex (`TriIn` in `shaders/overlay.wgsl`), 60 bytes.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub(crate) struct TriVertex {
    /// Eye-relative world position, mm.
    pub pos: [f32; 3],
    /// Linear RGBA (straight alpha), already shaded and highlighted.
    pub color: [f32; 4],
    /// Image coordinates, or plane coordinates (mm) of a grid.
    pub uv: [f32; 2],
    /// x = kind, y = depth mode, z = x-ray alpha, w = grid line width px.
    pub params: [f32; 4],
    /// Pick id and handle sub-id (pick id 0 = not pickable).
    pub pick: [u32; 2],
}

/// Byte stride of [`TriVertex`].
pub(crate) const TRI_VERTEX_BYTES: u64 = size_of::<TriVertex>() as u64;

/// A run of triangles drawn with one pipeline and image.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct TriDraw {
    /// First vertex.
    pub first: u32,
    /// Vertex count.
    pub count: u32,
    /// The image sampled (`None`: the white texture).
    pub image: Option<ImageKey>,
    /// Writes overlay depth (opaque).
    pub write: bool,
}

/// The draws of one overlay layer (the view's list, or one inset).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub(crate) struct LayerDraws {
    /// Segments (6 vertices each) in the shared segment buffer.
    pub segments: std::ops::Range<u32>,
    /// Points (6 vertices each) in the shared point buffer.
    pub points: std::ops::Range<u32>,
    /// Triangle draws (indices into [`OverlayGeometry::tri_draws`]).
    pub tri_draws: std::ops::Range<usize>,
    /// Some primitive of the layer has a handle.
    pub pickable: bool,
}

/// CPU geometry of all overlay layers of one view for one frame.
#[derive(Debug, Default)]
pub(crate) struct OverlayGeometry {
    pub records: Vec<InstanceRecord>,
    pub lines: Vec<LineRecord>,
    /// Two `[x, y, z, arc]` per segment (`build_segments`).
    pub segments: Vec<[f32; 4]>,
    /// Record slot of each segment.
    pub segment_slots: Vec<u32>,
    /// `[x, y, z, 0]` per point.
    pub points: Vec<[f32; 4]>,
    /// Record slot of each point.
    pub point_slots: Vec<u32>,
    pub tris: Vec<TriVertex>,
    pub tri_draws: Vec<TriDraw>,
    pub layers: Vec<LayerDraws>,
}

impl OverlayGeometry {
    pub fn clear(&mut self) {
        self.records.clear();
        self.lines.clear();
        self.segments.clear();
        self.segment_slots.clear();
        self.points.clear();
        self.point_slots.clear();
        self.tris.clear();
        self.tri_draws.clear();
        self.layers.clear();
    }

    /// Nothing to draw in any layer.
    pub fn is_empty(&self) -> bool {
        self.segment_slots.is_empty() && self.point_slots.is_empty() && self.tris.is_empty()
    }
}

/// Highlight state the tessellation applies to handles: the scene's layers and colours.
pub(crate) struct HandleHighlights<'a> {
    pub layers: [&'a [HighlightItem]; 4],
    pub colors: [[f32; 4]; 4],
}

impl HandleHighlights<'_> {
    /// `HL_*` bits of handle `(pick, sub)`: a layer lists the whole owner or a
    /// [`SubKind::Handle`] range containing `sub`.
    fn bits(&self, handle: Option<(PickId, u32)>) -> u32 {
        let Some((pick, sub)) = handle else {
            return 0;
        };
        let mut bits = 0;
        for (layer, items) in HighlightLayer::ALL.iter().zip(self.layers) {
            let hit = items.iter().any(|i| {
                i.pick == pick
                    && match i.sub {
                        None => true,
                        Some((SubKind::Handle, r)) => r.start <= sub && sub < r.end,
                        Some(_) => false,
                    }
            });
            if hit {
                bits |= layer.bit();
            }
        }
        bits
    }

    /// `rgb` with the highlight of `bits` (the CPU twin of `apply_highlight`, `scene.wgsl`).
    fn apply(&self, rgb: [f32; 3], bits: u32) -> [f32; 3] {
        let c = |l: HighlightLayer| {
            let v = self.colors[l as usize];
            [v[0], v[1], v[2]]
        };
        if bits == 0 {
            rgb
        } else if bits & HighlightLayer::Error.bit() != 0 {
            c(HighlightLayer::Error)
        } else if bits & HighlightLayer::Selected.bit() != 0 {
            let s = c(HighlightLayer::Selected);
            if bits & HighlightLayer::Hover.bit() != 0 {
                s.map(|v| v + (1.0 - v) * 0.35)
            } else {
                s
            }
        } else if bits & HighlightLayer::Hover.bit() != 0 {
            c(HighlightLayer::Hover)
        } else {
            c(HighlightLayer::Preview)
        }
    }
}

fn mode_of(depth: DepthMode, force_always: bool) -> (f32, f32) {
    if force_always {
        return (MODE_ALWAYS, 1.0);
    }
    match depth {
        DepthMode::Test => (MODE_TEST, 1.0),
        DepthMode::AlwaysOnTop => (MODE_ALWAYS, 1.0),
        DepthMode::XRay { hidden_alpha } => (MODE_XRAY, hidden_alpha.clamp(0.0, 1.0)),
    }
}

fn v3(v: Vec3) -> [f32; 3] {
    [v.x as f32, v.y as f32, v.z as f32]
}

/// Unit vectors `a`, `b` with `a ⟂ b ⟂ n`.
fn perpendicular_basis(n: Vec3) -> (Vec3, Vec3) {
    let helper = if n.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    let a = n.cross(helper).normalize();
    (a, n.cross(a))
}

/// Cell boundaries of one grid axis over `[-extent, extent]`: `anchor ± cell · growth^k`, so
/// cells are small near the anchor and grow geometrically.
fn grid_stops(anchor: f64, extent: f64, cell: f64) -> Vec<f64> {
    let mut stops = vec![-extent, anchor, extent];
    let mut d = cell.max(1e-300);
    while d < 2.0 * extent {
        stops.push((anchor - d).max(-extent));
        stops.push((anchor + d).min(extent));
        d *= GRID_CELL_GROWTH;
    }
    stops.sort_by(f64::total_cmp);
    stops.dedup();
    stops
}

/// Tessellates overlay lists of one view into [`OverlayGeometry`].
pub(crate) struct Tessellator<'a> {
    pub geometry: &'a mut OverlayGeometry,
    pub highlights: HandleHighlights<'a>,
    /// Whether an image key is live.
    pub image_ok: &'a dyn Fn(ImageKey) -> bool,
}

/// Per-layer state of a tessellation.
struct LayerBuild<'c> {
    cam: &'c Camera,
    size: [u32; 2],
    eye: Point3,
    right: Vec3,
    up: Vec3,
    forward: Vec3,
    force_always: bool,
    edges: EdgeData,
    opaque: Vec<(Option<ImageKey>, Vec<TriVertex>)>,
    translucent: Vec<(Option<ImageKey>, Vec<TriVertex>)>,
    pickable: bool,
}

impl LayerBuild<'_> {
    fn ps(&self, p: Point3) -> f64 {
        self.cam.pixel_size_at(p, self.size).max(1e-300)
    }

    fn rte(&self, p: Point3) -> [f32; 3] {
        v3(p - self.eye)
    }

    fn polyline(&mut self, pts: impl IntoIterator<Item = Point3>, closed: bool, slot: u32) {
        let first = self.edges.points.len() as u32;
        let eye = self.eye;
        self.edges
            .points
            .extend(pts.into_iter().map(|p| v3(p - eye)));
        let len = self.edges.points.len() as u32 - first;
        self.edges.polylines.push(Polyline {
            first,
            len,
            id: slot,
            closed,
        });
    }

    fn tris(&mut self, image: Option<ImageKey>, write: bool, verts: Vec<TriVertex>) {
        let list = if write {
            &mut self.opaque
        } else {
            &mut self.translucent
        };
        match list.last_mut() {
            Some((img, v)) if *img == image => v.extend(verts),
            _ => list.push((image, verts)),
        }
    }
}

impl Tessellator<'_> {
    /// Adds the record + line record of one styled primitive; returns its slot.
    fn record(
        &mut self,
        lb: &LayerBuild<'_>,
        handle: Option<(PickId, u32)>,
        line: Option<&LineStyle>,
        point: Option<&PointStyle>,
    ) -> u32 {
        let slot = self.geometry.records.len() as u32;
        let hi = lb.eye.as_vec3();
        let lo = (lb.eye - hi.as_dvec3()).as_vec3();
        let (pick, sub) = handle.map_or((0, 0), |(p, s)| (p.0, s));
        self.geometry.records.push(InstanceRecord {
            lin: [
                [1.0, 0.0, 0.0, hi.x],
                [0.0, 1.0, 0.0, hi.y],
                [0.0, 0.0, 1.0, hi.z],
            ],
            t_lo: [lo.x, lo.y, lo.z],
            // The handle sub-id rides in the material lane (overlay records are never shaded).
            material: sub,
            nrm0: [1.0, 0.0, 0.0],
            pick,
            nrm1: [0.0, 1.0, 0.0],
            flags: self.highlights.bits(handle),
            nrm2: [0.0, 0.0, 1.0],
            states: 0,
            color: [1.0; 4],
        });
        let (mode, alpha, color, width, dash) = match (line, point) {
            (Some(l), _) => {
                let (m, a) = mode_of(l.depth, lb.force_always);
                let dash = l
                    .dash
                    .filter(|&(on, off)| on > 0.0 && off > 0.0)
                    .map_or([0.0, 0.0], |(on, off)| [on, off]);
                (m, a, l.color, l.width_px.max(0.0), dash)
            }
            (None, Some(p)) => {
                let (m, a) = mode_of(p.depth, lb.force_always);
                (m, a, p.color, p.size_px.max(0.0), [0.0, 0.0])
            }
            (None, None) => (MODE_ALWAYS, 1.0, [1.0; 4], 0.0, [0.0, 0.0]),
        };
        self.geometry.lines.push(LineRecord {
            edge_color: color,
            point_color: color,
            edge_delta: [0.0, 0.0, 0.0, width],
            point_delta: [0.0, 0.0, 0.0, width],
            dash: [dash[0], dash[1], 0.0, 0.0],
            // x, y = x-ray alpha of lines and points; z = 0 (never sectioned); w = depth mode.
            hidden: [alpha, alpha, 0.0, mode],
        });
        slot
    }

    #[allow(
        clippy::too_many_arguments,
        reason = "one triangle vertex from its plain parts"
    )]
    fn vertex(
        lb: &LayerBuild<'_>,
        p: Point3,
        color: [f32; 4],
        uv: [f32; 2],
        kind: f32,
        depth: DepthMode,
        handle: Option<(PickId, u32)>,
        line_px: f32,
    ) -> TriVertex {
        let (mode, alpha) = mode_of(depth, lb.force_always);
        TriVertex {
            pos: lb.rte(p),
            color,
            uv,
            params: [kind, mode, alpha, line_px],
            pick: handle.map_or([0, 0], |(p, s)| [p.0, s]),
        }
    }

    fn shaded(&self, color: [f32; 4], normal: Vec3, lb: &LayerBuild<'_>, bits: u32) -> [f32; 4] {
        let f = (0.55 + 0.45 * normal.dot(-lb.forward).abs()) as f32;
        let rgb = self.highlights.apply([color[0], color[1], color[2]], bits);
        [rgb[0] * f, rgb[1] * f, rgb[2] * f, color[3]]
    }

    /// Shaft + cone of an arrow from `origin` along unit `dir`, `length` mm.
    #[allow(clippy::too_many_arguments, reason = "one arrow from its plain parts")]
    fn arrow(
        &mut self,
        lb: &mut LayerBuild<'_>,
        origin: Point3,
        dir: Vec3,
        length: f64,
        style: &LineStyle,
        handle: Option<(PickId, u32)>,
    ) {
        if length.is_nan() || length <= 0.0 || dir == Vec3::ZERO {
            return;
        }
        let head = length * HEAD_FRACTION;
        let base = origin + dir * (length - head);
        let tip = origin + dir * length;
        let slot = self.record(lb, handle, Some(style), None);
        lb.pickable |= handle.is_some();
        lb.polyline([origin, base], false, slot);
        let (a, b) = perpendicular_basis(dir);
        let r = head * HEAD_RADIUS;
        let bits = self.highlights.bits(handle);
        let ring = |k: usize| {
            let t = std::f64::consts::TAU * k as f64 / CONE_SEGMENTS as f64;
            a * t.cos() + b * t.sin()
        };
        let mut verts = Vec::with_capacity(CONE_SEGMENTS * 6);
        for k in 0..CONE_SEGMENTS {
            let (r0, r1) = (ring(k), ring(k + 1));
            let p0 = base + r0 * r;
            let p1 = base + r1 * r;
            // Side facet normal: perpendicular to the slant, pointing out.
            let mid = (r0 + r1).normalize_or_zero();
            let n = (mid * head + dir * r).normalize_or_zero();
            let side = self.shaded(style.color, n, lb, bits);
            let cap = self.shaded(style.color, -dir, lb, bits);
            for (p, c) in [
                (p0, side),
                (p1, side),
                (tip, side),
                (p1, cap),
                (p0, cap),
                (base, cap),
            ] {
                verts.push(Self::vertex(
                    lb,
                    p,
                    c,
                    [0.0, 0.0],
                    KIND_FLAT,
                    style.depth,
                    handle,
                    0.0,
                ));
            }
        }
        lb.tris(None, style.color[3] >= 1.0, verts);
    }

    /// Tessellates `list` as one layer seen by `cam` in a view (or inset) of `size`.
    pub fn layer(&mut self, list: &OverlayList, cam: &Camera, size: [u32; 2], force_always: bool) {
        let forward = cam.forward().get();
        let right = forward.cross(cam.up.get()).normalize_or_zero();
        let up = right.cross(forward);
        let mut lb = LayerBuild {
            cam,
            size,
            eye: cam.eye,
            right,
            up,
            forward,
            force_always,
            edges: EdgeData::default(),
            opaque: Vec::new(),
            translucent: Vec::new(),
            pickable: false,
        };
        let seg_start = self.geometry.segment_slots.len() as u32;
        let point_start = self.geometry.point_slots.len() as u32;
        for item in &list.items {
            match item {
                Item::Polyline { pts, style, handle } => {
                    let slot = self.record(&lb, *handle, Some(style), None);
                    lb.pickable |= handle.is_some();
                    let run = &list.points[pts.0 as usize..(pts.0 + pts.1) as usize];
                    lb.polyline(run.iter().copied(), false, slot);
                }
                Item::Arrow {
                    origin,
                    dir,
                    length_px,
                    style,
                    handle,
                } => {
                    let length = f64::from(*length_px) * lb.ps(*origin);
                    self.arrow(&mut lb, *origin, dir.get(), length, style, *handle);
                }
                Item::Ring {
                    center,
                    normal,
                    radius_px,
                    style,
                    handle,
                } => {
                    let r = f64::from(*radius_px) * lb.ps(*center);
                    let (a, b) = perpendicular_basis(normal.get());
                    let slot = self.record(&lb, *handle, Some(style), None);
                    lb.pickable |= handle.is_some();
                    let c = *center;
                    lb.polyline(
                        (0..RING_SEGMENTS).map(|k| {
                            let t = std::f64::consts::TAU * k as f64 / RING_SEGMENTS as f64;
                            c + (a * t.cos() + b * t.sin()) * r
                        }),
                        true,
                        slot,
                    );
                }
                Item::Triad {
                    frame,
                    size_px,
                    handle_base,
                } => {
                    let origin = frame.translation;
                    let length = f64::from(*size_px) * lb.ps(origin);
                    let axes = [
                        frame.matrix3.x_axis,
                        frame.matrix3.y_axis,
                        frame.matrix3.z_axis,
                    ];
                    for (k, axis) in axes.into_iter().enumerate() {
                        let style = LineStyle {
                            width_px: TRIAD_WIDTH_PX,
                            color: TRIAD_COLORS[k],
                            dash: None,
                            depth: DepthMode::AlwaysOnTop,
                        };
                        let handle = handle_base.map(|p| (p, k as u32));
                        self.arrow(
                            &mut lb,
                            origin,
                            axis.normalize_or_zero(),
                            length,
                            &style,
                            handle,
                        );
                    }
                }
                Item::PlaneQuad {
                    plane,
                    half_size,
                    fill,
                    border,
                    handle,
                } => {
                    let (x, y) = (plane.x.get() * half_size[0], plane.y.get() * half_size[1]);
                    let o = plane.origin;
                    let corners = [o - x - y, o + x - y, o + x + y, o - x + y];
                    let slot = self.record(&lb, *handle, Some(border), None);
                    lb.pickable |= handle.is_some();
                    lb.polyline(corners, true, slot);
                    if fill[3] > 0.0 {
                        let bits = self.highlights.bits(*handle);
                        let rgb = self.highlights.apply([fill[0], fill[1], fill[2]], bits);
                        let color = [rgb[0], rgb[1], rgb[2], fill[3]];
                        let verts = [0, 1, 2, 0, 2, 3]
                            .map(|i| {
                                Self::vertex(
                                    &lb,
                                    corners[i],
                                    color,
                                    [0.0, 0.0],
                                    KIND_FLAT,
                                    border.depth,
                                    *handle,
                                    0.0,
                                )
                            })
                            .to_vec();
                        lb.tris(None, fill[3] >= 1.0, verts);
                    }
                }
                Item::Grid {
                    plane,
                    extent,
                    color,
                } => {
                    let (x, y, o) = (plane.x.get(), plane.y.get(), plane.origin);
                    // The plane point nearest the camera target anchors the tessellation: plane
                    // coordinates are relative to an offset snapped to a large power of ten
                    // there, and the quad is a tensor mesh graded geometrically away from it,
                    // so the per-pixel spacing (`fract(uv / s)`) keeps its f32 precision where
                    // the lines are finest.
                    let t = cam.target - o;
                    let near = [
                        t.dot(x).clamp(-extent[0], extent[0]),
                        t.dot(y).clamp(-extent[1], extent[1]),
                    ];
                    let ps = lb.ps(o + x * near[0] + y * near[1]);
                    let unit = 10f64.powf((ps * GRID_SNAP_PX).log10().ceil());
                    let offset = near.map(|c| (c / unit).floor() * unit);
                    let axis = |k: usize| grid_stops(near[k], extent[k], ps * GRID_CELL_PX);
                    let (us, vs) = (axis(0), axis(1));
                    let vertex = |u: f64, v: f64| {
                        Self::vertex(
                            &lb,
                            o + x * u + y * v,
                            *color,
                            [(u - offset[0]) as f32, (v - offset[1]) as f32],
                            KIND_GRID,
                            DepthMode::Test,
                            None,
                            GRID_LINE_PX,
                        )
                    };
                    let mut verts = Vec::with_capacity(6 * us.len() * vs.len());
                    for v in vs.windows(2) {
                        for u in us.windows(2) {
                            let c = [
                                vertex(u[0], v[0]),
                                vertex(u[1], v[0]),
                                vertex(u[1], v[1]),
                                vertex(u[0], v[1]),
                            ];
                            verts.extend([c[0], c[1], c[2], c[0], c[2], c[3]]);
                        }
                    }
                    lb.tris(None, false, verts);
                }
                Item::Image {
                    img,
                    corners,
                    opacity,
                } => {
                    if !(self.image_ok)(*img) {
                        continue;
                    }
                    let uv = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];
                    let color = [1.0, 1.0, 1.0, opacity.clamp(0.0, 1.0)];
                    let verts = [0, 1, 2, 0, 2, 3]
                        .map(|i| {
                            Self::vertex(
                                &lb,
                                corners[i],
                                color,
                                uv[i],
                                KIND_IMAGE,
                                DepthMode::Test,
                                None,
                                0.0,
                            )
                        })
                        .to_vec();
                    lb.tris(Some(*img), *opacity >= 1.0, verts);
                }
                Item::Points { pts, style, handle } => {
                    let slot = self.record(&lb, *handle, None, Some(style));
                    lb.pickable |= handle.is_some();
                    let run = &list.points[pts.0 as usize..(pts.0 + pts.1) as usize];
                    for p in run {
                        let [px, py, pz] = lb.rte(*p);
                        self.geometry.points.push([px, py, pz, 0.0]);
                        self.geometry.point_slots.push(slot);
                    }
                }
                Item::Sprite {
                    at,
                    img,
                    uv,
                    size_px,
                    handle,
                } => {
                    if !(self.image_ok)(*img) {
                        continue;
                    }
                    lb.pickable |= handle.is_some();
                    let h = 0.5 * f64::from(*size_px) * lb.ps(*at);
                    let (r, u) = (lb.right * h, lb.up * h);
                    let corners = [*at - r + u, *at + r + u, *at + r - u, *at - r - u];
                    let [[u0, v0], [u1, v1]] = *uv;
                    let uvs = [[u0, v0], [u1, v0], [u1, v1], [u0, v1]];
                    let bits = self.highlights.bits(*handle);
                    let rgb = self.highlights.apply([1.0, 1.0, 1.0], bits);
                    let color = [rgb[0], rgb[1], rgb[2], 1.0];
                    let verts = [0, 1, 2, 0, 2, 3]
                        .map(|i| {
                            Self::vertex(
                                &lb,
                                corners[i],
                                color,
                                uvs[i],
                                KIND_IMAGE,
                                DepthMode::AlwaysOnTop,
                                *handle,
                                0.0,
                            )
                        })
                        .to_vec();
                    lb.tris(Some(*img), false, verts);
                }
                Item::Polygon {
                    pts,
                    tris,
                    fill,
                    depth,
                    handle,
                } => {
                    lb.pickable |= handle.is_some();
                    let run = &list.points[pts.0 as usize..(pts.0 + pts.1) as usize];
                    let bits = self.highlights.bits(*handle);
                    let rgb = self.highlights.apply([fill[0], fill[1], fill[2]], bits);
                    let color = [rgb[0], rgb[1], rgb[2], fill[3]];
                    let mut verts = Vec::with_capacity(tris.1 as usize * 3);
                    for t in &list.tris[tris.0 as usize..(tris.0 + tris.1) as usize] {
                        if t.iter().any(|&i| i as usize >= run.len()) {
                            continue;
                        }
                        for &i in t {
                            verts.push(Self::vertex(
                                &lb,
                                run[i as usize],
                                color,
                                [0.0, 0.0],
                                KIND_FLAT,
                                *depth,
                                *handle,
                                0.0,
                            ));
                        }
                    }
                    lb.tris(None, fill[3] >= 1.0, verts);
                }
            }
        }
        // RND-12's segment builder (arc lengths for dashes); the polyline id is the record slot.
        // The smooth-join flags are cleared: overlay lines keep a round cap at every join,
        // because a skipped cap leaves a sub-pixel wedge on the outer side of the turn that
        // swallows pixel centres lying on the join's bisector (a ring whose vertices sit on
        // pixel centres loses whole pixels there). Translucent overlay lines blend twice at
        // their joins instead.
        if !lb.edges.polylines.is_empty()
            && let Ok((segs, ids)) = build_segments(&lb.edges, &JobCtx::detached())
        {
            self.geometry
                .segments
                .extend(segs.iter().map(|&[x, y, z, arc]| [x, y, z, arc.abs()]));
            self.geometry.segment_slots.extend_from_slice(&ids);
        }
        let draw_start = self.geometry.tri_draws.len();
        for (list, write) in [(lb.opaque, true), (lb.translucent, false)] {
            for (image, verts) in list {
                if verts.is_empty() {
                    continue;
                }
                self.geometry.tri_draws.push(TriDraw {
                    first: self.geometry.tris.len() as u32,
                    count: verts.len() as u32,
                    image,
                    write,
                });
                self.geometry.tris.extend_from_slice(&verts);
            }
        }
        self.geometry.layers.push(LayerDraws {
            segments: seg_start..self.geometry.segment_slots.len() as u32,
            points: point_start..self.geometry.point_slots.len() as u32,
            tri_draws: draw_start..self.geometry.tri_draws.len(),
            pickable: lb.pickable,
        });
    }
}

/// An RGBA8 sRGB image on the GPU for image planes and sprites. Immutable and `Send + Sync`.
#[derive(Debug)]
pub struct GpuImage {
    pub(crate) view: wgpu::TextureView,
    size: [u32; 2],
    /// Group 3 of the overlay triangle pipeline, made by the renderer on first draw.
    pub(crate) bind: OnceLock<wgpu::BindGroup>,
}

impl GpuImage {
    /// Width and height, pixels.
    pub fn size(&self) -> [u32; 2] {
        self.size
    }

    /// VRAM used by the texture.
    pub fn gpu_bytes(&self) -> u64 {
        4 * u64::from(self.size[0]) * u64::from(self.size[1])
    }
}

/// Uploads an RGBA8 sRGB image for [`OverlayList::image_plane`] and [`OverlayList::sprite`]
/// (IO-10 canvases, icon atlases). Row 0 is the top of the image.
///
/// # Errors
/// [`RenderError::InvalidData`] for an empty image or a pixel buffer of the wrong length;
/// [`RenderError::TooLarge`] beyond the device's 2D texture size.
pub fn upload_image(gpu: &Gpu, img: &RgbaImage) -> Result<GpuImage, RenderError> {
    let [w, h] = img.size;
    if w == 0 || h == 0 {
        return Err(RenderError::InvalidData("the image is empty".into()));
    }
    let expected = 4 * u64::from(w) * u64::from(h);
    if img.pixels.len() as u64 != expected {
        return Err(RenderError::InvalidData(format!(
            "{} pixel bytes for a {w}x{h} image (expected {expected})",
            img.pixels.len()
        )));
    }
    let limit = gpu.device().limits().max_texture_dimension_2d;
    if w.max(h) > limit {
        return Err(RenderError::TooLarge {
            what: "image",
            bytes: u64::from(w.max(h)),
            limit: u64::from(limit),
        });
    }
    crate::upload::check_device(gpu)?;
    let extent = wgpu::Extent3d {
        width: w,
        height: h,
        depth_or_array_layers: 1,
    };
    let texture = gpu.device().create_texture(&wgpu::TextureDescriptor {
        label: Some("amg overlay image"),
        size: extent,
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8UnormSrgb,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    gpu.queue().write_texture(
        texture.as_image_copy(),
        &img.pixels,
        wgpu::TexelCopyBufferLayout {
            offset: 0,
            bytes_per_row: Some(4 * w),
            rows_per_image: Some(h),
        },
        extent,
    );
    Ok(GpuImage {
        view: texture.create_view(&wgpu::TextureViewDescriptor::default()),
        size: img.size,
        bind: OnceLock::new(),
    })
}

/// A GPU buffer that grows by powers of two and is rewritten every frame.
#[derive(Debug)]
pub(crate) struct GrowBuffer {
    label: &'static str,
    usage: wgpu::BufferUsages,
    buffer: Option<wgpu::Buffer>,
}

/// Smallest buffer made (a storage binding cannot be empty).
const MIN_BUFFER_BYTES: u64 = 256;

impl GrowBuffer {
    pub fn new(label: &'static str, usage: wgpu::BufferUsages) -> GrowBuffer {
        GrowBuffer {
            label,
            usage: usage | wgpu::BufferUsages::COPY_DST,
            buffer: None,
        }
    }

    /// Writes `bytes` (growing the buffer when needed) and returns the buffer.
    pub fn write(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        bytes: &[u8],
    ) -> &wgpu::Buffer {
        let need = (bytes.len() as u64).max(MIN_BUFFER_BYTES);
        if self.buffer.as_ref().is_none_or(|b| b.size() < need) {
            self.buffer = Some(device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(self.label),
                size: need.next_power_of_two(),
                usage: self.usage,
                mapped_at_creation: false,
            }));
        }
        let buffer = self.buffer.get_or_insert_with(|| unreachable!());
        if !bytes.is_empty() {
            queue.write_buffer(buffer, 0, bytes);
        }
        buffer
    }

    pub fn get(&self) -> Option<&wgpu::Buffer> {
        self.buffer.as_ref()
    }
}

/// Per-view overlay GPU state: the frame's geometry (kept for picks) and its buffers.
#[derive(Debug)]
pub(crate) struct OverlayGpu {
    pub geometry: OverlayGeometry,
    pub records: GrowBuffer,
    pub lines: GrowBuffer,
    pub segments: GrowBuffer,
    pub segment_ids: GrowBuffer,
    pub points: GrowBuffer,
    pub point_ids: GrowBuffer,
    pub tris: GrowBuffer,
    /// Camera uniforms of the insets.
    pub inset_cameras: Vec<wgpu::Buffer>,
    /// The scene depth target was sampleable once (kept so the targets are not rebuilt when a
    /// tool stops drawing overlays).
    pub sampled_depth: bool,
}

impl Default for OverlayGpu {
    fn default() -> Self {
        let storage = wgpu::BufferUsages::STORAGE;
        OverlayGpu {
            geometry: OverlayGeometry::default(),
            records: GrowBuffer::new("amg overlay records", storage),
            lines: GrowBuffer::new("amg overlay line records", storage),
            segments: GrowBuffer::new("amg overlay segments", storage),
            segment_ids: GrowBuffer::new("amg overlay segment slots", storage),
            points: GrowBuffer::new("amg overlay points", storage),
            point_ids: GrowBuffer::new("amg overlay point slots", storage),
            tris: GrowBuffer::new("amg overlay triangles", wgpu::BufferUsages::VERTEX),
            inset_cameras: Vec::new(),
            sampled_depth: false,
        }
    }
}

impl OverlayGpu {
    /// Writes the frame's geometry into the buffers.
    pub fn upload(&mut self, device: &wgpu::Device, queue: &wgpu::Queue) {
        let g = &self.geometry;
        self.records
            .write(device, queue, bytemuck::cast_slice(&g.records));
        self.lines
            .write(device, queue, bytemuck::cast_slice(&g.lines));
        self.segments
            .write(device, queue, bytemuck::cast_slice(&g.segments));
        self.segment_ids
            .write(device, queue, bytemuck::cast_slice(&g.segment_slots));
        self.points
            .write(device, queue, bytemuck::cast_slice(&g.points));
        self.point_ids
            .write(device, queue, bytemuck::cast_slice(&g.point_slots));
        self.tris
            .write(device, queue, bytemuck::cast_slice(&g.tris));
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, reason = "test code")]
#[allow(clippy::float_cmp, reason = "exact constants")]
mod tests {
    use super::*;
    use crate::camera::Projection;

    fn camera() -> Camera {
        Camera {
            eye: Point3::new(30.0, -200.0, 80.0),
            target: Point3::ZERO,
            up: Dir3::Z,
            projection: Projection::Perspective {
                fov_y: 45f64.to_radians(),
            },
        }
    }

    fn tessellate(list: &OverlayList, cam: &Camera) -> OverlayGeometry {
        let mut geometry = OverlayGeometry::default();
        let no: [&[HighlightItem]; 4] = [&[], &[], &[], &[]];
        let mut t = Tessellator {
            geometry: &mut geometry,
            highlights: HandleHighlights {
                layers: no,
                colors: [[1.0; 4]; 4],
            },
            image_ok: &|_| true,
        };
        t.layer(list, cam, [800, 600], false);
        geometry
    }

    #[test]
    fn tri_vertex_is_60_bytes_and_matches_the_wgsl_inputs() {
        assert_eq!(TRI_VERTEX_BYTES, 60);
        let wgsl = include_str!("shaders/overlay.wgsl");
        for field in [
            "@location(0) pos: vec3<f32>",
            "@location(1) color: vec4<f32>",
            "@location(2) uv: vec2<f32>",
            "@location(3) params: vec4<f32>",
            "@location(4) pick: vec2<u32>",
        ] {
            assert!(wgsl.contains(field), "{field}");
        }
    }

    #[test]
    fn arrow_world_length_is_the_pixel_length_times_the_pixel_size() {
        let cam = camera();
        let origin = Point3::new(5.0, 3.0, 1.0);
        let mut list = OverlayList::new();
        list.arrow(origin, Dir3::X, 80.0, LineStyle::default(), None);
        let g = tessellate(&list, &cam);
        let ps = cam.pixel_size_at(origin, [800, 600]);
        // The tip is the farthest cone vertex from the origin.
        let tip = g
            .tris
            .iter()
            .map(|v| {
                Vec3::new(
                    f64::from(v.pos[0]),
                    f64::from(v.pos[1]),
                    f64::from(v.pos[2]),
                )
            })
            .map(|p| (p + cam.eye - origin).length())
            .fold(0.0, f64::max);
        assert!(
            (tip - 80.0 * ps).abs() < 1e-6 * tip,
            "{tip} vs {}",
            80.0 * ps
        );
        // One shaft segment, one record.
        assert_eq!(g.segment_slots, vec![0]);
        assert_eq!(g.records.len(), 1);
    }

    #[test]
    fn records_carry_handles_depth_modes_and_highlights() {
        let cam = camera();
        let mut list = OverlayList::new();
        let style = LineStyle {
            depth: DepthMode::XRay { hidden_alpha: 0.3 },
            ..LineStyle::default()
        };
        list.polyline(&[Point3::ZERO, Point3::X], style, Some((PickId(7), 42)));
        list.ring(Point3::ZERO, Dir3::Z, 20.0, LineStyle::default(), None);
        let selected = [HighlightItem {
            pick: PickId(7),
            sub: Some((SubKind::Handle, crate::Range32 { start: 40, end: 43 })),
        }];
        let mut geometry = OverlayGeometry::default();
        let mut t = Tessellator {
            geometry: &mut geometry,
            highlights: HandleHighlights {
                layers: [&[], &selected, &[], &[]],
                colors: [[1.0; 4]; 4],
            },
            image_ok: &|_| true,
        };
        t.layer(&list, &cam, [800, 600], false);
        assert_eq!(geometry.records[0].pick, 7);
        assert_eq!(geometry.records[0].material, 42);
        assert_eq!(geometry.records[0].flags, HighlightLayer::Selected.bit());
        assert_eq!(geometry.lines[0].hidden, [0.3, 0.3, 0.0, MODE_XRAY]);
        assert_eq!(geometry.records[1].pick, 0);
        assert_eq!(geometry.lines[1].hidden[3], MODE_TEST);
        // The ring is closed: RING_SEGMENTS segments of slot 1.
        assert_eq!(
            geometry.segment_slots.iter().filter(|&&s| s == 1).count(),
            RING_SEGMENTS
        );
        assert!(geometry.layers[0].pickable);
        // An inset forces always-on-top.
        let mut g2 = OverlayGeometry::default();
        let no: [&[HighlightItem]; 4] = [&[], &[], &[], &[]];
        Tessellator {
            geometry: &mut g2,
            highlights: HandleHighlights {
                layers: no,
                colors: [[1.0; 4]; 4],
            },
            image_ok: &|_| true,
        }
        .layer(&list, &cam, [100, 100], true);
        assert!(g2.lines.iter().all(|l| l.hidden[3] == MODE_ALWAYS));
    }

    #[test]
    fn labels_project_with_the_camera_and_drop_anchors_behind_the_eye() {
        let cam = camera();
        let mut list = OverlayList::new();
        let style = LabelStyle {
            offset_px: [3.0, -2.0],
            ..LabelStyle::default()
        };
        list.label(Point3::new(1.0, 2.0, 3.0), "a", style);
        list.label(cam.eye - cam.forward().get() * 10.0, "behind", style);
        let mut out = Vec::new();
        list.project_labels(&cam, [800, 600], [10.0, 20.0], &mut out);
        assert_eq!(out.len(), 1);
        let p = cam.project(Point3::new(1.0, 2.0, 3.0), [800, 600]).unwrap();
        assert!((f64::from(out[0].pos_px[0]) - (p[0] + 13.0)).abs() < 1e-3);
        assert!((f64::from(out[0].pos_px[1]) - (p[1] + 18.0)).abs() < 1e-3);
        assert_eq!(out[0].text, "a");
    }

    #[test]
    fn opaque_triangles_come_first_and_images_get_their_own_draws() {
        let cam = camera();
        let mut keys = amg_core::SlotMap::<ImageKey, ()>::with_key();
        let img = keys.insert(());
        let mut list = OverlayList::new();
        list.grid(&Plane::XY, [100.0, 100.0], [1.0; 4]);
        list.plane_quad(
            &Plane::XY,
            [10.0, 10.0],
            [1.0, 0.0, 0.0, 1.0],
            LineStyle::default(),
            None,
        );
        list.image_plane(img, [Point3::ZERO, Point3::X, Point3::ONE, Point3::Y], 0.5);
        list.polygon(
            &[Point3::ZERO, Point3::X, Point3::Y],
            &[[0, 1, 2], [0, 1, 9]],
            [0.0, 1.0, 0.0, 0.5],
            DepthMode::Test,
            None,
        );
        let g = tessellate(&list, &cam);
        let draws: Vec<_> = g
            .tri_draws
            .iter()
            .map(|d| (d.write, d.image, d.count))
            .collect();
        assert_eq!(draws[0], (true, None, 6), "the opaque quad first");
        assert!(!draws[1].0, "then the grid");
        assert_eq!(draws[1].2 % 6, 0);
        assert_eq!(&draws[2..], &[(false, Some(img), 6), (false, None, 3)]);
    }

    #[test]
    fn grid_stops_are_graded_around_the_anchor() {
        let s = grid_stops(3.0, 100.0, 0.5);
        assert_eq!(s.first(), Some(&-100.0));
        assert_eq!(s.last(), Some(&100.0));
        assert!(s.contains(&3.0) && s.contains(&3.5) && s.contains(&2.5) && s.contains(&7.0));
        assert!(s.windows(2).all(|w| w[0] < w[1]));
    }

    #[test]
    fn grid_coordinates_are_snapped_near_the_camera() {
        let mut cam = camera();
        cam.target = Point3::new(1e5 + 0.3, 1e5, 0.0);
        cam.eye = cam.target + Vec3::new(0.0, -5.0, 5.0);
        let mut list = OverlayList::new();
        list.grid(&Plane::XY, [2e5, 2e5], [1.0; 4]);
        let g = tessellate(&list, &cam);
        // Plane coordinates are relative to a snapped offset near the target, so the vertices
        // next to the target have |uv| below the snap unit.
        let ps = cam.pixel_size_at(cam.target, [800, 600]);
        let unit = 10f64.powf((ps * GRID_SNAP_PX).log10().ceil());
        let u0 = f64::from(g.tris[0].uv[0]);
        let offset = -2e5 - u0;
        assert!((cam.target.x - offset).abs() <= unit, "{offset} {unit}");
        assert!(((offset / unit).round() * unit - offset).abs() < 1e-6 * unit);
        let near = g
            .tris
            .iter()
            .map(|v| f64::from(v.uv[0].abs().max(v.uv[1].abs())))
            .fold(f64::INFINITY, f64::min);
        assert!(near <= unit, "{near}");
        // Cells grow geometrically: few vertices for a huge extent.
        assert!(g.tris.len() < 6 * 40 * 40, "{}", g.tris.len());
    }
}
