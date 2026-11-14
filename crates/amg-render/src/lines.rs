//! Lines, points, edges and the edge-based visual styles (RND-12, design note §6).
//!
//! - [`upload_edges`] turns polylines into a segment buffer (two `vec4` per segment: local
//!   position and arc length) that `shaders/lines.wgsl` expands into screen-space quads with a
//!   capsule distance: any width, analytic antialiasing, round joins and screen-space dashes.
//! - [`upload_points`] uploads point sprites (circle distance).
//! - Every instance has a 96-byte [`LineRecord`] next to its 128-byte instance record with the
//!   colours, widths, dashes and the offsets of its edge and point sets.
//! - [`LinePlan`] decides per view which instances draw which line passes for the
//!   [`VisualStyle`]: visible (depth `GreaterEqual`), hidden (depth `Less`, dashed and dimmed) or
//!   always on top, and whether a mesh body draws its triangle wireframe
//!   (`POLYGON_MODE_LINE`) or its feature edges.

use std::sync::OnceLock;

use amg_core::{Aabb3, Point3, Vec3};
use amg_jobs::JobCtx;
use amg_jobs::gpu::Gpu;

use crate::cull::Frustum;
use crate::scene::{Instance, InstanceTable, LayerMask};
use crate::upload::{check_size, upload_chunked};
use crate::{EdgesKey, MeshKey, PointsKey, RenderError};

/// Elements checked or converted between two cancellation checkpoints.
const CHUNK_ELEMENTS: usize = 1 << 18;

/// Joins whose 3D turn is below this angle are "smooth": the cap is left to the neighbour so
/// translucent lines do not double-blend at every vertex of a tessellated curve (design note §6).
const SMOOTH_JOIN_RAD: f64 = 20.0 * std::f64::consts::PI / 180.0;

/// Dash of hidden edges when their style has none: (on, off) px.
pub const HIDDEN_DASH_PX: (f32, f32) = (5.0, 4.0);
/// Alpha factor of hidden edges in the hidden-edge visual styles (design note §6: 40 %).
pub const HIDDEN_ALPHA: f32 = 0.4;

/// Fusion's six visual styles (design note §6).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum VisualStyle {
    /// Shaded faces, no edges.
    Shaded,
    /// Shaded faces and visible edges.
    #[default]
    ShadedVisibleEdges,
    /// Shaded faces, visible edges and dashed, dimmed hidden edges.
    ShadedHiddenEdges,
    /// Every edge, no faces and no hidden-line removal.
    Wireframe,
    /// Visible edges plus dashed, dimmed hidden edges; faces are drawn into depth only.
    WireframeHiddenEdges,
    /// Visible edges only; faces are drawn into depth only.
    WireframeVisibleEdges,
}

impl VisualStyle {
    /// Faces are shaded.
    pub fn shades_faces(self) -> bool {
        matches!(
            self,
            VisualStyle::Shaded | VisualStyle::ShadedVisibleEdges | VisualStyle::ShadedHiddenEdges
        )
    }

    /// Faces are drawn into depth only (wireframe with hidden-line removal).
    pub fn depth_only_faces(self) -> bool {
        matches!(
            self,
            VisualStyle::WireframeHiddenEdges | VisualStyle::WireframeVisibleEdges
        )
    }

    /// Edges are drawn at all.
    pub fn shows_edges(self) -> bool {
        self != VisualStyle::Shaded
    }

    /// Occluded edges are drawn dashed and dimmed.
    pub fn shows_hidden_edges(self) -> bool {
        matches!(
            self,
            VisualStyle::ShadedHiddenEdges | VisualStyle::WireframeHiddenEdges
        )
    }

    /// A wireframe style (mesh bodies show their triangle wireframe).
    pub fn is_wireframe(self) -> bool {
        !self.shades_faces()
    }
}

/// How lines and points are depth tested.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DepthMode {
    /// Hidden by surfaces in front (the hidden-edge styles add a dashed, dimmed copy).
    Test,
    /// Drawn over everything.
    AlwaysOnTop,
    /// Visible parts drawn normally, occluded parts at `hidden_alpha` (solid unless dashed).
    XRay {
        /// Alpha factor of the occluded parts, `[0, 1]`.
        hidden_alpha: f32,
    },
}

/// Style of an instance's edges (and of overlay polylines, RND-07).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LineStyle {
    /// Width in physical pixels (any positive value; antialiased).
    pub width_px: f32,
    /// Linear RGBA.
    pub color: [f32; 4],
    /// Screen-space dash (on, off) px; `None` = solid.
    pub dash: Option<(f32, f32)>,
    /// Depth behaviour.
    pub depth: DepthMode,
}

impl Default for LineStyle {
    /// 1.5 px, near-black, solid, depth tested.
    fn default() -> Self {
        LineStyle {
            width_px: 1.5,
            color: [0.015, 0.017, 0.02, 1.0],
            dash: None,
            depth: DepthMode::Test,
        }
    }
}

/// Style of an instance's points (B-rep vertices, sketch points).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PointStyle {
    /// Diameter in physical pixels.
    pub size_px: f32,
    /// Linear RGBA.
    pub color: [f32; 4],
    /// Depth behaviour.
    pub depth: DepthMode,
}

impl Default for PointStyle {
    /// 6 px, near-black, depth tested.
    fn default() -> Self {
        PointStyle {
            size_px: 6.0,
            color: [0.015, 0.017, 0.02, 1.0],
            depth: DepthMode::Test,
        }
    }
}

/// Plain edge data for [`upload_edges`]: f32 points relative to `origin`, cut into polylines.
#[derive(Clone, Debug, Default)]
pub struct EdgeData {
    /// World position of the local frame, mm (in the frame of the instance transform).
    pub origin: Point3,
    /// Polyline vertices relative to `origin`.
    pub points: Vec<[f32; 3]>,
    /// Runs of `points`.
    pub polylines: Vec<Polyline>,
}

/// One polyline: `points[first..first + len]`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Polyline {
    /// First point.
    pub first: u32,
    /// Number of points; fewer than 2 draw nothing.
    pub len: u32,
    /// Edge sub-id for picking (RND-04): the B-rep edge index.
    pub id: u32,
    /// Joins the last point back to the first (needs at least 3 points).
    pub closed: bool,
}

/// Plain point data for [`upload_points`].
#[derive(Clone, Debug, Default)]
pub struct PointData {
    /// World position of the local frame, mm.
    pub origin: Point3,
    /// Points relative to `origin`.
    pub points: Vec<[f32; 3]>,
    /// Vertex sub-ids for picking (RND-04); empty = the point index.
    pub ids: Vec<u32>,
}

/// An edge set on the GPU. Immutable and `Send + Sync`.
#[derive(Debug)]
pub struct GpuEdges {
    /// Two `vec4<f32>` per segment (`shaders/lines.wgsl`).
    pub(crate) segments: wgpu::Buffer,
    /// Polyline id per segment (RND-04 picking).
    pub(crate) ids: wgpu::Buffer,
    pub(crate) origin: Point3,
    segment_count: u32,
    bounds: Aabb3,
    /// Group-1 bind group, made by the renderer on first draw (one renderer owns a scene).
    pub(crate) bind: OnceLock<wgpu::BindGroup>,
}

impl GpuEdges {
    /// Number of line segments.
    pub fn segment_count(&self) -> u32 {
        self.segment_count
    }

    /// Bounds with `origin` applied, f64 mm.
    pub fn bounds(&self) -> Aabb3 {
        self.bounds
    }

    /// VRAM used by the buffers.
    pub fn gpu_bytes(&self) -> u64 {
        self.segments.size() + self.ids.size()
    }
}

/// A point set on the GPU. Immutable and `Send + Sync`.
#[derive(Debug)]
pub struct GpuPoints {
    /// One `vec4<f32>` per point (`xyz` local position).
    pub(crate) points: wgpu::Buffer,
    /// Vertex sub-id per point (RND-04 picking).
    pub(crate) ids: wgpu::Buffer,
    pub(crate) origin: Point3,
    point_count: u32,
    bounds: Aabb3,
    pub(crate) bind: OnceLock<wgpu::BindGroup>,
}

impl GpuPoints {
    /// Number of points.
    pub fn point_count(&self) -> u32 {
        self.point_count
    }

    /// Bounds with `origin` applied, f64 mm.
    pub fn bounds(&self) -> Aabb3 {
        self.bounds
    }

    /// VRAM used by the buffers.
    pub fn gpu_bytes(&self) -> u64 {
        self.points.size() + self.ids.size()
    }
}

fn finite3(v: &[f32; 3]) -> bool {
    v[0].is_finite() && v[1].is_finite() && v[2].is_finite()
}

fn lift(origin: Point3, v: [f32; 3]) -> Point3 {
    origin + Vec3::new(f64::from(v[0]), f64::from(v[1]), f64::from(v[2]))
}

/// Checks the points (finite, `u32` count) and returns their bounds with `origin` applied.
fn validate_points(
    origin: Point3,
    points: &[[f32; 3]],
    ctx: &JobCtx,
) -> Result<Aabb3, RenderError> {
    if !origin.is_finite() {
        return Err(RenderError::InvalidData("the origin is not finite".into()));
    }
    if u32::try_from(points.len()).is_err() {
        return Err(RenderError::InvalidData("more than u32::MAX points".into()));
    }
    let mut bounds = Aabb3::EMPTY;
    for (chunk_index, chunk) in points.chunks(CHUNK_ELEMENTS).enumerate() {
        for (i, p) in chunk.iter().enumerate() {
            if !finite3(p) {
                return Err(RenderError::InvalidData(format!(
                    "point {} is not finite: {p:?}",
                    chunk_index * CHUNK_ELEMENTS + i
                )));
            }
            bounds.include_point(lift(origin, *p));
        }
        ctx.checkpoint()?;
    }
    Ok(bounds)
}

/// Segments of `data` in upload order: two `[x, y, z, arc]` per segment, and the polyline id of
/// each. The sign bit of `arc` marks a smooth join at that end ([`SMOOTH_JOIN_RAD`]).
///
/// # Errors
/// [`RenderError::InvalidData`] for a polyline outside `points`; [`RenderError::Cancelled`].
pub(crate) fn build_segments(
    data: &EdgeData,
    ctx: &JobCtx,
) -> Result<(Vec<[f32; 4]>, Vec<u32>), RenderError> {
    let n = data.points.len() as u64;
    let mut total = 0usize;
    for (i, pl) in data.polylines.iter().enumerate() {
        if u64::from(pl.first) + u64::from(pl.len) > n {
            return Err(RenderError::InvalidData(format!(
                "polyline {i} covers points {}..{}, but there are {n}",
                pl.first,
                u64::from(pl.first) + u64::from(pl.len)
            )));
        }
        total += segment_count(pl);
    }
    let mut segs = Vec::with_capacity(2 * total);
    let mut ids = Vec::with_capacity(total);
    let p = |i: u32| {
        let v = data.points[i as usize];
        Vec3::new(f64::from(v[0]), f64::from(v[1]), f64::from(v[2]))
    };
    let mut since_checkpoint = 0usize;
    for pl in &data.polylines {
        let count = segment_count(pl);
        if count == 0 {
            continue;
        }
        let vertex = |k: usize| pl.first + (k % pl.len as usize) as u32;
        let dir = |k: usize| (p(vertex(k + 1)) - p(vertex(k))).normalize_or_zero();
        let smooth = |a: Vec3, b: Vec3| {
            a != Vec3::ZERO && b != Vec3::ZERO && a.dot(b).clamp(-1.0, 1.0).acos() < SMOOTH_JOIN_RAD
        };
        let wraps = pl.closed && pl.len >= 3;
        let mut arc = 0.0f64;
        for k in 0..count {
            let (a, b) = (vertex(k), vertex(k + 1));
            let length = p(b).distance(p(a));
            let d = dir(k);
            let start_smooth = (k > 0 || wraps) && smooth(dir((k + count - 1) % count), d);
            let end_smooth = (k + 1 < count || wraps) && smooth(d, dir((k + 1) % count));
            let signed = |v: f64, s: bool| {
                let v = v as f32;
                if s { -v } else { v }
            };
            let [ax, ay, az] = data.points[a as usize];
            let [bx, by, bz] = data.points[b as usize];
            segs.push([ax, ay, az, signed(arc, start_smooth)]);
            arc += length;
            segs.push([bx, by, bz, signed(arc, end_smooth)]);
            ids.push(pl.id);
            since_checkpoint += 1;
            if since_checkpoint >= CHUNK_ELEMENTS {
                since_checkpoint = 0;
                ctx.checkpoint()?;
            }
        }
    }
    Ok((segs, ids))
}

fn segment_count(pl: &Polyline) -> usize {
    match pl.len {
        0 | 1 => 0,
        n if pl.closed && n >= 3 => n as usize,
        n => n as usize - 1,
    }
}

/// Uploads edge polylines. Intended for a job thread; checkpoints between chunks.
///
/// # Errors
/// [`RenderError::InvalidData`] for a non-finite point or origin or a polyline outside
/// `points`; [`RenderError::TooLarge`] beyond the device's buffer limit;
/// [`RenderError::Cancelled`].
pub fn upload_edges(gpu: &Gpu, data: &EdgeData, ctx: &JobCtx) -> Result<GpuEdges, RenderError> {
    let bounds = validate_points(data.origin, &data.points, ctx)?;
    let (segs, ids) = build_segments(data, ctx)?;
    let limit = gpu.device().limits().max_buffer_size;
    let bytes = (segs.len() * size_of::<[f32; 4]>()) as u64;
    check_size("edge segment buffer", bytes, limit)?;
    let segment_count = u32::try_from(ids.len())
        .map_err(|_| RenderError::InvalidData("more than u32::MAX segments".into()))?;
    // A zero-sized storage binding is invalid: keep one dummy element.
    let segs_or_dummy: &[[f32; 4]] = if segs.is_empty() {
        &[[0.0; 4]; 2]
    } else {
        &segs
    };
    let ids_or_dummy: &[u32] = if ids.is_empty() { &[0] } else { &ids };
    let usage = wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST;
    Ok(GpuEdges {
        segments: upload_chunked(gpu, "amg edge segments", segs_or_dummy, usage, ctx)?,
        ids: upload_chunked(gpu, "amg edge ids", ids_or_dummy, usage, ctx)?,
        origin: data.origin,
        segment_count,
        bounds,
        bind: OnceLock::new(),
    })
}

/// Uploads points. Intended for a job thread; checkpoints between chunks.
///
/// # Errors
/// [`RenderError::InvalidData`] for a non-finite point or origin, or `ids` of the wrong length;
/// [`RenderError::TooLarge`]; [`RenderError::Cancelled`].
pub fn upload_points(gpu: &Gpu, data: &PointData, ctx: &JobCtx) -> Result<GpuPoints, RenderError> {
    if !data.ids.is_empty() && data.ids.len() != data.points.len() {
        return Err(RenderError::InvalidData(format!(
            "{} ids for {} points",
            data.ids.len(),
            data.points.len()
        )));
    }
    let bounds = validate_points(data.origin, &data.points, ctx)?;
    let limit = gpu.device().limits().max_buffer_size;
    check_size(
        "point buffer",
        (data.points.len() * size_of::<[f32; 4]>()) as u64,
        limit,
    )?;
    let mut packed = Vec::with_capacity(data.points.len().max(1));
    for chunk in data.points.chunks(CHUNK_ELEMENTS) {
        packed.extend(chunk.iter().map(|p| [p[0], p[1], p[2], 0.0f32]));
        ctx.checkpoint()?;
    }
    if packed.is_empty() {
        packed.push([0.0; 4]);
    }
    let ids: Vec<u32> = if data.ids.is_empty() {
        (0..data.points.len().max(1) as u32).collect()
    } else {
        data.ids.clone()
    };
    let usage = wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST;
    Ok(GpuPoints {
        points: upload_chunked(gpu, "amg points", &packed, usage, ctx)?,
        ids: upload_chunked(gpu, "amg point ids", &ids, usage, ctx)?,
        origin: data.origin,
        point_count: data.points.len() as u32,
        bounds,
        bind: OnceLock::new(),
    })
}

/// One 96-byte record of the line-record storage buffer (`LineRecord` in `shaders/scene.wgsl`),
/// parallel to the 128-byte instance records.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub(crate) struct LineRecord {
    pub edge_color: [f32; 4],
    pub point_color: [f32; 4],
    /// Edge-set origin minus the record origin (local frame); `w` = width px.
    pub edge_delta: [f32; 4],
    /// Point-set origin minus the record origin; `w` = diameter px.
    pub point_delta: [f32; 4],
    /// Visible (on, off) and hidden (on, off) dash, px; 0 = solid.
    pub dash: [f32; 4],
    /// `x` = alpha factor of the hidden pass.
    pub hidden: [f32; 4],
}

/// Size of one line record, bytes.
pub(crate) const LINE_RECORD_BYTES: u64 = size_of::<LineRecord>() as u64;

fn delta(set_origin: Option<Point3>, record_origin: Point3) -> [f32; 3] {
    let d = set_origin.map_or(Vec3::ZERO, |o| o - record_origin);
    [d.x as f32, d.y as f32, d.z as f32]
}

fn dash_or_zero(d: Option<(f32, f32)>) -> [f32; 2] {
    d.filter(|&(on, off)| on > 0.0 && off > 0.0)
        .map_or([0.0, 0.0], |(on, off)| [on, off])
}

/// The line record of `inst`, whose instance record is built around `record_origin`.
pub(crate) fn build_line_record(
    inst: &Instance,
    record_origin: Point3,
    edges_origin: Option<Point3>,
    points_origin: Option<Point3>,
) -> LineRecord {
    let e = &inst.edge_style;
    let p = &inst.point_style;
    let [ex, ey, ez] = delta(edges_origin, record_origin);
    let [px, py, pz] = delta(points_origin, record_origin);
    let visible = dash_or_zero(e.dash);
    let (hidden_dash, hidden_alpha) = match e.depth {
        DepthMode::XRay { hidden_alpha } => (visible, hidden_alpha.clamp(0.0, 1.0)),
        _ => (
            dash_or_zero(Some(e.dash.unwrap_or(HIDDEN_DASH_PX))),
            HIDDEN_ALPHA,
        ),
    };
    let point_hidden = match p.depth {
        DepthMode::XRay { hidden_alpha } => hidden_alpha.clamp(0.0, 1.0),
        _ => HIDDEN_ALPHA,
    };
    LineRecord {
        edge_color: e.color,
        point_color: p.color,
        edge_delta: [ex, ey, ez, e.width_px.max(0.0)],
        point_delta: [px, py, pz, p.size_px.max(0.0)],
        dash: [visible[0], visible[1], hidden_dash[0], hidden_dash[1]],
        // The point pass reads `hidden.y`; edges read `hidden.x`. `z` marks a sectioned
        // instance and `w` holds its cap colour (RND-06; `cap_color` in `shaders/mesh.wgsl`).
        hidden: [
            hidden_alpha,
            point_hidden,
            f32::from(u8::from(inst.sectioned)),
            pack_cap_color(inst.material.cap_color),
        ],
    }
}

/// A cap colour (linear RGB) as sRGB 8-bit `r | g << 8 | b << 16`, stored as an exact f32
/// integer; `-1` for the default (the darkened base colour).
pub(crate) fn pack_cap_color(c: Option<[f32; 3]>) -> f32 {
    let Some(c) = c else {
        return -1.0;
    };
    let q = |v: f32| -> u32 {
        let v = v.clamp(0.0, 1.0);
        let s = if v <= 0.003_130_8 {
            12.92 * v
        } else {
            1.055 * v.powf(1.0 / 2.4) - 0.055
        };
        (s * 255.0).round() as u32
    };
    (q(c[0]) | (q(c[1]) << 8) | (q(c[2]) << 16)) as f32
}

/// Depth test of a line draw.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub(crate) enum LinePass {
    /// Behind surfaces: depth `Less` (reversed-Z), dashed and dimmed.
    Hidden,
    /// In front of surfaces: depth `GreaterEqual`.
    Visible,
    /// Depth `Always`.
    Always,
}

/// What a line draw reads.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub(crate) enum LineGeometry {
    /// Triangle wireframe of a mesh (`POLYGON_MODE_LINE`).
    Wire(MeshKey),
    /// Wide-line segments of an edge set.
    Edges(EdgesKey),
    /// Point sprites.
    Points(PointsKey),
}

/// One instanced line, point or wireframe draw.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct LineDraw {
    pub pass: LinePass,
    pub geometry: LineGeometry,
    /// First entry of [`LinePlan::slots`].
    pub first: u32,
    pub count: u32,
}

/// What the line plan needs to know about a view.
#[derive(Clone, Copy, Debug)]
pub(crate) struct LineView {
    pub style: VisualStyle,
    pub layers: LayerMask,
    /// Mesh bodies draw their triangle wireframe in the wireframe styles (`POLYGON_MODE_LINE`
    /// is available and enabled); otherwise they draw their edge set (feature edges).
    pub triangle_wire: bool,
    /// Faces were drawn into depth (shaded, or the depth-only wireframe pass ran), so a hidden
    /// pass can find occluded edges.
    pub has_depth: bool,
}

/// The line, point and wireframe draws of one view for one frame.
#[derive(Debug, Default)]
pub(crate) struct LinePlan {
    pub draws: Vec<LineDraw>,
    /// Instance slots in draw order (appended after the mesh plan's slots in the GPU list).
    pub slots: Vec<u32>,
    /// Segments drawn (each instance's edge set counted once).
    pub segments: u64,
    items: Vec<(LinePass, LineGeometry, u32)>,
}

impl LinePlan {
    /// Rebuilds the plan. The `*_ok` closures say whether a key is live and non-empty; `segs`
    /// gives an edge set's segment count.
    #[allow(
        clippy::too_many_arguments,
        reason = "plain inputs of one planning step"
    )]
    pub fn build(
        &mut self,
        table: &InstanceTable,
        frustum: &Frustum,
        view: LineView,
        mesh_ok: impl Fn(MeshKey) -> bool,
        edge_segments: impl Fn(EdgesKey) -> Option<u32>,
        points_ok: impl Fn(PointsKey) -> bool,
    ) {
        self.draws.clear();
        self.slots.clear();
        self.items.clear();
        self.segments = 0;
        let style = view.style;
        for (slot, (inst, info)) in table.instances().iter().zip(table.info()).enumerate() {
            if !inst.visible || !view.layers.contains(inst.layer) {
                continue;
            }
            let Some(sphere) = info.sphere else {
                continue;
            };
            if !frustum.intersects_sphere(sphere.center, sphere.radius) {
                continue;
            }
            let slot = slot as u32;
            let passes = |depth: DepthMode, hidden_style: bool| -> &'static [LinePass] {
                match depth {
                    DepthMode::AlwaysOnTop => &[LinePass::Always],
                    DepthMode::XRay { .. } if view.has_depth => {
                        &[LinePass::Hidden, LinePass::Visible]
                    }
                    DepthMode::Test if hidden_style && view.has_depth => {
                        &[LinePass::Hidden, LinePass::Visible]
                    }
                    _ => &[LinePass::Visible],
                }
            };
            if style.shows_edges() {
                let wire = (style.is_wireframe() && inst.mesh_wireframe && view.triangle_wire)
                    .then_some(inst.mesh)
                    .flatten()
                    .filter(|&m| mesh_ok(m));
                let edge_passes = passes(inst.edge_style.depth, style.shows_hidden_edges());
                if let Some(mesh) = wire {
                    for &pass in edge_passes {
                        self.items.push((pass, LineGeometry::Wire(mesh), slot));
                    }
                } else if let Some((key, n)) = inst
                    .edges
                    .and_then(|k| edge_segments(k).filter(|&n| n > 0).map(|n| (k, n)))
                {
                    self.segments += u64::from(n);
                    for &pass in edge_passes {
                        self.items.push((pass, LineGeometry::Edges(key), slot));
                    }
                }
            }
            if let Some(key) = inst.points.filter(|&k| points_ok(k)) {
                let point_passes: &[LinePass] = match inst.point_style.depth {
                    DepthMode::Test => &[LinePass::Visible],
                    other => passes(other, false),
                };
                for &pass in point_passes {
                    self.items.push((pass, LineGeometry::Points(key), slot));
                }
            }
        }
        // Hidden first (dimmed, underneath), then visible, then always-on-top; one draw per run.
        self.items.sort_unstable();
        let mut start = 0;
        for i in 1..=self.items.len() {
            let boundary = i == self.items.len()
                || (self.items[i].0, self.items[i].1) != (self.items[start].0, self.items[start].1);
            if boundary {
                if i > start {
                    self.draws.push(LineDraw {
                        pass: self.items[start].0,
                        geometry: self.items[start].1,
                        first: self.slots.len() as u32,
                        count: (i - start) as u32,
                    });
                    self.slots
                        .extend(self.items[start..i].iter().map(|&(_, _, s)| s));
                }
                start = i;
            }
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, reason = "test code")]
#[allow(
    clippy::float_cmp,
    reason = "exact arc lengths of axis-aligned segments"
)]
mod tests {
    use super::*;

    fn data(points: Vec<[f32; 3]>, polylines: Vec<Polyline>) -> EdgeData {
        EdgeData {
            origin: Point3::ZERO,
            points,
            polylines,
        }
    }

    fn smooth(w: f32) -> bool {
        w.to_bits() >> 31 != 0
    }

    #[test]
    fn line_record_is_96_bytes_and_matches_the_wgsl_struct() {
        assert_eq!(size_of::<LineRecord>(), 96);
        let wgsl = include_str!("shaders/scene.wgsl");
        for field in [
            "edge_color: vec4<f32>",
            "point_color: vec4<f32>",
            "edge_delta: vec4<f32>",
            "point_delta: vec4<f32>",
            "dash: vec4<f32>",
            "hidden: vec4<f32>",
        ] {
            assert!(wgsl.contains(field), "{field}");
        }
    }

    #[test]
    fn open_polyline_has_cumulative_arcs_and_sharp_ends() {
        let d = data(
            vec![[0.0, 0.0, 0.0], [3.0, 0.0, 0.0], [3.0, 4.0, 0.0]],
            vec![Polyline {
                first: 0,
                len: 3,
                id: 7,
                closed: false,
            }],
        );
        let (segs, ids) = build_segments(&d, &JobCtx::detached()).unwrap();
        assert_eq!(ids, vec![7, 7]);
        assert_eq!(segs.len(), 4);
        assert_eq!(segs[0][3].abs(), 0.0);
        assert_eq!(segs[1][3].abs(), 3.0);
        assert_eq!(segs[2][3].abs(), 3.0);
        assert_eq!(segs[3][3].abs(), 7.0);
        // A 90° corner and the open ends keep their round caps.
        assert!(segs.iter().all(|s| !smooth(s[3])));
    }

    #[test]
    fn closed_polyline_wraps_and_marks_gentle_joins_smooth() {
        // A regular 36-gon: every turn is 10°, below the 20° threshold.
        let n = 36;
        let points: Vec<[f32; 3]> = (0..n)
            .map(|i| {
                let a = std::f32::consts::TAU * i as f32 / n as f32;
                [10.0 * a.cos(), 10.0 * a.sin(), 0.0]
            })
            .collect();
        let d = data(
            points,
            vec![Polyline {
                first: 0,
                len: n,
                id: 1,
                closed: true,
            }],
        );
        let (segs, ids) = build_segments(&d, &JobCtx::detached()).unwrap();
        assert_eq!(ids.len(), n as usize, "the closing segment is included");
        assert!(segs.iter().all(|s| smooth(s[3])));
        let last = segs.last().unwrap();
        assert_eq!(&last[..3], &d.points[0][..], "wraps to the first point");
    }

    #[test]
    fn degenerate_and_out_of_range_polylines() {
        let pl = |first, len, closed| Polyline {
            first,
            len,
            id: 0,
            closed,
        };
        let d = data(
            vec![[0.0; 3], [1.0, 0.0, 0.0]],
            vec![pl(0, 1, false), pl(0, 0, false), pl(0, 2, true)],
        );
        let (segs, ids) = build_segments(&d, &JobCtx::detached()).unwrap();
        assert_eq!(ids.len(), 1, "a closed two-point polyline is one segment");
        assert_eq!(segs.len(), 2);
        let bad = data(vec![[0.0; 3]], vec![pl(0, 2, false)]);
        assert!(matches!(
            build_segments(&bad, &JobCtx::detached()),
            Err(RenderError::InvalidData(_))
        ));
        assert!(matches!(
            validate_points(Point3::ZERO, &[[f32::NAN, 0.0, 0.0]], &JobCtx::detached()),
            Err(RenderError::InvalidData(_))
        ));
    }

    #[test]
    fn line_records_carry_styles_and_origin_offsets() {
        let mut inst = Instance::new(
            amg_core::SlotMap::<MeshKey, ()>::with_key().insert(()),
            amg_core::Transform::IDENTITY,
        );
        inst.edge_style = LineStyle {
            width_px: 3.0,
            color: [1.0, 0.0, 0.0, 1.0],
            dash: None,
            depth: DepthMode::Test,
        };
        let r = build_line_record(
            &inst,
            Point3::new(10.0, 0.0, 0.0),
            Some(Point3::new(12.0, 1.0, 0.0)),
            None,
        );
        assert_eq!(r.edge_delta, [2.0, 1.0, 0.0, 3.0]);
        assert_eq!(r.dash, [0.0, 0.0, HIDDEN_DASH_PX.0, HIDDEN_DASH_PX.1]);
        assert_eq!(r.hidden[0], HIDDEN_ALPHA);
        inst.edge_style.depth = DepthMode::XRay { hidden_alpha: 0.25 };
        inst.edge_style.dash = Some((2.0, 1.0));
        let r = build_line_record(&inst, Point3::ZERO, None, None);
        assert_eq!(r.dash, [2.0, 1.0, 2.0, 1.0]);
        assert_eq!(r.hidden[0], 0.25);
    }

    #[test]
    fn visual_styles_select_faces_and_edges() {
        use VisualStyle as S;
        let all = [
            S::Shaded,
            S::ShadedVisibleEdges,
            S::ShadedHiddenEdges,
            S::Wireframe,
            S::WireframeHiddenEdges,
            S::WireframeVisibleEdges,
        ];
        let shaded: Vec<_> = all.iter().map(|s| s.shades_faces()).collect();
        assert_eq!(shaded, [true, true, true, false, false, false]);
        let edges: Vec<_> = all.iter().map(|s| s.shows_edges()).collect();
        assert_eq!(edges, [false, true, true, true, true, true]);
        let hidden: Vec<_> = all.iter().map(|s| s.shows_hidden_edges()).collect();
        assert_eq!(hidden, [false, false, true, false, true, false]);
        let depth: Vec<_> = all.iter().map(|s| s.depth_only_faces()).collect();
        assert_eq!(depth, [false, false, false, false, true, true]);
        assert_eq!(S::default(), S::ShadedVisibleEdges);
    }
}
