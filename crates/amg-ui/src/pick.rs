//! Display-side identities of pickable things (spec §7, Interfaces §A `amg_ui::pick`).
//!
//! UI-20 defines the types the scene adapter's [`PickMap`](crate::scene::PickMap) translates
//! to. UI-03 adds the [`PickService`] trait (screen-space requests with a filter, answered
//! asynchronously and never blocking), its CPU implementation [`CpuPickService`] (ray casts over
//! the adapter's display meshes; tests and the select-through path) and [`GpuPickService`] over
//! S10's `Renderer::request_pick` / `poll_picks` (RND-04).

use std::collections::{BTreeMap, HashMap};

use amg_core::{Point3, Ray3};
use amg_doc::{BodyId, ConstructionId, OccurrenceId, SketchId, TopoKind};
use amg_render::{
    PickFilter, PickRegion, PickRequest, PickResult, RectMode, Renderer, SubKind, ViewId,
};

use crate::input::CameraView;
use crate::scene::{RenderEntityKey, RenderSub, RenderSubKind, SceneAdapter};
use crate::select::EntityFilter;

/// Display-side identity of a pickable thing in the current `Evaluated`.
///
/// `instance` indexes the scene adapter's occurrence-path table
/// ([`SceneAdapter::occurrence_path`](crate::scene::SceneAdapter::occurrence_path)): the same
/// body seen through two occurrences is two targets (S06 request). A mesh body's
/// `(TopoKind::Face, i)` is triangle `i` (an `amg_mesh::FaceId`).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum PickTarget {
    /// A body, or one of its faces, edges or vertices (`sub == None`: the whole body).
    Body {
        /// The body.
        body: BodyId,
        /// Index into the adapter's occurrence-path table.
        instance: u32,
        /// Sub-entity.
        sub: Option<(TopoKind, u32)>,
    },
    /// A whole sketch (sketch entities and profiles come with the RND-07 overlays, UI-09).
    Sketch(SketchId),
    /// Construction geometry.
    Construction(ConstructionId),
    /// An occurrence (component pick priority, UI-04).
    Occurrence(OccurrenceId),
}

impl PickTarget {
    /// The body, for body targets.
    pub fn body(&self) -> Option<BodyId> {
        match self {
            PickTarget::Body { body, .. } => Some(*body),
            _ => None,
        }
    }

    /// The same target without its sub-entity (the whole body).
    #[must_use]
    pub fn whole(self) -> PickTarget {
        match self {
            PickTarget::Body { body, instance, .. } => PickTarget::Body {
                body,
                instance,
                sub: None,
            },
            other => other,
        }
    }
}

/// How many hits a point pick returns.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PickMode {
    /// The front-most hit.
    Nearest,
    /// Every pickable thing along the ray, front to back (select-through, UI-04).
    AllAlongRay,
}

/// Identifies a pick request until its result is polled.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct PickTicket(pub u64);

/// One pick result in document terms.
#[derive(Clone, Debug, PartialEq)]
pub struct PickHitUi {
    /// What was hit.
    pub target: PickTarget,
    /// World position of the hit, mm.
    pub world: Point3,
    /// Distance along the pick ray, mm (f32 like S10's depth).
    pub depth: f32,
}

/// Rectangle selection semantics.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum RegionMode {
    /// Fully inside (drag left to right).
    Window,
    /// Touching (drag right to left).
    Crossing,
}

/// What a pick service may look at once per frame: the scene (for the pick map) and the
/// camera of the newest frame.
#[derive(Clone, Copy, Debug)]
pub struct PickFrame<'a> {
    /// The scene adapter (pick map, CPU meshes).
    pub scene: &'a SceneAdapter,
    /// The newest camera snapshot (`None` before the viewport was first drawn).
    pub view: Option<&'a CameraView>,
}

/// Screen-space picking (spec §6 "Picking"). Implemented over S10 in the app
/// ([`GpuPickService`]) and on the CPU ([`CpuPickService`], tests). Never blocks: a request
/// returns a ticket and [`PickService::poll`] answers it later, exactly once.
pub trait PickService {
    /// A point pick at `pos` (points). `radius_pt` is a hint: S10 uses its configured pick
    /// radius, the CPU service the ray through `pos`.
    fn request_point(
        &mut self,
        pos: egui::Pos2,
        radius_pt: f32,
        filter: EntityFilter,
        mode: PickMode,
    ) -> PickTicket;
    /// A rectangle pick (`rect` in points). Hits carry no world point (`world` = origin,
    /// `depth` = 0).
    fn request_region(
        &mut self,
        rect: egui::Rect,
        filter: EntityFilter,
        mode: RegionMode,
    ) -> PickTicket;
    /// The result of a request, exactly once (`None` while pending and for unknown tickets).
    fn poll(&mut self, t: PickTicket) -> Option<Vec<PickHitUi>>;
    /// Once per frame before anything polls (additive): collect finished picks and map them
    /// through the scene's pick map.
    fn begin_frame(&mut self, _frame: &PickFrame<'_>) {}
    /// The camera that requests made from now on refer to (additive; the router calls it each
    /// frame before routing input).
    fn set_view(&mut self, _view: &CameraView) {}
    /// Requests not yet polled (additive; idle detection).
    fn pending(&self) -> usize {
        0
    }
}

#[derive(Clone, Debug)]
enum CpuRequest {
    Ray(Ray3, PickMode),
    Point {
        ray: Ray3,
        view: Box<CameraView>,
        radius_pt: f32,
        filter: EntityFilter,
        mode: PickMode,
    },
    Region {
        view: Box<CameraView>,
        rect: egui::Rect,
        filter: EntityFilter,
        mode: RegionMode,
    },
}

/// CPU picking: ray casts over the scene adapter's display meshes
/// ([`SceneAdapter::raycast`]), mapped to document targets through its
/// [`PickMap`](crate::scene::PickMap).
///
/// Point picks report faces and, when the filter has edges or vertices, the sub-entity within
/// the pick radius of the hit point, in front of the face it lies on: on mesh bodies the
/// triangle edge (`3 × triangle + corner`) or mesh vertex (UI-04), on solid bodies the B-rep
/// edge (UI-22). B-rep vertices need S10.
///
/// Asynchronous like S10's: a [`PickService`] request is answered in the next
/// [`PickService::begin_frame`] (one frame later). The UI-20 world-ray API
/// ([`Self::request_ray`] + the inherent [`Self::poll`] with the scene) stays for tests and
/// select-through. Call the trait's `poll` as `PickService::poll(&mut svc, t)`.
#[derive(Debug, Default)]
pub struct CpuPickService {
    next: u64,
    requests: BTreeMap<PickTicket, CpuRequest>,
    done: BTreeMap<PickTicket, Vec<PickHitUi>>,
    view: Option<CameraView>,
}

impl CpuPickService {
    /// A service with no pending requests.
    pub fn new() -> CpuPickService {
        CpuPickService::default()
    }

    /// Number of requests not yet polled.
    pub fn pending(&self) -> usize {
        self.requests.len() + self.done.len()
    }

    fn ticket(&mut self) -> PickTicket {
        self.next += 1;
        PickTicket(self.next)
    }

    /// Queues a pick along a world ray.
    pub fn request_ray(&mut self, ray: Ray3, mode: PickMode) -> PickTicket {
        let t = self.ticket();
        self.requests.insert(t, CpuRequest::Ray(ray, mode));
        t
    }

    /// The result of a request, exactly once (`None` for an unknown or already polled
    /// ticket). Answers every kind of request against `scene` now.
    pub fn poll(&mut self, scene: &SceneAdapter, t: PickTicket) -> Option<Vec<PickHitUi>> {
        if let Some(done) = self.done.remove(&t) {
            return Some(done);
        }
        let req = self.requests.remove(&t)?;
        Some(Self::answer(scene, &req))
    }

    /// Picks now: the hits along `ray` in document terms, front to back.
    pub fn pick(scene: &SceneAdapter, ray: &Ray3, mode: PickMode) -> Vec<PickHitUi> {
        scene
            .raycast(ray, mode)
            .into_iter()
            .filter_map(|h| {
                Some(PickHitUi {
                    target: scene.pick_map().target(h.key, Some(h.sub))?,
                    world: h.world,
                    #[allow(clippy::cast_possible_truncation, reason = "S10 depth is f32")]
                    depth: h.t as f32,
                })
            })
            .collect()
    }

    /// The mesh vertex, triangle edge or B-rep edge of a face hit within `radius_pt` of the hit
    /// point (vertices first), if the filter wants it.
    fn near_sub(
        scene: &SceneAdapter,
        h: &crate::scene::SceneHit,
        view: &CameraView,
        radius_pt: f32,
        filter: EntityFilter,
    ) -> Option<PickTarget> {
        let face = h.sub.index;
        let r = f64::from(radius_pt) * view.world_per_point(h.world);
        let sub = |kind, index| {
            scene
                .pick_map()
                .target(h.key, Some(RenderSub { kind, index }))
        };
        let Some((corners, verts)) = scene.mesh_triangle(h.key, face) else {
            if !filter.contains(EntityFilter::EDGE) {
                return None;
            }
            return sub(RenderSubKind::Edge, scene.edge_near(h.key, h.world, r)?);
        };
        if filter.contains(EntityFilter::VERTEX)
            && let Some(k) = (0..3)
                .map(|k| (k, (corners[k] - h.world).length()))
                .filter(|(_, d)| *d <= r)
                .min_by(|a, b| a.1.total_cmp(&b.1))
                .map(|(k, _)| k)
        {
            return sub(RenderSubKind::Vertex, verts[k]);
        }
        if filter.contains(EntityFilter::EDGE)
            && let Some(k) = (0..3)
                .map(|k| {
                    (
                        k,
                        segment_distance(h.world, corners[k], corners[(k + 1) % 3]),
                    )
                })
                .filter(|(_, d)| *d <= r)
                .min_by(|a, b| a.1.total_cmp(&b.1))
                .map(|(k, _)| k)
        {
            #[allow(clippy::cast_possible_truncation, reason = "k < 3")]
            return sub(RenderSubKind::Edge, face * 3 + k as u32);
        }
        None
    }

    fn answer(scene: &SceneAdapter, req: &CpuRequest) -> Vec<PickHitUi> {
        match req {
            CpuRequest::Ray(ray, mode) => Self::pick(scene, ray, *mode),
            CpuRequest::Point {
                ray,
                view,
                radius_pt,
                filter,
                mode,
            } => {
                let subs = filter.intersects(EntityFilter::EDGE | EntityFilter::VERTEX);
                let mut hits = Vec::new();
                for h in scene.raycast(ray, PickMode::AllAlongRay) {
                    let Some(target) = scene.pick_map().target(h.key, Some(h.sub)) else {
                        continue;
                    };
                    #[allow(clippy::cast_possible_truncation, reason = "S10 depth is f32")]
                    let depth = h.t as f32;
                    if subs && let Some(t) = Self::near_sub(scene, &h, view, *radius_pt, *filter) {
                        hits.push(PickHitUi {
                            target: t,
                            world: h.world,
                            depth,
                        });
                    }
                    if filter.admits(&target) {
                        hits.push(PickHitUi {
                            target,
                            world: h.world,
                            depth,
                        });
                    }
                }
                if filter.contains(EntityFilter::CONSTRUCTION) {
                    construction::merge(&mut hits, scene, ray, view, *radius_pt);
                }
                if *mode == PickMode::Nearest {
                    hits.truncate(1);
                }
                hits
            }
            CpuRequest::Region {
                view,
                rect,
                filter,
                mode,
            } => scene
                .region(|p| view.project(p), *rect, *mode)
                .into_iter()
                .filter_map(|k| scene.pick_map().target(k, None))
                .filter(|t| filter.admits(t))
                .map(|target| PickHitUi {
                    target,
                    world: Point3::ZERO,
                    depth: 0.0,
                })
                .collect(),
        }
    }
}

impl PickService for CpuPickService {
    fn request_point(
        &mut self,
        pos: egui::Pos2,
        radius_pt: f32,
        filter: EntityFilter,
        mode: PickMode,
    ) -> PickTicket {
        let t = self.ticket();
        match &self.view {
            Some(v) => {
                let ray = v.ray(pos);
                self.requests.insert(
                    t,
                    CpuRequest::Point {
                        ray,
                        view: Box::new(v.clone()),
                        radius_pt,
                        filter,
                        mode,
                    },
                );
            }
            None => {
                self.done.insert(t, Vec::new());
            }
        }
        t
    }

    fn request_region(
        &mut self,
        rect: egui::Rect,
        filter: EntityFilter,
        mode: RegionMode,
    ) -> PickTicket {
        let t = self.ticket();
        match &self.view {
            Some(v) => {
                let view = Box::new(v.clone());
                self.requests.insert(
                    t,
                    CpuRequest::Region {
                        view,
                        rect,
                        filter,
                        mode,
                    },
                );
            }
            None => {
                self.done.insert(t, Vec::new());
            }
        }
        t
    }

    fn poll(&mut self, t: PickTicket) -> Option<Vec<PickHitUi>> {
        self.done.remove(&t)
    }

    fn begin_frame(&mut self, frame: &PickFrame<'_>) {
        if let Some(v) = frame.view {
            self.view = Some(v.clone());
        }
        let keys: Vec<PickTicket> = self
            .requests
            .iter()
            .filter(|(_, r)| !matches!(r, CpuRequest::Ray(..)))
            .map(|(t, _)| *t)
            .collect();
        for t in keys {
            if let Some(req) = self.requests.remove(&t) {
                let hits = Self::answer(frame.scene, &req);
                self.done.insert(t, hits);
            }
        }
    }

    fn set_view(&mut self, view: &CameraView) {
        self.view = Some(view.clone());
    }

    fn pending(&self) -> usize {
        CpuPickService::pending(self)
    }
}

/// CPU hits on construction geometry (CON-04): what the RND-07 overlay handles give on the GPU.
mod construction {
    use amg_core::{Point3, Ray3};
    use amg_features::construct::display::ConstructionDisplay;

    use super::{PickHitUi, PickTarget};
    use crate::input::CameraView;
    use crate::scene::SceneAdapter;

    /// Smallest pick radius of axes and points, points.
    const MIN_RADIUS_PT: f32 = 5.0;

    /// The hit of `ray` on `shape`, with `radius` giving the pick radius (mm) at a point of the
    /// shape: the distance along the ray, the point and the sort bias. Axes and points are
    /// ordered as if nearer by the radius, so they win over a plane or face they lie on.
    fn hit(
        shape: &ConstructionDisplay,
        ray: &Ray3,
        radius: impl Fn(Point3) -> f64,
    ) -> Option<(f64, Point3, f64)> {
        let (o, d) = (ray.origin, ray.dir.get());
        match shape {
            ConstructionDisplay::PlaneSquare { frame, half_size } => {
                let denom = d.dot(frame.normal.get());
                if denom.abs() < 1e-12 {
                    return None;
                }
                let t = (frame.origin - o).dot(frame.normal.get()) / denom;
                let p = o + d * t;
                let uv = frame.to_local(p);
                (t >= 0.0 && uv.x.abs() <= *half_size && uv.y.abs() <= *half_size)
                    .then_some((t, p, 0.0))
            }
            ConstructionDisplay::PointMarker(p) => {
                let t = (*p - o).dot(d);
                let r = radius(*p);
                (t >= 0.0 && (o + d * t - *p).length() <= r).then_some((t, *p, r))
            }
            ConstructionDisplay::AxisSegment(s) => {
                // The segment parameter of the closest approach of the two lines.
                let e = s.b - s.a;
                let w = s.a - o;
                let (de, ee) = (d.dot(e), e.length_squared());
                let denom = ee - de * de;
                let u = if denom > 1e-12 * ee {
                    ((d.dot(w) * de - e.dot(w)) / denom).clamp(0.0, 1.0)
                } else {
                    0.5
                };
                let p = s.a + e * u;
                let t = (p - o).dot(d);
                let r = radius(p);
                (t >= 0.0 && (o + d * t - p).length() <= r).then_some((t, p, r))
            }
        }
    }

    /// Adds the construction hits to the mesh `hits`, front to back.
    pub(super) fn merge(
        hits: &mut Vec<PickHitUi>,
        scene: &SceneAdapter,
        ray: &Ray3,
        view: &CameraView,
        radius_pt: f32,
    ) {
        let radius_pt = f64::from(radius_pt.max(MIN_RADIUS_PT));
        let found: Vec<(f64, PickHitUi)> = scene
            .construction_picks()
            .iter()
            .filter_map(|c| {
                let (t, world, bias) = hit(&c.shape, ray, |p| radius_pt * view.world_per_point(p))?;
                #[allow(clippy::cast_possible_truncation, reason = "S10 depth is f32")]
                let depth = t as f32;
                let hit = PickHitUi {
                    target: PickTarget::Construction(c.id),
                    world,
                    depth,
                };
                Some((t - bias, hit))
            })
            .collect();
        if found.is_empty() {
            return;
        }
        let mut all: Vec<(f64, PickHitUi)> = std::mem::take(hits)
            .into_iter()
            .map(|h| (f64::from(h.depth), h))
            .collect();
        all.extend(found);
        // Stable: mesh hits at one depth keep their sub-entity-first order, and a plane on a
        // face (equal depth) comes after the face.
        all.sort_by(|a, b| a.0.total_cmp(&b.0));
        // Geometry seen through several occurrences is one target.
        for (_, h) in all {
            let repeated = matches!(h.target, PickTarget::Construction(_))
                && hits.iter().any(|o| o.target == h.target);
            if !repeated {
                hits.push(h);
            }
        }
    }

    #[cfg(test)]
    mod tests {
        #![allow(
            clippy::expect_used,
            clippy::float_cmp,
            reason = "test code, exact values"
        )]
        use super::*;
        use amg_core::{Dir3, Plane, Segment3};

        fn down(x: f64, y: f64) -> Ray3 {
            Ray3 {
                origin: Point3::new(x, y, 100.0),
                dir: -Dir3::Z,
            }
        }

        #[test]
        fn planes_axes_and_points_are_hit_within_their_extent() {
            let square = ConstructionDisplay::PlaneSquare {
                frame: Plane::XY,
                half_size: 10.0,
            };
            let r = |_| 0.5;
            let (t, p, bias) = hit(&square, &down(9.0, -9.0), r).expect("inside the square");
            assert_eq!((t, p, bias), (100.0, Point3::new(9.0, -9.0, 0.0), 0.0));
            assert!(hit(&square, &down(10.5, 0.0), r).is_none());
            let axis = ConstructionDisplay::AxisSegment(Segment3 {
                a: Point3::new(-10.0, 0.0, 5.0),
                b: Point3::new(10.0, 0.0, 5.0),
            });
            let (t, p, bias) = hit(&axis, &down(3.0, 0.4), r).expect("near the axis");
            assert!((t - 95.0).abs() < 1e-9 && (p - Point3::new(3.0, 0.0, 5.0)).length() < 1e-9);
            assert_eq!(bias, 0.5);
            assert!(hit(&axis, &down(3.0, 0.6), r).is_none());
            assert!(hit(&axis, &down(10.6, 0.0), r).is_none(), "past the end");
            // An axis seen end-on is hit.
            let end_on = ConstructionDisplay::AxisSegment(Segment3 {
                a: Point3::new(0.0, 0.0, -5.0),
                b: Point3::new(0.0, 0.0, 5.0),
            });
            assert!(hit(&end_on, &down(0.1, 0.0), r).is_some());
            let point = ConstructionDisplay::PointMarker(Point3::new(1.0, 1.0, 1.0));
            assert!(hit(&point, &down(1.3, 1.0), r).is_some());
            assert!(hit(&point, &down(1.7, 1.0), r).is_none());
        }
    }
}

/// Distance from `p` to the segment `a`–`b`, mm.
pub(crate) fn segment_distance(p: Point3, a: Point3, b: Point3) -> f64 {
    let ab = b - a;
    let len2 = ab.length_squared();
    let t = if len2 > 0.0 {
        ((p - a).dot(ab) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (p - (a + ab * t)).length()
}

/// Reaches the S10 [`Renderer`] for one call (egui's renderer lock, or a test cell).
pub type RendererAccess = Box<dyn FnMut(&mut dyn FnMut(&mut Renderer))>;

#[derive(Clone, Debug)]
struct GpuRequest {
    ticket: PickTicket,
    ray: Option<Ray3>,
    filter: EntityFilter,
    mode: PickMode,
}

/// [`PickService`] over S10's GPU picking (`Renderer::request_pick` / `poll_picks`, RND-04).
/// Point picks with [`PickMode::AllAlongRay`] (select-through) go to a [`CpuPickService`]
/// (S10 returns visible hits only).
pub struct GpuPickService {
    access: RendererAccess,
    view_id: ViewId,
    view: Option<CameraView>,
    next: u64,
    in_flight: HashMap<amg_render::PickTicket, GpuRequest>,
    raw: Vec<(GpuRequest, Result<PickResult, amg_render::RenderError>)>,
    done: BTreeMap<PickTicket, Vec<PickHitUi>>,
    cpu: CpuPickService,
    cpu_tickets: BTreeMap<PickTicket, PickTicket>,
}

impl std::fmt::Debug for GpuPickService {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GpuPickService")
            .field("view", &self.view_id)
            .field("in_flight", &self.in_flight.len())
            .field("done", &self.done.len())
            .finish_non_exhaustive()
    }
}

impl GpuPickService {
    /// Picks in `view` of the renderer reached through `access`.
    pub fn new(access: RendererAccess, view: ViewId) -> GpuPickService {
        GpuPickService {
            access,
            view_id: view,
            view: None,
            next: 0,
            in_flight: HashMap::new(),
            raw: Vec::new(),
            done: BTreeMap::new(),
            cpu: CpuPickService::new(),
            cpu_tickets: BTreeMap::new(),
        }
    }

    /// Picks in `view` of the renderer installed in egui (`egui_glue::install`).
    pub fn egui(rs: egui_wgpu::RenderState, view: ViewId) -> GpuPickService {
        GpuPickService::new(
            Box::new(move |f| {
                amg_render::egui_glue::with_renderer(&rs, |r| f(r));
            }),
            view,
        )
    }

    fn ticket(&mut self) -> PickTicket {
        self.next += 1;
        PickTicket(self.next)
    }

    fn submit(&mut self, req: PickRequest, mut pending: GpuRequest) -> PickTicket {
        let ticket = self.ticket();
        pending.ticket = ticket;
        let view = self.view_id;
        let mut gpu_ticket = None;
        (self.access)(&mut |r| gpu_ticket = Some(r.request_pick(view, req)));
        match gpu_ticket {
            Some(t) => {
                self.in_flight.insert(t, pending);
            }
            None => {
                self.done.insert(ticket, Vec::new());
            }
        }
        ticket
    }

    /// An answer for a request made before the first camera snapshot: nothing.
    fn empty(&mut self) -> PickTicket {
        let t = self.ticket();
        self.done.insert(t, Vec::new());
        t
    }
}

/// The S10 kinds drawn for an entity filter.
fn render_filter(f: EntityFilter) -> PickFilter {
    let mut out = PickFilter::NONE;
    if f.intersects(EntityFilter::BODY) {
        out = out | PickFilter::BODIES | PickFilter::FACES;
    }
    if f.intersects(EntityFilter::FACE | EntityFilter::MESH_FACE_GROUP) {
        out = out | PickFilter::FACES;
    }
    if f.intersects(EntityFilter::EDGE) {
        out = out | PickFilter::EDGES;
    }
    if f.intersects(EntityFilter::VERTEX) {
        out = out | PickFilter::VERTICES;
    }
    if f.intersects(
        EntityFilter::HANDLE
            | EntityFilter::SKETCH
            | EntityFilter::SKETCH_CURVE
            | EntityFilter::SKETCH_POINT
            | EntityFilter::PROFILE
            | EntityFilter::CONSTRUCTION,
    ) {
        out = out | PickFilter::HANDLES;
    }
    out
}

/// An S10 sub-id in adapter terms (`BODY_SUB`: the whole owner).
fn render_sub(kind: SubKind, sub: u32) -> Option<RenderSub> {
    if sub == amg_render::BODY_SUB {
        return None;
    }
    let kind = match kind {
        SubKind::Face => RenderSubKind::Face,
        SubKind::Edge => RenderSubKind::Edge,
        SubKind::Vertex => RenderSubKind::Vertex,
        SubKind::Handle => RenderSubKind::Handle,
    };
    Some(RenderSub { kind, index: sub })
}

impl PickService for GpuPickService {
    fn request_point(
        &mut self,
        pos: egui::Pos2,
        radius_pt: f32,
        filter: EntityFilter,
        mode: PickMode,
    ) -> PickTicket {
        let Some(view) = self.view.clone() else {
            return self.empty();
        };
        if mode == PickMode::AllAlongRay {
            let ours = self.ticket();
            let cpu = PickService::request_point(&mut self.cpu, pos, radius_pt, filter, mode);
            self.cpu_tickets.insert(cpu, ours);
            return ours;
        }
        let px = view.to_px(pos);
        #[allow(clippy::cast_possible_truncation, reason = "S10 takes f32 pixels")]
        let req = PickRequest {
            region: PickRegion::Point {
                px: [px[0] as f32, px[1] as f32],
            },
            filter: render_filter(filter),
        };
        self.submit(
            req,
            GpuRequest {
                ticket: PickTicket(0),
                ray: Some(view.ray(pos)),
                filter,
                mode,
            },
        )
    }

    fn request_region(
        &mut self,
        rect: egui::Rect,
        filter: EntityFilter,
        mode: RegionMode,
    ) -> PickTicket {
        let Some(view) = self.view.clone() else {
            return self.empty();
        };
        let a = view.to_px(rect.min);
        let b = view.to_px(rect.max);
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            reason = "clamped to the view size"
        )]
        let clamp = |v: f64, max: u32| v.clamp(0.0, f64::from(max)) as u32;
        let [w, h] = view.size_px;
        let min = [clamp(a[0].min(b[0]), w), clamp(a[1].min(b[1]), h)];
        let max = [
            clamp(a[0].max(b[0]).ceil(), w),
            clamp(a[1].max(b[1]).ceil(), h),
        ];
        let rect_mode = match mode {
            RegionMode::Window => RectMode::Window,
            RegionMode::Crossing => RectMode::Crossing,
        };
        let req = PickRequest::rect(min, max, rect_mode, render_filter(filter));
        self.submit(
            req,
            GpuRequest {
                ticket: PickTicket(0),
                ray: None,
                filter,
                mode: PickMode::AllAlongRay,
            },
        )
    }

    fn poll(&mut self, t: PickTicket) -> Option<Vec<PickHitUi>> {
        self.done.remove(&t)
    }

    fn begin_frame(&mut self, frame: &PickFrame<'_>) {
        if let Some(v) = frame.view {
            self.view = Some(v.clone());
        }
        if !self.in_flight.is_empty() {
            let mut results = Vec::new();
            (self.access)(&mut |r| results = r.poll_picks());
            for (t, res) in results {
                if let Some(req) = self.in_flight.remove(&t) {
                    self.raw.push((req, res));
                }
            }
        }
        let map = frame.scene.pick_map();
        for (req, res) in std::mem::take(&mut self.raw) {
            let hits = match res {
                Ok(PickResult::Point(hits)) => {
                    let mut out: Vec<PickHitUi> = hits
                        .iter()
                        .filter_map(|h| {
                            let key = RenderEntityKey(h.pick.0);
                            let target = map.target(key, render_sub(h.kind, h.sub))?;
                            let depth = req
                                .ray
                                .map_or(0.0, |r| (h.world - r.origin).dot(r.dir.get()));
                            #[allow(clippy::cast_possible_truncation, reason = "f32 depth")]
                            let hit = PickHitUi {
                                target,
                                world: h.world,
                                depth: depth as f32,
                            };
                            req.filter.admits(&target).then_some(hit)
                        })
                        .collect();
                    if req.mode == PickMode::Nearest {
                        out.truncate(1);
                    }
                    out
                }
                Ok(PickResult::Rect(items)) => items
                    .iter()
                    .filter_map(|(pick, kind, sub)| {
                        map.target(RenderEntityKey(pick.0), render_sub(*kind, *sub))
                    })
                    .filter(|t| req.filter.admits(t))
                    .map(|target| PickHitUi {
                        target,
                        world: Point3::ZERO,
                        depth: 0.0,
                    })
                    .collect(),
                Err(e) => {
                    tracing::warn!("GPU pick failed: {e}");
                    Vec::new()
                }
            };
            self.done.insert(req.ticket, hits);
        }
        self.cpu.begin_frame(frame);
        let cpu_done: Vec<(PickTicket, PickTicket)> =
            self.cpu_tickets.iter().map(|(c, o)| (*c, *o)).collect();
        for (c, ours) in cpu_done {
            if let Some(hits) = PickService::poll(&mut self.cpu, c) {
                self.cpu_tickets.remove(&c);
                self.done.insert(ours, hits);
            }
        }
    }

    fn set_view(&mut self, view: &CameraView) {
        self.view = Some(view.clone());
        self.cpu.set_view(view);
    }

    fn pending(&self) -> usize {
        self.in_flight.len() + self.raw.len() + self.done.len() + self.cpu_tickets.len()
    }
}
