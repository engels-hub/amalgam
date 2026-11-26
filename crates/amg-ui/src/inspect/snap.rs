//! From a picked mesh face to a measure operand (S05 §2 "Snapping", "Smart selection").
//! Solids, construction, origin and sketch geometry are in [`super::targets`].
//!
//! 1. RND-04 (or the CPU pick service) gives the picked face and the world pick ray.
//! 2. [`snap_pick`] snaps on that face with INS-01's `snap_on_face`: to a vertex within
//!    [`VERTEX_SNAP_PX`] pixels, else to an edge within [`EDGE_SNAP_PX`] pixels, else the surface
//!    point. Pixels become millimetres through the camera's pixel size at the hit
//!    ([`mm_per_px`]). Snapping radii of kinds the [`MeasureFilter`] cannot use are off.
//! 3. The snap and the filter give a [`Choice`] (vertex, edge, face, point or body).
//! 4. [`resolve`] turns the choice into an INS-01 [`Operand`]: with smart selection a face grows
//!    to its coplanar region and an edge to its feature chain (O(n): run in a job, see
//!    [`needs_job`]); Alt-click keeps the single triangle / edge.

use std::sync::Arc;

use amg_core::{Point3, Ray3, Transform};
use amg_doc::BodyId;
use amg_features::inspect::InspectError;
use amg_features::inspect::measure::{
    MeshElement, MeshOperand, Operand, OperandKind, RegionOpts, SnapOpts, Snapped,
    grow_feature_chain, grow_planar_region, snap_on_face,
};
use amg_jobs::JobCtx;
use amg_mesh::{FaceId, IndexedMesh, MeshBody};

use crate::input::CameraView;
use crate::pick::PickTarget;

/// Vertex snap radius, physical pixels (S05 §2 `snap_px`).
pub const VERTEX_SNAP_PX: f64 = 8.0;
/// Edge snap radius, physical pixels.
pub const EDGE_SNAP_PX: f64 = 5.0;
/// Feature-edge angle of smart edge chains (S05 §2, 30°).
pub const FEATURE_ANGLE: f64 = std::f64::consts::PI / 6.0;

/// What a pick may become (the dialog's selection filter). Several kinds may be on; a pick
/// becomes the first that applies in the order vertex, edge, face, point, body.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[allow(
    clippy::struct_excessive_bools,
    reason = "five independent filter check boxes"
)]
pub struct MeasureFilter {
    /// Whole bodies.
    pub body: bool,
    /// Faces (one triangle, or a smart planar region).
    pub face: bool,
    /// Edges (one triangle edge, or a smart feature chain).
    pub edge: bool,
    /// Mesh vertices.
    pub vertex: bool,
    /// Free points on the surface (snapped to vertices and edges).
    pub point: bool,
}

impl Default for MeasureFilter {
    /// Faces, edges and vertices (Fusion's default).
    fn default() -> Self {
        MeasureFilter {
            body: false,
            face: true,
            edge: true,
            vertex: true,
            point: false,
        }
    }
}

impl MeasureFilter {
    /// Only points (3-point circle mode).
    pub const POINTS: MeasureFilter = MeasureFilter {
        body: false,
        face: false,
        edge: false,
        vertex: false,
        point: true,
    };

    /// Nothing selectable.
    pub fn is_empty(self) -> bool {
        !(self.body || self.face || self.edge || self.vertex || self.point)
    }

    /// Snap radii in mm for `mm_per_px`: a kind that the filter cannot use does not snap.
    pub fn snap_opts(self, mm_per_px: f64) -> SnapOpts {
        let off = -1.0;
        SnapOpts {
            vertex_radius: if self.vertex || self.point {
                VERTEX_SNAP_PX * mm_per_px
            } else {
                off
            },
            edge_radius: if self.edge || self.point {
                EDGE_SNAP_PX * mm_per_px
            } else {
                off
            },
        }
    }
}

/// What a pick selects, before smart growth.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Choice {
    /// A mesh vertex.
    Vertex(u32),
    /// The triangle edge `a → b` (smart: its feature chain).
    Edge {
        /// First vertex.
        a: u32,
        /// Second vertex.
        b: u32,
    },
    /// The picked triangle (smart: its coplanar region).
    Face,
    /// A world point (snapped).
    Point(Point3),
    /// The whole body.
    Body,
}

/// One pick on a mesh body, ready to be resolved.
#[derive(Clone, Debug)]
pub struct MeshPick {
    /// The body.
    pub body: BodyId,
    /// The displayed instance it was picked through.
    pub instance: u32,
    /// Its mesh.
    pub mesh: Arc<IndexedMesh>,
    /// Body → world (the instance transform).
    pub to_world: Transform,
    /// The picked face.
    pub face: FaceId,
    /// The snapped world point (the marker position).
    pub at: Point3,
    /// What it selects.
    pub choice: Choice,
    /// Grow faces and edges (smart selection, and no Alt).
    pub smart: bool,
}

/// A resolved pick: the operand and what the viewport shows for it.
#[derive(Clone, Debug)]
pub struct Resolved {
    /// The INS-01 operand.
    pub operand: Operand,
    /// The body (None for free points).
    pub body: Option<(BodyId, u32)>,
    /// The snapped world point of the pick.
    pub at: Point3,
    /// Faces to highlight (face picks and regions, body-local).
    pub faces: Option<Arc<[FaceId]>>,
    /// World polyline of an edge or chain (closed chains repeat the first point).
    pub chain: Vec<Point3>,
    /// What the viewport highlights, for picks that are not mesh elements (INS-07).
    pub target: Option<PickTarget>,
    /// What the dialog calls the selection (INS-07; mesh elements describe themselves).
    pub label: Option<String>,
    /// Whether `at` is a picked point, drawn as a marker (not for geometry chosen without a
    /// point).
    pub marked: bool,
}

impl Resolved {
    /// The operand kind.
    pub fn kind(&self) -> OperandKind {
        self.operand.kind()
    }
}

/// Millimetres per physical pixel at `p` in `view`.
pub fn mm_per_px(view: &CameraView, p: Point3) -> f64 {
    view.camera.pixel_size_at(p, view.size_px)
}

/// Snaps a pick on `face` and classifies it under `filter` (O(1), UI thread). `None` if the
/// filter is empty or the ray misses the face plane.
pub fn snap_pick(
    mesh: &IndexedMesh,
    to_world: &Transform,
    face: FaceId,
    ray: &Ray3,
    mm_per_px: f64,
    filter: MeasureFilter,
) -> Option<(Choice, Point3)> {
    if filter.is_empty() {
        return None;
    }
    let (snapped, at) = snap_on_face(mesh, to_world, face, ray, &filter.snap_opts(mm_per_px))?;
    let fallback = |at| {
        if filter.face {
            Some(Choice::Face)
        } else if filter.point {
            Some(Choice::Point(at))
        } else if filter.body {
            Some(Choice::Body)
        } else {
            None
        }
    };
    let choice = match snapped {
        Snapped::Vertex(v) if filter.vertex => Some(Choice::Vertex(v)),
        Snapped::Edge { a, b, .. } if filter.edge => Some(Choice::Edge { a, b }),
        Snapped::Vertex(_) | Snapped::Edge { .. } if filter.point => Some(Choice::Point(at)),
        _ => fallback(at),
    }?;
    Some((choice, at))
}

/// True when resolving the pick is O(n) in the mesh (smart growth): run it in a job.
pub fn needs_job(p: &MeshPick) -> bool {
    p.smart && matches!(p.choice, Choice::Face | Choice::Edge { .. })
}

/// The operand of a pick (smart growth included). Cancellable.
///
/// # Errors
/// [`InspectError::Cancelled`], or an INS-01 error for a bad element.
pub fn resolve(p: &MeshPick, ctx: &JobCtx) -> Result<Resolved, InspectError> {
    let m = &*p.mesh;
    let world = |v: u32| p.to_world.transform_point3(m.positions[v as usize]);
    let mut faces = None;
    let mut chain = Vec::new();
    let element = match p.choice {
        Choice::Point(at) => {
            return Ok(Resolved {
                operand: Operand::Point(at),
                body: Some((p.body, p.instance)),
                at,
                faces: None,
                chain,
                target: None,
                label: None,
                marked: true,
            });
        }
        Choice::Vertex(v) => MeshElement::Vertex(v),
        Choice::Body => MeshElement::Body,
        Choice::Face => {
            let region: Arc<[FaceId]> = if p.smart {
                grow_planar_region(m, p.face, &RegionOpts::default(), ctx)?
            } else {
                Arc::from([p.face])
            };
            faces = Some(Arc::clone(&region));
            MeshElement::Faces(region)
        }
        Choice::Edge { a, b } => {
            let e = if p.smart {
                grow_feature_chain(m, a, b, FEATURE_ANGLE, ctx)?
            } else {
                MeshElement::Edge { a, b }
            };
            match &e {
                MeshElement::Edge { a, b } => chain = vec![world(*a), world(*b)],
                MeshElement::Polyline { verts, closed } => {
                    chain = verts.iter().map(|&v| world(v)).collect();
                    if *closed && let Some(&first) = chain.first() {
                        chain.push(first);
                    }
                }
                _ => {}
            }
            e
        }
    };
    Ok(Resolved {
        operand: Operand::Mesh(MeshOperand {
            body: MeshBody::from_arc(Arc::clone(&p.mesh)),
            to_world: p.to_world,
            element,
        }),
        body: Some((p.body, p.instance)),
        at: p.at,
        faces,
        chain,
        target: None,
        label: None,
        marked: true,
    })
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, reason = "test code")]

    use amg_core::{Dir3, Transform, Vec3};
    use amg_render::{Camera, Projection};

    use super::*;

    /// One 100 mm right triangle in z = 0, and a second sharing its hypotenuse.
    fn mesh() -> IndexedMesh {
        IndexedMesh::new(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(100.0, 0.0, 0.0),
                Point3::new(0.0, 100.0, 0.0),
                Point3::new(100.0, 100.0, 0.0),
            ],
            vec![[0, 1, 2], [1, 3, 2]],
        )
        .unwrap()
    }

    /// A synthetic camera looking straight down at `(50, 50, 0)` into an 800 × 600 px view.
    fn view(ppp: f32) -> CameraView {
        let cam = Camera {
            eye: Point3::new(30.0, 30.0, 400.0),
            target: Point3::new(30.0, 30.0, 0.0),
            up: Dir3::Y,
            projection: Projection::Perspective { fov_y: 0.6 },
        };
        CameraView::new(
            cam,
            egui::Rect::from_min_size(egui::pos2(10.0, 20.0), egui::vec2(800.0, 600.0) / ppp),
            ppp,
        )
    }

    /// The screen position `px_off` physical pixels from world point `p` along screen
    /// direction `dir` (unit, y down).
    fn screen(v: &CameraView, p: Point3, dir: egui::Vec2, px_off: f32) -> egui::Pos2 {
        v.project(p).unwrap() + dir.normalized() * (px_off / v.pixels_per_point)
    }

    fn pick(v: &CameraView, pos: egui::Pos2, filter: MeasureFilter) -> Option<(Choice, Point3)> {
        let m = mesh();
        let ray = v.ray(pos);
        // The face under the cursor (the pick service's job): the first triangle.
        let hit = ray.origin + ray.dir.get() * (-ray.origin.z / ray.dir.get().z);
        let face = if hit.x + hit.y <= 100.0 { 0 } else { 1 };
        snap_pick(
            &m,
            &Transform::IDENTITY,
            FaceId(face),
            &ray,
            mm_per_px(v, hit),
            filter,
        )
    }

    #[test]
    fn snaps_to_a_vertex_within_8_px_and_an_edge_within_5_px() {
        for ppp in [1.0, 2.0] {
            let v = view(ppp);
            let f = MeasureFilter::default();
            // Around vertex 0 (screen: x right, y up in the world → down on screen): inside
            // the triangle along the corner's bisector.
            let corner = Point3::ZERO;
            let inward = egui::vec2(1.0, -1.0);
            assert_eq!(
                pick(&v, screen(&v, corner, inward, 7.5), f).map(|c| c.0),
                Some(Choice::Vertex(0)),
                "7.5 px from a vertex snaps to it (ppp {ppp})"
            );
            // 8.5 px from the vertex along edge 0 → 1 stays on the edge.
            let along = egui::vec2(1.0, 0.0);
            let c = pick(&v, screen(&v, corner, along, 8.5), f).unwrap();
            assert!(
                matches!(c.0, Choice::Edge { a: 0, b: 1 }),
                "8.5 px is past the vertex radius: {c:?}"
            );
            // Middle of edge 0 → 1 (y = 0), inside the triangle (screen up).
            let mid = Point3::new(50.0, 0.0, 0.0);
            let up = egui::vec2(0.0, -1.0);
            let c = pick(&v, screen(&v, mid, up, 4.5), f).unwrap();
            assert!(matches!(c.0, Choice::Edge { a: 0, b: 1 }), "4.5 px: {c:?}");
            assert!(c.1.y.abs() < 1e-9, "snapped onto the edge: {:?}", c.1);
            let c = pick(&v, screen(&v, mid, up, 5.5), f).unwrap();
            assert_eq!(c.0, Choice::Face, "5.5 px from the edge keeps the face");
            // The shared hypotenuse from the second triangle's side.
            let hyp = Point3::new(60.0, 40.0, 0.0);
            let out = egui::vec2(1.0, -1.0);
            let c = pick(&v, screen(&v, hyp, out, 3.0), f).unwrap();
            assert!(
                matches!(
                    c.0,
                    Choice::Edge { a: 1, b: 2 } | Choice::Edge { a: 2, b: 1 }
                ),
                "{c:?}"
            );
        }
    }

    #[test]
    fn the_filter_decides_what_a_snap_becomes() {
        let v = view(1.0);
        let corner = Point3::ZERO;
        let near_vertex = screen(&v, corner, egui::vec2(1.0, -1.0), 3.0);
        let only = |f: MeasureFilter| pick(&v, near_vertex, f).map(|c| c.0);
        let none = MeasureFilter {
            body: false,
            face: false,
            edge: false,
            vertex: false,
            point: false,
        };
        assert_eq!(only(none), None);
        // Vertices off: the edge snap wins (vertex radius off).
        let edges = MeasureFilter { edge: true, ..none };
        assert!(matches!(only(edges), Some(Choice::Edge { .. })));
        let faces = MeasureFilter { face: true, ..none };
        assert_eq!(only(faces), Some(Choice::Face));
        let bodies = MeasureFilter { body: true, ..none };
        assert_eq!(only(bodies), Some(Choice::Body));
        // Points snap to the vertex position.
        match pick(&v, near_vertex, MeasureFilter::POINTS) {
            Some((Choice::Point(p), at)) => {
                assert!(p.length() < 1e-9 && at == p, "{p:?}");
            }
            other => panic!("{other:?}"),
        }
        // A point in the open face stays where the ray hits.
        let inside = v.project(Point3::new(30.0, 30.0, 0.0)).unwrap();
        match pick(&v, inside, MeasureFilter::POINTS) {
            Some((Choice::Point(p), _)) => {
                assert!((p - Point3::new(30.0, 30.0, 0.0)).length() < 0.2, "{p:?}");
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn resolve_grows_regions_and_chains_only_when_smart() {
        let m = Arc::new(mesh());
        let base = MeshPick {
            body: BodyId {
                origin: amg_doc::FeatureId(1),
                slot: 0,
            },
            instance: 0,
            mesh: Arc::clone(&m),
            to_world: Transform::from_translation(Vec3::new(0.0, 0.0, 5.0)),
            face: FaceId(0),
            at: Point3::ZERO,
            choice: Choice::Face,
            smart: true,
        };
        let ctx = JobCtx::detached();
        assert!(needs_job(&base));
        let r = resolve(&base, &ctx).unwrap();
        assert_eq!(r.kind(), OperandKind::Faces);
        assert_eq!(r.faces.as_deref(), Some(&[FaceId(0), FaceId(1)][..]));
        let single = MeshPick {
            smart: false,
            ..base.clone()
        };
        assert!(!needs_job(&single));
        assert_eq!(
            resolve(&single, &ctx).unwrap().faces.as_deref(),
            Some(&[FaceId(0)][..])
        );
        let edge = MeshPick {
            choice: Choice::Edge { a: 0, b: 1 },
            smart: false,
            ..base.clone()
        };
        let r = resolve(&edge, &ctx).unwrap();
        assert_eq!(r.kind(), OperandKind::Edge);
        assert_eq!(
            r.chain,
            vec![Point3::new(0.0, 0.0, 5.0), Point3::new(100.0, 0.0, 5.0)]
        );
        let point = MeshPick {
            choice: Choice::Point(Point3::X),
            ..base
        };
        assert!(!needs_job(&point));
        assert_eq!(resolve(&point, &ctx).unwrap().kind(), OperandKind::Point);
    }
}
