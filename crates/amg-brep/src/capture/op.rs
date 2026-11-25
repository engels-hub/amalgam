//! `KernelOp`: a kernel call as data, with shape arguments as indices into the dump's inputs.

use amg_core::{Aabb3, Line3, Plane, Point3, Tolerance, Vec3};
use amg_jobs::JobCtx;

use crate::capture::CaptureError;
use crate::error::KResult;
use crate::geom::{self, Curve3, GeomClass};
use crate::kernel::BrepKernel;
use crate::opts::{
    BoolOp, BoolOpts, ChamferSpec, FilletOpts, FilletSpec, HealOpts, OffsetJoin, PrismExtent,
    PrismOpts, SewOpts, ShellOpts, TessQuality, UnifyOpts,
};
use crate::shape::{EdgeId, FaceId, Shape, SubKind, SubShape};

/// A shape argument: index `n` is `inputs/<n>.brep` of the dump.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub struct InputRef(pub u32);

/// Geometric fingerprint of a sub-shape, used to find it again when indices do not survive a
/// reload. KH-01's own version of S02's `SubShapeGeom` (BREP-05 `sub_geom`, not done yet);
/// BREP-05 can replace it with the S02 type, keeping the field names.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SubShapeGeom {
    /// Surface class (faces), curve class (edges) or `Point`.
    pub class: GeomClass,
    /// Area (faces), length (edges; sampled for ellipses and B-splines), 0 (vertices).
    pub measure: f64,
    /// Edge: the point at the middle parameter; face: its bbox centre; vertex: its point.
    pub centroid: Point3,
    /// Bounding box.
    pub bbox: Aabb3,
}

/// Samples for the length of a curved edge without a closed form.
const LENGTH_SAMPLES: u32 = 64;

impl SubShapeGeom {
    /// The fingerprint of `sub` in `s`; `None` when the index is out of range.
    pub fn of(s: &Shape, sub: SubShape) -> Option<SubShapeGeom> {
        if sub.index >= s.count(sub.kind) {
            return None;
        }
        let bbox = s.sub_shape(sub).ok()?.bbox();
        match sub.kind {
            SubKind::Face => {
                let f = FaceId(sub.index);
                Some(SubShapeGeom {
                    class: s.surface(f).class(),
                    measure: s.face_area(f),
                    centroid: bbox.center(),
                    bbox,
                })
            }
            SubKind::Edge => Some(edge_geom(s, EdgeId(sub.index), bbox)),
            SubKind::Vertex => {
                let p = s.vertex_point(crate::VertexId(sub.index));
                Some(SubShapeGeom {
                    class: GeomClass::Point,
                    measure: 0.0,
                    centroid: p,
                    bbox,
                })
            }
        }
    }

    /// Whether `other` is the same sub-shape within `eps` (mm for points, relative for the
    /// measure).
    pub fn matches(&self, other: &SubShapeGeom, eps: f64) -> bool {
        let near = |a: Point3, b: Point3| a.distance(b) <= eps;
        self.class == other.class
            && (self.measure - other.measure).abs() <= eps * self.measure.abs().max(1.0)
            && near(self.centroid, other.centroid)
            && near(self.bbox.min, other.bbox.min)
            && near(self.bbox.max, other.bbox.max)
    }
}

fn edge_geom(s: &Shape, e: EdgeId, bbox: Aabb3) -> SubShapeGeom {
    let curve = s.curve(e);
    let class = match &curve {
        Some((Curve3::Line { .. }, ..)) => GeomClass::Line,
        Some((Curve3::Circle { .. }, ..)) => GeomClass::Circle,
        Some((Curve3::Ellipse { .. }, ..)) => GeomClass::Ellipse,
        _ => GeomClass::Curve,
    };
    let [v0, v1] = s.edge_vertices(e);
    let chord = s.vertex_point(v0).distance(s.vertex_point(v1));
    let (measure, mid) = match &curve {
        Some((c, t0, t1)) => {
            let measure = match c {
                Curve3::Line { .. } => (t1 - t0).abs(),
                Curve3::Circle { radius, .. } => radius * (t1 - t0).abs(),
                _ => sampled_length(c, *t0, *t1).unwrap_or(chord),
            };
            (measure, c.point_at(0.5 * (t0 + t1)))
        }
        None => (0.0, None),
    };
    SubShapeGeom {
        class,
        measure,
        centroid: mid.unwrap_or_else(|| bbox.center()),
        bbox,
    }
}

fn sampled_length(c: &Curve3, t0: f64, t1: f64) -> Option<f64> {
    let mut prev = c.point_at(t0)?;
    let mut len = 0.0;
    for i in 1..=LENGTH_SAMPLES {
        let t = t0 + (t1 - t0) * f64::from(i) / f64::from(LENGTH_SAMPLES);
        let p = c.point_at(t)?;
        len += prev.distance(p);
        prev = p;
    }
    Some(len)
}

/// An edge argument: its index in the input plus its fingerprint, so replay can find it again
/// if the index map of the reloaded input disagrees.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct EdgeRecord {
    /// The edge index in the input shape.
    pub id: EdgeId,
    /// Its fingerprint; `None` when the index was out of range at capture time.
    pub geom: Option<SubShapeGeom>,
}

impl EdgeRecord {
    /// The record of edge `id` of `s`.
    pub fn of(s: &Shape, id: EdgeId) -> EdgeRecord {
        EdgeRecord {
            id,
            geom: SubShapeGeom::of(
                s,
                SubShape {
                    kind: SubKind::Edge,
                    index: id.0,
                },
            ),
        }
    }
}

/// One variant per shape-producing [`BrepKernel`] operation that exists, with S02's parameter
/// types; shape arguments are [`InputRef`]s.
///
/// **Rule** (design note §1.2): every WP that adds a kernel operation adds its variant, its
/// [`KernelOp::run`] arm, the `RecordingKernel` arm and a round-trip test in the same PR. The
/// read-only queries (`check`, `mass_props`, `classify`, `distance`, `closest_point`,
/// `ray_cast`, `eval_surface`, `project_point_on_face`, `edge_ends`, `tessellate_edge`) are
/// forwarded without capture: their failures are argument errors of interactive picking, not
/// kernel failures worth a dump.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "op", rename_all = "snake_case")]
#[non_exhaustive]
pub enum KernelOp {
    /// `make_planar_faces`.
    MakePlanarFaces {
        /// Profiles.
        profiles: Vec<geom::PlanarProfile>,
        /// Tolerance.
        tol: Tolerance,
    },
    /// `make_planar_polygon`.
    MakePlanarPolygon {
        /// Outer loop.
        outer: Vec<Point3>,
        /// Hole loops.
        holes: Vec<Vec<Point3>>,
        /// Tolerance.
        tol: Tolerance,
    },
    /// `make_face`.
    MakeFace {
        /// Surface.
        surface: geom::Surface,
        /// Bounds.
        bounds: geom::FaceBounds,
        /// Tolerance, mm.
        tol: f64,
    },
    /// `make_wire`.
    MakeWire {
        /// Segments.
        segs: Vec<geom::CurveSeg>,
        /// Tolerance.
        tol: Tolerance,
    },
    /// `make_box`.
    MakeBox {
        /// Frame.
        frame: Plane,
        /// Size.
        size: Vec3,
    },
    /// `make_cylinder`.
    MakeCylinder {
        /// Frame.
        frame: Plane,
        /// Radius.
        radius: f64,
        /// Height.
        height: f64,
        /// Sweep angle.
        angle: f64,
    },
    /// `make_cone`.
    MakeCone {
        /// Frame.
        frame: Plane,
        /// Base radius.
        r1: f64,
        /// Top radius.
        r2: f64,
        /// Height.
        height: f64,
        /// Sweep angle.
        angle: f64,
    },
    /// `make_sphere`.
    MakeSphere {
        /// Frame.
        frame: Plane,
        /// Radius.
        radius: f64,
    },
    /// `make_torus`.
    MakeTorus {
        /// Frame.
        frame: Plane,
        /// Major radius.
        major: f64,
        /// Minor radius.
        minor: f64,
        /// Sweep angle.
        angle: f64,
    },
    /// `prism`.
    Prism {
        /// Profile.
        profile: InputRef,
        /// Extent.
        extent: PrismExtent,
        /// Options.
        opts: PrismOpts,
    },
    /// `revol`.
    Revol {
        /// Profile.
        profile: InputRef,
        /// Axis.
        axis: Line3,
        /// Angle.
        angle: f64,
    },
    /// `mirrored`.
    Mirrored {
        /// Shape.
        shape: InputRef,
        /// Mirror plane.
        plane: Plane,
    },
    /// `boolean`.
    Boolean {
        /// Fuse, cut or common.
        kind: BoolOp,
        /// Objects.
        objects: Vec<InputRef>,
        /// Tools.
        tools: Vec<InputRef>,
        /// Options.
        opts: BoolOpts,
    },
    /// `split`.
    Split {
        /// Objects.
        objects: Vec<InputRef>,
        /// Tools.
        tools: Vec<InputRef>,
        /// Options.
        opts: BoolOpts,
    },
    /// `section`.
    Section {
        /// First shape.
        a: InputRef,
        /// Second shape.
        b: InputRef,
    },
    /// `fillet`.
    Fillet {
        /// Shape.
        shape: InputRef,
        /// Edges, with fingerprints, and their specs.
        edges: Vec<(EdgeRecord, FilletSpec)>,
        /// Options.
        opts: FilletOpts,
    },
    /// `chamfer`.
    Chamfer {
        /// Shape.
        shape: InputRef,
        /// Edges, with fingerprints, and their specs (reference faces by index).
        edges: Vec<(EdgeRecord, ChamferSpec)>,
    },
    /// `fillet_max_radius`.
    FilletMaxRadius {
        /// Shape.
        shape: InputRef,
        /// Edges, with fingerprints.
        edges: Vec<EdgeRecord>,
        /// Upper radius.
        hi: f64,
        /// Bisection steps.
        iterations: u32,
    },
    /// `shell`.
    Shell {
        /// Shape.
        shape: InputRef,
        /// Removed faces (by index).
        remove: Vec<FaceId>,
        /// Options (per-face thicknesses by index).
        opts: ShellOpts,
    },
    /// `offset`.
    Offset {
        /// Shape.
        shape: InputRef,
        /// Distance.
        distance: f64,
        /// Join.
        join: OffsetJoin,
        /// Tolerance.
        tol: f64,
    },
    /// `thicken`.
    Thicken {
        /// Shape.
        shape: InputRef,
        /// Thickness.
        thickness: f64,
        /// Join.
        join: OffsetJoin,
    },
    /// `heal`.
    Heal {
        /// Shape.
        shape: InputRef,
        /// Options.
        opts: HealOpts,
    },
    /// `unify_same_domain`.
    UnifySameDomain {
        /// Shape.
        shape: InputRef,
        /// Options.
        opts: UnifyOpts,
    },
    /// `sew`.
    Sew {
        /// The shapes sewn, in order.
        faces: Vec<InputRef>,
        /// Options.
        opts: SewOpts,
    },
    /// `make_solid`.
    MakeSolid {
        /// The shell(s).
        shell: InputRef,
    },
    /// `make_compound`.
    MakeCompound {
        /// The parts, in order.
        parts: Vec<InputRef>,
    },
    /// `interference`.
    Interference {
        /// First shape.
        a: InputRef,
        /// Second shape.
        b: InputRef,
    },
    /// `plane_section`.
    PlaneSection {
        /// Shape.
        shape: InputRef,
        /// Cutting plane.
        plane: Plane,
    },
    /// `project_edges`.
    ProjectEdges {
        /// Shape.
        shape: InputRef,
        /// Edges, with fingerprints.
        edges: Vec<EdgeRecord>,
        /// Projection plane.
        plane: Plane,
    },
    /// `tessellate` (and each failed body of `tessellate_many`).
    Tessellate {
        /// Shape.
        shape: InputRef,
        /// Quality.
        quality: TessQuality,
    },
}

/// `InputRef`s `range.start..range.end`.
pub(crate) fn refs(range: std::ops::Range<usize>) -> Vec<InputRef> {
    range
        .map(|i| InputRef(u32::try_from(i).unwrap_or(u32::MAX)))
        .collect()
}

impl KernelOp {
    /// The op's name in snake case (the serde tag), used in dump directory names.
    pub fn kind(&self) -> &'static str {
        match self {
            KernelOp::MakePlanarFaces { .. } => "make_planar_faces",
            KernelOp::MakePlanarPolygon { .. } => "make_planar_polygon",
            KernelOp::MakeFace { .. } => "make_face",
            KernelOp::MakeWire { .. } => "make_wire",
            KernelOp::MakeBox { .. } => "make_box",
            KernelOp::MakeCylinder { .. } => "make_cylinder",
            KernelOp::MakeCone { .. } => "make_cone",
            KernelOp::MakeSphere { .. } => "make_sphere",
            KernelOp::MakeTorus { .. } => "make_torus",
            KernelOp::Prism { .. } => "prism",
            KernelOp::Revol { .. } => "revol",
            KernelOp::Mirrored { .. } => "mirrored",
            KernelOp::Boolean { .. } => "boolean",
            KernelOp::Split { .. } => "split",
            KernelOp::Section { .. } => "section",
            KernelOp::Fillet { .. } => "fillet",
            KernelOp::Chamfer { .. } => "chamfer",
            KernelOp::FilletMaxRadius { .. } => "fillet_max_radius",
            KernelOp::Shell { .. } => "shell",
            KernelOp::Offset { .. } => "offset",
            KernelOp::Thicken { .. } => "thicken",
            KernelOp::Heal { .. } => "heal",
            KernelOp::UnifySameDomain { .. } => "unify_same_domain",
            KernelOp::Sew { .. } => "sew",
            KernelOp::MakeSolid { .. } => "make_solid",
            KernelOp::MakeCompound { .. } => "make_compound",
            KernelOp::Interference { .. } => "interference",
            KernelOp::PlaneSection { .. } => "plane_section",
            KernelOp::ProjectEdges { .. } => "project_edges",
            KernelOp::Tessellate { .. } => "tessellate",
        }
    }

    /// Every shape argument, in parameter order.
    pub fn inputs(&self) -> Vec<InputRef> {
        match self {
            KernelOp::Prism { profile, .. } | KernelOp::Revol { profile, .. } => vec![*profile],
            KernelOp::Mirrored { shape, .. }
            | KernelOp::PlaneSection { shape, .. }
            | KernelOp::ProjectEdges { shape, .. }
            | KernelOp::Tessellate { shape, .. }
            | KernelOp::Fillet { shape, .. }
            | KernelOp::Chamfer { shape, .. }
            | KernelOp::FilletMaxRadius { shape, .. }
            | KernelOp::Shell { shape, .. }
            | KernelOp::Offset { shape, .. }
            | KernelOp::Thicken { shape, .. }
            | KernelOp::Heal { shape, .. }
            | KernelOp::UnifySameDomain { shape, .. } => vec![*shape],
            KernelOp::MakeSolid { shell } => vec![*shell],
            KernelOp::Sew { faces: shapes, .. } | KernelOp::MakeCompound { parts: shapes } => {
                shapes.clone()
            }
            KernelOp::Boolean { objects, tools, .. } | KernelOp::Split { objects, tools, .. } => {
                objects.iter().chain(tools).copied().collect()
            }
            KernelOp::Section { a, b } | KernelOp::Interference { a, b } => vec![*a, *b],
            _ => Vec::new(),
        }
    }

    /// Checks every edge argument against the fingerprints of the reloaded `inputs`. An edge
    /// whose index now names a different edge is re-matched to the unique edge with its
    /// fingerprint. Returns whether any index changed.
    ///
    /// # Errors
    /// `Unmatched` when an edge without a matching fingerprint has no unique match.
    pub fn reindex(&mut self, inputs: &[Shape]) -> Result<bool, CaptureError> {
        let (shape, edges): (&InputRef, Vec<&mut EdgeRecord>) = match self {
            KernelOp::ProjectEdges { shape, edges, .. }
            | KernelOp::FilletMaxRadius { shape, edges, .. } => (shape, edges.iter_mut().collect()),
            KernelOp::Fillet { shape, edges, .. } => {
                (shape, edges.iter_mut().map(|(e, _)| e).collect())
            }
            KernelOp::Chamfer { shape, edges } => {
                (shape, edges.iter_mut().map(|(e, _)| e).collect())
            }
            _ => return Ok(false),
        };
        let Some(s) = inputs.get(shape.0 as usize) else {
            return Ok(false);
        };
        let eps = 1e-7 * s.bbox().size().length().max(1.0);
        let mut changed = false;
        let mut all: Option<Vec<Option<SubShapeGeom>>> = None;
        for rec in edges {
            let Some(want) = &rec.geom else { continue };
            let here = SubShapeGeom::of(s, edge_sub(rec.id.0));
            if here.is_some_and(|g| g.matches(want, eps)) {
                continue;
            }
            let all = all.get_or_insert_with(|| {
                s.edges()
                    .map(|e| SubShapeGeom::of(s, edge_sub(e.0)))
                    .collect()
            });
            let mut hits = all
                .iter()
                .enumerate()
                .filter(|(_, g)| g.as_ref().is_some_and(|g| g.matches(want, eps)))
                .map(|(i, _)| i);
            match (hits.next(), hits.next()) {
                (Some(i), None) => {
                    rec.id = EdgeId(u32::try_from(i).unwrap_or(u32::MAX));
                    changed = true;
                }
                _ => {
                    return Err(CaptureError::Unmatched {
                        input: shape.0,
                        edge: rec.id.0,
                    });
                }
            }
        }
        Ok(changed)
    }

    /// Runs the op on `k` with `inputs` (index = [`InputRef`]) and returns the result shapes
    /// (empty for ops without one, such as `tessellate`), for the validity check of replay.
    ///
    /// # Errors
    /// The kernel's error. `InvalidInput { what: "inputs" }` when an [`InputRef`] is out of
    /// range.
    pub fn run(&self, k: &dyn BrepKernel, inputs: &[Shape], ctx: &JobCtx) -> KResult<Vec<Shape>> {
        let one = |r: &InputRef| {
            inputs
                .get(r.0 as usize)
                .ok_or_else(|| crate::KernelError::InvalidInput {
                    what: "inputs",
                    reason: format!("input {} is not in the dump", r.0),
                    offending: Vec::new(),
                })
        };
        let many = |rs: &[InputRef]| -> KResult<Vec<Shape>> {
            rs.iter().map(|r| one(r).cloned()).collect()
        };
        let shape = |r: KResult<crate::OpResult>| r.map(|r| vec![r.shape]);
        match self {
            KernelOp::MakePlanarFaces { profiles, tol } => k
                .make_planar_faces(profiles, tol)
                .map(|faces| faces.into_iter().map(|f| f.face).collect()),
            KernelOp::MakePlanarPolygon { outer, holes, tol } => {
                k.make_planar_polygon(outer, holes, tol).map(|s| vec![s])
            }
            KernelOp::MakeFace {
                surface,
                bounds,
                tol,
            } => k.make_face(surface, bounds, *tol).map(|f| vec![f.face]),
            KernelOp::MakeWire { segs, tol } => shape(k.make_wire(segs, tol)),
            KernelOp::MakeBox { frame, size } => shape(k.make_box(frame, *size)),
            KernelOp::MakeCylinder {
                frame,
                radius,
                height,
                angle,
            } => shape(k.make_cylinder(frame, *radius, *height, *angle)),
            KernelOp::MakeCone {
                frame,
                r1,
                r2,
                height,
                angle,
            } => shape(k.make_cone(frame, *r1, *r2, *height, *angle)),
            KernelOp::MakeSphere { frame, radius } => shape(k.make_sphere(frame, *radius)),
            KernelOp::MakeTorus {
                frame,
                major,
                minor,
                angle,
            } => shape(k.make_torus(frame, *major, *minor, *angle)),
            KernelOp::Prism {
                profile,
                extent,
                opts,
            } => shape(k.prism(one(profile)?, extent, opts, ctx)),
            KernelOp::Revol {
                profile,
                axis,
                angle,
            } => shape(k.revol(one(profile)?, axis, *angle, ctx)),
            KernelOp::Mirrored { shape: s, plane } => shape(k.mirrored(one(s)?, plane)),
            KernelOp::Boolean {
                kind,
                objects,
                tools,
                opts,
            } => shape(k.boolean(*kind, &many(objects)?, &many(tools)?, opts, ctx)),
            KernelOp::Split {
                objects,
                tools,
                opts,
            } => shape(k.split(&many(objects)?, &many(tools)?, opts, ctx)),
            KernelOp::Section { a, b } => shape(k.section(one(a)?, one(b)?, ctx)),
            KernelOp::Fillet {
                shape: s,
                edges,
                opts,
            } => {
                let edges: Vec<(EdgeId, FilletSpec)> =
                    edges.iter().map(|(e, f)| (e.id, f.clone())).collect();
                shape(k.fillet(one(s)?, &edges, opts, ctx))
            }
            KernelOp::Chamfer { shape: s, edges } => {
                let edges: Vec<(EdgeId, ChamferSpec)> =
                    edges.iter().map(|(e, c)| (e.id, c.clone())).collect();
                shape(k.chamfer(one(s)?, &edges, ctx))
            }
            KernelOp::FilletMaxRadius {
                shape: s,
                edges,
                hi,
                iterations,
            } => {
                let ids: Vec<EdgeId> = edges.iter().map(|e| e.id).collect();
                k.fillet_max_radius(one(s)?, &ids, *hi, *iterations, ctx)
                    .map(|_| Vec::new())
            }
            KernelOp::Shell {
                shape: s,
                remove,
                opts,
            } => shape(k.shell(one(s)?, remove, opts, ctx)),
            KernelOp::Offset {
                shape: s,
                distance,
                join,
                tol,
            } => shape(k.offset(one(s)?, *distance, *join, *tol, ctx)),
            KernelOp::Thicken {
                shape: s,
                thickness,
                join,
            } => shape(k.thicken(one(s)?, *thickness, *join, ctx)),
            KernelOp::Heal { shape: s, opts } => shape(k.heal(one(s)?, opts, ctx)),
            KernelOp::UnifySameDomain { shape: s, opts } => {
                shape(k.unify_same_domain(one(s)?, opts))
            }
            KernelOp::Sew { faces, opts } => shape(k.sew(&many(faces)?, opts, ctx)),
            KernelOp::MakeSolid { shell } => shape(k.make_solid(one(shell)?)),
            KernelOp::MakeCompound { parts } => k.make_compound(&many(parts)?).map(|c| vec![c]),
            KernelOp::Interference { a, b } => k
                .interference(one(a)?, one(b)?, ctx)
                .map(|r| r.map(|r| r.shape).into_iter().collect()),
            KernelOp::PlaneSection { shape: s, plane } => {
                k.plane_section(one(s)?, plane).map(|_| Vec::new())
            }
            KernelOp::ProjectEdges {
                shape: s,
                edges,
                plane,
            } => {
                let ids: Vec<EdgeId> = edges.iter().map(|e| e.id).collect();
                k.project_edges(one(s)?, &ids, plane).map(|_| Vec::new())
            }
            KernelOp::Tessellate { shape: s, quality } => {
                k.tessellate(one(s)?, quality, ctx).map(|_| Vec::new())
            }
        }
    }
}

fn edge_sub(index: u32) -> SubShape {
    SubShape {
        kind: SubKind::Edge,
        index,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use amg_core::Dir3;

    fn round_trip(op: &KernelOp) {
        let json = serde_json::to_string(op).unwrap();
        assert!(
            json.starts_with(&format!("{{\"op\":\"{}\"", op.kind())),
            "{json}"
        );
        let back: KernelOp = serde_json::from_str(&json).unwrap();
        assert_eq!(&back, op, "{json}");
    }

    /// One round trip per variant (design note §1.2 rule).
    #[test]
    fn every_variant_round_trips() {
        let frame = Plane::XY;
        let tol = Tolerance::DEFAULT;
        let p = Point3::new(1.0, 2.0, 3.0);
        let seg = geom::CurveSeg {
            curve: Curve3::Line {
                origin: Point3::ZERO,
                dir: Dir3::new_unchecked(Vec3::X),
            },
            t0: 0.0,
            t1: 2.0,
            tag: geom::Tag(7),
        };
        let edge = EdgeRecord {
            id: EdgeId(3),
            geom: Some(SubShapeGeom {
                class: GeomClass::Line,
                measure: 2.0,
                centroid: p,
                bbox: Aabb3::new(Point3::ZERO, p),
            }),
        };
        let ops = [
            KernelOp::MakePlanarFaces {
                profiles: Vec::new(),
                tol,
            },
            KernelOp::MakePlanarPolygon {
                outer: vec![p, Point3::ZERO],
                holes: vec![vec![p]],
                tol,
            },
            KernelOp::MakeFace {
                surface: geom::Surface::Plane(frame),
                bounds: geom::FaceBounds::Loops(vec![vec![seg.clone()]]),
                tol: 1e-6,
            },
            KernelOp::MakeWire {
                segs: vec![seg],
                tol,
            },
            KernelOp::MakeBox {
                frame,
                size: Vec3::ONE,
            },
            KernelOp::MakeCylinder {
                frame,
                radius: 1.0,
                height: 2.0,
                angle: 3.0,
            },
            KernelOp::MakeCone {
                frame,
                r1: 1.0,
                r2: 0.0,
                height: 2.0,
                angle: 3.0,
            },
            KernelOp::MakeSphere { frame, radius: 1.0 },
            KernelOp::MakeTorus {
                frame,
                major: 3.0,
                minor: 1.0,
                angle: 1.0,
            },
            KernelOp::Prism {
                profile: InputRef(0),
                extent: PrismExtent::Vector(Vec3::Z),
                opts: PrismOpts {
                    taper: 0.0,
                    merge_coplanar: false,
                },
            },
            KernelOp::Revol {
                profile: InputRef(0),
                axis: Line3 {
                    point: p,
                    dir: Dir3::new_unchecked(Vec3::Y),
                },
                angle: 1.0,
            },
            KernelOp::Mirrored {
                shape: InputRef(0),
                plane: frame,
            },
            KernelOp::Boolean {
                kind: BoolOp::Cut,
                objects: refs(0..1),
                tools: refs(1..3),
                opts: BoolOpts::default(),
            },
            KernelOp::Split {
                objects: refs(0..2),
                tools: Vec::new(),
                opts: BoolOpts::default(),
            },
            KernelOp::Section {
                a: InputRef(0),
                b: InputRef(1),
            },
            KernelOp::Interference {
                a: InputRef(0),
                b: InputRef(1),
            },
            KernelOp::Fillet {
                shape: InputRef(0),
                edges: vec![
                    (edge.clone(), FilletSpec::Constant(1.0)),
                    (
                        EdgeRecord {
                            id: EdgeId(4),
                            geom: None,
                        },
                        FilletSpec::Variable(vec![(0.0, 1.0), (1.0, 2.0)]),
                    ),
                ],
                opts: FilletOpts::default(),
            },
            KernelOp::Chamfer {
                shape: InputRef(0),
                edges: vec![(
                    edge.clone(),
                    ChamferSpec::DistanceAngle {
                        d: 1.0,
                        angle: 0.5,
                        face: FaceId(2),
                    },
                )],
            },
            KernelOp::FilletMaxRadius {
                shape: InputRef(0),
                edges: vec![edge.clone()],
                hi: 5.0,
                iterations: 12,
            },
            KernelOp::Shell {
                shape: InputRef(0),
                remove: vec![FaceId(5)],
                opts: ShellOpts {
                    thickness: -1.0,
                    join: OffsetJoin::Arc,
                    per_face: vec![(FaceId(0), -2.0)],
                    tol: 0.0,
                },
            },
            KernelOp::Offset {
                shape: InputRef(0),
                distance: 0.5,
                join: OffsetJoin::Intersection,
                tol: 1e-6,
            },
            KernelOp::Thicken {
                shape: InputRef(0),
                thickness: -0.25,
                join: OffsetJoin::Arc,
            },
            KernelOp::Heal {
                shape: InputRef(0),
                opts: HealOpts {
                    tol: 1e-4,
                    fix_small_edges: false,
                    fix_orientation: true,
                    max_tol: 1e-2,
                },
            },
            KernelOp::UnifySameDomain {
                shape: InputRef(0),
                opts: UnifyOpts {
                    concat_bsplines: true,
                    ..UnifyOpts::default()
                },
            },
            KernelOp::Sew {
                faces: refs(0..6),
                opts: SewOpts {
                    tol: 1e-3,
                    non_manifold: true,
                    make_solid: false,
                },
            },
            KernelOp::MakeSolid { shell: InputRef(1) },
            KernelOp::MakeCompound { parts: refs(0..2) },
            KernelOp::PlaneSection {
                shape: InputRef(0),
                plane: frame,
            },
            KernelOp::ProjectEdges {
                shape: InputRef(0),
                edges: vec![
                    edge,
                    EdgeRecord {
                        id: EdgeId(9),
                        geom: None,
                    },
                ],
                plane: frame,
            },
            KernelOp::Tessellate {
                shape: InputRef(0),
                quality: TessQuality::display(),
            },
        ];
        for op in &ops {
            round_trip(op);
        }
        let boolean = &ops[12];
        assert_eq!(boolean.inputs(), refs(0..3));
        assert!(ops[4].inputs().is_empty());
    }
}
