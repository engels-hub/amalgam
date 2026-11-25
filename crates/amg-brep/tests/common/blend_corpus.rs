//! The BREP-06 fillet/chamfer corpus: cube edges, edge loops, tangent chains, three-edge vertex
//! blends, cylinders and a concave edge, each with its analytic volume where one exists. Used by
//! `tests/blend.rs` (goldens, determinism), `tests/history.rs` (completeness) and the bench.
#![allow(dead_code, reason = "shared test helper")]

use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, FRAC_PI_6, PI, TAU};

use amg_brep::geom::{Curve2d, Loop2, PlanarProfile, ProfileEdge2, Tag};
use amg_brep::{
    BrepKernel, ChamferSpec, Dir3, EdgeId, FaceId, FilletOpts, FilletShape, FilletSpec, JobCtx,
    KResult, OpResult, Plane, Point2, Point3, PrismExtent, PrismOpts, Shape, SubKind, SubShape,
    Tolerance, Vec3,
};
use amg_core::Quat;

const TOL: Tolerance = Tolerance::DEFAULT;

/// The cube side of most cases.
pub const A: f64 = 10.0;

/// What one case runs.
#[derive(Clone, Debug)]
pub enum BlendOp {
    Fillet(Vec<(EdgeId, FilletSpec)>, FilletOpts),
    Chamfer(Vec<(EdgeId, ChamferSpec)>),
}

/// One corpus case: an input, the op on it, and the analytic volume with its relative bound.
#[derive(Clone, Debug)]
pub struct Case {
    pub name: &'static str,
    pub input: Shape,
    pub op: BlendOp,
    pub analytic: Option<(f64, f64)>,
}

impl Case {
    pub fn run(&self, k: &dyn BrepKernel) -> KResult<OpResult> {
        let ctx = JobCtx::detached();
        match &self.op {
            BlendOp::Fillet(edges, opts) => k.fillet(&self.input, edges, opts, &ctx),
            BlendOp::Chamfer(edges) => k.chamfer(&self.input, edges, &ctx),
        }
    }

    /// The input edges the op names.
    pub fn edges(&self) -> Vec<EdgeId> {
        match &self.op {
            BlendOp::Fillet(edges, _) => edges.iter().map(|(e, _)| *e).collect(),
            BlendOp::Chamfer(edges) => edges.iter().map(|(e, _)| *e).collect(),
        }
    }

    pub fn is_chamfer(&self) -> bool {
        matches!(self.op, BlendOp::Chamfer(_))
    }
}

/// A frame rotated off every axis and moved off the origin.
pub fn tilted() -> Plane {
    let q = Quat::from_euler(amg_core::glam::EulerRot::ZXY, 0.4, -0.7, 1.1);
    Plane {
        origin: Point3::new(3.0, -2.0, 5.0),
        x: Dir3::new_unchecked(q * Vec3::X),
        y: Dir3::new_unchecked(q * Vec3::Y),
        normal: Dir3::new_unchecked(q * Vec3::Z),
    }
}

fn sub(kind: SubKind, index: u32) -> SubShape {
    SubShape { kind, index }
}

/// The one sub-shape of `kind` whose `sub_geom` centroid is within 1e-7 of `at`.
fn sub_at(k: &dyn BrepKernel, s: &Shape, kind: SubKind, at: Point3) -> u32 {
    let hits: Vec<u32> = (0..s.count(kind))
        .filter(|&i| {
            let g = k.sub_geom(s, sub(kind, i)).expect("sub_geom");
            (g.centroid - at).length() < 1e-7
        })
        .collect();
    assert_eq!(hits.len(), 1, "{kind:?} at {at:?}: {hits:?}");
    hits[0]
}

/// The edge whose centroid (midpoint of a line, centre of a full circle) is `at`.
pub fn edge_at(k: &dyn BrepKernel, s: &Shape, at: Point3) -> EdgeId {
    EdgeId(sub_at(k, s, SubKind::Edge, at))
}

/// The face whose centroid is `at`.
pub fn face_at(k: &dyn BrepKernel, s: &Shape, at: Point3) -> FaceId {
    FaceId(sub_at(k, s, SubKind::Face, at))
}

pub fn cube(k: &dyn BrepKernel) -> Shape {
    k.make_box(&Plane::XY, Vec3::splat(A)).expect("box").shape
}

fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

/// The 12 edge midpoints of the cube `[0, A]³`.
pub fn cube_edge_midpoints() -> Vec<Point3> {
    let (h, a) = (A / 2.0, A);
    let mut out = Vec::new();
    for (u, v) in [(0.0, 0.0), (a, 0.0), (0.0, a), (a, a)] {
        out.push(p(h, u, v));
        out.push(p(u, h, v));
        out.push(p(u, v, h));
    }
    out
}

fn loop2(curves: Vec<Curve2d>) -> Loop2 {
    Loop2 {
        edges: curves
            .into_iter()
            .zip(0..)
            .map(|(curve, t)| ProfileEdge2 { curve, tag: Tag(t) })
            .collect(),
    }
}

fn line(a: (f64, f64), b: (f64, f64)) -> Curve2d {
    Curve2d::Line {
        a: Point2::new(a.0, a.1),
        b: Point2::new(b.0, b.1),
    }
}

fn extrude(k: &dyn BrepKernel, outer: Vec<Curve2d>, height: f64) -> Shape {
    let profile = PlanarProfile {
        plane: Plane::XY,
        outer: loop2(outer),
        holes: Vec::new(),
    };
    let face = k
        .make_planar_faces(&[profile], &TOL)
        .expect("face")
        .remove(0)
        .face;
    k.prism(
        &face,
        &PrismExtent::Vector(Vec3::new(0.0, 0.0, height)),
        &PrismOpts {
            taper: 0.0,
            merge_coplanar: false,
        },
        &JobCtx::detached(),
    )
    .expect("prism")
    .shape
}

/// The slot of the chain cases: lines of length [`SLOT_LEN`] joined by half circles of radius
/// [`SLOT_R`], extruded by [`SLOT_H`]. Its top boundary is one tangent chain of 4 edges.
pub const SLOT_LEN: f64 = 10.0;
pub const SLOT_R: f64 = 3.0;
pub const SLOT_H: f64 = 5.0;

pub fn slot_prism(k: &dyn BrepKernel) -> Shape {
    let (len, r) = (SLOT_LEN, SLOT_R);
    extrude(
        k,
        vec![
            line((0.0, -r), (len, -r)),
            Curve2d::Arc {
                center: Point2::new(len, 0.0),
                radius: r,
                start_angle: -FRAC_PI_2,
                sweep: PI,
            },
            line((len, r), (0.0, r)),
            Curve2d::Arc {
                center: Point2::ZERO,
                radius: r,
                start_angle: FRAC_PI_2,
                sweep: PI,
            },
        ],
        SLOT_H,
    )
}

/// An L-shaped block: the square `[0, 10]²` minus `[4, 10]²`, 6 high; its one concave edge is
/// the vertical edge at (4, 4).
pub fn l_block(k: &dyn BrepKernel) -> Shape {
    extrude(
        k,
        vec![
            line((0.0, 0.0), (10.0, 0.0)),
            line((10.0, 0.0), (10.0, 4.0)),
            line((10.0, 4.0), (4.0, 4.0)),
            line((4.0, 4.0), (4.0, 10.0)),
            line((4.0, 10.0), (0.0, 10.0)),
            line((0.0, 10.0), (0.0, 0.0)),
        ],
        6.0,
    )
}

/// Cross-section area of a radius-`r` fillet's removed (or added) spandrel at a right angle.
pub fn spandrel(r: f64) -> f64 {
    (1.0 - FRAC_PI_4) * r * r
}

/// Distance of the spandrel's centroid from either face.
pub fn spandrel_centroid(r: f64) -> f64 {
    r * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI)
}

/// The cube with its 4 top edges filleted by `r` (sharp vertical edges): the top band's
/// horizontal section is a square inset by `r − √(r² − t²)` on every side.
fn top_loop_fillet_volume(a: f64, r: f64) -> f64 {
    let b = a - 2.0 * r;
    a * a * (a - r) + b * b * r + PI * r * r * b + 8.0 * r * r * r / 3.0
}

/// The same with chamfers of `d`: the section is inset by `t`.
fn top_loop_chamfer_volume(a: f64, d: f64) -> f64 {
    a * a * (a - d) + a * a * d - 2.0 * a * d * d + 4.0 * d * d * d / 3.0
}

/// Pappus: the slot prism with its top chain blended, the removed section `area` with its
/// centroid `c` from the side wall (the arcs' centroid path has radius `R − c`).
fn slot_volume(removed_area: f64, c: f64) -> f64 {
    let (len, r, h) = (SLOT_LEN, SLOT_R, SLOT_H);
    (2.0 * r * len + PI * r * r) * h - removed_area * (2.0 * len + TAU * (r - c))
}

/// The corpus, in a fixed order.
pub fn corpus(k: &dyn BrepKernel) -> Vec<Case> {
    let mut out = Vec::new();
    let a = A;
    let h = a / 2.0;
    let cube = cube(k);
    let e = |s: &Shape, at: Point3| edge_at(k, s, at);
    let fillet = |edges: Vec<(EdgeId, FilletSpec)>| BlendOp::Fillet(edges, FilletOpts::default());
    let constant = |s: &Shape, pts: &[Point3], r: f64| {
        fillet(
            pts.iter()
                .map(|&q| (e(s, q), FilletSpec::Constant(r)))
                .collect(),
        )
    };
    let top = [p(h, 0.0, a), p(a, h, a), p(h, a, a), p(0.0, h, a)];
    let corner = [p(h, 0.0, 0.0), p(0.0, h, 0.0), p(0.0, 0.0, h)];
    let front_bottom = p(h, 0.0, 0.0);
    let bottom_face = face_at(k, &cube, p(h, h, 0.0));

    out.push(Case {
        name: "fillet_cube_edge",
        input: cube.clone(),
        op: constant(&cube, &[front_bottom], 2.0),
        analytic: Some((a * a * a - spandrel(2.0) * a, 1e-9)),
    });
    let tilted_box = k
        .make_box(&tilted(), Vec3::new(10.0, 20.0, 30.0))
        .expect("box")
        .shape;
    let t = tilted();
    let along_x = t.origin + 5.0 * t.x.get();
    out.push(Case {
        name: "fillet_tilted_box_edge",
        input: tilted_box.clone(),
        op: constant(&tilted_box, &[along_x], 3.0),
        analytic: Some((6000.0 - spandrel(3.0) * 10.0, 1e-9)),
    });
    let tall = k
        .make_box(&Plane::XY, Vec3::new(10.0, 20.0, 30.0))
        .expect("box")
        .shape;
    out.push(Case {
        name: "fillet_box_long_edge",
        input: tall.clone(),
        op: constant(&tall, &[p(10.0, 20.0, 15.0)], 4.0),
        analytic: Some((6000.0 - spandrel(4.0) * 30.0, 1e-9)),
    });
    out.push(Case {
        name: "fillet_top_loop",
        input: cube.clone(),
        op: constant(&cube, &top, 2.0),
        analytic: Some((top_loop_fillet_volume(a, 2.0), 1e-9)),
    });
    let r = 2.0;
    out.push(Case {
        name: "fillet_corner_three",
        input: cube.clone(),
        op: constant(&cube, &corner, r),
        analytic: Some((
            a * a * a - 3.0 * (a - r) * spandrel(r) - (1.0 - FRAC_PI_6) * r * r * r,
            1e-9,
        )),
    });
    let b = a - 2.0 * r;
    out.push(Case {
        name: "fillet_all_edges",
        input: cube.clone(),
        op: constant(&cube, &cube_edge_midpoints(), r),
        analytic: Some((
            b * b * b + 6.0 * b * b * r + 3.0 * PI * b * r * r + 4.0 / 3.0 * PI * r * r * r,
            1e-9,
        )),
    });
    let slot = slot_prism(k);
    let slot_top_line = p(SLOT_LEN / 2.0, -SLOT_R, SLOT_H);
    out.push(Case {
        name: "fillet_slot_chain",
        input: slot.clone(),
        op: constant(&slot, &[slot_top_line], 1.0),
        analytic: Some((slot_volume(spandrel(1.0), spandrel_centroid(1.0)), 1e-9)),
    });
    out.push(Case {
        name: "fillet_variable_linear",
        input: cube.clone(),
        op: fillet(vec![(
            e(&cube, front_bottom),
            FilletSpec::Variable(vec![(0.0, 1.0), (1.0, 3.0)]),
        )]),
        analytic: None,
    });
    out.push(Case {
        name: "fillet_variable_bump",
        input: cube.clone(),
        op: fillet(vec![(
            e(&cube, front_bottom),
            FilletSpec::Variable(vec![(0.0, 1.0), (0.5, 2.5), (1.0, 1.0)]),
        )]),
        analytic: None,
    });
    out.push(Case {
        name: "fillet_chord",
        input: cube.clone(),
        op: fillet(vec![(
            e(&cube, front_bottom),
            FilletSpec::Chord(2.0 * 2f64.sqrt()),
        )]),
        analytic: Some((a * a * a - spandrel(2.0) * a, 1e-9)),
    });
    let (cr, ch) = (5.0, 10.0);
    let cylinder = k
        .make_cylinder(&Plane::XY, cr, ch, TAU)
        .expect("cylinder")
        .shape;
    let rim = e(&cylinder, p(0.0, 0.0, ch));
    out.push(Case {
        name: "fillet_cylinder_rim",
        input: cylinder.clone(),
        op: fillet(vec![(rim, FilletSpec::Constant(1.0))]),
        analytic: Some((
            PI * cr * cr * ch - spandrel(1.0) * TAU * (cr - spandrel_centroid(1.0)),
            1e-9,
        )),
    });
    let l = l_block(k);
    out.push(Case {
        name: "fillet_concave_edge",
        input: l.clone(),
        op: constant(&l, &[p(4.0, 4.0, 3.0)], 1.0),
        analytic: Some((64.0 * 6.0 + spandrel(1.0) * 6.0, 1e-9)),
    });
    out.push(Case {
        name: "fillet_mixed_radii",
        input: cube.clone(),
        op: fillet(vec![
            (e(&cube, front_bottom), FilletSpec::Constant(1.0)),
            (e(&cube, p(0.0, h, 0.0)), FilletSpec::Constant(2.0)),
        ]),
        analytic: None,
    });
    out.push(Case {
        name: "fillet_quasi_angular",
        input: cube.clone(),
        op: BlendOp::Fillet(
            vec![(e(&cube, front_bottom), FilletSpec::Constant(2.0))],
            FilletOpts {
                shape: FilletShape::QuasiAngular,
                ..FilletOpts::default()
            },
        ),
        analytic: Some((a * a * a - spandrel(2.0) * a, 1e-6)),
    });
    out.push(Case {
        name: "chamfer_cube_edge",
        input: cube.clone(),
        op: BlendOp::Chamfer(vec![(e(&cube, front_bottom), ChamferSpec::Equal(2.0))]),
        analytic: Some((a * a * a - 2.0 * a, 1e-9)),
    });
    out.push(Case {
        name: "chamfer_two_distances",
        input: cube.clone(),
        op: BlendOp::Chamfer(vec![(
            e(&cube, front_bottom),
            ChamferSpec::TwoDistances {
                d1: 1.0,
                d2: 3.0,
                face: bottom_face,
            },
        )]),
        analytic: Some((a * a * a - 1.5 * a, 1e-9)),
    });
    let angle = PI / 6.0;
    out.push(Case {
        name: "chamfer_distance_angle",
        input: cube.clone(),
        op: BlendOp::Chamfer(vec![(
            e(&cube, front_bottom),
            ChamferSpec::DistanceAngle {
                d: 2.0,
                angle,
                face: bottom_face,
            },
        )]),
        analytic: Some((a * a * a - 2.0 * angle.tan() * a, 1e-9)),
    });
    out.push(Case {
        name: "chamfer_top_loop",
        input: cube.clone(),
        op: BlendOp::Chamfer(
            top.iter()
                .map(|&q| (e(&cube, q), ChamferSpec::Equal(2.0)))
                .collect(),
        ),
        analytic: Some((top_loop_chamfer_volume(a, 2.0), 1e-9)),
    });
    out.push(Case {
        name: "chamfer_corner_three",
        input: cube.clone(),
        op: BlendOp::Chamfer(
            corner
                .iter()
                .map(|&q| (e(&cube, q), ChamferSpec::Equal(2.0)))
                .collect(),
        ),
        analytic: None,
    });
    out.push(Case {
        name: "chamfer_cylinder_rim",
        input: cylinder.clone(),
        op: BlendOp::Chamfer(vec![(rim, ChamferSpec::Equal(1.0))]),
        analytic: Some((PI * cr * cr * ch - 0.5 * TAU * (cr - 1.0 / 3.0), 1e-9)),
    });
    out.push(Case {
        name: "chamfer_slot_chain",
        input: slot.clone(),
        op: BlendOp::Chamfer(vec![(e(&slot, slot_top_line), ChamferSpec::Equal(1.0))]),
        analytic: Some((slot_volume(0.5, 1.0 / 3.0), 1e-9)),
    });
    out
}
