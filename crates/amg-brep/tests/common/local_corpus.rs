//! The BREP-16 corpus: drafts, defeatures, face replacements, face splits, body splits (the
//! BREP-04 `split`) and single-face offsets on boxes and cylinders, each with its closed-form
//! volume. Used by `tests/local.rs` (goldens, Done-when items, determinism), `tests/history.rs`
//! (completeness) and the bench.
#![allow(dead_code, reason = "shared test helper")]

use std::f64::consts::{PI, TAU};

use amg_brep::geom::{CurveSeg, FaceBounds, GeomClass, Surface, Tag};
use amg_brep::{
    BoolOp, BoolOpts, BrepKernel, ChamferSpec, DraftOpts, FaceId, FilletSpec, JobCtx, KResult,
    OpResult, Plane, Point3, Shape, SubKind, SubShape, Tolerance, Vec3,
};
use amg_core::Dir3;

/// The box of most cases: `[0, A] × [0, B] × [0, C]`.
pub const A: f64 = 20.0;
pub const B: f64 = 16.0;
pub const C: f64 = 12.0;

/// Cylinder cases: radius and height.
pub const R: f64 = 6.0;
pub const H: f64 = 10.0;

/// Bound for exact (planar, analytic-surface) cases.
pub const EXACT: f64 = 1e-9;

/// What one case runs.
#[derive(Clone, Debug)]
pub enum LocalOp {
    Draft(Vec<FaceId>, DraftOpts),
    Defeature(Vec<FaceId>),
    Replace(FaceId, Surface),
    SplitFaces(Vec<FaceId>),
    /// BREP-04's `split` of the body by the tool (split body).
    SplitBody,
    OffsetFaces(Vec<(FaceId, f64)>),
}

/// One corpus case: an input (and the tool of a split), the op, and the reference volume with
/// its relative bound.
#[derive(Clone, Debug)]
pub struct Case {
    pub name: &'static str,
    pub input: Shape,
    pub tool: Option<Shape>,
    pub op: LocalOp,
    pub analytic: Option<(f64, f64)>,
    /// Solids in the result.
    pub solids: usize,
}

impl Case {
    pub fn run(&self, k: &dyn BrepKernel) -> KResult<OpResult> {
        let ctx = JobCtx::detached();
        match &self.op {
            LocalOp::Draft(faces, opts) => k.draft(&self.input, faces, opts, &ctx),
            LocalOp::Defeature(faces) => k.defeature(&self.input, faces, &ctx),
            LocalOp::Replace(face, with) => k.replace_face(&self.input, *face, with, &ctx),
            LocalOp::SplitFaces(faces) => {
                k.split_faces(&self.input, faces, self.tool.as_ref().unwrap(), &ctx)
            }
            LocalOp::SplitBody => k.split(
                std::slice::from_ref(&self.input),
                std::slice::from_ref(self.tool.as_ref().unwrap()),
                &BoolOpts::default(),
                &ctx,
            ),
            LocalOp::OffsetFaces(faces) => k.offset_faces(&self.input, faces, &ctx),
        }
    }

    /// The history operands: the input, plus the tool for the BREP-04 split.
    pub fn operands(&self) -> Vec<&Shape> {
        match self.op {
            LocalOp::SplitBody => vec![&self.input, self.tool.as_ref().unwrap()],
            _ => vec![&self.input],
        }
    }
}

pub fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

pub fn dir(x: f64, y: f64, z: f64) -> Dir3 {
    Dir3::new(Vec3::new(x, y, z), &Tolerance::DEFAULT).unwrap()
}

/// A right-handed frame at `origin` with the given normal and x axis.
pub fn frame(origin: Point3, normal: Dir3, x: Dir3) -> Plane {
    let y = Dir3::new(normal.get().cross(x.get()), &Tolerance::DEFAULT).unwrap();
    Plane {
        origin,
        x,
        y,
        normal,
    }
}

fn face(index: u32) -> SubShape {
    SubShape {
        kind: SubKind::Face,
        index,
    }
}

/// The faces whose `sub_geom` has class `class` and a centroid within 1e-6 of `at`.
pub fn faces_where(
    k: &dyn BrepKernel,
    s: &Shape,
    class: Option<GeomClass>,
    at: Option<Point3>,
) -> Vec<FaceId> {
    (0..s.count(SubKind::Face))
        .filter(|&i| {
            let g = k.sub_geom(s, face(i)).expect("sub_geom");
            class.is_none_or(|c| g.class == c)
                && at.is_none_or(|at| (g.centroid - at).length() < 1e-6)
        })
        .map(FaceId)
        .collect()
}

/// The one face with its centroid at `at`.
pub fn face_at(k: &dyn BrepKernel, s: &Shape, at: Point3) -> FaceId {
    let hits = faces_where(k, s, None, Some(at));
    assert_eq!(hits.len(), 1, "face at {at:?}: {hits:?}");
    hits[0]
}

pub fn block(k: &dyn BrepKernel) -> Shape {
    k.make_box(&Plane::XY, Vec3::new(A, B, C))
        .expect("box")
        .shape
}

pub fn top(k: &dyn BrepKernel, s: &Shape) -> FaceId {
    face_at(k, s, p(A / 2.0, B / 2.0, C))
}

/// The 4 side faces of the block.
pub fn sides(k: &dyn BrepKernel, s: &Shape) -> Vec<FaceId> {
    vec![
        face_at(k, s, p(0.0, B / 2.0, C / 2.0)),
        face_at(k, s, p(A, B / 2.0, C / 2.0)),
        face_at(k, s, p(A / 2.0, 0.0, C / 2.0)),
        face_at(k, s, p(A / 2.0, B, C / 2.0)),
    ]
}

pub fn cylinder(k: &dyn BrepKernel) -> Shape {
    k.make_cylinder(&Plane::XY, R, H, TAU)
        .expect("cylinder")
        .shape
}

/// The block with its top front edge (y = 0, z = C) filleted by `r`.
pub fn filleted(k: &dyn BrepKernel, r: f64) -> Shape {
    let b = block(k);
    let e = front_top_edge(&b);
    k.fillet(
        &b,
        &[(e, FilletSpec::Constant(r))],
        &Default::default(),
        &JobCtx::detached(),
    )
    .expect("fillet")
    .shape
}

/// The block with its top front edge chamfered by `d`.
pub fn chamfered(k: &dyn BrepKernel, d: f64) -> Shape {
    let b = block(k);
    let e = front_top_edge(&b);
    k.chamfer(&b, &[(e, ChamferSpec::Equal(d))], &JobCtx::detached())
        .expect("chamfer")
        .shape
}

fn front_top_edge(b: &Shape) -> amg_brep::EdgeId {
    b.edges()
        .find(|&e| {
            let [v0, v1] = b.edge_vertices(e);
            let (a, c) = (b.vertex_point(v0), b.vertex_point(v1));
            a.y.abs() < 1e-9 && c.y.abs() < 1e-9 && (a.z - C).abs() < 1e-9 && (c.z - C).abs() < 1e-9
        })
        .expect("front top edge")
}

/// Boss radius and height on the block's top, at the top's centre.
pub const BOSS_R: f64 = 3.0;
pub const BOSS_H: f64 = 5.0;

pub fn bossed(k: &dyn BrepKernel) -> Shape {
    let b = block(k);
    let boss = k
        .make_cylinder(
            &frame(p(A / 2.0, B / 2.0, C), Dir3::Z, Dir3::X),
            BOSS_R,
            BOSS_H,
            TAU,
        )
        .unwrap()
        .shape;
    k.boolean(
        BoolOp::Fuse,
        &[b],
        &[boss],
        &BoolOpts::default(),
        &JobCtx::detached(),
    )
    .expect("fuse")
    .shape
}

/// The boss's wall and top.
pub fn boss_faces(k: &dyn BrepKernel, s: &Shape) -> Vec<FaceId> {
    let mut faces = faces_where(k, s, Some(GeomClass::Cylinder), None);
    faces.push(face_at(k, s, p(A / 2.0, B / 2.0, C + BOSS_H)));
    faces
}

/// The block with a through hole of radius `HOLE_R` along z at the centre, or a blind one
/// `depth` deep from the top.
pub const HOLE_R: f64 = 2.0;

pub fn holed(k: &dyn BrepKernel, depth: Option<f64>) -> Shape {
    let b = block(k);
    let (z0, h) = match depth {
        Some(d) => (C - d, d + 1.0),
        None => (-1.0, C + 2.0),
    };
    let tool = k
        .make_cylinder(
            &frame(p(A / 2.0, B / 2.0, z0), Dir3::Z, Dir3::X),
            HOLE_R,
            h,
            TAU,
        )
        .unwrap()
        .shape;
    k.boolean(
        BoolOp::Cut,
        &[b],
        &[tool],
        &BoolOpts::default(),
        &JobCtx::detached(),
    )
    .expect("cut")
    .shape
}

/// The hole's wall (and its bottom for a blind hole).
pub fn hole_faces(k: &dyn BrepKernel, s: &Shape, depth: Option<f64>) -> Vec<FaceId> {
    let mut faces = faces_where(k, s, Some(GeomClass::Cylinder), None);
    if let Some(d) = depth {
        faces.push(face_at(k, s, p(A / 2.0, B / 2.0, C - d)));
    }
    faces
}

/// A wire through `segs`.
pub fn wire(k: &dyn BrepKernel, segs: &[CurveSeg]) -> Shape {
    k.make_wire(segs, &Tolerance::DEFAULT).unwrap().shape
}

/// A straight edge from `a` to `b`.
pub fn line(k: &dyn BrepKernel, a: Point3, b: Point3) -> Shape {
    let d = b - a;
    wire(
        k,
        &[CurveSeg {
            curve: amg_brep::geom::Curve3::Line {
                origin: a,
                dir: Dir3::new(d, &Tolerance::DEFAULT).unwrap(),
            },
            t0: 0.0,
            t1: d.length(),
            tag: Tag(1),
        }],
    )
}

/// A full circle of radius `r` on `frame`.
pub fn circle(k: &dyn BrepKernel, frame: Plane, r: f64) -> Shape {
    wire(
        k,
        &[CurveSeg {
            curve: amg_brep::geom::Curve3::Circle { frame, radius: r },
            t0: 0.0,
            t1: TAU,
            tag: Tag(2),
        }],
    )
}

/// A planar face on `frame`, `half` wide each way of its origin.
pub fn plane_face(k: &dyn BrepKernel, frame: Plane, half: f64) -> Shape {
    k.make_face(
        &Surface::Plane(frame),
        &FaceBounds::UvRect {
            u0: -half,
            u1: half,
            v0: -half,
            v1: half,
        },
        1e-7,
    )
    .unwrap()
    .face
}

/// Volume of the block with its 4 sides drafted by `angle` about the plane `z = z0`.
pub fn drafted_block_volume(angle: f64, z0: f64) -> f64 {
    let t = angle.tan();
    // Each side moves in by (z − z0) t; integrate (A − 2(z − z0)t)(B − 2(z − z0)t) over [0, C].
    let m1 = (C - z0).powi(2) / 2.0 - z0 * z0 / 2.0;
    let m2 = ((C - z0).powi(3) + z0.powi(3)) / 3.0;
    A * B * C - 2.0 * t * (A + B) * m1 + 4.0 * t * t * m2
}

pub fn corpus(k: &dyn BrepKernel) -> Vec<Case> {
    let mut cases = Vec::new();
    let b = block(k);
    let cyl = cylinder(k);
    let up = |z0: f64| DraftOpts {
        pull: Dir3::Z,
        angle: 3f64.to_radians(),
        neutral: Plane {
            origin: p(0.0, 0.0, z0),
            ..Plane::XY
        },
    };
    let case = |name, input: &Shape, tool: Option<Shape>, op, analytic: Option<f64>| Case {
        name,
        input: input.clone(),
        tool,
        op,
        analytic: analytic.map(|v| (v, EXACT)),
        solids: 1,
    };

    // -- draft --
    cases.push(case(
        "draft_box_sides",
        &b,
        None,
        LocalOp::Draft(sides(k, &b), up(0.0)),
        Some(drafted_block_volume(3f64.to_radians(), 0.0)),
    ));
    cases.push(case(
        "draft_box_sides_negative",
        &b,
        None,
        LocalOp::Draft(
            sides(k, &b),
            DraftOpts {
                angle: -3f64.to_radians(),
                ..up(0.0)
            },
        ),
        Some(drafted_block_volume(-3f64.to_radians(), 0.0)),
    ));
    cases.push(case(
        "draft_box_sides_mid_plane",
        &b,
        None,
        LocalOp::Draft(sides(k, &b), up(C / 2.0)),
        Some(drafted_block_volume(3f64.to_radians(), C / 2.0)),
    ));
    {
        let angle = 5f64.to_radians();
        let side = face_at(k, &b, p(A, B / 2.0, C / 2.0));
        cases.push(case(
            "draft_box_one_side",
            &b,
            None,
            LocalOp::Draft(vec![side], DraftOpts { angle, ..up(0.0) }),
            Some(A * B * C - B * C * C * angle.tan() / 2.0),
        ));
    }
    {
        let angle = 4f64.to_radians();
        let wall = faces_where(k, &cyl, Some(GeomClass::Cylinder), None);
        let r1 = R - H * angle.tan();
        cases.push(case(
            "draft_cylinder_wall",
            &cyl,
            None,
            LocalOp::Draft(wall, DraftOpts { angle, ..up(0.0) }),
            Some(PI * H / 3.0 * (R * R + R * r1 + r1 * r1)),
        ));
    }

    // -- defeature --
    let fil = filleted(k, 3.0);
    let fillet_face = faces_where(k, &fil, Some(GeomClass::Cylinder), None);
    cases.push(case(
        "defeature_fillet",
        &fil,
        None,
        LocalOp::Defeature(fillet_face),
        Some(A * B * C),
    ));
    let cham = chamfered(k, 2.0);
    let chamfer_face = faces_where(
        k,
        &cham,
        Some(GeomClass::Plane),
        Some(p(A / 2.0, 1.0, C - 1.0)),
    );
    cases.push(case(
        "defeature_chamfer",
        &cham,
        None,
        LocalOp::Defeature(chamfer_face),
        Some(A * B * C),
    ));
    let boss = bossed(k);
    cases.push(case(
        "defeature_boss",
        &boss,
        None,
        LocalOp::Defeature(boss_faces(k, &boss)),
        Some(A * B * C),
    ));
    let through = holed(k, None);
    cases.push(case(
        "defeature_through_hole",
        &through,
        None,
        LocalOp::Defeature(hole_faces(k, &through, None)),
        Some(A * B * C),
    ));
    let blind = holed(k, Some(4.0));
    cases.push(case(
        "defeature_blind_hole",
        &blind,
        None,
        LocalOp::Defeature(hole_faces(k, &blind, Some(4.0))),
        Some(A * B * C),
    ));

    // -- replace face --
    let t = top(k, &b);
    let at_z = |z: f64| {
        Surface::Plane(Plane {
            origin: p(0.0, 0.0, z),
            ..Plane::XY
        })
    };
    cases.push(case(
        "replace_top_up",
        &b,
        None,
        LocalOp::Replace(t, at_z(C + 3.0)),
        Some(A * B * (C + 3.0)),
    ));
    cases.push(case(
        "replace_top_down",
        &b,
        None,
        LocalOp::Replace(t, at_z(C - 4.0)),
        Some(A * B * (C - 4.0)),
    ));
    {
        // Tilted about the top's centre line: half extends, half trims; the volume stays.
        let a = 10f64.to_radians();
        let tilted = frame(
            p(A / 2.0, B / 2.0, C),
            dir(-a.sin(), 0.0, a.cos()),
            dir(a.cos(), 0.0, a.sin()),
        );
        cases.push(case(
            "replace_top_tilted",
            &b,
            None,
            LocalOp::Replace(t, Surface::Plane(tilted)),
            Some(A * B * C),
        ));
    }
    {
        let side = face_at(k, &b, p(A, B / 2.0, C / 2.0));
        let at = frame(p(A + 5.0, 0.0, 0.0), Dir3::X, Dir3::Y);
        cases.push(case(
            "replace_side_out",
            &b,
            None,
            LocalOp::Replace(side, Surface::Plane(at)),
            Some((A + 5.0) * B * C),
        ));
    }
    {
        // A cylindrical crown: axis along y, its top 2 mm above the old top at x = A/2.
        let rc = 40.0;
        let zc = C + 2.0 - rc;
        let axis = frame(p(A / 2.0, 0.0, zc), Dir3::Y, Dir3::Z);
        let half = A / 2.0;
        let segment = half * (rc * rc - half * half).sqrt() + rc * rc * (half / rc).asin();
        cases.push(case(
            "replace_top_cylinder",
            &b,
            None,
            LocalOp::Replace(
                t,
                Surface::Cylinder {
                    frame: axis,
                    radius: rc,
                },
            ),
            Some(B * (zc * A + segment)),
        ));
    }
    {
        let ct = face_at(k, &cyl, p(0.0, 0.0, H));
        cases.push(case(
            "replace_cylinder_top_up",
            &cyl,
            None,
            LocalOp::Replace(ct, at_z(H + 2.0)),
            Some(PI * R * R * (H + 2.0)),
        ));
    }

    // -- split faces --
    cases.push(case(
        "split_faces_top_line",
        &b,
        Some(line(k, p(5.0, -2.0, C), p(5.0, B + 2.0, C))),
        LocalOp::SplitFaces(vec![t]),
        Some(A * B * C),
    ));
    cases.push(case(
        "split_faces_top_circle",
        &b,
        Some(circle(
            k,
            frame(p(A / 2.0, B / 2.0, C), Dir3::Z, Dir3::X),
            3.0,
        )),
        LocalOp::SplitFaces(vec![t]),
        Some(A * B * C),
    ));
    cases.push(case(
        "split_faces_top_by_plane",
        &b,
        Some(plane_face(
            k,
            frame(p(12.0, B / 2.0, C / 2.0), Dir3::X, Dir3::Y),
            50.0,
        )),
        LocalOp::SplitFaces(vec![t]),
        Some(A * B * C),
    ));
    {
        let bottom = face_at(k, &b, p(A / 2.0, B / 2.0, 0.0));
        cases.push(case(
            "split_faces_top_bottom_by_plane",
            &b,
            Some(plane_face(
                k,
                frame(p(12.0, B / 2.0, C / 2.0), Dir3::X, Dir3::Y),
                50.0,
            )),
            LocalOp::SplitFaces(vec![t, bottom]),
            Some(A * B * C),
        ));
    }
    cases.push(case(
        "split_faces_cylinder_wall",
        &cyl,
        Some(plane_face(
            k,
            frame(p(0.0, 0.0, 4.0), Dir3::Z, Dir3::X),
            50.0,
        )),
        LocalOp::SplitFaces(faces_where(k, &cyl, Some(GeomClass::Cylinder), None)),
        Some(PI * R * R * H),
    ));

    // -- split body (BREP-04 split) --
    cases.push(Case {
        solids: 2,
        ..case(
            "split_body_plane",
            &b,
            Some(plane_face(
                k,
                frame(p(A / 2.0, B / 2.0, 5.0), Dir3::Z, Dir3::X),
                50.0,
            )),
            LocalOp::SplitBody,
            Some(A * B * C),
        )
    });
    {
        let two = k
            .make_compound(&[
                plane_face(k, frame(p(5.0, B / 2.0, C / 2.0), Dir3::X, Dir3::Y), 50.0),
                plane_face(k, frame(p(12.0, B / 2.0, C / 2.0), Dir3::X, Dir3::Y), 50.0),
            ])
            .unwrap();
        cases.push(Case {
            solids: 3,
            ..case(
                "split_body_face_set",
                &b,
                Some(two),
                LocalOp::SplitBody,
                Some(A * B * C),
            )
        });
    }

    // -- offset faces (R-S04-5) --
    cases.push(case(
        "offset_faces_top_up",
        &b,
        None,
        LocalOp::OffsetFaces(vec![(t, 3.0)]),
        Some(A * B * (C + 3.0)),
    ));
    cases.push(case(
        "offset_faces_top_down",
        &b,
        None,
        LocalOp::OffsetFaces(vec![(t, -4.0)]),
        Some(A * B * (C - 4.0)),
    ));
    {
        let side = face_at(k, &b, p(A, B / 2.0, C / 2.0));
        cases.push(case(
            "offset_faces_top_and_side",
            &b,
            None,
            LocalOp::OffsetFaces(vec![(t, 2.0), (side, 1.0)]),
            Some((A + 1.0) * B * (C + 2.0)),
        ));
    }
    {
        let ct = face_at(k, &cyl, p(0.0, 0.0, H));
        cases.push(case(
            "offset_faces_cylinder_top",
            &cyl,
            None,
            LocalOp::OffsetFaces(vec![(ct, 2.0)]),
            Some(PI * R * R * (H + 2.0)),
        ));
    }
    cases
}
