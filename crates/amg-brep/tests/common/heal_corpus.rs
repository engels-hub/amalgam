//! The BREP-10 corpus: the defect fixtures (`testdata/brep/defects/`), a cube of gapped polygon
//! faces, 20 parts exploded into their faces, and two coplanar boxes fused. Used by
//! `tests/heal.rs`, `tests/history.rs` (completeness) and the bench.
#![allow(dead_code, clippy::unwrap_used, reason = "shared test helper")]

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use amg_brep::geom::{Curve2d, Loop2, PlanarProfile, ProfileEdge2, Tag};
use amg_brep::{
    BoolOp, BoolOpts, BrepKernel, Dir3, EdgeId, FilletOpts, FilletSpec, HealOpts, JobCtx, KResult,
    Line3, OcctKernel, OpResult, Plane, Point2, Point3, PrismExtent, PrismOpts, SewOpts, Shape,
    SubKind, SubShape, Tolerance, Transform, UnifyOpts, Vec3,
};
use amg_core::Mat3;

#[path = "defects.rs"]
mod defects;
pub use defects::{DEFECTS, heal_opts};

/// The defects `heal` turns from BRepCheck-invalid into valid (Done-when 3).
pub const HEALED: [&str; 10] = [
    "vertex_off_edges",
    "edge_off_faces",
    "missing_pcurve",
    "pcurve_off_edge",
    "reversed_lateral",
    "reversed_face",
    "same_parameter_off",
    "same_range_off",
    "wire_gaps",
    "reversed_wire",
];

/// `testdata/brep/defects/<name>.brep`; `None` (with a SKIP notice) when it is an un-fetched
/// LFS pointer.
pub fn defect(name: &str) -> Option<Shape> {
    let path = amg_testkit::testdata(&format!("brep/defects/{name}.brep"))?;
    let file = std::fs::File::open(path).unwrap();
    Some(Shape::read_text(std::io::BufReader::new(file)).unwrap())
}

/// `testdata/brep/<name>.brep`.
pub fn fixture(name: &str) -> Option<Shape> {
    let path = amg_testkit::testdata(&format!("brep/{name}.brep"))?;
    let file = std::fs::File::open(path).unwrap();
    Some(Shape::read_text(std::io::BufReader::new(file)).unwrap())
}

fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

/// The six square faces of the cube `[0, side]³` as separate planar polygons, each moved
/// `gap / √2` along its own axis (the ±x faces along +x, and so on), so that the edges of
/// neighbouring faces are `gap` apart and no two faces share a vertex. Each face keeps its full
/// square, and each pair of opposite faces moves together, so the enclosed volume stays `side³`.
pub fn gapped_cube(k: &dyn BrepKernel, side: f64, gap: f64) -> Vec<Shape> {
    let s = gap / std::f64::consts::SQRT_2;
    // (fixed axis, value, the two in-plane axes in an order giving the outward normal).
    let faces: [(usize, f64, usize, usize); 6] = [
        (2, 0.0, 1, 0),
        (2, side, 0, 1),
        (1, 0.0, 0, 2),
        (1, side, 2, 0),
        (0, 0.0, 2, 1),
        (0, side, 1, 2),
    ];
    faces
        .iter()
        .map(|&(axis, value, u, v)| {
            let corner = |a: f64, b: f64| {
                let mut c = [0.0; 3];
                c[axis] = value + s;
                c[u] = a;
                c[v] = b;
                p(c[0], c[1], c[2])
            };
            let outer = [
                corner(0.0, 0.0),
                corner(side, 0.0),
                corner(side, side),
                corner(0.0, side),
            ];
            k.make_planar_polygon(&outer, &[], &Tolerance::DEFAULT)
                .unwrap()
        })
        .collect()
}

/// The faces of `s` as `sub_shape`s: they still share their edges and vertices.
pub fn faces_of(s: &Shape) -> Vec<Shape> {
    s.faces()
        .map(|f| {
            s.sub_shape(SubShape {
                kind: SubKind::Face,
                index: f.0,
            })
            .unwrap()
        })
        .collect()
}

/// The faces of `s`, each copied into new geometry on its own: no two share anything.
pub fn faces_apart(s: &Shape) -> Vec<Shape> {
    let identity = Transform::from_mat3_translation(Mat3::IDENTITY, Vec3::ZERO);
    faces_of(s)
        .iter()
        .map(|f| f.gtransformed(&identity).unwrap().shape)
        .collect()
}

fn line(a: (f64, f64), b: (f64, f64)) -> Curve2d {
    Curve2d::Line {
        a: Point2::new(a.0, a.1),
        b: Point2::new(b.0, b.1),
    }
}

fn loop_of(curves: Vec<Curve2d>) -> Loop2 {
    Loop2 {
        edges: curves
            .into_iter()
            .enumerate()
            .map(|(i, curve)| ProfileEdge2 {
                curve,
                tag: Tag(i as u64),
            })
            .collect(),
    }
}

fn polygon(points: &[(f64, f64)]) -> Vec<Curve2d> {
    (0..points.len())
        .map(|i| line(points[i], points[(i + 1) % points.len()]))
        .collect()
}

fn face(k: &dyn BrepKernel, plane: Plane, outer: Vec<Curve2d>, holes: Vec<Vec<Curve2d>>) -> Shape {
    let profile = PlanarProfile {
        plane,
        outer: loop_of(outer),
        holes: holes.into_iter().map(loop_of).collect(),
    };
    k.make_planar_faces(&[profile], &Tolerance::DEFAULT)
        .unwrap()
        .remove(0)
        .face
}

fn tilted(origin: Point3, angle: f64) -> Plane {
    let (s, c) = angle.sin_cos();
    Plane {
        origin,
        normal: Dir3::new_unchecked(Vec3::new(0.0, -s, c)),
        x: Dir3::new_unchecked(Vec3::X),
        y: Dir3::new_unchecked(Vec3::new(0.0, c, s)),
    }
}

fn extrude(k: &dyn BrepKernel, s: &Shape, v: Vec3) -> Shape {
    let opts = PrismOpts {
        taper: 0.0,
        merge_coplanar: false,
    };
    k.prism(s, &PrismExtent::Vector(v), &opts, &JobCtx::detached())
        .unwrap()
        .shape
}

/// 20 closed parts for Done-when 2: the closed fixtures of `testdata/brep` (when present) and
/// parts built here (primitives on tilted frames, partial sweeps, prisms of an L and a holed
/// square, revols, boolean results, a filleted box).
pub fn parts(k: &OcctKernel) -> Vec<(String, Shape)> {
    let mut out: Vec<(String, Shape)> = Vec::new();
    for name in [
        "box",
        "cylinder",
        "cone",
        "sphere",
        "torus",
        "filleted_box",
        "located_compound",
        "cylinder_mirrored",
    ] {
        if let Some(s) = fixture(name) {
            out.push((name.to_owned(), s));
        }
    }
    let ctx = JobCtx::detached();
    let t = tilted(p(1.0, 2.0, 3.0), 0.4);
    out.push((
        "tilted_box".into(),
        k.make_box(&t, Vec3::new(7.0, 5.0, 3.0)).unwrap().shape,
    ));
    out.push((
        "half_cylinder".into(),
        k.make_cylinder(&t, 4.0, 6.0, PI).unwrap().shape,
    ));
    out.push((
        "full_cone".into(),
        k.make_cone(&Plane::XY, 5.0, 0.0, 8.0, TAU).unwrap().shape,
    ));
    out.push((
        "quarter_torus".into(),
        k.make_torus(&t, 10.0, 2.0, FRAC_PI_2).unwrap().shape,
    ));
    out.push(("small_sphere".into(), k.make_sphere(&t, 1.5).unwrap().shape));
    let l_shape = face(
        k,
        Plane::XY,
        polygon(&[
            (0.0, 0.0),
            (8.0, 0.0),
            (8.0, 3.0),
            (3.0, 3.0),
            (3.0, 8.0),
            (0.0, 8.0),
        ]),
        vec![],
    );
    out.push((
        "l_prism".into(),
        extrude(k, &l_shape, Vec3::new(0.0, 0.0, 4.0)),
    ));
    let holed = face(
        k,
        t,
        polygon(&[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]),
        vec![vec![Curve2d::Circle {
            center: Point2::new(5.0, 5.0),
            radius: 2.0,
        }]],
    );
    out.push((
        "holed_prism".into(),
        extrude(k, &holed, 6.0 * t.normal.get()),
    ));
    let xz = Plane {
        origin: p(4.0, 0.0, 0.0),
        normal: Dir3::new_unchecked(-Vec3::Y),
        x: Dir3::new_unchecked(Vec3::X),
        y: Dir3::new_unchecked(Vec3::Z),
    };
    let rect = face(
        k,
        xz,
        polygon(&[(0.0, 0.0), (3.0, 0.0), (3.0, 5.0), (0.0, 5.0)]),
        vec![],
    );
    let z_axis = Line3 {
        point: Point3::ZERO,
        dir: Dir3::new_unchecked(Vec3::Z),
    };
    out.push((
        "revol_quarter".into(),
        k.revol(&rect, &z_axis, FRAC_PI_2, &ctx).unwrap().shape,
    ));
    out.push((
        "revol_ring".into(),
        k.revol(&rect, &z_axis, TAU, &ctx).unwrap().shape,
    ));
    let block = k
        .make_box(&Plane::XY, Vec3::new(10.0, 10.0, 10.0))
        .unwrap()
        .shape;
    let bar = k
        .make_cylinder(
            &Plane {
                origin: p(-1.0, 5.0, 10.0),
                normal: Dir3::new_unchecked(Vec3::X),
                x: Dir3::new_unchecked(Vec3::Y),
                y: Dir3::new_unchecked(Vec3::Z),
            },
            2.0,
            12.0,
            TAU,
        )
        .unwrap()
        .shape;
    let bool_opts = BoolOpts::default();
    out.push((
        "grooved_cube".into(),
        k.boolean(
            BoolOp::Cut,
            std::slice::from_ref(&block),
            std::slice::from_ref(&bar),
            &bool_opts,
            &ctx,
        )
        .unwrap()
        .shape,
    ));
    out.push((
        "cube_with_bar".into(),
        k.boolean(
            BoolOp::Fuse,
            std::slice::from_ref(&block),
            std::slice::from_ref(&bar),
            &bool_opts,
            &ctx,
        )
        .unwrap()
        .shape,
    ));
    let rounded = k
        .fillet(
            &block,
            &[(EdgeId(0), FilletSpec::Constant(2.0))],
            &FilletOpts::default(),
            &ctx,
        )
        .unwrap()
        .shape;
    out.push(("rounded_block".into(), rounded));
    out
}

/// Two 10 mm cubes side by side along x (`[0, 10]` and `[10, 20]`), and their fuse: the 4 side
/// faces of each are coplanar with the other's, so the fuse has 10 faces.
pub fn coplanar_fuse(k: &OcctKernel) -> (Shape, Shape, OpResult) {
    let a = k
        .make_box(&Plane::XY, Vec3::new(10.0, 10.0, 10.0))
        .unwrap()
        .shape;
    let b = k
        .make_box(
            &Plane {
                origin: p(10.0, 0.0, 0.0),
                ..Plane::XY
            },
            Vec3::new(10.0, 10.0, 10.0),
        )
        .unwrap()
        .shape;
    let fused = k
        .boolean(
            BoolOp::Fuse,
            &[a.clone(), b.clone()],
            &[],
            &BoolOpts::default(),
            &JobCtx::detached(),
        )
        .unwrap();
    (a, b, fused)
}

/// One BREP-10 op on its inputs.
#[derive(Clone, Debug)]
pub enum Op {
    Heal(HealOpts),
    Unify(UnifyOpts),
    Sew(SewOpts),
    MakeSolid,
}

/// One corpus case.
#[derive(Clone, Debug)]
pub struct Case {
    pub name: String,
    pub inputs: Vec<Shape>,
    pub op: Op,
}

impl Case {
    pub fn run(&self, k: &dyn BrepKernel) -> KResult<OpResult> {
        let ctx = JobCtx::detached();
        match &self.op {
            Op::Heal(o) => k.heal(&self.inputs[0], o, &ctx),
            Op::Unify(o) => k.unify_same_domain(&self.inputs[0], o),
            Op::Sew(o) => k.sew(&self.inputs, o, &ctx),
            Op::MakeSolid => k.make_solid(&self.inputs[0]),
        }
    }
}

/// Sewing at the default tolerance, into solids.
pub fn sew_solid() -> SewOpts {
    SewOpts::default()
}

/// Every BREP-10 op of the corpus that succeeds: heals of the defects, the unify of the
/// coplanar fuse, sews of the gapped cube and of every part (faces shared and apart), and
/// `make_solid` of the sewn shells.
pub fn corpus(k: &OcctKernel) -> Vec<Case> {
    let mut out = Vec::new();
    for name in DEFECTS {
        if let Some(s) = defect(name) {
            out.push(Case {
                name: format!("heal_{name}"),
                inputs: vec![s],
                op: Op::Heal(heal_opts()),
            });
        }
    }
    let (_, _, fused) = coplanar_fuse(k);
    out.push(Case {
        name: "unify_coplanar_fuse".into(),
        inputs: vec![fused.shape],
        op: Op::Unify(UnifyOpts::default()),
    });
    out.push(Case {
        name: "sew_gapped_cube".into(),
        inputs: gapped_cube(k, 10.0, 1e-4),
        op: Op::Sew(SewOpts {
            tol: 1e-3,
            ..sew_solid()
        }),
    });
    for (name, part) in parts(k) {
        out.push(Case {
            name: format!("sew_{name}"),
            inputs: faces_of(&part),
            op: Op::Sew(sew_solid()),
        });
        out.push(Case {
            name: format!("sew_apart_{name}"),
            inputs: faces_apart(&part),
            op: Op::Sew(sew_solid()),
        });
        let shell = k
            .sew(
                &faces_apart(&part),
                &SewOpts {
                    make_solid: false,
                    ..sew_solid()
                },
                &JobCtx::detached(),
            )
            .unwrap()
            .shape;
        out.push(Case {
            name: format!("make_solid_{name}"),
            inputs: vec![shell],
            op: Op::MakeSolid,
        });
    }
    out
}
