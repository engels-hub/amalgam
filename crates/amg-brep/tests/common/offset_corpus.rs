//! The BREP-07 shell / offset / thicken corpus: boxes, a cylinder, a filleted box, a sphere and
//! open faces (plane, cylinder patch, B-spline), each with its analytic volume where one exists.
//! Used by `tests/offset.rs` (goldens, roles, determinism), `tests/history.rs` (completeness)
//! and the bench.
#![allow(dead_code, reason = "shared test helper")]

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use amg_brep::geom::{BSplineSurfaceData, FaceBounds, Surface};
use amg_brep::{
    BrepKernel, FaceId, FilletOpts, FilletSpec, JobCtx, KResult, OffsetJoin, OpResult, Plane,
    Point2, Point3, Shape, ShellOpts, SubKind, SubShape, Vec3,
};

/// The box of most cases.
pub const A: f64 = 20.0;
pub const B: f64 = 16.0;
pub const C: f64 = 12.0;

/// Bound for exact (planar, analytic-surface) cases.
pub const EXACT: f64 = 1e-9;

/// What one case runs.
#[derive(Clone, Debug)]
pub enum OffsetOp {
    Shell(Vec<FaceId>, ShellOpts),
    Offset(f64, OffsetJoin),
    Thicken(f64, OffsetJoin),
}

/// One corpus case: an input, the op on it, and the reference volume with its relative bound.
#[derive(Clone, Debug)]
pub struct Case {
    pub name: &'static str,
    pub input: Shape,
    pub op: OffsetOp,
    pub analytic: Option<(f64, f64)>,
}

impl Case {
    pub fn run(&self, k: &dyn BrepKernel) -> KResult<OpResult> {
        let ctx = JobCtx::detached();
        match &self.op {
            OffsetOp::Shell(remove, opts) => k.shell(&self.input, remove, opts, &ctx),
            OffsetOp::Offset(d, join) => k.offset(&self.input, *d, *join, 0.0, &ctx),
            OffsetOp::Thicken(t, join) => k.thicken(&self.input, *t, *join, &ctx),
        }
    }

    pub fn is_shell(&self) -> bool {
        matches!(self.op, OffsetOp::Shell(..))
    }

    pub fn is_offset(&self) -> bool {
        matches!(self.op, OffsetOp::Offset(..))
    }

    /// The removed faces of a shell.
    pub fn removed(&self) -> Vec<FaceId> {
        match &self.op {
            OffsetOp::Shell(remove, _) => remove.clone(),
            _ => Vec::new(),
        }
    }
}

fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

/// The one face whose `sub_geom` centroid is within 1e-7 of `at`.
pub fn face_at(k: &dyn BrepKernel, s: &Shape, at: Point3) -> FaceId {
    let hits: Vec<u32> = (0..s.count(SubKind::Face))
        .filter(|&i| {
            let g = k
                .sub_geom(
                    s,
                    SubShape {
                        kind: SubKind::Face,
                        index: i,
                    },
                )
                .expect("sub_geom");
            (g.centroid - at).length() < 1e-7
        })
        .collect();
    assert_eq!(hits.len(), 1, "face at {at:?}: {hits:?}");
    FaceId(hits[0])
}

pub fn shell_opts(thickness: f64, join: OffsetJoin) -> ShellOpts {
    ShellOpts {
        thickness,
        join,
        per_face: Vec::new(),
        tol: 0.0,
    }
}

/// The box `[0, A] × [0, B] × [0, C]`.
pub fn block(k: &dyn BrepKernel) -> Shape {
    k.make_box(&Plane::XY, Vec3::new(A, B, C))
        .expect("box")
        .shape
}

pub fn top(k: &dyn BrepKernel, s: &Shape) -> FaceId {
    face_at(k, s, p(A / 2.0, B / 2.0, C))
}

pub fn bottom(k: &dyn BrepKernel, s: &Shape) -> FaceId {
    face_at(k, s, p(A / 2.0, B / 2.0, 0.0))
}

/// Radius and height of the cylinder cases.
pub const R: f64 = 6.0;
pub const H: f64 = 10.0;

pub fn cylinder(k: &dyn BrepKernel) -> Shape {
    k.make_cylinder(&Plane::XY, R, H, TAU)
        .expect("cylinder")
        .shape
}

/// The fillet radius of the filleted box.
pub const FILLET: f64 = 3.0;

/// The box with its 4 vertical edges rounded by `FILLET`.
pub fn filleted_block(k: &dyn BrepKernel) -> Shape {
    let b = block(k);
    let edges: Vec<_> = b
        .edges()
        .filter(|&e| {
            let [v0, v1] = b.edge_vertices(e);
            let (a, c) = (b.vertex_point(v0), b.vertex_point(v1));
            (a.x - c.x).abs() < 1e-9 && (a.y - c.y).abs() < 1e-9
        })
        .map(|e| (e, FilletSpec::Constant(FILLET)))
        .collect();
    assert_eq!(edges.len(), 4);
    k.fillet(&b, &edges, &FilletOpts::default(), &JobCtx::detached())
        .expect("fillet")
        .shape
}

/// The area of an `a × b` rectangle with corners rounded by `r`.
pub fn rounded_rect(a: f64, b: f64, r: f64) -> f64 {
    a * b - (4.0 - PI) * r * r
}

/// Sphere radius.
pub const SPHERE: f64 = 5.0;

pub fn sphere(k: &dyn BrepKernel) -> Shape {
    k.make_sphere(&Plane::XY, SPHERE).expect("sphere").shape
}

/// The volume of a box grown by `t` with round edges and corners (Steiner's formula).
pub fn steiner_box(a: f64, b: f64, c: f64, t: f64) -> f64 {
    a * b * c
        + 2.0 * t * (a * b + b * c + c * a)
        + PI * t * t * (a + b + c)
        + 4.0 / 3.0 * PI * t.powi(3)
}

/// The same for a cylinder of radius `r` and height `h`: the lateral face has mean curvature
/// 1/(2r), the rims are right-angled edges of length 2πr.
pub fn steiner_cylinder(r: f64, h: f64, t: f64) -> f64 {
    PI * r * r * h
        + (TAU * r * r + TAU * r * h) * t
        + (PI * h + PI * PI * r) * t * t
        + 4.0 / 3.0 * PI * t.powi(3)
}

/// A 20 × 10 planar face in z = 0.
pub fn plane_face(k: &dyn BrepKernel) -> Shape {
    k.make_face(
        &Surface::Plane(Plane::XY),
        &FaceBounds::UvRect {
            u0: 0.0,
            u1: 20.0,
            v0: 0.0,
            v1: 10.0,
        },
        1e-6,
    )
    .expect("plane face")
    .face
}

/// Radius and height of the cylinder patch.
pub const PATCH_R: f64 = 10.0;
pub const PATCH_H: f64 = 10.0;

/// A quarter of a cylinder of radius `PATCH_R`, `PATCH_H` high (normal outward).
pub fn cylinder_patch(k: &dyn BrepKernel) -> Shape {
    k.make_face(
        &Surface::Cylinder {
            frame: Plane::XY,
            radius: PATCH_R,
        },
        &FaceBounds::UvRect {
            u0: 0.0,
            u1: FRAC_PI_2,
            v0: 0.0,
            v1: PATCH_H,
        },
        1e-6,
    )
    .expect("cylinder patch")
    .face
}

/// A bicubic Bézier patch over 20 × 20 mm with its 4 middle poles raised by 3 mm.
pub fn bspline_face(k: &dyn BrepKernel) -> Shape {
    let mut poles = Vec::new();
    for i in 0..4 {
        for j in 0..4 {
            let z = if (1..3).contains(&i) && (1..3).contains(&j) {
                3.0
            } else {
                0.0
            };
            poles.push(p(20.0 * f64::from(i) / 3.0, 20.0 * f64::from(j) / 3.0, z));
        }
    }
    let data = BSplineSurfaceData {
        u_degree: 3,
        v_degree: 3,
        u_knots: vec![0.0, 1.0],
        u_mults: vec![4, 4],
        v_knots: vec![0.0, 1.0],
        v_mults: vec![4, 4],
        poles,
        nu: 4,
        nv: 4,
        weights: None,
        u_periodic: false,
        v_periodic: false,
    };
    k.make_face(&Surface::BSpline(data), &FaceBounds::Natural, 1e-6)
        .expect("bspline face")
        .face
}

/// Gauss–Legendre nodes and weights on [0, 1] (Newton on the Legendre polynomial).
pub fn gauss(n: usize) -> Vec<(f64, f64)> {
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let mut x = (PI * (i as f64 + 0.75) / (n as f64 + 0.5)).cos();
        let mut dp = 0.0;
        for _ in 0..100 {
            let (mut p0, mut p1) = (1.0, x);
            for k in 2..=n {
                let k = k as f64;
                let p2 = ((2.0 * k - 1.0) * x * p1 - (k - 1.0) * p0) / k;
                p0 = p1;
                p1 = p2;
            }
            dp = n as f64 * (x * p1 - p0) / (x * x - 1.0);
            let dx = p1 / dp;
            x -= dx;
            if dx.abs() < 1e-15 {
                break;
            }
        }
        let w = 2.0 / ((1.0 - x * x) * dp * dp);
        out.push((0.5 * (1.0 + x), 0.5 * w));
    }
    out
}

/// The volume swept by face 0 of `face` moving `t` along its (outward) normal field, by
/// quadrature of `det(S_u + s N_u, S_v + s N_v, N)` over the face's parameter rectangle and
/// `s ∈ [0, t]` (the normal derivatives by central differences). Exact up to quadrature for a
/// face without trimming whose offset does not fold.
pub fn swept_volume(k: &dyn BrepKernel, face: &Shape, t: f64) -> f64 {
    let f = FaceId(0);
    let [u0, u1, v0, v1] = face.face_uv_bounds(f);
    let (du, dv) = (u1 - u0, v1 - v0);
    let h = 1e-5;
    let eval = |u: f64, v: f64| k.eval_surface(face, f, Point2::new(u, v)).expect("eval");
    let normal = |u: f64, v: f64| eval(u, v).normal.get();
    let nodes = gauss(24);
    let mut total = 0.0;
    for &(su, wu) in &nodes {
        for &(sv, wv) in &nodes {
            let (u, v) = (u0 + su * du, v0 + sv * dv);
            let sp = eval(u, v);
            let n = sp.normal.get();
            let nu = (normal(u + h * du, v) - normal(u - h * du, v)) / (2.0 * h * du);
            let nv = (normal(u, v + h * dv) - normal(u, v - h * dv)) / (2.0 * h * dv);
            let (su_, sv_) = (sp.d1u, sp.d1v);
            let a0 = su_.cross(sv_).dot(n);
            let a1 = (su_.cross(nv) + nu.cross(sv_)).dot(n);
            let a2 = nu.cross(nv).dot(n);
            let integral = a0 * t + a1 * t * t / 2.0 + a2 * t.powi(3) / 3.0;
            total += wu * wv * integral * du * dv;
        }
    }
    total.abs()
}

/// Every case.
pub fn corpus(k: &dyn BrepKernel) -> Vec<Case> {
    let (a, b, c) = (A, B, C);
    let mut out = Vec::new();
    let blk = block(k);
    let (top_f, bottom_f) = (top(k, &blk), bottom(k, &blk));
    let left = face_at(k, &blk, p(0.0, B / 2.0, C / 2.0));
    let shell = |name, remove: Vec<FaceId>, opts: ShellOpts, v: Option<f64>| Case {
        name,
        input: blk.clone(),
        op: OffsetOp::Shell(remove, opts),
        analytic: v.map(|v| (v, EXACT)),
    };
    out.push(shell(
        "shell_box_top_inward",
        vec![top_f],
        shell_opts(-1.0, OffsetJoin::Arc),
        Some(a * b * c - (a - 2.0) * (b - 2.0) * (c - 1.0)),
    ));
    out.push(shell(
        "shell_box_top_inward_intersection",
        vec![top_f],
        shell_opts(-1.0, OffsetJoin::Intersection),
        Some(a * b * c - (a - 2.0) * (b - 2.0) * (c - 1.0)),
    ));
    out.push(shell(
        "shell_box_hollow_inward",
        Vec::new(),
        shell_opts(-1.0, OffsetJoin::Arc),
        Some(a * b * c - (a - 2.0) * (b - 2.0) * (c - 2.0)),
    ));
    out.push(shell(
        "shell_box_hollow_outward_arc",
        Vec::new(),
        shell_opts(1.0, OffsetJoin::Arc),
        Some(steiner_box(a, b, c, 1.0) - a * b * c),
    ));
    out.push(shell(
        "shell_box_hollow_outward_intersection",
        Vec::new(),
        shell_opts(1.0, OffsetJoin::Intersection),
        Some((a + 2.0) * (b + 2.0) * (c + 2.0) - a * b * c),
    ));
    out.push(shell(
        "shell_box_tube",
        vec![top_f, bottom_f],
        shell_opts(-1.0, OffsetJoin::Arc),
        Some(a * b * c - (a - 2.0) * (b - 2.0) * c),
    ));
    out.push(shell(
        "shell_box_per_face",
        vec![top_f],
        ShellOpts {
            per_face: vec![(left, -2.0), (bottom_f, -3.0)],
            ..shell_opts(-1.0, OffsetJoin::Arc)
        },
        Some(a * b * c - (a - 3.0) * (b - 2.0) * (c - 3.0)),
    ));
    out.push(shell(
        "shell_box_top_outward_arc",
        vec![top_f],
        shell_opts(1.0, OffsetJoin::Arc),
        None,
    ));

    let cyl = cylinder(k);
    let cyl_top = face_at(k, &cyl, p(0.0, 0.0, H));
    out.push(Case {
        name: "shell_cylinder_top_inward",
        input: cyl.clone(),
        op: OffsetOp::Shell(vec![cyl_top], shell_opts(-1.0, OffsetJoin::Arc)),
        analytic: Some((PI * R * R * H - PI * (R - 1.0).powi(2) * (H - 1.0), EXACT)),
    });

    let fb = filleted_block(k);
    let fb_top = top(k, &fb);
    out.push(Case {
        name: "shell_filleted_box_top_inward",
        input: fb.clone(),
        op: OffsetOp::Shell(vec![fb_top], shell_opts(-1.0, OffsetJoin::Arc)),
        analytic: Some((
            rounded_rect(a, b, FILLET) * c
                - rounded_rect(a - 2.0, b - 2.0, FILLET - 1.0) * (c - 1.0),
            EXACT,
        )),
    });

    let sph = sphere(k);
    for (name, d) in [("offset_sphere_out", 2.0), ("offset_sphere_in", -2.0)] {
        out.push(Case {
            name,
            input: sph.clone(),
            op: OffsetOp::Offset(d, OffsetJoin::Arc),
            analytic: Some((4.0 / 3.0 * PI * (SPHERE + d).powi(3), EXACT)),
        });
    }
    out.push(Case {
        name: "offset_box_arc",
        input: blk.clone(),
        op: OffsetOp::Offset(1.0, OffsetJoin::Arc),
        analytic: Some((steiner_box(a, b, c, 1.0), EXACT)),
    });
    out.push(Case {
        name: "offset_box_intersection",
        input: blk.clone(),
        op: OffsetOp::Offset(1.0, OffsetJoin::Intersection),
        analytic: Some(((a + 2.0) * (b + 2.0) * (c + 2.0), EXACT)),
    });
    out.push(Case {
        name: "offset_box_inward",
        input: blk.clone(),
        op: OffsetOp::Offset(-1.0, OffsetJoin::Arc),
        analytic: Some(((a - 2.0) * (b - 2.0) * (c - 2.0), EXACT)),
    });
    out.push(Case {
        name: "offset_cylinder_arc",
        input: cyl,
        op: OffsetOp::Offset(1.0, OffsetJoin::Arc),
        analytic: Some((steiner_cylinder(R, H, 1.0), EXACT)),
    });

    let pf = plane_face(k);
    for (name, t) in [("thicken_plane_up", 2.0), ("thicken_plane_down", -2.0)] {
        out.push(Case {
            name,
            input: pf.clone(),
            op: OffsetOp::Thicken(t, OffsetJoin::Arc),
            analytic: Some((400.0, EXACT)),
        });
    }
    out.push(Case {
        name: "thicken_cylinder_patch",
        input: cylinder_patch(k),
        op: OffsetOp::Thicken(1.0, OffsetJoin::Arc),
        analytic: Some((
            FRAC_PI_2 / 2.0 * ((PATCH_R + 1.0).powi(2) - PATCH_R * PATCH_R) * PATCH_H,
            EXACT,
        )),
    });
    let bs = bspline_face(k);
    let reference = swept_volume(k, &bs, 1.0);
    out.push(Case {
        name: "thicken_bspline_face",
        input: bs,
        op: OffsetOp::Thicken(1.0, OffsetJoin::Arc),
        analytic: Some((reference, 5e-3)),
    });
    out
}
