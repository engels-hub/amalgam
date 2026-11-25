//! The BREP-15 pipe and thru_sections corpus: circles and squares swept along lines, arcs, a
//! helix and corners, with twist and scale laws, guide, support, fixed and binormal modes, and
//! lofts through circles, squares and a point, each with its analytic volume where one exists.
//! Used by `tests/sweep.rs` (goldens, history, determinism), `tests/history.rs` (completeness)
//! and the bench.
#![allow(dead_code, reason = "shared test helper")]

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use amg_brep::geom::{BSplineCurveData, Curve3, CurveSeg, Tag};
use amg_brep::{
    BrepKernel, Dir3, JobCtx, KResult, LoftSection, OpResult, PipeMode, PipeOpts, Plane, Point3,
    Shape, SubKind, SubShape, ThruSectionsOpts, Tolerance, Transition, Vec3,
};

/// Bound for exact (analytic-surface) cases.
pub const EXACT: f64 = 1e-6;
/// Bound for approximated sweep and loft surfaces.
pub const APPROX: f64 = 1e-4;
/// Bound for the helix (S02 Done-when 1: 0.1 %).
pub const HELIX: f64 = 1e-3;

/// Helix of the corpus: radius, pitch and turns.
pub const HELIX_R: f64 = 10.0;
pub const HELIX_PITCH: f64 = 8.0;
pub const HELIX_TURNS: u32 = 2;

/// What one case runs.
#[derive(Clone, Debug)]
pub enum SweepOp {
    Pipe {
        profiles: usize,
        opts: PipeOpts,
    },
    Loft {
        sections: Vec<LoftSection>,
        opts: ThruSectionsOpts,
    },
}

/// One corpus case: the operands, the op, and the reference volume with its relative bound.
#[derive(Clone, Debug)]
pub struct Case {
    pub name: &'static str,
    pub operands: Vec<Shape>,
    pub op: SweepOp,
    pub analytic: Option<(f64, f64)>,
}

impl Case {
    pub fn run(&self, k: &dyn BrepKernel) -> KResult<OpResult> {
        self.run_with(k, &JobCtx::detached())
    }

    pub fn run_with(&self, k: &dyn BrepKernel, ctx: &JobCtx) -> KResult<OpResult> {
        match &self.op {
            SweepOp::Pipe { profiles, opts } => k.pipe(&self.operands, *profiles, opts, ctx),
            SweepOp::Loft { sections, opts } => {
                k.thru_sections(&self.operands, sections, opts, ctx)
            }
        }
    }

    pub fn is_pipe(&self) -> bool {
        matches!(self.op, SweepOp::Pipe { .. })
    }

    pub fn solid(&self) -> bool {
        match &self.op {
            SweepOp::Pipe { opts, .. } => opts.solid,
            SweepOp::Loft { opts, .. } => opts.solid,
        }
    }
}

pub fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

fn d(v: Vec3) -> Dir3 {
    Dir3::new_unchecked(v.normalize())
}

/// A right-handed frame at `origin` with normal `n` and x axis `x` (made orthogonal to `n`).
pub fn frame(origin: Point3, n: Vec3, x: Vec3) -> Plane {
    let n = n.normalize();
    let x = (x - n * x.dot(n)).normalize();
    Plane {
        origin,
        x: d(x),
        y: d(n.cross(x)),
        normal: d(n),
    }
}

fn seg(curve: Curve3, t0: f64, t1: f64, tag: u64) -> CurveSeg {
    CurveSeg {
        curve,
        t0,
        t1,
        tag: Tag(tag),
    }
}

fn line(a: Point3, b: Point3, tag: u64) -> CurveSeg {
    seg(
        Curve3::Line {
            origin: a,
            dir: d(b - a),
        },
        0.0,
        (b - a).length(),
        tag,
    )
}

pub fn wire(k: &dyn BrepKernel, segs: &[CurveSeg]) -> Shape {
    k.make_wire(segs, &Tolerance::DEFAULT).unwrap().shape
}

/// A full circle of radius `r` in `frame`.
pub fn circle(k: &dyn BrepKernel, frame: Plane, r: f64) -> Shape {
    wire(k, &[seg(Curve3::Circle { frame, radius: r }, 0.0, TAU, 1)])
}

/// A closed polyline through `points`.
pub fn polygon(k: &dyn BrepKernel, points: &[Point3]) -> Shape {
    let n = points.len();
    let segs: Vec<CurveSeg> = (0..n)
        .map(|i| line(points[i], points[(i + 1) % n], 10 + i as u64))
        .collect();
    wire(k, &segs)
}

/// An axis-aligned rectangle `w × h` centred on `frame.origin` in `frame`.
pub fn rectangle(k: &dyn BrepKernel, frame: Plane, w: f64, h: f64) -> Shape {
    let (x, y) = (frame.x.get() * (w / 2.0), frame.y.get() * (h / 2.0));
    let o = frame.origin;
    polygon(k, &[o - x - y, o + x - y, o + x + y, o - x + y])
}

/// An open polyline through `points`.
pub fn polyline(k: &dyn BrepKernel, points: &[Point3]) -> Shape {
    let segs: Vec<CurveSeg> = points
        .windows(2)
        .enumerate()
        .map(|(i, w)| line(w[0], w[1], 100 + i as u64))
        .collect();
    wire(k, &segs)
}

/// The helix point at angle `t` (rad) and its derivative.
fn helix_at(t: f64) -> (Point3, Vec3) {
    let c = HELIX_PITCH / TAU;
    (
        p(HELIX_R * t.cos(), HELIX_R * t.sin(), c * t),
        Vec3::new(-HELIX_R * t.sin(), HELIX_R * t.cos(), c),
    )
}

/// The control points of the helix approximation: one cubic Hermite (Bézier) piece per
/// quarter turn, C1 at the joints.
pub fn helix_poles() -> Vec<Point3> {
    let pieces = 4 * HELIX_TURNS;
    let h = FRAC_PI_2;
    let mut poles = vec![helix_at(0.0).0];
    for i in 0..pieces {
        let (a, b) = (f64::from(i) * h, f64::from(i + 1) * h);
        let ((p0, d0), (p3, d3)) = (helix_at(a), helix_at(b));
        poles.extend([p0 + d0 * (h / 3.0), p3 - d3 * (h / 3.0), p3]);
    }
    poles
}

/// The helix spine: a C1 cubic B-spline, parameter = quarter turns.
pub fn helix(k: &dyn BrepKernel) -> Shape {
    let pieces = 4 * HELIX_TURNS;
    let mut mults = vec![3u32; pieces as usize + 1];
    mults[0] = 4;
    mults[pieces as usize] = 4;
    let data = BSplineCurveData {
        degree: 3,
        knots: (0..=pieces).map(f64::from).collect(),
        mults,
        poles: helix_poles(),
        weights: None,
        periodic: false,
    };
    wire(k, &[seg(Curve3::BSpline(data), 0.0, f64::from(pieces), 7)])
}

/// The length of the helix B-spline, by 16-point Gauss-Legendre on each Bézier piece's speed.
pub fn helix_length() -> f64 {
    let poles = helix_poles();
    let (nodes, weights) = gauss16();
    let mut length = 0.0;
    for piece in poles.windows(4).step_by(3) {
        let [b0, b1, b2, b3] = [piece[0], piece[1], piece[2], piece[3]];
        for (x, w) in nodes.iter().zip(&weights) {
            let u = 0.5 * (x + 1.0);
            let v = 1.0 - u;
            let speed = 3.0 * ((b1 - b0) * v * v + (b2 - b1) * 2.0 * u * v + (b3 - b2) * u * u);
            length += 0.5 * w * speed.length();
        }
    }
    length
}

/// 16-point Gauss-Legendre nodes and weights on [-1, 1] (Golub-Welsch would need a solver;
/// Newton on the Legendre polynomial is enough here).
pub fn gauss16() -> (Vec<f64>, Vec<f64>) {
    let n = 16;
    let mut nodes = Vec::with_capacity(n);
    let mut weights = Vec::with_capacity(n);
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
        nodes.push(x);
        weights.push(2.0 / ((1.0 - x * x) * dp * dp));
    }
    (nodes, weights)
}

/// The volume of the solid of revolution of radius `r(z)` on `[z0, z1]` (Gauss, exact for
/// polynomials up to degree 31).
pub fn revolution_volume(r: impl Fn(f64) -> f64, z0: f64, z1: f64) -> f64 {
    let (nodes, weights) = gauss16();
    let half = 0.5 * (z1 - z0);
    nodes
        .iter()
        .zip(&weights)
        .map(|(x, w)| {
            let z = z0 + half * (x + 1.0);
            w * PI * r(z) * r(z) * half
        })
        .sum()
}

/// The quadratic through `(z0, r0)`, `(z1, r1)`, `(z2, r2)` (Lagrange form).
pub fn quadratic(points: [(f64, f64); 3]) -> impl Fn(f64) -> f64 {
    move |z| {
        let [(a, ra), (b, rb), (c, rc)] = points;
        ra * (z - b) * (z - c) / ((a - b) * (a - c))
            + rb * (z - a) * (z - c) / ((b - a) * (b - c))
            + rc * (z - a) * (z - b) / ((c - a) * (c - b))
    }
}

/// Frames perpendicular to +Z at height `z`.
pub fn xy(z: f64) -> Plane {
    frame(p(0.0, 0.0, z), Vec3::Z, Vec3::X)
}

/// The circular edge of `s` at height `z`.
pub fn top_rim(k: &dyn BrepKernel, s: &Shape, z: f64) -> Shape {
    let e = s
        .edges()
        .find(|&e| {
            matches!(s.curve(e), Some((Curve3::Circle { .. }, _, _)))
                && (k.edge_ends(s, e).unwrap().p0.z - z).abs() < 1e-9
        })
        .unwrap();
    s.sub_shape(SubShape {
        kind: SubKind::Edge,
        index: e.0,
    })
    .unwrap()
}

/// The edge of `s` from `a` to `b` (either way round).
pub fn edge_between(k: &dyn BrepKernel, s: &Shape, a: Point3, b: Point3) -> Shape {
    for e in s.edges() {
        let ends = k.edge_ends(s, e).unwrap();
        let fits =
            |x: Point3, y: Point3| (ends.p0 - x).length() < 1e-9 && (ends.p1 - y).length() < 1e-9;
        if fits(a, b) || fits(b, a) {
            return s
                .sub_shape(SubShape {
                    kind: SubKind::Edge,
                    index: e.0,
                })
                .unwrap();
        }
    }
    panic!("no edge from {a} to {b}");
}

fn pipe_opts() -> PipeOpts {
    PipeOpts::default()
}

fn pipe(
    name: &'static str,
    operands: Vec<Shape>,
    profiles: usize,
    opts: PipeOpts,
    analytic: Option<(f64, f64)>,
) -> Case {
    Case {
        name,
        operands,
        op: SweepOp::Pipe { profiles, opts },
        analytic,
    }
}

fn loft(
    name: &'static str,
    operands: Vec<Shape>,
    sections: Vec<LoftSection>,
    opts: ThruSectionsOpts,
    analytic: Option<(f64, f64)>,
) -> Case {
    Case {
        name,
        operands,
        op: SweepOp::Loft { sections, opts },
        analytic,
    }
}

/// The three smooth-loft circles: radii 2, 4, 3 at heights 0, 2, 3. Both spans rise as much as
/// the radius changes, so OCCT's chord-length parameters (0, 2/3, 1) are those of the heights
/// for every pole, and the smooth surface is exactly the revolution of the quadratic `r(z)`.
pub const LOFT_CIRCLES: [(f64, f64); 3] = [(0.0, 2.0), (2.0, 4.0), (3.0, 3.0)];

/// The corpus.
pub fn corpus(k: &dyn BrepKernel) -> Vec<Case> {
    let mut out = Vec::new();
    let r = 2.0;
    let len = 30.0;
    let z_line = |k: &dyn BrepKernel, l: f64| polyline(k, &[p(0.0, 0.0, 0.0), p(0.0, 0.0, l)]);

    // Done-when 1: a circle along a line (cylinder) and along an arc (torus segment).
    out.push(pipe(
        "pipe_circle_line",
        vec![circle(k, xy(0.0), r), z_line(k, len)],
        1,
        pipe_opts(),
        Some((PI * r * r * len, EXACT)),
    ));
    let big = 20.0;
    let arc_frame = Plane {
        origin: p(big, 0.0, 0.0),
        x: d(-Vec3::X),
        y: d(Vec3::Z),
        normal: d(Vec3::Y),
    };
    let arc = wire(
        k,
        &[seg(
            Curve3::Circle {
                frame: arc_frame,
                radius: big,
            },
            0.0,
            FRAC_PI_2,
            3,
        )],
    );
    out.push(pipe(
        "pipe_circle_arc",
        vec![circle(k, xy(0.0), r), arc.clone()],
        1,
        pipe_opts(),
        Some((PI * r * r * big * FRAC_PI_2, EXACT)),
    ));
    // A square along the helix: area × length (the section's centroid follows the spine).
    let (start, tangent) = helix_at(0.0);
    let helix_frame = frame(start, tangent, Vec3::X);
    out.push(pipe(
        "pipe_square_helix",
        vec![rectangle(k, helix_frame, 2.0, 2.0), helix(k)],
        1,
        pipe_opts(),
        Some((4.0 * helix_length(), HELIX)),
    ));
    out.push(pipe(
        "pipe_square_helix_frenet",
        vec![rectangle(k, helix_frame, 2.0, 2.0), helix(k)],
        1,
        PipeOpts {
            mode: PipeMode::Frenet,
            ..pipe_opts()
        },
        Some((4.0 * helix_length(), HELIX)),
    ));

    // Done-when 2: a 90° twist of a square (each section is the square turned: area × length)
    // and a linear scale 1 → 2 of a circle (π r² L ∫(1 + t)² dt = π r² L 7/3).
    let side = 4.0;
    out.push(pipe(
        "pipe_square_twist",
        vec![rectangle(k, xy(0.0), side, side), z_line(k, 20.0)],
        1,
        PipeOpts {
            twist: Some(vec![(0.0, 0.0), (1.0, FRAC_PI_2)]),
            ..pipe_opts()
        },
        Some((side * side * 20.0, APPROX)),
    ));
    out.push(pipe(
        "pipe_rectangle_twist_law",
        vec![rectangle(k, xy(0.0), 6.0, 2.0), z_line(k, 20.0)],
        1,
        PipeOpts {
            twist: Some(vec![(0.0, 0.0), (0.5, 0.3), (1.0, PI)]),
            ..pipe_opts()
        },
        Some((12.0 * 20.0, APPROX)),
    ));
    out.push(pipe(
        "pipe_circle_scale",
        vec![circle(k, xy(0.0), r), z_line(k, 20.0)],
        1,
        PipeOpts {
            scale: Some(vec![(0.0, 1.0), (1.0, 2.0)]),
            ..pipe_opts()
        },
        Some((PI * r * r * 20.0 * 7.0 / 3.0, APPROX)),
    ));
    // Two spine edges: the law runs along the whole spine.
    out.push(pipe(
        "pipe_circle_scale_two_edges",
        vec![
            circle(k, xy(0.0), r),
            polyline(k, &[p(0.0, 0.0, 0.0), p(0.0, 0.0, 5.0), p(0.0, 0.0, 20.0)]),
        ],
        1,
        PipeOpts {
            scale: Some(vec![(0.0, 1.0), (1.0, 2.0)]),
            ..pipe_opts()
        },
        Some((PI * r * r * 20.0 * 7.0 / 3.0, APPROX)),
    ));

    // Corners: a mitred square along an L (area × centre-line length), and round corners.
    let corner = |k: &dyn BrepKernel| {
        polyline(
            k,
            &[p(0.0, 0.0, 0.0), p(0.0, 0.0, 10.0), p(10.0, 0.0, 10.0)],
        )
    };
    out.push(pipe(
        "pipe_square_corner_right",
        vec![rectangle(k, xy(0.0), 2.0, 2.0), corner(k)],
        1,
        PipeOpts {
            transition: Transition::RightCorner,
            ..pipe_opts()
        },
        Some((4.0 * 20.0, EXACT)),
    ));
    out.push(pipe(
        "pipe_square_corner_round",
        vec![rectangle(k, xy(0.0), 2.0, 2.0), corner(k)],
        1,
        PipeOpts {
            transition: Transition::RoundCorner,
            ..pipe_opts()
        },
        None,
    ));

    // Modes: fixed (a prism), constant binormal along a planar arc (as Frenet), guide, support.
    out.push(pipe(
        "pipe_square_fixed",
        vec![rectangle(k, xy(0.0), 2.0, 3.0), z_line(k, 15.0)],
        1,
        PipeOpts {
            mode: PipeMode::Fixed {
                binormal: Dir3::new_unchecked(Vec3::Y),
            },
            ..pipe_opts()
        },
        Some((6.0 * 15.0, EXACT)),
    ));
    out.push(pipe(
        "pipe_square_binormal_arc",
        vec![rectangle(k, xy(0.0), 2.0, 2.0), arc],
        1,
        PipeOpts {
            mode: PipeMode::Binormal(Dir3::new_unchecked(Vec3::Y)),
            ..pipe_opts()
        },
        Some((4.0 * big * FRAC_PI_2, APPROX)),
    ));
    out.push(pipe(
        "pipe_square_guide",
        vec![
            rectangle(k, xy(0.0), 2.0, 2.0),
            z_line(k, 20.0),
            polyline(k, &[p(5.0, 0.0, 0.0), p(5.0, 0.0, 20.0)]),
        ],
        1,
        PipeOpts {
            mode: PipeMode::Guide {
                wire: 2,
                contact: false,
            },
            ..pipe_opts()
        },
        Some((4.0 * 20.0, APPROX)),
    ));
    // Support mode: a circle along the top rim of a cylinder, the binormal following the
    // cylinder's normal (a torus, 2π² r² R). OCCT raises `Standard_NotImplemented`
    // (`Adaptor3d_CurveOnSurface::EvalDN`) for straight spine edges on a support.
    let support = k.make_cylinder(&Plane::XY, 10.0, 20.0, TAU).unwrap().shape;
    let rim = top_rim(k, &support, 20.0);
    out.push(pipe(
        "pipe_circle_support",
        vec![
            circle(k, frame(p(10.0, 0.0, 20.0), Vec3::Y, Vec3::X), 1.0),
            rim,
            support,
        ],
        1,
        PipeOpts {
            mode: PipeMode::Support { shell: 2 },
            ..pipe_opts()
        },
        Some((2.0 * PI * PI * 10.0, APPROX)),
    ));

    // Several sections (a frustum), a point section (a cone) and an open shell.
    let (r1, r2, h) = (2.0, 3.0, 20.0);
    out.push(pipe(
        "pipe_two_sections",
        vec![circle(k, xy(0.0), r1), circle(k, xy(h), r2), z_line(k, h)],
        2,
        pipe_opts(),
        Some((PI * h * (r1 * r1 + r1 * r2 + r2 * r2) / 3.0, APPROX)),
    ));
    out.push(pipe(
        "pipe_point_to_circle",
        vec![
            vertex(k, p(0.0, 0.0, 0.0)),
            circle(k, xy(h), r2),
            z_line(k, h),
        ],
        2,
        pipe_opts(),
        Some((PI * r2 * r2 * h / 3.0, APPROX)),
    ));
    out.push(pipe(
        "pipe_shell_circle_line",
        vec![circle(k, xy(0.0), r), z_line(k, len)],
        1,
        PipeOpts {
            solid: false,
            ..pipe_opts()
        },
        None,
    ));

    // Done-when 3: the smooth loft through three circles, a point-section cone, ruled squares.
    let circles: Vec<Shape> = LOFT_CIRCLES
        .iter()
        .map(|&(z, r)| circle(k, xy(z), r))
        .collect();
    let wires = |n: usize| (0..n).map(LoftSection::Wire).collect::<Vec<_>>();
    out.push(loft(
        "loft_three_circles_smooth",
        circles.clone(),
        wires(3),
        ThruSectionsOpts::default(),
        Some((revolution_volume(quadratic(LOFT_CIRCLES), 0.0, 3.0), APPROX)),
    ));
    out.push(loft(
        "loft_three_circles_ruled",
        circles.clone(),
        wires(3),
        ThruSectionsOpts {
            ruled: true,
            ..ThruSectionsOpts::default()
        },
        Some((
            PI * 2.0 * (4.0 + 8.0 + 16.0) / 3.0 + PI * (16.0 + 12.0 + 9.0) / 3.0,
            EXACT,
        )),
    ));
    out.push(loft(
        "loft_three_circles_shell",
        circles,
        wires(3),
        ThruSectionsOpts {
            solid: false,
            ..ThruSectionsOpts::default()
        },
        None,
    ));
    let (cr, ch) = (3.0, 6.0);
    out.push(loft(
        "loft_cone_point",
        vec![circle(k, xy(0.0), cr)],
        vec![LoftSection::Wire(0), LoftSection::Point(p(0.0, 0.0, ch))],
        ThruSectionsOpts::default(),
        Some((PI * cr * cr * ch / 3.0, APPROX)),
    ));
    out.push(loft(
        "loft_point_cone_point",
        vec![circle(k, xy(0.0), cr)],
        vec![
            LoftSection::Point(p(0.0, 0.0, -ch)),
            LoftSection::Wire(0),
            LoftSection::Point(p(0.0, 0.0, ch)),
        ],
        ThruSectionsOpts {
            ruled: true,
            ..ThruSectionsOpts::default()
        },
        Some((2.0 * PI * cr * cr * ch / 3.0, APPROX)),
    ));
    let squares: Vec<Shape> = [(0.0, 4.0), (5.0, 6.0), (10.0, 4.0)]
        .iter()
        .map(|&(z, s)| rectangle(k, xy(z), s, s))
        .collect();
    // Two square frusta: h/3 (A1 + A2 + √(A1 A2)) each.
    out.push(loft(
        "loft_squares_ruled",
        squares.clone(),
        wires(3),
        ThruSectionsOpts {
            ruled: true,
            ..ThruSectionsOpts::default()
        },
        Some((2.0 * 5.0 / 3.0 * (16.0 + 36.0 + 24.0), EXACT)),
    ));
    out.push(loft(
        "loft_squares_smooth",
        squares,
        wires(3),
        ThruSectionsOpts::default(),
        None,
    ));
    // Different edge counts: the compatibility check splits the circle to match the square.
    out.push(loft(
        "loft_circle_to_square",
        vec![circle(k, xy(0.0), 3.0), rectangle(k, xy(8.0), 4.0, 4.0)],
        wires(2),
        ThruSectionsOpts::default(),
        None,
    ));
    out
}

/// The heavy loft of the cancel test and the bench: 40 sections 2 mm apart, each a 48-gon of
/// radius `5 + sin(0.4 z)`.
pub fn heavy_loft(k: &dyn BrepKernel) -> (Vec<Shape>, Vec<LoftSection>) {
    let wires = (0..40)
        .map(|i| {
            let z = f64::from(i) * 2.0;
            let r = 5.0 + (z * 0.4).sin();
            let points: Vec<Point3> = (0..48)
                .map(|j| {
                    let a = f64::from(j) * TAU / 48.0;
                    p(r * a.cos(), r * a.sin(), z)
                })
                .collect();
            polygon(k, &points)
        })
        .collect();
    (wires, (0..40).map(LoftSection::Wire).collect())
}

/// A vertex at `at` (the start vertex of a short line wire).
pub fn vertex(k: &dyn BrepKernel, at: Point3) -> Shape {
    let w = wire(k, &[line(at, at + Vec3::Z, 0)]);
    let v = w
        .vertices()
        .find(|&v| (w.vertex_point(v) - at).length() < 1e-12)
        .unwrap();
    w.sub_shape(SubShape {
        kind: SubKind::Vertex,
        index: v.0,
    })
    .unwrap()
}
