//! `gen_fillet_corpus`: writes the handmade fillet corpus (S20 KH-02) to
//! `tests/golden/kernel/fillet/handmade/<name>/` (or the directory given as the first argument):
//! `case.toml` (schema 1), `op.json` (`KernelOp::Fillet` on `InputRef(0)`) and `inputs/0.brep`.
//!
//! ```text
//! cargo run -p amg-brep --features occt --example gen_fillet_corpus [-- <out-dir>]
//! ```
//!
//! The cases are classic fillet situations (cube and box edges up to and past their limits,
//! loops, three-edge corners, variable radii, concave edges, prisms with sharp and nearly flat
//! vertices, cylinder and cone rims, holes, bosses, pockets, intersection curves, a fillet after
//! a chamfer), each with an analytic volume where one exists. The `.brep` files are committed,
//! so the harness does not need this program; rerun it only to change the handmade set.
#![allow(
    clippy::print_stdout,
    clippy::print_stderr,
    reason = "command-line tool"
)]
#![allow(
    clippy::expect_used,
    reason = "a generator: a failing part build is a bug here"
)]

use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI, TAU};
use std::fs;
use std::path::{Path, PathBuf};

use amg_brep::capture::{EdgeRecord, InputRef, KernelOp};
use amg_brep::geom::{Curve2d, GeomClass, Loop2, PlanarProfile, ProfileEdge2, Tag};
use amg_brep::robust::corpus::case::{CASE_SCHEMA, CaseToml, Expect, fillet_corpus_dir};
use amg_brep::robust::corpus::fillet::PrismPart;
use amg_brep::{
    BoolOp, BoolOpts, BrepKernel, ChamferSpec, Dir3, EdgeId, FilletOpts, FilletShape, FilletSpec,
    JobCtx, KernelConfig, OcctKernel, Plane, Point2, Point3, PrismExtent, PrismOpts, Shape,
    SubKind, SubShape, SubShapeGeom, Tolerance, Vec3,
};

const TOL: Tolerance = Tolerance::DEFAULT;

fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

/// Cross-section area of a radius-`r` fillet's spandrel at a right angle.
fn s(r: f64) -> f64 {
    (1.0 - FRAC_PI_4) * r * r
}

/// Distance of the spandrel's centroid from either face.
fn c(r: f64) -> f64 {
    r * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI)
}

/// A convex polygon prism of height `h` with its top boundary filleted by `r` and sharp
/// vertical edges: the top band's section at depth `z` is the polygon inset by
/// `δ = r − √(r² − z²)`, of area `A − Pδ + δ² Σ cot(αᵢ/2)`, so
/// `V = A·h − P·s(r) + (5/3 − π/2)·r³·Σ cot(αᵢ/2)`.
fn top_loop_volume(poly: &[Point2], h: f64, r: f64) -> f64 {
    let part = PrismPart {
        polygon: poly.to_vec(),
        height: h,
        vertical: Vec::new(),
        top: Vec::new(),
    };
    let n = poly.len();
    let perimeter: f64 = (0..n).map(|j| part.edge_len(j)).sum();
    let cots: f64 = (0..n)
        .map(|i| 1.0 / (0.5 * part.interior_angle(i)).tan())
        .sum();
    part.area() * h - perimeter * s(r) + (5.0 / 3.0 - FRAC_PI_2) * r * r * r * cots
}

fn rect(w: f64, d: f64) -> Vec<Point2> {
    vec![
        Point2::new(0.0, 0.0),
        Point2::new(w, 0.0),
        Point2::new(w, d),
        Point2::new(0.0, d),
    ]
}

fn regular(n: usize, radius: f64) -> Vec<Point2> {
    (0..n)
        .map(|i| {
            #[allow(clippy::cast_precision_loss, reason = "small counts")]
            let t = TAU * i as f64 / n as f64;
            Point2::new(radius * t.cos(), radius * t.sin())
        })
        .collect()
}

fn star(points: usize, outer: f64, inner: f64) -> Vec<Point2> {
    (0..2 * points)
        .map(|i| {
            #[allow(clippy::cast_precision_loss, reason = "small counts")]
            let t = PI * i as f64 / points as f64 + FRAC_PI_2;
            let r = if i % 2 == 0 { outer } else { inner };
            Point2::new(r * t.cos(), r * t.sin())
        })
        .collect()
}

fn l_profile() -> Vec<Point2> {
    vec![
        Point2::new(0.0, 0.0),
        Point2::new(10.0, 0.0),
        Point2::new(10.0, 4.0),
        Point2::new(4.0, 4.0),
        Point2::new(4.0, 10.0),
        Point2::new(0.0, 10.0),
    ]
}

/// The exact volume of a prism with vertical fillets (the generator's oracle).
fn prism_volume(poly: &[Point2], h: f64, vertical: &[(usize, f64)]) -> f64 {
    PrismPart {
        polygon: poly.to_vec(),
        height: h,
        vertical: vertical.to_vec(),
        top: Vec::new(),
    }
    .vertical_volume()
}

struct Parts<'k> {
    k: &'k dyn BrepKernel,
    ctx: JobCtx,
}

impl Parts<'_> {
    fn block(&self, frame: &Plane, size: Vec3) -> Shape {
        self.k.make_box(frame, size).expect("box").shape
    }

    fn cube(&self, a: f64) -> Shape {
        self.block(&Plane::XY, Vec3::splat(a))
    }

    fn prism(&self, poly: &[Point2], h: f64) -> Shape {
        let pts: Vec<Point3> = poly.iter().map(|q| p(q.x, q.y, 0.0)).collect();
        let face = self
            .k
            .make_planar_polygon(&pts, &[], &TOL)
            .expect("polygon");
        self.k
            .prism(
                &face,
                &PrismExtent::Vector(Vec3::new(0.0, 0.0, h)),
                &PrismOpts {
                    taper: 0.0,
                    merge_coplanar: false,
                },
                &self.ctx,
            )
            .expect("prism")
            .shape
    }

    fn cylinder(&self, frame: &Plane, r: f64, h: f64) -> Shape {
        self.k
            .make_cylinder(frame, r, h, TAU)
            .expect("cylinder")
            .shape
    }

    fn boolean(&self, op: BoolOp, a: &Shape, b: &Shape) -> Shape {
        self.k
            .boolean(
                op,
                std::slice::from_ref(a),
                std::slice::from_ref(b),
                &BoolOpts::default(),
                &self.ctx,
            )
            .expect("boolean")
            .shape
    }

    fn slot(&self, len: f64, r: f64, h: f64) -> Shape {
        let line = |a: (f64, f64), b: (f64, f64)| Curve2d::Line {
            a: Point2::new(a.0, a.1),
            b: Point2::new(b.0, b.1),
        };
        let curves = vec![
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
        ];
        let profile = PlanarProfile {
            plane: Plane::XY,
            outer: Loop2 {
                edges: curves
                    .into_iter()
                    .zip(0..)
                    .map(|(curve, t)| ProfileEdge2 { curve, tag: Tag(t) })
                    .collect(),
            },
            holes: Vec::new(),
        };
        let face = self
            .k
            .make_planar_faces(&[profile], &TOL)
            .expect("face")
            .remove(0)
            .face;
        self.k
            .prism(
                &face,
                &PrismExtent::Vector(Vec3::new(0.0, 0.0, h)),
                &PrismOpts {
                    taper: 0.0,
                    merge_coplanar: false,
                },
                &self.ctx,
            )
            .expect("prism")
            .shape
    }

    fn geoms(&self, sh: &Shape) -> Vec<(EdgeId, SubShapeGeom)> {
        sh.edges()
            .map(|e| {
                let g = self
                    .k
                    .sub_geom(
                        sh,
                        SubShape {
                            kind: SubKind::Edge,
                            index: e.0,
                        },
                    )
                    .expect("sub_geom");
                (e, g)
            })
            .collect()
    }

    /// The one edge whose length-weighted centre is `at`.
    fn at(&self, sh: &Shape, at: Point3) -> EdgeId {
        let hits: Vec<EdgeId> = self
            .geoms(sh)
            .into_iter()
            .filter(|(_, g)| g.centroid.distance(at) < 1e-6)
            .map(|(e, _)| e)
            .collect();
        assert_eq!(hits.len(), 1, "edges centred at {at:?}: {hits:?}");
        hits[0]
    }

    /// Every edge matching `pred`, at least one.
    fn wher(&self, sh: &Shape, pred: impl Fn(&SubShapeGeom) -> bool) -> Vec<EdgeId> {
        let hits: Vec<EdgeId> = self
            .geoms(sh)
            .into_iter()
            .filter(|(_, g)| pred(g))
            .map(|(e, _)| e)
            .collect();
        assert!(!hits.is_empty(), "no edge matches");
        hits
    }

    /// The vertical edges of a prism over `poly` at the given vertex indices.
    fn verticals(&self, sh: &Shape, poly: &[Point2], h: f64, idx: &[usize]) -> Vec<EdgeId> {
        idx.iter()
            .map(|&i| self.at(sh, p(poly[i].x, poly[i].y, 0.5 * h)))
            .collect()
    }

    /// Every edge of the top face `z = h` of a prism over `poly`.
    fn top_edges(&self, sh: &Shape, poly: &[Point2], h: f64) -> Vec<EdgeId> {
        let n = poly.len();
        (0..n)
            .map(|j| {
                let m = 0.5 * (poly[j] + poly[(j + 1) % n]);
                self.at(sh, p(m.x, m.y, h))
            })
            .collect()
    }
}

/// One handmade case.
struct Hand {
    name: &'static str,
    description: &'static str,
    tags: &'static [&'static str],
    input: Shape,
    edges: Vec<(EdgeId, FilletSpec)>,
    opts: FilletOpts,
    expect: Expect,
}

fn exact(v: f64) -> Expect {
    Expect {
        volume: Some(v),
        volume_rel_tol: Some(1e-6),
        ..Expect::success()
    }
}

fn exact_tol(v: f64, tol: f64) -> Expect {
    Expect {
        volume_rel_tol: Some(tol),
        ..exact(v)
    }
}

fn bounds(lo: f64, hi: f64) -> Expect {
    Expect {
        volume_bounds: Some([lo, hi]),
        ..Expect::success()
    }
}

fn too_large(max_radius: f64) -> Expect {
    Expect {
        max_radius: Some(max_radius),
        ..Expect::clean_failure(amg_brep::capture::FailureClass::RadiusTooLarge)
    }
}

fn constant(edges: Vec<EdgeId>, r: f64) -> Vec<(EdgeId, FilletSpec)> {
    edges
        .into_iter()
        .map(|e| (e, FilletSpec::Constant(r)))
        .collect()
}

#[allow(clippy::too_many_lines, reason = "the case list")]
fn cases(k: &dyn BrepKernel) -> Vec<Hand> {
    let ps = Parts {
        k,
        ctx: JobCtx::detached(),
    };
    let mut out = Vec::new();
    let mut add = |name, description, tags, input: &Shape, edges, opts, expect| {
        out.push(Hand {
            name,
            description,
            tags,
            input: input.clone(),
            edges,
            opts,
            expect,
        });
    };
    let d = FilletOpts::default;

    // -- cube [0, 10]³ --
    let a = 10.0;
    let h = 0.5 * a;
    let cube = ps.cube(a);
    let front = ps.at(&cube, p(h, 0.0, 0.0));
    let one = |r: f64| constant(vec![front], r);
    add(
        "cube-edge-r1",
        "one cube edge, r = 1",
        &["analytic", "convex", "smoke"],
        &cube,
        one(1.0),
        d(),
        exact(1000.0 - a * s(1.0)),
    );
    add(
        "cube-edge-r2-quasi-angular",
        "one cube edge, quasi-angular blend",
        &["analytic", "convex", "smoke", "shape_variant"],
        &cube,
        one(2.0),
        FilletOpts {
            shape: FilletShape::QuasiAngular,
            ..d()
        },
        exact(1000.0 - a * s(2.0)),
    );
    add(
        "cube-edge-r2-polynomial",
        "one cube edge, polynomial blend (an approximation of the circle)",
        &["analytic", "convex", "shape_variant"],
        &cube,
        one(2.0),
        FilletOpts {
            shape: FilletShape::Polynomial,
            ..d()
        },
        exact_tol(1000.0 - a * s(2.0), 1e-4),
    );
    add(
        "cube-edge-r4p9",
        "one cube edge, r just below half the face",
        &["analytic", "convex"],
        &cube,
        one(4.9),
        d(),
        exact(1000.0 - a * s(4.9)),
    );
    add(
        "cube-edge-r5",
        "one cube edge, r = half the face",
        &["analytic", "convex"],
        &cube,
        one(5.0),
        d(),
        exact(1000.0 - a * s(5.0)),
    );
    add(
        "cube-edge-r9p9",
        "one cube edge, r just below the face width",
        &["analytic", "convex", "near_limit"],
        &cube,
        one(9.9),
        d(),
        exact(1000.0 - a * s(9.9)),
    );
    add(
        "cube-edge-r10-full",
        "one cube edge, r = the face width: both faces consumed",
        &["analytic", "convex", "near_limit"],
        &cube,
        one(10.0),
        d(),
        exact(1000.0 - a * s(10.0)),
    );
    add(
        "cube-edge-r10p5-too-large",
        "one cube edge, r above the face width",
        &["analytic", "radius_too_large"],
        &cube,
        one(10.5),
        d(),
        too_large(10.0),
    );
    add(
        "cube-edge-r25-too-large",
        "one cube edge, r 2.5 times the face width",
        &["analytic", "radius_too_large", "smoke"],
        &cube,
        one(25.0),
        d(),
        too_large(10.0),
    );
    let back_top = ps.at(&cube, p(h, a, a));
    add(
        "cube-two-parallel-edges",
        "two opposite cube edges, r = 2",
        &["analytic", "convex", "smoke"],
        &cube,
        constant(vec![front, back_top], 2.0),
        d(),
        exact(1000.0 - 2.0 * a * s(2.0)),
    );
    let top: Vec<EdgeId> = [p(h, 0.0, a), p(a, h, a), p(h, a, a), p(0.0, h, a)]
        .into_iter()
        .map(|q| ps.at(&cube, q))
        .collect();
    let sq = rect(a, a);
    add(
        "cube-top-loop-r2",
        "the 4 top edges of a cube (a loop), r = 2",
        &["analytic", "convex", "loop", "smoke"],
        &cube,
        constant(top.clone(), 2.0),
        d(),
        exact(top_loop_volume(&sq, a, 2.0)),
    );
    add(
        "cube-top-loop-r4p9",
        "the 4 top edges of a cube, r = 4.9",
        &["analytic", "convex", "loop", "near_limit"],
        &cube,
        constant(top.clone(), 4.9),
        d(),
        exact(top_loop_volume(&sq, a, 4.9)),
    );
    let corner: Vec<EdgeId> = [p(h, 0.0, 0.0), p(0.0, h, 0.0), p(0.0, 0.0, h)]
        .into_iter()
        .map(|q| ps.at(&cube, q))
        .collect();
    let corner3 = |r: f64| a * a * a - 3.0 * (a - r) * s(r) - (1.0 - PI / 6.0) * r * r * r;
    add(
        "cube-corner3-r2",
        "the 3 edges at one cube vertex, r = 2",
        &["analytic", "convex", "corner3", "smoke"],
        &cube,
        constant(corner.clone(), 2.0),
        d(),
        exact(corner3(2.0)),
    );
    add(
        "cube-corner3-r4",
        "the 3 edges at one cube vertex, r = 4",
        &["analytic", "convex", "corner3"],
        &cube,
        constant(corner.clone(), 4.0),
        d(),
        exact(corner3(4.0)),
    );
    let all: Vec<EdgeId> = cube.edges().collect();
    let rounded = |r: f64| {
        let b = a - 2.0 * r;
        b * b * b + 6.0 * b * b * r + 3.0 * PI * b * r * r + 4.0 / 3.0 * PI * r * r * r
    };
    add(
        "cube-all-edges-r1",
        "all 12 cube edges, r = 1",
        &["analytic", "convex", "corner3", "smoke"],
        &cube,
        constant(all.clone(), 1.0),
        d(),
        exact(rounded(1.0)),
    );
    add(
        "cube-all-edges-r3",
        "all 12 cube edges, r = 3",
        &["analytic", "convex", "corner3"],
        &cube,
        constant(all.clone(), 3.0),
        d(),
        exact(rounded(3.0)),
    );
    add(
        "cube-all-edges-r4p9",
        "all 12 cube edges, r = 4.9 (almost a sphere)",
        &["analytic", "convex", "corner3", "near_limit"],
        &cube,
        constant(all.clone(), 4.9),
        d(),
        exact(rounded(4.9)),
    );
    add(
        "cube-all-edges-r5-sphere",
        "all 12 cube edges, r = 5: the cube becomes a sphere",
        &["analytic", "convex", "corner3", "near_limit"],
        &cube,
        constant(all.clone(), 5.0),
        d(),
        exact(rounded(5.0)),
    );
    let left = ps.at(&cube, p(0.0, h, 0.0));
    add(
        "cube-mixed-radii",
        "two edges meeting at a vertex, r = 1 and 2",
        &["analytic", "convex", "mixed_radii", "smoke"],
        &cube,
        vec![
            (front, FilletSpec::Constant(1.0)),
            (left, FilletSpec::Constant(2.0)),
        ],
        d(),
        bounds(
            1000.0 - a * (s(1.0) + s(2.0)) - 1.0,
            1000.0 - a * s(2.0) + 1.0,
        ),
    );
    add(
        "cube-corner3-mixed-radii",
        "the 3 edges at one vertex with r = 1, 2, 3",
        &["analytic", "convex", "corner3", "mixed_radii"],
        &cube,
        corner
            .iter()
            .zip([1.0, 2.0, 3.0])
            .map(|(&e, r)| (e, FilletSpec::Constant(r)))
            .collect(),
        d(),
        bounds(
            1000.0 - a * (s(1.0) + s(2.0) + s(3.0)) - 1.0,
            1000.0 - a * s(3.0) + 1.0,
        ),
    );
    add(
        "cube-variable-linear",
        "one cube edge, radius 1 to 3",
        &["analytic", "convex", "variable", "smoke"],
        &cube,
        vec![(front, FilletSpec::Variable(vec![(0.0, 1.0), (1.0, 3.0)]))],
        d(),
        bounds(1000.0 - a * s(3.0), 1000.0 - a * s(1.0)),
    );
    add(
        "cube-variable-bump",
        "one cube edge, radius 1, 2.5, 1",
        &["analytic", "convex", "variable"],
        &cube,
        vec![(
            front,
            FilletSpec::Variable(vec![(0.0, 1.0), (0.5, 2.5), (1.0, 1.0)]),
        )],
        d(),
        bounds(1000.0 - a * s(2.5), 1000.0 - a * s(1.0)),
    );
    add(
        "cube-chord",
        "one cube edge by chord length 2√2 (r = 2)",
        &["analytic", "convex", "chord", "smoke"],
        &cube,
        vec![(front, FilletSpec::Chord(2.0 * 2f64.sqrt()))],
        d(),
        exact(1000.0 - a * s(2.0)),
    );
    add(
        "cube-edge-r1e-3",
        "one cube edge, r = 0.001 mm",
        &["analytic", "convex", "tiny"],
        &cube,
        one(1e-3),
        d(),
        exact(1000.0 - a * s(1e-3)),
    );
    add(
        "cube-edge-r1e-2",
        "one cube edge, r = 0.01 mm",
        &["analytic", "convex", "tiny"],
        &cube,
        one(1e-2),
        d(),
        exact(1000.0 - a * s(1e-2)),
    );

    // -- boxes --
    let tall = ps.block(&Plane::XY, Vec3::new(10.0, 20.0, 30.0));
    add(
        "box-long-edge-r4",
        "a 30 mm vertical box edge, r = 4",
        &["analytic", "convex", "smoke"],
        &tall,
        constant(vec![ps.at(&tall, p(10.0, 20.0, 15.0))], 4.0),
        d(),
        exact(6000.0 - 30.0 * s(4.0)),
    );
    let flat = ps.block(&Plane::XY, Vec3::new(20.0, 30.0, 10.0));
    let fv: Vec<EdgeId> = [
        p(0.0, 0.0, 5.0),
        p(20.0, 0.0, 5.0),
        p(20.0, 30.0, 5.0),
        p(0.0, 30.0, 5.0),
    ]
    .into_iter()
    .map(|q| ps.at(&flat, q))
    .collect();
    add(
        "box-vertical-four-r3",
        "the 4 vertical edges of a box, r = 3",
        &["analytic", "convex", "smoke"],
        &flat,
        constant(fv, 3.0),
        d(),
        exact(6000.0 - 40.0 * s(3.0)),
    );
    let plate = ps.block(&Plane::XY, Vec3::new(50.0, 40.0, 2.0));
    let pr = rect(50.0, 40.0);
    let plate_top = ps.top_edges(&plate, &pr, 2.0);
    let plate_bottom = ps.top_edges(&plate, &pr, 0.0);
    let band = |r: f64| 2000.0 * 2.0 - top_loop_volume(&pr, 2.0, r);
    add(
        "plate-top-loop-r0p9",
        "the top loop of a 2 mm plate, r = 0.9",
        &["analytic", "convex", "loop", "thin"],
        &plate,
        constant(plate_top.clone(), 0.9),
        d(),
        exact(top_loop_volume(&pr, 2.0, 0.9)),
    );
    add(
        "plate-both-loops-r0p9",
        "top and bottom loops of a 2 mm plate, r = 0.9",
        &["analytic", "convex", "loop", "thin"],
        &plate,
        constant([plate_top.clone(), plate_bottom].concat(), 0.9),
        d(),
        exact(4000.0 - 2.0 * band(0.9)),
    );
    add(
        "plate-edge-r2p5-too-large",
        "a long top edge of a 2 mm plate, r = 2.5",
        &["analytic", "radius_too_large", "thin"],
        &plate,
        constant(vec![plate_top[0]], 2.5),
        d(),
        too_large(2.0),
    );
    let big = ps.block(&Plane::XY, Vec3::new(1000.0, 800.0, 600.0));
    add(
        "big-box-edge-r50",
        "a 1000 × 800 × 600 box edge, r = 50",
        &["analytic", "convex", "large"],
        &big,
        constant(vec![ps.at(&big, p(0.0, 0.0, 300.0))], 50.0),
        d(),
        exact(4.8e8 - 600.0 * s(50.0)),
    );
    let small = ps.cube(1.0);
    add(
        "small-cube-edge-r0p2",
        "a 1 mm cube edge, r = 0.2",
        &["analytic", "convex", "small"],
        &small,
        constant(vec![ps.at(&small, p(0.5, 0.0, 0.0))], 0.2),
        d(),
        exact(1.0 - s(0.2)),
    );
    let tilted = {
        let q = amg_core::Quat::from_euler(amg_core::glam::EulerRot::ZXY, 0.4, -0.7, 1.1);
        Plane {
            origin: p(3.0, -2.0, 5.0),
            x: Dir3::new_unchecked(q * Vec3::X),
            y: Dir3::new_unchecked(q * Vec3::Y),
            normal: Dir3::new_unchecked(q * Vec3::Z),
        }
    };
    let tbox = ps.block(&tilted, Vec3::new(10.0, 20.0, 30.0));
    add(
        "tilted-box-edge-r3",
        "a box in a frame off every axis, r = 3",
        &["analytic", "convex", "smoke"],
        &tbox,
        constant(
            vec![ps.at(&tbox, tilted.origin + 5.0 * tilted.x.get())],
            3.0,
        ),
        d(),
        exact(6000.0 - 10.0 * s(3.0)),
    );

    // -- prisms --
    let l = l_profile();
    let lb = ps.prism(&l, 6.0);
    add(
        "l-block-concave-r1",
        "the concave vertical edge of an L block, r = 1",
        &["analytic", "concave", "smoke"],
        &lb,
        constant(ps.verticals(&lb, &l, 6.0, &[3]), 1.0),
        d(),
        exact(prism_volume(&l, 6.0, &[(3, 1.0)])),
    );
    add(
        "l-block-concave-r3",
        "the concave vertical edge of an L block, r = 3",
        &["analytic", "concave"],
        &lb,
        constant(ps.verticals(&lb, &l, 6.0, &[3]), 3.0),
        d(),
        exact(prism_volume(&l, 6.0, &[(3, 3.0)])),
    );
    let all6: Vec<(usize, f64)> = (0..6).map(|i| (i, 1.0)).collect();
    add(
        "l-block-all-vertical-r1",
        "all 6 vertical edges of an L block, r = 1",
        &["analytic", "concave"],
        &lb,
        constant(ps.verticals(&lb, &l, 6.0, &[0, 1, 2, 3, 4, 5]), 1.0),
        d(),
        exact(prism_volume(&l, 6.0, &all6)),
    );
    let mut lmix = constant(ps.verticals(&lb, &l, 6.0, &[3]), 1.0);
    lmix.extend(constant(ps.top_edges(&lb, &l, 6.0)[2..4].to_vec(), 0.8));
    add(
        "l-block-concave-corner",
        "the concave vertical edge and the two top edges meeting it",
        &["analytic", "concave", "corner3"],
        &lb,
        lmix,
        d(),
        Expect::success(),
    );
    let hex = regular(6, 10.0);
    let hb = ps.prism(&hex, 8.0);
    let hex6: Vec<(usize, f64)> = (0..6).map(|i| (i, 1.0)).collect();
    add(
        "hex-prism-vertical-r1",
        "all 6 vertical edges of a hexagonal prism, r = 1",
        &["analytic", "convex", "smoke"],
        &hb,
        constant(ps.verticals(&hb, &hex, 8.0, &[0, 1, 2, 3, 4, 5]), 1.0),
        d(),
        exact(prism_volume(&hex, 8.0, &hex6)),
    );
    add(
        "hex-prism-top-loop-r1",
        "the top loop of a hexagonal prism (120° corners), r = 1",
        &["analytic", "convex", "loop"],
        &hb,
        constant(ps.top_edges(&hb, &hex, 8.0), 1.0),
        d(),
        exact(top_loop_volume(&hex, 8.0, 1.0)),
    );
    let tri = regular(3, 10.0 / 3f64.sqrt());
    let tb = ps.prism(&tri, 5.0);
    let tri3: Vec<(usize, f64)> = (0..3).map(|i| (i, 0.8)).collect();
    add(
        "triangle-prism-vertical-r0p8",
        "the 3 vertical edges of an equilateral prism (60°), r = 0.8",
        &["analytic", "convex", "acute"],
        &tb,
        constant(ps.verticals(&tb, &tri, 5.0, &[0, 1, 2]), 0.8),
        d(),
        exact(prism_volume(&tri, 5.0, &tri3)),
    );
    let wedge30 = vec![
        Point2::new(0.0, 0.0),
        Point2::new(20.0, 0.0),
        Point2::new(0.0, 20.0 * (PI / 6.0).tan()),
    ];
    let w30 = ps.prism(&wedge30, 5.0);
    add(
        "wedge-30deg-r0p5",
        "the 30° vertical edge of a wedge, r = 0.5",
        &["analytic", "convex", "acute"],
        &w30,
        constant(ps.verticals(&w30, &wedge30, 5.0, &[1]), 0.5),
        d(),
        exact(prism_volume(&wedge30, 5.0, &[(1, 0.5)])),
    );
    let wedge10 = vec![
        Point2::new(0.0, 0.0),
        Point2::new(40.0, 0.0),
        Point2::new(0.0, 40.0 * (PI / 18.0).tan()),
    ];
    let w10 = ps.prism(&wedge10, 5.0);
    add(
        "wedge-10deg-r0p3",
        "the 10° vertical edge of a sharp wedge, r = 0.3",
        &["analytic", "convex", "acute"],
        &w10,
        constant(ps.verticals(&w10, &wedge10, 5.0, &[1]), 0.3),
        d(),
        exact(prism_volume(&wedge10, 5.0, &[(1, 0.3)])),
    );
    let st = star(5, 10.0, 10.0 * (2.0 * PI / 5.0).cos() / (PI / 5.0).cos());
    let sb = ps.prism(&st, 4.0);
    let st10: Vec<(usize, f64)> = (0..10).map(|i| (i, 0.4)).collect();
    add(
        "star-prism-vertical-r0p4",
        "all 10 vertical edges of a pentagram prism (36° tips, 252° notches)",
        &["analytic", "concave", "acute", "smoke"],
        &sb,
        constant(
            ps.verticals(&sb, &st, 4.0, &(0..10).collect::<Vec<_>>()),
            0.4,
        ),
        d(),
        exact(prism_volume(&st, 4.0, &st10)),
    );
    let flat5 = vec![
        Point2::new(0.0, 0.0),
        Point2::new(10.0, -10.0 * (PI / 180.0).tan()),
        Point2::new(20.0, 0.0),
        Point2::new(20.0, 10.0),
        Point2::new(0.0, 10.0),
    ];
    let fb = ps.prism(&flat5, 5.0);
    add(
        "near-flat-vertex-r5",
        "a 178° vertical edge (nearly tangent faces), r = 5",
        &["analytic", "convex", "near_flat"],
        &fb,
        constant(ps.verticals(&fb, &flat5, 5.0, &[1]), 5.0),
        d(),
        exact(prism_volume(&flat5, 5.0, &[(1, 5.0)])),
    );
    add(
        "near-flat-vertex-r50",
        "a 178° vertical edge, r = 50 (a long shallow blend)",
        &["analytic", "convex", "near_flat"],
        &fb,
        constant(ps.verticals(&fb, &flat5, 5.0, &[1]), 50.0),
        d(),
        exact(prism_volume(&flat5, 5.0, &[(1, 50.0)])),
    );
    let slot = ps.slot(10.0, 3.0, 5.0);
    let slot_v =
        |r: f64| (2.0 * 3.0 * 10.0 + PI * 9.0) * 5.0 - s(r) * (2.0 * 10.0 + TAU * (3.0 - c(r)));
    add(
        "slot-top-chain-r1",
        "one top edge of a slot: the blend runs round the tangent chain",
        &["analytic", "convex", "chain", "smoke"],
        &slot,
        constant(vec![ps.at(&slot, p(5.0, -3.0, 5.0))], 1.0),
        d(),
        exact(slot_v(1.0)),
    );
    add(
        "slot-top-chain-r2p9",
        "one top edge of a slot, r = 2.9 (near the arc radius)",
        &["analytic", "convex", "chain", "near_limit"],
        &slot,
        constant(vec![ps.at(&slot, p(5.0, -3.0, 5.0))], 2.9),
        d(),
        exact(slot_v(2.9)),
    );

    // -- cylinders and cones --
    let (cr, ch) = (5.0, 10.0);
    let cyl = ps.cylinder(&Plane::XY, cr, ch);
    let rim_top = ps.at(&cyl, p(0.0, 0.0, ch));
    let rim_bottom = ps.at(&cyl, p(0.0, 0.0, 0.0));
    let rim = |r: f64| s(r) * TAU * (cr - c(r));
    let cyl_v = PI * cr * cr * ch;
    add(
        "cylinder-top-rim-r1",
        "the top rim of a cylinder, r = 1",
        &["analytic", "convex", "smoke"],
        &cyl,
        constant(vec![rim_top], 1.0),
        d(),
        exact(cyl_v - rim(1.0)),
    );
    add(
        "cylinder-both-rims-r1",
        "both rims of a cylinder, r = 1",
        &["analytic", "convex"],
        &cyl,
        constant(vec![rim_top, rim_bottom], 1.0),
        d(),
        exact(cyl_v - 2.0 * rim(1.0)),
    );
    add(
        "cylinder-rim-r4p9",
        "the top rim of a cylinder, r just below the radius",
        &["analytic", "convex", "near_limit"],
        &cyl,
        constant(vec![rim_top], 4.9),
        d(),
        exact(cyl_v - rim(4.9)),
    );
    add(
        "cylinder-rim-r5-dome",
        "the top rim of a cylinder, r = the radius: a hemispherical dome",
        &["analytic", "convex", "near_limit"],
        &cyl,
        constant(vec![rim_top], 5.0),
        d(),
        exact(cyl_v - rim(5.0)),
    );
    add(
        "cylinder-rim-r6-too-large",
        "the top rim of a cylinder, r above the radius",
        &["analytic", "radius_too_large"],
        &cyl,
        constant(vec![rim_top], 6.0),
        d(),
        too_large(5.0),
    );
    let cone = k
        .make_cone(&Plane::XY, 6.0, 3.0, 8.0, TAU)
        .expect("cone")
        .shape;
    add(
        "cone-base-rim-r1",
        "the base rim of a truncated cone (obtuse edge), r = 1",
        &["analytic", "convex"],
        &cone,
        constant(vec![ps.at(&cone, p(0.0, 0.0, 0.0))], 1.0),
        d(),
        Expect::success(),
    );
    add(
        "cone-top-rim-r0p5",
        "the top rim of a truncated cone (acute edge), r = 0.5",
        &["analytic", "convex"],
        &cone,
        constant(vec![ps.at(&cone, p(0.0, 0.0, 8.0))], 0.5),
        d(),
        Expect::success(),
    );

    // -- holes, bosses, pockets --
    let slab = ps.block(&Plane::XY, Vec3::new(40.0, 30.0, 5.0));
    let hole = |x: f64, y: f64, r: f64| {
        ps.cylinder(
            &Plane {
                origin: p(x, y, -1.0),
                ..Plane::XY
            },
            r,
            7.0,
        )
    };
    let holed = ps.boolean(BoolOp::Cut, &slab, &hole(20.0, 15.0, 4.0));
    let hole_rim = ps.at(&holed, p(20.0, 15.0, 5.0));
    let ring_out = |rad: f64, r: f64| s(r) * TAU * (rad + c(r));
    let ring_in = |rad: f64, r: f64| s(r) * TAU * (rad - c(r));
    let holed_v = 6000.0 - PI * 16.0 * 5.0;
    add(
        "plate-hole-rim-r0p5",
        "the top rim of a through hole, r = 0.5",
        &["analytic", "convex", "hole", "smoke"],
        &holed,
        constant(vec![hole_rim], 0.5),
        d(),
        exact(holed_v - ring_out(4.0, 0.5)),
    );
    add(
        "plate-hole-rim-r6-too-large",
        "the top rim of a hole in a 5 mm plate, r = 6",
        &["analytic", "radius_too_large", "hole"],
        &holed,
        constant(vec![hole_rim], 6.0),
        d(),
        too_large(5.0),
    );
    let two = ps.boolean(
        BoolOp::Cut,
        &ps.boolean(BoolOp::Cut, &slab, &hole(10.0, 15.0, 3.0)),
        &hole(30.0, 15.0, 3.0),
    );
    add(
        "plate-two-hole-rims-r0p5",
        "the top rims of two holes, r = 0.5",
        &["analytic", "convex", "hole"],
        &two,
        constant(
            vec![
                ps.at(&two, p(10.0, 15.0, 5.0)),
                ps.at(&two, p(30.0, 15.0, 5.0)),
            ],
            0.5,
        ),
        d(),
        exact(6000.0 - 2.0 * PI * 9.0 * 5.0 - 2.0 * ring_out(3.0, 0.5)),
    );
    let base = ps.block(&Plane::XY, Vec3::new(30.0, 30.0, 5.0));
    let boss = ps.boolean(
        BoolOp::Fuse,
        &base,
        &ps.cylinder(
            &Plane {
                origin: p(15.0, 15.0, 5.0),
                ..Plane::XY
            },
            5.0,
            6.0,
        ),
    );
    let boss_v = 4500.0 + PI * 25.0 * 6.0;
    let boss_base = ps.at(&boss, p(15.0, 15.0, 5.0));
    let boss_top = ps.at(&boss, p(15.0, 15.0, 11.0));
    add(
        "boss-base-concave-r1",
        "the concave base circle of a cylindrical boss, r = 1",
        &["analytic", "concave", "boss", "smoke"],
        &boss,
        constant(vec![boss_base], 1.0),
        d(),
        exact(boss_v + ring_out(5.0, 1.0)),
    );
    add(
        "boss-top-and-base-r1",
        "the base and top rims of a boss, r = 1",
        &["analytic", "concave", "boss"],
        &boss,
        constant(vec![boss_base, boss_top], 1.0),
        d(),
        exact(boss_v + ring_out(5.0, 1.0) - ring_in(5.0, 1.0)),
    );
    let block = ps.block(&Plane::XY, Vec3::new(30.0, 30.0, 10.0));
    let pocket = ps.boolean(
        BoolOp::Cut,
        &block,
        &ps.block(
            &Plane {
                origin: p(10.0, 10.0, 5.0),
                ..Plane::XY
            },
            Vec3::new(10.0, 10.0, 6.0),
        ),
    );
    let pv: Vec<EdgeId> = [(10.0, 10.0), (20.0, 10.0), (20.0, 20.0), (10.0, 20.0)]
        .into_iter()
        .map(|(x, y)| ps.at(&pocket, p(x, y, 7.5)))
        .collect();
    let floor: Vec<EdgeId> = [(15.0, 10.0), (20.0, 15.0), (15.0, 20.0), (10.0, 15.0)]
        .into_iter()
        .map(|(x, y)| ps.at(&pocket, p(x, y, 5.0)))
        .collect();
    let lip: Vec<EdgeId> = [(15.0, 10.0), (20.0, 15.0), (15.0, 20.0), (10.0, 15.0)]
        .into_iter()
        .map(|(x, y)| ps.at(&pocket, p(x, y, 10.0)))
        .collect();
    add(
        "pocket-vertical-edges-r1",
        "the 4 concave vertical edges of a pocket, r = 1",
        &["analytic", "concave", "pocket", "smoke"],
        &pocket,
        constant(pv.clone(), 1.0),
        d(),
        exact(9000.0 - 500.0 + 4.0 * 5.0 * s(1.0)),
    );
    add(
        "pocket-floor-loop-r1",
        "the 4 concave floor edges of a pocket, r = 1",
        &["analytic", "concave", "pocket", "loop"],
        &pocket,
        constant(floor.clone(), 1.0),
        d(),
        bounds(8500.0, 8500.0 + 40.0 * s(1.0) + 1.0),
    );
    add(
        "pocket-all-edges-r1",
        "floor, wall and lip edges of a pocket, r = 1",
        &["analytic", "concave", "pocket", "corner3"],
        &pocket,
        constant([pv, floor, lip].concat(), 1.0),
        d(),
        Expect::success(),
    );

    // -- intersection curves --
    let side = ps.boolean(
        BoolOp::Fuse,
        &ps.cube(20.0),
        &ps.cylinder(
            &Plane {
                origin: p(20.0, 10.0, 10.0),
                ..Plane::YZ
            },
            4.0,
            8.0,
        ),
    );
    add(
        "side-boss-intersection-r1",
        "the circle where a side boss meets a box wall, r = 1",
        &["analytic", "concave", "boss"],
        &side,
        constant(vec![ps.at(&side, p(20.0, 10.0, 10.0))], 1.0),
        d(),
        exact(8000.0 + PI * 16.0 * 8.0 + ring_out(4.0, 1.0)),
    );
    let pipe_a = ps.cylinder(&Plane::XY, 8.0, 30.0);
    let pipe_b = ps.cylinder(
        &Plane {
            origin: p(0.0, 0.0, 15.0),
            x: Dir3::Z,
            y: Dir3::X,
            normal: Dir3::Y,
        },
        4.0,
        15.0,
    );
    let tee = ps.boolean(BoolOp::Fuse, &pipe_a, &pipe_b);
    let seam_curve = ps.wher(&tee, |g| g.class == GeomClass::Curve);
    add(
        "tee-intersection-r1",
        "the space curve where two cylinders meet (a tee), r = 1",
        &["concave", "intersection", "smoke"],
        &tee,
        constant(seam_curve, 1.0),
        d(),
        Expect::success(),
    );
    let on_box = ps.boolean(
        BoolOp::Fuse,
        &ps.cube(20.0),
        &k.make_sphere(
            &Plane {
                origin: p(10.0, 10.0, 23.0),
                ..Plane::XY
            },
            5.0,
        )
        .expect("sphere")
        .shape,
    );
    let ring = ps.wher(&on_box, |g| {
        g.class == GeomClass::Circle
            && (g.bbox.min.z - 20.0).abs() < 1e-6
            && (g.bbox.max.z - 20.0).abs() < 1e-6
    });
    add(
        "sphere-on-box-r1",
        "the circle where a sphere meets a box top (obtuse concave edge), r = 1",
        &["concave", "intersection"],
        &on_box,
        constant(ring, 1.0),
        d(),
        Expect::success(),
    );
    let chamfered = k
        .chamfer(&cube, &[(front, ChamferSpec::Equal(2.0))], &ps.ctx)
        .expect("chamfer")
        .shape;
    let corner_edge = ps.wher(&chamfered, |g| {
        g.class == GeomClass::Line && g.bbox.max.x.abs() < 1e-6 && g.bbox.max.y.abs() < 1e-6
    });
    add(
        "fillet-after-chamfer-r1",
        "a vertical edge ending on a chamfer face, r = 1",
        &["convex", "after_chamfer"],
        &chamfered,
        constant(corner_edge, 1.0),
        d(),
        Expect::success(),
    );
    out
}

fn write_case(dir: &Path, h: &Hand) {
    let case = dir.join(h.name);
    let inputs = case.join("inputs");
    fs::create_dir_all(&inputs).expect("case dir");
    let mut file = fs::File::create(inputs.join("0.brep")).expect("brep file");
    h.input.write_text(&mut file).expect("brep");
    let op = KernelOp::Fillet {
        shape: InputRef(0),
        edges: h
            .edges
            .iter()
            .map(|(e, spec)| (EdgeRecord::of(&h.input, *e), spec.clone()))
            .collect(),
        opts: h.opts.clone(),
    };
    fs::write(
        case.join("op.json"),
        serde_json::to_string_pretty(&op).expect("op json") + "\n",
    )
    .expect("op.json");
    let mut tags = vec!["handmade".to_owned()];
    tags.extend(h.tags.iter().map(|t| (*t).to_owned()));
    let toml = CaseToml {
        schema: CASE_SCHEMA,
        id: format!("fillet/handmade/{}", h.name),
        op: "op.json".to_owned(),
        source: "handmade".to_owned(),
        licence: "LicenseRef-Amalgam-Personal".to_owned(),
        tags,
        description: Some(h.description.to_owned()),
        recorded_class: None,
        kernel_version: None,
        expect: h.expect.clone(),
    };
    fs::write(
        case.join("case.toml"),
        format!(
            "# Written by `cargo run -p amg-brep --features occt --example gen_fillet_corpus`.\n{}",
            toml::to_string(&toml).expect("case toml")
        ),
    )
    .expect("case.toml");
}

fn main() {
    let dir = std::env::args()
        .nth(1)
        .map_or_else(|| fillet_corpus_dir().join("handmade"), PathBuf::from);
    let k = OcctKernel::new(KernelConfig::default());
    let cases = cases(&k);
    let mut names: Vec<&str> = cases.iter().map(|c| c.name).collect();
    names.sort_unstable();
    names.dedup();
    assert_eq!(names.len(), cases.len(), "case names must be unique");
    for h in &cases {
        write_case(&dir, h);
    }
    let smoke = cases.iter().filter(|c| c.tags.contains(&"smoke")).count();
    eprintln!(
        "wrote {} handmade cases ({smoke} smoke) to {}",
        cases.len(),
        dir.display()
    );
}
