//! The S20 KH-20 shapes, at kernel level: the inputs of the S04 budget benches
//! `budget/sol-extrude-cut` (a 198-gon prism, 200 faces, and the 100-gon prism that cuts it) and
//! `budget/sol-shell-preview` (the 98-vertex star prism, 100 faces), plus the SOL-06 probe's
//! 120 × 120 × 10 plate with 94 holes (100 faces). Shared by `tests/kernel_speed.rs` and
//! `benches/kernel_speed.rs`.
#![allow(dead_code, clippy::unwrap_used, reason = "shared test helper")]

use std::f64::consts::TAU;

use amg_brep::geom::{Curve2d, Loop2, PlanarProfile, ProfileEdge2, Tag};
use amg_brep::{BrepKernel, FaceId, JobCtx, PrismExtent, PrismOpts, Shape, SubKind, SubShape};
use amg_core::{Plane, Point2, Point3, Tolerance, Vec3};

fn line_loop(points: &[Point2], first_tag: u64) -> Loop2 {
    Loop2 {
        edges: (0..points.len())
            .map(|i| ProfileEdge2 {
                curve: Curve2d::Line {
                    a: points[i],
                    b: points[(i + 1) % points.len()],
                },
                tag: Tag(first_tag + i as u64),
            })
            .collect(),
    }
}

/// The prism of `outer` (with round `holes` `(centre, radius)`) on the XY plane, `height` up.
pub fn prism_of(
    k: &dyn BrepKernel,
    outer: &[Point2],
    holes: &[(Point2, f64)],
    height: f64,
) -> Shape {
    let n = outer.len() as u64;
    let profile = PlanarProfile {
        plane: Plane::XY,
        outer: line_loop(outer, 0),
        holes: holes
            .iter()
            .enumerate()
            .map(|(i, &(center, radius))| Loop2 {
                edges: vec![ProfileEdge2 {
                    curve: Curve2d::Circle { center, radius },
                    tag: Tag(n + i as u64),
                }],
            })
            .collect(),
    };
    let face = k
        .make_planar_faces(&[profile], &Tolerance::DEFAULT)
        .unwrap()
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
    .unwrap()
    .shape
}

/// A regular `sides`-gon of circumradius `r` about `center`, first vertex at angle 0.
pub fn polygon(center: Point2, r: f64, sides: u32) -> Vec<Point2> {
    (0..sides)
        .map(|i| {
            let a = f64::from(i) * TAU / f64::from(sides);
            Point2::new(center.x + r * a.cos(), center.y + r * a.sin())
        })
        .collect()
}

/// `budget/sol-extrude-cut`: the 198-gon prism (r 50, 20 mm, 200 faces) and the 100-gon tool
/// (r 20 about (5, 3), 30 mm) that cuts through it from its bottom plane.
pub fn extrude_cut_pair(k: &dyn BrepKernel) -> (Shape, Shape) {
    let body = prism_of(k, &polygon(Point2::ZERO, 50.0, 198), &[], 20.0);
    let tool = prism_of(k, &polygon(Point2::new(5.0, 3.0), 20.0, 100), &[], 30.0);
    (body, tool)
}

/// `budget/sol-shell-preview`: the prism of a 98-vertex star (radii 50 / 46, 20 mm, 100 faces).
pub fn star_prism(k: &dyn BrepKernel) -> Shape {
    let pts: Vec<Point2> = (0..98)
        .map(|i| {
            let a = f64::from(i) * TAU / 98.0;
            let r = if i % 2 == 0 { 50.0 } else { 46.0 };
            Point2::new(r * a.cos(), r * a.sin())
        })
        .collect();
    prism_of(k, &pts, &[], 20.0)
}

/// The SOL-06 probe's plate: 120 × 120 × 10 with 94 through holes of r 2 (100 faces).
pub fn holed_plate(k: &dyn BrepKernel) -> Shape {
    let outer =
        [(0.0, 0.0), (120.0, 0.0), (120.0, 120.0), (0.0, 120.0)].map(|(x, y)| Point2::new(x, y));
    let holes: Vec<(Point2, f64)> = (0..10)
        .flat_map(|i| (0..10).map(move |j| (i, j)))
        .take(94)
        .map(|(i, j)| {
            (
                Point2::new(15.0 + 10.0 * f64::from(i), 15.0 + 10.0 * f64::from(j)),
                2.0,
            )
        })
        .collect();
    prism_of(k, &outer, &holes, 10.0)
}

/// The face of `s` whose centroid is at height `z` and that is the largest there (a cap).
pub fn cap_at(k: &dyn BrepKernel, s: &Shape, z: f64) -> FaceId {
    s.faces()
        .filter_map(|f| {
            let g = k
                .sub_geom(
                    s,
                    SubShape {
                        kind: SubKind::Face,
                        index: f.0,
                    },
                )
                .unwrap();
            ((g.centroid.z - z).abs() < 1e-9).then_some((f, g.measure))
        })
        .max_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(f, _)| f)
        .unwrap()
}

/// The face of `s` whose centroid is closest to `at`.
pub fn face_near(k: &dyn BrepKernel, s: &Shape, at: Point3) -> FaceId {
    s.faces()
        .map(|f| {
            let g = k
                .sub_geom(
                    s,
                    SubShape {
                        kind: SubKind::Face,
                        index: f.0,
                    },
                )
                .unwrap();
            (f, (g.centroid - at).length())
        })
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(f, _)| f)
        .unwrap()
}
