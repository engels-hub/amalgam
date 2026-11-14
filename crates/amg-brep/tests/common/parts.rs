//! Perforated plates for the BREP-04 boolean budgets and the cancel test: a square slab with an
//! `n × n` grid of round through holes has `n² + 6` faces (`n = 14`: 202, `n = 32`: 1030).
#![allow(dead_code, clippy::unwrap_used, reason = "shared test helper")]

use amg_brep::geom::{Curve2d, Loop2, PlanarProfile, ProfileEdge2, Tag};
use amg_brep::{BrepKernel, JobCtx, OcctKernel, PrismExtent, PrismOpts, Shape};
use amg_core::{Plane, Point2, Point3, Tolerance, Vec3};

/// Hole pitch, mm.
pub const PITCH: f64 = 4.0;
/// Hole radius, mm.
pub const RADIUS: f64 = 1.2;
/// Slab thickness, mm.
pub const THICKNESS: f64 = 10.0;

fn edge(curve: Curve2d, tag: u64) -> ProfileEdge2 {
    ProfileEdge2 {
        curve,
        tag: Tag(tag),
    }
}

/// A plate with `n × n` holes, its lower corner at `origin`.
pub fn plate(k: &OcctKernel, n: u32, origin: Point3) -> Shape {
    let side = PITCH * f64::from(n);
    let corners =
        [(0.0, 0.0), (side, 0.0), (side, side), (0.0, side)].map(|(x, y)| Point2::new(x, y));
    let outer = Loop2 {
        edges: (0..4)
            .map(|i| {
                edge(
                    Curve2d::Line {
                        a: corners[i],
                        b: corners[(i + 1) % 4],
                    },
                    i as u64,
                )
            })
            .collect(),
    };
    let holes = (0..n)
        .flat_map(|i| (0..n).map(move |j| (i, j)))
        .map(|(i, j)| Loop2 {
            edges: vec![edge(
                Curve2d::Circle {
                    center: Point2::new(PITCH * (f64::from(i) + 0.5), PITCH * (f64::from(j) + 0.5)),
                    radius: RADIUS,
                },
                u64::from(4 + i * n + j),
            )],
        })
        .collect();
    let profile = PlanarProfile {
        plane: Plane {
            origin,
            ..Plane::XY
        },
        outer,
        holes,
    };
    let face = k
        .make_planar_faces(&[profile], &Tolerance::DEFAULT)
        .unwrap()
        .remove(0)
        .face;
    k.prism(
        &face,
        &PrismExtent::Vector(Vec3::new(0.0, 0.0, THICKNESS)),
        &PrismOpts {
            taper: 0.0,
            merge_coplanar: false,
        },
        &JobCtx::detached(),
    )
    .unwrap()
    .shape
}

/// Two plates of `n × n` holes for a cut: the tool is shifted by half a pitch in x and y (its
/// holes fall between the object's) and by half the thickness in z, so every face of both meets
/// the other part.
pub fn plate_pair(k: &OcctKernel, n: u32) -> (Shape, Shape) {
    let object = plate(k, n, Point3::ZERO);
    let tool = plate(k, n, Point3::new(PITCH / 2.0, PITCH / 2.0, THICKNESS / 2.0));
    (object, tool)
}
