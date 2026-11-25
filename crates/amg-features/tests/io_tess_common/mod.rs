//! IO-09: the golden solids and a document that holds them. Shared by `io_tessellate.rs` and
//! `benches/io_tessellate.rs`.
#![allow(
    dead_code,
    clippy::unwrap_used,
    clippy::expect_used,
    reason = "test helper"
)]

use std::f64::consts::TAU;
use std::sync::OnceLock;

use amg_brep::geom::{Curve2d, Loop2, PlanarProfile, ProfileEdge2, Tag};
use amg_brep::{
    BoolOp, BoolOpts, BrepKernel, KernelConfig, OcctKernel, PrismExtent, PrismOpts, Shape,
};
use amg_core::{Dir3, Plane, Point2, Point3, Tolerance, Vec2, Vec3};
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, Command, ComponentId, CreatedId, Dependency,
    Document, Feature, FeatureBox, FeatureError, FeatureId, FeatureOutput, RecomputeCtx,
    register_feature,
};
use amg_jobs::{JobCtx, JobSystem, JobSystemConfig};
use serde::{Deserialize, Serialize};

/// Creates solid body slot 0 from `.brep` text.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Io09Solid {
    pub brep: String,
}

impl Feature for Io09Solid {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let shape = Shape::read_text(self.brep.as_bytes())
            .map_err(|e| FeatureError::Kernel(e.to_string()))?;
        Ok(FeatureOutput {
            bodies: vec![BodyChange {
                id: ctx.new_body_id(0),
                change: BodyChangeKind::Created {
                    geom: BodyGeom::Solid(amg_doc::stub::Shape::from_brep(shape)),
                    naming: None,
                },
            }],
            ..FeatureOutput::default()
        })
    }
}

register_feature!(Io09Solid, "test.io09_solid", version = 1);

pub fn jobs() -> &'static JobSystem {
    static JOBS: OnceLock<JobSystem> = OnceLock::new();
    JOBS.get_or_init(|| JobSystem::new(JobSystemConfig::default()).unwrap())
}

pub fn kernel() -> OcctKernel {
    OcctKernel::new(KernelConfig::default())
}

pub fn brep_text(shape: &Shape) -> String {
    let mut out = Vec::new();
    shape.write_text(&mut out).unwrap();
    String::from_utf8(out).unwrap()
}

/// A `testdata/brep` fixture (`None`: un-fetched LFS pointer).
pub fn fixture(name: &str) -> Option<String> {
    let path = amg_testkit::testdata(&format!("brep/{name}.brep"))?;
    Some(std::fs::read_to_string(path).unwrap())
}

fn frame(origin: Point3, normal: Vec3, x: Vec3) -> Plane {
    Plane {
        origin,
        x: Dir3::new_unchecked(x),
        y: Dir3::new_unchecked(normal.cross(x)),
        normal: Dir3::new_unchecked(normal),
    }
}

fn circle(center: Point2, radius: f64, tag: u64) -> Loop2 {
    Loop2 {
        edges: vec![ProfileEdge2 {
            curve: Curve2d::Circle { center, radius },
            tag: Tag(tag),
        }],
    }
}

fn polygon(points: &[Point2]) -> Loop2 {
    Loop2 {
        edges: (0..points.len())
            .map(|i| ProfileEdge2 {
                curve: Curve2d::Line {
                    a: points[i],
                    b: points[(i + 1) % points.len()],
                },
                tag: Tag(i as u64),
            })
            .collect(),
    }
}

fn extrude(outer: Loop2, holes: Vec<Loop2>, height: f64) -> Shape {
    let k = kernel();
    let profile = PlanarProfile {
        plane: Plane::XY,
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

/// A 24 mm square slab, 6 mm thick, with 4 × 4 round through holes.
pub fn perforated_plate() -> Shape {
    let side = 24.0;
    let holes = (0u32..4)
        .flat_map(|i| (0..4).map(move |j| (i, j)))
        .map(|(i, j)| {
            let c = Point2::new(6.0 * f64::from(i) + 3.0, 6.0 * f64::from(j) + 3.0);
            circle(c, 1.8, u64::from(4 + i * 4 + j))
        })
        .collect();
    let corners = [(0.0, 0.0), (side, 0.0), (side, side), (0.0, side)];
    extrude(
        polygon(&corners.map(|(x, y)| Point2::new(x, y))),
        holes,
        6.0,
    )
}

/// An ordinary 1000-face part: a 998-tooth-flank star wheel extruded to one solid (998 side
/// faces and two caps).
pub fn star_wheel_1000() -> Shape {
    let n = 998u32;
    let points: Vec<Point2> = (0..n)
        .map(|i| {
            let angle = TAU * f64::from(i) / f64::from(n);
            let r = if i % 2 == 0 { 100.0 } else { 98.0 };
            Point2::new(r * angle.cos(), r * angle.sin())
        })
        .collect();
    extrude(polygon(&points), Vec::new(), 10.0)
}

/// A curved 1000-face part: a wheel of 998 side faces, planes and cylinders alternating
/// (every second flank is an arc bulging outwards), and two caps.
pub fn scalloped_wheel_1000() -> Shape {
    let n = 998u32;
    let step = TAU / f64::from(n);
    let at = |i: u32, r: f64| {
        let angle = step * f64::from(i);
        Point2::new(r * angle.cos(), r * angle.sin())
    };
    // Pairs of vertices on two radii: a straight flank, then an arc between equal radii.
    let radius = |i: u32| {
        if (i / 2).is_multiple_of(2) {
            100.0
        } else {
            99.0
        }
    };
    let edges = (0..n)
        .map(|i| {
            let (a, b) = (at(i, radius(i)), at((i + 1) % n, radius((i + 1) % n)));
            let curve = if i.is_multiple_of(2) {
                // The circle through `a` and `b` with a sagitta of 0.15 mm, outwards.
                let chord = b - a;
                let (half, sag) = (chord.length() / 2.0, 0.15);
                let r = (half * half + sag * sag) / (2.0 * sag);
                let out = Vec2::new(chord.y, -chord.x).normalize();
                let center = (a + b) / 2.0 - out * (r - sag);
                let start = (a - center).to_angle();
                let sweep = (a - center).angle_to(b - center);
                Curve2d::Arc {
                    center,
                    radius: r,
                    start_angle: start,
                    sweep,
                }
            } else {
                Curve2d::Line { a, b }
            };
            ProfileEdge2 {
                curve,
                tag: Tag(u64::from(i)),
            }
        })
        .collect();
    extrude(Loop2 { edges }, Vec::new(), 10.0)
}

/// 38 filleted boxes in one compound: 988 faces, a third of them curved.
pub fn filleted_boxes_988() -> Option<Vec<String>> {
    let one = fixture("filleted_box")?;
    let shape = Shape::read_text(one.as_bytes()).unwrap();
    Some(
        (0..38)
            .map(|i| {
                let at = Vec3::new(f64::from(i % 7) * 60.0, f64::from(i / 7) * 60.0, 0.0);
                let moved = shape
                    .transformed(&amg_core::Transform::from_translation(at))
                    .unwrap();
                brep_text(&moved)
            })
            .collect(),
    )
}

fn boolean(op: BoolOp, a: Shape, b: Shape) -> Shape {
    kernel()
        .boolean(op, &[a], &[b], &BoolOpts::default(), &JobCtx::detached())
        .unwrap()
        .shape
}

/// A box with a blind hole (plane and cylinder faces, one boolean).
pub fn blind_hole() -> Shape {
    let k = kernel();
    let block = k
        .make_box(&Plane::XY, Vec3::new(20.0, 20.0, 10.0))
        .unwrap()
        .shape;
    let at = frame(Point3::new(10.0, 10.0, 4.0), Vec3::Z, Vec3::X);
    let hole = k.make_cylinder(&at, 5.0, 10.0, TAU).unwrap().shape;
    boolean(BoolOp::Cut, block, hole)
}

/// The common of two crossing cylinders (the Steinmetz solid).
pub fn steinmetz() -> Shape {
    let k = kernel();
    let z = frame(Point3::new(0.0, 0.0, -15.0), Vec3::Z, Vec3::X);
    let x = frame(Point3::new(-15.0, 0.0, 0.0), Vec3::X, Vec3::Y);
    let a = k.make_cylinder(&z, 6.0, 30.0, TAU).unwrap().shape;
    let b = k.make_cylinder(&x, 6.0, 30.0, TAU).unwrap().shape;
    boolean(BoolOp::Common, a, b)
}

/// The closed `testdata/brep` fixtures used as golden solids (the 1 030-face
/// `plate_with_holes` is separate: it takes seconds to mesh until BREP-18).
pub const SOLID_FIXTURES: [&str; 8] = [
    "box",
    "cylinder",
    "cone",
    "sphere",
    "torus",
    "filleted_box",
    "located_compound",
    "cylinder_mirrored",
];

/// The golden solids of the S02 corpus: name and `.brep` text. Empty when the fixtures are
/// un-fetched LFS pointers.
pub fn golden_solids() -> Vec<(&'static str, String)> {
    let mut out = Vec::new();
    for name in SOLID_FIXTURES {
        let Some(text) = fixture(name) else {
            return Vec::new();
        };
        out.push((name, text));
    }
    out.push(("perforated_plate", brep_text(&perforated_plate())));
    out.push(("blind_hole", brep_text(&blind_hole())));
    out.push(("steinmetz", brep_text(&steinmetz())));
    out
}

pub fn add_feature(doc: &mut Document, f: FeatureBox, context: ComponentId) -> FeatureId {
    let out = doc
        .apply(Command::AddFeature {
            at: None,
            feature: f,
            context,
            name: None,
        })
        .unwrap();
    out.created
        .iter()
        .find_map(|c| match c {
            CreatedId::Feature(f) => Some(*f),
            _ => None,
        })
        .unwrap()
}

pub fn add_solid(doc: &mut Document, brep: String) -> BodyId {
    let root = doc.design().root();
    let f = add_feature(doc, FeatureBox::new(Io09Solid { brep }), root);
    BodyId { origin: f, slot: 0 }
}

pub fn recompute(doc: &mut Document) {
    doc.recompute_blocking(jobs()).unwrap();
}

/// A recomputed document with one solid body per shape text.
pub fn document(breps: &[String]) -> (Document, Vec<BodyId>) {
    let mut doc = Document::new("io09");
    let bodies = breps
        .iter()
        .map(|b| add_solid(&mut doc, b.clone()))
        .collect();
    recompute(&mut doc);
    (doc, bodies)
}
