//! Fillet corpus generators (KH-02, design note §5.3).
//!
//! Every generator is a pure function of `(generator, seed)`: [`FilletGenCase::part`] draws the
//! part and the fillet request without a kernel, so tags and the analytic expectation are known
//! when the corpus is loaded, and [`FilletGenCase::build`] builds it on a kernel.
//!
//! | generator | part | oracle |
//! |---|---|---|
//! | `prism_vertical_edges` | extruded random star-shaped polygon (3–12 vertices), a random subset of its vertical edges filleted below the setback limit | exact volume and face count |
//! | `prism_mixed_edges` | the same prism with top-face edges filleted too (corner blends) | volume bounds |
//! | `boolean_stack` | a box with 1–3 bosses, holes and pockets (fuses and cuts of boxes and cylinders), random edges | volume bounds |
//! | `radius_too_large` | a prism vertical edge at a convex vertex with `r` = 1.3–3 × its setback limit | `clean_failure`, `RadiusTooLarge`, `max_radius` = the limit |
//!
//! **Exact volume** (`prism_vertical_edges`). A fillet of radius `r` on a vertical edge whose two
//! side faces enclose the angle `a` on the side the ball rolls in (the interior angle `α` at a
//! convex vertex, `2π − α` at a concave one) is tangent to both faces at the setback
//! `t = r·cot(a/2)` from the vertex and cuts off (convex) or fills in (concave) the planar area
//! `f(a) = r²·(cot(a/2) − (π − a)/2)` of the profile. With setbacks below 45 % of each adjacent
//! edge no two fillets meet, so `V = h·(A − Σ_convex f + Σ_concave f)` and the solid has
//! `n + 2 + k` faces for `n` vertices and `k` fillets.

use std::f64::consts::{PI, TAU};

use amg_core::{Plane, Point2, Point3, Tolerance, Vec3};
use amg_jobs::JobCtx;

use super::case::{Expect, ExpectOutcome};
use super::{int, splitmix, uniform};
use crate::capture::{FailureClass, SubShapeGeom};
use crate::error::{KResult, KernelError};
use crate::kernel::BrepKernel;
use crate::opts::{BoolOp, BoolOpts, FilletOpts, FilletSpec, PrismExtent, PrismOpts};
use crate::shape::{EdgeId, Shape, SubKind, SubShape};

/// The fillet generators.
#[derive(
    Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize,
)]
#[serde(rename_all = "snake_case")]
pub enum FilletGenerator {
    /// Vertical edges of an extruded polygon, exact volume oracle.
    PrismVerticalEdges,
    /// Vertical and top edges of an extruded polygon (corner blends), volume bounds.
    PrismMixedEdges,
    /// Edges of a box with bosses, holes and pockets, volume bounds.
    BooleanStack,
    /// One vertical prism edge with a radius above its setback limit: a clean failure.
    RadiusTooLarge,
}

impl FilletGenerator {
    /// Every generator.
    pub const ALL: [FilletGenerator; 4] = [
        FilletGenerator::PrismVerticalEdges,
        FilletGenerator::PrismMixedEdges,
        FilletGenerator::BooleanStack,
        FilletGenerator::RadiusTooLarge,
    ];

    /// The `seeds.toml` name.
    pub fn name(self) -> &'static str {
        match self {
            FilletGenerator::PrismVerticalEdges => "prism_vertical_edges",
            FilletGenerator::PrismMixedEdges => "prism_mixed_edges",
            FilletGenerator::BooleanStack => "boolean_stack",
            FilletGenerator::RadiusTooLarge => "radius_too_large",
        }
    }

    /// The generator called `name`.
    pub fn from_name(name: &str) -> Option<FilletGenerator> {
        FilletGenerator::ALL.into_iter().find(|g| g.name() == name)
    }
}

/// One synthetic case: a generator and a seed.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub struct FilletGenCase {
    /// The generator.
    pub generator: FilletGenerator,
    /// The seed.
    pub seed: u64,
}

/// An extruded polygon with fillets on some of its vertical and top edges.
#[derive(Clone, Debug, PartialEq)]
pub struct PrismPart {
    /// The profile in `z = 0`, counter-clockwise, mm.
    pub polygon: Vec<Point2>,
    /// The extrusion height along +z, mm.
    pub height: f64,
    /// `(vertex index, radius)` of the filleted vertical edges.
    pub vertical: Vec<(usize, f64)>,
    /// `(j, radius)` of the filleted top edges, edge `j` running from vertex `j` to `j + 1`.
    pub top: Vec<(usize, f64)>,
}

/// A feature of a [`StackPart`], on the box's top face.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Feature {
    /// A box fused on top: `min` corner in the top face, `size` in x and y, `height`.
    BossBox {
        /// Minimum corner, mm.
        min: Point2,
        /// Size in x and y, mm.
        size: Point2,
        /// Height above the top face, mm.
        height: f64,
    },
    /// A cylinder fused on top.
    BossCylinder {
        /// Axis position, mm.
        centre: Point2,
        /// Radius, mm.
        radius: f64,
        /// Height above the top face, mm.
        height: f64,
    },
    /// A through hole.
    Hole {
        /// Axis position, mm.
        centre: Point2,
        /// Radius, mm.
        radius: f64,
    },
    /// A blind hole from the top face.
    BlindHole {
        /// Axis position, mm.
        centre: Point2,
        /// Radius, mm.
        radius: f64,
        /// Depth, mm.
        depth: f64,
    },
    /// A rectangular pocket from the top face.
    Pocket {
        /// Minimum corner, mm.
        min: Point2,
        /// Size in x and y, mm.
        size: Point2,
        /// Depth, mm.
        depth: f64,
    },
}

impl Feature {
    /// The material the feature adds (positive) or removes (negative), mm³.
    pub fn volume(&self, box_height: f64) -> f64 {
        match *self {
            Feature::BossBox { size, height, .. } => size.x * size.y * height,
            Feature::BossCylinder { radius, height, .. } => PI * radius * radius * height,
            Feature::Hole { radius, .. } => -PI * radius * radius * box_height,
            Feature::BlindHole { radius, depth, .. } => -PI * radius * radius * depth,
            Feature::Pocket { size, depth, .. } => -size.x * size.y * depth,
        }
    }

    /// A short tag naming the feature kind.
    pub fn tag(&self) -> &'static str {
        match self {
            Feature::BossBox { .. } | Feature::BossCylinder { .. } => "boss",
            Feature::Hole { .. } | Feature::BlindHole { .. } => "hole",
            Feature::Pocket { .. } => "pocket",
        }
    }
}

/// A box `[0, size]` with features in distinct cells of a 2 × 2 grid on its top face, and
/// `edges` random edges of the result filleted with `radius`.
#[derive(Clone, Debug, PartialEq)]
pub struct StackPart {
    /// Box size, mm.
    pub size: Vec3,
    /// The features, applied in order.
    pub features: Vec<Feature>,
    /// How many edges to fillet.
    pub edges: usize,
    /// The radius of every filleted edge, mm.
    pub radius: f64,
    /// The seed of the edge draw on the built part.
    pub edge_seed: u64,
}

impl StackPart {
    /// The volume of the unfilleted part, mm³ (features do not overlap).
    pub fn volume(&self) -> f64 {
        let s = self.size;
        s.x * s.y * s.z + self.features.iter().map(|f| f.volume(s.z)).sum::<f64>()
    }
}

/// What a generator drew.
#[derive(Clone, Debug, PartialEq)]
pub enum GenPart {
    /// A prism generator's part.
    Prism(PrismPart),
    /// A `boolean_stack` part.
    Stack(StackPart),
}

/// A generated case built on a kernel.
#[derive(Clone, Debug)]
pub struct Built {
    /// The input solid.
    pub input: Shape,
    /// The fillet request on it.
    pub edges: Vec<(EdgeId, FilletSpec)>,
    /// The fillet options.
    pub opts: FilletOpts,
    /// The expectation, completed with what needs the built part (the volume bounds of
    /// `boolean_stack`).
    pub expect: Expect,
}

/// Interior angles below this, or within this of 2π, are too sharp for the corpus, rad.
const MIN_ANGLE: f64 = 20.0 * PI / 180.0;
/// Interior angles within this of π are too flat (the vertex barely exists), rad.
const MIN_BEND: f64 = 6.0 * PI / 180.0;
/// The shortest polygon edge, mm.
const MIN_EDGE: f64 = 1.5;
/// Setbacks stay below this fraction of each adjacent edge, so no two fillets meet.
const SETBACK_FRACTION: f64 = 0.45;
/// The largest vertical fillet radius drawn, mm.
const MAX_VERTICAL_RADIUS: f64 = 6.0;
/// Relative slack of the volume oracle (OCCT mass properties integrate to 1e-12; the fillet
/// surfaces of planar faces are exact cylinders).
pub const EXACT_VOLUME_REL_TOL: f64 = 1e-6;

/// `f(a) = r²·(cot(a/2) − (π − a)/2)`: the profile area a fillet of radius `r` cuts off a
/// corner of angle `a` ∈ (0, π) (see the module docs).
pub fn corner_area(r: f64, a: f64) -> f64 {
    r * r * (1.0 / (0.5 * a).tan() - 0.5 * (PI - a))
}

impl PrismPart {
    fn n(&self) -> usize {
        self.polygon.len()
    }

    fn at(&self, i: usize) -> Point2 {
        self.polygon[i % self.n()]
    }

    /// The length of edge `j` (vertex `j` to `j + 1`), mm.
    pub fn edge_len(&self, j: usize) -> f64 {
        self.at(j).distance(self.at(j + 1))
    }

    /// The profile area (shoelace), mm².
    pub fn area(&self) -> f64 {
        let n = self.n();
        0.5 * (0..n)
            .map(|i| {
                let (p, q) = (self.at(i), self.at(i + 1));
                p.x * q.y - q.x * p.y
            })
            .sum::<f64>()
    }

    /// The interior angle at vertex `i`, rad in (0, 2π).
    pub fn interior_angle(&self, i: usize) -> f64 {
        let n = self.n();
        let (prev, p, next) = (self.at(i + n - 1), self.at(i), self.at(i + 1));
        let (d0, d1) = (p - prev, next - p);
        let turn = d0.perp_dot(d1).atan2(d0.dot(d1));
        PI - turn
    }

    /// The angle `a` the fillet of vertical edge `i` rolls in and whether the vertex is convex.
    pub fn blend_angle(&self, i: usize) -> (f64, bool) {
        let alpha = self.interior_angle(i);
        if alpha < PI {
            (alpha, true)
        } else {
            (TAU - alpha, false)
        }
    }

    /// The radius at which a fillet of vertical edge `i` alone reaches the nearer neighbour
    /// vertex (its setback equals the shorter adjacent edge), mm: the analytic max radius.
    pub fn setback_limit(&self, i: usize) -> f64 {
        let n = self.n();
        let (a, _) = self.blend_angle(i);
        self.edge_len(i + n - 1).min(self.edge_len(i)) * (0.5 * a).tan()
    }

    /// The exact solid volume with only the vertical fillets, mm³.
    pub fn vertical_volume(&self) -> f64 {
        let changed: f64 = self
            .vertical
            .iter()
            .map(|&(i, r)| {
                let (a, convex) = self.blend_angle(i);
                let f = corner_area(r, a);
                if convex { -f } else { f }
            })
            .sum();
        self.height * (self.area() + changed)
    }

    /// Volume bounds with the top fillets too: each top fillet of edge `j` removes at most the
    /// right-angle corner area times the edge length plus `2r` (corner overlaps), mm³.
    pub fn volume_bounds(&self) -> (f64, f64) {
        let v = self.vertical_volume();
        let removed: f64 = self
            .top
            .iter()
            .map(|&(j, r)| corner_area(r, 0.5 * PI) * (self.edge_len(j) + 2.0 * r))
            .sum();
        let grown: f64 = self
            .top
            .iter()
            .map(|&(_, r)| corner_area(r, 0.5 * PI) * 2.0 * r)
            .sum();
        let slack = 1e-6 * v.abs();
        (v - removed - slack, v + grown + slack)
    }

    fn tags(&self, generator: FilletGenerator) -> Vec<String> {
        let mut tags = vec![
            generator.name().to_owned(),
            "synthetic".to_owned(),
            "analytic".to_owned(),
        ];
        let concave = self.vertical.iter().any(|&(i, _)| !self.blend_angle(i).1);
        tags.push(if concave { "concave" } else { "convex" }.to_owned());
        let top = |j: usize| self.top.iter().any(|&(t, _)| t == j % self.n());
        let n = self.n();
        if self.vertical.iter().any(|&(i, _)| top(i + n - 1) && top(i)) {
            tags.push("corner3".to_owned());
        }
        if self.top.len() > 1 && (0..n).any(|j| top(j) && top(j + 1)) {
            tags.push("chain".to_owned());
        }
        if self
            .vertical
            .iter()
            .any(|&(i, _)| self.interior_angle(i) < PI / 4.0)
        {
            tags.push("acute".to_owned());
        }
        tags
    }

    fn profile(&self) -> Vec<Point3> {
        self.polygon
            .iter()
            .map(|p| Point3::new(p.x, p.y, 0.0))
            .collect()
    }
}

/// A random star-shaped polygon: 3–12 vertices at sorted angles about the origin, every angular
/// gap below 0.9π (so the polygon is simple and contains the origin), interior angles in
/// `[20°, 340°]` and at least 6° from flat, edges at least 1.5 mm.
fn polygon(state: &mut u64) -> Vec<Point2> {
    loop {
        let n = int(state, 3, 12);
        let weights: Vec<f64> = (0..n).map(|_| uniform(state, 0.4, 1.0)).collect();
        let total: f64 = weights.iter().sum();
        if weights.iter().any(|w| TAU * w / total >= 0.9 * PI) {
            continue;
        }
        let start = uniform(state, 0.0, TAU);
        let mut theta = start;
        let mut pts = Vec::with_capacity(n);
        for w in &weights {
            let rho = uniform(state, 6.0, 12.0);
            pts.push(Point2::new(rho * theta.cos(), rho * theta.sin()));
            theta += TAU * w / total;
        }
        let probe = PrismPart {
            polygon: pts,
            height: 1.0,
            vertical: Vec::new(),
            top: Vec::new(),
        };
        let ok = (0..n).all(|i| {
            let alpha = probe.interior_angle(i);
            (MIN_ANGLE..=TAU - MIN_ANGLE).contains(&alpha)
                && (alpha - PI).abs() >= MIN_BEND
                && probe.edge_len(i) >= MIN_EDGE
        });
        if ok {
            return probe.polygon;
        }
    }
}

/// A random non-empty subset of `0..n`, each element with probability `p`.
fn subset(state: &mut u64, n: usize, p: f64) -> Vec<usize> {
    let mut out: Vec<usize> = (0..n).filter(|_| uniform(state, 0.0, 1.0) < p).collect();
    if out.is_empty() {
        out.push(int(state, 0, n - 1));
    }
    out
}

fn vertical_radius(state: &mut u64, part: &PrismPart, i: usize) -> f64 {
    let n = part.polygon.len();
    let (a, _) = part.blend_angle(i);
    let max = (SETBACK_FRACTION * part.edge_len(i + n - 1).min(part.edge_len(i)) * (0.5 * a).tan())
        .min(MAX_VERTICAL_RADIUS);
    uniform(state, 0.15, 0.9) * max
}

fn prism_vertical(state: &mut u64) -> PrismPart {
    let polygon = polygon(state);
    let height = uniform(state, 5.0, 20.0);
    let mut part = PrismPart {
        polygon,
        height,
        vertical: Vec::new(),
        top: Vec::new(),
    };
    let n = part.polygon.len();
    part.vertical = subset(state, n, 0.5)
        .into_iter()
        .map(|i| (i, vertical_radius(state, &part, i)))
        .collect();
    part
}

fn prism_mixed(state: &mut u64) -> PrismPart {
    let polygon = polygon(state);
    let height = uniform(state, 5.0, 20.0);
    let mut part = PrismPart {
        polygon,
        height,
        vertical: Vec::new(),
        top: Vec::new(),
    };
    let n = part.polygon.len();
    let vertical: Vec<usize> = (0..n).filter(|_| uniform(state, 0.0, 1.0) < 0.4).collect();
    part.vertical = vertical
        .into_iter()
        .map(|i| (i, vertical_radius(state, &part, i)))
        .collect();
    part.top = subset(state, n, 0.5)
        .into_iter()
        .map(|j| {
            let max = (0.3 * height).min(0.2 * part.edge_len(j)).min(3.0);
            (j, uniform(state, 0.15, 0.9) * max)
        })
        .collect();
    part
}

fn radius_too_large(state: &mut u64) -> PrismPart {
    let polygon = polygon(state);
    let height = uniform(state, 5.0, 20.0);
    let mut part = PrismPart {
        polygon,
        height,
        vertical: Vec::new(),
        top: Vec::new(),
    };
    let n = part.polygon.len();
    let convex = |i: usize| part.blend_angle(i % n).1;
    // A convex vertex between convex vertices, else any convex vertex (there are at least 3).
    let candidates: Vec<usize> = (0..n)
        .filter(|&i| convex(i) && convex(i + n - 1) && convex(i + 1))
        .collect();
    let candidates = if candidates.is_empty() {
        (0..n).filter(|&i| convex(i)).collect()
    } else {
        candidates
    };
    let i = candidates[int(state, 0, candidates.len() - 1)];
    let r = part.setback_limit(i) * uniform(state, 1.3, 3.0);
    part.vertical = vec![(i, r)];
    part
}

fn stack(state: &mut u64) -> StackPart {
    let size = Vec3::new(
        uniform(state, 20.0, 40.0),
        uniform(state, 20.0, 40.0),
        uniform(state, 8.0, 15.0),
    );
    let (cw, cd) = (0.5 * size.x, 0.5 * size.y);
    let mut cells = vec![(0, 0), (1, 0), (0, 1), (1, 1)];
    let count = int(state, 1, 3);
    let mut features = Vec::with_capacity(count);
    for _ in 0..count {
        let (cx, cy) = cells.remove(int(state, 0, cells.len() - 1));
        let (x0, y0) = (f64::from(cx) * cw, f64::from(cy) * cd);
        let margin = 3.0;
        let feature = match int(state, 0, 4) {
            0 | 4 => {
                let size2 = Point2::new(
                    uniform(state, 3.0, cw - 2.0 * margin),
                    uniform(state, 3.0, cd - 2.0 * margin),
                );
                let min = Point2::new(
                    x0 + margin + uniform(state, 0.0, cw - 2.0 * margin - size2.x),
                    y0 + margin + uniform(state, 0.0, cd - 2.0 * margin - size2.y),
                );
                if uniform(state, 0.0, 1.0) < 0.5 {
                    Feature::BossBox {
                        min,
                        size: size2,
                        height: uniform(state, 3.0, 8.0),
                    }
                } else {
                    Feature::Pocket {
                        min,
                        size: size2,
                        depth: uniform(state, 0.3, 0.6) * size.z,
                    }
                }
            }
            kind => {
                let radius = uniform(state, 1.5, 0.5 * cw.min(cd) - margin);
                let centre = Point2::new(
                    x0 + margin + radius + uniform(state, 0.0, cw - 2.0 * (margin + radius)),
                    y0 + margin + radius + uniform(state, 0.0, cd - 2.0 * (margin + radius)),
                );
                match kind {
                    1 => Feature::BossCylinder {
                        centre,
                        radius,
                        height: uniform(state, 3.0, 8.0),
                    },
                    2 => Feature::Hole { centre, radius },
                    _ => Feature::BlindHole {
                        centre,
                        radius,
                        depth: uniform(state, 0.3, 0.6) * size.z,
                    },
                }
            }
        };
        features.push(feature);
    }
    StackPart {
        size,
        features,
        edges: int(state, 1, 6),
        radius: uniform(state, 0.3, 1.0),
        edge_seed: splitmix(state),
    }
}

fn corpus_error(reason: String) -> KernelError {
    KernelError::InvalidInput {
        what: "corpus",
        reason,
        offending: Vec::new(),
    }
}

fn at_z(origin: Point3) -> Plane {
    Plane {
        origin,
        ..Plane::XY
    }
}

/// The edge of `s` whose two vertices are `a` and `b` (either way round) within `eps`.
fn edge_between(s: &Shape, a: Point3, b: Point3, eps: f64) -> KResult<EdgeId> {
    let near = |p: Point3, q: Point3| p.distance(q) <= eps;
    let mut hits = s.edges().filter(|&e| {
        let [v0, v1] = s.edge_vertices(e);
        let (p0, p1) = (s.vertex_point(v0), s.vertex_point(v1));
        (near(p0, a) && near(p1, b)) || (near(p0, b) && near(p1, a))
    });
    match (hits.next(), hits.next()) {
        (Some(e), None) => Ok(e),
        _ => Err(corpus_error(format!(
            "no unique edge between {a:?} and {b:?}"
        ))),
    }
}

impl FilletGenCase {
    /// The case id: `fillet/synthetic/<generator>-<seed>`.
    pub fn id(&self) -> String {
        format!(
            "fillet/synthetic/{}-{:04}",
            self.generator.name(),
            self.seed
        )
    }

    /// The part and fillet request this case draws (no kernel needed). The draw depends only
    /// on the generator and the seed.
    pub fn part(&self) -> GenPart {
        // Different generators with the same seed draw independent parts.
        let salt = match self.generator {
            FilletGenerator::PrismVerticalEdges => 0x4b48_0201,
            FilletGenerator::PrismMixedEdges => 0x4b48_0202,
            FilletGenerator::BooleanStack => 0x4b48_0203,
            FilletGenerator::RadiusTooLarge => 0x4b48_0204,
        };
        let mut state = self.seed ^ (salt << 32);
        match self.generator {
            FilletGenerator::PrismVerticalEdges => GenPart::Prism(prism_vertical(&mut state)),
            FilletGenerator::PrismMixedEdges => GenPart::Prism(prism_mixed(&mut state)),
            FilletGenerator::BooleanStack => GenPart::Stack(stack(&mut state)),
            FilletGenerator::RadiusTooLarge => GenPart::Prism(radius_too_large(&mut state)),
        }
    }

    /// The case's tags: the generator name, `synthetic`, `analytic` (every face planar or
    /// cylindrical before the fillet), `convex`/`concave`, `corner3` (a vertex where three
    /// filleted edges meet), `chain` (adjacent filleted top edges), `acute`, and the
    /// `boolean_stack` feature kinds.
    pub fn tags(&self) -> Vec<String> {
        match self.part() {
            GenPart::Prism(p) => p.tags(self.generator),
            GenPart::Stack(s) => {
                let mut tags = vec![
                    self.generator.name().to_owned(),
                    "synthetic".to_owned(),
                    "analytic".to_owned(),
                ];
                for f in &s.features {
                    if !tags.iter().any(|t| t == f.tag()) {
                        tags.push(f.tag().to_owned());
                    }
                }
                tags
            }
        }
    }

    /// The expectation known without a kernel: the exact volume and face count of
    /// `prism_vertical_edges`, the bounds of `prism_mixed_edges`, the clean failure of
    /// `radius_too_large` with its analytic max radius. `boolean_stack` bounds need the built
    /// part ([`Built::expect`]).
    pub fn expect(&self) -> Expect {
        let mut e = Expect::success();
        match (self.generator, self.part()) {
            (FilletGenerator::PrismVerticalEdges, GenPart::Prism(p)) => {
                e.volume = Some(p.vertical_volume());
                e.volume_rel_tol = Some(EXACT_VOLUME_REL_TOL);
                e.faces = u32::try_from(p.polygon.len() + 2 + p.vertical.len()).ok();
            }
            (FilletGenerator::PrismMixedEdges, GenPart::Prism(p)) => {
                let (lo, hi) = p.volume_bounds();
                e.volume_bounds = Some([lo, hi]);
            }
            (FilletGenerator::RadiusTooLarge, GenPart::Prism(p)) => {
                e.outcome = ExpectOutcome::CleanFailure;
                e.failure_class = Some(FailureClass::RadiusTooLarge);
                e.max_radius = p.vertical.first().map(|&(i, _)| p.setback_limit(i));
            }
            _ => {}
        }
        e
    }

    /// Builds the part on `k` and resolves the fillet request to its edges.
    ///
    /// # Errors
    /// The kernel's error when a part cannot be built (`BackendUnavailable` without OCCT);
    /// `InvalidInput { what: "corpus" }` when an edge of the request is not found.
    pub fn build(&self, k: &dyn BrepKernel) -> KResult<Built> {
        let ctx = JobCtx::detached();
        let tol = Tolerance::DEFAULT;
        let mut expect = self.expect();
        match self.part() {
            GenPart::Prism(p) => {
                let face = k.make_planar_polygon(&p.profile(), &[], &tol)?;
                let input = k
                    .prism(
                        &face,
                        &PrismExtent::Vector(Vec3::new(0.0, 0.0, p.height)),
                        &PrismOpts {
                            taper: 0.0,
                            merge_coplanar: false,
                        },
                        &ctx,
                    )?
                    .shape;
                let eps = 1e-6;
                let top = |i: usize| {
                    let q = p.polygon[i % p.polygon.len()];
                    Point3::new(q.x, q.y, p.height)
                };
                let bottom = |i: usize| {
                    let q = p.polygon[i % p.polygon.len()];
                    Point3::new(q.x, q.y, 0.0)
                };
                let mut edges = Vec::with_capacity(p.vertical.len() + p.top.len());
                for &(i, r) in &p.vertical {
                    let e = edge_between(&input, bottom(i), top(i), eps)?;
                    edges.push((e, FilletSpec::Constant(r)));
                }
                for &(j, r) in &p.top {
                    let e = edge_between(&input, top(j), top(j + 1), eps)?;
                    edges.push((e, FilletSpec::Constant(r)));
                }
                Ok(Built {
                    input,
                    edges,
                    opts: FilletOpts::default(),
                    expect,
                })
            }
            GenPart::Stack(s) => {
                let input = build_stack(k, &s, &ctx)?;
                let mut eligible: Vec<EdgeId> = input
                    .edges()
                    .filter(|&e| matches!(*input.edge_faces(e), [a, b] if a != b))
                    .collect();
                let mut state = s.edge_seed;
                let mut chosen = Vec::with_capacity(s.edges);
                while chosen.len() < s.edges && !eligible.is_empty() {
                    chosen.push(eligible.remove(int(&mut state, 0, eligible.len() - 1)));
                }
                chosen.sort_unstable();
                // Every edge of this family is between perpendicular faces: each fillet changes
                // the volume by at most the right-angle corner area times its length, plus 2r³
                // for its ends.
                let r = s.radius;
                let change: f64 = chosen
                    .iter()
                    .map(|&e| {
                        let len = SubShapeGeom::of(
                            &input,
                            SubShape {
                                kind: SubKind::Edge,
                                index: e.0,
                            },
                        )
                        .map_or(0.0, |g| g.measure);
                        corner_area(r, 0.5 * PI) * len + 2.0 * r * r * r
                    })
                    .sum();
                let v = s.volume();
                let slack = change + 1e-6 * v;
                expect.volume_bounds = Some([v - slack, v + slack]);
                Ok(Built {
                    input,
                    edges: chosen
                        .into_iter()
                        .map(|e| (e, FilletSpec::Constant(r)))
                        .collect(),
                    opts: FilletOpts::default(),
                    expect,
                })
            }
        }
    }
}

fn build_stack(k: &dyn BrepKernel, s: &StackPart, ctx: &JobCtx) -> KResult<Shape> {
    let h = s.size.z;
    let mut part = k.make_box(&Plane::XY, s.size)?.shape;
    let cylinder = |c: Point2, z0: f64, r: f64, height: f64| {
        k.make_cylinder(&at_z(Point3::new(c.x, c.y, z0)), r, height, TAU)
            .map(|r| r.shape)
    };
    let cuboid = |min: Point2, z0: f64, size: Point2, height: f64| {
        k.make_box(
            &at_z(Point3::new(min.x, min.y, z0)),
            Vec3::new(size.x, size.y, height),
        )
        .map(|r| r.shape)
    };
    for f in &s.features {
        let (op, tool) = match *f {
            Feature::BossBox { min, size, height } => (BoolOp::Fuse, cuboid(min, h, size, height)?),
            Feature::BossCylinder {
                centre,
                radius,
                height,
            } => (BoolOp::Fuse, cylinder(centre, h, radius, height)?),
            Feature::Hole { centre, radius } => {
                (BoolOp::Cut, cylinder(centre, -1.0, radius, h + 2.0)?)
            }
            Feature::BlindHole {
                centre,
                radius,
                depth,
            } => (
                BoolOp::Cut,
                cylinder(centre, h - depth, radius, depth + 1.0)?,
            ),
            Feature::Pocket { min, size, depth } => {
                (BoolOp::Cut, cuboid(min, h - depth, size, depth + 1.0)?)
            }
        };
        part = k
            .boolean(op, &[part], &[tool], &BoolOpts::default(), ctx)?
            .shape;
    }
    Ok(part)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square(side: f64) -> PrismPart {
        PrismPart {
            polygon: vec![
                Point2::new(0.0, 0.0),
                Point2::new(side, 0.0),
                Point2::new(side, side),
                Point2::new(0.0, side),
            ],
            height: 5.0,
            vertical: Vec::new(),
            top: Vec::new(),
        }
    }

    #[test]
    fn corner_area_is_the_right_angle_spandrel() {
        let r = 2.0;
        assert!((corner_area(r, 0.5 * PI) - r * r * (1.0 - PI / 4.0)).abs() < 1e-14);
        // A flat corner loses nothing; a sharper one more.
        assert!(corner_area(r, PI - 1e-9).abs() < 1e-8);
        assert!(corner_area(r, PI / 3.0) > corner_area(r, PI / 2.0));
    }

    #[test]
    fn square_prism_oracle() {
        let mut p = square(10.0);
        assert!((p.area() - 100.0).abs() < 1e-12);
        for i in 0..4 {
            assert!((p.interior_angle(i) - 0.5 * PI).abs() < 1e-12);
            assert!(p.blend_angle(i).1);
            assert!((p.setback_limit(i) - 10.0).abs() < 1e-12);
        }
        p.vertical = vec![(0, 1.0), (2, 2.0)];
        let want = 5.0 * (100.0 - (1.0 + 4.0) * (1.0 - PI / 4.0));
        assert!((p.vertical_volume() - want).abs() < 1e-12);
    }

    #[test]
    fn concave_vertices_add_material() {
        // An L: vertex 3 at (5, 5) is concave (interior angle 270°).
        let p = PrismPart {
            polygon: vec![
                Point2::new(0.0, 0.0),
                Point2::new(10.0, 0.0),
                Point2::new(10.0, 5.0),
                Point2::new(5.0, 5.0),
                Point2::new(5.0, 10.0),
                Point2::new(0.0, 10.0),
            ],
            height: 2.0,
            vertical: vec![(3, 1.0)],
            top: Vec::new(),
        };
        assert!((p.interior_angle(3) - 1.5 * PI).abs() < 1e-12);
        let (a, convex) = p.blend_angle(3);
        assert!(!convex && (a - 0.5 * PI).abs() < 1e-12);
        let want = 2.0 * (75.0 + (1.0 - PI / 4.0));
        assert!((p.vertical_volume() - want).abs() < 1e-12);
    }

    #[test]
    fn generated_polygons_are_simple_and_ccw() {
        for seed in 0..300 {
            let mut state = seed;
            let pts = polygon(&mut state);
            let p = PrismPart {
                polygon: pts.clone(),
                height: 1.0,
                vertical: Vec::new(),
                top: Vec::new(),
            };
            assert!(p.area() > 0.0, "seed {seed}");
            // Interior angles of a simple polygon sum to (n − 2)π.
            let sum: f64 = (0..pts.len()).map(|i| p.interior_angle(i)).sum();
            let want = (pts.len() as f64 - 2.0) * PI;
            assert!((sum - want).abs() < 1e-9, "seed {seed}: {sum} vs {want}");
            // No two non-adjacent edges cross.
            let n = pts.len();
            for i in 0..n {
                for j in i + 2..n {
                    if i == 0 && j == n - 1 {
                        continue;
                    }
                    assert!(
                        !segments_cross(pts[i], pts[(i + 1) % n], pts[j], pts[(j + 1) % n]),
                        "seed {seed}: edges {i} and {j} cross"
                    );
                }
            }
        }
    }

    fn segments_cross(a: Point2, b: Point2, c: Point2, d: Point2) -> bool {
        let side = |p: Point2, q: Point2, r: Point2| (q - p).perp_dot(r - p);
        let (d1, d2) = (side(a, b, c), side(a, b, d));
        let (d3, d4) = (side(c, d, a), side(c, d, b));
        d1 * d2 < 0.0 && d3 * d4 < 0.0
    }

    #[test]
    fn generators_respect_their_limits() {
        for seed in 0..200 {
            for generator in FilletGenerator::ALL {
                let case = FilletGenCase { generator, seed };
                assert_eq!(case.part(), case.part(), "deterministic");
                match case.part() {
                    GenPart::Prism(p) => {
                        let n = p.polygon.len();
                        assert!((3..=12).contains(&n));
                        for &(i, r) in &p.vertical {
                            let limit = p.setback_limit(i);
                            if generator == FilletGenerator::RadiusTooLarge {
                                assert!(r >= 1.3 * limit - 1e-12 && p.blend_angle(i).1);
                            } else {
                                assert!(r > 0.0 && r <= SETBACK_FRACTION * limit + 1e-12);
                            }
                        }
                        for &(j, r) in &p.top {
                            assert!(r > 0.0 && r <= 0.2 * p.edge_len(j) && r <= 0.3 * p.height);
                        }
                        if generator == FilletGenerator::PrismVerticalEdges {
                            assert!(p.top.is_empty() && !p.vertical.is_empty());
                            let v = case.expect().volume.unwrap();
                            assert!(v > 0.0, "seed {seed}");
                        }
                    }
                    GenPart::Stack(s) => {
                        assert!(!s.features.is_empty() && s.features.len() <= 3);
                        assert!(s.volume() > 0.0);
                        assert!((1..=6).contains(&s.edges));
                    }
                }
            }
        }
    }

    #[test]
    fn generator_names_round_trip() {
        for g in FilletGenerator::ALL {
            assert_eq!(FilletGenerator::from_name(g.name()), Some(g));
            let json = serde_json::to_string(&g).unwrap();
            assert_eq!(json, format!("\"{}\"", g.name()));
        }
        assert_eq!(FilletGenerator::from_name("nope"), None);
    }
}
