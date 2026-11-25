//! The concurrency stress harness (S02 §2, occt-integration.md §4.3, BREP-12).
//!
//! N threads run a seeded random mix of ops on one shared `Shape`. Every result must equal the
//! single-threaded result bit for bit (results are compared as `Debug` strings, which keep every
//! f64 bit, `-0.0` included), and the shared input's `BinTools` bytes must be unchanged
//! afterwards (no op wrote into it). Each op WP adds its op to [`StressOp`] and to the op list
//! of `tests/stress.rs`.

use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};
use std::sync::Mutex;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::{Duration, Instant};

use amg_brep::geom::Surface;
use amg_brep::{
    BrepKernel, ChamferSpec, CheckOpts, Dir3, EdgeId, FaceId, FilletOpts, FilletSpec, JobCtx,
    KResult, Line3, OcctKernel, OpResult, Point2, Point3, PrismExtent, PrismOpts, Shape, SubKind,
    SubShape, TessMesh, TessQuality, Vec3, VertexId,
};

/// One deterministic operation on the shared shape.
#[derive(Clone, Copy, Debug)]
pub enum StressOp {
    Surface(FaceId),
    FaceArea(FaceId),
    UvBounds(FaceId),
    FaceReversed(FaceId),
    FaceEdges(FaceId),
    FaceWires(FaceId),
    IsSeam(EdgeId, FaceId),
    Curve(EdgeId),
    EdgeFaces(EdgeId),
    EdgeVertices(EdgeId),
    VertexPoint(VertexId),
    VertexEdges(VertexId),
    Tolerance(SubShape),
    /// Extract as a standalone shape, then query it.
    SubShape(SubShape),
    MaxTolerance,
    Bbox,
    Counts,
    Solids,
    EstimatedBytes,
    /// `BRepCheck_Analyzer` (category R).
    Check,
    /// `tessellate` (category C: meshes a private copy), BREP-08.
    Tessellate(TessQuality),
    /// `tessellate_edge` (category R), BREP-08.
    TessellateEdge(EdgeId, TessQuality),
    /// `prism` of a planar face of the shared shape (category C: sweeps a private copy) along
    /// its normal, tapered by the given angle in milliradians, BREP-14.
    Prism(FaceId, i32),
    /// `revol` by 90° of a planar face of the shared shape about a line in its plane beside it
    /// (category C), BREP-14.
    Revol(FaceId),
    /// The BREP-09 measurement queries (category R).
    MassProps,
    Classify(Point3),
    ClosestPoint(Point3),
    RayCast(Point3, Dir3),
    EvalSurface(FaceId, Point2),
    ProjectOnFace(FaceId, Point3),
    EdgeEnds(EdgeId),
    /// `distance` to a sub-shape of the shared shape (both arguments share its TShapes).
    DistanceToSub(SubShape),
    /// `fillet` of one edge of the shared shape itself, radius in µm (category C: blends a
    /// private copy), BREP-06.
    Fillet(EdgeId, u32),
    /// `chamfer` of one edge of the shared shape, distance in µm (category C), BREP-06.
    Chamfer(EdgeId, u32),
    /// `heal` of the shared shape (category C: fixes a private copy), BREP-10.
    Heal,
    /// `unify_same_domain` of the shared shape (category C), BREP-10.
    Unify,
    /// `sew` of the shared shape's faces (`sub_shape`s sharing its TShapes; category C), BREP-10.
    SewFaces,
    /// `make_compound` of the shared shape and a moved copy (shares its sub-shapes), BREP-10.
    Compound,
    /// `check` with the `BOPAlgo_ArgumentAnalyzer` pass (category C: a private copy), BREP-10.
    CheckFull,
}

/// The BREP-10 healing and assembly ops on the shared shape.
pub fn heal_queries() -> Vec<StressOp> {
    vec![
        StressOp::Heal,
        StressOp::Unify,
        StressOp::SewFaces,
        StressOp::Compound,
        StressOp::CheckFull,
    ]
}

/// The BREP-06 blend ops: a fillet and a chamfer of up to `limit` edges between two faces, spread
/// over the shape.
pub fn blend_queries(shape: &Shape, limit: usize) -> Vec<StressOp> {
    let edges: Vec<EdgeId> = shape
        .edges()
        .filter(|&e| matches!(shape.edge_faces(e), [a, b] if a != b))
        .collect();
    let step = (edges.len() / limit).max(1);
    edges
        .into_iter()
        .step_by(step)
        .take(limit)
        .flat_map(|e| [StressOp::Fillet(e, 500), StressOp::Chamfer(e, 400)])
        .collect()
}

/// The BREP-09 measurement ops: mass properties, then classification, closest points and rays
/// from points around the bbox, surface evaluation and projection on up to `limit` faces, and
/// the ends of up to `limit` edges.
pub fn measure_queries(shape: &Shape, limit: usize) -> Vec<StressOp> {
    let bbox = shape.bbox();
    let centre = bbox.center();
    let size = bbox.max - bbox.min;
    let mut ops = vec![StressOp::MassProps];
    for k in 0..8u32 {
        let t = f64::from(k) / 8.0;
        let inside = bbox.min + size * Vec3::new(0.3 + 0.4 * t, 0.5, 0.7 - 0.4 * t);
        let outside = centre + size * Vec3::new(0.9 - 0.2 * t, -0.8, 0.6 + 0.1 * t);
        ops.extend([
            StressOp::Classify(inside),
            StressOp::ClosestPoint(outside),
            StressOp::ClosestPoint(inside),
            StressOp::RayCast(outside, Dir3::new_unchecked((inside - outside).normalize())),
        ]);
    }
    let step = (shape.count(SubKind::Face) as usize / limit).max(1);
    for f in shape.faces().step_by(step).take(limit) {
        let [u0, u1, v0, v1] = shape.face_uv_bounds(f);
        let uv = Point2::new(0.37 * u0 + 0.63 * u1, 0.58 * v0 + 0.42 * v1);
        ops.push(StressOp::EvalSurface(f, uv));
        ops.push(StressOp::ProjectOnFace(f, centre + size));
    }
    let step = (shape.count(SubKind::Edge) as usize / limit).max(1);
    for e in shape.edges().step_by(step).take(limit) {
        ops.push(StressOp::EdgeEnds(e));
    }
    ops.push(StressOp::DistanceToSub(SubShape {
        kind: SubKind::Face,
        index: 0,
    }));
    ops
}

/// The BREP-14 sweep ops: a straight prism and a revol of every planar face (at most `limit`
/// faces), and a tapered prism of the ones with at most 16 edges (a draft of every side face of a
/// large cap is slow).
pub fn sweep_queries(shape: &Shape, limit: usize) -> Vec<StressOp> {
    let mut ops = Vec::new();
    for f in shape
        .faces()
        .filter(|&f| matches!(shape.surface(f), Surface::Plane(_)))
        .take(limit)
    {
        ops.extend([StressOp::Prism(f, 0), StressOp::Revol(f)]);
        if shape.face_edges(f).len() <= 16 {
            ops.push(StressOp::Prism(f, -50));
        }
    }
    ops
}

/// The face of `shape` as a standalone profile (shares its TShapes) and its plane.
fn planar_profile(shape: &Shape, f: FaceId) -> (Shape, amg_brep::Plane) {
    let Surface::Plane(plane) = shape.surface(f) else {
        panic!("{f:?} is not planar");
    };
    let face = shape
        .sub_shape(SubShape {
            kind: SubKind::Face,
            index: f.0,
        })
        .expect("sub_shape");
    (face, plane)
}

/// An op result as an exact string: volume bits, counts and the whole history.
fn sweep_digest(result: KResult<OpResult>) -> String {
    match result {
        Ok(r) => format!(
            "{:016x} {} {} {} {:?}",
            r.shape.volume().expect("volume").to_bits(),
            r.shape.count(SubKind::Face),
            r.shape.count(SubKind::Edge),
            r.shape.count(SubKind::Vertex),
            r.history
        ),
        Err(e) => format!("Err {e:?}"),
    }
}

/// The BREP-08 tessellation ops: whole-shape meshes at two presets (first, so every worker
/// starts by meshing) and about eight edges.
pub fn tess_queries(shape: &Shape) -> Vec<StressOp> {
    let mut ops = vec![
        StressOp::Tessellate(TessQuality::preview()),
        StressOp::Tessellate(TessQuality::display()),
    ];
    let step = (shape.count(SubKind::Edge) as usize / 8).max(1);
    for e in shape.edges().step_by(step) {
        if shape.curve(e).is_some() {
            ops.push(StressOp::TessellateEdge(e, TessQuality::display()));
        }
    }
    ops
}

/// Hash of every bit of a mesh (`Debug` strings of large meshes are slow to build).
pub fn mesh_hash(mesh: &TessMesh) -> u64 {
    let mut h = DefaultHasher::new();
    for p in mesh
        .positions
        .iter()
        .chain(&mesh.normals)
        .chain(&mesh.edge_points)
    {
        for c in p.to_array() {
            c.to_bits().hash(&mut h);
        }
    }
    mesh.tris.hash(&mut h);
    for r in &mesh.face_ranges {
        (r.start, r.end).hash(&mut h);
    }
    mesh.weld.hash(&mut h);
    for e in &mesh.edges {
        (e.edge, e.points.start, e.points.end).hash(&mut h);
    }
    h.finish()
}

/// The category-R queries over every sub-shape of `shape`.
pub fn read_queries(shape: &Shape) -> Vec<StressOp> {
    let mut ops = vec![
        StressOp::MaxTolerance,
        StressOp::Bbox,
        StressOp::Counts,
        StressOp::Solids,
        StressOp::EstimatedBytes,
        StressOp::Check,
    ];
    for f in shape.faces() {
        ops.extend([
            StressOp::Surface(f),
            StressOp::FaceArea(f),
            StressOp::UvBounds(f),
            StressOp::FaceReversed(f),
            StressOp::FaceEdges(f),
            StressOp::FaceWires(f),
        ]);
        if let Some(&e) = shape.face_edges(f).first() {
            ops.push(StressOp::IsSeam(e, f));
        }
    }
    for e in shape.edges() {
        ops.extend([
            StressOp::Curve(e),
            StressOp::EdgeFaces(e),
            StressOp::EdgeVertices(e),
        ]);
    }
    for v in shape.vertices() {
        ops.extend([StressOp::VertexPoint(v), StressOp::VertexEdges(v)]);
    }
    for kind in [SubKind::Face, SubKind::Edge, SubKind::Vertex] {
        for index in 0..shape.count(kind) {
            let sub = SubShape { kind, index };
            ops.push(StressOp::Tolerance(sub));
            if index % 7 == 0 {
                ops.push(StressOp::SubShape(sub));
            }
        }
    }
    ops
}

/// Runs `op` on `shape`; the result as an exact string.
pub fn eval(kernel: &OcctKernel, shape: &Shape, op: StressOp) -> String {
    match op {
        StressOp::Surface(f) => format!("{:?}", shape.surface(f)),
        StressOp::FaceArea(f) => format!("{:?}", shape.face_area(f)),
        StressOp::UvBounds(f) => format!("{:?}", shape.face_uv_bounds(f)),
        StressOp::FaceReversed(f) => format!("{:?}", shape.face_reversed(f)),
        StressOp::FaceEdges(f) => format!("{:?}", shape.face_edges(f)),
        StressOp::FaceWires(f) => format!("{:?}", shape.face_wires(f)),
        StressOp::IsSeam(e, f) => format!("{:?}", shape.is_seam(e, f)),
        StressOp::Curve(e) => format!("{:?}", shape.curve(e)),
        StressOp::EdgeFaces(e) => format!("{:?}", shape.edge_faces(e)),
        StressOp::EdgeVertices(e) => format!("{:?}", shape.edge_vertices(e)),
        StressOp::VertexPoint(v) => format!("{:?}", shape.vertex_point(v)),
        StressOp::VertexEdges(v) => format!("{:?}", shape.vertex_edges(v)),
        StressOp::Tolerance(s) => format!("{:?}", shape.tolerance(s)),
        StressOp::SubShape(s) => {
            let sub = shape.sub_shape(s).expect("sub_shape");
            format!(
                "{:?} {:?} {:?}",
                sub.kind(),
                sub.bbox(),
                sub.max_tolerance()
            )
        }
        StressOp::MaxTolerance => format!("{:?}", shape.max_tolerance()),
        StressOp::Bbox => format!("{:?}", shape.bbox()),
        StressOp::Counts => format!(
            "{} {} {}",
            shape.count(SubKind::Face),
            shape.count(SubKind::Edge),
            shape.count(SubKind::Vertex)
        ),
        StressOp::Solids => {
            let solids = shape.solids();
            let counts: Vec<u32> = solids.iter().map(|s| s.count(SubKind::Face)).collect();
            format!("{counts:?}")
        }
        StressOp::EstimatedBytes => format!("{}", shape.estimated_bytes()),
        StressOp::Check => format!("{:?}", kernel.check(shape, &CheckOpts::default())),
        StressOp::Tessellate(q) => {
            let mesh = kernel
                .tessellate(shape, &q, &JobCtx::detached())
                .expect("tessellate");
            format!("{} tris {:016x}", mesh.triangle_count(), mesh_hash(&mesh))
        }
        StressOp::Prism(f, milliradians) => {
            let (profile, plane) = planar_profile(shape, f);
            let opts = PrismOpts {
                taper: f64::from(milliradians) * 1e-3,
                merge_coplanar: false,
            };
            let extent = PrismExtent::Vector(3.0 * plane.normal.get());
            sweep_digest(kernel.prism(&profile, &extent, &opts, &JobCtx::detached()))
        }
        StressOp::Revol(f) => {
            let (profile, plane) = planar_profile(shape, f);
            let bbox = profile.bbox();
            let reach = (bbox.max - bbox.min).length() + 1.0;
            let axis = Line3 {
                point: (bbox.min + bbox.max) / 2.0 - reach * plane.y.get(),
                dir: Dir3::new_unchecked(plane.x.get()),
            };
            sweep_digest(kernel.revol(
                &profile,
                &axis,
                std::f64::consts::FRAC_PI_2,
                &JobCtx::detached(),
            ))
        }
        StressOp::Heal => {
            sweep_digest(kernel.heal(shape, &amg_brep::HealOpts::default(), &JobCtx::detached()))
        }
        StressOp::Unify => {
            sweep_digest(kernel.unify_same_domain(shape, &amg_brep::UnifyOpts::default()))
        }
        StressOp::SewFaces => {
            let faces: Vec<Shape> = shape
                .faces()
                .map(|f| {
                    shape
                        .sub_shape(SubShape {
                            kind: SubKind::Face,
                            index: f.0,
                        })
                        .expect("sub_shape")
                })
                .collect();
            sweep_digest(kernel.sew(&faces, &amg_brep::SewOpts::default(), &JobCtx::detached()))
        }
        StressOp::Compound => {
            let moved = shape
                .transformed(&amg_brep::Transform::from_translation(Vec3::new(
                    1e3, 0.0, 0.0,
                )))
                .expect("transformed");
            match kernel.make_compound(&[shape.clone(), moved]) {
                Ok(c) => format!(
                    "{} {} {} {:?}",
                    c.count(SubKind::Face),
                    c.count(SubKind::Edge),
                    c.count(SubKind::Vertex),
                    c.bbox()
                ),
                Err(e) => format!("Err {e:?}"),
            }
        }
        StressOp::CheckFull => format!(
            "{:?}",
            kernel.check(
                shape,
                &CheckOpts {
                    self_interference: true
                }
            )
        ),
        StressOp::MassProps => format!("{:?}", kernel.mass_props(shape)),
        StressOp::Classify(p) => format!("{:?}", kernel.classify(shape, p, 1e-7)),
        StressOp::ClosestPoint(p) => format!("{:?}", kernel.closest_point(shape, p)),
        StressOp::RayCast(origin, dir) => format!(
            "{:?}",
            kernel.ray_cast(shape, &amg_brep::Ray3 { origin, dir }, 1e6)
        ),
        StressOp::EvalSurface(f, uv) => format!("{:?}", kernel.eval_surface(shape, f, uv)),
        StressOp::ProjectOnFace(f, p) => {
            format!("{:?}", kernel.project_point_on_face(shape, f, p))
        }
        StressOp::EdgeEnds(e) => format!("{:?}", kernel.edge_ends(shape, e)),
        StressOp::DistanceToSub(s) => {
            let sub = shape.sub_shape(s).expect("sub_shape");
            let moved = sub
                .transformed(&amg_brep::Transform::from_translation(Vec3::new(
                    0.0, 0.0, 1e3,
                )))
                .expect("transformed");
            format!("{:?}", kernel.distance(shape, &moved))
        }
        StressOp::Fillet(e, micrometres) => sweep_digest(kernel.fillet(
            shape,
            &[(e, FilletSpec::Constant(f64::from(micrometres) * 1e-3))],
            &FilletOpts::default(),
            &JobCtx::detached(),
        )),
        StressOp::Chamfer(e, micrometres) => sweep_digest(kernel.chamfer(
            shape,
            &[(e, ChamferSpec::Equal(f64::from(micrometres) * 1e-3))],
            &JobCtx::detached(),
        )),
        StressOp::TessellateEdge(e, q) => format!(
            "{:?}",
            kernel
                .tessellate_edge(shape, e, &q)
                .expect("tessellate_edge")
        ),
    }
}

/// `Shape::write_bin` bytes and their hash.
pub fn bin_hash(shape: &Shape) -> (Vec<u8>, u64) {
    let mut bytes = Vec::new();
    shape.write_bin(&mut bytes).expect("write_bin");
    let mut hasher = DefaultHasher::new();
    bytes.hash(&mut hasher);
    (bytes, hasher.finish())
}

/// SplitMix64: a seeded, dependency-free op picker.
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Rng {
        Rng(seed)
    }

    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    pub fn below(&mut self, n: usize) -> usize {
        usize::try_from(self.next_u64() % n as u64).expect("fits")
    }
}

/// What one stress run saw.
#[derive(Debug)]
pub struct StressReport {
    pub threads: usize,
    pub iterations: usize,
    pub ops: usize,
    pub mismatches: Vec<String>,
    pub hash_before: u64,
    pub hash_after: u64,
    pub input_unchanged: bool,
    pub elapsed: Duration,
}

/// Runs `threads × iterations` random `ops` on `shared` and compares each result with
/// `reference[op]`, computed beforehand on an independent copy of the same shape.
pub fn run(
    kernel: &OcctKernel,
    shared: &Shape,
    ops: &[StressOp],
    reference: &[String],
    threads: usize,
    iterations: usize,
    seed: u64,
) -> StressReport {
    run_with(
        kernel,
        shared,
        ops,
        reference,
        threads,
        iterations,
        seed,
        || {},
    )
    .0
}

/// [`run`] while one extra thread calls `background` in a loop (at least once) until the
/// workers finish, for example booleans on the shared shape (BREP-08 Done-when 4). Also
/// returns how many times `background` ran.
#[allow(clippy::too_many_arguments, reason = "test harness")]
pub fn run_with(
    kernel: &OcctKernel,
    shared: &Shape,
    ops: &[StressOp],
    reference: &[String],
    threads: usize,
    iterations: usize,
    seed: u64,
    background: impl Fn() + Sync,
) -> (StressReport, usize) {
    assert_eq!(ops.len(), reference.len());
    let (bytes_before, hash_before) = bin_hash(shared);
    // Every thread starts with the slowest op, so the heavy ones overlap at least once.
    let first = ops
        .iter()
        .position(|op| matches!(op, StressOp::Check))
        .unwrap_or(0);
    let mismatches = Mutex::new(Vec::new());
    let start = Instant::now();
    let running = AtomicUsize::new(threads);
    let background_runs = AtomicUsize::new(0);
    std::thread::scope(|scope| {
        scope.spawn(|| {
            loop {
                background();
                background_runs.fetch_add(1, Ordering::Relaxed);
                if running.load(Ordering::Acquire) == 0 {
                    break;
                }
            }
        });
        for t in 0..threads {
            let mismatches = &mismatches;
            let running = &running;
            scope.spawn(move || {
                let mut rng = Rng::new(seed ^ (t as u64).wrapping_mul(0xA24B_AED4_963E_E407));
                for i in 0..iterations {
                    let k = if i == 0 { first } else { rng.below(ops.len()) };
                    let got = eval(kernel, shared, ops[k]);
                    if got != reference[k] {
                        mismatches.lock().unwrap().push(format!(
                            "thread {t} iteration {i} {:?}: {got} != {}",
                            ops[k], reference[k]
                        ));
                    }
                }
                running.fetch_sub(1, Ordering::Release);
            });
        }
    });
    let elapsed = start.elapsed();
    let (bytes_after, hash_after) = bin_hash(shared);
    let report = StressReport {
        threads,
        iterations,
        ops: ops.len(),
        mismatches: mismatches.into_inner().unwrap(),
        hash_before,
        hash_after,
        input_unchanged: bytes_before == bytes_after,
        elapsed,
    };
    (report, background_runs.into_inner())
}
