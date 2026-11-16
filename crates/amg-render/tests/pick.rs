//! RND-04 GPU tests: picking (faces, triangles, edges, vertices, caps, rectangles) and the
//! hover/selection highlight. Named `gpu_*` (nextest test-group `gpu`, serial); they skip
//! without an adapter and fail under `AMG_REQUIRE_GPU=1` (CI runs them on WARP).
//!
//! The test part is a box with a square through-hole whose faces, edges and vertices are known,
//! so every pick is checked against an f64 CPU ray cast of the same triangles.

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]
#![allow(
    clippy::float_cmp,
    reason = "exact pick depths are compared between backends"
)]

use std::collections::HashMap;
use std::sync::Arc;

use amg_core::{Dir3, Plane, Point3, Ray3, Tolerance, Transform, Vec3};
use amg_jobs::JobCtx;
use amg_jobs::gpu::{Gpu, GpuConfig};
use amg_render::{
    BODY_SUB, Background, CAP_SUB, Camera, EdgeData, FaceRange, HighlightItem, HighlightLayer,
    Instance, InstanceKey, Lighting, LineStyle, MeshData, PickBackend, PickFilter, PickHit, PickId,
    PickRequest, PickResult, PointData, PointStyle, Polyline, Projection, Range32, RectMode,
    Renderer, RendererConfig, RgbaImage, SectionPlane, SubKind, ViewId, ViewStyle, VisualStyle,
    upload_edges, upload_mesh, upload_mesh_with_faces, upload_points,
};

const SIZE: [u32; 2] = [400, 300];
const GREY: [f32; 4] = [0.6, 0.6, 0.6, 1.0];

fn gpu() -> Option<Arc<Gpu>> {
    if !amg_testkit::gpu_available() {
        return None;
    }
    let mut cfg = GpuConfig::from_env();
    cfg.optional_features |= amg_render::OPTIONAL_FEATURES;
    let gpu = Gpu::new_blocking(&cfg).expect("adapter probed, device opens");
    let info = gpu.info();
    eprintln!("adapter: {} ({:?})", info.name, info.backend);
    Some(gpu)
}

fn renderer(gpu: &Arc<Gpu>, samples: u32, backend: PickBackend) -> Renderer {
    Renderer::new(
        Arc::clone(gpu),
        &RendererConfig {
            msaa_samples: samples,
            pick_backend: backend,
            ..Default::default()
        },
    )
    .unwrap()
}

fn style() -> ViewStyle {
    ViewStyle {
        background: Background::Solid([0.0, 0.0, 0.0]),
        lighting: Lighting::Flat,
        visual: VisualStyle::ShadedVisibleEdges,
        ..Default::default()
    }
}

fn add_view(r: &mut Renderer, camera: Camera, size: [u32; 2]) -> ViewId {
    let view = r.create_view(size);
    let v = r.view_mut(view).unwrap();
    v.camera = camera;
    v.style = style();
    assert!(r.prepare_view(view), "pipeline variants did not finish");
    view
}

fn capture(r: &mut Renderer, view: ViewId) -> RgbaImage {
    assert!(r.prepare_view(view), "pipeline variants did not finish");
    r.capture(view).unwrap().wait().unwrap()
}

/// Requests every pick, `per_frame` at a time, and waits for the answers; asserts that each
/// arrives within two `encode` calls of its request.
fn pick_all(r: &mut Renderer, view: ViewId, requests: &[PickRequest]) -> Vec<PickResult> {
    let mut out = Vec::with_capacity(requests.len());
    for chunk in requests.chunks(200) {
        let tickets: Vec<_> = chunk.iter().map(|q| r.request_pick(view, *q)).collect();
        let mut got = HashMap::new();
        let mut encodes = 0;
        while got.len() < tickets.len() {
            r.capture(view).unwrap().wait().unwrap();
            encodes += 1;
            for (t, res) in r.poll_picks() {
                got.insert(t, res.expect("pick succeeds"));
            }
            assert!(encodes <= 2, "picks not answered within two encodes");
        }
        out.extend(tickets.iter().map(|t| got.remove(t).unwrap()));
    }
    out
}

fn points(results: &[PickResult]) -> Vec<Vec<PickHit>> {
    results
        .iter()
        .map(|r| match r {
            PickResult::Point(h) => h.clone(),
            PickResult::Rect(_) => panic!("point pick answered with a rectangle"),
        })
        .collect()
}

/// xorshift64*: deterministic samples without a dependency.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        (self.0.wrapping_mul(0x2545_f491_4f6c_dd1d) >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// The box with a square through-hole: outer half-width `A`, hole half-width `B`, height `H`.
/// Faces: 0 top, 1 bottom, 2..=5 outer sides, 6..=9 hole walls (no vertex shared between
/// faces). Edges: 0..4 top outer, 4..8 top inner, 8..12 bottom outer, 12..16 bottom inner,
/// 16..20 vertical outer, 20..24 vertical inner. Vertices: top outer, top inner, bottom outer,
/// bottom inner (4 each).
const A: f64 = 50.0;
const B: f64 = 20.0;
const H: f64 = 30.0;

struct Part {
    origin: Point3,
    positions: Vec<[f32; 3]>,
    indices: Vec<u32>,
    faces: Vec<FaceRange>,
    edges: Vec<[Point3; 2]>,
    vertices: Vec<Point3>,
}

impl Part {
    fn new(origin: Point3) -> Part {
        let corner = |half: f64, i: usize, z: f64| {
            let (x, y) = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)][i % 4];
            Vec3::new(x * half, y * half, z)
        };
        let mut part = Part {
            origin,
            positions: Vec::new(),
            indices: Vec::new(),
            faces: Vec::new(),
            edges: Vec::new(),
            vertices: Vec::new(),
        };
        let face = |part: &mut Part, quads: Vec<[Vec3; 4]>| {
            let first_tri = (part.indices.len() / 3) as u32;
            for q in &quads {
                let base = part.positions.len() as u32;
                for v in q {
                    part.positions.push([v.x as f32, v.y as f32, v.z as f32]);
                }
                part.indices
                    .extend([base, base + 1, base + 2, base, base + 2, base + 3]);
            }
            let face = part.faces.len() as u32;
            part.faces.push(FaceRange {
                first_tri,
                tri_count: 2 * quads.len() as u32,
                face,
            });
        };
        let ring = |z: f64, up: bool| -> Vec<[Vec3; 4]> {
            (0..4)
                .map(|i| {
                    let q = [
                        corner(A, i, z),
                        corner(A, i + 1, z),
                        corner(B, i + 1, z),
                        corner(B, i, z),
                    ];
                    if up { q } else { [q[3], q[2], q[1], q[0]] }
                })
                .collect()
        };
        face(&mut part, ring(H, true));
        face(&mut part, ring(0.0, false));
        for i in 0..4 {
            face(
                &mut part,
                vec![[
                    corner(A, i, 0.0),
                    corner(A, i + 1, 0.0),
                    corner(A, i + 1, H),
                    corner(A, i, H),
                ]],
            );
        }
        for i in 0..4 {
            face(
                &mut part,
                vec![[
                    corner(B, i + 1, 0.0),
                    corner(B, i, 0.0),
                    corner(B, i, H),
                    corner(B, i + 1, H),
                ]],
            );
        }
        let o = origin;
        for (half, z) in [(A, H), (B, H), (A, 0.0), (B, 0.0)] {
            for i in 0..4 {
                part.edges
                    .push([o + corner(half, i, z), o + corner(half, i + 1, z)]);
            }
        }
        for half in [A, B] {
            for i in 0..4 {
                part.edges
                    .push([o + corner(half, i, 0.0), o + corner(half, i, H)]);
            }
        }
        for (half, z) in [(A, H), (B, H), (A, 0.0), (B, 0.0)] {
            for i in 0..4 {
                part.vertices.push(o + corner(half, i, z));
            }
        }
        part
    }

    fn mesh(&self) -> MeshData {
        MeshData {
            origin: self.origin,
            positions: self.positions.clone(),
            normals: Vec::new(),
            indices: self.indices.clone(),
            closed: true,
        }
    }

    fn edge_data(&self) -> EdgeData {
        let rel = |p: Point3| {
            let d = p - self.origin;
            [d.x as f32, d.y as f32, d.z as f32]
        };
        EdgeData {
            origin: self.origin,
            points: self
                .edges
                .iter()
                .flat_map(|e| [rel(e[0]), rel(e[1])])
                .collect(),
            polylines: (0..self.edges.len() as u32)
                .map(|i| Polyline {
                    first: 2 * i,
                    len: 2,
                    id: i,
                    closed: false,
                })
                .collect(),
        }
    }

    fn point_data(&self) -> PointData {
        PointData {
            origin: self.origin,
            points: self
                .vertices
                .iter()
                .map(|&p| {
                    let d = p - self.origin;
                    [d.x as f32, d.y as f32, d.z as f32]
                })
                .collect(),
            ids: Vec::new(),
        }
    }

    fn triangle(&self, t: usize) -> [Point3; 3] {
        let p = |i: u32| {
            let v = self.positions[i as usize];
            self.origin + Vec3::new(f64::from(v[0]), f64::from(v[1]), f64::from(v[2]))
        };
        [
            p(self.indices[3 * t]),
            p(self.indices[3 * t + 1]),
            p(self.indices[3 * t + 2]),
        ]
    }

    fn face_of(&self, t: u32) -> u32 {
        self.faces
            .iter()
            .find(|f| t >= f.first_tri && t < f.first_tri + f.tri_count)
            .unwrap()
            .face
    }

    /// Nearest hit of `ray`: (distance along the ray, triangle).
    fn cast(&self, ray: &Ray3) -> Option<(f64, u32)> {
        let mut best: Option<(f64, u32)> = None;
        for t in 0..self.indices.len() / 3 {
            if let Some(d) = ray_triangle(ray, self.triangle(t))
                && best.is_none_or(|(b, _)| d < b)
            {
                best = Some((d, t as u32));
            }
        }
        best
    }
}

/// Möller–Trumbore, both sides, f64.
fn ray_triangle(ray: &Ray3, [a, b, c]: [Point3; 3]) -> Option<f64> {
    let dir = ray.dir.get();
    let (e1, e2) = (b - a, c - a);
    let p = dir.cross(e2);
    let det = e1.dot(p);
    if det.abs() < 1e-14 {
        return None;
    }
    let s = ray.origin - a;
    let u = s.dot(p) / det;
    let q = s.cross(e1);
    let v = dir.dot(q) / det;
    let t = e2.dot(q) / det;
    (u >= 0.0 && v >= 0.0 && u + v <= 1.0 && t > 0.0).then_some(t)
}

fn segment_distance(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let len2 = dx * dx + dy * dy;
    let t = if len2 > 0.0 {
        (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    ((p[0] - a[0] - t * dx).powi(2) + (p[1] - a[1] - t * dy).powi(2)).sqrt()
}

fn px_of(cam: &Camera, p: Point3, size: [u32; 2]) -> [f64; 2] {
    let s = cam.project(p, size).unwrap();
    [s[0], s[1]]
}

fn oblique(origin: Point3) -> Camera {
    Camera {
        eye: origin + Vec3::new(70.0, -160.0, 140.0),
        target: origin + Vec3::new(0.0, 0.0, 8.0),
        up: Dir3::Z,
        projection: Projection::Perspective {
            fov_y: 40f64.to_radians(),
        },
    }
}

/// Adds the part as a B-rep instance (faces, 1-px edges, optionally vertices) with pick id 1.
fn add_part(r: &mut Renderer, gpu: &Gpu, part: &Part, brep: bool, points: bool) -> InstanceKey {
    let ctx = JobCtx::detached();
    let mesh = if brep {
        upload_mesh_with_faces(gpu, &part.mesh(), &part.faces, &ctx).unwrap()
    } else {
        upload_mesh(gpu, &part.mesh(), &ctx).unwrap()
    };
    let mesh = r.scene_mut().insert_mesh(mesh);
    let edges = r
        .scene_mut()
        .insert_edges(upload_edges(gpu, &part.edge_data(), &ctx).unwrap());
    let mut inst = Instance::new(mesh, Transform::IDENTITY);
    inst.edges = Some(edges);
    if points {
        inst.points = Some(
            r.scene_mut()
                .insert_points(upload_points(gpu, &part.point_data(), &ctx).unwrap()),
        );
        inst.point_style = PointStyle {
            size_px: 5.0,
            ..PointStyle::default()
        };
    }
    inst.edge_style = LineStyle {
        width_px: 1.0,
        ..LineStyle::default()
    };
    inst.material.base_color = GREY;
    inst.pick = PickId(1);
    r.scene_mut().add_instance(inst)
}

/// Done-when 1: cursor within 3 px of a visible projected 1-px edge returns that edge
/// (≥ 99.5 %), more than 6 px from every edge and vertex returns no edge (≥ 99.5 %).
#[test]
fn gpu_edges_are_picked_within_three_pixels_and_not_beyond_six() {
    let Some(gpu) = gpu() else { return };
    let part = Part::new(Point3::ZERO);
    let cam = oblique(Point3::ZERO);
    let mut r = renderer(&gpu, 4, PickBackend::Auto);
    add_part(&mut r, &gpu, &part, true, false);
    let view = add_view(&mut r, cam, SIZE);
    let projected: Vec<[[f64; 2]; 2]> = part
        .edges
        .iter()
        .map(|e| [px_of(&cam, e[0], SIZE), px_of(&cam, e[1], SIZE)])
        .collect();
    let mut rng = Rng(0x1234_5678_9abc_def1);

    // Positive samples: a visible point on a random edge, the cursor within 3 px of it, and no
    // other edge within 1.5 px more than this one (no ambiguity).
    let mut near = Vec::new();
    while near.len() < 1000 {
        let e = (rng.next() * part.edges.len() as f64) as usize % part.edges.len();
        let t = 0.05 + 0.9 * rng.next();
        let p = part.edges[e][0] + (part.edges[e][1] - part.edges[e][0]) * t;
        let to = p - cam.eye;
        let ray = Ray3 {
            origin: cam.eye,
            dir: Dir3::new(to, &Tolerance::default()).unwrap(),
        };
        if part
            .cast(&ray)
            .is_some_and(|(d, _)| d < to.length() * (1.0 - 1e-6))
        {
            continue; // hidden
        }
        let c = px_of(&cam, p, SIZE);
        let (angle, radius) = (rng.next() * std::f64::consts::TAU, 3.0 * rng.next().sqrt());
        let cursor = [c[0] + radius * angle.cos(), c[1] + radius * angle.sin()];
        if cursor[0] < 6.0
            || cursor[1] < 6.0
            || cursor[0] > f64::from(SIZE[0]) - 6.0
            || cursor[1] > f64::from(SIZE[1]) - 6.0
        {
            continue;
        }
        let d = segment_distance(cursor, projected[e][0], projected[e][1]);
        let ambiguous = projected
            .iter()
            .enumerate()
            .any(|(i, s)| i != e && segment_distance(cursor, s[0], s[1]) < d + 1.5);
        if !ambiguous {
            near.push((cursor, e as u32));
        }
    }
    let requests: Vec<_> = near
        .iter()
        .map(|(c, _)| PickRequest::point([c[0] as f32, c[1] as f32]))
        .collect();
    let hits = points(&pick_all(&mut r, view, &requests));
    let correct = near
        .iter()
        .zip(&hits)
        .filter(|((_, e), h)| {
            h.first()
                .is_some_and(|h| h.kind == SubKind::Edge && h.sub == *e && h.pick == PickId(1))
        })
        .count();
    eprintln!("edges within 3 px: {correct} / {} correct", near.len());
    assert!(
        correct * 1000 >= 995 * near.len(),
        "{correct} / {}",
        near.len()
    );

    // Negative samples: more than 6 px from every projected edge and vertex.
    let corners: Vec<[f64; 2]> = part
        .vertices
        .iter()
        .map(|&v| px_of(&cam, v, SIZE))
        .collect();
    let mut far = Vec::new();
    while far.len() < 1000 {
        let c = [
            rng.next() * f64::from(SIZE[0]),
            rng.next() * f64::from(SIZE[1]),
        ];
        let edge_d = projected
            .iter()
            .map(|s| segment_distance(c, s[0], s[1]))
            .fold(f64::INFINITY, f64::min);
        let vertex_d = corners
            .iter()
            .map(|v| ((c[0] - v[0]).powi(2) + (c[1] - v[1]).powi(2)).sqrt())
            .fold(f64::INFINITY, f64::min);
        if edge_d > 6.0 && vertex_d > 6.0 {
            far.push(c);
        }
    }
    let requests: Vec<_> = far
        .iter()
        .map(|c| PickRequest::point([c[0] as f32, c[1] as f32]))
        .collect();
    let hits = points(&pick_all(&mut r, view, &requests));
    let clean = hits
        .iter()
        .filter(|h| h.iter().all(|h| h.kind != SubKind::Edge))
        .count();
    eprintln!(
        "cursor > 6 px away: {clean} / {} without an edge",
        far.len()
    );
    assert!(clean * 1000 >= 995 * far.len());
}

/// Random interior pixels (≥ 2 px from every projected edge of `edges`) where the ray hits the
/// part: (cursor, ray-cast triangle, distance).
fn interior_samples(
    part: &Part,
    cam: &Camera,
    edges: &[[Point3; 2]],
    n: usize,
    seed: u64,
) -> Vec<([f64; 2], u32, f64)> {
    let projected: Vec<[[f64; 2]; 2]> = edges
        .iter()
        .map(|e| [px_of(cam, e[0], SIZE), px_of(cam, e[1], SIZE)])
        .collect();
    let mut rng = Rng(seed);
    let mut out = Vec::new();
    while out.len() < n {
        let c = [
            rng.next() * f64::from(SIZE[0]),
            rng.next() * f64::from(SIZE[1]),
        ];
        if projected
            .iter()
            .any(|s| segment_distance(c, s[0], s[1]) < 2.0)
        {
            continue;
        }
        if let Some((d, t)) = part.cast(&cam.ray(c, SIZE)) {
            out.push((c, t, d));
        }
    }
    out
}

/// Every edge of every triangle of the part (mesh triangle tests stay off the diagonals).
fn triangle_edges(part: &Part) -> Vec<[Point3; 2]> {
    (0..part.indices.len() / 3)
        .flat_map(|t| {
            let [a, b, c] = part.triangle(t);
            [[a, b], [b, c], [c, a]]
        })
        .collect()
}

/// Relative error of the hit's world point against the analytic hit along the ray through the
/// cursor as requested (f32).
fn check_world(hit: &PickHit, part: &Part, cam: &Camera, cursor: [f64; 2]) -> f64 {
    let requested = [f64::from(cursor[0] as f32), f64::from(cursor[1] as f32)];
    let ray = cam.ray(requested, SIZE);
    let (distance, _) = part.cast(&ray).unwrap();
    let analytic = ray.origin + ray.dir.get() * distance;
    hit.world.distance(analytic) / analytic.distance(cam.eye)
}

/// Done-when 2 and 5: interior face pixels return the right B-rep face (100 %) with the world
/// point within 1e-6 × distance of the analytic hit, also far from the origin.
#[test]
fn gpu_interior_face_pixels_return_the_face_and_its_world_point() {
    let Some(gpu) = gpu() else { return };
    for origin in [Point3::ZERO, Point3::new(1e5, 1e5, 0.0)] {
        let part = Part::new(origin);
        let cam = oblique(origin);
        let mut r = renderer(&gpu, 4, PickBackend::Auto);
        add_part(&mut r, &gpu, &part, true, true);
        let view = add_view(&mut r, cam, SIZE);
        let samples = interior_samples(&part, &cam, &part.edges, 1000, 0xfeed_beef_1234_5678);
        let requests: Vec<_> = samples
            .iter()
            .map(|(c, _, _)| PickRequest {
                region: amg_render::PickRegion::Point {
                    px: [c[0] as f32, c[1] as f32],
                },
                filter: PickFilter::FACES,
            })
            .collect();
        let hits = points(&pick_all(&mut r, view, &requests));
        let mut worst = 0.0f64;
        for ((c, t, _), h) in samples.iter().zip(&hits) {
            let best = h.first().unwrap_or_else(|| panic!("nothing at {c:?}"));
            assert_eq!(best.kind, SubKind::Face);
            assert_eq!(best.sub, part.face_of(*t), "face at {c:?}");
            assert_eq!(best.pick, PickId(1));
            assert!(best.instance.is_some());
            worst = worst.max(check_world(best, &part, &cam, *c));
        }
        eprintln!("origin {origin:?}: worst world error {worst:.3e} x distance");
        assert!(worst <= 1e-6, "world error {worst:e} x distance");
    }
}

/// Done-when 2 and 3: plain-mesh triangle sub-ids are correct (100 %), and the
/// `PrimitiveIndex` and `VertexPulling` backends give identical results.
#[test]
fn gpu_mesh_triangle_ids_are_exact_and_both_backends_agree() {
    let Some(gpu) = gpu() else { return };
    let part = Part::new(Point3::ZERO);
    let cam = oblique(Point3::ZERO);
    let samples = interior_samples(
        &part,
        &cam,
        &triangle_edges(&part),
        1000,
        0x0bad_cafe_0042_0042,
    );
    // Faces only: the B-rep edges of the part would win within 4 px.
    let requests: Vec<_> = samples
        .iter()
        .map(|(c, _, _)| PickRequest {
            region: amg_render::PickRegion::Point {
                px: [c[0] as f32, c[1] as f32],
            },
            filter: PickFilter::FACES,
        })
        .collect();
    let mut results = Vec::new();
    for backend in [PickBackend::PrimitiveIndex, PickBackend::VertexPulling] {
        let mut r = renderer(&gpu, 4, backend);
        if backend == PickBackend::PrimitiveIndex && r.pick_backend() != backend {
            eprintln!("PRIMITIVE_INDEX unavailable: comparing vertex pulling with itself");
        }
        assert_eq!(
            r.pick_backend(),
            if backend == PickBackend::VertexPulling {
                backend
            } else {
                r.pick_backend()
            }
        );
        add_part(&mut r, &gpu, &part, false, true);
        let view = add_view(&mut r, cam, SIZE);
        let hits = points(&pick_all(&mut r, view, &requests));
        let mut worst = 0.0f64;
        for ((c, t, _), h) in samples.iter().zip(&hits) {
            let best = h.first().unwrap();
            assert_eq!(
                (best.kind, best.sub),
                (SubKind::Face, *t),
                "{backend:?} at {c:?}"
            );
            worst = worst.max(check_world(best, &part, &cam, *c));
        }
        eprintln!("{backend:?}: 1000 / 1000 triangles, worst world error {worst:.3e}");
        assert!(worst <= 1e-6);
        results.push(hits);
    }
    assert_eq!(
        results[0], results[1],
        "both backends give identical results"
    );
}

/// Done-when 4: a pick is answered within two `encode` calls of its request (counted on a
/// fresh renderer, also when the request comes before the view was ever drawn).
#[test]
fn gpu_pick_results_arrive_within_two_encodes() {
    let Some(gpu) = gpu() else { return };
    let part = Part::new(Point3::ZERO);
    let cam = oblique(Point3::ZERO);
    let mut r = renderer(&gpu, 4, PickBackend::Auto);
    add_part(&mut r, &gpu, &part, true, true);
    let view = r.create_view(SIZE);
    r.view_mut(view).unwrap().camera = cam;
    assert!(r.prepare_view(view));
    let centre = px_of(&cam, Point3::new(0.0, -35.0, H), SIZE);
    for round in 0..5 {
        let start = r.encode_count();
        let ticket = r.request_pick(
            view,
            PickRequest::point([centre[0] as f32, centre[1] as f32]),
        );
        let result = loop {
            r.capture(view).unwrap().wait().unwrap();
            if let Some((_, res)) = r.poll_picks().into_iter().find(|(t, _)| *t == ticket) {
                break res.unwrap();
            }
            assert!(r.encode_count() - start < 2, "no answer after two encodes");
        };
        let encodes = r.encode_count() - start;
        eprintln!("round {round}: answered after {encodes} encode(s)");
        assert!(encodes <= 2);
        let PickResult::Point(hits) = result else {
            panic!("point result")
        };
        assert_eq!((hits[0].kind, hits[0].sub), (SubKind::Face, 0), "top face");
    }
    assert_eq!(r.pending_picks(), 0);
    assert!(r.poll_picks().is_empty(), "each ticket is returned once");
}

/// Vertex beats edge beats face; the filter keeps edges from hiding the face under them.
#[test]
fn gpu_vertices_win_over_edges_and_filters_drop_kinds() {
    let Some(gpu) = gpu() else { return };
    let part = Part::new(Point3::ZERO);
    let cam = oblique(Point3::ZERO);
    let mut r = renderer(&gpu, 1, PickBackend::Auto);
    add_part(&mut r, &gpu, &part, true, true);
    let view = add_view(&mut r, cam, SIZE);
    // 3 px beside the front top outer corner (vertex 1), and the middle of edge 0.
    let corner = px_of(&cam, part.vertices[1], SIZE);
    let mid = px_of(&cam, Point3::new(0.0, -A, H), SIZE);
    let at = |p: [f64; 2], filter| PickRequest {
        region: amg_render::PickRegion::Point {
            px: [p[0] as f32, p[1] as f32],
        },
        filter,
    };
    let requests = [
        at([corner[0] - 3.0, corner[1]], PickFilter::ALL),
        at([mid[0], mid[1] + 1.0], PickFilter::ALL),
        at([mid[0], mid[1] - 1.0], PickFilter::FACES),
        at([mid[0], mid[1] + 1.0], PickFilter::EDGES),
    ];
    let hits = points(&pick_all(&mut r, view, &requests));
    assert_eq!((hits[0][0].kind, hits[0][0].sub), (SubKind::Vertex, 1));
    assert!(hits[0][0].distance_px <= 5.0);
    assert!(
        hits[0].iter().any(|h| h.kind == SubKind::Edge),
        "the edges at the corner are listed after the vertex"
    );
    assert_eq!((hits[1][0].kind, hits[1][0].sub), (SubKind::Edge, 0));
    // World point of the edge hit lies on edge 0 (y = -A, z = H) within float depth precision.
    let w = hits[1][0].world;
    assert!((w.y + A).abs() < 0.05 && (w.z - H).abs() < 0.05, "{w:?}");
    // Faces only: the face next to the edge, never the edge.
    assert!(hits[2].iter().all(|h| h.kind == SubKind::Face));
    assert!(matches!(hits[2][0].sub, 0 | 2));
    // Edges only: no faces reported.
    assert!(hits[3].iter().all(|h| h.kind == SubKind::Edge));
    // Background.
    let empty = points(&pick_all(&mut r, view, &[PickRequest::point([2.0, 2.0])]));
    assert!(empty[0].is_empty());
}

/// Done-when 6: a 3×3 grid of boxes seen from the top; a rectangle around the centre box that
/// cuts into its four side neighbours selects the centre in window mode and five in crossing
/// mode (bodies and faces).
#[test]
fn gpu_rectangle_window_and_crossing_on_a_box_grid() {
    let Some(gpu) = gpu() else { return };
    let mut r = renderer(&gpu, 4, PickBackend::Auto);
    let part = Part::new(Point3::ZERO);
    let ctx = JobCtx::detached();
    let mesh = r
        .scene_mut()
        .insert_mesh(upload_mesh_with_faces(&gpu, &part.mesh(), &part.faces, &ctx).unwrap());
    // Boxes 100 mm wide on a 150 mm pitch; pick ids 1..=9 row by row (row 0 at y = +150).
    for row in 0..3 {
        for col in 0..3 {
            let t = Transform::from_translation(Vec3::new(
                150.0 * f64::from(col - 1),
                150.0 * f64::from(1 - row),
                0.0,
            ));
            let mut inst = Instance::new(mesh, t);
            inst.pick = PickId(1 + 3 * row as u32 + col as u32);
            r.scene_mut().add_instance(inst);
        }
    }
    let cam = Camera {
        eye: Point3::new(0.0, 0.0, 500.0),
        target: Point3::ZERO,
        up: Dir3::Y,
        projection: Projection::Orthographic { height: 600.0 },
    };
    let size = [600, 600];
    let view = add_view(&mut r, cam, size);
    // 1 px = 1 mm; the centre box spans x, y in [-50, 50] -> px [250, 350], its left and right
    // neighbours px x [100, 200] and [400, 500]. The rectangle holds the centre box and reaches
    // 20 px into both side neighbours, but not up or down to the others.
    let rect = |mode, filter| PickRequest::rect([180, 220], [420, 380], mode, filter);
    let results = pick_all(
        &mut r,
        view,
        &[
            rect(RectMode::Window, PickFilter::BODIES),
            rect(RectMode::Crossing, PickFilter::BODIES),
            rect(RectMode::Window, PickFilter::FACES),
            rect(RectMode::Crossing, PickFilter::BODIES | PickFilter::FACES),
        ],
    );
    let sel = |i: usize| match &results[i] {
        PickResult::Rect(s) => s.clone(),
        PickResult::Point(_) => panic!("rect result"),
    };
    let bodies = |s: &[(PickId, SubKind, u32)]| -> Vec<u32> {
        s.iter()
            .filter(|e| e.2 == BODY_SUB)
            .map(|e| e.0.0)
            .collect()
    };
    assert_eq!(bodies(&sel(0)), [5], "window");
    assert_eq!(bodies(&sel(1)), [4, 5, 6], "crossing");
    // Seen from the top only the top faces (face 0) of the boxes are visible.
    assert_eq!(sel(2), [(PickId(5), SubKind::Face, 0)], "window, faces");
    let crossing = sel(3);
    assert_eq!(bodies(&crossing), [4, 5, 6]);
    let faces: Vec<_> = crossing.iter().filter(|e| e.2 != BODY_SUB).collect();
    assert_eq!(faces.len(), 3);
    assert!(faces.iter().all(|e| e.1 == SubKind::Face && e.2 == 0));
    // The whole view selects all nine in window mode.
    let all = pick_all(
        &mut r,
        view,
        &[PickRequest::rect(
            [0, 0],
            size,
            RectMode::Window,
            PickFilter::BODIES,
        )],
    );
    let PickResult::Rect(all) = &all[0] else {
        panic!("rect result")
    };
    assert_eq!(bodies(all), (1..=9).collect::<Vec<u32>>());
    // A rectangle entirely in the background selects nothing.
    let none = pick_all(
        &mut r,
        view,
        &[PickRequest::rect(
            [0, 0],
            [20, 20],
            RectMode::Crossing,
            PickFilter::ALL,
        )],
    );
    assert_eq!(none[0], PickResult::Rect(Vec::new()));
}

fn top_face_masks(part: &Part, cam: &Camera) -> (Vec<bool>, Vec<bool>) {
    // Pixels whose centre ray hits the top face (resp. another face) at least 2 px from every
    // projected edge.
    let projected: Vec<[[f64; 2]; 2]> = part
        .edges
        .iter()
        .map(|e| [px_of(cam, e[0], SIZE), px_of(cam, e[1], SIZE)])
        .collect();
    let mut top = vec![false; (SIZE[0] * SIZE[1]) as usize];
    let mut other = top.clone();
    for y in 0..SIZE[1] {
        for x in 0..SIZE[0] {
            let c = [f64::from(x) + 0.5, f64::from(y) + 0.5];
            if projected
                .iter()
                .any(|s| segment_distance(c, s[0], s[1]) < 2.0)
            {
                continue;
            }
            if let Some((_, t)) = part.cast(&cam.ray(c, SIZE)) {
                let i = (y * SIZE[0] + x) as usize;
                if part.face_of(t) == 0 {
                    top[i] = true;
                } else {
                    other[i] = true;
                }
            }
        }
    }
    (top, other)
}

fn max_diff(a: &RgbaImage, b: &RgbaImage, mask: &[bool]) -> (u8, usize) {
    let mut worst = 0u8;
    let mut over = 0;
    for (i, m) in mask.iter().enumerate() {
        if !m {
            continue;
        }
        let d = (0..3)
            .map(|k| a.pixels[4 * i + k].abs_diff(b.pixels[4 * i + k]))
            .max()
            .unwrap();
        worst = worst.max(d);
        over += usize::from(d > 1);
    }
    (worst, over)
}

/// Done-when 7: a highlighted face shows the highlight colour on every pixel (it equals the
/// part drawn with the highlight as base colour, no z-fighting), the other faces are unchanged,
/// at 1× and 4× MSAA; a whole-body highlight tints every face.
#[test]
fn gpu_highlighted_faces_show_the_highlight_colour_without_z_fighting() {
    let Some(gpu) = gpu() else { return };
    let part = Part::new(Point3::ZERO);
    let cam = oblique(Point3::ZERO);
    let (top, other) = top_face_masks(&part, &cam);
    eprintln!(
        "{} top-face pixels, {} other-face pixels",
        top.iter().filter(|&&m| m).count(),
        other.iter().filter(|&&m| m).count()
    );
    let selected = [0.05f32, 0.36, 1.0, 1.0];
    for samples in [1, 4] {
        let render = |base: [f32; 4], highlight: Option<HighlightItem>| {
            let mut r = renderer(&gpu, samples, PickBackend::Auto);
            let key = add_part(&mut r, &gpu, &part, true, false);
            r.scene_mut().instance_mut(key).unwrap().material.base_color = base;
            r.scene_mut()
                .set_highlight_color(HighlightLayer::Selected, selected);
            if let Some(item) = highlight {
                r.scene_mut()
                    .set_highlight_layer(HighlightLayer::Selected, &[item]);
            }
            let view = add_view(&mut r, cam, SIZE);
            capture(&mut r, view)
        };
        let plain = render(GREY, None);
        let reference = render(selected, None);
        let face = render(
            GREY,
            Some(HighlightItem {
                pick: PickId(1),
                sub: Some((SubKind::Face, Range32::one(0))),
            }),
        );
        let (worst_top, bad_top) = max_diff(&face, &reference, &top);
        let (worst_other, bad_other) = max_diff(&face, &plain, &other);
        let (differs, _) = max_diff(&plain, &reference, &top);
        eprintln!(
            "x{samples}: top face vs reference max {worst_top} ({bad_top} px > 1), other faces vs plain max {worst_other} ({bad_other} px > 1); plain vs reference {differs}"
        );
        assert!(differs > 20, "the highlight colour is distinguishable");
        assert_eq!(bad_top, 0, "every top-face pixel shows the highlight");
        assert_eq!(bad_other, 0, "other faces are unchanged");
        let body = render(
            GREY,
            Some(HighlightItem {
                pick: PickId(1),
                sub: None,
            }),
        );
        let all: Vec<bool> = top.iter().zip(&other).map(|(a, b)| *a || *b).collect();
        let (_, bad_body) = max_diff(&body, &reference, &all);
        assert_eq!(bad_body, 0, "a whole-body highlight tints every face");
    }
}

/// Edge highlights tint only the highlighted edge (state buffer by edge id).
#[test]
fn gpu_edge_highlight_tints_only_that_edge() {
    let Some(gpu) = gpu() else { return };
    let part = Part::new(Point3::ZERO);
    let cam = oblique(Point3::ZERO);
    let mut r = renderer(&gpu, 1, PickBackend::Auto);
    let key = add_part(&mut r, &gpu, &part, true, false);
    r.scene_mut()
        .set_highlight_color(HighlightLayer::Error, [1.0, 0.0, 0.0, 1.0]);
    let inst = r.scene_mut().instance_mut(key).unwrap();
    inst.edge_style.width_px = 3.0;
    inst.material.base_color = [0.0, 0.0, 0.6, 1.0];
    r.scene_mut().set_highlight_layer(
        HighlightLayer::Error,
        &[HighlightItem {
            pick: PickId(1),
            sub: Some((SubKind::Edge, Range32::one(0))),
        }],
    );
    let view = add_view(&mut r, cam, SIZE);
    let img = capture(&mut r, view);
    let red_at = |p: Point3| {
        let c = px_of(&cam, p, SIZE);
        let px = img.pixel(c[0] as u32, c[1] as u32);
        px[0] > 150 && px[1] < 60 && px[2] < 60
    };
    assert!(red_at(Point3::new(0.0, -A, H)), "edge 0 is red");
    assert!(!red_at(Point3::new(A, 0.0, H)), "edge 1 is not");
    assert!(!red_at(Point3::new(0.0, -B, H)), "edge 4 is not");
}

/// Done-when 9 (RND-06 Done-when 5): picking a clipped-away region returns the geometry behind
/// it; picking a cap returns `CAP_SUB` with the world point on the plane (≤ 1e-6 mm); an edge
/// under the cap is not picked through it.
#[test]
fn gpu_section_picks_see_through_clipped_parts_and_hit_caps() {
    let Some(gpu) = gpu() else { return };
    let ctx = JobCtx::detached();
    // Removes z > 0.
    let plane = SectionPlane::new(Plane::XY);
    for backend in [PickBackend::Auto, PickBackend::VertexPulling] {
        let mut r = renderer(&gpu, 4, backend);
        // A tilted open plate z = x (|x|, |y| <= 10, pick 1) above a plate at z = -20 (pick 2).
        let plate = |z: [f32; 2], half: f32| MeshData {
            origin: Point3::ZERO,
            positions: vec![
                [-half, -half, z[0]],
                [half, -half, z[1]],
                [half, half, z[1]],
                [-half, half, z[0]],
            ],
            normals: Vec::new(),
            indices: vec![0, 1, 2, 0, 2, 3],
            closed: false,
        };
        for (pick, data) in [
            (1, plate([-10.0, 10.0], 10.0)),
            (2, plate([-20.0, -20.0], 40.0)),
        ] {
            let mesh = r
                .scene_mut()
                .insert_mesh(upload_mesh(&gpu, &data, &ctx).unwrap());
            let mut inst = Instance::new(mesh, Transform::IDENTITY);
            inst.pick = PickId(pick);
            r.scene_mut().add_instance(inst);
        }
        // The box with a hole, cut through the middle (z = 0 after moving it down by H / 2),
        // far to the right; with edges and vertices.
        let part = Part::new(Point3::new(150.0, 0.0, -H / 2.0));
        add_part(&mut r, &gpu, &part, true, true);
        r.scene_mut().add_section(plane);
        let cam = Camera {
            eye: Point3::new(75.0, -40.0, 300.0),
            target: Point3::new(75.0, 0.0, 0.0),
            up: Dir3::Y,
            projection: Projection::Perspective {
                fov_y: 50f64.to_radians(),
            },
        };
        let view = add_view(&mut r, cam, SIZE);
        let at = |p: Point3| {
            let c = px_of(&cam, p, SIZE);
            PickRequest::point([c[0] as f32, c[1] as f32])
        };
        // The bottom outer edge 8 (y = -50, z = -15) lies under the cap as seen from the eye.
        let under = part.edges[8][0] + (part.edges[8][1] - part.edges[8][0]) * 0.5;
        let requests = [
            at(Point3::new(5.0, 0.0, 5.0)),   // clipped part of the tilted plate
            at(Point3::new(-5.0, 0.0, -5.0)), // kept part of it
            at(Point3::new(150.0 - 35.0, 0.0, 0.0)), // the cap of the box ring
            at(under),
        ];
        let hits = points(&pick_all(&mut r, view, &requests));
        assert_eq!(
            (hits[0][0].pick, hits[0][0].kind),
            (PickId(2), SubKind::Face),
            "{backend:?}: behind the clipped part"
        );
        assert!((hits[0][0].world.z + 20.0).abs() < 1e-3);
        assert_eq!(hits[1][0].pick, PickId(1), "kept part");
        for h in [&hits[2], &hits[3]] {
            let best = h[0];
            assert_eq!(
                (best.pick, best.kind, best.sub),
                (PickId(1), SubKind::Face, CAP_SUB),
                "{backend:?}: cap {h:?}"
            );
            assert!(best.world.z.abs() <= 1e-6, "on the plane: {:?}", best.world);
            assert!(
                h.iter().all(|x| x.kind != SubKind::Edge),
                "no edge through the cap: {h:?}"
            );
        }
        let ray = cam.ray(
            {
                let c = px_of(&cam, Point3::new(115.0, 0.0, 0.0), SIZE);
                [f64::from(c[0] as f32), f64::from(c[1] as f32)]
            },
            SIZE,
        );
        let t = -ray.origin.z / ray.dir.get().z;
        let expected = ray.origin + ray.dir.get() * t;
        assert!(hits[2][0].world.distance(expected) < 1e-6);
    }
}
