//! UI-22 tests: solid bodies through the scene adapter (BREP-08 tessellation, face and edge
//! sub-ids, quality presets, latest-wins re-tessellation).
//!
//! The placeholder test runs everywhere. Tests on real solids need the `occt` feature
//! (`cargo nextest run -p amg-ui --features occt`, AMG_OCCT_DIR). `gpu_*` tests drive a real
//! S10 renderer and skip without an adapter. `timing_*` tests print their measurement and
//! assert the budget only under `AMG_STRICT_TIMING=1`.

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::sync::{Arc, Mutex};

use amg_doc::stub::Shape;
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, Command, CreatedId, Dependency, Document, Feature,
    FeatureBox, FeatureError, FeatureId, FeatureOutput, RecomputeCtx,
};
use amg_jobs::{JobSystem, JobSystemConfig};
use amg_ui::scene::{CpuSceneBackend, SceneAdapter};
use serde::{Deserialize, Serialize};
use tracing_subscriber::layer::SubscriberExt as _;

// ---- helpers ------------------------------------------------------------------------------

fn solid_output(ctx: &mut RecomputeCtx<'_>, shape: Shape) -> FeatureOutput {
    FeatureOutput {
        bodies: vec![BodyChange {
            id: ctx.new_body_id(0),
            change: BodyChangeKind::Created {
                geom: BodyGeom::Solid(shape),
                naming: None,
            },
        }],
        ..FeatureOutput::default()
    }
}

fn add(doc: &mut Document, f: FeatureBox) -> FeatureId {
    let root = doc.design().root();
    let out = doc
        .apply(Command::AddFeature {
            at: None,
            feature: f,
            context: root,
            name: None,
        })
        .unwrap();
    match out.created[0] {
        CreatedId::Feature(id) => id,
        ref other => panic!("{other:?}"),
    }
}

fn jobs() -> JobSystem {
    JobSystem::new(JobSystemConfig::default()).unwrap()
}

fn cpu(scene: &SceneAdapter) -> &CpuSceneBackend {
    scene
        .backend()
        .as_any()
        .downcast_ref::<CpuSceneBackend>()
        .expect("CPU backend")
}

// ---- a solid without geometry (any build) --------------------------------------------------

/// A solid body with placeholder geometry: all a build without OCCT can produce.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Ui22Placeholder {
    tag: u32,
}

impl Feature for Ui22Placeholder {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Ok(solid_output(ctx, Shape::new()))
    }
}

amg_doc::register_feature!(Ui22Placeholder, "uitest.ui22_placeholder", version = 1);

/// Counts the "not shown" warnings.
#[derive(Clone, Default)]
struct Warnings(Arc<Mutex<u32>>);

impl<S: tracing::Subscriber> tracing_subscriber::Layer<S> for Warnings {
    fn on_event(
        &self,
        event: &tracing::Event<'_>,
        _ctx: tracing_subscriber::layer::Context<'_, S>,
    ) {
        struct Message(bool);
        impl tracing::field::Visit for Message {
            fn record_debug(&mut self, _: &tracing::field::Field, value: &dyn std::fmt::Debug) {
                self.0 |= format!("{value:?}").contains("solid bodies without a B-rep shape");
            }
        }
        let mut m = Message(false);
        event.record(&mut m);
        if m.0 && *event.metadata().level() == tracing::Level::WARN {
            *self.0.lock().unwrap() += 1;
        }
    }
}

#[test]
fn solids_without_a_shape_show_nothing_and_are_logged_once() {
    let warnings = Warnings::default();
    let subscriber = tracing_subscriber::registry().with(warnings.clone());
    let _guard = tracing::subscriber::set_default(subscriber);

    let jobs = jobs();
    let mut scene = SceneAdapter::new(
        Box::new(CpuSceneBackend::new()),
        amg_ui::UiRegistry::build().tessellators(),
    );
    let mut doc = Document::new("placeholders");
    for tag in 0..3 {
        add(&mut doc, FeatureBox::new(Ui22Placeholder { tag }));
    }
    doc.recompute_blocking(&jobs).unwrap();
    let events = doc.drain_events();
    scene.handle_events(&events, doc.design(), doc.evaluated(), &jobs);
    assert_eq!(scene.pending(), 0);
    scene.poll();
    assert_eq!(scene.bodies().count(), 3);
    assert_eq!(scene.stats().undisplayable, 3);
    assert_eq!(scene.stats().tessellations, 0);
    assert_eq!(scene.stats().failed, 0);
    assert!(scene.take_failures().is_empty());
    assert_eq!(
        (cpu(&scene).mesh_count(), cpu(&scene).instance_count()),
        (0, 0)
    );
    assert_eq!(*warnings.0.lock().unwrap(), 1, "logged once");
}

// ---- real solids ---------------------------------------------------------------------------

#[cfg(feature = "occt")]
mod solid {
    use std::collections::BTreeMap;
    use std::sync::OnceLock;
    use std::thread::ThreadId;
    use std::time::{Duration, Instant};

    use amg_brep::{BoolOp, BoolOpts, BrepKernel, KernelConfig, OcctKernel, SubKind};
    use amg_core::{Dir3, Plane, Point3, Vec3};
    use amg_doc::{BodyId, Expr, ParamId, TopoKind};
    use amg_jobs::JobCtx;
    use amg_jobs::gpu::{Gpu, GpuConfig};
    use amg_render::{
        Camera, PickFilter, PickRegion, PickRequest, PickResult, Projection, Renderer,
        RendererConfig,
    };
    use amg_ui::CommitBatch;
    use amg_ui::input::CameraView;
    use amg_ui::module::{Registrar, UiModule};
    use amg_ui::pick::{CpuPickService, PickMode, PickService, PickTarget};
    use amg_ui::scene::{
        BodyTessellator, BodyTints, DisplayGeometry, DisplayMesh, DisplayQuality, GpuSceneBackend,
        RenderSub, RenderSubKind, SolidTessellator, TessellateError,
    };
    use amg_ui::select::EntityFilter;
    use amg_ui::testing::AppHarness;

    use super::*;

    fn kernel() -> &'static OcctKernel {
        static K: OnceLock<OcctKernel> = OnceLock::new();
        K.get_or_init(|| OcctKernel::new(KernelConfig::default()))
    }

    fn strict_timing() -> bool {
        std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
    }

    // -- the spy: the framework's tessellator, recording where and how it ran -----------------

    type Call = (ThreadId, DisplayQuality, Duration);
    /// Calls by the shape's face count (tests run in parallel and use different parts).
    static CALLS: Mutex<Vec<(u32, Call)>> = Mutex::new(Vec::new());

    struct Spy;

    impl BodyTessellator for Spy {
        fn handles(&self, geom: &BodyGeom) -> bool {
            SolidTessellator.handles(geom)
        }

        fn depends_on_quality(&self) -> bool {
            SolidTessellator.depends_on_quality()
        }

        fn tessellate(
            &self,
            geom: &BodyGeom,
            quality: DisplayQuality,
            ctx: &JobCtx,
        ) -> Result<DisplayMesh, TessellateError> {
            self.tessellate_display(geom, quality, ctx).map(|g| g.mesh)
        }

        fn tessellate_display(
            &self,
            geom: &BodyGeom,
            quality: DisplayQuality,
            ctx: &JobCtx,
        ) -> Result<DisplayGeometry, TessellateError> {
            let start = Instant::now();
            let out = SolidTessellator.tessellate_display(geom, quality, ctx)?;
            let faces = out.mesh.face_count();
            let call = (std::thread::current().id(), quality, start.elapsed());
            CALLS.lock().unwrap().push((faces, call));
            Ok(out)
        }
    }

    fn calls_with_faces(faces: u32) -> Vec<Call> {
        let calls = CALLS.lock().unwrap();
        calls
            .iter()
            .filter(|(f, _)| *f == faces)
            .map(|(_, c)| *c)
            .collect()
    }

    inventory::submit! { UiModule { id: "zz_ui22_spy", register } }

    fn register(r: &mut Registrar<'_>) {
        r.tessellator(Spy);
    }

    // -- test features ------------------------------------------------------------------------

    const PITCH: f64 = 4.0;
    const RADIUS: f64 = 1.2;
    const THICKNESS: f64 = 10.0;

    /// A square plate with `n × n` round through holes, built from a BREP-14 box and cylinders
    /// by one BREP-04 cut: `n² + 6` faces, `12 + 3 n²` edges. (BREP-13, the STEP reader, does
    /// not exist yet.)
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    struct Ui22Plate {
        n: u32,
    }

    impl Feature for Ui22Plate {
        fn inputs(&self) -> Vec<Dependency> {
            Vec::new()
        }

        fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
            let k = kernel();
            let kernel_error = |e: amg_brep::KernelError| FeatureError::Kernel(e.to_string());
            let side = PITCH * f64::from(self.n);
            let slab = k
                .make_box(&Plane::XY, Vec3::new(side, side, THICKNESS))
                .map_err(kernel_error)?
                .shape;
            let mut holes = Vec::new();
            for i in 0..self.n {
                for j in 0..self.n {
                    let frame = Plane {
                        origin: Point3::new(
                            PITCH * (f64::from(i) + 0.5),
                            PITCH * (f64::from(j) + 0.5),
                            -1.0,
                        ),
                        ..Plane::XY
                    };
                    let c = k
                        .make_cylinder(&frame, RADIUS, THICKNESS + 2.0, std::f64::consts::TAU)
                        .map_err(kernel_error)?;
                    holes.push(c.shape);
                }
            }
            let cut = k
                .boolean(
                    BoolOp::Cut,
                    &[slab],
                    &holes,
                    &BoolOpts::default(),
                    &JobCtx::detached(),
                )
                .map_err(kernel_error)?;
            Ok(solid_output(ctx, Shape::from_brep(cut.shape)))
        }
    }

    amg_doc::register_feature!(Ui22Plate, "uitest.ui22_plate", version = 1);

    /// A box `size × 5 × 5` mm at `x0`: its length is a parameter expression.
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    struct Ui22ParamBox {
        size: Expr,
        x0: f64,
    }

    impl Feature for Ui22ParamBox {
        fn inputs(&self) -> Vec<Dependency> {
            Vec::new()
        }

        fn exprs(&self) -> Vec<&Expr> {
            vec![&self.size]
        }

        fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
            let length = ctx.eval_length(&self.size)?;
            let frame = Plane {
                origin: Point3::new(self.x0, 0.0, 0.0),
                ..Plane::XY
            };
            let b = kernel()
                .make_box(&frame, Vec3::new(length, 5.0, 5.0))
                .map_err(|e| FeatureError::Kernel(e.to_string()))?;
            Ok(solid_output(ctx, Shape::from_brep(b.shape)))
        }
    }

    amg_doc::register_feature!(Ui22ParamBox, "uitest.ui22_param_box", version = 1);

    /// A cylinder (r 10, h 20 mm).
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    struct Ui22Cylinder {
        x0: f64,
    }

    impl Feature for Ui22Cylinder {
        fn inputs(&self) -> Vec<Dependency> {
            Vec::new()
        }

        fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
            let frame = Plane {
                origin: Point3::new(self.x0, 0.0, 0.0),
                ..Plane::XY
            };
            let c = kernel()
                .make_cylinder(&frame, 10.0, 20.0, std::f64::consts::TAU)
                .map_err(|e| FeatureError::Kernel(e.to_string()))?;
            Ok(solid_output(ctx, Shape::from_brep(c.shape)))
        }
    }

    amg_doc::register_feature!(Ui22Cylinder, "uitest.ui22_cylinder", version = 1);

    // -- helpers ------------------------------------------------------------------------------

    fn adapter() -> SceneAdapter {
        SceneAdapter::new(
            Box::new(CpuSceneBackend::new()),
            amg_ui::UiRegistry::build().tessellators(),
        )
    }

    fn wait_idle(scene: &mut SceneAdapter) {
        let start = Instant::now();
        while scene.pending() > 0 {
            scene.poll();
            assert!(
                start.elapsed() < Duration::from_secs(300),
                "tessellation hangs"
            );
            std::thread::sleep(Duration::from_millis(1));
        }
        assert_eq!(scene.take_failures(), Vec::new());
    }

    fn show(doc: &mut Document, scene: &mut SceneAdapter, jobs: &JobSystem) {
        doc.recompute_blocking(jobs).unwrap();
        let events = doc.drain_events();
        scene.handle_events(&events, doc.design(), doc.evaluated(), jobs);
        wait_idle(scene);
    }

    fn only_body(doc: &Document) -> BodyId {
        let mut bodies = doc.evaluated().bodies().map(|(id, _)| id);
        let id = bodies.next().expect("a body");
        assert!(bodies.next().is_none());
        id
    }

    fn brep(doc: &Document, body: BodyId) -> amg_brep::Shape {
        match &doc.evaluated().body(body).unwrap().geom {
            BodyGeom::Solid(s) => s.brep().unwrap().clone(),
            other => panic!("{other:?}"),
        }
    }

    /// Vertex `v` of a display mesh, world mm (identity placement).
    fn vertex(m: &DisplayMesh, v: u32) -> Point3 {
        let p = m.data.positions[v as usize];
        m.data.origin + Point3::new(p[0].into(), p[1].into(), p[2].into())
    }

    fn corners(m: &DisplayMesh, tri: u32) -> [Point3; 3] {
        let i = tri as usize * 3;
        [0, 1, 2].map(|k| vertex(m, m.data.indices[i + k]))
    }

    /// The largest triangle of face `f`: its centroid and outward unit normal.
    fn face_sample(m: &DisplayMesh, f: u32) -> (Point3, Vec3) {
        let (c, n) = m.face_ranges[f as usize]
            .clone()
            .map(|t| {
                let [a, b, c] = corners(m, t);
                ((a + b + c) / 3.0, (b - a).cross(c - a))
            })
            .max_by(|x, y| x.1.length_squared().total_cmp(&y.1.length_squared()))
            .expect("the face has triangles");
        (c, n.normalize())
    }

    fn target(body: BodyId, kind: TopoKind, index: u32) -> PickTarget {
        PickTarget::Body {
            body,
            instance: 0,
            sub: Some((kind, index)),
        }
    }

    // -- Done-when 1: every face and edge of a >= 1000-face part is pickable ------------------

    #[test]
    fn a_1030_face_part_displays_with_every_face_and_edge_pickable() {
        let jobs = jobs();
        let mut scene = adapter();
        let mut doc = Document::new("plate");
        add(&mut doc, FeatureBox::new(Ui22Plate { n: 32 }));
        let start = Instant::now();
        show(&mut doc, &mut scene, &jobs);
        let shown = start.elapsed();
        let body = only_body(&doc);
        let shape = brep(&doc, body);
        let (faces, edges) = (shape.count(SubKind::Face), shape.count(SubKind::Edge));
        assert_eq!((faces, edges), (1030, 3084));

        let mesh = Arc::clone(scene.display_mesh(body).unwrap());
        let lines = Arc::clone(scene.display_edges(body).unwrap());
        assert_eq!(mesh.face_count(), faces);
        assert!(mesh.data.closed, "a solid is capped by section planes");
        assert!(mesh.face_ranges.iter().all(|r| !r.is_empty()));
        let slot = scene.mesh_slot(body).unwrap();
        assert_eq!(
            cpu(&scene).mesh_triangles(slot),
            Some(mesh.triangle_count())
        );
        assert_eq!(cpu(&scene).mesh_edges(slot), Some(edges));
        // One polyline per B-rep edge, with the edge index as its sub-id.
        let ids: Vec<u32> = lines.polylines.iter().map(|p| p.id).collect();
        assert_eq!(ids, (0..edges).collect::<Vec<_>>());
        assert!(lines.polylines.iter().all(|p| p.len >= 2));

        let key = scene.pick_map().body_key(body, 0).unwrap();
        // PickMap round trip, both directions, for every face and edge.
        for (kind, render, count) in [
            (TopoKind::Face, RenderSubKind::Face, faces),
            (TopoKind::Edge, RenderSubKind::Edge, edges),
        ] {
            for index in 0..count {
                let want = target(body, kind, index);
                let sub = RenderSub {
                    kind: render,
                    index,
                };
                assert_eq!(scene.pick_map().render(&want), Some((key, Some(sub))));
                assert_eq!(scene.pick_map().target(key, Some(sub)), Some(want));
            }
        }

        // Every face is hit by a ray at it, from just outside (hole walls included).
        let start = Instant::now();
        for f in 0..faces {
            let (c, n) = face_sample(&mesh, f);
            let ray = amg_core::Ray3 {
                origin: c + n * 1e-3,
                dir: Dir3::new_unchecked(-n),
            };
            let hits = CpuPickService::pick(&scene, &ray, PickMode::Nearest);
            assert_eq!(hits.len(), 1, "face {f}");
            assert_eq!(hits[0].target, target(body, TopoKind::Face, f), "face {f}");
        }
        let face_time = start.elapsed();

        // Every edge is picked through the pick service: a camera just above an adjacent
        // triangle, looking at a point next to the middle of one of the edge's segments.
        let mut by_corner: BTreeMap<[u64; 3], Vec<u32>> = BTreeMap::new();
        let bits = |p: Point3| p.to_array().map(f64::to_bits);
        for t in 0..mesh.triangle_count() {
            for p in corners(&mesh, t) {
                by_corner.entry(bits(p)).or_default().push(t);
            }
        }
        let rect = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(100.0, 100.0));
        let mut picks = CpuPickService::new();
        let start = Instant::now();
        for line in &lines.polylines {
            let point = |i: u32| {
                let p = lines.points[(line.first + i) as usize];
                lines.origin + Point3::new(p[0].into(), p[1].into(), p[2].into())
            };
            let i = (line.len - 1) / 2;
            let (a, b) = (point(i), point(i + 1));
            let (third, n) = by_corner[&bits(a)]
                .iter()
                .find_map(|&t| {
                    let c = corners(&mesh, t);
                    let has = |p: Point3| c.iter().any(|q| bits(*q) == bits(p));
                    if !has(b) {
                        return None;
                    }
                    let third = c
                        .into_iter()
                        .find(|q| bits(*q) != bits(a) && bits(*q) != bits(b))?;
                    let n = (c[1] - c[0]).cross(c[2] - c[0]);
                    (n.length_squared() > 0.0).then(|| (third, n.normalize()))
                })
                .unwrap_or_else(|| panic!("edge {} borders no triangle", line.id));
            let mid = (a + b) * 0.5;
            let inward = third - mid;
            let at = mid + inward.normalize() * inward.length().min(0.01) * 0.5;
            let up = (b - a).normalize();
            let view = CameraView::new(
                Camera {
                    eye: at + n * 0.01,
                    target: at,
                    up: Dir3::new_unchecked(up),
                    projection: Projection::Orthographic { height: 1.0 },
                },
                rect,
                1.0,
            );
            picks.set_view(&view);
            let ticket = PickService::request_point(
                &mut picks,
                rect.center(),
                4.0,
                EntityFilter::EDGE,
                PickMode::Nearest,
            );
            let hits = picks.poll(&scene, ticket).unwrap();
            assert_eq!(
                hits.first().map(|h| h.target),
                Some(target(body, TopoKind::Edge, line.id)),
                "edge {}",
                line.id
            );
        }
        eprintln!(
            "plate: {faces} faces, {edges} edges, {} triangles; recompute + display {shown:?}; \
             {faces} face picks {face_time:?}; {edges} edge picks {:?}",
            mesh.triangle_count(),
            start.elapsed()
        );
    }

    // -- Done-when 2: only changed bodies are tessellated again --------------------------------

    fn commit(h: &mut AppHarness, commands: Vec<Command>) {
        h.app_mut().submit(CommitBatch {
            label: "Edit".into(),
            commands,
        });
        h.drive_until_idle();
    }

    #[test]
    fn editing_an_upstream_parameter_tessellates_only_the_changed_bodies() {
        let mut h = AppHarness::new();
        commit(
            &mut h,
            vec![Command::AddParam {
                name: "w".into(),
                expr: "10 mm".into(),
                unit: None,
                comment: String::new(),
            }],
        );
        let w: ParamId = h
            .document()
            .design()
            .params()
            .iter()
            .map(|(id, _)| id)
            .next()
            .expect("the parameter");
        let root = h.document().design().root();
        let boxes = [
            Ui22ParamBox {
                size: Expr::Param(w),
                x0: 0.0,
            },
            Ui22ParamBox {
                size: Expr::Inline("7 mm".into()),
                x0: 40.0,
            },
            Ui22ParamBox {
                size: Expr::Param(w),
                x0: 80.0,
            },
        ];
        commit(
            &mut h,
            boxes
                .into_iter()
                .map(|b| Command::AddFeature {
                    at: None,
                    feature: FeatureBox::new(b),
                    context: root,
                    name: None,
                })
                .collect(),
        );
        let bodies: Vec<BodyId> = h
            .document()
            .evaluated()
            .bodies()
            .map(|(id, _)| id)
            .collect();
        assert_eq!(bodies.len(), 3);
        assert_eq!(h.scene().stats().tessellations, 3);
        let slots: Vec<_> = bodies.iter().map(|b| h.scene().mesh_slot(*b)).collect();
        let length = |h: &AppHarness, b: BodyId| {
            let m = h.scene().display_mesh(b).unwrap();
            let x: Vec<f32> = m.data.positions.iter().map(|p| p[0]).collect();
            let min = x.iter().copied().fold(f32::INFINITY, f32::min);
            x.iter().copied().fold(f32::NEG_INFINITY, f32::max) - min
        };
        assert!((length(&h, bodies[0]) - 10.0).abs() < 1e-4);

        commit(
            &mut h,
            vec![Command::SetParamExpr {
                id: w,
                expr: "25 mm".into(),
            }],
        );
        let stats = h.scene().stats();
        assert_eq!(stats.tessellations, 5, "the two bodies reading w, once");
        assert_eq!(stats.inserted, 5);
        let uploads: Vec<_> = bodies.iter().map(|b| h.scene().uploads_of(*b)).collect();
        assert_eq!(uploads, vec![2, 1, 2]);
        assert_ne!(h.scene().mesh_slot(bodies[0]), slots[0]);
        assert_eq!(h.scene().mesh_slot(bodies[1]), slots[1], "untouched");
        assert!((length(&h, bodies[0]) - 25.0).abs() < 1e-4);
        assert!((length(&h, bodies[1]) - 7.0).abs() < 1e-4);

        // Display properties and colours rewrite instances only (INS-10 tints reach solids).
        let tints: BodyTints = [(bodies[1], [1.0, 0.0, 0.0])].into_iter().collect();
        h.app_mut()
            .scene_mut()
            .set_body_tints(Some(Arc::new(tints)));
        let key = h.scene().pick_map().body_key(bodies[1], 0).unwrap();
        let (_, instance) = cpu(h.scene())
            .instances()
            .find(|(_, d)| d.key == key)
            .unwrap();
        assert_eq!(instance.material.base_color[..3], [1.0, 0.0, 0.0]);
        assert_eq!(h.scene().stats().tessellations, 5);
    }

    // -- quality presets and latest-wins -------------------------------------------------------

    #[test]
    fn quality_presets_retessellate_solids_latest_wins() {
        let jobs = jobs();
        let mut scene = adapter();
        let mut doc = Document::new("quality");
        add(&mut doc, FeatureBox::new(Ui22Cylinder { x0: 0.0 }));
        show(&mut doc, &mut scene, &jobs);
        let body = only_body(&doc);
        let triangles = |s: &SceneAdapter| s.display_mesh(body).unwrap().triangle_count();
        let normal = triangles(&scene);

        scene.change_quality(DisplayQuality::Normal, &jobs);
        assert_eq!(scene.stats().tessellations, 1, "unchanged quality");

        // Two changes before a poll: the first job is superseded, the old mesh stays drawn.
        let old = scene.mesh_slot(body);
        scene.change_quality(DisplayQuality::Preview, &jobs);
        scene.change_quality(DisplayQuality::Fine, &jobs);
        assert_eq!(scene.mesh_slot(body), old);
        assert_eq!(cpu(&scene).mesh_count(), 1);
        wait_idle(&mut scene);
        assert_eq!(scene.stats().tessellations, 3);
        assert_eq!(scene.stats().superseded, 1);
        assert_eq!(scene.uploads_of(body), 2);
        let fine = triangles(&scene);

        scene.change_quality(DisplayQuality::Preview, &jobs);
        wait_idle(&mut scene);
        let preview = triangles(&scene);
        eprintln!("cylinder triangles: preview {preview}, normal {normal}, fine {fine}");
        assert!(preview < normal && normal < fine);

        // Preview bodies (dialogs) are always tessellated at the preview preset.
        scene.change_quality(DisplayQuality::Fine, &jobs);
        wait_idle(&mut scene);
        let mut other = Document::new("preview");
        add(&mut other, FeatureBox::new(Ui22Cylinder { x0: 50.0 }));
        other.recompute_blocking(&jobs).unwrap();
        let committed = Document::new("empty");
        let evaluated = Arc::clone(other.evaluated());
        let before = calls_with_faces(3).len();
        scene.set_preview(
            Some(evaluated),
            other.design(),
            committed.evaluated(),
            &jobs,
        );
        wait_idle(&mut scene);
        let calls = calls_with_faces(3);
        assert_eq!(calls.len(), before + 1);
        assert_eq!(calls.last().unwrap().1, DisplayQuality::Preview);
        assert_eq!(scene.preview_bodies().count(), 1);
        assert_eq!(cpu(&scene).mesh_count(), 2);
    }

    // -- Done-when 3: tessellation runs on a job -----------------------------------------------

    /// Records how long each `amg_ui.frame` span was entered (the UI thread's frame time).
    #[derive(Clone, Default)]
    struct FrameTimes(Arc<Mutex<Vec<Duration>>>);

    struct Entered(Instant);

    impl<S> tracing_subscriber::Layer<S> for FrameTimes
    where
        S: tracing::Subscriber + for<'a> tracing_subscriber::registry::LookupSpan<'a>,
    {
        fn on_enter(&self, id: &tracing::span::Id, ctx: tracing_subscriber::layer::Context<'_, S>) {
            if let Some(span) = ctx.span(id)
                && span.name() == "amg_ui.frame"
            {
                span.extensions_mut().replace(Entered(Instant::now()));
            }
        }

        fn on_exit(&self, id: &tracing::span::Id, ctx: tracing_subscriber::layer::Context<'_, S>) {
            if let Some(span) = ctx.span(id)
                && let Some(Entered(t)) = span.extensions().get::<Entered>()
            {
                self.0.lock().unwrap().push(t.elapsed());
            }
        }
    }

    #[test]
    fn timing_tessellating_a_large_solid_runs_on_a_job_without_slow_frames() {
        let times = FrameTimes::default();
        let subscriber = tracing_subscriber::registry().with(times.clone());
        let _guard = tracing::subscriber::set_default(subscriber);

        // 31 x 31 holes: 967 faces, a part of its own for the spy's records.
        const N: u32 = 31;
        const FACES: u32 = N * N + 6;
        let mut h = AppHarness::new();
        let root = h.document().design().root();
        h.app_mut().submit(CommitBatch {
            label: "Add".into(),
            commands: vec![Command::AddFeature {
                at: None,
                feature: FeatureBox::new(Ui22Plate { n: N }),
                context: root,
                name: None,
            }],
        });
        let start = Instant::now();
        let mut tessellating = 0u32;
        let mut measured_from = None;
        loop {
            h.kittest().step();
            if h.scene().pending() > 0 {
                tessellating += 1;
                // Frames from the tessellation's start on; the recompute before it is DOC's.
                measured_from.get_or_insert(times.0.lock().unwrap().len().saturating_sub(1));
            }
            if h.scene().stats().inserted == 1 && h.scene().pending() == 0 {
                break;
            }
            assert_eq!(h.scene().stats().failed, 0);
            assert!(
                start.elapsed() < Duration::from_secs(300),
                "the solid never appeared"
            );
            std::thread::sleep(Duration::from_millis(2));
        }
        // The frame that draws the new mesh.
        h.kittest().step();

        let calls = calls_with_faces(FACES);
        assert_eq!(calls.len(), 1);
        let (thread, quality, took) = calls[0];
        assert_ne!(thread, std::thread::current().id(), "not on the UI thread");
        assert_eq!(quality, DisplayQuality::Normal);

        let mut frames = times.0.lock().unwrap()[measured_from.expect("a job ran")..].to_vec();
        frames.sort();
        let max = *frames.last().unwrap();
        let p50 = frames[frames.len() / 2];
        println!(
            "{FACES}-face solid tessellated on a job in {took:?}; {} frames ({tessellating} \
             while pending): UI-thread frame p50 {p50:?}, max {max:?}",
            frames.len()
        );
        assert!(tessellating > 1, "frames ran while the job was in flight");
        if strict_timing() {
            assert!(max < Duration::from_millis(16), "a frame took {max:?}");
        }
    }

    // -- the real renderer -----------------------------------------------------------------------

    #[test]
    fn gpu_picks_report_brep_face_and_edge_indices() {
        if !amg_testkit::gpu_available() {
            eprintln!("no GPU adapter: skipped");
            return;
        }
        let gpu = Gpu::new_blocking(&GpuConfig::from_env()).expect("adapter probed, device opens");
        let renderer = Arc::new(Mutex::new(
            Renderer::new(gpu.clone(), &RendererConfig::default()).unwrap(),
        ));
        let access = Arc::clone(&renderer);
        let backend = GpuSceneBackend::new(
            gpu,
            Box::new(move |f| f(access.lock().unwrap().scene_mut())),
        );
        let mut scene = SceneAdapter::new(
            Box::new(backend),
            amg_ui::UiRegistry::build().tessellators(),
        );
        let jobs = jobs();
        let mut doc = Document::new("gpu solid");
        add(
            &mut doc,
            FeatureBox::new(Ui22ParamBox {
                size: Expr::Inline("5 mm".into()),
                x0: 0.0,
            }),
        );
        show(&mut doc, &mut scene, &jobs);
        let body = only_body(&doc);
        let mesh = Arc::clone(scene.display_mesh(body).unwrap());
        let lines = Arc::clone(scene.display_edges(body).unwrap());
        let key = scene.pick_map().body_key(body, 0).unwrap();
        {
            let r = renderer.lock().unwrap();
            let s = r.scene();
            assert_eq!(
                (s.mesh_count(), s.edges_count(), s.instance_count()),
                (1, 1, 1)
            );
        }

        // An orthographic view along (-1, -1, -1): three faces and nine edges face the eye.
        let centre = Point3::new(2.5, 2.5, 2.5);
        let toward_eye = Vec3::ONE.normalize();
        let camera = Camera {
            eye: centre + toward_eye * 100.0,
            target: centre,
            up: Dir3::Z,
            projection: Projection::Orthographic { height: 12.0 },
        };
        let size = [600, 600];
        let mut r = renderer.lock().unwrap();
        let view = r.create_view(size);
        r.view_mut(view).unwrap().camera = camera;
        assert!(r.prepare_view(view));
        let mut pick = |p: Point3, filter: PickFilter| {
            let px = camera.project(p, size).unwrap();
            #[allow(clippy::cast_possible_truncation, reason = "S10 takes f32 pixels")]
            let ticket = r.request_pick(
                view,
                PickRequest {
                    region: PickRegion::Point {
                        px: [px[0] as f32, px[1] as f32],
                    },
                    filter,
                },
            );
            for _ in 0..4 {
                r.capture(view).unwrap().wait().unwrap();
                for (t, result) in r.poll_picks() {
                    if t == ticket {
                        let PickResult::Point(hits) = result.unwrap() else {
                            panic!("a point pick");
                        };
                        let hit = hits.first().expect("a hit");
                        assert_eq!(hit.pick, key.pick_id());
                        return (hit.kind, hit.sub);
                    }
                }
            }
            panic!("the pick was not answered");
        };

        let mut faces = 0;
        for f in 0..mesh.face_count() {
            let range = mesh.face_ranges[f as usize].clone();
            let (_, n) = face_sample(&mesh, f);
            if n.dot(toward_eye) <= 0.0 {
                continue;
            }
            let sum: Vec3 = range.clone().flat_map(|t| corners(&mesh, t)).sum();
            let middle = sum / f64::from((range.end - range.start) * 3);
            assert_eq!(
                pick(middle, PickFilter::FACES),
                (amg_render::SubKind::Face, f)
            );
            faces += 1;
        }
        assert_eq!(faces, 3);

        // The visible edges: not touching the hidden corner (the box's minimum corner).
        let mut picked = 0;
        for line in &lines.polylines {
            let ends = [line.first, line.first + line.len - 1].map(|i| {
                let p = lines.points[i as usize];
                lines.origin + Point3::new(p[0].into(), p[1].into(), p[2].into())
            });
            if ends.iter().any(|p| p.length() < 1e-6) {
                continue;
            }
            // Off the middle: no other edge projects onto this point.
            let at = ends[0].lerp(ends[1], 0.3);
            assert_eq!(
                pick(at, PickFilter::EDGES),
                (amg_render::SubKind::Edge, line.id)
            );
            picked += 1;
        }
        assert_eq!(picked, 9);
        drop(r);
        scene.clear();
        let r = renderer.lock().unwrap();
        let s = r.scene();
        assert_eq!(
            (s.mesh_count(), s.edges_count(), s.instance_count()),
            (0, 0, 0)
        );
        assert_eq!(s.gpu_bytes(), 0);
    }
}
