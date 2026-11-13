//! UI-20 tests: the scene adapter (`DocEvent` → render scene), key allocation, `PickMap`,
//! the tessellator registry, highlight and preview layers and `CpuPickService`.
//!
//! App-level tests go through `AppHarness` (default `CpuSceneBackend`, no GPU). `gpu_*` tests
//! drive a real `RenderScene` and skip without an adapter. `timing_*` tests print their
//! measurement and assert the budget only under `AMG_STRICT_TIMING=1`.

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::collections::BTreeSet;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use amg_core::{Dir3, Point3, Ray3, Vec3};
use amg_doc::stub::{IndexedMesh, SdfGraph};
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, BodyProps, Command, CreatedId, Dependency,
    Document, EvalBody, Evaluated, FeatureBox, FeatureError, FeatureId, FeatureOutput,
    RecomputeCtx, TopoKind,
};
use amg_jobs::gpu::{Gpu, GpuConfig};
use amg_jobs::{JobCtx, JobSystem, JobSystemConfig};
use amg_render::{Layer, MeshData, RenderScene};
use amg_ui::CommitBatch;
use amg_ui::module::{Registrar, UiModule};
use amg_ui::pick::{PickMode, PickTarget};
use amg_ui::scene::{
    BodyTessellator, CpuSceneBackend, DisplayMesh, DisplayQuality, GpuSceneBackend, HighlightItem,
    HighlightLayer, Range32, RenderSub, RenderSubKind, SceneAdapter, TessellateError,
};
use amg_ui::testing::AppHarness;
use serde::{Deserialize, Serialize};

// ---- test features -----------------------------------------------------------------------

/// A flat `nx × ny` grid of `2 nx ny` triangles at height `z`, cell size 1 mm, from `x0`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct UiGrid {
    nx: u32,
    ny: u32,
    x0: f64,
    z: f64,
}

fn grid_mesh(g: &UiGrid) -> IndexedMesh {
    let mut pos = Vec::new();
    for j in 0..=g.ny {
        for i in 0..=g.nx {
            pos.push(Point3::new(g.x0 + f64::from(i), f64::from(j), g.z));
        }
    }
    let w = g.nx + 1;
    let mut tris = Vec::new();
    for j in 0..g.ny {
        for i in 0..g.nx {
            let a = j * w + i;
            tris.push([a, a + 1, a + w + 1]);
            tris.push([a, a + w + 1, a + w]);
        }
    }
    IndexedMesh::new(pos, tris).expect("valid grid")
}

impl amg_doc::Feature for UiGrid {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Ok(FeatureOutput {
            bodies: vec![BodyChange {
                id: ctx.new_body_id(0),
                change: BodyChangeKind::Created {
                    geom: BodyGeom::Mesh(Arc::new(grid_mesh(self))),
                    naming: None,
                },
            }],
            ..FeatureOutput::default()
        })
    }
}

amg_doc::register_feature!(UiGrid, "uitest.grid", version = 1);

/// An implicit body (the `SdfGraph` stub), displayed by [`CubeTessellator`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct UiImplicit {
    tag: u32,
}

impl amg_doc::Feature for UiImplicit {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Ok(FeatureOutput {
            bodies: vec![BodyChange {
                id: ctx.new_body_id(0),
                change: BodyChangeKind::Created {
                    geom: BodyGeom::Implicit(Arc::new(SdfGraph::default())),
                    naming: None,
                },
            }],
            ..FeatureOutput::default()
        })
    }
}

amg_doc::register_feature!(UiImplicit, "uitest.implicit", version = 1);

// ---- test tessellator (registered through inventory, no framework edit) -------------------

static CUBE_CALLS: AtomicU64 = AtomicU64::new(0);
/// While true, the cube tessellator waits (cancellable) before returning.
static CUBE_GATE: Mutex<bool> = Mutex::new(false);

struct CubeTessellator;

impl BodyTessellator for CubeTessellator {
    fn handles(&self, geom: &BodyGeom) -> bool {
        matches!(geom, BodyGeom::Implicit(_))
    }

    fn tessellate(
        &self,
        _geom: &BodyGeom,
        _quality: DisplayQuality,
        ctx: &JobCtx,
    ) -> Result<DisplayMesh, TessellateError> {
        CUBE_CALLS.fetch_add(1, Ordering::SeqCst);
        while *CUBE_GATE.lock().unwrap() {
            ctx.checkpoint()?;
            std::thread::sleep(Duration::from_millis(1));
        }
        let m = amg_testkit::shapes::cube(Point3::splat(-0.5), 1.0);
        Ok(DisplayMesh {
            data: MeshData {
                origin: Point3::ZERO,
                positions: m.positions.iter().map(|p| p.as_vec3().to_array()).collect(),
                normals: Vec::new(),
                indices: m.triangles.iter().flatten().copied().collect(),
                closed: true,
            },
            face_ranges: Vec::new(),
        })
    }
}

inventory::submit! { UiModule { id: "zz_scene_test", register } }

fn register(r: &mut Registrar<'_>) {
    r.tessellator(CubeTessellator);
}

// ---- helpers ------------------------------------------------------------------------------

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

fn grid(nx: u32, ny: u32, x0: f64, z: f64) -> FeatureBox {
    FeatureBox::new(UiGrid { nx, ny, x0, z })
}

fn add_batch(h: &mut AppHarness, features: Vec<FeatureBox>) {
    let root = h.document().design().root();
    h.app_mut().submit(CommitBatch {
        label: "Add".into(),
        commands: features
            .into_iter()
            .map(|f| Command::AddFeature {
                at: None,
                feature: f,
                context: root,
                name: None,
            })
            .collect(),
    });
    h.drive_until_idle();
}

fn commit(h: &mut AppHarness, cmd: Command) {
    h.app_mut().submit(CommitBatch {
        label: "Edit".into(),
        commands: vec![cmd],
    });
    h.drive_until_idle();
}

fn cpu(scene: &SceneAdapter) -> &CpuSceneBackend {
    scene
        .backend()
        .as_any()
        .downcast_ref::<CpuSceneBackend>()
        .expect("default backend")
}

fn live_bodies(doc: &Document) -> BTreeSet<BodyId> {
    doc.evaluated().bodies().map(|(id, _)| id).collect()
}

/// The scene holds exactly one instance per live body, every instance's key is mapped to
/// that body, and every mesh is used.
fn assert_scene_matches(h: &AppHarness) {
    let scene = h.scene();
    let live = live_bodies(h.document());
    assert_eq!(scene.bodies().collect::<BTreeSet<_>>(), live);
    let backend = cpu(scene);
    assert_eq!(backend.instance_count(), live.len());
    assert_eq!(backend.mesh_count(), live.len());
    let mut seen = BTreeSet::new();
    for (_, inst) in backend.instances() {
        let target = scene.pick_map().target(inst.key, None).expect("mapped key");
        let body = target.body().expect("a body");
        assert!(live.contains(&body), "{body:?} is not live");
        assert!(seen.insert(body), "{body:?} has two instances");
        assert_eq!(Some(inst.mesh), scene.mesh_slot(body));
        assert_eq!(inst.layer, Layer::Model);
    }
}

fn down_ray(x: f64, y: f64) -> Ray3 {
    Ray3 {
        origin: Point3::new(x, y, 100.0),
        dir: Dir3::NEG_Z,
    }
}

fn wait_idle(scene: &mut SceneAdapter) {
    let start = Instant::now();
    while scene.pending() > 0 {
        scene.poll();
        assert!(
            start.elapsed() < Duration::from_secs(20),
            "tessellation hangs"
        );
        std::thread::sleep(Duration::from_micros(200));
    }
}

fn jobs() -> JobSystem {
    JobSystem::new(JobSystemConfig::default()).unwrap()
}

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}

// ---- Done-when 1: add / edit / delete ------------------------------------------------------

#[test]
fn add_edit_delete_three_mesh_bodies_keeps_the_scene_exact_and_skips_unchanged_bodies() {
    let mut h = AppHarness::new();
    add_batch(
        &mut h,
        vec![
            grid(4, 4, 0.0, 0.0),
            grid(4, 4, 10.0, 0.0),
            grid(4, 4, 20.0, 0.0),
        ],
    );
    let bodies: Vec<BodyId> = live_bodies(h.document()).into_iter().collect();
    assert_eq!(bodies.len(), 3);
    assert_scene_matches(&h);
    assert_eq!(cpu(h.scene()).uploads(), 3);
    for b in &bodies {
        assert_eq!(h.scene().uploads_of(*b), 1);
        assert_eq!(
            cpu(h.scene()).mesh_triangles(h.scene().mesh_slot(*b).unwrap()),
            Some(32)
        );
    }

    // Edit the second body's feature: only it is re-uploaded, its old mesh is removed.
    let old_slot = h.scene().mesh_slot(bodies[1]).unwrap();
    let old_key = h.scene().pick_map().body_key(bodies[1], 0).unwrap();
    commit(
        &mut h,
        Command::EditFeature {
            id: bodies[1].origin,
            feature: grid(4, 2, 10.0, 3.0),
        },
    );
    assert_scene_matches(&h);
    assert_eq!(cpu(h.scene()).uploads(), 4);
    assert_eq!(h.scene().uploads_of(bodies[0]), 1);
    assert_eq!(h.scene().uploads_of(bodies[1]), 2);
    assert_eq!(h.scene().uploads_of(bodies[2]), 1);
    let new_slot = h.scene().mesh_slot(bodies[1]).unwrap();
    assert_ne!(new_slot, old_slot);
    assert_eq!(cpu(h.scene()).mesh_triangles(old_slot), None);
    assert_eq!(cpu(h.scene()).mesh_triangles(new_slot), Some(16));
    // The instance (and its pick key) survives a geometry change.
    assert_eq!(h.scene().pick_map().body_key(bodies[1], 0), Some(old_key));

    // Properties: instance only, no upload.
    commit(
        &mut h,
        Command::SetBodyProps {
            body: bodies[2],
            props: BodyProps {
                opacity: 0.5,
                ..BodyProps::default()
            },
        },
    );
    assert_eq!(cpu(h.scene()).uploads(), 4);
    let key2 = h.scene().pick_map().body_key(bodies[2], 0).unwrap();
    let inst = cpu(h.scene())
        .instances()
        .find(|(_, d)| d.key == key2)
        .unwrap()
        .1
        .clone();
    assert!((inst.material.base_color[3] - 0.5).abs() < 1e-6);
    commit(
        &mut h,
        Command::SetBodyProps {
            body: bodies[2],
            props: BodyProps {
                visible: false,
                ..BodyProps::default()
            },
        },
    );
    assert_eq!(cpu(h.scene()).uploads(), 4);
    assert!(
        !cpu(h.scene())
            .instances()
            .find(|(_, d)| d.key == key2)
            .unwrap()
            .1
            .visible
    );

    // Delete the first body.
    commit(
        &mut h,
        Command::DeleteFeatures {
            ids: vec![bodies[0].origin],
        },
    );
    assert_eq!(live_bodies(h.document()).len(), 2);
    assert_scene_matches(&h);
    assert_eq!(cpu(h.scene()).uploads(), 4);
    assert!(h.scene().pick_map().body_key(bodies[0], 0).is_none());

    // Undo brings it back as a new upload with a new key (keys are never reused).
    let before = h.scene().last_key();
    let doc = h.app().active_document().id;
    h.app_mut().request_undo(doc);
    h.drive_until_idle();
    assert_scene_matches(&h);
    assert_eq!(cpu(h.scene()).uploads(), 5);
    assert!(h.scene().pick_map().body_key(bodies[0], 0).unwrap() > before);
}

#[test]
fn gpu_scene_adapter_drives_a_real_render_scene() {
    if !amg_testkit::gpu_available() {
        eprintln!("no GPU adapter: skipped");
        return;
    }
    let gpu = Gpu::new_blocking(&GpuConfig::from_env()).expect("adapter probed, device opens");
    let rs = Arc::new(Mutex::new(RenderScene::default()));
    let backend = GpuSceneBackend::shared(Arc::clone(&gpu), Arc::clone(&rs));
    let mut scene = SceneAdapter::new(
        Box::new(backend),
        amg_ui::UiRegistry::build().tessellators(),
    );
    let jobs = jobs();
    let mut doc = Document::new("gpu");
    let f: Vec<FeatureId> = (0..3)
        .map(|i| add(&mut doc, grid(8, 8, f64::from(i) * 10.0, 0.0)))
        .collect();
    let step = |doc: &mut Document, scene: &mut SceneAdapter| {
        doc.recompute_blocking(&jobs).unwrap();
        let ev = doc.drain_events();
        scene.handle_events(&ev, doc.design(), doc.evaluated(), &jobs);
        wait_idle(scene);
    };
    step(&mut doc, &mut scene);
    let uploads = |s: &SceneAdapter| {
        s.backend()
            .as_any()
            .downcast_ref::<GpuSceneBackend>()
            .unwrap()
            .uploads()
    };
    {
        let r = rs.lock().unwrap();
        assert_eq!(r.instance_count(), 3);
        assert_eq!(r.mesh_count(), 3);
    }
    assert_eq!(uploads(&scene), 3);
    let one_mesh = rs.lock().unwrap().gpu_bytes() / 3;

    doc.apply(Command::EditFeature {
        id: f[1],
        feature: grid(8, 4, 10.0, 2.0),
    })
    .unwrap();
    step(&mut doc, &mut scene);
    {
        let r = rs.lock().unwrap();
        assert_eq!(r.instance_count(), 3);
        assert_eq!(r.mesh_count(), 3, "the old mesh was removed after the swap");
    }
    assert_eq!(uploads(&scene), 4);

    doc.apply(Command::DeleteFeatures { ids: vec![f[0]] })
        .unwrap();
    step(&mut doc, &mut scene);
    {
        let r = rs.lock().unwrap();
        assert_eq!(r.instance_count(), 2);
        assert_eq!(r.mesh_count(), 2);
        assert!(r.gpu_bytes() < 3 * one_mesh);
    }
    scene.clear();
    let r = rs.lock().unwrap();
    assert_eq!(
        (r.instance_count(), r.mesh_count(), r.gpu_bytes()),
        (0, 0, 0)
    );
}

// ---- Done-when 2: DocumentReplaced -------------------------------------------------------

#[test]
fn document_replaced_rebuilds_the_scene_with_fresh_keys() {
    let mut h = AppHarness::new();
    add_batch(
        &mut h,
        vec![
            grid(2, 2, 0.0, 0.0),
            grid(2, 2, 5.0, 0.0),
            grid(2, 2, 9.0, 0.0),
        ],
    );
    assert_scene_matches(&h);
    let old_keys: BTreeSet<_> = h.scene().pick_map().keys().into_iter().collect();
    let old_max = h.scene().last_key();
    let rebuilds = h.scene().stats().rebuilds;

    // Re-open the same design: the new document starts with `DocumentReplaced`.
    let design = h.document().design().clone();
    h.app_mut().active_document_mut().document = Document::from_design(design);
    h.drive_until_idle();

    assert_eq!(h.scene().stats().rebuilds, rebuilds + 1);
    assert_scene_matches(&h);
    assert_eq!(cpu(h.scene()).uploads(), 6, "everything uploaded again");
    let new_keys: BTreeSet<_> = h.scene().pick_map().keys().into_iter().collect();
    assert_eq!(new_keys.len(), 3);
    assert!(old_keys.is_disjoint(&new_keys));
    assert!(new_keys.iter().all(|k| *k > old_max));
}

#[test]
fn switching_document_tabs_shows_the_active_document_only() {
    let mut h = AppHarness::new();
    add_batch(&mut h, vec![grid(2, 2, 0.0, 0.0), grid(2, 2, 5.0, 0.0)]);
    let first = h.app().active_document().id;
    h.app_mut().new_document();
    h.drive_until_idle();
    assert_eq!(cpu(h.scene()).instance_count(), 0);
    add_batch(&mut h, vec![grid(3, 3, 0.0, 0.0)]);
    assert_scene_matches(&h);
    assert_eq!(cpu(h.scene()).instance_count(), 1);
    h.app_mut().activate(first);
    h.drive_until_idle();
    assert_scene_matches(&h);
    assert_eq!(cpu(h.scene()).instance_count(), 2);
}

#[test]
fn document_replaced_event_rebuilds_without_the_app() {
    let jobs = jobs();
    let mut scene = SceneAdapter::new(
        Box::new(CpuSceneBackend::new()),
        amg_ui::UiRegistry::build().tessellators(),
    );
    let mut doc = Document::new("a");
    add(&mut doc, grid(2, 2, 0.0, 0.0));
    doc.recompute_blocking(&jobs).unwrap();
    let ev = doc.drain_events();
    assert!(ev.contains(&amg_doc::DocEvent::DocumentReplaced));
    scene.handle_events(&ev, doc.design(), doc.evaluated(), &jobs);
    wait_idle(&mut scene);
    let k1 = scene.pick_map().keys();
    scene.handle_events(
        &[amg_doc::DocEvent::DocumentReplaced],
        doc.design(),
        doc.evaluated(),
        &jobs,
    );
    wait_idle(&mut scene);
    let k2 = scene.pick_map().keys();
    assert_eq!((k1.len(), k2.len()), (1, 1));
    assert!(k2[0] > k1[0]);
    assert_eq!(cpu(&scene).instance_count(), 1);
    assert_eq!(cpu(&scene).mesh_count(), 1);
}

// ---- Done-when 3: PickMap round trip over a 10 k-triangle body ----------------------------

#[test]
fn pick_map_round_trips_every_face_of_a_10k_triangle_body() {
    let mut h = AppHarness::new();
    add_batch(&mut h, vec![grid(100, 50, 0.0, 5.0)]);
    let body = *live_bodies(h.document()).first().unwrap();
    let mesh = Arc::clone(h.scene().display_mesh(body).unwrap());
    assert_eq!(mesh.triangle_count(), 10_000);
    let key = h.scene().pick_map().body_key(body, 0).unwrap();
    let start = Instant::now();
    for tri in 0..10_000u32 {
        let idx = &mesh.data.indices[tri as usize * 3..tri as usize * 3 + 3];
        let c = idx.iter().fold(Vec3::ZERO, |acc, &v| {
            let p = mesh.data.positions[v as usize];
            acc + Vec3::new(f64::from(p[0]), f64::from(p[1]), f64::from(p[2]))
        }) / 3.0
            + mesh.data.origin;
        let hits = h.pick_ray(down_ray(c.x, c.y), PickMode::Nearest);
        assert_eq!(hits.len(), 1, "triangle {tri}");
        let want = PickTarget::Body {
            body,
            instance: 0,
            sub: Some((TopoKind::Face, tri)),
        };
        assert_eq!(hits[0].target, want, "triangle {tri}");
        assert!((hits[0].world.z - 5.0).abs() < 1e-4);
        let sub = RenderSub {
            kind: RenderSubKind::Face,
            index: tri,
        };
        assert_eq!(h.scene().pick_map().render(&want), Some((key, Some(sub))));
        assert_eq!(h.scene().pick_map().target(key, Some(sub)), Some(want));
    }
    eprintln!("10 000 CPU picks: {:?}", start.elapsed());
    assert_eq!(h.scene().occurrence_path(0), Some(&Vec::new()));
}

#[test]
fn cpu_pick_service_is_asynchronous_and_select_through_is_depth_ordered() {
    let mut h = AppHarness::new();
    add_batch(
        &mut h,
        vec![
            grid(2, 2, 0.0, 0.0),
            grid(2, 2, 0.0, 4.0),
            grid(2, 2, 0.0, 8.0),
        ],
    );
    let ticket = h
        .picks_mut()
        .request_ray(down_ray(0.7, 0.3), PickMode::AllAlongRay);
    assert_eq!(h.picks().pending(), 1);
    let hits = h.poll_pick(ticket).unwrap();
    assert!(h.poll_pick(ticket).is_none(), "answered once");
    assert_eq!(h.picks().pending(), 0);
    let z: Vec<f64> = hits.iter().map(|h| h.world.z).collect();
    assert_eq!(hits.len(), 3);
    assert!((z[0] - 8.0).abs() < 1e-6 && (z[1] - 4.0).abs() < 1e-6 && z[2].abs() < 1e-6);
    let nearest = h.pick_ray(down_ray(0.7, 0.3), PickMode::Nearest);
    assert_eq!(nearest.len(), 1);
    assert_eq!(nearest[0].target, hits[0].target);
    assert!(
        h.pick_ray(down_ray(50.0, 50.0), PickMode::Nearest)
            .is_empty()
    );
}

// ---- Done-when 4: registered tessellator for a stub variant -------------------------------

#[test]
fn registered_test_tessellator_displays_a_stub_variant() {
    let mut h = AppHarness::new();
    assert!(
        h.app()
            .registry()
            .tessellator_owners()
            .contains(&"zz_scene_test")
    );
    let before = CUBE_CALLS.load(Ordering::SeqCst);
    add_batch(
        &mut h,
        vec![FeatureBox::new(UiImplicit { tag: 1 }), grid(2, 2, 5.0, 0.0)],
    );
    assert_eq!(CUBE_CALLS.load(Ordering::SeqCst), before + 1);
    assert_scene_matches(&h);
    let implicit = live_bodies(h.document())
        .into_iter()
        .find(|b| {
            matches!(
                h.document().evaluated().body(*b).unwrap().geom,
                BodyGeom::Implicit(_)
            )
        })
        .unwrap();
    let slot = h.scene().mesh_slot(implicit).unwrap();
    assert_eq!(cpu(h.scene()).mesh_triangles(slot), Some(12));
    // The Mesh body still goes through the framework's Mesh tessellator.
    assert_eq!(h.scene().stats().undisplayable, 0);
    // The cube is pickable.
    let hit = h.pick_ray(down_ray(0.1, 0.1), PickMode::Nearest);
    assert_eq!(hit[0].target.body(), Some(implicit));
}

#[test]
fn latest_wins_keeps_the_old_mesh_until_the_new_one_arrives() {
    let jobs = jobs();
    let mut scene = SceneAdapter::new(
        Box::new(CpuSceneBackend::new()),
        amg_ui::UiRegistry::build().tessellators(),
    );
    let mut doc = Document::new("lw");
    let f = add(&mut doc, FeatureBox::new(UiImplicit { tag: 0 }));
    doc.recompute_blocking(&jobs).unwrap();
    let ev = doc.drain_events();
    scene.handle_events(&ev, doc.design(), doc.evaluated(), &jobs);
    wait_idle(&mut scene);
    let body = scene.bodies().next().unwrap();
    let first = scene.mesh_slot(body).unwrap();

    *CUBE_GATE.lock().unwrap() = true;
    for tag in 1..=3 {
        doc.apply(Command::EditFeature {
            id: f,
            feature: FeatureBox::new(UiImplicit { tag }),
        })
        .unwrap();
        doc.recompute_blocking(&jobs).unwrap();
        let ev = doc.drain_events();
        scene.handle_events(&ev, doc.design(), doc.evaluated(), &jobs);
        scene.poll();
        // Meanwhile the old mesh is still drawn.
        assert_eq!(scene.mesh_slot(body), Some(first));
        assert_eq!(cpu(&scene).instance_count(), 1);
    }
    assert_eq!(scene.stats().superseded, 2);
    *CUBE_GATE.lock().unwrap() = false;
    wait_idle(&mut scene);
    assert_eq!(
        scene.uploads_of(body),
        2,
        "only the latest geometry was uploaded"
    );
    assert_ne!(scene.mesh_slot(body), Some(first));
    assert_eq!(cpu(&scene).mesh_count(), 1);
}

// ---- highlight and preview layers ---------------------------------------------------------

#[test]
fn highlights_resolve_through_the_pick_map_and_follow_the_theme() {
    let mut h = AppHarness::new();
    add_batch(&mut h, vec![grid(2, 2, 0.0, 0.0), grid(2, 2, 5.0, 0.0)]);
    let bodies: Vec<BodyId> = live_bodies(h.document()).into_iter().collect();
    let k0 = h.scene().pick_map().body_key(bodies[0], 0).unwrap();
    let k1 = h.scene().pick_map().body_key(bodies[1], 0).unwrap();
    let face = PickTarget::Body {
        body: bodies[1],
        instance: 0,
        sub: Some((TopoKind::Face, 3)),
    };
    h.app_mut().scene_mut().set_highlight(
        HighlightLayer::Selected,
        &[
            PickTarget::Body {
                body: bodies[0],
                instance: 0,
                sub: None,
            },
            face,
        ],
    );
    let want = vec![
        HighlightItem { key: k0, sub: None },
        HighlightItem {
            key: k1,
            sub: Some((RenderSubKind::Face, Range32 { start: 3, end: 4 })),
        },
    ];
    assert_eq!(
        h.scene().highlight(HighlightLayer::Selected),
        want.as_slice()
    );
    assert_eq!(
        cpu(h.scene()).highlight(HighlightLayer::Selected),
        want.as_slice()
    );
    let tokens = amg_ui::theme::ThemeTokens::dark();
    let sel = egui::Rgba::from(tokens.selected);
    let c = cpu(h.scene()).highlight_color(HighlightLayer::Selected);
    assert!((c[0] - sel.r()).abs() < 1e-6 && (c[2] - sel.b()).abs() < 1e-6);

    // A deleted body drops out of the layer.
    commit(
        &mut h,
        Command::DeleteFeatures {
            ids: vec![bodies[0].origin],
        },
    );
    assert_eq!(
        cpu(h.scene()).highlight(HighlightLayer::Selected),
        &want[1..]
    );
}

#[test]
fn preview_bodies_go_to_the_preview_layer_and_are_replaced_as_a_whole() {
    let jobs = jobs();
    let mut scene = SceneAdapter::new(
        Box::new(CpuSceneBackend::new()),
        amg_ui::UiRegistry::build().tessellators(),
    );
    let mut doc = Document::new("p");
    add(&mut doc, grid(2, 2, 0.0, 0.0));
    add(&mut doc, grid(2, 2, 5.0, 0.0));
    doc.recompute_blocking(&jobs).unwrap();
    let ev = doc.drain_events();
    scene.handle_events(&ev, doc.design(), doc.evaluated(), &jobs);
    wait_idle(&mut scene);
    let bodies: Vec<BodyId> = scene.bodies().collect();
    let keys_before = scene.pick_map().len();

    // A preview that changes the second body.
    let committed = Arc::clone(doc.evaluated());
    let mut p: Evaluated = (*committed).clone();
    let old = committed.body(bodies[1]).unwrap();
    p.set_body(
        bodies[1],
        Arc::new(EvalBody {
            component: old.component,
            geom: BodyGeom::Mesh(Arc::new(grid_mesh(&UiGrid {
                nx: 3,
                ny: 3,
                x0: 5.0,
                z: 1.0,
            }))),
            naming: None,
        }),
    );
    let p = Arc::new(p);
    scene.set_preview(Some(Arc::clone(&p)), doc.design(), &committed, &jobs);
    wait_idle(&mut scene);
    assert_eq!(scene.preview_bodies().collect::<Vec<_>>(), vec![bodies[1]]);
    let preview: Vec<_> = cpu(&scene)
        .instances()
        .filter(|(_, d)| d.layer == Layer::Preview)
        .map(|(_, d)| d.clone())
        .collect();
    assert_eq!(preview.len(), 1);
    assert!(preview[0].material.base_color[3] < 1.0, "ghost style");
    assert_eq!(cpu(&scene).instance_count(), 3);
    assert_eq!(
        scene.pick_map().len(),
        keys_before,
        "previews are not pickable"
    );
    let uploads = cpu(&scene).uploads();

    // The same preview again: nothing happens.
    scene.set_preview(Some(Arc::clone(&p)), doc.design(), &committed, &jobs);
    wait_idle(&mut scene);
    assert_eq!(cpu(&scene).uploads(), uploads);

    // Cleared.
    scene.set_preview(None, doc.design(), &committed, &jobs);
    assert_eq!(cpu(&scene).instance_count(), 2);
    assert_eq!(cpu(&scene).mesh_count(), 2);
    assert_eq!(scene.preview_bodies().count(), 0);
}

// ---- Done-when 5: 1 changed body in a 100-body scene ---------------------------------------

#[test]
fn timing_update_one_changed_body_of_100_under_1ms_ui_thread() {
    let jobs = jobs();
    let mut scene = SceneAdapter::new(
        Box::new(CpuSceneBackend::new()),
        amg_ui::UiRegistry::build().tessellators(),
    );
    let mut doc = Document::new("hundred");
    let features: Vec<FeatureId> = (0..100)
        .map(|i| add(&mut doc, grid(10, 10, f64::from(i) * 12.0, 0.0)))
        .collect();
    doc.recompute_blocking(&jobs).unwrap();
    let ev = doc.drain_events();
    scene.handle_events(&ev, doc.design(), doc.evaluated(), &jobs);
    wait_idle(&mut scene);
    assert_eq!(cpu(&scene).instance_count(), 100);

    let mut totals = Vec::new();
    for (round, f) in features.iter().enumerate().step_by(5) {
        doc.apply(Command::EditFeature {
            id: *f,
            feature: grid(10, 10, round as f64 * 12.0, 1.0 + round as f64),
        })
        .unwrap();
        doc.recompute_blocking(&jobs).unwrap();
        let ev = doc.drain_events();
        let tess = scene.stats().tessellations;
        let t0 = Instant::now();
        scene.handle_events(&ev, doc.design(), doc.evaluated(), &jobs);
        let events = t0.elapsed();
        assert_eq!(
            scene.stats().tessellations,
            tess + 1,
            "only the changed body"
        );
        // Frames until the result is in: each poll is one frame's UI-thread work.
        let mut polls = Duration::ZERO;
        let mut install = Duration::ZERO;
        let start = Instant::now();
        while scene.pending() > 0 {
            let t = Instant::now();
            let changed = scene.poll();
            let d = t.elapsed();
            polls += d;
            if changed {
                install = d;
            }
            assert!(start.elapsed() < Duration::from_secs(10));
            std::thread::sleep(Duration::from_micros(100));
        }
        // UI-thread time of the update: the event frame plus the frame that inserted it.
        totals.push((events + install, events, install, polls));
    }
    totals.sort_by_key(|t| t.0);
    let p50 = totals[totals.len() / 2];
    let worst = totals[totals.len() - 1];
    eprintln!(
        "update 1 of 100: p50 {:?} (events {:?} + insert {:?}), worst {:?} (events {:?} + insert {:?}); all polls while waiting, worst round {:?}",
        p50.0,
        p50.1,
        p50.2,
        worst.0,
        worst.1,
        worst.2,
        totals.iter().map(|t| t.3).max().unwrap()
    );
    assert_eq!(cpu(&scene).instance_count(), 100);
    if strict_timing() {
        assert!(
            worst.0 < Duration::from_millis(1),
            "UI-thread time {:?} >= 1 ms",
            worst.0
        );
    }
}
