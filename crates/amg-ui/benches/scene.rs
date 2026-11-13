//! `scene_update_1of100` (spec §Testing): UI-thread time of the scene adapter for one changed
//! body in a 100-body scene: `handle_events` for the edit plus the `poll` that inserts the new
//! upload (the tessellation job and the recompute are not UI-thread time and not measured).

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "bench code")]

use std::sync::Arc;
use std::time::{Duration, Instant};

use amg_core::Point3;
use amg_doc::stub::IndexedMesh;
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, Command, CreatedId, Dependency, Document, FeatureBox,
    FeatureError, FeatureId, FeatureOutput, RecomputeCtx,
};
use amg_jobs::{JobSystem, JobSystemConfig};
use amg_ui::scene::{CpuSceneBackend, SceneAdapter};
use criterion::{Criterion, criterion_group, criterion_main};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct BenchGrid {
    x0: f64,
    z: f64,
}

impl amg_doc::Feature for BenchGrid {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let n = 10u32;
        let mut pos = Vec::new();
        for j in 0..=n {
            for i in 0..=n {
                pos.push(Point3::new(self.x0 + f64::from(i), f64::from(j), self.z));
            }
        }
        let mut tris = Vec::new();
        for j in 0..n {
            for i in 0..n {
                let a = j * (n + 1) + i;
                tris.push([a, a + 1, a + n + 2]);
                tris.push([a, a + n + 2, a + n + 1]);
            }
        }
        let mesh = IndexedMesh::new(pos, tris).expect("grid");
        Ok(FeatureOutput {
            bodies: vec![BodyChange {
                id: ctx.new_body_id(0),
                change: BodyChangeKind::Created {
                    geom: BodyGeom::Mesh(Arc::new(mesh)),
                    naming: None,
                },
            }],
            ..FeatureOutput::default()
        })
    }
}

amg_doc::register_feature!(BenchGrid, "uibench.grid", version = 1);

fn add(doc: &mut Document, f: BenchGrid) -> FeatureId {
    let root = doc.design().root();
    match doc
        .apply(Command::AddFeature {
            at: None,
            feature: FeatureBox::new(f),
            context: root,
            name: None,
        })
        .unwrap()
        .created[0]
    {
        CreatedId::Feature(id) => id,
        ref other => panic!("{other:?}"),
    }
}

fn wait(scene: &mut SceneAdapter) -> Duration {
    let mut install = Duration::ZERO;
    while scene.pending() > 0 {
        let t = Instant::now();
        if scene.poll() {
            install = t.elapsed();
        }
        std::thread::sleep(Duration::from_micros(50));
    }
    install
}

fn bench(c: &mut Criterion) {
    let jobs = JobSystem::new(JobSystemConfig::default()).unwrap();
    let mut scene = SceneAdapter::new(
        Box::new(CpuSceneBackend::new()),
        amg_ui::UiRegistry::build().tessellators(),
    );
    let mut doc = Document::new("bench");
    let features: Vec<FeatureId> = (0..100)
        .map(|i| {
            add(
                &mut doc,
                BenchGrid {
                    x0: f64::from(i) * 12.0,
                    z: 0.0,
                },
            )
        })
        .collect();
    doc.recompute_blocking(&jobs).unwrap();
    let ev = doc.drain_events();
    scene.handle_events(&ev, doc.design(), doc.evaluated(), &jobs);
    wait(&mut scene);

    let mut round = 0u32;
    c.bench_function("scene_update_1of100", |b| {
        b.iter_custom(|iters| {
            let mut total = Duration::ZERO;
            for _ in 0..iters {
                round += 1;
                let i = (round % 100) as usize;
                doc.apply(Command::EditFeature {
                    id: features[i],
                    feature: FeatureBox::new(BenchGrid {
                        x0: i as f64 * 12.0,
                        z: f64::from(round),
                    }),
                })
                .unwrap();
                doc.recompute_blocking(&jobs).unwrap();
                let ev = doc.drain_events();
                let t = Instant::now();
                scene.handle_events(&ev, doc.design(), doc.evaluated(), &jobs);
                total += t.elapsed();
                total += wait(&mut scene);
            }
            total
        });
    });
}

criterion_group! {
    name = benches;
    config = Criterion::default().sample_size(30).measurement_time(Duration::from_secs(5));
    targets = bench
}
criterion_main!(benches);
