//! MESH-23: previews of mesh features on large inputs run on a decimated proxy; full
//! computes never do. The limits are lowered so the test runs on a small sphere (own test
//! binary: the limits are process-wide).

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    missing_docs,
    reason = "tests"
)]

use std::sync::{Arc, OnceLock};

use amg_doc::{
    BodyGeom, BodyId, Command, CreatedId, Dependency, Document, Feature, FeatureBox, FeatureError,
    FeatureId, FeatureOutput, RecomputeCtx,
};
use amg_features::mesh::{
    MeshReverseNormalFeature, MeshSmoothFeature, preview_proxy_limits, set_preview_proxy_limits,
    uses_preview_proxy,
};
use amg_jobs::{JobSystem, JobSystemConfig};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct ProxySource {
    subdiv: u32,
}

impl Feature for ProxySource {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let m = amg_mesh::corpus::bench::geodesic_sphere(self.subdiv, 10.0, None);
        Ok(FeatureOutput {
            bodies: vec![amg_doc::BodyChange {
                id: ctx.new_body_id(0),
                change: amg_doc::BodyChangeKind::Created {
                    geom: BodyGeom::Mesh(Arc::new(m)),
                    naming: None,
                },
            }],
            ..FeatureOutput::default()
        })
    }
}

amg_doc::register_feature!(ProxySource, "test.mesh_proxy_source", version = 1);

fn jobs() -> &'static JobSystem {
    static JOBS: OnceLock<JobSystem> = OnceLock::new();
    JOBS.get_or_init(|| JobSystem::new(JobSystemConfig::default()).unwrap())
}

fn add(doc: &mut Document, f: impl Feature) -> FeatureId {
    let out = doc
        .apply(Command::AddFeature {
            at: None,
            feature: FeatureBox::new(f),
            context: doc.design().root(),
            name: None,
        })
        .unwrap();
    match out.created.first() {
        Some(CreatedId::Feature(id)) => *id,
        other => panic!("{other:?}"),
    }
}

fn faces(ev: &amg_doc::Evaluated, id: BodyId) -> usize {
    match &ev.body(id).unwrap().geom {
        BodyGeom::Mesh(m) => m.face_count(),
        other => panic!("{other:?}"),
    }
}

#[test]
fn previews_of_large_inputs_run_on_a_proxy_and_full_computes_do_not() {
    amg_features::mesh::link();
    assert_eq!(
        preview_proxy_limits(),
        (
            amg_features::mesh::PREVIEW_PROXY_FACES,
            amg_features::mesh::PREVIEW_PROXY_TARGET
        )
    );
    let mut doc = Document::new("proxy");
    let src = add(&mut doc, ProxySource { subdiv: 16 });
    doc.recompute_blocking(jobs()).unwrap();
    let body = BodyId {
        origin: src,
        slot: 0,
    };
    let n = faces(doc.evaluated(), body);
    assert!(n > 2000, "{n} faces");

    let smooth = |doc: &Document| Command::AddFeature {
        at: None,
        feature: FeatureBox::new(MeshSmoothFeature::new(body)),
        context: doc.design().root(),
        name: None,
    };

    // Default limits (2 M): the preview runs on the full mesh. (Another feature than the one
    // below: previews are cached, and the cache key does not include the limits.)
    assert!(!uses_preview_proxy(n));
    let reverse = Command::AddFeature {
        at: None,
        feature: FeatureBox::new(MeshReverseNormalFeature { body, groups: None }),
        context: doc.design().root(),
        name: None,
    };
    let ev = doc
        .preview(jobs(), &reverse)
        .unwrap()
        .wait_blocking()
        .unwrap();
    assert_eq!(faces(&ev, body), n);

    // Lowered limits: the preview body is the proxy; downstream features see it too.
    set_preview_proxy_limits(1000, 400);
    assert!(uses_preview_proxy(n));
    let ev = doc
        .preview(jobs(), &smooth(&doc))
        .unwrap()
        .wait_blocking()
        .unwrap();
    let p = faces(&ev, body);
    assert!((390..=410).contains(&p), "proxy has {p} faces");
    // A second preview reuses the cached proxy (same result).
    let ev2 = doc
        .preview(jobs(), &smooth(&doc))
        .unwrap()
        .wait_blocking()
        .unwrap();
    assert_eq!(faces(&ev2, body), p);

    // A full compute ignores the limits.
    add(&mut doc, MeshSmoothFeature::new(body));
    add(&mut doc, MeshReverseNormalFeature { body, groups: None });
    doc.recompute_blocking(jobs()).unwrap();
    assert_eq!(faces(doc.evaluated(), body), n);
    set_preview_proxy_limits(
        amg_features::mesh::PREVIEW_PROXY_FACES,
        amg_features::mesh::PREVIEW_PROXY_TARGET,
    );
}
