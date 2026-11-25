//! IO-09: a solid body that cannot be tessellated gives a clear error, in the default build
//! (no geometry kernel) and, for a body without a B-rep, with the `occt` feature too.
#![allow(clippy::unwrap_used, clippy::expect_used, reason = "tests")]

use std::sync::OnceLock;

use amg_core::Diagnostic;
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, Command, Dependency, Document, Feature, FeatureBox,
    FeatureError, FeatureOutput, RecomputeCtx, register_feature,
};
use amg_features::io::{
    ExportPrep, ExportRefinement, ExportSelection, ImportPrepError, scene_from_document,
};
use amg_jobs::{JobCtx, JobSystem, JobSystemConfig};
use serde::{Deserialize, Serialize};

/// Creates a solid body that carries no B-rep.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Io09Empty;

impl Feature for Io09Empty {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Ok(FeatureOutput {
            bodies: vec![BodyChange {
                id: ctx.new_body_id(0),
                change: BodyChangeKind::Created {
                    geom: BodyGeom::Solid(amg_doc::stub::Shape::new()),
                    naming: None,
                },
            }],
            ..FeatureOutput::default()
        })
    }
}

register_feature!(Io09Empty, "test.io09_empty", version = 1);

#[test]
fn a_solid_without_geometry_gives_a_clear_error() {
    static JOBS: OnceLock<JobSystem> = OnceLock::new();
    let jobs = JOBS.get_or_init(|| JobSystem::new(JobSystemConfig::default()).unwrap());
    amg_features::io::link();
    let mut doc = Document::new("io09-empty");
    let root = doc.design().root();
    doc.apply(Command::AddFeature {
        at: None,
        feature: FeatureBox::new(Io09Empty),
        context: root,
        name: None,
    })
    .unwrap();
    doc.recompute_blocking(jobs).unwrap();
    let run = |prep: &ExportPrep| {
        scene_from_document(
            doc.design(),
            doc.evaluated(),
            &ExportSelection::AllVisible,
            prep,
            &JobCtx::detached(),
        )
        .unwrap_err()
    };

    assert!(matches!(
        run(&ExportPrep::default()),
        ImportPrepError::NeedsTessellation(1)
    ));
    let err = run(&ExportPrep {
        refinement: Some(ExportRefinement::Medium),
        ..ExportPrep::default()
    });
    if amg_brep::OCCT_BACKEND {
        assert!(matches!(err, ImportPrepError::NoFaces(_)), "{err:?}");
    } else {
        assert!(
            matches!(err, ImportPrepError::Tessellation { .. }),
            "{err:?}"
        );
        assert!(
            err.to_string()
                .contains("geometry kernel is not available in this build"),
            "{err}"
        );
        assert!(err.hint().unwrap().contains("occt"), "{:?}", err.hint());
    }
}
