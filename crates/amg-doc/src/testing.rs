//! Test-only features (`test.counter`, `test.box`, `test.add_mesh`) for `amg-doc`'s own tests
//! and for the recompute-engine tests (DOC-03). Compiled with `cfg(test)` or the
//! `test-features` cargo feature; never part of a shipping build.

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use amg_core::PersistentId;
use serde::{Deserialize, Serialize};

use crate::eval::BodyGeom;
use crate::feature::{
    BodyChange, BodyChangeKind, Dependency, Feature, FeatureError, FeatureOutput, RecomputeCtx,
};
use crate::id::FeatureId;
use crate::param::Expr;
use crate::stub::{IndexedMesh, Shape};

/// Kinds registered by this module.
pub const KINDS: [&str; 5] = [
    "test.add_mesh",
    "test.box",
    "test.counter",
    "test.param_box",
    "test.slow",
];

/// Forces these registrations to be linked (registry `link()` convention).
pub fn link() {}

static COUNTS: Mutex<BTreeMap<(PersistentId, FeatureId), u64>> = Mutex::new(BTreeMap::new());

/// How often `test.counter` feature `id` of the design `doc` has computed.
pub fn compute_count(doc: PersistentId, id: FeatureId) -> u64 {
    COUNTS
        .lock()
        .map_or(0, |m| m.get(&(doc, id)).copied().unwrap_or(0))
}

/// Reads arbitrary inputs, counts its computes, creates mesh body slot 0 and passes every input
/// body through as `Modified`. With `fail`, it errors (and has no effect).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TestCounter {
    /// Inputs (any kind).
    pub deps: Vec<Dependency>,
    /// Payload so edits change the feature's JSON.
    pub value: i64,
    /// Make `compute` fail.
    pub fail: bool,
}

impl Feature for TestCounter {
    fn inputs(&self) -> Vec<Dependency> {
        self.deps.clone()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        if let Ok(mut m) = COUNTS.lock() {
            *m.entry((ctx.design().meta().uuid, ctx.feature_id()))
                .or_default() += 1;
        }
        if self.fail {
            return Err(FeatureError::InvalidValue {
                field: "fail",
                msg: "asked to fail".to_owned(),
            });
        }
        let mut out = FeatureOutput::default();
        for d in &self.deps {
            if let Dependency::Body(b) = d {
                out.bodies.push(BodyChange {
                    id: *b,
                    change: BodyChangeKind::Modified {
                        geom: ctx.body(*b)?.clone(),
                        naming: None,
                    },
                });
            }
        }
        out.bodies.push(BodyChange {
            id: ctx.new_body_id(0),
            change: BodyChangeKind::Created {
                geom: BodyGeom::Mesh(Arc::new(IndexedMesh::default())),
                naming: None,
            },
        });
        Ok(out)
    }
}

crate::register_feature!(TestCounter, "test.counter", version = 1);

/// A box primitive: creates solid body slot 0. `size` must be finite and positive (mm).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TestBox {
    /// Edge lengths, mm.
    pub size: [f64; 3],
}

impl Feature for TestBox {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn validate(&self) -> Result<(), FeatureError> {
        if self.size.iter().all(|s| s.is_finite() && *s > 0.0) {
            Ok(())
        } else {
            Err(FeatureError::InvalidValue {
                field: "size",
                msg: format!("{:?} must be finite and positive", self.size),
            })
        }
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Ok(FeatureOutput {
            bodies: vec![BodyChange {
                id: ctx.new_body_id(0),
                change: BodyChangeKind::Created {
                    geom: BodyGeom::Solid(Shape::new()),
                    naming: None,
                },
            }],
            ..FeatureOutput::default()
        })
    }
}

/// Version 2 renamed `edge` (one number, v1) to `size` (three numbers).
fn box_migrate(
    from: u32,
    raw: serde_json::Value,
) -> Result<serde_json::Value, crate::MigrationError> {
    match (from, raw.get("edge")) {
        (1, Some(e)) => Ok(serde_json::json!({ "size": [e, e, e] })),
        _ => Err(crate::MigrationError(format!(
            "can't migrate test.box v{from}: {raw}"
        ))),
    }
}

crate::register_feature!(TestBox, "test.box", version = 2, migrate = box_migrate);

/// A box whose edge lengths are parameter expressions (DOC-10). The stub geometry carries no
/// data, so the box reports its volume (mm³) as a warning `volume=<v>`; warnings are cached
/// with the output like geometry, so tests can read the volume of cached results too.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TestParamBox {
    /// Edge lengths.
    pub size: [Expr; 3],
}

impl Feature for TestParamBox {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn exprs(&self) -> Vec<&Expr> {
        self.size.iter().collect()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        if let Ok(mut m) = COUNTS.lock() {
            *m.entry((ctx.design().meta().uuid, ctx.feature_id()))
                .or_default() += 1;
        }
        let mut volume = 1.0;
        for e in &self.size {
            volume *= ctx.eval_length(e)?;
        }
        Ok(FeatureOutput {
            bodies: vec![BodyChange {
                id: ctx.new_body_id(0),
                change: BodyChangeKind::Created {
                    geom: BodyGeom::Solid(Shape::new()),
                    naming: None,
                },
            }],
            warnings: vec![crate::eval::Diagnostic {
                message: format!("volume={volume}"),
                cause: None,
                entities: Vec::new(),
            }],
            ..FeatureOutput::default()
        })
    }
}

crate::register_feature!(TestParamBox, "test.param_box", version = 1);

/// Adds `bodies` mesh bodies (slots `0..bodies`), like a mesh import.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TestAddMesh {
    /// Number of bodies.
    pub bodies: u32,
    /// Free-form label (exercises string data).
    pub label: String,
}

impl Feature for TestAddMesh {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Ok(FeatureOutput {
            bodies: (0..self.bodies)
                .map(|s| BodyChange {
                    id: ctx.new_body_id(s),
                    change: BodyChangeKind::Created {
                        geom: BodyGeom::Mesh(Arc::new(IndexedMesh::default())),
                        naming: None,
                    },
                })
                .collect(),
            ..FeatureOutput::default()
        })
    }
}

crate::register_feature!(TestAddMesh, "test.add_mesh", version = 1);

/// Sleeps `ms` milliseconds (a dimensionless parameter) in 1 ms slices that check for
/// cancellation, then creates mesh body slot 0. Counts its computes like `test.counter`.
/// With `panic`, it panics instead. For the recompute-job tests (DOC-12).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TestSlow {
    /// Duration in milliseconds.
    pub ms: Expr,
    /// Payload so features with equal parameters have distinct keys.
    pub tag: i64,
    /// Panic in `compute`.
    pub panic: bool,
}

impl Feature for TestSlow {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn exprs(&self) -> Vec<&Expr> {
        vec![&self.ms]
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        if let Ok(mut m) = COUNTS.lock() {
            *m.entry((ctx.design().meta().uuid, ctx.feature_id()))
                .or_default() += 1;
        }
        assert!(!self.panic, "test.slow asked to panic");
        let ms = ctx.eval_scalar(&self.ms)?;
        let end = Instant::now() + Duration::from_secs_f64(ms.max(0.0) / 1000.0);
        while Instant::now() < end {
            ctx.check_cancelled()?;
            std::thread::sleep(Duration::from_millis(1));
        }
        Ok(FeatureOutput {
            bodies: vec![BodyChange {
                id: ctx.new_body_id(0),
                change: BodyChangeKind::Created {
                    geom: BodyGeom::Mesh(Arc::new(IndexedMesh::default())),
                    naming: None,
                },
            }],
            ..FeatureOutput::default()
        })
    }
}

crate::register_feature!(TestSlow, "test.slow", version = 1);
