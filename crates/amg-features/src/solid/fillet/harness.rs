//! Public test entry point for S20's corpus harness (KH-02/KH-03, SOL-05 Done-when 6): runs
//! `solid.fillet` itself on a corpus case — a `.brep` solid and `(EdgeId, FilletSpec)` pairs
//! as a `KernelOp::Fillet` states them — without building a timeline.
//!
//! The case solid becomes body `BodyId { FeatureId(1), 0 }` of an otherwise empty design, with
//! one name per face (`Face#i`), so every edge has a reference. Each edge is one fillet set
//! picked by `topo_ref_at` (key = position + 1); the feature is computed as
//! `FeatureId(2)` with the caller's kernel and mode. A failure comes back with the edges the
//! feature highlighted, in the case's index space.

use std::sync::Arc;

use amg_brep::{BrepKernel, EdgeId, FilletSpec, Shape, SubKind};
use amg_doc::naming::{Role, topo_ref_at};
use amg_doc::{
    BodyChangeKind, BodyGeom, BodyId, BodyNaming, ComputeMode, Diagnostic, Document, EntityRef,
    EvalBody, Evaluated, Expr, ExprSrc, Feature, FeatureError, FeatureId, RecomputeCtx, TopoKind,
};

use super::{CornerType, Fillet, FilletRadius, FilletSet, VarRadius};

/// The id of the case body.
pub const CASE_BODY: BodyId = BodyId {
    origin: FeatureId(1),
    slot: 0,
};

/// One fillet corpus case.
#[derive(Clone, Copy, Debug)]
pub struct FilletCase<'a> {
    /// The input solid.
    pub shape: &'a Shape,
    /// Edges (in `shape`'s index space) and their specs, in kernel order.
    pub edges: &'a [(EdgeId, FilletSpec)],
    /// The feature's `tangent_chain` flag.
    pub tangent_chain: bool,
    /// Full or preview compute.
    pub mode: ComputeMode,
}

/// A successful run.
#[derive(Clone, Debug)]
pub struct FilletRun {
    /// The filleted solid.
    pub shape: Shape,
    /// Its names, as the feature built them.
    pub naming: BodyNaming,
    /// Feature warnings (resolution, partial history).
    pub warnings: Vec<Diagnostic>,
}

/// A failed run.
#[derive(Debug, thiserror::Error)]
#[error("{error}")]
pub struct FilletRunError {
    /// The feature error (`Kernel` for kernel failures).
    pub error: FeatureError,
    /// The highlighted (failing) edges, in the case's index space.
    pub failing: Vec<EdgeId>,
}

fn expr(src: String) -> Expr {
    Expr::Inline(ExprSrc(src))
}

fn radius(spec: &FilletSpec) -> FilletRadius {
    match spec {
        FilletSpec::Constant(r) => FilletRadius::Constant(expr(format!("{r} mm"))),
        FilletSpec::Chord(c) => FilletRadius::Chord(expr(format!("{c} mm"))),
        FilletSpec::Variable(law) => FilletRadius::Variable(
            law.iter()
                .map(|(t, r)| VarRadius {
                    at: expr(format!("{t}")),
                    radius: expr(format!("{r} mm")),
                })
                .collect(),
        ),
    }
}

fn fail(error: FeatureError) -> FilletRunError {
    FilletRunError {
        error,
        failing: Vec::new(),
    }
}

/// The case shape as a named body: every face `Face#i`, edges and vertices derived.
fn case_naming(k: &dyn BrepKernel, shape: &Shape) -> Result<BodyNaming, FilletRunError> {
    let mut b = amg_doc::naming::NamingBuilder::new(CASE_BODY.origin);
    for i in 0..shape.count(SubKind::Face) {
        b.generated(TopoKind::Face, i, Role::new("Face", i), None);
    }
    b.finish(k, shape)
        .map_err(|e| fail(FeatureError::Kernel(format!("naming the case: {e}"))))
}

/// Runs `solid.fillet` on `case` with kernel `k`.
///
/// # Errors
/// The feature's error, with the highlighted edges for kernel failures.
pub fn run_fillet(k: &dyn BrepKernel, case: &FilletCase<'_>) -> Result<FilletRun, FilletRunError> {
    super::super::link();
    let doc = Document::new("fillet-harness");
    let design = doc.design();
    let root = design.root();
    let mut before = Evaluated::default();
    let naming = case_naming(k, case.shape)?;
    before.set_body(
        CASE_BODY,
        Arc::new(EvalBody {
            component: root,
            geom: BodyGeom::Solid(amg_doc::stub::Shape::from_brep(case.shape.clone())),
            naming: Some(Arc::new(naming)),
        }),
    );
    let mut sets = Vec::with_capacity(case.edges.len());
    for (pos, (e, spec)) in case.edges.iter().enumerate() {
        let mut r = topo_ref_at(&before, k, CASE_BODY, TopoKind::Edge, e.0).ok_or_else(|| {
            fail(FeatureError::InvalidValue {
                field: "edges",
                msg: format!("edge {} has no reference", e.0),
            })
        })?;
        r.key = u32::try_from(pos + 1).unwrap_or(u32::MAX);
        sets.push(FilletSet {
            edges: vec![r],
            radius: radius(spec),
        });
    }
    let feature = Fillet {
        sets,
        tangent_chain: case.tangent_chain,
        corner: CornerType::RollingBall,
    };
    feature.validate().map_err(fail)?;
    let mut ctx = RecomputeCtx::new(design, &before, FeatureId(2), root, case.mode).with_kernel(k);
    match feature.compute(&mut ctx) {
        Ok(out) => {
            let change = out
                .bodies
                .into_iter()
                .find(|c| c.id == CASE_BODY)
                .map(|c| c.change);
            match change {
                Some(BodyChangeKind::Modified {
                    geom: BodyGeom::Solid(s),
                    naming: Some(naming),
                }) => match s.brep() {
                    Some(shape) => Ok(FilletRun {
                        shape: shape.clone(),
                        naming,
                        warnings: out.warnings,
                    }),
                    None => Err(fail(FeatureError::Kernel("no B-rep result".into()))),
                },
                _ => Err(fail(FeatureError::Kernel(
                    "the fillet did not modify the case body".into(),
                ))),
            }
        }
        Err(error) => {
            let failing = ctx
                .highlights()
                .iter()
                .filter_map(|h| match h {
                    EntityRef::Topo(t) if t.kind == TopoKind::Edge => before.resolve(t).ok(),
                    _ => None,
                })
                .map(|r| EdgeId(r.index))
                .collect();
            Err(FilletRunError { error, failing })
        }
    }
}
