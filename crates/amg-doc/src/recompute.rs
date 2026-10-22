//! The recompute engine core (S01 §3, DOC-03): a synchronous pass over a [`Design`] snapshot
//! that produces an [`Evaluated`].
//!
//! # Algorithm
//! The pass walks the timeline in order and keeps a *producer map*: for every body, sketch and
//! construction the cache key of the last healthy feature that wrote it. A feature's
//! [`CacheKey`] chains the keys of the producers of everything it reads (constructive traces),
//! so editing feature *k* changes exactly the keys of the features that transitively read it.
//! Every other node hits the [`GeomCache`].
//!
//! - Suppressed node: `Health::Suppressed`, writes nothing (a later reader of its outputs gets
//!   `ErrorCause::Upstream`).
//! - Node at or after the marker: `Health::RolledBack`.
//! - A failed node has *no effect*: nothing it computed is applied, so the bodies it would have
//!   modified keep their previous state, and bodies it would have created don't exist. Failures
//!   are not cached.
//! - A node whose input has no producer (failed, suppressed, deleted) gets its own
//!   `Health::Error` with `ErrorCause::Upstream(producer)` without being computed.
//!
//! Cancellation is cooperative ([`JobCtx::checkpoint`] between features, and whatever the
//! features check themselves); outputs of features that finished stay in the cache.

use std::collections::BTreeMap;
use std::sync::Arc;
use std::time::Instant;

use amg_core::{Cancelled, Quantity, Tolerance};
use amg_jobs::JobCtx;

use crate::cache::GeomCache;
use crate::eval::{
    BodyGeom, CacheKey, Diagnostic, ErrorCause, EvalBody, Evaluated, Health, NodeEval,
};
use crate::feature::{
    BodyChangeKind, ComputeMode, Dependency, FeatureError, FeatureOutput, RecomputeCtx,
};
use crate::id::{BodyId, ConstructionId, FeatureId, SketchId};
use crate::model::Design;
use crate::naming::EntityRef;
use crate::param::Expr;
use crate::timeline::FeatureNode;

/// Version of the cache-key layout and of the geometry payload format. Bump when either
/// changes: every cached output is then invalid.
pub const ENGINE_FORMAT_VERSION: u32 = 1;

/// Kernel identity folded into [`engine_salt`].
///
/// TODO(BREP-02): `amg_brep::kernel_version()` (OCCT version + `robust::POLICY_VERSION`).
const KERNEL_VERSION: &str = "stub-kernel-0";

/// The salt every cache key starts with (S01 §3 `ENGINE_SALT`): format version, kernel version
/// and tolerance settings. Changing any of them invalidates every cache entry.
pub fn engine_salt(tol: &Tolerance) -> [u8; 32] {
    let mut h = KeyHasher::new();
    h.bytes(b"amg-doc/engine-salt");
    h.u64(u64::from(ENGINE_FORMAT_VERSION));
    h.bytes(KERNEL_VERSION.as_bytes());
    h.f64(tol.linear);
    h.f64(tol.angular);
    h.finish().0
}

/// Result of a recompute pass.
#[derive(Clone, Debug)]
pub struct RecomputeResult {
    /// The evaluated state for the design's revision.
    pub evaluated: Evaluated,
    /// Features whose `compute` ran, in timeline order.
    pub computed: Vec<FeatureId>,
    /// Features served from the cache, in timeline order.
    pub cached: Vec<FeatureId>,
}

/// Runs a full pass over `design` in `mode`.
///
/// `job` provides cancellation and progress (`JobCtx::detached()` for synchronous callers).
/// Returns `Err(Cancelled)` when cancelled; features that completed stay in `cache`.
pub fn recompute(
    design: &Design,
    cache: &GeomCache,
    mode: ComputeMode,
    job: &JobCtx,
) -> Result<RecomputeResult, Cancelled> {
    Pass::new(design, cache, mode, job).run()
}

// ---- key hashing ------------------------------------------------------------------------

/// blake3 with length-prefixed fields, so field boundaries are unambiguous.
struct KeyHasher(blake3::Hasher);

impl KeyHasher {
    fn new() -> KeyHasher {
        KeyHasher(blake3::Hasher::new())
    }

    fn bytes(&mut self, b: &[u8]) {
        self.u64(u64::try_from(b.len()).unwrap_or(u64::MAX));
        self.0.update(b);
    }

    fn u64(&mut self, v: u64) {
        self.0.update(&v.to_le_bytes());
    }

    fn tag(&mut self, t: u8) {
        self.0.update(&[t]);
    }

    /// `f64::to_bits` with `-0.0` normalised to `0.0` (S01 §3 `value_bits`).
    fn f64(&mut self, v: f64) {
        // -0.0 + 0.0 == +0.0; every other value is unchanged.
        self.0.update(&(v + 0.0).to_bits().to_le_bytes());
    }

    fn key(&mut self, k: &CacheKey) {
        self.0.update(&k.0);
    }

    fn finish(&self) -> CacheKey {
        CacheKey(*self.0.finalize().as_bytes())
    }
}

// ---- the pass ---------------------------------------------------------------------------

fn diag(message: impl Into<String>, cause: Option<ErrorCause>) -> Diagnostic {
    Diagnostic {
        message: message.into(),
        cause,
        entities: Vec::new(),
    }
}

/// Mutable state of one pass.
struct Pass<'a> {
    design: &'a Design,
    cache: &'a GeomCache,
    mode: ComputeMode,
    job: &'a JobCtx,
    salt: [u8; 32],
    out: Evaluated,
    /// Key of the last healthy writer of each body / sketch / construction.
    body_key: BTreeMap<BodyId, CacheKey>,
    sketch_key: BTreeMap<SketchId, CacheKey>,
    construction_key: BTreeMap<ConstructionId, CacheKey>,
    /// Key of every healthy executed feature (for `Dependency::Feature`).
    feature_key: BTreeMap<FeatureId, CacheKey>,
    computed: Vec<FeatureId>,
    cached: Vec<FeatureId>,
}

impl<'a> Pass<'a> {
    fn new(
        design: &'a Design,
        cache: &'a GeomCache,
        mode: ComputeMode,
        job: &'a JobCtx,
    ) -> Pass<'a> {
        Pass {
            design,
            cache,
            mode,
            job,
            salt: engine_salt(&design.meta().tolerance),
            out: Evaluated::new(design.revision()),
            body_key: BTreeMap::new(),
            sketch_key: BTreeMap::new(),
            construction_key: BTreeMap::new(),
            feature_key: BTreeMap::new(),
            computed: Vec::new(),
            cached: Vec::new(),
        }
    }

    fn run(mut self) -> Result<RecomputeResult, Cancelled> {
        let tl = self.design.timeline();
        let n = tl.len();
        let marker = tl.marker();
        for (i, node) in tl.nodes().enumerate() {
            self.job.checkpoint()?;
            #[allow(clippy::cast_precision_loss, reason = "progress fraction")]
            let (lo, hi) = (i as f32 / n.max(1) as f32, (i + 1) as f32 / n.max(1) as f32);
            let name = node.name.as_deref().unwrap_or_else(|| node.feature.kind());
            self.job.progress_msg(lo, name);
            if i >= marker {
                self.set(node.id, Health::RolledBack, None, Default::default());
            } else if node.suppressed {
                self.set(node.id, Health::Suppressed, None, Default::default());
            } else {
                let sub = self.job.sub(lo, hi);
                self.node(node, &sub)?;
            }
        }
        self.job.progress(1.0);
        Ok(RecomputeResult {
            evaluated: self.out,
            computed: self.computed,
            cached: self.cached,
        })
    }

    fn set(
        &mut self,
        id: FeatureId,
        health: Health,
        key: Option<CacheKey>,
        duration: std::time::Duration,
    ) {
        self.out.set_node(
            id,
            NodeEval {
                health,
                key,
                duration,
            },
        );
    }

    fn fail(&mut self, id: FeatureId, key: Option<CacheKey>, d: Diagnostic) {
        self.set(id, Health::Error(d), key, Default::default());
    }

    /// Evaluates one live node: key, cache lookup or compute, apply.
    fn node(&mut self, node: &FeatureNode, job: &JobCtx) -> Result<(), Cancelled> {
        let key = match self.node_key(node) {
            Ok(k) => k,
            Err(d) => {
                self.fail(node.id, None, d);
                return Ok(());
            }
        };
        let (output, duration, hit) = if let Some(o) = self.cache.get(&key) {
            (o, std::time::Duration::ZERO, true)
        } else {
            let started = Instant::now();
            let result = {
                let mut ctx =
                    RecomputeCtx::new(self.design, &self.out, node.id, node.context, self.mode)
                        .with_job(job);
                node.feature.compute(&mut ctx)
            };
            let duration = started.elapsed();
            let output = match result {
                Ok(o) => o,
                Err(FeatureError::Cancelled) => return Err(Cancelled),
                Err(e) => {
                    self.fail(node.id, Some(key), error_diag(&e));
                    return Ok(());
                }
            };
            if let Err(d) = self.check_output(node, &output) {
                self.fail(node.id, Some(key), d);
                return Ok(());
            }
            let output = Arc::new(output);
            self.cache.insert(key, Arc::clone(&output));
            (output, duration, false)
        };
        if hit {
            self.cached.push(node.id);
        } else {
            self.computed.push(node.id);
        }
        self.apply(node, key, &output);
        let health = if output.warnings.is_empty() {
            Health::Ok
        } else {
            Health::Warning(output.warnings.clone())
        };
        self.set(node.id, health, Some(key), duration);
        Ok(())
    }

    /// Engine-level sanity checks of a fresh output (cached outputs passed them already).
    fn check_output(&self, node: &FeatureNode, out: &FeatureOutput) -> Result<(), Diagnostic> {
        let bad = |m: String| Err(diag(m, None));
        for c in &out.bodies {
            match &c.change {
                BodyChangeKind::Created { .. } => {
                    if c.id.origin != node.id {
                        return bad(format!(
                            "feature created {:?}, which belongs to another feature",
                            c.id
                        ));
                    }
                }
                BodyChangeKind::Modified { .. } | BodyChangeKind::Deleted => {
                    if self.out.body(c.id).is_none() {
                        return bad(format!("feature changed {:?}, which does not exist", c.id));
                    }
                }
            }
        }
        if out.sketch.is_some() && out.construction.is_some() {
            return bad("a feature can't produce both a sketch and construction geometry".into());
        }
        Ok(())
    }

    /// Writes an output into the evaluated state and the producer map.
    fn apply(&mut self, node: &FeatureNode, key: CacheKey, out: &FeatureOutput) {
        for c in &out.bodies {
            match &c.change {
                BodyChangeKind::Created { geom, naming } => {
                    self.put_body(c.id, node, geom, naming.as_ref(), false);
                    self.body_key.insert(c.id, key);
                }
                BodyChangeKind::Modified { geom, naming } => {
                    self.put_body(c.id, node, geom, naming.as_ref(), true);
                    self.body_key.insert(c.id, key);
                }
                BodyChangeKind::Deleted => {
                    self.out.remove_body(c.id);
                    self.body_key.remove(&c.id);
                }
            }
        }
        if let Some(s) = &out.sketch {
            let id = SketchId(node.id);
            self.out.set_sketch(id, Arc::clone(s));
            self.sketch_key.insert(id, key);
        }
        if let Some(g) = out.construction {
            let id = ConstructionId(node.id);
            self.out.set_construction(id, g);
            self.construction_key.insert(id, key);
        }
        self.feature_key.insert(node.id, key);
    }

    fn put_body(
        &mut self,
        id: BodyId,
        node: &FeatureNode,
        geom: &BodyGeom,
        naming: Option<&crate::naming::BodyNaming>,
        keep_component: bool,
    ) {
        let component = keep_component
            .then(|| self.out.body(id).map(|b| b.component))
            .flatten()
            .unwrap_or(node.context);
        self.out.set_body(
            id,
            Arc::new(EvalBody {
                component,
                geom: geom.clone(),
                naming: naming.map(|n| Arc::new(n.clone())),
            }),
        );
    }

    // ---- keys ---------------------------------------------------------------------------

    /// The cache key of `node`, or why it can't have one (missing input, bad expression).
    fn node_key(&self, node: &FeatureNode) -> Result<CacheKey, Diagnostic> {
        let f = &node.feature;
        let mut h = KeyHasher::new();
        h.0.update(&self.salt);
        h.bytes(f.kind().as_bytes());
        h.u64(u64::from(f.version()));
        let json = f
            .to_json()
            .and_then(|v| serde_json::to_vec(&v))
            .map_err(|e| diag(format!("feature data can't be serialized: {e}"), None))?;
        h.bytes(&json);

        let exprs = f.exprs();
        h.u64(u64::try_from(exprs.len()).unwrap_or(u64::MAX));
        for e in exprs {
            let q = self.expr_value(e)?;
            match e {
                Expr::Param(_) => h.tag(0),
                Expr::Inline(src) => {
                    h.tag(1);
                    h.bytes(src.as_str().as_bytes());
                }
            }
            h.f64(q.value);
            let d = q.dim;
            for x in [d.length, d.mass, d.time, d.temperature, d.angle] {
                h.0.update(&x.to_le_bytes());
            }
        }

        let inputs = f.inputs();
        h.u64(u64::try_from(inputs.len()).unwrap_or(u64::MAX));
        for dep in &inputs {
            self.hash_input(&mut h, dep)?;
        }

        h.u64(node.context.0);
        h.tag(match self.mode {
            ComputeMode::Full => 0,
            ComputeMode::Preview => 1,
        });
        Ok(h.finish())
    }

    fn expr_value(&self, e: &Expr) -> Result<Quantity, Diagnostic> {
        let params = self.design.params();
        let q = match e {
            Expr::Param(id) => params.value(*id).ok_or_else(|| {
                diag(
                    format!("parameter {id} does not exist"),
                    Some(ErrorCause::Expr(*id)),
                )
            })?,
            Expr::Inline(src) => params
                .try_eval(src.as_str(), None, &self.design.meta().default_units)
                .map_err(|e| diag(e.to_string(), None))?,
        };
        if q.value.is_finite() {
            Ok(q)
        } else {
            let cause = match e {
                Expr::Param(id) => Some(ErrorCause::Expr(*id)),
                Expr::Inline(_) => None,
            };
            Err(diag("expression value is not finite", cause))
        }
    }

    fn hash_input(&self, h: &mut KeyHasher, dep: &Dependency) -> Result<(), Diagnostic> {
        match dep {
            Dependency::Body(b) => {
                h.tag(1);
                h.key(self.producer_of_body(*b)?);
            }
            Dependency::Entity(e) => {
                h.tag(2);
                h.key(self.producer_of_entity(e)?);
            }
            Dependency::Sketch(s) => {
                h.tag(3);
                h.key(self.sketch_key.get(s).ok_or_else(|| self.upstream(s.0))?);
            }
            Dependency::Construction(c) => {
                h.tag(4);
                h.key(
                    self.construction_key
                        .get(c)
                        .ok_or_else(|| self.upstream(c.0))?,
                );
            }
            Dependency::Feature(f) => {
                h.tag(5);
                h.key(self.feature_key.get(f).ok_or_else(|| self.upstream(*f))?);
            }
            Dependency::Occurrence(_) => h.tag(6),
        }
        for o in dep.occurrences() {
            let occ = self
                .design
                .occurrence(o)
                .ok_or_else(|| diag(format!("occurrence {o} does not exist"), None))?;
            h.u64(o.0);
            for x in occ.transform.to_cols_array() {
                h.f64(x);
            }
        }
        Ok(())
    }

    fn producer_of_body(&self, b: BodyId) -> Result<&CacheKey, Diagnostic> {
        self.body_key.get(&b).ok_or_else(|| self.upstream(b.origin))
    }

    fn producer_of_entity(&self, e: &EntityRef) -> Result<&CacheKey, Diagnostic> {
        match e {
            EntityRef::Body(b) => self.producer_of_body(*b),
            EntityRef::Topo(t) => self.producer_of_body(t.body),
            EntityRef::SketchCurve { sketch, .. }
            | EntityRef::SketchPoint { sketch, .. }
            | EntityRef::Profile { sketch, .. } => self
                .sketch_key
                .get(sketch)
                .ok_or_else(|| self.upstream(sketch.0)),
            EntityRef::Construction(c) => self
                .construction_key
                .get(c)
                .ok_or_else(|| self.upstream(c.0)),
            EntityRef::InContext { target, .. } => self.producer_of_entity(target),
        }
    }

    /// Error for an input whose producer didn't deliver, saying why.
    fn upstream(&self, producer: FeatureId) -> Diagnostic {
        let why = match self.out.node(producer).map(|n| &n.health) {
            Some(Health::Error(_)) => "failed",
            Some(Health::Suppressed) => "is suppressed",
            Some(Health::RolledBack) => "is rolled back",
            Some(_) => "did not produce it",
            None if self.design.timeline().contains(producer) => "comes later in the timeline",
            None => "no longer exists",
        };
        diag(
            format!("an input is not available: {producer} {why}"),
            Some(ErrorCause::Upstream(producer)),
        )
    }
}

/// Maps a feature's error to the node diagnostic.
fn error_diag(e: &FeatureError) -> Diagnostic {
    let cause = match e {
        FeatureError::Resolve(r) => Some(ErrorCause::Resolve(r.clone())),
        FeatureError::UnknownKind { kind, .. } => Some(ErrorCause::UnknownKind(kind.clone())),
        FeatureError::MissingInput(d) => d.producer().map(ErrorCause::Upstream),
        _ => None,
    };
    diag(e.to_string(), cause)
}
