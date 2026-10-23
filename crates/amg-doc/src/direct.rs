//! Direct mode and bake (S01 §7, DOC-08).
//!
//! In [`DesignMode::Direct`] there is no timeline. The bodies are *baked*: their geometry is
//! authored data in [`Design::baked`](crate::Design::baked) (and in `bodies/` in the file), and
//! each keeps its [`BodyId`], so body properties and references survive every switch.
//!
//! # Switching
//! - **History to Direct** needs the evaluated bodies, so it is applied through
//!   [`Document::apply`]: `Command::SetDesignMode { mode: Direct }` bakes every body of the
//!   current [`Evaluated`] state, drops the timeline (model parameters become user parameters)
//!   and is one undo entry. The document must be evaluated and free of failing features
//!   (a failed feature has no effect, so baking would silently lose it). Bodies after the
//!   rollback marker, suppressed features, sketches and construction geometry are not baked.
//!   [`apply_command`](crate::apply_command) alone refuses it, as it has no geometry.
//! - **Direct to History** is a pure design edit (`apply_command` does it): one
//!   [`BaseFeature`] node adopts all baked bodies (Fusion's "base feature") and a fresh timeline
//!   starts after it. Features that read an adopted body resolve its producer to the base
//!   feature ([`Timeline::effective_producer`](crate::Timeline)).
//!
//! # Features in direct mode
//! Timeline commands are refused with [`CommandError::WrongMode`]. A new feature is added with
//! [`Document::bake_feature`]: a job puts it on a scratch timeline, computes it once against the
//! baked bodies, and bakes its outputs; [`Document::commit_bake`] applies the result as one undo
//! entry if the document did not change meanwhile.
//!
//! # Blob names
//! Every baked geometry gets a fresh blob path (`bodies/<origin>-<slot>-g<n>.<ext>`, `n` from the
//! design's id allocator). A path therefore identifies one geometry, which is what the cache keys
//! of direct-mode features rely on.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use amg_core::Cancelled;
use amg_jobs::{JobError, JobHandle, JobSystem, SpawnOptions, is_ui_thread};

use crate::command::{Command, CommandError, CommandOutcome, CreatedId, apply_ungated};
use crate::document::Document;
use crate::eval::{BodyGeom, Evaluated, Health};
use crate::feature::{
    BodyChange, BodyChangeKind, ComputeMode, Dependency, Feature, FeatureBox, FeatureError,
    FeatureOutput, RecomputeCtx,
};
use crate::id::{BodyId, ComponentId, FeatureId, Revision};
use crate::model::{BakedBody, BlobRef, Design, DesignMode, ModelError};
use crate::recompute::recompute;

/// Kind string of [`BaseFeature`].
pub const BASE_FEATURE_KIND: &str = "core.base";

/// One body adopted by a [`BaseFeature`].
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BaseBody {
    /// The body (keeps the id it had in direct mode).
    pub id: BodyId,
    /// Its baked geometry's blob. Part of the feature data so cache keys follow the geometry.
    pub blob: BlobRef,
}

/// The single feature that holds the baked bodies of a design that left direct mode.
///
/// It reads no inputs; its geometry comes from [`Design::baked`]. It can be deleted (the
/// bodies go with it) but not edited.
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BaseFeature {
    /// The adopted bodies, in id order.
    pub bodies: Vec<BaseBody>,
}

impl BaseFeature {
    /// True if one of the adopted bodies has `origin` as its origin.
    pub fn owns(&self, origin: FeatureId) -> bool {
        self.bodies.iter().any(|b| b.id.origin == origin)
    }
}

impl Feature for BaseFeature {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn validate(&self) -> Result<(), FeatureError> {
        let mut seen = BTreeSet::new();
        for b in &self.bodies {
            if !seen.insert(b.id) {
                return Err(FeatureError::InvalidValue {
                    field: "bodies",
                    msg: format!("{:?} is listed twice", b.id),
                });
            }
        }
        Ok(())
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let design = ctx.design();
        let mut out = FeatureOutput::default();
        for b in &self.bodies {
            let bad = |msg: String| FeatureError::InvalidValue {
                field: "bodies",
                msg,
            };
            let baked = design
                .baked(b.id)
                .ok_or_else(|| bad(format!("{:?} is not baked", b.id)))?;
            if baked.blob != b.blob {
                return Err(bad(format!("{:?} was re-baked ({})", b.id, baked.blob.0)));
            }
            let geom = baked
                .geom
                .clone()
                .ok_or_else(|| bad(format!("geometry of {:?} is not loaded", b.id)))?;
            out.bodies.push(BodyChange {
                id: b.id,
                change: BodyChangeKind::Created { geom, naming: None },
            });
        }
        Ok(out)
    }
}

crate::register_feature!(BaseFeature, "core.base", version = 1);

/// File extension of a baked geometry kind (`bodies/` entry names).
fn geom_ext(g: &BodyGeom) -> &'static str {
    match g {
        BodyGeom::Solid(_) => "brep",
        BodyGeom::Mesh(_) => "mesh",
        BodyGeom::Implicit(_) => "sdf",
        BodyGeom::Form(_) => "subd",
    }
}

fn invalid(field: &'static str, msg: impl Into<String>) -> ModelError {
    ModelError::InvalidValue {
        field,
        msg: msg.into(),
    }
}

impl Design {
    /// A fresh, unique blob path for `geom` of body `id`.
    fn new_blob(&mut self, id: BodyId, geom: &BodyGeom) -> Result<BlobRef, ModelError> {
        let n = self.alloc()?;
        Ok(BlobRef(format!(
            "bodies/{}-{}-g{n}.{}",
            id.origin.0,
            id.slot,
            geom_ext(geom)
        )))
    }

    /// History to Direct: replaces the timeline by `bodies` (id, component, geometry), which
    /// become baked bodies with their properties kept. Model parameters become user
    /// parameters. Atomic: on `Err` the design is unchanged.
    ///
    /// Low-level: [`Document::apply`] with `SetDesignMode` collects `bodies` from the
    /// evaluated state.
    pub fn enter_direct_mode(
        &mut self,
        bodies: Vec<(BodyId, ComponentId, BodyGeom)>,
    ) -> Result<(), ModelError> {
        if self.meta.mode != DesignMode::History {
            return Err(invalid("mode", "the design is already in direct mode"));
        }
        let mut w = self.clone();
        w.ensure_ids(bodies.len() as u64)?;
        let props: Vec<_> = bodies
            .iter()
            .filter_map(|(id, ..)| w.body_props.get(id).map(|p| (*id, (**p).clone())))
            .collect();
        let ids: Vec<FeatureId> = w.timeline.nodes().map(|n| n.id).collect();
        for &id in &ids {
            w.remove_feature(id)?;
        }
        w.convert_model_params(&ids);
        for (id, component, geom) in bodies {
            let blob = w.new_blob(id, &geom)?;
            w.set_baked(
                id,
                Some(BakedBody {
                    component,
                    blob,
                    geom: Some(geom),
                }),
            )?;
        }
        for (id, p) in props {
            w.set_body_props(id, p)?;
        }
        w.meta.mode = DesignMode::Direct;
        w.bump();
        *self = w;
        Ok(())
    }

    /// Direct to History: a [`BaseFeature`] adopts every baked body that no timeline feature
    /// owns, at the start of the timeline. Returns its id (`None` if there was nothing to
    /// adopt). Atomic.
    pub fn leave_direct_mode(&mut self) -> Result<Option<FeatureId>, ModelError> {
        if self.meta.mode != DesignMode::Direct {
            return Err(invalid("mode", "the design is not in direct mode"));
        }
        let mut w = self.clone();
        let bodies: Vec<BaseBody> = w
            .baked_bodies()
            .filter(|(id, _)| !w.timeline.contains(id.origin))
            .map(|(id, b)| BaseBody {
                id,
                blob: b.blob.clone(),
            })
            .collect();
        w.meta.mode = DesignMode::History;
        let base = if bodies.is_empty() {
            None
        } else {
            let root = w.root();
            Some(w.add_feature(
                Some(0),
                FeatureBox::new(BaseFeature { bodies }),
                root,
                Some("Base feature".to_owned()),
            )?)
        };
        w.bump();
        *self = w;
        Ok(base)
    }

    /// Direct-mode feature: drops the scratch node `feature` and bakes what `evaluated`
    /// (computed for a design that contained it) says changed: new and modified bodies get a
    /// new blob, deleted ones are removed.
    fn bake_feature_result(
        &mut self,
        feature: FeatureId,
        evaluated: &Evaluated,
    ) -> Result<(), ModelError> {
        let old: BTreeMap<BodyId, BakedBody> =
            self.baked_bodies().map(|(id, b)| (id, b.clone())).collect();
        self.remove_feature(feature)?;
        self.convert_model_params(&[feature]);
        for (id, body) in evaluated.bodies() {
            let unchanged = old.get(&id).is_some_and(|o| {
                o.component == body.component
                    && o.geom.as_ref().is_some_and(|g| g.same_as(&body.geom))
            });
            if unchanged {
                continue;
            }
            let blob = self.new_blob(id, &body.geom)?;
            self.set_baked(
                id,
                Some(BakedBody {
                    component: body.component,
                    blob,
                    geom: Some(body.geom.clone()),
                }),
            )?;
        }
        for id in old.keys() {
            if evaluated.body(*id).is_none() {
                self.set_baked(*id, None)?;
            }
        }
        Ok(())
    }
}

/// Why a direct-mode feature could not be baked.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum DirectError {
    /// The command was rejected (wrong command, wrong mode, bad feature).
    #[error(transparent)]
    Command(#[from] CommandError),
    /// The feature failed to compute; the message.
    #[error("the feature failed: {0}")]
    Feature(String),
    /// Baking the result into the design failed.
    #[error(transparent)]
    Model(#[from] ModelError),
    /// The job was cancelled.
    #[error("cancelled")]
    Cancelled,
    /// The job panicked (a bug in a feature); the panic message.
    #[error("internal error: {0}")]
    Panicked(String),
}

impl From<Cancelled> for DirectError {
    fn from(_: Cancelled) -> DirectError {
        DirectError::Cancelled
    }
}

impl From<JobError<DirectError>> for DirectError {
    fn from(e: JobError<DirectError>) -> DirectError {
        match e {
            JobError::Cancelled => DirectError::Cancelled,
            JobError::Panicked(m) => DirectError::Panicked(m),
            JobError::Failed(e) => e,
        }
    }
}

impl amg_core::Diagnostic for DirectError {}

/// The result of computing one feature in direct mode: the design with the outputs baked,
/// ready for [`Document::commit_bake`].
#[derive(Debug)]
pub struct DirectBake {
    revision: Revision,
    command: Command,
    design: Design,
    created: Vec<CreatedId>,
}

impl Document {
    /// History to Direct (called by `apply`): bakes the evaluated bodies.
    pub(crate) fn enter_direct(&mut self, cmd: Command) -> Result<CommandOutcome, CommandError> {
        if !self.is_evaluated() {
            return Err(CommandError::Invalid(
                "recompute the design before switching to direct mode".to_owned(),
            ));
        }
        let ev = Arc::clone(self.evaluated());
        if let Some((id, d)) = ev.nodes().find_map(|(id, n)| match &n.health {
            Health::Error(d) => Some((id, d.message.clone())),
            _ => None,
        }) {
            return Err(CommandError::Invalid(format!(
                "{id} has an error ({d}); fix or delete it before switching to direct mode"
            )));
        }
        let bodies = ev
            .bodies()
            .map(|(id, b)| (id, b.component, b.geom.clone()))
            .collect();
        let mut work = (*self.snapshot()).clone();
        work.enter_direct_mode(bodies)?;
        let revision = self
            .history_mut()
            .push_design(cmd.label().to_owned(), work, vec![cmd]);
        Ok(CommandOutcome {
            created: Vec::new(),
            revision,
        })
    }

    /// Adds a feature in direct mode (`cmd` must be `Command::AddFeature`): a job computes it
    /// once against the baked bodies and bakes its outputs. Nothing changes in the document
    /// until [`Document::commit_bake`]. Cancel the handle when superseded.
    pub fn bake_feature(
        &self,
        jobs: &JobSystem,
        cmd: Command,
    ) -> Result<JobHandle<DirectBake, DirectError>, CommandError> {
        if self.design().meta().mode != DesignMode::Direct {
            return Err(CommandError::WrongMode(DesignMode::History));
        }
        if !matches!(cmd, Command::AddFeature { .. }) {
            return Err(CommandError::Invalid(
                "only AddFeature can be baked".to_owned(),
            ));
        }
        let mut work = (*self.snapshot()).clone();
        let created = apply_ungated(&mut work, cmd.clone())?;
        let Some(CreatedId::Feature(fid)) = created.first().cloned() else {
            return Err(CommandError::Invalid("no feature was created".to_owned()));
        };
        let revision = self.history().revision();
        let cache = Arc::clone(self.cache());
        Ok(
            jobs.spawn_fn("Direct feature", SpawnOptions::default(), move |ctx| {
                let r = recompute(&work, &cache, ComputeMode::Full, ctx)?;
                match r.evaluated.node(fid).map(|n| &n.health) {
                    Some(Health::Ok | Health::Warning(_)) => {}
                    Some(Health::Error(d)) => return Err(DirectError::Feature(d.message.clone())),
                    _ => return Err(DirectError::Feature("the feature did not run".to_owned())),
                }
                let mut design = work;
                design.bake_feature_result(fid, &r.evaluated)?;
                Ok(DirectBake {
                    revision,
                    command: cmd,
                    design,
                    created,
                })
            }),
        )
    }

    /// Applies a finished [`Document::bake_feature`] as one undo entry. Fails if the document
    /// changed since the job started (compute again).
    pub fn commit_bake(&mut self, bake: DirectBake) -> Result<CommandOutcome, CommandError> {
        if bake.revision != self.history().revision() {
            return Err(CommandError::Invalid(
                "the document changed while the feature was computed".to_owned(),
            ));
        }
        let label = bake.command.label().to_owned();
        let revision = self
            .history_mut()
            .push_design(label, bake.design, vec![bake.command]);
        Ok(CommandOutcome {
            created: bake.created,
            revision,
        })
    }

    /// [`Document::bake_feature`] + wait + [`Document::commit_bake`], for the CLI, Python and
    /// tests.
    ///
    /// # Panics
    /// In debug builds on the UI thread.
    pub fn bake_feature_blocking(
        &mut self,
        jobs: &JobSystem,
        cmd: Command,
    ) -> Result<CommandOutcome, DirectError> {
        debug_assert!(
            !is_ui_thread(),
            "Document::bake_feature_blocking called on the UI thread"
        );
        let bake = self.bake_feature(jobs, cmd)?.wait_blocking()?;
        Ok(self.commit_bake(bake)?)
    }
}
