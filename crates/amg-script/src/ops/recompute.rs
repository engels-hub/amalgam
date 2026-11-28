//! Recompute and report the health of every timeline feature.

use std::time::Instant;

use amg_doc::{FeatureId, Health};
use amg_jobs::CancelToken;
use serde::Serialize;

use super::REPORT_SCHEMA;
use crate::ScriptError;
use crate::host::ScriptDoc;

/// One timeline feature in a [`RecomputeReport`].
#[derive(Clone, Debug, Serialize)]
pub struct FeatureHealthRow {
    /// The feature.
    pub id: FeatureId,
    /// Its registered kind.
    pub kind: String,
    /// Its name (the kind if the user gave none).
    pub name: String,
    /// `"ok"`, `"warning"`, `"error"`, `"suppressed"` or `"rolled_back"`.
    pub health: &'static str,
    /// Error or warning messages.
    pub messages: Vec<String>,
    /// Compute time; 0 when the output came from the cache.
    pub millis: f64,
}

/// Result of [`recompute`].
#[derive(Clone, Debug, Serialize)]
pub struct RecomputeReport {
    /// [`REPORT_SCHEMA`].
    pub schema: u32,
    /// Revision of the script's document (moves on every edit, undo and redo).
    pub revision: u64,
    /// Features that computed successfully in the pass that produced the current state.
    pub computed: usize,
    /// Features that pass took from the cache. Failing, suppressed and rolled-back features
    /// are in neither count.
    pub cached: usize,
    /// Wall time of this call (about 0 if the document was already evaluated).
    pub millis: f64,
    /// Every timeline feature, in timeline order (none in direct mode).
    pub features: Vec<FeatureHealthRow>,
}

impl RecomputeReport {
    /// True if any feature failed.
    pub fn has_errors(&self) -> bool {
        self.features.iter().any(|f| f.health == "error")
    }
}

/// Brings the document up to date and reports it. A failing feature is not an error of this
/// call: see [`RecomputeReport::has_errors`].
///
/// # Errors
/// As [`ScriptDoc::evaluated`].
pub fn recompute(
    doc: &mut ScriptDoc,
    cancel: &CancelToken,
) -> Result<RecomputeReport, ScriptError> {
    let started = Instant::now();
    let eval = doc.evaluated(cancel)?;
    let millis = started.elapsed().as_secs_f64() * 1000.0;
    let document = doc.document();
    let (computed, cached) = document
        .last_run()
        .map_or((0, 0), |r| (r.computed.len(), r.cached.len()));
    let features = document
        .design()
        .timeline()
        .nodes()
        .filter_map(|node| {
            let e = eval.node(node.id)?;
            let (health, messages) = match &e.health {
                Health::Ok => ("ok", Vec::new()),
                Health::Warning(w) => ("warning", w.iter().map(|d| d.message.clone()).collect()),
                Health::Error(d) => ("error", vec![d.message.clone()]),
                Health::Suppressed => ("suppressed", Vec::new()),
                Health::RolledBack => ("rolled_back", Vec::new()),
            };
            let kind = node.feature.kind().to_owned();
            Some(FeatureHealthRow {
                id: node.id,
                name: node.name.clone().unwrap_or_else(|| kind.clone()),
                kind,
                health,
                messages,
                millis: e.duration.as_secs_f64() * 1000.0,
            })
        })
        .collect();
    Ok(RecomputeReport {
        schema: REPORT_SCHEMA,
        revision: document.history().revision().0,
        computed,
        cached,
        millis,
        features,
    })
}
