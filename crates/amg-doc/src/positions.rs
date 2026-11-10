//! Pending positions, Capture Position and Revert (S06 note §3, ASM-13).
//!
//! The local transform of an occurrence is defined by three authored layers, each overriding
//! the one below:
//! 1. **Base**: [`Occurrence::transform`](crate::Occurrence).
//! 2. **Capture nodes**: timeline features `assembly.capture_position`
//!    ([`CapturePositionFeature`]), built in like `core.base` because the recompute engine reads
//!    them. A node applies only if it is before the marker and not suppressed.
//! 3. **Pending**: [`PendingPositions`] on the [`Design`], on top at the marker.
//!
//! Features (in-context references, `Dependency::Occurrence`, cache keys) see the captured
//! layers at their own timeline index through a running [`PositionTable`] and never the pending
//! one, so a pending move recomputes no feature. The assembly stage seeds from
//! [`Design::seed_transform`].

use std::collections::BTreeMap;

use amg_core::Transform;
use serde::{Deserialize, Serialize};

use crate::eval::Diagnostic;
use crate::feature::{Dependency, Feature, FeatureError, FeatureOutput, RecomputeCtx};
use crate::id::{FeatureId, OccurrenceId};
use crate::joints::JointState;
use crate::model::{Design, ModelError, check_transform};
use crate::pcoll::PMap;

/// Kind string of [`CapturePositionFeature`].
pub const CAPTURE_POSITION_KIND: &str = "assembly.capture_position";

/// Pending moves (authored, saved in `design.json`, undoable). Empty after Capture Position,
/// Revert, a marker move, and always in direct mode.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "PendingRepr", into = "PendingRepr")]
pub struct PendingPositions {
    /// Local transforms relative to the parent.
    pub(crate) moves: PMap<OccurrenceId, Transform>,
    pub(crate) joint_states: PMap<FeatureId, JointState>,
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct PendingRepr {
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    moves: Vec<(OccurrenceId, Transform)>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    joint_states: Vec<(FeatureId, JointState)>,
}

impl From<PendingPositions> for PendingRepr {
    fn from(p: PendingPositions) -> PendingRepr {
        PendingRepr {
            moves: p.moves.into_iter().collect(),
            joint_states: p.joint_states.into_iter().collect(),
        }
    }
}

impl TryFrom<PendingRepr> for PendingPositions {
    type Error = String;
    fn try_from(r: PendingRepr) -> Result<PendingPositions, String> {
        let mut p = PendingPositions::default();
        for (o, t) in r.moves {
            if p.moves.insert(o, t).is_some() {
                return Err(format!("pending move of {o} is listed twice"));
            }
        }
        for (f, s) in r.joint_states {
            if p.joint_states.insert(f, s).is_some() {
                return Err(format!("pending joint state of {f} is listed twice"));
            }
        }
        Ok(p)
    }
}

impl PendingPositions {
    /// True if nothing is pending.
    pub fn is_empty(&self) -> bool {
        self.moves.is_empty() && self.joint_states.is_empty()
    }

    /// The pending local transform of `o`.
    pub fn get(&self, o: OccurrenceId) -> Option<Transform> {
        self.moves.get(&o).copied()
    }

    /// Pending moves in occurrence-id order.
    pub fn moves(&self) -> impl Iterator<Item = (OccurrenceId, Transform)> + '_ {
        self.moves.iter().map(|(o, t)| (*o, *t))
    }

    /// The pending state of joint `f`.
    pub fn joint_state(&self, f: FeatureId) -> Option<&JointState> {
        self.joint_states.get(&f)
    }

    /// Pending joint states in feature-id order.
    pub fn joint_states(&self) -> impl Iterator<Item = (FeatureId, &JointState)> + '_ {
        self.joint_states.iter().map(|(f, s)| (*f, s))
    }
}

/// The built-in timeline feature `assembly.capture_position` v1 (Capture Position): the local
/// transforms and joint states that apply to everything after it in the timeline. Root
/// context, no inputs, no bodies. Entries are sorted by id without duplicates.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CapturePositionFeature {
    /// `(occurrence, local transform relative to its parent)`, ascending ids.
    pub moves: Vec<(OccurrenceId, Transform)>,
    /// `(joint feature, unwrapped values)`, ascending ids.
    pub joint_states: Vec<(FeatureId, JointState)>,
}

impl CapturePositionFeature {
    /// The captured transform of `o`, if this node lists it.
    pub fn get(&self, o: OccurrenceId) -> Option<Transform> {
        self.moves
            .binary_search_by_key(&o, |(id, _)| *id)
            .ok()
            .map(|i| self.moves[i].1)
    }

    /// The captured state of joint `f`, if this node lists it.
    pub fn joint_state(&self, f: FeatureId) -> Option<&JointState> {
        self.joint_states
            .binary_search_by_key(&f, |(id, _)| *id)
            .ok()
            .map(|i| &self.joint_states[i].1)
    }
}

fn bad(field: &'static str, msg: impl Into<String>) -> FeatureError {
    FeatureError::InvalidValue {
        field,
        msg: msg.into(),
    }
}

impl Feature for CapturePositionFeature {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn validate(&self) -> Result<(), FeatureError> {
        if !self.moves.windows(2).all(|w| w[0].0 < w[1].0) {
            return Err(bad("moves", "occurrences must be ascending and unique"));
        }
        if !self.joint_states.windows(2).all(|w| w[0].0 < w[1].0) {
            return Err(bad("joint_states", "joints must be ascending and unique"));
        }
        for (_, t) in &self.moves {
            check_transform(t).map_err(|e| bad("moves", e.to_string()))?;
        }
        if self.joint_states.iter().any(|(_, s)| !s.is_finite()) {
            return Err(bad("joint_states", "values must be finite"));
        }
        Ok(())
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let d = ctx.design();
        let mut out = FeatureOutput::default();
        for o in self.missing_occurrences(d) {
            out.warnings.push(Diagnostic {
                message: format!("captured occurrence {o} no longer exists; ignored"),
                cause: None,
                entities: Vec::new(),
            });
        }
        for f in self.missing_joints(d) {
            out.warnings.push(Diagnostic {
                message: format!("captured joint {f} no longer exists; ignored"),
                cause: None,
                entities: Vec::new(),
            });
        }
        Ok(out)
    }
}

impl CapturePositionFeature {
    /// Listed occurrences that no longer exist.
    pub(crate) fn missing_occurrences(&self, d: &Design) -> Vec<OccurrenceId> {
        self.moves
            .iter()
            .map(|(o, _)| *o)
            .filter(|o| d.occurrence(*o).is_none())
            .collect()
    }

    /// Listed joints that are no longer timeline features.
    pub(crate) fn missing_joints(&self, d: &Design) -> Vec<FeatureId> {
        self.joint_states
            .iter()
            .map(|(f, _)| *f)
            .filter(|f| !d.timeline().contains(*f))
            .collect()
    }
}

crate::register_feature!(
    CapturePositionFeature,
    "assembly.capture_position",
    version = 1
);

/// The running position table of a timeline pass: the base transforms overridden by every
/// applying capture node passed so far. Stores overrides only.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PositionTable {
    moves: BTreeMap<OccurrenceId, Transform>,
    joints: BTreeMap<FeatureId, JointState>,
}

impl PositionTable {
    /// The captured override of `o`, if any capture node passed so far lists it.
    pub fn get(&self, o: OccurrenceId) -> Option<Transform> {
        self.moves.get(&o).copied()
    }

    /// The captured state of joint `f`, if any capture node passed so far lists it.
    pub fn joint_state(&self, f: FeatureId) -> Option<&JointState> {
        self.joints.get(&f)
    }

    /// Applies a capture node (entries for missing occurrences and joints are ignored).
    pub(crate) fn apply(&mut self, c: &CapturePositionFeature, d: &Design) {
        for (o, t) in &c.moves {
            if d.occurrence(*o).is_some() {
                self.moves.insert(*o, *t);
            }
        }
        for (f, s) in &c.joint_states {
            if d.timeline().contains(*f) {
                self.joints.insert(*f, s.clone());
            }
        }
    }
}

impl Design {
    /// Pending moves (empty after Capture Position, Revert, a marker move, and in direct mode).
    pub fn pending_positions(&self) -> &PendingPositions {
        &self.pending
    }

    /// True if any move or joint state is pending.
    pub fn has_pending_positions(&self) -> bool {
        !self.pending.is_empty()
    }

    /// The position table of the capture nodes before timeline index `index` (clamped to the
    /// marker: nodes at or after it do not apply; suppressed nodes are skipped).
    pub fn capture_table(&self, index: usize) -> PositionTable {
        let tl = self.timeline();
        let end = index.min(tl.marker());
        let mut table = PositionTable::default();
        for node in tl.nodes().take(end) {
            if node.suppressed {
                continue;
            }
            if let Some(c) = node.feature.downcast_ref::<CapturePositionFeature>() {
                table.apply(c, self);
            }
        }
        table
    }

    /// Local transform of `o` as seen by timeline node `index`: the base transform overridden
    /// by the applying capture nodes before `index` (never pending). `index = nodes.len()` is
    /// the end of the timeline. `None` if `o` does not exist.
    pub fn captured_transform_at(&self, o: OccurrenceId, index: usize) -> Option<Transform> {
        let base = self.occurrence(o)?.transform;
        let tl = self.timeline();
        let end = index.min(tl.marker());
        let mut found = None;
        for node in tl.nodes().take(end) {
            if node.suppressed {
                continue;
            }
            if let Some(t) = node
                .feature
                .downcast_ref::<CapturePositionFeature>()
                .and_then(|c| c.get(o))
            {
                found = Some(t);
            }
        }
        Some(found.unwrap_or(base))
    }

    /// What the assembly stage seeds from: pending, else captured at the marker.
    pub fn seed_transform(&self, o: OccurrenceId) -> Option<Transform> {
        if let Some(t) = self.pending.get(o) {
            return self.occurrence(o).map(|_| t);
        }
        self.captured_transform_at(o, self.timeline().marker())
    }

    /// [`seed_transform`](Self::seed_transform) with a precomputed `capture_table(marker)`.
    pub(crate) fn seed_with(&self, table: &PositionTable, o: OccurrenceId) -> Option<Transform> {
        let base = self.occurrence(o)?.transform;
        Some(self.pending.get(o).or_else(|| table.get(o)).unwrap_or(base))
    }

    /// Effective joint state at the marker: pending, else the latest applying capture node
    /// before the marker that lists it. (The base layer comes with the solver, ASM-04.)
    pub fn joint_state(&self, joint: FeatureId) -> Option<&JointState> {
        if let Some(s) = self.pending.joint_state(joint) {
            return Some(s);
        }
        let tl = self.timeline();
        let mut found = None;
        for node in tl.nodes().take(tl.marker()) {
            if node.suppressed {
                continue;
            }
            if let Some(s) = node
                .feature
                .downcast_ref::<CapturePositionFeature>()
                .and_then(|c| c.joint_state(joint))
            {
                found = Some(s);
            }
        }
        found
    }

    // ---- low-level edits behind the commands --------------------------------------------

    pub(crate) fn check_position_moves(
        &self,
        moves: &[(OccurrenceId, Transform)],
    ) -> Result<(), ModelError> {
        let mut seen = std::collections::BTreeSet::new();
        for (o, t) in moves {
            if self.occurrence(*o).is_none() {
                return Err(ModelError::NotFound {
                    what: "occurrence",
                    id: o.0,
                });
            }
            if !seen.insert(*o) {
                return Err(ModelError::InvalidValue {
                    field: "moves",
                    msg: format!("{o} is listed twice"),
                });
            }
            check_transform(t)?;
        }
        Ok(())
    }

    pub(crate) fn check_position_joints(
        &self,
        states: &[(FeatureId, JointState)],
    ) -> Result<(), ModelError> {
        let mut seen = std::collections::BTreeSet::new();
        for (f, s) in states {
            if !self.timeline().contains(*f) {
                return Err(ModelError::NotFound {
                    what: "feature",
                    id: f.0,
                });
            }
            if !seen.insert(*f) {
                return Err(ModelError::InvalidValue {
                    field: "joint_states",
                    msg: format!("{f} is listed twice"),
                });
            }
            if !s.is_finite() {
                return Err(ModelError::InvalidValue {
                    field: "joint_states",
                    msg: "values must be finite".to_owned(),
                });
            }
        }
        Ok(())
    }

    /// Merges moves and joint states into the pending layer (all or none). Grounded
    /// occurrences and direct mode are refused by the command layer, not here.
    pub fn merge_pending_positions(
        &mut self,
        moves: &[(OccurrenceId, Transform)],
        joint_states: &[(FeatureId, JointState)],
    ) -> Result<(), ModelError> {
        self.check_position_moves(moves)?;
        self.check_position_joints(joint_states)?;
        let before = self.pending.clone();
        for (o, t) in moves {
            self.pending.moves.insert(*o, *t);
        }
        for (f, s) in joint_states {
            self.pending.joint_states.insert(*f, s.clone());
        }
        if self.pending != before {
            self.bump();
        }
        Ok(())
    }

    /// A direct, non-pending placement (S06 `SetOccurrenceTransforms`), all or none. Each
    /// occurrence is written to the layer effective at the marker: the latest applying capture
    /// node that lists it (the node's data is replaced), else the base transform. Pending
    /// entries of these occurrences are dropped.
    pub fn place_occurrences(
        &mut self,
        moves: &[(OccurrenceId, Transform)],
    ) -> Result<(), ModelError> {
        self.check_position_moves(moves)?;
        let marker = self.timeline().marker();
        let mut nodes: BTreeMap<FeatureId, CapturePositionFeature> = BTreeMap::new();
        let mut base = Vec::new();
        for (o, t) in moves {
            let mut owner = None;
            for node in self.timeline().nodes().take(marker) {
                if !node.suppressed
                    && let Some(c) = node.feature.downcast_ref::<CapturePositionFeature>()
                    && c.get(*o).is_some()
                {
                    owner = Some((node.id, c));
                }
            }
            match owner {
                Some((id, c)) => {
                    let n = nodes.entry(id).or_insert_with(|| c.clone());
                    if let Ok(i) = n.moves.binary_search_by_key(o, |(id, _)| *id) {
                        n.moves[i].1 = *t;
                    }
                }
                None => base.push((*o, *t)),
            }
        }
        for (o, t) in base {
            self.set_occurrence_transform(o, t)?;
        }
        for (id, c) in nodes {
            self.replace_feature(id, crate::feature::FeatureBox::new(c))?;
        }
        let dropped: Vec<OccurrenceId> = moves
            .iter()
            .map(|(o, _)| *o)
            .filter(|o| self.pending.moves.contains_key(o))
            .collect();
        if !dropped.is_empty() {
            for o in dropped {
                self.pending.moves.remove(&o);
            }
            self.bump();
        }
        Ok(())
    }

    /// Drops every pending move and joint state.
    pub fn clear_pending_positions(&mut self) {
        if !self.pending.is_empty() {
            self.pending = PendingPositions::default();
            self.bump();
        }
    }

    /// Drops the pending entries of removed occurrences and joints.
    pub(crate) fn prune_pending(&mut self) {
        if self.pending.is_empty() {
            return;
        }
        let gone: Vec<OccurrenceId> = self
            .pending
            .moves
            .keys()
            .filter(|o| self.occurrence(**o).is_none())
            .copied()
            .collect();
        for o in gone {
            self.pending.moves.remove(&o);
        }
        let gone: Vec<FeatureId> = self
            .pending
            .joint_states
            .keys()
            .filter(|f| !self.timeline().contains(**f))
            .copied()
            .collect();
        for f in gone {
            self.pending.joint_states.remove(&f);
        }
    }

    /// Folds the seeds (captured, then pending) into the base transforms and clears pending:
    /// used when the timeline goes away (history to direct mode) so positions stay as seen.
    pub(crate) fn fold_seeds_into_base(&mut self) -> Result<(), ModelError> {
        let table = self.capture_table(self.timeline().marker());
        let moves: Vec<(OccurrenceId, Transform)> = self
            .occurrences()
            .filter_map(|(o, occ)| {
                self.seed_with(&table, o)
                    .filter(|t| *t != occ.transform)
                    .map(|t| (o, t))
            })
            .collect();
        for (o, t) in moves {
            self.set_occurrence_transform(o, t)?;
        }
        self.clear_pending_positions();
        Ok(())
    }

    /// Checks the pending layer against the rest of the design (load-time invariants).
    pub(crate) fn validate_pending(&self) -> Result<(), String> {
        if self.pending.is_empty() {
            return Ok(());
        }
        if self.meta.mode == crate::model::DesignMode::Direct {
            return Err("pending positions in a direct-mode design".to_owned());
        }
        for (o, t) in self.pending.moves() {
            if self.occurrence(o).is_none() {
                return Err(format!("pending move of missing {o}"));
            }
            if check_transform(&t).is_err() {
                return Err(format!("pending move of {o} is not rigid"));
            }
        }
        for (f, s) in self.pending.joint_states() {
            if !self.timeline().contains(f) {
                return Err(format!("pending joint state of missing {f}"));
            }
            if !s.is_finite() {
                return Err(format!("pending joint state of {f} is not finite"));
            }
        }
        Ok(())
    }
}
